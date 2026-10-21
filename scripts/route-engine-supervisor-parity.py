#!/usr/bin/env python3
"""Run a tool with the console's route supervision doubled by the Rust port.

  python3 scripts/route-engine-supervisor-parity.py [--engine PATH] [--every N] [--dump FILE] -- \
      scripts/course-sim.py --variant NAME
  python3 scripts/route-engine-supervisor-parity.py -- scripts/route-replay.py TRIAL_DIR
  python3 scripts/route-engine-supervisor-parity.py --keep-going -- scripts/run-console-tests.py MODULE ...

Every server.Console the tool creates (course-sim, the unit-test harness that
route-replay uses) gets a Rust supervisor (route-engine/src/supervisor.rs) in
the engine process. A route start (turn_cone_start with continue_route) and
every tick of a route session run in Python (the tool keeps using its
result) and then in Rust on the same inputs taken before the Python call:
scan, bridge status, clocks, sensor ages, owner lease, sequence. The coast
worker's run_pending is doubled after each tick. Compared exactly (types and
float bits): the outcome and refusal (with its clearance), the commands sent
(op, seq, tick, motor, nominal servo, and the physical servo where Python
calibrated it), the console fields (owner, mode, epoch, arm and command
sequences, stopped tick, stop latch and reason), the session fields, the
whole report (every N calls with --every) and the final result. Wall-clock
fields (coast processing_ms) are not compared. The Python motion is the
Python ManeuverSequence and the Rust one the engine's port (equal by
route-engine-parity.py), so this checks the supervision port. The first
difference is printed and the run exits with status 3; with --keep-going it
is raised as an AssertionError in the call instead (a unit test then fails)
and that console is no longer compared. Offline.
"""
import argparse
import copy
import importlib.machinery
import importlib.util
import json
import os
from pathlib import Path
import runpy
import sys

ROOT = Path(__file__).resolve().parents[1]
CONSOLE = ROOT/'web'/'vehicle-console'
SERVER = (CONSOLE/'server.py').resolve()
sys.path.insert(0, str(CONSOLE))
import route_engine  # noqa: E402

spec = importlib.util.spec_from_file_location('route_engine_parity', ROOT/'scripts'/'route-engine-parity.py')
parity = importlib.util.module_from_spec(spec)
spec.loader.exec_module(parity)
SKIP = {'processing_ms'}
SESSION_KEYS = ('phase', 'deadline', 'last_loop', 'heartbeat_seq', 'motion_scan_seq', 'turn_last_servo',
                'turn_servo_sequence', 'turn_servo_sent_at', 'coast_since', 'coast_deadline', 'coast_reason',
                'motion_generation', 'motion_ready_for_coast', 'coast_start_seq', 'coast_neutral_sequence',
                'coast_endpoint_verified', 'quality_wait_neutral_sequence', 'quality_resume_sequence',
                'quality_wait_kind', 'quality_wait_entry_neutral', 'quality_wait_front_sparse_entry',
                'route_quality_coast')


class State:
    engine = None
    keep_going = False
    every = 1
    calls = 0
    methods = {}
    dump = None
    last = None


def normalize(value):
    if isinstance(value, dict):
        return {k if type(k) is str else str(k): normalize(v) for k, v in value.items() if k not in SKIP}
    if isinstance(value, (list, tuple)):
        return [normalize(v) for v in value]
    return parity.normalize(value)


class Mismatch(AssertionError):
    pass


def fail(where, found):
    path, py, rust = found
    if State.keep_going:
        raise Mismatch(f'SUPERVISOR MISMATCH ({where}) at {path}: python {repr(py)[:300]} rust {repr(rust)[:300]}')
    if State.dump and State.last is not None:
        Path(State.dump).write_text(json.dumps({'call': State.calls, 'where': where, 'path': path,
                                                'arguments': State.last}, default=repr))
    sys.stdout.flush()
    print(f'\nSUPERVISOR MISMATCH after call {State.calls} ({where}) at {path}\n  python: {repr(py)[:700]}\n'
          f'  rust:   {repr(rust)[:700]}', file=sys.stderr, flush=True)
    os._exit(3)


def check(where, a, b, path):
    found = parity.difference(a, b, path)
    if found:
        fail(where, found)


class Core:
    """One Rust supervisor beside one Python console."""

    def __init__(self, console):
        self.console = console
        self.generation, self.sid = State.engine.open_session()
        self.scan_base = None
        self.report = {}
        response = State.engine.request(self.generation, {
            'op': 'sup_new', 'sid': self.sid, 'demo': bool(console.args.demo), 'boot': console.boot,
            'route_controller': console.route_controller(), 'deterministic': True})
        assert response.get('ok'), response

    def scan_arg(self, scan):
        if not isinstance(scan, dict):
            return scan
        base = self.scan_base
        if base is not None and base.keys() == scan.keys() and all(scan[k] is v for k, v in base.items()):
            return {'$same': 1, '$with': {}}
        self.scan_base = dict(scan)
        return {'$base': scan, '$with': {}}

    def inputs(self, now, module):
        c = self.console
        return {'scan': self.scan_arg(c.scan), 'scan_at': c.scan_at, 'status': copy.deepcopy(c.status),
                'control_at': c.control_at, 'owner_at': c.owner_at, 'errors': dict(c.errors),
                'ages': c.sensor_ages(now), 'mono': module.time.monotonic(), 'sequence': c.sequence}

    def call(self, method, inputs, arguments):
        response = State.engine.request(self.generation, {'op': 'sup_call', 'sid': self.sid, 'm': method,
                                                          'in': inputs, 'a': arguments})
        if response.get('exc') == 'ProtocolError':
            raise RuntimeError(f'engine protocol error: {response}')
        dump = response.get('dump') or {}
        if dump.get('session') is None:
            self.report = {}
        else:
            for key in dump.get('report_removed', ()):
                self.report.pop(key, None)
            self.report.update(dump.get('report_delta') or {})
        return response, dump


def capture(console):
    """Record the console's commands and calibrated servos during one call."""
    rows, physical = [], {}
    saved = {name: console.__dict__[name] for name in ('emit', 'output_servo') if name in console.__dict__}
    inner_emit, inner_servo = console.emit, console.output_servo

    def output_servo(op, servo, sequence):
        value = inner_servo(op, servo, sequence)
        if op == 'drive':
            physical[sequence] = value
        return value

    def emit(op, motor=1500, servo=1500, tick=0):
        inner_emit(op, motor, servo, tick)
        rows.append({'op': op, 'seq': console.sequence, 'tick': tick, 'motor': motor, 'servo': servo})

    console.output_servo, console.emit = output_servo, emit

    def restore():
        for name in ('emit', 'output_servo'):
            console.__dict__.pop(name, None)
        console.__dict__.update(saved)
    return rows, physical, restore


def python_outcome(call):
    try:
        return 'ok', call()
    except Exception as error:  # noqa: BLE001 - compared, then re-raised
        return 'error', error


def describe(error):
    return [type(error).__name__ if not isinstance(error, ValueError) else 'ValueError',
            str(error) if not isinstance(error, TypeError) else '', normalize(getattr(error, 'clearance', None))]


def compare(core, where, py_out, response, dump, rows, physical):
    State.calls += 1
    State.methods[where] = State.methods.get(where, 0)+1
    c = core.console
    if py_out[0] == 'error' or not response.get('ok'):
        rust_error = ([response.get('exc'), '' if response.get('exc') == 'TypeError' else response.get('msg'),
                       normalize(response.get('clearance'))] if not response.get('ok') else 'ok')
        check(where, describe(py_out[1]) if py_out[0] == 'error' else 'ok', rust_error, '$.outcome')
    elif where == 'start':
        check(where, normalize(py_out[1]), normalize(response.get('r')), '$.result')
    rust_rows = response.get('emits') or []
    check(where, normalize(rows), normalize([{k: e[k] for k in ('op', 'seq', 'tick', 'motor', 'servo')}
                                             for e in rust_rows]), '$.emits')
    for e in rust_rows:
        if e['seq'] in physical:
            check(where, normalize(physical[e['seq']]), normalize(e['physical']), f'$.physical[{e["seq"]}]')
    console = dump.get('console') or {}
    expected = {'owner': c.owner, 'control_mode': c.control_mode, 'control_epoch': c.control_epoch,
                'arm_sequence': c.arm_sequence, 'sequence': c.sequence, 'stopped_tick': c.stopped_tick,
                'stop_latched': c.stop_latched, 'last_stop_reason': (c.last_stop or {}).get('reason')}
    check(where, normalize(expected), normalize({k: console.get(k) for k in expected}), '$.console')
    session = c.auto_session
    if session is None:
        check(where, None, dump.get('session'), '$.session')
        if 'auto_result' in dump:
            check(where, normalize(c.auto_result), normalize(dump['auto_result']), '$.auto_result')
        return
    if dump.get('session') is None:
        fail(where, ('$.session', 'active', None))
    fields = {key: session.get(key) for key in SESSION_KEYS}
    fields['clearance_reused'] = session['clearance_reuse'].reused if 'clearance_reuse' in session else 0
    check(where, normalize(fields), normalize({k: dump['session'].get(k) for k in fields}), '$.session')
    if State.calls % State.every == 0:
        check(where, normalize(session['report']), normalize(core.report), '$.report')


def wrap_console(module):
    Console = module.Console
    if getattr(Console, '_supervisor_parity', False):
        return
    Console._supervisor_parity = True
    original_command, original_tick, original_halt = Console.autonomy_command, Console.autonomy_tick, Console.halt

    def diverged(console):
        return console.__dict__.get('_rust_diverged', False)

    def compared(console, call):
        try:
            call()
        except Mismatch:
            console.__dict__['_rust_diverged'] = True
            raise

    def core_of(console):
        core = console.__dict__.get('_rust_core')
        if core is None:
            core = console.__dict__['_rust_core'] = Core(console)
        return core

    def wrap_worker(console, core):
        worker = console.coast_worker
        if worker is None or getattr(worker, '_doubled', False):
            return
        inner = worker.run_pending

        def run_pending():
            inner()
            core.call('run_pending', {}, {})
        worker.run_pending = run_pending
        worker._doubled = True

    def route_session(console):
        session = console.auto_session
        return session is not None and session.get('continue_route') is True and session.get('maneuver_sequence') is True

    def autonomy_command(self, data):
        if data.get('op') != 'turn_cone_start' or data.get('continue_route') is not True or diverged(self):
            return original_command(self, data)
        core = core_of(self)
        wrap_worker(self, core)
        now = module.time.monotonic()
        inputs = core.inputs(now, module)
        rows, physical, restore = capture(self)
        self.__dict__['_in_parity_call'] = True
        try:
            py_out = python_outcome(lambda: original_command(self, data))
        finally:
            self.__dict__['_in_parity_call'] = False
            restore()
        run_id = py_out[1]['run_id'] if py_out[0] == 'ok' else 'refused'
        State.last = ['start', data, now]
        response, dump = core.call('start', inputs, {'data': data, 'now': now, 'run_id': run_id})
        compared(self, lambda: compare(core, 'start', py_out, response, dump, rows, physical))
        if py_out[0] == 'error':
            raise py_out[1]
        return py_out[1]

    def autonomy_tick(self, now):
        if not route_session(self) or diverged(self) or '_rust_core' not in self.__dict__:
            return original_tick(self, now)
        core = core_of(self)
        wrap_worker(self, core)
        inputs = core.inputs(now, module)
        rows, physical, restore = capture(self)
        self.__dict__['_in_parity_call'] = True
        try:
            py_out = python_outcome(lambda: original_tick(self, now))
        finally:
            self.__dict__['_in_parity_call'] = False
            restore()
        State.last = ['tick', now]
        response, dump = core.call('tick', inputs, {'now': now})
        compared(self, lambda: compare(core, 'tick', py_out, response, dump, rows, physical))
        if py_out[0] == 'error':
            raise py_out[1]
        return py_out[1]

    def halt(self, reason):
        # A halt from outside a doubled call (tests, bridge feedback) ends both.
        doubled = ('_rust_core' in self.__dict__ and not self.__dict__.get('_in_parity_call')
                   and not diverged(self))
        if not doubled:
            return original_halt(self, reason)
        core = self.__dict__['_rust_core']
        inputs = core.inputs(module.time.monotonic(), module)
        rows, physical, restore = capture(self)
        try:
            py_out = python_outcome(lambda: original_halt(self, reason))
        finally:
            restore()
        response, dump = core.call('halt', inputs, {'reason': reason})
        compared(self, lambda: compare(core, 'halt', py_out, response, dump, rows, physical))

    Console.autonomy_command, Console.autonomy_tick, Console.halt = autonomy_command, autonomy_tick, halt


def install(engine_path, every, keep_going=False):
    State.engine = route_engine.RouteEngine(engine_path)
    State.every = every
    State.keep_going = keep_going
    loader = importlib.machinery.SourceFileLoader
    original = loader.exec_module

    def exec_module(self, module):
        original(self, module)
        path = getattr(module, '__file__', None)
        if path and Path(path).resolve() == SERVER:
            wrap_console(module)
    loader.exec_module = exec_module


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--engine', default=str(parity.DEFAULT_ENGINE))
    parser.add_argument('--every', type=int, default=1, help='compare the whole report every N calls')
    parser.add_argument('--dump', help="write the mismatching call's arguments here (JSON)")
    parser.add_argument('--keep-going', action='store_true', help='raise mismatches in the call (unit tests)')
    parser.add_argument('script')
    parser.add_argument('args', nargs=argparse.REMAINDER)
    options = parser.parse_args()
    install(options.engine, max(1, options.every), options.keep_going)
    State.dump = options.dump
    sys.argv = [options.script, *options.args]
    try:
        runpy.run_path(options.script, run_name='__main__')
    finally:
        counts = ', '.join(f'{name} {count}' for name, count in sorted(State.methods.items()))
        print(f'supervisor parity: {State.calls} calls compared ({counts})', file=sys.stderr)
        State.engine.close()


if __name__ == '__main__':
    main()
