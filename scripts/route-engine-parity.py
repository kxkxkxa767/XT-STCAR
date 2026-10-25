#!/usr/bin/env python3
"""Run a tool with every ManeuverSequence doubled by the Rust route engine.

  python3 scripts/route-engine-parity.py [--engine PATH] [--every N] -- \
      scripts/course-sim.py --variant NAME
  python3 scripts/route-engine-parity.py -- scripts/route-replay.py TRIAL_DIR

Each console call goes to both the Python sequence (whose result the tool
keeps using) and the engine (route_engine.RustManeuverSequence); since round
97 that includes the console's body-clearance check (clearance_probe: the
contact target plus ClearanceReuse(probe_clearance), compared with its reuse
count and, for a body stop, the attached clearance). The results must match
exactly; values are compared by type and bits, so 1 and 1.0 differ
and so do 0.0 and -0.0. After a call (every N calls with --every) the whole
Python state (__dict__, nested objects as their __dict__) must equal the
engine's dump. The first difference is printed with its path and the run
exits with status 3; afterwards the two would only diverge further.
"""
import argparse
import copy
import math
import os
from pathlib import Path
import runpy
import sys

ROOT = Path(__file__).resolve().parents[1]
CONSOLE = ROOT/'web'/'vehicle-console'
DEFAULT_ENGINE = ROOT/'route-engine'/'target'/'release'/'route-engine'
# Not compared: pure same-frame caches; the input scan reference; the
# duplicate-scan signature (Python freezes it to tuples, the engine keeps the
# values; both compare by content).
SKIP = {'_snapshots', '_proximity_cache', '_axis_cache', '_current_scan', 'last_signature'}
# Python sets these only when first needed (getattr default None).
LATE = ('_task_left_upper', '_second_lane_heading', '_second_lane_previous', '_second_lane_rate')


def normalize(value):
    if value is None or type(value) in (bool, int, float, str):
        return value
    if isinstance(value, (list, tuple)):
        return [normalize(item) for item in value]
    if isinstance(value, dict):
        if any(type(key) is float for key in value):
            # The engine keeps float-keyed maps as [key, value] pairs.
            return [[key, normalize(item)] for key, item in value.items()]
        return {key if type(key) is str else str(key): normalize(item) for key, item in value.items()}
    if isinstance(value, (set, frozenset)):
        return sorted((normalize(item) for item in value), key=repr)
    if hasattr(value, '__dict__'):
        return {key: normalize(item) for key, item in vars(value).items() if key not in SKIP}
    return f'<{type(value).__name__}>'


def difference(a, b, path='$'):
    """Path and both values of the first difference, or None."""
    if type(a) is not type(b):
        return path, a, b
    if type(a) is float:
        if a != a and b != b:
            return None
        if a != b or math.copysign(1, a) != math.copysign(1, b):
            return path, a, b
        return None
    if type(a) is list:
        if len(a) != len(b):
            return f'{path}.len', len(a), len(b)
        for i, (x, y) in enumerate(zip(a, b)):
            found = difference(x, y, f'{path}[{i}]')
            if found:
                return found
        return None
    if type(a) is dict:
        if set(a)-SKIP != set(b)-SKIP:
            return f'{path}.keys', sorted(set(a)-set(b)-SKIP), sorted(set(b)-set(a)-SKIP)
        for key in a.keys()-SKIP:
            found = difference(a[key], b[key], f'{path}.{key}')
            if found:
                return found
        return None
    return None if a == b else (path, a, b)


def outcome(call):
    try:
        return 'ok', call()
    except Exception as error:  # noqa: BLE001 - compared, then re-raised
        return 'error', error


def describe_error(error):
    message = str(error) if not isinstance(error, TypeError) else ''
    return [type(error).__name__, message, normalize(getattr(error, 'clearance', None))]


class Parity:
    every = 1
    calls = 0
    methods = {}
    engine = None
    dump = None
    last_call = None


def fail(where, found):
    path, py, rust = found
    if Parity.dump and Parity.last_call is not None:
        import json
        with open(Parity.dump, 'w') as handle:
            json.dump({'call': Parity.calls, 'where': where, 'path': path, 'arguments': Parity.last_call},
                      handle, default=repr)
    text = lambda value: repr(value)[:600]  # noqa: E731
    sys.stdout.flush()
    print(f'\nPARITY MISMATCH after call {Parity.calls} ({where}) at {path}\n  python: {text(py)}\n  rust:   {text(rust)}',
          file=sys.stderr, flush=True)
    os._exit(3)


def install(engine_path, every):
    sys.path.insert(0, str(CONSOLE))
    import autonomy_live
    import maneuver_sequence
    import route_engine
    original = maneuver_sequence.ManeuverSequence
    Parity.engine = route_engine.RouteEngine(engine_path)
    Parity.every = every

    class DualManeuverSequence:
        _forward_lanes = staticmethod(original._forward_lanes)

        def __init__(self, *args, **kwargs):
            state = self.__dict__
            rust_out = outcome(lambda: route_engine.RustManeuverSequence(Parity.engine, *args, **kwargs))
            py_out = outcome(lambda: original(*args, **kwargs))
            self._compare('__init__', py_out, rust_out, result=False)
            if py_out[0] == 'error':
                raise py_out[1]
            state['_py'], state['_rust'] = py_out[1], rust_out[1]
            state['_py_reuse'] = autonomy_live.ClearanceReuse()
            self._check_state('__init__')

        def __getattr__(self, name):
            if name.startswith('__') or '_py' not in self.__dict__:
                raise AttributeError(name)
            return getattr(self.__dict__['_py'], name)

        def _compare(self, where, py_out, rust_out, result=True):
            Parity.calls += 1
            Parity.methods[where] = Parity.methods.get(where, 0)+1
            if py_out[0] != rust_out[0]:
                fail(where, ('$.outcome', py_out[0] == 'error' and describe_error(py_out[1]) or 'ok',
                             rust_out[0] == 'error' and describe_error(rust_out[1]) or 'ok'))
            if py_out[0] == 'error':
                found = difference(describe_error(py_out[1]), describe_error(rust_out[1]), '$.error')
            else:
                found = difference(normalize(py_out[1]), normalize(rust_out[1]), '$.result') if result else None
            if found:
                fail(where, found)

        def _check_state(self, where):
            if Parity.calls % Parity.every:
                return
            state = normalize(self._py)
            for key in LATE:
                state.setdefault(key, None)
            found = difference(state, self._rust.dump(), '$.state')
            if found:
                fail(where, found)

        def _both(self, name, *args, **kwargs):
            Parity.last_call = [name, list(args), kwargs]
            # The engine call only serializes its arguments, so it runs first
            # and the Python call still sees them exactly as passed.
            rust_out = outcome(lambda: getattr(self._rust, name)(*args, **kwargs))
            py_out = outcome(lambda: getattr(self._py, name)(*args, **kwargs))
            self._compare(name, py_out, rust_out)
            self._check_state(name)
            if py_out[0] == 'error':
                raise py_out[1]
            return py_out[1]

        def update(self, *args, **kwargs):
            return self._both('update', *args, **kwargs)

        def begin_coast(self, *args, **kwargs):
            return self._both('begin_coast', *args, **kwargs)

        def begin_quality_wait(self, *args, **kwargs):
            return self._both('begin_quality_wait', *args, **kwargs)

        def begin_left_turn_quality_pause(self, *args, **kwargs):
            return self._both('begin_left_turn_quality_pause', *args, **kwargs)

        def begin_post_pass_quality_pause(self, *args, **kwargs):
            return self._both('begin_post_pass_quality_pause', *args, **kwargs)

        def begin_route_quality_coast(self, *args, **kwargs):
            return self._both('begin_route_quality_coast', *args, **kwargs)

        def begin_quality_coast(self, *args, **kwargs):
            return self._both('begin_quality_coast', *args, **kwargs)

        def current_contact_target(self, *args, **kwargs):
            return self._both('current_contact_target', *args, **kwargs)

        def start_lane_quality_geometry(self, *args, **kwargs):
            return self._both('start_lane_quality_geometry', *args, **kwargs)

        def lane_wall_output_active(self):
            return self._both('lane_wall_output_active')

        def first_loop_output_active(self):
            return self._both('first_loop_output_active')

        def clearance_probe(self, scan, now, contact, ages, demo=False, centering=False, **kwargs):
            # server.autonomy_tick's Python path for the same arguments.
            def python_probe():
                target = self._py.current_contact_target(scan, now) if contact else None
                reuse = self.__dict__['_py_reuse']
                clearance = reuse.probe(autonomy_live.probe_clearance, scan, ages, demo, centering,
                                        body_contact_target=target, **kwargs)
                return target, clearance, reuse.reused
            Parity.last_call = ['clearance_probe', [scan, now, contact, ages, demo, centering], kwargs]
            rust_out = outcome(lambda: self._rust.clearance_probe(scan, now, contact, ages, demo, centering, **kwargs))
            py_out = outcome(python_probe)
            self._compare('clearance_probe', py_out, rust_out)
            self._check_state('clearance_probe')
            if py_out[0] == 'error':
                raise py_out[1]
            return py_out[1]

        def probe_measurement(self, scan, candidates, dt):
            def python_probe():
                probe = copy.copy(self._py)
                for field in ('corridor', 'outer_wall', 'turn_goal', 'exit_width_change'):
                    setattr(probe, field, copy.deepcopy(getattr(self._py, field)))
                probe._measurement(scan, candidates, dt)
                return True
            rust_out = outcome(lambda: self._rust.probe_measurement(scan, candidates, dt))
            py_out = outcome(python_probe)
            self._compare('probe_measurement', py_out, rust_out)
            self._check_state('probe_measurement')
            if py_out[0] == 'error':
                raise py_out[1]
            return py_out[1]

    maneuver_sequence.ManeuverSequence = DualManeuverSequence
    return DualManeuverSequence


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--engine', default=str(DEFAULT_ENGINE))
    parser.add_argument('--every', type=int, default=1, help='compare the whole state every N calls')
    parser.add_argument('--dump', help='write the mismatching call\'s arguments here (JSON)')
    parser.add_argument('script')
    parser.add_argument('args', nargs=argparse.REMAINDER)
    options = parser.parse_args()
    install(options.engine, max(1, options.every))
    Parity.dump = options.dump
    sys.argv = [options.script, *options.args]
    try:
        runpy.run_path(options.script, run_name='__main__')
    finally:
        counts = ', '.join(f'{name} {count}' for name, count in sorted(Parity.methods.items()))
        print(f'parity: {Parity.calls} calls compared ({counts})', file=sys.stderr)
        Parity.engine.close()


if __name__ == '__main__':
    main()
