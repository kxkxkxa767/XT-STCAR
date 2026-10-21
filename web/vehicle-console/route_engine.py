"""Route decisions computed by the Rust engine (route-engine/, round 96).

The engine is a line-for-line port of ManeuverSequence. This module gives the
console the same interface: every call goes to one shared engine process over
stdin/stdout JSON lines; the console keeps all of its own hard checks
(owner/heartbeat/gap/body clearance/ACK) and treats the result exactly like the
Python sequence's. Any engine failure (process exit, protocol error, timeout,
internal panic) raises RuntimeError, which the autonomy tick turns into a halt.
"""
import json
import os
import select
import subprocess
import threading
from time import monotonic  # bound now: tools may patch time.monotonic

from maneuver_sequence import DEFAULT_INITIAL_PWM, ManeuverSequence


ENGINE_TIMEOUT_S = .5
# The first reply after a start also covers process start (cold binary cache).
ENGINE_START_TIMEOUT_S = 5.
SCAN_EXTRA_KEYS = ('received_at',)
EXCEPTIONS = {'ValueError': ValueError, 'TypeError': TypeError, 'KeyError': KeyError,
              'IndexError': IndexError, 'ZeroDivisionError': ZeroDivisionError,
              'OverflowError': OverflowError}
_SCALARS = (type(None), bool, int, float, str)


def _engine_error(response):
    name, message = response.get('exc'), response.get('msg', '')
    kind = EXCEPTIONS.get(name)
    if kind is None:
        return RuntimeError(f'route_engine_{name or "error"}: {message}')
    if kind is KeyError and len(message) >= 2 and message[0] == message[-1] == "'":
        return KeyError(message[1:-1])
    return kind(message)


class RouteEngine:
    """One engine process shared by all sessions; requests are serialized."""

    def __init__(self, path):
        self.path = os.fspath(path)
        self._lock = threading.Lock()
        self._proc = None
        self._generation = 0
        self._next_sid = 1
        self._drops = []
        self._buffer = b''
        self._fresh = False

    def _start(self):
        self._proc = subprocess.Popen([self.path], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                      stderr=None, bufsize=0, close_fds=True)
        self._generation += 1
        self._buffer = b''
        self._drops = []
        self._fresh = True

    def _stop(self):
        proc, self._proc = self._proc, None
        if proc is not None:
            try:
                proc.kill()
                proc.wait(timeout=1)
            except (OSError, subprocess.SubprocessError):
                pass
            for pipe in (proc.stdin, proc.stdout):
                try:
                    pipe.close()
                except OSError:
                    pass

    def warm(self):
        """Start the process ahead of the first session (console start)."""
        with self._lock:
            if self._proc is None or self._proc.poll() is not None:
                self._stop()
                self._start()

    def close(self):
        with self._lock:
            self._stop()

    def open_session(self):
        """(generation, sid) of a new session on a running process."""
        with self._lock:
            if self._proc is None or self._proc.poll() is not None:
                self._stop()
                self._start()
            sid, self._next_sid = self._next_sid, self._next_sid+1
            return self._generation, sid

    def drop(self, generation, sid):
        # Called from __del__: no lock, no I/O; sent with the next request.
        if generation == self._generation:
            self._drops.append(sid)

    def _readline(self, deadline):
        fd = self._proc.stdout.fileno()
        while b'\n' not in self._buffer:
            remaining = deadline-monotonic()
            if remaining <= 0 or not select.select([fd], [], [], remaining)[0]:
                raise RuntimeError('route_engine_timeout')
            chunk = os.read(fd, 1 << 16)
            if not chunk:
                raise RuntimeError('route_engine_exited')
            self._buffer += chunk
        line, _, self._buffer = self._buffer.partition(b'\n')
        return line

    def request(self, generation, message):
        with self._lock:
            if self._proc is None or generation != self._generation:
                raise RuntimeError('route_engine_unavailable')
            if self._drops:
                message = {**message, 'drops': self._drops}
                self._drops = []
            try:
                text = json.dumps(message, separators=(',', ':')).encode()+b'\n'
                self._proc.stdin.write(text)
                timeout = ENGINE_START_TIMEOUT_S if self._fresh else ENGINE_TIMEOUT_S
                response = json.loads(self._readline(monotonic()+timeout))
                self._fresh = False
            except (OSError, ValueError, RuntimeError) as error:
                self._stop()
                raise RuntimeError(f'route_engine_failed: {error}') from None
            if not isinstance(response, dict) or response.get('exc') == 'ProtocolError':
                self._stop()
                raise RuntimeError('route_engine_protocol_error')
            return response


class RustManeuverSequence:
    """ManeuverSequence's console interface, decided by the Rust engine.

    The attributes the console reads after a call come back with every
    response (only changes); the decision is rebuilt from per-call deltas.
    """
    _forward_lanes = staticmethod(ManeuverSequence._forward_lanes)

    def __init__(self, engine, started_at, max_drive_s=10.0, initial_presteer_pwm=DEFAULT_INITIAL_PWM,
                 continue_route=False):
        state = self.__dict__
        state.update(_engine=engine, _mirror={}, _decision={}, _scan_base=None, _sid=None)
        generation, sid = engine.open_session()
        state['_generation'] = generation
        state['_sid'] = sid
        self._call_op({'op': 'new', 'started_at': started_at, 'max_drive_s': max_drive_s,
                       'initial_presteer_pwm': initial_presteer_pwm, 'continue_route': continue_route})

    def __del__(self):
        engine, sid = self.__dict__.get('_engine'), self.__dict__.get('_sid')
        if engine is not None and sid is not None:
            engine.drop(self.__dict__['_generation'], sid)

    def __getattr__(self, name):
        mirror = self.__dict__.get('_mirror')
        if mirror is not None and name in mirror:
            return mirror[name]
        if name.startswith('__') or '_engine' not in self.__dict__:
            raise AttributeError(name)
        values = self._call_op({'op': 'get', 'attrs': [name]})
        if name not in values:
            raise AttributeError(name)
        return values[name]

    def __setattr__(self, name, value):
        raise AttributeError(f'route_engine_attribute_read_only: {name}')

    def _call_op(self, message):
        message['sid'] = self._sid
        response = self._engine.request(self._generation, message)
        mirror = response.get('s')
        if isinstance(mirror, dict):
            self._mirror.update(mirror)
        if not response.get('ok'):
            raise _engine_error(response)
        if 'd' in response:
            decision = self._decision
            for key in response.get('x', ()):
                decision.pop(key, None)
            decision.update(response['d'])
        return response.get('r')

    def _scan(self, scan):
        """The scan once per content: later identical bases are referenced."""
        if not isinstance(scan, dict):
            return scan
        extra = {key: scan[key] for key in SCAN_EXTRA_KEYS if key in scan}
        base = self._scan_base
        if (base is not None and len(base) == len(scan)-len(extra)
                and all(key in scan and (scan[key] is value or type(value) in _SCALARS
                                         and type(scan[key]) is type(value) and scan[key] == value)
                        for key, value in base.items())):
            return {'$same': 1, '$with': extra}
        base = {key: value for key, value in scan.items() if key not in extra}
        self.__dict__['_scan_base'] = base
        return {'$base': base, '$with': extra}

    def _call(self, method, **arguments):
        return self._call_op({'op': 'call', 'm': method, 'a': arguments})

    def update(self, scan, lidar_age_s, now, control, *, safe=True, entry_stop=None, presteer_wait=False,
               quality_clear=False, quality_resume_ready=False):
        self._call('update', scan=self._scan(scan), lidar_age_s=lidar_age_s, now=now, control=control,
                   safe=safe, entry_stop=entry_stop, presteer_wait=presteer_wait,
                   quality_clear=quality_clear, quality_resume_ready=quality_resume_ready)
        return dict(self._decision)

    def begin_coast(self, reason, now):
        self._call('begin_coast', reason=reason, now=now)

    def begin_quality_wait(self, now, control, *, scan=None):
        return self._call('begin_quality_wait', now=now, control=control, scan=self._scan(scan))

    def begin_left_turn_quality_pause(self, now, control, *, scan=None):
        return self._call('begin_left_turn_quality_pause', now=now, control=control, scan=self._scan(scan))

    def begin_post_pass_quality_pause(self, now, control, *, scan=None):
        return self._call('begin_post_pass_quality_pause', now=now, control=control, scan=self._scan(scan))

    def begin_route_quality_coast(self, now, control):
        return self._call('begin_route_quality_coast', now=now, control=control)

    def begin_quality_coast(self, now, control):
        return self._call('begin_quality_coast', now=now, control=control)

    def current_contact_target(self, scan, now):
        return self._call('current_contact_target', scan=self._scan(scan), now=now)

    def start_lane_quality_geometry(self, scan):
        return self._call('start_lane_quality_geometry', scan=self._scan(scan))

    def lane_wall_output_active(self):
        # A pure predicate of the state, mirrored with every response.
        return self._mirror['lane_wall_output_active()']

    def probe_measurement(self, scan, candidates, dt):
        """server.turn_route_quality_scope's isolated _measurement check."""
        return self._call('probe_measurement', scan=self._scan(scan), candidates=candidates, dt=dt)

    def dump(self):
        return self._call_op({'op': 'dump'})
