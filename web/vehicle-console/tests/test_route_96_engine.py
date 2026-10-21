"""Round 96: the Rust route engine behind the console (route_engine.py).

Equality with the Python sequence over whole routes is checked by
scripts/route-engine-parity(-sweep).py; these tests cover the proxy's own
paths: Python exception types, read-only mirrored attributes, dropped
sessions, engine exit/hang -> RuntimeError (the tick halts), and the console
selecting the engine. They need the Mac build of route-engine/ and are skipped
without it. Synthetic scans and clocks; no physical claims.
"""
import gc
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).parents[1]))
import maneuver_sequence as ms
import route_engine
import test_turn_control as control

ENGINE = Path(__file__).resolve().parents[3]/'route-engine'/'target'/'release'/'route-engine'


@unittest.skipUnless(ENGINE.is_file(), 'route-engine not built (cargo build --release in route-engine/)')
class RouteEngineProxyTests(unittest.TestCase):
    def setUp(self):
        self.engine = route_engine.RouteEngine(ENGINE)

    def tearDown(self):
        self.engine.close()

    def test_construction_errors_keep_python_types_and_messages(self):
        for kwargs in ({'continue_route': 'yes'}, {'initial_presteer_pwm': 1500}, {'initial_presteer_pwm': 1671.5}):
            with self.subTest(kwargs=kwargs):
                with self.assertRaises(ValueError) as python:
                    ms.ManeuverSequence(0., **kwargs)
                with self.assertRaises(ValueError) as rust:
                    route_engine.RustManeuverSequence(self.engine, 0., **kwargs)
                self.assertEqual(str(rust.exception), str(python.exception))

    def test_mirrored_attributes_are_read_only(self):
        python = ms.ManeuverSequence(0., max_drive_s=16., continue_route=True)
        rust = route_engine.RustManeuverSequence(self.engine, 0., max_drive_s=16., continue_route=True)
        for name in ('phase', 'route_stage', 'continue_route', 'max_drive_s', 'max_presteer_s', 'drive_since',
                     'handover_observed', 'quality_coast_servo', 'entry_kind'):
            self.assertEqual(getattr(rust, name), getattr(python, name), name)
        self.assertIs(rust.lane_wall_output_active(), python.lane_wall_output_active())
        self.assertEqual(rust.initial_presteer_pwm, python.initial_presteer_pwm)  # not mirrored: one get
        with self.assertRaises(AttributeError):
            rust.phase = 'drive'
        with self.assertRaises(AttributeError):
            rust.no_such_attribute

    def test_dropped_sessions_are_released_with_the_next_request(self):
        first = route_engine.RustManeuverSequence(self.engine, 0., continue_route=True)
        generation, sid = first._generation, first._sid
        self.assertEqual(self.engine.request(generation, {'op': 'get', 'sid': sid, 'attrs': ['phase']})['r'],
                         {'phase': 'presteer'})
        del first
        gc.collect()
        route_engine.RustManeuverSequence(self.engine, 0., continue_route=True)
        response = self.engine.request(generation, {'op': 'get', 'sid': sid, 'attrs': ['phase']})
        self.assertEqual((response['ok'], response['exc'], response['msg']),
                         (False, 'ValueError', 'route_engine_no_session'))

    def test_engine_exit_is_a_runtime_error_and_a_new_session_restarts_it(self):
        motion = route_engine.RustManeuverSequence(self.engine, 0., continue_route=True)
        self.engine._proc.kill()
        self.engine._proc.wait()
        with self.assertRaisesRegex(RuntimeError, '^route_engine_failed'):
            motion.update(None, .01, .02, {})
        with self.assertRaisesRegex(RuntimeError, '^route_engine_unavailable$'):
            motion.update(None, .01, .04, {})
        fresh = route_engine.RustManeuverSequence(self.engine, 0., continue_route=True)
        self.assertEqual(fresh.phase, 'presteer')
        with self.assertRaisesRegex(RuntimeError, '^route_engine_unavailable$'):
            motion.begin_coast('x', .06)  # an old session never reaches the new process

    def test_hung_engine_times_out(self):
        with tempfile.TemporaryDirectory() as temp:
            fake = Path(temp)/'hung-engine'
            fake.write_text('#!/bin/sh\nexec sleep 30\n')
            os.chmod(fake, 0o700)
            engine = route_engine.RouteEngine(fake)
            try:
                with patch.object(route_engine, 'ENGINE_START_TIMEOUT_S', .3), \
                        self.assertRaisesRegex(RuntimeError, 'route_engine_timeout'):
                    route_engine.RustManeuverSequence(engine, 0., continue_route=True)
                self.assertIsNone(engine._proc)  # killed, not left running
            finally:
                engine.close()


@unittest.skipUnless(ENGINE.is_file(), 'route-engine not built (cargo build --release in route-engine/)')
class ConsoleRouteEngineTests(unittest.TestCase):
    """The console's own selection path: turn_preview -> RustManeuverSequence."""

    def run_console(self, engine, kill_at=None):
        h = control.TurnControlTests('test_compact_trial_preview_default_1670_and_placement_admission_keep_neutral')
        h.setUp()
        try:
            h.console.route_engine = engine
            with patch.object(h.server.time, 'monotonic', return_value=0.):
                status = h.console.state(trial_mode='turn-cone', continue_route=True)['autonomy']
            h.drive_compact_trial(continue_route=True, max_drive_s=16.)
            for index in range(1, 50):
                if h.console.auto_session is None:
                    break
                if kill_at is not None and index == kill_at:
                    engine._proc.kill()
                    engine._proc.wait()
                h.tick(round(h.now+.02, 6))
            report = (h.console.auto_session or {}).get('report') or h.console.auto_result
            return status, list(h.outputs), h.console.auto_result, report
        finally:
            h.tearDown()

    def test_engine_console_matches_python_console(self):
        engine = route_engine.RouteEngine(ENGINE)
        try:
            rust_status, rust_outputs, rust_result, rust_report = self.run_console(engine)
        finally:
            engine.close()
        python_status, python_outputs, python_result, python_report = self.run_console(None)
        self.assertEqual((rust_status['route_controller'], python_status['route_controller']), ('rust_engine', 'python'))
        self.assertEqual(rust_status['turn_preview'], python_status['turn_preview'])
        self.assertEqual(rust_report['route_controller'], 'rust_engine')
        self.assertEqual(rust_outputs, python_outputs)
        self.assertEqual(rust_report['turn'], python_report['turn'])
        self.assertEqual((rust_result or {}).get('reason'), (python_result or {}).get('reason'))

    def test_engine_exit_mid_run_halts_the_run(self):
        engine = route_engine.RouteEngine(ENGINE)
        try:
            _, outputs, result, _ = self.run_console(engine, kill_at=3)
        finally:
            engine.close()
        self.assertIsNotNone(result)
        self.assertRegex(result['reason'], '^route_engine_failed')
        self.assertEqual(outputs[-1]['op'], 'stop')


if __name__ == '__main__':
    unittest.main()
