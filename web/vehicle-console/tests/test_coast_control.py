"""Actual console output/state-machine checks with offline scans and an injected clock."""
import importlib.util
from pathlib import Path
import sys
import threading
import unittest
from unittest.mock import patch
import test_autonomy_live as live_tests
from test_coast_motion import MODULE as motion_module, room_scan, parallel_scan


class CoastControlTests(unittest.TestCase):
    def setUp(self):
        live_tests.QualityRecoveryTests.setUp(self)
        self.console.coast_worker = self.server.CoastMotionWorker(autostart=False)
        self.received_identity = None

    def tearDown(self):
        self.console.coast_worker.close()
        live_tests.QualityRecoveryTests.tearDown(self)

    def tick(self, now):
        identity = self.console.scan.get('seq'), self.console.scan.get('at_ms')
        if identity != self.received_identity:
            self.console.scan_at = now
            self.received_identity = identity
        live_tests.QualityRecoveryTests.tick(self, now)
        if self.console.coast_worker._thread is None:
            self.console.coast_worker.run_pending()

    def prepare(self, reason='probe_complete'):
        session = self.console.auto_session
        session['report']['centering'] = True
        session['steering'] = self.server.CorridorSteering()
        session['wall_scan_seq'] = None
        session['ramp'] = self.server.ApproachRamp(1560)
        session['junction'] = self.server.JunctionStop()
        self.console.scan = room_scan(1, y=-.12, half_width=.5)
        self.tick(0)
        self.assertEqual(self.output[-1], ('drive', 1560, 1480))
        self.console.begin_coast(reason, .02)
        self.tick(.02)
        return session

    def advance(self, start=.04, end=.82, scan_factory=None):
        for i in range(round((end-start)/.02)+1):
            now = round(start+i*.02, 6)
            seq = 1+int(round(now*10, 6))
            if scan_factory:
                self.console.scan = scan_factory(seq)
            self.tick(now)

    def test_normal_stop_keeps_neutral_motor_and_steering_until_lidar_still(self):
        session = self.prepare()
        self.assertEqual(self.output[-1], ('drive', 1500, 1480))
        self.advance(scan_factory=lambda seq: room_scan(seq, y=-.12, half_width=.5))
        self.assertIsNone(self.console.auto_session)
        self.assertTrue(self.console.auto_result['standstill_confirmed'])
        self.assertTrue(self.console.auto_result['completed'])
        self.assertEqual(self.output[-1], ('stop', 1500, 1500))
        coast = [row for row in self.output[1:] if row[0] == 'drive']
        self.assertTrue(all(motor == 1500 for _, motor, _ in coast))
        self.assertTrue(any(servo < 1480 for _, _, servo in coast))
        self.assertGreater(session['report']['coast_ticks'], 5)
        self.assertNotIn('arm', [row[0] for row in self.output])

    def test_identical_parallel_wall_scans_timeout_incomplete(self):
        self.prepare()
        self.advance(end=5.02, scan_factory=parallel_scan)
        self.assertIsNone(self.console.auto_session)
        self.assertEqual(self.console.auto_result['reason'], 'coast_standstill_unconfirmed')
        self.assertFalse(self.console.auto_result['completed'])
        self.assertFalse(self.console.auto_result['standstill_confirmed'])
        self.assertEqual(self.output[-1], ('stop', 1500, 1500))

    def test_actual_motion_prevents_completion_and_motor_never_reaccelerates(self):
        self.prepare()
        self.advance(end=1., scan_factory=lambda seq: room_scan(seq, x=seq*.04, y=-.12, half_width=.5))
        self.assertIsNotNone(self.console.auto_session)
        self.assertFalse(self.console.auto_session['report']['coast_motion']['stationary'])
        self.assertTrue(all(row[1] == 1500 for row in self.output[1:]))

    def test_perception_loss_resets_still_evidence_and_recovers_only_steering(self):
        self.prepare()
        self.advance(end=.3, scan_factory=lambda seq: room_scan(seq, y=-.12, half_width=.5))
        self.console.sensor_ages = lambda now: {'camera': 1.1, 'lidar': .01, 'control': .01}
        self.tick(.32)
        self.assertEqual(self.output[-1], ('drive', 1500, 1500))
        self.console.sensor_ages = lambda now: {'camera': .01, 'lidar': .01, 'control': .01}
        self.advance(start=.34, end=.8, scan_factory=lambda seq: room_scan(seq, y=-.12, half_width=.5))
        self.assertIsNotNone(self.console.auto_session)
        self.assertTrue(all(row[1] == 1500 for row in self.output[1:]))
        self.advance(start=.82, end=1.52, scan_factory=lambda seq: room_scan(seq, y=-.12, half_width=.5))
        self.assertTrue(self.console.auto_result['standstill_confirmed'])

    def test_emergency_and_known_close_obstacle_interrupt_coast_immediately(self):
        for fault in ['operator', 'front', 'feedback', 'heartbeat']:
            with self.subTest(fault=fault):
                if fault != 'operator':
                    self.tearDown(); self.setUp()
                self.prepare()
                if fault == 'operator':
                    self.console.command({'op': 'stop'})
                elif fault == 'front':
                    self.console.scan['ranges'][0] = .8
                    self.tick(.04)
                elif fault == 'feedback':
                    self.console.autonomy_healthy = lambda: False
                    self.tick(.04)
                else:
                    self.console.owner_at = -.2
                    self.console.autonomy_tick(.04)
                self.assertIsNone(self.console.auto_session)
                self.assertEqual(self.output[-1], ('stop', 1500, 1500))
                self.assertFalse(self.console.auto_result['completed'])
                old_output = list(self.output)
                self.tick(.06)
                self.assertEqual(self.output, old_output)

    def test_junction_success_requires_fresh_endpoint_after_coasting(self):
        for present in [True, False]:
            with self.subTest(endpoint_present=present):
                if not present:
                    self.tearDown(); self.setUp()
                self.prepare('left_junction_reached')
                def scan(seq):
                    value = room_scan(seq, y=-.12, half_width=.5)
                    value['left_junction'] = live_tests.junction_scan(seq)['left_junction'] if present else None
                    return value
                self.advance(scan_factory=scan)
                result = self.console.auto_result
                self.assertTrue(result['standstill_confirmed'])
                self.assertEqual(result['completed'], present)
                self.assertEqual(result['reason'], 'left_junction_reached' if present else 'left_junction_coast_position_unverified')

    def test_positive_drive_deadline_enters_neutral_coast_and_never_extends_drive(self):
        session = self.prepare()
        # Exercise the automatic deadline transition rather than the helper's transition.
        session['phase'] = 'drive'
        session['report']['phase'] = 'drive'
        session['deadline'] = .04
        self.tick(.04)
        self.assertEqual(session['phase'], 'coast')
        self.assertEqual(session['deadline'], .04)
        self.assertEqual(self.output[-1][1], 1500)
        self.assertAlmostEqual(session['coast_deadline'], 5.04)

    def test_still_evidence_starts_only_after_neutral_feedback_and_new_scan(self):
        session = self.prepare()
        session['coast_neutral_sequence'] = 100
        self.console.status['control'].update(seq=99, motor=1560)
        self.advance(end=.82, scan_factory=lambda seq: room_scan(seq, y=-.12, half_width=.5))
        self.assertIsNotNone(self.console.auto_session)
        self.assertFalse(session['report']['coast_neutral_ack'])
        self.assertFalse(session['report']['standstill_confirmed'])
        self.console.status['control'].update(seq=100, motor=1500)
        self.advance(start=.84, end=1.62, scan_factory=lambda seq: room_scan(seq, y=-.12, half_width=.5))
        self.assertTrue(self.console.auto_result['standstill_confirmed'])

    def test_timeout_wins_over_a_late_stationary_result(self):
        session = self.prepare()
        session['coast_deadline'] = .04
        class LateStill:
            def update(self, *args):
                return {'observable': True, 'stationary': True}
        self.console.coast_worker.close()
        self.console.coast_worker = self.server.CoastMotionWorker(LateStill, autostart=False)
        session['motion_generation'] = self.console.coast_worker.reset()
        self.console.scan = room_scan(2, y=-.12, half_width=.5)
        self.console.coast_worker.submit(session['motion_generation'], self.console.scan, .01, .02, .03)
        self.console.coast_worker.run_pending()
        self.tick(.04)
        self.assertEqual(self.console.auto_result['reason'], 'coast_standstill_unconfirmed')
        self.assertFalse(self.console.auto_result['completed'])

    def test_coast_timeout_cannot_be_bypassed_by_unarmed_or_old_feedback(self):
        session = self.prepare()
        session['coast_deadline'] = .04
        self.console.status['control'].update(armed=False, seq=0)
        self.tick(.04)
        self.assertEqual(self.console.auto_result['reason'], 'coast_standstill_unconfirmed')
        self.assertEqual(self.output[-1], ('stop', 1500, 1500))

    def test_auto_health_boundary_and_errors_remain_fail_closed(self):
        self.console.control_at = 10.
        with patch.object(self.server.time, 'monotonic', return_value=10.249):
            self.assertTrue(self.server.Console.autonomy_healthy(self.console))
        with patch.object(self.server.time, 'monotonic', return_value=10.250):
            self.assertFalse(self.server.Console.autonomy_healthy(self.console))
        self.console.errors['control'] = 'failed'
        with patch.object(self.server.time, 'monotonic', return_value=10.001):
            self.assertFalse(self.server.Console.autonomy_healthy(self.console))
        status = self.console.autonomy_status()
        self.assertEqual(status['control_age_limit_s'], .20)
        self.assertEqual(status['auto_control_health_s'], .25)

    def test_blocked_fit_does_not_hold_state_heartbeat_or_hard_stop_lock(self):
        entered, release, tick_done, controls_done = [threading.Event() for _ in range(4)]
        failures, results = [], []
        class HeldStill:
            def update(self, *args):
                entered.set()
                release.wait()
                return {'observable': True, 'stationary': True}
        self.console.coast_worker.close()
        worker = self.console.coast_worker = self.server.CoastMotionWorker(HeldStill)
        self.prepare()
        self.console.scan = room_scan(2, y=-.12, half_width=.5)
        def control_tick():
            try:
                with self.console.lock:
                    self.tick(.04)
            except Exception as error:
                failures.append(error)
            finally:
                tick_done.set()
        def controls():
            try:
                with patch.object(self.server.time, 'monotonic', return_value=.05):
                    results.append(self.console.state()['autonomy']['mode'])
                    results.append(self.console.autonomy_command({'op': 'heartbeat', 'boot': self.console.boot,
                        'epoch': self.console.control_epoch, 'run_id': 'test', 'seq': 1}))
                    self.console.command({'op': 'stop'})
                    results.append(self.console.state()['autonomy']['mode'])
            except Exception as error:
                failures.append(error)
            finally:
                controls_done.set()
        tick_thread = threading.Thread(target=control_tick)
        controls_thread = threading.Thread(target=controls)
        try:
            tick_thread.start()
            self.assertTrue(entered.wait(3))
            self.assertTrue(tick_done.wait(3), 'control tick waited for the blocked estimator')
            controls_thread.start()
            self.assertTrue(controls_done.wait(3), 'state, heartbeat or stop waited for fitting')
            self.assertFalse(release.is_set())
            self.assertEqual(failures, [])
            self.assertEqual(results, ['auto_probe', {'ok': True}, 'locked'])
            self.assertEqual(self.output[-1], ('stop', 1500, 1500))
            self.assertFalse(self.console.auto_result['completed'])
        finally:
            release.set()
            worker.close()
            tick_thread.join(3)
            if controls_thread.ident is not None:
                controls_thread.join(3)
            worker._thread.join(3)
        self.assertIsNone(self.console.auto_session)
        self.assertFalse(self.console.auto_result['standstill_confirmed'])


class CoastWorkerTests(unittest.TestCase):
    class Still:
        def update(self, *args):
            return {'observable': True, 'stationary': True}

    def test_stationary_result_requires_exact_current_identity_and_receive_age(self):
        worker = motion_module.CoastMotionWorker(self.Still, autostart=False)
        generation = worker.reset()
        scan = room_scan(1)
        worker.submit(generation, scan, .01, 1., 1.01)
        worker.run_pending()
        self.assertTrue(worker.result(generation, scan, .02, 1., 1.02)['stationary'])
        moved = dict(scan, ranges=list(scan['ranges']))
        moved['ranges'][0] += .01
        for value, age, received, now, reason in [
            (room_scan(2), .02, 1., 1.02, 'motion_result_superseded'),
            (dict(scan, at_ms=101), .02, 1., 1.02, 'motion_result_superseded'),
            (moved, .02, 1., 1.02, 'motion_result_superseded'),
            (scan, .01, 1.01, 1.02, 'motion_result_superseded'),
            (scan, .30, 1., 1.30, 'motion_result_stale_receive'),
            (scan, .01, 1., 1.20, 'motion_result_receive_age_disagreement'),
        ]:
            with self.subTest(reason=reason):
                result = worker.result(generation, value, age, received, now)
                self.assertFalse(result['stationary'])
                self.assertFalse(result['observable'])
                self.assertEqual(result['reason'], reason)
        worker.reset()
        self.assertFalse(worker.result(generation, scan, .02, 1., 1.02)['stationary'])
        worker.close()

    def test_one_inflight_fit_and_only_latest_pending_scan(self):
        entered, release = threading.Event(), threading.Event()
        calls = []
        class Held:
            def update(self, scan, *args):
                calls.append(scan['seq'])
                if scan['seq'] == 1:
                    entered.set()
                    release.wait()
                return {'observable': True, 'stationary': True}
        worker = motion_module.CoastMotionWorker(Held)
        generation = worker.reset()
        try:
            worker.submit(generation, room_scan(1), .01, 1., 1.01)
            self.assertTrue(entered.wait(3))
            worker.submit(generation, room_scan(2), .01, 1.1, 1.11)
            worker.submit(generation, room_scan(3), .01, 1.2, 1.21)
            self.assertEqual(calls, [1])
            self.assertFalse(worker.result(generation, room_scan(3), .02, 1.2, 1.22)['stationary'])
            release.set()
            with worker._condition:
                self.assertTrue(worker._condition.wait_for(
                    lambda: worker._result is not None and worker._result[1][0] == 3, timeout=3))
            self.assertEqual(calls, [1, 3])
            self.assertTrue(worker.result(generation, room_scan(3), .02, 1.2, 1.22)['stationary'])
        finally:
            release.set()
            worker.close()
            worker._thread.join(3)

    def test_cancel_new_session_and_close_revoke_inflight_evidence(self):
        entered, release = threading.Event(), threading.Event()
        calls = []
        class Held:
            def update(self, scan, *args):
                calls.append(scan['seq'])
                if scan['seq'] == 1:
                    entered.set()
                    release.wait()
                return {'observable': True, 'stationary': True}
        worker = motion_module.CoastMotionWorker(Held)
        old = worker.reset()
        try:
            worker.submit(old, room_scan(1), .01, 1., 1.01)
            self.assertTrue(entered.wait(3))
            current = worker.reset()
            worker.submit(current, room_scan(2), .01, 1.1, 1.11)
            self.assertFalse(worker.result(old, room_scan(1), .02, 1., 1.02)['stationary'])
            self.assertFalse(worker.result(current, room_scan(2), .02, 1.1, 1.12)['stationary'])
            release.set()
            with worker._condition:
                self.assertTrue(worker._condition.wait_for(
                    lambda: worker._result is not None and worker._result[0] == current, timeout=3))
            self.assertEqual(calls, [1, 2])
            self.assertTrue(worker.result(current, room_scan(2), .02, 1.1, 1.12)['stationary'])
            worker.close()
            self.assertFalse(worker.result(current, room_scan(2), .02, 1.1, 1.12)['stationary'])
            self.assertEqual(worker.submit(current, room_scan(3), .01, 1.2, 1.21),
                             'motion_worker_generation_changed')
        finally:
            release.set()
            worker.close()
            worker._thread.join(3)
        self.assertFalse(worker._thread.is_alive())

    def test_close_while_fitting_drops_pending_and_late_stationary_result(self):
        entered, release = threading.Event(), threading.Event()
        calls = []
        class Held:
            def update(self, scan, *args):
                calls.append(scan['seq'])
                entered.set()
                release.wait()
                return {'observable': True, 'stationary': True}
        worker = motion_module.CoastMotionWorker(Held)
        generation = worker.reset()
        try:
            worker.submit(generation, room_scan(1), .01, 1., 1.01)
            self.assertTrue(entered.wait(3))
            worker.submit(generation, room_scan(2), .01, 1.1, 1.11)
            worker.close()
            self.assertFalse(release.is_set())
            self.assertFalse(worker.result(generation, room_scan(1), .02, 1., 1.02)['stationary'])
        finally:
            release.set()
            worker.close()
            worker._thread.join(3)
        self.assertEqual(calls, [1])
        self.assertIsNone(worker._result)
        self.assertFalse(worker._thread.is_alive())

class CliStopFeedbackTests(unittest.TestCase):
    def test_failure_waits_for_fresh_neutral_feedback_before_returning(self):
        root = Path(__file__).parents[1]
        spec = importlib.util.spec_from_file_location('coast_cli', root/'autonomy-control.py')
        cli = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {'autonomy_live': live_tests.MODULE}):
            spec.loader.exec_module(cli)
        original = {'boot': 'boot', 'autonomy': {'epoch': 0}, 'status': {'control': {'tick': 100}}}
        reads, calls = [], []
        def request(path, data=None):
            if data:
                calls.append(data['op'])
                return {'run_id': 'run', 'epoch': 0}
            index = len(reads)
            reads.append(index)
            return {'autonomy': {'active': None, 'mode': 'locked',
                    'last_result': {'run_id': 'run', 'completed': False, 'reason': 'front_boundary_stop'}},
                    'ages': {'control': .3 if index == 1 else .01},
                    'status': {'control': {'armed': index == 0, 'motor': 1570 if index == 0 else 1500, 'servo': 1500}}}
        with patch.object(cli.time, 'sleep'), self.assertRaises(cli.ProbeInterrupted):
            cli.run_probe(request, original, 1570, 1000, centering=True)
        self.assertEqual(reads, [0, 1, 2])
        self.assertEqual(calls, ['straight_start', 'cancel'])


if __name__ == '__main__':
    unittest.main()
