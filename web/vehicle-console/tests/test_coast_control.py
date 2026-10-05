"""Actual console output/state-machine checks with offline scans and an injected clock."""
import importlib.util
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import test_autonomy_live as live_tests
from test_coast_motion import room_scan, parallel_scan


class CoastControlTests(unittest.TestCase):
    setUp = live_tests.QualityRecoveryTests.setUp
    tearDown = live_tests.QualityRecoveryTests.tearDown
    tick = live_tests.QualityRecoveryTests.tick

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
        session['coast_motion'] = LateStill()
        self.console.scan = room_scan(2, y=-.12, half_width=.5)
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
