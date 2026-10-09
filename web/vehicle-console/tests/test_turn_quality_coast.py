"""Neutral steering continuity; synthetic service clocks never certify coasting."""
import json
import copy
from pathlib import Path
import unittest
from unittest.mock import patch

import test_turn_control as control_tests


class TurnQualityCoastTests(unittest.TestCase):
    setUp = control_tests.TurnControlTests.setUp
    tearDown = control_tests.TurnControlTests.tearDown
    scan = staticmethod(control_tests.TurnControlTests.scan)
    start = control_tests.TurnControlTests.start
    tick = control_tests.TurnControlTests.tick
    observe = control_tests.TurnControlTests.observe
    drive = control_tests.TurnControlTests.drive
    start_compact_trial = control_tests.TurnControlTests.start_compact_trial
    drive_compact_trial = control_tests.TurnControlTests.drive_compact_trial
    compact_scan = control_tests.TurnControlTests.compact_scan
    enter_compact_orbit = control_tests.TurnControlTests.enter_compact_orbit

    def incomplete(self, seq):
        value = self.compact_scan(seq)
        fixture = Path(__file__).with_name('fixtures')/'left-cone-20-scan-incomplete.json'
        value['ranges'] = json.loads(fixture.read_text())['scan']['ranges']
        return value

    def hold(self):
        _, session = self.enter_compact_orbit()
        adopted = self.console.status['control']['servo']
        self.observe(round(self.now+.1, 6), self.incomplete(self.console.scan['seq']+1))
        self.assertIs(self.console.auto_session, session)
        self.assertEqual(session['phase'], 'coast')
        self.assertEqual(session['report']['quality_issues'], ['scan_incomplete'])
        self.assertEqual((self.outputs[-1]['motor'], self.outputs[-1]['servo']), (1500, adopted))
        self.assertTrue(session['report']['turn']['quality_coast_hold_active'])
        self.assertFalse(session['report']['coast_motion']['stationary'])
        return session, adopted

    def front_gap(self, seq):
        fixture = Path(__file__).with_name('fixtures')/'left-cone-33-front-gap-coast.json'
        value = json.loads(fixture.read_text())['scan']
        # Exact621 ranges/native observations, with source/receipt clocks
        # relabeled for the synthetic service session. Raw620 was not saved.
        value.update(seq=seq, at_ms=seq*100)
        return value

    def front_gap_hold(self):
        session, adopted = self.hold()  # Mock prior ACKed neutral/left state.
        self.assertEqual(self.console.status['control']['motor'], 1500)
        self.assertGreaterEqual(self.console.status['control']['seq'], session['coast_neutral_sequence'])
        deadline = session['coast_deadline']
        self.observe(round(self.now+.1, 6), self.front_gap(self.console.scan['seq']+1))
        self.assertIs(self.console.auto_session, session)
        self.assertEqual(session['report']['quality_issues'], ['scan_incomplete', 'front_sparse'])
        clearance = session['report']['clearance_current']
        self.assertEqual(clearance['front_unknown_bins'], [-30, -29, -28, -27, -26, 7])
        self.assertEqual(clearance['largest_gap_deg'], 5)
        self.assertFalse(clearance['motion_ready'])
        self.assertEqual(session['coast_deadline'], deadline)
        self.assertEqual((self.outputs[-1]['motor'], self.outputs[-1]['servo']), (1500, adopted))
        return session, adopted

    def test_actual33_front_gap_keeps_only_already_acknowledged_neutral_left_until_deadline(self):
        session, adopted = self.front_gap_hold()
        deadline, before = session['coast_deadline'], len(self.outputs)
        for _ in range(251):
            if self.console.auto_session is None:
                break
            now = round(self.now+.02, 6)
            self.tick(now, self.front_gap(1+int(round(now*10, 6))))
            if self.console.auto_session is not None:
                self.assertEqual(session['coast_deadline'], deadline)
                self.assertEqual((self.outputs[-1]['motor'], self.outputs[-1]['servo']), (1500, adopted))
        self.assertIsNone(self.console.auto_session)
        self.assertEqual(self.console.auto_result['reason'], 'coast_standstill_unconfirmed')
        self.assertTrue(all(row['motor'] == 1500 for row in self.outputs[before:]))
        self.assertFalse(self.console.auto_result['completed'])
        self.assertEqual((self.outputs[-1]['op'], self.outputs[-1]['servo']), ('stop', 1500))
        self.assertEqual(sum(row['op'] == 'arm' for row in self.outputs), 1)

    def test_actual33_front_gap_in_drive_still_halts_without_entering_hold(self):
        self.enter_compact_orbit()
        self.tick(round(self.now+.02, 6), self.front_gap(self.console.scan['seq']+1))
        self.assertIsNone(self.console.auto_session)
        self.assertEqual(self.console.auto_result['reason'], 'turn_perception_unavailable')
        self.assertEqual((self.outputs[-1]['op'], self.outputs[-1]['motor'], self.outputs[-1]['servo']),
                         ('stop', 1500, 1500))
        self.assertIsNone(self.console.auto_result.get('quality_coast_trigger'))

    def test_front_gap_recovery_keeps_original_neutral_hold_without_restart(self):
        session, adopted = self.front_gap_hold()
        before, deadline = len(self.outputs), session['coast_deadline']
        unknown = self.console.coast_worker.unknown('test_motion_unknown')
        with patch.object(self.console.coast_worker, 'result', return_value=unknown):
            for _ in range(10):
                self.observe(round(self.now+.1, 6), self.compact_scan(self.console.scan['seq']+1))
                self.assertIs(self.console.auto_session, session)
                self.assertEqual(session['phase'], 'coast')
                self.assertEqual(session['coast_deadline'], deadline)
        self.assertEqual(session['report']['quality_issues'], [])
        self.assertTrue(all((row['motor'], row['servo']) == (1500, adopted)
                            for row in self.outputs[before:]))

    def test_front_gap_cannot_hold_before_actual_neutral_ack_or_after_control_change(self):
        for fault in ('no_neutral_sequence', 'neutral_unacked', 'pending_positive_motor',
                      'positive_after_ack', 'servo_changed', 'no_handover', 'right_exit'):
            with self.subTest(fault=fault):
                if self.outputs:
                    self.tearDown()
                    self.setUp()
                session, _ = self.hold()
                control = self.console.status['control']
                if fault == 'no_neutral_sequence': session['coast_neutral_sequence'] = None
                if fault == 'neutral_unacked': session['coast_neutral_sequence'] = control['seq']+1
                if fault == 'pending_positive_motor':
                    control['motor'] = 1560
                    session['coast_neutral_sequence'] = control['seq']+1
                if fault == 'positive_after_ack': control['motor'] = 1560
                if fault == 'servo_changed': control['servo'] -= 1
                if fault == 'no_handover': session['turn_motion'].handover_observed = False
                if fault == 'right_exit': session['turn_motion'].first_pass_evidence = {'source_seq': 1}
                # A cached report ACK cannot replace this tick's actual feedback.
                session['report']['coast_neutral_ack'] = True
                self.tick(round(self.now+.02, 6), self.front_gap(self.console.scan['seq']+1))
                self.assertIsNone(self.console.auto_session)
                self.assertEqual((self.outputs[-1]['op'], self.outputs[-1]['motor'],
                                  self.outputs[-1]['servo']), ('stop', 1500, 1500))

    def test_front_gap_total_unknown_limit_and_blind_scan_still_halt(self):
        for fault in ('seventh_unknown', 'twelve_unknown', 'blind'):
            with self.subTest(fault=fault):
                if self.outputs:
                    self.tearDown()
                    self.setUp()
                self.hold()
                value = self.front_gap(self.console.scan['seq']+1)
                if fault == 'seventh_unknown': value['ranges'][8] = None
                if fault == 'twelve_unknown': value['ranges'][5:17] = [None]*12
                if fault == 'blind': value['ranges'] = [None]*360
                self.tick(round(self.now+.02, 6), value)
                self.assertIsNone(self.console.auto_session)
                self.assertEqual(self.console.auto_result['reason'], 'turn_perception_unavailable')
                self.assertEqual(self.outputs[-1]['op'], 'stop')

    def test_front_gap_eligibility_requires_matching_current_clearance(self):
        session, _ = self.front_gap_hold()
        report = session['report']
        original = copy.deepcopy(report['clearance_current'])
        for fault in ('missing', 'old_scan', 'missing_unknown', 'wrong_unknown', 'wrong_quality'):
            with self.subTest(fault=fault):
                current = copy.deepcopy(original)
                if fault == 'missing': current = None
                if fault == 'old_scan': current['scan_seq'] -= 1
                if fault == 'missing_unknown': del current['front_unknown_bins']
                if fault == 'wrong_unknown': current['front_unknown_bins'] = []
                if fault == 'wrong_quality': current['quality_issues'] = []
                report['clearance_current'] = current
                self.assertFalse(self.console.turn_quality_hold_eligible(session))
        report['clearance_current'] = original
        self.assertTrue(self.console.turn_quality_hold_eligible(session))
        self.console.scan, report['clearance_current'], report['quality_issues'] = None, None, []
        self.assertFalse(self.console.turn_quality_hold_eligible(session))

    def test_actual20_missing_returns_cut_power_hold_adopted_left_until_fixed_deadline(self):
        session, adopted = self.hold()
        self.assertAlmostEqual(session['report']['clearance_current']['body_proximity'][
            'current_known_min_distance_m'], .39667287410925445)
        start, before = self.now, len(self.outputs)
        deadline = session['coast_deadline']
        for i in range(1, 251):
            now = round(start+i*.02, 6)
            self.tick(now, self.incomplete(1+int(round(now*10, 6))))
            if i < 250:
                self.assertIs(self.console.auto_session, session)
                self.assertEqual(session['coast_deadline'], deadline)
                self.assertEqual((self.outputs[-1]['motor'], self.outputs[-1]['servo']), (1500, adopted))
                self.assertFalse(session['report']['standstill_confirmed'])
        self.assertIsNone(self.console.auto_session)
        self.assertEqual(self.console.auto_result['reason'], 'coast_standstill_unconfirmed')
        self.assertFalse(self.console.auto_result['completed'])
        self.assertTrue(all(row['motor'] == 1500 for row in self.outputs[before:]))
        self.assertEqual(self.outputs[-1]['servo'], 1500)
        self.assertEqual(sum(row['op'] == 'arm' for row in self.outputs), 1)

    def test_quality_recovery_keeps_neutral_and_left_without_restarting_entry(self):
        session, adopted = self.hold()
        before, deadline = len(self.outputs), session['coast_deadline']
        unknown = self.console.coast_worker.unknown('test_motion_unknown')
        with patch.object(self.console.coast_worker, 'result', return_value=unknown):
            for _ in range(10):
                self.observe(round(self.now+.1, 6), self.compact_scan(self.console.scan['seq']+1))
                self.assertIs(self.console.auto_session, session)
                self.assertEqual(session['phase'], 'coast')
                self.assertEqual(session['coast_deadline'], deadline)
        self.assertFalse(session['report']['perception_recovering'])
        self.assertTrue(all((row['motor'], row['servo']) == (1500, adopted)
                            for row in self.outputs[before:]))
        stationary = {**unknown, 'observable': True, 'stationary': True}
        with patch.object(self.console.coast_worker, 'result', return_value=stationary):
            self.observe(round(self.now+.1, 6), self.compact_scan(self.console.scan['seq']+1))
        self.assertIsNone(self.console.auto_session)
        self.assertEqual(self.console.auto_result['reason'], 'turn_perception_unavailable')
        self.assertFalse(self.console.auto_result['completed'])
        self.assertEqual(self.outputs[-1]['op'], 'stop')

    def test_no_handover_or_unacked_servo_cannot_enter_quality_hold(self):
        for fault in ['old_turn', 'presteer', 'before_handover', 'unacked', 'servo_mismatch']:
            with self.subTest(fault=fault):
                if self.outputs:
                    self.tearDown()
                    self.setUp()
                if fault == 'old_turn':
                    self.drive()
                elif fault == 'presteer':
                    self.start_compact_trial()
                elif fault == 'before_handover':
                    self.drive_compact_trial()
                else:
                    self.enter_compact_orbit()
                session = self.console.auto_session
                if fault == 'unacked':
                    session['turn_servo_sequence'] = self.console.sequence+1000
                elif fault == 'servo_mismatch':
                    self.console.status['control']['servo'] -= 1
                self.tick(round(self.now+.02, 6), self.incomplete(self.console.scan['seq']+1))
                self.assertIsNone(self.console.auto_session)
                self.assertEqual(self.console.auto_result['reason'], 'turn_perception_unavailable')
                self.assertEqual(self.outputs[-1]['op'], 'stop')

    def test_hold_does_not_mask_other_faults_or_operator_stop(self):
        for fault in ['front_sparse', 'body_near', 'servo_changed', 'motor_not_neutral',
                      'bridge_lock', 'control_stale', 'lidar_stale', 'loop_gap', 'operator_stop']:
            with self.subTest(fault=fault):
                if self.outputs:
                    self.tearDown()
                    self.setUp()
                session, _ = self.hold()
                value = self.incomplete(self.console.scan['seq']+1)
                if fault == 'front_sparse':
                    value['ranges'][5:17] = [None]*12
                elif fault == 'body_near':
                    value['ranges'][0] = .21
                elif fault == 'servo_changed':
                    self.console.status['control']['servo'] -= 1
                elif fault == 'motor_not_neutral':
                    self.console.status['control']['motor'] = 1560
                elif fault == 'bridge_lock':
                    self.console.status['control']['armed'] = False
                elif fault in ('control_stale', 'lidar_stale'):
                    key = fault.split('_')[0]
                    self.console.sensor_ages = lambda now: {'camera': .01, 'lidar': .01,
                                                          'control': .01, key: .31}
                if fault == 'operator_stop':
                    self.console.command({'op': 'stop'})
                else:
                    self.tick(round(self.now+(.121 if fault == 'loop_gap' else .02), 6), value)
                self.assertIsNone(self.console.auto_session)
                self.assertEqual((self.outputs[-1]['op'], self.outputs[-1]['motor'],
                                  self.outputs[-1]['servo']), ('stop', 1500, 1500))
                self.assertFalse(self.console.auto_result['completed'])


if __name__ == '__main__':
    unittest.main()
