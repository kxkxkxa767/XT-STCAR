"""Service/CLI left trial: fake adoption, synthetic scans and clocks, no devices."""
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

from test_coast_motion import room_scan
from test_turn_motion import scan as corridor_scan, opening_scan, recorded_opening_scan

ROOT = Path(__file__).parents[1]


class TurnControlTests(unittest.TestCase):
    def setUp(self):
        sys.path.insert(0, str(ROOT))
        try:
            spec = importlib.util.spec_from_file_location('turn_console_test', ROOT/'server.py')
            self.server = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(self.server)
        finally:
            sys.path.pop(0)
        self.temp = tempfile.TemporaryDirectory()
        self.console = self.server.Console(types.SimpleNamespace(
            access_file=str(Path(self.temp.name)/'access.json'), output=str(Path(self.temp.name)/'out'),
            demo=True, allow_reverse=False))
        self.console.coast_worker = self.server.CoastMotionWorker(autostart=False)
        self.console.healthy = self.console.autonomy_healthy = lambda: True
        self.console.sensor_ages = lambda now: {'camera': .01, 'lidar': now-self.console.scan_at, 'control': .01}
        self.console.status = {'control': {'armed': False, 'motor': 1500, 'servo': 1500, 'seq': 0, 'tick': 1000}}
        self.console.scan = self.scan(1)
        self.now = self.console.scan_at = self.console.control_at = 0.
        self.outputs = []
        self.adopt_servo = True
        def emit(op, motor=1500, servo=1500, tick=0):
            self.console.sequence += 1
            self.outputs.append({'op': op, 'motor': motor, 'servo': servo, 'now': self.now})
            control = self.console.status['control']
            control.update(seq=self.console.sequence, motor=motor)
            if op == 'arm':
                control['armed'] = True
            elif op == 'stop':
                control.update(armed=False, motor=1500, servo=1500)
            if self.adopt_servo:
                control['servo'] = servo
            self.console.control_at = self.now
        self.console.emit = emit

    def tearDown(self):
        self.console.coast_worker.close()
        self.temp.cleanup()

    @staticmethod
    def scan(seq, heading=60.):
        return {**room_scan(seq, half_width=.6), **corridor_scan(seq, heading=heading)}

    def start(self, max_drive_s=10.):
        with patch.object(self.server.time, 'monotonic', return_value=self.now):
            return self.console.autonomy_command({'op': 'turn_left_start', 'boot': self.console.boot,
                'epoch': self.console.control_epoch, 'tick': self.console.status['control']['tick'],
                'placement_confirmed': True, 'max_drive_s': max_drive_s})

    def tick(self, now, scan=None, control_tick=None):
        self.now = now
        self.console.owner_at = now
        self.console.control_at = now
        self.console.status['control']['tick'] = (1000+round(now*1000) if control_tick is None else control_tick)
        new_scan = self.scan(1+int(round(now*10, 6))) if scan is None else scan
        if new_scan['seq'] != self.console.scan['seq']:
            self.console.scan, self.console.scan_at = new_scan, now
        with patch.object(self.server.time, 'monotonic', return_value=now):
            self.console.autonomy_tick(now)
        self.console.coast_worker.run_pending()

    def observe(self, now, scan):
        while self.now+.02 < now-1e-9:
            self.tick(round(self.now+.02, 6), self.console.scan)
        self.tick(now, scan)

    def sparse_scan(self, seq, published=None):
        value = self.scan(seq)
        if published is not None:
            value['at_ms'] = published
        value['ranges'][5:17] = [None]*12
        return value

    def drive(self, max_drive_s=10.):
        self.start(max_drive_s)
        for index in range(1, 240):
            now = round(index*.02, 6)
            self.tick(now)
            if self.console.auto_session is None:
                self.fail(self.console.auto_result['reason'])
            if self.console.auto_session['phase'] == 'drive':
                return now
        self.fail('fresh adopted presteer failed to enter bounded drive')

    def test_preview_and_missing_placement_are_neutral_and_do_not_arm(self):
        with patch.object(self.server.time, 'monotonic', return_value=0.):
            before = self.console.autonomy_status()
            self.assertTrue(before['turn_ready'])
            self.assertFalse(before['turn_preview']['physical_steering_confirmed'])
            with self.assertRaisesRegex(ValueError, 'placement_confirmation'):
                self.console.autonomy_command({'op': 'turn_left_start', 'boot': self.console.boot,
                    'epoch': 0, 'tick': 1000})
        self.assertEqual(self.outputs, [])
        self.assertFalse(self.console.status['control']['armed'])

    def test_rejected_turn_preview_preserves_current_clearance_diagnostics(self):
        self.console.scan['corridor_candidates'] = []
        with patch.object(self.server.time, 'monotonic', return_value=0.):
            state = self.console.autonomy_status()
        self.assertFalse(state['turn_ready'])
        self.assertEqual(state['turn_rejection'], 'left_corridor_unknown')
        self.assertEqual(state['turn_clearance']['clearance_profile'], 'maneuver')
        self.assertFalse(state['turn_clearance']['body_proximity']['stop_requested'])
        self.console.scan['ranges'][90] = .24
        with patch.object(self.server.time, 'monotonic', return_value=0.):
            state = self.console.autonomy_status()
        self.assertEqual(state['turn_rejection'], 'probe_obstacle_close_body')
        self.assertTrue(state['turn_clearance']['body_proximity']['stop_requested'])
        self.assertAlmostEqual(state['turn_clearance']['body_proximity']['current_known_min_distance_m'], .07)
        self.assertEqual(self.outputs, [])

    def test_turn_lateral_margin_matches_maneuver_body_profile_and_keeps_straight_margin(self):
        turn_module = importlib.import_module(self.server.TurnMotion.__module__)
        live_module = importlib.import_module(self.server.probe_clearance.__module__)
        self.assertAlmostEqual(turn_module.TURN_LATERAL_MARGIN_M,
                               live_module.SIDE_BODY_EXTENT_M+live_module.MANEUVER_BODY_CLEARANCE_M)
        self.assertAlmostEqual(turn_module.TURN_LATERAL_MARGIN_M, .25)
        self.assertAlmostEqual(live_module.SIDE_CLEARANCE_M, .30)

    def test_turn_profile_diagnostics_follow_preview_presteer_drive_and_coast(self):
        def nearby_scan(seq):
            value = self.scan(seq)
            # These are outside the current body envelope but inside each old
            # straight gate. They do not certify any future steering sweep.
            for angle, distance in [(0, .8), (90, .29), (180, .35)]:
                value['ranges'][angle] = distance
            return value

        with patch.object(self.server.time, 'monotonic', return_value=0.):
            before = self.console.autonomy_status()
        self.assertEqual(before['clearance']['clearance_profile'], 'straight')
        self.assertEqual(before['turn_clearance']['clearance_profile'], 'maneuver')
        self.assertEqual(self.outputs, [])
        self.console.scan = nearby_scan(1)
        self.start()
        session = self.console.auto_session
        self.assertIs(session['turn_trial'], True)
        self.assertEqual(session['report']['clearance_profile_source'], 'bounded_left_turn_trial')
        self.assertEqual(session['report']['clearance_start']['clearance_profile'], 'maneuver')
        body = session['report']['clearance_start']['body_proximity']
        self.assertEqual(body['body_extents_m'], {'front': .21, 'rear': .20, 'left': .17, 'right': .17})
        self.assertEqual((body['min_net_clearance_m'], body['lidar_range_allowance_m']), (.05, .03))
        self.assertEqual(body['min_net_clearance_source'], 'operator_selected_maneuver_5cm')
        self.assertAlmostEqual(body['stop_distance_m'], .08)
        self.assertFalse(body['swept_path_certified'])
        phases = set()
        for index in range(1, 240):
            now = round(index*.02, 6)
            self.tick(now, nearby_scan(1+int(round(now*10, 6))))
            self.assertIs(self.console.auto_session, session)
            phases.add(session['phase'])
            self.assertEqual(session['report']['clearance_current']['clearance_profile'], 'maneuver')
            if session['phase'] == 'drive':
                break
        self.assertEqual(phases, {'presteer', 'drive'})
        self.assertEqual(self.outputs[-1]['motor'], 1560)
        self.assertGreaterEqual(session['report']['turn']['steering_settle_elapsed_s']+1e-9, 1.2)
        self.assertGreaterEqual(session['report']['turn']['steering_settle_feedback_ticks'], 3)
        before_coast = len(self.outputs)
        self.console.begin_coast('planned', self.now)
        now = round(self.now+.02, 6)
        self.tick(now, nearby_scan(self.console.scan['seq']+1))
        self.assertIs(self.console.auto_session, session)
        self.assertEqual(session['report']['clearance_current']['clearance_profile'], 'maneuver')
        self.assertTrue(all(row['motor'] == 1500 for row in self.outputs[before_coast:]))
        value = nearby_scan(self.console.scan['seq']+1)
        value['ranges'][90] = .24
        self.tick(round(self.now+.02, 6), value)
        self.assertIsNone(self.console.auto_session)
        self.assertEqual(self.console.auto_result['reason'], 'probe_obstacle_close_body')
        self.assertEqual((self.outputs[-1]['op'], self.outputs[-1]['motor']), ('stop', 1500))
        self.assertEqual(len([row for row in self.outputs if row['op'] == 'arm']), 1)
        old_outputs = list(self.outputs)
        self.tick(round(self.now+.02, 6))
        self.assertEqual(self.outputs, old_outputs)

    def test_request_profile_flags_cannot_weaken_straight_or_override_turn_mode(self):
        self.console.scan['ranges'][0] = .8
        for op in ['probe_start', 'straight_start', 'to_left_junction_start']:
            with self.subTest(op=op), patch.object(self.server.time, 'monotonic', return_value=0.):
                if op == 'to_left_junction_start':
                    self.console.scan.update(left_junction=None, front_boundary_m=None)
                with self.assertRaisesRegex(ValueError, 'probe_obstacle_in_straight_corridor'):
                    self.console.autonomy_command({'op': op, 'boot': self.console.boot, 'epoch': 0,
                        'tick': 1000, 'pwm': 1560, 'duration_ms': 250,
                        'clearance_profile': 'maneuver', 'camera_required': False})
        for profile in ['maneuver', 'straight', 'invalid']:
            with self.subTest(profile=profile), patch.object(self.server.time, 'monotonic', return_value=0.):
                with self.assertRaisesRegex(ValueError, 'placement_confirmation'):
                    self.console.autonomy_command({'op': 'turn_left_start', 'boot': self.console.boot,
                        'epoch': 0, 'tick': 1000, 'placement_confirmed': True, 'clearance_profile': profile})
        self.assertEqual(self.outputs, [])

    def test_straight_active_session_keeps_one_metre_gate_despite_request_flags(self):
        with patch.object(self.server.time, 'monotonic', return_value=0.):
            self.console.autonomy_command({'op': 'straight_start', 'boot': self.console.boot, 'epoch': 0,
                'tick': 1000, 'pwm': 1560, 'duration_ms': 1000,
                'clearance_profile': 'maneuver', 'camera_required': False, 'turn_trial': True})
        session = self.console.auto_session
        self.assertFalse(session.get('turn_trial', False))
        self.assertEqual(session['report']['clearance_start']['clearance_profile'], 'straight')
        self.assertTrue(session['report']['clearance_start']['camera_required'])
        value = self.scan(2)
        value['ranges'][0] = .8
        self.tick(.02, value)
        self.assertIsNone(self.console.auto_session)
        self.assertEqual(self.console.auto_result['reason'], 'probe_obstacle_in_straight_corridor')
        self.assertEqual((self.outputs[-1]['op'], self.outputs[-1]['motor']), ('stop', 1500))

    def test_recorded05_near_left_ranges_pass_turn_gate_with_synthetic_service_geometry(self):
        records = json.loads((Path(__file__).with_name('observed-turn05-body-20261007.json')).read_text())
        recorded = records['scans'][-1]
        self.assertEqual(recorded['seq'], 4216)

        def service_scan(seq):
            # Full unchanged actual05 ranges exercise the obstacle policy. The
            # corridor hypothesis and replay clock below are synthetic, so this
            # does not reconstruct the field turn or certify its future sweep.
            return {**self.scan(seq), 'ranges': list(recorded['ranges'])}

        self.console.scan = service_scan(1)
        self.start()
        session = self.console.auto_session
        for index in range(1, 240):
            now = round(index*.02, 6)
            self.tick(now, service_scan(1+int(round(now*10, 6))))
            self.assertIs(self.console.auto_session, session)
            report = session['report']
            current = report['clearance_current']
            self.assertEqual(current['clearance_profile'], 'maneuver')
            self.assertAlmostEqual(current['body_proximity']['current_known_min_distance_m'],
                                   recorded['closest_measured_body_point']['distance_to_measured_rectangle_m'])
            self.assertEqual(current['body_proximity']['nearest_point']['angle_deg'], 308)
            self.assertAlmostEqual(current['body_proximity']['nearest_point']['left_y_m'],
                                   recorded['closest_measured_body_point']['left_y_m'])
            self.assertFalse(current['body_proximity']['stop_requested'])
            if session['phase'] == 'drive':
                break
            self.assertEqual(report['current_pwm'], 1500)
        self.assertEqual(session['phase'], 'drive')
        self.assertEqual(self.outputs[-1]['motor'], 1560)
        self.assertGreaterEqual(report['turn']['steering_settle_elapsed_s']+1e-9, 1.2)
        self.assertGreaterEqual(report['turn']['steering_settle_feedback_ticks'], 3)
        value = service_scan(self.console.scan['seq']+1)
        value['ranges'][90] = .24
        self.tick(round(self.now+.02, 6), value)
        self.assertIsNone(self.console.auto_session)
        self.assertEqual(self.console.auto_result['reason'], 'probe_obstacle_close_body')
        body = self.console.auto_result['clearance_current']['body_proximity']
        self.assertTrue(body['stop_requested'])
        self.assertAlmostEqual(body['current_known_min_distance_m'], .07)
        self.assertEqual((self.outputs[-1]['op'], self.outputs[-1]['motor']), ('stop', 1500))
        self.assertFalse(self.console.auto_result['completed'])
        self.assertEqual(len([row for row in self.outputs if row['op'] == 'arm']), 1)

    def test_no_left_geometry_ambiguity_and_rear_obstacle_refuse_start(self):
        values = [dict(self.scan(1), corridor_candidates=[]), self.scan(1)]
        values[1]['corridor_candidates'].append(dict(values[1]['corridor_candidates'][0]))
        rear = self.scan(1)
        rear['ranges'][180] = .27
        values.append(rear)
        for scan in values:
            with self.subTest(scan=scan.get('corridor_candidates')):
                self.console.scan = scan
                with self.assertRaises(ValueError):
                    self.start()
        self.assertEqual(self.outputs, [])

    def test_single_owner_presteers_gradually_before_motor_and_never_completes_competition(self):
        drive_since = self.drive()
        report = self.console.auto_session['report']
        self.assertGreaterEqual(report['turn']['steering_settle_elapsed_s']+1e-9, 1.2)
        self.assertGreaterEqual(report['turn']['steering_settle_feedback_ticks'], 3)
        positives = [row for row in self.outputs if row['motor'] > 1500]
        self.assertEqual(len([row for row in self.outputs if row['op'] == 'arm']), 1)
        self.assertEqual(positives[0]['motor'], 1560)
        self.assertGreaterEqual(drive_since, 1.2)
        neutral = [row for row in self.outputs if row['op'] == 'drive' and row['motor'] == 1500]
        previous_servo, last_change = 1500, 0.
        for row in neutral:
            self.assertLessEqual(abs(row['servo']-previous_servo), 10)
            if row['servo'] != previous_servo:
                self.assertGreaterEqual(row['now']-last_change+1e-9, .10)
                last_change = row['now']
            previous_servo = row['servo']
        self.assertEqual(self.console.control_mode, 'auto_turn_trial')
        self.assertFalse(self.console.auto_session['report']['rear_launch_active'])
        self.console.command({'op': 'stop'})
        self.assertFalse(self.console.auto_result['completed'])
        self.assertEqual(self.outputs[-1]['motor'], 1500)

    def test_unadopted_steering_or_unacked_command_never_allows_positive_motor(self):
        for missing in ['servo_adoption', 'command_ack']:
            with self.subTest(missing=missing):
                if self.console.auto_session is None and self.outputs:
                    self.tearDown()
                    self.setUp()
                self.adopt_servo = missing != 'servo_adoption'
                self.start()
                for index in range(1, 260):
                    if missing == 'command_ack':
                        self.console.auto_session['turn_servo_sequence'] = self.console.sequence+1000
                    self.tick(round(index*.02, 6))
                    if self.console.auto_session is None:
                        break
                    report = self.console.auto_session['report']
                    self.assertEqual((report['drive_ticks'], report['motion_ticks']), (0, 0))
                self.assertTrue(all(row['motor'] == 1500 for row in self.outputs))
                self.assertEqual(self.console.auto_result['reason'], 'left_turn_presteer_timeout')
                self.assertFalse(self.console.auto_result['completed'])

    def test_held_control_tick_cannot_mature_adopted_presteer_on_fresh_scans(self):
        self.start()
        session = self.console.auto_session
        held_tick = None
        for index in range(1, 251):
            self.tick(round(index*.02, 6), control_tick=held_tick)
            if self.console.auto_session is None:
                break
            decision = session['report']['turn']
            if decision['steering_settle_feedback_ticks'] == 1 and held_tick is None:
                held_tick = self.console.status['control']['tick']
            if held_tick is not None:
                self.assertEqual(decision['steering_settle_feedback_ticks'], 1)
            self.assertEqual((session['phase'], session['report']['drive_ticks'], session['report']['motion_ticks']),
                             ('presteer', 0, 0))
        self.assertIsNotNone(held_tick)
        self.assertTrue(all(row['motor'] == 1500 for row in self.outputs))
        self.assertEqual(self.console.auto_result['reason'], 'left_turn_presteer_timeout')

    def test_one_sparse_presteer_scan_waits_in_same_owner_for_original_stable_recovery(self):
        self.start()
        self.observe(.1, self.scan(2))
        session = self.console.auto_session
        owner, epoch, deadline = self.console.owner, self.console.control_epoch, session['deadline']
        held_servo = session['report']['servo']
        self.observe(.2, self.sparse_scan(3))
        self.assertIs(self.console.auto_session, session)
        self.assertEqual(session['report']['quality_issues'], ['front_sparse'])
        self.assertEqual(session['report']['turn_stage'], 'presteer_wait')
        for seq, now in [(4, .303), (5, .406), (6, .509)]:
            value = self.scan(seq)
            value['at_ms'] = 100+round(now*1000)
            self.observe(now, value)
            self.assertIs(self.console.auto_session, session)
            self.assertTrue(session['report']['perception_recovering'])
            self.assertEqual((session['phase'], session['report']['current_pwm'], session['report']['servo']),
                             ('presteer', 1500, held_servo))
            self.assertEqual(session['report']['turn_stage'], 'presteer_wait')
            self.assertEqual(session['report']['turn']['geometry_source_seq'], seq)
            self.assertEqual(session['report']['turn']['steering_settle_feedback_ticks'], 0)
            self.assertEqual((session['report']['drive_ticks'], session['report']['motion_ticks']), (0, 0))
            if seq == 4:
                sparse_count = session['report']['quality_counts']['front_sparse']
            else:
                self.assertEqual(session['report']['quality_counts']['front_sparse'], sparse_count)
        value = self.scan(7)
        value['at_ms'] = 712
        self.observe(.612, value)
        self.assertFalse(session['report']['perception_recovering'])
        self.assertGreaterEqual(session['report']['recovery_stable_ms'], 300)
        self.assertEqual(session['phase'], 'presteer')
        self.assertEqual(session['report']['turn_stage'], 'presteer')
        self.assertTrue(all(row['motor'] == 1500 for row in self.outputs))
        self.assertGreater(session['report']['recovery_ticks'], 0)
        for index in range(1, 220):
            self.tick(round(.612+index*.02, 6))
            self.assertIs(self.console.auto_session, session)
            if session['phase'] == 'drive':
                break
        self.assertEqual(session['phase'], 'drive')
        self.assertEqual((self.console.owner, self.console.control_epoch, session['deadline']), (owner, epoch, deadline))
        self.assertEqual(len([row for row in self.outputs if row['op'] == 'arm']), 1)

    def test_continuous_fresh_sparse_presteer_keeps_original_five_second_deadline(self):
        self.start()
        session = self.console.auto_session
        deadline = session['deadline']
        for index in range(1, 251):
            now = round(index*.02, 6)
            self.tick(now, self.sparse_scan(1+int(round(now*10, 6))))
            if index < 250:
                self.assertIs(self.console.auto_session, session)
                self.assertEqual(session['deadline'], deadline)
                self.assertEqual(session['phase'], 'presteer')
        self.assertIsNone(self.console.auto_session)
        self.assertEqual(self.console.auto_result['reason'], 'left_turn_presteer_timeout')
        self.assertEqual(deadline, 15.)
        self.assertFalse(self.console.auto_result['completed'])
        self.assertTrue(all(row['motor'] == 1500 for row in self.outputs))
        self.assertEqual(len([row for row in self.outputs if row['op'] == 'arm']), 1)

    def test_sparse_during_drive_or_coast_still_immediately_locks(self):
        for coast in [False, True]:
            with self.subTest(coast=coast):
                if self.console.auto_session is None and self.outputs:
                    self.tearDown()
                    self.setUp()
                self.drive()
                if coast:
                    self.console.begin_coast('planned', self.now)
                now = round(self.now+.02, 6)
                self.observe(now, self.sparse_scan(self.console.scan['seq']+1,
                                                 self.console.scan['at_ms']+20))
                self.assertIsNone(self.console.auto_session)
                self.assertEqual(self.console.auto_result['reason'], 'turn_perception_unavailable')
                self.assertEqual((self.outputs[-1]['op'], self.outputs[-1]['motor'], self.outputs[-1]['servo']),
                                 ('stop', 1500, 1500))

    def test_neutral_presteer_wait_does_not_accept_incomplete_scan_or_positive_adoption(self):
        for fault in ['incomplete', 'positive_motor', 'reported_positive', 'unarmed', 'old_arm_ack']:
            with self.subTest(fault=fault):
                if self.console.auto_session is None and self.outputs:
                    self.tearDown()
                    self.setUp()
                self.start()
                value = self.sparse_scan(2)
                self.observe(.08, self.console.scan)
                if fault == 'incomplete': value['ranges'][80:87] = [None]*7
                if fault == 'positive_motor': self.console.status['control']['motor'] = 1560
                if fault == 'reported_positive': self.console.auto_session['report']['current_pwm'] = 1560
                if fault == 'unarmed': self.console.status['control']['armed'] = False
                if fault == 'old_arm_ack': self.console.status['control']['seq'] = 0
                self.tick(.1, value)
                self.assertIsNone(self.console.auto_session)
                self.assertEqual(self.console.auto_result['reason'], 'turn_perception_unavailable')
                self.assertEqual(self.outputs[-1]['op'], 'stop')

    def test_neutral_presteer_wait_preserves_hard_sensor_and_obstacle_stops(self):
        for fault in ['front', 'side', 'rear', 'lidar_stale', 'control_stale', 'loop_gap']:
            with self.subTest(fault=fault):
                if self.console.auto_session is None and self.outputs:
                    self.tearDown()
                    self.setUp()
                self.start()
                self.observe(.1, self.sparse_scan(2))
                self.assertIsNotNone(self.console.auto_session)
                if fault == 'front': self.console.scan['ranges'][0] = .28
                if fault == 'side': self.console.scan['ranges'][90] = .24
                if fault == 'rear': self.console.scan['ranges'][180] = .27
                if fault == 'lidar_stale': self.console.sensor_ages = lambda now: {'camera': .01, 'lidar': .3, 'control': .01}
                if fault == 'control_stale': self.console.sensor_ages = lambda now: {'camera': .01, 'lidar': .01, 'control': .2}
                self.tick(.181 if fault == 'loop_gap' else .12, self.console.scan)
                self.assertIsNone(self.console.auto_session)
                self.assertEqual((self.outputs[-1]['op'], self.outputs[-1]['motor'], self.outputs[-1]['servo']),
                                 ('stop', 1500, 1500))
                self.assertFalse(self.console.auto_result['completed'])

    def test_repeated_recovery_scan_and_operator_stop_cannot_resume_waiting_turn(self):
        self.start()
        self.observe(.1, self.sparse_scan(2))
        self.observe(.2, self.scan(3))
        session = self.console.auto_session
        self.observe(.48, self.console.scan)
        self.assertIs(self.console.auto_session, session)
        self.assertEqual(session['recovery'].good_scans, 1)
        self.assertEqual(session['report']['current_pwm'], 1500)
        self.console.command({'op': 'stop'})
        before = len(self.outputs)
        self.tick(.5, self.scan(4))
        self.assertIsNone(self.console.auto_session)
        self.assertEqual(len(self.outputs), before)
        self.assertEqual(self.console.auto_result['reason'], 'operator_stop')
        self.assertEqual(len([row for row in self.outputs if row['op'] == 'arm']), 1)

    def test_neutral_presteer_wait_does_not_extend_total_deadline(self):
        self.start()
        self.console.auto_session['deadline'] = .2
        self.observe(.1, self.sparse_scan(2))
        self.observe(.2, self.sparse_scan(3))
        self.assertIsNone(self.console.auto_session)
        self.assertEqual(self.console.auto_result['reason'], 'turn_total_deadline')
        self.assertFalse(self.console.auto_result['completed'])
        self.assertTrue(all(row['motor'] == 1500 for row in self.outputs))

    def test_opening_endpoint_ahead_still_presteers_left_before_positive_motor(self):
        def recorded_ahead_scan(seq):
            # Actual 04 first goal, wall and support rays. Other sectors and
            # replay clocks are synthetic; this isolates service ordering.
            value = recorded_opening_scan(886)
            value.update(seq=seq, at_ms=seq*100)
            value['ranges'] = [3.2 if distance is None else distance for distance in value['ranges']]
            return value
        for name, ahead_scan in [('synthetic', lambda seq: opening_scan(seq, endpoint_index=315)),
                                 ('actual_04_goal_and_rays', recorded_ahead_scan)]:
            with self.subTest(name=name):
                if self.outputs:
                    self.tearDown()
                    self.setUp()
                self.assert_opening_presteers_left(ahead_scan)

    def assert_opening_presteers_left(self, sample):
        self.console.scan = sample(1)
        self.start()
        session = self.console.auto_session
        self.assertEqual(session['report']['turn_stage'], 'presteer')
        first_left = None
        for index in range(1, 251):
            now = round(index*.02, 6)
            self.tick(now, sample(1+int(round(now*10, 6))))
            self.assertIs(self.console.auto_session, session)
            report = session['report']
            if report['servo'] > 1500 and first_left is None:
                first_left = now
            self.assertTrue(report['turn']['incoming_endpoint_current'])
            self.assertGreater(report['turn']['incoming_endpoint']['incoming_left_end_m'], .49)
            if session['phase'] == 'drive':
                break
            self.assertEqual((report['current_pwm'], report['drive_ticks'], report['motion_ticks']), (1500, 0, 0))
        self.assertIsNotNone(first_left)
        self.assertEqual(session['phase'], 'drive')
        self.assertEqual(session['report']['turn_stage'], 'drive')
        self.assertEqual(session['report']['current_pwm'], 1560)
        self.assertGreater(session['report']['servo'], 1500)
        self.assertGreaterEqual(session['report']['turn']['steering_settle_elapsed_s']+1e-9, 1.2)
        self.assertGreaterEqual(session['report']['turn']['steering_settle_feedback_ticks'], 3)
        self.assertGreaterEqual(self.now-first_left, 1.2)
        self.assertGreater(session['report']['presteer_ticks'], 0)
        self.assertEqual((session['report']['drive_ticks'], session['report']['motion_ticks']), (1, 1))
        self.assertFalse(session['report']['physical_steering_confirmed'])
        self.assertFalse(session['report']['turn']['completed'])
        self.assertEqual(len([row for row in self.outputs if row['op'] == 'arm']), 1)

    def test_drive_retargets_left_and_returns_toward_center_from_current_geometry(self):
        self.drive()
        session = self.console.auto_session
        before_servo = session['report']['servo']
        before_changes = session['report']['steering_changes']
        last_change = session['turn_motion'].last_change
        start, before = self.now, len(self.outputs)
        # Continuous changing native corridor headings model new observations,
        # not physical vehicle yaw or a measured steering response.
        for index in range(1, 6):
            now = round(start+index*.1, 6)
            self.observe(now, self.scan(self.console.scan['seq']+1, heading=60+index*4))
            self.assertIs(self.console.auto_session, session)
            self.assertEqual(session['phase'], 'drive')
        self.assertGreater(session['report']['servo'], before_servo)
        more_left_servo = session['report']['servo']
        for index in range(1, 21):
            now = round(start+.5+index*.1, 6)
            self.observe(now, self.scan(self.console.scan['seq']+1, heading=80-index*3.625))
            self.assertIs(self.console.auto_session, session)
            self.assertEqual(session['phase'], 'drive')
        self.assertLess(session['report']['servo'], before_servo)
        self.assertLess(session['report']['servo'], more_left_servo)
        self.assertEqual(session['report']['turn']['steering_target'], 1500)
        self.assertEqual(session['report']['servo'], 1500)
        changes, previous_servo = 0, before_servo
        for row in self.outputs[before:]:
            self.assertEqual((row['op'], row['motor']), ('drive', 1560))
            self.assertLessEqual(abs(row['servo']-previous_servo), 10)
            if row['servo'] != previous_servo:
                self.assertGreaterEqual(row['now']-last_change+1e-9, .10)
                last_change = row['now']
                changes += 1
            previous_servo = row['servo']
        self.assertGreater(changes, 0)
        self.assertEqual(session['report']['steering_changes']-before_changes, changes)
        self.assertEqual(session['turn_last_servo'], session['report']['servo'])
        self.assertLessEqual(session['turn_servo_sequence'], self.console.status['control']['seq'])
        self.assertEqual(session['report']['turn_stage'], 'drive')
        self.assertFalse(session['report']['entry_confirmed'])
        self.assertEqual(len([row for row in self.outputs if row['op'] == 'arm']), 1)

    def test_short_geometry_gap_retains_source_then_sustained_loss_locks_without_rearm(self):
        self.drive()
        now = self.now+.10
        scan = dict(self.scan(1+int(round(now*10, 6))), corridor_candidates=[])
        self.tick(now-.08)  # Preserve the 80ms loop before admitting the next new scan.
        self.tick(now-.06)
        self.tick(now-.04)
        self.tick(now-.02)
        self.tick(now, scan)
        session = self.console.auto_session
        self.assertIsNotNone(session)
        source_seq = session['report']['turn']['geometry_source_seq']
        self.assertTrue(session['report']['turn']['geometry_missing'])
        self.assertEqual(session['report']['current_pwm'], 1560)
        for index in range(1, 16):
            value_now = round(now+index*.02, 6)
            empty = dict(self.scan(1+int(round(value_now*10, 6))), corridor_candidates=[])
            self.tick(value_now, empty)
            if self.console.auto_session is None:
                break
            self.assertEqual(session['report']['turn']['geometry_source_seq'], source_seq)
        self.assertIsNone(self.console.auto_session)
        self.assertEqual(self.outputs[-1]['op'], 'stop')
        old = list(self.outputs)
        self.tick(round(self.now+.02, 6))
        self.assertEqual(self.outputs, old)
        self.assertFalse(self.console.auto_result['completed'])

    def test_coast_timeout_or_still_keeps_motor_neutral_and_no_second_arm(self):
        self.drive()
        session = self.console.auto_session
        motion = session['turn_motion']
        motion.begin_coast('left_turn_observed_corridor_alignment', self.now)
        motion.observed_alignment = True
        before = len(self.outputs)
        for index in range(1, 280):
            self.tick(round(self.now+.02, 6))
            if self.console.auto_session is None:
                break
        self.assertIsNone(self.console.auto_session)
        self.assertTrue(all(row['motor'] == 1500 for row in self.outputs[before:]))
        self.assertFalse(self.console.auto_result['completed'])
        self.assertFalse(self.console.auto_result['entry_confirmed'])
        self.assertEqual(len([row for row in self.outputs if row['op'] == 'arm']), 1)

    def test_coast_service_consumes_turn_updates_and_releases_servo_with_ack_bookkeeping(self):
        self.drive()
        session = self.console.auto_session
        before_servo = session['report']['servo']
        before_changes = session['report']['steering_changes']
        last_change = session['turn_motion'].last_change
        start, before = self.now, len(self.outputs)
        self.console.begin_coast('left_turn_drive_timeout', start)
        deadline = session['coast_deadline']
        unknown = self.console.coast_worker.unknown('test_motion_unknown')
        with patch.object(self.console.coast_worker, 'result', return_value=unknown):
            for index in range(1, 101):
                self.tick(round(start+index*.02, 6))
                self.assertIs(self.console.auto_session, session)
                self.assertEqual(session['coast_deadline'], deadline)
        changes, previous_servo = 0, before_servo
        for row in self.outputs[before:]:
            self.assertEqual((row['op'], row['motor']), ('drive', 1500))
            self.assertLessEqual(abs(row['servo']-previous_servo), 10)
            if row['servo'] != previous_servo:
                self.assertGreaterEqual(row['now']-last_change+1e-9, .10)
                last_change = row['now']
                changes += 1
            previous_servo = row['servo']
        self.assertGreater(changes, 0)
        self.assertEqual(session['report']['steering_changes']-before_changes, changes)
        self.assertEqual(session['turn_last_servo'], session['report']['servo'])
        self.assertEqual(session['report']['servo'], 1500)
        self.assertEqual(session['report']['turn']['geometry_source_seq'], self.console.scan['seq'])
        self.assertEqual(session['report']['turn_stage'], 'coast')
        self.assertTrue(session['report']['coast_neutral_ack'])
        self.assertLessEqual(session['turn_servo_sequence'], self.console.status['control']['seq'])
        self.assertEqual(len([row for row in self.outputs if row['op'] == 'arm']), 1)

    def test_turn_coast_exact_five_second_deadline_never_rearms_or_restores_motor(self):
        self.drive()
        session = self.console.auto_session
        start, before = self.now, len(self.outputs)
        self.console.begin_coast('left_turn_drive_timeout', start)
        self.assertEqual(session['coast_deadline']-start, 5.)
        unknown = self.console.coast_worker.unknown('test_motion_unknown')
        with patch.object(self.console.coast_worker, 'result', return_value=unknown):
            for index in range(1, 251):
                self.tick(round(start+index*.02, 6))
                if index < 250:
                    self.assertIs(self.console.auto_session, session)
        self.assertIsNone(self.console.auto_session)
        self.assertEqual(self.console.auto_result['reason'], 'coast_standstill_unconfirmed')
        self.assertFalse(self.console.auto_result['completed'])
        self.assertEqual(self.outputs[-1]['op'], 'stop')
        self.assertTrue(all(row['motor'] == 1500 for row in self.outputs[before:]))
        self.assertEqual(len([row for row in self.outputs if row['op'] == 'arm']), 1)

    def test_coast_bridge_lock_on_repeated_scan_is_immediate_neutral(self):
        self.drive()
        self.console.begin_coast('planned', self.now)
        self.console.status['control']['armed'] = False
        self.tick(round(self.now+.02, 6), self.console.scan)
        self.assertIsNone(self.console.auto_session)
        self.assertEqual(self.console.auto_result['reason'], 'turn_bridge_locked')
        self.assertEqual((self.outputs[-1]['op'], self.outputs[-1]['motor'], self.outputs[-1]['servo']),
                         ('stop', 1500, 1500))

    def test_drive_budget_is_not_extended_by_pending_servo_adoption(self):
        drive_since = self.drive(max_drive_s=1.)
        self.console.auto_session['turn_servo_sequence'] = self.console.sequence+1000
        for index in range(1, 80):
            self.tick(round(drive_since+index*.02, 6))
            if self.console.auto_session is None or self.console.auto_session['phase'] == 'coast':
                break
        self.assertIsNotNone(self.console.auto_session)
        self.assertEqual(self.console.auto_session['phase'], 'coast')
        self.assertEqual(self.console.auto_session['coast_reason'], 'left_turn_drive_timeout')
        self.assertEqual(self.outputs[-1]['motor'], 1500)
        self.assertLessEqual(self.now-drive_since, 1.02)

    def test_front_and_side_obstacle_prioritize_immediate_neutral_lock(self):
        for angle, distance in [(0, .28), (90, .24), (180, .27), (270, .24)]:
            with self.subTest(angle=angle):
                if self.outputs:
                    self.tearDown()
                    self.setUp()
                self.drive()
                now = round(self.now+.02, 6)
                value = self.scan(self.console.scan['seq']+1)
                value['ranges'][angle] = distance
                self.tick(now, value)
                self.assertIsNone(self.console.auto_session)
                self.assertEqual(self.console.auto_result['reason'], 'probe_obstacle_close_body')
                self.assertEqual(self.outputs[-1], {'op': 'stop', 'motor': 1500, 'servo': 1500, 'now': now})
                self.assertFalse(self.console.auto_result['completed'])

    def test_candidate_settings_bounds_and_defaults_preserve_unvalidated_right_label(self):
        self.assertEqual((self.console.settings['left'], self.console.settings['right']), (1650, 1350))
        self.console.configure({'forward': 1550, 'reverse': 1450, 'left': 1720, 'right': 1270})
        for key, value in [('left', 1721), ('right', 1269)]:
            with self.assertRaises(ValueError):
                self.console.configure({**self.console.settings, key: value})
        with patch.object(self.server.time, 'monotonic', return_value=0.):
            limits = self.console.autonomy_status()['steering_candidate_bounds']
        self.assertEqual((limits['min'], limits['max']), (1270, 1720))
        self.assertFalse(limits['right_physically_validated'])

    def test_camera_absence_is_diagnostic_for_lidar_only_turn_without_faking_age(self):
        self.console.sensor_ages = lambda now: {'camera': 99., 'lidar': now-self.console.scan_at, 'control': .01}
        self.console.errors['camera'] = 'offline camera'
        self.drive()
        session = self.console.auto_session
        self.assertEqual(session['report']['sensor_inputs'], ['lidar', 'control'])
        self.assertEqual(session['report']['sensor_ages']['camera'], 99.)
        self.assertNotIn('camera_unavailable', session['report']['quality_issues'])
        self.console.camera_failure(RuntimeError('camera ended'))
        self.assertIs(self.console.auto_session, session)
        self.tick(round(self.now+.02, 6))
        self.assertIs(self.console.auto_session, session)
        self.assertEqual(self.outputs[-1]['motor'], 1560)

    def test_control_fault_still_locks_turn_and_old_camera_failure_behavior_is_preserved(self):
        self.drive()
        self.console.errors['control_write'] = 'failed write'
        self.tick(round(self.now+.02, 6))
        self.assertIsNone(self.console.auto_session)
        self.assertEqual(self.outputs[-1]['op'], 'stop')
        self.console.camera_failure(RuntimeError('camera failed in locked/manual mode'))
        self.assertEqual(self.outputs[-1]['op'], 'stop')

    def test_manual_and_simulated_targets_register_but_point_execution_requires_real_pose(self):
        for source_kind in ['manual', 'simulated']:
            with self.subTest(source_kind=source_kind), patch.object(self.server.time, 'monotonic', return_value=0.):
                target = {'schema_version': 1, 'goal_id': source_kind+'-point', 'source_kind': source_kind,
                    'goal_type': 'point_stop', 'frame': 'run_start_lidar_reference',
                    'coordinate_convention': 'x_forward_y_left_yaw_left_positive', 'max_seconds': 10,
                    'target_point_left_m': {'x_m': 1.2, 'y_m': 1.5}}
                registered = self.console.autonomy_command({'op': 'trial_goal_register', 'boot': self.console.boot,
                    'epoch': 0, 'goal': target})
                self.assertTrue(registered['registered'])
                self.assertFalse(registered['motion_requested'])
                self.assertEqual(registered['trial_goal']['source_kind'], source_kind)
                status = self.console.autonomy_status()['trial_goal']
                self.assertEqual(status['execution_rejection'], 'real_pose_missing')
                with self.assertRaisesRegex(ValueError, 'real_pose_missing'):
                    self.console.autonomy_command({'op': 'turn_left_start', 'boot': self.console.boot,
                        'epoch': 0, 'tick': 1000, 'placement_confirmed': True, 'goal_id': target['goal_id']})
        self.assertEqual(self.outputs, [])

    def test_simulated_exit_intent_uses_real_opening_geometry_not_a_simulated_pose(self):
        target = {'schema_version': 1, 'goal_id': 'exit-intent', 'source_kind': 'simulated',
            'goal_type': 'turn_exit_align', 'frame': 'run_start_lidar_reference',
            'coordinate_convention': 'x_forward_y_left_yaw_left_positive', 'max_seconds': 4}
        self.console.scan = opening_scan(1)
        with patch.object(self.server.time, 'monotonic', return_value=0.):
            registered = self.console.autonomy_command({'op': 'trial_goal_register', 'boot': self.console.boot,
                'epoch': 0, 'goal': target})
            self.assertEqual(self.outputs, [])
            self.assertTrue(self.console.autonomy_status()['trial_goal']['execution_ready'])
            self.console.autonomy_command({'op': 'turn_left_start', 'boot': self.console.boot,
                'epoch': 0, 'tick': 1000, 'placement_confirmed': True, 'goal_id': target['goal_id']})
        session = self.console.auto_session
        self.assertEqual(session['turn_motion'].max_drive_s, 4)
        self.assertEqual(session['report']['turn']['geometry_mode'], 'opening_wall')
        self.assertFalse(session['report']['turn']['turn_goal']['origin_between_exit_walls'])
        self.assertFalse(session['report']['physical_steering_confirmed'])
        self.assertEqual([(row['op'], row['motor']) for row in self.outputs], [('arm', 1500)])
        self.assertEqual(registered['trial_goal']['reference_scan_seq'], 1)
        self.assertEqual(registered['trial_goal']['reference_publication_at_ms'], 100)

    def test_target_registration_expiry_does_not_refresh_measurement_or_allow_late_start(self):
        target = {'schema_version': 1, 'goal_id': 'exit-intent', 'source_kind': 'manual',
            'goal_type': 'turn_exit_align', 'frame': 'run_start_lidar_reference',
            'coordinate_convention': 'x_forward_y_left_yaw_left_positive', 'max_seconds': 4}
        with patch.object(self.server.time, 'monotonic', return_value=0.):
            registered = self.console.autonomy_command({'op': 'trial_goal_register', 'boot': self.console.boot,
                'epoch': 0, 'goal': target})
        expires = registered['trial_goal']['expires_monotonic_s']
        self.assertEqual(self.console.scan_at, 0.)
        self.now = self.console.scan_at = self.console.control_at = expires
        self.console.scan = self.scan(2)
        with patch.object(self.server.time, 'monotonic', return_value=expires):
            self.assertEqual(self.console.autonomy_status()['trial_goal']['execution_rejection'], 'trial_goal_expired')
            with self.assertRaisesRegex(ValueError, 'trial_goal_expired'):
                self.console.autonomy_command({'op': 'turn_left_start', 'boot': self.console.boot,
                    'epoch': 0, 'tick': 1000, 'placement_confirmed': True, 'goal_id': target['goal_id']})
        self.assertEqual(self.outputs, [])


class TurnCliTests(unittest.TestCase):
    def module(self):
        sys.path.insert(0, str(ROOT))
        try:
            spec = importlib.util.spec_from_file_location('turn_cli_test', ROOT/'autonomy-control.py')
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            return module
        finally:
            sys.path.pop(0)

    def test_default_cli_only_gets_state(self):
        cli = self.module()
        state = {'healthy': True, 'boot': 'boot', 'status': {'control': {'armed': False, 'motor': 1500, 'servo': 1500}},
            'autonomy': {'turn_ready': True, 'turn_rejection': None, 'turn_preview': {'candidate_only': True}}}
        requests = []
        class Opener:
            def open(self, request, timeout=None):
                requests.append(request)
                return io.BytesIO(json.dumps(state).encode())
        with tempfile.TemporaryDirectory() as directory:
            access = Path(directory)/'access.json'
            access.write_text(json.dumps({'port': 8081, 'token': 'offline-token'}))
            for command in ['turn-left', 'trial-goal-status']:
                with patch.object(sys, 'argv', ['autonomy-control.py', command, '--access-file', str(access)]),\
                     patch.object(cli.urllib.request, 'build_opener', return_value=Opener()), patch.object(sys, 'stdout', io.StringIO()):
                    cli.main()
        self.assertEqual([request.get_method() for request in requests], ['GET', 'GET'])

    def test_cli_explicit_target_registration_preserves_small_message_limit(self):
        cli = self.module()
        state = {'boot': 'boot', 'autonomy': {'epoch': 0}}
        target = {'schema_version': 1, 'goal_id': 'manual-intent', 'source_kind': 'manual',
            'goal_type': 'turn_exit_align', 'frame': 'run_start_lidar_reference',
            'coordinate_convention': 'x_forward_y_left_yaw_left_positive', 'max_seconds': 10}
        requests = []
        class Opener:
            def open(self, request, timeout=None):
                requests.append(request)
                return io.BytesIO(json.dumps(state if request.get_method() == 'GET' else {'registered': True}).encode())
        with tempfile.TemporaryDirectory() as directory:
            access, goal = Path(directory)/'access.json', Path(directory)/'goal.json'
            access.write_text(json.dumps({'port': 8081, 'token': 'offline-token'}))
            goal.write_text(json.dumps(target))
            argv = ['autonomy-control.py', 'trial-goal-register', '--access-file', str(access), '--goal-file', str(goal)]
            with patch.object(sys, 'argv', argv), patch.object(cli.urllib.request, 'build_opener', return_value=Opener()),\
                 patch.object(sys, 'stdout', io.StringIO()):
                cli.main()
                goal.write_text(' '*2049)
                with self.assertRaisesRegex(RuntimeError, 'HTTP limit'):
                    cli.main()
        self.assertEqual([request.get_method() for request in requests], ['GET', 'POST', 'GET'])
        body = json.loads(requests[1].data)
        self.assertEqual(body['op'], 'trial_goal_register')
        self.assertEqual(body['goal'], target)
        self.assertLessEqual(len(requests[1].data), 2048)
        self.assertNotIn('target_only', body['goal'])

    def test_cli_starts_once_and_waits_for_neutral_before_report(self):
        cli = self.module()
        calls, reads = [], []
        state = {'boot': 'boot', 'autonomy': {'epoch': 0}, 'status': {'control': {'tick': 100}}}
        def request(path, data=None):
            if data:
                calls.append(data)
                return {'run_id': 'run', 'epoch': 0}
            index = len(reads)
            reads.append(index)
            return {'autonomy': {'active': None, 'mode': 'locked', 'last_result': {
                'run_id': 'run', 'completed': False, 'reason': 'left_turn_observed_corridor_alignment', 'observed_alignment': True}},
                'ages': {'control': .01}, 'status': {'control': {'armed': index == 0,
                    'motor': 1500, 'servo': 1720 if index == 0 else 1500}}}
        with patch.object(cli.time, 'sleep'):
            result = cli.left_turn_trial(request, state, 10, True)
        self.assertEqual([call['op'] for call in calls], ['turn_left_start', 'cancel'])
        self.assertEqual(reads, [0, 1])
        self.assertTrue(calls[0]['placement_confirmed'])
        self.assertEqual(calls[0]['max_drive_s'], 10)
        self.assertNotIn('pwm', calls[0])
        self.assertFalse(result['completed'])
        self.assertFalse(result['entry_confirmed'])

    def test_cli_missing_invalid_or_stale_feedback_age_never_claims_fresh_neutral(self):
        cli = self.module()
        calls, reads = [], []
        state = {'boot': 'boot', 'autonomy': {'epoch': 0}, 'status': {'control': {'tick': 100}}}
        ages = [None, True, float('nan'), -.01, .2, .199]
        def request(path, data=None):
            if data:
                calls.append(data['op'])
                return {'run_id': 'run', 'epoch': 0}
            index = len(reads)
            reads.append(index)
            return {'autonomy': {'active': None, 'mode': 'locked', 'last_result': {
                'run_id': 'run', 'completed': False, 'reason': 'left_turn_drive_timeout'}},
                'ages': {'control': ages[index]},
                'status': {'control': {'armed': False, 'motor': 1500, 'servo': 1500}}}
        with patch.object(cli.time, 'sleep'):
            result = cli.left_turn_trial(request, state, 10, True)
        self.assertEqual(reads, list(range(6)))
        self.assertEqual(calls, ['turn_left_start', 'cancel'])
        self.assertTrue(result['fresh_neutral_locked_confirmed'])


if __name__ == '__main__':
    unittest.main()
