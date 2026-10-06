"""No hardware: native geometry, adoption evidence and bounded left trial checks."""
import importlib.util
import math
from pathlib import Path
import unittest

SPEC = importlib.util.spec_from_file_location('turn_motion', Path(__file__).parents[1]/'turn_motion.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def corridor(heading=60., offset=0., width=1.2):
    return {'heading_left_rad': math.radians(heading), 'center_offset_left_m': offset,
            'width_m': width, 'left_wall_points': 40, 'right_wall_points': 45,
            'support_span_m': .8, 'fit_error_m': .02, 'origin_between_walls': True,
            'candidate_only': True, 'turn_path_certified': False}


def scan(seq, heading=60., offset=0., width=1.2, published=None):
    forward = corridor(heading, offset, width)
    backward = corridor(heading-180., -offset, width)
    return {'seq': seq, 'at_ms': seq*100 if published is None else published,
            'frame_id': 'lidar_origin_coarse_body_heading', 'navigation_validated': False,
            'corridor_candidates': [forward, backward]}


def control(seq, servo=1500, armed=True, motor=1500, tick=None):
    return {'seq': seq, 'tick': seq*100 if tick is None else tick,
            'armed': armed, 'motor': motor, 'servo': servo}


def wall(heading=60., rho=-1.8, start=1., end=3.):
    theta = math.radians(heading)
    def point(t):
        return {'x_m': math.cos(theta)*t-math.sin(theta)*rho,
                'y_m': math.sin(theta)*t+math.cos(theta)*rho}
    return {'heading_left_rad': theta, 'rho_left_m': rho,
            'support_start_left_m': point(start), 'support_end_left_m': point(end),
            'support_span_m': end-start, 'points': 32, 'fit_error_m': .02, 'candidate_only': True}


def opening_scan(seq):
    value = dict(scan(seq), corridor_candidates=[])
    theta, target_range = math.radians(30.), 2.4
    outer = wall()
    value['ranges'] = [3.2]*360
    value['wall_candidates'] = [outer]
    value['left_turn_goal'] = {'heading_left_rad': math.radians(60.), 'center_offset_left_m': -1.2,
                              'width_m': 1.2, 'target_point_left_m': {'x_m': target_range*math.cos(theta),
                                                                   'y_m': target_range*math.sin(theta)},
                              'target_support_ray': {'index': 330, 'angle_left_rad': theta,
                                                     'range_m': 3.2, 'target_range_m': target_range},
                              'outer_wall': outer, 'incoming_heading_left_rad': 0., 'front_wall_m': 2.2,
                              'incoming_left_end_m': .5, 'origin_between_exit_walls': False,
                              'candidate_only': True, 'turn_path_certified': False,
                              'observation_type': 'left_opening_outer_wall_alignment'}
    return value


class TurnMotionTests(unittest.TestCase):
    def drive(self, heading=60., offset=0., max_drive_s=10.):
        motion = MODULE.TurnMotion(0., max_drive_s=max_drive_s)
        result = {'servo': 1500}
        outputs = []
        for seq in range(1, 49):
            now = (seq-1)*.1
            result = motion.update(scan(seq, heading, offset), .01, now, control(seq, result['servo']))
            outputs.append(result)
            if result['phase'] == 'drive':
                return motion, seq, now, result, outputs
        self.fail('fresh continuous adopted presteer should reach bounded trial drive')

    def test_neutral_preview_never_ramps_or_requests_positive_motor(self):
        motion = MODULE.TurnMotion(10.)
        result = motion.update(scan(1), .01, 10., control(1, armed=False))
        self.assertTrue(result['start_ready'])
        self.assertEqual((result['motor'], result['servo']), (1500, 1500))
        self.assertFalse(result['physical_steering_confirmed'])

    def test_pwm_gradual_presteer_and_actual_adoption_allowance(self):
        motion, _, now, result, outputs = self.drive()
        self.assertGreaterEqual(now, 1.2)
        self.assertTrue(all(r['motor'] == 1500 for r in outputs[:-1]))
        self.assertEqual(result['motor'], 1560)
        self.assertTrue(all(0 <= b['servo']-a['servo'] <= 10 for a, b in zip(outputs, outputs[1:])))
        self.assertTrue(all(1500 <= r['servo'] <= 1720 for r in outputs))
        self.assertGreaterEqual(result['steering_settle_elapsed_s'], 1.2-1e-9)
        self.assertGreaterEqual(result['steering_settle_feedback_ticks'], 3)
        self.assertFalse(result['physical_steering_confirmed'])
        self.assertFalse(result['completed'])

    def test_motor_neutral_ack_with_wrong_servo_never_starts_drive(self):
        motion = MODULE.TurnMotion(0.)
        for seq in range(1, 49):
            result = motion.update(scan(seq), .01, (seq-1)*.1, control(seq))
            self.assertEqual(result['motor'], 1500)
        self.assertEqual(result['steering_settle_elapsed_s'], 0)

    def test_same_adoption_tick_cannot_mature_settle(self):
        motion = MODULE.TurnMotion(0.)
        result = {'servo': 1500}
        for seq in range(1, 48):
            result = motion.update(scan(seq), .01, (seq-1)*.1,
                                   control(1, result['servo'], tick=100))
            self.assertEqual(result['motor'], 1500)
        self.assertLessEqual(result['steering_settle_feedback_ticks'], 1)

    def test_repeated_frames_do_not_ramp_or_refresh_observation(self):
        motion = MODULE.TurnMotion(0.)
        value = scan(1)
        first = motion.update(value, .01, 0., control(1))
        repeated = motion.update(value, .01, .2, control(2, first['servo']))
        self.assertEqual(repeated['servo'], first['servo'])
        self.assertEqual(repeated['steering_settle_elapsed_s'], 0)
        stale = motion.update(value, .01, .3, control(3, first['servo']))
        self.assertEqual(stale['terminal_reason'], 'turn_scan_not_advancing')
        self.assertEqual(stale['motor'], 1500)

    def test_scan_content_at_same_identity_cannot_change(self):
        motion = MODULE.TurnMotion(0.)
        motion.update(scan(1), .01, 0., control(1))
        result = motion.update(scan(1, heading=61.), .01, .1, control(2))
        self.assertEqual(result['terminal_reason'], 'duplicate_turn_scan_changed_content')

    def test_unknown_initial_left_target_refuses_start(self):
        for value in [dict(scan(1), corridor_candidates=[]), scan(1, heading=0.)]:
            with self.subTest(value=value):
                result = MODULE.TurnMotion(0.).update(value, .01, 0., control(1, armed=False))
                self.assertFalse(result['start_ready'])
                self.assertEqual(result['motor'], 1500)
                self.assertEqual(result['terminal_reason'], 'left_corridor_unknown')

    def test_ambiguous_initial_or_tracked_candidates_never_drive(self):
        for started in [False, True]:
            with self.subTest(started=started):
                motion = MODULE.TurnMotion(0.)
                if started:
                    motion.update(scan(1), .01, 0., control(1))
                value = scan(2 if started else 1)
                value['corridor_candidates'].append(corridor(62.))
                result = motion.update(value, .01, .1 if started else 0., control(2))
                self.assertEqual(result['terminal_reason'], 'left_corridor_ambiguous')
                self.assertEqual(result['motor'], 1500)

    def test_no_fixed_90_degree_target_and_unobserved_origin_rejected(self):
        for heading in [20., 45., 115.]:
            result = MODULE.TurnMotion(0.).update(scan(1, heading), .01, 0., control(1, armed=False))
            self.assertTrue(result['start_ready'])
            self.assertAlmostEqual(result['corridor']['heading_left_rad'], math.radians(heading))
        value = scan(1)
        value['corridor_candidates'][0]['origin_between_walls'] = False
        result = MODULE.TurnMotion(0.).update(value, .01, 0., control(1))
        self.assertEqual(result['terminal_reason'], 'invalid_native_corridor_candidates')

    def test_stale_reordered_jump_and_unknown_lock_immediately(self):
        for fault in ['stale', 'reordered', 'jump', 'publication']:
            with self.subTest(fault=fault):
                motion, seq, now, old, _ = self.drive()
                value = scan(seq+1)
                age = .3 if fault == 'stale' else .01
                if fault == 'reordered': value = scan(seq-1)
                if fault == 'jump': value = scan(seq+1, heading=120.)
                if fault == 'publication': value['at_ms'] = seq*100
                result = motion.update(value, age, now+.1, control(seq+1, old['servo'], motor=1560))
                self.assertTrue(result['lock_requested'])
                self.assertEqual((result['motor'], result['servo']), (1500, 1500))

    def test_geometry_empty_grace_keeps_old_target_without_renewing_evidence(self):
        motion, seq, now, previous, _ = self.drive()
        source_seq, source_at = previous['geometry_source_seq'], previous['geometry_source_at_ms']
        target = previous['steering_target']
        for i in range(1, 4):
            result = motion.update(dict(scan(seq+i), corridor_candidates=[]), .01, now+i*.1,
                                   control(seq+i, previous['servo'], motor=1560))
            self.assertFalse(result['lock_requested'])
            self.assertEqual(result['steering_target'], target)
            self.assertEqual(result['geometry_source_seq'], source_seq)
            self.assertEqual(result['geometry_source_at_ms'], source_at)
            self.assertEqual(result['alignment_confirmations'], 0)
            self.assertTrue(result['geometry_missing'])
            previous = result
        result = motion.update(dict(scan(seq+4), corridor_candidates=[]), .01, now+.4,
                               control(seq+4, previous['servo'], motor=1560))
        self.assertEqual(result['terminal_reason'], 'left_turn_geometry_lost')
        self.assertEqual(result['motor'], 1500)

    def test_unacknowledged_servo_command_cannot_start_settle_or_drive(self):
        motion = MODULE.TurnMotion(0.)
        previous = {'servo': 1500}
        for seq in range(1, 48):
            feedback = dict(control(seq, previous['servo']), command_acked=False)
            previous = motion.update(scan(seq), .01, (seq-1)*.1, feedback)
            self.assertEqual(previous['motor'], 1500)
            self.assertEqual(previous['steering_settle_feedback_ticks'], 0)

    def test_actual_ray_goal_allows_origin_before_exit_without_fixed_right_angle(self):
        motion = MODULE.TurnMotion(0.)
        result = motion.update(opening_scan(1), .01, 0., control(1, armed=False))
        self.assertTrue(result['start_ready'])
        self.assertEqual(result['geometry_mode'], 'opening_wall')
        self.assertFalse(result['corridor']['origin_between_walls'])
        self.assertAlmostEqual(result['corridor']['heading_left_rad'], math.radians(60.))
        self.assertEqual((result['motor'], result['servo']), (1500, 1500))

    def test_outer_wall_reprojects_target_and_handover_requires_real_double_walls(self):
        motion = MODULE.TurnMotion(0.)
        previous = {'servo': 1500}
        for seq in range(1, 49):
            now = (seq-1)*.1
            previous = motion.update(opening_scan(seq), .01, now, control(seq, previous['servo']))
            if previous['phase'] == 'drive': break
        self.assertEqual(previous['phase'], 'drive')
        old_point = previous['turn_goal']['target_point_left_m']
        for i in range(1, 13):
            heading, rho = 60.-i*5, -1.8+i*.1
            value = dict(scan(seq+i), corridor_candidates=[], wall_candidates=[wall(heading, rho)], left_turn_goal=None)
            if i == 12:
                value['corridor_candidates'] = [corridor(0., 0., 1.2)]
            previous = motion.update(value, .01, now+i*.1, control(seq+i, previous['servo'], motor=1560))
            self.assertFalse(previous['lock_requested'])
            if i < 12:
                self.assertEqual(previous['geometry_mode'], 'opening_wall')
                self.assertFalse(previous['observed_alignment'])
        self.assertNotEqual(previous['turn_goal']['target_point_left_m'], old_point)
        self.assertEqual(previous['geometry_mode'], 'corridor')
        for i in range(13, 16):
            previous = motion.update(scan(seq+i, heading=0.), .01, now+i*.1,
                                     control(seq+i, previous['servo'], motor=1560))
        self.assertTrue(previous['observed_alignment'])
        self.assertTrue(previous['test_sequence_finished'])
        self.assertFalse(previous['completed'])

    def test_overlapping_wall_fits_are_one_identity_but_different_parallel_walls_are_not(self):
        for ambiguous in [False, True]:
            with self.subTest(ambiguous=ambiguous):
                motion = MODULE.TurnMotion(0.)
                motion.update(opening_scan(1), .01, 0., control(1))
                second = dict(opening_scan(2), left_turn_goal=None)
                second['wall_candidates'].append(wall(60.1, -1.7 if ambiguous else -1.799, 1.2, 2.6))
                result = motion.update(second, .01, .1, control(2))
                self.assertEqual(result['lock_requested'], ambiguous)

    def test_turn_goal_without_real_support_ray_or_inconsistent_reference_refuses(self):
        for fault in ['ray', 'mirror', 'line', 'flag']:
            with self.subTest(fault=fault):
                value = opening_scan(1)
                if fault == 'ray': value['ranges'][330] = None
                if fault == 'mirror': value['left_turn_goal']['target_point_left_m']['y_m'] *= -1
                if fault == 'line': value['left_turn_goal']['center_offset_left_m'] += .3
                if fault == 'flag': value['left_turn_goal']['turn_path_certified'] = True
                result = MODULE.TurnMotion(0.).update(value, .01, 0., control(1, armed=False))
                self.assertFalse(result['start_ready'])
                self.assertTrue(result['lock_requested'])

    def test_target_only_manual_or_simulated_source_is_accepted_without_fake_pose(self):
        for source in ['manual', 'simulated']:
            record = {'schema_version': 1, 'goal_id': 'trial-1', 'source_kind': source, 'goal_type': 'point_stop',
                      'frame': 'run_start_lidar_reference', 'coordinate_convention': 'x_forward_y_left_yaw_left_positive',
                      'max_seconds': 10., 'target_point_left_m': {'x_m': 2., 'y_m': 1.}}
            result = MODULE.parse_trial_goal(record)
            self.assertTrue(result['target_only'])
            self.assertEqual(result['execution_rejection'], 'real_pose_missing')
            self.assertFalse(result['physical_control_ready'])
            for field in ['sim_pose', 'vehicle_pose', 'source_pose', 'speed_mps']:
                with self.assertRaises(ValueError): MODULE.parse_trial_goal(dict(record, **{field: 0}))
        record['goal_type'] = 'turn_exit_align'
        self.assertEqual(MODULE.parse_trial_goal(record)['execution_rejection'], 'native_left_turn_geometry_required')
        for change in [{'max_seconds': 11}, {'frame': 'world'}, {'coordinate_convention': 'x_forward_y_right'}]:
            with self.assertRaises(ValueError): MODULE.parse_trial_goal(dict(record, **change))

    def test_local_and_publication_epochs_are_independent(self):
        motion = MODULE.TurnMotion(900.)
        a = motion.update(scan(1, published=123456000), .01, 900., control(1))
        b = motion.update(scan(2, published=123456100), .01, 900.1, control(2, a['servo']))
        self.assertFalse(b['lock_requested'])
        self.assertEqual(b['servo']-a['servo'], 10)

    def test_pwm_interval_does_not_catch_up_after_pause(self):
        motion = MODULE.TurnMotion(0.)
        first = motion.update(scan(1), .01, 0., control(1))
        value = scan(2, published=150)
        short = motion.update(value, .01, .05, control(2, first['servo']))
        self.assertEqual(short['servo'], first['servo'])
        third = motion.update(scan(3, published=300), .01, .2, control(3, first['servo']))
        self.assertEqual(third['servo']-first['servo'], 10)

    def test_smaller_heading_error_and_trend_release_old_turn(self):
        motion, seq, now, previous, _ = self.drive()
        original = previous['servo']
        for i, heading in enumerate([55., 50., 45., 40., 35., 30.], start=1):
            result = motion.update(scan(seq+i, heading), .01, now+i*.1,
                                   control(seq+i, previous['servo'], motor=1560))
            self.assertLessEqual(abs(result['servo']-previous['servo']), 10)
            self.assertGreaterEqual(result['servo'], 1500)
            previous = result
        self.assertLess(result['servo'], original)
        self.assertLess(result['steering_target'], 1500+round(140*math.radians(30)))

    def test_alignment_requires_distinct_source_and_receive_time(self):
        motion, seq, now, previous, _ = self.drive(heading=20.)
        for i, heading in enumerate([12., 4., 3., 2., 1.], start=1):
            previous = motion.update(scan(seq+i, heading), .01, now+i*.1,
                                     control(seq+i, previous['servo'], motor=1560))
        self.assertTrue(previous['observed_alignment'])
        self.assertTrue(previous['request_coast'])
        self.assertEqual(previous['alignment_evidence'], 'double_wall')
        self.assertFalse(previous['entry_confirmed'])
        self.assertFalse(previous['completed'])
        self.assertEqual(previous['motor'], 1500)

    def test_coast_latches_neutral_even_after_new_targets_and_time(self):
        motion, seq, now, previous, _ = self.drive()
        motion.begin_coast('test_stop', now)
        for i in range(1, 30):
            result = motion.update(scan(seq+i, heading=60.), .01, now+i*.1,
                                   control(seq+i, previous['servo'], motor=1500))
            self.assertEqual(result['motor'], 1500)
            self.assertEqual(result['phase'], 'coast')
            self.assertLessEqual(abs(result['servo']-previous['servo']), 10)
            previous = result
        self.assertEqual(previous['servo'], 1500)

    def test_coast_consumes_fresh_geometry_for_bounded_correction_without_drive(self):
        motion, seq, now, previous, _ = self.drive(heading=20.)
        motion.begin_coast('planned', now)
        targets = []
        for i in range(1, 26):
            heading, offset = (15., .12) if i == 1 else (10., -.12)
            result = motion.update(scan(seq+i, heading, offset), .01, now+i*.1,
                                   control(seq+i, previous['servo'], motor=1500))
            self.assertEqual((result['phase'], result['motor']), ('coast', 1500))
            self.assertEqual(result['geometry_source_seq'], seq+i)
            self.assertLessEqual(abs(result['servo']-previous['servo']), 10)
            self.assertTrue(1445 <= result['steering_target'] <= 1555)
            self.assertFalse(result['completed'])
            targets.append(result['steering_target'])
            previous = result
        self.assertGreater(targets[0], 1500)
        self.assertLess(targets[1], 1500)
        self.assertEqual(previous['servo'], previous['steering_target'])

    def test_coast_missing_geometry_slews_neutral_without_reusing_old_geometry(self):
        motion, seq, now, previous, _ = self.drive()
        source_seq = previous['geometry_source_seq']
        motion.begin_coast('planned', now)
        for i in range(1, 41):
            result = motion.update(dict(scan(seq+i), corridor_candidates=[]), .01, now+i*.1,
                                   control(seq+i, previous['servo'], motor=1500))
            self.assertEqual((result['phase'], result['motor']), ('coast', 1500))
            self.assertEqual(result['steering_target'], 1500)
            self.assertEqual(result['terminal_reason'], 'planned')
            self.assertEqual(result['geometry_source_seq'], source_seq)
            self.assertTrue(result['geometry_missing'])
            self.assertLessEqual(abs(result['servo']-previous['servo']), 10)
            previous = result
        self.assertEqual(previous['servo'], 1500)

    def test_coast_faults_including_duplicate_scan_feedback_lock_immediately(self):
        for fault in ['stale', 'changed', 'reordered_scan', 'clock_gap', 'unarmed', 'old_tick', 'old_seq', 'malformed']:
            with self.subTest(fault=fault):
                motion, seq, now, previous, _ = self.drive()
                motion.begin_coast('planned', now)
                value, age, received = scan(seq), .01, now+.1
                feedback = control(seq+1, previous['servo'], motor=1500)
                if fault == 'stale': age = .3
                if fault == 'changed': value = scan(seq, heading=61.)
                if fault == 'reordered_scan': value = scan(seq-1)
                if fault == 'clock_gap': value, received = scan(seq+4), now+.4
                if fault == 'unarmed': feedback['armed'] = False
                if fault == 'old_tick': feedback['tick'] = 0
                if fault == 'old_seq': feedback['seq'] = 0
                if fault == 'malformed': feedback['servo'] = True
                result = motion.update(value, age, received, feedback)
                self.assertTrue(result['lock_requested'])
                self.assertEqual((result['motor'], result['servo']), (1500, 1500))
                returned = motion.update(scan(seq+5), .01, now+.5, control(seq+5))
                self.assertEqual(returned['phase'], 'locked')

    def test_tracked_wall_behind_car_does_not_restore_left_and_safe_exit_can_align(self):
        motion = MODULE.TurnMotion(0.)
        previous = {'servo': 1500}
        for seq in range(1, 49):
            now = (seq-1)*.1
            previous = motion.update(opening_scan(seq), .01, now, control(seq, previous['servo']))
            if previous['phase'] == 'drive': break
        for i in range(1, 17):
            heading = max(0., 60.-i*5)
            rho = -1.8+min(i, 12)*.1+.12
            support = wall(heading, rho, -3., -1.) if i >= 12 else wall(heading, rho)
            value = dict(scan(seq+i), corridor_candidates=[], wall_candidates=[support], left_turn_goal=None)
            result = motion.update(value, .01, now+i*.1, control(seq+i, previous['servo'], motor=1560))
            self.assertFalse(result['lock_requested'])
            if i >= 12:
                self.assertLessEqual(result['steering_target'], 1528)
                self.assertLessEqual(abs(result['servo']-previous['servo']), 10)
            previous = result
        self.assertTrue(previous['observed_alignment'])
        self.assertEqual(previous['alignment_evidence'], 'tracked_goal_wall')
        self.assertEqual(previous['geometry_mode'], 'opening_wall')
        self.assertAlmostEqual(previous['corridor']['center_offset_left_m'], .12)
        self.assertFalse(previous['entry_confirmed'])
        self.assertFalse(previous['completed'])

    def test_drive_deadline_uses_trial_time_and_cannot_extend(self):
        motion, seq, now, previous, _ = self.drive(max_drive_s=.2)
        for i in range(1, 4):
            result = motion.update(scan(seq+i), .01, now+i*.1, control(seq+i, previous['servo']))
            previous = result
        self.assertEqual(result['terminal_reason'], 'left_turn_drive_timeout')
        self.assertTrue(result['request_coast'])
        self.assertEqual(result['motor'], 1500)

    def test_stale_scan_at_drive_deadline_still_locks_immediately(self):
        motion, seq, now, previous, _ = self.drive(max_drive_s=.1)
        result = motion.update(scan(seq+1), .3, now+.1, control(seq+1, previous['servo']))
        self.assertTrue(result['lock_requested'])
        self.assertEqual((result['motor'], result['servo']), (1500, 1500))

    def test_presteer_deadline_cannot_wait_forever(self):
        motion = MODULE.TurnMotion(0.)
        result = motion.update(scan(1), .01, 5., control(1))
        self.assertEqual(result['terminal_reason'], 'left_turn_presteer_timeout')

    def test_close_obstacle_overrides_presteer_drive_and_coast(self):
        for phase in ['presteer', 'drive', 'coast']:
            with self.subTest(phase=phase):
                if phase == 'presteer':
                    motion, seq, now, previous = MODULE.TurnMotion(0.), 0, 0., {'servo': 1500}
                else:
                    motion, seq, now, previous, _ = self.drive()
                    if phase == 'coast': motion.begin_coast('planned', now)
                result = motion.update(scan(seq+1), .01, now+.1, control(seq+1, previous['servo']), safe=False)
                self.assertTrue(result['lock_requested'])
                self.assertEqual((result['motor'], result['servo']), (1500, 1500))

    def test_entry_evidence_must_match_fresh_native_scan_and_still_is_trial(self):
        for valid in [True, False]:
            with self.subTest(valid=valid):
                motion, seq, now, previous, _ = self.drive()
                evidence = {'kind': 'cone', 'source_seq': seq+1 if valid else seq,
                            'source_at_ms': (seq+1)*100, 'distance_forward_m': 1.2,
                            'error_m': .04, 'candidate_only': True}
                result = motion.update(scan(seq+1), .01, now+.1,
                                       control(seq+1, previous['servo']), entry_stop=evidence)
                self.assertEqual(result['entry_confirmed'], valid)
                self.assertEqual(result['request_coast'], valid)
                self.assertEqual(result['lock_requested'], not valid)
                self.assertFalse(result['completed'])
                self.assertEqual(result['motor'], 1500)

    def test_fault_never_restarts_when_geometry_returns(self):
        motion = MODULE.TurnMotion(0.)
        motion.update(dict(scan(1), corridor_candidates=[]), .01, 0., control(1))
        for seq in range(2, 10):
            result = motion.update(scan(seq), .01, seq*.1, control(seq))
            self.assertEqual(result['phase'], 'locked')
            self.assertEqual(result['motor'], 1500)

    def test_software_feedback_loss_or_reordering_latches_stop(self):
        for fault in ['unarmed', 'old_tick', 'old_seq', 'malformed']:
            with self.subTest(fault=fault):
                motion, seq, now, previous, _ = self.drive()
                feedback = control(seq+1, previous['servo'], motor=1560)
                if fault == 'unarmed': feedback['armed'] = False
                if fault == 'old_tick': feedback['tick'] = 0
                if fault == 'old_seq': feedback['seq'] = 0
                if fault == 'malformed': feedback['servo'] = True
                result = motion.update(scan(seq+1), .01, now+.1, feedback)
                self.assertTrue(result['lock_requested'])
                self.assertEqual((result['motor'], result['servo']), (1500, 1500))

    def test_invalid_configuration_and_source_finite_values(self):
        for kwargs in [{'started_at': False}, {'started_at': 0, 'max_drive_s': 11},
                       {'started_at': 0, 'motor_pwm': 1580}, {'started_at': 0, 'max_presteer_s': 6}]:
            with self.assertRaises(ValueError): MODULE.TurnMotion(**kwargs)
        for bad in [math.nan, math.inf, True]:
            value = scan(1)
            value['corridor_candidates'][0]['heading_left_rad'] = bad
            result = MODULE.TurnMotion(0.).update(value, .01, 0., control(1))
            self.assertTrue(result['lock_requested'])


if __name__ == '__main__':
    unittest.main()
