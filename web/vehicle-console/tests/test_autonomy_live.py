"""No physical devices: clearance faults and session fencing regression checks."""
import importlib.util
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location('autonomy_live', Path(__file__).parents[1] / 'autonomy_live.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def wall_scan(offset=0, slope=0, width=1., length=7.):
    bins = []
    for angle in range(360):
        sine, cosine = math.sin(math.radians(angle)), math.cos(math.radians(angle))
        denominator = sine-slope*cosine
        distances = [length/abs(cosine)] if abs(cosine) > 1e-6 else []
        if abs(denominator) > 1e-6:
            distances.extend(b/denominator for b in (offset-width/2, offset+width/2) if b/denominator > 0)
        bins.append(min(distances))
    return {'seq': 1, 'frame_id': 'lidar_origin_coarse_body_heading', 'ranges': bins}


def junction_scan(seq, end=.3):
    return {'seq': seq, 'ranges': [3.]*360, 'front_boundary_m': end+1.5, 'left_junction': {
        'front_wall_m': end+1.5, 'incoming_left_end_m': end,
        'incoming_width_m': 1.2, 'outgoing_width_m': 1.5,
        'heading_left_rad': 0., 'known_open_fraction': 1., 'turn_path_certified': False}}


class ApproachRampTests(unittest.TestCase):
    def test_progressively_reduces_on_fresh_plane_and_never_reaccelerates(self):
        ramp = MODULE.ApproachRamp(1570)
        values = []
        for i in range(20):
            scan = {'seq': i, 'front_boundary_m': 3.5}
            values.append(ramp.update(scan, .01, i*.16))
        self.assertEqual(values[0], 1565)
        self.assertEqual(values[-1], 1550)
        self.assertTrue(all(0 <= a-b <= 5 for a, b in zip(values, values[1:])))
        self.assertEqual(ramp.update({'seq': 20, 'front_boundary_m': 4.}, .01, 4), 1550)
        self.assertEqual(ramp.update({'seq': 21, 'front_boundary_m': None}, .01, 5), 1550)
        self.assertTrue(ramp.stop_requested)
        self.assertEqual(MODULE.ApproachRamp(1560).pwm, 1560)

    def test_duplicate_stale_and_missing_plane_do_not_lower_throttle(self):
        ramp = MODULE.ApproachRamp(1560)
        self.assertEqual(ramp.update({'seq': 1, 'front_boundary_m': 2.}, .3, 0), 1560)
        self.assertFalse(ramp.stop_requested)
        self.assertEqual(ramp.update({'seq': 1, 'front_boundary_m': 3.5}, .01, 1), 1555)
        self.assertEqual(ramp.update({'seq': 1, 'front_boundary_m': 1.}, .01, 2), 1555)
        self.assertFalse(ramp.stop_requested)
        self.assertEqual(ramp.update({'seq': 2, 'front_boundary_m': 3.5}, .01, 1.05), 1555)

    def test_lane_correction_reduces_then_release_regains_cruise_before_boundary(self):
        ramp = MODULE.ApproachRamp(1560)
        scan = {'seq': 1, 'front_boundary_m': None}
        self.assertEqual(ramp.update(scan, .3, 0, True), 1560)
        self.assertFalse(ramp.lane_limited)
        self.assertEqual(ramp.update(scan, .01, .1, True), 1560)
        self.assertEqual(ramp.update(scan, .01, .5, True), 1560)
        for i in range(2, 10):
            self.assertGreaterEqual(ramp.update(dict(scan, seq=i), .01, i*.16, True), 1550)
        self.assertEqual(ramp.pwm, 1550)
        self.assertIsNone(ramp.closest_front)
        for i in range(10, 20):
            before = ramp.pwm
            ramp.update(dict(scan, seq=i), .01, 2+(i-10)*.21, False)
            self.assertTrue(0 <= ramp.pwm-before <= 2)
        self.assertEqual(ramp.pwm, 1560)
        low = MODULE.ApproachRamp(1550)
        self.assertEqual(low.update(scan, .01, 0, True), 1550)

    def test_front_closing_time_requires_fresh_distinct_consistent_evidence(self):
        ramp = MODULE.ApproachRamp(1560)
        scan = {'seq': 1, 'front_boundary_m': 2.7}
        ramp.update(scan, .3, 0)
        self.assertFalse(ramp.stop_requested)
        ramp.update({'seq': 1, 'front_boundary_m': None}, .01, .1)
        ramp.update(scan, .01, .2)
        self.assertFalse(ramp.stop_requested)
        ramp.update(dict(scan, seq=2), .01, .3)
        self.assertFalse(ramp.stop_requested)
        ramp.update({'seq': 3, 'front_boundary_m': 2.6}, .01, .4)
        ramp.update({'seq': 4, 'front_boundary_m': 2.5}, .01, .5)
        self.assertTrue(ramp.stop_requested)
        ramp.update({'seq': 5, 'front_boundary_m': None}, .01, .6)
        self.assertTrue(ramp.stop_requested)

    def test_same_distance_has_different_stop_decision_at_different_closing_rates(self):
        fast, slow = MODULE.ApproachRamp(1560), MODULE.ApproachRamp(1560)
        for seq, (df, ds) in enumerate(zip([3.8, 3.6, 3.4], [3.5, 3.45, 3.4])):
            fast.update({'seq': seq, 'front_boundary_m': df}, .01, seq*.2)
            slow.update({'seq': seq, 'front_boundary_m': ds}, .01, seq*.2)
        self.assertTrue(fast.stop_requested)
        self.assertFalse(slow.stop_requested)
        self.assertAlmostEqual(fast.closing_speed, 1.)
        self.assertAlmostEqual(slow.closing_speed, .25)

    def test_capture_clock_not_server_delay_sets_closing_rate(self):
        ramp = MODULE.ApproachRamp(1560)
        for seq, (d, now) in enumerate(zip([3.6, 3.5, 3.4], [10., 10.15, 10.27])):
            ramp.update({'seq': seq, 'at_ms': 1000+seq*100, 'front_boundary_m': d}, .01, now)
        self.assertAlmostEqual(ramp.closing_speed, 1.)
        self.assertTrue(ramp.stop_requested)

    def test_launch_cruise_and_recovery_do_not_depend_on_course_length(self):
        ramp = MODULE.ApproachRamp(1580)
        for seq in range(8):
            ramp.update({'seq': seq}, .01, seq*.1, moving=False)
        self.assertIsNone(ramp.launch_since)
        self.assertEqual(ramp.pwm, 1580)
        for seq in range(8, 28):
            ramp.update({'seq': seq}, .01, seq*.11)
        self.assertTrue(ramp.cruise_limited)
        self.assertEqual(ramp.pwm, 1570)

    def test_plane_jump_and_long_gap_do_not_create_a_closing_speed(self):
        ramp = MODULE.ApproachRamp(1560)
        for seq, d in enumerate([4., 3.9, 1.5]):
            ramp.update({'seq': seq, 'front_boundary_m': d}, .01, seq*.1)
        self.assertIsNone(ramp.closing_speed)
        self.assertIsNone(ramp.time_to_clearance)
        ramp.update({'seq': 3, 'front_boundary_m': None}, .01, .7)
        self.assertTrue(ramp.boundary_lost)
        before = ramp.pwm
        for seq in range(4, 20):
            ramp.update({'seq': seq}, .01, .7+seq*.11)
        self.assertEqual(ramp.pwm, before)
        self.assertGreaterEqual(ramp.pwm, 1550)
        self.assertTrue(ramp.stop_requested)

    def test_new_forward_bounds_and_near_boundary_never_emit_subfloor_pwm(self):
        for value in [1549, 1581, True]:
            with self.assertRaises(ValueError): MODULE.ApproachRamp(value)
        ramp = MODULE.ApproachRamp(1580)
        for seq in range(12):
            value = ramp.update({'seq': seq, 'front_boundary_m': 3.8-.1*seq}, .01, seq*.1)
            self.assertTrue(1550 <= value <= 1580)
        self.assertTrue(ramp.stop_requested)


class JunctionStopTests(unittest.TestCase):
    def test_three_distinct_fresh_scans_and_only_at_endpoint(self):
        gate = MODULE.JunctionStop()
        self.assertFalse(gate.update(junction_scan(1, .7), .01, 0, True))
        self.assertFalse(gate.update(junction_scan(1, .7), .01, .02, True))
        self.assertEqual(gate.count, 1)
        self.assertFalse(gate.update(junction_scan(2, .6), .01, .1, True))
        self.assertFalse(gate.update(junction_scan(3, .5), .01, .2, True))
        self.assertTrue(gate.update(junction_scan(4, .4), .01, .3, True))
        self.assertFalse(MODULE.JunctionStop().update(junction_scan(5), .01, .4, True))

    def test_stale_unknown_quality_and_geometry_jumps_reset_votes(self):
        for invalid in ['age', 'unknown', 'quality', 'jump', 'gap', 'fraction']:
            gate = MODULE.JunctionStop()
            for seq in [1, 2]: gate.update(junction_scan(seq), .01, seq*.1, True)
            scan = junction_scan(3)
            age, now, ready = .01, .3, True
            if invalid == 'age': age = .3
            if invalid == 'unknown': scan['left_junction'] = None
            if invalid == 'quality': ready = False
            if invalid == 'jump': scan['left_junction']['front_wall_m'] = 2.6
            if invalid == 'gap': now = .6
            if invalid == 'fraction': scan['left_junction']['known_open_fraction'] = 1.1
            self.assertFalse(gate.update(scan, age, now, ready), invalid)
            self.assertLess(gate.count, 2, invalid)


class CorridorSteeringTests(unittest.TestCase):
    def test_cli_front_margin_is_incomplete_and_never_rearms(self):
        spec = importlib.util.spec_from_file_location('autonomy_cli_margin', Path(__file__).parents[1]/'autonomy-control.py')
        cli = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {'autonomy_live': MODULE}):
            spec.loader.exec_module(cli)
        state = {'boot': 'boot', 'autonomy': {'epoch': 0}, 'status': {'control': {'tick': 100}}}
        calls = []
        def request(path, data=None):
            if data:
                calls.append(data['op'])
                if data['op'] == 'to_left_junction_start': return {'run_id': 'run', 'epoch': 0}
                return {'ok': True}
            return {'autonomy': {'active': None, 'mode': 'locked',
                    'last_result': {'run_id': 'run', 'completed': False, 'reason': 'front_boundary_stop'}},
                    'status': {'control': {'armed': False, 'motor': 1500, 'servo': 1500}}}
        result = cli.straight_segment(request, state, 1560, 30, stop_left_junction=True)
        self.assertFalse(result['completed'])
        self.assertEqual(result['reason'], 'front_boundary_stop')
        self.assertEqual(calls, ['to_left_junction_start', 'cancel'])

    def test_cli_normal_deadline_racing_heartbeat_is_not_failure(self):
        spec = importlib.util.spec_from_file_location('autonomy_cli_test', Path(__file__).parents[1]/'autonomy-control.py')
        cli = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {'autonomy_live': MODULE}):
            spec.loader.exec_module(cli)
        state = {'boot': 'boot', 'autonomy': {'epoch': 0}, 'status': {'control': {'tick': 100}}}
        done = False
        def request(path, data=None):
            nonlocal done
            if data and data['op'] == 'probe_start': return {'run_id': 'run', 'epoch': 0}
            if data:
                done = True
                raise RuntimeError('stale_autonomy_generation')
            return {'autonomy': {'active': None if done else {'run_id': 'run'}, 'mode': 'locked' if done else 'auto_probe',
                                 'last_result': {'run_id': 'run', 'completed': True, 'reason': 'probe_complete'} if done else None},
                    'status': {'control': {'armed': not done, 'motor': 1500, 'servo': 1500}}}
        with patch.object(cli.time, 'sleep'):
            latest, run = cli.run_probe(request, state, 1560, 400)
        self.assertTrue(latest['autonomy']['last_result']['completed'])
        self.assertEqual(run['run_id'], 'run')

    def test_wall_geometry_direction_and_deadband(self):
        walls = MODULE.corridor_walls(wall_scan(.12, .02))
        self.assertAlmostEqual(walls['offset_right_m'], .12)
        steering = MODULE.CorridorSteering()
        self.assertEqual(steering.update(walls, 1, 0), 1480)
        self.assertEqual(steering.update(walls, 2, .14), 1480)
        self.assertLess(steering.update(walls, 3, .16), 1480)
        for i in range(20):
            value = steering.update(walls, i+4, .72+i*.36)
            self.assertGreaterEqual(value, 1445)
        centred = MODULE.CorridorSteering()
        for i in range(50):
            walls = MODULE.corridor_walls(wall_scan(.06+(.002 if i%2 else -.002), .01))
            self.assertEqual(centred.update(walls, i, i*.05), 1500)
        left = MODULE.CorridorSteering()
        self.assertEqual(left.update(MODULE.corridor_walls(wall_scan(-.12)), 1, 0), 1520)

    def test_hysteresis_no_same_frame_renewal_and_bad_wall_fit(self):
        steering = MODULE.CorridorSteering()
        walls = MODULE.corridor_walls(wall_scan(.12))
        self.assertEqual(steering.update(walls, 1, 0), 1480)
        self.assertEqual(steering.update(walls, 1, 1), 1480)
        middle = MODULE.corridor_walls(wall_scan(.06))
        steering.update(middle, 2, 1)
        self.assertTrue(steering.correcting)
        steering.update(MODULE.corridor_walls(wall_scan(.03)), 3, 1.4)
        self.assertFalse(steering.correcting)
        self.assertEqual(steering.update(None, 4, 1.5), 1500)
        self.assertEqual(steering.update(walls, 5, 2, False), 1500)
        broken = wall_scan(.12)
        for angle in range(60, 121): broken['ranges'][angle] = .7
        self.assertIsNone(MODULE.corridor_walls(broken))

    def test_heading_predicts_drift_before_old_threshold_without_chattering(self):
        steering = MODULE.CorridorSteering()
        walls = MODULE.corridor_walls(wall_scan(-.003, math.tan(math.radians(-2.9))))
        self.assertLess(abs(walls['offset_right_m']), .08)
        self.assertLess(abs(walls['heading_right_deg']), 4)
        self.assertGreater(steering.update(walls, 1, 0), 1500)
        self.assertLessEqual(steering.servo, 1515)
        for i in range(2, 8):
            walls = MODULE.corridor_walls(wall_scan(-.025, math.tan(math.radians(-1.8))))
            steering.update(walls, i, i*.26)
            self.assertTrue(steering.correcting)
        steering.update(MODULE.corridor_walls(wall_scan(-.005, -.01)), 8, 3)
        self.assertFalse(steering.correcting)

    def test_side_threshold_uses_body_extent_plus_net_and_range_allowance(self):
        self.assertAlmostEqual(MODULE.SIDE_CLEARANCE_M, .30)
        scan = wall_scan()
        scan['ranges'][90] = .299
        with self.assertRaisesRegex(ValueError, 'close_side'):
            MODULE.probe_clearance(scan, {'camera': .01, 'lidar': .01, 'control': .01})
        scan['ranges'][90] = .301
        result = MODULE.probe_clearance(scan, {'camera': .01, 'lidar': .01, 'control': .01})
        self.assertEqual(result['side_body_extent_m'], .17)
        self.assertEqual(result['side_min_net_m'], .1)

    def test_real_close_left_wall_and_converging_boards_keep_correction(self):
        scan = {'seq': 1, 'ranges': json.loads((Path(__file__).parent/'observed-near-left-20261005.json').read_text())}
        walls = MODULE.corridor_walls(scan)
        self.assertIsNotNone(walls)
        self.assertGreater(walls['offset_right_m'], .2)
        steering = MODULE.CorridorSteering()
        values = [steering.update(walls, i, i*.26) for i in range(6)]
        self.assertEqual(values[0], 1480)
        self.assertEqual(values[-1], 1445)
        self.assertTrue(all(1445 <= value < 1500 for value in values))

    def test_same_error_uses_available_width_and_has_no_left_or_right_bias(self):
        values = []
        for width in [.8, 1., 1.5, 2.]:
            right, left = MODULE.CorridorSteering(), MODULE.CorridorSteering()
            wr = MODULE.corridor_walls(wall_scan(.12, .03, width))
            wl = MODULE.corridor_walls(wall_scan(-.12, -.03, width))
            for seq in range(8):
                right.update(wr, seq, seq*.26)
                left.update(wl, seq, seq*.26)
            self.assertEqual(right.servo+left.servo, 3000)
            values.append(1500-right.servo)
        self.assertGreater(values[0], values[-1])
        self.assertTrue(all(a >= b for a,b in zip(values, values[1:])))

    def test_reversal_and_return_to_centre_use_the_same_bounded_slew(self):
        steering = MODULE.CorridorSteering()
        right = MODULE.corridor_walls(wall_scan(.12))
        left = MODULE.corridor_walls(wall_scan(-.12))
        self.assertEqual(steering.update(right, 1, 0), 1480)
        self.assertLess(steering.update(right, 2, .16), 1480)
        last = steering.servo
        for seq in range(3, 6):
            value = steering.update(left, seq, (seq-1)*.16)
            self.assertTrue(0 <= value-last <= 20)
            self.assertTrue(1445 <= value <= 1555)
            last = value
        self.assertGreater(steering.servo, 1500)
        centre = MODULE.corridor_walls(wall_scan())
        for seq in range(6, 10):
            before = steering.servo
            value = steering.update(centre, seq, 1+(seq-6)*.26)
            self.assertTrue(0 <= before-value <= 15)
        self.assertEqual(steering.servo, 1500)

    def test_wall_control_does_not_depend_on_straight_length_or_turn_location(self):
        for width in [.8, 1., 1.5, 2.]:
            for heading in [-8., 0., 8.]:
                outputs = []
                for length in [3., 7., 12.]:
                    walls = MODULE.corridor_walls(wall_scan(.04, math.tan(math.radians(heading)), width, length))
                    steering = MODULE.CorridorSteering()
                    outputs.append([steering.update(walls, seq, seq*.26) for seq in range(5)])
                self.assertEqual(outputs[0], outputs[1])
                self.assertEqual(outputs[1], outputs[2])


class RearLaunchTests(unittest.TestCase):
    def test_rear_only_grace_never_suppresses_front_or_side_or_quality(self):
        scan = wall_scan()
        scan['ranges'][180] = .28
        scan['ranges'][135] = .28
        ages = {'camera': .01, 'lidar': .01, 'control': .01}
        with self.assertRaises(ValueError): MODULE.probe_clearance(scan, ages)
        self.assertTrue(MODULE.probe_clearance(scan, ages, rear_launch=True)['motion_ready'])
        for angle in [0, 90, 270]:
            old = scan['ranges'][angle];scan['ranges'][angle] = .2
            with self.assertRaises(ValueError): MODULE.probe_clearance(scan, ages, rear_launch=True)
            scan['ranges'][angle] = old
        self.assertFalse(MODULE.probe_clearance(scan, dict(ages, lidar=.3), rear_launch=True)['motion_ready'])

    def test_one_second_grace_and_no_reactivation(self):
        scan = wall_scan();scan['ranges'][180] = .28
        gate = MODULE.RearLaunch()
        self.assertTrue(gate.update(scan, 10, False))
        self.assertTrue(gate.update(scan, 12, True))
        self.assertTrue(gate.update(scan, 12.99, True))
        self.assertFalse(gate.update(scan, 13, True))
        self.assertFalse(gate.update(scan, 13.1, True))
        gate = MODULE.RearLaunch()
        self.assertFalse(gate.update(wall_scan(), 0, False))
        self.assertFalse(gate.update(scan, .1, True))


class ProbeClearanceTests(unittest.TestCase):
    def setUp(self):
        self.scan = {'seq': 1, 'frame_id': 'lidar_origin_coarse_body_heading', 'ranges': [3.] * 360}
        self.ages = {'camera': .01, 'lidar': .01, 'control': .01}

    def test_known_near_obstacle_still_blocks_motion(self):
        for index, value in [(359, .99), (270, .29), (25, .7)]:
            with self.subTest(index=index):
                self.scan['ranges'][index] = value
                with self.assertRaises(ValueError): MODULE.probe_clearance(self.scan, self.ages)
                self.scan['ranges'][index] = 3.

    def test_bounded_holes_are_kept_unknown_and_large_holes_pause(self):
        for angle in [5, 6, 7, 12, 13, 14]: self.scan['ranges'][angle] = None
        result = MODULE.probe_clearance(self.scan, self.ages)
        self.assertTrue(result['motion_ready'])
        self.assertEqual(result['front_unknown_bins'], [5, 6, 7, 12, 13, 14])
        self.assertIsNone(self.scan['ranges'][6])
        self.scan['ranges'][8] = None
        result = MODULE.probe_clearance(self.scan, self.ages)
        self.assertFalse(result['motion_ready'])
        self.assertIn('front_sparse', result['quality_issues'])

    def test_side_boards_do_not_block_the_straight_corridor(self):
        bins = []
        for angle in range(360):
            sine, cosine = math.sin(math.radians(angle)), math.cos(math.radians(angle))
            candidates = [7 / abs(cosine)] if abs(cosine) > 1e-6 else []
            if sine > 1e-6: candidates.append(.535 / sine)
            if sine < -1e-6: candidates.append(.407 / -sine)
            bins.append(min(candidates))
        self.scan['ranges'] = bins
        result = MODULE.probe_clearance(self.scan, self.ages)
        self.assertLess(result['front_m'], 1)
        self.assertGreater(result['corridor_front_m'], 1)
        self.assertTrue(result['motion_ready'])

    def test_frame_quality_debounces_but_invalid_age_is_hard(self):
        camera = MODULE.probe_clearance(self.scan, {**self.ages, 'camera': .6})
        self.assertTrue(camera['motion_ready'])
        self.assertIn('camera_late', camera['quality_issues'])
        laser = MODULE.probe_clearance(self.scan, {**self.ages, 'lidar': .35})
        self.assertFalse(laser['motion_ready'])
        self.assertIn('lidar_late', laser['quality_issues'])
        self.assertTrue(MODULE.probe_clearance(self.scan, {**self.ages, 'lidar': .29})['motion_ready'])
        for key in ['camera', 'lidar']:
            self.assertFalse(MODULE.probe_clearance(self.scan, {**self.ages, key: 1.5})['motion_ready'])

    def test_invalid_heading_stale_frames_and_nonfinite_values(self):
        self.scan['frame_id'] = 'unknown'
        with self.assertRaises(ValueError): MODULE.probe_clearance(self.scan, self.ages)
        self.scan['frame_id'] = 'lidar_origin_coarse_body_heading'
        self.assertTrue(MODULE.probe_clearance(self.scan, {**self.ages, 'control': .082})['motion_ready'])
        for key, value in [('control', .15), ('lidar', -1), ('lidar', float('nan'))]:
            with self.subTest(key=key, value=value):
                ages = {**self.ages, key: value}
                with self.assertRaises(ValueError): MODULE.probe_clearance(self.scan, ages)
        for value in [float('inf'), float('nan'), False, -1]:
            self.scan['ranges'][120] = value
            with self.assertRaises(ValueError): MODULE.probe_clearance(self.scan, self.ages)

    def test_probe_never_promotes_navigation_calibration(self):
        self.scan['navigation_validated'] = False
        MODULE.probe_clearance(self.scan, self.ages)
        self.assertFalse(self.scan['navigation_validated'])
        self.assertEqual(MODULE.probe_parameters({'pwm': 1560, 'duration_ms': 400}), (1560, 400))
        self.assertEqual(MODULE.probe_parameters({'pwm': 1570, 'duration_ms': 400}), (1570, 400))
        self.assertEqual(MODULE.probe_parameters({'pwm': 1580, 'duration_ms': 400}), (1580, 400))
        self.assertEqual(MODULE.probe_parameters({'pwm': 1550, 'duration_ms': 400}), (1550, 400))
        with self.assertRaises(ValueError): MODULE.probe_parameters({'pwm': 1581, 'duration_ms': 400})
        with self.assertRaises(ValueError): MODULE.probe_parameters({'pwm': 1549, 'duration_ms': 400})
        with self.assertRaises(ValueError): MODULE.probe_parameters({'pwm': 1530, 'duration_ms': 501})
        with self.assertRaises(ValueError): MODULE.probe_parameters({'pwm': True, 'duration_ms': 400})
        self.assertEqual(MODULE.probe_parameters({'pwm': 1560, 'duration_ms': 30000}, straight=True), (1560, 30000))
        with self.assertRaises(ValueError): MODULE.probe_parameters({'pwm': 1560, 'duration_ms': 30001}, straight=True)


class QualityRecoveryTests(unittest.TestCase):
    def test_two_seconds_and_full_recovery_reset(self):
        latch = MODULE.QualityLatch()
        self.assertFalse(latch.update(['front_sparse'], 10))
        self.assertFalse(latch.update(['scan_incomplete'], 11.99))
        self.assertTrue(latch.update(['front_sparse'], 12))
        self.assertFalse(latch.update([], 12.1))
        self.assertFalse(latch.update(['front_sparse'], 13))
        self.assertFalse(latch.update(['front_sparse'], 14.99))
        self.assertTrue(latch.update(['front_sparse'], 15))

    def setUp(self):
        # Deterministic controller clock and recording emitter; no tty is opened.
        root = Path(__file__).parents[1]
        sys.path.insert(0, str(root))
        try:
            spec = importlib.util.spec_from_file_location('console_fault_test', root / 'server.py')
            server = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(server)
        finally:
            sys.path.pop(0)
        self.temp = tempfile.TemporaryDirectory()
        args = types.SimpleNamespace(access_file=str(Path(self.temp.name)/'access.json'),
                                     output=str(Path(self.temp.name)/'output'), demo=True, allow_reverse=False)
        self.console = server.Console(args)
        self.server = server
        self.console.healthy = lambda: True
        self.console.autonomy_healthy = lambda: True
        self.console.sensor_ages = lambda now: {'camera': .01, 'lidar': .01, 'control': .01}
        self.output = []
        self.console.emit = lambda op, motor=1500, servo=1500, tick=0: self.output.append((op, motor, servo))
        self.console.scan = {'seq': 1, 'ranges': [3.] * 360}
        self.console.scan['ranges'][5:12] = [None] * 7
        self.console.status = {'control': {'armed': True, 'seq': 1, 'tick': 100, 'motor': 1500, 'servo': 1500}}
        self.console.arm_sequence = 1
        self.console.owner = 'test-auto';self.console.stop_latched = False
        self.console.control_mode = 'auto_probe'
        self.console.auto_session = {'report': {'run_id': 'test', 'pwm': 1560, 'observed_pwm': False,
                                               'drive_ticks': 0, 'motion_ticks': 0, 'recovery_ticks': 0, 'steering_changes': 0},
                                     'deadline': 10, 'last_loop': 0, 'heartbeat_seq': 0,
                                     'quality': server.QualityLatch()}

    def tearDown(self):
        self.temp.cleanup()

    def tick(self, now):
        self.console.owner_at = now
        self.console.autonomy_tick(now)

    def test_sparse_observation_waits_neutral_recovers_without_rearm(self):
        for i in range(50): self.tick(i*.02)
        self.assertIsNotNone(self.console.auto_session)
        self.assertEqual(self.output[-1], ('drive', 1500, 1500))
        self.console.scan['ranges'] = [3.] * 360
        self.tick(1.0)
        self.assertEqual(self.output[-1], ('drive', 1560, 1500))
        self.assertEqual(self.console.auto_session['report']['quality_elapsed_ms'], 0)

    def test_continuous_sparse_observation_latches_after_two_seconds(self):
        for i in range(100): self.tick(i*.02)
        self.assertIsNotNone(self.console.auto_session)
        self.tick(2.0)
        self.assertIsNone(self.console.auto_session)
        self.assertEqual(self.console.auto_result['reason'], 'perception_quality_timeout')
        self.assertEqual(self.output[-1][0], 'stop')

    def test_close_obstacle_is_immediate_during_quality_recovery(self):
        self.tick(0)
        self.console.scan['ranges'][0] = .8
        self.tick(.02)
        self.assertIsNone(self.console.auto_session)
        self.assertEqual(self.console.auto_result['reason'], 'probe_obstacle_in_straight_corridor')

    def test_emergency_stop_remains_latched_even_if_scan_recovers(self):
        self.tick(0)
        self.console.command({'op': 'stop'})
        self.console.scan['ranges'] = [3.] * 360
        self.tick(.02)
        self.assertIsNone(self.console.auto_session)
        self.assertEqual(self.output[-1][0], 'stop')

    def test_centering_is_rate_limited_in_actual_output_loop(self):
        self.console.scan = wall_scan(.12)
        session = self.console.auto_session
        session['report']['centering'] = True
        session['steering'] = MODULE.CorridorSteering()
        session['wall_scan_seq'] = None
        for i in range(20):
            self.console.scan['seq'] += 1
            self.tick(i*.02)
        servos = [servo for op, _, servo in self.output if op == 'drive']
        self.assertEqual(servos[0], 1480)
        self.assertEqual(servos[6], 1480)
        self.assertLess(servos[-1], servos[0])
        self.assertEqual(session['report']['steering_changes'], 2)

    def test_missing_camera_waits_neutral_until_quality_latch(self):
        self.console.scan['ranges'] = [3.]*360
        self.console.sensor_ages = lambda now: {'camera': 1.2+now, 'lidar': .01, 'control': .01}
        for i in range(100): self.tick(i*.02)
        self.assertIsNotNone(self.console.auto_session)
        self.assertEqual(self.output[-1], ('drive', 1500, 1500))
        self.tick(2)
        self.assertEqual(self.console.auto_result['reason'], 'perception_quality_timeout')

    def test_normal_one_second_boundaries_do_not_reset_quality_streak(self):
        self.console.perception_quality = self.console.auto_session['quality']
        self.console.perception_quality.update(['front_sparse'], 10)
        self.console.auto_session['report']['observed_armed'] = True
        with patch.object(self.server.time, 'monotonic', return_value=11):
            self.console.halt('probe_complete')
        self.console.status['control'].update(armed=False, tick=200)
        with patch.object(self.server.time, 'monotonic', return_value=11.2):
            self.console.autonomy_command({'op': 'straight_start', 'boot': self.console.boot,
                'epoch': self.console.control_epoch, 'tick': 200, 'pwm': 1560, 'duration_ms': 3000})
        self.assertEqual(self.console.auto_session['quality'].since, 10)
        self.console.status['control'].update(armed=True, seq=100)
        for i in range(40): self.tick(11.2+i*.02)
        self.assertIsNotNone(self.console.auto_session)
        self.tick(12)
        self.assertEqual(self.console.auto_result['reason'], 'perception_quality_timeout')


    def set_junction_endpoint(self):
        self.console.scan = junction_scan(1)
        session = self.console.auto_session
        session['report']['endpoint'] = 'left_junction'
        session['junction'] = MODULE.JunctionStop()

    def test_output_loop_continuously_drives_then_stops_at_junction(self):
        self.set_junction_endpoint()
        for i in range(10): self.tick(i*.02)
        self.assertEqual(len(self.output), 10)
        self.assertTrue(all(row == ('drive', 1560, 1500) for row in self.output))
        for i in [10, 11]:
            self.console.scan['seq'] += 1
            self.tick(i*.02)
        self.assertIsNone(self.console.auto_session)
        self.assertEqual(self.console.auto_result['reason'], 'left_junction_reached')
        self.assertTrue(self.console.auto_result['completed'])
        self.assertEqual(self.output[-1][0], 'stop')
        self.tick(.24)
        self.assertNotIn('arm', [op for op, _, _ in self.output])

    def test_junction_timeout_is_not_success(self):
        self.set_junction_endpoint()
        self.console.auto_session['deadline'] = .04
        for i in range(3): self.tick(i*.02)
        self.assertEqual(self.console.auto_result['reason'], 'left_junction_timeout')
        self.assertFalse(self.console.auto_result['completed'])

    def test_manual_stop_cannot_resume_on_next_junction_vote(self):
        self.set_junction_endpoint()
        self.tick(0)
        self.console.scan['seq'] += 1
        self.tick(.02)
        self.console.command({'op': 'stop'})
        self.console.scan['seq'] += 1
        self.tick(.04)
        self.assertEqual(self.console.auto_result['reason'], 'operator_stop')
        self.assertFalse(self.console.auto_result['completed'])


    def test_rear_launch_keeps_servo_neutral_then_restores_guards(self):
        self.console.scan = wall_scan(.12)
        self.console.scan['ranges'][180] = .28
        session = self.console.auto_session
        session['report']['centering'] = True
        session['steering'] = MODULE.CorridorSteering()
        session['wall_scan_seq'] = None
        session['rear_launch'] = MODULE.RearLaunch()
        for i in range(50):
            self.console.scan['seq'] += 1
            self.tick(i*.02)
        self.assertTrue(all(servo == 1500 for op, _, servo in self.output))
        self.tick(1)
        self.assertIsNone(self.console.auto_session)
        self.assertEqual(self.console.auto_result['reason'], 'probe_obstacle_close')
        self.assertFalse(self.console.auto_result['completed'])



    def test_output_loop_decreases_pwm_then_front_margin_stops_without_junction(self):
        self.set_junction_endpoint()
        session = self.console.auto_session
        session['ramp'] = MODULE.ApproachRamp(1560)
        self.console.scan['left_junction'] = None
        self.console.scan['front_boundary_m'] = 3.8
        for i in range(30):
            self.console.scan['seq'] = i//5
            self.console.scan['front_boundary_m'] = 3.8-.1*(i//5)
            self.tick(i*.02)
            if self.console.auto_session is None:
                break
        motors = [motor for op, motor, _ in self.output if op == 'drive']
        self.assertEqual(motors[0], 1555)
        self.assertLess(motors[-1], motors[0])
        self.assertGreater(motors[-1], 1500)
        self.assertEqual(self.output[-1][0], 'stop')
        self.assertEqual(self.console.auto_result['reason'], 'front_boundary_stop')
        self.assertFalse(self.console.auto_result['completed'])
        self.tick(.62)
        self.assertEqual(self.output[-1][0], 'stop')

    def test_prediction_steering_and_speed_limit_share_first_fresh_output(self):
        self.console.scan = wall_scan(-.003, math.tan(math.radians(-2.9)))
        session = self.console.auto_session
        session['report']['centering'] = True
        session['steering'] = MODULE.CorridorSteering()
        session['wall_scan_seq'] = None
        session['ramp'] = MODULE.ApproachRamp(1560)
        self.tick(0)
        op, motor, servo = self.output[-1]
        self.assertEqual((op, motor), ('drive', 1560))
        self.assertTrue(1500 < servo <= 1515)
        self.assertTrue(session['report']['lane_correcting'])
        self.assertFalse(session['report']['lane_speed_limited'])
        for i in range(1, 38):
            self.console.scan['seq'] += 1
            self.tick(i*.02)
        self.assertTrue(session['report']['lane_speed_limited'])
        self.assertTrue(1550 <= self.output[-1][1] <= 1560)
        self.assertIsNone(session['report']['approach_front_m'])

    def test_bridge_large_scan_stream_is_buffered_and_eof_locks(self):
        class CountingRaw(io.RawIOBase):
            def __init__(self, data): self.data=io.BytesIO(data);self.calls=0
            def readable(self): return True
            def readinto(self, buffer):
                self.calls += 1
                return self.data.readinto(buffer)
        rows = b''.join((json.dumps({'seq': i, 'ranges': [3.]*360})+'\n').encode() for i in range(40))
        raw = CountingRaw(rows)
        self.console.auto_session = None
        self.console.owner = None
        self.console.read_bridge(types.SimpleNamespace(stdout=raw), 'lidar')
        self.assertEqual(self.console.scan['seq'], 39)
        self.assertLess(raw.calls, 10)  # Whole chunks, rather than 70,000 byte reads.
        self.assertEqual(self.console.last_stop['reason'], 'bridge_failure')
        self.assertEqual(self.output[-1][0], 'stop')


if __name__ == '__main__':
    unittest.main()
