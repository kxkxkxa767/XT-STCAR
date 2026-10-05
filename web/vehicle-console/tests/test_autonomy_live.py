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


def wall_scan(offset=0, slope=0, width=1.):
    bins = []
    for angle in range(360):
        sine, cosine = math.sin(math.radians(angle)), math.cos(math.radians(angle))
        denominator = sine-slope*cosine
        distances = [7/abs(cosine)] if abs(cosine) > 1e-6 else []
        if abs(denominator) > 1e-6:
            distances.extend(b/denominator for b in (offset-width/2, offset+width/2) if b/denominator > 0)
        bins.append(min(distances))
    return {'seq': 1, 'frame_id': 'lidar_origin_coarse_body_heading', 'ranges': bins}


class CorridorSteeringTests(unittest.TestCase):
    def test_cli_normal_deadline_racing_heartbeat_is_not_failure(self):
        spec = importlib.util.spec_from_file_location('autonomy_cli_test', Path(__file__).parents[1]/'autonomy-control.py')
        cli = importlib.util.module_from_spec(spec);spec.loader.exec_module(cli)
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
        self.assertEqual(steering.update(walls, 1, 0), 1495)
        self.assertEqual(steering.update(walls, 2, .34), 1495)
        self.assertEqual(steering.update(walls, 3, .36), 1490)
        for i in range(20):
            value = steering.update(walls, i+4, .72+i*.36)
            self.assertGreaterEqual(value, 1485)
        centred = MODULE.CorridorSteering()
        for i in range(50):
            walls = MODULE.corridor_walls(wall_scan(.06+(.002 if i%2 else -.002), .01))
            self.assertEqual(centred.update(walls, i, i*.05), 1500)
        left = MODULE.CorridorSteering()
        self.assertEqual(left.update(MODULE.corridor_walls(wall_scan(-.12)), 1, 0), 1505)

    def test_hysteresis_no_same_frame_renewal_and_bad_wall_fit(self):
        steering = MODULE.CorridorSteering()
        walls = MODULE.corridor_walls(wall_scan(.12))
        self.assertEqual(steering.update(walls, 1, 0), 1495)
        self.assertEqual(steering.update(walls, 1, 1), 1495)
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
        with self.assertRaises(ValueError): MODULE.probe_parameters({'pwm': 1561, 'duration_ms': 400})
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
        self.assertEqual(servos[0], 1495)
        self.assertEqual(servos[16], 1495)
        self.assertEqual(servos[-1], 1490)
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
