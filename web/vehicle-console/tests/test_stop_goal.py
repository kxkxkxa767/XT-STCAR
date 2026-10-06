"""Final-goal protocol and console checks; synthetic clocks/poses, no devices."""
import copy
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

from test_coast_motion import room_scan

ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT))
try:
    import stop_goal as goal_module
finally:
    sys.path.pop(0)


def contract_raw():
    return {'schema_version': 1, 'simulation_only': True, 'physical_source_verified': False,
        'world_frame': 'world', 'body_frame': 'lidar_body',
        'clock': {'domain': 'online_shared_monotonic_ms', 'epoch_id': 'synthetic-epoch',
            'source_anchor_ms': 0, 'local_anchor_monotonic_s': 0., 'max_error_ms': 0.,
            'valid_until_monotonic_s': 100., 'verified': False},
        'geometry': {'reference_origin': 'lidar_rotation_center', 'pose_origin_verified': True,
            'physical_tf_verified': False, 'measurement_error_bound_m': .001, 'wheelbase_m': .25,
            'footprint': {'front_m': .21, 'rear_m': .20, 'half_width_m': .17}},
        'pose': {'verified': False, 'position_error_m': .001, 'heading_error_rad': .001,
            'speed_error_mps': .001, 'yaw_rate_error_radps': .001, 'min_quality': .8,
            'max_age_ms': 200, 'max_gap_ms': 200},
        'budget': {'calibrated': False, 'response_delay_ms': 50, 'slowdown_distance_m': .5,
            'safety_margin_m': .03, 'max_heading_delta_rad': math.radians(5), 'max_lateral_error_m': .15,
            'max_goal_update_shift_m': .1, 'max_drive_ms': 10000, 'max_geometry_age_ms': 1000,
            'max_visual_age_ms': 1000, 'neutral_stop_envelope': [
                {'speed_upper_mps': .01, 'distance_upper_m': .01},
                {'speed_upper_mps': 1., 'distance_upper_m': .5}],
            'forward_command_envelope': {'pwm_min': 1550, 'pwm_max': 1580, 'speed_upper_mps': 1.,
                'launch_acceleration_covered': True, 'measured': False}}}


def raw_goal(at=1000, x=0., y=0., speed=0.):
    return {'schema_version': 1, 'role': 'final_stop', 'goal_id': 'task-1-track-4', 'track_id': 4,
        'task_revision': 1, 'element_kind': 'crosswalk', 'world_frame': 'world', 'body_frame': 'lidar_body',
        'coordinate_convention': 'x_forward_y_left_yaw_left_positive',
        'clock_domain': 'online_shared_monotonic_ms', 'clock_epoch_id': 'synthetic-epoch',
        'generated_at': at, 'geometry_source_at': at, 'visual_source_at': at, 'pose_source_at': at,
        'expires_at': at+190, 'source_pose': {'captured_at': at, 'frame_id': 'world',
            'pose': {'x_m': x, 'y_m': y, 'yaw_rad': 0.}, 'speed_mps': speed, 'yaw_rate_radps': 0., 'quality': 1.},
        'observed_region_pose': {'x_m': 4., 'y_m': 0., 'yaw_rad': 0.},
        'region_geometry': {'shape': 'line_region', 'lateral_half_width_m': .7, 'depth_m': 1.},
        'position_error_m': .02, 'heading_error_rad': 0., 'region_error_m': .02,
        'footprint': {'front_m': .21, 'rear_m': .20, 'half_width_m': .17},
        'final_pose': {'x_m': 3.72, 'y_m': 0., 'yaw_rad': 0.},
        'permission_boundary': {'origin': {'x_m': 4., 'y_m': 0.}, 'normal': {'x_m': 1., 'y_m': 0.},
            'max_projection_m': -.02, 'lateral_bounds_m': [-.7, .7]},
        'stop_semantics': 'before_crosswalk_near_edge', 'goal_tolerance_m': .04,
        'goal_heading_tolerance_rad': .04, 'stopped_corner_speed_mps': .02, 'required_hold_ms': 3000,
        'simulation_only': True, 'physical_control_ready': False, 'physical_budget_available': False}


def finish_record(at=1000, permission_change=None):
    goal = raw_goal(at=at, x=4.72)
    goal.update(element_kind='finish_marker', stop_semantics='inside_finish_region', required_hold_ms=0)
    goal['final_pose']['x_m'] = 4.72
    goal['permission_boundary']['max_projection_m'] = .98
    permission = {'light_cleared': True, 'verified': True, 'clock_epoch_id': 'synthetic-epoch', 'source_at': at,
                  'goal_id': goal['goal_id'], 'task_revision': goal['task_revision'], 'track_id': goal['track_id']}
    permission.update(permission_change or {})
    return {'final_stop_goal': goal, 'completion_permissions': permission}


class StopGoalTests(unittest.TestCase):
    def setUp(self):
        self.contract = goal_module.StopGoalContract(contract_raw(), real=False)

    def goal(self, raw=None, now=1.01):
        return goal_module.FinalStopGoal(raw or raw_goal(), self.contract, now, real=False)

    def test_rolling_target_cannot_replace_the_final_destination(self):
        record = {'target': {'point': {'x_m': .8, 'y_m': 0.}}, 'final_stop_goal': raw_goal()}
        self.assertEqual(self.goal(record).final_pose['x_m'], 3.72)
        with self.assertRaisesRegex(goal_module.StopGoalError, 'final_destination_missing'):
            self.goal({'target': record['target']})

    def test_axes_are_reflected_once_including_boundary_and_lateral_interval(self):
        raw = raw_goal(y=2.)
        for pose in (raw['final_pose'], raw['observed_region_pose']):
            pose.update(y_m=2., yaw_rad=.1)
        raw['source_pose']['pose']['yaw_rad'] = .1
        raw['permission_boundary'].update(origin={'x_m': 4., 'y_m': 2.},
            normal={'x_m': math.cos(.1), 'y_m': math.sin(.1)}, lateral_bounds_m=[-1., 2.])
        goal = self.goal(raw)
        self.assertEqual(goal.pose['y_m'], -2.)
        self.assertEqual(goal.final_pose['yaw_rad'], -.1)
        self.assertAlmostEqual(goal.normal[1], -math.sin(.1))
        self.assertEqual(goal.lateral_bounds, [-2., 1.])
        self.assertEqual(raw['final_pose']['y_m'], 2.)
        raw['coordinate_convention'] = 'x_forward_y_right_yaw_right_positive'
        with self.assertRaisesRegex(goal_module.StopGoalError, 'axes_unknown'):
            self.goal(raw)

    def test_translation_of_world_origin_preserves_remaining_goal_distance(self):
        first = self.goal()
        raw = raw_goal(x=20., y=-12.)
        for pose in (raw['final_pose'], raw['observed_region_pose'], raw['permission_boundary']['origin']):
            pose['x_m'] += 20.
            pose['y_m'] -= 12.
        shifted = self.goal(raw)
        a = goal_module.StopGoalConsumer(first, 1580).tick(1.01, 'drive', {'heading_right_deg': 0.})
        b = goal_module.StopGoalConsumer(shifted, 1580).tick(1.01, 'drive', {'heading_right_deg': 0.})
        self.assertAlmostEqual(a['remaining_goal_m'], b['remaining_goal_m'])
        self.assertEqual(a['motor_cap'], b['motor_cap'])

    def test_far_same_line_target_allows_small_heading_error_without_distance_lateral_growth(self):
        raw = raw_goal()
        raw['final_pose']['x_m'] = 10.
        raw['observed_region_pose']['x_m'] = raw['permission_boundary']['origin']['x_m'] = 10.28
        raw['source_pose']['pose']['yaw_rad'] = math.radians(3)
        result = goal_module.StopGoalConsumer(self.goal(raw), 1580).tick(1.01, 'drive', {'heading_right_deg': 3.})
        self.assertFalse(result['request_coast'])
        self.assertEqual(result['motor_cap'], 1580)
        self.assertAlmostEqual(result['remaining_goal_m'], 10*math.cos(math.radians(3)))
        # Applying the same rigid world transform does not change this decision.
        shifted = copy.deepcopy(raw)
        angle, tx, ty = .6, 7., -9.
        c, s = math.cos(angle), math.sin(angle)
        for pose in (shifted['final_pose'], shifted['observed_region_pose'], shifted['source_pose']['pose']):
            x, y = pose['x_m'], pose['y_m']
            pose.update(x_m=tx+c*x-s*y, y_m=ty+s*x+c*y, yaw_rad=pose['yaw_rad']+angle)
        shifted['permission_boundary']['origin'] = {k: shifted['observed_region_pose'][k] for k in ('x_m', 'y_m')}
        shifted['permission_boundary']['normal'] = {'x_m': c, 'y_m': s}
        transformed = goal_module.StopGoalConsumer(self.goal(shifted), 1580).tick(1.01, 'drive', {'heading_right_deg': 3.})
        self.assertAlmostEqual(result['remaining_goal_m'], transformed['remaining_goal_m'])
        self.assertEqual(result['motor_cap'], transformed['motor_cap'])

    def test_missing_nested_objects_and_caller_mutation_cannot_change_admitted_state(self):
        for field in ('source_pose', 'region_geometry', 'permission_boundary'):
            raw = raw_goal()
            raw[field] = None
            with self.subTest(field=field), self.assertRaises(goal_module.StopGoalError):
                self.goal(raw)
        raw = raw_goal()
        goal = self.goal(raw)
        raw['source_pose']['speed_mps'] = 99.
        raw['final_pose']['x_m'] = .8
        self.assertEqual(goal.source['speed_mps'], 0.)
        self.assertEqual(goal.final_pose['x_m'], 3.72)

    def test_finite_permission_side_bounds_apply_to_every_final_and_current_corner(self):
        raw = raw_goal()
        raw['permission_boundary']['lateral_bounds_m'] = [-.1, .1]
        with self.assertRaisesRegex(goal_module.StopGoalError, 'final_footprint_not_permitted'):
            self.goal(raw)
        raw = raw_goal(y=.1)
        raw['permission_boundary']['lateral_bounds_m'] = [-.2, .2]
        goal = self.goal(raw)
        self.assertTrue(goal.permitted(goal.final_pose))
        self.assertFalse(goal.permitted(goal.pose))
        with self.assertRaisesRegex(goal_module.StopGoalError, 'permission_boundary_crossed'):
            goal_module.StopGoalConsumer(goal, 1580).tick(1.01, 'drive', {'heading_right_deg': 0.})

    def test_current_simulation_export_is_rejected_in_real_mode(self):
        raw = raw_goal()
        raw['clock_epoch_id'] = None
        with self.assertRaisesRegex(goal_module.StopGoalError, 'simulation_or_physical_unready'):
            goal_module.FinalStopGoal(raw, None, 1.01, real=True)
        raw.update(simulation_only=False, physical_control_ready=True, physical_budget_available=True)
        with self.assertRaisesRegex(goal_module.StopGoalError, 'contract_mode_mismatch'):
            goal_module.FinalStopGoal(raw, self.contract, 1.01, real=True)

    def test_measured_sizes_without_tf_error_navigation_and_stopping_do_not_make_real_ready(self):
        raw = contract_raw()
        raw.update(simulation_only=False, physical_source_verified=True)
        raw['clock']['verified'] = raw['pose']['verified'] = raw['geometry']['physical_tf_verified'] = True
        profile = json.loads((ROOT.parents[1]/'config/vehicle-geometry-measured-20261006.json').read_text())
        with self.assertRaisesRegex(goal_module.StopGoalError, 'vehicle_profile_unverified'):
            goal_module.StopGoalContract(raw, real=True, vehicle_profile=profile)
        self.assertIsNone(profile['lidar_to_rear_axle_m'])

    def test_boolean_integer_protocol_fields_are_rejected(self):
        for field in ('schema_version', 'task_revision', 'track_id', 'generated_at', 'required_hold_ms'):
            raw = raw_goal()
            raw[field] = True
            with self.subTest(field=field), self.assertRaises(goal_module.StopGoalError):
                self.goal(raw)
        contract = contract_raw()
        contract['schema_version'] = True
        with self.assertRaises(goal_module.StopGoalError):
            goal_module.StopGoalContract(contract, real=False)

    def test_clock_epoch_frame_pose_source_and_error_budgets_are_required(self):
        for change in [{'clock_epoch_id': None}, {'body_frame': 'rear_axle'}, {'world_frame': 'other'},
                       {'pose_source_at': 999}, {'region_error_m': None}]:
            with self.subTest(change=change), self.assertRaises(goal_module.StopGoalError):
                self.goal(dict(raw_goal(), **change))
        raw = contract_raw()
        del raw['pose']['speed_error_mps']
        with self.assertRaisesRegex(goal_module.StopGoalError, 'error_budget_missing'):
            goal_module.StopGoalContract(raw, real=False)

    def test_goal_deadline_is_exclusive_and_not_a_bridge_or_lidar_clock(self):
        goal = self.goal()
        with self.assertRaisesRegex(goal_module.StopGoalError, 'expired_or_future'):
            goal.check_fresh(1.19)
        raw = raw_goal()
        raw['generated_at'] = 1030
        with self.assertRaisesRegex(goal_module.StopGoalError, 'expired_or_future'):
            self.goal(raw)

    def test_missing_neutral_or_launch_command_envelope_refuses_admission(self):
        for key in ('neutral_stop_envelope', 'forward_command_envelope'):
            raw = contract_raw()
            del raw['budget'][key]
            with self.subTest(key=key), self.assertRaises(goal_module.StopGoalError):
                goal_module.StopGoalContract(raw, real=False)

    def test_prospective_forward_speed_prevents_near_goal_launch_at_zero_current_speed(self):
        consumer = goal_module.StopGoalConsumer(self.goal(raw_goal(x=3.3)), 1580)
        result = consumer.tick(1.01, 'drive', {'heading_right_deg': 0.})
        self.assertEqual(result['prospective_speed_upper_mps'], 1.)
        self.assertGreater(result['neutral_distance_budget_m'], .5)
        self.assertTrue(result['request_coast'])
        self.assertEqual(result['motor_cap'], 1500)

    def test_repeat_does_not_renew_expiry_or_hold_and_reordered_or_changed_task_is_rejected(self):
        consumer = goal_module.StopGoalConsumer(self.goal(), 1580)
        raw = raw_goal()
        raw['expires_at'] = 2000
        self.assertFalse(consumer.update(self.goal(raw)))
        self.assertEqual(consumer.goal.raw['expires_at'], 1190)
        raw = raw_goal(at=1100)
        raw.update(task_revision=2, goal_id='task-2-track-4')
        with self.assertRaisesRegex(goal_module.StopGoalError, 'identity_changed'):
            consumer.update(self.goal(raw, 1.11))
        with self.assertRaisesRegex(goal_module.StopGoalError, 'source_reordered'):
            consumer.update(self.goal(raw_goal(at=900), .91))

    def test_same_source_clock_cannot_change_pose_or_geometry(self):
        consumer = goal_module.StopGoalConsumer(self.goal(), 1580)
        raw = raw_goal(x=.01)
        with self.assertRaisesRegex(goal_module.StopGoalError, 'pose_changed_without_new_source'):
            consumer.update(self.goal(raw))
        raw = raw_goal()
        raw['final_pose']['x_m'] -= .01
        with self.assertRaisesRegex(goal_module.StopGoalError, 'geometry_changed_without_new_source'):
            consumer.update(self.goal(raw))

    def test_coast_updates_never_restore_positive_pwm_or_complete_far_from_goal(self):
        consumer = goal_module.StopGoalConsumer(self.goal(), 1580)
        self.assertEqual(consumer.tick(1.01, 'coast', None)['motor_cap'], 1500)
        consumer.update(self.goal(raw_goal(at=1100), 1.11))
        result = consumer.tick(1.11, 'drive', {'heading_right_deg': 0.})
        self.assertEqual(result['motor_cap'], 1500)
        self.assertFalse(result['completed'])

    def test_hold_uses_new_measured_pose_corner_speed_and_preserves_three_seconds(self):
        consumer = goal_module.StopGoalConsumer(self.goal(raw_goal(x=3.72)), 1580)
        for at in range(1000, 4101, 100):
            now = at/1000+.01
            if at != 1000:
                consumer.update(self.goal(raw_goal(at=at, x=3.72), now))
            result = consumer.tick(now, 'coast', None)
            if at < 4000:
                self.assertFalse(result['completed'])
        self.assertTrue(result['completed'])
        self.assertGreaterEqual(result['hold_source_ms'], 3000)
        # Same pose report or late local polling cannot fabricate a new hold.
        self.assertEqual(consumer.tick(4.15, 'coast', None)['hold_observations'], result['hold_observations'])
        raw = raw_goal(at=4200, x=3.72)
        raw['source_pose']['yaw_rate_radps'] = .2
        consumer.update(self.goal(raw, 4.21))
        self.assertFalse(consumer.tick(4.21, 'coast', None)['completed'])

    def test_crosswalk_short_hold_and_ninety_degree_straight_budget_are_rejected(self):
        raw = raw_goal()
        raw['required_hold_ms'] = 100
        with self.assertRaisesRegex(goal_module.StopGoalError, 'hold_too_short'):
            self.goal(raw)
        raw = contract_raw()
        raw['budget']['max_heading_delta_rad'] = math.pi/2
        with self.assertRaisesRegex(goal_module.StopGoalError, 'straight_capability_invalid'):
            goal_module.StopGoalContract(raw, real=False)

    def test_finish_permission_binds_current_goal_revision_and_track(self):
        consumer = goal_module.StopGoalConsumer(self.goal(finish_record()), 1580)
        for at in (1000, 1100, 1200):
            now = at/1000+.01
            if at != 1000:
                consumer.update(self.goal(finish_record(at), now))
            result = consumer.tick(now, 'coast', None)
        self.assertTrue(result['completed'])
        changes = [{'goal_id': 'task-2-track-4'}, {'task_revision': 2}, {'track_id': 5},
                   {'goal_id': None}, {'task_revision': None}, {'track_id': None},
                   {'task_revision': True}, {'track_id': True}]
        for change in changes:
            with self.subTest(change=change):
                bad = goal_module.StopGoalConsumer(self.goal(finish_record(permission_change=change)), 1580)
                for at in (1000, 1100, 1200):
                    now = at/1000+.01
                    if at != 1000:
                        bad.update(self.goal(finish_record(at, change), now))
                    self.assertFalse(bad.goal.permission_fresh(now))
                    self.assertFalse(bad.tick(now, 'coast', None)['completed'])
        # Literal absence is also untrusted; an epoch alone cannot bind a task.
        record = finish_record()
        for key in ('goal_id', 'task_revision', 'track_id'):
            del record['completion_permissions'][key]
        self.assertFalse(self.goal(record).permission_fresh(1.01))


class StopGoalConsoleTests(unittest.TestCase):
    def setUp(self):
        spec = importlib.util.spec_from_file_location('goal_console', ROOT/'server.py')
        self.server = importlib.util.module_from_spec(spec)
        sys.path.insert(0, str(ROOT))
        try:
            spec.loader.exec_module(self.server)
        finally:
            sys.path.pop(0)
        self.temp = tempfile.TemporaryDirectory()
        args = types.SimpleNamespace(access_file=str(Path(self.temp.name)/'access.json'),
            output=str(Path(self.temp.name)/'out'), demo=True, allow_reverse=False)
        self.console = self.server.Console(args)
        self.console.stop_goal_contract = goal_module.StopGoalContract(contract_raw(), real=False)
        self.console.scan = room_scan(1, half_width=.6)
        self.console.scan_at = self.console.control_at = 1.
        self.console.status = {'control': {'armed': False, 'motor': 1500, 'servo': 1500, 'seq': 0, 'tick': 1000}}
        self.console.healthy = self.console.autonomy_healthy = lambda: True
        self.console.sensor_ages = lambda now: {'camera': .01, 'lidar': .01, 'control': .01}
        self.console.coast_worker = self.server.CoastMotionWorker(autostart=False)
        self.output = []
        self.console.emit = lambda op, motor=1500, servo=1500, tick=0: self.output.append((op, motor, servo))

    def tearDown(self):
        self.console.coast_worker.close()
        self.temp.cleanup()

    def message(self, op, **extra):
        return {'op': op, 'boot': self.console.boot, 'epoch': self.console.control_epoch, **extra}

    def test_default_real_entry_has_no_source_and_never_arms(self):
        self.console.args.demo = False
        self.console.stop_goal_contract = None
        with self.assertRaisesRegex(goal_module.StopGoalError, 'simulation_or_physical_unready'):
            self.console.accept_stop_goal(raw_goal(), now=1.01)
        with self.assertRaises(ValueError):
            self.console.autonomy_command(self.message('stop_goal_start', tick=1000, pwm=1580))
        self.assertEqual(self.output, [])
        self.assertFalse(self.console.status['control']['armed'])

    def test_refresh_uses_only_configured_local_file_and_never_renews_owner(self):
        path = Path(self.temp.name)/'goal.json'
        path.write_text(json.dumps({'final_stop_goal': raw_goal(), 'target': {'point': {'x_m': .8, 'y_m': 0.}}}))
        self.console.stop_goal_source = path
        self.console.owner_at = .7
        with patch.object(self.server.time, 'monotonic', return_value=1.01):
            first = self.console.autonomy_command(self.message('stop_goal_refresh'))
            second = self.console.autonomy_command(self.message('stop_goal_refresh'))
        self.assertTrue(first['updated'])
        self.assertFalse(second['updated'])
        self.assertEqual(first['final_stop_goal']['final_pose_right']['x_m'], 3.72)
        self.assertEqual(self.console.owner_at, .7)
        self.assertEqual(self.output, [])
        for extra in [{'source_path': str(path)}, {'final_stop_goal': raw_goal()}]:
            with self.assertRaisesRegex(ValueError, 'no_paths_or_inline_goal'):
                self.console.autonomy_command(self.message('stop_goal_refresh', **extra))

    def test_formal_status_uses_straight_rear_launch_gates_without_mutating_hold(self):
        self.console.scan = room_scan(1, half_width=.6)
        self.console.scan['ranges'][180] = .35
        with patch.object(self.server.time, 'monotonic', return_value=1.01):
            self.console.accept_stop_goal(raw_goal())
            consumer = self.console.stop_goal_registry
            consumer.hold_observations = 7
            consumer.hold_since = 600
            before = consumer.hold_observations, consumer.hold_since, consumer.last_pose, consumer.neutral_latched
            status = self.console.autonomy_status()
            self.assertFalse(status['probe_ready'])
            self.assertTrue(status['formal_ready'])
            self.assertEqual((consumer.hold_observations, consumer.hold_since, consumer.last_pose, consumer.neutral_latched), before)
            for angle, distance in [(0, .8), (90, .29)]:
                self.console.scan = room_scan(1, half_width=.6)
                self.console.scan['ranges'][180] = .35
                self.console.scan['ranges'][angle] = distance
                with self.subTest(angle=angle):
                    self.assertFalse(self.console.autonomy_status()['formal_ready'])

    def test_formal_start_does_not_read_native_test_junction_and_deadline_is_failure(self):
        class NoJunction(dict):
            def get(self, key, *args):
                if key == 'left_junction':
                    raise AssertionError('formal path used the test junction')
                return super().get(key, *args)
        self.console.scan = NoJunction(self.console.scan)
        with patch.object(self.server.time, 'monotonic', return_value=1.01):
            self.console.accept_stop_goal(raw_goal())
            self.console.autonomy_command(self.message('stop_goal_start', tick=1000, pwm=1580))
        self.assertEqual(self.output, [('arm', 1500, 1500)])
        session = self.console.auto_session
        self.assertEqual(session['report']['endpoint'], 'final_stop_goal')
        session['deadline'] = 1.02
        self.console.owner_at = 1.02
        self.console.autonomy_tick(1.02)
        self.assertFalse(self.console.auto_result['completed'])
        self.assertEqual(self.console.auto_result['reason'], 'formal_goal_deadline')
        self.assertEqual(self.output[-1], ('stop', 1500, 1500))

    def test_different_upstream_task_during_formal_session_stops_without_rearming(self):
        with patch.object(self.server.time, 'monotonic', return_value=1.01):
            self.console.accept_stop_goal(raw_goal())
            self.console.autonomy_command(self.message('stop_goal_start', tick=1000, pwm=1580))
        raw = raw_goal(at=1100)
        raw.update(task_revision=2, goal_id='task-2-track-4')
        with self.assertRaisesRegex(goal_module.StopGoalError, 'identity_changed'):
            self.console.accept_stop_goal(raw, now=1.11)
        self.assertIsNone(self.console.auto_session)
        self.assertFalse(self.console.auto_result['completed'])
        self.assertEqual([row[0] for row in self.output], ['arm', 'stop'])

    def test_uncalibrated_lidar_still_or_protective_lock_does_not_complete_formal_goal(self):
        with patch.object(self.server.time, 'monotonic', return_value=1.01):
            self.console.accept_stop_goal(raw_goal(x=3.72))
            self.console.autonomy_command(self.message('stop_goal_start', tick=1000, pwm=1580))
        self.console.status['control'].update(armed=True, motor=1500, seq=100)
        self.console.owner_at = self.console.control_at = self.console.scan_at = 1.03
        self.console.scan = room_scan(2, half_width=.6)
        self.console.autonomy_tick(1.03)
        self.assertEqual(self.console.auto_session['phase'], 'coast')
        self.console.coast_worker.result = lambda *args: {'observable': True, 'stationary': True}
        self.console.owner_at = self.console.control_at = self.console.scan_at = 1.04
        self.console.scan = room_scan(3, half_width=.6)
        self.console.autonomy_tick(1.04)
        self.assertTrue(self.console.auto_session['report']['standstill_confirmed'])
        self.assertFalse(self.console.auto_session['report']['formal_plan']['completed'])
        self.assertEqual(self.output[-1][1], 1500)
        self.console.command({'op': 'stop'})
        self.assertFalse(self.console.auto_result['completed'])


class StopGoalCliTests(unittest.TestCase):
    def test_goal_status_and_nonexecute_straight_never_post_or_refresh(self):
        spec = importlib.util.spec_from_file_location('goal_query_cli', ROOT/'autonomy-control.py')
        cli = importlib.util.module_from_spec(spec)
        sys.path.insert(0, str(ROOT))
        try:
            spec.loader.exec_module(cli)
        finally:
            sys.path.pop(0)
        state = {'boot': 'boot', 'healthy': True, 'status': {'control': {'tick': 100}},
                 'autonomy': {'epoch': 0, 'formal_ready': False, 'formal_rejection': 'source_unavailable', 'final_stop_goal': None}}
        requests = []
        class Opener:
            def open(self, request, timeout=None):
                requests.append(request)
                return io.BytesIO(json.dumps(state).encode())
        with tempfile.TemporaryDirectory() as folder:
            access = Path(folder)/'access.json'
            access.write_text(json.dumps({'port': 8081, 'token': 'offline-test-token'}))
            for command in ['goal-status', 'goal-straight']:
                with self.subTest(command=command):
                    requests.clear()
                    output = io.StringIO()
                    with patch.object(sys, 'argv', ['autonomy-control.py', command, '--access-file', str(access)]),\
                         patch.object(cli.urllib.request, 'build_opener', return_value=Opener()),\
                         patch.object(sys, 'stdout', output):
                        cli.main()
                    self.assertEqual([request.get_method() for request in requests], ['GET'])
                    self.assertTrue(all(request.data is None for request in requests))
                    self.assertFalse(json.loads(output.getvalue())['motion_requested'])

    def test_formal_cli_sends_only_small_registered_source_messages(self):
        spec = importlib.util.spec_from_file_location('goal_cli', ROOT/'autonomy-control.py')
        cli = importlib.util.module_from_spec(spec)
        sys.path.insert(0, str(ROOT))
        try:
            spec.loader.exec_module(cli)
        finally:
            sys.path.pop(0)
        original = {'boot': 'boot', 'autonomy': {'epoch': 0}, 'status': {'control': {'tick': 100}}}
        calls, reads = [], []
        def request(path, data=None):
            if data:
                calls.append(data)
                return {'run_id': 'run', 'epoch': 0}
            active = not reads
            reads.append(active)
            return {'autonomy': {'mode': 'auto_goal' if active else 'locked',
                    'active': {'run_id': 'run'} if active else None,
                    'last_result': None if active else {'run_id': 'run', 'completed': True, 'reason': 'formal_goal_complete'}},
                    'ages': {'control': .01}, 'status': {'control': {'armed': active, 'motor': 1500, 'servo': 1500}}}
        with patch.object(cli.time, 'sleep'):
            cli.run_probe(request, original, 1580, 1000, centering=True, formal_goal=True)
        self.assertEqual([data['op'] for data in calls], ['stop_goal_start', 'stop_goal_refresh', 'heartbeat', 'cancel'])
        self.assertNotIn('duration_ms', calls[0])
        self.assertTrue(all(len(json.dumps(data).encode()) <= 2048 for data in calls))
        self.assertTrue(all('source_path' not in data and 'final_stop_goal' not in data for data in calls))


if __name__ == '__main__':
    unittest.main()
