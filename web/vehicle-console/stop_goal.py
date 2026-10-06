"""Final upstream stop goals; no rolling targets, device IO or inferred calibration.

The source file is a FinalStopGoal or a report containing final_stop_goal. A
separate, explicitly configured contract binds its clock, pose origin/error and
measured neutral stopping envelope. Current simulation exports cannot pass the
real admission gates. Source timestamps, never reads or heartbeats, mature holds.
"""
import copy
import json
import math
import os
from pathlib import Path
import stat

from autonomy_live import MIN_FORWARD_PWM, MAX_PWM


class StopGoalError(ValueError):
    pass


def _require(value, reason):
    if not value:
        raise StopGoalError(reason)


def _number(value, reason, minimum=None):
    _require(type(value) in (int, float) and math.isfinite(value), reason)
    _require(minimum is None or value >= minimum, reason)
    return value


def _integer(value, reason, minimum=0):
    _require(type(value) is int and value >= minimum, reason)
    return value


def _mapping(value, reason):
    _require(isinstance(value, dict), reason)
    return value


def _pose(value):
    _require(isinstance(value, dict), 'stop_goal_pose_missing')
    return {'x_m': _number(value.get('x_m'), 'stop_goal_pose_invalid'),
            'y_m': -_number(value.get('y_m'), 'stop_goal_pose_invalid'),
            'yaw_rad': -_number(value.get('yaw_rad'), 'stop_goal_pose_invalid')}


def _wrap(value):
    return (value+math.pi) % (2*math.pi)-math.pi


def read_local_json(path):
    """Only a configured regular file; bounded reads, no symlinks/FIFOs or HTTP paths."""
    flags = os.O_RDONLY | os.O_NONBLOCK | getattr(os, 'O_NOFOLLOW', 0)
    try:
        fd = os.open(Path(path), flags)
        with os.fdopen(fd, 'rb') as handle:
            info = os.fstat(handle.fileno())
            _require(stat.S_ISREG(info.st_mode) and info.st_uid == os.getuid()
                     and not info.st_mode & 0o022 and info.st_size <= 65536, 'stop_goal_source_file_invalid')
            data = handle.read(65537)
        _require(len(data) <= 65536, 'stop_goal_source_file_too_large')
        return json.loads(data)
    except (OSError, json.JSONDecodeError) as error:
        raise StopGoalError('stop_goal_source_file_unavailable') from error


class StopGoalContract:
    """Explicit clock/origin/error and stopping measurements; no default physical values."""
    def __init__(self, raw, real, vehicle_profile=None):
        _require(isinstance(raw, dict) and type(raw.get('schema_version')) is int and raw['schema_version'] == 1,
                 'stop_goal_contract_missing')
        self.raw = copy.deepcopy(raw)
        raw = self.raw
        self.real = real
        _require(type(raw.get('simulation_only')) is bool, 'stop_goal_contract_provenance_missing')
        if real:
            _require(raw['simulation_only'] is False and raw.get('physical_source_verified') is True,
                     'stop_goal_physical_source_unverified')
        self.clock = _mapping(raw.get('clock'), 'stop_goal_clock_mapping_missing')
        _require(self.clock.get('domain') == 'online_shared_monotonic_ms'
                 and isinstance(self.clock.get('epoch_id'), str) and self.clock['epoch_id'], 'stop_goal_clock_epoch_missing')
        for key in ('source_anchor_ms',):
            _integer(self.clock.get(key), 'stop_goal_clock_mapping_missing')
        for key in ('local_anchor_monotonic_s', 'max_error_ms', 'valid_until_monotonic_s'):
            _number(self.clock.get(key), 'stop_goal_clock_mapping_missing', 0)
        _require(not real or self.clock.get('verified') is True, 'stop_goal_clock_mapping_unverified')
        self.geometry = _mapping(raw.get('geometry'), 'stop_goal_geometry_missing')
        _require(self.geometry.get('reference_origin') == 'lidar_rotation_center'
                 and self.geometry.get('pose_origin_verified') is True, 'stop_goal_pose_origin_unverified')
        self.world_frame, self.body_frame = raw.get('world_frame'), raw.get('body_frame')
        _require(all(isinstance(v, str) and v for v in (self.world_frame, self.body_frame)), 'stop_goal_frames_missing')
        self.footprint = _mapping(self.geometry.get('footprint'), 'stop_goal_footprint_missing')
        for key in ('front_m', 'rear_m', 'half_width_m'):
            _number(self.footprint.get(key), 'stop_goal_footprint_missing', .001)
        _number(self.geometry.get('measurement_error_bound_m'), 'stop_goal_measurement_error_missing', 0)
        _number(self.geometry.get('wheelbase_m'), 'stop_goal_wheelbase_missing', .001)
        if real:
            _require(self.geometry.get('physical_tf_verified') is True, 'stop_goal_physical_tf_unverified')
            _require(isinstance(vehicle_profile, dict) and vehicle_profile.get('reference_origin') == 'lidar_rotation_center',
                     'stop_goal_vehicle_profile_missing')
            _require(vehicle_profile.get('navigation_validated') is True
                     and vehicle_profile.get('stopping_envelope_calibrated') is True,
                     'stop_goal_vehicle_profile_unverified')
            measured = _mapping(vehicle_profile.get('straight_footprint'), 'stop_goal_vehicle_profile_invalid')
            expected = {'front_m': _number(measured.get('front_m'), 'stop_goal_vehicle_profile_invalid', .001),
                        'rear_m': _number(measured.get('rear_m'), 'stop_goal_vehicle_profile_invalid', .001),
                        'half_width_m': max(_number(measured.get('left_m'), 'stop_goal_vehicle_profile_invalid', .001),
                                            _number(measured.get('right_m'), 'stop_goal_vehicle_profile_invalid', .001))}
            _require(all(math.isclose(self.footprint[k], v, abs_tol=1e-9) for k, v in expected.items())
                     and math.isclose(self.geometry['wheelbase_m'],
                         _number(vehicle_profile.get('wheelbase_m'), 'stop_goal_vehicle_profile_invalid', .001), abs_tol=1e-9),
                     'stop_goal_measured_geometry_mismatch')
            _require(self.geometry['measurement_error_bound_m'] >= _number(
                vehicle_profile.get('measurement_error_bound_m'), 'stop_goal_vehicle_measurement_error_missing', 0),
                'stop_goal_measurement_error_underestimated')
        self.pose = _mapping(raw.get('pose'), 'stop_goal_pose_error_budget_missing')
        for key in ('position_error_m', 'heading_error_rad', 'speed_error_mps', 'yaw_rate_error_radps', 'min_quality'):
            _number(self.pose.get(key), 'stop_goal_pose_error_budget_missing', 0)
        for key in ('max_age_ms', 'max_gap_ms'):
            _integer(self.pose.get(key), 'stop_goal_pose_time_budget_missing', 1)
            _require(self.pose[key] <= 200, 'stop_goal_pose_time_budget_invalid')
        _require(self.pose['min_quality'] <= 1 and (not real or self.pose.get('verified') is True),
                 'stop_goal_pose_source_unverified')
        self.budget = _mapping(raw.get('budget'), 'stop_goal_physical_budget_missing')
        for key in ('response_delay_ms', 'slowdown_distance_m', 'safety_margin_m',
                    'max_heading_delta_rad', 'max_lateral_error_m', 'max_goal_update_shift_m'):
            _number(self.budget.get(key), 'stop_goal_physical_budget_missing', 0)
        # Existing wall controller fits only <=15 degrees; this interface is
        # not a turning route and cannot gain capability from a supplied budget.
        _require(0 < self.budget['max_heading_delta_rad'] <= math.radians(15)
                 and 0 < self.budget['max_lateral_error_m'] <= self.footprint['half_width_m'],
                 'stop_goal_straight_capability_invalid')
        for key in ('max_drive_ms', 'max_geometry_age_ms', 'max_visual_age_ms'):
            _integer(self.budget.get(key), 'stop_goal_physical_budget_missing', 1)
        _require(self.budget['max_drive_ms'] <= 30000, 'stop_goal_drive_budget_invalid')
        self.envelope = self.budget.get('neutral_stop_envelope')
        _require(isinstance(self.envelope, list) and len(self.envelope) >= 2, 'stop_goal_neutral_envelope_missing')
        previous_speed = previous_distance = -1.
        for item in self.envelope:
            _require(isinstance(item, dict), 'stop_goal_neutral_envelope_invalid')
            speed = _number(item.get('speed_upper_mps'), 'stop_goal_neutral_envelope_invalid', 0)
            distance = _number(item.get('distance_upper_m'), 'stop_goal_neutral_envelope_invalid', 0)
            _require(speed > previous_speed and distance >= previous_distance, 'stop_goal_neutral_envelope_invalid')
            previous_speed, previous_distance = speed, distance
        self.command = _mapping(self.budget.get('forward_command_envelope'), 'stop_goal_forward_command_envelope_missing')
        _require(type(self.command.get('pwm_min')) is int and type(self.command.get('pwm_max')) is int
                 and self.command['pwm_min'] <= MIN_FORWARD_PWM and self.command['pwm_max'] >= MAX_PWM
                 and self.command.get('launch_acceleration_covered') is True, 'stop_goal_forward_command_envelope_missing')
        command_speed = _number(self.command.get('speed_upper_mps'), 'stop_goal_forward_command_envelope_missing', .001)
        _require(command_speed <= self.envelope[-1]['speed_upper_mps'], 'stop_goal_command_exceeds_neutral_envelope')
        if real:
            evidence = self.budget.get('evidence_sha256', '')
            _require(self.budget.get('calibrated') is True and isinstance(self.budget.get('calibration_id'), str)
                     and self.budget['calibration_id'] and isinstance(evidence, str) and len(evidence) == 64
                     and all(c in '0123456789abcdef' for c in evidence), 'stop_goal_stopping_calibration_unverified')
            _require(self.budget.get('straight_control_validated') is True, 'stop_goal_straight_control_unverified')
            _require(self.command.get('measured') is True, 'stop_goal_command_envelope_unmeasured')

    def now_interval(self, now):
        _number(now, 'stop_goal_local_clock_invalid', 0)
        _require(now < self.clock['valid_until_monotonic_s'], 'stop_goal_clock_mapping_expired')
        mapped = self.clock['source_anchor_ms']+1000*(now-self.clock['local_anchor_monotonic_s'])
        return mapped-self.clock['max_error_ms'], mapped+self.clock['max_error_ms']

    def neutral_distance(self, speed):
        for item in self.envelope:
            if speed <= item['speed_upper_mps']:
                return item['distance_upper_m']
        raise StopGoalError('stop_goal_speed_outside_measured_envelope')


class FinalStopGoal:
    def __init__(self, record, contract, now, real):
        _require(isinstance(record, dict), 'stop_goal_missing')
        raw = record if record.get('role') == 'final_stop' else record.get('final_stop_goal')
        _require(isinstance(raw, dict) and raw.get('role') == 'final_stop'
                 and type(raw.get('schema_version')) is int and raw['schema_version'] == 1,
                 'stop_goal_final_destination_missing')
        self.raw = copy.deepcopy(raw)
        raw = self.raw
        _require(all(type(raw.get(k)) is bool for k in ('simulation_only', 'physical_control_ready', 'physical_budget_available')),
                 'stop_goal_physical_provenance_missing')
        if real:
            _require(raw.get('simulation_only') is False and raw.get('physical_control_ready') is True
                     and raw.get('physical_budget_available') is True, 'stop_goal_simulation_or_physical_unready')
        _require(isinstance(contract, StopGoalContract), 'stop_goal_contract_missing')
        _require(not real or contract.real, 'stop_goal_contract_mode_mismatch')
        self.contract = contract
        _require(raw.get('coordinate_convention') == 'x_forward_y_left_yaw_left_positive', 'stop_goal_axes_unknown')
        _require(raw.get('clock_domain') == contract.clock['domain']
                 and raw.get('clock_epoch_id') == contract.clock['epoch_id'], 'stop_goal_clock_epoch_mismatch')
        _require(raw.get('world_frame') == contract.world_frame and raw.get('body_frame') == contract.body_frame,
                 'stop_goal_frame_mismatch')
        revision = _integer(raw.get('task_revision'), 'stop_goal_revision_missing')
        track = _integer(raw.get('track_id'), 'stop_goal_track_missing')
        _require(raw.get('goal_id') == f'task-{revision}-track-{track}', 'stop_goal_identity_invalid')
        self.identity = raw['goal_id'], revision, track
        kinds = {'crosswalk': 'before_crosswalk_near_edge', 'stop_line': 'inside_light_stop_region',
                 'finish_marker': 'inside_finish_region'}
        _require(raw.get('element_kind') in kinds and raw.get('stop_semantics') == kinds[raw['element_kind']],
                 'stop_goal_semantics_invalid')
        for key in ('generated_at', 'geometry_source_at', 'visual_source_at', 'pose_source_at', 'expires_at', 'required_hold_ms'):
            _integer(raw.get(key), 'stop_goal_source_timestamp_missing')
        _require(raw['element_kind'] != 'crosswalk' or raw['required_hold_ms'] >= 3000, 'stop_goal_crosswalk_hold_too_short')
        _require(raw['expires_at'] > raw['generated_at']
                 and all(raw[k] <= raw['generated_at'] for k in ('geometry_source_at', 'visual_source_at', 'pose_source_at')),
                 'stop_goal_source_timestamp_invalid')
        self.source = _mapping(raw.get('source_pose'), 'stop_goal_pose_source_missing')
        _require(_integer(self.source.get('captured_at'), 'stop_goal_pose_source_mismatch') == raw['pose_source_at']
                 and self.source.get('frame_id') == raw['world_frame'],
                 'stop_goal_pose_source_mismatch')
        for key in ('speed_mps', 'yaw_rate_radps', 'quality'):
            _number(self.source.get(key), 'stop_goal_pose_measurement_missing')
        _require(contract.pose['min_quality'] <= self.source['quality'] <= 1, 'stop_goal_pose_quality_invalid')
        _require(self.source['speed_mps'] >= -contract.pose['speed_error_mps'], 'stop_goal_reverse_motion_unverified')
        _require(raw.get('footprint') == contract.footprint, 'stop_goal_footprint_origin_or_size_mismatch')
        for key in ('position_error_m', 'heading_error_rad', 'region_error_m', 'goal_tolerance_m',
                    'goal_heading_tolerance_rad', 'stopped_corner_speed_mps'):
            _number(raw.get(key), 'stop_goal_error_or_tolerance_missing', 0)
        self.pose = _pose(self.source.get('pose'))
        self.yaw_rate_right_radps = -self.source['yaw_rate_radps']
        self.final_pose = _pose(raw.get('final_pose'))  # The sole left-to-right conversion.
        self.region = _pose(raw.get('observed_region_pose'))
        shape = _mapping(raw.get('region_geometry'), 'stop_goal_region_geometry_missing')
        _require(shape.get('shape') == 'line_region', 'stop_goal_region_geometry_missing')
        self.width = _number(shape.get('lateral_half_width_m'), 'stop_goal_region_geometry_invalid', .001)
        self.depth = _number(shape.get('depth_m'), 'stop_goal_region_geometry_invalid', .001)
        _require(raw['region_error_m'] >= raw['position_error_m']+math.hypot(self.width, self.depth)*raw['heading_error_rad']-1e-9,
                 'stop_goal_region_error_inconsistent')
        boundary = _mapping(raw.get('permission_boundary'), 'stop_goal_boundary_missing')
        origin = _mapping(boundary.get('origin'), 'stop_goal_boundary_missing')
        normal = _mapping(boundary.get('normal'), 'stop_goal_boundary_missing')
        self.boundary_origin = (_number(origin.get('x_m'), 'stop_goal_boundary_missing'),
                                -_number(origin.get('y_m'), 'stop_goal_boundary_missing'))
        self.normal = (_number(normal.get('x_m'), 'stop_goal_boundary_missing'),
                       -_number(normal.get('y_m'), 'stop_goal_boundary_missing'))
        _require(abs(math.hypot(*self.normal)-1) <= 1e-6, 'stop_goal_boundary_normal_invalid')
        self.projection_limit = _number(boundary.get('max_projection_m'), 'stop_goal_boundary_missing')
        expected_limit = (0 if raw['element_kind'] == 'crosswalk' else self.depth)-raw['region_error_m']
        _require(abs(self.projection_limit-expected_limit) <= 1e-9
                 and math.dist(self.boundary_origin, (self.region['x_m'], self.region['y_m'])) <= 1e-9
                 and math.dist(self.normal, (math.cos(self.region['yaw_rad']), math.sin(self.region['yaw_rad']))) <= 1e-6,
                 'stop_goal_boundary_geometry_mismatch')
        bounds = boundary.get('lateral_bounds_m')
        _require(bounds is None or (isinstance(bounds, list) and len(bounds) == 2
                 and all(type(v) in (float, int) and math.isfinite(v) for v in bounds) and bounds[0] < bounds[1]),
                 'stop_goal_lateral_bounds_invalid')
        self.lateral_bounds = None if bounds is None else [-bounds[1], -bounds[0]]
        self.radius = math.hypot(max(contract.footprint['front_m'], contract.footprint['rear_m']), contract.footprint['half_width_m'])
        self.pose_margin = (contract.pose['position_error_m']+self.radius*contract.pose['heading_error_rad']
                            +contract.geometry['measurement_error_bound_m'])
        self.permission = copy.deepcopy(_mapping(record.get('completion_permissions', {}), 'stop_goal_permissions_invalid'))
        self.check_fresh(now)
        if real and raw['element_kind'] == 'finish_marker':
            _require(self.permission_fresh(now), 'stop_goal_finish_light_clearance_missing')
        _require(self.permitted(self.final_pose), 'stop_goal_final_footprint_not_permitted')
        _require(raw['element_kind'] == 'crosswalk' or self.inside(self.final_pose), 'stop_goal_final_footprint_outside_region')

    def check_fresh(self, now):
        low, high = self.contract.now_interval(now)
        raw = self.raw
        _require(raw['generated_at'] <= low and high < raw['expires_at'], 'stop_goal_expired_or_future')
        for key, age in [('pose_source_at', self.contract.pose['max_age_ms']),
                         ('geometry_source_at', self.contract.budget['max_geometry_age_ms']),
                         ('visual_source_at', self.contract.budget['max_visual_age_ms'])]:
            _require(0 <= high-raw[key] < age, 'stop_goal_source_stale')

    def corners(self, pose):
        c, s = math.cos(pose['yaw_rad']), math.sin(pose['yaw_rad'])
        f = self.contract.footprint
        return [(pose['x_m']+c*x-s*y, pose['y_m']+s*x+c*y)
                for x in (f['front_m'], -f['rear_m']) for y in (f['half_width_m'], -f['half_width_m'])]

    def permitted(self, pose):
        ox, oy = self.boundary_origin
        nx, ny = self.normal
        for x, y in self.corners(pose):
            dx, dy = x-ox, y-oy
            if nx*dx+ny*dy+self.pose_margin > self.projection_limit:
                return False
            if self.lateral_bounds is not None:
                lateral = -ny*dx+nx*dy
                if (lateral-self.pose_margin < self.lateral_bounds[0]
                        or lateral+self.pose_margin > self.lateral_bounds[1]):
                    return False
        return True

    def inside(self, pose):
        c, s = math.cos(self.region['yaw_rad']), math.sin(self.region['yaw_rad'])
        margin = self.raw['region_error_m']+self.pose_margin
        for x, y in self.corners(pose):
            dx, dy = x-self.region['x_m'], y-self.region['y_m']
            along, lateral = c*dx+s*dy, -s*dx+c*dy
            if not margin <= along <= self.depth-margin or abs(lateral) > self.width-margin:
                return False
        return True

    def summary(self):
        return {'goal_id': self.identity[0], 'task_revision': self.identity[1], 'element_kind': self.raw['element_kind'],
                'final_pose_right': self.final_pose, 'pose_source_at': self.raw['pose_source_at'],
                'expires_at': self.raw['expires_at'], 'simulation_only': self.raw['simulation_only'],
                'physical_control_ready': self.raw['physical_control_ready'],
                'coordinate_convention': 'x_forward_y_right_yaw_right_positive',
                'reference_origin': 'lidar_rotation_center'}

    def permission_fresh(self, now):
        permission = self.permission
        low, high = self.contract.now_interval(now)
        at = permission.get('source_at')
        return (permission.get('light_cleared') is True and permission.get('verified') is True
                and permission.get('goal_id') == self.identity[0]
                and type(permission.get('task_revision')) is int and permission['task_revision'] == self.identity[1]
                and type(permission.get('track_id')) is int and permission['track_id'] == self.identity[2]
                and permission.get('clock_epoch_id') == self.contract.clock['epoch_id']
                and type(at) is int and 0 <= at <= low and high-at < self.contract.pose['max_age_ms'])


class StopGoalConsumer:
    def __init__(self, goal, initial_pwm):
        _require(type(initial_pwm) is int and MIN_FORWARD_PWM <= initial_pwm <= MAX_PWM, 'stop_goal_initial_pwm_invalid')
        self.goal = goal
        self.pwm = initial_pwm
        self.last_pwm_change = float('-inf')
        self.neutral_latched = False
        self.hold_since = self.hold_last = self.last_pose = None
        self.hold_observations = 0
        self.completed = False

    def update(self, goal):
        old = self.goal
        _require(goal.identity == old.identity, 'stop_goal_identity_changed')
        _require(all(goal.raw[k] == old.raw[k] for k in ('element_kind', 'stop_semantics', 'footprint',
            'goal_tolerance_m', 'goal_heading_tolerance_rad', 'required_hold_ms', 'stopped_corner_speed_mps')),
            'stop_goal_contract_changed_within_task')
        keys = ('pose_source_at', 'geometry_source_at', 'visual_source_at', 'generated_at')
        _require(all(goal.raw[k] >= old.raw[k] for k in keys), 'stop_goal_source_reordered')
        if goal.raw['pose_source_at'] == old.raw['pose_source_at']:
            _require(goal.source == old.source, 'stop_goal_pose_changed_without_new_source')
        if goal.raw['geometry_source_at'] == old.raw['geometry_source_at']:
            _require(all(goal.raw[k] == old.raw[k] for k in ('final_pose', 'observed_region_pose', 'region_geometry',
                     'permission_boundary', 'position_error_m', 'heading_error_rad', 'region_error_m')),
                     'stop_goal_geometry_changed_without_new_source')
        shift = math.dist((goal.final_pose['x_m'], goal.final_pose['y_m']), (old.final_pose['x_m'], old.final_pose['y_m']))
        _require(shift <= goal.contract.budget['max_goal_update_shift_m'], 'stop_goal_geometry_jump')
        if all(goal.raw[k] == old.raw[k] for k in keys):
            return False  # Reads/heartbeats neither renew the old lease nor add hold observations.
        if any(goal.raw[k] != old.raw[k] for k in ('final_pose', 'observed_region_pose', 'region_geometry',
                   'permission_boundary', 'position_error_m', 'heading_error_rad', 'region_error_m')):
            # A revised stop region cannot inherit a hold certified against a
            # different boundary. Wait for new pose evidence in the new region.
            self.hold_since = self.hold_last = None
            self.hold_observations, self.completed = 0, False
        self.goal = goal
        return True

    def tick(self, now, phase, walls):
        goal = self.goal
        goal.check_fresh(now)
        pose, target, budget = goal.pose, goal.final_pose, goal.contract.budget
        dx, dy = target['x_m']-pose['x_m'], target['y_m']-pose['y_m']
        c, s = math.cos(pose['yaw_rad']), math.sin(pose['yaw_rad'])
        remaining = c*dx+s*dy
        # Lateral route error belongs to the target line, not the current yaw:
        # small correctable yaw error must not grow into distance*sin(error).
        lateral = -math.sin(target['yaw_rad'])*dx+math.cos(target['yaw_rad'])*dy
        heading = _wrap(target['yaw_rad']-pose['yaw_rad'])
        _require(abs(heading) <= budget['max_heading_delta_rad'] and abs(lateral) <= budget['max_lateral_error_m'],
                 'stop_goal_requires_turn_or_lateral_route')
        _require(goal.permitted(pose), 'stop_goal_permission_boundary_crossed')
        _require(remaining >= -goal.raw['goal_tolerance_m'], 'stop_goal_overshot')
        speed = abs(goal.source['speed_mps'])+goal.contract.pose['speed_error_mps']
        prospective_speed = max(speed, goal.contract.command['speed_upper_mps'])
        neutral_distance = goal.contract.neutral_distance(prospective_speed)
        distance_lower = remaining-goal.raw['position_error_m']-goal.pose_margin
        _, source_now_high = goal.contract.now_interval(now)
        source_age_ms = source_now_high-goal.raw['pose_source_at']
        allowance = (neutral_distance+prospective_speed*(budget['response_delay_ms']+source_age_ms)/1000
                     +budget['safety_margin_m'])
        if phase == 'coast' or distance_lower <= allowance:
            self.neutral_latched = True
        arrival = (math.hypot(dx, dy)+goal.contract.pose['position_error_m'] <= goal.raw['goal_tolerance_m']
                   and abs(heading)+goal.contract.pose['heading_error_rad'] <= goal.raw['goal_heading_tolerance_rad']
                   and (goal.raw['element_kind'] == 'crosswalk' or goal.inside(pose)))
        corner_speed = speed+(abs(goal.yaw_rate_right_radps)+goal.contract.pose['yaw_rate_error_radps'])*goal.radius
        if goal.raw['element_kind'] == 'finish_marker':
            arrival = arrival and self.permission_fresh(now)
        source_at = goal.raw['pose_source_at']
        if phase != 'coast' or not arrival or corner_speed > goal.raw['stopped_corner_speed_mps']:
            self.hold_since = self.hold_last = None
            self.hold_observations, self.completed = 0, False
        if source_at != self.last_pose:
            self.last_pose = source_at
            if phase == 'coast' and arrival and corner_speed <= goal.raw['stopped_corner_speed_mps']:
                if self.hold_last is None or source_at-self.hold_last > goal.contract.pose['max_gap_ms']:
                    self.hold_since, self.hold_observations = source_at, 0
                self.hold_last = source_at
                self.hold_observations += 1
                self.completed = self.hold_observations >= 3 and source_at-self.hold_since >= goal.raw['required_hold_ms']
            else:
                self.hold_since = self.hold_last = None
                self.hold_observations, self.completed = 0, False
        cap = MIN_FORWARD_PWM if distance_lower <= allowance+budget['slowdown_distance_m'] else MAX_PWM
        if not self.neutral_latched and cap < self.pwm and now-self.last_pwm_change >= .10:
            self.pwm = max(cap, self.pwm-5)
            self.last_pwm_change = now
        motor = 1500 if self.neutral_latched else self.pwm
        _require(motor == 1500 or walls is not None, 'stop_goal_wall_feedback_unavailable')
        if motor != 1500:
            _require(abs(math.radians(walls['heading_right_deg'])-heading) <= budget['max_heading_delta_rad'],
                     'stop_goal_corridor_heading_mismatch')
        return {'motor_cap': motor, 'request_coast': self.neutral_latched, 'completed': self.completed,
                'remaining_goal_m': remaining, 'neutral_distance_budget_m': allowance,
                'prospective_speed_upper_mps': prospective_speed,
                'arrival': arrival, 'corner_speed_upper_mps': corner_speed, 'hold_observations': self.hold_observations,
                'hold_source_ms': 0 if self.hold_since is None else max(0, source_at-self.hold_since),
                'goal': goal.summary()}

    def permission_fresh(self, now):
        return self.goal.permission_fresh(now)
