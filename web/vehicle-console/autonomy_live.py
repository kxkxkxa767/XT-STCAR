"""Bounded physical bring-up only. No competition planner or simulated pose."""
import math
import statistics

MIN_FORWARD_PWM = 1550
MAX_PWM = 1580
MAX_DURATION_MS = 500
HEARTBEAT_S = .20
QUALITY_CONFIRM_S = 2.0
QUALITY_RECOVERY_STABLE_S = .30
QUALITY_RECOVERY_MIN_SCANS = 3
COAST_MAX_S = 5.0
CAMERA_LATE_S = .50
LIDAR_LATE_S = .30
CONTROL_AGE_LIMIT_S = .20
AUTO_CONTROL_HEALTH_S = .25
FRONT_MAX_UNKNOWN = 6
FRONT_MAX_GAP = 3
CORRIDOR_HALF_WIDTH_M = .30
CORRIDOR_LOOKAHEAD_M = 1.0
# Operator's revised rough measurement, explicitly resolved to 28 cm total
# width on 2026-10-07. No calibrated measurement-error bound was supplied.
SIDE_BODY_EXTENT_M = .14
SIDE_MIN_NET_M = .10
LIDAR_RANGE_ALLOWANCE_M = .03  # Manufacturer's coarse 0..6 m accuracy reference.
SIDE_CLEARANCE_M = SIDE_BODY_EXTENT_M + SIDE_MIN_NET_M + LIDAR_RANGE_ALLOWANCE_M
# Measured lidar-to-body extents; x is forward and y is right in this frame.
FRONT_BODY_EXTENT_M = .20
REAR_BODY_EXTENT_M = .18
MANEUVER_MIN_NET_M = .05
MANEUVER_BODY_CLEARANCE_M = MANEUVER_MIN_NET_M + LIDAR_RANGE_ALLOWANCE_M
# Operator authorized a smaller inside-left turn gate after trial17. This
# experimental 4 cm total still includes the original 3 cm range reference;
# it is not a certified physical clearance or geometry-error allowance.
TURNING_LEFT_MIN_NET_M = .01
TURNING_LEFT_BODY_CLEARANCE_M = TURNING_LEFT_MIN_NET_M + LIDAR_RANGE_ALLOWANCE_M
# Vehicle bring-up limits, never a remembered course length or corner position.
LAUNCH_HOLD_S = .6
STRAIGHT_CRUISE_PWM = 1570
COAST_TIME_MARGIN_S = 2.5  # Conservative test allowance; not a calibrated brake model.
FRONT_BOUNDARY_LOSS_S = .70  # Fresh scans may temporarily lose the fitted plane; not sensor dropout.
APPROACH_SLOW_EXTRA_S = 1.5
STEERING_LIMIT_PWM = 55


def corridor_walls(scan):
    """Fit two extended parallel boards; isolated cones are not corridor walls."""
    bins = scan['ranges']
    fits = []
    for sector in (range(45, 136), range(225, 316)):
        points = [(r * math.cos(math.radians(a)), r * math.sin(math.radians(a)))
                  for a in sector if (r := bins[a]) is not None]
        if len(points) < 40:
            return None
        slopes = [(points[j][1]-y1)/(points[j][0]-x1) for i, (x1, y1) in enumerate(points)
                  for j in range(i+8, len(points), 8) if abs(points[j][0]-x1) >= .3]
        if not slopes:
            return None
        slope = statistics.median(slopes)
        intercept = statistics.median(y-slope*x for x, y in points)
        inliers = [(x, y) for x, y in points if abs(y-slope*x-intercept) <= .025]
        if len(inliers) < .80*len(points) or max(x for x, _ in inliers)-min(x for x, _ in inliers) < .25:
            return None
        mean_x = statistics.mean(x for x, _ in inliers)
        mean_y = statistics.mean(y for _, y in inliers)
        variance = sum((x-mean_x)**2 for x, _ in inliers)
        slope = sum((x-mean_x)*(y-mean_y) for x, y in inliers)/variance
        intercept = mean_y-slope*mean_x
        if abs(slope) > math.tan(math.radians(15)):
            return None
        fits.append((slope, intercept))
    (right_a, right_b), (left_a, left_b) = fits
    if (not left_b < 0 < right_b or not .65 <= right_b-left_b <= 2.5
            or abs(math.atan(right_a)-math.atan(left_a)) > math.radians(12)):
        return None
    slope = (right_a+left_a)/2
    return {'offset_right_m': (right_b+left_b)/2,
            'heading_right_deg': math.degrees(math.atan(slope)),
            'slope': slope, 'width_m': right_b-left_b}


class CorridorSteering:
    """Wall feedback with hysteresis, trend damping and a faster neutral return."""
    def __init__(self):
        self.servo = 1500
        self.correcting = False
        self.changed_at = float('-inf')
        self.last_scan = None
        self.last_publication = None
        self.history = []
        self.direction = 0
        self.reverse_count = 0

    def update(self, walls, scan_seq, now, motion_ready=True, scan_at_ms=None):
        if not motion_ready or walls is None:
            self.servo, self.correcting = 1500, False
            self.changed_at = now
            self.history = []
            self.direction, self.reverse_count = 0, 0
            return self.servo
        if type(scan_seq) is not int or (self.last_scan is not None and scan_seq < self.last_scan):
            self.history = []
            self.servo, self.correcting = 1500, False
            self.direction, self.reverse_count = 0, 0
            self.changed_at = now
            return self.servo
        if scan_seq == self.last_scan:
            return self.servo
        published = scan_at_ms/1000 if type(scan_at_ms) is int and scan_at_ms >= 0 else now
        if self.last_publication is not None and published <= self.last_publication:
            self.history = []
            self.servo, self.correcting = 1500, False
            self.direction, self.reverse_count = 0, 0
            self.changed_at = now
            return self.servo
        self.last_scan, self.last_publication = scan_seq, published
        offset, heading = walls['offset_right_m'], walls['heading_right_deg']
        projected_offset = offset + CORRIDOR_LOOKAHEAD_M * walls['slope']
        # Whole-scan publication clock. This is wall-motion evidence, not an IMU yaw rate.
        previous = self.history[-1] if self.history else None
        if previous and (not 0 < published-previous[0] < LIDAR_LATE_S
                         or abs(walls['width_m']-previous[3]) > .1
                         or abs(offset-previous[2]) > .15 or abs(heading-previous[1]) > 8):
            self.history = []
            self.reverse_count = 0
        self.history.append((published, heading, offset, walls['width_m']))
        self.history = self.history[-3:]
        damped_offset = projected_offset
        if len(self.history) == 3 and all(b[0]-a[0] >= .05 for a, b in zip(self.history, self.history[1:])):
            rates = [math.radians(b[1]-a[1])/(b[0]-a[0])
                     for a, b in zip(self.history, self.history[1:])]
            if max(abs(r) for r in rates) <= math.radians(30):
                candidate = projected_offset + .15*CORRIDOR_LOOKAHEAD_M*statistics.median(rates)
                # A noisy derivative may release the old turn; it cannot invent its opposite.
                if candidate*projected_offset <= 0:
                    damped_offset = 0
                elif abs(candidate) < abs(projected_offset):
                    damped_offset = candidate
        usable_half_width = max(.01, walls['width_m']/2-SIDE_CLEARANCE_M)
        if not self.correcting:
            self.correcting = (abs(offset) > min(.08, usable_half_width*.5) or abs(heading) > 4
                               or (abs(heading) > 1.5 and abs(projected_offset) > min(.05, usable_half_width*.3)))
        elif (abs(offset) < min(.04, usable_half_width*.25) and abs(heading) < 2
              and abs(projected_offset) < min(.035, usable_half_width*.2)):
            self.correcting = False
        target = 1500
        urgent = False
        if self.correcting:
            # Positive y is right. Lower PWM turns right (user's physical check).
            correction = round(STEERING_LIMIT_PWM*max(-1, min(1, damped_offset/usable_half_width)))
            target -= correction
            urgent = usable_half_width-abs(projected_offset) < SIDE_MIN_NET_M
        sign = (target > 1500)-(target < 1500)
        if sign and self.direction and sign != self.direction:
            self.reverse_count += 1
            if self.servo != 1500 or self.reverse_count < 2:
                target = 1500  # Release the old turn before growing the opposite turn.
            else:
                self.direction, self.reverse_count = sign, 0
        else:
            self.reverse_count = 0
            if sign:
                self.direction = sign
        interval, step = (.15, 20) if urgent else (.25, 15)
        if abs(target-1500) < abs(self.servo-1500):
            # A 99 ms published scan must not wait for another full scan just
            # because it missed a 100 ms staircase. Bound return by elapsed time.
            interval = .05
            step = math.floor(130*min(.1, max(0, now-self.changed_at))+1e-9)
        if now-self.changed_at >= interval and target != self.servo:
            self.servo += max(-step, min(step, target-self.servo))
            self.changed_at = now
        return self.servo


class RearLaunch:
    """At most one second after observed positive motor output; no turning in grace."""
    def __init__(self):
        self.active = True
        self.since = None

    def update(self, scan, now, moving):
        if not self.active:
            return False
        bins = scan.get('ranges') if isinstance(scan, dict) else None
        if isinstance(bins, list) and len(bins) == 360:
            sector = bins[115:246]
            known = [r for r in sector if type(r) in (int, float) and math.isfinite(r)]
            if len(known) >= .95*len(sector) and all(r >= .4 for r in known):
                self.active = False
        if moving and self.since is None:
            self.since = now
        if self.since is not None and now-self.since >= 1:
            self.active = False
        return self.active


class QualityLatch:
    """Diagnostic duration of a quality streak; never a reason to rearm."""
    def __init__(self):
        self.since = None

    def update(self, issues, now):
        if not issues:
            self.since = None
        elif self.since is None:
            self.since = now
        return self.since is not None and now - self.since >= QUALITY_CONFIRM_S

    def elapsed_ms(self, now):
        return 0 if self.since is None else round(max(0, now - self.since) * 1000)


class QualityRecovery:
    """Resume only an existing neutral-waiting session after stable fresh evidence."""
    def __init__(self):
        self.waiting = False
        self.good_since = None
        self.good_scans = 0
        self.last_seq = None
        self.last_good_at = None

    def update(self, motion_ready, issues, scan_seq, now):
        if not motion_ready:
            self.waiting = True
            self.good_since = None
            self.good_scans = 0
            self.last_seq = scan_seq
            self.last_good_at = None
            return False
        if not self.waiting:
            return True
        if issues or type(scan_seq) is not int:
            self.good_since = None
            self.good_scans = 0
            self.last_seq = scan_seq
            self.last_good_at = None
            return False
        if self.last_seq is not None and scan_seq <= self.last_seq:
            if scan_seq < self.last_seq:
                self.good_since = None
                self.good_scans = 0
                self.last_seq = scan_seq
                self.last_good_at = None
            return False
        self.last_seq = scan_seq
        if self.last_good_at is not None and now-self.last_good_at >= LIDAR_LATE_S:
            self.good_since = None
            self.good_scans = 0
        self.last_good_at = now
        if self.good_since is None:
            self.good_since = now
        self.good_scans += 1
        if self.good_scans >= QUALITY_RECOVERY_MIN_SCANS and now-self.good_since >= QUALITY_RECOVERY_STABLE_S:
            self.waiting = False
            return True
        return False

    def stable_ms(self, now):
        return 0 if self.good_since is None else round(max(0, now-self.good_since)*1000)


class ApproachRamp:
    """Vehicle launch/cruise plus measured wall closing time; no course coordinates."""
    def __init__(self, pwm):
        if type(pwm) is not int or not MIN_FORWARD_PWM <= pwm <= MAX_PWM:
            raise ValueError('invalid_forward_pwm')
        self.pwm = pwm
        self.initial = pwm
        self.last_seq = -1
        self.last_change = float('-inf')
        self.closest_front = None
        self.changes = 0
        self.lane_limited = False
        self.stop_requested = False
        self.launch_since = None
        self.cruise_limited = False
        self.previous_boundary = None
        self.closing_rates = []
        self.closing_speed = None
        self.time_to_clearance = None
        self.boundary_lost = False
        self.boundary_cap = pwm
        self.boundary_seen = False

    def update(self, scan, age, now, correcting=False, moving=True):
        if not isinstance(scan, dict) or type(age) not in (int, float) or not 0 <= age < .3:
            return self.pwm
        seq = scan.get('seq')
        if type(seq) is not int or seq <= self.last_seq:
            return self.pwm
        self.last_seq = seq
        if moving and self.launch_since is None:
            self.launch_since = now
        distance = scan.get('front_boundary_m')
        target = self.initial
        launch_held = self.launch_since is None or now-self.launch_since < LAUNCH_HOLD_S
        if type(distance) in (int, float) and math.isfinite(distance) and .6 <= distance <= 12:
            self.boundary_lost = False
            self.boundary_seen = True
            self.closest_front = distance if self.closest_front is None else min(self.closest_front, distance)
            previous = self.previous_boundary
            # Native at_ms is whole-scan publication time, not per-ray acquisition.
            published_ms = scan.get('at_ms')
            captured = published_ms/1000 if type(published_ms) is int and published_ms >= 0 else now
            continuous = previous is not None and .05 <= captured-previous[1] <= .35
            if continuous:
                rate = (previous[0]-distance)/(captured-previous[1])
                continuous = -.4 <= rate <= 3
            if continuous:
                self.closing_rates.append(max(0, rate))
                self.closing_rates = self.closing_rates[-3:]
            else:
                self.closing_rates = []
            self.previous_boundary = (distance, captured, now)
            self.closing_speed = None
            self.time_to_clearance = None
            if len(self.closing_rates) >= 2:
                speed = statistics.median(self.closing_rates)
                if max(self.closing_rates)-min(self.closing_rates) <= max(.4, speed*.5):
                    self.closing_speed = speed
                    if speed > .05:
                        self.time_to_clearance = max(0, (distance-CORRIDOR_LOOKAHEAD_M)/speed)
                        self.stop_requested |= self.time_to_clearance <= COAST_TIME_MARGIN_S
                        fraction = max(0, min(1, (self.time_to_clearance-COAST_TIME_MARGIN_S)/APPROACH_SLOW_EXTRA_S))
                        target = 1500+round((self.initial-1500)*fraction)
            if self.closing_speed is None:
                target = min(target, MIN_FORWARD_PWM)
            self.boundary_cap = min(self.boundary_cap, max(MIN_FORWARD_PWM, target))
        elif self.previous_boundary is not None and now-self.previous_boundary[2] > FRONT_BOUNDARY_LOSS_S:
            self.previous_boundary = None
            self.closing_rates = []
            self.closing_speed = None
            self.time_to_clearance = None
            self.boundary_lost = True
        if self.boundary_lost:
            self.stop_requested = True  # Lost a known boundary; request neutral, never sub-floor drive.
        if self.stop_requested:
            return self.pwm
        if not launch_held:
            target = min(target, STRAIGHT_CRUISE_PWM)
            self.cruise_limited = True
        if correcting and not launch_held:
            target = min(target, MIN_FORWARD_PWM)
            self.lane_limited = True
        target = max(MIN_FORWARD_PWM, min(target, self.boundary_cap))
        if target < self.pwm and now-self.last_change >= .10:
            self.pwm = max(target, self.pwm-5)
            self.last_change = now
            self.changes += 1
        elif moving and target > self.pwm and not self.boundary_seen and now-self.last_change >= .20:
            self.pwm = min(target, self.pwm+2)
            self.last_change = now
            self.changes += 1
        return self.pwm


class JunctionStop:
    """Per-session endpoint gate; distinct fresh native scans, never YOLO or map labels."""
    def __init__(self):
        self.last_seq = -1
        self.last_at = None
        self.previous = None
        self.count = 0

    def update(self, scan, age, now, motion_ready):
        geometry = scan.get('left_junction')
        fields = ['front_wall_m', 'incoming_left_end_m', 'incoming_width_m',
                  'outgoing_width_m', 'heading_left_rad', 'known_open_fraction']
        valid = (motion_ready and type(age) in (int, float) and 0 <= age < LIDAR_LATE_S and isinstance(geometry, dict)
                 and geometry.get('turn_path_certified') is False
                 and all(type(geometry.get(k)) in (int, float) and math.isfinite(geometry[k]) for k in fields)
                 and .75 <= geometry['incoming_width_m'] <= 2.2
                 and .75 <= geometry['outgoing_width_m'] <= 2.25
                 and .9 <= geometry['known_open_fraction'] <= 1
                 and abs(geometry['heading_left_rad']) <= math.radians(15)
                 and .6 <= geometry['front_wall_m'] <= 3
                 and -.3 <= geometry['incoming_left_end_m'] <= 1)
        if not valid:
            self.count, self.previous = 0, None
            return False
        seq = scan['seq']
        if seq <= self.last_seq:
            if seq < self.last_seq:
                self.count, self.previous = 0, None
            return False
        self.last_seq = seq
        continuous = self.previous is not None and now-self.last_at < .3
        if continuous:
            previous = self.previous
            df = geometry['front_wall_m']-previous['front_wall_m']
            de = geometry['incoming_left_end_m']-previous['incoming_left_end_m']
            continuous = (abs(df-de) < .15 and abs(df) < 3*(now-self.last_at)+.08
                          and abs(geometry['incoming_width_m']-previous['incoming_width_m']) < .15
                          and abs(geometry['outgoing_width_m']-previous['outgoing_width_m']) < .15
                          and abs(geometry['heading_left_rad']-previous['heading_left_rad']) < math.radians(8))
        self.count = self.count+1 if continuous else 1
        self.previous, self.last_at = dict(geometry), now
        return self.count >= 3 and geometry['incoming_left_end_m'] <= .40


class ProbeClearanceError(ValueError):
    """A validated known-body stop carries the current scan's complete diagnostics."""
    def __init__(self, reason, clearance):
        super().__init__(reason)
        self.clearance = clearance


def probe_clearance(scan, ages, demo=False, centering=False, rear_launch=False, camera_required=True,
                    clearance_profile='straight', relaxed_turn_left=False):
    """Keep nulls explicit; permit bounded holes, never ignore a known close return."""
    if clearance_profile not in ('straight', 'maneuver') or type(clearance_profile) is not str:
        raise ValueError('invalid_clearance_profile')
    if type(camera_required) is not bool:
        raise ValueError('invalid_camera_requirement')
    if type(relaxed_turn_left) is not bool or (relaxed_turn_left and clearance_profile != 'maneuver'):
        raise ValueError('invalid_relaxed_turn_clearance')
    sensor_inputs = ['camera', 'lidar', 'control'] if camera_required else ['lidar', 'control']
    if any(type(ages.get(k)) not in (int, float) or not math.isfinite(ages[k]) or ages[k] < 0
           for k in sensor_inputs):
        raise ValueError('probe_invalid_sensor_age')
    if ages['control'] >= CONTROL_AGE_LIMIT_S:
        raise ValueError('probe_sensor_unavailable')
    issues = []
    if camera_required and ages['camera'] >= CAMERA_LATE_S:
        issues.append('camera_late')
    if camera_required and ages['camera'] >= 1:
        issues.append('camera_unavailable')
    if ages['lidar'] >= LIDAR_LATE_S:
        issues.append('lidar_late')
    if not isinstance(scan, dict) or type(scan.get('seq')) is not int:
        raise ValueError('probe_invalid_scan')
    if not demo and scan.get('frame_id') != 'lidar_origin_coarse_body_heading':
        raise ValueError('probe_unknown_lidar_heading')
    bins = scan.get('ranges')
    if not isinstance(bins, list) or len(bins) != 360:
        raise ValueError('probe_invalid_scan')
    valid = []
    for r in bins:
        if r is None:
            continue
        if type(r) not in (int, float) or not math.isfinite(r) or not .02 <= r <= 12:
            raise ValueError('probe_invalid_scan')
        valid.append(r)
    # A known obstacle remains a hard stop even while coverage is degraded.
    nearest = min(valid) if valid else None
    corridor = []
    side_clearances = []
    maneuver = clearance_profile == 'maneuver'
    half_width = CORRIDOR_HALF_WIDTH_M + (.02 if centering else 0)
    nearest_body_point = None
    stop_trigger_point = None
    body_stop_requested = False
    for angle, r in enumerate(bins):
        if r is None:
            continue
        x, y = r * math.cos(math.radians(angle)), r * math.sin(math.radians(angle))
        # Euclidean distance to the measured rectangle includes all four corners.
        # It describes current known returns, not unobserved space or a future sweep.
        dx = max(-REAR_BODY_EXTENT_M-x, 0, x-FRONT_BODY_EXTENT_M)
        dy = max(abs(y)-SIDE_BODY_EXTENT_M, 0)
        body_distance = math.hypot(dx, dy)
        # Only known returns beyond the left body side get the turn allowance.
        # Front-center, right and non-turn observations retain their old gate.
        relaxed_point = relaxed_turn_left and -y >= SIDE_BODY_EXTENT_M
        point_stop_m = TURNING_LEFT_BODY_CLEARANCE_M if relaxed_point else MANEUVER_BODY_CLEARANCE_M
        body_point = {'angle_deg': angle, 'range_m': r, 'forward_x_m': x, 'left_y_m': -y,
                      'body_distance_m': body_distance, 'stop_distance_m': point_stop_m,
                      'relaxed_turn_left': relaxed_point}
        if nearest_body_point is None or body_distance < nearest_body_point['body_distance_m']:
            nearest_body_point = body_point
        if maneuver:
            # Equality is allowed; tolerate only floating point roundoff at that boundary.
            if body_distance < point_stop_m and not math.isclose(
                    body_distance, point_stop_m, rel_tol=0, abs_tol=1e-12):
                body_stop_requested = True
                if (stop_trigger_point is None or body_distance-point_stop_m <
                        stop_trigger_point['body_distance_m']-stop_trigger_point['stop_distance_m']):
                    stop_trigger_point = body_point
            if abs(y) > abs(x):
                side_clearances.append(abs(y))
        elif rear_launch and 115 <= angle <= 245:
            pass  # Only a bounded neutral-steering forward launch may exempt rear rays.
        elif abs(y) > abs(x):
            side_clearances.append(abs(y))
            if abs(y) < SIDE_CLEARANCE_M:
                raise ValueError('probe_obstacle_close_side')
        elif r < .40:
            raise ValueError('probe_obstacle_close')
        if x > 0 and abs(y) <= half_width:
            corridor.append(x)
    corridor_front = min(corridor) if corridor else None
    if not maneuver and corridor_front is not None and corridor_front < CORRIDOR_LOOKAHEAD_M:
        raise ValueError('probe_obstacle_in_straight_corridor')
    if len(valid) < 342:
        issues.append('scan_incomplete')
    front = [bins[i % 360] for i in range(-30, 31)]
    unknown = [i - 30 for i, r in enumerate(front) if r is None]
    run = largest_gap = 0
    for r in front:
        run = run + 1 if r is None else 0
        largest_gap = max(largest_gap, run)
    if len(unknown) > FRONT_MAX_UNKNOWN or largest_gap > FRONT_MAX_GAP:
        issues.append('front_sparse')
    front_values = [r for r in front if r is not None]
    nearest_relaxed = nearest_body_point is not None and nearest_body_point['relaxed_turn_left']
    nearest_stop_m = TURNING_LEFT_BODY_CLEARANCE_M if nearest_relaxed else MANEUVER_BODY_CLEARANCE_M
    clearance = {'front_m': min(front_values) if front_values else None,
            'clearance_profile': clearance_profile,
            'clearance_gates': {'straight_corridor_active': not maneuver,
                                'side_lateral_active': not maneuver, 'radial_active': not maneuver,
                                'current_body_proximity_active': maneuver,
                                'relaxed_turn_left_active': relaxed_turn_left,
                                'rear_launch_exemption_active': bool(rear_launch) and not maneuver},
            'body_proximity': {
                'scope': 'current_known_lidar_returns_only',
                'coordinate_convention': 'x_forward_y_left',
                'ray_bin_convention': 'clockwise_from_forward',
                'body_extents_source': 'user_measured_lidar_to_front_rear_and_outer_tyre_edges',
                'body_extents_measurement_date': '2026-10-07',
                'body_extents_measurement_quality': 'operator_rough_measurement',
                'body_extents_error_bound_m': None,
                'body_extents_m': {'front': FRONT_BODY_EXTENT_M, 'rear': REAR_BODY_EXTENT_M,
                                   'left': SIDE_BODY_EXTENT_M, 'right': SIDE_BODY_EXTENT_M},
                'body_rectangle_m': {'x_min': -REAR_BODY_EXTENT_M, 'x_max': FRONT_BODY_EXTENT_M,
                                      'y_min': -SIDE_BODY_EXTENT_M, 'y_max': SIDE_BODY_EXTENT_M},
                'min_net_clearance_m': ((TURNING_LEFT_MIN_NET_M if nearest_relaxed else MANEUVER_MIN_NET_M)
                                        if maneuver else None),
                'min_net_clearance_source': (('operator_authorized_turn_left_total4cm' if nearest_relaxed
                                             else 'operator_selected_maneuver_5cm') if maneuver else None),
                'lidar_range_allowance_m': LIDAR_RANGE_ALLOWANCE_M,
                'stop_distance_m': nearest_stop_m if maneuver else None,
                'default_stop_distance_m': MANEUVER_BODY_CLEARANCE_M if maneuver else None,
                'relaxed_turn_left_active': relaxed_turn_left,
                'turning_left_stop_distance_m': TURNING_LEFT_BODY_CLEARANCE_M if relaxed_turn_left else None,
                'stop_requested': body_stop_requested if maneuver else None,
                'current_known_min_distance_m': (nearest_body_point['body_distance_m']
                                                 if nearest_body_point is not None else None),
                'nearest_point': nearest_body_point,
                'stop_trigger_point': stop_trigger_point,
                'axis_stop_thresholds_m': ({'front': FRONT_BODY_EXTENT_M+MANEUVER_BODY_CLEARANCE_M,
                                           'rear': REAR_BODY_EXTENT_M+MANEUVER_BODY_CLEARANCE_M,
                                           'left': SIDE_BODY_EXTENT_M+(TURNING_LEFT_BODY_CLEARANCE_M
                                                       if relaxed_turn_left else MANEUVER_BODY_CLEARANCE_M),
                                           'right': SIDE_BODY_EXTENT_M+MANEUVER_BODY_CLEARANCE_M}
                                          if maneuver else None),
                'free_space_certified': False, 'swept_path_certified': False,
                'standstill_certified': False},
            'legacy_diagnostic_only_fields': (['corridor_front_m', 'corridor_half_width_m',
                                               'corridor_lookahead_m', 'side_clearance_m',
                                               'side_stop_threshold_m'] if maneuver else []),
            'sensor_inputs': sensor_inputs, 'camera_required': camera_required,
            'corridor_front_m': corridor_front, 'nearest_m': nearest, 'scan_seq': scan['seq'],
            'side_clearance_m': min(side_clearances) if side_clearances else None,
            'side_stop_threshold_m': (SIDE_BODY_EXTENT_M+MANEUVER_BODY_CLEARANCE_M
                                      if maneuver else SIDE_CLEARANCE_M),
            'side_body_extent_m': SIDE_BODY_EXTENT_M,
            'side_min_net_m': MANEUVER_MIN_NET_M if maneuver else SIDE_MIN_NET_M,
            'lidar_range_allowance_m': LIDAR_RANGE_ALLOWANCE_M,
            'front_unknown_bins': unknown, 'largest_gap_deg': largest_gap,
            'quality_issues': issues,
            'motion_ready': not body_stop_requested and not any(
                i in issues for i in ['camera_unavailable', 'lidar_late', 'scan_incomplete', 'front_sparse']),
            'corridor_half_width_m': half_width,
            'corridor_lookahead_m': CORRIDOR_LOOKAHEAD_M}
    if body_stop_requested:
        raise ProbeClearanceError('probe_obstacle_close_body', clearance)
    return clearance


def probe_parameters(data, straight=False):
    pwm, duration = data.get('pwm'), data.get('duration_ms')
    if type(pwm) is not int or not MIN_FORWARD_PWM <= pwm <= MAX_PWM:
        raise ValueError(f'probe_pwm_must_be_{MIN_FORWARD_PWM}_to_{MAX_PWM}')
    limit = 30000 if straight else MAX_DURATION_MS
    if type(duration) is not int or not 100 <= duration <= limit:
        raise ValueError('straight_duration_must_be_100_to_30000_ms' if straight else 'probe_duration_must_be_100_to_500_ms')
    return pwm, duration
