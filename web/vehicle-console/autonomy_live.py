"""Bounded physical bring-up only. No competition planner or simulated pose."""
import math
import statistics

MAX_PWM = 1560
MAX_DURATION_MS = 500
HEARTBEAT_S = .20
QUALITY_CONFIRM_S = 2.0
CAMERA_LATE_S = .50
LIDAR_LATE_S = .30
CONTROL_AGE_LIMIT_S = .15
FRONT_MAX_UNKNOWN = 6
FRONT_MAX_GAP = 3
CORRIDOR_HALF_WIDTH_M = .30
CORRIDOR_LOOKAHEAD_M = 1.0
# User measured lidar centre to both outside tyre boundaries on 2026-10-05.
SIDE_BODY_EXTENT_M = .17
SIDE_MIN_NET_M = .10
LIDAR_RANGE_ALLOWANCE_M = .03  # Manufacturer's coarse 0..6 m accuracy reference.
SIDE_CLEARANCE_M = SIDE_BODY_EXTENT_M + SIDE_MIN_NET_M + LIDAR_RANGE_ALLOWANCE_M


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
        if len(inliers) < .80*len(points) or max(x for x, _ in inliers)-min(x for x, _ in inliers) < .5:
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
            or abs(math.atan(right_a)-math.atan(left_a)) > math.radians(4)):
        return None
    slope = (right_a+left_a)/2
    return {'offset_right_m': (right_b+left_b)/2,
            'heading_right_deg': math.degrees(math.atan(slope)),
            'slope': slope, 'width_m': right_b-left_b}


class CorridorSteering:
    """Small experimental PWM correction, with hysteresis and a minimum interval."""
    def __init__(self):
        self.servo = 1500
        self.correcting = False
        self.changed_at = float('-inf')
        self.last_scan = None

    def update(self, walls, scan_seq, now, motion_ready=True):
        if not motion_ready or walls is None:
            self.servo, self.correcting = 1500, False
            self.changed_at = now
            return self.servo
        if scan_seq == self.last_scan:
            return self.servo
        self.last_scan = scan_seq
        offset, heading = walls['offset_right_m'], walls['heading_right_deg']
        if not self.correcting:
            self.correcting = abs(offset) > .08 or abs(heading) > 4
        elif abs(offset) < .04 and abs(heading) < 2:
            self.correcting = False
        target = 1500
        if self.correcting:
            # Positive y is right. Lower PWM turns right (user's physical check).
            correction = round(max(-15, min(15, 100*(offset+.6*walls['slope']))))
            target -= correction
        if now-self.changed_at >= .35 and target != self.servo:
            self.servo += max(-5, min(5, target-self.servo))
            self.changed_at = now
        return self.servo


class QualityLatch:
    """Only perception-quality faults debounce; recovery resets the whole streak."""
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


def probe_clearance(scan, ages, demo=False, centering=False):
    """Keep nulls explicit; permit bounded holes, never ignore a known close return."""
    if any(type(ages.get(k)) not in (int, float) or not math.isfinite(ages[k]) or ages[k] < 0
           for k in ['camera', 'lidar', 'control']):
        raise ValueError('probe_invalid_sensor_age')
    if ages['control'] >= CONTROL_AGE_LIMIT_S:
        raise ValueError('probe_sensor_unavailable')
    issues = []
    if ages['camera'] >= CAMERA_LATE_S:
        issues.append('camera_late')
    if ages['camera'] >= 1:
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
    for angle, r in enumerate(bins):
        if r is None:
            continue
        x, y = r * math.cos(math.radians(angle)), r * math.sin(math.radians(angle))
        if abs(y) > abs(x):
            side_clearances.append(abs(y))
            if abs(y) < SIDE_CLEARANCE_M:
                raise ValueError('probe_obstacle_close_side')
        elif r < .40:
            raise ValueError('probe_obstacle_close')
        half_width = CORRIDOR_HALF_WIDTH_M + (.02 if centering else 0)
        if x > 0 and abs(y) <= half_width:
            corridor.append(x)
    corridor_front = min(corridor) if corridor else None
    if corridor_front is not None and corridor_front < CORRIDOR_LOOKAHEAD_M:
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
    return {'front_m': min(front_values) if front_values else None,
            'corridor_front_m': corridor_front, 'nearest_m': nearest, 'scan_seq': scan['seq'],
            'side_clearance_m': min(side_clearances) if side_clearances else None,
            'side_stop_threshold_m': SIDE_CLEARANCE_M,
            'side_body_extent_m': SIDE_BODY_EXTENT_M, 'side_min_net_m': SIDE_MIN_NET_M,
            'lidar_range_allowance_m': LIDAR_RANGE_ALLOWANCE_M,
            'front_unknown_bins': unknown, 'largest_gap_deg': largest_gap,
            'quality_issues': issues,
            'motion_ready': not any(i in issues for i in ['camera_unavailable', 'lidar_late', 'scan_incomplete', 'front_sparse']),
            'corridor_half_width_m': half_width,
            'corridor_lookahead_m': CORRIDOR_LOOKAHEAD_M}


def probe_parameters(data, straight=False):
    pwm, duration = data.get('pwm'), data.get('duration_ms')
    if type(pwm) is not int or not 1501 <= pwm <= MAX_PWM:
        raise ValueError('probe_pwm_must_be_1501_to_1560')
    limit = 30000 if straight else MAX_DURATION_MS
    if type(duration) is not int or not 100 <= duration <= limit:
        raise ValueError('straight_duration_must_be_100_to_30000_ms' if straight else 'probe_duration_must_be_100_to_500_ms')
    return pwm, duration
