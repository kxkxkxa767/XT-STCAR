"""Bounded physical bring-up only. No competition planner or simulated pose."""
import math

MAX_PWM = 1530
MAX_DURATION_MS = 500
HEARTBEAT_S = .20


def probe_clearance(scan, ages, demo=False):
    """Fail closed on unknown front bins; coarse scans are for this probe only."""
    if any(not isinstance(ages.get(k), (int, float))
           or not math.isfinite(ages[k]) or not 0 <= ages[k] < limit
           for k, limit in [('camera', .25), ('lidar', .15), ('control', .08)]):
        raise ValueError('probe_sensor_stale')
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
    if len(valid) < 342:
        raise ValueError('probe_scan_incomplete')
    front = [bins[i % 360] for i in range(-30, 31)]
    if any(r is None for r in front):
        raise ValueError('probe_front_unknown')
    clearance = {'front_m': min(front), 'nearest_m': min(valid), 'scan_seq': scan['seq']}
    if clearance['front_m'] < 1.0 or clearance['nearest_m'] < .40:
        raise ValueError('probe_obstacle_close')
    return clearance


def probe_parameters(data):
    pwm, duration = data.get('pwm'), data.get('duration_ms')
    if type(pwm) is not int or not 1501 <= pwm <= MAX_PWM:
        raise ValueError('probe_pwm_must_be_1501_to_1530')
    if type(duration) is not int or not 100 <= duration <= MAX_DURATION_MS:
        raise ValueError('probe_duration_must_be_100_to_500_ms')
    return pwm, duration
