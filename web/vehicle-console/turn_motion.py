"""Pure adapter for one supervised left-turn trial, never a navigation permit.

Native corridor geometry is X-forward/Y-left/yaw-left-positive. Publication
intervals describe observation trends only; local monotonic time controls PWM
slew, the settling allowance and trial deadlines. No device or actuator API.
"""
import copy
import math

SERVO_MIN = 1270
SERVO_MAX = 1720
INITIAL_PRESTEER_MIN = 1650
NEUTRAL = 1500
TRIAL_MOTOR = 1560
MAX_DRIVE_S = 10.0
MAX_PRESTEER_S = 5.0
STEERING_ALLOWANCE_S = 1.2
PWM_STEP = 10
PWM_INTERVAL_S = .10
SCAN_AGE_S = .30
COAST_SERVO_DELTA = 55
# Turn-only lidar-to-wall margin: measured half-width + clearance + range allowance.
TURN_LATERAL_MARGIN_M = .17+.05+.03
_FRAME = 'lidar_origin_coarse_body_heading'
_FIELDS = ('heading_left_rad', 'center_offset_left_m', 'width_m',
           'left_wall_points', 'right_wall_points', 'support_span_m',
           'fit_error_m', 'origin_between_walls', 'candidate_only',
           'turn_path_certified')
_RAY_AXES = tuple((math.cos(math.radians(-i)), math.sin(math.radians(-i))) for i in range(360))


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def validate_initial_presteer_pwm(value):
    if value is not None and (type(value) is not int or not INITIAL_PRESTEER_MIN <= value <= SERVO_MAX):
        raise ValueError('invalid_initial_presteer_pwm')
    return value


def _wrap(angle):
    return (angle+math.pi) % (2*math.pi)-math.pi


def _point(value):
    return (isinstance(value, dict) and set(value) == {'x_m', 'y_m'}
            and all(_number(value[k]) and abs(value[k]) <= 12 for k in ('x_m', 'y_m')))


def _freeze(value):
    if isinstance(value, dict):
        return tuple((k, _freeze(v)) for k, v in sorted(value.items()))
    if isinstance(value, list):
        return tuple(_freeze(v) for v in value)
    return value


def _valid_wall(wall):
    if not isinstance(wall, dict):
        return False
    return (wall.get('candidate_only') is True
            and all(_number(wall.get(k)) for k in ('heading_left_rad', 'rho_left_m', 'support_span_m', 'fit_error_m'))
            and -math.pi <= wall['heading_left_rad'] <= math.pi and abs(wall['rho_left_m']) <= 12
            and type(wall.get('points')) is int and wall['points'] >= 16
            and wall['support_span_m'] >= .35 and 0 <= wall['fit_error_m'] <= .06
            and _point(wall.get('support_start_left_m')) and _point(wall.get('support_end_left_m')))


def _valid_opening(goal, scan):
    if not isinstance(goal, dict) or not _valid_wall(goal.get('outer_wall')):
        return False
    ray = goal.get('target_support_ray')
    end_ray = goal.get('incoming_left_end_support')
    if (goal.get('candidate_only') is not True or goal.get('turn_path_certified') is not False
            or goal.get('observation_type') != 'left_opening_outer_wall_alignment'
            or type(goal.get('origin_between_exit_walls')) is not bool
            or not all(_number(goal.get(k)) for k in ('heading_left_rad', 'center_offset_left_m', 'width_m',
                                                     'incoming_heading_left_rad', 'front_wall_m', 'incoming_left_end_m'))
            or not -math.pi <= goal['heading_left_rad'] <= math.pi or not .6 <= goal['width_m'] <= 5
            or not -math.pi <= goal['incoming_heading_left_rad'] <= math.pi
            or not -.3 <= goal['incoming_left_end_m'] <= 1.
            or not _point(goal.get('target_point_left_m')) or not isinstance(ray, dict)
            or type(ray.get('index')) is not int or not 0 <= ray['index'] < 360
            or not all(_number(ray.get(k)) for k in ('angle_left_rad', 'range_m', 'target_range_m'))
            or not .02 <= ray['range_m'] <= 12 or not 0 < ray['target_range_m'] < ray['range_m']
            or not _number(goal.get('known_open_fraction')) or not .9 <= goal['known_open_fraction'] <= 1
            or not isinstance(end_ray, dict) or type(end_ray.get('index')) is not int
            or not 0 <= end_ray['index'] < 360
            or not all(_number(end_ray.get(k)) for k in ('angle_left_rad', 'range_m'))
            or not .02 <= end_ray['range_m'] <= 12 or not _point(end_ray.get('point_left_m'))):
        return False
    wall, point = goal['outer_wall'], goal['target_point_left_m']
    theta, distance = ray['angle_left_rad'], ray['target_range_m']
    ranges = scan.get('ranges')
    end, end_angle, end_distance = end_ray['point_left_m'], end_ray['angle_left_rad'], end_ray['range_m']
    incoming = goal['incoming_heading_left_rad']
    return (isinstance(ranges, list) and len(ranges) == 360
            and _number(ranges[ray['index']]) and abs(ranges[ray['index']]-ray['range_m']) <= 1e-6
            and abs(_wrap(theta+math.radians(ray['index']))) <= 1e-6
            and math.hypot(point['x_m']-distance*math.cos(theta), point['y_m']-distance*math.sin(theta)) <= .02
            and abs(_wrap(goal['heading_left_rad']-wall['heading_left_rad'])) <= math.radians(2)
            and abs(goal['center_offset_left_m']-(wall['rho_left_m']+goal['width_m']/2)) <= .05
            and abs(-math.sin(goal['heading_left_rad'])*point['x_m']
                    + math.cos(goal['heading_left_rad'])*point['y_m']-goal['center_offset_left_m']) <= .06
            and _number(ranges[end_ray['index']]) and abs(ranges[end_ray['index']]-end_distance) <= 1e-6
            and abs(_wrap(end_angle+math.radians(end_ray['index']))) <= 1e-6
            and math.hypot(end['x_m']-end_distance*math.cos(end_angle),
                           end['y_m']-end_distance*math.sin(end_angle)) <= 1e-6
            and abs(math.cos(incoming)*end['x_m']+math.sin(incoming)*end['y_m']
                    -goal['incoming_left_end_m']) <= 1e-6
            and -math.sin(incoming)*end['x_m']+math.cos(incoming)*end['y_m'] > 0)


def parse_trial_goal(record):
    """Accept manual/simulated TARGETS only; no source is physical feedback.

    The service binds receipt/expiry and the actual run-start lidar reference.
    Arbitrary reference-frame points need real relative pose before execution.
    """
    required = {'schema_version', 'goal_id', 'source_kind', 'goal_type', 'frame',
                'coordinate_convention', 'max_seconds'}
    if (not isinstance(record, dict) or not required <= set(record)
            or set(record)-required-{'target_point_left_m'}
            or type(record['schema_version']) is not int or record['schema_version'] != 1
            or not isinstance(record['goal_id'], str) or not 1 <= len(record['goal_id']) <= 64
            or any(not (c.isascii() and (c.isalnum() or c in '_-')) for c in record['goal_id'])
            or record['source_kind'] not in ('manual', 'simulated')
            or record['goal_type'] not in ('turn_exit_align', 'point_stop')
            or record['frame'] != 'run_start_lidar_reference'
            or record['coordinate_convention'] != 'x_forward_y_left_yaw_left_positive'
            or not _number(record['max_seconds']) or not 0 < record['max_seconds'] <= MAX_DRIVE_S
            or ('target_point_left_m' in record and not _point(record['target_point_left_m']))
            or (record['goal_type'] == 'point_stop' and 'target_point_left_m' not in record)):
        raise ValueError('invalid_target_only_trial_goal')
    result = dict(record)
    if 'target_point_left_m' in result:
        result['target_point_left_m'] = dict(result['target_point_left_m'])
    result.update(target_only=True, physical_control_ready=False, completed=False,
                  execution_rejection='real_pose_missing' if record['goal_type'] == 'point_stop' else
                                      'native_left_turn_geometry_required')
    return result


def _valid_candidate(value):
    if not isinstance(value, dict) or any(k not in value for k in _FIELDS):
        return False
    if not all(_number(value[k]) for k in _FIELDS[:3]+('support_span_m', 'fit_error_m')):
        return False
    return (value['candidate_only'] is True and value['turn_path_certified'] is False
            and value['origin_between_walls'] is True
            and -math.pi <= value['heading_left_rad'] <= math.pi
            and .6 <= value['width_m'] <= 5
            and value['support_span_m'] >= .35 and 0 <= value['fit_error_m'] <= .06
            and all(type(value[k]) is int and value[k] >= 16
                    for k in ('left_wall_points', 'right_wall_points'))
            and abs(value['center_offset_left_m']) < value['width_m']/2-TURN_LATERAL_MARGIN_M)


def _surface_supports(scan, walls):
    """Current actual returns supporting local smooth surfaces, never free space.

    Coordinates and continuity edges are computed once, not per wall pair.
    Missing returns break the chain; publication time is not a per-point clock.
    """
    ranges = scan.get('ranges') if isinstance(scan, dict) else None
    if (not isinstance(ranges, list) or len(ranges) != 360
            or any(r is not None and (not _number(r) or not .02 <= r <= 12) for r in ranges)):
        return [None]*len(walls)
    points = [None if r is None else (r*a[0], r*a[1]) for r, a in zip(ranges, _RAY_AXES)]
    basic = []
    sample_angle = math.radians(1)
    for i, p in enumerate(points):
        j, q = (i+1) % 360, points[(i+1) % 360]
        # Bound the observed step by actual angular sample spacing plus the
        # native 4 cm inlier allowance, with a 15 cm cap for sparse far returns.
        limit = 0 if p is None or q is None else min(.15, .04+3*min(ranges[i], ranges[j])*sample_angle)
        basic.append(p is not None and q is not None and math.hypot(q[0]-p[0], q[1]-p[1]) <= limit)

    # Six-return windows stay inside their actual basic-connected run. Near a
    # measured run edge they become one-sided, rather than erasing five real
    # returns because a centred window would cross unknown space. Interior
    # edges retain the original separated before/after windows and gates.
    basic_pivot = next(((i+1) % 360 for i, edge in enumerate(basic) if not edge), 0)
    basic_order = [(basic_pivot+i) % 360 for i in range(360)]
    ordered = [points[i] for i in basic_order]
    circular = all(basic)
    run_start, run_end, start = [None]*360, [None]*360, 0
    for position, i in enumerate(basic_order):
        if points[i] is None:
            continue
        if position == 0 or not basic[basic_order[position-1]]:
            start = position
        run_start[position] = start
        if position == 359 or not basic[i]:
            for member in range(start, position+1):
                run_end[member] = position+1

    # Prefix moments and cached windows are shared by every candidate wall.
    # The padding serves only a genuinely continuous 359/0 run, not a gap.
    extended = ordered[-5:]+ordered+ordered[:6]
    sx, sy, sxx, syy, sxy = ([0.] for _ in range(5))
    for p in extended:
        x, y = (0., 0.) if p is None else p
        sx.append(sx[-1]+x)
        sy.append(sy[-1]+y)
        sxx.append(sxx[-1]+x*x)
        syy.append(syy[-1]+y*y)
        sxy.append(sxy[-1]+x*y)
    tangents = {}
    def tangent(window):
        if window not in tangents:
            i = window+5
            x, y = sx[i+6]-sx[i], sy[i+6]-sy[i]
            xx = sxx[i+6]-sxx[i]-x*x/6
            yy = syy[i+6]-syy[i]-y*y/6
            xy = sxy[i+6]-sxy[i]-x*y/6
            angle = .5*math.atan2(2*xy, xx-yy)
            direction = math.cos(angle), math.sin(angle)
            first, last = extended[i], extended[i+5]
            dx, dy = last[0]-first[0], last[1]-first[1]
            tangents[window] = (direction if direction[0]*dx+direction[1]*dy >= 0
                                else (-direction[0], -direction[1]))
        return tangents[window]

    strong = [False]*360
    for position, i in enumerate(basic_order):
        if not basic[i]:
            continue
        low, high = run_start[position], run_end[position]
        if high-low < 6:
            continue
        before_start, after_start = position-5, position+1
        if not circular:
            before_start = max(low, min(before_start, high-6))
            after_start = max(low, min(after_start, high-6))
        before, after = tangent(before_start), tangent(after_start)
        cross = before[0]*after[1]-before[1]*after[0]
        dot = before[0]*after[0]+before[1]*after[1]
        p, q = points[i], points[(i+1) % 360]
        dx, dy = q[0]-p[0], q[1]-p[1]
        strong[i] = (abs(math.atan2(cross, dot)) <= math.radians(20)
                     and abs(before[0]*dy-before[1]*dx) <= .04
                     and abs(after[0]*dy-after[1]*dx) <= .04)
    # Start after a real break so a surface crossing ray 359/0 remains one run.
    pivot = next(((i+1) % 360 for i, edge in enumerate(strong) if not edge), 0)
    order = [(pivot+i) % 360 for i in range(360)]
    labels, label = [], 0
    for position, i in enumerate(order):
        if position and not strong[order[position-1]]:
            label += 1
        labels.append(label)
    supports = []
    ordered_points = [(position, points[i], labels[position], 1 << position)
                      for position, i in enumerate(order) if points[i] is not None]
    projections = {}
    for wall in walls:
        theta = wall['heading_left_rad']
        c, s = math.cos(theta), math.sin(theta)
        low, high = sorted(c*wall[k]['x_m']+s*wall[k]['y_m']
                           for k in ('support_start_left_m', 'support_end_left_m'))
        groups, count, line_bits = {}, 0, 0
        if theta not in projections:
            projections[theta] = [(position, -s*p[0]+c*p[1], c*p[0]+s*p[1], key, bit)
                                  for position, p, key, bit in ordered_points]
        rho, error = wall['rho_left_m'], wall['fit_error_m']+1e-6
        low, high = low-1e-6, high+1e-6
        for position, normal, along, key, bit in projections[theta]:
            if abs(normal-rho) > error:
                continue
            line_bits |= bit
            if not low <= along <= high:
                continue
            count += 1
            mask, start, _ = groups.get(key, (0, position, position))
            groups[key] = mask | bit, start, position
        mature = {key: value for key, value in groups.items() if value[0].bit_count() >= 16}
        supports.append((mature, line_bits) if count >= 16 else None)
    return supports


def _same_surface(a, b):
    if a is None or b is None:
        return None
    a, left_line = a
    b, right_line = b
    shared = left_line & right_line
    # Actual disjoint near-zero-error supports are separate even if their
    # fitted headings are identical and separation is below the old 5 cm gate.
    if not shared:
        return False
    mature = False
    for key in a.keys() & b.keys():
        left, lo, hi = a[key]
        right, start, end = b[key]
        if left.bit_count() < 16 or right.bit_count() < 16:
            continue
        mature = True
        low, high = min(lo, start), max(hi, end)
        interval = ((1 << (high-low+1))-1) << low
        # Every actual connecting return must fit one of these lines within
        # its reported residual, including a visible bridge between windows.
        # The interval is bounded by their current finite supports; a distant
        # rounded corner cannot join independent parallel boards.
        if (left_line | right_line) & interval == interval and (shared & interval).bit_count() >= 4:
            return True
    # Fragmented or short raw support cannot relax the original alias rule.
    return False if mature else None


class TurnMotion:
    """Single-use state machine. The service retains safety/heartbeat/ACK ownership.

    ``control`` is fresh bridge feedback fenced by the service to commands in
    this session. Its armed/motor/servo values are software adoption evidence,
    never measured wheel position. Before arm, update is a neutral preview.
    """
    def __init__(self, started_at, max_drive_s=MAX_DRIVE_S, motor_pwm=TRIAL_MOTOR,
                 max_presteer_s=MAX_PRESTEER_S, initial_presteer_pwm=None):
        if (not _number(started_at) or started_at < 0
                or not _number(max_drive_s) or not 0 < max_drive_s <= MAX_DRIVE_S
                or not _number(max_presteer_s) or not 0 < max_presteer_s <= MAX_PRESTEER_S
                or type(motor_pwm) is not int or motor_pwm != TRIAL_MOTOR):
            raise ValueError('invalid_bounded_left_turn_trial')
        self.started_at = started_at
        self.initial_presteer_pwm = validate_initial_presteer_pwm(initial_presteer_pwm)
        self.natural_steering_target = None
        self.max_drive_s, self.max_presteer_s = max_drive_s, max_presteer_s
        self.phase = 'presteer'
        self.servo = self.steering_target = NEUTRAL
        self.last_change = started_at-PWM_INTERVAL_S
        self.last_now = started_at
        self.last_seq = self.last_publication = self.last_receive = None
        self.last_signature = None
        self.corridor = None
        self.geometry_mode = None
        self.outer_wall = self.turn_goal = None
        self.last_geometry_receive = self.last_geometry_publication = self.last_geometry_seq = None
        self.geometry_missing = False
        self.selected = False
        self.settle_since = None
        self.settle_feedback_ticks = 0
        self.last_control_tick = self.last_control_seq = None
        self.drive_since = None
        self.presteer_ended_at = None
        self.last_error = None
        self.alignment_count = 0
        self.alignment_publication = self.alignment_receive = None
        self.observed_alignment = self.entry_confirmed = False
        self.alignment_evidence = None
        self.reason = 'awaiting_left_corridor'
        self.start_ready = False
        self.presteer_waiting = False
        self.incoming_endpoint = None
        self.exit_width_change = None

    def _endpoint_observation(self, scan):
        # Diagnostics only: cached turn_goal fields are not current ray evidence.
        opening = scan.get('left_turn_goal')
        self.incoming_endpoint = (None if opening is None else {
            'source_seq': scan['seq'], 'source_at_ms': scan['at_ms'],
            'incoming_left_end_m': opening['incoming_left_end_m'],
            'incoming_left_end_support': copy.deepcopy(opening['incoming_left_end_support']),
            'known_open_fraction': opening['known_open_fraction'],
            'candidate_only': True, 'turn_path_certified': False})

    def _presteer_hold(self, reason):
        self.reason, self.start_ready = reason, False
        self.presteer_waiting = True
        self.settle_since = None
        self.settle_feedback_ticks = 0
        self.last_error = None
        self.natural_steering_target = None

    def _presteer_target(self, target):
        return self.initial_presteer_pwm if self.initial_presteer_pwm is not None else target

    def lock(self, reason):
        if self.phase == 'presteer':
            self.presteer_ended_at = self.last_now
        self.phase, self.reason = 'locked', reason
        self.presteer_waiting = False
        self.incoming_endpoint = None
        self.natural_steering_target = None
        self.servo = self.steering_target = NEUTRAL
        self.start_ready = False
        self.settle_since = None
        self.settle_feedback_ticks = 0

    def begin_coast(self, reason, now):
        if not _number(now) or now < self.last_now:
            self.lock('invalid_turn_receive_clock')
        elif self.phase != 'locked':
            if self.phase == 'presteer':
                self.presteer_ended_at = now
            self.phase, self.reason = 'coast', reason
            self.steering_target = NEUTRAL
            self.settle_since = None
            self.last_error = None
            self.natural_steering_target = None

    def _slew(self, now):
        if now-self.last_change+1e-9 >= PWM_INTERVAL_S and self.servo != self.steering_target:
            delta = max(-PWM_STEP, min(PWM_STEP, self.steering_target-self.servo))
            self.servo += delta
            self.last_change = now  # No accumulated catch-up steps after a delayed tick.
            self.settle_since = None

    def _result(self, now):
        coast = self.phase == 'coast'
        return {'phase': self.phase, 'motor': TRIAL_MOTOR if self.phase == 'drive' else NEUTRAL,
                'servo': self.servo, 'steering_target': self.steering_target,
                'initial_presteer_pwm': self.initial_presteer_pwm,
                'initial_presteer_active': (self.phase == 'presteer' and self.initial_presteer_pwm is not None
                                            and self.start_ready and self.natural_steering_target is not None
                                            and self.natural_steering_target > NEUTRAL),
                'natural_steering_target': self.natural_steering_target,
                'start_ready': self.start_ready and self.phase == 'presteer',
                'request_coast': coast, 'stop_requested': coast,
                'lock_requested': self.phase == 'locked',
                'terminal_reason': self.reason if self.phase in ('coast', 'locked') else None,
                'reason': self.reason, 'observed_alignment': self.observed_alignment,
                'test_sequence_finished': self.observed_alignment,
                'entry_confirmed': self.entry_confirmed, 'completed': False,
                'competition_navigation': False, 'candidate_only': True,
                'turn_path_certified': False, 'physical_steering_confirmed': False,
                'steering_allowance_s': STEERING_ALLOWANCE_S,
                'presteer_elapsed_s': max(0, (now if self.presteer_ended_at is None else
                                              self.presteer_ended_at)-self.started_at),
                'steering_settle_elapsed_s': 0 if self.settle_since is None else max(0, now-self.settle_since),
                'steering_settle_feedback_ticks': self.settle_feedback_ticks,
                'drive_elapsed_s': 0 if self.drive_since is None else max(0, now-self.drive_since),
                'alignment_confirmations': self.alignment_count,
                'alignment_evidence': self.alignment_evidence,
                'scan_seq': self.last_seq, 'publication_at_ms': self.last_publication,
                'geometry_mode': self.geometry_mode, 'geometry_missing': self.geometry_missing,
                'geometry_missing_s': 0 if not self.geometry_missing or self.last_geometry_receive is None else
                                      max(0, now-self.last_geometry_receive),
                'geometry_source_seq': self.last_geometry_seq,
                'geometry_source_at_ms': self.last_geometry_publication,
                'turn_stage': 'presteer_wait' if self.presteer_waiting and self.phase == 'presteer' else self.phase,
                'incoming_endpoint_current': self.incoming_endpoint is not None,
                'incoming_endpoint': copy.deepcopy(self.incoming_endpoint),
                'exit_width_change': copy.deepcopy(self.exit_width_change),
                'turn_goal': copy.deepcopy(self.turn_goal),
                'outer_wall': copy.deepcopy(self.outer_wall),
                'corridor': dict(self.corridor) if self.corridor else None}

    def _opening_candidate(self, wall, width):
        return {'heading_left_rad': wall['heading_left_rad'], 'center_offset_left_m': wall['rho_left_m']+width/2,
                'width_m': width, 'candidate_only': True, 'turn_path_certified': False,
                'origin_between_walls': False}

    def _wall_matches(self, walls, publication_dt, scan=None):
        old = self.outer_wall
        gate = min(math.radians(30), .12+math.radians(90)*publication_dt)
        def interval(w, theta):
            return sorted(math.cos(theta)*w[k]['x_m']+math.sin(theta)*w[k]['y_m']
                          for k in ('support_start_left_m', 'support_end_left_m'))
        def overlaps_previous(w):
            # Similar infinite lines can describe unrelated finite segments.
            # Associate only support that still overlaps the selected wall.
            theta = w['heading_left_rad']
            a, b = interval(old, theta), interval(w, theta)
            return min(a[1], b[1])-max(a[0], b[0]) >= .10
        matches = [w for w in walls if abs(_wrap(w['heading_left_rad']-old['heading_left_rad'])) <= gate
                   and abs(w['rho_left_m']-old['rho_left_m']) <= .30 and overlaps_previous(w)]
        # The opening goal commonly repeats an identical native wall candidate.
        # Remove exact copies before any raw-surface work, preserving order.
        distinct = []
        for wall in matches:
            if wall not in distinct:
                distinct.append(wall)
        matches = distinct
        if len(matches) <= 1:
            return [dict(wall) for wall in matches]
        supports = _surface_supports(scan, matches)
        # Overlapping fits of the same actual finite wall are one observation,
        # not two different destinations. Distinct parallel supports stay ambiguous.
        unique = []
        for index in sorted(range(len(matches)), key=lambda i: matches[i]['support_span_m'], reverse=True):
            wall, support = matches[index], supports[index]
            theta = wall['heading_left_rad']
            a = interval(wall, theta)
            def same(u, evidence):
                if wall == u:
                    return True
                observed = _same_surface(support, evidence)
                if observed is not None:
                    return observed
                # Incomplete raw evidence cannot relax the original geometric
                # alias rule. It does not manufacture a surface witness.
                return (abs(_wrap(wall['heading_left_rad']-u['heading_left_rad'])) <= math.radians(2)
                        and abs(wall['rho_left_m']-u['rho_left_m']) <= .05
                        and min(a[1], interval(u, theta)[1])-max(a[0], interval(u, theta)[0]) >= .10)
            # A broad fit cannot transitively join two distinct surfaces: a
            # new alias must agree with every actual member of its group.
            group = next((group for group in unique if all(same(u, evidence) for u, evidence in group)), None)
            if group is None:
                unique.append([(wall, support)])
            else:
                group.append((wall, support))
        return [dict(group[0][0]) for group in unique]

    def _exit_width_change_limit(self, previous_width):
        return .15

    def _continue_with_tracked_width(self):
        return False

    def _measurement(self, scan, candidates, publication_dt):
        opening, walls = scan.get('left_turn_goal'), scan.get('wall_candidates', [])
        if (not isinstance(walls, list) or len(walls) > 64 or any(not _valid_wall(w) for w in walls)
                or (opening is not None and not _valid_opening(opening, scan))):
            raise ValueError('invalid_native_turn_wall_goal')
        if not self.selected and opening is not None:
            if (not math.radians(15) < opening['heading_left_rad'] < math.radians(150)
                    or opening['target_point_left_m']['x_m'] <= 0 or opening['target_point_left_m']['y_m'] <= 0):
                raise ValueError('left_turn_goal_not_a_forward_left_exit')
            self.outer_wall, self.turn_goal, self.geometry_mode = dict(opening['outer_wall']), dict(opening), 'opening_wall'
            return self._opening_candidate(self.outer_wall, opening['width_m'])
        if self.geometry_mode == 'opening_wall':
            matches = self._wall_matches(walls+([opening['outer_wall']] if opening else []), publication_dt, scan)
            if len(matches) > 1:
                raise ValueError('left_turn_outer_wall_ambiguous')
            if not matches:
                if walls or opening:
                    raise ValueError('left_turn_outer_wall_identity_jump')
                raise ValueError('left_turn_geometry_missing')
            wall = matches[0]
            width = self.corridor['width_m']
            update_from_opening = opening is not None
            if opening:
                delta = abs(opening['width_m']-width)
                limit = self._exit_width_change_limit(width)
                accepted = delta <= limit or math.isclose(delta, limit, rel_tol=0, abs_tol=1e-12)
                self.exit_width_change = {'source_seq': scan['seq'],
                    'source_at_ms': scan['at_ms'], 'previous_width_m': width,
                    'current_width_m': opening['width_m'], 'absolute_change_m': delta,
                    'relative_change': delta/width, 'allowed_change_m': limit,
                    'accepted': accepted, 'wall_identity_checked': True,
                    'action': 'adopt_current_width' if accepted else 'reject_width_jump'}
                if not accepted:
                    if not self._continue_with_tracked_width():
                        raise ValueError('left_turn_exit_width_jump')
                    update_from_opening = False
                    self.exit_width_change['action'] = 'track_current_wall_keep_previous_width'
                else:
                    width = opening['width_m']
                    self.turn_goal = dict(opening)
            if not update_from_opening:
                # Live finite support defines a new reference point on the same
                # measured wall's offset centerline; no global pose is invented.
                theta = wall['heading_left_rad']
                a, b = wall['support_start_left_m'], wall['support_end_left_m']
                point = {'x_m': (a['x_m']+b['x_m'])/2-math.sin(theta)*width/2,
                         'y_m': (a['y_m']+b['y_m'])/2+math.cos(theta)*width/2}
                self.turn_goal = {**self.turn_goal, 'heading_left_rad': theta,
                                  'center_offset_left_m': wall['rho_left_m']+width/2,
                                  'target_point_left_m': point, 'outer_wall': dict(wall),
                                  'target_support_ray': None, 'observation_type': 'tracked_actual_outer_wall'}
            self.outer_wall = wall
            candidate = self._opening_candidate(wall, width)
            handover = [c for c in candidates
                        if abs(_wrap(c['heading_left_rad']-wall['heading_left_rad'])) <= math.radians(8)
                        and abs(c['center_offset_left_m']-candidate['center_offset_left_m']) <= .12
                        and abs(c['width_m']-width) <= .15]
            if len(handover) > 1:
                raise ValueError('left_turn_corridor_handover_ambiguous')
            if handover:
                self.geometry_mode = 'corridor'
                return dict(handover[0])
            return candidate
        if not candidates:
            if self.selected:
                raise ValueError('left_turn_geometry_missing')
        result = self._candidate(candidates, publication_dt)
        self.geometry_mode = 'corridor'
        return result

    def _candidate(self, candidates, publication_dt):
        if not self.selected:
            matches = [c for c in candidates if math.radians(15) < c['heading_left_rad'] < math.radians(150)]
        else:
            old = self.corridor
            # Continuity bounds are candidate association gates, not odometry.
            heading_gate = min(math.radians(30), .12+math.radians(90)*publication_dt)
            matches = [c for c in candidates
                       if abs(_wrap(c['heading_left_rad']-old['heading_left_rad'])) <= heading_gate
                       and abs(c['center_offset_left_m']-old['center_offset_left_m']) <= .25
                       and abs(c['width_m']-old['width_m']) <= .15]
        if len(matches) != 1:
            raise ValueError('left_corridor_ambiguous' if len(matches) > 1 else
                             'left_corridor_lost_or_jumped' if self.selected else 'left_corridor_unknown')
        return dict(matches[0])

    def _target(self, candidate, publication_dt):
        heading, offset = candidate['heading_left_rad'], candidate['center_offset_left_m']
        lookahead = max(.35, min(1., candidate['width_m']/2))
        if self.geometry_mode == 'opening_wall':
            point = self.turn_goal['target_point_left_m']
            # A finite wall support can move behind the lidar. Its midpoint is
            # then no forward steering target; atan2 would spuriously add pi.
            error = (.65*heading+.35*math.atan2(point['y_m'], point['x_m']) if point['x_m'] > 0
                     else heading+math.atan2(offset, lookahead))
        else:
            error = heading+math.atan2(offset, lookahead)
        damped = error
        if self.last_error is not None and .05 <= publication_dt < SCAN_AGE_S:
            rate = (error-self.last_error)/publication_dt
            if abs(rate) <= math.radians(90):
                release = error+.35*rate
                # Trend can release the old left correction; cannot create its opposite or enlarge it.
                if release*error <= 0:
                    damped = 0
                elif abs(release) < abs(error):
                    damped = release
        self.last_error = error
        # Trial feedback gain, not a curvature or speed-to-PWM calibration.
        return max(NEUTRAL, min(SERVO_MAX, NEUTRAL+round(140*damped)))

    def _coast_target(self, candidate):
        # Only an already associated, near-forward measured wall/corridor can
        # correct residual coast. Use its line, never a support midpoint behind
        # the car. This remains candidate geometry, not a certified path.
        available = candidate['width_m']/2-TURN_LATERAL_MARGIN_M
        if (abs(candidate['heading_left_rad']) > math.radians(15) or available <= 0
                or abs(candidate['center_offset_left_m']) >= available):
            return NEUTRAL
        lookahead = max(.35, min(1., candidate['width_m']/2))
        error = candidate['heading_left_rad']+math.atan2(candidate['center_offset_left_m'], lookahead)
        return max(NEUTRAL-COAST_SERVO_DELTA,
                   min(NEUTRAL+COAST_SERVO_DELTA, NEUTRAL+round(140*error)))

    def _entry(self, evidence, scan):
        return (isinstance(evidence, dict) and evidence.get('kind') in ('cone', 'obstacle')
                and evidence.get('candidate_only') is True
                and type(evidence.get('source_seq')) is int and evidence['source_seq'] == scan['seq']
                and type(evidence.get('source_at_ms')) is int and evidence['source_at_ms'] == scan['at_ms']
                and _number(evidence.get('distance_forward_m')) and evidence['distance_forward_m'] > 0
                and _number(evidence.get('error_m')) and 0 <= evidence['error_m'] < evidence['distance_forward_m'])

    def update(self, scan, lidar_age_s, now, control, *, safe=True, entry_stop=None, presteer_wait=False):
        if not _number(now) or now < self.last_now:
            self.lock('invalid_turn_receive_clock')
            return self._result(self.last_now)
        self.last_now = now
        if self.phase == 'locked':
            return self._result(now)
        if type(presteer_wait) is not bool:
            self.lock('invalid_turn_presteer_wait')
            return self._result(now)
        self.presteer_waiting = False
        if safe is not True:
            self.lock('turn_safety_rejected')
            return self._result(now)
        if self.phase == 'drive' and now-self.drive_since >= self.max_drive_s:
            self.begin_coast('left_turn_drive_timeout', now)
        if self.phase == 'presteer' and now-self.started_at >= self.max_presteer_s:
            self.lock('left_turn_presteer_timeout')
            return self._result(now)
        if (not _number(lidar_age_s) or not 0 <= lidar_age_s < SCAN_AGE_S
                or not isinstance(scan, dict) or scan.get('frame_id') != _FRAME
                or type(scan.get('seq')) is not int or scan['seq'] < 0
                or type(scan.get('at_ms')) is not int or scan['at_ms'] < 0):
            self.lock('turn_scan_stale_or_invalid')
            return self._result(now)
        candidates = scan.get('corridor_candidates')
        if (not isinstance(candidates, list) or len(candidates) > 24
                or any(not _valid_candidate(c) for c in candidates)):
            self.lock('invalid_native_corridor_candidates')
            return self._result(now)
        signature = (_freeze(candidates), _freeze(scan.get('wall_candidates', [])),
                     _freeze(scan.get('left_turn_goal')), _freeze(scan.get('ranges')))
        if (not isinstance(control, dict) or type(control.get('armed')) is not bool
                or type(control.get('motor')) is not int or type(control.get('servo')) is not int
                or type(control.get('tick')) is not int or control['tick'] < 0
                or type(control.get('seq')) is not int or control['seq'] < 0
                or not SERVO_MIN <= control['servo'] <= SERVO_MAX):
            self.lock('invalid_turn_adoption_feedback')
            return self._result(now)
        if self.last_control_tick is not None and (control['tick'] < self.last_control_tick
                or control['seq'] < self.last_control_seq):
            self.lock('reordered_turn_adoption_feedback')
            return self._result(now)
        new_feedback = self.last_control_tick is None or control['tick'] > self.last_control_tick
        self.last_control_tick, self.last_control_seq = control['tick'], control['seq']
        if presteer_wait and (self.phase != 'presteer' or control['motor'] != NEUTRAL):
            self.lock('turn_presteer_wait_requires_neutral')
            return self._result(now)
        if self.phase in ('drive', 'coast') and not control['armed']:
            self.lock('turn_bridge_locked')
            return self._result(now)
        seq, published = scan['seq'], scan['at_ms']
        if self.last_seq is not None and seq == self.last_seq:
            if published != self.last_publication or signature != self.last_signature:
                self.lock('duplicate_turn_scan_changed_content')
            elif now-self.last_receive >= SCAN_AGE_S:
                self.lock('turn_scan_not_advancing')
            elif self.phase == 'coast':
                self._slew(now)
            elif presteer_wait and self.phase != 'locked':
                self._presteer_hold('left_turn_presteer_quality_hold')
            # Repeated frames cannot presteer or mature alignment evidence;
            # coast may release an already selected target within the fresh lease.
            return self._result(now)
        publication_dt = 0 if self.last_publication is None else (published-self.last_publication)/1000
        if self.last_seq is not None and (seq < self.last_seq or publication_dt <= 0):
            self.lock('reordered_turn_scan')
            return self._result(now)
        if self.last_receive is not None and (publication_dt >= SCAN_AGE_S
                or now-self.last_receive >= SCAN_AGE_S
                or abs(publication_dt-(now-self.last_receive)) > .15):
            self.lock('turn_scan_clock_gap')
            return self._result(now)
        self.last_seq, self.last_publication, self.last_receive = seq, published, now
        self.last_signature = signature
        self.incoming_endpoint = None
        self.natural_steering_target = None
        geometry_dt = 0 if self.last_geometry_publication is None else (published-self.last_geometry_publication)/1000
        try:
            candidate = self._measurement(scan, candidates, geometry_dt)
        except ValueError as error:
            missing = str(error) == 'left_turn_geometry_missing'
            if missing and self.phase == 'coast':
                self.geometry_missing = True
                self.start_ready = False
                self.steering_target = NEUTRAL
                self.last_error = None
                self._slew(now)
                return self._result(now)
            if (missing and self.last_geometry_receive is not None
                    and now-self.last_geometry_receive <= SCAN_AGE_S+1e-9
                    and geometry_dt <= SCAN_AGE_S+1e-9):
                self.geometry_missing = True
                self.start_ready = False
                self.settle_since = None
                self.settle_feedback_ticks = self.alignment_count = 0
                self.alignment_evidence = None
                self.alignment_publication = self.alignment_receive = None
                self.last_error = None
                self.reason = 'left_turn_geometry_gap_hold'
                if presteer_wait:
                    self._presteer_hold(self.reason)
                elif control['armed']:
                    self._slew(now)
                return self._result(now)
            self.lock('left_turn_geometry_lost' if missing else str(error))
            return self._result(now)
        self.last_geometry_receive, self.last_geometry_publication, self.last_geometry_seq = now, published, seq
        self.geometry_missing = False
        self.corridor, self.selected, self.start_ready = candidate, True, True
        self._endpoint_observation(scan)
        if self.phase == 'coast':
            self.steering_target = self._coast_target(candidate)
            self._slew(now)
            return self._result(now)
        if presteer_wait:
            self._presteer_hold('left_turn_presteer_quality_hold')
            return self._result(now)
        if entry_stop is not None:
            if not self._entry(entry_stop, scan):
                self.lock('invalid_turn_entry_evidence')
            else:
                self.entry_confirmed = True
                self.begin_coast('left_turn_entry_candidate_stop', now)
            return self._result(now)
        target = self._target(candidate, publication_dt)
        self.natural_steering_target = target
        if self.phase == 'presteer':
            # A left-turn trial must first adopt a left command. A currently
            # neutral target is a hold, never permission to start with centered steering.
            if target <= NEUTRAL:
                self.steering_target = NEUTRAL
                self.start_ready = False
                self.settle_since = None
                self.settle_feedback_ticks = 0
                self.reason = 'left_turn_presteer_left_target_required'
                if control['armed']:
                    self._slew(now)
                return self._result(now)
            target = self._presteer_target(target)
            # Small measurement noise does not endlessly restart an unchanged presteer command.
            if self.steering_target == NEUTRAL or abs(target-self.steering_target) > 10:
                self.steering_target = target
                self.settle_since = None
                self.settle_feedback_ticks = 0
            if not control['armed']:
                self.reason = 'left_turn_neutral_preview'
                return self._result(now)
            self._slew(now)
            adopted = (self.servo == self.steering_target and control['motor'] == NEUTRAL
                       and control['servo'] == self.steering_target
                       and control.get('command_acked', True) is True)
            if not adopted:
                self.settle_since = None
                self.settle_feedback_ticks = 0
                self.reason = 'left_turn_presteering'
            else:
                if self.settle_since is None and new_feedback:
                    self.settle_since = now
                if new_feedback:
                    self.settle_feedback_ticks += 1
                self.reason = 'left_turn_steering_allowance'
                if (new_feedback and self.settle_since is not None and self.settle_feedback_ticks >= 3
                        and now-self.settle_since+1e-9 >= STEERING_ALLOWANCE_S):
                    self.phase, self.drive_since = 'drive', now
                    self.reason = 'left_turn_tracking_candidate'
                    self.presteer_ended_at = now
            return self._result(now)
        self.steering_target = target
        self._slew(now)
        heading, offset = candidate['heading_left_rad'], candidate['center_offset_left_m']
        available = candidate['width_m']/2-TURN_LATERAL_MARGIN_M
        aligned = available > 0 and abs(heading) <= math.radians(5) and abs(offset) < available
        if aligned:
            self.alignment_evidence = 'double_wall' if self.geometry_mode == 'corridor' else 'tracked_goal_wall'
            self.alignment_count += 1
            if self.alignment_publication is None:
                self.alignment_publication, self.alignment_receive = published, now
            if (self.alignment_count >= 3 and (published-self.alignment_publication)/1000 >= .25
                    and now-self.alignment_receive >= .25):
                self.observed_alignment = True
                self.begin_coast('left_turn_observed_corridor_alignment', now)
        else:
            self.alignment_count = 0
            self.alignment_evidence = None
            self.alignment_publication = self.alignment_receive = None
            if heading < -math.radians(8):
                self.begin_coast('left_turn_heading_overshoot', now)
        return self._result(now)
