"""Bounded left turn followed by one real relative-object orbit entry trial.

The compact object's semantic identity is unknown. This module observes no
global pose, speed, physical steering angle, passed cone count or complete lap.
The service retains fresh body-clearance checks, ownership, heartbeat and ACKs.
"""
import copy
import math

from compact_target import CompactTargetTracker
from autonomy_live import (REAR_BODY_EXTENT_M, SIDE_BODY_EXTENT_M,
                           MANEUVER_BODY_CLEARANCE_M)
from turn_motion import (TurnMotion, NEUTRAL, TRIAL_MOTOR, SERVO_MIN, SERVO_MAX, SCAN_AGE_S,
                         PWM_STEP, PWM_INTERVAL_S, _FRAME, _freeze, _number)

ORBIT_ENTRY_MAX_S = 3.0
COAST_MAX_S = 5.0
PRESTEER_MAX_S = 8.0
DEFAULT_INITIAL_PWM = 1670
INITIAL_PWM_MIN = 1670
PRESTEER_PWM_STEP = 20
LEFT_RELEASE_PWM_STEP = 20
ORBIT_BEARING_RAD = math.pi/2
ORBIT_BEARING_GAIN = 100.0
ORBIT_RANGE_GAIN = 70.0
LEFT_TRIAL_GAIN = 180.0
MAX_TREND_RELEASE_FRACTION = .20
MAX_INNER_RELEASE_FRACTION = .20


def validate_maneuver_initial_pwm(value):
    if type(value) is not int or not INITIAL_PWM_MIN <= value <= SERVO_MAX:
        raise ValueError('invalid_maneuver_initial_presteer_pwm')
    return value


class ManeuverSequence(TurnMotion):
    """Single arm and cumulative drive budget; no automatic restart after stop."""
    def __init__(self, started_at, max_drive_s=10.0, initial_presteer_pwm=DEFAULT_INITIAL_PWM):
        validate_maneuver_initial_pwm(initial_presteer_pwm)
        super().__init__(started_at, max_drive_s=max_drive_s,
                         initial_presteer_pwm=initial_presteer_pwm)
        # User-selected neutral preparation budget for this experiment only.
        # Parent construction and default TurnMotion retain their five seconds.
        self.max_presteer_s = PRESTEER_MAX_S
        self.target_tracker = CompactTargetTracker()
        self.compact_target = None
        self.handover_observed = False
        self.orbit_since = self.orbit_reference_range_m = None
        self.orbit_bias_pwm = None
        self.handover_control = None
        self.handover_wait_since = None
        self.orbit_track_id = None
        self.coast_since = None
        self.wall_ambiguity_hold = False
        self._last_actual_left_target = None
        self._presteer_adoption_context = None
        self._current_scan = None
        self.inner_clearance = None

    def begin_coast(self, reason, now):
        if self.phase not in ('coast', 'locked') and _number(now) and now >= self.last_now:
            self.coast_since = now
        super().begin_coast(reason, now)

    def _slew(self, now):
        if now-self.last_change+1e-9 >= PWM_INTERVAL_S and self.servo != self.steering_target:
            target = self.steering_target
            reducing_left = self.servo > NEUTRAL and target < self.servo
            step = PRESTEER_PWM_STEP if self.phase == 'presteer' else (
                LEFT_RELEASE_PWM_STEP if reducing_left else PWM_STEP)
            if reducing_left and target < NEUTRAL:
                target = NEUTRAL  # First release to center; a later right step stays at 10.
            delta = max(-step, min(step, target-self.servo))
            self.servo += delta
            self.last_change = now  # One step only; delayed ticks never catch up.
            self.settle_since = None

    def lock(self, reason):
        # Only this experimental sequence may hold its last actually commanded
        # servo briefly while wall aliases resolve. Neither geometry clock nor
        # alignment evidence advances; a compact observation is still required
        # for handover. Default TurnMotion is unchanged.
        adoption = self._presteer_adoption_context
        if (reason == 'left_turn_outer_wall_ambiguous' and self.phase == 'presteer'
                and self.selected and self.last_geometry_receive is not None
                and self.last_geometry_publication is not None
                and self._last_actual_left_target is not None
                and self._last_actual_left_target > NEUTRAL
                and adoption is not None and adoption['fresh_feedback']
                and adoption['control'].get('armed') is True
                and adoption['control'].get('motor') == NEUTRAL
                and adoption['control'].get('servo') == self.servo
                and adoption['control'].get('command_acked', True) is True):
            # Neutral presteer may wait within its original preparation budget.
            # Hold only an already adopted servo, not the unseen wall target.
            # All scan/control validation has run before this measurement error.
            # Fresh unique geometry must be observed again before either slew
            # or the full adoption/response allowance can restart.
            self.wall_ambiguity_hold = self.geometry_missing = True
            self.steering_target = self.servo
            self._presteer_hold('left_turn_presteer_wall_ambiguity_hold')
            self.alignment_count = 0
            self.alignment_evidence = None
            self.alignment_publication = self.alignment_receive = None
            return
        if (reason == 'left_turn_outer_wall_ambiguous' and self.phase == 'drive'
                and self.last_geometry_receive is not None
                and self.last_geometry_publication is not None
                and 0 <= self.last_now-self.last_geometry_receive <= SCAN_AGE_S+1e-9
                and 0 <= (self.last_publication-self.last_geometry_publication)/1000 <= SCAN_AGE_S+1e-9):
            self.wall_ambiguity_hold = self.geometry_missing = True
            self.start_ready = False
            self.reason = 'left_turn_wall_ambiguity_hold'
            self.steering_target = self.servo
            self.alignment_count = 0
            self.alignment_evidence = None
            self.alignment_publication = self.alignment_receive = None
            self.last_error = None
            return
        super().lock(reason)

    def _result(self, now):
        result = super()._result(now)
        stage = ('orbit_entry' if self.handover_observed and self.phase == 'drive'
                 else 'left_turn' if self.phase == 'drive' else result['turn_stage'])
        result.update(turn_stage=stage, trial_scope='first_lidar_compact_target_orbit_entry',
                      compact_target=copy.deepcopy(self.compact_target),
                      compact_target_reason=getattr(self.target_tracker, 'reason', None),
                      handover_observed=self.handover_observed,
                      orbit_reference_range_m=self.orbit_reference_range_m,
                      orbit_bias_pwm=self.orbit_bias_pwm,
                      handover_control=copy.deepcopy(self.handover_control),
                      orbit_entry_elapsed_s=0 if self.orbit_since is None else max(0, now-self.orbit_since),
                      orbit_entry_max_s=ORBIT_ENTRY_MAX_S,
                      presteer_max_s=self.max_presteer_s,
                      left_turn_servo_cap=SERVO_MAX,
                      steering_step_policy={'presteer': PRESTEER_PWM_STEP,
                          'reduce_left': LEFT_RELEASE_PWM_STEP, 'increase_left': PWM_STEP,
                          'center_to_right': PWM_STEP, 'interval_s': PWM_INTERVAL_S},
                      object_semantic_verified=False, passed_cones=None,
                      competition_supported=False, completed=False,
                      test_sequence_finished=False,
                      inner_clearance=copy.deepcopy(self.inner_clearance),
                      wall_ambiguity_hold=self.wall_ambiguity_hold)
        return result

    def _usable_target(self, target, scan):
        if not isinstance(target, dict) or target.get('confirmed') is not True:
            return False
        point = target.get('point_left_m')
        return (target.get('kind') == 'lidar_compact_object'
                and target.get('semantic_class') == 'unknown'
                and target.get('candidate_only') is True
                and type(target.get('source_seq')) is int and target['source_seq'] == scan.get('seq')
                and type(target.get('source_at_ms')) is int and target['source_at_ms'] == scan.get('at_ms')
                and isinstance(point, list) and len(point) == 2 and all(_number(v) for v in point)
                and _number(target.get('range_m')) and .35 <= target['range_m'] <= 3.0
                and _number(target.get('bearing_left_rad'))
                and math.radians(45) <= target['bearing_left_rad'] <= math.radians(135)
                and abs(math.hypot(*point)-target['range_m']) <= 1e-6
                and abs(math.atan2(point[1], point[0])-target['bearing_left_rad']) <= 1e-6
                and target.get('track_id') is not None)

    def _relative_target_pwm(self, target):
        # Left-positive bearing moves behind the car when yaw is insufficient:
        # increase left correction. A range above the actual entry reference
        # similarly requests a modest inward correction. Gains are trial PWM
        # feedback gains, not curvature/speed calibrations or a course radius.
        reference = self.orbit_reference_range_m
        correction = (ORBIT_BEARING_GAIN*(target['bearing_left_rad']-ORBIT_BEARING_RAD)
                      + ORBIT_RANGE_GAIN*(target['range_m']-reference)/reference)
        bias = self.orbit_bias_pwm if self.orbit_bias_pwm is not None else self.initial_presteer_pwm
        return max(NEUTRAL, min(SERVO_MAX, round(bias+correction)))

    def _limit_left_for_known_points(self, nominal, lookahead):
        """Apply a bounded release while a measured inside edge is still nearby.

        This is a relative-point steering heuristic, not a swept-path certificate
        or a mapping from PWM to wheel angle. The reference is the measured rear
        body edge plus the existing clearance, never an assumed rear axle.
        Its clearance bearing is only a correction signal: treating that small
        bearing as an absolute PWM erased the needed exit-turn demand in trial15.
        The hard current-body gate remains independent in the service.
        """
        scan = self._current_scan
        if not isinstance(scan, dict):
            return nominal
        ranges = scan.get('ranges')
        if not isinstance(ranges, list) or len(ranges) != 360:
            return nominal
        rear = REAR_BODY_EXTENT_M + MANEUVER_BODY_CLEARANCE_M
        side = SIDE_BODY_EXTENT_M + MANEUVER_BODY_CLEARANCE_M
        selected = None
        target = nominal
        for index in range(181, 360):
            distance = ranges[index]
            if not _number(distance) or not .02 <= distance <= 12:
                continue  # No interpolation and no free-space assertion.
            angle = math.radians(index)
            x, y = distance*math.cos(angle), -distance*math.sin(angle)
            if not -rear < x <= lookahead or y <= 0:
                continue
            clearance_bearing = math.atan2(max(0., y-side), x+rear)
            point_target = min(SERVO_MAX, NEUTRAL+round(LEFT_TRIAL_GAIN*clearance_bearing))
            if point_target < target:
                target = point_target
                selected = {'index': index, 'range_m': distance,
                            'point_left_m': {'x_m': x, 'y_m': y},
                            'clearance_bearing_rad': clearance_bearing}
        bearing_demand = target
        # Retain the live exit/relative-target demand. Near points can release
        # at most this fraction of the requested left offset from center; this
        # is a trial gain, not a fixed minimum PWM or a physical steering limit.
        target = nominal-round(MAX_INNER_RELEASE_FRACTION*(nominal-bearing_demand))
        self.inner_clearance = {
            'source_seq': scan.get('seq'), 'source_at_ms': scan.get('at_ms'),
            'scope': 'current_known_left_returns_only',
            'nominal_target_pwm': nominal, 'limited_target_pwm': target,
            'clearance_bearing_demand_pwm': bearing_demand,
            'max_release_fraction': MAX_INNER_RELEASE_FRACTION,
            'active': target < nominal, 'limiting_return': selected,
            'lookahead_m': lookahead, 'rear_reference_m': -rear,
            'left_reference_m': side, 'swept_path_certified': False,
            'physical_steering_confirmed': False}
        return target

    def _target(self, candidate, publication_dt):
        # Use the same actual heading/finite-point error as TurnMotion. In this
        # sequence the trend term may release at most 20% of the current error,
        # so a rapidly changing wall estimate does not erase most left steering
        # while the exit is still far from forward. No minimum fixed PWM is
        # introduced: a genuinely smaller current error still releases toward
        # center. These are trial gains, not a physical curvature calibration.
        heading, offset = candidate['heading_left_rad'], candidate['center_offset_left_m']
        lookahead = max(.35, min(1., candidate['width_m']/2))
        if self.geometry_mode == 'opening_wall':
            point = self.turn_goal['target_point_left_m']
            error = (.65*heading+.35*math.atan2(point['y_m'], point['x_m']) if point['x_m'] > 0
                     else heading+math.atan2(offset, lookahead))
        else:
            error = heading+math.atan2(offset, lookahead)
        damped = error
        if self.last_error is not None and .05 <= publication_dt < SCAN_AGE_S:
            rate = (error-self.last_error)/publication_dt
            if abs(rate) <= math.radians(90):
                release = error+.35*rate
                if error > 0:
                    damped = max((1-MAX_TREND_RELEASE_FRACTION)*error, min(error, release))
        self.last_error = error
        # Initial PWM selects neutral preparation only. After adoption, the
        # current measured turn error may request the full 1720 bound; _slew
        # still uses the phase/direction step limit per 100 ms.
        target = max(NEUTRAL, min(SERVO_MAX, NEUTRAL+round(LEFT_TRIAL_GAIN*damped)))
        if self.phase == 'drive':
            target = self._limit_left_for_known_points(target, lookahead)
        self._last_actual_left_target = target if target > NEUTRAL else None
        return target

    def _orbit_inputs(self, scan, lidar_age_s, now, control, safe, presteer_wait):
        """Same advancing raw-scan/control contract, without a fabricated wall."""
        if not _number(now) or now < self.last_now:
            self.lock('invalid_turn_receive_clock')
            return False
        self.last_now = now
        if safe is not True:
            self.lock('turn_safety_rejected')
            return False
        if type(presteer_wait) is not bool or presteer_wait:
            self.lock('turn_presteer_wait_requires_neutral')
            return False
        received = scan.get('received_at') if isinstance(scan, dict) else None
        ranges = scan.get('ranges') if isinstance(scan, dict) else None
        if (not _number(lidar_age_s) or not 0 <= lidar_age_s < SCAN_AGE_S
                or not isinstance(scan, dict) or scan.get('frame_id') != _FRAME
                or type(scan.get('seq')) is not int or scan['seq'] < 0
                or type(scan.get('at_ms')) is not int or scan['at_ms'] < 0
                or not _number(received) or not 0 <= now-received < SCAN_AGE_S
                or not isinstance(ranges, list) or len(ranges) != 360
                or any(r is not None and (not _number(r) or not .02 <= r <= 12) for r in ranges)):
            self.lock('turn_scan_stale_or_invalid')
            return False
        if (not isinstance(control, dict) or type(control.get('armed')) is not bool
                or type(control.get('motor')) is not int or type(control.get('servo')) is not int
                or type(control.get('tick')) is not int or control['tick'] < 0
                or type(control.get('seq')) is not int or control['seq'] < 0
                or not SERVO_MIN <= control['servo'] <= SERVO_MAX):
            self.lock('invalid_turn_adoption_feedback')
            return False
        if self.last_control_tick is not None and (control['tick'] < self.last_control_tick
                or control['seq'] < self.last_control_seq):
            self.lock('reordered_turn_adoption_feedback')
            return False
        self.last_control_tick, self.last_control_seq = control['tick'], control['seq']
        if not control['armed']:
            self.lock('turn_bridge_locked')
            return False
        signature = (_freeze(scan.get('corridor_candidates')), _freeze(scan.get('wall_candidates', [])),
                     _freeze(scan.get('left_turn_goal')), _freeze(scan.get('ranges')))
        seq, published = scan['seq'], scan['at_ms']
        if self.last_seq is not None and seq == self.last_seq:
            if published != self.last_publication or signature != self.last_signature:
                self.lock('duplicate_turn_scan_changed_content')
            elif now-self.last_receive >= SCAN_AGE_S:
                self.lock('turn_scan_not_advancing')
            return False
        publication_dt = 0 if self.last_publication is None else (published-self.last_publication)/1000
        if self.last_seq is not None and (seq < self.last_seq or publication_dt <= 0):
            self.lock('reordered_turn_scan')
            return False
        if self.last_receive is not None and (publication_dt >= SCAN_AGE_S
                or now-self.last_receive >= SCAN_AGE_S
                or abs(publication_dt-(now-self.last_receive)) > .15):
            self.lock('turn_scan_clock_gap')
            return False
        self.last_seq, self.last_publication, self.last_receive = seq, published, now
        self.last_signature = signature
        return True

    def _orbit_update(self, scan, lidar_age_s, now, control, *, safe, presteer_wait):
        advancing = self._orbit_inputs(scan, lidar_age_s, now, control, safe, presteer_wait)
        if self.phase == 'locked':
            return self._result(self.last_now)
        if self.phase == 'drive' and now-self.drive_since+1e-9 >= self.max_drive_s:
            self.begin_coast('maneuver_cumulative_drive_timeout', now)
        if self.phase == 'coast':
            if self.coast_since is not None and now-self.coast_since+1e-9 >= COAST_MAX_S:
                self.lock('coast_standstill_unconfirmed')
            else:
                self.steering_target = NEUTRAL
                self._slew(now)
            return self._result(now)
        if not advancing:
            return self._result(now)
        if not self._usable_target(self.compact_target, scan):
            self.begin_coast('compact_target_lost_or_ambiguous', now)
            self._slew(now)
            return self._result(now)
        just_handed_over = False
        if not self.handover_observed:
            # Continue the adopted left command across normal stage handover.
            # A target never bypasses an ACK, neutral/protection stop or re-arms
            # the bridge. This is command feedback, not physical wheel angle.
            adopted = (control.get('motor') == TRIAL_MOTOR
                       and control.get('servo') == self.servo > NEUTRAL
                       and control.get('command_acked') is True)
            if not adopted:
                if self.handover_wait_since is None:
                    self.handover_wait_since = now
                if now-self.handover_wait_since+1e-9 >= SCAN_AGE_S:
                    self.begin_coast('compact_handover_adoption_unconfirmed', now)
                    self._slew(now)
                else:
                    self.reason = 'compact_handover_waiting_adopted_left'
                return self._result(now)
            self.handover_observed = True
            just_handed_over = True
            self.orbit_since = now
            self.orbit_reference_range_m = self.compact_target['range_m']
            self.orbit_bias_pwm = control['servo']
            self.handover_control = {key: control[key] for key in ('motor', 'servo', 'tick', 'seq')}
            self.handover_control.update(command_acked=True, physical_steering_confirmed=False,
                target_source_seq=scan['seq'], target_source_at_ms=scan['at_ms'])
            self.orbit_track_id = self.compact_target['track_id']
            self.wall_ambiguity_hold = False
            self.geometry_missing = True
            self.alignment_count = 0
            self.alignment_evidence = None
            self.alignment_publication = self.alignment_receive = None
        elif self.compact_target['track_id'] != self.orbit_track_id:
            self.begin_coast('compact_target_identity_changed', now)
            self._slew(now)
            return self._result(now)
        if now-self.orbit_since+1e-9 >= ORBIT_ENTRY_MAX_S:
            self.begin_coast('first_relative_object_entry_trial_timeout', now)
            self._slew(now)
            return self._result(now)
        if just_handed_over:
            self.steering_target = self.natural_steering_target = self.orbit_bias_pwm
            self.reason = 'first_relative_object_handover_keep_adopted_left'
            return self._result(now)
        self.steering_target = self._limit_left_for_known_points(
            self._relative_target_pwm(self.compact_target),
            max(.35, min(1., self.compact_target['range_m'])))
        self.natural_steering_target = self.steering_target
        self.reason = 'first_relative_object_left_orbit_entry'
        self._slew(now)
        return self._result(now)

    def update(self, scan, lidar_age_s, now, control, *, safe=True, entry_stop=None, presteer_wait=False):
        # Expose only this call's raw returns; cached wall endpoints are never
        # used to manufacture current inside-edge evidence.
        self._current_scan = scan
        if (not isinstance(scan, dict) or self.inner_clearance is None
                or self.inner_clearance['source_seq'] != scan.get('seq')
                or self.inner_clearance['source_at_ms'] != scan.get('at_ms')):
            self.inner_clearance = None
        if self.phase == 'locked':
            return self._result(self.last_now)
        if not _number(now) or now < self.last_now:
            self.lock('invalid_turn_receive_clock')
            return self._result(self.last_now)
        try:
            self.compact_target = self.target_tracker.update(scan, now)
        except (ValueError, TypeError, KeyError):
            self.compact_target = None
        if entry_stop is not None:
            # The new trial is target tracking, not the old optional stop input.
            self.lock('unsupported_maneuver_entry_stop')
            return self._result(self.last_now)
        if (self.handover_observed or (self.phase == 'coast' and self.wall_ambiguity_hold)
                or (self.phase == 'drive' and self._usable_target(self.compact_target, scan))):
            return self._orbit_update(scan, lidar_age_s, now, control,
                                      safe=safe, presteer_wait=presteer_wait)
        self.wall_ambiguity_hold = False
        feedback_tick = control.get('tick') if isinstance(control, dict) else None
        self._presteer_adoption_context = {
            'control': dict(control) if isinstance(control, dict) else {},
            'fresh_feedback': (type(feedback_tick) is int
                               and (self.last_control_tick is None or feedback_tick > self.last_control_tick))}
        try:
            return super().update(scan, lidar_age_s, now, control, safe=safe,
                                  presteer_wait=presteer_wait)
        finally:
            self._presteer_adoption_context = None
