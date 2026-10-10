"""Bounded left turn and opt-in observed corridor / second-object continuation.

The compact object's semantic identity is unknown. This module observes no
global pose, speed, physical steering angle, passed cone count or complete lap.
The service retains fresh body-clearance checks, ownership, heartbeat and ACKs.
"""
import copy
import math

from compact_target import CompactTargetTracker, _candidates, MAX_DIAMETER_M, MIN_RANGE_M
from autonomy_live import (FRONT_BODY_EXTENT_M, REAR_BODY_EXTENT_M, SIDE_BODY_EXTENT_M,
                           MANEUVER_BODY_CLEARANCE_M)
from turn_motion import (TurnMotion, NEUTRAL, TRIAL_MOTOR, SERVO_MIN, SERVO_MAX, SCAN_AGE_S,
                         PWM_STEP, PWM_INTERVAL_S, _FRAME, _freeze, _number, _valid_candidate,
                         _valid_wall, _wrap, _surface_supports, _same_surface)

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
MAX_EXIT_WIDTH_CHANGE_FRACTION = .10
RIGHT_EXIT_MIN_PWM = 1350  # Existing manual right bound; no measured curvature implied.
RIGHT_EXIT_MAX_S = 3.0
# Explicit continue_route only: once the current first-target support is back
# at the measured body front, every further left step only extends the orbit.
# Release toward neutral (never past it) faster than the generic 20 PWM step.
# Rightward steps and every other phase keep their existing limits.
ROUTE_EXIT_RELEASE_PWM_STEP = 40
# Consecutive strictly fresh full-support frames for the body-front release.
ROUTE_EXIT_PREPARE_FRAMES = 2
# Explicit continue_route only: trial40 had two fresh permitting endpoint
# bearings (1947/1948) but lost the native opening before the third, so the
# entry cap coasted the left turn. Same frame count as the exit prepare.
ROUTE_ENTRY_RELEASE_FRAMES = 2
# Explicit continue_route only. Trial43: after the first-target right exit the
# second target appeared at the LEFT front (+11 deg, 1.9 m) and was passed on
# the wrong side at about 1.8 m/s (lidar closing rate). Passing it clockwise
# (support kept on the right) needs a left swing and later a right turn within
# about one second each, so every second-target steering step uses the route
# exit release step already exercised on the car in trial43 (center crossing
# and the right-after-center-ACK rule are unchanged).
SECOND_ORBIT_PWM_STEP = 40
# Lateral gap from the lidar origin to the support's current left edge at the
# bypass point: keeps the support above the detector's minimum range while it
# is abeam (trial43 lost the second support below .35 m) and leaves .26 m to
# the measured body side, above the existing .08 m maneuver clearance.
SECOND_BYPASS_SUPPORT_GAP_M = MIN_RANGE_M+.05
# Same-frame native wall-window pairing reuses the native corridor gates
# (corridor.rs): parallel 12 deg, width .65..2.5 m, shared support .35 m.
PAIR_MAX_ANGLE_RAD = math.radians(12)
PAIR_MIN_WIDTH_M, PAIR_MAX_WIDTH_M = .65, 2.5
PAIR_MIN_SHARED_SUPPORT_M = .35
# Several windows of the same two physical boards differ by a few degrees
# (trial39 scan480: 6 deg). The merge tolerance reuses the parallel gate; a
# different physical axis (corner/T) differs far more and stays ambiguous.
PAIR_MERGE_HEADING_RAD = PAIR_MAX_ANGLE_RAD
PAIR_MERGE_OFFSET_M = .15


def validate_maneuver_initial_pwm(value):
    if type(value) is not int or not INITIAL_PWM_MIN <= value <= SERVO_MAX:
        raise ValueError('invalid_maneuver_initial_presteer_pwm')
    return value


class ManeuverSequence(TurnMotion):
    """Single arm / cumulative budget; terminal coast never restarts.

    Explicit route mode may perform one fresh-evidence neutral quality wait.
    It does not reset a drive/phase budget or certify a physical two-cone path.
    """
    def __init__(self, started_at, max_drive_s=10.0, initial_presteer_pwm=DEFAULT_INITIAL_PWM,
                 continue_route=False):
        if type(continue_route) is not bool:
            raise ValueError('invalid_continue_route')
        validate_maneuver_initial_pwm(initial_presteer_pwm)
        super().__init__(started_at, max_drive_s=max_drive_s,
                         initial_presteer_pwm=initial_presteer_pwm)
        # User-selected neutral preparation budget for this experiment only.
        # Parent construction and default TurnMotion retain their five seconds.
        self.max_presteer_s = PRESTEER_MAX_S
        self.continue_route = continue_route
        self.target_tracker = (CompactTargetTracker(maintenance_bearing_rad=(-math.pi, math.pi),
                                                   select_associated=True)
                               if continue_route else CompactTargetTracker())
        self.compact_target = None
        self._last_confirmed_compact_target = None
        self.compact_target_error = None
        self.compact_loss_evidence = None
        self.handover_observed = False
        self.orbit_since = self.orbit_reference_range_m = None
        self.orbit_bias_pwm = None
        self.orbit_feedback = None
        self.orbit_left_entry_boost = False
        self._orbit_abeam_count = 0
        self._orbit_abeam_publication = self._orbit_abeam_receive = None
        self.left_turn_feedback = None
        self.left_handover_preparing = False
        self.handover_control = None
        self.handover_wait_since = None
        self.orbit_track_id = None
        self.coast_since = None
        self.wall_ambiguity_hold = False
        self._last_actual_left_target = None
        self._presteer_adoption_context = None
        self._current_scan = None
        self.inner_clearance = None
        self.quality_coast_servo = None
        self.quality_wait_refusal = None
        self.quality_coast_reason = None
        self.entry_bearing_required = False
        self.entry_bearing_released = False
        self.entry_bearing = None
        self._incoming_association = None
        self._entry_last_receive = self._entry_last_publication = None
        self._entry_ready_count = 0
        self._entry_ready_receive = self._entry_ready_publication = None
        self._entry_ready_observation_type = None
        self._entry_ready_history = []
        self.adopted_presteer_pwm = None
        self.first_pass_preparing = False
        self.first_pass_evidence = None
        self.first_pass_progress = None
        self._first_pass_history = []
        self._first_exit_prepare_history = []
        self._quality_exit_prepare_ready = False
        # Neutral-wait release: decided from current full first-target support
        # (or a release already latched before the wait). Release only.
        self.quality_wait_release = None
        self.quality_wait_previous_servo = None
        self._wait_release_history = []
        self.first_exit_prepare_evidence = None
        self.right_exit_since = None
        self.right_exit_center_ack = None
        self.right_exit_adoption_wait_since = None
        self.right_exit_geometry = None
        self._right_exit_last_receive = self._right_exit_last_publication = None
        self.route_stage = 'first_target'
        self.route_events = {}
        self.quality_wait_since = self.quality_wait_servo = None
        self.quality_wait_used = False
        self.quality_wait_evidence = None
        self.quality_resume_pending = False
        self.quality_resume_since = None
        self._quality_good_history = []
        self._quality_pass_history = []
        self._route_alignment_history = []
        self.route_alignment_evidence = None
        self._route_center_ack = None
        self.second_tracker = self._new_second_tracker() if continue_route else None
        self.second_target = None
        self.second_track_id = None
        self.second_target_handover_observed = False
        self.second_center_ack = None
        self.second_orbit_since = None
        self.second_pass_evidence = None
        self.second_feedback = None
        self.route_end_coast_servo = None
        self.role_distinct_evidence = None
        self.second_acquisition_reason = None
        self._second_pass_history = []

    @staticmethod
    def _new_second_tracker():
        # Trial43: the second support first appears ahead on either side;
        # once confirmed it moves from the front through the right to behind
        # the rear, where the pass is matured over .25 s.
        return CompactTargetTracker(acquisition_bearing_rad=(-math.pi/2, math.pi/2),
            maintenance_bearing_rad=(-math.pi, math.pi/2), select_associated=True)

    def _route_event(self, name, scan, control, evidence=None):
        # One snapshot per transition, never an unbounded in-control log.
        if name not in self.route_events:
            self.route_events[name] = {'scan': copy.deepcopy({k: scan.get(k) for k in
                ('frame_id', 'seq', 'at_ms', 'received_at', 'ranges', 'corridor_candidates')}),
                'control': copy.deepcopy(control), 'evidence': copy.deepcopy(evidence)}

    @staticmethod
    def _full_target_points(target, scan, identity):
        if (not isinstance(target, dict) or target.get('track_id') != identity
                or target.get('confirmed') is not True
                or target.get('tracking_only_boundary_gap') is not False
                or target.get('source_seq') != scan.get('seq')
                or target.get('source_at_ms') != scan.get('at_ms')
                or target.get('source_received_at') != scan.get('received_at')):
            return None
        bins, ranges = target.get('support_bins'), scan.get('ranges')
        if (not isinstance(bins, list) or len(bins) < 5 or len(set(bins)) != len(bins)
                or not isinstance(ranges, list) or len(ranges) != 360
                or any(type(i) is not int or not 0 <= i < 360 or not _number(ranges[i]) for i in bins)):
            return None
        return [(ranges[i]*math.cos(math.radians(i)), -ranges[i]*math.sin(math.radians(i))) for i in bins]

    @staticmethod
    def _mature_observation(history, scan, duration):
        row = (scan['seq'], scan['at_ms'], scan['received_at'])
        if history and row[0] == history[-1][0]:
            return False
        if history and (row[0] <= history[-1][0] or row[1] <= history[-1][1]
                        or row[2] <= history[-1][2]
                        or (row[1]-history[-1][1])/1000 >= SCAN_AGE_S
                        or row[2]-history[-1][2] >= SCAN_AGE_S
                        or abs((row[1]-history[-1][1])/1000-(row[2]-history[-1][2])) > .15):
            history.clear()
        history.append(row)
        del history[:-6]
        return (len(history) >= 3 and (row[1]-history[0][1])/1000+1e-9 >= duration
                and row[2]-history[0][2]+1e-9 >= duration)

    @staticmethod
    def _consecutive_fresh(history, scan, count=ROUTE_EXIT_PREPARE_FRAMES):
        """Same freshness/ordering resets as _mature_observation, frame count only.

        A repeated scan never counts twice; a gap or clock mismatch restarts.
        """
        row = (scan['seq'], scan['at_ms'], scan['received_at'])
        if history and row[0] == history[-1][0]:
            return len(history) >= count
        if history and (row[0] <= history[-1][0] or row[1] <= history[-1][1]
                        or row[2] <= history[-1][2]
                        or (row[1]-history[-1][1])/1000 >= SCAN_AGE_S
                        or row[2]-history[-1][2] >= SCAN_AGE_S
                        or abs((row[1]-history[-1][1])/1000-(row[2]-history[-1][2])) > .15):
            history.clear()
        history.append(row)
        del history[:-6]
        return len(history) >= count

    def _route_exit_release_active(self):
        return (self.continue_route and self.handover_observed
                and ((self.phase == 'drive' and (self.first_pass_preparing
                                                 or self.first_pass_evidence is not None))
                     or (self.phase == 'quality_wait' and self.quality_wait_release is not None)))

    @staticmethod
    def _paired_window_corridor(scan):
        """Pair same-frame native wall windows into one forward corridor.

        Only current native windows are used; nothing is cached or extended.
        Gates mirror corridor.rs. Two distinct axes are ambiguous (None, True).
        Returns (candidate | None, ambiguous).
        """
        walls = scan.get('wall_candidates')
        if not isinstance(walls, list) or len(walls) > 64:
            return None, False
        forward = [w for w in walls if _valid_wall(w) and abs(w['heading_left_rad']) < math.pi/2]
        pairs = []
        for left in forward:
            if left['rho_left_m'] <= 0:
                continue
            for right in forward:
                if right['rho_left_m'] >= 0:
                    continue
                difference = _wrap(left['heading_left_rad']-right['heading_left_rad'])
                if abs(difference) > PAIR_MAX_ANGLE_RAD:
                    continue
                heading = _wrap(right['heading_left_rad']+difference/2)
                if abs(heading) >= math.pi/2:
                    continue
                c, s = math.cos(heading), math.sin(heading)
                ends = {}
                for name, wall in (('left', left), ('right', right)):
                    points = [wall['support_start_left_m'], wall['support_end_left_m']]
                    normal = [-s*p['x_m']+c*p['y_m'] for p in points]
                    along = [c*p['x_m']+s*p['y_m'] for p in points]
                    ends[name] = (sum(normal)/2, min(along), max(along))
                width = ends['left'][0]-ends['right'][0]
                shared = min(ends['left'][2], ends['right'][2])-max(ends['left'][1], ends['right'][1])
                if (not ends['left'][0] > 0 > ends['right'][0]
                        or not PAIR_MIN_WIDTH_M <= width <= PAIR_MAX_WIDTH_M
                        or shared < PAIR_MIN_SHARED_SUPPORT_M):
                    continue
                pairs.append({'heading_left_rad': heading,
                    'center_offset_left_m': (ends['left'][0]+ends['right'][0])/2,
                    'width_m': width, 'left_wall_points': left['points'],
                    'right_wall_points': right['points'], 'support_span_m': shared,
                    'fit_error_m': max(left['fit_error_m'], right['fit_error_m']),
                    'origin_between_walls': True, 'candidate_only': True,
                    'turn_path_certified': False})
        if not pairs:
            return None, False
        base = max(pairs, key=lambda p: p['support_span_m'])
        for p in pairs:
            # Several windows of the same two physical walls merge; any other
            # axis/width/offset is a second interpretation, never a choice.
            if (abs(_wrap(p['heading_left_rad']-base['heading_left_rad'])) > PAIR_MERGE_HEADING_RAD
                    or abs(p['width_m']-base['width_m']) > max(.15, MAX_EXIT_WIDTH_CHANGE_FRACTION*base['width_m'])
                    or abs(p['center_offset_left_m']-base['center_offset_left_m']) > PAIR_MERGE_OFFSET_M):
                return None, True
        return {**base, 'source_kind': 'native_wall_window_pair', 'pair_count': len(pairs)}, False

    def _clear_exit_prepare_window(self):
        self._first_exit_prepare_history.clear()
        self._quality_exit_prepare_ready = False

    def _latch_exit_prepare(self, scan, control, *, basis, history=None):
        points = self._full_target_points(self.compact_target, scan, self.orbit_track_id)
        history = self._first_exit_prepare_history if history is None else history
        if points is None or not history:
            return
        self.first_pass_preparing = True
        self.orbit_left_entry_boost = False
        self.first_exit_prepare_evidence = {
            'source_seq': scan['seq'], 'source_at_ms': scan['at_ms'],
            'received_at': scan['received_at'],
            'center_x_m': sum(p[0] for p in points)/len(points),
            'track_id': self.orbit_track_id, 'support_bins': list(self.compact_target['support_bins']),
            'observed_frontmost_x_m': max(p[0] for p in points),
            'observed_left_body_gap_m': min(p[1] for p in points)-SIDE_BODY_EXTENT_M,
            'observation_count': len(history), 'first_source_seq': history[0][0],
            'last_preparation_source_seq': history[-1][0],
            'publication_span_s': (history[-1][1]-history[0][1])/1000,
            'receive_span_s': history[-1][2]-history[0][2],
            'adopted_control': {k: control.get(k) for k in ('motor', 'servo', 'tick', 'seq', 'command_acked')},
            'basis': basis, 'action': 'release_left_toward_neutral_only',
            'observed_rear_pass': False, 'right_steering_authorized': False,
            'physical_cone_pass_certified': False}
        self._route_event('first_exit_prepare', scan, control, self.first_exit_prepare_evidence)

    def begin_quality_wait(self, now, control, *, scan=None):
        """One opt-in neutral wait; this never reopens a terminal coast."""
        def refuse(reason):
            # Trial41: the refusal reason was not recorded; keep it for analysis.
            self.quality_wait_refusal = {'reason': reason, 'at': now if _number(now) else None,
                'control_servo': control.get('servo'), 'command_acked': control.get('command_acked'),
                'servo': self.servo, 'scan_seq': scan.get('seq') if isinstance(scan, dict) else None}
            return False
        if (not self.continue_route or self.quality_wait_used or self.phase != 'drive'
                or not self.handover_observed or self.first_pass_evidence is not None
                or self.right_exit_since is not None or self.route_stage != 'first_target'
                or not _number(now) or now < self.last_now):
            return refuse('route_state')
        if (control.get('armed') is not True or control.get('motor') != TRIAL_MOTOR
                or control.get('command_acked') is not True
                or not NEUTRAL < control.get('servo', 0) == self.servo <= SERVO_MAX):
            return refuse('control_not_acked_left')
        scan = self._current_scan if scan is None else scan
        if (not isinstance(scan, dict) or not _number(scan.get('received_at'))
                or not 0 <= now-scan['received_at'] < SCAN_AGE_S):
            return refuse('scan_stale')
        observed = self.target_tracker.update(scan, now, acquire=True)
        if self._full_target_points(observed, scan, self.orbit_track_id) is None:
            return refuse('target_not_fully_isolated')
        self.compact_target = observed
        self._current_scan = scan
        self.quality_wait_used = True
        self.quality_wait_since, self.quality_wait_servo = now, self.servo
        self.quality_wait_evidence = {'entry_seq': scan['seq'], 'entry_at_ms': scan['at_ms'],
            'entry_received_at': scan['received_at'], 'held_servo': self.servo,
            'target_id': self.orbit_track_id, 'maximum_resumes': 1}
        self._quality_good_history.clear()
        self._quality_pass_history.clear()
        self._first_pass_history.clear()
        self._clear_exit_prepare_window()
        self._wait_release_history.clear()
        self.quality_wait_previous_servo = None
        # A release decided from healthy drive frames before this wait stays a
        # release-only intention; it never carries observation windows.
        self.quality_wait_release = ({'basis': 'release_latched_before_neutral_wait',
                                      'source_seq': self.first_exit_prepare_evidence.get('source_seq')
                                      if isinstance(self.first_exit_prepare_evidence, dict) else None,
                                      'held_servo_at_entry': self.servo}
                                     if self.first_pass_preparing else None)
        self.first_pass_preparing = False
        self.phase, self.reason = 'quality_wait', 'route_quality_wait_neutral'
        self.steering_target = self.servo
        self._route_event('quality_wait', scan, control, self.quality_wait_evidence)
        return True

    def _quality_wait_update(self, scan, now, control, advancing, quality_clear, resume_ready):
        self.steering_target = self.quality_wait_servo
        if (now-self.quality_wait_since >= COAST_MAX_S
                or now-self.drive_since >= self.max_drive_s
                or now-self.orbit_since >= ORBIT_ENTRY_MAX_S):
            self.lock('route_quality_wait_original_budget_expired')
            return self._result(now)
        servo_feedback = control.get('servo')
        if servo_feedback == self.quality_wait_servo:
            self.quality_wait_previous_servo = None
        # Only the immediately preceding release step may still be reported
        # while its command is not yet ACKed; nothing else may change steering.
        pending_release = (self.quality_wait_previous_servo is not None
                           and servo_feedback == self.quality_wait_previous_servo
                           and control.get('command_acked') is False)
        if ((servo_feedback != self.quality_wait_servo and not pending_release)
                or control.get('motor') not in (NEUTRAL, TRIAL_MOTOR)
                or (control.get('neutral_acked') is True and control['motor'] != NEUTRAL)):
            self.lock('route_quality_wait_feedback_changed')
            return self._result(now)
        neutral = control.get('neutral_acked') is True and control['motor'] == NEUTRAL
        if not neutral and now-self.quality_wait_since >= SCAN_AGE_S:
            self.lock('route_quality_wait_neutral_ack_timeout')
            return self._result(now)
        points = self._full_target_points(self.compact_target, scan, self.orbit_track_id)
        if points is None:
            self._clear_exit_prepare_window()
            self.lock('route_quality_wait_target_unconfirmed')
            return self._result(now)
        if quality_clear is not True or not neutral:
            self._clear_exit_prepare_window()
            self._quality_good_history.clear()
        front, side = max(p[0] for p in points), min(p[1] for p in points)-SIDE_BODY_EXTENT_M
        # Release-only decision in the neutral wait. Degraded scan quality does
        # not hide this complete, isolated same-ID support; it never advances a
        # pass, authorizes right steering or restores motor power.
        if advancing and neutral and self.quality_wait_release is None:
            if front <= FRONT_BODY_EXTENT_M and side >= MANEUVER_BODY_CLEARANCE_M:
                if self._consecutive_fresh(self._wait_release_history, scan):
                    self.quality_wait_release = {'basis': 'neutral_wait_current_support_at_body_front',
                        'source_seq': scan['seq'], 'source_at_ms': scan['at_ms'],
                        'received_at': scan['received_at'], 'track_id': self.orbit_track_id,
                        'observed_frontmost_x_m': front, 'observed_left_body_gap_m': side,
                        'observation_count': len(self._wait_release_history),
                        'first_source_seq': self._wait_release_history[0][0],
                        'held_servo_at_decision': self.quality_wait_servo,
                        'action': 'release_left_toward_neutral_only',
                        'right_steering_authorized': False, 'physical_cone_pass_certified': False}
                    self._route_event('quality_wait_release', scan, control, self.quality_wait_release)
            else:
                self._wait_release_history.clear()
        def hold():
            # A release step is issued only on ticks that do not request the
            # powered resume, so that output always holds an ACKed command.
            if (self.phase == 'quality_wait' and self.quality_wait_release is not None and neutral
                    and servo_feedback == self.quality_wait_servo and self.servo > NEUTRAL):
                before = self.servo
                self.steering_target = NEUTRAL
                self._slew(now)
                if self.servo != before:
                    self.quality_wait_previous_servo = before
                    self.quality_wait_servo = self.servo
                    self.reason = 'route_quality_wait_neutral_release_left'
                self.steering_target = self.quality_wait_servo
            return self._result(now)
        if not advancing:
            return hold()
        passed = front <= -REAR_BODY_EXTENT_M and side >= MANEUVER_BODY_CLEARANCE_M
        if passed:
            pass_ready = self._mature_observation(self._quality_pass_history, scan, .25)
        else:
            self._quality_pass_history.clear()
            pass_ready = False
        if quality_clear is not True or not neutral:
            return hold()
        # Rebuild only from NEW, fully healthy observations in this neutral
        # wait. No pre-wait window or degraded frame can mature the intention.
        if front <= FRONT_BODY_EXTENT_M and side >= MANEUVER_BODY_CLEARANCE_M:
            self._quality_exit_prepare_ready = self._mature_observation(
                self._first_exit_prepare_history, scan, .25)
        else:
            self._clear_exit_prepare_window()
        good = self._mature_observation(self._quality_good_history, scan, .30)
        if (not good or resume_ready is not True
                or servo_feedback != self.quality_wait_servo or self.quality_wait_previous_servo is not None):
            # Resume only on an ACKed, unchanged neutral-wait steering command.
            return hold()
        if passed:
            if not pass_ready:
                return hold()
            _, rejection = self._right_exit_target(scan, now)
            if rejection is not None:
                self.lock('route_quality_wait_pass_corridor_unavailable')
                return self._result(now)
            self.first_pass_preparing = True
            self.first_pass_evidence = {'source_seq': scan['seq'], 'source_at_ms': scan['at_ms'],
                'received_at': scan['received_at'], 'track_id': self.orbit_track_id,
                'support_bins': list(self.compact_target['support_bins']),
                'observed_frontmost_x_m': front, 'observed_left_body_gap_m': side,
                'basis': 'current_complete_support_behind_rear_during_neutral_wait',
                'physical_cone_pass_certified': False, 'swept_path_certified': False}
            self.right_exit_since, self.route_stage = now, 'right_align'
            self._route_event('first_pass', scan, control, self.first_pass_evidence)
        self.phase, self.reason = 'drive', 'route_quality_resume_waiting_ack'
        self.quality_resume_pending, self.quality_resume_since = True, now
        self.quality_wait_evidence.update(resume_seq=scan['seq'], resume_at_ms=scan['at_ms'],
            resume_received_at=scan['received_at'], good_frame_count=len(self._quality_good_history))
        self._first_pass_history.clear()
        if not self._quality_exit_prepare_ready:
            self._clear_exit_prepare_window()
        self._route_event('quality_resume_request', scan, control, self.quality_wait_evidence)
        # First powered output holds the actually ACKed neutral-wait steering.
        return self._result(now)

    def _measurement(self, scan, candidates, publication_dt):
        candidate = super()._measurement(scan, candidates, publication_dt)
        if self.geometry_mode == 'opening_wall':
            self.entry_bearing_required = True
        return candidate

    def _endpoint_observation(self, scan):
        super()._endpoint_observation(scan)
        if not self.entry_bearing_required or self.entry_bearing_released:
            return
        # Called only after native geometry and its same-scan ray binding have
        # been validated. The cached turn_goal may contain an OLD endpoint.
        opening = scan.get('left_turn_goal')
        observation = ({'endpoint_return': opening['incoming_left_end_support'],
                        'heading_left_rad': opening['incoming_heading_left_rad'],
                        'observation_type': 'full_opening', 'wall': None}
                       if opening is not None else self._current_incoming_endpoint(scan))
        if observation is None:
            self.entry_bearing = None
            self._entry_ready_count = 0
            self._entry_ready_receive = self._entry_ready_publication = None
            self._entry_ready_observation_type = None
            self._entry_ready_history.clear()
            return
        point = observation['endpoint_return']['point_left_m']
        heading = observation['heading_left_rad']
        c, s = math.cos(heading), math.sin(heading)
        # This separate descriptor associates the next CURRENT observation.
        # It never supplies a missing point, heading, opening width or outer wall.
        received = scan.get('received_at')
        prior = self._incoming_association
        if (_number(received) and received >= 0 and 0 <= self.last_now-received < SCAN_AGE_S
                and (prior is None or (scan['seq'] > prior['source_seq']
                    and scan['at_ms'] > prior['source_at_ms']
                    and received > prior['source_received_at']))):
            self._incoming_association = {'source_seq': scan['seq'], 'source_at_ms': scan['at_ms'],
                'source_received_at': received, 'heading_left_rad': heading,
                'rho_left_m': -s*point['x_m']+c*point['y_m'],
                'endpoint_return': copy.deepcopy(observation['endpoint_return'])}
        corners = [(x, y) for x in (-REAR_BODY_EXTENT_M, FRONT_BODY_EXTENT_M)
                   for y in (-SIDE_BODY_EXTENT_M, SIDE_BODY_EXTENT_M)]
        # Project the measured body, with the existing 8 cm allowance, onto
        # the current incoming-wall axes. Neither distance is a course length.
        front = max(c*x+s*y for x, y in corners)+MANEUVER_BODY_CLEARANCE_M
        side = max(-s*x+c*y for x, y in corners)+MANEUVER_BODY_CLEARANCE_M
        forward_gap = c*point['x_m']+s*point['y_m']-front
        lateral_gap = -s*point['x_m']+c*point['y_m']-side
        bearing = heading+math.atan2(max(0., lateral_gap), max(0., forward_gap))
        cap = max(NEUTRAL, min(SERVO_MAX, NEUTRAL+round(LEFT_TRIAL_GAIN*bearing)))
        if lateral_gap <= 0:
            cap = NEUTRAL
        # The native full-junction candidate can disappear as the car rotates.
        # Trial25 already put the CURRENT endpoint at the inflated front
        # projection, but a second 250 ms maturity wait delayed the handoff
        # until that candidate was gone. This direct same-frame geometry can
        # end the early cap; it does not assert rear/body or path clearance.
        front_projection_reached = forward_gap <= 0 and lateral_gap > 0 and cap > NEUTRAL
        permits_prepared_left = cap >= self.initial_presteer_pwm
        observation_clock = (received if observation['observation_type'] == 'tracked_current_incoming_wall'
                             else self.last_now)
        # Keep the existing full-opening maturity clock. A change of source
        # starts a new maturity interval rather than mixing decision/receipt time.
        if self._entry_ready_observation_type != observation['observation_type']:
            self._entry_ready_count = 0
            self._entry_ready_receive = self._entry_ready_publication = None
            self._entry_ready_history.clear()
        self._entry_ready_observation_type = observation['observation_type']
        if permits_prepared_left:
            if self._entry_ready_count == 0:
                self._entry_ready_receive = observation_clock
                self._entry_ready_publication = scan['at_ms']
            self._entry_ready_count += 1
            if (self._entry_ready_count >= 3
                    and observation_clock-self._entry_ready_receive >= .25
                    and (scan['at_ms']-self._entry_ready_publication)/1000 >= .25):
                self.entry_bearing_released = True
            # Route mode: the same permitting bearing in two strictly fresh,
            # consistently clocked frames; a repeat, gap or clock skew restarts.
            if (self.continue_route and _number(received)
                    and self._consecutive_fresh(self._entry_ready_history, scan,
                                                ROUTE_ENTRY_RELEASE_FRAMES)):
                self.entry_bearing_released = True
        else:
            self._entry_ready_count = 0
            self._entry_ready_receive = self._entry_ready_publication = None
            self._entry_ready_history.clear()
        if front_projection_reached:
            self.entry_bearing_released = True
        self._entry_last_receive, self._entry_last_publication = observation_clock, scan['at_ms']
        self.entry_bearing = {'source_seq': scan['seq'], 'source_at_ms': scan['at_ms'],
            'source_received_at': received,
            'endpoint_return': copy.deepcopy(observation['endpoint_return']),
            'observation_type': observation['observation_type'],
            'current_incoming_wall': copy.deepcopy(observation['wall']),
            'current_near_support_bins': list(observation.get('near_support_bins', [])),
            'adjacent_far_return': copy.deepcopy(observation.get('adjacent_far_return')),
            'incoming_heading_left_rad': heading, 'body_forward_support_m': front,
            'body_left_support_m': side, 'forward_gap_m': forward_gap,
            'lateral_gap_m': lateral_gap, 'clearance_bearing_left_rad': bearing,
            'steering_cap_pwm': cap, 'prepared_left_pwm': self.initial_presteer_pwm,
            'confirmation_count': self._entry_ready_count,
            'released': self.entry_bearing_released,
            'release_basis': ('current_endpoint_at_body_front_projection' if front_projection_reached
                              else 'prepared_left_bearing_matured' if self.entry_bearing_released else None),
            'route_fresh_frames': len(self._entry_ready_history) if self.continue_route else None,
            'scope': 'early_turn_endpoint_bearing_heuristic',
            'endpoint_passed_proven': False, 'swept_path_certified': False,
            'physical_curvature_calibrated': False}

    def _current_incoming_endpoint(self, scan):
        """Associate a currently observed wall end after a full opening was seen.

        A native fit may stop at a sampling window. Its forward end must bind
        to a current return, followed immediately by a real farther return,
        and retain a contiguous, actually observed inlier chain behind it.
        """
        old = self._incoming_association
        if self.phase != 'drive' or old is None:
            return None
        publication_dt = (scan['at_ms']-old['source_at_ms'])/1000
        received = scan.get('received_at')
        if (scan['seq'] <= old['source_seq'] or not 0 < publication_dt < SCAN_AGE_S
                or not _number(received) or received < 0 or not 0 <= self.last_now-received < SCAN_AGE_S
                or received <= old['source_received_at']
                or not 0 <= self.last_now-old['source_received_at'] < SCAN_AGE_S
                or abs(publication_dt-(received-old['source_received_at'])) > .15):
            return None
        ranges, walls = scan.get('ranges'), scan.get('wall_candidates')
        if (not isinstance(ranges, list) or len(ranges) != 360
                or any(r is not None and (not _number(r) or not .02 <= r <= 12) for r in ranges)
                or not isinstance(walls, list) or len(walls) > 64
                or any(not _valid_wall(w) for w in walls)):
            return None
        gate = min(math.radians(30), .12+math.radians(90)*publication_dt)
        previous = old['endpoint_return']['point_left_m']
        matches = []
        for original in walls:
            wall = copy.deepcopy(original)
            heading = wall['heading_left_rad']
            if abs(_wrap(heading-old['heading_left_rad'])) > math.pi/2:
                heading = wall['heading_left_rad'] = _wrap(heading+math.pi)
                wall['rho_left_m'] = -wall['rho_left_m']
                wall['support_start_left_m'], wall['support_end_left_m'] = (
                    wall['support_end_left_m'], wall['support_start_left_m'])
            end = wall['support_end_left_m']
            if (abs(_wrap(heading-old['heading_left_rad'])) > gate or wall['rho_left_m'] <= 0
                    or abs(wall['rho_left_m']-old['rho_left_m']) > .30
                    or math.hypot(end['x_m']-previous['x_m'], end['y_m']-previous['y_m']) > .30):
                continue
            observation = self._observed_wall_endpoint(wall, ranges)
            if observation is not None and not any(wall == item['wall'] for item in matches):
                matches.append(observation)
        if not matches:
            return None
        supports = _surface_supports(scan, [item['wall'] for item in matches])
        groups = []
        for observation, support in zip(matches, supports):
            if support is None:
                continue
            # Merge only aliases of the same real endpoint AND current surface.
            # Similar headings alone never merge distinct parallel boards.
            group = next((g for g in groups if all(
                observation['endpoint_return']['index'] == prior['endpoint_return']['index']
                and _same_surface(support, evidence) is True for prior, evidence in g)), None)
            if group is None:
                groups.append([(observation, support)])
            else:
                group.append((observation, support))
        return groups[0][0][0] if len(groups) == 1 else None

    @staticmethod
    def _observed_wall_endpoint(wall, ranges):
        heading, end = wall['heading_left_rad'], wall['support_end_left_m']
        c, s = math.cos(heading), math.sin(heading)
        along = lambda p: c*p['x_m']+s*p['y_m']
        normal = lambda p: -s*p['x_m']+c*p['y_m']
        low, high = along(wall['support_start_left_m']), along(end)
        if low >= high or abs(normal(end)-wall['rho_left_m']) > .05:
            return None
        index = round(-math.degrees(math.atan2(end['y_m'], end['x_m']))) % 360
        distance = ranges[index]
        if distance is None:
            return None
        point = lambda i: {'x_m': ranges[i]*math.cos(math.radians(i)),
                           'y_m': -ranges[i]*math.sin(math.radians(i))}
        measured = point(index)
        if math.hypot(measured['x_m']-end['x_m'], measured['y_m']-end['y_m']) > 1e-6:
            return None
        for direction in (-1, 1):
            outside = (index+direction) % 360
            far = ranges[outside]
            if far is None or far <= distance+.18:
                continue
            # Require the adjacent beam to continue forward along this wall's
            # orientation, independently of its much farther observed range.
            adjacent = {'x_m': distance*math.cos(math.radians(outside)),
                        'y_m': -distance*math.sin(math.radians(outside))}
            if along(adjacent) <= high or along(point(outside)) <= high:
                continue
            previous = measured
            support_bins = [index]
            # Match the native wall's minimum actual support count at THIS end;
            # a separate mature patch elsewhere on the line is insufficient.
            for step in range(1, 16):
                inside = (index-direction*step) % 360
                if ranges[inside] is None:
                    break
                current = point(inside)
                if (not low-1e-6 <= along(current) < high
                        or abs(normal(current)-wall['rho_left_m']) > .05
                        or math.hypot(current['x_m']-previous['x_m'], current['y_m']-previous['y_m']) > .18):
                    break
                support_bins.append(inside)
                previous = current
            if len(support_bins) == 16:
                return {'endpoint_return': {'index': index,
                            'angle_left_rad': _wrap(-math.radians(index)),
                            'range_m': distance, 'point_left_m': measured},
                        'heading_left_rad': heading,
                        'observation_type': 'tracked_current_incoming_wall', 'wall': wall,
                        'near_support_bins': support_bins,
                        'adjacent_far_return': {'index': outside, 'range_m': far}}
        return None

    def _entry_target(self, target):
        if not self.entry_bearing_required or self.entry_bearing_released:
            return target
        if self.entry_bearing is not None:
            return min(target, self.entry_bearing['steering_cap_pwm'])
        if self.phase == 'presteer':
            return NEUTRAL  # Missing current endpoint cannot mature adoption.
        # A brief missing endpoint only holds the already commanded steering;
        # it cannot authorize more left, target handover or a renewed lease.
        if (self._entry_last_receive is not None
                and self.last_now-self._entry_last_receive < SCAN_AGE_S
                and (self.last_publication-self._entry_last_publication)/1000 < SCAN_AGE_S):
            return self.servo
        self.begin_coast('left_turn_entry_endpoint_lost', self.last_now)
        return NEUTRAL

    def _presteer_target(self, target):
        return self._entry_target(super()._presteer_target(target))

    def begin_quality_coast(self, now, control):
        return self._begin_adopted_left_coast('turn_perception_unavailable', now, control)

    def begin_route_quality_coast(self, now, control):
        """Route fallback for bounded missing returns in the first-target stage.

        Trial41 halted, snapping 1689 to center while still rolling, and hit
        the outer board; trial42 halted the same way on front_sparse during
        the left turn. Cut power and hold the ACKed left instead, as the
        default sequence has since trial20, before or after handover. An
        un-ACKed step keeps the old halt (a late ACK would change the held
        feedback).
        """
        if not self.continue_route or self.route_stage != 'first_target':
            return False
        return self._begin_adopted_left_coast('route_quality_coast_adopted_left', now, control,
                                              require_handover=False)

    def _begin_adopted_left_coast(self, reason, now, control, *, require_handover=True):
        """Cut power while retaining only an ACKed left orbit-entry command."""
        if (self.phase != 'drive' or (require_handover and not self.handover_observed)
                or self.first_pass_evidence is not None or self.right_exit_since is not None
                or control.get('armed') is not True
                or control.get('motor') != TRIAL_MOTOR
                or control.get('command_acked') is not True
                or type(control.get('servo')) is not int
                or not NEUTRAL < control['servo'] == self.servo <= SERVO_MAX):
            return False
        self.begin_coast(reason, now)
        if self.phase != 'coast':
            return False
        self.quality_coast_servo = control['servo']
        self.quality_coast_reason = reason
        self.steering_target = self.quality_coast_servo
        return True

    def begin_coast(self, reason, now):
        if self.phase not in ('coast', 'locked') and _number(now) and now >= self.last_now:
            self.coast_since = now
        super().begin_coast(reason, now)

    def _exit_width_change_limit(self, previous_width):
        # Operator permits moderate changes in the live exit-width estimate.
        # Trial21's same-wall pair changed 8.5%, just above the old 15 cm gate.
        # Scale with the previous accepted observation, not a course dimension;
        # unique actual wall association is still required before this check.
        return max(super()._exit_width_change_limit(previous_width),
                   MAX_EXIT_WIDTH_CHANGE_FRACTION*previous_width)

    def _continue_with_tracked_width(self):
        # The operator confirms the exit remains present when the soft board
        # moves. An abrupt width estimate alone must not center an active turn.
        # Reuse the existing current-wall-only path construction with the last
        # accepted observed width; never invent a new wall or renew a missing
        # wall. No new-width target is adopted. Current body gates stay outside.
        return self.phase == 'drive'

    def _slew(self, now):
        if now-self.last_change+1e-9 >= PWM_INTERVAL_S and self.servo != self.steering_target:
            target = self.steering_target
            reducing_left = self.servo > NEUTRAL and target < self.servo
            second_orbit = (self.continue_route and self.phase == 'drive'
                            and self.route_stage == 'second_orbit')
            step = PRESTEER_PWM_STEP if self.phase == 'presteer' else SECOND_ORBIT_PWM_STEP if second_orbit else (
                (ROUTE_EXIT_RELEASE_PWM_STEP if self._route_exit_release_active() else LEFT_RELEASE_PWM_STEP)
                if reducing_left else PWM_STEP)
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
        if self.phase == 'drive' and self.adopted_presteer_pwm is None:
            self.adopted_presteer_pwm = self.servo
        result = super()._result(now)
        stage = ('orbit_entry' if self.handover_observed and self.phase == 'drive'
                 else 'left_turn' if self.phase == 'drive' else result['turn_stage'])
        if self.phase == 'drive' and self.first_pass_preparing:
            stage = 'right_exit' if self.first_pass_evidence is not None else 'first_pass_prepare'
        result.update(turn_stage=stage, trial_scope='first_lidar_compact_target_orbit_entry',
                      compact_target=copy.deepcopy(self.compact_target),
                      compact_target_reason=getattr(self.target_tracker, 'reason', None),
                      compact_loss_evidence=copy.deepcopy(self.compact_loss_evidence),
                      first_pass_preparing=self.first_pass_preparing,
                      first_pass_progress=copy.deepcopy(self.first_pass_progress),
                      first_exit_prepare_evidence=copy.deepcopy(self.first_exit_prepare_evidence),
                      first_pass_evidence=copy.deepcopy(self.first_pass_evidence),
                      right_exit_center_ack=copy.deepcopy(self.right_exit_center_ack),
                      right_exit_geometry=copy.deepcopy(self.right_exit_geometry),
                      right_exit_min_pwm=RIGHT_EXIT_MIN_PWM,
                      right_exit_elapsed_s=0 if self.right_exit_since is None else max(0, now-self.right_exit_since),
                      right_exit_max_s=RIGHT_EXIT_MAX_S,
                      second_target_handover_observed=self.second_target_handover_observed,
                      full_two_cone_s_supported=False,
                      handover_observed=self.handover_observed,
                      orbit_reference_range_m=self.orbit_reference_range_m,
                      orbit_bias_pwm=self.orbit_bias_pwm,
                      orbit_feedback=copy.deepcopy(self.orbit_feedback),
                      orbit_left_entry_boost=self.orbit_left_entry_boost,
                      left_turn_feedback=copy.deepcopy(self.left_turn_feedback),
                      handover_control=copy.deepcopy(self.handover_control),
                      orbit_entry_elapsed_s=0 if self.orbit_since is None else max(0, now-self.orbit_since),
                      orbit_entry_max_s=ORBIT_ENTRY_MAX_S,
                      presteer_max_s=self.max_presteer_s,
                      left_turn_servo_cap=SERVO_MAX,
                      steering_step_policy={'presteer': PRESTEER_PWM_STEP,
                          'reduce_left': LEFT_RELEASE_PWM_STEP,
                          'route_exit_reduce_left': ROUTE_EXIT_RELEASE_PWM_STEP if self.continue_route else None,
                          'second_orbit': SECOND_ORBIT_PWM_STEP if self.continue_route else None,
                          'increase_left': PWM_STEP,
                          'center_to_right': PWM_STEP, 'interval_s': PWM_INTERVAL_S},
                      object_semantic_verified=False, passed_cones=None,
                      competition_supported=False, completed=False,
                      test_sequence_finished=False,
                      inner_clearance=copy.deepcopy(self.inner_clearance),
                      entry_bearing=copy.deepcopy(self.entry_bearing),
                      entry_bearing_required=self.entry_bearing_required,
                      entry_bearing_released=self.entry_bearing_released,
                      adopted_presteer_pwm=self.adopted_presteer_pwm,
                      quality_coast_servo=self.quality_coast_servo,
                      quality_wait_refusal=copy.deepcopy(self.quality_wait_refusal),
                      quality_coast_reason=self.quality_coast_reason,
                      quality_coast_hold_active=(self.phase == 'coast'
                                                and self.quality_coast_servo is not None),
                      wall_ambiguity_hold=self.wall_ambiguity_hold)
        result.update(continue_route=self.continue_route, route_stage=self.route_stage,
            quality_wait_active=self.phase == 'quality_wait',
            quality_wait_servo=self.quality_wait_servo,
            quality_wait_used=self.quality_wait_used,
            quality_wait_evidence=copy.deepcopy(self.quality_wait_evidence),
            quality_resume_pending=self.quality_resume_pending,
            quality_wait_release=copy.deepcopy(self.quality_wait_release),
            quality_wait_previous_servo=self.quality_wait_previous_servo,
            route_alignment_evidence=copy.deepcopy(self.route_alignment_evidence),
            second_target=copy.deepcopy(self.second_target),
            second_center_ack=copy.deepcopy(self.second_center_ack),
            second_pass_evidence=copy.deepcopy(self.second_pass_evidence),
            second_feedback=copy.deepcopy(self.second_feedback),
            route_end_coast_servo=self.route_end_coast_servo,
            second_acquisition_reason=self.second_acquisition_reason,
            role_distinct_evidence=copy.deepcopy(self.role_distinct_evidence),
            route_events=copy.deepcopy(self.route_events),
            two_target_observed_pass_complete=self.second_pass_evidence is not None,
            route_right_output_authorized=(self.continue_route and self.phase == 'drive'
                and not self.quality_resume_pending and self.first_pass_evidence is not None
                and self._route_center_ack is not None
                and ((self.route_stage in ('right_align', 'corridor_follow')
                      and self.right_exit_center_ack is not None)
                     or (self.route_stage == 'second_orbit' and self.second_center_ack is not None))),
            route_center_ack=copy.deepcopy(self._route_center_ack))
        if self.continue_route and self.phase == 'drive' and self.route_stage != 'first_target':
            result['turn_stage'] = self.route_stage
        if self.continue_route:
            result['trial_scope'] = 'opt_in_observed_two_target_route'
        return result

    def _usable_target(self, target, scan):
        if self.entry_bearing_required and not self.entry_bearing_released:
            return False
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

    def _capture_compact_loss(self, scan, now, control, reason):
        # A one-shot in-memory snapshot from the actual control decision.
        # HTTP observers can skip the failing frame between successive reads.
        # Later scans must not overwrite this evidence. No disk IO or new
        # actuator behavior is introduced into the control tick.
        if self.compact_loss_evidence is None:
            self.compact_loss_evidence = {
                'decision_at': now, 'reason': reason,
                'tracker_reason': getattr(self.target_tracker, 'reason', None),
                'tracker_error': self.compact_target_error,
                'scan': copy.deepcopy({k: scan.get(k) for k in
                    ('frame_id', 'seq', 'at_ms', 'received_at', 'ranges')}),
                'previous_confirmed_target': copy.deepcopy(self._last_confirmed_compact_target),
                'current_target': copy.deepcopy(self.compact_target),
                'control': copy.deepcopy({k: control.get(k) for k in
                    ('armed', 'motor', 'servo', 'tick', 'seq', 'command_acked')}),
                'physical_identity_verified': False}

    def _observe_first_pass(self, scan, now, control):
        """Prepare command reversal from current target/body geometry.

        Relative point motion is an observation trend, NOT vehicle speed or
        odometry. The lead covers commanded PWM steps only, not measured servo
        lag. Partial boundary observations cannot establish this new phase.
        """
        target = self.compact_target
        bins = target.get('support_bins') if isinstance(target, dict) else None
        if self.continue_route and (self._full_target_points(target, scan, self.orbit_track_id) is None
                or not _number(scan.get('received_at'))
                or not 0 <= now-scan['received_at'] < SCAN_AGE_S):
            self._clear_exit_prepare_window()
            self._first_pass_history.clear()
            return
        if (target is None or target.get('track_id') != self.orbit_track_id
                or not target.get('confirmed') or target.get('tracking_only_boundary_gap') is not False
                or not isinstance(bins, list) or len(bins) < 5
                or any(type(i) is not int or not 0 <= i < 360 for i in bins)):
            self._first_pass_history = []
            self._clear_exit_prepare_window()
            return
        ranges = scan['ranges']
        if any(not _number(ranges[i]) for i in bins):
            self._first_pass_history = []
            self._clear_exit_prepare_window()
            return
        points = [(ranges[i]*math.cos(math.radians(i)), -ranges[i]*math.sin(math.radians(i))) for i in bins]
        # Always derive extents from current raw support, never a cone radius.
        front = max(p[0] for p in points)
        side = min(p[1] for p in points)-SIDE_BODY_EXTENT_M
        x = sum(p[0] for p in points)/len(points)
        row = {'source_seq': scan['seq'], 'source_at_ms': scan['at_ms'],
               'received_at': scan['received_at'], 'center_x_m': x}
        history = self._first_pass_history
        if history and (scan['seq'] <= history[-1]['source_seq']
                        or scan['at_ms'] <= history[-1]['source_at_ms']
                        or scan['received_at'] <= history[-1]['received_at']):
            return
        history.append(row)
        self._first_pass_history = history = history[-6:]
        pub_span = (row['source_at_ms']-history[0]['source_at_ms'])/1000
        receive_span = row['received_at']-history[0]['received_at']
        approaching = (len(history) >= 3 and pub_span >= .25 and receive_span >= .25
                       and all(b['center_x_m'] < a['center_x_m'] for a, b in zip(history, history[1:])))
        adopted = (control.get('command_acked') is True and control.get('motor') == TRIAL_MOTOR
                   and control.get('servo') == self.servo and NEUTRAL <= self.servo <= SERVO_MAX)
        lead = (math.ceil((self.servo-NEUTRAL)/LEFT_RELEASE_PWM_STEP)+1)*PWM_INTERVAL_S
        closing = (min((history[0]['center_x_m']-x)/pub_span,
                       (history[0]['center_x_m']-x)/receive_span) if approaching else None)
        prediction_prepare = (adopted and approaching and x <= FRONT_BODY_EXTENT_M
                   and side >= MANEUVER_BODY_CLEARANCE_M
                   and front+REAR_BODY_EXTENT_M <= closing*lead)
        # Ending the continuing-left demand is distinct from observing a pass.
        # Sustained orbit geometry can make relative x motion approach zero,
        # so the predictive rear-crossing test alone can wait indefinitely.
        # This opt-in gate only releases toward center; rear passage and the
        # current-corridor / center-ACK conditions still own all right steering.
        geometric_ready = False
        eligible = (self.continue_route and self.handover_observed is True
                    and self.phase == 'drive' and self.route_stage == 'first_target'
                    and front <= FRONT_BODY_EXTENT_M
                    and side >= MANEUVER_BODY_CLEARANCE_M)
        if eligible:
            # Trial37..39: three frames over .25 s plus the 20 PWM release let the
            # orbit continue far past the target. Two strictly fresh current
            # full-support frames are required now; still release-only.
            geometric_ready = self._consecutive_fresh(self._first_exit_prepare_history, scan)
        else:
            self._clear_exit_prepare_window()
        geometric_prepare = geometric_ready and adopted
        if geometric_prepare and not self.first_pass_preparing:
            self._latch_exit_prepare(scan, control,
                basis='current_first_target_complete_support_behind_body_front')
        prepare = prediction_prepare or geometric_prepare
        self.first_pass_progress = {**row, 'observed_frontmost_x_m': front,
            'observed_left_body_gap_m': side, 'command_center_lead_s': lead,
            'relative_x_closing_mps': closing, 'relative_trend_is_vehicle_speed': False,
            'prepare_condition': prepare, 'prediction_prepare_condition': prediction_prepare,
            'geometric_exit_prepare_condition': geometric_prepare,
            'body_rear_x_m': -REAR_BODY_EXTENT_M,
            'physical_steering_confirmed': False}
        if prepare:
            self.first_pass_preparing = True
            self.orbit_left_entry_boost = False
        if (self.first_pass_preparing and self.first_pass_evidence is None
                and front <= -REAR_BODY_EXTENT_M and side >= MANEUVER_BODY_CLEARANCE_M):
            self.first_pass_evidence = {**self.first_pass_progress,
                'track_id': target['track_id'], 'support_bins': list(bins),
                'basis': 'current_confirmed_target_observed_support_behind_body_rear',
                'semantic_class': 'unknown', 'physical_cone_pass_certified': False,
                'swept_path_certified': False}
            self.right_exit_since = now
            if self.continue_route:
                self.route_stage = 'right_align'
                self._route_event('first_pass', scan, control, self.first_pass_evidence)

    def _right_exit_target(self, scan, now):
        """Use a current native forward corridor, never a guessed second cone."""
        values = scan.get('corridor_candidates')
        if (not isinstance(values, list) or len(values) > 24
                or any(not _valid_candidate(c, lateral_margin=SIDE_BODY_EXTENT_M+MANEUVER_BODY_CLEARANCE_M)
                       for c in values)):
            return None, 'right_exit_invalid_corridor'
        forward = [c for c in values if -math.pi/2 < c['heading_left_rad'] < math.pi/2]
        if len(forward) > 1:
            return None, 'right_exit_corridor_ambiguous'
        if forward:
            c = {**forward[0], 'source_kind': 'native_corridor'}
        elif self.continue_route:
            # A nearby compact object or a wall that ends inside one side
            # sector can defeat the whole-sector fit while the same frame still
            # holds two parallel native wall windows (trials38/39).
            c, ambiguous = self._paired_window_corridor(scan)
            if ambiguous:
                return None, 'right_exit_corridor_ambiguous'
            if c is None or not _valid_candidate(c, lateral_margin=SIDE_BODY_EXTENT_M+MANEUVER_BODY_CLEARANCE_M):
                return None, 'right_exit_corridor_missing'
        else:
            return None, 'right_exit_corridor_missing'
        previous = self.right_exit_geometry
        if previous is not None:
            # Bound each new corridor against the preceding real observation.
            dt = (scan['at_ms']-previous['source_at_ms'])/1000
            receive_dt = scan['received_at']-previous['received_at']
            if (dt <= 0 or dt >= SCAN_AGE_S
                    or (self.continue_route and (not 0 < receive_dt < SCAN_AGE_S
                        or now-previous['received_at'] >= SCAN_AGE_S
                        or abs(dt-receive_dt) > .15))
                    or abs(c['heading_left_rad']-previous['heading_left_rad']) > math.radians(25)
                    or abs(c['center_offset_left_m']-previous['center_offset_left_m']) > .30
                    or abs(c['width_m']-previous['width_m']) > max(.15, .1*previous['width_m'])):
                return None, 'right_exit_corridor_identity_changed'
        lookahead = max(FRONT_BODY_EXTENT_M+REAR_BODY_EXTENT_M,
                        min(1., c['width_m']/2))
        error = (c['heading_left_rad'] if self.continue_route else
                 c['heading_left_rad']+math.atan2(c['center_offset_left_m'], lookahead))
        target = max(RIGHT_EXIT_MIN_PWM, min(NEUTRAL, NEUTRAL+round(LEFT_TRIAL_GAIN*error)))
        self.right_exit_geometry = {**copy.deepcopy(c), 'source_seq': scan['seq'],
            'source_at_ms': scan['at_ms'], 'received_at': scan['received_at'],
            'lookahead_m': lookahead, 'steering_target_pwm': target,
            'source': 'current_native_corridor', 'second_target_confirmed': False}
        self._right_exit_last_receive, self._right_exit_last_publication = scan['received_at'], scan['at_ms']
        return target, None

    @staticmethod
    def _corridor_body_gap(candidate):
        heading = candidate['heading_left_rad']
        projections = [-math.sin(heading)*x+math.cos(heading)*y
                       for x in (-REAR_BODY_EXTENT_M, FRONT_BODY_EXTENT_M)
                       for y in (-SIDE_BODY_EXTENT_M, SIDE_BODY_EXTENT_M)]
        offset, half = candidate['center_offset_left_m'], candidate['width_m']/2
        return min(half+offset-max(projections), min(projections)-(offset-half))

    def _route_adopted(self, control, now):
        if control.get('servo', NEUTRAL) > NEUTRAL:
            self._route_center_ack = None
            self.second_center_ack = None
            if self.servo < NEUTRAL:
                self.lock('route_actual_steering_crossed_center')
                return False
        adopted = (control.get('motor') == TRIAL_MOTOR and control.get('servo') == self.servo
                   and control.get('command_acked') is True)
        if not adopted:
            if self.right_exit_adoption_wait_since is None:
                self.right_exit_adoption_wait_since = now
            if now-self.right_exit_adoption_wait_since >= SCAN_AGE_S:
                self.begin_coast('route_adoption_unconfirmed', now)
            else:
                self.steering_target = self.servo
                self.reason = 'route_waiting_adopted_command'
            return False
        self.right_exit_adoption_wait_since = None
        if self.servo > NEUTRAL:
            self._route_center_ack = None
        elif self.servo == NEUTRAL:
            self._route_center_ack = {k: control.get(k) for k in ('servo', 'motor', 'tick', 'seq')}
        return True

    def _right_exit_update(self, scan, now, control):
        if now-self.right_exit_since+1e-9 >= RIGHT_EXIT_MAX_S:
            self.begin_coast('first_target_right_exit_trial_timeout', now)
            self._slew(now)
            return self._result(now)
        target, rejection = self._right_exit_target(scan, now)
        if target is None:
            # Missing geometry only retains an already selected command under
            # its original observation lease; it cannot advance a right turn.
            lease_receive, lease_publication = self._right_exit_last_receive, self._right_exit_last_publication
            if (lease_receive is None and self.continue_route and self.servo > NEUTRAL
                    and isinstance(self.first_pass_evidence, dict)):
                # Before any corridor: only the actual first-pass frame's own
                # lease, and only to keep releasing toward neutral.
                lease_receive = self.first_pass_evidence.get('received_at')
                lease_publication = self.first_pass_evidence.get('source_at_ms')
            held = (rejection == 'right_exit_corridor_missing'
                    and _number(lease_receive) and _number(lease_publication)
                    and now-lease_receive < SCAN_AGE_S
                    and (scan['at_ms']-lease_publication)/1000 < SCAN_AGE_S)
            if held and self.continue_route and self.servo > NEUTRAL:
                # Trial39 489: holding residual left while the corridor was
                # missing continued the orbit. Release toward neutral only;
                # missing geometry still never authorizes a right command.
                self.steering_target = NEUTRAL
                self.reason = 'right_exit_corridor_missing_release_left'
                self._slew(now)
            elif held:
                self.steering_target = self.servo
                self.reason = 'right_exit_corridor_missing_hold'
            else:
                self.begin_coast(rejection, now)
                self._slew(now)
            return self._result(now)
        if self.continue_route and not self._route_adopted(control, now):
            return self._result(now)
        adopted = (control.get('motor') == TRIAL_MOTOR and control.get('servo') == self.servo
                   and control.get('command_acked') is True)
        if not adopted:
            if self.right_exit_adoption_wait_since is None:
                self.right_exit_adoption_wait_since = now
            if now-self.right_exit_adoption_wait_since+1e-9 >= SCAN_AGE_S:
                self.begin_coast('right_exit_adoption_unconfirmed', now)
                self._slew(now)
            else:
                self.steering_target = self.servo
                self.reason = 'right_exit_waiting_adopted_command'
            return self._result(now)
        self.right_exit_adoption_wait_since = None
        if self.right_exit_center_ack is None:
            if (self.servo == NEUTRAL and control.get('servo') == NEUTRAL
                    and control.get('motor') == TRIAL_MOTOR and control.get('command_acked') is True):
                self.right_exit_center_ack = {k: control.get(k) for k in ('servo', 'motor', 'tick', 'seq')}
                self._route_center_ack = dict(self.right_exit_center_ack)
            else:
                target = NEUTRAL
        if self.continue_route and self.right_exit_center_ack is not None:
            c = self.right_exit_geometry
            aligned = (abs(c['heading_left_rad']) <= math.radians(5)
                       and self._corridor_body_gap(c) >= MANEUVER_BODY_CLEARANCE_M)
            if aligned and self._mature_observation(self._route_alignment_history, scan, .25):
                self.route_alignment_evidence = {**copy.deepcopy(c),
                    'body_normal_gap_m': self._corridor_body_gap(c),
                    'physical_alignment_certified': False}
                self.route_stage = 'corridor_follow'
                self._route_event('corridor_follow', scan, control, self.route_alignment_evidence)
                self.steering_target = self.servo
                self.reason = 'route_corridor_alignment_observed'
                return self._result(now)
            if not aligned:
                self._route_alignment_history.clear()
        self.steering_target = self.natural_steering_target = target
        self.reason = ('first_target_right_exit_tracking' if self.right_exit_center_ack is not None
                       else 'first_target_right_exit_waiting_center_ack')
        self._slew(now)
        return self._result(now)

    def _second_role_observation(self, scan, now):
        """One distinct compact support ahead inside the current corridor.

        Trial43: the second target appeared at the left front while the first
        was already behind; the old right-half-only, first-still-visible proof
        never matched. The first support has a recorded pass behind the rear
        and the car follows a forward corridor, so a support ahead of the body
        front inside that corridor is a different object. If the first support
        is still fully visible it must also share no bins and be separated.
        """
        c = self.right_exit_geometry
        proof = None
        if self.first_pass_evidence is not None and isinstance(c, dict):
            first = self._full_target_points(self.compact_target, scan, self.orbit_track_id)
            heading = c['heading_left_rad']
            bound = c['width_m']/2-MANEUVER_BODY_CLEARANCE_M
            ahead = []
            for target in _candidates(scan['ranges'], bearing_range_rad=(-math.pi/2, math.pi/2)):
                x, y = target['point_left_m']
                along = math.cos(heading)*x+math.sin(heading)*y
                normal = -math.sin(heading)*x+math.cos(heading)*y
                if (x > FRONT_BODY_EXTENT_M and along > FRONT_BODY_EXTENT_M
                        and abs(normal-c['center_offset_left_m']) < bound):
                    ahead.append((target, along, normal))
            if len(ahead) == 1:
                target, along, normal = ahead[0]
                bins = target['support_bins']
                points = [(scan['ranges'][i]*math.cos(math.radians(i)),
                           -scan['ranges'][i]*math.sin(math.radians(i))) for i in bins]
                separation = (None if first is None else
                              min(math.dist(a, b) for a in first for b in points))
                distinct = first is None or (
                    not set(bins).intersection(self.compact_target['support_bins'])
                    and separation > MAX_DIAMETER_M)
                if distinct:
                    proof = {'source_seq': scan['seq'], 'source_at_ms': scan['at_ms'],
                        'source_received_at': scan['received_at'], 'first_track_id': self.orbit_track_id,
                        'first_support_visible': first is not None,
                        'first_support_bins': (list(self.compact_target['support_bins'])
                                               if first is not None else None),
                        'first_pass_source_seq': self.first_pass_evidence.get('source_seq'),
                        'second_support_bins': list(bins), 'actual_support_separation_m': separation,
                        'corridor_heading_left_rad': heading, 'second_along_m': along,
                        'second_normal_m': normal, 'second_side': 'left' if target['point_left_m'][1] > 0 else 'right',
                        'basis': 'single_support_ahead_inside_current_corridor_after_first_pass',
                        'physical_identity_verified': False}
        if proof is None:
            # acquire=False alone would still mature an existing unconfirmed
            # association. No role proof means discard that entire interval.
            self.second_tracker = self._new_second_tracker()
            self.second_target = None
            self.role_distinct_evidence = None
            self.second_acquisition_reason = 'second_role_current_distinct_support_missing'
            return
        self.second_target = self.second_tracker.update(scan, now, acquire=True)
        self.role_distinct_evidence = proof
        self.second_acquisition_reason = self.second_tracker.reason

    def _corridor_follow_update(self, scan, now, control):
        _, rejection = self._right_exit_target(scan, now)
        if rejection is not None:
            self.begin_coast(rejection, now)
            self._slew(now)
            return self._result(now)
        if self._corridor_body_gap(self.right_exit_geometry) < MANEUVER_BODY_CLEARANCE_M:
            self.begin_coast('route_corridor_body_margin_unavailable', now)
            return self._result(now)
        if not self._route_adopted(control, now):
            return self._result(now)
        self._second_role_observation(scan, now)
        if self.second_target is not None and self.second_target.get('confirmed') is True:
            self.second_track_id = self.second_target['track_id']
            self.second_target_handover_observed = True
            self.second_orbit_since, self.route_stage = now, 'second_orbit'
            # An uninterrupted right command may retain its actual center ACK.
            # A new left-to-right change must acquire a new center ACK below.
            if self.servo <= NEUTRAL and self._route_center_ack is not None:
                self.second_center_ack = dict(self._route_center_ack)
            self.steering_target = self.servo
            self.reason = 'second_target_handover_keep_adopted_command'
            self._route_event('second_handover', scan, control, self.role_distinct_evidence)
            return self._result(now)
        c = self.right_exit_geometry
        error = c['heading_left_rad']+math.atan2(c['center_offset_left_m'], c['lookahead_m'])
        target = max(NEUTRAL-55, min(NEUTRAL+55, NEUTRAL+round(LEFT_TRIAL_GAIN*error)))
        if target < NEUTRAL and self._route_center_ack is None:
            target = NEUTRAL
        self.steering_target = self.natural_steering_target = target
        self.reason = 'route_corridor_follow_current_geometry'
        self._slew(now)
        if self.servo > NEUTRAL:
            self._route_center_ack = None
        return self._result(now)

    def _second_orbit_update(self, scan, now, control):
        if now-self.second_orbit_since >= ORBIT_ENTRY_MAX_S:
            self.begin_coast('second_target_entry_trial_timeout', now)
            self._slew(now)
            return self._result(now)
        target = self.second_target
        if target is None or target.get('track_id') != self.second_track_id or not target.get('confirmed'):
            self.begin_coast('second_target_lost_or_identity_changed', now)
            self._slew(now)
            return self._result(now)
        if not self._route_adopted(control, now):
            return self._result(now)
        if self.second_center_ack is None and self.servo == NEUTRAL:
            self.second_center_ack = {k: control.get(k) for k in ('servo', 'motor', 'tick', 'seq')}
        points = self._full_target_points(target, scan, self.second_track_id)
        passed = (points is not None and max(p[0] for p in points) <= -REAR_BODY_EXTENT_M
                  and min(-p[1] for p in points)-SIDE_BODY_EXTENT_M >= MANEUVER_BODY_CLEARANCE_M)
        if passed and self._mature_observation(self._second_pass_history, scan, .25):
            self.second_pass_evidence = {'source_seq': scan['seq'], 'source_at_ms': scan['at_ms'],
                'received_at': scan['received_at'], 'track_id': self.second_track_id,
                'support_bins': list(target['support_bins']),
                'observed_frontmost_x_m': max(p[0] for p in points),
                'observed_right_body_gap_m': min(-p[1] for p in points)-SIDE_BODY_EXTENT_M,
                'basis': 'current_complete_support_behind_rear',
                'physical_cone_pass_certified': False, 'swept_path_certified': False}
            self._route_event('second_pass', scan, control, self.second_pass_evidence)
            self.begin_coast('two_target_observed_support_pass', now)
            if (self.phase == 'coast' and control.get('servo') == self.servo <= NEUTRAL
                    and control.get('command_acked') is True):
                # Trial43 rolled straight into the end board at ~1.8 m/s: keep
                # the ACKed clockwise (right/center) command, never recenter.
                self.route_end_coast_servo = self.servo
                self.steering_target = self.servo
            self._slew(now)
            return self._result(now)
        if not passed:
            self._second_pass_history.clear()
        if points is None:
            # Partial boundaries maintain only the already selected command;
            # they cannot define a new bypass point or prove a pass.
            self.steering_target = self.servo
            self.reason = 'second_target_partial_support_hold'
            return self._result(now)
        body_length = FRONT_BODY_EXTENT_M+REAR_BODY_EXTENT_M
        front = max(p[0] for p in points)
        near = min(p[0] for p in points)
        left_edge = max(p[1] for p in points)
        # Reach the bypass line at the support's near face, not beyond its far
        # face: a pursuit arc to a point past the support cuts inside it. The
        # receding point then keeps that gap while the support is abeam.
        waypoint = [max(body_length, near), left_edge+SECOND_BYPASS_SUPPORT_GAP_M]
        bearing = math.atan2(waypoint[1], waypoint[0])
        # Trial43: the bypass point left of a left-front support needs left;
        # right still requires a current actual center ACK after any left.
        nominal = max(RIGHT_EXIT_MIN_PWM, min(SERVO_MAX,
            NEUTRAL+round(LEFT_TRIAL_GAIN*bearing)))
        self.second_feedback = {'source_seq': scan['seq'], 'source_at_ms': scan['at_ms'],
            'current_support_left_edge_y_m': left_edge, 'current_support_frontmost_x_m': front,
            'current_support_nearest_x_m': near, 'bypass_support_gap_m': SECOND_BYPASS_SUPPORT_GAP_M,
            'relative_bypass_point_left_m': waypoint, 'bearing_left_rad': bearing,
            'nominal_target_pwm': nominal,
            'scope': 'current_support_relative_point_trial',
            'swept_path_certified': False, 'physical_curvature_calibrated': False}
        self.natural_steering_target = nominal
        if nominal < NEUTRAL and (self.second_center_ack is None or self.servo > NEUTRAL):
            self.steering_target = NEUTRAL
            self.reason = 'second_target_waiting_center_ack'
        else:
            self.steering_target = nominal
            self.reason = ('second_target_left_of_support_entry' if nominal > NEUTRAL
                           else 'second_target_clockwise_bounded_entry')
        self._slew(now)
        return self._result(now)

    def _relative_target_pwm(self, target):
        # This is the bounded entry stage, not a settled circular orbit. In
        # trial18 the object was still ahead of the lateral axis and approaching;
        # two negative terms erased the adopted left demand immediately after
        # handover. Trial19 additionally inherited an already released 1538.
        # Continue from the configured left reference, rather than treating
        # the wall controller's final release as an orbit feedforward value.
        # The first handover output still holds the current adopted command;
        # subsequent increases obey the normal slew. A target moving behind
        # or farther away adds left correction; current inside-edge clearance
        # can still independently release steering in the caller.
        # Gains are trial PWM feedback, not physical curvature or a course radius.
        reference = self.orbit_reference_range_m
        bearing_term = ORBIT_BEARING_GAIN*(target['bearing_left_rad']-ORBIT_BEARING_RAD)
        range_term = ORBIT_RANGE_GAIN*(target['range_m']-reference)/reference
        correction = max(0., bearing_term) + max(0., range_term)
        bias = self.orbit_bias_pwm if self.orbit_bias_pwm is not None else self.initial_presteer_pwm
        entry_base = max(bias, self.initial_presteer_pwm)
        requested = max(NEUTRAL, min(SERVO_MAX, round(entry_base+correction)))
        # Trial26's accepted initial turn handed over with the real object
        # still left/front. Both old correction terms stayed negative, so the
        # command stopped increasing at1670 and the operator hit the right
        # wall. For this bounded entry only, continue requesting the existing
        # left limit while that same confirmed object has not reached abeam.
        # Actual commands still increase at10/100ms, and current inside points
        # may release left independently. This is not a calibrated orbit law.
        if self.orbit_left_entry_boost:
            if target['bearing_left_rad'] >= ORBIT_BEARING_RAD:
                if self._orbit_abeam_count == 0:
                    self._orbit_abeam_publication = target['source_at_ms']
                    self._orbit_abeam_receive = self.last_now
                self._orbit_abeam_count += 1
                if (self._orbit_abeam_count >= 3
                        and (target['source_at_ms']-self._orbit_abeam_publication)/1000 >= .25
                        and self.last_now-self._orbit_abeam_receive >= .25):
                    self.orbit_left_entry_boost = False
            else:
                self._orbit_abeam_count = 0
                self._orbit_abeam_publication = self._orbit_abeam_receive = None
            if self.orbit_left_entry_boost:
                requested = SERVO_MAX
        self.orbit_feedback = {'source_seq': target.get('source_seq'),
            'source_at_ms': target.get('source_at_ms'), 'scope': 'bounded_orbit_entry_only',
            'adopted_handover_bias_pwm': bias, 'prepared_left_pwm': self.initial_presteer_pwm,
            'entry_base_pwm': entry_base, 'bearing_term_pwm': bearing_term,
            'range_term_pwm': range_term, 'applied_entry_correction_pwm': correction,
            'left_entry_boost_active': self.orbit_left_entry_boost,
            'abeam_confirmations': self._orbit_abeam_count,
            'boost_scope': 'first_target_ahead_at_handover_until_current_abeam_evidence',
            'nominal_target_pwm': requested, 'inside_edge_release_remains_independent': True,
            'physical_curvature_calibrated': False}
        return requested

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
        # center before the continuation rule below. These are trial gains,
        # not a physical curvature calibration.
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
        # Current measured turn error may request the full 1720 bound. The
        # adopted preparation value also anchors the handover phase below;
        # _slew retains the phase/direction step limit per 100 ms.
        target = max(NEUTRAL, min(SERVO_MAX, NEUTRAL+round(LEFT_TRIAL_GAIN*damped)))
        alignment_target = target
        opening_drive = self.phase == 'drive' and self.geometry_mode == 'opening_wall'
        transition_trigger = (opening_drive and heading > 0
                              and self.turn_goal['target_point_left_m']['y_m'] <= 0)
        if transition_trigger:
            self.left_handover_preparing = True
        continuing_left = (opening_drive and heading > 0
                           and (self.left_handover_preparing or self.entry_bearing_released))
        if continuing_left:
            # Opening-wall alignment is not a request to straighten before the
            # next left orbit. Trial19's projected target moved to the right
            # while exit heading was still left, driving its command near1500.
            # The current endpoint-bearing gate can also establish this demand
            # after the early approach. A small projection sign change cannot
            # release it again. Neither condition proves the endpoint is passed.
            # Current geometry, original leases and all hard gates still apply.
            # Initial PWM is the configured continuation reference; the entry
            # gate may have lowered actual neutral preparation, recorded as
            # adopted_presteer_pwm. All increases retain the normal slew.
            target = max(target, self.initial_presteer_pwm)
        self.left_turn_feedback = {
            'source_seq': self._current_scan.get('seq') if self._current_scan else None,
            'source_at_ms': self._current_scan.get('at_ms') if self._current_scan else None,
            'geometry_mode': self.geometry_mode,
            'opening_left_continuation_active': continuing_left,
            'handover_preparation_trigger_current': transition_trigger,
            'alignment_target_pwm': alignment_target,
            'prepared_left_pwm': self.initial_presteer_pwm,
            'nominal_target_pwm': target,
            'inside_edge_release_remains_independent': True,
            'physical_curvature_calibrated': False}
        if self.phase == 'drive':
            target = self._limit_left_for_known_points(target, lookahead)
        target = self._entry_target(target)
        self.left_turn_feedback['endpoint_limited_target_pwm'] = target
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

    def _orbit_update(self, scan, lidar_age_s, now, control, *, safe, presteer_wait,
                      quality_clear=False, quality_resume_ready=False):
        advancing = self._orbit_inputs(scan, lidar_age_s, now, control, safe, presteer_wait)
        if self.phase == 'locked':
            self._clear_exit_prepare_window()
            return self._result(self.last_now)
        if self.phase == 'quality_wait':
            return self._quality_wait_update(scan, now, control, advancing,
                                             quality_clear, quality_resume_ready)
        if self.phase == 'drive' and now-self.drive_since+1e-9 >= self.max_drive_s:
            self.begin_coast('maneuver_cumulative_drive_timeout', now)
        if self.phase == 'coast':
            if self.coast_since is not None and now-self.coast_since+1e-9 >= COAST_MAX_S:
                self.lock('coast_standstill_unconfirmed')
            else:
                self.steering_target = (self.quality_coast_servo if self.quality_coast_servo is not None
                                        else self.route_end_coast_servo if self.route_end_coast_servo is not None
                                        else NEUTRAL)
                self._slew(now)
            return self._result(now)
        if self.quality_resume_pending:
            points = self._full_target_points(self.compact_target, scan, self.orbit_track_id)
            if (quality_clear is not True or points is None
                    or max(p[0] for p in points) > FRONT_BODY_EXTENT_M
                    or min(p[1] for p in points)-SIDE_BODY_EXTENT_M < MANEUVER_BODY_CLEARANCE_M):
                self._clear_exit_prepare_window()
            if self.first_pass_evidence is not None and advancing:
                _, rejection = self._right_exit_target(scan, now)
                if rejection is not None:
                    self.lock('route_quality_resume_corridor_unconfirmed')
                    return self._result(now)
            if points is None:
                self.lock('route_quality_resume_target_unconfirmed')
            elif (self.first_pass_evidence is None and now-self.orbit_since >= ORBIT_ENTRY_MAX_S):
                self.lock('route_quality_resume_original_budget_expired')
            elif (control.get('servo') != self.quality_wait_servo
                    or control.get('motor') not in (NEUTRAL, TRIAL_MOTOR)):
                self.lock('route_quality_resume_feedback_changed')
            elif (control.get('resume_acked') is True and control['motor'] == TRIAL_MOTOR
                  and control.get('command_acked') is True):
                self.quality_resume_pending = False
                self.quality_wait_evidence['resume_ack'] = {k: control.get(k) for k in
                    ('motor', 'servo', 'seq', 'tick')}
                if self._quality_exit_prepare_ready and self.first_pass_evidence is None:
                    self._latch_exit_prepare(scan, control,
                        basis='healthy_neutral_wait_body_front_support_rechecked_at_resume_ack')
                    self._quality_exit_prepare_ready = False
                if self.quality_wait_release is not None and self.first_pass_evidence is None:
                    # The wait already released left; resumed power must not
                    # restart the orbit demand. Release-only, no right PWM.
                    if not self.first_pass_preparing:
                        self._latch_exit_prepare(scan, control,
                            basis='neutral_wait_release_continued_at_resume_ack',
                            history=self._wait_release_history or [(scan['seq'], scan['at_ms'],
                                                                    scan['received_at'])])
                    self.first_pass_preparing = True
                    self.orbit_left_entry_boost = False
            elif now-self.quality_resume_since >= SCAN_AGE_S:
                self.lock('route_quality_resume_ack_timeout')
            else:
                self.steering_target = self.quality_wait_servo
            if self.phase == 'locked':
                self._clear_exit_prepare_window()
            # Even the ACK observation cannot change steering on this output.
            return self._result(now)
        if not advancing:
            # A repeated fresh observation cannot establish preparation or a
            # pass. It may execute ONE already-authorized release step once
            # the previous actual command is ACKed and its 100 ms is due.
            if (self.continue_route and self.phase == 'drive'
                    and self.handover_observed and self.route_stage == 'first_target'
                    and self.first_pass_preparing and self.first_pass_evidence is None
                    and NEUTRAL <= self.servo <= SERVO_MAX
                    and self._full_target_points(self.compact_target, scan, self.orbit_track_id) is not None
                    and 0 <= now-scan['received_at'] < SCAN_AGE_S
                    and control.get('command_acked') is True
                    and control.get('motor') == TRIAL_MOTOR and control.get('servo') == self.servo):
                if now-self.orbit_since+1e-9 >= ORBIT_ENTRY_MAX_S:
                    self.begin_coast('first_relative_object_entry_trial_timeout', now)
                else:
                    self.steering_target = self.natural_steering_target = NEUTRAL
                    self.reason = 'first_target_pass_preparing_neutral'
                self._slew(now)
            return self._result(now)
        if self.continue_route and self.route_stage == 'corridor_follow':
            return self._corridor_follow_update(scan, now, control)
        if self.continue_route and self.route_stage == 'second_orbit':
            return self._second_orbit_update(scan, now, control)
        if self.first_pass_evidence is not None:
            return self._right_exit_update(scan, now, control)
        if not self._usable_target(self.compact_target, scan):
            self._clear_exit_prepare_window()
            self._capture_compact_loss(scan, now, control, 'compact_target_lost_or_ambiguous')
            # Losing object identity removes the drive request, not the
            # already adopted left steering. Keep that command only in the
            # existing bounded neutral coast; never reacquire powered motion.
            if not self._begin_adopted_left_coast('compact_target_lost_or_ambiguous', now, control):
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
            self.orbit_left_entry_boost = self.compact_target['bearing_left_rad'] < ORBIT_BEARING_RAD
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
            self._clear_exit_prepare_window()
            self._capture_compact_loss(scan, now, control, 'compact_target_identity_changed')
            if not self._begin_adopted_left_coast('compact_target_identity_changed', now, control):
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
        self._observe_first_pass(scan, now, control)
        if self.first_pass_evidence is not None:
            return self._right_exit_update(scan, now, control)
        if self.first_pass_preparing:
            self.steering_target = self.natural_steering_target = NEUTRAL
            self.reason = 'first_target_pass_preparing_neutral'
            self._slew(now)
            return self._result(now)
        self.steering_target = self._limit_left_for_known_points(
            self._relative_target_pwm(self.compact_target),
            max(.35, min(1., self.compact_target['range_m'])))
        self.natural_steering_target = self.steering_target
        self.reason = 'first_relative_object_left_orbit_entry'
        self._slew(now)
        return self._result(now)

    def update(self, scan, lidar_age_s, now, control, *, safe=True, entry_stop=None, presteer_wait=False,
               quality_clear=False, quality_resume_ready=False):
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
        if self.compact_target is not None and self.compact_target.get('confirmed') is True:
            self._last_confirmed_compact_target = self.compact_target
        self.compact_target_error = None
        try:
            self.compact_target = (self.target_tracker.update(scan, now, acquire=self.first_pass_evidence is None)
                                   if self.continue_route else self.target_tracker.update(scan, now))
            if self.continue_route and self.route_stage == 'second_orbit':
                self.second_target = self.second_tracker.update(scan, now, acquire=False)
        except (ValueError, TypeError, KeyError) as error:
            self.compact_target = None
            self.compact_target_error = type(error).__name__
        if entry_stop is not None:
            # The new trial is target tracking, not the old optional stop input.
            self.lock('unsupported_maneuver_entry_stop')
            return self._result(self.last_now)
        # A held quality coast (route mode may start one in the left turn)
        # only validates advancing scans/control and keeps its ACKed servo;
        # no geometry association can recenter it while still rolling.
        if (self.handover_observed
                or (self.phase == 'coast' and (self.wall_ambiguity_hold
                                               or self.quality_coast_servo is not None))
                or (self.phase == 'drive' and self._usable_target(self.compact_target, scan))):
            return self._orbit_update(scan, lidar_age_s, now, control,
                                      safe=safe, presteer_wait=presteer_wait,
                                      quality_clear=quality_clear, quality_resume_ready=quality_resume_ready)
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
