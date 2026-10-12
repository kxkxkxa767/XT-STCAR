"""Bounded left turn and opt-in observed corridor / second-object continuation.

The compact object's semantic identity is unknown. This module observes no
global pose, speed, physical steering angle, passed cone count or complete lap.
The service retains fresh body-clearance checks, ownership, heartbeat and ACKs.
"""
import copy
import math

from compact_target import (CompactTargetTracker, _associated, _candidates, contact_supports, MAX_DIAMETER_M,
                            MIN_RANGE_M, TOLERANT_MIN_POINTS, TOLERANT_MAINTENANCE_MIN_POINTS)
from autonomy_live import (FRONT_BODY_EXTENT_M, REAR_BODY_EXTENT_M, SIDE_BODY_EXTENT_M,
                           MANEUVER_BODY_CLEARANCE_M, MIN_FORWARD_PWM)
from turn_motion import (TurnMotion, NEUTRAL, TRIAL_MOTOR, SERVO_MIN, SERVO_MAX, SCAN_AGE_S, MAX_DRIVE_S,
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
# Explicit continue_route only. After the first-target pass every route
# steering change (exit alignment, corridor follow, second target) uses the
# route exit release step already exercised on the car in trial43. Trial43
# needed a left swing around the second support within about one second;
# trial45 needed a quick left away from a right wall .22 m from the body.
# Center crossing and the right-after-center-ACK rule are unchanged.
ROUTE_POST_PASS_PWM_STEP = 40
# Explicit continue_route only: after the first-target handover, LEFT increases
# also use 40 PWM/100 ms. Trial49 (path rebuilt from saved scans): the left turn
# had released to 1636 just before handover, the orbit asked 1707 but the 10
# PWM step reached only 1676 in .5 s; the U-turn radius grew from ~.95 to
# ~1.1 m and put the body .08 m from the outer wall. Releases are unchanged.
ROUTE_ORBIT_LEFT_PWM_STEP = ROUTE_POST_PASS_PWM_STEP
# Explicit continue_route only. After the first-target pass the motor uses the
# operator's manual forward PWM instead of 1560: trial43 reached ~1.8 m/s at
# the second target, and the traffic-light section must be driven slowly
# because the countdown starts when the body crosses the infrared sensor.
ROUTE_SLOW_MOTOR = MIN_FORWARD_PWM
DRIVE_MOTORS = (TRIAL_MOTOR, ROUTE_SLOW_MOTOR)
POST_PASS_STAGES = ('right_align', 'corridor_follow', 'second_orbit', 'light_turn', 'light_lane')
# Route mode alone may extend the cumulative drive budget (user permitted a
# measured relaxation): the light-lane turn follows the two targets. The
# total deadline, coast and every stage budget still apply.
# Round 58 (Claude): the route now continues through the light lane to the
# finish zone (trial57 reached the light turn after ~7.7 s of drive).
ROUTE_MAX_DRIVE_S = 24.0
# Exit alignment / corridor pursuit: lookahead about one corridor width plus a
# heading-rate damping term (trial46 oscillated with a 1 m lookahead).
ROUTE_ALIGN_LOOKAHEAD_MAX_M = 2.0
ROUTE_ALIGN_DAMPING_S = .30
ROUTE_ALIGN_HEADING_RAD = math.radians(8)
# Light-lane turn after the second target: follow the deepest free direction
# for the measured body plus clearance, then hold once a long free lane is
# straight ahead and the side clearances balance (camera faces the lane axis,
# where the traffic light hangs over the center). No course constants.
LIGHT_STAGE_MAX_S = 4.0
LIGHT_FREE_HORIZON_M = 3.0
LIGHT_ALIGNED_FREE_M = 2.5
LIGHT_ALIGNED_HEADING_RAD = math.radians(6)
LIGHT_CENTER_TOLERANCE_M = .20
LIGHT_HEADINGS_DEG = tuple(range(-100, 101, 2))
# Unknown is never free: a front null run of <= 5 bins between two known
# returns blocks at the nearer one (the clearance bridging rule); any longer
# or unbracketed run blocks at this short range so no heading or alignment
# is chosen through it.
LIGHT_UNKNOWN_BLOCK_M = .60
# Trial41/47: the scan_incomplete frame that triggers the orbit wait can hide
# the first support's edge, and the tracker may then re-acquire the same
# support under a new ID. The wait may start from the last FULL isolation of
# the orbit support if it is fresh, holds through missing frames, and re-binds
# a new identity only if its first sighting lies within the tracker's own
# association gate of that last full sighting and it then confirms.
QUALITY_WAIT_REBIND_M = .30
QUALITY_WAIT_REBIND_WINDOW_S = .60
# Trial50: the coasting wait lost the first support at ~.9 m; the tracker then
# found the same support as new IDs (.75 m, then .37 m once stopped) that the
# association gate above never matched, and the orbit budget locked a car that
# was well placed. In a first-target wait, the nearest compact candidate on the
# left, confirmed and fully isolated within this range, is the orbited support.
ORBIT_REBIND_MAX_RANGE_M = 1.2
# Explicit route only: steer away from close known returns before the service
# body gate (8 cm, unchanged) is reached. Within this gap a return on the right
# (or a head-on return with the left front more open) demands at least
# ROUTE_AVOID_LEFT_PWM; one on the left caps steering at center. Before the
# first-target pass the inside (left) is left to the turn/orbit laws, which
# already keep their own inside clearance. Steps use the route step.
ROUTE_AVOID_GAP_M = .15
ROUTE_AVOID_LEFT_PWM = NEUTRAL+150
ROUTE_AVOID_PWM_STEP = 40
# Route waits (motor at neutral, car may still roll): the held left is
# released only while it exceeds the closest inside return's clearance-bearing
# demand by more than this margin (trial50: 1710 held while the support closed
# from .9 to .37 m on the inside). A static scene never changes the hold.
ROUTE_WAIT_INSIDE_MARGIN_PWM = 60
# Explicit route only. Trial51: the first support never confirmed during the
# left turn, the turn passed its 90 deg exit axis by 8 deg and the overshoot
# coast centered the steering at ~1.5 m/s; the car rolled straight into the
# outer board of the next lane (same spot as trial49). Instead the adopted left
# is held at drive power while the first-target handover is checked every
# frame; only after this long does the held-left coast (no centering) follow.
ROUTE_LEFT_OVERSHOOT_HOLD_S = 1.0
# Explicit route, after the first-target handover (round 53): check the stage's
# steering against the predicted body sweep along its arc, independent of any
# object recognition (trial52: the second support was not recognized and the
# car drove straight into it). Curvature per PWM is the value measured from
# the rebuilt paths of trials 49/50/52 (scripts/trial-map.py): ~1.0-1.1 1/m at
# 1655-1665, ~1.3 1/m at 1710 slow. If the stage target's arc meets a known
# return (body side .14 m, +4 cm ahead of the front face) within the
# lookahead, the nearest clear candidate is used; corridor and second-target
# stages prefer passing an obstacle on its left (task rule: the second cone is
# passed clockwise). Right of center only where the route already authorizes it.
ROUTE_PWM_PER_CURVATURE = 160.
ROUTE_PATH_LOOKAHEAD_M = .70
ROUTE_PATH_MARGIN_M = .04
ROUTE_PATH_RANGE_M = 2.
ROUTE_PATH_LEFT_CANDIDATES = tuple(range(NEUTRAL, SERVO_MAX, 20))+(SERVO_MAX,)
ROUTE_PATH_RIGHT_CANDIDATES = (1350, 1390, 1430, 1470)
_DIRECTION_COS = tuple(math.cos(math.radians(i)) for i in range(360))
_DIRECTION_SIN = tuple(math.sin(math.radians(i)) for i in range(360))
LIGHT_BRIDGE_MAX_BINS = 5
# Round 58 (Claude): light lane to the finish, no vision yet (user: run the
# whole route first). The lane is driven slowly so that, once the light is
# recognized, the car can stop inside the 1 m stop area with neutral coasting
# only (no reverse braking; measured coast decel ~.5 m/s^2; 1550 cruises at
# ~.6-1.3 m/s; nothing below 1550 starts the car). Speed is the closing rate
# of the board straight ahead (median of 3 scan-to-scan rates): coast above
# LANE_SPEED_HIGH_MPS, 1550 below LANE_SPEED_LOW_MPS, a 1560 restart below
# LANE_SPEED_RESTART_MPS (rules: stopping > 10 s outside light/crosswalk fails).
# Finish (rules: stop inside the 80 cm zone at the lane end, board contact
# +10 s): cut power once the board ahead is within body front + gap + the
# neutral coasting distance v^2/(2a) + v*reaction. Unknown speed uses the
# fastest measured 1550 speed.
LANE_SPEED_HIGH_MPS = .70
LANE_SPEED_LOW_MPS = .45
LANE_SPEED_RESTART_MPS = .15
LANE_SPEED_UNKNOWN_MPS = 1.3
LANE_COAST_DECEL_MPS2 = .5
LANE_REACTION_S = .2
LANE_FINISH_GAP_M = .15
LANE_FRONT_HALF_BINS = 3
LANE_ALIGNED_HEADING_RAD = math.radians(8)
# Lateral gap from the lidar origin to the support's current left edge at the
# bypass point: keeps the support above the detector's minimum range while it
# is abeam (trial43 lost the second support below .35 m) and leaves .26 m to
# the measured body side, above the existing .08 m maneuver clearance.
SECOND_BYPASS_SUPPORT_GAP_M = MIN_RANGE_M+.05
# Trial48: steering by bearing (180 PWM/rad) asked only ~1590 with the second
# support .9 m ahead, while reaching the bypass line there needs a ~1 m path
# radius. Steer by pursuit curvature kappa = 2y/(x^2+y^2) instead, with the
# uncalibrated PWM-per-curvature seen in the first-target orbits (~1705 at
# ~1 m radius). Not a measured wheel angle.
SECOND_CURVATURE_GAIN_PWM_M = 205.
# Round 55 (Claude): stages whose task rule is to keep the obstacle on the
# car's right (second support clockwise). An obstacle correction there may add
# left from a neutral or slightly right stage target (trial52 scans 436/438:
# corridor target ~1494-1500 straight at the unconfirmed second support; the
# round-53 check chose 1580/1700, the first round-55 bounds kept 1494/1500).
# It never invents right. Exit/light turns keep the strict bounds.
TASK_LEFT_BYPASS_STAGES = ('corridor_follow', 'second_orbit')
# Second-role recovery after an identity loss: the last full observation is a
# car-frame prior, and the car keeps moving (~1 m/s at 1550, up to ~100 deg/s
# yaw). Its association gate widens at the tolerant tracker rates with the
# time since that observation, capped here (gate <= 1.2 m / 80 deg). With a
# fixed gate a support re-seen after >~.3 m of travel was never recovered.
SECOND_RECOVERY_MAX_SLACK_S = .6
# Round 57 (Claude): second-support bypass in the current lane frame. Trial56:
# the confirmed second support stayed 3.5-7 deg from straight ahead from 2.3 m
# to .87 m. The support-relative pursuit point sits at the support's own
# distance, so it asked only ~1540-1620 while far and 1720 at .65 m: the car
# reached the support turned hard left, pointing at the far board, and could
# not swing back in time. With a measured lane, steer toward the line
# SECOND_BYPASS_SUPPORT_GAP_M left of the support's left edge (limited by the
# lane's left wall less the body clearance), reached within at most this
# lookahead along the lane: the offset is taken early and the car passes
# parallel to the lane. Same corridor gain as exit/corridor following. The
# command is the larger of this and the support pursuit (trial48: a near
# support whose offset is not yet taken still needs the pursuit's strong
# left). No measured lane (or a lane at more than SECOND_LANE_MAX_HEADING_RAD):
# the support-relative pursuit alone, unchanged.
SECOND_LANE_LOOKAHEAD_M = 1.0
SECOND_LANE_MAX_HEADING_RAD = math.radians(45)
# While bypassing the second support, do not turn the car more than this
# left of the measured lane: a cone contact continues (user rule), a board
# contact ends the run. Trial56 re-confirmed the support only at 1.43 m and
# the pursuit then drove 1720 until the car faced the far board. Synthetic
# lane model with .3 s steering lag (work/claude-turn57-*/bypass_sim*):
# start 1.8 m -> max heading 55 -> 44 deg, 1.43 m -> 120 -> 71 deg; a start
# at 2.3 m is unchanged. Gain chosen so that a car aligned with the lane
# (trial48 scan509, lane -3 deg, support .9 m ahead) keeps the >= 1680 left
# the pursuit needs (limit 1712).
SECOND_BYPASS_MAX_HEADING_RAD = math.radians(30)
SECOND_HEADING_HOLD_GAIN = 450.
# Round 59 (Claude): the heading limit acts on the heading predicted one
# steering lag ahead (lane-heading rate over consecutive fresh frames).
# Trial58: 445 at 12 deg left with 72 deg/s yaw was still allowed 1646; the
# car reached 35 deg left of the lane (left board) before the return acted.
# trial-map: commands act on the trajectory ~.2-.3 s later.
SECOND_STEER_LAG_S = .3
# Round 59 (Claude): the second role (corridor proof, its tracker and identity
# recovery) acquires a 3-return support. At 2.5-2.9 m a ~.17 m support spans
# ~3.5 one-degree bins, so frames alternate 3/4 returns and an unconfirmed
# identity kept restarting (trial58: confirmed only at 1.79 m, the near
# pursuit then turned the car into the left board). Saved trials 47-58, ahead
# sector: first confirmation 52 1.72->2.28 m, 55 1.75->2.01, 56 1.20->2.04,
# 57 2.20->2.40, 58 1.68->2.46; no extra identities on boards. Size,
# isolation, association, corridor uniqueness and 3-frame confirmation stay.
SECOND_ACQUISITION_MIN_POINTS = 3
SECOND_MAX_HEADING_RATE_RAD_S = math.radians(150)
# Releasing left toward center while the second return is being prepared
# (round 57): trial56 needed ~.25 s per adopted 40-PWM step, 1720 -> 1600 in
# .53 s, still not centered at the board .7 s after preparing. Release only;
# each step still waits for the previous command's adoption.
ROUTE_RETURN_RELEASE_PWM_STEP = 80
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
        route_drive_s = None
        if (continue_route and _number(max_drive_s)
                and MAX_DRIVE_S < max_drive_s <= ROUTE_MAX_DRIVE_S):
            route_drive_s, max_drive_s = max_drive_s, MAX_DRIVE_S
        super().__init__(started_at, max_drive_s=max_drive_s,
                         initial_presteer_pwm=initial_presteer_pwm)
        if route_drive_s is not None:
            self.max_drive_s = route_drive_s
        # User-selected neutral preparation budget for this experiment only.
        # Parent construction and default TurnMotion retain their five seconds.
        self.max_presteer_s = PRESTEER_MAX_S
        self.continue_route = continue_route
        self.target_tracker = (CompactTargetTracker(maintenance_bearing_rad=(-math.pi, math.pi),
                                                   select_associated=True, unconfirmed_boundary_gap=True,
                                                   tolerant=True)
                               if continue_route else CompactTargetTracker())
        self.compact_target = None
        self._route_left_hold_since = None
        self.left_overshoot_evidence = None
        self._last_seen_compact = None  # (receive time, copy) of the latest first-tracker observation
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
        self.left_turn_pause_used = False
        self.left_turn_pause_evidence = None
        self.left_turn_pause_geometry_error = None
        # Self-recovery (trial47 operator): recoverable stops hold at neutral
        # and keep re-checking whether the current task can resume; only the
        # total deadline and cumulative drive budget bound how often.
        self.neutral_pause_kind = None
        self.recovery_count = 0
        self.recovery_evidence = None
        self.post_pass_pause_evidence = None
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
        self.quality_wait_anchor = None
        self.quality_wait_rebind = None
        self._wait_rebind_candidate = None
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
        self._quality_full_streak = 0
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
        self.second_return_preparing = False
        self.second_return_evidence = None
        self.route_end_coast_servo = None
        self.light_since = None
        self._light_history = []
        self._light_heading = None
        self.light_feedback = None
        self.light_alignment_evidence = None
        self.lane_motor = ROUTE_SLOW_MOTOR
        self._lane_ranges = []
        self.lane_feedback = None
        self.finish_evidence = None
        self._exit_heading = None
        self.role_distinct_evidence = None
        self.second_acquisition_reason = None
        self._second_proof_at = None
        self._second_pass_history = []
        self._second_role_anchor = None
        self._second_recovering = False
        self._second_recovery_last = None
        self.second_rebind_evidence = None
        self._pause_stage_held = False
        # Route: current-scan proximity avoidance and steering inside waits.
        self._avoid = None
        self._exit_release_cache = None
        self._path_cache = None
        self.path_evidence = None
        self.path_override_count = 0
        self.avoid_count = 0
        self.avoid_evidence = []
        self.wait_steering_changes = 0

    @staticmethod
    def _new_second_tracker():
        # Trial43: the second support first appears ahead on either side;
        # once confirmed it moves from the front through the right to behind
        # the rear, where the pass is matured over .25 s.
        return CompactTargetTracker(acquisition_bearing_rad=(-math.pi/2, math.pi/2),
            maintenance_bearing_rad=(-math.pi, math.pi/2), select_associated=True,
            unconfirmed_boundary_gap=True, tolerant=True,
            acquisition_min_points=SECOND_ACQUISITION_MIN_POINTS)

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
        if (not isinstance(bins, list) or len(bins) < TOLERANT_MAINTENANCE_MIN_POINTS or len(set(bins)) != len(bins)
                or not isinstance(ranges, list) or len(ranges) != 360
                or any(type(i) is not int or not 0 <= i < 360 or not _number(ranges[i]) for i in bins)):
            return None
        points = [(ranges[i]*math.cos(math.radians(i)), -ranges[i]*math.sin(math.radians(i))) for i in bins]
        unknown = target.get('boundary_unknown_bins') or []
        if any(type(i) is not int or not 0 <= i < 360 for i in unknown):
            return None
        # Size-bounded unknown edge bins (compact_target maintenance rule) may
        # hide part of the support. Count them at the nearest observed support
        # range: forward/lateral extents used for release, pass and bypass
        # decisions can then only be conservative, never optimistic.
        near = min(ranges[i] for i in bins)
        points += [(near*math.cos(math.radians(i)), -near*math.sin(math.radians(i))) for i in unknown]
        return points

    def current_contact_target(self, scan, now):
        """Exact current returns of every cone-like support, for the body gate.

        User 2026-10-12: a cone may be hit and the run continues; boards
        still stop. Any isolated compact support facing the sensor in this
        fresh scan (compact_target.contact_supports) is a witness, whether or
        not it is the support being orbited or already confirmed. The service
        recomputes the same supports from the raw scan before exempting them.
        Read-only: no tracker or stage state changes.
        """
        if (not self.continue_route or self.phase not in ('drive', 'quality_wait')
                or self.drive_since is None or not isinstance(scan, dict)
                or not _number(now) or not _number(scan.get('received_at'))
                or not 0 <= now-scan['received_at'] < SCAN_AGE_S
                or type(scan.get('seq')) is not int or type(scan.get('at_ms')) is not int
                or not isinstance(scan.get('ranges'), list) or len(scan['ranges']) != 360):
            return None
        try:
            supports = contact_supports(scan['ranges'])
        except (ValueError, TypeError, KeyError):
            return None
        if not supports:
            return None
        bins = sorted({i for support in supports for i in support['support_bins']})
        return {'role': 'cone_like_support', 'track_id': None, 'confirmed': False, 'full_observation': True,
                'source_seq': scan['seq'], 'source_at_ms': scan['at_ms'],
                'source_received_at': scan['received_at'], 'support_bins': bins,
                'support_ranges_m': [scan['ranges'][i] for i in bins], 'semantic_class': 'unknown',
                'objects': [{'support_bins': list(support['support_bins']), 'range_m': support['range_m'],
                             'bearing_left_rad': support['bearing_left_rad'],
                             'diameter_m': support['diameter_m']} for support in supports],
                'orbit_track_id': self.orbit_track_id, 'second_track_id': self.second_track_id}

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
    def _observation_lease(scan, now, published, received):
        """A missing frame may preserve history, never count or renew a lease."""
        if not (_number(published) and _number(received) and isinstance(scan, dict)
                and _number(scan.get('at_ms')) and _number(scan.get('received_at'))):
            return False
        dt, rt = (scan['at_ms']-published)/1000, scan['received_at']-received
        return (0 <= dt < SCAN_AGE_S and 0 <= rt < SCAN_AGE_S
                and 0 <= now-received < SCAN_AGE_S and abs(dt-rt) <= .15)

    def _target_gap_held(self, tracker, identity, scan, now):
        held = getattr(tracker, 'held', None)
        return (isinstance(held, dict) and held.get('confirmed') is True
                and held.get('track_id') == identity
                and self._observation_lease(scan, now, held.get('last_full_isolation_at_ms'),
                                            held.get('last_full_isolation_received_at')))

    def _clear_target_recovery_history(self):
        # A new ID cannot inherit another identity's readiness or pass window.
        self._quality_good_history.clear()
        self._quality_full_streak = 0
        self._quality_pass_history.clear()
        self._first_pass_history.clear()
        self._second_pass_history.clear()

    def _mature_recovery_observation(self, scan):
        previous = self._quality_good_history[-1][0] if self._quality_good_history else None
        mature = self._mature_observation(self._quality_good_history, scan, .30)
        if scan['seq'] != previous:
            self._quality_full_streak = (1 if len(self._quality_good_history) == 1
                                         else self._quality_full_streak+1)
        # A retained window is not evidence that the missing frame was good.
        # Require two current full frames after any held gap; alternating
        # isolated good/missing frames cannot trigger powered recovery.
        return mature and self._quality_full_streak >= 2

    @staticmethod
    def _consecutive_fresh(history, scan, count=ROUTE_EXIT_PREPARE_FRAMES, max_gap_s=SCAN_AGE_S):
        """Same freshness/ordering resets as _mature_observation, frame count only.

        A repeated scan never counts twice; a gap or clock mismatch restarts.
        """
        row = (scan['seq'], scan['at_ms'], scan['received_at'])
        if history and row[0] == history[-1][0]:
            return len(history) >= count
        if history and (row[0] <= history[-1][0] or row[1] <= history[-1][1]
                        or row[2] <= history[-1][2]
                        or (row[1]-history[-1][1])/1000 >= max_gap_s
                        or row[2]-history[-1][2] >= max_gap_s
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
        if (not self.continue_route or self.phase != 'drive'
                or not self.handover_observed or self.first_pass_evidence is not None
                or self.right_exit_since is not None or self.route_stage != 'first_target'
                or not _number(now) or now < self.last_now):
            return refuse('route_state')
        if (control.get('armed') is not True or control.get('motor') not in DRIVE_MOTORS
                or control.get('command_acked') is not True
                or not NEUTRAL < control.get('servo', 0) == self.servo <= SERVO_MAX):
            return refuse('control_not_acked_left')
        scan = self._current_scan if scan is None else scan
        if (not isinstance(scan, dict) or not _number(scan.get('received_at'))
                or not 0 <= now-scan['received_at'] < SCAN_AGE_S):
            return refuse('scan_stale')
        previous = self.compact_target  # the last frame's target, before this scan
        observed = self.target_tracker.update(scan, now, acquire=True)
        if self._full_target_points(observed, scan, self.orbit_track_id) is not None:
            anchor = {'track_id': observed['track_id'], 'point_left_m': list(observed['point_left_m']),
                      'received_at': scan['received_at'], 'basis': 'trigger_frame_full_isolation'}
        else:
            last = (previous if isinstance(previous, dict) and previous.get('confirmed') is True
                    else self._last_confirmed_compact_target)
            if (isinstance(last, dict) and last.get('track_id') == self.orbit_track_id
                    and _number(last.get('last_full_isolation_received_at'))
                    and 0 <= now-last['last_full_isolation_received_at'] < SCAN_AGE_S):
                anchor = {'track_id': last['track_id'], 'point_left_m': list(last['point_left_m']),
                          'received_at': last['last_full_isolation_received_at'],
                          'basis': 'previous_fresh_full_isolation'}
            else:
                # Trial50: wait without an anchor rather than ending the
                # route; the nearest-left rebind can still recover the support.
                anchor = None
        self.compact_target = observed
        self.neutral_pause_kind = None
        self.recovery_count += 1
        self.quality_wait_anchor = anchor
        self.quality_wait_rebind = None
        self._wait_rebind_candidate = None
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

    def _nearest_left_rebind(self, scan, control):
        target = self.compact_target
        if (self.route_stage != 'first_target' or self.first_pass_evidence is not None
                or not isinstance(target, dict) or target.get('confirmed') is not True
                or target.get('track_id') in (None, self.orbit_track_id)
                or not _number(target.get('bearing_left_rad')) or not _number(target.get('range_m'))
                or not 0 < target['bearing_left_rad'] < math.pi
                or not target['range_m'] <= ORBIT_REBIND_MAX_RANGE_M):
            return None
        points = self._full_target_points(target, scan, target['track_id'])
        if points is None:
            return None
        try:
            others = _candidates(scan['ranges'], bearing_range_rad=(0., math.pi), tolerant=True)
        except (ValueError, TypeError, KeyError):
            return None
        if any(_number(o.get('range_m')) and o['range_m'] < target['range_m']-.05 for o in others):
            return None
        self.quality_wait_rebind = {'old_track_id': self.orbit_track_id, 'new_track_id': target['track_id'],
            'source_seq': scan['seq'], 'anchor': copy.deepcopy(self.quality_wait_anchor),
            'range_m': target['range_m'], 'bearing_left_rad': target['bearing_left_rad'],
            'basis': 'nearest_confirmed_full_compact_on_left_within_1.2m_during_first_target_wait',
            'physical_identity_verified': False}
        self.orbit_track_id = target['track_id']
        self.target_tracker.enable_close_maintenance(self.orbit_track_id)
        self._clear_target_recovery_history()
        self._route_event('quality_wait_target_rebind', scan, control, self.quality_wait_rebind)
        return points

    def _quality_wait_rebind(self, scan, control):
        target, anchor = self.compact_target, self.quality_wait_anchor
        if (not isinstance(target, dict) or target.get('track_id') == self.orbit_track_id
                or not isinstance(target.get('point_left_m'), list) or not _number(target.get('source_received_at'))):
            return None
        if anchor is None:
            return self._nearest_left_rebind(scan, control)
        if target.get('confirmation_count') == 1:
            if (math.dist(target['point_left_m'], anchor['point_left_m']) <= QUALITY_WAIT_REBIND_M
                    and 0 <= target['source_received_at']-anchor['received_at'] < QUALITY_WAIT_REBIND_WINDOW_S):
                self._wait_rebind_candidate = target['track_id']
            return None
        if target.get('confirmed') is not True or target['track_id'] != self._wait_rebind_candidate:
            return self._nearest_left_rebind(scan, control)
        points = self._full_target_points(target, scan, target['track_id'])
        if points is None:
            return None
        self.quality_wait_rebind = {'old_track_id': self.orbit_track_id, 'new_track_id': target['track_id'],
            'source_seq': scan['seq'], 'anchor': copy.deepcopy(anchor),
            'basis': 'new_identity_first_seen_within_association_gate_of_last_full_orbit_support',
            'physical_identity_verified': False}
        self.orbit_track_id = target['track_id']
        self.target_tracker.enable_close_maintenance(self.orbit_track_id)
        self._clear_target_recovery_history()
        self._route_event('quality_wait_target_rebind', scan, control, self.quality_wait_rebind)
        return points

    def _wait_steer(self, now, servo_feedback, neutral, base):
        """Route wait/pause steering while the car may still be rolling (trial49:
        ~.5 m/s^2 rolling decel; trial50: a held 1710 curled into the support).
        One ACKed step at a time toward base, plus proximity avoidance; the
        motor stays at neutral and every step is recorded for the service."""
        self.steering_target = self.quality_wait_servo
        if not neutral or servo_feedback != self.quality_wait_servo or self.quality_wait_previous_servo is not None:
            return
        lower = RIGHT_EXIT_MIN_PWM if self.neutral_pause_kind in ('post_pass', 'recovery') else NEUTRAL
        before = self.servo
        self.steering_target = max(lower, min(SERVO_MAX, base))
        self._slew(now)
        if not lower <= self.servo <= SERVO_MAX:
            self.servo = before
        if self.servo != before:
            self.quality_wait_previous_servo = before
            self.quality_wait_servo = self.servo
            self.wait_steering_changes += 1
        self.steering_target = self.quality_wait_servo

    def _exit_release_target(self, scan, now):
        """Release-only target once the first support is at the body front.

        Round 53 (trial49): centering at heading ~145 deg of the first U-turn
        drove the car into the outer board while the exit corridor axis was
        still well to the left. Route mode releases toward the measured exit
        corridor pursuit target instead, never right of center and never above
        the current command; center when no corridor is measured.
        """
        if not self.continue_route or not isinstance(scan, dict):
            return NEUTRAL
        cached = self._exit_release_cache
        if cached is not None and cached[0] == scan.get('seq'):
            return max(NEUTRAL, min(self.servo, cached[1]))
        # Measure only: the corridor identity used after the first pass must
        # start fresh exactly as before (round-53 replay of trial52: a U-turn
        # corridor kept as identity was rejected at the first pass -> pause).
        saved = (self.right_exit_geometry, self._right_exit_last_receive, self._right_exit_last_publication)
        target, rejection = self._right_exit_target(scan, now)
        self.right_exit_geometry, self._right_exit_last_receive, self._right_exit_last_publication = saved
        value = NEUTRAL if rejection is not None or not _number(target) else max(NEUTRAL, int(target))
        self._exit_release_cache = (scan.get('seq'), value)
        return max(NEUTRAL, min(self.servo, value))

    def _wait_inside_base(self, lookahead):
        """Held left, released only for a close inside return (no creep)."""
        held = self.quality_wait_servo
        self._limit_left_for_known_points(held, lookahead)
        demand = (self.inner_clearance or {}).get('clearance_bearing_demand_pwm')
        if not _number(demand):
            return held
        return min(held, round(demand)+ROUTE_WAIT_INSIDE_MARGIN_PWM)

    def _quality_wait_update(self, scan, now, control, advancing, quality_clear, resume_ready):
        self.steering_target = self.quality_wait_servo
        # Trial50: the orbit-entry (3 s) and wait (5 s) clocks locked a car that
        # was stopped well placed beside the support. Only the cumulative drive
        # budget (and the service's total deadline) bound a route wait now.
        if now-self.drive_since >= self.max_drive_s:
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
                or control.get('motor') not in (NEUTRAL,)+DRIVE_MOTORS
                or (control.get('neutral_acked') is True and control['motor'] != NEUTRAL)):
            self.lock('route_quality_wait_feedback_changed')
            return self._result(now)
        neutral = control.get('neutral_acked') is True and control['motor'] == NEUTRAL
        if not neutral and now-self.quality_wait_since >= SCAN_AGE_S:
            self.lock('route_quality_wait_neutral_ack_timeout')
            return self._result(now)
        points = self._full_target_points(self.compact_target, scan, self.orbit_track_id)
        if points is None:
            points = self._quality_wait_rebind(scan, control)
        if points is None:
            # Missing/partial support: power never resumes without the
            # confirmed, fully isolated orbit support. While still rolling,
            # steering may only release left for close inside returns or move
            # away from close known returns.
            self._clear_exit_prepare_window()
            self._quality_full_streak = 0
            if not (quality_clear is True and neutral
                    and self._target_gap_held(self.target_tracker, self.orbit_track_id, scan, now)):
                self._quality_good_history.clear()
                self._quality_pass_history.clear()
            # Round 54 (trial53): a support last fully seen at/behind the body
            # front (or a release latched before the wait) keeps releasing
            # toward the exit while the car coasts; otherwise hold, releasing
            # only for close inside returns.
            anchor = self.quality_wait_anchor
            anchored_front = (isinstance(anchor, dict) and isinstance(anchor.get('point_left_m'), list)
                              and anchor['point_left_m'][0] <= FRONT_BODY_EXTENT_M)
            base = (self._exit_release_target(scan, now) if self.quality_wait_release is not None or anchored_front
                    else self._wait_inside_base(1.))
            self._wait_steer(now, servo_feedback, neutral, base)
            self.reason = 'route_quality_wait_target_reacquiring'
            return self._result(now)
        self.quality_wait_anchor = {'track_id': self.orbit_track_id,
                                    'point_left_m': list(self.compact_target['point_left_m']),
                                    'received_at': scan['received_at'], 'basis': 'wait_frame_full_isolation'}
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
            # A steering step is issued only on ticks that do not request the
            # powered resume, so that output always holds an ACKed command.
            if self.phase == 'quality_wait':
                before = self.servo
                base = (self._exit_release_target(scan, now) if self.quality_wait_release is not None
                        else self._wait_inside_base(max(.35, min(1., self.compact_target['range_m']))))
                self._wait_steer(now, servo_feedback, neutral, base)
                if self.servo != before and self.quality_wait_release is not None and self.servo < before:
                    self.reason = 'route_quality_wait_neutral_release_left'
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
        good = self._mature_recovery_observation(scan)
        if (not good or resume_ready is not True
                or servo_feedback != self.quality_wait_servo or self.quality_wait_previous_servo is not None):
            # Resume only on an ACKed, unchanged neutral-wait steering command.
            return hold()
        if passed:
            if not pass_ready:
                return hold()
            _, rejection = self._right_exit_target(scan, now)
            if rejection is not None:
                # The target pass is current, but it does not manufacture an
                # exit corridor. Keep neutral and retry from fresh geometry.
                self.right_exit_geometry = None
                self._right_exit_last_receive = self._right_exit_last_publication = None
                self.quality_wait_evidence['last_corridor_rejection'] = rejection
                hold()
                self.reason = 'route_quality_wait_pass_corridor_unavailable'
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

    def begin_left_turn_quality_pause(self, now, control, *, scan=None):
        """Explicit route only: one bounded neutral pause before target handover.

        Trial44: one front_sparse frame during the left turn ended the run in
        the terminal held coast .16 s before the first target confirmed; the
        next frame was clean. Hold the ACKed left with neutral motor instead.
        Power returns only after clean quality matures over .30 s together
        with a current left-turn geometry measurement. Once per run; the
        original drive budget keeps running; nothing here steers.
        """
        scan = self._current_scan if scan is None else scan
        if (not self.continue_route or self.phase != 'drive'
                or self.handover_observed or self.route_stage != 'first_target'
                or self.first_pass_evidence is not None or self.right_exit_since is not None
                or self.quality_resume_pending
                or not _number(now) or now < self.last_now
                or control.get('armed') is not True or control.get('motor') != TRIAL_MOTOR
                or control.get('command_acked') is not True
                or not NEUTRAL < control.get('servo', 0) == self.servo <= SERVO_MAX
                or not isinstance(scan, dict) or not _number(scan.get('received_at'))
                or not 0 <= now-scan['received_at'] < SCAN_AGE_S):
            return False
        self.left_turn_pause_used = True
        self.left_turn_pause_geometry_error = None
        self._enter_neutral_pause('left_turn', 'route_left_turn_quality_pause_neutral', now, scan, control,
                                  'quality', 'clean quality over .30 s and current left-turn geometry')
        return True

    def begin_post_pass_quality_pause(self, now, control, *, scan=None):
        """Explicit route after the first pass: quality faults pause, not halt."""
        scan = self._current_scan if scan is None else scan
        if (not self.continue_route or self.phase != 'drive' or self.first_pass_evidence is None
                or self.route_stage not in POST_PASS_STAGES or self.quality_resume_pending
                or not _number(now) or now < self.last_now
                or control.get('armed') is not True
                or control.get('motor') not in ((NEUTRAL,)+DRIVE_MOTORS if self.route_stage == 'light_lane'
                                                else DRIVE_MOTORS)
                or control.get('command_acked') is not True
                or not RIGHT_EXIT_MIN_PWM <= control.get('servo', 0) == self.servo <= SERVO_MAX
                or not isinstance(scan, dict) or not _number(scan.get('received_at'))
                or not 0 <= now-scan['received_at'] < SCAN_AGE_S):
            return False
        self._enter_neutral_pause('post_pass', 'route_post_pass_quality_pause_neutral', now, scan, control,
                                  'quality', 'clean quality over .30 s and the stage input observed again')
        return True

    def _begin_recovery_hold(self, cause, now, control):
        """Explicit route after the first pass: a recoverable stop holds at
        neutral and keeps re-checking whether the stage can resume. Anything
        else (or an un-ACKed command) keeps the previous coast."""
        scan = self._current_scan
        if (self.continue_route and self.phase == 'drive' and self.first_pass_evidence is not None
                and self.route_stage in POST_PASS_STAGES and not self.quality_resume_pending
                and control.get('armed') is True and control.get('command_acked') is True
                and control.get('servo') == self.servo and RIGHT_EXIT_MIN_PWM <= self.servo <= SERVO_MAX
                and isinstance(scan, dict) and _number(scan.get('received_at'))
                and 0 <= self.last_now-scan['received_at'] < SCAN_AGE_S):
            # The corridor identity restarts from the next fresh observation.
            self.right_exit_geometry = None
            self._right_exit_last_receive = self._right_exit_last_publication = None
            self._route_alignment_history.clear()
            self._enter_neutral_pause('recovery', 'route_recovery_hold_neutral', now, scan, control,
                                      cause, 'clean quality over .30 s and the stage input observed again')
            return True
        self.begin_coast(cause, now)
        return False

    def _enter_neutral_pause(self, kind, reason, now, scan, control, cause, resume_requires):
        self.neutral_pause_kind = kind
        self.recovery_count += 1
        self.quality_wait_since, self.quality_wait_servo = now, self.servo
        self.quality_wait_previous_servo = None
        self.quality_wait_release = None
        self._quality_good_history.clear()
        evidence = {'kind': kind, 'cause': cause, 'stage': self.route_stage, 'entry_seq': scan['seq'],
                    'entry_at_ms': scan['at_ms'], 'entry_received_at': scan['received_at'],
                    'held_servo': self.servo, 'recovery_count': self.recovery_count,
                    'resume_requires': resume_requires}
        if kind == 'left_turn':
            self.left_turn_pause_evidence = evidence
        elif kind == 'post_pass':
            self.post_pass_pause_evidence = evidence
        self.recovery_evidence = evidence
        self.phase, self.reason = 'quality_wait', reason
        self.steering_target = self.servo
        self._route_event('neutral_pause_'+kind, scan, control, evidence)

    def _left_turn_pause_geometry(self, scan, now):
        """Refresh left-turn geometry from this real scan; never steers."""
        candidates = scan.get('corridor_candidates')
        if (not isinstance(candidates, list) or len(candidates) > 24
                or any(not _valid_candidate(c) for c in candidates)):
            self.lock('invalid_native_corridor_candidates')
            return False
        self.incoming_endpoint = None
        geometry_dt = (0 if self.last_geometry_publication is None
                       else (scan['at_ms']-self.last_geometry_publication)/1000)
        try:
            candidate = self._measurement(scan, candidates, geometry_dt)
        except ValueError as error:
            self.left_turn_pause_geometry_error = str(error)
            return False
        # Same bookkeeping as a successful TurnMotion measurement.
        self.last_geometry_receive, self.last_geometry_publication, self.last_geometry_seq = (
            now, scan['at_ms'], scan['seq'])
        self.geometry_missing = False
        self.corridor, self.selected, self.start_ready = candidate, True, True
        self._endpoint_observation(scan)
        self.left_turn_pause_geometry_error = None
        return True

    def _pause_stage_ready(self, scan, now):
        """Whether the paused task can be executed again from this frame."""
        kind, stage = self.neutral_pause_kind, self.route_stage
        self._pause_stage_held = False
        if kind == 'left_turn':
            return self._left_turn_pause_geometry(scan, now)
        if (stage == 'corridor_follow' and self.second_target_handover_observed
                and self._full_target_points(self.second_target, scan, self.second_track_id) is not None):
            # A drive loss can return to corridor following. Its recovery uses
            # the same role proof as an orbit pause, including right-rear support.
            self.route_stage = stage = 'second_orbit'
            self.second_orbit_since = now
        if stage in ('right_align', 'corridor_follow'):
            _, rejection = self._right_exit_target(scan, now)
            if rejection is not None:
                self._pause_stage_held = (
                    rejection in ('right_exit_corridor_missing', 'right_exit_corridor_ambiguous')
                    and self._observation_lease(scan, now, self._right_exit_last_publication,
                                                self._right_exit_last_receive))
                if not self._pause_stage_held:
                    self.right_exit_geometry = None  # restart identity after lease expiry
                self.recovery_evidence['last_readiness_rejection'] = rejection
                return False
            # The service body gate (8 cm, raw returns) still stops a real
            # contact; a model-margin deficit alone must not keep a stopped car
            # waiting forever. Resume once the corridor model does not overlap.
            return self._corridor_body_gap(self.right_exit_geometry) >= 0.
        if stage == 'light_turn':
            return self._free_space(scan, self._light_heading) is not None
        if stage == 'light_lane':
            return (self._current_lane(scan, now)[1] is not None
                    or self._free_space(scan, self._light_heading) is not None)
        if stage == 'second_orbit':
            target = self.second_target
            self._pause_stage_held = self._target_gap_held(self.second_tracker, self.second_track_id, scan, now)
            return (isinstance(target, dict) and target.get('confirmed') is True
                    and target.get('track_id') == self.second_track_id
                    and self._full_target_points(target, scan, self.second_track_id) is not None)
        return False

    def _neutral_pause_update(self, scan, lidar_age_s, now, control, *, safe, presteer_wait,
                              quality_clear, resume_ready):
        advancing = self._orbit_inputs(scan, lidar_age_s, now, control, safe, presteer_wait)
        if self.phase == 'locked':
            return self._result(self.last_now)
        self.steering_target = self.quality_wait_servo
        kind = self.neutral_pause_kind
        # Trial50: only the cumulative drive budget (and the service's total
        # deadline) bound a route pause; it keeps re-checking until then.
        if now-self.drive_since >= self.max_drive_s:
            self.lock(f'route_{kind}_pause_original_budget_expired')
            return self._result(now)
        servo_feedback = control.get('servo')
        if servo_feedback == self.quality_wait_servo:
            self.quality_wait_previous_servo = None
        pending_step = (self.quality_wait_previous_servo is not None
                        and servo_feedback == self.quality_wait_previous_servo
                        and control.get('command_acked') is False)
        if ((servo_feedback != self.quality_wait_servo and not pending_step)
                or control.get('motor') not in (NEUTRAL,)+DRIVE_MOTORS):
            self.lock(f'route_{kind}_pause_feedback_changed')
            return self._result(now)
        if self.quality_resume_pending:
            if (control.get('resume_acked') is True and control['motor'] == TRIAL_MOTOR
                    and control.get('command_acked') is True):
                self.quality_resume_pending = False
                self.recovery_evidence['resume_ack'] = {k: control.get(k) for k in ('motor', 'servo', 'seq', 'tick')}
                # Resume the same task with fresh stage clocks; the total
                # deadline and cumulative drive budget are never renewed.
                if self.route_stage == 'right_align' and kind != 'left_turn':
                    self.right_exit_since = now
                if self.route_stage == 'light_turn':
                    self.light_since = now
                self.neutral_pause_kind = None
            elif now-self.quality_resume_since >= SCAN_AGE_S:
                self.lock(f'route_{kind}_pause_resume_ack_timeout')
            # Even the ACK observation keeps the held command on this output.
            return self._result(now)
        if control.get('neutral_acked') is True and control['motor'] != NEUTRAL:
            self.lock(f'route_{kind}_pause_feedback_changed')
            return self._result(now)
        neutral = control.get('neutral_acked') is True and control['motor'] == NEUTRAL
        if not neutral and now-self.quality_wait_since >= SCAN_AGE_S:
            self.lock(f'route_{kind}_pause_neutral_ack_timeout')
            return self._result(now)
        if not advancing:
            return self._result(now)
        stage_ready = self._pause_stage_ready(scan, now) if quality_clear is True and neutral else False
        if quality_clear is not True or not neutral or not stage_ready:
            self._quality_full_streak = 0
            if not (quality_clear is True and neutral and self._pause_stage_held):
                self._quality_good_history.clear()
            base = (min(self.quality_wait_servo, NEUTRAL)
                    if self.route_stage == 'second_orbit' and self.second_return_preparing
                    else self.quality_wait_servo)
            self._wait_steer(now, servo_feedback, neutral, base)
            return self._result(now)
        if (not self._mature_recovery_observation(scan) or resume_ready is not True
                or servo_feedback != self.quality_wait_servo or self.quality_wait_previous_servo is not None):
            base = (min(self.quality_wait_servo, NEUTRAL)
                    if self.route_stage == 'second_orbit' and self.second_return_preparing
                    else self.quality_wait_servo)
            self._wait_steer(now, servo_feedback, neutral, base)
            return self._result(now)
        self.phase, self.reason = 'drive', f'route_{kind}_pause_resume_waiting_ack'
        self.quality_resume_pending, self.quality_resume_since = True, now
        self.recovery_evidence.update(resume_seq=scan['seq'], resume_at_ms=scan['at_ms'],
            resume_received_at=scan['received_at'], good_frame_count=len(self._quality_good_history))
        self._route_event(f'{kind}_pause_resume_request', scan, control, self.recovery_evidence)
        return self._result(now)

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
                or control.get('motor') not in (DRIVE_MOTORS if self.continue_route else (TRIAL_MOTOR,))
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
        seen = self._last_seen_compact
        target_on_left = (seen is not None and _number(now) and 0 <= now-seen[0] < SCAN_AGE_S
                          and _number(seen[1].get('bearing_left_rad')) and _number(seen[1].get('range_m'))
                          and 0 < seen[1]['bearing_left_rad'] < math.pi
                          and seen[1]['range_m'] <= ORBIT_REBIND_MAX_RANGE_M+.3)
        if (reason == 'left_turn_heading_overshoot' and self.continue_route and self.phase == 'drive'
                and not self.handover_observed and self.route_stage == 'first_target'
                and self._route_left_hold_since is None and NEUTRAL < self.servo <= SERVO_MAX
                and target_on_left and now >= self.last_now):
            # Trial51: a (not yet confirmed) first-target identity was just
            # observed on the left; the rule passes it anticlockwise, so keep
            # the adopted left while its confirmation/handover is checked.
            # Without such live evidence the original coast applies (no
            # course-shaped assumption that the route always keeps turning).
            self._route_left_hold_since = now
            self.left_overshoot_evidence = {'at': now, 'held_servo': self.servo,
                'scan_seq': (self._current_scan or {}).get('seq'),
                'last_seen_first_target': copy.deepcopy(seen[1]), 'last_seen_age_s': now-seen[0],
                'tracker_reason': getattr(self.target_tracker, 'reason', None),
                'basis': 'left_turn_past_exit_axis_with_live_first_target_identity_on_left'}
            self.steering_target = self.servo
            self.reason = 'route_left_turn_overshoot_hold_left'
            return
        if self.phase not in ('coast', 'locked') and _number(now) and now >= self.last_now:
            self.coast_since = now
        super().begin_coast(reason, now)

    def _route_left_hold_update(self, scan, lidar_age_s, now, control, safe, presteer_wait):
        """Held left at drive power after a route left-turn overshoot (trial51)."""
        advancing = self._orbit_inputs(scan, lidar_age_s, now, control, safe, presteer_wait)
        if self.phase == 'locked':
            return self._result(self.last_now)
        held = self.left_overshoot_evidence['held_servo']
        if (now-self.drive_since+1e-9 >= self.max_drive_s
                or now-self._route_left_hold_since+1e-9 >= ROUTE_LEFT_OVERSHOOT_HOLD_S):
            reason = ('maneuver_cumulative_drive_timeout' if now-self.drive_since >= self.max_drive_s
                      else 'route_left_turn_overshoot_hold_expired')
            self._route_left_hold_since = None
            if not self._begin_adopted_left_coast(reason, now, control, require_handover=False):
                self.begin_coast(reason, now)
            self._slew(now)
            return self._result(now)
        if advancing:
            self.steering_target = held
            self.reason = 'route_left_turn_overshoot_hold_left'
            self._slew(now)
        return self._result(now)

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

    def _route_proximity(self, scan):
        """Nearest known return within ROUTE_AVOID_GAP_M of the body box, or None."""
        ranges = scan.get('ranges') if isinstance(scan, dict) else None
        if not isinstance(ranges, list) or len(ranges) != 360:
            return None
        best = None
        for index, distance in enumerate(ranges):
            if not _number(distance) or not .02 <= distance <= 12:
                continue
            angle = math.radians(index)
            x, y = distance*math.cos(angle), -distance*math.sin(angle)
            head_on = x > FRONT_BODY_EXTENT_M and abs(y) <= SIDE_BODY_EXTENT_M/2
            if y > 0 and not head_on and self.first_pass_evidence is None:
                continue  # inside of the left turn / first orbit: their own laws
            gap = math.hypot(max(-REAR_BODY_EXTENT_M-x, 0., x-FRONT_BODY_EXTENT_M),
                             max(abs(y)-SIDE_BODY_EXTENT_M, 0.))
            if gap < ROUTE_AVOID_GAP_M and x >= -REAR_BODY_EXTENT_M and (best is None or gap < best['gap_m']):
                best = {'gap_m': gap, 'point_left_m': [x, y], 'bin': index, 'head_on': head_on}
        if best is None:
            return None
        if best['head_on']:
            def nearest(bins):
                values = [ranges[i] for i in bins if _number(ranges[i])]
                return min(values) if values else 12.
            side = 'front_left_open' if nearest(range(270, 340)) >= nearest(range(20, 91)) else 'front_right_open'
        else:
            side = 'left' if best['point_left_m'][1] > 0 else 'right'
        return {**best, 'side': side, 'source_seq': scan.get('seq')}

    @staticmethod
    def _path_points(scan):
        ranges = scan.get('ranges') if isinstance(scan, dict) else None
        if not isinstance(ranges, list) or len(ranges) != 360:
            return None
        points = []
        for index, distance in enumerate(ranges):
            if _number(distance) and .02 <= distance <= ROUTE_PATH_RANGE_M:
                x, y = _DIRECTION_COS[index]*distance, -_DIRECTION_SIN[index]*distance
                if x > -REAR_BODY_EXTENT_M-.2:
                    points.append((x, y))
        return points

    @staticmethod
    def _path_free_distance(points, servo):
        """Lidar travel before the body meets a known return on this arc (inf: clear)."""
        kappa = (servo-NEUTRAL)/ROUTE_PWM_PER_CURVATURE
        best = math.inf
        for x, y in points:
            if abs(kappa) < 1e-3:
                along, normal = x, y
            else:
                radius = 1/abs(kappa)
                side = y if kappa > 0 else -y
                rho = math.hypot(x, radius-side)
                normal = radius-rho
                along = radius*math.atan2(x, radius-side)
            if along+REAR_BODY_EXTENT_M < 0:
                continue  # behind the rear face: never reached driving forward
            half = SIDE_BODY_EXTENT_M+(ROUTE_PATH_MARGIN_M if along > FRONT_BODY_EXTENT_M else 0.)
            if abs(normal) > half:
                continue
            best = min(best, max(0., along-FRONT_BODY_EXTENT_M))
        return best

    def _route_right_authorized(self):
        return (self.continue_route and self.phase == 'drive' and not self.quality_resume_pending
                and self.first_pass_evidence is not None and self._route_center_ack is not None
                and ((self.route_stage in ('right_align', 'corridor_follow') and self.right_exit_center_ack is not None)
                     or (self.route_stage == 'second_orbit' and self.second_center_ack is not None)
                     or self.route_stage in ('light_turn', 'light_lane')))

    def _route_override_bounds(self, target):
        """An obstacle correction may release a turn, never invent its opposite.

        The stage target has already passed its center-ACK gate. In particular
        a neutral target is a command, not permission for this layer to choose
        a new direction. During the first orbit, a hold/release must also not
        become a renewed left boost. These are task-relative bounds, with no
        assumption about where the support or exit lies on the course.
        """
        if self.route_stage == 'second_orbit' and self.second_return_preparing:
            # The stage already checks each return arc against the entire
            # support and the current boards. Another arc has not passed that
            # check. In particular, keep its neutral ACK request at neutral.
            return target, target
        if self.route_stage in TASK_LEFT_BYPASS_STAGES:
            lower = RIGHT_EXIT_MIN_PWM if target < NEUTRAL else NEUTRAL
            upper = SERVO_MAX
            cap = getattr(self, '_task_left_upper', None)
            scan = self._current_scan
            if cap is not None and isinstance(scan, dict) and cap[0] == scan.get('seq'):
                # Never turn the car past the lane-heading limit to dodge an
                # obstacle: a cone contact continues, a board contact ends.
                upper = max(target, min(SERVO_MAX, cap[1]))
            return lower, max(lower, upper)
        if target == NEUTRAL:
            return NEUTRAL, NEUTRAL
        if target < NEUTRAL:
            return RIGHT_EXIT_MIN_PWM, NEUTRAL
        upper = SERVO_MAX
        if (self.handover_observed and self.route_stage == 'first_target'
                and self.first_pass_evidence is None and target <= self.servo):
            upper = target
        return NEUTRAL, upper

    def _route_path_target(self, target):
        """Stage target, or the nearest candidate whose arc is clear (round 53).

        Trial53: with the car pointing at a board after the first pass, no arc
        was clear and the old fallback took the arc with the longest free
        distance (1-3 cm more, always left). That held the servo off center,
        so the center ACK that must precede the right turn never came and the
        car turned into the board. The check therefore never overrides when
        no candidate is clear, and never while the stage waits for its center
        ACK before steering right; it may only pick a clear arc consistent
        with the stage's direction and first-target hold/release intention.
        """
        scan = self._current_scan
        natural = self.natural_steering_target
        # A right turn waiting for its center ACK is left alone (trial53),
        # except where the task side of an obstacle is left anyway.
        pending_right = (_number(natural) and natural < NEUTRAL and not self._route_right_authorized()
                         and self.route_stage not in TASK_LEFT_BYPASS_STAGES)
        lower, upper = self._route_override_bounds(target)
        key = (scan.get('seq') if isinstance(scan, dict) else None, target, self.route_stage,
               self._route_right_authorized(), pending_right, lower, upper)
        if self._path_cache is not None and self._path_cache[0] == key:
            return self._path_cache[1]
        points = self._path_points(scan)
        choice = target
        if points and not pending_right:
            free = self._path_free_distance(points, target)
            if free < ROUTE_PATH_LOOKAHEAD_M:
                candidates = list(ROUTE_PATH_LEFT_CANDIDATES)
                if key[3]:
                    candidates += list(ROUTE_PATH_RIGHT_CANDIDATES)
                # Filter before scoring: clamping a clear opposite arc would
                # produce a different arc which has never been checked.
                candidates = [c for c in candidates if lower <= c <= upper]
                scored = {c: self._path_free_distance(points, c) for c in candidates}
                clear = [c for c, f in scored.items() if f >= ROUTE_PATH_LOOKAHEAD_M]
                prefer_left = self.route_stage in ('corridor_follow', 'second_orbit')
                if clear:
                    above = [c for c in clear if c >= target]
                    pool = above if prefer_left and above else clear
                    choice = min(pool, key=lambda c: (abs(c-target), -c))
                    self.path_override_count += int(choice != target)
                self.path_evidence = {'source_seq': key[0], 'stage': self.route_stage, 'stage_target': target,
                    'stage_free_m': free, 'chosen': choice,
                    'chosen_free_m': scored.get(choice, free), 'clear_candidates': len(clear),
                    'overridden': choice != target, 'right_authorized': key[3],
                    'stage_override_bounds': [lower, upper],
                    'pwm_per_curvature': ROUTE_PWM_PER_CURVATURE, 'swept_path_certified': False}
        self._path_cache = (key, choice)
        return choice

    def _avoid_target(self, target):
        avoid = self._avoid
        if avoid is None:
            return target
        if avoid['side'] in ('left', 'front_right_open'):
            choice = min(target, NEUTRAL)
        else:
            choice = max(target, ROUTE_AVOID_LEFT_PWM)
        lower, upper = self._route_override_bounds(target)
        return max(lower, min(upper, choice))

    def _slew(self, now):
        if (self.continue_route and self.phase == 'drive' and self.handover_observed
                and not self.quality_resume_pending):
            avoid_target = self._route_path_target(self.steering_target)
        elif self.continue_route and self.phase in ('drive', 'quality_wait'):
            avoid_target = self._avoid_target(self.steering_target)
        else:
            avoid_target = self.steering_target
        if now-self.last_change+1e-9 >= PWM_INTERVAL_S and self.servo != avoid_target:
            target = avoid_target
            reducing_left = self.servo > NEUTRAL and target < self.servo
            # Round 58: light turn/lane included (trial57 turned right at the
            # generic 10 PWM/100 ms and hit the left-front board).
            post_pass = (self.continue_route and self.phase == 'drive'
                         and self.first_pass_evidence is not None
                         and self.route_stage in ('right_align', 'corridor_follow', 'second_orbit',
                                                  'light_turn', 'light_lane'))
            orbit_left = (self.continue_route and self.phase == 'drive' and self.handover_observed
                          and self.route_stage == 'first_target' and self.first_pass_evidence is None
                          and target > self.servo >= NEUTRAL)
            if (post_pass and reducing_left and self.route_stage == 'second_orbit'
                    and self.second_return_preparing):
                post_step = ROUTE_RETURN_RELEASE_PWM_STEP
            else:
                post_step = ROUTE_POST_PASS_PWM_STEP
            step = PRESTEER_PWM_STEP if self.phase == 'presteer' else post_step if post_pass else (
                (ROUTE_EXIT_RELEASE_PWM_STEP if self._route_exit_release_active() else LEFT_RELEASE_PWM_STEP)
                if reducing_left else ROUTE_ORBIT_LEFT_PWM_STEP if orbit_left else PWM_STEP)
            if avoid_target != self.steering_target:
                step = max(step, ROUTE_AVOID_PWM_STEP)
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
        if result['motor'] == TRIAL_MOTOR:
            result['motor'] = self._route_motor()
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
                          'route_post_pass': ROUTE_POST_PASS_PWM_STEP if self.continue_route else None,
                          'route_orbit_increase_left': ROUTE_ORBIT_LEFT_PWM_STEP if self.continue_route else None,
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
            quality_wait_anchor=copy.deepcopy(self.quality_wait_anchor),
            quality_wait_rebind=copy.deepcopy(self.quality_wait_rebind),
            left_turn_pause_used=self.left_turn_pause_used,
            neutral_pause_kind=self.neutral_pause_kind,
            recovery_count=self.recovery_count,
            recovery_evidence=copy.deepcopy(self.recovery_evidence),
            route_left_hold_active=self._route_left_hold_since is not None,
            left_overshoot_evidence=copy.deepcopy(self.left_overshoot_evidence),
            route_avoid=copy.deepcopy(self._avoid), avoid_count=self.avoid_count,
            path_evidence=copy.deepcopy(self.path_evidence), path_override_count=self.path_override_count,
            avoid_evidence=copy.deepcopy(self.avoid_evidence),
            wait_steering_changes=self.wait_steering_changes,
            post_pass_pause_evidence=copy.deepcopy(self.post_pass_pause_evidence),
            left_turn_pause_evidence=copy.deepcopy(self.left_turn_pause_evidence),
            left_turn_pause_geometry_error=self.left_turn_pause_geometry_error,
            quality_wait_evidence=copy.deepcopy(self.quality_wait_evidence),
            quality_resume_pending=self.quality_resume_pending,
            quality_wait_release=copy.deepcopy(self.quality_wait_release),
            quality_wait_previous_servo=self.quality_wait_previous_servo,
            route_alignment_evidence=copy.deepcopy(self.route_alignment_evidence),
            second_target=copy.deepcopy(self.second_target),
            second_rebind_evidence=copy.deepcopy(self.second_rebind_evidence),
            second_identity_recovering=self._second_recovering,
            second_center_ack=copy.deepcopy(self.second_center_ack),
            second_pass_evidence=copy.deepcopy(self.second_pass_evidence),
            second_feedback=copy.deepcopy(self.second_feedback),
            second_return_preparing=self.second_return_preparing,
            second_return_evidence=copy.deepcopy(self.second_return_evidence),
            route_end_coast_servo=self.route_end_coast_servo,
            light_feedback=copy.deepcopy(self.light_feedback),
            light_alignment_evidence=copy.deepcopy(self.light_alignment_evidence),
            lane_feedback=copy.deepcopy(self.lane_feedback), lane_motor=self.lane_motor,
            finish_evidence=copy.deepcopy(self.finish_evidence),
            second_acquisition_reason=self.second_acquisition_reason,
            role_distinct_evidence=copy.deepcopy(self.role_distinct_evidence),
            route_events=copy.deepcopy(self.route_events),
            two_target_observed_pass_complete=self.second_pass_evidence is not None,
            route_right_output_authorized=(self.continue_route and self.phase == 'drive'
                and not self.quality_resume_pending and self.first_pass_evidence is not None
                and self._route_center_ack is not None
                and ((self.route_stage in ('right_align', 'corridor_follow')
                      and self.right_exit_center_ack is not None)
                     or (self.route_stage == 'second_orbit' and self.second_center_ack is not None)
                     or self.route_stage in ('light_turn', 'light_lane'))),
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
        close_bound_role = (self.continue_route and self.handover_observed
                            and self.first_pass_evidence is None
                            and target.get('close_maintenance_observation') is True
                            and self._full_target_points(target, scan, self.orbit_track_id) is not None)
        point = target.get('point_left_m')
        return (target.get('kind') == 'lidar_compact_object'
                and target.get('semantic_class') == 'unknown'
                and target.get('candidate_only') is True
                and type(target.get('source_seq')) is int and target['source_seq'] == scan.get('seq')
                and type(target.get('source_at_ms')) is int and target['source_at_ms'] == scan.get('at_ms')
                and isinstance(point, list) and len(point) == 2 and all(_number(v) for v in point)
                and _number(target.get('range_m'))
                and (.02 if close_bound_role else MIN_RANGE_M) <= target['range_m'] <= 3.0
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

    @staticmethod
    def _held_support_points(held):
        """Support points of a tracker-held identity's last observation (car frame then)."""
        if not isinstance(held, dict) or not isinstance(held.get('point_left_m'), list):
            return []
        x, y = held['point_left_m']
        radius = (held.get('diameter_m') or 0.)/2
        return [(x-radius, y), (x+radius, y)]

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
                or not isinstance(bins, list) or len(bins) < TOLERANT_MAINTENANCE_MIN_POINTS
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
        x = sum(p[0] for p in points)/len(points)
        if self.continue_route:
            # Route: extents include size-bounded unknown edges (conservative).
            points = self._full_target_points(target, scan, self.orbit_track_id) or points
        # Always derive extents from current raw support, never a cone radius.
        front = max(p[0] for p in points)
        side = min(p[1] for p in points)-SIDE_BODY_EXTENT_M
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
        adopted = (control.get('command_acked') is True
                   and control.get('motor') in (DRIVE_MOTORS if self.continue_route else (TRIAL_MOTOR,))
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
            # Round 54 (trial53): one missed scan between the two body-front
            # observations (tracker-held frame) must not restart the count.
            geometric_ready = self._consecutive_fresh(self._first_exit_prepare_history, scan,
                                                      max_gap_s=SCAN_AGE_S+.15 if self.continue_route else SCAN_AGE_S)
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
        if len(forward) > 1 and self.continue_route:
            # Trial34 604 / trial45: the native fit can report the same
            # corridor twice a few degrees apart. Aliases within the existing
            # window-merge tolerances are one axis (keep the best-supported
            # one); a different axis (corner/T) stays ambiguous.
            base = forward[0]
            if all(abs(_wrap(c['heading_left_rad']-base['heading_left_rad'])) <= PAIR_MERGE_HEADING_RAD
                   and abs(c['center_offset_left_m']-base['center_offset_left_m']) <= PAIR_MERGE_OFFSET_M
                   and abs(c['width_m']-base['width_m']) <= max(.15, MAX_EXIT_WIDTH_CHANGE_FRACTION*base['width_m'])
                   for c in forward[1:]):
                forward = [max(forward, key=lambda c: (c.get('left_wall_points', 0)+c.get('right_wall_points', 0),
                                                       -c.get('fit_error_m', 0.)))]
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
                        min(ROUTE_ALIGN_LOOKAHEAD_MAX_M, c['width_m']) if self.continue_route
                        else min(1., c['width_m']/2))
        heading_rate = 0.
        if self.continue_route and previous is not None:
            # Trial46 overshot both sides with a 1 m lookahead: damp with the
            # observed corridor-heading rate (the car yawing left makes it fall).
            heading_rate = (_wrap(c['heading_left_rad']-previous['heading_left_rad'])
                            / ((scan['at_ms']-previous['source_at_ms'])/1000))
        error = (c['heading_left_rad']+math.atan2(c['center_offset_left_m'], lookahead)
                 + ROUTE_ALIGN_DAMPING_S*heading_rate)
        # Trial45: after the first pass the corridor axis was 9.4 deg LEFT and
        # its center .66 m left (trial43: axis 10 deg right). The exit side is
        # observed, never assumed: route mode pursues the current corridor
        # center line either way; right still needs a current center ACK.
        upper = SERVO_MAX if self.continue_route else NEUTRAL
        target = max(RIGHT_EXIT_MIN_PWM, min(upper, NEUTRAL+round(LEFT_TRIAL_GAIN*error)))
        self.right_exit_geometry = {**copy.deepcopy(c), 'source_seq': scan['seq'],
            'source_at_ms': scan['at_ms'], 'received_at': scan['received_at'],
            'lookahead_m': lookahead, 'steering_target_pwm': target, 'heading_rate_rad_s': heading_rate,
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

    def _route_motor(self):
        """Commanded drive PWM: slow from the first-target handover in route mode.

        Trials 49-51 (rebuilt paths): ~1.3-1.6 m/s at 1560 through the first
        U-turn and ~.5 m/s^2 rolling decel; 1550 measured ~.6-1.1 m/s. The
        left turn keeps 1560; a powered resume keeps its 1560 ACK sequence.
        """
        slow_stage = ((self.handover_observed and self.route_stage == 'first_target')
                      or (self.first_pass_evidence is not None and self.route_stage in POST_PASS_STAGES))
        if (self.continue_route and self.phase == 'drive' and self.route_stage == 'light_lane'
                and not self.quality_resume_pending):
            return self.lane_motor  # slow light lane: 1550 / neutral coast / 1560 restart
        return (ROUTE_SLOW_MOTOR if (self.continue_route and self.phase == 'drive' and slow_stage
                                     and not self.quality_resume_pending)
                else TRIAL_MOTOR)

    def _route_adopted(self, control, now):
        if control.get('servo', NEUTRAL) > NEUTRAL:
            self._route_center_ack = None
            self.second_center_ack = None
            if self.servo < NEUTRAL:
                self.lock('route_actual_steering_crossed_center')
                return False
        motors = (NEUTRAL,)+DRIVE_MOTORS if self.route_stage == 'light_lane' else DRIVE_MOTORS
        adopted = (control.get('motor') in motors and control.get('servo') == self.servo
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
        elif self.servo == NEUTRAL and control.get('motor') in DRIVE_MOTORS:
            # A center ACK is recorded only under drive power (service check);
            # a neutral coasting pulse keeps the previous one.
            self._route_center_ack = {k: control.get(k) for k in ('servo', 'motor', 'tick', 'seq')}
        return True

    def _right_exit_update(self, scan, now, control):
        if now-self.right_exit_since+1e-9 >= RIGHT_EXIT_MAX_S:
            if self.continue_route:
                self._begin_recovery_hold('first_target_right_exit_trial_timeout', now, control)
            else:
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
            # Trial45: one ambiguous frame coasted the exit straight into a
            # wall; route mode gives it the same lease as a missing corridor.
            held = ((rejection == 'right_exit_corridor_missing'
                     or (self.continue_route and rejection == 'right_exit_corridor_ambiguous'))
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
            elif self.continue_route:
                self._begin_recovery_hold(rejection, now, control)
                self._slew(now)
            else:
                self.begin_coast(rejection, now)
                self._slew(now)
            return self._result(now)
        if self.continue_route and not self._route_adopted(control, now):
            return self._result(now)
        adopted = (control.get('motor') in (DRIVE_MOTORS if self.continue_route else (TRIAL_MOTOR,))
                   and control.get('servo') == self.servo and control.get('command_acked') is True)
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
                    and control.get('motor') in (DRIVE_MOTORS if self.continue_route else (TRIAL_MOTOR,))
                    and control.get('command_acked') is True):
                self.right_exit_center_ack = {k: control.get(k) for k in ('servo', 'motor', 'tick', 'seq')}
                self._route_center_ack = dict(self.right_exit_center_ack)
            elif not self.continue_route:
                target = NEUTRAL
        natural = target
        if self.continue_route and target < NEUTRAL and (
                self.right_exit_center_ack is None or self._route_center_ack is None or self.servo > NEUTRAL):
            # Left toward the observed center line is direct; right still
            # needs a current actual center ACK after any left command.
            target = NEUTRAL
        if self.continue_route:
            # Trial46: the second support was already visible during exit
            # alignment; the same role proof and confirmation apply here.
            self._second_role_observation(scan, now)
            if self.second_target is not None and self.second_target.get('confirmed') is True:
                return self._begin_second_orbit(scan, now, control)
            c = self.right_exit_geometry
            aligned = (abs(c['heading_left_rad']) <= ROUTE_ALIGN_HEADING_RAD
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
        self.steering_target, self.natural_steering_target = target, natural
        self.reason = ('first_target_exit_alignment_left' if target > NEUTRAL
                       else 'first_target_right_exit_tracking' if self.right_exit_center_ack is not None
                       else 'first_target_right_exit_waiting_center_ack')
        self._slew(now)
        return self._result(now)

    def _start_second_identity_recovery(self):
        self._clear_second_return()
        if self._second_role_anchor is None:
            return
        self._second_recovering = True
        self._second_recovery_last = None
        # Acquisition after role handover includes the right rear, not the
        # passed first target's left-rear sector. Role proof below is still
        # required; this does not acquire any arbitrary right-side object.
        self.second_tracker.widen_acquisition((-math.pi, math.pi/2))

    def _second_recovery_observation(self, scan, now, *, fallback=False):
        """Fresh reconfirmation shared by drive loss and neutral-wait loss.

        The last full second observation is an identity prior, not current
        support. Its association gate widens with the time since it, capped
        (SECOND_RECOVERY_MAX_SLACK_S). A new ID needs the tracker's original
        multi-frame confirmation, with every current candidate distinct from
        any currently observed first support. Returns True when exactly one
        proven support advanced the tracker this frame. With ``fallback`` a
        frame without one leaves the tracker untouched for the caller's own
        corridor role proof (driving: the support may now be ahead again).
        """
        anchor = self._second_role_anchor
        if not self._second_recovering or anchor is None or self.first_pass_evidence is None:
            return False
        previous = self._second_recovery_last
        reference = (previous if isinstance(previous, dict)
                     and self._observation_lease(scan, now, previous.get('source_at_ms'),
                                                 previous.get('source_received_at')) else anchor)
        candidates = _candidates(scan['ranges'], bearing_range_rad=(-math.pi, math.pi/2), tolerant=True,
                                 min_points=SECOND_ACQUISITION_MIN_POINTS)
        first = self.compact_target
        first_bins = (set(first['support_bins']) if isinstance(first, dict)
                      and first.get('track_id') == self.orbit_track_id
                      and first.get('source_seq') == scan.get('seq') else set())
        first_points = [(scan['ranges'][i]*math.cos(math.radians(i)),
                         -scan['ranges'][i]*math.sin(math.radians(i))) for i in first_bins]
        slack = min(SECOND_RECOVERY_MAX_SLACK_S,
                    max(0., (scan['at_ms']-reference['source_at_ms'])/1000-.1))
        def proven(candidate):
            if (not _associated(candidate, reference, slack)
                    or first_bins.intersection(candidate['support_bins'])):
                return False
            points = [(scan['ranges'][i]*math.cos(math.radians(i)),
                       -scan['ranges'][i]*math.sin(math.radians(i))) for i in candidate['support_bins']]
            return not first_points or min(math.dist(a, b) for a in first_points for b in points) > MAX_DIAMETER_M
        matches = [candidate for candidate in candidates if proven(candidate)]
        if len(matches) != 1 and fallback:
            self.second_acquisition_reason = 'second_role_recovery_unproven_corridor_proof_next'
            return False
        if len(matches) != 1:
            if candidates:
                # An unrelated/ambiguous current object cannot advance a new
                # identity, even if acquire=False would keep associating it.
                self.second_tracker._forget('second_role_recovery_unproven')
                self._second_recovery_last = None
            self.second_tracker.update(scan, now, acquire=False)
            self.second_target = None
            self.second_acquisition_reason = 'second_role_recovery_current_support_unproven'
            return False
        target = self.second_tracker.update(scan, now, acquire=True)
        self.second_target = target
        if target is not None and not target['tracking_only_boundary_gap']:
            self._second_recovery_last = copy.deepcopy(target)
        self.second_acquisition_reason = self.second_tracker.reason
        if (target is None or set(target['support_bins']) != set(matches[0]['support_bins'])
                or self._full_target_points(target, scan, target['track_id']) is None):
            return True
        old_id = self.second_track_id
        self.second_track_id = target['track_id']
        self.second_tracker.enable_close_maintenance(self.second_track_id)
        self._second_recovering = False
        self.second_target_handover_observed = True
        self._second_role_anchor = copy.deepcopy(target)
        self.second_orbit_since = now  # new confirmed attempt; cumulative deadline is unchanged
        self._clear_target_recovery_history()
        self._clear_second_return()
        self.second_center_ack = None
        self.second_rebind_evidence = {
            'old_track_id': old_id, 'new_track_id': target['track_id'],
            'source_seq': scan['seq'], 'source_at_ms': scan['at_ms'],
            'source_received_at': scan['received_at'], 'anchor_source_seq': anchor['source_seq'],
            'support_bins': list(target['support_bins']),
            'basis': 'current_confirmed_support_associated_with_last_complete_second_observation',
            'physical_identity_verified': False, 'observed_rear_pass': False}
        return True

    def _update_second_tracking(self, scan, now):
        if self._second_recovering:
            self._second_recovery_observation(scan, now)
            return
        self.second_target = self.second_tracker.update(scan, now, acquire=False)
        if self._full_target_points(self.second_target, scan, self.second_track_id) is not None:
            self._second_role_anchor = copy.deepcopy(self.second_target)
        elif self.second_target is None and not self._target_gap_held(
                self.second_tracker, self.second_track_id, scan, now):
            self._start_second_identity_recovery()

    def _second_role_observation(self, scan, now):
        """One distinct compact support ahead inside the current corridor.

        Trial43: the second target appeared at the left front while the first
        was already behind; the old right-half-only, first-still-visible proof
        never matched. The first support has a recorded pass behind the rear
        and the car follows a forward corridor, so a support ahead of the body
        front inside that corridor is a different object. If the first support
        is still fully visible it must also share no bins and be separated.
        """
        # Driving after a second-role loss: the anchored recovery first; when
        # it proves nothing this frame, the original corridor proof below may
        # acquire the support again ahead (trial52 class: the car moved on).
        if self._second_recovering and self._second_recovery_observation(scan, now, fallback=True):
            return
        c = self.right_exit_geometry
        proof = None
        if self.first_pass_evidence is not None and isinstance(c, dict):
            first = self._full_target_points(self.compact_target, scan, self.orbit_track_id)
            heading = c['heading_left_rad']
            bound = c['width_m']/2-MANEUVER_BODY_CLEARANCE_M
            ahead = []
            for target in _candidates(scan['ranges'], bearing_range_rad=(-math.pi/2, math.pi/2), tolerant=True,
                                      min_points=SECOND_ACQUISITION_MIN_POINTS):
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
        if (proof is None and self._second_proof_at is not None
                and 0 <= now-self._second_proof_at < SCAN_AGE_S):
            # Trial52: the second support flickered (dropout band) and every
            # missed frame discarded its tracker. Within one lease a frame
            # without proof neither advances (counts) nor discards the tracker.
            self.second_target = None
            self.second_acquisition_reason = 'second_role_proof_gap_held'
            return
        if proof is None:
            # acquire=False alone would still mature an existing unconfirmed
            # association. No role proof means discard that entire interval.
            self._second_proof_at = None
            if not self._second_recovering:
                self.second_tracker = self._new_second_tracker()
            self.second_target = None
            self.role_distinct_evidence = None
            self.second_acquisition_reason = 'second_role_current_distinct_support_missing'
            return
        self._second_proof_at = now
        self.second_target = self.second_tracker.update(scan, now, acquire=True)
        self.role_distinct_evidence = proof
        self.second_acquisition_reason = self.second_tracker.reason

    def _corridor_follow_update(self, scan, now, control):
        if (self.second_target_handover_observed
                and self._full_target_points(self.second_target, scan, self.second_track_id) is not None):
            return self._begin_second_orbit(scan, now, control)
        _, rejection = self._right_exit_target(scan, now)
        if rejection is not None:
            # Trial45: a single missing/ambiguous corridor frame only keeps the
            # adopted command within the last observation lease; no new
            # steering authority, and expiry still coasts.
            lease_receive, lease_publication = self._right_exit_last_receive, self._right_exit_last_publication
            if (rejection in ('right_exit_corridor_missing', 'right_exit_corridor_ambiguous')
                    and _number(lease_receive) and _number(lease_publication)
                    and now-lease_receive < SCAN_AGE_S
                    and (scan['at_ms']-lease_publication)/1000 < SCAN_AGE_S):
                self.steering_target = self.servo
                self.reason = 'route_corridor_' + rejection.rsplit('_', 1)[1] + '_hold'
                return self._result(now)
            self._begin_recovery_hold(rejection, now, control)
            self._slew(now)
            return self._result(now)
        if self._corridor_body_gap(self.right_exit_geometry) < 0.:
            # Model overlap only; a small model margin keeps pursuing the
            # center line (away from the near board) with proximity avoidance.
            self._begin_recovery_hold('route_corridor_body_margin_unavailable', now, control)
            return self._result(now)
        if not self._route_adopted(control, now):
            return self._result(now)
        self._second_role_observation(scan, now)
        if self.second_target is not None and self.second_target.get('confirmed') is True:
            return self._begin_second_orbit(scan, now, control)
        c = self.right_exit_geometry
        self._lane_left_limit(scan, c['heading_left_rad'], c.get('heading_rate_rad_s'))
        error = (c['heading_left_rad']+math.atan2(c['center_offset_left_m'], c['lookahead_m'])
                 + ROUTE_ALIGN_DAMPING_S*c.get('heading_rate_rad_s', 0.))
        target = max(NEUTRAL-55, min(NEUTRAL+55, NEUTRAL+round(LEFT_TRIAL_GAIN*error)))
        natural = target
        if target < NEUTRAL and self._route_center_ack is None:
            target = NEUTRAL
        self.steering_target, self.natural_steering_target = target, natural
        self.reason = 'route_corridor_follow_current_geometry'
        self._slew(now)
        if self.servo > NEUTRAL:
            self._route_center_ack = None
        return self._result(now)

    def _clear_second_return(self):
        self.second_return_preparing = False
        self.second_return_evidence = None

    @staticmethod
    def _second_return_support_clear(points, servo):
        """Keep the complete support to the RIGHT of the proposed forward arc.

        Unlike the short all-obstacle path check, this checks every target
        point, including a target beyond that horizon. An arc passing on its
        wrong side is not a clockwise bypass, even if it misses the target.
        This uses the existing uncalibrated path model, not a physical proof.
        """
        curvature = (servo-NEUTRAL)/ROUTE_PWM_PER_CURVATURE
        if curvature > 0:
            return False
        half = SIDE_BODY_EXTENT_M+MANEUVER_BODY_CLEARANCE_M
        for x, y in points:
            if abs(curvature) < 1e-9:
                along, right = x, -y
            else:
                radius = -1/curvature
                along = radius*math.atan2(x, radius+y)
                right = radius-math.hypot(x, radius+y)
            if along+REAR_BODY_EXTENT_M >= 0 and right < half:
                return False
        return True

    def _lane_left_limit(self, scan, heading, rate):
        """Left PWM allowed by the lane-heading limit, predicted one lag ahead.

        Round 59 (Claude). Also bounds a task-left path correction in this
        frame (trial58 442/445: the path check raised 1612/1625 to 1632/1646
        while the car was already yawing 72 deg/s toward the left board).
        """
        predicted = heading+(rate*SECOND_STEER_LAG_S if rate is not None and rate < 0 else 0.)
        limit = NEUTRAL+round(SECOND_HEADING_HOLD_GAIN*(predicted+SECOND_BYPASS_MAX_HEADING_RAD))
        self._task_left_upper = (scan.get('seq'), limit)
        return predicted, limit

    def _current_lane(self, scan, now):
        """This frame's corridor measurement, without keeping its identity."""
        saved = (self.right_exit_geometry, self._right_exit_last_receive,
                 self._right_exit_last_publication)
        self.right_exit_geometry = None
        self._right_exit_last_receive = self._right_exit_last_publication = None
        try:
            target, rejection = self._right_exit_target(scan, now)
            corridor = copy.deepcopy(self.right_exit_geometry)
        finally:
            (self.right_exit_geometry, self._right_exit_last_receive,
             self._right_exit_last_publication) = saved
        return (None, None) if rejection is not None else (target, corridor)

    def _second_lane_bypass(self, scan, now, points):
        """Lane-frame bypass target left of the second support, or None."""
        _, lane = self._current_lane(scan, now)
        self._second_lane_heading = None if lane is None else lane['heading_left_rad']
        previous = getattr(self, '_second_lane_previous', None)
        self._second_lane_rate = None
        if lane is not None:
            row = (scan['at_ms']/1000, scan['received_at'], lane['heading_left_rad'])
            if previous is not None and .05 <= row[0]-previous[0] <= SCAN_AGE_S and .05 <= row[1]-previous[1] <= SCAN_AGE_S:
                rate = _wrap(row[2]-previous[2])/(row[0]-previous[0])
                if abs(rate) <= SECOND_MAX_HEADING_RATE_RAD_S:
                    self._second_lane_rate = rate
            self._second_lane_previous = row
        else:
            self._second_lane_previous = None
        if lane is None or abs(lane['heading_left_rad']) > SECOND_LANE_MAX_HEADING_RAD:
            return None
        heading, offset, width = lane['heading_left_rad'], lane['center_offset_left_m'], lane['width_m']
        c, s = math.cos(heading), math.sin(heading)
        along = [c*x+s*y for x, y in points]
        normal = [-s*x+c*y for x, y in points]
        wanted = max(normal)+SECOND_BYPASS_SUPPORT_GAP_M
        room = offset+width/2-(SIDE_BODY_EXTENT_M+MANEUVER_BODY_CLEARANCE_M)
        desired = min(wanted, room)
        ahead = min(along)
        lookahead = (max(FRONT_BODY_EXTENT_M+REAR_BODY_EXTENT_M, min(SECOND_LANE_LOOKAHEAD_M, ahead))
                     if ahead > 0 else SECOND_LANE_LOOKAHEAD_M)
        error = heading+math.atan2(desired, lookahead)
        target = max(RIGHT_EXIT_MIN_PWM, min(SERVO_MAX, NEUTRAL+round(LEFT_TRIAL_GAIN*error)))
        return {'source_seq': scan['seq'], 'lane_heading_left_rad': heading,
                'lane_center_offset_left_m': offset, 'lane_width_m': width,
                'support_left_normal_m': max(normal), 'support_nearest_along_m': ahead,
                'wanted_offset_m': wanted, 'lane_room_left_m': room, 'desired_offset_m': desired,
                'lookahead_m': lookahead, 'heading_error_rad': error, 'nominal_target_pwm': target,
                'scope': 'current_lane_frame_bypass', 'swept_path_certified': False}

    def _second_return_target(self, scan, now, points, bypass_target):
        """Prepare from present support/corridor geometry, before final pass.

        A confirmed support already clear of the right flank can stop asking
        for a left waypoint ahead of the car. Measure the current corridor
        without renewing or inheriting the older first-exit corridor lease.
        Missing geometry may release left, but cannot create a new right step.
        """
        if min(-p[1] for p in points)-SIDE_BODY_EXTENT_M < MANEUVER_BODY_CLEARANCE_M:
            self._clear_second_return()
            return None
        saved = (self.right_exit_geometry, self._right_exit_last_receive,
                 self._right_exit_last_publication)
        self.right_exit_geometry = None
        self._right_exit_last_receive = self._right_exit_last_publication = None
        try:
            native, _ = self._right_exit_target(scan, now)
            corridor = copy.deepcopy(self.right_exit_geometry)
        finally:
            (self.right_exit_geometry, self._right_exit_last_receive,
             self._right_exit_last_publication) = saved
        if native is not None and native > NEUTRAL:
            # A different live layout still calls for left; no memorized
            # turn direction or permanent release latch overrides it.
            self._clear_second_return()
            return None
        candidates = []
        if native is not None:
            # Preserve a valid clockwise request already made by the support
            # pursuit law. A straight corridor must not suppress that orbit.
            requested = min(native, bypass_target)
            all_points = self._path_points(scan)
            candidates = [pwm for pwm in {*ROUTE_PATH_RIGHT_CANDIDATES, requested, NEUTRAL}
                          if requested <= pwm <= NEUTRAL
                          and self._second_return_support_clear(points, pwm)
                          and all_points
                          and self._path_free_distance(all_points, pwm) >= ROUTE_PATH_LOOKAHEAD_M]
        if not candidates:
            # Do not reuse old geometry to increase right steering. The
            # existing near-body and stale-frame gates remain authoritative.
            if self.second_return_preparing:
                return min(self.servo, NEUTRAL)
            return None
        choice = min(candidates, key=lambda pwm: abs(pwm-requested))
        self.second_return_preparing = True
        self.second_return_evidence = {
            'source_seq': scan['seq'], 'source_at_ms': scan['at_ms'],
            'received_at': scan['received_at'], 'track_id': self.second_track_id,
            'support_bins': list(self.second_target['support_bins']),
            'observed_frontmost_x_m': max(p[0] for p in points),
            'observed_right_body_gap_m': min(-p[1] for p in points)-SIDE_BODY_EXTENT_M,
            'corridor': corridor, 'corridor_target_pwm': native,
            'support_bypass_target_pwm': bypass_target, 'requested_target_pwm': requested,
            'selected_target_pwm': choice, 'clear_candidates': sorted(candidates),
            'all_obstacle_lookahead_m': ROUTE_PATH_LOOKAHEAD_M,
            'basis': 'current_complete_support_right_of_current_corridor_return_arc',
            'pass_proven': False, 'swept_path_certified': False,
            'physical_curvature_calibrated': False}
        return choice

    def _begin_second_orbit(self, scan, now, control):
        self._clear_second_return()
        self.second_track_id = self.second_target['track_id']
        self.second_tracker.enable_close_maintenance(self.second_track_id)
        self._second_role_anchor = copy.deepcopy(self.second_target)
        self._second_recovering = False
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

    def _second_orbit_update(self, scan, now, control):
        target = self.second_target
        held = getattr(self.second_tracker, 'held', None)
        if (target is None and isinstance(held, dict) and held.get('track_id') == self.second_track_id
                and now-self.second_orbit_since < ORBIT_ENTRY_MAX_S):
            # Tolerant tracker kept the identity through a missing frame:
            # keep the selected command, or continue an adopted left release.
            if self.second_return_preparing and not self._route_adopted(control, now):
                return self._result(now)
            self.steering_target = (min(self.servo, NEUTRAL)
                                    if self.second_return_preparing else self.servo)
            self.natural_steering_target = self.steering_target
            self.reason = 'second_target_missing_frame_hold'
            self._slew(now)
            return self._result(now)
        lost = (target is None or target.get('track_id') != self.second_track_id
                or not target.get('confirmed'))
        if now-self.second_orbit_since >= ORBIT_ENTRY_MAX_S or lost:
            self._clear_second_return()
            # Self-recovery: go back to corridor following, which re-acquires
            # the second support (or continues) from the current position.
            self.route_stage = 'corridor_follow'
            # The second orbit does not refresh the corridor estimate; restart
            # its identity from the next fresh observation (no stale-identity
            # rejection and recovery pause on return).
            self.right_exit_geometry = None
            self._right_exit_last_receive = self._right_exit_last_publication = None
            if lost:
                self._start_second_identity_recovery()
            else:
                self.second_tracker = self._new_second_tracker()
                self._second_recovering = False
            self.second_target = None
            self.second_center_ack = None
            self.second_target_handover_observed = False
            self.reason = ('second_target_entry_timeout_back_to_corridor' if not lost
                           else 'second_target_lost_back_to_corridor')
            self.steering_target = self.servo
            self._route_event('second_target_recovery', scan, control,
                              {'source_seq': scan['seq'], 'reason': self.reason})
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
            # Trial46 operator: after the second target turn on toward the
            # traffic light and end facing it. Direction comes from free space.
            self.route_stage, self.light_since = 'light_turn', now
            self.second_return_preparing = False
            self._light_history.clear()
            self._light_heading = self._exit_heading  # continue the next-lane window
            self.steering_target = self.servo
            self.reason = 'light_turn_started_keep_adopted_command'
            self._route_event('light_turn', scan, control, {'source_seq': scan['seq']})
            return self._result(now)
        if not passed:
            self._second_pass_history.clear()
        if points is None:
            # Partial boundaries maintain only the already selected command;
            # they cannot define a new bypass point or prove a pass.
            self.steering_target = (min(self.servo, NEUTRAL)
                                    if self.second_return_preparing else self.servo)
            self.natural_steering_target = self.steering_target
            self.reason = 'second_target_partial_support_hold'
            if self.second_return_preparing:
                self._slew(now)
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
        curvature = 2*waypoint[1]/(waypoint[0]**2+waypoint[1]**2)
        # Trial43: the bypass point left of a left-front support needs left;
        # right still requires a current actual center ACK after any left.
        nominal = max(RIGHT_EXIT_MIN_PWM, min(SERVO_MAX,
            NEUTRAL+round(SECOND_CURVATURE_GAIN_PWM_M*curvature)))
        self.second_feedback = {'source_seq': scan['seq'], 'source_at_ms': scan['at_ms'],
            'current_support_left_edge_y_m': left_edge, 'current_support_frontmost_x_m': front,
            'current_support_nearest_x_m': near, 'bypass_support_gap_m': SECOND_BYPASS_SUPPORT_GAP_M,
            'relative_bypass_point_left_m': waypoint, 'bearing_left_rad': bearing,
            'pursuit_curvature_per_m': curvature,
            'nominal_target_pwm': nominal,
            'scope': 'current_support_relative_point_trial',
            'swept_path_certified': False, 'physical_curvature_calibrated': False}
        lane = self._second_lane_bypass(scan, now, points)
        self.second_feedback['lane_bypass'] = lane
        if lane is not None:
            # Early offset from the lane frame; the support pursuit still
            # governs a near support whose offset is not yet taken (trial48:
            # .9 m ahead needs >= 1680). Both relax once the car is on the line.
            nominal = max(nominal, lane['nominal_target_pwm'])
            self.second_feedback['nominal_target_pwm'] = nominal
            self.second_feedback['scope'] = 'max_of_support_pursuit_and_lane_frame_bypass'
        lane_heading = getattr(self, '_second_lane_heading', None)
        if lane_heading is not None and nominal > NEUTRAL:
            predicted, limit = self._lane_left_limit(scan, lane_heading, getattr(self, '_second_lane_rate', None))
            self.second_feedback['predicted_lane_heading_rad'] = predicted
            if nominal > limit:
                nominal = max(RIGHT_EXIT_MIN_PWM, limit)
                self.second_feedback['heading_limited_target_pwm'] = nominal
                self.second_feedback['nominal_target_pwm'] = nominal
        # Round 58 (Claude): once the support is beside/behind the body front
        # with the right clearance, the rest of the clockwise orbit is the turn
        # into the next lane. Trial57 576-581: the return target went back to
        # the old lane axis (1413 -> 1486) while that lane ended in a board
        # ahead; the light turn then started late. Take the free-space target
        # when it is further right (task rule: second support clockwise);
        # support clearance, path check and the center ACK still apply.
        right_gap = min(-p[1] for p in points)-SIDE_BODY_EXTENT_M
        if front <= FRONT_BODY_EXTENT_M and right_gap >= MANEUVER_BODY_CLEARANCE_M:
            space = self._free_space(scan, self._exit_heading)
            if space is not None:
                self._exit_heading = space['best_heading_rad']
                exit_target = self._free_space_target(space)
                self.second_feedback['next_lane_free_space'] = {
                    'best_heading_rad': space['best_heading_rad'], 'free_ahead_m': space['free_ahead_m'],
                    'target_pwm': exit_target, 'applied': exit_target < nominal}
                if exit_target < nominal:
                    nominal = exit_target
                    self.second_feedback['nominal_target_pwm'] = nominal
        was_preparing = self.second_return_preparing
        returning = self._second_return_target(scan, now, points, nominal)
        if returning is not None:
            nominal = returning
            if not was_preparing:
                self._route_event('second_return_prepare', scan, control, self.second_return_evidence)
        self.natural_steering_target = nominal
        if nominal < NEUTRAL and (self.second_center_ack is None or self.servo > NEUTRAL):
            self.steering_target = NEUTRAL
            self.reason = 'second_target_waiting_center_ack'
        else:
            self.steering_target = nominal
            self.reason = ('second_target_left_of_support_entry' if nominal > NEUTRAL
                           else 'second_target_clockwise_bounded_entry')
        if self.second_return_preparing:
            self.reason = ('second_target_return_waiting_center_ack'
                           if nominal < NEUTRAL and self.steering_target == NEUTRAL
                           else 'second_target_return_current_geometry'
                           if self.second_return_evidence['source_seq'] == scan['seq']
                           else 'second_target_return_geometry_hold')
        self._slew(now)
        return self._result(now)

    @staticmethod
    def _free_space(scan, previous_heading=None):
        """Deepest free direction for the measured body plus clearance.

        Polar footprint: each known return blocks the headings whose body
        corridor (half width = side extent + .08 m) it overlaps, at its
        projected distance. A front null is never free: it blocks at its
        nearer known neighbour. Returns None without any usable return.
        """
        ranges = list(scan['ranges'])
        half = SIDE_BODY_EXTENT_M+MANEUVER_BODY_CLEARANCE_M
        step = LIGHT_HEADINGS_DEG[1]-LIGHT_HEADINGS_DEG[0]
        free = [LIGHT_FREE_HORIZON_M]*len(LIGHT_HEADINGS_DEG)
        left_clear = right_clear = None
        usable = any(r is not None for r in ranges)
        # Fill front nulls conservatively before the footprint pass.
        reach = LIGHT_HEADINGS_DEG[-1]+10
        start = None
        for angle in range(-reach, reach+2):
            missing = angle <= reach and ranges[(-angle) % 360] is None
            if missing and start is None:
                start = angle
            elif not missing and start is not None:
                edges = [ranges[(-(start-1)) % 360], ranges[(-angle) % 360]]
                bracketed = angle-start <= LIGHT_BRIDGE_MAX_BINS and None not in edges
                fill = min(edges) if bracketed else LIGHT_UNKNOWN_BLOCK_M
                for unknown in range(start, angle):
                    ranges[(-unknown) % 360] = fill
                start = None
        for index, r in enumerate(ranges):
            if r is None:
                continue
            bearing = math.degrees(_wrap(-math.radians(index)))  # left positive
            x, y = r*math.cos(math.radians(bearing)), r*math.sin(math.radians(bearing))
            if -REAR_BODY_EXTENT_M <= x <= 1.:
                if y > SIDE_BODY_EXTENT_M:
                    left_clear = y if left_clear is None else min(left_clear, y)
                elif y < -SIDE_BODY_EXTENT_M:
                    right_clear = -y if right_clear is None else min(right_clear, -y)
            if r >= LIGHT_FREE_HORIZON_M+half:
                continue
            spread = 90. if r <= half else math.degrees(math.asin(half/r))
            first = max(0, math.ceil((bearing-spread-LIGHT_HEADINGS_DEG[0])/step))
            last = min(len(LIGHT_HEADINGS_DEG)-1, math.floor((bearing+spread-LIGHT_HEADINGS_DEG[0])/step))
            for k in range(first, last+1):
                delta = math.radians(LIGHT_HEADINGS_DEG[k]-bearing)
                along = r*math.cos(delta)
                if along > 0 and abs(r*math.sin(delta)) <= half and along < free[k]:
                    free[k] = along
        if not usable:
            return None
        deepest = max(free)
        # Follow the gap: aim at the middle of a contiguous near-deepest window
        # (in a lane that is its axis). Keep the window holding the previous
        # choice so two similar openings do not flip the command; otherwise
        # take the widest window.
        windows, run = [], []
        for k, f in enumerate(free):
            if f >= .9*deepest:
                run.append(k)
            elif run:
                windows.append(run)
                run = []
        if run:
            windows.append(run)
        chosen = None
        if previous_heading is not None:
            previous = math.degrees(previous_heading)
            chosen = next((w for w in windows
                           if LIGHT_HEADINGS_DEG[w[0]]-step <= previous <= LIGHT_HEADINGS_DEG[w[-1]]+step), None)
        if chosen is None:
            chosen = max(windows, key=len)
        best = math.radians((LIGHT_HEADINGS_DEG[chosen[0]]+LIGHT_HEADINGS_DEG[chosen[-1]])/2)
        return {'best_heading_rad': best, 'deepest_free_m': deepest,
                'free_ahead_m': free[LIGHT_HEADINGS_DEG.index(0)],
                'left_clear_m': left_clear, 'right_clear_m': right_clear,
                'center_error_m': (None if left_clear is None or right_clear is None
                                   else (left_clear-right_clear)/2)}

    def _end_light_stage(self, reason, now, control):
        self.begin_coast(reason, now)
        if (self.phase == 'coast' and control.get('servo') == self.servo
                and control.get('command_acked') is True):
            # Never recenter while rolling; the final command is ACKed.
            self.route_end_coast_servo = self.servo
            self.steering_target = self.servo
        self._slew(now)
        return self._result(now)

    @staticmethod
    def _free_space_target(space):
        """Light-turn free-space law: deepest free heading, centered when nearly lined up."""
        best, center = space['best_heading_rad'], space['center_error_m']
        center_term = (math.atan2(center, 1.5)
                       if center is not None and abs(best) <= math.radians(20) else 0.)
        return max(RIGHT_EXIT_MIN_PWM, min(SERVO_MAX, NEUTRAL+round(LEFT_TRIAL_GAIN*(best+center_term))))

    def _lane_front_range(self, scan):
        """Nearest return straight ahead (+-LANE_FRONT_HALF_BINS deg), or None."""
        values = [scan['ranges'][i % 360] for i in range(-LANE_FRONT_HALF_BINS, LANE_FRONT_HALF_BINS+1)]
        values = [r for r in values if _number(r)]
        return min(values) if values else None

    def _lane_speed(self, scan, ahead):
        """Closing speed on the board ahead: median of 3 consecutive scan rates."""
        history = self._lane_ranges
        if ahead is None:
            history.clear()
            return None
        row = (scan['at_ms']/1000, scan['received_at'], ahead)
        if history:
            dt = row[0]-history[-1][0]
            rate = (history[-1][2]-ahead)/dt if dt > 0 else None
            if (not .05 <= dt <= SCAN_AGE_S+.05 or not .05 <= row[1]-history[-1][1] <= SCAN_AGE_S+.05
                    or rate is None or not -.4 <= rate <= 3.):
                history.clear()
        history.append(row)
        del history[:-4]
        if len(history) < 3:
            return None
        rates = sorted((a[2]-b[2])/(b[0]-a[0]) for a, b in zip(history, history[1:]))
        return max(0., rates[len(rates)//2])

    def _begin_light_lane(self, scan, now, control, evidence):
        self.light_alignment_evidence = evidence
        self._route_event('light_aligned', scan, control, evidence)
        self.route_stage = 'light_lane'
        self.lane_motor = ROUTE_SLOW_MOTOR  # speed history from the aligned frames is kept
        self._route_event('light_lane', scan, control, {'source_seq': scan['seq']})
        self.steering_target = self.servo
        self.reason = 'light_lane_started_keep_adopted_command'
        return self._result(now)

    def _light_lane_update(self, scan, now, control):
        """Follow the light lane slowly, then stop inside the finish zone.

        Round 58 (Claude). No traffic-light vision yet: the lane is only driven
        slowly. Steering follows the measured lane (corridor law, +-55 PWM like
        corridor following), else the free-space law. Right needs a center ACK.
        """
        if not self._route_adopted(control, now):
            return self._result(now)
        _, lane = self._current_lane(scan, now)
        space = None
        if lane is not None and abs(lane['heading_left_rad']) <= SECOND_LANE_MAX_HEADING_RAD:
            lookahead = max(FRONT_BODY_EXTENT_M+REAR_BODY_EXTENT_M, min(ROUTE_ALIGN_LOOKAHEAD_MAX_M, lane['width_m']))
            error = lane['heading_left_rad']+math.atan2(lane['center_offset_left_m'], lookahead)
            natural = max(NEUTRAL-55, min(NEUTRAL+55, NEUTRAL+round(LEFT_TRIAL_GAIN*error)))
            source = 'measured_lane'
        else:
            space = self._free_space(scan, self._light_heading)
            if space is None:
                self._begin_recovery_hold('light_lane_geometry_unavailable', now, control)
                return self._result(now)
            self._light_heading = space['best_heading_rad']
            natural = self._free_space_target(space)
            source = 'free_space'
        target = natural
        if target < NEUTRAL and (self._route_center_ack is None or self.servo > NEUTRAL):
            target = NEUTRAL  # right only after a current actual center ACK
        ahead = self._lane_front_range(scan)
        speed = self._lane_speed(scan, ahead)
        v = LANE_SPEED_UNKNOWN_MPS if speed is None else speed
        stop_at = (FRONT_BODY_EXTENT_M+LANE_FINISH_GAP_M
                   + v*v/(2*LANE_COAST_DECEL_MPS2)+v*LANE_REACTION_S)
        self.lane_feedback = {'source_seq': scan['seq'], 'steering_source': source,
            'nominal_target_pwm': natural, 'lane': lane, 'free_space': space,
            'front_range_m': ahead, 'speed_mps': speed, 'speed_basis': 'closing_rate_on_board_ahead',
            'finish_cut_range_m': stop_at, 'motor_before': self.lane_motor,
            'swept_path_certified': False, 'traffic_light_vision': False}
        if ahead is not None and ahead <= stop_at:
            self.finish_evidence = {**self.lane_feedback, 'received_at': scan['received_at'],
                'basis': 'board_ahead_within_body_gap_plus_neutral_coast_distance',
                'finish_zone_certified': False}
            self._route_event('finish_stop', scan, control, self.finish_evidence)
            return self._end_light_stage('route_finish_zone_stop', now, control)
        if speed is None:
            self.lane_motor = ROUTE_SLOW_MOTOR
        elif speed < LANE_SPEED_RESTART_MPS:
            self.lane_motor = TRIAL_MOTOR
        elif speed < LANE_SPEED_LOW_MPS:
            self.lane_motor = ROUTE_SLOW_MOTOR
        elif speed > LANE_SPEED_HIGH_MPS:
            self.lane_motor = NEUTRAL
        self.lane_feedback['motor'] = self.lane_motor
        self.steering_target, self.natural_steering_target = target, natural
        self.reason = 'light_lane_follow_slow'
        self._slew(now)
        return self._result(now)

    def _light_update(self, scan, now, control):
        if now-self.light_since+1e-9 >= LIGHT_STAGE_MAX_S:
            self._begin_recovery_hold('light_stage_timeout', now, control)
            return self._result(now)
        if not self._route_adopted(control, now):
            return self._result(now)
        space = self._free_space(scan, self._light_heading)
        if space is None:
            self._begin_recovery_hold('light_free_space_unavailable', now, control)
            return self._result(now)
        best = space['best_heading_rad']
        self._light_heading = best
        center = space['center_error_m']
        _, lane = self._current_lane(scan, now)
        lane_aligned = (lane is not None and abs(lane['heading_left_rad']) <= LANE_ALIGNED_HEADING_RAD
                        and self._corridor_body_gap(lane) >= MANEUVER_BODY_CLEARANCE_M)
        aligned = lane_aligned or (abs(best) <= LIGHT_ALIGNED_HEADING_RAD
                                   and space['free_ahead_m'] >= LIGHT_ALIGNED_FREE_M
                                   and center is not None and abs(center) <= LIGHT_CENTER_TOLERANCE_M)
        if aligned:
            # Heading is steady while aligned: start measuring the closing speed
            # now so the lane (and its finish cut) begins with a known speed.
            self._lane_speed(scan, self._lane_front_range(scan))
        else:
            self._lane_ranges.clear()
        if aligned and self._mature_observation(self._light_history, scan, .30):
            # Round 58: aligned with the light lane -> drive it slowly to the
            # finish (was: terminal coast here).
            return self._begin_light_lane(scan, now, control, {**space,
                'source_seq': scan['seq'], 'source_at_ms': scan['at_ms'],
                'received_at': scan['received_at'], 'frames': len(self._light_history),
                'measured_lane': lane, 'basis': ('measured_lane_aligned' if lane_aligned
                                                 else 'long_free_lane_ahead_and_balanced_side_clearance'),
                'camera_facing_certified': False})
        if not aligned:
            self._light_history.clear()
        # Toward the deepest free direction; once nearly lined up, also center.
        target = self._free_space_target(space)
        natural = target
        if target < NEUTRAL and (self._route_center_ack is None or self.servo > NEUTRAL):
            target = NEUTRAL  # right only after a current actual center ACK
        self.steering_target, self.natural_steering_target = target, natural
        self.light_feedback = {**space, 'source_seq': scan['seq'], 'nominal_target_pwm': natural,
                               'scope': 'free_space_follow_trial', 'swept_path_certified': False}
        self.reason = 'light_turn_follow_free_space'
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
                    and control.get('motor') in DRIVE_MOTORS and control.get('servo') == self.servo):
                # Route mode: no orbit-entry clock (trial50); the cumulative
                # drive budget and the service deadline still bound it. A
                # repeated frame reuses the last measured release target.
                cached = self._exit_release_cache
                self.steering_target = self.natural_steering_target = (
                    max(NEUTRAL, min(self.servo, cached[1])) if cached is not None else NEUTRAL)
                self.reason = 'first_target_pass_preparing_neutral'
                self._slew(now)
            return self._result(now)
        if self.continue_route and self.route_stage == 'light_turn':
            return self._light_update(scan, now, control)
        if self.continue_route and self.route_stage == 'light_lane':
            return self._light_lane_update(scan, now, control)
        if self.continue_route and self.route_stage == 'corridor_follow':
            return self._corridor_follow_update(scan, now, control)
        if self.continue_route and self.route_stage == 'second_orbit':
            return self._second_orbit_update(scan, now, control)
        if self.first_pass_evidence is not None:
            return self._right_exit_update(scan, now, control)
        held = getattr(self.target_tracker, 'held', None)
        if (self.continue_route and self.compact_target is None and self.handover_observed
                and isinstance(held, dict) and held.get('track_id') == self.orbit_track_id):
            # Tolerant tracker kept the orbited identity through a missing
            # frame (trial50/51 flicker): keep the current command this frame,
            # except that a support last seen at/behind the body front (or an
            # already latched release) keeps releasing toward the exit
            # (trial53: holding 1691 here over-rotated the car 66 deg past the
            # lane). The body-front window is kept, not cleared.
            last_front = max((p[0] for p in self._held_support_points(held)), default=None)
            if self.first_pass_preparing or (last_front is not None and last_front <= FRONT_BODY_EXTENT_M):
                self.steering_target = self._exit_release_target(scan, now)
                self.reason = 'first_target_missing_frame_release'
            else:
                self.steering_target = self.servo
                self.reason = 'first_target_missing_frame_hold'
            self._slew(now)
            return self._result(now)
        if not self._usable_target(self.compact_target, scan):
            self._clear_exit_prepare_window()
            self._capture_compact_loss(scan, now, control, 'compact_target_lost_or_ambiguous')
            # Losing object identity removes the drive request, not the
            # already adopted left steering. Keep that command only in the
            # existing bounded neutral coast; never reacquire powered motion.
            if self.continue_route and self.begin_quality_wait(now, control, scan=scan):
                # Self-recovery: hold at neutral and re-find the same support.
                self.reason = 'route_orbit_target_recovery_wait'
            elif not self._begin_adopted_left_coast('compact_target_lost_or_ambiguous', now, control):
                self.begin_coast('compact_target_lost_or_ambiguous', now)
            self._slew(now)
            return self._result(now)
        just_handed_over = False
        if not self.handover_observed:
            # Continue the adopted left command across normal stage handover.
            # A target never bypasses an ACK, neutral/protection stop or re-arms
            # the bridge. This is command feedback, not physical wheel angle.
            adopted = (control.get('motor') in (DRIVE_MOTORS if self.continue_route else (TRIAL_MOTOR,))
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
            self._route_left_hold_since = None
            if self.continue_route:
                # Trial54: the car passed the support while the tracker lost it
                # beside the body; it reappeared behind-left (128-163 deg) where
                # no new identity could be acquired, so the pass was never
                # observed and the wait stalled. After handover the first
                # support is anywhere on the left (task rule: anticlockwise),
                # so re-acquisition covers the rear-left too.
                self.target_tracker.widen_acquisition((math.radians(45), math.pi))
            just_handed_over = True
            self.orbit_since = now
            self.orbit_reference_range_m = self.compact_target['range_m']
            self.orbit_bias_pwm = control['servo']
            self.orbit_left_entry_boost = self.compact_target['bearing_left_rad'] < ORBIT_BEARING_RAD
            self.handover_control = {key: control[key] for key in ('motor', 'servo', 'tick', 'seq')}
            self.handover_control.update(command_acked=True, physical_steering_confirmed=False,
                target_source_seq=scan['seq'], target_source_at_ms=scan['at_ms'])
            self.orbit_track_id = self.compact_target['track_id']
            if self.continue_route:
                self.target_tracker.enable_close_maintenance(self.orbit_track_id)
            self.wall_ambiguity_hold = False
            self.geometry_missing = True
            self.alignment_count = 0
            self.alignment_evidence = None
            self.alignment_publication = self.alignment_receive = None
        elif self.compact_target['track_id'] != self.orbit_track_id:
            self._clear_exit_prepare_window()
            self._capture_compact_loss(scan, now, control, 'compact_target_identity_changed')
            if self.continue_route and self.begin_quality_wait(now, control, scan=scan):
                self.reason = 'route_orbit_target_recovery_wait'
            elif not self._begin_adopted_left_coast('compact_target_identity_changed', now, control):
                self.begin_coast('compact_target_identity_changed', now)
            self._slew(now)
            return self._result(now)
        if not self.continue_route and now-self.orbit_since+1e-9 >= ORBIT_ENTRY_MAX_S:
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
            self.steering_target = self.natural_steering_target = self._exit_release_target(scan, now)
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
        self._avoid = self._route_proximity(scan) if self.continue_route and self.phase != 'locked' else None
        if self._avoid is not None:
            self.avoid_count += 1
            if len(self.avoid_evidence) < 40:
                self.avoid_evidence.append({**self._avoid, 'phase': self.phase, 'route_stage': self.route_stage,
                                            'servo': self.servo})
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
            # Corridor following while driving runs its own recovery plus
            # corridor proof on this scan (_second_role_observation); feeding
            # the tracker here first would consume the frame.
            corridor_drive = (self.route_stage == 'corridor_follow' and self.phase == 'drive'
                              and not self.quality_resume_pending)
            if self.continue_route and (self.route_stage == 'second_orbit'
                                        or (self._second_recovering and not corridor_drive)):
                self._update_second_tracking(scan, now)
        except (ValueError, TypeError, KeyError) as error:
            self.compact_target = None
            self.compact_target_error = type(error).__name__
        if isinstance(self.compact_target, dict) and _number(now):
            self._last_seen_compact = (now, copy.deepcopy(self.compact_target))
        if entry_stop is not None:
            # The new trial is target tracking, not the old optional stop input.
            self.lock('unsupported_maneuver_entry_stop')
            return self._result(self.last_now)
        if (self.continue_route and self.neutral_pause_kind is not None
                and (self.phase == 'quality_wait' or self.quality_resume_pending)):
            return self._neutral_pause_update(scan, lidar_age_s, now, control, safe=safe,
                presteer_wait=presteer_wait, quality_clear=quality_clear,
                resume_ready=quality_resume_ready)
        # A held quality coast (route mode may start one in the left turn)
        # only validates advancing scans/control and keeps its ACKed servo;
        # no geometry association can recenter it while still rolling.
        if (self.handover_observed
                or (self.phase == 'coast' and (self.wall_ambiguity_hold
                                               or self.quality_coast_servo is not None))
                or (self.phase == 'drive' and self._usable_target(self.compact_target, scan))):
            if self.handover_observed:
                self._route_left_hold_since = None
            return self._orbit_update(scan, lidar_age_s, now, control,
                                      safe=safe, presteer_wait=presteer_wait,
                                      quality_clear=quality_clear, quality_resume_ready=quality_resume_ready)
        if self._route_left_hold_since is not None and self.phase == 'drive':
            return self._route_left_hold_update(scan, lidar_age_s, now, control, safe, presteer_wait)
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
