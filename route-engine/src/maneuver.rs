//! ManeuverSequence (maneuver_sequence.py) with its TurnMotion base
//! (turn_motion.py) in one struct: the console only ever runs the route
//! sequence, so every virtual call resolves to the ManeuverSequence override
//! and `super()` calls name the TurnMotion version (`tm_*`). Attribute names
//! are the Python ones; `dump` returns them for state comparison.

use std::rc::Rc;

use crate::compact_target::{
    CompactTargetTracker, TrackerArgs, EXTENDED_MAX_RANGE_M, MIN_RANGE_M, TOLERANT_MAINTENANCE_MIN_POINTS,
    floats, valid_ranges,
};
use crate::dict;
use crate::helpers::*;
use crate::py::*;

// ---------------------------------------------------------------- constants

pub const ORBIT_ENTRY_MAX_S: f64 = 3.0;
pub const COAST_MAX_S: f64 = 5.0;
pub const PRESTEER_MAX_S: f64 = 8.0;
pub const DEFAULT_INITIAL_PWM: i64 = 1670;
pub const INITIAL_PWM_MIN: i64 = 1670;
pub const PRESTEER_PWM_STEP: i64 = 20;
pub const LEFT_RELEASE_PWM_STEP: i64 = 20;
pub const ORBIT_BEARING_RAD: f64 = PI / 2.;
pub const ORBIT_BEARING_GAIN: f64 = 100.0;
pub const ORBIT_RANGE_GAIN: f64 = 70.0;
pub const LEFT_TRIAL_GAIN: f64 = 180.0;
pub const MAX_TREND_RELEASE_FRACTION: f64 = 0.20;
pub const MAX_INNER_RELEASE_FRACTION: f64 = 0.20;
pub const MAX_EXIT_WIDTH_CHANGE_FRACTION: f64 = 0.10;
pub const RIGHT_EXIT_MAX_S: f64 = 3.0;
pub const ROUTE_EXIT_RELEASE_PWM_STEP: i64 = 40;
pub const ROUTE_EXIT_PREPARE_FRAMES: usize = 2;
pub const ROUTE_ENTRY_RELEASE_FRAMES: usize = 2;
pub const ROUTE_POST_PASS_PWM_STEP: i64 = 40;
pub const ROUTE_ORBIT_LEFT_PWM_STEP: i64 = ROUTE_POST_PASS_PWM_STEP;
pub const ROUTE_SLOW_MOTOR: i64 = MIN_FORWARD_PWM;
// Round 100: the open start lane launches at 1600 (Python's comment).
pub const START_LAUNCH_MOTOR: i64 = 1600;
pub const DRIVE_MOTORS: [i64; 3] = [TRIAL_MOTOR, ROUTE_SLOW_MOTOR, START_LAUNCH_MOTOR];
pub const POST_PASS_STAGES: [&str; 8] =
    ["right_align", "corridor_follow", "second_orbit", "light_turn", "light_lane", "open_transfer", "second_loop", "light_entry"];
pub const ROUTE_MAX_DRIVE_S: f64 = 40.0;
pub const ROUTE_ALIGN_LOOKAHEAD_MAX_M: f64 = 2.0;
pub const ROUTE_ALIGN_DAMPING_S: f64 = 0.30;
pub const LIGHT_STAGE_MAX_S: f64 = 4.0;
pub const LIGHT_FREE_HORIZON_M: f64 = 3.0;
pub const LIGHT_ALIGNED_FREE_M: f64 = 2.5;
pub const LIGHT_CENTER_TOLERANCE_M: f64 = 0.20;
pub const LIGHT_UNKNOWN_BLOCK_M: f64 = 0.60;
pub const QUALITY_WAIT_REBIND_M: f64 = 0.30;
pub const QUALITY_WAIT_REBIND_WINDOW_S: f64 = 0.60;
pub const ORBIT_REBIND_MAX_RANGE_M: f64 = 1.2;
pub const ROUTE_AVOID_GAP_M: f64 = 0.15;
pub const ROUTE_AVOID_LEFT_PWM: i64 = NEUTRAL + 150;
pub const ROUTE_AVOID_PWM_STEP: i64 = 40;
pub const ROUTE_WAIT_INSIDE_MARGIN_PWM: i64 = 60;
pub const ROUTE_LEFT_OVERSHOOT_HOLD_S: f64 = 1.0;
pub const ROUTE_PATH_LOOKAHEAD_M: f64 = 0.70;
pub const ROUTE_PATH_MARGIN_M: f64 = 0.04;
pub const ROUTE_PATH_RANGE_M: f64 = 2.;
pub const LIGHT_BRIDGE_MAX_BINS: i64 = 5;
pub const LANE_SPEED_RESTART_MPS: f64 = 0.15;
pub const LANE_SPEED_UNKNOWN_MPS: f64 = 1.3;
pub const LANE_MAX_CLOSING_MPS: f64 = 3.5;
pub const LANE_STOP_DECEL_MPS2: f64 = 0.5;
pub const LANE_SPEED_HOLD_S: f64 = 0.5;
pub const LANE_SPEED_HOLD_MARGIN_MPS: f64 = 0.1;
pub const ESC_LAG_S: f64 = 0.45;
pub const LANE_REACTION_BASE_S: f64 = 0.2;
pub const LANE_REACTION_S: f64 = LANE_REACTION_BASE_S + ESC_LAG_S;
pub const LANE_POWER_ACCEL_MPS2: f64 = 1.;
pub const LANE_LAUNCH_S: f64 = 0.4;
pub const LANE_FINISH_GAP_M: f64 = 0.15;
pub const LANE_STOP_DECEL_MIN_MPS2: f64 = 0.45;
pub const LANE_COAST_MIN_SPAN_S: f64 = 0.4;
pub const LANE_COAST_MIN_DROP_MPS: f64 = 0.1;
pub const LANE_COAST_DECEL_CAP_MPS2: f64 = 0.70;
pub const LANE_FINISH_ZONE_GAP_M: f64 = 0.75;
// Round 101: aim the light-lane coast at a stop with the whole car in the zone
// and push again while it still rolls (maneuver_sequence.py has the derivation).
pub const LANE_FINISH_BODY_GAP_M: f64 = 0.40;
pub const LANE_FINISH_MIN_GAP_M: f64 = 0.05;
pub const LANE_FINISH_TARGET_GAP_M: f64 = 0.25;
pub const LANE_TOPUP_BAND_M: f64 = 0.10;
pub const LANE_PUSH_ROLLING_MPS: f64 = 0.25;
pub const LANE_PUSH_ACCEL_MPS2: f64 = 0.35;
pub const LANE_PUSH_ACCEL_MAX_MPS2: f64 = 0.5;
pub const LANE_PUSH_LATE_ACCEL_MAX_MPS2: f64 = 1.;
pub const LANE_KICK_ACCEL_MAX_MPS2: f64 = 1.6;
pub const LANE_PUSH_MAX_MPS: f64 = 1.7;
pub const LANE_PUSH_MIN_S: f64 = 0.1;
pub const LANE_ESC_LAG_NOMINAL_S: f64 = 0.3;
pub const LANE_ESC_LAG_MIN_S: f64 = 0.2;
pub const LANE_ESC_SKEW_S: f64 = 0.05;
pub const LANE_AIM_REACTION_S: f64 = 0.1;
pub const LANE_SPEED_GUARD_MARGIN_MPS: f64 = 0.05;
pub const LANE_COAST_GUARD_RATIO: f64 = 0.9;
pub const LANE_COAST_TRACK_POINTS: usize = 12;
pub const LANE_COAST_DECEL_SE_MAX_MPS2: f64 = 0.08;
pub const LANE_KICK_TRAVEL_M: f64 = 0.45;
pub const LANE_KICK_REST_S: f64 = 1.;
pub const LANE_COAST_DECEL_FLOOR_MPS2: f64 = 0.2;
pub const LANE_KICK_FRAMES: i64 = 3;
pub const LANE_SPEED_CUT_MARGIN_MPS: f64 = 0.15;
pub const LANE_SPEED_BASELINE: usize = 2;
pub const LANE_SPEED_HISTORY: usize = 7;
pub const LANE_FRONT_HALF_BINS: i64 = 3;
pub const LIGHT_TURN_MIN_S: f64 = 1.0;
pub const SECOND_BYPASS_SUPPORT_GAP_M: f64 = MIN_RANGE_M + 0.05;
pub const SECOND_CURVATURE_GAIN_PWM_M: f64 = 205.;
pub const TASK_LEFT_BYPASS_STAGES: [&str; 4] = ["corridor_follow", "second_orbit", "open_transfer", "second_loop"];
pub const SECOND_RECOVERY_MAX_SLACK_S: f64 = 0.6;
pub const SECOND_LANDMARK_SLACK_S: f64 = 0.2;
pub const SECOND_LANE_LOOKAHEAD_M: f64 = 1.0;
pub const SECOND_HEADING_HOLD_GAIN: f64 = 450.;
pub const SECOND_STEER_LAG_S: f64 = 0.3;
pub const SECOND_ACQUISITION_MIN_POINTS: i64 = 3;
pub const SECOND_APPROACH_LEFT_M: f64 = SECOND_BYPASS_SUPPORT_GAP_M;
pub const SECOND_APPROACH_WALL_MARGIN_M: f64 = 0.15;
pub const SECOND_ORBIT_MAX_S: f64 = 6.0;
pub const ROUTE_RETURN_RELEASE_PWM_STEP: i64 = 80;
pub const PAIR_MIN_WIDTH_M: f64 = 0.65;
pub const PAIR_MAX_WIDTH_M: f64 = 2.5;
pub const PAIR_MIN_SHARED_SUPPORT_M: f64 = 0.35;
pub const PAIR_MERGE_OFFSET_M: f64 = 0.15;
pub const LANE_START_WIDTH_M: (f64, f64) = (0.65, 2.2);
pub const LANE_START_MIN_FRONT_M: f64 = 1.;
pub const LANE_START_SETTLE_S: f64 = 0.3;
pub const LANE_START_HOLD_PWM: i64 = 55;
pub const LANE_TRACK_SPEED_POINTS: usize = 5;
pub const LANE_WALL_PLAY_PWM: i64 = STEER_PLAY_PWM;
pub const OPEN_FIRST_MIN_ALONG_M: f64 = -1.2;
pub const FIRST_APPROACH_MAX_RANGE_M: f64 = 3.5;
pub const OPEN_HANDOVER_MAX_RANGE_M: f64 = 2.5;
pub const LANE_START_STOP_DECEL_MPS2: f64 = 0.5;
pub const LANE_START_MAX_MPS: f64 = 1.8; // round 99 (Python's comment)
pub const LANE_START_SEARCH_M: f64 = 1.6;
pub const LANE_START_PUSH_BAND_MPS: f64 = 0.4;
pub const START_LAUNCH_MAX_MPS: f64 = 0.2;
pub const START_LAUNCH_ACCEL_MPS2: f64 = 2.5;
pub const START_LAUNCH_SETTLE_S: f64 = 0.3;
pub const OPEN_LEFT_RADIUS_M: (f64, f64) = (0.90, 0.95);
pub const OPEN_RIGHT_RADIUS_M: (f64, f64) = (0.90, 1.40);
pub const OPEN_PROBE_QUANTUM_M: f64 = 0.05;
pub const OPEN_PROBE_MAX_RATE_RAD_S: f64 = 0.6;
pub const OPEN_SUPPORT_SKIP_M: f64 = 0.08;
pub const OPEN_LOOKAHEAD_M: f64 = 1.1;
pub const OPEN_CARROT_SUPPORT_GAP_M: f64 = 0.10;
pub const OPEN_APPROACH_CHECK_M: f64 = 1.2;
pub const OPEN_LOST_HOLD_S: f64 = 0.5;
pub const OPEN_SECOND_BAND_HALF_M: f64 = 1.6;
pub const OPEN_SECOND_BAND_REACH_M: f64 = 3.2;
pub const OPEN_SECOND_BEYOND_FIRST_M: f64 = 0.8;
pub const OPEN_SECOND_CONFIRM_GRACE_M: f64 = 0.5;
pub const OPEN_SECOND_LOOKOUT_RAD: f64 = PI / 2.;
pub const OPEN_HANDOFF_MIN_PROGRESS_RAD: f64 = 2. * PI / 3.;
pub const OPEN_LIGHT_HANDOFF_PROGRESS_RAD: f64 = PI / 2.;
pub const OPEN_HEADING_HOLD_PWM: i64 = 120;
pub const OPEN_CHANNEL_OFFSET_M: (f64, f64) = (0.40, 0.60);
pub const OPEN_CHANNEL_MAX_LATERAL_M: f64 = 3.5;
pub const OPEN_CHANNEL_SLOPE: f64 = 1.5;
pub const OPEN_PWM_STEP: i64 = 40;
pub const OPEN_STEER_LAG_S: f64 = 0.3;
pub const OPEN_RELEASE_PWM_STEP: i64 = ROUTE_RETURN_RELEASE_PWM_STEP;
pub const OPEN_RELEASE_PWM_PER_S: f64 = 550.;
pub const OPEN_PREDICT_SPEED_MPS: f64 = 1.3;
pub const OPEN_LANE_LOOKAHEAD_M: (f64, f64) = (0.8, 1.6);
pub const OPEN_LANE_WIDTH_M: (f64, f64) = (0.65, 2.2);
pub const OPEN_LANE_INSIDE_MARGIN_M: f64 = 0.05;
pub const OPEN_LANE_SHIFT_MIN_GAP_M: f64 = 0.20;
pub const OPEN_LINE_REACH_M: f64 = 6.;
pub const OPEN_STAGES: [&str; 3] = ["open_transfer", "second_loop", "light_entry"];
pub const OPEN_SUPPORT_MAX_DEPTH_M: f64 = 0.15;

pub fn route_align_heading_rad() -> f64 {
    radians(8.)
}
pub fn light_aligned_heading_rad() -> f64 {
    radians(6.)
}
pub fn second_lane_max_heading_rad() -> f64 {
    radians(45.)
}
pub fn second_bypass_max_heading_rad() -> f64 {
    radians(30.)
}
pub fn second_max_heading_rate_rad_s() -> f64 {
    radians(150.)
}
pub fn pair_max_angle_rad() -> f64 {
    radians(12.)
}
pub fn pair_merge_heading_rad() -> f64 {
    pair_max_angle_rad()
}
pub fn lane_start_max_heading_rad() -> f64 {
    radians(15.)
}
pub fn lane_side_wall_heading_rad() -> f64 {
    radians(5.)
}
pub fn first_acquire_bearing_rad() -> (f64, f64) {
    (radians(10.), radians(100.))
}
pub fn first_handover_bearing_rad() -> f64 {
    radians(45.)
}
pub fn open_exit_margin_rad() -> f64 {
    radians(25.)
}
pub fn open_late_exit_rad() -> f64 {
    radians(60.)
}
pub fn open_lane_parallel_rad() -> f64 {
    radians(12.)
}
pub fn open_lane_entry_heading_rad() -> f64 {
    radians(20.)
}
pub fn light_lane_entry_heading_rad() -> f64 {
    radians(20.)
}
pub fn open_probe_directions_rad() -> [f64; 5] {
    [radians(-90.), radians(-45.), radians(0.), radians(45.), radians(90.)]
}
pub fn right_exit_min_pwm() -> i64 {
    nominal_right_limit_pwm()
}
pub fn route_path_left_candidates() -> Vec<i64> {
    let mut v: Vec<i64> = (NEUTRAL..SERVO_MAX).step_by(20).collect();
    v.push(SERVO_MAX);
    v
}
pub fn route_path_right_candidates() -> Vec<i64> {
    vec![right_exit_min_pwm(), 1350, 1390, 1430, 1470]
}

pub fn motor_in(v: &V, motors: &[i64]) -> bool {
    // `x in (1550, 1560)`: Python equality, so 1550.0 or True-like values compare by value.
    match v {
        V::Int(i) => motors.contains(i),
        V::Float(x) => motors.iter().any(|&m| m as f64 == *x),
        V::Bool(b) => motors.contains(&(*b as i64)),
        _ => false,
    }
}

/// `value == int` with Python equality.
pub fn eq_i(v: &V, i: i64) -> bool {
    py_eq(v, &V::Int(i))
}

pub fn opt_v(v: Option<i64>) -> V {
    v.map(V::Int).unwrap_or(V::None)
}

/// One history row: (seq, at_ms, received_at).
pub type Row = (i64, i64, f64);

pub fn rows_v(rows: &[Row]) -> V {
    V::list(rows.iter().map(|r| V::list(vec![V::Int(r.0), V::Int(r.1), V::Float(r.2)])).collect())
}

pub fn scan_row(scan: &V) -> Row {
    (scan.idx("seq").i(), scan.idx("at_ms").i(), scan.idx("received_at").f())
}

/// An open-course loop slot.
#[derive(Clone, Debug, Default)]
pub struct Slot {
    pub free: Vec<(f64, f64)>,
    pub plan: V,
    pub key: Option<(V, f64)>,
    pub radius: Option<f64>,
    pub plan_inputs: Option<V>,
    pub approaching: Option<bool>,
    pub plan_source_seq: Option<i64>,
}

impl Slot {
    pub fn free_get(&self, theta: f64) -> Option<f64> {
        self.free.iter().find(|(t, _)| *t == theta).map(|(_, v)| *v)
    }
    pub fn free_set(&mut self, theta: f64, value: f64) {
        if let Some(slot) = self.free.iter_mut().find(|(t, _)| *t == theta) {
            slot.1 = value;
        } else {
            self.free.push((theta, value));
        }
    }
    pub fn dump(&self) -> V {
        let mut d = dict! {
            "free" => V::list(self.free.iter().map(|(t, v)| V::from(vec![*t, *v])).collect()),
            "plan" => self.plan.clone(),
            "key" => self.key.as_ref().map(|(i, r)| V::list(vec![i.clone(), V::Float(*r)])),
            "radius" => self.radius,
        };
        if let Some(p) = &self.plan_inputs {
            d.set("plan_inputs", p.clone());
        }
        if let Some(a) = self.approaching {
            d.set("approaching", a);
        }
        if let Some(s) = self.plan_source_seq {
            d.set("plan_source_seq", s);
        }
        d
    }
}

#[derive(Clone, Debug)]
pub struct Maneuver {
    // ---- TurnMotion
    pub started_at: f64,
    pub initial_presteer_pwm: Option<i64>,
    pub natural_steering_target: Option<i64>,
    pub max_drive_s: f64,
    pub max_presteer_s: f64,
    pub phase: &'static str,
    pub servo: i64,
    pub steering_target: i64,
    pub last_change: f64,
    pub last_now: f64,
    pub last_seq: Option<i64>,
    pub last_publication: Option<i64>,
    pub last_receive: Option<f64>,
    pub last_signature: V,
    pub corridor: V,
    pub geometry_mode: Option<&'static str>,
    pub outer_wall: V,
    pub turn_goal: V,
    pub last_geometry_receive: Option<f64>,
    pub last_geometry_publication: Option<i64>,
    pub last_geometry_seq: Option<i64>,
    pub geometry_missing: bool,
    pub selected: bool,
    pub settle_since: Option<f64>,
    pub settle_feedback_ticks: i64,
    pub last_control_tick: Option<i64>,
    pub last_control_seq: Option<i64>,
    pub drive_since: Option<f64>,
    pub presteer_ended_at: Option<f64>,
    pub last_error: Option<f64>,
    pub alignment_count: i64,
    pub alignment_publication: Option<i64>,
    pub alignment_receive: Option<f64>,
    pub observed_alignment: bool,
    pub entry_confirmed: bool,
    pub alignment_evidence: Option<&'static str>,
    pub reason: String,
    pub start_ready: bool,
    pub presteer_waiting: bool,
    pub incoming_endpoint: V,
    pub exit_width_change: V,
    // ---- ManeuverSequence
    pub continue_route: bool,
    pub target_tracker: CompactTargetTracker,
    pub compact_target: V,
    pub _route_left_hold_since: Option<f64>,
    pub left_overshoot_evidence: V,
    pub _last_seen_compact: Option<(f64, V)>,
    pub _last_confirmed_compact_target: V,
    pub compact_target_error: Option<String>,
    pub compact_loss_evidence: V,
    pub handover_observed: bool,
    pub orbit_since: Option<f64>,
    pub orbit_reference_range_m: V,
    pub orbit_bias_pwm: V,
    pub orbit_feedback: V,
    pub orbit_left_entry_boost: bool,
    pub _orbit_abeam_count: i64,
    pub _orbit_abeam_publication: V,
    pub _orbit_abeam_receive: Option<f64>,
    pub left_turn_feedback: V,
    pub left_handover_preparing: bool,
    pub handover_control: V,
    pub handover_wait_since: Option<f64>,
    pub orbit_track_id: V,
    pub coast_since: Option<f64>,
    pub wall_ambiguity_hold: bool,
    pub _last_actual_left_target: Option<i64>,
    pub _presteer_adoption_context: Option<(V, bool)>,
    pub _current_scan: V,
    pub inner_clearance: V,
    pub quality_coast_servo: Option<i64>,
    pub quality_wait_refusal: V,
    pub left_turn_pause_used: bool,
    pub left_turn_pause_evidence: V,
    pub left_turn_pause_geometry_error: Option<String>,
    pub neutral_pause_kind: Option<&'static str>,
    pub recovery_count: i64,
    pub recovery_evidence: V,
    pub post_pass_pause_evidence: V,
    pub quality_coast_reason: Option<String>,
    pub entry_bearing_required: bool,
    pub entry_bearing_released: bool,
    pub entry_bearing: V,
    pub _incoming_association: V,
    pub _entry_last_receive: V,
    pub _entry_last_publication: Option<i64>,
    pub _entry_ready_count: i64,
    pub _entry_ready_receive: V,
    pub _entry_ready_publication: Option<i64>,
    pub _entry_ready_observation_type: Option<String>,
    pub _entry_ready_history: Vec<Row>,
    pub adopted_presteer_pwm: Option<i64>,
    pub first_pass_preparing: bool,
    pub first_pass_evidence: V,
    pub first_pass_progress: V,
    pub _first_pass_history: Vec<V>,
    pub _first_exit_prepare_history: Vec<Row>,
    pub _quality_exit_prepare_ready: bool,
    pub quality_wait_release: V,
    pub quality_wait_previous_servo: Option<i64>,
    pub _wait_release_history: Vec<Row>,
    pub quality_wait_anchor: V,
    pub quality_wait_rebind: V,
    pub _wait_rebind_candidate: V,
    pub first_exit_prepare_evidence: V,
    pub right_exit_since: Option<f64>,
    pub right_exit_center_ack: V,
    pub right_exit_adoption_wait_since: Option<f64>,
    pub right_exit_geometry: V,
    pub _right_exit_last_receive: V,
    pub _right_exit_last_publication: V,
    pub route_stage: &'static str,
    pub route_events: V,
    pub quality_wait_since: Option<f64>,
    pub quality_wait_servo: Option<i64>,
    pub quality_wait_used: bool,
    pub quality_wait_evidence: V,
    pub quality_resume_pending: bool,
    pub quality_resume_since: Option<f64>,
    pub _quality_good_history: Vec<Row>,
    pub _quality_full_streak: i64,
    pub _quality_pass_history: Vec<Row>,
    pub _route_alignment_history: Vec<Row>,
    pub route_alignment_evidence: V,
    pub _route_center_ack: V,
    pub second_tracker: Option<CompactTargetTracker>,
    pub second_target: V,
    pub second_track_id: V,
    pub second_target_handover_observed: bool,
    pub second_center_ack: V,
    pub second_orbit_since: Option<f64>,
    pub second_pass_evidence: V,
    pub second_feedback: V,
    pub second_return_preparing: bool,
    pub second_return_evidence: V,
    pub route_end_coast_servo: Option<i64>,
    pub light_since: Option<f64>,
    pub light_turn_started: Option<f64>,
    pub _lane_steering: CorridorSteering,
    pub _lane_speed_last: Option<(f64, f64)>,
    pub _lane_pulse: i64,
    pub _light_history: Vec<Row>,
    pub _light_heading: Option<f64>,
    pub light_feedback: V,
    pub light_alignment_evidence: V,
    pub lane_motor: i64,
    pub _lane_coast_track: Vec<(f64, f64)>,
    pub _lane_coast_decel: Option<f64>,
    pub _lane_coast_decel_se: Option<f64>,
    pub _lane_ranges: Vec<(f64, f64, f64)>,
    pub lane_feedback: V,
    pub finish_evidence: V,
    pub _exit_heading: Option<f64>,
    pub role_distinct_evidence: V,
    pub second_acquisition_reason: Option<String>,
    pub _second_proof_at: Option<f64>,
    pub _second_pass_history: Vec<Row>,
    pub _second_role_anchor: V,
    pub _second_recovering: bool,
    pub _second_recovery_last: V,
    pub second_recovery_prior: V,
    pub second_rebind_evidence: V,
    pub _pause_stage_held: bool,
    pub _pause_steer_target: Option<i64>,
    pub _avoid: V,
    pub _proximity_cache: Option<(V, V, V)>,
    pub _axis_cache: Option<(V, V, Vec<(f64, Rc<Vec<Line>>)>)>,
    pub _exit_release_cache: Option<(V, i64)>,
    pub _path_cache: Option<(V, i64)>,
    pub path_evidence: V,
    pub path_override_count: i64,
    pub avoid_count: i64,
    pub avoid_evidence: Vec<V>,
    pub wait_steering_changes: i64,
    pub compass: WallCompass,
    pub entry_kind: Option<&'static str>,
    pub _lane_start_steering: LaneWallSteering,
    pub _lane_start_settle_since: Option<f64>,
    pub _lane_start_ticks: i64,
    pub _lane_start_slow: bool,
    pub _start_motor: i64,
    pub _start_speed_measured: bool,
    pub _start_speed_last: Option<(f64, f64)>,
    pub start_lane_coast_steering: V,
    pub start_side_wall: V,
    pub _front_track: Option<(i64, Vec<(i64, f64)>, f64)>,
    pub _track_history: Vec<(f64, f64, f64)>,
    pub _power_cmd_at: Option<f64>,
    pub _power_since: Option<f64>,
    pub _power_prev_end: Option<f64>,
    pub _launch_cmd_at: Option<f64>,
    pub _power_cmd_motor: Option<i64>,
    pub _launch_until: Option<f64>,
    pub start_lane_feedback: V,
    pub _lane_start_ok: bool,
    pub first_approach: V,
    pub _start_lane_width: V,
    pub _open_slots: Vec<(i64, Slot)>,
    pub _open_lost_since: Option<f64>,
    pub _second_plan_pending: Option<(V, V)>,
    pub open_loop: V,
    pub open_next_loop: V,
    pub open_feedback: V,
    pub _open_lookout_seq: Option<i64>,
    // dynamically created attributes (getattr defaults)
    pub _task_left_upper: Option<(V, i64)>,
    pub _second_lane_heading: Option<f64>,
    pub _second_lane_previous: Option<(f64, f64, f64)>,
    pub _second_lane_rate: Option<f64>,
}

pub fn new_second_tracker() -> CompactTargetTracker {
    let mut tracker = CompactTargetTracker::new(TrackerArgs {
        acquisition_bearing_rad: (-PI / 2., PI / 2.),
        maintenance_bearing_rad: Some((-PI, PI / 2.)),
        select_associated: true,
        unconfirmed_boundary_gap: true,
        tolerant: true,
        acquisition_min_points: Some(SECOND_ACQUISITION_MIN_POINTS),
    });
    tracker.extend_acquisition_range(EXTENDED_MAX_RANGE_M);
    tracker
}

pub fn validate_maneuver_initial_pwm(value: &V) -> i64 {
    if !value.is_int() || !(INITIAL_PWM_MIN..=SERVO_MAX).contains(&value.i()) {
        value_error("invalid_maneuver_initial_presteer_pwm")
    }
    value.i()
}

impl Maneuver {
    /// ManeuverSequence(started_at, max_drive_s, initial_presteer_pwm, continue_route).
    pub fn new(started_at: &V, max_drive_s: &V, initial_presteer_pwm: &V, continue_route: &V) -> Maneuver {
        if !continue_route.is_bool() {
            value_error("invalid_continue_route")
        }
        let continue_route = continue_route.is_true();
        let initial = validate_maneuver_initial_pwm(initial_presteer_pwm);
        let mut route_drive_s = None;
        let mut max_drive = max_drive_s.clone();
        if continue_route && max_drive_s.is_num() && MAX_DRIVE_S < max_drive_s.f() && max_drive_s.f() <= ROUTE_MAX_DRIVE_S {
            route_drive_s = Some(max_drive_s.f());
            max_drive = V::Float(MAX_DRIVE_S);
        }
        // TurnMotion.__init__
        if !started_at.is_num()
            || started_at.f() < 0.
            || !max_drive.is_num()
            || !(0. < max_drive.f() && max_drive.f() <= MAX_DRIVE_S)
        {
            value_error("invalid_bounded_left_turn_trial")
        }
        if !(INITIAL_PRESTEER_MIN..=SERVO_MAX).contains(&initial) {
            value_error("invalid_initial_presteer_pwm")
        }
        let started = started_at.f();
        let mut tracker = if continue_route {
            CompactTargetTracker::new(TrackerArgs {
                maintenance_bearing_rad: Some((-PI, PI)),
                select_associated: true,
                unconfirmed_boundary_gap: true,
                tolerant: true,
                ..TrackerArgs::default()
            })
        } else {
            CompactTargetTracker::new(TrackerArgs::default())
        };
        let _ = &mut tracker;
        Maneuver {
            started_at: started,
            initial_presteer_pwm: Some(initial),
            natural_steering_target: None,
            max_drive_s: route_drive_s.unwrap_or(max_drive.f()),
            max_presteer_s: PRESTEER_MAX_S,
            phase: "presteer",
            servo: NEUTRAL,
            steering_target: NEUTRAL,
            last_change: started - PWM_INTERVAL_S,
            last_now: started,
            last_seq: None,
            last_publication: None,
            last_receive: None,
            last_signature: V::None,
            corridor: V::None,
            geometry_mode: None,
            outer_wall: V::None,
            turn_goal: V::None,
            last_geometry_receive: None,
            last_geometry_publication: None,
            last_geometry_seq: None,
            geometry_missing: false,
            selected: false,
            settle_since: None,
            settle_feedback_ticks: 0,
            last_control_tick: None,
            last_control_seq: None,
            drive_since: None,
            presteer_ended_at: None,
            last_error: None,
            alignment_count: 0,
            alignment_publication: None,
            alignment_receive: None,
            observed_alignment: false,
            entry_confirmed: false,
            alignment_evidence: None,
            reason: "awaiting_left_corridor".into(),
            start_ready: false,
            presteer_waiting: false,
            incoming_endpoint: V::None,
            exit_width_change: V::None,
            continue_route,
            target_tracker: tracker,
            compact_target: V::None,
            _route_left_hold_since: None,
            left_overshoot_evidence: V::None,
            _last_seen_compact: None,
            _last_confirmed_compact_target: V::None,
            compact_target_error: None,
            compact_loss_evidence: V::None,
            handover_observed: false,
            orbit_since: None,
            orbit_reference_range_m: V::None,
            orbit_bias_pwm: V::None,
            orbit_feedback: V::None,
            orbit_left_entry_boost: false,
            _orbit_abeam_count: 0,
            _orbit_abeam_publication: V::None,
            _orbit_abeam_receive: None,
            left_turn_feedback: V::None,
            left_handover_preparing: false,
            handover_control: V::None,
            handover_wait_since: None,
            orbit_track_id: V::None,
            coast_since: None,
            wall_ambiguity_hold: false,
            _last_actual_left_target: None,
            _presteer_adoption_context: None,
            _current_scan: V::None,
            inner_clearance: V::None,
            quality_coast_servo: None,
            quality_wait_refusal: V::None,
            left_turn_pause_used: false,
            left_turn_pause_evidence: V::None,
            left_turn_pause_geometry_error: None,
            neutral_pause_kind: None,
            recovery_count: 0,
            recovery_evidence: V::None,
            post_pass_pause_evidence: V::None,
            quality_coast_reason: None,
            entry_bearing_required: false,
            entry_bearing_released: false,
            entry_bearing: V::None,
            _incoming_association: V::None,
            _entry_last_receive: V::None,
            _entry_last_publication: None,
            _entry_ready_count: 0,
            _entry_ready_receive: V::None,
            _entry_ready_publication: None,
            _entry_ready_observation_type: None,
            _entry_ready_history: Vec::new(),
            adopted_presteer_pwm: None,
            first_pass_preparing: false,
            first_pass_evidence: V::None,
            first_pass_progress: V::None,
            _first_pass_history: Vec::new(),
            _first_exit_prepare_history: Vec::new(),
            _quality_exit_prepare_ready: false,
            quality_wait_release: V::None,
            quality_wait_previous_servo: None,
            _wait_release_history: Vec::new(),
            quality_wait_anchor: V::None,
            quality_wait_rebind: V::None,
            _wait_rebind_candidate: V::None,
            first_exit_prepare_evidence: V::None,
            right_exit_since: None,
            right_exit_center_ack: V::None,
            right_exit_adoption_wait_since: None,
            right_exit_geometry: V::None,
            _right_exit_last_receive: V::None,
            _right_exit_last_publication: V::None,
            route_stage: "first_target",
            route_events: V::dict(),
            quality_wait_since: None,
            quality_wait_servo: None,
            quality_wait_used: false,
            quality_wait_evidence: V::None,
            quality_resume_pending: false,
            quality_resume_since: None,
            _quality_good_history: Vec::new(),
            _quality_full_streak: 0,
            _quality_pass_history: Vec::new(),
            _route_alignment_history: Vec::new(),
            route_alignment_evidence: V::None,
            _route_center_ack: V::None,
            second_tracker: if continue_route { Some(new_second_tracker()) } else { None },
            second_target: V::None,
            second_track_id: V::None,
            second_target_handover_observed: false,
            second_center_ack: V::None,
            second_orbit_since: None,
            second_pass_evidence: V::None,
            second_feedback: V::None,
            second_return_preparing: false,
            second_return_evidence: V::None,
            route_end_coast_servo: None,
            light_since: None,
            light_turn_started: None,
            _lane_steering: CorridorSteering::default(),
            _lane_speed_last: None,
            _lane_pulse: 0,
            _light_history: Vec::new(),
            _light_heading: None,
            light_feedback: V::None,
            light_alignment_evidence: V::None,
            lane_motor: ROUTE_SLOW_MOTOR,
            _lane_coast_track: Vec::new(),
            _lane_coast_decel: None,
            _lane_coast_decel_se: None,
            _lane_ranges: Vec::new(),
            lane_feedback: V::None,
            finish_evidence: V::None,
            _exit_heading: None,
            role_distinct_evidence: V::None,
            second_acquisition_reason: None,
            _second_proof_at: None,
            _second_pass_history: Vec::new(),
            _second_role_anchor: V::None,
            _second_recovering: false,
            _second_recovery_last: V::None,
            second_recovery_prior: V::None,
            second_rebind_evidence: V::None,
            _pause_stage_held: false,
            _pause_steer_target: None,
            _avoid: V::None,
            _proximity_cache: None,
            _axis_cache: None,
            _exit_release_cache: None,
            _path_cache: None,
            path_evidence: V::None,
            path_override_count: 0,
            avoid_count: 0,
            avoid_evidence: Vec::new(),
            wait_steering_changes: 0,
            compass: WallCompass::default(),
            entry_kind: None,
            _lane_start_steering: LaneWallSteering::default(),
            _lane_start_settle_since: None,
            _lane_start_ticks: 0,
            _lane_start_slow: false,
            _start_motor: TRIAL_MOTOR,
            _start_speed_measured: false,
            _start_speed_last: None,
            start_lane_coast_steering: V::None,
            start_side_wall: V::None,
            _front_track: None,
            _track_history: Vec::new(),
            _power_cmd_at: None,
            _power_since: None,
            _power_prev_end: None,
            _launch_cmd_at: None,
            _power_cmd_motor: None,
            _launch_until: None,
            start_lane_feedback: V::None,
            _lane_start_ok: false,
            first_approach: V::None,
            _start_lane_width: V::None,
            _open_slots: Vec::new(),
            _open_lost_since: None,
            _second_plan_pending: None,
            open_loop: V::None,
            open_next_loop: V::None,
            open_feedback: V::None,
            _open_lookout_seq: None,
            _task_left_upper: None,
            _second_lane_heading: None,
            _second_lane_previous: None,
            _second_lane_rate: None,
        }
    }

    pub fn second_tracker_mut(&mut self) -> &mut CompactTargetTracker {
        self.second_tracker.as_mut().unwrap_or_else(|| raise(Exc::TypeError, "'NoneType' object has no attribute"))
    }

    pub fn slot_mut(&mut self, side: i64) -> &mut Slot {
        if !self._open_slots.iter().any(|(s, _)| *s == side) {
            self._open_slots.push((side, Slot::default()));
        }
        &mut self._open_slots.iter_mut().find(|(s, _)| *s == side).unwrap().1
    }

    pub fn has_route_event(&self, name: &str) -> bool {
        self.route_events.has(name)
    }

    /// _route_event(name, scan, control, evidence).
    pub fn route_event(&mut self, name: &str, scan: &V, control: &V, evidence: V) {
        if !self.route_events.has(name) {
            let snap = scan.pick(&["frame_id", "seq", "at_ms", "received_at", "ranges", "corridor_candidates"]);
            self.route_events.set(name, dict! {"scan" => snap, "control" => control.clone(), "evidence" => evidence});
        }
    }

    // ------------------------------------------------------------ TurnMotion core

    /// TurnMotion._presteer_hold.
    pub fn presteer_hold(&mut self, reason: &str) {
        self.reason = reason.into();
        self.start_ready = false;
        self.presteer_waiting = true;
        self.settle_since = None;
        self.settle_feedback_ticks = 0;
        self.last_error = None;
        self.natural_steering_target = None;
    }

    /// TurnMotion.lock.
    pub fn tm_lock(&mut self, reason: &str) {
        if self.phase == "presteer" {
            self.presteer_ended_at = Some(self.last_now);
        }
        self.phase = "locked";
        self.reason = reason.into();
        self.presteer_waiting = false;
        self.incoming_endpoint = V::None;
        self.natural_steering_target = None;
        self.servo = NEUTRAL;
        self.steering_target = NEUTRAL;
        self.start_ready = false;
        self.settle_since = None;
        self.settle_feedback_ticks = 0;
    }

    /// ManeuverSequence.lock.
    pub fn lock(&mut self, reason: &str) {
        let adoption = self._presteer_adoption_context.clone();
        if reason == "left_turn_outer_wall_ambiguous"
            && self.phase == "presteer"
            && self.selected
            && self.last_geometry_receive.is_some()
            && self.last_geometry_publication.is_some()
            && self._last_actual_left_target.is_some_and(|t| t > NEUTRAL)
            && adoption.as_ref().is_some_and(|(control, fresh)| {
                *fresh
                    && control.get("armed").is_true()
                    && eq_i(control.get("motor"), NEUTRAL)
                    && eq_i(control.get("servo"), self.servo)
                    && (if control.has("command_acked") { control.get("command_acked").is_true() } else { true })
            })
        {
            self.wall_ambiguity_hold = true;
            self.geometry_missing = true;
            self.steering_target = self.servo;
            self.presteer_hold("left_turn_presteer_wall_ambiguity_hold");
            self.alignment_count = 0;
            self.alignment_evidence = None;
            self.alignment_publication = None;
            self.alignment_receive = None;
            return;
        }
        if reason == "left_turn_outer_wall_ambiguous"
            && self.phase == "drive"
            && self.last_geometry_receive.is_some()
            && self.last_geometry_publication.is_some()
            && {
                let a = self.last_now - self.last_geometry_receive.unwrap();
                0. <= a && a <= SCAN_AGE_S + 1e-9
            }
            && {
                let b = (self.last_publication.unwrap_or_else(|| type_error("unsupported operand")) - self.last_geometry_publication.unwrap()) as f64 / 1000.;
                0. <= b && b <= SCAN_AGE_S + 1e-9
            }
        {
            self.wall_ambiguity_hold = true;
            self.geometry_missing = true;
            self.start_ready = false;
            self.reason = "left_turn_wall_ambiguity_hold".into();
            self.steering_target = self.servo;
            self.alignment_count = 0;
            self.alignment_evidence = None;
            self.alignment_publication = None;
            self.alignment_receive = None;
            self.last_error = None;
            return;
        }
        self.tm_lock(reason);
    }

    /// TurnMotion.begin_coast.
    pub fn tm_begin_coast(&mut self, reason: &str, now: f64) {
        if !now.is_finite() || now < self.last_now {
            self.lock("invalid_turn_receive_clock");
        } else if self.phase != "locked" {
            if self.phase == "presteer" {
                self.presteer_ended_at = Some(now);
            }
            self.phase = "coast";
            self.reason = reason.into();
            self.steering_target = NEUTRAL;
            self.settle_since = None;
            self.last_error = None;
            self.natural_steering_target = None;
        }
    }

    /// ManeuverSequence.begin_coast.
    pub fn begin_coast(&mut self, reason: &str, now: f64) {
        let seen = self._last_seen_compact.clone();
        let target_on_left = seen.as_ref().is_some_and(|(at, target)| {
            now.is_finite()
                && 0. <= now - at
                && now - at < SCAN_AGE_S
                && target.get("bearing_left_rad").is_num()
                && target.get("range_m").is_num()
                && 0. < target.idx("bearing_left_rad").f()
                && target.idx("bearing_left_rad").f() < PI
                && target.idx("range_m").f() <= ORBIT_REBIND_MAX_RANGE_M + 0.3
        });
        if reason == "left_turn_heading_overshoot"
            && self.continue_route
            && self.phase == "drive"
            && !self.handover_observed
            && self.route_stage == "first_target"
            && self._route_left_hold_since.is_none()
            && NEUTRAL < self.servo
            && self.servo <= SERVO_MAX
            && target_on_left
            && now >= self.last_now
        {
            let (at, target) = seen.unwrap();
            self._route_left_hold_since = Some(now);
            self.left_overshoot_evidence = dict! {
                "at" => now, "held_servo" => self.servo,
                "scan_seq" => if self._current_scan.truthy() { self._current_scan.get("seq").clone() } else { V::None },
                "last_seen_first_target" => target, "last_seen_age_s" => now - at,
                "tracker_reason" => self.target_tracker.reason.as_str(),
                "basis" => "left_turn_past_exit_axis_with_live_first_target_identity_on_left",
            };
            self.steering_target = self.servo;
            self.reason = "route_left_turn_overshoot_hold_left".into();
            return;
        }
        if self.phase != "coast" && self.phase != "locked" && now.is_finite() && now >= self.last_now {
            self.coast_since = Some(now);
        }
        self.tm_begin_coast(reason, now);
    }

    /// TurnMotion._result.
    pub fn tm_result(&self, now: f64) -> V {
        let coast = self.phase == "coast";
        let presteer_end = self.presteer_ended_at.unwrap_or(now);
        dict! {
            "phase" => self.phase, "motor" => if self.phase == "drive" { TRIAL_MOTOR } else { NEUTRAL },
            "servo" => self.servo, "steering_target" => self.steering_target,
            "initial_presteer_pwm" => self.initial_presteer_pwm,
            "initial_presteer_active" => self.phase == "presteer" && self.initial_presteer_pwm.is_some() && self.start_ready
                && self.natural_steering_target.is_some_and(|t| t > NEUTRAL),
            "natural_steering_target" => self.natural_steering_target,
            "start_ready" => self.start_ready && self.phase == "presteer",
            "request_coast" => coast, "stop_requested" => coast,
            "lock_requested" => self.phase == "locked",
            "terminal_reason" => if self.phase == "coast" || self.phase == "locked" { V::from(self.reason.as_str()) } else { V::None },
            "reason" => self.reason.as_str(), "observed_alignment" => self.observed_alignment,
            "test_sequence_finished" => self.observed_alignment,
            "entry_confirmed" => self.entry_confirmed, "completed" => false,
            "competition_navigation" => false, "candidate_only" => true,
            "turn_path_certified" => false, "physical_steering_confirmed" => false,
            "steering_allowance_s" => STEERING_ALLOWANCE_S,
            "presteer_elapsed_s" => max0(presteer_end - self.started_at),
            "steering_settle_elapsed_s" => match self.settle_since { None => V::Int(0), Some(s) => max0(now - s) },
            "steering_settle_feedback_ticks" => self.settle_feedback_ticks,
            "drive_elapsed_s" => match self.drive_since { None => V::Int(0), Some(s) => max0(now - s) },
            "alignment_confirmations" => self.alignment_count,
            "alignment_evidence" => self.alignment_evidence,
            "scan_seq" => self.last_seq, "publication_at_ms" => self.last_publication,
            "geometry_mode" => self.geometry_mode, "geometry_missing" => self.geometry_missing,
            "geometry_missing_s" => if !self.geometry_missing || self.last_geometry_receive.is_none() { V::Int(0) }
                else { max0(now - self.last_geometry_receive.unwrap()) },
            "geometry_source_seq" => self.last_geometry_seq,
            "geometry_source_at_ms" => self.last_geometry_publication,
            "turn_stage" => if self.presteer_waiting && self.phase == "presteer" { "presteer_wait" } else { self.phase },
            "incoming_endpoint_current" => self.incoming_endpoint.is_some(),
            "incoming_endpoint" => self.incoming_endpoint.clone(),
            "exit_width_change" => self.exit_width_change.clone(),
            "turn_goal" => self.turn_goal.clone(),
            "outer_wall" => self.outer_wall.clone(),
            "corridor" => if self.corridor.truthy() { self.corridor.clone() } else { V::None },
        }
    }

    /// ManeuverSequence._result.
    pub fn result(&mut self, now: f64) -> V {
        if self.phase == "drive" && self.adopted_presteer_pwm.is_none() {
            self.adopted_presteer_pwm = Some(self.servo);
        }
        let mut result = self.tm_result(now);
        if eq_i(result.get("motor"), TRIAL_MOTOR) {
            result.set("motor", self.route_motor());
        }
        let motor = result.get("motor").i();
        if !DRIVE_MOTORS.contains(&motor) {
            self._launch_until = None;
        } else if self._launch_until.is_none() {
            self._launch_until = Some(now + ESC_LAG_S + LANE_LAUNCH_S);
            self._power_since = Some(now);
            self._power_prev_end = self._power_cmd_at;
        }
        if motor == ROUTE_SLOW_MOTOR
            && self.continue_route
            && self.entry_kind == Some("lane")
            && now < self._launch_until.unwrap_or_else(|| type_error("'<' not supported between float and NoneType"))
            && ["first_target", "open_transfer", "second_loop", "light_entry"].contains(&self.route_stage)
        {
            result.set("motor", TRIAL_MOTOR);
        }
        let motor = result.get("motor").i();
        if DRIVE_MOTORS.contains(&motor) {
            self._power_cmd_at = Some(now);
            self._power_cmd_motor = Some(motor);
            if motor == START_LAUNCH_MOTOR {
                self._launch_cmd_at = Some(now);
            }
        }
        let mut stage: V = if self.handover_observed && self.phase == "drive" {
            "orbit_entry".into()
        } else if self.phase == "drive" {
            "left_turn".into()
        } else {
            result.get("turn_stage").clone()
        };
        if self.phase == "drive" && self.first_pass_preparing {
            stage = if self.first_pass_evidence.is_some() { "right_exit".into() } else { "first_pass_prepare".into() };
        }
        let route = self.continue_route;
        let o = |v: Option<i64>| -> V { if route { V::from(v) } else { V::None } };
        result.update(&dict! {
            "turn_stage" => stage, "trial_scope" => "first_lidar_compact_target_orbit_entry",
            "compact_target" => self.compact_target.clone(),
            "compact_target_reason" => self.target_tracker.reason.as_str(),
            "compact_loss_evidence" => self.compact_loss_evidence.clone(),
            "first_pass_preparing" => self.first_pass_preparing,
            "first_pass_progress" => self.first_pass_progress.clone(),
            "first_exit_prepare_evidence" => self.first_exit_prepare_evidence.clone(),
            "first_pass_evidence" => self.first_pass_evidence.clone(),
            "right_exit_center_ack" => self.right_exit_center_ack.clone(),
            "right_exit_geometry" => self.right_exit_geometry.clone(),
            "right_exit_min_pwm" => right_exit_min_pwm(),
            "right_exit_elapsed_s" => match self.right_exit_since { None => V::Int(0), Some(s) => max0(now - s) },
            "right_exit_max_s" => RIGHT_EXIT_MAX_S,
            "second_target_handover_observed" => self.second_target_handover_observed,
            "full_two_cone_s_supported" => false,
            "handover_observed" => self.handover_observed,
            "orbit_reference_range_m" => self.orbit_reference_range_m.clone(),
            "orbit_bias_pwm" => self.orbit_bias_pwm.clone(),
            "orbit_feedback" => self.orbit_feedback.clone(),
            "orbit_left_entry_boost" => self.orbit_left_entry_boost,
            "left_turn_feedback" => self.left_turn_feedback.clone(),
            "handover_control" => self.handover_control.clone(),
            "orbit_entry_elapsed_s" => match self.orbit_since { None => V::Int(0), Some(s) => max0(now - s) },
            "orbit_entry_max_s" => ORBIT_ENTRY_MAX_S, "second_orbit_max_s" => SECOND_ORBIT_MAX_S,
            "presteer_max_s" => self.max_presteer_s,
            "left_turn_servo_cap" => SERVO_MAX,
            "steering_step_policy" => dict! {
                "presteer" => PRESTEER_PWM_STEP, "reduce_left" => LEFT_RELEASE_PWM_STEP,
                "route_exit_reduce_left" => o(Some(ROUTE_EXIT_RELEASE_PWM_STEP)),
                "route_post_pass" => o(Some(ROUTE_POST_PASS_PWM_STEP)),
                "route_orbit_increase_left" => o(Some(ROUTE_ORBIT_LEFT_PWM_STEP)),
                "increase_left" => PWM_STEP, "center_to_right" => PWM_STEP, "interval_s" => PWM_INTERVAL_S,
            },
            "object_semantic_verified" => false, "passed_cones" => V::None,
            "competition_supported" => false, "completed" => false,
            "test_sequence_finished" => false,
            "inner_clearance" => self.inner_clearance.clone(),
            "entry_bearing" => self.entry_bearing.clone(),
            "entry_bearing_required" => self.entry_bearing_required,
            "entry_bearing_released" => self.entry_bearing_released,
            "adopted_presteer_pwm" => self.adopted_presteer_pwm,
            "quality_coast_servo" => self.quality_coast_servo,
            "quality_wait_refusal" => self.quality_wait_refusal.clone(),
            "quality_coast_reason" => self.quality_coast_reason.clone(),
            "quality_coast_hold_active" => self.phase == "coast" && self.quality_coast_servo.is_some(),
            "wall_ambiguity_hold" => self.wall_ambiguity_hold,
        });
        let right_authorized = self.continue_route
            && self.phase == "drive"
            && !self.quality_resume_pending
            && self._route_center_ack.is_some()
            && ((self.first_pass_evidence.is_some()
                && ((["right_align", "corridor_follow"].contains(&self.route_stage) && self.right_exit_center_ack.is_some())
                    || (self.route_stage == "second_orbit" && self.second_center_ack.is_some())
                    || ["light_turn", "light_lane"].contains(&self.route_stage)
                    || OPEN_STAGES.contains(&self.route_stage)))
                || (self.route_stage == "start_lane" && self.entry_kind == Some("lane")));
        result.update(&dict! {
            "continue_route" => self.continue_route, "route_stage" => self.route_stage,
            "quality_wait_active" => self.phase == "quality_wait",
            "quality_wait_servo" => self.quality_wait_servo,
            "quality_wait_used" => self.quality_wait_used,
            "quality_wait_anchor" => self.quality_wait_anchor.clone(),
            "quality_wait_rebind" => self.quality_wait_rebind.clone(),
            "left_turn_pause_used" => self.left_turn_pause_used,
            "neutral_pause_kind" => self.neutral_pause_kind,
            "recovery_count" => self.recovery_count,
            "recovery_evidence" => self.recovery_evidence.clone(),
            "route_left_hold_active" => self._route_left_hold_since.is_some(),
            "left_overshoot_evidence" => self.left_overshoot_evidence.clone(),
            "route_avoid" => self._avoid.clone(), "avoid_count" => self.avoid_count,
            "path_evidence" => self.path_evidence.clone(), "path_override_count" => self.path_override_count,
            "avoid_evidence" => V::list(self.avoid_evidence.clone()),
            "wait_steering_changes" => self.wait_steering_changes,
            "post_pass_pause_evidence" => self.post_pass_pause_evidence.clone(),
            "left_turn_pause_evidence" => self.left_turn_pause_evidence.clone(),
            "left_turn_pause_geometry_error" => self.left_turn_pause_geometry_error.clone(),
            "quality_wait_evidence" => self.quality_wait_evidence.clone(),
            "quality_resume_pending" => self.quality_resume_pending,
            "quality_wait_release" => self.quality_wait_release.clone(),
            "quality_wait_previous_servo" => self.quality_wait_previous_servo,
            "route_alignment_evidence" => self.route_alignment_evidence.clone(),
            "second_target" => self.second_target.clone(),
            "second_rebind_evidence" => self.second_rebind_evidence.clone(),
            "second_identity_recovering" => self._second_recovering,
            "second_center_ack" => self.second_center_ack.clone(),
            "second_pass_evidence" => self.second_pass_evidence.clone(),
            "second_feedback" => self.second_feedback.clone(),
            "second_return_preparing" => self.second_return_preparing,
            "second_return_evidence" => self.second_return_evidence.clone(),
            "route_end_coast_servo" => self.route_end_coast_servo,
            "light_feedback" => self.light_feedback.clone(),
            "light_alignment_evidence" => self.light_alignment_evidence.clone(),
            "lane_feedback" => self.lane_feedback.clone(), "lane_motor" => self.lane_motor,
            "finish_evidence" => self.finish_evidence.clone(),
            "second_acquisition_reason" => self.second_acquisition_reason.clone(),
            "second_recovery_prior" => self.second_recovery_prior.clone(),
            "role_distinct_evidence" => self.role_distinct_evidence.clone(),
            "route_events" => self.route_events.clone(),
            "two_target_observed_pass_complete" => self.second_pass_evidence.is_some(),
            "route_right_output_authorized" => right_authorized,
            "route_center_ack" => self._route_center_ack.clone(),
            "entry_kind" => self.entry_kind, "start_lane_feedback" => self.start_lane_feedback.clone(),
            "first_approach" => self.first_approach.clone(),
            "open_loop" => self.open_loop.clone(), "open_next_loop" => self.open_next_loop.clone(),
            "open_feedback" => self.open_feedback.clone(),
            "compass" => if self.continue_route { self.compass.evidence.clone() } else { V::None },
        });
        if self.continue_route && self.phase == "drive" && self.route_stage != "first_target" {
            result.set("turn_stage", self.route_stage);
        }
        if self.continue_route {
            result.set("trial_scope", "opt_in_observed_two_target_route");
        }
        result
    }

    /// _route_motor.
    pub fn route_motor(&self) -> i64 {
        let slow_stage = (self.handover_observed && self.route_stage == "first_target")
            || (self.first_pass_evidence.is_some() && POST_PASS_STAGES.contains(&self.route_stage));
        if self.continue_route && self.phase == "drive" && self.route_stage == "light_lane" && !self.quality_resume_pending {
            return self.lane_motor;
        }
        if self.continue_route && self.phase == "drive" && self.route_stage == "start_lane" {
            return self._start_motor;
        }
        if self.continue_route && self.phase == "drive" && slow_stage && !self.quality_resume_pending {
            ROUTE_SLOW_MOTOR
        } else {
            TRIAL_MOTOR
        }
    }

    // ------------------------------------------------------------ input contract

    /// _orbit_inputs: same advancing raw-scan/control contract.
    pub fn orbit_inputs(&mut self, scan: &V, lidar_age_s: &V, now: f64, control: &V, safe: &V, presteer_wait: &V) -> bool {
        if !now.is_finite() || now < self.last_now {
            self.lock("invalid_turn_receive_clock");
            return false;
        }
        self.last_now = now;
        if !safe.is_true() {
            self.lock("turn_safety_rejected");
            return false;
        }
        if !presteer_wait.is_bool() || presteer_wait.truthy() {
            self.lock("turn_presteer_wait_requires_neutral");
            return false;
        }
        let received = if scan.is_dict() { scan.get("received_at").clone() } else { V::None };
        let ranges = if scan.is_dict() { scan.get("ranges").clone() } else { V::None };
        if !lidar_age_s.is_num()
            || !(0. <= lidar_age_s.f() && lidar_age_s.f() < SCAN_AGE_S)
            || !scan.is_dict()
            || scan.get("frame_id").os() != Some(FRAME)
            || !scan.get("seq").is_int()
            || scan.get("seq").i() < 0
            || !scan.get("at_ms").is_int()
            || scan.get("at_ms").i() < 0
            || !received.is_num()
            || !(0. <= now - received.f() && now - received.f() < SCAN_AGE_S)
            || !valid_ranges(&ranges)
        {
            self.lock("turn_scan_stale_or_invalid");
            return false;
        }
        if !control.is_dict()
            || !control.get("armed").is_bool()
            || !control.get("motor").is_int()
            || !control.get("servo").is_int()
            || !control.get("tick").is_int()
            || control.get("tick").i() < 0
            || !control.get("seq").is_int()
            || control.get("seq").i() < 0
            || !(SERVO_MIN.min(right_exit_min_pwm()) <= control.get("servo").i() && control.get("servo").i() <= SERVO_MAX)
        {
            self.lock("invalid_turn_adoption_feedback");
            return false;
        }
        let (tick, cseq) = (control.idx("tick").i(), control.idx("seq").i());
        if let Some(last_tick) = self.last_control_tick
            && (tick < last_tick || cseq < self.last_control_seq.unwrap())
        {
            self.lock("reordered_turn_adoption_feedback");
            return false;
        }
        self.last_control_tick = Some(tick);
        self.last_control_seq = Some(cseq);
        if !control.idx("armed").truthy() {
            self.lock("turn_bridge_locked");
            return false;
        }
        let signature = V::list(vec![
            scan.get("corridor_candidates").clone(),
            get_or(scan, "wall_candidates", &V::list(vec![])).clone(),
            scan.get("left_turn_goal").clone(),
            ranges.clone(),
        ]);
        let (seq, published) = (scan.idx("seq").i(), scan.idx("at_ms").i());
        if self.last_seq == Some(seq) {
            if Some(published) != self.last_publication || !py_eq(&signature, &self.last_signature) {
                self.lock("duplicate_turn_scan_changed_content");
            } else if now - self.last_receive.unwrap() >= SCAN_AGE_S {
                self.lock("turn_scan_not_advancing");
            }
            return false;
        }
        let publication_dt = match self.last_publication {
            None => 0.,
            Some(last) => (published - last) as f64 / 1000.,
        };
        if let Some(last_seq) = self.last_seq
            && (seq < last_seq || publication_dt <= 0.)
        {
            self.lock("reordered_turn_scan");
            return false;
        }
        if let Some(last_receive) = self.last_receive
            && (publication_dt >= SCAN_AGE_S
                || now - last_receive >= SCAN_AGE_S
                || (publication_dt - (now - last_receive)).abs() > 0.15)
        {
            self.lock("turn_scan_clock_gap");
            return false;
        }
        self.last_seq = Some(seq);
        self.last_publication = Some(published);
        self.last_receive = Some(now);
        self.last_signature = signature;
        true
    }

    // ------------------------------------------------------------ update

    /// ManeuverSequence.update.
    #[allow(clippy::too_many_arguments)]
    pub fn update(
        &mut self,
        scan: &V,
        lidar_age_s: &V,
        now_v: &V,
        control: &V,
        safe: &V,
        entry_stop: bool,
        presteer_wait: &V,
        quality_clear: &V,
        quality_resume_ready: &V,
    ) -> V {
        self._current_scan = scan.clone();
        self._avoid = if self.continue_route && self.phase != "locked" { self.route_proximity(scan) } else { V::None };
        if self._avoid.is_some() {
            self.avoid_count += 1;
            if self.avoid_evidence.len() < 40 {
                let mut e = self._avoid.clone();
                e.update(&dict! {"phase" => self.phase, "route_stage" => self.route_stage, "servo" => self.servo});
                self.avoid_evidence.push(e);
            }
        }
        if !scan.is_dict()
            || self.inner_clearance.is_none()
            || !py_eq(self.inner_clearance.idx("source_seq"), scan.get("seq"))
            || !py_eq(self.inner_clearance.idx("source_at_ms"), scan.get("at_ms"))
        {
            self.inner_clearance = V::None;
        }
        if self.phase == "locked" {
            let at = self.last_now;
            return self.result(at);
        }
        if !now_v.is_num() || now_v.f() < self.last_now {
            self.lock("invalid_turn_receive_clock");
            let at = self.last_now;
            return self.result(at);
        }
        let now = now_v.f();
        if self.continue_route && scan.is_dict() && scan.get("at_ms").is_int() {
            self.compass.update(scan);
        }
        if self.compact_target.is_dict() && self.compact_target.get("confirmed").is_true() {
            self._last_confirmed_compact_target = self.compact_target.clone();
        }
        self.compact_target_error = None;
        let outcome = except(VTK, || {
            self.compact_target = if self.continue_route {
                let heading = self.tracking_heading(scan);
                let acquire = self.first_pass_evidence.is_none();
                self.target_tracker.update(scan, now, acquire, heading, None)
            } else {
                self.target_tracker.update(scan, now, true, None, None)
            };
            let corridor_drive =
                self.route_stage == "corridor_follow" && self.phase == "drive" && !self.quality_resume_pending;
            if self.continue_route
                && (["second_orbit", "second_loop"].contains(&self.route_stage) || (self._second_recovering && !corridor_drive))
            {
                self.update_second_tracking(scan, now);
            }
        });
        if let Err(error) = outcome {
            self.compact_target = V::None;
            self.compact_target_error = Some(error.kind.name().into());
        }
        if self.compact_target.is_dict() {
            self._last_seen_compact = Some((now, self.compact_target.clone()));
        }
        if entry_stop {
            self.lock("unsupported_maneuver_entry_stop");
            let at = self.last_now;
            return self.result(at);
        }
        if self.continue_route
            && self.neutral_pause_kind.is_some()
            && (self.phase == "quality_wait" || self.quality_resume_pending)
        {
            return self.neutral_pause_update(scan, lidar_age_s, now, control, safe, presteer_wait, quality_clear, quality_resume_ready);
        }
        if self.continue_route && self.entry_kind.is_none() && self.phase == "presteer" {
            self.decide_entry(scan);
        }
        if self.continue_route && self.route_stage == "start_lane" {
            if !(self.phase == "drive" && self.start_lane_handover_ready(scan, control)) {
                return self.start_lane_update(scan, lidar_age_s, now, control, safe, presteer_wait);
            }
            self.route_stage = "first_target";
            let evidence = self.first_approach.clone();
            self.route_event("first_approach_handover", scan, control, evidence);
        }
        if self.handover_observed
            || (self.phase == "coast" && (self.wall_ambiguity_hold || self.quality_coast_servo.is_some()))
            || (self.phase == "drive" && self.usable_target(&self.compact_target.clone(), scan))
        {
            if self.handover_observed {
                self._route_left_hold_since = None;
            }
            return self.orbit_update(scan, lidar_age_s, now, control, safe, presteer_wait, quality_clear, quality_resume_ready);
        }
        if self._route_left_hold_since.is_some() && self.phase == "drive" {
            return self.route_left_hold_update(scan, lidar_age_s, now, control, safe, presteer_wait);
        }
        self.wall_ambiguity_hold = false;
        let feedback_tick = if control.is_dict() { control.get("tick").clone() } else { V::None };
        let fresh = feedback_tick.is_int() && self.last_control_tick.is_none_or(|last| feedback_tick.i() > last);
        self._presteer_adoption_context =
            Some((if control.is_dict() { control.clone() } else { V::dict() }, fresh));
        let out = py_try(|| self.tm_update(scan, lidar_age_s, now, control, safe, presteer_wait));
        self._presteer_adoption_context = None;
        match out {
            Ok(v) => v,
            Err(e) => std::panic::panic_any(e),
        }
    }

    /// _orbit_update.
    #[allow(clippy::too_many_arguments)]
    pub fn orbit_update(
        &mut self,
        scan: &V,
        lidar_age_s: &V,
        now: f64,
        control: &V,
        safe: &V,
        presteer_wait: &V,
        quality_clear: &V,
        quality_resume_ready: &V,
    ) -> V {
        let advancing = self.orbit_inputs(scan, lidar_age_s, now, control, safe, presteer_wait);
        if self.phase == "locked" {
            self.clear_exit_prepare_window();
            let at = self.last_now;
            return self.result(at);
        }
        if self.phase == "quality_wait" {
            return self.quality_wait_update(scan, now, control, advancing, quality_clear, quality_resume_ready);
        }
        if self.phase == "drive" && now - self.drive_since.unwrap() + 1e-9 >= self.max_drive_s {
            self.begin_coast("maneuver_cumulative_drive_timeout", now);
        }
        if self.phase == "coast" {
            if self.coast_since.is_some_and(|s| now - s + 1e-9 >= COAST_MAX_S) {
                self.lock("coast_standstill_unconfirmed");
            } else {
                self.steering_target = self.quality_coast_servo.or(self.route_end_coast_servo).unwrap_or(NEUTRAL);
                self.slew(now);
            }
            return self.result(now);
        }
        if self.quality_resume_pending {
            let points = self.full_target_points(&self.compact_target.clone(), scan, &self.orbit_track_id.clone());
            let bad_points = match &points {
                None => true,
                Some(p) => {
                    p.iter().map(|q| q.0).fold(f64::NEG_INFINITY, pmax) > FRONT_BODY_EXTENT_M
                        || p.iter().map(|q| q.1).fold(f64::INFINITY, pmin) - SIDE_BODY_EXTENT_M < MANEUVER_BODY_CLEARANCE_M
                }
            };
            if !quality_clear.is_true() || bad_points {
                self.clear_exit_prepare_window();
            }
            if self.first_pass_evidence.is_some() && advancing {
                let (_, rejection) = self.right_exit_target(scan, now);
                if rejection.is_some() {
                    self.lock("route_quality_resume_corridor_unconfirmed");
                    return self.result(now);
                }
            }
            if points.is_none() {
                self.lock("route_quality_resume_target_unconfirmed");
            } else if !py_eq(control.get("servo"), &V::from(self.quality_wait_servo))
                || !motor_in(control.get("motor"), &[NEUTRAL, TRIAL_MOTOR])
            {
                self.lock("route_quality_resume_feedback_changed");
            } else if control.get("resume_acked").is_true()
                && eq_i(control.idx("motor"), TRIAL_MOTOR)
                && control.get("command_acked").is_true()
            {
                self.quality_resume_pending = false;
                let ack = control.pick(&["motor", "servo", "seq", "tick"]);
                self.quality_wait_evidence.set("resume_ack", ack);
                if self._quality_exit_prepare_ready && self.first_pass_evidence.is_none() && !self.open_loop_orbit() {
                    self.latch_exit_prepare(scan, control, "healthy_neutral_wait_body_front_support_rechecked_at_resume_ack", None);
                    self._quality_exit_prepare_ready = false;
                }
                if self.quality_wait_release.is_some() && self.first_pass_evidence.is_none() && !self.open_loop_orbit() {
                    if !self.first_pass_preparing {
                        let history = if self._wait_release_history.is_empty() {
                            vec![scan_row(scan)]
                        } else {
                            self._wait_release_history.clone()
                        };
                        self.latch_exit_prepare(scan, control, "neutral_wait_release_continued_at_resume_ack", Some(history));
                    }
                    self.first_pass_preparing = true;
                    self.orbit_left_entry_boost = false;
                }
            } else if now - self.quality_resume_since.unwrap() >= SCAN_AGE_S {
                self.lock("route_quality_resume_ack_timeout");
            } else {
                self.steering_target = self.quality_wait_servo.unwrap();
            }
            if self.phase == "locked" {
                self.clear_exit_prepare_window();
            }
            return self.result(now);
        }
        if !advancing {
            let pending_match = self._second_plan_pending.as_ref().is_some_and(|(s, a)| {
                py_eq(s, scan.get("seq")) && py_eq(a, scan.get("at_ms"))
            });
            if pending_match
                && self.continue_route
                && self.phase == "drive"
                && self.route_stage == "second_loop"
                && !self.quality_resume_pending
                && {
                    let age = now - scan.idx("received_at").f();
                    0. <= age && age < SCAN_AGE_S
                }
            {
                return self.open_second_loop_update(scan, now, control, true);
            }
            let fresh = || {
                let age = now - scan.idx("received_at").f();
                0. <= age && age < SCAN_AGE_S
            };
            let loop_step = self.open_loop_orbit()
                && self.phase == "drive"
                && self.reason == "open_first_loop"
                && fresh()
                && control.get("command_acked").is_true()
                && motor_in(control.get("motor"), &DRIVE_MOTORS)
                && eq_i(control.get("servo"), self.servo);
            if loop_step {
                self.slew(now);
            } else if self.continue_route
                && self.phase == "drive"
                && !self.open_loop_orbit()
                && self.handover_observed
                && self.route_stage == "first_target"
                && self.first_pass_preparing
                && self.first_pass_evidence.is_none()
                && NEUTRAL <= self.servo
                && self.servo <= SERVO_MAX
                && self.full_target_points(&self.compact_target.clone(), scan, &self.orbit_track_id.clone()).is_some()
                && fresh()
                && control.get("command_acked").is_true()
                && motor_in(control.get("motor"), &DRIVE_MOTORS)
                && eq_i(control.get("servo"), self.servo)
            {
                let target = match &self._exit_release_cache {
                    Some((_, value)) => NEUTRAL.max(self.servo.min(*value)),
                    None => NEUTRAL,
                };
                self.steering_target = target;
                self.natural_steering_target = Some(target);
                self.reason = "first_target_pass_preparing_neutral".into();
                self.slew(now);
            }
            return self.result(now);
        }
        if self.continue_route {
            match self.route_stage {
                "light_turn" => return self.light_update(scan, now, control),
                "light_lane" => return self.light_lane_update(scan, now, control),
                "corridor_follow" => return self.corridor_follow_update(scan, now, control),
                "second_orbit" => return self.second_orbit_update(scan, now, control),
                "open_transfer" => return self.open_transfer_update(scan, now, control),
                "second_loop" => return self.open_second_loop_update(scan, now, control, false),
                "light_entry" => return self.open_light_entry_update(scan, now, control),
                _ => {}
            }
        }
        if self.first_pass_evidence.is_some() {
            return self.right_exit_update(scan, now, control);
        }
        if self.continue_route && self.entry_kind == Some("lane") && self.handover_observed && self.route_stage == "first_target" {
            return self.open_first_loop_update(scan, now, control);
        }
        let held = self.target_tracker.held.clone();
        if self.continue_route
            && self.compact_target.is_none()
            && self.handover_observed
            && held.is_dict()
            && py_eq(held.get("track_id"), &self.orbit_track_id)
        {
            let last_front = held_support_points(&held).iter().map(|p| p.0).fold(None, |m: Option<f64>, x| {
                Some(match m {
                    None => x,
                    Some(m) => pmax(m, x),
                })
            });
            if self.first_pass_preparing || last_front.is_some_and(|f| f <= FRONT_BODY_EXTENT_M) {
                self.steering_target = self.exit_release_target(scan, now);
                self.reason = "first_target_missing_frame_release".into();
            } else {
                self.steering_target = self.servo;
                self.reason = "first_target_missing_frame_hold".into();
            }
            self.slew(now);
            return self.result(now);
        }
        if !self.usable_target(&self.compact_target.clone(), scan) {
            self.clear_exit_prepare_window();
            self.capture_compact_loss(scan, now, control, "compact_target_lost_or_ambiguous");
            if self.continue_route && self.begin_quality_wait(&V::Float(now), control, Some(scan)) {
                self.reason = "route_orbit_target_recovery_wait".into();
            } else if !self.begin_adopted_left_coast("compact_target_lost_or_ambiguous", now, control, true) {
                self.begin_coast("compact_target_lost_or_ambiguous", now);
            }
            self.slew(now);
            return self.result(now);
        }
        let mut just_handed_over = false;
        if !self.handover_observed {
            let motors: &[i64] = if self.continue_route { &DRIVE_MOTORS } else { &[TRIAL_MOTOR] };
            let adopted = motor_in(control.get("motor"), motors)
                && eq_i(control.get("servo"), self.servo)
                && self.servo > NEUTRAL
                && control.get("command_acked").is_true();
            if !adopted {
                if self.handover_wait_since.is_none() {
                    self.handover_wait_since = Some(now);
                }
                if now - self.handover_wait_since.unwrap() + 1e-9 >= SCAN_AGE_S {
                    self.begin_coast("compact_handover_adoption_unconfirmed", now);
                    self.slew(now);
                } else {
                    self.reason = "compact_handover_waiting_adopted_left".into();
                }
                return self.result(now);
            }
            self.handover_observed = true;
            self._route_left_hold_since = None;
            if self.continue_route {
                self.target_tracker.widen_acquisition((radians(45.), PI));
            }
            just_handed_over = true;
            self.orbit_since = Some(now);
            self.orbit_reference_range_m = self.compact_target.idx("range_m").clone();
            self.orbit_bias_pwm = control.idx("servo").clone();
            self.orbit_left_entry_boost = self.compact_target.idx("bearing_left_rad").f() < ORBIT_BEARING_RAD;
            let mut handover = V::dict();
            for k in ["motor", "servo", "tick", "seq"] {
                handover.set(k, control.idx(k).clone());
            }
            handover.update(&dict! {
                "command_acked" => true, "physical_steering_confirmed" => false,
                "target_source_seq" => scan.idx("seq").clone(), "target_source_at_ms" => scan.idx("at_ms").clone(),
            });
            self.handover_control = handover;
            self.orbit_track_id = self.compact_target.idx("track_id").clone();
            if self.continue_route {
                let id = self.orbit_track_id.clone();
                self.target_tracker.enable_close_maintenance(&id);
            }
            self.wall_ambiguity_hold = false;
            self.geometry_missing = true;
            self.alignment_count = 0;
            self.alignment_evidence = None;
            self.alignment_publication = None;
            self.alignment_receive = None;
        } else if !py_eq(self.compact_target.idx("track_id"), &self.orbit_track_id) {
            self.clear_exit_prepare_window();
            self.capture_compact_loss(scan, now, control, "compact_target_identity_changed");
            if self.continue_route && self.begin_quality_wait(&V::Float(now), control, Some(scan)) {
                self.reason = "route_orbit_target_recovery_wait".into();
            } else if !self.begin_adopted_left_coast("compact_target_identity_changed", now, control, true) {
                self.begin_coast("compact_target_identity_changed", now);
            }
            self.slew(now);
            return self.result(now);
        }
        if !self.continue_route && now - self.orbit_since.unwrap() + 1e-9 >= ORBIT_ENTRY_MAX_S {
            self.begin_coast("first_relative_object_entry_trial_timeout", now);
            self.slew(now);
            return self.result(now);
        }
        if just_handed_over {
            let bias = self.orbit_bias_pwm.i();
            self.steering_target = bias;
            self.natural_steering_target = Some(bias);
            self.reason = "first_relative_object_handover_keep_adopted_left".into();
            return self.result(now);
        }
        self.observe_first_pass(scan, now, control);
        if self.first_pass_evidence.is_some() {
            return self.right_exit_update(scan, now, control);
        }
        if self.first_pass_preparing {
            let t = self.exit_release_target(scan, now);
            self.steering_target = t;
            self.natural_steering_target = Some(t);
            self.reason = "first_target_pass_preparing_neutral".into();
            self.slew(now);
            return self.result(now);
        }
        let target = self.compact_target.clone();
        let relative = self.relative_target_pwm(&target);
        let lookahead = pmax(0.35, pmin(1., target.idx("range_m").f()));
        self.steering_target = self.limit_left_for_known_points(relative, lookahead);
        self.natural_steering_target = Some(self.steering_target);
        self.reason = "first_relative_object_left_orbit_entry".into();
        self.slew(now);
        self.result(now)
    }
}

/// _held_support_points(held).
pub fn held_support_points(held: &V) -> Vec<(f64, f64)> {
    if !held.is_dict() || !held.get("point_left_m").is_list() {
        return vec![];
    }
    let p = held.idx("point_left_m").l();
    if p.len() != 2 {
        value_error("too many values to unpack")
    }
    let (x, y) = (p[0].f(), p[1].f());
    let d = held.get("diameter_m");
    let radius = (if d.truthy() { d.f() } else { 0. }) / 2.;
    vec![(x - radius, y), (x + radius, y)]
}

/// The tracker's bins as indices (validated by the caller).
pub fn bins_of(v: &V) -> Vec<usize> {
    v.l().iter().map(|i| i.i() as usize).collect()
}

pub fn ints_of(v: &V) -> Vec<i64> {
    v.l().iter().map(|i| i.i()).collect()
}

pub fn ranges_of(scan: &V) -> Rc<Vec<Option<f64>>> {
    floats(scan.idx("ranges"))
}

pub const TOLERANT_MAINT_MIN: i64 = TOLERANT_MAINTENANCE_MIN_POINTS;
