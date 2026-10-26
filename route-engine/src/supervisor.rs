//! server.py's turn-cone route supervision (round 97, part B), line for line,
//! for the standalone control program: start admission (turn_preview,
//! start_turn_trial), the tick (autonomy_tick's turn path, turn_tick,
//! coast_tick, begin_coast, halt), the quality wait/coast eligibility checks,
//! emit with the steering calibration (steer_calibration.SteerOutput) and its
//! feedback translation, QualityLatch and QualityRecovery (autonomy_live).
//! Only route sessions (turn-cone with continue_route). Every protection keeps
//! its Python value: owner lease, .12 s control gap, total and cumulative
//! drive deadlines, the 8 cm body gate with the cone exception, sensor ages
//! and health, bridge arm/lock feedback, ACK sequences, output limits.
//!
//! Inputs (scan, bridge status, clocks, sensor ages, the owner lease) are the
//! caller's: the program's own I/O, or the console's state in the dual run
//! against Python (scripts/route-engine-supervisor-parity.py). Commands go to
//! `emits` (and the caller's writer); a failed write raises RuntimeError as
//! the console's emit does.

use std::collections::BTreeMap;

use crate::clearance::{front_gap_evidence, probe_clearance, ClearanceReuse, Probe};
use crate::coast::{CoastWorker, COAST_MIN_VALID_POINTS};
use crate::dict;
use crate::helpers::*;
use crate::maneuver::*;
use crate::py::*;

pub const HEARTBEAT_S: f64 = 0.20;
pub const QUALITY_CONFIRM_S: f64 = 2.0;
pub const QUALITY_RECOVERY_STABLE_S: f64 = 0.30;
pub const QUALITY_RECOVERY_MIN_SCANS: i64 = 3;
pub const AUTO_CONTROL_HEALTH_S: f64 = 0.25;
pub const MANEUVER_CONTROL_LOOP_LIMIT_S: f64 = 0.12;
pub const QUALITY_PAUSE_ACK_GRACE_S: f64 = 0.1;
pub const COMPACT_TARGET_TRIAL_SCOPE: &str = "first_lidar_compact_target_orbit_entry";
const NOMINAL_PWM_PER_CURVATURE: f64 = 160.;
const STEER_TABLE: [(i64, f64); 3] = [(1314, -1.2), (1500, 0.), (1720, 1.42)];
const PHYSICAL_RANGE: (i64, i64) = (1200, 1720);
const SESSION_REPORT_KEYS: [&str; 20] = [
    "continue_route", "route_stage", "quality_wait_active", "quality_wait_servo", "quality_wait_evidence",
    "quality_wait_used", "quality_resume_pending", "second_target", "left_turn_pause_used",
    "left_turn_pause_evidence", "left_turn_pause_geometry_error", "role_distinct_evidence",
    "route_right_output_authorized", "route_center_ack", "route_alignment_evidence", "second_center_ack",
    "second_pass_evidence", "second_acquisition_reason", "route_events", "two_target_observed_pass_complete",
];

// ---------------------------------------------------------------- autonomy_live

/// QualityLatch: diagnostic duration of a quality streak.
#[derive(Clone, Debug, Default)]
pub struct QualityLatch {
    since: Option<f64>,
}

impl QualityLatch {
    pub fn update(&mut self, issues: &V, now: f64) -> bool {
        if !issues.truthy() {
            self.since = None;
        } else if self.since.is_none() {
            self.since = Some(now);
        }
        self.since.is_some_and(|since| now - since >= QUALITY_CONFIRM_S)
    }

    pub fn elapsed_ms(&self, now: f64) -> i64 {
        match self.since {
            None => 0,
            Some(since) => round(pmax(0., now - since) * 1000.),
        }
    }
}

/// QualityRecovery: resume a neutral-waiting session after stable fresh evidence.
#[derive(Clone, Debug, Default)]
pub struct QualityRecovery {
    pub waiting: bool,
    good_since: Option<f64>,
    good_scans: i64,
    last_seq: Option<i64>,
    last_good_at: Option<f64>,
}

impl QualityRecovery {
    pub fn update(&mut self, motion_ready: bool, issues: &V, scan_seq: &V, now: f64) -> bool {
        let seq = if scan_seq.is_int() { Some(scan_seq.i()) } else { None };
        if !motion_ready {
            self.waiting = true;
            self.good_since = None;
            self.good_scans = 0;
            self.last_seq = seq;
            self.last_good_at = None;
            return false;
        }
        if !self.waiting {
            return true;
        }
        if issues.truthy() || !scan_seq.is_int() {
            self.good_since = None;
            self.good_scans = 0;
            self.last_seq = seq;
            self.last_good_at = None;
            return false;
        }
        let seq = scan_seq.i();
        if let Some(last) = self.last_seq
            && seq <= last
        {
            if seq < last {
                self.good_since = None;
                self.good_scans = 0;
                self.last_seq = Some(seq);
                self.last_good_at = None;
            }
            return false;
        }
        self.last_seq = Some(seq);
        if self.last_good_at.is_some_and(|at| now - at >= LIDAR_LATE_S) {
            self.good_since = None;
            self.good_scans = 0;
        }
        self.last_good_at = Some(now);
        if self.good_since.is_none() {
            self.good_since = Some(now);
        }
        self.good_scans += 1;
        if self.good_scans >= QUALITY_RECOVERY_MIN_SCANS && now - self.good_since.unwrap() >= QUALITY_RECOVERY_STABLE_S {
            self.waiting = false;
            return true;
        }
        false
    }

    pub fn stable_ms(&self, now: f64) -> i64 {
        match self.good_since {
            None => 0,
            Some(since) => round(pmax(0., now - since) * 1000.),
        }
    }
}

// ---------------------------------------------------------------- steer_calibration

fn wheel_for_curvature(kappa: f64) -> f64 {
    if kappa <= STEER_TABLE[0].1 {
        return STEER_TABLE[0].0 as f64;
    }
    for pair in STEER_TABLE.windows(2) {
        let ((w0, k0), (w1, k1)) = (pair[0], pair[1]);
        if kappa <= k1 {
            return w0 as f64 + (w1 - w0) as f64 * (kappa - k0) / (k1 - k0);
        }
    }
    STEER_TABLE[2].0 as f64
}

/// SteerOutput: nominal route PWM -> physical servo PWM with play compensation.
#[derive(Clone, Debug)]
pub struct SteerOutput {
    play: i64,
    wheel: f64,
    last: i64,
}

impl Default for SteerOutput {
    fn default() -> SteerOutput {
        SteerOutput { play: STEER_PLAY_PWM, wheel: NEUTRAL as f64, last: NEUTRAL }
    }
}

impl SteerOutput {
    pub fn physical(&mut self, nominal: &V, play: Option<i64>) -> i64 {
        if !nominal.is_int() {
            value_error("nominal servo must be an int PWM")
        }
        let play = play.unwrap_or(self.play);
        if !(0..=self.play).contains(&play) {
            value_error("steering play out of range")
        }
        let target = wheel_for_curvature((nominal.i() - NEUTRAL) as f64 / NOMINAL_PWM_PER_CURVATURE);
        let command = if target > self.wheel + 0.5 {
            target + play as f64
        } else if target < self.wheel - 0.5 {
            target - play as f64
        } else if (self.last as f64 - target).abs() <= (play + 1) as f64 {
            self.last as f64
        } else {
            target
        };
        let bounded = pmax(PHYSICAL_RANGE.0 as f64, pmin(PHYSICAL_RANGE.1 as f64, command));
        let command = round(bounded);
        self.wheel = pmin((command + play) as f64, pmax((command - play) as f64, self.wheel));
        self.last = command;
        command
    }
}

// ---------------------------------------------------------------- the console's state

/// One bridge command as the console's emit sends it.
#[derive(Clone, Debug)]
pub struct Emit {
    pub op: &'static str,
    pub seq: i64,
    pub tick: V,
    pub motor: V,
    /// The nominal servo the route decided.
    pub servo: V,
    /// What goes to the bridge (calibrated physical servo for route drives).
    pub physical: V,
}

impl Emit {
    /// The bridge row {"op", "seq", "tick", "motor", "servo"}.
    pub fn row(&self) -> V {
        dict! {"op" => self.op, "seq" => self.seq, "tick" => self.tick.clone(), "motor" => self.motor.clone(),
               "servo" => self.physical.clone()}
    }
}

/// Where a tick goes after a step (Python's `self.halt(...); return`).
enum Flow {
    Continue,
    Halt(String),
}

fn halt(reason: impl Into<String>) -> Flow {
    Flow::Halt(reason.into())
}

/// One route run (the console's auto_session).
pub struct Session {
    pub report: V,
    pub motion: Maneuver,
    pub phase: String,
    pub deadline: f64,
    pub last_loop: f64,
    pub heartbeat_seq: i64,
    pub recovery: QualityRecovery,
    pub motion_scan_seq: V,
    pub turn_last_servo: V,
    pub turn_servo_sequence: Option<i64>,
    pub turn_servo_sent_at: V,
    pub steer_output: SteerOutput,
    pub clearance_reuse: ClearanceReuse,
    pub coast_since: Option<V>,
    pub coast_deadline: Option<f64>,
    pub coast_reason: Option<String>,
    pub motion_generation: Option<i64>,
    pub motion_ready_for_coast: Option<bool>,
    pub coast_start_seq: V,
    pub coast_neutral_sequence: Option<i64>,
    pub coast_endpoint_verified: Option<bool>,
    pub quality_wait_neutral_sequence: Option<i64>,
    pub quality_resume_sequence: Option<i64>,
    pub quality_wait_kind: Option<&'static str>,
    pub quality_wait_entry_neutral: Option<bool>,
    pub quality_wait_front_sparse_entry: Option<bool>,
    pub route_quality_coast: Option<bool>,
}

/// The console's state for route runs (one vehicle, one owner at a time).
pub struct Supervisor {
    pub demo: bool,
    pub boot: String,
    pub route_controller: String,
    pub owner: Option<String>,
    pub owner_at: f64,
    pub control_mode: &'static str,
    pub control_epoch: i64,
    pub session: Option<Session>,
    pub auto_result: V,
    pub coast_worker: CoastWorker,
    pub steer_sent: BTreeMap<i64, (V, V)>,
    pub perception_quality: QualityLatch,
    pub arm_sequence: i64,
    pub last_stop: V,
    pub stop_latched: bool,
    pub stopped_tick: V,
    pub sequence: i64,
    pub status: V,
    pub control_at: f64,
    pub scan: V,
    pub scan_at: V,
    pub camera_at: f64,
    /// False in the standalone program (no camera): camera age None.
    pub camera: bool,
    pub errors: V,
    /// time.monotonic() during the current call (the console reads it inside
    /// the tick for the control health check and the lane scope).
    pub mono: f64,
    /// console.sensor_ages(now) when the caller supplies it (the dual run);
    /// otherwise computed from the clocks.
    pub ages_override: Option<V>,
    pub emits: Vec<Emit>,
    pub events: Vec<V>,
    pub writer: Option<Box<dyn FnMut(&Emit) -> Result<(), String>>>,
    pub unix_time: fn() -> f64,
}

fn unix_now() -> f64 {
    std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs_f64()).unwrap_or(0.)
}

fn num(value: &V) -> f64 {
    // A comparison operand: numbers (bool as int); anything else is Python's TypeError.
    match value {
        V::Int(_) | V::Float(_) | V::Bool(_) => value.f(),
        other => type_error(format!("'<' not supported for {}", other.type_name())),
    }
}

/// `control.get('seq', -1)` as a number.
fn seq_of(control: &V) -> f64 {
    if control.has("seq") { num(control.get("seq")) } else { -1. }
}

fn tick_of(control: &V) -> f64 {
    if control.has("tick") { num(control.get("tick")) } else { -1. }
}

fn in_motors(value: &V, motors: &[i64]) -> bool {
    motors.iter().any(|&m| py_eq(value, &V::Int(m)))
}

fn issues_set(value: &V) -> Vec<String> {
    match value {
        V::List(items) => items.iter().filter_map(|i| i.os().map(str::to_string)).collect(),
        _ => Vec::new(),
    }
}

/// `set(issues) <= allowed` for a list of issue strings.
fn issues_within(value: &V, allowed: &[&str]) -> bool {
    value.ol().unwrap_or(&[]).iter().all(|i| i.os().is_some_and(|s| allowed.contains(&s)))
}

/// `set(a) == set(b)` for lists of issue strings.
fn same_issue_set(a: &V, b: &V) -> bool {
    let (mut x, mut y) = (issues_set(a), issues_set(b));
    x.sort();
    x.dedup();
    y.sort();
    y.dedup();
    x == y
}

fn incr(report: &mut V, key: &str) {
    let n = report.get(key).i();
    report.set(key, n + 1);
}

fn incr_or_zero(report: &mut V, key: &str) {
    let n = if report.has(key) { report.get(key).i() } else { 0 };
    report.set(key, n + 1);
}

fn opt_i(value: Option<i64>) -> V {
    value.map(V::Int).unwrap_or(V::None)
}

fn opt_f(value: Option<f64>) -> V {
    value.map(V::Float).unwrap_or(V::None)
}

impl Supervisor {
    pub fn new(demo: bool, boot: &str, route_controller: &str) -> Supervisor {
        Supervisor {
            demo,
            boot: boot.to_string(),
            route_controller: route_controller.to_string(),
            owner: None,
            owner_at: 0.,
            control_mode: "locked",
            control_epoch: 0,
            session: None,
            auto_result: V::None,
            coast_worker: CoastWorker::new(),
            steer_sent: BTreeMap::new(),
            perception_quality: QualityLatch::default(),
            arm_sequence: 0,
            last_stop: V::None,
            stop_latched: true,
            stopped_tick: V::Int(-1),
            sequence: 0,
            status: V::dict(),
            control_at: 0.,
            scan: V::None,
            scan_at: V::Int(0),
            camera_at: 0.,
            camera: true,
            errors: V::dict(),
            mono: 0.,
            ages_override: None,
            emits: Vec::new(),
            events: Vec::new(),
            writer: None,
            unix_time: unix_now,
        }
    }

    /// `self.status.get('control', {})`.
    pub fn control(&self) -> V {
        if self.status.has("control") { self.status.get("control").clone() } else { V::dict() }
    }

    fn scan_at_f(&self) -> f64 {
        self.scan_at.f()
    }

    /// `{**self.scan, 'received_at': self.scan_at}`.
    fn received_scan(&self) -> V {
        self.scan.merged(&dict! {"received_at" => self.scan_at.clone()})
    }

    pub fn sensor_ages(&self, now: f64) -> V {
        if let Some(ages) = &self.ages_override {
            return ages.clone();
        }
        let camera = if self.camera { V::Float(now - self.camera_at) } else { V::None };
        dict! {"camera" => camera, "lidar" => now - self.scan_at_f(), "control" => now - self.control_at}
    }

    pub fn turn_healthy(&self) -> bool {
        self.mono - self.control_at < AUTO_CONTROL_HEALTH_S
            && !self.errors.items().iter().any(|(kind, _)| &**kind != "camera")
    }

    /// output_servo(op, servo, sequence): the physical servo of a route drive.
    fn output_servo(&mut self, session: Option<&mut Session>, op: &str, servo: &V, sequence: i64) -> V {
        let Some(s) = session else { return servo.clone() };
        if op != "drive" {
            return servo.clone();
        }
        let lane = s.motion.lane_wall_output_active();
        // Round 107: the first loop's left commands divided by FIRST_LOOP_STEER_GAIN.
        let command = match servo {
            V::Int(v) if !lane && *v > NEUTRAL && s.motion.first_loop_output_active() => {
                V::Int(NEUTRAL + round((*v - NEUTRAL) as f64 / FIRST_LOOP_STEER_GAIN))
            }
            _ => servo.clone(),
        };
        let physical = V::Int(s.steer_output.physical(&command, if lane { Some(LANE_WALL_PLAY_PWM) } else { None }));
        self.steer_sent.insert(sequence, (servo.clone(), physical.clone()));
        self.steer_sent.retain(|&k, _| k >= sequence - 64);
        physical
    }

    /// nominal_status(value): feedback of a calibrated command in nominal PWM.
    pub fn nominal_status(&self, value: &V) -> V {
        let control = if value.is_dict() { value.get("control") } else { &V::None };
        if !control.is_dict() || self.steer_sent.is_empty() {
            return value.clone();
        }
        let seq = control.get("seq");
        let sent = if seq.is_int() || seq.is_bool() || (seq.is_float() && seq.f().fract() == 0.) {
            self.steer_sent.get(&(seq.f() as i64))
        } else {
            None
        };
        let Some((nominal, physical)) = sent else { return value.clone() };
        if !py_eq(control.get("servo"), physical) {
            return value.clone();
        }
        let mut control = control.clone();
        control.set("servo", nominal.clone());
        control.set("physical_servo", physical.clone());
        value.merged(&dict! {"control" => control})
    }

    /// emit(op, motor, servo, tick).
    fn emit(&mut self, session: Option<&mut Session>, op: &'static str, motor: V, servo: V, tick: V) {
        self.sequence += 1;
        let sequence = self.sequence;
        let physical = self.output_servo(session, op, &servo, sequence);
        let emit = Emit { op, seq: sequence, tick, motor, servo, physical };
        let written = match self.writer.as_mut() {
            Some(writer) => writer(&emit),
            None => Ok(()),
        };
        let row = emit.row();
        self.emits.push(emit);
        if let Err(error) = written {
            self.errors.set("control_write", error);
            raise(Exc::RuntimeError, "control bridge unavailable")
        }
        let unix = (self.unix_time)();
        self.events.push(dict! {"unix_s" => unix, "request" => row});
    }

    /// halt(reason): lock, end the session with its result, stop the bridge.
    pub fn halt(&mut self, reason: &str) {
        if !self.stop_latched || self.last_stop.is_none() {
            let unix = (self.unix_time)();
            self.last_stop = dict! {"reason" => reason, "unix_s" => unix};
        }
        self.stop_latched = true;
        self.control_epoch += 1;
        self.control_mode = "locked";
        if let Some(session) = self.session.take() {
            let report = &session.report;
            let formal_complete = reason == "formal_goal_complete"
                && get_or(report, "formal_completion_evidence", &V::Bool(false)).truthy()
                && get_or(report, "coast_neutral_ack", &V::Bool(false)).truthy()
                && get_or(report, "observed_armed", &V::Bool(false)).truthy();
            let completed = formal_complete
                || (!report.get("formal_stop_goal").truthy()
                    && !report.get("turn_trial").truthy()
                    && ["probe_complete", "left_junction_reached"].contains(&reason)
                    && get_or(report, "observed_armed", &V::Bool(false)).truthy()
                    && (!report.get("centering").truthy()
                        || get_or(report, "standstill_confirmed", &V::Bool(false)).truthy()));
            let mut result = report.clone();
            result.set("reason", reason);
            result.set("completed", completed);
            self.auto_result = result;
        }
        self.steer_sent.clear();
        self.coast_worker.reset();
        self.owner = None;
        let control = self.control();
        self.stopped_tick = if control.has("tick") { control.get("tick").clone() } else { V::Int(-1) };
        let unix = (self.unix_time)();
        self.events.push(dict! {"unix_s" => unix, "stop_reason" => reason});
        let _ = py_try(|| self.emit(None, "stop", V::Int(NEUTRAL), V::Int(NEUTRAL), V::Int(0)));
    }
}

// ---------------------------------------------------------------- turn helpers

fn scan_clock_ok(scan: &V) -> bool {
    scan.is_dict()
        && scan.get("frame_id").os() == Some(FRAME)
        && scan.get("seq").is_int()
        && scan.get("seq").i() >= 0
        && scan.get("at_ms").is_int()
        && scan.get("at_ms").i() >= 0
}

fn candidates_ok(candidates: &V) -> bool {
    match candidates {
        V::List(list) => list.len() <= 24 && list.iter().all(valid_candidate_default),
        _ => false,
    }
}

/// turn_neutral_scan_bounded(scan).
fn neutral_scan_bounded(scan: &V) -> bool {
    match scan.get("ranges") {
        V::List(ranges) => {
            ranges.len() == 360
                && ranges.iter().all(|r| r.is_none() || (r.is_num() && (0.02..=12.).contains(&r.f())))
                && ranges.iter().filter(|r| r.is_some()).count() >= COAST_MIN_VALID_POINTS
        }
        _ => false,
    }
}

fn floats_of(ranges: &V) -> Vec<Option<f64>> {
    ranges.l().iter().map(|r| r.of()).collect()
}

impl Supervisor {
    /// turn_rear_start(session).
    fn turn_rear_start(s: &Session) -> bool {
        s.motion.continue_route
            && s.motion.route_stage == "start_lane"
            && ["presteer", "drive", "coast"].contains(&s.motion.phase)
    }

    /// turn_front_lane(session, now, scope_only): None/False, True, or the lane.
    fn turn_front_lane(&self, s: &Session, now: f64, scope_only: bool) -> V {
        let m = &s.motion;
        let scan = &self.scan;
        let in_scope = m.continue_route
            && m.entry_kind == Some("lane")
            && (m.route_stage == "start_lane" || m.route_stage == "light_lane")
            && ((["drive", "quality_wait"].contains(&s.phase.as_str()) && m.drive_since.is_some())
                || (scope_only && s.phase == "presteer" && m.route_stage == "start_lane"))
            && s.phase == m.phase
            && scan_clock_ok(scan)
            && (0. ..SCAN_AGE_S).contains(&(now - self.scan_at_f()))
            && !m.last_seq.is_some_and(|last| scan.get("seq").i() < last)
            && !m.last_publication.is_some_and(|last| scan.get("at_ms").i() < last);
        if !in_scope {
            return if scope_only { V::Bool(false) } else { V::None };
        }
        if scope_only {
            return V::Bool(true);
        }
        let candidates = scan.get("corridor_candidates");
        if !candidates_ok(candidates) {
            return V::None;
        }
        let forward = Maneuver::forward_lanes(candidates.l(), true);
        if forward.len() == 1 { forward[0].clone() } else { V::None }
    }

    /// turn_front_walls(session, now): (lane or None, side-wall flag).
    fn turn_front_walls(&self, s: &Session, now: f64) -> (V, bool) {
        let lane = self.turn_front_lane(s, now, false);
        if lane.is_some() {
            return (lane, false);
        }
        (V::None, self.turn_front_lane(s, now, true).is_true())
    }

    /// turn_route_quality_scope(session, now).
    fn turn_route_quality_scope(&self, s: &Session, now: f64) -> Option<&'static str> {
        let m = &s.motion;
        if !m.continue_route {
            return None;
        }
        if m.handover_observed {
            return Some("post_handover");
        }
        let scan = &self.scan;
        let fresh = |scan: &V| scan_clock_ok(scan) && (0. ..SCAN_AGE_S).contains(&(now - self.scan_at_f()));
        if m.entry_kind == Some("lane") && m.route_stage == "start_lane" {
            if s.phase != "drive" || !fresh(scan) {
                return None;
            }
            return if m.start_lane_quality_geometry(scan) { Some("start_lane_current_geometry") } else { None };
        }
        if m.drive_since.is_none()
            || m.route_stage != "first_target"
            || s.phase != m.phase
            || !(m.phase == "drive" || (m.phase == "quality_wait" && m.neutral_pause_kind == Some("left_turn")))
        {
            return None;
        }
        if !fresh(scan) {
            return None;
        }
        let candidates = scan.get("corridor_candidates");
        if !candidates_ok(candidates) {
            return None;
        }
        if m.last_geometry_seq.is_some_and(|last| scan.get("seq").i() < last)
            || m.last_geometry_publication.is_some_and(|last| scan.get("at_ms").i() < last)
        {
            return None;
        }
        let dt = match m.last_geometry_publication {
            None => 0.,
            Some(last) => (scan.get("at_ms").i() - last) as f64 / 1000.,
        };
        // _measurement on an isolated copy: the check cannot refresh a live
        // geometry lease or steering history.
        let mut probe = m.clone();
        match except(VTK, || probe.measurement(scan, candidates, dt)) {
            Ok(_) => Some("left_turn_current_geometry"),
            Err(_) => None,
        }
    }

    /// turn_front_evidence_matches(scan, clearance, front_lane, side_walls).
    fn turn_front_evidence_matches(scan: &V, clearance: &V, front_lane: &V, side_walls: bool) -> V {
        let evidence = front_gap_evidence(&floats_of(scan.idx("ranges")), true, front_lane, side_walls);
        if ["front_wall_projected_bins", "front_wall_projection"].iter().any(|k| clearance.has(k) && !evidence.has(k)) {
            return V::None;
        }
        if evidence.items().iter().all(|(k, value)| py_eq(clearance.get(k), value)) { evidence } else { V::None }
    }

    /// turn_quality_hold_eligible(session, route_entry=).
    fn turn_quality_hold_eligible(&self, s: &Session, route_entry: bool) -> bool {
        let report = &s.report;
        let empty = V::list(Vec::new());
        let issues = get_or(report, "quality_issues", &empty);
        let clearance = report.get("clearance_current");
        let scan = &self.scan;
        if !clearance.is_dict()
            || !scan.is_dict()
            || !scan.get("seq").is_int()
            || scan.get("seq").i() < 0
            || !clearance.get("scan_seq").is_int()
            || !issues_within(issues, &["scan_incomplete", "front_sparse"])
            || !py_eq(clearance.get("scan_seq"), scan.get("seq"))
            || !same_issue_set(get_or(clearance, "quality_issues", &empty), issues)
        {
            return false;
        }
        // Route sessions only (continue_route): the raw-return floor applies.
        if !neutral_scan_bounded(scan) {
            return false;
        }
        if issues_set(issues).iter().any(|i| i == "front_sparse") {
            let control = self.control();
            let neutral_seq = s.coast_neutral_sequence;
            let route_coast = s.route_quality_coast == Some(true) && s.phase == "coast";
            if !route_entry
                && !route_coast
                && (s.phase != "coast"
                    || neutral_seq.is_none()
                    || !py_eq(control.get("motor"), &V::Int(NEUTRAL))
                    || seq_of(&control) < neutral_seq.unwrap() as f64)
            {
                return false;
            }
            let ranges = scan.get("ranges");
            if !ranges.is_list() || ranges.len() != 360 {
                return false;
            }
            let (lane, side_walls) = self.turn_front_walls(s, self.mono);
            let evidence = Supervisor::turn_front_evidence_matches(scan, clearance, &lane, side_walls);
            if evidence.is_none() {
                return false;
            }
        }
        true
    }

    /// turn_quality_coast_active(session).
    fn turn_quality_coast_active(&self, s: &Session) -> bool {
        let m = &s.motion;
        let held = m.quality_coast_servo;
        let control = self.control();
        let neutral_seq = s.coast_neutral_sequence;
        s.phase == m.phase
            && m.phase == "coast"
            && (m.handover_observed || s.route_quality_coast == Some(true))
            && m.first_pass_evidence.is_none()
            && m.right_exit_since.is_none()
            && held.is_some_and(|h| NEUTRAL < h && h <= SERVO_MAX)
            && self.turn_quality_hold_eligible(s, false)
            && control.get("armed").is_true()
            && py_eq(control.get("servo"), &opt_i(held))
            && seq_of(&control) >= self.arm_sequence as f64
            && (py_eq(control.get("motor"), &V::Int(NEUTRAL))
                || (in_motors(control.get("motor"), &DRIVE_MOTORS)
                    && neutral_seq.is_none_or(|n| seq_of(&control) < n as f64)))
    }

    fn wait_lower(s: &Session) -> i64 {
        if matches!(s.quality_wait_kind, Some("post_pass_pause" | "recovery_hold")) { right_exit_min_pwm() } else { NEUTRAL }
    }

    /// turn_quality_wait_eligible(session).
    fn turn_quality_wait_eligible(&self, s: &Session) -> bool {
        let m = &s.motion;
        let control = self.control();
        let held = m.quality_wait_servo;
        let report = &s.report;
        let empty = V::list(Vec::new());
        let clearance = report.get("clearance_current");
        let scan = &self.scan;
        let issues = get_or(report, "quality_issues", &empty);
        let neutral_seq = s.quality_wait_neutral_sequence;
        if !m.continue_route
            || s.phase != "quality_wait"
            || m.phase != "quality_wait"
            || !clearance.is_dict()
            || !scan.is_dict()
            || !scan.get("seq").is_int()
            || scan.get("seq").i() < 0
            || !clearance.get("scan_seq").is_int()
            || !py_eq(clearance.get("scan_seq"), scan.get("seq"))
            || !same_issue_set(get_or(clearance, "quality_issues", &empty), issues)
            || !issues_within(issues, &["scan_incomplete", "front_sparse"])
            || held.is_none()
            || !(Supervisor::wait_lower(s) <= held.unwrap() && held.unwrap() <= SERVO_MAX)
            || !control.get("armed").is_true()
            || !self.turn_wait_servo_matches(s, &control)
            || seq_of(&control) < self.arm_sequence as f64
        {
            return false;
        }
        let neutral_ack = neutral_seq.is_some_and(|n| seq_of(&control) >= n as f64)
            && py_eq(control.get("motor"), &V::Int(NEUTRAL));
        // Pending neutral only before its first ACK (round 58: a light-lane
        // wait entered during a neutral pulse may already read 1500).
        let pending_motors: Vec<i64> = if s.quality_wait_entry_neutral == Some(true) {
            vec![NEUTRAL, DRIVE_MOTORS[0], DRIVE_MOTORS[1], DRIVE_MOTORS[2]]
        } else {
            DRIVE_MOTORS.to_vec()
        };
        let pending_neutral = in_motors(control.get("motor"), &pending_motors)
            && neutral_seq.is_none_or(|n| seq_of(&control) < n as f64);
        if !neutral_ack && !pending_neutral {
            return false;
        }
        if !neutral_scan_bounded(scan) {
            return false;
        }
        if issues_set(issues).iter().any(|i| i == "front_sparse") {
            let (lane, side_walls) = self.turn_front_walls(s, self.mono);
            if Supervisor::turn_front_evidence_matches(scan, clearance, &lane, side_walls).is_none() {
                return false;
            }
        }
        true
    }

    /// turn_wait_servo_matches(session, control).
    fn turn_wait_servo_matches(&self, s: &Session, control: &V) -> bool {
        let m = &s.motion;
        let (held, previous) = (m.quality_wait_servo, m.quality_wait_previous_servo);
        let pending_seq = s.turn_servo_sequence;
        let servo = control.get("servo");
        let (Some(held), true) = (held, servo.is_int()) else { return false };
        if servo.i() == held {
            return true;
        }
        let lower = Supervisor::wait_lower(s);
        previous.is_some_and(|p| {
            servo.i() == p
                && p != held
                && lower <= held
                && held <= SERVO_MAX
                && lower <= p
                && p <= SERVO_MAX
                && s.motion.continue_route
                && pending_seq.is_some()
                && control.get("seq").is_int()
                && control.get("seq").i() < pending_seq.unwrap()
        })
    }

    /// turn_route_control(session, control): neutral/resume ACKs from feedback.
    fn turn_route_control(&self, s: &Session, control: &V) -> V {
        let current = control.get("armed").is_true()
            && control.get("seq").is_int()
            && control.get("seq").i() >= self.arm_sequence
            && self.turn_wait_servo_matches(s, control);
        let seq = |c: &V| c.get("seq").i();
        let neutral_acked = current
            && s.quality_wait_neutral_sequence.is_some_and(|n| seq(control) >= n)
            && py_eq(control.get("motor"), &V::Int(NEUTRAL));
        let resume_acked = current
            && s.quality_resume_sequence.is_some_and(|n| seq(control) >= n)
            && py_eq(control.get("motor"), &V::Int(TRIAL_MOTOR));
        control.merged(&dict! {"neutral_acked" => neutral_acked, "resume_acked" => resume_acked})
    }

    /// turn_quality_snapshot(session, control, now).
    fn turn_quality_snapshot(&self, s: &Session, control: &V, now: &V) -> V {
        let scan = &self.scan;
        let mut picked = V::dict();
        for key in ["tick", "seq", "motor", "servo", "armed", "command_acked", "neutral_acked", "resume_acked"] {
            picked.set(key, control.get(key).clone());
        }
        dict! {
            "continue_route" => true, "observed_at" => now.clone(),
            "scan" => dict! {"seq" => scan.idx("seq").clone(), "at_ms" => scan.idx("at_ms").clone(),
                             "received_at" => self.scan_at.clone(), "frame_id" => scan.get("frame_id").clone(),
                             "ranges" => V::list(scan.idx("ranges").l().to_vec())},
            "control" => picked,
            "compact_target" => s.motion.compact_target.clone(),
            "clearance" => s.report.get("clearance_current").clone(),
            "drive_since" => opt_f(s.motion.drive_since),
            "session_deadline" => s.deadline,
        }
    }

    /// record_compact_target_decision(session, decision).
    fn record_compact_target_decision(s: &mut Session, decision: &V) {
        let report = &mut s.report;
        report.update(&dict! {
            "trial_scope" => COMPACT_TARGET_TRIAL_SCOPE, "semantic_class" => "unknown",
            "left_turn_servo_cap" => SERVO_MAX, "competition_supported" => false, "completed" => false,
            "compact_target" => decision.get("compact_target").clone(),
            "handover_observed" => decision.get("handover_observed").truthy(),
            "orbit_entry_elapsed_s" => get_or(decision, "orbit_entry_elapsed_s", &V::Float(0.)).clone(),
        });
        for key in SESSION_REPORT_KEYS {
            report.set(key, decision.get(key).clone());
        }
    }
}

// ---------------------------------------------------------------- the tick

const TICK_EXCEPTIONS: &[Exc] = &[Exc::ValueError, Exc::KeyError, Exc::RuntimeError];

impl Supervisor {
    /// begin_coast(reason, now): the planned neutral phase.
    fn begin_coast(&mut self, s: &mut Session, reason: &str, now: &V) {
        if s.phase == "coast" {
            return;
        }
        s.phase = "coast".to_string();
        s.coast_since = Some(now.clone());
        s.coast_deadline = Some(now.f() + COAST_MAX_S);
        s.coast_reason = Some(reason.to_string());
        s.motion.begin_coast(reason, if now.is_num() { now.f() } else { f64::NAN });
        s.motion_generation = Some(self.coast_worker.reset());
        s.motion_ready_for_coast = Some(false);
        s.motion_scan_seq = V::None;
        s.coast_start_seq = self.scan.idx("seq").clone();
        s.coast_neutral_sequence = None;
        s.coast_endpoint_verified = Some(false);
        if reason == "left_junction_reached" {
            // Only the straight path's junction stop uses it; never a route reason.
            value_error("route_supervisor_left_junction_unsupported")
        }
        s.report.update(&dict! {
            "phase" => "coast", "planned_stop_reason" => reason, "current_pwm" => NEUTRAL, "coast_ticks" => 0,
            "standstill_confirmed" => false,
        });
    }

    /// The motion's decision into the report (coast_tick / turn_tick).
    fn report_decision(s: &mut Session, decision: &V) {
        let report = &mut s.report;
        report.set("turn", decision.clone());
        report.set("physical_steering_confirmed", false);
        report.set("observed_alignment", decision.idx("observed_alignment").clone());
        report.set("alignment_evidence", decision.idx("alignment_evidence").clone());
        report.set("entry_confirmed", decision.idx("entry_confirmed").clone());
        report.set("turn_stage", decision.idx("turn_stage").clone());
        Supervisor::record_compact_target_decision(s, decision);
    }

    /// coast_tick(session, now, motion_ready, servo).
    fn coast_tick(&mut self, s: &mut Session, now: &V, motion_ready: bool) -> Flow {
        let t = now.f();
        if t >= s.coast_deadline.unwrap_or(f64::INFINITY) {
            return halt("coast_standstill_unconfirmed");
        }
        let quality_hold = self.turn_quality_coast_active(s);
        if s.motion.quality_coast_servo.is_some() && !quality_hold {
            value_error("turn_quality_coast_feedback_changed")
        }
        let control = self.status.idx("control").clone();
        let pending_servo = s.turn_servo_sequence;
        let acked = pending_servo.is_none_or(|p| seq_of(&control) >= p as f64);
        let turn_control = control.merged(&dict! {"command_acked" => acked});
        let scan = self.received_scan();
        let lidar_age = s.report.idx("sensor_ages").idx("lidar").clone();
        let f = V::Bool(false);
        let decision = s.motion.update(&scan, &lidar_age, now, &turn_control, &V::Bool(motion_ready || quality_hold),
                                       false, &f, &f, &f);
        Supervisor::report_decision(s, &decision);
        if decision.idx("lock_requested").truthy() {
            return halt(decision.idx("reason").s());
        }
        let coast_min = SERVO_MIN.min(right_exit_min_pwm());
        let servo_v = decision.idx("servo");
        if !py_eq(decision.idx("motor"), &V::Int(NEUTRAL))
            || !(coast_min as f64 <= num(servo_v) && num(servo_v) <= SERVO_MAX as f64)
        {
            value_error("invalid_turn_coast_output")
        }
        let servo = servo_v.clone();
        if s.coast_reason.as_deref() == Some("left_junction_reached") {
            value_error("route_supervisor_left_junction_unsupported")
        }
        let control = self.status.idx("control").clone();
        let neutral_ack = s.coast_neutral_sequence.is_some_and(|n| seq_of(&control) >= n as f64)
            && py_eq(control.get("motor"), &V::Int(NEUTRAL))
            && control.get("armed").truthy();
        s.report.set("coast_neutral_ack", neutral_ack);
        if !motion_ready || !neutral_ack || num(self.scan.idx("seq")) <= num(&s.coast_start_seq) {
            if s.motion_ready_for_coast.unwrap() {
                s.motion_generation = Some(self.coast_worker.reset());
            }
            s.motion_ready_for_coast = Some(false);
            s.motion_scan_seq = V::None;
            s.report.set("coast_motion", CoastWorker::unknown(if !motion_ready {
                "perception_unavailable"
            } else {
                "awaiting_neutral_and_new_scan"
            }));
        } else {
            s.motion_ready_for_coast = Some(true);
            let (age, received) = (lidar_age.clone(), self.scan_at.clone());
            let generation = s.motion_generation.unwrap();
            let error = self.coast_worker.submit(generation, &self.scan, &age, &received, now);
            let motion = match error {
                Some(error) => CoastWorker::unknown(error),
                None => self.coast_worker.result(generation, &self.scan, &age, &received, now),
            };
            s.report.set("coast_motion", motion);
            if error.is_some() {
                s.motion_generation = Some(self.coast_worker.reset());
                s.motion_ready_for_coast = Some(false);
            }
            s.motion_scan_seq = s.report.get("coast_motion").get("source_scan_seq").clone();
        }
        let motion = s.report.get("coast_motion").clone();
        if motion.get("observable").truthy() && motion.get("stationary").truthy() {
            s.report.set("standstill_confirmed", true);
            let reason = s.coast_reason.clone().unwrap_or_default();
            return halt(reason);
        }
        // No branch in coast may restore positive motor PWM.
        if !py_eq(&servo, get_or(&s.report, "servo", &V::Int(NEUTRAL))) {
            incr(&mut s.report, "steering_changes");
        }
        s.report.set("servo", servo.clone());
        s.report.set("current_pwm", NEUTRAL);
        let tick = self.status.idx("control").idx("tick").clone();
        self.emit(Some(s), "drive", V::Int(NEUTRAL), servo.clone(), tick);
        if !py_eq(&servo, &s.turn_last_servo) {
            s.turn_last_servo = servo;
            s.turn_servo_sequence = Some(self.sequence);
            s.turn_servo_sent_at = now.clone();
        }
        if s.coast_neutral_sequence.is_none() {
            s.coast_neutral_sequence = Some(self.sequence);
        }
        incr(&mut s.report, "coast_ticks");
        Flow::Continue
    }

    /// autonomy_tick(now): one supervised step of the route session.
    pub fn autonomy_tick(&mut self, now: &V) {
        let Some(mut s) = self.session.take() else { return };
        // Any other exception leaves the session as Python leaves auto_session.
        let flow = py_try(|| self.tick_session(&mut s, now));
        self.session = Some(s);
        match flow {
            Ok(Flow::Continue) => {}
            Ok(Flow::Halt(reason)) => self.halt(&reason),
            Err(error) => raise(error.kind, error.msg),
        }
    }

    fn tick_session(&mut self, s: &mut Session, now: &V) -> Flow {
        let t = now.f();
        let limit = MANEUVER_CONTROL_LOOP_LIMIT_S;
        let gap = t - s.last_loop;
        let max_gap = pmax(get_or(&s.report, "max_actual_gap_s", &V::Float(0.)).f(), gap);
        s.report.update(&dict! {"control_loop_limit_s" => limit, "actual_gap_s" => gap, "max_actual_gap_s" => max_gap});
        if t - self.owner_at >= HEARTBEAT_S {
            return halt("autonomy_heartbeat_timeout");
        }
        if gap > limit {
            return halt("autonomy_control_gap");
        }
        s.last_loop = t;
        if s.phase == "coast" && t >= s.coast_deadline.unwrap_or(f64::INFINITY) {
            return halt("coast_standstill_unconfirmed");
        }
        match except(TICK_EXCEPTIONS, || self.tick_body(s, now)) {
            Ok(flow) => flow,
            Err(error) => halt(error.msg),
        }
    }

    fn tick_body(&mut self, s: &mut Session, now: &V) -> Flow {
        let t = now.f();
        let ages = self.sensor_ages(t);
        s.report.set("sensor_ages", ages.clone());
        s.report.set("sensor_errors", self.errors.clone());
        let control = self.control();
        s.report.set("rear_launch_active", false);
        let m = &s.motion;
        let relaxed_turn_left = s.phase == "drive"
            && m.phase == "drive"
            && (!m.continue_route
                || (m.first_pass_evidence.is_none()
                    && m.right_exit_since.is_none()
                    && m.route_stage == "first_target"
                    && !m.quality_resume_pending))
            && control.get("armed").is_true()
            && in_motors(control.get("motor"), &DRIVE_MOTORS)
            && control.get("servo").is_int()
            && NEUTRAL < control.get("servo").i()
            && control.get("servo").i() <= SERVO_MAX;
        let route_contact = s.motion.continue_route;
        let clearance_scan = if route_contact && self.scan.is_dict() { self.received_scan() } else { self.scan.clone() };
        let (front_lane, front_side_walls) = self.turn_front_walls(s, t);
        let contact_target =
            if route_contact { s.motion.current_contact_target(&clearance_scan, now) } else { V::None };
        let kw = dict! {
            "rear_launch" => false, "camera_required" => false, "clearance_profile" => "maneuver",
            "relaxed_turn_left" => relaxed_turn_left, "body_contact_target" => contact_target,
            "front_lane" => front_lane, "front_side_walls" => front_side_walls,
            "rear_start" => Supervisor::turn_rear_start(s),
        };
        let demo = V::Bool(self.demo);
        let centering = get_or(&s.report, "centering", &V::Bool(false)).clone();
        let mut clearance = match s.clearance_reuse.probe(&clearance_scan, &ages, &demo, &centering, &kw) {
            Ok(Probe::Reused) => s.clearance_reuse.kept.clone(),
            Ok(Probe::Fresh { clearance, .. }) => clearance,
            Err(clearance) => {
                // ProbeClearanceError: the halt keeps this scan's diagnostics.
                s.report.set("clearance_current", clearance);
                return halt("probe_obstacle_close_body");
            }
        };
        s.report.set("clearance_reused_count", s.clearance_reuse.reused);
        if clearance.idx("body_proximity").idx("target_contact_tolerated").truthy() {
            incr_or_zero(&mut s.report, "target_contact_tolerated_count");
        }
        let issues = issues_set(clearance.idx("quality_issues"));
        let tolerable: Vec<&str> = ["scan_incomplete", "front_sparse"]
            .into_iter()
            .filter(|issue| {
                issues.iter().any(|i| i == issue) && clearance.get(&format!("{issue}_tolerable")).is_true()
            })
            .collect();
        let scope = if !tolerable.is_empty() { self.turn_route_quality_scope(s, t) } else { None };
        if let Some(scope) = scope {
            clearance.set("quality_tolerance_scope", scope);
            let kept: Vec<V> = clearance
                .idx("quality_issues")
                .l()
                .iter()
                .filter(|i| !i.os().is_some_and(|s| tolerable.contains(&s)))
                .cloned()
                .collect();
            clearance.set("quality_issues", V::list(kept));
            for issue in &tolerable {
                clearance.set(&format!("{issue}_tolerated"), true);
                incr_or_zero(&mut s.report, &format!("{issue}_tolerated_count"));
            }
            let remaining = issues_set(clearance.idx("quality_issues"));
            let ready = !["camera_unavailable", "lidar_late", "scan_incomplete", "front_sparse"]
                .iter()
                .any(|i| remaining.iter().any(|r| r == i));
            clearance.set("motion_ready", ready);
        }
        s.report.set("clearance_current", clearance.clone());
        s.report.set("relaxed_turn_left_clearance_active", relaxed_turn_left);
        if !self.turn_healthy() {
            value_error("probe_sensor_unavailable")
        }
        let quality_issues = clearance.idx("quality_issues").clone();
        s.report.set("quality_issues", quality_issues.clone());
        let mut counts = if s.report.has("quality_counts") { s.report.get("quality_counts").clone() } else { V::dict() };
        for issue in quality_issues.l() {
            let key = issue.s();
            let n = if counts.has(key) { counts.get(key).i() } else { 0 };
            counts.set(key, n + 1);
        }
        s.report.set("quality_counts", counts);
        let confirmed = self.perception_quality.update(&quality_issues, t);
        s.report.set("quality_elapsed_ms", self.perception_quality.elapsed_ms(t));
        s.report.set("quality_confirmed", confirmed);
        let motion_ready = s.recovery.update(clearance.idx("motion_ready").truthy(), &quality_issues, self.scan.idx("seq"), t);
        s.report.set("perception_recovering", s.recovery.waiting);
        s.report.set("recovery_stable_ms", s.recovery.stable_ms(t));
        self.turn_tick(s, now, motion_ready)
    }
}

impl Supervisor {
    /// The four bounded neutral entries share these guards (turn_tick).
    fn wait_entry_guard(&self, s: &Session, control: &V) -> bool {
        let empty = V::list(Vec::new());
        let issues = get_or(&s.report, "quality_issues", &empty);
        s.phase == s.motion.phase
            && s.motion.phase == "drive"
            && seq_of(control) >= self.arm_sequence as f64
            && issues.truthy()
            && issues_within(issues, &["scan_incomplete", "front_sparse"])
            && self.turn_quality_hold_eligible(s, true)
    }

    fn wait_trigger(&self, s: &Session, control: &V) -> V {
        dict! {
            "scan_seq" => self.scan.idx("seq").clone(), "source_at_ms" => self.scan.idx("at_ms").clone(),
            "quality_issues" => V::list(s.report.idx("quality_issues").l().to_vec()),
            "adopted_servo" => control.idx("servo").clone(), "control_tick" => control.idx("tick").clone(),
            "control_seq" => control.idx("seq").clone(), "motor_command" => NEUTRAL,
            "existing_arm_sequence" => self.arm_sequence,
        }
    }

    fn enter_quality_wait(s: &mut Session, kind: &'static str, entry_neutral: bool) {
        s.phase = "quality_wait".to_string();
        s.report.set("phase", "quality_wait");
        s.quality_wait_neutral_sequence = None;
        s.quality_resume_sequence = None;
        s.quality_wait_kind = Some(kind);
        s.quality_wait_entry_neutral = Some(entry_neutral);
        let front_sparse = issues_set(s.report.idx("quality_issues")).iter().any(|i| i == "front_sparse");
        s.quality_wait_front_sparse_entry = Some(front_sparse);
    }

    fn front_unknown_of(report: &V) -> V {
        let clearance = report.get("clearance_current");
        let unknown = if clearance.is_dict() && clearance.has("front_unknown_bins") {
            clearance.get("front_unknown_bins").clone()
        } else {
            V::list(Vec::new())
        };
        V::list(unknown.l().to_vec())
    }

    /// turn_tick(session, now, motion_ready).
    fn turn_tick(&mut self, s: &mut Session, now: &V, motion_ready: bool) -> Flow {
        let t = now.f();
        let control = self.control();
        // Round y2: a wait at the traffic light (the car standing at neutral)
        // moves the total deadline on as it moves the drive clock.
        let credit = {
            let turn = s.report.get("turn");
            let c = if turn.truthy() { turn.get("light_wait_credit_s").clone() } else { V::None };
            if c.truthy() { c.f() } else { 0. }
        };
        // Route session (continue_route): waiting never pauses the cumulative
        // drive clock; a deadline wins over same-tick recovery.
        if s.phase != "coast" {
            if t >= s.deadline + credit {
                return halt("turn_total_deadline");
            }
            if let Some(since) = s.motion.drive_since
                && t - since + 1e-9 >= s.motion.max_drive_s
            {
                return halt("maneuver_cumulative_drive_timeout");
            }
        }
        let neutral_wait = !motion_ready
            && s.phase == s.motion.phase
            && s.motion.phase == "presteer"
            && control.get("armed").is_true()
            && seq_of(&control) >= self.arm_sequence as f64
            && control.get("motor").is_int()
            && control.get("motor").i() == NEUTRAL
            && py_eq(s.report.get("current_pwm"), &V::Int(NEUTRAL))
            && issues_within(get_or(&s.report, "quality_issues", &V::list(Vec::new())), &["front_sparse"])
            && s.recovery.waiting;
        if !motion_ready && !neutral_wait {
            let pending_servo = s.turn_servo_sequence;
            let acked = pending_servo.is_none_or(|p| seq_of(&control) >= p as f64);
            let turn_control = control.merged(&dict! {"command_acked" => acked});
            if s.phase == s.motion.phase && s.motion.phase == "quality_wait" {
                if !self.turn_quality_wait_eligible(s) {
                    return halt("turn_perception_unavailable");
                }
            } else if self.wait_entry_guard(s, &control)
                && s.motion.begin_quality_wait(now, &turn_control, Some(&self.received_scan()))
            {
                // Trial44: bounded front_sparse may also use the single neutral wait.
                Supervisor::enter_quality_wait(s, "orbit_wait", false);
                let trigger = self.wait_trigger(s, &control);
                s.report.set("quality_wait_trigger", trigger);
                let snapshot = self.turn_quality_snapshot(s, &self.turn_route_control(s, &turn_control), now);
                s.report.set("quality_wait_snapshot", snapshot);
            } else if self.wait_entry_guard(s, &control)
                && s.motion.begin_left_turn_quality_pause(now, &turn_control, Some(&self.received_scan()))
            {
                // Trial44: one bounded neutral pause before target handover.
                Supervisor::enter_quality_wait(s, "left_turn_pause", false);
                let mut trigger = self.wait_trigger(s, &control);
                trigger.set("front_unknown_bins", Supervisor::front_unknown_of(&s.report));
                s.report.set("left_turn_pause_trigger", trigger);
                let snapshot = self.turn_quality_snapshot(s, &self.turn_route_control(s, &turn_control), now);
                s.report.set("left_turn_pause_snapshot", snapshot);
            } else if self.wait_entry_guard(s, &control)
                && s.motion.begin_post_pass_quality_pause(now, &turn_control, Some(&self.received_scan()))
            {
                // Self-recovery after the first pass: pause at neutral holding
                // the ACKed command, re-check, resume.
                let light_entry_stop = s.motion.route_stage == "light_entry"
                    && (if s.report.get("turn").truthy() { s.report.get("turn").get("light_stop_active").is_true() } else { false });
                let entry_neutral = (s.motion.route_stage == "light_lane" || light_entry_stop)
                    && py_eq(control.get("motor"), &V::Int(NEUTRAL));
                Supervisor::enter_quality_wait(s, "post_pass_pause", entry_neutral);
                let trigger = dict! {
                    "scan_seq" => self.scan.idx("seq").clone(), "source_at_ms" => self.scan.idx("at_ms").clone(),
                    "quality_issues" => V::list(s.report.idx("quality_issues").l().to_vec()),
                    "adopted_servo" => control.idx("servo").clone(), "route_stage" => s.motion.route_stage,
                    "control_seq" => control.idx("seq").clone(),
                };
                s.report.set("post_pass_pause_trigger", trigger);
            } else if self.wait_entry_guard(s, &control) && s.motion.begin_route_quality_coast(now, &turn_control) {
                // Trial41/42: cut motor, hold the adopted left; terminal coast.
                s.route_quality_coast = Some(true);
                self.begin_coast(s, "turn_perception_unavailable", now);
                let mut trigger = dict! {
                    "scan_seq" => self.scan.idx("seq").clone(), "source_at_ms" => self.scan.idx("at_ms").clone(),
                    "quality_issues" => V::list(s.report.idx("quality_issues").l().to_vec()),
                    "adopted_servo" => control.idx("servo").clone(), "control_tick" => control.idx("tick").clone(),
                    "control_seq" => control.idx("seq").clone(), "motor_command" => NEUTRAL,
                    "hold_max_s" => COAST_MAX_S, "positive_motor_restore_allowed" => false,
                };
                trigger.update(&dict! {
                    "route_wait_refusal" => s.motion.quality_wait_refusal.clone(),
                    "handover_observed" => s.motion.handover_observed,
                    "front_unknown_bins" => Supervisor::front_unknown_of(&s.report),
                });
                s.report.set("quality_coast_trigger", trigger);
            } else if s.phase == s.motion.phase
                && s.motion.phase == "drive"
                && POST_PASS_STAGES.contains(&s.motion.route_stage)
                && s.motion.first_pass_evidence.is_some()
                && !turn_control.get("command_acked").is_true()
                && s.turn_servo_sent_at.is_num()
                && (0. ..QUALITY_PAUSE_ACK_GRACE_S).contains(&(t - s.turn_servo_sent_at.f()))
                && get_or(&s.report, "quality_issues", &V::list(Vec::new())).truthy()
                && issues_within(get_or(&s.report, "quality_issues", &V::list(Vec::new())), &["scan_incomplete", "front_sparse"])
                && self.turn_quality_hold_eligible(s, true)
            {
                // Round 76: wait for the in-flight steering ACK (no new command,
                // no halt) up to the grace, then pause or halt.
                let wait = dict! {
                    "scan_seq" => self.scan.idx("seq").clone(), "pending_sequence" => opt_i(s.turn_servo_sequence),
                    "pending_age_s" => t - s.turn_servo_sent_at.f(),
                    "quality_issues" => V::list(s.report.idx("quality_issues").l().to_vec()),
                };
                s.report.set("quality_pause_ack_wait", wait);
                return Flow::Continue;
            } else if !self.turn_quality_coast_active(s) {
                return halt("turn_perception_unavailable");
            }
        }
        if s.phase == "coast" {
            return self.coast_tick(s, now, motion_ready);
        }
        if s.phase == "quality_wait" && !self.turn_quality_wait_eligible(s) {
            return halt("turn_quality_wait_feedback_changed");
        }
        if t >= s.deadline + credit {
            return halt("turn_total_deadline");
        }
        if !control.get("armed").truthy() || seq_of(&control) < self.arm_sequence as f64 {
            if s.phase != "presteer" || s.report.get("observed_armed").truthy() || s.motion.handover_observed {
                return halt("turn_bridge_locked_or_arm_feedback_lost");
            }
            return Flow::Continue;
        }
        s.report.set("observed_armed", true);
        let observed = s.report.get("observed_pwm").truthy() || py_eq(control.get("motor"), &V::Int(TRIAL_MOTOR));
        s.report.set("observed_pwm", observed);
        let pending_servo = s.turn_servo_sequence;
        let acked = pending_servo.is_none_or(|p| seq_of(&control) >= p as f64);
        let turn_control = control.merged(&dict! {"command_acked" => acked});
        let scan = self.received_scan();
        let turn_control = self.turn_route_control(s, &turn_control);
        if s.quality_resume_sequence.is_some() {
            s.report.set("quality_resume_ack", turn_control.idx("resume_acked").clone());
        }
        let clearance = { let c = s.report.get("clearance_current"); if c.truthy() { c.clone() } else { V::dict() } };
        let quality_clear = clearance.get("motion_ready").is_true()
            && !s.report.get("quality_issues").truthy()
            && py_eq(clearance.get("scan_seq"), self.scan.idx("seq"));
        let lidar_age = s.report.idx("sensor_ages").idx("lidar").clone();
        let decision = s.motion.update(&scan, &lidar_age, now, &turn_control, &V::Bool(true), false,
                                       &V::Bool(neutral_wait), &V::Bool(quality_clear), &V::Bool(motion_ready));
        Supervisor::report_decision(s, &decision);
        if decision.idx("lock_requested").truthy() {
            return halt(decision.idx("reason").s());
        }
        if neutral_wait
            && (!py_eq(decision.idx("phase"), &V::str("presteer"))
                || !py_eq(decision.idx("motor"), &V::Int(NEUTRAL))
                || !py_eq(decision.idx("servo"), s.report.idx("servo")))
        {
            value_error("invalid_turn_presteer_wait_output")
        }
        if decision.idx("stop_requested").truthy() {
            self.begin_coast(s, decision.idx("reason").s(), now);
            return self.coast_tick(s, now, true);
        }
        let phase = decision.idx("phase").s().to_string();
        if s.phase == "drive" && phase == "quality_wait" {
            // Motion-initiated self-recovery: the same wait gates from this tick.
            s.quality_wait_neutral_sequence = None;
            s.quality_resume_sequence = None;
            s.quality_wait_kind =
                Some(if s.motion.neutral_pause_kind == Some("recovery") { "recovery_hold" } else { "orbit_wait" });
            s.quality_wait_entry_neutral = Some(
                (s.motion.route_stage == "light_lane"
                    || (s.motion.route_stage == "light_entry" && decision.get("light_stop_active").is_true()))
                    && py_eq(control.get("motor"), &V::Int(NEUTRAL)),
            );
            s.quality_wait_front_sparse_entry = Some(false);
            s.report.set("recovery_hold_trigger", dict! {
                "scan_seq" => self.scan.idx("seq").clone(), "reason" => decision.get("reason").clone(),
                "route_stage" => decision.get("route_stage").clone(), "held_servo" => decision.idx("servo").clone(),
                "recovery_count" => decision.get("recovery_count").clone(),
            });
        }
        s.phase = phase.clone();
        s.report.set("phase", phase.as_str());
        let (motor, servo) = (decision.idx("motor").clone(), decision.idx("servo").clone());
        let m = &s.motion;
        let route_center = decision.get("route_center_ack");
        let route_stage = decision.get("route_stage");
        let stage = route_stage.os().unwrap_or("");
        let stage_is = |name: &str| route_stage.os() == Some(name);
        let route_right = phase == "drive"
            && m.phase == "drive"
            && route_stage.os() == Some(m.route_stage)
            && decision.get("route_right_output_authorized").is_true()
            && decision.get("quality_resume_pending").is_false()
            && control.get("servo").is_int()
            && control.get("servo").i() <= NEUTRAL
            && route_center.is_dict()
            && py_eq(route_center.get("servo"), &V::Int(NEUTRAL))
            // Round 82: the coasting light lane may prove centre at neutral.
            && (in_motors(route_center.get("motor"), &DRIVE_MOTORS)
                || (stage_is("light_lane") && py_eq(route_center.get("motor"), &V::Int(NEUTRAL)))
                // Round y2: the light stop coasts from the light entry on.
                || (stage_is("light_entry")
                    && decision.get("light_stop_active").is_true()
                    && py_eq(route_center.get("motor"), &V::Int(NEUTRAL))))
            && route_center.get("seq").is_int()
            && self.arm_sequence as f64 <= route_center.get("seq").f()
            && route_center.get("seq").f() <= seq_of(&control)
            && route_center.get("tick").is_int()
            && 0. <= route_center.get("tick").f()
            && route_center.get("tick").f() <= tick_of(&control)
            && ((m.first_pass_evidence.is_some()
                && (((stage_is("right_align") || stage_is("corridor_follow")) && m.right_exit_center_ack.is_some())
                    || (stage_is("second_orbit") && m.second_center_ack.is_some())
                    || stage_is("light_turn")
                    || stage_is("light_lane")
                    // Round 64: open-course stages after the first loop.
                    || (OPEN_STAGES.contains(&stage) && route_stage.is_str() && m.entry_kind == Some("lane"))))
                // Round 64: start-lane wall corrections (start-zone entry only).
                || (stage_is("start_lane")
                    && m.entry_kind == Some("lane")
                    && m.first_pass_evidence.is_none()
                    && !m.handover_observed));
        let right_exit = route_right;
        let wait_kind_right = matches!(s.quality_wait_kind, Some("post_pass_pause" | "recovery_hold"));
        let right_hold = (phase == "quality_wait" && wait_kind_right)
            || (decision.get("quality_resume_pending").is_true()
                && wait_kind_right
                && py_eq(&servo, &opt_i(m.quality_wait_servo)));
        let minimum_servo = if right_exit || right_hold { right_exit_min_pwm() } else { NEUTRAL };
        // After the first pass the route drives at the manual forward PWM.
        let slow_allowed = phase == "drive"
            && ((POST_PASS_STAGES.contains(&stage) && route_stage.is_str() && m.first_pass_evidence.is_some())
                || (stage_is("first_target") && m.handover_observed)
                || (stage_is("start_lane") && m.entry_kind == Some("lane")))
            && decision.get("quality_resume_pending").is_false();
        // Round 100: the start-zone lane launch may use 1600, before any
        // handover or pass and outside a resume only.
        let launch_allowed = phase == "drive"
            && stage_is("start_lane")
            && m.entry_kind == Some("lane")
            && m.first_pass_evidence.is_none()
            && !m.handover_observed
            && decision.get("quality_resume_pending").is_false();
        let mut motors: Vec<i64> = if slow_allowed { vec![NEUTRAL, TRIAL_MOTOR, ROUTE_SLOW_MOTOR] } else { vec![NEUTRAL, TRIAL_MOTOR] };
        if launch_allowed {
            motors.push(START_LAUNCH_MOTOR);
        }
        if !in_motors(&motor, &motors) || !(minimum_servo as f64 <= num(&servo) && num(&servo) <= SERVO_MAX as f64) {
            value_error("invalid_turn_trial_output")
        }
        if phase == "quality_wait"
            && (!py_eq(&motor, &V::Int(NEUTRAL)) || !py_eq(&servo, &opt_i(m.quality_wait_servo)))
        {
            value_error("invalid_turn_quality_wait_output")
        }
        if decision.get("quality_resume_pending").is_true()
            && (!py_eq(&motor, &V::Int(TRIAL_MOTOR)) || !py_eq(&servo, &opt_i(m.quality_wait_servo)))
        {
            value_error("invalid_turn_quality_resume_output")
        }
        let resume_key = match s.quality_wait_kind {
            Some("left_turn_pause") => "left_turn_pause_resume_snapshot",
            Some("post_pass_pause") => "post_pass_pause_resume_snapshot",
            Some("recovery_hold") => "recovery_hold_resume_snapshot",
            _ => "quality_resume_snapshot",
        };
        if turn_control.idx("resume_acked").truthy()
            && s.quality_resume_sequence.is_some()
            && !s.report.has(resume_key)
            && !decision.get("lock_requested").truthy()
        {
            let snapshot = self.turn_quality_snapshot(s, &turn_control, now);
            s.report.set(resume_key, snapshot);
        }
        let changed = !py_eq(&servo, &s.turn_last_servo);
        if changed {
            incr(&mut s.report, "steering_changes");
        }
        s.report.set("servo", servo.clone());
        s.report.set("current_pwm", motor.clone());
        let tick = control.idx("tick").clone();
        self.emit(Some(s), "drive", motor.clone(), servo.clone(), tick);
        if phase == "quality_wait" {
            if s.quality_wait_neutral_sequence.is_none() {
                s.quality_wait_neutral_sequence = Some(self.sequence);
            }
            s.report.set("quality_wait_neutral_ack", turn_control.idx("neutral_acked").clone());
        }
        if decision.get("quality_resume_pending").is_true() {
            if s.quality_resume_sequence.is_none() {
                s.quality_resume_sequence = Some(self.sequence);
            }
            s.report.set("quality_resume_ack", turn_control.idx("resume_acked").clone());
        }
        if changed {
            s.turn_last_servo = servo;
            s.turn_servo_sequence = Some(self.sequence);
            s.turn_servo_sent_at = now.clone();
        }
        if neutral_wait {
            incr(&mut s.report, "recovery_ticks");
        }
        if phase == "presteer" {
            incr(&mut s.report, "presteer_ticks");
        } else if num(&motor) > NEUTRAL as f64 {
            incr(&mut s.report, "motion_ticks");
        }
        if num(&motor) > NEUTRAL as f64 {
            incr(&mut s.report, "drive_ticks");
        }
        Flow::Continue
    }
}

// ---------------------------------------------------------------- start

impl Supervisor {
    /// turn_preview(now, max_drive_s, initial_presteer_pwm, compact, continue_route):
    /// (motion, decision, clearance), or the refusal with its clearance.
    pub fn turn_preview(&mut self, now: &V, max_drive_s: &V, initial: &V, continue_route: bool)
                        -> Result<(Maneuver, V, V), (PyErr, V)> {
        let ages = self.sensor_ages(now.f());
        // The route starts forward from the start zone (rear returns do not
        // block it) and this frame's side boards explain grazing rays.
        let kw = dict! {
            "rear_launch" => false, "camera_required" => false, "clearance_profile" => "maneuver",
            "rear_start" => continue_route, "front_side_walls" => continue_route,
        };
        let clearance = match py_try(|| probe_clearance(&self.scan, &ages, &V::Bool(self.demo), &V::Bool(true), &kw)) {
            Ok(Ok(clearance)) => clearance,
            Ok(Err(clearance)) => {
                return Err((PyErr { kind: Exc::ValueError, msg: "probe_obstacle_close_body".into() }, clearance));
            }
            Err(error) => return Err((error, V::None)),
        };
        let checked = py_try(|| {
            if !self.turn_healthy() || !clearance.idx("motion_ready").truthy() {
                value_error("turn_sensor_unavailable")
            }
            let control = self.control();
            if self.control_mode != "locked"
                || !control.get("armed").is_false()
                || !py_eq(control.get("motor"), &V::Int(NEUTRAL))
                || !py_eq(control.get("servo"), &V::Int(NEUTRAL))
            {
                value_error("turn_requires_fresh_neutral_lock")
            }
            let initial = if initial.is_none() { V::Int(DEFAULT_INITIAL_PWM) } else { initial.clone() };
            let mut motion = Maneuver::new(now, max_drive_s, &initial, &V::Bool(continue_route));
            let scan = self.received_scan();
            let f = V::Bool(false);
            let decision = motion.update(&scan, ages.idx("lidar"), now, &control, &V::Bool(true), false, &f, &f, &f);
            let decision = decision.merged(&dict! {
                "trial_scope" => COMPACT_TARGET_TRIAL_SCOPE, "semantic_class" => "unknown",
                "competition_supported" => false, "completed" => false,
            });
            if !decision.idx("start_ready").truthy() || decision.idx("lock_requested").truthy() {
                value_error(decision.idx("reason").s().to_string())
            }
            (motion, decision)
        });
        match checked {
            Ok((motion, decision)) => Ok((motion, decision, clearance)),
            Err(error) if error.kind == Exc::ValueError => Err((error, clearance)),
            Err(error) => raise(error.kind, error.msg),
        }
    }

    /// autonomy_command({'op': 'turn_cone_start', ...}) for a route run:
    /// {'ok', 'run_id', 'epoch'}, or the refusal (with a clearance when the
    /// console attaches one).
    pub fn start_turn_cone(&mut self, data: &V, now: &V, run_id: &str) -> Result<V, (PyErr, V)> {
        match py_try(|| self.start_checked(data, now, run_id)) {
            Ok(result) => result,
            Err(error) => Err((error, V::None)),
        }
    }

    fn start_checked(&mut self, data: &V, now: &V, run_id: &str) -> Result<V, (PyErr, V)> {
        if !data.get("boot").os().is_some_and(|b| b == self.boot)
            || !data.get("epoch").is_int()
            || data.get("epoch").i() != self.control_epoch
        {
            value_error("stale_autonomy_generation")
        }
        if data.get("op").os() != Some("turn_cone_start") {
            value_error("route_supervisor_supports_turn_cone_start_only")
        }
        let allowed = ["op", "boot", "epoch", "tick", "placement_confirmed", "max_drive_s", "initial_presteer_pwm",
                       "continue_route"];
        let continue_route = get_or(data, "continue_route", &V::Bool(false)).clone();
        if !continue_route.is_bool() {
            value_error("invalid_continue_route")
        }
        if data.items().iter().any(|(k, _)| !allowed.contains(&&**k)) || !data.get("placement_confirmed").is_true() {
            value_error("turn_trial_requires_operator_placement_confirmation")
        }
        if self.owner.is_some() || self.session.is_some() {
            value_error("already_armed_or_shutting_down")
        }
        if !continue_route.is_true() {
            // The program runs the race path only; other trials stay in the console.
            value_error("route_supervisor_requires_continue_route")
        }
        let initial = get_or(data, "initial_presteer_pwm", &V::Int(DEFAULT_INITIAL_PWM)).clone();
        validate_maneuver_initial_pwm(&initial);
        let limit = get_or(data, "max_drive_s", &V::Float(MAX_DRIVE_S)).clone();
        let (motion, decision, clearance) = self.turn_preview(now, &limit, &initial, true)?;
        let tick = data.get("tick");
        let control = self.status.idx("control").clone();
        let current_tick = get_or(&control, "tick", &V::Int(-1)).clone();
        if !tick.is_int() || !(0. ..100.).contains(&(num(&current_tick) - tick.f())) || tick.f() <= num(&self.stopped_tick) {
            value_error("stale_arm_request")
        }
        let t = now.f();
        let mut report = dict! {
            "run_id" => run_id, "epoch" => self.control_epoch, "pwm" => TRIAL_MOTOR, "servo" => NEUTRAL,
            "duration_ms" => round(1000. * (motion.max_presteer_s + motion.max_drive_s)), "phase" => "presteer",
            "turn_presteer_max_s" => motion.max_presteer_s, "turn_drive_max_s" => motion.max_drive_s,
            "control_loop_limit_s" => MANEUVER_CONTROL_LOOP_LIMIT_S,
            "actual_gap_s" => 0., "max_actual_gap_s" => 0.,
            "turn_stage" => decision.idx("turn_stage").clone(),
            "drive_ticks" => 0, "motion_ticks" => 0, "presteer_ticks" => 0, "coast_ticks" => 0, "recovery_ticks" => 0,
            "observed_pwm" => false, "observed_armed" => false, "steering_changes" => 0, "centering" => true,
            "turn_trial" => true, "formal_stop_goal" => false, "endpoint" => "left_turn_trial",
            "operator_placement_confirmed" => true, "start_scope" => "operator_placed_mid_segment",
            "sensor_inputs" => V::list(vec![V::str("lidar"), V::str("control")]), "camera_required" => false,
            "trial_goal" => V::None,
            "competition_navigation" => false, "wheel_motion_measured" => false, "current_pwm" => NEUTRAL,
            "physical_steering_confirmed" => false, "turn_path_certified" => false, "entry_confirmed" => false,
            "observed_alignment" => false, "turn" => decision.clone(), "walls" => V::None, "coast_motion" => V::None,
            "standstill_confirmed" => false, "quality_issues" => V::list(Vec::new()), "quality_counts" => V::dict(),
            "clearance_profile" => "maneuver", "clearance_profile_source" => "bounded_left_turn_trial",
            "clearance_start" => clearance.clone(), "clearance_current" => clearance.clone(),
            "rear_launch_active" => false, "rear_launch_max_s" => 0,
        };
        report.update(&dict! {
            "endpoint" => "first_compact_target_orbit_entry_trial", "continue_route" => true,
            "route_controller" => self.route_controller.as_str(), "left_turn_servo_cap" => SERVO_MAX,
            "trial_scope" => COMPACT_TARGET_TRIAL_SCOPE, "semantic_class" => "unknown", "competition_supported" => false,
            "completed" => false, "compact_target" => decision.get("compact_target").clone(),
            "handover_observed" => decision.get("handover_observed").truthy(),
            "orbit_entry_elapsed_s" => get_or(&decision, "orbit_entry_elapsed_s", &V::Float(0.)).clone(),
            "clearance_profile_source" => "bounded_first_compact_target_orbit_entry_trial",
        });
        let deadline = t + motion.max_presteer_s + motion.max_drive_s;
        self.session = Some(Session {
            report,
            motion,
            phase: "presteer".to_string(),
            deadline,
            last_loop: t,
            heartbeat_seq: 0,
            recovery: QualityRecovery::default(),
            motion_scan_seq: V::None,
            turn_last_servo: V::Int(NEUTRAL),
            turn_servo_sequence: None,
            turn_servo_sent_at: V::None,
            steer_output: SteerOutput::default(),
            clearance_reuse: ClearanceReuse::default(),
            coast_since: None,
            coast_deadline: None,
            coast_reason: None,
            motion_generation: None,
            motion_ready_for_coast: None,
            coast_start_seq: V::None,
            coast_neutral_sequence: None,
            coast_endpoint_verified: None,
            quality_wait_neutral_sequence: None,
            quality_resume_sequence: None,
            quality_wait_kind: None,
            quality_wait_entry_neutral: None,
            quality_wait_front_sparse_entry: None,
            route_quality_coast: None,
        });
        self.auto_result = V::None;
        self.owner = Some(format!("auto-turn-{run_id}"));
        self.owner_at = t;
        self.control_mode = "auto_turn_cone_trial";
        self.stop_latched = false;
        if let Err(error) = py_try(|| self.emit(None, "arm", V::Int(NEUTRAL), V::Int(NEUTRAL), tick.clone())) {
            if error.kind == Exc::RuntimeError {
                self.halt("turn_arm_failed");
            }
            raise(error.kind, error.msg)
        }
        self.arm_sequence = self.sequence;
        Ok(dict! {"ok" => true, "run_id" => run_id, "epoch" => self.control_epoch})
    }
}

// ---------------------------------------------------------------- the program's console duties

const ACTIVE_SUMMARY_FIELDS: [&str; 7] = [
    "route_events", "quality_wait_snapshot", "quality_resume_snapshot", "left_turn_pause_snapshot",
    "left_turn_pause_resume_snapshot", "post_pass_pause_resume_snapshot", "recovery_hold_resume_snapshot",
];

/// _evidence_summary(value).
fn evidence_summary(value: &V) -> V {
    if !value.is_dict() {
        return value.clone();
    }
    let seq = |item: &V| {
        let scan = if item.is_dict() { item.get("scan") } else { &V::None };
        if scan.is_dict() { scan.get("seq").clone() } else { V::None }
    };
    if value.items().iter().all(|(_, item)| item.is_dict() && item.has("scan")) {
        let mut events = V::dict();
        for (name, item) in value.items() {
            events.set(name, dict! {"scan_seq" => seq(item)});
        }
        return dict! {"summarized" => true, "events" => events};
    }
    let mut keys: Vec<String> = value.items().iter().map(|(k, _)| k.to_string()).collect();
    keys.sort();
    dict! {"summarized" => true, "keys" => V::list(keys.into_iter().map(V::from).collect()), "scan_seq" => seq(value)}
}

/// active_report_view(report): a live report with bulky evidence summarized.
pub fn active_report_view(report: &V) -> V {
    let mut view = report.clone();
    for key in ACTIVE_SUMMARY_FIELDS {
        if view.get(key).is_dict() {
            let summary = evidence_summary(view.get(key));
            view.set(key, summary);
        }
    }
    let turn = view.get("turn");
    if turn.is_dict() {
        let mut turn = turn.clone();
        if turn.get("route_events").is_dict() {
            let summary = evidence_summary(turn.get("route_events"));
            turn.set("route_events", summary);
        }
        let avoid = turn.get("avoid_evidence");
        if avoid.is_list() && avoid.len() > 2 {
            let list = avoid.l();
            let summary = dict! {"summarized" => true, "count" => list.len(), "last" => V::list(list[list.len() - 2..].to_vec())};
            turn.set("avoid_evidence", summary);
        }
        view.set("turn", turn);
    }
    view
}

impl Supervisor {
    /// accept_lidar(value, now): a newer scan replaces the current one.
    pub fn accept_lidar(&mut self, value: V, now: f64) -> bool {
        if !value.is_dict() || !value.get("seq").is_int() || value.get("seq").i() < 0 {
            value_error("invalid_lidar_sequence")
        }
        let published = value.get("at_ms").clone();
        if !self.demo && (!published.is_int() || published.i() < 0) {
            value_error("invalid_lidar_publication_clock")
        }
        if self.scan.is_some() {
            let (old_seq, old_publication) = (self.scan.get("seq"), self.scan.get("at_ms"));
            if old_seq.is_int() && value.get("seq").i() <= old_seq.i() {
                return false;
            }
            if old_publication.is_int() && (!published.is_int() || published.i() <= old_publication.i()) {
                return false;
            }
        }
        self.scan = value;
        self.scan_at = V::Float(now);
        true
    }

    /// read_bridge's control branch: feedback in nominal PWM; a lock after our arm halts.
    pub fn accept_control(&mut self, value: &V, now: f64) {
        self.status = self.nominal_status(value);
        self.control_at = now;
        let control = self.control();
        if self.owner.is_some() && !control.get("armed").truthy() && seq_of(&control) >= self.arm_sequence as f64 {
            let reason = if control.has("reason") { control.get("reason").s().to_string() } else { "locked".into() };
            self.halt(&format!("bridge_{reason}"));
        }
    }

    /// Round y2 (race program): the read-only vision process's newest light
    /// frame rides on the control status as control['light'].
    pub fn set_light(&mut self, light: &V) {
        if self.status.get("control").is_dict() {
            let mut control = self.status.get("control").clone();
            control.set("light", light.clone());
            self.status.set("control", control);
        }
    }

    /// A bridge ended or sent an invalid line (read_bridge's exception path).
    pub fn bridge_failed(&mut self, kind: &str, error: &str) {
        self.errors.set(kind, error);
        self.halt("bridge_failure");
    }

    /// health_watch, one pass (every 50 ms). The program has no camera: its
    /// idle health is lidar and control freshness without errors.
    pub fn health_watch(&mut self, now: f64) {
        self.mono = now;
        let healthy = if self.session.is_some() {
            self.turn_healthy()
        } else {
            now - self.scan_at_f() < 1. && now - self.control_at < 0.2 && self.errors.items().is_empty()
        };
        let unsafe_state = !healthy || (self.owner.is_some() && now - self.owner_at > 0.35);
        let armed = self.control().get("armed").truthy();
        if unsafe_state && (armed || self.owner.is_some()) {
            self.halt(if !healthy { "sensor_stale" } else { "browser_timeout" });
        }
        if self.session.is_none() {
            // The idle latch as the console keeps it (no camera here).
            let ages = self.sensor_ages(now);
            let kw = dict! {"camera_required" => false};
            if let Ok(Ok(clearance)) = except(&[Exc::ValueError], || {
                probe_clearance(&self.scan, &ages, &V::Bool(self.demo), &V::Bool(false), &kw)
            }) {
                self.perception_quality.update(clearance.idx("quality_issues"), now);
            }
        }
    }

    /// record_tick_timing(session, lock_wait, tick_wall, cpu=None, breakdown).
    pub fn record_tick_timing(&mut self, lock_wait_s: f64, tick_wall_s: f64, tick_cpu_s: Option<f64>,
                              breakdown: Option<V>, run_id: &V) {
        let report = match &self.session {
            Some(s) => s.report.clone(),
            None if py_eq(self.auto_result.get("run_id"), run_id) => {
                self.auto_result.clone()
            }
            None => return,
        };
        let mut timing = dict! {
            "last_lock_wait_s" => pmax(0., lock_wait_s), "last_tick_compute_s" => pmax(0., tick_wall_s),
            "max_lock_wait_s" => pmax(get_or(&report, "max_lock_wait_s", &V::Float(0.)).f(), lock_wait_s),
            "max_tick_compute_s" => pmax(get_or(&report, "max_tick_compute_s", &V::Float(0.)).f(), tick_wall_s),
            "control_tick_timing_basis" => "host_monotonic_wall_including_scheduling_and_waits",
        };
        if let Some(cpu) = tick_cpu_s {
            timing.update(&dict! {
                "last_tick_cpu_s" => cpu, "max_tick_cpu_s" => pmax(get_or(&report, "max_tick_cpu_s", &V::Float(0.)).f(), cpu),
                "tick_cpu_basis" => "control_thread_schedstat_run_time",
            });
        }
        if let Some(b) = breakdown {
            timing.set("max_wake_late_s", pmax(get_or(&report, "max_wake_late_s", &V::Float(0.)).f(), b.idx("wake_late_s").f()));
            let previous = report.get("max_gap_breakdown");
            let previous_gap = if previous.is_dict() && previous.has("gap_s") { previous.get("gap_s").f() } else { -1. };
            if b.idx("gap_s").f() > previous_gap {
                timing.set("max_gap_breakdown", b);
            }
        }
        if let Some(s) = self.session.as_mut() {
            s.report.update(&timing);
        }
        if self.auto_result.is_dict() && py_eq(self.auto_result.get("run_id"), run_id) {
            self.auto_result.update(&timing);
        }
    }

    /// The console's /api/state, the parts the analysis tools read.
    pub fn state_view(&mut self, now: f64, max_drive_s: &V, initial: &V) -> V {
        let owned = self.owner.is_some() || self.session.is_some() || self.control().get("armed").truthy();
        let (mut turn_ready, mut turn_rejection, mut turn_preview, mut turn_clearance) =
            (false, V::None, V::None, V::None);
        if owned {
            turn_rejection = V::str("control_owned_or_unlocked");
        } else {
            self.mono = now;
            match py_try(|| self.turn_preview(&V::Float(now), max_drive_s, initial, true)) {
                Ok(Ok((_, decision, clearance))) => {
                    turn_ready = true;
                    turn_preview = decision;
                    turn_clearance = clearance;
                }
                Ok(Err((error, clearance))) => {
                    turn_rejection = V::from(error.msg);
                    turn_clearance = clearance;
                }
                Err(error) => turn_rejection = V::from(format!("{}: {}", error.kind.name(), error.msg)),
            }
        }
        let autonomy = dict! {
            "mode" => self.control_mode, "epoch" => self.control_epoch,
            "control_loop_limit_s" => MANEUVER_CONTROL_LOOP_LIMIT_S,
            "turn_ready" => turn_ready, "turn_rejection" => turn_rejection, "turn_preview" => turn_preview,
            "turn_clearance" => turn_clearance, "route_controller" => self.route_controller.as_str(),
            "trial_mode" => "turn-cone", "continue_route" => true,
            "active" => self.session.as_ref().map(|s| active_report_view(&s.report)).unwrap_or(V::None),
            "last_result" => self.auto_result.clone(),
        };
        dict! {
            "demo" => self.demo, "boot" => self.boot.as_str(), "status" => self.status.clone(),
            "ages" => dict! {"camera" => V::None, "lidar" => now - self.scan_at_f(), "control" => now - self.control_at},
            "errors" => self.errors.clone(), "owner" => opt_s(&self.owner), "last_stop" => self.last_stop.clone(),
            "autonomy" => autonomy,
        }
    }

    /// The owner lease (the console's heartbeat): the program renews it itself.
    pub fn renew_owner(&mut self, now: f64) {
        if self.owner.is_some() {
            self.owner_at = now;
        }
    }
}

fn opt_s(value: &Option<String>) -> V {
    value.as_deref().map(V::str).unwrap_or(V::None)
}
