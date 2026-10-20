//! TurnMotion.update and the junction-entry / first-target methods of
//! ManeuverSequence (left turn, entry bearing, first-pass observation).

use crate::compact_target::{contact_supports, floats, CandidateArgs};
use crate::dict;
use crate::helpers::*;
use crate::maneuver::*;
use crate::py::*;

/// _mature_observation(history, scan, duration).
pub fn mature_observation(history: &mut Vec<Row>, scan: &V, duration: f64) -> bool {
    let row = scan_row(scan);
    if let Some(last) = history.last() {
        if row.0 == last.0 {
            return false;
        }
        if row.0 <= last.0
            || row.1 <= last.1
            || row.2 <= last.2
            || (row.1 - last.1) as f64 / 1000. >= SCAN_AGE_S
            || row.2 - last.2 >= SCAN_AGE_S
            || ((row.1 - last.1) as f64 / 1000. - (row.2 - last.2)).abs() > 0.15
        {
            history.clear();
        }
    }
    history.push(row);
    if history.len() > 6 {
        let n = history.len();
        history.drain(..n - 6);
    }
    history.len() >= 3
        && (row.1 - history[0].1) as f64 / 1000. + 1e-9 >= duration
        && row.2 - history[0].2 + 1e-9 >= duration
}

/// _consecutive_fresh(history, scan, count, max_gap_s).
pub fn consecutive_fresh(history: &mut Vec<Row>, scan: &V, count: usize, max_gap_s: f64) -> bool {
    let row = scan_row(scan);
    if let Some(last) = history.last() {
        if row.0 == last.0 {
            return history.len() >= count;
        }
        if row.0 <= last.0
            || row.1 <= last.1
            || row.2 <= last.2
            || (row.1 - last.1) as f64 / 1000. >= max_gap_s
            || row.2 - last.2 >= max_gap_s
            || ((row.1 - last.1) as f64 / 1000. - (row.2 - last.2)).abs() > 0.15
        {
            history.clear();
        }
    }
    history.push(row);
    if history.len() > 6 {
        let n = history.len();
        history.drain(..n - 6);
    }
    history.len() >= count
}

/// _observation_lease(scan, now, published, received).
pub fn observation_lease(scan: &V, now: f64, published: &V, received: &V) -> bool {
    if !(published.is_num() && received.is_num() && scan.is_dict() && scan.get("at_ms").is_num() && scan.get("received_at").is_num()) {
        return false;
    }
    let dt = (scan.idx("at_ms").f() - published.f()) / 1000.;
    let rt = scan.idx("received_at").f() - received.f();
    (0. ..SCAN_AGE_S).contains(&dt)
        && (0. ..SCAN_AGE_S).contains(&rt)
        && (0. ..SCAN_AGE_S).contains(&(now - received.f()))
        && (dt - rt).abs() <= 0.15
}

/// (sorted-by-`interval`) support projections on an axis.
fn interval(w: &V, theta: f64) -> (f64, f64) {
    let mut v: Vec<f64> = ["support_start_left_m", "support_end_left_m"]
        .iter()
        .map(|k| theta.cos() * w.idx(k).idx("x_m").f() + theta.sin() * w.idx(k).idx("y_m").f())
        .collect();
    sort_floats(&mut v);
    (v[0], v[1])
}

impl Maneuver {
    /// _full_target_points(target, scan, identity).
    pub fn full_target_points(&self, target: &V, scan: &V, identity: &V) -> Option<Vec<(f64, f64)>> {
        if !target.is_dict()
            || !py_eq(target.get("track_id"), identity)
            || !target.get("confirmed").is_true()
            || !target.get("tracking_only_boundary_gap").is_false()
            || !py_eq(target.get("source_seq"), scan.get("seq"))
            || !py_eq(target.get("source_at_ms"), scan.get("at_ms"))
            || !py_eq(target.get("source_received_at"), scan.get("received_at"))
        {
            return None;
        }
        let bins = target.get("support_bins");
        let ranges = scan.get("ranges");
        let Some(bl) = bins.ol() else { return None };
        if (bl.len() as i64) < TOLERANT_MAINT_MIN {
            return None;
        }
        let Some(rl) = ranges.ol() else { return None };
        if rl.len() != 360 {
            return None;
        }
        if bl.iter().any(|i| !i.is_int() || !(0..360).contains(&i.i()) || !rl[i.i() as usize].is_num()) {
            return None;
        }
        let mut seen: Vec<i64> = bl.iter().map(|i| i.i()).collect();
        seen.sort();
        seen.dedup();
        if seen.len() != bl.len() {
            return None;
        }
        let mut points: Vec<(f64, f64)> = bl
            .iter()
            .map(|i| {
                let i = i.i() as usize;
                let r = rl[i].f();
                let a = radians(i as f64);
                (r * a.cos(), -r * a.sin())
            })
            .collect();
        let unknown = target.get("boundary_unknown_bins");
        let empty: [V; 0] = [];
        let unknown: &[V] = if unknown.truthy() { unknown.l() } else { &empty };
        if unknown.iter().any(|i| !i.is_int() || !(0..360).contains(&i.i())) {
            return None;
        }
        let near = bl.iter().map(|i| rl[i.i() as usize].f()).fold(INF, pmin);
        for i in unknown {
            let a = radians(i.i() as f64);
            points.push((near * a.cos(), -near * a.sin()));
        }
        Some(points)
    }

    /// current_contact_target(scan, now).
    pub fn current_contact_target(&self, scan: &V, now: &V) -> V {
        if !self.continue_route
            || !(self.phase == "drive" || self.phase == "quality_wait")
            || self.drive_since.is_none()
            || !scan.is_dict()
            || !now.is_num()
            || !scan.get("received_at").is_num()
            || !(0. ..SCAN_AGE_S).contains(&(now.f() - scan.idx("received_at").f()))
            || !scan.get("seq").is_int()
            || !scan.get("at_ms").is_int()
            || !scan.get("ranges").is_list()
            || scan.idx("ranges").len() != 360
        {
            return V::None;
        }
        let Ok(supports) = except(VTK, || contact_supports(scan.idx("ranges"))) else { return V::None };
        if supports.is_empty() {
            return V::None;
        }
        let mut bins: Vec<i64> = supports.iter().flat_map(|s| ints_of(s.idx("support_bins"))).collect();
        bins.sort();
        bins.dedup();
        let list = scan.idx("ranges").l();
        dict! {
            "role" => "cone_like_support", "track_id" => V::None, "confirmed" => false, "full_observation" => true,
            "source_seq" => scan.idx("seq").clone(), "source_at_ms" => scan.idx("at_ms").clone(),
            "source_received_at" => scan.idx("received_at").clone(), "support_bins" => V::from(bins.clone()),
            "support_ranges_m" => V::list(bins.iter().map(|&i| list[i as usize].clone()).collect()),
            "semantic_class" => "unknown",
            "objects" => V::list(supports.iter().map(|s| dict! {
                "support_bins" => s.idx("support_bins").clone(), "range_m" => s.idx("range_m").clone(),
                "bearing_left_rad" => s.idx("bearing_left_rad").clone(), "diameter_m" => s.idx("diameter_m").clone(),
            }).collect()),
            "orbit_track_id" => self.orbit_track_id.clone(), "second_track_id" => self.second_track_id.clone(),
        }
    }

    /// _target_gap_held(tracker, identity, scan, now).
    pub fn target_gap_held(held: &V, identity: &V, scan: &V, now: f64) -> bool {
        held.is_dict()
            && held.get("confirmed").is_true()
            && py_eq(held.get("track_id"), identity)
            && observation_lease(scan, now, held.get("last_full_isolation_at_ms"), held.get("last_full_isolation_received_at"))
    }

    pub fn clear_target_recovery_history(&mut self) {
        self._quality_good_history.clear();
        self._quality_full_streak = 0;
        self._quality_pass_history.clear();
        self._first_pass_history.clear();
        self._second_pass_history.clear();
    }

    pub fn mature_recovery_observation(&mut self, scan: &V) -> bool {
        let previous = self._quality_good_history.last().map(|r| r.0);
        let mature = mature_observation(&mut self._quality_good_history, scan, 0.30);
        if Some(scan.idx("seq").i()) != previous {
            self._quality_full_streak =
                if self._quality_good_history.len() == 1 { 1 } else { self._quality_full_streak + 1 };
        }
        mature && self._quality_full_streak >= 2
    }

    pub fn route_exit_release_active(&self) -> bool {
        self.continue_route
            && self.handover_observed
            && ((self.phase == "drive" && (self.first_pass_preparing || self.first_pass_evidence.is_some()))
                || (self.phase == "quality_wait" && self.quality_wait_release.is_some()))
    }

    /// _paired_window_corridor(scan): (candidate | None, ambiguous).
    pub fn paired_window_corridor(scan: &V) -> (V, bool) {
        let walls = scan.get("wall_candidates");
        let Some(walls) = walls.ol() else { return (V::None, false) };
        if walls.len() > 64 {
            return (V::None, false);
        }
        let forward: Vec<&V> =
            walls.iter().filter(|w| valid_wall(w) && w.idx("heading_left_rad").f().abs() < PI / 2.).collect();
        let mut pairs: Vec<V> = Vec::new();
        for left in &forward {
            if left.idx("rho_left_m").f() <= 0. {
                continue;
            }
            for right in &forward {
                if right.idx("rho_left_m").f() >= 0. {
                    continue;
                }
                let difference = wrap(left.idx("heading_left_rad").f() - right.idx("heading_left_rad").f());
                if difference.abs() > pair_max_angle_rad() {
                    continue;
                }
                let heading = wrap(right.idx("heading_left_rad").f() + difference / 2.);
                if heading.abs() >= PI / 2. {
                    continue;
                }
                let (c, s) = (heading.cos(), heading.sin());
                let end = |wall: &V| -> (f64, f64, f64) {
                    let points = [wall.idx("support_start_left_m"), wall.idx("support_end_left_m")];
                    let normal: Vec<f64> = points.iter().map(|p| -s * p.idx("x_m").f() + c * p.idx("y_m").f()).collect();
                    let along: Vec<f64> = points.iter().map(|p| c * p.idx("x_m").f() + s * p.idx("y_m").f()).collect();
                    (fsum_py(normal.iter().cloned()) / 2., pmin(along[0], along[1]), pmax(along[0], along[1]))
                };
                let (le, re) = (end(left), end(right));
                let width = le.0 - re.0;
                let shared = pmin(le.2, re.2) - pmax(le.1, re.1);
                if !(le.0 > 0. && 0. > re.0) || !(PAIR_MIN_WIDTH_M <= width && width <= PAIR_MAX_WIDTH_M) || shared < PAIR_MIN_SHARED_SUPPORT_M {
                    continue;
                }
                pairs.push(dict! {
                    "heading_left_rad" => heading, "center_offset_left_m" => (le.0 + re.0) / 2.,
                    "width_m" => width, "left_wall_points" => left.idx("points").clone(),
                    "right_wall_points" => right.idx("points").clone(), "support_span_m" => shared,
                    "fit_error_m" => if right.idx("fit_error_m").f() > left.idx("fit_error_m").f() { right.idx("fit_error_m").clone() } else { left.idx("fit_error_m").clone() },
                    "origin_between_walls" => true, "candidate_only" => true, "turn_path_certified" => false,
                });
            }
        }
        if pairs.is_empty() {
            return (V::None, false);
        }
        let base = pairs[argmax_first(&pairs, |p| p.idx("support_span_m").f()).unwrap()].clone();
        for p in &pairs {
            if wrap(p.idx("heading_left_rad").f() - base.idx("heading_left_rad").f()).abs() > pair_merge_heading_rad()
                || (p.idx("width_m").f() - base.idx("width_m").f()).abs()
                    > pmax(0.15, MAX_EXIT_WIDTH_CHANGE_FRACTION * base.idx("width_m").f())
                || (p.idx("center_offset_left_m").f() - base.idx("center_offset_left_m").f()).abs() > PAIR_MERGE_OFFSET_M
            {
                return (V::None, true);
            }
        }
        let mut out = base;
        out.update(&dict! {"source_kind" => "native_wall_window_pair", "pair_count" => pairs.len()});
        (out, false)
    }

    pub fn open_loop_orbit(&self) -> bool {
        self.continue_route
            && self.entry_kind == Some("lane")
            && self.handover_observed
            && self.route_stage == "first_target"
            && self.first_pass_evidence.is_none()
    }

    pub fn clear_exit_prepare_window(&mut self) {
        self._first_exit_prepare_history.clear();
        self._quality_exit_prepare_ready = false;
    }

    /// _latch_exit_prepare(scan, control, basis, history).
    pub fn latch_exit_prepare(&mut self, scan: &V, control: &V, basis: &str, history: Option<Vec<Row>>) {
        let points = self.full_target_points(&self.compact_target.clone(), scan, &self.orbit_track_id.clone());
        let history = history.unwrap_or_else(|| self._first_exit_prepare_history.clone());
        let Some(points) = points else { return };
        if history.is_empty() {
            return;
        }
        self.first_pass_preparing = true;
        self.orbit_left_entry_boost = false;
        let n = points.len() as f64;
        self.first_exit_prepare_evidence = dict! {
            "source_seq" => scan.idx("seq").clone(), "source_at_ms" => scan.idx("at_ms").clone(),
            "received_at" => scan.idx("received_at").clone(),
            "center_x_m" => fsum_py(points.iter().map(|p| p.0)) / n,
            "track_id" => self.orbit_track_id.clone(),
            "support_bins" => self.compact_target.idx("support_bins").clone(),
            "observed_frontmost_x_m" => points.iter().map(|p| p.0).fold(f64::NEG_INFINITY, pmax),
            "observed_left_body_gap_m" => points.iter().map(|p| p.1).fold(INF, pmin) - SIDE_BODY_EXTENT_M,
            "observation_count" => history.len(), "first_source_seq" => history[0].0,
            "last_preparation_source_seq" => history[history.len() - 1].0,
            "publication_span_s" => (history[history.len() - 1].1 - history[0].1) as f64 / 1000.,
            "receive_span_s" => history[history.len() - 1].2 - history[0].2,
            "adopted_control" => control.pick(&["motor", "servo", "tick", "seq", "command_acked"]),
            "basis" => basis, "action" => "release_left_toward_neutral_only",
            "observed_rear_pass" => false, "right_steering_authorized" => false,
            "physical_cone_pass_certified" => false,
        };
        let evidence = self.first_exit_prepare_evidence.clone();
        self.route_event("first_exit_prepare", scan, control, evidence);
    }

    // ------------------------------------------------------------ TurnMotion.update

    pub fn tm_update(&mut self, scan: &V, lidar_age_s: &V, now: f64, control: &V, safe: &V, presteer_wait: &V) -> V {
        if !now.is_finite() || now < self.last_now {
            self.lock("invalid_turn_receive_clock");
            let at = self.last_now;
            return self.result(at);
        }
        self.last_now = now;
        if self.phase == "locked" {
            return self.result(now);
        }
        if !presteer_wait.is_bool() {
            self.lock("invalid_turn_presteer_wait");
            return self.result(now);
        }
        let presteer_wait = presteer_wait.is_true();
        self.presteer_waiting = false;
        if !safe.is_true() {
            self.lock("turn_safety_rejected");
            return self.result(now);
        }
        if self.phase == "drive" && now - self.drive_since.unwrap() >= self.max_drive_s {
            self.begin_coast("left_turn_drive_timeout", now);
        }
        if self.phase == "presteer" && now - self.started_at >= self.max_presteer_s {
            self.lock("left_turn_presteer_timeout");
            return self.result(now);
        }
        if !lidar_age_s.is_num()
            || !(0. ..SCAN_AGE_S).contains(&lidar_age_s.f())
            || !scan.is_dict()
            || scan.get("frame_id").os() != Some(FRAME)
            || !scan.get("seq").is_int()
            || scan.get("seq").i() < 0
            || !scan.get("at_ms").is_int()
            || scan.get("at_ms").i() < 0
        {
            self.lock("turn_scan_stale_or_invalid");
            return self.result(now);
        }
        let candidates = scan.get("corridor_candidates").clone();
        if !candidates.is_list() || candidates.len() > 24 || candidates.l().iter().any(|c| !valid_candidate_default(c)) {
            self.lock("invalid_native_corridor_candidates");
            return self.result(now);
        }
        let signature = V::list(vec![
            candidates.clone(),
            get_or(scan, "wall_candidates", &V::list(vec![])).clone(),
            scan.get("left_turn_goal").clone(),
            scan.get("ranges").clone(),
        ]);
        if !control.is_dict()
            || !control.get("armed").is_bool()
            || !control.get("motor").is_int()
            || !control.get("servo").is_int()
            || !control.get("tick").is_int()
            || control.get("tick").i() < 0
            || !control.get("seq").is_int()
            || control.get("seq").i() < 0
            || !(SERVO_MIN..=SERVO_MAX).contains(&control.get("servo").i())
        {
            self.lock("invalid_turn_adoption_feedback");
            return self.result(now);
        }
        let (tick, cseq) = (control.idx("tick").i(), control.idx("seq").i());
        if let Some(last_tick) = self.last_control_tick
            && (tick < last_tick || cseq < self.last_control_seq.unwrap())
        {
            self.lock("reordered_turn_adoption_feedback");
            return self.result(now);
        }
        let new_feedback = self.last_control_tick.is_none_or(|last| tick > last);
        self.last_control_tick = Some(tick);
        self.last_control_seq = Some(cseq);
        let armed = control.idx("armed").is_true();
        let motor = control.idx("motor").i();
        if presteer_wait && (self.phase != "presteer" || motor != NEUTRAL) {
            self.lock("turn_presteer_wait_requires_neutral");
            return self.result(now);
        }
        if (self.phase == "drive" || self.phase == "coast") && !armed {
            self.lock("turn_bridge_locked");
            return self.result(now);
        }
        let (seq, published) = (scan.idx("seq").i(), scan.idx("at_ms").i());
        if self.last_seq == Some(seq) {
            if Some(published) != self.last_publication || !py_eq(&signature, &self.last_signature) {
                self.lock("duplicate_turn_scan_changed_content");
            } else if now - self.last_receive.unwrap() >= SCAN_AGE_S {
                self.lock("turn_scan_not_advancing");
            } else if self.phase == "coast" {
                self.slew(now);
            } else if presteer_wait && self.phase != "locked" {
                self.presteer_hold("left_turn_presteer_quality_hold");
            }
            return self.result(now);
        }
        let publication_dt = match self.last_publication {
            None => 0.,
            Some(last) => (published - last) as f64 / 1000.,
        };
        if let Some(last_seq) = self.last_seq
            && (seq < last_seq || publication_dt <= 0.)
        {
            self.lock("reordered_turn_scan");
            return self.result(now);
        }
        if let Some(last_receive) = self.last_receive
            && (publication_dt >= SCAN_AGE_S
                || now - last_receive >= SCAN_AGE_S
                || (publication_dt - (now - last_receive)).abs() > 0.15)
        {
            self.lock("turn_scan_clock_gap");
            return self.result(now);
        }
        self.last_seq = Some(seq);
        self.last_publication = Some(published);
        self.last_receive = Some(now);
        self.last_signature = signature;
        self.incoming_endpoint = V::None;
        self.natural_steering_target = None;
        let geometry_dt = match self.last_geometry_publication {
            None => 0.,
            Some(last) => (published - last) as f64 / 1000.,
        };
        let measured = except(&[Exc::ValueError], || self.measurement(scan, &candidates, geometry_dt));
        let candidate = match measured {
            Ok(c) => c,
            Err(error) => {
                let missing = error.msg == "left_turn_geometry_missing";
                if missing && self.phase == "coast" {
                    self.geometry_missing = true;
                    self.start_ready = false;
                    self.steering_target = NEUTRAL;
                    self.last_error = None;
                    self.slew(now);
                    return self.result(now);
                }
                if missing
                    && self.last_geometry_receive.is_some_and(|r| now - r <= SCAN_AGE_S + 1e-9)
                    && geometry_dt <= SCAN_AGE_S + 1e-9
                {
                    self.geometry_missing = true;
                    self.start_ready = false;
                    self.settle_since = None;
                    self.settle_feedback_ticks = 0;
                    self.alignment_count = 0;
                    self.alignment_evidence = None;
                    self.alignment_publication = None;
                    self.alignment_receive = None;
                    self.last_error = None;
                    self.reason = "left_turn_geometry_gap_hold".into();
                    if presteer_wait {
                        let r = self.reason.clone();
                        self.presteer_hold(&r);
                    } else if armed {
                        self.slew(now);
                    }
                    return self.result(now);
                }
                let reason = if missing { "left_turn_geometry_lost".to_string() } else { error.msg.clone() };
                self.lock(&reason);
                return self.result(now);
            }
        };
        self.last_geometry_receive = Some(now);
        self.last_geometry_publication = Some(published);
        self.last_geometry_seq = Some(seq);
        self.geometry_missing = false;
        self.corridor = candidate.clone();
        self.selected = true;
        self.start_ready = true;
        self.endpoint_observation(scan);
        if self.phase == "coast" {
            self.steering_target = self.coast_target(&candidate);
            self.slew(now);
            return self.result(now);
        }
        if presteer_wait {
            self.presteer_hold("left_turn_presteer_quality_hold");
            return self.result(now);
        }
        let target = self.target(&candidate, publication_dt);
        self.natural_steering_target = Some(target);
        if self.phase == "presteer" {
            if target <= NEUTRAL {
                self.steering_target = NEUTRAL;
                self.start_ready = false;
                self.settle_since = None;
                self.settle_feedback_ticks = 0;
                self.reason = "left_turn_presteer_left_target_required".into();
                if armed {
                    self.slew(now);
                }
                return self.result(now);
            }
            let target = self.presteer_target(target);
            if self.steering_target == NEUTRAL || (target - self.steering_target).abs() > 10 {
                self.steering_target = target;
                self.settle_since = None;
                self.settle_feedback_ticks = 0;
            }
            if !armed {
                self.reason = "left_turn_neutral_preview".into();
                return self.result(now);
            }
            self.slew(now);
            let acked = if control.has("command_acked") { control.get("command_acked").is_true() } else { true };
            let adopted = self.servo == self.steering_target
                && motor == NEUTRAL
                && control.idx("servo").i() == self.steering_target
                && acked;
            if !adopted {
                self.settle_since = None;
                self.settle_feedback_ticks = 0;
                self.reason = "left_turn_presteering".into();
            } else {
                if self.settle_since.is_none() && new_feedback {
                    self.settle_since = Some(now);
                }
                if new_feedback {
                    self.settle_feedback_ticks += 1;
                }
                self.reason = "left_turn_steering_allowance".into();
                if new_feedback
                    && self.settle_since.is_some()
                    && self.settle_feedback_ticks >= 3
                    && now - self.settle_since.unwrap() + 1e-9 >= STEERING_ALLOWANCE_S
                {
                    self.phase = "drive";
                    self.drive_since = Some(now);
                    self.reason = "left_turn_tracking_candidate".into();
                    self.presteer_ended_at = Some(now);
                }
            }
            return self.result(now);
        }
        self.steering_target = target;
        self.slew(now);
        let heading = candidate.idx("heading_left_rad").f();
        let offset = candidate.idx("center_offset_left_m").f();
        let available = candidate.idx("width_m").f() / 2. - TURN_LATERAL_MARGIN_M;
        let aligned = available > 0. && heading.abs() <= radians(5.) && offset.abs() < available;
        if aligned {
            self.alignment_evidence = Some(if self.geometry_mode == Some("corridor") { "double_wall" } else { "tracked_goal_wall" });
            self.alignment_count += 1;
            if self.alignment_publication.is_none() {
                self.alignment_publication = Some(published);
                self.alignment_receive = Some(now);
            }
            if self.alignment_count >= 3
                && (published - self.alignment_publication.unwrap()) as f64 / 1000. >= 0.25
                && now - self.alignment_receive.unwrap() >= 0.25
            {
                self.observed_alignment = true;
                self.begin_coast("left_turn_observed_corridor_alignment", now);
            }
        } else {
            self.alignment_count = 0;
            self.alignment_evidence = None;
            self.alignment_publication = None;
            self.alignment_receive = None;
            if heading < -radians(8.) {
                self.begin_coast("left_turn_heading_overshoot", now);
            }
        }
        self.result(now)
    }

    /// _opening_candidate(wall, width).
    fn opening_candidate(wall: &V, width: f64) -> V {
        dict! {
            "heading_left_rad" => wall.idx("heading_left_rad").clone(),
            "center_offset_left_m" => wall.idx("rho_left_m").f() + width / 2.,
            "width_m" => width, "candidate_only" => true, "turn_path_certified" => false,
            "origin_between_walls" => false,
        }
    }

    /// _wall_matches(walls, publication_dt, scan).
    fn wall_matches(&self, walls: &[V], publication_dt: f64, scan: &V) -> Vec<V> {
        let old = &self.outer_wall;
        let gate = pmin(radians(30.), 0.12 + radians(90.) * publication_dt);
        let overlaps_previous = |w: &V| -> bool {
            let theta = w.idx("heading_left_rad").f();
            let (a, b) = (interval(old, theta), interval(w, theta));
            pmin(a.1, b.1) - pmax(a.0, b.0) >= 0.10
        };
        let matches: Vec<V> = walls
            .iter()
            .filter(|w| {
                wrap(w.idx("heading_left_rad").f() - old.idx("heading_left_rad").f()).abs() <= gate
                    && (w.idx("rho_left_m").f() - old.idx("rho_left_m").f()).abs() <= 0.30
                    && overlaps_previous(w)
            })
            .cloned()
            .collect();
        let mut distinct: Vec<V> = Vec::new();
        for wall in matches {
            if !distinct.iter().any(|d| py_eq(d, &wall)) {
                distinct.push(wall);
            }
        }
        let matches = distinct;
        if matches.len() <= 1 {
            return matches;
        }
        let supports = surface_supports(scan, &matches);
        let mut order: Vec<usize> = (0..matches.len()).collect();
        order.sort_by(|&i, &j| {
            let (a, b) = (matches[i].idx("support_span_m").f(), matches[j].idx("support_span_m").f());
            b.partial_cmp(&a).unwrap_or(std::cmp::Ordering::Equal)
        });
        let mut unique: Vec<Vec<(V, Support)>> = Vec::new();
        for index in order {
            let wall = &matches[index];
            let support = &supports[index];
            let theta = wall.idx("heading_left_rad").f();
            let a = interval(wall, theta);
            let same = |u: &V, evidence: &Support| -> bool {
                if py_eq(wall, u) {
                    return true;
                }
                if let Some(observed) = same_surface(support, evidence) {
                    return observed;
                }
                let iu = interval(u, theta);
                wrap(wall.idx("heading_left_rad").f() - u.idx("heading_left_rad").f()).abs() <= radians(2.)
                    && (wall.idx("rho_left_m").f() - u.idx("rho_left_m").f()).abs() <= 0.05
                    && pmin(a.1, iu.1) - pmax(a.0, iu.0) >= 0.10
            };
            let group = unique.iter().position(|group| group.iter().all(|(u, evidence)| same(u, evidence)));
            match group {
                None => unique.push(vec![(wall.clone(), support.clone())]),
                Some(g) => unique[g].push((wall.clone(), support.clone())),
            }
        }
        unique.into_iter().map(|group| group[0].0.clone()).collect()
    }

    /// TurnMotion._measurement.
    fn tm_measurement(&mut self, scan: &V, candidates: &V, publication_dt: f64) -> V {
        let opening = scan.get("left_turn_goal").clone();
        let walls = get_or(scan, "wall_candidates", &V::list(vec![])).clone();
        if !walls.is_list()
            || walls.len() > 64
            || walls.l().iter().any(|w| !valid_wall(w))
            || (opening.is_some() && !valid_opening(&opening, scan))
        {
            value_error("invalid_native_turn_wall_goal")
        }
        if !self.selected && opening.is_some() {
            let h = opening.idx("heading_left_rad").f();
            let p = opening.idx("target_point_left_m");
            if !(radians(15.) < h && h < radians(150.)) || p.idx("x_m").f() <= 0. || p.idx("y_m").f() <= 0. {
                value_error("left_turn_goal_not_a_forward_left_exit")
            }
            self.outer_wall = opening.idx("outer_wall").clone();
            self.turn_goal = opening.clone();
            self.geometry_mode = Some("opening_wall");
            return Self::opening_candidate(&self.outer_wall, opening.idx("width_m").f());
        }
        if self.geometry_mode == Some("opening_wall") {
            let mut all: Vec<V> = walls.l().to_vec();
            if opening.truthy() {
                all.push(opening.idx("outer_wall").clone());
            }
            let matches = self.wall_matches(&all, publication_dt, scan);
            if matches.len() > 1 {
                value_error("left_turn_outer_wall_ambiguous")
            }
            if matches.is_empty() {
                if walls.truthy() || opening.truthy() {
                    value_error("left_turn_outer_wall_identity_jump")
                }
                value_error("left_turn_geometry_missing")
            }
            let wall = matches[0].clone();
            let mut width = self.corridor.idx("width_m").f();
            let mut update_from_opening = opening.is_some();
            if opening.truthy() {
                let ow = opening.idx("width_m").f();
                let delta = (ow - width).abs();
                let limit = self.exit_width_change_limit(width);
                let accepted = delta <= limit || isclose(delta, limit, 0., 1e-12);
                self.exit_width_change = dict! {
                    "source_seq" => scan.idx("seq").clone(), "source_at_ms" => scan.idx("at_ms").clone(),
                    "previous_width_m" => width, "current_width_m" => opening.idx("width_m").clone(),
                    "absolute_change_m" => delta, "relative_change" => div(delta, width), "allowed_change_m" => limit,
                    "accepted" => accepted, "wall_identity_checked" => true,
                    "action" => if accepted { "adopt_current_width" } else { "reject_width_jump" },
                };
                if !accepted {
                    if !self.continue_with_tracked_width() {
                        value_error("left_turn_exit_width_jump")
                    }
                    update_from_opening = false;
                    self.exit_width_change.set("action", "track_current_wall_keep_previous_width");
                } else {
                    width = ow;
                    self.turn_goal = opening.clone();
                }
            }
            if !update_from_opening {
                let theta = wall.idx("heading_left_rad").f();
                let (a, b) = (wall.idx("support_start_left_m"), wall.idx("support_end_left_m"));
                let point = dict! {
                    "x_m" => (a.idx("x_m").f() + b.idx("x_m").f()) / 2. - theta.sin() * width / 2.,
                    "y_m" => (a.idx("y_m").f() + b.idx("y_m").f()) / 2. + theta.cos() * width / 2.,
                };
                let mut goal = self.turn_goal.clone();
                goal.update(&dict! {
                    "heading_left_rad" => theta, "center_offset_left_m" => wall.idx("rho_left_m").f() + width / 2.,
                    "target_point_left_m" => point, "outer_wall" => wall.clone(),
                    "target_support_ray" => V::None, "observation_type" => "tracked_actual_outer_wall",
                });
                self.turn_goal = goal;
            }
            self.outer_wall = wall.clone();
            let candidate = Self::opening_candidate(&wall, width);
            let handover: Vec<&V> = candidates
                .l()
                .iter()
                .filter(|c| {
                    wrap(c.idx("heading_left_rad").f() - wall.idx("heading_left_rad").f()).abs() <= radians(8.)
                        && (c.idx("center_offset_left_m").f() - candidate.idx("center_offset_left_m").f()).abs() <= 0.12
                        && (c.idx("width_m").f() - width).abs() <= 0.15
                })
                .collect();
            if handover.len() > 1 {
                value_error("left_turn_corridor_handover_ambiguous")
            }
            if let Some(c) = handover.first() {
                self.geometry_mode = Some("corridor");
                return (*c).clone();
            }
            return candidate;
        }
        if candidates.l().is_empty() && self.selected {
            value_error("left_turn_geometry_missing")
        }
        let result = self.candidate(candidates, publication_dt);
        self.geometry_mode = Some("corridor");
        result
    }

    /// ManeuverSequence._measurement.
    pub fn measurement(&mut self, scan: &V, candidates: &V, publication_dt: f64) -> V {
        let candidate = self.tm_measurement(scan, candidates, publication_dt);
        if self.geometry_mode == Some("opening_wall") {
            self.entry_bearing_required = true;
        }
        candidate
    }

    /// TurnMotion._candidate.
    fn candidate(&self, candidates: &V, publication_dt: f64) -> V {
        let matches: Vec<&V> = if !self.selected {
            candidates
                .l()
                .iter()
                .filter(|c| radians(15.) < c.idx("heading_left_rad").f() && c.idx("heading_left_rad").f() < radians(150.))
                .collect()
        } else {
            let old = &self.corridor;
            let heading_gate = pmin(radians(30.), 0.12 + radians(90.) * publication_dt);
            candidates
                .l()
                .iter()
                .filter(|c| {
                    wrap(c.idx("heading_left_rad").f() - old.idx("heading_left_rad").f()).abs() <= heading_gate
                        && (c.idx("center_offset_left_m").f() - old.idx("center_offset_left_m").f()).abs() <= 0.25
                        && (c.idx("width_m").f() - old.idx("width_m").f()).abs() <= 0.15
                })
                .collect()
        };
        if matches.len() != 1 {
            value_error(if matches.len() > 1 {
                "left_corridor_ambiguous"
            } else if self.selected {
                "left_corridor_lost_or_jumped"
            } else {
                "left_corridor_unknown"
            })
        }
        matches[0].clone()
    }

    /// TurnMotion._coast_target.
    fn coast_target(&self, candidate: &V) -> i64 {
        let width = candidate.idx("width_m").f();
        let available = width / 2. - TURN_LATERAL_MARGIN_M;
        let heading = candidate.idx("heading_left_rad").f();
        let offset = candidate.idx("center_offset_left_m").f();
        if heading.abs() > radians(15.) || available <= 0. || offset.abs() >= available {
            return NEUTRAL;
        }
        let lookahead = pmax(0.35, pmin(1., width / 2.));
        let error = heading + offset.atan2(lookahead);
        clampi(NEUTRAL + round(140. * error), NEUTRAL - COAST_SERVO_DELTA, NEUTRAL + COAST_SERVO_DELTA)
    }

    fn exit_width_change_limit(&self, previous_width: f64) -> f64 {
        pmax(0.15, MAX_EXIT_WIDTH_CHANGE_FRACTION * previous_width)
    }

    fn continue_with_tracked_width(&self) -> bool {
        self.phase == "drive"
    }

    /// ManeuverSequence._endpoint_observation (TurnMotion's first).
    pub fn endpoint_observation(&mut self, scan: &V) {
        let opening = scan.get("left_turn_goal").clone();
        self.incoming_endpoint = if opening.is_none() {
            V::None
        } else {
            dict! {
                "source_seq" => scan.idx("seq").clone(), "source_at_ms" => scan.idx("at_ms").clone(),
                "incoming_left_end_m" => opening.idx("incoming_left_end_m").clone(),
                "incoming_left_end_support" => opening.idx("incoming_left_end_support").clone(),
                "known_open_fraction" => opening.idx("known_open_fraction").clone(),
                "candidate_only" => true, "turn_path_certified" => false,
            }
        };
        if !self.entry_bearing_required || self.entry_bearing_released {
            return;
        }
        let observation = if opening.is_some() {
            dict! {
                "endpoint_return" => opening.idx("incoming_left_end_support").clone(),
                "heading_left_rad" => opening.idx("incoming_heading_left_rad").clone(),
                "observation_type" => "full_opening", "wall" => V::None,
            }
        } else {
            self.current_incoming_endpoint(scan)
        };
        if observation.is_none() {
            self.entry_bearing = V::None;
            self._entry_ready_count = 0;
            self._entry_ready_receive = V::None;
            self._entry_ready_publication = None;
            self._entry_ready_observation_type = None;
            self._entry_ready_history.clear();
            return;
        }
        let point = observation.idx("endpoint_return").idx("point_left_m").clone();
        let heading = observation.idx("heading_left_rad").f();
        let (c, s) = (heading.cos(), heading.sin());
        let (px, py) = (point.idx("x_m").f(), point.idx("y_m").f());
        let received = scan.get("received_at").clone();
        let prior = self._incoming_association.clone();
        if received.is_num()
            && received.f() >= 0.
            && (0. ..SCAN_AGE_S).contains(&(self.last_now - received.f()))
            && (prior.is_none()
                || (scan.idx("seq").i() > prior.idx("source_seq").i()
                    && scan.idx("at_ms").i() > prior.idx("source_at_ms").i()
                    && received.f() > prior.idx("source_received_at").f()))
        {
            self._incoming_association = dict! {
                "source_seq" => scan.idx("seq").clone(), "source_at_ms" => scan.idx("at_ms").clone(),
                "source_received_at" => received.clone(), "heading_left_rad" => heading,
                "rho_left_m" => -s * px + c * py,
                "endpoint_return" => observation.idx("endpoint_return").clone(),
            };
        }
        let corners = [
            (-REAR_BODY_EXTENT_M, -SIDE_BODY_EXTENT_M),
            (-REAR_BODY_EXTENT_M, SIDE_BODY_EXTENT_M),
            (FRONT_BODY_EXTENT_M, -SIDE_BODY_EXTENT_M),
            (FRONT_BODY_EXTENT_M, SIDE_BODY_EXTENT_M),
        ];
        let front = corners.iter().map(|&(x, y)| c * x + s * y).fold(f64::NEG_INFINITY, pmax) + MANEUVER_BODY_CLEARANCE_M;
        let side = corners.iter().map(|&(x, y)| -s * x + c * y).fold(f64::NEG_INFINITY, pmax) + MANEUVER_BODY_CLEARANCE_M;
        let forward_gap = c * px + s * py - front;
        let lateral_gap = -s * px + c * py - side;
        let bearing = heading + pmax(0., lateral_gap).atan2(pmax(0., forward_gap));
        let mut cap = clampi(NEUTRAL + round(LEFT_TRIAL_GAIN * bearing), NEUTRAL, SERVO_MAX);
        if lateral_gap <= 0. {
            cap = NEUTRAL;
        }
        let front_projection_reached = forward_gap <= 0. && lateral_gap > 0. && cap > NEUTRAL;
        let permits_prepared_left = cap >= self.initial_presteer_pwm.unwrap();
        let otype = observation.idx("observation_type").s().to_string();
        let observation_clock: V =
            if otype == "tracked_current_incoming_wall" { received.clone() } else { V::Float(self.last_now) };
        if self._entry_ready_observation_type.as_deref() != Some(otype.as_str()) {
            self._entry_ready_count = 0;
            self._entry_ready_receive = V::None;
            self._entry_ready_publication = None;
            self._entry_ready_history.clear();
        }
        self._entry_ready_observation_type = Some(otype.clone());
        let at_ms = scan.idx("at_ms").i();
        if permits_prepared_left {
            if self._entry_ready_count == 0 {
                self._entry_ready_receive = observation_clock.clone();
                self._entry_ready_publication = Some(at_ms);
            }
            self._entry_ready_count += 1;
            if self._entry_ready_count >= 3
                && observation_clock.f() - self._entry_ready_receive.f() >= 0.25
                && (at_ms - self._entry_ready_publication.unwrap()) as f64 / 1000. >= 0.25
            {
                self.entry_bearing_released = true;
            }
            if self.continue_route
                && received.is_num()
                && consecutive_fresh(&mut self._entry_ready_history, scan, ROUTE_ENTRY_RELEASE_FRAMES, SCAN_AGE_S)
            {
                self.entry_bearing_released = true;
            }
        } else {
            self._entry_ready_count = 0;
            self._entry_ready_receive = V::None;
            self._entry_ready_publication = None;
            self._entry_ready_history.clear();
        }
        if front_projection_reached {
            self.entry_bearing_released = true;
        }
        self._entry_last_receive = observation_clock;
        self._entry_last_publication = Some(at_ms);
        let near_bins = get_or(&observation, "near_support_bins", &V::list(vec![])).clone();
        self.entry_bearing = dict! {
            "source_seq" => scan.idx("seq").clone(), "source_at_ms" => at_ms,
            "source_received_at" => received,
            "endpoint_return" => observation.idx("endpoint_return").clone(),
            "observation_type" => otype.as_str(),
            "current_incoming_wall" => observation.idx("wall").clone(),
            "current_near_support_bins" => near_bins,
            "adjacent_far_return" => observation.get("adjacent_far_return").clone(),
            "incoming_heading_left_rad" => heading, "body_forward_support_m" => front,
            "body_left_support_m" => side, "forward_gap_m" => forward_gap,
            "lateral_gap_m" => lateral_gap, "clearance_bearing_left_rad" => bearing,
            "steering_cap_pwm" => cap, "prepared_left_pwm" => self.initial_presteer_pwm,
            "confirmation_count" => self._entry_ready_count,
            "released" => self.entry_bearing_released,
            "release_basis" => if front_projection_reached { V::from("current_endpoint_at_body_front_projection") }
                else if self.entry_bearing_released { V::from("prepared_left_bearing_matured") } else { V::None },
            "route_fresh_frames" => if self.continue_route { V::from(self._entry_ready_history.len()) } else { V::None },
            "scope" => "early_turn_endpoint_bearing_heuristic",
            "endpoint_passed_proven" => false, "swept_path_certified" => false,
            "physical_curvature_calibrated" => false,
        };
    }

    /// _current_incoming_endpoint(scan).
    fn current_incoming_endpoint(&self, scan: &V) -> V {
        let old = &self._incoming_association;
        if self.phase != "drive" || old.is_none() {
            return V::None;
        }
        let publication_dt = (scan.idx("at_ms").i() - old.idx("source_at_ms").i()) as f64 / 1000.;
        let received = scan.get("received_at");
        let osr = old.idx("source_received_at").f();
        if scan.idx("seq").i() <= old.idx("source_seq").i()
            || !(0. < publication_dt && publication_dt < SCAN_AGE_S)
            || !received.is_num()
            || received.f() < 0.
            || !(0. ..SCAN_AGE_S).contains(&(self.last_now - received.f()))
            || received.f() <= osr
            || !(0. ..SCAN_AGE_S).contains(&(self.last_now - osr))
            || (publication_dt - (received.f() - osr)).abs() > 0.15
        {
            return V::None;
        }
        let ranges = scan.get("ranges");
        let walls = scan.get("wall_candidates");
        let ok_ranges = ranges.ol().is_some_and(|l| {
            l.len() == 360 && l.iter().all(|r| r.is_none() || (r.is_num() && (0.02..=12.).contains(&r.f())))
        });
        if !ok_ranges || !walls.is_list() || walls.len() > 64 || walls.l().iter().any(|w| !valid_wall(w)) {
            return V::None;
        }
        let gate = pmin(radians(30.), 0.12 + radians(90.) * publication_dt);
        let previous = old.idx("endpoint_return").idx("point_left_m");
        let oh = old.idx("heading_left_rad").f();
        let mut matches: Vec<V> = Vec::new();
        for original in walls.l() {
            let mut wall = original.clone();
            let mut heading = wall.idx("heading_left_rad").f();
            if wrap(heading - oh).abs() > PI / 2. {
                heading = wrap(heading + PI);
                wall.set("heading_left_rad", heading);
                let rho = -wall.idx("rho_left_m").f();
                wall.set("rho_left_m", rho);
                let (start, end) = (wall.idx("support_start_left_m").clone(), wall.idx("support_end_left_m").clone());
                wall.set("support_start_left_m", end);
                wall.set("support_end_left_m", start);
            }
            let end = wall.idx("support_end_left_m").clone();
            if wrap(heading - oh).abs() > gate
                || wall.idx("rho_left_m").f() <= 0.
                || (wall.idx("rho_left_m").f() - old.idx("rho_left_m").f()).abs() > 0.30
                || hypot(end.idx("x_m").f() - previous.idx("x_m").f(), end.idx("y_m").f() - previous.idx("y_m").f()) > 0.30
            {
                continue;
            }
            let observation = Self::observed_wall_endpoint(&wall, ranges);
            if observation.is_some() && !matches.iter().any(|item| py_eq(&wall, item.idx("wall"))) {
                matches.push(observation);
            }
        }
        if matches.is_empty() {
            return V::None;
        }
        let walls_m: Vec<V> = matches.iter().map(|m| m.idx("wall").clone()).collect();
        let supports = surface_supports(scan, &walls_m);
        let mut groups: Vec<Vec<(V, Support)>> = Vec::new();
        for (observation, support) in matches.iter().zip(supports.iter()) {
            if support.is_none() {
                continue;
            }
            let index = observation.idx("endpoint_return").idx("index").clone();
            let group = groups.iter().position(|g| {
                g.iter().all(|(prior, evidence)| {
                    py_eq(&index, prior.idx("endpoint_return").idx("index")) && same_surface(support, evidence) == Some(true)
                })
            });
            match group {
                None => groups.push(vec![(observation.clone(), support.clone())]),
                Some(g) => groups[g].push((observation.clone(), support.clone())),
            }
        }
        if groups.len() == 1 { groups[0][0].0.clone() } else { V::None }
    }

    /// _observed_wall_endpoint(wall, ranges).
    fn observed_wall_endpoint(wall: &V, ranges: &V) -> V {
        let heading = wall.idx("heading_left_rad").f();
        let end = wall.idx("support_end_left_m");
        let (c, s) = (heading.cos(), heading.sin());
        let along = |x: f64, y: f64| c * x + s * y;
        let normal = |x: f64, y: f64| -s * x + c * y;
        let st = wall.idx("support_start_left_m");
        let low = along(st.idx("x_m").f(), st.idx("y_m").f());
        let high = along(end.idx("x_m").f(), end.idx("y_m").f());
        let rho = wall.idx("rho_left_m").f();
        if low >= high || (normal(end.idx("x_m").f(), end.idx("y_m").f()) - rho).abs() > 0.05 {
            return V::None;
        }
        let index = imod(round(-degrees(end.idx("y_m").f().atan2(end.idx("x_m").f()))), 360) as usize;
        let list = ranges.l();
        let distance_v = &list[index];
        if distance_v.is_none() {
            return V::None;
        }
        let distance = distance_v.f();
        let point = |i: usize| -> (f64, f64) {
            let r = list[i].f();
            let a = radians(i as f64);
            (r * a.cos(), -r * a.sin())
        };
        let measured = point(index);
        if hypot(measured.0 - end.idx("x_m").f(), measured.1 - end.idx("y_m").f()) > 1e-6 {
            return V::None;
        }
        for direction in [-1i64, 1] {
            let outside = imod(index as i64 + direction, 360) as usize;
            let far = &list[outside];
            if far.is_none() || far.f() <= distance + 0.18 {
                continue;
            }
            let ao = radians(outside as f64);
            let adjacent = (distance * ao.cos(), -distance * ao.sin());
            let po = point(outside);
            if along(adjacent.0, adjacent.1) <= high || along(po.0, po.1) <= high {
                continue;
            }
            let mut previous = measured;
            let mut support_bins = vec![index as i64];
            for step in 1..16 {
                let inside = imod(index as i64 - direction * step, 360) as usize;
                if list[inside].is_none() {
                    break;
                }
                let current = point(inside);
                let a = along(current.0, current.1);
                if !(low - 1e-6 <= a && a < high)
                    || (normal(current.0, current.1) - rho).abs() > 0.05
                    || hypot(current.0 - previous.0, current.1 - previous.1) > 0.18
                {
                    break;
                }
                support_bins.push(inside as i64);
                previous = current;
            }
            if support_bins.len() == 16 {
                return dict! {
                    "endpoint_return" => dict! {
                        "index" => index, "angle_left_rad" => wrap(-radians(index as f64)),
                        "range_m" => distance_v.clone(),
                        "point_left_m" => dict! {"x_m" => measured.0, "y_m" => measured.1},
                    },
                    "heading_left_rad" => heading,
                    "observation_type" => "tracked_current_incoming_wall", "wall" => wall.clone(),
                    "near_support_bins" => V::from(support_bins),
                    "adjacent_far_return" => dict! {"index" => outside, "range_m" => far.clone()},
                };
            }
        }
        V::None
    }

    /// _entry_target(target).
    fn entry_target(&mut self, target: i64) -> i64 {
        if !self.entry_bearing_required || self.entry_bearing_released {
            return target;
        }
        if self.entry_bearing.is_some() {
            return target.min(self.entry_bearing.idx("steering_cap_pwm").i());
        }
        if self.phase == "presteer" {
            return NEUTRAL;
        }
        if self._entry_last_receive.is_some()
            && self.last_now - self._entry_last_receive.f() < SCAN_AGE_S
            && (self.last_publication.unwrap() - self._entry_last_publication.unwrap()) as f64 / 1000. < SCAN_AGE_S
        {
            return self.servo;
        }
        let at = self.last_now;
        self.begin_coast("left_turn_entry_endpoint_lost", at);
        NEUTRAL
    }

    /// ManeuverSequence._presteer_target.
    fn presteer_target(&mut self, target: i64) -> i64 {
        let base = self.initial_presteer_pwm.unwrap_or(target);
        self.entry_target(base)
    }

    /// ManeuverSequence._target(candidate, publication_dt).
    pub fn target(&mut self, candidate: &V, publication_dt: f64) -> i64 {
        let heading = candidate.idx("heading_left_rad").f();
        let offset = candidate.idx("center_offset_left_m").f();
        let lookahead = pmax(0.35, pmin(1., candidate.idx("width_m").f() / 2.));
        let error = if self.geometry_mode == Some("opening_wall") {
            let point = self.turn_goal.idx("target_point_left_m");
            if point.idx("x_m").f() > 0. {
                0.65 * heading + 0.35 * point.idx("y_m").f().atan2(point.idx("x_m").f())
            } else {
                heading + offset.atan2(lookahead)
            }
        } else {
            heading + offset.atan2(lookahead)
        };
        let mut damped = error;
        if let Some(last) = self.last_error
            && 0.05 <= publication_dt
            && publication_dt < SCAN_AGE_S
        {
            let rate = (error - last) / publication_dt;
            if rate.abs() <= radians(90.) {
                let release = error + 0.35 * rate;
                if error > 0. {
                    damped = pmax((1. - MAX_TREND_RELEASE_FRACTION) * error, pmin(error, release));
                }
            }
        }
        self.last_error = Some(error);
        let mut target = clampi(NEUTRAL + round(LEFT_TRIAL_GAIN * damped), NEUTRAL, SERVO_MAX);
        let alignment_target = target;
        let opening_drive = self.phase == "drive" && self.geometry_mode == Some("opening_wall");
        let transition_trigger =
            opening_drive && heading > 0. && self.turn_goal.idx("target_point_left_m").idx("y_m").f() <= 0.;
        if transition_trigger {
            self.left_handover_preparing = true;
        }
        let continuing_left = opening_drive && heading > 0. && (self.left_handover_preparing || self.entry_bearing_released);
        if continuing_left {
            target = target.max(self.initial_presteer_pwm.unwrap());
        }
        let cs = self._current_scan.clone();
        self.left_turn_feedback = dict! {
            "source_seq" => if cs.truthy() { cs.get("seq").clone() } else { V::None },
            "source_at_ms" => if cs.truthy() { cs.get("at_ms").clone() } else { V::None },
            "geometry_mode" => self.geometry_mode,
            "opening_left_continuation_active" => continuing_left,
            "handover_preparation_trigger_current" => transition_trigger,
            "alignment_target_pwm" => alignment_target,
            "prepared_left_pwm" => self.initial_presteer_pwm,
            "nominal_target_pwm" => target,
            "inside_edge_release_remains_independent" => true,
            "physical_curvature_calibrated" => false,
        };
        if self.phase == "drive" {
            target = self.limit_left_for_known_points(target, lookahead);
        }
        let target = self.entry_target(target);
        self.left_turn_feedback.set("endpoint_limited_target_pwm", target);
        self._last_actual_left_target = if target > NEUTRAL { Some(target) } else { None };
        target
    }

    /// _relative_target_pwm(target).
    pub fn relative_target_pwm(&mut self, target: &V) -> i64 {
        let reference = self.orbit_reference_range_m.f();
        let bearing_term = ORBIT_BEARING_GAIN * (target.idx("bearing_left_rad").f() - ORBIT_BEARING_RAD);
        let range_term = div(ORBIT_RANGE_GAIN * (target.idx("range_m").f() - reference), reference);
        let correction = pmax(0., bearing_term) + pmax(0., range_term);
        let bias = if self.orbit_bias_pwm.is_some() { self.orbit_bias_pwm.i() } else { self.initial_presteer_pwm.unwrap() };
        let entry_base = bias.max(self.initial_presteer_pwm.unwrap());
        let mut requested = clampi(round(entry_base as f64 + correction), NEUTRAL, SERVO_MAX);
        if self.orbit_left_entry_boost {
            if target.idx("bearing_left_rad").f() >= ORBIT_BEARING_RAD {
                if self._orbit_abeam_count == 0 {
                    self._orbit_abeam_publication = target.idx("source_at_ms").clone();
                    self._orbit_abeam_receive = Some(self.last_now);
                }
                self._orbit_abeam_count += 1;
                if self._orbit_abeam_count >= 3
                    && (target.idx("source_at_ms").i() - self._orbit_abeam_publication.i()) as f64 / 1000. >= 0.25
                    && self.last_now - self._orbit_abeam_receive.unwrap() >= 0.25
                {
                    self.orbit_left_entry_boost = false;
                }
            } else {
                self._orbit_abeam_count = 0;
                self._orbit_abeam_publication = V::None;
                self._orbit_abeam_receive = None;
            }
            if self.orbit_left_entry_boost {
                requested = SERVO_MAX;
            }
        }
        self.orbit_feedback = dict! {
            "source_seq" => target.get("source_seq").clone(),
            "source_at_ms" => target.get("source_at_ms").clone(), "scope" => "bounded_orbit_entry_only",
            "adopted_handover_bias_pwm" => bias, "prepared_left_pwm" => self.initial_presteer_pwm,
            "entry_base_pwm" => entry_base, "bearing_term_pwm" => bearing_term,
            "range_term_pwm" => range_term,
            "applied_entry_correction_pwm" => correction,
            "left_entry_boost_active" => self.orbit_left_entry_boost,
            "abeam_confirmations" => self._orbit_abeam_count,
            "boost_scope" => "first_target_ahead_at_handover_until_current_abeam_evidence",
            "nominal_target_pwm" => requested, "inside_edge_release_remains_independent" => true,
            "physical_curvature_calibrated" => false,
        };
        // max(0., x) keeps 0. a float; the sum stays a float.
        requested
    }

    /// _limit_left_for_known_points(nominal, lookahead).
    pub fn limit_left_for_known_points(&mut self, nominal: i64, lookahead: f64) -> i64 {
        let scan = self._current_scan.clone();
        if !scan.is_dict() {
            return nominal;
        }
        let ranges = scan.get("ranges");
        let Some(list) = ranges.ol() else { return nominal };
        if list.len() != 360 {
            return nominal;
        }
        let rear = REAR_BODY_EXTENT_M + MANEUVER_BODY_CLEARANCE_M;
        let side = SIDE_BODY_EXTENT_M + MANEUVER_BODY_CLEARANCE_M;
        let mut selected = V::None;
        let mut target = nominal;
        for (index, distance_v) in list.iter().enumerate().take(360).skip(181) {
            if !distance_v.is_num() || !(0.02..=12.).contains(&distance_v.f()) {
                continue;
            }
            let distance = distance_v.f();
            let angle = radians(index as f64);
            let (x, y) = (distance * angle.cos(), -distance * angle.sin());
            if !(-rear < x && x <= lookahead) || y <= 0. {
                continue;
            }
            let clearance_bearing = pmax(0., y - side).atan2(x + rear);
            let point_target = SERVO_MAX.min(NEUTRAL + round(LEFT_TRIAL_GAIN * clearance_bearing));
            if point_target < target {
                target = point_target;
                selected = dict! {
                    "index" => index, "range_m" => distance_v.clone(),
                    "point_left_m" => dict! {"x_m" => x, "y_m" => y},
                    "clearance_bearing_rad" => clearance_bearing,
                };
            }
        }
        let bearing_demand = target;
        let target = nominal - round(MAX_INNER_RELEASE_FRACTION * (nominal - bearing_demand) as f64);
        self.inner_clearance = dict! {
            "source_seq" => scan.get("seq").clone(), "source_at_ms" => scan.get("at_ms").clone(),
            "scope" => "current_known_left_returns_only",
            "nominal_target_pwm" => nominal, "limited_target_pwm" => target,
            "clearance_bearing_demand_pwm" => bearing_demand,
            "max_release_fraction" => MAX_INNER_RELEASE_FRACTION,
            "active" => target < nominal, "limiting_return" => selected,
            "lookahead_m" => lookahead, "rear_reference_m" => -rear,
            "left_reference_m" => side, "swept_path_certified" => false,
            "physical_steering_confirmed" => false,
        };
        target
    }

    /// _route_left_hold_update.
    pub fn route_left_hold_update(&mut self, scan: &V, lidar_age_s: &V, now: f64, control: &V, safe: &V, presteer_wait: &V) -> V {
        let advancing = self.orbit_inputs(scan, lidar_age_s, now, control, safe, presteer_wait);
        if self.phase == "locked" {
            let at = self.last_now;
            return self.result(at);
        }
        let held = self.left_overshoot_evidence.idx("held_servo").i();
        let drive_since = self.drive_since.unwrap();
        if now - drive_since + 1e-9 >= self.max_drive_s || now - self._route_left_hold_since.unwrap() + 1e-9 >= ROUTE_LEFT_OVERSHOOT_HOLD_S {
            let reason = if now - drive_since >= self.max_drive_s {
                "maneuver_cumulative_drive_timeout"
            } else {
                "route_left_turn_overshoot_hold_expired"
            };
            self._route_left_hold_since = None;
            if !self.begin_adopted_left_coast(reason, now, control, false) {
                self.begin_coast(reason, now);
            }
            self.slew(now);
            return self.result(now);
        }
        if advancing {
            self.steering_target = held;
            self.reason = "route_left_turn_overshoot_hold_left".into();
            self.slew(now);
        }
        self.result(now)
    }

    /// _usable_target(target, scan).
    pub fn usable_target(&self, target: &V, scan: &V) -> bool {
        if self.entry_bearing_required && !self.entry_bearing_released {
            return false;
        }
        if !target.is_dict() || !target.get("confirmed").is_true() {
            return false;
        }
        let close_bound_role = self.continue_route
            && self.handover_observed
            && self.first_pass_evidence.is_none()
            && target.get("close_maintenance_observation").is_true()
            && self.full_target_points(target, scan, &self.orbit_track_id).is_some();
        let point = target.get("point_left_m");
        let point_ok = point.ol().is_some_and(|p| p.len() == 2 && p.iter().all(|v| v.is_num()));
        let low = if close_bound_role { 0.02 } else { crate::compact_target::MIN_RANGE_M };
        target.get("kind").os() == Some("lidar_compact_object")
            && target.get("semantic_class").os() == Some("unknown")
            && target.get("candidate_only").is_true()
            && target.get("source_seq").is_int()
            && py_eq(target.idx("source_seq"), scan.get("seq"))
            && target.get("source_at_ms").is_int()
            && py_eq(target.idx("source_at_ms"), scan.get("at_ms"))
            && point_ok
            && target.get("range_m").is_num()
            && low <= target.idx("range_m").f()
            && target.idx("range_m").f() <= 3.0
            && target.get("bearing_left_rad").is_num()
            && radians(45.) <= target.idx("bearing_left_rad").f()
            && target.idx("bearing_left_rad").f() <= radians(135.)
            && {
                let p = point.l();
                (hypot(p[0].f(), p[1].f()) - target.idx("range_m").f()).abs() <= 1e-6
                    && (p[1].f().atan2(p[0].f()) - target.idx("bearing_left_rad").f()).abs() <= 1e-6
            }
            && target.get("track_id").is_some()
    }

    /// _capture_compact_loss(scan, now, control, reason).
    pub fn capture_compact_loss(&mut self, scan: &V, now: f64, control: &V, reason: &str) {
        if self.compact_loss_evidence.is_none() {
            self.compact_loss_evidence = dict! {
                "decision_at" => now, "reason" => reason,
                "tracker_reason" => self.target_tracker.reason.as_str(),
                "tracker_error" => self.compact_target_error.clone(),
                "scan" => scan.pick(&["frame_id", "seq", "at_ms", "received_at", "ranges"]),
                "previous_confirmed_target" => self._last_confirmed_compact_target.clone(),
                "current_target" => self.compact_target.clone(),
                "control" => control.pick(&["armed", "motor", "servo", "tick", "seq", "command_acked"]),
                "physical_identity_verified" => false,
            };
        }
    }

    /// _observe_first_pass(scan, now, control).
    pub fn observe_first_pass(&mut self, scan: &V, now: f64, control: &V) {
        let target = self.compact_target.clone();
        let bins = if target.is_dict() { target.get("support_bins").clone() } else { V::None };
        if self.continue_route
            && (self.full_target_points(&target, scan, &self.orbit_track_id.clone()).is_none()
                || !scan.get("received_at").is_num()
                || !(0. ..SCAN_AGE_S).contains(&(now - scan.idx("received_at").f())))
        {
            self.clear_exit_prepare_window();
            self._first_pass_history.clear();
            return;
        }
        if target.is_none()
            || !py_eq(target.get("track_id"), &self.orbit_track_id)
            || !target.get("confirmed").truthy()
            || !target.get("tracking_only_boundary_gap").is_false()
            || !bins.is_list()
            || (bins.len() as i64) < TOLERANT_MAINT_MIN
            || bins.l().iter().any(|i| !i.is_int() || !(0..360).contains(&i.i()))
        {
            self._first_pass_history.clear();
            self.clear_exit_prepare_window();
            return;
        }
        let list = scan.idx("ranges").l();
        if bins.l().iter().any(|i| !list[i.i() as usize].is_num()) {
            self._first_pass_history.clear();
            self.clear_exit_prepare_window();
            return;
        }
        let mut points: Vec<(f64, f64)> = bins
            .l()
            .iter()
            .map(|i| {
                let i = i.i() as usize;
                let r = list[i].f();
                let a = radians(i as f64);
                (r * a.cos(), -r * a.sin())
            })
            .collect();
        let x = fsum_py(points.iter().map(|p| p.0)) / points.len() as f64;
        if self.continue_route
            && let Some(full) = self.full_target_points(&target, scan, &self.orbit_track_id.clone())
            && !full.is_empty()
        {
            points = full;
        }
        let front = points.iter().map(|p| p.0).fold(f64::NEG_INFINITY, pmax);
        let side = points.iter().map(|p| p.1).fold(INF, pmin) - SIDE_BODY_EXTENT_M;
        let row = dict! {
            "source_seq" => scan.idx("seq").clone(), "source_at_ms" => scan.idx("at_ms").clone(),
            "received_at" => scan.idx("received_at").clone(), "center_x_m" => x,
        };
        if let Some(last) = self._first_pass_history.last()
            && (scan.idx("seq").i() <= last.idx("source_seq").i()
                || scan.idx("at_ms").i() <= last.idx("source_at_ms").i()
                || scan.idx("received_at").f() <= last.idx("received_at").f())
        {
            return;
        }
        self._first_pass_history.push(row.clone());
        if self._first_pass_history.len() > 6 {
            let n = self._first_pass_history.len();
            self._first_pass_history.drain(..n - 6);
        }
        let history = self._first_pass_history.clone();
        let pub_span = (row.idx("source_at_ms").i() - history[0].idx("source_at_ms").i()) as f64 / 1000.;
        let receive_span = row.idx("received_at").f() - history[0].idx("received_at").f();
        let approaching = history.len() >= 3
            && pub_span >= 0.25
            && receive_span >= 0.25
            && history.windows(2).all(|w| w[1].idx("center_x_m").f() < w[0].idx("center_x_m").f());
        let motors: &[i64] = if self.continue_route { &DRIVE_MOTORS } else { &[TRIAL_MOTOR] };
        let adopted = control.get("command_acked").is_true()
            && motor_in(control.get("motor"), motors)
            && eq_i(control.get("servo"), self.servo)
            && NEUTRAL <= self.servo
            && self.servo <= SERVO_MAX;
        let lead = (ceil((self.servo - NEUTRAL) as f64 / LEFT_RELEASE_PWM_STEP as f64) + 1) as f64 * PWM_INTERVAL_S;
        let closing = if approaching {
            let d = history[0].idx("center_x_m").f() - x;
            Some(pmin(d / pub_span, d / receive_span))
        } else {
            None
        };
        let prediction_prepare = adopted
            && approaching
            && x <= FRONT_BODY_EXTENT_M
            && side >= MANEUVER_BODY_CLEARANCE_M
            && front + REAR_BODY_EXTENT_M <= closing.unwrap() * lead;
        let mut geometric_ready = false;
        let eligible = self.continue_route
            && self.handover_observed
            && self.phase == "drive"
            && self.route_stage == "first_target"
            && front <= FRONT_BODY_EXTENT_M
            && side >= MANEUVER_BODY_CLEARANCE_M;
        if eligible {
            let gap = if self.continue_route { SCAN_AGE_S + 0.15 } else { SCAN_AGE_S };
            geometric_ready = consecutive_fresh(&mut self._first_exit_prepare_history, scan, ROUTE_EXIT_PREPARE_FRAMES, gap);
        } else {
            self.clear_exit_prepare_window();
        }
        let geometric_prepare = geometric_ready && adopted;
        if geometric_prepare && !self.first_pass_preparing {
            self.latch_exit_prepare(scan, control, "current_first_target_complete_support_behind_body_front", None);
        }
        let prepare = prediction_prepare || geometric_prepare;
        let mut progress = row.clone();
        progress.update(&dict! {
            "observed_frontmost_x_m" => front,
            "observed_left_body_gap_m" => side, "command_center_lead_s" => lead,
            "relative_x_closing_mps" => closing, "relative_trend_is_vehicle_speed" => false,
            "prepare_condition" => prepare, "prediction_prepare_condition" => prediction_prepare,
            "geometric_exit_prepare_condition" => geometric_prepare,
            "body_rear_x_m" => -REAR_BODY_EXTENT_M,
            "physical_steering_confirmed" => false,
        });
        self.first_pass_progress = progress;
        if prepare {
            self.first_pass_preparing = true;
            self.orbit_left_entry_boost = false;
        }
        if self.first_pass_preparing
            && self.first_pass_evidence.is_none()
            && front <= -REAR_BODY_EXTENT_M
            && side >= MANEUVER_BODY_CLEARANCE_M
        {
            let mut evidence = self.first_pass_progress.clone();
            evidence.update(&dict! {
                "track_id" => target.idx("track_id").clone(), "support_bins" => bins.clone(),
                "basis" => "current_confirmed_target_observed_support_behind_body_rear",
                "semantic_class" => "unknown", "physical_cone_pass_certified" => false,
                "swept_path_certified" => false,
            });
            self.first_pass_evidence = evidence;
            self.right_exit_since = Some(now);
            if self.continue_route {
                self.route_stage = "right_align";
                let e = self.first_pass_evidence.clone();
                self.route_event("first_pass", scan, control, e);
            }
        }
    }

    /// _candidates(ranges, bearing_range_rad=(0, pi), tolerant=True) helper.
    pub fn left_candidates(scan: &V) -> Vec<V> {
        crate::compact_target::candidates(
            scan.idx("ranges"),
            CandidateArgs { bearing_range_rad: (0., PI), tolerant: true, ..CandidateArgs::default() },
        )
    }

    pub fn ranges_floats(scan: &V) -> std::rc::Rc<Vec<Option<f64>>> {
        floats(scan.idx("ranges"))
    }
}
