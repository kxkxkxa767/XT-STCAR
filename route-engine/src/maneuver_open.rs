//! Start lane and the open-course stages (first loop, transfer, second
//! loop, light-lane entry) of ManeuverSequence.

use crate::compact_target::{candidates, CandidateArgs};
use crate::dict;
use crate::helpers::*;
use crate::maneuver::*;
use crate::py::*;

/// An axis-parallel board for _open_lane_target (native wall or raw line).
#[derive(Clone, Debug)]
struct Board {
    lateral: f64,
    along: (f64, f64),
    source: &'static str,
}

fn round3_list(values: &[f64]) -> V {
    V::from(values.iter().map(|v| round_n(*v, 3)).collect::<Vec<f64>>())
}

impl Maneuver {
    /// _decide_entry(scan).
    pub fn decide_entry(&mut self, scan: &V) {
        if !scan.is_dict() {
            return;
        }
        if scan.get("left_turn_goal").is_some() {
            self.entry_kind = Some("junction");
            return;
        }
        let (ok, evidence) = self.lane_start_geometry(scan);
        self.start_lane_feedback = evidence.clone();
        if ok {
            self.entry_kind = Some("lane");
            self.route_stage = "start_lane";
            self._start_lane_width = evidence.idx("width_m").clone();
            self.target_tracker.widen_acquisition(first_acquire_bearing_rad());
            self.target_tracker.extend_acquisition_range(FIRST_APPROACH_MAX_RANGE_M);
        }
    }

    /// _forward_lanes(values, validated).
    pub fn forward_lanes(values: &[V], validated: bool) -> Vec<V> {
        let mut forward: Vec<V> = values
            .iter()
            .filter(|c| (validated || valid_candidate_default(c)) && c.idx("heading_left_rad").f().abs() < PI / 2.)
            .cloned()
            .collect();
        if forward.len() > 1 {
            let base = forward[0].clone();
            if forward[1..].iter().all(|c| {
                wrap(c.idx("heading_left_rad").f() - base.idx("heading_left_rad").f()).abs() <= pair_merge_heading_rad()
                    && (c.idx("center_offset_left_m").f() - base.idx("center_offset_left_m").f()).abs() <= PAIR_MERGE_OFFSET_M
                    && (c.idx("width_m").f() - base.idx("width_m").f()).abs() <= 0.15
            }) {
                let i = argmax_first(&forward, |c| (c.idx("left_wall_points").i() + c.idx("right_wall_points").i()) as f64).unwrap();
                forward = vec![forward[i].clone()];
            }
        }
        forward
    }

    /// start_lane_quality_geometry(scan).
    pub fn start_lane_quality_geometry(&self, scan: &V) -> bool {
        if !self.continue_route
            || self.entry_kind != Some("lane")
            || self.route_stage != "start_lane"
            || self.phase != "drive"
            || self.drive_since.is_none()
            || !scan.is_dict()
        {
            return false;
        }
        let values = scan.get("corridor_candidates");
        let Some(list) = values.ol() else { return false };
        if list.len() > 24 || list.iter().any(|c| !valid_candidate_default(c)) {
            return false;
        }
        Self::forward_lanes(list, false).len() == 1 || self.compass.measurable(scan)
    }

    /// _lane_start_geometry(scan): (ok, evidence).
    fn lane_start_geometry(&self, scan: &V) -> (bool, V) {
        let values = if scan.is_dict() { scan.get("corridor_candidates").clone() } else { V::None };
        let ranges = if scan.is_dict() { scan.get("ranges").clone() } else { V::None };
        if !values.is_list() || values.len() > 24 || !ranges.is_list() || ranges.len() != 360 {
            return (false, dict! {"reason" => "start_lane_invalid_scan"});
        }
        if scan.get("left_turn_goal").is_some() {
            return (false, dict! {"reason" => "start_lane_native_left_opening_present"});
        }
        let forward = Self::forward_lanes(values.l(), false);
        if forward.len() != 1 {
            return (false, dict! {"reason" => "start_lane_unique_lane_missing", "forward_lanes" => forward.len()});
        }
        let lane = &forward[0];
        let list = ranges.l();
        let mut front: Option<V> = None;
        for i in -10..11i64 {
            let r = &list[imod(i, 360) as usize];
            if r.is_num() && front.as_ref().is_none_or(|f| r.f() < f.f()) {
                front = Some(r.clone());
            }
        }
        let evidence = dict! {
            "source_seq" => scan.get("seq").clone(), "source_at_ms" => scan.get("at_ms").clone(),
            "heading_left_rad" => lane.idx("heading_left_rad").clone(),
            "center_offset_left_m" => lane.idx("center_offset_left_m").clone(), "width_m" => lane.idx("width_m").clone(),
            "body_gap_m" => Self::corridor_body_gap(lane), "front_m" => front.clone(),
            "compass_heading_rad" => self.compass.current(scan),
        };
        let with = |reason: Option<&str>| -> V {
            let mut e = evidence.clone();
            e.set("reason", V::from(reason));
            e
        };
        if lane.idx("heading_left_rad").f().abs() > lane_start_max_heading_rad() {
            return (false, with(Some("start_lane_heading_too_large")));
        }
        let width = lane.idx("width_m").f();
        if !(LANE_START_WIDTH_M.0 <= width && width <= LANE_START_WIDTH_M.1) {
            return (false, with(Some("start_lane_width_out_of_range")));
        }
        if evidence.idx("body_gap_m").f() < MANEUVER_BODY_CLEARANCE_M {
            return (false, with(Some("start_lane_body_gap")));
        }
        if front.as_ref().is_none_or(|f| f.f() < LANE_START_MIN_FRONT_M) {
            return (false, with(Some("start_lane_front_blocked")));
        }
        if evidence.idx("compass_heading_rad").is_none() {
            return (false, with(Some("start_lane_compass_unavailable")));
        }
        (true, with(None))
    }

    /// _start_lane_presteer(...).
    fn start_lane_presteer(&mut self, scan: &V, lidar_age_s: &V, now: f64, control: &V, safe: &V, presteer_wait: &V) -> V {
        self.presteer_waiting = false;
        if !(control.is_dict() && control.get("armed").is_true()) {
            self.last_now = now;
            let (ok, evidence) = self.lane_start_geometry(scan);
            self.start_lane_feedback = evidence.clone();
            self.start_ready = ok;
            self.steering_target = NEUTRAL;
            self.reason = if ok { "start_lane_neutral_preview".into() } else { evidence.idx("reason").s().to_string() };
            return self.result(now);
        }
        if now - self.started_at + 1e-9 >= self.max_presteer_s {
            self.lock("left_turn_presteer_timeout");
            return self.result(now);
        }
        let tick = control.get("tick");
        let new_feedback = tick.is_int() && self.last_control_tick.is_none_or(|last| tick.i() > last);
        let advancing = self.orbit_inputs(scan, lidar_age_s, now, control, safe, &V::Bool(false));
        if self.phase == "locked" {
            let at = self.last_now;
            return self.result(at);
        }
        if advancing {
            let (ok, evidence) = self.lane_start_geometry(scan);
            self.start_lane_feedback = evidence;
            self._lane_start_ok = ok;
        }
        let ok = self._lane_start_ok;
        self.steering_target = NEUTRAL;
        if presteer_wait.truthy() || !ok {
            self._lane_start_settle_since = None;
            self._lane_start_ticks = 0;
            self.presteer_waiting = presteer_wait.truthy();
            self.start_ready = false;
            self.reason = if presteer_wait.truthy() {
                "start_lane_presteer_quality_hold".into()
            } else {
                let r = self.start_lane_feedback.get("reason");
                if r.truthy() { r.s().to_string() } else { "start_lane_geometry_missing".into() }
            };
            return self.result(now);
        }
        self.start_ready = true;
        let acked = if control.has("command_acked") { control.get("command_acked").is_true() } else { true };
        let adopted = self.servo == NEUTRAL && eq_i(control.idx("servo"), NEUTRAL) && eq_i(control.idx("motor"), NEUTRAL) && acked;
        if !adopted {
            self._lane_start_settle_since = None;
            self._lane_start_ticks = 0;
            self.reason = "start_lane_waiting_neutral_adoption".into();
            return self.result(now);
        }
        if new_feedback {
            if self._lane_start_settle_since.is_none() {
                self._lane_start_settle_since = Some(now);
            }
            self._lane_start_ticks += 1;
        }
        self.reason = "start_lane_neutral_allowance".into();
        if new_feedback && self._lane_start_ticks >= 3 && now - self._lane_start_settle_since.unwrap() + 1e-9 >= LANE_START_SETTLE_S {
            self.phase = "drive";
            self.drive_since = Some(now);
            self.presteer_ended_at = Some(now);
            self.reason = "start_lane_drive".into();
            let evidence = self.start_lane_feedback.clone();
            self.route_event("start_lane", scan, control, evidence);
        }
        self.result(now)
    }

    /// _start_lane_handover_ready(scan, control).
    pub fn start_lane_handover_ready(&self, scan: &V, control: &V) -> bool {
        let target = &self.compact_target;
        scan.is_dict()
            && !py_eq(scan.get("seq"), &V::from(self.last_seq))
            && self.first_approach.is_some()
            && self.usable_target(target, scan)
            && py_eq(target.idx("track_id"), self.first_approach.get("track_id"))
            && target.idx("range_m").f() <= OPEN_HANDOVER_MAX_RANGE_M
            && target.idx("bearing_left_rad").f() >= first_handover_bearing_rad()
            && control.is_dict()
            && control.get("command_acked").is_true()
            && motor_in(control.get("motor"), &DRIVE_MOTORS)
            && eq_i(control.get("servo"), self.servo)
            && self.servo > NEUTRAL
    }

    /// _start_side_wall(scan, heading).
    fn start_side_wall(&mut self, scan: &V, heading: Option<f64>) -> V {
        let width = self._start_lane_width.clone();
        if !width.is_num() {
            return V::None;
        }
        let width = width.f();
        let floats = Self::ranges_floats(scan);
        let mut fits: Vec<V> = ["right", "left"].iter().map(|side| side_wall_fit(&floats, side)).filter(|f| f.is_some()).collect();
        if let Some(h) = heading {
            fits.retain(|f| (f.idx("slope").f().atan() - h).abs() <= lane_side_wall_heading_rad());
        }
        if fits.is_empty() {
            return V::None;
        }
        let fit = fits[argmax_first(&fits, |f| f.idx("points").i() as f64).unwrap()].clone();
        let intercept = fit.idx("intercept_m").f();
        let centre = if fit.idx("side").s() == "right" { intercept - width / 2. } else { intercept + width / 2. };
        let mut wall = dict! {"source_seq" => scan.idx("seq").clone()};
        wall.update(&fit);
        wall.set("lane_width_m", width);
        self.start_side_wall = wall;
        let slope = fit.idx("slope").f();
        dict! {
            "offset_right_m" => centre, "heading_right_deg" => degrees(slope.atan()),
            "slope" => slope, "width_m" => width, "side_wall" => fit.idx("side").clone(),
        }
    }

    /// lane_wall_output_active().
    pub fn lane_wall_output_active(&self) -> bool {
        self.continue_route
            && self.route_stage == "start_lane"
            && ((self.phase == "drive" && (self.reason == "start_lane_lane_walls" || self.reason == "start_lane_side_wall"))
                || (self.phase == "coast" && self.start_lane_coast_steering.is_some()))
    }

    /// _start_lane_update(...).
    pub fn start_lane_update(&mut self, scan: &V, lidar_age_s: &V, now: f64, control: &V, safe: &V, presteer_wait: &V) -> V {
        if self.phase == "presteer" {
            return self.start_lane_presteer(scan, lidar_age_s, now, control, safe, presteer_wait);
        }
        let advancing = self.orbit_inputs(scan, lidar_age_s, now, control, safe, presteer_wait);
        if self.phase == "locked" {
            let at = self.last_now;
            return self.result(at);
        }
        if self.phase == "drive" && now - self.drive_since.unwrap() + 1e-9 >= self.max_drive_s {
            self.begin_coast("maneuver_cumulative_drive_timeout", now);
        }
        if self.phase == "coast" {
            if self.coast_since.is_some_and(|s| now - s + 1e-9 >= COAST_MAX_S) {
                self.lock("coast_standstill_unconfirmed");
            } else {
                if advancing {
                    let walls = corridor_walls(scan);
                    let mut target = if walls.is_none() {
                        NEUTRAL
                    } else {
                        self._lane_start_steering.update(&walls, scan.idx("seq"), now, true, scan.idx("at_ms"))
                    };
                    if target < NEUTRAL && (self._route_center_ack.is_none() || self.servo > NEUTRAL) {
                        target = NEUTRAL;
                    }
                    self.steering_target = target;
                    self.start_lane_coast_steering = dict! {
                        "source_seq" => scan.idx("seq").clone(), "walls" => walls,
                        "target_pwm" => target, "reason" => self.reason.as_str(),
                    };
                }
                self.slew(now);
            }
            return self.result(now);
        }
        if self.phase != "drive" {
            return self.result(now);
        }
        if !advancing {
            self.slew(now);
            return self.result(now);
        }
        if !self.route_adopted(control, now) {
            return self.result(now);
        }
        let heading = self.compass.current(scan);
        let approach = self.open_first_approach(scan);
        let held = self.target_tracker.held.clone();
        let mut walls = V::None;
        let (natural, source): (i64, &str);
        if approach.is_some() {
            self.first_approach = approach.clone();
            self._lane_start_slow = true;
            natural = approach.idx("target_pwm").i();
            source = "first_support_approach";
        } else if self.first_approach.is_some() && held.is_dict() && py_eq(held.get("track_id"), self.first_approach.idx("track_id")) {
            natural = self.servo;
            source = "first_support_missing_frame_hold";
        } else {
            walls = corridor_walls(scan);
            if walls.is_some() {
                self._lane_start_steering.servo = clampi(self.servo, NEUTRAL - LANE_START_HOLD_PWM, NEUTRAL + LANE_START_HOLD_PWM);
                natural = self._lane_start_steering.update(&walls, scan.idx("seq"), now, true, scan.idx("at_ms"));
                source = "lane_walls";
            } else {
                walls = self.start_side_wall(scan, heading);
                if walls.is_some() {
                    self._lane_start_steering.servo = clampi(self.servo, NEUTRAL - LANE_START_HOLD_PWM, NEUTRAL + LANE_START_HOLD_PWM);
                    natural = self._lane_start_steering.update(&walls, scan.idx("seq"), now, true, scan.idx("at_ms"));
                    source = "side_wall";
                } else if let Some(h) = heading {
                    natural = NEUTRAL + clampi(round(LEFT_TRIAL_GAIN * (-h)), -LANE_START_HOLD_PWM, LANE_START_HOLD_PWM);
                    source = "compass_heading_hold";
                } else {
                    natural = NEUTRAL;
                    source = "no_geometry_center";
                }
            }
        }
        let mut target = natural;
        if target < NEUTRAL && (self._route_center_ack.is_none() || self.servo > NEUTRAL) {
            target = NEUTRAL;
        }
        let ahead = self.lane_front_range(scan);
        let track = self.lane_front_track(scan, ahead);
        let speed = self.track_speed(scan, track);
        let mut stop_at = None;
        let mut allowed = None;
        if approach.is_none() && source != "first_support_missing_frame_hold" {
            let reaction = self.reaction_s(now);
            let rise = if self._power_cmd_motor == Some(TRIAL_MOTOR) { LANE_POWER_ACCEL_MPS2 * (reaction - LANE_REACTION_BASE_S) } else { 0. };
            let v = speed.unwrap_or(LANE_SPEED_UNKNOWN_MPS) + rise;
            stop_at = Some(FRONT_BODY_EXTENT_M + LANE_FINISH_GAP_M + v * v / (2. * LANE_START_STOP_DECEL_MPS2) + v * reaction);
            allowed = Self::start_allowed_speed(ahead);
            if speed.is_some() {
                self._start_speed_measured = true;
            }
            self._start_motor = Self::start_governor(speed, rise, allowed, self._start_speed_measured);
        } else {
            self._start_motor =
                if speed.is_some_and(|s| s < LANE_SPEED_RESTART_MPS) { TRIAL_MOTOR } else { ROUTE_SLOW_MOTOR };
        }
        self.start_lane_feedback = dict! {
            "source_seq" => scan.idx("seq").clone(), "steering_source" => source,
            "nominal_target_pwm" => natural, "walls" => walls, "compass_heading_rad" => heading,
            "first_approach" => approach.clone(), "front_range_m" => ahead, "speed_mps" => speed,
            "front_stop_range_m" => stop_at, "allowed_speed_mps" => allowed, "slow" => self._lane_start_slow,
            "motor" => self._start_motor, "swept_path_certified" => false,
        };
        if let (Some(stop), Some(a)) = (stop_at, ahead)
            && a <= stop
        {
            self.begin_coast("start_lane_board_ahead_without_first_support", now);
            self.slew(now);
            return self.result(now);
        }
        self.steering_target = target;
        self.natural_steering_target = Some(natural);
        self.reason = format!("start_lane_{source}");
        self.slew(now);
        if self.servo > NEUTRAL {
            self._route_center_ack = V::None;
        }
        self.result(now)
    }

    // ------------------------------------------------------------ open-course loops

    /// _open_loop_geometry(scan, target, identity, side, axis, pose).
    pub fn open_loop_geometry(&mut self, scan: &V, target: &V, identity: &V, side: i64, axis: f64, pose: Option<(f64, f64, f64, f64)>) -> V {
        let Some(heading) = self.compass.current(scan) else { return V::None };
        if self.full_target_points(target, scan, identity).is_none() {
            return V::None;
        }
        let rate = self.compass.rate;
        let floats = Self::ranges_floats(scan);
        let bins = bins_of(target.idx("support_bins"));
        let (center, radius0) = support_center(&floats, &bins, target.get("diameter_m"));
        let sidef = side as f64;
        let (e1, e2) = axis_frame(axis, heading, sidef);
        let points = scan_points(&floats, hypot(center.0, center.1) + OPEN_PROBE_REACH_M);
        let directions = open_probe_directions_rad();
        let slot = self.slot_mut(side);
        let radius = pmax(radius0, slot.radius.unwrap_or(0.));
        slot.radius = Some(radius);
        let skip = radius + OPEN_SUPPORT_SKIP_M;
        let mut changed = false;
        for &theta in &directions {
            let known = slot.free_get(theta);
            if known.is_some() && rate.abs() > OPEN_PROBE_MAX_RATE_RAD_S {
                continue;
            }
            let (c, s) = (theta.pcos(), theta.psin());
            let mut free = probe_free(&points, center, (c * e1.0 + s * e2.0, c * e1.1 + s * e2.1), skip);
            if free != INF {
                free = floor(free / OPEN_PROBE_QUANTUM_M) as f64 * OPEN_PROBE_QUANTUM_M;
            }
            if known.is_none_or(|k| free < k) {
                slot.free_set(theta, free);
                changed = true;
            }
        }
        let key = (identity.clone(), round_n(radius, 2));
        let approach = (-(center.0 * e1.0 + center.1 * e1.1), -(center.0 * e2.0 + center.1 * e2.1));
        let mut plan_update = V::None;
        let mut sorted_free = slot.free.clone();
        sorted_free.sort_by(|a, b| OrdF(a.0).cmp(&OrdF(b.0)).then(OrdF(a.1).cmp(&OrdF(b.1))));
        let plan_inputs = V::list(vec![
            V::list(vec![key.0.clone(), V::Float(key.1)]),
            V::list(sorted_free.iter().map(|(t, v)| V::from(vec![*t, *v])).collect()),
            V::Float(round_n(approach.0, 2)),
            V::Float(round_n(approach.1, 2)),
        ]);
        let same_inputs = slot.plan.is_some() && slot.plan_inputs.as_ref().is_some_and(|p| py_eq(p, &plan_inputs));
        let key_differs = match &slot.key {
            None => true,
            Some((i, r)) => !(py_eq(i, &key.0) && *r == key.1),
        };
        if same_inputs {
            let _ = changed;
        } else if changed || slot.plan.is_none() || key_differs || slot.approaching.unwrap_or(true) {
            let free_list: Vec<(f64, f64)> = directions.iter().map(|&t| (t, slot.free_get(t).unwrap_or(INF))).collect();
            let radii = if side > 0 { OPEN_LEFT_RADIUS_M } else { OPEN_RIGHT_RADIUS_M };
            let candidate = plan_loop(&free_list, radius, radii, Some(approach));
            let old_plan = slot.plan.clone();
            let mut reject_reversal = false;
            let mut update = dict! {
                "checked_source_seq" => scan.idx("seq").clone(),
                "constraint_basis" => "existing_low_yaw_probe_history_running_minimum",
                "prior_plan_margins_m" => V::None, "candidate_curvature_per_m" => V::None,
                "prior_curvature_per_m" => V::None, "rejected_task_direction_reversal" => false,
                "swept_path_certified" => false,
            };
            let same_identity = slot.key.as_ref().is_some_and(|(i, _)| py_eq(i, identity));
            if old_plan.is_some() && slot.key.is_some() && same_identity && !slot.approaching.unwrap_or(true) {
                let old_r = old_plan.idx("radius_m").f();
                let (old_a, old_b) = (old_plan.idx("offset_axis_m").f(), old_plan.idx("offset_side_m").f());
                let inner = radius + SIDE_BODY_EXTENT_M + OPEN_INNER_GAP_M;
                let extents: Vec<(f64, f64)> = directions.iter().map(|&t| (t, loop_extent(old_a, old_b, old_r, t))).collect();
                let enclosing = old_r - hypot(old_a, old_b) - inner;
                let min_inner = extents.iter().map(|(_, e)| e - inner).fold(INF, pmin);
                let min_outer = extents
                    .iter()
                    .map(|(t, e)| slot.free_get(*t).unwrap_or(INF) - SIDE_BODY_EXTENT_M - OPEN_OUTER_GAP_M - e)
                    .fold(INF, pmin);
                let fin = |v: f64| if v.is_finite() { V::Float(v) } else { V::None };
                update.set(
                    "prior_plan_margins_m",
                    dict! {
                        "enclosing_support_margin_m" => fin(enclosing),
                        "minimum_inner_probe_margin_m" => fin(min_inner),
                        "minimum_outer_probe_margin_m" => fin(min_outer),
                    },
                );
                let old_feasible = old_r - hypot(old_a, old_b) >= inner
                    && extents.iter().all(|(t, e)| {
                        inner <= *e && *e <= slot.free_get(*t).unwrap_or(INF) - SIDE_BODY_EXTENT_M - OPEN_OUTER_GAP_M
                    });
                if old_feasible {
                    let plan_carrot = |value: &V| -> (f64, f64) {
                        let (pa, pb) = (value.idx("offset_axis_m").f(), value.idx("offset_side_m").f());
                        let pc = (center.0 + pa * e1.0 + pb * e2.0, center.1 + pa * e1.1 + pb * e2.1);
                        loop_carrot(
                            pc,
                            value.idx("radius_m").f(),
                            sidef,
                            OPEN_LOOKAHEAD_M,
                            e1,
                            e2,
                            Some((center, radius + SIDE_BODY_EXTENT_M + OPEN_CARROT_SUPPORT_GAP_M)),
                        )
                        .0
                    };
                    let (candidate_carrot, prior_carrot) = (plan_carrot(&candidate), plan_carrot(&old_plan));
                    reject_reversal = sidef * candidate_carrot.1 < 0. && 0. <= sidef * prior_carrot.1;
                    update.set("candidate_curvature_per_m", pursuit_pwm(candidate_carrot).0);
                    update.set("prior_curvature_per_m", pursuit_pwm(prior_carrot).0);
                }
            }
            update.set("rejected_task_direction_reversal", reject_reversal);
            plan_update = update;
            if !reject_reversal {
                slot.plan = candidate;
                slot.key = Some(key.clone());
                slot.plan_source_seq = Some(scan.idx("seq").i());
                slot.plan_inputs = Some(plan_inputs);
            }
        }
        let plan = slot.plan.clone();
        let (a, b) = (plan.idx("offset_axis_m").f(), plan.idx("offset_side_m").f());
        let loop_center = (center.0 + a * e1.0 + b * e2.0, center.1 + a * e1.1 + b * e2.1);
        let support = Some((center, radius + SIDE_BODY_EXTENT_M + OPEN_CARROT_SUPPORT_GAP_M));
        let plan_radius = plan.idx("radius_m").f();
        let (carrot, mode, before) = loop_carrot(loop_center, plan_radius, sidef, OPEN_LOOKAHEAD_M, e1, e2, support);
        slot.approaching = Some(before > 0.);
        let free_m = {
            let mut d = V::dict();
            for (t, v) in slot.free.iter() {
                d.set(&round(degrees(*t)).to_string(), if *v == INF { V::None } else { V::Float(*v) });
            }
            d
        };
        let plan_source_seq = slot.plan_source_seq;
        let (mut curvature, mut pwm) = pursuit_pwm(carrot);
        let current_pose_pwm = pwm;
        let mut command = None;
        if let Some(pose) = pose
            && mode == "approach_line"
        {
            let cmd = loop_carrot(
                in_pose(loop_center, pose),
                plan_radius,
                sidef,
                OPEN_LOOKAHEAD_M,
                turned(e1, pose.2),
                turned(e2, pose.2),
                Some((in_pose(center, pose), radius + SIDE_BODY_EXTENT_M + OPEN_CARROT_SUPPORT_GAP_M)),
            )
            .0;
            command = Some(cmd);
            let guarded = overshoot_guard(pwm, pursuit_pwm(cmd).1);
            if guarded != pwm {
                (curvature, pwm) = pursuit_pwm(cmd);
            }
        }
        let plan_out = {
            let mut d = V::dict();
            for (k, v) in plan.items() {
                let nonfinite = matches!(v, V::Float(x) if !x.is_finite());
                d.set(k, if nonfinite { V::None } else { v.clone() });
            }
            d
        };
        dict! {
            "source_seq" => scan.idx("seq").clone(), "track_id" => identity.clone(),
            "direction" => if side > 0 { "anticlockwise" } else { "clockwise" }, "axis_rad" => axis,
            "compass_heading_rad" => heading, "progress_rad" => sidef * (heading - axis),
            "support_center_left_m" => round3_list(&[center.0, center.1]), "support_radius_m" => round_n(radius, 3),
            "free_m" => free_m,
            "plan" => plan_out,
            "plan_metrics_basis" => "selection_time_optimizer_score_and_probe_margins",
            "plan_selection_source_seq" => plan_source_seq, "plan_update" => plan_update,
            "loop_center_left_m" => round3_list(&[loop_center.0, loop_center.1]), "pursuit_mode" => mode,
            "before_tangent_point_m" => round_n(before, 3), "carrot_left_m" => round3_list(&[carrot.0, carrot.1]),
            "response_pose" => pose.map(|p| round3_list(&[p.0, p.1, p.2, p.3])),
            "response_carrot_left_m" => command.map(|c| round3_list(&[c.0, c.1])),
            "pursuit_curvature_per_m" => curvature, "target_pwm" => pwm, "current_pose_pwm" => current_pose_pwm,
            "basis" => "support_loop_planned_from_probed_free_space", "swept_path_certified" => false,
        }
    }

    /// _tracking_heading(scan).
    pub fn tracking_heading(&self, scan: &V) -> Option<f64> {
        if self.entry_kind == Some("lane") { self.compass.current(scan) } else { None }
    }

    /// _open_target_usable(target, scan, identity, side).
    fn open_target_usable(target: &V, scan: &V, identity: &V, side: i64) -> bool {
        target.is_dict()
            && target.get("confirmed").is_true()
            && py_eq(target.get("track_id"), identity)
            && identity.is_some()
            && target.get("kind").os() == Some("lidar_compact_object")
            && target.get("source_seq").is_int()
            && py_eq(target.idx("source_seq"), scan.get("seq"))
            && target.get("source_at_ms").is_int()
            && py_eq(target.idx("source_at_ms"), scan.get("at_ms"))
            && target.get("bearing_left_rad").is_num()
            && {
                let v = side as f64 * target.idx("bearing_left_rad").f();
                0. <= v && v <= PI
            }
    }

    /// _open_reset_loop(side).
    pub fn open_reset_loop(&mut self, side: Option<i64>) {
        match side {
            None => self._open_slots.clear(),
            Some(s) => self._open_slots.retain(|(k, _)| *k != s),
        }
        self._open_lost_since = None;
    }

    /// _open_first_approach(scan).
    fn open_first_approach(&mut self, scan: &V) -> V {
        let target = self.compact_target.clone();
        if !target.is_dict()
            || !target.get("confirmed").is_true()
            || target.get("track_id").is_none()
            || !target.get("range_m").is_num()
            || !target.get("bearing_left_rad").is_num()
            || !(0. < target.idx("bearing_left_rad").f() && target.idx("bearing_left_rad").f() <= first_acquire_bearing_rad().1)
            || target.idx("range_m").f() > FIRST_APPROACH_MAX_RANGE_M
        {
            return V::None;
        }
        if let Some(heading) = self.compass.current(scan) {
            let (e1, _) = axis_frame(0., heading, 1.);
            let p = target.idx("point_left_m").l();
            let (x, y) = (p[0].f(), p[1].f());
            if e1.0 * x + e1.1 * y < OPEN_FIRST_MIN_ALONG_M {
                return V::None;
            }
            let lines = self.scan_axis_lines(scan, heading);
            let floats = Self::ranges_floats(scan);
            let bins = get_or(&target, "support_bins", &V::list(vec![])).clone();
            if on_board_run((x, y), &lines, heading) || support_depth(&floats, &ints_of(&bins)) > OPEN_SUPPORT_MAX_DEPTH_M {
                return V::None;
            }
        }
        if self.first_approach.is_some() && !py_eq(self.first_approach.get("track_id"), target.idx("track_id")) {
            self.open_reset_loop(Some(1));
        }
        let id = target.idx("track_id").clone();
        let lp = self.open_loop_geometry(scan, &target, &id, 1, 0., None);
        if lp.is_none() {
            return V::None;
        }
        let mut clear = true;
        if self.first_approach.is_none() {
            let c = lp.idx("carrot_left_m").l();
            let (cx, cy) = (c[0].f(), c[1].f());
            let h = hypot(cx, cy);
            let length = if h != 0. { h } else { 1. };
            let end = (OPEN_APPROACH_CHECK_M * cx / length, OPEN_APPROACH_CHECK_M * cy / length);
            let floats = Self::ranges_floats(scan);
            let sc = lp.idx("support_center_left_m").l();
            clear = strip_clear(
                &scan_points(&floats, OPEN_APPROACH_CHECK_M + 1.),
                end,
                Some((sc[0].f(), sc[1].f())),
                lp.idx("support_radius_m").f() + OPEN_SUPPORT_SKIP_M,
            );
        }
        self.open_loop = lp.clone();
        let mut approach = lp;
        approach.update(&dict! {
            "range_m" => target.idx("range_m").clone(), "bearing_left_rad" => target.idx("bearing_left_rad").clone(),
            "approach_strip_clear" => clear,
            "basis" => "first_support_left_of_car_anticlockwise_rule_loop_plan",
        });
        if clear { approach } else { V::None }
    }

    /// _open_second_maturing(scan, ahead, stop_at).
    pub fn open_second_maturing(&self, scan: &V, ahead: Option<f64>, stop_at: f64) -> bool {
        let target = &self.second_target;
        ahead.is_some_and(|a| a > stop_at - OPEN_SECOND_CONFIRM_GRACE_M)
            && target.is_dict()
            && target.get("confirmed").is_false()
            && target.get("track_id").is_int()
            && py_eq(target.get("source_seq"), scan.get("seq"))
            && target.get("range_m").is_num()
            && target.idx("range_m").f() < ahead.unwrap()
            && self.role_distinct_evidence.is_some()
            && py_eq(self.role_distinct_evidence.get("source_seq"), scan.get("seq"))
    }

    /// _open_confirmed_second(scan).
    pub fn open_confirmed_second(&self, scan: &V) -> V {
        let target = &self.second_target;
        if !target.is_dict() || !target.get("confirmed").is_true() {
            return V::None;
        }
        let identity = if self.second_track_id.is_some() { self.second_track_id.clone() } else { target.get("track_id").clone() };
        if self.full_target_points(target, scan, &identity).is_some() { target.clone() } else { V::None }
    }

    /// _open_first_loop_update(scan, now, control).
    pub fn open_first_loop_update(&mut self, scan: &V, now: f64, control: &V) -> V {
        let heading = self.compass.current(scan);
        let progress = heading;
        if let Some(p) = progress
            && p >= OPEN_SECOND_LOOKOUT_RAD
            && self.second_track_id.is_none()
        {
            self.open_second_observation(scan, now, heading.unwrap());
        }
        if progress.is_some_and(|p| p >= PI - open_exit_margin_rad()) {
            return self.open_first_pass(scan, now, control, "compass_heading_reversed_in_loop");
        }
        let target = self.compact_target.clone();
        let held = self.target_tracker.held.clone();
        if target.is_none() && held.is_dict() && py_eq(held.get("track_id"), &self.orbit_track_id) && progress.is_some() {
            self.steering_target = self.servo;
            self.natural_steering_target = Some(self.servo);
            self.reason = "open_first_loop_missing_frame_hold".into();
            self.slew(now);
            return self.result(now);
        }
        if !Self::open_target_usable(&target, scan, &self.orbit_track_id, 1) {
            if progress.is_some_and(|p| p >= PI - open_late_exit_rad()) {
                return self.open_first_pass(scan, now, control, "support_lost_near_heading_reversal");
            }
            self.capture_compact_loss(scan, now, control, "compact_target_lost_or_ambiguous");
            if self.begin_quality_wait(&V::Float(now), control, Some(scan)) {
                self.reason = "route_orbit_target_recovery_wait".into();
            } else if !self.begin_adopted_left_coast("compact_target_lost_or_ambiguous", now, control, true) {
                self.begin_coast("compact_target_lost_or_ambiguous", now);
            }
            self.slew(now);
            return self.result(now);
        }
        let id = self.orbit_track_id.clone();
        let lp = self.open_loop_geometry(scan, &target, &id, 1, 0., None);
        if lp.is_none() {
            self.steering_target = self.servo;
            self.natural_steering_target = Some(self.servo);
            self.reason = "open_first_loop_geometry_hold".into();
            self.slew(now);
            return self.result(now);
        }
        self.open_loop = lp.clone();
        let second = self.open_confirmed_second(scan);
        if second.is_some() {
            let p = progress.unwrap_or_else(|| type_error("'>=' not supported between instances of 'NoneType' and 'float'"));
            if p >= OPEN_HANDOFF_MIN_PROGRESS_RAD {
                let sid = second.idx("track_id").clone();
                let following = self.open_loop_geometry(scan, &second, &sid, -1, PI, None);
                self.open_next_loop = following.clone();
                if following.is_some() && following.idx("current_pose_pwm").i() <= lp.idx("current_pose_pwm").i() {
                    return self.open_first_pass(scan, now, control, "second_loop_needs_less_left");
                }
            }
        }
        let t = lp.idx("target_pwm").i();
        self.natural_steering_target = Some(t);
        self.steering_target = NEUTRAL.max(t);
        self.reason = "open_first_loop".into();
        self.slew(now);
        self.result(now)
    }

    /// _open_first_pass(scan, now, control, basis).
    fn open_first_pass(&mut self, scan: &V, now: f64, control: &V, basis: &str) -> V {
        self.first_pass_evidence = dict! {
            "source_seq" => scan.idx("seq").clone(), "source_at_ms" => scan.idx("at_ms").clone(),
            "received_at" => scan.idx("received_at").clone(), "track_id" => self.orbit_track_id.clone(), "basis" => basis,
            "compass" => self.compass.evidence.clone(), "loop" => self.open_loop.clone(),
            "next_loop" => self.open_next_loop.clone(),
            "semantic_class" => "unknown", "physical_cone_pass_certified" => false, "swept_path_certified" => false,
        };
        self.right_exit_since = Some(now);
        self.open_reset_loop(Some(1));
        let e = self.first_pass_evidence.clone();
        self.route_event("first_pass", scan, control, e);
        if self.open_confirmed_second(scan).is_some() {
            self.open_begin_second_loop(scan, now, control);
            return self.open_second_loop_update(scan, now, control, true);
        }
        self.route_stage = "open_transfer";
        self.steering_target = self.servo;
        self.natural_steering_target = Some(self.servo);
        self.reason = "open_first_pass_keep_command".into();
        self.result(now)
    }

    /// _open_begin_second_loop(scan, now, control).
    fn open_begin_second_loop(&mut self, scan: &V, now: f64, control: &V) {
        let target = self.second_target.clone();
        self.second_track_id = target.idx("track_id").clone();
        let id = self.second_track_id.clone();
        self.second_tracker_mut().enable_close_maintenance(&id);
        self._second_role_anchor = self.second_prior(&target, scan);
        self._second_recovering = false;
        self.second_target_handover_observed = true;
        self.second_orbit_since = Some(now);
        self.route_stage = "second_loop";
        self._open_lost_since = None;
        self.open_next_loop = V::None;
        let e = self.role_distinct_evidence.clone();
        self.route_event("second_handover", scan, control, e);
    }

    /// _open_heading_hold(heading, axis).
    pub fn open_heading_hold(heading: f64, axis: f64) -> i64 {
        let error = wrap(axis - heading);
        NEUTRAL + clampi(round(LEFT_TRIAL_GAIN * error), -OPEN_HEADING_HOLD_PWM, OPEN_HEADING_HOLD_PWM)
    }

    /// _open_right_gate(target).
    fn open_right_gate(&self, target: i64) -> i64 {
        if target < NEUTRAL && (self._route_center_ack.is_none() || self.servo > NEUTRAL) { NEUTRAL } else { target }
    }

    /// _open_front_stop(scan): (ahead, speed, stop_at).
    pub fn open_front_stop(&mut self, scan: &V) -> (Option<f64>, Option<f64>, f64) {
        let ahead = self.lane_front_range(scan);
        let speed = self.lane_speed(scan, ahead, 2);
        let v = LANE_SPEED_UNKNOWN_MPS;
        let stop_at = FRONT_BODY_EXTENT_M + LANE_FINISH_GAP_M + v * v / (2. * LANE_START_STOP_DECEL_MPS2) + v * LANE_REACTION_S;
        (ahead, speed, stop_at)
    }

    /// _open_first_center(scan).
    fn open_first_center(&self, scan: &V) -> V {
        let target = &self.compact_target;
        if target.is_dict()
            && py_eq(target.get("track_id"), &self.orbit_track_id)
            && py_eq(target.get("source_seq"), scan.get("seq"))
            && target.get("point_left_m").is_list()
        {
            return target.idx("point_left_m").clone();
        }
        let held = &self.target_tracker.held;
        if held.is_dict() && py_eq(held.get("track_id"), &self.orbit_track_id) {
            return held.get("point_left_m").clone();
        }
        V::None
    }

    /// _open_second_observation(scan, now, heading).
    pub fn open_second_observation(&mut self, scan: &V, now: f64, heading: f64) {
        let (e1, _) = axis_frame(PI, heading, 1.);
        let first = self.full_target_points(&self.compact_target.clone(), scan, &self.orbit_track_id.clone());
        let first_center = self.open_first_center(scan);
        let first_along = match first_center.ol() {
            Some(c) if c.len() == 2 => Some(e1.0 * c[0].f() + e1.1 * c[1].f()),
            _ => None,
        };
        let mut ahead: Vec<(V, f64, f64)> = Vec::new();
        let mut boards: Option<std::rc::Rc<Vec<Line>>> = None;
        let floats = Self::ranges_floats(scan);
        for target in candidates(
            scan.idx("ranges"),
            CandidateArgs {
                bearing_range_rad: (-PI / 2., PI / 2.),
                tolerant: true,
                min_points: Some(SECOND_ACQUISITION_MIN_POINTS),
                max_range_m: crate::compact_target::EXTENDED_MAX_RANGE_M,
                ..CandidateArgs::default()
            },
        ) {
            let p = target.idx("point_left_m").l();
            let (x, y) = (p[0].f(), p[1].f());
            let (along, lateral) = (e1.0 * x + e1.1 * y, e1.0 * y - e1.1 * x);
            if !(x > FRONT_BODY_EXTENT_M
                && FRONT_BODY_EXTENT_M < along
                && along <= OPEN_SECOND_BAND_REACH_M
                && lateral.abs() <= OPEN_SECOND_BAND_HALF_M
                && first_along.is_none_or(|fa| along >= fa + OPEN_SECOND_BEYOND_FIRST_M))
            {
                continue;
            }
            if boards.is_none() {
                boards = Some(self.scan_axis_lines(scan, heading));
            }
            if on_board_run((x, y), boards.as_ref().unwrap(), heading)
                || support_depth(&floats, &ints_of(target.idx("support_bins"))) > OPEN_SUPPORT_MAX_DEPTH_M
            {
                continue;
            }
            ahead.push((target, along, lateral));
        }
        let mut proof = V::None;
        if ahead.len() == 1 {
            let (target, along, lateral) = ahead[0].clone();
            let bins = ints_of(target.idx("support_bins"));
            let points: Vec<(f64, f64)> = bins
                .iter()
                .map(|&i| {
                    let r = floats[i as usize].unwrap();
                    let a = radians(i as f64);
                    (r * a.pcos(), -r * a.psin())
                })
                .collect();
            let separation = first.as_ref().map(|f| {
                let mut m = INF;
                for a in f {
                    for b in &points {
                        m = pmin(m, dist(*a, *b));
                    }
                }
                m
            });
            let distinct = match &first {
                None => true,
                Some(_) => {
                    let first_bins = ints_of(self.compact_target.idx("support_bins"));
                    !bins.iter().any(|b| first_bins.contains(b)) && separation.unwrap() > MAX_DIAMETER_M
                }
            };
            if distinct {
                proof = dict! {
                    "source_seq" => scan.idx("seq").clone(), "source_at_ms" => scan.idx("at_ms").clone(),
                    "source_received_at" => scan.idx("received_at").clone(), "first_track_id" => self.orbit_track_id.clone(),
                    "first_support_visible" => first.is_some(), "second_support_bins" => V::from(bins.clone()),
                    "actual_support_separation_m" => separation, "compass_heading_rad" => heading,
                    "second_along_axis_m" => along, "second_lateral_m" => lateral,
                    "first_along_axis_m" => first_along, "route_stage" => self.route_stage,
                    "basis" => "single_support_ahead_in_band_along_reversed_axis_beyond_first",
                    "physical_identity_verified" => false,
                };
            }
        }
        if proof.is_none() && self._second_proof_at.is_some_and(|at| (0. ..SCAN_AGE_S).contains(&(now - at))) {
            self.second_target = V::None;
            self.second_acquisition_reason = Some("second_role_proof_gap_held".into());
            return;
        }
        if proof.is_none() {
            self._second_proof_at = None;
            self.second_tracker = Some(new_second_tracker());
            self.second_target = V::None;
            self.role_distinct_evidence = V::None;
            self.second_acquisition_reason = Some("second_role_band_support_missing".into());
            return;
        }
        self._second_proof_at = Some(now);
        let mut tracked = scan.clone();
        if self.first_pass_evidence.is_none() {
            if first.is_none() {
                self.second_acquisition_reason = Some("second_lookout_first_support_not_current".into());
                return;
            }
            let mut hidden: Vec<i64> = ints_of(self.compact_target.idx("support_bins"));
            let unknown = self.compact_target.get("boundary_unknown_bins");
            if unknown.truthy() {
                hidden.extend(ints_of(unknown));
            }
            let list = scan.idx("ranges").l();
            let masked: Vec<V> =
                list.iter().enumerate().map(|(i, r)| if hidden.contains(&(i as i64)) { V::None } else { r.clone() }).collect();
            tracked.set("ranges", V::list(masked));
            self._open_lookout_seq = Some(scan.idx("seq").i());
        }
        let target = self.second_tracker_mut().update(&tracked, now, true, Some(heading), None);
        self.second_target = target;
        self.role_distinct_evidence = proof;
        self.second_acquisition_reason = Some(self.second_tracker.as_ref().unwrap().reason.clone());
    }

    /// _open_channel_target(scan, heading, pose).
    fn open_channel_target(&mut self, scan: &V, heading: f64, pose: Option<(f64, f64, f64, f64)>) -> V {
        let reverse = heading - PI;
        let lines: Vec<Line> = self
            .scan_axis_lines(scan, reverse)
            .iter()
            .filter(|l| SIDE_BODY_EXTENT_M < l.lateral_m && l.lateral_m <= OPEN_CHANNEL_MAX_LATERAL_M && l.along_m.0 > FRONT_BODY_EXTENT_M)
            .cloned()
            .collect();
        if lines.is_empty() {
            return V::None;
        }
        let divider = lines[argmin_first(&lines, |l| l.lateral_m).unwrap()].clone();
        let width = if self._start_lane_width.is_num() { self._start_lane_width.f() } else { 2. * OPEN_CHANNEL_OFFSET_M.0 };
        let offset = pmax(OPEN_CHANNEL_OFFSET_M.0, pmin(OPEN_CHANNEL_OFFSET_M.1, width / 2.));
        let lateral = divider.lateral_m - offset;
        let u = ((-reverse).pcos(), (-reverse).psin());
        let channel_point = |moved_along: f64, moved_lateral: f64| -> (f64, f64) {
            let a = pmax(
                pmax(OPEN_LANE_LOOKAHEAD_M.0, pmin(OPEN_LANE_LOOKAHEAD_M.1, divider.along_m.0 - moved_along)),
                OPEN_CHANNEL_SLOPE * (lateral - moved_lateral).abs(),
            ) + moved_along;
            (a * u.0 - lateral * u.1, a * u.1 + lateral * u.0)
        };
        let point = channel_point(0., 0.);
        let (curvature, pwm) = guarded_pursuit(point, &channel_point, pose, u);
        dict! {
            "divider_lateral_m" => divider.lateral_m, "divider_open_end_along_m" => divider.along_m.0,
            "channel_offset_m" => offset, "pursuit_point_left_m" => round3_list(&[point.0, point.1]),
            "response_pose" => pose.map(|p| round3_list(&[p.0, p.1, p.2, p.3])),
            "pursuit_curvature_per_m" => curvature, "target_pwm" => pwm,
        }
    }

    /// _response_pose().
    pub fn response_pose(&self) -> (f64, f64, f64, f64) {
        let rate = if self.compass.rate.is_finite() { self.compass.rate } else { 0. };
        let horizon = OPEN_STEER_LAG_S + (self.servo - NEUTRAL).abs() as f64 / OPEN_RELEASE_PWM_PER_S / 2.;
        let turn = rate * horizon;
        let distance = OPEN_PREDICT_SPEED_MPS * horizon;
        if turn.abs() < 1e-6 {
            return (distance, 0., 0., horizon);
        }
        let radius = distance / turn;
        (radius * turn.psin(), radius * (1. - turn.pcos()), turn, horizon)
    }

    /// _open_transfer_update(scan, now, control).
    pub fn open_transfer_update(&mut self, scan: &V, now: f64, control: &V) -> V {
        if !self.route_adopted(control, now) {
            return self.result(now);
        }
        let Some(heading) = self.compass.current(scan) else {
            self.begin_recovery_hold("open_compass_unavailable", now, control);
            return self.result(now);
        };
        self.open_second_observation(scan, now, heading);
        if self.open_confirmed_second(scan).is_some() {
            self.open_begin_second_loop(scan, now, control);
            self._second_plan_pending = Some((scan.idx("seq").clone(), scan.idx("at_ms").clone()));
            self.reason = "open_second_loop_plan_next_tick".into();
            return self.result(now);
        }
        let pose = self.response_pose();
        let channel = self.open_channel_target(scan, heading, Some(pose));
        let natural = if channel.is_some() {
            channel.idx("target_pwm").i()
        } else {
            overshoot_guard(Self::open_heading_hold(heading, PI), Self::open_heading_hold(heading + pose.2, PI))
        };
        let (ahead, speed, stop_at) = self.open_front_stop(scan);
        self.open_feedback = dict! {
            "source_seq" => scan.idx("seq").clone(), "stage" => "open_transfer", "compass_heading_rad" => heading,
            "channel" => channel.clone(), "nominal_target_pwm" => natural, "front_range_m" => ahead, "speed_mps" => speed,
            "front_stop_range_m" => stop_at, "second_acquisition_reason" => self.second_acquisition_reason.clone(),
        };
        let maturing = self.open_second_maturing(scan, ahead, stop_at);
        self.open_feedback.set("second_confirm_grace", maturing);
        if ahead.is_some_and(|a| a <= stop_at) && !maturing {
            self.begin_recovery_hold("open_board_ahead_without_second_support", now, control);
            return self.result(now);
        }
        self.natural_steering_target = Some(natural);
        self.steering_target = self.open_right_gate(natural);
        self.reason = if channel.is_some() { "open_transfer_channel_entry".into() } else { "open_transfer_reversed_axis_hold".into() };
        self.slew(now);
        self.result(now)
    }

    /// _open_second_loop_update(scan, now, control, adopted).
    pub fn open_second_loop_update(&mut self, scan: &V, now: f64, control: &V, adopted: bool) -> V {
        self._second_plan_pending = None;
        if !adopted && !self.route_adopted(control, now) {
            return self.result(now);
        }
        let heading = self.compass.current(scan);
        let progress = heading.map(|h| PI - h);
        if progress.is_some_and(|p| p >= PI - open_exit_margin_rad()) {
            return self.open_second_pass(scan, now, control, "compass_heading_reversed_in_loop");
        }
        let target = self.second_target.clone();
        let pose = self.response_pose();
        let id = self.second_track_id.clone();
        let lp = if self.full_target_points(&target, scan, &id).is_none() {
            V::None
        } else {
            self.open_loop_geometry(scan, &target, &id, -1, PI, Some(pose))
        };
        if lp.is_none() {
            if progress.is_some_and(|p| p >= PI - open_late_exit_rad()) {
                return self.open_second_pass(scan, now, control, "support_lost_near_heading_reversal");
            }
            if self._open_lost_since.is_none() {
                self._open_lost_since = Some(now);
            }
            if now - self._open_lost_since.unwrap() < OPEN_LOST_HOLD_S {
                self.steering_target = self.servo;
                self.natural_steering_target = Some(self.servo);
                self.reason = "open_second_loop_missing_frame_hold".into();
                self.slew(now);
                return self.result(now);
            }
            self.begin_recovery_hold("open_second_support_lost", now, control);
            return self.result(now);
        }
        self._open_lost_since = None;
        self.open_loop = lp.clone();
        if let Some(p) = progress
            && p >= OPEN_LIGHT_HANDOFF_PROGRESS_RAD
        {
            let heading = heading.unwrap();
            if let Some((lane, _, _, pursuit_pwm_v)) = self.open_light_pursuit(scan, heading, Some(pose)) {
                let kappa = (NEUTRAL - self.servo) as f64 / ROUTE_PWM_PER_CURVATURE;
                let remaining_lateral =
                    if kappa > 0. && 0. <= heading && heading <= PI / 2. { Some((1. - heading.pcos()) / kappa) } else { None };
                let reaches_lane = remaining_lateral.is_some_and(|r| r >= lane.idx("center_lateral_m").f());
                if pursuit_pwm_v >= lp.idx("target_pwm").i() || reaches_lane {
                    let basis = if reaches_lane { "adopted_arc_reaches_light_lane" } else { "light_lane_pursuit_needs_less_right" };
                    return self.open_second_pass(scan, now, control, basis);
                }
            }
        }
        let t = lp.idx("target_pwm").i();
        self.natural_steering_target = Some(t);
        self.steering_target = self.open_right_gate(t);
        self.reason = "open_second_loop".into();
        self.slew(now);
        self.result(now)
    }

    /// _open_second_pass(scan, now, control, basis).
    fn open_second_pass(&mut self, scan: &V, now: f64, control: &V, basis: &str) -> V {
        self.second_pass_evidence = dict! {
            "source_seq" => scan.idx("seq").clone(), "source_at_ms" => scan.idx("at_ms").clone(),
            "received_at" => scan.idx("received_at").clone(), "track_id" => self.second_track_id.clone(), "basis" => basis,
            "compass" => self.compass.evidence.clone(), "loop" => self.open_loop.clone(),
            "semantic_class" => "unknown", "physical_cone_pass_certified" => false, "swept_path_certified" => false,
        };
        self.route_stage = "light_entry";
        self.light_since = Some(now);
        self.light_turn_started = Some(now);
        self.open_reset_loop(None);
        let e = self.second_pass_evidence.clone();
        self.route_event("second_pass", scan, control, e);
        self.steering_target = self.servo;
        self.natural_steering_target = Some(self.servo);
        self.reason = "open_second_pass_keep_command".into();
        self.result(now)
    }

    /// _open_lane_target(scan, heading).
    fn open_lane_target(&mut self, scan: &V, heading: f64) -> V {
        let u = ((-heading).pcos(), (-heading).psin());
        let n = (-u.1, u.0);
        let mut walls: Vec<Board> = Vec::new();
        let wc = scan.get("wall_candidates");
        let empty: [V; 0] = [];
        let items: &[V] = if wc.truthy() { wc.l() } else { &empty };
        for wall in items {
            if !valid_wall(wall)
                || wall.idx("support_span_m").f() < OPEN_LANE_MIN_SPAN_M
                || (fmod(wall.idx("heading_left_rad").f() + heading + PI / 2., PI) - PI / 2.).abs() > open_lane_parallel_rad()
            {
                continue;
            }
            let ends: Vec<(f64, f64)> = [wall.idx("support_start_left_m"), wall.idx("support_end_left_m")]
                .iter()
                .map(|p| (p.idx("x_m").f(), p.idx("y_m").f()))
                .collect();
            let mut along: Vec<f64> = ends.iter().map(|p| u.0 * p.0 + u.1 * p.1).collect();
            sort_floats(&mut along);
            walls.push(Board {
                lateral: fsum_py(ends.iter().map(|p| n.0 * p.0 + n.1 * p.1)) / 2.,
                along: (along[0], along[1]),
                source: "native_wall",
            });
        }
        for line in self.scan_axis_lines(scan, heading).iter() {
            walls.push(Board { lateral: line.lateral_m, along: line.along_m, source: line.source });
        }
        let outer_candidates: Vec<&Board> =
            walls.iter().filter(|w| w.lateral > SIDE_BODY_EXTENT_M && w.along.1 > FRONT_BODY_EXTENT_M).collect();
        let Some(oi) = argmax_first(&outer_candidates, |w| w.lateral) else { return V::None };
        let outer = outer_candidates[oi].clone();
        let inner_candidates: Vec<&Board> = walls
            .iter()
            .filter(|w| {
                outer.lateral - OPEN_LANE_WIDTH_M.1 <= w.lateral
                    && w.lateral <= outer.lateral - OPEN_LANE_WIDTH_M.0
                    && w.along.1 > FRONT_BODY_EXTENT_M
            })
            .collect();
        let inner = argmax_first(&inner_candidates, |w| w.lateral).map(|i| inner_candidates[i].clone());
        let prior = if self._start_lane_width.is_num() { self._start_lane_width.f() } else { OPEN_LANE_WIDTH_M.0 };
        let width = match &inner {
            Some(i) => outer.lateral - i.lateral,
            None => prior,
        };
        dict! {
            "outer_lateral_m" => outer.lateral, "outer_source" => outer.source,
            "inner_lateral_m" => inner.as_ref().map(|i| i.lateral),
            "inner_source" => inner.as_ref().map(|i| i.source),
            "inner_start_along_m" => inner.as_ref().map(|i| i.along.0),
            "width_m" => width, "width_source" => if inner.is_some() { "measured" } else { "start_lane_prior" },
            "center_lateral_m" => outer.lateral - width / 2., "walls" => walls.len(),
        }
    }

    /// _open_light_pursuit(scan, heading, pose): (lane, point, curvature, pwm).
    pub fn open_light_pursuit(&mut self, scan: &V, heading: f64, pose: Option<(f64, f64, f64, f64)>) -> Option<(V, (f64, f64), f64, i64)> {
        let lane = self.open_lane_target(scan, heading);
        if lane.is_none() {
            return None;
        }
        let u = ((-heading).pcos(), (-heading).psin());
        let lateral = lane.idx("center_lateral_m").f();
        let start0 = lane.idx("inner_start_along_m").of();
        let lane_point = |moved_along: f64, moved_lateral: f64| -> (f64, f64) {
            let start = start0.map(|s| s - moved_along);
            let mut along = match start {
                None => OPEN_LANE_LOOKAHEAD_M.1,
                Some(s) if s <= FRONT_BODY_EXTENT_M => OPEN_LANE_LOOKAHEAD_M.1,
                Some(s) => pmax(OPEN_LANE_LOOKAHEAD_M.0, pmin(OPEN_LANE_LOOKAHEAD_M.1, s)),
            };
            along = pmin(along, pmax(OPEN_LANE_LOOKAHEAD_M.0, OPEN_LOOKAHEAD_M + (lateral - moved_lateral).abs()));
            along += moved_along;
            (along * u.0 - lateral * u.1, along * u.1 + lateral * u.0)
        };
        let point = lane_point(0., 0.);
        let (curvature, pwm) = guarded_pursuit(point, &lane_point, pose, u);
        Some((lane, point, curvature, pwm))
    }

    /// _open_entry_path_hit(scan, point, heading, reach, fallback).
    pub fn open_entry_path_hit(&self, scan: &V, point: (f64, f64), heading: f64, reach: f64, fallback: Option<f64>) -> Option<f64> {
        if !scan.get("ranges").is_list() {
            return fallback;
        }
        let half = SIDE_BODY_EXTENT_M + OPEN_APPROACH_MARGIN_M;
        let limit = reach + FRONT_BODY_EXTENT_M;
        let (px, py) = point;
        let curvature = 2. * py / pmax(1e-6, px * px + py * py);
        let arc_to_point = if curvature.abs() < 1e-3 {
            hypot(px, py)
        } else {
            let radius = 1. / curvature;
            radius.abs() * fmod(if curvature > 0. { px.atan2(radius - py) } else { px.atan2(py - radius) }, 2. * PI)
        };
        let u = ((-heading).pcos(), (-heading).psin());
        let mut best: Option<f64> = None;
        let floats = Self::ranges_floats(scan);
        for (x, y) in scan_points(&floats, limit + 1.) {
            let mut hit = None;
            let (along, offset) = if curvature.abs() < 1e-3 {
                (x, y)
            } else {
                let radius = 1. / curvature;
                let offset = radius.abs() - hypot(x, y - radius);
                let along = radius.abs() * fmod(if curvature > 0. { x.atan2(radius - y) } else { x.atan2(y - radius) }, 2. * PI);
                (along, offset)
            };
            if 0. < along && along <= pmin(limit, arc_to_point) && offset.abs() <= half {
                hit = Some(along);
            } else {
                let (dx, dy) = (x - px, y - py);
                let along = dx * u.0 + dy * u.1;
                if 0. <= along && along <= limit - arc_to_point && (dy * u.0 - dx * u.1).abs() <= half {
                    hit = Some(arc_to_point + along);
                }
            }
            if let Some(h) = hit {
                best = Some(match best {
                    None => h,
                    Some(b) => pmin(b, h),
                });
            }
        }
        best
    }

    /// _open_light_lane_entry(scan, heading, lane).
    pub fn open_light_lane_entry(&self, scan: &V, heading: f64, lane: &V) -> V {
        let walls = corridor_walls(scan);
        let inside = lane.idx("inner_lateral_m").is_some()
            && lane.idx("inner_lateral_m").f() <= -(SIDE_BODY_EXTENT_M + OPEN_LANE_INSIDE_MARGIN_M)
            && lane.idx("outer_lateral_m").f() >= SIDE_BODY_EXTENT_M + OPEN_LANE_INSIDE_MARGIN_M
            && lane.idx("inner_start_along_m").f() <= OPEN_LANE_LOOKAHEAD_M.0;
        let fitted = walls.is_some()
            && OPEN_LANE_WIDTH_M.0 <= walls.idx("width_m").f()
            && walls.idx("width_m").f() <= OPEN_LANE_WIDTH_M.1
            && lane.idx("center_lateral_m").f().abs() <= walls.idx("width_m").f() / 2. - SIDE_BODY_EXTENT_M;
        if heading.abs() <= open_lane_entry_heading_rad() && (inside || fitted) {
            return dict! {
                "source_seq" => scan.idx("seq").clone(), "walls" => walls,
                "lane_target" => lane.clone(), "compass_heading_rad" => heading,
                "basis" => if inside { "car_between_outer_board_and_near_divider_lines" } else { "two_parallel_boards_round_car_on_light_lane_line" },
            };
        }
        V::None
    }

    /// _open_light_entry_update(scan, now, control).
    pub fn open_light_entry_update(&mut self, scan: &V, now: f64, control: &V) -> V {
        if !self.route_adopted(control, now) {
            return self.result(now);
        }
        let heading = self.compass.current(scan);
        let pursuit = match heading {
            None => None,
            Some(h) => {
                let pose = self.response_pose();
                self.open_light_pursuit(scan, h, Some(pose))
            }
        };
        let Some((lane, point, curvature, natural)) = pursuit else {
            self.begin_recovery_hold("open_light_lane_unmeasured", now, control);
            return self.result(now);
        };
        let heading = heading.unwrap();
        let entry = self.open_light_lane_entry(scan, heading, &lane);
        if entry.is_some() {
            return self.begin_light_lane(scan, now, control, entry);
        }
        let walls = corridor_walls(scan);
        let (axis_ahead, speed, stop_at) = self.open_front_stop(scan);
        let ahead = self.open_entry_path_hit(scan, point, heading, stop_at, axis_ahead);
        self.light_feedback = dict! {
            "source_seq" => scan.idx("seq").clone(), "stage" => "light_entry", "compass_heading_rad" => heading,
            "lane_target" => lane, "pursuit_point_left_m" => round3_list(&[point.0, point.1]),
            "pursuit_curvature_per_m" => curvature, "nominal_target_pwm" => natural, "walls" => walls,
            "front_range_m" => ahead, "axis_front_range_m" => axis_ahead, "speed_mps" => speed,
            "front_stop_range_m" => stop_at, "swept_path_certified" => false,
        };
        if ahead.is_some_and(|a| a <= stop_at) {
            self.begin_recovery_hold("open_board_ahead_before_light_lane", now, control);
            return self.result(now);
        }
        self.natural_steering_target = Some(natural);
        self.steering_target = self.open_right_gate(natural);
        self.reason = "open_light_entry".into();
        self.slew(now);
        self.result(now)
    }
}

/// _guarded_pursuit(point, build, pose, u).
fn guarded_pursuit(
    point: (f64, f64),
    build: &dyn Fn(f64, f64) -> (f64, f64),
    pose: Option<(f64, f64, f64, f64)>,
    u: (f64, f64),
) -> (f64, i64) {
    let (curvature, pwm) = pursuit_pwm(point);
    let Some(pose) = pose else { return (curvature, pwm) };
    let moved = (pose.0 * u.0 + pose.1 * u.1, pose.1 * u.0 - pose.0 * u.1);
    let predicted = pursuit_pwm(in_pose(build(moved.0, moved.1), pose));
    if overshoot_guard(pwm, predicted.1) != pwm { predicted } else { (curvature, pwm) }
}
