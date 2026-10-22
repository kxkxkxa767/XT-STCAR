//! After the first pass: exit corridor, second support (corridor proof,
//! recovery, orbit), light turn and the light lane to the finish.

use crate::compact_target::{associated, candidates, rotated_observation, CandidateArgs};
use crate::dict;
use crate::helpers::*;
use crate::maneuver::*;
use crate::maneuver_tm::{mature_observation, observation_lease};
use crate::py::*;

pub const LIGHT_HEADINGS: std::ops::RangeInclusive<i64> = -100..=100;

pub fn light_headings_deg() -> Vec<i64> {
    LIGHT_HEADINGS.step_by(2).collect()
}

pub struct FreeTables {
    pub bearing_deg: Vec<f64>,
    pub point_cos: Vec<f64>,
    pub point_sin: Vec<f64>,
    pub cos: Vec<Vec<f64>>,
    pub sin: Vec<Vec<f64>>,
}

thread_local! {
    pub static FREE: FreeTables = {
        let headings = light_headings_deg();
        let bearing: Vec<f64> = (0..360).map(|i| degrees(wrap(-radians(i as f64)))).collect();
        FreeTables {
            point_cos: bearing.iter().map(|b| radians(*b).pcos()).collect(),
            point_sin: bearing.iter().map(|b| radians(*b).psin()).collect(),
            cos: bearing.iter().map(|b| headings.iter().map(|&h| radians(h as f64 - b).pcos()).collect()).collect(),
            sin: bearing.iter().map(|b| headings.iter().map(|&h| radians(h as f64 - b).psin()).collect()).collect(),
            bearing_deg: bearing,
        }
    };
}

fn rate_ok(rate: Option<f64>) -> Option<f64> {
    rate
}

impl Maneuver {
    /// _right_exit_target(scan, now): (target, rejection).
    pub fn right_exit_target(&mut self, scan: &V, now: f64) -> (Option<i64>, Option<&'static str>) {
        let margin = SIDE_BODY_EXTENT_M + MANEUVER_BODY_CLEARANCE_M;
        let values = scan.get("corridor_candidates");
        let Some(list) = values.ol() else { return (None, Some("right_exit_invalid_corridor")) };
        if list.len() > 24 || list.iter().any(|c| !valid_candidate(c, margin)) {
            return (None, Some("right_exit_invalid_corridor"));
        }
        let mut forward: Vec<V> = list
            .iter()
            .filter(|c| -PI / 2. < c.idx("heading_left_rad").f() && c.idx("heading_left_rad").f() < PI / 2.)
            .cloned()
            .collect();
        if forward.len() > 1 && self.continue_route {
            let base = forward[0].clone();
            if forward[1..].iter().all(|c| {
                wrap(c.idx("heading_left_rad").f() - base.idx("heading_left_rad").f()).abs() <= pair_merge_heading_rad()
                    && (c.idx("center_offset_left_m").f() - base.idx("center_offset_left_m").f()).abs() <= PAIR_MERGE_OFFSET_M
                    && (c.idx("width_m").f() - base.idx("width_m").f()).abs()
                        <= pmax(0.15, MAX_EXIT_WIDTH_CHANGE_FRACTION * base.idx("width_m").f())
            }) {
                let key = |c: &V| -> (f64, f64) {
                    let lp = get_or(c, "left_wall_points", &V::Int(0)).f();
                    let rp = get_or(c, "right_wall_points", &V::Int(0)).f();
                    (lp + rp, -get_or(c, "fit_error_m", &V::Float(0.)).f())
                };
                let mut best = 0usize;
                for i in 1..forward.len() {
                    if key(&forward[i]) > key(&forward[best]) {
                        best = i;
                    }
                }
                forward = vec![forward[best].clone()];
            }
        }
        if forward.len() > 1 {
            return (None, Some("right_exit_corridor_ambiguous"));
        }
        let c = if let Some(first) = forward.first() {
            let mut c = first.clone();
            c.set("source_kind", "native_corridor");
            c
        } else if self.continue_route {
            let (c, ambiguous) = Self::paired_window_corridor(scan);
            if ambiguous {
                return (None, Some("right_exit_corridor_ambiguous"));
            }
            if c.is_none() || !valid_candidate(&c, margin) {
                return (None, Some("right_exit_corridor_missing"));
            }
            c
        } else {
            return (None, Some("right_exit_corridor_missing"));
        };
        let previous = self.right_exit_geometry.clone();
        if previous.is_some() {
            let dt = (scan.idx("at_ms").i() - previous.idx("source_at_ms").i()) as f64 / 1000.;
            let receive_dt = scan.idx("received_at").f() - previous.idx("received_at").f();
            if dt <= 0.
                || dt >= SCAN_AGE_S
                || (self.continue_route
                    && (!(0. < receive_dt && receive_dt < SCAN_AGE_S)
                        || now - previous.idx("received_at").f() >= SCAN_AGE_S
                        || (dt - receive_dt).abs() > 0.15))
                || (c.idx("heading_left_rad").f() - previous.idx("heading_left_rad").f()).abs() > radians(25.)
                || (c.idx("center_offset_left_m").f() - previous.idx("center_offset_left_m").f()).abs() > 0.30
                || (c.idx("width_m").f() - previous.idx("width_m").f()).abs() > pmax(0.15, 0.1 * previous.idx("width_m").f())
            {
                return (None, Some("right_exit_corridor_identity_changed"));
            }
        }
        let width = c.idx("width_m").f();
        let lookahead = pmax(
            FRONT_BODY_EXTENT_M + REAR_BODY_EXTENT_M,
            if self.continue_route { pmin(ROUTE_ALIGN_LOOKAHEAD_MAX_M, width) } else { pmin(1., width / 2.) },
        );
        let mut heading_rate = 0.;
        if self.continue_route && previous.is_some() {
            heading_rate = div(
                wrap(c.idx("heading_left_rad").f() - previous.idx("heading_left_rad").f()),
                (scan.idx("at_ms").i() - previous.idx("source_at_ms").i()) as f64 / 1000.,
            );
        }
        let approach = self.approach_offset(&c);
        let error = c.idx("heading_left_rad").f()
            + (c.idx("center_offset_left_m").f() + approach).atan2(lookahead)
            + ROUTE_ALIGN_DAMPING_S * heading_rate;
        let upper = if self.continue_route { SERVO_MAX } else { NEUTRAL };
        let target = clampi(NEUTRAL + round(LEFT_TRIAL_GAIN * error), right_exit_min_pwm(), upper);
        let mut geometry = c.clone();
        geometry.update(&dict! {
            "source_seq" => scan.idx("seq").clone(), "source_at_ms" => scan.idx("at_ms").clone(),
            "received_at" => scan.idx("received_at").clone(),
            "lookahead_m" => lookahead, "steering_target_pwm" => target, "heading_rate_rad_s" => heading_rate,
            "approach_offset_left_m" => approach,
            "source" => "current_native_corridor", "second_target_confirmed" => false,
        });
        self.right_exit_geometry = geometry;
        self._right_exit_last_receive = scan.idx("received_at").clone();
        self._right_exit_last_publication = scan.idx("at_ms").clone();
        (Some(target), None)
    }

    /// _approach_offset(corridor).
    fn approach_offset(&self, corridor: &V) -> f64 {
        if !(self.continue_route && self.first_pass_evidence.is_some() && ["right_align", "corridor_follow"].contains(&self.route_stage)) {
            return 0.;
        }
        let room = corridor.idx("width_m").f() / 2. - (SIDE_BODY_EXTENT_M + MANEUVER_BODY_CLEARANCE_M) - SECOND_APPROACH_WALL_MARGIN_M;
        pmax(0., pmin(SECOND_APPROACH_LEFT_M, room))
    }

    /// _corridor_body_gap(candidate).
    pub fn corridor_body_gap(candidate: &V) -> f64 {
        let heading = candidate.idx("heading_left_rad").f();
        let mut projections = Vec::new();
        for x in [-REAR_BODY_EXTENT_M, FRONT_BODY_EXTENT_M] {
            for y in [-SIDE_BODY_EXTENT_M, SIDE_BODY_EXTENT_M] {
                projections.push(-heading.psin() * x + heading.pcos() * y);
            }
        }
        let offset = candidate.idx("center_offset_left_m").f();
        let half = candidate.idx("width_m").f() / 2.;
        let pmax_v = projections.iter().cloned().fold(f64::NEG_INFINITY, pmax);
        let pmin_v = projections.iter().cloned().fold(INF, pmin);
        pmin(half + offset - pmax_v, pmin_v - (offset - half))
    }

    /// _route_adopted(control, now).
    pub fn route_adopted(&mut self, control: &V, now: f64) -> bool {
        let servo = get_or(control, "servo", &V::Int(NEUTRAL)).clone();
        if servo.f() > NEUTRAL as f64 {
            self._route_center_ack = V::None;
            self.second_center_ack = V::None;
            if self.servo < NEUTRAL {
                self.lock("route_actual_steering_crossed_center");
                return false;
            }
        }
        let mut motors = Vec::new();
        if self.route_stage == "light_lane" || self.route_stage == "start_lane" {
            motors.push(NEUTRAL);
        }
        motors.extend_from_slice(&DRIVE_MOTORS);
        let adopted = motor_in(control.get("motor"), &motors)
            && eq_i(control.get("servo"), self.servo)
            && control.get("command_acked").is_true();
        if !adopted {
            if self.right_exit_adoption_wait_since.is_none() {
                self.right_exit_adoption_wait_since = Some(now);
            }
            if now - self.right_exit_adoption_wait_since.unwrap() >= SCAN_AGE_S {
                self.begin_coast("route_adoption_unconfirmed", now);
            } else {
                self.steering_target = self.servo;
                self.reason = "route_waiting_adopted_command".into();
            }
            return false;
        }
        self.right_exit_adoption_wait_since = None;
        if self.servo > NEUTRAL {
            self._route_center_ack = V::None;
        } else if self.servo == NEUTRAL
            && (motor_in(control.get("motor"), &DRIVE_MOTORS) || (self.route_stage == "light_lane" && eq_i(control.get("motor"), NEUTRAL)))
        {
            self._route_center_ack = control.pick(&["servo", "motor", "tick", "seq"]);
        }
        true
    }

    /// _right_exit_update(scan, now, control).
    pub fn right_exit_update(&mut self, scan: &V, now: f64, control: &V) -> V {
        if now - self.right_exit_since.unwrap() + 1e-9 >= RIGHT_EXIT_MAX_S {
            if self.continue_route {
                self.begin_recovery_hold("first_target_right_exit_trial_timeout", now, control);
            } else {
                self.begin_coast("first_target_right_exit_trial_timeout", now);
            }
            self.slew(now);
            return self.result(now);
        }
        let (target, rejection) = self.right_exit_target(scan, now);
        let Some(target) = target else {
            let rejection = rejection.unwrap();
            let mut lease_receive = self._right_exit_last_receive.clone();
            let mut lease_publication = self._right_exit_last_publication.clone();
            if lease_receive.is_none() && self.continue_route && self.servo > NEUTRAL && self.first_pass_evidence.is_dict() {
                lease_receive = self.first_pass_evidence.get("received_at").clone();
                lease_publication = self.first_pass_evidence.get("source_at_ms").clone();
            }
            let held = (rejection == "right_exit_corridor_missing" || (self.continue_route && rejection == "right_exit_corridor_ambiguous"))
                && lease_receive.is_num()
                && lease_publication.is_num()
                && now - lease_receive.f() < SCAN_AGE_S
                && (scan.idx("at_ms").f() - lease_publication.f()) / 1000. < SCAN_AGE_S;
            if held && self.continue_route && self.servo > NEUTRAL {
                self.steering_target = NEUTRAL;
                self.reason = "right_exit_corridor_missing_release_left".into();
                self.slew(now);
            } else if held {
                self.steering_target = self.servo;
                self.reason = "right_exit_corridor_missing_hold".into();
            } else if self.continue_route {
                self.begin_recovery_hold(rejection, now, control);
                self.slew(now);
            } else {
                self.begin_coast(rejection, now);
                self.slew(now);
            }
            return self.result(now);
        };
        if self.continue_route && !self.route_adopted(control, now) {
            return self.result(now);
        }
        let motors: &[i64] = if self.continue_route { &DRIVE_MOTORS } else { &[TRIAL_MOTOR] };
        let adopted = motor_in(control.get("motor"), motors) && eq_i(control.get("servo"), self.servo) && control.get("command_acked").is_true();
        if !adopted {
            if self.right_exit_adoption_wait_since.is_none() {
                self.right_exit_adoption_wait_since = Some(now);
            }
            if now - self.right_exit_adoption_wait_since.unwrap() + 1e-9 >= SCAN_AGE_S {
                self.begin_coast("right_exit_adoption_unconfirmed", now);
                self.slew(now);
            } else {
                self.steering_target = self.servo;
                self.reason = "right_exit_waiting_adopted_command".into();
            }
            return self.result(now);
        }
        self.right_exit_adoption_wait_since = None;
        let mut target = target;
        if self.right_exit_center_ack.is_none() {
            if self.servo == NEUTRAL
                && eq_i(control.get("servo"), NEUTRAL)
                && motor_in(control.get("motor"), motors)
                && control.get("command_acked").is_true()
            {
                self.right_exit_center_ack = control.pick(&["servo", "motor", "tick", "seq"]);
                self._route_center_ack = self.right_exit_center_ack.clone();
            } else if !self.continue_route {
                target = NEUTRAL;
            }
        }
        let natural = target;
        if self.continue_route
            && target < NEUTRAL
            && (self.right_exit_center_ack.is_none() || self._route_center_ack.is_none() || self.servo > NEUTRAL)
        {
            target = NEUTRAL;
        }
        if self.continue_route {
            self.second_role_observation(scan, now);
            if self.second_target.is_some() && self.second_target.get("confirmed").is_true() {
                return self.begin_second_orbit(scan, now, control);
            }
            let c = self.right_exit_geometry.clone();
            let aligned = c.idx("heading_left_rad").f().abs() <= route_align_heading_rad()
                && Self::corridor_body_gap(&c) >= MANEUVER_BODY_CLEARANCE_M;
            if aligned && mature_observation(&mut self._route_alignment_history, scan, 0.25) {
                let mut e = c.clone();
                e.update(&dict! {"body_normal_gap_m" => Self::corridor_body_gap(&c), "physical_alignment_certified" => false});
                self.route_alignment_evidence = e;
                self.route_stage = "corridor_follow";
                let e = self.route_alignment_evidence.clone();
                self.route_event("corridor_follow", scan, control, e);
                self.steering_target = self.servo;
                self.reason = "route_corridor_alignment_observed".into();
                return self.result(now);
            }
            if !aligned {
                self._route_alignment_history.clear();
            }
        }
        self.steering_target = target;
        self.natural_steering_target = Some(natural);
        self.reason = if target > NEUTRAL {
            "first_target_exit_alignment_left".into()
        } else if self.right_exit_center_ack.is_some() {
            "first_target_right_exit_tracking".into()
        } else {
            "first_target_right_exit_waiting_center_ack".into()
        };
        self.slew(now);
        self.result(now)
    }

    /// _start_second_identity_recovery().
    fn start_second_identity_recovery(&mut self) {
        self.clear_second_return();
        if self._second_role_anchor.is_none() {
            return;
        }
        self._second_recovering = true;
        self._second_recovery_last = V::None;
        self.second_tracker_mut().widen_acquisition((-PI, PI / 2.));
    }

    /// _second_recovery_observation(scan, now, fallback).
    fn second_recovery_observation(&mut self, scan: &V, now: f64, fallback: bool) -> bool {
        let anchor = self._second_role_anchor.clone();
        if !self._second_recovering || anchor.is_none() || self.first_pass_evidence.is_none() {
            return false;
        }
        let previous = self._second_recovery_last.clone();
        let mut reference = if previous.is_dict()
            && observation_lease(scan, now, previous.get("source_at_ms"), previous.get("source_received_at"))
        {
            previous
        } else {
            anchor.clone()
        };
        let found = candidates(
            scan.idx("ranges"),
            CandidateArgs {
                bearing_range_rad: (-PI, PI / 2.),
                tolerant: true,
                min_points: Some(SECOND_ACQUISITION_MIN_POINTS),
                ..CandidateArgs::default()
            },
        );
        let first = self.compact_target.clone();
        let first_bins: Vec<i64> = if first.is_dict()
            && py_eq(first.get("track_id"), &self.orbit_track_id)
            && py_eq(first.get("source_seq"), scan.get("seq"))
        {
            let mut b = ints_of(first.idx("support_bins"));
            b.sort();
            b.dedup();
            b
        } else {
            vec![]
        };
        let floats = Self::ranges_floats(scan);
        let pt = |i: i64| -> (f64, f64) {
            let r = floats[i as usize].unwrap_or_else(|| type_error("unsupported operand type(s) for *"));
            let a = radians(i as f64);
            (r * a.pcos(), -r * a.psin())
        };
        let first_points: Vec<(f64, f64)> = first_bins.iter().map(|&i| pt(i)).collect();
        let mut slack = pmin(SECOND_RECOVERY_MAX_SLACK_S, pmax(0., (scan.idx("at_ms").f() - reference.idx("source_at_ms").f()) / 1000. - 0.1));
        let heading = self.tracking_heading(scan);
        let landmark = self.first_landmark(scan);
        let prior_landmark = reference.get("first_landmark_left_m").clone();
        if let Some(h) = heading
            && reference.get("compass_heading_rad").is_num()
            && landmark.is_some()
            && prior_landmark.is_list()
        {
            let turn = -(h - reference.idx("compass_heading_rad").f());
            let (c, s) = (turn.pcos(), turn.psin());
            let rp = reference.idx("point_left_m").l();
            let pl = prior_landmark.l();
            let dx = rp[0].f() - pl[0].f();
            let dy = rp[1].f() - pl[1].f();
            let lm = landmark.unwrap();
            let point = (lm.0 + c * dx - s * dy, lm.1 + s * dx + c * dy);
            reference.update(&dict! {"point_left_m" => V::from(point), "bearing_left_rad" => point.1.atan2(point.0)});
            slack = pmin(slack, SECOND_LANDMARK_SLACK_S);
            self.second_recovery_prior = dict! {
                "basis" => "first_support_landmark", "source_seq" => scan.idx("seq").clone(),
                "point_left_m" => V::from(point), "slack_s" => slack,
            };
        } else if let Some(h) = heading
            && reference.get("compass_heading_rad").is_num()
        {
            reference = rotated_observation(&reference, Some(h - reference.idx("compass_heading_rad").f()));
            self.second_recovery_prior = dict! {
                "basis" => "compass_rotation", "source_seq" => scan.idx("seq").clone(),
                "point_left_m" => reference.idx("point_left_m").clone(), "slack_s" => slack,
            };
        }
        let proven = |candidate: &V| -> bool {
            let bins = ints_of(candidate.idx("support_bins"));
            if !associated(candidate, &reference, slack) || bins.iter().any(|b| first_bins.contains(b)) {
                return false;
            }
            let points: Vec<(f64, f64)> = bins.iter().map(|&i| pt(i)).collect();
            if first_points.is_empty() {
                return true;
            }
            let mut m = INF;
            for a in &first_points {
                for b in &points {
                    m = pmin(m, dist(*a, *b));
                }
            }
            m > MAX_DIAMETER_M
        };
        let matches: Vec<V> = found.iter().filter(|c| proven(c)).cloned().collect();
        if matches.len() != 1 && fallback {
            self.second_acquisition_reason = Some("second_role_recovery_unproven_corridor_proof_next".into());
            return false;
        }
        if matches.len() != 1 {
            if !found.is_empty() {
                self.second_tracker_mut().forget_pub("second_role_recovery_unproven");
                self._second_recovery_last = V::None;
            }
            self.second_tracker_mut().update(scan, now, false, heading, None);
            self.second_target = V::None;
            self.second_acquisition_reason = Some("second_role_recovery_current_support_unproven".into());
            return false;
        }
        let only = ints_of(matches[0].idx("support_bins"));
        let target = self.second_tracker_mut().update(scan, now, true, heading, Some(&only));
        self.second_target = target.clone();
        if target.is_some() && !target.idx("tracking_only_boundary_gap").truthy() {
            self._second_recovery_last = self.second_prior(&target, scan);
        }
        self.second_acquisition_reason = Some(self.second_tracker.as_ref().unwrap().reason.clone());
        let same_bins = target.is_some() && {
            let mut a = ints_of(target.idx("support_bins"));
            a.sort();
            a.dedup();
            let mut b = only.clone();
            b.sort();
            b.dedup();
            a == b
        };
        if target.is_none() || !same_bins || self.full_target_points(&target, scan, target.idx("track_id")).is_none() {
            return true;
        }
        let old_id = self.second_track_id.clone();
        self.second_track_id = target.idx("track_id").clone();
        let id = self.second_track_id.clone();
        self.second_tracker_mut().enable_close_maintenance(&id);
        self._second_recovering = false;
        self.second_target_handover_observed = true;
        self._second_role_anchor = self.second_prior(&target, scan);
        self.second_orbit_since = Some(now);
        self.clear_target_recovery_history();
        self.clear_second_return();
        self.second_center_ack = V::None;
        self.second_rebind_evidence = dict! {
            "old_track_id" => old_id, "new_track_id" => target.idx("track_id").clone(),
            "source_seq" => scan.idx("seq").clone(), "source_at_ms" => scan.idx("at_ms").clone(),
            "source_received_at" => scan.idx("received_at").clone(), "anchor_source_seq" => anchor.idx("source_seq").clone(),
            "support_bins" => target.idx("support_bins").clone(),
            "basis" => "current_confirmed_support_associated_with_last_complete_second_observation",
            "physical_identity_verified" => false, "observed_rear_pass" => false,
        };
        true
    }

    /// _first_landmark(scan).
    fn first_landmark(&self, scan: &V) -> Option<(f64, f64)> {
        let first = &self.compact_target;
        if self.orbit_track_id.is_some()
            && first.is_dict()
            && py_eq(first.get("track_id"), &self.orbit_track_id)
            && first.get("confirmed").is_true()
            && py_eq(first.get("source_seq"), scan.get("seq"))
            && first.get("point_left_m").is_list()
        {
            let p = first.idx("point_left_m").l();
            return Some((p[0].f(), p[1].f()));
        }
        None
    }

    fn first_landmark_v(&self, scan: &V) -> V {
        let first = &self.compact_target;
        if self.first_landmark(scan).is_some() { first.idx("point_left_m").clone() } else { V::None }
    }

    /// _second_prior(observation, scan).
    pub fn second_prior(&self, observation: &V, scan: &V) -> V {
        let mut prior = observation.clone();
        let landmark = self.first_landmark_v(scan);
        if prior.is_dict() && landmark.is_some() && py_eq(prior.get("source_seq"), scan.get("seq")) {
            prior.set("first_landmark_left_m", landmark);
        }
        prior
    }

    /// _update_second_tracking(scan, now).
    pub fn update_second_tracking(&mut self, scan: &V, now: f64) {
        if let Some(seq) = self._open_lookout_seq
            && eq_i(scan.get("seq"), seq)
        {
            return;
        }
        if self._second_recovering {
            self.second_recovery_observation(scan, now, false);
            return;
        }
        let heading = self.tracking_heading(scan);
        self.second_target = self.second_tracker_mut().update(scan, now, false, heading, None);
        if self.full_target_points(&self.second_target.clone(), scan, &self.second_track_id.clone()).is_some() {
            self._second_role_anchor = self.second_prior(&self.second_target.clone(), scan);
        } else if self.second_target.is_none() {
            let held = self.second_tracker.as_ref().unwrap().held.clone();
            if !Self::target_gap_held(&held, &self.second_track_id, scan, now) {
                self.start_second_identity_recovery();
            }
        }
    }

    /// _second_role_observation(scan, now).
    fn second_role_observation(&mut self, scan: &V, now: f64) {
        if self._second_recovering && self.second_recovery_observation(scan, now, true) {
            return;
        }
        let c = self.right_exit_geometry.clone();
        let mut proof = V::None;
        if self.first_pass_evidence.is_some() && c.is_dict() {
            let first = self.full_target_points(&self.compact_target.clone(), scan, &self.orbit_track_id.clone());
            let heading = c.idx("heading_left_rad").f();
            let bound = c.idx("width_m").f() / 2. - MANEUVER_BODY_CLEARANCE_M;
            let mut ahead: Vec<(V, f64, f64)> = Vec::new();
            for target in candidates(
                scan.idx("ranges"),
                CandidateArgs {
                    bearing_range_rad: (-PI / 2., PI / 2.),
                    tolerant: true,
                    min_points: Some(SECOND_ACQUISITION_MIN_POINTS),
                    ..CandidateArgs::default()
                },
            ) {
                let p = target.idx("point_left_m").l();
                let (x, y) = (p[0].f(), p[1].f());
                let along = heading.pcos() * x + heading.psin() * y;
                let normal = -heading.psin() * x + heading.pcos() * y;
                if x > FRONT_BODY_EXTENT_M && along > FRONT_BODY_EXTENT_M && (normal - c.idx("center_offset_left_m").f()).abs() < bound {
                    ahead.push((target, along, normal));
                }
            }
            if ahead.len() == 1 {
                let (target, along, normal) = ahead[0].clone();
                let bins = ints_of(target.idx("support_bins"));
                let floats = Self::ranges_floats(scan);
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
                        let fb = ints_of(self.compact_target.idx("support_bins"));
                        !bins.iter().any(|b| fb.contains(b)) && separation.unwrap() > MAX_DIAMETER_M
                    }
                };
                if distinct {
                    proof = dict! {
                        "source_seq" => scan.idx("seq").clone(), "source_at_ms" => scan.idx("at_ms").clone(),
                        "source_received_at" => scan.idx("received_at").clone(), "first_track_id" => self.orbit_track_id.clone(),
                        "first_support_visible" => first.is_some(),
                        "first_support_bins" => if first.is_some() { self.compact_target.idx("support_bins").clone() } else { V::None },
                        "first_pass_source_seq" => self.first_pass_evidence.get("source_seq").clone(),
                        "second_support_bins" => V::from(bins.clone()), "actual_support_separation_m" => separation,
                        "corridor_heading_left_rad" => heading, "second_along_m" => along,
                        "second_normal_m" => normal, "second_side" => if p_y(&target) > 0. { "left" } else { "right" },
                        "basis" => "single_support_ahead_inside_current_corridor_after_first_pass",
                        "physical_identity_verified" => false,
                    };
                }
            }
        }
        if proof.is_none() && self._second_proof_at.is_some_and(|at| (0. ..SCAN_AGE_S).contains(&(now - at))) {
            self.second_target = V::None;
            self.second_acquisition_reason = Some("second_role_proof_gap_held".into());
            return;
        }
        if proof.is_none() {
            self._second_proof_at = None;
            if !self._second_recovering {
                self.second_tracker = Some(new_second_tracker());
            }
            self.second_target = V::None;
            self.role_distinct_evidence = V::None;
            self.second_acquisition_reason = Some("second_role_current_distinct_support_missing".into());
            return;
        }
        self._second_proof_at = Some(now);
        self.second_target = self.second_tracker_mut().update(scan, now, true, None, None);
        self.role_distinct_evidence = proof;
        self.second_acquisition_reason = Some(self.second_tracker.as_ref().unwrap().reason.clone());
    }

    /// _corridor_follow_update(scan, now, control).
    pub fn corridor_follow_update(&mut self, scan: &V, now: f64, control: &V) -> V {
        if self.second_target_handover_observed
            && self.full_target_points(&self.second_target.clone(), scan, &self.second_track_id.clone()).is_some()
        {
            return self.begin_second_orbit(scan, now, control);
        }
        let (_, rejection) = self.right_exit_target(scan, now);
        if let Some(rejection) = rejection {
            let (lease_receive, lease_publication) = (self._right_exit_last_receive.clone(), self._right_exit_last_publication.clone());
            if (rejection == "right_exit_corridor_missing" || rejection == "right_exit_corridor_ambiguous")
                && lease_receive.is_num()
                && lease_publication.is_num()
                && now - lease_receive.f() < SCAN_AGE_S
                && (scan.idx("at_ms").f() - lease_publication.f()) / 1000. < SCAN_AGE_S
            {
                self.steering_target = self.servo;
                let last = rejection.rsplit('_').next().unwrap();
                self.reason = format!("route_corridor_{last}_hold");
                return self.result(now);
            }
            self.begin_recovery_hold(rejection, now, control);
            self.slew(now);
            return self.result(now);
        }
        if Self::corridor_body_gap(&self.right_exit_geometry) < 0. {
            self.begin_recovery_hold("route_corridor_body_margin_unavailable", now, control);
            return self.result(now);
        }
        if !self.route_adopted(control, now) {
            return self.result(now);
        }
        self.second_role_observation(scan, now);
        if self.second_target.is_some() && self.second_target.get("confirmed").is_true() {
            return self.begin_second_orbit(scan, now, control);
        }
        let c = self.right_exit_geometry.clone();
        let rate = c.get("heading_rate_rad_s").of();
        self.lane_left_limit(scan, c.idx("heading_left_rad").f(), rate);
        let error = c.idx("heading_left_rad").f()
            + (c.idx("center_offset_left_m").f() + get_or(&c, "approach_offset_left_m", &V::Float(0.)).f()).atan2(c.idx("lookahead_m").f())
            + ROUTE_ALIGN_DAMPING_S * get_or(&c, "heading_rate_rad_s", &V::Float(0.)).f();
        let mut target = clampi(NEUTRAL + round(LEFT_TRIAL_GAIN * error), NEUTRAL - 55, NEUTRAL + 55);
        let natural = target;
        if target < NEUTRAL && self._route_center_ack.is_none() {
            target = NEUTRAL;
        }
        self.steering_target = target;
        self.natural_steering_target = Some(natural);
        self.reason = "route_corridor_follow_current_geometry".into();
        self.slew(now);
        if self.servo > NEUTRAL {
            self._route_center_ack = V::None;
        }
        self.result(now)
    }

    pub fn clear_second_return(&mut self) {
        self.second_return_preparing = false;
        self.second_return_evidence = V::None;
    }

    /// _second_return_support_clear(points, servo).
    fn second_return_support_clear(points: &[(f64, f64)], servo: i64) -> bool {
        let curvature = (servo - NEUTRAL) as f64 / ROUTE_PWM_PER_CURVATURE;
        if curvature > 0. {
            return false;
        }
        let half = SIDE_BODY_EXTENT_M + MANEUVER_BODY_CLEARANCE_M;
        for &(x, y) in points {
            let (along, right) = if curvature.abs() < 1e-9 {
                (x, -y)
            } else {
                let radius = -1. / curvature;
                (radius * x.atan2(radius + y), radius - hypot(x, radius + y))
            };
            if along + REAR_BODY_EXTENT_M >= 0. && right < half {
                return false;
            }
        }
        true
    }

    /// _lane_left_limit(scan, heading, rate).
    fn lane_left_limit(&mut self, scan: &V, heading: f64, rate: Option<f64>) -> (f64, i64) {
        let predicted = heading + match rate_ok(rate) {
            Some(r) if r < 0. => r * SECOND_STEER_LAG_S,
            _ => 0.,
        };
        let limit = NEUTRAL + round(SECOND_HEADING_HOLD_GAIN * (predicted + second_bypass_max_heading_rad()));
        self._task_left_upper = Some((scan.get("seq").clone(), limit));
        (predicted, limit)
    }

    /// _current_lane(scan, now): (target, corridor).
    pub fn current_lane(&mut self, scan: &V, now: f64) -> (Option<i64>, Option<V>) {
        let saved = (self.right_exit_geometry.clone(), self._right_exit_last_receive.clone(), self._right_exit_last_publication.clone());
        self.right_exit_geometry = V::None;
        self._right_exit_last_receive = V::None;
        self._right_exit_last_publication = V::None;
        let out = py_try(|| {
            let (target, rejection) = self.right_exit_target(scan, now);
            (target, rejection, self.right_exit_geometry.clone())
        });
        self.right_exit_geometry = saved.0;
        self._right_exit_last_receive = saved.1;
        self._right_exit_last_publication = saved.2;
        let (target, rejection, corridor) = match out {
            Ok(v) => v,
            Err(e) => std::panic::panic_any(e),
        };
        if rejection.is_some() { (None, None) } else { (target, Some(corridor)) }
    }

    /// _second_lane_bypass(scan, now, points).
    fn second_lane_bypass(&mut self, scan: &V, now: f64, points: &[(f64, f64)]) -> V {
        let (_, lane) = self.current_lane(scan, now);
        self._second_lane_heading = lane.as_ref().map(|l| l.idx("heading_left_rad").f());
        let previous = self._second_lane_previous;
        self._second_lane_rate = None;
        if let Some(lane) = &lane {
            let row = (scan.idx("at_ms").i() as f64 / 1000., scan.idx("received_at").f(), lane.idx("heading_left_rad").f());
            if let Some(p) = previous
                && 0.05 <= row.0 - p.0
                && row.0 - p.0 <= SCAN_AGE_S
                && 0.05 <= row.1 - p.1
                && row.1 - p.1 <= SCAN_AGE_S
            {
                let rate = wrap(row.2 - p.2) / (row.0 - p.0);
                if rate.abs() <= second_max_heading_rate_rad_s() {
                    self._second_lane_rate = Some(rate);
                }
            }
            self._second_lane_previous = Some(row);
        } else {
            self._second_lane_previous = None;
        }
        let Some(lane) = lane else { return V::None };
        if lane.idx("heading_left_rad").f().abs() > second_lane_max_heading_rad() {
            return V::None;
        }
        let (heading, offset, width) = (lane.idx("heading_left_rad").f(), lane.idx("center_offset_left_m").f(), lane.idx("width_m").f());
        let (c, s) = (heading.pcos(), heading.psin());
        let along: Vec<f64> = points.iter().map(|&(x, y)| c * x + s * y).collect();
        let normal: Vec<f64> = points.iter().map(|&(x, y)| -s * x + c * y).collect();
        let max_normal = normal.iter().cloned().fold(f64::NEG_INFINITY, pmax);
        let wanted = max_normal + SECOND_BYPASS_SUPPORT_GAP_M;
        let room = offset + width / 2. - (SIDE_BODY_EXTENT_M + MANEUVER_BODY_CLEARANCE_M);
        let desired = pmin(wanted, room);
        let ahead = along.iter().cloned().fold(INF, pmin);
        let lookahead = if ahead > 0. { pmax(FRONT_BODY_EXTENT_M + REAR_BODY_EXTENT_M, pmin(SECOND_LANE_LOOKAHEAD_M, ahead)) } else { SECOND_LANE_LOOKAHEAD_M };
        let error = heading + desired.atan2(lookahead);
        let target = clampi(NEUTRAL + round(LEFT_TRIAL_GAIN * error), right_exit_min_pwm(), SERVO_MAX);
        dict! {
            "source_seq" => scan.idx("seq").clone(), "lane_heading_left_rad" => heading,
            "lane_center_offset_left_m" => offset, "lane_width_m" => width,
            "support_left_normal_m" => max_normal, "support_nearest_along_m" => ahead,
            "wanted_offset_m" => wanted, "lane_room_left_m" => room, "desired_offset_m" => desired,
            "lookahead_m" => lookahead, "heading_error_rad" => error, "nominal_target_pwm" => target,
            "scope" => "current_lane_frame_bypass", "swept_path_certified" => false,
        }
    }

    /// _second_return_target(scan, now, points, bypass_target).
    fn second_return_target(&mut self, scan: &V, now: f64, points: &[(f64, f64)], bypass_target: i64) -> Option<i64> {
        if points.iter().map(|p| -p.1).fold(INF, pmin) - SIDE_BODY_EXTENT_M < MANEUVER_BODY_CLEARANCE_M {
            self.clear_second_return();
            return None;
        }
        let saved = (self.right_exit_geometry.clone(), self._right_exit_last_receive.clone(), self._right_exit_last_publication.clone());
        self.right_exit_geometry = V::None;
        self._right_exit_last_receive = V::None;
        self._right_exit_last_publication = V::None;
        let out = py_try(|| {
            let (native, _) = self.right_exit_target(scan, now);
            (native, self.right_exit_geometry.clone())
        });
        self.right_exit_geometry = saved.0;
        self._right_exit_last_receive = saved.1;
        self._right_exit_last_publication = saved.2;
        let (native, corridor) = match out {
            Ok(v) => v,
            Err(e) => std::panic::panic_any(e),
        };
        if native.is_some_and(|n| n > NEUTRAL) {
            self.clear_second_return();
            return None;
        }
        let mut cands: Vec<i64> = Vec::new();
        let mut requested = 0;
        if let Some(native) = native {
            requested = native.min(bypass_target);
            let all_points = Self::path_points(scan);
            let mut set_values = route_path_right_candidates();
            set_values.push(requested);
            set_values.push(NEUTRAL);
            for pwm in py_int_set_order(&set_values) {
                if requested <= pwm
                    && pwm <= NEUTRAL
                    && Self::second_return_support_clear(points, pwm)
                    && all_points.as_ref().is_some_and(|a| !a.is_empty())
                    && Self::path_free_distance(all_points.as_ref().unwrap(), pwm) >= ROUTE_PATH_LOOKAHEAD_M
                {
                    cands.push(pwm);
                }
            }
        }
        if cands.is_empty() {
            if self.second_return_preparing {
                return Some(self.servo.min(NEUTRAL));
            }
            return None;
        }
        let mut choice = cands[0];
        for &c in &cands[1..] {
            if (c - requested).abs() < (choice - requested).abs() {
                choice = c;
            }
        }
        self.second_return_preparing = true;
        let mut sorted = cands.clone();
        sorted.sort();
        self.second_return_evidence = dict! {
            "source_seq" => scan.idx("seq").clone(), "source_at_ms" => scan.idx("at_ms").clone(),
            "received_at" => scan.idx("received_at").clone(), "track_id" => self.second_track_id.clone(),
            "support_bins" => self.second_target.idx("support_bins").clone(),
            "observed_frontmost_x_m" => points.iter().map(|p| p.0).fold(f64::NEG_INFINITY, pmax),
            "observed_right_body_gap_m" => points.iter().map(|p| -p.1).fold(INF, pmin) - SIDE_BODY_EXTENT_M,
            "corridor" => corridor, "corridor_target_pwm" => native,
            "support_bypass_target_pwm" => bypass_target, "requested_target_pwm" => requested,
            "selected_target_pwm" => choice, "clear_candidates" => V::from(sorted),
            "all_obstacle_lookahead_m" => ROUTE_PATH_LOOKAHEAD_M,
            "basis" => "current_complete_support_right_of_current_corridor_return_arc",
            "pass_proven" => false, "swept_path_certified" => false,
            "physical_curvature_calibrated" => false,
        };
        Some(choice)
    }

    /// _begin_second_orbit(scan, now, control).
    fn begin_second_orbit(&mut self, scan: &V, now: f64, control: &V) -> V {
        self.clear_second_return();
        self.second_track_id = self.second_target.idx("track_id").clone();
        let id = self.second_track_id.clone();
        self.second_tracker_mut().enable_close_maintenance(&id);
        self._second_role_anchor = self.second_prior(&self.second_target.clone(), scan);
        self._second_recovering = false;
        self.second_target_handover_observed = true;
        self.second_orbit_since = Some(now);
        self.route_stage = "second_orbit";
        if self.servo <= NEUTRAL && self._route_center_ack.is_some() {
            self.second_center_ack = self._route_center_ack.clone();
        }
        self.steering_target = self.servo;
        self.reason = "second_target_handover_keep_adopted_command".into();
        let e = self.role_distinct_evidence.clone();
        self.route_event("second_handover", scan, control, e);
        self.result(now)
    }

    /// _second_orbit_update(scan, now, control).
    pub fn second_orbit_update(&mut self, scan: &V, now: f64, control: &V) -> V {
        let target = self.second_target.clone();
        let held = self.second_tracker.as_ref().map(|t| t.held.clone()).unwrap_or(V::None);
        if target.is_none()
            && held.is_dict()
            && py_eq(held.get("track_id"), &self.second_track_id)
            && now - self.second_orbit_since.unwrap() < SECOND_ORBIT_MAX_S
        {
            if self.second_return_preparing && !self.route_adopted(control, now) {
                return self.result(now);
            }
            self.steering_target = if self.second_return_preparing { self.servo.min(NEUTRAL) } else { self.servo };
            self.natural_steering_target = Some(self.steering_target);
            self.reason = "second_target_missing_frame_hold".into();
            self.slew(now);
            return self.result(now);
        }
        let lost = target.is_none() || !py_eq(target.get("track_id"), &self.second_track_id) || !target.get("confirmed").truthy();
        if now - self.second_orbit_since.unwrap() >= SECOND_ORBIT_MAX_S || lost {
            self.clear_second_return();
            self.route_stage = "corridor_follow";
            self.right_exit_geometry = V::None;
            self._right_exit_last_receive = V::None;
            self._right_exit_last_publication = V::None;
            if lost {
                self.start_second_identity_recovery();
            } else {
                self.second_tracker = Some(new_second_tracker());
                self._second_recovering = false;
            }
            self.second_target = V::None;
            self.second_center_ack = V::None;
            self.second_target_handover_observed = false;
            self.reason = if !lost { "second_target_entry_timeout_back_to_corridor".into() } else { "second_target_lost_back_to_corridor".into() };
            self.steering_target = self.servo;
            let e = dict! {"source_seq" => scan.idx("seq").clone(), "reason" => self.reason.as_str()};
            self.route_event("second_target_recovery", scan, control, e);
            return self.result(now);
        }
        if !self.route_adopted(control, now) {
            return self.result(now);
        }
        if self.second_center_ack.is_none() && self.servo == NEUTRAL {
            self.second_center_ack = control.pick(&["servo", "motor", "tick", "seq"]);
        }
        let points = self.full_target_points(&target, scan, &self.second_track_id.clone());
        let passed = points.as_ref().is_some_and(|p| {
            p.iter().map(|q| q.0).fold(f64::NEG_INFINITY, pmax) <= -REAR_BODY_EXTENT_M
                && p.iter().map(|q| -q.1).fold(INF, pmin) - SIDE_BODY_EXTENT_M >= MANEUVER_BODY_CLEARANCE_M
        });
        if passed && mature_observation(&mut self._second_pass_history, scan, 0.25) {
            let p = points.as_ref().unwrap();
            self.second_pass_evidence = dict! {
                "source_seq" => scan.idx("seq").clone(), "source_at_ms" => scan.idx("at_ms").clone(),
                "received_at" => scan.idx("received_at").clone(), "track_id" => self.second_track_id.clone(),
                "support_bins" => target.idx("support_bins").clone(),
                "observed_frontmost_x_m" => p.iter().map(|q| q.0).fold(f64::NEG_INFINITY, pmax),
                "observed_right_body_gap_m" => p.iter().map(|q| -q.1).fold(INF, pmin) - SIDE_BODY_EXTENT_M,
                "basis" => "current_complete_support_behind_rear",
                "physical_cone_pass_certified" => false, "swept_path_certified" => false,
            };
            let e = self.second_pass_evidence.clone();
            self.route_event("second_pass", scan, control, e);
            self.route_stage = "light_turn";
            self.light_since = Some(now);
            self.light_turn_started = Some(now);
            self.second_return_preparing = false;
            self._light_history.clear();
            self._light_heading = self._exit_heading;
            self.steering_target = self.servo;
            self.reason = "light_turn_started_keep_adopted_command".into();
            let e = dict! {"source_seq" => scan.idx("seq").clone()};
            self.route_event("light_turn", scan, control, e);
            return self.result(now);
        }
        if !passed {
            self._second_pass_history.clear();
        }
        let Some(points) = points else {
            self.steering_target = if self.second_return_preparing { self.servo.min(NEUTRAL) } else { self.servo };
            self.natural_steering_target = Some(self.steering_target);
            self.reason = "second_target_partial_support_hold".into();
            if self.second_return_preparing {
                self.slew(now);
            }
            return self.result(now);
        };
        let body_length = FRONT_BODY_EXTENT_M + REAR_BODY_EXTENT_M;
        let front = points.iter().map(|p| p.0).fold(f64::NEG_INFINITY, pmax);
        let near = points.iter().map(|p| p.0).fold(INF, pmin);
        let left_edge = points.iter().map(|p| p.1).fold(f64::NEG_INFINITY, pmax);
        let waypoint = (pmax(body_length, near), left_edge + SECOND_BYPASS_SUPPORT_GAP_M);
        let bearing = waypoint.1.atan2(waypoint.0);
        let curvature = div(2. * waypoint.1, sq(waypoint.0) + sq(waypoint.1));
        let mut nominal = clampi(NEUTRAL + round(SECOND_CURVATURE_GAIN_PWM_M * curvature), right_exit_min_pwm(), SERVO_MAX);
        self.second_feedback = dict! {
            "source_seq" => scan.idx("seq").clone(), "source_at_ms" => scan.idx("at_ms").clone(),
            "current_support_left_edge_y_m" => left_edge, "current_support_frontmost_x_m" => front,
            "current_support_nearest_x_m" => near, "bypass_support_gap_m" => SECOND_BYPASS_SUPPORT_GAP_M,
            "relative_bypass_point_left_m" => V::from(waypoint), "bearing_left_rad" => bearing,
            "pursuit_curvature_per_m" => curvature,
            "nominal_target_pwm" => nominal,
            "scope" => "current_support_relative_point_trial",
            "swept_path_certified" => false, "physical_curvature_calibrated" => false,
        };
        let lane = self.second_lane_bypass(scan, now, &points);
        self.second_feedback.set("lane_bypass", lane.clone());
        if lane.is_some() {
            nominal = nominal.max(lane.idx("nominal_target_pwm").i());
            self.second_feedback.set("nominal_target_pwm", nominal);
            self.second_feedback.set("scope", "max_of_support_pursuit_and_lane_frame_bypass");
        }
        if let Some(lane_heading) = self._second_lane_heading
            && nominal > NEUTRAL
        {
            let rate = self._second_lane_rate;
            let (predicted, limit) = self.lane_left_limit(scan, lane_heading, rate);
            self.second_feedback.set("predicted_lane_heading_rad", predicted);
            if nominal > limit {
                nominal = right_exit_min_pwm().max(limit);
                self.second_feedback.set("heading_limited_target_pwm", nominal);
                self.second_feedback.set("nominal_target_pwm", nominal);
            }
        }
        let right_gap = points.iter().map(|p| -p.1).fold(INF, pmin) - SIDE_BODY_EXTENT_M;
        if front <= FRONT_BODY_EXTENT_M && right_gap >= MANEUVER_BODY_CLEARANCE_M {
            let space = Self::free_space(scan, self._exit_heading);
            if let Some(space) = space {
                self._exit_heading = Some(space.idx("best_heading_rad").f());
                let exit_target = Self::free_space_target(&space);
                self.second_feedback.set(
                    "next_lane_free_space",
                    dict! {
                        "best_heading_rad" => space.idx("best_heading_rad").clone(), "free_ahead_m" => space.idx("free_ahead_m").clone(),
                        "target_pwm" => exit_target, "applied" => exit_target < nominal,
                    },
                );
                if exit_target < nominal {
                    nominal = exit_target;
                    self.second_feedback.set("nominal_target_pwm", nominal);
                }
            }
        }
        let was_preparing = self.second_return_preparing;
        let returning = self.second_return_target(scan, now, &points, nominal);
        if let Some(r) = returning {
            nominal = r;
            if !was_preparing {
                let e = self.second_return_evidence.clone();
                self.route_event("second_return_prepare", scan, control, e);
            }
        }
        self.natural_steering_target = Some(nominal);
        if nominal < NEUTRAL && (self.second_center_ack.is_none() || self.servo > NEUTRAL) {
            self.steering_target = NEUTRAL;
            self.reason = "second_target_waiting_center_ack".into();
        } else {
            self.steering_target = nominal;
            self.reason = if nominal > NEUTRAL { "second_target_left_of_support_entry".into() } else { "second_target_clockwise_bounded_entry".into() };
        }
        if self.second_return_preparing {
            self.reason = if nominal < NEUTRAL && self.steering_target == NEUTRAL {
                "second_target_return_waiting_center_ack".into()
            } else if py_eq(self.second_return_evidence.idx("source_seq"), scan.idx("seq")) {
                "second_target_return_current_geometry".into()
            } else {
                "second_target_return_geometry_hold".into()
            };
        }
        self.slew(now);
        self.result(now)
    }

    /// _free_space(scan, previous_heading).
    pub fn free_space(scan: &V, previous_heading: Option<f64>) -> Option<V> {
        let mut ranges: Vec<Option<f64>> = scan.idx("ranges").l().iter().map(|r| r.of()).collect();
        let headings = light_headings_deg();
        let half = SIDE_BODY_EXTENT_M + MANEUVER_BODY_CLEARANCE_M;
        let step = headings[1] - headings[0];
        let mut free = vec![LIGHT_FREE_HORIZON_M; headings.len()];
        let mut left_clear: Option<f64> = None;
        let mut right_clear: Option<f64> = None;
        let usable = ranges.iter().any(|r| r.is_some());
        let reach = headings[headings.len() - 1] + 10;
        let mut start: Option<i64> = None;
        for angle in -reach..reach + 2 {
            let missing = angle <= reach && ranges[imod(-angle, 360) as usize].is_none();
            if missing && start.is_none() {
                start = Some(angle);
            } else if !missing && let Some(s) = start {
                let edges = [ranges[imod(-(s - 1), 360) as usize], ranges[imod(-angle, 360) as usize]];
                let bracketed = angle - s <= LIGHT_BRIDGE_MAX_BINS && edges.iter().all(|e| e.is_some());
                let fill = if bracketed { pmin(edges[0].unwrap(), edges[1].unwrap()) } else { LIGHT_UNKNOWN_BLOCK_M };
                for unknown in s..angle {
                    ranges[imod(-unknown, 360) as usize] = Some(fill);
                }
                start = None;
            }
        }
        FREE.with(|t| {
            for (index, r) in ranges.iter().enumerate() {
                let Some(r) = *r else { continue };
                let bearing = t.bearing_deg[index];
                let (x, y) = (r * t.point_cos[index], r * t.point_sin[index]);
                if -REAR_BODY_EXTENT_M <= x && x <= 1. {
                    if y > SIDE_BODY_EXTENT_M {
                        left_clear = Some(match left_clear {
                            None => y,
                            Some(l) => pmin(l, y),
                        });
                    } else if y < -SIDE_BODY_EXTENT_M {
                        right_clear = Some(match right_clear {
                            None => -y,
                            Some(rc) => pmin(rc, -y),
                        });
                    }
                }
                if r >= LIGHT_FREE_HORIZON_M + half {
                    continue;
                }
                let spread = if r <= half { 90. } else { degrees(asin(half / r)) };
                let first = 0i64.max(ceil((bearing - spread - headings[0] as f64) / step as f64));
                let last = (headings.len() as i64 - 1).min(floor((bearing + spread - headings[0] as f64) / step as f64));
                let (cos_row, sin_row) = (&t.cos[index], &t.sin[index]);
                let mut k = first;
                while k <= last {
                    let ku = k as usize;
                    let along = r * cos_row[ku];
                    if along > 0. && (r * sin_row[ku]).abs() <= half && along < free[ku] {
                        free[ku] = along;
                    }
                    k += 1;
                }
            }
        });
        if !usable {
            return None;
        }
        let deepest = free.iter().cloned().fold(f64::NEG_INFINITY, pmax);
        let mut windows: Vec<Vec<usize>> = Vec::new();
        let mut run: Vec<usize> = Vec::new();
        for (k, f) in free.iter().enumerate() {
            if *f >= 0.9 * deepest {
                run.push(k);
            } else if !run.is_empty() {
                windows.push(std::mem::take(&mut run));
            }
        }
        if !run.is_empty() {
            windows.push(run);
        }
        let mut chosen: Option<Vec<usize>> = None;
        if let Some(prev) = previous_heading {
            let previous = degrees(prev);
            chosen = windows
                .iter()
                .find(|w| {
                    (headings[w[0]] - step) as f64 <= previous && previous <= (headings[*w.last().unwrap()] + step) as f64
                })
                .cloned();
        }
        let chosen = chosen.unwrap_or_else(|| windows[argmax_first(&windows, |w| w.len() as f64).unwrap()].clone());
        let best = radians((headings[chosen[0]] + headings[*chosen.last().unwrap()]) as f64 / 2.);
        let zero = headings.iter().position(|&h| h == 0).unwrap();
        Some(dict! {
            "best_heading_rad" => best, "deepest_free_m" => deepest,
            "free_ahead_m" => free[zero],
            "left_clear_m" => left_clear, "right_clear_m" => right_clear,
            "center_error_m" => match (left_clear, right_clear) { (Some(l), Some(r)) => V::Float((l - r) / 2.), _ => V::None },
        })
    }

    /// _end_light_stage(reason, now, control).
    fn end_light_stage(&mut self, reason: &str, now: f64, control: &V) -> V {
        self.begin_coast(reason, now);
        if self.phase == "coast" && eq_i(control.get("servo"), self.servo) && control.get("command_acked").is_true() {
            self.route_end_coast_servo = Some(self.servo);
            self.steering_target = self.servo;
        }
        self.slew(now);
        self.result(now)
    }

    /// _free_space_target(space).
    pub fn free_space_target(space: &V) -> i64 {
        let best = space.idx("best_heading_rad").f();
        let center = space.idx("center_error_m");
        let center_term = if center.is_some() && best.abs() <= radians(20.) { center.f().atan2(1.5) } else { 0. };
        clampi(NEUTRAL + round(LEFT_TRIAL_GAIN * (best + center_term)), right_exit_min_pwm(), SERVO_MAX)
    }

    /// _lane_front_range(scan).
    pub fn lane_front_range(&self, scan: &V) -> Option<f64> {
        let heading = if self.entry_kind == Some("lane") { self.compass.current(scan) } else { None };
        let list = scan.idx("ranges").l();
        if let Some(heading) = heading {
            let axis = if self.route_stage == "open_transfer" { PI } else { 0. };
            let center = round(degrees(wrap(heading - axis)));
            let mut best: Option<f64> = None;
            for i in center - LANE_FRONT_HALF_BINS..center + LANE_FRONT_HALF_BINS + 1 {
                let r = &list[imod(i, 360) as usize];
                if r.is_num() {
                    let v = r.f() * radians((i - center) as f64).pcos();
                    best = Some(match best {
                        None => v,
                        Some(b) => pmin(b, v),
                    });
                }
            }
            return best;
        }
        let mut best: Option<f64> = None;
        for i in -LANE_FRONT_HALF_BINS..LANE_FRONT_HALF_BINS + 1 {
            let r = &list[imod(i, 360) as usize];
            if r.is_num() {
                best = Some(match best {
                    None => r.f(),
                    Some(b) => pmin(b, r.f()),
                });
            }
        }
        best
    }

    /// _lane_front_track(scan, ahead).
    pub fn lane_front_track(&mut self, scan: &V, ahead: Option<f64>) -> Option<f64> {
        let heading = self.compass.current(scan);
        let previous = self._front_track.take();
        let (Some(heading), Some(ahead)) = (heading, ahead) else { return ahead };
        let center = round(degrees(wrap(heading)));
        let list = scan.idx("ranges").l();
        let mut sector: Vec<(i64, f64)> = Vec::new();
        for j in -LANE_FRONT_HALF_BINS..LANE_FRONT_HALF_BINS + 1 {
            let r = &list[imod(center + j, 360) as usize];
            if r.is_num() {
                sector.push((j, r.f() * (radians((center + j) as f64) - heading).pcos()));
            }
        }
        let mut value = ahead;
        let at_ms = scan.idx("at_ms").i();
        if let Some((pat, psector, pvalue)) = &previous {
            let dt = at_ms - pat;
            if 0 < dt && (dt as f64) <= (SCAN_AGE_S + 0.05) * 1000. {
                let mut deltas: Vec<f64> = Vec::new();
                for (j, v) in &sector {
                    if let Some((_, pv)) = psector.iter().find(|(pj, _)| pj == j) {
                        deltas.push(pv - v);
                    }
                }
                sort_floats(&mut deltas);
                if deltas.len() >= 3 {
                    value = pvalue - (deltas[(deltas.len() - 1) / 2] + deltas[deltas.len() / 2]) / 2.;
                }
            }
        }
        self._front_track = Some((at_ms, sector, value));
        Some(value)
    }

    /// _track_speed(scan, track).
    pub fn track_speed(&mut self, scan: &V, track: Option<f64>) -> Option<f64> {
        let Some(track) = track else {
            self._track_history.clear();
            return None;
        };
        let row = (scan.idx("at_ms").i() as f64 / 1000., scan.idx("received_at").f(), track);
        if let Some(&last) = self._track_history.last() {
            let dt = row.0 - last.0;
            let rate = if dt > 0. { Some((last.2 - track) / dt) } else { None };
            let dr = row.1 - last.1;
            if !(0.05 <= dt && dt <= SCAN_AGE_S + 0.05)
                || !(0.05 <= dr && dr <= SCAN_AGE_S + 0.05)
                || rate.is_none_or(|r| !(-0.4 <= r && r <= LANE_MAX_CLOSING_MPS))
            {
                self._track_history.clear();
            }
        }
        self._track_history.push(row);
        if self._track_history.len() > LANE_TRACK_SPEED_POINTS {
            let n = self._track_history.len();
            self._track_history.drain(..n - LANE_TRACK_SPEED_POINTS);
        }
        if self._track_history.len() < LANE_TRACK_SPEED_POINTS {
            return None;
        }
        let h = &self._track_history;
        let mut slopes = Vec::new();
        for (i, a) in h.iter().enumerate() {
            for b in &h[i + 1..] {
                slopes.push((b.2 - a.2) / (b.0 - a.0));
            }
        }
        sort_floats(&mut slopes);
        let slope = (slopes[(slopes.len() - 1) / 2] + slopes[slopes.len() / 2]) / 2.;
        Some(pmax(0., -slope))
    }

    /// _lane_speed(scan, ahead, min_rates).
    pub fn lane_speed(&mut self, scan: &V, ahead: Option<f64>, min_rates: usize) -> Option<f64> {
        let Some(ahead) = ahead else {
            self._lane_ranges.clear();
            return None;
        };
        let row = (scan.idx("at_ms").i() as f64 / 1000., scan.idx("received_at").f(), ahead);
        if let Some(&last) = self._lane_ranges.last() {
            let dt = row.0 - last.0;
            let rate = if dt > 0. { Some((last.2 - ahead) / dt) } else { None };
            let dr = row.1 - last.1;
            if !(0.05 <= dt && dt <= SCAN_AGE_S + 0.05)
                || !(0.05 <= dr && dr <= SCAN_AGE_S + 0.05)
                || rate.is_none_or(|r| !(-0.4 <= r && r <= LANE_MAX_CLOSING_MPS))
            {
                self._lane_ranges.clear();
            }
        }
        self._lane_ranges.push(row);
        if self._lane_ranges.len() > LANE_SPEED_HISTORY {
            let n = self._lane_ranges.len();
            self._lane_ranges.drain(..n - LANE_SPEED_HISTORY);
        }
        let n = self._lane_ranges.len();
        if n < 3 || n - LANE_SPEED_BASELINE.min(n - 2) < min_rates {
            return None;
        }
        let step = LANE_SPEED_BASELINE.min(n - 2);
        let h = &self._lane_ranges;
        let mut rates: Vec<f64> = h.iter().zip(h[step..].iter()).map(|(a, b)| (a.2 - b.2) / (b.0 - a.0)).collect();
        sort_floats(&mut rates);
        Some(pmax(0., rates[rates.len() / 2]))
    }

    /// _begin_light_lane(scan, now, control, evidence).
    pub fn begin_light_lane(&mut self, scan: &V, now: f64, control: &V, evidence: V) -> V {
        self.light_alignment_evidence = evidence.clone();
        self.route_event("light_aligned", scan, control, evidence);
        self.route_stage = "light_lane";
        self.lane_motor = ROUTE_SLOW_MOTOR;
        self._lane_steering = CorridorSteering::default();
        self._lane_pulse = 0;
        let e = dict! {"source_seq" => scan.idx("seq").clone()};
        self.route_event("light_lane", scan, control, e);
        self.steering_target = self.servo;
        self.reason = "light_lane_started_keep_adopted_command".into();
        self.result(now)
    }

    /// _start_allowed_speed(ahead).
    pub fn start_allowed_speed(ahead: Option<f64>) -> Option<f64> {
        let ahead = ahead?;
        let room = ahead - FRONT_BODY_EXTENT_M - LANE_FINISH_GAP_M - LANE_START_SEARCH_M;
        if room <= 0. {
            return Some(0.);
        }
        let (a, r) = (LANE_START_STOP_DECEL_MPS2, LANE_REACTION_S);
        Some(pmin(LANE_START_MAX_MPS, -r * a + sqrt(sq(r * a) + 2. * a * room)))
    }

    /// _start_governor(speed, rise, allowed, measured_before, settling).
    pub fn start_governor(speed: Option<f64>, rise: f64, allowed: Option<f64>, measured_before: bool, settling: bool) -> i64 {
        let cap = allowed.unwrap_or(LANE_START_MAX_MPS);
        let launch_rise = START_LAUNCH_ACCEL_MPS2 * ESC_LAG_S;
        let Some(speed) = speed else {
            if cap < LANE_SPEED_UNKNOWN_MPS * 0.6 {
                return NEUTRAL;
            }
            return if measured_before { ROUTE_SLOW_MOTOR } else { TRIAL_MOTOR };
        };
        if speed + rise >= cap {
            return NEUTRAL;
        }
        if settling {
            return ROUTE_SLOW_MOTOR;
        }
        if speed < START_LAUNCH_MAX_MPS && speed + launch_rise < cap {
            return START_LAUNCH_MOTOR;
        }
        if speed < LANE_SPEED_RESTART_MPS || speed + rise < cap - LANE_START_PUSH_BAND_MPS {
            return TRIAL_MOTOR;
        }
        ROUTE_SLOW_MOTOR
    }

    /// _reaction_s(now).
    pub fn reaction_s(&self, now: f64) -> f64 {
        match self._power_cmd_at {
            None => LANE_REACTION_BASE_S,
            Some(at) => LANE_REACTION_BASE_S + pmax(0., pmin(ESC_LAG_S, ESC_LAG_S - (now - at))),
        }
    }

    /// _lane_decels(): (aim, guard, strong).
    fn lane_decels(&self) -> (f64, f64, Option<f64>) {
        let Some(decel) = self._lane_coast_decel else {
            return (LANE_STOP_DECEL_MPS2, LANE_STOP_DECEL_MIN_MPS2, None);
        };
        let se = self._lane_coast_decel_se.unwrap();
        (
            decel,
            pmax(LANE_COAST_DECEL_FLOOR_MPS2, pmin(LANE_COAST_GUARD_RATIO * decel, decel - 2. * se)),
            Some(pmax(decel / LANE_COAST_GUARD_RATIO, decel + 2. * se)),
        )
    }

    /// _lane_pipeline(now, onset, offset, push_s): (coast_s, power_s).
    fn lane_pipeline(&self, now: f64, onset: f64, offset: f64, push_s: f64) -> (f64, f64) {
        let (mut since, mut last) = (self._power_since, self._power_cmd_at);
        if last.is_some() && since.is_none() {
            since = last;
        }
        if push_s > 0. {
            if last.is_none_or(|l| l + offset <= now) {
                since = Some(now);
            }
            last = Some(now + push_s);
        }
        let Some(last) = last else {
            return (0., 0.);
        };
        let (start, end) = (since.unwrap() + onset, last + offset);
        if end <= now || end <= start {
            return (0., 0.);
        }
        (pmax(0., start - now), end - pmax(now, start))
    }

    /// _lane_speed_now(speed, now, onset, offset, decel, accel).
    fn lane_speed_now(&self, speed: f64, now: f64, onset: f64, offset: f64, decel: f64, accel: f64) -> f64 {
        let history = &self._lane_ranges;
        let lag = if history.is_empty() { 0. } else { (history[history.len() - 1].0 - history[0].0) / 2. };
        let mut powered_s = 0.;
        if let Some(cmd_at) = self._power_cmd_at {
            let since = self._power_since.unwrap_or(cmd_at);
            powered_s = pmax(0., pmin(now, cmd_at + offset) - pmax(now - lag, since + onset));
            if let Some(prev_end) = self._power_prev_end {
                powered_s += pmax(0., pmin(pmin(now, prev_end + offset), since + onset) - (now - lag));
            }
        }
        powered_s = pmin(lag, powered_s);
        pmax(0., speed + accel * powered_s - decel * (lag - powered_s))
    }

    /// _lane_worst_gap(gap, speed, now, decel, accel, reaction, push_s).
    #[allow(clippy::too_many_arguments)]
    fn lane_worst_gap(&self, gap: f64, speed: f64, now: f64, decel: f64, accel: f64, reaction: f64, push_s: f64) -> f64 {
        let mut worst: Option<f64> = None;
        for lag in [LANE_ESC_LAG_MIN_S, ESC_LAG_S] {
            let (onset, offset) = (lag - LANE_ESC_SKEW_S, lag + LANE_ESC_SKEW_S);
            let v = self.lane_speed_now(speed, now, onset, offset, decel, accel) + LANE_SPEED_GUARD_MARGIN_MPS;
            let (coast_s, power_s) = self.lane_pipeline(now, onset, offset, push_s);
            let value = Self::lane_stop(gap, v, decel, reaction, coast_s, power_s, accel);
            worst = Some(match worst {
                None => value,
                Some(w) => pmin(w, value),
            });
        }
        worst.unwrap()
    }

    /// _lane_stop(gap, speed, decel, reaction, coast_s, power_s, accel).
    fn lane_stop(gap: f64, speed: f64, decel: f64, reaction: f64, coast_s: f64, power_s: f64, accel: f64) -> f64 {
        let travel = speed * reaction;
        if speed <= decel * coast_s {
            return gap - travel - speed * speed / (2. * decel);
        }
        let coasted = speed - decel * coast_s;
        let peak = pmin(coasted + accel * power_s, pmax(coasted, LANE_PUSH_MAX_MPS));
        gap - travel - (speed + coasted) / 2. * coast_s - peak * power_s - peak * peak / (2. * decel)
    }

    /// _lane_coast_observe(now, speed).
    fn lane_coast_observe(&mut self, now: f64, speed: Option<f64>) {
        let n = self._lane_ranges.len();
        let coasting = self.lane_motor == NEUTRAL
            && speed.is_some()
            && n >= 3
            && self._power_cmd_at.is_none_or(|at| self._lane_ranges[n.saturating_sub(5)].1 - at >= ESC_LAG_S);
        if !coasting {
            self._lane_coast_track.clear();
            return;
        }
        let speed = speed.unwrap();
        if speed < LANE_SPEED_RESTART_MPS {
            return;
        }
        if let Some(&last) = self._lane_coast_track.last()
            && (now <= last.0 || speed > last.1 + LANE_SPEED_CUT_MARGIN_MPS)
        {
            self._lane_coast_track.clear();
        }
        self._lane_coast_track.push((now, speed));
        if self._lane_coast_track.len() > LANE_COAST_TRACK_POINTS {
            let m = self._lane_coast_track.len();
            self._lane_coast_track.drain(..m - LANE_COAST_TRACK_POINTS);
        }
        let track = &self._lane_coast_track;
        let (t0, v0) = track[0];
        let (t1, v1) = track[track.len() - 1];
        if t1 - t0 >= LANE_COAST_MIN_SPAN_S && v0 - v1 >= LANE_COAST_MIN_DROP_MPS && track.len() >= 4 {
            let count = track.len() as f64;
            let mean_t = fsum_py(track.iter().map(|p| p.0)) / count;
            let mean_v = fsum_py(track.iter().map(|p| p.1)) / count;
            let sxx = fsum_py(track.iter().map(|p| (p.0 - mean_t) * (p.0 - mean_t)));
            let slope = fsum_py(track.iter().map(|p| (p.0 - mean_t) * (p.1 - mean_v))) / sxx;
            let residual = fsum_py(track.iter().map(|p| (p.1 - mean_v - slope * (p.0 - mean_t)) * (p.1 - mean_v - slope * (p.0 - mean_t))));
            let se = sqrt(residual / (count - 2.) / sxx);
            if se <= LANE_COAST_DECEL_SE_MAX_MPS2 {
                self._lane_coast_decel = Some(pmax(LANE_COAST_DECEL_FLOOR_MPS2, pmin(LANE_COAST_DECEL_CAP_MPS2, -slope)));
                self._lane_coast_decel_se = Some(se);
            }
        }
    }

    /// _light_lane_update(scan, now, control).
    pub fn light_lane_update(&mut self, scan: &V, now: f64, control: &V) -> V {
        if !self.route_adopted(control, now) {
            return self.result(now);
        }
        let (_, lane) = self.current_lane(scan, now);
        let mut space = V::None;
        let walls = corridor_walls(scan);
        let heading = if self.entry_kind == Some("lane") { self.compass.current(scan) } else { None };
        let open_pursuit = match heading {
            None => None,
            Some(h) => {
                let pose = self.response_pose();
                self.open_light_pursuit(scan, h, Some(pose), None)
            }
        };
        let (natural, source): (i64, &str);
        if let Some((_, _, _, pwm)) = open_pursuit {
            natural = pwm;
            source = "open_lane_axis_pursuit";
        } else if walls.is_some() {
            self._lane_steering.servo = clampi(self.servo, NEUTRAL - 55, NEUTRAL + 55);
            natural = self._lane_steering.update(&walls, scan.idx("seq"), now, true, scan.idx("at_ms"));
            source = "straight_lane_walls";
        } else if let Some(l) = lane.as_ref().filter(|l| l.idx("heading_left_rad").f().abs() <= second_lane_max_heading_rad()) {
            let lookahead = pmax(FRONT_BODY_EXTENT_M + REAR_BODY_EXTENT_M, pmin(ROUTE_ALIGN_LOOKAHEAD_MAX_M, l.idx("width_m").f()));
            let error = l.idx("heading_left_rad").f() + l.idx("center_offset_left_m").f().atan2(lookahead);
            natural = clampi(NEUTRAL + round(LEFT_TRIAL_GAIN * error), NEUTRAL - 55, NEUTRAL + 55);
            source = "measured_lane";
        } else {
            let Some(s) = Self::free_space(scan, self._light_heading) else {
                self.begin_recovery_hold("light_lane_geometry_unavailable", now, control);
                return self.result(now);
            };
            self._light_heading = Some(s.idx("best_heading_rad").f());
            natural = Self::free_space_target(&s);
            space = s;
            source = "free_space";
        }
        let mut target = natural;
        if target < NEUTRAL && (self._route_center_ack.is_none() || self.servo > NEUTRAL) {
            target = NEUTRAL;
        }
        let ahead = self.lane_front_range(scan);
        let speed = self.lane_speed(scan, ahead, 2);
        self.lane_coast_observe(now, speed);
        let held = self._lane_speed_last;
        let mut speed_known = true;
        let v = if let Some(s) = speed {
            self._lane_speed_last = Some((now, s));
            s
        } else if let Some(h) = held.filter(|h| 0. <= now - h.0 && now - h.0 <= LANE_SPEED_HOLD_S) {
            h.1 + LANE_SPEED_HOLD_MARGIN_MPS
        } else {
            speed_known = false;
            LANE_SPEED_UNKNOWN_MPS
        };
        let gap = ahead.map(|a| a - FRONT_BODY_EXTENT_M);
        let (decel, guard_decel, strong_decel) = self.lane_decels();
        let powered = DRIVE_MOTORS.contains(&self.lane_motor);
        let (aim_coast, aim_power) = self.lane_pipeline(now, LANE_ESC_LAG_NOMINAL_S, LANE_ESC_LAG_NOMINAL_S, 0.);
        let coasting = !powered && self._power_cmd_at.is_none_or(|at| now >= at + ESC_LAG_S + LANE_ESC_SKEW_S);
        let kick = self._power_cmd_motor == Some(TRIAL_MOTOR);
        let aim_accel = if kick { LANE_POWER_ACCEL_MPS2 } else { LANE_PUSH_ACCEL_MPS2 };
        let late = gap.is_some_and(|g| g <= LANE_FINISH_ZONE_GAP_M);
        let guard_accel = if kick {
            LANE_KICK_ACCEL_MAX_MPS2
        } else if late {
            LANE_PUSH_LATE_ACCEL_MAX_MPS2
        } else {
            LANE_PUSH_ACCEL_MAX_MPS2
        };
        let mut v_aim = v;
        if speed.is_some() {
            v_aim = self.lane_speed_now(v, now, LANE_ESC_LAG_NOMINAL_S, LANE_ESC_LAG_NOMINAL_S, decel, aim_accel);
        }
        let guard_reaction = if powered { LANE_REACTION_BASE_S } else { LANE_AIM_REACTION_S };
        let (mut stop_gap, mut worst_gap) = (None, None);
        if let Some(g) = gap {
            stop_gap = Some(Self::lane_stop(g, v_aim, decel, if powered { LANE_AIM_REACTION_S } else { 0. }, aim_coast, aim_power, aim_accel));
            worst_gap = Some(self.lane_worst_gap(g, v, now, guard_decel, guard_accel, guard_reaction, 0.));
        }
        let stopped = speed.is_some_and(|s| s < LANE_SPEED_RESTART_MPS);
        let in_zone = gap.is_some_and(|g| g <= LANE_FINISH_ZONE_GAP_M);
        let body_in_zone = gap.is_some_and(|g| g <= LANE_FINISH_BODY_GAP_M);
        let kick_room = gap.is_some_and(|g| g >= LANE_FINISH_MIN_GAP_M + LANE_KICK_TRAVEL_M);
        let rested = self._power_cmd_at.is_none_or(|at| now - at >= LANE_KICK_REST_S);
        let mut basis: Option<&str> = None;
        if speed_known && stopped && body_in_zone {
            basis = Some("stopped_with_the_whole_car_in_the_finish_zone");
        } else if speed_known && stopped && in_zone && !kick_room {
            basis = Some("stopped_front_in_finish_zone_restart_could_reach_the_board");
        } else if speed_known
            && coasting
            && !stopped
            && strong_decel.is_some()
            && gap.is_some()
            && stop_gap.unwrap() >= LANE_FINISH_MIN_GAP_M
            && Self::lane_stop(gap.unwrap(), v_aim, strong_decel.unwrap(), 0., 0., 0., 0.) <= LANE_FINISH_BODY_GAP_M
        {
            basis = Some("coast_stops_the_whole_car_in_the_finish_zone");
        }
        self.lane_feedback = dict! {
            "source_seq" => scan.idx("seq").clone(), "steering_source" => source,
            "nominal_target_pwm" => natural, "lane" => lane.clone(), "free_space" => space.clone(), "walls" => walls,
            "front_range_m" => ahead, "front_gap_m" => gap, "speed_mps" => speed, "speed_for_cut_mps" => v,
            "speed_aim_mps" => v_aim, "speed_known" => speed_known, "speed_basis" => "closing_rate_on_board_ahead_axis",
            "predicted_stop_gap_m" => stop_gap, "worst_stop_gap_m" => worst_gap, "in_finish_zone" => in_zone,
            "whole_car_in_finish_zone" => body_in_zone, "pipeline_power_s" => aim_power,
            "coast_decel_mps2" => decel, "guard_decel_mps2" => guard_decel, "strong_decel_mps2" => strong_decel,
            "coast_decel_se_mps2" => self._lane_coast_decel_se,
            "coast_decel_measured" => self._lane_coast_decel.is_some(),
            "motor_before" => self.lane_motor, "swept_path_certified" => false, "traffic_light_vision" => false,
        };
        if let Some(basis) = basis {
            let mut finish = self.lane_feedback.clone();
            finish.update(&dict! {
                "received_at" => scan.idx("received_at").clone(),
                "basis" => basis,
                "finish_zone_certified" => false,
            });
            self.finish_evidence = finish;
            let e = self.finish_evidence.clone();
            self.route_event("finish_stop", scan, control, e);
            return self.end_light_stage("route_finish_zone_stop", now, control);
        }
        if gap.is_some() && (stop_gap.unwrap() <= LANE_FINISH_TARGET_GAP_M || worst_gap.unwrap() <= LANE_FINISH_MIN_GAP_M) {
            self.lane_motor = NEUTRAL;
        } else if speed.is_none() {
            self.lane_motor = ROUTE_SLOW_MOTOR;
        } else if stopped {
            self.lane_motor = if kick_room && (rested || self._lane_pulse > 0) { TRIAL_MOTOR } else { NEUTRAL };
        } else if self.lane_motor == ROUTE_SLOW_MOTOR {
        } else if self.lane_motor == NEUTRAL
            && stop_gap.unwrap() > LANE_FINISH_TARGET_GAP_M + LANE_TOPUP_BAND_M
            && v_aim - decel * LANE_ESC_LAG_NOMINAL_S >= LANE_PUSH_ROLLING_MPS
            && self.lane_worst_gap(gap.unwrap(), v, now, guard_decel, guard_accel, guard_reaction, LANE_PUSH_MIN_S) > LANE_FINISH_MIN_GAP_M
        {
            self.lane_motor = ROUTE_SLOW_MOTOR;
        } else {
            self.lane_motor = NEUTRAL;
        }
        if self.lane_motor == TRIAL_MOTOR && self._lane_pulse >= LANE_KICK_FRAMES {
            self.lane_motor = NEUTRAL;
            self._lane_pulse = 0;
        } else if self.lane_motor == TRIAL_MOTOR {
            self._lane_pulse += 1;
        } else {
            self._lane_pulse = 0;
        }
        self.lane_feedback.set("motor", self.lane_motor);
        self.steering_target = target;
        self.natural_steering_target = Some(natural);
        self.reason = "light_lane_follow_straight".into();
        self.slew(now);
        self.result(now)
    }

    /// _light_update(scan, now, control).
    pub fn light_update(&mut self, scan: &V, now: f64, control: &V) -> V {
        if now - self.light_since.unwrap() + 1e-9 >= LIGHT_STAGE_MAX_S {
            self.begin_recovery_hold("light_stage_timeout", now, control);
            return self.result(now);
        }
        if !self.route_adopted(control, now) {
            return self.result(now);
        }
        let Some(space) = Self::free_space(scan, self._light_heading) else {
            self.begin_recovery_hold("light_free_space_unavailable", now, control);
            return self.result(now);
        };
        let best = space.idx("best_heading_rad").f();
        self._light_heading = Some(best);
        let center = space.idx("center_error_m").clone();
        let (_, lane) = self.current_lane(scan, now);
        let turned = self.light_turn_started.is_some_and(|s| now - s + 1e-9 >= LIGHT_TURN_MIN_S);
        let lane_aligned = turned
            && lane.as_ref().is_some_and(|l| {
                l.idx("heading_left_rad").f().abs() <= light_lane_entry_heading_rad()
                    && Self::corridor_body_gap(l) >= MANEUVER_BODY_CLEARANCE_M
            });
        let aligned = lane_aligned
            || (best.abs() <= light_aligned_heading_rad()
                && space.idx("free_ahead_m").f() >= LIGHT_ALIGNED_FREE_M
                && center.is_some()
                && center.f().abs() <= LIGHT_CENTER_TOLERANCE_M);
        if aligned {
            let ahead = self.lane_front_range(scan);
            self.lane_speed(scan, ahead, 2);
        } else {
            self._lane_ranges.clear();
        }
        if aligned && mature_observation(&mut self._light_history, scan, 0.20) {
            let mut evidence = space.clone();
            evidence.update(&dict! {
                "source_seq" => scan.idx("seq").clone(), "source_at_ms" => scan.idx("at_ms").clone(),
                "received_at" => scan.idx("received_at").clone(), "frames" => self._light_history.len(),
                "measured_lane" => lane.clone(),
                "basis" => if lane_aligned { "measured_lane_aligned" } else { "long_free_lane_ahead_and_balanced_side_clearance" },
                "camera_facing_certified" => false,
            });
            return self.begin_light_lane(scan, now, control, evidence);
        }
        if !aligned {
            self._light_history.clear();
        }
        let mut target = Self::free_space_target(&space);
        let natural = target;
        if target < NEUTRAL && (self._route_center_ack.is_none() || self.servo > NEUTRAL) {
            target = NEUTRAL;
        }
        self.steering_target = target;
        self.natural_steering_target = Some(natural);
        let mut feedback = space.clone();
        feedback.update(&dict! {
            "source_seq" => scan.idx("seq").clone(), "nominal_target_pwm" => natural,
            "scope" => "free_space_follow_trial", "swept_path_certified" => false,
        });
        self.light_feedback = feedback;
        self.reason = "light_turn_follow_free_space".into();
        self.slew(now);
        self.result(now)
    }
}

fn p_y(target: &V) -> f64 {
    target.idx("point_left_m").l()[1].f()
}
