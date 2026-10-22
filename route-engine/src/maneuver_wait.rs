//! Route waits and pauses of ManeuverSequence: the first-target quality
//! wait, rebinding, neutral pauses and their resume rules.

use crate::compact_target::CandidateArgs;
use crate::dict;
use crate::helpers::*;
use crate::maneuver::*;
use crate::maneuver_tm::{consecutive_fresh, mature_observation};
use crate::py::*;

impl Maneuver {
    /// In-place change of recovery_evidence. Python's _enter_neutral_pause
    /// stores ONE dict as recovery_evidence and as the left-turn or post-pass
    /// pause evidence, so the change shows in both while neither has been
    /// reassigned. They are assigned together only in enter_neutral_pause, so
    /// a shared value here is exactly Python's shared object.
    fn update_recovery_evidence(&mut self, change: &V) {
        let left = is_same_object(&self.recovery_evidence, &self.left_turn_pause_evidence);
        let post = is_same_object(&self.recovery_evidence, &self.post_pass_pause_evidence);
        self.recovery_evidence.update(change);
        if left {
            self.left_turn_pause_evidence = self.recovery_evidence.clone();
        }
        if post {
            self.post_pass_pause_evidence = self.recovery_evidence.clone();
        }
    }

    /// begin_quality_wait(now, control, scan=None).
    pub fn begin_quality_wait(&mut self, now: &V, control: &V, scan: Option<&V>) -> bool {
        let scan_v = match scan {
            Some(s) => s.clone(),
            None => V::None,
        };
        let refuse = |me: &mut Maneuver, reason: &str| -> bool {
            me.quality_wait_refusal = dict! {
                "reason" => reason, "at" => if now.is_num() { now.clone() } else { V::None },
                "control_servo" => control.get("servo").clone(), "command_acked" => control.get("command_acked").clone(),
                "servo" => me.servo, "scan_seq" => if scan_v.is_dict() { scan_v.get("seq").clone() } else { V::None },
            };
            false
        };
        if !self.continue_route
            || self.phase != "drive"
            || !self.handover_observed
            || self.first_pass_evidence.is_some()
            || self.right_exit_since.is_some()
            || self.route_stage != "first_target"
            || !now.is_num()
            || now.f() < self.last_now
        {
            return refuse(self, "route_state");
        }
        let servo_v = control.get("servo");
        let servo_ok = {
            let s = if servo_v.is_none() { V::Int(0) } else { servo_v.clone() };
            s.is_real() && (NEUTRAL as f64) < s.f() && py_eq(&s, &V::Int(self.servo)) && self.servo <= SERVO_MAX
        };
        if !control.get("armed").is_true()
            || !motor_in(control.get("motor"), &DRIVE_MOTORS)
            || !control.get("command_acked").is_true()
            || !servo_ok
        {
            return refuse(self, "control_not_acked_left");
        }
        let scan = if scan.is_none() { self._current_scan.clone() } else { scan_v.clone() };
        if !scan.is_dict() || !scan.get("received_at").is_num() || !(0. ..SCAN_AGE_S).contains(&(now.f() - scan.idx("received_at").f())) {
            return refuse(self, "scan_stale");
        }
        let now = now.f();
        let previous = self.compact_target.clone();
        let observed = self.target_tracker.update(&scan, now, true, None, None);
        let anchor = if self.full_target_points(&observed, &scan, &self.orbit_track_id.clone()).is_some() {
            dict! {
                "track_id" => observed.idx("track_id").clone(), "point_left_m" => observed.idx("point_left_m").clone(),
                "received_at" => scan.idx("received_at").clone(), "basis" => "trigger_frame_full_isolation",
            }
        } else {
            let last = if previous.is_dict() && previous.get("confirmed").is_true() {
                previous.clone()
            } else {
                self._last_confirmed_compact_target.clone()
            };
            if last.is_dict()
                && py_eq(last.get("track_id"), &self.orbit_track_id)
                && last.get("last_full_isolation_received_at").is_num()
                && (0. ..SCAN_AGE_S).contains(&(now - last.idx("last_full_isolation_received_at").f()))
            {
                dict! {
                    "track_id" => last.idx("track_id").clone(), "point_left_m" => last.idx("point_left_m").clone(),
                    "received_at" => last.idx("last_full_isolation_received_at").clone(),
                    "basis" => "previous_fresh_full_isolation",
                }
            } else {
                V::None
            }
        };
        self.compact_target = observed;
        self.neutral_pause_kind = None;
        self.recovery_count += 1;
        self.quality_wait_anchor = anchor;
        self.quality_wait_rebind = V::None;
        self._wait_rebind_candidate = V::None;
        self._current_scan = scan.clone();
        self.quality_wait_used = true;
        self.quality_wait_since = Some(now);
        self.quality_wait_servo = Some(self.servo);
        self.quality_wait_evidence = dict! {
            "entry_seq" => scan.idx("seq").clone(), "entry_at_ms" => scan.idx("at_ms").clone(),
            "entry_received_at" => scan.idx("received_at").clone(), "held_servo" => self.servo,
            "target_id" => self.orbit_track_id.clone(), "maximum_resumes" => 1,
        };
        self._quality_good_history.clear();
        self._quality_pass_history.clear();
        self._first_pass_history.clear();
        self.clear_exit_prepare_window();
        self._wait_release_history.clear();
        self.quality_wait_previous_servo = None;
        self.quality_wait_release = if self.first_pass_preparing {
            dict! {
                "basis" => "release_latched_before_neutral_wait",
                "source_seq" => if self.first_exit_prepare_evidence.is_dict() { self.first_exit_prepare_evidence.get("source_seq").clone() } else { V::None },
                "held_servo_at_entry" => self.servo,
            }
        } else {
            V::None
        };
        self.first_pass_preparing = false;
        self.phase = "quality_wait";
        self.reason = "route_quality_wait_neutral".into();
        self.steering_target = self.servo;
        let evidence = self.quality_wait_evidence.clone();
        self.route_event("quality_wait", &scan, control, evidence);
        true
    }

    /// _nearest_left_rebind(scan, control).
    fn nearest_left_rebind(&mut self, scan: &V, control: &V) -> Option<Vec<(f64, f64)>> {
        let target = self.compact_target.clone();
        if self.route_stage != "first_target"
            || self.first_pass_evidence.is_some()
            || !target.is_dict()
            || !target.get("confirmed").is_true()
            || target.get("track_id").is_none()
            || py_eq(target.get("track_id"), &self.orbit_track_id)
            || !target.get("bearing_left_rad").is_num()
            || !target.get("range_m").is_num()
            || !(0. < target.idx("bearing_left_rad").f() && target.idx("bearing_left_rad").f() < PI)
            || target.idx("range_m").f() > ORBIT_REBIND_MAX_RANGE_M
        {
            return None;
        }
        let points = self.full_target_points(&target, scan, target.idx("track_id"))?;
        let others = except(VTK, || {
            crate::compact_target::candidates(
                scan.idx("ranges"),
                CandidateArgs { bearing_range_rad: (0., PI), tolerant: true, ..CandidateArgs::default() },
            )
        });
        let Ok(others) = others else { return None };
        let range = target.idx("range_m").f();
        if others.iter().any(|o| o.get("range_m").is_num() && o.idx("range_m").f() < range - 0.05) {
            return None;
        }
        self.quality_wait_rebind = dict! {
            "old_track_id" => self.orbit_track_id.clone(), "new_track_id" => target.idx("track_id").clone(),
            "source_seq" => scan.idx("seq").clone(), "anchor" => self.quality_wait_anchor.clone(),
            "range_m" => target.idx("range_m").clone(), "bearing_left_rad" => target.idx("bearing_left_rad").clone(),
            "basis" => "nearest_confirmed_full_compact_on_left_within_1.2m_during_first_target_wait",
            "physical_identity_verified" => false,
        };
        self.orbit_track_id = target.idx("track_id").clone();
        let id = self.orbit_track_id.clone();
        self.target_tracker.enable_close_maintenance(&id);
        self.clear_target_recovery_history();
        let e = self.quality_wait_rebind.clone();
        self.route_event("quality_wait_target_rebind", scan, control, e);
        Some(points)
    }

    /// _quality_wait_rebind(scan, control).
    fn quality_wait_rebind_fn(&mut self, scan: &V, control: &V) -> Option<Vec<(f64, f64)>> {
        let target = self.compact_target.clone();
        let anchor = self.quality_wait_anchor.clone();
        if !target.is_dict()
            || py_eq(target.get("track_id"), &self.orbit_track_id)
            || !target.get("point_left_m").is_list()
            || !target.get("source_received_at").is_num()
        {
            return None;
        }
        if anchor.is_none() {
            return self.nearest_left_rebind(scan, control);
        }
        if eq_i(target.get("confirmation_count"), 1) {
            let tp = target.idx("point_left_m").l();
            let ap = anchor.idx("point_left_m").l();
            if dist((tp[0].f(), tp[1].f()), (ap[0].f(), ap[1].f())) <= QUALITY_WAIT_REBIND_M
                && (0. ..QUALITY_WAIT_REBIND_WINDOW_S)
                    .contains(&(target.idx("source_received_at").f() - anchor.idx("received_at").f()))
            {
                self._wait_rebind_candidate = target.idx("track_id").clone();
            }
            return None;
        }
        if !target.get("confirmed").is_true() || !py_eq(target.idx("track_id"), &self._wait_rebind_candidate) {
            return self.nearest_left_rebind(scan, control);
        }
        let points = self.full_target_points(&target, scan, target.idx("track_id"))?;
        self.quality_wait_rebind = dict! {
            "old_track_id" => self.orbit_track_id.clone(), "new_track_id" => target.idx("track_id").clone(),
            "source_seq" => scan.idx("seq").clone(), "anchor" => anchor,
            "basis" => "new_identity_first_seen_within_association_gate_of_last_full_orbit_support",
            "physical_identity_verified" => false,
        };
        self.orbit_track_id = target.idx("track_id").clone();
        let id = self.orbit_track_id.clone();
        self.target_tracker.enable_close_maintenance(&id);
        self.clear_target_recovery_history();
        let e = self.quality_wait_rebind.clone();
        self.route_event("quality_wait_target_rebind", scan, control, e);
        Some(points)
    }

    /// _wait_steer(now, servo_feedback, neutral, base).
    pub fn wait_steer(&mut self, now: f64, servo_feedback: &V, neutral: bool, base: i64) {
        self.steering_target = self.quality_wait_servo.unwrap();
        if !neutral || !py_eq(servo_feedback, &V::from(self.quality_wait_servo)) || self.quality_wait_previous_servo.is_some() {
            return;
        }
        let lower = if matches!(self.neutral_pause_kind, Some("post_pass") | Some("recovery")) { right_exit_min_pwm() } else { NEUTRAL };
        let before = self.servo;
        self.steering_target = clampi(base, lower, SERVO_MAX);
        self.slew(now);
        if !(lower <= self.servo && self.servo <= SERVO_MAX) {
            self.servo = before;
        }
        if self.servo != before {
            self.quality_wait_previous_servo = Some(before);
            self.quality_wait_servo = Some(self.servo);
            self.wait_steering_changes += 1;
        }
        self.steering_target = self.quality_wait_servo.unwrap();
    }

    /// _exit_release_target(scan, now).
    pub fn exit_release_target(&mut self, scan: &V, now: f64) -> i64 {
        if !self.continue_route || !scan.is_dict() {
            return NEUTRAL;
        }
        if let Some((seq, value)) = &self._exit_release_cache
            && py_eq(seq, scan.get("seq"))
        {
            return NEUTRAL.max(self.servo.min(*value));
        }
        let saved = (
            self.right_exit_geometry.clone(),
            self._right_exit_last_receive.clone(),
            self._right_exit_last_publication.clone(),
        );
        let (target, rejection) = self.right_exit_target(scan, now);
        self.right_exit_geometry = saved.0;
        self._right_exit_last_receive = saved.1;
        self._right_exit_last_publication = saved.2;
        let value = match (rejection, target) {
            (None, Some(t)) => NEUTRAL.max(t),
            _ => NEUTRAL,
        };
        self._exit_release_cache = Some((scan.get("seq").clone(), value));
        NEUTRAL.max(self.servo.min(value))
    }

    /// _wait_inside_base(lookahead).
    fn wait_inside_base(&mut self, lookahead: f64) -> i64 {
        let held = self.quality_wait_servo.unwrap();
        self.limit_left_for_known_points(held, lookahead);
        let demand = if self.inner_clearance.is_some() {
            self.inner_clearance.get("clearance_bearing_demand_pwm").clone()
        } else {
            V::None
        };
        if !demand.is_num() {
            return held;
        }
        held.min(round(demand.f()) + ROUTE_WAIT_INSIDE_MARGIN_PWM)
    }

    /// _quality_wait_update(scan, now, control, advancing, quality_clear, resume_ready).
    pub fn quality_wait_update(&mut self, scan: &V, now: f64, control: &V, advancing: bool, quality_clear: &V, resume_ready: &V) -> V {
        self.steering_target = self.quality_wait_servo.unwrap();
        if now - self.drive_since.unwrap() >= self.max_drive_s {
            self.lock("route_quality_wait_original_budget_expired");
            return self.result(now);
        }
        let servo_feedback = control.get("servo").clone();
        if py_eq(&servo_feedback, &V::from(self.quality_wait_servo)) {
            self.quality_wait_previous_servo = None;
        }
        let pending_release = self.quality_wait_previous_servo.is_some()
            && py_eq(&servo_feedback, &V::from(self.quality_wait_previous_servo))
            && control.get("command_acked").is_false();
        let mut motors = vec![NEUTRAL];
        motors.extend_from_slice(&DRIVE_MOTORS);
        if (!py_eq(&servo_feedback, &V::from(self.quality_wait_servo)) && !pending_release)
            || !motor_in(control.get("motor"), &motors)
            || (control.get("neutral_acked").is_true() && !eq_i(control.idx("motor"), NEUTRAL))
        {
            self.lock("route_quality_wait_feedback_changed");
            return self.result(now);
        }
        let neutral = control.get("neutral_acked").is_true() && eq_i(control.idx("motor"), NEUTRAL);
        if !neutral && now - self.quality_wait_since.unwrap() >= SCAN_AGE_S {
            self.lock("route_quality_wait_neutral_ack_timeout");
            return self.result(now);
        }
        let mut points = self.full_target_points(&self.compact_target.clone(), scan, &self.orbit_track_id.clone());
        if points.is_none() {
            points = self.quality_wait_rebind_fn(scan, control);
        }
        let Some(points) = points else {
            self.clear_exit_prepare_window();
            self._quality_full_streak = 0;
            let held = self.target_tracker.held.clone();
            if !(quality_clear.is_true() && neutral && Self::target_gap_held(&held, &self.orbit_track_id, scan, now)) {
                self._quality_good_history.clear();
                self._quality_pass_history.clear();
            }
            let anchor = self.quality_wait_anchor.clone();
            let anchored_front = anchor.is_dict()
                && anchor.get("point_left_m").is_list()
                && anchor.idx("point_left_m").at(0).f() <= FRONT_BODY_EXTENT_M;
            let base = if (self.quality_wait_release.is_some() || anchored_front) && !self.open_loop_orbit() {
                self.exit_release_target(scan, now)
            } else {
                self.wait_inside_base(1.)
            };
            self.wait_steer(now, &servo_feedback, neutral, base);
            self.reason = "route_quality_wait_target_reacquiring".into();
            return self.result(now);
        };
        self.quality_wait_anchor = dict! {
            "track_id" => self.orbit_track_id.clone(),
            "point_left_m" => self.compact_target.idx("point_left_m").clone(),
            "received_at" => scan.idx("received_at").clone(), "basis" => "wait_frame_full_isolation",
        };
        if !quality_clear.is_true() || !neutral {
            self.clear_exit_prepare_window();
            self._quality_good_history.clear();
        }
        let front = points.iter().map(|p| p.0).fold(f64::NEG_INFINITY, pmax);
        let side = points.iter().map(|p| p.1).fold(INF, pmin) - SIDE_BODY_EXTENT_M;
        if advancing && neutral && self.quality_wait_release.is_none() && !self.open_loop_orbit() {
            if front <= FRONT_BODY_EXTENT_M && side >= MANEUVER_BODY_CLEARANCE_M {
                if consecutive_fresh(&mut self._wait_release_history, scan, ROUTE_EXIT_PREPARE_FRAMES, SCAN_AGE_S) {
                    self.quality_wait_release = dict! {
                        "basis" => "neutral_wait_current_support_at_body_front",
                        "source_seq" => scan.idx("seq").clone(), "source_at_ms" => scan.idx("at_ms").clone(),
                        "received_at" => scan.idx("received_at").clone(), "track_id" => self.orbit_track_id.clone(),
                        "observed_frontmost_x_m" => front, "observed_left_body_gap_m" => side,
                        "observation_count" => self._wait_release_history.len(),
                        "first_source_seq" => self._wait_release_history[0].0,
                        "held_servo_at_decision" => self.quality_wait_servo,
                        "action" => "release_left_toward_neutral_only",
                        "right_steering_authorized" => false, "physical_cone_pass_certified" => false,
                    };
                    let e = self.quality_wait_release.clone();
                    self.route_event("quality_wait_release", scan, control, e);
                }
            } else {
                self._wait_release_history.clear();
            }
        }
        let hold = |me: &mut Maneuver| -> V {
            if me.phase == "quality_wait" {
                let before = me.servo;
                let base = if me.quality_wait_release.is_some() {
                    me.exit_release_target(scan, now)
                } else {
                    let r = me.compact_target.idx("range_m").f();
                    me.wait_inside_base(pmax(0.35, pmin(1., r)))
                };
                me.wait_steer(now, &servo_feedback, neutral, base);
                if me.servo != before && me.quality_wait_release.is_some() && me.servo < before {
                    me.reason = "route_quality_wait_neutral_release_left".into();
                }
            }
            me.result(now)
        };
        if !advancing {
            return hold(self);
        }
        let passed = front <= -REAR_BODY_EXTENT_M && side >= MANEUVER_BODY_CLEARANCE_M;
        let pass_ready = if passed {
            mature_observation(&mut self._quality_pass_history, scan, 0.25)
        } else {
            self._quality_pass_history.clear();
            false
        };
        if !quality_clear.is_true() || !neutral {
            return hold(self);
        }
        if front <= FRONT_BODY_EXTENT_M && side >= MANEUVER_BODY_CLEARANCE_M {
            self._quality_exit_prepare_ready = mature_observation(&mut self._first_exit_prepare_history, scan, 0.25);
        } else {
            self.clear_exit_prepare_window();
        }
        let good = self.mature_recovery_observation(scan);
        if !good
            || !resume_ready.is_true()
            || !py_eq(&servo_feedback, &V::from(self.quality_wait_servo))
            || self.quality_wait_previous_servo.is_some()
        {
            return hold(self);
        }
        if passed {
            if !pass_ready {
                return hold(self);
            }
            let (_, rejection) = self.right_exit_target(scan, now);
            if let Some(rejection) = rejection {
                self.right_exit_geometry = V::None;
                self._right_exit_last_receive = V::None;
                self._right_exit_last_publication = V::None;
                self.quality_wait_evidence.set("last_corridor_rejection", rejection);
                hold(self);
                self.reason = "route_quality_wait_pass_corridor_unavailable".into();
                return self.result(now);
            }
            self.first_pass_preparing = true;
            self.first_pass_evidence = dict! {
                "source_seq" => scan.idx("seq").clone(), "source_at_ms" => scan.idx("at_ms").clone(),
                "received_at" => scan.idx("received_at").clone(), "track_id" => self.orbit_track_id.clone(),
                "support_bins" => self.compact_target.idx("support_bins").clone(),
                "observed_frontmost_x_m" => front, "observed_left_body_gap_m" => side,
                "basis" => "current_complete_support_behind_rear_during_neutral_wait",
                "physical_cone_pass_certified" => false, "swept_path_certified" => false,
            };
            self.right_exit_since = Some(now);
            self.route_stage = "right_align";
            let e = self.first_pass_evidence.clone();
            self.route_event("first_pass", scan, control, e);
        }
        self.phase = "drive";
        self.reason = "route_quality_resume_waiting_ack".into();
        self.quality_resume_pending = true;
        self.quality_resume_since = Some(now);
        let good_count = self._quality_good_history.len();
        self.quality_wait_evidence.update(&dict! {
            "resume_seq" => scan.idx("seq").clone(), "resume_at_ms" => scan.idx("at_ms").clone(),
            "resume_received_at" => scan.idx("received_at").clone(), "good_frame_count" => good_count,
        });
        self._first_pass_history.clear();
        if !self._quality_exit_prepare_ready {
            self.clear_exit_prepare_window();
        }
        let e = self.quality_wait_evidence.clone();
        self.route_event("quality_resume_request", scan, control, e);
        self.result(now)
    }

    /// begin_quality_coast(now, control).
    pub fn begin_quality_coast(&mut self, now: &V, control: &V) -> bool {
        self.begin_adopted_left_coast_v("turn_perception_unavailable", now, control, true)
    }

    /// begin_left_turn_quality_pause(now, control, scan=None).
    pub fn begin_left_turn_quality_pause(&mut self, now: &V, control: &V, scan: Option<&V>) -> bool {
        let scan = match scan {
            Some(s) => s.clone(),
            None => self._current_scan.clone(),
        };
        let servo = if control.get("servo").is_none() { V::Int(0) } else { control.get("servo").clone() };
        if !self.continue_route
            || self.phase != "drive"
            || self.handover_observed
            || self.route_stage != "first_target"
            || self.first_pass_evidence.is_some()
            || self.right_exit_since.is_some()
            || self.quality_resume_pending
            || !now.is_num()
            || now.f() < self.last_now
            || !control.get("armed").is_true()
            || !eq_i(control.get("motor"), TRIAL_MOTOR)
            || !control.get("command_acked").is_true()
            || !(servo.is_real() && (NEUTRAL as f64) < servo.f() && eq_i(&servo, self.servo) && self.servo <= SERVO_MAX)
            || !scan.is_dict()
            || !scan.get("received_at").is_num()
            || !(0. ..SCAN_AGE_S).contains(&(now.f() - scan.idx("received_at").f()))
        {
            return false;
        }
        self.left_turn_pause_used = true;
        self.left_turn_pause_geometry_error = None;
        self.enter_neutral_pause(
            "left_turn",
            "route_left_turn_quality_pause_neutral",
            now.f(),
            &scan,
            control,
            "quality",
            "clean quality over .30 s and current left-turn geometry",
        );
        true
    }

    /// begin_post_pass_quality_pause(now, control, scan=None).
    pub fn begin_post_pass_quality_pause(&mut self, now: &V, control: &V, scan: Option<&V>) -> bool {
        let scan = match scan {
            Some(s) => s.clone(),
            None => self._current_scan.clone(),
        };
        let mut motors = Vec::new();
        if self.route_stage == "light_lane" {
            motors.push(NEUTRAL);
        }
        motors.extend_from_slice(&DRIVE_MOTORS);
        let servo = if control.get("servo").is_none() { V::Int(0) } else { control.get("servo").clone() };
        if !self.continue_route
            || self.phase != "drive"
            || self.first_pass_evidence.is_none()
            || !POST_PASS_STAGES.contains(&self.route_stage)
            || self.quality_resume_pending
            || !now.is_num()
            || now.f() < self.last_now
            || !control.get("armed").is_true()
            || !motor_in(control.get("motor"), &motors)
            || !control.get("command_acked").is_true()
            || !(servo.is_real() && right_exit_min_pwm() as f64 <= servo.f() && eq_i(&servo, self.servo) && self.servo <= SERVO_MAX)
            || !scan.is_dict()
            || !scan.get("received_at").is_num()
            || !(0. ..SCAN_AGE_S).contains(&(now.f() - scan.idx("received_at").f()))
        {
            return false;
        }
        self.enter_neutral_pause(
            "post_pass",
            "route_post_pass_quality_pause_neutral",
            now.f(),
            &scan,
            control,
            "quality",
            "clean quality over .30 s and the stage input observed again",
        );
        true
    }

    /// _begin_recovery_hold(cause, now, control).
    pub fn begin_recovery_hold(&mut self, cause: &str, now: f64, control: &V) -> bool {
        let scan = self._current_scan.clone();
        if self.continue_route
            && self.phase == "drive"
            && self.first_pass_evidence.is_some()
            && POST_PASS_STAGES.contains(&self.route_stage)
            && !self.quality_resume_pending
            && control.get("armed").is_true()
            && control.get("command_acked").is_true()
            && eq_i(control.get("servo"), self.servo)
            && right_exit_min_pwm() <= self.servo
            && self.servo <= SERVO_MAX
            && scan.is_dict()
            && scan.get("received_at").is_num()
            && (0. ..SCAN_AGE_S).contains(&(self.last_now - scan.idx("received_at").f()))
        {
            self.right_exit_geometry = V::None;
            self._right_exit_last_receive = V::None;
            self._right_exit_last_publication = V::None;
            self._route_alignment_history.clear();
            self.enter_neutral_pause(
                "recovery",
                "route_recovery_hold_neutral",
                now,
                &scan,
                control,
                cause,
                "clean quality over .30 s and the stage input observed again",
            );
            return true;
        }
        self.begin_coast(cause, now);
        false
    }

    /// _enter_neutral_pause(kind, reason, now, scan, control, cause, resume_requires).
    #[allow(clippy::too_many_arguments)]
    fn enter_neutral_pause(&mut self, kind: &'static str, reason: &str, now: f64, scan: &V, control: &V, cause: &str, resume_requires: &str) {
        self.neutral_pause_kind = Some(kind);
        self._pause_steer_target = None;
        self.recovery_count += 1;
        self.quality_wait_since = Some(now);
        self.quality_wait_servo = Some(self.servo);
        self.quality_wait_previous_servo = None;
        self.quality_wait_release = V::None;
        self._quality_good_history.clear();
        let evidence = dict! {
            "kind" => kind, "cause" => cause, "stage" => self.route_stage, "entry_seq" => scan.idx("seq").clone(),
            "entry_at_ms" => scan.idx("at_ms").clone(), "entry_received_at" => scan.idx("received_at").clone(),
            "held_servo" => self.servo, "recovery_count" => self.recovery_count,
            "resume_requires" => resume_requires,
        };
        if kind == "left_turn" {
            self.left_turn_pause_evidence = evidence.clone();
        } else if kind == "post_pass" {
            self.post_pass_pause_evidence = evidence.clone();
        }
        self.recovery_evidence = evidence.clone();
        self.phase = "quality_wait";
        self.reason = reason.into();
        self.steering_target = self.servo;
        self.route_event(&format!("neutral_pause_{kind}"), scan, control, evidence);
    }

    /// _left_turn_pause_geometry(scan, now).
    fn left_turn_pause_geometry(&mut self, scan: &V, now: f64) -> bool {
        let candidates = scan.get("corridor_candidates").clone();
        if !candidates.is_list() || candidates.len() > 24 || candidates.l().iter().any(|c| !valid_candidate_default(c)) {
            self.lock("invalid_native_corridor_candidates");
            return false;
        }
        self.incoming_endpoint = V::None;
        let geometry_dt = match self.last_geometry_publication {
            None => 0.,
            Some(last) => (scan.idx("at_ms").i() - last) as f64 / 1000.,
        };
        let candidate = match except(&[Exc::ValueError], || self.measurement(scan, &candidates, geometry_dt)) {
            Ok(c) => c,
            Err(error) => {
                self.left_turn_pause_geometry_error = Some(error.msg);
                return false;
            }
        };
        self.last_geometry_receive = Some(now);
        self.last_geometry_publication = Some(scan.idx("at_ms").i());
        self.last_geometry_seq = Some(scan.idx("seq").i());
        self.geometry_missing = false;
        self.corridor = candidate;
        self.selected = true;
        self.start_ready = true;
        self.endpoint_observation(scan);
        self.left_turn_pause_geometry_error = None;
        true
    }

    /// _pause_stage_ready(scan, now).
    fn pause_stage_ready(&mut self, scan: &V, now: f64) -> bool {
        let kind = self.neutral_pause_kind;
        let mut stage = self.route_stage;
        self._pause_stage_held = false;
        if kind == Some("left_turn") {
            return self.left_turn_pause_geometry(scan, now);
        }
        if stage == "corridor_follow"
            && self.second_target_handover_observed
            && self.full_target_points(&self.second_target.clone(), scan, &self.second_track_id.clone()).is_some()
        {
            self.route_stage = "second_orbit";
            stage = "second_orbit";
            self.second_orbit_since = Some(now);
        }
        if stage == "right_align" || stage == "corridor_follow" {
            let (_, rejection) = self.right_exit_target(scan, now);
            if let Some(rejection) = rejection {
                self._pause_stage_held = (rejection == "right_exit_corridor_missing" || rejection == "right_exit_corridor_ambiguous")
                    && crate::maneuver_tm::observation_lease(
                        scan,
                        now,
                        &self._right_exit_last_publication.clone(),
                        &self._right_exit_last_receive.clone(),
                    );
                if !self._pause_stage_held {
                    self.right_exit_geometry = V::None;
                }
                self.update_recovery_evidence(&dict! {"last_readiness_rejection" => rejection});
                return false;
            }
            return Self::corridor_body_gap(&self.right_exit_geometry) >= 0.;
        }
        if stage == "open_transfer" || stage == "light_entry" {
            let Some(heading) = self.compass.current(scan) else { return false };
            let (ahead, _, stop_at) = self.open_front_stop(scan);
            let board_ahead = ahead.is_some_and(|a| a <= stop_at);
            if stage == "light_entry" {
                let Some((lane, point, _, _)) = self.open_light_pursuit(scan, heading, None, None) else { return false };
                self._pause_steer_target = self.light_entry_pause_hold(scan);
                let hit = self.open_entry_path_hit(scan, point, heading, stop_at, ahead);
                // Round 102: also where the drive would take a free parallel line.
                return hit.is_none_or(|h| h > stop_at)
                    || self.open_light_lane_entry(scan, heading, &lane).is_some()
                    || self.open_entry_shift(scan, heading, &lane, None, stop_at, ahead, hit.unwrap()).is_some();
            }
            if stage == "open_transfer" && self.second_track_id.is_none() {
                self.open_second_observation(scan, now, heading);
            }
            let second = if stage == "open_transfer" { self.open_confirmed_second(scan) } else { V::None };
            if second.is_some() {
                let id = second.idx("track_id").clone();
                self.open_loop_geometry(scan, &second, &id, -1, PI, None);
            }
            return !board_ahead || second.is_some();
        }
        if stage == "second_loop" {
            let target = self.second_target.clone();
            let held = self.second_tracker.as_ref().map(|t| t.held.clone()).unwrap_or(V::None);
            self._pause_stage_held = Self::target_gap_held(&held, &self.second_track_id, scan, now);
            return target.is_dict()
                && target.get("confirmed").is_true()
                && py_eq(target.get("track_id"), &self.second_track_id)
                && self.full_target_points(&target, scan, &self.second_track_id.clone()).is_some()
                && self.compass.current(scan).is_some();
        }
        if stage == "light_turn" {
            return Self::free_space(scan, self._light_heading).is_some();
        }
        if stage == "light_lane" {
            return self.current_lane(scan, now).1.is_some() || Self::free_space(scan, self._light_heading).is_some();
        }
        if stage == "second_orbit" {
            let target = self.second_target.clone();
            let held = self.second_tracker.as_ref().map(|t| t.held.clone()).unwrap_or(V::None);
            self._pause_stage_held = Self::target_gap_held(&held, &self.second_track_id, scan, now);
            return target.is_dict()
                && target.get("confirmed").is_true()
                && py_eq(target.get("track_id"), &self.second_track_id)
                && self.full_target_points(&target, scan, &self.second_track_id.clone()).is_some();
        }
        false
    }

    /// _pause_wait_base(kind).
    fn pause_wait_base(&self) -> i64 {
        if self.route_stage == "second_orbit" && self.second_return_preparing {
            return self.quality_wait_servo.unwrap().min(NEUTRAL);
        }
        if self.route_stage == "light_entry"
            && let Some(t) = self._pause_steer_target
        {
            return t;
        }
        self.quality_wait_servo.unwrap()
    }

    /// _light_entry_pause_hold(scan).
    pub fn light_entry_pause_hold(&self, scan: &V) -> Option<i64> {
        let heading = self.compass.current(scan)?;
        let pose = self.response_pose();
        Some(overshoot_guard(Self::open_heading_hold(heading, 0.), Self::open_heading_hold(heading + pose.2, 0.)))
    }

    /// _neutral_pause_update(...).
    #[allow(clippy::too_many_arguments)]
    pub fn neutral_pause_update(
        &mut self,
        scan: &V,
        lidar_age_s: &V,
        now: f64,
        control: &V,
        safe: &V,
        presteer_wait: &V,
        quality_clear: &V,
        resume_ready: &V,
    ) -> V {
        let advancing = self.orbit_inputs(scan, lidar_age_s, now, control, safe, presteer_wait);
        if self.phase == "locked" {
            let at = self.last_now;
            return self.result(at);
        }
        self.steering_target = self.quality_wait_servo.unwrap();
        let kind = self.neutral_pause_kind.unwrap();
        if now - self.drive_since.unwrap() >= self.max_drive_s {
            self.lock(&format!("route_{kind}_pause_original_budget_expired"));
            return self.result(now);
        }
        let servo_feedback = control.get("servo").clone();
        if py_eq(&servo_feedback, &V::from(self.quality_wait_servo)) {
            self.quality_wait_previous_servo = None;
        }
        let pending_step = self.quality_wait_previous_servo.is_some()
            && py_eq(&servo_feedback, &V::from(self.quality_wait_previous_servo))
            && control.get("command_acked").is_false();
        let mut motors = vec![NEUTRAL];
        motors.extend_from_slice(&DRIVE_MOTORS);
        if (!py_eq(&servo_feedback, &V::from(self.quality_wait_servo)) && !pending_step) || !motor_in(control.get("motor"), &motors) {
            self.lock(&format!("route_{kind}_pause_feedback_changed"));
            return self.result(now);
        }
        if self.quality_resume_pending {
            if control.get("resume_acked").is_true() && eq_i(control.idx("motor"), TRIAL_MOTOR) && control.get("command_acked").is_true() {
                self.quality_resume_pending = false;
                let ack = control.pick(&["motor", "servo", "seq", "tick"]);
                self.update_recovery_evidence(&dict! {"resume_ack" => ack});
                if self.route_stage == "right_align" && kind != "left_turn" {
                    self.right_exit_since = Some(now);
                }
                if self.route_stage == "light_turn" {
                    self.light_since = Some(now);
                }
                self.neutral_pause_kind = None;
            } else if now - self.quality_resume_since.unwrap() >= SCAN_AGE_S {
                self.lock(&format!("route_{kind}_pause_resume_ack_timeout"));
            }
            return self.result(now);
        }
        if control.get("neutral_acked").is_true() && !eq_i(control.idx("motor"), NEUTRAL) {
            self.lock(&format!("route_{kind}_pause_feedback_changed"));
            return self.result(now);
        }
        let neutral = control.get("neutral_acked").is_true() && eq_i(control.idx("motor"), NEUTRAL);
        if !neutral && now - self.quality_wait_since.unwrap() >= SCAN_AGE_S {
            self.lock(&format!("route_{kind}_pause_neutral_ack_timeout"));
            return self.result(now);
        }
        if !advancing {
            return self.result(now);
        }
        if self.route_stage == "light_entry"
            && let Some(hold) = self.light_entry_pause_hold(scan)
        {
            self._pause_steer_target = Some(hold);
        }
        let stage_ready = if quality_clear.is_true() && neutral { self.pause_stage_ready(scan, now) } else { false };
        if !quality_clear.is_true() || !neutral || !stage_ready {
            self._quality_full_streak = 0;
            if !(quality_clear.is_true() && neutral && self._pause_stage_held) {
                self._quality_good_history.clear();
            }
            let base = self.pause_wait_base();
            self.wait_steer(now, &servo_feedback, neutral, base);
            return self.result(now);
        }
        if !self.mature_recovery_observation(scan)
            || !resume_ready.is_true()
            || !py_eq(&servo_feedback, &V::from(self.quality_wait_servo))
            || self.quality_wait_previous_servo.is_some()
        {
            let base = self.pause_wait_base();
            self.wait_steer(now, &servo_feedback, neutral, base);
            return self.result(now);
        }
        self.phase = "drive";
        self.reason = format!("route_{kind}_pause_resume_waiting_ack");
        self.quality_resume_pending = true;
        self.quality_resume_since = Some(now);
        let good_count = self._quality_good_history.len();
        self.update_recovery_evidence(&dict! {
            "resume_seq" => scan.idx("seq").clone(), "resume_at_ms" => scan.idx("at_ms").clone(),
            "resume_received_at" => scan.idx("received_at").clone(), "good_frame_count" => good_count,
        });
        let e = self.recovery_evidence.clone();
        self.route_event(&format!("{kind}_pause_resume_request"), scan, control, e);
        self.result(now)
    }

    /// begin_route_quality_coast(now, control).
    pub fn begin_route_quality_coast(&mut self, now: &V, control: &V) -> bool {
        if !self.continue_route || self.route_stage != "first_target" {
            return false;
        }
        self.begin_adopted_left_coast_v("route_quality_coast_adopted_left", now, control, false)
    }

    /// _begin_adopted_left_coast(reason, now, control, require_handover).
    pub fn begin_adopted_left_coast(&mut self, reason: &str, now: f64, control: &V, require_handover: bool) -> bool {
        self.begin_adopted_left_coast_v(reason, &V::Float(now), control, require_handover)
    }

    pub fn begin_adopted_left_coast_v(&mut self, reason: &str, now: &V, control: &V, require_handover: bool) -> bool {
        let motors: &[i64] = if self.continue_route { &DRIVE_MOTORS } else { &[TRIAL_MOTOR] };
        if self.phase != "drive"
            || (require_handover && !self.handover_observed)
            || self.first_pass_evidence.is_some()
            || self.right_exit_since.is_some()
            || !control.get("armed").is_true()
            || !motor_in(control.get("motor"), motors)
            || !control.get("command_acked").is_true()
            || !control.get("servo").is_int()
            || !(NEUTRAL < control.idx("servo").i() && control.idx("servo").i() == self.servo && self.servo <= SERVO_MAX)
        {
            return false;
        }
        let now_f = if now.is_num() { now.f() } else { f64::NAN };
        self.begin_coast(reason, now_f);
        if self.phase != "coast" {
            return false;
        }
        self.quality_coast_servo = Some(control.idx("servo").i());
        self.quality_coast_reason = Some(reason.into());
        self.steering_target = self.quality_coast_servo.unwrap();
        true
    }
}
