//! Steering slew, proximity avoidance and the predicted-arc path check.

use std::rc::Rc;

use crate::dict;
use crate::helpers::*;
use crate::maneuver::*;
use crate::py::*;

impl Maneuver {
    /// _scan_axis_lines(scan, heading): once per scan and heading.
    pub fn scan_axis_lines(&mut self, scan: &V, heading: f64) -> Rc<Vec<Line>> {
        let ranges = scan.idx("ranges").clone();
        let key = V::list(vec![scan.get("seq").clone(), scan.get("at_ms").clone()]);
        let fresh = match &self._axis_cache {
            Some((k, r, _)) => !(py_eq(k, &key) && py_eq(r, &ranges)),
            None => true,
        };
        if fresh {
            self._axis_cache = Some((key, ranges.clone(), Vec::new()));
        }
        let cache = self._axis_cache.as_mut().unwrap();
        if let Some((_, lines)) = cache.2.iter().find(|(h, _)| *h == heading) {
            return lines.clone();
        }
        let floats = crate::compact_target::floats(&ranges);
        let lines = Rc::new(axis_lines(&scan_points(&floats, OPEN_LINE_REACH_M), heading));
        cache.2.push((heading, lines.clone()));
        lines
    }

    /// _route_proximity(scan).
    pub fn route_proximity(&mut self, scan: &V) -> V {
        let ranges = if scan.is_dict() { scan.get("ranges").clone() } else { V::None };
        if !ranges.is_list() || ranges.len() != 360 {
            return V::None;
        }
        let key = V::list(vec![scan.get("seq").clone(), scan.get("at_ms").clone(), V::Bool(self.first_pass_evidence.is_none())]);
        if let Some((k, r, result)) = &self._proximity_cache
            && py_eq(k, &key)
            && py_eq(r, &ranges)
        {
            return result.clone();
        }
        let result = self.route_proximity_scan(scan, &ranges);
        self._proximity_cache = Some((key, ranges, result.clone()));
        result
    }

    fn route_proximity_scan(&self, scan: &V, ranges: &V) -> V {
        let list = ranges.l();
        let mut best: Option<(f64, f64, f64, usize, bool)> = None;
        for (index, distance) in list.iter().enumerate() {
            if !distance.is_num() || !(0.02..=12.).contains(&distance.f()) {
                continue;
            }
            let d = distance.f();
            let angle = radians(index as f64);
            let (x, y) = (d * angle.pcos(), -d * angle.psin());
            let head_on = x > FRONT_BODY_EXTENT_M && y.abs() <= SIDE_BODY_EXTENT_M / 2.;
            if y > 0. && !head_on && self.first_pass_evidence.is_none() {
                continue;
            }
            let gap = hypot(
                pmax(pmax(-REAR_BODY_EXTENT_M - x, 0.), x - FRONT_BODY_EXTENT_M),
                pmax(y.abs() - SIDE_BODY_EXTENT_M, 0.),
            );
            if gap < ROUTE_AVOID_GAP_M && x >= -REAR_BODY_EXTENT_M && best.is_none_or(|b| gap < b.0) {
                best = Some((gap, x, y, index, head_on));
            }
        }
        let Some((gap, x, y, index, head_on)) = best else { return V::None };
        let side = if head_on {
            let nearest = |bins: std::ops::Range<usize>| -> f64 {
                let values: Vec<f64> = bins.filter(|&i| list[i].is_num()).map(|i| list[i].f()).collect();
                if values.is_empty() { 12. } else { values.iter().cloned().fold(INF, pmin) }
            };
            if nearest(270..340) >= nearest(20..91) { "front_left_open" } else { "front_right_open" }
        } else if y > 0. {
            "left"
        } else {
            "right"
        };
        dict! {
            "gap_m" => gap, "point_left_m" => V::from((x, y)), "bin" => index, "head_on" => head_on,
            "side" => side, "source_seq" => scan.get("seq").clone(),
        }
    }

    /// _path_points(scan).
    pub fn path_points(scan: &V) -> Option<Vec<(f64, f64)>> {
        let ranges = if scan.is_dict() { scan.get("ranges") } else { &V::None };
        let list = ranges.ol()?;
        if list.len() != 360 {
            return None;
        }
        let mut points = Vec::new();
        for (index, distance) in list.iter().enumerate() {
            if distance.is_num() && (0.02..=ROUTE_PATH_RANGE_M).contains(&distance.f()) {
                let d = distance.f();
                let (x, y) = (dcos(index) * d, -dsin(index) * d);
                if x > -REAR_BODY_EXTENT_M - 0.2 {
                    points.push((x, y));
                }
            }
        }
        Some(points)
    }

    /// _path_free_distance(points, servo).
    pub fn path_free_distance(points: &[(f64, f64)], servo: i64) -> f64 {
        let kappa = (servo - NEUTRAL) as f64 / ROUTE_PWM_PER_CURVATURE;
        let mut best = INF;
        for &(x, y) in points {
            let (along, normal) = if kappa.abs() < 1e-3 {
                (x, y)
            } else {
                let radius = 1. / kappa.abs();
                let side = if kappa > 0. { y } else { -y };
                let rho = hypot(x, radius - side);
                (radius * x.atan2(radius - side), radius - rho)
            };
            if along + REAR_BODY_EXTENT_M < 0. {
                continue;
            }
            let half = SIDE_BODY_EXTENT_M + if along > FRONT_BODY_EXTENT_M { ROUTE_PATH_MARGIN_M } else { 0. };
            if normal.abs() > half {
                continue;
            }
            best = pmin(best, pmax(0., along - FRONT_BODY_EXTENT_M));
        }
        best
    }

    pub fn route_right_authorized(&self) -> bool {
        self.continue_route
            && self.phase == "drive"
            && !self.quality_resume_pending
            && self.first_pass_evidence.is_some()
            && self._route_center_ack.is_some()
            && ((["right_align", "corridor_follow"].contains(&self.route_stage) && self.right_exit_center_ack.is_some())
                || (self.route_stage == "second_orbit" && self.second_center_ack.is_some())
                || ["light_turn", "light_lane"].contains(&self.route_stage)
                || OPEN_STAGES.contains(&self.route_stage))
    }

    /// _route_override_bounds(target).
    fn route_override_bounds(&self, target: i64) -> (i64, i64) {
        if self.route_stage == "second_orbit" && self.second_return_preparing {
            return (target, target);
        }
        if TASK_LEFT_BYPASS_STAGES.contains(&self.route_stage) {
            let lower = if target < NEUTRAL { right_exit_min_pwm() } else { NEUTRAL };
            let mut upper = SERVO_MAX;
            let scan = &self._current_scan;
            if let Some((seq, cap)) = &self._task_left_upper
                && scan.is_dict()
                && py_eq(seq, scan.get("seq"))
            {
                upper = target.max(SERVO_MAX.min(*cap));
            }
            return (lower, lower.max(upper));
        }
        if self.entry_kind == Some("lane")
            && (self.route_stage == "start_lane" || self.route_stage == "first_target")
            && self.first_pass_evidence.is_none()
        {
            return (NEUTRAL, SERVO_MAX);
        }
        if target == NEUTRAL {
            return (NEUTRAL, NEUTRAL);
        }
        if target < NEUTRAL {
            return (right_exit_min_pwm(), NEUTRAL);
        }
        let mut upper = SERVO_MAX;
        if self.handover_observed && self.route_stage == "first_target" && self.first_pass_evidence.is_none() && target <= self.servo {
            upper = target;
        }
        (NEUTRAL, upper)
    }

    /// _route_path_target(target).
    fn route_path_target(&mut self, target: i64) -> i64 {
        let scan = self._current_scan.clone();
        let natural = self.natural_steering_target;
        let authorized = self.route_right_authorized();
        let pending_right =
            natural.is_some_and(|n| n < NEUTRAL) && !authorized && !TASK_LEFT_BYPASS_STAGES.contains(&self.route_stage);
        let (lower, upper) = self.route_override_bounds(target);
        let seq = if scan.is_dict() { scan.get("seq").clone() } else { V::None };
        let key = V::list(vec![
            seq.clone(),
            V::Int(target),
            V::from(self.route_stage),
            V::Bool(authorized),
            V::Bool(pending_right),
            V::Int(lower),
            V::Int(upper),
        ]);
        if let Some((k, choice)) = &self._path_cache
            && py_eq(k, &key)
        {
            return *choice;
        }
        let points = Self::path_points(&scan);
        let mut choice = target;
        if let Some(points) = points
            && !points.is_empty()
            && !pending_right
        {
            let free = Self::path_free_distance(&points, target);
            if free < ROUTE_PATH_LOOKAHEAD_M {
                let mut candidates = route_path_left_candidates();
                if authorized {
                    candidates.extend(route_path_right_candidates());
                }
                candidates.retain(|&c| lower <= c && c <= upper);
                let scored: Vec<(i64, f64)> = candidates.iter().map(|&c| (c, Self::path_free_distance(&points, c))).collect();
                let clear: Vec<i64> = scored.iter().filter(|(_, f)| *f >= ROUTE_PATH_LOOKAHEAD_M).map(|(c, _)| *c).collect();
                let prefer_left = TASK_LEFT_BYPASS_STAGES.contains(&self.route_stage);
                if !clear.is_empty() {
                    let above: Vec<i64> = clear.iter().cloned().filter(|&c| c >= target).collect();
                    let pool = if prefer_left && !above.is_empty() { above } else { clear.clone() };
                    let mut best = pool[0];
                    for &c in &pool[1..] {
                        let (kc, kb) = (((c - target).abs(), -c), ((best - target).abs(), -best));
                        if kc < kb {
                            best = c;
                        }
                    }
                    choice = best;
                    self.path_override_count += (choice != target) as i64;
                }
                let chosen_free = scored.iter().find(|(c, _)| *c == choice).map(|(_, f)| *f).unwrap_or(free);
                self.path_evidence = dict! {
                    "source_seq" => seq, "stage" => self.route_stage, "stage_target" => target,
                    "stage_free_m" => free, "chosen" => choice,
                    "chosen_free_m" => chosen_free, "clear_candidates" => clear.len(),
                    "overridden" => choice != target, "right_authorized" => authorized,
                    "stage_override_bounds" => V::from(vec![lower, upper]),
                    "pwm_per_curvature" => ROUTE_PWM_PER_CURVATURE, "swept_path_certified" => false,
                };
            }
        }
        self._path_cache = Some((key, choice));
        choice
    }

    /// _avoid_target(target).
    fn avoid_target(&self, target: i64) -> i64 {
        let avoid = &self._avoid;
        if avoid.is_none() {
            return target;
        }
        let side = avoid.idx("side").s();
        let choice = if side == "left" || side == "front_right_open" { target.min(NEUTRAL) } else { target.max(ROUTE_AVOID_LEFT_PWM) };
        let (lower, upper) = self.route_override_bounds(target);
        lower.max(upper.min(choice))
    }

    /// ManeuverSequence._slew(now).
    pub fn slew(&mut self, now: f64) {
        let avoid_target = if self.continue_route
            && self.phase == "drive"
            && !self.quality_resume_pending
            && (self.handover_observed || (self.route_stage == "start_lane" && self.first_approach.is_some()))
        {
            let t = self.steering_target;
            self.route_path_target(t)
        } else if self.continue_route && (self.phase == "drive" || self.phase == "quality_wait") {
            self.avoid_target(self.steering_target)
        } else {
            self.steering_target
        };
        if now - self.last_change + 1e-9 >= PWM_INTERVAL_S && self.servo != avoid_target {
            let mut target = avoid_target;
            let reducing_left = self.servo > NEUTRAL && target < self.servo;
            let post_pass = self.continue_route
                && self.phase == "drive"
                && self.first_pass_evidence.is_some()
                && POST_PASS_STAGES.contains(&self.route_stage);
            let orbit_left = self.continue_route
                && self.phase == "drive"
                && self.handover_observed
                && self.route_stage == "first_target"
                && self.first_pass_evidence.is_none()
                && target > self.servo
                && self.servo >= NEUTRAL;
            let lane_approach = self.continue_route
                && self.phase == "drive"
                && ((self.route_stage == "start_lane"
                    && (self._lane_start_slow || self.reason == "start_lane_lane_walls" || self.reason == "start_lane_side_wall"))
                    || (self.route_stage == "first_target" && self.entry_kind == Some("lane") && self.handover_observed));
            // Round 102: a light-entry pause steps its lane-axis hold at the open-stage steps.
            let light_pause = self.continue_route
                && self.phase == "quality_wait"
                && self.first_pass_evidence.is_some()
                && self.route_stage == "light_entry"
                && self._pause_steer_target.is_some();
            let open_steps = light_pause || (post_pass && OPEN_STAGES.contains(&self.route_stage));
            let crossing = (target - NEUTRAL) * (self.servo - NEUTRAL) < 0;
            let releasing = self.servo != NEUTRAL && (crossing || (target - NEUTRAL).abs() < (self.servo - NEUTRAL).abs());
            let post_step = if post_pass && reducing_left && self.route_stage == "second_orbit" && self.second_return_preparing {
                ROUTE_RETURN_RELEASE_PWM_STEP
            } else if open_steps && releasing {
                if crossing {
                    target = NEUTRAL;
                }
                OPEN_RELEASE_PWM_STEP
            } else if lane_approach || open_steps {
                OPEN_PWM_STEP
            } else {
                ROUTE_POST_PASS_PWM_STEP
            };
            let mut step = if self.phase == "presteer" {
                PRESTEER_PWM_STEP
            } else if post_pass || lane_approach || light_pause {
                post_step
            } else if reducing_left {
                if self.route_exit_release_active() { ROUTE_EXIT_RELEASE_PWM_STEP } else { LEFT_RELEASE_PWM_STEP }
            } else if orbit_left {
                ROUTE_ORBIT_LEFT_PWM_STEP
            } else {
                PWM_STEP
            };
            if avoid_target != self.steering_target {
                step = step.max(ROUTE_AVOID_PWM_STEP);
            }
            if reducing_left && target < NEUTRAL {
                target = NEUTRAL;
            }
            let delta = (-step).max(step.min(target - self.servo));
            self.servo += delta;
            self.last_change = now;
            self.settle_since = None;
        }
    }
}
