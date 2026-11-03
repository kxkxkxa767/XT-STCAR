//! Terminal board control: release launch, predicted coast and the single ESC brake.
//! The live approach to a traffic light has its own controller and speed history.
use crate::dict;
use crate::helpers::*;
use crate::maneuver::*;
use crate::maneuver_post::fit_value;
use crate::motion_sensors::ESC_COAST_DELAY_S;
use crate::py::*;

#[derive(Clone, Debug, Default)]
pub(crate) struct FinishControl {
    pub ranges: crate::motion_sensors::RangeHistory,
    pub last_speed: Option<(f64, f64)>,
    pub launch_until: Option<f64>,
    pub launch_from: Option<f64>,
    pub brake_since: Option<f64>,
    pub brake_end: Option<f64>,
    pub pulse: i64,
    pub creeping: bool,
    pub creeps: i64,
    /// Round y35: the IMU's forward integral at the light release (the car stood),
    /// its speed since then over the last moments and the launch's measured rise.
    pub imu_zero: Option<f64>,
    pub imu_history: Vec<(f64, f64)>,
    pub imu_accel: Option<f64>,
}

// Round y35: y34's first run (12.0 V, y32 11.8 V) launched to 1.97 m/s where the
// launch's 1.5/2.5 m/s2 aim and bound expected less; the IMU measures the rise.
const IMU_SLOPE_S: f64 = 0.2;
// The single 1300 brake's deceleration on the car's IMU, y25-y27: .71-.82 m/s2.
const LANE_BRAKE_DECEL_MPS2: f64 = 0.71;

impl FinishControl {
    pub fn has_launch(&self) -> bool { self.launch_until.is_some() }
}

impl Maneuver {
    pub(super) fn finish_control(&mut self, scan: &V, ahead: Option<f64>, now: f64) -> (Option<String>, V) {
        self.lane_goal_control(scan, ahead, now, &FINISH_GOAL)
    }

    // Only the legacy console reference uses the original generic light goal.
    pub(super) fn reference_light_goal(&mut self, scan: &V, ahead: Option<f64>, now: f64) -> (Option<String>, V) {
        self.lane_goal_control(scan, ahead, now, &LIGHT_GOAL)
    }

    pub(super) fn prepare_finish_wait(&mut self, scan: &V) {
        self.finish.pulse = 0;
        self.finish.creeping = false;
        self.lane_speed(scan, None, 2);
    }

    pub(super) fn prepare_finish_release(&mut self, scan: &V, now: f64) {
        let board = self.lane_front_range(scan);
        if self.route_stage == "light_lane" && board.is_some_and(|b| b - FRONT_BODY_EXTENT_M >= LIGHT_LAUNCH_MIN_GAP_M) {
            self.finish.launch_until = Some(now + ESC_LAG_S + LIGHT_LAUNCH_S);
            self.finish.launch_from = Some(now);
        }
        self.finish.creeps = 0;
        self.finish.imu_zero = self.race_inputs.and_then(|i| i.forward_integral); // standing at the light
        self.finish.imu_history.clear();
        self.finish.imu_accel = None;
    }

    /// Round y35: the IMU's forward speed since the light release, and the launch's
    /// measured rise (its slope over the last IMU_SLOPE_S while the launch acts).
    fn finish_imu_speed(&mut self, now: f64) -> Option<f64> {
        let integral = self.race_inputs.and_then(|i| i.forward_integral)?;
        let v = (integral - self.finish.imu_zero?).max(0.);
        let h = &mut self.finish.imu_history;
        h.push((now, v));
        while h.len() > 2 && h[1].0 <= now - IMU_SLOPE_S {
            h.remove(0);
        }
        let (t0, v0) = h[0];
        if self.finish.launch_from.is_some() && now - t0 >= IMU_SLOPE_S / 2. {
            let rise = (v - v0) / (now - t0);
            self.finish.imu_accel = Some(self.finish.imu_accel.map_or(rise, |a| pmax(a, rise)));
        }
        Some(v)
    }

    /// _lane_speed_now(speed, now, onset, offset, decel, accel).
    fn lane_speed_now(&self, speed: f64, now: f64, onset: f64, offset: f64, decel: f64, accel: f64) -> f64 {
        let history = &self.finish.ranges;
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

    /// _lane_worst_gap(gap, speed, now, decel, accel, reaction, push_s, long_lag).
    #[allow(clippy::too_many_arguments)]
    fn lane_worst_gap(&self, gap: f64, speed: f64, now: f64, decel: f64, accel: f64, reaction: f64, push_s: f64, long_lag: f64) -> f64 {
        let mut worst: Option<f64> = None;
        for lag in [LANE_ESC_LAG_MIN_S, long_lag] {
            let (onset, offset) = (lag - LANE_ESC_SKEW_S, lag + LANE_ESC_SKEW_S);
            let mut v = self.lane_speed_now(speed, now, onset, offset, decel, accel);
            if let Some(from) = self.finish.launch_from {
                // round y3: from rest, no faster than the kick's bound while its power has acted
                // (round y35: or the launch's rise the IMU measured)
                let bound = self.finish.imu_accel.map_or(LIGHT_LAUNCH_ACCEL_MAX_MPS2, |a| pmax(LIGHT_LAUNCH_ACCEL_MAX_MPS2, a));
                v = pmin(v, bound * pmax(0., now - (from + onset)));
            }
            v += LANE_SPEED_GUARD_MARGIN_MPS;
            let (coast_s, power_s) = self.lane_pipeline(now, onset, offset, push_s);
            // round y20: the launch's peak not capped at LANE_PUSH_MAX_MPS
            let peak_max = if self.finish.launch_from.is_some() { f64::INFINITY } else { LANE_PUSH_MAX_MPS };
            let value = Self::lane_stop_to(gap, v, decel, reaction, coast_s, power_s, accel, peak_max);
            worst = Some(match worst {
                None => value,
                Some(w) => pmin(w, value),
            });
        }
        worst.unwrap()
    }

    /// _lane_stop(gap, speed, decel, reaction, coast_s, power_s, accel).
    fn lane_stop(gap: f64, speed: f64, decel: f64, reaction: f64, coast_s: f64, power_s: f64, accel: f64) -> f64 {
        Self::lane_stop_to(gap, speed, decel, reaction, coast_s, power_s, accel, LANE_PUSH_MAX_MPS)
    }

    /// _lane_stop with its peak_max.
    #[allow(clippy::too_many_arguments)]
    fn lane_stop_to(gap: f64, speed: f64, decel: f64, reaction: f64, coast_s: f64, power_s: f64, accel: f64, peak_max: f64) -> f64 {
        let travel = speed * reaction;
        if speed <= decel * coast_s {
            return gap - travel - speed * speed / (2. * decel);
        }
        let coasted = speed - decel * coast_s;
        let peak = pmin(coasted + accel * power_s, pmax(coasted, peak_max));
        gap - travel - (speed + coasted) / 2. * coast_s - peak * power_s - peak * peak / (2. * decel)
    }

    /// _lane_coast_observe(scan, now, speed) (round y15: the board ahead and behind).
    fn lane_coast_observe(&mut self, scan: &V, now: f64, speed: Option<f64>) {
        let coasting = self.lane_motor == NEUTRAL
            && speed.is_none_or(|s| s >= LANE_SPEED_RESTART_MPS)
            && self.coast_fit_window(now);
        if !coasting {
            self.coast_measure(scan, false, None, None);
            return;
        }
        let front = self.lane_front_range(scan);
        let rear = self.lane_rear_range(scan);
        self.coast_measure(scan, true, front, rear);
    }

    /// _lane_push_observe(now, speed) (round y15).
    fn lane_push_observe(&mut self, now: f64, speed: Option<f64>) {
        let Some((motor, since, mut low, mut end, mut high)) = self._lane_push_probe else {
            return;
        };
        if end.is_none() && self.lane_motor != motor {
            if DRIVE_MOTORS.contains(&self.lane_motor) {
                self._lane_push_probe = None; // turned into another push
                return;
            }
            end = self._power_cmd_at;
        } else if let Some(e) = end
            && (DRIVE_MOTORS.contains(&self.lane_motor) || now - e > LANE_PUSH_PROBE_S)
        {
            self._lane_push_probe = None;
            if let (Some(l), Some(h)) = (low, high)
                && now - e >= LANE_PUSH_PROBE_MIN_S
            {
                let rate = pmax(LANE_PUSH_RATE_MIN_MPS2, (h - l) / (e - since + LANE_PUSH_MIN_S));
                if motor == TRIAL_MOTOR {
                    self._lane_push_rate_strong = Some(rate);
                } else {
                    self._lane_push_rate_slow = Some(rate);
                }
            }
            return;
        }
        if let Some(sp) = speed {
            if now - since <= LANE_PUSH_PROBE_S {
                low = Some(match low {
                    None => sp,
                    Some(l) => pmin(l, sp),
                });
            }
            if end.is_some() {
                high = Some(match high {
                    None => sp,
                    Some(h) => pmax(h, sp),
                });
            }
        }
        self._lane_push_probe = Some((motor, since, low, end, high));
    }

    /// _lane_goal_control(scan, ahead, now, goal): the light lane's
    /// coast/push/kick/creep control toward a line ``ahead`` (the end board,
    /// round y2 also the light line): (basis, feedback).
    fn lane_goal_control(&mut self, scan: &V, ahead: Option<f64>, now: f64, goal: &Goal) -> (Option<String>, V) {
        let kind = goal.kind;
        let speed = self.lane_speed(scan, ahead, 2);
        self.lane_coast_observe(scan, now, speed);
        self.lane_push_observe(now, speed);
        let held = self.finish.last_speed;
        let mut speed_known = true;
        let v = if let Some(s) = speed {
            self.finish.last_speed = Some((now, s));
            s
        } else if let Some(h) = held.filter(|h| 0. <= now - h.0 && now - h.0 <= LANE_SPEED_HOLD_S) {
            h.1 + LANE_SPEED_HOLD_MARGIN_MPS
        } else {
            speed_known = false;
            LANE_SPEED_UNKNOWN_MPS
        };
        // Round y35: after a light stop the IMU's speed from that rest (the lidar slope
        // runs behind it, low while the launch pushes and high while the car slows)
        let imu = if kind == "finish" { self.finish_imu_speed(now) } else { None };
        let v = imu.map_or(v, |i| pmax(v, i));
        let gap = ahead.map(|a| a - FRONT_BODY_EXTENT_M);
        let (decel, guard_decel, strong_decel) = self.lane_decels();
        let powered = DRIVE_MOTORS.contains(&self.lane_motor);
        let (aim_coast, aim_power) = self.lane_pipeline(now, LANE_ESC_LAG_NOMINAL_S, LANE_ESC_LAG_NOMINAL_S, 0.);
        let coasting = !powered && self._power_cmd_at.is_none_or(|at| now >= at + ESC_LAG_S + LANE_ESC_SKEW_S);
        let restarted = self.light_stop_evidence.has("go") || self.light_stop_state == Some("passed");
        // Round y15: after the light the finish's tail lag, its 1560 top-ups and
        // the pushes' measured responses.
        let after_light = kind == "finish" && restarted;
        // once no launch power can be left in the ESC pipeline
        // Round y20: the light's approach too (no launch before it).
        let relaxed = kind == "light"
            || after_light
                && (self._lane_topup_motor.is_some() || self._power_cmd_at.is_none_or(|at| now - at >= ESC_LAG_S + LANE_ESC_SKEW_S));
        let long_lag = if relaxed { LANE_FINISH_ESC_LAG_S } else { ESC_LAG_S };
        let strong = self._lane_topup_motor == Some(TRIAL_MOTOR) && self._power_cmd_motor == Some(TRIAL_MOTOR);
        // round y15: the launch's 1560 still acts through its 1550 tail
        let kick = self._power_cmd_motor == Some(TRIAL_MOTOR) && !strong || self.finish.launch_from.is_some();
        let launch = self.finish.launch_from.is_some(); // round y18: the launch's powered run (y20: 1600)
        let (rate_slow, rate_strong) = (self._lane_push_rate_slow, self._lane_push_rate_strong);
        let launch_rise = self.finish.imu_accel; // round y35: the launch's rise on the IMU
        let aim_accel = if strong {
            rate_strong.unwrap_or(LANE_POWER_ACCEL_MPS2)
        } else if launch {
            launch_rise.map_or(LIGHT_LAUNCH_ACCEL_MPS2, |a| pmax(LIGHT_LAUNCH_ACCEL_MPS2, a))
        } else if kick {
            LANE_POWER_ACCEL_MPS2
        } else {
            rate_slow.unwrap_or(LANE_PUSH_ACCEL_MPS2)
        };
        let strong_accel = match rate_strong {
            None => LANE_KICK_ACCEL_MAX_MPS2,
            Some(r) => pmax(LANE_STRONG_PUSH_ACCEL_MIN_MPS2, LANE_PUSH_RATE_MARGIN * r),
        };
        let late = kind == "finish" && gap.is_some_and(|g| g <= goal.zone_gap); // round y20: the finish's only
        let guard_accel = if strong {
            strong_accel
        } else if launch {
            launch_rise.map_or(LIGHT_LAUNCH_ACCEL_MAX_MPS2, |a| pmax(LIGHT_LAUNCH_ACCEL_MAX_MPS2, a))
        } else if kick {
            LANE_KICK_ACCEL_MAX_MPS2
        } else if late {
            LANE_PUSH_LATE_ACCEL_MAX_MPS2
        } else {
            LANE_PUSH_ACCEL_MAX_MPS2
        };
        let mut v_aim = v;
        if let Some(i) = imu {
            v_aim = i; // round y35: the speed now, measured
        } else if speed.is_some() {
            v_aim = self.lane_speed_now(v, now, LANE_ESC_LAG_NOMINAL_S, LANE_ESC_LAG_NOMINAL_S, decel, aim_accel);
        }
        let guard_reaction = if powered { LANE_REACTION_BASE_S } else { LANE_AIM_REACTION_S };
        let (mut stop_gap, mut worst_gap) = (None, None);
        if let Some(g) = gap {
            stop_gap = Some(Self::lane_stop(g, v_aim, decel, if powered { LANE_AIM_REACTION_S } else { 0. }, aim_coast, aim_power, aim_accel));
            worst_gap = Some(self.lane_worst_gap(g, v, now, guard_decel, guard_accel, guard_reaction, 0., long_lag));
        }
        let stopped = speed.is_some_and(|s| s < LANE_SPEED_RESTART_MPS);
        let in_zone = gap.is_some_and(|g| g <= goal.zone_gap);
        let body_in_zone = gap.is_some_and(|g| g <= goal.body_gap);
        let kick_room = gap.is_some_and(|g| g >= goal.min_gap + LANE_KICK_TRAVEL_M);
        let rested = self._power_cmd_at.is_none_or(|at| now - at >= LANE_KICK_REST_S);
        // Round 105: the finish creep.
        let creep_room = in_zone
            && !body_in_zone
            && !kick_room
            && gap.unwrap() >= goal.min_gap + goal.creep_travel
            && (self.finish.creeps < goal.creep_max || !rested);
        // Round y2: the light stop ends stopped only; so does the finish after a
        // restart from a light wait (the launch's own coast fit reads low).
        // Round y20 (rules 2.4.2: 红灯亮起立即停车): after a red frame taken with the front in
        // the light's zone, no power once the whole car is in; with the rear out it only comes in.
        if kind == "light" && in_zone {
            let zone_at = *self._light_zone_at.get_or_insert(now);
            if self._light_frames.last().is_some_and(|f| f.at(2).f() >= LIGHT_RED_MIN_SCORE && f.at(1).f() >= zone_at) {
                self._light_red_hold = true;
            }
        }
        let red_hold = kind == "light" && self._light_red_hold && body_in_zone;
        let mut basis: Option<String> = None;
        if speed_known && stopped && body_in_zone {
            basis = Some(format!("stopped_with_the_whole_car_in_the_{kind}_zone"));
        } else if speed_known && stopped && in_zone && !kick_room && !creep_room {
            basis = Some(if self.finish.creeps != 0 {
                format!("stopped_front_in_{kind}_zone_creeps_used")
            } else {
                format!("stopped_front_in_{kind}_zone_restart_could_reach_the_{}", if kind == "finish" { "board" } else { "line" })
            });
        } else if kind == "light" && gap.is_none() && speed_known && stopped {
            basis = Some("stopped_with_the_light_line_lost_in_the_zone".into());
        } else if kind == "finish"
            && !restarted
            && speed_known
            && coasting
            && !stopped
            && strong_decel.is_some()
            && gap.is_some()
            && stop_gap.unwrap() >= goal.min_gap
            && Self::lane_stop(gap.unwrap(), v_aim, strong_decel.unwrap(), 0., 0., 0., 0.) <= goal.body_gap
        {
            basis = Some("coast_stops_the_whole_car_in_the_finish_zone".into());
        }
        let mut feedback = dict! {
            "front_range_m" => ahead, "front_gap_m" => gap, "speed_mps" => speed, "speed_for_cut_mps" => v,
            "speed_aim_mps" => v_aim, "speed_known" => speed_known,
            "speed_basis" => if kind == "finish" { "closing_rate_on_board_ahead_axis" } else { "closing_rate_on_light_line_axis" },
            "predicted_stop_gap_m" => stop_gap, "worst_stop_gap_m" => worst_gap, "in_finish_zone" => in_zone,
            "whole_car_in_finish_zone" => body_in_zone, "pipeline_power_s" => aim_power,
            "coast_decel_mps2" => decel, "guard_decel_mps2" => guard_decel, "strong_decel_mps2" => strong_decel,
            "coast_decel_se_mps2" => self._lane_coast_decel_se,
            "coast_decel_measured" => self._lane_coast_decel.is_some(), "coast_fit" => fit_value(self._coast_fit),
            "push_rates_mps2" => V::list(vec![V::from(rate_slow), V::from(rate_strong)]),
            "guard_accel_mps2" => guard_accel, "tail_lag_s" => long_lag,
            "motor_before" => self.lane_motor, "swept_path_certified" => false, "traffic_light_vision" => self.light_vision,
            "creep_room" => creep_room, "creeps" => self.finish.creeps, "goal" => kind, "red_hold" => red_hold,
        };
        if basis.is_some() {
            return (basis, feedback);
        }
        let launching = self.finish.launch_until.is_some_and(|until| now < until);
        let mut topup_motor: Option<i64> = None; // round y15: this scan's output continues or starts a top-up
        let braking = self.finish.brake_since.is_some();
        let brake = after_light
            && self.finish.brake_end.is_none()
            && gap.is_some()
            && speed.is_some_and(|s| s >= LANE_BRAKE_MIN_MPS)
            && stop_gap.is_some_and(|s| s < goal.target_gap)
            && match self.finish.brake_since {
                Some(since) => now - since < LANE_BRAKE_MAX_S,
                // Round y35: farther out once the measured speed needs more than 1.2 m (y34's
                // first run met 1.2 m at 1.63 m/s and reached the board); the brake still ends
                // on the predicted coast, so an early start does not stop it short.
                None => gap.unwrap() <= LANE_BRAKE_START_GAP_M
                    || imu.is_some()
                        && gap.unwrap() <= goal.target_gap + v * ESC_COAST_DELAY_S + v * v / (2. * LANE_BRAKE_DECEL_MPS2),
            };
        if red_hold {
            self.lane_motor = NEUTRAL; // round y20: in the red with the whole car in: no power
        } else if brake {
            self.lane_motor = LANE_BRAKE_MOTOR; // round y22: the coast would end past the target
        } else if gap.is_none() && kind == "light" {
            self.lane_motor = NEUTRAL; // round y2: no light line this scan: coast
        } else if gap.is_some() && (stop_gap.unwrap() <= goal.target_gap || worst_gap.unwrap() <= goal.min_gap) {
            self.lane_motor = NEUTRAL;
        } else if launching {
            self.lane_motor = LIGHT_LAUNCH_MOTOR; // round y2: the launch after green (y18: 1580, y20: 1600)
        } else if self.finish.launch_until.is_some() && (self.lane_motor == TRIAL_MOTOR || self.lane_motor == LIGHT_LAUNCH_MOTOR) {
            self.lane_motor = ROUTE_SLOW_MOTOR; // the launch goes on at 1550
        } else if speed.is_none() {
            self.lane_motor = ROUTE_SLOW_MOTOR;
        } else if stopped {
            self.lane_motor = if (kick_room || creep_room) && (rested || self.finish.pulse > 0) { TRIAL_MOTOR } else { NEUTRAL };
        } else if self.lane_motor == ROUTE_SLOW_MOTOR || self.lane_motor == TRIAL_MOTOR && strong {
            // the straight, or a push (round y15: also a 1560 top-up), until the coast is aimed
            if self._lane_topup_motor == Some(self.lane_motor) {
                topup_motor = Some(self.lane_motor);
            }
        } else if self.lane_motor == NEUTRAL
            && stop_gap.unwrap() > goal.target_gap + LANE_TOPUP_BAND_M
            && v_aim - decel * LANE_ESC_LAG_NOMINAL_S >= LANE_PUSH_ROLLING_MPS
            // round y19: after the light not while the launch's power may still act
            && (relaxed || !after_light)
        {
            // it would stop short: push again while it rolls (round y15: after the
            // light at 1560 where that push's guard passes, outside the zone)
            let g = gap.unwrap();
            if after_light
                && !late
                && self.lane_worst_gap(g, v, now, guard_decel, pmax(guard_accel, strong_accel), guard_reaction, LANE_STRONG_PUSH_MIN_S, long_lag)
                    > goal.min_gap
            {
                self.lane_motor = TRIAL_MOTOR;
            } else if self.lane_worst_gap(g, v, now, guard_decel, guard_accel, guard_reaction, LANE_PUSH_MIN_S, long_lag) > goal.min_gap {
                self.lane_motor = ROUTE_SLOW_MOTOR;
            } else {
                self.lane_motor = NEUTRAL;
            }
            if self.lane_motor != NEUTRAL {
                topup_motor = Some(self.lane_motor);
                self._lane_push_probe = Some((self.lane_motor, now, speed, None, None));
            }
        } else {
            self.lane_motor = NEUTRAL;
        }
        if self.lane_motor != TRIAL_MOTOR && self.lane_motor != LIGHT_LAUNCH_MOTOR && self.finish.launch_until.is_some() {
            self.finish.launch_until = None; // after the launch: the lane's own pushes and kicks
        }
        // the launch's powered run has ended (round y20: its power out of the ESC pipeline too)
        if self.lane_motor == NEUTRAL && self._power_cmd_at.is_none_or(|at| now - at >= ESC_LAG_S + LANE_ESC_SKEW_S) {
            self.finish.launch_from = None;
        }
        if DRIVE_MOTORS.contains(&self.lane_motor) {
            self._lane_topup_motor = topup_motor;
        }
        if self.lane_motor == LANE_BRAKE_MOTOR {
            if self.finish.brake_since.is_none() {
                self.finish.brake_since = Some(now);
            }
        } else if braking {
            (self.finish.brake_since, self.finish.brake_end) = (None, Some(now)); // round y22: once per run
        }
        let frames = if self.finish.creeping { LANE_CREEP_FRAMES } else { LANE_KICK_FRAMES };
        if self.lane_motor == TRIAL_MOTOR && (launching || topup_motor == Some(TRIAL_MOTOR)) {
            self.finish.pulse = 0;
            self.finish.creeping = false;
        } else if self.lane_motor == TRIAL_MOTOR && self.finish.pulse >= frames {
            self.lane_motor = NEUTRAL;
            self.finish.pulse = 0;
            self.finish.creeping = false;
        } else if self.lane_motor == TRIAL_MOTOR {
            if self.finish.pulse == 0 && !kick_room && creep_room {
                self.finish.creeping = true;
                self.finish.creeps += 1;
            }
            self.finish.pulse += 1;
        } else {
            self.finish.pulse = 0;
            self.finish.creeping = false;
        }
        if imu.is_some() {
            feedback.set("imu_speed_mps", imu);
            feedback.set("imu_launch_rise_mps2", launch_rise);
        }
        feedback.set("motor", self.lane_motor);
        feedback.set("creeps", self.finish.creeps);
        feedback.set("brake", V::list(vec![V::from(self.finish.brake_since), V::from(self.finish.brake_end)]));
        (None, feedback)
    }
}
