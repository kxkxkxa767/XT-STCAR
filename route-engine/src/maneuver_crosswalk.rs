//! Round y5: the crosswalk stop in the start lane (maneuver_sequence.py
//! _crosswalk_observe, _crosswalk_control, _crosswalk_stopped; only with
//! control['light']). race4's crosswalk box bottom row gives the near edge's
//! distance (bench 10-28), held on the start lane's front track; the car stops
//! before it and waits, else stops in the first turn and waits there.

use crate::dict;
use crate::helpers::*;
use crate::maneuver::*;
use crate::maneuver_post::pairwise_track_speed;
use crate::py::*;

impl Maneuver {
    pub(crate) fn crosswalk_braking_stage(&self) -> bool {
        self.race_program && self.route_stage == "start_lane" && self.race_inputs.is_some()
            && matches!(self.crosswalk_state, Some("approach" | "fallback"))
    }

    pub(crate) fn crosswalk_brake_ack_expected(&self) -> bool {
        self.race_inputs.is_some()
            && ((self.route_stage == "start_lane"
                    && (self.race_crosswalk_stop.brake.has_braked() || self.race_crosswalk.has_braked()))
                || (self.route_stage == "first_target" && self.race_crosswalk.has_braked())
                || (matches!(self.route_stage, "open_transfer" | "second_loop" | "light_entry")
                    && self.race_transfer.has_braked()))
    }

    fn race_crosswalk_approach(&mut self, scan: &V, now: f64, control: &V,
        gap: Option<f64>, radar_speed: Option<f64>, input: crate::motion_sensors::RaceInputs) -> Option<&'static str> {
        use crate::race_light_brake::{BrakeModel, BRAKE_REST_MPS};
        if self._power_cmd_at.is_none() { return None; }
        let decel = self.lane_decels().0;
        let model = BrakeModel::light(1., decel);
        let applied = control.get("motor").oi().filter(|_| control.get("command_acked").is_true());
        let history = if self._crosswalk_rear == Some(true) { &self._crosswalk_rear_history } else { &self._track_history };
        let range_speed = self.race_crosswalk_stop.brake.observe_range_speed(now, radar_speed, history, decel, model);
        let from_rest = self.race_crosswalk_stop.imu_speed(input.forward_integral);
        let speed = self.race_surface_speed.speed(now, input.forward_integral).or(from_rest);
        let map_axis = self.field_map.axis_speed(scan);
        // Use projected range_speed + map axis speed; when actively braking and lidar/map agree we are at rest,
        // do not let residual IMU drift hold 1300 past standstill.
        let speed = if self.race_crosswalk_stop.brake.is_active()
            && range_speed.is_some_and(|v| v <= BRAKE_REST_MPS)
            && map_axis.is_none_or(|v| v <= 0.06)
        {
            range_speed
        } else {
            [speed, range_speed, map_axis].into_iter().flatten()
                .filter(|v| v.is_finite()).reduce(f64::max)
        };
        let moving = speed.is_some_and(|v| v > BRAKE_REST_MPS);
        let turn_fallback = self.crosswalk_state == Some("fallback");
        let fallback = turn_fallback || gap.is_none();
        let acceleration = applied.filter(|&pwm| pwm > NEUTRAL)
            .and_then(|pwm| {
                self.race_surface.predicted_drive_accel(pwm, speed, now)
                    .or_else(|| if pwm >= START_LAUNCH_MOTOR { Some(5.05) } else if pwm >= 1600 { Some(4.14) } else { None })
            }).unwrap_or(0.).max(0.);
        let tail = if self.race_crosswalk_stop.brake.is_active() {
            0.
        } else {
            speed.map_or(0., |v| {
                let power_resp = 0.056;
                let rise = acceleration * power_resp;
                let scan_rise = range_speed.zip(from_rest).map_or(0., |(radar, inertial)|
                    (radar - inertial).max(0.).min(acceleration * self.race_crosswalk_stop.scan_s.unwrap_or(0.)));
                model.distance(v + scan_rise + rise) - model.distance(v) + 0.5 * rise * power_resp
            })
        };
        let brake = &mut self.race_crosswalk_stop.brake;
        brake.resume_after_forward(applied, range_speed.or(radar_speed));
        brake.observe(now, gap, speed, input.forward_integral, 1., decel, model, CROSSWALK_TARGET_GAP_M + 0.06 + tail);
        if (fallback || gap.is_some_and(|g| g <= CROSSWALK_TARGET_GAP_M))
            && (moving || (speed.is_none() && control.get("motor").oi().is_some_and(|p| p > NEUTRAL))) {
            brake.begin_stop(now);
        }
        // Release 1300 immediately once physical speed is confirmed at rest (<= 0.03 m/s)
        // so the mandatory 3 s crosswalk standstill clock begins without ~0.18 s dead hold.
        if brake.is_active() && speed.is_some_and(|v| v <= 0.03)
            && range_speed.is_none_or(|v| v <= 0.05)
        {
            brake.release_at_rest(now);
        }
        let motor = brake.command(now, input.forward_integral);
        let rolled_out = self._power_cmd_at.is_none_or(|at| now - at >= CROSSWALK_STOP_MAX_S);
        let settled = range_speed.or(radar_speed).is_some_and(|v| v < LANE_SPEED_RESTART_MPS)
            || speed.is_some_and(|v| v <= BRAKE_REST_MPS)
            || rolled_out;
        let fallback_rest = !moving && settled || (rolled_out && radar_speed.is_none());
        if (brake.is_used() && settled) || (fallback && fallback_rest) {
            if fallback || gap.is_some_and(|g| g <= CROSSWALK_ZONE_M) {
                self._start_motor = NEUTRAL;
                let basis = if turn_fallback { "stopped_in_the_first_turn_fallback" }
                    else if gap.is_none() { "stopped_with_the_crosswalk_odometry_lost" }
                    else if gap.is_some_and(|g| g < 0.) { "stopped_past_the_crosswalk_edge" }
                    else { "stopped_before_the_crosswalk" };
                return Some(self.crosswalk_stopped(scan, now, control, gap, basis));
            }
            // A stop short of the stripe uses the existing forward pull-up.
            // The next brake requires acknowledged forward motion.
            self._start_motor = TRIAL_MOTOR;
        } else if let Some(motor) = motor {
            self._start_motor = motor;
        } else if brake.is_used() || fallback {
            self._start_motor = NEUTRAL;
        } else if moving && self._start_motor == NEUTRAL {
            self._start_motor = ROLLING_HOLD_MOTOR;
        }
        None
    }

    /// _lane_rear_range(scan): round y9, the nearest return straight behind
    /// along the compass axis (+-LANE_FRONT_HALF_BINS), or None.
    pub fn lane_rear_range(&self, scan: &V) -> Option<f64> {
        let heading = self.compass.current(scan);
        if let (Some(input), Some(heading)) = (self.race_inputs, heading) {
            let speed = self.race_crosswalk_stop.imu_speed(input.forward_integral)
                .or_else(|| self.race_surface_speed.speed(scan.get("received_at").f(), input.forward_integral));
            if let Some(board) = crate::race_end_board::observe(scan, heading + PI, speed, input.yaw_rate) {
                return Some(board.range);
            }
        }
        let center = 180 + heading.map_or(0, |h| round(degrees(wrap(h))));
        let list = scan.idx("ranges").l();
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
        best
    }

    /// _crosswalk_odometry(scan, track, speed): round y9.
    pub fn crosswalk_odometry(&mut self, scan: &V, track: Option<f64>, speed: Option<f64>) -> (Option<f64>, Option<f64>) {
        if self.crosswalk_state.is_none() && self.light_vision {
            self.crosswalk_state = Some("search");
        }
        if !matches!(self.crosswalk_state, Some("search" | "approach")) {
            return (track, speed);
        }
        let rear = self.lane_rear_range(scan);
        if self._crosswalk_rear.is_none() {
            self._crosswalk_rear = Some(rear.is_some_and(|r| r <= CROSSWALK_REAR_MAX_M));
        }
        let rear_track = rear.filter(|r| *r <= CROSSWALK_REAR_MAX_M).map(|r| -r);
        let race_start = self.race_program && self.route_stage == "start_lane" && self.race_inputs.is_some();
        let field_x = if race_start {
            self.field_map.longitudinal_position(scan)
        } else { None };
        let first_fix = self._crosswalk_rest.is_none();
        let moving_fix = self._crosswalk_board_m.is_some() || !self._crosswalk_fixes.is_empty();
        if self._crosswalk_rear == Some(true) {
            let plan = &mut self.race_crosswalk_stop;
            let mut rear = plan.aligned_track(rear_track, field_x, first_fix, moving_fix);
            if race_start {
                let paired_front = plan.front_on_rear_track(rear, track, self._track_history.len() >= 2);
                if rear.is_none() && plan.has_optical_observation(moving_fix) {
                    rear = paired_front;
                }
            }
            if rear.is_some() || self._crosswalk_board_m.is_some() {
                let measured = pairwise_track_speed(&mut self._crosswalk_rear_history, scan, rear);
                let v = measured.or_else(|| (rear_track.is_none() && rear.is_some())
                    .then(|| self.field_map.axis_speed(scan)).flatten());
                return (rear, v);
            }
            if race_start && self._crosswalk_rest.take().is_some() {
                // Switching to the unpaired front reference. Neither a rear
                // coordinate nor an older camera exposure can be placed on it.
                plan.discard_unpaired_observations();
                self._crosswalk_fixes.clear();
                self._crosswalk_raw.clear();
            }
            self._crosswalk_rear = Some(false);
            self._crosswalk_track.clear();
            self._crosswalk_search_passed = true;
            self.race_crosswalk_stop.change_unplanned_reference();
        }
        let aligned = self.race_crosswalk_stop.aligned_track(track, field_x, first_fix, moving_fix);
        if aligned.is_some() && (track.is_none() || self.race_crosswalk_stop.map_origin()) {
            return (aligned, self.field_map.axis_speed(scan).or(speed));
        }
        (aligned, speed)
    }

    /// _crosswalk_observe(scan, track).
    pub fn crosswalk_observe(&mut self, scan: &V, track: Option<f64>) {
        if self.crosswalk_state.is_none() && self.light_vision {
            self.crosswalk_state = Some("search");
        }
        if !matches!(self.crosswalk_state, Some("search" | "approach")) {
            self._crosswalk_raw.clear();
            return;
        }
        let received = scan.get("received_at").f();
        if let Some(t) = track {
            self._crosswalk_track.push((received, t));
            if self._crosswalk_rest.is_none() {
                self._crosswalk_rest = Some((received, t)); // round y10: the first odometry, at the start
            }
        }
        while self._crosswalk_track.first().is_some_and(|h| h.0 < received - CROSSWALK_TRACK_S) {
            self._crosswalk_track.remove(0);
        }
        // Round y35 (user): the stop planned at rest against the board ahead; from then
        // on the camera is left out (not a start condition: without it, as before).
        if let Some(input) = self.race_inputs {
            let rest = self._crosswalk_rest;
            if let Some(evidence) = self.race_crosswalk_stop.observe(scan, received, track, rest, input.forward_integral) {
                self._crosswalk_board_m = self.race_crosswalk_stop.planned;
                self._crosswalk_board_sigma = Some(0.);
                if self.crosswalk_state == Some("search") {
                    self.crosswalk_state = Some("approach");
                }
                self.route_event("crosswalk_planned_at_rest", scan, &V::None, evidence);
            }
            if self.race_crosswalk_stop.planned.is_some() {
                self._crosswalk_raw.clear();
                return;
            }
        }
        let raw = std::mem::take(&mut self._crosswalk_raw);
        let mut added = false;
        for f in &raw {
            let (frame, captured, score, row, metres) =
                (f.at(0).clone(), f.at(1).f(), f.at(2).clone(), f.at(3).clone(), f.at(4).clone());
            self._crosswalk_seen.push(captured);
            let at = captured - CROSSWALK_CAMERA_LAG_S;
            let rest = self._crosswalk_rest;
            if !metres.is_num() || !(CROSSWALK_RANGE_M.0 <= metres.f() && metres.f() <= CROSSWALK_RANGE_M.1) {
                continue;
            }
            let history = &self._crosswalk_track;
            let place = if !history.is_empty() && at >= history[0].0 {
                let mut place = history[history.len() - 1].1;
                for w in history.windows(2) {
                    let ((t0, x0), (t1, x1)) = (w[0], w[1]);
                    if t0 <= at && at <= t1 {
                        place = if t1 <= t0 { x0 } else { x0 + (x1 - x0) * (at - t0) / (t1 - t0) };
                        break;
                    }
                }
                place
            } else if let Some((rest_at, rest_track)) = rest.filter(|(rest_at, _)| at <= *rest_at) {
                let _ = rest_at;
                rest_track // taken at the standstill before the first output
            } else {
                continue;
            };
            let distance = metres.f();
            let sigma = CROSSWALK_SIGMA_M + CROSSWALK_SIGMA_PER_M2 * distance * distance;
            let board = place - FRONT_BODY_EXTENT_M - (distance - 2. * sigma);
            self._crosswalk_fixes.push(V::list(vec![
                frame,
                V::Float(captured),
                score,
                row,
                V::Float(distance),
                V::Float(sigma),
                V::Float(board),
            ]));
            added = true;
        }
        let n = self._crosswalk_seen.len();
        if n > 8 {
            self._crosswalk_seen.drain(..n - 8);
        }
        let n = self._crosswalk_fixes.len();
        if n > 8 {
            self._crosswalk_fixes.drain(..n - 8);
        }
        let Some(last) = self._crosswalk_fixes.last() else { return };
        let newest = last.at(1).f();
        let n = self._crosswalk_fixes.len();
        // (board, sigma) of the newest fixes, in order (a stable sort keeps ties as Python)
        let mut recent: Vec<(f64, f64)> = self._crosswalk_fixes[n.saturating_sub(CROSSWALK_FIX_FRAMES)..]
            .iter()
            .filter(|f| newest - f.at(1).f() <= CROSSWALK_SEEN_S)
            .map(|f| (f.at(6).f(), f.at(5).f()))
            .collect();
        if !added || recent.len() < CROSSWALK_FIX_FRAMES {
            return;
        }
        let latest = recent[recent.len() - 1];
        recent.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        let mut median = recent[recent.len() / 2];
        // Round y20: the newest frame (the nearest, the sharpest) sets the edge with its
        // own sigmas where it agrees with the median within the median's two.
        if latest.1 < median.1 && (latest.0 - 2. * latest.1 - median.0 + 2. * median.1).abs() <= 2. * median.1 {
            median = latest;
        }
        let (board, sigma) = median;
        // round y11: a fix as precise as the one that set the edge replaces it
        // (the car closing in), a less precise one only brings it nearer
        match (self._crosswalk_board_m, self._crosswalk_board_sigma) {
            (Some(old), Some(old_sigma)) if sigma > old_sigma => self._crosswalk_board_m = Some(pmax(old, board)),
            _ => {
                self._crosswalk_board_m = Some(board);
                self._crosswalk_board_sigma = Some(sigma);
            }
        }
        if self.crosswalk_state == Some("search") {
            self.crosswalk_state = Some("approach");
            let evidence = dict! {
                "source_seq" => scan.idx("seq").clone(), "received_at" => scan.get("received_at").clone(),
                "board_m" => self._crosswalk_board_m, "front_track_m" => track,
                "fixes" => V::list(self._crosswalk_fixes.clone()),
            };
            self.route_event("crosswalk_fixed", scan, &V::None, evidence);
        }
    }

    /// _crosswalk_coast_observe(scan, now, track, speed) (round y15): the coast
    /// after the cut on the board behind (the track is that range negated).
    pub fn crosswalk_coast_observe(&mut self, scan: &V, now: f64, track: Option<f64>, speed: Option<f64>) {
        let coasting = self.crosswalk_state == Some("approach")
            && self._crosswalk_rear == Some(true)
            && self._crosswalk_coasting
            && speed.is_some_and(|s| s >= LANE_SPEED_RESTART_MPS)
            && self.coast_fit_window(now);
        self.coast_measure(scan, coasting, None, track.map(|t| -t));
    }

    /// _crosswalk_control(scan, now, control, track, speed, v): sets _start_motor.
    pub fn crosswalk_control(
        &mut self,
        scan: &V,
        now: f64,
        control: &V,
        track: Option<f64>,
        speed: Option<f64>,
        v: Option<f64>,
    ) -> V {
        let mut state = match self.crosswalk_state {
            Some(s @ ("search" | "approach" | "fallback" | "waiting")) => s,
            _ => return V::None,
        };
        if (state == "search" || state == "approach") && self.first_approach.is_some() {
            state = "fallback";
            self.crosswalk_state = Some(state);
            let evidence = dict! {
                "source_seq" => scan.idx("seq").clone(), "received_at" => scan.get("received_at").clone(),
                "board_m" => self._crosswalk_board_m, "front_track_m" => track,
                "seen" => self._crosswalk_seen.len() as i64, "fixes" => V::list(self._crosswalk_fixes.clone()),
            };
            self.route_event("crosswalk_fallback", scan, control, evidence);
        }
        let gap = match (track, self._crosswalk_board_m) {
            (Some(t), Some(b)) => Some(t - FRONT_BODY_EXTENT_M - b),
            _ => None,
        };
        let stopped = speed.is_some_and(|s| s < LANE_SPEED_RESTART_MPS);
        let rolled_out = self._power_cmd_at.is_none_or(|at| now - at >= CROSSWALK_STOP_MAX_S);
        // round y11: nothing of a 1560/1550 left in the ESC pipeline
        let settled = self._power_cmd_at.is_none_or(|at| now - at >= ESC_LAG_S + LANE_ESC_SKEW_S);
        let (mut aim, mut worst) = (None, None);
        // Round y15: a measured coast aims the stop; the guard takes its weak end
        // where that is above the tile's. In race mode before a coast is measured,
        // use 0.72 m/s^2 (y111-2/3/4 measured ~0.70-0.75) instead of 0.60 to avoid
        // stopping 50-75 cm short of the stripes.
        let measured = self._lane_coast_decel.is_some();
        let default_decel = if self.race_inputs.is_some() { 0.72 } else { CROSSWALK_AIM_DECEL_MPS2 };
        let decel = self._lane_coast_decel.unwrap_or(default_decel);
        let guard_decel = if measured { pmax(CROSSWALK_GUARD_DECEL_MPS2, self.lane_decels().1) } else { CROSSWALK_GUARD_DECEL_MPS2 };
        // Round y35: a cut waits for the next scan unless made on this one, so the travel to
        // it counts (y34's first run cut ~.1 m past its aim at 1.03 m/s).
        let next_scan_m = if self.race_inputs.is_some() {
            v.zip(self.race_crosswalk_stop.scan_s).map_or(0., |(v, dt)| v * dt)
        } else {
            0.
        };
        // y68 (race): a cut left to the next scan also carries that scan's speed gain while the
        // push still accelerates. Cap single-scan IMU spike to 0.18 m/s (y111-2 spiked to
        // next_scan_gain_m=0.758 m and cut 52 cm early).
        let next_scan_gain_m = match (self.race_inputs, v, self.race_crosswalk_stop.scan_s, self.race_crosswalk_stop.imu_accel) {
            (Some(_), Some(v), Some(dt), Some(accel)) if self._start_motor != NEUTRAL => {
                let rise = pmin(0.18, pmax(0., accel) * dt);
                rise * CROSSWALK_AIM_LAG_S + ((v + rise) * (v + rise) - v * v) / (2. * decel)
            }
            _ => 0.,
        };
        if self.race_program && matches!(state, "approach" | "fallback") && let Some(input) = self.race_inputs {
            state = self.race_crosswalk_approach(scan, now, control, gap, speed, input).unwrap_or(state);
        } else if state == "approach" {
            if let (Some(v), Some(g)) = (v, gap) {
                aim = Some(g - v * CROSSWALK_AIM_LAG_S - v * v / (2. * decel));
                worst = Some(g - v * CROSSWALK_GUARD_REACTION_S - v * v / (2. * guard_decel)); // round y20: no speed margin
            }
            match gap {
                None => {
                    self._start_motor = NEUTRAL; // the edge's odometry lost: stop where it is
                    if stopped || rolled_out {
                        state = self.crosswalk_stopped(scan, now, control, gap, "stopped_with_the_crosswalk_odometry_lost");
                    }
                }
                Some(g) if (stopped && settled || rolled_out)
                    && g <= CROSSWALK_ZONE_M
                    && (!self.race_program || self._crosswalk_coasting || g <= CROSSWALK_RACE_TOPUP_GAP_M) => {
                    let basis = if g >= 0. { "stopped_before_the_crosswalk" } else { "stopped_past_the_crosswalk_edge" };
                    state = self.crosswalk_stopped(scan, now, control, gap, basis);
                }
                Some(g) => {
                    if self._crosswalk_coasting && !stopped && !rolled_out {
                        // round y14: push 1550 again while rolling when the coast would stop
                        // short of the target and even the push's worst case stays off the floor
                        // round y15: once this coast's fit (the board behind) is good, the
                        // stop it foresees from its speed and decel, else the aim
                        let coast = match self._coast_fit {
                            Some((d, se, rolling)) if se <= LANE_COAST_DECEL_SE_MAX_MPS2 => {
                                let rolling = pmax(0., rolling);
                                Some(g - rolling * rolling / (2. * pmax(LANE_COAST_DECEL_FLOOR_MPS2, d)))
                            }
                            _ => aim,
                        };
                        let topup = match (coast, v, speed) {
                            (Some(a), Some(v), Some(s)) => {
                                let push = v + LANE_SPEED_GUARD_MARGIN_MPS + LANE_PUSH_ACCEL_MAX_MPS2 * (ESC_LAG_S + LANE_PUSH_MIN_S);
                                let push_worst = g - push * (CROSSWALK_GUARD_REACTION_S + LANE_PUSH_MIN_S)
                                    - push * push / (2. * guard_decel);
                                // y70 (race): the cut made a scan early left .45-.6 m, and 1550 fired twice at
                                // .3 m/s (seq 43, 48: .7 s of creeping). The rules only ask for a stop before
                                // the stripes, so a race top-up is for a stop that would fall far short.
                                let short_of = if self.race_inputs.is_some() { CROSSWALK_RACE_TOPUP_GAP_M }
                                    else { CROSSWALK_TARGET_GAP_M + CROSSWALK_TOPUP_BAND_M };
                                a > short_of
                                    && s >= LANE_PUSH_ROLLING_MPS
                                    && push_worst > CROSSWALK_MIN_GAP_M
                            }
                            _ => false,
                        };
                        self._start_motor = if topup { ROUTE_SLOW_MOTOR } else { NEUTRAL }; // else round y8: one coast
                    } else if aim.is_none_or(|a| a - next_scan_m - next_scan_gain_m <= CROSSWALK_TARGET_GAP_M)
                        || worst.is_some_and(|w| w <= CROSSWALK_MIN_GAP_M)
                    {
                        self._start_motor = NEUTRAL;
                        // round y11: held on a measured speed or once the car has left the start
                        let moved = matches!((self._crosswalk_rest, track),
                                             (Some((_, r)), Some(t)) if (t - r).abs() > CROSSWALK_MOVED_M);
                        self._crosswalk_coasting = speed.is_some() || moved;
                    } else if self._crosswalk_coasting || self._crosswalk_restart || self._crosswalk_kick != 0 {
                        // round y11: kicks only once a coast has ended short of the zone;
                        // before any cut the start lane's own cruise drives
                        self._crosswalk_coasting = false;
                        self._crosswalk_restart = true;
                        let rested = self._power_cmd_at.is_none_or(|at| now - at >= LANE_KICK_REST_S);
                        if self._crosswalk_kick >= LANE_KICK_FRAMES && rested && stopped {
                            self._crosswalk_kick = 0;
                        }
                        if (0 < self._crosswalk_kick && self._crosswalk_kick < LANE_KICK_FRAMES)
                            || (self._crosswalk_kick == 0 && rested && g - LANE_KICK_TRAVEL_M >= CROSSWALK_MIN_GAP_M)
                        {
                            self._start_motor = TRIAL_MOTOR;
                            self._crosswalk_kick += 1;
                        } else {
                            self._start_motor = NEUTRAL;
                        }
                    }
                }
            }
        } else if state == "fallback" {
            self._start_motor = NEUTRAL;
            if stopped || rolled_out {
                state = self.crosswalk_stopped(scan, now, control, gap, "stopped_in_the_first_turn_fallback");
            }
        }
        if state == "waiting" {
            self._start_motor = NEUTRAL;
            let waited_now = pmax(0., now - self._crosswalk_wait_tick.unwrap());
            self._crosswalk_wait_tick = Some(now);
            let since = self.drive_since.unwrap_or_else(|| type_error("unsupported operand type(s) for +=: 'NoneType' and 'float'"));
            self.drive_since = Some(since + waited_now); // standing is not drive time
            self.crosswalk_wait_credit_s += waited_now;
            let stop_at = self._crosswalk_stop_at.unwrap();
            // y68 (race): 3.1 s. y60-y67 showed 3.00-3.10 s (the 10 Hz scan after 3 s); the
            // IMU standstill ran 3.11-3.34 s, y67's 3.11 s the least.
            let wait_s = if self.race_inputs.is_some() { CROSSWALK_RACE_WAIT_S } else { CROSSWALK_WAIT_S };
            if now - stop_at >= wait_s {
                state = "done";
                self.crosswalk_state = Some(state);
                let go = dict! {
                    "source_seq" => scan.idx("seq").clone(), "received_at" => scan.get("received_at").clone(),
                    "waited_s" => now - stop_at,
                };
                let mut evidence = if self.crosswalk_evidence.is_dict() { self.crosswalk_evidence.clone() } else { V::dict() };
                evidence.set("go", go.clone());
                self.crosswalk_evidence = evidence;
                self.route_event("crosswalk_go", scan, control, go);
            }
        }
        let mut out = dict! {
            "state" => state, "gap_m" => gap, "board_m" => self._crosswalk_board_m, "speed_mps" => speed,
            "speed_for_cut_mps" => v, "aimed_stop_gap_m" => aim, "worst_stop_gap_m" => worst,
            "coast_decel_mps2" => decel, "guard_decel_mps2" => guard_decel,
            "coast_fit" => crate::maneuver_post::fit_value(self._coast_fit),
            "motor" => self._start_motor,
        };
        if let Some(input) = self.race_inputs {
            out.set("brake_phase", self.race_crosswalk_stop.brake.phase());
            out.set("planned_at_rest", self.race_crosswalk_stop.planned.is_some());
            out.set("imu_speed_mps", self.race_crosswalk_stop.imu_speed(input.forward_integral));
            out.set("next_scan_m", next_scan_m);
            out.set("next_scan_gain_m", next_scan_gain_m);
            out.set("imu_accel_mps2", self.race_crosswalk_stop.imu_accel);
        }
        out
    }

    /// _crosswalk_stopped(scan, now, control, gap, basis).
    fn crosswalk_stopped(&mut self, scan: &V, now: f64, control: &V, gap: Option<f64>, basis: &str) -> &'static str {
        self.crosswalk_state = Some("waiting");
        self._crosswalk_stop_at = Some(now);
        self._crosswalk_wait_tick = Some(now);
        let stop = dict! {
            "source_seq" => scan.idx("seq").clone(), "received_at" => scan.get("received_at").clone(),
            "basis" => basis, "gap_m" => gap, "board_m" => self._crosswalk_board_m,
            "fixes" => V::list(self._crosswalk_fixes.clone()),
        };
        let mut evidence = if self.crosswalk_evidence.is_dict() { self.crosswalk_evidence.clone() } else { V::dict() };
        evidence.set("stop", stop.clone());
        self.crosswalk_evidence = evidence;
        self.route_event("crosswalk_stop", scan, control, stop);
        "waiting"
    }
}
