//! Round y5: the crosswalk stop in the start lane (maneuver_sequence.py
//! _crosswalk_observe, _crosswalk_control, _crosswalk_stopped; only with
//! control['light']). race4's crosswalk box bottom row gives the near edge's
//! distance (bench 10-28), held on the start lane's front track; the car stops
//! before it and waits, else stops in the first turn and waits there.

use crate::dict;
use crate::helpers::*;
use crate::maneuver::*;
use crate::py::*;

impl Maneuver {
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
        }
        while self._crosswalk_track.first().is_some_and(|h| h.0 < received - CROSSWALK_TRACK_S) {
            self._crosswalk_track.remove(0);
        }
        let raw = std::mem::take(&mut self._crosswalk_raw);
        for f in &raw {
            let (frame, captured, score, row) = (f.at(0).clone(), f.at(1).f(), f.at(2).clone(), f.at(3).clone());
            self._crosswalk_seen.push(captured);
            let at = captured - CROSSWALK_CAMERA_LAG_S;
            let history = &self._crosswalk_track;
            if score.f() < CROSSWALK_RANGE_SCORE
                || !row.is_num()
                || !(CROSSWALK_ROW_MIN_PX <= row.f() && row.f() <= CROSSWALK_ROW_MAX_PX)
                || history.is_empty()
                || at < history[0].0
            {
                continue;
            }
            let mut place = history[history.len() - 1].1;
            for w in history.windows(2) {
                let ((t0, x0), (t1, x1)) = (w[0], w[1]);
                if t0 <= at && at <= t1 {
                    place = if t1 <= t0 { x0 } else { x0 + (x1 - x0) * (at - t0) / (t1 - t0) };
                    break;
                }
            }
            let depth = row.f() - CROSSWALK_ROW_HORIZON_PX;
            let distance = CROSSWALK_ROW_SCALE / depth - CROSSWALK_ROW_OFFSET_M;
            let sigma = CROSSWALK_ROW_SCALE * CROSSWALK_ROW_SIGMA_PX / (depth * depth);
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
        let seen = self._crosswalk_seen.iter().filter(|c| (*c - newest).abs() <= CROSSWALK_SEEN_S).count() as i64;
        if seen < CROSSWALK_SEEN_FRAMES {
            return;
        }
        let mut best = self._crosswalk_fixes.clone();
        best.sort_by(|a, b| {
            (a.at(5).f(), -a.at(1).f()).partial_cmp(&(b.at(5).f(), -b.at(1).f())).unwrap_or(std::cmp::Ordering::Equal)
        });
        best.truncate(2);
        let mut board = best[0].at(6).f();
        for f in &best[1..] {
            board = pmax(board, f.at(6).f());
        }
        self._crosswalk_board_m = Some(board);
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
        let (mut aim, mut worst) = (None, None);
        if state == "approach" {
            if let (Some(v), Some(g)) = (v, gap) {
                aim = Some(g - v * CROSSWALK_AIM_LAG_S - v * v / (2. * CROSSWALK_AIM_DECEL_MPS2));
                let guard = v + LANE_SPEED_GUARD_MARGIN_MPS;
                worst = Some(g - guard * self.reaction_s(now) - guard * guard / (2. * LANE_STOP_DECEL_MIN_MPS2));
            }
            match gap {
                None => {
                    self._start_motor = NEUTRAL; // the edge's odometry lost: stop where it is
                    if stopped || rolled_out {
                        state = self.crosswalk_stopped(scan, now, control, gap, "stopped_with_the_crosswalk_odometry_lost");
                    }
                }
                Some(g) if (stopped || rolled_out) && g <= CROSSWALK_ZONE_M => {
                    let basis = if g >= 0. { "stopped_before_the_crosswalk" } else { "stopped_past_the_crosswalk_edge" };
                    state = self.crosswalk_stopped(scan, now, control, gap, basis);
                }
                Some(g) => {
                    if self._crosswalk_coasting && !stopped && !rolled_out {
                        self._start_motor = NEUTRAL; // round y8: one coast to the stop
                    } else if aim.is_none_or(|a| a <= CROSSWALK_TARGET_GAP_M) || worst.is_some_and(|w| w <= CROSSWALK_MIN_GAP_M) {
                        self._start_motor = NEUTRAL;
                        self._crosswalk_coasting = true;
                    } else if speed.is_none_or(|s| s < LANE_SPEED_RESTART_MPS) || self._crosswalk_kick != 0 {
                        self._crosswalk_coasting = false;
                        // from rest only kicks (the start lane's own launch overshoots)
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
            if now - stop_at >= CROSSWALK_WAIT_S {
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
        dict! {
            "state" => state, "gap_m" => gap, "board_m" => self._crosswalk_board_m, "speed_mps" => speed,
            "speed_for_cut_mps" => v, "aimed_stop_gap_m" => aim, "worst_stop_gap_m" => worst,
            "motor" => self._start_motor,
        }
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
