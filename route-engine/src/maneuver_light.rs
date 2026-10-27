//! Round y2: the traffic-light stop (maneuver_sequence.py _thin_posts and the
//! _light_* methods; only with control['light']). The gantry legs set the
//! light line, the light lane's goal control stops the car before it, and
//! the car waits there for green in fresh frames.

use crate::dict;
use crate::helpers::*;
use crate::maneuver::*;
use crate::py::*;

/// _thin_posts(ranges): isolated thin posts (car frame, left +), nearest first.
pub fn thin_posts(ranges: &[V]) -> Vec<(f64, f64)> {
    let n = ranges.len() as i64;
    let at = |i: i64| &ranges[i as usize];
    let joined = |a: i64, b: i64| at(a).is_num() && at(b).is_num() && (at(a).f() - at(b).f()).abs() <= LIGHT_POST_STEP_M;
    let mut posts: Vec<(f64, f64)> = Vec::new();
    for start in 0..n {
        if !at(start).is_num() || joined(imod(start - 1, n), start) {
            continue;
        }
        let mut run = vec![start];
        while run.len() <= LIGHT_POST_MAX_BINS && joined(*run.last().unwrap(), imod(run.last().unwrap() + 1, n)) {
            run.push(imod(run.last().unwrap() + 1, n));
        }
        if run.len() > LIGHT_POST_MAX_BINS {
            continue;
        }
        let (before_i, after_i) = (imod(run[0] - 1, n), imod(run.last().unwrap() + 1, n));
        if std::iter::once(&before_i).chain(run.iter()).chain(std::iter::once(&after_i)).any(|&i| i == 0 || i == n - 1) {
            continue;
        }
        let far = run.iter().map(|&i| at(i).f()).fold(at(run[0]).f(), pmax);
        let (before, after) = (at(before_i), at(after_i));
        if !(before.is_num()
            && after.is_num()
            && before.f() >= far + LIGHT_POST_JUMP_M
            && after.f() >= far + LIGHT_POST_JUMP_M)
        {
            continue;
        }
        let points: Vec<(f64, f64)> =
            run.iter().map(|&i| (at(i).f() * dcos(i as usize), -at(i).f() * dsin(i as usize))).collect();
        if points.iter().any(|&a| points.iter().any(|&b| dist(a, b) > LIGHT_POST_MAX_EXTENT_M)) {
            continue;
        }
        let count = points.len() as f64;
        posts.push((fsum_py(points.iter().map(|p| p.0)) / count, fsum_py(points.iter().map(|p| p.1)) / count));
    }
    posts.sort_by(|a, b| (a.0 * a.0 + a.1 * a.1).partial_cmp(&(b.0 * b.0 + b.1 * b.1)).unwrap());
    posts
}

/// _light_leg_matches(posts, legs, reach, side): (leg index, post) pairs in
/// assignment order, one post per leg and one leg per post, nearest in lateral
/// first, in one along window for the gantry.
pub fn light_leg_matches(posts: &[(f64, f64)], legs: &[(f64, f64)], reach: f64, side: f64) -> Vec<(usize, (f64, f64))> {
    let low = legs.iter().map(|leg| leg.0).fold(legs[0].0, pmin) - reach;
    let high = legs.iter().map(|leg| leg.0).fold(legs[0].0, pmax) + LIGHT_LEG_ALONG_M;
    let mut candidates: Vec<(f64, f64, usize, usize)> = Vec::new();
    for (k, leg) in legs.iter().enumerate() {
        for (j, post) in posts.iter().enumerate() {
            if (post.1 - leg.1).abs() <= side && low <= post.0 && post.0 <= high {
                candidates.push(((post.1 - leg.1).abs(), (post.0 - leg.0).abs(), k, j));
            }
        }
    }
    candidates.sort_by(|a, b| {
        a.0.partial_cmp(&b.0).unwrap().then(a.1.partial_cmp(&b.1).unwrap()).then(a.2.cmp(&b.2)).then(a.3.cmp(&b.3))
    });
    let mut matches: Vec<(usize, (f64, f64))> = Vec::new();
    let mut used: Vec<usize> = Vec::new();
    for (_, _, k, j) in candidates {
        if !matches.iter().any(|(mk, _)| *mk == k) && !used.contains(&j) {
            matches.push((k, posts[j]));
            used.push(j);
        }
    }
    matches
}

/// The posts in the lane frame (along, lateral) for compass heading ``heading``.
fn lane_posts(ranges: &[V], heading: f64) -> Vec<(f64, f64)> {
    let u = ((-heading).pcos(), (-heading).psin());
    thin_posts(ranges).into_iter().map(|(x, y)| (x * u.0 + y * u.1, y * u.0 - x * u.1)).collect()
}

fn legs_v(legs: &[(f64, f64)]) -> V {
    V::list(legs.iter().map(|&leg| V::from(leg)).collect())
}

fn legs_of(v: &V) -> Vec<(f64, f64)> {
    v.l().iter().map(|leg| (leg.at(0).f(), leg.at(1).f())).collect()
}

/// `(d.get(key) or []) + [item]`.
fn appended(d: &V, key: &str, item: V) -> V {
    let mut items = if d.get(key).truthy() { d.get(key).l().to_vec() } else { Vec::new() };
    items.push(item);
    V::list(items)
}

struct Gantry {
    along_m: f64,
    legs: Vec<(f64, f64)>,
    basis: &'static str,
}

impl Maneuver {
    /// _light_stop_active().
    pub fn light_stop_active(&self) -> bool {
        matches!(self.light_stop_state, Some("approach" | "waiting"))
    }

    /// _light_entry_coast_ack_dropped(): a centre ACK proven at neutral
    /// authorizes right in the light entry only while its light stop lasts.
    pub fn light_entry_coast_ack_dropped(&mut self) {
        if self.route_stage == "light_entry"
            && !self.light_stop_active()
            && self._route_center_ack.is_dict()
            && eq_i(self._route_center_ack.get("motor"), NEUTRAL)
        {
            self._route_center_ack = V::None;
        }
    }

    /// _light_observe(control): the newest frame of control['light'], once per frame.
    pub fn light_observe(&mut self, control: &V) {
        let light = if control.is_dict() { control.get("light").clone() } else { V::None };
        if !light.is_dict() {
            return;
        }
        self.light_vision = true;
        let (frame, captured) = (light.get("frame"), light.get("captured_at"));
        if !frame.is_int() || !captured.is_num() || self._light_frames.last().is_some_and(|f| py_eq(f.at(0), frame)) {
            return;
        }
        let score = |v: &V| if v.is_num() { v.clone() } else { V::Float(0.) };
        let (red, green) = (score(light.get("red")), score(light.get("green")));
        self._light_frames.push(V::list(vec![frame.clone(), captured.clone(), red.clone(), green.clone()]));
        let n = self._light_frames.len();
        let keep = LIGHT_GREEN_FRAMES.max(LIGHT_RED_GONE_FRAMES) + 2;
        if n > keep {
            self._light_frames.drain(..n - keep);
        }
        if self.light_stop_active() && captured.f() >= self._light_gantry_since.unwrap() {
            if red.f() >= LIGHT_RED_MIN_SCORE {
                self._light_red_frames += 1;
                if self._light_red_at.is_none() {
                    self._light_red_at = Some(captured.f());
                }
            }
            if red.f() >= LIGHT_RED_MIN_SCORE || green.f() >= LIGHT_RED_MIN_SCORE {
                self._light_seen = true;
            }
            if self.light_stop_state == Some("waiting")
                && captured.f() >= self._light_wait_since.unwrap()
                && red.f() >= LIGHT_GREEN_MIN_SCORE
            {
                self._light_stop_red_frames += 1;
            }
        }
    }

    /// _light_lane_band(lane): (low, high) lateral the gantry legs may stand at.
    fn light_lane_band(lane: &V) -> Option<(f64, f64)> {
        if !lane.is_dict() {
            return None;
        }
        let outer = lane.idx("outer_lateral_m").f();
        let inner = lane.idx("inner_lateral_m");
        let inner = if inner.is_some() { inner.f() } else { outer - LIGHT_LANE_MAX_WIDTH_M };
        Some((inner - LIGHT_GANTRY_EDGE_M, outer + LIGHT_GANTRY_EDGE_M))
    }

    /// _light_gantry(scan, heading, lane): the nearest gantry ahead.
    fn light_gantry(&self, scan: &V, heading: f64, lane: &V) -> Option<Gantry> {
        let band = Self::light_lane_band(lane)?;
        if !scan.get("ranges").is_list() {
            return None;
        }
        let legs: Vec<(f64, f64)> = lane_posts(scan.idx("ranges").l(), heading)
            .into_iter()
            .filter(|&(along, lateral)| {
                FRONT_BODY_EXTENT_M < along && along <= LIGHT_GANTRY_REACH_M && band.0 <= lateral && lateral <= band.1
            })
            .collect();
        let mut best: Option<(f64, Vec<(f64, f64)>)> = None;
        for (i, &a) in legs.iter().enumerate() {
            for &b in &legs[i + 1..] {
                if LIGHT_GANTRY_SPAN_M.0 <= (a.1 - b.1).abs()
                    && (a.1 - b.1).abs() <= LIGHT_GANTRY_SPAN_M.1
                    && (a.0 - b.0).abs() <= LIGHT_GANTRY_ALONG_DIFF_M
                    && best.as_ref().is_none_or(|bst| pmin(a.0, b.0) < bst.0)
                {
                    best = Some((pmin(a.0, b.0), if a.1 >= b.1 { vec![a, b] } else { vec![b, a] }));
                }
            }
        }
        if let Some((along_m, legs)) = best {
            return Some(Gantry { along_m, legs, basis: "two_legs" });
        }
        if heading.abs() > light_gantry_max_heading_rad() {
            return None;
        }
        let outer = lane.idx("outer_lateral_m").f();
        let sides = (outer - LIGHT_LEG_INSIDE_M.1, outer - LIGHT_LEG_INSIDE_M.0);
        let inner = lane.idx("inner_lateral_m");
        for &(along, lateral) in &legs {
            if ((sides.0 <= lateral && lateral <= sides.1)
                || (inner.is_some()
                    && inner.f() + LIGHT_LEG_INSIDE_M.0 <= lateral
                    && lateral <= inner.f() + LIGHT_LEG_INSIDE_M.1))
                && best.as_ref().is_none_or(|bst| along < bst.0)
            {
                best = Some((along, vec![(along, lateral)]));
            }
        }
        best.map(|(along_m, legs)| Gantry { along_m, legs, basis: "one_leg_inside_a_board" })
    }

    /// _light_leg_followed(posts, legs, along_m, since): (line along, legs moved).
    fn light_leg_followed(posts: &[(f64, f64)], legs: &[(f64, f64)], along_m: f64, since: f64) -> Option<(f64, Vec<(f64, f64)>)> {
        let reach = LANE_MAX_CLOSING_MPS * since + LIGHT_LEG_ALONG_M;
        let side = LIGHT_LEG_LATERAL_M + LIGHT_LEG_LATERAL_MPS * since;
        // the leg nearest its own lateral (the first assignment)
        let (k, (along, lateral)) = *light_leg_matches(posts, legs, reach, side).first()?;
        let (moved, sideways) = (along - legs[k].0, lateral - legs[k].1);
        Some((along_m + moved, legs.iter().map(|leg| (leg.0 + moved, leg.1 + sideways)).collect()))
    }

    /// _light_line_held(at, now): the line from its last measurement, or None.
    /// _light_center_offset(legs, lane, current): the gantry's centre less the
    /// lane's centre (left +), bounded; ``current`` without two legs or the lane.
    fn light_center_offset(legs: &[(f64, f64)], lane: &V, current: f64) -> f64 {
        if legs.len() != 2 || !lane.is_dict() {
            return current;
        }
        let shift = (legs[0].1 + legs[1].1) / 2. - lane.idx("center_lateral_m").f();
        pmax(-LIGHT_CENTER_MAX_SHIFT_M, pmin(LIGHT_CENTER_MAX_SHIFT_M, shift))
    }

    fn light_line_held(&self, at: f64, now: f64) -> Option<f64> {
        let line = &self._light_line;
        let dt = at - line.idx("at_s").f();
        let v0 = if line.idx("speed_mps").is_some() { line.idx("speed_mps").f() } else { LANE_SPEED_UNKNOWN_MPS };
        let coasting = self.lane_motor == NEUTRAL && self._power_cmd_at.is_none_or(|cmd| now - cmd >= ESC_LAG_S);
        let (travel, limit);
        if coasting {
            let decel = self.lane_decels().1;
            let stop_s = v0 / decel;
            let moving = pmin(dt, stop_s);
            travel = v0 * moving - decel * moving * moving / 2.;
            let in_zone = line.idx("along_m").f() - FRONT_BODY_EXTENT_M <= LIGHT_STOP_ZONE_GAP_M;
            limit = if in_zone { stop_s + LIGHT_LINE_HOLD_S } else { LIGHT_LINE_HOLD_S };
        } else {
            (travel, limit) = (v0 * dt, LIGHT_LINE_HOLD_S);
        }
        if dt > limit { None } else { Some(line.idx("along_m").f() - travel) }
    }

    /// _light_line_ahead(scan, heading, lane, now): lidar to the light line along the axis.
    fn light_line_ahead(&mut self, scan: &V, heading: f64, lane: &V, now: f64) -> Option<f64> {
        let at = scan.idx("at_ms").i() as f64 / 1000.;
        let board = self.lane_front_range(scan);
        if self._light_line.is_none() {
            let mut gantry = self.light_gantry(scan, heading, lane);
            if let (Some(g), Some(rearm)) = (&gantry, self._light_rearm_min_m)
                && g.along_m < rearm
            {
                gantry = None; // the posts of a stop that was not at the light
            }
            // both legs of the last pair, followed, before one leg alone
            if gantry.as_ref().is_none_or(|g| g.legs.len() == 1)
                && let Some(last) = self._light_gantry_seen.last()
                && ["two_legs", "leg_followed"].contains(&last.idx("basis").s())
            {
                let posts = lane_posts(scan.idx("ranges").l(), heading);
                let followed = Self::light_leg_followed(
                    &posts,
                    &legs_of(last.idx("legs")),
                    last.idx("along_m").f(),
                    at - last.idx("at_s").f(),
                );
                if let Some((along_m, legs)) = followed {
                    gantry = Some(Gantry { along_m, legs, basis: "leg_followed" });
                }
            }
            let gantry = gantry?;
            if let Some(last) = self._light_gantry_seen.last() {
                let dt = at - last.idx("at_s").f();
                let closer = last.idx("along_m").f() - gantry.along_m;
                if !(0. < dt
                    && dt <= LIGHT_GANTRY_STEP_S
                    && -LIGHT_LEG_ALONG_M <= closer
                    && closer <= LANE_MAX_CLOSING_MPS * dt + LIGHT_LEG_ALONG_M)
                {
                    self._light_gantry_seen.clear();
                }
            }
            self._light_gantry_seen.push(dict! {
                "at_s" => at, "received_at" => scan.idx("received_at").clone(), "seq" => scan.idx("seq").clone(),
                "along_m" => gantry.along_m, "legs" => legs_v(&gantry.legs), "board_m" => board,
                "basis" => gantry.basis,
            });
            let seen = &self._light_gantry_seen;
            let tail = |count: usize| &seen[seen.len().saturating_sub(count)..];
            let paired = seen.len() >= LIGHT_GANTRY_PAIR_CONFIRM
                && tail(LIGHT_GANTRY_PAIR_CONFIRM).iter().any(|row| row.idx("basis").s() == "two_legs");
            if !paired
                && (seen.len() < LIGHT_GANTRY_CONFIRM
                    || tail(LIGHT_GANTRY_CONFIRM).iter().any(|row| row.idx("basis").s() != "one_leg_inside_a_board"))
            {
                return None;
            }
            let near = gantry.along_m;
            self._light_line = dict! {
                "along_m" => near, "at_s" => at, "source" => "legs", "confirmed_seq" => scan.idx("seq").clone(),
                "legs" => legs_v(&gantry.legs), "speed_mps" => V::None, "board_m" => board,
                "leg_offsets_m" => V::list(gantry.legs.iter().map(|leg| V::Float(leg.0 - near)).collect()),
            };
            self._light_center_shift_m = Self::light_center_offset(&gantry.legs, lane, 0.);
            return Some(near);
        }
        let posts = lane_posts(scan.idx("ranges").l(), heading);
        let mut line = self._light_line.clone();
        let since = pmax(0., at - line.idx("at_s").f());
        let reach = LANE_MAX_CLOSING_MPS * since + LIGHT_LEG_ALONG_M;
        let side = LIGHT_LEG_LATERAL_M + LIGHT_LEG_LATERAL_MPS * since;
        let mut legs = legs_of(line.idx("legs"));
        let offsets: Vec<f64> = line.idx("leg_offsets_m").l().iter().map(|o| o.f()).collect();
        let (mut matched, mut near, mut shifts): (Vec<usize>, Option<f64>, Vec<f64>) = (Vec::new(), None, Vec::new());
        let matches = light_leg_matches(&posts, &legs, reach, side);
        for k in 0..legs.len() {
            let leg = legs[k];
            if let Some(&(_, m)) = matches.iter().find(|(mk, _)| *mk == k) {
                matched.push(k);
                shifts.push(m.1 - leg.1);
                legs[k] = m;
                let value = m.0 - offsets[k];
                near = Some(match near {
                    None => value,
                    Some(n) => pmin(n, value),
                });
            }
        }
        let Some(near) = near else {
            // the last leg measurement stays the reference
            let reference = line.idx("board_m");
            if let Some(board) = board
                && reference.is_some()
                && -LIGHT_LINE_JUMP_M <= reference.f() - board
                && reference.f() - board <= LANE_MAX_CLOSING_MPS * since + LIGHT_LINE_JUMP_M
            {
                let value = line.idx("along_m").f() - (reference.f() - board);
                self._light_line.set("source", "board_odometry");
                return Some(value);
            }
            let held = self.light_line_held(at, now);
            self._light_line.set("source", if held.is_none() { V::None } else { V::from("held") });
            return held;
        };
        let moved = near - line.idx("along_m").f();
        let sideways = if shifts.is_empty() { 0. } else { fsum_py(shifts.iter().copied()) / shifts.len() as f64 };
        for (k, leg) in legs.iter_mut().enumerate() {
            if !matched.contains(&k) {
                leg.0 += moved; // an unseen leg moves with the line
                leg.1 += sideways;
            }
        }
        if legs.len() == 1
            && let Some(pair) = self.light_gantry(scan, heading, lane)
            && pair.basis == "two_legs"
            && pair.legs.contains(&legs[0])
        {
            // a line on one leg takes the pair that leg is part of now
            legs = pair.legs;
            matched = vec![0, 1];
        }
        let mut near = near;
        if legs.len() == 2 && matched.len() == 2 {
            // both legs seen: the nearer one, and their offsets to it now
            near = pmin(legs[0].0, legs[1].0);
            line.set("leg_offsets_m", V::list(legs.iter().map(|leg| V::Float(leg.0 - near)).collect()));
            self._light_center_shift_m = Self::light_center_offset(&legs, lane, self._light_center_shift_m);
        }
        let held_speed = self._lane_speed_last;
        line.update(&dict! {
            "along_m" => near, "at_s" => at, "source" => "legs", "board_m" => board,
            "speed_mps" => match held_speed {
                Some(h) if now - h.0 <= LANE_SPEED_HOLD_S => V::Float(h.1),
                _ => V::None,
            },
        });
        line.set("legs", legs_v(&legs));
        self._light_line = line;
        Some(near)
    }

    /// _light_stop_track(scan, now, control, heading, lane): start the light
    /// stop on a confirmed gantry; the light line ahead while approaching.
    pub fn light_stop_track(&mut self, scan: &V, now: f64, control: &V, heading: Option<f64>, lane: &V) -> Option<f64> {
        let heading = heading?;
        if !self.light_vision || matches!(self.light_stop_state, Some("waiting" | "released")) {
            return None;
        }
        let ahead = self.light_line_ahead(scan, heading, lane, now);
        let approach = self.light_stop_state == Some("approach");
        let passed = approach
            && ahead.is_some_and(|a| a - FRONT_BODY_EXTENT_M < -LIGHT_PASSED_M)
            && matches!(self._light_line.idx("source").os(), Some("legs" | "board_odometry"));
        if passed
            || (ahead.is_none()
                && approach
                && self._light_line.idx("along_m").f() - FRONT_BODY_EXTENT_M > LIGHT_STOP_ZONE_GAP_M)
        {
            let key = if passed { "passed" } else { "lost" };
            let ended = dict! {
                "source_seq" => scan.idx("seq").clone(), "received_at" => scan.idx("received_at").clone(),
                "route_stage" => self.route_stage, "line" => self._light_line.clone(),
            };
            let rows = appended(&self.light_stop_evidence, key, ended);
            self.light_stop_evidence = self.light_stop_evidence.merged(&dict! {key => rows});
            self.light_stop_state = None;
            self._light_line = V::None;
            self._light_gantry_seen.clear();
            if passed {
                self._light_rearm_min_m = Some(LIGHT_REARM_MIN_M);
            }
            self.lane_speed(scan, None, 2);
            self.light_entry_coast_ack_dropped();
            if self.route_stage != "light_lane" {
                self.lane_motor = ROUTE_SLOW_MOTOR;
            }
            return None;
        }
        if ahead.is_none() || self.light_stop_state == Some("approach") {
            return ahead;
        }
        let evidence = dict! {
            "source_seq" => scan.idx("seq").clone(), "received_at" => scan.idx("received_at").clone(),
            "route_stage" => self.route_stage, "compass_heading_rad" => heading, "line" => self._light_line.clone(),
            "seen" => V::list(self._light_gantry_seen.clone()),
            "basis" => "gantry_legs_confirmed_on_consecutive_scans",
        };
        self.light_stop_state = Some("approach");
        (self._light_gantry_since, self._light_red_frames, self._light_seen) = (Some(now), 0, false);
        self._light_red_at = None;
        let base = if self.light_stop_evidence.truthy() { self.light_stop_evidence.clone() } else { V::dict() };
        self.light_stop_evidence = base.merged(&dict! {"gantry" => evidence.clone()});
        self.route_event("light_gantry", scan, control, evidence);
        // The approach's speed is the closing rate on the line itself (the
        // seeded rows end with this scan's own: the history restarts on it).
        self._lane_ranges = self
            ._light_gantry_seen
            .iter()
            .map(|row| (row.idx("at_s").f(), row.idx("received_at").f(), row.idx("along_m").f()))
            .collect();
        self._lane_speed_last = None;
        self._lane_coast_track.clear();
        (self._lane_pulse, self._lane_creep, self._lane_creeps) = (0, false, 0);
        if self.route_stage != "light_lane" {
            self.lane_motor = ROUTE_SLOW_MOTOR; // the light entry drives at 1550 until the coast is aimed
        }
        ahead
    }

    /// _light_stop_reached(scan, now, control, basis).
    pub fn light_stop_reached(&mut self, scan: &V, now: f64, control: &V, basis: &str) {
        let stop = self.lane_feedback.merged(&dict! {
            "basis" => basis, "received_at" => scan.idx("received_at").clone(),
            "route_stage" => self.route_stage, "light_line" => self._light_line.clone(),
            "zone_m" => LIGHT_ZONE_M, "stop_zone_certified" => false,
        });
        self.light_stop_state = Some("waiting");
        self._light_wait_since = Some(now);
        self._light_wait_tick = Some(now);
        self._light_stop_red_frames = 0;
        self.lane_motor = NEUTRAL;
        (self._lane_pulse, self._lane_creep) = (0, false);
        self.light_stop_evidence = self.light_stop_evidence.merged(&dict! {"stop" => stop.clone()});
        self.route_event("light_stop", scan, control, stop);
        self.lane_speed(scan, None, 2); // the wait follows the board ahead: the restart's speed
    }

    /// _light_wait(scan, now, control): True once released.
    pub fn light_wait(&mut self, scan: &V, now: f64, control: &V) -> bool {
        let waited_now = pmax(0., now - self._light_wait_tick.unwrap());
        self._light_wait_tick = Some(now);
        let since = self.drive_since.unwrap_or_else(|| type_error("unsupported operand type(s) for +=: 'NoneType' and 'float'"));
        self.drive_since = Some(since + waited_now); // standing is not drive time
        self.light_wait_credit_s += waited_now;
        self.lane_motor = NEUTRAL;
        let wait_since = self._light_wait_since.unwrap();
        let frames: Vec<V> = self._light_frames[self._light_frames.len().saturating_sub(LIGHT_GREEN_FRAMES)..].to_vec();
        let green = frames.len() == LIGHT_GREEN_FRAMES
            && frames.iter().all(|f| {
                f.at(1).f() >= wait_since
                    && now - f.at(1).f() <= LIGHT_FRAME_MAX_AGE_S
                    && f.at(3).f() >= LIGHT_GREEN_MIN_SCORE
                    && f.at(2).f() < LIGHT_RED_MIN_SCORE
            });
        let recent: Vec<&V> = self._light_frames[self._light_frames.len().saturating_sub(LIGHT_RED_GONE_FRAMES)..]
            .iter()
            .filter(|f| f.at(1).f() >= wait_since && now - f.at(1).f() <= LIGHT_FRAME_MAX_AGE_S)
            .collect();
        let red_now = recent.iter().any(|f| f.at(2).f() >= LIGHT_RED_MIN_SCORE);
        let red_ended =
            self._light_stop_red_frames >= LIGHT_RED_STOP_FRAMES && recent.len() == LIGHT_RED_GONE_FRAMES && !red_now;
        let red_s = self._light_red_at.map(|at| now - at);
        let red_timed = red_s.is_some_and(|s| s >= LIGHT_RED_MAX_S || (s >= LIGHT_RED_GO_S && !red_now));
        let waited = now - wait_since;
        let no_light = !self._light_seen && waited >= LIGHT_NO_LIGHT_WAIT_S;
        if !green && !red_ended && !red_timed && !no_light && waited < LIGHT_WAIT_MAX_S {
            // the board ahead's history for the restart
            let ahead = self.lane_front_range(scan);
            self.lane_speed(scan, ahead, 2);
            return false;
        }
        let board = self.lane_front_range(scan);
        if self.route_stage == "light_lane" && board.is_some_and(|b| b - FRONT_BODY_EXTENT_M >= LIGHT_LAUNCH_MIN_GAP_M) {
            // (in the light entry the open stages' own resume launch applies)
            self._light_launch_until = Some(now + ESC_LAG_S + LIGHT_LAUNCH_S);
            self._light_launch_from = Some(now);
        }
        let basis = if green && self._light_red_frames != 0 {
            "green_in_fresh_frames_after_the_stop"
        } else if green {
            "green_without_red_since_the_gantry_looking_again"
        } else if red_ended {
            "red_gone_from_fresh_frames_after_the_stop"
        } else if red_timed && !red_now {
            "green_not_seen_red_go_time"
        } else if red_timed {
            "red_still_seen_red_max_time"
        } else if no_light {
            "no_light_seen_since_the_gantry_looking_again"
        } else {
            "green_not_seen_light_wait_max"
        };
        let go = dict! {
            "source_seq" => scan.idx("seq").clone(), "received_at" => scan.idx("received_at").clone(),
            "waited_s" => waited, "basis" => basis,
            "frames" => V::list(self._light_frames[self._light_frames.len().saturating_sub(LIGHT_GREEN_FRAMES.max(LIGHT_RED_GONE_FRAMES))..].to_vec()),
            "red_frames_since_gantry" => self._light_red_frames, "red_frames_since_stop" => self._light_stop_red_frames,
            "red_since_s" => red_s, "route_stage" => self.route_stage,
            "launch" => self._light_launch_until.is_some(),
        };
        self.light_stop_evidence = self.light_stop_evidence.merged(&dict! {"go" => go.clone()});
        self.route_event("light_go", scan, control, go.clone());
        self._lane_creeps = 0;
        if (no_light && !green) || (green && self._light_red_frames == 0) {
            // not the light: a later gantry may start a new stop
            let rearmed = appended(&self.light_stop_evidence, "rearmed", go);
            self.light_stop_evidence = self.light_stop_evidence.merged(&dict! {"rearmed" => rearmed});
            self.light_stop_state = None;
            self._light_line = V::None;
            self._light_gantry_seen.clear();
            self._light_rearm_min_m = Some(LIGHT_REARM_MIN_M);
        } else {
            self.light_stop_state = Some("released");
        }
        self.light_entry_coast_ack_dropped();
        true
    }
}

#[cfg(test)]
mod tests {
    //! The light-entry frames of car run 108 (seq 139..153) and the Python
    //! results in web/vehicle-console/tests/fixtures/light_gantry_run108_frames.json.
    use super::*;
    use crate::py::parse_json;

    fn fixture() -> V {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../web/vehicle-console/tests/fixtures/light_gantry_run108_frames.json"
        );
        parse_json(&std::fs::read_to_string(path).unwrap()).unwrap()
    }

    #[test]
    fn posts_and_gantries_match_python() {
        let fixture = fixture();
        let m = Maneuver::new(&V::Float(0.), &V::Float(40.), &V::Int(1670), &V::Bool(true));
        let mut found = 0;
        for frame in fixture.idx("frames").l() {
            let python = frame.idx("python");
            let posts = thin_posts(frame.idx("ranges").l());
            let expected = python.idx("posts").l();
            assert_eq!(posts.len(), expected.len(), "seq {}", frame.idx("seq").i());
            for (p, e) in posts.iter().zip(expected) {
                assert_eq!(p.0.to_bits(), e.at(0).f().to_bits());
                assert_eq!(p.1.to_bits(), e.at(1).f().to_bits());
            }
            let scan = dict! {"ranges" => frame.idx("ranges").clone()};
            let gantry = m.light_gantry(&scan, frame.idx("compass_heading_rad").f(), frame.idx("lane_target"));
            let expected = python.idx("gantry");
            assert_eq!(gantry.is_some(), expected.is_some(), "seq {}", frame.idx("seq").i());
            if let Some(g) = gantry {
                found += 1;
                assert_eq!(g.along_m.to_bits(), expected.idx("along_m").f().to_bits());
                assert_eq!(g.basis, expected.idx("basis").s());
                assert!(same(&legs_v(&g.legs), expected.idx("legs")));
            }
        }
        assert_eq!(found, 7);
    }

    #[test]
    fn one_post_goes_to_the_leg_nearest_its_lateral() {
        // as tests/test_route_y2_light_stop.py: both windows reach it, the inner leg takes it
        let legs = [(2.0, 0.45), (2.0, -0.45)];
        assert_eq!(light_leg_matches(&[(1.9, -0.4)], &legs, 4., 1.15), vec![(1, (1.9, -0.4))]);
        let both = light_leg_matches(&[(1.9, -0.4), (1.95, 0.5)], &legs, 4., 1.15);
        assert_eq!(both, vec![(0, (1.95, 0.5)), (1, (1.9, -0.4))]); // equal laterals: leg order, as Python
        assert!(light_leg_matches(&[(2.5, 0.45)], &legs, 4., 1.15).is_empty()); // beyond the gantry's window
    }
}
