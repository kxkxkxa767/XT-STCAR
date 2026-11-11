//! Round y2: the traffic-light stop (maneuver_sequence.py _thin_posts and the
//! _light_* methods; only with control['light']). The gantry legs set the
//! light line, the light lane's goal control stops the car before it, and
//! the car waits there for green in fresh frames.

use crate::dict;
use crate::helpers::*;
use crate::maneuver::*;
use crate::py::*;
use crate::race_light::{LightRoll, at_predicted_beam, station_pair};
use crate::race_light_brake::{LIGHT_BRAKE_MOTOR, LIGHT_BRAKE_RESPONSE_S, BrakeModel};

// Pre-red approach settings, independent of the terminal board controller.
const LIGHT_RESTART_MPS: f64 = 0.15;
const NEAR_POST_EDGE_M: f64 = 0.04; // measured close to both boards at the static gantry pose
const VISIBLE_LIGHT_STOP_GAP_M: f64 = 0.55; // head visible at 0.52 m, out of frame at the y29 stop at 0.20 m
// y69: a coast before the beam is revoked once the renewed stop is this far past the visible
// point. Coasting before the beam, renew peaked .70-1.40 m in the runs that stopped short or
// re-pushed (y59 .92, y60 .98, y61 .76, y63 .80, y63-2 .71, y68 1.40 from a cut at seq 162) and at
// most .59 m in the good stops (y64 .44, y66 .59, y67 .47).
const LIGHT_COAST_REVOKE_BAND_M: f64 = 0.15;
// y59 frames94/95 visibly show green and score0.356/0.302. After observed red,
// accept the detector's existing publication threshold, still on two fresh
// consecutive green frames with no red in either frame.
const RACE_GREEN_AFTER_RED_MIN_SCORE: f64 = 0.25;
// y34: the practice red follows the beam by 1.0-1.3 s, then a 3.6 Hz frame and
// inference; y32 decided it 1.36 s after the crossing but had refilled at 1.0 s.
const LIGHT_RED_DUE_S: f64 = 2.0;

/// _thin_posts(ranges): isolated thin posts (car frame, left +), nearest first.
pub fn thin_posts(ranges: &[V]) -> Vec<(f64, f64)> {
    post_runs(ranges, LIGHT_POST_STEP_M, LIGHT_POST_JUMP_M)
}

fn post_runs(ranges: &[V], step: f64, jump: f64) -> Vec<(f64, f64)> {
    let mut posts = Vec::new();
    visit_posts(ranges, step, jump, |p, _| posts.push(p));
    posts.sort_by(|a, b| (a.0 * a.0 + a.1 * a.1).total_cmp(&(b.0 * b.0 + b.1 * b.1)));
    posts
}

fn visit_posts(ranges: &[V], step: f64, jump: f64, mut visit: impl FnMut((f64, f64), f64)) {
    let n = ranges.len() as i64;
    let at = |i: i64| &ranges[i as usize];
    let joined = |a: i64, b: i64| at(a).is_num() && at(b).is_num() && (at(a).f() - at(b).f()).abs() <= step;
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
            && before.f() >= far + jump
            && after.f() >= far + jump)
        {
            continue;
        }
        let points: Vec<(f64, f64)> =
            run.iter().map(|&i| (at(i).f() * dcos(i as usize), -at(i).f() * dsin(i as usize))).collect();
        if points.iter().any(|&a| points.iter().any(|&b| dist(a, b) > LIGHT_POST_MAX_EXTENT_M)) {
            continue;
        }
        let count = points.len() as f64;
        visit((fsum_py(points.iter().map(|p| p.0)) / count, fsum_py(points.iter().map(|p| p.1)) / count),
              before.f().min(after.f()) - far);
    }
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
    project_posts(thin_posts(ranges), heading)
}

fn project_posts(posts: Vec<(f64, f64)>, heading: f64) -> Vec<(f64, f64)> {
    let u = ((-heading).pcos(), (-heading).psin());
    posts.into_iter().map(|(x, y)| (x * u.0 + y * u.1, y * u.0 - x * u.1)).collect()
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
    beam_along: Option<f64>,
}

fn two_legs(pair: crate::race_light::StationPair) -> Gantry {
    Gantry { along_m: pair.along(), legs: pair.legs.to_vec(), basis: "two_legs", beam_along: pair.beam_along }
}

impl Maneuver {
    /// _light_stop_active().
    pub fn light_stop_active(&self) -> bool {
        self.light_approaching() || self.light_stop_state == Some("waiting")
    }

    pub fn light_approaching(&self) -> bool {
        matches!(self.light_stop_state, Some("divider_approach" | "approach"))
    }

    /// Include the delayed ACK of a completed/cancelled light brake without
    /// permitting another brake command or changing the terminal controller.
    pub fn light_brake_ack_expected(&self) -> bool {
        self.race_inputs.is_some() && matches!(self.route_stage, "light_entry" | "light_lane")
            && (self.race_light_brake.is_active() || self.race_light_brake.is_used())
    }

    /// _light_entry_coast_ack_dropped(): a centre ACK proven at neutral
    /// authorizes right in the light entry only while its light stop lasts.
    pub fn light_entry_coast_ack_dropped(&mut self) {
        if self.route_stage == "light_entry"
            && !self.light_stop_active()
            && !self.released_light_entry()
            && self._route_center_ack.is_dict()
            && !self._route_center_ack.get("transfer_coast").is_true()
            && eq_i(self._route_center_ack.get("motor"), NEUTRAL)
        {
            self._route_center_ack = V::None;
        }
    }

    /// _light_observe(control): the newest frame of control['light'], once per frame.
    pub fn light_observe(&mut self, control: &V, now: f64) {
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
        let (crosswalk, row, metres) = (light.get("crosswalk"), light.get("crosswalk_row"), light.get("crosswalk_m"));
        if self.route_stage == "start_lane" && crosswalk.is_num() && crosswalk.f() >= CROSSWALK_SEEN_SCORE {
            // round y5: placed on the start lane's next scan (crosswalk_observe);
            // round y11: with the camera model's distance
            let row = if row.is_num() { row.clone() } else { V::None };
            let metres = if metres.is_num() { metres.clone() } else { V::None };
            self._crosswalk_raw.push(V::list(vec![frame.clone(), captured.clone(), crosswalk.clone(), row, metres]));
            let n = self._crosswalk_raw.len();
            if n > 8 {
                self._crosswalk_raw.drain(..n - 8);
            }
        }
        // round y35: the frame's stripes' edge against the board ahead, for the plan at rest
        // (y36: not only with a crosswalk at the seen score)
        if self.route_stage == "start_lane"
            && self.race_inputs.is_some()
            && let (Some(edge), Some(base)) = (light.get("crosswalk_edge_y").of(), light.get("board_base_y").of())
        {
            self.race_crosswalk_stop.push_frame(captured.f(), edge, base);
        }
        self._light_frames.push(V::list(vec![frame.clone(), captured.clone(), red.clone(), green.clone()]));
        let n = self._light_frames.len();
        let keep = LIGHT_GREEN_FRAMES.max(LIGHT_RED_GONE_FRAMES) + 2;
        if n > keep {
            self._light_frames.drain(..n - keep);
        }
        let fresh_race_red = self.race_inputs.is_some() && matches!(self.route_stage, "light_entry" | "light_lane")
            && !matches!(self.light_stop_state, Some("waiting" | "released" | "passed"))
            && red.f() >= LIGHT_RED_MIN_SCORE && (0.0..=LIGHT_FRAME_MAX_AGE_S).contains(&(now - captured.f()))
            && self.light_since.is_some_and(|entry| captured.f() >= entry);
        if fresh_race_red {
            if !self._light_red_hold {
                self._light_red_at = Some(captured.f());
                if !self.light_stop_evidence.is_dict() { self.light_stop_evidence = V::dict(); }
                self.light_stop_evidence.set("red_trigger", dict! {
                    "frame" => frame.clone(), "captured_at" => captured.f(), "decision_at" => now, "red" => red.f()
                });
            }
            self._light_gantry_since.get_or_insert(captured.f());
            self._light_red_hold = true;
            self.light_stop_state = Some("approach");
            // A red frame must not interrupt an ongoing single brake with a
            // neutral pulse, which would make the next 1300 a second ESC action.
            self.lane_motor = if self.race_light_brake.is_active() { LIGHT_BRAKE_MOTOR } else { NEUTRAL };
        }
        if self.light_stop_active() && (fresh_race_red || captured.f() >= self._light_gantry_since.unwrap()) {
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

    /// Both stations come from one revolution. Only returns near a side board
    /// or a tracked leg use the shallow edge measured in the static gantry scan.
    fn race_station_posts(&self, scan: &V, heading: f64, lane: &V) -> Vec<(f64, f64)> {
        let known = self._light_line.get("legs").ol().unwrap_or(&[]);
        let band = Self::light_lane_band(lane).or_else(|| {
            let first = known.first()?.at(1).f();
            let (lo, hi) =
                known.iter().fold((first, first), |(lo, hi), leg| (lo.min(leg.at(1).f()), hi.max(leg.at(1).f())));
            Some((lo - LIGHT_LEG_LATERAL_M, hi + LIGHT_LEG_LATERAL_M))
        });
        let (Some((lo, hi)), Some(ranges)) = (band, scan.get("ranges").ol()) else { return Vec::new() };
        let outer = lane.get("outer_lateral_m").of();
        let inner = lane.get("inner_lateral_m").of();
        let (sin, cos) = heading.sin_cos();
        let mut posts = Vec::new();
        visit_posts(ranges, LIGHT_POST_STEP_M, NEAR_POST_EDGE_M, |(x, y), edge| {
            let (along, left) = (x * cos - y * sin, x * sin + y * cos);
            if !(lo..=hi).contains(&left) {
                return;
            }
            let on_side = |d: f64| (-LIGHT_GANTRY_EDGE_M..=LIGHT_LEG_INSIDE_M.1).contains(&d);
            let near_side = outer.is_some_and(|v| on_side(v - left))
                || inner.is_some_and(|v| on_side(left - v))
                || known.iter().any(|leg| (leg.at(1).f() - left).abs() <= LIGHT_LEG_LATERAL_M);
            if edge >= LIGHT_POST_JUMP_M
                || (along.abs() <= LIGHT_SENSOR_TO_GANTRY_M + FRONT_BODY_EXTENT_M && near_side)
            {
                posts.push((along, left));
            }
        });
        posts
    }

    fn race_single_leg(posts: &[(f64, f64)], heading: f64, lane: &V, reference: Option<f64>) -> Option<Gantry> {
        if heading.abs() > light_gantry_max_heading_rad() {
            return None;
        }
        let outer = lane.get("outer_lateral_m").of()?;
        let inner = lane.get("inner_lateral_m").of();
        let inside = |d: f64| (LIGHT_LEG_INSIDE_M.0..=LIGHT_LEG_INSIDE_M.1).contains(&d);
        let score = |x: f64| reference.map_or(x, |r| (x - r).abs());
        let p = posts
            .iter()
            .copied()
            .filter(|&(x, y)| {
                (FRONT_BODY_EXTENT_M..=LIGHT_GANTRY_REACH_M).contains(&x)
                    && (inside(outer - y) || inner.is_some_and(|v| inside(y - v)))
            })
            .min_by(|a, b| score(a.0).total_cmp(&score(b.0)))?;
        Some(Gantry { along_m: p.0, legs: vec![p], basis: "one_leg_inside_a_board", beam_along: None })
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
                FRONT_BODY_EXTENT_M < along && along <= LIGHT_GANTRY_REACH_M
                    && band.0 <= lateral && lateral <= band.1
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
            return Some(Gantry { along_m, legs, basis: "two_legs", beam_along: None });
        }
        if heading.abs() > light_gantry_max_heading_rad() {
            return None;
        }
        let outer = lane.idx("outer_lateral_m").f();
        let sides = (outer - LIGHT_LEG_INSIDE_M.1, outer - LIGHT_LEG_INSIDE_M.0);
        let inner = lane.idx("inner_lateral_m");
        for &(along, lateral) in &legs {
            if along <= LIGHT_GANTRY_REACH_M && ((sides.0 <= lateral && lateral <= sides.1)
                || (inner.is_some()
                    && inner.f() + LIGHT_LEG_INSIDE_M.0 <= lateral
                    && lateral <= inner.f() + LIGHT_LEG_INSIDE_M.1))
                && best.as_ref().is_none_or(|bst| along < bst.0)
            {
                best = Some((along, vec![(along, lateral)]));
            }
        }
        best.map(|(along_m, legs)| Gantry { along_m, legs, basis: "one_leg_inside_a_board", beam_along: None })
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
    pub(super) fn light_line_ahead(&mut self, scan: &V, heading: f64, lane: &V, now: f64) -> Option<f64> {
        let at = scan.idx("at_ms").i() as f64 / 1000.;
        let race = self.race_inputs.is_some();
        let divider = if race { self.race_light.project(scan, heading) } else { None };
        let board = divider.zip(self.race_light.landmark()).map(|(d,(_,offset))| d+offset)
            .or_else(|| self.lane_front_range(scan));
        let last_plane = self._light_line.get("along_m").of();
        let transported = last_plane.zip(board).zip(self._light_line.get("board_m").of())
            .map(|((line,wall),last_wall)| line+wall-last_wall);
        let spacing_known = self._light_line.get("station_spacing_m").is_num();
        let tracked = transported.or(last_plane);
        let expected = if spacing_known { tracked.or(divider) } else { divider.or(tracked) };
        let mut race_posts = race.then(|| self.race_station_posts(scan, heading, lane));
        let observed_pair = race_posts.as_ref().and_then(|posts| station_pair(posts,expected));
        let current_gate = observed_pair.filter(|p| p.beam_along.is_some()).map(|p| p.along()).or(expected);
        if let Some(posts) = &mut race_posts {
            posts.retain(|&(x,_)| !at_predicted_beam(x,current_gate));
        }
        if self._light_line.is_none() {
            let mut gantry = match &race_posts {
                Some(posts) => observed_pair.map(two_legs).or_else(|| Self::race_single_leg(posts,heading,lane,expected)),
                None => self.light_gantry(scan, heading, lane),
            };
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
                let legacy_posts;
                let posts = match &race_posts {
                    Some(p) => p.as_slice(),
                    None => { legacy_posts = lane_posts(scan.idx("ranges").l(), heading); &legacy_posts }
                };
                let followed = Self::light_leg_followed(
                    posts,
                    &legs_of(last.idx("legs")),
                    last.idx("along_m").f(),
                    at - last.idx("at_s").f(),
                );
                if let Some((along_m, legs)) = followed {
                    gantry = Some(Gantry { along_m, legs, basis: "leg_followed", beam_along: None });
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
            let mut sighting = dict! {
                "at_s" => at, "received_at" => scan.idx("received_at").clone(), "seq" => scan.idx("seq").clone(),
                "along_m" => gantry.along_m, "legs" => legs_v(&gantry.legs), "board_m" => board,
                "basis" => gantry.basis,
            };
            if let Some(beam) = gantry.beam_along {
                sighting.set("station_spacing_m", gantry.along_m - beam);
            }
            self._light_gantry_seen.push(sighting);
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
            if let Some(spacing) = tail(LIGHT_GANTRY_PAIR_CONFIRM).iter().rev().find_map(|r| r.get("station_spacing_m").of()) {
                self._light_line.set("station_spacing_m", spacing);
            }
            if race {
                let identity = if self._light_line.has("station_spacing_m") {
                    "two_stations"
                } else if gantry.legs.len() == 2 && divider.is_some_and(|d| (d-near).abs() <= LIGHT_LINE_JUMP_M) {
                    "divider_and_pair"
                } else {
                    "provisional"
                };
                self._light_line.set("station_identity", identity);
            }
            self._light_center_shift_m = Self::light_center_offset(&gantry.legs, lane, 0.);
            return Some(near);
        }
        let mut line = self._light_line.clone();
        let posts = race_posts.unwrap_or_else(|| lane_posts(scan.idx("ranges").l(), heading));
        if let Some(pair) = observed_pair {
            let gate = pair.along();
            let corrects_beam = tracked.is_some_and(|last| at_predicted_beam(last, Some(gate)))
                && divider.is_some_and(|p| (p-gate).abs() <= LIGHT_LINE_JUMP_M);
            if pair.beam_along.is_some() || corrects_beam {
                line.update(&dict! {
                    "along_m" => gate, "legs" => legs_v(&pair.legs),
                    "leg_offsets_m" => V::list(pair.legs.iter().map(|leg| V::Float(leg.0-gate)).collect()),
                    "at_s" => at, "source" => "legs", "board_m" => board, "speed_mps" => self.race_light.last_speed.map(|v| v.1)
                });
                if let Some(beam) = pair.beam_along {
                    line.set("station_spacing_m", gate-beam);
                    line.set("station_identity", "two_stations");
                } else {
                    line.set("station_identity", "divider_and_pair");
                }
                self._light_center_shift_m = Self::light_center_offset(&pair.legs,lane,self._light_center_shift_m);
                self._light_line = line;
                return Some(gate);
            }
        }
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
            if !spacing_known && transported.is_some_and(|x| at_predicted_beam(x,divider)) {
                self._light_line.set("source", "divider_after_beam");
                return divider;
            }
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
            && let Some(pair) = if race { observed_pair.map(two_legs) } else { self.light_gantry(scan, heading, lane) }
            && pair.basis == "two_legs"
            && pair.legs.contains(&legs[0])
        {
            // a line on one leg takes the pair that leg is part of now
            legs = pair.legs;
            matched = vec![0, 1];
        }
        let mut near = near;
        if legs.len() == 2 && matched.len() == 2 {
            // The race plane passes through the pair's midpoint.
            near = if race { (legs[0].0 + legs[1].0)*0.5 } else { pmin(legs[0].0, legs[1].0) };
            if line.get("station_identity").os() == Some("provisional")
                && divider.is_some_and(|d| (d-near).abs() <= LIGHT_LINE_JUMP_M) {
                line.set("station_identity", "divider_and_pair");
            }
            line.set("leg_offsets_m", V::list(legs.iter().map(|leg| V::Float(leg.0 - near)).collect()));
            self._light_center_shift_m = Self::light_center_offset(&legs, lane, self._light_center_shift_m);
        }
        let held_speed = if self.race_inputs.is_some() { self.race_light.last_speed } else { self.finish.last_speed };
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
        if !self.light_vision || matches!(self.light_stop_state, Some("waiting" | "released" | "passed")) {
            return None;
        }
        let ahead = self.light_line_ahead(scan, heading, lane, now);
        if self.race_inputs.is_some() && !self.race_light.speed_uses_gantry
            && ahead.is_some() && self._light_line.has("confirmed_seq")
            && self._light_line.get("source").os() == Some("legs") {
            // y60: the end-wall/divider reference jumped13.7cm while the real
            // legs moved smoothly, producing a false speed rise before braking.
            // Switch before the normal approach early-return, and never mix
            // two position references in one velocity history.
            self.race_light.speed_uses_gantry = true;
            self.race_light.ranges.clear();
            self.race_light.ranges.extend(self._light_gantry_seen.iter()
                .filter(|row| row.idx("seq").i() < scan.idx("seq").i())
                .map(|row| (row.idx("at_s").f(), row.idx("received_at").f(), row.idx("along_m").f())));
        }
        let prior = self.race_light.prior_ahead;
        let approach = self.light_stop_state == Some("approach");
        let red_latched = self.race_inputs.is_some() && self._light_red_hold;
        let preview = self.light_stop_state == Some("divider_approach");
        if ahead.is_none() && (preview || self.light_stop_state.is_none()) && !self.race_light.passed_hint {
            if let Some(d) = prior {
                // The estimate assumes the gantry stands at the divider's near end. The legs are
                // confirmed from 2.2 m or more when it does (simulated practice field, 200 runs); this
                // close with none in view it stands somewhere else, so stop coasting for this spot.
                if d > LIGHT_PRIOR_UNCONFIRMED_M {
                    if !preview {
                        self.light_stop_state = Some("divider_approach");
                        self._light_gantry_since = Some(now);
                        self.race_light.ranges.clear();
                        self.race_light.speed_uses_gantry = false;
                        self.race_light.last_speed = None;
                        self.race_light.beam_at = None;
                        self.route_event("light_divider_prediction", scan, control, dict! {
                            "basis" => "first_loop_board_end_relative_to_end_wall", "along_m" => d,
                            "end_wall_offset_m" => self.race_light.landmark().map(|v| v.1),
                            "observations" => self.race_light.landmark().map(|v| v.0 as i64),
                            "beam_ahead_of_front_m" => d-FRONT_BODY_EXTENT_M-LIGHT_SENSOR_TO_GANTRY_M
                        });
                    }
                    return prior;
                }
                // Passing an estimate does not prove that a moved or unlit gantry was passed.
                self.race_light.passed_hint = true;
                if preview {
                    self.light_stop_state = None;
                    self.race_light.phase = LightRoll::Cruise;
                    self.lane_motor = self.cruise_motor();
                    self.light_entry_coast_ack_dropped();
                }
            }
            return None;
        }
        let crossed_line = approach && !red_latched
            && ahead.is_some_and(|a| a - FRONT_BODY_EXTENT_M < -LIGHT_PASSED_M)
            && matches!(self._light_line.get("source").os(), Some("legs" | "board_odometry"));
        let passed = crossed_line && self._light_line.get("station_identity").os() != Some("provisional");
        // If leg tracking drops out, crossing the remembered end releases the
        // preview, but leave fresh red detection armed for a relocated gantry.
        let crossed_hint = approach && !red_latched && ahead.is_none()
            && prior.is_some_and(|d| d-FRONT_BODY_EXTENT_M < -LIGHT_PASSED_M);
        if crossed_line
            || crossed_hint
            || (ahead.is_none() && prior.is_none()
                && approach
                && !red_latched && self._light_line.get("along_m").of().is_some_and(|d| d - FRONT_BODY_EXTENT_M > LIGHT_STOP_ZONE_GAP_M))
        {
            let key = if passed { "passed" } else if crossed_line { "passed_provisional_posts" } else { "lost" };
            let ended = dict! {
                "source_seq" => scan.idx("seq").clone(), "received_at" => scan.idx("received_at").clone(),
                "route_stage" => self.route_stage, "line" => self._light_line.clone(),
            };
            let rows = appended(&self.light_stop_evidence, key, ended.clone());
            self.light_stop_evidence = self.light_stop_evidence.merged(&dict! {key => rows});
            self.light_stop_state = None;
            if crossed_hint || (crossed_line && !passed) {
                self.race_light.passed_hint = crossed_hint;
                self.race_light.beam_at = None;
                self.race_light.phase = LightRoll::Cruise;
            }
            if passed && self.race_inputs.is_some() {
                self.light_stop_state = Some("passed");
                self.route_event("light_pass_without_red", scan, control, ended);
            }
            self._light_line = V::None;
            self._light_gantry_seen.clear();
            if passed {
                self._light_rearm_min_m = Some(LIGHT_REARM_MIN_M);
            }
            self.lane_speed(scan, None, 2);
            if self.race_inputs.is_some() {
                self.race_light.ranges.clear();
                self.race_light.speed_uses_gantry = false;
                self.race_light.beam_at = None;
            }
            self.light_entry_coast_ack_dropped();
            if self.route_stage != "light_lane" {
                self.lane_motor = self.cruise_motor();
            }
            return None;
        }
        if ahead.is_none() || (approach && (!red_latched || self.light_stop_evidence.has("gantry"))) {
            return ahead.or(prior);
        }
        let evidence = dict! {
            "source_seq" => scan.idx("seq").clone(), "received_at" => scan.idx("received_at").clone(),
            "route_stage" => self.route_stage, "compass_heading_rad" => heading, "line" => self._light_line.clone(),
            "seen" => V::list(self._light_gantry_seen.clone()),
            "basis" => "gantry_legs_confirmed_on_consecutive_scans",
        };
        self.light_stop_state = Some("approach");
        if !red_latched {
            (self._light_gantry_since, self._light_red_frames, self._light_seen) = (Some(now), 0, false);
            self._light_red_at = None;
            (self._light_zone_at, self._light_red_hold) = (None, false);
        }
        let base = if self.light_stop_evidence.truthy() { self.light_stop_evidence.clone() } else { V::dict() };
        self.light_stop_evidence = base.merged(&dict! {"gantry" => evidence.clone()});
        self.route_event("light_gantry", scan, control, evidence);
        // Reference-mode behavior is unchanged. Race mode has already switched
        // to real legs above, preserving only its recent IMU velocity anchor.
        if self.race_inputs.is_none() && (!preview || prior.is_none()) {
            let seed = self._light_gantry_seen.iter()
                .map(|row| (row.idx("at_s").f(), row.idx("received_at").f(), row.idx("along_m").f()));
            self.finish.ranges = seed.collect();
            self.finish.last_speed = None;
            (self.finish.pulse, self.finish.creeping, self.finish.creeps) = (0, false, 0);
        }
        if self.route_stage != "light_lane" && !red_latched && !preview {
            self.lane_motor = self.cruise_motor();
        }
        ahead
    }

    /// Coast through the optical beam into view of the light. Only camera red can hold.
    pub fn light_approach_control(&mut self, scan: &V, ahead: Option<f64>, now: f64) -> (Option<String>, V) {
        let Some(input) = self.race_inputs else { return self.reference_light_goal(scan, ahead, now) };
        let (sin, axis_cos) = self.compass.current(scan).unwrap_or(0.).sin_cos();
        let lateral_extent = SIDE_BODY_EXTENT_M * sin.abs();
        let front_extent = lateral_extent
            + if axis_cos >= 0. { FRONT_BODY_EXTENT_M * axis_cos } else { -REAR_BODY_EXTENT_M * axis_cos };
        let rear_extent = lateral_extent
            + if axis_cos >= 0. { REAR_BODY_EXTENT_M * axis_cos } else { -FRONT_BODY_EXTENT_M * axis_cos };
        let gap = ahead.map(|a| a - front_extent);
        let to_sensor = gap.map(|g| g - LIGHT_SENSOR_TO_GANTRY_M);
        let reference = if self.race_light.speed_uses_gantry {
            (self._light_line.get("source").os() == Some("legs")).then_some(ahead).flatten()
        } else { self.race_light.prior_ahead.or(ahead).or_else(|| self.lane_front_range(scan)) };
        let (speed, current_speed) = self.race_light.measure_speed(scan, reference, input.forward_integral, axis_cos);
        if let Some(v) = current_speed.or(speed) { self.race_light.last_speed = Some((now, v)); }
        let held = self.race_light.last_speed.filter(|(at, _)| now-at <= LANE_SPEED_HOLD_S).map(|(_,v)| v);
        let v = current_speed.or(speed).or(held);
        let decel = self.lane_decels().0;
        // The range slope is along the lane. Approximate the longitudinal
        // component of drag over the coast from the present heading to straight.
        let axial_decel = decel * (1. + axis_cos.max(0.)) * 0.5;
        let brake_model = BrakeModel::light(axis_cos, axial_decel);
        // Fuse observation corrections once, then let cruise and brake use
        // the same continuous position estimate. Zone checks below keep the
        // measured front/rear gaps, never a filtered declaration of success.
        self.race_light_brake.observe(now, gap, current_speed.or(speed), input.forward_integral, axis_cos,
            axial_decel, brake_model, VISIBLE_LIGHT_STOP_GAP_M);
        let control_gap = self.race_light_brake.control_gap(now, input.forward_integral).or(gap);
        let lag = crate::motion_sensors::ESC_COAST_DELAY_S;
        let beam_speed = (2.*decel*(LIGHT_SENSOR_TO_GANTRY_M-VISIBLE_LIGHT_STOP_GAP_M)).sqrt();
        let (_, tail) = self.lane_pipeline(now, lag, lag, 0.);
        let (step, acceleration) = self.race_light.approach_step(scan, input.forward_integral, axis_cos);
        // d(v_body*cos(heading))/dt includes the change of projection as the
        // car straightens, in addition to its measured forward acceleration.
        let heading_accel = current_speed.zip(input.yaw_rate)
            .filter(|_| axis_cos >= std::f64::consts::FRAC_1_SQRT_2)
            .map_or(0., |(axial_speed, yaw)| -axial_speed / axis_cos * sin * yaw);
        let acceleration = acceleration.map(|a| a + heading_accel);
        // 1570 has its own measured response in SurfaceResponse. Forecast its
        // base propulsion even when load compensation is zero; the old 0.35
        // constant described 1550. Project body acceleration into the lane.
        let body_speed = v.filter(|_| axis_cos > 0.).map(|v| v / axis_cos);
        let drive_acceleration = |pwm| self.race_surface.predicted_drive_accel(pwm, body_speed, now)
            .map(|a| a * axis_cos.max(0.) + heading_accel);
        let cruise_accel = drive_acceleration(RACE_CRUISE_MOTOR).or(acceleration).unwrap_or(0.);
        // 1530 sustains motion but has no fitted force curve. Its current IMU
        // slope, not either the 1550 model or coast drag, describes this tail.
        let pending_accel = drive_acceleration(self._power_cmd_motor.unwrap_or(RACE_CRUISE_MOTOR))
            .or(acceleration).unwrap_or(0.);
        let coast_travel = |v: f64, powered_s: f64| {
            let release = (v + pending_accel * powered_s).max(0.);
            (v + release) * powered_s / 2. + release * release / (2. * axial_decel)
        };
        let predicted = control_gap.zip(v).map(|(g,v)| g - coast_travel(v, tail));
        // The light now has its own active brake. Keep one continuous hold
        // until the neutral tail plus that brake fits the visible stopping
        // point, instead of aiming a long coast and then refilling it twice.
        let if_renewed = control_gap.zip(v).map(|(g,v)| {
            let next = (v + acceleration.unwrap_or(cruise_accel) * step).max(0.);
            let release = (next + cruise_accel * lag).max(0.);
            g-(v+next)*step/2.-(next+release)*lag/2.
                -release*LIGHT_BRAKE_RESPONSE_S-brake_model.distance(release)
        });
        let rear_gap = ahead.map(|a| a + rear_extent);
        let red = self._light_red_hold;
        // A leg sighting can move the estimate back upstream; small noise at the beam cannot.
        match to_sensor {
            Some(s) if s <= 0. => {
                self.race_light.beam_at.get_or_insert(now);
            }
            Some(s) if s > LIGHT_LINE_JUMP_M => self.race_light.beam_at = None,
            _ => {}
        }
        let crossed = self.race_light.beam_at.is_some();
        let red_due = self.race_light.beam_at.is_some_and(|at| now - at >= LIGHT_RED_DUE_S);
        if !red {
            let cut = if_renewed.is_some_and(|g| g <= VISIBLE_LIGHT_STOP_GAP_M);
            // A powered 1530 hold continually renews the old power-tail clock.
            // It must still use the existing restart path if motion runs out.
            let running_out = (tail == 0. || matches!(self.race_light.phase, LightRoll::Coast | LightRoll::ZoneCoast))
                && current_speed.or(speed).is_some_and(|s| s < LIGHT_RESTART_MPS);
            let revoke = !crossed && !running_out
                && if_renewed.is_some_and(|g| g > VISIBLE_LIGHT_STOP_GAP_M + LIGHT_COAST_REVOKE_BAND_M);
            self.race_light.phase = match self.race_light.phase {
                LightRoll::Cruise if cut => LightRoll::Coast,
                LightRoll::Coast if revoke => LightRoll::Cruise,
                // No red: do not park to wait for an unlit gantry. Short of the
                // beam push through to trigger it; past it, wait for the red
                // the beam has started, and keep going only once that is overdue.
                LightRoll::Coast | LightRoll::ZoneCoast
                    if running_out && (!crossed || red_due) => LightRoll::Through,
                LightRoll::Through if crossed && !red_due && rear_gap.is_some_and(|g| g <= LIGHT_ZONE_M) => LightRoll::ZoneCoast,
                phase => phase,
            };
        }
        let motor = if red || self.race_light_brake.is_used() {
            NEUTRAL
        } else if matches!(self.race_light.phase, LightRoll::Coast | LightRoll::ZoneCoast) {
            // y77 spent a second in neutral from 2.14 m to the brake point.
            // The operator measured 1530 as a moving hold; braking still uses
            // current measured speed, never the unpowered coast deceleration.
            ROLLING_HOLD_MOTOR
        } else if matches!(self.race_light.phase, LightRoll::Through | LightRoll::Cruise)
            && current_speed.or(speed).is_some_and(|s| s < LIGHT_RESTART_MPS)
        {
            // y34: 1550 does not start the car from rest (y33 sim short5 sat at 1550 for 26 s).
            // The rule-range simulation found the same stall in Cruise after a coast was revoked.
            TRIAL_MOTOR
        } else { RACE_CRUISE_MOTOR };
        let brake_stop = control_gap.zip(v).map(|(g, v)| g-brake_model.stop_travel(v, acceleration.unwrap_or(0.)));
        let next_brake_stop = control_gap.zip(v).map(|(g, v)| {
            let next = (v + acceleration.unwrap_or(0.) * step).max(0.);
            g-(v+next)*step/2.-brake_model.stop_travel(next, acceleration.unwrap_or(0.))
        });
        let motor = self.race_light_brake.command(now, input.forward_integral)
            .unwrap_or(motor);
        self.lane_motor = motor;
        let rear_past_sensor = rear_gap.map(|g| LIGHT_ZONE_M-g);
        let whole_in_zone = gap.is_some_and(|g| g >= 0.) && rear_past_sensor.is_some_and(|d| d > 0.);
        let stopped = red && current_speed.or(speed).is_some_and(|s| s < LIGHT_RESTART_MPS)
            && self._power_cmd_at.is_none_or(|at| now-at >= lag);
        let basis = stopped.then(|| if whole_in_zone { "stopped_after_camera_red" }
            else { "stopped_after_camera_red_outside_zone" }.to_string());
        let feedback = dict! {"goal" => "light", "front_range_m" => ahead, "front_gap_m" => gap,
            "control_front_gap_m" => control_gap, "speed_fusion" => self.race_light.speed_fusion_diagnostics(),
            "speed_reference_source" => if self.race_light.speed_uses_gantry { "confirmed_gantry_legs" } else { "pre_gantry_preview" },
            "speed_reference_m" => reference,
            "sensor_ahead_of_front_m" => to_sensor, "sensor_to_gantry_m" => LIGHT_SENSOR_TO_GANTRY_M,
            "speed_mps" => speed, "speed_now_mps" => current_speed, "beam_coast_speed_mps" => beam_speed,
            "stop_target_gap_m" => VISIBLE_LIGHT_STOP_GAP_M, "predicted_stop_gap_m" => predicted, "coast_decel_mps2" => decel,
            "rolling_hold_pwm" => ROLLING_HOLD_MOTOR,
            "cruise_pwm" => RACE_CRUISE_MOTOR, "cruise_accel_mps2" => cruise_accel,
            "pending_power_accel_mps2" => pending_accel,
            "stop_gap_if_renewed_m" => if_renewed, "renewal_stop_basis" => "neutral_tail_then_light_brake",
            "rear_to_gantry_m" => rear_gap, "rear_past_sensor_m" => rear_past_sensor,
            "heading_accel_mps2" => heading_accel, "brake_stop_gap_m" => brake_stop,
            "next_brake_stop_gap_m" => next_brake_stop, "visibility_brake" => self.race_light_brake.phase(),
            "brake_timing_basis" => "current_control_tick_prediction",
            "visibility_brake_used" => self.race_light_brake.is_used(),
            "divider_gantry_ahead_m" => self.race_light.prior_ahead, "approach_phase" => self.race_light.phase.label(),
            "beam_crossed_s_ago" => self.race_light.beam_at.map(|at| now - at), "red_due" => red_due,
            "red_hold" => red, "motor" => motor, "creeps" => 0,
            "whole_car_in_finish_zone" => whole_in_zone};
        (basis, feedback)
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
        self.prepare_finish_wait(scan);
        self.light_stop_evidence = self.light_stop_evidence.merged(&dict! {"stop" => stop.clone()});
        self.route_event("light_stop", scan, control, stop);
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
        let green_min_score = if self.race_inputs.is_some() && self._light_red_frames > 0 {
            RACE_GREEN_AFTER_RED_MIN_SCORE
        } else { LIGHT_GREEN_MIN_SCORE };
        let green = frames.len() == LIGHT_GREEN_FRAMES
            && frames.iter().all(|f| {
                f.at(1).f() >= wait_since
                    && now - f.at(1).f() <= LIGHT_FRAME_MAX_AGE_S
                    && f.at(3).f() >= green_min_score
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
        let race_fallback = red_s.is_some_and(|s| s >= LIGHT_RED_GO_S) && !red_now;
        let wait = if self.race_inputs.is_some() { !green && !race_fallback }
            else { !green && !red_ended && !red_timed && !no_light && waited < LIGHT_WAIT_MAX_S };
        if wait {
            // the board ahead's history for the restart
            let ahead = self.lane_front_range(scan);
            self.lane_speed(scan, ahead, 2);
            return false;
        }
        self.prepare_finish_release(scan, now);
        let basis = if green && self._light_red_frames != 0 {
            "green_in_fresh_frames_after_the_stop"
        } else if green {
            "green_without_red_since_the_gantry_looking_again"
        } else if self.race_inputs.is_some() && race_fallback {
            "red_seen_10_5s_no_fresh_red"
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
            "launch" => self.finish.launch_until.is_some(),
        };
        let mut go = go;
        if self.race_inputs.is_some() { go.set("green_min_score", green_min_score); }
        self.light_stop_evidence = self.light_stop_evidence.merged(&dict! {"go" => go.clone()});
        self.route_event("light_go", scan, control, go.clone());
        if (no_light && !green) || (green && self._light_red_frames == 0) {
            // not the light: a later gantry may start a new stop
            let rearmed = appended(&self.light_stop_evidence, "rearmed", go);
            self.light_stop_evidence = self.light_stop_evidence.merged(&dict! {"rearmed" => rearmed});
            self.light_stop_state = None;
            self._light_line = V::None;
            self._light_gantry_seen.clear();
            self._light_rearm_min_m = Some(LIGHT_REARM_MIN_M);
            if self.race_inputs.is_some() {
                self.race_light.ranges.clear();
                self.race_light.speed_uses_gantry = false;
            }
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

    #[test]
    fn a_paused_light_brake_accepts_its_ack_without_reissuing_the_cached_brake() {
        let mut m = Maneuver::new(&0.0.into(), &40.0.into(), &1670.into(), &true.into());
        m.race_inputs = Some(Default::default());
        m.route_stage = "light_entry";
        m.light_stop_state = Some("approach");
        m.servo = NEUTRAL;
        m.lane_motor = LIGHT_BRAKE_MOTOR;
        m.race_light_brake.observe(1., Some(1.1), Some(1.), Some(0.), 1., 0.6, BrakeModel::light(1., 0.6), 0.55);
        assert_eq!(m.race_light_brake.command(1., Some(0.)), Some(1300));
        let ack = dict! {"motor" => 1300, "servo" => 1500, "seq" => 10, "tick" => 100,
            "armed" => true, "command_acked" => true};
        assert!(m.route_adopted(&ack, 1.1));
        assert_eq!(m._route_center_ack.get("motor").i(), 1300);
        m.phase = "quality_wait";
        m.quality_wait_servo = Some(NEUTRAL);
        assert_eq!(m.result(1.2).get("motor").i(), 1500);
        assert_eq!(m.lane_motor, NEUTRAL);
        assert!(m.race_light_brake.is_used());
        m.phase = "drive";
        assert_eq!(m.route_motor(), NEUTRAL);
        assert_eq!(m.race_light_brake.command(1.3, Some(0.)), None);
    }

    #[test]
    fn two_stations_override_a_wrong_prior_and_a_lone_beam_does_not_become_the_gantry() {
        let beam = [(1.0, 0.51), (1.02, -0.46)];
        let gate = [(2.02, 0.47), (2.00, -0.48)];
        let posts = [beam[1], gate[1], beam[0], gate[0], (4.3, 0.46), (4.32, -0.48)];
        let paired = station_pair(&posts, Some(1.0)).unwrap();
        assert_eq!(paired.legs, gate);
        assert!((paired.beam_along.unwrap() - 1.01).abs() < 1e-9);
        assert!(station_pair(&beam, Some(2.01)).is_none());
        let single = station_pair(&gate, Some(2.01)).unwrap();
        assert_eq!(single.legs, gate);
        assert!(single.beam_along.is_none());
    }

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
