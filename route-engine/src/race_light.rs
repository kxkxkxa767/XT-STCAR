//! The open end of the light lane's inner board, observed while passing cone one.
//! Store its distance from the end wall, not its bearing in a turning car.
use crate::helpers::{FRONT_BODY_EXTENT_M, OPEN_LANE_MIN_SPAN_M, OPEN_LINE_MIN_POINTS, SIDE_BODY_EXTENT_M, dcos, dsin};
use crate::maneuver::{
    LIGHT_GANTRY_EDGE_M, LIGHT_GANTRY_SPAN_M, LIGHT_GANTRY_ALONG_DIFF_M, LIGHT_LEG_LATERAL_M,
    LIGHT_LINE_JUMP_M, LIGHT_POST_JUMP_M, LIGHT_SENSOR_TO_GANTRY_M, OPEN_LANE_WIDTH_M,
};
use crate::py::V;
use crate::motion_sensors::{RangeHistory, lane_range_speed};
use crate::maneuver::{LANE_SPEED_BASELINE, LANE_SPEED_HISTORY};

type Post = (f64, f64); // longitudinal and leftward distances in the lane frame

#[derive(Clone, Copy)]
pub(super) struct StationPair {
    pub legs: [Post; 2], // left, right
    pub beam_along: Option<f64>,
}

impl StationPair {
    pub fn along(self) -> f64 {
        (self.legs[0].0 + self.legs[1].0) * 0.5
    }
}

fn across_lane(a: Post, b: Post) -> bool {
    (LIGHT_GANTRY_SPAN_M.0..=LIGHT_GANTRY_SPAN_M.1).contains(&(a.1 - b.1).abs())
        && (a.0 - b.0).abs() <= LIGHT_GANTRY_ALONG_DIFF_M
}

pub(super) fn at_predicted_beam(along: f64, gantry: Option<f64>) -> bool {
    gantry.is_some_and(|g| (g - along - LIGHT_SENSOR_TO_GANTRY_M).abs() <= LIGHT_LINE_JUMP_M)
}

/// A matching post one beam spacing upstream on each side identifies the
/// downstream pair, even if another station was tracked first. With just one
/// pair, use the first-cone divider or the tracked line to resolve its role.
pub(super) fn station_pair(posts: &[Post], predicted: Option<f64>) -> Option<StationPair> {
    if posts.len() < 2 {
        return None;
    }
    let upstream: Vec<Option<Post>> = posts
        .iter()
        .map(|&(x, y)| {
            posts
                .iter()
                .copied()
                .filter(|&(u, v)| (y - v).abs() <= LIGHT_LEG_LATERAL_M && at_predicted_beam(u, Some(x)))
                .min_by(|a, b| {
                    (x - a.0 - LIGHT_SENSOR_TO_GANTRY_M)
                        .abs()
                        .total_cmp(&(x - b.0 - LIGHT_SENSOR_TO_GANTRY_M).abs())
                })
        })
        .collect();
    let mut best: Option<(StationPair, f64)> = None;
    for (i, &a) in posts.iter().enumerate() {
        for (j, &b) in posts[..i].iter().enumerate() {
            if a.0 <= FRONT_BODY_EXTENT_M || b.0 <= FRONT_BODY_EXTENT_M || !across_lane(a, b) {
                continue;
            }
            let beam_along =
                upstream[i].zip(upstream[j]).filter(|&(l, r)| across_lane(l, r)).map(|(l, r)| (l.0 + r.0) * 0.5);
            let pair = StationPair { legs: if a.1 > b.1 { [a, b] } else { [b, a] }, beam_along };
            let along = pair.along();
            if beam_along.is_none() && at_predicted_beam(along, predicted) {
                continue;
            }
            let error = predicted.map_or(along, |g| (along - g).abs()) + (a.0 - b.0).abs();
            if best.as_ref().is_none_or(|(old, score)| match (beam_along.is_some(), old.beam_along.is_some()) {
                (true, false) => true,
                (false, true) => false,
                _ => error < *score,
            }) {
                best = Some((pair, error));
            }
        }
    }
    best.map(|(pair, _)| pair)
}

struct ScanAxis<'a> {
    ranges: &'a [V],
    sin: f64,
    cos: f64,
    forward: usize,
}

impl<'a> ScanAxis<'a> {
    fn new(scan: &'a V, heading: f64) -> Option<Self> {
        let r = scan.get("ranges");
        if !r.is_list() || r.len() != 360 {
            return None;
        }
        let (sin, cos) = heading.sin_cos();
        Some(Self {
            ranges: r.l(),
            sin,
            cos,
            forward: (heading.to_degrees().round() as i64).rem_euclid(360) as usize,
        })
    }
    // Coordinates in the finish lane's direction of travel, relative to the lidar.
    fn point(&self, i: usize) -> Option<(f64, f64)> {
        let r = self.ranges[i].of()?;
        let (c, s) = (dcos(i), dsin(i));
        Some((
            r * (self.cos * c + self.sin * s),
            r * (self.sin * c - self.cos * s),
        ))
    }
    fn end_wall(&self) -> Option<f64> {
        // The centre ray can hit a cone or a light post. Seek a common farther
        // plane among 13 projected rays; no whole-scan fit or pose integration.
        let mut x = [0.; 13];
        let mut n = 0;
        for j in 0..13 {
            if let Some((d, _)) = self.point((self.forward + 348 + j * 2) % 360)
                && d > 0.
            {
                x[n] = d;
                n += 1;
            }
        }
        x[..n].sort_unstable_by(f64::total_cmp);
        x[..n]
            .windows(3)
            .rev()
            .find(|w| w[2] - w[0] <= LIGHT_GANTRY_EDGE_M)
            .map(|w| w[1])
    }
    fn divider_span(&self, wall: f64) -> Option<f64> {
        let mut found: Option<f64> = None;
        for i in 0..360 {
            let Some(r) = self.ranges[i].of() else {
                continue;
            };
            // Beside the upstream end the adjacent rays go *farther* into the
            // light lane; a closer cone hiding the rest of a board is not an end.
            if self.ranges[(i + 359) % 360]
                .of()
                .is_none_or(|next| next <= r + LIGHT_POST_JUMP_M)
            {
                continue;
            }
            let (x, y) = self.point(i)?;
            if y <= SIDE_BODY_EXTENT_M || wall - x <= OPEN_LANE_MIN_SPAN_M {
                continue;
            }
            let outside = |k| self.point((i + 360 - k) % 360).map(|(_, v)| v - y);
            if !(1..=2).all(|k| outside(k).is_some_and(|d| d > LIGHT_POST_JUMP_M))
                || !(1..=3).any(|k| {
                    outside(k).is_some_and(|d| {
                        (OPEN_LANE_WIDTH_M.0 - LIGHT_GANTRY_EDGE_M..=OPEN_LANE_WIDTH_M.1)
                            .contains(&d)
                    })
                })
            {
                continue;
            }
            // Follow the near face downstream, not merely the last bin of a
            // native fitted line (the fit can end in the middle of a board).
            for j in 1..180 {
                let Some((a, b)) = self.point((i + j) % 360) else {
                    break;
                };
                if (b - y).abs() > LIGHT_GANTRY_EDGE_M || a < x - LIGHT_GANTRY_EDGE_M {
                    break;
                }
                if j + 1 >= OPEN_LINE_MIN_POINTS && a - x >= OPEN_LANE_MIN_SPAN_M {
                    let span = wall - x;
                    if found.is_some_and(|other| (other - span).abs() > LIGHT_LINE_JUMP_M) {
                        return None;
                    }
                    found = Some(span);
                    break;
                }
            }
        }
        found
    }
}

#[derive(Clone, Copy, Default, Debug)]
pub enum LightRoll {
    #[default]
    Cruise,
    Coast,
    Topup,
    FinalCoast,
    EnterZone,
    ZoneCoast,
    Through,
}

impl LightRoll {
    pub fn label(self) -> &'static str {
        match self {
            Self::Cruise => "cruise",
            Self::Coast => "coast",
            Self::Topup => "rolling_topup",
            Self::FinalCoast => "final_coast",
            Self::EnterZone => "rolling_rear_into_zone",
            Self::ZoneCoast => "zone_coast",
            Self::Through => "unlit_continue",
        }
    }
}

#[derive(Clone, Default, Debug)]
pub struct RaceLight {
    votes: [(u32, f64); 2],
    last_seq: Option<i64>,
    pub phase: LightRoll,
    pub prior_ahead: Option<f64>,
    pub passed_hint: bool,
    pub ranges: RangeHistory,
    pub last_speed: Option<(f64, f64)>,
    imu_history: Vec<(f64, f64, f64)>, // scan time, body IMU integral, lane projection cos
    body_speed_anchor: Option<(f64, f64, f64)>, // scan time, body speed, IMU integral
    /// When the front was first estimated past the photo beam (y34).
    pub beam_at: Option<f64>,
    approach_sample: Option<(i64, f64, Option<f64>)>,
}
impl RaceLight {
    /// Divide by sequence distance if a scan was skipped.
    pub fn approach_step(&mut self, scan: &V, integral: Option<f64>, axis_cos: f64) -> (f64, Option<f64>) {
        let seq = scan.idx("seq").i();
        let at = scan.idx("at_ms").f() / 1000.;
        if self.approach_sample.is_some_and(|(last_seq, last_at, _)| seq == last_seq && at == last_at) {
            return (0., None);
        }
        let prev = self.approach_sample.replace((seq, at, integral));
        let Some((prev_seq, prev_at, prev_integral)) = prev else { return (0., None) };
        let dt = at - prev_at;
        if seq <= prev_seq || dt <= 0. || dt > crate::helpers::SCAN_AGE_S {
            return (0., None);
        }
        (dt / (seq - prev_seq) as f64, integral.zip(prev_integral).map(|(v, p)| (v - p) * axis_cos / dt))
    }

    pub fn measure_speed(&mut self, scan: &V, reference: Option<f64>, forward_integral: Option<f64>, axis_cos: f64)
        -> (Option<f64>, Option<f64>) {
        let raw = lane_range_speed(&mut self.ranges, scan, reference, 2);
        let at = scan.idx("at_ms").i() as f64 * 0.001;
        (raw, self.adjusted_speed(forward_integral, at, raw, axis_cos))
    }

    /// Range differences measure lane-axis motion, whose projection changes
    /// while the car leaves the second loop. Correct each existing difference
    /// interval before taking its median, then project the current body speed.
    fn adjusted_speed(&mut self, forward_integral: Option<f64>, at: f64, raw: Option<f64>, axis_cos: f64) -> Option<f64> {
        let Some(current) = forward_integral.filter(|v| v.is_finite()) else {
            self.imu_history.clear();
            self.body_speed_anchor = None;
            return None;
        };
        if !at.is_finite() || !axis_cos.is_finite() { return None; }
        let h = &mut self.imu_history;
        if h.last().is_none_or(|&(t, _, _)| t != at) {
            if h.last().is_some_and(|&(t, _, _)| at < t || at - t > crate::helpers::SCAN_AGE_S) {
                h.clear();
                self.body_speed_anchor = None;
            }
            h.push((at, current, axis_cos));
            if h.len() > LANE_SPEED_HISTORY { h.remove(0); }
        }
        if raw.is_some() {
            let n = self.ranges.len();
            let step = LANE_SPEED_BASELINE.min(n.saturating_sub(2));
            let mut bodies = [0.; LANE_SPEED_HISTORY];
            let mut count = 0;
            if step > 0 {
                for (a, b) in self.ranges.iter().zip(self.ranges[step..].iter()) {
                    let (mut projected_time, mut projected_integral, mut covered) = (0., 0., 0.);
                    for p in h.windows(2) {
                        let start = a.0.max(p[0].0);
                        let end = b.0.min(p[1].0);
                        if start >= end { continue; }
                        let interpolate = |t: f64| {
                            let f = (t-p[0].0)/(p[1].0-p[0].0);
                            (p[0].1+f*(p[1].1-p[0].1), p[0].2+f*(p[1].2-p[0].2))
                        };
                        let ((i0, c0), (i1, c1)) = (interpolate(start), interpolate(end));
                        let dt = end-start;
                        projected_time += dt*(c0+c1)*0.5;
                        // Exact product integral for linearly interpolated I and cos.
                        projected_integral += dt*(2.*i0*c0+i0*c1+i1*c0+2.*i1*c1)/6.;
                        covered += dt;
                    }
                    let duration = b.0-a.0;
                    // Reuse the surface observer's geometric conditioning:
                    // an interval averaging >=cos(45deg) amplifies range noise
                    // by <=sqrt(2). No division by an instantaneous near-zero cos.
                    if (covered-duration).abs() > 1e-8 || duration <= 0.
                        || projected_time/duration < std::f64::consts::FRAC_1_SQRT_2
                    { continue; }
                    // distance = integral[(v_now + I(t)-I_now) * cos(t)] dt.
                    let body = (a.2-b.2-projected_integral)/projected_time + current;
                    if body.is_finite() { bodies[count] = body.max(0.); count += 1; }
                }
            }
            if count > 0 {
                bodies[..count].sort_unstable_by(f64::total_cmp);
                self.body_speed_anchor = Some((at, bodies[count/2], current));
            }
        }
        let (anchor_at, body, origin) = self.body_speed_anchor?;
        if !(0. ..=crate::helpers::SCAN_AGE_S).contains(&(at-anchor_at)) { return None; }
        Some(((body+current-origin).max(0.)*axis_cos).max(0.))
    }

    pub fn imu_history_dump(&self) -> V {
        V::list(self.imu_history.iter().map(|r| V::from(vec![r.0,r.1,r.2])).collect())
    }

    pub fn observe_first(&mut self, scan: &V, heading: f64) {
        let seq = scan.idx("seq").i();
        if self.last_seq == Some(seq) {
            return;
        }
        self.last_seq = Some(seq);
        let Some(axis) = ScanAxis::new(scan, heading) else {
            return;
        };
        let Some(span) = axis.end_wall().and_then(|w| axis.divider_span(w)) else {
            return;
        };
        // Keep the consistent landmark across viewpoints, not an isolated gap.
        if let Some((n, mean)) = self
            .votes
            .iter_mut()
            .filter(|(n, m)| *n > 0 && (*m - span).abs() <= LIGHT_LINE_JUMP_M)
            .min_by(|a, b| (a.1 - span).abs().total_cmp(&(b.1 - span).abs()))
        {
            *n += 1;
            *mean += (span - *mean) / f64::from(*n);
        } else if let Some(slot) = self.votes.iter_mut().min_by_key(|v| v.0) {
            *slot = (1, span);
        }
    }
    pub fn landmark(&self) -> Option<(u32, f64)> {
        let [a, b] = self.votes;
        let best = if a.0 > b.0 { a } else { b };
        (best.0 >= 2 && a.0 != b.0).then_some(best)
    }
    pub fn project(&mut self, scan: &V, heading: f64) -> Option<f64> {
        self.prior_ahead = self
            .landmark()
            .and_then(|(_, span)| Some(ScanAxis::new(scan, heading)?.end_wall()? - span));
        self.prior_ahead
    }
}
