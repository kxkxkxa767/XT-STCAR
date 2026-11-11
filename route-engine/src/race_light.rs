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
use crate::race_speed_kalman::SpeedKalman;

#[derive(Clone, Debug)]
struct ConeMotionSample {
    seq: i64,
    observed_at: f64,
    at: f64,
    identity: V,
    center: (f64, f64),
    yaw: f64,
    integral: f64,
}

/// Motion at cone two for the entry response, independent of route decisions.
#[derive(Clone, Debug, Default)]
pub struct EntryMotionSpeed {
    previous: Option<ConeMotionSample>,
    anchors: [Option<(f64, f64, f64)>; 3], // control time, body speed, IMU integral
    next: usize,
}

impl EntryMotionSpeed {
    pub fn observe(&mut self, scan: &V, now: f64, identity: &V, center: (f64, f64), input: crate::motion_sensors::RaceInputs) {
        let seq = scan.get("seq").i();
        let observed_at = scan.get("received_at").f();
        if self.previous.as_ref().is_some_and(|old| old.seq == seq && crate::py::py_eq(&old.identity, identity)) { return; }
        let (Some(yaw), Some(integral)) = (input.yaw_rate, input.forward_integral) else {
            *self = Self::default();
            return;
        };
        if ![now, observed_at, center.0, center.1, yaw, integral].iter().all(|v| v.is_finite()) {
            *self = Self::default();
            return;
        }
        let sample = ConeMotionSample { seq, observed_at, at: now, identity: identity.clone(), center, yaw, integral };
        let Some(old) = self.previous.replace(sample) else { return };
        let dt = observed_at - old.observed_at;
        let imu_dt = now - old.at;
        if seq <= old.seq || !(0. < dt && dt <= crate::helpers::SCAN_AGE_S)
            || !(0. < imu_dt && imu_dt <= crate::helpers::SCAN_AGE_S)
            || !crate::py::py_eq(&old.identity, identity) {
            self.anchors = [None; 3];
            self.next = 0;
            return;
        }
        // Same stationary support: rotate the new centre into the old car
        // frame. The translation's midpoint-forward component is body motion;
        // the sensor's lateral motion from yaw does not enter that component.
        let midpoint = (old.observed_at + observed_at) * 0.5;
        // Align the yaw trapezoid to the actual scan interval; the current
        // control sample can be a few milliseconds later than scan receipt.
        let mean_yaw = old.yaw + (yaw - old.yaw) * (midpoint - old.at) / imu_dt;
        let angle = mean_yaw * dt;
        let (sin, cos) = angle.sin_cos();
        let displacement = (old.center.0 - (cos * center.0 - sin * center.1),
                            old.center.1 - (sin * center.0 + cos * center.1));
        let half = angle * 0.5;
        let (sh, ch) = half.sin_cos();
        let sinc = if half.abs() < 1e-6 { 1. } else { sh / half };
        let mean = (displacement.0 * ch + displacement.1 * sh) / (dt * sinc);
        // Geometry gives interval-mean speed, not current speed. Transport it
        // from the interval midpoint to this control/IMU sample before use.
        let speed = mean + (integral - old.integral) / imu_dt * (now - midpoint);
        if !speed.is_finite() || speed < 0. { return; }
        self.anchors[self.next] = Some((now, speed, integral));
        self.next = (self.next + 1) % self.anchors.len();
    }

    pub fn speed(&self, now: f64, integral: Option<f64>) -> Option<f64> {
        let current = integral.filter(|v| v.is_finite())?;
        let previous = self.previous.as_ref()?;
        // Reuse the existing short IMU bridge. Reading a prediction never
        // renews the true target-observation timestamp.
        if !(0. ..=crate::race_speed_kalman::BRIDGE_S).contains(&(now - previous.observed_at)) { return None; }
        let mut values = [0.; 3];
        let mut n = 0;
        for &(at, speed, origin) in self.anchors.iter().flatten() {
            if (0. ..=crate::race_speed_kalman::BRIDGE_S).contains(&(now - at)) {
                values[n] = (speed + current - origin).max(0.);
                n += 1;
            }
        }
        if n == 0 { return None; }
        values[..n].sort_unstable_by(f64::total_cmp);
        Some(values[n / 2])
    }

    pub fn diagnostics(&self, now: f64, integral: Option<f64>) -> V {
        crate::dict! {"source_seq" => self.previous.as_ref().map(|v| v.seq),
            "observed_at" => self.previous.as_ref().map(|v| v.observed_at),
            "track_id" => self.previous.as_ref().map(|v| v.identity.clone()),
            "body_speed_mps" => self.speed(now, integral),
            "basis" => "fresh_cone_center_displacement_imu_yaw_midpoint_to_current"}
    }
}

#[cfg(test)]
mod entry_motion_tests {
    use super::*;
    use crate::motion_sensors::RaceInputs;

    fn scan(seq: i64, at: f64) -> V { crate::dict! {"seq" => seq, "received_at" => at} }

    #[test]
    fn entry_speed_aligns_interval_midpoint_to_the_later_control_sample() {
        let mut speed = EntryMotionSpeed::default();
        // v(t)=1+2t. Scans at 0/.1, control/IMU samples at .02/.12.
        speed.observe(&scan(1, 0.), 0.02, &1.into(), (3., 0.),
            RaceInputs { yaw_rate: Some(0.), forward_integral: Some(0.04) });
        speed.observe(&scan(2, 0.1), 0.12, &1.into(), (2.89, 0.),
            RaceInputs { yaw_rate: Some(0.), forward_integral: Some(0.24) });
        assert!((speed.speed(0.12, Some(0.24)).unwrap() - 1.24).abs() < 1e-10);
        assert!((speed.speed(0.17, Some(0.34)).unwrap() - 1.34).abs() < 1e-10);
    }

    #[test]
    fn entry_speed_removes_yaw_and_does_not_renew_old_or_rebound_targets() {
        let mut speed = EntryMotionSpeed::default();
        let (v, yaw, dt): (f64, f64, f64) = (2.4, -1.8, 0.1);
        let angle = yaw * dt;
        let shift = (v / yaw * angle.sin(), v / yaw * (1. - angle.cos()));
        let p = (2. - shift.0, -1. - shift.1);
        let next = (angle.cos() * p.0 + angle.sin() * p.1, -angle.sin() * p.0 + angle.cos() * p.1);
        let input = RaceInputs { yaw_rate: Some(yaw), forward_integral: Some(0.) };
        speed.observe(&scan(1, 0.), 0., &1.into(), (2., -1.), input);
        speed.observe(&scan(2, dt), dt, &1.into(), next, input);
        assert!((speed.speed(dt, Some(0.)).unwrap() - v).abs() < 1e-10);
        speed.observe(&scan(2, dt), 0.8, &1.into(), next, input);
        assert_eq!(speed.speed(1.11, Some(0.)), None);
        speed.observe(&scan(3, 0.2), 0.2, &2.into(), (8., 8.), input);
        assert_eq!(speed.speed(0.2, Some(0.)), None);
    }
}

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
    ZoneCoast,
    Through,
}

impl LightRoll {
    pub fn label(self) -> &'static str {
        match self {
            Self::Cruise => "cruise",
            Self::Coast => "rolling_low",
            Self::ZoneCoast => "zone_rolling_low",
            Self::Through => "unlit_continue",
        }
    }
}

#[derive(Clone, Default, Debug)]
pub struct RaceLight {
    pub entry_motion: EntryMotionSpeed,
    votes: [(u32, f64); 2],
    last_seq: Option<i64>,
    pub phase: LightRoll,
    pub prior_ahead: Option<f64>,
    pub passed_hint: bool,
    pub ranges: RangeHistory,
    pub speed_uses_gantry: bool,
    pub last_speed: Option<(f64, f64)>,
    imu_history: Vec<(f64, f64, f64)>, // scan time, body IMU integral, lane projection cos
    speed_kalman: SpeedKalman,
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
        // Missing legs are not a new position sample. Keep the real samples
        // across a short gap; the existing dt checks reset stale histories
        // when a real leg returns. The IMU anchor still has its own short age.
        let raw = if self.speed_uses_gantry && reference.is_none() { None }
            else { lane_range_speed(&mut self.ranges, scan, reference, 2) };
        let at = scan.idx("at_ms").i() as f64 * 0.001;
        (raw, self.adjusted_speed(forward_integral, at, raw, axis_cos))
    }

    /// Range differences measure lane-axis motion, whose projection changes
    /// while the car leaves the second loop. Correct each existing difference
    /// interval before taking its median, then project the current body speed.
    fn adjusted_speed(&mut self, forward_integral: Option<f64>, at: f64, raw: Option<f64>, axis_cos: f64) -> Option<f64> {
        let Some(current) = forward_integral.filter(|v| v.is_finite()) else {
            self.imu_history.clear();
            self.speed_kalman.clear();
            return None;
        };
        if !at.is_finite() || !axis_cos.is_finite() { return None; }
        let h = &mut self.imu_history;
        if h.last().is_none_or(|&(t, _, _)| t != at) {
            if h.last().is_some_and(|&(t, _, _)| at < t || at - t > crate::helpers::SCAN_AGE_S) {
                h.clear();
                self.speed_kalman.clear();
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
                // Keep physical acceleration on the IMU path; only the radar correction is
                // filtered. Missing radar never reaches the filter (race_speed_kalman).
                self.speed_kalman.update(at, current, bodies[count/2]);
            }
        }
        let (anchor_at, body, origin) = self.speed_kalman.anchor()?;
        if !(0. ..=crate::helpers::SCAN_AGE_S).contains(&(at-anchor_at)) { return None; }
        Some(((body+current-origin).max(0.)*axis_cos).max(0.))
    }

    pub fn imu_history_dump(&self) -> V {
        V::list(self.imu_history.iter().map(|r| V::from(vec![r.0,r.1,r.2])).collect())
    }

    pub fn speed_fusion_diagnostics(&self) -> V {
        let last = self.speed_kalman.last();
        crate::dict! {
            "observed_at" => last.map(|v| v.at),
            "radar_body_speed_mps" => last.map(|v| v.measured),
            "predicted_body_speed_mps" => last.and_then(|v| v.predicted),
            "innovation_mps" => last.and_then(|v| v.predicted.map(|p| v.measured-p)),
            "correction_mps" => last.and_then(|v| v.predicted.map(|p| v.filtered-p)),
            "filtered_body_speed_mps" => last.map(|v| v.filtered),
            "innovation_weight" => last.map(|v| v.gain),
            "measurement_noise_scale" => last.map(|v| v.noise_scale),
            "speed_variance" => last.map(|v| v.variance),
            "basis" => "kalman_imu_prediction_radar_correction_with_scaled_outliers",
        }
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
