//! The open end of the light lane's inner board, observed while passing cone one.
//! Carry that same gap through the two stationary cones as the car turns.
use crate::compact_target::CompactTargetTracker;
use crate::helpers::{FRONT_BODY_EXTENT_M, OPEN_LANE_MIN_SPAN_M, OPEN_LINE_MIN_POINTS, SIDE_BODY_EXTENT_M, dcos, dsin};
use crate::maneuver::{
    LIGHT_GANTRY_EDGE_M, LIGHT_GANTRY_SPAN_M, LIGHT_GANTRY_ALONG_DIFF_M, LIGHT_LEG_LATERAL_M,
    LIGHT_LINE_JUMP_M, LIGHT_POST_JUMP_M, OPEN_LANE_WIDTH_M,
};
use crate::py::V;
use crate::motion_sensors::{RangeHistory, lane_range_speed};
use crate::maneuver::{LANE_SPEED_BASELINE, LANE_SPEED_HISTORY};
use crate::race_speed_kalman::SpeedKalman;
#[path = "race_light_trim.rs"]
pub(crate) mod trim;

fn vote(votes: &mut [(u32, f64); 2], value: f64) {
    if let Some((n, mean)) = votes.iter_mut()
        .filter(|(n, m)| *n > 0 && (*m - value).abs() <= LIGHT_LINE_JUMP_M)
        .min_by(|a, b| (a.1 - value).abs().total_cmp(&(b.1 - value).abs())) {
        *n += 1;
        *mean += (value - *mean) / f64::from(*n);
    } else if let Some(slot) = votes.iter_mut().min_by_key(|v| v.0) {
        *slot = (1, value);
    }
}

fn voted([a, b]: [(u32, f64); 2]) -> Option<(u32, f64)> {
    let best = if a.0 > b.0 { a } else { b };
    (best.0 >= 2 && a.0 != b.0).then_some(best)
}

/// Use the physical support, never a shifted planning circle or a held target.
fn fresh_center(scan: &V, target: &V) -> Option<(f64, f64)> {
    if !target.get("confirmed").is_true() || !target.get("tracking_only_boundary_gap").is_false()
        || !crate::py::py_eq(target.get("source_seq"), scan.get("seq"))
        || !crate::py::py_eq(target.get("source_at_ms"), scan.get("at_ms"))
        || !crate::py::py_eq(target.get("source_received_at"), scan.get("received_at")) { return None; }
    let ranges = scan.get("ranges").ol()?;
    let bins = target.get("support_bins").ol()?;
    if ranges.len() != 360 || bins.is_empty()
        || bins.iter().any(|b| !b.is_int() || !(0..360).contains(&b.i()) || !ranges[b.i() as usize].is_num()) { return None; }
    let ranges: Vec<_> = ranges.iter().map(V::of).collect();
    let bins: Vec<_> = bins.iter().map(|b| b.i() as usize).collect();
    Some(crate::helpers::support_center(&ranges, &bins, target.get("diameter_m")).0)
}

fn along(center: (f64, f64), heading: f64) -> f64 {
    center.0 * heading.cos() - center.1 * heading.sin()
}

#[derive(Clone, Copy, Debug)]
struct GapPosition {
    observed_at: f64,
    at: f64,
    ahead: f64,
    axial_speed: Option<f64>,
}

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

    /// Current interval only: do not transport older intervals through the
    /// same turning accelerometer drift that this measurement is checking.
    pub fn measured_this_scan(&self, seq: i64) -> Option<f64> {
        let p = self.previous.as_ref().filter(|p|p.seq==seq)?;
        self.anchors.iter().flatten().find(|&&(t,_,_)|t==p.at).map(|&(_,v,_)|v)
    }
}

#[cfg(test)]
mod entry_motion_tests {
    use super::*;
    use crate::motion_sensors::RaceInputs;

    #[test]
    fn reference_change_never_interpolates_different_origins() {
        let mut r=RaceLight::default();
        let sample=|r:&mut RaceLight,t:f64,g:f64,source,v| {
            r.opening_source=Some(source);r.opening_sample=Some(OpeningSample{at:t,ahead:g+1.19,speed:Some(v)});
            r.observe_beam(t,Some(g));
        };
        sample(&mut r,13.383,2.52,"surveyed_finish_opening",2.0);
        sample(&mut r,13.581,-1.43,"end_board_to_measured_opening",2.2);
        assert!(r.beam_at.is_none());
        sample(&mut r,13.681,1.91,"end_board_to_measured_opening",2.2);
        assert!(r.beam_at.is_none());
        sample(&mut r,14.6,0.08,"surveyed_finish_opening",1.2);
        sample(&mut r,14.7,-0.02,"confirmed_gantry_stations",1.2);
        // Current plane at the previous pose: -.02+.12=.10, not the old .08.
        let t=r.beam_at.unwrap();assert!((t-(14.6+0.1*0.10/0.12)).abs()<1e-9);
        assert_eq!(r.beam_basis,Some("motion_transported_wheel_bracket"));
        r.observe_beam(15.,None);sample(&mut r,16.,0.14,"visible_divider_opening",0.);
        assert_eq!(r.beam_at,Some(t));
    }

    #[test]
    fn a_fresh_cone_corrects_a_missing_gantry_speed_without_renewing_missing_samples() {
        let mut light = RaceLight::default();
        light.speed_kalman.update(16.848, 0., 1.2957473688466818);
        light.speed_from_cone = false;
        let speed = light.adjusted_speed(Some(-0.05), 16.947, None, 1., Some(1.67702519811144)).unwrap();
        assert!(speed > 1.2957473688466818 - 0.05 && speed < 1.67702519811144);
        assert!(!light.speed_from_cone); // keep the established filter, not a new seed
        let at = light.speed_kalman.anchor().unwrap().0;
        let bridged = light.adjusted_speed(Some(-0.15), 17.047, None, 1., None).unwrap();
        assert!((bridged - (speed - 0.1)).abs() < 1e-9);
        assert_eq!(light.speed_kalman.anchor().unwrap().0, at);
    }

    fn scan(seq: i64, at: f64) -> V { crate::dict! {"seq" => seq, "received_at" => at} }

    #[test]
    fn a_carried_gap_ignores_a_different_wall_and_does_not_refresh_missing_observations() {
        let input = RaceInputs { yaw_rate: Some(0.), forward_integral: Some(0.) };
        let mut light = RaceLight::default();
        light.entry_motion.observe(&scan(1, 0.), 0., &1.into(), (3., -1.), input);
        light.entry_motion.observe(&scan(2, 0.1), 0.1, &1.into(), (2.8, -1.), input);
        let current = crate::dict! {"seq" => 2, "at_ms" => 100, "received_at" => 0.1};
        assert!((light.measure_speed(&current, None, Some(0.), 1.).1.unwrap() - 2.).abs() < 1e-9);
        light.second_gap = [(2, 2.), (0, 0.)];
        light.gap_position = Some(GapPosition { observed_at: 0.1, at: 0.1, ahead: 2., axial_speed: Some(2.) });
        let wrong_wall = crate::dict! {"seq" => 3, "received_at" => 0.2, "ranges" => vec![V::Float(9.); 360]};
        assert!((light.project(&wrong_wall, 0., 0.2, input).unwrap() - 1.8).abs() < 1e-9);
        assert_eq!(light.project(&wrong_wall, 0., 1.2, input), None);
    }

    #[test]
    fn cone_speeds_restart_the_filter_until_an_aligned_radar_speed_corrects_it() {
        // y88 seq 156-159: the turning IMU integral fell 0.37 m/s while the cone speed held.
        let mut light = RaceLight::default();
        light.adjusted_speed(Some(0.591), 15.584, None, 0.23, Some(1.963));
        light.adjusted_speed(Some(0.511), 15.684, None, 0.38, Some(1.883));
        light.adjusted_speed(Some(0.223), 15.886, None, 0.73, Some(1.951));
        let (at, body, origin) = light.speed_kalman.anchor().unwrap();
        assert_eq!((at, body, origin), (15.886, 1.951, 0.223));
        // Once a radar speed has corrected it, later cone observations use
        // the same outlier weighting rather than replacing the filter.
        light.speed_kalman.update(15.984, 0.072, 1.935);
        light.speed_from_cone = false;
        let before = light.speed_kalman.anchor();
        light.adjusted_speed(Some(-0.036), 16.085, None, 0.94, Some(1.0));
        let (at, body, _) = light.speed_kalman.anchor().unwrap();
        assert_eq!(at, 16.085);
        assert!(body > 1.0 && body < before.unwrap().1 - 0.108);
    }

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
    gantry.is_some_and(|g| (g - along - crate::race_field_map::beam_spacing()).abs() <= LIGHT_LINE_JUMP_M)
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
                    (x - a.0 - crate::race_field_map::beam_spacing())
                        .abs()
                        .total_cmp(&(x - b.0 - crate::race_field_map::beam_spacing()).abs())
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

/// Shared physical end-board observation. Isolated closer posts and oblique
/// side returns must not become a changing terminal-distance reference.
pub(crate) fn observed_end_wall(scan: &V, heading: f64) -> Option<f64> {
    ScanAxis::new(scan, heading)?.end_wall()
}

/// Looking down the finish lane, the *right* board runs downstream from the
/// opening. At its upstream tip neighbouring clockwise rays must actually go
/// through the opening. The first point of a truncated line fit isn't a tip.
fn right_opening_tip(scan:&V,heading:f64,outer:f64,body:f64,yaw:f64)->Option<(f64,f64)> {
    let r=scan.get("ranges").ol().filter(|v|v.len()==360)?;
    let ages=scan.get("point_age_ms").ol();
    let point=|i:usize| {
        let d=r[i].of()?;let age=ages.and_then(|v|v.get(i)).and_then(V::of).unwrap_or(0.).clamp(0.,150.)/1000.;
        let (s,c)=((i as f64).to_radians()-heading+yaw*age).sin_cos();
        let (sy,cy)=(heading-0.5*yaw*age).sin_cos();
        Some((d*c-body*age*cy,-d*s-body*age*sy))
    };
    let mut best:Option<(f64,f64)>=None;
    for i in 0..360 {
        let Some(d)=r[i].of() else{continue};
        if !r[(i+1)%360].of().is_some_and(|n|n-d>LIGHT_POST_JUMP_M) {continue}
        let (x,y)=point(i)?;
        if y>=-SIDE_BODY_EXTENT_M || !(OPEN_LANE_WIDTH_M.0..=OPEN_LANE_WIDTH_M.1).contains(&(outer-y)){continue}
        if (1..=3).filter_map(|k|point((i+k)%360)).filter(|p|p.1<y-LIGHT_LINE_JUMP_M).count()<2 {continue}
        let mut count=1;let mut far=x;
        for k in 1..=40 {
            let Some((u,v))=point((i+360-k)%360) else {break};
            if (v-y).abs()>crate::helpers::OPEN_LINE_BAND_M || u<x-crate::helpers::OPEN_LINE_BAND_M {break}
            count+=1;far=far.max(u);
            if count>=3 && far-x>=OPEN_LANE_MIN_SPAN_M {break}
        }
        if count<3 || far-x<OPEN_LANE_MIN_SPAN_M {continue}
        if best.is_some_and(|(u,_)|(u-x).abs()>LIGHT_LINE_JUMP_M){return None}
        best=Some((x,y));
    }
    best
}

#[derive(Clone,Copy,Debug)]
struct TipFix { at:f64, ahead:f64, side:f64, map:Option<f64>, confirmed:bool }

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

#[derive(Clone, Copy, Default, Debug, PartialEq, Eq)]
pub enum LightRoll {
    #[default]
    Cruise,
    Hold,
    Coast,
}

impl LightRoll {
    pub fn label(self) -> &'static str {
        match self {
            Self::Cruise => "cruise",
            Self::Hold => "approach_hold",
            Self::Coast => "rolling_low",
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct LightRest {
    first: f64,
    last: f64,
    reference: &'static str,
    min_range: f64,
    max_range: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct OpeningSupport {
    pub mapped: Option<f64>,       // current two-axis board fix consistent with the IMU prior
    pub station: Option<f64>,      // positively identified gantry, not an arbitrary post
    pub live_legs: bool,
    pub sees_both_stations: bool,
    pub axis_speed: Option<f64>,
    pub edge: Option<f64>,
}

#[derive(Clone, Copy, Debug)]
struct OpeningSample { at: f64, ahead: f64, speed: Option<f64> }

impl OpeningSample {
    fn fits(self, at: f64, ahead: f64, speed: Option<f64>) -> bool {
        let dt = at-self.at;
        let travel = self.ahead-ahead;
        match self.speed.zip(speed).map(|(a,b)| (a+b)*0.5).or(speed).or(self.speed) {
            Some(v) => (travel-v*dt).abs() <= LIGHT_LINE_JUMP_M,
            None => (-LIGHT_LINE_JUMP_M..=crate::maneuver::LANE_MAX_CLOSING_MPS*dt+LIGHT_LINE_JUMP_M).contains(&travel),
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct BeamSample {
    at: f64,
    gap: f64,
    source: &'static str,
    wheel_extent: f64,
    axis_speed: Option<f64>,
}

#[derive(Clone, Default, Debug)]
pub struct RaceLight {
    pub trim: trim::Trim,
    pub housing: crate::race_lamp_range::HousingTrack,
    tip_fix: Option<TipFix>,
    pub raw_tip: Option<f64>,
    pub entry_motion: EntryMotionSpeed,
    pub entry_tracker: Option<CompactTargetTracker>,
    votes: [(u32, f64); 2],
    first_gap: [(u32, f64); 2],
    second_gap: [(u32, f64); 2],
    second_identity: V,
    second_seq: Option<i64>,
    gap_position: Option<GapPosition>,
    last_seq: Option<i64>,
    pub phase: LightRoll,
    pub prior_ahead: Option<f64>,
    prior_from_cone: bool,
    opening_spans: [(u32, f64); 2],
    opening_seq: Option<i64>,
    opening_sample: Option<OpeningSample>,
    direct_station: Option<OpeningSample>,
    station_offset: Option<f64>,
    pub opening_rebased: bool,
    pub opening_source: Option<&'static str>,
    pub prev_opening_source: Option<&'static str>,
    pub completing_zone_stop: bool,
    pub front_wheel_gap_m: Option<f64>,
    previous_beam_gap: Option<BeamSample>,
    rest: Option<LightRest>,
    pub ranges: RangeHistory,
    pub speed_uses_gantry: bool,
    pub last_speed: Option<(f64, f64)>,
    imu_history: Vec<(f64, f64, f64)>, // scan time, body IMU integral, lane projection cos
    speed_kalman: SpeedKalman,
    /// The filter has only been set by cone speeds, never by an aligned radar speed.
    speed_from_cone: bool,
    /// When the front was first estimated past the photo beam (y34).
    pub beam_at: Option<f64>,
    pub beam_basis: Option<&'static str>,
    approach_sample: Option<(i64, f64, Option<f64>)>,
}

impl RaceLight {
    pub fn register_housing(
        &mut self,
        map: &crate::race_field_map::Locator,
        scan: &V,
        heading: f64,
    ) -> Option<crate::race_lamp_range::Plane> {
        self.housing.direct = None;
        let raw_edge = self
            .tip_fix
            .filter(|f| f.confirmed && Some(f.at) == scan.get("received_at").of())
            .map(|f| f.ahead);
        let shot = self.housing.latest?;
        let plane = map.housing_plane(scan, shot)?;
        let mapped = map.finish_divider_ahead(scan);
        if self.beam_at.is_none()
            && self.housing.pending().is_some()
            && (map.motion_supported(scan) || (plane.both && raw_edge.is_some()))
        {
            let end = (heading.abs() <= crate::maneuver::open_lane_entry_heading_rad()
                && map.finish_ahead(scan).is_some())
            .then(|| observed_end_wall(scan, heading))
            .flatten();
            self.housing.register(shot, plane, raw_edge, end);
        }
        // Measure the car-to-lamp distance again even after the front-wheel clock
        // latches. This keeps the stop position relative to the fixture when the
        // end wall is out of LiDAR range. The one-shot clock itself cannot move.
        let allowance = (2.0 * plane.uncertainty).clamp(0.10, 0.22);
        let agrees = [
            self.housing.ahead(mapped, None, None),
            self.housing.ahead(None, raw_edge, None),
        ]
        .into_iter()
        .flatten()
        .any(|(d, _)| (plane.ahead - d).abs() <= allowance);
        if agrees && (plane.both || plane.uncertainty <= 0.12) {
            self.housing.direct = scan.get("seq").oi().map(|seq| (seq, plane.ahead));
        }
        Some(plane)
    }

    /// Anchor the surveyed plane to the actual edge this run. Only relative
    /// displacement is carried across a brief dropout. Missing scans don't refresh it.
    pub fn observe_lane_tip(&mut self,scan:&V,h:f64,lane:&V,map:Option<f64>,body:f64,yaw:f64,axis_v:Option<f64>)->Option<f64> {
        let at=scan.get("received_at").of()?;
        let old=self.tip_fix;
        let tip=lane.get("outer_lateral_m").of().filter(|&o|h.abs()<=crate::maneuver::open_lane_entry_heading_rad()
            && lane.get("width_m").of().is_some_and(|w|(0. ..=w).contains(&o)))
            .and_then(|o|right_opening_tip(scan,h,o,body,yaw))
            .filter(|&(x,_)|!at_predicted_beam(x,map));
        self.raw_tip=tip.map(|v|v.0);
        if let Some((ahead,side))=tip {
            if old.is_none_or(|s|s.at!=at) {
                let confirmed=old.is_some_and(|s| {
                    let dt=at-s.at;let travel=s.map.zip(map).map(|(a,b)|a-b).or_else(||axis_v.map(|v|v*dt));
                    dt>0. && dt<=crate::helpers::SCAN_AGE_S && (side-s.side).abs()<=LIGHT_LINE_JUMP_M
                        && travel.is_some_and(|dx|(s.ahead-ahead-dx).abs()<=LIGHT_POST_JUMP_M)
                });
                self.tip_fix=Some(TipFix{at,ahead,side,map,confirmed});
            }
            if self.tip_fix.is_some_and(|s|s.confirmed){return Some(ahead)}
        }
        let f=old.filter(|s|s.confirmed && (0. ..=crate::helpers::SCAN_AGE_S).contains(&(at-s.at)))?;
        Some(f.ahead + (map?-f.map?))
    }

    /// The finish divider is a fixed reference. A directly identified gantry or
    /// a camera-registered black backplate can occupy a different longitudinal plane.
    pub fn opening_ahead(
        &mut self,
        scan: &V,
        heading: f64,
        lane: &V,
        support: OpeningSupport,
    ) -> Option<f64> {
        let OpeningSupport { mapped, station, live_legs, sees_both_stations, axis_speed, edge } = support;
        let at = scan.idx("received_at").f();
        let last = self.opening_sample.filter(|s| (0. ..=crate::helpers::SCAN_AGE_S).contains(&(at-s.at)));
        let consistent = |d: f64| last.is_none_or(|s|s.fits(at,d,axis_speed));
        // A continuous but misplaced survey must not lock out an identified physical
        // station indefinitely. Two fresh leg measurements can establish a different
        // origin using THEIR displacement, never the step from the survey to the legs.
        let live = station.filter(|_|live_legs);
        // A leg classified FROM the old divider is not an independent witness to
        // overturn that divider. Require this scan's spacing or its actual board tip.
        let identified = live.filter(|d| sees_both_stations || lane.get("divider_corner_along_m").of()
            .is_some_and(|tip|(d-tip).abs()<=LIGHT_LINE_JUMP_M));
        let station_motion=identified.zip(self.direct_station).is_some_and(|(d,s)|
            at>s.at && at-s.at<=crate::helpers::SCAN_AGE_S && s.fits(at,d,axis_speed));
        self.opening_rebased = station_motion && identified.is_some_and(|d|!consistent(d));
        if station_motion && sees_both_stations && let Some((d,g))=live.zip(mapped) {
            let offset=d-g;
            self.station_offset=Some(self.station_offset.filter(|b|(b-offset).abs()<LIGHT_LINE_JUMP_M)
                .map_or(offset,|b|0.5*(offset+b)));
        }
        self.direct_station = identified.map(|ahead| OpeningSample{at,ahead,speed:axis_speed});
        let station_rebased = self.opening_rebased;
        // During the entry turn, central rays can hit the near face of the
        // divider. Only subtract its length when looking down the actual lane.
        let along_lane = heading.abs() <= crate::maneuver::open_lane_entry_heading_rad();
        let in_lane = lane
            .get("outer_lateral_m")
            .of()
            .zip(lane.get("width_m").of())
            .is_some_and(|(outer, width)| (0.0..=width).contains(&outer));
        let wall = (along_lane && in_lane).then(|| observed_end_wall(scan, heading)).flatten();
        let same = |a:f64,b:f64| (a-b).abs() <= LIGHT_LINE_JUMP_M;
        // A truncated fitted line is not necessarily the end of the divider. Pair it
        // only when its observed near end agrees with another opening observation.
        let tip = lane.get("divider_corner_along_m").of().or_else(|| {
            lane.get("inner_start_along_m")
                .of()
                .filter(|d| *d > 0. && lane.get("inner_source").os() == Some("raw_returns")
                    && station.or(mapped).or(self.prior_ahead).is_some_and(|a| same(a,*d)))
        });
        let seq = scan.get("seq").oi();
        if seq != self.opening_seq {
            self.opening_seq = seq;
            if let Some(span) =
                wall.zip(live.filter(|&d|consistent(d) || self.opening_rebased).or(tip.filter(|&d|consistent(d))))
                    .map(|(w, opening)| w - opening).filter(|d| *d > OPEN_LANE_MIN_SPAN_M)
            {
                vote(&mut self.opening_spans, span);
            }
        }
        let local_span = voted(self.opening_spans);
        let wall_opening = wall.zip(local_span.or_else(|| self.landmark())).map(|(w, (_, span))| w-span);
        let map = mapped.filter(|&d|consistent(d));
        let station = station.filter(|&d|consistent(d) || self.opening_rebased);
        let tip = along_lane.then_some(tip).flatten().filter(|&d|consistent(d));
        let from_wall = wall_opening.filter(|&d|consistent(d));
        // Keep the common surveyed frame while supported. A directly identified plane
        // can correct the survey. A remote wall minus an old span cannot outvote both
        // the current wall fix and the displacement since the last accepted opening.
        let physical = station.filter(|_|live_legs).map(|d|(d,"confirmed_gantry_stations"))
            .or_else(|| tip.map(|d|(d,"visible_divider_opening")))
            .or_else(|| local_span.and(from_wall).map(|d|(d,"end_board_to_measured_opening")));
        let position = map.filter(|&d|physical.is_none_or(|(v,_)|same(d,v)))
            .map(|d|(d,"surveyed_finish_opening"))
            .or(physical)
            .or_else(|| station.map(|d|(d,"confirmed_gantry_stations")))
            .or_else(|| (along_lane && in_lane && self.prior_from_cone).then_some(self.prior_ahead).flatten()
                .filter(|&d|consistent(d)).map(|d|(d,"tracked_divider_opening")))
            .or_else(|| from_wall.map(|d|(d,"end_board_to_measured_opening")));
        // Matching within the association window is not the same as sharing a
        // longitudinal origin. Keep a repeatedly observed station's actual
        // offset when the map supplies subsequent motion (even when <25 cm).
        let calibrated=mapped.zip(self.station_offset).filter(|(_,b)|b.abs()>crate::helpers::OPEN_LINE_BAND_M)
            .map(|(g,b)|g+b).filter(|d|consistent(*d)||station_motion);
        let position=if let Some(d)=calibrated.filter(|_|sees_both_stations || edge.is_none()) {
            Some((d,"registered_station_opening"))
        } else if let Some(d)=edge.filter(|x|(!sees_both_stations || !live_legs)
            && map.is_none_or(|g|(g-x).abs()>crate::helpers::OPEN_LINE_BAND_M)) {
            let source=if self.raw_tip.is_some() {"measured_right_board_opening"} else {"carried_right_board_opening"};
            self.opening_rebased |= !consistent(d);
            Some((d,source))
        }else{position};
        let from_housing = self
            .housing
            .direct
            .filter(|(n, _)| Some(*n) == seq)
            .map(|(_, d)| (d, "measured_housing"))
            .or_else(|| self.housing.ahead(mapped, edge, wall))
            .or_else(|| match position {
                Some((d, "visible_divider_opening" | "tracked_divider_opening")) => {
                    self.housing.ahead(None, Some(d), None)
                }
                _ => None,
            });
        let position = if let Some(visual) = from_housing {
            // An independently identified physical station is already in lamp coordinates.
            // The camera offset only transfers divider/end-wall distances into that frame.
            // A pair at the entrance can belong to a separate support. After the
            // black frame has been registered, use post ranges only for the same plane.
            let same_lamp = |d: f64| (d - visual.0).abs() <= LIGHT_GANTRY_EDGE_M;
            let station = live.filter(|&d| sees_both_stations && same_lamp(d));
            let (d, source) = if let Some(d) = station {
                (d, "confirmed_gantry_stations")
            } else if let Some((m, b)) = mapped
                .zip(self.station_offset)
                .filter(|(m, b)| visual.1 != "measured_housing" && same_lamp(m + b))
            {
                (m + b, "registered_station_opening")
            } else {
                visual
            };
            let revised = self.housing.changed_reference();
            self.opening_rebased = !consistent(d)
                || (revised
                    && last.is_some_and(|s| {
                        let projected =
                            s.ahead - axis_speed.or(s.speed).unwrap_or(0.0) * (at - s.at);
                        (d - projected).abs() > crate::helpers::OPEN_LINE_BAND_M
                    }));
            if self.opening_rebased {
                self.previous_beam_gap = None;
            }
            Some((d, source))
        } else {
            if station_rebased {
                self.previous_beam_gap = None;
            }
            position
        };
        self.opening_source = position.map(|(_, source)| source);
        if let Some((ahead,_)) = position {self.opening_sample = Some(OpeningSample{at,ahead,speed:axis_speed});}
        position.map(|(ahead, _)| ahead)
    }

    /// One clock per run. Interpolate a bracketed crossing on the program's
    /// monotonic clock. A later missing scan or upstream correction cannot rearm it.
    pub fn observe_beam(&mut self, at: f64, wheel_gap: Option<f64>) {
        self.front_wheel_gap_m = wheel_gap;
        if self.beam_at.is_some() {
            return;
        }
        if let (Some(gap), Some(source), Some(o)) = (wheel_gap, self.opening_source, self.opening_sample) {
            let extent = o.ahead - crate::maneuver::LIGHT_SENSOR_TO_GANTRY_M - gap;
            if let Some(old) = self.previous_beam_gap && gap <= 0. && at > old.at && at-old.at <= crate::helpers::SCAN_AGE_S {
                let dt = at-old.at;
                let travel = old.axis_speed.zip(o.speed).map(|(a,b)| (a+b)*0.5*dt + extent-old.wheel_extent);
                let sane = (gap-old.gap).abs() <= crate::maneuver::LANE_MAX_CLOSING_MPS*dt + LIGHT_GANTRY_EDGE_M;
                if old.source == source && sane && old.gap <= 0. {
                    // Reacquisition downstream: the first confirmed inside observation,
                    // not a crossing backdated over a blind interval.
                    self.beam_at=Some(old.at);
                    self.beam_basis=Some("confirmed_inside_after_reacquisition");
                } else if old.gap>0. {
                    // On a switch, transport THIS plane to the preceding wheel pose.
                    // Don't interpolate two raw gaps with different origins.
                    let before = if old.source==source && sane {
                        travel.is_none_or(|d| d>0. && (old.gap-gap-d).abs()<=LIGHT_GANTRY_EDGE_M).then_some(old.gap)
                    } else {
                        travel.map(|d|gap+d).filter(|b| *b>0. && (*b-old.gap).abs()<=LIGHT_GANTRY_EDGE_M)
                    };
                    if let Some(b) = before {
                        self.beam_at=Some(old.at+dt*b/(b-gap));
                        self.beam_basis=Some(if old.source==source {"same_plane_wheel_bracket"} else {"motion_transported_wheel_bracket"});
                    }
                }
            }
            self.previous_beam_gap = Some(BeamSample{at,gap,source,wheel_extent:extent,axis_speed:o.speed});
        } else {
            self.previous_beam_gap = None;
        }
    }

    /// A neutral acknowledgement and a stable measured longitudinal range
    /// distinguish a stop from rolling slowly through the zone on an expired timer.
    pub fn stationary_in_zone(&mut self, at: f64, sample: Option<(&'static str, f64)>, stable_s: f64) -> bool {
        let Some((reference, range)) = sample else {
            self.rest = None;
            return false;
        };
        let w =
            self.rest.get_or_insert(LightRest { first: at, last: at, reference, min_range: range, max_range: range });
        let min_range = w.min_range.min(range);
        let max_range = w.max_range.max(range);
        if reference != w.reference
            || at < w.last
            || at - w.last > crate::helpers::SCAN_AGE_S
            || max_range - min_range > crate::coast::MAX_STABLE_TOTAL_TRANSLATION_M
        {
            *w = LightRest { first: at, last: at, reference, min_range: range, max_range: range };
        } else {
            w.last = at;
            w.min_range = min_range;
            w.max_range = max_range;
        }
        at - w.first >= stable_s
    }

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
        let seed = self.entry_motion.previous.as_ref().filter(|p| p.seq == scan.idx("seq").i())
            .and_then(|p| self.entry_motion.speed(p.at, forward_integral));
        (raw, self.adjusted_speed(forward_integral, at, raw, axis_cos, seed))
    }

    /// Range differences measure lane-axis motion, whose projection changes
    /// while the car leaves the second loop. Correct each existing difference
    /// interval before taking its median, then project the current body speed.
    fn adjusted_speed(&mut self, forward_integral: Option<f64>, at: f64, raw: Option<f64>, axis_cos: f64, seed: Option<f64>) -> Option<f64> {
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
        let mut radar = None;
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
                radar = Some(bodies[count/2]);
            }
        }
        // Initialize from this scan's identified physical cone. Otherwise the
        // first displaced gantry range gets gain 1 before any prior exists.
        // Do not count two correlated lidar observations at the same time.
        // y88: seeded at 89 deg, the anchor then carried 0.5 s of hard-turn IMU
        // integral (-0.52 m/s) into the first aligned radar speed, which was
        // scaled as an outlier: 1.54 m/s filtered at a true 1.87, the step-down
        // and the brake came late and the car stopped 0.21 m from the gantry.
        // A fresh aligned gantry speed owns the correction. If it is missing,
        // a current cone speed supplies the missing correction rather than
        // being ignored; after the first aligned observation it keeps the
        // existing Kalman state and outlier weights.
        // y92 seq169-173 otherwise bridged the old low anchor during missing
        // legs (about1.01m/s against about1.5m/s measured range closure).
        // The seed requires this scan's fresh physical cone; held predictions
        // do not renew the observation clock.
        let anchored = self.speed_kalman.anchor().is_some();
        match (radar, seed) {
            (Some(body), _) if anchored => {
                // Keep physical acceleration on the IMU path; only the radar correction is
                // filtered. Missing radar never reaches the filter (race_speed_kalman).
                self.speed_kalman.update(at, current, body);
                self.speed_from_cone = false;
            }
            (_, Some(cone)) if !anchored || self.speed_from_cone => {
                self.speed_kalman.clear();
                self.speed_kalman.update(at, current, cone);
                self.speed_from_cone = true;
            }
            (None, Some(cone)) => self.speed_kalman.update(at, current, cone),
            (Some(body), None) => {
                self.speed_kalman.update(at, current, body);
                self.speed_from_cone = false;
            }
            _ => {}
        }
        let (anchor_at, body, origin) = self.speed_kalman.anchor()?;
        if !(0. ..=crate::helpers::SCAN_AGE_S).contains(&(at-anchor_at)) { return None; }
        Some(((body+current-origin).max(0.)*axis_cos).max(0.))
    }

    pub fn body_speed(&self, scan: &V, now: f64, forward_integral: Option<f64>) -> Option<f64> {
        let current = forward_integral.filter(|v| v.is_finite())?;
        let at = scan.idx("at_ms").i() as f64 * 0.001;
        let kalman = self.speed_kalman.anchor()
            .filter(|&(anchor_at, _, _)| (0. ..=crate::helpers::SCAN_AGE_S).contains(&(at - anchor_at)))
            .map(|(_, body, origin)| (body + current - origin).max(0.));
        let entry = self.entry_motion.speed(now, forward_integral);
        if self.speed_kalman.last().is_some_and(|v| (v.at - at).abs() < 1e-6) {
            kalman.or(entry)
        } else {
            entry.or(kalman)
        }
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

    pub fn observe_first(&mut self, scan: &V, heading: f64, first: &V) {
        let seq = scan.idx("seq").i();
        if self.last_seq == Some(seq) {
            return;
        }
        self.last_seq = Some(seq);
        let Some(axis) = ScanAxis::new(scan, heading) else {
            return;
        };
        let Some((wall, span)) = axis.end_wall().and_then(|w| Some((w, axis.divider_span(w)?))) else {
            return;
        };
        // Keep the consistent landmark across viewpoints, not an isolated gap.
        vote(&mut self.votes, span);
        if let Some(center) = fresh_center(scan, first) {
            vote(&mut self.first_gap, wall - span - along(center, heading));
        }
    }
    pub fn landmark(&self) -> Option<(u32, f64)> {
        voted(self.votes)
    }

    #[allow(clippy::too_many_arguments)]
    pub fn observe_second(&mut self, scan: &V, now: f64, heading: f64, second: &V, first: &V,
        input: crate::motion_sensors::RaceInputs) {
        let Some(center) = fresh_center(scan, second) else { return };
        let identity = second.get("track_id");
        if !crate::py::py_eq(&self.second_identity, identity) {
            self.second_gap = Default::default();
            self.gap_position = None;
            self.second_seq = None;
            self.second_identity = identity.clone();
        }
        if self.second_seq == Some(scan.idx("seq").i()) { return; }
        self.second_seq = Some(scan.idx("seq").i());
        self.entry_motion.observe(scan, now, identity, center, input);
        // Simultaneous observations cancel ego translation. The offsets are
        // measured each run; neither cone spacing nor lamp position is preset.
        if let Some((_, offset)) = voted(self.first_gap)
            && let Some(first) = fresh_center(scan, first) {
            vote(&mut self.second_gap, offset + along(first, heading) - along(center, heading));
        }
        if let Some((_, offset)) = voted(self.second_gap) {
            self.gap_position = Some(GapPosition {
                observed_at: scan.idx("received_at").f(), at: now,
                ahead: along(center, heading) + offset,
                axial_speed: self.entry_motion.speed(now, input.forward_integral).map(|v| v * heading.cos()),
            });
        }
    }

    pub fn project(&mut self, scan: &V, heading: f64, now: f64, input: crate::motion_sensors::RaceInputs) -> Option<f64> {
        // Continue the already identified cone in a private tracker: this must
        // not rebind a route target or start route recovery after the handover.
        if let Some(tracker) = &mut self.entry_tracker {
            let target = tracker.update(scan, now, false, Some(heading), None);
            if crate::py::py_eq(target.get("track_id"), &self.second_identity) {
                self.observe_second(scan, now, heading, &target, &V::None, input);
            }
        }
        let carried = if voted(self.second_gap).is_some() {
            self.gap_position.as_mut().and_then(|p| {
                if !(0. ..=crate::race_speed_kalman::BRIDGE_S).contains(&(now - p.observed_at)) { return None; }
                let current = self.entry_motion.speed(now, input.forward_integral).map(|v| v * heading.cos());
                if now > p.at {
                    p.ahead -= (p.axial_speed? + current?) * (now - p.at) * 0.5;
                    p.at = now;
                    p.axial_speed = current;
                }
                Some(p.ahead)
            })
        } else {
            None
        };
        self.prior_from_cone = carried.is_some();
        self.prior_ahead = carried.or_else(|| self.landmark().and_then(|(_, span)| Some(observed_end_wall(scan, heading)? - span)));
        self.prior_ahead
    }

    pub fn position_diagnostics(&self) -> V {
        crate::dict! {"basis" => if voted(self.second_gap).is_some() { "first_gap_carried_by_cones" } else { "end_wall_offset" },
            "gap_from_first_cone_m" => voted(self.first_gap).map(|v| v.1),
            "gap_from_second_cone_m" => voted(self.second_gap).map(|v| v.1),
            "observed_at" => self.gap_position.map(|v| v.observed_at),
            "source_seq" => self.second_seq, "ahead_m" => self.prior_ahead,
            "housing_ahead_at_measurement_m"=>self.housing.last_range,
            "housing_minus_divider_m"=>self.housing.registered.map(|r|r.offset),
            "end_wall_minus_housing_m"=>self.housing.registered.and_then(|r|r.end_span)}
    }
}
