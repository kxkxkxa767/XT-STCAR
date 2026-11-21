//! A start lane too wide for the native corridor detector.
//!
//! The detector fits each side wall over 45-135 degrees. In a lane wider than about 1.4 m the
//! rays behind the rear quarter land on the start zone's end wall, the side fit loses its 80 %
//! inlier share and no lane is reported, so the run was refused with `left_corridor_unknown`
//! (rule-range simulation 11-08: 36 % of starts at 1.5-1.6 m, 84 % above 1.8 m). These sectors
//! stop before the end wall. The result only decides whether the start is accepted; driving
//! keeps its own wall, side-wall and compass steering.
use crate::dict;
use crate::helpers::{dcos, dsin};
use crate::py::*;

/// Clockwise bins from 40 degrees ahead of abeam to 10 degrees behind it.
const RIGHT_BINS: std::ops::RangeInclusive<usize> = 50..=100;
const LEFT_BINS: std::ops::RangeInclusive<usize> = 260..=310;
const MIN_POINTS: usize = 18;
const INLIER_M: f64 = 0.06;
const MIN_INLIER_SHARE: f64 = 0.60;
const MIN_SLOPE_PAIR_DX_M: f64 = 0.3;
const MAX_WALL_ANGLE_DIFF_RAD: f64 = 15. * PI / 180.;

struct Wall {
    slope: f64,
    intercept: f64,
    points: usize,
}

/// A placed competition car starts along +x. A single real side board can
/// refine that heading; successive calibrated gyro samples carry it when
/// neither side makes a line. This is an alternate input, never a start veto.
#[derive(Clone, Debug, Default)]
pub struct StartAxis {
    stamp: Option<(i64, i64)>,
    yaw: f64,
    rate: Option<f64>,
    pub hint: V,
}

impl StartAxis {
    pub fn observe(&mut self, scan: &V, rate: Option<f64>, measured: Option<f64>, width: f64) {
        let (Some(seq), Some(ms)) = (scan.get("seq").oi(), scan.get("at_ms").oi()) else {
            self.hint = V::None;
            return;
        };
        if self.stamp == Some((seq, ms)) {
            return;
        }
        let previous = self.stamp;
        self.stamp = Some((seq, ms));
        self.hint = V::None;
        let Some((ranges, true)) = crate::compact_target::ranges_info(scan.get("ranges")) else {
            return;
        };
        if !ranges.iter().any(Option::is_some) {
            return;
        }
        let raw = scan.get("ranges").l();
        let rate = rate.filter(|v| v.is_finite());
        let dt = previous.map_or(0., |(_, old)| {
            ((ms - old) as f64 / 1000.).clamp(0., crate::helpers::SCAN_AGE_S)
        });
        let predicted = self.yaw + dt * self.rate.zip(rate).map_or(rate.unwrap_or(0.), |(a, b)| 0.5 * (a + b));
        self.rate = rate;
        let right_count = RIGHT_BINS.filter(|&i| ranges[i].is_some()).count();
        let left_count = LEFT_BINS.filter(|&i| ranges[i].is_some()).count();
        let side = [(true, fit(raw, RIGHT_BINS)), (false, fit(raw, LEFT_BINS))]
            .into_iter()
            .filter_map(|(right, wall)| Some((right, wall?)))
            .filter(|(right, w)| {
                w.intercept * if *right { 1. } else { -1. } > 0.
                    && w.intercept.abs() < width + 0.25
                    && crate::py::wrap(w.slope.atan() - predicted).abs() < 35_f64.to_radians()
            })
            .max_by_key(|(_, w)| w.points);
        self.yaw = measured
            .or_else(|| side.as_ref().map(|(_, w)| w.slope.atan()))
            .unwrap_or(predicted);
        let offset = side.as_ref().map(|(right, w)| {
            let dist = w.intercept.abs() / w.slope.hypot(1.);
            if *right { width * 0.5 - dist } else { dist - width * 0.5 }
        });
        let basis = if measured.is_some() {
            "current_compass"
        } else if side.is_some() {
            "single_observed_side"
        } else if previous.is_some() && rate.is_some() {
            "placement_axis_and_imu"
        } else {
            "confirmed_placement_axis"
        };
        self.hint = crate::dict! {"source_seq"=>seq,"source_at_ms"=>ms,"reason"=>V::None,"width_m"=>width,
        "width_source"=>"surveyed_start_channel","heading_left_rad"=>-self.yaw,"compass_heading_rad"=>self.yaw,
        "center_offset_left_m"=>offset.unwrap_or(0.),"body_gap_m"=>V::None,"front_m"=>ranges[0],
        "start_axis_source"=>basis,"left_side_returns"=>left_count,"right_side_returns"=>right_count,
        "observed_side"=>side.as_ref().map(|(right,_)|if *right {"right"} else {"left"})};
    }

    pub fn carry_compass(&self, compass: &mut crate::helpers::WallCompass) {
        let Some((_, ms)) = self.stamp else { return };
        if self.hint.is_none() {
            return;
        }
        let at = ms as f64 / 1000.;
        let side = self.hint.get("start_axis_source").os() == Some("single_observed_side");
        compass.heading = Some(self.yaw);
        compass.at_s = Some(at);
        compass.rate = self.rate.unwrap_or(0.);
        if side {
            compass.measured_at_s = Some(at);
            compass.last_measurement = Some((self.yaw, at));
        }
        compass.evidence = crate::dict! {"source_at_ms"=>ms,"heading_rad"=>self.yaw,"rate_rad_s"=>compass.rate,
        "basis"=>"placed_race_start_axis","heading_source"=>self.hint.get("start_axis_source").clone(),"measured"=>side};
    }
}

/// Median-of-slopes line through one sector, y to the right of the car axis.
fn fit(ranges: &[V], bins: std::ops::RangeInclusive<usize>) -> Option<Wall> {
    let points: Vec<(f64, f64)> =
        bins.filter_map(|a| ranges[a].of().map(|r| (r * dcos(a), r * dsin(a)))).collect();
    if points.len() < MIN_POINTS {
        return None;
    }
    let mut slopes = Vec::new();
    for (i, &(x, y)) in points.iter().enumerate() {
        for &(x2, y2) in points[i..].iter().step_by(8).skip(1) {
            if (x2 - x).abs() >= MIN_SLOPE_PAIR_DX_M {
                slopes.push((y2 - y) / (x2 - x));
            }
        }
    }
    if slopes.is_empty() {
        return None;
    }
    let slope = median(slopes);
    let intercept = median(points.iter().map(|&(x, y)| y - slope * x).collect());
    let inliers = points.iter().filter(|&&(x, y)| (y - slope * x - intercept).abs() <= INLIER_M).count();
    ((inliers as f64) >= MIN_INLIER_SHARE * points.len() as f64).then_some(Wall { slope, intercept, points: inliers })
}

/// The lane through the car from its two side walls, in the shape of a native candidate.
pub fn wide_start_lane(ranges: &[V]) -> Option<V> {
    let right = fit(ranges, RIGHT_BINS)?;
    let left = fit(ranges, LEFT_BINS)?;
    if right.intercept <= 0.
        || left.intercept >= 0.
        || (right.slope.atan() - left.slope.atan()).abs() > MAX_WALL_ANGLE_DIFF_RAD
    {
        return None;
    }
    let slope = (right.slope + left.slope) / 2.;
    let cos = 1. / hypot(1., slope);
    Some(dict! {
        "heading_left_rad" => -slope.atan(),
        "center_offset_left_m" => -(right.intercept + left.intercept) / 2. * cos,
        "width_m" => (right.intercept - left.intercept) * cos,
        "left_wall_points" => left.points as i64, "right_wall_points" => right.points as i64,
        "source_kind" => "race_wide_start_lane",
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A car in a lane with an end wall `rear` behind it and a far wall ahead.
    fn scan(half_width: f64, offset: f64, rear: f64) -> Vec<V> {
        (0..360usize)
            .map(|a| {
                let (c, s) = (dcos(a), dsin(a));
                let mut best = f64::INFINITY;
                if s > 1e-9 {
                    best = best.min((half_width - offset) / s);
                }
                if s < -1e-9 {
                    best = best.min((half_width + offset) / -s);
                }
                if c < -1e-9 {
                    best = best.min(rear / -c);
                }
                if c > 1e-9 {
                    best = best.min(6. / c);
                }
                V::Float(best)
            })
            .collect()
    }

    #[test]
    fn a_wide_lane_is_measured_through_its_side_walls() {
        let lane = wide_start_lane(&scan(0.95, 0.05, 0.3)).expect("lane");
        assert!((lane.idx("width_m").f() - 1.9).abs() < 1e-6);
        // Offset 0.05 toward the right wall puts the centre 0.05 on the car's left.
        assert!((lane.idx("center_offset_left_m").f() - 0.05).abs() < 1e-6);
        assert!(lane.idx("heading_left_rad").f().abs() < 1e-6);
    }

    #[test]
    fn a_missing_side_wall_is_not_a_lane() {
        let mut ranges = scan(0.95, 0., 0.3);
        for bin in RIGHT_BINS {
            ranges[bin] = V::Float(4. + bin as f64 * 0.1);
        }
        assert!(wide_start_lane(&ranges).is_none());
    }

    #[test]
    fn a_bowed_soft_board_and_skewed_start_is_admitted() {
        // Flexible taped left board bowed 4.5 cm + skewed car placement (y111-5 pattern)
        let mut ranges = scan(0.55, -0.10, 0.30);
        for (k, bin) in LEFT_BINS.enumerate() {
            if let Some(r) = ranges[bin].of() {
                let bow = 0.045 * ((k as f64 / 50.0) * PI).sin();
                ranges[bin] = V::Float((r - bow).max(0.25));
            }
        }
        let lane = wide_start_lane(&ranges).expect("bowed soft board lane admitted");
        assert!((0.85..=1.25).contains(&lane.idx("width_m").f()));
    }
}
