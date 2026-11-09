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
const MIN_POINTS: usize = 30;
const INLIER_M: f64 = 0.03;
const MIN_INLIER_SHARE: f64 = 0.85;
const MIN_SLOPE_PAIR_DX_M: f64 = 0.3;
const MAX_WALL_ANGLE_DIFF_RAD: f64 = 6. * PI / 180.;

struct Wall {
    slope: f64,
    intercept: f64,
    points: usize,
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
}
