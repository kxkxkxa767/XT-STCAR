//! Local wall/opening evidence, never a turn trajectory or a free-space certificate.
//! Coordinates are forward X / left Y. The caller supplies a known heading frame.
use crate::scan::{ScanConfig, validate_full_scan};
use crate::{FrameId, LidarSample, ValidationError};
use serde::{Deserialize, Serialize};
use std::f64::consts::{PI, TAU};

mod corridor;
pub use corridor::{CorridorCandidate, detect_corridor_candidates};
mod turn_goal;
pub use turn_goal::{LeftTurnGoal, TurnGeometry, WallCandidate, detect_turn_geometry};

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct LeftJunction {
    pub front_wall_m: f64,
    pub incoming_left_end_m: f64,
    pub incoming_width_m: f64,
    pub outgoing_width_m: f64,
    pub heading_left_rad: f64,
    pub known_open_fraction: f64,
    pub turn_path_certified: bool,
}

fn median(values: &mut [f64]) -> f64 {
    values.sort_by(f64::total_cmp);
    values[values.len() / 2]
}

fn wall(points: &[(f64, f64)]) -> Option<(f64, f64)> {
    if points.len() < 16 {
        return None;
    }
    let mut slopes = Vec::new();
    for (i, &(x, y)) in points.iter().enumerate() {
        for &(other_x, other_y) in points.iter().skip(i + 4).step_by(4) {
            if (other_x - x).abs() >= 0.15 {
                slopes.push((other_y - y) / (other_x - x));
            }
        }
    }
    if slopes.is_empty() {
        return None;
    }
    let slope = median(&mut slopes);
    if slope.abs() > (15.0_f64.to_radians()).tan() {
        return None;
    }
    let mut intercepts: Vec<_> = points.iter().map(|&(x, y)| y - slope * x).collect();
    let intercept = median(&mut intercepts);
    let inliers: Vec<_> = points
        .iter()
        .filter(|&&(x, y)| (y - slope * x - intercept).abs() <= 0.04)
        .collect();
    let min_x = inliers.iter().map(|p| p.0).fold(f64::INFINITY, f64::min);
    let max_x = inliers
        .iter()
        .map(|p| p.0)
        .fold(f64::NEG_INFINITY, f64::max);
    (inliers.len() as f64 >= 0.8 * points.len() as f64 && max_x - min_x >= 0.25)
        .then_some((slope, intercept))
}

/// Require a full valid scan, two extended incoming walls, a transverse front
/// boundary, retained right wall, and known returns beyond the left wall end.
/// Unknown beams, a single cone, a dead end, or a T opening are not a left turn.
pub fn detect_left_junction(
    scan: &LidarSample,
    heading_frame: &FrameId,
) -> Result<Option<LeftJunction>, ValidationError> {
    validate_full_scan(
        scan,
        &ScanConfig {
            frame_id: heading_frame.clone(),
            bins: scan.ranges_m.len(),
            min_coverage_fraction: 0.95,
            min_valid_fraction: 0.95,
            max_missing_arc_rad: 5.0_f64.to_radians(),
            max_revolution_ms: 300,
            max_packet_gap_ms: 100,
        },
    )?;
    let rays: Vec<_> = scan
        .ranges_m
        .iter()
        .enumerate()
        .map(|(i, range)| {
            let angle = (scan.angle_min_rad + i as f64 * scan.angle_increment_rad + PI)
                .rem_euclid(TAU)
                - PI;
            (angle, range.map(|r| (r * angle.cos(), r * angle.sin())))
        })
        .collect();
    let side = |lo: f64, hi: f64| -> Vec<_> {
        rays.iter()
            .filter(|(a, _)| (lo..=hi).contains(&a.to_degrees()))
            .filter_map(|(_, p)| *p)
            .collect()
    };
    let Some((right_a, right_b)) = wall(&side(-135.0, -45.0)) else {
        return Ok(None);
    };
    let Some((left_a, left_b)) = wall(&side(90.0, 135.0)) else {
        return Ok(None);
    };
    if right_b >= 0.0
        || left_b <= 0.0
        || (left_a.atan() - right_a.atan()).abs() > 12.0_f64.to_radians()
    {
        return Ok(None);
    }
    // Portable boards may converge slightly; keep each fitted wall below.
    let heading = ((left_a + right_a) / 2.0).atan();
    let (sin, cos) = heading.sin_cos();
    let aligned: Vec<_> = rays
        .iter()
        .map(|&(a, p)| {
            (
                a - heading,
                p.map(|(x, y)| (x * cos + y * sin, -x * sin + y * cos)),
            )
        })
        .collect();
    let left = left_b / (cos + left_a * sin);
    let right = right_b / (cos + right_a * sin);
    let left_slope = (left_a * cos - sin) / (cos + left_a * sin);
    let right_slope = (right_a * cos - sin) / (cos + right_a * sin);
    let width = left - right;
    if !(0.75..=2.2).contains(&width) {
        return Ok(None);
    }
    let front_rays: Vec<_> = aligned
        .iter()
        .filter(|(a, _)| a.abs() <= 15.0_f64.to_radians())
        .collect();
    let mut front_x: Vec<_> = front_rays
        .iter()
        .filter_map(|(_, p)| p.map(|(x, _)| x))
        .collect();
    if front_x.len() < 12 || (front_x.len() as f64) < 0.9 * front_rays.len() as f64 {
        return Ok(None);
    }
    let front = median(&mut front_x);
    if !(0.6..=3.0).contains(&front)
        || (front_x
            .iter()
            .filter(|x| (**x - front).abs() <= 0.08)
            .count() as f64)
            < 0.85 * front_x.len() as f64
    {
        return Ok(None);
    }
    let points: Vec<_> = aligned.iter().filter_map(|(_, p)| *p).collect();
    let end = points
        .iter()
        .filter(|&&(x, y)| (-1.0..=1.0).contains(&x) && (y - left - left_slope * x).abs() <= 0.05)
        .map(|&(x, _)| x)
        .fold(f64::NEG_INFINITY, f64::max);
    let outgoing = front - end;
    if !end.is_finite() || !(-0.3..=1.0).contains(&end) || !(0.75..=2.25).contains(&outgoing) {
        return Ok(None);
    }
    // A T-junction loses the right forward wall; retain an extended segment.
    let retained: Vec<_> = points
        .iter()
        .filter(|&&(x, y)| {
            x > end + 0.25 && x < front - 0.08 && (y - right - right_slope * x).abs() <= 0.05
        })
        .collect();
    let min_x = retained.iter().map(|p| p.0).fold(f64::INFINITY, f64::min);
    let max_x = retained
        .iter()
        .map(|p| p.0)
        .fold(f64::NEG_INFINITY, f64::max);
    if retained.len() < 8 || max_x - min_x < 0.3 {
        return Ok(None);
    }
    let opening: Vec<_> = aligned
        .iter()
        .filter(|(a, _)| {
            let crossing_x = left / (a.tan() - left_slope);
            (25.0..=85.0).contains(&a.to_degrees())
                && crossing_x > end + 0.10
                && crossing_x < front - 0.20
        })
        .collect();
    let known_open = opening
        .iter()
        .filter(|(_, p)| {
            p.is_some_and(|(x, y)| {
                y > left + left_slope * x + 0.10 && x > end + 0.05 && x <= front + 0.15
            })
        })
        .count();
    let fraction = known_open as f64 / opening.len().max(1) as f64;
    if opening.len() < 16 || fraction < 0.9 {
        return Ok(None);
    }
    Ok(Some(LeftJunction {
        front_wall_m: front,
        incoming_left_end_m: end,
        incoming_width_m: width,
        outgoing_width_m: outgoing,
        heading_left_rad: heading,
        known_open_fraction: fraction,
        turn_path_certified: false,
    }))
}

/// Transverse boundary used only to reduce approach PWM, never to claim a turn.
/// Require two extended side lines and at least 35 cm of a known interior plane.
/// Search the scan's declared valid range, rather than truncating early returns
/// to a fixed approach distance. Sparse distant returns remain unknown unless
/// they satisfy the same point-count, span and plane-consistency requirements.
pub fn detect_front_boundary(
    scan: &LidarSample,
    heading_frame: &FrameId,
) -> Result<Option<f64>, ValidationError> {
    validate_full_scan(
        scan,
        &ScanConfig {
            frame_id: heading_frame.clone(),
            bins: scan.ranges_m.len(),
            min_coverage_fraction: 0.95,
            min_valid_fraction: 0.95,
            max_missing_arc_rad: 5.0_f64.to_radians(),
            max_revolution_ms: 300,
            max_packet_gap_ms: 100,
        },
    )?;
    let rays: Vec<_> = scan
        .ranges_m
        .iter()
        .enumerate()
        .filter_map(|(i, r)| {
            let angle = (scan.angle_min_rad + i as f64 * scan.angle_increment_rad + PI)
                .rem_euclid(TAU)
                - PI;
            r.map(|r| (angle, r * angle.cos(), r * angle.sin()))
        })
        .collect();
    let sector = |lo: f64, hi: f64| {
        rays.iter()
            .filter(|(a, _, _)| (lo..=hi).contains(&a.to_degrees()))
            .map(|&(_, x, y)| (x, y))
            .collect::<Vec<_>>()
    };
    let Some((ra, rb)) = wall(&sector(-135.0, -45.0)) else {
        return Ok(None);
    };
    let Some((la, lb)) = wall(&sector(90.0, 135.0)) else {
        return Ok(None);
    };
    if rb >= 0.0
        || lb <= 0.0
        || !(0.75..=2.2).contains(&(lb - rb))
        || (la.atan() - ra.atan()).abs() > 12.0_f64.to_radians()
    {
        return Ok(None);
    }
    let heading = ((la + ra) / 2.0).atan();
    let (sin, cos) = heading.sin_cos();
    let points: Vec<_> = rays
        .iter()
        .filter(|(a, x, y)| {
            (*a - heading).abs() <= 20.0_f64.to_radians()
                && *y > ra * x + rb + 0.08
                && *y < la * x + lb - 0.08
        })
        .map(|&(_, x, y)| (x * cos + y * sin, -x * sin + y * cos))
        .filter(|(x, _)| (0.55..=scan.range_max_m).contains(x))
        .collect();
    let mut candidates = Vec::new();
    for &(anchor, _) in &points {
        let inliers: Vec<_> = points
            .iter()
            .filter(|&&(x, _)| (x - anchor).abs() <= 0.06)
            .collect();
        let low = inliers.iter().map(|p| p.1).fold(f64::INFINITY, f64::min);
        let high = inliers
            .iter()
            .map(|p| p.1)
            .fold(f64::NEG_INFINITY, f64::max);
        if inliers.len() >= 8 && high - low >= 0.35 {
            let mut xs: Vec<_> = inliers.iter().map(|p| p.0).collect();
            candidates.push(median(&mut xs));
        }
    }
    Ok(candidates
        .into_iter()
        .filter(|x| (0.6..=scan.range_max_m).contains(x))
        .min_by(f64::total_cmp))
}

#[cfg(test)]
mod tests;
