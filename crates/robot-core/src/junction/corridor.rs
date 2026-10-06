//! Local double-wall orientation candidates, never turn or free-space permits.
use super::wall;
use crate::scan::{ScanConfig, validate_full_scan};
use crate::{FrameId, LidarSample, ValidationError};
use serde::Serialize;
use std::f64::consts::{PI, TAU};

#[derive(Clone, Debug, Serialize)]
pub struct CorridorCandidate {
    /// Heading in the caller's X-forward/Y-left frame, counterclockwise positive.
    pub heading_left_rad: f64,
    /// Centre line's signed distance to the lidar origin along this heading's
    /// LEFT normal. Reversing the heading also reverses this sign.
    pub center_offset_left_m: f64,
    pub width_m: f64,
    pub left_wall_points: usize,
    pub right_wall_points: usize,
    /// Actual projected support shared by both extended wall observations.
    pub support_span_m: f64,
    pub fit_error_m: f64,
    pub origin_between_walls: bool,
    pub candidate_only: bool,
    pub turn_path_certified: bool,
}

fn wrap(angle: f64) -> f64 {
    (angle + PI).rem_euclid(TAU) - PI
}

pub(super) struct WallSupport {
    pub(super) slope: f64,
    pub(super) intercept: f64,
    pub(super) points: Vec<(f64, f64)>,
    pub(super) error_m: f64,
}

pub(super) fn supported_wall(points: &[(f64, f64)]) -> Option<WallSupport> {
    let (slope, intercept) = wall(points)?;
    let inliers: Vec<_> = points
        .iter()
        .copied()
        .filter(|&(x, y)| (y - slope * x - intercept).abs() <= 0.04)
        .collect();
    if inliers.len() < 16 || (inliers.len() as f64) < 0.8 * points.len() as f64 {
        return None;
    }
    let error = inliers
        .iter()
        .map(|&(x, y)| (y - slope * x - intercept).abs() / slope.hypot(1.0))
        .fold(0.0, f64::max);
    Some(WallSupport {
        slope,
        intercept,
        points: inliers,
        error_m: error,
    })
}

/// Fit the same real extended-wall evidence in bounded rotated frames. The
/// twelve 15-degree seeds cover every undirected axis; fitted headings are not
/// snapped to those seeds. Both directional alternatives are reported. This
/// only describes a corridor THROUGH the current origin, not an unseen outgoing
/// branch: if the origin has not entered that branch, its geometry stays unknown.
/// Missing returns remain missing, and no candidate certifies a turn, the whole
/// swept vehicle, or stopping space. The scan timestamp is whatever the source
/// supplied; this function does not manufacture per-ray acquisition times.
pub fn detect_corridor_candidates(
    scan: &LidarSample,
    heading_frame: &FrameId,
) -> Result<Vec<CorridorCandidate>, ValidationError> {
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
        .filter_map(|(i, range)| {
            let angle = scan.angle_min_rad + i as f64 * scan.angle_increment_rad;
            range.map(|r| (angle, r * angle.cos(), r * angle.sin()))
        })
        .collect();
    let mut candidates: Vec<CorridorCandidate> = Vec::new();
    for seed in 0..12 {
        let hint = seed as f64 * PI / 12.0;
        let (s, c) = hint.sin_cos();
        let sector = |lo: f64, hi: f64| {
            rays.iter()
                .filter(|(angle, _, _)| (lo..=hi).contains(&wrap(*angle - hint).to_degrees()))
                .map(|&(_, x, y)| (c * x + s * y, -s * x + c * y))
                .collect::<Vec<_>>()
        };
        let Some(left) = supported_wall(&sector(45.0, 135.0)) else {
            continue;
        };
        let Some(right) = supported_wall(&sector(-135.0, -45.0)) else {
            continue;
        };
        if left.intercept <= 0.0
            || right.intercept >= 0.0
            // Match the incoming-wall tolerance used by the junction/front
            // detector: portable boards can taper without invalidating the
            // independently supported local orientation observation.
            || (left.slope.atan() - right.slope.atan()).abs() > 12.0_f64.to_radians()
        {
            continue;
        }
        let correction = ((left.slope + right.slope) * 0.5).atan();
        let (fs, fc) = correction.sin_cos();
        let left_rho = left.intercept / (fc + left.slope * fs);
        let right_rho = right.intercept / (fc + right.slope * fs);
        let width = left_rho - right_rho;
        if !(0.65..=2.5).contains(&width) {
            continue;
        }
        let support = |points: &[(f64, f64)]| {
            let along = points.iter().map(|&(x, y)| fc * x + fs * y);
            let low = along.clone().fold(f64::INFINITY, f64::min);
            let high = along.fold(f64::NEG_INFINITY, f64::max);
            (low, high)
        };
        let (left_low, left_high) = support(&left.points);
        let (right_low, right_high) = support(&right.points);
        let span = left_high.min(right_high) - left_low.max(right_low);
        if span < 0.35 {
            continue;
        }
        let heading = wrap(hint + correction);
        let offset = (left_rho + right_rho) * 0.5;
        for reverse in [false, true] {
            let candidate = CorridorCandidate {
                heading_left_rad: wrap(heading + if reverse { PI } else { 0.0 }),
                center_offset_left_m: if reverse { -offset } else { offset },
                width_m: width,
                left_wall_points: if reverse {
                    right.points.len()
                } else {
                    left.points.len()
                },
                right_wall_points: if reverse {
                    left.points.len()
                } else {
                    right.points.len()
                },
                support_span_m: span,
                fit_error_m: left.error_m.max(right.error_m),
                origin_between_walls: true,
                candidate_only: true,
                turn_path_certified: false,
            };
            if let Some(old) = candidates.iter_mut().find(|old| {
                wrap(old.heading_left_rad - candidate.heading_left_rad).abs()
                    <= 2.0_f64.to_radians()
                    && (old.center_offset_left_m - candidate.center_offset_left_m).abs() <= 0.05
                    && (old.width_m - candidate.width_m).abs() <= 0.1
            }) {
                if candidate.support_span_m > old.support_span_m {
                    *old = candidate;
                }
            } else {
                candidates.push(candidate);
            }
        }
    }
    // A highly inconsistent multi-axis cloud is unknown, not a convenient
    // nearest candidate. Never silently truncate competing observations.
    if candidates.len() > 8 {
        candidates.clear();
    }
    candidates.sort_by(|a, b| a.heading_left_rad.total_cmp(&b.heading_left_rad));
    Ok(candidates)
}
