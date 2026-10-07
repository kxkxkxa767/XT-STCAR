//! Finite observed walls and left-opening alignment geometry, no turn permit.
use super::corridor::{WallSupport, supported_wall};
use super::{LeftJunction, detect_left_junction, wall};
use crate::autonomy::Point2;
use crate::scan::{ScanConfig, validate_full_scan};
use crate::{FrameId, LidarSample, ValidationError};
use serde::Serialize;
use std::f64::consts::{PI, TAU};

#[derive(Clone, Debug, Serialize)]
pub struct WallCandidate {
    pub heading_left_rad: f64,
    /// Left normal at this heading dot a wall point = rho. Not unsigned range.
    pub rho_left_m: f64,
    /// Actual measured endpoints, not an extrapolated infinite wall.
    pub support_start_left_m: Point2,
    pub support_end_left_m: Point2,
    pub support_span_m: f64,
    pub points: usize,
    pub fit_error_m: f64,
    pub candidate_only: bool,
}

#[derive(Clone, Debug, Serialize)]
pub struct TargetSupportRay {
    pub index: usize,
    pub angle_left_rad: f64,
    pub range_m: f64,
    pub target_range_m: f64,
}

#[derive(Clone, Debug, Serialize)]
pub struct IncomingLeftEndSupport {
    pub index: usize,
    pub angle_left_rad: f64,
    pub range_m: f64,
    /// Original same-scan return, not an interpolated or certified board end.
    pub point_left_m: Point2,
}

#[derive(Clone, Debug, Serialize)]
pub struct LeftTurnGoal {
    pub heading_left_rad: f64,
    /// Candidate exit-centre line rho under this heading's LEFT normal.
    pub center_offset_left_m: f64,
    pub width_m: f64,
    pub target_point_left_m: Point2,
    pub target_support_ray: TargetSupportRay,
    pub outer_wall: WallCandidate,
    pub incoming_heading_left_rad: f64,
    pub front_wall_m: f64,
    pub incoming_left_end_m: f64,
    /// Binds the candidate extent to a real return in this publication.
    pub incoming_left_end_support: IncomingLeftEndSupport,
    pub known_open_fraction: f64,
    pub origin_between_exit_walls: bool,
    pub candidate_only: bool,
    pub turn_path_certified: bool,
    pub observation_type: &'static str,
}

#[derive(Default)]
pub struct TurnGeometry {
    pub walls: Vec<WallCandidate>,
    pub left_goal: Option<LeftTurnGoal>,
}

fn wrap(angle: f64) -> f64 {
    (angle + PI).rem_euclid(TAU) - PI
}

fn wall_candidate(support: &WallSupport, hint: f64, reverse: bool) -> WallCandidate {
    let heading = wrap(hint + support.slope.atan());
    let (s, c) = hint.sin_cos();
    let (ts, tc) = heading.sin_cos();
    let points: Vec<_> = support
        .points
        .iter()
        .map(|&(x, y)| Point2 {
            x_m: c * x - s * y,
            y_m: s * x + c * y,
        })
        .collect();
    let along = |point: &Point2| tc * point.x_m + ts * point.y_m;
    let start = *points
        .iter()
        .min_by(|a, b| along(a).total_cmp(&along(b)))
        .unwrap();
    let end = *points
        .iter()
        .max_by(|a, b| along(a).total_cmp(&along(b)))
        .unwrap();
    let rho = support.intercept / support.slope.hypot(1.0);
    WallCandidate {
        heading_left_rad: wrap(heading + if reverse { PI } else { 0.0 }),
        rho_left_m: if reverse { -rho } else { rho },
        support_start_left_m: if reverse { end } else { start },
        support_end_left_m: if reverse { start } else { end },
        support_span_m: along(&end) - along(&start),
        points: points.len(),
        fit_error_m: support.error_m,
        candidate_only: true,
    }
}

fn add_wall(walls: &mut Vec<WallCandidate>, candidate: WallCandidate) {
    if candidate.support_span_m < 0.35 {
        return;
    }
    if let Some(old) = walls.iter_mut().find(|old| {
        wrap(old.heading_left_rad - candidate.heading_left_rad).abs() <= 2.0_f64.to_radians()
            && (old.rho_left_m - candidate.rho_left_m).abs() <= 0.05
    }) {
        if candidate.support_span_m > old.support_span_m {
            *old = candidate;
        }
    } else {
        walls.push(candidate);
    }
}

fn observed_walls(rays: &[(usize, f64, f64, Point2)]) -> Vec<WallCandidate> {
    let mut walls = Vec::new();
    for seed in 0..12 {
        let hint = seed as f64 * PI / 12.0;
        let (s, c) = hint.sin_cos();
        for (lo, hi) in [(-135.0, -45.0), (45.0, 135.0)] {
            let mut sector: Vec<_> = rays
                .iter()
                .filter(|(_, angle, _, _)| (lo..=hi).contains(&wrap(*angle - hint).to_degrees()))
                .collect();
            sector.sort_by(|a, b| wrap(a.1 - hint).total_cmp(&wrap(b.1 - hint)));
            let points: Vec<_> = sector
                .iter()
                .map(|(_, _, _, p)| (c * p.x_m + s * p.y_m, -s * p.x_m + c * p.y_m))
                .collect();
            let mut fits = Vec::new();
            if let Some(fit) = supported_wall(&points) {
                fits.push(fit);
            }
            // Overlapping bounded windows retain a visible wall even when a
            // corner/branch makes the complete side sector contain two planes.
            for start in (0..points.len().saturating_sub(23)).step_by(12) {
                if let Some(fit) = supported_wall(&points[start..start + 24]) {
                    fits.push(fit);
                }
            }
            for fit in fits {
                for reverse in [false, true] {
                    add_wall(&mut walls, wall_candidate(&fit, hint, reverse));
                }
            }
        }
    }
    walls
}

fn left_goal(rays: &[(usize, f64, f64, Point2)], junction: LeftJunction) -> Option<LeftTurnGoal> {
    let incoming = junction.heading_left_rad;
    let (s, c) = incoming.sin_cos();
    let local: Vec<_> = rays
        .iter()
        .map(|&(index, angle, range, p)| {
            (
                index,
                angle,
                range,
                c * p.x_m + s * p.y_m,
                -s * p.x_m + c * p.y_m,
                p,
            )
        })
        .collect();
    let incoming_left: Vec<_> = local
        .iter()
        .filter(|(_, angle, _, _, _, _)| (90.0..=135.0).contains(&wrap(*angle).to_degrees()))
        .map(|(_, _, _, x, y, _)| (*x, *y))
        .collect();
    let (left_slope, left_intercept) = wall(&incoming_left)?;
    // The extent is the detector's most-forward observed incoming-wall point,
    // not an inferred physical endpoint. Bind it to the original return; if
    // no current point supports this extent, there is no alignment candidate.
    let &(index, angle, range, _, _, point) = local.iter().find(|(_, _, _, x, y, _)| {
        (-1.0..=1.0).contains(x)
            && (*x - junction.incoming_left_end_m).abs() <= 1e-6
            && (*y - left_slope * *x - left_intercept).abs() <= 0.05
    })?;
    let incoming_left_end_support = IncomingLeftEndSupport {
        index,
        angle_left_rad: angle,
        range_m: range,
        point_left_m: point,
    };
    // Only actual transverse-wall returns beyond the observed left opening.
    let outer_points: Vec<_> = local
        .iter()
        .filter(|(_, _, _, x, y, _)| {
            (*x - junction.front_wall_m).abs() <= 0.12
                && *y > left_slope * *x + left_intercept + 0.10
        })
        .map(|(_, _, _, x, y, _)| (*y, *x))
        .collect();
    let outer = supported_wall(&outer_points)?;
    // Swapping y/x fits x=a*y+b. Choose the fitted tangent toward the observed
    // LEFT branch, rather than declaring an incoming+90-degree target.
    let exit_heading = wrap(incoming + 1.0_f64.atan2(outer.slope));
    let (es, ec) = exit_heading.sin_cos();
    let normal = Point2 { x_m: -es, y_m: ec };
    let rho = -outer.intercept / outer.slope.hypot(1.0);
    let measured: Vec<_> = outer
        .points
        .iter()
        .map(|&(y, x)| Point2 {
            x_m: c * x - s * y,
            y_m: s * x + c * y,
        })
        .collect();
    let along = |p: &Point2| ec * p.x_m + es * p.y_m;
    let start = *measured
        .iter()
        .min_by(|a, b| along(a).total_cmp(&along(b)))?;
    let end = *measured
        .iter()
        .max_by(|a, b| along(a).total_cmp(&along(b)))?;
    let outer_wall = WallCandidate {
        heading_left_rad: exit_heading,
        rho_left_m: rho,
        support_start_left_m: start,
        support_end_left_m: end,
        support_span_m: along(&end) - along(&start),
        points: measured.len(),
        fit_error_m: outer.error_m,
        candidate_only: true,
    };
    if outer_wall.support_span_m < 0.35 {
        return None;
    }
    let centre = rho + junction.outgoing_width_m * 0.5;
    let middle = (along(&start) + along(&end)) * 0.5;
    let mut targets = Vec::new();
    for &(index, angle, range, x, y, _) in &local {
        let relative = wrap(angle - incoming);
        if !(25.0..=85.0).contains(&relative.to_degrees()) {
            continue;
        }
        let crossing = left_intercept / (relative.tan() - left_slope);
        if !(crossing > junction.incoming_left_end_m + 0.10
            && crossing < junction.front_wall_m - 0.20
            && y > left_slope * x + left_intercept + 0.10
            && x > junction.incoming_left_end_m + 0.05
            && x <= junction.front_wall_m + 0.15)
        {
            continue;
        }
        let denominator = normal.x_m * angle.cos() + normal.y_m * angle.sin();
        if denominator.abs() < 1e-6 {
            continue;
        }
        let distance = centre / denominator;
        if !(distance > 0.0 && distance + 0.03 < range) {
            continue;
        }
        let target = Point2 {
            x_m: distance * angle.cos(),
            y_m: distance * angle.sin(),
        };
        let target_along = along(&target);
        if target_along < along(&start) || target_along > along(&end) {
            continue;
        }
        targets.push((
            (target_along - middle).abs(),
            target,
            TargetSupportRay {
                index,
                angle_left_rad: angle,
                range_m: range,
                target_range_m: distance,
            },
        ));
    }
    targets.sort_by(|a, b| a.0.total_cmp(&b.0));
    let (_, target, target_support_ray) = targets.into_iter().next()?;
    Some(LeftTurnGoal {
        heading_left_rad: exit_heading,
        center_offset_left_m: centre,
        width_m: junction.outgoing_width_m,
        target_point_left_m: target,
        target_support_ray,
        outer_wall,
        incoming_heading_left_rad: incoming,
        front_wall_m: junction.front_wall_m,
        incoming_left_end_m: junction.incoming_left_end_m,
        incoming_left_end_support,
        known_open_fraction: junction.known_open_fraction,
        origin_between_exit_walls: rho <= 0.0 && 0.0 <= rho + junction.outgoing_width_m,
        candidate_only: true,
        turn_path_certified: false,
        observation_type: "left_opening_outer_wall_alignment",
    })
}

/// An alignment target supported by current opening/wall/ray observations.
/// The origin may still be outside the exit strip. The finite observed wall is
/// not extended to an invisible back wall; ray support is not a swept-path proof.
pub fn detect_turn_geometry(
    scan: &LidarSample,
    frame: &FrameId,
) -> Result<TurnGeometry, ValidationError> {
    validate_full_scan(
        scan,
        &ScanConfig {
            frame_id: frame.clone(),
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
            range.map(|r| {
                let a = wrap(scan.angle_min_rad + i as f64 * scan.angle_increment_rad);
                (
                    i,
                    a,
                    r,
                    Point2 {
                        x_m: r * a.cos(),
                        y_m: r * a.sin(),
                    },
                )
            })
        })
        .collect();
    let mut walls = observed_walls(&rays);
    let goal = detect_left_junction(scan, frame)?.and_then(|junction| left_goal(&rays, junction));
    if let Some(goal) = &goal {
        add_wall(&mut walls, goal.outer_wall.clone());
        let mut reverse = goal.outer_wall.clone();
        reverse.heading_left_rad = wrap(reverse.heading_left_rad + PI);
        reverse.rho_left_m = -reverse.rho_left_m;
        std::mem::swap(
            &mut reverse.support_start_left_m,
            &mut reverse.support_end_left_m,
        );
        add_wall(&mut walls, reverse);
    }
    if walls.len() > 32 {
        return Ok(TurnGeometry::default());
    }
    walls.sort_by(|a, b| {
        a.heading_left_rad
            .total_cmp(&b.heading_left_rad)
            .then(a.rho_left_m.total_cmp(&b.rho_left_m))
    });
    Ok(TurnGeometry {
        walls,
        left_goal: goal,
    })
}
