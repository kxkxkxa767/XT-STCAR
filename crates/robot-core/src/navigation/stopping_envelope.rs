//! A second complete stopping-set proof, used only after the original disk
//! rejects a rolling-local candidate. It is not a sampled nominal trajectory.
use super::*;
use crate::admission::StoppingEnvelope;
#[cfg(test)]
#[path = "stopping_envelope_tests.rs"]
mod tests;

/// Keep the original total travel bound D = 2*T*v + v²/(2*a). For every
/// forward path of length s <= D with arbitrary signed curvature |k| <= Kmax,
/// |yaw(s)-yaw(0)| <= Kmax*s and |lateral(s)| <= min(s,Kmax*s²/2).
/// Every body point's rotational displacement is <= min(r*Kmax*s,2*r).
/// StoppingEnvelope combines these bounds with the complete initial rectangle,
/// the original clearance, and outward floating-point padding. Kmax covers the
/// measured, commanded and historical curvature, including arbitrary changes
/// of sign during both original reaction periods and subsequent braking.
///
/// Thus this proof covers the physical motion set that the disk overbounds;
/// it does not assume steering stays constant, instantly centers, or follows
/// one planned arc. As before, braking >= configured deceleration and forward
/// motion are model assumptions requiring hardware verification. All normal
/// rollout, lattice/capsule, error, adoption and final source gates still apply.
pub(super) fn certify(
    config: &NavigationConfig,
    pose: Pose2,
    speed_bound_mps: f64,
    obstacles: &[ObstacleDisc],
    boundary: Option<HalfPlane>,
) -> Option<StoppingMargin> {
    let envelope = StoppingEnvelope::new(
        config,
        pose,
        speed_bound_mps,
        config.max_curvature_per_m,
        2.0 * config.control_period_ms as f64 / 1000.0,
    )?;
    let corners = envelope.corners().map(|point| pose.body_to_world(point));
    let mut result = StoppingMargin {
        clearance_m: f64::INFINITY,
        constraint: StoppingConstraint::MinX,
    };
    // Convex full-body rectangle, not only the reference center. Boundaries
    // keep their original contact convention; obstacle contact is rejected.
    for point in corners {
        if !point.valid() {
            return None;
        }
        for (clearance_m, constraint) in [
            (point.x_m - config.bounds.min_x_m, StoppingConstraint::MinX),
            (config.bounds.max_x_m - point.x_m, StoppingConstraint::MaxX),
            (point.y_m - config.bounds.min_y_m, StoppingConstraint::MinY),
            (config.bounds.max_y_m - point.y_m, StoppingConstraint::MaxY),
        ] {
            if clearance_m < result.clearance_m {
                result = StoppingMargin {
                    clearance_m,
                    constraint,
                };
            }
        }
    }
    if let Some(boundary) = boundary {
        let clearance_m = boundary.signed_points_margin(&corners, 0.0);
        if clearance_m < result.clearance_m {
            result = StoppingMargin {
                clearance_m,
                constraint: StoppingConstraint::TravelBoundary,
            };
        }
    }
    for (index, obstacle) in obstacles.iter().enumerate() {
        if !obstacle.center.valid() || !obstacle.radius_m.is_finite() || obstacle.radius_m < 0.0 {
            return None;
        }
        let clearance_m =
            envelope.signed_disc_margin(pose.world_to_body(obstacle.center), obstacle.radius_m);
        if clearance_m <= result.clearance_m {
            result = StoppingMargin {
                clearance_m,
                constraint: StoppingConstraint::Obstacle { index },
            };
        }
    }
    result.clear().then_some(result)
}
