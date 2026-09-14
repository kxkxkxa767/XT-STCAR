//! Bounded quadrature for a rate-limited model segment, from one numerical pose.
//!
//! Saturation times split speed and steering into at most three affine pieces.
//! Heading and distance are analytic on each piece. Each affine piece uses four
//! midpoint subintervals and carries their vector remainders, rather than
//! treating numerical endpoints as exact. This fixed amount of work reduces
//! the quadratic global error bound without relying on sample convergence.
//! These are model bounds, not actuator measurements. A chain must carry its
//! preceding pose uncertainty with `ErrorBound::advance`; resetting that bound
//! declares a new numerical initial state, not a directly executable whole path.

use crate::autonomy::Pose2;
use crate::motion_transition::{MotionProjection, MotionTransition, lateral_acceleration_peak};

#[derive(Clone, Copy, Debug)]
pub(super) struct BoundedProjection {
    pub projection: MotionProjection,
    pub position_error_m: f64,
    pub heading_error_rad: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub(super) struct ErrorBound {
    pub position_m: f64,
    pub heading_rad: f64,
}

impl ErrorBound {
    /// Rotation of an otherwise identical displacement by delta changes it by
    /// at most distance * min(|delta|, 2). Position/yaw errors never cancel.
    pub fn advance(self, next: &BoundedProjection) -> Option<Self> {
        if !nonnegative(self.position_m)
            || !nonnegative(self.heading_rad)
            || !nonnegative(next.position_error_m)
            || !nonnegative(next.heading_error_rad)
            || !nonnegative(next.projection.distance_m)
        {
            return None;
        }
        let bound = Self {
            position_m: (self.position_m
                + next.position_error_m
                + next.projection.distance_m * self.heading_rad.min(2.0))
            .next_up(),
            heading_rad: (self.heading_rad + next.heading_error_rad).next_up(),
        };
        (nonnegative(bound.position_m) && nonnegative(bound.heading_rad)).then_some(bound)
    }
}

/// Integrate a single model interval without assuming that a target is reached
/// at its endpoint. Work is fixed (at most twelve subintervals), independent of dt.
/// Returned errors are relative to the supplied numerical pose and motion state.
pub(super) fn predict(pose: Pose2, motion: MotionTransition, dt: f64) -> Option<BoundedProjection> {
    // Reuse the common finite/nonnegative/rate validation without imposing a
    // sampled-position work budget. Long intervals remain fixed-work here;
    // unrepresentable results or error bounds still fail closed.
    lateral_acceleration_peak(motion, dt)?;
    if !pose.valid()
        || [pose.x_m, pose.y_m, pose.yaw_rad]
            .into_iter()
            .any(|value| value.abs() > 1_000_000.0)
        || motion.initial_speed_mps.max(motion.target_speed_mps) > 3.0
        || motion
            .initial_curvature_per_m
            .abs()
            .max(motion.target_curvature_per_m.abs())
            > 10.0
        || motion.max_accel_mps2 > 10.0
        || motion.max_decel_mps2 > 10.0
        || motion.max_curvature_rate_per_s > 50.0
    {
        return None;
    }
    let acceleration = if motion.target_speed_mps >= motion.initial_speed_mps {
        motion.max_accel_mps2
    } else {
        motion.max_decel_mps2
    };
    let speed = Ramp::new(
        motion.initial_speed_mps,
        motion.target_speed_mps,
        acceleration,
    )?;
    let curvature = Ramp::new(
        motion.initial_curvature_per_m,
        motion.target_curvature_per_m,
        motion.max_curvature_rate_per_s,
    )?;
    let mut cuts = [0.0, speed.arrival.min(dt), curvature.arrival.min(dt), dt];
    cuts.sort_unstable_by(f64::total_cmp);
    let mut projected = MotionProjection {
        pose,
        speed_mps: motion.initial_speed_mps,
        curvature_per_m: motion.initial_curvature_per_m,
        distance_m: 0.0,
    };
    let mut error = ErrorBound::default();
    for affine in cuts.windows(2) {
        if affine[0] == affine[1] {
            continue;
        }
        for subinterval in 0..4 {
            let start = affine[0] + (affine[1] - affine[0]) * (subinterval as f64 / 4.0);
            let end = if subinterval == 3 {
                affine[1]
            } else {
                affine[0] + (affine[1] - affine[0]) * ((subinterval + 1) as f64 / 4.0)
            };
            let h = end - start;
            if h == 0.0 {
                continue;
            }
            let v = speed.at(start);
            let k = curvature.at(start);
            let a = speed.derivative(start);
            let r = curvature.derivative(start);
            let yaw_delta =
                |t: f64| v * k * t + (v * r + a * k) * t * t * 0.5 + a * r * t * t * t / 3.0;
            let dyaw = yaw_delta(h);
            let middle_yaw = projected.pose.yaw_rad + yaw_delta(h * 0.5);
            let travel = v * h + a * h * h * 0.5;
            let middle_speed = v + a * h * 0.5;
            let dx = middle_speed * middle_yaw.cos() * h;
            let dy = middle_speed * middle_yaw.sin() * h;

            let maximum_speed = v.max(speed.at(end));
            let maximum_curvature = k.abs().max(curvature.at(end).abs());
            // For f(t)=v(t)e^{i theta(t)}, theta'=v*k and theta''=a*k+v*r:
            // ||f''|| <= 3|a|VK + V²|r| + V³K². The vector midpoint remainder
            // is bounded by sup ||f''|| * h³/24, including changing speed/steering.
            let second_derivative_bound = 3.0 * a.abs() * maximum_speed * maximum_curvature
                + maximum_speed.powi(2) * r.abs()
                + maximum_speed.powi(3) * maximum_curvature.powi(2);
            let quadrature_error = second_derivative_bound * h.powi(3) / 24.0;
            // Explicit floating arithmetic allowance, separate from the analytic
            // quadrature bound; coordinate/angle scale is restricted above.
            let angle_scale = 1.0
                + projected.pose.yaw_rad.abs()
                + (v * k * h).abs()
                + ((v * r + a * k) * h * h * 0.5).abs()
                + (a * r * h * h * h / 3.0).abs();
            let heading_roundoff = 128.0 * f64::EPSILON * angle_scale;
            let position_roundoff = 128.0
                * f64::EPSILON
                * (1.0 + projected.pose.x_m.abs() + projected.pose.y_m.abs() + travel.abs())
                + travel.abs() * heading_roundoff;
            if !nonnegative(travel)
                || !nonnegative(quadrature_error)
                || !nonnegative(heading_roundoff)
                || !nonnegative(position_roundoff)
                || !dx.is_finite()
                || !dy.is_finite()
                || !dyaw.is_finite()
            {
                return None;
            }
            let local = BoundedProjection {
                projection: MotionProjection {
                    pose: projected.pose,
                    speed_mps: speed.at(end),
                    curvature_per_m: curvature.at(end),
                    distance_m: travel,
                },
                position_error_m: (quadrature_error + position_roundoff).next_up(),
                heading_error_rad: heading_roundoff.next_up(),
            };
            error = error.advance(&local)?;
            projected.pose.x_m += dx;
            projected.pose.y_m += dy;
            projected.pose.yaw_rad += dyaw;
            projected.distance_m += travel;
            projected.speed_mps = local.projection.speed_mps;
            projected.curvature_per_m = local.projection.curvature_per_m;
        }
    }
    if !projected.pose.valid()
        || !nonnegative(projected.distance_m)
        || !nonnegative(projected.speed_mps)
        || !projected.curvature_per_m.is_finite()
    {
        return None;
    }
    Some(BoundedProjection {
        projection: projected,
        position_error_m: error.position_m,
        heading_error_rad: error.heading_rad,
    })
}

struct Ramp {
    initial: f64,
    target: f64,
    rate: f64,
    arrival: f64,
}

impl Ramp {
    fn new(initial: f64, target: f64, maximum_rate: f64) -> Option<Self> {
        let difference = target - initial;
        difference.is_finite().then_some(Self {
            initial,
            target,
            rate: difference.signum() * maximum_rate,
            arrival: difference.abs() / maximum_rate,
        })
    }

    fn at(&self, time: f64) -> f64 {
        if time == 0.0 {
            self.initial
        } else if time >= self.arrival {
            self.target
        } else {
            self.initial + self.rate * time
        }
    }

    fn derivative(&self, time: f64) -> f64 {
        if time >= self.arrival { 0.0 } else { self.rate }
    }
}

fn nonnegative(value: f64) -> bool {
    value.is_finite() && value >= 0.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::autonomy::Point2;
    use crate::navigation::integrate;

    fn motion(initial_k: f64, target_k: f64) -> MotionTransition {
        MotionTransition {
            initial_speed_mps: 0.3,
            target_speed_mps: 0.3,
            initial_curvature_per_m: initial_k,
            target_curvature_per_m: target_k,
            max_accel_mps2: 0.4,
            max_decel_mps2: 0.6,
            max_curvature_rate_per_s: 4.0,
        }
    }

    fn segment_distance(point: Point2, from: Pose2, to: Pose2) -> f64 {
        let dx = to.x_m - from.x_m;
        let dy = to.y_m - from.y_m;
        let fraction = (((point.x_m - from.x_m) * dx + (point.y_m - from.y_m) * dy)
            / (dx * dx + dy * dy))
            .clamp(0.0, 1.0);
        point.distance(Point2 {
            x_m: from.x_m + fraction * dx,
            y_m: from.y_m + fraction * dy,
        })
    }

    #[test]
    fn saturation_counterexample_and_mirror_need_the_bounded_ramped_endpoint() {
        // Independent 80,000 midpoint integral with analytic source-time yaw;
        // doubling 40,000 samples changed xy by 3.13e-13 m. This is a local
        // mathematical regression, not an observed vehicle/collision claim.
        let expected = Pose2 {
            x_m: 0.39989631341910775,
            y_m: 0.007849910327996932,
            yaw_rad: 0.03962500000000001,
        };
        let obstacle = Point2 {
            x_m: 0.6607748862403154,
            y_m: 0.17629431073210472,
        };
        let body_radius = 0.22_f64.hypot(0.13);
        let radius = body_radius + 0.04 + 0.015;
        let ds = 0.025;
        let sagitta = 2.0 * ds * ds / 8.0;
        for sign in [-1.0, 1.0] {
            let truth = Pose2 {
                y_m: sign * expected.y_m,
                yaw_rad: sign * expected.yaw_rad,
                ..expected
            };
            let inserted = Point2 {
                y_m: sign * obstacle.y_m,
                ..obstacle
            };
            let mut old_pose = Pose2::default();
            let mut old_curvature: f64 = 0.0;
            let mut pose = Pose2::default();
            let mut curvature = 0.0;
            let mut bound = ErrorBound::default();
            let mut old_margin = f64::INFINITY;
            let mut corrected_margin = f64::INFINITY;
            for _ in 0..16 {
                let next_k = (sign * 0.1).clamp(
                    old_curvature - 4.0 / 0.3 * ds,
                    old_curvature + 4.0 / 0.3 * ds,
                );
                let old_end = integrate(old_pose, ds, (old_curvature + next_k) * 0.5);
                old_margin = old_margin
                    .min(segment_distance(inserted, old_pose, old_end) - radius - sagitta);
                old_pose = old_end;
                old_curvature = next_k;

                let next = predict(pose, motion(curvature, sign * 0.1), ds / 0.3).unwrap();
                bound = bound.advance(&next).unwrap();
                corrected_margin = corrected_margin.min(
                    segment_distance(inserted, pose, next.projection.pose)
                        - radius
                        - sagitta
                        - bound.position_m,
                );
                pose = next.projection.pose;
                curvature = next.projection.curvature_per_m;
            }
            assert!(
                old_margin > 1.5e-5,
                "old capsules incorrectly clear the inserted obstacle"
            );
            assert!(
                corrected_margin < 0.0,
                "bounded ramp rejects it without weakening clearance"
            );
            assert!(pose.point().distance(truth.point()) <= bound.position_m);
            assert!((pose.yaw_rad - truth.yaw_rad).abs() <= bound.heading_rad);
            assert!((old_pose.yaw_rad - truth.yaw_rad).abs() > 0.0008);
            assert!(bound.position_m < 1e-6);
            let local = truth.world_to_body(inserted);
            let true_margin = (local.x_m - 0.22).hypot(local.y_m.abs() - 0.13) - 0.04 - 0.015;
            assert!(true_margin < -4.99e-6);
        }
    }

    #[test]
    fn coupled_speed_and_steering_split_integrals_cover_independent_fine_model() {
        let transition = MotionTransition {
            initial_speed_mps: 0.05,
            target_speed_mps: 0.2,
            initial_curvature_per_m: -0.6,
            target_curvature_per_m: 1.1,
            ..motion(0.0, 0.0)
        };
        let duration = 0.6;
        let mut pose = Pose2::default();
        let mut error = ErrorBound::default();
        let mut speed = transition.initial_speed_mps;
        let mut curvature = transition.initial_curvature_per_m;
        let mut distance = 0.0;
        for _ in 0..6 {
            let next = predict(
                pose,
                MotionTransition {
                    initial_speed_mps: speed,
                    initial_curvature_per_m: curvature,
                    ..transition
                },
                0.1,
            )
            .unwrap();
            error = error.advance(&next).unwrap();
            pose = next.projection.pose;
            speed = next.projection.speed_mps;
            curvature = next.projection.curvature_per_m;
            distance += next.projection.distance_m;
        }
        // Independent time-parametric midpoint dynamics, never predict or its
        // piece segmentation. Fine integration error is much smaller than the
        // tested quadrature tube; compare it at two resolutions below.
        let reference = |count: usize| {
            let h = duration / count as f64;
            let mut truth = Pose2::default();
            for i in 0..count {
                let t = (i as f64 + 0.5) * h;
                let v = (0.05 + 0.4 * t).min(0.2);
                let k = (-0.6 + 4.0 * t).min(1.1);
                let yaw = truth.yaw_rad + v * k * h * 0.5;
                truth.x_m += v * yaw.cos() * h;
                truth.y_m += v * yaw.sin() * h;
                truth.yaw_rad += v * k * h;
            }
            truth
        };
        let truth = reference(120_000);
        assert!(truth.point().distance(reference(60_000).point()) < 1e-10);
        assert!(pose.point().distance(truth.point()) <= error.position_m);
        // Independent fine theta still has O(h²) error. The analytic reference
        // is checked separately so this allowance is not part of the bound.
        // [0, .375]: -.03t - .02t² + (1.6/3)t³;
        // [.375, .425]: -.12t + .4t²; then .22 rad/s.
        let exact_yaw = 0.0625625;
        assert!((pose.yaw_rad - exact_yaw).abs() <= error.heading_rad);
        assert!((pose.yaw_rad - truth.yaw_rad).abs() < 1e-10);
        assert!((distance - 0.091875).abs() < 1e-12);
        assert_eq!(speed, 0.2);
        assert_eq!(curvature, 1.1);
        assert!(error.position_m < 0.0001);
    }

    #[test]
    fn uncertainty_is_composed_and_invalid_numeric_bounds_fail_closed() {
        let next = predict(Pose2::default(), motion(0.0, 0.0), 0.1).unwrap();
        let before = ErrorBound {
            position_m: 0.01,
            heading_rad: 0.02,
        };
        let after = before.advance(&next).unwrap();
        assert!(after.position_m >= 0.01 + 0.03 * 0.02);
        assert!(after.heading_rad >= before.heading_rad);
        assert!(
            ErrorBound {
                position_m: f64::NAN,
                ..before
            }
            .advance(&next)
            .is_none()
        );
        assert!(predict(Pose2::default(), motion(0.0, f64::NAN), 0.1).is_none());
        assert!(predict(Pose2::default(), motion(0.0, 0.0), -0.1).is_none());
        assert!(predict(Pose2::default(), motion(0.0, 0.0), f64::INFINITY).is_none());
        let stopped = MotionTransition {
            initial_speed_mps: 0.0,
            target_speed_mps: 0.0,
            ..motion(-1.0, 1.0)
        };
        let stationary = predict(Pose2::default(), stopped, 0.6).unwrap();
        assert_eq!(stationary.projection.pose, Pose2::default());
        assert_eq!(stationary.projection.distance_m, 0.0);
        assert_eq!(stationary.projection.curvature_per_m, 1.0);
    }
}
