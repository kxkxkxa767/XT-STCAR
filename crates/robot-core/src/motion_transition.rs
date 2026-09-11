//! Analytic limits for a forward, rate-limited kinematic motion transition.
//! These are model predictions, not measurements of steering or braking response.

use crate::autonomy::Pose2;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotionProjection {
    pub pose: Pose2,
    pub speed_mps: f64,
    pub curvature_per_m: f64,
    pub distance_m: f64,
}

/// Project a held command through independently rate-limited speed and steering.
/// This is a model state, never a replacement for a sensor's capture timestamp.
/// Work is bounded by 2000 one-millisecond intervals plus ramp-arrival splits;
/// no allocation or I/O occurs. Inputs beyond the navigation model's numerical
/// domain, horizons over two seconds, and non-finite arithmetic are rejected.
/// Yaw and distance are integrated analytically on each affine segment; planar
/// position uses Simpson quadrature. This does not prove swept collision safety.
pub fn project_motion(
    pose: Pose2,
    motion: MotionTransition,
    horizon_s: f64,
) -> Option<MotionProjection> {
    lateral_acceleration_peak(motion, horizon_s)?;
    if !pose.valid()
        || [pose.x_m, pose.y_m, pose.yaw_rad]
            .iter()
            .any(|v| v.abs() > 1_000_000.0)
        || !(0.0..=2.0).contains(&horizon_s)
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
    let mut state = MotionProjection {
        pose,
        speed_mps: motion.initial_speed_mps,
        curvature_per_m: motion.initial_curvature_per_m,
        distance_m: 0.0,
    };
    let count = (horizon_s / 0.001).ceil() as usize;
    if count == 0 {
        return Some(state);
    }
    for _ in 0..count {
        let mut remaining = horizon_s / count as f64;
        for _ in 0..3 {
            if remaining <= 0.0 {
                break;
            }
            let a = if motion.target_speed_mps > state.speed_mps {
                motion.max_accel_mps2
            } else if motion.target_speed_mps < state.speed_mps {
                -motion.max_decel_mps2
            } else {
                0.0
            };
            let r = (motion.target_curvature_per_m - state.curvature_per_m).signum()
                * motion.max_curvature_rate_per_s;
            let r = if motion.target_curvature_per_m == state.curvature_per_m {
                0.0
            } else {
                r
            };
            let tv = if a == 0.0 {
                f64::INFINITY
            } else {
                (motion.target_speed_mps - state.speed_mps) / a
            };
            let tk = if r == 0.0 {
                f64::INFINITY
            } else {
                (motion.target_curvature_per_m - state.curvature_per_m) / r
            };
            let h = remaining.min(tv).min(tk);
            let v = state.speed_mps;
            let k = state.curvature_per_m;
            let yaw = |t: f64| {
                state.pose.yaw_rad
                    + v * k * t
                    + (v * r + a * k) * t * t * 0.5
                    + a * r * t * t * t / 3.0
            };
            let ym = yaw(h * 0.5);
            let ye = yaw(h);
            let vm = v + a * h * 0.5;
            let ve = v + a * h;
            state.pose.x_m +=
                h / 6.0 * (v * state.pose.yaw_rad.cos() + 4.0 * vm * ym.cos() + ve * ye.cos());
            state.pose.y_m +=
                h / 6.0 * (v * state.pose.yaw_rad.sin() + 4.0 * vm * ym.sin() + ve * ye.sin());
            state.pose.yaw_rad = (ye + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)
                - std::f64::consts::PI;
            state.distance_m += v * h + a * h * h * 0.5;
            state.speed_mps = if h >= tv { motion.target_speed_mps } else { ve };
            state.curvature_per_m = if h >= tk {
                motion.target_curvature_per_m
            } else {
                k + r * h
            };
            remaining -= h;
        }
        if remaining > 1e-12 {
            return None;
        }
    }
    (state.pose.valid()
        && state.distance_m.is_finite()
        && state.speed_mps.is_finite()
        && state.curvature_per_m.is_finite())
    .then_some(state)
}

/// Speed and curvature independently approach their targets at the given rates,
/// then remain constant. Rates are positive magnitudes; speed must be nonnegative.
#[derive(Clone, Copy, Debug)]
pub struct MotionTransition {
    pub initial_speed_mps: f64,
    pub target_speed_mps: f64,
    pub initial_curvature_per_m: f64,
    pub target_curvature_per_m: f64,
    pub max_accel_mps2: f64,
    pub max_decel_mps2: f64,
    pub max_curvature_rate_per_s: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LateralAccelerationPeak {
    /// Time from the start of the transition, in seconds.
    pub at_s: f64,
    /// Maximum of `speed² * abs(curvature)` over the requested time interval.
    pub lateral_accel_mps2: f64,
}

/// Find the peak on the closed interval `[0, horizon_s]`, including both ramps
/// and their constant tails. A zero horizon checks the initial state only.
///
/// Returns `None` for non-finite inputs, negative speed/horizon, nonpositive
/// rates, or arithmetic that cannot be represented by finite values. The caller
/// must treat `None` as a failed check, not as zero lateral acceleration.
///
/// There are at most three affine intervals, separated by the two ramp arrival
/// times. Each checks its endpoints and at most one interior stationary point:
/// for `v = v₀ + a*t` and `k = k₀ + b*t`, the derivative of `v²*k` is
/// `v * (2*a*k + b*v)`. The latter factor is affine. Curvature zero crossings
/// cannot be maxima of `abs(v²*k)` unless the whole segment is zero. This gives
/// a fixed amount of work, without sampling or heap allocation.
pub fn lateral_acceleration_peak(
    transition: MotionTransition,
    horizon_s: f64,
) -> Option<LateralAccelerationPeak> {
    let values = [
        transition.initial_speed_mps,
        transition.target_speed_mps,
        transition.initial_curvature_per_m,
        transition.target_curvature_per_m,
        transition.max_accel_mps2,
        transition.max_decel_mps2,
        transition.max_curvature_rate_per_s,
        horizon_s,
    ];
    if values.iter().any(|value| !value.is_finite())
        || transition.initial_speed_mps < 0.0
        || transition.target_speed_mps < 0.0
        || horizon_s < 0.0
        || transition.max_accel_mps2 <= 0.0
        || transition.max_decel_mps2 <= 0.0
        || transition.max_curvature_rate_per_s <= 0.0
    {
        return None;
    }
    let speed = Ramp::new(
        transition.initial_speed_mps,
        transition.target_speed_mps,
        if transition.target_speed_mps >= transition.initial_speed_mps {
            transition.max_accel_mps2
        } else {
            transition.max_decel_mps2
        },
    )?;
    let curvature = Ramp::new(
        transition.initial_curvature_per_m,
        transition.target_curvature_per_m,
        transition.max_curvature_rate_per_s,
    )?;
    let mut peak = LateralAccelerationPeak {
        at_s: 0.0,
        lateral_accel_mps2: lateral_acceleration(speed.initial, curvature.initial)?,
    };
    // Division by a tiny positive rate may produce infinity: clipping that
    // arrival time to the finite horizon correctly keeps the ramp unfinished.
    let mut cuts = [
        0.0,
        horizon_s,
        speed.arrival_s.min(horizon_s),
        curvature.arrival_s.min(horizon_s),
    ];
    cuts.sort_unstable_by(f64::total_cmp);
    for interval in cuts.windows(2) {
        let [start, end] = [interval[0], interval[1]];
        if start == end {
            continue;
        }
        let v0 = speed.at(start);
        let v1 = speed.at(end);
        let k0 = curvature.at(start);
        let k1 = curvature.at(end);
        update_peak(&mut peak, end, v1, k1)?;

        // Normalize both state axes and the changes before evaluating the
        // affine derivative. This avoids products of very small rates and
        // unnecessary overflow for large but finite values. Its sign change
        // locates the same root as -(2*a*k₀ + b*v₀)/(3*a*b).
        let speed_scale = v0.max(v1);
        let curvature_scale = k0.abs().max(k1.abs());
        if speed_scale == 0.0 || curvature_scale == 0.0 {
            continue;
        }
        let [v0, v1] = [v0 / speed_scale, v1 / speed_scale];
        let [k0, k1] = [k0 / curvature_scale, k1 / curvature_scale];
        let dv = v1 - v0;
        let dk = k1 - k0;
        let change_scale = dv.abs().max(dk.abs());
        if change_scale == 0.0 {
            continue;
        }
        let [dv, dk] = [dv / change_scale, dk / change_scale];
        let derivative_start = (2.0 * dv).mul_add(k0, dk * v0);
        let derivative_end = (2.0 * dv).mul_add(k1, dk * v1);
        if (derivative_start < 0.0 && derivative_end > 0.0)
            || (derivative_start > 0.0 && derivative_end < 0.0)
        {
            let fraction = derivative_start / (derivative_start - derivative_end);
            let at_s = (end - start).mul_add(fraction, start);
            update_peak(&mut peak, at_s, speed.at(at_s), curvature.at(at_s))?;
        }
    }
    Some(peak)
}

struct Ramp {
    initial: f64,
    target: f64,
    change: f64,
    rate: f64,
    arrival_s: f64,
}

impl Ramp {
    fn new(initial: f64, target: f64, rate: f64) -> Option<Self> {
        let change = target - initial;
        change.is_finite().then_some(Self {
            initial,
            target,
            change,
            rate,
            arrival_s: change.abs() / rate,
        })
    }

    fn at(&self, at_s: f64) -> f64 {
        if at_s == 0.0 {
            self.initial
        } else if at_s >= self.arrival_s {
            self.target
        } else {
            let travel = (self.rate * at_s).min(self.change.abs());
            self.initial + self.change.signum() * travel
        }
    }
}

fn lateral_acceleration(speed: f64, curvature: f64) -> Option<f64> {
    // Multiplying curvature before the second speed factor avoids an infinite
    // speed² intermediate when the final result is finite (e.g. tiny curvature).
    let value = speed * (speed * curvature.abs());
    value.is_finite().then_some(value)
}

fn update_peak(
    peak: &mut LateralAccelerationPeak,
    at_s: f64,
    speed: f64,
    curvature: f64,
) -> Option<()> {
    let value = lateral_acceleration(speed, curvature)?;
    if value > peak.lateral_accel_mps2 || (value == peak.lateral_accel_mps2 && at_s < peak.at_s) {
        *peak = LateralAccelerationPeak {
            at_s,
            lateral_accel_mps2: value,
        };
    }
    Some(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn projection_integrates_stop_distance_and_constant_curvature_exactly() {
        let motion = MotionTransition {
            initial_speed_mps: 0.3,
            target_speed_mps: 0.0,
            initial_curvature_per_m: 0.7,
            target_curvature_per_m: 0.7,
            max_accel_mps2: 0.4,
            max_decel_mps2: 0.8,
            max_curvature_rate_per_s: 4.0,
        };
        let projected = project_motion(Pose2::default(), motion, 0.8).unwrap();
        let distance = 0.3_f64.powi(2) / (2.0 * 0.8);
        let expected = crate::navigation::integrate(Pose2::default(), distance, 0.7);
        assert!((projected.distance_m - distance).abs() < 1e-12);
        assert!(projected.pose.point().distance(expected.point()) < 1e-10);
        assert!((projected.pose.yaw_rad - expected.yaw_rad).abs() < 1e-10);
        assert_eq!(projected.speed_mps, 0.0);
        assert!(project_motion(Pose2::default(), motion, 2.001).is_none());
        assert!(
            project_motion(
                Pose2 {
                    x_m: 1e100,
                    ..Pose2::default()
                },
                motion,
                0.1
            )
            .is_none()
        );
        assert!(
            project_motion(
                Pose2::default(),
                MotionTransition {
                    target_speed_mps: f64::NAN,
                    ..motion
                },
                0.1
            )
            .is_none()
        );
    }

    #[test]
    fn projection_handles_independent_ramp_splits_against_fine_midpoint_reference() {
        let motion = MotionTransition {
            initial_speed_mps: 0.3,
            target_speed_mps: 0.12,
            initial_curvature_per_m: -0.7,
            target_curvature_per_m: 1.2,
            max_accel_mps2: 0.4,
            max_decel_mps2: 0.8,
            max_curvature_rate_per_s: 4.0,
        };
        let projected = project_motion(Pose2::default(), motion, 0.6).unwrap();
        // Independent fine midpoint model, evaluated directly from time rather
        // than from the production projection's state or saturation splits.
        let dt = 0.6 / 60_000.0;
        let mut expected = Pose2::default();
        for i in 0..60_000 {
            let t = (i as f64 + 0.5) * dt;
            let v = (0.3 - 0.8 * t).max(0.12);
            let k = (-0.7 + 4.0 * t).min(1.2);
            let yaw = expected.yaw_rad + v * k * dt * 0.5;
            expected.x_m += v * yaw.cos() * dt;
            expected.y_m += v * yaw.sin() * dt;
            expected.yaw_rad += v * k * dt;
        }
        assert!(projected.pose.point().distance(expected.point()) < 1e-8);
        assert!((projected.pose.yaw_rad - expected.yaw_rad).abs() < 1e-8);
        assert_eq!(projected.speed_mps, 0.12);
        assert_eq!(projected.curvature_per_m, 1.2);
        let stationary = project_motion(
            Pose2::default(),
            MotionTransition {
                initial_speed_mps: 0.0,
                target_speed_mps: 0.0,
                ..motion
            },
            0.6,
        )
        .unwrap();
        assert_eq!(stationary.pose, Pose2::default());
        assert_eq!(stationary.curvature_per_m, 1.2);
    }

    fn transition() -> MotionTransition {
        MotionTransition {
            initial_speed_mps: 1.0,
            target_speed_mps: (0.5_f64 / 0.9).sqrt(),
            initial_curvature_per_m: 0.5,
            target_curvature_per_m: 0.9,
            max_accel_mps2: 0.4,
            max_decel_mps2: 2.6,
            max_curvature_rate_per_s: 4.0,
        }
    }

    #[test]
    fn legal_endpoints_hide_an_interior_peak_in_either_turn_direction() {
        for sign in [-1.0, 1.0] {
            let mut motion = transition();
            motion.initial_curvature_per_m *= sign;
            motion.target_curvature_per_m *= sign;
            let peak = lateral_acceleration_peak(motion, 0.1).unwrap();
            assert!((peak.at_s - 0.044_871_794_871_794_865).abs() < 1e-12);
            assert!((peak.lateral_accel_mps2 - 0.530_188_746_438_746_5).abs() < 1e-12);
            assert!(
                motion.initial_speed_mps - motion.max_decel_mps2 * 0.1 <= motion.target_speed_mps
            );
            assert!((motion.target_curvature_per_m - motion.initial_curvature_per_m).abs() <= 0.4);
            assert!(
                lateral_acceleration(motion.target_speed_mps, motion.target_curvature_per_m)
                    .unwrap()
                    <= 0.5 + 1e-12
            );
        }
    }

    #[test]
    fn simultaneous_braking_and_steering_need_not_use_independent_maxima() {
        let motion = MotionTransition {
            target_speed_mps: 0.6,
            max_decel_mps2: 4.0,
            ..transition()
        };
        let peak = lateral_acceleration_peak(motion, 0.1).unwrap();
        assert!((peak.lateral_accel_mps2 - 0.5).abs() < 1e-12);
        assert_eq!(peak.at_s, 0.0);
        assert!(motion.initial_speed_mps.powi(2) * motion.target_curvature_per_m > 0.5);
    }

    #[test]
    fn unequal_ramp_arrivals_keep_the_constant_tail_in_the_peak_search() {
        let speed_first = MotionTransition {
            initial_curvature_per_m: 0.1,
            target_curvature_per_m: 1.0,
            target_speed_mps: 0.5,
            max_decel_mps2: 5.0,
            max_curvature_rate_per_s: 1.0,
            ..transition()
        };
        let peak = lateral_acceleration_peak(speed_first, 1.0).unwrap();
        assert!((peak.lateral_accel_mps2 - 0.25).abs() < 1e-12);
        assert!((peak.at_s - 0.9).abs() < 1e-12);

        let curvature_first = MotionTransition {
            initial_speed_mps: 0.2,
            target_speed_mps: 1.0,
            initial_curvature_per_m: 0.1,
            target_curvature_per_m: 2.0,
            max_accel_mps2: 0.2,
            max_curvature_rate_per_s: 10.0,
            ..transition()
        };
        let peak = lateral_acceleration_peak(curvature_first, 5.0).unwrap();
        assert!((peak.lateral_accel_mps2 - 2.0).abs() < 1e-12);
        assert!((peak.at_s - 4.0).abs() < 1e-12);
    }

    #[test]
    fn horizon_can_cover_a_partial_ramp_or_a_long_steering_transition() {
        let motion = MotionTransition {
            initial_speed_mps: 0.3,
            target_speed_mps: 0.3,
            initial_curvature_per_m: 0.0,
            target_curvature_per_m: 2.0,
            max_curvature_rate_per_s: 0.4,
            ..transition()
        };
        let short = lateral_acceleration_peak(motion, 0.1).unwrap();
        let long = lateral_acceleration_peak(motion, 10.0).unwrap();
        assert!((short.lateral_accel_mps2 - 0.0036).abs() < 1e-12);
        assert_eq!(short.at_s, 0.1);
        assert!((long.lateral_accel_mps2 - 0.18).abs() < 1e-12);
        assert_eq!(long.at_s, 5.0);
        assert_eq!(
            lateral_acceleration_peak(motion, 0.0)
                .unwrap()
                .lateral_accel_mps2,
            0.0
        );
    }

    #[test]
    fn constants_zero_speed_and_curvature_sign_crossing_are_well_defined() {
        let still = MotionTransition {
            initial_speed_mps: 0.0,
            target_speed_mps: 0.0,
            initial_curvature_per_m: 1.0,
            target_curvature_per_m: -2.0,
            ..transition()
        };
        assert_eq!(
            lateral_acceleration_peak(still, 2.0),
            Some(LateralAccelerationPeak {
                at_s: 0.0,
                lateral_accel_mps2: 0.0
            })
        );
        let constant = MotionTransition {
            initial_speed_mps: 0.5,
            target_speed_mps: 0.5,
            initial_curvature_per_m: -1.0,
            target_curvature_per_m: -1.0,
            ..transition()
        };
        assert_eq!(
            lateral_acceleration_peak(constant, 2.0).unwrap(),
            LateralAccelerationPeak {
                at_s: 0.0,
                lateral_accel_mps2: 0.25
            }
        );
        let crossing = MotionTransition {
            target_curvature_per_m: 2.0,
            max_curvature_rate_per_s: 1.0,
            ..constant
        };
        let peak = lateral_acceleration_peak(crossing, 4.0).unwrap();
        assert_eq!(peak.lateral_accel_mps2, 0.5);
        assert_eq!(peak.at_s, 3.0);
    }

    #[test]
    fn analytic_peak_bounds_independent_dense_samples_across_transition_shapes() {
        // Independent direct ramps validate the analytic result, including
        // sign changes and saturation in different orders; no sampled value
        // is used to choose the implementation's peak.
        for (v0, vt) in [(0.0, 0.8), (0.8, 0.2), (0.4, 0.4)] {
            for (k0, kt) in [(-1.0, 1.2), (0.8, -0.3), (-1.0, -0.2), (0.0, 0.0)] {
                for horizon in [0.07, 0.5, 2.0] {
                    let motion = MotionTransition {
                        initial_speed_mps: v0,
                        target_speed_mps: vt,
                        initial_curvature_per_m: k0,
                        target_curvature_per_m: kt,
                        max_accel_mps2: 0.7,
                        max_decel_mps2: 1.4,
                        max_curvature_rate_per_s: 3.0,
                    };
                    let peak = lateral_acceleration_peak(motion, horizon).unwrap();
                    assert!((0.0..=horizon).contains(&peak.at_s));
                    let sample = |at: f64| {
                        let acceleration = if vt >= v0 { 0.7 } else { 1.4 };
                        let v = v0 + (vt - v0).clamp(-acceleration * at, acceleration * at);
                        let k = k0 + (kt - k0).clamp(-3.0 * at, 3.0 * at);
                        v * v * k.abs()
                    };
                    assert!((sample(peak.at_s) - peak.lateral_accel_mps2).abs() < 1e-12);
                    for i in 0..=1000 {
                        assert!(
                            sample(horizon * i as f64 / 1000.0) <= peak.lateral_accel_mps2 + 1e-12
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn invalid_inputs_and_unrepresentable_arithmetic_fail_closed() {
        let valid = transition();
        let base = [
            valid.initial_speed_mps,
            valid.target_speed_mps,
            valid.initial_curvature_per_m,
            valid.target_curvature_per_m,
            valid.max_accel_mps2,
            valid.max_decel_mps2,
            valid.max_curvature_rate_per_s,
            0.1,
        ];
        for index in 0..base.len() {
            for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
                let mut values = base;
                values[index] = value;
                let motion = MotionTransition {
                    initial_speed_mps: values[0],
                    target_speed_mps: values[1],
                    initial_curvature_per_m: values[2],
                    target_curvature_per_m: values[3],
                    max_accel_mps2: values[4],
                    max_decel_mps2: values[5],
                    max_curvature_rate_per_s: values[6],
                };
                assert!(lateral_acceleration_peak(motion, values[7]).is_none());
            }
        }
        for motion in [
            MotionTransition {
                initial_speed_mps: -0.1,
                ..valid
            },
            MotionTransition {
                target_speed_mps: -0.1,
                ..valid
            },
            MotionTransition {
                max_accel_mps2: 0.0,
                ..valid
            },
            MotionTransition {
                max_decel_mps2: -1.0,
                ..valid
            },
            MotionTransition {
                max_curvature_rate_per_s: 0.0,
                ..valid
            },
            MotionTransition {
                initial_speed_mps: f64::MAX,
                ..valid
            },
            MotionTransition {
                initial_curvature_per_m: -f64::MAX,
                target_curvature_per_m: f64::MAX,
                ..valid
            },
        ] {
            assert!(lateral_acceleration_peak(motion, 0.1).is_none());
        }
        assert!(lateral_acceleration_peak(valid, -0.1).is_none());
    }

    #[test]
    fn tiny_positive_rates_and_finite_large_products_do_not_require_sampling() {
        let motion = MotionTransition {
            target_speed_mps: 2.0,
            max_accel_mps2: f64::from_bits(1),
            max_curvature_rate_per_s: f64::from_bits(1),
            ..transition()
        };
        let peak = lateral_acceleration_peak(motion, 0.1).unwrap();
        assert_eq!(peak.lateral_accel_mps2, 0.5);
        let large = MotionTransition {
            initial_speed_mps: 1e200,
            target_speed_mps: 1e200,
            initial_curvature_per_m: 1e-200,
            target_curvature_per_m: 1e-200,
            ..transition()
        };
        assert_eq!(
            lateral_acceleration_peak(large, 1.0)
                .unwrap()
                .lateral_accel_mps2,
            1e200
        );
    }
}
