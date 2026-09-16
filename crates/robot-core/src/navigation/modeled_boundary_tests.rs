use super::super::*;
use super::body_chord_padding;
use super::oriented_boundary_tests::{replay, restore};
use serde_json::{Value, from_value};

fn fixture() -> Value {
    serde_json::from_str(include_str!("fixtures/online32000_body_chord_tube.json")).unwrap()
}

#[test]
fn actual_sweep_false_negative_gets_strict_short_route_with_original_budgets() {
    let data = fixture();
    for index in 0..5 {
        let input = &data["controller_inputs"][index];
        let mut nav = restore(input, TargetPolicy::RollingLocal);
        let mut actual = nav.execution_state();
        actual
            .advance_to(
                Timestamp(input["now"].as_u64().unwrap()),
                nav.config.max_curvature_rate_per_s,
            )
            .unwrap();
        let decision = replay(&mut nav, input);
        if index == 4 {
            // This is the actual state AFTER the old 32000 ms Stop. A new
            // short route is not a permission to invent an executable rollout.
            assert_eq!(decision.status, NavigationStatus::Blocked);
            assert_eq!(
                decision.reason,
                Some("no_collision_free_braking_trajectory".into())
            );
        } else {
            assert_eq!(decision.status, NavigationStatus::Driving, "{decision:?}");
            assert!(decision.diagnostics.candidates.accepted > 0);
        }
        assert!(decision.diagnostics.remaining_distance_m.unwrap() < 0.6);
        assert_eq!(
            decision.diagnostics.forward_search.ordinary.allocated_nodes,
            0
        );
        assert_eq!(
            decision.diagnostics.forward_search.recovery.allocated_nodes,
            0
        );
        assert!(decision.diagnostics.travel_boundary.is_some());
        let work = decision.diagnostics.terminal_work;
        assert!(!work.budget_exhausted);
        assert!(
            work.solver_attempts <= 256
                && work.iterations <= 1024
                && work.primitive_samples <= 65_536
        );
        assert_eq!(
            nav.execution_state(),
            actual,
            "planning cannot adopt steering"
        );
        if index == 3 {
            assert_eq!(
                data["baseline_plans"][index]["navigation"]["reason"],
                "forward_node_budget_exhausted"
            );
            let mut fixed = restore(input, TargetPolicy::Fixed);
            let rejected = replay(&mut fixed, input);
            assert_eq!(rejected.status, NavigationStatus::Blocked);
            assert_eq!(
                rejected.reason,
                Some("forward_node_budget_exhausted".into())
            );
            assert_eq!(
                rejected.diagnostics.forward_search.recovery.allocated_nodes,
                nav.config.max_grid_cells
            );
            // Some rollout candidates still violate the real boundary. This
            // certificate does not turn the whole lateral band into free space.
            assert!(decision.diagnostics.candidates.accepted < 44);
        }
    }
}

fn grid(config: &NavigationConfig, obstacles: &[ObstacleDisc], boundary: HalfPlane) -> Grid {
    let mut grid = Grid::with_boundary(config, obstacles, Some(boundary));
    grid.recovery.oriented_boundary = true;
    grid.enable_recovery();
    grid
}

#[test]
fn tighter_model_proof_preserves_actual_domain_uncertainty_and_missing_model_rejection() {
    let data = fixture();
    let input = &data["controller_inputs"][3];
    let config: NavigationConfig = from_value(input["config"].clone()).unwrap();
    let boundary = super::super::short_point_target::recorded_boundary(input);
    // A constant-curvature segment at the captured first-rejection location.
    // Full controller replay above covers the exact original ramp and cache.
    let from = Pose2 {
        x_m: 4.2904494007081055,
        y_m: 2.6110348797437894,
        yaw_rad: 2.714903182792701,
    };
    let motion = MotionTransition {
        initial_speed_mps: 0.3,
        target_speed_mps: 0.3,
        initial_curvature_per_m: 1.61,
        target_curvature_per_m: 1.61,
        max_accel_mps2: config.max_accel_mps2,
        max_decel_mps2: config.max_decel_mps2,
        max_curvature_rate_per_s: config.max_curvature_rate_per_s,
    };
    let duration = 0.023312368509006195 / 0.3;
    let predicted = primitive_envelope::predict(from, motion, duration).unwrap();
    let error = primitive_envelope::ErrorBound {
        position_m: 0.000002,
        heading_rad: 0.000000001,
    }
    .advance(&predicted)
    .unwrap();
    let to = predicted.projection.pose;
    let length = predicted.projection.distance_m;
    let obstacles: Vec<ObstacleDisc> = from_value(input["obstacles"].clone()).unwrap();
    let allowed = grid(&config, &obstacles, boundary);
    assert!(!allowed.oriented_motion_transition_clear(from, to, length, error));
    assert!(!allowed.modeled_motion_transition_clear(from, to, length, error, None));
    assert!(allowed.modeled_motion_transition_clear(
        from,
        to,
        length,
        error,
        Some((motion, duration))
    ));
    assert!(!allowed.modeled_motion_transition_clear(
        from,
        to,
        length,
        error,
        Some((motion, f64::NAN))
    ));
    assert!(!allowed.modeled_motion_transition_clear(
        from,
        to,
        length,
        error,
        Some((motion, -0.1))
    ));
    for uncertain in [
        primitive_envelope::ErrorBound {
            position_m: 0.03,
            heading_rad: error.heading_rad,
        },
        primitive_envelope::ErrorBound {
            position_m: error.position_m,
            heading_rad: 0.15,
        },
    ] {
        assert!(!allowed.modeled_motion_transition_clear(
            from,
            to,
            length,
            uncertain,
            Some((motion, duration))
        ));
    }
    let occupied = grid(
        &config,
        &[ObstacleDisc {
            center: from.point(),
            radius_m: 0.01,
        }],
        boundary,
    );
    assert!(!occupied.modeled_motion_transition_clear(
        from,
        to,
        length,
        error,
        Some((motion, duration))
    ));
    let mut tight_bounds = config.clone();
    tight_bounds.bounds.max_x_m = from.x_m + 0.1;
    assert!(
        !grid(&tight_bounds, &[], boundary).modeled_motion_transition_clear(
            from,
            to,
            length,
            error,
            Some((motion, duration))
        )
    );
    let mut crossed = to;
    crossed.y_m += 0.1;
    assert!(!allowed.modeled_motion_transition_clear(
        from,
        crossed,
        length + 0.1,
        error,
        Some((motion, duration))
    ));
    let mut fixed = grid(&config, &obstacles, boundary);
    fixed.recovery.oriented_boundary = false;
    assert!(!fixed.modeled_motion_transition_clear(
        from,
        to,
        length,
        error,
        Some((motion, duration))
    ));
    let mut invalid = motion;
    invalid.initial_speed_mps = -0.1;
    assert!(body_chord_padding(config.footprint, invalid, duration).is_none());
}

#[test]
fn body_chord_tube_covers_acceleration_braking_and_steering_saturation() {
    let data = fixture();
    let input = &data["controller_inputs"][3];
    let config: NavigationConfig = from_value(input["config"].clone()).unwrap();
    let boundary = super::super::short_point_target::recorded_boundary(input);
    let domain = grid(&config, &[], boundary);
    let mut accepted = 0;
    for (initial_speed, target_speed) in [(0.0, 0.3), (0.3, 0.0), (0.3, 0.1), (0.3, 0.3)] {
        for (initial_curvature, target_curvature) in
            [(-2.0, 2.0), (2.0, -2.0), (1.4, 1.5), (1.5, 1.5)]
        {
            for duration in [0.025, 0.075, 0.15, 0.4] {
                let motion = MotionTransition {
                    initial_speed_mps: initial_speed,
                    target_speed_mps: target_speed,
                    initial_curvature_per_m: initial_curvature,
                    target_curvature_per_m: target_curvature,
                    max_accel_mps2: config.max_accel_mps2,
                    max_decel_mps2: config.max_decel_mps2,
                    max_curvature_rate_per_s: config.max_curvature_rate_per_s,
                };
                let start = Pose2 {
                    x_m: 4.3,
                    y_m: 2.60,
                    yaw_rad: 2.72,
                };
                let predicted = primitive_envelope::predict(start, motion, duration).unwrap();
                let error = primitive_envelope::ErrorBound::default()
                    .advance(&predicted)
                    .unwrap();
                let padding = body_chord_padding(config.footprint, motion, duration).unwrap();
                if !domain.recovery.oriented_boundary_with_padding(
                    start,
                    predicted.projection.pose,
                    predicted.projection.distance_m,
                    error,
                    padding,
                ) {
                    continue;
                }
                accepted += 1;
                for sample in 0..=128 {
                    let at = duration * sample as f64 / 128.0;
                    let actual = project_motion(start, motion, at).unwrap();
                    assert!(boundary.contains_footprint(
                        config.footprint,
                        actual.pose,
                        domain.recovery.boundary_padding
                    ));
                }
            }
        }
    }
    assert!(
        accepted >= 24,
        "exercise accepted ramps, including saturation inside the segment"
    );
}
