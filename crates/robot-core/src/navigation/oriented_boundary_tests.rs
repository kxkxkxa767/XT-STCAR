use super::super::*;
use serde_json::{Value, from_value};

fn fixture() -> Value {
    serde_json::from_str(include_str!(
        "fixtures/online31400_finite_boundary_capsule.json"
    ))
    .unwrap()
}

fn arrival(value: &Value) -> ArrivalBehavior {
    match value["type"].as_str().unwrap() {
        "stop" => ArrivalBehavior::Stop,
        "pass_through" => ArrivalBehavior::PassThrough {
            next: from_value(value["next"].clone()).unwrap(),
            next_heading_rad: value["next_heading_rad"].as_f64(),
            next_max_speed_mps: value["next_max_speed_mps"].as_f64().unwrap(),
            admission_radius_m: value["admission_radius_m"].as_f64().unwrap(),
        },
        other => panic!("unexpected recorded arrival {other}"),
    }
}

pub(super) fn restore(input: &Value, policy: TargetPolicy) -> Navigator {
    let mut nav =
        Navigator::new_with_target_policy(from_value(input["config"].clone()).unwrap(), policy)
            .unwrap();
    nav.set_travel_boundary(Some(super::super::short_point_target::recorded_boundary(
        input,
    )));
    nav.steering = SteeringEstimate {
        at: Timestamp(input["before_steering"]["at"].as_u64().unwrap()),
        applied_curvature_per_m: input["before_steering"]["applied_curvature_per_m"]
            .as_f64()
            .unwrap(),
        commanded_curvature_per_m: input["before_steering"]["commanded_curvature_per_m"]
            .as_f64()
            .unwrap(),
    };
    nav.last_step = input["before_last_step"].as_u64().map(Timestamp);
    nav.route = from_value(input["before_route"].clone()).unwrap();
    nav.arrival = arrival(&input["before_arrival"]);
    nav.via_index = input["before_via_index"].as_u64().unwrap() as usize;
    nav.route_revision = input["before_revision"].as_u64().unwrap();
    nav.continuation_checked = input["before_continuation_checked"].as_bool().unwrap();
    nav.recovery_active = input["before_recovery_active"].as_bool().unwrap();
    nav.recovery_route_error_m = input["before_recovery_route_error_m"].as_f64().unwrap();
    nav.recovery_sample_arc_bound_m = input["before_recovery_sample_arc_bound_m"]
        .as_f64()
        .unwrap();
    nav.pending_route_rebuild = if input["before_pending_rebuild"].is_null() {
        None
    } else {
        assert_eq!(input["before_pending_rebuild"], "boundary_changed");
        Some(RouteRebuildReason::BoundaryChanged)
    };
    if !input["before_seed"].is_null() {
        let seed = &input["before_seed"];
        nav.terminal_seed = Some(CachedTerminalSeed {
            goal: from_value(seed["goal"].clone()).unwrap(),
            heading: seed["heading"].as_f64().unwrap(),
            seed: from_value(seed["seed"].clone()).unwrap(),
            anchor_at: Timestamp(seed["anchor_at"].as_u64().unwrap()),
            anchor_pose: from_value(seed["anchor_pose"].clone()).unwrap(),
            prefer_continuation: seed["prefer_continuation"].as_bool().unwrap(),
        });
    }
    nav.set_adoption_constraints(from_value(input["adoption_constraints"].clone()).unwrap());
    nav
}

pub(super) fn replay(nav: &mut Navigator, input: &Value) -> NavigationDecision {
    nav.plan_with_arrival(
        Timestamp(input["now"].as_u64().unwrap()),
        &from_value(input["estimate"].clone()).unwrap(),
        &from_value::<Vec<ObstacleDisc>>(input["obstacles"].clone()).unwrap(),
        Timestamp(input["obstacles_at"].as_u64().unwrap()),
        from_value(input["goal"].clone()).unwrap(),
        input["heading"].as_f64(),
        input["speed_limit_mps"].as_f64().unwrap(),
        arrival(&input["arrival"]),
    )
    .unwrap()
}

#[test]
fn real_finite_boundary_rejection_keeps_full_obstacles_and_becomes_a_short_drive() {
    let data = fixture();
    for index in [1, 2] {
        let input = &data["controller_inputs"][index];
        let baseline = &data["baseline_plans"][index]["navigation"];
        assert_eq!(baseline["reason"], "forward_node_budget_exhausted");
        let mut nav = restore(input, TargetPolicy::RollingLocal);
        let mut steering = nav.execution_state();
        steering
            .advance_to(
                Timestamp(input["now"].as_u64().unwrap()),
                nav.config.max_curvature_rate_per_s,
            )
            .unwrap();
        let result = replay(&mut nav, input);
        assert_eq!(result.status, NavigationStatus::Driving, "{result:?}");
        assert!(result.diagnostics.remaining_distance_m.unwrap() < 0.6);
        assert_eq!(result.diagnostics.candidates.accepted, 44);
        assert_eq!(
            result.diagnostics.forward_search.ordinary.allocated_nodes,
            0
        );
        // A tighter modeled sweep can now also avoid the continuation's
        // former one-node rejection; it never adds a search allocation.
        assert!(result.diagnostics.forward_search.recovery.allocated_nodes <= 1);
        let work = result.diagnostics.terminal_work;
        assert!(!work.budget_exhausted);
        assert!(
            work.solver_attempts <= 256
                && work.iterations <= 1024
                && work.primitive_samples <= 65_536
        );
        assert_eq!(
            nav.execution_state(),
            steering,
            "planning does not adopt steering"
        );
        assert!(nav.adoption_constraints.is_none());
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
    }
}

#[test]
fn recorded_neighbor_and_already_stopped_state_keep_their_original_result() {
    let data = fixture();
    for index in [0, 3] {
        let input = &data["controller_inputs"][index];
        let baseline = &data["baseline_plans"][index]["navigation"];
        let mut nav = restore(input, TargetPolicy::RollingLocal);
        let result = replay(&mut nav, input);
        assert_eq!(
            serde_json::to_value(result.status).unwrap(),
            baseline["status"]
        );
        assert_eq!(
            serde_json::to_value(result.reason).unwrap(),
            baseline["reason"]
        );
        let expected: MotionIntent = from_value(baseline["intent"].clone()).unwrap();
        assert!((result.intent.speed_mps - expected.speed_mps).abs() < 1e-12);
        assert!((result.intent.curvature_per_m - expected.curvature_per_m).abs() < 1e-12);
        let expected: Vec<Point2> = from_value(baseline["path"].clone()).unwrap();
        assert_eq!(result.path.len(), expected.len());
        assert!(
            result
                .path
                .iter()
                .zip(expected)
                .all(|(a, b)| a.distance(b) < 1e-12)
        );
    }
}

fn boundary_grid(
    config: &NavigationConfig,
    obstacles: &[ObstacleDisc],
    boundary: HalfPlane,
) -> Grid {
    let mut grid = Grid::with_boundary(config, obstacles, Some(boundary));
    grid.recovery.oriented_boundary = true;
    grid.enable_recovery();
    grid
}

#[test]
fn full_body_sweep_retains_corner_crossing_uncertainty_obstacles_and_bounds() {
    let data = fixture();
    let input = &data["controller_inputs"][1];
    let config: NavigationConfig = from_value(input["config"].clone()).unwrap();
    let boundary = super::super::short_point_target::recorded_boundary(input);
    let pose = Pose2 {
        x_m: 4.32296532855371,
        y_m: 2.5927211794361615,
        yaw_rad: 2.6426844023390696,
    };
    let next = integrate(pose, 0.01, 1.49);
    let obstacles: Vec<ObstacleDisc> = from_value(input["obstacles"].clone()).unwrap();
    let grid = boundary_grid(&config, &obstacles, boundary);
    assert!(!grid.motion_transition_clear(pose.point(), next.point(), 0.01));
    assert!(grid.oriented_motion_transition_clear(
        pose,
        next,
        0.01,
        primitive_envelope::ErrorBound::default()
    ));
    // A point-only cached/reference segment cannot borrow the pose proof.
    assert!(!grid.transition_clear(pose.point(), next.point()));
    for error in [
        primitive_envelope::ErrorBound {
            position_m: 0.15,
            heading_rad: 0.0,
        },
        primitive_envelope::ErrorBound {
            position_m: 0.0,
            heading_rad: 1.0,
        },
        primitive_envelope::ErrorBound {
            position_m: f64::NAN,
            heading_rad: 0.0,
        },
        primitive_envelope::ErrorBound {
            position_m: 0.0,
            heading_rad: f64::NAN,
        },
    ] {
        assert!(
            !grid
                .recovery
                .oriented_boundary_clear(pose, next, 0.01, error)
        );
    }
    let mut crossed = next;
    crossed.y_m += 0.2;
    assert!(boundary.contains_disc(crossed.point(), 0.0));
    assert!(!boundary.contains_footprint(config.footprint, crossed, config.clearance_m));
    assert!(!grid.oriented_motion_transition_clear(
        pose,
        crossed,
        0.22,
        primitive_envelope::ErrorBound::default()
    ));
    let occupied = boundary_grid(
        &config,
        &[ObstacleDisc {
            center: pose.point(),
            radius_m: 0.001,
        }],
        boundary,
    );
    assert!(!occupied.oriented_motion_transition_clear(
        pose,
        next,
        0.01,
        primitive_envelope::ErrorBound::default()
    ));
    let mut confined = config.clone();
    confined.bounds.max_x_m = pose.x_m + 0.1;
    let confined = boundary_grid(&confined, &[], boundary);
    assert!(!confined.oriented_motion_transition_clear(
        pose,
        next,
        0.01,
        primitive_envelope::ErrorBound::default()
    ));
    let mut disabled = boundary_grid(&config, &obstacles, boundary);
    disabled.recovery.oriented_boundary = false;
    assert!(!disabled.oriented_motion_transition_clear(
        pose,
        next,
        0.01,
        primitive_envelope::ErrorBound::default()
    ));
    // Endpoint bodies on opposite safe sides do not prove their hull avoids
    // the forbidden corner: the common-separator requirement must remain.
    let corner = HalfPlane::new(Point2 { x_m: 0.0, y_m: 0.0 }, 0.0, 0.0)
        .unwrap()
        .with_lateral_region(Rect {
            min_x_m: -2.0,
            max_x_m: 2.0,
            min_y_m: -0.5,
            max_y_m: 0.5,
        })
        .unwrap();
    let a = Pose2 {
        x_m: -0.5,
        y_m: 0.0,
        yaw_rad: 0.0,
    };
    let b = Pose2 {
        x_m: 0.5,
        y_m: 1.0,
        yaw_rad: 0.0,
    };
    assert!(corner.contains_footprint(config.footprint, a, config.clearance_m));
    assert!(corner.contains_footprint(config.footprint, b, config.clearance_m));
    let cornergrid = boundary_grid(&config, &[], corner);
    assert!(!cornergrid.recovery.oriented_boundary_clear(
        a,
        b,
        1.5,
        primitive_envelope::ErrorBound::default()
    ));
}

#[test]
fn finite_band_sweep_covers_variable_curvature_and_both_error_signs() {
    let data = fixture();
    let config: NavigationConfig =
        from_value(data["controller_inputs"][1]["config"].clone()).unwrap();
    let boundary =
        super::super::short_point_target::recorded_boundary(&data["controller_inputs"][1]);
    let grid = boundary_grid(&config, &[], boundary);
    let error = primitive_envelope::ErrorBound {
        position_m: 0.005,
        heading_rad: 0.01,
    };
    let mut certified = 0;
    // Includes instantaneous sign flips: a superset of the rate-limited model.
    // The analytic proof in recovery.rs carries the guarantee; these dense
    // exact constant-curvature pieces guard its implementation and error signs.
    for yaw in [2.4, 2.6, 2.8, 3.0] {
        for length in [0.01, 0.025, 0.05, 0.1] {
            for pattern in 0..4 {
                let start = Pose2 {
                    x_m: 4.3,
                    y_m: 2.57,
                    yaw_rad: yaw,
                };
                let mut pose = start;
                let mut samples = vec![start];
                for index in 0..128 {
                    let sign = match pattern {
                        0 => 1.0,
                        1 => -1.0,
                        2 => {
                            if index < 64 {
                                1.0
                            } else {
                                -1.0
                            }
                        }
                        _ => {
                            if index % 2 == 0 {
                                1.0
                            } else {
                                -1.0
                            }
                        }
                    };
                    pose = integrate(pose, length / 128.0, sign * config.max_curvature_per_m);
                    samples.push(pose);
                }
                if !grid
                    .recovery
                    .oriented_boundary_clear(start, pose, length, error)
                {
                    continue;
                }
                certified += 1;
                for sample in samples {
                    for position_sign in [-1.0, 1.0] {
                        for heading_sign in [-1.0, 1.0] {
                            let perturbed = Pose2 {
                                x_m: sample.x_m,
                                y_m: sample.y_m + position_sign * error.position_m,
                                yaw_rad: sample.yaw_rad + heading_sign * error.heading_rad,
                            };
                            assert!(boundary.contains_footprint(
                                config.footprint,
                                perturbed,
                                grid.recovery.boundary_padding
                            ));
                        }
                    }
                }
            }
        }
    }
    assert!(
        certified >= 32,
        "exercise accepted sweeps rather than only rejection"
    );
}
