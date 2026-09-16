//! Actual short-exit counterexample, including source steering clock and boundary.
use super::*;
use serde_json::{Value, from_value};
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

fn restore(input: &Value, policy: TargetPolicy) -> Navigator {
    let mut nav =
        Navigator::new_with_target_policy(from_value(input["config"].clone()).unwrap(), policy)
            .unwrap();
    nav.set_travel_boundary(Some(super::short_point_target::recorded_boundary(input)));
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

fn replay(nav: &mut Navigator, input: &Value) -> NavigationDecision {
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

fn fixture() -> Value {
    serde_json::from_str(include_str!("fixtures/online38400_exit_heading.json")).unwrap()
}

// Checked HalfPlane reconstruction and decimal JSON round trips can differ by
// a few ulps. This comparison is only for saved test evidence, never permission.
fn same_evidence(actual: &Value, expected: &Value) {
    match (actual, expected) {
        (Value::Number(a), Value::Number(b)) => {
            let a = a.as_f64().unwrap();
            let b = b.as_f64().unwrap();
            assert!((a - b).abs() <= 16.0 * f64::EPSILON * (1.0 + a.abs().max(b.abs())));
        }
        (Value::Array(a), Value::Array(b)) => {
            assert_eq!(a.len(), b.len());
            for (a, b) in a.iter().zip(b) {
                same_evidence(a, b);
            }
        }
        (Value::Object(a), Value::Object(b)) => {
            assert_eq!(a.len(), b.len());
            for (key, value) in a {
                same_evidence(value, &b[key]);
            }
        }
        _ => assert_eq!(actual, expected),
    }
}

#[test]
fn actual_exit_rebuild_keeps_short_certified_route_and_fixed_behavior() {
    let data = fixture();
    for frame in data["frames"].as_array().unwrap() {
        let input = &frame["controller_input"];
        let fixed = replay(&mut restore(input, TargetPolicy::Fixed), input);
        same_evidence(
            &serde_json::to_value(fixed).unwrap(),
            &frame["recorded_fixed_decision"],
        );
        let mut nav = restore(input, TargetPolicy::RollingLocal);
        let commanded = nav.steering.commanded_curvature_per_m;
        let mut advanced = nav.steering;
        advanced
            .advance_to(
                Timestamp(input["now"].as_u64().unwrap()),
                nav.config.max_curvature_rate_per_s,
            )
            .unwrap();
        let decision = replay(&mut nav, input);
        if input["now"] == 38300 {
            same_evidence(
                &serde_json::to_value(&decision).unwrap(),
                &frame["recorded_rolling_decision"],
            );
        } else {
            assert!(
                frame["recorded_rolling_decision"]["diagnostics"]["remaining_distance_m"]
                    .as_f64()
                    .unwrap()
                    > 3.0
            );
            assert!(matches!(decision.status, NavigationStatus::Driving));
            assert!(decision.diagnostics.remaining_distance_m.unwrap() < 0.4);
            assert_eq!(
                decision.diagnostics.forward_search.ordinary.allocated_nodes,
                0
            );
            assert_eq!(
                decision.diagnostics.terminal_work.arrival_region_accepted,
                1
            );
        }
        assert_eq!(nav.steering.commanded_curvature_per_m, commanded);
        assert_eq!(
            nav.steering.applied_curvature_per_m,
            advanced.applied_curvature_per_m
        );
        let work = decision.diagnostics.terminal_work;
        assert!(
            work.solver_attempts <= 256
                && work.iterations <= 1024
                && work.primitive_samples <= 65_536
        );
        assert!(!work.budget_exhausted);
    }
}

#[test]
fn actual_exit_heading_primitive_obeys_integrated_endpoint_and_error_bounds() {
    let data = fixture();
    let input = &data["frames"][1]["controller_input"];
    let nav = restore(input, TargetPolicy::RollingLocal);
    let config = &nav.config;
    let start: Pose2 = from_value(input["estimate"]["pose"].clone()).unwrap();
    let goal: Point2 = from_value(input["goal"].clone()).unwrap();
    let heading = input["heading"].as_f64().unwrap();
    let obstacles: Vec<ObstacleDisc> = from_value(input["obstacles"].clone()).unwrap();
    let mut steering = nav.steering;
    steering
        .advance_to(Timestamp(38400), config.max_curvature_rate_per_s)
        .unwrap();
    for continuous in [false, true] {
        let grid = Grid::with_boundary(config, &obstacles, nav.travel_boundary);
        if continuous {
            grid.enable_recovery();
        }
        let budget = TerminalBudget::default();
        assert!(
            terminal::oriented_arrival_region(
                start,
                steering.applied_curvature_per_m,
                goal,
                heading,
                config,
                &grid,
                &budget,
                primitive_envelope::ErrorBound::default()
            )
            .is_none()
        );
        assert!(
            terminal::zero_target_curvature_arrival_region(
                start,
                steering.applied_curvature_per_m,
                goal,
                heading,
                config,
                &grid,
                &budget,
                primitive_envelope::ErrorBound::default()
            )
            .is_none()
        );
        let (points, end, k, error) = terminal::heading_ramp_arrival_region(
            start,
            steering.applied_curvature_per_m,
            goal,
            heading,
            config,
            &grid,
            &budget,
            primitive_envelope::ErrorBound::default(),
        )
        .unwrap();
        assert_eq!(points.last(), Some(&end.point()));
        assert!(end.point().distance(goal) > 0.01);
        assert!(end.point().distance(goal) + error.position_m < config.goal_tolerance_m * 0.5);
        assert!(
            angle_error(end.yaw_rad, heading).abs() + error.heading_rad
                <= config.goal_heading_tolerance_rad * 0.5
        );
        assert!(k.abs() < 1e-12);
        assert!(nav.travel_boundary.unwrap().contains_footprint(
            config.footprint,
            end,
            config.clearance_m + error.position_m
        ));
    }
}

#[test]
fn exit_heading_primitive_rejects_real_crossing_obstacles_errors_and_exhausted_work() {
    let data = fixture();
    let input = &data["frames"][1]["controller_input"];
    let nav = restore(input, TargetPolicy::RollingLocal);
    let config = &nav.config;
    let start: Pose2 = from_value(input["estimate"]["pose"].clone()).unwrap();
    let goal: Point2 = from_value(input["goal"].clone()).unwrap();
    let heading = input["heading"].as_f64().unwrap();
    let mut steering = nav.steering;
    steering
        .advance_to(Timestamp(38400), config.max_curvature_rate_per_s)
        .unwrap();
    let k = steering.applied_curvature_per_m;
    let attempt = |grid: &Grid, budget: &TerminalBudget, error| {
        terminal::heading_ramp_arrival_region(start, k, goal, heading, config, grid, budget, error)
    };
    let boundary = HalfPlane::new(start.point(), heading, 0.26).unwrap();
    assert!(boundary.contains_footprint(config.footprint, start, 0.0));
    assert!(!boundary.contains_footprint(
        config.footprint,
        Pose2 {
            x_m: goal.x_m,
            y_m: goal.y_m,
            yaw_rad: heading
        },
        0.0
    ));
    assert!(
        attempt(
            &Grid::with_boundary(config, &[], Some(boundary)),
            &TerminalBudget::default(),
            primitive_envelope::ErrorBound::default()
        )
        .is_none()
    );
    let obstacle = ObstacleDisc {
        center: Point2 {
            x_m: start.x_m + 0.3 * heading.cos(),
            y_m: start.y_m + 0.3 * heading.sin(),
        },
        radius_m: 0.01,
    };
    assert!(
        attempt(
            &Grid::with_boundary(config, &[obstacle], None),
            &TerminalBudget::default(),
            primitive_envelope::ErrorBound::default()
        )
        .is_none()
    );
    let open = Grid::with_boundary(config, &[], None);
    open.enable_recovery();
    for error in [
        primitive_envelope::ErrorBound {
            position_m: config.goal_tolerance_m * 0.5,
            heading_rad: 0.0,
        },
        primitive_envelope::ErrorBound {
            position_m: 0.0,
            heading_rad: config.goal_heading_tolerance_rad,
        },
    ] {
        assert!(attempt(&open, &TerminalBudget::default(), error).is_none());
    }
    let budget = TerminalBudget::default();
    for _ in 0..=256 {
        let _ = attempt(&open, &budget, primitive_envelope::ErrorBound::default());
        if budget.snapshot().budget_exhausted {
            break;
        }
    }
    assert!(attempt(&open, &budget, primitive_envelope::ErrorBound::default()).is_none());
    assert_eq!(budget.snapshot().solver_attempts, 256);
    assert!(budget.snapshot().budget_exhausted);
    for bad in [
        f64::NAN,
        f64::INFINITY,
        0.0,
        config.max_curvature_per_m + 1.0,
    ] {
        assert!(
            terminal::heading_ramp_arrival_region(
                start,
                bad,
                goal,
                heading,
                config,
                &open,
                &TerminalBudget::default(),
                primitive_envelope::ErrorBound::default()
            )
            .is_none()
        );
    }
}
