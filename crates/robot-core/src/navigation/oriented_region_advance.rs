//! Replays a real stopped async crosswalk approach, including its finite line,
//! measured scan obstacles, applied steering and unchanged adoption history.
use super::*;
use serde_json::{Value, from_value};

fn fixture() -> Value {
    serde_json::from_str(include_str!(
        "fixtures/online21380_crosswalk_arrival_region.json"
    ))
    .unwrap()
}

fn restore(input: &Value, policy: TargetPolicy) -> Navigator {
    let mut nav =
        Navigator::new_with_target_policy(from_value(input["config"].clone()).unwrap(), policy)
            .unwrap();
    nav.set_travel_boundary(Some(short_point_target::recorded_boundary(input)));
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
    assert!(input["before_route"].is_null());
    assert!(input["before_seed"].is_null());
    assert_eq!(input["before_arrival"]["type"], "stop");
    assert_eq!(input["before_pending_rebuild"], "boundary_changed");
    nav.pending_route_rebuild = Some(RouteRebuildReason::BoundaryChanged);
    nav.via_index = input["before_via_index"].as_u64().unwrap() as usize;
    nav.route_revision = input["before_revision"].as_u64().unwrap();
    nav.continuation_checked = input["before_continuation_checked"].as_bool().unwrap();
    nav.recovery_active = input["before_recovery_active"].as_bool().unwrap();
    nav.recovery_route_error_m = input["before_recovery_route_error_m"].as_f64().unwrap();
    nav.recovery_sample_arc_bound_m = input["before_recovery_sample_arc_bound_m"]
        .as_f64()
        .unwrap();
    nav.set_adoption_constraints(from_value(input["adoption_constraints"].clone()).unwrap());
    nav
}

fn replay(input: &Value, policy: TargetPolicy) -> (Navigator, NavigationDecision) {
    let mut nav = restore(input, policy);
    let decision = nav
        .plan_with_arrival(
            Timestamp(input["now"].as_u64().unwrap()),
            &from_value(input["estimate"].clone()).unwrap(),
            &from_value::<Vec<ObstacleDisc>>(input["obstacles"].clone()).unwrap(),
            Timestamp(input["obstacles_at"].as_u64().unwrap()),
            from_value(input["goal"].clone()).unwrap(),
            input["heading"].as_f64(),
            input["speed_limit_mps"].as_f64().unwrap(),
            ArrivalBehavior::Stop,
        )
        .unwrap();
    (nav, decision)
}

#[test]
fn captured_crosswalk_advance_is_certified_in_original_continuous_domain() {
    let data = fixture();
    let input = &data["controller_input"];
    let config: NavigationConfig = from_value(input["config"].clone()).unwrap();
    let start: Pose2 = from_value(input["estimate"]["pose"].clone()).unwrap();
    let goal: Point2 = from_value(input["goal"].clone()).unwrap();
    let heading = input["heading"].as_f64().unwrap();
    let obstacles: Vec<ObstacleDisc> = from_value(input["obstacles"].clone()).unwrap();
    let boundary = short_point_target::recorded_boundary(input);
    let grid = Grid::with_boundary(&config, &obstacles, Some(boundary));
    grid.enable_recovery();
    let budget = TerminalBudget::default();
    assert!(
        terminal::oriented_arrival_region(
            start,
            0.0,
            goal,
            heading,
            &config,
            &grid,
            &budget,
            primitive_envelope::ErrorBound::default()
        )
        .is_none()
    );
    let (points, end, curvature, error) = terminal::rolling_oriented_arrival_region(
        start,
        0.0,
        goal,
        heading,
        &config,
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
    assert!(boundary.contains_footprint(
        config.footprint,
        end,
        config.clearance_m + error.position_m
    ));
    assert_eq!(curvature, 0.0);
    assert!((end.yaw_rad - start.yaw_rad).abs() < 1e-12);
    assert_eq!(budget.snapshot().arrival_region_accepted, 1);
    assert!(!budget.snapshot().budget_exhausted);
}

#[test]
fn captured_rolling_arrival_drives_without_fabricating_an_adopted_command() {
    let data = fixture();
    let input = &data["controller_input"];
    let (nav, decision) = replay(input, TargetPolicy::RollingLocal);
    assert_eq!(decision.status, NavigationStatus::Driving, "{decision:?}");
    assert!(decision.intent.speed_mps > 0.0);
    assert!(decision.diagnostics.remaining_distance_m.unwrap() < 0.11);
    assert!(decision.diagnostics.terminal_work.arrival_region_accepted > 0);
    assert_eq!(nav.execution_state().applied_curvature_per_m, 0.0);
    assert_eq!(nav.execution_state().commanded_curvature_per_m, 0.0);
    assert_eq!(
        decision
            .diagnostics
            .adoption_constraints
            .unwrap()
            .adopted_revision,
        95
    );
    assert!(decision.diagnostics.terminal_work.solver_attempts <= 256);
    assert!(decision.diagnostics.terminal_work.iterations <= 1024);
    assert!(decision.diagnostics.terminal_work.primitive_samples <= 65536);
}

#[test]
fn captured_default_fixed_arrival_retains_original_blocked_outcome() {
    let data = fixture();
    let (_, decision) = replay(&data["controller_input"], TargetPolicy::Fixed);
    assert_eq!(decision.status, NavigationStatus::Blocked);
    assert_eq!(
        decision.reason.as_deref(),
        Some("no_forward_kinematic_path")
    );
    assert_eq!(
        decision.diagnostics.terminal_work.arrival_region_accepted,
        0
    );
    assert_eq!(decision.diagnostics.terminal_work.solver_attempts, 4);
    assert_eq!(decision.diagnostics.terminal_work.iterations, 4);
    assert_eq!(decision.diagnostics.terminal_work.primitive_samples, 48);
    assert_eq!(
        decision.diagnostics.forward_search.recovery.allocated_nodes,
        1
    );
}

#[test]
fn zero_target_candidate_ramps_actual_steering_and_preserves_all_constraints() {
    let data = fixture();
    let input = &data["controller_input"];
    let config: NavigationConfig = from_value(input["config"].clone()).unwrap();
    let start: Pose2 = from_value(input["estimate"]["pose"].clone()).unwrap();
    let goal: Point2 = from_value(input["goal"].clone()).unwrap();
    let heading = input["heading"].as_f64().unwrap();
    let obstacles: Vec<ObstacleDisc> = from_value(input["obstacles"].clone()).unwrap();
    let boundary = Some(short_point_target::recorded_boundary(input));
    let grid = Grid::with_boundary(&config, &obstacles, boundary);
    grid.enable_recovery();
    let solve = |curvature, goal, heading, grid: &Grid, error| {
        terminal::zero_target_curvature_arrival_region(
            start,
            curvature,
            goal,
            heading,
            &config,
            grid,
            &TerminalBudget::default(),
            error,
        )
    };
    let no_error = primitive_envelope::ErrorBound::default();
    let (_, straight, _, _) = solve(0.0, goal, heading, &grid, no_error).unwrap();
    let (points, ramped, _, error) = solve(0.4, goal, heading, &grid, no_error).unwrap();
    assert_eq!(points.last(), Some(&ramped.point()));
    assert!(ramped.point().distance(straight.point()) > 0.0001);
    assert!(ramped.yaw_rad - straight.yaw_rad > 0.005);
    assert!(ramped.point().distance(goal) + error.position_m < config.goal_tolerance_m * 0.5);
    assert!(solve(1.0, goal, heading, &grid, no_error).is_none());
    assert!(solve(0.0, goal, heading + 0.2, &grid, no_error).is_none());
    assert!(
        solve(
            0.0,
            goal,
            heading,
            &grid,
            primitive_envelope::ErrorBound {
                position_m: config.goal_tolerance_m * 0.5,
                heading_rad: 0.0,
            }
        )
        .is_none()
    );
    assert!(
        solve(
            0.0,
            goal,
            heading,
            &grid,
            primitive_envelope::ErrorBound {
                position_m: 0.0,
                heading_rad: config.goal_heading_tolerance_rad * 0.5,
            }
        )
        .is_none()
    );
    for x in [-0.1, 0.351] {
        assert!(
            solve(
                0.0,
                start.body_to_world(Point2 { x_m: x, y_m: 0.0 }),
                heading,
                &grid,
                no_error
            )
            .is_none()
        );
    }
    let mut blocked = obstacles.clone();
    blocked.push(ObstacleDisc {
        center: straight.body_to_world(Point2 {
            x_m: config.footprint.front_m,
            y_m: config.footprint.half_width_m,
        }),
        radius_m: 0.01,
    });
    let blocked_grid = Grid::with_boundary(&config, &blocked, boundary);
    blocked_grid.enable_recovery();
    assert!(solve(0.0, goal, heading, &blocked_grid, no_error).is_none());
}

#[test]
fn zero_target_candidate_shares_ledger_and_never_resets_it() {
    let data = fixture();
    let input = &data["controller_input"];
    let config: NavigationConfig = from_value(input["config"].clone()).unwrap();
    let start: Pose2 = from_value(input["estimate"]["pose"].clone()).unwrap();
    let goal: Point2 = from_value(input["goal"].clone()).unwrap();
    let heading = input["heading"].as_f64().unwrap();
    let obstacles: Vec<ObstacleDisc> = from_value(input["obstacles"].clone()).unwrap();
    let grid = Grid::with_boundary(
        &config,
        &obstacles,
        Some(short_point_target::recorded_boundary(input)),
    );
    grid.enable_recovery();
    let budget = TerminalBudget::default();
    let call = || {
        terminal::zero_target_curvature_arrival_region(
            start,
            0.0,
            goal,
            heading,
            &config,
            &grid,
            &budget,
            primitive_envelope::ErrorBound::default(),
        )
    };
    for _ in 0..256 {
        assert!(call().is_some());
    }
    let charged = budget.snapshot();
    assert_eq!(charged.solver_attempts, 256);
    assert_eq!(charged.iterations, 256);
    assert_eq!(charged.primitive_samples, 1024);
    assert!(call().is_none());
    let before = budget.snapshot();
    assert!(before.budget_exhausted);
    assert!(
        terminal::rolling_oriented_arrival_region(
            start,
            0.0,
            goal,
            heading,
            &config,
            &grid,
            &budget,
            primitive_envelope::ErrorBound::default()
        )
        .is_none()
    );
    let after = budget.snapshot();
    assert_eq!(after.solver_attempts, before.solver_attempts);
    assert_eq!(after.iterations, before.iterations);
    assert_eq!(after.primitive_samples, before.primitive_samples);
    assert_eq!(
        after.arrival_region_attempts,
        before.arrival_region_attempts
    );
}
