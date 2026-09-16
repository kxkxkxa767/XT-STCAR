//! Exact worker input captured on second-cone acquisition, before the original
//! lattice selected a 2.94 m detour for a 0.79 m rolling point target.
use super::*;
use serde_json::{Value, from_value};

fn fixture() -> Value {
    serde_json::from_str(include_str!("fixtures/online49580_short_point_target.json")).unwrap()
}

pub(super) fn recorded_boundary(input: &Value) -> HalfPlane {
    let value = &input["boundary"];
    let origin: Point2 = from_value(value["origin"].clone()).unwrap();
    let normal: Point2 = from_value(value["normal"].clone()).unwrap();
    let yaw = normal.y_m.atan2(normal.x_m);
    let lower = value["lateral_bounds_m"][0].as_f64().unwrap();
    let upper = value["lateral_bounds_m"][1].as_f64().unwrap();
    let middle = (lower + upper) * 0.5;
    // HalfPlane deliberately has no unchecked Deserialize. Any rectangle with
    // this same support interval reconstructs the recorded finite half-plane.
    let center = Point2 {
        x_m: origin.x_m - middle * normal.y_m,
        y_m: origin.y_m + middle * normal.x_m,
    };
    let half_x = 0.25;
    let half_y = ((upper - lower) * 0.5 - half_x * normal.y_m.abs()) / normal.x_m.abs();
    let boundary = HalfPlane::new(origin, yaw, value["max_projection_m"].as_f64().unwrap())
        .unwrap()
        .with_lateral_region(Rect {
            min_x_m: center.x_m - half_x,
            max_x_m: center.x_m + half_x,
            min_y_m: center.y_m - half_y,
            max_y_m: center.y_m + half_y,
        })
        .unwrap();
    let restored = serde_json::to_value(boundary).unwrap();
    for i in 0..2 {
        assert!(
            (restored["lateral_bounds_m"][i].as_f64().unwrap()
                - value["lateral_bounds_m"][i].as_f64().unwrap())
            .abs()
                < 1e-14
        );
    }
    boundary
}

fn restore(input: &Value) -> Navigator {
    restore_with_policy(input, TargetPolicy::RollingLocal)
}

fn restore_with_policy(input: &Value, target_policy: TargetPolicy) -> Navigator {
    let mut nav = Navigator::new_with_target_policy(
        from_value(input["config"].clone()).unwrap(),
        target_policy,
    )
    .unwrap();
    nav.set_travel_boundary(Some(recorded_boundary(input)));
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
    nav.via_index = input["before_via_index"].as_u64().unwrap() as usize;
    nav.route_revision = input["before_revision"].as_u64().unwrap();
    nav.continuation_checked = input["before_continuation_checked"].as_bool().unwrap();
    nav.recovery_active = input["before_recovery_active"].as_bool().unwrap();
    nav.recovery_route_error_m = input["before_recovery_route_error_m"].as_f64().unwrap();
    nav.recovery_sample_arc_bound_m = input["before_recovery_sample_arc_bound_m"]
        .as_f64()
        .unwrap();
    assert!(input["before_pending_rebuild"].is_null());
    assert_eq!(input["before_arrival"]["type"], "stop");
    nav.pending_route_rebuild = None;
    let seed = &input["before_seed"];
    nav.terminal_seed = Some(CachedTerminalSeed {
        goal: from_value(seed["goal"].clone()).unwrap(),
        heading: seed["heading"].as_f64().unwrap(),
        seed: from_value(seed["seed"].clone()).unwrap(),
        anchor_at: Timestamp(seed["anchor_at"].as_u64().unwrap()),
        anchor_pose: from_value(seed["anchor_pose"].clone()).unwrap(),
        prefer_continuation: seed["prefer_continuation"].as_bool().unwrap(),
    });
    nav.set_adoption_constraints(from_value(input["adoption_constraints"].clone()).unwrap());
    nav
}

fn replay(input: &Value) -> NavigationDecision {
    replay_with_navigator(input, restore(input))
}

fn replay_with_navigator(input: &Value, mut nav: Navigator) -> NavigationDecision {
    nav.plan_with_arrival(
        Timestamp(input["now"].as_u64().unwrap()),
        &from_value(input["estimate"].clone()).unwrap(),
        &from_value::<Vec<ObstacleDisc>>(input["obstacles"].clone()).unwrap(),
        Timestamp(input["obstacles_at"].as_u64().unwrap()),
        from_value(input["goal"].clone()).unwrap(),
        input["heading"].as_f64(),
        input["speed_limit_mps"].as_f64().unwrap(),
        ArrivalBehavior::Stop,
    )
    .unwrap()
}

#[test]
fn captured_point_goal_has_a_certified_direct_ramped_connection() {
    let data = fixture();
    let input = &data["controller_inputs"][1];
    let config: NavigationConfig = from_value(input["config"].clone()).unwrap();
    let estimate: PoseEstimate = from_value(input["estimate"].clone()).unwrap();
    let goal: Point2 = from_value(input["goal"].clone()).unwrap();
    let obstacles: Vec<ObstacleDisc> = from_value(input["obstacles"].clone()).unwrap();
    let grid = Grid::with_boundary(&config, &obstacles, Some(recorded_boundary(input)));
    let curvature = input["before_steering"]["applied_curvature_per_m"]
        .as_f64()
        .unwrap();
    let budget = TerminalBudget::default();
    assert!(input["heading"].is_null());
    assert!(
        terminal::single_arc_with_error(
            estimate.pose,
            curvature,
            goal,
            None,
            &config,
            &grid,
            &budget,
            primitive_envelope::ErrorBound::default(),
        )
        .is_none()
    );
    assert_eq!(budget.snapshot().domain_rejections, 1);
    let result = terminal::short_point_arc_with_error(
        estimate.pose,
        curvature,
        goal,
        &config,
        &grid,
        &budget,
        primitive_envelope::ErrorBound::default(),
    );
    assert!(result.is_some(), "{:?}", budget.snapshot());
    let (points, end, curvature, error) = result.unwrap();
    assert!(end.point().distance(goal) > 0.0001);
    assert!(end.point().distance(goal) + error.position_m < config.goal_tolerance_m * 0.5);
    assert!(curvature > 0.0);
    assert_eq!(points.last(), Some(&end.point()));
    assert!(budget.snapshot().iterations <= 16);
    assert_eq!(budget.snapshot().arrival_region_attempts, 1);
    assert_eq!(budget.snapshot().arrival_region_accepted, 1);
    assert!(!budget.snapshot().budget_exhausted);
}

#[test]
fn recorded_point_acquisition_turns_into_short_connection_without_lattice_loop() {
    let data = fixture();
    let decision = replay(&data["controller_inputs"][1]);
    let old = &data["baseline_plans"][1]["navigation_diagnostics"];
    assert!(old["remaining_distance_m"].as_f64().unwrap() > 2.9);
    assert!(old["selected_curvature_per_m"].as_f64().unwrap() < 0.0);
    assert_eq!(decision.status, NavigationStatus::Driving);
    assert!(decision.intent.curvature_per_m > 0.0, "{decision:?}");
    assert!(decision.diagnostics.remaining_distance_m.unwrap() < 1.0);
    assert_eq!(
        decision.diagnostics.forward_search.ordinary.allocated_nodes,
        0
    );
    assert_eq!(
        decision.diagnostics.forward_search.recovery.allocated_nodes,
        0
    );
    assert_eq!(
        decision.diagnostics.terminal_work.arrival_region_accepted,
        1
    );
    assert!(decision.diagnostics.terminal_work.solver_attempts <= 256);
    assert!(decision.diagnostics.terminal_work.iterations <= 1024);
    assert!(decision.diagnostics.terminal_work.primitive_samples <= 65536);
}

#[test]
fn recorded_preceding_oriented_search_retains_original_decision() {
    let data = fixture();
    let decision = replay(&data["controller_inputs"][0]);
    let old = &data["baseline_plans"][0]["navigation_diagnostics"];
    assert_eq!(decision.status, NavigationStatus::Driving);
    assert!(
        (decision.intent.curvature_per_m - old["selected_curvature_per_m"].as_f64().unwrap()).abs()
            < 1e-12
    );
    assert!(
        (decision.diagnostics.remaining_distance_m.unwrap()
            - old["remaining_distance_m"].as_f64().unwrap())
        .abs()
            < 1e-12
    );
    assert_eq!(
        decision.diagnostics.terminal_work.arrival_region_attempts,
        0
    );
    assert_eq!(
        decision.diagnostics.terminal_work.arrival_region_accepted,
        0
    );
}

#[test]
fn default_fixed_target_policy_preserves_original_recorded_lattice_decision() {
    let data = fixture();
    let input = &data["controller_inputs"][1];
    let config: NavigationConfig = from_value(input["config"].clone()).unwrap();
    assert_eq!(
        Navigator::new(config).unwrap().target_policy,
        TargetPolicy::Fixed
    );
    let decision = replay_with_navigator(input, restore_with_policy(input, TargetPolicy::Fixed));
    let old = &data["baseline_plans"][1]["navigation_diagnostics"];
    assert_eq!(decision.status, NavigationStatus::Driving);
    assert!(
        (decision.intent.curvature_per_m - old["selected_curvature_per_m"].as_f64().unwrap()).abs()
            < 1e-12
    );
    assert!(
        (decision.diagnostics.remaining_distance_m.unwrap()
            - old["remaining_distance_m"].as_f64().unwrap())
        .abs()
            < 1e-12
    );
    assert_eq!(
        decision.diagnostics.terminal_work.arrival_region_attempts,
        0
    );
    assert_eq!(
        decision.diagnostics.terminal_work.arrival_region_accepted,
        0
    );
    assert_eq!(
        decision.diagnostics.forward_search.ordinary.allocated_nodes,
        17915
    );
}

#[test]
fn short_point_connection_preserves_obstacles_error_and_forward_domain() {
    let data = fixture();
    let input = &data["controller_inputs"][1];
    let config: NavigationConfig = from_value(input["config"].clone()).unwrap();
    let estimate: PoseEstimate = from_value(input["estimate"].clone()).unwrap();
    let goal: Point2 = from_value(input["goal"].clone()).unwrap();
    let obstacles: Vec<ObstacleDisc> = from_value(input["obstacles"].clone()).unwrap();
    let boundary = Some(recorded_boundary(input));
    let grid = Grid::with_boundary(&config, &obstacles, boundary);
    let k = input["before_steering"]["applied_curvature_per_m"]
        .as_f64()
        .unwrap();
    let solve = |goal, grid: &Grid, error| {
        terminal::short_point_arc_with_error(
            estimate.pose,
            k,
            goal,
            &config,
            grid,
            &TerminalBudget::default(),
            error,
        )
    };
    let (path, _, _, _) = solve(goal, &grid, primitive_envelope::ErrorBound::default()).unwrap();
    let mut blocked = obstacles.clone();
    blocked.push(ObstacleDisc {
        center: path[path.len() / 2],
        radius_m: 0.02,
    });
    let blocked_grid = Grid::with_boundary(&config, &blocked, boundary);
    assert!(
        solve(
            goal,
            &blocked_grid,
            primitive_envelope::ErrorBound::default()
        )
        .is_none()
    );
    assert!(
        solve(
            estimate.pose.body_to_world(Point2 {
                x_m: -0.8,
                y_m: 0.0
            }),
            &grid,
            primitive_envelope::ErrorBound::default()
        )
        .is_none()
    );
    assert!(
        solve(
            estimate.pose.body_to_world(Point2 {
                x_m: 1.101,
                y_m: 0.0
            }),
            &grid,
            primitive_envelope::ErrorBound::default()
        )
        .is_none()
    );
    grid.enable_recovery();
    let error = primitive_envelope::ErrorBound {
        position_m: config.goal_tolerance_m,
        heading_rad: 0.0,
    };
    assert!(
        solve(goal, &grid, error).is_none(),
        "carried error must remain in the endpoint test"
    );
}

#[test]
fn short_point_connection_never_resets_an_exhausted_terminal_ledger() {
    let data = fixture();
    let input = &data["controller_inputs"][1];
    let config: NavigationConfig = from_value(input["config"].clone()).unwrap();
    let estimate: PoseEstimate = from_value(input["estimate"].clone()).unwrap();
    let goal: Point2 = from_value(input["goal"].clone()).unwrap();
    let obstacles: Vec<ObstacleDisc> = from_value(input["obstacles"].clone()).unwrap();
    let grid = Grid::with_boundary(&config, &obstacles, Some(recorded_boundary(input)));
    let budget = TerminalBudget::default();
    for _ in 0..257 {
        terminal::single_arc_with_error(
            estimate.pose,
            0.0,
            estimate.pose.body_to_world(Point2 { x_m: 0.1, y_m: 0.0 }),
            None,
            &config,
            &grid,
            &budget,
            primitive_envelope::ErrorBound::default(),
        );
    }
    let before = budget.snapshot();
    assert!(before.budget_exhausted);
    assert!(
        terminal::short_point_arc_with_error(
            estimate.pose,
            0.0,
            goal,
            &config,
            &grid,
            &budget,
            primitive_envelope::ErrorBound::default(),
        )
        .is_none()
    );
    let after = budget.snapshot();
    assert_eq!(before.solver_attempts, after.solver_attempts);
    assert_eq!(before.iterations, after.iterations);
    assert_eq!(before.primitive_samples, after.primitive_samples);
    assert_eq!(after.arrival_region_attempts, 0);
    assert!(after.budget_exhausted);
}
