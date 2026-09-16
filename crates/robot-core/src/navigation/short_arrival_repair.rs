//! Replays an explicitly captured worker state; no fabricated route, steering,
//! seed, source time, adoption constraints or obstacle geometry.
use super::*;
use serde_json::{Value, from_value};

fn fixture() -> Value {
    serde_json::from_str(include_str!(
        "fixtures/online26580_short_arrival_drift.json"
    ))
    .unwrap()
}

fn restore(input: &Value) -> Navigator {
    let mut nav = Navigator::new(from_value(input["config"].clone()).unwrap()).unwrap();
    assert!(input["boundary"].is_null());
    assert_eq!(input["before_arrival"]["type"], "stop");
    assert!(input["before_pending_rebuild"].is_null());
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

fn replay(
    nav: &mut Navigator,
    input: &Value,
    goal: Point2,
    heading: Option<f64>,
    arrival: ArrivalBehavior,
) -> NavigationDecision {
    nav.plan_with_arrival(
        Timestamp(input["now"].as_u64().unwrap()),
        &from_value(input["estimate"].clone()).unwrap(),
        &from_value::<Vec<ObstacleDisc>>(input["obstacles"].clone()).unwrap(),
        Timestamp(input["obstacles_at"].as_u64().unwrap()),
        goal,
        heading,
        input["speed_limit_mps"].as_f64().unwrap(),
        arrival,
    )
    .unwrap()
}

#[test]
fn captured_drift_uses_same_region_before_lattice_replaces_short_route_with_loop() {
    let data = fixture();
    let input = &data["controller_inputs"][1];
    let mut nav = restore(input);
    let goal: Point2 = from_value(input["goal"].clone()).unwrap();
    let baseline = &data["baseline_plans"][1]["navigation_diagnostics"];
    assert_eq!(
        baseline["route_rebuild_reason"],
        "footprint_reference_drift"
    );
    assert!(
        baseline["route_length_change"]["old_remaining_distance_m"]
            .as_f64()
            .unwrap()
            < 0.37
    );
    assert!(
        baseline["route_length_change"]["new_remaining_distance_m"]
            .as_f64()
            .unwrap()
            > 3.7
    );
    assert!(!baseline["terminal_continuity_enforced"].as_bool().unwrap());
    let decision = replay(&mut nav, input, goal, Some(0.0), ArrivalBehavior::Stop);
    let diagnostics = &decision.diagnostics;
    assert_eq!(
        diagnostics.route_rebuild_reason,
        Some(RouteRebuildReason::FootprintReferenceDrift)
    );
    assert!(diagnostics.terminal_work.arrival_region_accepted >= 1);
    assert!(diagnostics.remaining_distance_m.unwrap() < 0.6);
    assert!(decision.intent.curvature_per_m < 0.0, "{decision:?}");
    assert!(diagnostics.terminal_work.solver_attempts <= 256);
    assert!(diagnostics.terminal_work.iterations <= 1024);
    assert!(diagnostics.terminal_work.primitive_samples <= 65_536);
    assert_eq!(diagnostics.forward_search.ordinary.allocated_nodes, 0);
    assert_eq!(diagnostics.forward_search.recovery.allocated_nodes, 0);
    assert_eq!(
        nav.execution_state().commanded_curvature_per_m,
        input["before_steering"]["commanded_curvature_per_m"]
            .as_f64()
            .unwrap()
    );
}

#[test]
fn captured_strict_failure_and_region_success_preserve_integrated_endpoint_and_error() {
    let data = fixture();
    let input = &data["controller_inputs"][1];
    let mut nav = restore(input);
    let pose: Pose2 = from_value(input["estimate"]["pose"].clone()).unwrap();
    let goal: Point2 = from_value(input["goal"].clone()).unwrap();
    let obstacles: Vec<ObstacleDisc> = from_value(input["obstacles"].clone()).unwrap();
    nav.last_step = Some(Timestamp(input["now"].as_u64().unwrap()));
    let grid = Grid::with_boundary(&nav.config, &obstacles, None);
    let k = nav.steering.applied_curvature_per_m;
    assert!(
        terminal_connection(
            &nav.config,
            &nav.terminal_budget,
            nav.terminal_seed,
            nav.last_step,
            pose,
            k,
            goal,
            0.0,
            &grid,
            primitive_envelope::ErrorBound::default()
        )
        .is_none()
    );
    let (points, end, _, error) = terminal::oriented_arrival_region(
        pose,
        k,
        goal,
        0.0,
        &nav.config,
        &grid,
        &nav.terminal_budget,
        primitive_envelope::ErrorBound::default(),
    )
    .unwrap();
    assert_eq!(points.last(), Some(&end.point()));
    assert!(end.point().distance(goal) > 0.0001);
    assert!(end.point().distance(goal) + error.position_m <= nav.config.goal_tolerance_m * 0.5);
    assert!(
        angle_error(end.yaw_rad, 0.0).abs() + error.heading_rad
            <= nav.config.goal_heading_tolerance_rad * 0.5
    );
    assert!(!nav.terminal_budget.snapshot().budget_exhausted);
}

#[test]
fn preceding_actual_cache_stays_on_original_successful_branch() {
    let data = fixture();
    let input = &data["controller_inputs"][0];
    let mut nav = restore(input);
    let decision = replay(
        &mut nav,
        input,
        from_value(input["goal"].clone()).unwrap(),
        Some(0.0),
        ArrivalBehavior::Stop,
    );
    assert_eq!(
        decision.diagnostics.terminal_work.arrival_region_attempts,
        0
    );
    assert_eq!(
        decision.diagnostics.route_revision,
        input["before_revision"].as_u64().unwrap()
    );
    assert!((decision.intent.curvature_per_m - -0.5984).abs() < 1e-12);
}

#[test]
fn short_repair_cannot_reset_or_extend_an_exhausted_shared_terminal_ledger() {
    let data = fixture();
    let input = &data["controller_inputs"][1];
    let mut nav = restore(input);
    let pose: Pose2 = from_value(input["estimate"]["pose"].clone()).unwrap();
    let goal: Point2 = from_value(input["goal"].clone()).unwrap();
    let obstacles: Vec<ObstacleDisc> = from_value(input["obstacles"].clone()).unwrap();
    nav.last_step = input["now"].as_u64().map(Timestamp);
    let grid = Grid::with_boundary(&nav.config, &obstacles, None);
    for _ in 0..257 {
        terminal::oriented_arrival_region(
            pose,
            nav.steering.applied_curvature_per_m,
            goal,
            0.0,
            &nav.config,
            &grid,
            &nav.terminal_budget,
            primitive_envelope::ErrorBound::default(),
        );
        if nav.terminal_budget.snapshot().budget_exhausted {
            break;
        }
    }
    let before = nav.terminal_budget.snapshot();
    assert!(before.budget_exhausted);
    assert!(
        nav.kinematic_path_from_with_arrival_repair(
            pose,
            nav.steering.applied_curvature_per_m,
            goal,
            Some(0.0),
            &grid,
            true,
        )
        .is_none()
    );
    let after = nav.terminal_budget.snapshot();
    assert_eq!(after.solver_attempts, before.solver_attempts);
    assert_eq!(after.iterations, before.iterations);
    assert_eq!(after.primitive_samples, before.primitive_samples);
    assert_eq!(grid.forward_search().ordinary.allocated_nodes, 0);
    assert_eq!(grid.forward_search().recovery.allocated_nodes, 0);
}

#[test]
fn changed_arrival_heading_goal_or_boundary_do_not_use_cached_drift_repair() {
    let data = fixture();
    let input = &data["controller_inputs"][1];
    for change in 0..4 {
        let mut nav = restore(input);
        let mut goal: Point2 = from_value(input["goal"].clone()).unwrap();
        let mut heading = Some(0.0);
        let mut arrival = ArrivalBehavior::Stop;
        match change {
            0 => {
                arrival = ArrivalBehavior::PassThrough {
                    next: Point2 {
                        x_m: goal.x_m + 0.5,
                        y_m: goal.y_m,
                    },
                    next_heading_rad: Some(0.0),
                    next_max_speed_mps: 0.18,
                    admission_radius_m: nav.config.goal_tolerance_m,
                }
            }
            1 => heading = None,
            2 => goal.x_m += 0.5,
            _ => {
                nav.set_travel_boundary(Some(HalfPlane::new(Point2::default(), 0.0, 9.0).unwrap()))
            }
        }
        let decision = replay(&mut nav, input, goal, heading, arrival);
        assert_eq!(
            decision.diagnostics.terminal_work.arrival_region_attempts, 0,
            "change={change}, {decision:?}"
        );
    }
}
