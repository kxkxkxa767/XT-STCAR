//! Five real online inputs straddling fresh finite-boundary updates. The
//! original nearby strict connections alternate between success and a lattice
//! loop; the current-boundary region connector is independently replayed here.
use super::*;
use serde_json::{Value, from_value};

fn fixture() -> Value {
    serde_json::from_str(include_str!(
        "fixtures/online18100_boundary_short_arrival.json"
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
    // The caller has just supplied a new measured boundary. This is the
    // actual post-invalidation cache state, not a synthetic missing route.
    assert!(input["before_route"].is_null() && input["before_seed"].is_null());
    assert_eq!(input["before_arrival"]["type"], "stop");
    assert_eq!(input["before_pending_rebuild"], "boundary_changed");
    assert!(input["adoption_constraints"].is_null());
    nav.pending_route_rebuild = Some(RouteRebuildReason::BoundaryChanged);
    nav.via_index = input["before_via_index"].as_u64().unwrap() as usize;
    nav.route_revision = input["before_revision"].as_u64().unwrap();
    nav.continuation_checked = input["before_continuation_checked"].as_bool().unwrap();
    nav.recovery_active = input["before_recovery_active"].as_bool().unwrap();
    nav.recovery_route_error_m = input["before_recovery_route_error_m"].as_f64().unwrap();
    nav.recovery_sample_arc_bound_m = input["before_recovery_sample_arc_bound_m"]
        .as_f64()
        .unwrap();
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
        ArrivalBehavior::Stop,
    )
    .unwrap()
}

#[test]
fn new_boundary_short_arrivals_use_real_region_endpoint_before_a_lattice_loop() {
    let data = fixture();
    for index in [1, 4] {
        let input = &data["controller_inputs"][index];
        let baseline = &data["baseline_plans"][index]["navigation"];
        let mut nav = restore(input, TargetPolicy::RollingLocal);
        let mut steering = nav.execution_state();
        // A synchronous capture still advances the last actually adopted
        // command through the elapsed interval before it evaluates a new plan.
        steering
            .advance_to(
                Timestamp(input["now"].as_u64().unwrap()),
                nav.config.max_curvature_rate_per_s,
            )
            .unwrap();
        let decision = replay(&mut nav, input);
        assert_eq!(decision.status, NavigationStatus::Driving, "{decision:?}");
        assert!(decision.intent.curvature_per_m < 0.0);
        assert!(decision.diagnostics.remaining_distance_m.unwrap() < 0.4);
        assert!(
            baseline["diagnostics"]["remaining_distance_m"]
                .as_f64()
                .unwrap()
                > 3.7
        );
        assert_eq!(
            decision.diagnostics.route_rebuild_reason,
            Some(RouteRebuildReason::BoundaryChanged)
        );
        assert_eq!(
            decision.diagnostics.terminal_work.arrival_region_accepted,
            1
        );
        assert_eq!(
            decision.diagnostics.forward_search.ordinary.allocated_nodes,
            0
        );
        assert_eq!(
            decision.diagnostics.forward_search.recovery.allocated_nodes,
            0
        );
        assert!(!decision.diagnostics.terminal_work.budget_exhausted);
        assert!(decision.diagnostics.terminal_work.solver_attempts <= 256);
        assert!(decision.diagnostics.terminal_work.iterations <= 1024);
        assert!(decision.diagnostics.terminal_work.primitive_samples <= 65_536);
        assert_eq!(
            nav.execution_state(),
            steering,
            "planning cannot adopt a command"
        );
        assert!(decision.diagnostics.adoption_constraints.is_none());

        let start: Pose2 = from_value(input["estimate"]["pose"].clone()).unwrap();
        let goal: Point2 = from_value(input["goal"].clone()).unwrap();
        let obstacles: Vec<ObstacleDisc> = from_value(input["obstacles"].clone()).unwrap();
        let boundary = short_point_target::recorded_boundary(input);
        let grid = Grid::with_boundary(&nav.config, &obstacles, Some(boundary));
        let budget = TerminalBudget::default();
        assert!(
            terminal_connection(
                &nav.config,
                &budget,
                None,
                nav.last_step,
                start,
                steering.applied_curvature_per_m,
                goal,
                0.0,
                &grid,
                primitive_envelope::ErrorBound::default(),
            )
            .is_none()
        );
        assert!(
            terminal::single_arc_with_error(
                start,
                steering.applied_curvature_per_m,
                goal,
                Some(0.0),
                &nav.config,
                &grid,
                &budget,
                primitive_envelope::ErrorBound::default(),
            )
            .is_none()
        );
        let (points, end, _, error) = terminal::rolling_oriented_arrival_region(
            start,
            steering.applied_curvature_per_m,
            goal,
            0.0,
            &nav.config,
            &grid,
            &budget,
            primitive_envelope::ErrorBound::default(),
        )
        .unwrap();
        assert_eq!(points.last(), Some(&end.point()));
        assert!(
            end.point().distance(goal) > 1e-4,
            "endpoint is not snapped to center"
        );
        assert!(end.point().distance(goal) + error.position_m < nav.config.goal_tolerance_m * 0.5);
        assert!(
            angle_error(end.yaw_rad, 0.0).abs() + error.heading_rad
                <= nav.config.goal_heading_tolerance_rad * 0.5
        );
        assert!(boundary.contains_footprint(
            nav.config.footprint,
            end,
            nav.config.clearance_m + error.position_m
        ));
    }
}

#[test]
fn successful_strict_neighbors_and_fixed_policy_keep_the_recorded_paths() {
    let data = fixture();
    for index in 0..5 {
        let input = &data["controller_inputs"][index];
        let baseline = &data["baseline_plans"][index]["navigation"];
        let mut fixed = restore(input, TargetPolicy::Fixed);
        let fixed_decision = replay(&mut fixed, input);
        let expected = baseline["diagnostics"]["remaining_distance_m"]
            .as_f64()
            .unwrap();
        assert!((fixed_decision.diagnostics.remaining_distance_m.unwrap() - expected).abs() < 1e-9);
        let expected_path: Vec<Point2> = from_value(baseline["path"].clone()).unwrap();
        assert_eq!(fixed_decision.path.len(), expected_path.len());
        for (actual, expected) in fixed_decision.path.iter().zip(&expected_path) {
            assert!(actual.distance(*expected) < 1e-9);
        }
        if [0, 2, 3].contains(&index) {
            let mut rolling = restore(input, TargetPolicy::RollingLocal);
            let decision = replay(&mut rolling, input);
            assert_eq!(
                decision.diagnostics.terminal_work.arrival_region_accepted,
                0
            );
            assert_eq!(decision.path, fixed_decision.path);
            assert_eq!(decision.intent, fixed_decision.intent);
        }
    }
}

#[test]
fn rolling_short_recertification_keeps_current_boundary_error_and_shared_budget() {
    let data = fixture();
    let input = &data["controller_inputs"][1];
    let nav = restore(input, TargetPolicy::RollingLocal);
    let start: Pose2 = from_value(input["estimate"]["pose"].clone()).unwrap();
    let goal: Point2 = from_value(input["goal"].clone()).unwrap();
    let obstacles: Vec<ObstacleDisc> = from_value(input["obstacles"].clone()).unwrap();
    let k = nav.steering.applied_curvature_per_m;
    let boundary = HalfPlane::new(start.point(), 0.0, 0.4).unwrap();
    assert!(boundary.contains_footprint(nav.config.footprint, start, nav.config.clearance_m));
    let restricted = Grid::with_boundary(&nav.config, &obstacles, Some(boundary));
    assert!(
        nav.certify_arrival_region(start, k, goal, Some(0.0), &restricted)
            .is_none()
    );
    let clear = Grid::with_boundary(
        &nav.config,
        &obstacles,
        Some(short_point_target::recorded_boundary(input)),
    );
    clear
        .recovery
        .next_leg_error
        .set(primitive_envelope::ErrorBound {
            position_m: nav.config.goal_tolerance_m,
            heading_rad: 0.0,
        });
    assert!(
        nav.certify_arrival_region(start, k, goal, Some(0.0), &clear)
            .is_none()
    );
    clear
        .recovery
        .next_leg_error
        .set(primitive_envelope::ErrorBound::default());
    for _ in 0..257 {
        let _ = nav.certify_arrival_region(start, k, goal, Some(0.0), &clear);
        if nav.terminal_budget.snapshot().budget_exhausted {
            break;
        }
    }
    let before = nav.terminal_budget.snapshot();
    assert!(before.budget_exhausted);
    assert!(
        nav.certify_arrival_region(start, k, goal, Some(0.0), &clear)
            .is_none()
    );
    assert_eq!(
        nav.terminal_budget.snapshot().solver_attempts,
        before.solver_attempts
    );
    assert_eq!(nav.terminal_budget.snapshot().iterations, before.iterations);
    assert_eq!(
        nav.terminal_budget.snapshot().primitive_samples,
        before.primitive_samples
    );
}
