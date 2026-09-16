//! Actual async inputs before and after the first strict-continuation failure.
//! The later input contains the original Stop and plant history, not a claimed
//! future state after adopting the newly recovered command.
use super::*;
use serde_json::{Value, from_value};

fn fixture() -> Value {
    serde_json::from_str(include_str!(
        "fixtures/online38580_terminal_continuity_region.json"
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
        other => panic!("unexpected captured arrival {other}"),
    }
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
    // These are captured cache invalidations, not fabricated empty histories.
    assert!(input["before_route"].is_null() && input["before_seed"].is_null());
    nav.via_index = input["before_via_index"].as_u64().unwrap() as usize;
    nav.route_revision = input["before_revision"].as_u64().unwrap();
    nav.continuation_checked = input["before_continuation_checked"].as_bool().unwrap();
    nav.recovery_active = input["before_recovery_active"].as_bool().unwrap();
    nav.recovery_route_error_m = input["before_recovery_route_error_m"].as_f64().unwrap();
    nav.recovery_sample_arc_bound_m = input["before_recovery_sample_arc_bound_m"]
        .as_f64()
        .unwrap();
    nav.pending_route_rebuild = Some(match input["before_pending_rebuild"].as_str().unwrap() {
        "boundary_changed" => RouteRebuildReason::BoundaryChanged,
        "no_accepted_candidate" => RouteRebuildReason::NoAcceptedCandidate,
        other => panic!("unexpected captured invalidation {other}"),
    });
    nav.arrival = arrival(&input["before_arrival"]);
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

fn assert_recorded_path_and_intent(decision: &NavigationDecision, baseline: &Value) {
    assert_eq!(
        serde_json::to_value(decision.status).unwrap(),
        baseline["status"]
    );
    assert_eq!(
        serde_json::to_value(&decision.reason).unwrap(),
        baseline["reason"]
    );
    assert!(
        (decision.intent.speed_mps - baseline["intent"]["speed_mps"].as_f64().unwrap()).abs()
            < 1e-12
    );
    assert!(
        (decision.intent.curvature_per_m - baseline["intent"]["curvature_per_m"].as_f64().unwrap())
            .abs()
            < 1e-12
    );
    let points: Vec<Point2> = from_value(baseline["path"].clone()).unwrap();
    assert_eq!(points.len(), decision.path.len());
    for (old, new) in points.iter().zip(&decision.path) {
        assert!(old.distance(*new) < 1e-10);
    }
}

#[test]
fn recorded_async_continuation_reaches_real_half_region_after_all_strict_candidates_fail() {
    let data = fixture();
    let input = &data["controller_inputs"][1];
    let baseline = &data["baseline_decisions"][1];
    assert_eq!(baseline["status"], "blocked");
    assert_eq!(
        baseline["diagnostics"]["candidates"]["terminal_unreachable"],
        44
    );
    let mut nav = restore(input, TargetPolicy::RollingLocal);
    let mut steering = nav.execution_state();
    steering
        .advance_to(Timestamp(38580), nav.config.max_curvature_rate_per_s)
        .unwrap();
    let decision = replay(&mut nav, input);
    assert_eq!(decision.status, NavigationStatus::Driving, "{decision:?}");
    assert_eq!(decision.diagnostics.candidates.accepted, 1);
    assert_eq!(decision.diagnostics.candidates.terminal_unreachable, 43);
    assert_eq!(
        decision.diagnostics.terminal_work.arrival_region_accepted,
        1
    );
    assert_eq!(decision.diagnostics.terminal_work.solver_attempts, 115);
    assert_eq!(decision.diagnostics.terminal_work.iterations, 193);
    assert_eq!(decision.diagnostics.terminal_work.primitive_samples, 10266);
    assert!(!decision.diagnostics.terminal_work.budget_exhausted);
    assert!(decision.diagnostics.remaining_distance_m.unwrap() < 0.4);
    assert_eq!(
        nav.execution_state(),
        steering,
        "planning cannot adopt output"
    );
    assert!(
        nav.terminal_seed.is_none(),
        "region is not a fictitious exact seed"
    );

    let source: PoseEstimate = from_value(input["estimate"].clone()).unwrap();
    let constraints = nav.adoption_constraints.unwrap();
    let obstacles: Vec<ObstacleDisc> = from_value(input["obstacles"].clone()).unwrap();
    let goal: Point2 = from_value(input["goal"].clone()).unwrap();
    let heading = input["heading"].as_f64().unwrap();
    constraints
        .check_command(
            &nav.config,
            decision.intent.speed_mps,
            decision.intent.curvature_per_m,
            &obstacles,
            nav.travel_boundary,
        )
        .unwrap();
    let next = project_motion(
        source.pose,
        MotionTransition {
            initial_speed_mps: source.speed_mps,
            target_speed_mps: decision.intent.speed_mps,
            initial_curvature_per_m: steering.applied_curvature_per_m,
            target_curvature_per_m: decision.intent.curvature_per_m,
            max_accel_mps2: nav.config.max_accel_mps2,
            max_decel_mps2: nav.config.max_decel_mps2,
            max_curvature_rate_per_s: nav.config.max_curvature_rate_per_s,
        },
        nav.config.control_period_ms as f64 / 1000.0,
    )
    .unwrap();
    let grid = Grid::with_boundary(&nav.config, &obstacles, nav.travel_boundary);
    let (points, end, _, error) = terminal::oriented_arrival_region(
        next.pose,
        next.curvature_per_m,
        goal,
        heading,
        &nav.config,
        &grid,
        &TerminalBudget::default(),
        primitive_envelope::ErrorBound::default(),
    )
    .unwrap();
    assert_eq!(points.last(), Some(&end.point()));
    assert!(end.point().distance(goal) > 1e-4, "no endpoint snap");
    assert!(end.point().distance(goal) + error.position_m < nav.config.goal_tolerance_m * 0.5);
    assert!(
        angle_error(end.yaw_rad, heading).abs() + error.heading_rad
            <= nav.config.goal_heading_tolerance_rad * 0.5
    );
}

#[test]
fn strict_successful_neighbor_and_all_fixed_inputs_keep_original_results() {
    let data = fixture();
    for index in 0..3 {
        let input = &data["controller_inputs"][index];
        let baseline = &data["baseline_decisions"][index];
        let fixed = replay(&mut restore(input, TargetPolicy::Fixed), input);
        assert_recorded_path_and_intent(&fixed, baseline);
        assert_eq!(fixed.diagnostics.terminal_work.arrival_region_attempts, 0);
        if index == 0 {
            let rolling = replay(&mut restore(input, TargetPolicy::RollingLocal), input);
            assert_recorded_path_and_intent(&rolling, baseline);
            assert_eq!(rolling.diagnostics.terminal_work.arrival_region_attempts, 0);
        }
    }
}

#[test]
fn region_continuation_keeps_current_boundary_history_error_and_one_terminal_budget() {
    let data = fixture();
    let input = &data["controller_inputs"][1];
    let mut nav = restore(input, TargetPolicy::RollingLocal);
    let decision = replay(&mut nav, input);
    let mut changed_history = nav.adoption_constraints.unwrap();
    changed_history.last_command_change_at = Some(changed_history.planned_at);
    changed_history.held_curvature_per_m = -nav.config.max_curvature_per_m;
    changed_history.curvature_bound_per_m = nav.config.max_curvature_per_m;
    changed_history.validate(&nav.config).unwrap();
    let obstacles: Vec<ObstacleDisc> = from_value(input["obstacles"].clone()).unwrap();
    assert!(
        changed_history
            .check_command(
                &nav.config,
                decision.intent.speed_mps,
                decision.intent.curvature_per_m,
                &obstacles,
                nav.travel_boundary
            )
            .is_err()
    );

    let start = Pose2 {
        x_m: 5.2306068577517975,
        y_m: 3.15848683404157,
        yaw_rad: 3.0354455673851515,
    };
    let initial_curvature = 0.9963194976018632;
    let goal = from_value(input["goal"].clone()).unwrap();
    let heading = input["heading"].as_f64().unwrap();
    let boundary = HalfPlane::new(start.point(), heading, 0.3).unwrap();
    assert!(boundary.contains_footprint(nav.config.footprint, start, nav.config.clearance_m));
    let closed = Grid::with_boundary(&nav.config, &obstacles, Some(boundary));
    assert!(
        terminal::oriented_arrival_region(
            start,
            initial_curvature,
            goal,
            heading,
            &nav.config,
            &closed,
            &TerminalBudget::default(),
            primitive_envelope::ErrorBound::default()
        )
        .is_none()
    );
    let clear = Grid::with_boundary(&nav.config, &obstacles, nav.travel_boundary);
    assert!(
        terminal::oriented_arrival_region(
            start,
            initial_curvature,
            goal,
            heading,
            &nav.config,
            &clear,
            &TerminalBudget::default(),
            primitive_envelope::ErrorBound {
                position_m: nav.config.goal_tolerance_m,
                heading_rad: 0.0,
            }
        )
        .is_none()
    );
    let budget = TerminalBudget::default();
    for _ in 0..512 {
        let _ = terminal::oriented_arrival_region(
            start,
            initial_curvature,
            goal,
            heading,
            &nav.config,
            &clear,
            &budget,
            primitive_envelope::ErrorBound::default(),
        );
        if budget.snapshot().budget_exhausted {
            break;
        }
    }
    let before = budget.snapshot();
    assert!(before.budget_exhausted);
    assert!(
        terminal::oriented_arrival_region(
            start,
            initial_curvature,
            goal,
            heading,
            &nav.config,
            &clear,
            &budget,
            primitive_envelope::ErrorBound::default()
        )
        .is_none()
    );
    let after = budget.snapshot();
    assert_eq!(before.solver_attempts, after.solver_attempts);
    assert_eq!(before.iterations, after.iterations);
    assert_eq!(before.primitive_samples, after.primitive_samples);
}
