use super::*;

fn recorded_constraints(
    value: &serde_json::Value,
    at: Timestamp,
    period_ms: u64,
) -> AdoptionConstraints {
    let number = |key: &str| value[key].as_f64().unwrap();
    AdoptionConstraints {
        // The original capture predates the actual-adoption-time contract.
        // A synthetic one-period interval isolates candidate-stage ordering;
        // it is not claimed to be the historical command change timestamp.
        planned_at: at,
        last_command_change_at: Some(Timestamp(at.0 - period_ms)),
        adopted_revision: 1,
        source_pose: serde_json::from_value(value["source_pose"].clone()).unwrap(),
        projected_pose: serde_json::from_value(value["projected_pose"].clone()).unwrap(),
        held_speed_mps: number("held_speed_mps"),
        source_age_s: number("source_age_s"),
        adoption_window_s: number("adoption_window_s"),
        speed_bound_mps: number("speed_bound_mps"),
        curvature_bound_per_m: number("curvature_bound_per_m"),
        projected_speed_mps: number("projected_speed_mps"),
        projected_curvature_per_m: number("projected_curvature_per_m"),
        window_end_speed_mps: number("window_end_speed_mps"),
        window_end_curvature_per_m: number("window_end_curvature_per_m"),
        held_curvature_per_m: number("held_curvature_per_m"),
        source_travel_time_s: number("source_travel_time_s"),
        transition_horizon_s: number("transition_horizon_s"),
        future_travel_time_s: number("future_travel_time_s"),
    }
}

fn arrival(value: &serde_json::Value) -> ArrivalBehavior {
    assert_eq!(value["type"], "pass_through");
    ArrivalBehavior::PassThrough {
        next: serde_json::from_value(value["next"].clone()).unwrap(),
        next_heading_rad: value["next_heading_rad"].as_f64(),
        next_max_speed_mps: value["next_max_speed_mps"].as_f64().unwrap(),
        admission_radius_m: value["admission_radius_m"].as_f64().unwrap(),
    }
}

#[test]
fn recorded_pp_grid_rejects_all_44_candidates_without_forecast_work() {
    let input: serde_json::Value =
        serde_json::from_str(include_str!("fixtures/pp19380_grid_blocked.json")).unwrap();
    let config: NavigationConfig = serde_json::from_value(input["config"].clone()).unwrap();
    let estimate: PoseEstimate = serde_json::from_value(input["estimate"].clone()).unwrap();
    let obstacles: Vec<ObstacleDisc> = serde_json::from_value(input["obstacles"].clone()).unwrap();
    let at = Timestamp(input["now"].as_u64().unwrap());
    let constraints =
        recorded_constraints(&input["adoption_constraints"], at, config.control_period_ms);
    let mut nav = Navigator::new(config).unwrap();
    nav.set_adoption_constraints(Some(constraints));
    let steering = &input["steering"];
    nav.set_execution_state(SteeringEstimate {
        at: Timestamp(steering["at"].as_u64().unwrap()),
        commanded_curvature_per_m: steering["commanded_curvature_per_m"].as_f64().unwrap(),
        applied_curvature_per_m: steering["applied_curvature_per_m"].as_f64().unwrap(),
    })
    .unwrap();
    let boundary = &input["travel_boundary"];
    nav.set_travel_boundary(Some(
        HalfPlane::new(
            serde_json::from_value(boundary["origin"].clone()).unwrap(),
            boundary["normal"]["y_m"]
                .as_f64()
                .unwrap()
                .atan2(boundary["normal"]["x_m"].as_f64().unwrap()),
            boundary["max_projection_m"].as_f64().unwrap(),
        )
        .unwrap(),
    ));
    nav.last_step = input["last_step"].as_u64().map(Timestamp);
    nav.arrival = arrival(&input["cached_arrival"]);
    nav.continuation_checked = input["continuation_checked"].as_bool().unwrap();
    nav.via_index = input["via_index"].as_u64().unwrap() as usize;
    nav.route_revision = input["route_revision"].as_u64().unwrap();
    nav.route = Some(serde_json::from_value(input["cached_route"].clone()).unwrap());
    nav.pending_route_rebuild = None;
    assert!(!input["recovery_active"].as_bool().unwrap());
    assert_eq!(input["terminal_seed_debug"], "None");
    assert_eq!(obstacles.len(), 361);
    assert_eq!(nav.route.as_ref().unwrap().2.len(), 285);
    let decision = nav
        .plan_with_arrival(
            at,
            &estimate,
            &obstacles,
            Timestamp(input["obstacles_at"].as_u64().unwrap()),
            serde_json::from_value(input["goal"].clone()).unwrap(),
            input["goal_heading_rad"].as_f64(),
            input["speed_limit_mps"].as_f64().unwrap(),
            arrival(&input["input_arrival"]),
        )
        .unwrap();
    assert_eq!(decision.status, NavigationStatus::Blocked, "{decision:?}");
    let diagnostics = &decision.diagnostics;
    assert_eq!(diagnostics.candidates.rollouts_evaluated, 44);
    assert_eq!(diagnostics.candidates.grid, 44);
    assert_eq!(diagnostics.candidates.accepted, 0);
    assert_eq!(diagnostics.candidates.admission_current, 0);
    assert_eq!(diagnostics.candidates.admission_next, 0);
    assert_eq!(
        diagnostics.admission_forecast_work,
        AdmissionForecastWork::default()
    );
    assert_eq!(diagnostics.route_revision, 3);
    assert_eq!(diagnostics.route_points, 130);
    assert_eq!(diagnostics.route_progress, Some(52));
    assert_eq!(diagnostics.terminal_work.solver_attempts, 0);
    assert!(diagnostics.route_rebuild_reason.is_none());
    assert_eq!(
        nav.pending_route_rebuild,
        Some(RouteRebuildReason::NoAcceptedCandidate)
    );
    eprintln!(
        "PP19380 forecast before={} after={:?}; candidates={:?}",
        input["original_navigation"]["admission_forecast_work"],
        diagnostics.admission_forecast_work,
        diagnostics.candidates
    );
}

fn open_scene() -> (Navigator, PoseEstimate, AdoptionConstraints) {
    let mut config = NavigationConfig::simulation(
        Rect {
            min_x_m: 0.0,
            min_y_m: 0.0,
            max_x_m: 10.0,
            max_y_m: 10.0,
        },
        Footprint {
            front_m: 0.22,
            rear_m: 0.18,
            half_width_m: 0.13,
        },
        FrameId("map".into()),
    );
    config.control_period_ms = 100;
    let at = Timestamp(1000);
    let pose = Pose2 {
        x_m: 5.0,
        y_m: 5.0,
        yaw_rad: 0.0,
    };
    let constraints = AdoptionConstraints {
        planned_at: at,
        last_command_change_at: Some(Timestamp(900)),
        adopted_revision: 1,
        source_pose: pose,
        projected_pose: pose,
        held_speed_mps: 0.2,
        source_age_s: 0.0,
        adoption_window_s: 0.1,
        speed_bound_mps: 0.2,
        curvature_bound_per_m: 0.0,
        projected_speed_mps: 0.2,
        projected_curvature_per_m: 0.0,
        window_end_speed_mps: 0.2,
        window_end_curvature_per_m: 0.0,
        held_curvature_per_m: 0.0,
        source_travel_time_s: 0.35,
        transition_horizon_s: 0.35,
        future_travel_time_s: 0.35,
    };
    let estimate = PoseEstimate {
        captured_at: at,
        frame_id: config.frame_id.clone(),
        pose,
        speed_mps: 0.2,
        yaw_rate_radps: 0.0,
        quality: 1.0,
    };
    let mut nav = Navigator::new(config).unwrap();
    nav.set_adoption_constraints(Some(constraints));
    (nav, estimate, constraints)
}

#[test]
fn open_candidates_still_pass_the_multisource_forecast_before_selection() {
    let (mut nav, estimate, constraints) = open_scene();
    let decision = nav
        .plan_with_arrival(
            estimate.captured_at,
            &estimate,
            &[],
            estimate.captured_at,
            Point2 { x_m: 7.0, y_m: 5.0 },
            None,
            nav.config.max_speed_mps,
            ArrivalBehavior::Stop,
        )
        .unwrap();
    assert_eq!(decision.status, NavigationStatus::Driving, "{decision:?}");
    assert_eq!(decision.diagnostics.candidates.rollouts_evaluated, 44);
    assert_eq!(decision.diagnostics.candidates.accepted, 44);
    let work = decision.diagnostics.admission_forecast_work;
    assert_eq!(work.adoption_scenarios, 132);
    assert!(work.source_checks > 0 && work.projection_calls > 0 && work.projection_intervals > 0);
    assert!(
        constraints
            .check_next_with_work(
                &nav.config,
                estimate.pose,
                decision.intent.speed_mps,
                decision.intent.curvature_per_m,
                &[],
                None,
            )
            .0
            .is_ok()
    );
}

#[test]
fn asynchronous_candidate_curvature_uses_actual_adoption_time() {
    // Both shorter and longer actual command intervals replace the unrelated
    // previous planning interval when preparing the asynchronous candidates.
    for (last_change, last_plan, expected_upper) in [(589, 580, 0.684), (400, 640, 1.44)] {
        let (mut nav, mut estimate, mut constraints) = open_scene();
        let at = Timestamp(660);
        estimate.captured_at = at;
        constraints.planned_at = at;
        constraints.last_command_change_at = Some(Timestamp(last_change));
        constraints.held_curvature_per_m = 0.4;
        constraints.projected_curvature_per_m = 0.4;
        constraints.window_end_curvature_per_m = 0.4;
        constraints.curvature_bound_per_m = 0.4;
        nav.set_adoption_constraints(Some(constraints));
        nav.set_execution_state(SteeringEstimate {
            at,
            commanded_curvature_per_m: 0.4,
            applied_curvature_per_m: 0.4,
        })
        .unwrap();
        nav.last_step = Some(Timestamp(last_plan));
        let goal = Point2 { x_m: 5.3, y_m: 7.0 };
        nav.route = Some((goal, None, vec![estimate.pose.point(), goal], 0));
        nav.pending_route_rebuild = None;
        nav.via_index = 1;
        let decision = nav
            .plan_with_arrival(
                at,
                &estimate,
                &[],
                at,
                goal,
                None,
                0.3,
                ArrivalBehavior::Stop,
            )
            .unwrap();
        assert_eq!(decision.status, NavigationStatus::Driving, "{decision:?}");
        let prepared = decision.diagnostics.slew_limited_curvature_per_m.unwrap();
        assert!((prepared - expected_upper).abs() < 1e-12, "{prepared}");
        assert!(decision.intent.curvature_per_m <= expected_upper + 1e-12);
    }
}

#[test]
fn adoption_contract_from_another_planning_time_fails_closed() {
    let (mut nav, estimate, mut constraints) = open_scene();
    constraints.planned_at = Timestamp(estimate.captured_at.0 + 1);
    nav.set_adoption_constraints(Some(constraints));
    let at = estimate.captured_at;
    let decision = nav
        .plan_with_arrival(
            at,
            &estimate,
            &[],
            at,
            Point2 { x_m: 7.0, y_m: 5.0 },
            None,
            0.3,
            ArrivalBehavior::Stop,
        )
        .unwrap();
    assert_eq!(decision.status, NavigationStatus::Blocked);
    assert_eq!(decision.reason.as_deref(), Some("invalid_adoption_time"));
    assert_eq!(decision.diagnostics.candidates.accepted, 0);
    assert!(decision.diagnostics.admission_forecast_work.is_empty());
}

#[test]
fn route_rebuild_cause_survives_failed_search_and_clears_after_success() {
    let (mut nav, mut estimate, _) = open_scene();
    nav.set_adoption_constraints(None);
    let goal = Point2 { x_m: 7.0, y_m: 5.0 };
    let mut plan = |nav: &mut Navigator, time: u64, goal: Point2| {
        let at = Timestamp(time);
        estimate.captured_at = at;
        nav.plan_with_arrival(
            at,
            &estimate,
            &[],
            at,
            goal,
            None,
            0.3,
            ArrivalBehavior::Stop,
        )
        .unwrap()
    };
    let initial = plan(&mut nav, 1000, goal);
    assert_eq!(initial.status, NavigationStatus::Driving);
    assert_eq!(
        initial.diagnostics.route_rebuild_reason,
        Some(RouteRebuildReason::Initial)
    );
    assert!(nav.pending_route_rebuild.is_none());
    let reused = plan(&mut nav, 1100, goal);
    assert!(reused.diagnostics.route_rebuild_reason.is_none());
    assert_eq!(
        reused.diagnostics.route_revision,
        initial.diagnostics.route_revision
    );
    let held = nav.plan_stop(Timestamp(1200)).unwrap();
    assert!(held.diagnostics.route_rebuild_reason.is_none());
    for at in [1300, 1400] {
        let failure = plan(
            &mut nav,
            at,
            Point2 {
                x_m: 11.0,
                y_m: 5.0,
            },
        );
        assert_eq!(failure.status, NavigationStatus::Blocked);
        assert_eq!(failure.reason.as_deref(), Some("no_grid_path"));
        assert_eq!(
            failure.diagnostics.route_rebuild_reason,
            Some(RouteRebuildReason::TaskHold)
        );
        assert_eq!(
            nav.pending_route_rebuild,
            Some(RouteRebuildReason::TaskHold)
        );
    }
    let rebuilt = plan(&mut nav, 1500, goal);
    assert_eq!(rebuilt.status, NavigationStatus::Driving);
    assert_eq!(
        rebuilt.diagnostics.route_rebuild_reason,
        Some(RouteRebuildReason::TaskHold)
    );
    assert!(nav.pending_route_rebuild.is_none());
    let reused = plan(&mut nav, 1600, goal);
    assert_eq!(reused.status, NavigationStatus::Driving);
    assert!(reused.diagnostics.route_rebuild_reason.is_none());
}

#[test]
fn returning_to_cached_goal_cancels_a_failed_goal_change_diagnostic() {
    let (mut nav, mut estimate, _) = open_scene();
    nav.set_adoption_constraints(None);
    let original_goal = Point2 { x_m: 7.0, y_m: 5.0 };
    let mut plan = |nav: &mut Navigator, time: u64, goal: Point2| {
        let at = Timestamp(time);
        estimate.captured_at = at;
        nav.plan_with_arrival(
            at,
            &estimate,
            &[],
            at,
            goal,
            None,
            0.3,
            ArrivalBehavior::Stop,
        )
        .unwrap()
    };
    let original = plan(&mut nav, 1000, original_goal);
    assert_eq!(original.status, NavigationStatus::Driving);
    let cached_route = nav.route.clone();
    let failed_change = plan(
        &mut nav,
        1100,
        Point2 {
            x_m: 11.0,
            y_m: 5.0,
        },
    );
    assert_eq!(failed_change.reason.as_deref(), Some("no_grid_path"));
    assert_eq!(
        failed_change.diagnostics.route_rebuild_reason,
        Some(RouteRebuildReason::GoalOrHeadingChanged)
    );
    assert_eq!(nav.route, cached_route);
    let returned = plan(&mut nav, 1200, original_goal);
    assert_eq!(returned.status, NavigationStatus::Driving);
    assert_eq!(
        returned.diagnostics.route_revision,
        original.diagnostics.route_revision
    );
    assert_eq!(nav.route, cached_route);
    assert!(returned.diagnostics.route_rebuild_reason.is_none());
    assert!(nav.pending_route_rebuild.is_none());
    nav.plan_stop(Timestamp(1300)).unwrap();
    let after_hold = plan(&mut nav, 1400, original_goal);
    assert_eq!(after_hold.status, NavigationStatus::Driving);
    assert_eq!(
        after_hold.diagnostics.route_rebuild_reason,
        Some(RouteRebuildReason::TaskHold)
    );
    assert_eq!(
        after_hold.diagnostics.route_revision,
        original.diagnostics.route_revision + 1
    );
    assert!(nav.pending_route_rebuild.is_none());
}

#[test]
fn route_rebuild_reports_the_actual_goal_boundary_arrival_and_drift_entry() {
    let (mut nav, mut estimate, _) = open_scene();
    nav.set_adoption_constraints(None);
    let goal = Point2 { x_m: 7.0, y_m: 5.0 };
    let mut next_time = 1000;
    let mut plan =
        |nav: &mut Navigator, estimate: &mut PoseEstimate, goal: Point2, heading, arrival| {
            let at = Timestamp(next_time);
            next_time += 100;
            estimate.captured_at = at;
            nav.plan_with_arrival(at, estimate, &[], at, goal, heading, 0.3, arrival)
                .unwrap()
        };
    assert_eq!(
        plan(&mut nav, &mut estimate, goal, None, ArrivalBehavior::Stop).status,
        NavigationStatus::Driving
    );
    let heading_changed = plan(
        &mut nav,
        &mut estimate,
        goal,
        Some(0.0),
        ArrivalBehavior::Stop,
    );
    assert_eq!(
        heading_changed.diagnostics.route_rebuild_reason,
        Some(RouteRebuildReason::GoalOrHeadingChanged)
    );
    nav.set_travel_boundary(Some(
        HalfPlane::new(Point2 { x_m: 9.0, y_m: 0.0 }, 0.0, 0.0).unwrap(),
    ));
    let boundary_changed = plan(
        &mut nav,
        &mut estimate,
        goal,
        Some(0.0),
        ArrivalBehavior::Stop,
    );
    assert_eq!(
        boundary_changed.diagnostics.route_rebuild_reason,
        Some(RouteRebuildReason::BoundaryChanged)
    );
    let through = ArrivalBehavior::PassThrough {
        next: Point2 { x_m: 8.0, y_m: 5.0 },
        next_heading_rad: Some(0.0),
        next_max_speed_mps: 0.18,
        admission_radius_m: 0.1,
    };
    let arrival_changed = plan(&mut nav, &mut estimate, goal, Some(0.0), through);
    assert_eq!(
        arrival_changed.diagnostics.route_rebuild_reason,
        Some(RouteRebuildReason::ArrivalChanged)
    );
    estimate.pose.y_m += 0.1;
    let drifted = plan(&mut nav, &mut estimate, goal, Some(0.0), through);
    assert_eq!(
        drifted.diagnostics.route_rebuild_reason,
        Some(RouteRebuildReason::FootprintReferenceDrift)
    );
}
