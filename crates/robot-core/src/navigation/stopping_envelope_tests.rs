use super::*;
use serde_json::{Value, from_value};

fn fixture() -> Value {
    serde_json::from_str(include_str!("fixtures/online29600_stopping_disk.json")).unwrap()
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

fn restore(input: &Value, policy: TargetPolicy) -> Navigator {
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

#[test]
fn real_disk_rejections_get_a_complete_certificate_without_adopting_steering() {
    let data = fixture();
    for index in [1, 2] {
        let input = &data["controller_inputs"][index];
        let mut nav = restore(input, TargetPolicy::RollingLocal);
        let mut steering = nav.execution_state();
        steering
            .advance_to(
                Timestamp(input["now"].as_u64().unwrap()),
                nav.config.max_curvature_rate_per_s,
            )
            .unwrap();
        let decision = replay(&mut nav, input);
        assert_eq!(decision.status, NavigationStatus::Driving, "{decision:?}");
        assert!(decision.intent.speed_mps > 0.12);
        assert!(decision.intent.curvature_per_m > 1.4);
        assert_eq!(decision.diagnostics.candidates.accepted, 44);
        assert_eq!(decision.diagnostics.candidates.stop_reachable, 0);
        assert!(
            decision
                .diagnostics
                .selected_stopping_margin
                .unwrap()
                .clearance_m
                < 0.0
        );
        assert!(
            decision
                .diagnostics
                .selected_braking_envelope_margin
                .unwrap()
                .clearance_m
                > 0.05
        );
        assert_eq!(nav.execution_state(), steering);
        assert!(decision.diagnostics.adoption_constraints.is_none());
        let work = decision.diagnostics.terminal_work;
        assert!(!work.budget_exhausted);
        assert!(
            work.solver_attempts <= 256
                && work.iterations <= 1024
                && work.primitive_samples <= 65_536
        );
        let search = decision.diagnostics.forward_search;
        assert!(
            search.ordinary.allocated_nodes + search.recovery.allocated_nodes
                <= nav.config.max_grid_cells
        );
    }
}

#[test]
fn fixed_and_successful_disk_recordings_keep_original_behavior() {
    let data = fixture();
    for index in 0..6 {
        let input = &data["controller_inputs"][index];
        let baseline = &data["baseline_plans"][index]["navigation"];
        let mut fixed = restore(input, TargetPolicy::Fixed);
        let decision = replay(&mut fixed, input);
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
            (decision.intent.curvature_per_m
                - baseline["intent"]["curvature_per_m"].as_f64().unwrap())
            .abs()
                < 1e-12
        );
        assert_eq!(
            serde_json::to_value(decision.diagnostics.candidates).unwrap(),
            baseline["diagnostics"]["candidates"]
        );
        assert!(
            decision
                .diagnostics
                .selected_braking_envelope_margin
                .is_none()
        );
        let encoded = serde_json::to_value(decision.diagnostics).unwrap();
        assert!(encoded.get("selected_braking_envelope_margin").is_none());
        if [0, 4].contains(&index) {
            let mut rolling = restore(input, TargetPolicy::RollingLocal);
            let rolling_decision = replay(&mut rolling, input);
            if index == 4 {
                // The later 36900 ms failure had the same finite-band capsule
                // cause as the independently captured 31400 ms fixture. Its
                // full swept-body boundary proof now permits the original
                // short connector; the Fixed baseline above stays unchanged.
                assert_eq!(rolling_decision.status, NavigationStatus::Driving);
                assert!(rolling_decision.diagnostics.remaining_distance_m.unwrap() < 0.6);
                assert!(rolling_decision.intent.speed_mps > 0.0);
                let work = rolling_decision.diagnostics.terminal_work;
                assert!(!work.budget_exhausted);
                assert!(
                    work.solver_attempts <= 256
                        && work.iterations <= 1024
                        && work.primitive_samples <= 65_536
                );
                continue;
            }
            assert_eq!(rolling_decision.status, decision.status);
            assert_eq!(rolling_decision.reason, decision.reason);
            assert_eq!(rolling_decision.intent, decision.intent);
            assert_eq!(rolling_decision.path, decision.path);
            assert!(
                rolling_decision
                    .diagnostics
                    .selected_braking_envelope_margin
                    .is_none()
            );
        }
    }
}

#[test]
fn full_rectangle_contact_boundary_history_and_invalid_bounds_fail_closed() {
    let data = fixture();
    let input = &data["controller_inputs"][1];
    let nav = restore(input, TargetPolicy::RollingLocal);
    let estimate: PoseEstimate = from_value(input["estimate"].clone()).unwrap();
    let obstacles: Vec<ObstacleDisc> = from_value(input["obstacles"].clone()).unwrap();
    let speed = input["speed_limit_mps"].as_f64().unwrap();
    assert!(
        certify(
            &nav.config,
            estimate.pose,
            speed,
            &obstacles,
            nav.travel_boundary
        )
        .is_some()
    );
    let envelope = StoppingEnvelope::new(
        &nav.config,
        estimate.pose,
        speed,
        nav.config.max_curvature_per_m,
        0.2,
    )
    .unwrap();
    // Front corner contact, not the center, invalidates the full certificate.
    let point = estimate.pose.body_to_world(Point2 {
        x_m: envelope.body.max_x_m,
        y_m: envelope.body.max_y_m,
    });
    let contact = ObstacleDisc {
        center: point,
        radius_m: 0.001,
    };
    assert!(certify(&nav.config, estimate.pose, speed, &[contact], None).is_none());
    let boundary = HalfPlane::new(estimate.pose.point(), estimate.pose.yaw_rad, 0.28).unwrap();
    assert!(boundary.contains_footprint(
        nav.config.footprint,
        estimate.pose,
        nav.config.clearance_m
    ));
    assert!(certify(&nav.config, estimate.pose, speed, &[], Some(boundary)).is_none());
    let mut bounds = nav.config.clone();
    bounds.bounds.max_x_m = envelope
        .corners()
        .into_iter()
        .map(|p| estimate.pose.body_to_world(p).x_m)
        .fold(f64::NEG_INFINITY, f64::max)
        - 0.001;
    assert!(certify(&bounds, estimate.pose, speed, &[], None).is_none());
    for invalid in [
        -0.01,
        f64::NAN,
        f64::INFINITY,
        nav.config.max_speed_mps + 0.01,
    ] {
        assert!(
            certify(
                &nav.config,
                estimate.pose,
                invalid,
                &obstacles,
                nav.travel_boundary
            )
            .is_none()
        );
    }
    let grid = Grid::with_boundary(&nav.config, &obstacles, nav.travel_boundary);
    grid.enable_recovery();
    let solve = |policy, historical| {
        rollout_with_stopping_policy(
            &nav.config,
            estimate.pose,
            estimate.speed_mps,
            speed,
            1.4926190289578993,
            1.4926190289578993,
            &obstacles,
            0.5763620881318252,
            &grid,
            None,
            policy,
            historical,
        )
    };
    assert!(solve(TargetPolicy::RollingLocal, 0.0).is_ok());
    assert_eq!(
        solve(TargetPolicy::Fixed, 0.0).unwrap_err(),
        RolloutRejection::StopReachable
    );
    assert_eq!(
        solve(TargetPolicy::RollingLocal, nav.config.max_speed_mps).unwrap_err(),
        RolloutRejection::StopReachable
    );
    assert_eq!(
        solve(TargetPolicy::RollingLocal, f64::NAN).unwrap_err(),
        RolloutRejection::StopReachable
    );
}

#[test]
fn envelope_contains_arbitrary_signed_curvature_during_both_periods_and_full_braking() {
    let data = fixture();
    let nav = restore(&data["controller_inputs"][1], TargetPolicy::RollingLocal);
    let config = &nav.config;
    let period = config.control_period_ms as f64 / 1000.0;
    for speed_bound in [0.04, 0.14, 0.3, config.max_speed_mps] {
        for yaw in [0.0, 0.3, 1.27, -2.8] {
            let source = Pose2 {
                x_m: 3.0,
                y_m: 2.0,
                yaw_rad: yaw,
            };
            let envelope = StoppingEnvelope::new(
                config,
                source,
                speed_bound,
                config.max_curvature_per_m,
                2.0 * period,
            )
            .unwrap();
            for seed in 0..32 {
                let mut pose = source;
                let mut speed = speed_bound;
                let mut time = 0.0;
                let mut step = 0;
                while time < 2.0 * period + speed_bound / config.max_decel_mps2 {
                    let dt: f64 = 0.001;
                    let (next_speed, distance) = if time < 2.0 * period {
                        (speed, speed * dt.min(2.0 * period - time))
                    } else {
                        ramp_integral(speed, 0.0, config.max_decel_mps2, dt)
                    };
                    // Deliberately includes instantaneous sign changes: this
                    // larger set also contains the original slew-limited stop.
                    let sign = if seed == 0 {
                        -1.0
                    } else if seed == 1 {
                        1.0
                    } else {
                        ((seed * 17 + step * 31) % 19) as f64 / 9.0 - 1.0
                    };
                    pose = integrate(pose, distance, sign * config.max_curvature_per_m);
                    speed = next_speed;
                    time += dt;
                    step += 1;
                    for corner in config.footprint.corners(pose) {
                        let local = source.world_to_body(corner);
                        assert!(
                            envelope.body.contains(local),
                            "v={speed_bound} yaw={yaw} seed={seed} t={time} point={local:?} envelope={envelope:?}"
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn later_recorded_disk_case_rechecks_model_sweep_and_original_error_budget() {
    let data = fixture();
    let input = &data["controller_inputs"][5];
    let mut nav = restore(input, TargetPolicy::RollingLocal);
    let actual: PoseEstimate = from_value(input["estimate"].clone()).unwrap();
    // This frame followed an earlier Stop but had already restarted. Preserve
    // its actual .04 m/s and adopted .4/m command rather than invent standstill.
    assert!((actual.speed_mps - 0.04).abs() < 1e-12);
    let mut steering = nav.execution_state();
    steering
        .advance_to(actual.captured_at, nav.config.max_curvature_rate_per_s)
        .unwrap();
    assert!((steering.applied_curvature_per_m - 0.4).abs() < 1e-12);
    let decision = replay(&mut nav, input);
    eprintln!(
        "LATER_MODEL_PROOF status={:?} remaining={:?} intent={:?} nodes={:?} work={:?}",
        decision.status,
        decision.diagnostics.remaining_distance_m,
        decision.intent,
        decision.diagnostics.forward_search,
        decision.diagnostics.terminal_work
    );
    assert_eq!(decision.status, NavigationStatus::Driving);
    assert!(decision.diagnostics.remaining_distance_m.unwrap() < 0.6);
    assert_eq!(nav.execution_state(), steering);
    let work = decision.diagnostics.terminal_work;
    assert!(!work.budget_exhausted);
    assert!(
        work.solver_attempts <= 256 && work.iterations <= 1024 && work.primitive_samples <= 65_536
    );
    assert!(
        decision.diagnostics.forward_search.ordinary.allocated_nodes
            + decision.diagnostics.forward_search.recovery.allocated_nodes
            <= nav.config.max_grid_cells
    );
    let obstacles: Vec<ObstacleDisc> = from_value(input["obstacles"].clone()).unwrap();
    let mut domain = Grid::with_boundary(&nav.config, &obstacles, nav.travel_boundary);
    domain.recovery.oriented_boundary = true;
    domain.enable_recovery();
    let start: Pose2 = from_value(input["estimate"]["pose"].clone()).unwrap();
    let goal: Point2 = from_value(input["goal"].clone()).unwrap();
    let heading = input["heading"].as_f64().unwrap();
    let budget = TerminalBudget::default();
    let connection = terminal_connection(
        &nav.config,
        &budget,
        None,
        nav.last_step,
        start,
        nav.execution_state().applied_curvature_per_m,
        goal,
        heading,
        &domain,
        primitive_envelope::ErrorBound::default(),
    );
    let (connection, _) = connection.expect("actual short model connection");
    assert!(connection.error.position_m > 0.0 && connection.error.heading_rad > 0.0);
    assert!(
        connection.endpoint.point().distance(goal) + connection.error.position_m
            < nav.config.goal_tolerance_m * 0.5
    );
    assert!(
        angle_error(connection.endpoint.yaw_rad, heading).abs() + connection.error.heading_rad
            < nav.config.goal_heading_tolerance_rad * 0.5
    );
    // Independently replay both returned steering pieces through the complete
    // current grid/body domain, carrying the first piece's quadrature error.
    let seed = serde_json::to_value(connection.seed).unwrap();
    let variables = seed["variables"].as_array().unwrap();
    let length = variables[2].as_f64().unwrap();
    let split = seed["first_fraction"].as_f64().unwrap();
    let first = car_primitive_with_error(
        start,
        steering.applied_curvature_per_m,
        variables[0].as_f64().unwrap(),
        length * split,
        &nav.config,
        &domain,
        primitive_envelope::ErrorBound::default(),
    )
    .unwrap();
    let second = car_primitive_with_error(
        first.1,
        first.2,
        variables[1].as_f64().unwrap(),
        length * (1.0 - split),
        &nav.config,
        &domain,
        first.3,
    )
    .unwrap();
    assert!(second.1.point().distance(connection.endpoint.point()) < 1e-12);
    assert!(second.3.position_m >= first.3.position_m);
    assert!(second.3.heading_rad >= first.3.heading_rad);
    assert!((second.3.position_m - connection.error.position_m).abs() < 1e-12);
    let margin = nav.config.clearance_m
        + second.3.position_m
        + body_radius(nav.config.footprint) * second.3.heading_rad.min(2.0);
    assert!(pose_clear(&nav.config, second.1, &obstacles, margin));
    assert!(domain.footprint_inside_boundary(nav.config.footprint, second.1, margin));
    let endpoint_corner = nav.config.footprint.corners(connection.endpoint)[0];
    let mut blocked = Grid::with_boundary(
        &nav.config,
        &[ObstacleDisc {
            center: endpoint_corner,
            radius_m: 0.01,
        }],
        nav.travel_boundary,
    );
    blocked.recovery.oriented_boundary = true;
    blocked.enable_recovery();
    assert!(
        terminal_connection(
            &nav.config,
            &TerminalBudget::default(),
            None,
            nav.last_step,
            start,
            steering.applied_curvature_per_m,
            goal,
            heading,
            &blocked,
            primitive_envelope::ErrorBound::default()
        )
        .is_none()
    );
    assert!(
        terminal_connection(
            &nav.config,
            &TerminalBudget::default(),
            None,
            nav.last_step,
            start,
            steering.applied_curvature_per_m,
            goal,
            heading,
            &domain,
            primitive_envelope::ErrorBound {
                position_m: 0.03,
                heading_rad: 0.1
            }
        )
        .is_none()
    );
}
