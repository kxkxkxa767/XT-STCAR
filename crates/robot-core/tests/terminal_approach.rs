use xt_stcar_robot_core::autonomy::{HalfPlane, ObstacleDisc, Point2, Pose2, PoseEstimate};
use xt_stcar_robot_core::navigation::{
    ArrivalBehavior, NavigationConfig, NavigationStatus, Navigator,
};
use xt_stcar_robot_core::{MotionOutput, Timestamp};

fn competition_navigation() -> NavigationConfig {
    let json: serde_json::Value =
        serde_json::from_str(include_str!("../../../config/competition-sim.json")).unwrap();
    serde_json::from_value(json["autonomy"]["navigation"].clone()).unwrap()
}

#[test]
fn short_s_arrivals_keep_their_two_turns_in_both_directions() {
    for direction in [-1.0, 1.0] {
        let cfg = competition_navigation();
        let mut nav = Navigator::new(cfg.clone()).unwrap();
        // Mirror the recorded PP lamp approach at 66.8 s, before its cached S
        // became unreachable at 68.5 s. This test uses a separate 1 ms plant.
        let mut pose = Pose2 {
            x_m: 4.975272797,
            y_m: 2.5 + direction * 0.063358593,
            yaw_rad: direction * 0.09105425,
        };
        let goal = Point2 {
            x_m: 5.55,
            y_m: 2.5,
        };
        let mut speed = 0.18_f64;
        let mut curvature = direction * -1.29288;
        let mut adopted = nav.execution_state();
        adopted.applied_curvature_per_m = curvature;
        adopted.commanded_curvature_per_m = curvature;
        nav.set_execution_state(adopted).unwrap();
        let mut guarded_ticks = 0;
        let mut rejections = 0;
        let mut distance = 0.0;
        let mut minimum_curvature = 0.0_f64;
        let mut maximum_curvature = 0.0_f64;
        let mut reached = false;
        for tick in 0..120 {
            let at = Timestamp(tick * cfg.control_period_ms);
            let estimate = PoseEstimate {
                captured_at: at,
                frame_id: cfg.frame_id.clone(),
                pose,
                speed_mps: speed,
                yaw_rate_radps: speed * curvature,
                quality: 1.0,
            };
            let decision = nav
                .plan_with_arrival(
                    at,
                    &estimate,
                    &[],
                    at,
                    goal,
                    Some(0.0),
                    0.18,
                    ArrivalBehavior::Stop,
                )
                .unwrap();
            guarded_ticks += usize::from(decision.diagnostics.terminal_continuity_enforced);
            rejections += decision.diagnostics.candidates.terminal_unreachable;
            assert!(
                decision.diagnostics.terminal_connections_checked
                    <= 1 + 2 * (cfg.curvature_samples + 1)
            );
            assert!(
                decision.status != NavigationStatus::Blocked
                    || decision.reason.as_deref() == Some("goal_braking"),
                "direction={direction}, tick={tick}, pose={pose:?}, decision={decision:?}"
            );
            let command = if decision.status == NavigationStatus::Driving {
                assert!(decision.intent.speed_mps >= speed - cfg.max_decel_mps2 * 0.1 - 1e-12);
                assert!(decision.intent.speed_mps <= speed + cfg.max_accel_mps2 * 0.1 + 1e-12);
                assert!(
                    (decision.intent.curvature_per_m
                        - nav.execution_state().commanded_curvature_per_m)
                        .abs()
                        <= cfg.max_curvature_rate_per_s * 0.1 + 1e-12
                );
                MotionOutput::Drive {
                    speed_mps: decision.intent.speed_mps,
                    curvature_per_m: decision.intent.curvature_per_m,
                }
            } else {
                MotionOutput::Stop
            };
            nav.adopt_command(at, &command).unwrap();
            let (target_speed, target_curvature) = match command {
                MotionOutput::Drive {
                    speed_mps,
                    curvature_per_m,
                } => (speed_mps, curvature_per_m),
                MotionOutput::Stop => (0.0, 0.0),
            };
            // Faster actual acceleration/braking matches competition-sim.
            // Midpoint integration is independent of project_motion/rollout.
            for _ in 0..100 {
                let dt = 0.001;
                let rate = if target_speed >= speed { 0.6 } else { 0.8 };
                let next_speed = speed + (target_speed - speed).clamp(-rate * dt, rate * dt);
                let next_curvature = curvature
                    + (target_curvature - curvature).clamp(
                        -cfg.max_curvature_rate_per_s * dt,
                        cfg.max_curvature_rate_per_s * dt,
                    );
                let midpoint_speed = (speed + next_speed) * 0.5;
                let midpoint_curvature = (curvature + next_curvature) * 0.5;
                let yaw_step = midpoint_speed * midpoint_curvature * dt;
                let yaw = pose.yaw_rad + yaw_step * 0.5;
                pose.x_m += midpoint_speed * yaw.cos() * dt;
                pose.y_m += midpoint_speed * yaw.sin() * dt;
                pose.yaw_rad += yaw_step;
                speed = next_speed;
                curvature = next_curvature;
                distance += midpoint_speed * dt;
                minimum_curvature = minimum_curvature.min(curvature);
                maximum_curvature = maximum_curvature.max(curvature);
                assert!(cfg.footprint.inside(pose, cfg.bounds));
                assert!(speed.powi(2) * curvature.abs() <= cfg.max_lateral_accel_mps2 + 1e-12);
            }
            if decision.status == NavigationStatus::Reached && speed < 1e-12 {
                assert!(pose.point().distance(goal) <= cfg.goal_tolerance_m);
                assert!(pose.yaw_rad.abs() <= cfg.goal_heading_tolerance_rad);
                reached = true;
                break;
            }
        }
        assert!(
            reached,
            "direction={direction}, pose={pose:?}, distance={distance}"
        );
        assert!(
            distance < 0.7,
            "a short S must not become a long loop: {distance}"
        );
        assert!(
            guarded_ticks > 0 && rejections > 0,
            "guard was not exercised"
        );
        assert!(
            minimum_curvature < -0.1 && maximum_curvature > 0.1,
            "the actual plant must execute both S turns"
        );
    }
}

fn archived_navigation(
    at: u64,
    count: usize,
    direction: f64,
) -> (
    NavigationConfig,
    Navigator,
    PoseEstimate,
    Vec<ObstacleDisc>,
    Point2,
) {
    let raw: serde_json::Value = serde_json::from_str(include_str!(
        "../../../docs/motion-v5-original-input-window.json"
    ))
    .unwrap();
    let frame = raw["frames"]
        .as_array()
        .unwrap()
        .iter()
        .find(|frame| frame["at"] == at)
        .unwrap();
    let mut cfg = competition_navigation();
    cfg.curvature_samples = count;
    cfg.tracking = serde_json::from_value(serde_json::json!({
        "kind":"lqr", "q_lateral":4.0, "q_heading":2.0, "r_curvature":1.0,
        "min_speed_mps":0.03, "max_heading_error_rad":0.7, "max_lateral_error_m":0.5,
    }))
    .unwrap();
    let mut estimate: PoseEstimate =
        serde_json::from_value(frame["snapshot"]["pose"].clone()).unwrap();
    estimate.pose.y_m = 2.5 + direction * (estimate.pose.y_m - 2.5);
    estimate.pose.yaw_rad *= direction;
    estimate.yaw_rate_radps *= direction;
    let mut obstacles: Vec<ObstacleDisc> =
        serde_json::from_value(frame["world_obstacles"].clone()).unwrap();
    assert_eq!(obstacles.len(), 360);
    for obstacle in &mut obstacles {
        obstacle.center.y_m = 2.5 + direction * (obstacle.center.y_m - 2.5);
    }
    let goal = Point2 {
        x_m: 5.55,
        y_m: 2.5,
    };
    let mut nav = Navigator::new(cfg.clone()).unwrap();
    nav.set_travel_boundary(Some(HalfPlane::new(goal, 0.0, 0.45).unwrap()));
    let mut execution = nav.execution_state();
    execution.at = estimate.captured_at;
    let archived = &frame["control"]["navigation"]["diagnostics"]["execution_state"];
    execution.applied_curvature_per_m =
        direction * archived["applied_curvature_per_m"].as_f64().unwrap();
    execution.commanded_curvature_per_m =
        direction * archived["commanded_curvature_per_m"].as_f64().unwrap();
    nav.set_execution_state(execution).unwrap();
    (cfg, nav, estimate, obstacles, goal)
}

#[test]
fn original_lqr_42200_all_candidate_counts_recover_with_full_gates_on_both_sides() {
    for count in [21, 41, 61, 81] {
        for direction in [-1.0, 1.0] {
            let (cfg, mut nav, mut estimate, mut obstacles, goal) =
                archived_navigation(42200, count, direction);
            let execution = nav.execution_state();
            // Fresh Navigator route, exact archived pose/execution and all 360
            // world obstacles. No unaccounted budget, extra candidate or rollout.
            let result = nav
                .plan_with_arrival(
                    estimate.captured_at,
                    &estimate,
                    &obstacles,
                    estimate.captured_at,
                    goal,
                    Some(0.0),
                    0.18,
                    ArrivalBehavior::Stop,
                )
                .unwrap();
            assert_eq!(
                result.status,
                NavigationStatus::Driving,
                "count={count}, direction={direction}: {result:?}"
            );
            let d = result.diagnostics;
            let work = d.terminal_work;
            assert_eq!(work.continued_seed_accepted, 1);
            assert!(
                result.intent.speed_mps >= estimate.speed_mps - cfg.max_decel_mps2 * 0.1 - 1e-12
            );
            assert!(result.intent.speed_mps <= 0.18);
            assert!(
                (result.intent.curvature_per_m - execution.commanded_curvature_per_m).abs()
                    <= cfg.max_curvature_rate_per_s * 0.1 + 1e-12
            );
            assert!((result.intent.curvature_per_m - direction * 0.5975567493046594).abs() < 1e-12);
            assert_eq!(
                (work.solver_limit, work.iteration_limit, work.sample_limit),
                (256, 1024, 65536)
            );
            assert!(work.primitive_samples <= work.sample_limit);
            assert!(work.iterations <= work.iteration_limit);
            assert!(work.solver_attempts <= work.solver_limit);
            assert!(!work.budget_exhausted);
            assert_eq!(work.recovery_reservation_shortfalls, 0);
            assert!(work.recovery_reserved_samples > 0);
            assert_eq!(work.recovery_reserved_iterations, 8);
            assert_eq!(work.recovery_reserved_solvers, 1);
            assert_eq!(work.cold_budget_deferrals > 0, count > 21);
            assert_eq!(
                d.candidates.rollouts_evaluated,
                2 * (count + 1),
                "must not redo rollouts"
            );
            assert_eq!(
                d.candidates.accepted
                    + d.candidates.terminal_unreachable
                    + d.candidates.terminal_budget,
                2 * (count + 1)
            );
            assert!(d.selected_stopping_margin.unwrap().clearance_m > 0.0);
            assert_eq!(work.solver_elapsed_ns, None);
            println!(
                "count={count}, direction={direction}, intent={:?}, work={work:?}, candidates={:?}",
                result.intent, d.candidates
            );
            // A changed observation occupying the goal revokes all cached hints
            // and cannot be overcome by the reserved computational allowance.
            obstacles.push(ObstacleDisc {
                center: goal,
                radius_m: 0.2,
            });
            estimate.captured_at = Timestamp(42300);
            let blocked = nav
                .plan_with_arrival(
                    estimate.captured_at,
                    &estimate,
                    &obstacles,
                    estimate.captured_at,
                    goal,
                    Some(0.0),
                    0.18,
                    ArrivalBehavior::Stop,
                )
                .unwrap();
            assert_eq!(blocked.status, NavigationStatus::Blocked);
            assert_eq!(blocked.intent.speed_mps, 0.0);
        }
    }
}

#[test]
fn ordinary_best_remains_preferred_when_later_cold_work_is_deferred() {
    let (_, mut nav, estimate, obstacles, goal) = archived_navigation(42100, 81, 1.0);
    let result = nav
        .plan_with_arrival(
            estimate.captured_at,
            &estimate,
            &obstacles,
            estimate.captured_at,
            goal,
            Some(0.0),
            0.18,
            ArrivalBehavior::Stop,
        )
        .unwrap();
    assert_eq!(result.status, NavigationStatus::Driving);
    assert_eq!(result.intent.speed_mps, 0.18);
    assert!((result.intent.curvature_per_m - 0.5975567493046594).abs() < 1e-12);
    assert!(result.diagnostics.terminal_work.cold_budget_deferrals > 0);
    assert_eq!(result.diagnostics.terminal_work.continued_seed_attempts, 0);
    assert!(result.diagnostics.candidates.accepted > 1);
    assert!(!result.diagnostics.terminal_work.budget_exhausted);
}

#[test]
fn archived_lattice_route_work_stays_in_the_shared_ledger() {
    // In this real neighboring frame the initial short solve fails, lattice
    // finds the route, and the current-state short guard also fails. It does
    // not exercise recovery in the same tick; do not manufacture that premise.
    for count in [21, 41, 61, 81] {
        let (_, mut nav, estimate, obstacles, goal) = archived_navigation(42300, count, 1.0);
        let result = nav
            .plan_with_arrival(
                estimate.captured_at,
                &estimate,
                &obstacles,
                estimate.captured_at,
                goal,
                Some(0.0),
                0.18,
                ArrivalBehavior::Stop,
            )
            .unwrap();
        assert_eq!(result.status, NavigationStatus::Driving);
        assert_eq!(result.path.len(), 179);
        let work = result.diagnostics.terminal_work;
        assert_eq!(work.solver_attempts, 194);
        assert_eq!(work.iterations, 604);
        assert_eq!(work.primitive_samples, 18087);
        assert_eq!(work.primitive_grid_rejections, 77);
        assert_eq!(work.recovery_reserved_samples, 0);
        assert!(!result.diagnostics.terminal_continuity_enforced);
        assert_eq!(
            result.diagnostics.candidates.rollouts_evaluated,
            2 * (count + 1)
        );
        assert_eq!(result.diagnostics.candidates.accepted, 2 * (count + 1));
    }
}

#[test]
fn solver_wall_timing_is_opt_in_and_does_not_change_the_plan() {
    let mut reference = None;
    for enabled in [false, true] {
        let (_, mut nav, mut estimate, obstacles, goal) = archived_navigation(42200, 41, 1.0);
        nav.set_timing_enabled(enabled);
        for tick in 0..2 {
            estimate.captured_at = Timestamp(42200 + tick * 100);
            let result = nav
                .plan_with_arrival(
                    estimate.captured_at,
                    &estimate,
                    &obstacles,
                    estimate.captured_at,
                    goal,
                    Some(0.0),
                    0.18,
                    ArrivalBehavior::Stop,
                )
                .unwrap();
            assert_eq!(
                result.diagnostics.terminal_work.solver_elapsed_ns.is_some(),
                enabled
            );
            if enabled {
                assert!(result.diagnostics.terminal_work.solver_elapsed_ns.unwrap() > 0);
            }
            if tick == 0 {
                if let Some(intent) = reference {
                    assert_eq!(result.intent, intent);
                } else {
                    reference = Some(result.intent);
                }
            }
        }
        nav.set_timing_enabled(false);
        estimate.captured_at = Timestamp(42400);
        let result = nav
            .plan_with_arrival(
                estimate.captured_at,
                &estimate,
                &obstacles,
                estimate.captured_at,
                goal,
                Some(0.0),
                0.18,
                ArrivalBehavior::Stop,
            )
            .unwrap();
        assert_eq!(result.diagnostics.terminal_work.solver_elapsed_ns, None);
    }
}
