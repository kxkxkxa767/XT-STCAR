use xt_stcar_robot_core::autonomy::{Point2, Pose2, PoseEstimate};
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
