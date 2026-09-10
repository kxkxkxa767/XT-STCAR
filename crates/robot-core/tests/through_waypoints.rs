use xt_stcar_robot_core::autonomy::{Footprint, ObstacleDisc, Point2, Pose2, PoseEstimate, Rect};
use xt_stcar_robot_core::navigation::{
    ArrivalBehavior, NavigationConfig, NavigationDecision, NavigationStatus, Navigator,
};
use xt_stcar_robot_core::{FrameId, MotionOutput, Timestamp};

fn config() -> NavigationConfig {
    let mut config = NavigationConfig::simulation(
        Rect {
            min_x_m: 0.0,
            min_y_m: 0.0,
            max_x_m: 20.0,
            max_y_m: 20.0,
        },
        Footprint {
            front_m: 0.22,
            rear_m: 0.18,
            half_width_m: 0.13,
        },
        FrameId("map".into()),
    );
    config.control_period_ms = 100;
    config.goal_tolerance_m = 0.065;
    config.validate().unwrap();
    config
}

fn point(x_m: f64, y_m: f64) -> Point2 {
    Point2 { x_m, y_m }
}

fn estimate(at: u64, pose: Pose2, speed_mps: f64, curvature: f64) -> PoseEstimate {
    PoseEstimate {
        captured_at: Timestamp(at),
        frame_id: FrameId("map".into()),
        pose,
        speed_mps,
        yaw_rate_radps: speed_mps * curvature,
        quality: 1.0,
    }
}

fn initial_pose() -> Pose2 {
    Pose2 {
        x_m: 5.0,
        y_m: 5.0,
        yaw_rad: 0.0,
    }
}

fn plan(
    nav: &mut Navigator,
    at: u64,
    pose: Pose2,
    speed: f64,
    obstacles: &[ObstacleDisc],
    goal: Point2,
    arrival: ArrivalBehavior,
) -> NavigationDecision {
    nav.plan_with_arrival(
        Timestamp(at),
        &estimate(at, pose, speed, 0.0),
        obstacles,
        Timestamp(at),
        goal,
        None,
        0.3,
        arrival,
    )
    .unwrap()
}

#[test]
fn only_a_verified_continuation_relaxes_braking_at_a_required_waypoint() {
    let goal = point(5.066, 5.0);
    let next = point(6.066, 5.0);
    let through = ArrivalBehavior::PassThrough {
        next,
        next_heading_rad: None,
        next_max_speed_mps: 0.3,
    };
    let mut terminal = Navigator::new(config()).unwrap();
    let stop = plan(
        &mut terminal,
        0,
        initial_pose(),
        0.25,
        &[],
        goal,
        ArrivalBehavior::Stop,
    );
    assert_eq!(stop.status, NavigationStatus::Blocked);
    assert_eq!(stop.reason.as_deref(), Some("empty_speed_interval"));
    assert_eq!(stop.intent.speed_mps, 0.0);
    let lower = stop.diagnostics.speed_lower_mps.unwrap();
    assert!((lower - 0.19).abs() < 1e-12);
    assert!(stop.diagnostics.goal_speed_cap_mps.unwrap() < lower);
    assert!(!stop.diagnostics.continuation_checked);

    let mut passing = Navigator::new(config()).unwrap();
    let drive = plan(&mut passing, 0, initial_pose(), 0.25, &[], goal, through);
    assert_eq!(drive.status, NavigationStatus::Driving, "{drive:?}");
    assert!(drive.intent.speed_mps >= 0.19 - 1e-12);
    assert!(drive.diagnostics.continuation_checked);
    assert!(drive.diagnostics.checked_continuation_distance_m.unwrap() > 0.9);

    // An obstacle beyond the current waypoint leaves its approach free, but
    // makes the proposed next target unreachable. It supplies no braking room.
    let obstacle = ObstacleDisc {
        center: next,
        radius_m: 0.2,
    };
    let mut blocked = Navigator::new(config()).unwrap();
    let rejected = plan(
        &mut blocked,
        0,
        initial_pose(),
        0.25,
        &[obstacle],
        goal,
        through,
    );
    assert_eq!(rejected.status, NavigationStatus::Blocked);
    assert_eq!(rejected.reason.as_deref(), Some("empty_speed_interval"));
    assert!(!rejected.diagnostics.continuation_checked);
    assert_eq!(
        rejected.diagnostics.checked_continuation_distance_m,
        Some(0.0)
    );
    assert_eq!(rejected.intent.speed_mps, 0.0);
}

#[test]
fn a_new_obstacle_revokes_the_cached_continuations_braking_allowance() {
    let goal = point(5.066, 5.0);
    let next = point(6.066, 5.0);
    let through = ArrivalBehavior::PassThrough {
        next,
        next_heading_rad: None,
        next_max_speed_mps: 0.3,
    };
    let mut nav = Navigator::new(config()).unwrap();
    let original = plan(&mut nav, 0, initial_pose(), 0.25, &[], goal, through);
    assert_eq!(original.status, NavigationStatus::Driving);
    assert!(original.diagnostics.continuation_checked);
    let version = original.diagnostics.route_revision;
    // Inspect a fresh planning snapshot without adopting the first proposal.
    // This isolates invalidation of the existing cached route from actuator I/O.
    let obstructed = plan(
        &mut nav,
        100,
        initial_pose(),
        0.25,
        &[ObstacleDisc {
            center: next,
            radius_m: 0.2,
        }],
        goal,
        through,
    );
    assert_eq!(obstructed.diagnostics.route_revision, version);
    assert!(!obstructed.diagnostics.continuation_checked);
    assert_eq!(
        obstructed.diagnostics.checked_continuation_distance_m,
        Some(0.0)
    );
    assert!(
        obstructed.diagnostics.goal_speed_cap_mps.unwrap()
            < original.diagnostics.goal_speed_cap_mps.unwrap()
    );
    assert_eq!(obstructed.status, NavigationStatus::Blocked);
    assert_eq!(obstructed.reason.as_deref(), Some("empty_speed_interval"));
    assert_eq!(obstructed.intent.speed_mps, 0.0);
}

#[test]
fn a_slower_next_phase_is_commanded_before_waypoint_admission_without_abrupt_braking() {
    let cfg = config();
    let current_speed = 0.2;
    let next_limit = 0.18;
    let mut nav = Navigator::new(cfg.clone()).unwrap();
    let decision = plan(
        &mut nav,
        0,
        initial_pose(),
        current_speed,
        &[],
        point(5.07, 5.0),
        ArrivalBehavior::PassThrough {
            next: point(6.07, 5.0),
            next_heading_rad: None,
            next_max_speed_mps: next_limit,
        },
    );
    assert_eq!(decision.status, NavigationStatus::Driving, "{decision:?}");
    assert!(decision.diagnostics.continuation_checked);
    assert_eq!(
        decision.diagnostics.allowed_waypoint_speed_mps,
        Some(next_limit)
    );
    assert!(decision.intent.speed_mps <= next_limit + 1e-12);
    let normal_lower = current_speed - cfg.max_decel_mps2 * 0.1;
    assert!(decision.intent.speed_mps >= normal_lower - 1e-12);
    assert!(decision.intent.speed_mps > 0.0);
    assert!(decision.diagnostics.goal_speed_cap_mps.unwrap() <= next_limit + 1e-12);
}

#[test]
fn curved_continuation_cannot_skip_the_required_waypoints_admission_region() {
    let cfg = config();
    let mut nav = Navigator::new(cfg.clone()).unwrap();
    let goal = point(6.0, 6.0);
    let next = point(8.0, 6.0);
    let arrival = ArrivalBehavior::PassThrough {
        next,
        next_heading_rad: None,
        next_max_speed_mps: cfg.max_speed_mps,
    };
    let mut pose = initial_pose();
    let mut speed = 0.0_f64;
    let mut curvature = 0.0_f64;
    let mut closest = f64::INFINITY;
    let mut admitted = false;
    let mut checked_continuation = false;
    let mut last_reason = None;
    for tick in 0..1200 {
        let at = tick * cfg.control_period_ms;
        let distance = pose.point().distance(goal);
        closest = closest.min(distance);
        if distance <= cfg.goal_tolerance_m {
            assert!(
                speed > 0.02,
                "a through waypoint should not require standstill"
            );
            admitted = true;
            break;
        }
        assert!(
            pose.point().distance(next) > cfg.goal_tolerance_m,
            "arrived at the next target while skipping the required waypoint; closest={closest}"
        );
        let decision = nav
            .plan_with_arrival(
                Timestamp(at),
                &estimate(at, pose, speed, curvature),
                &[],
                Timestamp(at),
                goal,
                None,
                cfg.max_speed_mps,
                arrival,
            )
            .unwrap();
        checked_continuation |= decision.diagnostics.continuation_checked;
        last_reason = decision.reason.clone();
        let command = match decision.status {
            NavigationStatus::Driving => MotionOutput::Drive {
                speed_mps: decision.intent.speed_mps,
                curvature_per_m: decision.intent.curvature_per_m,
            },
            _ => MotionOutput::Stop,
        };
        nav.adopt_command(Timestamp(at), &command).unwrap();
        let (target_speed, target_curvature) = match command {
            MotionOutput::Drive {
                speed_mps,
                curvature_per_m,
            } => (speed_mps, curvature_per_m),
            MotionOutput::Stop => (0.0, 0.0),
        };
        // Independent 1 ms midpoint integration of the rate-limited bicycle
        // model: do not reuse navigation::integrate or rollout's step size.
        for _ in 0..100 {
            let dt = 0.001;
            let acceleration = if target_speed >= speed {
                cfg.max_accel_mps2
            } else {
                cfg.max_decel_mps2
            };
            let next_speed =
                speed + (target_speed - speed).clamp(-acceleration * dt, acceleration * dt);
            let next_curvature = curvature
                + (target_curvature - curvature).clamp(
                    -cfg.max_curvature_rate_per_s * dt,
                    cfg.max_curvature_rate_per_s * dt,
                );
            let midpoint_speed = 0.5 * (speed + next_speed);
            let midpoint_curvature = 0.5 * (curvature + next_curvature);
            let yaw_step = midpoint_speed * midpoint_curvature * dt;
            let midpoint_yaw = pose.yaw_rad + 0.5 * yaw_step;
            pose.x_m += midpoint_speed * midpoint_yaw.cos() * dt;
            pose.y_m += midpoint_speed * midpoint_yaw.sin() * dt;
            pose.yaw_rad += yaw_step;
            speed = next_speed;
            curvature = next_curvature;
            assert!(cfg.footprint.inside(pose, cfg.bounds));
        }
    }
    assert!(
        checked_continuation,
        "the test must exercise a verified curved continuation"
    );
    assert!(
        admitted,
        "missed required waypoint: pose={pose:?}, closest={closest}, last_reason={last_reason:?}"
    );
}
