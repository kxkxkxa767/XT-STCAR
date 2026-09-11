use xt_stcar_robot_core::autonomy::{Footprint, ObstacleDisc, Point2, Pose2, PoseEstimate, Rect};
use xt_stcar_robot_core::navigation::{NavigationConfig, NavigationStatus, Navigator, integrate};
use xt_stcar_robot_core::{FrameId, Timestamp};

fn config() -> NavigationConfig {
    NavigationConfig::simulation(
        Rect {
            min_x_m: 0.0,
            min_y_m: 0.0,
            max_x_m: 7.0,
            max_y_m: 5.0,
        },
        Footprint {
            front_m: 0.22,
            rear_m: 0.18,
            half_width_m: 0.13,
        },
        FrameId("map".into()),
    )
}
fn estimate(t: u64, pose: Pose2, speed_mps: f64) -> PoseEstimate {
    PoseEstimate {
        captured_at: Timestamp(t),
        frame_id: FrameId("map".into()),
        pose,
        speed_mps,
        yaw_rate_radps: 0.0,
        quality: 1.0,
    }
}
fn point(x_m: f64, y_m: f64) -> Point2 {
    Point2 { x_m, y_m }
}

#[test]
fn old_navigation_json_preserves_pure_pursuit_and_rejects_unknown_trackers() {
    use xt_stcar_robot_core::tracking::TrackingConfig;
    let mut old = serde_json::to_value(config()).unwrap();
    old.as_object_mut().unwrap().remove("tracking");
    let decoded: NavigationConfig = serde_json::from_value(old.clone()).unwrap();
    assert!(matches!(decoded.tracking, TrackingConfig::PurePursuit));
    decoded.validate().unwrap();
    old["tracking"] = serde_json::json!({"kind": "unknown"});
    assert!(serde_json::from_value::<NavigationConfig>(old).is_err());
}

#[test]
fn negligible_negative_speed_roundoff_preserves_the_forward_tracking_gate() {
    let mut nav = Navigator::new(config()).unwrap();
    let decision = nav
        .step(
            Timestamp(0),
            &estimate(0, pose(1.0, 2.5), -0.000_000_1),
            &[],
            Timestamp(0),
            point(3.0, 2.5),
        )
        .unwrap();
    assert_eq!(decision.status, NavigationStatus::Driving);
    assert!(decision.intent.speed_mps > 0.0);
}
fn pose(x_m: f64, y_m: f64) -> Pose2 {
    Pose2 {
        x_m,
        y_m,
        yaw_rad: 0.0,
    }
}

#[test]
fn closed_loop_drives_around_two_cones_and_stops_at_goal() {
    let cfg = config();
    let mut nav = Navigator::new(cfg.clone()).unwrap();
    let obstacles = [
        ObstacleDisc {
            center: point(2.5, 2.5),
            radius_m: 0.15,
        },
        ObstacleDisc {
            center: point(4.3, 2.5),
            radius_m: 0.15,
        },
    ];
    let goal = point(6.3, 2.5);
    let mut actual = pose(0.6, 2.5);
    let mut speed = 0.0;
    let mut curvature = 0.0;
    let mut last_command_curvature = 0.0;
    let mut deviation = 0.0_f64;
    let mut reached = false;
    let mut consecutive_stops = 0;
    for step in 0..1800 {
        let t = step * 50;
        let decision = nav
            .step(
                Timestamp(t),
                &estimate(t, actual, speed),
                &obstacles,
                Timestamp(t),
                goal,
            )
            .unwrap();
        if decision.status == NavigationStatus::Reached {
            assert_eq!(decision.intent.speed_mps, 0.0);
            reached = true;
            break;
        }
        let motion = decision.intent;
        if decision.status == NavigationStatus::Blocked {
            consecutive_stops += 1;
            assert!(
                consecutive_stops < 40,
                "step={step} pose={actual:?} speed={speed} reason={:?}",
                decision.reason
            );
            assert_eq!(motion.speed_mps, 0.0);
            speed = (speed - cfg.max_decel_mps2 * 0.05).max(0.0);
            last_command_curvature = 0.0;
        } else {
            consecutive_stops = 0;
            assert!(motion.speed_mps <= cfg.max_speed_mps + 1e-9);
            assert!(motion.speed_mps - speed <= cfg.max_accel_mps2 * 0.05 + 1e-9);
            assert!(speed - motion.speed_mps <= cfg.max_decel_mps2 * 0.05 + 1e-9);
            assert!(motion.curvature_per_m.abs() <= cfg.max_curvature_per_m + 1e-9);
            assert!(
                (motion.curvature_per_m - last_command_curvature).abs()
                    <= cfg.max_curvature_rate_per_s * 0.05 + 1e-9
            );
            assert!(
                motion.speed_mps.powi(2) * motion.curvature_per_m.abs()
                    <= cfg.max_lateral_accel_mps2 + 1e-9
            );
            speed = motion.speed_mps;
            last_command_curvature = motion.curvature_per_m;
        }
        curvature += (motion.curvature_per_m - curvature).clamp(
            -cfg.max_curvature_rate_per_s * 0.05,
            cfg.max_curvature_rate_per_s * 0.05,
        );
        actual = integrate(actual, speed * 0.05, curvature);
        assert!(cfg.footprint.inside(actual, cfg.bounds));
        for obstacle in obstacles {
            let local = actual.world_to_body(obstacle.center);
            let dx = (local.x_m
                - local
                    .x_m
                    .clamp(-cfg.footprint.rear_m, cfg.footprint.front_m))
            .abs();
            let dy = (local.y_m
                - local
                    .y_m
                    .clamp(-cfg.footprint.half_width_m, cfg.footprint.half_width_m))
            .abs();
            assert!(dx.hypot(dy) > obstacle.radius_m + cfg.clearance_m);
        }
        deviation = deviation.max((actual.y_m - 2.5).abs());
    }
    assert!(reached, "failed to reach: {actual:?}, speed={speed}");
    assert!(actual.point().distance(goal) <= cfg.goal_tolerance_m);
    assert!(deviation > 0.35, "cones must cause a physical detour");
}

#[test]
fn sealed_field_and_goal_behind_car_stop_without_rotation() {
    let mut nav = Navigator::new(config()).unwrap();
    let wall: Vec<_> = (0..=10)
        .map(|i| ObstacleDisc {
            center: point(3.0, f64::from(i) * 0.5),
            radius_m: 0.26,
        })
        .collect();
    let e = estimate(0, pose(1.0, 2.5), 0.0);
    let d = nav
        .step(Timestamp(0), &e, &wall, Timestamp(0), point(6.0, 2.5))
        .unwrap();
    assert_eq!(d.status, NavigationStatus::Blocked);
    assert_eq!(d.reason.as_deref(), Some("no_grid_path"));
    assert_eq!(d.intent.speed_mps, 0.0);
    let d = nav
        .step(Timestamp(0), &e, &[], Timestamp(0), point(0.5, 2.5))
        .unwrap();
    assert_eq!(d.status, NavigationStatus::Blocked);
    assert_eq!(integrate(e.pose, 0.0, 2.0), e.pose);
}

#[test]
fn braking_envelope_blocks_a_new_front_obstacle() {
    let mut cfg = config();
    cfg.preview_horizon_s = 0.01; // braking envelope still takes precedence.
    cfg.max_decel_mps2 = 0.1;
    cfg.max_curvature_per_m = 10.0;
    cfg.max_curvature_rate_per_s = 50.0;
    let mut nav = Navigator::new(cfg).unwrap();
    let obstacle = ObstacleDisc {
        center: point(1.62, 2.5),
        radius_m: 0.05,
    };
    let d = nav
        .step(
            Timestamp(0),
            &estimate(0, pose(1.0, 2.5), 0.3),
            &[obstacle],
            Timestamp(0),
            point(6.0, 2.5),
        )
        .unwrap();
    assert_eq!(d.status, NavigationStatus::Blocked);
    assert_eq!(
        d.reason.as_deref(),
        Some("no_collision_free_braking_trajectory")
    );
    assert_eq!(d.intent.speed_mps, 0.0);
}

#[test]
fn full_footprint_and_stale_inputs_stop() {
    let mut nav = Navigator::new(config()).unwrap();
    let mut e = estimate(0, pose(0.1, 1.0), 0.0);
    let goal = point(6.0, 2.5);
    let d = nav.step(Timestamp(0), &e, &[], Timestamp(0), goal).unwrap();
    assert_eq!(
        d.reason.as_deref(),
        Some("current_footprint_collision_or_boundary")
    );
    e.pose = pose(1.0, 2.5);
    let d = nav
        .step(Timestamp(250), &e, &[], Timestamp(250), goal)
        .unwrap();
    assert_eq!(d.status, NavigationStatus::Blocked);
    e.captured_at = Timestamp(300);
    let d = nav
        .step(Timestamp(300), &e, &[], Timestamp(0), goal)
        .unwrap();
    assert_eq!(d.status, NavigationStatus::Blocked);
    assert!(
        nav.step(Timestamp(200), &e, &[], Timestamp(0), goal)
            .is_err()
    );
}

#[test]
fn invalid_frames_geometry_and_bounded_resources_are_rejected() {
    let mut cfg = config();
    cfg.max_grid_cells = 100;
    assert!(Navigator::new(cfg).is_err());
    let mut nav = Navigator::new(config()).unwrap();
    let mut e = estimate(0, pose(1.0, 2.5), 0.0);
    e.frame_id = FrameId("laser".into());
    assert!(
        nav.step(Timestamp(0), &e, &[], Timestamp(0), point(6.0, 2.5))
            .is_err()
    );
    e.frame_id = FrameId("map".into());
    e.pose.yaw_rad = f64::NAN;
    assert!(
        nav.step(Timestamp(0), &e, &[], Timestamp(0), point(6.0, 2.5))
            .is_err()
    );
    e.pose.yaw_rad = 0.0;
    assert!(
        nav.step(
            Timestamp(0),
            &e,
            &[ObstacleDisc {
                center: point(2.0, 2.0),
                radius_m: -1.0
            }],
            Timestamp(0),
            point(6.0, 2.5)
        )
        .is_err()
    );
    assert!(
        nav.step(Timestamp(0), &e, &[], Timestamp(1), point(6.0, 2.5))
            .is_err()
    );
}

#[test]
fn temporary_limit_and_task_hold_resume_with_bounded_acceleration() {
    let mut nav = Navigator::new(config()).unwrap();
    let e = estimate(0, pose(1.0, 2.5), 0.0);
    let d = nav
        .step_with_speed_limit(Timestamp(0), &e, &[], Timestamp(0), point(6.0, 2.5), 0.01)
        .unwrap();
    assert!(d.intent.speed_mps <= 0.01);
    assert_eq!(nav.stop(Timestamp(50)).unwrap().intent.speed_mps, 0.0);
    let e = estimate(10000, pose(1.0, 2.5), 0.0);
    let d = nav
        .step(Timestamp(10000), &e, &[], Timestamp(10000), point(6.0, 2.5))
        .unwrap();
    assert_eq!(d.status, NavigationStatus::Driving);
    assert!(d.intent.speed_mps <= 0.02 + 1e-9);
}

// A moving ray origin observes changing surface points, as a real scanner does.
fn surface_obstacles(actual: Pose2, cones: &[ObstacleDisc]) -> Vec<ObstacleDisc> {
    let mut obstacles = cones.to_vec();
    for index in 0..360 {
        let angle = actual.yaw_rad - f64::from(index) * std::f64::consts::TAU / 360.0;
        let (dy, dx) = angle.sin_cos();
        let mut distance = 12.0_f64;
        for (boundary, coordinate, direction) in [
            (0.0, actual.x_m, dx),
            (7.0, actual.x_m, dx),
            (0.0, actual.y_m, dy),
            (5.0, actual.y_m, dy),
        ] {
            if direction.abs() > 1e-9 {
                let hit = (boundary - coordinate) / direction;
                if hit > 0.0 {
                    distance = distance.min(hit);
                }
            }
        }
        for cone in cones {
            let ox = actual.x_m - cone.center.x_m;
            let oy = actual.y_m - cone.center.y_m;
            let b = ox * dx + oy * dy;
            let discriminant = b * b - (ox * ox + oy * oy - cone.radius_m.powi(2));
            if discriminant >= 0.0 {
                let hit = -b - discriminant.sqrt();
                if hit > 0.0 {
                    distance = distance.min(hit);
                }
            }
        }
        obstacles.push(ObstacleDisc {
            center: point(actual.x_m + distance * dx, actual.y_m + distance * dy),
            radius_m: 0.015,
        });
    }
    obstacles
}

#[test]
fn changing_surface_cloud_and_successive_turning_waypoints_remain_navigable() {
    let mut cfg = config();
    cfg.control_period_ms = 100;
    cfg.goal_tolerance_m = 0.045;
    let mut nav = Navigator::new(cfg.clone()).unwrap();
    let cones = [
        ObstacleDisc {
            center: point(3.0, 2.5),
            radius_m: 0.14,
        },
        ObstacleDisc {
            center: point(4.25, 2.85),
            radius_m: 0.14,
        },
    ];
    let goals = [
        point(3.1, 3.35),
        point(4.5, 1.85),
        point(5.55, 2.5),
        point(6.43, 2.5),
    ];
    let mut actual = Pose2 {
        x_m: 1.3985,
        y_m: 2.50156,
        yaw_rad: -0.07975,
    };
    let mut speed = 0.0;
    let mut waypoint = 0;
    let mut curvature = 0.0;
    let mut consecutive_stops = 0;
    for step in 0..2000 {
        let t = step * 100;
        let obstacles = surface_obstacles(actual, &cones);
        let d = nav
            .step_with_goal_heading(
                Timestamp(t),
                &estimate(t, actual, speed),
                &obstacles,
                Timestamp(t),
                goals[waypoint],
                (waypoint >= 2).then_some(0.0),
                cfg.max_speed_mps,
            )
            .unwrap();
        if d.status == NavigationStatus::Reached {
            if waypoint >= 2 {
                let yaw_error = (actual.yaw_rad + std::f64::consts::PI)
                    .rem_euclid(std::f64::consts::TAU)
                    - std::f64::consts::PI;
                assert!(yaw_error.abs() <= cfg.goal_heading_tolerance_rad);
            }
            waypoint += 1;
            speed = 0.0;
            if waypoint == goals.len() {
                break;
            }
            continue;
        }
        if d.status == NavigationStatus::Blocked {
            consecutive_stops += 1;
            assert!(
                consecutive_stops < 20,
                "step={step}, waypoint={waypoint}, pose={actual:?}, speed={speed}, reason={:?}",
                d.reason
            );
            assert_eq!(d.intent.speed_mps, 0.0);
        } else {
            consecutive_stops = 0;
        }
        // The former endpoint-Euler plant jumped immediately to target speed
        // and applied the final steering for the whole period. Match the
        // declared continuous actuator rates with an independent 1 ms midpoint
        // plant; do not call navigation::integrate or project_motion here.
        for _ in 0..100 {
            let dt = 0.001;
            let acceleration = if d.intent.speed_mps >= speed {
                cfg.max_accel_mps2
            } else {
                cfg.max_decel_mps2
            };
            let next_speed =
                speed + (d.intent.speed_mps - speed).clamp(-acceleration * dt, acceleration * dt);
            let next_curvature = curvature
                + (d.intent.curvature_per_m - curvature).clamp(
                    -cfg.max_curvature_rate_per_s * dt,
                    cfg.max_curvature_rate_per_s * dt,
                );
            let midpoint_speed = (speed + next_speed) * 0.5;
            let midpoint_curvature = (curvature + next_curvature) * 0.5;
            let yaw_step = midpoint_speed * midpoint_curvature * dt;
            let midpoint_yaw = actual.yaw_rad + yaw_step * 0.5;
            actual.x_m += midpoint_speed * midpoint_yaw.cos() * dt;
            actual.y_m += midpoint_speed * midpoint_yaw.sin() * dt;
            actual.yaw_rad += yaw_step;
            speed = next_speed;
            curvature = next_curvature;
            assert!(cfg.footprint.inside(actual, cfg.bounds));
            for cone in cones {
                let p = actual.world_to_body(cone.center);
                assert!(
                    (p.x_m - p.x_m.clamp(-cfg.footprint.rear_m, cfg.footprint.front_m)).hypot(
                        p.y_m
                            - p.y_m
                                .clamp(-cfg.footprint.half_width_m, cfg.footprint.half_width_m)
                    ) > cone.radius_m + cfg.clearance_m
                );
            }
        }
    }
    assert_eq!(waypoint, goals.len(), "last pose={actual:?}");
}

#[test]
fn position_alone_cannot_satisfy_an_explicit_goal_heading() {
    let mut nav = Navigator::new(config()).unwrap();
    let e = estimate(
        0,
        Pose2 {
            x_m: 3.0,
            y_m: 2.5,
            yaw_rad: 1.57,
        },
        0.0,
    );
    let d = nav
        .step_with_goal_heading(
            Timestamp(0),
            &e,
            &[],
            Timestamp(0),
            e.pose.point(),
            Some(0.0),
            0.3,
        )
        .unwrap();
    assert_ne!(d.status, NavigationStatus::Reached);
    assert_eq!(integrate(e.pose, 0.0, d.intent.curvature_per_m), e.pose);
    assert!(
        nav.step_with_goal_heading(
            Timestamp(0),
            &e,
            &[],
            Timestamp(0),
            e.pose.point(),
            Some(f64::NAN),
            0.3
        )
        .is_err()
    );
}

#[test]
fn grid_path_cannot_cut_between_touching_occupied_corners() {
    let mut cfg = config();
    cfg.bounds = Rect {
        min_x_m: 0.0,
        min_y_m: 0.0,
        max_x_m: 1.0,
        max_y_m: 1.0,
    };
    cfg.footprint = Footprint {
        front_m: 0.01,
        rear_m: 0.01,
        half_width_m: 0.01,
    };
    cfg.clearance_m = 0.005;
    let nav = Navigator::new(cfg).unwrap();
    let obstacles = [
        ObstacleDisc {
            center: point(0.45, 0.35),
            radius_m: 0.01,
        },
        ObstacleDisc {
            center: point(0.35, 0.45),
            radius_m: 0.01,
        },
    ];
    let path = nav
        .plan_path(point(0.35, 0.35), point(0.45, 0.45), &obstacles)
        .unwrap();
    // A diagonal-only planner would return the two endpoints through the blocked corner.
    assert!(path.len() > 4, "{path:?}");
    assert!(
        path.windows(2)
            .map(|pair| pair[0].distance(pair[1]))
            .sum::<f64>()
            > 0.5
    );
}

#[test]
fn terminal_reference_replans_heading_drift_before_the_center_leaves_the_path() {
    let mut cfg = config();
    cfg.goal_tolerance_m = 0.045;
    let mut nav = Navigator::new(cfg).unwrap();
    let target = point(6.0, 2.5);
    let first = nav
        .step_with_goal_heading(
            Timestamp(0),
            &estimate(0, pose(5.0, 2.5), 0.18),
            &[],
            Timestamp(0),
            target,
            Some(0.0),
            0.18,
        )
        .unwrap();
    assert_eq!(first.status, NavigationStatus::Driving);
    // The center is still exactly on the straight cached route, but rotating
    // the body by .1 rad puts its far corners beyond the .0225 m drift budget.
    let drifted = Pose2 {
        yaw_rad: 0.1,
        ..pose(5.1, 2.5)
    };
    let replanned = nav
        .step_with_goal_heading(
            Timestamp(100),
            &estimate(100, drifted, 0.18),
            &[],
            Timestamp(100),
            target,
            Some(0.0),
            0.18,
        )
        .unwrap();
    assert_eq!(replanned.status, NavigationStatus::Driving, "{replanned:?}");
    assert!(replanned.diagnostics.route_revision > first.diagnostics.route_revision);
    assert_eq!(replanned.path[0], drifted.point());
}
