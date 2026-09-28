// Policy-level measured-pose sequences; full plant tests live in lidar_matrix.
fn lidar_setup() -> (OnlineMission, LocalWorld) {
    let (mut mission, world) = setup();
    let mut config = world.config().clone();
    config.lidar_cones = Some(crate::lidar_cones::LidarConeConfig::simulation());
    mission.phase = MissionPhase::Cones;
    mission.initial_heading = Some(0.0);
    (mission, LocalWorld::new(config).unwrap())
}
fn observe_raw_cones(world: &mut LocalWorld, p: &PoseEstimate, centers: &[Point2]) {
    let mut ranges = vec![Some(5.0f64); 720];
    for (i, value) in ranges.iter_mut().enumerate() {
        let angle = i as f64 * TAU / 720.0;
        for center in centers {
            let c = p.pose.world_to_body(*center);
            let projection = c.x_m * angle.cos() + c.y_m * angle.sin();
            let side = c.x_m * angle.sin() - c.y_m * angle.cos();
            if projection > 0.0 && side.abs() < 0.2 {
                let hit = projection - (0.04 - side * side).sqrt();
                *value = Some(value.unwrap().min(hit));
            }
        }
    }
    let scan = LidarSample {
        captured_at: p.captured_at,
        frame_id: FrameId("sim_laser".into()),
        angle_min_rad: 0.0,
        angle_increment_rad: TAU / 720.0,
        range_min_m: 0.02,
        range_max_m: 20.0,
        ranges_m: ranges,
    };
    world
        .update_scan(p.captured_at, p, &scan, Pose2::default())
        .unwrap();
    world.update_lidar_cones(p.captured_at).unwrap();
}

#[test]
fn lidar_two_cone_policy_keeps_order_actual_progress_revision_and_real_visual_clocks() {
    let (mut mission, mut world) = lidar_setup();
    let centers = [Point2 { x_m: 2.0, y_m: 1.0 }, Point2 { x_m: 0.5, y_m: 1.8 }];
    let old_road = road(0, LightState::Green);
    let mut now = 22000;
    for _ in 0..3 {
        let p = pose(now, Pose2::default());
        observe_raw_cones(&mut world, &p, &centers);
        let r = mission.update(Timestamp(now), &p, &old_road, &mut world);
        assert_ne!(r.mission.phase, MissionPhase::Fault);
        assert!(!r.requires_road_semantics);
        now += 100;
    }
    let mut previous_revision = mission.task_revision;
    for (index, center) in centers.into_iter().enumerate() {
        if index == 1 {
            let p = pose(
                now,
                Pose2 {
                    x_m: 1.6,
                    y_m: 1.85,
                    yaw_rad: PI,
                },
            );
            observe_raw_cones(&mut world, &p, &centers);
            mission.update(Timestamp(now), &p, &old_road, &mut world);
            now += 100;
        }
        let id = mission.active.expect("confirmed cone is locked");
        let radius = mission.orbit.unwrap().radius_m;
        let direction = if index == 0 { 1.0 } else { -1.0 };
        for step in 0..=4 {
            let angle = -FRAC_PI_2 + direction * step as f64 * FRAC_PI_4;
            let point = polar(center, radius, angle);
            let p = pose(
                now,
                Pose2 {
                    x_m: point.x_m,
                    y_m: point.y_m,
                    yaw_rad: wrap(angle + direction * FRAC_PI_2),
                },
            );
            observe_raw_cones(&mut world, &p, &centers);
            let r = mission.update(Timestamp(now), &p, &old_road, &mut world);
            assert_eq!(r.active_track_id, Some(id));
            assert_eq!(r.processed_cones, index);
            assert!(r.active_track.unwrap().lidar.is_some());
            assert_eq!(r.active_track.unwrap().last_visual_at, None);
            now += 100;
        }
        let error = mission.active_geometry.unwrap().position_error_m;
        let p = pose(
            now,
            Pose2 {
                x_m: center.x_m - direction * (error + 0.13),
                y_m: center.y_m + radius,
                yaw_rad: if index == 0 { PI } else { 0.0 },
            },
        );
        observe_raw_cones(&mut world, &p, &centers);
        let r = mission.update(Timestamp(now), &p, &old_road, &mut world);
        assert_eq!(r.processed_cones, index + 1);
        assert!(r.task_revision > previous_revision);
        previous_revision = r.task_revision;
        now += 100;
        if index == 0 {
            assert_eq!(r.mission.phase, MissionPhase::Cones);
        } else {
            assert_eq!(r.mission.phase, MissionPhase::ApproachLight);
            assert_eq!(r.mission.output, MissionOutput::Stop);
            assert!(r.requires_road_semantics);
            assert_eq!(r.mission.green_elapsed_ms, 0);
            let next = pose(now, p.pose);
            observe_raw_cones(&mut world, &next, &centers);
            let r = mission.update(Timestamp(now), &next, &old_road, &mut world);
            assert_eq!(r.mission.phase, MissionPhase::Fault); // Old green cannot release next element.
        }
    }
}

#[test]
fn lidar_side_front_acquisition_is_not_camera_fov_and_multiple_candidates_do_not_guess() {
    for angles in [vec![1.15], vec![0.15, 0.65, 1.15], vec![0.65, 0.77]] {
        let (mission, mut world) = lidar_setup();
        let centers: Vec<_> = angles
            .iter()
            .enumerate()
            .map(|(i, a)| {
                polar(
                    Point2::default(),
                    if angles.len() == 2 {
                        3.0 + 1.5 * i as f64
                    } else {
                        3.0
                    },
                    *a,
                )
            })
            .collect();
        for at in [0, 100, 200] {
            observe_raw_cones(&mut world, &pose(at, Pose2::default()), &centers);
        }
        assert_eq!(world.tracks(Timestamp(200)).count(), angles.len());
        let selected =
            mission.select_track(Timestamp(200), Pose2::default(), ElementKind::Cone, &world);
        assert_eq!(selected.is_some(), angles.len() == 1, "{angles:?}");
        if let Some(track) = selected {
            assert!(track.position.y_m > track.position.x_m);
            assert_eq!(track.color, ElementColor::Unknown);
        }
    }
}
