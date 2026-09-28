// Included in local_world::tests to reuse raw raycast fixtures, not labels.
fn lidar_world() -> LocalWorld {
    let mut cfg = config();
    cfg.lidar_cones = Some(crate::lidar_cones::LidarConeConfig::simulation());
    LocalWorld::new(cfg).unwrap()
}

fn lidar_tick(world: &mut LocalWorld, s: &LidarSample, p: &PoseEstimate, extrinsic: Pose2) {
    world.update_scan(s.captured_at, p, s, extrinsic).unwrap();
    world.update_lidar_cones(s.captured_at).unwrap();
}

#[test]
fn lidar_only_confirms_distinct_scans_and_survives_visual_lease_without_faking_vision() {
    let mut world = lidar_world();
    let center = Point2 { x_m: 2.0, y_m: 0.0 }; // straddles scan seam
    for at in (0..=22000).step_by(100) {
        let s = cone_scan(at, center, 0.2, Pose2::default());
        lidar_tick(&mut world, &s, &pose(at), Pose2::default());
        world.update_lidar_cones(Timestamp(at)).unwrap();
        if at < 200 {
            assert_eq!(world.tracks(Timestamp(at)).count(), 0);
        }
    }
    let track = world.tracks(Timestamp(22000)).next().unwrap();
    assert!(track.position.distance(center) < 1e-8);
    assert_eq!(track.last_visual_at, None);
    assert_eq!(track.observations, 0);
    assert_eq!(track.color, ElementColor::Unknown);
    assert_eq!(track.lidar.unwrap().observations, 221);
    assert_eq!(world.tracks(Timestamp(23800)).count(), 0);
    assert!(world.update_lidar_cones(Timestamp(23800)).is_err());
}

#[test]
fn lidar_pose_extrinsic_loss_reconfirmation_and_processed_identity_are_independent() {
    let mut world = lidar_world();
    let extrinsic = Pose2 {
        x_m: 0.12,
        y_m: -0.03,
        yaw_rad: 0.17,
    };
    let center = Point2 { x_m: 0.6, y_m: 1.3 };
    for at in [0, 100, 200] {
        lidar_tick(
            &mut world,
            &cone_scan(at, center, 0.2, extrinsic),
            &pose(at),
            extrinsic,
        );
    }
    let old = world.tracks(Timestamp(200)).next().unwrap();
    assert!(old.position.distance(center) < 1e-8);
    for at in [2200, 2300] {
        lidar_tick(
            &mut world,
            &cone_scan(at, center, 0.2, extrinsic),
            &pose(at),
            extrinsic,
        );
        assert_eq!(world.tracks(Timestamp(at)).count(), 0);
    }
    lidar_tick(
        &mut world,
        &cone_scan(2400, center, 0.2, extrinsic),
        &pose(2400),
        extrinsic,
    );
    let renewed = world.tracks(Timestamp(2400)).next().unwrap();
    assert_eq!(renewed.id, old.id);
    world.mark_processed(old.id).unwrap();
    for at in [5000, 5100, 5200] {
        lidar_tick(
            &mut world,
            &cone_scan(at, center, 0.2, extrinsic),
            &pose(at),
            extrinsic,
        );
    }
    assert!(!world.tracks(Timestamp(5200)).any(|t| !t.processed));
    assert_eq!(world.elements.iter().flatten().count(), 1);
    assert!(
        world
            .update_scan(
                Timestamp(5300),
                &pose(5300),
                &cone_scan(5200, center, 0.2, extrinsic),
                extrinsic
            )
            .is_err()
    );
}

#[test]
fn lidar_visual_auxiliary_reuses_identity_and_cannot_create_or_renew_geometry() {
    let mut world = lidar_world();
    let center = Point2 { x_m: 2.0, y_m: 0.0 };
    for at in [0, 100, 200] {
        lidar_tick(
            &mut world,
            &cone_scan(at, center, 0.2, Pose2::default()),
            &pose(at),
            Pose2::default(),
        );
    }
    let old = world.tracks(Timestamp(200)).next().unwrap();
    for at in [300, 400, 2100] {
        world
            .update(
                Timestamp(at),
                &pose(at),
                &frame(at, vec![cone(2.0), cone(4.0)]),
            )
            .unwrap();
    }
    assert_eq!(world.elements.iter().flatten().count(), 1);
    let track = world.elements.iter().flatten().next().unwrap();
    assert_eq!(track.id, old.id);
    assert_eq!(track.last_geometry_at, Timestamp(200));
    assert_eq!(track.last_visual_at, Some(Timestamp(2100)));
    assert_eq!(world.tracks(Timestamp(2100)).count(), 0);
}

#[test]
fn lidar_walls_corners_sparse_noise_and_oversized_clusters_do_not_become_cones() {
    for iteration in 0..40 {
        let mut world = lidar_world();
        for at in [0, 100, 200, 300] {
            let mut s = scan(at);
            let heading = iteration as f64 * 0.157;
            let x = 0.5 + (iteration % 6) as f64 * 0.5;
            let y = 0.5 + (iteration % 5) as f64 * 0.4;
            for (i, r) in s.ranges_m.iter_mut().enumerate() {
                let a = heading + i as f64 * s.angle_increment_rad;
                *r = Some(
                    [
                        (0.0 - x) / a.cos(),
                        (5.0 - x) / a.cos(),
                        (0.0 - y) / a.sin(),
                        (4.0 - y) / a.sin(),
                    ]
                    .into_iter()
                    .filter(|v| *v > 0.0)
                    .fold(f64::INFINITY, f64::min),
                );
            }
            if iteration % 2 == 0 {
                s.ranges_m[73] = Some(0.7);
            }
            lidar_tick(&mut world, &s, &pose(at), Pose2::default());
        }
        assert_eq!(
            world.tracks(Timestamp(300)).count(),
            0,
            "wall pose {iteration}"
        );
    }
}

#[test]
fn lidar_ambiguity_blocks_both_tracks_and_cannot_be_recast_as_a_visual_region() {
    let mut world = lidar_world();
    for at in [0, 100, 200] {
        lidar_tick(
            &mut world,
            &cone_scan(at, Point2 { x_m: 2.0, y_m: 0.0 }, 0.2, Pose2::default()),
            &pose(at),
            Pose2::default(),
        );
    }
    let mut copy = world.elements[0].unwrap();
    copy.id = TrackId(88);
    copy.position.y_m += 0.1;
    world.elements[1] = Some(copy);
    lidar_tick(
        &mut world,
        &cone_scan(300, Point2 { x_m: 2.0, y_m: 0.0 }, 0.2, Pose2::default()),
        &pose(300),
        Pose2::default(),
    );
    assert_eq!(world.tracks(Timestamp(300)).count(), 0);
    assert!(!world.elements[0].unwrap().valid);
    assert!(!world.elements[1].unwrap().valid);
    let mut fake = cone(2.0);
    fake.source = ObservationSource::LidarGeometry;
    assert!(
        world
            .update(Timestamp(400), &pose(400), &frame(400, vec![fake]))
            .is_err()
    );
}

#[test]
fn supplied_python_probe_raw_noisy_scans_are_consumed_without_center_labels() {
    let data: serde_json::Value = serde_json::from_str(include_str!(
        "../tests/fixtures/lidar-first-probe-scans.json"
    ))
    .unwrap();
    for case in data["scans"].as_array().unwrap() {
        let mut world = lidar_world();
        let coordinates = case["pose"].as_array().unwrap();
        let mut source = pose(case["captured_at_ms"].as_u64().unwrap());
        source.pose = Pose2 {
            x_m: coordinates[0].as_f64().unwrap(),
            y_m: coordinates[1].as_f64().unwrap(),
            yaw_rad: coordinates[2].as_f64().unwrap(),
        };
        let scan = LidarSample {
            captured_at: source.captured_at,
            frame_id: FrameId("laser".into()),
            angle_min_rad: case["angle_min_rad"].as_f64().unwrap(),
            angle_increment_rad: case["angle_increment_rad"].as_f64().unwrap(),
            range_min_m: 0.02,
            range_max_m: 12.0,
            ranges_m: serde_json::from_value(case["ranges_m"].clone()).unwrap(),
        };
        lidar_tick(&mut world, &scan, &source, Pose2::default());
        assert_eq!(world.tracks(source.captured_at).count(), 0); // One frame cannot confirm.
        let expected = &case["selected_track_center"];
        let expected = Point2 {
            x_m: expected[0].as_f64().unwrap(),
            y_m: expected[1].as_f64().unwrap(),
        };
        // The large scene's first Python acquisition has less geometric
        // support than this detector admits. Retain the rejection; do not
        // weaken point/shape gates to reproduce the prototype's selection.
        let expected_match = case["name"] != "large_8x6";
        assert_eq!(
            world
                .elements
                .iter()
                .flatten()
                .any(|t| t.position.distance(expected) < 0.1),
            expected_match,
            "{}",
            case["name"]
        );
        for t in world.elements.iter().flatten() {
            assert_eq!(t.observations, 0);
            assert_eq!(t.last_visual_at, None);
        }
    }
}
