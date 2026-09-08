use xt_stcar_robot_core::autonomy::{Point2, Pose2};
use xt_stcar_robot_core::localization::{LocalizationConfig, ScanOdometry};
use xt_stcar_robot_core::{FrameId, Timestamp};

fn config() -> LocalizationConfig {
    LocalizationConfig::simulation(FrameId("map".into()))
}

// Irregular static landmarks are synthetic, not recorded vehicle scan evidence.
fn cloud() -> Vec<Point2> {
    (0..100)
        .map(|i| {
            let angle = f64::from(i) * 2.399_963_229_728_653;
            let radius = 1.0 + f64::from((i * 37) % 101) * 0.014;
            Point2 {
                x_m: radius * angle.cos(),
                y_m: radius * angle.sin(),
            }
        })
        .collect()
}
fn moved(points: &[Point2], relative: Pose2) -> Vec<Point2> {
    points.iter().map(|p| relative.world_to_body(*p)).collect()
}

#[test]
fn known_rigid_motion_has_correct_sign_world_frame_and_time() {
    let anchor = Pose2 {
        x_m: 1.0,
        y_m: 2.0,
        yaw_rad: 0.3,
    };
    let relative = Pose2 {
        x_m: 0.04,
        y_m: 0.008,
        yaw_rad: 0.02,
    };
    let mut odom = ScanOdometry::new(config(), anchor).unwrap();
    let scan = cloud();
    let first = odom.update(Timestamp(0), &scan).unwrap();
    assert!(!first.accepted);
    assert!(first.estimate.is_none());
    assert_eq!(first.reason.as_deref(), Some("reference_initialized"));
    let update = odom
        .update(Timestamp(100), &moved(&scan, relative))
        .unwrap();
    assert!(update.accepted, "{update:?}");
    assert!(update.rmse_m.unwrap() < 1e-9);
    let estimate = update.estimate.unwrap();
    let expected_point = anchor.body_to_world(relative.point());
    assert!(estimate.pose.point().distance(expected_point) < 1e-8);
    assert!((estimate.pose.yaw_rad - 0.32).abs() < 1e-8);
    assert!((estimate.speed_mps - 0.04_f64.hypot(0.008) / 0.1).abs() < 1e-8);
    assert!((estimate.yaw_rate_radps - 0.2).abs() < 1e-8);
    assert_eq!(estimate.frame_id, FrameId("map".into()));
    assert_eq!(estimate.captured_at, Timestamp(100));
    assert!(estimate.quality > 0.99);
}

#[test]
fn successive_scan_motion_composes_in_the_correct_order() {
    let mut odom = ScanOdometry::new(config(), Pose2::default()).unwrap();
    let world = cloud();
    odom.update(Timestamp(0), &world).unwrap();
    for i in 1..=20 {
        let pose = Pose2 {
            x_m: f64::from(i) * 0.02,
            y_m: f64::from(i) * 0.003,
            yaw_rad: f64::from(i) * 0.012,
        };
        let update = odom
            .update(Timestamp(u64::from(i as u32) * 100), &moved(&world, pose))
            .unwrap();
        assert!(update.accepted, "step {i}: {update:?}");
        let estimate = update.estimate.unwrap();
        assert!(estimate.pose.point().distance(pose.point()) < 1e-7);
        assert!((estimate.pose.yaw_rad - pose.yaw_rad).abs() < 1e-7);
    }
}

#[test]
fn partial_overlap_is_measured_and_far_outliers_do_not_move_pose() {
    let mut odom = ScanOdometry::new(config(), Pose2::default()).unwrap();
    let scan = cloud();
    odom.update(Timestamp(0), &scan).unwrap();
    let relative = Pose2 {
        x_m: 0.03,
        y_m: -0.006,
        yaw_rad: -0.015,
    };
    let mut next = moved(&scan[..80], relative);
    next.extend((0..20).map(|i| Point2 {
        x_m: 20.0 + f64::from(i),
        y_m: 20.0,
    }));
    let update = odom.update(Timestamp(100), &next).unwrap();
    assert!(update.accepted, "{update:?}");
    assert_eq!(update.matched_points, 80);
    let e = update.estimate.unwrap();
    assert!(e.pose.point().distance(relative.point()) < 1e-8);
    assert!(e.quality > 0.79 && e.quality < 0.81);
}

#[test]
fn collinear_identical_and_many_duplicate_points_never_produce_pose() {
    for points in [
        (0..40)
            .map(|i| Point2 {
                x_m: f64::from(i) * 0.1,
                y_m: 0.0,
            })
            .collect::<Vec<_>>(),
        vec![Point2 { x_m: 1.0, y_m: 2.0 }; 40],
    ] {
        let mut odom = ScanOdometry::new(config(), Pose2::default()).unwrap();
        for t in [0, 100] {
            let update = odom.update(Timestamp(t), &points).unwrap();
            assert!(!update.accepted);
            assert!(update.estimate.is_none());
            assert_eq!(update.reason.as_deref(), Some("degenerate_geometry"));
        }
    }
    let duplicates: Vec<_> = (0..40)
        .map(|i| Point2 {
            x_m: if i % 2 == 0 { -1.0 } else { 1.0 },
            y_m: if i % 4 < 2 { -1.0 } else { 1.0 },
        })
        .collect();
    let mut odom = ScanOdometry::new(config(), Pose2::default()).unwrap();
    odom.update(Timestamp(0), &duplicates).unwrap();
    let update = odom.update(Timestamp(100), &duplicates).unwrap();
    assert!(!update.accepted);
    assert!(update.estimate.is_none());
    assert!(update.matched_points <= 4);
}

#[test]
fn rejected_cloud_does_not_refresh_reference_and_gap_requires_explicit_reset() {
    let scan = cloud();
    let mut odom = ScanOdometry::new(config(), Pose2::default()).unwrap();
    odom.update(Timestamp(0), &scan).unwrap();
    assert!(odom.update(Timestamp(50), &scan).unwrap().accepted);
    assert!(!odom.update(Timestamp(200), &scan[..5]).unwrap().accepted);
    let gap = odom.update(Timestamp(300), &scan).unwrap();
    assert_eq!(
        gap.reason.as_deref(),
        Some("scan_gap_requires_explicit_reset")
    );
    assert!(gap.estimate.is_none());
    let later = odom.update(Timestamp(350), &scan).unwrap();
    assert_eq!(
        later.reason.as_deref(),
        Some("lost_requires_explicit_reset")
    );
    odom.reset(Pose2 {
        x_m: 5.0,
        y_m: 3.0,
        yaw_rad: 0.0,
    })
    .unwrap();
    assert!(!odom.update(Timestamp(400), &scan).unwrap().accepted);
    let estimate = odom
        .update(Timestamp(450), &scan)
        .unwrap()
        .estimate
        .unwrap();
    assert!((estimate.pose.x_m - 5.0).abs() < 1e-9);
}

#[test]
fn invalid_or_regressing_input_is_rejected_without_mutating_reference() {
    let scan = cloud();
    let mut odom = ScanOdometry::new(config(), Pose2::default()).unwrap();
    odom.update(Timestamp(100), &scan).unwrap();
    assert!(odom.update(Timestamp(100), &scan).is_err());
    assert!(odom.update(Timestamp(99), &scan).is_err());
    assert!(
        odom.update(
            Timestamp(150),
            &[Point2 {
                x_m: f64::NAN,
                y_m: 1.0
            }]
        )
        .is_err()
    );
    assert!(
        odom.update(Timestamp(150), &vec![Point2::default(); 8193])
            .is_err()
    );
    assert!(odom.update(Timestamp(150), &scan).unwrap().accepted);
    let mut invalid = config();
    invalid.max_points = usize::MAX;
    assert!(ScanOdometry::new(invalid, Pose2::default()).is_err());
}

#[test]
fn excessive_motion_and_unrelated_cloud_are_not_trusted() {
    let scan = cloud();
    let mut cfg = config();
    cfg.max_speed_mps = 0.1;
    let mut odom = ScanOdometry::new(cfg, Pose2::default()).unwrap();
    odom.update(Timestamp(0), &scan).unwrap();
    let update = odom
        .update(
            Timestamp(100),
            &moved(
                &scan,
                Pose2 {
                    x_m: 0.04,
                    y_m: 0.0,
                    yaw_rad: 0.0,
                },
            ),
        )
        .unwrap();
    assert_eq!(update.reason.as_deref(), Some("motion_jump_exceeds_limits"));
    assert!(update.estimate.is_none());
    let unrelated: Vec<_> = scan
        .iter()
        .map(|p| Point2 {
            x_m: p.x_m + 20.0,
            y_m: p.y_m,
        })
        .collect();
    assert!(
        odom.update(Timestamp(150), &unrelated)
            .unwrap()
            .estimate
            .is_none()
    );
}
