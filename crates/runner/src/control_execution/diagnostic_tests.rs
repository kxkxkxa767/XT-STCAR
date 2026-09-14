// Tests run the real certificate; no hardware, wall clock, or test-only admission.

#[test]
fn certificate_preserves_success_window_stop_policy_and_source_records() {
    let (config, mut input, context, mut step) = fixture();
    let before = serde_json::to_value(&input).unwrap();
    let certificate = certify(&config, &input, &context, &step, Timestamp(0), 250).unwrap();
    assert_eq!(certificate.from, Timestamp(60));
    assert_eq!(certificate.through, Timestamp(160));
    assert_eq!(certificate.revision, context.adopted_revision);
    assert_eq!(serde_json::to_value(&input).unwrap(), before);

    // Stop retains the original early return, after source/time validation.
    input.scan.captured_at = Timestamp(1);
    input.road.observation.cones_body_m = vec![Point2 { x_m: 0.0, y_m: 0.0 }];
    step.command = MotionOutput::Stop;
    assert!(certify(&config, &input, &context, &step, Timestamp(0), 250).is_ok());
    input.pose.speed_mps = f64::NAN;
    assert_eq!(
        certify(&config, &input, &context, &step, Timestamp(0), 250)
            .unwrap_err()
            .reason,
        CertificateFailureReason::SourceSpeedInvalid
    );
}

#[test]
fn certificate_distinguishes_invalid_source_command_and_sensor_times() {
    use CertificateFailureReason as Reason;
    let (config, input, context, _) = fixture();
    for (case, expected) in [
        (0, Reason::SourceSpeedInvalid),
        (1, Reason::CommandSpeedInvalid),
        (2, Reason::ProjectedSpeedInvalid),
        (3, Reason::HeldSpeedInvalid),
        (4, Reason::CommandCurvatureInvalid),
        (5, Reason::ScanTimeMismatch),
        (6, Reason::ConeTimeMismatch),
        (7, Reason::ProjectionFailed),
        (8, Reason::CommandCurvatureInvalid),
        (9, Reason::StoppingEnvelopeInvalid),
    ] {
        let mut input = input.clone();
        let mut context = context.clone();
        let (_, _, _, mut step) = fixture();
        match case {
            0 => input.pose.speed_mps = f64::NAN,
            1 => {
                step.command = MotionOutput::Drive {
                    speed_mps: -0.01,
                    curvature_per_m: 0.0,
                }
            }
            2 => context.projected_pose.speed_mps = -0.01,
            3 => context.held_speed_mps = 0.31,
            4 => {
                step.command = MotionOutput::Drive {
                    speed_mps: 0.04,
                    curvature_per_m: 2.01,
                }
            }
            5 => input.scan.captured_at = Timestamp(1),
            6 => {
                input.road.observation.captured_at = Timestamp(1);
                input.road.observation.cones_body_m = vec![Point2 { x_m: 5.0, y_m: 0.0 }];
            }
            7 => context.projected_pose.pose.yaw_rad = f64::NAN,
            8 => {
                step.command = MotionOutput::Drive {
                    speed_mps: 0.04,
                    curvature_per_m: f64::NAN,
                }
            }
            9 => context.historical_speed_bound_mps = 0.31,
            _ => unreachable!(),
        }
        let error = certify(&config, &input, &context, &step, Timestamp(0), 250).unwrap_err();
        assert_eq!(error.reason, expected, "case {case}");
        assert_eq!(error.source_at, input.at);
        assert_eq!(error.pose_at, input.pose.captured_at);
        assert_eq!(error.planned_at, context.planned_at);
        assert_eq!(error.lease_expires_at, Some(Timestamp(250)));
        assert_eq!(error.adoption_through, Some(Timestamp(160)));
        assert!(error.constraint_index.is_none());
        assert!(error.signed_margin.is_none());
        // An invalid input still produces a serializable failure record.
        serde_json::to_string(&error).unwrap();
    }
}

#[test]
fn certificate_distinguishes_expiry_overflow_and_invalid_envelope_time() {
    use CertificateFailureReason as Reason;
    let (config, mut input, mut context, step) = fixture();
    let error = certify(&config, &input, &context, &step, Timestamp(0), 60).unwrap_err();
    assert_eq!(error.reason, Reason::PlanExpired);
    assert_eq!(error.lease_expires_at, Some(Timestamp(60)));
    assert_eq!(error.adoption_through, None);

    let error = certify(
        &config,
        &input,
        &context,
        &step,
        Timestamp(u64::MAX - 10),
        250,
    )
    .unwrap_err();
    assert_eq!(error.reason, Reason::LeaseOverflow);
    assert_eq!(error.lease_expires_at, None);

    context.planned_at = Timestamp(u64::MAX - 50);
    let error = certify(
        &config,
        &input,
        &context,
        &step,
        Timestamp(u64::MAX - 100),
        100,
    )
    .unwrap_err();
    assert_eq!(error.reason, Reason::AdoptionWindowOverflow);
    assert_eq!(error.lease_expires_at, Some(Timestamp(u64::MAX)));
    assert_eq!(error.adoption_through, None);

    context.planned_at = Timestamp(60);
    input.pose.captured_at = Timestamp(400);
    input.scan.captured_at = Timestamp(400);
    let error = certify(&config, &input, &context, &step, Timestamp(0), 250).unwrap_err();
    assert_eq!(error.reason, Reason::EnvelopeTimeInvalid);
    assert_eq!(error.speed_bound_mps, Some(0.04));
    assert!(error.stopping_rectangle_body_m.is_none());
}

#[test]
fn certificate_distinguishes_slew_speed_and_lateral_constraints() {
    let (mut config, input, context, mut step) = fixture();
    step.command = MotionOutput::Drive {
        speed_mps: 0.04,
        curvature_per_m: 0.401,
    };
    let error = certify(&config, &input, &context, &step, Timestamp(0), 250).unwrap_err();
    assert_eq!(error.reason, CertificateFailureReason::CurvatureSlew);
    assert_eq!(
        error.margin_unit,
        Some(CertificateMarginUnit::InverseMeters)
    );
    assert!((error.signed_margin.unwrap() - (-0.001 + 1e-9)).abs() < 1e-14);

    step.command = MotionOutput::Drive {
        speed_mps: 0.06,
        curvature_per_m: 0.0,
    };
    let error = certify(&config, &input, &context, &step, Timestamp(0), 250).unwrap_err();
    assert_eq!(error.reason, CertificateFailureReason::SpeedInterval);
    assert_eq!(
        error.margin_unit,
        Some(CertificateMarginUnit::MetersPerSecond)
    );
    assert!(error.signed_margin.unwrap() < -0.019);

    step.command = MotionOutput::Drive {
        speed_mps: 0.04,
        curvature_per_m: 0.4,
    };
    config.navigation.max_lateral_accel_mps2 = 0.0001;
    let error = certify(&config, &input, &context, &step, Timestamp(0), 250).unwrap_err();
    assert_eq!(error.reason, CertificateFailureReason::LateralAcceleration);
    assert_eq!(
        error.margin_unit,
        Some(CertificateMarginUnit::MetersPerSecondSquared)
    );
    assert!(error.signed_margin.unwrap() < 0.0);
}

#[test]
fn certificate_reports_first_laser_then_first_cone_with_signed_disc_clearance() {
    let (config, mut input, context, step) = fixture();
    input.scan.angle_min_rad = 0.0;
    input.scan.angle_increment_rad = 0.0;
    input.scan.ranges_m = vec![None, Some(5.0), Some(0.2), Some(0.1)];
    input.road.observation.cones_body_m = vec![
        Point2 { x_m: 3.0, y_m: 1.0 },
        Point2 { x_m: 0.2, y_m: 0.0 },
        Point2 { x_m: 0.1, y_m: 0.0 },
    ];
    let error = certify(&config, &input, &context, &step, Timestamp(0), 250).unwrap_err();
    assert_eq!(error.reason, CertificateFailureReason::LaserObstacle);
    assert_eq!(error.constraint_index, Some(2));
    assert_eq!(error.signed_margin, Some(-config.laser_point_radius_m));
    assert_eq!(error.margin_unit, Some(CertificateMarginUnit::Meters));
    assert_eq!(error.speed_bound_mps, Some(0.04));
    assert_eq!(error.curvature_bound_per_m, Some(0.0));
    assert!(error.stopping_rectangle_body_m.is_some());

    // Isolate the next check in a test copy; this is not the original input.
    input.scan.ranges_m = vec![Some(5.0)];
    let error = certify(&config, &input, &context, &step, Timestamp(0), 250).unwrap_err();
    assert_eq!(error.reason, CertificateFailureReason::VisionCone);
    assert_eq!(error.constraint_index, Some(1));
    assert_eq!(error.signed_margin, Some(-config.cone_radius_m));

    input.scan.ranges_m = vec![Some(f64::NAN)];
    let error = certify(&config, &input, &context, &step, Timestamp(0), 250).unwrap_err();
    assert_eq!(error.reason, CertificateFailureReason::LaserObstacle);
    assert_eq!(error.constraint_index, Some(0));
    assert_eq!(error.signed_margin, None);
}

#[test]
fn certificate_distinguishes_rotated_map_and_oblique_light_boundaries() {
    use xt_stcar_robot_core::mission::{MissionOutput, MissionReport};
    let (mut config, mut input, mut context, mut step) = fixture();
    input.scan.ranges_m = vec![Some(5.0); 360];
    input.pose.pose.yaw_rad = std::f64::consts::FRAC_PI_2;
    input.pose.pose.x_m = 6.85;
    context.projected_pose.pose = input.pose.pose;
    let error = certify(&config, &input, &context, &step, Timestamp(0), 250).unwrap_err();
    assert_eq!(error.reason, CertificateFailureReason::MapBoundary);
    assert!(error.constraint_index.unwrap() < 4);
    assert!(error.signed_margin.unwrap() < 0.0);

    config.mission.light_approach_yaw_rad = std::f64::consts::FRAC_PI_4;
    step.mission = Some(MissionReport {
        at: context.planned_at,
        phase: MissionPhase::ApproachLight,
        output: MissionOutput::Stop,
        reason: "diagnostic oblique boundary fixture".into(),
        crosswalk_stop_elapsed_ms: 0,
        green_elapsed_ms: 0,
        waypoint_index: 2,
    });
    input.pose.pose.x_m = 5.5;
    input.pose.pose.y_m = 3.24;
    context.projected_pose.pose = input.pose.pose;
    let error = certify(&config, &input, &context, &step, Timestamp(0), 250).unwrap_err();
    assert_eq!(error.reason, CertificateFailureReason::LightBoundary);
    assert!(error.constraint_index.unwrap() < 4);
    assert!(error.signed_margin.unwrap() < 0.0);

    config.mission.light_approach_yaw_rad = f64::NAN;
    assert_eq!(
        certify(&config, &input, &context, &step, Timestamp(0), 250)
            .unwrap_err()
            .reason,
        CertificateFailureReason::LightBoundaryInvalid
    );
}

#[test]
fn published_lqr_source_has_the_same_obstacle_failure_for_every_admissible_test_target() {
    let fixture_value: serde_json::Value =
        serde_json::from_str(include_str!("fixtures/lqr-first-rejection.json")).unwrap();
    let config: AutonomyConfig = serde_json::from_value(fixture_value["config"].clone()).unwrap();
    let input: SensorSnapshot = serde_json::from_value(fixture_value["source"].clone()).unwrap();
    let before = serde_json::to_value(&input).unwrap();
    let steering = SteeringEstimate {
        at: input.at,
        commanded_curvature_per_m: -2.0,
        applied_curvature_per_m: -2.0,
    };
    let history = AtomicExecution::new(SteeringEstimate::stationary(input.at));
    history.publish(
        steering,
        &MotionOutput::Drive {
            speed_mps: 0.3,
            curvature_per_m: -2.0,
        },
    );
    let context = history
        .read()
        .unwrap()
        .project(&input, Timestamp(16860), &config)
        .unwrap();
    let (_, _, _, mut step) = fixture();
    step.at = context.planned_at;
    assert_eq!(input.scan.ranges_m.len(), 360);
    assert_eq!(context.historical_speed_bound_mps, 0.3);
    assert_eq!(context.historical_curvature_bound_per_m, 2.0);
    for speed_mps in [0.24, 0.27, 0.3] {
        for curvature_per_m in [-2.0, -1.9, -1.8] {
            // These meet the certificate's .06 m/s decel and actual 60ms .24 per_m slew
            // bounds. This test does not claim all are Navigator candidates.
            step.command = MotionOutput::Drive {
                speed_mps,
                curvature_per_m,
            };
            let error = certify(&config, &input, &context, &step, input.at, 250).unwrap_err();
            assert_eq!(error.reason, CertificateFailureReason::LaserObstacle);
            assert_eq!(error.constraint_index, Some(331));
            assert!((error.signed_margin.unwrap() + 0.004311433629389386).abs() < 1e-12);
            assert_eq!(error.source_at, Timestamp(16800));
            assert_eq!(error.pose_at, Timestamp(16800));
            assert_eq!(error.oldest_sensor_at, Timestamp(16800));
            assert_eq!(error.lease_expires_at, Some(Timestamp(17050)));
            assert_eq!(error.adoption_through, Some(Timestamp(16960)));
            assert_eq!(error.speed_bound_mps, Some(0.3));
            assert_eq!(error.curvature_bound_per_m, Some(2.0));
            let rectangle = error.stopping_rectangle_body_m.unwrap();
            assert!((rectangle.max_x_m - 0.5319939128423334).abs() < 1e-12);
            assert!((rectangle.max_y_m - 0.29439391284233335).abs() < 1e-12);
        }
    }
    assert_eq!(serde_json::to_value(&input).unwrap(), before);

    let mut cone_only = input.clone();
    cone_only.scan.ranges_m[331] = None;
    cone_only.scan.ranges_m[332] = None;
    let error = certify(&config, &cone_only, &context, &step, input.at, 250).unwrap_err();
    assert_eq!(error.reason, CertificateFailureReason::VisionCone);
    assert_eq!(error.constraint_index, Some(1));
    assert!((error.signed_margin.unwrap() + 0.0003750674728799641).abs() < 1e-12);
}
