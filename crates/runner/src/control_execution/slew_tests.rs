// Real shared-history, admission and certificate paths, with fixed timestamps.
#[test]
fn planning_context_uses_changed_command_time_not_latest_poll_time() {
    let (config, input, _, _) = fixture();
    let execution = AtomicExecution::new(SteeringEstimate::stationary(Timestamp(0)));
    let mut steering = SteeringEstimate::stationary(Timestamp(0));
    let bootstrap = execution
        .read()
        .unwrap()
        .project(&input, Timestamp(1), &config)
        .unwrap();
    assert_eq!(bootstrap.last_command_change_at, None);
    assert_eq!(bootstrap.adopted_revision, 0);
    for (at, speed, curvature, stopped, expected_change, expected_revision) in [
        (10, 0.02, 0.2, false, 10, 1),
        (15, 0.02, 0.2, false, 10, 1), // Same entire command.
        (20, 0.03, 0.2, false, 20, 2), // Only speed changed.
        (25, 0.03, 0.2, false, 20, 2),
        (30, 0.0, 0.0, true, 30, 3),
        (35, 0.0, 0.0, true, 30, 3),  // Repeated Stop.
        (40, 0.0, 0.0, false, 40, 4), // Stop -> Drive even at zero.
        (45, 0.02, 0.04, false, 45, 5),
    ] {
        let command = if stopped {
            MotionOutput::Stop
        } else {
            MotionOutput::Drive {
                speed_mps: speed,
                curvature_per_m: curvature,
            }
        };
        steering.adopt(Timestamp(at), &command, 4.0, 2.0).unwrap();
        execution.publish(steering, &command);
        let history = execution.read().unwrap();
        assert_eq!(history.latest.steering.at, Timestamp(at));
        assert_eq!(
            execution.last_change().steering.at,
            Timestamp(expected_change)
        );
        let context = history.project(&input, Timestamp(at + 1), &config).unwrap();
        assert_eq!(context.steering.at, Timestamp(at + 1));
        assert_eq!(
            context.last_command_change_at,
            Some(Timestamp(expected_change))
        );
        assert_eq!(context.adopted_revision, expected_revision);
        assert_eq!(context.held_speed_mps, speed);
        assert_eq!(context.steering.commanded_curvature_per_m, curvature);
        assert!(context.historical_curvature_bound_per_m >= 0.2);
    }
}

#[test]
fn prepared_interval_and_certificate_share_short_long_and_stop_change_clocks() {
    let (config, input, mut context, mut step) = fixture();
    for (last, held, target, expected_delta) in [
        (49, 0.4, 0.444, 0.044), // 11 ms; planner period is still 100 ms.
        (0, 0.4, 0.64, 0.24),
        (60, 0.4, 0.4, 0.0),
        (49, 0.0, 0.044, 0.044), // Restart after a changed Stop.
    ] {
        context.adopted_revision = 2;
        context.last_command_change_at = Some(Timestamp(last));
        context.steering.commanded_curvature_per_m = held;
        context.historical_curvature_bound_per_m = held;
        let constraints =
            crate::control_admission::prepare(&config, &input, &context, Timestamp(0), 250)
                .unwrap();
        let (_, high) = constraints.curvature_interval(&config.navigation);
        assert!((high - held - expected_delta).abs() < 1e-12);
        step.command = MotionOutput::Drive {
            speed_mps: 0.04,
            curvature_per_m: target,
        };
        assert!(
            constraints
                .check_command(&config.navigation, 0.04, target, &[], None)
                .is_ok()
        );
        let certificate = certify(&config, &input, &context, &step, Timestamp(0), 250).unwrap();
        assert_eq!(certificate.from, Timestamp(60));
        assert_eq!(certificate.through, Timestamp(160));
        step.command = MotionOutput::Drive {
            speed_mps: 0.04,
            curvature_per_m: target + 0.001,
        };
        assert_eq!(
            constraints.check_command(&config.navigation, 0.04, target + 0.001, &[], None),
            Err(xt_stcar_robot_core::admission::AdmissionRejection::Curvature)
        );
        let failure = certify(&config, &input, &context, &step, Timestamp(0), 250).unwrap_err();
        assert_eq!(failure.reason, CertificateFailureReason::CurvatureSlew);
        assert_eq!(failure.last_command_change_at, Some(Timestamp(last)));
        assert!((failure.signed_margin.unwrap() - (-0.001 + 1e-9)).abs() < 1e-12);
    }
    for (revision, stamp) in [(1, None), (0, Some(Timestamp(0))), (1, Some(Timestamp(61)))] {
        context.adopted_revision = revision;
        context.last_command_change_at = stamp;
        assert!(
            crate::control_admission::prepare(&config, &input, &context, Timestamp(0), 250)
                .is_err()
        );
        assert_eq!(
            certify(&config, &input, &context, &step, Timestamp(0), 250)
                .unwrap_err()
                .reason,
            CertificateFailureReason::CurvatureTimingInvalid
        );
    }
}

#[test]
fn revision_zero_cannot_represent_nonzero_applied_or_historical_steering() {
    let (config, input, context, step) = fixture();
    for case in 0..4 {
        let mut context = context.clone();
        match case {
            0 => context.held_speed_mps = 0.01,
            1 => context.steering.commanded_curvature_per_m = 0.1,
            2 => context.steering.applied_curvature_per_m = 0.1,
            _ => context.historical_curvature_bound_per_m = 0.1,
        }
        assert!(
            crate::control_admission::prepare(&config, &input, &context, Timestamp(0), 250)
                .is_err()
        );
        assert_eq!(
            certify(&config, &input, &context, &step, Timestamp(0), 250)
                .unwrap_err()
                .reason,
            CertificateFailureReason::CurvatureTimingInvalid
        );
    }
}
