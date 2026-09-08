use xt_stcar_robot_core::*;

fn config() -> SafetyConfig {
    let mut config: SafetyConfig =
        serde_json::from_str(include_str!("../../../config/robot-sim.json")).unwrap();
    config.required_sensors = vec![SensorKind::Lidar];
    config
}

fn lidar(angle_min_rad: f64, angle_increment_rad: f64, beams: usize) -> LidarSample {
    LidarSample {
        captured_at: Timestamp(0),
        frame_id: config().frames.lidar_frame,
        angle_min_rad,
        angle_increment_rad,
        range_min_m: 0.1,
        range_max_m: 10.0,
        ranges_m: vec![Some(1.0); beams],
    }
}

#[test]
fn finite_lidar_fields_cannot_hide_nonfinite_beam_geometry() {
    for (origin, increment, beams) in [
        (f64::MAX, f64::MAX, 2),
        (-f64::MAX, -f64::MAX, 2),
        (0.0, f64::MAX, 3),
        (0.0, -f64::MAX, 3),
        // Even when origin and step have opposite signs, the intermediate
        // multiplication must remain usable by downstream beam consumers.
        (-f64::MAX, f64::MAX, 3),
    ] {
        assert!(origin.is_finite() && increment.is_finite());
        assert!(
            SensorSample::Lidar(lidar(origin, increment, beams))
                .validate(&config().frames)
                .is_err()
        );
    }
}

#[test]
fn negative_n10_angles_across_zero_and_single_beams_remain_valid() {
    // N10 350 -> 5 degrees in the driver's x=cos(a), y=-sin(a) convention.
    let sample = lidar(-350.0_f64.to_radians(), -1.0_f64.to_radians(), 16);
    let final_angle = sample.angle_min_rad + 15.0 * sample.angle_increment_rad;
    assert!((final_angle.cos() - 5.0_f64.to_radians().cos()).abs() < 1e-12);
    assert!((final_angle.sin() + 5.0_f64.to_radians().sin()).abs() < 1e-12);
    assert!(
        SensorSample::Lidar(sample)
            .validate(&config().frames)
            .is_ok()
    );
    assert!(
        SensorSample::Lidar(lidar(f64::MAX, f64::MAX, 1))
            .validate(&config().frames)
            .is_ok()
    );
    assert!(
        SensorSample::Lidar(lidar(0.0, 0.1, 0))
            .validate(&config().frames)
            .is_err()
    );
}

#[test]
fn unusable_geometry_faults_a_running_controller_and_latches_stop() {
    let mut controller = Controller::new(config()).unwrap();
    for event in [
        Event::Heartbeat,
        Event::Deadman { pressed: true },
        Event::Sensor {
            sample: SensorSample::Lidar(lidar(0.0, 0.1, 2)),
        },
        Event::Arm,
        Event::Motion {
            intent: MotionIntent {
                speed_mps: 0.1,
                curvature_per_m: 0.0,
            },
        },
        Event::Start,
    ] {
        controller.handle(TimedEvent {
            at: Timestamp(0),
            event,
        });
    }
    assert_eq!(controller.state(), State::Running);
    let mut invalid = lidar(1e308, 1e308, 2);
    invalid.captured_at = Timestamp(1);
    let rejected = controller.handle(TimedEvent {
        at: Timestamp(1),
        event: Event::Sensor {
            sample: SensorSample::Lidar(invalid),
        },
    });
    assert_eq!(rejected.state, State::Fault);
    assert_eq!(rejected.output.command, MotionOutput::Stop);
    assert!(matches!(
        rejected.output.reason,
        Some(StopReason::InvalidInput { .. })
    ));
    let mut valid = lidar(0.0, 0.1, 2);
    valid.captured_at = Timestamp(2);
    let later = controller.handle(TimedEvent {
        at: Timestamp(2),
        event: Event::Sensor {
            sample: SensorSample::Lidar(valid),
        },
    });
    assert_eq!(later.state, State::Fault);
    assert_eq!(later.output.command, MotionOutput::Stop);
}
