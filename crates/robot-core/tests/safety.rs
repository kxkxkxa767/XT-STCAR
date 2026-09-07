use xt_stcar_robot_core::*;

fn config() -> SafetyConfig {
    serde_json::from_str(include_str!("../../../config/robot-sim.json")).unwrap()
}
fn event(controller: &mut Controller, at: u64, value: Event) -> StepReport {
    controller.handle(TimedEvent {
        at: Timestamp(at),
        event: value,
    })
}
fn command() -> Event {
    Event::Motion {
        intent: MotionIntent {
            speed_mps: 0.1,
            curvature_per_m: 0.0,
        },
    }
}
fn ready(config: SafetyConfig) -> Controller {
    let mut controller = Controller::new(config).unwrap();
    for line in include_str!("../../../examples/robot-sim.jsonl")
        .lines()
        .take(8)
    {
        controller.handle(serde_json::from_str(line).unwrap());
    }
    assert_eq!(controller.state(), State::Running);
    controller
}
fn assert_stop(report: &StepReport) {
    assert_eq!(report.output.command, MotionOutput::Stop);
    assert_eq!(report.state, State::Fault);
}

#[test]
fn sample_replay_is_deterministic_and_offline() {
    let mut first = Controller::new(config()).unwrap();
    let mut second = Controller::new(config()).unwrap();
    let mut sink = RecordingSink::default();
    let mut drive_count = 0;
    for line in include_str!("../../../examples/robot-sim.jsonl").lines() {
        let input: TimedEvent = serde_json::from_str(line).unwrap();
        let report = first.handle(input.clone());
        assert_eq!(report, second.handle(input));
        drive_count += usize::from(matches!(report.output.command, MotionOutput::Drive { .. }));
        sink.emit(&report.output).unwrap();
    }
    assert_eq!(drive_count, 2);
    assert_eq!(sink.records().len(), 17);
    assert_eq!(sink.records().last().unwrap().state, State::Disarmed);
    assert_eq!(sink.into_records().len(), 17);
}

#[test]
fn arming_and_starting_require_explicit_readiness() {
    let mut controller = Controller::new(config()).unwrap();
    assert_eq!(event(&mut controller, 0, command()).state, State::Disarmed);
    assert_stop(&event(&mut controller, 1, Event::Start));
    let mut controller = Controller::new(config()).unwrap();
    event(&mut controller, 0, Event::Deadman { pressed: true });
    let report = event(&mut controller, 1, Event::Arm);
    assert_stop(&report);
    assert_eq!(report.output.reason, Some(StopReason::HeartbeatMissing));
}

#[test]
fn timeout_boundary_and_late_heartbeat_cannot_restore_motion() {
    let mut controller = ready(config());
    assert!(matches!(
        controller.tick(Timestamp(151)).output.command,
        MotionOutput::Drive { .. }
    ));
    let report = event(&mut controller, 152, Event::Heartbeat);
    assert_stop(&report);
    assert_eq!(report.output.reason, Some(StopReason::CommandExpired));
    assert_stop(&event(&mut controller, 153, command()));
    assert_stop(&event(&mut controller, 154, Event::Arm));
}

#[test]
fn late_command_cannot_erase_its_own_expiry() {
    let mut controller = ready(config());
    let report = event(&mut controller, 152, command());
    assert_stop(&report);
    assert_eq!(report.output.reason, Some(StopReason::CommandExpired));
}

#[test]
fn each_keepalive_expires_and_late_input_remains_stopped() {
    for (deadline, expected, input) in [
        ("heartbeat", StopReason::HeartbeatExpired, Event::Heartbeat),
        (
            "deadman",
            StopReason::DeadmanExpired,
            Event::Deadman { pressed: true },
        ),
        (
            "sensor",
            StopReason::SensorExpired {
                sensor: SensorKind::Imu,
            },
            Event::Tick,
        ),
    ] {
        let mut config = config();
        config.heartbeat_timeout_ms = 1000;
        config.deadman_timeout_ms = 1000;
        config.command_timeout_ms = 1000;
        config.sensor_timeout_ms = 1000;
        match deadline {
            "heartbeat" => config.heartbeat_timeout_ms = 50,
            "deadman" => config.deadman_timeout_ms = 50,
            _ => config.sensor_timeout_ms = 50,
        }
        let mut controller = ready(config);
        let report = event(&mut controller, 50, input);
        assert_stop(&report);
        assert_eq!(report.output.reason, Some(expected));
    }
}

#[test]
fn estop_latches_until_release_and_explicit_reset() {
    let mut controller = ready(config());
    let report = event(&mut controller, 10, Event::EmergencyStop);
    assert_stop(&report);
    assert!(report.emergency_stop_latched);
    assert_stop(&event(&mut controller, 11, Event::ResetFault));
    let report = event(&mut controller, 12, Event::Disarm);
    assert_stop(&report);
    assert!(report.emergency_stop_latched);
    assert_stop(&event(&mut controller, 13, Event::ResetFault));
    event(&mut controller, 14, Event::Deadman { pressed: false });
    let report = event(&mut controller, 15, Event::ResetFault);
    assert_eq!(report.state, State::Disarmed);
    assert!(!report.emergency_stop_latched);
    assert_eq!(report.output.command, MotionOutput::Stop);
    // Reset discards previous keepalives, sensor freshness and command.
    event(&mut controller, 16, Event::Deadman { pressed: true });
    assert_eq!(
        event(&mut controller, 17, Event::Arm).output.reason,
        Some(StopReason::HeartbeatMissing)
    );
}

#[test]
fn deadman_release_stops_immediately() {
    let mut controller = ready(config());
    let report = event(&mut controller, 4, Event::Deadman { pressed: false });
    assert_stop(&report);
    assert_eq!(report.output.reason, Some(StopReason::DeadmanReleased));
}

#[test]
fn invalid_or_excessive_motion_is_rejected_instead_of_clamped() {
    for intent in [
        MotionIntent {
            speed_mps: f64::NAN,
            curvature_per_m: 0.0,
        },
        MotionIntent {
            speed_mps: 0.0,
            curvature_per_m: f64::INFINITY,
        },
        MotionIntent {
            speed_mps: 0.20001,
            curvature_per_m: 0.0,
        },
        MotionIntent {
            speed_mps: -0.20001,
            curvature_per_m: 0.0,
        },
        MotionIntent {
            speed_mps: 0.1,
            curvature_per_m: -0.50001,
        },
    ] {
        let mut controller = ready(config());
        assert_stop(&event(&mut controller, 4, Event::Motion { intent }));
    }
    let mut controller = ready(config());
    let report = event(
        &mut controller,
        4,
        Event::Motion {
            intent: MotionIntent {
                speed_mps: -0.2,
                curvature_per_m: 0.5,
            },
        },
    );
    assert_eq!(report.state, State::Running);
}

#[test]
fn backward_event_time_faults_without_backward_output_time() {
    let mut controller = ready(config());
    controller.tick(Timestamp(10));
    let report = event(&mut controller, 9, Event::Heartbeat);
    assert_stop(&report);
    assert_eq!(report.event_at, Timestamp(9));
    assert_eq!(report.output.at, Timestamp(10));
    assert_eq!(report.output.reason, Some(StopReason::TimeRegression));
}

#[test]
fn missing_required_sensor_prevents_arm() {
    let mut cfg = config();
    cfg.required_sensors.push(SensorKind::Vision);
    let mut controller = Controller::new(cfg).unwrap();
    for line in include_str!("../../../examples/robot-sim.jsonl")
        .lines()
        .take(5)
    {
        controller.handle(serde_json::from_str(line).unwrap());
    }
    let report = event(&mut controller, 1, Event::Arm);
    assert_stop(&report);
    assert_eq!(
        report.output.reason,
        Some(StopReason::SensorMissing {
            sensor: SensorKind::Vision
        })
    );
}

#[test]
fn sensor_frames_stamps_and_geometry_are_validated() {
    let sample = VisionSample {
        captured_at: Timestamp(5),
        frame_id: FrameId("sim_camera".into()),
        image_width_px: 320,
        image_height_px: 320,
        detection_count: 1,
    };
    let mut controller = ready(config());
    assert_eq!(
        event(
            &mut controller,
            5,
            Event::Sensor {
                sample: SensorSample::Vision(sample.clone())
            }
        )
        .state,
        State::Running
    );
    let mut wrong = sample.clone();
    wrong.frame_id = FrameId("unmapped_camera".into());
    assert_stop(&event(
        &mut controller,
        6,
        Event::Sensor {
            sample: SensorSample::Vision(wrong),
        },
    ));
    let mut controller = ready(config());
    assert_stop(&event(
        &mut controller,
        4,
        Event::Sensor {
            sample: SensorSample::Vision(sample),
        },
    ));
    let invalid_imu = ImuSample {
        captured_at: Timestamp(4),
        frame_id: FrameId("sim_imu".into()),
        acceleration_mps2: Vec3 {
            x: 0.0,
            y: 0.0,
            z: 9.81,
        },
        angular_velocity_radps: Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        orientation_xyzw: Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 0.0,
        },
    };
    assert!(
        SensorSample::Imu(invalid_imu)
            .validate(&config().frames)
            .is_err()
    );
    let lidar = LidarSample {
        captured_at: Timestamp(0),
        frame_id: FrameId("sim_lidar".into()),
        angle_min_rad: 0.0,
        angle_increment_rad: 0.1,
        range_min_m: 0.1,
        range_max_m: 10.0,
        ranges_m: vec![None, Some(1.0)],
    };
    assert!(
        SensorSample::Lidar(lidar.clone())
            .validate(&config().frames)
            .is_ok()
    );
    let mut invalid = lidar;
    invalid.ranges_m = vec![Some(f64::INFINITY)];
    assert!(
        SensorSample::Lidar(invalid)
            .validate(&config().frames)
            .is_err()
    );
}

#[test]
fn config_is_explicit_finite_and_simulation_only() {
    let original = config();
    for index in 0..5 {
        let mut cfg = original.clone();
        match index {
            0 => cfg.simulation_only = false,
            1 => cfg.max_speed_mps = f64::NAN,
            2 => cfg.deadman_timeout_ms = 0,
            3 => cfg.required_sensors.push(SensorKind::Imu),
            _ => cfg.frames.body_frame = cfg.frames.odometry_frame.clone(),
        }
        assert!(Controller::new(cfg).is_err());
    }
    assert!(serde_json::from_str::<SafetyConfig>("{}").is_err());
}
