use xt_stcar_robot_core::autonomy::{Pose2, PoseEstimate};
use xt_stcar_robot_core::startup_assist::*;
use xt_stcar_robot_core::{FrameId, MotionOutput, Timestamp};

fn config() -> StartupConfig {
    let value: serde_json::Value =
        serde_json::from_str(include_str!("../../../config/startup-assist-sim.json")).unwrap();
    serde_json::from_value(value["startup"].clone()).unwrap()
}
fn input(t: u64, x: f64, speed: f64) -> StartupInput {
    StartupInput {
        now: Timestamp(t),
        command: MotionOutput::Drive {
            speed_mps: 0.1,
            curvature_per_m: 0.0,
        },
        nominal_motor_us: 1510,
        nominal_servo_us: 1500,
        permit: Some(StartupPermit {
            issued_at: Timestamp(t),
            valid_until: Timestamp(t + 200),
            max_motor_us: 1514,
            max_speed_mps: 0.3,
        }),
        feedback: Some(StartupFeedback {
            accepted: true,
            localization_epoch: 1,
            estimate: PoseEstimate {
                captured_at: Timestamp(t),
                frame_id: FrameId("map".into()),
                pose: Pose2 {
                    x_m: x,
                    y_m: 0.0,
                    yaw_rad: 0.0,
                },
                speed_mps: speed,
                yaw_rate_radps: 0.0,
                quality: 0.95,
            },
        }),
    }
}
fn machine() -> StartupAssist {
    StartupAssist::new(config()).unwrap()
}
fn motor(d: &StartupDecision) -> Option<u16> {
    match d.action {
        StartupAction::Preview { command } | StartupAction::Stop { command } => {
            let b = command.encode();
            Some(u16::from_le_bytes([b[1], b[2]]))
        }
        StartupAction::Handoff => None,
    }
}
fn ramp() -> StartupAssist {
    let mut m = machine();
    for t in [0, 100, 200, 300] {
        m.update(input(t, 0.0, 0.0));
    }
    m
}
#[test]
fn stationary_window_then_one_step_per_full_dwell_and_cap_stop() {
    let mut m = machine();
    for t in (0..=3300).step_by(100) {
        let d = m.update(input(t, 0.0, 0.0));
        assert_eq!(
            motor(&d),
            Some(match t {
                0..=299 => 1500,
                300..=1299 => 1510,
                1300..=2299 => 1512,
                2300..=3299 => 1514,
                _ => 1500,
            })
        );
        if t == 3300 {
            assert_eq!(d.phase, StartupPhase::Fault);
            assert_eq!(d.reason, "startup_pwm_limit_without_motion");
        }
    }
}
#[test]
fn two_motion_samples_end_boost_and_do_not_reboost_after_stall() {
    let mut m = ramp();
    assert_eq!(m.update(input(400, 0.04, 0.1)).phase, StartupPhase::Ramping);
    let d = m.update(input(500, 0.05, 0.1));
    assert_eq!(d.phase, StartupPhase::Moving);
    assert!(matches!(d.action, StartupAction::Handoff));
    // A normal speed controller may change its nominal request after handoff.
    let mut next = input(600, 0.06, 0.1);
    next.nominal_motor_us = 1511;
    next.command = MotionOutput::Drive {
        speed_mps: 0.12,
        curvature_per_m: 0.01,
    };
    assert!(matches!(m.update(next).action, StartupAction::Handoff));
    for t in [700, 800, 900] {
        let d = m.update(input(t, 0.06, 0.0));
        if t == 900 {
            assert_eq!(d.reason, "stalled_after_start_no_automatic_retry");
            assert_eq!(motor(&d), Some(1500));
        }
    }
}
#[test]
fn transient_rocking_is_not_stationarity_or_confirmed_progress() {
    let mut m = ramp();
    for t in (400..=3900).step_by(100) {
        let x = if t % 200 == 0 { 0.02 } else { 0.0 };
        let d = m.update(input(t, x, 0.01));
        assert_eq!(d.phase, StartupPhase::Ramping);
        assert_eq!(motor(&d), Some(1510));
    }
    assert_eq!(m.update(input(4000, 0.0, 0.0)).reason, "startup_timeout");
}
#[test]
fn old_frames_cannot_count_as_stationary_and_expire() {
    let mut m = machine();
    m.update(input(0, 0.0, 0.0));
    for t in [50, 100, 150, 200] {
        let mut i = input(t, 0.0, 0.0);
        i.feedback.as_mut().unwrap().estimate.captured_at = Timestamp(0);
        assert_eq!(motor(&m.update(i)), Some(1500));
    }
    let mut i = input(250, 0.0, 0.0);
    i.feedback = None;
    assert_eq!(m.update(i).reason, "feedback_stale");
}
#[test]
fn faults_latch_through_stop_and_good_observations() {
    let mut m = ramp();
    let mut i = input(400, 0.0, 0.0);
    i.permit = None;
    let d = m.update(i);
    assert_eq!(d.reason, "permit_missing");
    let mut stop = input(500, 0.0, 0.0);
    stop.command = MotionOutput::Stop;
    assert_eq!(m.update(stop).phase, StartupPhase::Fault);
    assert_eq!(motor(&m.update(input(600, 0.0, 0.0))), Some(1500));
}
#[test]
fn stop_and_zero_request_never_boost() {
    for zero_drive in [false, true] {
        let mut m = ramp();
        let mut i = input(400, 0.0, 0.0);
        i.command = if zero_drive {
            MotionOutput::Drive {
                speed_mps: 0.0,
                curvature_per_m: 0.2,
            }
        } else {
            MotionOutput::Stop
        };
        i.permit = None;
        i.feedback = None;
        let d = m.update(i);
        assert_eq!(d.phase, StartupPhase::Idle);
        assert_eq!(motor(&d), Some(1500));
    }
}
#[test]
fn invalid_feedback_and_authorization_always_stop() {
    for kind in 0..12 {
        let mut m = ramp();
        let mut i = input(400, 0.0, 0.0);
        match kind {
            0 => i.feedback.as_mut().unwrap().accepted = false,
            1 => i.feedback.as_mut().unwrap().estimate.quality = 0.1,
            2 => i.feedback.as_mut().unwrap().estimate.pose.x_m = f64::NAN,
            3 => i.feedback.as_mut().unwrap().estimate.frame_id = FrameId("other".into()),
            4 => i.feedback.as_mut().unwrap().localization_epoch = 2,
            5 => i.feedback.as_mut().unwrap().estimate.captured_at = Timestamp(500),
            6 => i.feedback.as_mut().unwrap().estimate.captured_at = Timestamp(200),
            7 => i.permit.as_mut().unwrap().valid_until = Timestamp(400),
            8 => i.permit.as_mut().unwrap().max_motor_us = 1509,
            9 => i.permit.as_mut().unwrap().max_speed_mps = f64::NAN,
            10 => i.feedback.as_mut().unwrap().estimate.speed_mps = 0.4,
            _ => {
                i.command = MotionOutput::Drive {
                    speed_mps: -0.1,
                    curvature_per_m: 0.0,
                }
            }
        }
        let d = m.update(i);
        assert_eq!(d.phase, StartupPhase::Fault, "case {kind}");
        assert_eq!(motor(&d), Some(1500));
    }
}
#[test]
fn backward_sideways_and_turning_motion_do_not_trigger_handoff() {
    for (x, y, yaw) in [(-0.04, 0.0, 0.0), (0.04, 0.1, 0.0), (0.04, 0.0, 0.2)] {
        let mut m = ramp();
        let mut i = input(400, x, 0.1);
        let p = &mut i.feedback.as_mut().unwrap().estimate.pose;
        p.y_m = y;
        p.yaw_rad = yaw;
        assert_eq!(m.update(i).reason, "unexpected_motion");
    }
}
#[test]
fn control_gap_and_time_regression_do_not_catch_up_steps() {
    let mut m = ramp();
    assert_eq!(m.update(input(1600, 0.0, 0.0)).reason, "control_gap");
    let mut m = ramp();
    assert_eq!(
        m.update(input(300, 0.0, 0.0)).reason,
        "control_time_not_increasing"
    );
}
#[test]
fn rejected_localization_with_no_pose_stops_immediately() {
    let mut m = ramp();
    assert_eq!(
        m.reject_localization(Timestamp(400)).phase,
        StartupPhase::Fault
    );
    assert_eq!(motor(&m.update(input(500, 0.0, 0.0))), Some(1500));
}
#[test]
fn disabled_is_default_and_bad_configuration_is_rejected() {
    let mut v = serde_json::to_value(config()).unwrap();
    v.as_object_mut().unwrap().remove("enabled");
    let c: StartupConfig = serde_json::from_value(v).unwrap();
    assert!(!c.enabled);
    assert!(matches!(
        StartupAssist::new(c)
            .unwrap()
            .update(input(0, 0.0, 0.0))
            .action,
        StartupAction::Handoff
    ));
    for kind in 0..7 {
        let mut c = config();
        match kind {
            0 => c.step_us = 0,
            1 => c.max_motor_us = 1490,
            2 => c.step_wait_ms = 10,
            3 => c.moving_distance_m = c.stationary_distance_m,
            4 => c.min_quality = f64::NAN,
            5 => c.max_attempt_ms = 30001,
            _ => c.min_motion_samples = 1,
        };
        assert!(StartupAssist::new(c).is_err());
    }
}

#[test]
fn brief_displacement_then_return_cannot_restart_pwm_increases() {
    let mut m = ramp();
    m.update(input(400, 0.02, 0.01));
    for t in (500..=3900).step_by(100) {
        let d = m.update(input(t, 0.0, 0.0));
        assert_eq!(motor(&d), Some(1510));
    }
    assert_eq!(m.update(input(4000, 0.0, 0.0)).phase, StartupPhase::Fault);
}
#[test]
fn localization_adapter_rejects_missing_estimate_and_respects_stop() {
    use xt_stcar_robot_core::localization::LocalizationUpdate;
    let mut m = machine();
    for t in [0, 100, 200, 300] {
        let i = input(t, 0.0, 0.0);
        let u = LocalizationUpdate {
            estimate: Some(i.feedback.as_ref().unwrap().estimate.clone()),
            accepted: true,
            reason: None,
            matched_points: 200,
            rmse_m: Some(0.001),
        };
        let d = m.update_with_localization(i, 1, &u);
        if t == 300 {
            assert_eq!(d.phase, StartupPhase::Ramping);
        }
    }
    let bad = LocalizationUpdate {
        estimate: None,
        accepted: true,
        reason: None,
        matched_points: 0,
        rmse_m: None,
    };
    assert_eq!(
        m.update_with_localization(input(400, 0.0, 0.0), 1, &bad)
            .phase,
        StartupPhase::Fault
    );
    let mut m = ramp();
    let mut i = input(400, 0.0, 0.0);
    i.command = MotionOutput::Stop;
    assert_eq!(
        m.update_with_localization(i, 1, &bad).phase,
        StartupPhase::Idle
    );
}

#[test]
fn late_motion_at_step_deadline_prevents_another_increment() {
    let mut m = ramp();
    for t in (400..1300).step_by(100) {
        m.update(input(t, 0.0, 0.0));
    }
    let d = m.update(input(1300, 0.04, 0.1));
    assert_eq!(motor(&d), Some(1510));
    assert!(matches!(
        m.update(input(1400, 0.05, 0.1)).action,
        StartupAction::Handoff
    ));
}
#[test]
fn reduced_permit_and_request_change_stop_existing_boost() {
    let mut m = ramp();
    for t in (400..=1300).step_by(100) {
        m.update(input(t, 0.0, 0.0));
    }
    let mut i = input(1400, 0.0, 0.0);
    i.permit.as_mut().unwrap().max_motor_us = 1511;
    assert_eq!(m.update(i).reason, "authorized_pwm_limit");
    let mut m = ramp();
    let mut i = input(400, 0.0, 0.0);
    i.nominal_servo_us = 1510;
    assert_eq!(m.update(i).reason, "request_changed_requires_stop");
}
#[test]
fn excessive_start_displacement_and_modified_duplicate_are_rejected() {
    let mut m = ramp();
    assert_eq!(m.update(input(400, 0.16, 0.1)).reason, "unexpected_motion");
    let mut m = ramp();
    let mut i = input(400, 0.01, 0.0);
    i.feedback.as_mut().unwrap().estimate.captured_at = Timestamp(300);
    assert_eq!(m.update(i).reason, "changed_duplicate_feedback");
}

#[test]
fn scan_odometry_pipeline_supplies_motion_feedback_without_commanded_speed_substitution() {
    use xt_stcar_robot_core::autonomy::Point2;
    use xt_stcar_robot_core::localization::{LocalizationConfig, ScanOdometry};
    let scene: Vec<Point2> = (0..64)
        .map(|i| {
            let a = i as f64 * std::f64::consts::TAU / 64.0;
            let r = 2.0 + 0.2 * (3.0 * a).sin();
            Point2 {
                x_m: r * a.cos(),
                y_m: r * a.sin(),
            }
        })
        .collect();
    let mut odom = ScanOdometry::new(
        LocalizationConfig::simulation(FrameId("map".into())),
        Pose2::default(),
    )
    .unwrap();
    // Localization warmup occurs before arming the assistance state machine.
    assert!(!odom.update(Timestamp(0), &scene).unwrap().accepted);
    let mut assist = machine();
    for t in (100..=1800).step_by(100) {
        let dx = if t > 1400 {
            (t - 1400) as f64 / 100.0 * 0.015
        } else {
            0.0
        };
        let points: Vec<_> = scene
            .iter()
            .map(|p| Point2 {
                x_m: p.x_m - dx,
                y_m: p.y_m,
            })
            .collect();
        let update = odom.update(Timestamp(t), &points).unwrap();
        assert!(update.accepted, "{:?}", update.reason);
        let d = assist.update_with_localization(input(t, 0.0, 0.0), 1, &update);
        assert_ne!(d.phase, StartupPhase::Fault, "{}", d.reason);
        if t == 1400 {
            assert_eq!(motor(&d), Some(1512));
        }
        if t == 1800 {
            assert_eq!(d.phase, StartupPhase::Moving);
            assert!(matches!(d.action, StartupAction::Handoff));
        }
    }
}
