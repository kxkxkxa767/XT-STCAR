use xt_stcar_robot_core::MotionOutput;
use xt_stcar_robot_core::protocol::calibrated_chassis::{
    CalibratedChassis, ChassisCalibration, CurvaturePoint, PwmRange, ReversePolicy, SpeedPoint,
};
use xt_stcar_robot_core::protocol::chassis::PwmCommand;

fn config() -> ChassisCalibration {
    serde_json::from_str(include_str!("../../../config/chassis-calibration-sim.json")).unwrap()
}

fn drive(speed_mps: f64, curvature_per_m: f64) -> MotionOutput {
    MotionOutput::Drive {
        speed_mps,
        curvature_per_m,
    }
}

#[test]
fn nonlinear_points_interpolate_in_physical_units() {
    let mapper = CalibratedChassis::new(config()).unwrap();
    // Synthetic non-linear speed table: the second interval is not a factory gain.
    assert_eq!(
        mapper.preview(&drive(0.15, -0.25)).unwrap(),
        PwmCommand::new(1565, 1450).unwrap()
    );
    assert_eq!(
        mapper.preview(&drive(0.2, 0.5)).unwrap(),
        PwmCommand::new(1600, 1600).unwrap()
    );
    assert_eq!(
        mapper.preview(&drive(0.1, -0.5)).unwrap(),
        PwmCommand::new(1530, 1400).unwrap()
    );
    // The smallest valid nonzero input exercises a sub-microsecond result.
    assert_eq!(
        mapper.preview(&drive(f64::from_bits(1), 0.0)).unwrap(),
        mapper.stop_command()
    );
}

#[test]
fn stop_and_zero_use_explicit_neutrals_without_reusing_a_previous_turn() {
    let mut c = config();
    c.stop_motor_us = 1510;
    c.speed_points[0].motor_us = 1510;
    c.stop_servo_us = 1490;
    c.curvature_points[1].servo_us = 1490;
    let mapper = CalibratedChassis::new(c).unwrap();
    let _ = mapper.preview(&drive(0.2, 0.5)).unwrap();
    let neutral = PwmCommand::new(1510, 1490).unwrap();
    assert_eq!(mapper.preview(&MotionOutput::Stop).unwrap(), neutral);
    assert_eq!(mapper.preview(&drive(-0.0, -0.0)).unwrap(), neutral);
    // A zero-speed Drive can request steering in preview; Stop centers it.
    assert_ne!(mapper.preview(&drive(0.0, 0.5)).unwrap(), neutral);
    assert_eq!(mapper.stop_command(), neutral);
}

#[test]
fn out_of_table_values_and_nonfinite_commands_are_rejected_without_clipping() {
    let mapper = CalibratedChassis::new(config()).unwrap();
    for output in [
        drive(-0.01, 0.0),
        drive(0.20000001, 0.0),
        drive(0.1, -0.50000001),
        drive(0.1, 0.50000001),
        drive(f64::NAN, 0.0),
        drive(f64::INFINITY, 0.0),
        drive(f64::NEG_INFINITY, 0.0),
        drive(0.0, f64::NAN),
        drive(0.0, f64::INFINITY),
        drive(0.0, f64::NEG_INFINITY),
    ] {
        assert!(mapper.preview(&output).is_err(), "accepted {output:?}");
    }
    assert_eq!(
        mapper.preview(&MotionOutput::Stop).unwrap(),
        mapper.stop_command()
    );
}

#[test]
fn reverse_needs_a_separate_negative_table_and_keeps_signed_curvature() {
    let mut c = config();
    c.reverse_policy = ReversePolicy::CalibratedSignedSpeed;
    assert!(CalibratedChassis::new(c.clone()).is_err());
    c.motor_pwm_range.min_us = 1400;
    c.speed_points.insert(
        0,
        SpeedPoint {
            speed_mps: -0.1,
            motor_us: 1400,
        },
    );
    let mapper = CalibratedChassis::new(c.clone()).unwrap();
    assert_eq!(
        mapper.preview(&drive(-0.05, 0.25)).unwrap(),
        PwmCommand::new(1450, 1550).unwrap()
    );
    assert!(mapper.preview(&drive(-0.10001, 0.0)).is_err());
    c.reverse_policy = ReversePolicy::Disabled;
    assert!(CalibratedChassis::new(c).is_err());
}

#[test]
fn reversed_pwm_directions_are_explicit_and_accepted() {
    let mut c = config();
    c.motor_pwm_range = PwmRange {
        min_us: 1400,
        max_us: 1500,
    };
    for p in &mut c.speed_points {
        p.motor_us = 3000 - p.motor_us;
    }
    for p in &mut c.curvature_points {
        p.servo_us = 3000 - p.servo_us;
    }
    let mapper = CalibratedChassis::new(c).unwrap();
    assert_eq!(
        mapper.preview(&drive(0.15, 0.25)).unwrap(),
        PwmCommand::new(1435, 1450).unwrap()
    );
}

#[test]
fn invalid_tables_are_rejected_at_construction() {
    let mut cases = Vec::new();
    let mut c = config();
    c.speed_points[1].speed_mps = 0.0;
    cases.push(c);
    let mut c = config();
    c.speed_points.swap(1, 2);
    cases.push(c);
    let mut c = config();
    c.speed_points[1].motor_us = c.speed_points[2].motor_us;
    cases.push(c);
    let mut c = config();
    c.speed_points[1].motor_us = 1590;
    c.speed_points[2].motor_us = 1530;
    cases.push(c);
    let mut c = config();
    c.speed_points[0].speed_mps = 0.01;
    cases.push(c);
    let mut c = config();
    c.stop_motor_us = 1501;
    cases.push(c);
    let mut c = config();
    c.stop_servo_us = 1501;
    cases.push(c);
    let mut c = config();
    c.speed_points[2].speed_mps = f64::NAN;
    cases.push(c);
    let mut c = config();
    c.curvature_points[2].curvature_per_m = f64::INFINITY;
    cases.push(c);
    let mut c = config();
    c.curvature_points = vec![
        CurvaturePoint {
            curvature_per_m: -f64::MAX,
            servo_us: 1400,
        },
        CurvaturePoint {
            curvature_per_m: f64::MAX,
            servo_us: 1600,
        },
    ];
    cases.push(c);
    let mut c = config();
    c.curvature_points.remove(0);
    cases.push(c);
    let mut c = config();
    c.speed_points.clear();
    cases.push(c);
    let mut c = config();
    c.speed_points.truncate(1);
    cases.push(c);
    let mut c = config();
    c.speed_points.resize(1025, c.speed_points[0]);
    cases.push(c);
    for c in cases {
        assert!(CalibratedChassis::new(c).is_err());
    }
}

#[test]
fn pwm_envelopes_are_distinct_from_protocol_limits() {
    let mut cases = Vec::new();
    let mut c = config();
    c.motor_pwm_range.min_us = 499;
    cases.push(c);
    let mut c = config();
    c.servo_pwm_range.max_us = 2501;
    cases.push(c);
    let mut c = config();
    c.motor_pwm_range.max_us = 1500;
    cases.push(c);
    let mut c = config();
    c.servo_pwm_range.min_us = 1601;
    cases.push(c);
    let mut c = config();
    c.speed_points[2].motor_us = 1700; // Legal protocol value, outside calibration.
    cases.push(c);
    let mut c = config();
    c.stop_motor_us = 1499;
    cases.push(c);
    for c in cases {
        assert!(CalibratedChassis::new(c).is_err());
    }
}

#[test]
fn config_rejects_live_claims_unknown_fields_and_implicit_defaults() {
    let mut c = config();
    c.simulation_only = false;
    assert!(CalibratedChassis::new(c).is_err());
    let mut c = config();
    c.schema_version = 2;
    assert!(CalibratedChassis::new(c).is_err());
    let json = serde_json::to_value(config()).unwrap();
    let mut altered = json.clone();
    altered["measurement_status"] = "verified".into();
    assert!(serde_json::from_value::<ChassisCalibration>(altered).is_err());
    let mut altered = json.clone();
    altered["speed_points"][0]["speed_kmh"] = 0.0.into();
    assert!(serde_json::from_value::<ChassisCalibration>(altered).is_err());
    let mut altered = json;
    altered.as_object_mut().unwrap().remove("reverse_policy");
    assert!(serde_json::from_value::<ChassisCalibration>(altered).is_err());
}
