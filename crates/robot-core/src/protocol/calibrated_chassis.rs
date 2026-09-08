//! Explicit physical-unit lookup tables for offline PWM previews.
//!
//! The factory gains are deliberately absent. This mapper neither opens a device
//! nor proves that a neutral PWM stops a vehicle; the current schema only accepts
//! unverified simulation configurations.
use super::chassis::PwmCommand;
use crate::{MotionOutput, ValidationError};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum MeasurementStatus {
    Unverified,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ReversePolicy {
    Disabled,
    /// Requires explicit negative-speed entries; curvature keeps its geometric
    /// sign during reverse, so the intended yaw rate is speed * curvature.
    /// The configuration still remains an unverified simulation preview.
    CalibratedSignedSpeed,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct PwmRange {
    pub min_us: u16,
    pub max_us: u16,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct SpeedPoint {
    pub speed_mps: f64,
    pub motor_us: u16,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct CurvaturePoint {
    pub curvature_per_m: f64,
    pub servo_us: u16,
}

/// No defaults: units, zero points, envelope and reverse behavior are explicit.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct ChassisCalibration {
    pub schema_version: u32,
    pub simulation_only: bool,
    pub measurement_status: MeasurementStatus,
    pub motor_pwm_range: PwmRange,
    pub servo_pwm_range: PwmRange,
    pub stop_motor_us: u16,
    pub stop_servo_us: u16,
    pub reverse_policy: ReversePolicy,
    pub speed_points: Vec<SpeedPoint>,
    pub curvature_points: Vec<CurvaturePoint>,
}

struct Table {
    points: Vec<(f64, u16)>,
    envelope: PwmRange,
    name: &'static str,
}

impl Table {
    fn new(
        points: Vec<(f64, u16)>,
        envelope: PwmRange,
        neutral: u16,
        name: &'static str,
    ) -> Result<Self, ValidationError> {
        let fail = |why: &str| ValidationError(format!("{name}: {why}"));
        if envelope.min_us < 500
            || envelope.max_us > 2500
            || envelope.min_us >= envelope.max_us
            || !(envelope.min_us..=envelope.max_us).contains(&neutral)
        {
            return Err(fail("explicit PWM range and neutral must fit 500..2500 us"));
        }
        if !(2..=1024).contains(&points.len()) {
            return Err(fail("table must contain 2..1024 points"));
        }
        if points.iter().any(|&(input, pwm)| {
            !input.is_finite() || !(envelope.min_us..=envelope.max_us).contains(&pwm)
        }) {
            return Err(fail(
                "table inputs must be finite and PWM must fit its range",
            ));
        }
        let increasing_pwm = points[1].1 > points[0].1;
        for pair in points.windows(2) {
            let ((x0, y0), (x1, y1)) = (pair[0], pair[1]);
            if x1 <= x0 || !(x1 - x0).is_finite() {
                return Err(fail(
                    "physical inputs must strictly increase with finite spans",
                ));
            }
            if y0 == y1 || (y1 > y0) != increasing_pwm {
                return Err(fail(
                    "PWM values must be strictly monotonic in one direction",
                ));
            }
        }
        if !points.iter().any(|&(x, y)| x == 0.0 && y == neutral) {
            return Err(fail(
                "table requires an exact zero anchor at the Stop neutral PWM",
            ));
        }
        Ok(Self {
            points,
            envelope,
            name,
        })
    }

    fn interpolate(&self, input: f64) -> Result<u16, ValidationError> {
        if !input.is_finite()
            || input < self.points[0].0
            || input > self.points[self.points.len() - 1].0
        {
            return Err(ValidationError(format!(
                "{}: input is non-finite or outside the calibrated table; extrapolation is disabled",
                self.name
            )));
        }
        let upper = self.points.partition_point(|&(x, _)| x < input);
        if self.points[upper].0 == input {
            return Ok(self.points[upper].1);
        }
        let (x0, y0) = self.points[upper - 1];
        let (x1, y1) = self.points[upper];
        let fraction = (input - x0) / (x1 - x0);
        let pwm = (f64::from(y0) + fraction * (f64::from(y1) - f64::from(y0))).round();
        // Validate before casting even though construction already bounds both
        // interpolation endpoints. No clamp or saturating conversion is used.
        if !pwm.is_finite()
            || pwm < f64::from(self.envelope.min_us)
            || pwm > f64::from(self.envelope.max_us)
        {
            return Err(ValidationError(format!(
                "{}: interpolated PWM exceeds the explicit range",
                self.name
            )));
        }
        Ok(pwm as u16)
    }
}

pub struct CalibratedChassis {
    config: ChassisCalibration,
    motor: Table,
    servo: Table,
    stop: PwmCommand,
}

impl CalibratedChassis {
    pub fn new(config: ChassisCalibration) -> Result<Self, ValidationError> {
        if config.schema_version != 1 || !config.simulation_only {
            return Err(ValidationError(
                "chassis calibration requires schema_version=1 and simulation_only=true".into(),
            ));
        }
        let motor = Table::new(
            config
                .speed_points
                .iter()
                .map(|point| (point.speed_mps, point.motor_us))
                .collect(),
            config.motor_pwm_range,
            config.stop_motor_us,
            "speed_mps -> motor_us",
        )?;
        let servo = Table::new(
            config
                .curvature_points
                .iter()
                .map(|point| (point.curvature_per_m, point.servo_us))
                .collect(),
            config.servo_pwm_range,
            config.stop_servo_us,
            "curvature_per_m -> servo_us",
        )?;
        let first_speed = motor.points[0].0;
        let last_speed = motor.points[motor.points.len() - 1].0;
        let reverse_valid = match config.reverse_policy {
            ReversePolicy::Disabled => first_speed == 0.0 && last_speed > 0.0,
            ReversePolicy::CalibratedSignedSpeed => first_speed < 0.0 && last_speed > 0.0,
        };
        if !reverse_valid {
            return Err(ValidationError(
                "speed table must include forward motion; reverse requires explicit calibrated_signed_speed and negative entries".into(),
            ));
        }
        if servo.points[0].0 >= 0.0 || servo.points[servo.points.len() - 1].0 <= 0.0 {
            return Err(ValidationError(
                "curvature table must cover both sides of its zero anchor".into(),
            ));
        }
        let stop = PwmCommand::new(config.stop_motor_us, config.stop_servo_us)?;
        Ok(Self {
            config,
            motor,
            servo,
            stop,
        })
    }

    pub fn config(&self) -> &ChassisCalibration {
        &self.config
    }

    /// The configured neutral frame, not an acknowledgment of physical stopping.
    pub fn stop_command(&self) -> PwmCommand {
        self.stop
    }

    /// Map an existing controller output without changing its state or reason.
    /// An error is returned rather than clipping or substituting a Drive value.
    pub fn preview(&self, output: &MotionOutput) -> Result<PwmCommand, ValidationError> {
        match *output {
            MotionOutput::Stop => Ok(self.stop),
            MotionOutput::Drive {
                speed_mps,
                curvature_per_m,
            } => {
                if self.config.reverse_policy == ReversePolicy::Disabled && speed_mps < 0.0 {
                    return Err(ValidationError("reverse motion is disabled".into()));
                }
                PwmCommand::new(
                    self.motor.interpolate(speed_mps)?,
                    self.servo.interpolate(curvature_per_m)?,
                )
            }
        }
    }
}
