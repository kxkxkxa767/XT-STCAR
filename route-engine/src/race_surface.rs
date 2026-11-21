//! A time-local estimate of extra longitudinal load, shared by race speed holds.
//! It includes surface resistance, battery and drivetrain variation. It is not
//! an estimate of the tire's kinetic friction coefficient.

use crate::dict;
use crate::helpers::{NEUTRAL, SCAN_AGE_S};
use crate::maneuver::{HOLD_PWM_MAX, HOLD_PWM_STEP, LANE_COAST_DECEL_SE_MAX_MPS2, ROUTE_SLOW_MOTOR};
use crate::motion_sensors::ESC_COAST_DELAY_S;
use crate::py::V;

/// Maximum forward ESC command for race program 3, including any load compensation.
pub(crate) const FORWARD_PWM_MAX: i64 = 1560;

// y65: the indoor coast decel of y57-y64 pooled, IMU pairs .588 (n 86) and the lidar coast
// fits .615 (n 46); y57 had .574 from y53.
const REFERENCE_COAST_MPS2: f64 = 0.60;
const REFERENCE_ACCEL_MAX_MPS2: f64 = 4.14;
const FILTER_RESPONSE_S: f64 = 1.0;
const COMPENSATION_PWM_PER_S: f64 = 20.0;
const LEARNING_MIN_SPEED_MPS: f64 = 0.2;
const LEARNING_MAX_YAW_RADPS: f64 = 0.6;
// y65: only the load beyond the indoor floor is compensated. Replaying y57-y64 through this
// reference, the filtered load reached at most .065 m/s2 (its noise about .017); the indoor tile
// is the reference surface, so it should not change a hold.
const LOAD_DEADBAND_MPS2: f64 = 0.08;
/// Seconds after the crosswalk stop before the standstill measures the IMU forward bias.
pub const REST_SETTLE_S: f64 = 0.6;
const REST_MIN_WINDOW_S: f64 = 1.0;

// (PWM, asymptotic target m/s, first-order response s). 1570 and 1600 are y53's short response
// segments (y54 held out); the 1570 fit still matches y57-y64 (9 pushes, mean +.04 m/s2).
// y65: 1550 from the y57-y64 holds (43 learner-gated pairs at 1.62-2.21 m/s: a = .608 - .335 v,
// residual .079 m/s2); y53's 1.598 m/s made every 1550 sample above 1.6 m/s read -.1 to -.3 and
// pulled the learned load to 0 after the crosswalk. y53's 1560 anchor (1.600 m/s, weaker than
// 1550) has no gated sample in y57-y64, so 1555-1565 are interpolated between 1550 and 1570.
const RESPONSE: [(i64, f64, f64); 3] = [
    (1550, 1.814, 2.98),
    (1570, 2.340, 1.00),
    (1600, 3.837, 0.70),
];

fn reference_acceleration(pwm: i64, speed: f64) -> f64 {
    let mut target = RESPONSE[RESPONSE.len() - 1].1;
    let mut response_s = RESPONSE[RESPONSE.len() - 1].2;
    for pair in RESPONSE.windows(2) {
        if pwm <= pair[1].0 {
            let weight = ((pwm - pair[0].0) as f64 / (pair[1].0 - pair[0].0) as f64).clamp(0., 1.);
            target = pair[0].1 + weight * (pair[1].1 - pair[0].1);
            response_s = pair[0].2 + weight * (pair[1].2 - pair[0].2);
            break;
        }
    }
    ((target - speed) / response_s).clamp(-REFERENCE_COAST_MPS2, REFERENCE_ACCEL_MAX_MPS2)
}

fn hold_motor(pwm: i64) -> bool {
    (ROUTE_SLOW_MOTOR..=HOLD_PWM_MAX).contains(&pwm) && (pwm - ROUTE_SLOW_MOTOR) % HOLD_PWM_STEP == 0
}

#[derive(Clone, Copy, Debug)]
struct Observation {
    seq: i64,
    at: f64,
    motor: i64,
    speed: Option<f64>,
    integral: Option<f64>,
    yaw: Option<f64>,
}

#[derive(Clone, Debug, Default)]
pub struct SurfaceResponse {
    previous: Option<Observation>,
    motor_since: Option<f64>,
    extra_load_mps2: f64,
    initial_load_mps2: f64,
    last_load_sample_mps2: Option<f64>,
    learned_samples: u64,
    learning_source: Option<&'static str>,
    latest_actual_accel: Option<(f64, i64, f64)>,
    compensation_pwm: i64,
    compensation_credit: f64,
    compensation_at: Option<f64>,
    last_compensated_command: Option<(i64, f64)>,
    rest_anchor: Option<(f64, f64)>,
    rest_bias_mps2: f64,
}

impl SurfaceResponse {
    /// The load measured on this floor before the race (scripts/t-ground-drag.py); the
    /// learner carries on from it. Zero, the default, is the indoor reference floor.
    pub fn set_initial_load(&mut self, load_mps2: f64) {
        self.initial_load_mps2 = load_mps2.max(0.);
        self.extra_load_mps2 = self.initial_load_mps2;
    }

    /// y65: called each scan of the crosswalk standstill (from REST_SETTLE_S after the stop):
    /// the forward integral's slope there is the IMU forward bias at this spot (mounting pitch
    /// and floor slope; y57-y64 indoors -.135 to +.039 m/s2). The learner's IMU samples subtract
    /// it; the other stages read the integral unchanged.
    pub fn observe_rest(&mut self, now: f64, integral: Option<f64>) {
        let Some(integral) = integral.filter(|v| v.is_finite()) else { return };
        if !now.is_finite() {
            return;
        }
        match self.rest_anchor {
            None => self.rest_anchor = Some((now, integral)),
            Some((at, start)) if now - at >= REST_MIN_WINDOW_S => self.rest_bias_mps2 = (integral - start) / (now - at),
            Some(_) => {}
        }
    }

    /// The caller supplies the current velocity and a coast fit only while the
    /// neutral coast that produced it is still observable. A repeated laser
    /// sequence cannot contribute another learning update.
    #[allow(clippy::too_many_arguments)]
    pub fn observe(
        &mut self,
        seq: i64,
        now: f64,
        acknowledged_motor: i64,
        speed: Option<f64>,
        integral: Option<f64>,
        yaw: Option<f64>,
        coast_fit: Option<(f64, f64, f64)>,
    ) {
        if seq < 0 || !now.is_finite() || self.previous.is_some_and(|last| seq <= last.seq) {
            return;
        }
        let current = Observation {
            seq,
            at: now,
            motor: acknowledged_motor,
            speed: speed.filter(|v| v.is_finite() && *v >= 0.),
            integral: integral.filter(|v| v.is_finite()),
            yaw: yaw.filter(|v| v.is_finite()),
        };
        let previous = self.previous.replace(current);
        let Some(previous) = previous else {
            self.motor_since = Some(now);
            return;
        };
        let dt = now - previous.at;
        if previous.motor != acknowledged_motor || !(0. < dt && dt <= SCAN_AGE_S) {
            self.motor_since = Some(now);
            return;
        }
        if self.motor_since.is_none_or(|since| previous.at - since < ESC_COAST_DELAY_S)
            || !current.speed.is_some_and(|v| v >= LEARNING_MIN_SPEED_MPS)
            || !previous.speed.is_some_and(|v| v >= LEARNING_MIN_SPEED_MPS)
            || !current.yaw.is_some_and(|v| v.abs() <= LEARNING_MAX_YAW_RADPS)
            || !previous.yaw.is_some_and(|v| v.abs() <= LEARNING_MAX_YAW_RADPS)
        {
            return;
        }

        let sample = if acknowledged_motor == NEUTRAL {
            coast_fit.filter(|(decel, error, rolling)| {
                decel.is_finite() && *decel > 0. && error.is_finite() && *error >= 0.
                    && *error <= LANE_COAST_DECEL_SE_MAX_MPS2
                    && rolling.is_finite() && *rolling >= LEARNING_MIN_SPEED_MPS
            }).map(|(decel, _, _)| (decel - REFERENCE_COAST_MPS2, "neutral_coast_fit"))
                .or_else(|| previous.integral.zip(current.integral).map(|(before, after)| {
                    (-((after - before) / dt - self.rest_bias_mps2) - REFERENCE_COAST_MPS2, "neutral_imu_response")
                }))
        } else if hold_motor(acknowledged_motor) {
            previous.integral.zip(current.integral).map(|(before, after)| {
                let measured_accel = (after - before) / dt - self.rest_bias_mps2;
                let mean_speed = (previous.speed.unwrap() + current.speed.unwrap()) * 0.5;
                if measured_accel.is_finite() {
                    self.latest_actual_accel = Some((now, acknowledged_motor, measured_accel));
                }
                (reference_acceleration(acknowledged_motor, mean_speed) - measured_accel, "powered_response")
            })
        } else {
            None
        };
        if let Some((extra, source)) = sample.filter(|(extra, _)| extra.is_finite()) {
            // Use this scan interval, not the time since the last eligible sample.
            // A pause or a turn therefore cannot accumulate a large learning step.
            let weight = 1. - (-dt / FILTER_RESPONSE_S).exp();
            self.extra_load_mps2 = (self.extra_load_mps2 + weight * (extra - self.extra_load_mps2)).max(0.);
            self.last_load_sample_mps2 = Some(extra);
            self.learned_samples += 1;
            self.learning_source = Some(source);
        }
    }

    /// Apply the race output ceiling, including launches without a speed estimate.
    /// A slower hold may use the remaining room for the existing load compensation.
    pub fn compensate(&mut self, base_pwm: i64, speed: Option<f64>, now: f64, active: bool) -> i64 {
        let base_pwm = base_pwm.min(FORWARD_PWM_MAX);
        let dt = self.compensation_at.replace(now).map(|last| now - last);
        let current_speed = speed.filter(|v| v.is_finite() && *v >= 0.);
        let fresh = self.previous.is_some_and(|last| (0. ..=SCAN_AGE_S).contains(&(now - last.at)));
        if !active || !hold_motor(base_pwm) || !now.is_finite() {
            self.compensation_pwm = 0;
            self.compensation_credit = 0.;
            return base_pwm;
        }
        let dt = dt.filter(|dt| 0. <= *dt && *dt <= SCAN_AGE_S).unwrap_or_else(|| {
            self.compensation_credit = 0.;
            0.
        });
        // Missing velocity cannot increase compensation. Retire the previous
        // correction at the same slew rate while this forward hold continues.
        let desired = current_speed.filter(|_| fresh)
            .map_or(0, |speed| self.required_pwm(base_pwm, speed) - base_pwm);
        self.compensation_pwm = self.compensation_pwm.min(FORWARD_PWM_MAX - base_pwm);
        if desired == self.compensation_pwm {
            self.compensation_credit = 0.;
        } else {
            self.compensation_credit = (self.compensation_credit + COMPENSATION_PWM_PER_S * dt).min(HOLD_PWM_STEP as f64);
            if self.compensation_credit + 1e-9 >= HOLD_PWM_STEP as f64 {
                self.compensation_pwm += (desired - self.compensation_pwm).signum() * HOLD_PWM_STEP;
                self.compensation_credit = 0.;
            }
        }
        let command = base_pwm + self.compensation_pwm;
        if self.compensation_pwm > 0 {
            self.last_compensated_command = Some((command, now));
        }
        command
    }

    fn required_pwm(&self, base_pwm: i64, speed: f64) -> i64 {
        let wanted_accel = reference_acceleration(base_pwm, speed) + (self.extra_load_mps2 - LOAD_DEADBAND_MPS2).max(0.);
        (base_pwm..=FORWARD_PWM_MAX).step_by(HOLD_PWM_STEP as usize)
            .min_by(|a, b| {
                let a_error = (reference_acceleration(*a, speed) - wanted_accel).abs();
                let b_error = (reference_acceleration(*b, speed) - wanted_accel).abs();
                a_error.total_cmp(&b_error).then_with(|| a.cmp(b))
            })
            .unwrap_or(base_pwm)
    }

    /// Budget the current correction, or its next possible five-PWM step. This
    /// is a read-only propulsion forecast for the existing coast calculation.
    pub fn predicted_hold_accel(&self, base_pwm: i64, speed: Option<f64>, now: f64) -> Option<f64> {
        let base_pwm = base_pwm.min(FORWARD_PWM_MAX);
        let speed = speed.filter(|v| v.is_finite() && *v >= 0.)?;
        if !hold_motor(base_pwm) || !now.is_finite() {
            return None;
        }
        let current_pwm = (base_pwm + self.compensation_pwm).min(FORWARD_PWM_MAX);
        let fresh = self.previous.is_some_and(|last| (0. ..=SCAN_AGE_S).contains(&(now - last.at)));
        let next_pwm = if fresh && self.required_pwm(base_pwm, speed) > current_pwm {
            (current_pwm + HOLD_PWM_STEP).min(FORWARD_PWM_MAX)
        } else { current_pwm };
        if next_pwm == base_pwm { return None; }
        let measured = self.latest_actual_accel
            .filter(|(at, pwm, _)| (0. ..=SCAN_AGE_S).contains(&(now - at)) && (*pwm == current_pwm || *pwm == next_pwm))
            .map_or(0., |(_, _, acceleration)| acceleration);
        Some((reference_acceleration(next_pwm, speed) - self.extra_load_mps2).max(measured).max(0.))
    }

    /// Forward-throttle forecast including the base command, not just its load
    /// compensation and this program's ceiling. The 1530 rolling hold has no
    /// fitted response; its consumers use the current IMU slope instead.
    pub fn predicted_drive_accel(&self, pwm: i64, speed: Option<f64>, now: f64) -> Option<f64> {
        let pwm = pwm.min(FORWARD_PWM_MAX);
        let speed = speed.filter(|v| v.is_finite() && *v >= 0.)?;
        if !hold_motor(pwm) || !now.is_finite() { return None; }
        let measured = self.latest_actual_accel
            .filter(|(at, motor, _)| *motor == pwm && (0. ..=SCAN_AGE_S).contains(&(now - at)))
            .map_or(0., |(_, _, acceleration)| acceleration);
        Some((reference_acceleration(pwm, speed) - self.extra_load_mps2).max(measured).max(0.))
    }

    pub fn applied_hold_accel(&self, pwm: i64, speed: Option<f64>, now: f64) -> f64 {
        let Some(speed) = speed.filter(|v| v.is_finite() && *v >= 0.) else { return 0.; };
        if !hold_motor(pwm) || !self.last_compensated_command.is_some_and(|(motor, at)|
            motor == pwm && (0. ..=ESC_COAST_DELAY_S).contains(&(now - at))) { return 0.; }
        let measured = self.latest_actual_accel
            .filter(|(at, motor, _)| *motor == pwm && (0. ..=SCAN_AGE_S).contains(&(now - at)))
            .map_or(0., |(_, _, a)| a);
        (reference_acceleration(pwm, speed) - self.extra_load_mps2).max(measured).max(0.)
    }

    pub fn diagnostics(&self) -> V {
        dict! {
            "forward_pwm_max" => FORWARD_PWM_MAX,
            "effective_extra_load_mps2" => self.extra_load_mps2,
            "last_load_sample_mps2" => self.last_load_sample_mps2,
            "learning_samples" => self.learned_samples as i64,
            "learning_source" => self.learning_source,
            "latest_actual_accel_mps2" => self.latest_actual_accel.map(|v| v.2),
            "source_seq" => self.previous.map(|p| p.seq),
            "compensation_pwm" => self.compensation_pwm,
            "load_deadband_mps2" => LOAD_DEADBAND_MPS2,
            "initial_load_mps2" => self.initial_load_mps2,
            "imu_rest_bias_mps2" => self.rest_bias_mps2,
            "basis" => "local_longitudinal_response_including_surface_battery_and_drivetrain",
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::maneuver::Maneuver;

    #[test]
    fn race_decisions_and_motor_history_use_the_applied_pwm() {
        for (stage, requested, expected) in [
            ("start_lane", 1610, 1560), ("start_lane", 1600, 1560),
            ("start_lane", 1500, 1500), ("open_transfer", 1590, 1560),
            ("open_transfer", 1555, 1555), ("light_lane", 1600, 1560),
            ("light_lane", 1565, 1560), ("light_lane", 1560, 1560),
            ("light_lane", 1550, 1550), ("light_lane", 1530, 1530),
            ("light_lane", 1500, 1500), ("light_lane", 1300, 1300),
        ] {
            let mut m = Maneuver::new(&0.0.into(), &40.0.into(), &1670.into(), &true.into());
            m.race_program = true;
            m.race_inputs = Some(Default::default());
            m.phase = "drive";
            m.entry_kind = Some("lane");
            m.route_stage = stage;
            m.drive_since = Some(0.);
            m._launch_until = Some(0.);
            m._start_motor = requested;
            m._transfer_motor = requested;
            m.lane_motor = requested;
            let actual = m.result(1.).idx("motor").i();
            assert_eq!(actual, expected, "{stage} request {requested}");
            assert_eq!(m._power_cmd_motor, (expected > NEUTRAL).then_some(expected));
        }
    }

    #[test]
    fn load_compensation_and_propulsion_forecasts_respect_the_same_ceiling() {
        let mut response = SurfaceResponse::default();
        response.set_initial_load(3.);
        let mut applied = 1550;
        for seq in 0..20 {
            let at = seq as f64 * 0.1;
            response.observe(seq, at, applied, Some(1.), None, None, None);
            applied = response.compensate(1550, Some(1.), at, true);
            assert!((1550..=1560).contains(&applied));
        }
        assert_eq!(applied, 1560);
        assert_eq!(response.last_compensated_command.map(|c| c.0), Some(applied));
        assert_eq!(response.predicted_drive_accel(1600, Some(1.), 1.9),
                   response.predicted_drive_accel(1560, Some(1.), 1.9));
        assert_eq!(response.predicted_hold_accel(1590, Some(1.), 1.9), None);
        for pwm in [1300, 1500, 1530] {
            assert_eq!(response.compensate(pwm, Some(1.), 2., true), pwm);
        }
    }
}
