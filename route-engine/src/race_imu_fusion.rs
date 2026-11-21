//! Check a calibrated IMU increment against recent independent board motion.
//! Keep a continuous forward integral when a sensor impulse is replaced. This
//! only changes estimated motion; it never accepts, rejects or stops a run.
use crate::motion_sensors::RaceInputs;

#[derive(Clone, Copy, Debug)]
pub struct RadarMotion {
    pub at: f64,
    pub yaw: f64,
    pub speed: f64,
    pub accel: f64,
    pub forward_observed: bool,
    integral: Option<f64>,
}
#[derive(Clone, Debug, Default)]
pub struct Filter {
    pub radar: Option<RadarMotion>,
    forward_radar: Option<RadarMotion>,
    tick: Option<f64>,
    raw_integral: Option<f64>,
    offset: f64,
    last: RaceInputs,
    pub yaw_integral: f64,
    pub integrated_time: f64,
    pub last_gyro_correction: Option<f64>,
    pub yaw_available: bool,
    pub gyro_replaced: bool,
    pub accel_replaced: bool,
    suspect_dv: bool,
}
impl Filter {
    pub fn sample(&mut self, raw: RaceInputs, now: f64) -> RaceInputs {
        if self.tick == Some(now) {
            return self.last;
        }
        let dt = self.tick.map_or(0., |t| now - t);
        let recent = self
            .radar
            .filter(|r| (0.0..=crate::helpers::SCAN_AGE_S).contains(&(now - r.at)));
        let mut yaw = raw.yaw_rate.filter(|v| v.is_finite());
        let previous_rejection = self.gyro_replaced;
        self.gyro_replaced = false;
        self.accel_replaced = false;
        if let (Some(w), Some(r)) = (yaw, recent) {
            // y113-7 / y114-21 change by at most 1.03 rad/s in a revolution.
            // Compare both the radar slope and the last accepted sample so a
            // sustained, measured fast turn remains a fast turn.
            if (w - r.yaw).abs() > 1.30
                && (previous_rejection || self.last.yaw_rate.is_some_and(|v| (w - v).abs() > 0.85))
            {
                yaw = Some(r.yaw);
                self.gyro_replaced = true;
                self.last_gyro_correction = Some(now);
            }
        }
        self.yaw_available = yaw.is_some();
        if (0.0..=0.15).contains(&dt) {
            if let Some((a, b)) = self.last.yaw_rate.zip(yaw) {
                self.yaw_integral += (a + b) * 0.5 * dt;
                self.integrated_time += dt;
            }
            let forward = self.forward_radar.filter(|r| (0.0..=1.25).contains(&(now - r.at)));
            if dt > 0.
                && let (Some(prev), Some(next), Some(r)) = (self.raw_integral, raw.forward_integral, forward)
            {
                let delta = next - prev;
                let accel = delta / dt;
                if accel.abs() > 4.8 && (accel - r.accel).abs() > 4.8 {
                    self.offset += r.accel * dt - delta;
                    self.accel_replaced = true;
                    self.suspect_dv = true;
                }
            }
        }
        self.tick = Some(now);
        self.raw_integral = raw.forward_integral;
        self.last = RaceInputs {
            yaw_rate: yaw,
            forward_integral: raw.forward_integral.map(|v| v + self.offset),
        };
        self.last
    }

    pub fn anchor(&mut self, at: f64, yaw: f64, speed: f64, forward: bool) {
        let mut accel = 0.;
        if let Some(old) = self.forward_radar {
            let dt = at - old.at;
            if (0.045..=0.3).contains(&dt) && forward && old.forward_observed {
                accel = 0.50 * old.accel + 0.5 * (speed - old.speed) / dt;
                if let Some((previous, now)) = old.integral.zip(self.last.forward_integral) {
                    let error = (speed - old.speed) - (now - previous);
                    if self.suspect_dv && error.abs() > 0.12 {
                        self.offset += error;
                        self.last.forward_integral = Some(now + error);
                    }
                }
            }
        }
        self.radar = Some(RadarMotion {
            at,
            yaw,
            speed,
            accel,
            forward_observed: forward,
            integral: self.last.forward_integral,
        });
        if forward {
            self.forward_radar = self.radar;
        }
        self.suspect_dv = false;
    }
    pub fn current(&self) -> RaceInputs {
        self.last
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn a_radar_contradicted_spike_does_not_leave_a_velocity_step_or_hide_a_measured_turn() {
        let mut filter = Filter::default();
        let sample = |w, v| RaceInputs {
            yaw_rate: Some(w),
            forward_integral: Some(v),
        };
        filter.sample(sample(0., 0.), 0.);
        filter.anchor(0., 0., 0., true);
        let shock = filter.sample(sample(8., 0.20), 0.02);
        assert_eq!(shock.yaw_rate, Some(0.));
        assert!(shock.forward_integral.unwrap().abs() < 1e-9);
        let after = filter.sample(sample(0., 0.20), 0.04);
        assert!(after.forward_integral.unwrap().abs() < 1e-9);
        filter.anchor(0.10, 2.1, 0., true);
        assert_eq!(filter.sample(sample(2.05, 0.20), 0.12).yaw_rate, Some(2.05));
    }
}
