//! One continuous brake application, with a separate instance for each stopping stage.
//! Fresh light geometry anchors the existing control tick's stopping prediction.
//! The light and finish controllers never share brake state or observations.

use crate::dict;
use crate::helpers::{NEUTRAL, SCAN_AGE_S};
use crate::py::V;

pub const LIGHT_BRAKE_MOTOR: i64 = 1300;
pub const LIGHT_BRAKE_RESPONSE_S: f64 = 0.1;
/// y79 position + calibrated IMU fit, 2026-11-11. Separate stages: the
/// continuous brake's deceleration falls with speed, b(v) = base + rate*v.
/// The measured coast drag replaces y79's floor contribution each run. These
/// coefficients describe this vehicle, not any course coordinates or lengths.
#[derive(Clone, Copy, Debug)]
pub struct BrakeModel { base: f64, rate: f64 }

impl BrakeModel {
    pub fn light(axis_cos: f64, coast_decel: f64) -> Self {
        Self { base: coast_decel + (0.668863 - 0.5761712726842791) * (1. + axis_cos.max(0.)) * 0.5, rate: 0.315219 }
    }
    pub fn finish(coast_decel: f64) -> Self { Self { base: coast_decel + (0.583426 - 0.5761712726842791), rate: 0.378253 } }
    pub fn distance(self, speed: f64) -> f64 {
        let v = speed.max(0.);
        if self.rate == 0. { return v*v/(2.*self.base); }
        (v/self.rate - self.base/(self.rate*self.rate)*(self.rate*v/self.base).ln_1p()).max(0.)
    }
    pub fn stop_travel(self, speed: f64, acceleration: f64) -> f64 {
        let a = acceleration.min(0.);
        let response = if a < 0. { LIGHT_BRAKE_RESPONSE_S.min(speed.max(0.) / -a) } else { LIGHT_BRAKE_RESPONSE_S };
        let release = (speed + a*response).max(0.);
        (speed + release)*response*0.5 + self.distance(release)
    }
    fn decel(self, speed: f64) -> f64 { self.base + self.rate*speed.max(0.) }
}
// Same rest criterion as maneuver_light::LIGHT_RESTART_MPS.
const STOP_SPEED_MPS: f64 = 0.15;
// Observation-fusion response, not a calibrated braking parameter. Verify
// against fixed recorded inputs; stopping targets and brake rates stay separate.
const GAP_FUSION_RESPONSE_S: f64 = 0.2;
const MIN_AXIS_COS: f64 = std::f64::consts::FRAC_1_SQRT_2;

#[derive(Clone, Copy, Debug, Default)]
enum BrakeState {
    #[default]
    Ready,
    Active {
        since: f64,
    },
    Used,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn brake_distance_integrates_speed_dependent_deceleration_and_floor_drag() {
        for model in [BrakeModel::light(1., 0.5761712726842791), BrakeModel::finish(0.5761712726842791)] {
            for speed in [0.1, 1.0, 2.10804] {
                let step = speed / 10000.;
                let numeric: f64 = (0..10000).map(|i| {
                    let v = (i as f64 + 0.5)*step;
                    v/model.decel(v)*step
                }).sum();
                assert!((model.distance(speed)-numeric).abs() < 1e-7);
                assert!(model.stop_travel(speed, -0.45) < model.stop_travel(speed, 0.));
            }
        }
        assert!(BrakeModel::finish(0.9).distance(2.) < BrakeModel::finish(0.5).distance(2.));
    }

    #[test]
    fn a_light_brake_remains_continuous_and_cannot_be_applied_twice() {
        let mut brake = LightBrake::default();
        // No new scan is needed to reach the brake point between 10 Hz frames.
        brake.observe(1., Some(1.2), Some(1.), Some(0.), 1., 0.6, BrakeModel { base: 1., rate: 0. }, 0.55);
        assert_eq!(brake.command(1., Some(0.)), None);
        assert_eq!(brake.command(1.04, Some(0.)), None);
        assert_eq!(brake.command(1.06, Some(0.)), Some(1300));
        brake.output_observed(1300);
        assert_eq!(brake.command(1.1, Some(-0.04)), Some(1300));
        // A stale or missing observation cannot insert a neutral pulse.
        assert_eq!(brake.command(1.4, None), Some(1300));
        // The moving approach now stays powered until braking. Do not split
        // the single brake with a rolling neutral phase.
        brake.observe(1.42, Some(0.82), Some(0.6), Some(-0.4), 1., 0.6, BrakeModel { base: 1., rate: 0. }, 0.55);
        assert_eq!(brake.command(1.42, Some(-0.4)), Some(1300));
        brake.observe(1.8, Some(0.57), Some(0.1), Some(-0.9), 1., 0.6, BrakeModel { base: 1., rate: 0. }, 0.55);
        assert_eq!(brake.command(1.8, Some(-0.9)), Some(1500));
        assert!(brake.is_used());
        brake.observe(1.9, Some(0.8), Some(1.), Some(0.), 1., 0.6, BrakeModel { base: 1., rate: 0. }, 0.55);
        assert_eq!(brake.command(1.9, Some(0.)), None);
    }
}

#[derive(Clone, Copy, Debug)]
struct Anchor {
    at: f64,
    gap: f64,
    speed: f64,
    integral: Option<f64>,
    axis_cos: f64,
    coast_decel: f64,
    brake: BrakeModel,
    response_accel: f64,
    target: f64,
}

#[derive(Clone, Copy, Debug)]
struct Projection {
    at: f64,
    anchor_at: f64,
    age: f64,
    gap: f64,
    speed: f64,
    coast_stop: f64,
    brake_stop: f64,
    release_stop: f64,
    imu_delta: bool,
}

#[derive(Clone, Copy, Debug)]
struct GapFusion {
    at: f64,
    measured: f64,
    predicted: Option<f64>,
    weight: f64,
    control: f64,
}

#[derive(Clone, Debug, Default)]
pub struct LightBrake {
    state: BrakeState,
    anchor: Option<Anchor>,
    projection: Option<Projection>,
    started_at: Option<f64>,
    ended_at: Option<f64>,
    end_reason: Option<&'static str>,
    gap_fusion: Option<GapFusion>,
}

impl LightBrake {
    /// Observe once per new scan. All distances, speeds and decelerations are
    /// along the light-lane axis. An unavailable observation does not renew the
    /// preceding anchor's age; missing geometry is never replaced by a timer.
    #[allow(clippy::too_many_arguments)]
    pub fn observe(
        &mut self,
        now: f64,
        gap: Option<f64>,
        speed: Option<f64>,
        forward_integral: Option<f64>,
        axis_cos: f64,
        coast_decel: f64,
        brake: BrakeModel,
        target: f64,
    ) {
        let (Some(gap), Some(speed)) = (gap.filter(|v| v.is_finite()), speed.filter(|v| v.is_finite() && *v >= 0.)) else {
            return;
        };
        if !now.is_finite() || !axis_cos.is_finite() || !target.is_finite()
            || !coast_decel.is_finite() || coast_decel <= 0.
            || !brake.base.is_finite() || brake.base <= 0. || !brake.rate.is_finite() || brake.rate < 0.
            || self.anchor.is_some_and(|old| now <= old.at)
        {
            return;
        }
        let integral = forward_integral.filter(|v| v.is_finite());
        // 1530 was already slowing the car before y79's light brake. Carry
        // that measured negative slope through the response; don't add a
        // full powered acceleration tail after commanding 1300.
        let response_accel = self.anchor.filter(|old| 0. < now-old.at && now-old.at <= SCAN_AGE_S)
            .and_then(|old| old.integral.zip(integral).map(|(before,current)|
                ((current-before)/(now-old.at)*axis_cos.max(0.)).min(0.))).unwrap_or(0.);
        let prediction = self.anchor.filter(|old| {
            (0. ..=SCAN_AGE_S).contains(&(now-old.at)) && old.axis_cos >= MIN_AXIS_COS
        }).and_then(|old| {
            let (origin, current) = old.integral.zip(integral)?;
            let dt = now-old.at;
            // Transport the old body speed into the current lane projection
            // before integrating displacement. A near-transverse old axis
            // cannot identify body speed and therefore starts a new anchor.
            let projected_speed = ((old.speed/old.axis_cos+current-origin).max(0.)*axis_cos).max(0.);
            let predicted_gap = old.gap-(old.speed+projected_speed)*dt*0.5;
            predicted_gap.is_finite().then_some((predicted_gap, dt))
        });
        let weight = prediction.map_or(1., |(_, dt)| dt/(GAP_FUSION_RESPONSE_S+dt));
        let control_gap = prediction.map_or(gap, |(predicted, _)| predicted+weight*(gap-predicted));
        self.gap_fusion = Some(GapFusion {
            at: now, measured: gap, predicted: prediction.map(|p| p.0), weight, control: control_gap,
        });
        self.anchor = Some(Anchor {
            at: now, gap: control_gap, speed, integral,
            axis_cos, coast_decel, brake, response_accel, target,
        });
    }

    /// Read the same short projection used by the brake, for the approach's
    /// coast budget. The caller keeps measured front/rear gaps for zone checks.
    pub fn control_gap(&self, now: f64, forward_integral: Option<f64>) -> Option<f64> {
        self.project(now, forward_integral).map(|p| p.gap)
    }

    fn project(&self, now: f64, forward_integral: Option<f64>) -> Option<Projection> {
        let anchor = self.anchor?;
        let age = now - anchor.at;
        if !now.is_finite() || !(0. ..=SCAN_AGE_S).contains(&age) { return None; }
        let change = anchor.integral.zip(forward_integral.filter(|v| v.is_finite()))
            .map(|(start, current)| (current - start) * anchor.axis_cos);
        let speed = (anchor.speed + change.unwrap_or(0.)).max(0.);
        let gap = anchor.gap - (anchor.speed + speed) * age * 0.5;
        let coast_stop = gap - speed * speed / (2. * anchor.coast_decel);
        let brake_stop = gap - anchor.brake.stop_travel(speed, anchor.response_accel);
        // After commanding neutral, the current braking response lasts briefly
        // before normal coasting takes over. Never integrate speed below zero.
        let brake_decel = anchor.brake.decel(speed);
        let response = LIGHT_BRAKE_RESPONSE_S.min(speed / brake_decel);
        let release_speed = (speed - brake_decel * response).max(0.);
        let release_stop = gap - (speed + release_speed) * response * 0.5
            - release_speed * release_speed / (2. * anchor.coast_decel);
        if ![speed, gap, coast_stop, brake_stop, release_stop].iter().all(|v| v.is_finite()) { return None; }
        Some(Projection { at: now, anchor_at: anchor.at, age, gap, speed, coast_stop, brake_stop, release_stop, imu_delta: change.is_some() })
    }

    /// Advance on the existing control tick, without waiting another scan.
    /// An active application remains continuous through missing observations;
    /// the existing data supervisor still owns a stale-input pause or stop.
    pub fn command(&mut self, now: f64, forward_integral: Option<f64>) -> Option<i64> {
        if self.is_used() { return None; }
        self.projection = self.project(now, forward_integral);
        match self.state {
            BrakeState::Used => None,
            BrakeState::Active { since } => {
                let release = self.projection.filter(|p| now - since >= LIGHT_BRAKE_RESPONSE_S
                    && p.speed < STOP_SPEED_MPS);
                if release.is_some() {
                    self.state = BrakeState::Used;
                    self.ended_at = Some(now);
                    self.end_reason = Some("stopped");
                    Some(NEUTRAL)
                } else {
                    // Missing speed does not insert a neutral pulse between
                    // brake commands: a second brake application could reverse
                    // an ESC that has already completed its braking sequence.
                    Some(LIGHT_BRAKE_MOTOR)
                }
            }
            BrakeState::Ready => {
                let projected = self.projection?;
                let target = self.anchor.unwrap().target;
                if projected.speed >= STOP_SPEED_MPS
                    && projected.coast_stop < target
                    && projected.brake_stop <= target
                {
                    self.state = BrakeState::Active { since: now };
                    self.started_at = Some(now);
                    Some(LIGHT_BRAKE_MOTOR)
                } else {
                    None
                }
            }
        }
    }

    pub fn is_active(&self) -> bool {
        matches!(self.state, BrakeState::Active { .. })
    }

    pub fn is_used(&self) -> bool {
        matches!(self.state, BrakeState::Used)
    }

    pub fn phase(&self) -> &'static str {
        match self.state {
            BrakeState::Ready => "ready",
            BrakeState::Active { .. } => "braking",
            BrakeState::Used => "used",
        }
    }

    pub fn diagnostics(&self) -> V {
        dict! {
            "phase" => self.phase(), "started_at" => self.started_at,
            "ended_at" => self.ended_at, "end_reason" => self.end_reason,
            "anchor_at" => self.anchor.map(|a| a.at),
            "brake_base_decel_mps2" => self.anchor.map(|a| a.brake.base),
            "brake_speed_gain_per_s" => self.anchor.map(|a| a.brake.rate),
            "response_accel_mps2" => self.anchor.map(|a| a.response_accel),
            "projection_at" => self.projection.map(|p| p.at),
            "projection_anchor_at" => self.projection.map(|p| p.anchor_at),
            "projection_age_s" => self.projection.map(|p| p.age),
            "projected_gap_m" => self.projection.map(|p| p.gap),
            "projected_speed_mps" => self.projection.map(|p| p.speed),
            "coast_stop_gap_m" => self.projection.map(|p| p.coast_stop),
            "brake_stop_gap_m" => self.projection.map(|p| p.brake_stop),
            "release_stop_gap_m" => self.projection.map(|p| p.release_stop),
            "projection_basis" => self.projection.map(|p| if p.imu_delta { "short_imu_delta" } else { "constant_anchor_speed" }),
            "gap_observed_at" => self.gap_fusion.map(|g| g.at),
            "measured_gap_m" => self.gap_fusion.map(|g| g.measured),
            "predicted_gap_m" => self.gap_fusion.and_then(|g| g.predicted),
            "gap_innovation_m" => self.gap_fusion.and_then(|g| g.predicted.map(|p| g.measured-p)),
            "gap_correction_m" => self.gap_fusion.and_then(|g| g.predicted.map(|p| g.control-p)),
            "control_gap_m" => self.gap_fusion.map(|g| g.control),
            "gap_innovation_weight" => self.gap_fusion.map(|g| g.weight),
            "gap_fusion_response_s" => GAP_FUSION_RESPONSE_S,
            "gap_fusion_basis" => "candidate_range_innovation_fusion_on_imu_propagation",
        }
    }

    /// A quality pause or another controller taking over permanently retires
    /// this brake application. The caller owns the replacement motor command.
    pub fn cancel_active(&mut self) {
        if self.is_active() {
            self.state = BrakeState::Used;
            // The compatibility callback has no clock. Do not invent an end
            // timestamp for an external override; normal release is timed.
            self.end_reason = Some("command_overridden");
        }
    }

    /// Observe the final selected/emitted motor command, after overrides.
    /// Do not feed delayed bridge ACKs here: an old 1500 ACK may still arrive
    /// while the first 1300 command is on its way to the ESC.
    pub fn output_observed(&mut self, motor: i64) {
        if motor != LIGHT_BRAKE_MOTOR {
            self.cancel_active();
        }
    }
}
