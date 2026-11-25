//! One continuous brake application, with a separate instance for each stopping stage.
//! Fresh light geometry anchors the existing control tick's stopping prediction.
//! The light and finish controllers never share brake state or observations.

use crate::dict;
use crate::helpers::{NEUTRAL, SCAN_AGE_S};
use crate::py::V;

pub const LIGHT_BRAKE_MOTOR: i64 = 1100;
pub const LIGHT_BRAKE_RESPONSE_S: f64 = 0.085;
/// y79 position + calibrated IMU fit, 2026-11-11. Separate stages: the
/// continuous brake's deceleration falls with speed, b(v) = base + rate*v.
/// The measured coast drag replaces y79's floor contribution each run. These
/// coefficients describe this vehicle, not any course coordinates or lengths.
#[derive(Clone, Copy, Debug)]
pub struct BrakeModel { base: f64, rate: f64, travel_correction_s: f64 }

impl BrakeModel {
    pub fn light(axis_cos: f64, coast_decel: f64) -> Self {
        Self { base: coast_decel + (1.1642 - 0.5761712726842791) * (1. + axis_cos.max(0.)) * 0.5,
            rate: 1.4121, travel_correction_s: 0.0 }
    }
    pub fn finish(coast_decel: f64) -> Self {
        Self { base: coast_decel + (1.1642 - 0.5761712726842791), rate: 1.4121, travel_correction_s: 0.0 }
    }
    pub fn distance(self, speed: f64) -> f64 {
        let v = speed.max(0.);
        if self.rate == 0. { return v*v/(2.*self.base); }
        (v/self.rate - self.base/(self.rate*self.rate)*(self.rate*v/self.base).ln_1p()).max(0.)
    }
    pub fn stop_travel(self, speed: f64, acceleration: f64) -> f64 {
        let a = acceleration.min(0.);
        let response = if a < 0. { LIGHT_BRAKE_RESPONSE_S.min(speed.max(0.) / -a) } else { LIGHT_BRAKE_RESPONSE_S };
        let release = (speed + a*response).max(0.);
        // Command-to-stop travel residual for the speed seen by this controller.
        // Finish -0.048 s (y90 variants + y92); the continuous lamp brake uses the raw travel.
        // This corrects prediction distance, not an actuator delay or a wait.
        // It keeps the physical response and continuous-brake state unchanged.
        ((speed + release)*response*0.5 + self.distance(release)
            + speed.max(0.) * self.travel_correction_s).max(0.)
    }
    pub(crate) fn decel(self, speed: f64) -> f64 { self.base + self.rate*speed.max(0.) }
}
// Same rest criterion as maneuver_light::LIGHT_RESTART_MPS.
const STOP_SPEED_MPS: f64 = 0.15;
pub(crate) const BRAKE_REST_MPS: f64 = 0.04;
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
    fn fresh_finish_board_speed_preserves_the_brake_during_an_imu_dip() {
        let mut raw = LightBrake::default();
        raw.observe(10., Some(1.12), Some(1.36), Some(4.), 1., 0.61, BrakeModel::finish(0.61), 0.25);
        raw.state = BrakeState::Active { since: 9.6 };
        let mut measured = raw.clone();
        measured.observe_range_speed(10., Some(1.34), &[(2., 10., 1.12)], 0.61, BrakeModel::finish(0.61));
        assert!(raw.control_speed(10.06, Some(3.45)).unwrap() < 1.);
        assert_eq!(measured.command(10.06, Some(3.45)), Some(LIGHT_BRAKE_MOTOR));
        measured.observe(10.1, Some(1.), Some(1.1), Some(3.45), 1., 0.61, BrakeModel::finish(0.61), 0.25);
        assert!((measured.control_speed(10.12, Some(3.25)).unwrap() - 0.9).abs() < 1e-9);
    }

    #[test]
    fn continuous_lamp_brake_uses_physical_travel_and_finish_keeps_its_range_correction() {
        let light_model = BrakeModel::light(1., 0.6081911341424501);
        let finish_model = BrakeModel::finish(0.6081911341424501);
        for (model, gap, speed, target, corrected_command, original_command) in [
            (light_model, 1.9, 1.52, 0.55, None, None),
            (light_model, 1.65, 1.52, 0.55, Some(1300), Some(1300)),
            (finish_model, 2.45, 2.114, 0.25, None, Some(1300)),
        ] {
            let mut corrected = LightBrake::default();
            let mut original = LightBrake::default();
            corrected.observe(1., Some(gap), Some(speed), Some(0.), 1., 0.6081911341424501, model, target);
            original.observe(1., Some(gap), Some(speed), Some(0.), 1., 0.6081911341424501,
                BrakeModel { travel_correction_s: 0., ..model }, target);
            assert_eq!(corrected.command(1., Some(0.)), corrected_command);
            assert_eq!(original.command(1., Some(0.)), original_command);
        }
    }

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
        brake.observe(1., Some(1.2), Some(1.), Some(0.), 1., 0.6, BrakeModel { base: 1., rate: 0., travel_correction_s: 0. }, 0.55);
        assert_eq!(brake.command(1., Some(0.)), None);
        assert_eq!(brake.command(1.04, Some(0.)), None);
        assert_eq!(brake.command(1.06, Some(0.)), Some(1300));
        brake.output_observed(1300);
        assert_eq!(brake.command(1.1, Some(-0.04)), Some(1300));
        // A stale or missing observation cannot insert a neutral pulse.
        assert_eq!(brake.command(1.4, None), Some(1300));
        // The moving approach now stays powered until braking. Do not split
        // the single brake with a rolling neutral phase.
        brake.observe(1.42, Some(0.82), Some(0.6), Some(-0.4), 1., 0.6, BrakeModel { base: 1., rate: 0., travel_correction_s: 0. }, 0.55);
        assert_eq!(brake.command(1.42, Some(-0.4)), Some(1300));
        brake.observe(1.8, Some(0.57), Some(0.1), Some(-0.9), 1., 0.6, BrakeModel { base: 1., rate: 0., travel_correction_s: 0. }, 0.55);
        assert_eq!(brake.command(1.8, Some(-0.9)), Some(1300));
        brake.observe(1.9, Some(0.56), Some(0.), Some(-1.), 1., 0.6, BrakeModel::light(1., 0.6), 0.55);
        assert_eq!(brake.command(1.9, Some(-1.)), Some(1300));
        assert_eq!(brake.command(2.02, Some(-1.)), Some(1500));
        assert!(brake.is_used());
        brake.observe(1.9, Some(0.8), Some(1.), Some(0.), 1., 0.6, BrakeModel { base: 1., rate: 0., travel_correction_s: 0. }, 0.55);
        assert_eq!(brake.command(1.9, Some(0.)), None);
    }

    #[test]
    fn switching_to_confirmed_gantry_legs_resets_preview_gap_bias_while_keeping_imu_slope() {
        let mut brake = LightBrake::default();
        let model = BrakeModel::light(1., 0.5914163596473377);
        brake.observe(17.057, Some(2.395), Some(1.80), Some(0.), 1., 0.5914163596473377, model, 0.55);
        brake.reset_gap_fusion();
        brake.observe(17.156, Some(2.15), Some(1.80), Some(-0.05886), 1., 0.5914163596473377, model, 0.55);
        assert!((brake.control_gap(17.156, Some(-0.05886)).unwrap() - 2.15).abs() < 1e-9);
        assert_eq!(brake.command(17.156, Some(-0.05886)), Some(LIGHT_BRAKE_MOTOR));
    }

    #[test]
    fn finish_brake_continues_until_rest_even_if_coasting_would_reach_the_zone() {
        let mut brake = LightBrake::default();
        let model = BrakeModel::finish(0.60);
        // Trigger brake at 1.0s with overestimated speed (2.15 m/s at 2.38 m, like y112).
        brake.observe(1.0, Some(2.38), Some(2.15), Some(0.), 1., 0.60, model, 0.25);
        assert_eq!(brake.command(1.0, Some(0.)), Some(LIGHT_BRAKE_MOTOR));
        // Before 0.3s settle, keep continuous brake.
        brake.observe(1.2, Some(1.90), Some(1.65), Some(-0.5), 1., 0.60, model, 0.25);
        assert_eq!(brake.command(1.2, Some(-0.5)), Some(LIGHT_BRAKE_MOTOR));
        // At 1.65s (like y112 t=27.51): gap=1.312, speed=1.133 -> brake_stop~0.58 > 0.40, release_stop~0.27 >= 0.25.
        brake.observe(1.65, Some(1.312), Some(1.133), Some(-1.02), 1., 0.60, model, 0.25);
        assert_eq!(brake.command(1.65, Some(-1.02)), Some(LIGHT_BRAKE_MOTOR));
        for t in [2.7, 2.82] {
            brake.observe(t, Some(0.32), Some(0.), Some(-2.1), 1., 0.60, model, 0.25);
            let motor = brake.command(t, Some(-2.1));
            assert_eq!(motor, Some(if t < 2.8 { LIGHT_BRAKE_MOTOR } else { NEUTRAL }));
        }
        brake.resume_after_forward(Some(NEUTRAL), Some(0.5));
        assert!(brake.is_used());
        assert_eq!(brake.command(2.85, Some(-2.1)), None);
    }

    #[test]
    fn corrective_brake_needs_forward_ack_and_motion_after_the_previous_stop() {
        let mut brake = LightBrake::default();
        brake.begin_stop(1.);
        brake.output_observed(NEUTRAL);
        for (pwm, speed) in [(1500, 0.25), (1300, 0.25), (1560, 0.)] {
            brake.resume_after_forward(Some(pwm), Some(speed));
            brake.begin_stop(2.);
            assert!(brake.is_used());
        }
        brake.resume_after_forward(Some(1560), Some(0.25));
        assert!(brake.has_braked());
        brake.begin_stop(2.2);
        assert_eq!(brake.command(2.2, None), Some(LIGHT_BRAKE_MOTOR));
    }

    #[test]
    fn measured_lamp_motion_outlasts_a_zero_imu_estimate() {
        let model = BrakeModel::light(1., 0.6);
        let mut brake = LightBrake::default();
        brake.begin_stop(19.9);
        for at in [20.0, 20.1, 20.2] {
            let history = [(at-10.2, at-0.2, 0.76), (at-10., at, 0.65)];
            brake.observe_range_speed(at, Some(0.55), &history, 0.60, model);
            brake.observe(at, Some(0.65), Some(0.), Some(0.), 1., 0.60, model, 0.55);
            assert!(brake.control_speed(at, Some(0.)).unwrap() > 0.40);
            assert_eq!(brake.command(at, Some(0.)), Some(LIGHT_BRAKE_MOTOR));
        }
        for at in [20.3, 20.42] {
            brake.observe_range_speed(at, Some(0.), &[(at-10., at, 0.55)], 0.60, model);
            brake.observe(at, Some(0.55), Some(0.), Some(0.), 1., 0.60, model, 0.55);
            assert_eq!(brake.command(at, Some(0.)), Some(if at < 20.4 { LIGHT_BRAKE_MOTOR } else { NEUTRAL }));
        }
    }

    #[test]
    fn finish_brake_keeps_braking_when_releasing_would_overshoot_target() {
        let mut brake = LightBrake::default();
        let model = BrakeModel::finish(0.60);
        brake.observe(1.0, Some(2.38), Some(2.15), Some(0.), 1., 0.60, model, 0.25);
        assert_eq!(brake.command(1.0, Some(0.)), Some(LIGHT_BRAKE_MOTOR));
        // At 1.5s: gap=1.57, speed=1.41 -> brake_stop~0.46 > 0.40, but release_stop < 0.25 (would overshoot target).
        brake.observe(1.5, Some(1.57), Some(1.41), Some(-0.74), 1., 0.60, model, 0.25);
        assert_eq!(brake.command(1.5, Some(-0.74)), Some(LIGHT_BRAKE_MOTOR));
        assert!(brake.is_active());
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
    gap_reference_reset: bool,
    rest_since: Option<f64>,
    // A timestamped closing-rate measurement on the same straight as the brake.
    range_speed: Option<(f64, f64)>,
}

impl LightBrake {
    /// Drop the upstream preview gap anchor when switching to confirmed gantry legs
    /// so the physical leg plane is not blended with a stale preview offset.
    pub fn reset_gap_fusion(&mut self) {
        self.gap_reference_reset = true;
    }

    /// Move a board's measured closing rate from the middle of its scan
    /// window to the current control tick. Missing ranges retire this sample.
    pub(crate) fn observe_range_speed(&mut self, now: f64, raw: Option<f64>,
        history: &[(f64, f64, f64)], coast_decel: f64, model: BrakeModel) -> Option<f64> {
        let projected = raw.zip(history.first().zip(history.last())).and_then(|(speed, (first, last))| {
            let delivery_age = now - last.1;
            if !(0. ..=SCAN_AGE_S).contains(&delivery_age) { return None; }
            let age = delivery_age + (last.0 - first.0) * 0.5;
            let decel = if self.is_active() { model.decel(speed) } else { coast_decel };
            Some((speed - decel * age).max(0.))
        });
        self.range_speed = projected.map(|v| (now, v));
        projected
    }

    /// A forward ACK and measured motion establish a new approach after a
    /// corrective pull-up. Neutral or a delayed brake ACK cannot rearm it.
    pub fn resume_after_forward(&mut self, applied: Option<i64>, speed: Option<f64>) {
        if self.is_used() && applied.is_some_and(|pwm| pwm > NEUTRAL)
            && speed.is_some_and(|v| v >= STOP_SPEED_MPS) {
            *self = Self { started_at: self.started_at, ..Self::default() };
        }
    }

    pub fn begin_stop(&mut self, now: f64) {
        if matches!(self.state, BrakeState::Ready) {
            self.state = BrakeState::Active { since: now };
            self.started_at = Some(now);
        }
    }

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
        let reset = std::mem::take(&mut self.gap_reference_reset);
        let integral = forward_integral.filter(|v| v.is_finite());
        // 1530 was already slowing the car before y79's light brake. Carry
        // that measured negative slope through the response; don't add a
        // full powered acceleration tail after commanding 1300.
        let response_accel = self.anchor.filter(|old| 0. < now-old.at && now-old.at <= SCAN_AGE_S)
            .and_then(|old| old.integral.zip(integral).map(|(before,current)|
                ((current-before)/(now-old.at)*axis_cos.max(0.)).min(0.))).unwrap_or(0.);
        // A forward hold cannot produce a multi-g impulse. An available
        // range-rate on the same straight bounds the IMU tail before braking.
        let response_accel = if !self.is_active() && self.range_speed.is_some_and(|(at, _)| at == now) {
            response_accel.max(-coast_decel)
        } else { response_accel };
        let prediction = self.anchor.filter(|old| {
            !reset && (0. ..=SCAN_AGE_S).contains(&(now-old.at)) && old.axis_cos >= MIN_AXIS_COS
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

    pub fn control_speed(&self, now: f64, forward_integral: Option<f64>) -> Option<f64> {
        self.project(now, forward_integral).map(|p| p.speed)
    }

    fn project(&self, now: f64, forward_integral: Option<f64>) -> Option<Projection> {
        let anchor = self.anchor?;
        let age = now - anchor.at;
        if !now.is_finite() || !(0. ..=SCAN_AGE_S).contains(&age) { return None; }
        let change = anchor.integral.zip(forward_integral.filter(|v| v.is_finite()))
            .map(|(start, current)| (current - start) * anchor.axis_cos);
        let speed = (anchor.speed + change.unwrap_or(0.)).max(0.);
        // Preserve the independent closing rate across a brief IMU impulse.
        let speed = if let Some((at, range_speed)) = self.range_speed
            && at == anchor.at
        {
            speed.max(range_speed - anchor.brake.decel(range_speed) * age)
        } else { speed };
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
                let early_glide = now - since >= 0.20
                    && self.projection.is_some_and(|p| p.speed <= 0.16 && p.release_stop >= 0.09);
                let still = self.projection.is_some_and(|p| p.speed <= BRAKE_REST_MPS);
                if !still { self.rest_since = None; }
                if early_glide || (still && now - since >= LIGHT_BRAKE_RESPONSE_S
                    && now - *self.rest_since.get_or_insert(now) >= LIGHT_BRAKE_RESPONSE_S) {
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
                    self.begin_stop(now);
                    Some(LIGHT_BRAKE_MOTOR)
                } else {
                    None
                }
            }
        }
    }

    pub fn has_braked(&self) -> bool { self.started_at.is_some() }

    pub fn is_active(&self) -> bool {
        matches!(self.state, BrakeState::Active { .. })
    }

    pub fn is_used(&self) -> bool {
        matches!(self.state, BrakeState::Used)
    }

    /// Release an active brake immediately once independent measurements confirm
    /// the vehicle has come to physical rest, avoiding ~0.2-0.4 s of extra dead hold.
    pub fn release_at_rest(&mut self, now: f64) -> bool {
        if let BrakeState::Active { since } = self.state
            && now - since >= 0.20
        {
            self.state = BrakeState::Used;
            self.ended_at = Some(now);
            self.end_reason = Some("stopped");
            true
        } else {
            false
        }
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
            "brake_travel_correction_s" => self.anchor.map(|a| a.brake.travel_correction_s),
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
