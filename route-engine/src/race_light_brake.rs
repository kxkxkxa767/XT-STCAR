//! One continuous brake application for the traffic-light approach.
//! The light controller owns the stopping-distance prediction and permission
//! to start; this state is independent of the finish controller's brake.

use crate::helpers::NEUTRAL;

pub const LIGHT_BRAKE_MOTOR: i64 = 1300;
pub const LIGHT_BRAKE_RESPONSE_S: f64 = 0.1;
pub const LIGHT_BRAKE_DECEL_MPS2: f64 = 0.95;
// Same rest criterion as maneuver_light::LIGHT_RESTART_MPS.
const STOP_SPEED_MPS: f64 = 0.15;

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
    fn a_light_brake_remains_continuous_and_cannot_be_applied_twice() {
        let mut brake = LightBrake::default();
        assert_eq!(brake.command(1., Some(1.), Some(0.2), Some(0.8), 0.55), None);
        assert_eq!(brake.command(1.1, Some(1.), Some(0.2), Some(0.5), 0.55), Some(1300));
        brake.output_observed(1300);
        assert_eq!(brake.command(1.15, Some(0.1), None, None, 0.55), Some(1300));
        assert_eq!(brake.command(1.3, None, None, None, 0.55), Some(1300));
        assert_eq!(brake.command(1.4, Some(0.1), None, None, 0.55), Some(1500));
        assert_eq!(brake.command(1.5, Some(1.), Some(0.1), Some(0.1), 0.55), None);
    }
}

#[derive(Clone, Debug, Default)]
pub struct LightBrake {
    state: BrakeState,
}

impl LightBrake {
    /// Distances are predicted remaining gaps to the gantry, in metres.
    /// `next_brake_stop` includes waiting for the next scan before braking,
    /// then LIGHT_BRAKE_RESPONSE_S and LIGHT_BRAKE_DECEL_MPS2. This permits a
    /// brake only when coasting would overrun the target and another scan of
    /// delay would use up the remaining braking distance.
    ///
    /// Call on each updated light-approach decision even if range is missing.
    /// Between scans the caller preserves the selected 1300 command.
    pub fn command(
        &mut self,
        now: f64,
        speed: Option<f64>,
        coast_stop: Option<f64>,
        next_brake_stop: Option<f64>,
        target: f64,
    ) -> Option<i64> {
        let speed = speed.filter(|v| v.is_finite());
        match self.state {
            BrakeState::Used => None,
            BrakeState::Active { since } => {
                if now - since >= LIGHT_BRAKE_RESPONSE_S
                    && speed.is_some_and(|v| v < STOP_SPEED_MPS)
                {
                    self.state = BrakeState::Used;
                    Some(NEUTRAL)
                } else {
                    // Missing speed does not insert a neutral pulse between
                    // brake commands: a second brake application could reverse
                    // an ESC that has already completed its braking sequence.
                    Some(LIGHT_BRAKE_MOTOR)
                }
            }
            BrakeState::Ready => {
                let coast_stop = coast_stop.filter(|gap| gap.is_finite())?;
                let next_brake_stop = next_brake_stop.filter(|gap| gap.is_finite())?;
                if now.is_finite()
                    && target.is_finite()
                    && speed.is_some_and(|v| v >= STOP_SPEED_MPS)
                    && coast_stop < target
                    && next_brake_stop <= target
                {
                    self.state = BrakeState::Active { since: now };
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

    /// A quality pause or another controller taking over permanently retires
    /// this brake application. The caller owns the replacement motor command.
    pub fn cancel_active(&mut self) {
        if self.is_active() {
            self.state = BrakeState::Used;
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
