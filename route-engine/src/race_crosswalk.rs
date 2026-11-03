//! Drive from the crosswalk release to the first cone; no cone-transfer or light state.
use crate::helpers::{FRONT_BODY_EXTENT_M, NEUTRAL, TRIAL_MOTOR};
use crate::maneuver::*;
use crate::py::*;
use crate::dict;

// y38 (user: the crosswalk start felt slow): y27 reached the first support at ~1.2-1.3 m/s
// after a 1560/1550 cruise; y31-y37 boosted 1600 for .4 s, then coasted 1.2 s to the
// y23 straight's 1.1 before the 1550 hold.
const FIRST_ENTRY_MPS: f64 = 1.3;

#[derive(Clone, Debug, Default)]
pub struct CrosswalkExit {
    crosswalk_release: Option<(f64, Option<f64>)>,
    first_approach: FirstApproach,
}

#[derive(Default, Clone, Copy, Debug, PartialEq, Eq)]
enum FirstApproach {
    #[default]
    Cruise,
    Coast,
    Hold,
}

impl CrosswalkExit {
    pub fn release_crosswalk(&mut self, now: f64, forward_integral: Option<f64>) {
        self.crosswalk_release = Some((now, forward_integral));
    }

    pub fn crosswalk_released(&self) -> bool { self.crosswalk_release.is_some() }

    /// After the crosswalk the car is already rolling when the first support
    /// is acquired. Aim one coast at the planned bend, then hold cruise.
    pub fn first_approach_drive(&mut self, radar: Option<f64>, before: Option<f64>, decel: f64, now: f64, forward_integral: Option<f64>) -> (i64, V) {
        let inertial = self.crosswalk_release.and_then(|(_, zero)| zero)
            .zip(forward_integral).map(|(zero, integral)| (integral-zero).max(0.));
        let speed = inertial.or(radar);
        let boosted = self.crosswalk_released() && self.first_approach == FirstApproach::Cruise;
        let lag = CROSSWALK_AIM_LAG_S;
        // Tile measurements near the 1620 cut showed about 4 m/s². The IMU
        // already includes the acceleration since the last lidar frame.
        let pending_rise = self.crosswalk_release.filter(|_| boosted)
            .map_or(0., |(since, _)| 4.0 * (now - since).clamp(0., lag));
        let coast_m = speed.map(|v| {
            let released = v + pending_rise;
            (v + released)*lag/2. + (released*released - FIRST_ENTRY_MPS*FIRST_ENTRY_MPS).max(0.)/(2.*decel)
        });
        let board_stop_m = speed.map(|v| {
            let released = v + pending_rise;
            FRONT_BODY_EXTENT_M + FIRST_APPROACH_BOARD_GAP_M + (v+released)*lag/2. + released*released/(2.*decel)
        });
        if self.first_approach == FirstApproach::Cruise
            && (before.zip(coast_m).is_some_and(|(d,c)| d <= c) || (boosted && (before.is_none() || speed.is_none()))) {
            self.first_approach = FirstApproach::Coast;
        }
        if self.first_approach == FirstApproach::Coast && (before.is_some_and(|d| d <= 0.) || speed.is_none_or(|v| v <= FIRST_ENTRY_MPS)) {
            self.first_approach = FirstApproach::Hold;
        }
        let phase = self.first_phase();
        let motor = if self.first_approach == FirstApproach::Coast { NEUTRAL }
            else if self.first_approach == FirstApproach::Cruise && boosted { START_LAUNCH_MOTOR }
            else if speed.is_some_and(|v| v < LANE_SPEED_RESTART_MPS) { TRIAL_MOTOR }
            else { ROUTE_SLOW_MOTOR };
        (motor, dict! {"phase" => phase, "before_tangent_m" => before, "coast_to_entry_m" => coast_m,
                       "board_stop_range_m" => board_stop_m, "speed_now_mps" => speed,
                       "speed_source" => if inertial.is_some() { "imu_from_crosswalk_rest" } else { "lidar" },
                       "entry_target_mps" => FIRST_ENTRY_MPS})
    }

    fn first_phase(&self) -> &'static str {
        match self.first_approach { FirstApproach::Cruise => "cruise", FirstApproach::Coast => "coast", FirstApproach::Hold => "hold" }
    }

    pub fn first_rolling_hold(&self) -> bool { self.first_approach == FirstApproach::Hold }


    pub fn append_diagnostics(&self, out: &mut V) {
        out.set("first_approach_phase", self.first_phase());
        out.set("crosswalk_release", self.crosswalk_release.map(|(t, v)| V::list(vec![t.into(), v.into()])));
    }
}
