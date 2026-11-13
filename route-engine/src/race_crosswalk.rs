//! Drive from the crosswalk release to the first cone; no cone-transfer or light state.
use crate::helpers::{FRONT_BODY_EXTENT_M, NEUTRAL, SCAN_AGE_S, TRIAL_MOTOR, dcos, dsin};
use crate::maneuver::*;
use crate::py::*;
use crate::dict;

// y38 (user: the crosswalk start felt slow): y27 reached the first support at ~1.2-1.3 m/s
// after a 1560/1550 cruise; y31-y37 boosted 1600 for .4 s, then coasted 1.2 s to the
// y23 straight's 1.1 before the 1550 hold. 11-04: cone one took 1.30-1.51 m/s with .35-.49 m to
// the outer board; at 1.87-2.11 (the 1570 hold, before the y50 plan rules) it ran wide, .08-.13 m.
// With the y50/y53 loop rules 1.65-1.80 kept >= .28 m; y53 coasted from 1.93 to 1.54 and then
// held 1.63 at1550. Use1.75 for the geometric deceleration budget, then latch1550.
pub(crate) const FIRST_ENTRY_MPS: f64 = 1.75;

// A side opening can put the two real returns bounding a missing run at quite
// different ranges. Compare both to this launch's stopping distance along the
// car axis. A known previous return inside the driving corridor rules it out.
pub(crate) fn flank_beyond_stop(clearance: &V, old: &[V], new: &[V], missing: &[V], horizon: f64) -> bool {
    let fits = clearance.get("front_wall_projection").get("fits");
    fits.get("left").is_dict() && fits.get("right").is_dict() && far_flank_gap(clearance, old, new, missing, horizon)
}

/// Every missing run is bounded by current returns beyond the stopping range along the
/// axis, the central corridor is clear past it, and the previous scan saw nothing inside
/// the corridor within it. y50 seq 91-92: the open area past the left-turn opening (flanks
/// 3.3-5.6 m ahead, 1.2-2.3 m left) stayed dark for two scans and paused the launch .4 s.
pub(crate) fn far_flank_gap(clearance: &V, old: &[V], new: &[V], missing: &[V], horizon: f64) -> bool {
    let Some(half_width) = clearance.get("corridor_half_width_m").of() else { return false };
    let corridor_clear = clearance.get("corridor_front_m").of().is_some_and(|d| d > horizon);
    let beside = |i: usize, r: f64| (r * dsin(i)).abs() > half_width;
    // A flank beside the corridor (y51 seq 87: the left wall's end .80 m ahead, .43 m left)
    // clears the corridor along its own ray as well as one beyond the stopping range.
    missing.chunk_by(|a, b| b.i() == a.i() + 1).all(|run| {
        let left = run[0].i() - 1;
        let right = run[run.len() - 1].i() + 1;
        let flanks = [left, right].map(|angle| {
            let i = imod(angle, 360) as usize;
            (i, new[i].of())
        });
        // y70 seq 90: cone one stood in the corridor 3.57 m ahead, inside the 4.6 m horizon, and
        // paused the launch .5 s. The corridor's front says nothing about a dark run off to the
        // side: with both flanks beside the corridor only the previous scan's returns matter.
        (corridor_clear || flanks.iter().all(|&(i, r)| r.is_some_and(|r| beside(i, r))))
            && flanks.iter().all(|&(i, r)| r.is_some_and(|r| r * dcos(i) > horizon || beside(i, r)))
            && (left..=right).all(|angle| {
                let i = imod(angle, 360) as usize;
                old[i].of().is_none_or(|r| r * dcos(i) > horizon || beside(i, r))
            })
    })
}

#[derive(Clone, Debug, Default)]
pub struct CrosswalkExit {
    crosswalk_release: Option<(f64, Option<f64>)>,
    first_approach: FirstApproach,
    last_drive_at: Option<f64>,
    last_launch_at: Option<f64>,
    first_motor: Option<i64>,
}

#[derive(Default, Clone, Copy, Debug, PartialEq, Eq)]
enum FirstApproach {
    #[default]
    Launch,
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
    #[allow(clippy::too_many_arguments)]
    pub fn first_approach_drive(&mut self, radar: Option<f64>, body_speed: Option<f64>, before: Option<f64>, rolling_tangent: bool, decel: f64, now: f64, forward_integral: Option<f64>, hold_accel: f64, entry_mps: f64) -> (i64, V) {
        // y70: a quality pause (seq 90-95) left the phase in Cruise, so the 1600 launch came
        // back at 1.49 m/s and cone one was entered at 1.90 (y67, y68: 1.73). A pause ends the boost.
        if self.last_drive_at.replace(now).is_some_and(|last| now - last > SCAN_AGE_S)
            && matches!(self.first_approach, FirstApproach::Launch | FirstApproach::Cruise)
        {
            self.first_approach = FirstApproach::Hold;
        }
        let inertial = self.crosswalk_release.and_then(|(_, zero)| zero)
            .zip(forward_integral).map(|(zero, integral)| (integral-zero).max(0.));
        // y90: the rest-long integral still said1.92 while the recent
        // radar/IMU anchor measured1.71, extending neutral past entry speed.
        let speed = body_speed.or(inertial).or(radar);
        let lag = CROSSWALK_AIM_LAG_S;
        // End the start pulse when its measured speed plus the existing ESC
        // tail reaches entry speed. The remaining straight uses 1570.
        if self.first_approach == FirstApproach::Launch
            && (!self.crosswalk_released() || speed.is_some_and(|v| v + 4. * lag >= entry_mps)) {
            self.first_approach = FirstApproach::Cruise;
        }
        let boosted = self.crosswalk_released() && self.first_approach == FirstApproach::Launch;
        let launch_tail = self.last_launch_at.map_or(0., |at| (lag - (now-at)).clamp(0., lag));
        let pending_rise = self.crosswalk_release.filter(|_| boosted)
            .map_or(0., |(since, _)| 4.0 * (now - since).clamp(0., lag))
            + if self.first_approach == FirstApproach::Cruise {
                4. * launch_tail + hold_accel * (lag-launch_tail)
            } else { 0. };
        let coast_m = speed.map(|v| {
            let released = v + pending_rise;
            (v + released)*lag/2. + (released*released - entry_mps*entry_mps).max(0.)/(2.*decel)
        });
        let board_stop_m = speed.map(|v| {
            let released = v + pending_rise;
            FRONT_BODY_EXTENT_M + FIRST_APPROACH_BOARD_GAP_M + (v+released)*lag/2. + released*released/(2.*decel)
        });
        if matches!(self.first_approach, FirstApproach::Launch | FirstApproach::Cruise)
            && (before.zip(coast_m).is_some_and(|(d,c)| d <= c) || (boosted && (before.is_none() || speed.is_none()))) {
            self.first_approach = FirstApproach::Coast;
        }
        // Once the 1600 tail has expired and the actual first tangent is visible,
        // switch to the single rolling hold if its tail fits before the bend.
        let hold_room = self.first_approach == FirstApproach::Coast && rolling_tangent && before.zip(speed).is_some_and(|(d, v)| {
            v <= entry_mps && {
            let release = v + hold_accel * lag;
            d > (v + release) * lag / 2. + (release * release - entry_mps * entry_mps).max(0.) / (2. * decel)
            }
        });
        if self.first_approach == FirstApproach::Coast && (hold_room || before.is_some_and(|d| d <= 0.) || speed.is_none_or(|v| v <= entry_mps)) {
            self.first_approach = FirstApproach::Hold;
        }
        let phase = self.first_phase();
        let motor = if self.first_approach == FirstApproach::Coast { NEUTRAL }
            else if boosted && self.first_approach == FirstApproach::Launch { START_LAUNCH_MOTOR }
            else if speed.is_some_and(|v| v < LANE_SPEED_RESTART_MPS) { TRIAL_MOTOR }
            else if self.first_approach == FirstApproach::Cruise { RACE_CRUISE_MOTOR }
            else { ROUTE_SLOW_MOTOR };
        if motor == START_LAUNCH_MOTOR { self.last_launch_at = Some(now); }
        self.first_motor = Some(motor);
        (motor, dict! {"phase" => phase, "before_tangent_m" => before, "coast_to_entry_m" => coast_m,
                       "board_stop_range_m" => board_stop_m, "speed_now_mps" => speed,
                       "speed_source" => if body_speed.is_some() { "recent_radar_anchor_with_short_imu" }
                           else if inertial.is_some() { "imu_from_crosswalk_rest" } else { "lidar" },
                       "entry_target_mps" => entry_mps, "early_rolling_hold" => hold_room})
    }

    fn first_phase(&self) -> &'static str {
        match self.first_approach { FirstApproach::Launch => "launch", FirstApproach::Cruise => "cruise", FirstApproach::Coast => "coast", FirstApproach::Hold => "hold" }
    }

    pub fn first_rolling_hold(&self) -> bool { self.first_approach == FirstApproach::Hold }

    pub fn first_motor(&self) -> Option<i64> { self.first_motor }

    /// The board search can cut before a tangent is acquired. That is the
    /// same approach coast, not a separate pulse that may re-enable cruise.
    pub fn board_cut(&mut self) -> &'static str {
        if matches!(self.first_approach, FirstApproach::Launch | FirstApproach::Cruise) {
            self.first_approach = FirstApproach::Coast;
        }
        self.first_motor = Some(NEUTRAL);
        self.first_phase()
    }

    pub fn enter_first_bend(&mut self) {
        self.first_approach = FirstApproach::Hold;
        self.first_motor = Some(ROUTE_SLOW_MOTOR);
    }

    /// The IMU's forward speed since the crosswalk release (the car stood there).
    pub fn imu_speed(&self, forward_integral: Option<f64>) -> Option<f64> {
        self.crosswalk_release.and_then(|(_, zero)| zero).zip(forward_integral).map(|(zero, i)| (i - zero).max(0.))
    }


    pub fn append_diagnostics(&self, out: &mut V) {
        out.set("first_approach_phase", self.first_phase());
        out.set("crosswalk_release", self.crosswalk_release.map(|(t, v)| V::list(vec![t.into(), v.into()])));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_cruise_ends_once_even_if_the_tangent_moves_back_or_speed_drops() {
        let mut exit = CrosswalkExit::default();
        exit.release_crosswalk(0., Some(0.));
        let mut drive = |at, speed, before| exit.first_approach_drive(None, None, Some(before), true,
            0.6, at, Some(speed), 0.8, FIRST_ENTRY_MPS).0;
        assert_eq!(drive(0., 0., 4.), START_LAUNCH_MOTOR);
        assert_eq!(drive(0.2, 1.1, 4.), RACE_CRUISE_MOTOR);
        assert_eq!(drive(0.3, 2., 0.4), NEUTRAL);
        assert_eq!(drive(0.4, 1.6, 0.2), ROUTE_SLOW_MOTOR);
        assert_eq!(drive(0.5, 1.4, 5.), ROUTE_SLOW_MOTOR);
        assert_eq!(drive(0.6, 1.7, 3.), ROUTE_SLOW_MOTOR);

        let mut exit = CrosswalkExit::default();
        exit.release_crosswalk(0., Some(0.));
        exit.first_approach_drive(None, Some(2.), Some(4.), false, 0.6, 1., Some(2.1), 0.4, FIRST_ENTRY_MPS);
        exit.board_cut();
        assert_eq!(exit.first_motor(), Some(NEUTRAL));
        assert_eq!(exit.first_approach_drive(None, Some(1.70), Some(2.), true, 0.6, 1.1,
            Some(1.92), 0.4, FIRST_ENTRY_MPS).0, ROUTE_SLOW_MOTOR);
        assert_eq!(exit.first_approach_drive(None, Some(1.6), Some(4.), true, 0.6, 1.2,
            Some(1.8), 0.4, FIRST_ENTRY_MPS).0, ROUTE_SLOW_MOTOR);
    }

    #[test]
    fn a_far_side_opening_does_not_hide_a_previous_return_in_the_drive_corridor() {
        let clearance = dict! {
            "corridor_front_m" => 5.85, "corridor_half_width_m" => 0.32,
            "front_wall_projection" => dict! {"fits" => dict! {"left" => V::dict(), "right" => V::dict()}},
        };
        let missing: Vec<V> = (-22..=-15).map(V::Int).collect();
        let mut current = vec![V::None; 360];
        current[337] = 3.849.into();
        current[346] = 7.006.into();
        let mut previous = current.clone();
        previous[340] = 1.40.into(); // side wall at 0.48 m lateral
        assert!(flank_beyond_stop(&clearance, &previous, &current, &missing, 3.49));
        previous[340] = 0.50.into(); // now only 0.17 m lateral
        assert!(!flank_beyond_stop(&clearance, &previous, &current, &missing, 3.49));
    }
}
