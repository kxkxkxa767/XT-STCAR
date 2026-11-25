//! Drive from the crosswalk release to the first cone; no cone-transfer or light state.
use crate::helpers::{FRONT_BODY_EXTENT_M, NEUTRAL, SCAN_AGE_S, SERVO_MAX, TRIAL_MOTOR, dcos, dsin};
use crate::maneuver::*;
use crate::py::*;
use crate::dict;

// y38 (user: the crosswalk start felt slow): y27 reached the first support at ~1.2-1.3 m/s
// after a 1560/1550 cruise; y31-y37 boosted 1600 for .4 s, then coasted 1.2 s to the
// y23 straight's 1.1 before the 1550 hold. 11-04: cone one took 1.30-1.51 m/s with .35-.49 m to
// the outer board; at 1.87-2.11 (the 1570 hold, before the y50 plan rules) it ran wide, .08-.13 m.
// With the y50/y53 loop rules 1.65-1.80 kept >= .28 m; y53 coasted from 1.93 to 1.54 and then
// held 1.63 at1550. Use1.75 for the geometric deceleration budget, then latch1550.
pub(crate) const FIRST_ENTRY_MPS: f64 = 1.76;
pub(crate) const APPROACH_PWM_ADD: i64 = 0;
pub(crate) const APPROACH_CRUISE_MOTOR: i64 = 1600;

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

#[derive(Clone, Copy, Debug)]
struct CrosswalkBrakeAnchor {
    at: f64,
    distance: Option<f64>,
    speed: Option<f64>,
    integral: Option<f64>,
    decel: f64,
    entry_mps: f64,
    accel: f64,
    rolling_tangent: bool,
}

#[derive(Clone, Debug, Default)]
pub struct CrosswalkExit {
    crosswalk_release: Option<(f64, Option<f64>)>,
    first_approach: FirstApproach,
    last_drive_at: Option<f64>,
    last_launch_at: Option<f64>,
    first_motor: Option<i64>,
    brake_used: bool,
    brake_since: Option<f64>,
    brake_anchor: Option<CrosswalkBrakeAnchor>,
    pub cone_motion: crate::race_light::EntryMotionSpeed,
}

/// One existing PWM step, only with longitudinal and steering reserve. Use the
/// higher of *current* independent body-speed estimates; a long integral from
/// the zebra stop is especially unreliable in this turn. No change to tangent braking.
pub(crate) fn first_arc_drive(lp: &V, map: Option<f64>, cone: Option<f64>, steering: i64, wall_turn: bool) -> V {
    let speed = map.zip(cone).filter(|(a,b)|(a-b).abs() <= CROSSWALK_CRUISE_BAND_MPS).map(|(a,b)|a.max(b));
    let room = lp.get("support_source").os()==Some("current_lidar")
        && lp.get("plan").get("min_slack_m").of().is_some_and(|s|s>0.)
        && lp.get("plan").get("board_slack_m").of().is_some_and(|s|s>0.)
        && steering < SERVO_MAX && !wall_turn;
    let pwm = ROUTE_SLOW_MOTOR + if room && speed.is_some_and(|v|v<FIRST_ENTRY_MPS-LANE_SPEED_HOLD_MARGIN_MPS) {HOLD_PWM_STEP} else {0};
    dict!{"motor"=>pwm,"cone_motion_mps"=>cone,"localized_body_mps"=>map,"supported_body_mps"=>speed,
          "steer_reserve_pwm"=>SERVO_MAX-steering,"plan_and_steering_room"=>room}
}

impl Maneuver {
    pub(crate) fn tangent_body_speed(&self,scan:&V)->Option<f64> {
        let map=self.field_map.motion_supported(scan).then(||self.field_map.body_speed(scan)).flatten()?;
        let cone=self.race_crosswalk.cone_motion.measured_this_scan(scan.idx("seq").i())?;
        ((map-cone).abs()<=CROSSWALK_CRUISE_BAND_MPS).then_some((map+cone)*0.5)
    }
    pub(crate) fn first_arc_feedback(&self, scan: &V, lp: &V, control: &V, requested: i64) -> V {
        let map = self.field_map.motion_supported(scan).then(|| self.field_map.body_speed(scan)).flatten();
        let cone = self.race_crosswalk.cone_motion.measured_this_scan(scan.idx("seq").i());
        let steer = self.servo.max(requested).max(lp.get("current_pose_pwm").oi().unwrap_or(requested))
            .max(control.get("servo").oi().unwrap_or(requested));
        first_arc_drive(lp,map,cone,steer,self.wall_turn.active || self.wall_turn.late_sight)
    }
}

#[derive(Default, Clone, Copy, Debug, PartialEq, Eq)]
enum FirstApproach {
    #[default]
    Launch,
    Cruise,
    Brake,
    Coast,
    Hold,
}

impl CrosswalkExit {
    pub fn release_crosswalk(&mut self, now: f64, forward_integral: Option<f64>) {
        self.crosswalk_release = Some((now, forward_integral));
    }

    pub fn crosswalk_released(&self) -> bool { self.crosswalk_release.is_some() }

    pub fn is_braking(&self) -> bool { self.first_approach == FirstApproach::Brake }

    pub fn has_braked(&self) -> bool { self.brake_used }

    pub fn output_observed(&mut self, motor: i64) {
        if self.first_approach == FirstApproach::Brake && motor != LANE_BRAKE_MOTOR {
            self.brake_used = true;
            self.first_approach = FirstApproach::Hold;
        }
    }

    fn brake_need_m(v: f64, accel: f64, decel: f64, entry_mps: f64) -> f64 {
        let model = crate::race_light_brake::BrakeModel::light(1.0, decel);
        let power_tail = 0.056;
        let v_rel = (v + accel * power_tail).max(0.);
        (v + v_rel) * power_tail * 0.5
            + v_rel * crate::race_light_brake::LIGHT_BRAKE_RESPONSE_S
            + (model.distance(v_rel) - model.distance(entry_mps)).max(0.)
            + 0.66
    }

    /// 20 ms tick evaluation for pre-Cone-1 active 1100 electronic braking.
    /// Strictly one continuous <1500 pulse preceded by >1500 drive and exiting directly to Hold (>1500).
    pub fn command(&mut self, now: f64, forward_integral: Option<f64>) -> Option<i64> {
        let a = self.brake_anchor?;
        let age = now - a.at;
        if !(0. ..=SCAN_AGE_S).contains(&age) { return None; }
        let speed = a.speed.map(|v| (v + a.integral.zip(forward_integral.filter(|x| x.is_finite()))
            .map_or(0., |(old, cur)| cur - old)).max(0.));
        let distance = a.distance.map(|d| (d - a.speed.zip(speed).map_or(0., |(old, v)| (old + v) * age * 0.5)).max(0.));
        if matches!(self.first_approach, FirstApproach::Launch | FirstApproach::Cruise)
            && !self.brake_used
            && let (Some(d), Some(v)) = (distance, speed)
        {
            let brake_m = Self::brake_need_m(v, a.accel, a.decel, a.entry_mps);
            if d <= brake_m && v > a.entry_mps + 0.14 {
                self.brake_used = true;
                self.first_approach = FirstApproach::Brake;
                self.brake_since = Some(now);
                self.first_motor = Some(LANE_BRAKE_MOTOR);
                return Some(LANE_BRAKE_MOTOR);
            }
        }
        if self.first_approach == FirstApproach::Brake {
            let since = self.brake_since.unwrap_or(now);
            let done = speed.is_none_or(|v| v <= a.entry_mps + 0.25)
                || now - since >= 0.36
                || (distance.is_some_and(|d| d <= 0.10) && speed.is_none_or(|v| v <= a.entry_mps + 0.30));
            if done {
                self.first_approach = FirstApproach::Hold;
                let m = hold_pwm(speed, a.entry_mps);
                self.first_motor = Some(m);
                return Some(m);
            }
            self.first_motor = Some(LANE_BRAKE_MOTOR);
            return Some(LANE_BRAKE_MOTOR);
        }
        None
    }

    /// After the crosswalk the car is already rolling when the first support
    /// is acquired. Sprint 1650, then use one brief 1100 electronic brake to hit entry_mps cleanly.
    #[allow(clippy::too_many_arguments)]
    pub fn first_approach_drive(&mut self, radar: Option<f64>, body_speed: Option<f64>, before: Option<f64>, rolling_tangent: bool, decel: f64, now: f64, forward_integral: Option<f64>, hold_accel: f64, entry_mps: f64) -> (i64, V) {
        if self.last_drive_at.replace(now).is_some_and(|last| now - last > SCAN_AGE_S)
            && matches!(self.first_approach, FirstApproach::Launch | FirstApproach::Cruise | FirstApproach::Brake)
        {
            if self.first_approach == FirstApproach::Brake { self.brake_used = true; }
            self.first_approach = FirstApproach::Hold;
        }
        let inertial = self.crosswalk_release.and_then(|(_, zero)| zero)
            .zip(forward_integral).map(|(zero, integral)| (integral-zero).max(0.));
        let speed = body_speed.or(inertial).or(radar);
        let lag = 0.14;
        if self.first_approach == FirstApproach::Launch
            && (!self.crosswalk_released() || speed.is_some_and(|v| v + 5.0 * lag >= 2.72)) {
            self.first_approach = FirstApproach::Cruise;
        }
        let boosted = self.crosswalk_released() && self.first_approach == FirstApproach::Launch;
        let cur_accel = if boosted { 5.0 } else if self.first_approach == FirstApproach::Cruise { hold_accel.max(1.0) } else { 0. };
        let launch_tail = self.last_launch_at.map_or(0., |at| (lag - (now-at)).clamp(0., lag));
        let pending_rise = self.crosswalk_release.filter(|_| boosted)
            .map_or(0., |(since, _)| 5.0 * (now - since).clamp(0., lag))
            + if self.first_approach == FirstApproach::Cruise {
                5.0 * launch_tail + hold_accel * (lag-launch_tail)
            } else { 0. };
        let coast_m = speed.map(|v| {
            let released = v + pending_rise;
            (v + released)*lag/2. + (released*released - entry_mps*entry_mps).max(0.)/(2.*decel)
        });
        let brake_m = speed.map(|v| Self::brake_need_m(v, cur_accel, decel, entry_mps));
        let board_stop_m = speed.map(|v| {
            let released = v + pending_rise;
            FRONT_BODY_EXTENT_M + FIRST_APPROACH_BOARD_GAP_M + (v+released)*lag/2. + released*released/(2.*decel)
        });
        let integral = forward_integral.filter(|x| x.is_finite());
        self.brake_anchor = Some(CrosswalkBrakeAnchor {
            at: now, distance: before, speed, integral, decel, entry_mps, accel: cur_accel, rolling_tangent,
        });
        if matches!(self.first_approach, FirstApproach::Launch | FirstApproach::Cruise) {
            if !self.brake_used && before.zip(brake_m).is_some_and(|(d, b)| d <= b) {
                if speed.is_some_and(|v| v > entry_mps + 0.14) {
                    self.brake_used = true;
                    self.brake_since = Some(now);
                    self.first_approach = FirstApproach::Brake;
                } else {
                    self.first_approach = FirstApproach::Hold;
                }
            } else if boosted && (before.is_none() || speed.is_none()) {
                self.first_approach = FirstApproach::Coast;
            }
        }
        if self.first_approach == FirstApproach::Brake {
            if speed.is_none_or(|v| v <= entry_mps + 0.25)
                || self.brake_since.is_some_and(|since| now - since >= 0.36)
                || (before.is_some_and(|d| d <= 0.10) && speed.is_none_or(|v| v <= entry_mps + 0.30))
            {
                self.first_approach = FirstApproach::Hold;
            }
        }
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
        let motor = if self.first_approach == FirstApproach::Brake {
            LANE_BRAKE_MOTOR
        } else if self.first_approach == FirstApproach::Coast {
            if rolling_tangent && before.is_some() && speed.is_some_and(|v|v>entry_mps) {
                if speed.is_some_and(|v|v>entry_mps+LANE_SPEED_HOLD_MARGIN_MPS) {ROLLING_HOLD_MOTOR} else {ROUTE_SLOW_MOTOR}
            } else {NEUTRAL}
        }
            else if boosted && self.first_approach == FirstApproach::Launch { CROSSWALK_LAUNCH_MOTOR }
            else if speed.is_some_and(|v| v < LANE_SPEED_RESTART_MPS) { TRIAL_MOTOR }
            else if self.first_approach == FirstApproach::Cruise {
                if speed.is_some_and(|v| v < 2.75) { RACE_CRUISE_MOTOR } else { ROUTE_SLOW_MOTOR }
            }
            else { hold_pwm(speed, entry_mps) };
        let motor = if motor == CROSSWALK_LAUNCH_MOTOR {
            self.last_launch_at = Some(now);
            motor
        } else { motor };
        self.first_motor = Some(motor);
        (motor, dict! {"phase" => if phase=="coast" && motor>NEUTRAL {"rolling_deceleration"} else {phase}, "before_tangent_m" => before, "coast_to_entry_m" => coast_m,
                       "brake_to_entry_m" => brake_m, "board_stop_range_m" => board_stop_m, "speed_now_mps" => speed,
                       "speed_source" => if body_speed.is_some() { "recent_radar_anchor_with_short_imu" }
                           else if inertial.is_some() { "imu_from_crosswalk_rest" } else { "lidar" },
                       "entry_target_mps" => entry_mps, "early_rolling_hold" => hold_room})
    }

    fn first_phase(&self) -> &'static str {
        match self.first_approach { FirstApproach::Launch => "launch", FirstApproach::Cruise => "cruise", FirstApproach::Brake => "brake", FirstApproach::Coast => "coast", FirstApproach::Hold => "hold" }
    }

    pub fn first_rolling_hold(&self) -> bool { self.first_approach == FirstApproach::Hold }

    pub fn first_motor(&self) -> Option<i64> { self.first_motor }

    /// The board search can cut before a tangent is acquired. That is the
    /// same approach coast, not a separate pulse that may re-enable cruise.
    pub fn board_cut(&mut self) -> &'static str {
        if matches!(self.first_approach, FirstApproach::Launch | FirstApproach::Cruise | FirstApproach::Brake) {
            if self.first_approach == FirstApproach::Brake { self.brake_used = true; }
            self.first_approach = FirstApproach::Coast;
        }
        self.first_motor = Some(NEUTRAL);
        self.first_phase()
    }

    pub fn enter_first_bend(&mut self) {
        if self.first_approach == FirstApproach::Brake { self.brake_used = true; }
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
        assert_eq!(drive(0., 0., 4.), CROSSWALK_LAUNCH_MOTOR);
        assert_eq!(drive(0.2, 1.75, 4.), APPROACH_CRUISE_MOTOR);
        assert_eq!(drive(0.3, 2., 0.4), ROLLING_HOLD_MOTOR);
        assert_eq!(drive(0.4, 1.6, 0.2), 1570);
        assert_eq!(drive(0.5, 1.4, 5.), 1590);
        assert_eq!(drive(0.6, 1.7, 3.), 1565);
        assert!(exit.first_rolling_hold());

        let mut exit = CrosswalkExit::default();
        exit.release_crosswalk(0., Some(0.));
        exit.first_approach_drive(None, Some(2.), Some(4.), false, 0.6, 1., Some(2.1), 0.4, FIRST_ENTRY_MPS);
        exit.board_cut();
        assert_eq!(exit.first_motor(), Some(NEUTRAL));
        assert_eq!(exit.first_approach_drive(None, Some(1.70), Some(2.), true, 0.6, 1.1,
            Some(1.92), 0.4, FIRST_ENTRY_MPS).0, 1565);
        assert_eq!(exit.first_approach_drive(None, Some(1.6), Some(4.), true, 0.6, 1.2,
            Some(1.8), 0.4, FIRST_ENTRY_MPS).0, 1570);
    }

    #[test]
    fn first_arc_requires_current_speed_agreement_and_steering_reserve() {
        let lp=dict!{"support_source"=>"current_lidar","plan"=>dict!{"min_slack_m"=>0.096,"board_slack_m"=>0.107}};
        let motor=|m,c,s,w| first_arc_drive(&lp,m,c,s,w).idx("motor").i();
        assert_eq!(motor(Some(1.46),Some(0.41),1711,false),1550);
        assert_eq!(motor(Some(1.46),None,1711,false),1550);
        assert_eq!(motor(Some(1.46),Some(1.45),1720,false),1550);
        assert_eq!(motor(Some(1.46),Some(1.45),1711,true),1550);
        assert_eq!(motor(Some(1.46),Some(1.45),1711,false),1555);
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
