//! Longitudinal control from the first loop's exit to the second loop's tangent.
use crate::helpers::{NEUTRAL, SCAN_AGE_S};
use crate::motion_sensors::ESC_COAST_DELAY_S;
use crate::maneuver::*;
use crate::maneuver_post::pairwise_track_speed;
use crate::py::*;
use crate::dict;

// y36: y27 entered the second loop at 2.10 m/s without contact and y30 touched the
// board at ~2.5 (IMU). The 1.24 of y25-y35 (y22's natural 1550 handover speed) was met
// ~2 m before the tangent (the divider's open end lies ~.9 m short of it), so y31-y35
// crawled at 1550: first pass to tangent 3.3-3.8 s against y27's 2.0.
const ENTRY_MPS: f64 = 1.8;
// Rule-range simulation (wide lanes): the passage between cone two and the start divider's top,
// measured by the free-space probe, was the strongest predictor of contact. Below .9 m the car
// has under 60 cm of play, and the 1.8 m/s tangent runs wide. The practice field measures .85 or more.
const PASSAGE_FULL_M: f64 = 0.85;
const PASSAGE_TIGHT_M: f64 = 0.6;
const TIGHT_ENTRY_MPS: f64 = 1.1;

/// Cone two's entry speed for this passage on the board side.
pub fn passage_entry_mps(passage: f64) -> Option<f64> {
    (passage < PASSAGE_FULL_M).then(|| {
        TIGHT_ENTRY_MPS + (ENTRY_MPS - TIGHT_ENTRY_MPS) * ((passage - PASSAGE_TIGHT_M) / (PASSAGE_FULL_M - PASSAGE_TIGHT_M)).clamp(0., 1.)
    })
}
const ROLLING_MPS: f64 = 0.15;
const STRAIGHT_YAW_RAD_S: f64 = 0.6;
const STRAIGHT_STEER_BAND_PWM: i64 = 60;
// y76/y77: after the cut the IMU speed peaked in 0.092/0.056 s and gained
// 0.183/0.148 m/s, not the old fixed 4 * 0.2 = 0.8 m/s prediction.
const BOOST_CUT_RESPONSE_S: f64 = 0.1;

#[derive(Clone, Copy, Debug)]
struct DriveAnchor {
    at: f64,
    distance: Option<f64>,
    speed: Option<f64>,
    integral: Option<f64>,
    acceleration: f64,
    decel: f64,
    entry_mps: f64,
    continuing: bool,
    hold_accel: f64,
}

#[derive(Clone, Copy, Debug)]
struct TransferControl {
    at: f64,
    speed: Option<f64>,
    distance: Option<f64>,
    coast: Option<f64>,
    motor: i64,
}

/// A small steering correction can continue a push already in progress.
pub fn steering_window(yaw: f64, target_pwm: i64, servo: i64) -> (bool, bool) {
    let low_yaw = yaw.abs() <= STRAIGHT_YAW_RAD_S;
    let target_near_center = (target_pwm - NEUTRAL).abs() <= STRAIGHT_STEER_BAND_PWM;
    (low_yaw && (target_near_center || (servo - NEUTRAL).abs() <= STRAIGHT_STEER_BAND_PWM), low_yaw || target_near_center)
}

#[derive(Clone, Debug, Default)]
pub struct RaceTransfer {
    history: Vec<(f64, f64, f64)>,
    speed: Option<f64>,
    phase: TransferPhase,
    transfer_anchor: Option<(f64, f64)>,
    waiting_for_support: bool,
    phase_before_wait: Option<TransferPhase>,
    previous_integral: Option<(f64, f64)>,
    drive_anchor: Option<DriveAnchor>,
    control: Option<TransferControl>,
    /// The channel's board as last identified (lateral, metres).
    pub channel_board: Option<f64>,
    /// Cone two's entry speed when the passage on its board side is narrow.
    pub tight_entry: Option<f64>,
}

#[derive(Default, Clone, Copy, Debug)]
enum TransferPhase {
    #[default]
    Approach,
    Boost(f64),
    Coast,
    Bend,
}

impl RaceTransfer {
    /// The end wall behind recedes along the start axis. A cone can cover
    /// the centre ray; nine projected rays retained the wall in y22 and y24.
    pub fn observe(&mut self, scan: &V, heading: Option<f64>) {
        let track = heading.and_then(|h| {
            let center = round(degrees(wrap(h)));
            let ranges = scan.idx("ranges").l();
            let mut rear = Vec::with_capacity(9);
            for i in (center - 12..=center + 12).step_by(3) {
                let r = &ranges[imod(i, 360) as usize];
                if r.is_num() {
                    rear.push(r.f() * (radians(i as f64) - h).pcos());
                }
            }
            (!rear.is_empty()).then(|| -median(rear))
        });
        self.speed = pairwise_track_speed(&mut self.history, scan, track);
    }

    pub fn rolling(&self) -> bool {
        self.speed.is_some_and(|v| v >= ROLLING_MPS)
    }

    /// Geometry can temporarily lack a confirmed second support while the
    /// vehicle is still moving. Keep measurements and steering alive, but do
    /// not reapply power until the support is back in view or confirmed.
    pub fn coast_for_support(&mut self) {
        if !self.waiting_for_support {
            self.phase_before_wait = Some(self.phase);
        }
        self.waiting_for_support = true;
        self.phase = TransferPhase::Coast;
    }

    pub fn waiting_for_support(&self) -> bool { self.waiting_for_support }

    /// The wait ends in the phase it interrupted; a boost is not resumed. y71: cone two appeared
    /// one scan after the board came inside the stop range and the latched coast ran from
    /// seq 132 to 138, cutting the tangent speed to 1.48 m/s (y67: 1.77).
    pub fn support_handed_over(&mut self) {
        self.waiting_for_support = false;
        if let Some(phase) = self.phase_before_wait.take().filter(|p| !matches!(p, TransferPhase::Boost(_))) {
            self.phase = phase;
        }
    }

    /// Metadata for the separate load observer; it does not change transfer
    /// speed, phase, boost anchors or the existing entry-speed selection.
    pub fn surface_axis_speed(&self) -> Option<crate::race_surface_speed::AxisSpeed> {
        Some(crate::race_surface_speed::AxisSpeed {
            speed_mps: self.speed?,
            window_start_at: self.history.first()?.1,
            window_end_at: self.history.last()?.1,
            axis_heading_rad: PI,
        })
    }

    #[allow(clippy::too_many_arguments)]
    pub fn drive(&mut self, now: f64, distance: Option<f64>, entering: bool, continuing: bool, decel: f64, forward_integral: Option<f64>, body_speed: Option<f64>, cone_speed: Option<f64>, surface: &crate::race_surface::SurfaceResponse) -> (i64, V) {
        // The five-scan lidar slope lagged the y30 push: it still reported
        // 1.44 m/s while the IMU change from the pre-push anchor implied 2.52.
        let imu_speed = self.transfer_anchor.zip(forward_integral)
            .map(|((radar, origin), current)| (radar + current - origin).max(0.));
        let cut_speed = cone_speed.or_else(|| if matches!(self.phase, TransferPhase::Coast) {
            // The range slope still shows the earlier peak while slowing. y77
            // seq 134: radar 2.04, current IMU 1.71; max() prolonged neutral.
            imu_speed.or(body_speed).or(self.speed)
        } else { match (self.speed, imu_speed) {
            (Some(lidar), Some(live)) => Some(lidar.max(live)),
            (lidar, live) => lidar.or(live),
        }});
        let integral = forward_integral.filter(|v| v.is_finite());
        let acceleration = self.previous_integral.zip(integral)
            .filter(|((at, _), _)| 0. < now - at && now - at <= SCAN_AGE_S)
            .map(|((at, before), current)| ((current - before) / (now - at)).max(0.))
            .unwrap_or(4.0);
        if let Some(current) = integral { self.previous_integral = Some((now, current)); }
        let entry_mps = self.tight_entry.unwrap_or(ENTRY_MPS);
        let before_tangent = distance.is_some_and(|d| d > 0.);
        let hold_motor = if continuing || before_tangent { hold_pwm(cut_speed, entry_mps) } else { RACE_CRUISE_MOTOR };
        let hold_accel = surface.predicted_hold_accel(hold_motor, cone_speed.or(body_speed), now)
            .or_else(|| surface.predicted_drive_accel(hold_motor, cone_speed.or(body_speed), now)).unwrap_or(0.);
        let anchor = DriveAnchor { at: now, distance, speed: cut_speed, integral,
            acceleration, decel, entry_mps, continuing, hold_accel };
        let coast_m = self.coast_budget(now, cut_speed, anchor);
        let room = self.rolling() && distance.zip(coast_m).is_some_and(|(d, coast)| d > coast);
        self.phase = match self.phase {
            TransferPhase::Approach if entering && room => {
                self.transfer_anchor = self.speed.zip(forward_integral);
                TransferPhase::Boost(now)
            }
            TransferPhase::Approach if distance == Some(0.) => TransferPhase::Bend,
            other => other,
        };
        self.drive_anchor = Some(anchor);
        let motor = self.command(now, integral).unwrap_or(hold_motor);
        let coast_m = self.control.and_then(|c| c.coast).or(coast_m);
        (motor, dict! {"speed_mps" => self.speed, "speed_basis" => "rear_wall_return_axis",
                       "imu_anchored_speed_mps" => imu_speed, "speed_for_cut_mps" => cut_speed,
                       "cone_body_speed_mps" => cone_speed,
                       "straight_remaining_m" => distance, "coast_to_entry_m" => coast_m, "transfer_motor" => motor,
                       "hold_accel_mps2" => hold_accel, "boost_accel_mps2" => acceleration,
                       "boost_cut_response_s" => BOOST_CUT_RESPONSE_S,
                       "waiting_for_second_support" => self.waiting_for_support})
    }

    fn coast_budget(&self, now: f64, speed: Option<f64>, a: DriveAnchor) -> Option<f64> {
        speed.map(|v| {
            let (lag, acceleration) = match self.phase {
                TransferPhase::Boost(since) => (BOOST_CUT_RESPONSE_S,
                    if now - since <= BOOST_CUT_RESPONSE_S { 4.0 } else { a.acceleration }),
                TransferPhase::Approach => (ESC_COAST_DELAY_S, a.hold_accel),
                _ => (0., 0.),
            };
            let release = v + acceleration * lag;
            (v + release) * lag / 2. + (release * release - a.entry_mps * a.entry_mps).max(0.) / (2. * a.decel)
        })
    }

    /// Re-evaluate the one boost/cut on the existing 20 ms tick. A fresh scan
    /// still owns geometry; the IMU transports speed and distance between scans.
    pub fn command(&mut self, now: f64, integral: Option<f64>) -> Option<i64> {
        let a = self.drive_anchor?;
        let age = now - a.at;
        if !(0. ..=SCAN_AGE_S).contains(&age) { return None; }
        let speed = a.speed.map(|v| (v + a.integral.zip(integral)
            .map_or(0., |(before, current)| current - before)).max(0.));
        let distance = a.distance.map(|d| (d - a.speed.zip(speed).map_or(0., |(old, v)| (old + v) * age / 2.)).max(0.));
        let coast = self.coast_budget(now, speed, a);
        if matches!(self.phase, TransferPhase::Boost(_))
            && (!a.continuing || !distance.zip(coast).is_some_and(|(d, c)| d > c)) {
            self.phase = TransferPhase::Coast;
        }
        if !self.waiting_for_support && matches!(self.phase, TransferPhase::Coast) && speed.is_none_or(|v| v <= a.entry_mps) {
            self.phase = TransferPhase::Bend;
            self.transfer_anchor = None;
        }
        let motor = if self.waiting_for_support { NEUTRAL } else { match self.phase {
            TransferPhase::Boost(_) => TRANSFER_BOOST_MOTOR,
            TransferPhase::Coast => NEUTRAL,
            // User 11-04 (open PWM steps): out of cone one and straight again, hold toward the
            // cone-two entry speed instead of 1550 (y53: 1550 at 1.29-1.41 m/s for .5 s).
            TransferPhase::Approach if a.continuing || distance.is_some_and(|d| d > 0.) => hold_pwm(speed, a.entry_mps),
            // Cutting the boost does not mean the physical bend has begun:
            // y77 still had over 2 m to travel when the old state fixed 1550.
            // Steering eligibility ends the boost, not the approach speed hold.
            // y86 resumed 1570 with 1.28 m still to its tangent because a heading
            // correction made continuing=false. The geometric tangent owns
            // this handover, even while correcting the approach heading.
            TransferPhase::Bend if distance.is_some_and(|d| d > 0.) => hold_pwm(speed, a.entry_mps),
            _ if self.tight_entry.is_some_and(|e| speed.is_some_and(|v| v > e)) => NEUTRAL,
            _ => RACE_CRUISE_MOTOR,
        }};
        self.control = Some(TransferControl { at: now, speed, distance, coast, motor });
        Some(motor)
    }

    pub fn dump(&self) -> V {
        let (phase, since) = match self.phase {
            TransferPhase::Approach => ("approach", None),
            TransferPhase::Boost(since) => ("boost", Some(since)),
            TransferPhase::Coast => ("coast", None),
            TransferPhase::Bend => ("bend", None),
        };
        dict! {"speed_mps" => self.speed, "phase" => phase, "boost_since" => since,
               "waiting_for_second_support" => self.waiting_for_support,
               "control_at" => self.control.map(|c| c.at), "control_speed_mps" => self.control.and_then(|c| c.speed),
               "control_remaining_m" => self.control.and_then(|c| c.distance), "control_coast_to_entry_m" => self.control.and_then(|c| c.coast),
               "control_motor" => self.control.map(|c| c.motor),
               "transfer_anchor" => self.transfer_anchor.map(|(radar, imu)| V::from(vec![radar, imu])),
               "history" => V::list(self.history.iter().map(|r| V::from(vec![r.0, r.1, r.2])).collect())}
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn heading_correction_does_not_end_the_hold_before_the_actual_tangent() {
        let mut transfer = RaceTransfer {
            phase: TransferPhase::Bend,
            drive_anchor: Some(DriveAnchor { at: 1., distance: Some(1.28), speed: Some(2.06),
                integral: Some(0.), acceleration: 0., decel: 0.6, entry_mps: 1.8,
                continuing: false, hold_accel: 0. }),
            ..Default::default()
        };
        assert_eq!(transfer.command(1., Some(0.)), Some(ROUTE_SLOW_MOTOR));
        transfer.drive_anchor.as_mut().unwrap().continuing = true;
        assert_eq!(transfer.command(1., Some(0.)), Some(ROUTE_SLOW_MOTOR));
        transfer.drive_anchor.as_mut().unwrap().distance = Some(0.);
        assert_eq!(transfer.command(1., Some(0.)), Some(RACE_CRUISE_MOTOR));
        transfer.phase = TransferPhase::Coast;
        transfer.speed = Some(2.06);
        transfer.transfer_anchor = Some((1.75, 0.));
        let (motor, _) = transfer.drive(2., Some(1.28), false, false, 0.6, Some(0.), None,
            Some(2.16), &crate::race_surface::SurfaceResponse::default());
        assert_eq!(motor, NEUTRAL, "fresh cone motion must not be replaced by the old low boost anchor");
    }

    #[test]
    fn boost_cuts_between_scans_and_current_speed_ends_the_coast() {
        let mut transfer = RaceTransfer {
            speed: Some(2.2), phase: TransferPhase::Boost(0.5),
            transfer_anchor: Some((2., 0.)),
            drive_anchor: Some(DriveAnchor { at: 1., distance: Some(1.4), speed: Some(2.),
                integral: Some(0.), acceleration: 1., decel: 0.6, entry_mps: 1.8,
                continuing: true, hold_accel: 0. }),
            ..Default::default()
        };
        assert_eq!(transfer.command(1., Some(0.)), Some(TRANSFER_BOOST_MOTOR));
        assert_eq!(transfer.command(1.06, Some(0.06)), Some(NEUTRAL));
        let (motor, _) = transfer.drive(1.2, Some(1.), false, true, 0.6, Some(-0.3), None,
            None,
            &crate::race_surface::SurfaceResponse::default());
        assert!(motor > NEUTRAL && motor < TRANSFER_BOOST_MOTOR,
            "a delayed radar peak must not extend the coast after current speed falls below entry speed");
        assert_eq!(transfer.command(1.6, Some(-0.4)), None);
    }
}
