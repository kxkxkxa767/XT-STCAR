//! Longitudinal control from the first loop's exit to the second loop's tangent.
use crate::helpers::{MAX_DIAMETER_M, NEUTRAL, SCAN_AGE_S};
use crate::motion_sensors::ESC_COAST_DELAY_S;
use crate::maneuver::*;
use crate::maneuver_post::pairwise_track_speed;
use crate::py::*;
use crate::dict;

// Corner-entry target from y36. The speed plan ends at the cone's tangent.
const ENTRY_MPS: f64 = 1.8;
const ROLLING_MPS: f64 = 0.15;
const STRAIGHT_YAW_RAD_S: f64 = 0.6;
const STRAIGHT_STEER_BAND_PWM: i64 = 60;

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
    last_radar: Option<(f64, f64, Option<f64>)>, // receive time, speed, synchronized IMU integral
    last_motion: Option<(i64, f64, Option<f64>)>, // scan sequence, sensor time, speed along the straight
    station: f64,
    sighting: Option<Sighting>,
    preview_offset: Option<f64>,
    tangent_station: Option<f64>,
    distance_source: &'static str,
    preview_plan_count: u32,
    last_tangent_seq: Option<i64>,
}

#[derive(Clone, Copy, Debug)]
struct Sighting {
    seq: i64,
    at: f64,
    car_station: f64,
    cone_station: f64,
    lateral: f64,
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
    pub fn observe(&mut self, scan: &V, heading: Option<f64>, integral: Option<f64>) {
        let seq = scan.idx("seq").i();
        if self.last_motion.is_some_and(|(previous_seq, _, _)| previous_seq == seq) {
            return;
        }
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
        let received = scan.idx("received_at").f();
        let at = scan.idx("at_ms").f() / 1000.;
        if let Some(speed) = self.speed {
            self.last_radar = Some((received, speed, integral));
        }
        let v = self.speed.or_else(|| {
            let (_, radar, origin) = self.recent_radar(received)?;
            Some((radar + (integral? - origin?)).max(0.))
        });
        if let Some((_, previous_at, previous_v)) = self.last_motion {
            let dt = at - previous_at;
            // Two accepted wall positions directly measure forward progress.
            // Use short-window delta-v only for a discontinuity in the wall data.
            let wall_step = self
                .history
                .windows(2)
                .next_back()
                .filter(|w| (w[0].0 - previous_at).abs() < 1e-6)
                .map(|w| w[0].2 - w[1].2);
            let travelled = wall_step.or_else(|| previous_v.zip(v).map(|(a, b)| (a + b) * dt / 2.));
            if let Some(d) = travelled.filter(|_| dt > 0. && dt <= SCAN_AGE_S) {
                self.station += d.max(0.);
            } else {
                self.forget_reference();
                self.transfer_anchor = None;
            }
        }
        self.last_motion = Some((seq, at, v));
        if self.tangent_station.is_some() {
            self.distance_source = "carried_tangent";
        }
    }

    fn recent_radar(&self, now: f64) -> Option<(f64, f64, Option<f64>)> {
        self.last_radar.filter(|(at, _, _)| (0.0..=SCAN_AGE_S).contains(&(now - at)))
    }

    fn forget_reference(&mut self) {
        self.sighting = None;
        self.preview_offset = None;
        self.tangent_station = None;
        self.last_tangent_seq = None;
        self.distance_source = "unobserved";
    }

    /// A cone and the planned circle's longitudinal offset fix the tangent
    /// in the same coordinate throughout the transfer. The divider only steers.
    pub fn see_cone(&mut self, seq: i64, at: f64, along: f64, lateral: f64, plan_offset: impl FnOnce() -> Option<f64>) {
        let cone_station = self.station + along;
        let current = Sighting { seq, at, car_station: self.station, cone_station, lateral };
        if let Some(old) = self.sighting {
            if old.seq == seq {
                return;
            }
            let travelled = self.station - old.car_station;
            let same = (cone_station - old.cone_station).abs() <= MAX_DIAMETER_M
                && (lateral - old.lateral).abs() <= MAX_DIAMETER_M + travelled;
            if !same || (self.preview_offset.is_none() && !(0.0..=SCAN_AGE_S).contains(&(at - old.at))) {
                self.forget_reference();
            } else {
                if self.preview_offset.is_none() {
                    self.preview_plan_count += 1;
                    self.preview_offset = plan_offset();
                }
                if let Some(offset) = self.preview_offset {
                    self.set_tangent(along + offset, "second_cone_preview");
                }
            }
        }
        self.sighting = Some(current);
    }

    fn set_tangent(&mut self, distance: f64, source: &'static str) {
        let observed = self.station + distance.max(0.);
        // A newly observed closer turn shortens the available straight at
        // once; changing controller or refitting the circle cannot add runway.
        self.tangent_station = Some(self.tangent_station.map_or(observed, |old| old.min(observed)));
        self.distance_source = source;
        self.last_tangent_seq = self.last_motion.map(|(seq, _, _)| seq);
    }

    pub fn remaining(&self) -> Option<f64> {
        self.tangent_station.map(|target| (target - self.station).max(0.))
    }

    pub fn rolling(&self) -> bool {
        self.speed.is_some_and(|v| v >= ROLLING_MPS)
    }

    pub fn drive(&mut self, now: f64, live_tangent: Option<f64>, entering: bool, continuing: bool, decel: f64, forward_integral: Option<f64>) -> (i64, V) {
        if let Some(d) = live_tangent { self.set_tangent(d, "second_loop_plan"); }
        let distance = self.remaining();
        // The five-scan lidar slope lagged the y30 push: it still reported
        // 1.44 m/s while the IMU change from the pre-push anchor implied 2.52.
        let imu_speed = self.transfer_anchor.zip(forward_integral).filter(|_| self.recent_radar(now).is_some())
            .map(|((radar, origin), current)| (radar + current - origin).max(0.));
        let cut_speed = match (self.speed, imu_speed) {
            (Some(lidar), Some(live)) => Some(lidar.max(live)),
            (lidar, live) => lidar.or(live),
        };
        let coast_m = cut_speed.map(|v| {
            let lag = ESC_COAST_DELAY_S; // measured mean cut lag on the car
            let powered = match self.phase {
                TransferPhase::Boost(since) => (now - since).clamp(0., lag),
                _ => 0.,
            };
            let release_v = v + 4.0 * powered; // measured 1600 launch acceleration
            (v + release_v) * lag / 2. + (release_v * release_v - ENTRY_MPS * ENTRY_MPS).max(0.) / (2. * decel)
        });
        let rolling = self.rolling() || (matches!(self.phase, TransferPhase::Boost(_))
            && imu_speed.is_some_and(|v| v >= ROLLING_MPS));
        let room = rolling && distance.zip(coast_m).is_some_and(|(d, coast)| d > coast);
        self.phase = match self.phase {
            TransferPhase::Approach if entering && room => {
                self.transfer_anchor = self.speed.zip(forward_integral);
                TransferPhase::Boost(now)
            }
            TransferPhase::Approach if distance == Some(0.) => TransferPhase::Bend,
            TransferPhase::Boost(_) if !room || !continuing => TransferPhase::Coast,
            other => other,
        };
        if matches!(self.phase, TransferPhase::Coast) && cut_speed.is_none_or(|v| v <= ENTRY_MPS) {
            self.phase = TransferPhase::Bend;
            self.transfer_anchor = None;
        }
        let motor = match self.phase {
            TransferPhase::Boost(_) => TRANSFER_BOOST_MOTOR,
            TransferPhase::Coast => NEUTRAL,
            _ => ROUTE_SLOW_MOTOR,
        };
        (motor, dict! {"speed_mps" => self.speed, "speed_basis" => "rear_wall_return_axis",
                       "imu_anchored_speed_mps" => imu_speed, "speed_for_cut_mps" => cut_speed,
                       "straight_remaining_m" => distance, "tangent_source" => if distance.is_some() { self.distance_source } else { "unobserved" },
                       "tangent_measurement_seq" => self.last_tangent_seq, "preview_plan_count" => self.preview_plan_count as i64,
                       "transfer_progress_m" => self.station, "tangent_station_m" => self.tangent_station,
                       "observed_plan_remaining_m" => live_tangent, "preview_axis_offset_m" => self.preview_offset,
                       "rolling_source" => if self.rolling() { "lidar" } else if rolling { "anchored_imu" } else { "unavailable" },
                       "coast_to_entry_m" => coast_m, "transfer_motor" => motor})
    }

    pub fn dump(&self) -> V {
        let (phase, since) = match self.phase {
            TransferPhase::Approach => ("approach", None),
            TransferPhase::Boost(since) => ("boost", Some(since)),
            TransferPhase::Coast => ("coast", None),
            TransferPhase::Bend => ("bend", None),
        };
        dict! {"speed_mps" => self.speed, "phase" => phase, "boost_since" => since,
               "transfer_anchor" => self.transfer_anchor.map(|(radar, imu)| V::from(vec![radar, imu])),
               "tangent_station_m" => self.tangent_station, "transfer_progress_m" => self.station,
               "last_tangent_seq" => self.last_tangent_seq, "preview_plan_count" => self.preview_plan_count as i64,
               "history" => V::list(self.history.iter().map(|r| V::from(vec![r.0, r.1, r.2])).collect())}
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_handover_can_shorten_but_not_restore_spent_straight() {
        let mut t = RaceTransfer::default();
        t.see_cone(10, 1., 4., 0.4, || panic!("single observation"));
        assert_eq!(t.remaining(), None);
        t.station = 0.2;
        t.see_cone(11, 1.1, 3.8, 0.42, || Some(0.4));
        t.station = 1.;
        assert!((t.remaining().unwrap() - 3.4).abs() < 1e-9);
        t.set_tangent(3.9, "second_loop_plan");
        assert!((t.remaining().unwrap() - 3.4).abs() < 1e-9);
        t.set_tangent(2.7, "second_loop_plan");
        assert!((t.remaining().unwrap() - 2.7).abs() < 1e-9);
    }

    #[test]
    fn a_lidar_anchored_imu_can_continue_only_the_current_push() {
        let mut t = RaceTransfer::default();
        t.last_radar = Some((1., 1.2, Some(0.)));
        t.set_tangent(3.7, "second_cone_preview");
        assert_ne!(t.drive(1., None, true, true, 0.6, Some(0.)).0, TRANSFER_BOOST_MOTOR);
        t.speed = Some(1.2);
        assert_eq!(t.drive(1., None, true, true, 0.6, Some(0.)).0, TRANSFER_BOOST_MOTOR);
        t.speed = None;
        assert_eq!(t.drive(1.1, None, true, true, 0.6, Some(0.28)).0, TRANSFER_BOOST_MOTOR);
        let mut no_imu = t.clone();
        assert_ne!(no_imu.drive(1.2, None, true, true, 0.6, None).0, TRANSFER_BOOST_MOTOR);
        assert_ne!(t.drive(1.4, None, true, true, 0.6, Some(0.4)).0, TRANSFER_BOOST_MOTOR);
    }
}
