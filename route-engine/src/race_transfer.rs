//! Longitudinal control from the first loop's exit to the second loop's tangent.
use crate::helpers::NEUTRAL;
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

    #[allow(clippy::too_many_arguments)]
    pub fn drive(&mut self, now: f64, distance: Option<f64>, entering: bool, continuing: bool, decel: f64, forward_integral: Option<f64>, body_speed: Option<f64>, surface: &crate::race_surface::SurfaceResponse) -> (i64, V) {
        // The five-scan lidar slope lagged the y30 push: it still reported
        // 1.44 m/s while the IMU change from the pre-push anchor implied 2.52.
        let imu_speed = self.transfer_anchor.zip(forward_integral)
            .map(|((radar, origin), current)| (radar + current - origin).max(0.));
        let cut_speed = match (self.speed, imu_speed) {
            (Some(lidar), Some(live)) => Some(lidar.max(live)),
            (lidar, live) => lidar.or(live),
        };
        let hold_motor = if continuing { hold_pwm(cut_speed, ENTRY_MPS) } else { ROUTE_SLOW_MOTOR };
        let hold_accel = surface.predicted_hold_accel(hold_motor, body_speed, now).unwrap_or(0.);
        let coast_m = cut_speed.map(|v| {
            let lag = ESC_COAST_DELAY_S; // measured mean cut lag on the car
            let pending_rise = match self.phase {
                TransferPhase::Boost(since) => 4.0 * (now - since).clamp(0., lag),
                TransferPhase::Approach => hold_accel * lag,
                _ => 0.,
            };
            let release_v = v + pending_rise;
            (v + release_v) * lag / 2. + (release_v * release_v - ENTRY_MPS * ENTRY_MPS).max(0.) / (2. * decel)
        });
        let room = self.rolling() && distance.zip(coast_m).is_some_and(|(d, coast)| d > coast);
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
            // User 11-04 (open PWM steps): out of cone one and straight again, hold toward the
            // cone-two entry speed instead of 1550 (y53: 1550 at 1.29-1.41 m/s for .5 s).
            TransferPhase::Approach if continuing => hold_motor,
            _ => ROUTE_SLOW_MOTOR,
        };
        (motor, dict! {"speed_mps" => self.speed, "speed_basis" => "rear_wall_return_axis",
                       "imu_anchored_speed_mps" => imu_speed, "speed_for_cut_mps" => cut_speed,
                       "straight_remaining_m" => distance, "coast_to_entry_m" => coast_m, "transfer_motor" => motor,
                       "hold_accel_mps2" => hold_accel})
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
               "history" => V::list(self.history.iter().map(|r| V::from(vec![r.0, r.1, r.2])).collect())}
    }
}
