//! Longitudinal control from the first loop's exit to the second loop's tangent.
use crate::helpers::NEUTRAL;
use crate::maneuver::*;
use crate::maneuver_post::pairwise_track_speed;
use crate::py::*;
use crate::dict;

const ENTRY_MPS: f64 = 1.24; // y22, second-support handover

#[derive(Clone, Debug, Default)]
pub struct RaceTransfer {
    history: Vec<(f64, f64, f64)>,
    speed: Option<f64>,
    phase: TransferPhase,
    pub imu_yaw_rate: Option<f64>,
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
        self.speed.is_some_and(|v| v >= LANE_SPEED_RESTART_MPS)
    }

    pub fn drive(&mut self, now: f64, distance: Option<f64>, entering: bool, continuing: bool, decel: f64) -> (i64, V) {
        let coast_m = self.speed.map(|v| {
            let lag = CROSSWALK_AIM_LAG_S; // measured mean cut lag on the car
            let powered = match self.phase {
                TransferPhase::Boost(since) => (now - since).clamp(0., lag),
                _ => 0.,
            };
            let release_v = v + STRAIGHT_ACCEL_MAX_MPS2 * powered;
            (v + release_v) * lag / 2. + (release_v * release_v - ENTRY_MPS * ENTRY_MPS).max(0.) / (2. * decel)
        });
        let room = self.rolling() && distance.zip(coast_m).is_some_and(|(d, coast)| d > coast);
        self.phase = match self.phase {
            TransferPhase::Approach if entering && room => TransferPhase::Boost(now),
            TransferPhase::Approach if distance == Some(0.) => TransferPhase::Bend,
            TransferPhase::Boost(_) if !room || !continuing => TransferPhase::Coast,
            other => other,
        };
        if matches!(self.phase, TransferPhase::Coast) && self.speed.is_none_or(|v| v <= ENTRY_MPS) {
            self.phase = TransferPhase::Bend;
        }
        let motor = match self.phase {
            TransferPhase::Boost(_) => TRANSFER_BOOST_MOTOR,
            TransferPhase::Coast => NEUTRAL,
            _ => ROUTE_SLOW_MOTOR,
        };
        (motor, dict! {"speed_mps" => self.speed, "speed_basis" => "rear_wall_return_axis",
                       "straight_remaining_m" => distance, "coast_to_entry_m" => coast_m, "transfer_motor" => motor})
    }

    pub fn dump(&self) -> V {
        let (phase, since) = match self.phase {
            TransferPhase::Approach => ("approach", None),
            TransferPhase::Boost(t) => ("boost", Some(t)),
            TransferPhase::Coast => ("coast", None),
            TransferPhase::Bend => ("bend", None),
        };
        dict! {"speed_mps" => self.speed, "phase" => phase, "boost_since" => since, "imu_yaw_rate" => self.imu_yaw_rate,
               "history" => V::list(self.history.iter().map(|r| V::from(vec![r.0, r.1, r.2])).collect())}
    }
}
