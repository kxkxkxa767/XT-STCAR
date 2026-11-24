//! First-cone association during vehicle motion, in the lidar body frame.
use super::{MAX_AGE_S, associated, point_of};
use crate::py::V;

/// Two real, confirmed returns of the same first cone, in their acquisition body frames.
#[derive(Clone, Copy, Debug)]
struct ConeFix {
    id: i64,
    seq: i64,
    at_ms: i64,
    point: (f64, f64),
    yaw_rate: f64,
}

#[derive(Clone, Debug, Default)]
pub(super) struct FirstConeMotion {
    previous: Option<ConeFix>,
    latest: Option<ConeFix>,
}

#[derive(Clone, Copy)]
pub(super) struct MotionPrior {
    pub(super) transported: (f64, f64),
    pub(super) radar_trend: (f64, f64),
}

fn rotate_body(point: (f64, f64), yaw_change: f64) -> (f64, f64) {
    let (s, c) = yaw_change.sin_cos();
    (c * point.0 + s * point.1, -s * point.0 + c * point.1)
}

impl FirstConeMotion {
    pub(super) fn observe(&mut self, target: &V, rate: Option<f64>) {
        let Some(rate) = rate.filter(|r| r.is_finite()) else {
            return;
        };
        if !target.get("confirmed").is_true() || target.get("tracking_only_boundary_gap").is_true()
        {
            return;
        }
        let sample = ConeFix {
            id: target.idx("track_id").i(),
            seq: target.idx("source_seq").i(),
            at_ms: target.idx("source_at_ms").i(),
            point: point_of(target.idx("point_left_m")),
            yaw_rate: rate,
        };
        if self
            .latest
            .is_some_and(|last| last.id == sample.id && last.seq == sample.seq)
        {
            return;
        }
        self.previous = self
            .latest
            .filter(|last| last.id == sample.id && last.at_ms < sample.at_ms);
        self.latest = Some(sample);
    }

    pub(super) fn prior(&self, scan: &V, target: &V, rate: Option<f64>) -> Option<MotionPrior> {
        let (a, b) = (self.previous?, self.latest?);
        let r = rate.filter(|r| r.is_finite())?;
        if target.get("track_id").oi() != Some(b.id) || target.get("source_seq").oi() != Some(b.seq)
        {
            return None;
        }
        let previous_dt = (b.at_ms - a.at_ms) as f64 * 0.001;
        let dt = scan.get("at_ms").oi()?.checked_sub(b.at_ms)? as f64 * 0.001;
        let received_dt = scan.get("received_at").of()? - target.get("source_received_at").of()?;
        if !(0.0 < received_dt && received_dt < MAX_AGE_S) {
            return None;
        }
        if !(0.0 < previous_dt && previous_dt < MAX_AGE_S && 0.0 < dt && dt < MAX_AGE_S) {
            return None;
        }

        // De-rotate the preceding real echo with the gyro. Its motion along the
        // midpoint heading measures distance travelled without a fixed course map.
        let turn = (a.yaw_rate + b.yaw_rate) * 0.5 * previous_dt;
        let prior = rotate_body(a.point, turn);
        let (s, c) = (0.5 * turn).sin_cos();
        let travelled = (prior.0 - b.point.0) * c - (prior.1 - b.point.1) * s;
        let scale = dt / previous_dt;
        let turn = (b.yaw_rate + r) * 0.5 * dt;
        let rotated = rotate_body(b.point, turn);
        let (s, c) = (0.5 * turn).sin_cos();
        Some(MotionPrior {
            transported: (
                rotated.0 - travelled * scale * c,
                rotated.1 + travelled * scale * s,
            ),
            radar_trend: (
                b.point.0 + (b.point.0 - a.point.0) * scale,
                b.point.1 + (b.point.1 - a.point.1) * scale,
            ),
        })
    }
}

fn reference_at(old: &V, point: (f64, f64)) -> V {
    let mut reference = old.clone();
    reference.set("point_left_m", V::from(point));
    reference.set("bearing_left_rad", point.1.atan2(point.0));
    reference
}

impl MotionPrior {
    /// Keep a single observed cluster inside both existing association gates.
    /// Multiple matches are left to the normal hold/reacquisition logic.
    pub(super) fn unique_current(&self, candidates: &[V], old: &V, slack: f64) -> Option<(V, V)> {
        let carried = reference_at(old, self.transported);
        let trend = reference_at(old, self.radar_trend);
        let mut matches = candidates.iter().filter(|candidate| {
            !candidate.get("tracking_only_boundary_gap").is_true()
                && associated(candidate, &carried, slack)
                && associated(candidate, &trend, slack)
        });
        let real = matches.next()?.clone();
        if matches.next().is_some() {
            return None;
        }
        Some((real, carried))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dict;
    fn observation(seq: i64, at: i64, x: f64, y: f64) -> V {
        dict! {"source_seq"=>seq, "source_at_ms"=>at, "source_received_at"=>at as f64/1000.,
        "track_id"=>1,"point_left_m"=>V::from((x,y)),"bearing_left_rad"=>y.atan2(x),
        "diameter_m"=>0.17,"confirmed"=>true,"tracking_only_boundary_gap"=>false}
    }
    #[test]
    fn first_cone_recovers_a_unique_current_return_when_the_wall_heading_jumps() {
        let previous = observation(20, 1000, 0.38, 1.08);
        let accepted = observation(21, 1100, 0.35, 0.99);
        let current = observation(22, 1200, 0.34, 0.91);
        let mut motion = FirstConeMotion::default();
        motion.observe(&previous, Some(1.55));
        motion.observe(&accepted, Some(1.75));
        let scan = dict! {"at_ms"=>1200,"received_at"=>1.2};
        let prior = motion.prior(&scan, &accepted, Some(1.70)).unwrap();
        let wall = reference_at(&accepted, rotate_body((0.35, 0.99), 30_f64.to_radians()));
        assert!(!associated(&current, &wall, 0.));
        assert!(
            prior
                .unique_current(std::slice::from_ref(&current), &accepted, 0.)
                .is_some()
        );
        assert!(prior.unique_current(&[], &accepted, 0.).is_none());
        let other = observation(22, 1200, 0.37, 0.92);
        assert!(
            prior
                .unique_current(&[current.clone(), other], &accepted, 0.)
                .is_none()
        );
        let spike = motion.prior(&scan, &accepted, Some(27.)).unwrap();
        assert!(
            spike
                .unique_current(std::slice::from_ref(&current), &accepted, 0.)
                .is_none()
        );
        assert!(motion.prior(&scan, &accepted, None).is_none());
        assert!(
            motion
                .prior(
                    &dict! {"at_ms"=>1510,"received_at"=>1.51},
                    &accepted,
                    Some(1.)
                )
                .is_none()
        );
        let mut fragment = current;
        fragment.set("tracking_only_boundary_gap", true);
        assert!(prior.unique_current(&[fragment], &accepted, 0.).is_none());
    }
}
