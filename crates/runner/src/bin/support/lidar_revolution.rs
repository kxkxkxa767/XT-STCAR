//! N10 packets to published 360-bin revolutions (round 104).
//!
//! A revolution is the run of packets from the first one whose start angle is
//! more than 180 deg below its predecessor's. The packet straddling 0 deg thus
//! puts its points past 0 deg into the revolution it started in, and the next
//! revolution's first packet starts where that straddle ended. As the packet
//! phase precesses backwards (round 79 at rest: ~1.7 deg per revolution) the
//! sliver between a revolution's own straddle end and its first packet was
//! never sampled (raw bins 0..12: 1-3 null bins in 70 % of car run 103's
//! scans), and a revolution without a straddling packet lacked a whole packet
//! there (car run 103: raw 0-10 and 0-11, the two front_sparse neutral pauses
//! of the first loop). A bin this revolution never sampled now takes the
//! previous revolution's point past 0 deg, measured right before this
//! revolution's first packet and so as old as the points beside it; a sampled
//! bin keeps only this revolution's points.
use xt_stcar_robot_core::protocol::n10::POINTS_PER_PACKET;

pub struct Published {
    pub ranges: Vec<Option<f64>>,
    pub valid: usize,
    pub coverage: usize,
    pub seam_filled: usize,
}

pub struct Revolution {
    bins: Vec<Option<f64>>,
    seen: Vec<bool>,
    // Points past 0 deg: this revolution's (tail) and the previous one's (carry).
    tail: Vec<Option<f64>>,
    tail_seen: Vec<bool>,
    carry: Vec<Option<f64>>,
    carry_seen: Vec<bool>,
    prev: u16,
    synced: bool,
}

fn keep_nearest(slot: &mut Option<f64>, r: Option<f64>) {
    if let Some(r) = r {
        *slot = Some(slot.map_or(r, |old| old.min(r)));
    }
}

impl Revolution {
    pub fn new() -> Self {
        Self {
            bins: vec![None; 360],
            seen: vec![false; 360],
            tail: vec![None; 360],
            tail_seen: vec![false; 360],
            carry: vec![None; 360],
            carry_seen: vec![false; 360],
            prev: 0,
            synced: false,
        }
    }

    /// Adds one decoded packet; returns the revolution it completes, if any.
    pub fn push(
        &mut self,
        start_cdeg: u16,
        end_cdeg: u16,
        ranges: &[Option<f64>; POINTS_PER_PACKET],
    ) -> Option<Published> {
        let mut done = None;
        if self.prev > start_cdeg.saturating_add(18000) {
            if self.synced {
                let mut seam_filled = 0;
                for bin in 0..360 {
                    if !self.seen[bin] && self.carry_seen[bin] {
                        self.seen[bin] = true;
                        self.bins[bin] = self.carry[bin];
                        seam_filled += 1;
                    }
                }
                done = Some(Published {
                    ranges: self.bins.clone(),
                    valid: self.bins.iter().filter(|r| r.is_some()).count(),
                    coverage: self.seen.iter().filter(|v| **v).count(),
                    seam_filled,
                });
            }
            self.bins.fill(None);
            self.seen.fill(false);
            std::mem::swap(&mut self.carry, &mut self.tail);
            std::mem::swap(&mut self.carry_seen, &mut self.tail_seen);
            self.tail.fill(None);
            self.tail_seen.fill(false);
            self.synced = true;
        }
        self.prev = start_cdeg;
        let span = (i32::from(end_cdeg) - i32::from(start_cdeg)).rem_euclid(36000) as f64;
        for (i, r) in ranges.iter().enumerate() {
            let angle =
                (f64::from(start_cdeg) + span * i as f64 / (POINTS_PER_PACKET - 1) as f64) / 100.0;
            let rounded = angle.round();
            let bin = (rounded as usize) % 360;
            self.seen[bin] = true;
            keep_nearest(&mut self.bins[bin], *r);
            if rounded >= 360.0 {
                self.tail_seen[bin] = true;
                keep_nearest(&mut self.tail[bin], *r);
            }
        }
        done
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A lidar whose points are `step` cdeg apart, 16 per packet, from
    /// `first`: the packet phase moves by 16 * step * n - 360 deg a turn.
    fn stream(
        step: u32,
        first: u32,
        packets: usize,
        mut drop: impl FnMut(u32) -> bool,
    ) -> Vec<Published> {
        let mut revolution = Revolution::new();
        let mut out = Vec::new();
        let mut start = first;
        let wire = |a: u32| (a % 36_000) as u16;
        for _ in 0..packets {
            let ranges = std::array::from_fn(|i| Some(1.0 + i as f64 / 100.0));
            if !drop(start) {
                out.extend(revolution.push(wire(start), wire(start + 15 * step), &ranges));
            }
            start = (start + 16 * step) % 36_000;
        }
        out
    }

    #[test]
    fn a_backward_precessing_phase_leaves_no_seam_gap() {
        // 12.8-deg packet steps: the phase moves back 1.6 deg a turn.
        let frames = stream(80, 30_000, 29 * 60, |_| false);
        assert!(frames.len() > 50);
        assert!(
            frames[1..]
                .iter()
                .all(|f| f.coverage == 360 && f.valid == 360)
        );
        // Without the previous turn's points past 0 deg most frames had a gap.
        let filled = frames[1..].iter().filter(|f| f.seam_filled > 0).count();
        assert!(filled * 2 > frames.len(), "{filled}");
        assert!(frames[1..].iter().all(|f| f.seam_filled <= 3));
    }

    #[test]
    fn a_revolution_missing_its_packet_past_zero_takes_the_previous_turns() {
        // 12-deg packet steps from 11.6 deg: every turn ends with the packet
        // 359.6 -> 370.85 deg (bins 0-11). Once it is missing, as in car run
        // 103 (raw 0-11 null, 12+ and 359 sampled), the turn before fills it.
        let mut seen = 0;
        let frames = stream(75, 1_160, 30 * 12, |start| {
            seen += usize::from(start == 35_960);
            start == 35_960 && seen == 2
        });
        assert_eq!(seen, 12);
        assert!(frames[1..].iter().all(|f| f.coverage == 360));
        let filled: Vec<usize> = frames
            .iter()
            .map(|f| f.seam_filled)
            .filter(|n| *n > 0)
            .collect();
        assert_eq!(filled, vec![12]);
    }

    #[test]
    fn sampled_bins_keep_their_own_points() {
        let near = [Some(1.0); POINTS_PER_PACKET];
        let far = [Some(3.0); POINTS_PER_PACKET];
        for keep_straddle in [true, false] {
            let mut revolution = Revolution::new();
            let mut frames = Vec::new();
            // 12-deg packets from 6 deg; the straddle (354 -> 366 deg) of the
            // first turn reads 1 m, every other packet 3 m.
            for turn in 0..3 {
                for k in 0..30u32 {
                    if turn == 1 && k == 29 && !keep_straddle {
                        continue;
                    }
                    let start = k * 1200 + 600;
                    let ranges = if turn == 0 && k == 29 { &near } else { &far };
                    frames.extend(revolution.push(
                        start as u16,
                        ((start + 1200) % 36_000) as u16,
                        ranges,
                    ));
                }
            }
            assert_eq!(frames.len(), 1);
            let frame = &frames[0];
            if keep_straddle {
                assert_eq!(
                    (frame.seam_filled, frame.coverage, frame.ranges[3]),
                    (0, 360, Some(3.0))
                );
            } else {
                // Bins 0-5 from the first turn's straddle; 355-359 stay unknown.
                assert_eq!(
                    (frame.seam_filled, frame.coverage, frame.ranges[3]),
                    (6, 355, Some(1.0))
                );
                assert_eq!(frame.ranges[357], None);
            }
        }
    }

    #[test]
    fn nothing_is_published_before_the_first_complete_revolution() {
        let mut revolution = Revolution::new();
        let ranges = [Some(2.0); POINTS_PER_PACKET];
        assert!(revolution.push(30_000, 31_200, &ranges).is_none());
        assert!(revolution.push(100, 1_300, &ranges).is_none()); // first wrap: sync only
        for k in 1..30u16 {
            assert!(
                revolution
                    .push(100 + 1_200 * k, (100 + 1_200 * (k + 1)) % 36_000, &ranges)
                    .is_none()
            );
        }
        let first = revolution.push(500, 1_700, &ranges).unwrap();
        assert_eq!(
            (first.seam_filled, first.coverage, first.valid),
            (0, 360, 360)
        );
    }
}
