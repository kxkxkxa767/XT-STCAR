//! Crosswalk stop planned at rest (user, 2026-11-03): in the start's still frames the
//! stripes' near edge is ranged against the board straight ahead (the left-turn exit
//! board), whose lidar range fixes that frame's camera pitch. The plan is then held and
//! the camera left out (the body's pitching moved the far fixes); the stop runs on the
//! lidar track and the IMU speed from that rest. Never a start condition: without a
//! plan the camera fixes in motion place the crosswalk as before.
use crate::helpers::{FRONT_BODY_EXTENT_M, SCAN_AGE_S};
use crate::maneuver::{CROSSWALK_CAMERA_LAG_S, CROSSWALK_FIX_FRAMES, CROSSWALK_RANGE_M};
use crate::py::*;
use crate::dict;

// The camera mount (scripts/vision-log.py): the lens .195 m above the floor and .041 m
// behind the front, so .159 m ahead of the lidar.
const CAMERA_HEIGHT_M: f64 = 0.195;
const CAMERA_BEHIND_FRONT_M: f64 = 0.041;
const CAMERA_AHEAD_OF_LIDAR_M: f64 = FRONT_BODY_EXTENT_M - CAMERA_BEHIND_FRONT_M;
const BOARD_AHEAD_HALF_BINS: i64 = 3; // scripts/camera-pitch-check.py's rays
const STILL_TRACK_M: f64 = 0.02; // lidar range noise: the car has not left its rest
const FRAMES_KEPT: usize = 8;

#[derive(Clone, Debug, Default)]
pub struct CrosswalkStopPlan {
    /// (capture time less the camera lag, stripes' edge height, board base height)
    frames: Vec<(f64, f64, f64)>,
    /// the lidar's range to the board ahead on the still scans
    board_ahead: Vec<f64>,
    moved_at: Option<f64>,
    /// (front to the edge m, pitch rad, the edge on the crosswalk track)
    fixes: Vec<(f64, f64, f64)>,
    pub planned: Option<f64>,
    imu_zero: Option<f64>,
    last_scan_at: Option<f64>,
    pub scan_s: Option<f64>,
}

/// The front's distance to the stripes' edge and the frame's pitch, from the two image
/// heights and the lidar's range to the board ahead (its base fixes the pitch).
fn edge_from_board(edge_y: f64, base_y: f64, board_m: f64) -> Option<(f64, f64)> {
    let to_board = board_m - CAMERA_AHEAD_OF_LIDAR_M;
    if to_board <= 0. {
        return None;
    }
    let pitch = base_y.atan() - (CAMERA_HEIGHT_M / to_board).atan();
    let down = edge_y.atan() - pitch;
    if down <= 0. {
        return None;
    }
    let front = CAMERA_HEIGHT_M / down.tan() - CAMERA_BEHIND_FRONT_M;
    (CROSSWALK_RANGE_M.0..=CROSSWALK_RANGE_M.1).contains(&front).then_some((front, pitch))
}

fn median(values: &[f64]) -> f64 {
    let mut v = values.to_vec();
    sort_floats(&mut v);
    let n = v.len();
    if n % 2 == 1 { v[n / 2] } else { (v[n / 2 - 1] + v[n / 2]) / 2. }
}

/// The lidar's range straight ahead: the median of the centre rays (at least four).
fn board_ahead(scan: &V) -> Option<f64> {
    let list = scan.idx("ranges").l();
    if list.len() < 360 {
        return None;
    }
    let rays: Vec<f64> = (-BOARD_AHEAD_HALF_BINS..=BOARD_AHEAD_HALF_BINS)
        .filter_map(|i| list[i.rem_euclid(360) as usize].of())
        .collect();
    (rays.len() >= 4).then(|| median(&rays))
}

impl CrosswalkStopPlan {
    pub fn push_frame(&mut self, captured: f64, edge_y: f64, base_y: f64) {
        if self.planned.is_none() {
            self.frames.push((captured - CROSSWALK_CAMERA_LAG_S, edge_y, base_y));
            let n = self.frames.len();
            if n > FRAMES_KEPT {
                self.frames.drain(..n - FRAMES_KEPT);
            }
        }
    }

    /// Once per start-lane scan with the crosswalk track and its rest; the plan's
    /// evidence on the scan that makes it.
    pub fn observe(&mut self, scan: &V, received: f64, track: Option<f64>, rest: Option<(f64, f64)>,
                   forward_integral: Option<f64>) -> Option<V> {
        if let Some(last) = self.last_scan_at {
            let dt = received - last;
            if 0. < dt && dt <= SCAN_AGE_S {
                self.scan_s = Some(dt);
            }
        }
        self.last_scan_at = Some(received);
        let (_, rest_track) = rest?;
        match track {
            _ if self.moved_at.is_some() => {}
            Some(t) if (t - rest_track).abs() <= STILL_TRACK_M => {
                if let Some(b) = board_ahead(scan) {
                    self.board_ahead.push(b);
                }
                if self.imu_zero.is_none() {
                    self.imu_zero = forward_integral;
                }
            }
            Some(_) => self.moved_at = Some(received),
            None => {} // no track this scan: neither still nor moved
        }
        if self.planned.is_some() || self.board_ahead.is_empty() {
            return None;
        }
        let board_m = median(&self.board_ahead);
        let still_until = self.moved_at.unwrap_or(f64::INFINITY);
        for (at, edge_y, base_y) in std::mem::take(&mut self.frames) {
            if at < still_until && let Some((front, pitch)) = edge_from_board(edge_y, base_y, board_m) {
                self.fixes.push((front, pitch, rest_track - FRONT_BODY_EXTENT_M - front));
            }
        }
        if self.fixes.len() < CROSSWALK_FIX_FRAMES {
            return None;
        }
        let edge = median(&self.fixes.iter().map(|f| f.2).collect::<Vec<_>>());
        self.planned = Some(edge);
        let front = median(&self.fixes.iter().map(|f| f.0).collect::<Vec<_>>());
        Some(dict! {
            "source_seq" => scan.idx("seq").clone(), "received_at" => received,
            "basis" => "still_frames_against_the_board_ahead", "board_ahead_m" => board_m,
            "front_to_edge_m" => front, "edge_from_board_ahead_m" => board_m - FRONT_BODY_EXTENT_M - front,
            "board_m" => edge, "rest_track_m" => rest_track,
            "fixes" => V::list(self.fixes.iter().map(|f| V::from(vec![f.0, f.1.to_degrees(), f.2])).collect()),
        })
    }

    /// The IMU's forward speed since the start's rest.
    pub fn imu_speed(&self, forward_integral: Option<f64>) -> Option<f64> {
        self.imu_zero.zip(forward_integral).map(|(zero, now)| (now - zero).max(0.))
    }
}
