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
// y53/y54: the IMU already measured the launch rise. Only the short ESC
// response still ahead of the current sample belongs in its cut-speed estimate.
pub(crate) const IMU_POWER_RESPONSE_S: f64 = 0.1;

#[derive(Clone, Debug, Default)]
pub struct CrosswalkStopPlan {
    /// YOLO: (capture time less the camera lag, stripes' edge height, board base height)
    frames: Vec<(f64, f64, f64)>,
    /// Pure-Rust: (capture time less lag, front-to-edge m, pitch rad, gap-to-edge m, pitch source)
    direct_frames: Vec<(f64, f64, f64, Option<f64>, &'static str)>,
    /// the lidar's range to the board ahead on the still scans
    board_ahead: Vec<f64>,
    moved_at: Option<f64>,
    /// (front to the edge m, pitch rad, the edge on the crosswalk track)
    fixes: Vec<(f64, f64, f64)>,
    gap_distances: Vec<f64>,
    yolo_fixes: usize,
    rust_fixes: usize,
    pub planned: Option<f64>,
    imu_zero: Option<f64>,
    last_scan_at: Option<f64>,
    pub scan_s: Option<f64>,
    last_integral: Option<f64>,
    map_pair: Option<(f64, f64)>,
    map_origin: bool,
    front_to_rear_offset: Option<f64>,
    /// y68: the IMU forward acceleration over the last scan interval.
    pub imu_accel: Option<f64>,
}

/// The front's distance to the stripes' edge and the frame's pitch, from the two image
/// heights and the lidar's range to the board ahead (its base fixes the pitch).
pub(crate) fn edge_from_board(edge_y: f64, base_y: f64, board_m: f64) -> Option<(f64, f64)> {
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
pub(crate) fn board_ahead(scan: &V) -> Option<f64> {
    let list = scan.get("ranges").ol()?;
    if list.len() < 360 {
        return None;
    }
    let rays: Vec<f64> = (-BOARD_AHEAD_HALF_BINS..=BOARD_AHEAD_HALF_BINS)
        .filter_map(|i| list[i.rem_euclid(360) as usize].of())
        .collect();
    (rays.len() >= 4).then(|| median(&rays))
}

impl CrosswalkStopPlan {
    /// Keep one decreasing longitudinal coordinate throughout this optical stop.
    /// A paired real measurement fixes the offset; its occasional return never
    /// switches a map-origin plan into the unrelated front-board coordinate.
    pub fn aligned_track(
        &mut self, raw: Option<f64>, field_x: Option<f64>, at_first_fix: bool, moving_fix: bool,
    ) -> Option<f64> {
        if let (Some(track), Some(x)) = (raw, field_x) {
            self.map_pair = Some((track, x));
        }
        // observe() consumes queued frames. Their accepted fixes still use
        // this odometry reference between camera captures and during a dropout.
        let optical = self.has_optical_observation(moving_fix);
        if at_first_fix && raw.is_none() && field_x.is_some() && optical {
            self.map_origin = true;
        }
        if self.map_origin {
            return field_x.map(|x| -x).or_else(|| {
                let (paired, x) = self.map_pair?;
                Some(-x + raw? - paired)
            });
        }
        raw.or_else(|| {
            if !optical { return None; }
            let (paired, x) = self.map_pair?;
            Some(paired - (field_x? - x))
        })
    }

    pub fn map_origin(&self) -> bool { self.map_origin }

    pub fn has_optical_observation(&self, moving_fix: bool) -> bool {
        moving_fix || self.planned.is_some() || !self.fixes.is_empty()
            || !self.direct_frames.is_empty() || !self.frames.is_empty()
    }

    /// Reuse an offset only within one front-board track. The lane speed
    /// history already resets when samples are lost or their range jumps.
    pub fn front_on_rear_track(&mut self, rear: Option<f64>, front: Option<f64>, continuous: bool) -> Option<f64> {
        self.front_to_rear_offset = match (rear, front) {
            (Some(rear), Some(front)) => Some(rear - front),
            (None, Some(_)) if continuous => self.front_to_rear_offset,
            _ => None,
        };
        front.zip(self.front_to_rear_offset).map(|(range, offset)| range + offset)
    }

    pub fn change_unplanned_reference(&mut self) {
        self.map_pair = None;
        self.map_origin = false;
        self.front_to_rear_offset = None;
    }

    /// Only called before a plan exists, when the old and new tracks cannot
    /// be related. Keep the IMU rest and the time movement actually began.
    pub fn discard_unpaired_observations(&mut self) {
        self.frames.clear();
        self.direct_frames.clear();
        self.board_ahead.clear();
        self.fixes.clear();
        self.gap_distances.clear();
        self.yolo_fixes = 0;
        self.rust_fixes = 0;
    }
    pub fn push_frame(&mut self, captured: f64, edge_y: f64, base_y: f64) {
        if self.planned.is_none() {
            self.frames.push((captured - CROSSWALK_CAMERA_LAG_S, edge_y, base_y));
            let n = self.frames.len();
            if n > FRAMES_KEPT {
                self.frames.drain(..n - FRAMES_KEPT);
            }
        }
    }

    /// Direct stationary distance fix from pure-Rust crosswalk vision (using left-gap / board pitch).
    pub fn push_direct_fix(
        &mut self,
        captured: f64,
        front_to_edge_m: f64,
        pitch_rad: f64,
        gap_to_edge_m: Option<f64>,
        source: &'static str,
    ) {
        if self.planned.is_none() && (CROSSWALK_RANGE_M.0..=CROSSWALK_RANGE_M.1).contains(&front_to_edge_m) {
            // Pure-Rust runs right after capture (no ~230 ms YOLO delay); subtract a short capture lag.
            self.direct_frames.push((captured - 0.05, front_to_edge_m, pitch_rad, gap_to_edge_m, source));
            let n = self.direct_frames.len();
            if n > FRAMES_KEPT {
                self.direct_frames.drain(..n - FRAMES_KEPT);
            }
        }
    }

    /// Once per start-lane scan with the crosswalk track and its rest; the plan's
    /// evidence on the scan that makes it.
    pub fn observe(&mut self, scan: &V, received: f64, track: Option<f64>, rest: Option<(f64, f64)>,
                   forward_integral: Option<f64>) -> Option<V> {
        self.imu_accel = None;
        if let Some(last) = self.last_scan_at {
            let dt = received - last;
            if 0. < dt && dt <= SCAN_AGE_S {
                self.scan_s = Some(dt);
                self.imu_accel = self.last_integral.zip(forward_integral).map(|(before, now)| (now - before) / dt);
            }
        }
        self.last_scan_at = Some(received);
        self.last_integral = forward_integral;
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
        if self.planned.is_some() {
            return None;
        }
        let still_until = self.moved_at.unwrap_or(f64::INFINITY);
        let board_m = (!self.board_ahead.is_empty()).then(|| median(&self.board_ahead));
        if let Some(bm) = board_m {
            for (at, edge_y, base_y) in std::mem::take(&mut self.frames) {
                if at < still_until && let Some((front, pitch)) = edge_from_board(edge_y, base_y, bm) {
                    self.fixes.push((front, pitch, rest_track - FRONT_BODY_EXTENT_M - front));
                    self.yolo_fixes += 1;
                }
            }
        }
        for (at, front, pitch, gap_m, _src) in std::mem::take(&mut self.direct_frames) {
            if at < still_until {
                self.fixes.push((front, pitch, rest_track - FRONT_BODY_EXTENT_M - front));
                if let Some(g) = gap_m {
                    self.gap_distances.push(g);
                }
                self.rust_fixes += 1;
            }
        }
        if self.fixes.len() < CROSSWALK_FIX_FRAMES {
            return None;
        }
        let edge = median(&self.fixes.iter().map(|f| f.2).collect::<Vec<_>>());
        self.planned = Some(edge);
        let front = median(&self.fixes.iter().map(|f| f.0).collect::<Vec<_>>());
        let basis = match (self.yolo_fixes > 0, self.rust_fixes > 0) {
            (true, true) => "yolo_and_rust_still_fusion",
            (false, true) => "rust_stripes_with_left_gap",
            _ => "still_frames_against_the_board_ahead",
        };
        let mut evidence = dict! {
            "source_seq" => scan.idx("seq").clone(), "received_at" => received,
            "basis" => basis,
            "front_to_edge_m" => front,
            "board_m" => edge, "rest_track_m" => rest_track,
            "yolo_fixes" => self.yolo_fixes as i64, "rust_fixes" => self.rust_fixes as i64,
            "fixes" => V::list(self.fixes.iter().map(|f| V::from(vec![f.0, f.1.to_degrees(), f.2])).collect()),
        };
        if let Some(bm) = board_m {
            evidence.set("board_ahead_m", bm);
            evidence.set("edge_from_board_ahead_m", bm - FRONT_BODY_EXTENT_M - front);
        }
        if !self.gap_distances.is_empty() {
            evidence.set("gap_to_crosswalk_m", median(&self.gap_distances));
        }
        Some(evidence)
    }

    /// The IMU's forward speed since the start's rest.
    pub fn imu_speed(&self, forward_integral: Option<f64>) -> Option<f64> {
        self.imu_zero.zip(forward_integral).map(|(zero, now)| (now - zero).max(0.))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn missing_central_rays_preserve_the_coordinates_of_the_optical_fix() {
        let mut p = CrosswalkStopPlan::default();
        assert_eq!(p.aligned_track(Some(-0.42), Some(0.38), true, false), Some(-0.42));
        assert_eq!(p.aligned_track(None, Some(0.50), false, false), None);
        p.planned=Some(-2.6);
        assert!((p.aligned_track(None, Some(0.58), false, false).unwrap()+0.62).abs()<1e-9);
        assert_eq!(p.aligned_track(None,None,false, false),None);
        assert_eq!(p.aligned_track(Some(-0.76), Some(0.70), false, false), Some(-0.76));
        let mut from_map = CrosswalkStopPlan::default();
        from_map.push_direct_fix(0.1,2.10,0.14,None,"rust");
        assert_eq!(from_map.aligned_track(None,Some(0.40),true, false),Some(-0.40));
        assert_eq!(from_map.aligned_track(Some(7.5),Some(0.50),false, false),Some(-0.50));
        assert!((from_map.aligned_track(Some(7.40),None,false, false).unwrap()+0.60).abs()<1e-9);
    }

    #[test]
    fn consumed_stripes_keep_their_reference_between_camera_frames() {
        let mut plan = CrosswalkStopPlan::default();
        let scan = dict! { "seq" => 1, "ranges" => vec![V::None; 360] };
        plan.aligned_track(Some(-0.4), Some(0.4), true, false);
        plan.push_direct_fix(0.1, 2.1, 0.14, None, "rust");
        plan.observe(&scan, 0.2, Some(-0.4), Some((0., -0.4)), Some(0.));
        assert!(plan.direct_frames.is_empty() && plan.planned.is_none());
        assert_eq!(plan.aligned_track(None, Some(0.5), false, false), Some(-0.5));
        plan.discard_unpaired_observations();
        assert_eq!(plan.aligned_track(None, Some(0.5), false, false), None);
        assert_eq!(plan.aligned_track(None, Some(0.5), false, true), Some(-0.5));
    }

    #[test]
    fn front_board_only_continues_a_paired_unbroken_track() {
        let mut plan = CrosswalkStopPlan::default();
        assert_eq!(plan.front_on_rear_track(None, Some(7.0), false), None);
        assert_eq!(plan.front_on_rear_track(Some(-0.5), Some(7.5), false), Some(-0.5));
        assert_eq!(plan.front_on_rear_track(None, Some(7.0), true), Some(-1.0));
        assert_eq!(plan.front_on_rear_track(None, Some(3.0), false), None);
        assert_eq!(plan.front_on_rear_track(Some(-1.5), Some(6.5), true), Some(-1.5));
        assert_eq!(plan.front_on_rear_track(None, None, false), None);
        assert_eq!(plan.front_on_rear_track(None, Some(6.0), true), None);
    }

    #[test]
    fn no_visual_fixes_never_invents_a_fixed_crosswalk() {
        assert!(board_ahead(&V::None).is_none());
        assert!(board_ahead(&V::dict()).is_none());
        let scan = dict! {"seq" => 1, "ranges" => vec![V::None; 360]};
        let mut plan = CrosswalkStopPlan::default();
        assert!(plan.observe(&scan, 0., Some(-0.3), Some((0., -0.3)), Some(0.)).is_none());
        assert!(plan.planned.is_none());
    }

    #[test]
    fn pure_rust_still_fixes_plan_crosswalk_without_yolo_or_far_board() {
        let scan = dict! {"seq" => 1, "ranges" => vec![V::None; 360]};
        let mut plan = CrosswalkStopPlan::default();
        for (i, d) in [(0.1, 2.18), (0.3, 2.20), (0.5, 2.19)] {
            plan.push_direct_fix(i, d, 8.0_f64.to_radians(), Some(0.22), "left_gap");
        }
        let ev = plan
            .observe(&scan, 0.6, Some(-0.40), Some((0.0, -0.40)), Some(0.0))
            .expect("planned from rust");
        assert_eq!(ev.get("basis").os(), Some("rust_stripes_with_left_gap"));
        assert!((ev.get("front_to_edge_m").f() - 2.19).abs() < 1e-6);
        assert!((ev.get("gap_to_crosswalk_m").f() - 0.22).abs() < 1e-6);
        assert!((plan.planned.unwrap() - (-0.40 - FRONT_BODY_EXTENT_M - 2.19)).abs() < 1e-6);
    }
}
