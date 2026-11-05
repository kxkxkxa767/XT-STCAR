//! Recent radar/IMU body speed for the surface-load learner only.
//!
//! Stage controllers keep their own measurements. Radar supplies an absolute
//! velocity anchor; only the short IMU change since that anchor is integrated.
//! The learner must still receive the original IMU integral when computing
//! acceleration: changing a radar anchor is not physical acceleration.

use crate::dict;
use crate::helpers::SCAN_AGE_S;
use crate::py::V;

const HISTORY_LEN: usize = 8;
// The same straight-motion criterion used by transfer/load learning.
const MAX_YAW_RADPS: f64 = 0.6;
// Condition the geometric conversion, not the driving speed. A nearly
// perpendicular wall-axis velocity cannot identify forward body velocity.
const MIN_AXIS_COS: f64 = std::f64::consts::FRAC_1_SQRT_2;

/// An already-valid, five-point lane-axis speed. Times are host receive times,
/// not the lidar's independent publication clock. The axis points in the
/// positive travel direction: 0 for the start straight, PI for the return.
#[derive(Clone, Copy, Debug)]
pub struct AxisSpeed {
    pub speed_mps: f64,
    pub window_start_at: f64,
    pub window_end_at: f64,
    pub axis_heading_rad: f64,
}

#[derive(Clone, Copy, Debug)]
struct Sample {
    at: f64,
    heading: Option<f64>,
    integral: Option<f64>,
    yaw: Option<f64>,
}

#[derive(Clone, Copy, Debug)]
struct Anchor {
    at: f64,
    received_at: f64,
    body_speed: f64,
    integral: f64,
    axis_heading: f64,
    projection_cos: f64,
}

#[derive(Clone, Debug, Default)]
pub struct SurfaceSpeed {
    samples: [Option<Sample>; HISTORY_LEN],
    len: usize,
    last_seq: Option<i64>,
    anchor: Option<Anchor>,
    current_speed: Option<f64>,
    reanchored: bool,
    reason: Option<&'static str>,
}

fn finite(value: Option<f64>) -> Option<f64> {
    value.filter(|v| v.is_finite())
}

fn angle_delta(from: f64, to: f64) -> f64 {
    (to - from).sin().atan2((to - from).cos())
}

impl SurfaceSpeed {
    /// Call once per lidar sequence, including unavailable radar observations,
    /// so a turn or a stale source cannot silently extend an old IMU anchor.
    #[allow(clippy::too_many_arguments)]
    pub fn observe(
        &mut self,
        seq: i64,
        now: f64,
        axis: Option<AxisSpeed>,
        heading: Option<f64>,
        integral: Option<f64>,
        yaw: Option<f64>,
    ) -> Option<f64> {
        if seq < 0 || !now.is_finite() {
            return None;
        }
        if self.last_seq.is_some_and(|last| seq <= last) {
            return self.speed(now, integral);
        }
        self.last_seq = Some(seq);
        self.reanchored = false;
        self.current_speed = None;
        let sample = Sample { at: now, heading: finite(heading), integral: finite(integral), yaw: finite(yaw) };
        if self.len > 0 && self.samples[self.len - 1].is_some_and(|last| {
            !(0. < now - last.at && now - last.at <= SCAN_AGE_S)
        }) {
            self.samples = [None; HISTORY_LEN];
            self.len = 0;
            self.anchor = None;
        }
        if self.len == HISTORY_LEN {
            self.samples.copy_within(1.., 0);
            self.len -= 1;
        }
        self.samples[self.len] = Some(sample);
        self.len += 1;
        if sample.integral.is_none() || sample.heading.is_none() || sample.yaw.is_none() {
            self.anchor = None;
            self.reason = Some("imu_or_heading_missing");
            return None;
        }
        if sample.yaw.unwrap().abs() > MAX_YAW_RADPS {
            self.anchor = None;
            self.reason = Some("turning");
            return None;
        }

        self.reason = Some("axis_unavailable");
        if let Some(axis) = axis {
            self.reason = Some("axis_window_unaligned");
            if let Some(anchor) = self.align_axis(axis, now)
                && self.anchor.is_none_or(|old| anchor.received_at > old.received_at) {
                self.anchor = Some(anchor);
                self.reanchored = true;
            }
        }
        self.current_speed = self.speed(now, integral);
        if self.current_speed.is_some() {
            self.reason = Some(if self.reanchored { "radar_imu_aligned" } else { "imu_short_hold" });
        }
        self.current_speed
    }

    fn interpolate(&self, at: f64) -> Option<Sample> {
        for pair in self.samples[..self.len].windows(2) {
            let (before, after) = (pair[0]?, pair[1]?);
            if before.at <= at && at <= after.at {
                let fraction = (at - before.at) / (after.at - before.at);
                let heading = before.heading? + fraction * angle_delta(before.heading?, after.heading?);
                let integral = before.integral? + fraction * (after.integral? - before.integral?);
                let yaw = before.yaw? + fraction * (after.yaw? - before.yaw?);
                return Some(Sample { at, heading: Some(heading), integral: Some(integral), yaw: Some(yaw) });
            }
        }
        None
    }

    fn align_axis(&self, axis: AxisSpeed, now: f64) -> Option<Anchor> {
        if !axis.speed_mps.is_finite() || axis.speed_mps < 0.
            || !axis.window_start_at.is_finite() || !axis.window_end_at.is_finite()
            || !axis.axis_heading_rad.is_finite() || axis.window_start_at >= axis.window_end_at
            || !(0. ..=SCAN_AGE_S).contains(&(now - axis.window_end_at))
        {
            return None;
        }
        // pairwise_track_speed uses a median of the last five scans' slopes.
        // Use the window midpoint as its representative time, rather than the
        // newest scan (an approximation for uneven/accelerating samples).
        // Low yaw throughout that window keeps the axis/body conversion valid.
        let start = self.interpolate(axis.window_start_at)?;
        let end = self.interpolate(axis.window_end_at)?;
        let middle = self.interpolate((axis.window_start_at + axis.window_end_at) * 0.5)?;
        for sample in [start, end].into_iter().chain(self.samples[..self.len].iter().flatten().copied()
            .filter(|s| axis.window_start_at <= s.at && s.at <= now))
        {
            if sample.yaw?.abs() > MAX_YAW_RADPS
                || (sample.heading? - axis.axis_heading_rad).cos() < MIN_AXIS_COS
            {
                return None;
            }
        }
        let projection_cos = (middle.heading? - axis.axis_heading_rad).cos();
        if projection_cos < MIN_AXIS_COS {
            return None;
        }
        Some(Anchor {
            at: middle.at,
            received_at: axis.window_end_at,
            body_speed: axis.speed_mps / projection_cos,
            integral: middle.integral?,
            axis_heading: axis.axis_heading_rad,
            projection_cos,
        })
    }

    /// The same source may propagate only briefly beyond its latest radar scan.
    /// No long integration through a loop is a fallback for missing geometry.
    pub fn speed(&self, now: f64, integral: Option<f64>) -> Option<f64> {
        let anchor = self.anchor?;
        let latest = self.samples.get(self.len.checked_sub(1)?)?.as_ref()?;
        if !now.is_finite() || !(0. ..=SCAN_AGE_S).contains(&(now - anchor.received_at))
            || !(0. ..=SCAN_AGE_S).contains(&(now - latest.at))
            || latest.yaw?.abs() > MAX_YAW_RADPS
            || (latest.heading? - anchor.axis_heading).cos() < MIN_AXIS_COS
        {
            return None;
        }
        let speed = anchor.body_speed + finite(integral)? - anchor.integral;
        speed.is_finite().then_some(speed.max(0.))
    }

    pub fn diagnostics(&self) -> V {
        dict! {
            "body_speed_mps" => self.current_speed,
            "source_seq" => self.last_seq,
            "anchor_at" => self.anchor.map(|a| a.at),
            "radar_received_at" => self.anchor.map(|a| a.received_at),
            "anchor_body_speed_mps" => self.anchor.map(|a| a.body_speed),
            "projection_cos" => self.anchor.map(|a| a.projection_cos),
            "reanchored" => self.reanchored,
            "reason" => self.reason,
            "basis" => "five_scan_axis_midpoint_plus_short_imu_change"
        }
    }
}
