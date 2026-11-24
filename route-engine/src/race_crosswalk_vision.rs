//! Pure-Rust crosswalk detector with lidar left-gap & wall alignment.
//!
//! Reads the Rust V4L2 camera's atomic BGR snapshots while the car is at the start
//! or approaching the stripes. The worker requests CPU 6 and stops once the
//! crosswalk distance is resolved or the start-lane stage ends.
//! - Uses the lidar's lane walls and left divider-wall end ("left gap") to align a compact
//!   metric BEV grid inside the start lane and calibrate camera pitch (robust even on bright
//!   sky-blue basketball courts where simple B-R far-board detection merges with the floor).
//! - Matches the competition half-A4 crosswalk pattern (10.5 cm wide x 29.7 cm deep, 10.5 cm
//!   gaps -> 21 cm lateral pitch, >= 3 stripes, >= 2 clean gaps, sharp near/far truncation)
//!   so continuous 5 cm white basketball-court lines (horizontal, vertical, or curved) are
//!   rejected with zero false positives.

use std::path::{Path, PathBuf};
#[cfg(target_os = "linux")]
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, mpsc::Sender};
use std::thread;
use std::time::{Duration, Instant};

use crate::helpers::FRONT_BODY_EXTENT_M;
use crate::py::V;
use crate::race::{Event, spawn_named};

// Camera calibration matching config/camera-race4-20261031.json
const FX: f64 = 426.55;
const FY: f64 = 424.47;
const CX: f64 = 349.59;
const CY: f64 = 227.85;
const K1: f64 = -0.371764;
const K2: f64 = 0.172259;
const P1: f64 = -0.000092;
const P2: f64 = -0.000226;
const K3: f64 = -0.042575;
const CAM_H: f64 = 0.195;
const CAM_BEHIND_FRONT: f64 = 0.041;
const CAM_AHEAD_LIDAR: f64 = FRONT_BODY_EXTENT_M - CAM_BEHIND_FRONT; // 0.159 m
const NOMINAL_PITCH_DEG: f64 = 8.05;
const SHM_MAGIC: &[u8; 4] = b"XTF1";
pub(crate) const SHM_HEADER_LEN: usize = 32;

/// Compact lidar geometry snapshot of the start lane, updated by the control thread.
#[derive(Clone, Copy, Debug)]
pub struct LaneLidarHint {
    pub has_scan: bool,
    pub heading_left_rad: f64,
    pub y_left_m: f64,
    pub y_right_m: f64,
    pub left_gap_x_m: Option<f64>,
    pub left_gap_y_m: f64,
    pub front_board_m: Option<f64>,
}

impl Default for LaneLidarHint {
    fn default() -> Self {
        Self {
            has_scan: false,
            heading_left_rad: 0.0,
            y_left_m: 0.45,
            y_right_m: -0.45,
            left_gap_x_m: None,
            left_gap_y_m: 0.45,
            front_board_m: None,
        }
    }
}

/// One pure-Rust crosswalk observation sent to the control thread.
#[derive(Clone, Debug)]
pub struct RustCrosswalkObs {
    pub frame: i64,
    pub captured_mono_s: f64,
    pub front_to_edge_m: f64,
    pub edge_y: f64,
    pub pitch_rad: f64,
    pub pitch_source: &'static str,
    pub score: f64,
    pub stripes: usize,
    pub gaps: usize,
    pub left_gap_x_m: Option<f64>,
    pub gap_to_crosswalk_m: Option<f64>,
    pub compute_ms: f64,
}

/// Summary telemetry for summary.json.
#[derive(Clone, Debug, Default)]
pub struct RustCrosswalkStats {
    pub frames_read: i64,
    pub detections: i64,
    pub compute_ms: Vec<f64>,
    pub pinned_cpu: Option<usize>,
}

fn median_f64(vals: &mut [f64]) -> f64 {
    vals.sort_by(|a, b| a.total_cmp(b));
    let n = vals.len();
    if n % 2 == 1 { vals[n / 2] } else { 0.5 * (vals[n / 2 - 1] + vals[n / 2]) }
}

fn fit_wall_line(pts: &[(f64, f64)]) -> Option<(f64, f64)> {
    if pts.len() < 12 {
        return None;
    }
    let mut slopes = Vec::with_capacity(64);
    let n = pts.len();
    let mut i = 0;
    while i < n {
        let (x1, y1) = pts[i];
        let mut j = i + 4;
        while j < n {
            let (x2, y2) = pts[j];
            if (x2 - x1).abs() >= 0.25 {
                slopes.push((y2 - y1) / (x2 - x1));
            }
            j += 4;
        }
        i += 2;
    }
    if slopes.len() < 5 {
        return None;
    }
    let slope = median_f64(&mut slopes);
    let mut intercepts: Vec<f64> = pts.iter().map(|&(x, y)| y - slope * x).collect();
    let intercept = median_f64(&mut intercepts);
    let inliers = pts.iter().filter(|&&(x, y)| (y - (slope * x + intercept)).abs() <= 0.04).count();
    (inliers >= (n * 65 / 100).max(10)).then_some((slope, intercept))
}

impl LaneLidarHint {
    /// Extract lane heading, lane walls, and left exit-gap end from a 360-bin lidar scan.
    pub fn from_scan(scan: &V) -> Option<Self> {
        let ranges = scan.get("ranges").ol()?;
        if ranges.len() != 360 {
            return None;
        }
        let mut l_pts = Vec::with_capacity(90);
        let mut r_pts = Vec::with_capacity(90);
        for (i, rv) in ranges.iter().enumerate() {
            let Some(r) = rv.of() else { continue };
            if !(0.20..=2.40).contains(&r) {
                continue;
            }
            if (260..345).contains(&i) {
                let ang = ((360 - i) as f64).to_radians();
                let (x, y) = (r * ang.cos(), r * ang.sin());
                if (-0.30..=2.20).contains(&x) && (0.20..=1.20).contains(&y) {
                    l_pts.push((x, y));
                }
            } else if (15..100).contains(&i) {
                let ang = (i as f64).to_radians();
                let (x, y) = (r * ang.cos(), -r * ang.sin());
                if (-0.30..=2.20).contains(&x) && (-1.20..=-0.20).contains(&y) {
                    r_pts.push((x, y));
                }
            }
        }
        let mut slope_sum = 0.0;
        let mut slope_n = 0;
        if let Some((s, _)) = fit_wall_line(&l_pts) {
            slope_sum += s;
            slope_n += 1;
        }
        if let Some((s, _)) = fit_wall_line(&r_pts) {
            slope_sum += s;
            slope_n += 1;
        }
        let mut heading_left = if slope_n > 0 { -(slope_sum / slope_n as f64).atan() } else { 0.0 };
        if heading_left.abs() > 18.0_f64.to_radians() {
            heading_left = 0.0;
        }
        let (ch, sh) = (heading_left.cos(), heading_left.sin());
        let to_lane = |xb: f64, yb: f64| (xb * ch - yb * sh, xb * sh + yb * ch);

        let mut ly: Vec<f64> = l_pts.iter().map(|&(x, y)| to_lane(x, y).1).collect();
        let mut ry: Vec<f64> = r_pts.iter().map(|&(x, y)| to_lane(x, y).1).collect();
        let y_left = if ly.len() >= 10 { median_f64(&mut ly) } else { 0.45 };
        let y_right = if ry.len() >= 10 { median_f64(&mut ry) } else { -0.45 };

        // Walk clockwise along left divider wall (bins 280..359) to find the exact exit gap.
        let mut left_gap_x = None;
        let mut left_gap_y = y_left;
        let mut last: Option<(f64, f64, usize)> = None;
        for (i, rv) in ranges.iter().enumerate().take(359).skip(280) {
            let Some(r) = rv.of() else { continue };
            let ang = ((360 - i) as f64).to_radians();
            let (xl, yl) = to_lane(r * ang.cos(), r * ang.sin());
            let ref_y = last.map_or(y_left, |(_, y, _)| y);
            let on_wall = xl > 0.30
                && (0.20..=y_left + 0.25).contains(&yl)
                && (yl - ref_y).abs() <= 0.12
                && last.is_none_or(|(lx, _, _)| xl <= lx + 0.65);
            if on_wall {
                last = Some((xl, yl, i));
            } else if let Some((lx, ly_end, li)) = last
                && lx >= 1.20
                && i.saturating_sub(li) <= 4
                && (xl > lx + 0.80 || yl > ref_y + 0.30)
            {
                left_gap_x = Some(lx);
                left_gap_y = ly_end;
                break;
            }
        }

        let h_deg = heading_left.to_degrees().round() as i64;
        let mut fronts = Vec::with_capacity(7);
        for di in -3..=3_i64 {
            let idx = (h_deg + di).rem_euclid(360) as usize;
            if let Some(r) = ranges[idx].of() {
                fronts.push(r * (di as f64).to_radians().cos());
            }
        }

        Some(Self {
            has_scan: true,
            heading_left_rad: heading_left,
            y_left_m: y_left,
            y_right_m: y_right,
            left_gap_x_m: left_gap_x,
            left_gap_y_m: left_gap_y,
            front_board_m: (fronts.len() >= 4).then(|| median_f64(&mut fronts)),
        })
    }
}

fn lane_to_cam(xl: f64, yl: f64, heading_left: f64) -> (f64, f64) {
    let (ch, sh) = (heading_left.cos(), heading_left.sin());
    let xb = xl * ch + yl * sh;
    let yb = -xl * sh + yl * ch;
    (xb - CAM_AHEAD_LIDAR, yb)
}

/// Forward project ground point (xc forward from camera, yc left of camera) to distorted pixel (u, v).
/// Camera optical axis is tilted UP by +pitch_rad above horizontal.
fn project_cam_ground(xc: f64, yc: f64, pitch_rad: f64) -> Option<(f64, f64)> {
    if xc <= 0.15 {
        return None;
    }
    let (cp, sp) = (pitch_rad.cos(), pitch_rad.sin());
    let z_rot = xc * cp - CAM_H * sp;
    let y_rot = xc * sp + CAM_H * cp;
    if z_rot <= 0.05 {
        return None;
    }
    let xn = -yc / z_rot;
    let yn = y_rot / z_rot;
    let r2 = xn * xn + yn * yn;
    if r2 > 2.2 {
        return None;
    }
    let rad = 1.0 + r2 * (K1 + r2 * (K2 + r2 * K3));
    let xd = xn * rad + 2.0 * P1 * xn * yn + P2 * (r2 + 2.0 * xn * xn);
    let yd = yn * rad + P1 * (r2 + 2.0 * yn * yn) + 2.0 * P2 * xn * yn;
    Some((FX * xd + CX, FY * yd + CY))
}

fn undistort_pt(u: f64, v: f64) -> Option<(f64, f64)> {
    let (xd, yd) = ((u - CX) / FX, (v - CY) / FY);
    let (mut x, mut y) = (xd, yd);
    for _ in 0..25 {
        let r2 = x * x + y * y;
        let rad = 1.0 + r2 * (K1 + r2 * (K2 + r2 * K3));
        if rad.abs() < 1e-6 {
            return None;
        }
        x = (xd - 2.0 * P1 * x * y - P2 * (r2 + 2.0 * x * x)) / rad;
        y = (yd - P1 * (r2 + 2.0 * y * y) - 2.0 * P2 * x * y) / rad;
    }
    let r2 = x * x + y * y;
    let rad = 1.0 + r2 * (K1 + r2 * (K2 + r2 * K3));
    let eu = (FX * (x * rad + 2.0 * P1 * x * y + P2 * (r2 + 2.0 * x * x)) + CX - u).abs();
    let ev = (FY * (y * rad + P1 * (r2 + 2.0 * y * y) + 2.0 * P2 * x * y) + CY - v).abs();
    (eu <= 1e-2 && ev <= 1e-2).then_some((x, y))
}

#[inline]
fn bgr_at(bgr: &[u8], w: usize, u: usize, v: usize) -> (f64, f64, f64) {
    let idx = (v * w + u) * 3;
    (bgr[idx] as f64, bgr[idx + 1] as f64, bgr[idx + 2] as f64)
}

/// Calibrate camera pitch from the left divider-wall bottom corner (works on red, blue, grass, tile).
fn estimate_pitch_from_left_gap(bgr: &[u8], w: usize, h: usize, hint: &LaneLidarHint) -> Option<f64> {
    let gx = hint.left_gap_x_m?;
    if !(0.70..=4.20).contains(&gx) {
        return None;
    }
    let (xc, yc) = lane_to_cam(gx, hint.left_gap_y_m, hint.heading_left_rad);
    if xc <= 0.45 {
        return None;
    }
    let (u0, v0) = project_cam_ground(xc, yc, NOMINAL_PITCH_DEG.to_radians())?;
    if !(24.0..=(w as f64 - 24.0)).contains(&u0) || !(180.0..=(h as f64 - 30.0)).contains(&v0) {
        return None;
    }
    let y_top = (v0 as isize - 90).max(20) as usize;
    let y_bot = (v0 as isize - 18).max(y_top as isize + 15).min(h as isize - 1) as usize;
    let u_lo = (u0 as isize - 42).max(14) as usize;
    let u_hi = (u0 as isize + 42).min(w as isize - 14) as usize;
    if y_bot <= y_top || u_hi <= u_lo {
        return None;
    }

    let inv_rows = 1.0 / (y_bot - y_top) as f64;
    let mut col_blue = vec![0.0_f64; w];
    for (u, slot) in col_blue
        .iter_mut()
        .enumerate()
        .take((u_hi + 10).min(w - 1) + 1)
        .skip(u_lo - 10)
    {
        let mut s = 0.0;
        for v in y_top..y_bot {
            let (b, _, r) = bgr_at(bgr, w, u, v);
            s += b - r;
        }
        *slot = s * inv_rows;
    }
    let mut best_u = None;
    let mut best_drop = 25.0;
    for u in u_lo..=u_hi {
        let lb: f64 = col_blue[u - 10..u - 2].iter().sum::<f64>() / 8.0;
        let rb: f64 = col_blue[u + 2..u + 10].iter().sum::<f64>() / 8.0;
        if lb > 35.0 && (lb - rb) > best_drop {
            best_drop = lb - rb;
            best_u = Some(u);
        }
    }
    let best_u = best_u?;

    // Sample right inside the barrier end (best_u - 6 .. best_u - 1) so the column is at the gap's x.
    let c_lo = best_u.saturating_sub(6);
    let c_hi = best_u.saturating_sub(1).max(c_lo + 1);
    let mut ref_bs = Vec::new();
    let mut ref_gs = Vec::new();
    let mut ref_rs = Vec::new();
    for v in (y_top..y_bot).step_by(2) {
        for u in c_lo..c_hi {
            let (b, g, r) = bgr_at(bgr, w, u, v);
            ref_bs.push(b);
            ref_gs.push(g);
            ref_rs.push(r);
        }
    }
    if ref_bs.is_empty() {
        return None;
    }
    let ref_b = median_f64(&mut ref_bs);
    let ref_g = median_f64(&mut ref_gs);
    let ref_r = median_f64(&mut ref_rs);
    let ref_br = ref_b - ref_r;

    let v_lo = (v0 as isize - 30).max(y_bot as isize - 10).max(0) as usize;
    let v_hi = (v0 as isize + 32).min(h as isize - 10) as usize;
    if v_hi <= v_lo + 6 {
        return None;
    }

    let mut found_v = None;
    let mut run = 0usize;
    let mut rb = Vec::with_capacity(c_hi - c_lo);
    let mut rg = Vec::with_capacity(c_hi - c_lo);
    let mut rr = Vec::with_capacity(c_hi - c_lo);
    for v in v_lo..v_hi {
        rb.clear();
        rg.clear();
        rr.clear();
        for u in c_lo..c_hi {
            let (b, g, r) = bgr_at(bgr, w, u, v);
            rb.push(b);
            rg.push(g);
            rr.push(r);
        }
        let (b, g, r) = (median_f64(&mut rb), median_f64(&mut rg), median_f64(&mut rr));
        let br = b - r;
        let color_dist = (b - ref_b).abs() + (g - ref_g).abs() + (r - ref_r).abs();
        let bright_sky_floor = (g - ref_g) > 35.0 && color_dist > 90.0;
        let is_bar = br >= (ref_br * 0.65).max(35.0) && !bright_sky_floor;
        if is_bar {
            run += 1;
        } else {
            if run >= 4 {
                found_v = Some(v as f64 - 0.5);
            }
            run = 0;
        }
    }
    let found_v = found_v?;
    let (_, yn) = undistort_pt(best_u as f64, found_v)?;
    let pitch = yn.atan() - (CAM_H / xc).atan();
    (6.5_f64.to_radians()..=10.0_f64.to_radians()).contains(&pitch).then_some(pitch)
}

/// Fallback pitch from the far front blue board, guarding against bright sky-blue court floor.
fn estimate_pitch_from_front_board(bgr: &[u8], w: usize, h: usize, hint: &LaneLidarHint) -> Option<f64> {
    let fb = hint.front_board_m?;
    if !(2.5..=10.0).contains(&fb) || w < 400 || h < 370 {
        return None;
    }
    let xc = fb - CAM_AHEAD_LIDAR;
    let mut rows = Vec::with_capacity(7);
    let mut u = 330usize;
    while u <= 390 {
        let mut best = None;
        let mut start: Option<usize> = None;
        for i in 0..=160usize {
            let is_bar = if i < 160 {
                let v = 200 + i;
                let mut sb = 0.0;
                let mut sg = 0.0;
                let mut sr = 0.0;
                for du in [u - 1, u, u + 1] {
                    let (b, g, r) = bgr_at(bgr, w, du, v);
                    sb += b;
                    sg += g;
                    sr += r;
                }
                let (b, g, r) = (sb / 3.0, sg / 3.0, sr / 3.0);
                let br = b - r;
                let sky_floor = g > 140.0 && b > 195.0 && br > 120.0;
                br > 45.0 && !sky_floor
            } else {
                false
            };
            if is_bar && start.is_none() {
                start = Some(i);
            } else if !is_bar && let Some(s) = start.take()
                && i - s > 5
                && 200 + i - 1 < 340
            {
                best = Some(200 + i - 1);
            }
        }
        if let Some(b) = best {
            rows.push(b as f64 + 0.5);
        }
        u += 10;
    }
    if rows.len() < 4 {
        return None;
    }
    let row = median_f64(&mut rows);
    let (_, yn) = undistort_pt(CX, row)?;
    let pitch = yn.atan() - (CAM_H / xc).atan();
    (6.5_f64.to_radians()..=10.0_f64.to_radians()).contains(&pitch).then_some(pitch)
}

/// Pure-Rust crosswalk detection on one 640x480 BGR image with lidar lane & left-gap geometry.
pub fn detect_crosswalk(
    bgr: &[u8],
    w: usize,
    h: usize,
    hint: &LaneLidarHint,
    frame: i64,
    captured_mono_s: f64,
) -> Option<RustCrosswalkObs> {
    if bgr.len() < w * h * 3 || w < 160 || h < 120 {
        return None;
    }
    let t0 = Instant::now();
    let (pitch, pitch_source) = if let Some(p) = estimate_pitch_from_left_gap(bgr, w, h, hint) {
        (p, "left_gap")
    } else if let Some(p) = estimate_pitch_from_front_board(bgr, w, h, hint) {
        (p, "front_board")
    } else {
        (NOMINAL_PITCH_DEG.to_radians(), "nominal")
    };

    let y_left = hint.y_left_m.clamp(0.30, 0.85);
    let y_right = hint.y_right_m.clamp(-0.85, -0.30);
    let (mut y_min, mut y_max) = (y_right + 0.06, y_left - 0.06);
    if y_max - y_min < 0.48 {
        y_min = -0.39;
        y_max = 0.39;
    }

    let x_min = 0.48;
    let mut x_max = 3.35_f64;
    if let Some(gx) = hint.left_gap_x_m
        && gx > 0.90
    {
        x_max = x_max.min((gx + 0.60).max(1.30));
    }
    let dx = 0.02;
    let dy = 0.01;
    let nx = ((x_max - x_min) / dx).round() as usize + 1;
    let ny = ((y_max - y_min) / dy).round() as usize + 1;
    if nx < 22 || ny < 45 {
        return None;
    }

    let mut y_grid = vec![0.0_f32; nx * ny];
    let mut s_grid = vec![0.0_f32; nx * ny];
    let mut br_grid = vec![0.0_f32; nx * ny];
    let mut valid = vec![false; nx * ny];

    let (ch, sh) = (hint.heading_left_rad.cos(), hint.heading_left_rad.sin());
    let (cp, sp) = (pitch.cos(), pitch.sin());
    for ix in 0..nx {
        let xl = x_min + ix as f64 * dx;
        for iy in 0..ny {
            let yl = y_min + iy as f64 * dy;
            let xc = xl * ch + yl * sh - CAM_AHEAD_LIDAR;
            let yc = -xl * sh + yl * ch;
            if xc <= 0.18 {
                continue;
            }
            let z_rot = xc * cp - CAM_H * sp;
            let y_rot = xc * sp + CAM_H * cp;
            if z_rot <= 0.05 {
                continue;
            }
            let xn = -yc / z_rot;
            let yn = y_rot / z_rot;
            let r2 = xn * xn + yn * yn;
            if r2 > 2.2 {
                continue;
            }
            let rad = 1.0 + r2 * (K1 + r2 * (K2 + r2 * K3));
            let xd = xn * rad + 2.0 * P1 * xn * yn + P2 * (r2 + 2.0 * xn * xn);
            let yd = yn * rad + P1 * (r2 + 2.0 * yn * yn) + 2.0 * P2 * xn * yn;
            let u = (FX * xd + CX).round() as isize;
            let v = (FY * yd + CY).round() as isize;
            if u >= 0 && (u as usize) < w && v >= 0 && (v as usize) < h {
                let (b, g, r) = bgr_at(bgr, w, u as usize, v as usize);
                let mx = b.max(g).max(r);
                let mn = b.min(g).min(r);
                let cell = ix * ny + iy;
                y_grid[cell] = (0.114 * b + 0.587 * g + 0.299 * r) as f32;
                s_grid[cell] = ((mx - mn) / mx.max(1.0)) as f32;
                br_grid[cell] = (b - r) as f32;
                valid[cell] = true;
            }
        }
    }

    // Per-row local-window adaptive white-paper score W[ix, iy] in [0, 1].
    let mut w_map = vec![0.0_f32; nx * ny];
    let mut loc_y = Vec::with_capacity(40);
    let mut loc_s = Vec::with_capacity(40);
    for ix in 0..nx {
        let base = ix * ny;
        let row_valid = valid[base..base + ny].iter().filter(|&&v| v).count();
        if row_valid < 28 {
            continue;
        }
        for iy in 0..ny {
            if !valid[base + iy] {
                continue;
            }
            let lo = iy.saturating_sub(18);
            let hi = (iy + 19).min(ny);
            loc_y.clear();
            loc_s.clear();
            for j in lo..hi {
                if valid[base + j] {
                    loc_y.push(y_grid[base + j]);
                    loc_s.push(s_grid[base + j]);
                }
            }
            if loc_y.len() < 12 {
                continue;
            }
            loc_y.sort_by(|a, b| a.total_cmp(b));
            loc_s.sort_by(|a, b| a.total_cmp(b));
            let bg_y = loc_y[loc_y.len() / 4];
            let bg_s = loc_s[(3 * loc_s.len()) / 4];
            let yv = y_grid[base + iy];
            let sat = s_grid[base + iy];
            let br = br_grid[base + iy];
            if br > 42.0 && sat > 0.24 {
                continue;
            }
            let dy_lum = yv - bg_y;
            if bg_s >= 0.18 {
                let low_sat = sat <= 0.26 && (sat <= bg_s - 0.10 || sat <= 0.15);
                if low_sat && yv >= 60.0 && dy_lum >= 14.0 {
                    w_map[base + iy] = ((dy_lum / 36.0).min(1.0) * ((0.30 - sat) / 0.14).min(1.0)).max(0.0);
                } else if low_sat && yv >= 155.0 && (bg_s - sat) >= 0.22 {
                    w_map[base + iy] = ((bg_s - sat) / 0.35).clamp(0.0, 1.0);
                }
            } else if sat <= 0.22 && yv >= 115.0 && dy_lum >= 11.0 {
                w_map[base + iy] = ((dy_lum - 7.0) / 24.0).clamp(0.0, 1.0);
            }
        }
    }

    // Prefix sums along x per column for O(1) strip/before/after averages.
    let mut pref_w = vec![0.0_f32; (nx + 1) * ny];
    let mut pref_v = vec![0_u16; (nx + 1) * ny];
    for ix in 0..nx {
        let src = ix * ny;
        let dst = (ix + 1) * ny;
        for iy in 0..ny {
            pref_w[dst + iy] = pref_w[src + iy] + w_map[src + iy];
            pref_v[dst + iy] = pref_v[src + iy] + u16::from(valid[src + iy]);
        }
    }
    let col_mean_w = |r0: usize, r1: usize, iy: usize| -> f32 {
        if r1 <= r0 { return 0.0; }
        (pref_w[r1 * ny + iy] - pref_w[r0 * ny + iy]) / (r1 - r0) as f32
    };
    let col_mean_v = |r0: usize, r1: usize, iy: usize| -> f32 {
        if r1 <= r0 { return 0.0; }
        (pref_v[r1 * ny + iy] - pref_v[r0 * ny + iy]) as f32 / (r1 - r0) as f32
    };

    struct Best {
        ix0: usize,
        stripes: usize,
        gaps: usize,
        cols: Vec<(usize, usize)>,
        score: f64,
    }
    let mut best: Option<Best> = None;
    let mut p_in = vec![0.0_f32; ny];
    let mut p_bef = vec![0.0_f32; ny];
    let mut q_col = vec![0.0_f32; ny];
    let mut win_v = vec![0.0_f32; ny];

    for depth_rows in [12usize, 15usize] {
        if nx <= depth_rows + 5 {
            continue;
        }
        let half_d = depth_rows / 2;
        for ix0 in 3..(nx - depth_rows - 2) {
            let b_lo = ix0.saturating_sub(6);
            let b_hi = ix0.saturating_sub(1).max(b_lo + 1);
            let a_lo = (ix0 + depth_rows + 2).min(nx - 1);
            let a_hi = (ix0 + depth_rows + 7).min(nx);
            let mut valid_cols = 0usize;
            let mut bef_sum = 0.0_f32;
            for iy in 0..ny {
                let wv = col_mean_v(ix0, ix0 + depth_rows, iy);
                win_v[iy] = wv;
                if wv > 0.6 {
                    valid_cols += 1;
                }
                let pin = col_mean_w(ix0, ix0 + depth_rows, iy);
                let pb = col_mean_w(b_lo, b_hi, iy);
                let pa = col_mean_w(a_lo, a_hi, iy);
                let p_near = col_mean_w(ix0, ix0 + half_d, iy);
                let p_far = col_mean_w(ix0 + half_d, ix0 + depth_rows, iy);
                p_in[iy] = pin;
                p_bef[iy] = pb;
                bef_sum += pb;
                q_col[iy] = p_near.min(p_far) - 0.55 * pb - 0.45 * pa;
            }
            if valid_cols < 42 {
                continue;
            }
            let bef_mean = (bef_sum / ny as f32) as f64;
            if bef_mean > 0.45 {
                continue;
            }
            for pitch_cm in [19usize, 21usize, 23usize] {
                let half_p = pitch_cm / 2;
                for phase in 0..pitch_cm {
                    let mut pos = phase;
                    let mut stripes_ok = 0usize;
                    let mut gaps_ok = 0usize;
                    let mut s_sum = 0.0_f64;
                    let mut g_sum = 0.0_f64;
                    let mut cols = Vec::with_capacity(6);
                    while pos + half_p <= ny {
                        let (slo, shi) = (pos, pos + half_p);
                        let v_avg: f32 = win_v[slo..shi].iter().sum::<f32>() / (shi - slo) as f32;
                        if v_avg >= 0.7 {
                            let clo = slo + 1;
                            let chi = (shi - 1).max(clo + 1);
                            let inv = 1.0 / (chi - clo) as f32;
                            let sq = (q_col[clo..chi].iter().sum::<f32>() * inv) as f64;
                            let sr = (p_in[clo..chi].iter().sum::<f32>() * inv) as f64;
                            if sq >= 0.30 && sr >= 0.40 {
                                stripes_ok += 1;
                                s_sum += sq;
                                cols.push((slo, shi));
                                let glo = shi;
                                let ghi = (pos + pitch_cm).min(ny);
                                if ghi >= glo + 4 {
                                    let g1 = glo + 1;
                                    let g2 = (ghi - 1).max(g1 + 1);
                                    let gv = (p_in[g1..g2].iter().sum::<f32>() / (g2 - g1) as f32) as f64;
                                    if gv <= 0.24 && (sr - gv) >= 0.26 {
                                        gaps_ok += 1;
                                        g_sum += gv;
                                    }
                                }
                            }
                        }
                        pos += pitch_cm;
                    }
                    if stripes_ok >= 3 && gaps_ok >= 2 {
                        let ms = s_sum / stripes_ok as f64;
                        let mg = g_sum / gaps_ok as f64;
                        let sc = (ms - mg) * (stripes_ok as f64 / 4.0).min(1.0) * (1.0 - 0.5 * bef_mean);
                        if best.as_ref().is_none_or(|b| sc > b.score) {
                            best = Some(Best { ix0, stripes: stripes_ok, gaps: gaps_ok, cols, score: sc });
                        }
                    }
                }
            }
        }
    }

    let cand = best.filter(|b| b.score >= 0.25)?;
    let x_rough = x_min + cand.ix0 as f64 * dx;

    // Refine near-edge sub-pixel in original grayscale image along the detected stripe centers.
    let mut edge_ys = Vec::with_capacity(cand.cols.len() * 5);
    let mut col_buf = Vec::with_capacity(36);
    for &(slo, shi) in &cand.cols {
        let y_mid = y_min + 0.5 * (slo + shi) as f64 * dy;
        let (xc, yc) = lane_to_cam(x_rough, y_mid, hint.heading_left_rad);
        let Some((uc_f, vc_f)) = project_cam_ground(xc, yc, pitch) else { continue };
        let (u_c, v_c) = (uc_f.round() as isize, vc_f.round() as isize);
        if !(4..w as isize - 4).contains(&u_c) || !(15..h as isize - 12).contains(&v_c) {
            continue;
        }
        let top = (v_c - 18).max(0) as usize;
        let bot = (v_c + 14).min(h as isize - 1) as usize;
        for du in [-4_isize, -2, 0, 2, 4] {
            let u = u_c + du;
            if !(1..w as isize - 1).contains(&u) {
                continue;
            }
            let u = u as usize;
            col_buf.clear();
            for v in top..=bot {
                let mut lsum = 0.0;
                for uu in [u - 1, u, u + 1] {
                    let (b, g, r) = bgr_at(bgr, w, uu, v);
                    lsum += 0.114 * b + 0.587 * g + 0.299 * r;
                }
                col_buf.push(lsum / 3.0);
            }
            let mut pk = 0usize;
            for (k, &val) in col_buf.iter().enumerate() {
                if val > col_buf[pk] {
                    pk = k;
                }
            }
            if pk + 2 >= col_buf.len() {
                continue;
            }
            let fl = col_buf[pk..].iter().copied().fold(f64::INFINITY, f64::min);
            if col_buf[pk] - fl < 18.0 {
                continue;
            }
            let half = 0.5 * (col_buf[pk] + fl);
            let mut k = pk;
            while k + 1 < col_buf.len() && col_buf[k + 1] >= half {
                k += 1;
            }
            if k + 1 >= col_buf.len() || (col_buf[k] - col_buf[k + 1]).abs() < 1e-9 {
                continue;
            }
            let row = top as f64 + k as f64 + (col_buf[k] - half) / (col_buf[k] - col_buf[k + 1]);
            if row < 472.0
                && let Some((_, yn)) = undistort_pt(u as f64, row)
            {
                edge_ys.push(yn);
            }
        }
    }

    let (edge_y, cam_to_edge_m) = if edge_ys.len() >= 4 {
        let ey = median_f64(&mut edge_ys);
        let down = ey.atan() - pitch;
        if down > 1e-4 {
            (ey, CAM_H / down.tan())
        } else {
            (ey, (x_rough - CAM_AHEAD_LIDAR).max(0.25))
        }
    } else {
        let c = (x_rough - CAM_AHEAD_LIDAR).max(0.25);
        (((CAM_H / c).atan() + pitch).tan(), c)
    };

    let front_to_edge_m = cam_to_edge_m - CAM_BEHIND_FRONT;
    if !(0.28..=3.55).contains(&front_to_edge_m) {
        return None;
    }
    let lidar_to_edge_m = front_to_edge_m + FRONT_BODY_EXTENT_M;
    let gap_to_crosswalk_m = hint.left_gap_x_m.map(|gx| gx - lidar_to_edge_m);

    Some(RustCrosswalkObs {
        frame,
        captured_mono_s,
        front_to_edge_m,
        edge_y,
        pitch_rad: pitch,
        pitch_source,
        score: (cand.score * 1.4).min(0.99),
        stripes: cand.stripes,
        gaps: cand.gaps,
        left_gap_x_m: hint.left_gap_x_m,
        gap_to_crosswalk_m,
        compute_ms: t0.elapsed().as_secs_f64() * 1000.0,
    })
}

/// Parse one raw frame published by the Rust V4L2 camera.
/// Header (32 bytes, little-endian): magic[4]="XTF1", frame:u64, captured_s:f64, w:u32, h:u32, flags:u32.
pub fn parse_shm_frame(bytes: &[u8]) -> Option<(i64, f64, usize, usize, &[u8])> {
    if bytes.len() < SHM_HEADER_LEN || &bytes[0..4] != SHM_MAGIC {
        return None;
    }
    let frame = u64::from_le_bytes(bytes[4..12].try_into().ok()?) as i64;
    let captured_s = f64::from_le_bytes(bytes[12..20].try_into().ok()?);
    let w = u32::from_le_bytes(bytes[20..24].try_into().ok()?) as usize;
    let h = u32::from_le_bytes(bytes[24..28].try_into().ok()?) as usize;
    let payload = &bytes[SHM_HEADER_LEN..];
    if w == 0 || h == 0 || !captured_s.is_finite() || payload.len() != w.checked_mul(h)?.checked_mul(3)? {
        return None;
    }
    Some((frame, captured_s, w, h, payload))
}

/// Best-effort pin current Linux thread to a dedicated CPU core without unsafe code.
#[cfg(target_os = "linux")]
pub(crate) fn pin_current_thread(cpu: usize) -> Option<usize> {
    let status = std::fs::read_to_string("/proc/thread-self/status").ok()?;
    let tid = status
        .lines()
        .find_map(|line| line.strip_prefix("Pid:"))
        .map(str::trim)?;
    if tid.is_empty() || !tid.chars().all(|c| c.is_ascii_digit()) {
        return None;
    }
    let ok = Command::new("taskset")
        .args(["-pc", &cpu.to_string(), tid])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .ok()
        .is_some_and(|s| s.success());
    ok.then_some(cpu)
}

#[cfg(not(target_os = "linux"))]
pub(crate) fn pin_current_thread(_cpu: usize) -> Option<usize> {
    None
}

/// Spawn the dedicated pure-Rust crosswalk worker on CPU 6 (exits automatically once `stop` is set).
pub fn spawn_worker(
    shm_path: PathBuf,
    hint: Arc<Mutex<LaneLidarHint>>,
    stop: Arc<AtomicBool>,
    stats: Arc<Mutex<RustCrosswalkStats>>,
    tx: Sender<Event>,
) -> thread::JoinHandle<()> {
    spawn_named("rust-cw-vision", move || {
        let pinned = pin_current_thread(6);
        if let Ok(mut s) = stats.lock() {
            s.pinned_cpu = pinned;
        }
        let started_at = Instant::now();
        let mut last_frame: i64 = -1;
        while !stop.load(Ordering::Acquire) {
            let h_snap = hint.lock().map(|g| *g).unwrap_or_default();
            // Wait up to 0.8 s for the first lidar scan so initial stationary frames use
            // live left-gap pitch calibration instead of nominal 8.05 deg; never block past 0.8 s.
            if !h_snap.has_scan && started_at.elapsed() < Duration::from_millis(800) {
                thread::sleep(Duration::from_millis(15));
                continue;
            }
            if let Ok(bytes) = std::fs::read(&shm_path)
                && let Some((frame, captured_s, w, h, payload)) = parse_shm_frame(&bytes)
                && frame != last_frame
            {
                last_frame = frame;
                let obs = detect_crosswalk(payload, w, h, &h_snap, frame, captured_s);
                if let Ok(mut s) = stats.lock() {
                    s.frames_read += 1;
                    if let Some(ref o) = obs {
                        s.detections += 1;
                        if s.compute_ms.len() < 256 {
                            s.compute_ms.push(o.compute_ms);
                        }
                    }
                }
                if let Some(o) = obs
                    && tx.send(Event::RustCrosswalk(o, Instant::now())).is_err()
                {
                    break;
                }
            }
            thread::sleep(Duration::from_millis(15));
        }
        // The launcher owns these files until camera and lamp acquisition have stopped.
    })
}

pub fn remove_shm_best_effort(path: &Path) -> std::io::Result<()> {
    let _ = std::fs::remove_file(path);
    let mut tmp = path.as_os_str().to_os_string();
    tmp.push(".tmp");
    let _ = std::fs::remove_file(PathBuf::from(tmp));
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dict;

    fn make_court_scene(with_crosswalk: bool, xl_edge: f64, with_white_lines: bool) -> Vec<u8> {
        let (w, h) = (640usize, 480usize);
        let mut bgr = vec![0_u8; w * h * 3];
        let pitch = NOMINAL_PITCH_DEG.to_radians();
        // Split red/blue basketball court background + optional continuous 5 cm white basketball lines
        for v in 0..h {
            for u in 0..w {
                let idx = (v * w + u) * 3;
                if u < w / 2 {
                    // Terracotta red court BGR
                    bgr[idx] = 110;
                    bgr[idx + 1] = 132;
                    bgr[idx + 2] = 212;
                } else {
                    // Bright sky-blue court BGR
                    bgr[idx] = 232;
                    bgr[idx + 1] = 175;
                    bgr[idx + 2] = 60;
                }
            }
        }
        let paint_ground = |bgr: &mut [u8], xl0: f64, xl1: f64, yl0: f64, yl1: f64, col: [u8; 3]| {
            let mut x = xl0;
            while x <= xl1 {
                let mut y = yl0;
                while y <= yl1 {
                    if let Some((uf, vf)) = project_cam_ground(x - CAM_AHEAD_LIDAR, y, pitch) {
                        let (u, v) = (uf.round() as isize, vf.round() as isize);
                        if (0..w as isize).contains(&u) && (0..h as isize).contains(&v) {
                            let idx = (v as usize * w + u as usize) * 3;
                            bgr[idx..idx + 3].copy_from_slice(&col);
                        }
                    }
                    y += 0.005;
                }
                x += 0.005;
            }
        };
        if with_white_lines {
            // Horizontal 5 cm court line across entire lane + vertical 5 cm court line along lane
            paint_ground(&mut bgr, 1.30, 1.35, -0.45, 0.45, [245, 245, 245]);
            paint_ground(&mut bgr, 0.55, 2.80, -0.025, 0.025, [245, 245, 245]);
        }
        if with_crosswalk {
            // 4 half-A4 strips: 10.5 cm wide x 29.7 cm deep, 10.5 cm spacing
            for yc in [-0.315, -0.105, 0.105, 0.315] {
                paint_ground(&mut bgr, xl_edge, xl_edge + 0.297, yc - 0.0525, yc + 0.0525, [244, 244, 242]);
            }
        }
        bgr
    }


    #[test]
    fn retiring_crosswalk_worker_leaves_the_camera_frame_and_write_buffer() {
        let directory = std::env::temp_dir().join(format!("xt-cw-owner-{}",std::process::id()));
        std::fs::create_dir_all(&directory).unwrap();
        let shared=directory.join("camera.bgr");
        let pending=shared.with_added_extension("tmp");
        std::fs::write(&shared,b"current capture").unwrap();
        std::fs::write(&pending,b"next capture").unwrap();
        let (tx,_rx)=std::sync::mpsc::channel();
        let worker=spawn_worker(shared.clone(), Default::default(), Arc::new(AtomicBool::new(true)), Default::default(), tx);
        worker.join().unwrap();
        assert_eq!(std::fs::read(&shared).unwrap(),b"current capture");
        assert_eq!(std::fs::read(&pending).unwrap(),b"next capture");
        remove_shm_best_effort(&shared).unwrap();
        std::fs::remove_dir(directory).unwrap();
    }

    #[test]
    fn white_basketball_lines_alone_do_not_trigger_crosswalk() {
        let bgr = make_court_scene(false, 1.80, true);
        let hint = LaneLidarHint { left_gap_x_m: Some(2.60), ..Default::default() };
        assert!(detect_crosswalk(&bgr, 640, 480, &hint, 1, 1.0).is_none());
    }

    #[test]
    fn half_a4_crosswalk_detected_accurately_over_red_blue_and_white_court_lines() {
        let xl_edge = 1.80;
        let bgr = make_court_scene(true, xl_edge, true);
        let hint = LaneLidarHint { left_gap_x_m: Some(2.60), ..Default::default() };
        let obs = detect_crosswalk(&bgr, 640, 480, &hint, 2, 1.2).expect("crosswalk detected");
        let true_front = xl_edge - FRONT_BODY_EXTENT_M;
        assert!(
            (obs.front_to_edge_m - true_front).abs() <= 0.05,
            "front_to_edge_m={} vs true={}",
            obs.front_to_edge_m,
            true_front
        );
        assert!(obs.stripes >= 3 && obs.gaps >= 2);
        let gap_to_cw = obs.gap_to_crosswalk_m.expect("gap_to_crosswalk_m");
        assert!((gap_to_cw - (2.60 - xl_edge)).abs() <= 0.06);
    }

    #[test]
    fn lidar_hint_finds_left_exit_gap_with_slight_yaw() {
        let yaw = 3.0_f64.to_radians();
        let mut ranges = vec![V::None; 360];
        for (i, slot) in ranges.iter_mut().enumerate() {
            // Clockwise bin i -> body CCW angle = -i deg; lane CCW angle = -i + yaw
            let body_ccw = -(i as f64).to_radians();
            let lane_ccw = body_ccw + yaw;
            let (c, s) = (lane_ccw.cos(), lane_ccw.sin());
            let mut best = f64::INFINITY;
            // Right wall y = -0.45 for all x
            if s < -1e-6 { best = best.min(-0.45 / s); }
            // Left wall y = +0.45 only up to left gap at x = 2.60 m (10 deg ray hits at 2.552 m)
            if s > 1e-6 {
                let r = 0.45 / s;
                if r * c <= 2.60 {
                    best = best.min(r);
                }
            }
            // Far front wall x = 7.6 m, rear wall x = -0.4 m
            if c > 1e-6 { best = best.min(7.6 / c); }
            if c < -1e-6 { best = best.min(-0.40 / c); }
            if best.is_finite() {
                *slot = V::Float(best);
            }
        }
        let scan = dict! {"ranges" => V::list(ranges)};
        let hint = LaneLidarHint::from_scan(&scan).expect("hint");
        assert!((hint.heading_left_rad - yaw).abs() < 0.02, "yaw={}", hint.heading_left_rad);
        let gx = hint.left_gap_x_m.expect("left_gap_x_m");
        assert!((gx - 2.55).abs() < 0.08, "gx={gx}");
        assert!((hint.front_board_m.unwrap() - 7.6).abs() < 0.05);
    }

    #[test]
    fn shm_frame_header_roundtrip() {
        let mut buf = Vec::with_capacity(SHM_HEADER_LEN + 4 * 2 * 3);
        buf.extend_from_slice(SHM_MAGIC);
        buf.extend_from_slice(&42_u64.to_le_bytes());
        buf.extend_from_slice(&123.45_f64.to_le_bytes());
        buf.extend_from_slice(&4_u32.to_le_bytes());
        buf.extend_from_slice(&2_u32.to_le_bytes());
        buf.extend_from_slice(&0_u32.to_le_bytes());
        buf.extend_from_slice(&[7_u8; 24]);
        let (frame, cap, w, h, pay) = parse_shm_frame(&buf).expect("valid");
        assert_eq!((frame, w, h, pay.len()), (42, 4, 2, 24));
        assert!((cap - 123.45).abs() < 1e-9);
    }
}
