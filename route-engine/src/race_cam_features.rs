#![forbid(unsafe_code)]
//! Ambient-lighting-robust, zero-allocation visual feature extractor for
//! camera-assisted field localization. Runs on the existing `rust-light` CPU-3
//! thread alongside traffic-lamp detection without spawning extra threads.

use std::sync::LazyLock;
use std::time::Instant;

// Calibrated 640x480 intrinsics (config/camera-race4-20261031.json).
pub const IMG_W: usize = 640;
pub const IMG_H: usize = 480;
pub const FX: f64 = 426.55;
pub const FY: f64 = 424.47;
pub const CX: f64 = 349.59;
pub const CY: f64 = 227.85;
const K1: f64 = -0.371764;
const K2: f64 = 0.172259;
const P1: f64 = -0.000092;
const P2: f64 = -0.000226;
const K3: f64 = -0.042575;

// Camera extrinsics relative to LiDAR spindle (metres / radians).
// Positive pitch is camera optical axis tilted UP above horizontal (~8.2 deg empirical).
pub const CAM_AHEAD_LIDAR_M: f64 = 0.159;
pub const CAM_HEIGHT_M: f64 = 0.195;
pub const NOMINAL_PITCH_RAD: f64 = 8.2 * std::f64::consts::PI / 180.0;
pub const NOMINAL_CAM_YAW_RAD: f64 = 0.0;
pub const BOARD_TOP_H_M: f64 = 0.50;
pub const OUTDOOR_RAIL_H_M: f64 = 0.043;

pub const NUM_COLS: usize = 64;
pub const MAX_VERT_EDGES: usize = 16;
pub const MAX_CONES: usize = 4;

const LUT_W: usize = 65;
const LUT_H: usize = 49;
const LUT_STEP: f64 = 10.0;

static UNDISTORT_LUT: LazyLock<Box<[(f32, f32); LUT_W * LUT_H]>> = LazyLock::new(|| {
    let mut table = Box::new([(0.0_f32, 0.0_f32); LUT_W * LUT_H]);
    for gy in 0..LUT_H {
        let v = gy as f64 * LUT_STEP;
        for gx in 0..LUT_W {
            let u = gx as f64 * LUT_STEP;
            let (xn, yn) = undistort_exact(u, v);
            table[gy * LUT_W + gx] = (xn as f32, yn as f32);
        }
    }
    table
});

#[cfg(test)]
pub fn distort_norm(xn: f64, yn: f64) -> (f64, f64) {
    let r2 = xn * xn + yn * yn;
    let r4 = r2 * r2;
    let r6 = r4 * r2;
    let rad = 1.0 + K1 * r2 + K2 * r4 + K3 * r6;
    let xd = xn * rad + 2.0 * P1 * xn * yn + P2 * (r2 + 2.0 * xn * xn);
    let yd = yn * rad + P1 * (r2 + 2.0 * yn * yn) + 2.0 * P2 * xn * yn;
    (FX * xd + CX, FY * yd + CY)
}

fn undistort_exact(u: f64, v: f64) -> (f64, f64) {
    let xd0 = (u - CX) / FX;
    let yd0 = (v - CY) / FY;
    // Clamp extreme corners to the monotone branch of the k1/k2/k3 barrel polynomial.
    let rd = xd0.hypot(yd0);
    let scale = if rd > 0.855 { 0.855 / rd } else { 1.0 };
    let xd = xd0 * scale;
    let yd = yd0 * scale;
    let (mut x, mut y) = (xd, yd);
    for _ in 0..30 {
        let r2 = (x * x + y * y).min(1.85);
        let rad = (1.0 + r2 * (K1 + r2 * (K2 + r2 * K3))).max(0.25);
        let nx = (xd - 2.0 * P1 * x * y - P2 * (r2 + 2.0 * x * x)) / rad;
        let ny = (yd - P1 * (r2 + 2.0 * y * y) - 2.0 * P2 * x * y) / rad;
        let dx = nx - x;
        let dy = ny - y;
        x += 0.7 * dx;
        y += 0.7 * dy;
        let nr2 = x * x + y * y;
        if nr2 > 1.85 {
            let s = (1.85 / nr2).sqrt();
            x *= s;
            y *= s;
        }
        if dx.abs().max(dy.abs()) < 1e-8 {
            break;
        }
    }
    (x, y)
}

/// O(1) bilinear-interpolated undistortion into normalized camera coords `(xn, yn)`.
#[inline]
pub fn undistort_pixel(u: f64, v: f64) -> (f64, f64) {
    let lut = &*UNDISTORT_LUT;
    let gx = (u / LUT_STEP).clamp(0.0, (LUT_W - 1) as f64 - 1e-6);
    let gy = (v / LUT_STEP).clamp(0.0, (LUT_H - 1) as f64 - 1e-6);
    let ix = gx as usize;
    let iy = gy as usize;
    let fx = gx - ix as f64;
    let fy = gy - iy as f64;
    let p00 = lut[iy * LUT_W + ix];
    let p10 = lut[iy * LUT_W + ix + 1];
    let p01 = lut[(iy + 1) * LUT_W + ix];
    let p11 = lut[(iy + 1) * LUT_W + ix + 1];
    let xn = (p00.0 as f64 * (1.0 - fx) + p10.0 as f64 * fx) * (1.0 - fy)
        + (p01.0 as f64 * (1.0 - fx) + p11.0 as f64 * fx) * fy;
    let yn = (p00.1 as f64 * (1.0 - fx) + p10.1 as f64 * fx) * (1.0 - fy)
        + (p01.1 as f64 * (1.0 - fx) + p11.1 as f64 * fx) * fy;
    (xn, yn)
}

#[derive(Clone, Copy, Debug, Default)]
pub struct WallColObs {
    pub valid: bool,
    pub u: f32,
    pub v_bot: f32,
    pub v_top: f32,
    pub xn: f32,
    pub yn: f32,
    pub yn_top: f32,
    pub confidence: f32,
    pub has_rail: bool,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct VertEdgeObs {
    pub u: f32,
    pub xn: f32,
    pub weight: f32,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ConeObs {
    pub u: f32,
    pub v_bot: f32,
    pub xn: f32,
    pub yn: f32,
    pub width_px: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct VisualFrame {
    pub frame: i64,
    pub captured_mono_s: f64,
    pub cols: [WallColObs; NUM_COLS],
    pub valid_cols: u8,
    pub vert_edges: [VertEdgeObs; MAX_VERT_EDGES],
    pub n_vert_edges: u8,
    pub cones: [ConeObs; MAX_CONES],
    pub n_cones: u8,
    pub rail_fraction: f32,
    pub mean_luma: f32,
    pub compute_us: u32,
}

impl Default for VisualFrame {
    fn default() -> Self {
        Self {
            frame: 0,
            captured_mono_s: 0.0,
            cols: [WallColObs::default(); NUM_COLS],
            valid_cols: 0,
            vert_edges: [VertEdgeObs::default(); MAX_VERT_EDGES],
            n_vert_edges: 0,
            cones: [ConeObs::default(); MAX_CONES],
            n_cones: 0,
            rail_fraction: 0.0,
            mean_luma: 0.0,
            compute_us: 0,
        }
    }
}

/// Lighting-invariant integer chromaticity from BGR pixel: returns `(cb, bg, luma)`.
/// `cb` = 256*(2B - G - R)/(B + G + R + 32), `bg` = 256*(B - G)/(B + G + R + 32).
#[inline(always)]
fn pixel_chroma(b: u8, g: u8, r: u8) -> (i32, i32, i32) {
    let (bi, gi, ri) = (b as i32, g as i32, r as i32);
    let sum = bi + gi + ri + 32;
    let cb = ((2 * bi - gi - ri) << 8) / sum;
    let bg = ((bi - gi) << 8) / sum;
    let luma = (bi + 2 * gi + ri) >> 2;
    (cb, bg, luma)
}

#[inline(always)]
fn is_blue_board_wb(b: u8, g: u8, r: u8, cb_bg: i32, bg_bg: i32) -> (bool, i32, i32) {
    let (cb, bg, luma) = pixel_chroma(b, g, r);
    let cb_rel = cb - cb_bg;
    let bg_rel = bg - bg_bg;
    let normal_blue = cb_rel >= 26
        && bg_rel >= 10
        && luma >= 12
        && (b as i32) >= (r as i32) + 8
        && (cb >= 14 || cb_rel >= 36);
    // Overexposure guard: when B clips near 255 and G also climbs, B - R still separates blue board.
    let overexp_blue = b >= 246 && (b as i32) >= (r as i32) + 26 && cb_rel >= 18;
    (normal_blue || overexp_blue, cb_rel, luma)
}

#[inline(always)]
fn is_glare_blue(b: u8, g: u8, r: u8, cb_bg: i32) -> bool {
    let (cb, _, luma) = pixel_chroma(b, g, r);
    luma >= 195 && (cb - cb_bg) >= 6 && (b as i32) >= (r as i32) + 2
}

/// Fast (~2 us) neutral-surface white-balance baseline `(cb_bg, bg_bg)` estimated from
/// the lower-chroma floor/background pixels so warm/cool illuminant shifts cancel out.
fn estimate_wb_baseline(bgr: &[u8], w: usize, h: usize) -> (i32, i32) {
    let mut cbs = [0i32; 96];
    let mut bgs = [0i32; 96];
    let mut n = 0usize;
    // Sample floor strip (v in 78%..95%) and upper background strip (v in 6%..22%).
    for &vf in &[0.08, 0.16, 0.80, 0.86, 0.91, 0.95] {
        let v = ((vf * h as f64) as usize).min(h - 1);
        for k in 0..16 {
            let u = ((k * 2 + 1) * w / 32).min(w - 1);
            let (b, g, r) = sample_bgr(bgr, w, u, v);
            let (cb, bg, luma) = pixel_chroma(b, g, r);
            if (35..=245).contains(&luma) && cb.abs() < 90 {
                cbs[n] = cb;
                bgs[n] = bg;
                n += 1;
            }
        }
    }
    if n < 8 {
        return (0, 0);
    }
    cbs[..n].sort_unstable();
    bgs[..n].sort_unstable();
    // Pick median (robust to blue floor patches or shadows) and clamp to realistic illuminant cast.
    let cb_med = cbs[n / 2].clamp(-45, 58);
    let bg_med = bgs[n / 2].clamp(-22, 24);
    // Keep neutral indoor/outdoor frames crisp: ignore tiny casts < 8.
    let cb_bg = if cb_med.abs() < 8 { 0 } else { cb_med };
    let bg_bg = if bg_med.abs() < 5 { 0 } else { bg_med };
    (cb_bg, bg_bg)
}

#[inline(always)]
fn is_red_cone(b: u8, g: u8, r: u8) -> bool {
    let (bi, gi, ri) = (b as i32, g as i32, r as i32);
    let sum = bi + gi + ri + 32;
    let cr = ((2 * ri - gi - bi) << 8) / sum;
    cr >= 48 && ri >= gi + 28 && ri >= bi + 38 && ri >= 60
}

#[inline(always)]
fn sample_bgr(bgr: &[u8], w: usize, u: usize, v: usize) -> (u8, u8, u8) {
    let idx = (v * w + u) * 3;
    (bgr[idx], bgr[idx + 1], bgr[idx + 2])
}

/// Extract wall bottom-edge columns, depth-step vertical edges, and cone bearings
/// from a raw BGR frame using fixed stack storage only (zero heap allocations).
pub fn extract_visual_frame(
    bgr: &[u8],
    w: usize,
    h: usize,
    frame: i64,
    captured_mono_s: f64,
) -> VisualFrame {
    let t0 = Instant::now();
    let mut out = VisualFrame {
        frame,
        captured_mono_s,
        ..VisualFrame::default()
    };
    if w < 64 || h < 64 || bgr.len() < w * h * 3 {
        return out;
    }
    let sx = w as f64 / IMG_W as f64;
    let sy = h as f64 / IMG_H as f64;
    let v_scan_lo = (24.0 * sy) as usize;
    let v_scan_hi = ((470.0 * sy) as usize).min(h - 2);
    // Camera is pitched UP ~8.6 deg, so horizon lies at v ~ 280-296 px.
    // Every 50 cm board (lens at 19.5 cm) straddles this horizon band.
    let v_horiz_lo = (262.0 * sy) as usize;
    let v_horiz_hi = (306.0 * sy) as usize;
    let (cb_bg, bg_bg) = estimate_wb_baseline(bgr, w, h);
    let mut luma_sum: u64 = 0;
    let mut luma_cnt: u64 = 0;
    let mut rail_votes: u32 = 0;
    let mut rail_eligible: u32 = 0;

    for k in 0..NUM_COLS {
        let u_nom = 8 + k * 10;
        let u = ((u_nom as f64 * sx) as usize).min(w - 1);
        let mut best_top = 0usize;
        let mut best_bot = 0usize;
        let mut best_score = 0i32;
        let mut best_core_cb = 0i32;

        let mut in_run = false;
        let mut run_top = 0usize;
        let mut run_bot = 0usize;
        let mut blue_cnt = 0i32;
        let mut gap = 0usize;
        let mut cb_sum = 0i32;
        let mut core_cb = 0i32;
        let mut core_cnt = 0i32;

        let mut v = v_scan_lo;
        while v <= v_scan_hi {
            let (b, g, r) = sample_bgr(bgr, w, u, v);
            let (blue, cb, luma) = is_blue_board_wb(b, g, r, cb_bg, bg_bg);
            if v.is_multiple_of(16) {
                luma_sum += luma as u64;
                luma_cnt += 1;
            }
            let min_cb = if core_cnt >= 4 {
                ((core_cb / core_cnt) * 44 / 100).max(26)
            } else {
                26
            };
            let strong = blue && cb >= min_cb;
            // Reject glossy floor reflection jump: below horizon, sudden bright/low-chroma shift ends board.
            let glare_ok = in_run && gap <= 8 && is_glare_blue(b, g, r, cb_bg);
            if strong {
                if !in_run {
                    in_run = true;
                    run_top = v;
                    blue_cnt = 0;
                    cb_sum = 0;
                    core_cb = 0;
                    core_cnt = 0;
                }
                run_bot = v;
                blue_cnt += 1;
                cb_sum += cb;
                if v <= v_horiz_hi || core_cnt < 6 {
                    core_cb += cb;
                    core_cnt += 1;
                }
                gap = 0;
            } else if glare_ok {
                gap += 2;
            } else if in_run {
                gap += 2;
                if gap > 6 {
                    let span = run_bot.saturating_sub(run_top);
                    if run_top <= v_horiz_hi && run_bot >= v_horiz_lo && span >= 12 && blue_cnt >= 5 {
                        let score = blue_cnt * 16 + (cb_sum / blue_cnt.max(1));
                        if score > best_score {
                            best_score = score;
                            best_top = run_top;
                            best_bot = run_bot;
                            best_core_cb = cb_sum / blue_cnt.max(1);
                        }
                    }
                    in_run = false;
                    gap = 0;
                }
            }
            v += 2;
        }
        if in_run {
            let span = run_bot.saturating_sub(run_top);
            if run_top <= v_horiz_hi && run_bot >= v_horiz_lo && span >= 12 && blue_cnt >= 5 {
                let score = blue_cnt * 16 + (cb_sum / blue_cnt.max(1));
                if score > best_score {
                    best_score = score;
                    best_top = run_top;
                    best_bot = run_bot;
                    best_core_cb = cb_sum / blue_cnt.max(1);
                }
            }
        }
        if best_score == 0 {
            continue;
        }
        // Refine bottom edge to 1 px and guard against glossy blue reflection tail.
        let thresh_cb = (best_core_cb * 48 / 100).max(26);
        let mut vb = best_bot;
        while vb + 1 < h && vb < best_bot + 3 {
            let (b, g, r) = sample_bgr(bgr, w, u, vb + 1);
            let (ok, cb, _) = is_blue_board_wb(b, g, r, cb_bg, bg_bg);
            if ok && cb >= thresh_cb {
                vb += 1;
            } else {
                break;
            }
        }
        // Check for reflection crease inside the lower 40% of a tall below-horizon run.
        let mut vt = best_top;
        while vt > 1 && vt + 3 > best_top {
            let (b, g, r) = sample_bgr(bgr, w, u, vt - 1);
            let (ok, cb, _) = is_blue_board_wb(b, g, r, cb_bg, bg_bg);
            if ok && cb >= 22 {
                vt -= 1;
            } else {
                break;
            }
        }
        // Outdoor aluminum rail signature: thin neutral-gray band right below blue panel.
        let mut has_rail = false;
        let span = vb.saturating_sub(vt);
        if span >= 28 && vb + 18 < h {
            rail_eligible += 1;
            let rail_h = ((span as f64 * 0.095).round() as usize).clamp(3, 22);
            let mid_v = (vb + (rail_h / 2).max(2)).min(h - 3);
            let (rb, rg, rr) = sample_bgr(bgr, w, u, mid_v);
            let (rcb, _, rluma) = pixel_chroma(rb, rg, rr);
            let (fb, fg, fr) = sample_bgr(bgr, w, u, (vb + rail_h + 3).min(h - 1));
            let (_, _, fluma) = pixel_chroma(fb, fg, fr);
            let sat = rb.max(rg).max(rr) - rb.min(rg).min(rr);
            if (rcb - cb_bg).abs() <= 24 && sat <= 42 && rluma >= 45 && (rluma - fluma) >= 16 {
                has_rail = true;
                rail_votes += 1;
            }
        }
        let u640 = u as f64 / sx;
        let vb640 = vb as f64 / sy;
        let vt640 = vt as f64 / sy;
        let (xn, yn) = undistort_pixel(u640, vb640);
        let (_, yn_top) = undistort_pixel(u640, vt640);
        let conf = ((best_core_cb as f64 / 70.0).clamp(0.35, 1.0)
            * ((span as f64) / 20.0).clamp(0.4, 1.0)) as f32;
        out.cols[k] = WallColObs {
            valid: true,
            u: u640 as f32,
            v_bot: vb640 as f32,
            v_top: vt640 as f32,
            xn: xn as f32,
            yn: yn as f32,
            yn_top: yn_top as f32,
            confidence: conf,
            has_rail,
        };
    }

    // Edge-preserving 3-column spike rejection (removes isolated 1-column glitches).
    let raw_v: [f32; NUM_COLS] = std::array::from_fn(|i| out.cols[i].v_bot);
    let raw_ok: [bool; NUM_COLS] = std::array::from_fn(|i| out.cols[i].valid);
    let mut valid_cols = 0u8;
    for k in 0..NUM_COLS {
        if !raw_ok[k] {
            continue;
        }
        if k > 0 && k + 1 < NUM_COLS && raw_ok[k - 1] && raw_ok[k + 1] {
            let avg = 0.5 * (raw_v[k - 1] + raw_v[k + 1]);
            if (raw_v[k - 1] - raw_v[k + 1]).abs() < 12.0 && (raw_v[k] - avg).abs() > 14.0 {
                out.cols[k].valid = false;
                continue;
            }
        }
        valid_cols += 1;
    }
    out.valid_cols = valid_cols;
    out.rail_fraction = if rail_eligible >= 4 {
        rail_votes as f32 / rail_eligible as f32
    } else {
        0.0
    };
    out.mean_luma = if luma_cnt > 0 {
        (luma_sum as f64 / luma_cnt as f64) as f32
    } else {
        0.0
    };

    // Pitch-invariant vertical depth-step & corner edges from wall-bottom profile.
    let mut n_edges = 0usize;
    for k in 0..(NUM_COLS - 1) {
        if n_edges >= MAX_VERT_EDGES {
            break;
        }
        let a = out.cols[k];
        let b = out.cols[k + 1];
        if a.valid && b.valid && (a.v_bot - b.v_bot).abs() >= 14.0 {
            let u_mid = 0.5 * (a.u + b.u) as f64;
            let v_mid = 0.5 * (a.v_bot + b.v_bot) as f64;
            let (xn, _) = undistort_pixel(u_mid, v_mid);
            out.vert_edges[n_edges] = VertEdgeObs {
                u: u_mid as f32,
                xn: xn as f32,
                weight: 1.0,
            };
            n_edges += 1;
        } else if k >= 2 && k + 2 < NUM_COLS && out.cols[k - 2].valid && a.valid && b.valid && out.cols[k + 2].valid {
            let s_left = (a.v_bot - out.cols[k - 2].v_bot) / 20.0;
            let s_right = (out.cols[k + 2].v_bot - b.v_bot) / 20.0;
            if s_left * s_right < -0.02 && (s_left - s_right).abs() >= 0.25 {
                let u_mid = 0.5 * (a.u + b.u) as f64;
                let v_mid = 0.5 * (a.v_bot + b.v_bot) as f64;
                let (xn, _) = undistort_pixel(u_mid, v_mid);
                out.vert_edges[n_edges] = VertEdgeObs {
                    u: u_mid as f32,
                    xn: xn as f32,
                    weight: 0.7,
                };
                n_edges += 1;
            }
        }
    }
    out.n_vert_edges = n_edges as u8;

    // Coarse red/orange cone detector below horizon (pitch-invariant bearing landmark).
    let mut cone_col = [0u16; NUM_COLS];
    let mut cone_vbot = [0u16; NUM_COLS];
    let v_cone_lo = (210.0 * sy) as usize;
    let v_cone_hi = ((470.0 * sy) as usize).min(h - 1);
    for k in 0..NUM_COLS {
        let u = (((8 + k * 10) as f64 * sx) as usize).min(w - 1);
        let mut cnt = 0u16;
        let mut vmax = 0u16;
        let mut v = v_cone_lo;
        while v <= v_cone_hi {
            let (b, g, r) = sample_bgr(bgr, w, u, v);
            if is_red_cone(b, g, r) {
                cnt += 1;
                vmax = v as u16;
            }
            v += 3;
        }
        if cnt >= 4 {
            cone_col[k] = cnt;
            cone_vbot[k] = vmax;
        }
    }
    let mut n_cones = 0usize;
    let mut k = 0usize;
    while k < NUM_COLS && n_cones < MAX_CONES {
        if cone_col[k] == 0 {
            k += 1;
            continue;
        }
        let start = k;
        let mut wsum = 0.0;
        let mut usum = 0.0;
        let mut vmax = 0u16;
        while k < NUM_COLS && cone_col[k] > 0 {
            let wt = cone_col[k] as f64;
            wsum += wt;
            usum += wt * (8 + k * 10) as f64;
            vmax = vmax.max(cone_vbot[k]);
            k += 1;
        }
        let cols_span = k - start;
        if (1..=14).contains(&cols_span) {
            let u_c = usum / wsum;
            let v_c = vmax as f64 / sy;
            let (xn, yn) = undistort_pixel(u_c, v_c);
            out.cones[n_cones] = ConeObs {
                u: u_c as f32,
                v_bot: v_c as f32,
                xn: xn as f32,
                yn: yn as f32,
                width_px: (cols_span * 10) as f32,
            };
            n_cones += 1;
        }
    }
    out.n_cones = n_cones as u8;
    out.compute_us = t0.elapsed().as_micros().min(u32::MAX as u128) as u32;
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn undistort_lut_round_trip_subpixel() {
        for v in (40..=440).step_by(40) {
            for u in (40..=600).step_by(40) {
                let rd = ((u as f64 - CX) / FX).hypot((v as f64 - CY) / FY);
                if rd > 0.84 {
                    continue;
                }
                let (xn, yn) = undistort_pixel(u as f64, v as f64);
                let (u2, v2) = distort_norm(xn, yn);
                assert!((u2 - u as f64).abs() < 0.15, "u={u} v={v} err={}", u2 - u as f64);
                assert!((v2 - v as f64).abs() < 0.15, "u={u} v={v} err={}", v2 - v as f64);
            }
        }
    }

    #[test]
    fn extractor_handles_bright_dark_and_rejects_blue_floor() {
        for &(wall_bgr, floor_bgr) in &[
            ([210u8, 120, 45], [140u8, 145, 145]), // bright outdoor
            ([70u8, 38, 14], [30u8, 34, 30]),      // dark dusk
            ([185u8, 105, 40], [160u8, 155, 110]), // cyan-tinted court floor
        ] {
            let mut img = vec![0u8; IMG_W * IMG_H * 3];
            for v in 0..IMG_H {
                let px = if (205..=345).contains(&v) {
                    wall_bgr
                } else if v > 345 {
                    floor_bgr
                } else {
                    [200, 200, 200]
                };
                for u in 0..IMG_W {
                    let i = (v * IMG_W + u) * 3;
                    img[i..i + 3].copy_from_slice(&px);
                }
            }
            let vis = extract_visual_frame(&img, IMG_W, IMG_H, 1, 1.0);
            assert!(vis.valid_cols >= 60, "valid_cols={}", vis.valid_cols);
            let mean_v: f64 = vis.cols.iter().filter(|c| c.valid).map(|c| c.v_bot as f64).sum::<f64>()
                / vis.valid_cols as f64;
            assert!((mean_v - 345.0).abs() <= 2.0, "mean_v={mean_v}");
        }
    }
}
