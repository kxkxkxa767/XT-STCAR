#![forbid(unsafe_code)]
//! Ambient-lighting-robust camera-assisted localization solver and LiDAR speckle filter.
//! Runs zero-allocation 4-DoF Gauss-Newton IRLS fusion `(x, y, yaw, pitch)` on top of
//! `race_field_map::Field` and `race_cam_features::VisualFrame`.

use std::time::Instant;
use xt_stcar_robot_core::autonomy::Pose2;

use crate::race_cam_features::{
    BOARD_TOP_H_M, CAM_AHEAD_LIDAR_M, CAM_HEIGHT_M, FX, FY, NOMINAL_CAM_YAW_RAD,
    NOMINAL_PITCH_RAD, OUTDOOR_RAIL_H_M, VisualFrame,
};
use crate::race_field_map::{Field, Ray};

const MATCH_M: f64 = 0.25;
const CAM_MAX_AGE_S: f64 = 0.45;
const PITCH_MIN_RAD: f64 = 5.0 * std::f64::consts::PI / 180.0;
const PITCH_MAX_RAD: f64 = 11.5 * std::f64::consts::PI / 180.0;
const CAM_YAW_MAX_RAD: f64 = 5.0 * std::f64::consts::PI / 180.0;

/// Angular-coherence speckle filter for degraded/noisy LiDAR scans.
/// Keeps clean R1 scans bit-for-bit untouched; removes isolated random noise in R3-R5
/// while preserving short <= 0.5 m real board arcs and cones.
pub(crate) fn filter_lidar_speckles(ranges: &[Option<f64>]) -> ([Option<f64>; 360], bool) {
    let mut out = [None; 360];
    let n = ranges.len().min(360);
    if n == 0 {
        return (out, false);
    }
    let mut raw_valid = 0usize;
    let mut kept = 0usize;
    for i in 0..n {
        let Some(r) = ranges[i] else { continue };
        if !r.is_finite() || r <= 0.04 {
            continue;
        }
        raw_valid += 1;
        let mut coh = 0usize;
        let mut adj = 0usize;
        for step in [-2i32, -1, 1, 2] {
            let j = (i as i32 + step).rem_euclid(n as i32) as usize;
            if let Some(rn) = ranges[j] {
                let s = step.unsigned_abs() as f64;
                let tol = (0.048 + 0.055 * r.min(rn)) * s;
                if (rn - r).abs() <= tol {
                    coh += 1;
                    if step.abs() == 1 {
                        adj += 1;
                    }
                }
            }
        }
        if coh >= 2 && adj >= 1 {
            out[i] = Some(r);
            kept += 1;
        }
    }
    // Preserve clean full scans (R1) verbatim so clean baseline never shifts by even 1 ray.
    if raw_valid >= 180 && kept * 100 >= raw_valid * 85 {
        out[..n].copy_from_slice(&ranges[..n]);
    }
    // A short but continuous board arc also has low coverage. Only scattered
    // returns call for temporal cone verification.
    let isolated = raw_valid - kept;
    (out, isolated >= 4 && 3 * isolated >= raw_valid)
}

#[derive(Clone, Debug)]
pub(crate) struct CamLocState {
    pub pitch_rad: f64,
    pub cam_yaw_rad: f64,
    pub rail_height_m: f64,
    pub rail_belief: f64,
    latest_obs: Option<(VisualFrame, f64)>,
    last_fused_frame: Option<i64>,
    pub last_cam_fix_at: Option<f64>,
    pub frames_observed: u64,
    pub frames_fused: u64,
    pub cal_updates: u64,
    pub last_inliers: usize,
    pub last_edge_inliers: usize,
    pub last_pixel_rms: f64,
    pub last_solve_us: u32,
    pub last_cam_anchored: [bool; 2],
    pub last_used_cam: bool,
}

impl Default for CamLocState {
    fn default() -> Self {
        Self {
            pitch_rad: NOMINAL_PITCH_RAD,
            cam_yaw_rad: NOMINAL_CAM_YAW_RAD,
            rail_height_m: 0.0,
            rail_belief: 0.0,
            latest_obs: None,
            last_fused_frame: None,
            last_cam_fix_at: None,
            frames_observed: 0,
            frames_fused: 0,
            cal_updates: 0,
            last_inliers: 0,
            last_edge_inliers: 0,
            last_pixel_rms: 0.0,
            last_solve_us: 0,
            last_cam_anchored: [false; 2],
            last_used_cam: false,
        }
    }
}

#[inline]
fn lens_at_cam_time(
    pose: Pose2,
    cam_yaw: f64,
    dt_cam: f64,
    speed: f64,
    yaw_rate: f64,
) -> (f64, f64, f64, f64, f64) {
    let yaw_c = pose.yaw_rad - yaw_rate * dt_cam;
    let (sy, cy) = yaw_c.sin_cos();
    let (dx_m, dy_m, dmx_dyaw, dmy_dyaw) = if yaw_rate.abs() > 0.08 && dt_cam.abs() > 1e-3 {
        // Exact constant-curvature circular-arc back-projection during turns.
        let r = speed / yaw_rate;
        let (s0, c0) = pose.yaw_rad.sin_cos();
        let dx = r * (s0 - sy);
        let dy = r * (cy - c0);
        let ddx = r * (c0 - cy);
        let ddy = r * (s0 - sy);
        (dx, dy, ddx, ddy)
    } else {
        let mid = pose.yaw_rad - 0.5 * yaw_rate * dt_cam;
        let (sm, cm) = mid.sin_cos();
        let dist = speed * dt_cam;
        (dist * cm, dist * sm, -dist * sm, dist * cm)
    };
    let cx = pose.x_m - dx_m + CAM_AHEAD_LIDAR_M * cy;
    let cy_w = pose.y_m - dy_m + CAM_AHEAD_LIDAR_M * sy;
    let dcx_dyaw = -dmx_dyaw - CAM_AHEAD_LIDAR_M * sy;
    let dcy_dyaw = -dmy_dyaw + CAM_AHEAD_LIDAR_M * cy;
    (cx, cy_w, yaw_c + cam_yaw, dcx_dyaw, dcy_dyaw)
}

#[inline]
fn ray_world_dir(xn: f64, yn_obs: f64, pitch: f64, theta: f64) -> (f64, f64, f64, f64) {
    // Camera optical axis is tilted UP by +pitch above horizontal.
    let (sp, cp) = pitch.sin_cos();
    let fwd_r = cp + yn_obs * sp;
    let left_r = -xn;
    let hlen = fwd_r.hypot(left_r).max(1e-6);
    let cos_az = fwd_r / hlen;
    let sin_az = left_r / hlen;
    let (st, ct) = theta.sin_cos();
    let dx = ct * cos_az - st * sin_az;
    let dy = st * cos_az + ct * sin_az;
    (dx, dy, cos_az, sin_az)
}

#[inline]
fn proj_yn(fwd: f64, z_m: f64, pitch: f64) -> Option<(f64, f64, f64)> {
    // Camera optical axis is tilted UP by +pitch above horizontal.
    let up = z_m - CAM_HEIGHT_M;
    let (sp, cp) = pitch.sin_cos();
    let yc = sp * fwd - cp * up;
    let zc = cp * fwd + sp * up;
    if zc <= 0.08 {
        return None;
    }
    Some((yc / zc, up, zc))
}

#[allow(clippy::too_many_arguments)]
fn eval_col_fixed_board(
    field: &Field,
    board_id: usize,
    cx: f64,
    cy: f64,
    theta: f64,
    pitch: f64,
    xn: f64,
    yn_obs: f64,
    z_m: f64,
) -> Option<(f64, f64, f64, f64, f64)> {
    let b = &field.walls[board_id];
    let (dx, dy, cos_az, _sin_az) = ray_world_dir(xn, yn_obs, pitch, theta);
    let (q, u, other, v) = if b.vertical { (cx, dx, cy, dy) } else { (cy, dy, cx, dx) };
    if u.abs() < 0.08 {
        return None;
    }
    let dist = (b.fixed - q) / u;
    let along = other + dist * v;
    if dist <= 0.12 || dist > 9.5 || along < b.lo - 0.05 || along > b.hi + 0.05 {
        return None;
    }
    let fwd = dist * cos_az;
    if fwd <= 0.10 {
        return None;
    }
    let (yn_pred, up, zc) = proj_yn(fwd, z_m, pitch)?;
    Some((dist, yn_pred, up / (zc * zc), cos_az, u))
}

#[allow(clippy::needless_range_loop)]
fn solve_4x4(mut a: [[f64; 4]; 4], mut b: [f64; 4]) -> Option<[f64; 4]> {
    for col in 0..4 {
        let mut piv = col;
        let mut best = a[col][col].abs();
        for r in (col + 1)..4 {
            if a[r][col].abs() > best {
                best = a[r][col].abs();
                piv = r;
            }
        }
        if best < 1e-12 {
            return None;
        }
        if piv != col {
            a.swap(col, piv);
            b.swap(col, piv);
        }
        let diag = a[col][col];
        for r in (col + 1)..4 {
            let f = a[r][col] / diag;
            b[r] -= f * b[col];
            for c in col..4 {
                a[r][c] -= f * a[col][c];
            }
        }
    }
    let mut x = [0.0; 4];
    for r in (0..4).rev() {
        let mut s = b[r];
        for c in (r + 1)..4 {
            s -= a[r][c] * x[c];
        }
        x[r] = s / a[r][r];
    }
    Some(x)
}

fn add_normal_eq(h: &mut [[f64; 4]; 4], g: &mut [f64; 4], j: [f64; 4], r: f64, w: f64) {
    for a in 0..4 {
        let wa = w * j[a];
        g[a] -= wa * r;
        for b in a..4 {
            h[a][b] += wa * j[b];
        }
    }
}

impl CamLocState {
    pub fn observe_camera(&mut self, obs: VisualFrame, captured_at: f64) {
        self.latest_obs = Some((obs, captured_at));
        self.frames_observed += 1;
        // Online visual prior for aluminum bottom rail before LiDAR calibration overrides.
        let rf = obs.rail_fraction as f64;
        if self.frames_observed == 1 {
            self.rail_belief = rf;
        } else {
            self.rail_belief = 0.85 * self.rail_belief + 0.15 * rf;
        }
        if self.cal_updates < 5 {
            self.rail_height_m = if self.rail_belief >= 0.20 {
                OUTDOOR_RAIL_H_M
            } else {
                0.0
            };
        }
    }

    pub fn fresh_obs(&self, at: f64) -> Option<(&VisualFrame, f64)> {
        let (obs, cam_at) = self.latest_obs.as_ref()?;
        let age = at - *cam_at;
        if (-0.12..=CAM_MAX_AGE_S).contains(&age) && obs.valid_cols >= 4 {
            Some((obs, age))
        } else {
            None
        }
    }

    /// True only when an unfused fresh visual frame is ready for a new camera update,
    /// avoiding 3-4x duplicate over-fusion of the same frame between captures.
    pub fn has_new_obs(&self, at: f64) -> bool {
        self.fresh_obs(at)
            .is_some_and(|(obs, _)| self.last_fused_frame != Some(obs.frame))
    }

    /// Project a detected camera cone onto the ground plane (`z = 0`) using the
    /// motion-compensated camera pose at capture time, returning `(world_center, dist_from_lens, bearing_body_rad)`.
    /// Rejects observations occluded by a closer field wall or inconsistent with physical cone size.
    pub fn project_cone_to_world(
        &self,
        field: &Field,
        pose: Pose2,
        dt_cam: f64,
        speed: f64,
        yaw_rate: f64,
        cn: &crate::race_cam_features::ConeObs,
    ) -> Option<(xt_stcar_robot_core::autonomy::Point2, f64, f64)> {
        let (cx, cy, theta, _, _) =
            lens_at_cam_time(pose, self.cam_yaw_rad, dt_cam, speed, yaw_rate);
        let pitch = self.pitch_rad;
        let (sp, cp) = pitch.sin_cos();
        let xn = cn.xn as f64;
        let yn = cn.yn as f64;
        // Upward-pitched camera ray in camera-aligned horizontal frame.
        let fwd_r = cp + yn * sp;
        let up_r = sp - yn * cp;
        let left_r = -xn;
        if fwd_r <= 0.15 || up_r >= -0.024 {
            return None;
        }
        // Intersect ground contact ray at z = 0 (up = -CAM_HEIGHT_M).
        let scale_g = -CAM_HEIGHT_M / up_r;
        let fwd_g = scale_g * fwd_r;
        let left_g = scale_g * left_r;
        let horiz_dist = fwd_g.hypot(left_g);
        if !(0.32..=6.20).contains(&horiz_dist) {
            return None;
        }
        // Physical cone dimensions check (real cone: ~0.22 m base, ~0.48 m tall).
        let est_w = horiz_dist * (cn.width_px as f64 / FX);
        let est_h = horiz_dist * (cn.height_px as f64 / FY);
        if !(0.06..=0.65).contains(&est_w) || !(0.16..=0.95).contains(&est_h) {
            return None;
        }
        let (st, ct) = theta.sin_cos();
        let ux = (ct * fwd_g - st * left_g) / horiz_dist;
        let uy = (st * fwd_g + ct * left_g) / horiz_dist;
        // Reject if a surveyed board blocks the line of sight before the projected cone.
        if let Some((_, wall_d)) = field.hit(cx, cy, ux, uy)
            && wall_d + 0.18 < horiz_dist
        {
            return None;
        }
        // Camera base edge sits on the near face of the ~0.10 m radius cone; shift toward center.
        let center_d = horiz_dist + 0.09;
        let world = xt_stcar_robot_core::autonomy::Point2 {
            x_m: cx + ux * center_d,
            y_m: cy + uy * center_d,
        };
        let body = pose.world_to_body(world);
        let bearing = body.y_m.atan2(body.x_m);
        Some((world, horiz_dist, bearing))
    }

    pub fn exposure_origin(&self, pose: Pose2, age: f64, speed: f64, yaw_rate: f64) -> xt_stcar_robot_core::autonomy::Point2 {
        let (x_m, y_m, _, _, _) = lens_at_cam_time(pose, self.cam_yaw_rad, age, speed, yaw_rate);
        xt_stcar_robot_core::autonomy::Point2 { x_m, y_m }
    }

    pub fn cone_silhouette_matches(&self, field: &Field, pose: Pose2, age: f64, speed: f64,
        yaw_rate: f64, cn: &crate::race_cam_features::ConeObs, world: xt_stcar_robot_core::autonomy::Point2) -> bool {
        let (cx, cy, theta, _, _) = lens_at_cam_time(pose, self.cam_yaw_rad, age, speed, yaw_rate);
        let (dx, dy) = (world.x_m - cx, world.y_m - cy);
        let distance = dx.hypot(dy);
        if !(0.32..=6.2).contains(&distance)
            || field.hit(cx, cy, dx / distance, dy / distance).is_some_and(|(_, wall)| wall + 0.18 < distance)
        {
            return false;
        }
        let Some((xn, yn)) = project_world_pt(world.x_m, world.y_m, 0.48, cx, cy, theta, self.pitch_rad)
        else { return false };
        let (top_x, top_y) = crate::race_cam_features::undistort_pixel(cn.u as f64, cn.v_top as f64);
        (FX * (xn - top_x)).abs() <= (0.45 * f64::from(cn.width_px)).max(8.0)
            && (FY * (yn - top_y)).abs() <= (0.25 * f64::from(cn.height_px)).clamp(12.0, 40.0)
    }

    pub fn latest_visual_obs(&self, at: f64) -> Option<(VisualFrame, f64)> {
        let (obs, cam_at) = self.latest_obs.as_ref()?;
        let age = at - *cam_at;
        if (-0.12..=CAM_MAX_AGE_S).contains(&age) {
            Some((*obs, age))
        } else {
            None
        }
    }

    /// Passively calibrate pitch, camera-to-LiDAR yaw offset, and rail height (0 vs 4.3 cm)
    /// when LiDAR has a strong clean fix. Never modifies the LiDAR pose.
    pub fn calibrate_from_lidar(
        &mut self,
        field: &Field,
        pose: Pose2,
        at: f64,
        speed: f64,
        yaw_rate: f64,
    ) {
        self.last_used_cam = false;
        self.last_cam_anchored = [false; 2];
        let Some((obs, dt_cam)) = self.fresh_obs(at) else { return };
        let obs = *obs;
        let (cx, cy, theta, _, _) =
            lens_at_cam_time(pose, self.cam_yaw_rad, dt_cam, speed, yaw_rate);
        let eval_z = |z_m: f64, p: f64| -> (f64, usize, f64) {
            let mut sum_abs = 0.0;
            let mut sum_dp = 0.0;
            let mut sum_dp2 = 0.0;
            let mut n = 0usize;
            for col in &obs.cols {
                if !col.valid || col.confidence < 0.35 {
                    continue;
                }
                let xn = col.xn as f64;
                let yn = col.yn as f64;
                let (dx, dy, _, _) = ray_world_dir(xn, yn, p, theta);
                let Some((bid, _)) = field.hit(cx, cy, dx, dy) else { continue };
                let Some((_, yn_pred, _, _, _)) =
                    eval_col_fixed_board(field, bid, cx, cy, theta, p, xn, yn, z_m)
                else {
                    continue;
                };
                let r = FY * (yn_pred - yn);
                if r.abs() > 28.0 {
                    continue;
                }
                let eps = 1e-4;
                let yn_p = eval_col_fixed_board(field, bid, cx, cy, theta, p + eps, xn, yn, z_m)
                    .map(|t| t.1)
                    .unwrap_or(yn_pred);
                let j_p = FY * (yn_p - yn_pred) / eps;
                let w = 6.0 / r.abs().max(6.0);
                sum_abs += r.abs();
                sum_dp += w * j_p * r;
                sum_dp2 += w * j_p * j_p;
                n += 1;
            }
            let dp = if sum_dp2 > 1e-3 { -sum_dp / sum_dp2 } else { 0.0 };
            (if n > 0 { sum_abs / n as f64 } else { 999.0 }, n, dp)
        };
        let (e0, n0, dp0) = eval_z(0.0, self.pitch_rad);
        let (er, nr, dpr) = eval_z(OUTDOOR_RAIL_H_M, self.pitch_rad);
        if n0.max(nr) < 10 {
            return;
        }
        let use_rail = if nr >= 10 && er + 1.2 < e0 {
            true
        } else if n0 >= 10 && e0 + 1.2 < er {
            false
        } else {
            self.rail_belief >= 0.30
        };
        let target_z = if use_rail { OUTDOOR_RAIL_H_M } else { 0.0 };
        self.rail_height_m = 0.85 * self.rail_height_m + 0.15 * target_z;
        let dp = if use_rail { dpr } else { dp0 };
        self.pitch_rad = (self.pitch_rad + 0.15 * dp.clamp(-0.012, 0.012))
            .clamp(PITCH_MIN_RAD, PITCH_MAX_RAD);

        // Passive cam_yaw update from corners & cones (pitch-invariant).
        let mut num = 0.0;
        let mut den = 0.0;
        let landmarks = field_vertical_landmarks(field);
        for e in &obs.vert_edges[..obs.n_vert_edges as usize] {
            if let Some((_, r_u, j_yaw)) =
                match_vert_landmark(field, &landmarks, cx, cy, theta, self.pitch_rad, e.xn as f64)
            {
                let w = (e.weight as f64) * 8.0 / r_u.abs().max(8.0);
                num -= w * j_yaw * r_u;
                den += w * j_yaw * j_yaw;
            }
        }
        if den > 500.0 {
            let dcy = (num / den).clamp(-0.015, 0.015);
            self.cam_yaw_rad = (self.cam_yaw_rad + 0.12 * dcy).clamp(-CAM_YAW_MAX_RAD, CAM_YAW_MAX_RAD);
        }
        self.cal_updates += 1;
    }

    /// Joint 4-DoF `(x, y, yaw, pitch)` Gauss-Newton IRLS registration combining
    /// surviving LiDAR wall beams, IMU prior, and camera wall/edge/cone geometry.
    #[allow(clippy::too_many_arguments, clippy::needless_range_loop)]
    pub fn joint_register(
        &mut self,
        field: &Field,
        prior: Pose2,
        beams: &[Ray],
        imu_yaw: bool,
        at: f64,
        speed: f64,
        yaw_rate: f64,
    ) -> (Pose2, [usize; 2], [bool; 2], f64) {
        let t0 = Instant::now();
        let Some((obs_ref, dt_cam)) = self.fresh_obs(at) else {
            self.last_used_cam = false;
            self.last_cam_anchored = [false; 2];
            return (prior, [0, 0], [false, false], 0.0);
        };
        let obs = *obs_ref;
        let landmarks = field_vertical_landmarks(field);
        let z_bot = self.rail_height_m;
        let mut pose = prior;
        let mut pitch = self.pitch_rad;
        let mut lidar_support = [0usize; 2];
        let mut lidar_err = 0.0;
        let mut cam_inliers = 0usize;
        let mut edge_inliers = 0usize;
        let mut cam_sq_sum = 0.0;
        let mut cam_info_xy = [0.0f64; 2];
        let mut cam_boards_xy = [0usize; 2];

        for iter in 0..5 {
            let mut h = [[0.0f64; 4]; 4];
            let mut g = [0.0f64; 4];

            // 1. Time-adaptive inertial prior (EKF-style covariance growth since last anchor):
            //    tight on short 0.1s steps (zero single-frame jitter), softer after 0.3s gaps
            //    so multi-frame continuous tracking stays tightly locked through 2 m/s S-turns.
            let dt_fix = self
                .last_cam_fix_at
                .map_or(0.30, |t| (at - t).clamp(0.08, 0.50));
            let pos_prior = if beams.is_empty() {
                (1.6 / dt_fix).clamp(4.0, 14.0)
            } else {
                (1.3 / dt_fix).clamp(3.5, 11.0)
            };
            let yaw_prior = if imu_yaw {
                (22.0 / dt_fix).clamp(60.0, 150.0)
            } else {
                8.0
            };
            let pitch_prior = 1400.0;
            h[0][0] += pos_prior;
            g[0] += pos_prior * (prior.x_m - pose.x_m);
            h[1][1] += pos_prior;
            g[1] += pos_prior * (prior.y_m - pose.y_m);
            h[2][2] += yaw_prior;
            g[2] += yaw_prior * (prior.yaw_rad - pose.yaw_rad);
            h[3][3] += pitch_prior;
            g[3] += pitch_prior * (self.pitch_rad - pitch);

            // 2. Surviving clean LiDAR beams (exact same Huber & per-face cap as register).
            let (s, c) = pose.yaw_rad.sin_cos();
            let mut f_cnt = [0usize; 6];
            let mut f_w = [0.0; 6];
            let mut f_lev = [0.0; 6];
            let mut f_lev2 = [0.0; 6];
            let mut f_res = [0.0; 6];
            let mut f_mom = [0.0; 6];
            let mut f_abs = [0.0; 6];
            for ray in beams {
                let dx = c * ray.dx - s * ray.dy;
                let dy = s * ray.dx + c * ray.dy;
                let ox = c * ray.ox - s * ray.oy;
                let oy = s * ray.ox + c * ray.oy;
                let Some((id, expected)) = field.hit(pose.x_m + ox, pose.y_m + oy, dx, dy) else {
                    continue;
                };
                let (normal, lever) = if field.walls[id].vertical {
                    (dx, -oy - ray.range * dy)
                } else {
                    (dy, ox + ray.range * dx)
                };
                let diff = ray.range - expected;
                let e = diff * normal;
                if normal.abs() < 0.19 || e.abs() > MATCH_M || diff.abs() > 0.48_f64.max(expected * 0.18) {
                    continue;
                }
                let w = 0.047 / e.abs().max(0.047);
                f_cnt[id] += 1;
                f_w[id] += w;
                f_lev[id] += w * lever;
                f_lev2[id] += w * lever * lever;
                f_res[id] += w * e;
                f_mom[id] += w * lever * e;
                f_abs[id] += e.abs();
            }
            lidar_support = [0, 0];
            let mut sum_abs = 0.0;
            for i in 0..6 {
                if f_cnt[i] < 3 {
                    continue;
                }
                let axis = usize::from(!field.walls[i].vertical);
                let scale = 26.0 / (f_cnt[i] as f64).max(26.0);
                h[axis][axis] += scale * f_w[i];
                h[axis][2] += scale * f_lev[i];
                g[axis] -= scale * f_res[i];
                h[2][2] += scale * f_lev2[i];
                g[2] -= scale * f_mom[i];
                lidar_support[axis] += f_cnt[i];
                sum_abs += f_abs[i];
            }
            let l_n = lidar_support[0] + lidar_support[1];
            lidar_err = if l_n > 0 { sum_abs / l_n as f64 } else { 0.0 };

            // 3. Motion-compensated camera wall-bottom (and top) residuals with per-board cap
            //    so correlated columns on one board never overwhelm IMU or LiDAR.
            let (cx, cy, theta, dcx_dyaw, dcy_dyaw) =
                lens_at_cam_time(pose, self.cam_yaw_rad, dt_cam, speed, yaw_rate);
            cam_inliers = 0;
            cam_sq_sum = 0.0;
            cam_info_xy = [0.0, 0.0];
            cam_boards_xy = [0, 0];
            let gate_px = if iter == 0 { 44.0 } else { 24.0 };
            let px_sigma = 9.0;
            // Mild downweight only when motion-compensation lag or turn rate is large.
            let age_w = 1.0 / (1.0 + 4.0 * dt_cam * dt_cam + 0.20 * (yaw_rate * dt_cam).powi(2));
            let mut b_h = [[[0.0f64; 4]; 4]; 6];
            let mut b_g = [[0.0f64; 4]; 6];
            let mut b_cnt = [0usize; 6];

            for col in &obs.cols {
                if !col.valid || col.confidence < 0.25 {
                    continue;
                }
                let xn = col.xn as f64;
                let yn = col.yn as f64;
                let (dx_w, dy_w, _, _) = ray_world_dir(xn, yn, pitch, theta);
                let Some((bid, _)) = field.hit(cx, cy, dx_w, dy_w) else { continue };
                let Some((dist, yn_pred, dyn_dfwd, cos_az, u_norm)) =
                    eval_col_fixed_board(field, bid, cx, cy, theta, pitch, xn, yn, z_bot)
                else {
                    continue;
                };
                if dist > 7.5 || u_norm.abs() < 0.16 {
                    continue;
                }
                let r = FY * (yn_pred - yn);
                if r.abs() > gate_px {
                    continue;
                }
                let vert = field.walls[bid].vertical;
                let ddist_dx = if vert { -1.0 / u_norm } else { 0.0 };
                let ddist_dy = if vert { 0.0 } else { -1.0 / u_norm };
                let (dq_dyaw, du_dyaw) = if vert { (dcx_dyaw, -dy_w) } else { (dcy_dyaw, dx_w) };
                let ddist_dyaw = -(dq_dyaw + dist * du_dyaw) / u_norm;
                let eps = 1e-4;
                let yn_p = eval_col_fixed_board(field, bid, cx, cy, theta, pitch + eps, xn, yn, z_bot)
                    .map(|t| t.1)
                    .unwrap_or(yn_pred);
                let j_scale = FY * dyn_dfwd * cos_az;
                let j = [
                    j_scale * ddist_dx,
                    j_scale * ddist_dy,
                    j_scale * ddist_dyaw,
                    FY * (yn_p - yn_pred) / eps,
                ];
                let huber = 6.0 / r.abs().max(6.0);
                let w = (col.confidence as f64) * huber / (px_sigma * px_sigma);
                add_normal_eq(&mut b_h[bid], &mut b_g[bid], j, r, w);
                b_cnt[bid] += 1;
                cam_inliers += 1;
                cam_sq_sum += r * r;
                let axis = usize::from(!vert);
                cam_boards_xy[axis] += 1;

                // Wall-top edge is free of floor reflection and aluminum bottom rail.
                if col.v_top >= 24.0 && col.v_bot - col.v_top >= 18.0 && col.v_bot < 465.0 {
                    let yn_top = col.yn_top as f64;
                    if let Some((yt_pred, up_t, zc_t)) = proj_yn(dist * cos_az, BOARD_TOP_H_M, pitch) {
                        let rt = FY * (yt_pred - yn_top);
                        if rt.abs() <= 20.0 {
                            let jt_scale = FY * (up_t / (zc_t * zc_t)) * cos_az;
                            let yt_p = proj_yn(dist * cos_az, BOARD_TOP_H_M, pitch + eps)
                                .map(|t| t.0)
                                .unwrap_or(yt_pred);
                            let jt = [
                                jt_scale * ddist_dx,
                                jt_scale * ddist_dy,
                                jt_scale * ddist_dyaw,
                                FY * (yt_p - yt_pred) / eps,
                            ];
                            let wt = 0.55 * (col.confidence as f64) * (6.0 / rt.abs().max(6.0)) / (px_sigma * px_sigma);
                            add_normal_eq(&mut b_h[bid], &mut b_g[bid], jt, rt, wt);
                        }
                    }
                }
            }
            for bid in 0..6 {
                if b_cnt[bid] < 2 {
                    continue;
                }
                let scale = age_w * 5.8 / (b_cnt[bid] as f64).max(5.8);
                for a in 0..4 {
                    g[a] += scale * b_g[bid][a];
                    for b in a..4 {
                        h[a][b] += scale * b_h[bid][a][b];
                    }
                }
                let axis = usize::from(!field.walls[bid].vertical);
                cam_info_xy[axis] += scale * b_h[bid][axis][axis];
            }

            // 4. Pitch-invariant vertical edges (field corners & divider tips).
            //    Movable cones are intentionally excluded from pose localization so
            //    relocated cones can never pull body pose or bias themselves.
            edge_inliers = 0;
            for e in &obs.vert_edges[..obs.n_vert_edges as usize] {
                if let Some((lm, r_u, _)) =
                    match_vert_landmark(field, &landmarks, cx, cy, theta, pitch, e.xn as f64)
                {
                    let (j, _) = point_xn_jacobian(lm, cx, cy, theta, pitch, dcx_dyaw, dcy_dyaw);
                    let w = (e.weight as f64) * (8.0 / r_u.abs().max(8.0)) / (11.0 * 11.0);
                    add_normal_eq(&mut h, &mut g, j, r_u, w);
                    edge_inliers += 1;
                    cam_info_xy[0] += w * j[0] * j[0];
                    cam_info_xy[1] += w * j[1] * j[1];
                }
            }

            // Mirror upper triangle to lower triangle.
            for a in 0..4 {
                for b in 0..a {
                    let v = h[b][a];
                    h[a][b] = v;
                }
            }
            if cam_inliers == 0 && edge_inliers == 0 && l_n == 0 {
                break;
            }
            let Some(step) = solve_4x4(h, g) else { break };
            let dx = step[0].clamp(-0.14, 0.14);
            let dy = step[1].clamp(-0.14, 0.14);
            let dh = step[2].clamp(-0.065, 0.065);
            let dp = step[3].clamp(-0.025, 0.025);
            pose.x_m = (pose.x_m + dx).clamp(0.02, field.walls[1].fixed - 0.02);
            pose.y_m = (pose.y_m + dy).clamp(0.02, field.walls[3].fixed - 0.02);
            pose.yaw_rad += dh;
            pitch = (pitch + dp).clamp(PITCH_MIN_RAD, PITCH_MAX_RAD);
            if dx.abs().max(dy.abs()).max(dh.abs()) < 0.0006 {
                break;
            }
        }

        let pixel_rms = if cam_inliers > 0 {
            (cam_sq_sum / cam_inliers as f64).sqrt()
        } else {
            999.0
        };
        let cam_anchored = [
            cam_inliers >= 6 && pixel_rms < 18.0 && (cam_info_xy[0] >= 0.30 || cam_boards_xy[0] >= 4),
            cam_inliers >= 6 && pixel_rms < 18.0 && (cam_info_xy[1] >= 0.30 || cam_boards_xy[1] >= 4),
        ];
        // Do NOT permanently drift the static hot-glued mount pitch during blind camera solves;
        // static pitch is updated only by `calibrate_from_lidar`.
        if cam_anchored[0] || cam_anchored[1] {
            self.last_cam_fix_at = Some(at);
        }
        self.last_fused_frame = Some(obs.frame);
        self.frames_fused += 1;
        self.last_inliers = cam_inliers;
        self.last_edge_inliers = edge_inliers;
        self.last_pixel_rms = pixel_rms;
        self.last_cam_anchored = cam_anchored;
        self.last_used_cam = cam_inliers > 0 || edge_inliers > 0;
        self.last_solve_us = t0.elapsed().as_micros().min(u32::MAX as u128) as u32;

        let combined_err = if lidar_support[0] + lidar_support[1] > 0 {
            lidar_err
        } else {
            (pixel_rms * 0.006).min(0.25)
        };
        (pose, lidar_support, cam_anchored, combined_err)
    }
}

fn field_vertical_landmarks(field: &Field) -> [(f64, f64); 6] {
    let l = field.walls[1].fixed;
    let w = field.walls[3].fixed;
    [
        (0.0, 0.0),
        (l, 0.0),
        (l, w),
        (0.0, w),
        (field.walls[4].hi, field.walls[4].fixed),
        (field.walls[5].lo, field.walls[5].fixed),
    ]
}

fn project_world_pt(
    xw: f64,
    yw: f64,
    zw: f64,
    cx: f64,
    cy: f64,
    theta: f64,
    pitch: f64,
) -> Option<(f64, f64)> {
    let (st, ct) = theta.sin_cos();
    let dx = xw - cx;
    let dy = yw - cy;
    let fwd = dx * ct + dy * st;
    let left = -dx * st + dy * ct;
    if fwd <= 0.30 {
        return None;
    }
    let up = zw - CAM_HEIGHT_M;
    let (sp, cp) = pitch.sin_cos();
    let zc = cp * fwd + sp * up;
    if zc <= 0.25 {
        return None;
    }
    let yc = sp * fwd - cp * up;
    Some((-left / zc, yc / zc))
}

fn match_vert_landmark(
    field: &Field,
    landmarks: &[(f64, f64); 6],
    cx: f64,
    cy: f64,
    theta: f64,
    pitch: f64,
    xn_obs: f64,
) -> Option<((f64, f64), f64, f64)> {
    let mut best: Option<((f64, f64), f64, f64)> = None;
    let mut second = f64::INFINITY;
    for &lm in landmarks {
        let Some((xn_p, _)) = project_world_pt(lm.0, lm.1, 0.25, cx, cy, theta, pitch) else {
            continue;
        };
        let dist = (lm.0 - cx).hypot(lm.1 - cy);
        if !(0.35..=8.2).contains(&dist) {
            continue;
        }
        let dx = (lm.0 - cx) / dist;
        let dy = (lm.1 - cy) / dist;
        if let Some((_, hit_d)) = field.hit(cx, cy, dx, dy)
            && hit_d + 0.15 < dist
        {
            continue;
        }
        let r_u = FX * (xn_p - xn_obs);
        let a = r_u.abs();
        if a < 36.0 {
            let eps = 1e-4;
            let xn_p2 = project_world_pt(lm.0, lm.1, 0.25, cx, cy, theta + eps, pitch)
                .map(|t| t.0)
                .unwrap_or(xn_p);
            let j_theta = FX * (xn_p2 - xn_p) / eps;
            if best.is_none_or(|b| a < b.1.abs()) {
                if let Some(prev) = best {
                    second = prev.1.abs();
                }
                best = Some((lm, r_u, j_theta));
            } else if a < second {
                second = a;
            }
        }
    }
    best.filter(|b| second - b.1.abs() > 12.0)
}

fn point_xn_jacobian(
    lm: (f64, f64),
    cx: f64,
    cy: f64,
    theta: f64,
    pitch: f64,
    dcx_dyaw: f64,
    dcy_dyaw: f64,
) -> ([f64; 4], f64) {
    let eps = 1e-4;
    let base = project_world_pt(lm.0, lm.1, 0.25, cx, cy, theta, pitch)
        .map(|t| t.0)
        .unwrap_or(0.0);
    let px = project_world_pt(lm.0, lm.1, 0.25, cx + eps, cy, theta, pitch)
        .map(|t| t.0)
        .unwrap_or(base);
    let py = project_world_pt(lm.0, lm.1, 0.25, cx, cy + eps, theta, pitch)
        .map(|t| t.0)
        .unwrap_or(base);
    let pt = project_world_pt(
        lm.0,
        lm.1,
        0.25,
        cx + dcx_dyaw * eps,
        cy + dcy_dyaw * eps,
        theta + eps,
        pitch,
    )
    .map(|t| t.0)
    .unwrap_or(base);
    let pp = project_world_pt(lm.0, lm.1, 0.25, cx, cy, theta, pitch + eps)
        .map(|t| t.0)
        .unwrap_or(base);
    (
        [
            FX * (px - base) / eps,
            FX * (py - base) / eps,
            FX * (pt - base) / eps,
            FX * (pp - base) / eps,
        ],
        base,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::race_cam_features::{NUM_COLS, WallColObs};
    use crate::race_field_map::FIELD;

    #[test]
    fn speckle_filter_rejects_random_noise_and_keeps_short_wall() {
        let mut ranges = [None; 360];
        // Synthetic 0.45 m wall arc on bins 80..115.
        for i in 80..115 {
            let ang = ((i as f64 - 90.0).to_radians()).cos().max(0.3);
            ranges[i] = Some(0.35 / ang);
        }
        // Inject 140 isolated pseudo-random speckle spikes across remaining bins.
        let mut seed = 1234567u64;
        for i in 0..360 {
            if (80..115).contains(&i) {
                continue;
            }
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            if (seed >> 32) % 10 < 5 {
                let r = 0.20 + ((seed >> 16) % 500) as f64 * 0.01;
                ranges[i] = Some(r);
            }
        }
        let (filtered, _) = filter_lidar_speckles(&ranges);
        let wall_kept = (80..115).filter(|&i| filtered[i].is_some()).count();
        let noise_kept = (0..360).filter(|&i| !(80..115).contains(&i) && filtered[i].is_some()).count();
        assert!(wall_kept >= 33, "short wall arc lost: {wall_kept}/35");
        assert!(noise_kept <= 8, "too much speckle survived: {noise_kept}");
    }

    fn wall_frame(field: &Field, true_pose: Pose2) -> VisualFrame {
        let pitch = NOMINAL_PITCH_RAD;
        let theta = true_pose.yaw_rad + NOMINAL_CAM_YAW_RAD;
        let cx = true_pose.x_m + CAM_AHEAD_LIDAR_M * true_pose.yaw_rad.cos();
        let cy = true_pose.y_m + CAM_AHEAD_LIDAR_M * true_pose.yaw_rad.sin();
        let mut vis = VisualFrame { frame: 1, captured_mono_s: 1.0, ..Default::default() };
        let mut valid = 0u8;
        for c in 0..NUM_COLS {
            let az = -0.62 + 1.24 * (c as f64 / (NUM_COLS - 1) as f64);
            let (st, ct) = (theta + az).sin_cos();
            if let Some((_, dist)) = field.hit(cx, cy, ct, st)
                && let Some((xn, yn)) = project_world_pt(cx + dist * ct, cy + dist * st, 0.0, cx, cy, theta, pitch)
                && let Some((_, yn_t)) = project_world_pt(cx + dist * ct, cy + dist * st, BOARD_TOP_H_M, cx, cy, theta, pitch)
            {
                vis.cols[c] = WallColObs {
                    valid: true,
                    u: (FX * xn + 349.59) as f32,
                    v_bot: (FY * yn + 227.85) as f32,
                    v_top: (FY * yn_t + 227.85) as f32,
                    xn: xn as f32,
                    yn: yn as f32,
                    yn_top: yn_t as f32,
                    confidence: 1.0,
                    has_rail: false,
                };
                valid += 1;
            }
        }
        vis.valid_cols = valid;
        if let Some((xn_e, _)) = project_world_pt(field.walls[4].hi, field.walls[4].fixed, 0.25, cx, cy, theta, pitch) {
            vis.vert_edges[0] = crate::race_cam_features::VertEdgeObs {
                u: (FX * xn_e + 349.59) as f32,
                xn: xn_e as f32,
                weight: 1.0,
            };
            vis.n_vert_edges = 1;
        }
        vis
    }

    #[test]
    fn blind_lidar_camera_converges_to_true_pose() {
        let field = FIELD.as_ref().expect("field");
        let true_pose = Pose2 { x_m: 1.85, y_m: 0.46, yaw_rad: 0.05 };
        let vis = wall_frame(field, true_pose);
        let mut st = CamLocState::default();
        st.observe_camera(vis, 1.0);
        let perturbed = Pose2 { x_m: 1.65, y_m: 0.34, yaw_rad: -0.02 };
        let (est, _, anchored, _) = st.joint_register(field, perturbed, &[], true, 1.0, 0.0, 0.0);
        assert!(anchored[0] && anchored[1], "expected camera anchor: {anchored:?}");
        // Single-frame balanced Kalman update pulls 20 cm / 4 deg perturbation strongly toward truth.
        assert!((est.x_m - true_pose.x_m).abs() < 0.14, "x err: {}", est.x_m - true_pose.x_m);
        assert!((est.y_m - true_pose.y_m).abs() < 0.04, "y err: {}", est.y_m - true_pose.y_m);
        assert!((est.yaw_rad - true_pose.yaw_rad).abs() < 0.05, "yaw err: {}", est.yaw_rad - true_pose.yaw_rad);
    }

    #[test]
    fn camera_velocity_remains_finite_across_long_scan_interval() {
        use crate::motion_sensors::RaceInputs;
        use crate::py::V;
        use crate::race_field_map::Locator;

        let pose = Pose2 { x_m: 1.85, y_m: 0.46, yaw_rad: 0.05 };
        let walls = wall_frame(FIELD.as_ref().expect("field"), pose);
        let mut locator = Locator::default();
        locator.force_state_for_eval(1, 1.0, pose, 0.0, Some(0.0), Some(0.0));
        let imu = RaceInputs { yaw_rate: Some(0.0), forward_integral: Some(0.0) };
        let missing = V::list(vec![V::None; 360]);
        for (seq, at) in [(2i64, 1.1), (3, 1.72)] {
            let scan = crate::dict! { "seq" => seq, "received_at" => at, "ranges" => missing.clone() };
            locator.observe_camera(VisualFrame { frame: seq, captured_mono_s: at, ..walls }, at);
            locator.observe(&scan, at, imu, false);
            assert!(locator.camera_localized(&scan));
            assert_eq!(locator.cam_loc.last_cam_fix_at, Some(at));
            let speed = locator.body_speed(&scan).expect("camera anchor");
            assert!((0.0..=2.5).contains(&speed), "speed: {speed}");
        }
    }

    #[test]
    fn heavy_speckle_scan_in_locator_observe_never_panics() {
        use crate::motion_sensors::RaceInputs;
        use crate::py::V;
        use crate::race_field_map::Locator;

        let mut loc = Locator::default();
        loc.force_state_for_eval(
            1,
            1.0,
            Pose2 { x_m: 1.5, y_m: 0.45, yaw_rad: 0.0 },
            0.8,
            Some(0.0),
            Some(0.0),
        );
        // Inject heavy 50% random speckle noise across 360 bins (R4/R5 regime).
        let mut seed = 987654321u64;
        let mut ranges = Vec::with_capacity(360);
        for _ in 0..360 {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            if (seed >> 32) % 2 == 0 {
                let r = 0.15 + ((seed >> 16) % 500) as f64 * 0.01;
                ranges.push(V::from(r));
            } else {
                ranges.push(V::None);
            }
        }
        let scan = crate::dict! {
            "seq" => 2i64,
            "received_at" => 1.1,
            "ranges" => V::list(ranges),
        };
        let imu = RaceInputs { yaw_rate: Some(0.0), forward_integral: Some(0.0) };
        loc.observe(&scan, 1.1, imu, false);
        assert!(loc.raw_pose_xy_yaw().is_some());
    }
}
