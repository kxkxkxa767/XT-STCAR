//! Deterministic 640x480 BGR synthetic camera renderer for `surface_sim`.
//! Uses the exact calibrated Brown-Conrady lens model (`race_cam_features::undistort_pixel`),
//! camera extrinsics (`CAM_AHEAD_LIDAR_M`, `CAM_HEIGHT_M`, `NOMINAL_PITCH_RAD`),
//! true `world::World` boards/dividers and tapered 3D cones (Cone 0 red/orange, Cone 1 blue),
//! then feeds the raw BGR buffer into production `race_cam_features::extract_visual_frame`.

use crate::plant::Pose;
use crate::race_cam_features::{
    BOARD_TOP_H_M, CAM_AHEAD_LIDAR_M, CAM_HEIGHT_M, IMG_H, IMG_W, NOMINAL_PITCH_RAD,
    OUTDOOR_RAIL_H_M, undistort_pixel,
};
use crate::world::World;

const AZ_BINS: usize = 1024;
const AZ_MAX_RAD: f64 = 1.05; // +-60.2 deg covers full distorted 640x480 FOV
const CONE_HEIGHT_M: f64 = 0.48;
const CONE_BASE_R_M: f64 = 0.11;
const CONE_TOP_R_M: f64 = 0.032;

#[derive(Clone, Copy)]
struct PixelRay {
    az_bin: u16,
    tan_el: f32,
}

#[derive(Clone, Copy, Default)]
struct ConeColumnHit {
    dist: f32,
    lat_abs: f32,
    kind: u8, // 0 = none, 1 = red/orange (cone 0), 2 = blue (cone 1)
}

pub struct CameraRenderer {
    lut: Vec<PixelRay>,
    az_dirs: Vec<(f64, f64)>, // (cos(phi), sin(phi)) in camera horizontal frame (fwd, left)
    outdoor_rail: bool,
    bgr: Vec<u8>,
}

impl CameraRenderer {
    pub fn new(pitch_rad: f64, outdoor_rail: bool) -> Self {
        let (sp, cp) = pitch_rad.sin_cos();
        let mut lut = Vec::with_capacity(IMG_W * IMG_H);
        let az_scale = (AZ_BINS - 1) as f64 / (2.0 * AZ_MAX_RAD);
        for v in 0..IMG_H {
            for u in 0..IMG_W {
                let (xn, yn) = undistort_pixel(u as f64, v as f64);
                let fwd_r = cp + yn * sp;
                let up_r = sp - yn * cp;
                let left_r = -xn;
                let horiz = fwd_r.hypot(left_r).max(1e-6);
                let phi = left_r.atan2(fwd_r);
                let bin = (((phi + AZ_MAX_RAD) * az_scale).round() as isize)
                    .clamp(0, (AZ_BINS - 1) as isize) as u16;
                lut.push(PixelRay {
                    az_bin: bin,
                    tan_el: (up_r / horiz) as f32,
                });
            }
        }
        let mut az_dirs = Vec::with_capacity(AZ_BINS);
        for b in 0..AZ_BINS {
            let phi = -AZ_MAX_RAD + (b as f64) * (2.0 * AZ_MAX_RAD) / (AZ_BINS - 1) as f64;
            az_dirs.push((phi.cos(), phi.sin()));
        }
        Self {
            lut,
            az_dirs,
            outdoor_rail,
            bgr: vec![0u8; IMG_W * IMG_H * 3],
        }
    }

    pub fn render_bgr(&mut self, world: &World, pose: Pose, cam_yaw_rad: f64, hide_cones: bool, show_housing: bool) -> &[u8] {
        let theta = pose.yaw + cam_yaw_rad;
        let (st, ct) = theta.sin_cos();
        let cx = pose.x + CAM_AHEAD_LIDAR_M * pose.yaw.cos();
        let cy = pose.y + CAM_AHEAD_LIDAR_M * pose.yaw.sin();

        // Only board segments belong to blue walls (exclude thin gantry/beam posts at the end of segments).
        // World::new pushes: 4 outer walls + 4 segments for start divider + 4 segments for finish divider = 12 segments.
        let n_board_segs = world.segments.len().min(12);
        let mut wall_dist = [f32::INFINITY; AZ_BINS];
        let mut cone_hit = [ConeColumnHit::default(); AZ_BINS];
        let mut lamp = [(f32::INFINITY,0.0f32); AZ_BINS];

        for b in 0..AZ_BINS {
            let (cf, sl) = self.az_dirs[b];
            // World ray direction: fwd*(ct, st) + left*(-st, ct)
            let dx = ct * cf - st * sl;
            let dy = st * cf + ct * sl;
            if show_housing && dx > 0.001 {
                let dist=(world.gantry_x-cx)/dx;
                let side=cy+dist*dy-world.light_y;
                if dist>0.05 && side.abs()<=0.27 {lamp[b]=(dist as f32,side as f32);}
            }

            let mut best_w = f64::INFINITY;
            for &((x1, y1), (x2, y2)) in &world.segments[..n_board_segs] {
                let (ex, ey) = (x2 - x1, y2 - y1);
                let den = dx * ey - dy * ex;
                if den.abs() < 1e-12 {
                    continue;
                }
                let t = ((x1 - cx) * ey - (y1 - cy) * ex) / den;
                let u = ((x1 - cx) * dy - (y1 - cy) * dx) / den;
                if t > 0.05 && t < best_w && (-1e-6..=1.0 + 1e-6).contains(&u) {
                    best_w = t;
                }
            }
            wall_dist[b] = best_w as f32;
            if hide_cones {
                continue;
            }

            // Intersect with physical tapered cones (keep closest visible cone ahead of the wall).
            let mut best_c = ConeColumnHit {
                dist: f32::INFINITY,
                lat_abs: 0.0,
                kind: 0,
            };
            for (idx, &(kx, ky, _r)) in world.cones.iter().enumerate() {
                let rx = kx - cx;
                let ry = ky - cy;
                let along = rx * dx + ry * dy;
                if along <= 0.14 || along >= best_w {
                    continue;
                }
                let lat = (-rx * dy + ry * dx).abs();
                if lat <= CONE_BASE_R_M {
                    let front_d = (along - (CONE_BASE_R_M * CONE_BASE_R_M - lat * lat).max(0.0).sqrt())
                        .max(0.08) as f32;
                    if front_d < best_c.dist && (front_d as f64) < best_w {
                        best_c = ConeColumnHit {
                            dist: front_d,
                            lat_abs: lat as f32,
                            kind: if idx == 0 { 1 } else { 2 },
                        };
                    }
                }
            }
            cone_hit[b] = best_c;
        }

        let rail_h = if self.outdoor_rail {
            OUTDOOR_RAIL_H_M as f32
        } else {
            0.0
        };
        let board_top = BOARD_TOP_H_M as f32;
        let cam_h = CAM_HEIGHT_M as f32;
        let cone_h = CONE_HEIGHT_M as f32;
        let base_r = CONE_BASE_R_M as f32;
        let taper = (CONE_BASE_R_M - CONE_TOP_R_M) as f32 / cone_h;

        // Realistic outdoor/indoor BGR surface colors matching real camera observations.
        const SKY_BGR: [u8; 3] = [210, 212, 208];
        const FLOOR_BGR: [u8; 3] = [138, 144, 136];
        const RAIL_BGR: [u8; 3] = [182, 184, 182];
        const WALL_BGR: [u8; 3] = [196, 108, 42];
        const CONE_RED_BGR: [u8; 3] = [30, 82, 234];
        const CONE_BLUE_BGR: [u8; 3] = [214, 114, 34];

        for (px_idx, ray) in self.lut.iter().enumerate() {
            let b = ray.az_bin as usize;
            let ch = cone_hit[b];
            let mut color = if ray.tan_el < 0.0 { FLOOR_BGR } else { SKY_BGR };
            let mut depth = f32::INFINITY;

            let dw = wall_dist[b];
            if dw.is_finite() {
                let zw = cam_h + dw * ray.tan_el;
                if zw >= 0.0 && zw <= board_top {
                    color = if zw < rail_h { RAIL_BGR } else { WALL_BGR };
                    depth=dw;
                }
            }

            if ch.kind != 0 {
                let zc = cam_h + ch.dist * ray.tan_el;
                if zc >= 0.0 && zc <= cone_h {
                    let r_at_z = base_r - taper * zc;
                    if ch.lat_abs <= r_at_z {
                        depth=ch.dist;
                        color = if ch.kind == 1 {
                            CONE_RED_BGR
                        } else {
                            CONE_BLUE_BGR
                        };
                    }
                }
            }

            let (ld,side)=lamp[b];
            if ld<depth {
                let z=cam_h+ld*ray.tan_el;
                if (0.43..=0.60).contains(&z) {
                    color=[26,25,24];
                    for (y,lens) in [(0.16,[22,38,183]),(0.0,[8,100,204]),(-0.16,[34,87,29])] {
                        if (side-y)*(side-y)+(z-0.515)*(z-0.515)<=0.046*0.046 { color=lens;break; }
                    }
                }
            }

            let out_i = px_idx * 3;
            self.bgr[out_i] = color[0];
            self.bgr[out_i + 1] = color[1];
            self.bgr[out_i + 2] = color[2];
        }
        &self.bgr
    }
}

pub fn default_pitch_rad() -> f64 {
    NOMINAL_PITCH_RAD
}
