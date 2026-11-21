//! Fixed 640 x 480 camera and its ground plane in the lidar frame.
use crate::helpers::FRONT_BODY_EXTENT_M;

pub(crate) const FX: f64 = 426.55;
pub(crate) const FY: f64 = 424.47;
pub(crate) const CX: f64 = 349.59;
pub(crate) const CY: f64 = 227.85;
pub(crate) const K1: f64 = -0.371764;
pub(crate) const K2: f64 = 0.172259;
pub(crate) const P1: f64 = -0.000092;
pub(crate) const P2: f64 = -0.000226;
pub(crate) const K3: f64 = -0.042575;
pub(crate) const CAM_H: f64 = 0.195;
pub(crate) const CAM_BEHIND_FRONT: f64 = 0.041;
pub(crate) const CAM_AHEAD_LIDAR: f64 = FRONT_BODY_EXTENT_M - CAM_BEHIND_FRONT; // 0.159 m

/// Forward project ground point (xc forward from camera, yc left of camera) to distorted pixel (u, v).
/// Camera optical axis is tilted UP by +pitch_rad above horizontal (matching scripts/vision-log.py).
pub(crate) fn project_cam_ground(xc: f64, yc: f64, pitch_rad: f64) -> Option<(f64, f64)> {
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

pub(crate) fn undistort_pt(u: f64, v: f64) -> Option<(f64, f64)> {
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

/// Invert the calibrated upward-pitched pinhole ray at the ground plane.
pub(crate) fn ground_point(normal: (f64, f64), pitch: f64) -> Option<(f64, f64)> {
    let (sin, cos) = pitch.sin_cos();
    let down = normal.1 * cos - sin;
    if down <= 0. {
        return None;
    }
    let depth = CAM_H / down;
    let forward = depth * (cos + normal.1 * sin) + CAM_AHEAD_LIDAR;
    Some((forward, -normal.0 * depth))
}
