#![forbid(unsafe_code)]
//! Offline benchmark CLI (`route-engine cam-loc-eval`) that drives the exact compiled
//! `Locator`, `extract_visual_frame`, and `race_field_path::Planner` over JSONL stdin/stdout.

use std::io::{BufRead, BufWriter, Write};

use crate::motion_sensors::RaceInputs;
use crate::py::{V, parse_json, to_json};
use crate::race_cam_features::{IMG_H, IMG_W, extract_visual_frame};
use crate::race_field_map::Locator;
use xt_stcar_robot_core::autonomy::Pose2;

pub fn main(_args: &[String]) -> i32 {
    let stdin = std::io::stdin();
    let stdout = std::io::stdout();
    let mut out = BufWriter::with_capacity(1 << 16, stdout.lock());
    let mut loc = Locator::default();
    let mut bgr_buf: Vec<u8> = Vec::with_capacity(IMG_W * IMG_H * 3);
    let mut line = String::new();
    let mut reader = stdin.lock();

    loop {
        line.clear();
        match reader.read_line(&mut line) {
            Ok(0) | Err(_) => break,
            Ok(_) => {}
        }
        let trimmed = line.trim();
        if trimmed.is_empty() {
            continue;
        }
        let req = match parse_json(trimmed) {
            Ok(v) => v,
            Err(e) => {
                let _ = writeln!(out, "{}", to_json(&crate::dict! {"ok" => false, "err" => e}));
                let _ = out.flush();
                continue;
            }
        };
        let resp = match req.get("op").os() {
            Some("reset") => {
                loc = Locator::default();
                crate::dict! {"ok" => true}
            }
            Some("set_state") => {
                let seq = req.get("seq").oi().unwrap_or(0);
                let at = req.get("at").of().unwrap_or(0.0);
                let pose = Pose2 {
                    x_m: req.get("x").of().unwrap_or(0.0),
                    y_m: req.get("y").of().unwrap_or(0.0),
                    yaw_rad: req.get("yaw").of().unwrap_or(0.0),
                };
                let speed = req.get("speed").of().unwrap_or(0.0);
                let yaw_rate = req.get("yaw_rate").of();
                let integral = req.get("integral").of();
                loc.force_state_for_eval(seq, at, pose, speed, yaw_rate, integral);
                if let Some(p) = req.get("pitch_rad").of() {
                    loc.cam_loc.pitch_rad = p;
                }
                if let Some(cy) = req.get("cam_yaw_rad").of() {
                    loc.cam_loc.cam_yaw_rad = cy;
                }
                if let Some(rh) = req.get("rail_height_m").of() {
                    loc.cam_loc.rail_height_m = rh;
                    loc.cam_loc.cal_updates = 10;
                }
                crate::dict! {"ok" => true}
            }
            Some("imu") => {
                let at = req.get("at").of().unwrap_or(0.0);
                let raw = RaceInputs {
                    yaw_rate: req.get("yaw_rate").of(),
                    forward_integral: req.get("forward_integral").of(),
                };
                loc.imu.sample(raw, at);
                crate::dict! {"ok" => true}
            }
            Some("cam_raw") => {
                let Some(path) = req.get("path").os() else {
                    let _ = writeln!(out, "{}", to_json(&crate::dict! {"ok" => false, "err" => "missing path"}));
                    let _ = out.flush();
                    continue;
                };
                let frame = req.get("frame").oi().unwrap_or(0);
                let captured_at = req.get("captured_at").of().unwrap_or(0.0);
                let w = req.get("w").oi().unwrap_or(IMG_W as i64) as usize;
                let h = req.get("h").oi().unwrap_or(IMG_H as i64) as usize;
                bgr_buf.clear();
                match std::fs::File::open(path).and_then(|mut f| std::io::Read::read_to_end(&mut f, &mut bgr_buf)) {
                    Ok(_) if bgr_buf.len() >= w * h * 3 => {
                        let vis = extract_visual_frame(&bgr_buf, w, h, frame, captured_at);
                        let valid_cols = vis.valid_cols as i64;
                        let n_edges = vis.n_vert_edges as i64;
                        let n_cones = vis.n_cones as i64;
                        let rail_frac = vis.rail_fraction as f64;
                        let mean_luma = vis.mean_luma as f64;
                        let compute_us = vis.compute_us as i64;
                        loc.observe_camera(vis, captured_at);
                        crate::dict! {
                            "ok" => true,
                            "valid_cols" => valid_cols,
                            "n_edges" => n_edges,
                            "n_cones" => n_cones,
                            "rail_fraction" => rail_frac,
                            "mean_luma" => mean_luma,
                            "compute_us" => compute_us,
                        }
                    }
                    Ok(_) => crate::dict! {"ok" => false, "err" => "short bgr file"},
                    Err(e) => crate::dict! {"ok" => false, "err" => e.to_string()},
                }
            }
            Some("scan") => {
                let seq = req.get("seq").oi().unwrap_or(0);
                let at = req.get("at").of().unwrap_or(0.0);
                let at_start = req.get("at_start").truthy();
                let raw = RaceInputs {
                    yaw_rate: req.get("yaw_rate").of(),
                    forward_integral: req.get("forward_integral").of(),
                };
                let live = if req.get("use_imu_filter").truthy() {
                    loc.imu.sample(raw, at)
                } else {
                    raw
                };
                let mut scan = crate::dict! {
                    "seq" => seq,
                    "received_at" => at,
                    "ranges" => req.get("ranges").clone(),
                };
                if !req.get("point_age_ms").is_none() {
                    scan.set("point_age_ms", req.get("point_age_ms").clone());
                }
                loc.observe(&scan, at, live, at_start);
                let diag = loc.diagnostics(&scan);
                let aim_json = |side: usize| -> V {
                    if let Some(a) = loc.planner.aim(seq, side) {
                        crate::dict! {
                            "pwm" => a.pwm,
                            "curvature" => a.curvature,
                            "point" => vec![a.point.0, a.point.1],
                            "before" => a.before,
                            "lookahead" => a.lookahead,
                        }
                    } else {
                        V::None
                    }
                };
                crate::dict! {
                    "ok" => true,
                    "localized" => diag.get("localized").truthy(),
                    "pose" => diag.get("pose_xy_yaw").clone(),
                    "raw_pose" => loc.raw_pose_xy_yaw().map(|p| vec![p.0, p.1, p.2]),
                    "speed" => diag.get("speed_mps").clone(),
                    "support" => diag.get("wall_points_xy").clone(),
                    "cam_anchored" => diag.get("cam_anchored_xy").clone(),
                    "cam_used" => diag.get("cam_used").truthy(),
                    "cam_inliers" => diag.get("cam_inliers").clone(),
                    "cam_rms_px" => diag.get("cam_rms_px").clone(),
                    "cam_solve_us" => diag.get("cam_solve_us").clone(),
                    "cam_pitch_deg" => diag.get("cam_pitch_deg").clone(),
                    "cam_yaw_deg" => diag.get("cam_yaw_deg").clone(),
                    "cam_rail_h_m" => diag.get("cam_rail_h_m").clone(),
                    "aim0" => aim_json(0),
                    "aim1" => aim_json(1),
                }
            }
            _ => crate::dict! {"ok" => false, "err" => "unknown op"},
        };
        let mut s = to_json(&resp);
        s.push('\n');
        if out.write_all(s.as_bytes()).is_err() || out.flush().is_err() {
            break;
        }
    }
    0
}
