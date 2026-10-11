//! Host-side scan geometry for offline analysis and simulation (never on the car).
//!
//! Reads JSON lines with "ranges" (360 calibrated car-frame bins, clockwise from
//! forward, null for no return) and "at_ms", and writes each line back with the
//! same native fields the vehicle bridge adds after calibration:
//! front_boundary_m, left_junction, corridor_candidates, wall_candidates,
//! left_turn_goal. Usage:
//!   cargo run --release -q -p xt-stcar-robot-core --example scan_geometry < scans.jsonl
use std::io::{self, BufRead, Write};
use xt_stcar_robot_core::{
    FrameId, LidarSample, Timestamp,
    junction::{
        detect_corridor_candidates, detect_front_boundary, detect_left_junction,
        detect_turn_geometry,
    },
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let stdin = io::stdin();
    let mut out = io::stdout().lock();
    for line in stdin.lock().lines() {
        let line = line?;
        if line.trim().is_empty() {
            continue;
        }
        let mut row: serde_json::Value = serde_json::from_str(&line)?;
        let frame = FrameId("lidar_origin_coarse_body_heading".into());
        let sample = LidarSample {
            captured_at: Timestamp(row["at_ms"].as_u64().unwrap_or(0)),
            frame_id: frame.clone(),
            angle_min_rad: 0.0,
            angle_increment_rad: -std::f64::consts::TAU / 360.0,
            range_min_m: 0.02,
            range_max_m: 12.0,
            ranges_m: serde_json::from_value(row["ranges"].clone())?,
        };
        row["front_boundary_m"] =
            serde_json::to_value(detect_front_boundary(&sample, &frame).ok().flatten())?;
        row["left_junction"] =
            serde_json::to_value(detect_left_junction(&sample, &frame).ok().flatten())?;
        row["corridor_candidates"] = serde_json::to_value(
            detect_corridor_candidates(&sample, &frame).unwrap_or_default(),
        )?;
        let turn_geometry = detect_turn_geometry(&sample, &frame).unwrap_or_default();
        row["wall_candidates"] = serde_json::to_value(turn_geometry.walls)?;
        row["left_turn_goal"] = serde_json::to_value(turn_geometry.left_goal)?;
        serde_json::to_writer(&mut out, &row)?;
        writeln!(out)?;
        out.flush()?;
    }
    Ok(())
}
