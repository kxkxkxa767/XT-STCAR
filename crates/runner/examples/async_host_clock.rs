//! Explicit wall-clock observation; intentionally not a load-sensitive unit test.
use xt_stcar_robot_runner::host_clock_simulation::observe_host_clock;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let report = observe_host_clock()?;
    let passed = report["drive_polls"].as_u64().unwrap_or(0) > 0
        && report["turning_drive_polls"].as_u64().unwrap_or(0) > 0
        && report["first_fault"]["fault"] == "command_expired"
        && report["final_speed_mps"] == 0.0
        && report["final_curvature_per_m"] == 0.0
        && report["collision_or_boundary_violation"] == false;
    serde_json::to_writer_pretty(std::io::stdout().lock(), &report)?;
    println!();
    if !passed {
        return Err("host observation failed; inspect the preserved report".into());
    }
    Ok(())
}
