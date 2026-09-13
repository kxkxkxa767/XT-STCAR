//! Explicit complete functional comparison through the actual asynchronous worker.
//! cargo run --release -p xt-stcar-robot-runner --example async_motion_comparison -- [pp|lqr|both] [timing.json]
use xt_stcar_robot_core::tracking::TrackingConfig;
use xt_stcar_robot_runner::async_simulation::{AsyncSimulationOptions, simulate_async};
use xt_stcar_robot_runner::input::read_regular_file;
use xt_stcar_robot_runner::simulation::SimulationConfig;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let choice = args.next().unwrap_or_else(|| "both".into());
    let timing: AsyncSimulationOptions = if let Some(path) = args.next() {
        serde_json::from_slice(&read_regular_file(std::path::Path::new(&path), 64 * 1024)?)?
    } else {
        AsyncSimulationOptions::default()
    };
    if !matches!(choice.as_str(), "pp" | "lqr" | "both") || args.next().is_some() {
        return Err("usage: async_motion_comparison [pp|lqr|both] [timing.json]".into());
    }
    let trackers = [
        ("pp", TrackingConfig::PurePursuit),
        (
            "lqr",
            TrackingConfig::Lqr {
                q_lateral: 4.0,
                q_heading: 2.0,
                r_curvature: 1.0,
                min_speed_mps: 0.03,
                max_heading_error_rad: 0.7,
                max_lateral_error_m: 0.5,
            },
        ),
    ];
    let mut runs = Vec::new();
    let mut all_completed = true;
    for (name, tracker) in trackers {
        if choice != "both" && choice != name {
            continue;
        }
        let mut config = SimulationConfig::example();
        config.autonomy.navigation.tracking = tracker;
        let summary = simulate_async(&config, &timing, &mut std::io::sink())?;
        all_completed &= summary.completed;
        runs.push(serde_json::json!({"tracker":name,"config":config,"summary":summary}));
    }
    serde_json::to_writer_pretty(
        std::io::stdout().lock(),
        &serde_json::json!({
            "schema_version":1,"physical_output_enabled":false,
            "scope":"Full real AutonomyWorker functional simulation. Host waits freeze logical time; not deadline performance or hardware acceptance.",
            "all_completed":all_completed,"runs":runs,
        }),
    )?;
    println!();
    if !all_completed {
        return Err(
            "async functional run did not complete; inspect the recorded real failure".into(),
        );
    }
    Ok(())
}
