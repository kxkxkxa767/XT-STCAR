//! Compare both trackers through the same synthetic competition controller.
//! Prints outcomes, including failures; a successful process is not race acceptance.
use serde::Serialize;
use xt_stcar_robot_core::tracking::TrackingConfig;
use xt_stcar_robot_runner::simulation::{SimulationConfig, SimulationSummary, simulate};

#[derive(Serialize)]
struct Run {
    tracking: TrackingConfig,
    summary: Option<SimulationSummary>,
    error: Option<String>,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let choices = [
        TrackingConfig::PurePursuit,
        TrackingConfig::Lqr {
            q_lateral: 4.0,
            q_heading: 2.0,
            r_curvature: 1.0,
            min_speed_mps: 0.03,
            max_heading_error_rad: 0.7,
            max_lateral_error_m: 0.5,
        },
    ];
    let runs: Vec<_> = choices
        .into_iter()
        .map(|tracking| {
            let mut config = SimulationConfig::example();
            config.autonomy.navigation.tracking = tracking;
            let (summary, error) = match simulate(&config, &mut std::io::sink()) {
                Ok(summary) => (Some(summary), None),
                Err(error) => (None, Some(error)),
            };
            Run {
                tracking,
                summary,
                error,
            }
        })
        .collect();
    serde_json::to_writer_pretty(
        std::io::stdout().lock(),
        &serde_json::json!({
            "schema_version": 1,
            "scope": "Identical synthetic RGB/laser/ideal-pose competition scene; no hardware or target timing measurement",
            "physical_output_enabled": false,
            "runs": runs,
        }),
    )?;
    println!();
    Ok(())
}
