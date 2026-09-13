//! Opt-in host wall-clock measurements of the complete synthetic simulation.
//! Run with: cargo run --release -p xt-stcar-robot-runner --example motion_profile -- 3
//! The optional repetition count is bounded to 1..=20. No physical output exists.
use serde::Serialize;
use std::time::Instant;
use xt_stcar_robot_core::tracking::TrackingConfig;
use xt_stcar_robot_runner::simulation::{SimulationConfig, SimulationSummary, simulate};

#[derive(Serialize)]
struct Sample {
    wall_time_ms: f64,
    simulated_elapsed_ms: Option<u64>,
    control_ticks: Option<usize>,
    completed: bool,
    distance_m: Option<f64>,
    final_actual_speed_mps: Option<f64>,
    error: Option<String>,
}

fn measure(config: &SimulationConfig) -> (Sample, Option<SimulationSummary>) {
    let start = Instant::now();
    let result = simulate(config, &mut std::io::sink());
    let wall_time_ms = start.elapsed().as_secs_f64() * 1000.0;
    match result {
        Ok(summary) => (
            Sample {
                wall_time_ms,
                simulated_elapsed_ms: Some(summary.elapsed_ms),
                control_ticks: Some(summary.ticks),
                completed: summary.completed,
                distance_m: Some(summary.distance_m),
                final_actual_speed_mps: Some(summary.final_actual_speed_mps),
                error: summary.fault.clone(),
            },
            Some(summary),
        ),
        Err(error) => (
            Sample {
                wall_time_ms,
                simulated_elapsed_ms: None,
                control_ticks: None,
                completed: false,
                distance_m: None,
                final_actual_speed_mps: None,
                error: Some(error),
            },
            None,
        ),
    }
}

#[derive(Serialize)]
struct Profile {
    config: SimulationConfig,
    /// One warm-up run, reported separately and excluded from timing aggregates.
    warmup: Sample,
    samples: Vec<Sample>,
    total_wall_time_ms: f64,
    minimum_wall_time_ms: f64,
    maximum_wall_time_ms: f64,
    mean_wall_time_ms: f64,
    /// Bounded phase, candidate, route and terminal-work details of the final run.
    last_summary: Option<SimulationSummary>,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let repetitions: usize = args.next().as_deref().unwrap_or("3").parse()?;
    if !(1..=20).contains(&repetitions) || args.next().is_some() {
        return Err("usage: motion_profile [repetitions: 1..=20]".into());
    }
    if cfg!(debug_assertions) {
        return Err("host performance measurements require cargo run --release".into());
    }
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
    let mut profiles = Vec::with_capacity(choices.len());
    for tracking in choices {
        let mut config = SimulationConfig::example();
        config.autonomy.navigation.tracking = tracking;
        let (warmup, _) = measure(&config);
        let mut samples = Vec::with_capacity(repetitions);
        let mut last_summary = None;
        for _ in 0..repetitions {
            let (sample, summary) = measure(&config);
            samples.push(sample);
            last_summary = summary;
        }
        let total_wall_time_ms = samples.iter().map(|s| s.wall_time_ms).sum::<f64>();
        let minimum_wall_time_ms = samples
            .iter()
            .map(|s| s.wall_time_ms)
            .fold(f64::INFINITY, f64::min);
        let maximum_wall_time_ms = samples.iter().map(|s| s.wall_time_ms).fold(0.0, f64::max);
        profiles.push(Profile {
            config,
            warmup,
            samples,
            total_wall_time_ms,
            minimum_wall_time_ms,
            maximum_wall_time_ms,
            mean_wall_time_ms: total_wall_time_ms / repetitions as f64,
            last_summary,
        });
    }
    let all_completed = profiles.iter().all(|p| {
        p.warmup.completed
            && p.warmup.error.is_none()
            && p.samples.iter().all(|s| s.completed && s.error.is_none())
    });
    serde_json::to_writer_pretty(
        std::io::stdout().lock(),
        &serde_json::json!({
            "schema_version": 1,
            "scope": "Host wall time for complete simulate calls, including synthetic RGB/laser generation, control, plant and sink telemetry; excludes report serialization. Not solver-only CPU time, per-tick latency, hardware timing or a worst-case deadline guarantee.",
            "physical_output_enabled": false,
            "host_os": std::env::consts::OS,
            "host_arch": std::env::consts::ARCH,
            "build": "release",
            "repetitions_per_tracker": repetitions,
            "warmup_runs_per_tracker": 1,
            "all_completed": all_completed,
            "profiles": profiles,
        }),
    )?;
    println!();
    if !all_completed {
        return Err("one or more synthetic runs failed; inspect the emitted report".into());
    }
    Ok(())
}
