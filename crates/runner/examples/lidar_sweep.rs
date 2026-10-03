//! One arbitrary offline scene per process, for resumable external batch scheduling.
use serde_json::json;
use std::io::Write;
use xt_stcar_robot_runner::async_simulation::{AsyncSimulationOptions, simulate_async_observed};
use xt_stcar_robot_runner::simulation::simulate_observed;
use xt_stcar_robot_runner::sweep_simulation::{SweepScene, accepted};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    if args.first().map(String::as_str) == Some("example") {
        println!("{}", serde_json::to_string_pretty(&SweepScene::example())?);
        return Ok(());
    }
    if args.len() != 3 || !["sync", "async", "async-delay", "validate"].contains(&args[0].as_str())
    {
        return Err("usage: lidar_sweep example | [sync|async|async-delay|validate] input.json evidence-dir".into());
    }
    let data = std::fs::read(&args[1])?;
    if data.len() > 1024 * 1024 {
        return Err("sweep input exceeds 1 MiB".into());
    }
    let input: SweepScene = serde_json::from_slice(&data)?;
    let config = match input.compile() {
        Ok(config) => config,
        Err(error) => {
            println!(
                "{}",
                json!({"status":"input_incompatible","accepted":false,"error":error,"input":input})
            );
            return Err("scene construction rejected; not a physical impossibility proof".into());
        }
    };
    if args[0] == "validate" {
        println!(
            "{}",
            json!({"status":"validated_not_simulated","input":input})
        );
        return Ok(());
    }
    let dir = std::path::Path::new(&args[2]);
    std::fs::create_dir_all(dir)?;
    std::fs::write(dir.join("input.json"), serde_json::to_vec_pretty(&input)?)?;
    std::fs::write(
        dir.join("compiled.json"),
        serde_json::to_vec_pretty(&config)?,
    )?;
    let mut log = std::io::BufWriter::new(std::fs::File::create(dir.join("events.jsonl"))?);
    let mut trajectory = Vec::new();
    let mut omitted = 0usize;
    let mut last_at = None;
    let mut observe = |at: u64, pose, speed, curvature| {
        if last_at.is_some_and(|old| at < old + 200) {
            return;
        }
        last_at = Some(at);
        if trajectory.len() < 2048 {
            trajectory.push(
                json!({"at_ms":at,"pose":pose,"speed_mps":speed,"curvature_per_m":curvature}),
            );
        } else {
            omitted += 1;
        }
    };
    let result = if args[0] == "sync" {
        simulate_observed(&config, &mut log, |step, pose, speed, curvature| {
            observe(step.at.0, pose, speed, curvature)
        })
        .map(|s| serde_json::to_value(s).unwrap())
    } else {
        let mut timing = AsyncSimulationOptions::default();
        if args[0] == "async-delay" {
            timing.input_delays_ms = [40, 60];
            timing.adoption_delays_ms = [5, 9, 11];
        }
        simulate_async_observed(
            &config,
            &timing,
            &mut log,
            |poll, pose, speed, curvature| observe(poll.at.0, pose, speed, curvature),
        )
        .map(|s| serde_json::to_value(s).unwrap())
    };
    log.flush()?;
    std::fs::write(
        dir.join("trajectory.json"),
        serde_json::to_vec(&json!({"samples":trajectory,"omitted":omitted}))?,
    )?;
    let record = match result {
        Ok(summary) => {
            json!({"status":"finished","accepted":accepted(&summary),"mode":args[0],"summary":summary,"scope":"ideal scoped non-cone semantics, ideal localization, lidar-only cones; optional corridor walls are a hypothesis"})
        }
        Err(error) => json!({"status":"runtime_error","accepted":false,"error":error}),
    };
    println!("{}", serde_json::to_string(&record)?);
    if record["accepted"] == true {
        Ok(())
    } else {
        Err("scene did not pass independent acceptance".into())
    }
}
