//! Native production pipeline probe: the supplied eight scenes plus independent
//! cone offsets and bounded range noise. Truth enters raycasting/referee only.
use serde_json::json;
use std::io::BufWriter;
use xt_stcar_robot_runner::async_simulation::{AsyncSimulationOptions, simulate_async_observed};
use xt_stcar_robot_runner::online::OnlineControlConfig;
use xt_stcar_robot_runner::online_simulation::{OnlineScenario, VisualOcclusion};
use xt_stcar_robot_runner::simulation::simulate_observed;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mode = args.first().map_or("both", String::as_str);
    let selected = args.get(1).map_or("all", String::as_str);
    let directory = args.get(2).map(std::path::PathBuf::from);
    if args.len() > 3 || !["sync", "async", "async-delay", "both"].contains(&mode) {
        return Err(
            "usage: lidar_matrix [sync|async|async-delay|both] [case|all] [evidence-directory]"
                .into(),
        );
    }
    if let Some(path) = &directory {
        std::fs::create_dir_all(path)?;
    }
    // name, length, width, bottom/top lanes, right x/y, left x/y, crosswalk x.
    let scenes = [
        ("small_5x4", 5., 4., 1., 1., 4., 2., 1., 2., 1.8),
        ("tall_5x6", 5., 6., 1.2, 1.2, 4., 3., 1., 3.5, 2.),
        ("wide_8x4", 8., 4., 1., 1., 6.5, 2., 1.5, 2., 2.5),
        ("large_8x6", 8., 6., 1.2, 1.2, 6.5, 3., 1.5, 3.5, 2.5),
        ("nominal_7x5", 7., 5., 1.2, 1.2, 5.5, 2.5, 1.5, 2.5, 2.5),
        (
            "unequal_bottom1_top2",
            7.,
            6.,
            1.,
            2.,
            5.5,
            3.,
            1.5,
            3.5,
            2.5,
        ),
        (
            "unequal_bottom2_top1",
            7.,
            6.,
            2.,
            1.,
            5.5,
            3.,
            1.5,
            3.5,
            2.5,
        ),
        ("cones_shifted", 7., 5., 1.2, 1.2, 5.2, 2.7, 1.8, 2.3, 2.5),
        ("offset_outward", 7., 5., 1.2, 1.2, 5.8, 2.3, 1.2, 2.7, 2.5),
        (
            "offset_both_forward",
            7.,
            5.,
            1.2,
            1.2,
            5.8,
            2.5,
            1.8,
            2.5,
            2.5,
        ),
        (
            "offset_both_back",
            7.,
            5.,
            1.2,
            1.2,
            5.2,
            2.5,
            1.2,
            2.5,
            2.5,
        ),
        ("offset_both_up", 7., 5., 1.2, 1.2, 5.5, 2.8, 1.5, 2.8, 2.5),
        (
            "offset_both_down",
            7.,
            5.,
            1.2,
            1.2,
            5.5,
            2.2,
            1.5,
            2.2,
            2.5,
        ),
        ("nominal_noise", 7., 5., 1.2, 1.2, 5.5, 2.5, 1.5, 2.5, 2.5),
        ("shifted_noise", 7., 5., 1.2, 1.2, 5.2, 2.7, 1.8, 2.3, 2.5),
    ];
    if selected != "all" && !scenes.iter().any(|s| s.0 == selected) {
        return Err("unknown scene".into());
    }
    let mut results = Vec::new();
    for (name, length, width, bottom, top, rx, ry, lx, ly, crosswalk) in scenes {
        if selected != "all" && selected != name {
            continue;
        }
        let mut scene = OnlineScenario::example();
        let spec = &mut scene.scene.spec;
        spec.length_m = length;
        spec.width_m = width;
        spec.bottom_lane_width_m = bottom;
        spec.top_lane_width_m = top;
        spec.bottom_straight_span_m = length - 2.;
        spec.top_straight_span_m = length - 2.;
        spec.right_cone_from_right_m = length - rx;
        spec.right_cone_from_bottom_m = ry;
        spec.left_cone_from_left_m = lx;
        spec.left_cone_from_top_m = width - ly;
        spec.light_front_x_m = length - 0.8;
        scene.scene.crosswalk_near_x_m = crosswalk;
        scene.occlusions.push(VisualOcclusion {
            from_ms: 0,
            through_ms: 180000,
            cones: true,
            markers: false,
        });
        let mut config = scene.compile()?;
        config.autonomy.online = Some(OnlineControlConfig::lidar_first(&config.autonomy));
        config
            .online_scene
            .as_mut()
            .unwrap()
            .ideal_non_cone_semantics = true;
        if name.ends_with("_noise") {
            config.online_scene.as_mut().unwrap().lidar_range_noise_m = 0.002;
        }
        config.validate()?;
        if let Some(path) = &directory {
            std::fs::write(
                path.join(format!("{name}.input.json")),
                serde_json::to_vec_pretty(&config)?,
            )?;
        }
        for schedule in ["sync", "async"] {
            if mode != "both" && mode != schedule && !(mode == "async-delay" && schedule == "async")
            {
                continue;
            }
            let schedule = if mode == "async-delay" {
                "async-delay"
            } else {
                schedule
            };
            let mut output: Box<dyn std::io::Write> = match &directory {
                Some(path) => Box::new(BufWriter::new(std::fs::File::create(
                    path.join(format!("{name}.{schedule}.jsonl")),
                )?)),
                None => Box::new(std::io::sink()),
            };
            let mut trajectory = Vec::new();
            let mut omitted_trajectory_samples = 0usize;
            let result = if schedule == "sync" {
                simulate_observed(&config, &mut output, |step, pose, speed, curvature| {
                    if directory.is_some() && trajectory.len() < 2048 {
                        trajectory.push(json!({"at":step.at, "pose":pose, "speed":speed,
                            "curvature":curvature, "command":step.command, "online":step.online,
                            "navigation":step.navigation.as_ref().map(|n| json!({"reason":n.reason,
                                "diagnostics":n.diagnostics}))}));
                    } else if directory.is_some() {
                        omitted_trajectory_samples += 1;
                    }
                })
                .map(|s| serde_json::to_value(s).unwrap())
            } else {
                let mut timing = AsyncSimulationOptions::default();
                if schedule == "async-delay" {
                    timing.input_delays_ms = [40, 60];
                    timing.adoption_delays_ms = [5, 9, 11];
                }
                let mut last_plan = None;
                simulate_async_observed(&config, &timing, &mut output, |poll, pose, speed, curvature| {
                    if directory.is_none() { return; }
                    if let Some(plan) = &poll.observed_plan && last_plan != Some(plan.planned_at) {
                        last_plan = Some(plan.planned_at);
                        if trajectory.len() >= 2048 {
                            omitted_trajectory_samples += 1;
                            return;
                        }
                        let step = &plan.report;
                        trajectory.push(json!({"at":poll.at, "source_at":plan.source_at,
                            "pose":pose,"speed":speed,"curvature":curvature,"command":poll.command,
                            "online":step.online,"navigation":step.navigation.as_ref().map(|n|json!({
                                "reason":n.reason,"diagnostics":n.diagnostics}))}));
                    }
                })
                    .map(|s| serde_json::to_value(s).unwrap())
            };
            output.flush()?;
            if let Some(path) = &directory {
                std::fs::write(
                    path.join(format!("{name}.{schedule}.trajectory.json")),
                    serde_json::to_vec(&trajectory)?,
                )?;
            }
            let record = match result {
                Ok(summary) => json!({"name":name,"mode":schedule,
                    "accepted": accepted(&summary), "summary":summary,
                    "omitted_trajectory_samples": omitted_trajectory_samples}),
                Err(error) => json!({"name":name,"mode":schedule,"error":error}),
            };
            eprintln!(
                "{name}/{schedule}: completed={} fault={}",
                record["summary"]["completed"], record["summary"]["fault"]
            );
            results.push(record);
        }
    }
    let all_accepted = results.iter().all(|r| r["accepted"] == true);
    let report = json!({"probe":"native Rust lidar-only cones; ideal non-cone semantics; unchanged navigation/safety limits", "all_accepted":all_accepted, "results":results});
    println!("{}", serde_json::to_string_pretty(&report)?);
    if all_accepted {
        Ok(())
    } else {
        Err("one or more scenes failed independent acceptance; see JSON results".into())
    }
}

fn accepted(summary: &serde_json::Value) -> bool {
    summary["completed"] == true
        && summary["fault"].is_null()
        && summary["physical_output_enabled"] == false
        && summary["online_referee"]["both_completed"] == true
        && summary["online_referee"]["order_valid"] == true
        && summary["online_referee"]["trajectory_valid"] == true
        && summary["final_actual_speed_mps"].as_f64() == Some(0.0)
        && summary["final_actual_curvature_per_m"].as_f64() == Some(0.0)
        && summary["crosswalk_hold_ms"]
            .as_u64()
            .is_some_and(|ms| ms >= 3000)
        && summary["green_observed_ms"]
            .as_u64()
            .is_some_and(|ms| ms >= 300)
}
