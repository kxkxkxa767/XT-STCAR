//! Predeclared scene changes with one sensor-only PP controller configuration.
use serde::Serialize;
use serde_json::json;
use xt_stcar_robot_runner::async_simulation::{AsyncSimulationOptions, simulate_async_observed};
use xt_stcar_robot_runner::online_simulation::{OnlineScenario, VisualOcclusion};
use xt_stcar_robot_runner::simulation::simulate;

#[derive(Clone, Serialize)]
struct Case {
    name: &'static str,
    scene: OnlineScenario,
    expected_completion: bool,
    perturbed: bool,
}

fn cases() -> Vec<Case> {
    let nominal = OnlineScenario::example();
    let base = |name, scene| Case {
        name,
        scene,
        expected_completion: true,
        perturbed: false,
    };
    let mut shifted = nominal.clone();
    shifted.scene.spec.left_cone_from_left_m = 1.8;
    shifted.scene.spec.right_cone_from_right_m = 1.8;
    shifted.scene.spec.left_cone_from_top_m = 2.3;
    shifted.scene.spec.right_cone_from_bottom_m = 2.3;
    let mut small = nominal.clone();
    small.scene.spec.length_m = 5.0;
    small.scene.spec.width_m = 4.0;
    small.scene.spec.bottom_lane_width_m = 1.0;
    small.scene.spec.top_lane_width_m = 1.0;
    small.scene.spec.bottom_straight_span_m = 3.0;
    small.scene.spec.top_straight_span_m = 3.0;
    small.scene.spec.left_cone_from_left_m = 1.0;
    small.scene.spec.right_cone_from_right_m = 1.0;
    small.scene.spec.left_cone_from_top_m = 2.0;
    small.scene.spec.right_cone_from_bottom_m = 2.0;
    small.scene.spec.light_front_x_m = 3.6;
    small.scene.crosswalk_near_x_m = 1.8;
    let mut unequal = nominal.clone();
    unequal.scene.spec.width_m = 6.0;
    unequal.scene.spec.bottom_lane_width_m = 1.0;
    unequal.scene.spec.top_lane_width_m = 2.0;
    unequal.scene.spec.left_cone_from_top_m = 2.5;
    let mut crosswalk = nominal.clone();
    crosswalk.scene.crosswalk_near_x_m = 3.0;
    let mut occlusion = nominal.clone();
    occlusion.occlusions.push(VisualOcclusion {
        from_ms: 20_000,
        through_ms: 20_400,
        cones: true,
        markers: false,
    });
    let mut no_markers = nominal.clone();
    no_markers.experimental_markers_enabled = false;
    let mut no_cones = nominal.clone();
    no_cones.occlusions.push(VisualOcclusion {
        from_ms: 0,
        through_ms: 180_000,
        cones: true,
        markers: false,
    });
    let mut result = vec![
        base("nominal", nominal),
        base("cones_shifted", shifted),
        base("small_5x4", small),
        base("unequal_1x2", unequal),
        base("crosswalk_shifted", crosswalk),
        base("short_cone_occlusion", occlusion),
        Case {
            name: "missing_markers",
            scene: no_markers,
            expected_completion: false,
            perturbed: false,
        },
        Case {
            name: "missing_cones",
            scene: no_cones,
            expected_completion: false,
            perturbed: false,
        },
    ];
    for index in [0, 2, 3] {
        let mut c = result[index].clone();
        c.name = match index {
            0 => "nominal_delay",
            2 => "small_delay",
            _ => "unequal_delay",
        };
        c.perturbed = true;
        result.push(c);
    }
    result
}

/// These are output gates, never inputs to the controller or scene compiler.
fn failures(summary: &serde_json::Value, case: &Case) -> Vec<&'static str> {
    let mut failures = Vec::new();
    let completed = summary["completed"].as_bool();
    let fault = summary["fault"].as_str();
    let phase_seen = |phase: &str| {
        summary["phases"]
            .as_array()
            .is_some_and(|phases| phases.iter().any(|p| p.as_str() == Some(phase)))
    };
    for field in ["final_actual_speed_mps", "final_actual_curvature_per_m"] {
        if !summary[field]
            .as_f64()
            .is_some_and(|value| value.is_finite() && value.abs() <= 1e-12)
        {
            failures.push("actual vehicle did not stop and center steering");
        }
    }
    if !summary["minimum_cone_clearance_m"]
        .as_f64()
        .is_some_and(|value| value.is_finite() && value >= 0.0)
    {
        failures.push("missing or negative independent physical clearance");
    }
    if summary["crosswalk_hold_ms"].as_u64().unwrap_or(0) < 3000 {
        failures.push("crosswalk stationary hold was not completed");
    }
    let referee = &summary["online_referee"];
    if referee["order_valid"].as_bool() != Some(true)
        || referee["trajectory_valid"].as_bool() != Some(true)
    {
        failures.push("independent cone trajectory or order evidence is invalid");
    }
    if (case.expected_completion || case.name == "missing_markers")
        && referee["both_completed"].as_bool() != Some(true)
    {
        failures.push("independent actual trajectory did not prove both cone passages");
    }
    if case.name == "missing_cones" && referee["completed_cones"].as_u64() != Some(0) {
        failures.push("missing cones case lacks independent zero-passage evidence");
    }
    if case.expected_completion {
        if completed != Some(true) || !summary["fault"].is_null() {
            failures.push("expected complete run without fault");
        }
        if summary["green_observed_ms"].as_u64().unwrap_or(0) < 300 {
            failures.push("fresh stationary green confirmation was not completed");
        }
        for phase in [
            "cones",
            "approach_light",
            "wait_green",
            "finish",
            "completed",
        ] {
            if !phase_seen(phase) {
                failures.push("required task phase absent");
            }
        }
    } else {
        if completed != Some(false) || !fault.is_some_and(|f| {
            f == "current element lost beyond bounded reacquisition time"
                || f.contains("ordinary stop")
                || f == "bounded element search exhausted without an identified next task element"
        }) {
            failures.push("missing evidence did not produce a bounded task stop");
        }
        let required_phase = if case.name == "missing_markers" {
            "approach_light"
        } else {
            "cones"
        };
        if !phase_seen(required_phase) || phase_seen("finish") || phase_seen("completed") {
            failures.push("missing evidence case failed before its intended task or bypassed it");
        }
        if case.name == "missing_cones" && phase_seen("approach_light") {
            failures.push("invisible cones were incorrectly processed");
        }
    }
    failures
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mode = args.first().map_or("both", String::as_str);
    let selected = args.get(1).map_or("all", String::as_str);
    if args.len() > 2 || !["cases", "sync", "async", "async-trace", "both"].contains(&mode) {
        return Err("usage: online_matrix [cases|sync|async|async-trace|both] [case|all]".into());
    }
    let cases = cases();
    if mode == "cases" {
        println!("{}", serde_json::to_string_pretty(&cases)?);
        return Ok(());
    }
    if selected != "all" && !cases.iter().any(|c| c.name == selected) {
        return Err("unknown online case".into());
    }
    let mut results = Vec::new();
    let mut all_passed = true;
    for case in cases
        .iter()
        .filter(|c| selected == "all" || c.name == selected)
    {
        let config = match case.scene.compile() {
            Ok(c) => c,
            Err(error) => {
                all_passed = false;
                results.push(json!({"name":case.name,"compilation_error":error,"passed":false}));
                continue;
            }
        };
        for schedule in ["sync", "async"]
            .into_iter()
            .filter(|m| mode == "both" || mode == *m || (mode == "async-trace" && *m == "async"))
        {
            if case.perturbed && schedule == "sync" {
                continue;
            }
            let mut trace = Vec::new();
            let mut omitted_plans = 0;
            let mut last_plan = None;
            let result = if schedule == "sync" {
                simulate(&config, &mut std::io::sink()).map(|s| serde_json::to_value(s).unwrap())
            } else {
                let mut timing = AsyncSimulationOptions::default();
                if case.perturbed {
                    timing.input_delays_ms = [40, 60];
                    timing.adoption_delays_ms = [5, 9, 11];
                }
                simulate_async_observed(&config, &timing, &mut std::io::sink(), |poll, pose, speed, curvature| {
                    if mode != "async-trace" {
                        return;
                    }
                    let Some(plan) = &poll.observed_plan else { return; };
                    if last_plan == Some(plan.planned_at) { return; }
                    last_plan = Some(plan.planned_at);
                    if trace.len() == 2048 {
                        omitted_plans += 1;
                        return;
                    }
                    trace.push(json!({"at":poll.at,"source_at":plan.source_at,
                        "pose":pose,"speed_mps":speed,"curvature_per_m":curvature,"online":plan.report.online,"command":poll.command,
                        "navigation_status":plan.report.navigation.as_ref().map(|n|&n.status),
                        "navigation_diagnostics":plan.report.navigation.as_ref().map(|n|&n.diagnostics),
                        "navigation_reason":plan.report.navigation.as_ref().and_then(|n|n.reason.as_ref())}));
                })
                    .map(|s| serde_json::to_value(s).unwrap())
            };
            match result {
                Ok(summary) => {
                    let failed_gates = failures(&summary, case);
                    let passed = failed_gates.is_empty();
                    all_passed &= passed;
                    results.push(json!({"name":case.name,"mode":schedule,"expected_completion":case.expected_completion,"passed":passed,"failed_gates":failed_gates,"summary":summary}));
                    if mode == "async-trace" {
                        let result = results.last_mut().unwrap();
                        result["offline_plan_trace"] = json!(trace);
                        result["omitted_trace_plans"] = json!(omitted_plans);
                    }
                }
                Err(error) => {
                    all_passed = false;
                    results.push(
                        json!({"name":case.name,"mode":schedule,"passed":false,"error":error}),
                    );
                }
            }
        }
    }
    println!(
        "{}",
        serde_json::to_string_pretty(
            &json!({"schema_version":1,"predeclared_cases":cases,"selection":{"mode":mode,"case":selected},"physical_output_enabled":false,"controller_receives_scene_truth":false,"all_passed":all_passed,"results":results})
        )?
    );
    if !all_passed {
        std::process::exit(1);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn completion_flag_cannot_replace_physical_and_task_gates() {
        let case = cases().remove(0);
        let good = json!({"completed":true,"fault":null,
            "final_actual_speed_mps":0.0,"final_actual_curvature_per_m":0.0,
            "minimum_cone_clearance_m":0.1,"crosswalk_hold_ms":3000,
            "green_observed_ms":300,
            "phases":["cones","approach_light","wait_green","finish","completed"],
            "online_referee":{"order_valid":true,"trajectory_valid":true,"both_completed":true}});
        assert!(failures(&good, &case).is_empty());
        for (field, value) in [
            ("final_actual_speed_mps", json!(0.01)),
            ("final_actual_curvature_per_m", json!(0.01)),
            ("minimum_cone_clearance_m", json!(-0.01)),
            ("crosswalk_hold_ms", json!(2999)),
            ("green_observed_ms", json!(299)),
            ("phases", json!(["completed"])),
            ("online_referee", json!({"both_completed":true})),
        ] {
            let mut bad = good.clone();
            bad[field] = value;
            assert!(!failures(&bad, &case).is_empty(), "{field}");
        }
    }

    #[test]
    fn missing_marker_case_must_reach_marker_task_and_end_with_bounded_stop() {
        let case = cases().remove(6);
        let mut result = json!({"completed":false,"fault":"ordinary stop exceeded 10 seconds",
            "final_actual_speed_mps":0.0,"final_actual_curvature_per_m":0.0,
            "minimum_cone_clearance_m":0.1,"crosswalk_hold_ms":3000,
            "phases":["cones","approach_light","fault"],
            "online_referee":{"order_valid":true,"trajectory_valid":true,"both_completed":true}});
        assert!(failures(&result, &case).is_empty());
        result["phases"] = json!(["cones", "fault"]);
        assert!(!failures(&result, &case).is_empty());
        result["phases"] = json!(["cones", "approach_light", "fault"]);
        result["fault"] = json!("physical collision");
        assert!(!failures(&result, &case).is_empty());
    }
}
