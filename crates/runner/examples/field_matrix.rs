//! Predeclared field geometry/placement matrix, not a search for successful cases.
//! Each run uses the unchanged controller and competition gates. No trace capture
//! or vehicle I/O; this does not measure target deadlines or hardware reliability.
use serde::Serialize;
use serde_json::{Value, json};
use xt_stcar_robot_core::MotionOutput;
use xt_stcar_robot_core::autonomy::Pose2;
use xt_stcar_robot_core::field::CROSSWALK_DEPTH_M;
use xt_stcar_robot_core::mission::MissionPhase;
use xt_stcar_robot_core::tracking::TrackingConfig;
use xt_stcar_robot_runner::async_simulation::{AsyncSimulationOptions, simulate_async};
use xt_stcar_robot_runner::field::FieldScenario;
use xt_stcar_robot_runner::phase_statistics::CompetitionStatistics;
use xt_stcar_robot_runner::simulation::{SimulationConfig, simulate};

const SCENARIO_DURATION_MS: u64 = 180_000;
const CASE_NAMES: [&str; 9] = [
    "small_5x4",
    "tall_5x6",
    "wide_8x4",
    "large_8x6",
    "nominal_7x5",
    "crosswalk_earliest",
    "crosswalk_latest",
    "light_forward",
    "cones_shifted",
];

#[derive(Clone, Copy, Serialize)]
#[serde(rename_all = "snake_case")]
enum Mode {
    Sync,
    Async,
}

impl Mode {
    fn name(self) -> &'static str {
        match self {
            Self::Sync => "sync",
            Self::Async => "async",
        }
    }
}

#[derive(Serialize)]
struct Case {
    name: &'static str,
    scenario: FieldScenario,
}

/// Fixed before executing the matrix. Even a rejected compilation is retained;
/// no case is removed, and no parameter is adjusted using an observed outcome.
fn cases() -> Vec<Case> {
    let mut result = Vec::with_capacity(CASE_NAMES.len());
    for (name, length, width) in [
        ("small_5x4", 5.0, 4.0),
        ("tall_5x6", 5.0, 6.0),
        ("wide_8x4", 8.0, 4.0),
        ("large_8x6", 8.0, 6.0),
    ] {
        let mut scenario = FieldScenario::example();
        let narrow = length == 5.0;
        let short = width == 4.0;
        scenario.spec.length_m = length;
        scenario.spec.width_m = width;
        scenario.spec.bottom_lane_width_m = if short { 1.0 } else { 1.2 };
        scenario.spec.top_lane_width_m = scenario.spec.bottom_lane_width_m;
        scenario.spec.bottom_straight_span_m = if narrow { 3.0 } else { 5.5 };
        scenario.spec.top_straight_span_m = scenario.spec.bottom_straight_span_m;
        scenario.spec.left_cone_from_left_m = if narrow { 1.0 } else { 1.5 };
        scenario.spec.right_cone_from_right_m = scenario.spec.left_cone_from_left_m;
        scenario.spec.left_cone_from_top_m = if short { 2.0 } else { 2.5 };
        scenario.spec.right_cone_from_bottom_m = width / 2.0;
        scenario.spec.light_front_x_m = length - 0.8;
        scenario.cone_route_radius_m = if narrow { 0.7 } else { 0.85 };
        scenario.crosswalk_near_x_m = if narrow { 2.0 } else { 2.5 };
        result.push(Case { name, scenario });
    }
    result.push(Case {
        name: "nominal_7x5",
        scenario: FieldScenario::example(),
    });
    let mut earliest = FieldScenario::example();
    earliest.crosswalk_near_x_m = earliest.spec.crosswalk_search_start_m;
    result.push(Case {
        name: "crosswalk_earliest",
        scenario: earliest,
    });
    let mut latest = FieldScenario::example();
    latest.crosswalk_near_x_m = latest.spec.bottom_straight_span_m - CROSSWALK_DEPTH_M;
    result.push(Case {
        name: "crosswalk_latest",
        scenario: latest,
    });
    let mut light = FieldScenario::example();
    light.spec.light_front_x_m = 4.3;
    result.push(Case {
        name: "light_forward",
        scenario: light,
    });
    let mut cones = FieldScenario::example();
    cones.spec.left_cone_from_left_m = 1.8;
    cones.spec.right_cone_from_right_m = 1.8;
    cones.spec.left_cone_from_top_m = 2.7;
    cones.spec.right_cone_from_bottom_m = 2.7;
    result.push(Case {
        name: "cones_shifted",
        scenario: cones,
    });
    result
}

#[derive(Serialize)]
struct Metrics {
    completed: bool,
    fault: Option<String>,
    elapsed_ms: u64,
    distance_m: f64,
    minimum_cone_clearance_m: f64,
    crosswalk_hold_ms: u64,
    green_observed_ms: u64,
    phases: Vec<MissionPhase>,
    final_pose: Pose2,
    final_actual_speed_mps: f64,
    /// None means the synchronous summary does not expose the actual value.
    final_actual_curvature_per_m: Option<f64>,
    terminal_command_stop: Option<bool>,
    counts: Option<Value>,
    statistics: Value,
    first_problem: Option<Value>,
}

#[derive(Serialize)]
struct Run {
    mode: Mode,
    tracker: &'static str,
    passed: bool,
    checks: Vec<Check>,
    summary: Option<Metrics>,
    error: Option<String>,
}

#[derive(Serialize)]
struct Check {
    name: &'static str,
    passed: bool,
}

#[derive(Serialize)]
struct CaseResult {
    name: &'static str,
    scenario: FieldScenario,
    compilation_valid: bool,
    compilation_error: Option<String>,
    passed: bool,
    runs: Vec<Run>,
}

fn compact_statistics(statistics: &CompetitionStatistics) -> Value {
    let phases: Vec<_> = statistics
        .phases
        .iter()
        .map(|phase| {
            json!({
                "phase": phase.phase,
                "first_at": phase.first_at,
                "duration_ms": phase.duration_ms,
                "distance_m": phase.distance_m,
                "unexpected_stop_episodes": phase.unexpected_stop_episodes,
                "unexpected_stop_command_ms": phase.unexpected_stop_command_ms,
                "fault_ticks": phase.fault_ticks,
                "navigation_reason_ticks": phase.navigation_reason_ticks,
                "route_generations": phase.routes.generations,
                "terminal_budget_exhausted_ticks": phase.terminal_work.budget_exhausted_ticks,
                "arrival_region_attempts": phase.terminal_work.arrival_region_attempts,
                "arrival_region_accepted": phase.terminal_work.arrival_region_accepted,
                "recovery_attempted_ticks": phase.forward_search.recovery_attempted_ticks,
                "recovery_accepted_ticks": phase.forward_search.recovery_accepted_ticks,
            })
        })
        .collect();
    json!({"phases": phases, "final_braking": statistics.final_braking,
        "omitted_distance_samples": statistics.omitted_distance_samples})
}

fn execute(config: &SimulationConfig, mode: Mode) -> Result<Metrics, String> {
    match mode {
        Mode::Sync => {
            let summary = simulate(config, &mut std::io::sink())?;
            let first_problem = summary
                .first_navigation_failure
                .as_ref()
                .and_then(|window| {
                    window.frames.get(window.trigger_index).map(|frame| {
                        json!({
                            "at": frame.at, "phase": frame.phase,
                            "navigation_status": frame.navigation_status,
                            "navigation_reason": frame.navigation_reason, "fault": frame.fault,
                        })
                    })
                });
            Ok(Metrics {
                completed: summary.completed,
                fault: summary.fault,
                elapsed_ms: summary.elapsed_ms,
                distance_m: summary.distance_m,
                minimum_cone_clearance_m: summary.minimum_cone_clearance_m,
                crosswalk_hold_ms: summary.crosswalk_hold_ms,
                green_observed_ms: summary.green_observed_ms,
                phases: summary.phases,
                final_pose: summary.final_pose,
                final_actual_speed_mps: summary.final_actual_speed_mps,
                final_actual_curvature_per_m: None,
                terminal_command_stop: None,
                counts: None,
                statistics: compact_statistics(&summary.statistics),
                first_problem,
            })
        }
        Mode::Async => {
            let summary = simulate_async(
                config,
                &AsyncSimulationOptions::default(),
                &mut std::io::sink(),
            )?;
            let first_problem = summary.first_problem.as_ref().map(|problem| json!({
                "at": problem.at, "actual_pose": problem.actual_pose,
                "runtime_fault": problem.runtime_fault,
                "adoption_rejection": problem.adoption_rejection,
                "navigation_reason": problem.control.as_ref().and_then(|c| c.navigation_reason.as_deref()),
            }));
            Ok(Metrics {
                completed: summary.completed,
                fault: summary.fault,
                elapsed_ms: summary.elapsed_ms,
                distance_m: summary.distance_m,
                minimum_cone_clearance_m: summary.minimum_cone_clearance_m,
                crosswalk_hold_ms: summary.crosswalk_hold_ms,
                green_observed_ms: summary.green_observed_ms,
                phases: summary.phases,
                final_pose: summary.final_pose,
                final_actual_speed_mps: summary.final_actual_speed_mps,
                final_actual_curvature_per_m: Some(summary.final_actual_curvature_per_m),
                terminal_command_stop: Some(summary.terminal_command == MotionOutput::Stop),
                counts: Some(json!(summary.counts)),
                statistics: compact_statistics(&summary.statistics),
                first_problem,
            })
        }
    }
}

fn acceptance(config: &SimulationConfig, summary: &Metrics) -> Vec<Check> {
    let mission = &config.autonomy.mission;
    let heading_error = (summary.final_pose.yaw_rad - mission.finish_yaw_rad
        + std::f64::consts::PI)
        .rem_euclid(std::f64::consts::TAU)
        - std::f64::consts::PI;
    let mut checks: Vec<Check> = [
        ("completed", summary.completed),
        ("no_fault", summary.fault.is_none()),
        (
            "within_declared_duration",
            summary.elapsed_ms <= SCENARIO_DURATION_MS,
        ),
        (
            "finite_nonnegative_travel",
            summary.distance_m.is_finite() && summary.distance_m >= 0.0,
        ),
        (
            "no_cone_collision",
            summary.minimum_cone_clearance_m.is_finite() && summary.minimum_cone_clearance_m >= 0.0,
        ),
        (
            "crosswalk_hold_at_least_3000ms",
            summary.crosswalk_hold_ms >= 3000,
        ),
        (
            "green_observed_at_least_300ms",
            summary.green_observed_ms >= 300,
        ),
        (
            "final_speed_zero",
            summary.final_actual_speed_mps.is_finite()
                && summary.final_actual_speed_mps.abs() <= 1e-9,
        ),
        (
            "full_footprint_inside_finish",
            mission
                .footprint
                .inside(summary.final_pose, mission.finish_region),
        ),
        (
            "finish_heading_matches",
            heading_error.is_finite() && heading_error.abs() <= mission.goal_heading_tolerance_rad,
        ),
        (
            "final_phase_completed",
            summary.phases.last() == Some(&MissionPhase::Completed),
        ),
        (
            "original_task_gates",
            config.max_duration_ms == SCENARIO_DURATION_MS
                && mission.crosswalk_hold_ms == 3000
                && mission.min_green_ms == 300,
        ),
    ]
    .into_iter()
    .map(|(name, passed)| Check { name, passed })
    .collect();
    if let Some(curvature) = summary.final_actual_curvature_per_m {
        checks.push(Check {
            name: "final_curvature_zero",
            passed: curvature.is_finite() && curvature.abs() <= 1e-9,
        });
    }
    if let Some(stopped) = summary.terminal_command_stop {
        checks.push(Check {
            name: "terminal_command_stop",
            passed: stopped,
        });
    }
    checks
}

fn trackers() -> [(&'static str, TrackingConfig); 2] {
    [
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
    ]
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    if args.first().is_some_and(|arg| arg == "--help") && args.len() == 1 {
        println!(
            "field_matrix [sync|async|both] [case_name|all] [pp|lqr|both]\nDefaults: both all both. Cases: {}.\nFixed 180000 ms scenario ceiling; original 3000 ms crossing hold, 300 ms green confirmation, 10 s ordinary-stop rule and controller limits.\nInvalid and failed cases are retained in JSON and cause a nonzero exit. No hardware, no real-time deadline or continuous-range guarantee.",
            CASE_NAMES.join(", ")
        );
        return Ok(());
    }
    let mode_choice = args.first().map_or("both", String::as_str);
    let case_choice = args.get(1).map_or("all", String::as_str);
    let tracker_choice = args.get(2).map_or("both", String::as_str);
    if args.len() > 3
        || !["sync", "async", "both"].contains(&mode_choice)
        || (case_choice != "all" && !CASE_NAMES.contains(&case_choice))
        || !["pp", "lqr", "both"].contains(&tracker_choice)
    {
        return Err("usage: field_matrix [sync|async|both] [case_name|all] [pp|lqr|both]; use --help for cases".into());
    }
    let frozen_cases = cases();
    let mut results = Vec::with_capacity(frozen_cases.len());
    for case in &frozen_cases {
        if case_choice != "all" && case_choice != case.name {
            continue;
        }
        let compiled = case.scenario.compile();
        let compilation_error = compiled.as_ref().err().cloned();
        let mut runs = Vec::with_capacity(4);
        for mode in [Mode::Sync, Mode::Async] {
            if mode_choice != "both" && mode_choice != mode.name() {
                continue;
            }
            for (tracker_name, tracking) in trackers() {
                if tracker_choice != "both" && tracker_choice != tracker_name {
                    continue;
                }
                eprintln!(
                    "field_matrix: {} / {} / {}",
                    case.name,
                    mode.name(),
                    tracker_name
                );
                let outcome = match &compiled {
                    Ok(base) => {
                        let mut config = base.clone();
                        config.autonomy.navigation.tracking = tracking;
                        execute(&config, mode).map(|summary| {
                            let checks = acceptance(&config, &summary);
                            (summary, checks)
                        })
                    }
                    Err(error) => Err(format!("field compilation rejected: {error}")),
                };
                let run = match outcome {
                    Ok((summary, checks)) => Run {
                        mode,
                        tracker: tracker_name,
                        passed: checks.iter().all(|c| c.passed),
                        checks,
                        summary: Some(summary),
                        error: None,
                    },
                    Err(error) => Run {
                        mode,
                        tracker: tracker_name,
                        passed: false,
                        checks: Vec::new(),
                        summary: None,
                        error: Some(error.chars().take(512).collect()),
                    },
                };
                eprintln!(
                    "field_matrix: {} / {} / {}: {}",
                    case.name,
                    mode.name(),
                    tracker_name,
                    if run.passed { "passed" } else { "failed" }
                );
                runs.push(run);
            }
        }
        results.push(CaseResult {
            name: case.name,
            scenario: case.scenario.clone(),
            compilation_valid: compiled.is_ok(),
            compilation_error,
            passed: !runs.is_empty() && runs.iter().all(|r| r.passed),
            runs,
        });
    }
    let all_passed = !results.is_empty() && results.iter().all(|case| case.passed);
    let selected_runs: usize = results.iter().map(|case| case.runs.len()).sum();
    let passed_runs: usize = results
        .iter()
        .flat_map(|case| &case.runs)
        .filter(|run| run.passed)
        .count();
    serde_json::to_writer_pretty(
        std::io::stdout().lock(),
        &json!({
            "schema_version": 1,
            "physical_output_enabled": false,
            "all_passed": all_passed,
            "selected_runs": selected_runs,
            "passed_runs": passed_runs,
            "predeclared_cases": frozen_cases,
            "selection": {"mode": mode_choice, "case": case_choice, "tracker": tracker_choice},
            "fixed_gates": {"scenario_duration_ms": SCENARIO_DURATION_MS, "crosswalk_hold_ms": 3000,
                "green_confirmation_ms": 300, "ordinary_stop_limit_ms": 10000,
                "controller_limits": "Unchanged original simulation controller, footprint, clearance, Q/R, search and certificate budgets",
                "legacy_motion_performance_gate": "Not used: field routes have different lengths; original eight timing runs remain separate regressions"},
            "async_timing": AsyncSimulationOptions::default(),
            "coverage": {
                "complete_declared_matrix": selected_runs == 36,
                "pp": "default controller", "lqr": "experimental controller",
                "scope": "Nine fixed geometry/placement cases; invalid combinations and functional failures are retained",
                "limits": "Finite synthetic cases are not proof for every combination of the official ranges, localization noise, unseen obstacle layouts, real camera calibration, hardware readiness or target WCET. Async host waits freeze functional time.",
                "sync_final_curvature": "Unavailable in the synchronous summary; not inferred from zero speed",
                "diagnostics": "Per-phase counts and one compact first-problem description; no point clouds, images, routes or per-tick trajectory export"
            },
            "results": results,
        }),
    )?;
    println!();
    if !all_passed {
        return Err(
            "field matrix contains rejected or failed cases; inspect all retained results".into(),
        );
    }
    Ok(())
}
