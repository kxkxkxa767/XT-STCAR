//! Offline 2×2 timing experiment. Bounded traces are retained only in this
//! explicit example, compared in memory, then one JSON report is emitted.
use serde::Serialize;
use xt_stcar_robot_core::autonomy::{Point2, Pose2};
use xt_stcar_robot_core::mission::MissionPhase;
use xt_stcar_robot_core::tracking::TrackingConfig;
use xt_stcar_robot_core::{MotionOutput, Timestamp};
use xt_stcar_robot_runner::async_simulation::{
    AsyncSimulationOptions, AsyncSimulationSummary, simulate_async_observed,
};
use xt_stcar_robot_runner::control_runtime::{AdoptionRejection, ControlPoll};
use xt_stcar_robot_runner::simulation::SimulationConfig;

const MAX_COMMAND_CHANGES: usize = 16_384;
const MAX_PLANS: usize = 2_048;
const MAX_PATH_POINTS: usize = 2_048;

#[derive(Clone, Debug, Serialize)]
struct CommandChange {
    at: Timestamp,
    command: MotionOutput,
    pose: Pose2,
}

#[derive(Debug, Serialize)]
struct PlanSample {
    source_at: Timestamp,
    planned_at: Timestamp,
    observed_at: Timestamp,
    pose: Pose2,
    proposed_command: MotionOutput,
    phase: Option<MissionPhase>,
    waypoint_index: Option<usize>,
    output_command: MotionOutput,
    adoption_rejection: Option<AdoptionRejection>,
    navigation_reason: Option<String>,
    route_revision: u64,
    route_rebuild_reason: Option<String>,
    route_points: usize,
    path_length_m: f64,
    path_start: Option<Point2>,
    path_end: Option<Point2>,
    recovery_attempted: bool,
    recovery_active: bool,
    #[serde(skip)]
    path: Vec<Point2>,
}

#[derive(Default)]
struct Trace {
    commands: Vec<CommandChange>,
    plans: Vec<PlanSample>,
    omitted_commands: u64,
    omitted_plans: u64,
    omitted_path_points: u64,
    last_plan: Option<Timestamp>,
    last_command: Option<MotionOutput>,
    last_at: Option<Timestamp>,
    stop_duration_ms: u64,
    stop_episodes: u64,
    drive_during_rejection_polls: u64,
    stop_during_rejection_polls: u64,
}

impl Trace {
    fn observe(&mut self, poll: &ControlPoll, pose: Pose2) {
        if self.last_command == Some(MotionOutput::Stop)
            && let Some(at) = self.last_at
        {
            self.stop_duration_ms += poll.at.0.saturating_sub(at.0);
        }
        self.last_at = Some(poll.at);
        if self.last_command.as_ref() != Some(&poll.command) {
            self.stop_episodes += u64::from(poll.command == MotionOutput::Stop);
            self.last_command = Some(poll.command.clone());
            if self.commands.len() < MAX_COMMAND_CHANGES {
                self.commands.push(CommandChange {
                    at: poll.at,
                    command: poll.command.clone(),
                    pose,
                });
            } else {
                self.omitted_commands += 1;
            }
        }
        if poll.adoption_rejection.is_some() {
            if poll.command == MotionOutput::Stop {
                self.stop_during_rejection_polls += 1;
            } else {
                self.drive_during_rejection_polls += 1;
            }
        }
        let Some(plan) = &poll.observed_plan else {
            return;
        };
        if self.last_plan == Some(plan.planned_at) {
            return;
        }
        self.last_plan = Some(plan.planned_at);
        if self.plans.len() == MAX_PLANS {
            self.omitted_plans += 1;
            return;
        }
        let nav = plan.report.navigation.as_ref();
        let path = nav.map_or(&[][..], |nav| nav.path.as_slice());
        self.omitted_path_points += path.len().saturating_sub(MAX_PATH_POINTS) as u64;
        self.plans.push(PlanSample {
            source_at: plan.source_at,
            planned_at: plan.planned_at,
            observed_at: poll.at,
            pose,
            proposed_command: plan.report.command.clone(),
            phase: plan.report.mission.as_ref().map(|mission| mission.phase),
            waypoint_index: plan
                .report
                .mission
                .as_ref()
                .map(|mission| mission.waypoint_index),
            output_command: poll.command.clone(),
            adoption_rejection: poll.adoption_rejection,
            navigation_reason: nav.and_then(|nav| nav.reason.clone()),
            route_revision: nav.map_or(0, |nav| nav.diagnostics.route_revision),
            // Optional JSON lookup keeps this observer source compilable against
            // the original review baseline, which lacks this diagnostic field.
            // This allocation is confined to this explicit offline example.
            route_rebuild_reason: nav
                .and_then(|nav| serde_json::to_value(nav.diagnostics).ok())
                .and_then(|value| {
                    value
                        .get("route_rebuild_reason")?
                        .as_str()
                        .map(str::to_owned)
                }),
            route_points: path.len(),
            path_length_m: path.windows(2).map(|p| p[0].distance(p[1])).sum(),
            path_start: path.first().copied(),
            path_end: path.last().copied(),
            recovery_attempted: nav
                .is_some_and(|nav| nav.diagnostics.forward_search.recovery_attempted),
            recovery_active: nav.is_some_and(|nav| nav.diagnostics.forward_search.recovery_active),
            path: path.iter().take(MAX_PATH_POINTS).copied().collect(),
        });
    }

    fn complete(&self) -> bool {
        self.omitted_commands == 0 && self.omitted_plans == 0 && self.omitted_path_points == 0
    }

    fn counts(&self) -> serde_json::Value {
        serde_json::json!({
            "command_changes":self.commands.len(), "plans":self.plans.len(),
            "omitted_commands":self.omitted_commands,"omitted_plans":self.omitted_plans,
            "omitted_path_points":self.omitted_path_points,
            "stop_duration_ms":self.stop_duration_ms,"stop_episodes_including_startup_and_shutdown":self.stop_episodes,
            "drive_during_rejection_polls":self.drive_during_rejection_polls,
            "stop_during_rejection_polls":self.stop_during_rejection_polls,
            "route_search_events":self.plans.iter().filter(|plan| plan.route_rebuild_reason.is_some()).collect::<Vec<_>>(),
        })
    }
}

fn first_output_difference(a: &Trace, b: &Trace) -> serde_json::Value {
    let (mut i, mut j) = (0, 0);
    let (mut ca, mut cb) = (None, None);
    let mut times: Vec<_> = a
        .commands
        .iter()
        .chain(&b.commands)
        .map(|c| c.at.0)
        .collect();
    times.sort_unstable();
    times.dedup();
    // Compare only the common observed interval; do not extrapolate beyond a run.
    let end = a
        .last_at
        .unwrap_or(Timestamp(0))
        .0
        .min(b.last_at.unwrap_or(Timestamp(0)).0);
    for at in times.into_iter().take_while(|at| *at <= end) {
        while i < a.commands.len() && a.commands[i].at.0 <= at {
            ca = Some(&a.commands[i]);
            i += 1;
        }
        while j < b.commands.len() && b.commands[j].at.0 <= at {
            cb = Some(&b.commands[j]);
            j += 1;
        }
        if ca.map(|c| &c.command) != cb.map(|c| &c.command) {
            return serde_json::json!({"at_ms":at,"a_last_change":ca,"b_last_change":cb});
        }
    }
    serde_json::Value::Null
}

fn first_plan_difference(
    a: &Trace,
    b: &Trace,
    differs: impl Fn(&PlanSample, &PlanSample) -> bool,
) -> serde_json::Value {
    for pa in &a.plans {
        if let Some(pb) = b.plans.iter().find(|p| p.source_at == pa.source_at)
            && differs(pa, pb)
        {
            return serde_json::json!({"source_at":pa.source_at,"a":pa,"b":pb});
        }
    }
    serde_json::Value::Null
}

fn compare(a: &Trace, b: &Trace) -> serde_json::Value {
    serde_json::json!({
        "traces_complete":a.complete() && b.complete(),
        "scope":"Actual outputs aligned by model time; plans and returned navigation path aligned by identical source capture stamp. This path includes the current projected position and remaining reference segment. Exact point/count differences need not mean replanning or different topology; consult revision, rebuild reason and phase. Null means no difference in the common captured interval only when traces_complete=true.",
        "first_actual_output_difference":first_output_difference(a,b),
        "first_same_source_command_difference":first_plan_difference(a,b,|a,b| a.proposed_command != b.proposed_command),
        "first_same_source_path_difference":first_plan_difference(a,b,|a,b| a.path != b.path),
        "first_same_source_route_revision_difference":first_plan_difference(a,b,|a,b| a.route_revision != b.route_revision),
        "first_same_source_path_point_count_difference":first_plan_difference(a,b,|a,b| a.route_points != b.route_points),
        "first_same_source_recovery_difference":first_plan_difference(a,b,|a,b| a.recovery_active != b.recovery_active || a.recovery_attempted != b.recovery_attempted),
    })
}

#[derive(Serialize)]
struct MatrixRun {
    tracker: &'static str,
    input_delays_ms: [u64; 2],
    adoption_delays_ms: [u64; 3],
    output_trace_counts: serde_json::Value,
    summary: AsyncSimulationSummary,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    if std::env::args().len() != 1 {
        return Err("usage: async_timing_matrix (fixed original 2x2 schedules, PP and LQR)".into());
    }
    let mut runs = Vec::new();
    let mut comparisons = Vec::new();
    for (name, tracker) in [
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
    ] {
        let mut traces = Vec::new();
        for input in [[60, 80], [40, 60]] {
            for adoption in [[3, 7, 9], [5, 9, 11]] {
                let mut config = SimulationConfig::example();
                config.autonomy.navigation.tracking = tracker;
                let timing = AsyncSimulationOptions {
                    input_delays_ms: input,
                    adoption_delays_ms: adoption,
                    ..Default::default()
                };
                let mut trace = Trace::default();
                let summary = simulate_async_observed(
                    &config,
                    &timing,
                    &mut std::io::sink(),
                    |poll, pose, _, _| trace.observe(poll, pose),
                )?;
                eprintln!(
                    "{name} input={input:?} adopt={adoption:?}: completed={} elapsed={} fault={:?}",
                    summary.completed, summary.elapsed_ms, summary.fault
                );
                runs.push(MatrixRun {
                    tracker: name,
                    input_delays_ms: input,
                    adoption_delays_ms: adoption,
                    output_trace_counts: trace.counts(),
                    summary,
                });
                traces.push(trace);
            }
        }
        for (a, b, factor) in [
            (0, 1, "adoption_offsets"),
            (2, 3, "adoption_offsets"),
            (0, 2, "input_delays"),
            (1, 3, "input_delays"),
        ] {
            comparisons.push(serde_json::json!({
                "tracker":name,"changed_factor":factor,"a_schedule_index":a,"b_schedule_index":b,
                "comparison":compare(&traces[a],&traces[b]),
            }));
        }
    }
    let all_completed = runs.iter().all(|r| r.summary.completed);
    let traces_complete = comparisons
        .iter()
        .all(|c| c["comparison"]["traces_complete"] == true);
    serde_json::to_writer_pretty(
        std::io::stdout().lock(),
        &serde_json::json!({
            "schema_version":1,"physical_output_enabled":false,
            "scope":"Offline real-worker 2x2 functional timing comparison. No Q/R, limits, budgets or other configuration changes. Observer overhead included in host wall_time_ms, no timing-performance claim.",
            "trace_limits":{"command_changes":MAX_COMMAND_CHANGES,"plans":MAX_PLANS,"path_points_per_plan":MAX_PATH_POINTS},
            "schedule_order":"[60,80]x[3,7,9]; [60,80]x[5,9,11]; [40,60]x[3,7,9]; [40,60]x[5,9,11]",
            "stop_scope":"Stop duration integrates the actual output state from startup through final braking. Stop episodes include startup/shutdown; navigation unexpected stops and first-observation rejection counts remain separate in summary.",
            "all_completed":all_completed,"traces_complete":traces_complete,"runs":runs,"single_factor_comparisons":comparisons,
        }),
    )?;
    println!();
    if !all_completed || !traces_complete {
        return Err("matrix contains a failed run or truncated comparison; inspect JSON".into());
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stop_time_uses_held_output_and_rejected_drive_is_not_a_stop() {
        let mut trace = Trace::default();
        for (at, command, rejection) in [
            (0, MotionOutput::Stop, None),
            (
                10,
                MotionOutput::Drive {
                    speed_mps: 0.1,
                    curvature_per_m: 0.0,
                },
                Some(AdoptionRejection::CurvatureSlew),
            ),
            (25, MotionOutput::Stop, None),
            (25, MotionOutput::Stop, None),
            (40, MotionOutput::Stop, None),
        ] {
            trace.observe(
                &ControlPoll {
                    at: Timestamp(at),
                    command,
                    fault: None,
                    adoption_rejection: rejection,
                    observed_plan: None,
                    latest: None,
                },
                Pose2::default(),
            );
        }
        assert_eq!(trace.stop_duration_ms, 25);
        assert_eq!(trace.stop_episodes, 2);
        assert_eq!(trace.drive_during_rejection_polls, 1);
        assert_eq!(trace.stop_during_rejection_polls, 0);
        assert_eq!(trace.commands.len(), 3);
    }

    fn change(at: u64, command: MotionOutput) -> CommandChange {
        CommandChange {
            at: Timestamp(at),
            command,
            pose: Pose2::default(),
        }
    }

    #[test]
    fn held_output_comparison_distinguishes_adoption_timing_and_common_horizon() {
        let drive = MotionOutput::Drive {
            speed_mps: 0.1,
            curvature_per_m: 0.0,
        };
        let a = Trace {
            commands: vec![change(0, MotionOutput::Stop), change(63, drive.clone())],
            last_at: Some(Timestamp(100)),
            ..Default::default()
        };
        let mut b = Trace {
            commands: vec![change(0, MotionOutput::Stop), change(65, drive)],
            last_at: Some(Timestamp(100)),
            ..Default::default()
        };
        assert_eq!(first_output_difference(&a, &b)["at_ms"], 63);
        b.last_at = Some(Timestamp(60));
        assert!(first_output_difference(&a, &b).is_null());
        b.omitted_plans = 1;
        assert!(!compare(&a, &b)["traces_complete"].as_bool().unwrap());
    }
}
