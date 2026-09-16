//! Bounded in-memory observation of the synthetic competition. Never changes control.
use crate::autonomy::AutonomyStep;
use crate::navigation_diagnostics::is_unexpected_stop;
use serde::Serialize;
use std::collections::VecDeque;
use xt_stcar_robot_core::mission::MissionPhase;
use xt_stcar_robot_core::navigation::{
    CandidateDiagnostics, NavigationDecision, NavigationDiagnostics,
};
use xt_stcar_robot_core::{MotionOutput, Timestamp};

pub const RECENT_ROUTE_GENERATIONS: usize = 16;

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum NavigationReason {
    TaskStop,
    GoalBraking,
    GoalReached,
    StaleOrUnreliableInput,
    MeasuredSpeedOutsideForwardLimits,
    CurrentFootprintCollisionOrBoundary,
    StopBoundaryNotReachable,
    NoGridPath,
    NoForwardKinematicPath,
    InvalidLocalReference,
    PathTracking,
    EmptySpeedInterval,
    NoCollisionFreeBrakingTrajectory,
    TerminalBudgetExhausted,
    ForwardNodeBudgetExhausted,
    Other,
}

impl NavigationReason {
    fn classify(reason: &str) -> Self {
        match reason {
            "task_stop" => Self::TaskStop,
            "goal_braking" => Self::GoalBraking,
            "goal_reached" => Self::GoalReached,
            "stale_or_unreliable_pose_or_obstacles" => Self::StaleOrUnreliableInput,
            "measured_speed_outside_forward_limits" => Self::MeasuredSpeedOutsideForwardLimits,
            "current_footprint_collision_or_boundary" => Self::CurrentFootprintCollisionOrBoundary,
            "stop_boundary_not_reachable" => Self::StopBoundaryNotReachable,
            "no_grid_path" => Self::NoGridPath,
            "no_forward_kinematic_path" => Self::NoForwardKinematicPath,
            "invalid_local_reference" => Self::InvalidLocalReference,
            "empty_speed_interval" => Self::EmptySpeedInterval,
            "no_collision_free_braking_trajectory" => Self::NoCollisionFreeBrakingTrajectory,
            "terminal_budget_exhausted" => Self::TerminalBudgetExhausted,
            "forward_node_budget_exhausted" => Self::ForwardNodeBudgetExhausted,
            text if text.starts_with("path_tracking:") => Self::PathTracking,
            _ => Self::Other,
        }
    }
}

#[derive(Debug, PartialEq, Serialize)]
pub struct ReasonCount {
    pub reason: NavigationReason,
    pub ticks: u64,
}

#[derive(Debug, Default, PartialEq, Serialize)]
pub struct CandidateTotals {
    pub admission_current: u64,
    pub admission_next: u64,
    pub curvature_speed_limits: u64,
    pub lateral_acceleration: u64,
    pub near_zero_speed: u64,
    pub rollouts_evaluated: u64,
    pub accepted: u64,
    pub stop_reachable: u64,
    pub grid: u64,
    pub footprint: u64,
    pub sample_budget: u64,
    pub reference: u64,
    pub terminal_unreachable: u64,
    pub terminal_budget: u64,
}

impl CandidateTotals {
    fn add(&mut self, sample: CandidateDiagnostics) {
        macro_rules! add {
            ($($field:ident),+) => {$(self.$field = self.$field.saturating_add(sample.$field as u64);)+};
        }
        add!(
            admission_current,
            admission_next,
            curvature_speed_limits,
            lateral_acceleration,
            near_zero_speed,
            rollouts_evaluated,
            accepted,
            stop_reachable,
            grid,
            footprint,
            sample_budget,
            reference,
            terminal_unreachable,
            terminal_budget
        );
    }
}

/// Bounded counts of both the ordinary and continuous-geometry searches.
/// Each phase keeps totals/peaks rather than a per-expansion disk trace.
#[derive(Debug, Default, PartialEq, Serialize)]
pub struct ForwardSearchStatistics {
    pub admission_forecast_sources: WorkCounter,
    pub admission_projection_intervals: WorkCounter,
    pub allocated_nodes: WorkCounter,
    pub expanded_nodes: WorkCounter,
    pub primitive_attempts: WorkCounter,
    pub primitive_rejections: WorkCounter,
    pub recovery_attempted_ticks: u64,
    pub recovery_accepted_ticks: u64,
    pub recovery_active_ticks: u64,
}

impl ForwardSearchStatistics {
    /// An absent JSON field denotes exactly this all-zero default.
    fn is_empty(&self) -> bool {
        *self == Self::default()
    }

    fn observe(&mut self, diagnostics: &NavigationDiagnostics) {
        self.admission_forecast_sources
            .observe(diagnostics.admission_forecast_work.source_checks);
        self.admission_projection_intervals
            .observe(diagnostics.admission_forecast_work.projection_intervals);
        let sample = diagnostics.forward_search;
        self.allocated_nodes.observe(
            sample
                .ordinary
                .allocated_nodes
                .saturating_add(sample.recovery.allocated_nodes),
        );
        self.expanded_nodes.observe(
            sample
                .ordinary
                .expanded_nodes
                .saturating_add(sample.recovery.expanded_nodes),
        );
        self.primitive_attempts.observe(
            sample
                .ordinary
                .primitive_attempts
                .saturating_add(sample.recovery.primitive_attempts),
        );
        self.primitive_rejections.observe(
            sample
                .ordinary
                .primitive_rejections
                .saturating_add(sample.recovery.primitive_rejections),
        );
        self.recovery_attempted_ticks = self
            .recovery_attempted_ticks
            .saturating_add(u64::from(sample.recovery_attempted));
        self.recovery_accepted_ticks = self
            .recovery_accepted_ticks
            .saturating_add(u64::from(sample.recovery_accepted));
        self.recovery_active_ticks = self
            .recovery_active_ticks
            .saturating_add(u64::from(sample.recovery_active));
    }
}

#[derive(Debug, Default, PartialEq, Serialize)]
pub struct WorkCounter {
    pub total: u64,
    pub maximum_per_tick: u64,
}

impl WorkCounter {
    fn is_zero(&self) -> bool {
        self.total == 0 && self.maximum_per_tick == 0
    }
    fn observe(&mut self, value: usize) {
        self.total = self.total.saturating_add(value as u64);
        self.maximum_per_tick = self.maximum_per_tick.max(value as u64);
    }
}

/// Fixed-size work accounting. Counters are not CPU durations; the separately
/// optional solver_elapsed_ns field records host wall time when enabled.
/// Terminal work includes initial/global/lattice connections and candidate checks
/// sharing the core's per-call ledger, rather than only rejected candidates.
#[derive(Debug, Default, PartialEq, Serialize)]
pub struct TerminalWorkStatistics {
    pub solver_attempts: WorkCounter,
    pub iterations: WorkCounter,
    /// A solver consumed all eight local iterations without certification;
    /// distinct from exhausting the shared per-tick ledger.
    pub solver_iteration_exhaustions: WorkCounter,
    /// Omitted only when both total and per-tick peak are zero.
    #[serde(skip_serializing_if = "WorkCounter::is_zero")]
    pub cold_budget_deferrals: WorkCounter,
    /// Reserved capacity across ticks, not work actually performed.
    pub recovery_reserved_solvers: WorkCounter,
    pub recovery_reserved_iterations: WorkCounter,
    pub recovery_reserved_samples: WorkCounter,
    /// Omitted only when no reservation shortfall occurred in this phase.
    #[serde(skip_serializing_if = "WorkCounter::is_zero")]
    pub recovery_reservation_shortfalls: WorkCounter,
    /// Host timing only when explicitly enabled in Navigator.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub solver_elapsed_ns: Option<WorkCounter>,
    /// Budget-charged samples: a rejected primitive may not execute its tail.
    pub primitive_samples: WorkCounter,
    pub continued_seed_attempts: WorkCounter,
    pub continued_seed_accepted: WorkCounter,
    /// Rejected only by the short solver's forward/distance applicability domain.
    pub domain_rejections: WorkCounter,
    /// Original grid-transition rejections after sample budget was admitted;
    /// this does not identify a physical collision or prove geometric infeasibility.
    pub primitive_grid_rejections: WorkCounter,
    #[serde(skip_serializing_if = "WorkCounter::is_zero")]
    pub arrival_region_attempts: WorkCounter,
    #[serde(skip_serializing_if = "WorkCounter::is_zero")]
    pub arrival_region_accepted: WorkCounter,
    pub terminal_connections_checked: WorkCounter,
    pub budget_exhausted_ticks: u64,
    pub continuity_enforced_ticks: u64,
    /// Largest configured ledger limits observed in this phase, not consumption.
    pub maximum_solver_limit: usize,
    pub maximum_iteration_limit: usize,
    pub maximum_sample_limit: usize,
}

impl TerminalWorkStatistics {
    fn observe(&mut self, diagnostics: &NavigationDiagnostics) {
        let sample = diagnostics.terminal_work;
        self.solver_attempts.observe(sample.solver_attempts);
        self.iterations.observe(sample.iterations);
        self.solver_iteration_exhaustions
            .observe(sample.solver_iteration_exhaustions);
        self.cold_budget_deferrals
            .observe(sample.cold_budget_deferrals);
        self.recovery_reserved_solvers
            .observe(sample.recovery_reserved_solvers);
        self.recovery_reserved_iterations
            .observe(sample.recovery_reserved_iterations);
        self.recovery_reserved_samples
            .observe(sample.recovery_reserved_samples);
        self.recovery_reservation_shortfalls
            .observe(sample.recovery_reservation_shortfalls);
        if let Some(elapsed_ns) = sample.solver_elapsed_ns {
            let timing = self
                .solver_elapsed_ns
                .get_or_insert_with(WorkCounter::default);
            timing.total = timing.total.saturating_add(elapsed_ns);
            timing.maximum_per_tick = timing.maximum_per_tick.max(elapsed_ns);
        }
        self.primitive_samples.observe(sample.primitive_samples);
        self.continued_seed_attempts
            .observe(sample.continued_seed_attempts);
        self.continued_seed_accepted
            .observe(sample.continued_seed_accepted);
        self.domain_rejections.observe(sample.domain_rejections);
        self.primitive_grid_rejections
            .observe(sample.primitive_grid_rejections);
        self.arrival_region_attempts
            .observe(sample.arrival_region_attempts);
        self.arrival_region_accepted
            .observe(sample.arrival_region_accepted);
        self.terminal_connections_checked
            .observe(diagnostics.terminal_connections_checked);
        self.budget_exhausted_ticks = self
            .budget_exhausted_ticks
            .saturating_add(u64::from(sample.budget_exhausted));
        self.continuity_enforced_ticks = self
            .continuity_enforced_ticks
            .saturating_add(u64::from(diagnostics.terminal_continuity_enforced));
        self.maximum_solver_limit = self.maximum_solver_limit.max(sample.solver_limit);
        self.maximum_iteration_limit = self.maximum_iteration_limit.max(sample.iteration_limit);
        self.maximum_sample_limit = self.maximum_sample_limit.max(sample.sample_limit);
    }
}

#[derive(Debug, PartialEq, Serialize)]
pub struct RouteGeneration {
    pub at: Timestamp,
    pub revision: u64,
    /// A revision jump may include generations this observer did not see.
    pub generations_since_previous_observation: u64,
    /// Returned current leg, excluding the separately checked continuation.
    /// None means a generation was observed without a usable returned path.
    pub current_leg_length_m: Option<f64>,
    pub checked_continuation_length_m: Option<f64>,
}

#[derive(Debug, Default, PartialEq, Serialize)]
pub struct RouteStatistics {
    /// Successful path generations, including initial paths and changed goals.
    /// A revision increment does not identify why a route was generated.
    pub generations: u64,
    pub generations_without_observed_length: u64,
    pub observed_length_count: u64,
    pub observed_current_leg_length_sum_m: f64,
    pub minimum_observed_current_leg_length_m: Option<f64>,
    pub maximum_observed_current_leg_length_m: Option<f64>,
    pub recent_generations: VecDeque<RouteGeneration>,
    pub omitted_generation_events: u64,
}

impl RouteStatistics {
    fn observe(&mut self, at: Timestamp, nav: &NavigationDecision, delta: u64) {
        self.generations = self.generations.saturating_add(delta);
        let length = (nav.path.len() >= 2)
            .then(|| {
                nav.path
                    .windows(2)
                    .map(|pair| pair[0].distance(pair[1]))
                    .sum::<f64>()
            })
            .filter(|length| length.is_finite() && *length >= 0.0);
        self.generations_without_observed_length = self
            .generations_without_observed_length
            .saturating_add(delta.saturating_sub(u64::from(length.is_some())));
        if let Some(length) = length {
            self.observed_length_count = self.observed_length_count.saturating_add(1);
            self.observed_current_leg_length_sum_m += length;
            self.minimum_observed_current_leg_length_m = Some(
                self.minimum_observed_current_leg_length_m
                    .map_or(length, |old| old.min(length)),
            );
            self.maximum_observed_current_leg_length_m = Some(
                self.maximum_observed_current_leg_length_m
                    .map_or(length, |old| old.max(length)),
            );
        }
        if self.recent_generations.len() == RECENT_ROUTE_GENERATIONS {
            self.recent_generations.pop_front();
            self.omitted_generation_events = self.omitted_generation_events.saturating_add(1);
        }
        self.recent_generations.push_back(RouteGeneration {
            at,
            revision: nav.diagnostics.route_revision,
            generations_since_previous_observation: delta,
            current_leg_length_m: length,
            checked_continuation_length_m: nav.diagnostics.checked_continuation_distance_m.filter(
                |length| {
                    nav.diagnostics.continuation_checked && length.is_finite() && *length >= 0.0
                },
            ),
        });
    }
}

#[derive(Debug, PartialEq, Serialize)]
pub struct PhaseStatistics {
    /// None is reserved for a control result with no reported mission phase.
    pub phase: Option<MissionPhase>,
    pub first_at: Timestamp,
    pub last_control_at: Timestamp,
    pub control_ticks: u64,
    /// Time and actual integrated travel of intervals beginning in this phase.
    pub duration_ms: u64,
    pub distance_m: f64,
    pub unexpected_stop_ticks: u64,
    /// Contiguous unexpected Stop runs within this phase, not blocked candidates.
    pub unexpected_stop_episodes: u64,
    /// Duration of adopted Stop command intervals, not time physically stationary.
    pub unexpected_stop_command_ms: u64,
    pub fault_ticks: u64,
    pub navigation_reason_ticks: Vec<ReasonCount>,
    pub candidate_totals: CandidateTotals,
    #[serde(skip_serializing_if = "ForwardSearchStatistics::is_empty")]
    pub forward_search: ForwardSearchStatistics,
    pub terminal_work: TerminalWorkStatistics,
    pub routes: RouteStatistics,
}

impl PhaseStatistics {
    fn new(phase: Option<MissionPhase>, at: Timestamp) -> Self {
        Self {
            phase,
            first_at: at,
            last_control_at: at,
            control_ticks: 0,
            duration_ms: 0,
            distance_m: 0.0,
            unexpected_stop_ticks: 0,
            unexpected_stop_episodes: 0,
            unexpected_stop_command_ms: 0,
            fault_ticks: 0,
            navigation_reason_ticks: Vec::new(),
            candidate_totals: CandidateTotals::default(),
            forward_search: ForwardSearchStatistics::default(),
            terminal_work: TerminalWorkStatistics::default(),
            routes: RouteStatistics::default(),
        }
    }

    fn reason(&mut self, reason: &str) {
        let reason = NavigationReason::classify(reason);
        if let Some(count) = self
            .navigation_reason_ticks
            .iter_mut()
            .find(|count| count.reason == reason)
        {
            count.ticks = count.ticks.saturating_add(1);
        } else {
            // The closed enum bounds this vector regardless of input strings.
            self.navigation_reason_ticks
                .push(ReasonCount { reason, ticks: 1 });
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum FinalBrakingCause {
    Completion,
    FaultOrScenarioEnd,
}

#[derive(Debug, Default, PartialEq, Serialize)]
pub struct FinalBrakingStatistics {
    /// Post-loop Stop propagation is separate; it is not assigned to Completed.
    pub after_phase: Option<MissionPhase>,
    pub cause: Option<FinalBrakingCause>,
    pub started_at: Option<Timestamp>,
    pub ticks: u64,
    pub duration_ms: u64,
    pub distance_m: f64,
}

#[derive(Debug, Default, PartialEq, Serialize)]
pub struct CompetitionStatistics {
    /// At most nine mission phases plus one unreported-phase bucket.
    pub phases: Vec<PhaseStatistics>,
    pub final_braking: FinalBrakingStatistics,
    /// Invalid optional distance samples are skipped, never a control fault.
    pub omitted_distance_samples: u64,
}

#[derive(Default)]
pub(crate) struct CompetitionStatisticsCollector {
    summary: CompetitionStatistics,
    active: Option<usize>,
    previous_unexpected: bool,
    previous_revision: u64,
}

impl CompetitionStatisticsCollector {
    pub fn observe_step(&mut self, step: &AutonomyStep) {
        let phase = step
            .mission
            .as_ref()
            .map(|mission| mission.phase)
            .or_else(|| step.fault.as_ref().map(|_| MissionPhase::Fault));
        self.observe(
            step.at,
            phase,
            &step.command,
            step.navigation.as_ref(),
            step.fault.is_some(),
        );
    }

    fn observe(
        &mut self,
        at: Timestamp,
        phase: Option<MissionPhase>,
        command: &MotionOutput,
        navigation: Option<&NavigationDecision>,
        fault: bool,
    ) {
        let index = self
            .summary
            .phases
            .iter()
            .position(|stats| stats.phase == phase)
            .unwrap_or_else(|| {
                self.summary.phases.push(PhaseStatistics::new(phase, at));
                self.summary.phases.len() - 1
            });
        let phase_changed = self.active != Some(index);
        let unexpected = *command == MotionOutput::Stop
            && is_unexpected_stop(
                navigation.map(|nav| nav.status),
                navigation.and_then(|nav| nav.reason.as_deref()),
                fault,
            );
        self.active = Some(index);
        let stats = &mut self.summary.phases[index];
        stats.last_control_at = at;
        stats.control_ticks = stats.control_ticks.saturating_add(1);
        stats.fault_ticks = stats.fault_ticks.saturating_add(u64::from(fault));
        if unexpected {
            stats.unexpected_stop_ticks = stats.unexpected_stop_ticks.saturating_add(1);
            if phase_changed || !self.previous_unexpected {
                stats.unexpected_stop_episodes = stats.unexpected_stop_episodes.saturating_add(1);
            }
        }
        self.previous_unexpected = unexpected;
        if let Some(nav) = navigation {
            if let Some(reason) = &nav.reason {
                stats.reason(reason);
            }
            stats.candidate_totals.add(nav.diagnostics.candidates);
            stats.forward_search.observe(&nav.diagnostics);
            stats.terminal_work.observe(&nav.diagnostics);
            if nav.diagnostics.route_revision > self.previous_revision {
                let delta = nav.diagnostics.route_revision - self.previous_revision;
                stats.routes.observe(at, nav, delta);
                self.previous_revision = nav.diagnostics.route_revision;
            }
        }
    }

    pub fn advance_control_interval(&mut self, duration_ms: u64, distance_m: f64) {
        if let Some(index) = self.active {
            let stats = &mut self.summary.phases[index];
            stats.duration_ms = stats.duration_ms.saturating_add(duration_ms);
            if distance_m.is_finite() && distance_m >= 0.0 {
                stats.distance_m += distance_m;
            } else {
                self.summary.omitted_distance_samples =
                    self.summary.omitted_distance_samples.saturating_add(1);
            }
            if self.previous_unexpected {
                stats.unexpected_stop_command_ms =
                    stats.unexpected_stop_command_ms.saturating_add(duration_ms);
            }
        }
    }

    pub fn advance_final_braking(
        &mut self,
        start: Timestamp,
        cause: FinalBrakingCause,
        duration_ms: u64,
        distance_m: f64,
    ) {
        let stats = &mut self.summary.final_braking;
        if stats.started_at.is_none() {
            stats.started_at = Some(start);
            stats.cause = Some(cause);
            stats.after_phase = self
                .active
                .and_then(|index| self.summary.phases[index].phase);
        }
        stats.ticks = stats.ticks.saturating_add(1);
        stats.duration_ms = stats.duration_ms.saturating_add(duration_ms);
        if distance_m.is_finite() && distance_m >= 0.0 {
            stats.distance_m += distance_m;
        } else {
            self.summary.omitted_distance_samples =
                self.summary.omitted_distance_samples.saturating_add(1);
        }
    }

    pub fn finish(self) -> CompetitionStatistics {
        self.summary
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use xt_stcar_robot_core::MotionIntent;
    use xt_stcar_robot_core::autonomy::Point2;
    use xt_stcar_robot_core::navigation::{
        NavigationDiagnostics, NavigationStatus, TerminalWorkDiagnostics,
    };

    fn decision(reason: Option<&str>, revision: u64) -> NavigationDecision {
        NavigationDecision {
            status: if reason.is_some() {
                NavigationStatus::Blocked
            } else {
                NavigationStatus::Driving
            },
            intent: MotionIntent {
                speed_mps: 0.2,
                curvature_per_m: 0.0,
            },
            path: vec![Point2 { x_m: 0.0, y_m: 0.0 }, Point2 { x_m: 3.0, y_m: 4.0 }],
            reason: reason.map(str::to_owned),
            diagnostics: NavigationDiagnostics {
                route_revision: revision,
                ..NavigationDiagnostics::default()
            },
        }
    }

    #[test]
    fn sparse_phase_work_preserves_nonzero_recovery_statistics() {
        let mut phase = PhaseStatistics::new(Some(MissionPhase::Cones), Timestamp(0));
        let empty = serde_json::to_value(&phase).unwrap();
        assert!(empty.get("forward_search").is_none());
        phase.forward_search.recovery_attempted_ticks = 1;
        let written = serde_json::to_value(&phase).unwrap();
        assert_eq!(
            written["forward_search"],
            serde_json::to_value(&phase.forward_search).unwrap()
        );
    }

    #[test]
    fn phase_intervals_stop_episodes_and_final_braking_have_distinct_accounting() {
        let mut c = CompetitionStatisticsCollector::default();
        let drive = MotionOutput::Drive {
            speed_mps: 0.2,
            curvature_per_m: 0.0,
        };
        c.observe(
            Timestamp(0),
            Some(MissionPhase::Cones),
            &drive,
            Some(&decision(None, 1)),
            false,
        );
        c.advance_control_interval(100, 0.02);
        c.observe(
            Timestamp(100),
            Some(MissionPhase::Cones),
            &MotionOutput::Stop,
            Some(&decision(Some("goal_braking"), 1)),
            false,
        );
        c.advance_control_interval(100, 0.01);
        for at in [200, 300] {
            c.observe(
                Timestamp(at),
                Some(MissionPhase::Cones),
                &MotionOutput::Stop,
                Some(&decision(Some("empty_speed_interval"), 1)),
                false,
            );
            c.advance_control_interval(100, 0.005);
        }
        c.observe(
            Timestamp(400),
            Some(MissionPhase::ApproachLight),
            &drive,
            Some(&decision(None, 2)),
            false,
        );
        c.advance_control_interval(100, 0.02);
        c.observe(
            Timestamp(500),
            Some(MissionPhase::Fault),
            &MotionOutput::Stop,
            None,
            true,
        );
        c.advance_final_braking(
            Timestamp(500),
            FinalBrakingCause::FaultOrScenarioEnd,
            100,
            0.007,
        );
        let summary = c.finish();
        let cones = &summary.phases[0];
        assert_eq!((cones.control_ticks, cones.duration_ms), (4, 400));
        assert_eq!(
            (
                cones.unexpected_stop_ticks,
                cones.unexpected_stop_episodes,
                cones.unexpected_stop_command_ms
            ),
            (2, 1, 200)
        );
        assert!((cones.distance_m - 0.04).abs() < 1e-12);
        assert_eq!(summary.phases[2].unexpected_stop_ticks, 1);
        assert_eq!(summary.phases[2].duration_ms, 0);
        assert_eq!(summary.final_braking.after_phase, Some(MissionPhase::Fault));
        assert_eq!(summary.final_braking.duration_ms, 100);
        assert_eq!(
            summary.phases.iter().map(|p| p.duration_ms).sum::<u64>()
                + summary.final_braking.duration_ms,
            600
        );
    }

    #[test]
    fn repeated_paths_are_not_generations_and_revision_gaps_do_not_invent_lengths() {
        let mut c = CompetitionStatisticsCollector::default();
        let drive = MotionOutput::Drive {
            speed_mps: 0.2,
            curvature_per_m: 0.0,
        };
        for (at, revision) in [(0, 1), (100, 1), (200, 3)] {
            c.observe(
                Timestamp(at),
                Some(MissionPhase::Cones),
                &drive,
                Some(&decision(None, revision)),
                false,
            );
        }
        let summary = c.finish();
        let routes = &summary.phases[0].routes;
        assert_eq!(routes.generations, 3);
        assert_eq!(routes.observed_length_count, 2);
        assert_eq!(routes.generations_without_observed_length, 1);
        assert_eq!(routes.observed_current_leg_length_sum_m, 10.0);
        assert_eq!(routes.recent_generations.len(), 2);
    }

    #[test]
    fn terminal_work_preserves_phase_totals_peaks_and_budget_rejections() {
        let mut c = CompetitionStatisticsCollector::default();
        for (at, phase, work) in [
            (0, MissionPhase::Cones, 3),
            (100, MissionPhase::Cones, 7),
            (200, MissionPhase::ApproachLight, 5),
        ] {
            let mut nav = decision(Some("terminal_budget_exhausted"), 0);
            nav.diagnostics.terminal_work = TerminalWorkDiagnostics {
                solver_attempts: work,
                iterations: work * 2,
                primitive_samples: work * 10,
                budget_exhausted: work == 7,
                solver_limit: 256,
                iteration_limit: 1024,
                sample_limit: 65_536,
                continued_seed_attempts: work,
                continued_seed_accepted: work - 1,
                domain_rejections: work - 2,
                primitive_grid_rejections: work - 3,
                arrival_region_attempts: work - 2,
                arrival_region_accepted: work - 3,
                solver_iteration_exhaustions: work - 1,
                cold_budget_deferrals: work - 2,
                recovery_reserved_solvers: 1,
                recovery_reserved_iterations: 8,
                recovery_reserved_samples: 64,
                recovery_reservation_shortfalls: usize::from(work == 7),
                solver_elapsed_ns: Some(work as u64 * 100),
            };
            nav.diagnostics.terminal_connections_checked = work + 1;
            nav.diagnostics.terminal_continuity_enforced = work != 3;
            nav.diagnostics.candidates.terminal_budget = usize::from(work == 7);
            nav.diagnostics.candidates.terminal_unreachable = 2;
            c.observe(
                Timestamp(at),
                Some(phase),
                &MotionOutput::Stop,
                Some(&nav),
                false,
            );
        }
        let summary = c.finish();
        let cones = &summary.phases[0];
        let work = &cones.terminal_work;
        assert_eq!(
            (
                work.solver_attempts.total,
                work.solver_attempts.maximum_per_tick
            ),
            (10, 7)
        );
        assert_eq!(
            (work.iterations.total, work.iterations.maximum_per_tick),
            (20, 14)
        );
        assert_eq!(
            (
                work.solver_iteration_exhaustions.total,
                work.solver_iteration_exhaustions.maximum_per_tick
            ),
            (8, 6)
        );
        assert_eq!(
            (
                work.cold_budget_deferrals.total,
                work.cold_budget_deferrals.maximum_per_tick
            ),
            (6, 5)
        );
        assert_eq!(
            (
                work.recovery_reserved_solvers.total,
                work.recovery_reserved_iterations.total,
                work.recovery_reserved_samples.total
            ),
            (2, 16, 128)
        );
        assert_eq!(work.recovery_reservation_shortfalls.total, 1);
        let timing = work.solver_elapsed_ns.as_ref().unwrap();
        assert_eq!((timing.total, timing.maximum_per_tick), (1000, 700));
        assert_eq!(
            (
                work.primitive_samples.total,
                work.primitive_samples.maximum_per_tick
            ),
            (100, 70)
        );
        assert_eq!(
            (
                work.continued_seed_attempts.total,
                work.continued_seed_accepted.total
            ),
            (10, 8)
        );
        assert_eq!(
            (
                work.domain_rejections.total,
                work.domain_rejections.maximum_per_tick
            ),
            (6, 5)
        );
        assert_eq!(
            (
                work.primitive_grid_rejections.total,
                work.primitive_grid_rejections.maximum_per_tick
            ),
            (4, 4)
        );
        assert_eq!(
            (
                work.arrival_region_attempts.total,
                work.arrival_region_attempts.maximum_per_tick
            ),
            (6, 5)
        );
        assert_eq!(
            (
                work.arrival_region_accepted.total,
                work.arrival_region_accepted.maximum_per_tick
            ),
            (4, 4)
        );
        assert_eq!(
            (
                work.terminal_connections_checked.total,
                work.terminal_connections_checked.maximum_per_tick
            ),
            (12, 8)
        );
        assert_eq!(
            (work.budget_exhausted_ticks, work.continuity_enforced_ticks),
            (1, 1)
        );
        assert_eq!(
            (
                work.maximum_solver_limit,
                work.maximum_iteration_limit,
                work.maximum_sample_limit
            ),
            (256, 1024, 65_536)
        );
        assert_eq!(
            (
                cones.candidate_totals.terminal_budget,
                cones.candidate_totals.terminal_unreachable
            ),
            (1, 4)
        );
        assert_eq!(
            cones.navigation_reason_ticks[0].reason,
            NavigationReason::TerminalBudgetExhausted
        );
        assert_eq!(summary.phases[1].terminal_work.solver_attempts.total, 5);
        assert_eq!(summary.phases[1].terminal_work.budget_exhausted_ticks, 0);
    }

    #[test]
    fn unused_arrival_region_counters_remain_absent_from_legacy_json() {
        let core = serde_json::to_value(TerminalWorkDiagnostics::default()).unwrap();
        let phase = serde_json::to_value(TerminalWorkStatistics::default()).unwrap();
        for value in [&core, &phase] {
            assert!(value.get("arrival_region_attempts").is_none());
            assert!(value.get("arrival_region_accepted").is_none());
        }
    }

    #[test]
    fn sparse_terminal_exceptions_preserve_observed_counts_and_zero_time_measurements() {
        let mut statistics = TerminalWorkStatistics::default();
        let empty = serde_json::to_value(&statistics).unwrap();
        for key in [
            "cold_budget_deferrals",
            "recovery_reservation_shortfalls",
            "solver_elapsed_ns",
        ] {
            assert!(empty.get(key).is_none(), "{key}");
        }
        statistics.observe(&NavigationDiagnostics {
            terminal_work: TerminalWorkDiagnostics {
                cold_budget_deferrals: 2,
                recovery_reservation_shortfalls: 1,
                solver_elapsed_ns: Some(0),
                ..TerminalWorkDiagnostics::default()
            },
            ..NavigationDiagnostics::default()
        });
        let observed = serde_json::to_value(&statistics).unwrap();
        assert_eq!(observed["cold_budget_deferrals"]["total"], 2);
        assert_eq!(observed["cold_budget_deferrals"]["maximum_per_tick"], 2);
        assert_eq!(observed["recovery_reservation_shortfalls"]["total"], 1);
        assert_eq!(
            observed["recovery_reservation_shortfalls"]["maximum_per_tick"],
            1
        );
        assert_eq!(observed["solver_elapsed_ns"]["total"], 0);
        assert_eq!(observed["solver_elapsed_ns"]["maximum_per_tick"], 0);
        assert!(
            observed.get("solver_elapsed_ns").is_some(),
            "observed zero is not an absent clock"
        );
    }

    #[test]
    fn accumulated_work_saturates_without_losing_the_single_tick_peak() {
        let mut counter = WorkCounter {
            total: u64::MAX - 2,
            maximum_per_tick: 4,
        };
        counter.observe(7);
        assert_eq!(counter.total, u64::MAX);
        assert_eq!(counter.maximum_per_tick, 7);
    }

    #[test]
    fn history_and_reason_storage_stay_bounded_without_discarding_totals() {
        let mut c = CompetitionStatisticsCollector::default();
        for revision in 1..=1000 {
            let reason = format!("arbitrary diagnostic {revision}");
            c.observe(
                Timestamp(revision),
                Some(MissionPhase::Cones),
                &MotionOutput::Stop,
                Some(&decision(Some(&reason), revision)),
                false,
            );
            c.advance_control_interval(100, if revision == 1000 { f64::NAN } else { 0.0 });
        }
        let summary = c.finish();
        let phase = &summary.phases[0];
        assert_eq!(phase.routes.generations, 1000);
        assert_eq!(
            phase.routes.recent_generations.len(),
            RECENT_ROUTE_GENERATIONS
        );
        assert_eq!(
            phase.routes.omitted_generation_events,
            1000 - RECENT_ROUTE_GENERATIONS as u64
        );
        assert_eq!(phase.navigation_reason_ticks.len(), 1);
        assert_eq!(phase.navigation_reason_ticks[0].ticks, 1000);
        assert_eq!(phase.unexpected_stop_episodes, 1);
        assert_eq!(summary.omitted_distance_samples, 1);
    }
}
