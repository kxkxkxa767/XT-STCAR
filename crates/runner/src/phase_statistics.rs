//! Bounded in-memory observation of the synthetic competition. Never changes control.
use crate::autonomy::AutonomyStep;
use crate::navigation_diagnostics::is_unexpected_stop;
use serde::Serialize;
use std::collections::VecDeque;
use xt_stcar_robot_core::mission::MissionPhase;
use xt_stcar_robot_core::navigation::{CandidateDiagnostics, NavigationDecision};
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
}

impl CandidateTotals {
    fn add(&mut self, sample: CandidateDiagnostics) {
        macro_rules! add {
            ($($field:ident),+) => {$(self.$field = self.$field.saturating_add(sample.$field as u64);)+};
        }
        add!(
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
            terminal_unreachable
        );
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
    use xt_stcar_robot_core::navigation::{NavigationDiagnostics, NavigationStatus};

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
