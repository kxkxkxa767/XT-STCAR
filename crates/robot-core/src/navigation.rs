//! Grid A*, bounded forward-car lattice search, and curvature rollout.
//! Collision envelopes are conservative; this is not a hardware braking guarantee.
//! The heading-aware search uses a weighted heuristic and a hard node budget;
//! failure means no candidate was found within that budget, not proof of no route.
//! Pure-pursuit geometry: R. Craig Coulter, CMU-RI-TR-92-01 (1992):
//! <https://publications.ri.cmu.edu/implementation-of-the-pure-pursuit-path-tracking-algorithm>
//! Forward-car circle/tangent geometry: LaValle, Planning Algorithms §15.3.1:
//! <https://lavalle.pl/planning/node821.html>
use crate::admission::{AdmissionForecastWork, AdmissionRejection, AdoptionConstraints};
use crate::autonomy::{Footprint, HalfPlane, ObstacleDisc, Point2, Pose2, PoseEstimate, Rect};
use crate::motion_transition::{MotionTransition, lateral_acceleration_peak, project_motion};
use crate::reference::{PreparedReference, ReferenceCursor};
use crate::tracking::{PathTracker, TrackInput, TrackingConfig, TrackingDiagnostics};
use crate::{FrameId, MotionIntent, MotionOutput, Timestamp, ValidationError};
use serde::{Deserialize, Serialize};
#[cfg(test)]
mod candidate_pipeline;
mod continuity;
use continuity::{CachedTerminalSeed, terminal_connection};
#[cfg(test)]
mod oriented_region_advance;
mod primitive_envelope;
mod recovery;
#[cfg(test)]
mod rolling_oriented_arrival;
#[cfg(test)]
mod short_arrival_repair;
#[cfg(test)]
mod short_point_target;
mod stopping_envelope;
mod terminal;
#[cfg(test)]
mod terminal_continuity_region;
pub use recovery::ForwardSearchDiagnostics;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};
pub use terminal::TerminalWorkDiagnostics;
use terminal::{TerminalBudget, TwoArcSeed};

fn default_heading_tolerance() -> f64 {
    0.12
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct NavigationConfig {
    pub frame_id: FrameId,
    pub bounds: Rect,
    pub footprint: Footprint,
    pub grid_resolution_m: f64,
    pub clearance_m: f64,
    pub max_grid_cells: usize,
    pub max_obstacles: usize,
    pub max_speed_mps: f64,
    pub max_curvature_per_m: f64,
    pub max_accel_mps2: f64,
    pub max_decel_mps2: f64,
    pub max_lateral_accel_mps2: f64,
    pub max_curvature_rate_per_s: f64,
    pub lookahead_m: f64,
    /// Pure Pursuit remains the default; LQR is an explicit offline experiment.
    #[serde(default)]
    pub tracking: TrackingConfig,
    pub preview_horizon_s: f64,
    pub control_period_ms: u64,
    pub max_input_age_ms: u64,
    pub min_pose_quality: f64,
    pub goal_tolerance_m: f64,
    #[serde(default = "default_heading_tolerance")]
    pub goal_heading_tolerance_rad: f64,
    pub curvature_samples: usize,
}

impl NavigationConfig {
    /// Synthetic initial values, never vehicle calibration.
    pub fn simulation(bounds: Rect, footprint: Footprint, frame_id: FrameId) -> Self {
        Self {
            frame_id,
            bounds,
            footprint,
            grid_resolution_m: 0.1,
            clearance_m: 0.04,
            max_grid_cells: 40_000,
            max_obstacles: 2048,
            max_speed_mps: 0.3,
            max_curvature_per_m: 2.0,
            max_accel_mps2: 0.4,
            max_decel_mps2: 0.6,
            max_lateral_accel_mps2: 0.4,
            max_curvature_rate_per_s: 4.0,
            lookahead_m: 0.55,
            tracking: TrackingConfig::default(),
            preview_horizon_s: 2.0,
            control_period_ms: 50,
            max_input_age_ms: 250,
            min_pose_quality: 0.4,
            goal_tolerance_m: 0.1,
            goal_heading_tolerance_rad: default_heading_tolerance(),
            curvature_samples: 21,
        }
    }

    pub fn validate(&self) -> Result<(), ValidationError> {
        self.frame_id.validate()?;
        self.bounds.validate()?;
        self.footprint.validate()?;
        self.tracking.validate()?;
        let positive = [
            self.grid_resolution_m,
            self.max_speed_mps,
            self.max_curvature_per_m,
            self.max_accel_mps2,
            self.max_decel_mps2,
            self.max_lateral_accel_mps2,
            self.max_curvature_rate_per_s,
            self.lookahead_m,
            self.preview_horizon_s,
            self.goal_tolerance_m,
        ];
        let width = self.bounds.max_x_m - self.bounds.min_x_m;
        let height = self.bounds.max_y_m - self.bounds.min_y_m;
        if positive.iter().any(|v| !v.is_finite() || *v <= 0.0)
            || !self.clearance_m.is_finite()
            || !(0.005..=1.0).contains(&self.clearance_m)
            || !(0.025..=1.0).contains(&self.grid_resolution_m)
            || self.max_speed_mps > 3.0
            || !(1e-6..=10.0).contains(&self.max_curvature_per_m)
            || self.max_accel_mps2 > 10.0
            || self.max_decel_mps2 > 10.0
            || self.max_lateral_accel_mps2 > 10.0
            || self.max_curvature_rate_per_s > 50.0
            || self.preview_horizon_s > 5.0
            || !(1e-6..=5.0).contains(&self.lookahead_m)
            || self.goal_tolerance_m > 1.0
            || !self.goal_heading_tolerance_rad.is_finite()
            || !(0.01..=0.5).contains(&self.goal_heading_tolerance_rad)
            || !(10..=200).contains(&self.control_period_ms)
            || !(self.control_period_ms..=2000).contains(&self.max_input_age_ms)
            || !self.min_pose_quality.is_finite()
            || !(0.0..=1.0).contains(&self.min_pose_quality)
            || !(5..=81).contains(&self.curvature_samples)
            || !(100..=100_000).contains(&self.max_grid_cells)
            || !(1..=4096).contains(&self.max_obstacles)
            || width > 100.0
            || height > 100.0
            || (width / self.grid_resolution_m).ceil() * (height / self.grid_resolution_m).ceil()
                > self.max_grid_cells as f64
            || [
                self.bounds.min_x_m,
                self.bounds.min_y_m,
                self.bounds.max_x_m,
                self.bounds.max_y_m,
            ]
            .iter()
            .any(|v| v.abs() > 1_000_000.0)
        {
            return Err(ValidationError(
                "invalid or excessive navigation limits".into(),
            ));
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum NavigationStatus {
    Driving,
    Reached,
    Blocked,
}

/// Fixed-size accounting, separate from path/point-cloud storage and control policy.
#[derive(Clone, Copy, Debug, Default, Serialize)]
pub struct CandidateDiagnostics {
    pub admission_current: usize,
    pub admission_next: usize,
    pub curvature_speed_limits: usize,
    pub lateral_acceleration: usize,
    pub near_zero_speed: usize,
    pub rollouts_evaluated: usize,
    pub accepted: usize,
    pub stop_reachable: usize,
    pub grid: usize,
    pub footprint: usize,
    pub sample_budget: usize,
    pub reference: usize,
    /// No terminal connection was certified; this does not prove geometric
    /// impossibility. See terminal_work for solver/iteration/grid causes.
    pub terminal_unreachable: usize,
    pub terminal_budget: usize,
}

/// The first cache invalidation leading to this route search. Failed searches
/// retain that cause until a route is generated; cache reuse reports no cause.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum RouteRebuildReason {
    Initial,
    TaskHold,
    ArrivalChanged,
    BoundaryChanged,
    GoalOrHeadingChanged,
    FootprintReferenceDrift,
    TrackingRejected,
    NoAcceptedCandidate,
    GridConnectivityRejected,
}

/// New admission/recovery fields use sparse JSON: an absent field means its
/// Rust default (zero work, no search, or no constraint/failure). Nondefault
/// values are serialized in full; the in-memory diagnostic is unchanged.
#[derive(Clone, Copy, Debug, Serialize)]
pub struct RouteLengthChange {
    pub old_remaining_distance_m: f64,
    pub new_remaining_distance_m: f64,
}

#[derive(Clone, Copy, Debug, Default, Serialize)]
pub struct NavigationDiagnostics {
    /// Present only for a same-goal footprint-drift rebuild with an available old cache.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub route_length_change: Option<RouteLengthChange>,
    #[serde(skip_serializing_if = "AdmissionForecastWork::is_empty")]
    pub admission_forecast_work: AdmissionForecastWork,
    #[serde(skip_serializing_if = "ForwardSearchDiagnostics::is_empty")]
    pub forward_search: ForwardSearchDiagnostics,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub adoption_constraints: Option<AdoptionConstraints>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub current_admission_failure: Option<AdmissionRejection>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub next_admission_failure: Option<AdmissionRejection>,
    pub travel_boundary: Option<HalfPlane>,
    pub execution_state: Option<SteeringEstimate>,
    pub continuation_checked: bool,
    pub speed_lower_mps: Option<f64>,
    pub speed_upper_mps: Option<f64>,
    pub goal_speed_cap_mps: Option<f64>,
    pub allowed_waypoint_speed_mps: Option<f64>,
    pub waypoint_admission_radius_m: Option<f64>,
    pub waypoint_admission_distance_m: Option<f64>,
    pub remaining_distance_m: Option<f64>,
    pub checked_continuation_distance_m: Option<f64>,
    pub route_revision: u64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub route_rebuild_reason: Option<RouteRebuildReason>,
    pub route_progress: Option<usize>,
    pub route_points: usize,
    pub tracking: Option<TrackingDiagnostics>,
    pub requested_curvature_per_m: Option<f64>,
    pub slew_limited_curvature_per_m: Option<f64>,
    pub selected_curvature_per_m: Option<f64>,
    pub selected_prediction_distance_m: Option<f64>,
    pub selected_prediction_endpoint: Option<Pose2>,
    pub selected_cross_track_m: Option<f64>,
    pub selected_heading_error_rad: Option<f64>,
    pub selected_progress_m: Option<f64>,
    pub selected_reference_cost_m: Option<f64>,
    pub current_stopping_margin: Option<StoppingMargin>,
    pub selected_stopping_margin: Option<StoppingMargin>,
    /// Present only when the selected rolling-local candidate's original
    /// stopping disk failed, but the complete bounded-curvature envelope passed.
    /// The existing disk margins remain visible and are not relabeled as safe.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub selected_braking_envelope_margin: Option<StoppingMargin>,
    pub selected_next_stopping_margin: Option<StoppingMargin>,
    /// Current/next states queried against bounded terminal families. Recovery
    /// may recheck a rejected next state; all solvers share terminal_work limits.
    pub terminal_connections_checked: usize,
    /// A single-arc arrival is unavailable but a short two-part arrival exists.
    pub terminal_continuity_enforced: bool,
    pub terminal_work: TerminalWorkDiagnostics,
    pub candidates: CandidateDiagnostics,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum StoppingConstraint {
    MinX,
    MaxX,
    MinY,
    MaxY,
    TravelBoundary,
    /// Index in this call's world obstacle list, not a persistent sensor ID.
    Obstacle {
        index: usize,
    },
}

#[derive(Clone, Copy, Debug, Serialize)]
pub struct StoppingMargin {
    pub clearance_m: f64,
    pub constraint: StoppingConstraint,
}

impl StoppingMargin {
    fn clear(self) -> bool {
        self.clearance_m.is_finite()
            && match self.constraint {
                StoppingConstraint::Obstacle { .. } => self.clearance_m > 0.0,
                _ => self.clearance_m >= 0.0,
            }
    }
}

#[derive(Clone, Debug, Serialize)]
pub struct NavigationDecision {
    pub status: NavigationStatus,
    pub intent: MotionIntent,
    pub path: Vec<Point2>,
    pub reason: Option<String>,
    pub diagnostics: NavigationDiagnostics,
}

impl NavigationDecision {
    fn stopped(status: NavigationStatus, reason: &str) -> Self {
        Self {
            status,
            intent: MotionIntent {
                speed_mps: 0.0,
                curvature_per_m: 0.0,
            },
            path: vec![],
            reason: Some(reason.into()),
            diagnostics: NavigationDiagnostics::default(),
        }
    }
}

/// Model estimate driven only by commands adopted by the output owner.
/// This is not measured steering feedback. All stamps use the session clock.
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct SteeringEstimate {
    pub at: Timestamp,
    pub commanded_curvature_per_m: f64,
    pub applied_curvature_per_m: f64,
}

impl SteeringEstimate {
    pub fn stationary(at: Timestamp) -> Self {
        Self {
            at,
            commanded_curvature_per_m: 0.0,
            applied_curvature_per_m: 0.0,
        }
    }

    pub fn advance_to(&mut self, now: Timestamp, rate: f64) -> Result<(), ValidationError> {
        if now < self.at
            || !rate.is_finite()
            || rate <= 0.0
            || !self.commanded_curvature_per_m.is_finite()
            || !self.applied_curvature_per_m.is_finite()
        {
            return Err(ValidationError("invalid steering estimate or time".into()));
        }
        let difference = self.commanded_curvature_per_m - self.applied_curvature_per_m;
        if !difference.is_finite() {
            return Err(ValidationError(
                "steering transition is not representable".into(),
            ));
        }
        let amount = rate * ((now.0 - self.at.0) as f64 / 1000.0);
        self.applied_curvature_per_m += difference.clamp(-amount, amount);
        self.at = now;
        Ok(())
    }

    pub fn adopt(
        &mut self,
        now: Timestamp,
        command: &MotionOutput,
        rate: f64,
        max_curvature: f64,
    ) -> Result<(), ValidationError> {
        let target = match command {
            MotionOutput::Stop => 0.0,
            MotionOutput::Drive {
                speed_mps,
                curvature_per_m,
            } => {
                if !speed_mps.is_finite() || *speed_mps < 0.0 {
                    return Err(ValidationError("invalid adopted speed".into()));
                }
                *curvature_per_m
            }
        };
        if !max_curvature.is_finite()
            || max_curvature <= 0.0
            || !target.is_finite()
            || target.abs() > max_curvature
            || self.applied_curvature_per_m.abs() > max_curvature
            || self.commanded_curvature_per_m.abs() > max_curvature
        {
            return Err(ValidationError("adopted steering exceeds limits".into()));
        }
        self.advance_to(now, rate)?;
        self.commanded_curvature_per_m = target;
        Ok(())
    }
}

/// Stop targets retain terminal braking. A through target can use a checked
/// continuation to the next target; failed continuation falls back to stopping.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ArrivalBehavior {
    #[default]
    Stop,
    PassThrough {
        next: Point2,
        next_heading_rad: Option<f64>,
        next_max_speed_mps: f64,
        /// The task owner's position radius for accepting this waypoint.
        /// This is independent of the navigator's stopping-goal tolerance.
        admission_radius_m: f64,
    },
}

/// Target semantics are fixed for the lifetime of a navigator. A sensor-local
/// rolling point may need a direct bounded connection before the lattice;
/// established fixed-waypoint callers retain their original search ordering.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum TargetPolicy {
    #[default]
    Fixed,
    RollingLocal,
}

pub struct Navigator {
    target_policy: TargetPolicy,
    adoption_constraints: Option<AdoptionConstraints>,
    recovery_active: bool,
    recovery_route_error_m: f64,
    recovery_sample_arc_bound_m: f64,
    terminal_budget: TerminalBudget,
    terminal_seed: Option<CachedTerminalSeed>,
    config: NavigationConfig,
    travel_boundary: Option<HalfPlane>,
    last_step: Option<Timestamp>,
    steering: SteeringEstimate,
    arrival: ArrivalBehavior,
    continuation_checked: bool,
    via_index: usize,
    route_revision: u64,
    pending_route_rebuild: Option<RouteRebuildReason>,
    diagnostics: NavigationDiagnostics,
    route: Option<(Point2, Option<f64>, Vec<Point2>, usize)>,
}

impl Navigator {
    /// Enable optional host solver timing for diagnostics. Normal control does
    /// not read a wall clock; this setting persists across per-plan resets.
    pub fn set_timing_enabled(&mut self, enabled: bool) {
        self.terminal_budget.set_timing_enabled(enabled);
    }

    pub fn new(config: NavigationConfig) -> Result<Self, ValidationError> {
        Self::new_with_target_policy(config, TargetPolicy::Fixed)
    }

    pub fn new_with_target_policy(
        config: NavigationConfig,
        target_policy: TargetPolicy,
    ) -> Result<Self, ValidationError> {
        config.validate()?;
        Ok(Self {
            target_policy,
            adoption_constraints: None,
            recovery_active: false,
            recovery_route_error_m: 0.0,
            recovery_sample_arc_bound_m: 0.0,
            terminal_budget: TerminalBudget::default(),
            terminal_seed: None,
            config,
            travel_boundary: None,
            last_step: None,
            steering: SteeringEstimate::stationary(Timestamp(0)),
            arrival: ArrivalBehavior::Stop,
            continuation_checked: false,
            via_index: 0,
            route_revision: 0,
            pending_route_rebuild: Some(RouteRebuildReason::Initial),
            diagnostics: NavigationDiagnostics::default(),
            route: None,
        })
    }

    pub fn config(&self) -> &NavigationConfig {
        &self.config
    }

    /// Replaced for each source snapshot; synchronous callers use None.
    pub fn set_adoption_constraints(&mut self, constraints: Option<AdoptionConstraints>) {
        self.adoption_constraints = constraints;
    }

    pub fn execution_state(&self) -> SteeringEstimate {
        self.steering
    }

    pub fn set_execution_state(&mut self, state: SteeringEstimate) -> Result<(), ValidationError> {
        if state.at < self.steering.at
            || self.last_step.is_some_and(|at| state.at < at)
            || !state.applied_curvature_per_m.is_finite()
            || !state.commanded_curvature_per_m.is_finite()
            || state.applied_curvature_per_m.abs() > self.config.max_curvature_per_m
            || state.commanded_curvature_per_m.abs() > self.config.max_curvature_per_m
        {
            return Err(ValidationError("invalid adopted steering state".into()));
        }
        self.steering = state;
        Ok(())
    }

    pub fn adopt_command(
        &mut self,
        now: Timestamp,
        command: &MotionOutput,
    ) -> Result<(), ValidationError> {
        self.steering.adopt(
            now,
            command,
            self.config.max_curvature_rate_per_s,
            self.config.max_curvature_per_m,
        )
    }

    /// Change the allowed travel domain without inventing actuator motion.
    /// Geometry is validated by HalfPlane's constructors. A cached route cannot
    /// survive adding, moving, or removing a boundary, even with the same goal.
    pub fn set_travel_boundary(&mut self, boundary: Option<HalfPlane>) {
        if self.travel_boundary != boundary {
            self.travel_boundary = boundary;
            self.note_route_rebuild(RouteRebuildReason::BoundaryChanged);
            self.route = None;
            self.terminal_seed = None;
            self.recovery_active = false;
        }
    }

    /// Call during a task hold; no timer expiry or motion history is invented.
    pub fn stop(&mut self, now: Timestamp) -> Result<NavigationDecision, ValidationError> {
        let decision = self.plan_stop(now)?;
        self.adopt_command(now, &MotionOutput::Stop)?;
        Ok(decision)
    }

    /// Prepare a hold without claiming that its command has been adopted.
    pub fn plan_stop(&mut self, now: Timestamp) -> Result<NavigationDecision, ValidationError> {
        if self.last_step.is_some_and(|old| now < old) || now < self.steering.at {
            return Err(ValidationError("navigation time regresses".into()));
        }
        self.steering
            .advance_to(now, self.config.max_curvature_rate_per_s)?;
        self.last_step = Some(now);
        self.terminal_budget.reset();
        self.note_route_rebuild(RouteRebuildReason::TaskHold);
        self.route = None;
        self.terminal_seed = None;
        self.recovery_active = false;
        self.diagnostics = NavigationDiagnostics {
            execution_state: Some(self.steering),
            travel_boundary: self.travel_boundary,
            route_revision: self.route_revision,
            ..NavigationDiagnostics::default()
        };
        Ok(self.blocked("task_stop"))
    }

    pub fn step(
        &mut self,
        now: Timestamp,
        pose: &PoseEstimate,
        obstacles: &[ObstacleDisc],
        obstacles_at: Timestamp,
        goal: Point2,
    ) -> Result<NavigationDecision, ValidationError> {
        self.step_with_speed_limit(
            now,
            pose,
            obstacles,
            obstacles_at,
            goal,
            self.config.max_speed_mps,
        )
    }

    pub fn step_with_speed_limit(
        &mut self,
        now: Timestamp,
        estimate: &PoseEstimate,
        obstacles: &[ObstacleDisc],
        obstacles_at: Timestamp,
        goal: Point2,
        speed_limit_mps: f64,
    ) -> Result<NavigationDecision, ValidationError> {
        self.step_with_goal_heading(
            now,
            estimate,
            obstacles,
            obstacles_at,
            goal,
            None,
            speed_limit_mps,
        )
    }

    /// Optional final heading is in the configured world frame. Arrival requires
    /// the configured heading tolerance. Search exhaustion requests Stop, never a spin.
    #[allow(clippy::too_many_arguments)]
    pub fn step_with_goal_heading(
        &mut self,
        now: Timestamp,
        estimate: &PoseEstimate,
        obstacles: &[ObstacleDisc],
        obstacles_at: Timestamp,
        goal: Point2,
        goal_heading_rad: Option<f64>,
        speed_limit_mps: f64,
    ) -> Result<NavigationDecision, ValidationError> {
        let decision = self.plan_with_arrival(
            now,
            estimate,
            obstacles,
            obstacles_at,
            goal,
            goal_heading_rad,
            speed_limit_mps,
            ArrivalBehavior::Stop,
        )?;
        let command = if decision.intent.speed_mps > 0.0 {
            MotionOutput::Drive {
                speed_mps: decision.intent.speed_mps,
                curvature_per_m: decision.intent.curvature_per_m,
            }
        } else {
            MotionOutput::Stop
        };
        self.adopt_command(now, &command)?;
        Ok(decision)
    }

    /// Planning only. The output owner must acknowledge its final command or
    /// supply a timestamped execution estimate before the next planning call.
    #[allow(clippy::too_many_arguments)]
    pub fn plan_with_arrival(
        &mut self,
        now: Timestamp,
        estimate: &PoseEstimate,
        obstacles: &[ObstacleDisc],
        obstacles_at: Timestamp,
        goal: Point2,
        goal_heading_rad: Option<f64>,
        speed_limit_mps: f64,
        arrival: ArrivalBehavior,
    ) -> Result<NavigationDecision, ValidationError> {
        if self.last_step.is_some_and(|old| now < old)
            || now < self.steering.at
            || matches!(arrival, ArrivalBehavior::PassThrough { next, next_heading_rad, next_max_speed_mps, admission_radius_m } if !next.valid() || next == goal || next_heading_rad.is_some_and(|v| !v.is_finite()) || !next_max_speed_mps.is_finite() || next_max_speed_mps <= 0.0 || !admission_radius_m.is_finite() || admission_radius_m <= 0.0)
            || now < estimate.captured_at
            || now < obstacles_at
            || !estimate.pose.valid()
            || estimate.frame_id != self.config.frame_id
            || !estimate.speed_mps.is_finite()
            || !estimate.yaw_rate_radps.is_finite()
            || !estimate.quality.is_finite()
            || !(0.0..=1.0).contains(&estimate.quality)
            || !goal.valid()
            || goal_heading_rad.is_some_and(|yaw| !yaw.is_finite())
            || !speed_limit_mps.is_finite()
            || speed_limit_mps < 0.0
            || obstacles.len() > self.config.max_obstacles
            || obstacles.iter().any(|o| {
                !o.center.valid()
                    || !o.radius_m.is_finite()
                    || !(0.0..=10.0).contains(&o.radius_m)
                    || o.center.x_m.abs() > 1_000_000.0
                    || o.center.y_m.abs() > 1_000_000.0
            })
        {
            return Err(ValidationError(
                "invalid navigation pose, obstacle, frame, time or limit".into(),
            ));
        }
        self.steering
            .advance_to(now, self.config.max_curvature_rate_per_s)?;
        if self.arrival != arrival {
            self.note_route_rebuild(RouteRebuildReason::ArrivalChanged);
            self.route = None;
            self.recovery_active = false;
            self.continuation_checked = false;
            self.arrival = arrival;
            self.terminal_seed = None;
        }
        let period = self.config.control_period_ms as f64 / 1000.0;
        self.diagnostics = NavigationDiagnostics {
            adoption_constraints: self.adoption_constraints,
            travel_boundary: self.travel_boundary,
            execution_state: Some(self.steering),
            route_revision: self.route_revision,
            ..NavigationDiagnostics::default()
        };
        let dt = self
            .last_step
            .map_or(period, |at| ((now.0 - at.0) as f64 / 1000.0).min(period));
        self.last_step = Some(now);
        self.terminal_budget.reset();
        if now.0 - estimate.captured_at.0 >= self.config.max_input_age_ms
            || now.0 - obstacles_at.0 >= self.config.max_input_age_ms
            || estimate.quality < self.config.min_pose_quality
        {
            return Ok(self.blocked("stale_or_unreliable_pose_or_obstacles"));
        }
        if estimate.speed_mps < -1e-6 || estimate.speed_mps > self.config.max_speed_mps + 1e-6 {
            return Ok(self.blocked("measured_speed_outside_forward_limits"));
        }
        if speed_limit_mps == 0.0 {
            return self.plan_stop(now);
        }
        let pose = estimate.pose;
        self.diagnostics.current_stopping_margin = Some(stopping_margin(
            &self.config,
            pose,
            estimate.speed_mps.max(0.0),
            obstacles,
            self.travel_boundary,
        ));
        if !pose_clear(&self.config, pose, obstacles, self.config.clearance_m)
            || self.travel_boundary.is_some_and(|boundary| {
                !boundary.contains_footprint(self.config.footprint, pose, self.config.clearance_m)
            })
        {
            return Ok(self.blocked("current_footprint_collision_or_boundary"));
        }
        // Even a near-stationary arrival must retain enough room to stop on
        // the allowed side. Do this before Reached/goal_braking can bypass rollout.
        let measured_speed = estimate.speed_mps.max(0.0);
        if self.travel_boundary.is_some_and(|boundary| {
            !boundary.contains_disc(
                pose.point(),
                body_radius(self.config.footprint)
                    + self.config.clearance_m
                    + 2.0 * measured_speed * period
                    + measured_speed.powi(2) / (2.0 * self.config.max_decel_mps2),
            )
        }) {
            return Ok(self.blocked("stop_boundary_not_reachable"));
        }
        let distance = pose.point().distance(goal);
        if let ArrivalBehavior::PassThrough {
            admission_radius_m, ..
        } = arrival
        {
            self.diagnostics.waypoint_admission_radius_m = Some(admission_radius_m);
            self.diagnostics.waypoint_admission_distance_m =
                Some((distance - admission_radius_m).max(0.0));
        }
        let heading_reached = goal_heading_rad.is_none_or(|yaw| {
            angle_error(pose.yaw_rad, yaw).abs() <= self.config.goal_heading_tolerance_rad
        });
        if arrival == ArrivalBehavior::Stop
            && distance <= self.config.goal_tolerance_m
            && heading_reached
            && estimate.speed_mps <= 0.02
        {
            let mut decision =
                NavigationDecision::stopped(NavigationStatus::Reached, "goal_reached");
            decision.diagnostics = self.diagnostics_snapshot();
            return Ok(decision);
        }
        if arrival == ArrivalBehavior::Stop
            && distance <= self.config.goal_tolerance_m
            && heading_reached
        {
            // A target inside the arrival region may now lie behind the body.
            // Request the stop contract and wait for measured standstill.
            return Ok(self.blocked("goal_braking"));
        }
        let previous_route_revision = self.route_revision;
        let mut grid = Grid::with_boundary(&self.config, obstacles, self.travel_boundary);
        grid.recovery.oriented_boundary = self.target_policy == TargetPolicy::RollingLocal;
        if self
            .route
            .as_ref()
            .is_some_and(|(old_goal, old_heading, _, _)| {
                *old_goal != goal || *old_heading != goal_heading_rad
            })
        {
            self.note_route_rebuild(RouteRebuildReason::GoalOrHeadingChanged);
            self.terminal_seed = None;
            self.recovery_active = false;
        } else if self.route.is_some()
            && self.pending_route_rebuild == Some(RouteRebuildReason::GoalOrHeadingChanged)
        {
            // A failed request for another goal leaves the old cache intact.
            // Returning to its goal cancels that pending diagnostic; otherwise
            // it would hide the cause of a later, unrelated cache invalidation.
            self.pending_route_rebuild = None;
        }
        if self
            .route
            .as_ref()
            .is_none_or(|(old_goal, old_heading, _, _)| {
                *old_goal != goal || *old_heading != goal_heading_rad
            })
        {
            self.diagnostics.route_rebuild_reason = self.pending_route_rebuild;
        }
        // Outside-map requests are not quantization failures. Preserve the
        // existing early rejection and unrelated valid cache for these, also
        // when a previous in-map attempt has activated continuous recovery.
        if grid.index(pose.point()).is_none() || grid.index(goal).is_none() {
            return Ok(self.blocked("no_grid_path"));
        }
        if self.recovery_active {
            grid.enable_recovery();
        }
        if !grid.recovery_active() && self.plan_on_grid(pose.point(), goal, &grid).is_none() {
            // Conservative cell inflation can close a physically clear corridor
            // before the ordinary car lattice gets a chance to empty its open
            // set. Reuse the existing continuous full-body recovery domain and
            // its original shared budgets; 2-D failure is not proof of no route.
            if !self.enable_recovery_after_grid_rejection(&grid) {
                self.diagnostics.forward_search = grid.forward_search();
                return Ok(self.blocked("no_grid_path"));
            }
            // A path certified in the old cell domain has no accumulated
            // continuous integration error. Rebuild it in the recovery domain
            // instead of relabelling that cached geometry as newly certified.
            self.note_route_rebuild(RouteRebuildReason::GridConnectivityRejected);
            self.route = None;
            self.continuation_checked = false;
        }
        let mut previous_remaining_length = None;
        // A collision-free cached terminal path may no longer be reachable
        // after tracking drift. Heading error displaces the body even when
        // its center remains on the reference. Replan against the same metric
        // and positional tolerance while there is still room to correct it.
        if goal_heading_rad.is_some()
            && let Some((_, _, route, progress)) = &self.route
            && let Some(window) = PreparedReference::new(
                route,
                *progress,
                self.config.max_speed_mps * self.config.preview_horizon_s + self.config.lookahead_m,
            )
            && let Some(projection) = window.project(pose.point())
            && footprint_pose_error(
                self.config.footprint,
                pose,
                Pose2 {
                    x_m: projection.point.x_m,
                    y_m: projection.point.y_m,
                    yaw_rad: projection.heading_rad,
                },
            ) > self.config.goal_tolerance_m * 0.5
        {
            previous_remaining_length =
                self.route
                    .as_ref()
                    .and_then(|(old_goal, old_heading, route, progress)| {
                        if *old_goal != goal || *old_heading != goal_heading_rad {
                            return None;
                        }
                        continuity::remaining_length(pose, route.get(..=self.via_index)?, *progress)
                    });
            self.note_route_rebuild(RouteRebuildReason::FootprintReferenceDrift);
            self.route = None;
        }
        if self
            .route
            .as_ref()
            .is_none_or(|(old_goal, old_heading, _, _)| {
                *old_goal != goal || *old_heading != goal_heading_rad
            })
        {
            self.diagnostics.route_rebuild_reason = self.pending_route_rebuild;
            self.continuation_checked = false;
            self.route = self
                .kinematic_path_from_with_arrival_repair(
                    pose,
                    self.steering.applied_curvature_per_m,
                    goal,
                    goal_heading_rad,
                    &grid,
                    previous_remaining_length.is_some_and(|length| {
                        let local_bound = (2.0 * self.config.lookahead_m).min(2.0);
                        length <= local_bound && distance <= local_bound
                    }),
                )
                .map(|(mut path, end_pose, end_curvature)| {
                    self.via_index = path.len() - 1;
                    if let ArrivalBehavior::PassThrough {
                        next,
                        next_heading_rad,
                        ..
                    } = arrival
                        && (grid.recovery_active()
                            || self.plan_on_grid(end_pose.point(), next, &grid).is_some())
                        && let Some((continuation, _, _)) = self.kinematic_path_from(
                            end_pose,
                            end_curvature,
                            next,
                            next_heading_rad,
                            &grid,
                        )
                    {
                        // The second leg starts at the actually integrated first
                        // endpoint and curvature; there is no heading teleport.
                        path.extend_from_slice(&continuation[1..]);
                        self.continuation_checked = true;
                    }
                    (goal, goal_heading_rad, path, 0)
                });
            if self.route.is_some() {
                if let Some(old_remaining_distance_m) = previous_remaining_length
                    && let Some((_, _, route, progress)) = &self.route
                    && let Some(new_remaining_distance_m) = route
                        .get(..=self.via_index)
                        .and_then(|leg| continuity::remaining_length(pose, leg, *progress))
                {
                    self.diagnostics.route_length_change = Some(RouteLengthChange {
                        old_remaining_distance_m,
                        new_remaining_distance_m,
                    });
                }
                self.route_revision = self.route_revision.saturating_add(1);
                self.diagnostics.route_revision = self.route_revision;
                self.pending_route_rebuild = None;
            }
        }
        self.recovery_active = grid.recovery_active();
        if self.route.is_some()
            && self.recovery_active
            && self.route_revision != previous_route_revision
        {
            self.recovery_route_error_m = grid.completed_path_error().position_m;
            self.recovery_sample_arc_bound_m = grid.sample_arc_bound_m();
        }
        self.diagnostics.forward_search = grid.forward_search();
        let Some((_, _, route, progress)) = &mut self.route else {
            return Ok(
                self.blocked(if self.terminal_budget.snapshot().budget_exhausted {
                    "terminal_budget_exhausted"
                } else if self.diagnostics.forward_search.node_budget_exhausted {
                    "forward_node_budget_exhausted"
                } else {
                    "no_forward_kinematic_path"
                }),
            );
        };
        // Keep pursuit and scoring anchored to the current mandatory waypoint.
        // The continuation supplies braking room only; it must not let a long
        // lookahead cut the waypoint or let cached progress skip its admission.
        let continuation_distance = if self.continuation_checked
            && route[self.via_index..].windows(2).all(|pair| {
                if self.recovery_active {
                    grid.motion_transition_clear_with_error(
                        pair[0],
                        pair[1],
                        self.recovery_sample_arc_bound_m,
                        self.recovery_route_error_m,
                    )
                } else {
                    grid.transition_clear(pair[0], pair[1])
                }
            }) {
            route[self.via_index..]
                .windows(2)
                .map(|pair| pair[0].distance(pair[1]))
                .sum::<f64>()
        } else {
            self.continuation_checked = false;
            0.0
        };
        let route = &route[..=self.via_index];
        // Follow local progress through the cached forward-car path. Restrict
        // nearest-point search to a short route interval to avoid jumping loops.
        let mut travel = 0.0;
        let mut nearest = *progress;
        let mut nearest_distance = pose.point().distance(route[nearest]);
        for index in (*progress + 1)..route.len() {
            travel += route[index - 1].distance(route[index]);
            if travel > 1.0 {
                break;
            }
            let d = pose.point().distance(route[index]);
            if d < nearest_distance {
                nearest = index;
                nearest_distance = d;
            }
        }
        *progress = nearest;
        self.diagnostics.route_progress = Some(nearest);
        self.diagnostics.route_points = route.len();
        let mut path = vec![pose.point()];
        path.extend_from_slice(&route[(nearest + 1).min(route.len() - 1)..]);
        let remaining_distance = path
            .windows(2)
            .map(|pair| pair[0].distance(pair[1]))
            .sum::<f64>();
        // Feed the real cached route to the tracker: replacing its first point
        // with the current pose would erase the LQR cross-track error.
        let tracking = match self.config.tracking.track(TrackInput {
            pose,
            // The outer gate permits 1e-6 m/s measurement roundoff. Normalize
            // only that accepted edge for the tracker's strictly forward model.
            speed_mps: estimate.speed_mps.clamp(0.0, self.config.max_speed_mps),
            path: route,
            progress: nearest,
            lookahead_m: self.config.lookahead_m,
            max_curvature_per_m: self.config.max_curvature_per_m,
        }) {
            Ok(command) => command,
            Err(error) => {
                self.note_route_rebuild(RouteRebuildReason::TrackingRejected);
                self.route = None;
                return Ok(self.blocked(&format!("path_tracking: {error}")));
            }
        };
        let reference_horizon =
            self.config.max_speed_mps * self.config.preview_horizon_s + self.config.lookahead_m;
        let Some(reference_window) = PreparedReference::new(route, nearest, reference_horizon)
        else {
            return Ok(self.blocked("invalid_local_reference"));
        };
        let Some(current_reference) = reference_window.project(pose.point()) else {
            return Ok(self.blocked("invalid_local_reference"));
        };
        let desired_curvature = tracking.curvature_per_m;
        self.diagnostics.tracking = Some(tracking.diagnostics);
        self.diagnostics.requested_curvature_per_m = Some(desired_curvature);
        let (low, high) = if let Some(constraints) = self.adoption_constraints {
            if constraints.planned_at != now {
                return Ok(self.blocked("invalid_adoption_time"));
            }
            constraints.curvature_interval(&self.config)
        } else {
            let slew = self.config.max_curvature_rate_per_s * dt;
            (
                (self.steering.commanded_curvature_per_m - slew)
                    .max(-self.config.max_curvature_per_m),
                (self.steering.commanded_curvature_per_m + slew)
                    .min(self.config.max_curvature_per_m),
            )
        };
        if !low.is_finite() || !high.is_finite() || low > high {
            return Ok(self.blocked("empty_curvature_interval"));
        }
        let preferred = desired_curvature.clamp(low, high);
        self.diagnostics.slew_limited_curvature_per_m = Some(preferred);
        let mut speed_upper = self
            .config
            .max_speed_mps
            .min(speed_limit_mps)
            .min(estimate.speed_mps + self.config.max_accel_mps2 * dt);
        let mut speed_lower = (estimate.speed_mps - self.config.max_decel_mps2 * dt).max(0.0);
        if let Some(constraints) = self.adoption_constraints {
            let (lower, upper) = constraints.speed_interval(&self.config);
            speed_lower = speed_lower.max(lower);
            speed_upper = speed_upper.min(upper);
        }
        let goal_speed = (2.0
            * self.config.max_decel_mps2
            * (remaining_distance + continuation_distance - self.config.goal_tolerance_m * 0.5)
                .max(0.0))
        .mul_add(1.0, (self.config.max_decel_mps2 * period).powi(2))
        .sqrt()
            - self.config.max_decel_mps2 * period;
        // Respect the next phase's limit before entering its admission radius.
        // This prevents a through waypoint from abruptly changing .3 to .18
        // while the normal deceleration lower bound is still above .18.
        let (goal_speed, waypoint_speed) = if self.continuation_checked
            && let ArrivalBehavior::PassThrough {
                next_max_speed_mps,
                admission_radius_m,
                ..
            } = arrival
        {
            let terminal = next_max_speed_mps.min(self.config.max_speed_mps);
            let available = (distance - admission_radius_m).max(0.0);
            let cap = ((terminal.powi(2)
                + 2.0 * self.config.max_decel_mps2 * available
                + (self.config.max_decel_mps2 * period).powi(2))
            .sqrt()
                - self.config.max_decel_mps2 * period)
                .max(terminal);
            (goal_speed.min(cap), terminal)
        } else {
            (goal_speed, 0.0)
        };
        self.diagnostics.allowed_waypoint_speed_mps = Some(waypoint_speed);
        self.diagnostics.speed_lower_mps = Some(speed_lower);
        self.diagnostics.speed_upper_mps = Some(speed_upper);
        self.diagnostics.goal_speed_cap_mps = Some(goal_speed);
        self.diagnostics.remaining_distance_m = Some(remaining_distance);
        self.diagnostics.checked_continuation_distance_m = Some(continuation_distance);
        self.diagnostics.continuation_checked = self.continuation_checked;
        // Protect an already feasible short, oriented arrival, including a
        // heading-constrained through gate. Task admission still uses the real
        // measured pose; this guard neither reports Reached nor requests Stop.
        // Point-only through waypoints retain their existing branch.
        // A held-curvature
        // preview cannot represent both turns of an S, so its average score
        // alone may trade away the ability to finish on the next control tick.
        // A certified single-arc arrival retains the existing controller.
        // Certification depends on each solver's distance/iteration budget;
        // failure does not imply that two turns are geometrically necessary.
        // Neither bounded solver reruns the lattice per candidate.
        let protected_connection = goal_heading_rad.and_then(|heading| {
            if pose.world_to_body(goal).x_m <= 0.0
                || !(1e-6..=(2.0 * self.config.lookahead_m).min(2.0)).contains(&distance)
            {
                return None;
            }
            self.diagnostics.terminal_connections_checked += 1;
            if terminal::single_arc(
                pose,
                self.steering.applied_curvature_per_m,
                goal,
                Some(heading),
                &self.config,
                &grid,
                &self.terminal_budget,
            )
            .is_some()
            {
                return None;
            }
            terminal_connection(
                &self.config,
                &self.terminal_budget,
                self.terminal_seed,
                self.last_step,
                pose,
                self.steering.applied_curvature_per_m,
                goal,
                heading,
                &grid,
                primitive_envelope::ErrorBound::default(),
            )
            .map(|(connection, keep_family)| (heading, connection.seed, keep_family))
        });
        if self.terminal_budget.snapshot().budget_exhausted {
            return Ok(self.blocked("terminal_budget_exhausted"));
        }
        self.diagnostics.terminal_continuity_enforced = protected_connection.is_some();
        let mut best: Option<EvaluatedCandidate> = None;
        if protected_connection.is_some() {
            let max_next_distance = (distance
                + period * estimate.speed_mps.max(speed_upper).max(0.0))
            .min((2.0 * self.config.lookahead_m).min(2.0));
            self.terminal_budget
                .reserve_continuation(max_next_distance, &grid);
        }
        // Store only candidates that passed every original rollout gate. A
        // second bounded terminal solve is considered only if the original
        // family certifies no candidate. Cold work cannot consume the reserved
        // continuation allowance; any ordinary certified best stays preferred.
        let mut terminal_recovery = Vec::new();
        for index in 0..=self.config.curvature_samples {
            let curvature = if index == self.config.curvature_samples {
                preferred
            } else {
                low + (high - low) * index as f64 / (self.config.curvature_samples - 1) as f64
            };
            let turning_speed = if curvature.abs() > 1e-9 {
                (self.config.max_lateral_accel_mps2 / curvature.abs()).sqrt()
            } else {
                self.config.max_speed_mps
            };
            let nominal = speed_upper.min(turning_speed).min(goal_speed);
            if nominal + 1e-9 < speed_lower {
                self.diagnostics.candidates.curvature_speed_limits += 1;
                continue; // A normal candidate cannot exceed braking/turning limits.
            }
            for fraction in [1.0, 0.5] {
                let speed = (nominal * fraction).max(speed_lower).min(nominal);
                if speed < 1e-6 {
                    self.diagnostics.candidates.near_zero_speed += 1;
                    continue;
                }
                if let Some(constraints) = self.adoption_constraints
                    && let Err(reason) = constraints.check_command(
                        &self.config,
                        speed,
                        curvature,
                        obstacles,
                        self.travel_boundary,
                    )
                {
                    self.diagnostics.candidates.admission_current += 1;
                    self.diagnostics
                        .current_admission_failure
                        .get_or_insert(reason);
                    continue;
                }
                self.diagnostics.candidates.rollouts_evaluated += 1;
                let prediction = match rollout_with_stopping_policy(
                    &self.config,
                    pose,
                    estimate.speed_mps.max(0.0),
                    speed,
                    self.steering.applied_curvature_per_m,
                    curvature,
                    obstacles,
                    remaining_distance,
                    &grid,
                    goal_heading_rad.map(|goal_heading_rad| RolloutReference {
                        cursor: reference_window.cursor_to_end(),
                        initial_arc_m: current_reference.arc_m,
                        goal_heading_rad,
                    }),
                    self.target_policy,
                    self.adoption_constraints.map_or(0.0, |c| c.speed_bound_mps),
                ) {
                    Ok(prediction) => prediction,
                    Err(reason) => {
                        let count = match reason {
                            RolloutRejection::StopReachable => {
                                &mut self.diagnostics.candidates.stop_reachable
                            }
                            RolloutRejection::Grid => &mut self.diagnostics.candidates.grid,
                            RolloutRejection::Footprint => {
                                &mut self.diagnostics.candidates.footprint
                            }
                            RolloutRejection::SampleBudget => {
                                &mut self.diagnostics.candidates.sample_budget
                            }
                            RolloutRejection::LateralAcceleration => {
                                &mut self.diagnostics.candidates.lateral_acceleration
                            }
                            RolloutRejection::Reference => {
                                &mut self.diagnostics.candidates.reference
                            }
                        };
                        *count += 1;
                        continue;
                    }
                };
                // The multi-source fallback forecast is pure and expensive.
                // Run it only after the normal motion and collision gates; a
                // survivor must still pass it before terminal work or selection.
                if let Some(constraints) = self.adoption_constraints {
                    let (forecast, work) = constraints.check_next_with_work(
                        &self.config,
                        pose,
                        speed,
                        curvature,
                        obstacles,
                        self.travel_boundary,
                    );
                    let total = &mut self.diagnostics.admission_forecast_work;
                    total.adoption_scenarios = total
                        .adoption_scenarios
                        .saturating_add(work.adoption_scenarios);
                    total.source_checks = total.source_checks.saturating_add(work.source_checks);
                    total.projection_calls =
                        total.projection_calls.saturating_add(work.projection_calls);
                    total.projection_intervals = total
                        .projection_intervals
                        .saturating_add(work.projection_intervals);
                    if let Err(reason) = forecast {
                        self.diagnostics.candidates.admission_next += 1;
                        self.diagnostics
                            .next_admission_failure
                            .get_or_insert(reason);
                        continue;
                    }
                }
                let Some(reference) = reference_window.project(prediction.endpoint.point()) else {
                    continue;
                };
                let progress_m = reference.arc_m - current_reference.arc_m;
                let reference_heading =
                    if remaining_distance - progress_m <= self.config.goal_tolerance_m {
                        goal_heading_rad.unwrap_or(reference.heading_rad)
                    } else {
                        reference.heading_rad
                    };
                let heading_error = angle_error(prediction.endpoint.yaw_rad, reference_heading);
                let transition = MotionTransition {
                    initial_speed_mps: estimate.speed_mps.max(0.0),
                    target_speed_mps: speed,
                    initial_curvature_per_m: self.steering.applied_curvature_per_m,
                    target_curvature_per_m: curvature,
                    max_accel_mps2: self.config.max_accel_mps2,
                    max_decel_mps2: self.config.max_decel_mps2,
                    max_curvature_rate_per_s: self.config.max_curvature_rate_per_s,
                };
                let score = if let Some(cost) = prediction.reference_cost_m {
                    // Oriented goals need the whole approach, not just a final
                    // point: constant-curvature previews can hide intermediate
                    // errors across a bend. Steering preference has the scale
                    // of one tick's lateral displacement, so it cannot dominate
                    // the remaining path's position and heading corrections.
                    cost + 0.5
                        * (speed * period).powi(2)
                        * ((curvature - desired_curvature).abs()
                            + (curvature - self.steering.commanded_curvature_per_m).abs())
                        - 0.3 * speed
                } else {
                    // Point-only waypoints retain their pursuit target. Fade
                    // steering preference near arrival as its geometric effect
                    // falls quadratically with remaining travel distance.
                    let approach_scale = (remaining_distance / self.config.lookahead_m).min(1.0);
                    prediction.endpoint.point().distance(tracking.target)
                        + 0.15 * approach_scale.powi(2) * (curvature - desired_curvature).abs()
                        + 0.08
                            * approach_scale.powi(2)
                            * (curvature - self.steering.commanded_curvature_per_m).abs()
                        - 0.3 * speed
                };
                let mut next_projection = None;
                let mut next_error = primitive_envelope::ErrorBound::default();
                let mut candidate_terminal_seed = None;
                if let Some((heading, _, _)) = protected_connection {
                    next_projection = if grid.recovery_active() {
                        primitive_envelope::predict(pose, transition, period).and_then(|bounded| {
                            next_error = next_error.advance(&bounded)?;
                            Some(bounded.projection)
                        })
                    } else {
                        project_motion(pose, transition, period)
                    };
                    let Some(next) = next_projection else {
                        self.diagnostics.candidates.terminal_unreachable += 1;
                        continue;
                    };
                    let inside_goal = next.pose.point().distance(goal) + next_error.position_m
                        <= self.config.goal_tolerance_m
                        && angle_error(next.pose.yaw_rad, heading).abs() + next_error.heading_rad
                            <= self.config.goal_heading_tolerance_rad;
                    if !inside_goal {
                        self.terminal_budget.begin_cold_candidate();
                        self.diagnostics.terminal_connections_checked += 1;
                        if terminal::single_arc_with_error(
                            next.pose,
                            next.curvature_per_m,
                            goal,
                            Some(heading),
                            &self.config,
                            &grid,
                            &self.terminal_budget,
                            next_error,
                        )
                        .is_none()
                        {
                            if let Some(connection) = protected_connection
                                .filter(|(_, _, used_seed)| *used_seed)
                                .and_then(|(_, seed, _)| seed.after_travel(next.distance_m))
                                .and_then(|seed| {
                                    terminal::continue_two_arc_with_error(
                                        next.pose,
                                        next.curvature_per_m,
                                        goal,
                                        heading,
                                        &self.config,
                                        &grid,
                                        &self.terminal_budget,
                                        seed,
                                        next_error,
                                    )
                                })
                                .or_else(|| {
                                    terminal::two_arc_with_error(
                                        next.pose,
                                        next.curvature_per_m,
                                        goal,
                                        heading,
                                        &self.config,
                                        &grid,
                                        &self.terminal_budget,
                                        None,
                                        next_error,
                                    )
                                })
                            {
                                candidate_terminal_seed = now
                                    .0
                                    .checked_add(self.config.control_period_ms)
                                    .map(|at| CachedTerminalSeed {
                                        anchor_at: Timestamp(at),
                                        anchor_pose: next.pose,
                                        prefer_continuation: protected_connection
                                            .is_some_and(|(_, _, keep)| keep)
                                            && connection.used_seed,
                                        goal,
                                        heading,
                                        seed: connection.seed,
                                    });
                            } else {
                                if self.terminal_budget.snapshot().budget_exhausted {
                                    self.diagnostics.candidates.terminal_budget += 1;
                                } else {
                                    terminal_recovery.push(EvaluatedCandidate {
                                        score,
                                        intent: MotionIntent {
                                            speed_mps: speed,
                                            curvature_per_m: curvature,
                                        },
                                        prediction,
                                        reference,
                                        heading_error,
                                        progress_m,
                                        transition,
                                        next_projection,
                                        next_error,
                                        terminal_seed: None,
                                        terminal_deferred: self.terminal_budget.cold_deferred(),
                                    });
                                    if self.terminal_budget.cold_deferred() {
                                        self.diagnostics.candidates.terminal_budget += 1;
                                    } else {
                                        self.diagnostics.candidates.terminal_unreachable += 1;
                                    }
                                }
                                continue;
                            }
                        }
                    }
                }
                self.diagnostics.candidates.accepted += 1;
                if best.as_ref().is_none_or(|old| {
                    // A rebased hint rescued this family only after the original
                    // solvers failed. As in terminal recovery, prefer its certified
                    // first segment; a held-curvature score cannot represent both
                    // turns. Outside that family the original score is unchanged.
                    if let Some((_, seed, true)) = protected_connection {
                        let continued =
                            candidate_terminal_seed.is_some_and(|s| s.prefer_continuation);
                        let old_continued =
                            old.terminal_seed.is_some_and(|s| s.prefer_continuation);
                        if continued != old_continued {
                            return continued;
                        }
                        if continued {
                            let difference = (curvature - seed.first_curvature()).abs().total_cmp(
                                &(old.intent.curvature_per_m - seed.first_curvature()).abs(),
                            );
                            if !difference.is_eq() {
                                return difference.is_lt();
                            }
                        }
                    }
                    score < old.score
                }) {
                    best = Some(EvaluatedCandidate {
                        score,
                        intent: MotionIntent {
                            speed_mps: speed,
                            curvature_per_m: curvature,
                        },
                        prediction,
                        reference,
                        heading_error,
                        progress_m,
                        transition,
                        next_projection,
                        next_error,
                        terminal_seed: candidate_terminal_seed,
                        terminal_deferred: false,
                    });
                }
            }
        }
        self.terminal_budget.release_continuation();
        if best.is_none()
            && let Some((heading, seed, _)) = protected_connection
        {
            // Spend recovery work on actions closest to the certified first
            // control segment before trying large departures from that segment.
            // All original motion/collision constraints still apply. Recovery
            // takes the first certified continuation; normal candidates retain
            // their original score.
            terminal_recovery.sort_by(|a, b| {
                (a.intent.curvature_per_m - seed.first_curvature())
                    .abs()
                    .total_cmp(&(b.intent.curvature_per_m - seed.first_curvature()).abs())
                    .then_with(|| a.score.total_cmp(&b.score))
            });
            for mut candidate in terminal_recovery.iter().copied() {
                if self.terminal_budget.snapshot().budget_exhausted {
                    if !candidate.terminal_deferred {
                        self.diagnostics.candidates.terminal_unreachable -= 1;
                        self.diagnostics.candidates.terminal_budget += 1;
                    }
                    continue;
                }
                let next = candidate
                    .next_projection
                    .expect("recovery candidates have a full-period projection");
                let Some(remaining_seed) = seed.after_travel(next.distance_m) else {
                    if candidate.terminal_deferred {
                        self.diagnostics.candidates.terminal_budget -= 1;
                        self.diagnostics.candidates.terminal_unreachable += 1;
                    }
                    continue;
                };
                self.diagnostics.terminal_connections_checked += 1;
                if let Some(connection) = terminal::continue_two_arc_with_error(
                    next.pose,
                    next.curvature_per_m,
                    goal,
                    heading,
                    &self.config,
                    &grid,
                    &self.terminal_budget,
                    remaining_seed,
                    candidate.next_error,
                ) {
                    if candidate.terminal_deferred {
                        self.diagnostics.candidates.terminal_budget -= 1;
                    } else {
                        self.diagnostics.candidates.terminal_unreachable -= 1;
                    }
                    self.diagnostics.candidates.accepted += 1;
                    candidate.terminal_seed =
                        now.0.checked_add(self.config.control_period_ms).map(|at| {
                            CachedTerminalSeed {
                                anchor_at: Timestamp(at),
                                anchor_pose: next.pose,
                                prefer_continuation: protected_connection
                                    .is_some_and(|(_, _, keep)| keep)
                                    && connection.used_seed,
                                goal,
                                heading,
                                seed: connection.seed,
                            }
                        });
                    // Recovery executes the closest certified first segment;
                    // a held-curvature preview cannot score both future turns.
                    best = Some(candidate);
                    break;
                } else if self.terminal_budget.snapshot().budget_exhausted {
                    if !candidate.terminal_deferred {
                        self.diagnostics.candidates.terminal_unreachable -= 1;
                        self.diagnostics.candidates.terminal_budget += 1;
                    }
                } else if candidate.terminal_deferred {
                    // It was deferred in the cold round, but the complete
                    // continued-family attempt has now rejected it without
                    // exhausting the global allowance.
                    self.diagnostics.candidates.terminal_budget -= 1;
                    self.diagnostics.candidates.terminal_unreachable += 1;
                }
            }
            if best.is_none() && self.target_policy == TargetPolicy::RollingLocal {
                // Exact-center continuations can all fail even though a real
                // integrated endpoint reaches the unchanged arrival region.
                // Only after every original candidate failed, certify that
                // smaller region with the same ledger and projected error.
                // This does not report task arrival or adopt the command.
                for candidate in terminal_recovery {
                    if self.terminal_budget.snapshot().budget_exhausted {
                        break;
                    }
                    let next = candidate
                        .next_projection
                        .expect("recovery candidates have a full-period projection");
                    self.diagnostics.terminal_connections_checked += 1;
                    if terminal::oriented_arrival_region(
                        next.pose,
                        next.curvature_per_m,
                        goal,
                        heading,
                        &self.config,
                        &grid,
                        &self.terminal_budget,
                        candidate.next_error,
                    )
                    .is_some()
                    {
                        self.diagnostics.candidates.terminal_unreachable -= 1;
                        self.diagnostics.candidates.accepted += 1;
                        best = Some(candidate);
                        break;
                    }
                }
            }
        }
        self.terminal_seed = best.as_ref().and_then(|candidate| candidate.terminal_seed);
        if let Some(candidate) = best {
            let intent = candidate.intent;
            self.diagnostics.selected_curvature_per_m = Some(intent.curvature_per_m);
            self.diagnostics.selected_prediction_distance_m = Some(candidate.prediction.distance_m);
            self.diagnostics.selected_prediction_endpoint = Some(candidate.prediction.endpoint);
            self.diagnostics.selected_cross_track_m = Some(candidate.reference.distance_m);
            self.diagnostics.selected_heading_error_rad = Some(candidate.heading_error);
            self.diagnostics.selected_progress_m = Some(candidate.progress_m);
            self.diagnostics.selected_reference_cost_m = candidate.prediction.reference_cost_m;
            self.diagnostics.selected_braking_envelope_margin =
                candidate.prediction.stopping_envelope_margin;
            self.diagnostics.selected_stopping_margin = Some(stopping_margin(
                &self.config,
                pose,
                estimate.speed_mps.max(intent.speed_mps),
                obstacles,
                self.travel_boundary,
            ));
            self.diagnostics.selected_next_stopping_margin = candidate
                .next_projection
                .or_else(|| project_motion(pose, candidate.transition, period))
                .map(|next| {
                    stopping_margin(
                        &self.config,
                        next.pose,
                        next.speed_mps,
                        obstacles,
                        self.travel_boundary,
                    )
                });
            Ok(NavigationDecision {
                status: NavigationStatus::Driving,
                intent,
                path,
                reason: None,
                diagnostics: self.diagnostics_snapshot(),
            })
        } else {
            self.note_route_rebuild(RouteRebuildReason::NoAcceptedCandidate);
            self.route = None; // changed observations require a new bounded search.
            let reason = if self.diagnostics.candidates.rollouts_evaluated == 0
                && self.diagnostics.candidates.curvature_speed_limits > 0
            {
                "empty_speed_interval"
            } else if self.terminal_budget.snapshot().budget_exhausted {
                "terminal_budget_exhausted"
            } else {
                "no_collision_free_braking_trajectory"
            };
            let mut decision = self.blocked(reason);
            decision.path = path;
            Ok(decision)
        }
    }

    fn note_route_rebuild(&mut self, reason: RouteRebuildReason) {
        self.pending_route_rebuild.get_or_insert(reason);
    }

    fn diagnostics_snapshot(&self) -> NavigationDiagnostics {
        NavigationDiagnostics {
            terminal_work: self.terminal_budget.snapshot(),
            ..self.diagnostics
        }
    }

    fn blocked(&mut self, reason: &str) -> NavigationDecision {
        let mut decision = NavigationDecision::stopped(NavigationStatus::Blocked, reason);
        decision.diagnostics = self.diagnostics_snapshot();
        decision
    }

    /// Forward-only lattice search supplements 2-D A*: a short grid route may
    /// be impossible for a car with bounded steering. States contain heading
    /// and curvature; primitives ramp steering at the configured rate and are
    /// sampled inside the same inflated occupancy domain as the local rollout.
    #[cfg(test)]
    fn kinematic_path(
        &self,
        start: Pose2,
        goal: Point2,
        goal_heading_rad: Option<f64>,
        grid: &Grid,
    ) -> Option<Vec<Point2>> {
        self.kinematic_path_from(
            start,
            self.steering.applied_curvature_per_m,
            goal,
            goal_heading_rad,
            grid,
        )
        .map(|(path, _, _)| path)
    }

    fn kinematic_path_from(
        &self,
        start: Pose2,
        initial_curvature: f64,
        goal: Point2,
        goal_heading_rad: Option<f64>,
        grid: &Grid,
    ) -> Option<(Vec<Point2>, Pose2, f64)> {
        self.kinematic_path_from_with_arrival_repair(
            start,
            initial_curvature,
            goal,
            goal_heading_rad,
            grid,
            false,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn kinematic_path_from_with_arrival_repair(
        &self,
        start: Pose2,
        initial_curvature: f64,
        goal: Point2,
        goal_heading_rad: Option<f64>,
        grid: &Grid,
        repair_short_arrival: bool,
    ) -> Option<(Vec<Point2>, Pose2, f64)> {
        // Try one bounded smooth approach before quantizing into lattice bins.
        // Small sideways corrections with the same final heading need an S,
        // which the five steering bins can otherwise replace with a full loop.
        if let Some(heading) = goal_heading_rad
            && let Some((connection, _)) = terminal_connection(
                &self.config,
                &self.terminal_budget,
                self.terminal_seed,
                self.last_step,
                start,
                initial_curvature,
                goal,
                heading,
                grid,
                grid.completed_path_error(),
            )
        {
            grid.recovery.next_leg_error.set(connection.error);
            if !grid.recovery_active() {
                grid.recovery.normal_path_completed.set(true);
            } else if grid.forward_search().grid_connectivity_rejected {
                let mut diagnostics = grid.forward_search();
                diagnostics.recovery_accepted = true;
                grid.recovery.diagnostics.set(diagnostics);
            }
            let (points, end_pose, end_curvature) = connection.into_path();
            let mut path = Vec::with_capacity(points.len() + 1);
            path.push(start.point());
            path.extend(points);
            return Some((path, end_pose, end_curvature));
        }
        if self.terminal_budget.snapshot().budget_exhausted {
            return None;
        }
        // A rolling, unoriented local target can lie beyond the old lattice
        // terminal's 35 cm domain while still admitting one short forward arc.
        // Certify that arc before a quantized search replaces it with a loop.
        // Existing oriented targets and the old near-terminal domain retain
        // their original order. Failure leaves all spent work on this ledger.
        if self.target_policy == TargetPolicy::RollingLocal
            && goal_heading_rad.is_none()
            && (0.35..(2.0 * self.config.lookahead_m).min(2.0))
                .contains(&start.point().distance(goal))
            && let Some((points, endpoint, curvature, error)) = terminal::short_point_arc_with_error(
                start,
                initial_curvature,
                goal,
                &self.config,
                grid,
                &self.terminal_budget,
                grid.completed_path_error(),
            )
        {
            grid.recovery.next_leg_error.set(error);
            if grid.recovery_active() {
                let mut diagnostics = grid.forward_search();
                diagnostics.recovery_accepted = true;
                grid.recovery.diagnostics.set(diagnostics);
            } else {
                grid.recovery.normal_path_completed.set(true);
            }
            let mut path = Vec::with_capacity(points.len() + 1);
            path.push(start.point());
            path.extend(points);
            return Some((path, endpoint, curvature));
        }
        // Fixed routes retain the existing measured-drift repair condition.
        // Online rolling targets also re-certify a nearby oriented arrival
        // after a fresh boundary invalidates its cache. All strict connectors
        // keep priority; the original bounded arrival region then precedes a
        // lattice loop, using the CURRENT boundary and the SAME work ledger.
        // The 35 cm bound is the existing terminal primitive's local domain.
        if goal_heading_rad.is_some()
            && (repair_short_arrival
                || (self.target_policy == TargetPolicy::RollingLocal
                    && (1e-9..0.35).contains(&start.point().distance(goal))))
        {
            if let Some((points, endpoint, curvature, error)) = terminal::single_arc_with_error(
                start,
                initial_curvature,
                goal,
                goal_heading_rad,
                &self.config,
                grid,
                &self.terminal_budget,
                grid.completed_path_error(),
            ) {
                grid.recovery.next_leg_error.set(error);
                if grid.recovery_active() {
                    let mut diagnostics = grid.forward_search();
                    diagnostics.recovery_accepted = true;
                    grid.recovery.diagnostics.set(diagnostics);
                } else {
                    grid.recovery.normal_path_completed.set(true);
                }
                // The strict wrapper returns its integrated samples; normal
                // and recovery paths retain their existing domain checks.
                let mut path = Vec::with_capacity(points.len() + 1);
                path.push(start.point());
                path.extend(points);
                return Some((path, endpoint, curvature));
            }
            if let Some(path) =
                self.certify_arrival_region(start, initial_curvature, goal, goal_heading_rad, grid)
            {
                return Some(path);
            }
        }
        let consumed = grid.forward_search();
        let remaining_nodes = self
            .config
            .max_grid_cells
            .saturating_sub(consumed.ordinary.allocated_nodes)
            .saturating_sub(consumed.recovery.allocated_nodes);
        let mut work = recovery::ForwardSearchWork::default();
        let result = self.search_kinematic_path_from(
            start,
            initial_curvature,
            goal,
            goal_heading_rad,
            grid,
            remaining_nodes,
            &mut work,
        );
        let mut diagnostics = grid.forward_search();
        diagnostics.node_budget_exhausted |= work.exit == recovery::ForwardSearchExit::NodeBudget;
        if grid.recovery_active() {
            diagnostics.recovery.accumulate(work);
            diagnostics.recovery_attempted = true;
            diagnostics.recovery_accepted |= result.is_some();
            grid.recovery.diagnostics.set(diagnostics);
            return result.or_else(|| {
                self.arrival_region_after_search(
                    start,
                    initial_curvature,
                    goal,
                    goal_heading_rad,
                    grid,
                    work.exit,
                )
            });
        }
        diagnostics.ordinary.accumulate(work);
        grid.recovery.diagnostics.set(diagnostics);
        if result.is_some() {
            grid.recovery.normal_path_completed.set(true);
        }
        // A tight quantized exit can exhaust the complete ordinary frontier
        // although the actual full circumscribed body has clearance. Only that
        // bounded-search outcome enables continuous-capsule recovery; ordinary
        // successful paths retain their original priority. A final arrival
        // region solve uses only remaining terminal work, never lattice nodes.
        if result.is_some() || grid.recovery.normal_path_completed.get() {
            return result;
        }
        if work.exit != recovery::ForwardSearchExit::OpenEmpty {
            return self.arrival_region_after_search(
                start,
                initial_curvature,
                goal,
                goal_heading_rad,
                grid,
                work.exit,
            );
        }
        let consumed = grid.forward_search();
        let remaining_nodes = self
            .config
            .max_grid_cells
            .saturating_sub(consumed.ordinary.allocated_nodes)
            .saturating_sub(consumed.recovery.allocated_nodes);
        if remaining_nodes == 0 || self.terminal_budget.snapshot().budget_exhausted {
            if remaining_nodes == 0 {
                let mut diagnostics = grid.forward_search();
                diagnostics.node_budget_exhausted = true;
                grid.recovery.diagnostics.set(diagnostics);
            }
            return self.arrival_region_after_search(
                start,
                initial_curvature,
                goal,
                goal_heading_rad,
                grid,
                work.exit,
            );
        }
        grid.enable_recovery();
        let mut recovery_work = recovery::ForwardSearchWork::default();
        let result = self.search_kinematic_path_from(
            start,
            initial_curvature,
            goal,
            goal_heading_rad,
            grid,
            remaining_nodes,
            &mut recovery_work,
        );
        let mut diagnostics = grid.forward_search();
        diagnostics.node_budget_exhausted |=
            recovery_work.exit == recovery::ForwardSearchExit::NodeBudget;
        diagnostics.recovery.accumulate(recovery_work);
        diagnostics.recovery_attempted = true;
        diagnostics.recovery_accepted |= result.is_some();
        grid.recovery.diagnostics.set(diagnostics);
        result.or_else(|| {
            self.arrival_region_after_search(
                start,
                initial_curvature,
                goal,
                goal_heading_rad,
                grid,
                recovery_work.exit,
            )
        })
    }

    /// An oriented arrival is a region, although the preferred connectors solve
    /// for its center. After those connectors and the applicable lattice search
    /// fail, certify a bounded ramp into the unchanged region using the same
    /// accumulated error, geometry and terminal ledger. Node exhaustion does
    /// not allocate additional nodes or erase the recorded search failure.
    #[allow(clippy::too_many_arguments)]
    fn arrival_region_after_search(
        &self,
        start: Pose2,
        initial_curvature: f64,
        goal: Point2,
        goal_heading_rad: Option<f64>,
        grid: &Grid,
        exit: recovery::ForwardSearchExit,
    ) -> Option<(Vec<Point2>, Pose2, f64)> {
        if !matches!(
            exit,
            recovery::ForwardSearchExit::OpenEmpty | recovery::ForwardSearchExit::NodeBudget
        ) {
            return None;
        }
        self.certify_arrival_region(start, initial_curvature, goal, goal_heading_rad, grid)
    }

    fn certify_arrival_region(
        &self,
        start: Pose2,
        initial_curvature: f64,
        goal: Point2,
        goal_heading_rad: Option<f64>,
        grid: &Grid,
    ) -> Option<(Vec<Point2>, Pose2, f64)> {
        let connect = if self.target_policy == TargetPolicy::RollingLocal {
            terminal::rolling_oriented_arrival_region
        } else {
            terminal::oriented_arrival_region
        };
        let (points, endpoint, curvature, error) = connect(
            start,
            initial_curvature,
            goal,
            goal_heading_rad?,
            &self.config,
            grid,
            &self.terminal_budget,
            grid.completed_path_error(),
        )?;
        grid.recovery.next_leg_error.set(error);
        if grid.recovery_active() {
            let mut diagnostics = grid.forward_search();
            diagnostics.recovery_accepted = true;
            grid.recovery.diagnostics.set(diagnostics);
        } else {
            grid.recovery.normal_path_completed.set(true);
        }
        let mut path = Vec::with_capacity(points.len() + 1);
        path.push(start.point());
        path.extend(points);
        Some((path, endpoint, curvature))
    }

    fn enable_recovery_after_grid_rejection(&self, grid: &Grid) -> bool {
        let mut diagnostics = grid.forward_search();
        diagnostics.grid_connectivity_rejected = true;
        let remaining_nodes = self
            .config
            .max_grid_cells
            .saturating_sub(diagnostics.ordinary.allocated_nodes)
            .saturating_sub(diagnostics.recovery.allocated_nodes);
        if remaining_nodes == 0 {
            diagnostics.node_budget_exhausted = true;
        }
        grid.recovery.diagnostics.set(diagnostics);
        if remaining_nodes == 0 || self.terminal_budget.snapshot().budget_exhausted {
            return false;
        }
        grid.enable_recovery();
        let mut diagnostics = grid.forward_search();
        diagnostics.recovery_attempted = true;
        grid.recovery.diagnostics.set(diagnostics);
        true
    }

    #[allow(clippy::too_many_arguments)]
    fn search_kinematic_path_from(
        &self,
        start: Pose2,
        initial_curvature: f64,
        goal: Point2,
        goal_heading_rad: Option<f64>,
        grid: &Grid,
        node_limit: usize,
        work: &mut recovery::ForwardSearchWork,
    ) -> Option<(Vec<Point2>, Pose2, f64)> {
        if node_limit == 0 {
            work.exit = recovery::ForwardSearchExit::NodeBudget;
            return None;
        }
        let mut nodes = vec![CarNode {
            pose: start,
            curvature: initial_curvature,
            parent: None,
            cost: 0.0,
            error: grid.completed_path_error(),
        }];
        work.allocated_nodes = 1;
        let mut best = HashMap::new();
        let Some(start_key) = car_key(
            start,
            initial_curvature,
            grid,
            self.config.max_curvature_per_m,
        ) else {
            work.exit = recovery::ForwardSearchExit::InvalidStart;
            return None;
        };
        best.insert(start_key, 0usize);
        let mut open = BinaryHeap::new();
        open.push(QueueNode {
            index: 0,
            score: start.point().distance(goal),
        });
        let length = (2.0 * grid.resolution).clamp(0.15, 0.4);
        while let Some(entry) = open.pop() {
            // Every successful lattice route still needs a terminal connector.
            // An exhausted ledger cannot certify one, so stop this search.
            if self.terminal_budget.snapshot().budget_exhausted {
                work.exit = recovery::ForwardSearchExit::TerminalBudget;
                return None;
            }
            let node = nodes[entry.index];
            let key = car_key(
                node.pose,
                node.curvature,
                grid,
                self.config.max_curvature_per_m,
            )?;
            if best.get(&key) != Some(&entry.index) {
                continue;
            }
            work.expanded_nodes += 1;
            // Recovery can reach the unchanged oriented arrival region with a
            // fully certified primitive whose endpoint has passed the goal's
            // center. Do not force another forward connector back to that
            // center, and never replace the integrated endpoint with the goal.
            if grid.recovery_active()
                && goal_heading_rad.is_some_and(|heading| {
                    node.pose.point().distance(goal) + node.error.position_m
                        <= self.config.goal_tolerance_m
                        && angle_error(node.pose.yaw_rad, heading).abs() + node.error.heading_rad
                            <= self.config.goal_heading_tolerance_rad
                })
                && entry.index != 0
            {
                let (path, error) = car_path_from_nodes(&nodes, entry.index, &self.config, grid)?;
                grid.recovery.next_leg_error.set(error);
                work.exit = recovery::ForwardSearchExit::Found;
                return Some((path, node.pose, node.curvature));
            }
            let local_goal = node.pose.world_to_body(goal);
            let squared = local_goal.x_m.powi(2) + local_goal.y_m.powi(2);
            if squared < 0.35_f64.powi(2)
                && local_goal.x_m > 0.0
                && let Some((terminal, end_pose, end_curvature, error)) =
                    terminal::single_arc_with_error(
                        node.pose,
                        node.curvature,
                        goal,
                        goal_heading_rad,
                        &self.config,
                        grid,
                        &self.terminal_budget,
                        node.error,
                    )
            {
                let (mut path, _) = car_path_from_nodes(&nodes, entry.index, &self.config, grid)?;
                path.extend(terminal);
                grid.recovery.next_leg_error.set(error);
                work.exit = recovery::ForwardSearchExit::Found;
                return Some((path, end_pose, end_curvature));
            }
            for curvature in
                [-1.0, -0.5, 0.0, 0.5, 1.0].map(|f| f * self.config.max_curvature_per_m)
            {
                work.primitive_attempts += 1;
                let primitive = car_primitive_with_error(
                    node.pose,
                    node.curvature,
                    curvature,
                    length,
                    &self.config,
                    grid,
                    node.error,
                );
                let (_, next_pose, curvature, error) = match primitive {
                    Ok(primitive) => primitive,
                    Err(failure) => {
                        work.primitive_rejections += 1;
                        work.first_primitive_rejection.get_or_insert(failure);
                        continue;
                    }
                };
                let Some(key) =
                    car_key(next_pose, curvature, grid, self.config.max_curvature_per_m)
                else {
                    continue;
                };
                let cost = node.cost + length + 0.01 * (curvature - node.curvature).abs();
                if best
                    .get(&key)
                    .is_some_and(|index| nodes[*index].cost <= cost)
                {
                    continue;
                }
                if nodes.len() >= node_limit {
                    work.exit = recovery::ForwardSearchExit::NodeBudget;
                    return None;
                }
                let index = nodes.len();
                nodes.push(CarNode {
                    pose: next_pose,
                    curvature,
                    parent: Some(entry.index),
                    cost,
                    error,
                });
                work.generated_nodes += 1;
                work.allocated_nodes += 1;
                best.insert(key, index);
                open.push(QueueNode {
                    index,
                    // A weighted goal/heading heuristic prioritizes feasible
                    // forward approaches inside a hard work budget. It does
                    // not claim a shortest path or search completeness.
                    score: cost
                        + car_heuristic(
                            next_pose,
                            goal,
                            goal_heading_rad,
                            self.config.max_curvature_per_m,
                        ),
                });
            }
        }
        work.exit = recovery::ForwardSearchExit::OpenEmpty;
        None
    }

    /// Circumscribed-body inflation makes grid search independent of car heading.
    pub fn plan_path(
        &self,
        start: Point2,
        goal: Point2,
        obstacles: &[ObstacleDisc],
    ) -> Option<Vec<Point2>> {
        if !start.valid()
            || !goal.valid()
            || obstacles.len() > self.config.max_obstacles
            || obstacles
                .iter()
                .any(|o| !o.center.valid() || !o.radius_m.is_finite() || o.radius_m < 0.0)
        {
            return None;
        }
        let mut grid = Grid::with_boundary(&self.config, obstacles, self.travel_boundary);
        grid.recovery.oriented_boundary = self.target_policy == TargetPolicy::RollingLocal;
        self.plan_on_grid(start, goal, &grid)
    }

    fn plan_on_grid(&self, start: Point2, goal: Point2, grid: &Grid) -> Option<Vec<Point2>> {
        let first = grid.index(start)?;
        let last = grid.index(goal)?;
        if grid.blocked[first] || grid.blocked[last] {
            return None;
        }
        if first == last {
            return Some(vec![start, goal]);
        }
        let mut costs = vec![f64::INFINITY; grid.blocked.len()];
        let mut parent = vec![usize::MAX; grid.blocked.len()];
        let mut closed = vec![false; grid.blocked.len()];
        let mut open = BinaryHeap::new();
        costs[first] = 0.0;
        open.push(QueueNode {
            index: first,
            score: grid.point(first).distance(grid.point(last)),
        });
        while let Some(node) = open.pop() {
            if closed[node.index] {
                continue;
            }
            if node.index == last {
                let mut route = vec![goal];
                let mut current = last;
                while current != first {
                    current = parent[current];
                    if current == usize::MAX {
                        return None;
                    }
                    route.push(grid.point(current));
                }
                *route.last_mut()? = start;
                route.reverse();
                if route.len() == 1 {
                    route.insert(0, start);
                }
                return Some(route);
            }
            closed[node.index] = true;
            let x = node.index % grid.width;
            let y = node.index / grid.width;
            for dy in -1isize..=1 {
                for dx in -1isize..=1 {
                    if dx == 0 && dy == 0 {
                        continue;
                    }
                    let Some(nx) = x.checked_add_signed(dx) else {
                        continue;
                    };
                    let Some(ny) = y.checked_add_signed(dy) else {
                        continue;
                    };
                    if nx >= grid.width || ny >= grid.height {
                        continue;
                    }
                    let next = ny * grid.width + nx;
                    if grid.blocked[next] || closed[next] {
                        continue;
                    }
                    // A diagonal edge cannot cross the corners of occupied cells.
                    if dx != 0
                        && dy != 0
                        && (grid.blocked[y * grid.width + nx] || grid.blocked[ny * grid.width + x])
                    {
                        continue;
                    }
                    let candidate = costs[node.index]
                        + if dx != 0 && dy != 0 {
                            std::f64::consts::SQRT_2 * grid.resolution
                        } else {
                            grid.resolution
                        };
                    if candidate < costs[next] {
                        costs[next] = candidate;
                        parent[next] = node.index;
                        open.push(QueueNode {
                            index: next,
                            score: candidate + grid.point(next).distance(grid.point(last)),
                        });
                    }
                }
            }
        }
        None
    }
}

#[derive(Clone, Copy)]
struct CarNode {
    pose: Pose2,
    curvature: f64,
    parent: Option<usize>,
    cost: f64,
    error: primitive_envelope::ErrorBound,
}

fn car_path_from_nodes(
    nodes: &[CarNode],
    end_index: usize,
    config: &NavigationConfig,
    grid: &Grid,
) -> Option<(Vec<Point2>, primitive_envelope::ErrorBound)> {
    let mut indices = vec![end_index];
    while let Some(parent) = nodes[*indices.last()?].parent {
        indices.push(parent);
    }
    indices.reverse();
    let mut path = vec![nodes.first()?.pose.point()];
    let mut error = nodes.first()?.error;
    for pair in indices.windows(2) {
        let parent = nodes[pair[0]];
        let child = nodes[pair[1]];
        let (samples, _, _, end_error) = car_primitive_with_error(
            parent.pose,
            parent.curvature,
            child.curvature,
            length_for_primitive(grid),
            config,
            grid,
            error,
        )
        .ok()?;
        error = end_error;
        path.extend(samples);
    }
    Some((path, error))
}

fn length_for_primitive(grid: &Grid) -> f64 {
    (2.0 * grid.resolution).clamp(0.15, 0.4)
}

fn car_key(
    pose: Pose2,
    curvature: f64,
    grid: &Grid,
    maximum_curvature: f64,
) -> Option<(usize, u8, u8)> {
    let cell = grid.index(pose.point())?;
    if if grid.recovery_active() {
        !grid.recovery.segment_clear(pose.point(), pose.point(), 0.0)
    } else {
        grid.blocked[cell]
    } {
        return None;
    }
    let heading = ((pose.yaw_rad.rem_euclid(std::f64::consts::TAU) / std::f64::consts::TAU * 36.0)
        .round() as u8)
        % 36;
    let steering = ((curvature / maximum_curvature + 1.0) * 4.0)
        .round()
        .clamp(0.0, 8.0) as u8;
    Some((cell, heading, steering))
}

#[cfg(test)]
fn car_two_arc_connection(
    start: Pose2,
    k: f64,
    goal: Point2,
    heading: f64,
    config: &NavigationConfig,
    grid: &Grid,
) -> Option<(Vec<Point2>, Pose2, f64)> {
    terminal::two_arc(
        start,
        k,
        goal,
        heading,
        config,
        grid,
        &TerminalBudget::default(),
        None,
    )
    .map(|c| c.into_path())
}
#[cfg(test)]
fn car_terminal_connection(
    start: Pose2,
    k: f64,
    goal: Point2,
    heading: Option<f64>,
    config: &NavigationConfig,
    grid: &Grid,
) -> Option<(Vec<Point2>, Pose2, f64)> {
    terminal::single_arc(
        start,
        k,
        goal,
        heading,
        config,
        grid,
        &TerminalBudget::default(),
    )
}

#[cfg(test)]
fn car_primitive(
    start: Pose2,
    initial_curvature: f64,
    target_curvature: f64,
    length: f64,
    config: &NavigationConfig,
    grid: &Grid,
) -> Option<(Vec<Point2>, Pose2, f64)> {
    car_primitive_checked(
        start,
        initial_curvature,
        target_curvature,
        length,
        config,
        grid,
    )
    .ok()
}

#[cfg(test)]
fn car_primitive_checked(
    start: Pose2,
    initial_curvature: f64,
    target_curvature: f64,
    length: f64,
    config: &NavigationConfig,
    grid: &Grid,
) -> Result<(Vec<Point2>, Pose2, f64), recovery::PrimitiveGridFailure> {
    car_primitive_with_error(
        start,
        initial_curvature,
        target_curvature,
        length,
        config,
        grid,
        primitive_envelope::ErrorBound::default(),
    )
    .map(|(points, pose, curvature, _)| (points, pose, curvature))
}

#[allow(clippy::too_many_arguments)]
fn car_primitive_with_error(
    start: Pose2,
    initial_curvature: f64,
    target_curvature: f64,
    length: f64,
    config: &NavigationConfig,
    grid: &Grid,
    mut error: primitive_envelope::ErrorBound,
) -> Result<(Vec<Point2>, Pose2, f64, primitive_envelope::ErrorBound), recovery::PrimitiveGridFailure>
{
    let count = (length / (grid.resolution / 3.0).min(0.025))
        .ceil()
        .max(1.0) as usize;
    let ds = length / count as f64;
    let max_change = config.max_curvature_rate_per_s / config.max_speed_mps * ds;
    let mut pose = start;
    let mut curvature = initial_curvature;
    let mut points = Vec::with_capacity(count);
    for sample_index in 0..count {
        let previous_pose = pose;
        let previous = pose.point();
        let model_failure = || recovery::PrimitiveGridFailure {
            continuous_domain: true,
            sample_index,
            from_cell: grid.index(previous),
            to_cell: None,
            blocked_cell: None,
        };
        let mut model = None;
        if grid.recovery_active() {
            let motion = MotionTransition {
                initial_speed_mps: config.max_speed_mps,
                target_speed_mps: config.max_speed_mps,
                initial_curvature_per_m: curvature,
                target_curvature_per_m: target_curvature,
                max_accel_mps2: config.max_accel_mps2,
                max_decel_mps2: config.max_decel_mps2,
                max_curvature_rate_per_s: config.max_curvature_rate_per_s,
            };
            let duration = ds / config.max_speed_mps;
            let projected =
                primitive_envelope::predict(pose, motion, duration).ok_or_else(model_failure)?;
            model = Some((motion, duration));
            error = error.advance(&projected).ok_or_else(model_failure)?;
            pose = projected.projection.pose;
            curvature = projected.projection.curvature_per_m;
        } else {
            let next = target_curvature.clamp(curvature - max_change, curvature + max_change);
            pose = integrate(pose, ds, (curvature + next) * 0.5);
            curvature = next;
        }
        if !grid.modeled_motion_transition_clear(previous_pose, pose, ds, error, model) {
            let first = grid.index(previous);
            let last = grid.index(pose.point());
            let blocked_cell = (!grid.recovery_active())
                .then(|| {
                    first
                        .filter(|i| grid.blocked[*i])
                        .or_else(|| last.filter(|i| grid.blocked[*i]))
                        .or_else(|| {
                            first.zip(last).and_then(|(first, last)| {
                                let (ax, ay) = (first % grid.width, first / grid.width);
                                let (bx, by) = (last % grid.width, last / grid.width);
                                [ay * grid.width + bx, by * grid.width + ax]
                                    .into_iter()
                                    .find(|i| grid.blocked[*i])
                            })
                        })
                })
                .flatten();
            return Err(recovery::PrimitiveGridFailure {
                continuous_domain: grid.recovery_active(),
                sample_index,
                from_cell: first,
                to_cell: last,
                blocked_cell,
            });
        }
        points.push(pose.point());
    }
    Ok((points, pose, curvature, error))
}

fn body_radius(footprint: Footprint) -> f64 {
    footprint
        .front_m
        .max(footprint.rear_m)
        .hypot(footprint.half_width_m)
}

fn footprint_pose_error(footprint: Footprint, actual: Pose2, reference: Pose2) -> f64 {
    footprint
        .corners(actual)
        .into_iter()
        .zip(footprint.corners(reference))
        .map(|(actual, reference)| actual.distance(reference))
        .fold(0.0, f64::max)
}

fn angle_error(a: f64, b: f64) -> f64 {
    (a - b + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

fn car_heuristic(pose: Pose2, goal: Point2, heading: Option<f64>, max_curvature: f64) -> f64 {
    let distance = pose.point().distance(goal);
    let Some(goal_yaw) = heading else {
        return distance;
    };
    // Geometric circle-straight-circle estimates account for the real forward
    // approach direction. Four tangent combinations are evaluated, not a full
    // Dubins solver: CCC families, obstacles and steering ramps are omitted.
    // This guides a bounded search; only validated primitives become a route.
    let radius = 1.0 / max_curvature;
    let mut best = f64::INFINITY;
    for start_turn in [-1.0, 1.0] {
        for end_turn in [-1.0, 1.0] {
            let start_center = Point2 {
                x_m: pose.x_m - start_turn * radius * pose.yaw_rad.sin(),
                y_m: pose.y_m + start_turn * radius * pose.yaw_rad.cos(),
            };
            let end_center = Point2 {
                x_m: goal.x_m - end_turn * radius * goal_yaw.sin(),
                y_m: goal.y_m + end_turn * radius * goal_yaw.cos(),
            };
            let separation = start_center.distance(end_center);
            let normal_offset = (end_turn - start_turn) * radius;
            if separation < normal_offset.abs() || separation < 1e-12 {
                continue;
            }
            let bearing =
                (end_center.y_m - start_center.y_m).atan2(end_center.x_m - start_center.x_m);
            let tangent_yaw = bearing - (normal_offset / separation).clamp(-1.0, 1.0).asin();
            let start_angle =
                (start_turn * (tangent_yaw - pose.yaw_rad)).rem_euclid(std::f64::consts::TAU);
            let end_angle = (end_turn * (goal_yaw - tangent_yaw)).rem_euclid(std::f64::consts::TAU);
            let straight = (separation.powi(2) - normal_offset.powi(2)).max(0.0).sqrt();
            best = best.min(straight + radius * (start_angle + end_angle));
        }
    }
    if best.is_finite() {
        1.2 * best.max(distance)
    } else {
        distance
    }
}

/// Exact constant-curvature planar integration; zero speed never changes yaw.
pub fn integrate(pose: Pose2, distance_m: f64, curvature_per_m: f64) -> Pose2 {
    let yaw = pose.yaw_rad + distance_m * curvature_per_m;
    if curvature_per_m.abs() < 1e-9 {
        Pose2 {
            x_m: pose.x_m + distance_m * pose.yaw_rad.cos(),
            y_m: pose.y_m + distance_m * pose.yaw_rad.sin(),
            yaw_rad: yaw,
        }
    } else {
        Pose2 {
            x_m: pose.x_m + (yaw.sin() - pose.yaw_rad.sin()) / curvature_per_m,
            y_m: pose.y_m - (yaw.cos() - pose.yaw_rad.cos()) / curvature_per_m,
            yaw_rad: yaw,
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct EvaluatedCandidate {
    next_error: primitive_envelope::ErrorBound,
    score: f64,
    intent: MotionIntent,
    prediction: RolloutPrediction,
    reference: crate::reference::ReferenceProjection,
    heading_error: f64,
    progress_m: f64,
    transition: MotionTransition,
    next_projection: Option<crate::motion_transition::MotionProjection>,
    terminal_seed: Option<CachedTerminalSeed>,
    terminal_deferred: bool,
}

#[derive(Clone, Copy, Debug)]
struct RolloutPrediction {
    endpoint: Pose2,
    distance_m: f64,
    reference_cost_m: Option<f64>,
    stopping_envelope_margin: Option<StoppingMargin>,
}

struct RolloutReference<'a> {
    cursor: ReferenceCursor<'a>,
    initial_arc_m: f64,
    goal_heading_rad: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum RolloutRejection {
    StopReachable,
    LateralAcceleration,
    Grid,
    Footprint,
    SampleBudget,
    Reference,
}

/// Predict normal candidate motion separately from the emergency stopping
/// envelope. Slower targets change the former without shrinking the latter
/// below the measured-speed reaction and braking requirement.
#[allow(clippy::too_many_arguments)]
#[cfg(test)]
fn rollout(
    config: &NavigationConfig,
    pose: Pose2,
    current_speed: f64,
    target_speed: f64,
    initial_curvature: f64,
    target_curvature: f64,
    obstacles: &[ObstacleDisc],
    goal_distance: f64,
    grid: &Grid,
    reference: Option<RolloutReference<'_>>,
) -> Result<RolloutPrediction, RolloutRejection> {
    rollout_with_stopping_policy(
        config,
        pose,
        current_speed,
        target_speed,
        initial_curvature,
        target_curvature,
        obstacles,
        goal_distance,
        grid,
        reference,
        TargetPolicy::Fixed,
        0.0,
    )
}

#[allow(clippy::too_many_arguments)]
fn rollout_with_stopping_policy(
    config: &NavigationConfig,
    pose: Pose2,
    current_speed: f64,
    target_speed: f64,
    initial_curvature: f64,
    target_curvature: f64,
    obstacles: &[ObstacleDisc],
    goal_distance: f64,
    grid: &Grid,
    mut reference: Option<RolloutReference<'_>>,
    target_policy: TargetPolicy,
    historical_speed_bound_mps: f64,
) -> Result<RolloutPrediction, RolloutRejection> {
    let reaction_s = config.control_period_ms as f64 / 1000.0;
    // Cover the whole adopted control interval even when geometry preview ends
    // early near a target. Also cover the held-command preview if it is longer.
    let peak = lateral_acceleration_peak(
        MotionTransition {
            initial_speed_mps: current_speed,
            target_speed_mps: target_speed,
            initial_curvature_per_m: initial_curvature,
            target_curvature_per_m: target_curvature,
            max_accel_mps2: config.max_accel_mps2,
            max_decel_mps2: config.max_decel_mps2,
            max_curvature_rate_per_s: config.max_curvature_rate_per_s,
        },
        reaction_s.max(config.preview_horizon_s),
    )
    .ok_or(RolloutRejection::LateralAcceleration)?;
    if peak.lateral_accel_mps2 > config.max_lateral_accel_mps2 + 1e-9 {
        return Err(RolloutRejection::LateralAcceleration);
    }
    let envelope_speed = current_speed.max(target_speed).max(0.0);
    let braking_distance = envelope_speed * reaction_s
        + envelope_speed * envelope_speed / (2.0 * config.max_decel_mps2);
    let step = config.grid_resolution_m.min(config.clearance_m).min(0.05) / 2.0;
    // Stop targets zero speed and zero curvature; steering may return gradually.
    // Two boundary trajectories (fixed steering and instant center) do not cover
    // all intermediate paths. A center can travel at most this path length,
    // regardless of its intermediate steering. Expanding that reachable disk by
    // the complete body radius bounds every intermediate footprint, including
    // one accepted tick before the reaction/deceleration interval. This assumes
    // actual braking >= configured deceleration, requiring vehicle calibration.
    let stopping_envelope_margin = if !stop_reachable_clear(
        config,
        pose,
        braking_distance + envelope_speed * reaction_s,
        obstacles,
    ) || grid.travel_boundary.is_some_and(|boundary| {
        !boundary.contains_disc(
            pose.point(),
            body_radius(config.footprint)
                + config.clearance_m
                + braking_distance
                + envelope_speed * reaction_s,
        )
    }) {
        if target_policy != TargetPolicy::RollingLocal
            || !historical_speed_bound_mps.is_finite()
            || historical_speed_bound_mps < 0.0
        {
            return Err(RolloutRejection::StopReachable);
        }
        Some(
            stopping_envelope::certify(
                config,
                pose,
                envelope_speed.max(historical_speed_bound_mps),
                obstacles,
                grid.travel_boundary,
            )
            .ok_or(RolloutRejection::StopReachable)?,
        )
    } else {
        None
    };
    // At most one control period and `step` meters per integration step. The
    // total loop has a separate hard cap even for extreme valid configurations.
    let dt_limit = reaction_s.min(step / envelope_speed.max(1e-6));
    let mut sample = pose;
    let mut speed = current_speed.max(0.0);
    let mut curvature = initial_curvature;
    let mut elapsed = 0.0;
    let mut distance_m = 0.0;
    let mut reference_integral = 0.0;
    let mut motion_error = primitive_envelope::ErrorBound::default();
    if !grid.transition_clear(sample.point(), sample.point()) {
        return Err(RolloutRejection::Grid);
    }
    if !pose_clear(config, sample, obstacles, config.clearance_m)
        || !grid.footprint_inside_boundary(config.footprint, sample, config.clearance_m)
    {
        return Err(RolloutRejection::Footprint);
    }
    for _ in 0..1024 {
        if elapsed >= config.preview_horizon_s - 1e-12 || distance_m >= goal_distance - 1e-12 {
            return Ok(RolloutPrediction {
                endpoint: sample,
                distance_m,
                stopping_envelope_margin,
                reference_cost_m: reference
                    .as_ref()
                    .map(|_| reference_integral / distance_m.max(1e-9)),
            });
        }
        let dt = dt_limit.min(config.preview_horizon_s - elapsed);
        let acceleration = if target_speed >= speed {
            config.max_accel_mps2
        } else {
            config.max_decel_mps2
        };
        let (next_speed, travel) = ramp_integral(speed, target_speed, acceleration, dt);
        let (next_curvature, curvature_integral) = ramp_integral(
            curvature,
            target_curvature,
            config.max_curvature_rate_per_s,
            dt,
        );
        let ds = travel.min((goal_distance - distance_m).max(0.0));
        let mut model = None;
        let (next, next_speed, next_curvature, ds) = if grid.recovery_active() {
            // In recovery, the capsule needs a bound around the actual ramp,
            // including a speed/steering target reached inside this time step.
            // Solve the monotone speed-distance relation for a shortened tail;
            // do not advance steering for time that the geometry did not travel.
            let duration = if ds < travel {
                let ramp_time = (target_speed - speed).abs() / acceleration;
                let ramp_distance = (speed + target_speed) * ramp_time * 0.5;
                if ds > ramp_distance {
                    ramp_time + (ds - ramp_distance) / target_speed
                } else {
                    let a = if target_speed >= speed {
                        acceleration
                    } else {
                        -acceleration
                    };
                    let end_speed = (speed * speed + 2.0 * a * ds).max(0.0).sqrt();
                    2.0 * ds / (speed + end_speed)
                }
            } else {
                dt
            };
            if !duration.is_finite() || duration < 0.0 {
                return Err(RolloutRejection::SampleBudget);
            }
            let motion = MotionTransition {
                initial_speed_mps: speed,
                target_speed_mps: target_speed,
                initial_curvature_per_m: curvature,
                target_curvature_per_m: target_curvature,
                max_accel_mps2: config.max_accel_mps2,
                max_decel_mps2: config.max_decel_mps2,
                max_curvature_rate_per_s: config.max_curvature_rate_per_s,
            };
            model = Some((motion, duration.min(dt)));
            let bounded = primitive_envelope::predict(sample, motion, duration.min(dt))
                .ok_or(RolloutRejection::SampleBudget)?;
            motion_error = motion_error
                .advance(&bounded)
                .ok_or(RolloutRejection::SampleBudget)?;
            (
                bounded.projection.pose,
                bounded.projection.speed_mps,
                bounded.projection.curvature_per_m,
                bounded.projection.distance_m,
            )
        } else {
            // Preserve the ordinary planner's existing numerical reference.
            (
                integrate(sample, ds, curvature_integral / dt),
                next_speed,
                next_curvature,
                ds,
            )
        };
        let max_curvature = curvature.abs().max(next_curvature.abs());
        // Bound a full step's body-point travel from its starting footprint,
        // preserving a conservative swept margin while curvature changes.
        let swept_padding = config.clearance_m
            + ds * (1.0 + max_curvature * body_radius(config.footprint))
            + motion_error.position_m
            + body_radius(config.footprint) * motion_error.heading_rad.min(2.0);
        // Keep local control inside the same conservative domain as A*. Without
        // this, a safe rectangle could enter a cell whose inflated start is
        // blocked on the next tick, stranding an otherwise clear vehicle.
        if !grid.modeled_motion_transition_clear(sample, next, ds, motion_error, model) {
            return Err(RolloutRejection::Grid);
        }
        if !pose_clear(config, sample, obstacles, swept_padding)
            || !pose_clear(config, next, obstacles, swept_padding)
            || !grid.footprint_inside_boundary(config.footprint, sample, swept_padding)
            || !grid.footprint_inside_boundary(config.footprint, next, swept_padding)
        {
            return Err(RolloutRejection::Footprint);
        }
        if let Some(reference) = &mut reference {
            let location = reference
                .cursor
                .at_arc(reference.initial_arc_m + distance_m + ds)
                .ok_or(RolloutRejection::Reference)?;
            let heading = if distance_m + ds >= goal_distance - config.goal_tolerance_m {
                reference.goal_heading_rad
            } else {
                location.heading_rad
            };
            // Compare corresponding body corners in meters. Converting yaw
            // error using the minimum turning radius can reward centering yaw
            // too early while lateral error still needs that heading to recover.
            // This pose metric uses actual body geometry, without a tuned
            // relative gain between meters and radians.
            let reference_pose = Pose2 {
                x_m: location.point.x_m,
                y_m: location.point.y_m,
                yaw_rad: heading,
            };
            let footprint_error = [-config.footprint.rear_m, config.footprint.front_m]
                .into_iter()
                .flat_map(|x_m| {
                    [
                        -config.footprint.half_width_m,
                        config.footprint.half_width_m,
                    ]
                    .map(|y_m| Point2 { x_m, y_m })
                })
                .map(|corner| {
                    next.body_to_world(corner)
                        .distance(reference_pose.body_to_world(corner))
                })
                .fold(0.0_f64, f64::max);
            reference_integral += ds * footprint_error;
            if !reference_integral.is_finite() {
                return Err(RolloutRejection::Reference);
            }
        }
        sample = next;
        speed = next_speed;
        curvature = next_curvature;
        elapsed += dt;
        distance_m += ds;
    }
    if elapsed >= config.preview_horizon_s - 1e-12 || distance_m >= goal_distance - 1e-12 {
        Ok(RolloutPrediction {
            endpoint: sample,
            distance_m,
            stopping_envelope_margin,
            reference_cost_m: reference
                .as_ref()
                .map(|_| reference_integral / distance_m.max(1e-9)),
        })
    } else {
        Err(RolloutRejection::SampleBudget)
    }
}

/// End value and exact integral of a rate-limited linear transition, including
/// the constant remainder when the target is reached partway through a step.
fn ramp_integral(current: f64, target: f64, rate: f64, dt: f64) -> (f64, f64) {
    let ramp_time = ((target - current).abs() / rate).min(dt);
    let next = current + (target - current).clamp(-rate * dt, rate * dt);
    let integral = (current + next) * 0.5 * ramp_time + target * (dt - ramp_time);
    (next, integral)
}

#[cfg(test)]
mod velocity_rollout_tests {
    use super::*;

    fn scene() -> (NavigationConfig, Pose2) {
        let mut config = NavigationConfig::simulation(
            Rect {
                min_x_m: 0.0,
                min_y_m: 0.0,
                max_x_m: 20.0,
                max_y_m: 20.0,
            },
            Footprint {
                front_m: 0.22,
                rear_m: 0.18,
                half_width_m: 0.13,
            },
            FrameId("map".into()),
        );
        config.control_period_ms = 100;
        let pose = Pose2 {
            x_m: 5.0,
            y_m: 5.0,
            yaw_rad: 0.0,
        };
        (config, pose)
    }

    #[test]
    fn future_stopping_diagnostic_exposes_acceleration_that_loses_room() {
        let (mut config, pose) = scene();
        config.grid_resolution_m = 0.05;
        config.bounds.max_x_m = 10.0;
        config.bounds.max_y_m = 10.0;
        let obstacles = [ObstacleDisc {
            center: Point2 { x_m: 5.2, y_m: 5.4 },
            radius_m: 0.015,
        }];
        let at = Timestamp(0);
        let estimate = PoseEstimate {
            captured_at: at,
            frame_id: config.frame_id.clone(),
            pose,
            speed_mps: 0.28,
            yaw_rate_radps: 0.0,
            quality: 1.0,
        };
        let mut nav = Navigator::new(config.clone()).unwrap();
        let decision = nav
            .step(at, &estimate, &obstacles, at, Point2 { x_m: 7.0, y_m: 5.0 })
            .unwrap();
        assert_eq!(decision.status, NavigationStatus::Driving, "{decision:?}");
        assert!(
            decision
                .diagnostics
                .selected_stopping_margin
                .unwrap()
                .clear()
        );
        assert!(decision.diagnostics.selected_next_stopping_margin.is_some());
        // The faster command passes the current complete disk, but has no
        // stopping room one full interval later. Do not fix it by shrinking
        // the current disk to the lower target speed.
        assert!(stopping_margin(&config, pose, 0.3, &obstacles, None).clear());
        let faster = project_motion(
            pose,
            MotionTransition {
                initial_speed_mps: 0.28,
                target_speed_mps: 0.3,
                initial_curvature_per_m: 0.0,
                target_curvature_per_m: 0.0,
                max_accel_mps2: 0.4,
                max_decel_mps2: 0.6,
                max_curvature_rate_per_s: 4.0,
            },
            0.1,
        )
        .unwrap();
        assert!(!stopping_margin(&config, faster.pose, faster.speed_mps, &obstacles, None).clear());
    }

    #[test]
    fn stopping_margin_reports_obstacle_contact_and_directional_boundaries() {
        let (config, pose) = scene();
        let radius = body_radius(config.footprint) + config.clearance_m + 0.135;
        let obstacle = ObstacleDisc {
            center: Point2 {
                x_m: pose.x_m + radius + 0.015,
                y_m: pose.y_m,
            },
            radius_m: 0.015,
        };
        let margin = disc_margin(&config, pose, radius, &[obstacle], None);
        assert_eq!(margin.constraint, StoppingConstraint::Obstacle { index: 0 });
        assert_eq!(
            margin.clear(),
            stop_reachable_clear(&config, pose, 0.135, &[obstacle])
        );
        let edge = Pose2 { x_m: 0.2, ..pose };
        let margin = disc_margin(&config, edge, radius, &[], None);
        assert_eq!(margin.constraint, StoppingConstraint::MinX);
        assert!(!margin.clear());
    }

    #[test]
    fn full_reference_cost_exposes_mid_route_error_despite_a_matching_endpoint() {
        let (config, pose) = scene();
        let grid = Grid::new(&config, &[]);
        let goal = Point2 { x_m: 5.6, y_m: 5.0 };
        let straight = [pose.point(), goal];
        let bowed = [pose.point(), Point2 { x_m: 5.3, y_m: 5.2 }, goal];
        let predict = |route: &[Point2]| {
            // Deliberately short projection window: arc sampling must still
            // follow the real route, not treat the window edge as its endpoint.
            let reference = PreparedReference::new(route, 0, 0.1).unwrap();
            rollout(
                &config,
                pose,
                0.3,
                0.3,
                0.0,
                0.0,
                &[],
                0.6,
                &grid,
                Some(RolloutReference {
                    cursor: reference.cursor_to_end(),
                    initial_arc_m: 0.0,
                    goal_heading_rad: 0.0,
                }),
            )
            .unwrap()
        };
        let aligned = predict(&straight);
        let missed_bend = predict(&bowed);
        assert_eq!(aligned.endpoint, missed_bend.endpoint);
        assert!(aligned.endpoint.point().distance(goal) < 1e-10);
        assert!(aligned.reference_cost_m.unwrap() < 1e-10);
        assert!(missed_bend.reference_cost_m.unwrap() > 0.1);
    }

    #[test]
    fn slowing_candidate_predicts_its_own_distance_and_curved_endpoint() {
        let (config, pose) = scene();
        let grid = Grid::new(&config, &[]);
        let fast = rollout(&config, pose, 0.3, 0.3, 0.5, 0.5, &[], 10.0, &grid, None).unwrap();
        let slow = rollout(&config, pose, 0.3, 0.24, 0.5, 0.5, &[], 10.0, &grid, None).unwrap();
        // Slow target is reachable after one 100 ms normal deceleration tick.
        // Old max(current, target) prediction gave both candidates 0.6 meters.
        assert!((fast.distance_m - 0.6).abs() < 1e-10);
        assert!((slow.distance_m - 0.483).abs() < 1e-10);
        assert!(fast.endpoint.point().distance(slow.endpoint.point()) > 0.11);
        assert!(slow.endpoint.yaw_rad < fast.endpoint.yaw_rad);
        assert!(
            slow.endpoint
                .point()
                .distance(integrate(pose, 0.483, 0.5).point())
                < 1e-10
        );
    }

    #[test]
    fn acceleration_starts_at_current_speed_instead_of_commanded_speed() {
        let (mut config, pose) = scene();
        config.preview_horizon_s = 0.5;
        let grid = Grid::new(&config, &[]);
        let predicted = rollout(&config, pose, 0.0, 0.3, 0.0, 0.0, &[], 10.0, &grid, None).unwrap();
        // 0.4 m/s² from rest over 0.5 s: 0.05 m, not instantaneous 0.3 * 0.5.
        assert!((predicted.distance_m - 0.05).abs() < 1e-10);
        assert!((predicted.endpoint.x_m - pose.x_m - 0.05).abs() < 1e-10);
    }

    #[test]
    fn curvature_starts_at_previous_value_and_respects_slew() {
        let (mut config, pose) = scene();
        config.preview_horizon_s = 0.1;
        config.max_curvature_rate_per_s = 0.4;
        let grid = Grid::new(&config, &[]);
        let predicted = rollout(&config, pose, 0.3, 0.3, 0.0, 2.0, &[], 10.0, &grid, None).unwrap();
        // Constant speed makes the yaw integral exact: 0.3 * 0.4 * t² / 2.
        assert!((predicted.endpoint.yaw_rad - 0.0006).abs() < 1e-10);
        assert!(predicted.endpoint.yaw_rad < integrate(pose, 0.03, 2.0).yaw_rad / 10.0);
    }

    #[test]
    fn slower_target_and_near_goal_do_not_reduce_measured_speed_stop_envelope() {
        let (config, pose) = scene();
        let obstacle = ObstacleDisc {
            center: Point2 {
                x_m: pose.x_m - body_radius(config.footprint) - config.clearance_m - 0.12,
                y_m: pose.y_m,
            },
            radius_m: 0.005,
        };
        let obstacles = [obstacle];
        let grid = Grid::new(&config, &obstacles);
        // A stop envelope incorrectly based on the lower 0.24 m/s target would
        // accept this obstacle behind the vehicle. Measured 0.3 m/s must reject.
        let lower_speed_distance = 2.0 * 0.24 * 0.1 + 0.24_f64.powi(2) / (2.0 * 0.6);
        assert!(stop_reachable_clear(
            &config,
            pose,
            lower_speed_distance,
            &obstacles
        ));
        for target_speed in [0.3, 0.24, 0.01] {
            assert_eq!(
                rollout(
                    &config,
                    pose,
                    0.3,
                    target_speed,
                    0.5,
                    0.5,
                    &obstacles,
                    0.001,
                    &grid,
                    None,
                )
                .unwrap_err(),
                RolloutRejection::StopReachable
            );
        }
    }

    #[test]
    fn excessive_prediction_returns_bounded_sample_budget_rejection() {
        let (mut config, pose) = scene();
        config.clearance_m = 0.005;
        config.max_speed_mps = 3.0;
        config.max_decel_mps2 = 10.0;
        config.preview_horizon_s = 5.0;
        config.validate().unwrap();
        let grid = Grid::new(&config, &[]);
        assert_eq!(
            rollout(&config, pose, 3.0, 3.0, 0.0, 0.0, &[], 10.0, &grid, None).unwrap_err(),
            RolloutRejection::SampleBudget
        );
    }
}

fn stop_reachable_clear(
    config: &NavigationConfig,
    pose: Pose2,
    distance: f64,
    obstacles: &[ObstacleDisc],
) -> bool {
    let radius = body_radius(config.footprint) + config.clearance_m + distance;
    pose.x_m - radius >= config.bounds.min_x_m
        && pose.x_m + radius <= config.bounds.max_x_m
        && pose.y_m - radius >= config.bounds.min_y_m
        && pose.y_m + radius <= config.bounds.max_y_m
        && obstacles
            .iter()
            .all(|obstacle| pose.point().distance(obstacle.center) > radius + obstacle.radius_m)
}

fn stopping_margin(
    config: &NavigationConfig,
    pose: Pose2,
    speed: f64,
    obstacles: &[ObstacleDisc],
    boundary: Option<HalfPlane>,
) -> StoppingMargin {
    let period = config.control_period_ms as f64 / 1000.0;
    let braking_distance = speed * period + speed.powi(2) / (2.0 * config.max_decel_mps2);
    disc_margin(
        config,
        pose,
        body_radius(config.footprint) + config.clearance_m + (braking_distance + speed * period),
        obstacles,
        boundary,
    )
}

/// Same full stopping disk as the hard gate, retaining the numerical margin
/// and the constraining object. Boundary contact is allowed; obstacle contact
/// is rejected, as before. A zero-margin obstacle wins a tie with a boundary.
fn disc_margin(
    config: &NavigationConfig,
    pose: Pose2,
    radius: f64,
    obstacles: &[ObstacleDisc],
    boundary: Option<HalfPlane>,
) -> StoppingMargin {
    let mut result = StoppingMargin {
        clearance_m: pose.x_m - radius - config.bounds.min_x_m,
        constraint: StoppingConstraint::MinX,
    };
    for (clearance_m, constraint) in [
        (
            config.bounds.max_x_m - (pose.x_m + radius),
            StoppingConstraint::MaxX,
        ),
        (
            pose.y_m - radius - config.bounds.min_y_m,
            StoppingConstraint::MinY,
        ),
        (
            config.bounds.max_y_m - (pose.y_m + radius),
            StoppingConstraint::MaxY,
        ),
    ] {
        if clearance_m < result.clearance_m {
            result = StoppingMargin {
                clearance_m,
                constraint,
            };
        }
    }
    if let Some(boundary) = boundary {
        let clearance_m = boundary.signed_disc_margin(pose.point(), radius);
        if clearance_m < result.clearance_m {
            result = StoppingMargin {
                clearance_m,
                constraint: StoppingConstraint::TravelBoundary,
            };
        }
    }
    for (index, obstacle) in obstacles.iter().enumerate() {
        let clearance_m = pose.point().distance(obstacle.center) - (radius + obstacle.radius_m);
        if clearance_m <= result.clearance_m {
            result = StoppingMargin {
                clearance_m,
                constraint: StoppingConstraint::Obstacle { index },
            };
        }
    }
    result
}

fn pose_clear(
    config: &NavigationConfig,
    pose: Pose2,
    obstacles: &[ObstacleDisc],
    margin: f64,
) -> bool {
    let interior = Rect {
        min_x_m: config.bounds.min_x_m + margin,
        min_y_m: config.bounds.min_y_m + margin,
        max_x_m: config.bounds.max_x_m - margin,
        max_y_m: config.bounds.max_y_m - margin,
    };
    if !config.footprint.inside(pose, interior) {
        return false;
    }
    obstacles.iter().all(|obstacle| {
        let local = pose.world_to_body(obstacle.center);
        let x = local
            .x_m
            .clamp(-config.footprint.rear_m, config.footprint.front_m);
        let y = local.y_m.clamp(
            -config.footprint.half_width_m,
            config.footprint.half_width_m,
        );
        (local.x_m - x).hypot(local.y_m - y) > obstacle.radius_m + margin
    })
}

struct Grid {
    recovery: recovery::RecoveryGrid,
    travel_boundary: Option<HalfPlane>,
    width: usize,
    height: usize,
    resolution: f64,
    bounds: Rect,
    blocked: Vec<bool>,
}
impl Grid {
    fn footprint_inside_boundary(&self, footprint: Footprint, pose: Pose2, margin: f64) -> bool {
        self.travel_boundary
            .is_none_or(|boundary| boundary.contains_footprint(footprint, pose, margin))
    }
    /// Samples are spaced at most one cell apart. For a diagonal transition,
    /// require both orthogonal neighbors so neither a coarse motion primitive
    /// nor a fine rollout can jump across an occupied corner.
    fn transition_clear(&self, from: Point2, to: Point2) -> bool {
        if self.recovery_active() {
            // A straight reference segment check only. Curved primitives and
            // actual rollout segments pass arc length to motion_transition_clear.
            return self.recovery.segment_clear(from, to, 0.0);
        }
        let (Some(first), Some(last)) = (self.index(from), self.index(to)) else {
            return false;
        };
        if self.blocked[first] || self.blocked[last] {
            return false;
        }
        let (ax, ay) = (first % self.width, first / self.width);
        let (bx, by) = (last % self.width, last / self.width);
        if ax.abs_diff(bx) > 1 || ay.abs_diff(by) > 1 {
            return false;
        }
        ax == bx
            || ay == by
            || (!self.blocked[ay * self.width + bx] && !self.blocked[by * self.width + ax])
    }

    #[cfg(test)]
    fn new(config: &NavigationConfig, obstacles: &[ObstacleDisc]) -> Self {
        Self::with_boundary(config, obstacles, None)
    }

    fn with_boundary(
        config: &NavigationConfig,
        obstacles: &[ObstacleDisc],
        travel_boundary: Option<HalfPlane>,
    ) -> Self {
        let width = ((config.bounds.max_x_m - config.bounds.min_x_m) / config.grid_resolution_m)
            .ceil() as usize;
        let height = ((config.bounds.max_y_m - config.bounds.min_y_m) / config.grid_resolution_m)
            .ceil() as usize;
        let mut grid = Self {
            recovery: recovery::RecoveryGrid::new(config, obstacles, travel_boundary),
            travel_boundary,
            width,
            height,
            resolution: config.grid_resolution_m,
            bounds: config.bounds,
            blocked: vec![false; width * height],
        };
        // Include the half diagonal so a free cell's entire area is conservative.
        let inflation = body_radius(config.footprint)
            + config.clearance_m
            + config.grid_resolution_m * std::f64::consts::FRAC_1_SQRT_2;
        for index in 0..grid.blocked.len() {
            let p = grid.point(index);
            grid.blocked[index] = p.x_m - inflation < config.bounds.min_x_m
                || p.y_m - inflation < config.bounds.min_y_m
                || p.x_m + inflation > config.bounds.max_x_m
                || p.y_m + inflation > config.bounds.max_y_m
                || travel_boundary.is_some_and(|boundary| !boundary.contains_disc(p, inflation));
        }
        for obstacle in obstacles {
            let radius = obstacle.radius_m + inflation;
            let minimum_x = (((obstacle.center.x_m - radius - grid.bounds.min_x_m)
                / grid.resolution)
                .floor()
                .max(0.0) as usize)
                .min(width);
            let maximum_x = (((obstacle.center.x_m + radius - grid.bounds.min_x_m)
                / grid.resolution)
                .ceil()
                .max(0.0) as usize)
                .min(width);
            let minimum_y = (((obstacle.center.y_m - radius - grid.bounds.min_y_m)
                / grid.resolution)
                .floor()
                .max(0.0) as usize)
                .min(height);
            let maximum_y = (((obstacle.center.y_m + radius - grid.bounds.min_y_m)
                / grid.resolution)
                .ceil()
                .max(0.0) as usize)
                .min(height);
            for y in minimum_y..maximum_y {
                for x in minimum_x..maximum_x {
                    let index = y * width + x;
                    if grid.point(index).distance(obstacle.center) <= radius {
                        grid.blocked[index] = true;
                    }
                }
            }
        }
        grid
    }
    fn index(&self, point: Point2) -> Option<usize> {
        if !self.bounds.contains(point) {
            return None;
        }
        let x = ((point.x_m - self.bounds.min_x_m) / self.resolution).floor() as usize;
        let y = ((point.y_m - self.bounds.min_y_m) / self.resolution).floor() as usize;
        (x < self.width && y < self.height).then_some(y * self.width + x)
    }
    fn point(&self, index: usize) -> Point2 {
        Point2 {
            x_m: self.bounds.min_x_m
                + (index % self.width) as f64 * self.resolution
                + self.resolution / 2.0,
            y_m: self.bounds.min_y_m
                + (index / self.width) as f64 * self.resolution
                + self.resolution / 2.0,
        }
    }
}

#[cfg(test)]
mod travel_boundary_tests {
    use super::*;

    #[test]
    fn finite_boundary_stopping_margin_matches_the_disc_gate() {
        let (config, _, global) = scene(0.0);
        let boundary = global
            .with_lateral_region(Rect {
                min_x_m: 4.5,
                max_x_m: 5.5,
                min_y_m: 4.6,
                max_y_m: 5.4,
            })
            .unwrap();
        for y_m in [0.0, -0.8, 0.8] {
            let pose = Pose2 {
                x_m: 5.5,
                y_m: 5.0 + y_m,
                yaw_rad: 0.0,
            };
            let margin = disc_margin(&config, pose, 0.1, &[], Some(boundary));
            assert_eq!(
                margin.clearance_m >= 0.0,
                boundary.contains_disc(pose.point(), 0.1)
            );
        }
    }

    fn scene(yaw: f64) -> (NavigationConfig, Pose2, HalfPlane) {
        let mut config = NavigationConfig::simulation(
            Rect {
                min_x_m: 0.0,
                min_y_m: 0.0,
                max_x_m: 20.0,
                max_y_m: 20.0,
            },
            Footprint {
                front_m: 0.22,
                rear_m: 0.18,
                half_width_m: 0.13,
            },
            FrameId("map".into()),
        );
        config.control_period_ms = 100;
        config.max_curvature_per_m = 3.0;
        config.preview_horizon_s = 5.0;
        let origin = Pose2 {
            x_m: 5.0,
            y_m: 5.0,
            yaw_rad: yaw,
        };
        let start = origin.body_to_world(Point2 {
            x_m: -0.6,
            y_m: 0.0,
        });
        let pose = Pose2 {
            x_m: start.x_m,
            y_m: start.y_m,
            yaw_rad: yaw,
        };
        (
            config,
            pose,
            HalfPlane::new(origin.point(), yaw, 0.0).unwrap(),
        )
    }

    #[test]
    fn rotated_boundary_rejects_a_curve_that_crosses_then_returns_to_the_allowed_side() {
        for yaw in [
            0.0,
            std::f64::consts::FRAC_PI_2,
            std::f64::consts::FRAC_PI_4,
            -2.0,
        ] {
            let (config, start, boundary) = scene(yaw);
            let open = Grid::new(&config, &[]);
            let constrained = Grid::with_boundary(&config, &[], Some(boundary));
            let length = std::f64::consts::PI / 2.5;
            let (_, end, _) = car_primitive(start, 2.5, 2.5, length, &config, &open).unwrap();
            assert!(boundary.contains_footprint(config.footprint, start, config.clearance_m));
            assert!(boundary.contains_footprint(config.footprint, end, config.clearance_m));
            // Both endpoints fit, but the middle of this semicircle crosses the
            // line. The same primitive validates lattice and terminal edges.
            assert!(car_primitive(start, 2.5, 2.5, length, &config, &constrained).is_none());
            assert!(rollout(&config, start, 0.3, 0.3, 2.5, 2.5, &[], length, &open, None).is_ok());
            assert!(
                rollout(
                    &config,
                    start,
                    0.3,
                    0.3,
                    2.5,
                    2.5,
                    &[],
                    length,
                    &constrained,
                    None
                )
                .is_err()
            );
        }
    }

    #[test]
    fn lower_target_cannot_shrink_the_measured_speed_stop_circle_at_a_boundary() {
        let (config, mut pose, _) = scene(0.0);
        pose.x_m = 5.0;
        let radius = body_radius(config.footprint) + config.clearance_m;
        let boundary = HalfPlane::new(pose.point(), 0.0, radius + 0.12).unwrap();
        let grid = Grid::with_boundary(&config, &[], Some(boundary));
        assert!(boundary.contains_disc(
            pose.point(),
            radius + 2.0 * 0.24 * 0.1 + 0.24_f64.powi(2) / 1.2
        ));
        for target in [0.3, 0.24, 0.01] {
            assert_eq!(
                rollout(
                    &config,
                    pose,
                    0.3,
                    target,
                    0.0,
                    0.0,
                    &[],
                    0.001,
                    &grid,
                    None
                )
                .unwrap_err(),
                RolloutRejection::StopReachable
            );
        }
    }

    #[test]
    fn a_violating_current_footprint_cannot_be_reported_as_reached() {
        let (config, pose, _) = scene(std::f64::consts::FRAC_PI_4);
        let mut navigator = Navigator::new(config.clone()).unwrap();
        navigator.set_travel_boundary(Some(
            HalfPlane::new(pose.point(), pose.yaw_rad, 0.1).unwrap(),
        ));
        let estimate = PoseEstimate {
            captured_at: Timestamp(0),
            frame_id: config.frame_id,
            pose,
            speed_mps: 0.0,
            yaw_rate_radps: 0.0,
            quality: 1.0,
        };
        let decision = navigator
            .step(Timestamp(0), &estimate, &[], Timestamp(0), pose.point())
            .unwrap();
        assert_eq!(decision.status, NavigationStatus::Blocked);
        assert_eq!(
            decision.reason.as_deref(),
            Some("current_footprint_collision_or_boundary")
        );
    }

    #[test]
    fn near_stationary_arrival_cannot_bypass_the_boundary_stopping_requirement() {
        let (mut config, pose, boundary) = scene(0.0);
        config.max_decel_mps2 = 0.0001;
        let estimate = PoseEstimate {
            captured_at: Timestamp(0),
            frame_id: config.frame_id.clone(),
            pose,
            speed_mps: 0.01,
            yaw_rate_radps: 0.0,
            quality: 1.0,
        };
        let mut navigator = Navigator::new(config).unwrap();
        navigator.set_travel_boundary(Some(boundary));
        let decision = navigator
            .step(Timestamp(0), &estimate, &[], Timestamp(0), pose.point())
            .unwrap();
        assert_eq!(decision.status, NavigationStatus::Blocked);
        assert_eq!(
            decision.reason.as_deref(),
            Some("stop_boundary_not_reachable")
        );
        assert_eq!(decision.intent.speed_mps, 0.0);
    }

    #[test]
    fn changed_boundary_invalidates_cached_routes_without_resetting_steering() {
        let (config, pose, boundary) = scene(0.0);
        let goal = Point2 { x_m: 4.5, y_m: 5.0 };
        let mut navigator = Navigator::new(config).unwrap();
        navigator.steering.commanded_curvature_per_m = 0.4;
        navigator.route = Some((goal, None, vec![pose.point(), goal], 0));
        navigator.set_travel_boundary(Some(boundary));
        assert!(navigator.route.is_none());
        assert_eq!(navigator.steering.commanded_curvature_per_m, 0.4);
        navigator.route = Some((goal, None, vec![pose.point(), goal], 0));
        navigator.set_travel_boundary(Some(boundary));
        assert!(navigator.route.is_some());
        navigator.set_travel_boundary(None);
        assert!(navigator.route.is_none());
        assert_eq!(navigator.steering.commanded_curvature_per_m, 0.4);
    }
}

#[derive(Clone, Copy)]
struct QueueNode {
    index: usize,
    score: f64,
}
impl PartialEq for QueueNode {
    fn eq(&self, other: &Self) -> bool {
        self.index == other.index && self.score == other.score
    }
}
impl Eq for QueueNode {}
impl PartialOrd for QueueNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for QueueNode {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .score
            .total_cmp(&self.score)
            .then_with(|| other.index.cmp(&self.index))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn small_sideways_heading_goal_gets_a_short_safe_s_instead_of_a_loop() {
        let mut config = NavigationConfig::simulation(
            Rect {
                min_x_m: 0.0,
                min_y_m: 0.0,
                max_x_m: 7.0,
                max_y_m: 5.0,
            },
            Footprint {
                front_m: 0.22,
                rear_m: 0.18,
                half_width_m: 0.13,
            },
            FrameId("map".into()),
        );
        config.control_period_ms = 100;
        config.goal_tolerance_m = 0.045;
        // Recorded arrival at the preceding heading-constrained waypoint.
        // The five-bin lattice previously returned a 4.228 m complete loop.
        let start = Pose2 {
            x_m: 5.538209091104441,
            y_m: 2.534661640176846,
            yaw_rad: 0.009926220914205695,
        };
        let goal = Point2 {
            x_m: 6.43,
            y_m: 2.5,
        };
        let grid = Grid::new(&config, &[]);
        let navigator = Navigator::new(config.clone()).unwrap();
        let points = navigator
            .kinematic_path(start, goal, Some(0.0), &grid)
            .unwrap();
        assert_eq!(points.first(), Some(&start.point()));
        assert!(points.last().unwrap().distance(goal) < 0.0001);
        let mut pose = start;
        let mut curvature = 0.0;
        let mut length = 0.0;
        let mut min_curvature = 0.0_f64;
        let mut max_curvature = 0.0_f64;
        for point in &points[1..] {
            let dx = point.x_m - pose.x_m;
            let dy = point.y_m - pose.y_m;
            let yaw_change = 2.0 * angle_error(dy.atan2(dx), pose.yaw_rad);
            let chord = dx.hypot(dy);
            let ds = if yaw_change.abs() < 1e-9 {
                chord
            } else {
                chord * yaw_change / (2.0 * (yaw_change * 0.5).sin())
            };
            let next_curvature = 2.0 * yaw_change / ds - curvature;
            assert!(next_curvature.abs() <= config.max_curvature_per_m + 1e-8);
            assert!(
                (next_curvature - curvature).abs()
                    <= config.max_curvature_rate_per_s / config.max_speed_mps * ds + 1e-8
            );
            min_curvature = min_curvature.min(next_curvature);
            max_curvature = max_curvature.max(next_curvature);
            curvature = next_curvature;
            pose = Pose2 {
                x_m: point.x_m,
                y_m: point.y_m,
                yaw_rad: pose.yaw_rad + yaw_change,
            };
            assert!(pose_clear(&config, pose, &[], config.clearance_m));
            length += ds;
        }
        assert!(
            length < 1.0,
            "short sideways correction must not require a full loop: {length}"
        );
        assert!(min_curvature < -0.1 && max_curvature > 0.1);
        assert!(angle_error(pose.yaw_rad, 0.0).abs() <= config.goal_heading_tolerance_rad * 0.5);

        // A shortcut still uses the inflated obstacle grid. It cannot accept
        // the otherwise smooth connection when an obstacle occupies its middle.
        let obstacle = ObstacleDisc {
            center: points[points.len() / 2],
            radius_m: 0.05,
        };
        let occupied = Grid::new(&config, &[obstacle]);
        assert!(car_two_arc_connection(start, 0.0, goal, 0.0, &config, &occupied).is_none());
        assert!(
            car_two_arc_connection(
                start,
                0.0,
                start.body_to_world(Point2 {
                    x_m: -0.3,
                    y_m: 0.0
                }),
                0.0,
                &config,
                &grid
            )
            .is_none()
        );
    }

    #[test]
    fn terminal_connection_reverses_steering_continuously_and_keeps_its_actual_endpoint() {
        let mut config = NavigationConfig::simulation(
            Rect {
                min_x_m: 0.0,
                min_y_m: 0.0,
                max_x_m: 7.0,
                max_y_m: 5.0,
            },
            Footprint {
                front_m: 0.22,
                rear_m: 0.18,
                half_width_m: 0.13,
            },
            FrameId("map".into()),
        );
        config.max_curvature_rate_per_s = 8.0;
        config.goal_tolerance_m = 0.01;
        let grid = Grid::new(&config, &[]);
        let start = Pose2 {
            x_m: 3.0,
            y_m: 2.5,
            yaw_rad: 0.0,
        };
        let initial_curvature = 2.0;
        let (_, target, _) =
            car_primitive(start, initial_curvature, -1.5, 0.32, &config, &grid).unwrap();
        let local = start.world_to_body(target.point());
        let circle_k = 2.0 * local.y_m / (local.x_m.powi(2) + local.y_m.powi(2));
        let circle_length = 2.0 * local.y_m.atan2(local.x_m) / circle_k;
        assert!(
            circle_k < 0.0,
            "the old arc jumps from positive to negative steering"
        );
        let old_endpoint = integrate(start, circle_length, circle_k);
        assert!(old_endpoint.point().distance(target.point()) < 1e-12);
        let (_, ramped_old, _) = car_primitive(
            start,
            initial_curvature,
            circle_k,
            circle_length,
            &config,
            &grid,
        )
        .unwrap();
        assert!(ramped_old.point().distance(target.point()) > config.goal_tolerance_m);

        let (points, endpoint, final_curvature) = car_terminal_connection(
            start,
            initial_curvature,
            target.point(),
            Some(target.yaw_rad),
            &config,
            &grid,
        )
        .unwrap();
        assert!(endpoint.point().distance(target.point()) < config.goal_tolerance_m * 0.25);
        assert!(
            angle_error(endpoint.yaw_rad, target.yaw_rad).abs()
                <= config.goal_heading_tolerance_rad * 0.5
        );
        assert_eq!(points.last(), Some(&endpoint.point()));
        assert!(final_curvature < 0.0);
        // Recover each sampled arc's yaw and curvature from its chord. This
        // independently checks the returned points instead of trusting an
        // unsampled steering value or a last point overwritten with the goal.
        let mut pose = start;
        let mut curvature = initial_curvature;
        for point in points {
            let dx = point.x_m - pose.x_m;
            let dy = point.y_m - pose.y_m;
            let yaw_change = 2.0 * angle_error(dy.atan2(dx), pose.yaw_rad);
            let chord = dx.hypot(dy);
            let ds = if yaw_change.abs() < 1e-9 {
                chord
            } else {
                chord * yaw_change / (2.0 * (yaw_change * 0.5).sin())
            };
            let next_curvature = 2.0 * yaw_change / ds - curvature;
            assert!(
                (next_curvature - curvature).abs()
                    <= config.max_curvature_rate_per_s / config.max_speed_mps * ds + 1e-9
            );
            assert!(next_curvature.abs() <= config.max_curvature_per_m + 1e-9);
            pose = Pose2 {
                x_m: point.x_m,
                y_m: point.y_m,
                yaw_rad: pose.yaw_rad + yaw_change,
            };
            curvature = next_curvature;
        }
        assert!((curvature - final_curvature).abs() < 1e-9);
        assert!(angle_error(pose.yaw_rad, endpoint.yaw_rad).abs() < 1e-9);
        assert!(
            car_terminal_connection(
                start,
                initial_curvature,
                target.point(),
                Some(target.yaw_rad + config.goal_heading_tolerance_rad),
                &config,
                &grid,
            )
            .is_none()
        );
    }

    #[test]
    fn car_primitives_cannot_jump_an_occupied_corner_between_free_samples() {
        let config = NavigationConfig::simulation(
            Rect {
                min_x_m: 0.0,
                min_y_m: 0.0,
                max_x_m: 7.0,
                max_y_m: 5.0,
            },
            Footprint {
                front_m: 0.22,
                rear_m: 0.18,
                half_width_m: 0.13,
            },
            FrameId("map".into()),
        );
        let mut grid = Grid::new(&config, &[]);
        grid.blocked[33 * grid.width + 43] = true;
        let start = Pose2 {
            x_m: 4.360511494180138,
            y_m: 3.4066422375933807,
            yaw_rad: -0.22424239384201927,
        };
        // Both old 25 mm endpoint samples are free, but the connecting motion
        // crosses from (43,34) to (44,33) past occupied cell (43,33).
        for distance in [0.0, 0.025, 0.05] {
            let sample = integrate(start, distance, 0.0);
            assert!(!grid.blocked[grid.index(sample.point()).unwrap()]);
        }
        assert!(car_primitive(start, 0.0, 0.0, 0.05, &config, &grid).is_none());
        assert!(
            car_terminal_connection(
                start,
                0.0,
                integrate(start, 0.05, 0.0).point(),
                None,
                &config,
                &grid,
            )
            .is_none()
        );
    }

    #[test]
    fn stop_envelope_covers_obstacle_between_straight_and_fixed_curvature_paths() {
        let mut config = NavigationConfig::simulation(
            Rect {
                min_x_m: 0.0,
                min_y_m: 0.0,
                max_x_m: 7.0,
                max_y_m: 5.0,
            },
            Footprint {
                front_m: 0.01,
                rear_m: 0.01,
                half_width_m: 0.01,
            },
            FrameId("map".into()),
        );
        config.clearance_m = 0.005;
        let start = Pose2 {
            x_m: 3.0,
            y_m: 2.5,
            yaw_rad: 0.0,
        };
        let mut ramped = start;
        // A one-second stop from 1 m/s, while 2/m steering returns at 2/m/s.
        for i in 1..=1000 {
            let t = f64::from(i) / 1000.0;
            ramped = integrate(ramped, (1.0 - t) * 0.001, 2.0 * (1.0 - t));
        }
        let obstacle = ObstacleDisc {
            center: ramped.point(),
            radius_m: 0.003,
        };
        // Checking just these two boundary paths would miss the actual ramped stop.
        for i in 0..=1000 {
            let distance = f64::from(i) / 2000.0;
            for curvature in [0.0, 2.0] {
                assert!(pose_clear(
                    &config,
                    integrate(start, distance, curvature),
                    &[obstacle],
                    config.clearance_m
                ));
            }
        }
        assert!(!pose_clear(
            &config,
            ramped,
            &[obstacle],
            config.clearance_m
        ));
        assert!(!stop_reachable_clear(&config, start, 0.5, &[obstacle]));
    }
}

#[cfg(test)]
mod transition_rollout_tests {
    use super::*;

    fn scene(preview_horizon_s: f64) -> (NavigationConfig, Pose2) {
        let mut config = NavigationConfig::simulation(
            Rect {
                min_x_m: -5.0,
                min_y_m: -5.0,
                max_x_m: 5.0,
                max_y_m: 5.0,
            },
            Footprint {
                front_m: 0.22,
                rear_m: 0.18,
                half_width_m: 0.13,
            },
            FrameId("map".into()),
        );
        config.max_speed_mps = 1.0;
        config.max_decel_mps2 = 2.6;
        config.max_lateral_accel_mps2 = 0.5;
        config.control_period_ms = 100;
        config.preview_horizon_s = preview_horizon_s;
        config.validate().unwrap();
        (config, Pose2::default())
    }

    #[test]
    fn near_goal_and_short_preview_cannot_bypass_the_full_command_lateral_peak() {
        // Both endpoints are exactly 0.5 m/s², but the joint braking/steering
        // ramp reaches 0.5301887464 at 44.87 ms. Geometry may finish earlier;
        // that must not truncate the whole adopted tick's dynamic check.
        let target_speed = (0.5_f64 / 0.9).sqrt();
        assert!((target_speed.powi(2) * 0.9 - 0.5).abs() < 1e-12);
        for preview in [0.001, 2.0] {
            let (config, pose) = scene(preview);
            let grid = Grid::new(&config, &[]);
            for goal_distance in [0.0, 0.0001] {
                for sign in [-1.0, 1.0] {
                    assert_eq!(
                        rollout(
                            &config,
                            pose,
                            1.0,
                            target_speed,
                            0.5 * sign,
                            0.9 * sign,
                            &[],
                            goal_distance,
                            &grid,
                            None,
                        )
                        .unwrap_err(),
                        RolloutRejection::LateralAcceleration,
                        "preview={preview}, distance={goal_distance}, sign={sign}",
                    );
                }
            }
        }
    }

    #[test]
    fn safe_joint_braking_and_steering_is_not_rejected_by_independent_maxima() {
        // Braking at 4 m/s² keeps the actual peak at the initial 0.5 m/s².
        // max(speed)² * max(abs(curvature)) would incorrectly reject it at 0.9.
        for preview in [0.001, 2.0] {
            let (mut config, pose) = scene(preview);
            config.max_decel_mps2 = 4.0;
            config.validate().unwrap();
            let grid = Grid::new(&config, &[]);
            for goal_distance in [0.0, 0.0001, 10.0] {
                for sign in [-1.0, 1.0] {
                    let prediction = rollout(
                        &config,
                        pose,
                        1.0,
                        0.6,
                        0.5 * sign,
                        0.9 * sign,
                        &[],
                        goal_distance,
                        &grid,
                        None,
                    )
                    .unwrap();
                    assert!(prediction.endpoint.valid());
                    assert!(prediction.distance_m <= goal_distance + 1e-12);
                }
            }
        }
    }
}

#[cfg(test)]
mod exit_heading_arrival;
