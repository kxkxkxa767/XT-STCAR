//! Grid A*, bounded forward-car lattice search, and curvature rollout.
//! Collision envelopes are conservative; this is not a hardware braking guarantee.
//! The heading-aware search uses a weighted heuristic and a hard node budget;
//! failure means no candidate was found within that budget, not proof of no route.
//! Pure-pursuit geometry: R. Craig Coulter, CMU-RI-TR-92-01 (1992):
//! <https://publications.ri.cmu.edu/implementation-of-the-pure-pursuit-path-tracking-algorithm>
//! Forward-car circle/tangent geometry: LaValle, Planning Algorithms §15.3.1:
//! <https://lavalle.pl/planning/node821.html>
use crate::autonomy::{Footprint, HalfPlane, ObstacleDisc, Point2, Pose2, PoseEstimate, Rect};
use crate::reference::{PreparedReference, ReferenceCursor};
use crate::tracking::{PathTracker, TrackInput, TrackingConfig, TrackingDiagnostics};
use crate::{FrameId, MotionIntent, Timestamp, ValidationError};
use serde::{Deserialize, Serialize};
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

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
    pub curvature_speed_limits: usize,
    pub near_zero_speed: usize,
    pub rollouts_evaluated: usize,
    pub accepted: usize,
    pub stop_reachable: usize,
    pub grid: usize,
    pub footprint: usize,
    pub sample_budget: usize,
    pub reference: usize,
}

#[derive(Clone, Copy, Debug, Default, Serialize)]
pub struct NavigationDiagnostics {
    pub travel_boundary: Option<HalfPlane>,
    pub route_revision: u64,
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
    pub candidates: CandidateDiagnostics,
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

pub struct Navigator {
    config: NavigationConfig,
    travel_boundary: Option<HalfPlane>,
    last_step: Option<Timestamp>,
    last_curvature: f64,
    route_revision: u64,
    diagnostics: NavigationDiagnostics,
    route: Option<(Point2, Option<f64>, Vec<Point2>, usize)>,
}

impl Navigator {
    pub fn new(config: NavigationConfig) -> Result<Self, ValidationError> {
        config.validate()?;
        Ok(Self {
            config,
            travel_boundary: None,
            last_step: None,
            last_curvature: 0.0,
            route_revision: 0,
            diagnostics: NavigationDiagnostics::default(),
            route: None,
        })
    }

    pub fn config(&self) -> &NavigationConfig {
        &self.config
    }

    /// Change the allowed travel domain without inventing actuator motion.
    /// Geometry is validated by HalfPlane's constructors. A cached route cannot
    /// survive adding, moving, or removing a boundary, even with the same goal.
    pub fn set_travel_boundary(&mut self, boundary: Option<HalfPlane>) {
        if self.travel_boundary != boundary {
            self.travel_boundary = boundary;
            self.route = None;
        }
    }

    /// Call during a task hold; no timer expiry or motion history is invented.
    pub fn stop(&mut self, now: Timestamp) -> Result<NavigationDecision, ValidationError> {
        if self.last_step.is_some_and(|old| now < old) {
            return Err(ValidationError("navigation time regresses".into()));
        }
        self.last_step = Some(now);
        self.last_curvature = 0.0;
        self.route = None;
        Ok(NavigationDecision::stopped(
            NavigationStatus::Blocked,
            "task_stop",
        ))
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
        if self.last_step.is_some_and(|old| now < old)
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
        let period = self.config.control_period_ms as f64 / 1000.0;
        self.diagnostics = NavigationDiagnostics {
            travel_boundary: self.travel_boundary,
            route_revision: self.route_revision,
            ..NavigationDiagnostics::default()
        };
        let dt = self
            .last_step
            .map_or(period, |at| ((now.0 - at.0) as f64 / 1000.0).min(period));
        self.last_step = Some(now);
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
            return self.stop(now);
        }
        let pose = estimate.pose;
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
        let heading_reached = goal_heading_rad.is_none_or(|yaw| {
            angle_error(pose.yaw_rad, yaw).abs() <= self.config.goal_heading_tolerance_rad
        });
        if distance <= self.config.goal_tolerance_m && heading_reached && estimate.speed_mps <= 0.02
        {
            self.last_curvature = 0.0;
            return Ok(NavigationDecision::stopped(
                NavigationStatus::Reached,
                "goal_reached",
            ));
        }
        if distance <= self.config.goal_tolerance_m && heading_reached {
            // A target inside the arrival region may now lie behind the body.
            // Request the stop contract and wait for measured standstill.
            return Ok(self.blocked("goal_braking"));
        }
        let grid = Grid::with_boundary(&self.config, obstacles, self.travel_boundary);
        if self.plan_on_grid(pose.point(), goal, &grid).is_none() {
            return Ok(self.blocked("no_grid_path"));
        }
        if self
            .route
            .as_ref()
            .is_none_or(|(old_goal, old_heading, _, _)| {
                *old_goal != goal || *old_heading != goal_heading_rad
            })
        {
            self.route = self
                .kinematic_path(pose, goal, goal_heading_rad, &grid)
                .map(|path| (goal, goal_heading_rad, path, 0));
            if self.route.is_some() {
                self.route_revision = self.route_revision.saturating_add(1);
                self.diagnostics.route_revision = self.route_revision;
            }
        }
        let Some((_, _, route, progress)) = &mut self.route else {
            return Ok(self.blocked("no_forward_kinematic_path"));
        };
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
        let slew = self.config.max_curvature_rate_per_s * dt;
        let low = (self.last_curvature - slew).max(-self.config.max_curvature_per_m);
        let high = (self.last_curvature + slew).min(self.config.max_curvature_per_m);
        let preferred = desired_curvature.clamp(low, high);
        self.diagnostics.slew_limited_curvature_per_m = Some(preferred);
        let speed_upper = self
            .config
            .max_speed_mps
            .min(speed_limit_mps)
            .min(estimate.speed_mps + self.config.max_accel_mps2 * dt);
        let speed_lower = (estimate.speed_mps - self.config.max_decel_mps2 * dt).max(0.0);
        let goal_speed = (2.0
            * self.config.max_decel_mps2
            * (remaining_distance - self.config.goal_tolerance_m * 0.5).max(0.0))
        .mul_add(1.0, (self.config.max_decel_mps2 * period).powi(2))
        .sqrt()
            - self.config.max_decel_mps2 * period;
        let mut best: Option<(f64, MotionIntent)> = None;
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
                self.diagnostics.candidates.rollouts_evaluated += 1;
                let prediction = match rollout(
                    &self.config,
                    pose,
                    estimate.speed_mps.max(0.0),
                    speed,
                    self.last_curvature,
                    curvature,
                    obstacles,
                    remaining_distance,
                    &grid,
                    goal_heading_rad.map(|goal_heading_rad| RolloutReference {
                        cursor: reference_window.cursor_to_end(),
                        initial_arc_m: current_reference.arc_m,
                        goal_heading_rad,
                    }),
                ) {
                    Ok(prediction) => {
                        self.diagnostics.candidates.accepted += 1;
                        prediction
                    }
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
                            RolloutRejection::Reference => {
                                &mut self.diagnostics.candidates.reference
                            }
                        };
                        *count += 1;
                        continue;
                    }
                };
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
                let score = if let Some(cost) = prediction.reference_cost_m {
                    // Oriented goals need the whole approach, not just a final
                    // point: constant-curvature previews can hide intermediate
                    // errors across a bend. Steering preference has the scale
                    // of one tick's lateral displacement, so it cannot dominate
                    // the remaining path's position and heading corrections.
                    cost + 0.5
                        * (speed * period).powi(2)
                        * ((curvature - desired_curvature).abs()
                            + (curvature - self.last_curvature).abs())
                        - 0.3 * speed
                } else {
                    // Point-only waypoints retain their pursuit target. Fade
                    // steering preference near arrival as its geometric effect
                    // falls quadratically with remaining travel distance.
                    let approach_scale = (remaining_distance / self.config.lookahead_m).min(1.0);
                    prediction.endpoint.point().distance(tracking.target)
                        + 0.15 * approach_scale.powi(2) * (curvature - desired_curvature).abs()
                        + 0.08 * approach_scale.powi(2) * (curvature - self.last_curvature).abs()
                        - 0.3 * speed
                };
                if best.as_ref().is_none_or(|(old, _)| score < *old) {
                    self.diagnostics.selected_prediction_distance_m = Some(prediction.distance_m);
                    self.diagnostics.selected_prediction_endpoint = Some(prediction.endpoint);
                    self.diagnostics.selected_cross_track_m = Some(reference.distance_m);
                    self.diagnostics.selected_heading_error_rad = Some(heading_error);
                    self.diagnostics.selected_progress_m = Some(progress_m);
                    self.diagnostics.selected_reference_cost_m = prediction.reference_cost_m;
                    best = Some((
                        score,
                        MotionIntent {
                            speed_mps: speed,
                            curvature_per_m: curvature,
                        },
                    ));
                }
            }
        }
        if let Some((_, intent)) = best {
            self.diagnostics.selected_curvature_per_m = Some(intent.curvature_per_m);
            self.last_curvature = intent.curvature_per_m;
            Ok(NavigationDecision {
                status: NavigationStatus::Driving,
                intent,
                path,
                reason: None,
                diagnostics: self.diagnostics,
            })
        } else {
            self.route = None; // changed observations require a new bounded search.
            let mut decision = self.blocked("no_collision_free_braking_trajectory");
            decision.path = path;
            Ok(decision)
        }
    }

    fn blocked(&mut self, reason: &str) -> NavigationDecision {
        self.last_curvature = 0.0;
        let mut decision = NavigationDecision::stopped(NavigationStatus::Blocked, reason);
        decision.diagnostics = self.diagnostics;
        decision
    }

    /// Forward-only lattice search supplements 2-D A*: a short grid route may
    /// be impossible for a car with bounded steering. States contain heading
    /// and curvature; primitives ramp steering at the configured rate and are
    /// sampled inside the same inflated occupancy domain as the local rollout.
    fn kinematic_path(
        &self,
        start: Pose2,
        goal: Point2,
        goal_heading_rad: Option<f64>,
        grid: &Grid,
    ) -> Option<Vec<Point2>> {
        // Try one bounded smooth approach before quantizing into lattice bins.
        // Small sideways corrections with the same final heading need an S,
        // which the five steering bins can otherwise replace with a full loop.
        if let Some(heading) = goal_heading_rad
            && let Some((points, _, _)) = car_two_arc_connection(
                start,
                self.last_curvature,
                goal,
                heading,
                &self.config,
                grid,
            )
        {
            let mut path = Vec::with_capacity(points.len() + 1);
            path.push(start.point());
            path.extend(points);
            return Some(path);
        }
        let mut nodes = vec![CarNode {
            pose: start,
            curvature: self.last_curvature,
            parent: None,
            cost: 0.0,
        }];
        let mut best = HashMap::new();
        best.insert(
            car_key(
                start,
                self.last_curvature,
                grid,
                self.config.max_curvature_per_m,
            )?,
            0usize,
        );
        let mut open = BinaryHeap::new();
        open.push(QueueNode {
            index: 0,
            score: start.point().distance(goal),
        });
        let length = (2.0 * grid.resolution).clamp(0.15, 0.4);
        while let Some(entry) = open.pop() {
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
            let local_goal = node.pose.world_to_body(goal);
            let squared = local_goal.x_m.powi(2) + local_goal.y_m.powi(2);
            if squared < 0.35_f64.powi(2)
                && local_goal.x_m > 0.0
                && let Some((terminal, _, _)) = car_terminal_connection(
                    node.pose,
                    node.curvature,
                    goal,
                    goal_heading_rad,
                    &self.config,
                    grid,
                )
            {
                let mut indices = vec![entry.index];
                while let Some(parent) = nodes[*indices.last()?].parent {
                    indices.push(parent);
                }
                indices.reverse();
                let mut path = vec![start.point()];
                for pair in indices.windows(2) {
                    let parent = nodes[pair[0]];
                    let child = nodes[pair[1]];
                    let (samples, _, _) = car_primitive(
                        parent.pose,
                        parent.curvature,
                        child.curvature,
                        length_for_primitive(grid),
                        &self.config,
                        grid,
                    )?;
                    path.extend(samples);
                }
                path.extend(terminal);
                return Some(path);
            }
            for curvature in
                [-1.0, -0.5, 0.0, 0.5, 1.0].map(|f| f * self.config.max_curvature_per_m)
            {
                let Some((_, next_pose, curvature)) = car_primitive(
                    node.pose,
                    node.curvature,
                    curvature,
                    length,
                    &self.config,
                    grid,
                ) else {
                    continue;
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
                if nodes.len() >= self.config.max_grid_cells {
                    return None;
                }
                let index = nodes.len();
                nodes.push(CarNode {
                    pose: next_pose,
                    curvature,
                    parent: Some(entry.index),
                    cost,
                });
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
        let grid = Grid::with_boundary(&self.config, obstacles, self.travel_boundary);
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
    if grid.blocked[cell] {
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

/// A short two-part approach with three bounded variables: the two target
/// curvatures and total length. Equal-length parts avoid another search axis.
/// Both parts use the existing steering ramp and inflated grid transitions.
/// This is a single initial connection attempt, not a replacement path planner.
fn car_two_arc_connection(
    start: Pose2,
    initial_curvature: f64,
    goal: Point2,
    goal_heading: f64,
    config: &NavigationConfig,
    grid: &Grid,
) -> Option<(Vec<Point2>, Pose2, f64)> {
    let local = start.world_to_body(goal);
    let distance = start.point().distance(goal);
    // Two lookahead distances cover a near-goal approach; the absolute cap
    // keeps even the largest valid configuration's additional work bounded.
    if local.x_m <= 0.0 || !(1e-6..=(2.0 * config.lookahead_m).min(2.0)).contains(&distance) {
        return None;
    }
    let heading_change = angle_error(goal_heading, start.yaw_rad);
    // Small-angle geometry supplies only a seed. Acceptance below checks the
    // actual curved endpoint, never this approximate solution.
    let offset = 4.0 * local.y_m / distance.powi(2);
    let mut variables = [
        (offset - heading_change / distance)
            .clamp(-config.max_curvature_per_m, config.max_curvature_per_m),
        (3.0 * heading_change / distance - offset)
            .clamp(-config.max_curvature_per_m, config.max_curvature_per_m),
        distance,
    ];
    let max_length = distance * std::f64::consts::FRAC_PI_2;
    let position_tolerance = (config.goal_tolerance_m * 0.25).min(0.0001);
    let sample = |parameters: [f64; 3]| {
        let (mut points, middle, middle_curvature) = car_primitive(
            start,
            initial_curvature,
            parameters[0],
            parameters[2] * 0.5,
            config,
            grid,
        )?;
        let (last, endpoint, curvature) = car_primitive(
            middle,
            middle_curvature,
            parameters[1],
            parameters[2] * 0.5,
            config,
            grid,
        )?;
        points.extend(last);
        Some((points, endpoint, curvature))
    };
    for _ in 0..8 {
        let result = sample(variables)?;
        let error = [
            result.1.x_m - goal.x_m,
            result.1.y_m - goal.y_m,
            angle_error(result.1.yaw_rad, goal_heading),
        ];
        if error[0].hypot(error[1]) < position_tolerance
            && error[2].abs() <= config.goal_heading_tolerance_rad * 0.5
        {
            return Some(result);
        }
        let mut system = [[0.0; 4]; 3];
        for column in 0..3 {
            let (step, upper) = if column < 2 {
                (
                    (config.max_curvature_per_m * 0.001).min(0.001),
                    config.max_curvature_per_m,
                )
            } else {
                ((distance * 0.001).min(0.0001), max_length)
            };
            let delta = if variables[column] + step <= upper {
                step
            } else {
                -step
            };
            let mut perturbed = variables;
            perturbed[column] += delta;
            let (_, endpoint, _) = sample(perturbed)?;
            system[0][column] = (endpoint.x_m - result.1.x_m) / delta;
            system[1][column] = (endpoint.y_m - result.1.y_m) / delta;
            system[2][column] = angle_error(endpoint.yaw_rad, result.1.yaw_rad) / delta;
        }
        for row in 0..3 {
            system[row][3] = error[row];
        }
        let change = solve_endpoint_correction(system)?;
        for index in 0..3 {
            variables[index] -= change[index];
            if !variables[index].is_finite() {
                return None;
            }
            variables[index] = if index < 2 {
                variables[index].clamp(-config.max_curvature_per_m, config.max_curvature_per_m)
            } else {
                variables[index].clamp(distance, max_length)
            };
        }
    }
    None
}

/// Fixed 3x3 elimination with partial pivoting; a singular local model declines
/// the shortcut and lets the bounded lattice search proceed normally.
fn solve_endpoint_correction(mut system: [[f64; 4]; 3]) -> Option<[f64; 3]> {
    for column in 0..3 {
        let pivot = (column..3)
            .max_by(|&a, &b| system[a][column].abs().total_cmp(&system[b][column].abs()))?;
        system.swap(column, pivot);
        let divisor = system[column][column];
        if !divisor.is_finite() || divisor.abs() < 1e-12 {
            return None;
        }
        for value in &mut system[column][column..] {
            *value /= divisor;
        }
        let pivot_row = system[column];
        for (row_index, row) in system.iter_mut().enumerate() {
            if row_index != column {
                let scale = row[column];
                for (value, pivot_value) in row[column..].iter_mut().zip(&pivot_row[column..]) {
                    *value -= scale * pivot_value;
                }
            }
        }
    }
    let result = [system[0][3], system[1][3], system[2][3]];
    result
        .iter()
        .all(|value| value.is_finite())
        .then_some(result)
}

/// Connect a nearby goal with the same steering ramp as every lattice edge.
/// The instantaneous circular solution is only an initial guess. A bounded
/// two-variable endpoint correction adjusts length and target curvature, then
/// accepts only the actual sampled endpoint, heading and collision checks.
fn car_terminal_connection(
    start: Pose2,
    initial_curvature: f64,
    goal: Point2,
    goal_heading: Option<f64>,
    config: &NavigationConfig,
    grid: &Grid,
) -> Option<(Vec<Point2>, Pose2, f64)> {
    let local = start.world_to_body(goal);
    let distance = start.point().distance(goal);
    if local.x_m <= 0.0 || !(1e-9..0.35).contains(&distance) {
        return None;
    }
    let circle_curvature = 2.0 * local.y_m / distance.powi(2);
    let mut curvature =
        circle_curvature.clamp(-config.max_curvature_per_m, config.max_curvature_per_m);
    let mut length = if circle_curvature.abs() < 1e-9 {
        local.x_m
    } else {
        2.0 * local.y_m.atan2(local.x_m) / circle_curvature
    };
    // A forward, nearby circular connector previously spanned less than pi
    // radians. Keep its maximum arc/chord ratio; never search a long loop here.
    let max_length = distance * std::f64::consts::FRAC_PI_2;
    let position_tolerance = (config.goal_tolerance_m * 0.25).min(0.0001);
    for _ in 0..8 {
        let result = car_primitive(start, initial_curvature, curvature, length, config, grid)?;
        let error_x = result.1.x_m - goal.x_m;
        let error_y = result.1.y_m - goal.y_m;
        if error_x.hypot(error_y) < position_tolerance {
            return goal_heading
                .is_none_or(|yaw| {
                    angle_error(result.1.yaw_rad, yaw).abs()
                        <= config.goal_heading_tolerance_rad * 0.5
                })
                .then_some(result);
        }
        // Finite differences use the same sampled ramp as the accepted route.
        // Fixed iterations and bounded perturbations add no unbounded solver.
        let perturb_k = (config.max_curvature_per_m * 0.001).min(0.001);
        let delta_k = if curvature + perturb_k <= config.max_curvature_per_m {
            perturb_k
        } else {
            -perturb_k
        };
        let perturb_s = (distance * 0.001).min(0.0001);
        let delta_s = if length + perturb_s <= max_length {
            perturb_s
        } else {
            -perturb_s
        };
        let (_, turn, _) = car_primitive(
            start,
            initial_curvature,
            curvature + delta_k,
            length,
            config,
            grid,
        )?;
        let (_, travel, _) = car_primitive(
            start,
            initial_curvature,
            curvature,
            length + delta_s,
            config,
            grid,
        )?;
        let dx_k = (turn.x_m - result.1.x_m) / delta_k;
        let dy_k = (turn.y_m - result.1.y_m) / delta_k;
        let dx_s = (travel.x_m - result.1.x_m) / delta_s;
        let dy_s = (travel.y_m - result.1.y_m) / delta_s;
        let determinant = dx_k * dy_s - dx_s * dy_k;
        if !determinant.is_finite() || determinant.abs() < 1e-12 {
            return None;
        }
        curvature = (curvature - (error_x * dy_s - error_y * dx_s) / determinant)
            .clamp(-config.max_curvature_per_m, config.max_curvature_per_m);
        length =
            (length - (dx_k * error_y - dy_k * error_x) / determinant).clamp(distance, max_length);
    }
    None
}

fn car_primitive(
    start: Pose2,
    initial_curvature: f64,
    target_curvature: f64,
    length: f64,
    config: &NavigationConfig,
    grid: &Grid,
) -> Option<(Vec<Point2>, Pose2, f64)> {
    let count = (length / (grid.resolution / 3.0).min(0.025))
        .ceil()
        .max(1.0) as usize;
    let ds = length / count as f64;
    let max_change = config.max_curvature_rate_per_s / config.max_speed_mps * ds;
    let mut pose = start;
    let mut curvature = initial_curvature;
    let mut points = Vec::with_capacity(count);
    for _ in 0..count {
        let next = target_curvature.clamp(curvature - max_change, curvature + max_change);
        let previous = pose.point();
        pose = integrate(pose, ds, (curvature + next) * 0.5);
        curvature = next;
        if !grid.transition_clear(previous, pose.point()) {
            return None;
        }
        points.push(pose.point());
    }
    Some((points, pose, curvature))
}

fn body_radius(footprint: Footprint) -> f64 {
    footprint
        .front_m
        .max(footprint.rear_m)
        .hypot(footprint.half_width_m)
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
struct RolloutPrediction {
    endpoint: Pose2,
    distance_m: f64,
    reference_cost_m: Option<f64>,
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
    Grid,
    Footprint,
    SampleBudget,
    Reference,
}

/// Predict normal candidate motion separately from the emergency stopping
/// envelope. Slower targets change the former without shrinking the latter
/// below the measured-speed reaction and braking requirement.
#[allow(clippy::too_many_arguments)]
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
    mut reference: Option<RolloutReference<'_>>,
) -> Result<RolloutPrediction, RolloutRejection> {
    let reaction_s = config.control_period_ms as f64 / 1000.0;
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
    if !stop_reachable_clear(
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
        return Err(RolloutRejection::StopReachable);
    }
    // At most one control period and `step` meters per integration step. The
    // total loop has a separate hard cap even for extreme valid configurations.
    let dt_limit = reaction_s.min(step / envelope_speed.max(1e-6));
    let mut sample = pose;
    let mut speed = current_speed.max(0.0);
    let mut curvature = initial_curvature;
    let mut elapsed = 0.0;
    let mut distance_m = 0.0;
    let mut reference_integral = 0.0;
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
        // Speed integration is exact for the configured piecewise linear ramp;
        // curvature uses its time average over this short step. This remains a
        // sampled kinematic prediction, not a measured actuator response.
        let next = integrate(sample, ds, curvature_integral / dt);
        let max_curvature = curvature.abs().max(next_curvature.abs());
        // Bound a full step's body-point travel from its starting footprint,
        // preserving a conservative swept margin while curvature changes.
        let swept_padding =
            config.clearance_m + ds * (1.0 + max_curvature * body_radius(config.footprint));
        // Keep local control inside the same conservative domain as A*. Without
        // this, a safe rectangle could enter a cell whose inflated start is
        // blocked on the next tick, stranding an otherwise clear vehicle.
        if !grid.transition_clear(sample.point(), next.point()) {
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
            reference_integral += ds
                * (next.point().distance(location.point)
                    + angle_error(next.yaw_rad, heading).abs() / config.max_curvature_per_m);
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
        navigator.last_curvature = 0.4;
        navigator.route = Some((goal, None, vec![pose.point(), goal], 0));
        navigator.set_travel_boundary(Some(boundary));
        assert!(navigator.route.is_none());
        assert_eq!(navigator.last_curvature, 0.4);
        navigator.route = Some((goal, None, vec![pose.point(), goal], 0));
        navigator.set_travel_boundary(Some(boundary));
        assert!(navigator.route.is_some());
        navigator.set_travel_boundary(None);
        assert!(navigator.route.is_none());
        assert_eq!(navigator.last_curvature, 0.4);
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
