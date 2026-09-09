//! Grid A*, bounded forward-car lattice search, and curvature rollout.
//! Collision envelopes are conservative; this is not a hardware braking guarantee.
//! The heading-aware search uses a weighted heuristic and a hard node budget;
//! failure means no candidate was found within that budget, not proof of no route.
//! Pure-pursuit geometry: R. Craig Coulter, CMU-RI-TR-92-01 (1992):
//! <https://publications.ri.cmu.edu/implementation-of-the-pure-pursuit-path-tracking-algorithm>
//! Forward-car circle/tangent geometry: LaValle, Planning Algorithms §15.3.1:
//! <https://lavalle.pl/planning/node821.html>
use crate::autonomy::{Footprint, ObstacleDisc, Point2, Pose2, PoseEstimate, Rect};
use crate::tracking::{PathTracker, TrackInput, TrackingConfig};
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

#[derive(Clone, Debug, Serialize)]
pub struct NavigationDecision {
    pub status: NavigationStatus,
    pub intent: MotionIntent,
    pub path: Vec<Point2>,
    pub reason: Option<String>,
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
        }
    }
}

pub struct Navigator {
    config: NavigationConfig,
    last_step: Option<Timestamp>,
    last_curvature: f64,
    route: Option<(Point2, Option<f64>, Vec<Point2>, usize)>,
}

impl Navigator {
    pub fn new(config: NavigationConfig) -> Result<Self, ValidationError> {
        config.validate()?;
        Ok(Self {
            config,
            last_step: None,
            last_curvature: 0.0,
            route: None,
        })
    }

    pub fn config(&self) -> &NavigationConfig {
        &self.config
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
        if !pose_clear(&self.config, pose, obstacles, self.config.clearance_m) {
            return Ok(self.blocked("current_footprint_collision_or_boundary"));
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
        let grid = Grid::new(&self.config, obstacles);
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
        let target = tracking.target;
        let desired_curvature = tracking.curvature_per_m;
        let slew = self.config.max_curvature_rate_per_s * dt;
        let low = (self.last_curvature - slew).max(-self.config.max_curvature_per_m);
        let high = (self.last_curvature + slew).min(self.config.max_curvature_per_m);
        let preferred = desired_curvature.clamp(low, high);
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
                continue; // A normal candidate cannot exceed braking/turning limits.
            }
            for fraction in [1.0, 0.5] {
                let speed = (nominal * fraction).max(speed_lower).min(nominal);
                if speed < 1e-6 {
                    continue;
                }
                let Some(endpoint) = rollout(
                    &self.config,
                    pose,
                    estimate.speed_mps.max(speed),
                    curvature,
                    obstacles,
                    remaining_distance,
                    &grid,
                ) else {
                    continue;
                };
                // Track the local path first; curvature preference avoids branch chatter.
                let score = endpoint.point().distance(target)
                    + 0.15 * (curvature - desired_curvature).abs()
                    + 0.08 * (curvature - self.last_curvature).abs()
                    - 0.3 * speed;
                if best.as_ref().is_none_or(|(old, _)| score < *old) {
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
            self.last_curvature = intent.curvature_per_m;
            Ok(NavigationDecision {
                status: NavigationStatus::Driving,
                intent,
                path,
                reason: None,
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
        NavigationDecision::stopped(NavigationStatus::Blocked, reason)
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
            if squared < 0.35_f64.powi(2) && local_goal.x_m > 0.0 {
                let k = 2.0 * local_goal.y_m / squared;
                let length = if k.abs() < 1e-9 {
                    local_goal.x_m
                } else {
                    2.0 * local_goal.y_m.atan2(local_goal.x_m) / k
                };
                if k.abs() <= self.config.max_curvature_per_m
                    && goal_heading_rad.is_none_or(|yaw| {
                        angle_error(node.pose.yaw_rad + k * length, yaw).abs()
                            <= self.config.goal_heading_tolerance_rad * 0.5
                    })
                    && car_arc_clear(node.pose, length, k, grid)
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
                    let count = (length / 0.025).ceil().max(1.0) as usize;
                    for i in 1..=count {
                        path.push(
                            integrate(node.pose, length * i as f64 / count as f64, k).point(),
                        );
                    }
                    *path.last_mut()? = goal;
                    return Some(path);
                }
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
        let grid = Grid::new(&self.config, obstacles);
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

fn car_arc_clear(start: Pose2, length: f64, curvature: f64, grid: &Grid) -> bool {
    let count = (length / (grid.resolution / 3.0).min(0.025))
        .ceil()
        .max(1.0) as usize;
    let mut previous = start.point();
    (0..=count).all(|i| {
        let next = integrate(start, length * i as f64 / count as f64, curvature).point();
        let clear = grid.transition_clear(previous, next);
        previous = next;
        clear
    })
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

/// Includes reaction travel and a complete deceleration distance at the larger
/// of measured and commanded speed. Sampling adds a swept-point motion bound.
fn rollout(
    config: &NavigationConfig,
    pose: Pose2,
    speed: f64,
    curvature: f64,
    obstacles: &[ObstacleDisc],
    goal_distance: f64,
    grid: &Grid,
) -> Option<Pose2> {
    let reaction_s = config.control_period_ms as f64 / 1000.0;
    let braking_distance = speed * reaction_s + speed * speed / (2.0 * config.max_decel_mps2);
    let distance = (speed * config.preview_horizon_s)
        .min(goal_distance)
        .max(braking_distance);
    let step = config.grid_resolution_m.min(config.clearance_m).min(0.05) / 2.0;
    let count = (distance / step).ceil().max(1.0) as usize;
    if count > 1024 {
        return None;
    }
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
        braking_distance + speed * reaction_s,
        obstacles,
    ) {
        return None;
    }
    let ds = distance / count as f64;
    let swept_padding =
        config.clearance_m + ds * (1.0 + curvature.abs() * body_radius(config.footprint)) / 2.0;
    let mut previous = pose.point();
    for i in 0..=count {
        let sample = integrate(pose, ds * i as f64, curvature);
        // Keep local control inside the same conservative domain as A*. Without
        // this, a safe rectangle could enter a cell whose inflated start is
        // blocked on the next tick, stranding an otherwise clear vehicle.
        if !grid.transition_clear(previous, sample.point()) {
            return None;
        }
        previous = sample.point();
        if !pose_clear(config, sample, obstacles, swept_padding) {
            return None;
        }
    }
    Some(integrate(pose, distance, curvature))
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
    width: usize,
    height: usize,
    resolution: f64,
    bounds: Rect,
    blocked: Vec<bool>,
}
impl Grid {
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

    fn new(config: &NavigationConfig, obstacles: &[ObstacleDisc]) -> Self {
        let width = ((config.bounds.max_x_m - config.bounds.min_x_m) / config.grid_resolution_m)
            .ceil() as usize;
        let height = ((config.bounds.max_y_m - config.bounds.min_y_m) / config.grid_resolution_m)
            .ceil() as usize;
        let mut grid = Self {
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
                || p.y_m + inflation > config.bounds.max_y_m;
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
        assert!(!car_arc_clear(start, 0.05, 0.0, &grid));
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
