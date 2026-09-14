//! Bounded recovery within the original footprint, clearance and steering limits.

use super::primitive_envelope::ErrorBound;
use super::{Grid, NavigationConfig, body_radius};
use crate::autonomy::{HalfPlane, ObstacleDisc, Point2, Rect};
use serde::Serialize;
use std::cell::Cell;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ForwardSearchExit {
    #[default]
    NotRun,
    Found,
    OpenEmpty,
    NodeBudget,
    TerminalBudget,
    InvalidStart,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub struct PrimitiveGridFailure {
    pub continuous_domain: bool,
    pub sample_index: usize,
    pub from_cell: Option<usize>,
    pub to_cell: Option<usize>,
    pub blocked_cell: Option<usize>,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize)]
pub struct ForwardSearchWork {
    pub expanded_nodes: usize,
    pub generated_nodes: usize,
    pub allocated_nodes: usize,
    pub primitive_attempts: usize,
    pub primitive_rejections: usize,
    pub first_primitive_rejection: Option<PrimitiveGridFailure>,
    pub exit: ForwardSearchExit,
}

impl ForwardSearchWork {
    pub(super) fn accumulate(&mut self, other: Self) {
        self.expanded_nodes = self.expanded_nodes.saturating_add(other.expanded_nodes);
        self.generated_nodes = self.generated_nodes.saturating_add(other.generated_nodes);
        self.allocated_nodes = self.allocated_nodes.saturating_add(other.allocated_nodes);
        self.primitive_attempts = self
            .primitive_attempts
            .saturating_add(other.primitive_attempts);
        self.primitive_rejections = self
            .primitive_rejections
            .saturating_add(other.primitive_rejections);
        self.first_primitive_rejection = self
            .first_primitive_rejection
            .or(other.first_primitive_rejection);
        self.exit = other.exit;
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize)]
pub struct ForwardSearchDiagnostics {
    pub ordinary: ForwardSearchWork,
    pub recovery: ForwardSearchWork,
    pub recovery_attempted: bool,
    pub recovery_accepted: bool,
    pub recovery_active: bool,
    pub node_budget_exhausted: bool,
}

impl ForwardSearchDiagnostics {
    pub fn is_empty(&self) -> bool {
        *self == Self::default()
    }
}

pub(super) struct RecoveryGrid {
    pub active: Cell<bool>,
    pub diagnostics: Cell<ForwardSearchDiagnostics>,
    pub next_leg_error: Cell<ErrorBound>,
    pub normal_path_completed: Cell<bool>,
    obstacles: Vec<ObstacleDisc>,
    bounds: Rect,
    boundary: Option<HalfPlane>,
    radius: f64,
    maximum_curvature: f64,
}

impl RecoveryGrid {
    pub fn new(
        config: &NavigationConfig,
        obstacles: &[ObstacleDisc],
        boundary: Option<HalfPlane>,
    ) -> Self {
        let period = config.control_period_ms as f64 / 1000.0;
        let restart_speed = 0.5 * config.max_speed_mps.min(config.max_accel_mps2 * period);
        let restart_distance =
            2.0 * period * restart_speed + restart_speed.powi(2) / (2.0 * config.max_decel_mps2);
        Self {
            active: Cell::new(false),
            diagnostics: Cell::new(ForwardSearchDiagnostics::default()),
            next_leg_error: Cell::new(ErrorBound::default()),
            normal_path_completed: Cell::new(false),
            obstacles: obstacles.to_vec(),
            bounds: config.bounds,
            boundary,
            // Replace grid quantization with the continuous full body capsule.
            // Reserve the original stopping gate's distance for the smallest
            // positive candidate from standstill, so a statically clear route
            // cannot enter a region that forbids even that normal restart.
            radius: body_radius(config.footprint) + config.clearance_m + restart_distance,
            maximum_curvature: config.max_curvature_per_m,
        }
    }

    pub fn segment_clear(&self, from: Point2, to: Point2, padding: f64) -> bool {
        if !from.valid() || !to.valid() || !padding.is_finite() || padding < 0.0 {
            return false;
        }
        let radius = self.radius + padding;
        // Rectangle and half-plane domains are convex, so checking both
        // endpoint disks encloses the entire capsule between them.
        for point in [from, to] {
            if point.x_m - radius < self.bounds.min_x_m
                || point.x_m + radius > self.bounds.max_x_m
                || point.y_m - radius < self.bounds.min_y_m
                || point.y_m + radius > self.bounds.max_y_m
                || self
                    .boundary
                    .is_some_and(|b| !b.contains_disc(point, radius))
            {
                return false;
            }
        }
        let dx = to.x_m - from.x_m;
        let dy = to.y_m - from.y_m;
        let squared = dx * dx + dy * dy;
        if !squared.is_finite() {
            return false;
        }
        self.obstacles.iter().all(|obstacle| {
            let fraction = if squared == 0.0 {
                0.0
            } else {
                (((obstacle.center.x_m - from.x_m) * dx + (obstacle.center.y_m - from.y_m) * dy)
                    / squared)
                    .clamp(0.0, 1.0)
            };
            let nearest = Point2 {
                x_m: from.x_m + fraction * dx,
                y_m: from.y_m + fraction * dy,
            };
            nearest.distance(obstacle.center) > radius + obstacle.radius_m
        })
    }
}

impl Grid {
    pub(super) fn enable_recovery(&self) {
        self.recovery.active.set(true);
        let mut diagnostics = self.recovery.diagnostics.get();
        diagnostics.recovery_active = true;
        self.recovery.diagnostics.set(diagnostics);
    }

    pub(super) fn recovery_active(&self) -> bool {
        self.recovery.active.get()
    }

    pub(super) fn forward_search(&self) -> ForwardSearchDiagnostics {
        self.recovery.diagnostics.get()
    }

    pub(super) fn completed_path_error(&self) -> ErrorBound {
        self.recovery.next_leg_error.get()
    }

    pub(super) fn sample_arc_bound_m(&self) -> f64 {
        (self.resolution / 3.0).min(0.025)
    }

    /// Check a modeled motion segment using its actual nonnegative arc length,
    /// never inferring traveled distance from a chord. For |p''(s)| <= K the
    /// center curve is inside a K L²/8 tube around endpoint interpolation.
    #[cfg(test)]
    pub(super) fn motion_transition_clear(
        &self,
        from: Point2,
        to: Point2,
        arc_length_m: f64,
    ) -> bool {
        self.motion_transition_clear_with_error(from, to, arc_length_m, 0.0)
    }

    pub(super) fn motion_transition_clear_with_error(
        &self,
        from: Point2,
        to: Point2,
        arc_length_m: f64,
        position_error_m: f64,
    ) -> bool {
        if !arc_length_m.is_finite()
            || arc_length_m < 0.0
            || !position_error_m.is_finite()
            || position_error_m < 0.0
        {
            return false;
        }
        if self.recovery_active() {
            self.recovery.segment_clear(
                from,
                to,
                self.recovery.maximum_curvature * arc_length_m.powi(2) / 8.0 + position_error_m,
            )
        } else {
            self.transition_clear(from, to)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::super::*;
    use super::*;

    struct Fixture {
        config: NavigationConfig,
        estimate: PoseEstimate,
        obstacles: Vec<ObstacleDisc>,
        at: Timestamp,
        obstacle_at: Timestamp,
        steering: SteeringEstimate,
        goal: Point2,
        boundary: HalfPlane,
        arrival: ArrivalBehavior,
    }

    fn fixture() -> Fixture {
        fixture_from(include_str!(
            "../../tests/fixtures/async_lqr_stopped_source.json"
        ))
    }

    fn fixture_from(json: &str) -> Fixture {
        let input: serde_json::Value = serde_json::from_str(json).unwrap();
        let cfg = &input["autonomy_config"];
        let source = &input["source"];
        let estimate: PoseEstimate = serde_json::from_value(source["pose"].clone()).unwrap();
        let tf: Pose2 = serde_json::from_value(cfg["lidar_in_body"].clone()).unwrap();
        let mut obstacles = Vec::new();
        let scan = &source["scan"];
        for (index, range) in scan["ranges_m"].as_array().unwrap().iter().enumerate() {
            if let Some(range) = range.as_f64() {
                let angle = scan["angle_min_rad"].as_f64().unwrap()
                    + index as f64 * scan["angle_increment_rad"].as_f64().unwrap();
                obstacles.push(ObstacleDisc {
                    center: estimate.pose.body_to_world(tf.body_to_world(Point2 {
                        x_m: range * angle.cos(),
                        y_m: range * angle.sin(),
                    })),
                    radius_m: cfg["laser_point_radius_m"].as_f64().unwrap(),
                });
            }
        }
        for point in source["road"]["observation"]["cones_body_m"]
            .as_array()
            .unwrap()
        {
            obstacles.push(ObstacleDisc {
                center: estimate
                    .pose
                    .body_to_world(serde_json::from_value(point.clone()).unwrap()),
                radius_m: cfg["cone_radius_m"].as_f64().unwrap(),
            });
        }
        let mission = &input["mission"]["output"];
        let arrival = &mission["arrival"];
        let boundary = &input["navigation"]["travel_boundary"];
        Fixture {
            config: serde_json::from_value(cfg["navigation"].clone()).unwrap(),
            estimate,
            obstacles,
            at: Timestamp(input["planned_at"].as_u64().unwrap()),
            obstacle_at: Timestamp(scan["captured_at"].as_u64().unwrap()),
            steering: SteeringEstimate {
                at: Timestamp(
                    input["navigation"]["execution_state"]["at"]
                        .as_u64()
                        .unwrap(),
                ),
                commanded_curvature_per_m:
                    input["navigation"]["execution_state"]["commanded_curvature_per_m"]
                        .as_f64()
                        .unwrap(),
                applied_curvature_per_m:
                    input["navigation"]["execution_state"]["applied_curvature_per_m"]
                        .as_f64()
                        .unwrap(),
            },
            goal: serde_json::from_value(mission["point"].clone()).unwrap(),
            boundary: HalfPlane::new(
                serde_json::from_value(boundary["origin"].clone()).unwrap(),
                boundary["normal"]["y_m"]
                    .as_f64()
                    .unwrap()
                    .atan2(boundary["normal"]["x_m"].as_f64().unwrap()),
                boundary["max_projection_m"].as_f64().unwrap(),
            )
            .unwrap(),
            arrival: if arrival["type"] == "stop" {
                ArrivalBehavior::Stop
            } else {
                ArrivalBehavior::PassThrough {
                    next: serde_json::from_value(arrival["next"].clone()).unwrap(),
                    next_heading_rad: arrival["next_heading_rad"].as_f64(),
                    next_max_speed_mps: arrival["next_max_speed_mps"].as_f64().unwrap(),
                    admission_radius_m: arrival["admission_radius_m"].as_f64().unwrap(),
                }
            },
        }
    }

    #[test]
    fn recorded_stopped_input_has_no_original_first_layer_primitive() {
        let f = fixture();
        assert_eq!(f.obstacles.len(), 362);
        assert_eq!(f.at, Timestamp(27980));
        assert_eq!(f.obstacle_at, Timestamp(27900));
        assert_eq!(f.estimate.speed_mps, 0.0);
        assert_eq!(f.steering.applied_curvature_per_m, 0.0);
        let mut nav = Navigator::new(f.config.clone()).unwrap();
        nav.set_execution_state(f.steering).unwrap();
        nav.set_travel_boundary(Some(f.boundary));
        let grid = Grid::with_boundary(&f.config, &f.obstacles, Some(f.boundary));
        assert!(
            nav.plan_on_grid(f.estimate.pose.point(), f.goal, &grid)
                .is_some()
        );
        for target in [-2.0, -1.0, 0.0, 1.0, 2.0] {
            let failure = car_primitive_checked(
                f.estimate.pose,
                0.0,
                target,
                length_for_primitive(&grid),
                &f.config,
                &grid,
            )
            .unwrap_err();
            assert_eq!(failure.sample_index, 1);
            assert_eq!(failure.from_cell, Some(28 * grid.width + 36));
            assert_eq!(failure.to_cell, Some(28 * grid.width + 37));
            assert_eq!(failure.blocked_cell, Some(28 * grid.width + 37));
        }
        let mut work = ForwardSearchWork::default();
        assert!(
            nav.search_kinematic_path_from(
                f.estimate.pose,
                0.0,
                f.goal,
                None,
                &grid,
                f.config.max_grid_cells,
                &mut work,
            )
            .is_none()
        );
        assert_eq!(work.expanded_nodes, 1);
        assert_eq!(work.generated_nodes, 0);
        assert_eq!(work.primitive_attempts, 5);
        assert_eq!(work.exit, ForwardSearchExit::OpenEmpty);
        assert_eq!(nav.terminal_budget.snapshot().solver_attempts, 0);
    }

    #[test]
    fn full_recorded_context_recovers_through_continuous_body_clearance() {
        let f = fixture();
        let mut nav = Navigator::new(f.config.clone()).unwrap();
        nav.set_execution_state(f.steering).unwrap();
        nav.set_travel_boundary(Some(f.boundary));
        let decision = nav
            .plan_with_arrival(
                f.at,
                &f.estimate,
                &f.obstacles,
                f.obstacle_at,
                f.goal,
                None,
                f.config.max_speed_mps,
                f.arrival,
            )
            .unwrap();
        assert_eq!(decision.status, NavigationStatus::Driving, "{decision:?}");
        assert!(decision.intent.speed_mps > 0.0);
        assert!(decision.intent.curvature_per_m.abs() <= f.config.max_curvature_rate_per_s * 0.1);
        assert!(decision.diagnostics.candidates.rollouts_evaluated > 0);
        assert!(!decision.diagnostics.terminal_work.budget_exhausted);
        eprintln!(
            "original stopped fixture: route_position_error_m={}, sample_arc_bound_m={}, search={:?}, terminal={:?}",
            nav.recovery_route_error_m,
            nav.recovery_sample_arc_bound_m,
            decision.diagnostics.forward_search,
            decision.diagnostics.terminal_work
        );
    }

    #[test]
    fn capsules_cover_curved_interiors_and_reject_obstacle_contact() {
        let mut cfg = fixture().config;
        cfg.bounds = Rect {
            min_x_m: 0.0,
            min_y_m: 0.0,
            max_x_m: 10.0,
            max_y_m: 10.0,
        };
        let start = Pose2 {
            x_m: 5.0,
            y_m: 5.0,
            yaw_rad: 0.0,
        };
        let length = 0.4;
        let radius = 0.015;
        for direction in [-1.0, 1.0] {
            let curvature = direction * cfg.max_curvature_per_m;
            let end = integrate(start, length, curvature);
            let middle = integrate(start, length / 2.0, curvature);
            let outward_x = direction * middle.yaw_rad.sin();
            let outward_y = -direction * middle.yaw_rad.cos();
            let separation = body_radius(cfg.footprint) + cfg.clearance_m + radius - 0.001;
            let obstacles = [ObstacleDisc {
                center: Point2 {
                    x_m: middle.x_m + separation * outward_x,
                    y_m: middle.y_m + separation * outward_y,
                },
                radius_m: radius,
            }];
            let grid = Grid::with_boundary(&cfg, &obstacles, None);
            grid.enable_recovery();
            assert!(grid.transition_clear(start.point(), end.point()));
            assert!(!grid.motion_transition_clear(start.point(), end.point(), length));
        }
        let contact = [ObstacleDisc {
            center: Point2 {
                x_m: start.x_m,
                y_m: start.y_m + body_radius(cfg.footprint) + cfg.clearance_m + radius,
            },
            radius_m: radius,
        }];
        let grid = Grid::with_boundary(&cfg, &contact, None);
        grid.enable_recovery();
        assert!(!grid.transition_clear(start.point(), start.point()));
    }

    #[test]
    fn recovery_capsule_preserves_the_minimum_forward_restart_space() {
        let config = NavigationConfig::simulation(
            Rect {
                min_x_m: 0.0,
                min_y_m: 0.0,
                max_x_m: 7.0,
                max_y_m: 5.0,
            },
            crate::autonomy::Footprint {
                front_m: 0.22,
                rear_m: 0.18,
                half_width_m: 0.13,
            },
            crate::FrameId("map".into()),
        );
        // Actual step 390 from the original two-cone closed-loop regression:
        // 1.067410 mm static space, but the next 0.01 m/s normal candidate
        // needs 1.083333 mm under the unchanged stopping-disk gate.
        let trapped = Point2 {
            x_m: 4.299801450339034,
            y_m: 2.0533939874827416,
        };
        let before = Point2 {
            x_m: 4.28,
            y_m: 2.05,
        };
        let obstacles = [ObstacleDisc {
            center: Point2 { x_m: 4.3, y_m: 2.5 },
            radius_m: 0.15,
        }];
        let mut old_static_capsule = RecoveryGrid::new(&config, &obstacles, None);
        old_static_capsule.radius = body_radius(config.footprint) + config.clearance_m;
        assert!(old_static_capsule.segment_clear(before, trapped, 0.0));
        let grid = Grid::new(&config, &obstacles);
        grid.enable_recovery();
        assert!(grid.transition_clear(before, before));
        assert!(!grid.transition_clear(before, trapped));
        assert!(!grid.motion_transition_clear(before, trapped, before.distance(trapped)));
        let static_margin = trapped.distance(obstacles[0].center)
            - old_static_capsule.radius
            - obstacles[0].radius_m;
        let minimum_restart = 2.0 * 0.05 * 0.01 + 0.01_f64.powi(2) / (2.0 * 0.6);
        assert!(static_margin > 0.0 && static_margin < minimum_restart);
    }

    #[test]
    fn recovery_uses_remaining_node_budget_and_current_obstacles() {
        let f = fixture();
        let nav = Navigator::new(f.config.clone()).unwrap();
        let grid = Grid::with_boundary(&f.config, &f.obstacles, Some(f.boundary));
        let (path, _, _) = nav
            .kinematic_path_from(f.estimate.pose, 0.0, f.goal, None, &grid)
            .unwrap();
        let work = grid.forward_search();
        assert_eq!(work.ordinary.expanded_nodes, 1);
        assert_eq!(work.ordinary.primitive_attempts, 5);
        assert_eq!(work.ordinary.generated_nodes, 0);
        assert!(work.recovery_accepted);
        assert!(
            work.ordinary.allocated_nodes + work.recovery.allocated_nodes
                <= f.config.max_grid_cells
        );
        assert!(
            path.windows(2)
                .all(|pair| grid.transition_clear(pair[0], pair[1]))
        );

        let mut obstacles = f.obstacles.clone();
        let inserted = path
            .iter()
            .copied()
            .find(|p| p.distance(f.estimate.pose.point()) > 0.5)
            .unwrap();
        obstacles.push(ObstacleDisc {
            center: inserted,
            radius_m: 0.015,
        });
        let changed = Grid::with_boundary(&f.config, &obstacles, Some(f.boundary));
        changed.enable_recovery();
        assert!(
            !path
                .windows(2)
                .all(|pair| changed.transition_clear(pair[0], pair[1]))
        );
        assert!(!changed.motion_transition_clear(inserted, inserted, 0.0));

        let constrained = HalfPlane::new(f.estimate.pose.point(), 0.0, 0.05).unwrap();
        let boundary_grid = Grid::with_boundary(&f.config, &f.obstacles, Some(constrained));
        boundary_grid.enable_recovery();
        assert!(!boundary_grid.transition_clear(f.estimate.pose.point(), f.estimate.pose.point()));

        let mut limited = f.config.clone();
        limited.max_grid_cells = 1;
        // Directly exercise the original search allocation cap; this helper
        // does not need a valid whole-map NavigationConfig with just one cell.
        let mut work = ForwardSearchWork::default();
        assert!(
            nav.search_kinematic_path_from(
                f.estimate.pose,
                0.0,
                f.goal,
                None,
                &grid,
                limited.max_grid_cells,
                &mut work
            )
            .is_none()
        );
        assert_eq!(work.allocated_nodes, 1);
        assert_eq!(work.exit, ForwardSearchExit::NodeBudget);
    }

    #[test]
    fn recovery_executes_forward_across_quantized_exit_with_original_clearance() {
        let f = fixture();
        let mut nav = Navigator::new(f.config.clone()).unwrap();
        nav.set_execution_state(f.steering).unwrap();
        nav.set_travel_boundary(Some(f.boundary));
        let original_grid = Grid::with_boundary(&f.config, &f.obstacles, Some(f.boundary));
        let mut pose = f.estimate.pose;
        let mut speed = 0.0_f64;
        let mut curvature = 0.0_f64;
        let mut traveled = 0.0;
        let mut entered_quantized_obstacle_cell = false;
        let mut recovered_ticks = 0;
        // The first tick is the exact saved input. Subsequent inputs are an
        // independent 1 ms forward-only plant in those fixed observed obstacles;
        // this is a recovery regression, not replay of the original async run.
        for tick in 0..100 {
            let at = Timestamp(f.at.0 + tick * f.config.control_period_ms);
            let estimate = if tick == 0 {
                f.estimate.clone()
            } else {
                PoseEstimate {
                    captured_at: at,
                    pose,
                    speed_mps: speed,
                    yaw_rate_radps: speed * curvature,
                    ..f.estimate.clone()
                }
            };
            let decision = nav
                .plan_with_arrival(
                    at,
                    &estimate,
                    &f.obstacles,
                    if tick == 0 { f.obstacle_at } else { at },
                    f.goal,
                    None,
                    f.config.max_speed_mps,
                    f.arrival,
                )
                .unwrap();
            recovered_ticks += usize::from(decision.diagnostics.forward_search.recovery_active);
            let command = if decision.intent.speed_mps > 0.0 {
                MotionOutput::Drive {
                    speed_mps: decision.intent.speed_mps,
                    curvature_per_m: decision.intent.curvature_per_m,
                }
            } else {
                MotionOutput::Stop
            };
            nav.adopt_command(at, &command).unwrap();
            let (target_speed, target_curvature) = match command {
                MotionOutput::Drive {
                    speed_mps,
                    curvature_per_m,
                } => (speed_mps, curvature_per_m),
                MotionOutput::Stop => (0.0, 0.0),
            };
            for _ in 0..f.config.control_period_ms {
                let dt = 0.001;
                let rate = if target_speed >= speed {
                    f.config.max_accel_mps2
                } else {
                    f.config.max_decel_mps2
                };
                let next_speed = speed + (target_speed - speed).clamp(-rate * dt, rate * dt);
                let next_curvature = curvature
                    + (target_curvature - curvature).clamp(
                        -f.config.max_curvature_rate_per_s * dt,
                        f.config.max_curvature_rate_per_s * dt,
                    );
                let v = (speed + next_speed) * 0.5;
                let k = (curvature + next_curvature) * 0.5;
                let yaw_middle = pose.yaw_rad + v * k * dt * 0.5;
                pose.x_m += v * yaw_middle.cos() * dt;
                pose.y_m += v * yaw_middle.sin() * dt;
                pose.yaw_rad += v * k * dt;
                traveled += v * dt;
                speed = next_speed;
                curvature = next_curvature;
                assert!(speed >= 0.0 && speed <= f.config.max_speed_mps);
                assert!(curvature.abs() <= f.config.max_curvature_per_m);
                assert!(pose_clear(
                    &f.config,
                    pose,
                    &f.obstacles,
                    f.config.clearance_m
                ));
                assert!(f.boundary.contains_footprint(
                    f.config.footprint,
                    pose,
                    f.config.clearance_m
                ));
                entered_quantized_obstacle_cell |= original_grid
                    .index(pose.point())
                    .is_some_and(|i| original_grid.blocked[i]);
            }
            if traveled > 0.4 {
                break;
            }
        }
        assert!(traveled > 0.4, "only moved {traveled} m");
        assert!(entered_quantized_obstacle_cell);
        assert!(recovered_ticks > 1);
        assert!(pose.point().distance(f.goal) < f.estimate.pose.point().distance(f.goal));
    }

    #[test]
    fn light_stop_frontier_exhaustion_recovers_within_the_same_node_limit() {
        let f = fixture_from(include_str!(
            "../../tests/fixtures/async_lqr_light_stopped_source.json"
        ));
        let mut nav = Navigator::new(f.config.clone()).unwrap();
        nav.set_execution_state(f.steering).unwrap();
        nav.set_travel_boundary(Some(f.boundary));
        let grid = Grid::with_boundary(&f.config, &f.obstacles, Some(f.boundary));
        let mut normal = ForwardSearchWork::default();
        assert!(
            nav.search_kinematic_path_from(
                f.estimate.pose,
                0.0,
                f.goal,
                Some(0.0),
                &grid,
                f.config.max_grid_cells,
                &mut normal
            )
            .is_none()
        );
        assert_eq!(normal.expanded_nodes, 6);
        assert_eq!(normal.generated_nodes, 5);
        assert_eq!(normal.primitive_attempts, 30);
        assert_eq!(normal.exit, ForwardSearchExit::OpenEmpty);
        nav.terminal_budget.reset();
        let decision = nav
            .plan_with_arrival(
                f.at,
                &f.estimate,
                &f.obstacles,
                f.obstacle_at,
                f.goal,
                Some(0.0),
                0.18,
                f.arrival,
            )
            .unwrap();
        assert_eq!(decision.status, NavigationStatus::Driving, "{decision:?}");
        assert!(decision.diagnostics.forward_search.recovery_accepted);
        let work = decision.diagnostics.forward_search;
        assert!(
            work.ordinary.allocated_nodes + work.recovery.allocated_nodes
                <= f.config.max_grid_cells
        );
        assert!(!decision.diagnostics.terminal_work.budget_exhausted);
        let endpoint = *decision.path.last().unwrap();
        assert!(endpoint.distance(f.goal) <= f.config.goal_tolerance_m);
        assert_ne!(endpoint, f.goal);
        eprintln!(
            "light stopped fixture: route_position_error_m={}, sample_arc_bound_m={}, search={:?}, terminal={:?}",
            nav.recovery_route_error_m,
            nav.recovery_sample_arc_bound_m,
            decision.diagnostics.forward_search,
            decision.diagnostics.terminal_work
        );
    }

    #[test]
    fn later_legs_and_fallback_cannot_replenish_the_shared_node_ledger() {
        let f = fixture();
        let nav = Navigator::new(f.config.clone()).unwrap();
        let grid = Grid::with_boundary(&f.config, &f.obstacles, Some(f.boundary));
        // Represent nodes charged by an earlier leg using the same Grid.
        let mut earlier = ForwardSearchDiagnostics::default();
        earlier.ordinary.allocated_nodes = f.config.max_grid_cells - 2;
        grid.recovery.diagnostics.set(earlier);
        assert!(
            nav.kinematic_path_from(f.estimate.pose, 0.0, f.goal, None, &grid)
                .is_none()
        );
        let exhausted = grid.forward_search();
        assert_eq!(
            exhausted.ordinary.allocated_nodes + exhausted.recovery.allocated_nodes,
            f.config.max_grid_cells
        );
        assert_eq!(exhausted.recovery.allocated_nodes, 1);
        assert_eq!(exhausted.recovery.exit, ForwardSearchExit::NodeBudget);
        assert!(exhausted.node_budget_exhausted);
        assert!(
            nav.kinematic_path_from(f.estimate.pose, 0.0, f.goal, None, &grid)
                .is_none()
        );
        let repeated = grid.forward_search();
        assert_eq!(
            repeated.ordinary.allocated_nodes + repeated.recovery.allocated_nodes,
            f.config.max_grid_cells
        );
    }
}
