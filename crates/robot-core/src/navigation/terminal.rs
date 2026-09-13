//! Bounded terminal geometry. A seed is a numerical hint, never permission to
//! reuse a path: every solve checks its actual ramped samples in the new grid.
use super::{Grid, NavigationConfig, angle_error, car_primitive};
use crate::autonomy::{Point2, Pose2};
use serde::Serialize;
use std::cell::Cell;

/// A shared ledger for every terminal solver in one navigation call, including
/// lattice endpoint attempts and candidate checks. Lattice expansion retains
/// its separate pre-existing node budget.
#[derive(Clone, Copy, Debug, Default, Serialize)]
pub struct TerminalWorkDiagnostics {
    /// In-domain solver invocations; fast domain rejections are counted separately.
    pub solver_attempts: usize,
    pub iterations: usize,
    /// Samples reserved before each primitive. Early grid rejection can execute
    /// fewer samples; this is charged work, not a measured operation count.
    pub primitive_samples: usize,
    pub budget_exhausted: bool,
    pub solver_limit: usize,
    pub iteration_limit: usize,
    pub sample_limit: usize,
    pub continued_seed_attempts: usize,
    pub continued_seed_accepted: usize,
    /// An individual solver used all eight correction iterations without a
    /// certified connection; this is distinct from the shared work limit.
    pub solver_iteration_exhaustions: usize,
    pub domain_rejections: usize,
    pub primitive_grid_rejections: usize,
}

pub(super) struct TerminalBudget(Cell<TerminalWorkDiagnostics>);
impl Default for TerminalBudget {
    fn default() -> Self {
        Self(Cell::new(TerminalWorkDiagnostics {
            solver_limit: 256,
            iteration_limit: 1024,
            sample_limit: 65_536,
            ..TerminalWorkDiagnostics::default()
        }))
    }
}
impl TerminalBudget {
    pub(super) fn reset(&self) {
        self.0.set(Self::default().snapshot());
    }
    pub(super) fn snapshot(&self) -> TerminalWorkDiagnostics {
        self.0.get()
    }
    fn charge(&self, solvers: usize, iterations: usize, samples: usize) -> Option<()> {
        let mut work = self.0.get();
        if work.budget_exhausted {
            return None;
        }
        if work.solver_attempts + solvers > work.solver_limit
            || work.iterations + iterations > work.iteration_limit
            || work.primitive_samples + samples > work.sample_limit
        {
            work.budget_exhausted = true;
            self.0.set(work);
            return None;
        }
        work.solver_attempts += solvers;
        work.iterations += iterations;
        work.primitive_samples += samples;
        self.0.set(work);
        Some(())
    }
    fn solver(&self) -> Option<()> {
        self.charge(1, 0, 0)
    }
    fn iteration(&self) -> Option<()> {
        self.charge(0, 1, 0)
    }
}

#[derive(Clone, Copy, Debug)]
pub(super) struct TwoArcSeed {
    variables: [f64; 3],
    first_fraction: f64,
}
impl TwoArcSeed {
    pub(super) fn first_curvature(self) -> f64 {
        self.variables[0]
    }
    /// Subtract the projected full-period travel from the certified first
    /// segment. Keeping this ratio avoids resetting the remaining path to two
    /// equal halves. Any different actual steering still requires a new solve.
    pub(super) fn after_travel(self, distance: f64) -> Option<Self> {
        let first = self.variables[2] * self.first_fraction - distance;
        let remaining = self.variables[2] - distance;
        if first <= 1e-6 || remaining <= 1e-6 {
            return None;
        }
        Some(Self {
            variables: [self.variables[0], self.variables[1], remaining],
            first_fraction: first / remaining,
        })
    }
}

pub(super) struct TwoArcConnection {
    pub(super) points: Vec<Point2>,
    pub(super) endpoint: Pose2,
    pub(super) end_curvature: f64,
    pub(super) seed: TwoArcSeed,
}
impl TwoArcConnection {
    pub(super) fn into_path(self) -> (Vec<Point2>, Pose2, f64) {
        (self.points, self.endpoint, self.end_curvature)
    }
}

#[allow(clippy::too_many_arguments)]
pub(super) fn two_arc(
    start: Pose2,
    initial_curvature: f64,
    goal: Point2,
    goal_heading: f64,
    config: &NavigationConfig,
    grid: &Grid,
    budget: &TerminalBudget,
    seed: Option<TwoArcSeed>,
) -> Option<TwoArcConnection> {
    solve_two_arc(
        start,
        initial_curvature,
        goal,
        goal_heading,
        config,
        grid,
        budget,
        None,
    )
    .or_else(|| {
        seed.and_then(|seed| {
            continue_two_arc(
                start,
                initial_curvature,
                goal,
                goal_heading,
                config,
                grid,
                budget,
                seed,
            )
        })
    })
}

#[allow(clippy::too_many_arguments)]
pub(super) fn continue_two_arc(
    start: Pose2,
    initial_curvature: f64,
    goal: Point2,
    goal_heading: f64,
    config: &NavigationConfig,
    grid: &Grid,
    budget: &TerminalBudget,
    seed: TwoArcSeed,
) -> Option<TwoArcConnection> {
    let mut work = budget.snapshot();
    work.continued_seed_attempts += 1;
    budget.0.set(work);
    let connection = solve_two_arc(
        start,
        initial_curvature,
        goal,
        goal_heading,
        config,
        grid,
        budget,
        Some(seed),
    )?;
    let mut work = budget.snapshot();
    work.continued_seed_accepted += 1;
    budget.0.set(work);
    Some(connection)
}

#[allow(clippy::too_many_arguments)]
fn terminal_primitive(
    start: Pose2,
    initial_curvature: f64,
    target_curvature: f64,
    length: f64,
    config: &NavigationConfig,
    grid: &Grid,
    budget: &TerminalBudget,
) -> Option<(Vec<Point2>, Pose2, f64)> {
    let samples = (length / (grid.resolution / 3.0).min(0.025))
        .ceil()
        .max(1.0) as usize;
    budget.charge(0, 0, samples)?;
    let result = car_primitive(
        start,
        initial_curvature,
        target_curvature,
        length,
        config,
        grid,
    );
    if result.is_none() {
        let mut work = budget.snapshot();
        work.primitive_grid_rejections += 1;
        budget.0.set(work);
    }
    result
}

/// A short two-part approach with three bounded variables: the two target
/// curvatures and total length. Cold starts use equal parts; a continued seed
/// keeps the two remaining segment lengths, without a split-ratio search.
/// Every attempt uses the existing ramp/grid checks and the shared work ledger.
#[allow(clippy::too_many_arguments)]
fn solve_two_arc(
    start: Pose2,
    initial_curvature: f64,
    goal: Point2,
    goal_heading: f64,
    config: &NavigationConfig,
    grid: &Grid,
    budget: &TerminalBudget,
    seed: Option<TwoArcSeed>,
) -> Option<TwoArcConnection> {
    let local = start.world_to_body(goal);
    let distance = start.point().distance(goal);
    // Two lookahead distances cover a near-goal approach; the absolute cap
    // keeps even the largest valid configuration's additional work bounded.
    if local.x_m <= 0.0 || !(1e-6..=(2.0 * config.lookahead_m).min(2.0)).contains(&distance) {
        let mut work = budget.snapshot();
        work.domain_rejections += 1;
        budget.0.set(work);
        return None;
    }
    let heading_change = angle_error(goal_heading, start.yaw_rad);
    // Small-angle geometry supplies only a seed. Acceptance below checks the
    // actual curved endpoint, never this approximate solution.
    let offset = 4.0 * local.y_m / distance.powi(2);
    budget.solver()?;
    let fraction = seed.map_or(0.5, |seed| seed.first_fraction);
    let mut variables = seed.map_or(
        [
            (offset - heading_change / distance)
                .clamp(-config.max_curvature_per_m, config.max_curvature_per_m),
            (3.0 * heading_change / distance - offset)
                .clamp(-config.max_curvature_per_m, config.max_curvature_per_m),
            distance,
        ],
        |seed| seed.variables,
    );
    let max_length = distance * std::f64::consts::FRAC_PI_2;
    variables[2] = variables[2].clamp(distance, max_length);
    let position_tolerance = (config.goal_tolerance_m * 0.25).min(0.0001);
    let sample = |parameters: [f64; 3]| {
        let (mut points, middle, middle_curvature) = terminal_primitive(
            start,
            initial_curvature,
            parameters[0],
            parameters[2] * fraction,
            config,
            grid,
            budget,
        )?;
        let (last, endpoint, curvature) = terminal_primitive(
            middle,
            middle_curvature,
            parameters[1],
            parameters[2] * (1.0 - fraction),
            config,
            grid,
            budget,
        )?;
        points.extend(last);
        Some((points, endpoint, curvature))
    };
    for _ in 0..8 {
        budget.iteration()?;
        let result = sample(variables)?;
        let error = [
            result.1.x_m - goal.x_m,
            result.1.y_m - goal.y_m,
            angle_error(result.1.yaw_rad, goal_heading),
        ];
        if error[0].hypot(error[1]) < position_tolerance
            && error[2].abs() <= config.goal_heading_tolerance_rad * 0.5
        {
            return Some(TwoArcConnection {
                points: result.0,
                endpoint: result.1,
                end_curvature: result.2,
                seed: TwoArcSeed {
                    variables,
                    first_fraction: fraction,
                },
            });
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
    let mut work = budget.snapshot();
    work.solver_iteration_exhaustions += 1;
    budget.0.set(work);
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
#[allow(clippy::too_many_arguments)]
pub(super) fn single_arc(
    start: Pose2,
    initial_curvature: f64,
    goal: Point2,
    goal_heading: Option<f64>,
    config: &NavigationConfig,
    grid: &Grid,
    budget: &TerminalBudget,
) -> Option<(Vec<Point2>, Pose2, f64)> {
    let local = start.world_to_body(goal);
    let distance = start.point().distance(goal);
    if local.x_m <= 0.0 || !(1e-9..0.35).contains(&distance) {
        let mut work = budget.snapshot();
        work.domain_rejections += 1;
        budget.0.set(work);
        return None;
    }
    budget.solver()?;
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
        budget.iteration()?;
        let result = terminal_primitive(
            start,
            initial_curvature,
            curvature,
            length,
            config,
            grid,
            budget,
        )?;
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
        let (_, turn, _) = terminal_primitive(
            start,
            initial_curvature,
            curvature + delta_k,
            length,
            config,
            grid,
            budget,
        )?;
        let (_, travel, _) = terminal_primitive(
            start,
            initial_curvature,
            curvature,
            length + delta_s,
            config,
            grid,
            budget,
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
    let mut work = budget.snapshot();
    work.solver_iteration_exhaustions += 1;
    budget.0.set(work);
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::autonomy::{HalfPlane, ObstacleDisc, PoseEstimate};
    use crate::motion_transition::{MotionTransition, project_motion};

    fn original_scene(
        direction: f64,
    ) -> (
        NavigationConfig,
        PoseEstimate,
        Vec<ObstacleDisc>,
        Point2,
        f64,
    ) {
        let raw: serde_json::Value =
            serde_json::from_str(include_str!("../../../../config/competition-sim.json")).unwrap();
        let cfg: NavigationConfig =
            serde_json::from_value(raw["autonomy"]["navigation"].clone()).unwrap();
        let raw: serde_json::Value = serde_json::from_str(include_str!(
            "../../../../docs/motion-v5-original-input-window.json"
        ))
        .unwrap();
        let frame = raw["frames"]
            .as_array()
            .unwrap()
            .iter()
            .find(|frame| frame["at"] == 42200)
            .unwrap();
        let mut estimate: PoseEstimate =
            serde_json::from_value(frame["snapshot"]["pose"].clone()).unwrap();
        estimate.pose.y_m = 2.5 + direction * (estimate.pose.y_m - 2.5);
        estimate.pose.yaw_rad *= direction;
        estimate.yaw_rate_radps *= direction;
        let mut obstacles: Vec<ObstacleDisc> =
            serde_json::from_value(frame["world_obstacles"].clone()).unwrap();
        assert_eq!(obstacles.len(), 360);
        for obstacle in &mut obstacles {
            obstacle.center.y_m = 2.5 + direction * (obstacle.center.y_m - 2.5);
        }
        (
            cfg,
            estimate,
            obstacles,
            Point2 {
                x_m: 5.55,
                y_m: 2.5,
            },
            direction * frame["actual_curvature_per_m"].as_f64().unwrap(),
        )
    }

    #[test]
    fn original_lqr_42200_remaining_segments_restore_a_grid_checked_candidate_on_both_sides() {
        for direction in [-1.0, 1.0] {
            let (cfg, estimate, obstacles, goal, k) = original_scene(direction);
            let boundary = HalfPlane::new(goal, 0.0, 0.45).unwrap();
            let grid = Grid::with_boundary(&cfg, &obstacles, Some(boundary));
            let budget = TerminalBudget::default();
            let current = two_arc(estimate.pose, k, goal, 0.0, &cfg, &grid, &budget, None).unwrap();
            let mut next_candidates = Vec::new();
            // Reproduce the full original 44-entry cold round before recovery,
            // sharing its accounting. The original source has no collision rejection here.
            for i in 0..=21 {
                let target_k = direction
                    * (0.47755674930465936 - 0.4
                        + if i == 21 { 0.0 } else { 0.8 * i as f64 / 20.0 });
                for target_v in [0.18, 0.12] {
                    let next = project_motion(
                        estimate.pose,
                        MotionTransition {
                            initial_speed_mps: 0.18,
                            target_speed_mps: target_v,
                            initial_curvature_per_m: k,
                            target_curvature_per_m: target_k,
                            max_accel_mps2: 0.4,
                            max_decel_mps2: 0.6,
                            max_curvature_rate_per_s: 4.0,
                        },
                        0.1,
                    )
                    .unwrap();
                    assert!(
                        two_arc(
                            next.pose,
                            next.curvature_per_m,
                            goal,
                            0.0,
                            &cfg,
                            &grid,
                            &budget,
                            None
                        )
                        .is_none()
                    );
                    next_candidates.push(next);
                }
            }
            let cold = budget.snapshot();
            assert!(!cold.budget_exhausted, "cold work: {cold:?}");
            assert_eq!(cold.solver_iteration_exhaustions, 44);
            assert_eq!(cold.iterations, 3 + 44 * 8);
            let mut accepted = 0;
            for next in next_candidates {
                if let Some(connection) = continue_two_arc(
                    next.pose,
                    next.curvature_per_m,
                    goal,
                    0.0,
                    &cfg,
                    &grid,
                    &budget,
                    current.seed.after_travel(next.distance_m).unwrap(),
                ) {
                    accepted += 1;
                    assert!(connection.endpoint.point().distance(goal) < 0.0001);
                    assert!(
                        angle_error(connection.endpoint.yaw_rad, 0.0).abs()
                            <= cfg.goal_heading_tolerance_rad * 0.5
                    );
                    assert!(
                        connection.seed.variables[..2]
                            .iter()
                            .all(|k| k.abs() <= cfg.max_curvature_per_m)
                    );
                    assert!(connection.seed.variables[2] < 1.0);
                }
            }
            let work = budget.snapshot();
            println!("direction={direction}, cold={cold:?}, final={work:?}, recovered={accepted}");
            assert!(accepted > 0);
            assert!(work.iterations <= work.iteration_limit);
            assert!(work.primitive_samples <= work.sample_limit);
            assert!(work.solver_attempts <= work.solver_limit);
            assert_eq!(work.primitive_grid_rejections, 0);
            // A changed scan occupying the entire goal must invalidate the seed.
            let blocked = Grid::with_boundary(
                &cfg,
                &[ObstacleDisc {
                    center: goal,
                    radius_m: 0.2,
                }],
                Some(boundary),
            );
            let blocked_budget = TerminalBudget::default();
            assert!(
                continue_two_arc(
                    estimate.pose,
                    k,
                    goal,
                    0.0,
                    &cfg,
                    &blocked,
                    &blocked_budget,
                    current.seed
                )
                .is_none()
            );
            assert!(blocked_budget.snapshot().primitive_grid_rejections > 0);
            assert!(!blocked_budget.snapshot().budget_exhausted);
        }
    }

    #[test]
    fn every_terminal_attempt_shares_solver_iteration_and_sample_limits() {
        let (cfg, estimate, _, goal, k) = original_scene(1.0);
        let grid = Grid::new(&cfg, &[]);
        for axis in 0..3 {
            let budget = TerminalBudget::default();
            let mut limits = budget.snapshot();
            match axis {
                0 => limits.solver_limit = 1,
                1 => limits.iteration_limit = 1,
                _ => limits.sample_limit = 1,
            }
            budget.0.set(limits);
            for _ in 0..3 {
                let _ = two_arc(estimate.pose, k, goal, 0.0, &cfg, &grid, &budget, None);
            }
            let used = budget.snapshot();
            assert!(used.budget_exhausted);
            assert!(used.solver_attempts <= used.solver_limit);
            assert!(used.iterations <= used.iteration_limit);
            assert!(used.primitive_samples <= used.sample_limit);
        }
    }
}
