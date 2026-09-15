//! Bounded terminal geometry. A seed is a numerical hint, never permission to
//! reuse a path: every solve checks its actual ramped samples in the new grid.
use super::primitive_envelope::ErrorBound;
use super::{Grid, NavigationConfig, angle_error, car_primitive_with_error};
use crate::autonomy::{Point2, Pose2};
use serde::Serialize;
use std::cell::Cell;
use std::time::Instant;

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
    /// Bounded arrival-region fallback calls after the exact search failed.
    /// Each still charges the same solver/iteration/sample ledger.
    #[serde(skip_serializing_if = "is_zero")]
    pub arrival_region_attempts: usize,
    #[serde(skip_serializing_if = "is_zero")]
    pub arrival_region_accepted: usize,
    /// Cold candidate requests deferred to preserve the continuation allowance.
    /// This does not mean the global ledger is exhausted or geometry is impossible.
    pub cold_budget_deferrals: usize,
    pub recovery_reserved_solvers: usize,
    pub recovery_reserved_iterations: usize,
    /// Actual allowance held aside from remaining global charged-sample capacity.
    pub recovery_reserved_samples: usize,
    /// One if prior work left less than the worst-case continuation allowance.
    pub recovery_reservation_shortfalls: usize,
    /// Opt-in host wall time inside single/two-arc solvers, including fast domain
    /// rejection; wrappers do not double count. Default None never reads a clock.
    pub solver_elapsed_ns: Option<u64>,
}

fn is_zero(value: &usize) -> bool {
    *value == 0
}

#[derive(Clone, Copy, Debug, Default)]
struct WorkAllowance {
    solvers: usize,
    iterations: usize,
    samples: usize,
}

pub(super) struct TerminalBudget {
    work: Cell<TerminalWorkDiagnostics>,
    cold_ceiling: Cell<Option<WorkAllowance>>,
    cold_deferred: Cell<bool>,
    timing_enabled: Cell<bool>,
}

#[derive(Clone, Copy)]
enum EndpointPolicy {
    ExactCenter,
    ArrivalRegion,
}

impl EndpointPolicy {
    fn position_tolerance(self, config: &NavigationConfig) -> f64 {
        match self {
            Self::ExactCenter => (config.goal_tolerance_m * 0.25).min(0.0001),
            Self::ArrivalRegion => config.goal_tolerance_m * 0.5,
        }
    }
}
impl Default for TerminalBudget {
    fn default() -> Self {
        Self {
            work: Cell::new(TerminalWorkDiagnostics {
                solver_limit: 256,
                iteration_limit: 1024,
                sample_limit: 65_536,
                ..TerminalWorkDiagnostics::default()
            }),
            cold_ceiling: Cell::new(None),
            cold_deferred: Cell::new(false),
            timing_enabled: Cell::new(false),
        }
    }
}
impl TerminalBudget {
    pub(super) fn reset(&self) {
        let mut work = Self::default().snapshot();
        work.solver_elapsed_ns = self.timing_enabled.get().then_some(0);
        self.work.set(work);
        self.cold_ceiling.set(None);
        self.cold_deferred.set(false);
    }
    pub(super) fn set_timing_enabled(&self, enabled: bool) {
        self.timing_enabled.set(enabled);
    }
    pub(super) fn snapshot(&self) -> TerminalWorkDiagnostics {
        self.work.get()
    }
    fn timer(&self) -> SolverTimer<'_> {
        SolverTimer {
            budget: self,
            started: self.timing_enabled.get().then(Instant::now),
        }
    }
    /// Leave one complete eight-iteration continuation's worst-case work out of
    /// the cold round. Every iteration samples the two segments four times
    /// (one residual plus three finite differences). Total length never exceeds
    /// distance*pi/2, including its bounded perturbation. The extra two samples
    /// cover rounding the two segment lengths separately and floating rounding.
    /// Prior route/lattice work stays charged; a shortfall reserves only what
    /// remains and still permits recovery to try under the original global cap.
    pub(super) fn reserve_continuation(&self, max_distance: f64, grid: &Grid) {
        let segment_samples = (max_distance * std::f64::consts::FRAC_PI_2
            / (grid.resolution / 3.0).min(0.025))
        .ceil() as usize;
        let samples = segment_samples
            .checked_add(2)
            .and_then(|n| n.checked_mul(4))
            .and_then(|n| n.checked_mul(8))
            .unwrap_or(usize::MAX);
        let mut work = self.work.get();
        let requested = WorkAllowance {
            solvers: 1,
            iterations: 8,
            samples,
        };
        let reserved = WorkAllowance {
            solvers: requested
                .solvers
                .min(work.solver_limit - work.solver_attempts),
            iterations: requested
                .iterations
                .min(work.iteration_limit - work.iterations),
            samples: requested
                .samples
                .min(work.sample_limit - work.primitive_samples),
        };
        work.recovery_reserved_solvers = reserved.solvers;
        work.recovery_reserved_iterations = reserved.iterations;
        work.recovery_reserved_samples = reserved.samples;
        work.recovery_reservation_shortfalls = usize::from(
            reserved.solvers < requested.solvers
                || reserved.iterations < requested.iterations
                || reserved.samples < requested.samples,
        );
        self.cold_ceiling.set(Some(WorkAllowance {
            solvers: work.solver_limit - reserved.solvers,
            iterations: work.iteration_limit - reserved.iterations,
            samples: work.sample_limit - reserved.samples,
        }));
        self.work.set(work);
    }
    pub(super) fn begin_cold_candidate(&self) {
        self.cold_deferred.set(false);
    }
    pub(super) fn cold_deferred(&self) -> bool {
        self.cold_deferred.get()
    }
    pub(super) fn release_continuation(&self) {
        self.cold_ceiling.set(None);
        self.cold_deferred.set(false);
    }
    fn charge(&self, solvers: usize, iterations: usize, samples: usize) -> Option<()> {
        let mut work = self.work.get();
        if work.budget_exhausted || self.cold_deferred.get() {
            return None;
        }
        let next = work
            .solver_attempts
            .checked_add(solvers)
            .zip(work.iterations.checked_add(iterations))
            .zip(work.primitive_samples.checked_add(samples));
        if let Some(ceiling) = self.cold_ceiling.get()
            && next.is_none_or(|((s, i), p)| {
                s > ceiling.solvers || i > ceiling.iterations || p > ceiling.samples
            })
        {
            self.cold_deferred.set(true);
            work.cold_budget_deferrals += 1;
            self.work.set(work);
            return None;
        }
        let Some(((solvers, iterations), samples)) = next.filter(|((s, i), p)| {
            *s <= work.solver_limit && *i <= work.iteration_limit && *p <= work.sample_limit
        }) else {
            work.budget_exhausted = true;
            self.work.set(work);
            return None;
        };
        work.solver_attempts = solvers;
        work.iterations = iterations;
        work.primitive_samples = samples;
        self.work.set(work);
        Some(())
    }
    fn solver(&self) -> Option<()> {
        self.charge(1, 0, 0)
    }
    fn iteration(&self) -> Option<()> {
        self.charge(0, 1, 0)
    }
}

struct SolverTimer<'a> {
    budget: &'a TerminalBudget,
    started: Option<Instant>,
}
impl Drop for SolverTimer<'_> {
    fn drop(&mut self) {
        if let Some(started) = self.started {
            let mut work = self.budget.work.get();
            let elapsed = u64::try_from(started.elapsed().as_nanos()).unwrap_or(u64::MAX);
            work.solver_elapsed_ns =
                Some(work.solver_elapsed_ns.unwrap_or(0).saturating_add(elapsed));
            self.budget.work.set(work);
        }
    }
}

#[derive(Clone, Copy, Debug)]
#[cfg_attr(test, derive(serde::Deserialize, serde::Serialize))]
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
    /// True only when the returned connection was solved from the supplied seed.
    pub(super) used_seed: bool,
    pub(super) points: Vec<Point2>,
    pub(super) endpoint: Pose2,
    pub(super) end_curvature: f64,
    pub(super) seed: TwoArcSeed,
    pub(super) error: ErrorBound,
}
impl TwoArcConnection {
    pub(super) fn into_path(self) -> (Vec<Point2>, Pose2, f64) {
        (self.points, self.endpoint, self.end_curvature)
    }
}

#[allow(clippy::too_many_arguments)]
#[cfg(test)]
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
    two_arc_with_error(
        start,
        initial_curvature,
        goal,
        goal_heading,
        config,
        grid,
        budget,
        seed,
        ErrorBound::default(),
    )
}

#[allow(clippy::too_many_arguments)]
pub(super) fn two_arc_with_error(
    start: Pose2,
    initial_curvature: f64,
    goal: Point2,
    goal_heading: f64,
    config: &NavigationConfig,
    grid: &Grid,
    budget: &TerminalBudget,
    seed: Option<TwoArcSeed>,
    initial_error: ErrorBound,
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
        initial_error,
        EndpointPolicy::ExactCenter,
    )
    .or_else(|| {
        seed.and_then(|seed| {
            continue_two_arc_with_error(
                start,
                initial_curvature,
                goal,
                goal_heading,
                config,
                grid,
                budget,
                seed,
                initial_error,
            )
        })
    })
}

#[allow(clippy::too_many_arguments)]
#[cfg(test)]
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
    continue_two_arc_with_error(
        start,
        initial_curvature,
        goal,
        goal_heading,
        config,
        grid,
        budget,
        seed,
        ErrorBound::default(),
    )
}

#[allow(clippy::too_many_arguments)]
pub(super) fn continue_two_arc_with_error(
    start: Pose2,
    initial_curvature: f64,
    goal: Point2,
    goal_heading: f64,
    config: &NavigationConfig,
    grid: &Grid,
    budget: &TerminalBudget,
    seed: TwoArcSeed,
    initial_error: ErrorBound,
) -> Option<TwoArcConnection> {
    let mut work = budget.snapshot();
    work.continued_seed_attempts += 1;
    budget.work.set(work);
    let connection = solve_two_arc(
        start,
        initial_curvature,
        goal,
        goal_heading,
        config,
        grid,
        budget,
        Some(seed),
        initial_error,
        EndpointPolicy::ExactCenter,
    )?;
    let mut work = budget.snapshot();
    work.continued_seed_accepted += 1;
    budget.work.set(work);
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
    initial_error: ErrorBound,
) -> Option<(Vec<Point2>, Pose2, f64, ErrorBound)> {
    let samples = (length / (grid.resolution / 3.0).min(0.025))
        .ceil()
        .max(1.0) as usize;
    budget.charge(0, 0, samples)?;
    let result = car_primitive_with_error(
        start,
        initial_curvature,
        target_curvature,
        length,
        config,
        grid,
        initial_error,
    )
    .ok();
    if result.is_none() {
        let mut work = budget.snapshot();
        work.primitive_grid_rejections += 1;
        budget.work.set(work);
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
    initial_error: ErrorBound,
    endpoint_policy: EndpointPolicy,
) -> Option<TwoArcConnection> {
    let _timer = budget.timer();
    let local = start.world_to_body(goal);
    let distance = start.point().distance(goal);
    // Two lookahead distances cover a near-goal approach; the absolute cap
    // keeps even the largest valid configuration's additional work bounded.
    if local.x_m <= 0.0 || !(1e-6..=(2.0 * config.lookahead_m).min(2.0)).contains(&distance) {
        let mut work = budget.snapshot();
        work.domain_rejections += 1;
        budget.work.set(work);
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
    let position_tolerance = endpoint_policy.position_tolerance(config);
    let sample = |parameters: [f64; 3]| {
        let (mut points, middle, middle_curvature, middle_error) = terminal_primitive(
            start,
            initial_curvature,
            parameters[0],
            parameters[2] * fraction,
            config,
            grid,
            budget,
            initial_error,
        )?;
        let (last, endpoint, curvature, end_error) = terminal_primitive(
            middle,
            middle_curvature,
            parameters[1],
            parameters[2] * (1.0 - fraction),
            config,
            grid,
            budget,
            middle_error,
        )?;
        points.extend(last);
        Some((points, endpoint, curvature, end_error))
    };
    for _ in 0..8 {
        budget.iteration()?;
        let result = sample(variables)?;
        let error = [
            result.1.x_m - goal.x_m,
            result.1.y_m - goal.y_m,
            angle_error(result.1.yaw_rad, goal_heading),
        ];
        if error[0].hypot(error[1]) + result.3.position_m < position_tolerance
            && error[2].abs() + result.3.heading_rad <= config.goal_heading_tolerance_rad * 0.5
        {
            return Some(TwoArcConnection {
                used_seed: seed.is_some(),
                points: result.0,
                endpoint: result.1,
                end_curvature: result.2,
                seed: TwoArcSeed {
                    variables,
                    first_fraction: fraction,
                },
                error: result.3,
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
            let (_, endpoint, _, _) = sample(perturbed)?;
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
    budget.work.set(work);
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
    single_arc_with_error(
        start,
        initial_curvature,
        goal,
        goal_heading,
        config,
        grid,
        budget,
        ErrorBound::default(),
    )
    .map(|(points, pose, curvature, _)| (points, pose, curvature))
}

#[allow(clippy::too_many_arguments)]
pub(super) fn single_arc_with_error(
    start: Pose2,
    initial_curvature: f64,
    goal: Point2,
    goal_heading: Option<f64>,
    config: &NavigationConfig,
    grid: &Grid,
    budget: &TerminalBudget,
    initial_error: ErrorBound,
) -> Option<(Vec<Point2>, Pose2, f64, ErrorBound)> {
    solve_single_arc(
        start,
        initial_curvature,
        goal,
        goal_heading,
        config,
        grid,
        budget,
        initial_error,
        EndpointPolicy::ExactCenter,
    )
}

/// The exact shortcuts and bounded searches retain priority. This separate
/// fallback accepts only the actual integrated endpoint strictly inside half
/// the original position tolerance and within half the original heading
/// tolerance, including the full accumulated model error. No lattice nodes or
/// extra ledger are allocated, and the endpoint is never replaced by the goal.
#[allow(clippy::too_many_arguments)]
pub(super) fn oriented_arrival_region(
    start: Pose2,
    initial_curvature: f64,
    goal: Point2,
    goal_heading: f64,
    config: &NavigationConfig,
    grid: &Grid,
    budget: &TerminalBudget,
    initial_error: ErrorBound,
) -> Option<(Vec<Point2>, Pose2, f64, ErrorBound)> {
    if budget.snapshot().budget_exhausted {
        return None;
    }
    let mut work = budget.snapshot();
    work.arrival_region_attempts += 1;
    budget.work.set(work);
    let result = solve_single_arc(
        start,
        initial_curvature,
        goal,
        Some(goal_heading),
        config,
        grid,
        budget,
        initial_error,
        EndpointPolicy::ArrivalRegion,
    )
    .or_else(|| {
        solve_two_arc(
            start,
            initial_curvature,
            goal,
            goal_heading,
            config,
            grid,
            budget,
            None,
            initial_error,
            EndpointPolicy::ArrivalRegion,
        )
        .map(|connection| {
            (
                connection.points,
                connection.endpoint,
                connection.end_curvature,
                connection.error,
            )
        })
    });
    if result.is_some() {
        let mut work = budget.snapshot();
        work.arrival_region_accepted += 1;
        budget.work.set(work);
    }
    result
}

#[allow(clippy::too_many_arguments)]
fn solve_single_arc(
    start: Pose2,
    initial_curvature: f64,
    goal: Point2,
    goal_heading: Option<f64>,
    config: &NavigationConfig,
    grid: &Grid,
    budget: &TerminalBudget,
    initial_error: ErrorBound,
    endpoint_policy: EndpointPolicy,
) -> Option<(Vec<Point2>, Pose2, f64, ErrorBound)> {
    let _timer = budget.timer();
    let local = start.world_to_body(goal);
    let distance = start.point().distance(goal);
    if local.x_m <= 0.0 || !(1e-9..0.35).contains(&distance) {
        let mut work = budget.snapshot();
        work.domain_rejections += 1;
        budget.work.set(work);
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
    let position_tolerance = endpoint_policy.position_tolerance(config);
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
            initial_error,
        )?;
        let error_x = result.1.x_m - goal.x_m;
        let error_y = result.1.y_m - goal.y_m;
        if error_x.hypot(error_y) + result.3.position_m < position_tolerance {
            return goal_heading
                .is_none_or(|yaw| {
                    angle_error(result.1.yaw_rad, yaw).abs() + result.3.heading_rad
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
        let (_, turn, _, _) = terminal_primitive(
            start,
            initial_curvature,
            curvature + delta_k,
            length,
            config,
            grid,
            budget,
            initial_error,
        )?;
        let (_, travel, _, _) = terminal_primitive(
            start,
            initial_curvature,
            curvature,
            length + delta_s,
            config,
            grid,
            budget,
            initial_error,
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
    budget.work.set(work);
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::autonomy::{HalfPlane, ObstacleDisc, PoseEstimate};
    use crate::motion_transition::{MotionTransition, project_motion};

    fn arrival_region_scene() -> (NavigationConfig, Pose2, Point2) {
        let raw: serde_json::Value =
            serde_json::from_str(include_str!("../../../../config/competition-sim.json")).unwrap();
        let mut config: NavigationConfig =
            serde_json::from_value(raw["autonomy"]["navigation"].clone()).unwrap();
        config.bounds.max_x_m = 8.0;
        config.bounds.max_y_m = 6.0;
        // Actual stopped state from the pre-fix large_8x6 async PP result.
        // These geometry tests use an empty scene, not a worker-input replay.
        let start = Pose2 {
            x_m: 6.614652446,
            y_m: 5.384546628,
            yaw_rad: 0.013694076,
        };
        (
            config,
            start,
            Point2 {
                x_m: 6.68,
                y_m: 5.4,
            },
        )
    }

    #[test]
    fn arrival_region_certifies_actual_short_endpoint_without_forcing_goal_center() {
        let (config, mut start, goal) = arrival_region_scene();
        for sign in [-1.0, 1.0] {
            start.y_m = goal.y_m + sign * (5.384546628 - goal.y_m);
            start.yaw_rad = sign * 0.013694076;
            let boundary = HalfPlane::new(goal, 0.0, 0.52).unwrap();
            let grid = Grid::with_boundary(&config, &[], Some(boundary));
            for continuous in [false, true] {
                if continuous {
                    grid.enable_recovery();
                }
                let budget = TerminalBudget::default();
                assert!(single_arc(start, 0.0, goal, Some(0.0), &config, &grid, &budget).is_none());
                assert!(two_arc(start, 0.0, goal, 0.0, &config, &grid, &budget, None).is_none());
                let before = budget.snapshot();
                let (points, end, curvature, error) = oriented_arrival_region(
                    start,
                    0.0,
                    goal,
                    0.0,
                    &config,
                    &grid,
                    &budget,
                    ErrorBound::default(),
                )
                .unwrap();
                assert_eq!(points.last(), Some(&end.point()));
                assert!(end.point().distance(start.point()) > 0.04);
                assert!(end.point().distance(goal) > 0.0001);
                assert!(
                    end.point().distance(goal) + error.position_m < config.goal_tolerance_m * 0.5
                );
                assert!(
                    angle_error(end.yaw_rad, 0.0).abs() + error.heading_rad
                        <= config.goal_heading_tolerance_rad * 0.5
                );
                assert!(curvature.abs() <= config.max_curvature_per_m);
                let work = budget.snapshot();
                assert_eq!(
                    (work.arrival_region_attempts, work.arrival_region_accepted),
                    (1, 1)
                );
                assert!(work.solver_attempts > before.solver_attempts);
                assert!(work.iterations > before.iterations);
                assert!(work.primitive_samples > before.primitive_samples);
                assert!(!work.budget_exhausted);
            }
        }
    }

    #[test]
    fn arrival_region_keeps_error_collision_boundary_and_work_constraints() {
        let (config, start, goal) = arrival_region_scene();
        let grid = Grid::with_boundary(&config, &[], None);
        let budget = TerminalBudget::default();
        assert!(
            oriented_arrival_region(
                start,
                0.0,
                goal,
                0.0,
                &config,
                &grid,
                &budget,
                ErrorBound {
                    position_m: config.goal_tolerance_m * 0.5,
                    heading_rad: 0.0
                },
            )
            .is_none()
        );
        assert_eq!(budget.snapshot().arrival_region_accepted, 0);
        for boundary_only in [false, true] {
            let obstacles = if boundary_only {
                vec![]
            } else {
                vec![ObstacleDisc {
                    center: goal,
                    radius_m: 0.02,
                }]
            };
            let boundary = boundary_only.then(|| HalfPlane::new(goal, 0.0, 0.01).unwrap());
            let grid = Grid::with_boundary(&config, &obstacles, boundary);
            grid.enable_recovery();
            let budget = TerminalBudget::default();
            assert!(
                oriented_arrival_region(
                    start,
                    0.0,
                    goal,
                    0.0,
                    &config,
                    &grid,
                    &budget,
                    ErrorBound::default(),
                )
                .is_none()
            );
            assert!(budget.snapshot().primitive_grid_rejections > 0);
        }
        // Existing terminal work is charged first. The region helper cannot
        // replenish this ledger, even if a region connection would be simple.
        let budget = TerminalBudget::default();
        budget.charge(256, 0, 0).unwrap();
        assert!(
            oriented_arrival_region(
                start,
                0.0,
                goal,
                0.0,
                &config,
                &grid,
                &budget,
                ErrorBound::default(),
            )
            .is_none()
        );
        let exhausted = budget.snapshot();
        assert!(exhausted.budget_exhausted);
        assert_eq!(exhausted.solver_attempts, 256);
        assert_eq!(exhausted.primitive_samples, 0);
        assert!(
            oriented_arrival_region(
                start,
                0.0,
                goal,
                0.0,
                &config,
                &grid,
                &budget,
                ErrorBound::default(),
            )
            .is_none()
        );
        assert_eq!(
            budget.snapshot().arrival_region_attempts,
            exhausted.arrival_region_attempts
        );
    }

    #[test]
    fn arrival_region_after_node_exhaustion_uses_only_the_existing_terminal_ledger() {
        let (config, start, goal) = arrival_region_scene();
        for continuous in [false, true] {
            let nav = super::super::Navigator::new(config.clone()).unwrap();
            let grid = Grid::with_boundary(&config, &[], None);
            if continuous {
                grid.enable_recovery();
            }
            // Nodes and terminal work charged by earlier legs in this call.
            let mut earlier = grid.forward_search();
            earlier.ordinary.allocated_nodes = config.max_grid_cells;
            grid.recovery.diagnostics.set(earlier);
            nav.terminal_budget.charge(230, 900, 60_000).unwrap();
            let (points, end, _) = nav
                .kinematic_path_from(start, 0.0, goal, Some(0.0), &grid)
                .unwrap();
            assert_eq!(points.first(), Some(&start.point()));
            assert_eq!(points.last(), Some(&end.point()));
            assert!(
                end.point().distance(goal) + grid.completed_path_error().position_m
                    < config.goal_tolerance_m * 0.5
            );
            let search = grid.forward_search();
            assert_eq!(
                search.ordinary.allocated_nodes + search.recovery.allocated_nodes,
                config.max_grid_cells
            );
            assert!(search.node_budget_exhausted);
            assert_eq!(
                if continuous {
                    search.recovery.exit
                } else {
                    search.ordinary.exit
                },
                super::super::recovery::ForwardSearchExit::NodeBudget
            );
            let work = nav.terminal_budget.snapshot();
            assert_eq!(work.arrival_region_accepted, 1);
            assert!(work.solver_attempts > 230 && work.solver_attempts <= 256);
            assert!(work.iterations > 900 && work.iterations <= 1024);
            assert!(work.primitive_samples > 60_000 && work.primitive_samples <= 65_536);
            assert!(!work.budget_exhausted);
        }
    }

    #[test]
    fn exact_success_and_point_only_search_failure_do_not_enter_arrival_region_fallback() {
        let (config, start, mut goal) = arrival_region_scene();
        let nav = super::super::Navigator::new(config.clone()).unwrap();
        let grid = Grid::with_boundary(&config, &[], None);
        goal.y_m = start.y_m + (goal.x_m - start.x_m) * start.yaw_rad.tan();
        assert!(
            nav.kinematic_path_from(start, 0.0, goal, Some(start.yaw_rad), &grid)
                .is_some()
        );
        assert_eq!(nav.terminal_budget.snapshot().arrival_region_attempts, 0);
        assert_eq!(grid.forward_search().ordinary.allocated_nodes, 0);

        let nav = super::super::Navigator::new(config.clone()).unwrap();
        let grid = Grid::with_boundary(&config, &[], None);
        let mut earlier = grid.forward_search();
        earlier.ordinary.allocated_nodes = config.max_grid_cells;
        grid.recovery.diagnostics.set(earlier);
        assert!(
            nav.kinematic_path_from(start, 0.0, goal, None, &grid)
                .is_none()
        );
        assert_eq!(nav.terminal_budget.snapshot().arrival_region_attempts, 0);
        assert!(grid.forward_search().node_budget_exhausted);
    }

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
            budget.work.set(limits);
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

    #[test]
    fn reserved_work_is_shared_uncharged_and_restored_on_every_axis() {
        let (cfg, _, _, _, _) = original_scene(1.0);
        let grid = Grid::new(&cfg, &[]);
        for axis in 0..3 {
            let budget = TerminalBudget::default();
            // Accounting-only prior work, not a claimed lattice/recovery scene.
            budget.charge(11, 23, 101).unwrap();
            budget.reserve_continuation(0.95, &grid);
            let reserved = budget.snapshot();
            assert_eq!(
                reserved.primitive_samples, 101,
                "reservation is not execution"
            );
            let cold = budget.cold_ceiling.get().unwrap();
            let mut fill = [0, 0, 0];
            fill[axis] = match axis {
                0 => cold.solvers - reserved.solver_attempts,
                1 => cold.iterations - reserved.iterations,
                _ => cold.samples - reserved.primitive_samples,
            };
            budget.charge(fill[0], fill[1], fill[2]).unwrap();
            let mut one = [0, 0, 0];
            one[axis] = 1;
            budget.begin_cold_candidate();
            assert!(budget.charge(one[0], one[1], one[2]).is_none());
            assert!(budget.cold_deferred());
            assert!(!budget.snapshot().budget_exhausted);
            assert_eq!(budget.snapshot().cold_budget_deferrals, 1);
            // A second operation in the same deferred candidate stays cheap,
            // and cannot silently borrow from recovery or double count it.
            assert!(budget.charge(0, 0, 0).is_none());
            assert_eq!(budget.snapshot().cold_budget_deferrals, 1);
            budget.release_continuation();
            budget
                .charge(
                    reserved.recovery_reserved_solvers,
                    reserved.recovery_reserved_iterations,
                    reserved.recovery_reserved_samples,
                )
                .unwrap();
            assert!(budget.charge(one[0], one[1], one[2]).is_none());
            assert!(budget.snapshot().budget_exhausted);
        }
        let budget = TerminalBudget::default();
        budget.charge(1, 1, 1).unwrap();
        assert!(budget.charge(usize::MAX, usize::MAX, usize::MAX).is_none());
        assert!(budget.snapshot().budget_exhausted);
        assert_eq!(budget.snapshot().primitive_samples, 1);
    }

    #[test]
    fn reservation_shortfall_still_attempts_a_complete_connection_under_remaining_budget() {
        let (cfg, estimate, obstacles, goal, k) = original_scene(1.0);
        let grid = Grid::new(&cfg, &obstacles);
        let seed_budget = TerminalBudget::default();
        let current =
            two_arc(estimate.pose, k, goal, 0.0, &cfg, &grid, &seed_budget, None).unwrap();
        let next = project_motion(
            estimate.pose,
            MotionTransition {
                initial_speed_mps: 0.18,
                target_speed_mps: 0.18,
                initial_curvature_per_m: k,
                target_curvature_per_m: k + 0.12,
                max_accel_mps2: 0.4,
                max_decel_mps2: 0.6,
                max_curvature_rate_per_s: 4.0,
            },
            0.1,
        )
        .unwrap();
        // Deliberately near-full ledger checks shortfall policy. It does not
        // assert that a real lattice consumed this amount at the source pose.
        let budget = TerminalBudget::default();
        budget.charge(255, 1016, 65536 - 200).unwrap();
        budget.reserve_continuation(0.95, &grid);
        assert_eq!(budget.snapshot().recovery_reservation_shortfalls, 1);
        assert_eq!(budget.snapshot().recovery_reserved_samples, 200);
        budget.begin_cold_candidate();
        assert!(budget.solver().is_none());
        assert!(!budget.snapshot().budget_exhausted);
        budget.release_continuation();
        assert!(
            continue_two_arc(
                next.pose,
                next.curvature_per_m,
                goal,
                0.0,
                &cfg,
                &grid,
                &budget,
                current.seed.after_travel(next.distance_m).unwrap()
            )
            .is_some()
        );
        assert_eq!(budget.snapshot().primitive_samples, 65536 - 200 + 185);
        assert!(!budget.snapshot().budget_exhausted);
        budget.reset();
        assert_eq!(budget.snapshot().primitive_samples, 0);
        assert_eq!(budget.snapshot().recovery_reserved_samples, 0);
        assert!(budget.cold_ceiling.get().is_none());
    }

    #[test]
    fn continuation_allowance_covers_full_eight_iteration_work_at_grid_extremes() {
        let (mut cfg, estimate, _, goal, k) = original_scene(1.0);
        for resolution in [0.025, 0.05, 0.1, 1.0] {
            cfg.grid_resolution_m = resolution;
            cfg.max_grid_cells = 100000;
            cfg.validate().unwrap();
            let grid = Grid::new(&cfg, &[]);
            let budget = TerminalBudget::default();
            let seed = two_arc(estimate.pose, k, goal, 0.0, &cfg, &grid, &budget, None)
                .unwrap()
                .seed;
            budget.reset();
            let distance = estimate.pose.point().distance(goal);
            budget.reserve_continuation(distance, &grid);
            let allowance = budget.snapshot();
            budget.release_continuation();
            // Both equal and unequal splits, plus opposite initial curvature,
            // exercise different convergence/rejection paths under one bound.
            for fraction in [0.001, 0.35, 0.5, 0.65, 0.999] {
                for initial_k in [-2.0, k, 2.0] {
                    budget.reset();
                    let _ = continue_two_arc(
                        estimate.pose,
                        initial_k,
                        goal,
                        0.0,
                        &cfg,
                        &grid,
                        &budget,
                        TwoArcSeed {
                            first_fraction: fraction,
                            ..seed
                        },
                    );
                    let used = budget.snapshot();
                    assert!(used.solver_attempts <= allowance.recovery_reserved_solvers);
                    assert!(used.iterations <= allowance.recovery_reserved_iterations);
                    assert!(
                        used.primitive_samples <= allowance.recovery_reserved_samples,
                        "resolution={resolution}, fraction={fraction}, used={used:?}, allowance={allowance:?}"
                    );
                    assert!(!used.budget_exhausted);
                }
            }
        }
    }
}
