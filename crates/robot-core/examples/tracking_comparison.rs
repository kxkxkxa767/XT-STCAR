//! Deterministic tracking-only experiment; no obstacles, device access or files.
//! Run: cargo run -p xt-stcar-robot-core --example tracking_comparison --offline
//! Redirect stdout explicitly if a single JSON summary should be retained.
use serde::Serialize;
use std::{collections::VecDeque, f64::consts::PI, io::Write};
use xt_stcar_robot_core::{
    autonomy::{Point2, Pose2},
    navigation::integrate,
    tracking::{PathTracker, TrackInput, TrackingConfig},
};

const DT_S: f64 = 0.04;
const SPEED_MPS: f64 = 0.3;
const LOOKAHEAD_M: f64 = 0.55;
const MAX_CURVATURE_PER_M: f64 = 2.0;
const MAX_CURVATURE_RATE_PER_M_S: f64 = 4.0;
const MAX_STEPS: usize = 2500;
const PROGRESS_WINDOW_M: f64 = 0.25;
const END_POSITION_TOLERANCE_M: f64 = 0.08;
const END_HEADING_TOLERANCE_RAD: f64 = 0.2;

struct Scenario {
    name: &'static str,
    path: Vec<Point2>,
    initial_pose: Pose2,
}

#[derive(Serialize)]
struct Parameters {
    dt_s: f64,
    constant_speed_mps: f64,
    lookahead_m: f64,
    max_curvature_per_m: f64,
    max_actual_curvature_rate_per_m_s: f64,
    max_steps_per_run: usize,
    forward_progress_window_m: f64,
    end_position_tolerance_m: f64,
    end_heading_tolerance_rad: f64,
}

#[derive(Serialize)]
struct ResultRow {
    scenario: &'static str,
    tracker: TrackingConfig,
    assumed_steering_delay_s: f64,
    initial_pose: Pose2,
    reference_path_length_m: f64,
    reference_point_count: usize,
    reached: bool,
    failure: Option<String>,
    simulated_steps: usize,
    simulated_duration_s: f64,
    position_samples: usize,
    lateral_rms_m: f64,
    lateral_max_abs_m: f64,
    endpoint_error_m: f64,
    endpoint_heading_error_rad: f64,
    max_abs_actual_curvature_per_m: f64,
    max_actual_curvature_rate_per_m_s: f64,
    max_requested_curvature_rate_per_m_s: f64,
    final_pose: Pose2,
}

#[derive(Serialize)]
struct Report {
    schema_version: u32,
    simulation_only: bool,
    calibration_status: &'static str,
    scope: &'static str,
    time_measurement: &'static str,
    assumptions: [&'static str; 4],
    parameters: Parameters,
    run_count: usize,
    failure_count: usize,
    results: Vec<ResultRow>,
}

/// The queue models command transport delay; the rate limiter models steering
/// motion separately. Both are explicit synthetic assumptions, not measured ESC
/// or servo behaviour. At zero delay, a command affects this same simulation tick.
struct SteeringPlant {
    pending: VecDeque<f64>,
    actual_curvature_per_m: f64,
}

impl SteeringPlant {
    fn new(delay_steps: usize) -> Self {
        Self {
            pending: VecDeque::from(vec![0.0; delay_steps]),
            actual_curvature_per_m: 0.0,
        }
    }

    fn step(&mut self, requested: f64) -> f64 {
        self.pending.push_back(requested);
        let delayed = self.pending.pop_front().expect("just pushed a command");
        let change = (delayed - self.actual_curvature_per_m).clamp(
            -MAX_CURVATURE_RATE_PER_M_S * DT_S,
            MAX_CURVATURE_RATE_PER_M_S * DT_S,
        );
        self.actual_curvature_per_m += change;
        self.actual_curvature_per_m
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let lqr = TrackingConfig::Lqr {
        q_lateral: 4.0,
        q_heading: 2.0,
        r_curvature: 1.0,
        min_speed_mps: 0.03,
        max_heading_error_rad: 0.7,
        max_lateral_error_m: 0.5,
    };
    lqr.validate()?;
    let mut results = Vec::with_capacity(16);
    for scenario in scenarios() {
        for delay_steps in [0, 5] {
            for tracker in [TrackingConfig::default(), lqr] {
                results.push(run(&scenario, tracker, delay_steps));
            }
        }
    }
    let report = Report {
        schema_version: 1,
        simulation_only: true,
        calibration_status: "unverified",
        scope: "tracking layer only; no obstacle avoidance, mission, braking or hardware validation",
        time_measurement: "simulated_steps * dt_s; no wall-clock timing or target CPU performance claim",
        assumptions: [
            "Ideal, synchronous planar pose feedback; no localisation noise or drift.",
            "Forward constant speed starts at 0.3 m/s; endpoint checks end the experiment without simulating braking.",
            "Exact constant-curvature car kinematics with a bounded actual steering curvature rate; no slip or actuator identification.",
            "Each reference is tested with zero delay and an assumed 200 ms command delay using identical tracker parameters; results do not establish a preferred tracker.",
        ],
        parameters: Parameters {
            dt_s: DT_S,
            constant_speed_mps: SPEED_MPS,
            lookahead_m: LOOKAHEAD_M,
            max_curvature_per_m: MAX_CURVATURE_PER_M,
            max_actual_curvature_rate_per_m_s: MAX_CURVATURE_RATE_PER_M_S,
            max_steps_per_run: MAX_STEPS,
            forward_progress_window_m: PROGRESS_WINDOW_M,
            end_position_tolerance_m: END_POSITION_TOLERANCE_M,
            end_heading_tolerance_rad: END_HEADING_TOLERANCE_RAD,
        },
        run_count: results.len(),
        failure_count: results.iter().filter(|result| !result.reached).count(),
        results,
    };
    let mut stdout = std::io::stdout().lock();
    serde_json::to_writer_pretty(&mut stdout, &report)?;
    writeln!(stdout)?;
    Ok(())
}

fn run(scenario: &Scenario, tracker: TrackingConfig, delay_steps: usize) -> ResultRow {
    let path = &scenario.path;
    let end = *path.last().expect("generated reference has points");
    let end_heading = tangent(path[path.len() - 2], end);
    let mut pose = scenario.initial_pose;
    let mut progress = 0;
    let mut plant = SteeringPlant::new(delay_steps);
    let mut steps = 0;
    let mut samples = 0;
    let mut lateral_squared = 0.0;
    let mut lateral_max: f64 = 0.0;
    let mut actual_max: f64 = 0.0;
    let mut actual_rate_max: f64 = 0.0;
    let mut requested_rate_max: f64 = 0.0;
    let mut previous_requested = 0.0;
    let mut reached = false;
    let mut failure = None;
    for step in 0..=MAX_STEPS {
        progress = advance_progress(path, pose.point(), progress);
        let lateral = lateral_error(path, pose.point(), progress);
        lateral_squared += lateral * lateral;
        lateral_max = lateral_max.max(lateral.abs());
        samples += 1;
        if progress >= path.len() - 2
            && pose.point().distance(end) <= END_POSITION_TOLERANCE_M
            && wrap(pose.yaw_rad - end_heading).abs() <= END_HEADING_TOLERANCE_RAD
        {
            reached = true;
            break;
        }
        if step == MAX_STEPS {
            failure = Some("simulation step budget exhausted before terminal tolerances".into());
            break;
        }
        // Detect forward passage of the endpoint instead of extending a finite
        // path indefinitely until a geometric controller circles back to it.
        let along_end =
            (pose.x_m - end.x_m) * end_heading.cos() + (pose.y_m - end.y_m) * end_heading.sin();
        if progress >= path.len() - 2 && along_end > END_POSITION_TOLERANCE_M {
            failure = Some("passed endpoint without satisfying terminal tolerances".into());
            break;
        }
        let command = match tracker.track(TrackInput {
            pose,
            speed_mps: SPEED_MPS,
            path,
            progress,
            lookahead_m: LOOKAHEAD_M,
            max_curvature_per_m: MAX_CURVATURE_PER_M,
        }) {
            Ok(command) => command,
            Err(error) => {
                failure = Some(format!("tracker rejected input: {error}"));
                break;
            }
        };
        let requested = command.curvature_per_m;
        if !requested.is_finite() || requested.abs() > MAX_CURVATURE_PER_M + 1e-12 {
            failure = Some("tracker returned invalid or out-of-envelope curvature".into());
            break;
        }
        requested_rate_max = requested_rate_max.max((requested - previous_requested).abs() / DT_S);
        previous_requested = requested;
        let old_actual = plant.actual_curvature_per_m;
        let actual = plant.step(requested);
        actual_max = actual_max.max(actual.abs());
        actual_rate_max = actual_rate_max.max((actual - old_actual).abs() / DT_S);
        pose = integrate(pose, SPEED_MPS * DT_S, actual);
        steps += 1;
    }
    ResultRow {
        scenario: scenario.name,
        tracker,
        assumed_steering_delay_s: delay_steps as f64 * DT_S,
        initial_pose: scenario.initial_pose,
        reference_path_length_m: path.windows(2).map(|pair| pair[0].distance(pair[1])).sum(),
        reference_point_count: path.len(),
        reached,
        failure,
        simulated_steps: steps,
        simulated_duration_s: steps as f64 * DT_S,
        position_samples: samples,
        lateral_rms_m: (lateral_squared / samples as f64).sqrt(),
        lateral_max_abs_m: lateral_max,
        endpoint_error_m: pose.point().distance(end),
        endpoint_heading_error_rad: wrap(pose.yaw_rad - end_heading),
        max_abs_actual_curvature_per_m: actual_max,
        max_actual_curvature_rate_per_m_s: actual_rate_max,
        max_requested_curvature_rate_per_m_s: requested_rate_max,
        final_pose: pose,
    }
}

/// Monotonic nearest-vertex hint, searched only within a short forward arc
/// window. A spatially near future crossing cannot skip the intervening route.
fn advance_progress(path: &[Point2], point: Point2, previous: usize) -> usize {
    let mut best = previous;
    let mut best_distance = point.distance(path[previous]);
    let mut arc = 0.0;
    for index in previous + 1..path.len() {
        arc += path[index - 1].distance(path[index]);
        if arc > PROGRESS_WINDOW_M {
            break;
        }
        let distance = point.distance(path[index]);
        if distance < best_distance {
            best = index;
            best_distance = distance;
        }
    }
    best
}

/// Signed perpendicular error on the nearer segment incident to the current
/// vertex. End-on longitudinal error is reported separately as endpoint error.
fn lateral_error(path: &[Point2], point: Point2, progress: usize) -> f64 {
    let mut best_distance = f64::INFINITY;
    let mut best_error = 0.0;
    for index in progress.saturating_sub(1)..=(progress.min(path.len() - 2)) {
        let a = path[index];
        let b = path[index + 1];
        let dx = b.x_m - a.x_m;
        let dy = b.y_m - a.y_m;
        let length = dx.hypot(dy);
        let fraction = (((point.x_m - a.x_m) * dx + (point.y_m - a.y_m) * dy) / (length * length))
            .clamp(0.0, 1.0);
        let projection = Point2 {
            x_m: a.x_m + fraction * dx,
            y_m: a.y_m + fraction * dy,
        };
        let distance = point.distance(projection);
        if distance <= best_distance {
            best_distance = distance;
            best_error = ((point.y_m - a.y_m) * dx - (point.x_m - a.x_m) * dy) / length;
        }
    }
    best_error
}

fn scenarios() -> Vec<Scenario> {
    let straight: Vec<_> = (0..=100)
        .map(|i| Point2 {
            x_m: i as f64 * 0.05,
            y_m: 0.0,
        })
        .collect();
    let mut circle: Vec<_> = (0..=100)
        .map(|i| {
            let angle = i as f64 / 100.0 * PI / 2.0;
            Point2 {
                x_m: 2.2 * angle.sin(),
                y_m: 2.2 * (1.0 - angle.cos()),
            }
        })
        .collect();
    circle.extend((1..=25).map(|i| Point2 {
        x_m: 2.2,
        y_m: 2.2 + i as f64 * 0.04,
    }));
    let s_curve: Vec<_> = (0..=120)
        .map(|i| {
            let x = i as f64 * 0.05;
            Point2 {
                x_m: x,
                y_m: 0.55 * (2.0 * PI * x / 6.0).sin(),
            }
        })
        .collect();
    vec![
        scenario("straight_lateral_0.25_m", straight.clone(), 0.25, 0.0),
        scenario("straight_heading_0.18_rad", straight, 0.0, 0.18),
        scenario(
            "quarter_circle_r2.2_m_with_tangent_tail",
            circle,
            0.12,
            0.08,
        ),
        scenario("s_curve_amplitude_0.55_m_length_x6_m", s_curve, 0.12, 0.08),
    ]
}

fn scenario(name: &'static str, path: Vec<Point2>, lateral: f64, heading: f64) -> Scenario {
    let yaw = tangent(path[0], path[1]);
    let initial_pose = Pose2 {
        x_m: path[0].x_m - yaw.sin() * lateral,
        y_m: path[0].y_m + yaw.cos() * lateral,
        yaw_rad: yaw + heading,
    };
    Scenario {
        name,
        path,
        initial_pose,
    }
}

fn tangent(a: Point2, b: Point2) -> f64 {
    (b.y_m - a.y_m).atan2(b.x_m - a.x_m)
}

fn wrap(angle: f64) -> f64 {
    (angle + PI).rem_euclid(2.0 * PI) - PI
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn local_progress_does_not_jump_to_a_spatially_close_later_branch() {
        let mut path: Vec<_> = (0..=40)
            .map(|i| Point2 {
                x_m: i as f64 * 0.05,
                y_m: 0.0,
            })
            .collect();
        path.extend([
            Point2 { x_m: 2.0, y_m: 1.0 },
            Point2 { x_m: 0.0, y_m: 1.0 },
            Point2 {
                x_m: 0.1,
                y_m: 0.01,
            },
        ]);
        let point = Point2 {
            x_m: 0.1,
            y_m: 0.01,
        };
        assert_eq!(advance_progress(&path, point, 0), 2);
        assert_eq!(advance_progress(&path, path[0], 2), 2);
        assert!((lateral_error(&path, point, 2) - 0.01).abs() < 1e-12);
    }

    #[test]
    fn delay_and_rate_are_distinct_and_zero_delay_applies_in_current_tick() {
        let mut delayed = SteeringPlant::new(5);
        let mut immediate = SteeringPlant::new(0);
        let first = immediate.step(1.0);
        assert!((first - MAX_CURVATURE_RATE_PER_M_S * DT_S).abs() < 1e-12);
        for _ in 0..5 {
            assert_eq!(delayed.step(1.0), 0.0);
        }
        assert_eq!(delayed.step(1.0), first);
        let before_reverse = delayed.actual_curvature_per_m;
        let after_reverse = delayed.step(-1.0);
        assert!((after_reverse - before_reverse).abs() <= MAX_CURVATURE_RATE_PER_M_S * DT_S);
    }
}
