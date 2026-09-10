//! Bounded, forward-only path tracking in physical curvature units. No I/O.
//!
//! Pure pursuit retains the navigator's existing lookahead geometry (Coulter,
//! CMU-RI-TR-92-01). The optional LQR is an experimental continuous-time,
//! small-error model: `e_y' = v e_heading`, `e_heading' = v delta_curvature`.
//! For positive constant speed, `Q = diag(q_y, q_heading)` and `R = r`, the
//! stabilizing gains are `sqrt(q_y/r)` and `sqrt(q_heading/r + 2*sqrt(q_y/r))`.
//! The common positive speed factor cancels in these gains. This is not a
//! discrete-time, dynamic-tire, actuator-delay, or constrained-MPC model.
//! This project applies the straight-reference approximation with geometric
//! curvature feedforward on bends. The exact curved Frenet linearization also
//! contains `-v * k_ref^2 * e_y`, which this model omits; its gains do not prove
//! stability on every bend or under speed variation and actuator saturation.
//! See <https://underactuated.mit.edu/lqr.html> for continuous-time LQR.
//!
//! Neither tracker owns speed, collision checking, steering slew, or stopping.
//! The caller must apply those checks to every proposed curvature. Curvature is
//! not a steering angle or PWM; no wheelbase or actuator calibration is assumed.
use crate::ValidationError;
use crate::autonomy::{Point2, Pose2};
use serde::{Deserialize, Serialize};

/// Input validation and all geometry scans have this explicit point bound.
pub const MAX_PATH_POINTS: usize = 100_000;
const MIN_SEGMENT_M: f64 = 1e-6;

#[derive(Clone, Copy, Debug)]
pub struct TrackInput<'a> {
    pub pose: Pose2,
    pub speed_mps: f64,
    /// Original cached route, in the same frame as pose. Do not prepend pose.
    pub path: &'a [Point2],
    /// Monotonically maintained nearest route vertex, not a segment index.
    pub progress: usize,
    pub lookahead_m: f64,
    pub max_curvature_per_m: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TrackingCommand {
    /// The common pure-pursuit lookahead target, also useful for local scoring.
    pub target: Point2,
    pub curvature_per_m: f64,
    pub diagnostics: TrackingDiagnostics,
}

/// The algorithm that produced this command, including low-speed PP fallback.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum TrackingMode {
    PurePursuit,
    Lqr,
}

/// Values already computed for this command; diagnostics do not run extra
/// geometry or add validation. PP does not compute a Frenet reference, so its
/// error/reference fields are None, including when selected by LQR fallback.
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct TrackingDiagnostics {
    pub mode: TrackingMode,
    pub lateral_error_m: Option<f64>,
    pub heading_error_rad: Option<f64>,
    pub reference_curvature_per_m: Option<f64>,
}

pub trait PathTracker {
    fn track(&self, input: TrackInput<'_>) -> Result<TrackingCommand, ValidationError>;
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum TrackingConfig {
    #[default]
    PurePursuit,
    /// Experimental weights and linearization envelope, never a vehicle tune.
    Lqr {
        q_lateral: f64,
        q_heading: f64,
        r_curvature: f64,
        min_speed_mps: f64,
        max_heading_error_rad: f64,
        max_lateral_error_m: f64,
    },
}

impl<'de> Deserialize<'de> for TrackingConfig {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        // serde's internally tagged unit variant accepts trailing object fields.
        // Use an empty struct variant on the wire so PP also rejects misspellings.
        #[derive(Deserialize)]
        #[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
        enum StrictConfig {
            PurePursuit {},
            Lqr {
                q_lateral: f64,
                q_heading: f64,
                r_curvature: f64,
                min_speed_mps: f64,
                max_heading_error_rad: f64,
                max_lateral_error_m: f64,
            },
        }
        Ok(match StrictConfig::deserialize(deserializer)? {
            StrictConfig::PurePursuit {} => Self::PurePursuit,
            StrictConfig::Lqr {
                q_lateral,
                q_heading,
                r_curvature,
                min_speed_mps,
                max_heading_error_rad,
                max_lateral_error_m,
            } => Self::Lqr {
                q_lateral,
                q_heading,
                r_curvature,
                min_speed_mps,
                max_heading_error_rad,
                max_lateral_error_m,
            },
        })
    }
}

impl TrackingConfig {
    pub fn validate(&self) -> Result<(), ValidationError> {
        match self {
            Self::PurePursuit => Ok(()),
            Self::Lqr { .. } => LqrTracker::new(*self).map(|_| ()),
        }
    }
}

impl PathTracker for TrackingConfig {
    fn track(&self, input: TrackInput<'_>) -> Result<TrackingCommand, ValidationError> {
        match self {
            Self::PurePursuit => PurePursuitTracker.track(input),
            Self::Lqr { .. } => LqrTracker::new(*self)?.track(input),
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct PurePursuitTracker;

impl PathTracker for PurePursuitTracker {
    fn track(&self, input: TrackInput<'_>) -> Result<TrackingCommand, ValidationError> {
        validate_input(input)?;
        pure_pursuit(input)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct LqrTracker {
    lateral_gain: f64,
    heading_gain: f64,
    min_speed_mps: f64,
    max_heading_error_rad: f64,
    max_lateral_error_m: f64,
}

impl LqrTracker {
    pub fn new(config: TrackingConfig) -> Result<Self, ValidationError> {
        let TrackingConfig::Lqr {
            q_lateral,
            q_heading,
            r_curvature,
            min_speed_mps,
            max_heading_error_rad,
            max_lateral_error_m,
        } = config
        else {
            return Err(error("LQR tracker requires LQR configuration"));
        };
        if [q_lateral, q_heading, r_curvature]
            .iter()
            .any(|v| !v.is_finite() || *v <= 0.0)
            || !min_speed_mps.is_finite()
            || !(1e-6..=3.0).contains(&min_speed_mps)
            || !max_heading_error_rad.is_finite()
            || !(1e-6..=0.7).contains(&max_heading_error_rad)
            || !max_lateral_error_m.is_finite()
            || !(1e-6..=2.0).contains(&max_lateral_error_m)
        {
            return Err(error("invalid LQR weights or small-error envelope"));
        }
        let lateral_gain = (q_lateral / r_curvature).sqrt();
        let heading_gain = (q_heading / r_curvature + 2.0 * lateral_gain).sqrt();
        if [lateral_gain, heading_gain]
            .iter()
            .any(|v| !v.is_finite() || *v <= 0.0)
        {
            return Err(error("LQR gain overflow or underflow"));
        }
        Ok(Self {
            lateral_gain,
            heading_gain,
            min_speed_mps,
            max_heading_error_rad,
            max_lateral_error_m,
        })
    }
}

impl PathTracker for LqrTracker {
    fn track(&self, input: TrackInput<'_>) -> Result<TrackingCommand, ValidationError> {
        validate_input(input)?;
        // Always apply the forward-target contract, also on an LQR step.
        let mut command = pure_pursuit(input)?;
        if input.speed_mps < self.min_speed_mps {
            // At standstill this continuous forward model is uncontrollable.
            // PP proposes curvature only; it does not rotate a stationary car.
            return Ok(command);
        }
        let reference = local_reference(input)?;
        let heading_error = angle_difference(input.pose.yaw_rad, reference.heading);
        let (sin, cos) = reference.heading.sin_cos();
        let lateral_error = -sin * (input.pose.x_m - reference.point.x_m)
            + cos * (input.pose.y_m - reference.point.y_m);
        if heading_error.abs() > self.max_heading_error_rad
            || lateral_error.abs() > self.max_lateral_error_m
            || (reference.curvature * lateral_error).abs() > 0.25
        {
            return Err(error(&format!(
                "LQR reference outside the small-error envelope: lateral_error_m={lateral_error}, \
                 heading_error_rad={heading_error}, reference_curvature_per_m={}, \
                 max_lateral_error_m={}, max_heading_error_rad={}, max_abs_curvature_lateral=0.25",
                reference.curvature, self.max_lateral_error_m, self.max_heading_error_rad
            )));
        }
        let curvature = reference.curvature
            - self.lateral_gain * lateral_error
            - self.heading_gain * heading_error;
        if !curvature.is_finite() {
            return Err(error("LQR curvature is not finite"));
        }
        command.curvature_per_m =
            curvature.clamp(-input.max_curvature_per_m, input.max_curvature_per_m);
        command.diagnostics = TrackingDiagnostics {
            mode: TrackingMode::Lqr,
            lateral_error_m: Some(lateral_error),
            heading_error_rad: Some(heading_error),
            reference_curvature_per_m: Some(reference.curvature),
        };
        Ok(command)
    }
}

fn validate_input(input: TrackInput<'_>) -> Result<(), ValidationError> {
    if !(2..=MAX_PATH_POINTS).contains(&input.path.len())
        || input.progress >= input.path.len()
        || !input.pose.valid()
        || input.pose.x_m.abs() > 1_000_000.0
        || input.pose.y_m.abs() > 1_000_000.0
        || !input.speed_mps.is_finite()
        || !(0.0..=3.0).contains(&input.speed_mps)
        || !input.lookahead_m.is_finite()
        || !(1e-6..=5.0).contains(&input.lookahead_m)
        || !input.max_curvature_per_m.is_finite()
        || !(1e-6..=10.0).contains(&input.max_curvature_per_m)
        || input.path.iter().any(|point| {
            !point.valid() || point.x_m.abs() > 1_000_000.0 || point.y_m.abs() > 1_000_000.0
        })
        || !input
            .path
            .windows(2)
            .any(|pair| pair[0].distance(pair[1]) > MIN_SEGMENT_M)
    {
        return Err(error("invalid, excessive, or degenerate tracking input"));
    }
    Ok(())
}

fn pure_pursuit(input: TrackInput<'_>) -> Result<TrackingCommand, ValidationError> {
    // Iterate the exact old navigator chain without allocating its temporary Vec:
    // [pose.point(), route[(progress + 1).min(route.len() - 1)..]].
    let mut target = *input.path.last().expect("validated route length");
    let mut previous = input.pose.point();
    let mut remaining = input.lookahead_m;
    for point in &input.path[(input.progress + 1).min(input.path.len() - 1)..] {
        let length = previous.distance(*point);
        if length > remaining {
            let ratio = remaining / length;
            target = Point2 {
                x_m: previous.x_m + ratio * (point.x_m - previous.x_m),
                y_m: previous.y_m + ratio * (point.y_m - previous.y_m),
            };
            break;
        }
        remaining -= length;
        previous = *point;
    }
    let body_target = input.pose.world_to_body(target);
    let squared = body_target.x_m.powi(2) + body_target.y_m.powi(2);
    if !squared.is_finite() || squared <= 1e-12 || body_target.x_m <= 0.0 {
        return Err(error("tracking target requires reverse or turn in place"));
    }
    let curvature = 2.0 * body_target.y_m / squared;
    if !curvature.is_finite() {
        return Err(error("pure-pursuit curvature is not finite"));
    }
    Ok(TrackingCommand {
        target,
        curvature_per_m: curvature.clamp(-input.max_curvature_per_m, input.max_curvature_per_m),
        diagnostics: TrackingDiagnostics {
            mode: TrackingMode::PurePursuit,
            lateral_error_m: None,
            heading_error_rad: None,
            reference_curvature_per_m: None,
        },
    })
}

#[derive(Clone, Copy)]
struct Projection {
    first: usize,
    last: usize,
    fraction: f64,
    point: Point2,
    distance: f64,
}

struct Reference {
    point: Point2,
    heading: f64,
    curvature: f64,
}

fn local_reference(input: TrackInput<'_>) -> Result<Reference, ValidationError> {
    let path = input.path;
    let vertex = input.progress;
    let incoming = previous_distinct(path, vertex).map(|first| project(input, first, vertex));
    let outgoing = next_distinct(path, vertex).map(|last| project(input, vertex, last));
    // Only the two segments incident on the supplied progress vertex are eligible.
    // Searching the whole route for a nearest segment would jump at crossings.
    let projection = match (incoming, outgoing) {
        (Some(before), Some(after)) if before.distance < after.distance => before,
        (_, Some(after)) => after,
        (Some(before), None) => before,
        (None, None) => return Err(error("LQR local path is degenerate")),
    };
    let first = path[projection.first];
    let last = path[projection.last];
    let curvature = if let Some(before) = previous_distinct(path, projection.first) {
        signed_curvature(path[before], first, last)?
    } else if let Some(after) = next_distinct(path, projection.last) {
        signed_curvature(first, last, path[after])?
    } else {
        0.0
    };
    // A local three-point circle supplies feedforward and endpoint tangents.
    // On a circle, chord heading differs from the starting tangent by half the
    // arc angle. Interpolation recovers both vertex tangents without using pose
    // as a fake path point. Between vertices the reference remains a polyline.
    let half_turn = (curvature * first.distance(last) * 0.5)
        .clamp(-1.0, 1.0)
        .asin();
    let heading = (last.y_m - first.y_m).atan2(last.x_m - first.x_m)
        + (2.0 * projection.fraction - 1.0) * half_turn;
    if !heading.is_finite() || !curvature.is_finite() {
        return Err(error("LQR reference geometry is not finite"));
    }
    Ok(Reference {
        point: projection.point,
        heading,
        curvature,
    })
}

fn previous_distinct(path: &[Point2], vertex: usize) -> Option<usize> {
    (0..vertex)
        .rev()
        .find(|index| path[*index].distance(path[vertex]) > MIN_SEGMENT_M)
}

fn next_distinct(path: &[Point2], vertex: usize) -> Option<usize> {
    ((vertex + 1)..path.len()).find(|index| path[*index].distance(path[vertex]) > MIN_SEGMENT_M)
}

fn project(input: TrackInput<'_>, first: usize, last: usize) -> Projection {
    let a = input.path[first];
    let b = input.path[last];
    let dx = b.x_m - a.x_m;
    let dy = b.y_m - a.y_m;
    let fraction = (((input.pose.x_m - a.x_m) * dx + (input.pose.y_m - a.y_m) * dy)
        / (dx * dx + dy * dy))
        .clamp(0.0, 1.0);
    let point = Point2 {
        x_m: a.x_m + fraction * dx,
        y_m: a.y_m + fraction * dy,
    };
    Projection {
        first,
        last,
        fraction,
        point,
        distance: input.pose.point().distance(point),
    }
}

fn signed_curvature(a: Point2, b: Point2, c: Point2) -> Result<f64, ValidationError> {
    let ab = a.distance(b);
    let bc = b.distance(c);
    let ac = a.distance(c);
    if ac <= MIN_SEGMENT_M {
        return Err(error("LQR path contains a local reversal"));
    }
    let cross = (b.x_m - a.x_m) * (c.y_m - a.y_m) - (b.y_m - a.y_m) * (c.x_m - a.x_m);
    let dot = (b.x_m - a.x_m) * (c.x_m - b.x_m) + (b.y_m - a.y_m) * (c.y_m - b.y_m);
    if dot <= 0.0 {
        return Err(error(
            "LQR path corner exceeds the forward small-error model",
        ));
    }
    let curvature = 2.0 * cross / (ab * bc * ac);
    if !curvature.is_finite() {
        return Err(error("LQR path curvature is not finite"));
    }
    Ok(curvature)
}

fn angle_difference(a: f64, b: f64) -> f64 {
    let delta = a - b;
    delta.sin().atan2(delta.cos())
}

fn error(message: &str) -> ValidationError {
    ValidationError(message.into())
}
