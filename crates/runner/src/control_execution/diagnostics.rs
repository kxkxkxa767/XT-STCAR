//! Fixed-size first-failure evidence; constructing it never reads a clock or logs.
use super::PlanningContext;
use crate::autonomy_replay::SensorSnapshot;
use serde::Serialize;
use xt_stcar_robot_core::Timestamp;
use xt_stcar_robot_core::autonomy::Rect;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum CertificateFailureReason {
    SourceSpeedInvalid,
    LeaseOverflow,
    PlanExpired,
    AdoptionWindowOverflow,
    CommandSpeedInvalid,
    ProjectedSpeedInvalid,
    HeldSpeedInvalid,
    CommandCurvatureInvalid,
    ScanTimeMismatch,
    ConeTimeMismatch,
    ProjectionFailed,
    CurvatureTimingInvalid,
    CurvatureSlew,
    SpeedInterval,
    LateralTransitionInvalid,
    LateralAcceleration,
    EnvelopeTimeInvalid,
    StoppingEnvelopeInvalid,
    MapBoundary,
    LaserObstacle,
    VisionCone,
    LightBoundaryInvalid,
    LightBoundary,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum CertificateMarginUnit {
    Meters,
    MetersPerSecond,
    InverseMeters,
    MetersPerSecondSquared,
}

/// The first failed original certificate check, not an exhaustive collision list.
/// A geometric overlap is a conservative-envelope rejection, not proof of impact.
#[derive(Clone, Debug, Serialize)]
pub struct CertificateFailure {
    pub reason: CertificateFailureReason,
    /// Zero-based laser/cone index or source-envelope corner index, when relevant.
    pub constraint_index: Option<usize>,
    /// Remaining allowance: negative violates a limit; disc tangency (zero) fails.
    /// Non-finite/undefined arithmetic has no numeric margin.
    pub signed_margin: Option<f64>,
    pub margin_unit: Option<CertificateMarginUnit>,
    pub source_at: Timestamp,
    pub pose_at: Timestamp,
    pub oldest_sensor_at: Timestamp,
    pub planned_at: Timestamp,
    pub adopted_revision: u64,
    pub last_command_change_at: Option<Timestamp>,
    pub held_curvature_per_m: Option<f64>,
    pub max_age_ms: u64,
    pub lease_expires_at: Option<Timestamp>,
    /// The adoption window starts at planned_at; an invalid window has no end.
    pub adoption_through: Option<Timestamp>,
    pub historical_speed_bound_mps: Option<f64>,
    pub historical_curvature_bound_per_m: Option<f64>,
    /// Populated only when certification reaches the stopping-envelope stage.
    pub speed_bound_mps: Option<f64>,
    pub curvature_bound_per_m: Option<f64>,
    pub stopping_rectangle_body_m: Option<Rect>,
}

impl CertificateFailure {
    pub(super) fn new(
        reason: CertificateFailureReason,
        input: &SensorSnapshot,
        context: &PlanningContext,
        oldest_sensor_at: Timestamp,
        max_age_ms: u64,
        control_period_ms: u64,
    ) -> Self {
        let expires = oldest_sensor_at.0.checked_add(max_age_ms);
        let through = expires
            .filter(|expires| context.planned_at.0 < *expires)
            .and_then(|expires| {
                context
                    .planned_at
                    .0
                    .checked_add(control_period_ms)
                    .map(|through| through.min(expires - 1))
            });
        Self {
            reason,
            constraint_index: None,
            signed_margin: None,
            margin_unit: None,
            source_at: input.at,
            pose_at: input.pose.captured_at,
            oldest_sensor_at,
            planned_at: context.planned_at,
            adopted_revision: context.adopted_revision,
            last_command_change_at: context.last_command_change_at,
            held_curvature_per_m: finite(context.steering.commanded_curvature_per_m),
            max_age_ms,
            lease_expires_at: expires.map(Timestamp),
            adoption_through: through.map(Timestamp),
            historical_speed_bound_mps: finite(context.historical_speed_bound_mps),
            historical_curvature_bound_per_m: finite(context.historical_curvature_bound_per_m),
            speed_bound_mps: None,
            curvature_bound_per_m: None,
            stopping_rectangle_body_m: None,
        }
    }

    pub(super) fn with_margin(mut self, margin: f64, unit: CertificateMarginUnit) -> Self {
        self.signed_margin = finite(margin);
        self.margin_unit = Some(unit);
        self
    }

    pub(super) fn with_index(mut self, index: usize) -> Self {
        self.constraint_index = Some(index);
        self
    }

    pub(super) fn with_bounds(mut self, speed: f64, curvature: f64) -> Self {
        self.speed_bound_mps = finite(speed);
        self.curvature_bound_per_m = finite(curvature);
        self
    }

    pub(super) fn with_rectangle(mut self, rectangle: Rect) -> Self {
        self.stopping_rectangle_body_m = Some(rectangle);
        self
    }
}

fn finite(value: f64) -> Option<f64> {
    value.is_finite().then_some(value)
}
