//! Shared stopping geometry and prospective asynchronous command constraints.
//! Original source/history bounds remain in force. A predicted future check is
//! a planning constraint, not a replacement for the output owner's certificate.
use crate::Timestamp;
use crate::autonomy::{HalfPlane, ObstacleDisc, Point2, Pose2, Rect};
use crate::motion_transition::{
    MotionProjection, MotionTransition, lateral_acceleration_peak, project_motion,
};
use crate::navigation::NavigationConfig;
use serde::Serialize;

mod slew;
use slew::ForecastSlewClock;
pub use slew::command_slew_allowance;

/// Source-body rectangle containing travel until expiry plus reaction and full
/// braking. Bounds follow path-length/yaw inequalities, not projected xy samples.
#[derive(Clone, Copy, Debug, Serialize)]
pub struct StoppingEnvelope {
    pub body: Rect,
}

impl StoppingEnvelope {
    pub fn new(
        nav: &NavigationConfig,
        source: Pose2,
        speed_bound: f64,
        curvature_bound: f64,
        travel_time_s: f64,
    ) -> Option<Self> {
        if !(0.0..=nav.max_speed_mps).contains(&speed_bound)
            || !(0.0..=nav.max_curvature_per_m).contains(&curvature_bound)
            || !source.valid()
            || !travel_time_s.is_finite()
            || travel_time_s < 0.0
        {
            return None;
        }
        let distance_m =
            speed_bound * travel_time_s + speed_bound.powi(2) / (2.0 * nav.max_decel_mps2);
        let yaw_bound = curvature_bound * distance_m;
        let radius = nav
            .footprint
            .front_m
            .max(nav.footprint.rear_m)
            .hypot(nav.footprint.half_width_m);
        // |yaw(s)| <= K*s and |sin(yaw)| <= min(1,K*s).
        let sideways = distance_m.min(0.5 * curvature_bound * distance_m.powi(2));
        let rotation = (radius * yaw_bound).min(2.0 * radius);
        let backwards = if yaw_bound < std::f64::consts::FRAC_PI_2 {
            0.0
        } else {
            distance_m
        };
        let roundoff = 128.0
            * f64::EPSILON
            * (1.0 + source.x_m.abs() + source.y_m.abs() + distance_m + radius);
        let padding = rotation + nav.clearance_m + roundoff;
        let body = Rect {
            min_x_m: -nav.footprint.rear_m - backwards - padding,
            max_x_m: nav.footprint.front_m + distance_m + padding,
            min_y_m: -nav.footprint.half_width_m - sideways - padding,
            max_y_m: nav.footprint.half_width_m + sideways + padding,
        };
        [body.min_x_m, body.max_x_m, body.min_y_m, body.max_y_m]
            .into_iter()
            .all(f64::is_finite)
            .then_some(Self { body })
    }

    pub fn corners(self) -> [Point2; 4] {
        [
            Point2 {
                x_m: self.body.min_x_m,
                y_m: self.body.min_y_m,
            },
            Point2 {
                x_m: self.body.min_x_m,
                y_m: self.body.max_y_m,
            },
            Point2 {
                x_m: self.body.max_x_m,
                y_m: self.body.min_y_m,
            },
            Point2 {
                x_m: self.body.max_x_m,
                y_m: self.body.max_y_m,
            },
        ]
    }

    pub fn signed_disc_margin(self, point: Point2, radius: f64) -> f64 {
        (point.x_m - point.x_m.clamp(self.body.min_x_m, self.body.max_x_m))
            .hypot(point.y_m - point.y_m.clamp(self.body.min_y_m, self.body.max_y_m))
            - radius
    }

    pub fn clear_of_disc(self, point: Point2, radius: f64) -> bool {
        point.valid()
            && radius.is_finite()
            && radius >= 0.0
            && (point.x_m - point.x_m.clamp(self.body.min_x_m, self.body.max_x_m))
                .hypot(point.y_m - point.y_m.clamp(self.body.min_y_m, self.body.max_y_m))
                > radius
    }

    fn check_world(
        self,
        source: Pose2,
        bounds: Rect,
        obstacles: &[ObstacleDisc],
        boundary: Option<HalfPlane>,
    ) -> Result<(), AdmissionRejection> {
        let corners = self.corners().map(|p| source.body_to_world(p));
        for point in corners {
            if !point.valid() || !bounds.contains(point) {
                return Err(AdmissionRejection::Bounds);
            }
            if boundary.is_some_and(|b| {
                !b.is_laterally_limited() && b.projection(point) > b.max_projection_m()
            }) {
                return Err(AdmissionRejection::Boundary);
            }
        }
        if boundary.is_some_and(|b| b.is_laterally_limited() && !b.contains_points(&corners, 0.0)) {
            return Err(AdmissionRejection::Boundary);
        }
        let (sin, cos) = source.yaw_rad.sin_cos();
        for (index, obstacle) in obstacles.iter().enumerate() {
            let dx = obstacle.center.x_m - source.x_m;
            let dy = obstacle.center.y_m - source.y_m;
            let point = Point2 {
                x_m: cos * dx + sin * dy,
                y_m: -sin * dx + cos * dy,
            };
            let radius = obstacle.radius_m;
            if !point.valid() || !radius.is_finite() || radius < 0.0 {
                return Err(AdmissionRejection::Invalid);
            }
            // Same coordinate differences as the exact distance predicate.
            // A separated axis proves clearance without hypot; subtraction is
            // not replaced with an outward-rounded min/max + radius threshold.
            let gap_x = point.x_m - point.x_m.clamp(self.body.min_x_m, self.body.max_x_m);
            if gap_x.abs() > radius {
                continue;
            }
            let gap_y = point.y_m - point.y_m.clamp(self.body.min_y_m, self.body.max_y_m);
            if gap_y.abs() > radius {
                continue;
            }
            if gap_x.hypot(gap_y) <= radius {
                return Err(AdmissionRejection::Obstacle { index });
            }
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum AdmissionRejection {
    Invalid,
    ForecastBudget,
    Speed,
    Curvature,
    LateralAcceleration,
    Bounds,
    Boundary,
    Obstacle { index: usize },
}

/// Prepared once for a source snapshot. Times are durations derived from that
/// source's original lease; they never renew a command. Obstacles are borrowed
/// from Navigator's already prepared world geometry for each check.
#[derive(Clone, Copy, Debug, Serialize)]
#[cfg_attr(test, derive(serde::Deserialize))]
pub struct AdoptionConstraints {
    /// Earliest certified adoption time; candidate eligibility starts here.
    pub planned_at: Timestamp,
    /// Time of the last changed output command, including speed-only and Stop
    /// changes. Repeated identical commands do not reset this clock.
    pub last_command_change_at: Option<Timestamp>,
    pub adopted_revision: u64,
    pub source_pose: Pose2,
    pub projected_pose: Pose2,
    pub held_speed_mps: f64,
    /// Age of this pose source at planning; defines the next modeled capture phase.
    pub source_age_s: f64,
    /// Inclusive adoption-window duration; actual output still validates its revision.
    pub adoption_window_s: f64,
    pub speed_bound_mps: f64,
    pub curvature_bound_per_m: f64,
    pub projected_speed_mps: f64,
    pub projected_curvature_per_m: f64,
    pub window_end_speed_mps: f64,
    pub window_end_curvature_per_m: f64,
    pub held_curvature_per_m: f64,
    pub source_travel_time_s: f64,
    pub transition_horizon_s: f64,
    /// Full fresh-source age allowance plus one control period, used only to
    /// anticipate a subsequent certificate in the normal-motion model.
    pub future_travel_time_s: f64,
}

/// A bound on work actually requested by prospective checks. Intervals are
/// reserved one-millisecond project_motion intervals, not measured CPU operations.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize)]
pub struct AdmissionForecastWork {
    pub adoption_scenarios: usize,
    pub source_checks: usize,
    pub projection_calls: usize,
    pub projection_intervals: usize,
}

impl AdmissionForecastWork {
    pub fn is_empty(&self) -> bool {
        *self == Self::default()
    }
}

const MAX_FORECAST_PLANS: usize = 64;
const MAX_FORECAST_TIME_S: f64 = 2.0;

#[derive(Clone, Copy)]
struct ForecastSource {
    pose: Pose2,
    speed_bound: f64,
    curvature_bound: f64,
    pending: bool,
}

impl AdoptionConstraints {
    /// Validate prepared values before f64::max/min can hide NaN or a negative
    /// historical bound. Navigator configuration itself is validated on creation.
    pub fn validate(self, nav: &NavigationConfig) -> Result<(), AdmissionRejection> {
        let period = nav.control_period_ms as f64 / 1000.0;
        if command_slew_allowance(
            nav.max_curvature_rate_per_s,
            self.planned_at,
            self.last_command_change_at,
            self.adopted_revision,
            nav.control_period_ms,
        )
        .is_none()
            || (self.adopted_revision == 0
                && (self.held_speed_mps != 0.0
                    || self.held_curvature_per_m != 0.0
                    || self.projected_curvature_per_m != 0.0
                    || self.window_end_curvature_per_m != 0.0
                    || self.curvature_bound_per_m != 0.0))
            || !self.source_pose.valid()
            || !self.projected_pose.valid()
            || [
                self.speed_bound_mps,
                self.held_speed_mps,
                self.projected_speed_mps,
                self.window_end_speed_mps,
            ]
            .iter()
            .any(|v| !(0.0..=nav.max_speed_mps).contains(v))
            || [
                self.projected_curvature_per_m,
                self.window_end_curvature_per_m,
                self.held_curvature_per_m,
            ]
            .iter()
            .any(|k| !k.is_finite() || k.abs() > nav.max_curvature_per_m)
            || !(0.0..=nav.max_curvature_per_m).contains(&self.curvature_bound_per_m)
            || [
                self.source_age_s,
                self.adoption_window_s,
                self.source_travel_time_s,
                self.transition_horizon_s,
                self.future_travel_time_s,
            ]
            .iter()
            .any(|v| !v.is_finite() || *v < 0.0)
            || self.adoption_window_s > period + 1e-12
            || self.source_travel_time_s < period
            || self.future_travel_time_s < period
            || self.transition_horizon_s + 1e-12 < period + self.adoption_window_s
            || self.future_travel_time_s + 1e-12 < self.source_travel_time_s
            || self.source_age_s > nav.max_input_age_ms as f64 / 1000.0
            || (self.source_travel_time_s - self.source_age_s - self.transition_horizon_s).abs()
                > 1e-9
            || self.speed_bound_mps + 1e-12
                < self
                    .projected_speed_mps
                    .max(self.held_speed_mps)
                    .max(self.window_end_speed_mps)
            || self.curvature_bound_per_m + 1e-12
                < self
                    .projected_curvature_per_m
                    .abs()
                    .max(self.held_curvature_per_m.abs())
                    .max(self.window_end_curvature_per_m.abs())
        {
            return Err(AdmissionRejection::Invalid);
        }
        Ok(())
    }

    pub fn speed_interval(self, nav: &NavigationConfig) -> (f64, f64) {
        if self.validate(nav).is_err() {
            // Inverted finite interval also makes callers fail closed before
            // candidate enumeration; check_command returns explicit Invalid.
            return (nav.max_speed_mps + 1.0, 0.0);
        }
        let period = nav.control_period_ms as f64 / 1000.0;
        let low =
            self.projected_speed_mps.max(self.window_end_speed_mps) - nav.max_decel_mps2 * period;
        let high =
            self.projected_speed_mps.min(self.window_end_speed_mps) + nav.max_accel_mps2 * period;
        (low.max(0.0), high.min(nav.max_speed_mps))
    }

    /// Curvature targets already eligible at the start of the adoption window.
    /// Actual poll still checks its own clock and the same execution revision.
    pub fn curvature_interval(self, nav: &NavigationConfig) -> (f64, f64) {
        if self.validate(nav).is_err() {
            return (
                nav.max_curvature_per_m + 1.0,
                -nav.max_curvature_per_m - 1.0,
            );
        }
        let allowance = self.curvature_allowance(nav).expect("validated slew clock");
        (
            (self.held_curvature_per_m - allowance).max(-nav.max_curvature_per_m),
            (self.held_curvature_per_m + allowance).min(nav.max_curvature_per_m),
        )
    }

    fn curvature_allowance(self, nav: &NavigationConfig) -> Option<f64> {
        command_slew_allowance(
            nav.max_curvature_rate_per_s,
            self.planned_at,
            self.last_command_change_at,
            self.adopted_revision,
            nav.control_period_ms,
        )
    }

    pub fn check_command(
        self,
        nav: &NavigationConfig,
        speed: f64,
        curvature: f64,
        obstacles: &[ObstacleDisc],
        boundary: Option<HalfPlane>,
    ) -> Result<(), AdmissionRejection> {
        self.validate(nav)?;
        let (low, high) = self.speed_interval(nav);
        if !speed.is_finite() || speed < low - 1e-9 || speed > high + 1e-9 {
            return Err(AdmissionRejection::Speed);
        }
        if !curvature.is_finite()
            || curvature.abs() > nav.max_curvature_per_m
            || (curvature - self.held_curvature_per_m).abs()
                > self
                    .curvature_allowance(nav)
                    .ok_or(AdmissionRejection::Invalid)?
                    + 1e-9
        {
            return Err(AdmissionRejection::Curvature);
        }
        for v in [self.projected_speed_mps, self.window_end_speed_mps] {
            for k in [
                self.projected_curvature_per_m,
                self.window_end_curvature_per_m,
            ] {
                check_lateral(nav, v, k, speed, curvature, self.transition_horizon_s)?;
            }
        }
        self.envelope(
            nav,
            self.source_pose,
            speed,
            curvature,
            self.source_travel_time_s,
        )?
        .check_world(self.source_pose, nav.bounds, obstacles, boundary)
    }

    /// Bounded model samples, not a proof of recursive feasibility or a
    /// replacement for the independent continuous-window output certificate.
    /// Source captures retain their actual phase relative to planned_at; only
    /// future latency is modeled as the current observed source age.
    pub fn check_next(
        self,
        nav: &NavigationConfig,
        pose: Pose2,
        speed: f64,
        curvature: f64,
        obstacles: &[ObstacleDisc],
        boundary: Option<HalfPlane>,
    ) -> Result<(), AdmissionRejection> {
        self.check_next_with_work(nav, pose, speed, curvature, obstacles, boundary)
            .0
    }

    pub fn check_next_with_work(
        self,
        nav: &NavigationConfig,
        pose: Pose2,
        speed: f64,
        curvature: f64,
        obstacles: &[ObstacleDisc],
        boundary: Option<HalfPlane>,
    ) -> (Result<(), AdmissionRejection>, AdmissionForecastWork) {
        let mut work = AdmissionForecastWork::default();
        let result = self.forecast(nav, pose, speed, curvature, obstacles, boundary, &mut work);
        (result, work)
    }

    #[allow(clippy::too_many_arguments)]
    fn forecast(
        self,
        nav: &NavigationConfig,
        pose: Pose2,
        speed: f64,
        curvature: f64,
        obstacles: &[ObstacleDisc],
        boundary: Option<HalfPlane>,
        work: &mut AdmissionForecastWork,
    ) -> Result<(), AdmissionRejection> {
        self.validate(nav)?;
        if pose != self.projected_pose
            || !(0.0..=nav.max_speed_mps).contains(&speed)
            || !curvature.is_finite()
            || curvature.abs() > nav.max_curvature_per_m
        {
            return Err(AdmissionRejection::Invalid);
        }
        if (curvature - self.held_curvature_per_m).abs()
            > self
                .curvature_allowance(nav)
                .ok_or(AdmissionRejection::Invalid)?
                + 1e-9
        {
            return Err(AdmissionRejection::Curvature);
        }
        let period = nav.control_period_ms as f64 / 1000.0;
        // One candidate interval, adoption slack, full normal braking and a
        // source-to-plan tail. Latest adoption can require repeated deceleration
        // targets to satisfy the earliest endpoint of the following window. We never truncate a too-long braking prediction.
        let horizon = self.projected_speed_mps.max(self.held_speed_mps).max(speed)
            / nav.max_decel_mps2
            * (1.0 + self.adoption_window_s / period)
            + self.adoption_window_s
            + 2.0 * period
            + self.source_age_s;
        let plans = (horizon / period).ceil() as usize;
        if !horizon.is_finite()
            || horizon > MAX_FORECAST_TIME_S
            || plans == 0
            || plans > MAX_FORECAST_PLANS
            || plans as f64 * period > MAX_FORECAST_TIME_S
        {
            return Err(AdmissionRejection::ForecastBudget);
        }
        for fraction in [0.0, 0.5, 1.0] {
            if fraction != 0.0 && self.adoption_window_s == 0.0 {
                continue;
            }
            work.adoption_scenarios += 1;
            self.forecast_sample(
                nav,
                pose,
                speed,
                curvature,
                obstacles,
                boundary,
                plans,
                fraction * self.adoption_window_s,
                work,
            )?;
        }
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    fn forecast_sample(
        self,
        nav: &NavigationConfig,
        pose: Pose2,
        speed: f64,
        curvature: f64,
        obstacles: &[ObstacleDisc],
        boundary: Option<HalfPlane>,
        plans: usize,
        adoption_delay: f64,
        work: &mut AdmissionForecastWork,
    ) -> Result<(), AdmissionRejection> {
        let period = nav.control_period_ms as f64 / 1000.0;
        let mut state = MotionProjection {
            pose,
            speed_mps: self.projected_speed_mps,
            curvature_per_m: self.projected_curvature_per_m,
            distance_m: 0.0,
        };
        let mut sources = [ForecastSource {
            pose,
            speed_bound: 0.0,
            curvature_bound: 0.0,
            pending: false,
        }; MAX_FORECAST_PLANS + 1];
        let mut slew_clock = ForecastSlewClock::new(
            self.planned_at,
            self.last_command_change_at,
            self.adopted_revision,
            nav.control_period_ms,
        )
        .ok_or(AdmissionRejection::Invalid)?;
        let mut held_speed = self.held_speed_mps;
        let mut held_curvature = self.held_curvature_per_m;
        let mut pending_speed = speed;
        let mut pending_at = adoption_delay;
        let mut plan_index = 1_usize;
        // n*period - age > 0 selects genuinely new capture times. Historical
        // peaks are not deleted from the existing source: check_command keeps
        // them verbatim. Every new source starts at its modeled measured state.
        let mut source_index = (self.source_age_s / period).floor() as usize + 1;
        let mut now = 0.0;
        while plan_index <= plans {
            let plan_at = plan_index as f64 * period;
            let source_at = if source_index <= plans {
                source_index as f64 * period - self.source_age_s
            } else {
                f64::INFINITY
            };
            let at = plan_at.min(source_at).min(pending_at);
            if at + 1e-12 < now {
                return Err(AdmissionRejection::Invalid);
            }
            let dt = (at - now).max(0.0);
            if dt > 0.0 {
                work.projection_calls += 1;
                work.projection_intervals += (dt / 0.001).ceil() as usize;
                state = project_motion(
                    state.pose,
                    transition(
                        nav,
                        state.speed_mps,
                        state.curvature_per_m,
                        held_speed,
                        held_curvature,
                    ),
                    dt,
                )
                .ok_or(AdmissionRejection::Invalid)?;
                now = at;
            }
            if pending_at <= at + 1e-12 {
                let allowance = slew_clock
                    .allowance(nav.max_curvature_rate_per_s, at)
                    .ok_or(AdmissionRejection::Invalid)?;
                if (curvature - held_curvature).abs() > allowance + 1e-9 {
                    return Err(AdmissionRejection::Curvature);
                }
                if pending_speed != held_speed || curvature != held_curvature {
                    slew_clock
                        .changed_at(at)
                        .ok_or(AdmissionRejection::Invalid)?;
                }
                held_speed = pending_speed;
                held_curvature = curvature;
                pending_at = f64::INFINITY;
                for source in sources.iter_mut().filter(|s| s.pending) {
                    source.speed_bound = source.speed_bound.max(held_speed);
                    source.curvature_bound = source.curvature_bound.max(held_curvature.abs());
                }
            }
            if source_at <= at + 1e-12 {
                sources[source_index] = ForecastSource {
                    pose: state.pose,
                    speed_bound: state.speed_mps.max(held_speed),
                    curvature_bound: state.curvature_per_m.abs().max(held_curvature.abs()),
                    pending: true,
                };
                source_index += 1;
            }
            if plan_at <= at + 1e-12 {
                // Each virtual plan uses the latest actual model adoption,
                // including a speed-only change in its braking fallback.
                let allowance = slew_clock
                    .allowance(nav.max_curvature_rate_per_s, plan_at)
                    .ok_or(AdmissionRejection::Invalid)?;
                if (curvature - held_curvature).abs() > allowance + 1e-9 {
                    return Err(AdmissionRejection::Curvature);
                }
                // A valid normal-braking candidate for every state in this
                // modeled adoption window, not an instantaneous jump to zero.
                let end_speed = ramp(
                    state.speed_mps,
                    held_speed,
                    nav.max_accel_mps2,
                    nav.max_decel_mps2,
                    self.adoption_window_s,
                );
                let end_curvature = ramp(
                    state.curvature_per_m,
                    held_curvature,
                    nav.max_curvature_rate_per_s,
                    nav.max_curvature_rate_per_s,
                    self.adoption_window_s,
                );
                let next_speed =
                    (state.speed_mps.max(end_speed) - nav.max_decel_mps2 * period).max(0.0);
                for v in [state.speed_mps, end_speed] {
                    for k in [state.curvature_per_m, end_curvature] {
                        check_lateral(nav, v, k, next_speed, curvature, self.transition_horizon_s)?;
                    }
                }
                if sources[plan_index].pending {
                    let source = &mut sources[plan_index];
                    work.source_checks += 1;
                    let envelope = StoppingEnvelope::new(
                        nav,
                        source.pose,
                        source.speed_bound.max(next_speed),
                        source.curvature_bound.max(curvature.abs()),
                        self.future_travel_time_s,
                    )
                    .ok_or(AdmissionRejection::Invalid)?;
                    envelope.check_world(source.pose, nav.bounds, obstacles, boundary)?;
                    source.pending = false;
                }
                pending_speed = next_speed;
                pending_at = plan_at + adoption_delay;
                plan_index += 1;
            }
        }
        // At the chosen bounded horizon the fallback must actually have stopped
        // in this model; otherwise its prospective geometry is incomplete.
        if state.speed_mps > 1e-9 {
            return Err(AdmissionRejection::ForecastBudget);
        }
        Ok(())
    }

    fn envelope(
        self,
        nav: &NavigationConfig,
        pose: Pose2,
        speed: f64,
        curvature: f64,
        time_s: f64,
    ) -> Result<StoppingEnvelope, AdmissionRejection> {
        StoppingEnvelope::new(
            nav,
            pose,
            self.speed_bound_mps.max(speed),
            self.curvature_bound_per_m.max(curvature.abs()),
            time_s,
        )
        .ok_or(AdmissionRejection::Invalid)
    }
}

fn ramp(initial: f64, target: f64, increase: f64, decrease: f64, time: f64) -> f64 {
    if initial < target {
        (initial + increase * time).min(target)
    } else {
        (initial - decrease * time).max(target)
    }
}

fn check_lateral(
    nav: &NavigationConfig,
    v: f64,
    k: f64,
    speed: f64,
    curvature: f64,
    horizon: f64,
) -> Result<(), AdmissionRejection> {
    let peak = lateral_acceleration_peak(transition(nav, v, k, speed, curvature), horizon)
        .ok_or(AdmissionRejection::Invalid)?;
    if peak.lateral_accel_mps2 > nav.max_lateral_accel_mps2 + 1e-9 {
        Err(AdmissionRejection::LateralAcceleration)
    } else {
        Ok(())
    }
}

fn transition(
    nav: &NavigationConfig,
    v: f64,
    k: f64,
    speed: f64,
    curvature: f64,
) -> MotionTransition {
    MotionTransition {
        initial_speed_mps: v,
        initial_curvature_per_m: k,
        target_speed_mps: speed,
        target_curvature_per_m: curvature,
        max_accel_mps2: nav.max_accel_mps2,
        max_decel_mps2: nav.max_decel_mps2,
        max_curvature_rate_per_s: nav.max_curvature_rate_per_s,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::FrameId;
    use crate::autonomy::Footprint;

    #[test]
    fn finite_boundary_checks_the_entire_stopping_envelope() {
        let nav = config();
        let envelope = StoppingEnvelope {
            body: Rect {
                min_x_m: 0.5,
                max_x_m: 0.7,
                min_y_m: -1.0,
                max_y_m: 1.0,
            },
        };
        let boundary = HalfPlane::new(Point2::default(), 0.0, 0.2)
            .unwrap()
            .with_lateral_region(Rect {
                min_x_m: -1.0,
                max_x_m: 1.0,
                min_y_m: -0.4,
                max_y_m: 0.4,
            })
            .unwrap();
        assert!(
            envelope
                .corners()
                .into_iter()
                .all(|p| boundary.contains_disc(p, 0.0))
        );
        assert_eq!(
            envelope.check_world(Pose2::default(), nav.bounds, &[], Some(boundary)),
            Err(AdmissionRejection::Boundary)
        );
        assert!(
            envelope
                .check_world(
                    Pose2 {
                        y_m: 2.0,
                        ..Pose2::default()
                    },
                    nav.bounds,
                    &[],
                    Some(boundary)
                )
                .is_ok()
        );
    }

    fn config() -> NavigationConfig {
        let mut nav = NavigationConfig::simulation(
            Rect {
                min_x_m: -10.0,
                max_x_m: 10.0,
                min_y_m: -10.0,
                max_y_m: 10.0,
            },
            Footprint {
                front_m: 0.22,
                rear_m: 0.18,
                half_width_m: 0.13,
            },
            FrameId("map".into()),
        );
        nav.control_period_ms = 100;
        nav
    }

    #[test]
    fn source_history_at_limits_makes_all_legal_candidate_envelopes_identical() {
        let nav = config();
        let pose = Pose2 {
            x_m: 3.5826095652422083,
            y_m: 3.1671618603443346,
            yaw_rad: -0.9566791890666444,
        };
        let reference = StoppingEnvelope::new(&nav, pose, 0.3, 2.0, 0.35).unwrap();
        assert!((reference.body.max_x_m - 0.5319939128423334).abs() < 1e-15);
        for speed in [0.03_f64, 0.06, 0.12, 0.18, 0.24, 0.3] {
            for index in 0..=40 {
                let k = -2.0 + index as f64 * 0.1;
                let envelope = StoppingEnvelope::new(
                    &nav,
                    pose,
                    0.3_f64.max(speed),
                    2.0_f64.max(k.abs()),
                    0.35,
                )
                .unwrap();
                assert_eq!(reference.body, envelope.body);
                assert!(
                    envelope.signed_disc_margin(
                        Point2 {
                            x_m: 0.5411247215224134,
                            y_m: 0.2999503311047326
                        },
                        0.015
                    ) < -0.0043
                );
            }
        }
    }

    #[test]
    fn adoption_window_intersects_both_speed_endpoints() {
        let nav = config();
        let constraints = AdoptionConstraints {
            planned_at: Timestamp(100),
            last_command_change_at: Some(Timestamp(0)),
            adopted_revision: 1,
            source_pose: Pose2 {
                x_m: 0.0,
                y_m: 0.0,
                yaw_rad: 0.0,
            },
            projected_pose: Pose2 {
                x_m: 0.0,
                y_m: 0.0,
                yaw_rad: 0.0,
            },
            held_speed_mps: 0.24,
            source_age_s: 0.06,
            adoption_window_s: 0.1,
            speed_bound_mps: 0.25,
            curvature_bound_per_m: 0.0,
            projected_speed_mps: 0.2,
            projected_curvature_per_m: 0.0,
            window_end_speed_mps: 0.24,
            window_end_curvature_per_m: 0.0,
            held_curvature_per_m: 0.0,
            source_travel_time_s: 0.35,
            transition_horizon_s: 0.29,
            future_travel_time_s: 0.35,
        };
        let (low, high) = constraints.speed_interval(&nav);
        assert!((low - 0.18).abs() < 1e-12);
        assert!((high - 0.24).abs() < 1e-12);
        assert_eq!(
            constraints.check_command(&nav, 0.16, 0.0, &[], None),
            Err(AdmissionRejection::Speed)
        );
        assert!(
            constraints
                .check_command(&nav, 0.22, 0.0, &[], None)
                .is_ok()
        );
    }

    fn straight_constraints() -> AdoptionConstraints {
        AdoptionConstraints {
            planned_at: Timestamp(100),
            last_command_change_at: Some(Timestamp(0)),
            adopted_revision: 1,
            source_pose: Pose2 {
                x_m: 0.0,
                y_m: 0.0,
                yaw_rad: 0.0,
            },
            projected_pose: Pose2 {
                x_m: 0.0,
                y_m: 0.0,
                yaw_rad: 0.0,
            },
            speed_bound_mps: 0.3,
            curvature_bound_per_m: 0.0,
            projected_speed_mps: 0.3,
            projected_curvature_per_m: 0.0,
            window_end_speed_mps: 0.3,
            window_end_curvature_per_m: 0.0,
            held_speed_mps: 0.3,
            held_curvature_per_m: 0.0,
            source_age_s: 0.0,
            adoption_window_s: 0.1,
            source_travel_time_s: 0.35,
            transition_horizon_s: 0.35,
            future_travel_time_s: 0.35,
        }
    }

    #[test]
    fn multi_source_braking_forecast_avoids_the_two_tick_wall_trap() {
        let nav = config();
        let c = straight_constraints();
        let wall = [ObstacleDisc {
            center: Point2 {
                x_m: 0.489,
                y_m: 0.0,
            },
            radius_m: 0.015,
        }];
        // Both current commands have EXACTLY the same history-limited envelope.
        for speed in [0.3, 0.24] {
            assert!(c.check_command(&nav, speed, 0.0, &wall, None).is_ok());
            assert_eq!(
                c.envelope(&nav, c.source_pose, speed, 0.0, 0.35)
                    .unwrap()
                    .body,
                c.envelope(&nav, c.source_pose, 0.3, 0.0, 0.35)
                    .unwrap()
                    .body
            );
        }
        // The old one-step check accepts the fast command (front x=.47).
        let first =
            project_motion(c.projected_pose, transition(&nav, 0.3, 0.0, 0.3, 0.0), 0.1).unwrap();
        assert!(
            c.envelope(&nav, first.pose, 0.3, 0.0, 0.35)
                .unwrap()
                .check_world(first.pose, nav.bounds, &wall, None)
                .is_ok()
        );
        // Further source windows expose the trap, including latest adoption.
        assert_eq!(
            c.check_next(&nav, c.projected_pose, 0.3, 0.0, &wall, None),
            Err(AdmissionRejection::Obstacle { index: 0 })
        );
        let (slower, work) = c.check_next_with_work(&nav, c.projected_pose, 0.24, 0.0, &wall, None);
        assert_eq!(slower, Ok(()));
        assert_eq!(work.adoption_scenarios, 3);
        assert!(work.source_checks > 3);
        assert!(work.projection_intervals <= 6100);
    }

    #[test]
    fn nonfinite_or_inconsistent_prepared_fields_are_explicitly_invalid() {
        let nav = config();
        for change in 0..8 {
            let mut c = straight_constraints();
            match change {
                0 => c.speed_bound_mps = f64::NAN,
                1 => c.curvature_bound_per_m = f64::NAN,
                2 => c.held_curvature_per_m = f64::NAN,
                3 => c.source_age_s = -0.01,
                4 => c.source_travel_time_s = f64::INFINITY,
                5 => c.adoption_window_s = 0.2,
                6 => c.speed_bound_mps = -0.1,
                _ => c.transition_horizon_s = 0.2,
            }
            assert_eq!(
                c.check_command(&nav, 0.24, 0.0, &[], None),
                Err(AdmissionRejection::Invalid)
            );
            assert_eq!(
                c.check_next(&nav, c.projected_pose, 0.24, 0.0, &[], None),
                Err(AdmissionRejection::Invalid)
            );
        }
    }

    #[test]
    fn future_capture_phase_and_braking_budget_are_bounded() {
        let mut nav = config();
        let mut c = straight_constraints();
        c.source_age_s = 0.08;
        c.transition_horizon_s = 0.27;
        c.projected_pose.x_m = 0.024;
        let (result, work) = c.check_next_with_work(&nav, c.projected_pose, 0.24, 0.0, &[], None);
        assert_eq!(result, Ok(()));
        assert_eq!(work.adoption_scenarios, 3);
        assert!(work.source_checks >= 18);
        assert!(work.projection_calls < 150);
        nav.max_decel_mps2 = 0.01;
        let (result, work) = c.check_next_with_work(&nav, c.projected_pose, 0.3, 0.0, &[], None);
        assert_eq!(result, Err(AdmissionRejection::ForecastBudget));
        assert_eq!(work.projection_calls, 0);
    }

    #[test]
    fn capture_before_late_adoption_preserves_the_old_target_in_its_history() {
        let nav = config();
        let mut c = straight_constraints();
        c.source_age_s = 0.08;
        c.transition_horizon_s = 0.27;
        c.projected_pose.x_m = 0.024;
        let wall = [ObstacleDisc {
            center: Point2 {
                x_m: 0.48,
                y_m: 0.0,
            },
            radius_m: 0.015,
        }];
        assert!(c.check_command(&nav, 0.24, 0.0, &wall, None).is_ok());
        // Next capture is 20 ms after planned_at, not planned_at+100 ms.
        // Early adoption gives measured v=.288 at x=.02988. The late branch
        // still measures v=.3 at x=.03, whose historical envelope ends at .47
        // and intersects the wall beginning at .465. The later low target may
        // not remove the preceding held .3 from this future source's history.
        let mut early = AdmissionForecastWork::default();
        assert_eq!(
            c.forecast_sample(
                &nav,
                c.projected_pose,
                0.24,
                0.0,
                &wall,
                None,
                14,
                0.0,
                &mut early
            ),
            Ok(())
        );
        let mut late = AdmissionForecastWork::default();
        assert_eq!(
            c.forecast_sample(
                &nav,
                c.projected_pose,
                0.24,
                0.0,
                &wall,
                None,
                14,
                0.1,
                &mut late
            ),
            Err(AdmissionRejection::Obstacle { index: 0 })
        );
        assert_eq!(late.source_checks, 1);
    }
}
