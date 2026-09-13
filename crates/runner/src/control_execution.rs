//! Fixed-size adopted-command history and conservative asynchronous admission.
//! Predictions are model states; original sensor timestamps and poses are retained.
use crate::autonomy::{AutonomyConfig, AutonomyStep};
use crate::autonomy_replay::SensorSnapshot;
use std::sync::atomic::{AtomicU64, Ordering};
use xt_stcar_robot_core::autonomy::{Point2, PoseEstimate, Rect};
use xt_stcar_robot_core::mission::MissionPhase;
use xt_stcar_robot_core::motion_transition::{
    MotionTransition, lateral_acceleration_peak, project_motion,
};
use xt_stcar_robot_core::navigation::SteeringEstimate;
use xt_stcar_robot_core::{MotionOutput, Timestamp};

const HISTORY_CAPACITY: usize = 128;

#[derive(Clone, Copy, Debug)]
pub(crate) struct ExecutionRecord {
    pub steering: SteeringEstimate,
    pub speed_target: f64,
    pub stopped: bool,
}

impl ExecutionRecord {
    fn stationary(at: Timestamp) -> Self {
        Self {
            steering: SteeringEstimate::stationary(at),
            speed_target: 0.0,
            stopped: true,
        }
    }
}

struct AtomicRecord {
    at: AtomicU64,
    commanded: AtomicU64,
    applied: AtomicU64,
    speed: AtomicU64,
    stopped: AtomicU64,
}

impl AtomicRecord {
    fn new(record: ExecutionRecord) -> Self {
        Self {
            at: AtomicU64::new(record.steering.at.0),
            commanded: AtomicU64::new(record.steering.commanded_curvature_per_m.to_bits()),
            applied: AtomicU64::new(record.steering.applied_curvature_per_m.to_bits()),
            speed: AtomicU64::new(record.speed_target.to_bits()),
            stopped: AtomicU64::new(u64::from(record.stopped)),
        }
    }

    fn write(&self, record: ExecutionRecord) {
        self.at.store(record.steering.at.0, Ordering::SeqCst);
        self.commanded.store(
            record.steering.commanded_curvature_per_m.to_bits(),
            Ordering::SeqCst,
        );
        self.applied.store(
            record.steering.applied_curvature_per_m.to_bits(),
            Ordering::SeqCst,
        );
        self.speed
            .store(record.speed_target.to_bits(), Ordering::SeqCst);
        self.stopped
            .store(u64::from(record.stopped), Ordering::SeqCst);
    }

    fn read(&self) -> ExecutionRecord {
        ExecutionRecord {
            steering: SteeringEstimate {
                at: Timestamp(self.at.load(Ordering::SeqCst)),
                commanded_curvature_per_m: f64::from_bits(self.commanded.load(Ordering::SeqCst)),
                applied_curvature_per_m: f64::from_bits(self.applied.load(Ordering::SeqCst)),
            },
            speed_target: f64::from_bits(self.speed.load(Ordering::SeqCst)),
            stopped: self.stopped.load(Ordering::SeqCst) != 0,
        }
    }
}

/// Single output owner, one bounded seqlock read attempt; poll never takes a lock.
pub(crate) struct AtomicExecution {
    version: AtomicU64,
    revision: AtomicU64,
    latest: AtomicRecord,
    records: [AtomicRecord; HISTORY_CAPACITY],
}

#[derive(Clone)]
pub(crate) struct ExecutionHistory {
    pub revision: u64,
    pub latest: ExecutionRecord,
    records: [ExecutionRecord; HISTORY_CAPACITY],
    len: usize,
}

impl AtomicExecution {
    pub fn new(state: SteeringEstimate) -> Self {
        let record = ExecutionRecord::stationary(state.at);
        Self {
            version: AtomicU64::new(0),
            revision: AtomicU64::new(0),
            latest: AtomicRecord::new(record),
            records: std::array::from_fn(|_| AtomicRecord::new(record)),
        }
    }

    pub fn publish(&self, state: SteeringEstimate, command: &MotionOutput) {
        let previous = self.latest.read();
        let (speed_target, stopped) = match command {
            MotionOutput::Stop => (0.0, true),
            MotionOutput::Drive { speed_mps, .. } => (*speed_mps, false),
        };
        let next = ExecutionRecord {
            steering: state,
            speed_target,
            stopped,
        };
        self.version.fetch_add(1, Ordering::SeqCst);
        // Unchanged commands do not consume history capacity or invalidate plans.
        if previous.speed_target != next.speed_target
            || previous.stopped != next.stopped
            || previous.steering.commanded_curvature_per_m != state.commanded_curvature_per_m
        {
            let revision = self.revision.load(Ordering::SeqCst).wrapping_add(1);
            self.records[revision as usize % HISTORY_CAPACITY].write(next);
            self.revision.store(revision, Ordering::SeqCst);
        }
        self.latest.write(next);
        self.version.fetch_add(1, Ordering::SeqCst);
    }

    pub fn revision(&self) -> u64 {
        self.revision.load(Ordering::SeqCst)
    }

    /// Called only by the single poll owner, before its next publication.
    pub fn last_change(&self) -> ExecutionRecord {
        self.records[self.revision() as usize % HISTORY_CAPACITY].read()
    }

    pub fn read(&self) -> Option<ExecutionHistory> {
        let version = self.version.load(Ordering::SeqCst);
        if !version.is_multiple_of(2) {
            return None;
        }
        let revision = self.revision.load(Ordering::SeqCst);
        let latest = self.latest.read();
        let len = revision.saturating_add(1).min(HISTORY_CAPACITY as u64) as usize;
        let first = revision.saturating_sub((HISTORY_CAPACITY - 1) as u64);
        let mut records = [ExecutionRecord::stationary(Timestamp(0)); HISTORY_CAPACITY];
        for (index, record) in records[..len].iter_mut().enumerate() {
            *record = self.records[(first as usize + index) % HISTORY_CAPACITY].read();
        }
        (self.version.load(Ordering::SeqCst) == version).then_some(ExecutionHistory {
            revision,
            latest,
            records,
            len,
        })
    }
}

impl ExecutionHistory {
    fn anchor(&self, at: Timestamp) -> Option<usize> {
        self.records[..self.len]
            .iter()
            .rposition(|record| record.steering.at <= at)
    }

    pub fn steering_at(&self, at: Timestamp, rate: f64) -> Option<SteeringEstimate> {
        let mut state = self.records[self.anchor(at)?].steering;
        state.advance_to(at, rate).ok()?;
        Some(state)
    }

    pub fn project(
        &self,
        input: &SensorSnapshot,
        planned_at: Timestamp,
        config: &AutonomyConfig,
    ) -> Option<PlanningContext> {
        let nav = &config.navigation;
        if self.latest.steering.at > planned_at
            || input.pose.captured_at > planned_at
            || planned_at.0 - input.pose.captured_at.0 > 2000
        {
            return None;
        }
        let initial_speed = measurement_speed(input.pose.speed_mps, nav.max_speed_mps)?;
        let start = self.anchor(input.pose.captured_at)?;
        let mut pose = input.pose.clone();
        pose.speed_mps = initial_speed;
        let mut steering = self.steering_at(pose.captured_at, nav.max_curvature_rate_per_s)?;
        let mut command = self.records[start];
        let mut speed_bound = initial_speed;
        let mut curvature_bound = steering.applied_curvature_per_m.abs();
        for index in start + 1..=self.len {
            let at = if index == self.len {
                planned_at
            } else {
                self.records[index].steering.at.min(planned_at)
            };
            if at < steering.at {
                return None;
            }
            if command.speed_target < 0.0 || command.speed_target > nav.max_speed_mps {
                return None;
            }
            speed_bound = speed_bound.max(command.speed_target);
            curvature_bound = curvature_bound.max(command.steering.commanded_curvature_per_m.abs());
            let motion = transition(
                config,
                pose.speed_mps,
                steering.applied_curvature_per_m,
                command.speed_target,
                command.steering.commanded_curvature_per_m,
            );
            let projected =
                project_motion(pose.pose, motion, (at.0 - steering.at.0) as f64 / 1000.0)?;
            pose.pose = projected.pose;
            pose.speed_mps = projected.speed_mps;
            pose.yaw_rate_radps = projected.speed_mps * projected.curvature_per_m;
            steering.at = at;
            steering.applied_curvature_per_m = projected.curvature_per_m;
            if at == planned_at {
                break;
            }
            command = self.records[index];
            steering.commanded_curvature_per_m = command.steering.commanded_curvature_per_m;
        }
        pose.captured_at = planned_at;
        // Same-time changes are ordered by their published revision, not rewound.
        steering.commanded_curvature_per_m = self.latest.steering.commanded_curvature_per_m;
        Some(PlanningContext {
            source_at: input.at,
            planned_at,
            projected_pose: pose,
            steering,
            adopted_revision: self.revision,
            held_speed_mps: self.latest.speed_target,
            // Include same-time adoption at planned_at even though it has not
            // moved the model yet. Earlier transient targets must not disappear.
            historical_speed_bound_mps: speed_bound.max(self.latest.speed_target),
            historical_curvature_bound_per_m: curvature_bound
                .max(self.latest.steering.commanded_curvature_per_m.abs()),
        })
    }
}

/// Explicit model projection. `projected_pose.captured_at` denotes its model time;
/// the original SensorSnapshot remains untouched and is still supplied separately.
#[derive(Clone, Debug)]
pub struct PlanningContext {
    pub source_at: Timestamp,
    pub planned_at: Timestamp,
    pub projected_pose: PoseEstimate,
    pub steering: SteeringEstimate,
    pub adopted_revision: u64,
    pub held_speed_mps: f64,
    /// Source measurement and every adopted target through planned_at. These
    /// bounds survive later lower targets, including Stop while still recentering.
    pub historical_speed_bound_mps: f64,
    pub historical_curvature_bound_per_m: f64,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct AdoptionCertificate {
    pub from: Timestamp,
    pub through: Timestamp,
    pub revision: u64,
}

/// Match Navigator's existing measurement-roundoff allowance. This applies only
/// to measured source speed, never to a command or model limit. Source records
/// remain unchanged; the projected model starts within the strict forward range.
fn measurement_speed(speed: f64, maximum: f64) -> Option<f64> {
    (speed.is_finite() && (-1e-6..=maximum + 1e-6).contains(&speed))
        .then(|| speed.clamp(0.0, maximum))
}

fn transition(
    config: &AutonomyConfig,
    speed: f64,
    curvature: f64,
    target_speed: f64,
    target_curvature: f64,
) -> MotionTransition {
    MotionTransition {
        initial_speed_mps: speed,
        target_speed_mps: target_speed,
        initial_curvature_per_m: curvature,
        target_curvature_per_m: target_curvature,
        max_accel_mps2: config.navigation.max_accel_mps2,
        max_decel_mps2: config.navigation.max_decel_mps2,
        max_curvature_rate_per_s: config.navigation.max_curvature_rate_per_s,
    }
}

/// A source-body rectangle containing the entire possible stopping motion.
/// The bound uses path length/yaw inequalities, not numerically projected xy.
#[derive(Clone, Copy, Debug)]
struct StoppingEnvelope {
    body: Rect,
}

impl StoppingEnvelope {
    fn new(
        config: &AutonomyConfig,
        source: &PoseEstimate,
        speed_bound: f64,
        curvature_bound: f64,
        travel_time_s: f64,
    ) -> Option<Self> {
        let nav = &config.navigation;
        if !(0.0..=nav.max_speed_mps).contains(&speed_bound)
            || !(0.0..=nav.max_curvature_per_m).contains(&curvature_bound)
            || !travel_time_s.is_finite()
            || travel_time_s < 0.0
        {
            return None;
        }
        // A monotone ramp never exceeds its initial speed/target maximum.
        // This bound does not depend on the acceleration rate, so a faster
        // actuator ramp cannot escape it. Braking assumes at least max_decel.
        let distance_m =
            speed_bound * travel_time_s + speed_bound.powi(2) / (2.0 * nav.max_decel_mps2);
        let yaw_bound = curvature_bound * distance_m;
        let radius = nav
            .footprint
            .front_m
            .max(nav.footprint.rear_m)
            .hypot(nav.footprint.half_width_m);
        // |yaw(s)| <= K*s; integrate |sin(yaw)| <= min(1,K*s).
        let sideways = distance_m.min(0.5 * curvature_bound * distance_m.powi(2));
        // A corner rotates by at most min(R*|yaw|,2R). This includes the
        // intermediate curvature while Stop brings the steering back to zero.
        let rotation = (radius * yaw_bound).min(2.0 * radius);
        // A direction beyond +/- pi/2 can travel behind the original pose.
        let backwards = if yaw_bound < std::f64::consts::FRAC_PI_2 {
            0.0
        } else {
            distance_m
        };
        // Roundoff allowance for the few source/world transforms below. Source
        // coordinates are already restricted by project_motion to +/- 1e6 m.
        let roundoff = 128.0
            * f64::EPSILON
            * (1.0 + source.pose.x_m.abs() + source.pose.y_m.abs() + distance_m + radius);
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

    fn corners(self) -> [Point2; 4] {
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

    fn clear_of_disc(self, point: Point2, radius: f64) -> bool {
        point.valid()
            && (point.x_m - point.x_m.clamp(self.body.min_x_m, self.body.max_x_m))
                .hypot(point.y_m - point.y_m.clamp(self.body.min_y_m, self.body.max_y_m))
                > radius
    }
}

/// Conservative static-world certificate. The source-body envelope contains all
/// travel until the original source lease expires, one output period, and full
/// braking, for every adoption time in the window. It preserves direction without
/// assuming the projected pose is exact or shortening the braking distance.
pub(crate) fn certify(
    config: &AutonomyConfig,
    input: &SensorSnapshot,
    context: &PlanningContext,
    step: &AutonomyStep,
    oldest_at: Timestamp,
    max_age_ms: u64,
) -> Option<AdoptionCertificate> {
    let nav = &config.navigation;
    let source_speed = measurement_speed(input.pose.speed_mps, nav.max_speed_mps)?;
    let expires = oldest_at.0.checked_add(max_age_ms)?;
    if context.planned_at.0 >= expires {
        return None;
    }
    let through = context
        .planned_at
        .0
        .checked_add(nav.control_period_ms)?
        .min(expires - 1);
    let certificate = AdoptionCertificate {
        from: context.planned_at,
        through: Timestamp(through),
        revision: context.adopted_revision,
    };
    let MotionOutput::Drive {
        speed_mps,
        curvature_per_m,
    } = step.command
    else {
        return Some(certificate);
    };
    if !(0.0..=nav.max_speed_mps).contains(&speed_mps)
        || !(0.0..=nav.max_speed_mps).contains(&context.projected_pose.speed_mps)
        || !(0.0..=nav.max_speed_mps).contains(&context.held_speed_mps)
        || curvature_per_m.abs() > nav.max_curvature_per_m
        || input.scan.captured_at != input.pose.captured_at
        || (!input.road.observation.cones_body_m.is_empty()
            && input.road.observation.captured_at != input.pose.captured_at)
    {
        return None;
    }
    let end = project_motion(
        context.projected_pose.pose,
        transition(
            config,
            context.projected_pose.speed_mps,
            context.steering.applied_curvature_per_m,
            context.held_speed_mps,
            context.steering.commanded_curvature_per_m,
        ),
        (through - context.planned_at.0) as f64 / 1000.0,
    )?;
    let period_s = nav.control_period_ms as f64 / 1000.0;
    if (curvature_per_m - context.steering.commanded_curvature_per_m).abs()
        > nav.max_curvature_rate_per_s * period_s + 1e-9
    {
        return None;
    }
    // Monotone ramps lie in this rectangle. For every later time the largest
    // nonnegative speed and largest |curvature| occur at rectangle corners.
    for speed in [context.projected_pose.speed_mps, end.speed_mps] {
        if speed_mps < (speed - nav.max_decel_mps2 * period_s).max(0.0) - 1e-9
            || speed_mps > (speed + nav.max_accel_mps2 * period_s).min(nav.max_speed_mps) + 1e-9
        {
            return None;
        }
        for curvature in [
            context.steering.applied_curvature_per_m,
            end.curvature_per_m,
        ] {
            let peak = lateral_acceleration_peak(
                transition(config, speed, curvature, speed_mps, curvature_per_m),
                (expires - context.planned_at.0 + nav.control_period_ms) as f64 / 1000.0,
            )?;
            if peak.lateral_accel_mps2 > nav.max_lateral_accel_mps2 + 1e-9 {
                return None;
            }
        }
    }
    let speed_bound = context
        .historical_speed_bound_mps
        .max(source_speed)
        .max(context.projected_pose.speed_mps)
        .max(context.held_speed_mps)
        .max(speed_mps);
    let curvature_bound = context
        .historical_curvature_bound_per_m
        .max(context.steering.applied_curvature_per_m.abs())
        .max(context.steering.commanded_curvature_per_m.abs())
        .max(curvature_per_m.abs());
    let envelope = StoppingEnvelope::new(
        config,
        &input.pose,
        speed_bound,
        curvature_bound,
        expires
            .checked_add(nav.control_period_ms)?
            .checked_sub(input.pose.captured_at.0)? as f64
            / 1000.0,
    )?;
    let corners = envelope
        .corners()
        .map(|point| input.pose.pose.body_to_world(point));
    if corners
        .iter()
        .any(|point| !point.valid() || !nav.bounds.contains(*point))
    {
        return None;
    }
    for (index, range) in input.scan.ranges_m.iter().enumerate() {
        if let Some(range) = range {
            let angle = input.scan.angle_min_rad + index as f64 * input.scan.angle_increment_rad;
            let point = config.lidar_in_body.body_to_world(Point2 {
                x_m: range * angle.cos(),
                y_m: range * angle.sin(),
            });
            if !envelope.clear_of_disc(point, config.laser_point_radius_m) {
                return None;
            }
        }
    }
    if input
        .road
        .observation
        .cones_body_m
        .iter()
        .any(|point| !envelope.clear_of_disc(*point, config.cone_radius_m))
    {
        return None;
    }
    if step.mission.as_ref().is_some_and(|mission| {
        matches!(
            mission.phase,
            MissionPhase::Cones | MissionPhase::ApproachLight | MissionPhase::WaitGreen
        )
    }) {
        let boundary = config.mission.light_stop_boundary().ok()?;
        if corners
            .into_iter()
            .any(|point| boundary.projection(point) > boundary.max_projection_m())
        {
            return None;
        }
    }
    Some(certificate)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unchanged_polls_preserve_old_anchors_and_changes_have_bounded_storage() {
        let history = AtomicExecution::new(SteeringEstimate::stationary(Timestamp(0)));
        for at in 1..=1000 {
            history.publish(
                SteeringEstimate::stationary(Timestamp(at)),
                &MotionOutput::Stop,
            );
        }
        assert!(
            history
                .read()
                .unwrap()
                .steering_at(Timestamp(0), 4.0)
                .is_some()
        );
        let mut state = SteeringEstimate::stationary(Timestamp(1000));
        for index in 1..=150 {
            let command = MotionOutput::Drive {
                speed_mps: 0.1,
                curvature_per_m: if index % 2 == 0 { 0.2 } else { -0.2 },
            };
            state
                .adopt(Timestamp(1000 + index), &command, 4.0, 2.0)
                .unwrap();
            history.publish(state, &command);
        }
        let snapshot = history.read().unwrap();
        assert_eq!(snapshot.len, HISTORY_CAPACITY);
        assert!(snapshot.steering_at(Timestamp(1001), 4.0).is_none());
        assert!(snapshot.steering_at(Timestamp(1100), 4.0).is_some());
    }

    #[test]
    fn same_time_adoption_does_not_rewrite_a_previously_bound_history() {
        let history = AtomicExecution::new(SteeringEstimate::stationary(Timestamp(0)));
        let previous = history.read().unwrap();
        let mut state = SteeringEstimate::stationary(Timestamp(0));
        let command = MotionOutput::Drive {
            speed_mps: 0.1,
            curvature_per_m: 1.0,
        };
        state.adopt(Timestamp(10), &command, 4.0, 2.0).unwrap();
        history.publish(state, &command);
        assert_eq!(
            previous
                .steering_at(Timestamp(20), 4.0)
                .unwrap()
                .applied_curvature_per_m,
            0.0
        );
        assert_eq!(
            history
                .read()
                .unwrap()
                .steering_at(Timestamp(5), 4.0)
                .unwrap()
                .commanded_curvature_per_m,
            0.0
        );
        assert_eq!(
            history
                .read()
                .unwrap()
                .steering_at(Timestamp(20), 4.0)
                .unwrap()
                .applied_curvature_per_m,
            0.04
        );
    }

    fn fixture() -> (
        AutonomyConfig,
        SensorSnapshot,
        PlanningContext,
        AutonomyStep,
    ) {
        use crate::autonomy::RoadFrame;
        use crate::simulation::{SimulationConfig, synthetic_scan};
        use xt_stcar_robot_core::autonomy::{LightState, RoadObservation};
        use xt_stcar_robot_core::{OutputRecord, State, StepReport};
        let simulation = SimulationConfig::example();
        let config = simulation.autonomy.clone();
        let input = SensorSnapshot {
            at: Timestamp(0),
            pose: PoseEstimate {
                captured_at: Timestamp(0),
                frame_id: config.mission.world_frame.clone(),
                pose: simulation.initial_pose,
                speed_mps: 0.0,
                yaw_rate_radps: 0.0,
                quality: 1.0,
            },
            scan: synthetic_scan(&simulation, simulation.initial_pose, &[], Timestamp(0)),
            road: RoadFrame {
                observation: RoadObservation {
                    captured_at: Timestamp(0),
                    frame_id: config.mission.body_frame.clone(),
                    crosswalk: None,
                    light: LightState::Unknown,
                    light_confidence: 0.0,
                    cones_body_m: vec![],
                },
                image_width_px: 320,
                image_height_px: 240,
            },
        };
        let history = AtomicExecution::new(SteeringEstimate::stationary(Timestamp(0)));
        let context = history
            .read()
            .unwrap()
            .project(&input, Timestamp(60), &config)
            .unwrap();
        let command = MotionOutput::Drive {
            speed_mps: 0.04,
            curvature_per_m: 0.0,
        };
        let step = AutonomyStep {
            kind: "autonomy_step",
            mode: "certificate_test",
            physical_output_enabled: false,
            at: context.planned_at,
            mission: None,
            navigation: None,
            safety: StepReport {
                event_at: context.planned_at,
                previous_state: State::Running,
                state: State::Running,
                output: OutputRecord {
                    at: context.planned_at,
                    state: State::Running,
                    command: command.clone(),
                    reason: None,
                },
                emergency_stop_latched: false,
            },
            command,
            fault: None,
        };
        (config, input, context, step)
    }

    #[test]
    fn transient_history_peaks_and_stop_recentring_remain_in_the_envelope() {
        let (config, mut input, _, _) = fixture();
        let history = AtomicExecution::new(SteeringEstimate::stationary(Timestamp(0)));
        let mut steering = SteeringEstimate::stationary(Timestamp(0));
        for index in 0..8 {
            let turn = MotionOutput::Drive {
                speed_mps: (0.04 * (index + 1) as f64).min(0.3),
                curvature_per_m: (0.4 * (index + 1) as f64).min(2.0),
            };
            steering
                .adopt(Timestamp(index * 100), &turn, 4.0, 2.0)
                .unwrap();
            history.publish(steering, &turn);
        }
        steering
            .adopt(Timestamp(800), &MotionOutput::Stop, 4.0, 2.0)
            .unwrap();
        history.publish(steering, &MotionOutput::Stop);
        let history = history.read().unwrap();
        for source_at in [730, 810] {
            input.at = Timestamp(source_at);
            input.pose.captured_at = input.at;
            input.pose.speed_mps = 0.19;
            let context = history.project(&input, Timestamp(840), &config).unwrap();
            assert_eq!(context.held_speed_mps, 0.0);
            assert_eq!(context.steering.commanded_curvature_per_m, 0.0);
            assert!((context.steering.applied_curvature_per_m - 1.84).abs() < 1e-12);
            if source_at == 730 {
                assert_eq!(context.historical_speed_bound_mps, 0.3);
                assert_eq!(context.historical_curvature_bound_per_m, 2.0);
            } else {
                assert_eq!(context.historical_speed_bound_mps, 0.19);
                assert!((context.historical_curvature_bound_per_m - 1.96).abs() < 1e-12);
            }
        }
    }

    #[test]
    fn analytic_envelope_contains_independent_delayed_adoption_and_full_braking_paths() {
        use xt_stcar_robot_core::autonomy::Pose2;
        let (mut config, mut input, _, _) = fixture();
        // Last case reaches the legal speed/curvature limits and turns well
        // beyond pi/2. The others include reversal of steering and Stop target
        // while applied steering is still nonzero. Plant accel is deliberately
        // greater than nav accel; the certificate relies on the speed bound.
        for (v, k, decel, rate, initial_v, initial_k, held_v, held_k, target_v, target_k) in [
            (0.3, 2.0, 0.6, 4.0, 0.3, 0.0, 0.3, 0.0, 0.3, 0.0),
            (0.3, 2.0, 0.6, 4.0, 0.15, 1.8, 0.3, 2.0, 0.05, -2.0),
            (0.3, 2.0, 0.6, 4.0, 0.3, -2.0, 0.0, 0.0, 0.02, 0.0),
            (3.0, 10.0, 1.0, 50.0, 3.0, 10.0, 3.0, 10.0, 3.0, -10.0),
        ] {
            config.navigation.max_speed_mps = v;
            config.navigation.max_curvature_per_m = k;
            config.navigation.max_decel_mps2 = decel;
            config.navigation.max_curvature_rate_per_s = rate;
            config.navigation.validate().unwrap();
            input.pose.pose = Pose2 {
                x_m: 2.0,
                y_m: 2.5,
                yaw_rad: 1.2,
            };
            let envelope = StoppingEnvelope::new(&config, &input.pose, v, k, 0.35).unwrap();
            let travel_bound = v * 0.35 + v * v / (2.0 * decel);
            if k * travel_bound >= std::f64::consts::FRAC_PI_2 {
                assert!(envelope.body.min_x_m < -travel_bound);
            }
            for adopt_ms in 80..=180 {
                let mut pose = input.pose.pose;
                let mut speed: f64 = initial_v;
                let mut curvature: f64 = initial_k;
                let mut travelled = 0.0;
                let dt = 0.0005;
                for index in 0..20_000 {
                    let t = index as f64 * dt;
                    let (target_speed, target_curvature) = if t < adopt_ms as f64 / 1000.0 {
                        (held_v, held_k)
                    } else if t < 0.35 {
                        (target_v, target_k)
                    } else {
                        (0.0, 0.0)
                    };
                    let acceleration = if target_speed >= speed { 10.0 } else { decel };
                    let next_speed =
                        speed + (target_speed - speed).clamp(-acceleration * dt, acceleration * dt);
                    let next_curvature =
                        curvature + (target_curvature - curvature).clamp(-rate * dt, rate * dt);
                    // Independent midpoint integration, never project_motion.
                    let ds = (speed + next_speed) * 0.5 * dt;
                    let dyaw = ds * (curvature + next_curvature) * 0.5;
                    pose.x_m += ds * (pose.yaw_rad + dyaw * 0.5).cos();
                    pose.y_m += ds * (pose.yaw_rad + dyaw * 0.5).sin();
                    pose.yaw_rad += dyaw;
                    travelled += ds;
                    speed = next_speed;
                    curvature = next_curvature;
                    for corner in config.navigation.footprint.corners(pose) {
                        let source_body = input.pose.pose.world_to_body(corner);
                        assert!(
                            envelope.body.contains(source_body),
                            "v={v} k={k} adopt={adopt_ms} t={t} corner={source_body:?} envelope={envelope:?}"
                        );
                    }
                    if t >= 0.35 && speed == 0.0 && curvature == 0.0 {
                        assert!(travelled <= travel_bound + 1e-9);
                        break;
                    }
                    assert!(index < 19_999, "full stop and recenter must be simulated");
                }
            }
        }
    }

    #[test]
    fn envelope_preserves_forward_stopping_room_laser_tf_and_cone_radii() {
        use xt_stcar_robot_core::autonomy::Pose2;
        let (mut config, mut input, mut context, mut step) = fixture();
        input.pose.speed_mps = 0.3;
        context.projected_pose.speed_mps = 0.3;
        context.historical_speed_bound_mps = 0.3;
        context.held_speed_mps = 0.3;
        step.command = MotionOutput::Drive {
            speed_mps: 0.3,
            curvature_per_m: 0.0,
        };
        config.lidar_in_body = Pose2 {
            x_m: 0.1,
            y_m: -0.04,
            yaw_rad: 0.2,
        };
        for (obstacle, clear) in [
            (
                Point2 {
                    x_m: 0.0,
                    y_m: 0.45,
                },
                true,
            ),
            (
                Point2 {
                    x_m: 0.30,
                    y_m: 0.0,
                },
                false,
            ),
        ] {
            let laser = config.lidar_in_body.world_to_body(obstacle);
            input.scan.angle_min_rad = laser.y_m.atan2(laser.x_m);
            input.scan.ranges_m = vec![Some(laser.x_m.hypot(laser.y_m))];
            assert_eq!(
                certify(&config, &input, &context, &step, Timestamp(0), 250).is_some(),
                clear
            );
        }
        input.scan.ranges_m = vec![Some(5.0)];
        for (y, clear) in [(0.29, false), (0.32, true)] {
            input.road.observation.cones_body_m = vec![Point2 { x_m: 0.0, y_m: y }];
            assert_eq!(
                certify(&config, &input, &context, &step, Timestamp(0), 250).is_some(),
                clear
            );
        }
    }

    #[test]
    fn rotated_envelope_checks_world_bounds_oblique_light_line_and_disc_tangency() {
        use xt_stcar_robot_core::mission::{MissionOutput, MissionReport};
        let (mut config, mut input, mut context, mut step) = fixture();
        input.scan.ranges_m = vec![Some(5.0); 360];
        input.pose.pose.yaw_rad = std::f64::consts::FRAC_PI_2;
        input.pose.pose.x_m = 6.85;
        context.projected_pose.pose = input.pose.pose;
        assert!(
            config
                .navigation
                .footprint
                .inside(input.pose.pose, config.navigation.bounds)
        );
        // Rotating the source footprint rotates the envelope too; its lateral
        // margin crosses world +x even though its forward travel is world +y.
        assert!(certify(&config, &input, &context, &step, Timestamp(0), 250).is_none());
        config.mission.light_approach_yaw_rad = std::f64::consts::FRAC_PI_4;
        step.mission = Some(MissionReport {
            at: context.planned_at,
            phase: MissionPhase::ApproachLight,
            output: MissionOutput::Stop,
            reason: "test oblique boundary".into(),
            crosswalk_stop_elapsed_ms: 0,
            green_elapsed_ms: 0,
            waypoint_index: 2,
        });
        input.pose.pose.x_m = 5.5;
        for (y, allowed) in [(3.10, true), (3.24, false)] {
            input.pose.pose.y_m = y;
            context.projected_pose.pose = input.pose.pose;
            assert!(
                config
                    .mission
                    .light_stop_boundary()
                    .unwrap()
                    .contains_footprint(config.navigation.footprint, input.pose.pose, 0.0)
            );
            assert_eq!(
                certify(&config, &input, &context, &step, Timestamp(0), 250).is_some(),
                allowed
            );
        }
        let envelope = StoppingEnvelope {
            body: Rect {
                min_x_m: -1.0,
                max_x_m: 1.0,
                min_y_m: -1.0,
                max_y_m: 1.0,
            },
        };
        assert!(!envelope.clear_of_disc(
            Point2 {
                x_m: 0.0,
                y_m: 1.125
            },
            0.125
        ));
        assert!(envelope.clear_of_disc(
            Point2 {
                x_m: 0.0,
                y_m: 1.125000001
            },
            0.125
        ));
    }

    #[test]
    #[ignore = "opt-in host cost measurement, not a target deadline guarantee"]
    fn certificate_host_cost_measurement() {
        use std::hint::black_box;
        use std::time::{Duration, Instant};
        let (config, mut input, context, step) = fixture();
        for points in [360, 2048] {
            input.scan.ranges_m = vec![Some(5.0); points];
            input.scan.angle_increment_rad = std::f64::consts::TAU / points as f64;
            let mut total = Duration::ZERO;
            let mut peak = Duration::ZERO;
            let n = 2000;
            for _ in 0..n {
                let start = Instant::now();
                let certificate = certify(
                    black_box(&config),
                    black_box(&input),
                    black_box(&context),
                    black_box(&step),
                    Timestamp(0),
                    250,
                );
                let elapsed = start.elapsed();
                total += elapsed;
                peak = peak.max(elapsed);
                assert!(black_box(certificate).is_some());
            }
            eprintln!(
                "async certificate only host cost: n={n} points={points} total_ns={} mean_ns={} max_ns={} (not WCET)",
                total.as_nanos(),
                total.as_nanos() / n,
                peak.as_nanos()
            );
        }
    }
}
