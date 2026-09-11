//! Fixed-size adopted-command history and conservative asynchronous admission.
//! Predictions are model states; original sensor timestamps and poses are retained.
use crate::autonomy::{AutonomyConfig, AutonomyStep};
use crate::autonomy_replay::SensorSnapshot;
use std::sync::atomic::{AtomicU64, Ordering};
use xt_stcar_robot_core::autonomy::{Point2, PoseEstimate};
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

/// Conservative static-world certificate. Its circle is centered on the measured
/// source pose and includes all travel until the original source lease expires,
/// one output period, and braking, regardless of steering or projection error.
pub(crate) fn certify(
    config: &AutonomyConfig,
    input: &SensorSnapshot,
    context: &PlanningContext,
    step: &AutonomyStep,
    oldest_at: Timestamp,
    max_age_ms: u64,
) -> Option<AdoptionCertificate> {
    let nav = &config.navigation;
    measurement_speed(input.pose.speed_mps, nav.max_speed_mps)?;
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
    let radius = nav
        .footprint
        .front_m
        .max(nav.footprint.rear_m)
        .hypot(nav.footprint.half_width_m)
        + nav.clearance_m
        + nav.max_speed_mps
            * (expires
                .checked_add(nav.control_period_ms)?
                .checked_sub(input.pose.captured_at.0)?) as f64
            / 1000.0
        + nav.max_speed_mps.powi(2) / (2.0 * nav.max_decel_mps2);
    let center = input.pose.pose.point();
    if !radius.is_finite()
        || !center.valid()
        || center.x_m - radius < nav.bounds.min_x_m
        || center.x_m + radius > nav.bounds.max_x_m
        || center.y_m - radius < nav.bounds.min_y_m
        || center.y_m + radius > nav.bounds.max_y_m
    {
        return None;
    }
    for (index, range) in input.scan.ranges_m.iter().enumerate() {
        if let Some(range) = range {
            let angle = input.scan.angle_min_rad + index as f64 * input.scan.angle_increment_rad;
            let point = input
                .pose
                .pose
                .body_to_world(config.lidar_in_body.body_to_world(Point2 {
                    x_m: range * angle.cos(),
                    y_m: range * angle.sin(),
                }));
            if !point.valid() || center.distance(point) <= radius + config.laser_point_radius_m {
                return None;
            }
        }
    }
    if input.road.observation.cones_body_m.iter().any(|point| {
        !point.valid()
            || center.distance(input.pose.pose.body_to_world(*point))
                <= radius + config.cone_radius_m
    }) {
        return None;
    }
    if step.mission.as_ref().is_some_and(|mission| {
        matches!(
            mission.phase,
            MissionPhase::Cones | MissionPhase::ApproachLight | MissionPhase::WaitGreen
        )
    }) && !config
        .mission
        .light_stop_boundary()
        .ok()?
        .contains_disc(center, radius)
    {
        return None;
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
}
