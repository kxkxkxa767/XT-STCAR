//! Complete synthetic competition through the actual asynchronous worker.
//! Functional time advances on a fixed event schedule; waiting for host planning
//! freezes that clock. This deliberately does not claim processor deadlines.
use crate::autonomy::{Result, RoadFrame};
use crate::autonomy_replay::SensorSnapshot;
use crate::control_diagnostics::{PlanTimings, WorkerDiagnosticsOptions};
use crate::control_runtime::{
    AdoptionRejection, AutonomyWorker, ControlFault, ControlPoll, ControlRuntimeConfig,
    SubmitStatus,
};
use crate::navigation_diagnostics::{NavigationFrame, is_unexpected_stop};
use crate::phase_statistics::{
    CompetitionStatistics, CompetitionStatisticsCollector, FinalBrakingCause,
};
use crate::simulation::{
    PlantDynamics, PlantState, SimulationConfig, SimulationFault, advance_plant,
    boundary_for_phase, check_plant_pose, obstacle_clearance, render_camera, synthetic_scan,
};
use crate::telemetry::{RunJournal, TelemetryMode};
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use std::io::Write;
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};
use xt_stcar_robot_core::autonomy::{HalfPlane, LightState, ObstacleDisc, Pose2, PoseEstimate};
use xt_stcar_robot_core::mission::{MissionPhase, MissionReport};
use xt_stcar_robot_core::{MotionOutput, State, Timestamp};
use xt_stcar_vision::road::RoadDetector;

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct AsyncSimulationOptions {
    pub capture_period_ms: u64,
    pub output_period_ms: u64,
    pub input_delays_ms: [u64; 2],
    pub adoption_delays_ms: [u64; 3],
    pub stop_input_at_ms: Option<u64>,
    pub measure_wall_time: bool,
}

impl Default for AsyncSimulationOptions {
    fn default() -> Self {
        Self {
            capture_period_ms: 100,
            output_period_ms: 20,
            input_delays_ms: [60, 80],
            adoption_delays_ms: [3, 7, 9],
            stop_input_at_ms: None,
            measure_wall_time: false,
        }
    }
}

impl AsyncSimulationOptions {
    fn validate(&self, config: &SimulationConfig) -> Result<()> {
        if !(20..=config.autonomy.navigation.control_period_ms).contains(&self.output_period_ms)
            || self.capture_period_ms != config.time_step_ms
            || !self.capture_period_ms.is_multiple_of(self.output_period_ms)
            || self
                .input_delays_ms
                .iter()
                .any(|delay| *delay >= self.capture_period_ms)
            || self
                .input_delays_ms
                .iter()
                .any(|delay| !delay.is_multiple_of(self.output_period_ms))
            || self
                .adoption_delays_ms
                .iter()
                .any(|delay| *delay >= self.output_period_ms)
            || self.input_delays_ms.iter().max().unwrap()
                - self.input_delays_ms.iter().min().unwrap()
                + self.adoption_delays_ms.iter().max().unwrap()
                >= self.capture_period_ms
        {
            return Err("functional async timing requires bounded non-overlapping captures/adoptions and periodic output".into());
        }
        Ok(())
    }
}

#[derive(Debug, Default, Serialize)]
pub struct AsyncCounts {
    pub captures: u64,
    pub submissions: u64,
    pub completed_plans: u64,
    pub newly_adopted_plans: u64,
    /// Scheduled model-time outputs, not repeated frozen-time host wait polls.
    pub output_polls: u64,
    pub drive_polls: u64,
    pub stop_polls: u64,
    pub turning_drive_polls: u64,
    pub before_window: u64,
    pub after_window: u64,
    pub execution_changed: u64,
    pub curvature_slew: u64,
    pub certificate_unsafe: u64,
}

impl AsyncCounts {
    fn rejection(&mut self, rejection: AdoptionRejection) {
        let count = match rejection {
            AdoptionRejection::BeforeWindow => &mut self.before_window,
            AdoptionRejection::AfterWindow => &mut self.after_window,
            AdoptionRejection::ExecutionChanged => &mut self.execution_changed,
            AdoptionRejection::CurvatureSlew => &mut self.curvature_slew,
            AdoptionRejection::CertificateUnsafe => &mut self.certificate_unsafe,
        };
        *count += 1;
    }
}

#[derive(Debug, Serialize)]
pub struct AsyncProblem {
    pub certificate_failure: Option<crate::control_runtime::CertificateFailure>,
    pub at: Timestamp,
    pub actual_pose: Pose2,
    pub actual_speed_mps: f64,
    pub actual_curvature_per_m: f64,
    pub command: MotionOutput,
    pub runtime_fault: Option<ControlFault>,
    pub adoption_rejection: Option<AdoptionRejection>,
    pub control: Option<NavigationFrame>,
    /// One bounded source snapshot, no RGB or model tensor payload.
    pub source: Option<SensorSnapshot>,
}

#[derive(Debug, Serialize)]
pub struct AsyncAttempt {
    pub certificate_failure: Option<crate::control_runtime::CertificateFailure>,
    pub at: Timestamp,
    pub source_at: Timestamp,
    pub oldest_sensor_at: Timestamp,
    pub planned_at: Timestamp,
    pub source_age_ms: u64,
    pub plan_age_ms: u64,
    /// Whether adoption happened on this attempt's first observation.
    pub newly_adopted: bool,
    pub adopted_at: Option<Timestamp>,
    pub adopted_command: Option<MotionOutput>,
    pub publication_to_adoption_ns: Option<u64>,
    pub actual_pose: Pose2,
    pub actual_speed_mps: f64,
    pub actual_curvature_per_m: f64,
    pub command: MotionOutput,
    pub runtime_fault: Option<ControlFault>,
    pub adoption_rejection: Option<AdoptionRejection>,
    pub mission: Option<MissionReport>,
    pub control: NavigationFrame,
    pub source: Option<SensorSnapshot>,
    pub timings: Option<PlanTimings>,
    pub publication_to_observation_ns: Option<u64>,
}

#[derive(Debug, Default, Serialize)]
pub struct SampleStatistics {
    pub samples: u64,
    pub total: u64,
    pub maximum: u64,
}

impl SampleStatistics {
    fn observe(&mut self, value: u64) {
        self.samples = self.samples.saturating_add(1);
        self.total = self.total.saturating_add(value);
        self.maximum = self.maximum.max(value);
    }
}

#[derive(Debug, Default, Serialize)]
pub struct AsyncTimingStatistics {
    pub source_age_ms: SampleStatistics,
    pub plan_age_ms: SampleStatistics,
    pub adopted_source_age_ms: SampleStatistics,
    pub adopted_plan_age_ms: SampleStatistics,
    pub publication_to_adoption_ns: SampleStatistics,
    pub queue_wait_ns: SampleStatistics,
    pub projection_ns: SampleStatistics,
    pub processor_ns: SampleStatistics,
    pub navigation_ns: SampleStatistics,
    pub terminal_solver_ns: SampleStatistics,
    pub certificate_ns: SampleStatistics,
    pub publication_wait_ns: SampleStatistics,
    pub publication_to_observation_ns: SampleStatistics,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
struct TransitionState {
    stopped: bool,
    phase: Option<MissionPhase>,
    adoption_rejection: Option<AdoptionRejection>,
    navigation_reason: Option<String>,
    runtime_fault: Option<ControlFault>,
}

#[derive(Debug, Serialize)]
pub struct AsyncTransition {
    pub certificate_failure: Option<crate::control_runtime::CertificateFailure>,
    pub at: Timestamp,
    pub source_at: Option<Timestamp>,
    pub planned_at: Option<Timestamp>,
    pub pose: Pose2,
    pub speed_mps: f64,
    pub curvature_per_m: f64,
    state: TransitionState,
}

#[derive(Debug, Serialize)]
pub struct AsyncSimulationSummary {
    pub kind: &'static str,
    pub physical_output_enabled: bool,
    pub clock_scope: &'static str,
    pub timing: AsyncSimulationOptions,
    pub completed: bool,
    pub elapsed_ms: u64,
    pub wall_time_ms: f64,
    pub distance_m: f64,
    pub minimum_cone_clearance_m: f64,
    pub crosswalk_hold_ms: u64,
    pub green_observed_ms: u64,
    pub phases: Vec<MissionPhase>,
    pub final_pose: Pose2,
    pub final_actual_speed_mps: f64,
    pub final_actual_curvature_per_m: f64,
    pub terminal_command: MotionOutput,
    pub fault: Option<String>,
    pub counts: AsyncCounts,
    pub statistics: CompetitionStatistics,
    pub first_problem: Option<AsyncProblem>,
    pub last_control: Option<NavigationFrame>,
    /// The final 16 published attempts, including rejected/faulted reports.
    pub recent_attempts: VecDeque<AsyncAttempt>,
    pub timing_statistics: AsyncTimingStatistics,
    /// At most 128 Drive/Stop/phase/rejection/reason changes; ordinary changes
    /// of speed within Drive do not consume this event budget.
    pub transitions: Vec<AsyncTransition>,
    pub omitted_transitions: u64,
    pub dropped_trace_records: u64,
    pub logging_errors: u64,
}

struct Pending {
    delivery_at: u64,
    snapshot: Arc<SensorSnapshot>,
}

struct Awaiting {
    planned_at: u64,
    adoption_at: u64,
    snapshot: Arc<SensorSnapshot>,
}

struct Observation {
    counts: AsyncCounts,
    phases: Vec<MissionPhase>,
    active_boundary: Option<HalfPlane>,
    held: u64,
    green: u64,
    completed: bool,
    fault: Option<String>,
    last_step_at: Option<Timestamp>,
    last_plan_at: Option<Timestamp>,
    first_problem: Option<AsyncProblem>,
    last_control: Option<NavigationFrame>,
    recent_attempts: VecDeque<AsyncAttempt>,
    timing_statistics: AsyncTimingStatistics,
    transitions: Vec<AsyncTransition>,
    previous_transition: Option<TransitionState>,
    omitted_transitions: u64,
    statistics: CompetitionStatisticsCollector,
    journal: RunJournal,
    logging_errors: u64,
}

impl Observation {
    fn new(config: &SimulationConfig) -> Result<Self> {
        Ok(Self {
            counts: AsyncCounts::default(),
            phases: vec![],
            active_boundary: None,
            held: 0,
            green: 0,
            completed: false,
            fault: None,
            last_step_at: None,
            last_plan_at: None,
            first_problem: None,
            last_control: None,
            recent_attempts: VecDeque::with_capacity(16),
            timing_statistics: AsyncTimingStatistics::default(),
            transitions: Vec::with_capacity(128),
            previous_transition: None,
            omitted_transitions: 0,
            statistics: CompetitionStatisticsCollector::default(),
            journal: RunJournal::new(config.telemetry.clone())?,
            logging_errors: 0,
        })
    }

    fn poll(
        &mut self,
        poll: &ControlPoll,
        plant: PlantState,
        source: Option<&SensorSnapshot>,
        light_boundary: HalfPlane,
    ) {
        self.counts.output_polls += 1;
        match poll.command {
            MotionOutput::Stop => self.counts.stop_polls += 1,
            MotionOutput::Drive {
                curvature_per_m, ..
            } => {
                self.counts.drive_polls += 1;
                self.counts.turning_drive_polls += u64::from(curvature_per_m.abs() > 1e-9);
            }
        }
        let new_plan = poll
            .observed_plan
            .as_ref()
            .is_some_and(|plan| Some(plan.planned_at) != self.last_plan_at);
        let newly_adopted = poll
            .observed_plan
            .as_ref()
            .is_some_and(|plan| plan.newly_adopted);
        if let Some(plan) = &poll.observed_plan
            && new_plan
        {
            self.last_plan_at = Some(plan.planned_at);
            self.counts.completed_plans += 1;
            if let Some(rejection) = poll.adoption_rejection {
                self.counts.rejection(rejection);
            }
            self.timing_statistics
                .source_age_ms
                .observe(plan.source_age_ms);
            self.timing_statistics.plan_age_ms.observe(plan.plan_age_ms);
            if let Some(timing) = &plan.timings {
                self.timing_statistics
                    .queue_wait_ns
                    .observe(timing.queue_wait_ns);
                self.timing_statistics
                    .projection_ns
                    .observe(timing.projection_duration_ns);
                self.timing_statistics
                    .processor_ns
                    .observe(timing.processor_duration_ns);
                self.timing_statistics
                    .certificate_ns
                    .observe(timing.certify_duration_ns);
                self.timing_statistics.publication_wait_ns.observe(
                    timing
                        .published_host_ns
                        .saturating_sub(timing.ready_to_publish_host_ns),
                );
                if let Some(value) = timing.navigation_duration_ns {
                    self.timing_statistics.navigation_ns.observe(value);
                }
                if let Some(value) = timing.terminal_solver_duration_ns {
                    self.timing_statistics.terminal_solver_ns.observe(value);
                }
                if let Some(observed) = plan.observed_host_ns {
                    self.timing_statistics
                        .publication_to_observation_ns
                        .observe(observed.saturating_sub(timing.published_host_ns));
                }
            }
            self.last_control = Some(NavigationFrame::from_step(
                poll.at,
                plant.pose,
                plant.speed_mps,
                &plan.report,
            ));
            if let Some(fault) = &plan.report.fault {
                self.fault.get_or_insert_with(|| fault.clone());
            }
            if self.recent_attempts.len() == 16 {
                self.recent_attempts.pop_front();
            }
            self.recent_attempts.push_back(AsyncAttempt {
                certificate_failure: plan.certificate_failure.as_deref().cloned(),
                at: poll.at,
                source_at: plan.source_at,
                oldest_sensor_at: plan.oldest_sensor_at,
                planned_at: plan.planned_at,
                source_age_ms: plan.source_age_ms,
                plan_age_ms: plan.plan_age_ms,
                newly_adopted: plan.newly_adopted,
                adopted_at: None,
                adopted_command: None,
                publication_to_adoption_ns: None,
                actual_pose: plant.pose,
                actual_speed_mps: plant.speed_mps,
                actual_curvature_per_m: plant.curvature_per_m,
                command: poll.command.clone(),
                runtime_fault: poll.fault,
                adoption_rejection: poll.adoption_rejection,
                mission: plan.report.mission.clone(),
                control: self.last_control.clone().expect("observed report"),
                source: source
                    .filter(|snapshot| snapshot.at == plan.source_at)
                    .cloned(),
                timings: plan.timings.as_deref().cloned(),
                publication_to_observation_ns: plan
                    .timings
                    .as_ref()
                    .zip(plan.observed_host_ns)
                    .map(|(timing, observed)| observed.saturating_sub(timing.published_host_ns)),
            });
        }
        if let Some(plan) = &poll.observed_plan
            && newly_adopted
        {
            self.counts.newly_adopted_plans += 1;
            self.timing_statistics
                .adopted_source_age_ms
                .observe(plan.source_age_ms);
            self.timing_statistics
                .adopted_plan_age_ms
                .observe(plan.plan_age_ms);
            let publication_to_adoption_ns = plan
                .timings
                .as_ref()
                .zip(plan.observed_host_ns)
                .map(|(timing, observed)| observed.saturating_sub(timing.published_host_ns));
            if let Some(value) = publication_to_adoption_ns {
                self.timing_statistics
                    .publication_to_adoption_ns
                    .observe(value);
            }
            if let Some(attempt) = self
                .recent_attempts
                .iter_mut()
                .rev()
                .find(|attempt| attempt.planned_at == plan.planned_at)
            {
                attempt.adopted_at = Some(poll.at);
                attempt.adopted_command = Some(poll.command.clone());
                attempt.publication_to_adoption_ns = publication_to_adoption_ns;
            }
        }
        let mut phase_changed = false;
        let mut unexpected_stop = false;
        if let Some(step) = &poll.latest
            && self.last_step_at != Some(step.at)
        {
            self.last_step_at = Some(step.at);
            self.statistics.observe_step(step);
            if let Some(mission) = &step.mission {
                self.active_boundary =
                    boundary_for_phase(self.active_boundary, mission.phase, light_boundary);
                if self.phases.last() != Some(&mission.phase) {
                    self.phases.push(mission.phase);
                    phase_changed = true;
                }
                self.held = self.held.max(mission.crosswalk_stop_elapsed_ms);
                self.green = self.green.max(mission.green_elapsed_ms);
                self.completed = mission.phase == MissionPhase::Completed
                    && step.safety.state == State::Running
                    && step.fault.is_none()
                    && poll.fault.is_none();
            }
            unexpected_stop = is_unexpected_stop(
                step.navigation.as_ref().map(|nav| nav.status),
                step.navigation
                    .as_ref()
                    .and_then(|nav| nav.reason.as_deref()),
                step.fault.is_some(),
            );
            self.last_control = Some(NavigationFrame::from_step(
                poll.at,
                plant.pose,
                plant.speed_mps,
                step,
            ));
            if let Some(fault) = &step.fault {
                self.fault.get_or_insert_with(|| fault.clone());
            }
        }
        if let Some(fault) = poll.fault {
            self.fault
                .get_or_insert_with(|| format!("runtime {fault:?}"));
        }
        {
            let report = poll.observed_plan.as_ref().map(|plan| &plan.report);
            let state = TransitionState {
                stopped: poll.command == MotionOutput::Stop,
                phase: report.and_then(|step| step.mission.as_ref().map(|mission| mission.phase)),
                adoption_rejection: poll.adoption_rejection,
                navigation_reason: report
                    .and_then(|step| step.navigation.as_ref())
                    .and_then(|nav| nav.reason.as_ref())
                    .map(|reason| reason.chars().take(256).collect()),
                runtime_fault: poll.fault,
            };
            if self.previous_transition.as_ref() != Some(&state) {
                self.previous_transition = Some(state.clone());
                if self.transitions.len() < 128 {
                    self.transitions.push(AsyncTransition {
                        certificate_failure: poll
                            .observed_plan
                            .as_ref()
                            .and_then(|plan| plan.certificate_failure.as_deref().cloned()),
                        at: poll.at,
                        source_at: poll.observed_plan.as_ref().map(|plan| plan.source_at),
                        planned_at: poll.observed_plan.as_ref().map(|plan| plan.planned_at),
                        pose: plant.pose,
                        speed_mps: plant.speed_mps,
                        curvature_per_m: plant.curvature_per_m,
                        state,
                    });
                } else {
                    self.omitted_transitions = self.omitted_transitions.saturating_add(1);
                }
            }
        }
        let first_problem = self.first_problem.is_none()
            && (poll.fault.is_some()
                || (new_plan && poll.adoption_rejection.is_some())
                || unexpected_stop);
        if first_problem {
            self.first_problem = Some(AsyncProblem {
                certificate_failure: poll
                    .observed_plan
                    .as_ref()
                    .and_then(|plan| plan.certificate_failure.as_deref().cloned()),
                at: poll.at,
                actual_pose: plant.pose,
                actual_speed_mps: plant.speed_mps,
                actual_curvature_per_m: plant.curvature_per_m,
                command: poll.command.clone(),
                runtime_fault: poll.fault,
                adoption_rejection: poll.adoption_rejection,
                control: self.last_control.clone(),
                source: source
                    .filter(|snapshot| {
                        poll.observed_plan
                            .as_ref()
                            .is_some_and(|plan| plan.source_at == snapshot.at)
                    })
                    .cloned(),
            });
        }
        let important = first_problem
            || (phase_changed && self.journal.config().mode != TelemetryMode::Summary);
        if (new_plan || newly_adopted || important || poll.fault.is_some()) && self.journal.record(&serde_json::json!({
            "kind":"async_poll", "at":poll.at, "command":poll.command,
            "fault":poll.fault,"adoption_rejection":poll.adoption_rejection,
            "source_at":poll.observed_plan.as_ref().map(|plan| plan.source_at),
            "planned_at":poll.observed_plan.as_ref().map(|plan| plan.planned_at),
            "pose":plant.pose,"speed_mps":plant.speed_mps,"curvature_per_m":plant.curvature_per_m,
            "control":self.last_control,
        }), important).is_err() {
            self.logging_errors += 1;
        }
    }
}

fn capture(
    config: &SimulationConfig,
    detector: &RoadDetector,
    plant: PlantState,
    at: u64,
    light: LightState,
    obstacles: &[ObstacleDisc],
) -> Result<SensorSnapshot> {
    let at = Timestamp(at);
    let camera = render_camera(config, plant.pose, light)?;
    Ok(SensorSnapshot {
        at,
        pose: PoseEstimate {
            captured_at: at,
            frame_id: config.autonomy.mission.world_frame.clone(),
            pose: plant.pose,
            speed_mps: plant.speed_mps,
            yaw_rate_radps: plant.speed_mps * plant.curvature_per_m,
            quality: 1.0,
        },
        scan: synthetic_scan(config, plant.pose, obstacles, at),
        road: RoadFrame {
            observation: detector.detect(
                &camera,
                &[],
                at,
                config.autonomy.mission.body_frame.clone(),
            )?,
            image_width_px: camera.width(),
            image_height_px: camera.height(),
        },
    })
}

fn submit(worker: &AutonomyWorker, snapshot: &Arc<SensorSnapshot>, now: u64) -> Result<()> {
    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        match worker.try_submit(Arc::clone(snapshot), Timestamp(now))? {
            SubmitStatus::Queued => return Ok(()),
            SubmitStatus::Busy if Instant::now() < deadline => thread::yield_now(),
            status => return Err(format!("functional submission {status:?} at {now}")),
        }
    }
}

fn wait_published(worker: &mut AutonomyWorker, expected: &Awaiting) -> Result<ControlPoll> {
    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        let poll = worker.poll(Timestamp(expected.adoption_at));
        if poll.fault.is_some()
            || poll
                .observed_plan
                .as_ref()
                .is_some_and(|plan| plan.planned_at == Timestamp(expected.planned_at))
        {
            return Ok(poll);
        }
        if Instant::now() >= deadline {
            return Err(format!(
                "host planner did not publish source {} in bounded functional wait",
                expected.snapshot.at.0
            ));
        }
        thread::sleep(Duration::from_micros(100));
    }
}

/// Run a full functional competition, recording failures without relaxing any
/// controller gate. Runtime failures return a summary after complete braking;
/// invalid startup configuration and final output errors return Err.
pub fn simulate_async(
    config: &SimulationConfig,
    timing: &AsyncSimulationOptions,
    writer: &mut impl Write,
) -> Result<AsyncSimulationSummary> {
    simulate_async_observed(config, timing, writer, |_, _, _, _| {})
}

/// Offline observation at each scheduled output, including final braking.
/// The callback runs on the simulation owner, outside the worker, and cannot
/// replace its command. It sees the model state before that output takes effect.
/// Callback time is included in host wall time; the functional clock is frozen.
/// Production worker/poll and the default simulation do not store this trace.
pub fn simulate_async_observed(
    config: &SimulationConfig,
    timing: &AsyncSimulationOptions,
    writer: &mut impl Write,
    mut observe: impl FnMut(&ControlPoll, Pose2, f64, f64),
) -> Result<AsyncSimulationSummary> {
    config.validate()?;
    timing.validate(config)?;
    let started = Instant::now();
    let detector = RoadDetector::new(config.road.clone())?;
    let runtime = ControlRuntimeConfig {
        max_command_age_ms: config.autonomy.max_sensor_age_ms,
        startup_timeout_ms: config.autonomy.max_sensor_age_ms,
    };
    let mut worker = if timing.measure_wall_time {
        AutonomyWorker::spawn_with_diagnostics(
            config.autonomy.clone(),
            runtime,
            Timestamp(0),
            WorkerDiagnosticsOptions {
                measure_wall_time: true,
                schedule_hook: None,
            },
        )?
    } else {
        AutonomyWorker::spawn(config.autonomy.clone(), runtime, Timestamp(0))?
    };
    let mut observation = Observation::new(config)?;
    let mut plant = PlantState {
        pose: config.initial_pose,
        speed_mps: 0.0,
        curvature_per_m: 0.0,
    };
    let dynamics = PlantDynamics {
        acceleration_mps2: config.plant_accel_mps2,
        braking_mps2: config.plant_brake_mps2,
        curvature_rate_per_s: config.autonomy.navigation.max_curvature_rate_per_s,
    };
    let mut command = MotionOutput::Stop;
    let mut pending = VecDeque::<Pending>::with_capacity(2);
    let mut awaiting: Option<Awaiting> = None;
    let mut previous: Option<Arc<SensorSnapshot>> = None;
    let mut last_submitted: Option<Arc<SensorSnapshot>> = None;
    let mut light_trigger = None;
    let light_boundary = config
        .autonomy
        .mission
        .light_stop_boundary()
        .map_err(|e| e.to_string())?;
    let mut obstacles = config.cones.clone();
    if config.fault == SimulationFault::Blocked {
        obstacles.extend((0..26).map(|index| ObstacleDisc {
            center: xt_stcar_robot_core::autonomy::Point2 {
                x_m: 2.65,
                y_m: index as f64 * 0.2,
            },
            radius_m: 0.22,
        }));
    }
    let mut minimum = obstacles
        .iter()
        .map(|cone| obstacle_clearance(plant.pose, config.autonomy.mission.footprint, *cone))
        .fold(12.0f64, f64::min);
    let mut distance = 0.0;
    let mut now = 0;
    while now <= config.max_duration_ms {
        if light_trigger.is_none() && crate::field::light_triggered(config, plant.pose) {
            light_trigger = Some(now);
        }
        if now.is_multiple_of(timing.capture_period_ms)
            && timing.stop_input_at_ms.is_none_or(|at| now < at)
        {
            let light = if config.fault == SimulationFault::RedOnly
                || light_trigger.is_some_and(|at| now - at < config.light_red_duration_ms)
            {
                LightState::Red
            } else {
                LightState::Green
            };
            match capture(config, &detector, plant, now, light, &obstacles) {
                Ok(mut snapshot) => {
                    if now >= config.fault_at_ms
                        && let Some(old) = &previous
                    {
                        match config.fault {
                            SimulationFault::PoseDropout => snapshot.pose = old.pose.clone(),
                            SimulationFault::LidarDropout => snapshot.scan = old.scan.clone(),
                            SimulationFault::CameraDropout => snapshot.road = old.road.clone(),
                            _ => {}
                        }
                    }
                    let snapshot = Arc::new(snapshot);
                    let delay = timing.input_delays_ms[observation.counts.captures as usize % 2];
                    observation.counts.captures += 1;
                    previous = Some(Arc::clone(&snapshot));
                    pending.push_back(Pending {
                        delivery_at: now + delay,
                        snapshot,
                    });
                    debug_assert!(pending.len() <= 2);
                }
                Err(error) => {
                    observation.fault = Some(format!("synthetic capture: {error}"));
                    break;
                }
            }
        }
        // Poll before publishing a same-time input, so the configured adoption
        // offset cannot disappear merely because the host planner finished fast.
        if now.is_multiple_of(timing.output_period_ms) {
            let poll = worker.poll(Timestamp(now));
            observation.poll(&poll, plant, last_submitted.as_deref(), light_boundary);
            observe(&poll, plant.pose, plant.speed_mps, plant.curvature_per_m);
            command = poll.command;
        }
        if let Some(input) = pending.pop_front_if(|input| input.delivery_at == now) {
            if let Err(error) = submit(&worker, &input.snapshot, now) {
                observation.fault = Some(error);
                break;
            }
            let delay = timing.adoption_delays_ms[observation.counts.submissions as usize % 3];
            observation.counts.submissions += 1;
            last_submitted = Some(Arc::clone(&input.snapshot));
            awaiting = Some(Awaiting {
                planned_at: now,
                adoption_at: now + delay,
                snapshot: input.snapshot,
            });
        }
        if awaiting
            .as_ref()
            .is_some_and(|input| input.adoption_at == now)
        {
            let input = awaiting.take().expect("due adoption");
            match wait_published(&mut worker, &input) {
                Ok(poll) => {
                    observation.poll(&poll, plant, Some(&input.snapshot), light_boundary);
                    observe(&poll, plant.pose, plant.speed_mps, plant.curvature_per_m);
                    command = poll.command;
                }
                Err(error) => {
                    observation.fault = Some(error);
                    break;
                }
            }
        }
        if observation.fault.is_some() || observation.completed || now == config.max_duration_ms {
            break;
        }
        let mut violation = false;
        let advanced = advance_plant(plant, &command, dynamics, 0.001, |pose| {
            violation |= check_plant_pose(
                config,
                pose,
                &obstacles,
                observation.active_boundary,
                &mut minimum,
            );
        });
        plant = advanced.state;
        distance += advanced.distance_m;
        observation
            .statistics
            .advance_control_interval(1, advanced.distance_m);
        now += 1;
        if violation {
            observation.fault = Some("synthetic async collision or boundary violation".into());
            break;
        }
    }
    if observation.fault.is_none() && !observation.completed {
        observation.fault = Some("async scenario duration exhausted before completion".into());
    }
    // Explicit scenario shutdown still goes through the worker's final Stop;
    // neither a backend Drive nor a cached report can reach final braking.
    worker.request_stop();
    let poll = worker.poll(Timestamp(now));
    observe(&poll, plant.pose, plant.speed_mps, plant.curvature_per_m);
    let mut terminal = poll.command;
    let cause = if observation.completed {
        FinalBrakingCause::Completion
    } else {
        FinalBrakingCause::FaultOrScenarioEnd
    };
    for _ in 0..5000 {
        if plant.speed_mps == 0.0 && plant.curvature_per_m == 0.0 {
            break;
        }
        let mut violation = false;
        let advanced = advance_plant(
            plant,
            &terminal,
            dynamics,
            timing.output_period_ms as f64 / 1000.0,
            |pose| {
                violation |= check_plant_pose(
                    config,
                    pose,
                    &obstacles,
                    observation.active_boundary,
                    &mut minimum,
                );
            },
        );
        plant = advanced.state;
        distance += advanced.distance_m;
        observation.statistics.advance_final_braking(
            Timestamp(now),
            cause,
            timing.output_period_ms,
            advanced.distance_m,
        );
        now += timing.output_period_ms;
        let poll = worker.poll(Timestamp(now));
        observe(&poll, plant.pose, plant.speed_mps, plant.curvature_per_m);
        terminal = poll.command;
        if violation {
            observation.fault.get_or_insert_with(|| {
                "synthetic async final braking collision or boundary violation".into()
            });
        }
    }
    if plant.speed_mps != 0.0 || plant.curvature_per_m != 0.0 {
        observation.fault.get_or_insert_with(|| {
            "synthetic async plant did not fully stop/recenter within bounded rollout".into()
        });
    }
    if observation.completed {
        let mission = &config.autonomy.mission;
        let heading = (plant.pose.yaw_rad - mission.finish_yaw_rad + std::f64::consts::PI)
            .rem_euclid(std::f64::consts::TAU)
            - std::f64::consts::PI;
        if !mission.footprint.inside(plant.pose, mission.finish_region)
            || heading.abs() > mission.goal_heading_tolerance_rad
        {
            observation.fault.get_or_insert_with(|| "observed completion did not retain the true final footprint/heading after braking".into());
        }
    }
    let summary = AsyncSimulationSummary {
        kind: "async_simulation_summary",
        physical_output_enabled: false,
        clock_scope: "Deterministic functional clock; host planner waits freeze model time. Ideal pose/RGB/raycast inputs, no target deadline or physical acceptance claim.",
        timing: timing.clone(),
        completed: observation.completed && observation.fault.is_none(),
        elapsed_ms: now,
        wall_time_ms: started.elapsed().as_secs_f64() * 1000.0,
        distance_m: distance,
        minimum_cone_clearance_m: minimum,
        crosswalk_hold_ms: observation.held,
        green_observed_ms: observation.green,
        phases: observation.phases,
        final_pose: plant.pose,
        final_actual_speed_mps: plant.speed_mps,
        final_actual_curvature_per_m: plant.curvature_per_m,
        terminal_command: terminal,
        fault: observation.fault,
        counts: observation.counts,
        statistics: observation.statistics.finish(),
        first_problem: observation.first_problem,
        last_control: observation.last_control,
        recent_attempts: observation.recent_attempts,
        timing_statistics: observation.timing_statistics,
        transitions: observation.transitions,
        omitted_transitions: observation.omitted_transitions,
        dropped_trace_records: observation.journal.dropped_records(),
        logging_errors: observation.logging_errors,
    };
    observation
        .journal
        .write_to(writer)
        .map_err(|e| e.to_string())?;
    serde_json::to_writer(&mut *writer, &summary).map_err(|e| e.to_string())?;
    writer.write_all(b"\n").map_err(|e| e.to_string())?;
    Ok(summary)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::autonomy::AutonomyStep;
    use crate::control_diagnostics::ObservedPlan;
    use xt_stcar_robot_core::{OutputRecord, StepReport};

    #[test]
    fn late_adoption_of_observed_plan_is_counted_once_and_records_actual_transition() {
        let config = SimulationConfig::example();
        let mut observation = Observation::new(&config).unwrap();
        let plant = PlantState {
            pose: config.initial_pose,
            speed_mps: 0.0,
            curvature_per_m: 0.0,
        };
        let drive = MotionOutput::Drive {
            speed_mps: 0.04,
            curvature_per_m: 0.0,
        };
        let report = Arc::new(AutonomyStep {
            kind: "autonomy_step",
            mode: "observation_fixture",
            physical_output_enabled: false,
            at: Timestamp(60),
            mission: None,
            navigation: None,
            safety: StepReport {
                event_at: Timestamp(60),
                previous_state: State::Running,
                state: State::Running,
                output: OutputRecord {
                    at: Timestamp(60),
                    state: State::Running,
                    command: drive.clone(),
                    reason: None,
                },
                emergency_stop_latched: false,
            },
            command: drive.clone(),
            fault: None,
        });
        let boundary = config.autonomy.mission.light_stop_boundary().unwrap();
        let mut poll = ControlPoll {
            at: Timestamp(63),
            command: MotionOutput::Stop,
            fault: None,
            adoption_rejection: Some(AdoptionRejection::CurvatureSlew),
            observed_plan: Some(ObservedPlan {
                certificate_failure: None,
                source_at: Timestamp(0),
                oldest_sensor_at: Timestamp(0),
                planned_at: Timestamp(60),
                newly_adopted: false,
                source_age_ms: 63,
                plan_age_ms: 3,
                observed_host_ns: Some(130),
                timings: Some(Arc::new(PlanTimings {
                    published_host_ns: 100,
                    ..Default::default()
                })),
                report: Arc::clone(&report),
            }),
            latest: None,
        };
        observation.poll(&poll, plant, None, boundary);
        poll.at = Timestamp(80);
        poll.command = drive.clone();
        poll.adoption_rejection = None;
        poll.latest = Some(report);
        let plan = poll.observed_plan.as_mut().unwrap();
        plan.newly_adopted = true;
        plan.source_age_ms = 80;
        plan.plan_age_ms = 20;
        plan.observed_host_ns = Some(180);
        observation.poll(&poll, plant, None, boundary);
        poll.at = Timestamp(100);
        poll.observed_plan.as_mut().unwrap().newly_adopted = false;
        observation.poll(&poll, plant, None, boundary);

        assert_eq!(observation.counts.completed_plans, 1);
        assert_eq!(observation.counts.newly_adopted_plans, 1);
        assert_eq!(observation.timing_statistics.processor_ns.samples, 1);
        assert_eq!(
            observation.timing_statistics.adopted_source_age_ms.total,
            80
        );
        assert_eq!(observation.timing_statistics.adopted_plan_age_ms.total, 20);
        assert_eq!(
            observation
                .timing_statistics
                .publication_to_adoption_ns
                .total,
            80
        );
        assert_eq!(observation.transitions.len(), 2);
        assert_eq!(observation.transitions[1].at, Timestamp(80));
        assert!(!observation.transitions[1].state.stopped);
        let attempt = &observation.recent_attempts[0];
        assert!(!attempt.newly_adopted);
        assert_eq!(attempt.adopted_at, Some(Timestamp(80)));
        assert_eq!(attempt.adopted_command, Some(drive));
        assert_eq!(attempt.publication_to_adoption_ns, Some(80));
    }
}
