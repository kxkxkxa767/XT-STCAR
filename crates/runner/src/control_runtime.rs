//! Memory-only output watchdog, independent of background navigation.
//!
//! The caller must run `poll` on its own periodic control thread and honor its
//! `command`, never issue `latest.command` directly. This module does not schedule
//! physical output or provide a hardware watchdog. All timestamps share the caller's
//! monotonic session clock; completion time and wall-clock time are never substituted.
//! Drop requests stop and detaches: a hung processor can retain its active snapshot
//! until process exit. Repeatedly spawning replacements for a hung runtime is unsafe.
//! Steering estimates follow only the final command returned by `poll`. A submitted
//! snapshot binds a fixed-capacity history of commands actually adopted by poll.
//! The real controller projects the source pose to submission time, preserving the
//! original measurements for task admission, obstacle projection and Safety. A
//! static-world stopping certificate permits adoption within one control period,
//! provided the assumed held command has not changed. Missing history, computation
//! beyond that window, and changed assumptions never authorize a delayed Drive.
//! These remain model estimates: output polling must meet the configured control
//! period, and physical feedback, moving obstacles and model errors need separate
//! commissioning. The original sensor lease is checked before every adoption.
use crate::autonomy::{AutonomyConfig, AutonomyController, AutonomyStep, Result};
use crate::autonomy_replay::SensorSnapshot;
pub use crate::control_execution::PlanningContext;
use crate::control_execution::{AdoptionCertificate, AtomicExecution, ExecutionHistory, certify};
use serde::{Deserialize, Serialize};
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use std::sync::mpsc::{SyncSender, sync_channel};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use xt_stcar_robot_core::mission::MissionPhase;
use xt_stcar_robot_core::navigation::SteeringEstimate;
use xt_stcar_robot_core::{MotionOutput, State, Timestamp};

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct ControlRuntimeConfig {
    pub max_command_age_ms: u64,
    pub startup_timeout_ms: u64,
}

impl ControlRuntimeConfig {
    fn validate(self) -> Result<Self> {
        if self.max_command_age_ms == 0 || self.startup_timeout_ms == 0 {
            return Err("control command age and startup timeout must be positive".into());
        }
        Ok(self)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
#[repr(u8)]
pub enum ControlFault {
    StartupExpired = 1,
    CommandExpired,
    ClockRegression,
    InvalidInput,
    InvalidResult,
    ControllerFault,
    ProcessorFailed,
    ProcessorPanicked,
    MailboxPoisoned,
    Stopped,
}

impl ControlFault {
    fn from_code(code: u8) -> Option<Self> {
        match code {
            0 => None,
            1 => Some(Self::StartupExpired),
            2 => Some(Self::CommandExpired),
            3 => Some(Self::ClockRegression),
            4 => Some(Self::InvalidInput),
            5 => Some(Self::InvalidResult),
            6 => Some(Self::ControllerFault),
            7 => Some(Self::ProcessorFailed),
            8 => Some(Self::ProcessorPanicked),
            9 => Some(Self::MailboxPoisoned),
            _ => Some(Self::Stopped),
        }
    }
}

/// Small envelope; the complete planner report is shared without deep cloning.
#[derive(Clone, Debug)]
pub struct PlannedCommand {
    pub source_at: Timestamp,
    pub oldest_sensor_at: Timestamp,
    /// Model planning time, distinct from the immutable source capture time.
    pub planned_at: Timestamp,
    pub step: Arc<AutonomyStep>,
}

#[derive(Debug)]
pub struct ControlPoll {
    pub at: Timestamp,
    pub command: MotionOutput,
    pub fault: Option<ControlFault>,
    pub adoption_rejection: Option<AdoptionRejection>,
    /// Diagnostic only: a latched watchdog may override this report's Drive.
    pub latest: Option<Arc<AutonomyStep>>,
}

pub struct ControlWatchdog {
    config: ControlRuntimeConfig,
    epoch: Timestamp,
    last_poll: Timestamp,
    accepted: Option<PlannedCommand>,
    fault: Option<ControlFault>,
}

impl ControlWatchdog {
    pub fn new(config: ControlRuntimeConfig, epoch: Timestamp) -> Result<Self> {
        Ok(Self {
            config: config.validate()?,
            epoch,
            last_poll: epoch,
            accepted: None,
            fault: None,
        })
    }

    fn latch(&mut self, fault: ControlFault) {
        self.fault.get_or_insert(fault);
    }

    /// Repeated reports must retain their Arc identity and source metadata. An
    /// unchanged report never renews its lease; a new report needs a newer source_at.
    pub fn poll(&mut self, now: Timestamp, latest: Option<PlannedCommand>) -> ControlPoll {
        if now < self.last_poll {
            self.latch(ControlFault::ClockRegression);
        } else {
            self.last_poll = now;
        }
        // Expire the previously issued command before considering any late result.
        if self.fault.is_none() {
            if let Some(old) = &self.accepted {
                if now.0 - old.oldest_sensor_at.0 >= self.config.max_command_age_ms {
                    self.latch(ControlFault::CommandExpired);
                }
            } else if now.0 - self.epoch.0 >= self.config.startup_timeout_ms {
                self.latch(ControlFault::StartupExpired);
            }
        }
        if self.fault.is_none()
            && let Some(latest) = latest
        {
            self.accept(now, latest);
        }
        ControlPoll {
            at: now,
            command: if self.fault.is_none() {
                self.accepted
                    .as_ref()
                    .map_or(MotionOutput::Stop, |value| value.step.command.clone())
            } else {
                MotionOutput::Stop
            },
            fault: self.fault,
            adoption_rejection: None,
            latest: self.accepted.as_ref().map(|value| Arc::clone(&value.step)),
        }
    }

    fn accept(&mut self, now: Timestamp, latest: PlannedCommand) {
        if latest.source_at < self.epoch
            || latest.source_at > latest.planned_at
            || latest.planned_at > now
            || latest.oldest_sensor_at > latest.source_at
        {
            self.latch(ControlFault::InvalidResult);
            return;
        }
        if let Some(old) = &self.accepted {
            if latest.source_at < old.source_at
                || latest.oldest_sensor_at < old.oldest_sensor_at
                || (latest.source_at == old.source_at
                    && (!Arc::ptr_eq(&latest.step, &old.step)
                        || latest.oldest_sensor_at != old.oldest_sensor_at
                        || latest.planned_at != old.planned_at))
            {
                self.latch(ControlFault::InvalidResult);
                return;
            }
            if latest.source_at == old.source_at {
                return;
            }
        }
        if now.0 - latest.oldest_sensor_at.0 >= self.config.max_command_age_ms {
            self.latch(ControlFault::CommandExpired);
            return;
        }
        if let Some(fault) = result_fault(&latest.step, latest.planned_at) {
            self.latch(fault);
            return;
        }
        self.accepted = Some(latest);
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SubmitStatus {
    Queued,
    Replaced,
    Duplicate,
    Busy,
    /// Reserved for compatibility; source lookup now uses bounded history.
    ExecutionAhead,
    /// Required source time has fallen out of the fixed adopted-command history.
    HistoryUnavailable,
    Stopped,
}

#[derive(Clone, Copy)]
struct SteeringLimits {
    rate: f64,
    max_curvature: f64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum AdoptionRejection {
    BeforeWindow,
    AfterWindow,
    ExecutionChanged,
    CurvatureSlew,
    CertificateUnsafe,
}

#[derive(Clone)]
struct WorkerPlan {
    command: PlannedCommand,
    certificate: Option<AdoptionCertificate>,
    admission_rejected: bool,
}

struct PendingInput {
    snapshot: Arc<SensorSnapshot>,
    execution: Option<SteeringEstimate>,
    history: Option<ExecutionHistory>,
    planned_at: Timestamp,
}

#[derive(Default)]
struct Slots {
    pending: Option<PendingInput>,
    // This aliases active/pending storage, never a third independently owned snapshot.
    previous: Option<Arc<SensorSnapshot>>,
    latest: Option<WorkerPlan>,
    last_plan_at: Option<Timestamp>,
    last_submit_now: Option<Timestamp>,
}

struct Shared {
    slots: Mutex<Slots>,
    execution: Option<AtomicExecution>,
    stopped: AtomicBool,
    fault: AtomicU8,
}

impl Shared {
    fn fail(&self, fault: ControlFault) {
        let _ = self
            .fault
            .compare_exchange(0, fault as u8, Ordering::AcqRel, Ordering::Acquire);
        self.stopped.store(true, Ordering::Release);
    }
}

/// Clone this small handle for the input thread. Polling remains owned by the
/// independent output thread and does not require an outer shared worker mutex.
#[derive(Clone)]
pub struct AutonomySubmitter {
    shared: Arc<Shared>,
    wake: SyncSender<()>,
    epoch: Timestamp,
    max_command_age_ms: u64,
    steering_limits: Option<SteeringLimits>,
    aligned: bool,
}

impl AutonomySubmitter {
    pub fn try_submit(&self, input: Arc<SensorSnapshot>, now: Timestamp) -> Result<SubmitStatus> {
        if self.shared.stopped.load(Ordering::Acquire) {
            return Ok(SubmitStatus::Stopped);
        }
        if !valid_snapshot(&input, self.epoch, now, self.max_command_age_ms) {
            self.shared.fail(ControlFault::InvalidInput);
            let _ = self.wake.try_send(());
            return Err("control snapshot has invalid times, frames or storage bounds".into());
        }
        let mut slots = match self.shared.slots.try_lock() {
            Ok(slots) => slots,
            Err(std::sync::TryLockError::WouldBlock) => return Ok(SubmitStatus::Busy),
            Err(_) => {
                self.shared.fail(ControlFault::MailboxPoisoned);
                return Err("control mailbox is poisoned".into());
            }
        };
        if self.shared.stopped.load(Ordering::Acquire) {
            return Ok(SubmitStatus::Stopped);
        }
        if slots.last_submit_now.is_some_and(|old| now < old) {
            self.shared.fail(ControlFault::ClockRegression);
            let _ = self.wake.try_send(());
            return Err("control submission clock regressed".into());
        }
        if let Some(old) = &slots.previous {
            if input.at < old.at || changed_sample(&input, old) {
                self.shared.fail(ControlFault::InvalidInput);
                let _ = self.wake.try_send(());
                return Err(
                    "control source time regressed or duplicate sample contents changed".into(),
                );
            }
            if input.at == old.at {
                slots.last_submit_now = Some(now);
                return Ok(SubmitStatus::Duplicate);
            }
        }
        slots.last_submit_now = Some(now);
        if self.aligned && slots.last_plan_at.is_some_and(|at| now <= at) {
            return Ok(SubmitStatus::Busy);
        }
        let history = if self.steering_limits.is_some() {
            let Some(history) = self
                .shared
                .execution
                .as_ref()
                .and_then(AtomicExecution::read)
            else {
                return Ok(SubmitStatus::Busy);
            };
            if history.latest.steering.at > now {
                return Ok(SubmitStatus::Busy);
            }
            Some(history)
        } else {
            None
        };
        let execution = if let Some(limits) = self.steering_limits {
            let Some(state) = history.as_ref().and_then(|history| {
                history.steering_at(
                    if self.aligned {
                        input.pose.captured_at
                    } else {
                        input.at
                    },
                    limits.rate,
                )
            }) else {
                return Ok(SubmitStatus::HistoryUnavailable);
            };
            Some(state)
        } else {
            None
        };
        let planned_at = if self.aligned { now } else { input.at };
        slots.last_plan_at = Some(planned_at);
        let previous = slots.previous.replace(Arc::clone(&input));
        let retired = slots.pending.replace(PendingInput {
            snapshot: input,
            execution,
            history,
            planned_at,
        });
        let status = if retired.is_some() {
            SubmitStatus::Replaced
        } else {
            SubmitStatus::Queued
        };
        drop(slots);
        let _ = self.wake.try_send(());
        drop((previous, retired));
        Ok(status)
    }
}

pub struct AutonomyWorker {
    submitter: AutonomySubmitter,
    watchdog: ControlWatchdog,
    thread: Option<JoinHandle<()>>,
    execution: Option<SteeringEstimate>,
}

impl AutonomyWorker {
    /// Explicitly starts an offline controller session before spawning navigation.
    pub fn spawn(
        config: AutonomyConfig,
        runtime: ControlRuntimeConfig,
        epoch: Timestamp,
    ) -> Result<Self> {
        let mut controller = AutonomyController::new(config.clone())?;
        controller.start()?;
        Self::spawn_with_aligned_processor(config, runtime, epoch, move |input, context| {
            Ok(controller.tick_with_projection(
                input.at,
                &input.pose,
                &input.scan,
                &input.road,
                context,
            ))
        })
    }

    /// Trusted offline processor with a projected model state and a conservative
    /// static-world admission certificate. Inputs retain their capture timestamps.
    pub fn spawn_with_aligned_processor<F>(
        config: AutonomyConfig,
        runtime: ControlRuntimeConfig,
        epoch: Timestamp,
        mut processor: F,
    ) -> Result<Self>
    where
        F: FnMut(&SensorSnapshot, &PlanningContext) -> Result<AutonomyStep> + Send + 'static,
    {
        let controller_deadline = [
            config.max_tick_gap_ms,
            config.max_sensor_age_ms,
            config.safety.command_timeout_ms,
            config.safety.sensor_timeout_ms,
            config.safety.heartbeat_timeout_ms,
            config.safety.deadman_timeout_ms,
            config.mission.max_pose_age_ms,
            config.mission.max_road_age_ms,
            config.navigation.max_input_age_ms,
        ]
        .into_iter()
        .min()
        .unwrap();
        if runtime.max_command_age_ms > controller_deadline {
            return Err("output watchdog lease cannot exceed controller safety deadlines".into());
        }
        AutonomyController::new(config.clone())?;
        let limits = SteeringLimits {
            rate: config.navigation.max_curvature_rate_per_s,
            max_curvature: config.navigation.max_curvature_per_m,
        };
        Self::spawn_inner(
            runtime,
            epoch,
            Some(limits),
            Some(config),
            move |input, _, context| {
                processor(
                    input,
                    context.expect("aligned processor projects its source state"),
                )
            },
        )
    }

    /// Legacy trusted processor hook without a steering model. Input storage and
    /// output leases remain bounded. Use `spawn_with_execution_processor` when
    /// the processor needs the output owner's adopted-command history.
    pub fn spawn_with_processor<F>(
        runtime: ControlRuntimeConfig,
        epoch: Timestamp,
        mut processor: F,
    ) -> Result<Self>
    where
        F: FnMut(&SensorSnapshot) -> Result<AutonomyStep> + Send + 'static,
    {
        Self::spawn_inner(runtime, epoch, None, None, move |input, _, _| {
            processor(input)
        })
    }

    /// Trusted processor hook with explicit model limits. Estimates include only
    /// poll-adopted commands known at submission; planning does not acknowledge
    /// its own result. This trusted hook only recovers source-time steering; it
    /// does not project poses or certify adoption delay. Use `spawn` or
    /// `spawn_with_aligned_processor` for the complete aligned controller path.
    pub fn spawn_with_execution_processor<F>(
        runtime: ControlRuntimeConfig,
        epoch: Timestamp,
        max_curvature_rate_per_s: f64,
        max_curvature_per_m: f64,
        mut processor: F,
    ) -> Result<Self>
    where
        F: FnMut(&SensorSnapshot, SteeringEstimate) -> Result<AutonomyStep> + Send + 'static,
    {
        if !max_curvature_rate_per_s.is_finite()
            || max_curvature_rate_per_s <= 0.0
            || max_curvature_rate_per_s > 50.0
            || !(1e-6..=10.0).contains(&max_curvature_per_m)
        {
            return Err("control steering model requires bounded positive limits".into());
        }
        let limits = SteeringLimits {
            rate: max_curvature_rate_per_s,
            max_curvature: max_curvature_per_m,
        };
        Self::spawn_inner(
            runtime,
            epoch,
            Some(limits),
            None,
            move |input, steering, _| {
                processor(
                    input,
                    steering.expect("execution processor binds steering at submission"),
                )
            },
        )
    }

    fn spawn_inner<F>(
        runtime: ControlRuntimeConfig,
        epoch: Timestamp,
        steering_limits: Option<SteeringLimits>,
        alignment: Option<AutonomyConfig>,
        mut processor: F,
    ) -> Result<Self>
    where
        F: FnMut(
                &SensorSnapshot,
                Option<SteeringEstimate>,
                Option<&PlanningContext>,
            ) -> Result<AutonomyStep>
            + Send
            + 'static,
    {
        let watchdog = ControlWatchdog::new(runtime, epoch)?;
        let execution = steering_limits.map(|_| SteeringEstimate::stationary(epoch));
        let shared = Arc::new(Shared {
            slots: Mutex::new(Slots::default()),
            execution: execution.map(AtomicExecution::new),
            stopped: AtomicBool::new(false),
            fault: AtomicU8::new(0),
        });
        let worker_shared = Arc::clone(&shared);
        let aligned = alignment.is_some();
        let (wake, receiver) = sync_channel(1);
        let thread = thread::Builder::new()
            .name("autonomy-planner".into())
            .spawn(move || {
                while receiver.recv().is_ok() {
                    if worker_shared.stopped.load(Ordering::Acquire) {
                        break;
                    }
                    let input = match worker_shared.slots.lock() {
                        Ok(mut slots) => slots.pending.take(),
                        Err(_) => {
                            worker_shared.fail(ControlFault::MailboxPoisoned);
                            break;
                        }
                    };
                    let Some(input) = input else { continue };
                    let context = if let Some(config) = &alignment {
                        let Some(context) = input.history.as_ref().and_then(|history| {
                            history.project(&input.snapshot, input.planned_at, config)
                        }) else {
                            worker_shared.fail(ControlFault::InvalidInput);
                            break;
                        };
                        Some(context)
                    } else {
                        None
                    };
                    let mut step = match catch_unwind(AssertUnwindSafe(|| {
                        processor(&input.snapshot, input.execution, context.as_ref())
                    })) {
                        Ok(Ok(step)) => step,
                        Ok(Err(_)) => {
                            worker_shared.fail(ControlFault::ProcessorFailed);
                            break;
                        }
                        Err(_) => {
                            worker_shared.fail(ControlFault::ProcessorPanicked);
                            break;
                        }
                    };
                    if worker_shared.stopped.load(Ordering::Acquire) {
                        break;
                    }
                    let original_fault = result_fault(&step, input.planned_at);
                    let drive = matches!(step.command, MotionOutput::Drive { .. });
                    let certificate = alignment
                        .as_ref()
                        .zip(context.as_ref())
                        .filter(|_| drive)
                        .and_then(|(config, context)| {
                            certify(
                                config,
                                &input.snapshot,
                                context,
                                &step,
                                oldest(&input.snapshot),
                                runtime.max_command_age_ms,
                            )
                        });
                    let admission_rejected = aligned && drive && certificate.is_none();
                    if admission_rejected {
                        // A valid plan may be too close to an obstacle for the wider
                        // asynchronous lease. Stop does not claim the rejected Drive ran.
                        step.command = MotionOutput::Stop;
                        step.safety.output.command = MotionOutput::Stop;
                    }
                    let command = PlannedCommand {
                        source_at: input.snapshot.at,
                        oldest_sensor_at: oldest(&input.snapshot),
                        planned_at: input.planned_at,
                        step: Arc::new(step),
                    };
                    let fault =
                        original_fault.or_else(|| result_fault(&command.step, input.planned_at));
                    let latest = WorkerPlan {
                        command,
                        certificate,
                        admission_rejected,
                    };
                    let retired = match worker_shared.slots.lock() {
                        Ok(mut slots) => {
                            let old = slots.latest.replace(latest);
                            if let Some(fault) = fault {
                                worker_shared.fail(fault);
                            }
                            old
                        }
                        Err(_) => {
                            worker_shared.fail(ControlFault::MailboxPoisoned);
                            break;
                        }
                    };
                    drop(retired);
                    if fault.is_some() {
                        break;
                    }
                }
                worker_shared.stopped.store(true, Ordering::Release);
                let retired = worker_shared
                    .slots
                    .lock()
                    .ok()
                    .map(|mut slots| (slots.pending.take(), slots.previous.take()));
                drop(retired);
            })
            .map_err(|error| format!("start autonomy worker: {error}"))?;
        Ok(Self {
            submitter: AutonomySubmitter {
                shared,
                wake,
                epoch,
                max_command_age_ms: runtime.max_command_age_ms,
                steering_limits,
                aligned,
            },
            watchdog,
            thread: Some(thread),
            execution,
        })
    }

    pub fn submitter(&self) -> AutonomySubmitter {
        self.submitter.clone()
    }

    pub fn try_submit(&self, input: Arc<SensorSnapshot>, now: Timestamp) -> Result<SubmitStatus> {
        self.submitter.try_submit(input, now)
    }

    /// Output-owner model after the latest poll, never measured actuator state.
    /// The legacy processor hook has no model and returns None.
    pub fn execution_state(&self) -> Option<SteeringEstimate> {
        self.execution
    }

    /// No waiting, navigation, model calls, disk access or large report cloning.
    /// A busy mailbox retains the previous command only within its original lease.
    pub fn poll(&mut self, now: Timestamp) -> ControlPoll {
        let mut latest = match self.submitter.shared.slots.try_lock() {
            Ok(slots) => slots.latest.clone(),
            Err(std::sync::TryLockError::WouldBlock) => None,
            Err(_) => {
                self.submitter.shared.fail(ControlFault::MailboxPoisoned);
                None
            }
        };
        if let Some(fault) =
            ControlFault::from_code(self.submitter.shared.fault.load(Ordering::Acquire))
        {
            self.watchdog.latch(fault);
        }
        let mut rejection = latest.as_ref().and_then(|plan| {
            plan.admission_rejected
                .then_some(AdoptionRejection::CertificateUnsafe)
        });
        if let Some(plan) = &latest
            && let Some(certificate) = plan.certificate
            && self
                .watchdog
                .accepted
                .as_ref()
                .is_none_or(|old| !Arc::ptr_eq(&old.step, &plan.command.step))
        {
            rejection = if now < certificate.from {
                Some(AdoptionRejection::BeforeWindow)
            } else if now > certificate.through {
                Some(AdoptionRejection::AfterWindow)
            } else if self
                .submitter
                .shared
                .execution
                .as_ref()
                .is_some_and(|execution| execution.revision() != certificate.revision)
            {
                Some(AdoptionRejection::ExecutionChanged)
            } else if certificate.revision > 0
                && self
                    .submitter
                    .shared
                    .execution
                    .as_ref()
                    .is_some_and(|execution| {
                        let previous = execution.last_change();
                        let MotionOutput::Drive {
                            curvature_per_m, ..
                        } = plan.command.step.command
                        else {
                            return false;
                        };
                        let elapsed_s =
                            now.0.saturating_sub(previous.steering.at.0) as f64 / 1000.0;
                        (curvature_per_m - previous.steering.commanded_curvature_per_m).abs()
                            > self
                                .submitter
                                .steering_limits
                                .expect("certificate has steering limits")
                                .rate
                                * elapsed_s
                                + 1e-9
                    })
            {
                Some(AdoptionRejection::CurvatureSlew)
            } else {
                None
            };
            if rejection.is_some() {
                latest = None;
            }
        }
        let mut result = self.watchdog.poll(now, latest.map(|plan| plan.command));
        result.adoption_rejection = rejection;
        self.acknowledge(&mut result);
        if let Some(fault) = result.fault {
            self.submitter.shared.fail(fault);
            self.stop_background();
        }
        result
    }

    fn acknowledge(&mut self, result: &mut ControlPoll) {
        let Some(old) = self.execution else { return };
        let limits = self
            .submitter
            .steering_limits
            .expect("execution model has validated limits");
        let mut next = old;
        if next
            .adopt(
                result.at,
                &result.command,
                limits.rate,
                limits.max_curvature,
            )
            .is_err()
        {
            // Never publish or return a command the model rejects. In particular,
            // a regressing clock cannot rewind applied curvature; latch Stop at
            // the last valid model time while preserving the original fault.
            self.watchdog.latch(ControlFault::InvalidResult);
            result.fault = self.watchdog.fault;
            result.command = MotionOutput::Stop;
            next = old;
            next.adopt(
                result.at.max(old.at),
                &MotionOutput::Stop,
                limits.rate,
                limits.max_curvature,
            )
            .expect("validated model can always adopt Stop without rewinding");
        }
        self.execution = Some(next);
        self.submitter
            .shared
            .execution
            .as_ref()
            .expect("execution model has a fixed-size publication slot")
            .publish(next, &result.command);
    }

    pub fn request_stop(&self) {
        self.submitter.shared.fail(ControlFault::Stopped);
        self.stop_background();
    }

    fn stop_background(&self) {
        self.submitter.shared.stopped.store(true, Ordering::Release);
        let retired = self
            .submitter
            .shared
            .slots
            .try_lock()
            .ok()
            .map(|mut slots| (slots.pending.take(), slots.previous.take()));
        let _ = self.submitter.wake.try_send(());
        drop(retired);
    }
}

impl Drop for AutonomyWorker {
    fn drop(&mut self) {
        self.request_stop();
        drop(self.thread.take());
    }
}

fn oldest(input: &SensorSnapshot) -> Timestamp {
    input
        .at
        .min(input.pose.captured_at)
        .min(input.scan.captured_at)
        .min(input.road.observation.captured_at)
}

fn result_fault(step: &AutonomyStep, at: Timestamp) -> Option<ControlFault> {
    if step.at != at || step.physical_output_enabled {
        return Some(ControlFault::InvalidResult);
    }
    if step.fault.is_some()
        || step.safety.state == State::Fault
        || step.safety.output.state == State::Fault
        || step.safety.emergency_stop_latched
        || step
            .mission
            .as_ref()
            .is_some_and(|mission| mission.phase == MissionPhase::Fault)
    {
        return Some(ControlFault::ControllerFault);
    }
    if let MotionOutput::Drive {
        speed_mps,
        curvature_per_m,
    } = step.command
        && (!speed_mps.is_finite()
            || !curvature_per_m.is_finite()
            || step.safety.state != State::Running
            || step.safety.output.state != step.safety.state
            || step.safety.output.command != step.command
            || step.safety.output.at != step.at
            || step.safety.event_at != step.at)
    {
        return Some(ControlFault::InvalidResult);
    }
    None
}

fn valid_snapshot(input: &SensorSnapshot, epoch: Timestamp, now: Timestamp, max_age: u64) -> bool {
    let road = &input.road;
    input.at >= epoch
        && input.at <= now
        && [
            input.pose.captured_at,
            input.scan.captured_at,
            road.observation.captured_at,
        ]
        .iter()
        .all(|stamp| *stamp <= input.at)
        && now.0 - oldest(input).0 < max_age
        && input.scan.ranges_m.len() <= 1440
        && input.scan.ranges_m.capacity() <= 1440
        && road.observation.cones_body_m.len() <= 256
        && road.observation.cones_body_m.capacity() <= 256
        && road.image_width_px > 0
        && road.image_height_px > 0
        && u64::from(road.image_width_px) * u64::from(road.image_height_px) <= 64_000_000
        && input.pose.frame_id.validate().is_ok()
        && input.scan.frame_id.validate().is_ok()
        && road.observation.frame_id.validate().is_ok()
}

fn changed_sample(new: &SensorSnapshot, old: &SensorSnapshot) -> bool {
    let new_road = &new.road;
    let old_road = &old.road;
    new.pose.captured_at < old.pose.captured_at
        || new.scan.captured_at < old.scan.captured_at
        || new_road.observation.captured_at < old_road.observation.captured_at
        || (new.pose.captured_at == old.pose.captured_at && new.pose != old.pose)
        || (new.scan.captured_at == old.scan.captured_at && new.scan != old.scan)
        || (new_road.observation.captured_at == old_road.observation.captured_at
            && (new_road.observation != old_road.observation
                || new_road.image_width_px != old_road.image_width_px
                || new_road.image_height_px != old_road.image_height_px))
        || (new.at == old.at
            && (new.pose != old.pose
                || new.scan != old.scan
                || new_road.observation != old_road.observation
                || new_road.image_width_px != old_road.image_width_px
                || new_road.image_height_px != old_road.image_height_px))
}
