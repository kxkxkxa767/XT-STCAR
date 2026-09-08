//! Memory-only output watchdog, independent of background navigation.
//!
//! The caller must run `poll` on its own periodic control thread and honor its
//! `command`, never issue `latest.command` directly. This module does not schedule
//! physical output or provide a hardware watchdog. All timestamps share the caller's
//! monotonic session clock; completion time and wall-clock time are never substituted.
//! Drop requests stop and detaches: a hung processor can retain its active snapshot
//! until process exit. Repeatedly spawning replacements for a hung runtime is unsafe.
use crate::autonomy::{AutonomyConfig, AutonomyController, AutonomyStep, Result};
use crate::autonomy_replay::SensorSnapshot;
use serde::{Deserialize, Serialize};
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use std::sync::mpsc::{SyncSender, sync_channel};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use xt_stcar_robot_core::mission::MissionPhase;
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
    pub step: Arc<AutonomyStep>,
}

#[derive(Debug)]
pub struct ControlPoll {
    pub at: Timestamp,
    pub command: MotionOutput,
    pub fault: Option<ControlFault>,
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
            latest: self.accepted.as_ref().map(|value| Arc::clone(&value.step)),
        }
    }

    fn accept(&mut self, now: Timestamp, latest: PlannedCommand) {
        if latest.source_at < self.epoch
            || latest.source_at > now
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
                        || latest.oldest_sensor_at != old.oldest_sensor_at))
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
        if let Some(fault) = result_fault(&latest.step, latest.source_at) {
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
    Stopped,
}

#[derive(Default)]
struct Slots {
    pending: Option<Arc<SensorSnapshot>>,
    // This aliases active/pending storage, never a third independently owned snapshot.
    previous: Option<Arc<SensorSnapshot>>,
    latest: Option<PlannedCommand>,
    last_submit_now: Option<Timestamp>,
}

struct Shared {
    slots: Mutex<Slots>,
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
        let previous = slots.previous.replace(Arc::clone(&input));
        let retired = slots.pending.replace(input);
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
}

impl AutonomyWorker {
    /// Explicitly starts an offline controller session before spawning navigation.
    pub fn spawn(
        config: AutonomyConfig,
        runtime: ControlRuntimeConfig,
        epoch: Timestamp,
    ) -> Result<Self> {
        let controller_deadline = [
            config.max_tick_gap_ms,
            config.max_sensor_age_ms,
            config.safety.command_timeout_ms,
            config.safety.sensor_timeout_ms,
            config.safety.heartbeat_timeout_ms,
            config.safety.deadman_timeout_ms,
        ]
        .into_iter()
        .min()
        .expect("controller has fixed deadline fields");
        if runtime.max_command_age_ms > controller_deadline {
            return Err("output watchdog lease cannot exceed controller safety deadlines".into());
        }
        let mut controller = AutonomyController::new(config)?;
        controller.start()?;
        Self::spawn_with_processor(runtime, epoch, move |input| {
            Ok(controller.tick(input.at, &input.pose, &input.scan, &input.road))
        })
    }

    /// Trusted in-process processor hook; input storage and output leases remain bounded.
    pub fn spawn_with_processor<F>(
        runtime: ControlRuntimeConfig,
        epoch: Timestamp,
        mut processor: F,
    ) -> Result<Self>
    where
        F: FnMut(&SensorSnapshot) -> Result<AutonomyStep> + Send + 'static,
    {
        let watchdog = ControlWatchdog::new(runtime, epoch)?;
        let shared = Arc::new(Shared {
            slots: Mutex::new(Slots::default()),
            stopped: AtomicBool::new(false),
            fault: AtomicU8::new(0),
        });
        let worker_shared = Arc::clone(&shared);
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
                    let step = match catch_unwind(AssertUnwindSafe(|| processor(&input))) {
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
                    let latest = PlannedCommand {
                        source_at: input.at,
                        oldest_sensor_at: oldest(&input),
                        step: Arc::new(step),
                    };
                    let fault = result_fault(&latest.step, input.at);
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
            },
            watchdog,
            thread: Some(thread),
        })
    }

    pub fn submitter(&self) -> AutonomySubmitter {
        self.submitter.clone()
    }

    pub fn try_submit(&self, input: Arc<SensorSnapshot>, now: Timestamp) -> Result<SubmitStatus> {
        self.submitter.try_submit(input, now)
    }

    /// No waiting, navigation, model calls, disk access or large report cloning.
    /// A busy mailbox retains the previous command only within its original lease.
    pub fn poll(&mut self, now: Timestamp) -> ControlPoll {
        let latest = match self.submitter.shared.slots.try_lock() {
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
        let result = self.watchdog.poll(now, latest);
        if let Some(fault) = result.fault {
            self.submitter.shared.fail(fault);
            self.stop_background();
        }
        result
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
