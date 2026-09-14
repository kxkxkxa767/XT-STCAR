//! Optional bounded observations of background planning. No disk I/O or control policy.
use crate::autonomy::AutonomyStep;
use serde::Serialize;
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Instant;
use xt_stcar_robot_core::Timestamp;

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize)]
pub struct PlanIdentity {
    pub source_at: Timestamp,
    pub oldest_sensor_at: Timestamp,
    /// Submission/model anchor, never the worker's actual start time.
    pub planned_at: Timestamp,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum WorkerStage {
    Dequeued,
    ProjectionFinished,
    ProcessorFinished,
    CertificateFinished,
    BeforePublish,
}

pub type ScheduleHook = Arc<dyn Fn(WorkerStage, PlanIdentity) + Send + Sync>;

#[derive(Clone, Default)]
pub struct WorkerDiagnosticsOptions {
    /// Disabled by default: no Instant reads in submission, planning or polling.
    pub measure_wall_time: bool,
    /// Explicit offline scheduling/fault injection, not a telemetry callback.
    /// A hook may suspend the worker while the independent output owner keeps
    /// polling; the original watchdog and admission windows still apply.
    /// Called outside mailbox locks, never from poll. Leave None for host timing.
    pub schedule_hook: Option<ScheduleHook>,
}

/// Host nanoseconds since creation of this worker's optional diagnostic clock.
/// They do not replace any source, submission, lease or model timestamp.
/// Function durations exclude explicit schedule hooks; their injected delays
/// remain visible in gaps between the corresponding host marks.
#[derive(Clone, Debug, Default, Serialize)]
pub struct PlanTimings {
    pub enqueued_host_ns: u64,
    pub dequeued_host_ns: u64,
    pub queue_wait_ns: u64,
    pub projection_started_host_ns: u64,
    pub projection_finished_host_ns: u64,
    pub projection_duration_ns: u64,
    pub processor_started_host_ns: u64,
    pub processor_finished_host_ns: u64,
    pub processor_duration_ns: u64,
    /// Only the actual Navigator plan call, when the real controller measures it.
    pub navigation_duration_ns: Option<u64>,
    pub terminal_solver_duration_ns: Option<u64>,
    pub certify_started_host_ns: u64,
    pub certify_finished_host_ns: u64,
    pub certify_duration_ns: u64,
    /// All computation and explicit scheduling hooks have returned at this mark.
    pub ready_to_publish_host_ns: u64,
    /// Captured while holding the mailbox lock immediately before publication.
    pub published_host_ns: u64,
    pub replaced_pending_input: bool,
}

/// The latest published attempt, including rejected attempts. This shared report
/// is diagnostic only; the vehicle must use ControlPoll.command exclusively.
#[derive(Clone, Debug)]
pub struct ObservedPlan {
    /// First failed final-certificate condition, shared with the worker report.
    pub certificate_failure: Option<Arc<crate::control_execution::CertificateFailure>>,
    pub source_at: Timestamp,
    pub oldest_sensor_at: Timestamp,
    pub planned_at: Timestamp,
    pub newly_adopted: bool,
    /// Age of the oldest source sensor, matching lease accounting. Ages clamp
    /// to zero for a future timestamp already identified by rejection/fault.
    pub source_age_ms: u64,
    pub plan_age_ms: u64,
    pub observed_host_ns: Option<u64>,
    pub timings: Option<Arc<PlanTimings>>,
    pub report: Arc<AutonomyStep>,
}

#[derive(Clone, Copy, Debug, Default, Serialize)]
pub struct WorkerDiagnosticsSnapshot {
    /// Accepted submissions, including inputs replacing an older pending input.
    pub queued_inputs: u64,
    pub replaced_inputs: u64,
    pub published_plans: u64,
    pub newly_adopted_plans: u64,
    /// Poll observations, not unique plans or stop episodes.
    pub rejected_polls: u64,
}

pub(crate) struct DiagnosticState {
    epoch: Option<Instant>,
    queued: AtomicU64,
    replaced: AtomicU64,
    published: AtomicU64,
    adopted: AtomicU64,
    rejected: AtomicU64,
}

impl DiagnosticState {
    pub(crate) fn new(enabled: bool) -> Self {
        Self {
            epoch: enabled.then(Instant::now),
            queued: AtomicU64::new(0),
            replaced: AtomicU64::new(0),
            published: AtomicU64::new(0),
            adopted: AtomicU64::new(0),
            rejected: AtomicU64::new(0),
        }
    }

    pub(crate) fn now(&self) -> Option<u64> {
        self.epoch
            .map(|epoch| epoch.elapsed().as_nanos().try_into().unwrap_or(u64::MAX))
    }

    // Each counter has one writer: submissions are serialized by the existing
    // mailbox; publication is worker-owned; adoption/rejection is poll-owned.
    // Reads may be concurrent. No retry loop or diagnostic lock is needed.
    fn increment(&self, counter: &AtomicU64) {
        if self.epoch.is_some() {
            counter.store(
                counter.load(Ordering::Relaxed).saturating_add(1),
                Ordering::Relaxed,
            );
        }
    }

    pub(crate) fn enqueue(&self, replaced: bool) {
        self.increment(&self.queued);
        if replaced {
            self.increment(&self.replaced);
        }
    }
    pub(crate) fn publish(&self) {
        self.increment(&self.published);
    }
    pub(crate) fn adopt(&self) {
        self.increment(&self.adopted);
    }
    pub(crate) fn reject(&self) {
        self.increment(&self.rejected);
    }

    pub(crate) fn snapshot(&self) -> Option<WorkerDiagnosticsSnapshot> {
        self.epoch.map(|_| WorkerDiagnosticsSnapshot {
            queued_inputs: self.queued.load(Ordering::Relaxed),
            replaced_inputs: self.replaced.load(Ordering::Relaxed),
            published_plans: self.published.load(Ordering::Relaxed),
            newly_adopted_plans: self.adopted.load(Ordering::Relaxed),
            rejected_polls: self.rejected.load(Ordering::Relaxed),
        })
    }
}
