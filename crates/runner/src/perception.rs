//! In-memory road perception and a bounded latest-frame worker.
//!
//! Native inference runs on one background thread, never while a mailbox is locked.
//! Stopping or dropping a worker requests cooperative exit and detaches its thread;
//! it does not claim to cancel an arbitrary native call. If that call never returns,
//! its session and active frame remain alive until the process exits. Do not create
//! replacement workers repeatedly after a hung runtime. No frames or tensors are saved.
use crate::autonomy::{Result, RoadFrame};
use crate::input::{MAX_CONFIG_BYTES, read_regular_file};
use crate::vision::VisionOptions;
use image::RgbImage;
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{SyncSender, sync_channel};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use xt_stcar::NativeOrtBackend;
use xt_stcar_robot_core::{FrameId, Timestamp};
use xt_stcar_vision::road::{RoadConfig, RoadDetector};
use xt_stcar_vision::{InferenceBackend, ModelSpec, decode, preprocess};

pub const MAX_FRAME_PIXELS: u64 = 64_000_000;
pub const MAX_FRAME_BYTES: usize = 192_000_000;

/// One persistent native session, with Rust preprocessing, decoding and road cues.
pub struct RoadPipeline {
    road: RoadDetector,
    vision: Option<(NativeOrtBackend, ModelSpec)>,
}

impl RoadPipeline {
    /// Constructor-only model/spec loading. With no vision model, light recognition
    /// uses the explicitly configured ROIs; an empty ROI list produces Unknown.
    pub fn new(road: RoadConfig, vision: Option<&VisionOptions>) -> Result<Self> {
        let road = RoadDetector::new(road)?;
        let vision = vision
            .map(|options| -> Result<_> {
                let bytes = read_regular_file(&options.spec, MAX_CONFIG_BYTES)
                    .map_err(|error| format!("vision config: {error}"))?;
                let spec: ModelSpec =
                    serde_json::from_slice(&bytes).map_err(|error| error.to_string())?;
                spec.validate()?;
                let backend = NativeOrtBackend::new(&options.model, &options.runtime_lib, &spec)?;
                Ok((backend, spec))
            })
            .transpose()?;
        Ok(Self { road, vision })
    }

    pub fn process(
        &mut self,
        image: &RgbImage,
        at: Timestamp,
        frame_id: FrameId,
    ) -> Result<RoadFrame> {
        validate_image(image, MAX_FRAME_PIXELS, MAX_FRAME_BYTES)?;
        frame_id.validate().map_err(|error| error.to_string())?;
        let detections = if let Some((backend, spec)) = &mut self.vision {
            let input = preprocess(image, spec)?;
            let output = backend.infer(&input, spec)?;
            decode(&output, spec, &input.transform)?
        } else {
            Vec::new()
        };
        // RoadDetector selects class 9 lamp ROIs and excludes them from cone cues.
        let observation = self.road.detect(image, &detections, at, frame_id)?;
        Ok(RoadFrame {
            observation,
            image_width_px: image.width(),
            image_height_px: image.height(),
        })
    }
}

/// Input memory limits apply to each of the one active and one pending frames.
/// Queue capacity is always one and cannot be increased through configuration.
#[derive(Clone, Copy, Debug)]
pub struct WorkerLimits {
    pub max_pixels: u64,
    pub max_frame_bytes: usize,
}

impl Default for WorkerLimits {
    fn default() -> Self {
        Self {
            max_pixels: 1920 * 1080,
            max_frame_bytes: 1920 * 1080 * 3,
        }
    }
}

impl WorkerLimits {
    fn validate(self) -> Result<Self> {
        if self.max_pixels == 0
            || self.max_pixels > MAX_FRAME_PIXELS
            || self.max_frame_bytes < 3
            || self.max_frame_bytes > MAX_FRAME_BYTES
        {
            return Err("perception limits exceed the supported image bounds".into());
        }
        Ok(self)
    }
}

fn validate_image(image: &RgbImage, max_pixels: u64, max_bytes: usize) -> Result<()> {
    let pixels = u64::from(image.width()) * u64::from(image.height());
    if pixels == 0
        || pixels > max_pixels
        || pixels.checked_mul(3) != Some(image.as_raw().len() as u64)
        || image.as_raw().len() > max_bytes
        || image.as_raw().capacity() > max_bytes
    {
        return Err("perception image dimensions, RGB storage or allocation exceed limits".into());
    }
    Ok(())
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SubmitStatus {
    Queued,
    Replaced,
    /// A brief mailbox operation owns the slot. The caller may drop this frame.
    Busy,
    Stopped,
}

#[derive(Debug)]
pub struct PerceptionResult {
    pub captured_at: Timestamp,
    pub frame_id: FrameId,
    /// Errors are latched: the worker stops accepting further input after one.
    pub result: Result<RoadFrame>,
}

struct Job {
    image: Arc<RgbImage>,
    captured_at: Timestamp,
    frame_id: FrameId,
}

#[derive(Default)]
struct Slots {
    pending: Option<Job>,
    latest: Option<Arc<PerceptionResult>>,
    last_submitted_at: Option<Timestamp>,
}

struct Shared {
    slots: Mutex<Slots>,
    stopped: AtomicBool,
}

pub struct PerceptionWorker {
    limits: WorkerLimits,
    shared: Arc<Shared>,
    wake: SyncSender<()>,
    // Never joined by Drop: a native runtime may ignore cooperative cancellation.
    thread: Option<JoinHandle<()>>,
}

impl PerceptionWorker {
    pub fn spawn(mut pipeline: RoadPipeline, limits: WorkerLimits) -> Result<Self> {
        Self::spawn_with_processor(limits, move |image, at, frame_id| {
            pipeline.process(image, at, frame_id)
        })
    }

    /// The processor is trusted in-process code, not a memory or execution sandbox.
    /// Its output must retain the input timestamp, frame identity and dimensions.
    pub fn spawn_with_processor<F>(limits: WorkerLimits, mut processor: F) -> Result<Self>
    where
        F: FnMut(&RgbImage, Timestamp, FrameId) -> Result<RoadFrame> + Send + 'static,
    {
        let limits = limits.validate()?;
        let shared = Arc::new(Shared {
            slots: Mutex::new(Slots::default()),
            stopped: AtomicBool::new(false),
        });
        let worker_shared = Arc::clone(&shared);
        let (wake, receive) = sync_channel(1);
        let thread = thread::Builder::new()
            .name("road-perception".into())
            .spawn(move || {
                while receive.recv().is_ok() {
                    if worker_shared.stopped.load(Ordering::Acquire) {
                        break;
                    }
                    let job = match worker_shared.slots.lock() {
                        Ok(mut slots) => slots.pending.take(),
                        Err(_) => break,
                    };
                    let Some(job) = job else { continue };
                    let result = catch_unwind(AssertUnwindSafe(|| {
                        processor(&job.image, job.captured_at, job.frame_id.clone())
                    }))
                    .unwrap_or_else(|_| Err("perception processor panicked".into()))
                    .and_then(|frame| validate_result(frame, &job))
                    .map_err(bounded_error);
                    let failed = result.is_err();
                    let result = Arc::new(PerceptionResult {
                        captured_at: job.captured_at,
                        frame_id: job.frame_id,
                        result,
                    });
                    if worker_shared.stopped.load(Ordering::Acquire) {
                        break;
                    }
                    let retired = match worker_shared.slots.lock() {
                        Ok(mut slots) => {
                            let old = slots.latest.replace(result);
                            let pending = if failed {
                                worker_shared.stopped.store(true, Ordering::Release);
                                slots.pending.take()
                            } else {
                                None
                            };
                            (old, pending)
                        }
                        Err(_) => break,
                    };
                    // Release old allocations after unlocking the control mailbox.
                    drop(retired);
                    if failed {
                        break;
                    }
                }
                worker_shared.stopped.store(true, Ordering::Release);
                let pending = worker_shared
                    .slots
                    .lock()
                    .ok()
                    .and_then(|mut slots| slots.pending.take());
                drop(pending);
            })
            .map_err(|error| format!("start perception worker: {error}"))?;
        Ok(Self {
            limits,
            shared,
            wake,
            thread: Some(thread),
        })
    }

    /// Constant-size validation and try_lock only; never waits for inference.
    /// Arrival may replace one pending frame, but cannot replace the active frame.
    /// Regressing capture times are rejected instead of making old data look new.
    pub fn try_submit(
        &self,
        image: Arc<RgbImage>,
        captured_at: Timestamp,
        frame_id: FrameId,
    ) -> Result<SubmitStatus> {
        validate_image(&image, self.limits.max_pixels, self.limits.max_frame_bytes)?;
        frame_id.validate().map_err(|error| error.to_string())?;
        let mut slots = match self.shared.slots.try_lock() {
            Ok(slots) => slots,
            Err(std::sync::TryLockError::WouldBlock) => return Ok(SubmitStatus::Busy),
            Err(_) => return Err("perception mailbox is poisoned".into()),
        };
        if self.shared.stopped.load(Ordering::Acquire) {
            return Ok(SubmitStatus::Stopped);
        }
        if slots.last_submitted_at.is_some_and(|old| captured_at < old) {
            return Err("perception capture timestamp regressed".into());
        }
        slots.last_submitted_at = Some(captured_at);
        let retired = slots.pending.replace(Job {
            image,
            captured_at,
            frame_id,
        });
        let status = if retired.is_some() {
            SubmitStatus::Replaced
        } else {
            SubmitStatus::Queued
        };
        drop(slots);
        // A full wake channel already guarantees a future pending-slot check.
        let _ = self.wake.try_send(());
        drop(retired);
        Ok(status)
    }

    /// O(1) Arc clone; a busy mailbox returns an error instead of waiting.
    /// The consumer must use captured_at for freshness, not the time of this call.
    pub fn snapshot(&self) -> Result<Option<Arc<PerceptionResult>>> {
        Ok(self
            .shared
            .slots
            .try_lock()
            .map_err(|_| "perception mailbox is busy or poisoned")?
            .latest
            .clone())
    }

    pub fn is_stopped(&self) -> bool {
        self.shared.stopped.load(Ordering::Acquire)
    }

    /// Nonblocking cooperative stop. An in-flight native call may remain alive.
    pub fn request_stop(&self) {
        self.shared.stopped.store(true, Ordering::Release);
        // Usually succeeds immediately: processing never owns this lock. If a
        // concurrent mailbox call owns it, the detached thread clears the slots
        // when it exits; at most one pending allocation can remain until then.
        let pending = self
            .shared
            .slots
            .try_lock()
            .ok()
            .and_then(|mut slots| slots.pending.take());
        let _ = self.wake.try_send(());
        drop(pending);
    }
}

impl Drop for PerceptionWorker {
    fn drop(&mut self) {
        self.request_stop();
        // Dropping JoinHandle detaches; disconnecting wake also releases an idle worker.
        drop(self.thread.take());
    }
}

fn bounded_error(mut error: String) -> String {
    let mut end = error.len().min(1024);
    while !error.is_char_boundary(end) {
        end -= 1;
    }
    error.truncate(end);
    // Release excess capacity returned by a custom processor before publication.
    error.shrink_to_fit();
    error
}

fn validate_result(mut frame: RoadFrame, job: &Job) -> Result<RoadFrame> {
    if frame.observation.captured_at != job.captured_at
        || frame.observation.frame_id != job.frame_id
        || frame.image_width_px != job.image.width()
        || frame.image_height_px != job.image.height()
        || frame.observation.cones_body_m.len() > 256
    {
        return Err("perception processor changed frame metadata or exceeded result limits".into());
    }
    frame.observation.cones_body_m.shrink_to_fit();
    frame.observation.frame_id.0.shrink_to_fit();
    Ok(frame)
}
