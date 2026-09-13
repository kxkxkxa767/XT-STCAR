//! Deterministic deadline-policy tests with the real controller and certificate.
//! While an explicitly selected worker stage is suspended, logical time, poll,
//! and the independent plant continue. Host handshakes at event boundaries have
//! zero injected duration; these tests do not measure actual CPU deadlines.
use std::sync::{Arc, Mutex, mpsc};
use std::thread;
use std::time::{Duration, Instant};
use xt_stcar_robot_core::autonomy::{LightState, Pose2, PoseEstimate, RoadObservation};
use xt_stcar_robot_core::{MotionOutput, Timestamp};
use xt_stcar_robot_runner::autonomy::RoadFrame;
use xt_stcar_robot_runner::autonomy_replay::SensorSnapshot;
use xt_stcar_robot_runner::control_diagnostics::{
    PlanIdentity, ScheduleHook, WorkerDiagnosticsOptions, WorkerStage,
};
use xt_stcar_robot_runner::control_runtime::{
    AdoptionRejection, AutonomyWorker, ControlFault, ControlPoll, ControlRuntimeConfig,
    SubmitStatus,
};
use xt_stcar_robot_runner::simulation::{SimulationConfig, synthetic_scan};

struct Gate {
    entered: mpsc::Receiver<PlanIdentity>,
    release: mpsc::SyncSender<()>,
}

impl Gate {
    fn wait(&self) {
        self.entered
            .recv_timeout(Duration::from_secs(5))
            .expect("real worker reached scheduled stage");
    }
    fn release(&self) {
        self.release.send(()).unwrap();
    }
}

fn schedule(specs: &[(u64, WorkerStage)]) -> (ScheduleHook, Vec<Gate>) {
    let mut gates = Vec::new();
    let mut worker_gates = Vec::new();
    for &(planned_at, stage) in specs {
        let (entered_tx, entered_rx) = mpsc::sync_channel(1);
        let (release_tx, release_rx) = mpsc::sync_channel(1);
        gates.push(Gate {
            entered: entered_rx,
            release: release_tx,
        });
        worker_gates.push((
            Timestamp(planned_at),
            stage,
            entered_tx,
            Mutex::new(release_rx),
        ));
    }
    let hook = Arc::new(move |stage, identity: PlanIdentity| {
        for (planned_at, wanted_stage, entered, release) in &worker_gates {
            if identity.planned_at == *planned_at && stage == *wanted_stage {
                entered.send(identity).unwrap();
                release
                    .lock()
                    .unwrap()
                    .recv_timeout(Duration::from_secs(5))
                    .unwrap();
            }
        }
    });
    (hook, gates)
}

struct Harness {
    config: SimulationConfig,
    worker: AutonomyWorker,
    at: u64,
    pose: Pose2,
    speed: f64,
    curvature: f64,
    command: MotionOutput,
    distance_m: f64,
    poll_count: usize,
}

impl Harness {
    fn new(hook: Option<ScheduleHook>, timings: bool) -> Self {
        Self::with_config(SimulationConfig::example(), hook, timings)
    }

    fn with_config(config: SimulationConfig, hook: Option<ScheduleHook>, timings: bool) -> Self {
        let worker = AutonomyWorker::spawn_with_diagnostics(
            config.autonomy.clone(),
            ControlRuntimeConfig {
                max_command_age_ms: 250,
                startup_timeout_ms: 250,
            },
            Timestamp(0),
            WorkerDiagnosticsOptions {
                measure_wall_time: timings,
                schedule_hook: hook,
            },
        )
        .unwrap();
        let pose = config.initial_pose;
        Self {
            config,
            worker,
            at: 0,
            pose,
            speed: 0.0,
            curvature: 0.0,
            command: MotionOutput::Stop,
            distance_m: 0.0,
            poll_count: 0,
        }
    }

    fn capture(&self) -> Arc<SensorSnapshot> {
        let at = Timestamp(self.at);
        Arc::new(SensorSnapshot {
            at,
            pose: PoseEstimate {
                captured_at: at,
                frame_id: self.config.autonomy.mission.world_frame.clone(),
                pose: self.pose,
                speed_mps: self.speed,
                yaw_rate_radps: self.speed * self.curvature,
                quality: 1.0,
            },
            scan: synthetic_scan(&self.config, self.pose, &[], at),
            road: RoadFrame {
                observation: RoadObservation {
                    captured_at: at,
                    frame_id: self.config.autonomy.mission.body_frame.clone(),
                    crosswalk: None,
                    light: LightState::Unknown,
                    light_confidence: 0.0,
                    cones_body_m: vec![],
                },
                image_width_px: 320,
                image_height_px: 240,
            },
        })
    }

    fn submit(&self) -> SubmitStatus {
        self.submit_snapshot(self.capture(), self.at)
    }

    fn submit_snapshot(&self, input: Arc<SensorSnapshot>, planned_at: u64) -> SubmitStatus {
        let deadline = Instant::now() + Duration::from_secs(5);
        loop {
            let status = self
                .worker
                .try_submit(Arc::clone(&input), Timestamp(planned_at))
                .unwrap();
            if status != SubmitStatus::Busy {
                return status;
            }
            assert!(Instant::now() < deadline);
            thread::yield_now();
        }
    }

    fn poll(&mut self) -> ControlPoll {
        let result = self.worker.poll(Timestamp(self.at));
        self.command = result.command.clone();
        self.poll_count += 1;
        result
    }

    fn advance_to(&mut self, target: u64) -> ControlPoll {
        assert!(target > self.at);
        loop {
            let next_at = (self.at + 20).min(target);
            let duration = (next_at - self.at) as f64 / 1000.0;
            let (target_speed, target_curvature) = match self.command {
                MotionOutput::Stop => (0.0, 0.0),
                MotionOutput::Drive {
                    speed_mps,
                    curvature_per_m,
                } => (speed_mps, curvature_per_m),
            };
            // Independent RK4 integration at <=1ms, using only poll commands.
            let rate = if target_speed >= self.speed { 0.4 } else { 0.6 };
            let count = (duration / 0.001).ceil() as usize;
            let dt = duration / count as f64;
            for _ in 0..count {
                let velocity =
                    |t: f64| self.speed + (target_speed - self.speed).clamp(-rate * t, rate * t);
                let curvature = |t: f64| {
                    self.curvature + (target_curvature - self.curvature).clamp(-4.0 * t, 4.0 * t)
                };
                let (v0, vm, v1) = (velocity(0.0), velocity(dt / 2.0), velocity(dt));
                let (w0, wm, w1) = (
                    v0 * curvature(0.0),
                    vm * curvature(dt / 2.0),
                    v1 * curvature(dt),
                );
                let yaw = self.pose.yaw_rad;
                let yaw2 = yaw + w0 * dt / 2.0;
                let yaw3 = yaw + wm * dt / 2.0;
                let yaw4 = yaw + wm * dt;
                self.pose.x_m += dt / 6.0
                    * (v0 * yaw.cos()
                        + 2.0 * vm * yaw2.cos()
                        + 2.0 * vm * yaw3.cos()
                        + v1 * yaw4.cos());
                self.pose.y_m += dt / 6.0
                    * (v0 * yaw.sin()
                        + 2.0 * vm * yaw2.sin()
                        + 2.0 * vm * yaw3.sin()
                        + v1 * yaw4.sin());
                self.pose.yaw_rad += dt / 6.0 * (w0 + 4.0 * wm + w1);
                self.distance_m += dt / 6.0 * (v0 + 4.0 * vm + v1);
                self.curvature = curvature(dt);
                self.speed = v1;
            }
            self.at = next_at;
            assert!(
                self.config
                    .autonomy
                    .navigation
                    .footprint
                    .inside(self.pose, self.config.autonomy.navigation.bounds)
            );
            let result = self.poll();
            if self.at == target {
                return result;
            }
        }
    }

    // Synchronize a zero-duration scheduler event, not the injected stall being
    // tested. Every stalled interval above is advanced explicitly via advance_to.
    fn published(&mut self, planned_at: u64) -> ControlPoll {
        let deadline = Instant::now() + Duration::from_secs(5);
        loop {
            let result = self.poll();
            if result
                .observed_plan
                .as_ref()
                .is_some_and(|plan| plan.planned_at == Timestamp(planned_at))
            {
                return result;
            }
            assert!(
                Instant::now() < deadline,
                "latest published attempt {planned_at} unavailable: {result:?}"
            );
            thread::yield_now();
        }
    }

    fn start_drive(&mut self) {
        assert_eq!(self.submit(), SubmitStatus::Queued);
        let result = self.published(0);
        assert!(result.fault.is_none(), "{result:?}");
        assert!(matches!(result.command, MotionOutput::Drive { .. }));
        assert!(result.observed_plan.unwrap().report.navigation.is_some());
    }
}

#[test]
fn suspended_real_stages_pass_adoption_window_while_poll_and_plant_continue() {
    for stage in [
        WorkerStage::Dequeued,
        WorkerStage::ProjectionFinished,
        WorkerStage::ProcessorFinished,
        WorkerStage::CertificateFinished,
        WorkerStage::BeforePublish,
    ] {
        let (hook, gates) = schedule(&[(100, stage)]);
        let mut h = Harness::new(Some(hook), true);
        h.start_drive();
        h.advance_to(100);
        assert_eq!(h.submit(), SubmitStatus::Queued);
        gates[0].wait();
        let before = (h.distance_m, h.poll_count);
        let still_held = h.advance_to(220);
        assert!(still_held.fault.is_none());
        assert!(matches!(still_held.command, MotionOutput::Drive { .. }));
        assert!(h.distance_m > before.0 && h.poll_count >= before.1 + 6);
        gates[0].release();
        let rejected = h.published(100);
        assert_eq!(
            rejected.adoption_rejection,
            Some(AdoptionRejection::AfterWindow)
        );
        assert!(rejected.fault.is_none());
        assert_eq!(rejected.latest.as_ref().unwrap().at, Timestamp(0));
        let observed = rejected.observed_plan.unwrap();
        assert!(!observed.newly_adopted);
        assert_eq!((observed.source_age_ms, observed.plan_age_ms), (120, 120));
        let timing = observed.timings.unwrap();
        assert!(timing.processor_duration_ns >= timing.navigation_duration_ns.unwrap());
        assert!(
            timing.navigation_duration_ns.unwrap() >= timing.terminal_solver_duration_ns.unwrap()
        );
        assert!(timing.published_host_ns >= timing.ready_to_publish_host_ns);
        assert!(timing.ready_to_publish_host_ns >= timing.dequeued_host_ns);
        let expired = h.advance_to(260);
        assert_eq!(expired.fault, Some(ControlFault::CommandExpired));
        assert_eq!(expired.command, MotionOutput::Stop);
        h.advance_to(400);
        assert_eq!(h.speed, 0.0);
    }
}

#[test]
fn old_lease_expires_before_a_fresh_real_result_is_published_and_cannot_revive() {
    let (hook, gates) = schedule(&[(200, WorkerStage::BeforePublish)]);
    let mut h = Harness::new(Some(hook), true);
    h.start_drive();
    h.advance_to(200);
    assert_eq!(h.submit(), SubmitStatus::Queued);
    gates[0].wait();
    let result = h.advance_to(260);
    assert_eq!(result.fault, Some(ControlFault::CommandExpired));
    assert_eq!(result.command, MotionOutput::Stop);
    let brake_start_distance = h.distance_m;
    gates[0].release();
    let late = h.published(200);
    assert_eq!(late.fault, Some(ControlFault::CommandExpired));
    assert_eq!(late.command, MotionOutput::Stop);
    assert_eq!(late.observed_plan.unwrap().source_age_ms, 60);
    h.advance_to(400);
    assert!(h.distance_m > brake_start_distance);
    assert_eq!(h.speed, 0.0);
    assert_eq!(h.submit(), SubmitStatus::Stopped);
}

#[test]
fn busy_real_processor_replaces_only_pending_inputs_without_fake_adoption() {
    let (hook, gates) = schedule(&[(100, WorkerStage::Dequeued)]);
    let mut h = Harness::new(Some(hook), true);
    h.start_drive();
    h.advance_to(100);
    assert_eq!(h.submit(), SubmitStatus::Queued);
    gates[0].wait();
    for (at, expected) in [
        (120, SubmitStatus::Queued),
        (140, SubmitStatus::Replaced),
        (160, SubmitStatus::Replaced),
    ] {
        let held = h.advance_to(at);
        assert_eq!(held.latest.unwrap().at, Timestamp(0));
        assert_eq!(h.submit(), expected);
    }
    gates[0].release();
    // Wait for the newest publication without adopting the intermediate result.
    let deadline = Instant::now() + Duration::from_secs(5);
    while h.worker.diagnostics().unwrap().published_plans < 3 {
        assert!(Instant::now() < deadline);
        thread::yield_now();
    }
    let result = h.published(160);
    assert!(result.fault.is_none(), "{result:?}");
    assert!(result.adoption_rejection.is_none(), "{result:?}");
    assert!(result.observed_plan.as_ref().unwrap().newly_adopted);
    let stats = h.worker.diagnostics().unwrap();
    assert_eq!(
        (
            stats.queued_inputs,
            stats.replaced_inputs,
            stats.published_plans
        ),
        (5, 2, 3)
    );
    assert_eq!(stats.newly_adopted_plans, 2);
}

#[test]
fn actual_adoption_after_binding_invalidates_a_real_queued_plan_revision() {
    let (hook, gates) = schedule(&[
        (0, WorkerStage::BeforePublish),
        (100, WorkerStage::ProjectionFinished),
    ]);
    let mut h = Harness::new(Some(hook), true);
    assert_eq!(h.submit(), SubmitStatus::Queued);
    gates[0].wait();
    let initial = h.advance_to(100);
    assert_eq!(initial.command, MotionOutput::Stop);
    assert_eq!(h.distance_m, 0.0);
    assert_eq!(h.submit(), SubmitStatus::Queued);
    gates[0].release();
    gates[1].wait();
    let first = h.published(0);
    assert!(matches!(first.command, MotionOutput::Drive { .. }));
    h.advance_to(120);
    gates[1].release();
    let rejected = h.published(100);
    assert_eq!(
        rejected.adoption_rejection,
        Some(AdoptionRejection::ExecutionChanged)
    );
    assert!(rejected.fault.is_none());
    assert_eq!(rejected.latest.unwrap().at, Timestamp(0));
    assert!(h.distance_m > 0.0);
}

#[test]
fn disabled_timings_still_identify_publication_without_reading_optional_clock() {
    let mut h = Harness::new(None, false);
    h.start_drive();
    let result = h.poll();
    let observed = result.observed_plan.unwrap();
    assert!(observed.timings.is_none());
    assert!(observed.observed_host_ns.is_none());
    assert!(h.worker.diagnostics().is_none());
    assert!(
        observed
            .report
            .navigation
            .as_ref()
            .unwrap()
            .diagnostics
            .terminal_work
            .solver_elapsed_ns
            .is_none()
    );
    assert!(!observed.newly_adopted);
    assert!(matches!(result.command, MotionOutput::Drive { .. }));
}

#[test]
fn real_turning_controller_respects_actual_adoption_curvature_spacing() {
    let (hook, gates) = schedule(&[(100, WorkerStage::BeforePublish)]);
    let mut config = SimulationConfig::example();
    config.initial_pose.x_m = 1.6;
    config.initial_pose.yaw_rad = -0.15;
    let mut h = Harness::with_config(config, Some(hook), true);
    let source_zero = h.capture();
    h.advance_to(100);
    let source_hundred = h.capture();
    assert_eq!(h.submit_snapshot(source_zero, 100), SubmitStatus::Queued);
    gates[0].wait();
    h.advance_to(190);
    gates[0].release();
    let first = h.published(100);
    assert!(
        first.fault.is_none() && first.adoption_rejection.is_none(),
        "{first:?}"
    );
    let MotionOutput::Drive {
        curvature_per_m: first_curvature,
        ..
    } = first.command
    else {
        panic!("real turn must initially drive: {first:?}");
    };
    h.advance_to(200);
    assert_eq!(h.submit_snapshot(source_hundred, 200), SubmitStatus::Queued);
    let rejected = h.published(200);
    assert_eq!(
        rejected.adoption_rejection,
        Some(AdoptionRejection::CurvatureSlew),
        "{rejected:?}"
    );
    assert!(rejected.fault.is_none());
    assert_eq!(rejected.command, first.command);
    let attempted = rejected.observed_plan.unwrap();
    let MotionOutput::Drive {
        curvature_per_m: next_curvature,
        ..
    } = attempted.report.command
    else {
        panic!("real planner's candidate must pass the certificate");
    };
    assert!((next_curvature - first_curvature).abs() > 4.0 * 0.010 + 1e-9);
    assert!((next_curvature - first_curvature).abs() <= 4.0 * 0.100 + 1e-9);
    assert!(h.distance_m > 0.0);
    let expired = h.advance_to(260);
    assert_eq!(expired.fault, Some(ControlFault::CommandExpired));
    h.advance_to(500);
    assert_eq!(h.speed, 0.0);
    assert_eq!(h.curvature, 0.0);
}
