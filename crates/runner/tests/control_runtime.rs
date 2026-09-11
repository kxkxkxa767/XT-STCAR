use std::sync::Arc;
use std::sync::mpsc::{Sender, channel};
use std::thread;
use std::time::{Duration, Instant};
use xt_stcar_robot_core::autonomy::{LightState, Pose2, PoseEstimate, RoadObservation};
use xt_stcar_robot_core::navigation::SteeringEstimate;
use xt_stcar_robot_core::{
    FrameId, LidarSample, MotionOutput, OutputRecord, State, StepReport, Timestamp,
};
use xt_stcar_robot_runner::autonomy::{AutonomyStep, RoadFrame};
use xt_stcar_robot_runner::autonomy_replay::SensorSnapshot;
use xt_stcar_robot_runner::control_runtime::{
    AutonomyWorker, ControlFault, ControlPoll, ControlRuntimeConfig, ControlWatchdog,
    PlannedCommand, SubmitStatus,
};
use xt_stcar_robot_runner::simulation::{SimulationConfig, synthetic_scan};

fn config() -> ControlRuntimeConfig {
    ControlRuntimeConfig {
        max_command_age_ms: 100,
        startup_timeout_ms: 200,
    }
}

fn drive() -> MotionOutput {
    MotionOutput::Drive {
        speed_mps: 0.1,
        curvature_per_m: 0.0,
    }
}

fn step(at: u64, command: MotionOutput, fault: bool) -> AutonomyStep {
    let state = if fault { State::Fault } else { State::Running };
    AutonomyStep {
        kind: "autonomy_step",
        mode: "test",
        physical_output_enabled: false,
        at: Timestamp(at),
        mission: None,
        navigation: None,
        safety: StepReport {
            event_at: Timestamp(at),
            previous_state: State::Running,
            state,
            output: OutputRecord {
                at: Timestamp(at),
                state,
                command: command.clone(),
                reason: None,
            },
            emergency_stop_latched: fault,
        },
        command,
        fault: fault.then(|| "synthetic fault".into()),
    }
}

fn planned(at: u64, oldest: u64, command: MotionOutput, fault: bool) -> PlannedCommand {
    PlannedCommand {
        source_at: Timestamp(at),
        oldest_sensor_at: Timestamp(oldest),
        planned_at: Timestamp(at),
        step: Arc::new(step(at, command, fault)),
    }
}

fn input(at: u64) -> Arc<SensorSnapshot> {
    Arc::new(SensorSnapshot {
        at: Timestamp(at),
        pose: PoseEstimate {
            captured_at: Timestamp(at),
            frame_id: FrameId("world".into()),
            pose: Pose2::default(),
            speed_mps: 0.0,
            yaw_rate_radps: 0.0,
            quality: 1.0,
        },
        scan: LidarSample {
            captured_at: Timestamp(at),
            frame_id: FrameId("laser".into()),
            angle_min_rad: -3.0,
            angle_increment_rad: 0.1,
            range_min_m: 0.1,
            range_max_m: 10.0,
            ranges_m: vec![Some(5.0); 60],
        },
        road: RoadFrame {
            observation: RoadObservation {
                captured_at: Timestamp(at),
                frame_id: FrameId("body".into()),
                crosswalk: None,
                light: LightState::Unknown,
                light_confidence: 0.0,
                cones_body_m: Vec::new(),
            },
            image_width_px: 320,
            image_height_px: 240,
        },
    })
}

fn submit(worker: &AutonomyWorker, input: &Arc<SensorSnapshot>, now: u64) -> SubmitStatus {
    let deadline = Instant::now() + Duration::from_secs(2);
    loop {
        let status = worker
            .try_submit(Arc::clone(input), Timestamp(now))
            .unwrap();
        if status != SubmitStatus::Busy {
            return status;
        }
        assert!(Instant::now() < deadline, "submission remained busy");
        thread::yield_now();
    }
}

fn wait_report(worker: &mut AutonomyWorker, now: u64, source_at: u64) -> ControlPoll {
    let deadline = Instant::now() + Duration::from_secs(2);
    loop {
        let report = worker.poll(Timestamp(now));
        assert!(report.fault.is_none(), "unexpected fault: {report:?}");
        if report
            .latest
            .as_ref()
            .is_some_and(|step| step.at == Timestamp(source_at))
        {
            return report;
        }
        assert!(Instant::now() < deadline, "missing planner result");
        thread::sleep(Duration::from_millis(1));
    }
}

#[test]
fn leases_use_oldest_capture_and_late_results_cannot_revive_expired_output() {
    let mut watchdog = ControlWatchdog::new(config(), Timestamp(0)).unwrap();
    let first = planned(70, 20, drive(), false);
    assert_eq!(
        watchdog.poll(Timestamp(75), Some(first.clone())).command,
        drive()
    );
    for now in 76..120 {
        let report = watchdog.poll(Timestamp(now), Some(first.clone()));
        assert_eq!(report.command, drive());
        assert!(report.fault.is_none());
    }
    // The new result is fresh, but the command previously issued expired at 120.
    let expired = watchdog.poll(Timestamp(120), Some(planned(120, 120, drive(), false)));
    assert_eq!(expired.command, MotionOutput::Stop);
    assert_eq!(expired.fault, Some(ControlFault::CommandExpired));
    assert_eq!(
        watchdog
            .poll(Timestamp(121), Some(planned(121, 121, drive(), false)))
            .command,
        MotionOutput::Stop
    );
}

#[test]
fn startup_timeout_and_clock_regression_latch_without_wall_clock_time() {
    let mut watchdog = ControlWatchdog::new(config(), Timestamp(1000)).unwrap();
    assert_eq!(
        watchdog.poll(Timestamp(1000), None).command,
        MotionOutput::Stop
    );
    assert!(watchdog.poll(Timestamp(1199), None).fault.is_none());
    let report = watchdog.poll(Timestamp(1200), Some(planned(1200, 1200, drive(), false)));
    assert_eq!(report.fault, Some(ControlFault::StartupExpired));
    assert_eq!(report.command, MotionOutput::Stop);
    let mut watchdog = ControlWatchdog::new(config(), Timestamp(1000)).unwrap();
    assert_eq!(
        watchdog.poll(Timestamp(999), None).fault,
        Some(ControlFault::ClockRegression)
    );
}

#[test]
fn ordinary_stop_resumes_but_controller_fault_is_permanent() {
    let mut watchdog = ControlWatchdog::new(config(), Timestamp(0)).unwrap();
    assert_eq!(
        watchdog
            .poll(Timestamp(10), Some(planned(10, 10, drive(), false)))
            .command,
        drive()
    );
    let stop = watchdog.poll(
        Timestamp(20),
        Some(planned(20, 20, MotionOutput::Stop, false)),
    );
    assert_eq!(stop.command, MotionOutput::Stop);
    assert!(stop.fault.is_none());
    assert_eq!(
        watchdog
            .poll(Timestamp(30), Some(planned(30, 30, drive(), false)))
            .command,
        drive()
    );
    assert_eq!(
        watchdog
            .poll(
                Timestamp(40),
                Some(planned(40, 40, MotionOutput::Stop, true))
            )
            .fault,
        Some(ControlFault::ControllerFault)
    );
    let late = watchdog.poll(Timestamp(50), Some(planned(50, 50, drive(), false)));
    assert_eq!(late.command, MotionOutput::Stop);
    assert_eq!(late.fault, Some(ControlFault::ControllerFault));
}

#[test]
fn changed_duplicate_future_and_regressing_results_are_rejected() {
    for malformed in [
        planned(10, 10, MotionOutput::Stop, false),
        planned(9, 9, drive(), false),
        planned(21, 21, drive(), false),
    ] {
        let mut watchdog = ControlWatchdog::new(config(), Timestamp(0)).unwrap();
        watchdog.poll(Timestamp(10), Some(planned(10, 10, drive(), false)));
        let report = watchdog.poll(Timestamp(20), Some(malformed));
        assert_eq!(report.fault, Some(ControlFault::InvalidResult));
        assert_eq!(report.command, MotionOutput::Stop);
    }
}

#[test]
fn slow_planning_keeps_latest_input_and_never_blocks_the_output_watchdog() {
    let (started_tx, started_rx) = channel();
    let (release_tx, release_rx) = channel();
    let mut worker = AutonomyWorker::spawn_with_processor(
        ControlRuntimeConfig {
            max_command_age_ms: 1000,
            startup_timeout_ms: 1000,
        },
        Timestamp(0),
        move |input| {
            started_tx.send(input.at.0).unwrap();
            release_rx.recv_timeout(Duration::from_secs(5)).unwrap();
            Ok(step(input.at.0, drive(), false))
        },
    )
    .unwrap();
    assert_eq!(submit(&worker, &input(10), 10), SubmitStatus::Queued);
    assert_eq!(started_rx.recv_timeout(Duration::from_secs(2)).unwrap(), 10);
    let discarded = input(11);
    assert_eq!(submit(&worker, &discarded, 11), SubmitStatus::Queued);
    let latest = input(50);
    let start = Instant::now();
    for at in 12..50 {
        assert_eq!(submit(&worker, &input(at), at), SubmitStatus::Replaced);
        assert_eq!(worker.poll(Timestamp(at)).command, MotionOutput::Stop);
    }
    assert_eq!(submit(&worker, &latest, 50), SubmitStatus::Replaced);
    assert!(start.elapsed() < Duration::from_millis(250));
    assert_eq!(Arc::strong_count(&discarded), 1);
    assert_eq!(
        Arc::strong_count(&latest),
        3,
        "only pending and previous aliases may retain latest input"
    );
    release_tx.send(()).unwrap();
    assert_eq!(started_rx.recv_timeout(Duration::from_secs(2)).unwrap(), 50);
    assert_eq!(wait_report(&mut worker, 50, 10).command, drive());
    assert_eq!(worker.poll(Timestamp(1009)).command, drive());
    let expired = worker.poll(Timestamp(1010));
    assert_eq!(expired.fault, Some(ControlFault::CommandExpired));
    release_tx.send(()).unwrap();
    assert_eq!(worker.poll(Timestamp(1011)).command, MotionOutput::Stop);
    assert_eq!(submit(&worker, &input(1012), 1012), SubmitStatus::Stopped);
}

struct NotifyDrop(Sender<()>);
impl Drop for NotifyDrop {
    fn drop(&mut self) {
        let _ = self.0.send(());
    }
}

#[test]
fn blocked_first_plan_expires_and_drop_does_not_join_it() {
    let (started_tx, started_rx) = channel();
    let (release_tx, release_rx) = channel();
    let (dropped_tx, dropped_rx) = channel();
    let guard = NotifyDrop(dropped_tx);
    let mut worker = AutonomyWorker::spawn_with_processor(config(), Timestamp(500), move |input| {
        let _keep_guard = &guard;
        started_tx.send(()).unwrap();
        release_rx.recv_timeout(Duration::from_secs(5)).unwrap();
        Ok(step(input.at.0, drive(), false))
    })
    .unwrap();
    submit(&worker, &input(500), 500);
    started_rx.recv_timeout(Duration::from_secs(2)).unwrap();
    assert!(worker.poll(Timestamp(699)).fault.is_none());
    assert_eq!(
        worker.poll(Timestamp(700)).fault,
        Some(ControlFault::StartupExpired)
    );
    let start = Instant::now();
    drop(worker);
    assert!(start.elapsed() < Duration::from_millis(250));
    assert!(
        dropped_rx.try_recv().is_err(),
        "processor was still blocked"
    );
    release_tx.send(()).unwrap();
    dropped_rx.recv_timeout(Duration::from_secs(2)).unwrap();
}

#[test]
fn duplicate_inputs_do_not_reprocess_and_invalid_source_data_latches_stop() {
    for invalid in 0..5 {
        let mut worker = AutonomyWorker::spawn_with_processor(config(), Timestamp(0), |input| {
            Ok(step(input.at.0, drive(), false))
        })
        .unwrap();
        let first = input(20);
        submit(&worker, &first, 20);
        wait_report(&mut worker, 20, 20);
        assert_eq!(submit(&worker, &first, 21), SubmitStatus::Duplicate);
        let mut malformed = (*input(22)).clone();
        let mut now = 22;
        match invalid {
            0 => malformed.at = Timestamp(23),
            1 => malformed.pose.captured_at = Timestamp(23),
            2 => {
                malformed = (*first).clone();
                malformed.pose.pose.x_m = 0.1;
            }
            3 => malformed = (*input(19)).clone(),
            _ => {
                malformed = (*first).clone();
                now = 20;
            }
        }
        assert!(
            worker
                .try_submit(Arc::new(malformed), Timestamp(now))
                .is_err()
        );
        let stopped = worker.poll(Timestamp(22));
        assert_eq!(stopped.command, MotionOutput::Stop);
        assert!(stopped.fault.is_some());
    }
}

#[test]
fn processor_faults_latch_even_before_output_is_polled() {
    for fault in 0..3 {
        let mut worker = AutonomyWorker::spawn_with_processor(
            config(),
            Timestamp(0),
            move |input| match fault {
                0 => Err("planner failed".into()),
                1 => panic!("synthetic planner panic"),
                _ => Ok(step(input.at.0, MotionOutput::Stop, true)),
            },
        )
        .unwrap();
        submit(&worker, &input(1), 1);
        let deadline = Instant::now() + Duration::from_secs(2);
        let report = loop {
            let report = worker.poll(Timestamp(1));
            if report.fault.is_some() {
                break report;
            }
            assert!(Instant::now() < deadline);
            thread::sleep(Duration::from_millis(1));
        };
        assert_eq!(report.command, MotionOutput::Stop);
        assert_eq!(
            report.fault,
            Some(match fault {
                0 => ControlFault::ProcessorFailed,
                1 => ControlFault::ProcessorPanicked,
                _ => ControlFault::ControllerFault,
            })
        );
        assert_eq!(submit(&worker, &input(2), 2), SubmitStatus::Stopped);
    }
}

#[test]
fn real_controller_session_is_started_and_runs_on_the_worker() {
    let simulation = SimulationConfig::example();
    let mut record = (*input(0)).clone();
    record.pose.pose = simulation.initial_pose;
    record.pose.frame_id = simulation.autonomy.mission.world_frame.clone();
    record.scan = synthetic_scan(
        &simulation,
        simulation.initial_pose,
        &simulation.cones,
        Timestamp(0),
    );
    record.road.observation.frame_id = simulation.autonomy.mission.body_frame.clone();
    let mut worker = AutonomyWorker::spawn(simulation.autonomy, config(), Timestamp(0)).unwrap();
    // The submitter can live on a separate input thread; poll owns no producer lock.
    let producer = worker.submitter();
    let status =
        thread::spawn(move || producer.try_submit(Arc::new(record), Timestamp(0)).unwrap())
            .join()
            .unwrap();
    assert_eq!(status, SubmitStatus::Queued);
    let report = wait_report(&mut worker, 0, 0);
    assert!(report.latest.unwrap().mission.is_some());
    assert!(report.fault.is_none());
}

#[test]
fn real_worker_cannot_relax_existing_controller_deadlines() {
    let simulation = SimulationConfig::example();
    assert!(
        AutonomyWorker::spawn(
            simulation.autonomy,
            ControlRuntimeConfig {
                max_command_age_ms: 1000,
                startup_timeout_ms: 200
            },
            Timestamp(0),
        )
        .is_err()
    );
    assert!(
        ControlWatchdog::new(
            ControlRuntimeConfig {
                max_command_age_ms: 0,
                startup_timeout_ms: 200
            },
            Timestamp(0),
        )
        .is_err()
    );
}

fn turning(curvature_per_m: f64) -> MotionOutput {
    MotionOutput::Drive {
        speed_mps: 0.1,
        curvature_per_m,
    }
}

fn assert_steering(state: SteeringEstimate, at: u64, commanded: f64, applied: f64) {
    assert_eq!(state.at, Timestamp(at));
    assert!((state.commanded_curvature_per_m - commanded).abs() < 1e-12);
    assert!(
        (state.applied_curvature_per_m - applied).abs() < 1e-12,
        "wrong applied estimate: {state:?}, expected {applied}"
    );
}

#[test]
fn processor_receives_only_poll_adopted_history_and_replaced_inputs_keep_bound_estimates() {
    let (started_tx, started_rx) = channel();
    let (release_tx, release_rx) = channel();
    let mut worker = AutonomyWorker::spawn_with_execution_processor(
        ControlRuntimeConfig {
            max_command_age_ms: 1000,
            startup_timeout_ms: 1000,
        },
        Timestamp(0),
        2.0,
        2.0,
        move |input, steering| {
            started_tx.send((input.at.0, steering)).unwrap();
            release_rx.recv_timeout(Duration::from_secs(5)).unwrap();
            let command = match input.at.0 {
                10 => turning(1.5),
                20 => turning(-1.5),
                _ => MotionOutput::Stop,
            };
            Ok(step(input.at.0, command, false))
        },
    )
    .unwrap();
    assert_eq!(submit(&worker, &input(10), 10), SubmitStatus::Queued);
    let (at, steering) = started_rx.recv_timeout(Duration::from_secs(2)).unwrap();
    assert_eq!(at, 10);
    assert_steering(steering, 10, 0.0, 0.0);
    assert_eq!(submit(&worker, &input(20), 20), SubmitStatus::Queued);
    release_tx.send(()).unwrap();
    let (at, steering) = started_rx.recv_timeout(Duration::from_secs(2)).unwrap();
    assert_eq!(at, 20);
    // The first Drive has been planned and published, but poll has not adopted it.
    assert_steering(steering, 20, 0.0, 0.0);
    assert_eq!(wait_report(&mut worker, 20, 10).command, turning(1.5));
    assert_steering(worker.execution_state().unwrap(), 20, 1.5, 0.0);
    worker.poll(Timestamp(120));
    assert_steering(worker.execution_state().unwrap(), 120, 1.5, 0.2);

    let discarded = input(130);
    assert_eq!(submit(&worker, &discarded, 130), SubmitStatus::Queued);
    assert_eq!(submit(&worker, &input(140), 140), SubmitStatus::Replaced);
    assert_eq!(Arc::strong_count(&discarded), 1);
    release_tx.send(()).unwrap();
    let (at, steering) = started_rx.recv_timeout(Duration::from_secs(2)).unwrap();
    assert_eq!(at, 140);
    // The second plan's opposite steering is still unadopted. This input keeps
    // its submission-time estimate, even though the second plan now exists.
    assert_steering(steering, 140, 1.5, 0.24);
    assert_eq!(wait_report(&mut worker, 140, 20).command, turning(-1.5));
    assert_steering(worker.execution_state().unwrap(), 140, -1.5, 0.24);
    worker.poll(Timestamp(150));
    assert_eq!(submit(&worker, &input(160), 160), SubmitStatus::Queued);
    release_tx.send(()).unwrap();
    let (at, steering) = started_rx.recv_timeout(Duration::from_secs(2)).unwrap();
    assert_eq!(at, 160);
    assert_steering(steering, 160, -1.5, 0.2);
    release_tx.send(()).unwrap();
}

#[test]
fn historical_execution_recovers_old_source_without_rewinding_or_renewing_duplicate_leases() {
    let (observed_tx, observed_rx) = channel();
    let mut worker = AutonomyWorker::spawn_with_execution_processor(
        config(),
        Timestamp(0),
        2.0,
        2.0,
        move |input, steering| {
            observed_tx.send((input.at.0, steering)).unwrap();
            Ok(step(input.at.0, turning(1.0), false))
        },
    )
    .unwrap();
    let first = input(10);
    submit(&worker, &first, 10);
    wait_report(&mut worker, 10, 10);
    let (_, steering) = observed_rx.recv_timeout(Duration::from_secs(2)).unwrap();
    assert_steering(steering, 10, 0.0, 0.0);
    worker.poll(Timestamp(30));
    let delayed = input(20);
    assert_eq!(submit(&worker, &delayed, 30), SubmitStatus::Queued);
    let (at, steering) = observed_rx.recv_timeout(Duration::from_secs(2)).unwrap();
    assert_eq!(at, 20);
    assert_steering(steering, 20, 1.0, 0.02);
    assert_steering(worker.execution_state().unwrap(), 30, 1.0, 0.04);
    assert_eq!(submit(&worker, &delayed, 31), SubmitStatus::Duplicate);
    let aligned = input(35);
    assert_eq!(submit(&worker, &aligned, 35), SubmitStatus::Queued);
    let (at, steering) = observed_rx.recv_timeout(Duration::from_secs(2)).unwrap();
    assert_eq!(at, 35);
    assert_steering(steering, 35, 1.0, 0.05);
    wait_report(&mut worker, 35, 35);
    assert_eq!(submit(&worker, &aligned, 100), SubmitStatus::Duplicate);
    assert_eq!(worker.poll(Timestamp(134)).command, turning(1.0));
    let expired = worker.poll(Timestamp(135));
    assert_eq!(expired.fault, Some(ControlFault::CommandExpired));
    assert_eq!(expired.command, MotionOutput::Stop);
    assert_steering(worker.execution_state().unwrap(), 135, 0.0, 0.25);
    worker.poll(Timestamp(185));
    assert_steering(worker.execution_state().unwrap(), 185, 0.0, 0.15);
}

#[test]
fn late_drive_is_never_adopted_after_watchdog_stop_and_stop_recenters_over_time() {
    let (started_tx, started_rx) = channel();
    let (release_tx, release_rx) = channel();
    let mut worker = AutonomyWorker::spawn_with_execution_processor(
        config(),
        Timestamp(0),
        2.0,
        2.0,
        move |input, steering| {
            started_tx.send(steering).unwrap();
            if input.at.0 == 30 {
                release_rx.recv_timeout(Duration::from_secs(5)).unwrap();
            }
            Ok(step(
                input.at.0,
                turning(if input.at.0 == 10 { 1.5 } else { -1.5 }),
                false,
            ))
        },
    )
    .unwrap();
    submit(&worker, &input(10), 10);
    assert_steering(
        started_rx.recv_timeout(Duration::from_secs(2)).unwrap(),
        10,
        0.0,
        0.0,
    );
    wait_report(&mut worker, 20, 10);
    submit(&worker, &input(30), 30);
    assert_steering(
        started_rx.recv_timeout(Duration::from_secs(2)).unwrap(),
        30,
        1.5,
        0.02,
    );
    let expired = worker.poll(Timestamp(110));
    assert_eq!(expired.fault, Some(ControlFault::CommandExpired));
    assert_eq!(expired.command, MotionOutput::Stop);
    assert_steering(worker.execution_state().unwrap(), 110, 0.0, 0.18);
    release_tx.send(()).unwrap();
    let later = worker.poll(Timestamp(120));
    assert_eq!(later.command, MotionOutput::Stop);
    assert_eq!(later.fault, Some(ControlFault::CommandExpired));
    assert_eq!(later.latest.unwrap().command, turning(1.5));
    assert_steering(worker.execution_state().unwrap(), 120, 0.0, 0.16);
    worker.poll(Timestamp(210));
    assert_steering(worker.execution_state().unwrap(), 210, 0.0, 0.0);
    assert_eq!(submit(&worker, &input(211), 211), SubmitStatus::Stopped);
}

#[test]
fn invalid_adopted_curvature_and_regressing_poll_clock_cannot_corrupt_execution_state() {
    for curvature in [1.0, 3.0] {
        let mut worker = AutonomyWorker::spawn_with_execution_processor(
            config(),
            Timestamp(0),
            2.0,
            2.0,
            move |input, _| Ok(step(input.at.0, turning(curvature), false)),
        )
        .unwrap();
        submit(&worker, &input(10), 10);
        if curvature <= 2.0 {
            wait_report(&mut worker, 10, 10);
            worker.poll(Timestamp(30));
            let regressed = worker.poll(Timestamp(20));
            assert_eq!(regressed.fault, Some(ControlFault::ClockRegression));
            assert_eq!(regressed.command, MotionOutput::Stop);
            assert_steering(worker.execution_state().unwrap(), 30, 0.0, 0.04);
        } else {
            let deadline = Instant::now() + Duration::from_secs(2);
            loop {
                let result = worker.poll(Timestamp(10));
                assert_eq!(result.command, MotionOutput::Stop);
                if result.fault.is_some() {
                    assert_eq!(result.fault, Some(ControlFault::InvalidResult));
                    break;
                }
                assert!(Instant::now() < deadline);
                thread::yield_now();
            }
            assert_steering(worker.execution_state().unwrap(), 10, 0.0, 0.0);
        }
    }
    for (rate, curvature) in [
        (0.0, 2.0),
        (f64::NAN, 2.0),
        (51.0, 2.0),
        (2.0, f64::INFINITY),
        (2.0, 0.0),
        (2.0, 11.0),
    ] {
        assert!(
            AutonomyWorker::spawn_with_execution_processor(
                config(),
                Timestamp(0),
                rate,
                curvature,
                |input, _| Ok(step(input.at.0, MotionOutput::Stop, false)),
            )
            .is_err()
        );
    }
}

fn aligned_drive(curvature_per_m: f64) -> MotionOutput {
    MotionOutput::Drive {
        speed_mps: 0.02,
        curvature_per_m,
    }
}

fn aligned_input(at: u64) -> Arc<SensorSnapshot> {
    let mut record = (*input(at)).clone();
    record.pose.pose = Pose2 {
        x_m: 1.5 + at as f64 * 0.00002,
        y_m: 2.5,
        yaw_rad: 0.0,
    };
    record.pose.speed_mps = if at == 0 { 0.0 } else { 0.02 };
    Arc::new(record)
}

#[test]
fn delayed_sources_with_jitter_keep_driving_using_projected_states_and_original_leases() {
    let (seen_tx, seen_rx) = channel();
    let mut worker = AutonomyWorker::spawn_with_aligned_processor(
        SimulationConfig::example().autonomy,
        ControlRuntimeConfig {
            max_command_age_ms: 250,
            startup_timeout_ms: 250,
        },
        Timestamp(0),
        move |input, context| {
            seen_tx.send((input.clone(), context.clone())).unwrap();
            Ok(step(context.planned_at.0, aligned_drive(0.0), false))
        },
    )
    .unwrap();
    for at in [0, 20, 40] {
        assert!(worker.poll(Timestamp(at)).fault.is_none());
    }
    for source in (0..=400).step_by(20) {
        let now = source + 60;
        let record = aligned_input(source);
        assert_eq!(submit(&worker, &record, now), SubmitStatus::Queued);
        let (observed, context) = seen_rx.recv_timeout(Duration::from_secs(2)).unwrap();
        assert_eq!(observed.at, Timestamp(source));
        assert_eq!(observed.pose.captured_at, Timestamp(source));
        assert_eq!(observed.pose, record.pose);
        assert_eq!(context.source_at, Timestamp(source));
        assert_eq!(context.planned_at, Timestamp(now));
        assert_eq!(context.projected_pose.captured_at, Timestamp(now));
        if source > 0 {
            assert!(context.projected_pose.pose.x_m > record.pose.pose.x_m);
        }
        // The actual output tick is deliberately not the planning timestamp.
        let adopted_at = now + 3 + source % 7;
        let output = wait_report(&mut worker, adopted_at, now);
        assert_eq!(output.command, aligned_drive(0.0));
        assert!(output.adoption_rejection.is_none());
    }
    // Receipt/planning time 460 must not renew the last source-400 lease.
    assert_eq!(worker.poll(Timestamp(649)).command, aligned_drive(0.0));
    assert_eq!(
        worker.poll(Timestamp(650)).fault,
        Some(ControlFault::CommandExpired)
    );
    assert_eq!(worker.poll(Timestamp(660)).command, MotionOutput::Stop);
}

#[test]
fn aligned_admission_window_rejects_a_late_result_without_starting_motion() {
    use xt_stcar_robot_runner::control_runtime::AdoptionRejection;
    let (started_tx, started_rx) = channel();
    let (release_tx, release_rx) = channel();
    let mut worker = AutonomyWorker::spawn_with_aligned_processor(
        SimulationConfig::example().autonomy,
        ControlRuntimeConfig {
            max_command_age_ms: 250,
            startup_timeout_ms: 250,
        },
        Timestamp(0),
        move |_, context| {
            started_tx.send(()).unwrap();
            release_rx.recv_timeout(Duration::from_secs(2)).unwrap();
            Ok(step(context.planned_at.0, aligned_drive(0.0), false))
        },
    )
    .unwrap();
    assert_eq!(submit(&worker, &aligned_input(0), 60), SubmitStatus::Queued);
    started_rx.recv_timeout(Duration::from_secs(2)).unwrap();
    assert_eq!(worker.poll(Timestamp(161)).command, MotionOutput::Stop);
    release_tx.send(()).unwrap();
    let deadline = Instant::now() + Duration::from_secs(2);
    loop {
        let poll = worker.poll(Timestamp(162));
        assert_eq!(poll.command, MotionOutput::Stop);
        assert!(poll.fault.is_none());
        if poll.adoption_rejection == Some(AdoptionRejection::AfterWindow) {
            break;
        }
        assert!(Instant::now() < deadline);
        thread::yield_now();
    }
    assert_eq!(
        worker.poll(Timestamp(250)).fault,
        Some(ControlFault::StartupExpired)
    );
}

#[test]
fn an_intervening_adoption_invalidates_a_queued_projection() {
    use xt_stcar_robot_runner::control_runtime::AdoptionRejection;
    let (started_tx, started_rx) = channel();
    let (release_tx, release_rx) = channel();
    let mut worker = AutonomyWorker::spawn_with_aligned_processor(
        SimulationConfig::example().autonomy,
        ControlRuntimeConfig {
            max_command_age_ms: 250,
            startup_timeout_ms: 250,
        },
        Timestamp(0),
        move |input, context| {
            started_tx.send(input.at.0).unwrap();
            release_rx.recv_timeout(Duration::from_secs(2)).unwrap();
            Ok(step(
                context.planned_at.0,
                aligned_drive(if input.at.0 == 0 { 0.2 } else { -0.2 }),
                false,
            ))
        },
    )
    .unwrap();
    submit(&worker, &aligned_input(0), 60);
    assert_eq!(started_rx.recv_timeout(Duration::from_secs(2)).unwrap(), 0);
    submit(&worker, &aligned_input(20), 80);
    release_tx.send(()).unwrap();
    assert_eq!(started_rx.recv_timeout(Duration::from_secs(2)).unwrap(), 20);
    // The second input was bound before the first command was actually adopted.
    assert_eq!(wait_report(&mut worker, 83, 60).command, aligned_drive(0.2));
    release_tx.send(()).unwrap();
    let deadline = Instant::now() + Duration::from_secs(2);
    loop {
        let poll = worker.poll(Timestamp(85));
        assert_eq!(poll.command, aligned_drive(0.2));
        assert!(poll.fault.is_none());
        if poll.adoption_rejection == Some(AdoptionRejection::ExecutionChanged) {
            break;
        }
        assert!(Instant::now() < deadline);
        thread::yield_now();
    }
    // Rejection cannot extend the old source-0 command past its original lease.
    assert_eq!(
        worker.poll(Timestamp(250)).fault,
        Some(ControlFault::CommandExpired)
    );
}

#[test]
fn asynchronous_certificate_covers_source_to_deadline_stopping_space() {
    let mut worker = AutonomyWorker::spawn_with_aligned_processor(
        SimulationConfig::example().autonomy,
        ControlRuntimeConfig {
            max_command_age_ms: 250,
            startup_timeout_ms: 250,
        },
        Timestamp(0),
        |_, context| Ok(step(context.planned_at.0, aligned_drive(0.0), false)),
    )
    .unwrap();
    let mut close = (*aligned_input(0)).clone();
    // The ordinary small body fits, but not the source-to-lease braking envelope.
    close.pose.pose.x_m = 0.35;
    submit(&worker, &Arc::new(close), 60);
    let poll = wait_report(&mut worker, 65, 60);
    assert_eq!(poll.command, MotionOutput::Stop);
    assert!(poll.fault.is_none());
}

#[test]
fn admission_checks_speed_limits_at_both_ends_of_the_window() {
    use xt_stcar_robot_runner::control_runtime::AdoptionRejection;
    let mut worker = AutonomyWorker::spawn_with_aligned_processor(
        SimulationConfig::example().autonomy,
        ControlRuntimeConfig {
            max_command_age_ms: 250,
            startup_timeout_ms: 250,
        },
        Timestamp(0),
        |_, context| {
            Ok(step(
                context.planned_at.0,
                MotionOutput::Drive {
                    speed_mps: 0.24,
                    curvature_per_m: 0.0,
                },
                false,
            ))
        },
    )
    .unwrap();
    let mut record = (*aligned_input(0)).clone();
    record.pose.speed_mps = 0.2;
    submit(&worker, &Arc::new(record), 0);
    // At plan time .24 is reachable from .20 in one tick, but the old Stop
    // reduces the speed to .14 by the end of the 100 ms adoption window.
    let poll = wait_report(&mut worker, 4, 0);
    assert_eq!(poll.command, MotionOutput::Stop);
    assert_eq!(
        poll.adoption_rejection,
        Some(AdoptionRejection::CertificateUnsafe)
    );
}

#[test]
fn aligned_runtime_cannot_outlive_shorter_mission_or_navigation_freshness() {
    for which in 0..3 {
        let mut config = SimulationConfig::example().autonomy;
        match which {
            0 => config.mission.max_pose_age_ms = 100,
            1 => config.mission.max_road_age_ms = 100,
            _ => config.navigation.max_input_age_ms = 100,
        }
        assert!(
            AutonomyWorker::spawn(
                config.clone(),
                ControlRuntimeConfig {
                    max_command_age_ms: 150,
                    startup_timeout_ms: 250
                },
                Timestamp(0)
            )
            .is_err()
        );
        assert!(
            AutonomyWorker::spawn_with_aligned_processor(
                config,
                ControlRuntimeConfig {
                    max_command_age_ms: 150,
                    startup_timeout_ms: 250
                },
                Timestamp(0),
                |_, context| Ok(step(context.planned_at.0, MotionOutput::Stop, false))
            )
            .is_err()
        );
    }
}

#[test]
fn projected_navigation_does_not_make_old_task_observations_fresh() {
    use xt_stcar_robot_runner::autonomy::AutonomyController;
    use xt_stcar_robot_runner::control_runtime::PlanningContext;
    let simulation = SimulationConfig::example();
    let mut config = simulation.autonomy.clone();
    config.mission.max_road_age_ms = 50;
    let mut controller = AutonomyController::new(config).unwrap();
    controller.start().unwrap();
    let mut record = (*input(0)).clone();
    record.pose.pose = simulation.initial_pose;
    record.pose.frame_id = simulation.autonomy.mission.world_frame.clone();
    record.road.observation.frame_id = simulation.autonomy.mission.body_frame.clone();
    record.scan = synthetic_scan(
        &simulation,
        simulation.initial_pose,
        &simulation.cones,
        Timestamp(0),
    );
    let mut projected = record.pose.clone();
    projected.captured_at = Timestamp(60);
    let context = PlanningContext {
        source_at: Timestamp(0),
        planned_at: Timestamp(60),
        projected_pose: projected,
        steering: SteeringEstimate::stationary(Timestamp(60)),
        adopted_revision: 0,
        held_speed_mps: 0.0,
    };
    let output = controller.tick_with_projection(
        record.at,
        &record.pose,
        &record.scan,
        &record.road,
        &context,
    );
    assert_eq!(output.command, MotionOutput::Stop);
    assert_eq!(output.safety.state, State::Fault);
    assert!(
        output.mission.as_ref().is_some_and(
            |mission| mission.phase == xt_stcar_robot_core::mission::MissionPhase::Fault
        )
    );
}

#[test]
fn asynchronous_certificate_keeps_the_light_boundary_until_mission_release() {
    use xt_stcar_robot_core::mission::{MissionOutput, MissionPhase, MissionReport};
    use xt_stcar_robot_runner::control_runtime::AdoptionRejection;
    let mut worker = AutonomyWorker::spawn_with_aligned_processor(
        SimulationConfig::example().autonomy,
        ControlRuntimeConfig {
            max_command_age_ms: 250,
            startup_timeout_ms: 250,
        },
        Timestamp(0),
        |_, context| {
            let mut report = step(context.planned_at.0, aligned_drive(0.0), false);
            report.mission = Some(MissionReport {
                at: context.planned_at,
                phase: MissionPhase::ApproachLight,
                output: MissionOutput::Stop,
                reason: "test constrained phase".into(),
                crosswalk_stop_elapsed_ms: 0,
                green_elapsed_ms: 0,
                waypoint_index: 2,
            });
            Ok(report)
        },
    )
    .unwrap();
    let mut record = (*aligned_input(0)).clone();
    record.pose.pose.x_m = 5.7;
    submit(&worker, &Arc::new(record), 60);
    let poll = wait_report(&mut worker, 65, 60);
    assert_eq!(poll.command, MotionOutput::Stop);
    assert_eq!(
        poll.adoption_rejection,
        Some(AdoptionRejection::CertificateUnsafe)
    );
}

#[test]
fn commanded_steering_slew_uses_real_adoption_spacing() {
    use xt_stcar_robot_runner::control_runtime::AdoptionRejection;
    let (ready_tx, ready_rx) = channel();
    let mut worker = AutonomyWorker::spawn_with_aligned_processor(
        SimulationConfig::example().autonomy,
        ControlRuntimeConfig {
            max_command_age_ms: 250,
            startup_timeout_ms: 250,
        },
        Timestamp(0),
        move |input, context| {
            ready_tx.send(()).unwrap();
            Ok(step(
                context.planned_at.0,
                aligned_drive(if input.at.0 == 0 { 0.2 } else { 0.6 }),
                false,
            ))
        },
    )
    .unwrap();
    submit(&worker, &aligned_input(0), 100);
    ready_rx.recv_timeout(Duration::from_secs(2)).unwrap();
    assert_eq!(
        wait_report(&mut worker, 190, 100).command,
        aligned_drive(0.2)
    );
    submit(&worker, &aligned_input(100), 200);
    ready_rx.recv_timeout(Duration::from_secs(2)).unwrap();
    let deadline = Instant::now() + Duration::from_secs(2);
    loop {
        let output = worker.poll(Timestamp(200));
        assert_eq!(output.command, aligned_drive(0.2));
        if output.adoption_rejection == Some(AdoptionRejection::CurvatureSlew) {
            break;
        }
        assert!(Instant::now() < deadline);
        thread::yield_now();
    }
    // Planning elapsed 100 ms; real command adoption elapsed only 10 ms.
    // Waiting for the slew allowance cannot renew the original source-0 lease.
    assert_eq!(
        worker.poll(Timestamp(250)).fault,
        Some(ControlFault::CommandExpired)
    );
}

#[test]
fn async_projection_and_certificate_share_navigation_measurement_roundoff() {
    for (measured, normalized, target) in [(-1e-7, 0.0, 0.02), (0.3 + 1e-7, 0.3, 0.26)] {
        let (seen_tx, seen_rx) = channel();
        let command = MotionOutput::Drive {
            speed_mps: target,
            curvature_per_m: 0.0,
        };
        let expected = command.clone();
        let mut worker = AutonomyWorker::spawn_with_aligned_processor(
            SimulationConfig::example().autonomy,
            ControlRuntimeConfig {
                max_command_age_ms: 250,
                startup_timeout_ms: 250,
            },
            Timestamp(0),
            move |input, context| {
                seen_tx
                    .send((input.pose.speed_mps, context.projected_pose.speed_mps))
                    .unwrap();
                Ok(step(context.planned_at.0, command.clone(), false))
            },
        )
        .unwrap();
        let mut record = (*aligned_input(0)).clone();
        record.pose.speed_mps = measured;
        submit(&worker, &Arc::new(record), 0);
        let (source_speed, model_speed) = seen_rx.recv_timeout(Duration::from_secs(2)).unwrap();
        assert_eq!(source_speed, measured);
        assert_eq!(model_speed, normalized);
        let output = wait_report(&mut worker, 3, 0);
        assert_eq!(output.command, expected);
        assert!(output.adoption_rejection.is_none());
        assert!(output.fault.is_none());
    }
}

#[test]
fn async_projection_still_latches_truly_out_of_range_measurements() {
    for measured in [-1e-4, 0.3 + 1e-4] {
        let (seen_tx, seen_rx) = channel();
        let mut worker = AutonomyWorker::spawn_with_aligned_processor(
            SimulationConfig::example().autonomy,
            ControlRuntimeConfig {
                max_command_age_ms: 250,
                startup_timeout_ms: 250,
            },
            Timestamp(0),
            move |_, context| {
                seen_tx.send(()).unwrap();
                Ok(step(context.planned_at.0, aligned_drive(0.0), false))
            },
        )
        .unwrap();
        let mut record = (*aligned_input(0)).clone();
        record.pose.speed_mps = measured;
        submit(&worker, &Arc::new(record), 0);
        let deadline = Instant::now() + Duration::from_secs(2);
        loop {
            let output = worker.poll(Timestamp(1));
            assert_eq!(output.command, MotionOutput::Stop);
            if output.fault == Some(ControlFault::InvalidInput) {
                break;
            }
            assert!(Instant::now() < deadline);
            thread::yield_now();
        }
        assert!(seen_rx.try_recv().is_err());
        assert_eq!(
            worker.poll(Timestamp(10)).fault,
            Some(ControlFault::InvalidInput)
        );
    }
}

#[test]
fn measurement_roundoff_does_not_expand_the_command_speed_limit() {
    use xt_stcar_robot_runner::control_runtime::AdoptionRejection;
    let mut worker = AutonomyWorker::spawn_with_aligned_processor(
        SimulationConfig::example().autonomy,
        ControlRuntimeConfig {
            max_command_age_ms: 250,
            startup_timeout_ms: 250,
        },
        Timestamp(0),
        |_, context| {
            Ok(step(
                context.planned_at.0,
                MotionOutput::Drive {
                    speed_mps: 0.3 + 1e-7,
                    curvature_per_m: 0.0,
                },
                false,
            ))
        },
    )
    .unwrap();
    let mut record = (*aligned_input(0)).clone();
    record.pose.speed_mps = 0.3;
    submit(&worker, &Arc::new(record), 0);
    let output = wait_report(&mut worker, 3, 0);
    assert_eq!(output.command, MotionOutput::Stop);
    assert_eq!(
        output.adoption_rejection,
        Some(AdoptionRejection::CertificateUnsafe)
    );
}
