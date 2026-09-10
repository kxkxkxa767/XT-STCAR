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
fn newer_execution_rejects_old_source_without_rewinding_or_renewing_duplicate_leases() {
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
    assert_eq!(
        submit(&worker, &input(20), 30),
        SubmitStatus::ExecutionAhead
    );
    assert!(observed_rx.try_recv().is_err());
    assert_eq!(submit(&worker, &first, 31), SubmitStatus::Duplicate);
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
