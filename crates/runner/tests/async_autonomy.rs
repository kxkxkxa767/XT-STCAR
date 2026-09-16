//! Real AutonomyWorker/AutonomyController wiring with delayed synthetic sensors.
//! This is a short empty-road test, not asynchronous race or hardware acceptance.
use std::collections::VecDeque;
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};
use xt_stcar_robot_core::autonomy::{LightState, Pose2, PoseEstimate, RoadObservation};
use xt_stcar_robot_core::{MotionOutput, Timestamp};
use xt_stcar_robot_runner::autonomy::RoadFrame;
use xt_stcar_robot_runner::autonomy_replay::SensorSnapshot;
use xt_stcar_robot_runner::control_runtime::{
    AutonomyWorker, ControlFault, ControlPoll, ControlRuntimeConfig, SubmitStatus,
};
use xt_stcar_robot_runner::simulation::{SimulationConfig, synthetic_scan};

struct StraightPlant {
    pose: Pose2,
    speed: f64,
}

impl StraightPlant {
    fn advance(&mut self, command: &MotionOutput, seconds: f64) {
        let target = match command {
            MotionOutput::Stop => 0.0,
            MotionOutput::Drive {
                speed_mps,
                curvature_per_m,
            } => {
                assert!(
                    curvature_per_m.abs() < 1e-9,
                    "empty straight-road command: {command:?}"
                );
                *speed_mps
            }
        };
        // An independent exact straight-line speed integral. Only poll's final
        // adopted command reaches this plant; rejected/background plans do not.
        let rate = 0.6; // Plant acceleration is faster than the planner's 0.4 m/s².
        let ramp_time = ((target - self.speed).abs() / rate).min(seconds);
        let next = self.speed + (target - self.speed).clamp(-rate * seconds, rate * seconds);
        let distance = (self.speed + next) * 0.5 * ramp_time + target * (seconds - ramp_time);
        self.pose.x_m += distance;
        self.speed = next;
    }

    fn capture(&self, config: &SimulationConfig, at: u64) -> Arc<SensorSnapshot> {
        Arc::new(SensorSnapshot {
            at: Timestamp(at),
            pose: PoseEstimate {
                captured_at: Timestamp(at),
                frame_id: config.autonomy.mission.world_frame.clone(),
                pose: self.pose,
                speed_mps: self.speed,
                yaw_rate_radps: 0.0,
                quality: 1.0,
            },
            scan: synthetic_scan(config, self.pose, &[], Timestamp(at)),
            road: RoadFrame {
                elements: None,
                observation: RoadObservation {
                    captured_at: Timestamp(at),
                    frame_id: config.autonomy.mission.body_frame.clone(),
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
}

fn submit_and_adopt(
    worker: &mut AutonomyWorker,
    input: Arc<SensorSnapshot>,
    now: u64,
) -> ControlPoll {
    submit_and_adopt_at(worker, input, now, now)
}

fn submit_and_adopt_at(
    worker: &mut AutonomyWorker,
    input: Arc<SensorSnapshot>,
    now: u64,
    adopt_at: u64,
) -> ControlPoll {
    let deadline = Instant::now() + Duration::from_secs(3);
    loop {
        match worker
            .try_submit(Arc::clone(&input), Timestamp(now))
            .unwrap()
        {
            SubmitStatus::Queued => break,
            SubmitStatus::Busy if Instant::now() < deadline => {
                thread::sleep(Duration::from_millis(1))
            }
            status => panic!(
                "real controller could not queue source {} at {now}: {status:?}",
                input.at.0
            ),
        }
    }
    loop {
        let poll = worker.poll(Timestamp(adopt_at));
        assert!(poll.fault.is_none(), "{poll:?}");
        assert!(poll.adoption_rejection.is_none(), "{poll:?}");
        if poll
            .latest
            .as_ref()
            .is_some_and(|step| step.at == Timestamp(now))
        {
            assert_eq!(input.pose.captured_at, input.at);
            assert_eq!(input.scan.captured_at, input.at);
            assert_eq!(input.road.observation.captured_at, input.at);
            let step = poll.latest.as_ref().unwrap();
            assert!(
                step.mission.is_some(),
                "default worker must run the real task controller"
            );
            let navigation = step.navigation.as_ref().expect("real navigator report");
            assert_eq!(
                navigation.diagnostics.execution_state.unwrap().at,
                Timestamp(now)
            );
            return poll;
        }
        assert!(
            Instant::now() < deadline,
            "real background controller did not finish"
        );
        // Only waiting for host scheduling; the synthetic clock is frozen so
        // this test does not pretend to measure processor deadline performance.
        thread::sleep(Duration::from_millis(1));
    }
}

#[test]
fn default_worker_drives_from_delayed_feedback_and_stops_on_original_source_expiry() {
    let config = SimulationConfig::example();
    let mut worker = AutonomyWorker::spawn(
        config.autonomy.clone(),
        ControlRuntimeConfig {
            max_command_age_ms: 250,
            startup_timeout_ms: 250,
        },
        Timestamp(0),
    )
    .unwrap();
    let mut plant = StraightPlant {
        pose: config.initial_pose,
        speed: 0.0,
    };
    let mut command = MotionOutput::Stop;
    let mut pending = VecDeque::new();
    let mut adopted_reports = 0;
    // Capture at 10 Hz, deliver after 60 or 80 ms, and poll outputs at 50 Hz.
    // There is no custom processor returning preselected Drive commands.
    for now in (0..=1280).step_by(20) {
        if now > 0 {
            plant.advance(&command, 0.02);
        }
        if now <= 1200 && now % 100 == 0 {
            let delay = if now / 100 % 2 == 0 { 60 } else { 80 };
            pending.push_back((now + delay, plant.capture(&config, now)));
        }
        let poll = if pending
            .front()
            .is_some_and(|(delivery, _)| *delivery == now)
        {
            let (_, input) = pending.pop_front().unwrap();
            let poll = submit_and_adopt(&mut worker, input, now);
            adopted_reports += 1;
            assert!(
                matches!(poll.command, MotionOutput::Drive { .. }),
                "{poll:?}"
            );
            poll
        } else {
            worker.poll(Timestamp(now))
        };
        assert!(poll.fault.is_none(), "{poll:?}");
        assert!(poll.adoption_rejection.is_none(), "{poll:?}");
        command = poll.command;
    }
    assert_eq!(adopted_reports, 13);
    assert!(pending.is_empty());
    assert!(plant.pose.x_m > config.initial_pose.x_m + 0.1);
    assert!(plant.speed > 0.1);
    // The last capture is 1200, delivered at 1260. Its deadline is 1450,
    // not 1510 (delivery plus lease), and repeated polls cannot extend it.
    for now in (1300..=1440).step_by(20) {
        plant.advance(&command, 0.02);
        let poll = worker.poll(Timestamp(now));
        assert!(poll.fault.is_none());
        assert!(matches!(poll.command, MotionOutput::Drive { .. }));
        command = poll.command;
    }
    plant.advance(&command, 0.01);
    let expired = worker.poll(Timestamp(1450));
    assert_eq!(expired.fault, Some(ControlFault::CommandExpired));
    assert_eq!(expired.command, MotionOutput::Stop);
    command = expired.command;
    for now in (1470..=1950).step_by(20) {
        plant.advance(&command, 0.02);
        let poll = worker.poll(Timestamp(now));
        assert_eq!(poll.fault, Some(ControlFault::CommandExpired));
        assert_eq!(poll.command, MotionOutput::Stop);
        command = poll.command;
    }
    assert_eq!(plant.speed, 0.0);
}

#[test]
fn default_worker_accepts_delayed_plan_in_clear_ninety_centimeter_corridor() {
    let config = SimulationConfig::example();
    let mut scan_config = config.clone();
    scan_config.autonomy.navigation.bounds.min_y_m = 2.05;
    scan_config.autonomy.navigation.bounds.max_y_m = 2.95;
    let mut worker = AutonomyWorker::spawn(
        config.autonomy.clone(),
        ControlRuntimeConfig {
            max_command_age_ms: 250,
            startup_timeout_ms: 250,
        },
        Timestamp(0),
    )
    .unwrap();
    let mut plant = StraightPlant {
        pose: config.initial_pose,
        speed: 0.0,
    };
    let mut command = MotionOutput::Stop;
    let mut last_now = 0;
    let mut pending = VecDeque::new();
    let mut costs = Vec::new();
    for now in (0..=2280).step_by(20) {
        plant.advance(&command, (now - last_now) as f64 / 1000.0);
        last_now = now;
        if now <= 2200 && now % 100 == 0 {
            let delay = if now / 100 % 2 == 0 { 60 } else { 80 };
            pending.push_back((now + delay, plant.capture(&scan_config, now)));
        }
        let poll = if pending
            .front()
            .is_some_and(|(delivery, _)| *delivery == now)
        {
            let (_, input) = pending.pop_front().unwrap();
            let jitter = [3, 7, 9][costs.len() % 3];
            plant.advance(&command, jitter as f64 / 1000.0);
            let started = Instant::now();
            let poll = submit_and_adopt_at(&mut worker, input, now, now + jitter);
            costs.push(started.elapsed());
            last_now = now + jitter;
            poll
        } else {
            worker.poll(Timestamp(now))
        };
        assert!(poll.fault.is_none(), "{poll:?}");
        assert!(poll.adoption_rejection.is_none(), "{poll:?}");
        if now >= 60 {
            assert!(
                matches!(poll.command, MotionOutput::Drive { .. }),
                "{poll:?}"
            );
        }
        command = poll.command;
    }
    assert_eq!(costs.len(), 23);
    assert!(plant.speed > 0.17);
    assert!(plant.pose.x_m > config.initial_pose.x_m + 0.25);
    // Source 2200 is the final capture; adoption jitter never renews that lease.
    plant.advance(&command, (2450 - last_now) as f64 / 1000.0);
    let expired = worker.poll(Timestamp(2450));
    assert_eq!(expired.fault, Some(ControlFault::CommandExpired));
    assert_eq!(expired.command, MotionOutput::Stop);
    plant.advance(&expired.command, 1.0);
    assert_eq!(plant.speed, 0.0);
    let total: Duration = costs.iter().sum();
    eprintln!(
        "async default-worker corridor host submit-to-visible: n={} total_us={} mean_us={} max_us={} (full worker + scheduling; frozen synthetic clock, not WCET)",
        costs.len(),
        total.as_micros(),
        total.as_micros() / costs.len() as u128,
        costs.iter().max().unwrap().as_micros()
    );
}

#[test]
fn default_worker_keeps_stopped_when_corridor_has_a_front_wall() {
    let config = SimulationConfig::example();
    let mut scan_config = config.clone();
    scan_config.autonomy.navigation.bounds.min_y_m = 2.05;
    scan_config.autonomy.navigation.bounds.max_y_m = 2.95;
    scan_config.autonomy.navigation.bounds.max_x_m = 0.9;
    let plant = StraightPlant {
        pose: config.initial_pose,
        speed: 0.0,
    };
    let mut worker = AutonomyWorker::spawn(
        config.autonomy,
        ControlRuntimeConfig {
            max_command_age_ms: 250,
            startup_timeout_ms: 250,
        },
        Timestamp(0),
    )
    .unwrap();
    let poll = submit_and_adopt_at(&mut worker, plant.capture(&scan_config, 0), 60, 67);
    assert_eq!(poll.command, MotionOutput::Stop);
    assert_eq!(
        poll.latest
            .as_ref()
            .unwrap()
            .navigation
            .as_ref()
            .unwrap()
            .intent
            .speed_mps,
        0.0
    );
}
