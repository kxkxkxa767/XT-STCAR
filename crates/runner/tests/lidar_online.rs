//! Independent semantic/camera/lidar clocks through the production controller
//! and worker. These are sensor fixtures; they never send actuator commands.
use std::sync::Arc;
use std::time::{Duration, Instant};
use xt_stcar_robot_core::autonomy::{
    LightState, ObstacleDisc, Point2, Pose2, PoseEstimate, RoadObservation,
};
use xt_stcar_robot_core::local_world::{
    ElementColor, ElementFrame, ElementGeometry, ElementKind, ElementObservation, ObservationSource,
};
use xt_stcar_robot_core::mission::MissionPhase;
use xt_stcar_robot_core::{MotionOutput, Timestamp};
use xt_stcar_robot_runner::autonomy::{AutonomyController, RoadFrame};
use xt_stcar_robot_runner::autonomy_replay::SensorSnapshot;
use xt_stcar_robot_runner::control_runtime::{AutonomyWorker, ControlRuntimeConfig, SubmitStatus};
use xt_stcar_robot_runner::online::OnlineControlConfig;
use xt_stcar_robot_runner::online_simulation::OnlineScenario;
use xt_stcar_robot_runner::simulation::{SimulationConfig, synthetic_scan};

fn config() -> SimulationConfig {
    let mut c = OnlineScenario::example().compile().unwrap();
    c.autonomy.online = Some(OnlineControlConfig::lidar_first(&c.autonomy));
    c
}
fn snapshot(c: &SimulationConfig, at: u64) -> SensorSnapshot {
    let at = Timestamp(at);
    let pose = PoseEstimate {
        captured_at: at,
        frame_id: c.autonomy.mission.world_frame.clone(),
        pose: Pose2 {
            x_m: 2.0,
            y_m: 2.0,
            yaw_rad: 0.0,
        },
        speed_mps: 0.0,
        yaw_rate_radps: 0.0,
        quality: 1.0,
    };
    let cone = ObstacleDisc {
        center: Point2 { x_m: 4.0, y_m: 3.0 },
        radius_m: 0.2,
    };
    SensorSnapshot {
        at,
        scan: synthetic_scan(c, pose.pose, &[cone], at),
        road: RoadFrame {
            camera_captured_at: Some(at),
            observation_pose: Some(pose.clone()),
            observation: RoadObservation {
                captured_at: at,
                frame_id: c.autonomy.mission.body_frame.clone(),
                crosswalk: None,
                light: LightState::Unknown,
                light_confidence: 0.0,
                cones_body_m: vec![],
            },
            elements: Some(ElementFrame {
                captured_at: at,
                frame_id: c.autonomy.mission.body_frame.clone(),
                observations: vec![ElementObservation {
                    kind: ElementKind::Crosswalk,
                    color: ElementColor::White,
                    position_body_m: Point2 { x_m: 0.4, y_m: 0.0 },
                    heading_body_rad: Some(0.0),
                    geometry: ElementGeometry::LineRegion {
                        lateral_half_width_m: 0.6,
                        depth_m: 0.297,
                    },
                    source: ObservationSource::GroundProjection,
                    confidence: 1.0,
                    position_error_m: 0.001,
                    heading_error_rad: 0.0,
                }],
            }),
            image_width_px: 160,
            image_height_px: 120,
        },
        pose,
    }
}
fn submit_and_wait(
    worker: &mut AutonomyWorker,
    input: SensorSnapshot,
) -> xt_stcar_robot_runner::control_runtime::ControlPoll {
    let at = input.at;
    let input = Arc::new(input);
    let end = Instant::now() + Duration::from_secs(10);
    while let SubmitStatus::Busy = worker.try_submit(input.clone(), at).unwrap() {
        assert!(Instant::now() < end);
        std::thread::yield_now();
    }
    loop {
        let report = worker.poll(at);
        assert!(report.fault.is_none(), "{report:?}");
        if report
            .observed_plan
            .as_ref()
            .is_some_and(|s| s.source_at == at)
        {
            return report;
        }
        assert!(Instant::now() < end, "worker failed to publish");
        std::thread::yield_now();
    }
}

#[test]
fn lidar_updates_without_new_yolo_but_camera_dropout_still_stops_sync_and_async() {
    for asynchronous in [false, true] {
        let c = config();
        let mut controller = AutonomyController::new(c.autonomy.clone()).unwrap();
        controller.start().unwrap();
        let mut worker = asynchronous.then(|| {
            AutonomyWorker::spawn(
                c.autonomy.clone(),
                ControlRuntimeConfig {
                    max_command_age_ms: 200,
                    startup_timeout_ms: 200,
                },
                Timestamp(0),
            )
            .unwrap()
        });
        let mut retained = None;
        let mut locked = None;
        for at in (0..=4200).step_by(100) {
            let mut input = snapshot(&c, at);
            if at == 3300 {
                retained = Some(input.road.clone());
            }
            if at > 3300 {
                input.road = retained.clone().unwrap();
                input.road.camera_captured_at = Some(input.at);
            }
            let step = if let Some(w) = worker.as_mut() {
                submit_and_wait(w, input).observed_plan.unwrap().report
            } else {
                Arc::new(controller.tick(input.at, &input.pose, &input.scan, &input.road))
            };
            assert!(
                step.fault.is_none(),
                "{asynchronous} {at}: {:?}",
                step.fault
            );
            if at >= 3400 {
                let r = step.online.as_ref().unwrap();
                assert_eq!(r.mission.phase, MissionPhase::Cones);
                assert!(!r.requires_road_semantics);
                let t = r.active_track.unwrap();
                assert_eq!(*locked.get_or_insert(t.id), t.id);
                assert_eq!(t.observations, 0);
                assert_eq!(t.last_visual_at, None);
                assert_eq!(t.last_geometry_at, Timestamp(at));
            }
        }
        let mut input = snapshot(&c, 4300);
        input.road = retained.unwrap(); // camera actually stale
        if let Some(w) = worker.as_mut() {
            assert!(w.try_submit(Arc::new(input), Timestamp(4300)).is_err());
            assert_eq!(w.poll(Timestamp(4300)).command, MotionOutput::Stop);
        } else {
            let r = controller.tick(input.at, &input.pose, &input.scan, &input.road);
            assert_eq!(r.command, MotionOutput::Stop);
            assert!(r.fault.is_some());
        }
    }
}

#[test]
fn lidar_mode_cannot_use_stale_semantics_to_skip_crosswalk_or_accept_a_forged_pose() {
    for invalid_pose in [false, true] {
        let c = config();
        let mut ctrl = AutonomyController::new(c.autonomy.clone()).unwrap();
        ctrl.start().unwrap();
        let first = snapshot(&c, 0);
        let mut input = snapshot(&c, 500);
        input.road = first.road;
        input.road.camera_captured_at = Some(input.at);
        if invalid_pose {
            input.road.observation.captured_at = input.at;
            input.road.elements.as_mut().unwrap().captured_at = input.at;
        }
        let r = ctrl.tick(input.at, &input.pose, &input.scan, &input.road);
        assert!(r.fault.is_some());
        assert_eq!(r.command, MotionOutput::Stop);
    }
}

#[test]
fn lidar_pose_jump_cannot_recreate_objects_in_a_different_task_location() {
    let c = config();
    let mut controller = AutonomyController::new(c.autonomy.clone()).unwrap();
    controller.start().unwrap();
    let first = snapshot(&c, 0);
    assert!(
        controller
            .tick(first.at, &first.pose, &first.scan, &first.road)
            .fault
            .is_none()
    );
    let mut jumped = snapshot(&c, 100);
    jumped.pose.pose.x_m += 1.0;
    jumped.road.observation_pose = Some(jumped.pose.clone());
    let result = controller.tick(jumped.at, &jumped.pose, &jumped.scan, &jumped.road);
    assert_eq!(result.command, MotionOutput::Stop);
    assert!(result.fault.unwrap().contains("pose jumped"));
}

#[test]
fn task_transition_revokes_published_drive_before_the_new_plan_finishes_publishing() {
    use std::sync::{Mutex, mpsc};
    use xt_stcar_robot_runner::control_diagnostics::{WorkerDiagnosticsOptions, WorkerStage};
    use xt_stcar_robot_runner::control_runtime::AdoptionRejection;
    let c = config();
    let (arrived_tx, arrived_rx) = mpsc::channel();
    let (release_tx, release_rx) = mpsc::channel();
    let release_rx = Mutex::new(release_rx);
    let diagnostics = WorkerDiagnosticsOptions {
        measure_wall_time: false,
        schedule_hook: Some(Arc::new(move |stage, id| {
            if stage == WorkerStage::BeforePublish && id.source_at == Timestamp(100) {
                arrived_tx.send(()).unwrap();
                release_rx
                    .lock()
                    .unwrap()
                    .recv_timeout(Duration::from_secs(10))
                    .unwrap();
            }
        })),
    };
    let mut worker = AutonomyWorker::spawn_with_diagnostics(
        c.autonomy.clone(),
        ControlRuntimeConfig {
            max_command_age_ms: 200,
            startup_timeout_ms: 200,
        },
        Timestamp(0),
        diagnostics,
    )
    .unwrap();
    let first = submit_and_wait(&mut worker, snapshot(&c, 0));
    assert!(matches!(first.command, MotionOutput::Drive { .. }));
    let input = Arc::new(snapshot(&c, 100));
    while matches!(
        worker.try_submit(input.clone(), Timestamp(100)).unwrap(),
        SubmitStatus::Busy
    ) {
        std::thread::yield_now();
    }
    arrived_rx.recv_timeout(Duration::from_secs(10)).unwrap();
    let during = worker.poll(Timestamp(100));
    // Release before asserting, including the failure path, so no test strands a worker.
    release_tx.send(()).unwrap();
    assert_eq!(during.command, MotionOutput::Stop);
    assert_eq!(
        during.adoption_rejection,
        Some(AdoptionRejection::TaskChanged)
    );
    assert!(during.fault.is_none());
}

#[test]
fn captured_native_stop_windows_keep_lidar_geometry_without_visual_cones() {
    use xt_stcar_robot_core::local_world::LocalWorld;
    for text in [
        include_str!("fixtures/lidar-first/small_5x4.sync.stop-window.json"),
        include_str!("fixtures/lidar-first/tall_5x6.sync.stop-window.json"),
        include_str!("fixtures/lidar-first/nominal_7x5.async.stop-window.json"),
        include_str!("fixtures/lidar-first/cones_shifted.sync.stop-window.json"),
        include_str!("fixtures/lidar-first/cones_shifted.async-delay.stop-window.json"),
    ] {
        let fixture: serde_json::Value = serde_json::from_str(text).unwrap();
        let c = config();
        let mut world = LocalWorld::new(c.autonomy.online.as_ref().unwrap().world.clone()).unwrap();
        let attempts = fixture["attempts"].as_array().unwrap();
        let mut last = Timestamp(0);
        for attempt in attempts {
            let source: SensorSnapshot = serde_json::from_value(attempt["source"].clone()).unwrap();
            assert!(source.road.observation.cones_body_m.is_empty());
            assert!(
                source
                    .road
                    .elements
                    .as_ref()
                    .unwrap()
                    .observations
                    .iter()
                    .all(|o| o.kind != ElementKind::Cone)
            );
            world
                .update_scan(
                    source.at,
                    &source.pose,
                    &source.scan,
                    c.autonomy.lidar_in_body,
                )
                .unwrap();
            world.update_lidar_cones(source.at).unwrap();
            last = source.at;
        }
        let tracks: Vec<_> = world.tracks(last).collect();
        assert!(!tracks.is_empty(), "{}", fixture["case"]);
        for track in &tracks {
            assert_eq!(track.last_visual_at, None);
            assert_eq!(track.observations, 0);
        }
        let active = &attempts.last().unwrap()["online"]["active_track"];
        if active["kind"] == "cone" {
            let center: Point2 = serde_json::from_value(active["position"].clone()).unwrap();
            assert!(
                tracks
                    .iter()
                    .any(|t| t.position.distance(center) < t.position_error_m),
                "{}",
                fixture["case"]
            );
        }
    }
}
