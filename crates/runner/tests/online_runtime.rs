//! Online target ownership and sensor/output boundary regressions. No field
//! completion claim: these are deliberately short controller/worker checks.
use std::sync::Arc;
use std::time::{Duration, Instant};
use xt_stcar_robot_core::autonomy::{
    LightState, Point2, Pose2, PoseEstimate, Rect, RoadObservation,
};
use xt_stcar_robot_core::local_world::{
    ElementColor, ElementFrame, ElementGeometry, ElementKind, ElementObservation, MAX_ELEMENTS,
    ObservationSource,
};
use xt_stcar_robot_core::{LidarSample, MotionOutput, Timestamp};
use xt_stcar_robot_runner::autonomy::{AutonomyConfig, AutonomyController, RoadFrame};
use xt_stcar_robot_runner::autonomy_replay::SensorSnapshot;
use xt_stcar_robot_runner::control_runtime::{
    AdoptionRejection, AutonomyWorker, CertificateFailureReason, ControlFault,
    ControlRuntimeConfig, SubmitStatus,
};
use xt_stcar_robot_runner::online_simulation::{OnlineScenario, controller_config};
use xt_stcar_robot_runner::simulation::SimulationConfig;

fn snapshot(config: &AutonomyConfig, stamp: u64) -> SensorSnapshot {
    let at = Timestamp(stamp);
    SensorSnapshot {
        at,
        pose: PoseEstimate {
            captured_at: at,
            frame_id: config.navigation.frame_id.clone(),
            pose: Pose2 {
                x_m: 2.0,
                y_m: 2.5,
                yaw_rad: 0.0,
            },
            speed_mps: 0.0,
            yaw_rate_radps: 0.0,
            quality: 1.0,
        },
        scan: LidarSample {
            captured_at: at,
            frame_id: config.scan.frame_id.clone(),
            angle_min_rad: 0.0,
            angle_increment_rad: std::f64::consts::TAU / config.scan.bins as f64,
            range_min_m: 0.05,
            range_max_m: 20.0,
            ranges_m: vec![Some(5.0); config.scan.bins],
        },
        road: RoadFrame {
            observation: RoadObservation {
                captured_at: at,
                frame_id: config.mission.body_frame.clone(),
                crosswalk: None,
                light: LightState::Unknown,
                light_confidence: 0.0,
                cones_body_m: Vec::new(),
            },
            elements: Some(ElementFrame {
                captured_at: at,
                frame_id: config.mission.body_frame.clone(),
                observations: Vec::new(),
            }),
            image_width_px: 160,
            image_height_px: 120,
        },
    }
}
fn crossing() -> ElementObservation {
    ElementObservation {
        kind: ElementKind::Crosswalk,
        color: ElementColor::White,
        position_body_m: Point2 { x_m: 2.0, y_m: 0.0 },
        heading_body_rad: Some(0.0),
        geometry: ElementGeometry::LineRegion {
            lateral_half_width_m: 0.9,
            depth_m: 0.297,
        },
        source: ObservationSource::GroundProjection,
        confidence: 0.95,
        position_error_m: 0.02,
        heading_error_rad: 0.01,
    }
}
fn runtime() -> ControlRuntimeConfig {
    ControlRuntimeConfig {
        max_command_age_ms: 200,
        startup_timeout_ms: 200,
    }
}
fn submit(worker: &AutonomyWorker, input: Arc<SensorSnapshot>) -> Result<SubmitStatus, String> {
    let deadline = Instant::now() + Duration::from_secs(3);
    loop {
        let result = worker.try_submit(Arc::clone(&input), input.at);
        if !matches!(result, Ok(SubmitStatus::Busy)) {
            return result;
        }
        assert!(Instant::now() < deadline, "worker mailbox remained busy");
        std::thread::yield_now();
    }
}

#[test]
fn scene_truth_changes_do_not_change_controller_configuration_or_original_limits() {
    let first = OnlineScenario::example();
    let mut changed = first.clone();
    changed.scene.spec.length_m = 8.0;
    changed.scene.spec.width_m = 6.0;
    changed.scene.spec.left_cone_from_left_m = 1.8;
    changed.scene.spec.right_cone_from_right_m = 1.8;
    changed.scene.spec.light_front_x_m = 5.5;
    changed.scene.crosswalk_near_x_m = 2.0;
    let a = first.compile().unwrap();
    let b = changed.compile().unwrap();
    assert_ne!(a.cones, b.cones);
    assert_ne!(a.crosswalk, b.crosswalk);
    assert_ne!(
        a.online_scene.as_ref().unwrap().finish_region,
        b.online_scene.as_ref().unwrap().finish_region
    );
    assert_eq!(
        serde_json::to_value(&a.autonomy).unwrap(),
        serde_json::to_value(&b.autonomy).unwrap()
    );
    assert!(a.field.is_none() && b.field.is_none());
    // Q/R, slew/acceleration/footprint and node/terminal budgets retain the
    // existing baseline. Only the declared broad operating bounds differ.
    let mut expected = SimulationConfig::example().autonomy.navigation;
    expected.bounds = a.autonomy.navigation.bounds;
    assert_eq!(
        serde_json::to_value(expected).unwrap(),
        serde_json::to_value(&a.autonomy.navigation).unwrap()
    );
    assert_eq!(
        a.autonomy.online.as_ref().unwrap().world.self_footprint,
        Some(a.autonomy.navigation.footprint)
    );
}

#[test]
fn poisoned_legacy_mission_geometry_cannot_move_online_targets_or_boundaries() {
    let config = controller_config();
    let mut poison = config.clone();
    let impossible = Point2 {
        x_m: f64::NAN,
        y_m: f64::INFINITY,
    };
    let invalid = Rect {
        min_x_m: 99.0,
        max_x_m: -99.0,
        min_y_m: 99.0,
        max_y_m: -99.0,
    };
    poison.mission.approach_goal = impossible;
    poison.mission.crosswalk_region = invalid;
    poison.mission.cone_waypoints = vec![impossible];
    poison.mission.cone_waypoint_headings_rad = vec![Some(f64::NAN)];
    poison.mission.light_detection_region = invalid;
    poison.mission.light_stop_region = invalid;
    poison.mission.light_boundary_region = Some(invalid);
    poison.mission.light_stop_goal = impossible;
    poison.mission.light_approach_yaw_rad = f64::NAN;
    poison.mission.finish_region = invalid;
    poison.mission.finish_goal = impossible;
    poison.mission.finish_yaw_rad = f64::NAN;
    let mut baseline = AutonomyController::new(config.clone()).unwrap();
    let mut poisoned =
        AutonomyController::new(poison).expect("online never validates unused legacy geometry");
    baseline.start().unwrap();
    poisoned.start().unwrap();
    for at in [0, 100, 200] {
        let mut input = snapshot(&config, at);
        input
            .road
            .elements
            .as_mut()
            .unwrap()
            .observations
            .push(crossing());
        let a = baseline.tick(input.at, &input.pose, &input.scan, &input.road);
        let b = poisoned.tick(input.at, &input.pose, &input.scan, &input.road);
        assert!(a.fault.is_none(), "{a:?}");
        assert_eq!(
            serde_json::to_value(a).unwrap(),
            serde_json::to_value(b).unwrap()
        );
    }
}

#[test]
fn missing_asynchronous_future_and_unknown_observations_stop_and_latch() {
    let config = controller_config();
    for fault in 0..5 {
        let mut control = AutonomyController::new(config.clone()).unwrap();
        control.start().unwrap();
        let mut input = snapshot(&config, 0);
        match fault {
            0 => input.road.elements = None,
            1 => input.road.elements.as_mut().unwrap().captured_at = Timestamp(1),
            2 => input.scan.captured_at = Timestamp(1),
            3 => input.scan.ranges_m.fill(None),
            _ => input.road.elements.as_mut().unwrap().frame_id.0 = "other_body".into(),
        }
        let step = control.tick(input.at, &input.pose, &input.scan, &input.road);
        assert_eq!(step.command, MotionOutput::Stop, "case {fault}: {step:?}");
        assert!(step.fault.is_some(), "case {fault}: {step:?}");
        let fresh = snapshot(&config, 100);
        let latched = control.tick(fresh.at, &fresh.pose, &fresh.scan, &fresh.road);
        assert_eq!(latched.command, MotionOutput::Stop);
        assert!(latched.fault.is_some());
    }
    let mut control = AutonomyController::new(config.clone()).unwrap();
    control.start().unwrap();
    let input = snapshot(&config, 0);
    let stale = control.tick(
        Timestamp(config.max_sensor_age_ms),
        &input.pose,
        &input.scan,
        &input.road,
    );
    assert_eq!(stale.command, MotionOutput::Stop);
    assert!(stale.fault.is_some());
}

#[test]
fn duplicate_element_frame_does_not_create_a_second_confirmation() {
    let config = controller_config();
    let mut control = AutonomyController::new(config.clone()).unwrap();
    control.start().unwrap();
    let mut input = snapshot(&config, 0);
    input
        .road
        .elements
        .as_mut()
        .unwrap()
        .observations
        .push(crossing());
    let first = control.tick(Timestamp(0), &input.pose, &input.scan, &input.road);
    assert!(first.online.as_ref().unwrap().active_track_id.is_none());
    let duplicate = control.tick(Timestamp(100), &input.pose, &input.scan, &input.road);
    assert!(duplicate.fault.is_none(), "{duplicate:?}");
    assert!(duplicate.online.as_ref().unwrap().active_track_id.is_none());
    let mut fresh = snapshot(&config, 200);
    fresh
        .road
        .elements
        .as_mut()
        .unwrap()
        .observations
        .push(crossing());
    let confirmed = control.tick(fresh.at, &fresh.pose, &fresh.scan, &fresh.road);
    assert!(
        confirmed.online.as_ref().unwrap().active_track_id.is_some(),
        "{confirmed:?}"
    );
}

#[test]
fn asynchronous_mailbox_rejects_changed_duplicate_elements_and_excess_allocation() {
    let config = controller_config();
    let mut worker = AutonomyWorker::spawn(config.clone(), runtime(), Timestamp(0)).unwrap();
    let input = snapshot(&config, 0);
    assert_eq!(
        submit(&worker, Arc::new(input.clone())).unwrap(),
        SubmitStatus::Queued
    );
    let mut changed = input;
    changed
        .road
        .elements
        .as_mut()
        .unwrap()
        .observations
        .push(crossing());
    assert!(
        submit(&worker, Arc::new(changed)).is_err(),
        "new element content is not an unchanged duplicate"
    );
    assert_eq!(
        worker.poll(Timestamp(0)).fault,
        Some(ControlFault::InvalidInput)
    );
    worker.request_stop();
    let mut worker = AutonomyWorker::spawn(config.clone(), runtime(), Timestamp(0)).unwrap();
    let mut input = snapshot(&config, 0);
    input.road.elements.as_mut().unwrap().observations = Vec::with_capacity(MAX_ELEMENTS + 1);
    assert!(
        submit(&worker, Arc::new(input)).is_err(),
        "empty length must not hide excess allocation"
    );
    assert_eq!(
        worker.poll(Timestamp(0)).fault,
        Some(ControlFault::InvalidInput)
    );
    worker.request_stop();
}

#[test]
fn independent_async_certificate_rejects_unobserved_space_even_if_planner_uses_clear_scan() {
    let config = controller_config();
    let mut control = AutonomyController::new(config.clone()).unwrap();
    control.start().unwrap();
    let mut worker = AutonomyWorker::spawn_with_aligned_processor(
        config.clone(),
        runtime(),
        Timestamp(0),
        move |input, context| {
            let mut incorrectly_clear = input.clone();
            incorrectly_clear.scan.ranges_m.fill(Some(5.0));
            let step = control.tick_with_projection(
                input.at,
                &incorrectly_clear.pose,
                &incorrectly_clear.scan,
                &incorrectly_clear.road,
                context,
            );
            assert!(
                matches!(step.command, MotionOutput::Drive { .. }),
                "positive control must propose Drive: {step:?}"
            );
            Ok(step)
        },
    )
    .unwrap();
    let mut input = snapshot(&config, 0);
    input.scan.ranges_m[0] = None;
    submit(&worker, Arc::new(input)).unwrap();
    let deadline = Instant::now() + Duration::from_secs(3);
    loop {
        let poll = worker.poll(Timestamp(0));
        assert_eq!(poll.command, MotionOutput::Stop);
        assert!(poll.fault.is_none(), "{poll:?}");
        if poll.adoption_rejection.is_some() {
            assert_eq!(
                poll.adoption_rejection,
                Some(AdoptionRejection::CertificateUnsafe)
            );
            assert_eq!(
                poll.observed_plan
                    .as_ref()
                    .unwrap()
                    .certificate_failure
                    .as_ref()
                    .unwrap()
                    .reason,
                CertificateFailureReason::UnobservedSpace
            );
            break;
        }
        assert!(
            Instant::now() < deadline,
            "no independent certificate decision: {poll:?}"
        );
        std::thread::sleep(Duration::from_millis(1));
    }
    worker.request_stop();
}

#[test]
fn nominal_initial_scan_keeps_near_free_space_despite_far_occlusion_edges() {
    use xt_stcar_robot_core::admission::StoppingEnvelope;
    use xt_stcar_robot_core::local_world::LocalWorld;
    use xt_stcar_robot_runner::simulation::synthetic_scan;
    let scenario = OnlineScenario::example().compile().unwrap();
    let config = &scenario.autonomy;
    let mut source = snapshot(config, 0);
    source.pose.pose = scenario.initial_pose;
    source.scan = synthetic_scan(
        &scenario,
        scenario.initial_pose,
        &scenario.cones,
        Timestamp(0),
    );
    let mut world = LocalWorld::new(config.online.as_ref().unwrap().world.clone()).unwrap();
    world
        .update_scan(
            Timestamp(0),
            &source.pose,
            &source.scan,
            config.lidar_in_body,
        )
        .unwrap();
    let footprint = config.navigation.footprint;
    let padding = footprint
        .front_m
        .max(footprint.rear_m)
        .hypot(footprint.half_width_m)
        + config.navigation.clearance_m;
    let target = source
        .pose
        .pose
        .body_to_world(Point2 { x_m: 0.8, y_m: 0.0 });
    assert!(world.known_free_segment(Timestamp(0), source.pose.pose.point(), target, padding));
    let envelope = StoppingEnvelope::new(
        &config.navigation,
        source.pose.pose,
        0.04,
        0.0,
        (config.max_sensor_age_ms + config.navigation.control_period_ms) as f64 / 1000.0,
    )
    .unwrap();
    let corners = envelope
        .corners()
        .map(|p| source.pose.pose.body_to_world(p));
    assert!(world.known_free_convex_hull(Timestamp(0), &corners, 0.0));
    let mut control = AutonomyController::new(config.clone()).unwrap();
    control.start().unwrap();
    let step = control.tick(source.at, &source.pose, &source.scan, &source.road);
    assert!(step.fault.is_none(), "{step:?}");
    assert!(
        matches!(step.command, MotionOutput::Drive { .. }),
        "{step:?}"
    );
}
