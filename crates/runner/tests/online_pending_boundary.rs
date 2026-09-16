//! First credible stop-region evidence can restrict motion before confirmation.
use xt_stcar_robot_core::autonomy::{LightState, Point2, Pose2, PoseEstimate, RoadObservation};
use xt_stcar_robot_core::local_world::{
    ElementColor, ElementFrame, ElementGeometry, ElementKind, ElementObservation, ObservationSource,
};
use xt_stcar_robot_core::{LidarSample, MotionOutput, Timestamp};
use xt_stcar_robot_runner::autonomy::{AutonomyConfig, AutonomyController, RoadFrame};
use xt_stcar_robot_runner::autonomy_replay::SensorSnapshot;
use xt_stcar_robot_runner::online_simulation::controller_config;
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
fn stop_region(front: f64) -> ElementObservation {
    ElementObservation {
        kind: ElementKind::StopLine,
        color: ElementColor::White,
        position_body_m: Point2 {
            x_m: front,
            y_m: 0.0,
        },
        heading_body_rad: Some(0.0),
        geometry: ElementGeometry::LineRegion {
            lateral_half_width_m: 0.9,
            depth_m: 0.4,
        },
        source: ObservationSource::GroundMarker,
        confidence: 0.99,
        position_error_m: 0.001,
        heading_error_rad: 0.001,
    }
}
fn control(config: &AutonomyConfig) -> AutonomyController {
    let mut control = AutonomyController::new(config.clone()).unwrap();
    control.start().unwrap();
    control
}
#[test]
fn first_stop_line_blocks_complete_vehicle_envelope_without_confirming_or_green() {
    let config = controller_config();
    let mut plain = control(&config);
    let mut guarded = control(&config);
    let mut input = snapshot(&config, 0);
    let baseline = plain.tick(input.at, &input.pose, &input.scan, &input.road);
    assert!(
        matches!(baseline.command, MotionOutput::Drive { .. }),
        "{baseline:?}"
    );
    // Vehicle center remains before the far edge (0.3 m), but its complete
    // uncertainty/clearance/braking envelope cannot stay before that edge.
    input
        .road
        .elements
        .as_mut()
        .unwrap()
        .observations
        .push(stop_region(-0.1));
    let step = guarded.tick(input.at, &input.pose, &input.scan, &input.road);
    assert_eq!(step.command, MotionOutput::Stop, "{step:?}");
    assert!(step.fault.is_none());
    let online = step.online.unwrap();
    assert!(online.execution_space_rejected);
    assert!(online.active_track_id.is_none());
    assert_eq!(online.mission.green_elapsed_ms, 0);
    assert_eq!(online.processed_cones, 0);
    // The marker is no longer visible; forgetting it must not restore Drive.
    for at in [100, 200] {
        let fresh = snapshot(&config, at);
        let step = guarded.tick(fresh.at, &fresh.pose, &fresh.scan, &fresh.road);
        assert_eq!(step.command, MotionOutput::Stop, "{step:?}");
        assert!(step.fault.is_none());
    }
}
#[test]
fn far_first_line_expiry_keeps_its_safe_half_plane_instead_of_blocking_whole_band() {
    let config = controller_config();
    let mut guarded = control(&config);
    for at in (0..=1900).step_by(100) {
        let mut input = snapshot(&config, at);
        if at == 0 {
            input
                .road
                .elements
                .as_mut()
                .unwrap()
                .observations
                .push(stop_region(2.0));
        }
        let step = guarded.tick(input.at, &input.pose, &input.scan, &input.road);
        assert!(step.fault.is_none(), "{step:?}");
        if at == 0 {
            assert!(
                matches!(step.command, MotionOutput::Drive { .. }),
                "{step:?}"
            );
        }
        if at >= config.online.as_ref().unwrap().world.track_ttl_ms {
            assert!(
                matches!(step.command, MotionOutput::Drive { .. }),
                "{step:?}"
            );
            assert!(!step.online.unwrap().execution_space_rejected);
        }
    }
}
#[test]
fn raw_green_does_not_release_pending_stop_line_and_invalid_source_is_rejected() {
    let config = controller_config();
    for invalid in [false, true] {
        let mut guarded = control(&config);
        let mut input = snapshot(&config, 0);
        let mut marker = stop_region(-0.1);
        if invalid {
            marker.source = ObservationSource::VisualLidar;
        }
        input
            .road
            .elements
            .as_mut()
            .unwrap()
            .observations
            .push(marker);
        input.road.observation.light = LightState::Green;
        input.road.observation.light_confidence = 1.0;
        let step = guarded.tick(input.at, &input.pose, &input.scan, &input.road);
        assert_eq!(step.command, MotionOutput::Stop, "{step:?}");
        assert_eq!(step.fault.is_some(), invalid);
    }
}
#[test]
fn sync_raw_elements_reject_same_timestamp_mutation_even_if_cone_fusion_drops_both() {
    let config = controller_config();
    let mut guarded = control(&config);
    let mut input = snapshot(&config, 0);
    let mut cone = stop_region(2.0);
    cone.kind = ElementKind::Cone;
    cone.heading_body_rad = None;
    cone.geometry = ElementGeometry::Cone { radius_m: 0.2 };
    cone.source = ObservationSource::GroundProjection;
    input
        .road
        .elements
        .as_mut()
        .unwrap()
        .observations
        .push(cone);
    let first = guarded.tick(input.at, &input.pose, &input.scan, &input.road);
    assert!(first.fault.is_none(), "{first:?}");
    input.road.elements.as_mut().unwrap().observations[0].color = ElementColor::Blue;
    let changed = guarded.tick(Timestamp(100), &input.pose, &input.scan, &input.road);
    assert_eq!(changed.command, MotionOutput::Stop);
    assert!(
        changed
            .fault
            .as_ref()
            .unwrap()
            .contains("raw elements changed")
    );
}

#[test]
fn expired_confirmed_line_allows_safe_distance_and_unrelated_lateral_lane() {
    let config = controller_config();
    for lateral in [0.0, 3.0] {
        let mut guarded = control(&config);
        for at in (0..=2000).step_by(100) {
            let mut input = snapshot(&config, at);
            if at <= 100 {
                let mut marker = stop_region(2.0);
                marker.position_body_m.y_m = lateral;
                input
                    .road
                    .elements
                    .as_mut()
                    .unwrap()
                    .observations
                    .push(marker);
            }
            let step = guarded.tick(input.at, &input.pose, &input.scan, &input.road);
            assert!(step.fault.is_none(), "{step:?}");
            if at == 100 {
                assert!(step.online.as_ref().unwrap().travel_boundary.is_some());
            }
            if at == 2000 {
                assert!(
                    matches!(step.command, MotionOutput::Drive { .. }),
                    "{step:?}"
                );
                assert!(!step.online.unwrap().execution_space_rejected);
            }
        }
    }
}
#[test]
fn final_async_certificate_rejects_raw_line_hidden_by_processor() {
    use std::sync::Arc;
    use std::time::{Duration, Instant};
    use xt_stcar_robot_runner::control_runtime::{
        AdoptionRejection, AutonomyWorker, CertificateFailureReason, ControlRuntimeConfig,
        SubmitStatus,
    };
    let config = controller_config();
    let mut guarded = control(&config);
    let mut worker = AutonomyWorker::spawn_with_aligned_processor(
        config.clone(),
        ControlRuntimeConfig {
            max_command_age_ms: 200,
            startup_timeout_ms: 200,
        },
        Timestamp(0),
        move |input, context| {
            let mut hidden = input.clone();
            hidden.road.elements.as_mut().unwrap().observations.clear();
            let step = guarded.tick_with_projection(
                input.at,
                &hidden.pose,
                &hidden.scan,
                &hidden.road,
                context,
            );
            assert!(
                matches!(step.command, MotionOutput::Drive { .. }),
                "{step:?}"
            );
            Ok(step)
        },
    )
    .unwrap();
    let mut input = snapshot(&config, 0);
    input
        .road
        .elements
        .as_mut()
        .unwrap()
        .observations
        .push(stop_region(-0.1));
    let input = Arc::new(input);
    let deadline = Instant::now() + Duration::from_secs(3);
    while matches!(
        worker.try_submit(Arc::clone(&input), input.at).unwrap(),
        SubmitStatus::Busy
    ) {
        assert!(Instant::now() < deadline);
        std::thread::yield_now();
    }
    loop {
        let poll = worker.poll(Timestamp(0));
        assert_eq!(poll.command, MotionOutput::Stop);
        assert!(poll.fault.is_none(), "{poll:?}");
        if let Some(rejection) = poll.adoption_rejection {
            assert_eq!(rejection, AdoptionRejection::CertificateUnsafe);
            assert_eq!(
                poll.observed_plan
                    .unwrap()
                    .certificate_failure
                    .as_ref()
                    .unwrap()
                    .reason,
                CertificateFailureReason::LightBoundary
            );
            break;
        }
        assert!(Instant::now() < deadline, "{poll:?}");
        std::thread::sleep(Duration::from_millis(1));
    }
    worker.request_stop();
}
