use image::{Rgb, RgbImage};
use std::sync::Arc;
use std::time::{Duration, Instant};
use xt_stcar_robot_core::autonomy::LightState;
use xt_stcar_robot_core::local_world::{ElementFrame, ElementKind, MAX_ELEMENTS};
use xt_stcar_robot_core::{FrameId, Timestamp};
use xt_stcar_robot_runner::perception::{
    PerceptionWorker, RoadPipeline, SubmitStatus, WorkerLimits,
};
use xt_stcar_robot_runner::simulation::{SimulationConfig, render_camera};
use xt_stcar_vision::ground_markers::ExperimentalMarkerConfig;

fn frame() -> FrameId {
    FrameId("sim_body".into())
}
fn synthetic() -> (SimulationConfig, RgbImage) {
    let config = SimulationConfig::example();
    let image = render_camera(&config, config.initial_pose, LightState::Green).unwrap();
    (config, image)
}
fn wait(
    worker: &PerceptionWorker,
    at: u64,
) -> Arc<xt_stcar_robot_runner::perception::PerceptionResult> {
    let deadline = Instant::now() + Duration::from_secs(3);
    loop {
        if let Ok(Some(result)) = worker.snapshot()
            && result.captured_at == Timestamp(at)
        {
            return result;
        }
        assert!(
            Instant::now() < deadline,
            "perception worker did not publish"
        );
        std::thread::sleep(Duration::from_millis(1));
    }
}

#[test]
fn online_rgb_pipeline_adds_oriented_elements_and_legacy_pipeline_keeps_none() {
    let (config, image) = synthetic();
    let mut legacy = RoadPipeline::new(config.road.clone(), None).unwrap();
    let old = legacy.process(&image, Timestamp(9), frame()).unwrap();
    assert!(old.elements.is_none());
    let mut online =
        RoadPipeline::new_online(config.road, None, ExperimentalMarkerConfig::default()).unwrap();
    let new = online.process(&image, Timestamp(9), frame()).unwrap();
    assert_eq!(
        serde_json::to_value(&old.observation).unwrap(),
        serde_json::to_value(&new.observation).unwrap()
    );
    let elements = new.elements.unwrap();
    assert_eq!(elements.captured_at, Timestamp(9));
    assert_eq!(elements.frame_id, frame());
    assert!(elements.observations.len() <= MAX_ELEMENTS);
    assert!(elements.observations.capacity() <= MAX_ELEMENTS);
    let crossing = elements
        .observations
        .iter()
        .find(|o| o.kind == ElementKind::Crosswalk)
        .expect("synthetic papers");
    assert!(crossing.heading_body_rad.unwrap().abs() < crossing.heading_error_rad);
    assert!(
        elements
            .observations
            .iter()
            .all(|o| !matches!(o.kind, ElementKind::StopLine | ElementKind::FinishMarker))
    );
    let reversed = online
        .process_with_heading(&image, Timestamp(10), frame(), std::f64::consts::PI)
        .unwrap();
    let crossing = reversed
        .elements
        .unwrap()
        .observations
        .into_iter()
        .find(|o| o.kind == ElementKind::Crosswalk)
        .unwrap();
    assert!(
        (crossing.heading_body_rad.unwrap().abs() - std::f64::consts::PI).abs()
            < crossing.heading_error_rad
    );
}

#[test]
fn persistent_worker_publishes_explicit_rgb_marker_and_missing_frame_stays_empty() {
    let (mut config, _) = synthetic();
    config.crosswalk.min_x_m = 20.;
    config.crosswalk.max_x_m = 21.;
    config.cones.clear();
    let mut image = render_camera(&config, config.initial_pose, LightState::Unknown).unwrap();
    // One declared transverse bar, centered at x=1.68 in body coordinates.
    // There is no landmark position argument to RoadPipeline.
    for (x, y, pixel) in image.enumerate_pixels_mut() {
        let forward = 5. * (1. - (f64::from(y) + 0.5) / 240.);
        let lateral = 2. - 4. * (f64::from(x) + 0.5) / 320.;
        if (forward - 1.68).abs() < 0.04 && lateral.abs() < 0.3 {
            *pixel = Rgb([245, 245, 245]);
        }
    }
    let pipeline =
        RoadPipeline::new_online(config.road, None, ExperimentalMarkerConfig::simulation())
            .unwrap();
    let worker = PerceptionWorker::spawn(pipeline, WorkerLimits::default()).unwrap();
    assert_eq!(
        worker
            .try_submit(Arc::new(image), Timestamp(12), frame())
            .unwrap(),
        SubmitStatus::Queued
    );
    let result = wait(&worker, 12);
    let elements = result.result.as_ref().unwrap().elements.as_ref().unwrap();
    assert_eq!(elements.observations.len(), 1);
    assert_eq!(elements.observations[0].kind, ElementKind::StopLine);
    assert_eq!(
        elements.observations[0].geometry,
        xt_stcar_robot_core::local_world::ElementGeometry::LineRegion {
            lateral_half_width_m: 0.4,
            depth_m: 1.0,
        }
    );
    assert!(
        (elements.observations[0].position_body_m.x_m - 0.8).abs()
            < elements.observations[0].position_error_m
    );
    assert!(elements.observations.capacity() <= MAX_ELEMENTS);
    assert_eq!(
        worker
            .try_submit(
                Arc::new(RgbImage::from_pixel(320, 240, Rgb([35, 35, 35]))),
                Timestamp(13),
                frame()
            )
            .unwrap(),
        SubmitStatus::Queued
    );
    assert!(
        wait(&worker, 13)
            .result
            .as_ref()
            .unwrap()
            .elements
            .as_ref()
            .unwrap()
            .observations
            .is_empty()
    );
}

#[test]
fn worker_rejects_element_metadata_changes_and_compacts_custom_allocation() {
    for wrong_stamp in [false, true] {
        let (config, image) = synthetic();
        let mut pipeline = RoadPipeline::new(config.road, None).unwrap();
        let worker = PerceptionWorker::spawn_with_processor(
            WorkerLimits::default(),
            move |image, at, id| {
                let mut frame = pipeline.process(image, at, id.clone())?;
                frame.elements = Some(ElementFrame {
                    captured_at: Timestamp(at.0 + u64::from(wrong_stamp)),
                    frame_id: id,
                    observations: Vec::with_capacity(1024),
                });
                Ok(frame)
            },
        )
        .unwrap();
        assert_eq!(
            worker
                .try_submit(Arc::new(image), Timestamp(7), frame())
                .unwrap(),
            SubmitStatus::Queued
        );
        let result = wait(&worker, 7);
        if wrong_stamp {
            assert!(result.result.as_ref().unwrap_err().contains("metadata"));
            assert!(worker.is_stopped());
        } else {
            assert!(
                result
                    .result
                    .as_ref()
                    .unwrap()
                    .elements
                    .as_ref()
                    .unwrap()
                    .observations
                    .capacity()
                    <= MAX_ELEMENTS
            );
        }
    }
}
