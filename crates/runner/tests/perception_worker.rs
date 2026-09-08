use image::{Rgb, RgbImage};
use std::sync::Arc;
use std::sync::mpsc::{Receiver, Sender, channel};
use std::thread;
use std::time::{Duration, Instant};
use xt_stcar_robot_core::autonomy::{LightState, RoadObservation};
use xt_stcar_robot_core::{FrameId, Timestamp};
use xt_stcar_robot_runner::autonomy::RoadFrame;
use xt_stcar_robot_runner::perception::{
    PerceptionResult, PerceptionWorker, RoadPipeline, SubmitStatus, WorkerLimits,
};
use xt_stcar_vision::road::RoadConfig;

fn frame_id() -> FrameId {
    FrameId("base_link".into())
}

fn image() -> Arc<RgbImage> {
    Arc::new(RgbImage::from_pixel(32, 24, Rgb([30, 30, 30])))
}

fn report(image: &RgbImage, at: Timestamp, frame_id: FrameId) -> RoadFrame {
    RoadFrame {
        observation: RoadObservation {
            captured_at: at,
            frame_id,
            crosswalk: None,
            light: LightState::Unknown,
            light_confidence: 0.0,
            cones_body_m: Vec::new(),
        },
        image_width_px: image.width(),
        image_height_px: image.height(),
    }
}

fn submit(worker: &PerceptionWorker, image: &Arc<RgbImage>, at: u64) -> SubmitStatus {
    let deadline = Instant::now() + Duration::from_secs(2);
    loop {
        let status = worker
            .try_submit(Arc::clone(image), Timestamp(at), frame_id())
            .unwrap();
        if status != SubmitStatus::Busy {
            return status;
        }
        assert!(Instant::now() < deadline, "mailbox remained busy");
        thread::yield_now();
    }
}

fn wait_result(worker: &PerceptionWorker, at: u64) -> Arc<PerceptionResult> {
    let deadline = Instant::now() + Duration::from_secs(2);
    loop {
        if let Ok(Some(result)) = worker.snapshot()
            && result.captured_at == Timestamp(at)
        {
            return result;
        }
        assert!(Instant::now() < deadline, "missing result for {at}");
        thread::sleep(Duration::from_millis(1));
    }
}

/// recv_timeout keeps failed regressions from leaving the test's processor hung.
fn block_processing(started: &Sender<u64>, release: &Receiver<()>, at: Timestamp) {
    started.send(at.0).unwrap();
    release.recv_timeout(Duration::from_secs(5)).unwrap();
}

#[test]
fn blocked_processor_does_not_block_submission_and_only_latest_pending_frame_survives() {
    let (started_tx, started_rx) = channel();
    let (release_tx, release_rx) = channel();
    let worker = PerceptionWorker::spawn_with_processor(
        WorkerLimits::default(),
        move |image, at, frame_id| {
            block_processing(&started_tx, &release_rx, at);
            Ok(report(image, at, frame_id))
        },
    )
    .unwrap();
    let active = image();
    assert_eq!(submit(&worker, &active, 7), SubmitStatus::Queued);
    assert_eq!(started_rx.recv_timeout(Duration::from_secs(2)).unwrap(), 7);
    let pending = image();
    assert_eq!(submit(&worker, &pending, 8), SubmitStatus::Queued);
    let latest = image();
    let started = Instant::now();
    for at in 9..=500 {
        assert_eq!(submit(&worker, &latest, at), SubmitStatus::Replaced);
        assert!(worker.snapshot().unwrap().is_none());
    }
    assert!(started.elapsed() < Duration::from_millis(500));
    assert_eq!(
        Arc::strong_count(&pending),
        1,
        "replaced image was retained"
    );
    assert_eq!(Arc::strong_count(&latest), 2, "queue accumulated images");
    release_tx.send(()).unwrap();
    assert_eq!(
        started_rx.recv_timeout(Duration::from_secs(2)).unwrap(),
        500
    );
    let old_result = wait_result(&worker, 7);
    assert_eq!(
        old_result.result.as_ref().unwrap().observation.captured_at,
        Timestamp(7)
    );
    // A later frame remains active while the previous capture stays visible.
    assert!(!worker.is_stopped());
    release_tx.send(()).unwrap();
    let new_result = wait_result(&worker, 500);
    assert_eq!(
        new_result.result.as_ref().unwrap().observation.captured_at,
        Timestamp(500)
    );
    assert_eq!(new_result.frame_id, frame_id());
    assert!(started_rx.recv_timeout(Duration::from_millis(30)).is_err());
}

#[test]
fn processor_error_is_bounded_latched_and_retains_capture_identity() {
    let worker = PerceptionWorker::spawn_with_processor(WorkerLimits::default(), |_, _, _| {
        Err("模型加载失败".repeat(1000))
    })
    .unwrap();
    assert_eq!(submit(&worker, &image(), 41), SubmitStatus::Queued);
    let result = wait_result(&worker, 41);
    let error = result.result.as_ref().unwrap_err();
    assert!(error.contains("模型加载失败"));
    assert!(error.len() <= 1024);
    assert_eq!(result.frame_id, frame_id());
    assert!(worker.is_stopped());
    assert_eq!(submit(&worker, &image(), 42), SubmitStatus::Stopped);
    assert!(Arc::ptr_eq(&result, &worker.snapshot().unwrap().unwrap()));
}

#[test]
fn incorrect_processor_timestamp_and_panics_are_visible_errors() {
    for panics in [false, true] {
        let worker = PerceptionWorker::spawn_with_processor(
            WorkerLimits::default(),
            move |image, at, frame_id| {
                assert!(!panics, "synthetic processor panic");
                Ok(report(image, Timestamp(at.0 + 1), frame_id))
            },
        )
        .unwrap();
        submit(&worker, &image(), 10);
        let result = wait_result(&worker, 10);
        let error = result.result.as_ref().unwrap_err();
        assert!(error.contains(if panics { "panicked" } else { "metadata" }));
        assert!(worker.is_stopped());
    }
}

#[test]
fn invalid_frames_and_regressing_captures_never_enter_the_queue() {
    let worker = PerceptionWorker::spawn_with_processor(
        WorkerLimits {
            max_pixels: 32 * 24,
            max_frame_bytes: 32 * 24 * 3,
        },
        |image, at, frame_id| Ok(report(image, at, frame_id)),
    )
    .unwrap();
    for invalid in [
        RgbImage::new(0, 0),
        RgbImage::new(33, 24),
        RgbImage::from_raw(1, 1, vec![0; 4]).unwrap(),
        RgbImage::from_raw(1, 1, {
            let mut storage = Vec::with_capacity(32 * 24 * 3 + 1);
            storage.extend([0, 0, 0]);
            storage
        })
        .unwrap(),
    ] {
        assert!(
            worker
                .try_submit(Arc::new(invalid), Timestamp(1), frame_id())
                .is_err()
        );
    }
    assert!(
        worker
            .try_submit(image(), Timestamp(1), FrameId(String::new()))
            .is_err()
    );
    submit(&worker, &image(), 20);
    wait_result(&worker, 20);
    assert!(
        worker
            .try_submit(image(), Timestamp(19), frame_id())
            .is_err()
    );
    assert_eq!(
        worker.snapshot().unwrap().unwrap().captured_at,
        Timestamp(20)
    );
    assert!(
        PerceptionWorker::spawn_with_processor(
            WorkerLimits {
                max_pixels: 0,
                max_frame_bytes: 3
            },
            |image, at, frame_id| Ok(report(image, at, frame_id)),
        )
        .is_err()
    );
}

#[test]
fn drop_during_blocked_processing_detaches_and_idle_drop_releases_processor() {
    let (started_tx, started_rx) = channel();
    let (release_tx, release_rx) = channel();
    let (exited_tx, exited_rx) = channel();
    let worker = PerceptionWorker::spawn_with_processor(
        WorkerLimits::default(),
        move |image, at, frame_id| {
            block_processing(&started_tx, &release_rx, at);
            exited_tx.send(()).unwrap();
            Ok(report(image, at, frame_id))
        },
    )
    .unwrap();
    let active = image();
    submit(&worker, &active, 1);
    started_rx.recv_timeout(Duration::from_secs(2)).unwrap();
    let start = Instant::now();
    drop(worker);
    assert!(start.elapsed() < Duration::from_millis(250));
    assert_eq!(
        Arc::strong_count(&active),
        2,
        "active processor should still own its frame"
    );
    release_tx.send(()).unwrap();
    exited_rx.recv_timeout(Duration::from_secs(2)).unwrap();
    let deadline = Instant::now() + Duration::from_secs(2);
    while Arc::strong_count(&active) != 1 {
        assert!(
            Instant::now() < deadline,
            "detached worker did not exit after release"
        );
        thread::yield_now();
    }

    struct NotifyDrop(Sender<()>);
    impl Drop for NotifyDrop {
        fn drop(&mut self) {
            let _ = self.0.send(());
        }
    }
    let (drop_tx, drop_rx) = channel();
    let guard = NotifyDrop(drop_tx);
    let idle = PerceptionWorker::spawn_with_processor(
        WorkerLimits::default(),
        move |image, at, frame_id| {
            let _keep_guard = &guard;
            Ok(report(image, at, frame_id))
        },
    )
    .unwrap();
    drop(idle);
    drop_rx.recv_timeout(Duration::from_secs(2)).unwrap();
}

#[test]
fn real_rgb_pipeline_detects_rules_papers_and_explicit_light_without_ort() {
    let mut image = RgbImage::from_pixel(320, 240, Rgb([30, 30, 30]));
    for (x, y, pixel) in image.enumerate_pixels_mut() {
        let u = (f64::from(x) + 0.5) / 320.;
        let v = (f64::from(y) + 0.5) / 240.;
        for index in 0..8 {
            let left = 0.25 + f64::from(index) * 0.21 / 4.;
            if (left..left + 0.105 / 4.).contains(&u)
                && (1. - 1.797 / 5. ..1. - 1.5 / 5.).contains(&v)
            {
                *pixel = Rgb([245, 245, 245]);
            }
        }
        if (f64::from(x) + 0.5 - 264.).hypot(f64::from(y) + 0.5 - 36.) < 9. {
            *pixel = Rgb([0, 255, 0]);
        }
    }
    let mut road = RoadConfig::simulation();
    road.light_rois = vec![[0.7, 0.03, 0.95, 0.3]];
    let mut pipeline = RoadPipeline::new(road, None).unwrap();
    let frame = pipeline
        .process(&image, Timestamp(123), frame_id())
        .unwrap();
    assert_eq!(frame.observation.light, LightState::Green);
    let crossing = frame.observation.crosswalk.unwrap();
    assert!((crossing.near_edge_m - 1.5).abs() < 0.045);
    assert!((crossing.far_edge_m - 1.797).abs() < 0.045);
    assert_eq!((frame.image_width_px, frame.image_height_px), (320, 240));
    assert_eq!(frame.observation.captured_at, Timestamp(123));

    let worker = PerceptionWorker::spawn(pipeline, WorkerLimits::default()).unwrap();
    submit(&worker, &Arc::new(image.clone()), 321);
    let latest = wait_result(&worker, 321);
    assert_eq!(
        latest.result.as_ref().unwrap().observation.light,
        LightState::Green
    );

    let mut no_roi = RoadPipeline::new(RoadConfig::simulation(), None).unwrap();
    assert_eq!(
        no_roi
            .process(&image, Timestamp(1), frame_id())
            .unwrap()
            .observation
            .light,
        LightState::Unknown
    );
}
