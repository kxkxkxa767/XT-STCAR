use serde_json::{Value, json};
use std::{fs, process::Command};
use xt_stcar_robot_core::Timestamp;
use xt_stcar_robot_core::autonomy::{LightState, Pose2, PoseEstimate};
use xt_stcar_robot_core::localization::LocalizationConfig;
use xt_stcar_robot_runner::autonomy::RoadFrame;
use xt_stcar_robot_runner::autonomy_replay::{SensorSnapshot, read_snapshots, replay_snapshots};
use xt_stcar_robot_runner::laser_pose::LaserPosePipeline;
use xt_stcar_robot_runner::simulation::{SimulationConfig, render_camera, synthetic_scan};
use xt_stcar_robot_runner::telemetry::TelemetryConfig;
use xt_stcar_vision::road::RoadDetector;

fn snapshot(c: &SimulationConfig, ms: u64) -> SensorSnapshot {
    let at = Timestamp(ms);
    let rgb = render_camera(c, c.initial_pose, LightState::Red).unwrap();
    SensorSnapshot {
        at,
        pose: PoseEstimate {
            captured_at: at,
            frame_id: c.autonomy.mission.world_frame.clone(),
            pose: c.initial_pose,
            speed_mps: 0.0,
            yaw_rate_radps: 0.0,
            quality: 1.0,
        },
        scan: synthetic_scan(c, c.initial_pose, &c.cones, at),
        road: RoadFrame {
            observation: RoadDetector::new(c.road.clone())
                .unwrap()
                .detect(&rgb, &[], at, c.autonomy.mission.body_frame.clone())
                .unwrap(),
            image_width_px: rgb.width(),
            image_height_px: rgb.height(),
        },
    }
}

#[test]
fn full_scan_tf_and_odometry_preserve_source_time_and_reject_unknown_coverage() {
    let mut c = SimulationConfig::example();
    c.autonomy.lidar_in_body = Pose2 {
        x_m: 0.1,
        y_m: 0.03,
        yaw_rad: 0.2,
    };
    let anchor = Pose2 {
        x_m: 2.0,
        y_m: 2.3,
        yaw_rad: 0.1,
    };
    let mut pipeline = LaserPosePipeline::new(
        c.autonomy.scan.clone(),
        LocalizationConfig::simulation(c.autonomy.mission.world_frame.clone()),
        c.autonomy.lidar_in_body,
        anchor,
    )
    .unwrap();
    assert!(
        pipeline
            .update(&synthetic_scan(&c, anchor, &c.cones, Timestamp(0)))
            .unwrap()
            .estimate
            .is_none()
    );
    let next = Pose2 {
        x_m: 2.012,
        y_m: 2.304,
        yaw_rad: 0.105,
    };
    let scan = synthetic_scan(&c, next, &c.cones, Timestamp(100));
    let update = pipeline.update(&scan).unwrap();
    assert!(update.accepted, "{update:?}");
    let estimate = update.estimate.unwrap();
    assert_eq!(estimate.captured_at, Timestamp(100));
    assert_eq!(estimate.frame_id, c.autonomy.mission.world_frame);
    assert!(
        estimate.pose.point().distance(next.point()) < 0.02,
        "{estimate:?}"
    );
    assert!((estimate.pose.yaw_rad - next.yaw_rad).abs() < 0.02);
    let mut broken = scan.clone();
    broken.captured_at = Timestamp(200);
    broken.ranges_m[0..40].fill(None);
    assert!(pipeline.update(&broken).is_err());
    let lost = pipeline
        .update(&synthetic_scan(&c, next, &c.cones, Timestamp(400)))
        .unwrap();
    assert!(!lost.accepted);
    assert!(lost.estimate.is_none());
}

#[test]
fn typed_sensor_replay_computes_motion_and_records_stop_at_incomplete_eof() {
    let c = SimulationConfig::example();
    let records = [snapshot(&c, 0), snapshot(&c, 100)];
    let temp = tempfile::tempdir().unwrap();
    let path = temp.path().join("sensors.jsonl");
    fs::write(
        &path,
        records
            .iter()
            .map(|s| serde_json::to_string(s).unwrap() + "\n")
            .collect::<String>(),
    )
    .unwrap();
    let decoded = read_snapshots(&path).unwrap();
    let mut log = Vec::new();
    let summary =
        replay_snapshots(c.autonomy, &decoded, TelemetryConfig::default(), &mut log).unwrap();
    assert!(!summary.completed);
    assert_eq!(summary.processed_snapshots, 2);
    assert!(summary.fault.unwrap().contains("ended before"));
    let rows: Vec<Value> = std::str::from_utf8(&log)
        .unwrap()
        .lines()
        .map(|l| serde_json::from_str(l).unwrap())
        .collect();
    assert_eq!(rows[0]["command"]["type"], "drive");
    assert_eq!(rows[rows.len() - 2]["command"]["type"], "stop");
    assert_eq!(rows.last().unwrap()["physical_output_enabled"], false);
}

#[test]
fn sensor_adapter_rejects_motion_fields_duplicate_times_and_nonregular_files() {
    let c = SimulationConfig::example();
    let temp = tempfile::tempdir().unwrap();
    let path = temp.path().join("input.jsonl");
    let record = serde_json::to_value(snapshot(&c, 0)).unwrap();
    let mut command = record.clone();
    command["event"] = json!({"type":"motion","intent":{"speed_mps":1,"curvature_per_m":0}});
    for text in [
        format!("{command}\n"),
        format!("{record}\n{record}\n"),
        String::new(),
    ] {
        fs::write(&path, text).unwrap();
        assert!(read_snapshots(&path).is_err());
    }
    assert!(read_snapshots(temp.path()).is_err());
}

#[test]
fn cli_fault_output_is_atomic_and_flags_cannot_overwrite_inputs() {
    let temp = tempfile::tempdir().unwrap();
    let cfg = temp.path().join("sim.json");
    let log = temp.path().join("run.jsonl");
    let mut c = SimulationConfig::example();
    c.fault = xt_stcar_robot_runner::simulation::SimulationFault::PoseDropout;
    c.fault_at_ms = 1000;
    let original = serde_json::to_vec(&c).unwrap();
    fs::write(&cfg, &original).unwrap();
    let run = |extra: &[&str]| {
        Command::new(env!("CARGO_BIN_EXE_xt-stcar-robot"))
            .args(["autonomy-sim", "--config"])
            .arg(&cfg)
            .args(extra)
            .output()
            .unwrap()
    };
    for args in [
        vec!["--output", cfg.to_str().unwrap()],
        vec!["--trace", "--trace"],
        vec!["--unknown", "x"],
        vec!["--image", "x"],
    ] {
        assert!(!run(&args).status.success());
        assert_eq!(fs::read(&cfg).unwrap(), original);
    }
    fs::write(&log, "prior log").unwrap();
    let result = run(&["--output", log.to_str().unwrap()]);
    assert!(!result.status.success());
    assert!(result.stdout.is_empty());
    let rows: Vec<Value> = fs::read_to_string(&log)
        .unwrap()
        .lines()
        .map(|l| serde_json::from_str(l).unwrap())
        .collect();
    assert_eq!(rows[rows.len() - 2]["command"]["type"], "stop");
    assert_eq!(rows.last().unwrap()["final_actual_speed_mps"], 0.0);
    let summary_only = run(&[]);
    assert_eq!(
        String::from_utf8(summary_only.stdout)
            .unwrap()
            .lines()
            .count(),
        1
    );
    fs::write(&cfg, "{}").unwrap();
    fs::write(&log, "prior log").unwrap();
    assert!(!run(&["--output", log.to_str().unwrap()]).status.success());
    assert_eq!(fs::read_to_string(&log).unwrap(), "prior log");
}

#[test]
fn road_cli_runs_rust_rgb_detection_and_protects_the_image() {
    let c = SimulationConfig::example();
    let temp = tempfile::tempdir().unwrap();
    let cfg = temp.path().join("road.json");
    let image = temp.path().join("frame.png");
    fs::write(&cfg, serde_json::to_vec(&c.road).unwrap()).unwrap();
    render_camera(&c, c.initial_pose, LightState::Green)
        .unwrap()
        .save(&image)
        .unwrap();
    let run = |extra: &[&str]| {
        Command::new(env!("CARGO_BIN_EXE_xt-stcar-robot"))
            .args(["road-detect", "--config"])
            .arg(&cfg)
            .arg("--image")
            .arg(&image)
            .args(extra)
            .output()
            .unwrap()
    };
    let result = run(&[]);
    assert!(
        result.status.success(),
        "{}",
        String::from_utf8_lossy(&result.stderr)
    );
    let value: Value = serde_json::from_slice(&result.stdout).unwrap();
    assert_eq!(value["road"]["observation"]["light"], "green");
    assert!(value["road"]["observation"]["crosswalk"].is_object());
    assert_eq!(value["native_yolo_enabled"], false);
    let original = fs::read(&image).unwrap();
    assert!(!run(&["--output", image.to_str().unwrap()]).status.success());
    assert_eq!(fs::read(&image).unwrap(), original);
}

#[test]
fn shipped_configuration_files_form_a_valid_offline_pipeline() {
    use xt_stcar_robot_core::scan::ScanConfig;
    use xt_stcar_robot_runner::autonomy::{AutonomyConfig, AutonomyController};
    let scene: SimulationConfig =
        serde_json::from_str(include_str!("../../../config/competition-sim.json")).unwrap();
    scene.validate().unwrap();
    let mut controller: AutonomyConfig = serde_json::from_str(include_str!(
        "../../../config/competition-controller-sim.json"
    ))
    .unwrap();
    AutonomyController::new(controller.clone()).unwrap();
    let road =
        serde_json::from_str(include_str!("../../../config/road-perception-sim.json")).unwrap();
    RoadDetector::new(road).unwrap();
    let scan: ScanConfig =
        serde_json::from_str(include_str!("../../../config/scan-assembly-sim.json")).unwrap();
    let local: LocalizationConfig =
        serde_json::from_str(include_str!("../../../config/laser-localization-sim.json")).unwrap();
    assert_eq!(local.frame_id, controller.mission.world_frame);
    assert_eq!(scan.frame_id, controller.scan.frame_id);
    LaserPosePipeline::new(scan, local, controller.lidar_in_body, scene.initial_pose).unwrap();
    controller.simulation_only = false;
    assert!(AutonomyController::new(controller).is_err());
}
