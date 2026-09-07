use serde_json::{Value, json};
use std::fs;
use std::path::PathBuf;
use std::process::Command;
use tempfile::TempDir;

struct Fixture {
    temp: TempDir,
    config: PathBuf,
    events: PathBuf,
}

impl Fixture {
    fn new(events: &str) -> Self {
        let temp = tempfile::tempdir().unwrap();
        let config = temp.path().join("robot.json");
        let events_path = temp.path().join("events.jsonl");
        let mut spec: Value =
            serde_json::from_str(include_str!("../../../config/robot-sim.json")).unwrap();
        spec["required_sensors"] = json!([]);
        fs::write(&config, serde_json::to_vec(&spec).unwrap()).unwrap();
        fs::write(&events_path, events).unwrap();
        Self {
            temp,
            config,
            events: events_path,
        }
    }

    fn command(&self) -> Command {
        let mut command = Command::new(env!("CARGO_BIN_EXE_xt-stcar-robot"));
        command
            .args(["replay", "--events"])
            .arg(&self.events)
            .arg("--config")
            .arg(&self.config);
        command
    }
}

const DRIVE: &str = r#"{"at":0,"event":{"type":"heartbeat"}}
{"at":0,"event":{"type":"deadman","pressed":true}}
{"at":0,"event":{"type":"arm"}}
{"at":10,"event":{"type":"motion","intent":{"speed_mps":0.1,"curvature_per_m":0.2}}}
{"at":10,"event":{"type":"start"}}
"#;

fn records(bytes: &[u8]) -> Vec<Value> {
    String::from_utf8_lossy(bytes)
        .lines()
        .map(|line| serde_json::from_str(line).unwrap())
        .collect()
}

#[test]
fn successful_replay_records_motion_and_always_stops_at_eof() {
    let fixture = Fixture::new(DRIVE);
    let result = fixture.command().output().unwrap();
    assert!(
        result.status.success(),
        "{}",
        String::from_utf8_lossy(&result.stderr)
    );
    let records = records(&result.stdout);
    assert!(
        records
            .iter()
            .all(|r| r["physical_output_enabled"] == false)
    );
    let summary = records.last().unwrap();
    assert_eq!(summary["input_events"], 5);
    assert_eq!(summary["drive_records"], 1);
    assert_eq!(summary["final_state"], "disarmed");
    assert_eq!(records[5]["terminal"], "end_of_stream");
    assert_eq!(records[5]["step"]["output"]["command"]["type"], "stop");
}

#[test]
fn timeout_is_a_recorded_fault_and_never_recovered_by_late_heartbeat() {
    let fixture = Fixture::new(&format!(
        "{DRIVE}{{\"at\":1000,\"event\":{{\"type\":\"heartbeat\"}}}}\n"
    ));
    let result = fixture.command().output().unwrap();
    assert!(result.status.success());
    let logs = records(&result.stdout);
    assert_eq!(logs[5]["step"]["state"], "fault");
    assert_eq!(logs[5]["step"]["output"]["command"]["type"], "stop");
    assert_eq!(logs.last().unwrap()["final_state"], "fault");
}

#[test]
fn malformed_or_regressing_input_cannot_truncate_existing_logs() {
    for events in [
        "not-json",
        "",
        "{\"at\":2,\"event\":{\"type\":\"tick\"}}\n{\"at\":1,\"event\":{\"type\":\"tick\"}}",
        "{\"at\":0,\"event\":{\"type\":\"tick\",\"extra\":true}}",
    ] {
        let fixture = Fixture::new(events);
        let destination = fixture.temp.path().join("log.jsonl");
        fs::write(&destination, "prior log").unwrap();
        let output = fixture
            .command()
            .arg("--output")
            .arg(&destination)
            .output()
            .unwrap();
        assert!(!output.status.success());
        assert!(output.stdout.is_empty());
        assert_eq!(fs::read_to_string(destination).unwrap(), "prior log");
    }
}

#[test]
fn invalid_config_and_output_input_collision_are_rejected() {
    let fixture = Fixture::new(DRIVE);
    let original = fs::read(&fixture.events).unwrap();
    let result = fixture
        .command()
        .arg("--output")
        .arg(&fixture.events)
        .output()
        .unwrap();
    assert!(!result.status.success());
    assert_eq!(fs::read(&fixture.events).unwrap(), original);
    fs::write(&fixture.config, "{}").unwrap();
    let result = fixture.command().output().unwrap();
    assert!(!result.status.success());
    assert!(result.stdout.is_empty());
}

#[cfg(unix)]
#[test]
fn hard_linked_output_never_truncates_source_events() {
    let fixture = Fixture::new(DRIVE);
    let alias = fixture.temp.path().join("hardlink.jsonl");
    fs::hard_link(&fixture.events, &alias).unwrap();
    let original = fs::read(&fixture.events).unwrap();
    let result = fixture
        .command()
        .arg("--output")
        .arg(&alias)
        .output()
        .unwrap();
    assert!(result.status.success());
    assert_eq!(fs::read(&fixture.events).unwrap(), original);
    assert_eq!(
        records(&fs::read(alias).unwrap()).last().unwrap()["kind"],
        "summary"
    );
}

#[test]
fn frame_stream_requires_native_runtime_and_monotonic_sequence() {
    let fixture = Fixture::new("");
    let image = fixture.temp.path().join("frame.png");
    image::RgbImage::from_pixel(2, 2, image::Rgb([1, 2, 3]))
        .save(&image)
        .unwrap();
    let frame = json!({"at":0,"event":{"type":"vision_frame","path":"frame.png","sequence":1,"frame_id":"sim_camera"}}).to_string();
    fs::write(&fixture.events, format!("{frame}\n")).unwrap();
    let result = fixture.command().output().unwrap();
    assert!(!result.status.success());
    assert!(String::from_utf8_lossy(&result.stderr).contains("--runtime-lib"));
    fs::write(&fixture.events, format!("{frame}\n{frame}\n")).unwrap();
    let result = fixture.command().output().unwrap();
    assert!(!result.status.success());
    assert!(String::from_utf8_lossy(&result.stderr).contains("sequence"));
    // A camera source must not be overwritten by the log output either.
    fs::write(&fixture.events, format!("{frame}\n")).unwrap();
    let original = fs::read(&image).unwrap();
    let result = fixture
        .command()
        .arg("--output")
        .arg(&image)
        .output()
        .unwrap();
    assert!(!result.status.success());
    assert_eq!(fs::read(image).unwrap(), original);
}

#[test]
fn raw_imu_corruption_does_not_refresh_safety_watchdog() {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let result = Command::new(env!("CARGO_BIN_EXE_xt-stcar-robot"))
        .current_dir(&root)
        .args([
            "replay",
            "--events",
            "examples/robot-imu-replay.jsonl",
            "--config",
            "config/robot-imu-replay.json",
            "--imu-config",
            "config/imu-replay.json",
        ])
        .output()
        .unwrap();
    assert!(
        result.status.success(),
        "{}",
        String::from_utf8_lossy(&result.stderr)
    );
    let logs = records(&result.stdout);
    assert_eq!(logs.last().unwrap()["imu_samples"], 1);
    assert_eq!(logs[0]["imu_decode"]["samples"], json!([]));
    assert_eq!(logs[1]["imu_decode"]["samples"][0]["captured_at"], 0);
    assert_eq!(logs[6]["step"]["state"], "running");
    assert_eq!(logs[7]["imu_decode"]["checksum_errors"], 1);
    assert_eq!(logs[8]["step"]["state"], "fault");
    assert_eq!(logs[8]["step"]["output"]["command"]["type"], "stop");
    assert_eq!(logs.last().unwrap()["final_state"], "fault");
}

#[test]
fn raw_imu_requires_explicit_config_and_protects_it_from_output() {
    let fixture = Fixture::new("{\"at\":0,\"event\":{\"type\":\"imu_bytes\",\"bytes\":[85]}}\n");
    assert!(!fixture.command().output().unwrap().status.success());
    let config = fixture.temp.path().join("imu.json");
    fs::write(&config, include_str!("../../../config/imu-replay.json")).unwrap();
    let original = fs::read(&config).unwrap();
    let result = fixture
        .command()
        .arg("--imu-config")
        .arg(&config)
        .arg("--output")
        .arg(&config)
        .output()
        .unwrap();
    assert!(!result.status.success());
    assert_eq!(fs::read(&config).unwrap(), original);
    let mut invalid: Value = serde_json::from_slice(&original).unwrap();
    invalid["frame_id"] = json!("IMU_link");
    fs::write(&config, serde_json::to_vec(&invalid).unwrap()).unwrap();
    let result = fixture
        .command()
        .arg("--imu-config")
        .arg(&config)
        .output()
        .unwrap();
    assert!(!result.status.success());
    assert!(String::from_utf8_lossy(&result.stderr).contains("frame differs"));
}

#[test]
fn chassis_preview_is_explicit_and_never_writes_to_hardware() {
    let result = Command::new(env!("CARGO_BIN_EXE_xt-stcar-robot"))
        .args([
            "chassis-preview",
            "--profile",
            "teleop_pwm_degrees",
            "--linear",
            "1500",
            "--angular",
            "90",
        ])
        .output()
        .unwrap();
    assert!(result.status.success());
    let output: Value = serde_json::from_slice(&result.stdout).unwrap();
    assert_eq!(output["hex"], "AA DC 05 DC 05 C2 55");
    assert_eq!(output["physical_output_enabled"], false);
    let result = Command::new(env!("CARGO_BIN_EXE_xt-stcar-robot"))
        .args([
            "chassis-preview",
            "--profile",
            "navigation1300",
            "--linear",
            "NaN",
            "--angular",
            "0",
        ])
        .output()
        .unwrap();
    assert!(!result.status.success());
    assert!(result.stdout.is_empty());
}
