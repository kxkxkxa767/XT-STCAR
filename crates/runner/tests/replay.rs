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
