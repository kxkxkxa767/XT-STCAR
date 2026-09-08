use serde_json::{Value, json};
use std::fs;
use std::path::PathBuf;
use std::process::{Command, Output};
use tempfile::TempDir;

struct Fixture {
    temp: TempDir,
    robot_config: PathBuf,
    n10_config: PathBuf,
    events: PathBuf,
}

impl Fixture {
    fn new() -> Self {
        let temp = tempfile::tempdir().unwrap();
        let robot_config = temp.path().join("robot.json");
        let n10_config = temp.path().join("n10.json");
        let events = temp.path().join("events.jsonl");
        fs::write(
            &robot_config,
            include_str!("../../../config/robot-n10-replay.json"),
        )
        .unwrap();
        fs::write(&n10_config, include_str!("../../../config/n10-replay.json")).unwrap();
        fs::write(
            &events,
            include_str!("../../../examples/robot-n10-replay.jsonl"),
        )
        .unwrap();
        Self {
            temp,
            robot_config,
            n10_config,
            events,
        }
    }

    fn command_without_n10(&self) -> Command {
        let mut command = Command::new(env!("CARGO_BIN_EXE_xt-stcar-robot"));
        command
            .args(["replay", "--events"])
            .arg(&self.events)
            .arg("--config")
            .arg(&self.robot_config);
        command
    }

    fn command(&self) -> Command {
        let mut command = self.command_without_n10();
        command.arg("--n10-config").arg(&self.n10_config);
        command
    }
}

fn records(output: &Output) -> Vec<Value> {
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    String::from_utf8_lossy(&output.stdout)
        .lines()
        .map(|line| serde_json::from_str(line).unwrap())
        .collect()
}

fn rejected(output: Output, message: &str) {
    assert!(!output.status.success());
    assert!(output.stdout.is_empty());
    assert!(
        String::from_utf8_lossy(&output.stderr).contains(message),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
}

#[test]
fn synthetic_fragmented_n10_replay_preserves_metadata_and_stops_at_exact_timeout() {
    let fixture = Fixture::new();
    let logs = records(&fixture.command().output().unwrap());
    assert_eq!(logs.len(), 12); // Eleven motion records plus one summary.
    assert!(
        logs.iter()
            .all(|record| record["physical_output_enabled"] == false)
    );
    let summary = logs.last().unwrap();
    assert_eq!(summary["input_events"], 10);
    assert_eq!(summary["motion_records"], 11);
    assert_eq!(summary["drive_records"], 3);
    assert_eq!(summary["stop_records"], 8);
    assert_eq!(summary["n10_packets"], 1);
    assert_eq!(summary["lidar_samples"], 1);
    assert_eq!(summary["final_state"], "fault");
    assert_eq!(logs[0]["n10_decode"]["packets"], json!([]));
    assert_eq!(logs[1]["lidar_coverage"], "partial_packet");
    let packet = &logs[1]["n10_decode"]["packets"][0];
    assert_eq!(packet["first_received_at"], 0);
    assert_eq!(packet["received_at"], 5);
    assert_eq!(packet["frame_id"], "laser_link");
    assert_eq!(packet["start_angle_cdeg"], 35000);
    assert_eq!(packet["end_angle_cdeg"], 500);
    assert_eq!(packet["header_bytes"], json!([18, 52]));
    let points = packet["points"].as_array().unwrap();
    assert_eq!(points.len(), 16);
    for (slot, point) in points.iter().enumerate() {
        assert_eq!(point["raw_distance_mm"], 1000 + 100 * slot);
        assert_eq!(point["range_m"], (1000 + 100 * slot) as f64 / 1000.0);
        assert_eq!(point["intensity"], slot);
    }
    assert_eq!(logs[6]["step"]["state"], "running");
    assert_eq!(logs[7]["step"]["event_at"], 40);
    assert_eq!(logs[7]["n10_decode"]["checksum_errors"], 1);
    assert_eq!(logs[7]["n10_decode"]["packets"], json!([]));
    assert_eq!(logs[8]["step"]["event_at"], 79);
    assert_eq!(logs[8]["step"]["state"], "running");
    // Neither completion at t=5 nor a corrupt frame at t=40 refreshes t=0.
    assert_eq!(logs[9]["step"]["event_at"], 80);
    assert_eq!(logs[9]["step"]["state"], "fault");
    assert_eq!(logs[9]["step"]["output"]["command"]["type"], "stop");
    assert_eq!(
        logs[9]["step"]["output"]["reason"],
        json!({"type":"sensor_expired","sensor":"lidar"})
    );
    assert_eq!(logs[10]["terminal"], "end_of_stream");
    assert_eq!(logs[10]["step"]["output"]["command"]["type"], "stop");
}

#[test]
fn n10_config_is_required_and_cannot_silently_apply_to_other_input() {
    let fixture = Fixture::new();
    rejected(
        fixture.command_without_n10().output().unwrap(),
        "requires --n10-config",
    );
    fs::write(
        &fixture.events,
        "{\"at\":0,\"event\":{\"type\":\"tick\"}}\n",
    )
    .unwrap();
    rejected(
        fixture.command().output().unwrap(),
        "option requires n10_bytes",
    );
}

#[test]
fn n10_frame_mismatch_is_rejected_before_creating_any_output() {
    let fixture = Fixture::new();
    let mut config: Value =
        serde_json::from_slice(&fs::read(&fixture.n10_config).unwrap()).unwrap();
    config["frame_id"] = json!("another_laser");
    fs::write(&fixture.n10_config, serde_json::to_vec(&config).unwrap()).unwrap();
    rejected(fixture.command().output().unwrap(), "frame differs");
}

#[test]
fn decoded_lidar_cannot_be_mixed_with_raw_n10_even_when_frames_match() {
    let fixture = Fixture::new();
    let direct = json!({"at":0,"event":{"type":"sensor","sample":{
        "type":"lidar","captured_at":0,"frame_id":"laser_link",
        "angle_min_rad":0.0,"angle_increment_rad":0.1,"range_min_m":0.02,
        "range_max_m":12.0,"ranges_m":[1.0]
    }}});
    let source = fs::read_to_string(&fixture.events).unwrap();
    fs::write(&fixture.events, format!("{direct}\n{source}")).unwrap();
    rejected(
        fixture.command().output().unwrap(),
        "cannot mix raw N10 and decoded lidar",
    );
}

#[test]
fn malformed_n10_envelopes_preserve_existing_output() {
    let fixture = Fixture::new();
    let destination = fixture.temp.path().join("old-log.jsonl");
    for event in [
        json!({"at":0,"event":{"type":"n10_bytes","bytes":[]}}),
        json!({"at":0,"event":{"type":"n10_bytes","bytes":[256]}}),
        json!({"at":0,"event":{"type":"n10_bytes","bytes":[-1]}}),
        json!({"at":0,"event":{"type":"n10_bytes","bytes":vec![0;4097]}}),
        json!({"at":0,"event":{"type":"n10_bytes","bytes":[165],"unexpected":true}}),
        json!({"at":0,"unexpected":true,"event":{"type":"n10_bytes","bytes":[165]}}),
    ] {
        fs::write(&fixture.events, format!("{event}\n")).unwrap();
        fs::write(&destination, "prior complete log\n").unwrap();
        let result = fixture
            .command()
            .arg("--output")
            .arg(&destination)
            .output()
            .unwrap();
        assert!(
            !result.status.success(),
            "accepted malformed event: {event}"
        );
        assert!(result.stdout.is_empty());
        assert_eq!(
            fs::read_to_string(&destination).unwrap(),
            "prior complete log\n"
        );
    }
}

#[test]
fn output_cannot_replace_the_n10_configuration() {
    let fixture = Fixture::new();
    let original = fs::read(&fixture.n10_config).unwrap();
    rejected(
        fixture
            .command()
            .arg("--output")
            .arg(&fixture.n10_config)
            .output()
            .unwrap(),
        "must not overwrite",
    );
    assert_eq!(fs::read(&fixture.n10_config).unwrap(), original);
}

#[cfg(unix)]
#[test]
fn output_aliases_never_modify_the_n10_configuration_inode() {
    let fixture = Fixture::new();
    let original = fs::read(&fixture.n10_config).unwrap();
    let symlink = fixture.temp.path().join("n10-symlink.json");
    std::os::unix::fs::symlink(&fixture.n10_config, &symlink).unwrap();
    rejected(
        fixture
            .command()
            .arg("--output")
            .arg(&symlink)
            .output()
            .unwrap(),
        "must not overwrite",
    );
    assert_eq!(fs::read(&fixture.n10_config).unwrap(), original);
    let hardlink = fixture.temp.path().join("n10-hardlink.json");
    fs::hard_link(&fixture.n10_config, &hardlink).unwrap();
    let result = fixture
        .command()
        .arg("--output")
        .arg(&hardlink)
        .output()
        .unwrap();
    assert!(
        result.status.success(),
        "{}",
        String::from_utf8_lossy(&result.stderr)
    );
    assert_eq!(fs::read(&fixture.n10_config).unwrap(), original);
    let log = fs::read_to_string(&hardlink).unwrap();
    let summary: Value = serde_json::from_str(log.lines().last().unwrap()).unwrap();
    assert_eq!(summary["kind"], "summary");
}
