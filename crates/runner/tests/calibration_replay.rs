use serde_json::{Value, json};
use std::fs;
use std::path::PathBuf;
use std::process::Command;
use tempfile::TempDir;

struct Fixture {
    temp: TempDir,
    config: PathBuf,
    events: PathBuf,
    calibration: PathBuf,
}

impl Fixture {
    fn new(curvature: f64) -> Self {
        let temp = tempfile::tempdir().unwrap();
        let config = temp.path().join("robot.json");
        let events = temp.path().join("events.jsonl");
        let calibration = temp.path().join("calibration.json");
        let mut spec: Value =
            serde_json::from_str(include_str!("../../../config/robot-sim.json")).unwrap();
        spec["required_sensors"] = json!([]);
        // A .6 command is within the controller's policy but outside the .5
        // calibration table, so that regression reaches the mapping boundary.
        spec["max_abs_curvature_per_m"] = json!(0.8);
        fs::write(&config, serde_json::to_vec(&spec).unwrap()).unwrap();
        fs::write(
            &calibration,
            include_str!("../../../config/chassis-calibration-sim.json"),
        )
        .unwrap();
        let input = [
            json!({"at":0,"event":{"type":"heartbeat"}}),
            json!({"at":0,"event":{"type":"deadman","pressed":true}}),
            json!({"at":0,"event":{"type":"arm"}}),
            json!({"at":10,"event":{"type":"motion","intent":{"speed_mps":0.1,"curvature_per_m":curvature}}}),
            json!({"at":10,"event":{"type":"start"}}),
        ]
        .into_iter()
        .map(|event| format!("{event}\n"))
        .collect::<String>();
        fs::write(&events, input).unwrap();
        Self {
            temp,
            config,
            events,
            calibration,
        }
    }

    fn command(&self) -> Command {
        let mut command = Command::new(env!("CARGO_BIN_EXE_xt-stcar-robot"));
        command
            .args(["replay", "--events"])
            .arg(&self.events)
            .arg("--config")
            .arg(&self.config)
            .arg("--chassis-calibration")
            .arg(&self.calibration);
        command
    }
}

fn records(bytes: &[u8]) -> Vec<Value> {
    assert!(
        bytes.ends_with(b"\n"),
        "recording must end in a complete line"
    );
    std::str::from_utf8(bytes)
        .unwrap()
        .lines()
        .map(|line| serde_json::from_str(line).unwrap())
        .collect()
}

#[test]
fn physical_units_produce_only_pwm_previews_and_eof_restores_neutral() {
    let fixture = Fixture::new(0.2);
    let result = fixture.command().output().unwrap();
    assert!(
        result.status.success(),
        "{}",
        String::from_utf8_lossy(&result.stderr)
    );
    let logs = records(&result.stdout);
    assert!(logs.iter().all(|r| r["physical_output_enabled"] == false));
    let steps: Vec<&Value> = logs.iter().filter(|r| r["kind"] == "step").collect();
    assert_eq!(steps.len(), 6);
    assert!(steps.iter().all(|r| {
        r["chassis_preview"]["simulation_only"] == true
            && r["chassis_preview"]["measurement_status"] == "unverified"
    }));
    let drives: Vec<&&Value> = steps
        .iter()
        .filter(|r| r["step"]["output"]["command"]["type"] == "drive")
        .collect();
    assert_eq!(drives.len(), 1);
    let preview = &drives[0]["chassis_preview"];
    assert_eq!(preview["command"]["motor_us"], 1530);
    assert_eq!(preview["command"]["servo_us"], 1540);
    assert_eq!(preview["bytes"], json!([0xaa, 0xfa, 5, 4, 6, 9, 0x55]));
    let terminal = steps.last().unwrap();
    assert_eq!(terminal["terminal"], "end_of_stream");
    assert_eq!(terminal["step"]["output"]["command"]["type"], "stop");
    assert_eq!(
        terminal["chassis_preview"]["command"],
        json!({"motor_us":1500,"servo_us":1500})
    );
    assert_eq!(
        terminal["chassis_preview"]["bytes"],
        json!([0xaa, 0xdc, 5, 0xdc, 5, 0xc2, 0x55])
    );
    assert_eq!(logs.last().unwrap()["drive_records"], 1);
    assert_eq!(logs.last().unwrap()["final_state"], "disarmed");
}

#[test]
fn mapping_failure_commits_a_complete_latched_stop_without_recording_drive() {
    let fixture = Fixture::new(0.6);
    let output = fixture.temp.path().join("output.jsonl");
    fs::write(&output, "previous recording\n").unwrap();
    let result = fixture
        .command()
        .arg("--output")
        .arg(&output)
        .output()
        .unwrap();
    assert!(!result.status.success());
    assert!(result.stdout.is_empty());
    assert!(String::from_utf8_lossy(&result.stderr).contains("chassis mapping failed"));
    let logs = records(&fs::read(&output).unwrap());
    assert_eq!(logs.len(), 5);
    assert!(logs.iter().all(|r| {
        r["kind"] == "step"
            && r["physical_output_enabled"] == false
            && r["step"]["output"]["command"]["type"] == "stop"
    }));
    let terminal = logs.last().unwrap();
    assert_eq!(terminal["terminal"], "chassis_error");
    assert_eq!(terminal["step"]["state"], "fault");
    assert_eq!(terminal["step"]["emergency_stop_latched"], true);
    assert_eq!(
        terminal["step"]["output"]["reason"]["type"],
        "emergency_stop"
    );
    assert_eq!(terminal["rejected_output"]["command"]["type"], "drive");
    assert_eq!(
        terminal["rejected_output"]["command"]["curvature_per_m"],
        0.6
    );
    assert!(terminal["error"].as_str().unwrap().contains("outside"));
    assert_eq!(
        terminal["chassis_preview"]["bytes"],
        json!([0xaa, 0xdc, 5, 0xdc, 5, 0xc2, 0x55])
    );
}

#[test]
fn invalid_calibration_preserves_existing_output_and_input_files() {
    let mut live_claim: Value =
        serde_json::from_str(include_str!("../../../config/chassis-calibration-sim.json")).unwrap();
    live_claim["simulation_only"] = false.into();
    for invalid in [
        "{}".to_owned(),
        "not-json".to_owned(),
        live_claim.to_string(),
    ] {
        let fixture = Fixture::new(0.2);
        fs::write(&fixture.calibration, &invalid).unwrap();
        let original_events = fs::read(&fixture.events).unwrap();
        let output = fixture.temp.path().join("previous.jsonl");
        fs::write(&output, "previous complete recording\n").unwrap();
        let result = fixture
            .command()
            .arg("--output")
            .arg(&output)
            .output()
            .unwrap();
        assert!(!result.status.success());
        assert!(result.stdout.is_empty());
        assert_eq!(
            fs::read_to_string(&output).unwrap(),
            "previous complete recording\n"
        );
        assert_eq!(fs::read_to_string(&fixture.calibration).unwrap(), invalid);
        assert_eq!(fs::read(&fixture.events).unwrap(), original_events);
    }
}

#[test]
fn output_cannot_overwrite_its_calibration() {
    let fixture = Fixture::new(0.2);
    let original = fs::read(&fixture.calibration).unwrap();
    let result = fixture
        .command()
        .arg("--output")
        .arg(&fixture.calibration)
        .output()
        .unwrap();
    assert!(!result.status.success());
    assert!(result.stdout.is_empty());
    assert!(String::from_utf8_lossy(&result.stderr).contains("must not overwrite"));
    assert_eq!(fs::read(&fixture.calibration).unwrap(), original);
}

#[cfg(unix)]
#[test]
fn output_symlink_cannot_overwrite_its_calibration() {
    let fixture = Fixture::new(0.2);
    let alias = fixture.temp.path().join("calibration-alias.json");
    std::os::unix::fs::symlink(&fixture.calibration, &alias).unwrap();
    let original = fs::read(&fixture.calibration).unwrap();
    let result = fixture
        .command()
        .arg("--output")
        .arg(&alias)
        .output()
        .unwrap();
    assert!(!result.status.success());
    assert!(result.stdout.is_empty());
    assert!(String::from_utf8_lossy(&result.stderr).contains("must not overwrite"));
    assert_eq!(fs::read(&fixture.calibration).unwrap(), original);
    assert!(fs::symlink_metadata(&alias).unwrap().is_symlink());
}
