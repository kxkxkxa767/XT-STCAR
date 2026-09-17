use std::path::PathBuf;
use std::process::Command;
fn root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..")
}
fn command() -> Command {
    let mut c = Command::new(env!("CARGO_BIN_EXE_xt-stcar-robot"));
    c.current_dir(root())
        .arg("startup-replay")
        .args(["--config", "config/startup-assist-sim.json"]);
    c
}
#[test]
fn example_steps_then_hands_off_without_any_physical_output() {
    let out = command()
        .args(["--events", "examples/startup-assist-sim.jsonl"])
        .output()
        .unwrap();
    assert!(
        out.status.success(),
        "{}",
        String::from_utf8_lossy(&out.stderr)
    );
    let rows: Vec<serde_json::Value> = String::from_utf8(out.stdout)
        .unwrap()
        .lines()
        .map(|l| serde_json::from_str(l).unwrap())
        .collect();
    assert!(rows.iter().all(|r| r["physical_output_enabled"] == false));
    assert!(
        rows.iter()
            .any(|r| r["decision"]["action"]["command"]["motor_us"] == 1512)
    );
    assert!(
        rows.iter()
            .any(|r| r["decision"]["reason"] == "motion_confirmed_stop_increasing")
    );
    assert_eq!(rows.last().unwrap()["decision"]["action"]["type"], "stop");
}
#[test]
fn permit_failure_returns_nonzero_and_keeps_latched_stop_record() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("events.jsonl");
    let output = dir.path().join("out.jsonl");
    let text = std::fs::read_to_string(root().join("examples/startup-assist-sim.jsonl")).unwrap();
    let rows: Vec<String> = text
        .lines()
        .enumerate()
        .map(|(n, l)| {
            let mut v: serde_json::Value = serde_json::from_str(l).unwrap();
            if n == 6 {
                v["permit"] = serde_json::Value::Null;
            }
            v.to_string()
        })
        .collect();
    std::fs::write(&path, rows.join("\n")).unwrap();
    let out = command()
        .arg("--events")
        .arg(path)
        .arg("--output")
        .arg(&output)
        .output()
        .unwrap();
    assert!(!out.status.success());
    let rows: Vec<serde_json::Value> = std::fs::read_to_string(output)
        .unwrap()
        .lines()
        .map(|l| serde_json::from_str(l).unwrap())
        .collect();
    assert!(rows[6..].iter().all(|r| r["decision"]["phase"] == "fault"
        && r["decision"]["action"]["command"]["motor_us"] == 1500));
}
#[test]
fn malformed_or_unterminated_input_cannot_overwrite_output() {
    let dir = tempfile::tempdir().unwrap();
    let events = dir.path().join("input.jsonl");
    let output = dir.path().join("output.jsonl");
    std::fs::write(&output, "preserve").unwrap();
    for text in [
        "{}\n",
        include_str!("../../../examples/startup-assist-sim.jsonl")
            .lines()
            .next()
            .unwrap(),
    ] {
        std::fs::write(&events, text).unwrap();
        let out = command()
            .arg("--events")
            .arg(&events)
            .arg("--output")
            .arg(&output)
            .output()
            .unwrap();
        assert!(!out.status.success());
        assert_eq!(std::fs::read_to_string(&output).unwrap(), "preserve");
    }
    let original = include_str!("../../../examples/startup-assist-sim.jsonl");
    std::fs::write(&events, original).unwrap();
    let out = command()
        .arg("--events")
        .arg(&events)
        .arg("--output")
        .arg(&events)
        .output()
        .unwrap();
    assert!(!out.status.success());
    assert_eq!(std::fs::read_to_string(events).unwrap(), original);
}
