#![cfg(any(target_os = "linux", target_os = "macos"))]

use rustix::{
    fd::OwnedFd,
    fs::{Mode, OFlags, fcntl_getfl, fcntl_setfl, open},
    io::{Errno, FdFlags, fcntl_setfd, read, write},
    pty::{OpenptFlags, grantpt, openpt, ptsname, unlockpt},
    termios::{self, LocalModes, OptionalActions},
};
use serde_json::{Value, json};
use std::{
    fs,
    path::{Path, PathBuf},
    process::{Child, Command, Output, Stdio},
    thread,
    time::{Duration, Instant},
};
use tempfile::TempDir;

const PRIOR_LOG: &[u8] = b"prior capture\n";

struct Fixture {
    temp: TempDir,
    config: PathBuf,
    output: PathBuf,
}

impl Fixture {
    fn new(device: &Path) -> Self {
        let temp = tempfile::tempdir().unwrap();
        let fixture = Self {
            config: temp.path().join("capture.json"),
            output: temp.path().join("capture.jsonl"),
            temp,
        };
        fixture.set_config(&json!({
            "device":device,
            "protocol":"imu_wit11",
            "duration_ms":1000,
            "read_timeout_ms":10,
            "max_bytes":4096,
        }));
        fixture
    }

    fn set_config(&self, value: &Value) {
        fs::write(&self.config, serde_json::to_vec(value).unwrap()).unwrap();
    }

    fn command(&self) -> Command {
        let mut command = Command::new(env!("CARGO_BIN_EXE_xt-stcar-robot"));
        command
            .args(["serial-capture", "--config"])
            .arg(&self.config)
            .arg("--output")
            .arg(&self.output);
        command
    }
}

// Every spawned CLI has a deadline, and a failed assertion still reaps it.
struct Running(Option<Child>);

impl Running {
    fn spawn(command: &mut Command) -> Self {
        Self(Some(
            command
                .stdout(Stdio::piped())
                .stderr(Stdio::piped())
                .spawn()
                .unwrap(),
        ))
    }

    fn wait(mut self) -> Output {
        let deadline = Instant::now() + Duration::from_secs(5);
        loop {
            if self.0.as_mut().unwrap().try_wait().unwrap().is_some() {
                return self.0.take().unwrap().wait_with_output().unwrap();
            }
            assert!(Instant::now() < deadline, "capture CLI exceeded 5 seconds");
            thread::sleep(Duration::from_millis(5));
        }
    }
}

impl Drop for Running {
    fn drop(&mut self) {
        if let Some(child) = self.0.as_mut() {
            let _ = child.kill();
            let _ = child.wait();
        }
    }
}

struct Pty {
    master: OwnedFd,
    monitor: OwnedFd,
    path: PathBuf,
}

impl Pty {
    fn new() -> Self {
        let master = openpt(OpenptFlags::RDWR | OpenptFlags::NOCTTY).unwrap();
        // OpenptFlags::CLOEXEC is unavailable on macOS. Avoid leaking the master
        // into the child, which would hide a simulated disconnect from it.
        fcntl_setfd(&master, FdFlags::CLOEXEC).unwrap();
        fcntl_setfl(&master, fcntl_getfl(&master).unwrap() | OFlags::NONBLOCK).unwrap();
        grantpt(&master).unwrap();
        unlockpt(&master).unwrap();
        let path = PathBuf::from(ptsname(&master, Vec::new()).unwrap().to_str().unwrap());
        let monitor = open(
            &path,
            OFlags::RDWR | OFlags::NOCTTY | OFlags::NONBLOCK | OFlags::CLOEXEC,
            Mode::empty(),
        )
        .unwrap();
        let mut initial = termios::tcgetattr(&monitor).unwrap();
        initial
            .local_modes
            .insert(LocalModes::ECHO | LocalModes::ICANON);
        initial.set_speed(38400).unwrap();
        termios::tcsetattr(&monitor, OptionalActions::Now, &initial).unwrap();
        Self {
            master,
            monitor,
            path,
        }
    }

    fn wait_until_capture_is_ready(&self, child: &mut Running) {
        let deadline = Instant::now() + Duration::from_secs(2);
        loop {
            assert!(
                child.0.as_mut().unwrap().try_wait().unwrap().is_none(),
                "capture exited before configuring its PTY"
            );
            let actual = termios::tcgetattr(&self.monitor).unwrap();
            if actual.input_speed() == 115200
                && !actual
                    .local_modes
                    .intersects(LocalModes::ECHO | LocalModes::ICANON)
            {
                return;
            }
            assert!(
                Instant::now() < deadline,
                "capture never configured its PTY"
            );
            thread::sleep(Duration::from_millis(2));
        }
    }
}

fn assert_success(output: &Output) {
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
}

fn assert_failed_preserving_log(fixture: &Fixture, command: &mut Command) {
    fs::write(&fixture.output, PRIOR_LOG).unwrap();
    let output = Running::spawn(command).wait();
    assert!(!output.status.success());
    assert!(output.stdout.is_empty());
    assert_eq!(fs::read(&fixture.output).unwrap(), PRIOR_LOG);
}

#[test]
fn default_plan_never_opens_missing_device_or_changes_output() {
    let missing = PathBuf::from(format!("/dev/xt-stcar-test-missing-{}", std::process::id()));
    assert!(!missing.exists());
    let fixture = Fixture::new(&missing);
    let original_config = fs::read(&fixture.config).unwrap();
    let result = Running::spawn(&mut fixture.command()).wait();
    assert_success(&result);
    let plan: Value = serde_json::from_slice(&result.stdout).unwrap();
    assert_eq!(plan["kind"], "capture_plan");
    assert_eq!(plan["open_device"], false);
    assert_eq!(plan["execute_required"], true);
    assert_eq!(plan["device_commands_sent"], false);
    assert!(!fixture.output.exists());

    fs::write(&fixture.output, PRIOR_LOG).unwrap();
    assert_success(&Running::spawn(&mut fixture.command()).wait());
    assert_eq!(fs::read(&fixture.output).unwrap(), PRIOR_LOG);
    assert_eq!(fs::read(&fixture.config).unwrap(), original_config);
    assert_eq!(fs::read_dir(fixture.temp.path()).unwrap().count(), 2);
}

#[test]
fn execute_captures_binary_and_idle_ticks_on_one_monotonic_clock_without_transmitting() {
    let pty = Pty::new();
    let fixture = Fixture::new(&pty.path);
    let wall_start = Instant::now();
    let mut child = Running::spawn(fixture.command().arg("--execute"));
    pty.wait_until_capture_is_ready(&mut child);
    // Leave an idle interval before and after the raw binary chunks. Waiting for
    // actual tty configuration avoids racing the default echo/canonical modes.
    thread::sleep(Duration::from_millis(40));
    let first = [0, 13, 10, 17, 19, 85, 255];
    let second = [85, 81, 1, 2, 3, 4];
    assert_eq!(write(&pty.master, &first).unwrap(), first.len());
    thread::sleep(Duration::from_millis(40));
    assert_eq!(write(&pty.master, &second).unwrap(), second.len());
    let result = child.wait();
    assert_success(&result);
    let summary: Value = serde_json::from_slice(&result.stdout).unwrap();
    let records: Vec<Value> = fs::read_to_string(&fixture.output)
        .unwrap()
        .lines()
        .map(|line| serde_json::from_str(line).unwrap())
        .collect();
    let times: Vec<u64> = records.iter().map(|r| r["at"].as_u64().unwrap()).collect();
    assert!(times.windows(2).all(|pair| pair[0] <= pair[1]));
    assert_eq!(times.last().copied(), summary["elapsed_ms"].as_u64());
    assert!(summary["elapsed_ms"].as_u64().unwrap() >= 1000);
    assert!(u128::from(*times.last().unwrap()) <= wall_start.elapsed().as_millis());
    assert!(
        records
            .iter()
            .all(|r| { r["event"]["type"] == "imu_bytes" || r["event"]["type"] == "tick" })
    );
    let first_raw = records
        .iter()
        .position(|r| r["event"]["type"] == "imu_bytes")
        .unwrap();
    let last_raw = records
        .iter()
        .rposition(|r| r["event"]["type"] == "imu_bytes")
        .unwrap();
    assert!(
        records[..first_raw]
            .iter()
            .any(|r| r["event"]["type"] == "tick")
    );
    // Exclude the unconditional terminal tick: an idle read itself must tick.
    assert!(
        records[last_raw + 1..records.len() - 1]
            .iter()
            .any(|r| r["event"]["type"] == "tick")
    );
    assert_eq!(records.last().unwrap()["event"]["type"], "tick");
    let captured: Vec<u8> = records
        .iter()
        .filter(|r| r["event"]["type"] == "imu_bytes")
        .flat_map(|r| r["event"]["bytes"].as_array().unwrap())
        .map(|byte| u8::try_from(byte.as_u64().unwrap()).unwrap())
        .collect();
    assert_eq!(captured, [first.as_slice(), second.as_slice()].concat());
    assert_eq!(summary["bytes"], captured.len());
    assert_eq!(summary["records"], records.len());
    assert_eq!(summary["device_commands_sent"], false);
    let mut outbound = [0; 64];
    assert_eq!(read(&pty.master, &mut outbound), Err(Errno::AGAIN));
}

#[test]
fn invalid_configuration_preserves_existing_capture() {
    let fixture = Fixture::new(Path::new("/dev/xt-stcar-test-invalid-config"));
    let original: Value = serde_json::from_slice(&fs::read(&fixture.config).unwrap()).unwrap();
    for (field, value) in [
        ("duration_ms", json!(0)),
        ("read_timeout_ms", json!(0)),
        ("max_bytes", json!(0)),
        ("protocol", json!("unknown")),
        ("unrecognized_setting", json!(true)),
    ] {
        let mut config = original.clone();
        config[field] = value;
        fixture.set_config(&config);
        assert_failed_preserving_log(&fixture, &mut fixture.command());
        assert_failed_preserving_log(&fixture, fixture.command().arg("--execute"));
    }
}

#[test]
fn invalid_paths_and_config_output_collision_do_not_overwrite_files() {
    let fixture = Fixture::new(Path::new("/dev/xt-stcar-test-invalid-path"));
    let ordinary_file = fixture.temp.path().join("not-a-tty");
    fs::write(&ordinary_file, b"ordinary input").unwrap();
    let mut config: Value = serde_json::from_slice(&fs::read(&fixture.config).unwrap()).unwrap();
    config["device"] = json!(ordinary_file);
    fixture.set_config(&config);
    assert_failed_preserving_log(&fixture, fixture.command().arg("--execute"));
    assert_eq!(fs::read(ordinary_file).unwrap(), b"ordinary input");

    config["device"] = json!("/dev/xt-stcar-test-invalid-path");
    fixture.set_config(&config);
    let before = fs::read(&fixture.config).unwrap();
    let mut command = Command::new(env!("CARGO_BIN_EXE_xt-stcar-robot"));
    command
        .args(["serial-capture", "--config"])
        .arg(&fixture.config)
        .arg("--output")
        .arg(&fixture.config);
    let result = Running::spawn(&mut command).wait();
    assert!(!result.status.success());
    assert_eq!(fs::read(&fixture.config).unwrap(), before);
    assert_eq!(fs::read(&fixture.output).unwrap(), PRIOR_LOG);
}

#[test]
fn disconnect_after_capture_started_keeps_previous_log() {
    let pty = Pty::new();
    let fixture = Fixture::new(&pty.path);
    fs::write(&fixture.output, PRIOR_LOG).unwrap();
    let mut child = Running::spawn(fixture.command().arg("--execute"));
    pty.wait_until_capture_is_ready(&mut child);
    assert_eq!(write(&pty.master, &[85, 81, 0]).unwrap(), 3);
    thread::sleep(Duration::from_millis(20));
    drop(pty.master);
    let result = child.wait();
    assert!(!result.status.success());
    assert!(result.stdout.is_empty());
    assert_eq!(fs::read(&fixture.output).unwrap(), PRIOR_LOG);
}
