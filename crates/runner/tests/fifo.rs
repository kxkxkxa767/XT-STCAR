#![cfg(any(target_os = "linux", target_os = "macos"))]

use std::{
    fs,
    process::{Child, Command, Output, Stdio},
    thread,
    time::{Duration, Instant},
};

const PRIOR_LOG: &[u8] = b"previous complete log\n";

struct Running(Option<Child>);

impl Running {
    fn run(command: &mut Command) -> Output {
        let mut guard = Self(Some(
            command
                .stdout(Stdio::piped())
                .stderr(Stdio::piped())
                .spawn()
                .unwrap(),
        ));
        let deadline = Instant::now() + Duration::from_secs(5);
        loop {
            if guard.0.as_mut().unwrap().try_wait().unwrap().is_some() {
                return guard.0.take().unwrap().wait_with_output().unwrap();
            }
            assert!(Instant::now() < deadline, "static input blocked the CLI");
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

#[test]
fn fifo_event_and_configuration_inputs_fail_without_waiting_or_replacing_logs() {
    let temp = tempfile::tempdir().unwrap();
    let fifo = temp.path().join("unconnected.fifo");
    // rustix has no mkfifo wrapper on macOS; invoke the standard utility with
    // separate arguments, never a shell, and only inside this temporary folder.
    let created = Running::run(Command::new("mkfifo").args(["-m", "600"]).arg(&fifo));
    assert!(created.status.success());
    let config = temp.path().join("robot.json");
    let events = temp.path().join("events.jsonl");
    let output = temp.path().join("previous.jsonl");
    fs::write(&config, include_str!("../../../config/robot-sim.json")).unwrap();
    fs::write(&events, b"{\"at\":0,\"event\":{\"type\":\"tick\"}}\n").unwrap();

    // No process opens the FIFO's write end. A blocking File::open would hang.
    let mut commands = Vec::new();
    let mut replay_events = Command::new(env!("CARGO_BIN_EXE_xt-stcar-robot"));
    replay_events
        .args(["replay", "--events"])
        .arg(&fifo)
        .arg("--config")
        .arg(&config);
    commands.push(replay_events);
    let mut replay_config = Command::new(env!("CARGO_BIN_EXE_xt-stcar-robot"));
    replay_config
        .args(["replay", "--events"])
        .arg(&events)
        .arg("--config")
        .arg(&fifo);
    commands.push(replay_config);
    for execute in [false, true] {
        let mut capture = Command::new(env!("CARGO_BIN_EXE_xt-stcar-robot"));
        capture.args(["serial-capture", "--config"]).arg(&fifo);
        if execute {
            capture.arg("--execute");
        }
        commands.push(capture);
    }
    for mut command in commands {
        fs::write(&output, PRIOR_LOG).unwrap();
        command.arg("--output").arg(&output);
        let result = Running::run(&mut command);
        assert!(!result.status.success());
        assert!(result.stdout.is_empty());
        assert!(String::from_utf8_lossy(&result.stderr).contains("regular file"));
        assert_eq!(fs::read(&output).unwrap(), PRIOR_LOG);
    }
}

#[test]
fn oversized_static_inputs_are_rejected_before_json_parsing_and_keep_prior_log() {
    let temp = tempfile::tempdir().unwrap();
    let config = temp.path().join("robot.json");
    let events = temp.path().join("events.jsonl");
    let large = temp.path().join("too-large.json");
    let output = temp.path().join("previous.jsonl");
    fs::write(&config, include_str!("../../../config/robot-sim.json")).unwrap();
    fs::write(&events, b"{\"at\":0,\"event\":{\"type\":\"tick\"}}\n").unwrap();

    for is_config in [true, false] {
        let limit = if is_config {
            1024 * 1024
        } else {
            64 * 1024 * 1024
        };
        fs::File::create(&large)
            .unwrap()
            .set_len(limit + 1)
            .unwrap();
        fs::write(&output, PRIOR_LOG).unwrap();
        let mut command = Command::new(env!("CARGO_BIN_EXE_xt-stcar-robot"));
        command
            .args(["replay", "--events"])
            .arg(if is_config { &events } else { &large })
            .arg("--config")
            .arg(if is_config { &large } else { &config })
            .arg("--output")
            .arg(&output);
        let result = Running::run(&mut command);
        assert!(!result.status.success());
        assert!(result.stdout.is_empty());
        assert!(
            String::from_utf8_lossy(&result.stderr).contains(&format!("exceeds {limit} bytes"))
        );
        assert_eq!(fs::read(&output).unwrap(), PRIOR_LOG);
    }
}

#[test]
fn checked_static_reader_accepts_exact_limit_and_frame_decoder_reuses_regular_file() {
    use xt_stcar_robot_runner::input::{load_frame, read_regular_file};
    let temp = tempfile::tempdir().unwrap();
    let path = temp.path().join("input.bin");
    fs::write(&path, b"1234").unwrap();
    assert_eq!(read_regular_file(&path, 4).unwrap(), b"1234");
    assert!(
        read_regular_file(&path, 3)
            .unwrap_err()
            .contains("exceeds 3 bytes")
    );
    let image_path = temp.path().join("frame.png");
    let expected = image::RgbImage::from_pixel(3, 2, image::Rgb([13, 10, 255]));
    expected.save(&image_path).unwrap();
    assert_eq!(load_frame(&image_path).unwrap(), expected);
    assert!(
        load_frame(temp.path())
            .unwrap_err()
            .contains("regular file")
    );
}
