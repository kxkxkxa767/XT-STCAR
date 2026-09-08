#![cfg(any(target_os = "linux", target_os = "macos"))]

use image::{Rgb, RgbImage};
use std::{
    fs,
    os::unix::fs::{FileTypeExt, PermissionsExt},
    path::{Path, PathBuf},
    process::{Child, Command, Output, Stdio},
    thread,
    time::{Duration, Instant},
};
use tempfile::TempDir;

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

struct Fixture {
    temp: TempDir,
    config: PathBuf,
    image: PathBuf,
    model: PathBuf,
    provenance: PathBuf,
    runtime: PathBuf,
    output: PathBuf,
}
impl Fixture {
    fn new() -> Self {
        let temp = tempfile::tempdir().unwrap();
        let fixture = Self {
            config: temp.path().join("spec.json"),
            image: temp.path().join("image.png"),
            model: temp.path().join("mock.onnx"),
            provenance: temp.path().join("mock.provenance.json"),
            runtime: temp.path().join("not-loaded.dylib"),
            output: temp.path().join("report.json"),
            temp,
        };
        fs::write(
            &fixture.config,
            include_str!("../../../config/yolo26n.json"),
        )
        .unwrap();
        RgbImage::from_pixel(2, 2, Rgb([13, 10, 255]))
            .save(&fixture.image)
            .unwrap();
        fs::write(
            &fixture.model,
            b"mock model; no runtime is used by these tests",
        )
        .unwrap();
        fs::write(&fixture.provenance, b"{}").unwrap();
        fs::write(
            &fixture.runtime,
            b"not a runtime; input validation must run first",
        )
        .unwrap();
        fs::write(&fixture.output, b"prior complete report\n").unwrap();
        fixture
    }
    fn command(&self, mode: &str) -> Command {
        let mut command = Command::new(env!("CARGO_BIN_EXE_xt-stcar"));
        command.arg(mode).arg("--config").arg(&self.config);
        command
    }
    fn reference(&self, worker: &Path) -> Command {
        let mut command = self.command("infer");
        command
            .args(["--backend", "python-reference"])
            .arg("--image")
            .arg(&self.image)
            .arg("--model")
            .arg(&self.model)
            .arg("--worker")
            .arg(worker);
        command
    }
    fn assert_preserved_failure(&self, command: &mut Command, message: &str) {
        let output = Running::run(command.arg("--output").arg(&self.output));
        assert!(!output.status.success());
        assert!(output.stdout.is_empty());
        assert!(
            String::from_utf8_lossy(&output.stderr).contains(message),
            "{}",
            String::from_utf8_lossy(&output.stderr)
        );
        assert_eq!(fs::read(&self.output).unwrap(), b"prior complete report\n");
    }
}

fn fifo(path: &Path) {
    // rustix has no mkfifo wrapper on macOS. Use separate utility arguments and
    // create only the test-owned temporary path, with no peer on the other end.
    assert!(
        Running::run(Command::new("mkfifo").args(["-m", "600"]).arg(path))
            .status
            .success()
    );
}

#[test]
fn all_static_cli_input_kinds_reject_fifos_before_reading_or_loading_a_runtime() {
    let fixture = Fixture::new();
    let pipe = fixture.temp.path().join("no-peer.fifo");
    fifo(&pipe);
    let mut config = Command::new(env!("CARGO_BIN_EXE_xt-stcar"));
    config.args(["self-check", "--config"]).arg(&pipe);
    fixture.assert_preserved_failure(&mut config, "regular file");
    let tensor = fixture.temp.path().join("tensor.json");
    fs::write(&tensor, b"{}").unwrap();
    fixture.assert_preserved_failure(
        fixture
            .command("replay")
            .arg("--image")
            .arg(&pipe)
            .arg("--tensor")
            .arg(&tensor),
        "regular file",
    );
    fixture.assert_preserved_failure(
        fixture
            .command("replay")
            .arg("--image")
            .arg(&fixture.image)
            .arg("--tensor")
            .arg(&pipe),
        "regular file",
    );
    for model_fifo in [true, false] {
        let mut command = fixture.command("infer");
        command
            .arg("--image")
            .arg(&fixture.image)
            .arg("--model")
            .arg(if model_fifo { &pipe } else { &fixture.model })
            .arg("--provenance")
            .arg(if model_fifo {
                &fixture.provenance
            } else {
                &pipe
            })
            .arg("--runtime-lib")
            .arg(&fixture.runtime)
            .args(["--timeout-secs", "0.01"]);
        fixture.assert_preserved_failure(&mut command, "regular file");
    }
}

#[test]
fn output_fifos_are_preserved_and_invalid_transform_does_not_replace_tensor() {
    let fixture = Fixture::new();
    let pipe = fixture.temp.path().join("output.fifo");
    fifo(&pipe);
    let output = Running::run(fixture.command("self-check").arg("--output").arg(&pipe));
    assert!(!output.status.success());
    assert!(fs::metadata(&pipe).unwrap().file_type().is_fifo());
    let tensor = fixture.temp.path().join("input.f32le");
    fs::write(&tensor, b"prior tensor").unwrap();
    let output = Running::run(
        fixture
            .command("preprocess")
            .arg("--image")
            .arg(&fixture.image)
            .arg("--tensor")
            .arg(&tensor)
            .arg("--transform")
            .arg(&pipe),
    );
    assert!(!output.status.success());
    assert_eq!(fs::read(&tensor).unwrap(), b"prior tensor");
    assert!(fs::metadata(&pipe).unwrap().file_type().is_fifo());
}

#[test]
fn explicit_and_path_interpreters_are_protected_without_breaking_default_resolution() {
    let fixture = Fixture::new();
    let interpreter = fixture.temp.path().join("python3");
    let wrapper = b"#!/bin/sh\nprintf '%s\\n' \"$0\" >&2\nexec /bin/sh \"$@\"\n";
    fs::write(&interpreter, wrapper).unwrap();
    fs::set_permissions(&interpreter, fs::Permissions::from_mode(0o700)).unwrap();
    let worker = fixture.temp.path().join("worker.sh");
    let tensor = serde_json::json!({"shape":[1,300,6],"values":vec![0;1800]}).to_string();
    fs::write(&worker, format!("printf '%s' '{tensor}' > \"$8\"\n")).unwrap();
    let inherited = std::env::var_os("PATH").unwrap_or_default();
    let search = std::env::join_paths(
        std::iter::once(fixture.temp.path().to_path_buf()).chain(std::env::split_paths(&inherited)),
    )
    .unwrap();
    for explicit in [true, false] {
        let mut command = fixture.reference(&worker);
        command
            .env("PATH", &search)
            .arg("--output")
            .arg(&interpreter);
        if explicit {
            command.arg("--python").arg(&interpreter);
        }
        let result = Running::run(&mut command);
        assert!(!result.status.success());
        assert!(String::from_utf8_lossy(&result.stderr).contains("distinct"));
        assert_eq!(fs::read(&interpreter).unwrap(), wrapper);
    }
    let output = Running::run(fixture.reference(&worker).env("PATH", &search));
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let environment = fixture.temp.path().join("venv/bin");
    fs::create_dir_all(&environment).unwrap();
    let linked_interpreter = environment.join("python3");
    std::os::unix::fs::symlink(&interpreter, &linked_interpreter).unwrap();
    let output = Running::run(
        fixture
            .reference(&worker)
            .arg("--python")
            .arg(&linked_interpreter),
    );
    assert!(output.status.success());
    assert!(
        String::from_utf8_lossy(&output.stderr).contains(linked_interpreter.to_str().unwrap()),
        "the selected virtualenv symlink must be executed without replacing it by the base interpreter"
    );
}

#[test]
fn unset_path_requires_explicit_interpreter_but_empty_path_keeps_cwd_lookup() {
    let fixture = Fixture::new();
    let interpreter = fixture.temp.path().join("python3");
    let marker = fixture.temp.path().join("python3.called");
    fs::write(
        &interpreter,
        b"#!/bin/sh\nprintf called > \"$0.called\"\nexec /bin/sh \"$@\"\n",
    )
    .unwrap();
    fs::set_permissions(&interpreter, fs::Permissions::from_mode(0o700)).unwrap();
    let worker = fixture.temp.path().join("worker.sh");
    let tensor = serde_json::json!({"shape":[1,300,6],"values":vec![0;1800]}).to_string();
    fs::write(&worker, format!("printf '%s' '{tensor}' > \"$8\"\n")).unwrap();

    fixture.assert_preserved_failure(
        fixture
            .reference(&worker)
            .current_dir(fixture.temp.path())
            .env_remove("PATH"),
        "PATH is unset; supply --python",
    );
    assert!(
        !marker.exists(),
        "unset PATH must not search the current directory"
    );

    let output = Running::run(
        fixture
            .reference(&worker)
            .current_dir(fixture.temp.path())
            .env("PATH", ""),
    );
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(
        marker.exists(),
        "an explicitly empty PATH retains current-directory lookup"
    );
    fs::remove_file(&marker).unwrap();

    let output = Running::run(
        fixture
            .reference(&worker)
            .env_remove("PATH")
            .arg("--python")
            .arg(&interpreter),
    );
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(
        marker.exists(),
        "an explicit interpreter works independently of PATH"
    );
}

#[test]
fn configured_static_byte_limits_reject_oversized_files_and_accept_exact_small_limit() {
    let fixture = Fixture::new();
    let large = fixture.temp.path().join("large.json");
    for mode in ["config", "tensor", "model", "provenance"] {
        let limit = if mode == "model" {
            64 * 1024 * 1024
        } else {
            1024 * 1024
        };
        fs::File::create(&large)
            .unwrap()
            .set_len(limit + 1)
            .unwrap();
        let mut command = match mode {
            "config" => {
                let mut command = Command::new(env!("CARGO_BIN_EXE_xt-stcar"));
                command.args(["self-check", "--config"]).arg(&large);
                command
            }
            "tensor" => {
                let mut command = fixture.command("replay");
                command
                    .arg("--image")
                    .arg(&fixture.image)
                    .arg("--tensor")
                    .arg(&large);
                command
            }
            _ => {
                let mut command = fixture.command("infer");
                command
                    .arg("--image")
                    .arg(&fixture.image)
                    .arg("--model")
                    .arg(if mode == "model" {
                        &large
                    } else {
                        &fixture.model
                    })
                    .arg("--provenance")
                    .arg(if mode == "provenance" {
                        &large
                    } else {
                        &fixture.provenance
                    })
                    .arg("--runtime-lib")
                    .arg(&fixture.runtime);
                command
            }
        };
        fixture.assert_preserved_failure(&mut command, &format!("exceeds {limit} bytes"));
    }
    fs::write(&large, b"1234").unwrap();
    assert_eq!(
        xt_stcar::file_io::read_regular_file(&large, 4).unwrap(),
        b"1234"
    );
    assert!(xt_stcar::file_io::read_regular_file(&large, 3).is_err());
}

#[test]
fn successful_worker_cannot_hang_output_reading_with_a_fifo_and_temp_files_are_cleaned() {
    let fixture = Fixture::new();
    let worker = fixture.temp.path().join("fifo-worker.sh");
    fs::write(&worker, "printf '%s' \"$8\" > \"$2\"\nmkfifo \"$8\"\n").unwrap();
    fixture.assert_preserved_failure(
        fixture
            .reference(&worker)
            .args(["--python", "/bin/sh", "--timeout-secs", "0.5"]),
        "regular file",
    );
    let worker_output = fs::read_to_string(&fixture.model).unwrap();
    assert!(
        !Path::new(&worker_output).exists(),
        "failed output must release the worker temporary directory"
    );
}
