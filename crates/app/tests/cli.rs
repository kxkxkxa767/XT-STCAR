use image::{Rgb, RgbImage};
use serde_json::{Value, json};
use std::fs;
#[cfg(unix)]
use std::path::Path;
use std::path::PathBuf;
use std::process::{Command, Output};
#[cfg(unix)]
use std::time::{Duration, Instant};
use tempfile::TempDir;

struct Fixture {
    temp: TempDir,
    config: PathBuf,
    image: PathBuf,
    tensor: PathBuf,
}

impl Fixture {
    fn new() -> Self {
        let temp = tempfile::Builder::new()
            .prefix("xt-stcar CLI ")
            .tempdir()
            .unwrap();
        let config = temp.path().join("spec.json");
        let image = temp.path().join("image $(printf unsafe) ; 'test'.png");
        let tensor = temp.path().join("detections.json");
        let mut spec: Value =
            serde_json::from_str(include_str!("../../../config/yolo26n.json")).unwrap();
        spec["input_width"] = json!(32);
        spec["input_height"] = json!(32);
        spec["max_detections"] = json!(2);
        fs::write(&config, serde_json::to_vec(&spec).unwrap()).unwrap();
        RgbImage::from_pixel(64, 48, Rgb([255, 0, 128]))
            .save(&image)
            .unwrap();
        fs::write(&tensor, Self::tensor_json()).unwrap();
        Self {
            temp,
            config,
            image,
            tensor,
        }
    }

    fn tensor_json() -> String {
        json!({"shape":[1,2,6],"values":[5,6.5,15,16.5,0.8,2,5,6.5,15,16.5,0.25,2]}).to_string()
    }

    fn command(&self, mode: &str) -> Command {
        let mut command = Command::new(env!("CARGO_BIN_EXE_xt-stcar"));
        command.arg(mode).arg("--config").arg(&self.config);
        command
    }

    fn replay(&self) -> Command {
        let mut command = self.command("replay");
        command
            .arg("--image")
            .arg(&self.image)
            .arg("--tensor")
            .arg(&self.tensor);
        command
    }

    #[cfg(unix)]
    fn infer(&self, script: &str) -> (Command, PathBuf) {
        let worker = self.temp.path().join("worker ; 'literal'.sh");
        let model = self.temp.path().join("model $(printf literal).onnx");
        fs::write(&worker, script).unwrap();
        fs::write(&model, b"mock model; no inference is executed by the test").unwrap();
        let mut command = self.command("infer");
        command
            .arg("--backend")
            .arg("python-reference")
            .arg("--image")
            .arg(&self.image)
            .arg("--python")
            .arg("/bin/sh")
            .arg("--worker")
            .arg(worker)
            .arg("--model")
            .arg(&model);
        (command, model)
    }
}

fn report(output: Output) -> Value {
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    serde_json::from_slice(&output.stdout).unwrap()
}

fn failure(output: Output, contains: &str) {
    assert!(!output.status.success());
    assert!(
        output.stdout.is_empty(),
        "failure must not emit a success report"
    );
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        stderr.contains(contains),
        "expected {contains:?}, got {stderr:?}"
    );
}

#[test]
fn self_check_explicitly_reports_synthetic_data() {
    let fixture = Fixture::new();
    let report = report(fixture.command("self-check").output().unwrap());
    assert_eq!(report["mode"], "self-check");
    assert_eq!(report["backend"], "synthetic");
    assert_eq!(report["inference_performed"], false);
    assert_eq!(
        report["detections"][0]["xyxy"],
        json!([0.0, 0.0, 640.0, 480.0])
    );
}

#[test]
fn preprocess_writes_little_endian_rgb_chw_and_plain_transform() {
    let fixture = Fixture::new();
    let tensor = fixture.temp.path().join("input.f32le");
    let transform = fixture.temp.path().join("transform.json");
    let output = fixture
        .command("preprocess")
        .arg("--image")
        .arg(&fixture.image)
        .arg("--tensor")
        .arg(&tensor)
        .arg("--transform")
        .arg(&transform)
        .output()
        .unwrap();
    let report = report(output);
    assert_eq!(report["mode"], "preprocess");
    assert_eq!(report["inference_performed"], false);
    let bytes = fs::read(tensor).unwrap();
    assert_eq!(bytes.len(), 3 * 32 * 32 * 4);
    let values: Vec<f32> = bytes
        .chunks_exact(4)
        .map(|v| f32::from_le_bytes(v.try_into().unwrap()))
        .collect();
    assert_eq!(values[0], 114.0 / 255.0);
    assert_eq!(values[4 * 32], 1.0);
    assert_eq!(values[1024 + 4 * 32], 0.0);
    assert_eq!(values[2048 + 4 * 32], 128.0 / 255.0);
    let transform: Value = serde_json::from_slice(&fs::read(transform).unwrap()).unwrap();
    assert_eq!(transform["pad_top"], 4);
    assert_eq!(transform["scale"], 0.5);
    assert_eq!(transform["source_width"], 64);
}

#[test]
fn replay_maps_coordinates_and_filters_equal_threshold() {
    let fixture = Fixture::new();
    let report = report(fixture.replay().output().unwrap());
    assert_eq!(report["mode"], "replay");
    assert_eq!(report["backend"], "saved-tensor");
    assert_eq!(report["inference_performed"], false);
    assert_eq!(report["detections"].as_array().unwrap().len(), 1);
    assert_eq!(
        report["detections"][0]["xyxy"],
        json!([10.0, 5.0, 30.0, 25.0])
    );
    let destination = fixture.temp.path().join("report.json");
    let output = fixture
        .replay()
        .arg("--output")
        .arg(&destination)
        .output()
        .unwrap();
    assert!(output.status.success());
    assert!(output.stdout.is_empty());
    assert_eq!(
        serde_json::from_slice::<Value>(&fs::read(destination).unwrap()).unwrap()["mode"],
        "replay"
    );
}

#[test]
fn malformed_cli_config_image_and_tensor_fail_explicitly() {
    let fixture = Fixture::new();
    failure(fixture.command("replay").output().unwrap(), "--image");
    failure(
        fixture
            .command("self-check")
            .arg("--unknown")
            .arg("1")
            .output()
            .unwrap(),
        "unknown",
    );
    failure(
        fixture
            .command("self-check")
            .arg("--config")
            .arg(&fixture.config)
            .output()
            .unwrap(),
        "duplicate",
    );
    fs::write(&fixture.tensor, r#"{"shape":[1,84,2100],"values":[]}"#).unwrap();
    failure(fixture.replay().output().unwrap(), "raw YOLO11/one-to-many");
    fs::write(&fixture.tensor, "not json").unwrap();
    failure(fixture.replay().output().unwrap(), "parse");
    fs::write(&fixture.image, "not an image").unwrap();
    failure(fixture.replay().output().unwrap(), "image");
    fs::write(&fixture.config, "{}").unwrap();
    failure(
        fixture.command("self-check").output().unwrap(),
        "parse config",
    );
}

#[test]
fn output_collisions_do_not_overwrite_inputs() {
    let fixture = Fixture::new();
    let image_before = fs::read(&fixture.image).unwrap();
    failure(
        fixture
            .replay()
            .arg("--output")
            .arg(&fixture.image)
            .output()
            .unwrap(),
        "distinct",
    );
    let transform = fixture.temp.path().join("same.json");
    failure(
        fixture
            .command("preprocess")
            .arg("--image")
            .arg(&fixture.image)
            .arg("--tensor")
            .arg(&transform)
            .arg("--transform")
            .arg(&transform)
            .output()
            .unwrap(),
        "distinct",
    );
    assert_eq!(image_before, fs::read(&fixture.image).unwrap());
    assert!(!transform.exists());
}

#[cfg(unix)]
#[test]
fn output_hard_links_are_replaced_without_modifying_the_input_inode() {
    let fixture = Fixture::new();
    let original = fs::read(&fixture.image).unwrap();
    let report_path = fixture.temp.path().join("image-hardlink-report.json");
    fs::hard_link(&fixture.image, &report_path).unwrap();
    let output = fixture
        .replay()
        .arg("--output")
        .arg(&report_path)
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    assert_eq!(fs::read(&fixture.image).unwrap(), original);
    assert_eq!(
        serde_json::from_slice::<Value>(&fs::read(report_path).unwrap()).unwrap()["mode"],
        "replay"
    );

    let tensor_path = fixture.temp.path().join("image-hardlink-tensor.f32le");
    let transform_path = fixture.temp.path().join("image-hardlink-transform.json");
    fs::hard_link(&fixture.image, &tensor_path).unwrap();
    fs::hard_link(&fixture.image, &transform_path).unwrap();
    let result = report(
        fixture
            .command("preprocess")
            .arg("--image")
            .arg(&fixture.image)
            .arg("--tensor")
            .arg(&tensor_path)
            .arg("--transform")
            .arg(&transform_path)
            .output()
            .unwrap(),
    );
    assert_eq!(result["mode"], "preprocess");
    assert_eq!(fs::read(&fixture.image).unwrap(), original);
    assert_eq!(fs::metadata(tensor_path).unwrap().len(), 3 * 32 * 32 * 4);
    assert_eq!(
        serde_json::from_slice::<Value>(&fs::read(transform_path).unwrap()).unwrap()["source_width"],
        64
    );
}

#[test]
fn native_is_the_default_and_python_requires_explicit_reference_selection() {
    let fixture = Fixture::new();
    failure(
        fixture
            .command("infer")
            .arg("--image")
            .arg(&fixture.image)
            .arg("--model")
            .arg(&fixture.tensor)
            .output()
            .unwrap(),
        "--runtime-lib",
    );
    failure(
        fixture
            .command("infer")
            .arg("--image")
            .arg(&fixture.image)
            .arg("--model")
            .arg(&fixture.tensor)
            .arg("--runtime-lib")
            .arg(fixture.temp.path().join("explicit-runtime.dylib"))
            .arg("--python")
            .arg("python3")
            .output()
            .unwrap(),
        "inapplicable option --python",
    );
}

#[cfg(unix)]
#[test]
fn reference_worker_receives_separate_literal_arguments_and_reports_diagnostics() {
    let fixture = Fixture::new();
    let script = format!(
        "[ \"$1\" = --model ] && [ -f \"$2\" ] && [ \"$3\" = --spec ] && [ -s \"$4\" ] && [ \"$5\" = --input ] && [ -s \"$6\" ] && [ \"$7\" = --output ] || exit 91\nprintf '%s' '{tensor}' > \"$8\"\necho 'worker stdout diagnostic'\necho 'worker stderr diagnostic' >&2\n",
        tensor = Fixture::tensor_json()
    );
    let (mut command, _) = fixture.infer(&script);
    let output = command.output().unwrap();
    assert!(String::from_utf8_lossy(&output.stderr).contains("worker stdout diagnostic"));
    let report = report(output);
    assert_eq!(report["mode"], "infer");
    assert_eq!(report["backend"], "python-onnxruntime-reference");
    assert_eq!(report["inference_performed"], true);
    assert_eq!(report["detections"].as_array().unwrap().len(), 1);
}

#[cfg(unix)]
#[test]
fn failed_worker_cannot_supply_a_successful_tensor() {
    let fixture = Fixture::new();
    let script = format!(
        "printf '%s' '{}' > \"$8\"\necho expected-failure >&2\nexit 7\n",
        Fixture::tensor_json()
    );
    let (mut command, _) = fixture.infer(&script);
    let destination = fixture.temp.path().join("must-not-exist.json");
    failure(
        command.arg("--output").arg(&destination).output().unwrap(),
        "expected-failure",
    );
    assert!(!destination.exists());
    let (mut command, _) = fixture.infer("exit 0\n");
    failure(command.output().unwrap(), "output.json");
    let (mut command, _) = fixture.infer("printf '%s' 'invalid json' > \"$8\"\n");
    failure(command.output().unwrap(), "parse");
}

#[cfg(unix)]
#[test]
fn timeout_kills_and_reaps_worker_and_cleans_temporary_files() {
    let fixture = Fixture::new();
    let script = "printf '%s\\n%s\\n' \"$$\" \"$4\" > \"$2\"\nexec /bin/sleep 5\n";
    let (mut command, model) = fixture.infer(script);
    let started = Instant::now();
    failure(
        command.arg("--timeout-secs").arg("0.15").output().unwrap(),
        "timed out",
    );
    assert!(started.elapsed() < Duration::from_secs(3));
    let record = fs::read_to_string(model).unwrap();
    let mut lines = record.lines();
    let pid = lines.next().unwrap();
    let temporary_spec = Path::new(lines.next().unwrap());
    let alive = Command::new("/bin/kill")
        .arg("-0")
        .arg(pid)
        .output()
        .unwrap();
    assert!(!alive.status.success(), "timed-out child must be reaped");
    assert!(
        !temporary_spec.exists(),
        "worker temporary directory must be removed"
    );
}

#[cfg(unix)]
#[test]
fn invalid_timeouts_and_missing_interpreter_fail() {
    let fixture = Fixture::new();
    for timeout in ["0", "-1", "NaN", "inf", "3601", "invalid"] {
        let (mut command, _) = fixture.infer("exit 0\n");
        failure(
            command.arg("--timeout-secs").arg(timeout).output().unwrap(),
            "timeout",
        );
    }
    let missing = fixture.temp.path().join("missing-python");
    let (command, _) = fixture.infer("exit 0\n");
    let mut args: Vec<_> = command.get_args().map(ToOwned::to_owned).collect();
    let index = args.iter().position(|a| a == "--python").unwrap();
    args[index + 1] = missing.into_os_string();
    failure(
        Command::new(env!("CARGO_BIN_EXE_xt-stcar"))
            .args(args)
            .output()
            .unwrap(),
        "start reference worker",
    );
}
