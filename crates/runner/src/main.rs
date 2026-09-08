use std::collections::BTreeMap;
use std::ffi::OsString;
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use xt_stcar_robot_runner::input::{MAX_CONFIG_BYTES, read_regular_file};
use xt_stcar_robot_runner::{ReplayOptions, replay, vision::VisionOptions};

const HELP: &str = "XT-STCAR Rust robot module runner (offline recording only)

Usage:
  xt-stcar-robot autonomy-example
  xt-stcar-robot autonomy-sim --config FILE [--output FILE] [--trace]
  xt-stcar-robot autonomy-replay --config FILE --events FILE [--output FILE] [--trace]
  xt-stcar-robot road-detect --config FILE --image FILE [--output FILE]
                    [--model ONNX --runtime-lib LIB --vision-config FILE]
  Closed-loop simulation uses synthetic RGB/range/pose feedback, never hardware.
  xt-stcar-robot serial-capture --config FILE --output FILE [--execute]
  Sensor capture defaults to a plan; --execute opens the explicitly selected tty.
  xt-stcar-robot chassis-preview --profile PROFILE --linear VALUE --angular VALUE
  PROFILE: navigation1300 | navigation_one1200 | teleop_pwm_degrees
  Reference PWM preview only; values retain factory topic conventions.
  xt-stcar-robot replay --events FILE [--config FILE] [--output FILE]
                      [--model ONNX --runtime-lib LIB --vision-config FILE]
                      [--imu-config FILE] [--n10-config FILE] [--chassis-calibration FILE]

Replay default: --config config/robot-sim.json. Vision default: config/yolo26n.json.
Autonomy commands require explicit --config; input timestamps are monotonic session milliseconds.
Replay input is strict event JSONL; autonomy-replay instead takes typed sensor snapshots.
Sensor/control events are validated by robot-core; vision_frame events load image
files relative to the event manifest and use a persistent Rust ONNX Runtime session.
--model and --runtime-lib must be supplied together for vision_frame replay or road-detect.
Model provenance is loaded from the model's sibling .provenance.json file.
Replay output is step + summary JSONL; without --output it is written to stdout.
Autonomy stdout defaults to summary only; --output saves sparse event/terminal logs.
--trace enables bounded debug detail; a full trace budget never stops control.
Motion output is RecordingSink only. Replay/preview use no physical devices.
Explicit serial-capture --execute reads a sensor tty; it sends no device commands.
Simulation timing and motion limits are examples, not vehicle calibration.
";

type Result<T> = std::result::Result<T, String>;

#[derive(Default)]
struct LogBuffer(Vec<u8>);

impl io::Write for LogBuffer {
    fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
        if self.0.len().saturating_add(bytes.len()) > 64 * 1024 * 1024 {
            return Err(io::Error::other(
                "replay log exceeds 64 MiB; split the event recording",
            ));
        }
        self.0.extend_from_slice(bytes);
        Ok(bytes.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

fn ends_with_error_stop(bytes: &[u8]) -> bool {
    if !bytes.ends_with(b"\n") {
        return false;
    }
    let Some(line) = bytes
        .split(|byte| *byte == b'\n')
        .rev()
        .find(|line| !line.is_empty())
    else {
        return false;
    };
    let Ok(record) = serde_json::from_slice::<serde_json::Value>(line) else {
        return false;
    };
    matches!(
        record["terminal"].as_str(),
        Some("vision_error" | "chassis_error")
    ) && record["step"]["output"]["command"]["type"] == "stop"
        && record["physical_output_enabled"] == false
}

fn distinct_output(output: &Path, inputs: &[&Path]) -> Result<()> {
    if output.exists() && !output.metadata().map_err(|e| e.to_string())?.is_file() {
        return Err("output must be a regular file".into());
    }
    let target = if output.exists() {
        output.canonicalize().map_err(|e| e.to_string())?
    } else {
        let parent = output
            .parent()
            .filter(|p| !p.as_os_str().is_empty())
            .unwrap_or_else(|| Path::new("."));
        parent
            .canonicalize()
            .map_err(|e| e.to_string())?
            .join(output.file_name().ok_or("output needs a filename")?)
    };
    for path in inputs {
        if path
            .canonicalize()
            .map_err(|e| format!("input {}: {e}", path.display()))?
            == target
        {
            return Err(
                "output must not overwrite a configuration, model, runtime, or input".into(),
            );
        }
    }
    Ok(())
}

fn chassis_preview(rest: Vec<OsString>) -> Result<()> {
    use xt_stcar_robot_core::protocol::chassis::FactoryProfile;
    let mut args = rest.into_iter();
    let mut values = BTreeMap::new();
    while let Some(key) = args.next() {
        let key = key.into_string().map_err(|_| "option must be UTF-8")?;
        if !["--profile", "--linear", "--angular"].contains(&key.as_str()) {
            return Err(format!("unknown preview option {key}"));
        }
        let value = args
            .next()
            .ok_or("missing preview value")?
            .into_string()
            .map_err(|_| "preview value must be UTF-8")?;
        if values.insert(key, value).is_some() {
            return Err("duplicate preview option".into());
        }
    }
    let profile = values.get("--profile").ok_or("--profile is required")?;
    let mapping = match profile.as_str() {
        "navigation1300" => FactoryProfile::Navigation1300,
        "navigation_one1200" => FactoryProfile::NavigationOne1200,
        "teleop_pwm_degrees" => FactoryProfile::TeleopPwmDegrees,
        _ => return Err("unknown factory profile".into()),
    };
    let number = |key| -> Result<f64> {
        values
            .get(key)
            .ok_or_else(|| format!("{key} is required"))?
            .parse()
            .map_err(|_| format!("{key} must be numeric"))
    };
    let packet = mapping
        .preview(number("--linear")?, number("--angular")?)
        .map_err(|e| e.to_string())?
        .encode();
    println!(
        "{}",
        serde_json::json!({"kind":"chassis_preview", "profile":profile,
        "physical_output_enabled":false, "vehicle_calibration_verified":false,
        "bytes":packet, "hex":packet.iter().map(|b| format!("{b:02X}")).collect::<Vec<_>>().join(" ")})
    );
    Ok(())
}

fn serial_capture(rest: Vec<OsString>) -> Result<()> {
    use xt_stcar_robot_runner::capture::{CaptureConfig, capture};
    let mut args = rest.into_iter();
    let mut options = BTreeMap::new();
    let mut execute = false;
    while let Some(key) = args.next() {
        let key = key.into_string().map_err(|_| "option must be UTF-8")?;
        if key == "--execute" {
            if execute {
                return Err("duplicate --execute".into());
            }
            execute = true;
            continue;
        }
        if !["--config", "--output"].contains(&key.as_str()) {
            return Err(format!("unknown capture option {key}"));
        }
        let value = args.next().ok_or("missing capture option value")?;
        if value.is_empty() || value.to_string_lossy().starts_with("--") {
            return Err("missing capture value".into());
        }
        if options.insert(key, PathBuf::from(value)).is_some() {
            return Err("duplicate capture option".into());
        }
    }
    let config_path = options.remove("--config").ok_or("--config is required")?;
    let path = options.remove("--output").ok_or("--output is required")?;
    let config: CaptureConfig =
        serde_json::from_slice(&read_regular_file(&config_path, MAX_CONFIG_BYTES)?)
            .map_err(|e| e.to_string())?;
    config.validate()?;
    distinct_output(&path, &[&config_path])?;
    if !execute {
        println!(
            "{}",
            serde_json::json!({"kind":"capture_plan","open_device":false,"config":config,
            "baud_rate":config.protocol.baud(),"format":"8N1 raw, no flow control","output":path,
            "device_commands_sent":false,"execute_required":true})
        );
        return Ok(());
    }
    distinct_output(&path, &[&config_path, &config.device])?;
    let mut buffer = LogBuffer::default();
    let summary = capture(&config, &mut buffer)?;
    let parent = path
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or_else(|| Path::new("."));
    let mut file = tempfile::NamedTempFile::new_in(parent).map_err(|e| e.to_string())?;
    file.write_all(&buffer.0).map_err(|e| e.to_string())?;
    file.flush().map_err(|e| e.to_string())?;
    file.persist(&path).map_err(|e| e.to_string())?;
    println!(
        "{}",
        serde_json::to_string(&summary).map_err(|e| e.to_string())?
    );
    Ok(())
}

fn run() -> Result<()> {
    let mut arguments = std::env::args_os().skip(1);
    let Some(command) = arguments.next() else {
        print!("{HELP}");
        return Ok(());
    };
    let rest: Vec<OsString> = arguments.collect();
    if command == "--help" || command == "-h" || (rest.len() == 1 && rest[0] == "--help") {
        print!("{HELP}");
        return Ok(());
    }
    if command == "serial-capture" {
        return serial_capture(rest);
    }
    if command == "autonomy-example" {
        if !rest.is_empty() {
            return Err("autonomy-example takes no arguments".into());
        }
        println!(
            "{}",
            serde_json::to_string_pretty(
                &xt_stcar_robot_runner::simulation::SimulationConfig::example()
            )
            .map_err(|e| e.to_string())?
        );
        return Ok(());
    }
    if command == "autonomy-sim" || command == "autonomy-replay" || command == "road-detect" {
        return autonomy_command(command.to_str().ok_or("command must be UTF-8")?, rest);
    }
    if command == "chassis-preview" {
        return chassis_preview(rest);
    }
    if command != "replay" {
        return Err("expected replay; see --help".into());
    }
    let mut args = rest.into_iter();
    let mut options = BTreeMap::new();
    while let Some(key) = args.next() {
        let key = key.into_string().map_err(|_| "option must be UTF-8")?;
        if ![
            "--events",
            "--config",
            "--output",
            "--model",
            "--runtime-lib",
            "--vision-config",
            "--imu-config",
            "--n10-config",
            "--chassis-calibration",
        ]
        .contains(&key.as_str())
        {
            return Err(format!("unknown option {key}"));
        }
        let value = args
            .next()
            .ok_or_else(|| format!("missing value for {key}"))?;
        if value.is_empty() || value.to_string_lossy().starts_with("--") {
            return Err(format!("missing value for {key}"));
        }
        if options.insert(key.clone(), PathBuf::from(value)).is_some() {
            return Err(format!("duplicate option {key}"));
        }
    }
    let events = options.remove("--events").ok_or("--events is required")?;
    let config = options
        .remove("--config")
        .unwrap_or_else(|| "config/robot-sim.json".into());
    let output = options.remove("--output");
    let imu_config = options.remove("--imu-config");
    let n10_config = options.remove("--n10-config");
    let chassis_calibration = options.remove("--chassis-calibration");
    let model = options.remove("--model");
    let runtime = options.remove("--runtime-lib");
    let spec = options.remove("--vision-config");
    let vision = match (model, runtime, spec) {
        (Some(model), Some(runtime_lib), spec) => Some(VisionOptions {
            model,
            runtime_lib,
            spec: spec.unwrap_or_else(|| "config/yolo26n.json".into()),
        }),
        (None, None, None) => None,
        _ => {
            return Err(
                "supply --model and --runtime-lib together; --vision-config requires both".into(),
            );
        }
    };
    if let Some(path) = &output {
        let mut sources = vec![config.as_path(), events.as_path()];
        if let Some(path) = &imu_config {
            sources.push(path.as_path());
        }
        for path in [&n10_config, &chassis_calibration].into_iter().flatten() {
            sources.push(path.as_path());
        }
        let provenance;
        if let Some(vision) = &vision {
            provenance = vision.model.with_extension("provenance.json");
            sources.extend([
                vision.model.as_path(),
                vision.runtime_lib.as_path(),
                vision.spec.as_path(),
                provenance.as_path(),
            ]);
        }
        // Protect referenced camera images as well as the event manifest itself.
        let records = xt_stcar_robot_runner::input::read_events(&events)?;
        let frames: Vec<PathBuf> = records
            .into_iter()
            .filter_map(|record| match record {
                xt_stcar_robot_runner::input::ReplayEvent::Frame(frame) => {
                    let xt_stcar_robot_runner::input::FrameEvent::VisionFrame { path, .. } =
                        frame.event;
                    Some(path)
                }
                _ => None,
            })
            .collect();
        sources.extend(frames.iter().map(PathBuf::as_path));
        distinct_output(path, &sources)?;
    }
    let options = ReplayOptions {
        config,
        events,
        vision,
        imu_config,
        n10_config,
        chassis_calibration,
    };
    // Buffer output until a valid replay finishes or records a terminal stop.
    // Invalid config/events or a missing runtime never truncate an existing log.
    let mut buffer = LogBuffer::default();
    let result = replay(options, &mut buffer);
    let bytes = buffer.0;
    // Failed/incomplete log writes must not replace a prior complete recording.
    if !bytes.is_empty() && (result.is_ok() || ends_with_error_stop(&bytes)) {
        match output {
            Some(path) => {
                let parent = path
                    .parent()
                    .filter(|p| !p.as_os_str().is_empty())
                    .unwrap_or_else(|| Path::new("."));
                let mut writer =
                    tempfile::NamedTempFile::new_in(parent).map_err(|e| e.to_string())?;
                writer.write_all(&bytes).map_err(|e| e.to_string())?;
                writer.flush().map_err(|e| e.to_string())?;
                // Atomic replacement severs output hard-link aliases without
                // ever truncating the input inode to which they pointed.
                writer
                    .persist(&path)
                    .map_err(|e| format!("commit output {}: {e}", path.display()))?;
            }
            None => {
                io::stdout()
                    .lock()
                    .write_all(&bytes)
                    .map_err(|e| e.to_string())?;
            }
        }
    }
    result.map(|_| ())
}

fn autonomy_command(command: &str, rest: Vec<OsString>) -> Result<()> {
    let mut args = rest.into_iter();
    let mut options = BTreeMap::new();
    let mut trace = false;
    while let Some(key) = args.next() {
        let key = key.into_string().map_err(|_| "option must be UTF-8")?;
        if key == "--trace" {
            if trace {
                return Err("duplicate --trace".into());
            }
            trace = true;
            continue;
        }
        if ![
            "--config",
            "--output",
            "--image",
            "--events",
            "--model",
            "--runtime-lib",
            "--vision-config",
        ]
        .contains(&key.as_str())
        {
            return Err(format!("unknown autonomy option {key}"));
        }
        let value = args.next().ok_or("missing autonomy option value")?;
        if value.is_empty() || value.to_string_lossy().starts_with("--") {
            return Err("missing autonomy option value".into());
        }
        if options.insert(key, PathBuf::from(value)).is_some() {
            return Err("duplicate autonomy option".into());
        }
    }
    let config_path = options.remove("--config").ok_or("--config is required")?;
    let output = options.remove("--output");
    let image = options.remove("--image");
    let events = options.remove("--events");
    let vision = match (
        options.remove("--model"),
        options.remove("--runtime-lib"),
        options.remove("--vision-config"),
    ) {
        (Some(model), Some(runtime_lib), spec) if command == "road-detect" => Some(VisionOptions {
            model,
            runtime_lib,
            spec: spec.unwrap_or_else(|| "config/yolo26n.json".into()),
        }),
        (None, None, None) => None,
        _ => return Err(
            "road-detect accepts --model and --runtime-lib together; --vision-config requires both"
                .into(),
        ),
    };
    let mut inputs = vec![config_path.as_path()];
    if let Some(image) = &image {
        inputs.push(image.as_path());
    }
    if let Some(events) = &events {
        inputs.push(events.as_path());
    }
    let provenance;
    if let Some(vision) = &vision {
        provenance = vision.model.with_extension("provenance.json");
        inputs.extend([
            vision.model.as_path(),
            vision.runtime_lib.as_path(),
            vision.spec.as_path(),
            provenance.as_path(),
        ]);
    }
    if let Some(path) = &output {
        distinct_output(path, &inputs)?;
    }
    let config = read_regular_file(&config_path, MAX_CONFIG_BYTES)?;
    let mut buffer = LogBuffer::default();
    let failure = if command == "autonomy-sim" {
        if image.is_some() || events.is_some() {
            return Err("autonomy-sim generates its own synthetic sensor data".into());
        }
        let mut config: xt_stcar_robot_runner::simulation::SimulationConfig =
            serde_json::from_slice(&config).map_err(|e| e.to_string())?;
        if trace {
            config.telemetry.mode = xt_stcar_robot_runner::telemetry::TelemetryMode::Trace;
        }
        let summary = xt_stcar_robot_runner::simulation::simulate(&config, &mut buffer)?;
        summary.fault
    } else if command == "autonomy-replay" {
        if image.is_some() {
            return Err("autonomy-replay takes typed sensor snapshots".into());
        }
        let events = events.ok_or("autonomy-replay requires --events")?;
        let config: xt_stcar_robot_runner::autonomy::AutonomyConfig =
            serde_json::from_slice(&config).map_err(|e| e.to_string())?;
        let snapshots = xt_stcar_robot_runner::autonomy_replay::read_snapshots(&events)?;
        let mut telemetry = xt_stcar_robot_runner::telemetry::TelemetryConfig::default();
        if trace {
            telemetry.mode = xt_stcar_robot_runner::telemetry::TelemetryMode::Trace;
        }
        xt_stcar_robot_runner::autonomy_replay::replay_snapshots(
            config,
            &snapshots,
            telemetry,
            &mut buffer,
        )?
        .fault
    } else {
        if trace || events.is_some() {
            return Err("road-detect does not accept --trace or --events".into());
        }
        let image_path = image.ok_or("road-detect requires --image")?;
        let config: xt_stcar_vision::road::RoadConfig =
            serde_json::from_slice(&config).map_err(|e| e.to_string())?;
        let mut detector =
            xt_stcar_robot_runner::perception::RoadPipeline::new(config, vision.as_ref())?;
        let rgb = xt_stcar_robot_runner::input::load_frame(&image_path)?;
        let road = detector.process(
            &rgb,
            xt_stcar_robot_core::Timestamp(0),
            xt_stcar_robot_core::FrameId("body".into()),
        )?;
        serde_json::to_writer_pretty(&mut buffer,&serde_json::json!({"kind":"road_image_diagnostic","physical_output_enabled":false,
            "timestamp_scope":"single image diagnostic only","native_yolo_enabled":vision.is_some(),"road":road})).map_err(|e|e.to_string())?;
        buffer.write_all(b"\n").map_err(|e| e.to_string())?;
        None
    };
    if let Some(path) = output {
        xt_stcar::backend::write_atomic(&path, |file| file.write_all(&buffer.0))?;
    } else {
        let bytes = if command != "road-detect" && !trace {
            buffer
                .0
                .split_inclusive(|b| *b == b'\n')
                .next_back()
                .unwrap_or(&buffer.0)
        } else {
            &buffer.0
        };
        io::stdout()
            .lock()
            .write_all(bytes)
            .map_err(|e| e.to_string())?;
    }
    if let Some(error) = failure {
        Err(format!(
            "autonomy stopped; terminal stop was recorded: {error}"
        ))
    } else {
        Ok(())
    }
}

fn main() -> std::process::ExitCode {
    match run() {
        Ok(()) => std::process::ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("xt-stcar-robot: {error}");
            std::process::ExitCode::from(2)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn an_incomplete_or_unstopped_log_is_not_publishable_on_error() {
        let stopped = b"{\"terminal\":\"vision_error\",\"step\":{\"output\":{\"command\":{\"type\":\"stop\"}}},\"physical_output_enabled\":false}\n";
        assert!(ends_with_error_stop(stopped));
        assert!(!ends_with_error_stop(&stopped[..stopped.len() - 1]));
        let mut partial = stopped.to_vec();
        partial.extend_from_slice(b"{\"kind\":");
        assert!(!ends_with_error_stop(&partial));
        assert!(!ends_with_error_stop(b"{\"kind\":\"step\"}\n"));
    }
}
