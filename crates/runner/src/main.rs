use std::collections::BTreeMap;
use std::ffi::OsString;
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use xt_stcar_robot_runner::{ReplayOptions, replay, vision::VisionOptions};

const HELP: &str = "XT-STCAR Rust robot module runner (offline recording only)

Usage:
  xt-stcar-robot chassis-preview --profile PROFILE --linear VALUE --angular VALUE
  PROFILE: navigation1300 | navigation_one1200 | teleop_pwm_degrees
  Reference PWM preview only; values retain factory topic conventions.
  xt-stcar-robot replay --events FILE [--config FILE] [--output FILE]
                      [--model ONNX --runtime-lib LIB --vision-config FILE]
                      [--imu-config FILE]

Defaults: --config config/robot-sim.json, --vision-config config/yolo26n.json.
Input is strict JSONL with monotonic boot-relative millisecond timestamps.
Sensor/control events are validated by robot-core; vision_frame events load image
files relative to the event manifest and use a persistent Rust ONNX Runtime session.
--model and --runtime-lib must be supplied together and only for vision_frame input.
Model provenance is loaded from the model's sibling .provenance.json file.
Output is step + summary JSONL; without --output it is written to stdout.
Motion output is RecordingSink only. No physical devices, ROS, or motors are used.
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
    record["terminal"] == "vision_error"
        && record["step"]["output"]["command"]["type"] == "stop"
        && record["physical_output_enabled"] == false
}

fn distinct_output(output: &Path, inputs: &[&Path]) -> Result<()> {
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
