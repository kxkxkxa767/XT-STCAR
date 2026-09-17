//! Offline exercise of the bounded startup state machine. Never opens a tty.
use crate::input::{MAX_CONFIG_BYTES, read_regular_file};
use serde::Deserialize;
use std::io::Write;
use std::path::Path;
use xt_stcar_robot_core::MotionOutput;
use xt_stcar_robot_core::startup_assist::{StartupAssist, StartupConfig, StartupInput};

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StartupReplayConfig {
    pub schema_version: u32,
    pub simulation_only: bool,
    pub startup: StartupConfig,
}

pub fn replay(config: &Path, events: &Path, writer: &mut impl Write) -> Result<bool, String> {
    let config_bytes = read_regular_file(config, MAX_CONFIG_BYTES)?;
    let spec: StartupReplayConfig =
        serde_json::from_slice(&config_bytes).map_err(|e| e.to_string())?;
    if spec.schema_version != 1 || !spec.simulation_only {
        return Err("startup-replay requires schema_version=1 and simulation_only=true".into());
    }
    let mut assist = StartupAssist::new(spec.startup).map_err(|e| e.to_string())?;
    let bytes = read_regular_file(events, 4 * 1024 * 1024)?;
    let text = std::str::from_utf8(&bytes).map_err(|e| e.to_string())?;
    let mut inputs = Vec::new();
    for (index, line) in text.lines().enumerate() {
        if line.trim().is_empty() {
            continue;
        }
        if inputs.len() >= 10_000 {
            return Err("startup replay exceeds 10000 events".into());
        }
        inputs.push(
            serde_json::from_str::<StartupInput>(line)
                .map_err(|e| format!("startup event {}: {e}", index + 1))?,
        );
    }
    // A complete replay explicitly closes its episode with Stop. There is no
    // implicitly retained PWM on EOF; this is not a streaming actuator sink.
    if inputs.is_empty()
        || inputs
            .last()
            .is_none_or(|v| v.command != MotionOutput::Stop)
    {
        return Err("startup replay must end with an explicit Stop event".into());
    }
    let mut faulted = false;
    for input in inputs {
        let decision = assist.update(input);
        faulted |= decision.phase == xt_stcar_robot_core::startup_assist::StartupPhase::Fault;
        let record = serde_json::json!({"kind":"startup_preview", "physical_output_enabled":false,
            "simulation_only":true, "vehicle_calibration_verified":false, "decision":decision});
        serde_json::to_writer(&mut *writer, &record).map_err(|e| e.to_string())?;
        writer.write_all(b"\n").map_err(|e| e.to_string())?;
    }
    Ok(faulted)
}
