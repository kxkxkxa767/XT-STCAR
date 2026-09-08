//! Timestamped sensor snapshots enter the same controller used by the closed-loop plant.
//! No motion command field is accepted; this is a diagnostic input adapter, not a race recording strategy.
use crate::autonomy::{AutonomyConfig, AutonomyController, Result, RoadFrame};
use crate::input::read_regular_file;
use crate::telemetry::{RunJournal, TelemetryConfig, TelemetryMode};
use serde::{Deserialize, Serialize};
use std::io::Write;
use std::path::Path;
use xt_stcar_robot_core::autonomy::PoseEstimate;
use xt_stcar_robot_core::mission::MissionPhase;
use xt_stcar_robot_core::{LidarSample, Timestamp};

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct SensorSnapshot {
    pub at: Timestamp,
    pub pose: PoseEstimate,
    pub scan: LidarSample,
    pub road: RoadFrame,
}

pub fn read_snapshots(path: &Path) -> Result<Vec<SensorSnapshot>> {
    let bytes = read_regular_file(path, 64 * 1024 * 1024)?;
    let text = std::str::from_utf8(&bytes).map_err(|e| e.to_string())?;
    let mut snapshots = Vec::new();
    let mut previous = None;
    for (index, line) in text.lines().enumerate() {
        if line.trim().is_empty() {
            continue;
        }
        if line.len() > 1024 * 1024 || snapshots.len() >= 10_000 {
            return Err("snapshot input exceeds bounded line/record limits".into());
        }
        let record: SensorSnapshot =
            serde_json::from_str(line).map_err(|e| format!("snapshot line {}: {e}", index + 1))?;
        if previous.is_some_and(|old| record.at <= old)
            || record.scan.ranges_m.len() > 1440
            || record.road.observation.cones_body_m.len() > 256
        {
            return Err(format!(
                "snapshot line {} has invalid time or sample limits",
                index + 1
            ));
        }
        previous = Some(record.at);
        snapshots.push(record);
    }
    if snapshots.is_empty() {
        return Err("snapshot stream is empty".into());
    }
    Ok(snapshots)
}

#[derive(Debug, Serialize)]
pub struct SnapshotSummary {
    pub kind: &'static str,
    pub physical_output_enabled: bool,
    pub input_snapshots: usize,
    pub processed_snapshots: usize,
    pub completed: bool,
    pub fault: Option<String>,
    pub dropped_trace_records: u64,
    pub logging_errors: usize,
}

pub fn replay_snapshots(
    config: AutonomyConfig,
    snapshots: &[SensorSnapshot],
    telemetry: TelemetryConfig,
    writer: &mut impl Write,
) -> Result<SnapshotSummary> {
    if snapshots.is_empty() || snapshots.len() > 10_000 {
        return Err("snapshot stream must contain 1..10000 records".into());
    }
    let mut controller = AutonomyController::new(config)?;
    controller.start()?;
    let mut journal = RunJournal::new(telemetry.clone())?;
    let mut last_phase = None;
    let mut completed = false;
    let mut fault = None;
    let mut processed = 0;
    let mut logging_errors = 0;
    for record in snapshots {
        let step = controller.tick(record.at, &record.pose, &record.scan, &record.road);
        processed += 1;
        let phase = step.mission.as_ref().map(|m| m.phase);
        let important = step.fault.is_some()
            || (phase != last_phase && telemetry.mode != TelemetryMode::Summary);
        if (important || telemetry.mode == TelemetryMode::Trace)
            && journal.record(&step, important).is_err()
        {
            logging_errors += 1;
        }
        last_phase = phase;
        completed = phase == Some(MissionPhase::Completed)
            && step.fault.is_none()
            && step.safety.state == xt_stcar_robot_core::State::Running;
        if step.fault.is_some() || completed {
            fault = step.fault;
            break;
        }
    }
    if !completed && fault.is_none() {
        fault = Some("sensor snapshot stream ended before mission completion".into());
    }
    journal.write_to(writer).map_err(|e| e.to_string())?;
    serde_json::to_writer(
        &mut *writer,
        &serde_json::json!({"kind":"autonomy_terminal","physical_output_enabled":false,
        "command":{"type":"stop"},"at":snapshots[processed-1].at}),
    )
    .map_err(|e| e.to_string())?;
    writer.write_all(b"\n").map_err(|e| e.to_string())?;
    let summary = SnapshotSummary {
        kind: "snapshot_summary",
        physical_output_enabled: false,
        input_snapshots: snapshots.len(),
        processed_snapshots: processed,
        completed,
        fault,
        dropped_trace_records: journal.dropped_records(),
        logging_errors,
    };
    serde_json::to_writer(&mut *writer, &summary).map_err(|e| e.to_string())?;
    writer.write_all(b"\n").map_err(|e| e.to_string())?;
    Ok(summary)
}
