//! Offline robot orchestration: validated replay, persistent vision, safety and logs.
//! All motion output is recorded locally; there is no physical actuator sink.
pub mod input;
pub mod vision;

use input::{FrameEvent, ReplayEvent, Result, read_events};
use serde::Serialize;
use std::io::Write;
use std::path::PathBuf;
use vision::{VisionOptions, VisionPipeline};
use xt_stcar_robot_core::{
    Controller, Event, FrameId, MotionOutput, MotionSink, RecordingSink, SafetyConfig,
    SensorSample, State, TimedEvent, Timestamp, VisionSample,
};

pub struct ReplayOptions {
    pub config: PathBuf,
    pub events: PathBuf,
    pub vision: Option<VisionOptions>,
}

#[derive(Debug, Serialize)]
pub struct RunSummary {
    pub kind: &'static str,
    pub mode: &'static str,
    pub physical_output_enabled: bool,
    pub input_events: usize,
    pub vision_frames: usize,
    pub motion_records: usize,
    pub drive_records: usize,
    pub stop_records: usize,
    pub final_state: State,
}

fn json_line(writer: &mut impl Write, record: &impl Serialize) -> Result<()> {
    let mut bytes = serde_json::to_vec(record).map_err(|e| format!("serialize replay log: {e}"))?;
    bytes.push(b'\n');
    writer
        .write_all(&bytes)
        .map_err(|e| format!("write replay log: {e}"))
}

/// Run a deterministic recording. Simulated timestamps do not delay wall-clock time.
/// A final disarm is always recorded, even when the input ends in Running.
pub fn replay(options: ReplayOptions, writer: &mut impl Write) -> Result<RunSummary> {
    let config_bytes = std::fs::read(&options.config).map_err(|e| format!("robot config: {e}"))?;
    let config: SafetyConfig =
        serde_json::from_slice(&config_bytes).map_err(|e| format!("robot config: {e}"))?;
    let events = read_events(&options.events)?;
    let input_events = events.len();
    let has_frames = events.iter().any(|e| matches!(e, ReplayEvent::Frame(_)));
    if has_frames && options.vision.is_none() {
        return Err(
            "vision_frame events require --model and --runtime-lib; no replay was executed".into(),
        );
    }
    if !has_frames && options.vision.is_some() {
        return Err(
            "model/runtime options supplied but the event stream has no vision_frame".into(),
        );
    }
    let mut controller = Controller::new(config).map_err(|e| e.to_string())?;
    // Initialize and validate the real model before producing any control records.
    let mut pipeline = options
        .vision
        .as_ref()
        .map(VisionPipeline::new)
        .transpose()?;
    let mut sink = RecordingSink::default();
    let mut vision_frames = 0;
    let mut last_at = Timestamp(0);
    for event in events {
        let (timed, perception) = match event {
            ReplayEvent::Core(event) => (event, None),
            ReplayEvent::Frame(frame) => {
                let FrameEvent::VisionFrame {
                    path,
                    sequence,
                    frame_id,
                } = frame.event;
                let result = pipeline
                    .as_mut()
                    .ok_or("missing vision pipeline")?
                    .infer(&path, sequence, &frame_id);
                let perception = match result {
                    Ok(perception) => perception,
                    Err(error) => {
                        let stop = controller.handle(TimedEvent {
                            at: frame.at,
                            event: Event::EmergencyStop,
                        });
                        sink.emit(&stop.output).map_err(|e| e.to_string())?;
                        json_line(
                            writer,
                            &serde_json::json!({
                                "kind": "step", "mode": "replay", "physical_output_enabled": false,
                                "terminal": "vision_error", "step": stop, "error": error,
                            }),
                        )?;
                        writer.flush().map_err(|e| e.to_string())?;
                        return Err(format!("vision failed; stop was recorded: {error}"));
                    }
                };
                let sample = SensorSample::Vision(VisionSample {
                    captured_at: frame.at,
                    frame_id: FrameId(frame_id),
                    image_width_px: perception.transform.source_width,
                    image_height_px: perception.transform.source_height,
                    detection_count: perception.detections.len() as u32,
                });
                vision_frames += 1;
                (
                    TimedEvent {
                        at: frame.at,
                        event: Event::Sensor { sample },
                    },
                    Some(perception),
                )
            }
        };
        last_at = timed.at;
        let report = controller.handle(timed);
        sink.emit(&report.output).map_err(|e| e.to_string())?;
        json_line(
            writer,
            &serde_json::json!({
                "kind": "step", "mode": "replay", "physical_output_enabled": false,
                "step": report, "vision": perception,
            }),
        )?;
    }
    // An ended command stream must not leave even the simulated sink at Drive.
    let terminal = controller.handle(TimedEvent {
        at: last_at,
        event: Event::Disarm,
    });
    sink.emit(&terminal.output).map_err(|e| e.to_string())?;
    json_line(
        writer,
        &serde_json::json!({
            "kind": "step", "mode": "replay", "physical_output_enabled": false,
            "terminal": "end_of_stream", "step": terminal,
        }),
    )?;
    let drive_records = sink
        .records()
        .iter()
        .filter(|record| matches!(record.command, MotionOutput::Drive { .. }))
        .count();
    let summary = RunSummary {
        kind: "summary",
        mode: "replay",
        physical_output_enabled: false,
        input_events,
        vision_frames,
        motion_records: sink.records().len(),
        drive_records,
        stop_records: sink.records().len() - drive_records,
        final_state: controller.state(),
    };
    json_line(writer, &summary)?;
    writer
        .flush()
        .map_err(|e| format!("flush replay log: {e}"))?;
    Ok(summary)
}
