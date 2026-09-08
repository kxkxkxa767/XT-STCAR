//! Offline robot orchestration: validated replay, persistent vision, safety and logs.
//! All motion output is recorded locally; there is no physical actuator sink.
pub mod capture;
pub mod input;
pub mod vision;

use input::{FrameEvent, ReplayEvent, Result, read_events};
use input::{ImuBytesEvent, N10BytesEvent};
use input::{MAX_CONFIG_BYTES, read_regular_file};
use serde::Serialize;
use std::io::Write;
use std::path::PathBuf;
use vision::{VisionOptions, VisionPipeline};
use xt_stcar_robot_core::protocol::calibrated_chassis::{CalibratedChassis, ChassisCalibration};
use xt_stcar_robot_core::protocol::imu::{ImuConfig, ImuDecoder};
use xt_stcar_robot_core::protocol::n10::{N10Config, N10Decoder};
use xt_stcar_robot_core::{
    Controller, Event, FrameId, MotionOutput, MotionSink, RecordingSink, SafetyConfig,
    SensorSample, State, StepReport, TimedEvent, Timestamp, VisionSample,
};

pub struct ReplayOptions {
    pub config: PathBuf,
    pub events: PathBuf,
    pub vision: Option<VisionOptions>,
    pub imu_config: Option<PathBuf>,
    pub n10_config: Option<PathBuf>,
    pub chassis_calibration: Option<PathBuf>,
}

#[derive(Debug, Serialize)]
pub struct RunSummary {
    pub kind: &'static str,
    pub mode: &'static str,
    pub physical_output_enabled: bool,
    pub input_events: usize,
    pub vision_frames: usize,
    pub imu_samples: usize,
    pub n10_packets: usize,
    pub lidar_samples: usize,
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

/// Validate the optional physical-unit mapping before recording any Drive.
/// A rejected mapping records a latched stop and a complete terminal diagnostic.
fn record_step(
    controller: &mut Controller,
    sink: &mut RecordingSink,
    writer: &mut impl Write,
    report: StepReport,
    calibration: Option<&CalibratedChassis>,
    mut extra: serde_json::Value,
) -> Result<()> {
    let preview = match calibration
        .map(|mapping| mapping.preview(&report.output.command))
        .transpose()
    {
        Ok(preview) => preview,
        Err(error) => {
            let stop = controller.handle(TimedEvent {
                at: report.event_at,
                event: Event::EmergencyStop,
            });
            sink.emit(&stop.output).map_err(|e| e.to_string())?;
            let command = calibration.expect("mapping was present").stop_command();
            json_line(
                writer,
                &serde_json::json!({
                    "kind":"step", "mode":"replay", "physical_output_enabled":false,
                    "terminal":"chassis_error", "step":stop, "error":error.to_string(),
                    "rejected_output":report.output,
                    "chassis_preview":{"command":command,"bytes":command.encode(),"simulation_only":true,"measurement_status":"unverified"}
                }),
            )?;
            writer.flush().map_err(|e| e.to_string())?;
            return Err(format!(
                "chassis mapping failed; stop was recorded: {error}"
            ));
        }
    };
    sink.emit(&report.output).map_err(|e| e.to_string())?;
    extra["kind"] = "step".into();
    extra["mode"] = "replay".into();
    extra["physical_output_enabled"] = false.into();
    extra["step"] = serde_json::to_value(report).map_err(|e| e.to_string())?;
    if let Some(command) = preview {
        extra["chassis_preview"] = serde_json::json!({"command":command,"bytes":command.encode(),"simulation_only":true,"measurement_status":"unverified"});
    }
    json_line(writer, &extra)
}

/// Run a deterministic recording. Simulated timestamps do not delay wall-clock time.
/// A final disarm is always recorded, even when the input ends in Running.
pub fn replay(options: ReplayOptions, writer: &mut impl Write) -> Result<RunSummary> {
    let config_bytes = read_regular_file(&options.config, MAX_CONFIG_BYTES)
        .map_err(|e| format!("robot config: {e}"))?;
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
    let has_imu = events.iter().any(|e| matches!(e, ReplayEvent::ImuBytes(_)));
    if has_imu != options.imu_config.is_some() {
        return Err(
            "imu_bytes input requires --imu-config, and the option requires imu_bytes input".into(),
        );
    }
    if has_imu
        && events.iter().any(|e| {
            matches!(
                e,
                ReplayEvent::Core(TimedEvent {
                    event: Event::Sensor {
                        sample: SensorSample::Imu(_)
                    },
                    ..
                })
            )
        })
    {
        return Err("cannot mix raw and decoded IMU sources in one replay".into());
    }
    let mut imu = options
        .imu_config
        .as_ref()
        .map(|path| {
            let bytes = read_regular_file(path, MAX_CONFIG_BYTES)?;
            let spec: ImuConfig = serde_json::from_slice(&bytes).map_err(|e| e.to_string())?;
            if spec.frame_id != config.frames.imu_frame {
                return Err(
                    "IMU frame differs from robot config; explicit transform required".into(),
                );
            }
            ImuDecoder::new(spec).map_err(|e| e.to_string())
        })
        .transpose()?;
    let has_n10 = events.iter().any(|e| matches!(e, ReplayEvent::N10Bytes(_)));
    if has_n10 != options.n10_config.is_some() {
        return Err(
            "n10_bytes input requires --n10-config, and the option requires n10_bytes input".into(),
        );
    }
    if has_n10
        && events.iter().any(|e| {
            matches!(
                e,
                ReplayEvent::Core(TimedEvent {
                    event: Event::Sensor {
                        sample: SensorSample::Lidar(_)
                    },
                    ..
                })
            )
        })
    {
        return Err("cannot mix raw N10 and decoded lidar sources in one replay".into());
    }
    let mut n10 = options
        .n10_config
        .as_ref()
        .map(|path| {
            let bytes = read_regular_file(path, MAX_CONFIG_BYTES)?;
            let spec: N10Config = serde_json::from_slice(&bytes).map_err(|e| e.to_string())?;
            if spec.frame_id != config.frames.lidar_frame {
                return Err(
                    "N10 frame differs from robot config; explicit transform required".into(),
                );
            }
            N10Decoder::new(spec).map_err(|e| e.to_string())
        })
        .transpose()?;
    let calibration = options
        .chassis_calibration
        .as_ref()
        .map(|path| {
            let bytes = read_regular_file(path, MAX_CONFIG_BYTES)?;
            let spec: ChassisCalibration =
                serde_json::from_slice(&bytes).map_err(|e| e.to_string())?;
            CalibratedChassis::new(spec).map_err(|e| e.to_string())
        })
        .transpose()?;
    let mut controller = Controller::new(config).map_err(|e| e.to_string())?;
    // Initialize and validate the real model before producing any control records.
    let mut pipeline = options
        .vision
        .as_ref()
        .map(VisionPipeline::new)
        .transpose()?;
    let mut sink = RecordingSink::default();
    let mut vision_frames = 0;
    let mut imu_samples = 0;
    let mut n10_packets = 0;
    let mut lidar_samples = 0;
    let mut last_at = Timestamp(0);
    for event in events {
        if let ReplayEvent::ImuBytes(chunk) = event {
            last_at = chunk.at;
            let ImuBytesEvent::ImuBytes { bytes } = chunk.event;
            let batch = imu
                .as_mut()
                .ok_or("missing IMU decoder")?
                .feed(&bytes, chunk.at)
                .map_err(|e| e.to_string())?;
            imu_samples += batch.samples.len();
            // An incomplete/invalid packet is a Tick, never a refreshed sensor.
            let sensor_events: Vec<Event> = if batch.samples.is_empty() {
                vec![Event::Tick]
            } else {
                batch
                    .samples
                    .iter()
                    .cloned()
                    .map(|sample| Event::Sensor {
                        sample: SensorSample::Imu(sample),
                    })
                    .collect()
            };
            let mut diagnostic = Some(serde_json::to_value(&batch).map_err(|e| e.to_string())?);
            for (sample_index, sensor_event) in sensor_events.into_iter().enumerate() {
                let report = controller.handle(TimedEvent {
                    at: chunk.at,
                    event: sensor_event,
                });
                let mut extra = serde_json::json!({"imu_sample_index":sample_index});
                if let Some(batch) = diagnostic.take() {
                    extra["imu_decode"] = batch;
                }
                record_step(
                    &mut controller,
                    &mut sink,
                    writer,
                    report,
                    calibration.as_ref(),
                    extra,
                )?;
            }
            continue;
        }
        if let ReplayEvent::N10Bytes(chunk) = event {
            last_at = chunk.at;
            let N10BytesEvent::N10Bytes { bytes } = chunk.event;
            let batch = n10
                .as_mut()
                .ok_or("missing N10 decoder")?
                .feed(&bytes, chunk.at)
                .map_err(|e| e.to_string())?;
            n10_packets += batch.packets.len();
            let mut sensor_events: Vec<Event> = batch
                .packets
                .iter()
                .map(|packet| {
                    if let Some(sample) = packet.packet_sample() {
                        lidar_samples += 1;
                        Event::Sensor {
                            sample: SensorSample::Lidar(sample),
                        }
                    } else {
                        Event::Tick
                    }
                })
                .collect();
            if sensor_events.is_empty() {
                sensor_events.push(Event::Tick);
            }
            let mut diagnostic = Some(serde_json::to_value(&batch).map_err(|e| e.to_string())?);
            for (packet_index, sensor_event) in sensor_events.into_iter().enumerate() {
                let report = controller.handle(TimedEvent {
                    at: chunk.at,
                    event: sensor_event,
                });
                let mut extra = serde_json::json!({"n10_packet_index":packet_index,"lidar_coverage":"partial_packet"});
                if let Some(batch) = diagnostic.take() {
                    extra["n10_decode"] = batch;
                }
                record_step(
                    &mut controller,
                    &mut sink,
                    writer,
                    report,
                    calibration.as_ref(),
                    extra,
                )?;
            }
            continue;
        }
        let (timed, perception) = match event {
            ReplayEvent::ImuBytes(_) | ReplayEvent::N10Bytes(_) => unreachable!("handled above"),
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
                        record_step(
                            &mut controller,
                            &mut sink,
                            writer,
                            stop,
                            calibration.as_ref(),
                            serde_json::json!({"terminal":"vision_error", "error":error}),
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
        record_step(
            &mut controller,
            &mut sink,
            writer,
            report,
            calibration.as_ref(),
            serde_json::json!({"vision":perception}),
        )?;
    }
    // An ended command stream must not leave even the simulated sink at Drive.
    let terminal = controller.handle(TimedEvent {
        at: last_at,
        event: Event::Disarm,
    });
    record_step(
        &mut controller,
        &mut sink,
        writer,
        terminal,
        calibration.as_ref(),
        serde_json::json!({"terminal":"end_of_stream"}),
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
        imu_samples,
        n10_packets,
        lidar_samples,
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
