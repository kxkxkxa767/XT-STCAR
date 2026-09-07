//! File-backed camera and event input. No device or vendor protocol is assumed.
use image::{ImageReader, RgbImage};
use serde::Deserialize;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::{Path, PathBuf};
use xt_stcar_robot_core::{TimedEvent, Timestamp};

pub type Result<T> = std::result::Result<T, String>;

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TimedFrame {
    pub at: Timestamp,
    pub event: FrameEvent,
}

#[derive(Debug, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum FrameEvent {
    VisionFrame {
        path: PathBuf,
        sequence: u64,
        frame_id: String,
    },
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TimedImuBytes {
    pub at: Timestamp,
    pub event: ImuBytesEvent,
}

#[derive(Debug, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum ImuBytesEvent {
    ImuBytes { bytes: Vec<u8> },
}

pub enum ReplayEvent {
    ImuBytes(TimedImuBytes),
    Core(TimedEvent),
    Frame(TimedFrame),
}

/// Resolve all source records before producing any replay output.
/// Times are boot-relative monotonic milliseconds, not Unix time.
pub fn read_events(path: &Path) -> Result<Vec<ReplayEvent>> {
    let path = path
        .canonicalize()
        .map_err(|e| format!("events {}: {e}", path.display()))?;
    let file = File::open(&path).map_err(|e| e.to_string())?;
    if file.metadata().map_err(|e| e.to_string())?.len() > 64 * 1024 * 1024 {
        return Err("event input exceeds 64 MiB".into());
    }
    let base = path.parent().ok_or("event file has no parent")?;
    let mut events = Vec::new();
    let mut previous_time = None;
    let mut previous_frame_sequence = None;
    for (index, line) in BufReader::new(file).lines().enumerate() {
        let line = line.map_err(|e| e.to_string())?;
        if line.trim().is_empty() {
            continue;
        }
        if line.len() > 1024 * 1024 || events.len() >= 100_000 {
            return Err(format!("event line {} exceeds input limits", index + 1));
        }
        let value: serde_json::Value =
            serde_json::from_str(&line).map_err(|e| format!("event line {}: {e}", index + 1))?;
        // Internally tagged unit variants may ignore extra map keys despite
        // deny_unknown_fields; check the JSON envelope before deserializing.
        let allowed: &[&str] = match value["event"]["type"].as_str() {
            Some(
                "heartbeat" | "arm" | "start" | "disarm" | "emergency_stop" | "reset_fault"
                | "tick",
            ) => &["type"],
            Some("deadman") => &["type", "pressed"],
            Some("motion") => &["type", "intent"],
            Some("sensor") => &["type", "sample"],
            Some("imu_bytes") => &["type", "bytes"],
            Some("vision_frame") => &["type", "path", "sequence", "frame_id"],
            _ => {
                return Err(format!(
                    "unknown or missing event type at line {}",
                    index + 1
                ));
            }
        };
        if value["event"]
            .as_object()
            .is_some_and(|event| event.keys().any(|key| !allowed.contains(&key.as_str())))
        {
            return Err(format!("unknown event field at line {}", index + 1));
        }
        let (event, at) = if value["event"]["type"] == "vision_frame" {
            let mut frame: TimedFrame = serde_json::from_str(&line)
                .map_err(|e| format!("frame line {}: {e}", index + 1))?;
            let FrameEvent::VisionFrame {
                path,
                sequence,
                frame_id,
            } = &mut frame.event;
            if frame_id.is_empty() || frame_id.len() > 128 || frame_id.chars().any(char::is_control)
            {
                return Err(format!("invalid frame_id at line {}", index + 1));
            }
            if previous_frame_sequence.is_some_and(|old| *sequence <= old) {
                return Err(format!(
                    "camera frame sequence must increase at line {}",
                    index + 1
                ));
            }
            previous_frame_sequence = Some(*sequence);
            let resolved = if path.is_absolute() {
                path.clone()
            } else {
                base.join(&*path)
            };
            *path = resolved
                .canonicalize()
                .map_err(|e| format!("frame {}: {e}", resolved.display()))?;
            if !path.is_file() {
                return Err("camera frame must be an existing file".into());
            }
            let at = frame.at.0;
            (ReplayEvent::Frame(frame), at)
        } else if value["event"]["type"] == "imu_bytes" {
            let chunk: TimedImuBytes =
                serde_json::from_str(&line).map_err(|e| format!("IMU line {}: {e}", index + 1))?;
            let ImuBytesEvent::ImuBytes { bytes } = &chunk.event;
            if bytes.is_empty() || bytes.len() > 4096 {
                return Err("IMU chunk must contain 1..4096 bytes".into());
            }
            let at = chunk.at.0;
            (ReplayEvent::ImuBytes(chunk), at)
        } else {
            let core: TimedEvent = serde_json::from_str(&line)
                .map_err(|e| format!("event line {}: {e}", index + 1))?;
            let at = core.at.0;
            (ReplayEvent::Core(core), at)
        };
        if previous_time.is_some_and(|old| at < old) {
            return Err(format!(
                "event time must not go backwards at line {}",
                index + 1
            ));
        }
        previous_time = Some(at);
        events.push(event);
    }
    if events.is_empty() {
        return Err("event stream must not be empty".into());
    }
    Ok(events)
}

pub fn load_frame(path: &Path) -> Result<RgbImage> {
    let reader = || {
        ImageReader::open(path)
            .map_err(|e| e.to_string())?
            .with_guessed_format()
            .map_err(|e| e.to_string())
    };
    let (width, height) = reader()?
        .into_dimensions()
        .map_err(|e| format!("image dimensions: {e}"))?;
    if width == 0 || height == 0 || u64::from(width) * u64::from(height) > 64_000_000 {
        return Err("camera frame must be positive and at most 64 megapixels".into());
    }
    let mut decoder = reader()?;
    let mut limits = image::Limits::default();
    limits.max_alloc = Some(512 * 1024 * 1024);
    decoder.limits(limits);
    decoder
        .decode()
        .map(|image| image.to_rgb8())
        .map_err(|e| format!("decode frame: {e}"))
}
