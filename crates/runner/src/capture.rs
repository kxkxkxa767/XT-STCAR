//! Explicit sensor-only capture, producing replayable raw-byte events and idle ticks.
use crate::{input::Result, json_line};
use serde::{Deserialize, Serialize};
use std::{
    io::Write,
    path::PathBuf,
    time::{Duration, Instant},
};
use xt_stcar_device_io::SerialPort;

#[derive(Debug, Clone, Copy, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum SensorProtocol {
    ImuWit11,
    N10,
}
impl SensorProtocol {
    pub fn baud(self) -> u32 {
        match self {
            Self::ImuWit11 => 115200,
            Self::N10 => 230400,
        }
    }
    fn event(self) -> &'static str {
        match self {
            Self::ImuWit11 => "imu_bytes",
            Self::N10 => "n10_bytes",
        }
    }
}
#[derive(Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct CaptureConfig {
    pub device: PathBuf,
    pub protocol: SensorProtocol,
    pub duration_ms: u64,
    pub read_timeout_ms: u64,
    pub max_bytes: usize,
}
impl CaptureConfig {
    pub fn validate(&self) -> Result<()> {
        if !self.device.is_absolute()
            || !self.device.starts_with("/dev")
            || self
                .device
                .components()
                .any(|p| matches!(p, std::path::Component::ParentDir))
        {
            return Err("capture requires an explicit absolute /dev path".into());
        }
        if !(1..=60_000).contains(&self.duration_ms)
            || !(1..=1000).contains(&self.read_timeout_ms)
            || !(1..=512 * 1024).contains(&self.max_bytes)
        {
            return Err("capture duration/timeout/byte limit invalid".into());
        }
        Ok(())
    }
}
#[derive(Debug, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum CaptureEnd {
    DurationLimit,
    ByteLimit,
    RecordLimit,
}
#[derive(Debug, Serialize)]
pub struct CaptureSummary {
    pub bytes: usize,
    pub records: usize,
    pub elapsed_ms: u64,
    pub ended_by: CaptureEnd,
    pub device_commands_sent: bool,
}

pub fn capture(config: &CaptureConfig, writer: &mut impl Write) -> Result<CaptureSummary> {
    config.validate()?;
    let mut port = SerialPort::open(&config.device, config.protocol.baud())
        .map_err(|e| format!("serial open/configuration: {e}"))?;
    let start = Instant::now();
    let end = start + Duration::from_millis(config.duration_ms);
    let mut bytes = 0;
    let mut records = 0;
    let mut buffer = [0; 4096];
    while Instant::now() < end && bytes < config.max_bytes && records < 10_000 {
        let deadline = (Instant::now() + Duration::from_millis(config.read_timeout_ms)).min(end);
        let limit = buffer.len().min(config.max_bytes - bytes);
        let count = port
            .read_until(&mut buffer[..limit], deadline)
            .map_err(|e| format!("serial capture: {e}"))?;
        let at = start.elapsed().as_millis() as u64;
        let event = match count {
            Some(n) => {
                bytes += n;
                serde_json::json!({"type":config.protocol.event(),"bytes":&buffer[..n]})
            }
            None => serde_json::json!({"type":"tick"}),
        };
        json_line(writer, &serde_json::json!({"at":at,"event":event}))?;
        records += 1;
    }
    let elapsed_ms = start.elapsed().as_millis() as u64;
    json_line(
        writer,
        &serde_json::json!({"at":elapsed_ms,"event":{"type":"tick"}}),
    )?;
    writer.flush().map_err(|e| e.to_string())?;
    let ended_by = if bytes >= config.max_bytes {
        CaptureEnd::ByteLimit
    } else if records >= 10_000 {
        CaptureEnd::RecordLimit
    } else {
        CaptureEnd::DurationLimit
    };
    Ok(CaptureSummary {
        bytes,
        records: records + 1,
        elapsed_ms,
        ended_by,
        device_commands_sent: false,
    })
}
