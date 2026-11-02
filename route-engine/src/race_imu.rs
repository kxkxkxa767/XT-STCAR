//! Parallel IMU evidence on the lidar receive clock. The race loop never waits for a sample.
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use xt_stcar_device_io::SerialPort;
use xt_stcar_robot_core::protocol::imu::{ImuConfig, ImuDecoder};
use xt_stcar_robot_core::{FrameId, Timestamp, Vec3};

pub fn record(out: &Path, base: Instant, stop: &AtomicBool) {
    let device = Path::new("/dev/imu");
    let (mut samples, mut checksum_errors) = (0_u64, 0_usize);
    let result = (|| -> Result<(), Box<dyn std::error::Error>> {
        let mut port = SerialPort::open(device, 115200)?;
        let zero = Vec3 { x: 0., y: 0., z: 0. };
        let mut decoder = ImuDecoder::new(ImuConfig {
            frame_id: FrameId("imu_link".into()),
            acceleration_bias_mps2: zero,
            gyro_bias_radps: zero,
            max_component_age_ms: 100,
            max_component_skew_ms: 20,
        })?;
        let mut log = BufWriter::new(File::create(out.join("imu.jsonl"))?);
        let mut bytes = [0_u8; 4096];
        while !stop.load(Ordering::Relaxed) {
            if let Some(n) = port.read_until(&mut bytes, Instant::now() + Duration::from_millis(50))? {
                let batch = decoder.feed(&bytes[..n], Timestamp(base.elapsed().as_millis() as u64))?;
                checksum_errors += batch.checksum_errors;
                for sample in batch.samples {
                    serde_json::to_writer(&mut log, &sample)?;
                    log.write_all(b"\n")?;
                    samples += 1;
                }
            }
        }
        log.flush()?;
        Ok(())
    })();
    let summary = serde_json::json!({
        "device": device, "samples": samples, "checksum_errors": checksum_errors,
        "clock": "captured_at in ms from the same Instant as lidar.received_program_s",
        "error": result.err().map(|e| e.to_string())
    });
    if let Err(e) = std::fs::write(out.join("imu-summary.json"), summary.to_string()) {
        eprintln!("IMU summary: {e}");
    }
}
