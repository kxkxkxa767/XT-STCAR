//! IMU evidence and the latest calibrated yaw rate on the lidar receive clock.
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::time::{Duration, Instant};

use xt_stcar_device_io::SerialPort;
use xt_stcar_robot_core::protocol::imu::{ImuConfig, ImuDecoder};
use xt_stcar_robot_core::{FrameId, Timestamp, Vec3};

#[derive(Default)]
pub struct TimedValue(AtomicU64);

impl TimedValue {
    pub(super) fn publish(&self, at_ms: u64, value: f64) {
        // Pair each scalar with its own receive timestamp in one atomic load.
        self.0.store((at_ms << 32) | u64::from((value as f32).to_bits()), Ordering::Relaxed);
    }

    pub fn at(&self, now: f64) -> Option<f64> {
        let word = self.0.load(Ordering::Relaxed);
        let age = now - (word >> 32) as f64 * 0.001;
        (word != 0 && (0.0..=0.10).contains(&age)).then(|| f32::from_bits(word as u32) as f64)
    }
}

#[derive(Default)]
pub struct LiveImu {
    pub yaw: TimedValue,
    pub forward_integral: TimedValue,
}

pub fn record(out: &Path, base: Instant, stop: &AtomicBool, latest: &LiveImu) {
    let device = Path::new("/dev/imu");
    let (mut samples, mut checksum_errors) = (0_u64, 0_usize);
    let result = (|| -> Result<(), Box<dyn std::error::Error>> {
        let mut port = SerialPort::open(device, 115200)?;
        let calibration: serde_json::Value = serde_json::from_str(include_str!("../../config/imu-planar-y25-20261102.json"))?;
        let params = &calibration["parameters"];
        let up: [f64; 3] = serde_json::from_value(params["body_axes_in_sensor"]["up"].clone())?;
        let bias: [f64; 3] = serde_json::from_value(params["gyro_bias_radps"].clone())?;
        let gain: f64 = serde_json::from_value(params["yaw_rate_gain"].clone())?;
        let forward: [f64; 3] = serde_json::from_value(params["body_axes_in_sensor"]["forward"].clone())?;
        let rest: [f64; 3] = serde_json::from_value(params["stationary_specific_force_mps2"].clone())?;
        let accel_gain: f64 = serde_json::from_value(params["forward_acceleration_gain"].clone())?;
        let mut last_accel: Option<(u64, f64)> = None;
        let mut integral = 0.;
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
                    let w = sample.angular_velocity_radps;
                    let rate = up[0]*(w.x-bias[0]) + up[1]*(w.y-bias[1]) + up[2]*(w.z-bias[2]);
                    let at = sample.captured_at.0;
                    latest.yaw.publish(at, gain * rate);
                    let a = sample.acceleration_mps2;
                    let accel = accel_gain * (forward[0]*(a.x-rest[0]) + forward[1]*(a.y-rest[1]) + forward[2]*(a.z-rest[2]));
                    if let Some((previous_at, previous_a)) = last_accel {
                        integral += (previous_a + accel) * 0.0005 * (at - previous_at) as f64;
                    }
                    last_accel = Some((at, accel));
                    latest.forward_integral.publish(at, integral);
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
