//! Explicit WIT normal 11-byte profile matching the factory 0x51/52/53 parser.
//! Timestamps are host receive times, not device measurement times.
use crate::{FrameId, ImuSample, Quaternion, Timestamp, ValidationError, Vec3};
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use std::f64::consts::PI;

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ImuConfig {
    pub frame_id: FrameId,
    pub acceleration_bias_mps2: Vec3,
    pub gyro_bias_radps: Vec3,
    pub max_component_age_ms: u64,
    pub max_component_skew_ms: u64,
}

#[derive(Clone, Debug, Default, Serialize)]
pub struct DecodeBatch {
    pub samples: Vec<ImuSample>,
    pub checksum_errors: usize,
    pub discarded_bytes: usize,
    pub expired_components: usize,
}

pub struct ImuDecoder {
    config: ImuConfig,
    // At most 11 bytes, each retaining its receive time across fragmented reads.
    pending: VecDeque<(u8, Timestamp)>,
    components: [Option<(Vec3, Timestamp)>; 3],
    last_received: Option<Timestamp>,
}

impl ImuDecoder {
    pub fn new(config: ImuConfig) -> Result<Self, ValidationError> {
        config.frame_id.validate()?;
        if config.max_component_age_ms == 0
            || config.max_component_skew_ms == 0
            || config.max_component_skew_ms > config.max_component_age_ms
            || ![config.acceleration_bias_mps2, config.gyro_bias_radps]
                .iter()
                .all(|v| [v.x, v.y, v.z].iter().all(|x| x.is_finite()))
        {
            return Err(ValidationError("invalid IMU timing or calibration".into()));
        }
        Ok(Self {
            config,
            pending: VecDeque::with_capacity(11),
            components: [None; 3],
            last_received: None,
        })
    }

    pub fn feed(&mut self, bytes: &[u8], at: Timestamp) -> Result<DecodeBatch, ValidationError> {
        if bytes.len() > 4096 || self.last_received.is_some_and(|old| at < old) {
            return Err(ValidationError(
                "IMU chunk exceeds 4096 bytes or receive time regresses".into(),
            ));
        }
        self.last_received = Some(at);
        let mut batch = DecodeBatch::default();
        for component in &mut self.components {
            if component.is_some_and(|(_, time)| at.0 - time.0 >= self.config.max_component_age_ms)
            {
                *component = None;
                batch.expired_components += 1;
            }
        }
        if self
            .pending
            .front()
            .is_some_and(|(_, time)| at.0 - time.0 >= self.config.max_component_age_ms)
        {
            batch.discarded_bytes += self.pending.len();
            self.pending.clear();
        }
        for &byte in bytes {
            self.pending.push_back((byte, at));
            while let Some(&(head, _)) = self.pending.front() {
                if head != 0x55 {
                    self.pending.pop_front();
                    batch.discarded_bytes += 1;
                    continue;
                }
                if self.pending.len() < 11 {
                    break;
                }
                let frame: Vec<_> = self.pending.iter().take(11).copied().collect();
                let checksum = frame[..10]
                    .iter()
                    .fold(0u8, |sum, (v, _)| sum.wrapping_add(*v));
                if checksum != frame[10].0 {
                    batch.checksum_errors += 1;
                    self.pending.pop_front();
                    batch.discarded_bytes += 1;
                    // Do not combine earlier components with data after corruption.
                    self.components = [None; 3];
                    continue;
                }
                self.pending.drain(..11);
                let kind = frame[1].0;
                if !(0x51..=0x53).contains(&kind) {
                    continue;
                }
                let scale = match kind {
                    0x51 => 16.0 * 9.8,
                    0x52 => 2000.0 * PI / 180.0,
                    _ => PI,
                } / 32768.0;
                let value =
                    |i: usize| f64::from(i16::from_le_bytes([frame[i].0, frame[i + 1].0])) * scale;
                let bias = match kind {
                    0x51 => self.config.acceleration_bias_mps2,
                    0x52 => self.config.gyro_bias_radps,
                    _ => Vec3 {
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                    },
                };
                let v = Vec3 {
                    x: value(2) - bias.x,
                    y: value(4) - bias.y,
                    z: value(6) - bias.z,
                };
                self.components[usize::from(kind - 0x51)] = Some((v, frame[0].1));
                if let [Some((acc, ta)), Some((gyro, tg)), Some((angle, te))] = self.components {
                    let oldest = ta.min(tg).min(te);
                    let newest = ta.max(tg).max(te);
                    if newest.0 - oldest.0 > self.config.max_component_skew_ms {
                        for item in &mut self.components {
                            if item.is_some_and(|(_, t)| {
                                newest.0 - t.0 > self.config.max_component_skew_ms
                            }) {
                                *item = None;
                                batch.expired_components += 1;
                            }
                        }
                        continue;
                    }
                    let (sr, cr) = (angle.x / 2.0).sin_cos();
                    let (sp, cp) = (angle.y / 2.0).sin_cos();
                    let (sy, cy) = (angle.z / 2.0).sin_cos();
                    batch.samples.push(ImuSample {
                        captured_at: oldest,
                        frame_id: self.config.frame_id.clone(),
                        acceleration_mps2: acc,
                        angular_velocity_radps: gyro,
                        orientation_xyzw: Quaternion {
                            x: sr * cp * cy - cr * sp * sy,
                            y: cr * sp * cy + sr * cp * sy,
                            z: cr * cp * sy - sr * sp * cy,
                            w: cr * cp * cy + sr * sp * sy,
                        },
                    });
                    // Require three fresh components for every subsequent sample.
                    self.components = [None; 3];
                }
            }
        }
        Ok(batch)
    }
}
