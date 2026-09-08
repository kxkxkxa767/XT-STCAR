//! LSlidar N10's 58-byte, 16-return serial packet, checked against the factory source.
//!
//! This decoder opens no device and assembles no complete revolution. Timestamps are
//! host receive times. The two undocumented header bytes remain opaque; neither a
//! motor speed nor a measurement timestamp is inferred from them.
use crate::{FrameId, LidarSample, Timestamp, ValidationError};
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;

pub const PACKET_BYTES: usize = 58;
pub const POINTS_PER_PACKET: usize = 16;
pub const MAX_CHUNK_BYTES: usize = 4096;

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct N10Config {
    pub frame_id: FrameId,
    pub range_min_m: f64,
    pub range_max_m: f64,
    pub max_packet_age_ms: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct N10Point {
    /// Original big-endian distance word, including the invalid 0xffff sentinel.
    pub raw_distance_mm: u16,
    /// None means invalid, zero, or outside the explicitly configured range.
    pub range_m: Option<f64>,
    pub intensity: u8,
}

#[derive(Clone, Debug, Serialize)]
pub struct N10Packet {
    pub first_received_at: Timestamp,
    pub received_at: Timestamp,
    pub frame_id: FrameId,
    /// Bytes 3 and 4; their meaning is not used by the supplied N10 parser.
    pub header_bytes: [u8; 2],
    pub start_angle_cdeg: u16,
    pub end_angle_cdeg: u16,
    pub points: [N10Point; POINTS_PER_PACKET],
    range_min_m: f64,
    range_max_m: f64,
}

impl N10Packet {
    /// Convert this packet's 16 fixed slots to a partial scan in the factory
    /// driver's coordinate convention (x = r*cos(a), y = -r*sin(a)).
    ///
    /// Missing returns retain their slots; they never change other beam angles.
    /// This is receive health evidence only, not proof of full angular coverage.
    /// No sample is emitted for an entirely unknown or zero-span packet.
    pub fn packet_sample(&self) -> Option<LidarSample> {
        let span_cdeg =
            (i32::from(self.end_angle_cdeg) - i32::from(self.start_angle_cdeg)).rem_euclid(36_000);
        if span_cdeg == 0 || self.points.iter().all(|p| p.range_m.is_none()) {
            return None;
        }
        Some(LidarSample {
            captured_at: self.first_received_at,
            frame_id: self.frame_id.clone(),
            angle_min_rad: -(f64::from(self.start_angle_cdeg) / 100.0).to_radians(),
            angle_increment_rad: -(f64::from(span_cdeg) / 100.0 / (POINTS_PER_PACKET - 1) as f64)
                .to_radians(),
            range_min_m: self.range_min_m,
            range_max_m: self.range_max_m,
            ranges_m: self.points.iter().map(|p| p.range_m).collect(),
        })
    }
}

#[derive(Clone, Debug, Default, Serialize)]
pub struct N10DecodeBatch {
    pub packets: Vec<N10Packet>,
    pub checksum_errors: usize,
    pub length_errors: usize,
    pub angle_errors: usize,
    pub discarded_bytes: usize,
    pub expired_packets: usize,
}

pub struct N10Decoder {
    config: N10Config,
    // Each byte retains its receive time across reads. At most one packet is held.
    pending: VecDeque<(u8, Timestamp)>,
    last_received: Option<Timestamp>,
}

impl N10Decoder {
    pub fn new(config: N10Config) -> Result<Self, ValidationError> {
        config.frame_id.validate()?;
        if !config.range_min_m.is_finite()
            || !config.range_max_m.is_finite()
            || config.range_min_m < 0.0
            || config.range_max_m <= config.range_min_m
            || config.max_packet_age_ms == 0
        {
            return Err(ValidationError("invalid N10 ranges or packet age".into()));
        }
        Ok(Self {
            config,
            pending: VecDeque::with_capacity(PACKET_BYTES),
            last_received: None,
        })
    }

    pub fn buffered_len(&self) -> usize {
        self.pending.len()
    }

    pub fn feed(&mut self, bytes: &[u8], at: Timestamp) -> Result<N10DecodeBatch, ValidationError> {
        // Validate before mutation so an invalid call cannot destroy valid input.
        if bytes.len() > MAX_CHUNK_BYTES || self.last_received.is_some_and(|old| at < old) {
            return Err(ValidationError(
                "N10 chunk exceeds 4096 bytes or receive time regresses".into(),
            ));
        }
        self.last_received = Some(at);
        let mut batch = N10DecodeBatch::default();
        if self
            .pending
            .front()
            .is_some_and(|(_, first)| at.0 - first.0 >= self.config.max_packet_age_ms)
        {
            batch.expired_packets += 1;
            batch.discarded_bytes += self.pending.len();
            self.pending.clear();
        }
        for &byte in bytes {
            self.pending.push_back((byte, at));
            while let Some(&(head, _)) = self.pending.front() {
                if head != 0xa5 || self.pending.get(1).is_some_and(|&(b, _)| b != 0x5a) {
                    self.discard_one(&mut batch);
                    continue;
                }
                if self
                    .pending
                    .get(2)
                    .is_some_and(|&(b, _)| usize::from(b) != PACKET_BYTES)
                {
                    batch.length_errors += 1;
                    self.discard_one(&mut batch);
                    continue;
                }
                if self.pending.len() < PACKET_BYTES {
                    break;
                }
                let frame: [u8; PACKET_BYTES] = std::array::from_fn(|i| self.pending[i].0);
                // The vendor calls this CRC8, but it is a wrapping byte sum.
                let sum = frame[..PACKET_BYTES - 1]
                    .iter()
                    .fold(0u8, |acc, b| acc.wrapping_add(*b));
                if sum != frame[PACKET_BYTES - 1] {
                    batch.checksum_errors += 1;
                    self.discard_one(&mut batch);
                    continue;
                }
                let start = u16::from_be_bytes([frame[5], frame[6]]);
                let end = u16::from_be_bytes([frame[55], frame[56]]);
                if start > 36_000 || end > 36_000 {
                    batch.angle_errors += 1;
                    self.discard_one(&mut batch);
                    continue;
                }
                let points = std::array::from_fn(|i| {
                    let offset = 7 + 3 * i;
                    let raw = u16::from_be_bytes([frame[offset], frame[offset + 1]]);
                    let metres = f64::from(raw) / 1000.0;
                    let valid = raw != 0
                        && raw != u16::MAX
                        && metres >= self.config.range_min_m
                        && metres <= self.config.range_max_m;
                    N10Point {
                        raw_distance_mm: raw,
                        range_m: valid.then_some(metres),
                        intensity: frame[offset + 2],
                    }
                });
                batch.packets.push(N10Packet {
                    first_received_at: self.pending[0].1,
                    received_at: self.pending[PACKET_BYTES - 1].1,
                    frame_id: self.config.frame_id.clone(),
                    header_bytes: [frame[3], frame[4]],
                    start_angle_cdeg: start,
                    end_angle_cdeg: end,
                    points,
                    range_min_m: self.config.range_min_m,
                    range_max_m: self.config.range_max_m,
                });
                self.pending.clear();
            }
        }
        Ok(batch)
    }

    fn discard_one(&mut self, batch: &mut N10DecodeBatch) {
        self.pending.pop_front();
        batch.discarded_bytes += 1;
    }
}
