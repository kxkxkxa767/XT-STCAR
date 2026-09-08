//! Bounded N10 revolution assembly. Missing returns remain unknown, never free space.
use crate::protocol::n10::N10Packet;
use crate::{FrameId, LidarSample, Timestamp, ValidationError};
use serde::{Deserialize, Serialize};
use std::f64::consts::TAU;

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct ScanConfig {
    pub frame_id: FrameId,
    pub bins: usize,
    pub min_coverage_fraction: f64,
    pub min_valid_fraction: f64,
    pub max_missing_arc_rad: f64,
    pub max_revolution_ms: u64,
    pub max_packet_gap_ms: u64,
}

impl ScanConfig {
    pub fn validate(&self) -> Result<(), ValidationError> {
        self.frame_id.validate()?;
        if !(64..=1440).contains(&self.bins)
            || ![self.min_coverage_fraction, self.min_valid_fraction]
                .iter()
                .all(|v| v.is_finite() && *v > 0.0 && *v <= 1.0)
            || !self.max_missing_arc_rad.is_finite()
            || !(0.0..=TAU).contains(&self.max_missing_arc_rad)
            || self.max_revolution_ms == 0
            || self.max_revolution_ms > 10_000
            || self.max_packet_gap_ms == 0
            || self.max_packet_gap_ms > self.max_revolution_ms
        {
            return Err(ValidationError(
                "invalid N10 scan coverage/time limits".into(),
            ));
        }
        Ok(())
    }
}

#[derive(Clone, Debug, Serialize)]
pub struct FullScan {
    pub sample: LidarSample,
    pub completed_at: Timestamp,
    pub coverage_fraction: f64,
    pub valid_fraction: f64,
}

/// Require almost a full angular turn, finite valid ranges and bounded unknown sectors.
/// This describes usable observations; it does not infer free space from no-return rays.
pub fn validate_full_scan(
    sample: &LidarSample,
    config: &ScanConfig,
) -> Result<(), ValidationError> {
    config.validate()?;
    let n = sample.ranges_m.len();
    if sample.frame_id != config.frame_id
        || n != config.bins
        || !sample.angle_min_rad.is_finite()
        || !sample.angle_increment_rad.is_finite()
        || (sample.angle_increment_rad.abs() * n as f64 - TAU).abs() > 1e-6
        || !sample.range_min_m.is_finite()
        || sample.range_min_m < 0.0
        || !sample.range_max_m.is_finite()
        || sample.range_max_m <= sample.range_min_m
    {
        return Err(ValidationError(
            "scan must have the configured frame and one full uniform revolution".into(),
        ));
    }
    let mut known = 0;
    for range in sample.ranges_m.iter().flatten() {
        if !range.is_finite() || *range < sample.range_min_m || *range > sample.range_max_m {
            return Err(ValidationError("invalid laser range".into()));
        }
        known += 1;
    }
    let mut longest = 0;
    let mut run = 0;
    for range in sample.ranges_m.iter().cycle().take(n * 2) {
        if range.is_none() {
            run += 1;
            longest = longest.max(run.min(n));
        } else {
            run = 0;
        }
    }
    if known as f64 / (n as f64) < config.min_valid_fraction
        || longest as f64 * sample.angle_increment_rad.abs() > config.max_missing_arc_rad
    {
        return Err(ValidationError(
            "laser coverage has too many unknown returns or an excessive blind sector".into(),
        ));
    }
    Ok(())
}

pub struct N10RevolutionAssembler {
    config: ScanConfig,
    observed: Vec<bool>,
    ranges: Vec<Option<f64>>,
    first: Option<Timestamp>,
    last: Option<Timestamp>,
    previous_start: Option<u16>,
    synchronized: bool,
    range_bounds: Option<(f64, f64)>,
    pub rejected_revolutions: u64,
}

impl N10RevolutionAssembler {
    pub fn new(config: ScanConfig) -> Result<Self, ValidationError> {
        config.validate()?;
        Ok(Self {
            observed: vec![false; config.bins],
            ranges: vec![None; config.bins],
            config,
            first: None,
            last: None,
            previous_start: None,
            synchronized: false,
            range_bounds: None,
            rejected_revolutions: 0,
        })
    }

    fn clear(&mut self) {
        self.observed.fill(false);
        self.ranges.fill(None);
        self.first = None;
        self.range_bounds = None;
    }

    pub fn push(&mut self, packet: &N10Packet) -> Result<Option<FullScan>, ValidationError> {
        if packet.frame_id != self.config.frame_id
            || packet.first_received_at > packet.received_at
            || self.last.is_some_and(|at| packet.received_at < at)
        {
            return Err(ValidationError("N10 scan frame/time mismatch".into()));
        }
        if self
            .last
            .is_some_and(|at| packet.received_at.0 - at.0 > self.config.max_packet_gap_ms)
        {
            self.clear();
            self.synchronized = false;
            self.previous_start = None;
            self.rejected_revolutions += 1;
        }
        let wrap = self
            .previous_start
            .is_some_and(|old| old > 27_000 && packet.start_angle_cdeg < 9_000);
        if self
            .previous_start
            .is_some_and(|old| packet.start_angle_cdeg < old)
            && !wrap
        {
            self.clear();
            self.synchronized = false;
            self.previous_start = None;
            return Err(ValidationError(
                "N10 packet angles regressed without a revolution boundary".into(),
            ));
        }
        let completed = if wrap {
            let result = self.finish(packet.received_at);
            self.clear();
            self.synchronized = true;
            result
        } else {
            None
        };
        if self.previous_start.is_none() && packet.start_angle_cdeg == 0 {
            self.synchronized = true;
        }
        self.last = Some(packet.received_at);
        self.previous_start = Some(packet.start_angle_cdeg);
        if !self.synchronized {
            return Ok(completed);
        }
        self.first.get_or_insert(packet.first_received_at);
        // Only the existing decoder's checked angular interpolation is reused.
        if let Some(sample) = packet.packet_sample() {
            if self
                .range_bounds
                .is_some_and(|bounds| bounds != (sample.range_min_m, sample.range_max_m))
            {
                return Err(ValidationError(
                    "N10 range bounds changed within a revolution".into(),
                ));
            }
            self.range_bounds = Some((sample.range_min_m, sample.range_max_m));
            for (index, range) in sample.ranges_m.iter().enumerate() {
                let bearing = -(sample.angle_min_rad + index as f64 * sample.angle_increment_rad);
                let bin = ((bearing.rem_euclid(TAU) / TAU * self.config.bins as f64).round()
                    as usize)
                    % self.config.bins;
                self.observed[bin] = true;
                if let Some(value) = range {
                    self.ranges[bin] = Some(self.ranges[bin].map_or(*value, |old| old.min(*value)));
                }
            }
        }
        Ok(completed)
    }

    fn finish(&mut self, completed_at: Timestamp) -> Option<FullScan> {
        let first = self.first?;
        let bounds = self.range_bounds?;
        let coverage =
            self.observed.iter().filter(|seen| **seen).count() as f64 / self.config.bins as f64;
        let valid = self.ranges.iter().flatten().count() as f64 / self.config.bins as f64;
        let sample = LidarSample {
            captured_at: first,
            frame_id: self.config.frame_id.clone(),
            angle_min_rad: 0.0,
            angle_increment_rad: -TAU / self.config.bins as f64,
            range_min_m: bounds.0,
            range_max_m: bounds.1,
            ranges_m: self.ranges.clone(),
        };
        if !self.synchronized
            || completed_at.0 - first.0 > self.config.max_revolution_ms
            || coverage < self.config.min_coverage_fraction
            || validate_full_scan(&sample, &self.config).is_err()
        {
            self.rejected_revolutions += 1;
            return None;
        }
        Some(FullScan {
            sample,
            completed_at,
            coverage_fraction: coverage,
            valid_fraction: valid,
        })
    }
}
