//! A one-state Kalman filter of the body speed in the light approach (user, 2026-11-08: the
//! speed alone gets a Kalman filter, nothing else changes). The IMU integral predicts and
//! the lidar speed corrects.
//!
//! Noise from the y64-y68 light approaches. Ordinary innovations are about .04 m/s and
//! this filter settles at the gain the earlier fixed weight had at 10 Hz (.33), so an
//! ordinary scan behaves as before. Six of the 71 samples were .165-.353 m/s off, each
//! reversed on a following scan: one displaced range, not a speed change. Beyond three
//! sigma the measurement noise grows in proportion to the excess rather than the sample
//! being dropped, so a real and lasting difference still comes through.
use crate::helpers::SCAN_AGE_S;

const MEASUREMENT_VARIANCE: f64 = 0.033 * 0.033;
const PROCESS_VARIANCE_PER_S: f64 = 0.0018;
const GATE_SIGMA: f64 = 3.0;
const SCAN_S: f64 = 0.1;
/// The IMU carries the last corrected speed across lidar speeds missing for this long.
///
/// Calibrated simulation of the practice field, 2026-11-10: in 6-15% of light approaches the
/// leg line was displaced for a few scans while the car straightened, the lidar speed was
/// missing for 0.4 s, and the first one after it read 2.4-3.3 m/s at a true 1.1-1.3. With the
/// anchor older than one scan age the filter started afresh on that sample, the single brake
/// fired 1.8 m from the gantry and the car stood 0.3 m short of the beam. Bridged, the sample
/// meets the prediction and is weighed as the displaced range it is.
pub(crate) const BRIDGE_S: f64 = 1.0;

/// The variance after a correction once the filter has settled at the lidar's rate, so
/// that the second scan already has the settled gain, as the fixed weight did.
fn settled_variance() -> f64 {
    let step = PROCESS_VARIANCE_PER_S * SCAN_S;
    let predicted = (step + (step * step + 4. * step * MEASUREMENT_VARIANCE).sqrt()) / 2.;
    predicted * MEASUREMENT_VARIANCE / (predicted + MEASUREMENT_VARIANCE)
}

#[derive(Clone, Copy, Debug)]
pub struct SpeedUpdate {
    pub at: f64,
    pub measured: f64,
    pub predicted: Option<f64>,
    pub gain: f64,
    pub noise_scale: f64,
    pub variance: f64,
    pub filtered: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct SpeedKalman {
    anchor: Option<(f64, f64, f64)>, // scan time, body speed, IMU integral
    variance: f64,
    last: Option<SpeedUpdate>,
}

impl SpeedKalman {
    pub fn clear(&mut self) {
        self.anchor = None;
    }

    pub fn anchor(&self) -> Option<(f64, f64, f64)> {
        self.anchor
    }

    pub fn last(&self) -> Option<SpeedUpdate> {
        self.last
    }

    /// One lidar body speed at scan time `at`, with the IMU integral of the same moment.
    /// Missing lidar never calls this, so it neither advances the clock nor renews the anchor.
    pub fn update(&mut self, at: f64, integral: f64, measured: f64) {
        let prior = self.anchor.filter(|(previous, _, _)| 0. < at - previous && at - previous <= BRIDGE_S.max(SCAN_AGE_S));
        let Some((previous, body, origin)) = prior else {
            let filtered = measured.max(0.);
            self.variance = settled_variance();
            self.anchor = Some((at, filtered, integral));
            self.last = Some(SpeedUpdate { at, measured, predicted: None, gain: 1., noise_scale: 1., variance: self.variance, filtered });
            return;
        };
        let predicted = (body + integral - origin).max(0.);
        let variance = self.variance + PROCESS_VARIANCE_PER_S * (at - previous);
        let innovation = measured - predicted;
        let gate = GATE_SIGMA * (variance + MEASUREMENT_VARIANCE).sqrt();
        let noise_scale = (innovation.abs() / gate).max(1.);
        let gain = variance / (variance + MEASUREMENT_VARIANCE * noise_scale);
        let filtered = (predicted + gain * innovation).max(0.);
        self.variance = (1. - gain) * variance;
        self.anchor = Some((at, filtered, integral));
        self.last = Some(SpeedUpdate { at, measured, predicted: Some(predicted), gain, noise_scale, variance: self.variance, filtered });
    }
}
