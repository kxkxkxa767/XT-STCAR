//! Shared measurements, never stage commands. Each consumer owns its own range history.
use crate::helpers::SCAN_AGE_S;
use crate::maneuver::{LANE_MAX_CLOSING_MPS, LANE_SPEED_BASELINE, LANE_SPEED_HISTORY};
use crate::py::{V, pmax, sort_floats};

/// Latest calibrated samples, read once per race tick. The outer Option in
/// Maneuver selects the race program, independently of any stage controller.
#[derive(Clone, Copy, Debug, Default)]
pub struct RaceInputs {
    pub yaw_rate: Option<f64>,
    pub forward_integral: Option<f64>,
}

/// Measured ESC response to a return to neutral; not a phase-specific speed limit.
pub const ESC_COAST_DELAY_S: f64 = 0.2;

pub type RangeHistory = Vec<(f64, f64, f64)>;

/// Positive closing rate on one reference line (sensor time, receive time, range).
pub fn lane_range_speed(history: &mut RangeHistory, scan: &V, ahead: Option<f64>, min_rates: usize) -> Option<f64> {
    let Some(ahead) = ahead else {
        history.clear();
        return None;
    };
    let row = (scan.idx("at_ms").i() as f64 / 1000., scan.idx("received_at").f(), ahead);
    if let Some(&last) = history.last() {
        let dt = row.0 - last.0;
        let rate = if dt > 0. { Some((last.2 - ahead) / dt) } else { None };
        let dr = row.1 - last.1;
        if !(0.05 <= dt && dt <= SCAN_AGE_S + 0.05)
            || !(0.05 <= dr && dr <= SCAN_AGE_S + 0.05)
            || rate.is_none_or(|r| !(-0.4 <= r && r <= LANE_MAX_CLOSING_MPS))
        {
            history.clear();
        }
    }
    history.push(row);
    if history.len() > LANE_SPEED_HISTORY {
        let n = history.len();
        history.drain(..n - LANE_SPEED_HISTORY);
    }
    let n = history.len();
    if n < 3 || n - LANE_SPEED_BASELINE.min(n - 2) < min_rates {
        return None;
    }
    let step = LANE_SPEED_BASELINE.min(n - 2);
    let h = &*history;
    let mut rates: Vec<f64> = h.iter().zip(h[step..].iter()).map(|(a, b)| (a.2 - b.2) / (b.0 - a.0)).collect();
    sort_floats(&mut rates);
    Some(pmax(0., rates[rates.len() / 2]))
}
