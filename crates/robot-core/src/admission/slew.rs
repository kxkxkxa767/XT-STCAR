//! One clock contract for planning, certification and actual adoption.
use crate::Timestamp;

/// The only bootstrap is revision zero with no changed output command yet.
/// An existing revision must supply its actual change time; future times fail.
pub fn command_slew_allowance(
    rate: f64,
    at: Timestamp,
    last_change_at: Option<Timestamp>,
    revision: u64,
    bootstrap_period_ms: u64,
) -> Option<f64> {
    let elapsed_s = match (revision, last_change_at) {
        (0, None) if bootstrap_period_ms > 0 => bootstrap_period_ms as f64 / 1000.0,
        (1.., Some(last)) => at.0.checked_sub(last.0)? as f64 / 1000.0,
        _ => return None,
    };
    elapsed_allowance(rate, elapsed_s)
}

fn elapsed_allowance(rate: f64, elapsed_s: f64) -> Option<f64> {
    let allowance = rate * elapsed_s;
    (rate.is_finite()
        && rate > 0.0
        && elapsed_s.is_finite()
        && elapsed_s >= 0.0
        && allowance.is_finite())
    .then_some(allowance)
}

/// Relative seconds retain the half-millisecond middle adoption sample without
/// rounding an artificial timestamp. Origin is the real current planned_at.
pub(super) struct ForecastSlewClock {
    last_change_s: Option<f64>,
    bootstrap_period_s: f64,
}

impl ForecastSlewClock {
    pub(super) fn new(
        planned_at: Timestamp,
        last_change_at: Option<Timestamp>,
        revision: u64,
        period_ms: u64,
    ) -> Option<Self> {
        command_slew_allowance(1.0, planned_at, last_change_at, revision, period_ms)?;
        Some(Self {
            last_change_s: last_change_at.map(|last| -((planned_at.0 - last.0) as f64 / 1000.0)),
            bootstrap_period_s: period_ms as f64 / 1000.0,
        })
    }

    pub(super) fn allowance(&self, rate: f64, at_s: f64) -> Option<f64> {
        if !at_s.is_finite() || at_s < 0.0 {
            return None;
        }
        elapsed_allowance(
            rate,
            self.last_change_s
                .map_or(self.bootstrap_period_s, |last| at_s - last),
        )
    }

    pub(super) fn changed_at(&mut self, at_s: f64) -> Option<()> {
        if !at_s.is_finite() || at_s < 0.0 || self.last_change_s.is_some_and(|last| at_s < last) {
            return None;
        }
        self.last_change_s = Some(at_s);
        Some(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn real_change_clock_covers_short_long_zero_and_bootstrap_intervals() {
        for (at, last, expected) in [(660, 589, 0.284), (720, 589, 0.524), (589, 589, 0.0)] {
            let allowance =
                command_slew_allowance(4.0, Timestamp(at), Some(Timestamp(last)), 1, 100).unwrap();
            assert!((allowance - expected).abs() < 1e-14);
        }
        assert_eq!(
            command_slew_allowance(4.0, Timestamp(580), None, 0, 100),
            Some(0.4)
        );
        let plan =
            command_slew_allowance(4.0, Timestamp(660), Some(Timestamp(589)), 1, 100).unwrap();
        for adopted_at in 660..=749 {
            let actual =
                command_slew_allowance(4.0, Timestamp(adopted_at), Some(Timestamp(589)), 1, 100)
                    .unwrap();
            assert!(plan <= actual);
        }
        assert!(0.72 - 0.4 > plan + 1e-9);
        assert!((0.4 + plan - 0.684).abs() < 1e-14);
    }

    #[test]
    fn missing_future_inconsistent_and_nonfinite_slew_clocks_fail_closed() {
        for (last, revision, period) in [
            (None, 1, 100),
            (Some(Timestamp(0)), 0, 100),
            (Some(Timestamp(661)), 1, 100),
            (None, 0, 0),
        ] {
            assert!(command_slew_allowance(4.0, Timestamp(660), last, revision, period).is_none());
        }
        for rate in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert!(
                command_slew_allowance(rate, Timestamp(660), Some(Timestamp(589)), 1, 100)
                    .is_none()
            );
        }
        assert!(
            command_slew_allowance(f64::MAX, Timestamp(u64::MAX), Some(Timestamp(0)), 1, 100)
                .is_none()
        );
        assert!(
            command_slew_allowance(
                4.0,
                Timestamp(u64::MAX),
                Some(Timestamp(u64::MAX - 71)),
                1,
                100
            )
            .is_some()
        );
    }

    #[test]
    fn fractional_forecast_changes_reset_clock_without_replacing_real_origin() {
        let mut clock =
            ForecastSlewClock::new(Timestamp(660), Some(Timestamp(589)), 1, 100).unwrap();
        assert!((clock.allowance(4.0, 0.0).unwrap() - 0.284).abs() < 1e-14);
        clock.changed_at(0.0495).unwrap();
        assert!((clock.allowance(4.0, 0.1).unwrap() - 0.202).abs() < 1e-14);
        // The subsequent speed-only braking adoption is a real model change.
        clock.changed_at(0.1495).unwrap();
        assert!((clock.allowance(4.0, 0.2).unwrap() - 0.202).abs() < 1e-14);
        // Identical held targets do not call changed_at, so elapsed time grows.
        assert!((clock.allowance(4.0, 0.3).unwrap() - 0.602).abs() < 1e-14);
        assert!(clock.changed_at(0.1).is_none());
        assert!(clock.changed_at(f64::INFINITY).is_none());
        assert!(clock.allowance(4.0, f64::NAN).is_none());
    }
}
