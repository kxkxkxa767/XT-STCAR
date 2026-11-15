//! Observation filtering for race loop planning only. The supervisor, target
//! tracker and light detector continue to receive the untouched scan.

#[derive(Clone, Debug, Default)]
pub struct PlanningReturns {
    last: Option<(i64, i64, f64)>,
    previous: Vec<((f64, f64), u8)>,
    kept: Vec<Option<f64>>,
    pub deferred_bins: Vec<i64>,
}

impl PlanningReturns {
    pub fn update(&mut self, ranges: &[Option<f64>], observation: (i64, i64, f64),
                  center: (f64, f64), heading: f64, support: &[usize]) -> Vec<Option<f64>> {
        let (identity, seq, at) = observation;
        if self.last.is_some_and(|last| last.0 == identity && last.1 == seq) {
            return self.kept.clone();
        }
        let recent = self.last.is_some_and(|last| last.0 == identity && seq > last.1
            && (0.0..=crate::helpers::SCAN_AGE_S).contains(&(at - last.2)));
        let (s, c) = heading.sin_cos();
        let mut previous = Vec::new();
        self.kept = ranges.to_vec();
        self.deferred_bins.clear();
        for (i, range) in ranges.iter().enumerate() {
            let Some(r) = *range else { continue };
            let a = (i as f64).to_radians();
            let (x, y) = (r * a.cos() - center.0, -r * a.sin() - center.1);
            // A fixed object stays here as the car translates and turns around
            // the measured cone; fixed-angle sensor speckles generally do not.
            let p = (c*x - s*y, s*x + c*y);
            let neighbors = [-2, -1, 1, 2].iter().filter(|&&d| {
                ranges[(i as i64 + d).rem_euclid(360) as usize]
                    .is_some_and(|v| (v-r).abs() <= 0.08_f64.max(0.06*r))
            }).count();
            let coherent = neighbors >= 2 || support.contains(&i);
            let count = if coherent { 3 } else if recent {
                self.previous.iter().filter(|(q, _)| (p.0-q.0).hypot(p.1-q.1) <= 0.05)
                    .map(|(_, n)| (*n + 1).min(3)).max().unwrap_or(1)
            } else { 1 };
            previous.push((p, count));
            if count < 3 {
                self.kept[i] = None;
                self.deferred_bins.push(i as i64);
            }
        }
        self.previous = previous;
        self.last = Some((identity, seq, at));
        self.kept.clone()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn isolated_planning_returns_need_distinct_observations_and_do_not_change_raw_scan() {
        let mut filter = PlanningReturns::default();
        let mut raw = vec![None; 360];
        raw[20] = Some(1.0); // A thin fixed post: keep after repeated observations.
        for r in &mut raw[269..=279] { *r = Some(0.9); }
        let center = (0., 1.);
        assert_eq!(filter.update(&raw, (2, 1, 0.), center, 0., &[269,270,271])[20], None);
        assert_eq!(filter.update(&raw, (2, 1, 0.), center, 0., &[269,270,271])[20], None);
        assert_eq!(filter.update(&raw, (2, 2, 0.1), center, 0., &[269,270,271])[20], None);
        raw[215] = Some(0.393); // One transient return cannot enter the running minimum.
        let kept = filter.update(&raw, (2, 3, 0.2), center, 0., &[269,270,271]);
        assert_eq!(kept[20], Some(1.0));
        assert_eq!(kept[215], None);
        assert_eq!(raw[215], Some(0.393));
        assert_eq!(kept[275], Some(0.9));
        assert_eq!(filter.update(&raw, (3, 4, 0.3), center, 0., &[269,270,271])[20], None);
    }
}
