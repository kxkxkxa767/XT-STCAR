//! The open end of the light lane's inner board, observed while passing cone one.
//! Store its distance from the end wall, not its bearing in a turning car.
use crate::helpers::{OPEN_LANE_MIN_SPAN_M, OPEN_LINE_MIN_POINTS, SIDE_BODY_EXTENT_M, dcos, dsin};
use crate::maneuver::{
    LIGHT_GANTRY_EDGE_M, LIGHT_LINE_JUMP_M, LIGHT_POST_JUMP_M, OPEN_LANE_WIDTH_M,
};
use crate::py::V;

struct ScanAxis<'a> {
    ranges: &'a [V],
    sin: f64,
    cos: f64,
    forward: usize,
}

impl<'a> ScanAxis<'a> {
    fn new(scan: &'a V, heading: f64) -> Option<Self> {
        let r = scan.get("ranges");
        if !r.is_list() || r.len() != 360 {
            return None;
        }
        let (sin, cos) = heading.sin_cos();
        Some(Self {
            ranges: r.l(),
            sin,
            cos,
            forward: (heading.to_degrees().round() as i64).rem_euclid(360) as usize,
        })
    }
    // Coordinates in the finish lane's direction of travel, relative to the lidar.
    fn point(&self, i: usize) -> Option<(f64, f64)> {
        let r = self.ranges[i].of()?;
        let (c, s) = (dcos(i), dsin(i));
        Some((
            r * (self.cos * c + self.sin * s),
            r * (self.sin * c - self.cos * s),
        ))
    }
    fn end_wall(&self) -> Option<f64> {
        // The centre ray can hit a cone or a light post. Seek a common farther
        // plane among 13 projected rays; no whole-scan fit or pose integration.
        let mut x = [0.; 13];
        let mut n = 0;
        for j in 0..13 {
            if let Some((d, _)) = self.point((self.forward + 348 + j * 2) % 360)
                && d > 0.
            {
                x[n] = d;
                n += 1;
            }
        }
        x[..n].sort_unstable_by(f64::total_cmp);
        x[..n]
            .windows(3)
            .rev()
            .find(|w| w[2] - w[0] <= LIGHT_GANTRY_EDGE_M)
            .map(|w| w[1])
    }
    fn divider_span(&self, wall: f64) -> Option<f64> {
        let mut found: Option<f64> = None;
        for i in 0..360 {
            let Some(r) = self.ranges[i].of() else {
                continue;
            };
            // Beside the upstream end the adjacent rays go *farther* into the
            // light lane; a closer cone hiding the rest of a board is not an end.
            if self.ranges[(i + 359) % 360]
                .of()
                .is_none_or(|next| next <= r + LIGHT_POST_JUMP_M)
            {
                continue;
            }
            let (x, y) = self.point(i)?;
            if y <= SIDE_BODY_EXTENT_M || wall - x <= OPEN_LANE_MIN_SPAN_M {
                continue;
            }
            let outside = |k| self.point((i + 360 - k) % 360).map(|(_, v)| v - y);
            if !(1..=2).all(|k| outside(k).is_some_and(|d| d > LIGHT_POST_JUMP_M))
                || !(1..=3).any(|k| {
                    outside(k).is_some_and(|d| {
                        (OPEN_LANE_WIDTH_M.0 - LIGHT_GANTRY_EDGE_M..=OPEN_LANE_WIDTH_M.1)
                            .contains(&d)
                    })
                })
            {
                continue;
            }
            // Follow the near face downstream, not merely the last bin of a
            // native fitted line (the fit can end in the middle of a board).
            for j in 1..180 {
                let Some((a, b)) = self.point((i + j) % 360) else {
                    break;
                };
                if (b - y).abs() > LIGHT_GANTRY_EDGE_M || a < x - LIGHT_GANTRY_EDGE_M {
                    break;
                }
                if j + 1 >= OPEN_LINE_MIN_POINTS && a - x >= OPEN_LANE_MIN_SPAN_M {
                    let span = wall - x;
                    if found.is_some_and(|other| (other - span).abs() > LIGHT_LINE_JUMP_M) {
                        return None;
                    }
                    found = Some(span);
                    break;
                }
            }
        }
        found
    }
}

#[derive(Clone, Copy, Default, Debug)]
pub enum LightRoll {
    #[default]
    Cruise,
    Coast,
    Topup,
    FinalCoast,
    Through,
}

impl LightRoll {
    pub fn label(self) -> &'static str {
        match self {
            Self::Cruise => "cruise",
            Self::Coast => "coast",
            Self::Topup => "rolling_topup",
            Self::FinalCoast => "final_coast",
            Self::Through => "unlit_continue",
        }
    }
}

#[derive(Clone, Default, Debug)]
pub struct RaceLight {
    votes: [(u32, f64); 2],
    last_seq: Option<i64>,
    pub phase: LightRoll,
    pub prior_ahead: Option<f64>,
    pub passed_hint: bool,
}
impl RaceLight {
    pub fn observe_first(&mut self, scan: &V, heading: f64) {
        let seq = scan.idx("seq").i();
        if self.last_seq == Some(seq) {
            return;
        }
        self.last_seq = Some(seq);
        let Some(axis) = ScanAxis::new(scan, heading) else {
            return;
        };
        let Some(span) = axis.end_wall().and_then(|w| axis.divider_span(w)) else {
            return;
        };
        // Keep the consistent landmark across viewpoints, not an isolated gap.
        if let Some((n, mean)) = self
            .votes
            .iter_mut()
            .filter(|(n, m)| *n > 0 && (*m - span).abs() <= LIGHT_LINE_JUMP_M)
            .min_by(|a, b| (a.1 - span).abs().total_cmp(&(b.1 - span).abs()))
        {
            *n += 1;
            *mean += (span - *mean) / f64::from(*n);
        } else if let Some(slot) = self.votes.iter_mut().min_by_key(|v| v.0) {
            *slot = (1, span);
        }
    }
    pub fn landmark(&self) -> Option<(u32, f64)> {
        let [a, b] = self.votes;
        let best = if a.0 > b.0 { a } else { b };
        (best.0 >= 2 && a.0 != b.0).then_some(best)
    }
    pub fn project(&mut self, scan: &V, heading: f64) -> Option<f64> {
        self.prior_ahead = self
            .landmark()
            .and_then(|(_, span)| Some(ScanAxis::new(scan, heading)?.end_wall()? - span));
        self.prior_ahead
    }
}
