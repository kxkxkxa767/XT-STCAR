//! Optical plane of the black lamp backplate, registered separately from the finish divider.
use crate::race_cam_features::{CAM_HEIGHT_M, FY, undistort_pixel};

/// Pixel coordinates in the calibrated 640 x 480 image. A cropped edge has no row.
#[derive(Clone, Copy, Debug, Default)]
pub struct Edges {
    pub u: f64,
    pub top: Option<f64>,
    pub bottom: Option<f64>,
}

#[derive(Clone, Copy, Debug)]
pub struct MetricRange {
    pub forward: f64,
    pub left: f64,
    pub uncertainty: f64,
    pub both: bool,
}

impl Edges {
    /// Intersect each undistorted ray with its known horizontal height plane.
    pub fn range(self, heights: [f64; 2], pitch: f64) -> Option<MetricRange> {
        let (sp, cp) = pitch.sin_cos();
        let ray = |v: f64, height: f64| {
            if !(1.0..479.0).contains(&v) {
                return None;
            }
            let (x, y) = undistort_pixel(self.u, v);
            let horizontal = cp + y * sp;
            let slope = (sp - y * cp) / horizontal;
            let forward = (height - CAM_HEIGHT_M) / slope;
            (horizontal > 0.0 && (0.18..=5.0).contains(&forward)).then_some((
                forward,
                -x / horizontal,
                slope,
            ))
        };
        let top = self.top.and_then(|v| ray(v, heights[1]));
        let bottom = self.bottom.and_then(|v| ray(v, heights[0]));
        let (forward, side, uncertainty, both) = match (top, bottom) {
            (Some((ft, lt, st)), Some((fb, lb, sb))) => {
                if self.top? >= self.bottom? || st <= sb {
                    return None;
                }
                let by_size = (heights[1] - heights[0]) / (st - sb);
                if !(0.18..=5.0).contains(&by_size) {
                    return None;
                }
                // Angular height largely cancels a common pitch error. The surveyed edge heights
                // independently reject a pole, a lens rim, or the dark wall behind the fixture.
                let h_res = (CAM_HEIGHT_M + by_size * sb - heights[0]).abs();
                if h_res > 0.035 + 0.012 * by_size {
                    return None;
                }
                let slope_err = 2.0 / FY;
                let sigma_size = slope_err * by_size * by_size / (heights[1] - heights[0]);
                let sigma_height = (slope_err + 0.010) * ft * ft / (heights[1] - CAM_HEIGHT_M);
                let size_weight =
                    sigma_height.powi(2) / (sigma_height.powi(2) + sigma_size.powi(2));
                let f = size_weight * by_size + (1.0 - size_weight) * (0.75 * ft + 0.25 * fb);
                (
                    f,
                    (lt + lb) * 0.5,
                    (sigma_size.min(sigma_height) + 0.018).max(0.025),
                    true,
                )
            }
            (Some((f, l, _)), None) if self.bottom.is_none() => (
                f,
                l,
                0.025 + f * f * 0.016 / (heights[1] - CAM_HEIGHT_M),
                false,
            ),
            (None, Some((f, l, _))) if self.top.is_none() => (
                f,
                l,
                0.025 + f * f * 0.016 / (heights[0] - CAM_HEIGHT_M),
                false,
            ),
            _ => return None,
        };
        Some(MetricRange {
            forward,
            left: forward * side,
            uncertainty,
            both,
        })
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Sighting {
    pub frame: i64,
    pub at: f64,
    pub edges: Edges,
}

#[derive(Clone, Copy, Debug)]
pub struct Plane {
    pub ahead: f64,  // signed along the finish straight at the current radar scan
    pub offset: f64, // signed displacement from the surveyed divider tip
    pub uncertainty: f64,
    pub both: bool,
}

#[derive(Clone, Copy, Debug)]
struct Vote {
    at: f64,
    ahead: f64,
    offset: f64,
    from_tip: Option<f64>,
    from_end: Option<f64>,
    uncertainty: f64,
    both: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct Registration {
    pub offset: f64,
    tip_delta: Option<f64>,
    pub end_span: Option<f64>,
    error: f64,
}

#[derive(Clone, Debug, Default)]
pub struct HousingTrack {
    pub latest: Option<Sighting>,
    last_frame: Option<i64>,
    votes: [Option<Vote>; 5],
    next: usize,
    pub registered: Option<Registration>,
    pub last_range: Option<f64>,
    pub direct: Option<(i64, f64)>,
    revision: u32,
    applied: u32,
}

fn median(mut a: [f64; 5], n: usize) -> Option<f64> {
    if n == 0 {
        return None;
    }
    a[..n].sort_unstable_by(f64::total_cmp);
    Some((a[(n - 1) / 2] + a[n / 2]) * 0.5)
}

impl HousingTrack {
    pub fn pending(&self) -> Option<Sighting> {
        self.latest.filter(|s| Some(s.frame) != self.last_frame)
    }

    pub fn changed_reference(&mut self) -> bool {
        let changed = self.applied != self.revision;
        self.applied = self.revision;
        changed
    }

    /// Only independent exposures vote. Losing the outline does not erase a fixed fixture.
    pub fn register(
        &mut self,
        shot: Sighting,
        plane: Plane,
        tip: Option<f64>,
        end_wall: Option<f64>,
    ) {
        if self.last_frame == Some(shot.frame) {
            return;
        }
        self.last_frame = Some(shot.frame);
        self.last_range = Some(plane.ahead);
        self.votes[self.next] = Some(Vote {
            at: shot.at,
            ahead: plane.ahead,
            offset: plane.offset,
            from_tip: tip.map(|x| plane.ahead - x),
            from_end: end_wall.map(|x| x - plane.ahead),
            both: plane.both,
            uncertainty: plane.uncertainty,
        });
        self.next = (self.next + 1) % self.votes.len();
        let (mut offsets, mut tips, mut spans, mut errors) = ([0.; 5], [0.; 5], [0.; 5], [0.; 5]);
        let (mut n, mut nt, mut nw, mut dual) = (0, 0, 0, 0);
        let mut travel: f64 = 0.0;
        for v in self.votes.iter().flatten() {
            if !(0.0..=1.0).contains(&(shot.at - v.at))
                || (v.offset - plane.offset).abs()
                    > (v.uncertainty + plane.uncertainty).clamp(0.06, 0.18)
            {
                continue;
            }
            travel = travel.max((v.ahead - plane.ahead).abs());
            offsets[n] = v.offset;
            errors[n] = v.uncertainty;
            n += 1;
            dual += usize::from(v.both);
            if let Some(t) = v.from_tip {
                tips[nt] = t;
                nt += 1;
            }
            if let Some(w) = v.from_end {
                spans[nw] = w;
                nw += 1;
            }
        }
        if n < 2 || (dual < 2 && n < 3) {
            return;
        }
        let Some(offset) = median(offsets, n) else {
            return;
        };
        let single_error = median(errors, n).unwrap_or(plane.uncertainty);
        // A moving approach samples independent rows and apparent sizes. Keep a
        // common calibration error floor; repeated stationary pixels do not improve it.
        let error = if dual >= 3 && travel >= 0.15 {
            let common = 0.035 + 0.012 * plane.ahead.max(0.0);
            let scatter = offsets[..n]
                .iter()
                .map(|v| (v - offset).abs())
                .fold(0.0, f64::max);
            common + scatter.max((single_error - common).max(0.0) / (dual as f64).sqrt())
        } else {
            single_error
        };
        if error > 0.18 {
            return;
        }
        // A newly visible physical reference may arrive after the best camera
        // view. Retain independently measured spans across subsequent occlusion.
        let stable = |values: [f64; 5], count: usize| {
            let m = median(values, count)?;
            (count >= 2 && values[..count].iter().all(|v| (v - m).abs() <= 0.12)).then_some(m)
        };
        let tip = stable(tips, nt);
        let end = stable(spans, nw);
        if let Some(r) = &mut self.registered {
            if (r.offset - offset).abs() > (2.0 * r.error + error).max(0.12) {
                return;
            }
            if error >= 0.70 * r.error {
                if (r.offset - offset).abs() <= 0.10 {
                    r.tip_delta = r.tip_delta.or(tip);
                    r.end_span = r.end_span.or(end);
                }
                return;
            }
        }
        self.registered = Some(Registration {
            offset,
            error,
            tip_delta: tip.or(self.registered.and_then(|r| r.tip_delta)),
            end_span: end.or(self.registered.and_then(|r| r.end_span)),
        });
        self.revision += 1;
    }

    pub fn ahead(
        &self,
        mapped: Option<f64>,
        tip: Option<f64>,
        end_wall: Option<f64>,
    ) -> Option<(f64, &'static str)> {
        let r = self.registered?;
        let map = mapped.map(|d| d + r.offset);
        let wall = end_wall.zip(r.end_span).map(|(w, span)| w - span);
        let edge = tip.zip(r.tip_delta).map(|(d, off)| d + off).filter(|d| {
            map.is_none_or(|m| (d - m).abs() <= crate::maneuver::LIGHT_LINE_JUMP_M)
                || wall.is_some_and(|w| (d - w).abs() <= crate::maneuver::LIGHT_GANTRY_EDGE_M)
        });
        edge.map(|d| (d, "housing_from_divider_tip"))
            .or_else(|| map.map(|d| (d, "housing_from_mapped_divider")))
            .or_else(|| wall.map(|d| (d, "housing_from_end_board")))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::race_cam_features::{CX, distort_norm};

    const HEIGHTS: [f64; 2] = [0.43, 0.60];

    fn edges(distance: f64, pitch: f64) -> Edges {
        let (s, c) = pitch.sin_cos();
        let project = |height: f64| {
            let z = height - CAM_HEIGHT_M;
            distort_norm(0.0, (s * distance - c * z) / (c * distance + s * z)).1
        };
        Edges {
            u: CX,
            top: Some(project(HEIGHTS[1])),
            bottom: Some(project(HEIGHTS[0])),
        }
    }

    #[test]
    fn surveyed_edges_range_in_metres_with_pitch_distortion_and_clipping() {
        for distance in [0.85, 1.3, 2.4] {
            for pitch in [0.10, 0.145, 0.19] {
                let e = edges(distance, pitch);
                for (top, bottom) in [(e.top, e.bottom), (None, e.bottom), (e.top, None)] {
                    let m = Edges {
                        u: e.u,
                        top,
                        bottom,
                    }
                    .range(HEIGHTS, pitch)
                    .unwrap();
                    assert!((m.forward - distance).abs() < 0.008);
                    assert!(m.left.abs() < 0.002);
                    assert_eq!(m.both, top.is_some() && bottom.is_some());
                }
            }
        }
        assert!(Edges::default().range(HEIGHTS, 0.14).is_none());
        let e = edges(1.2, 0.14);
        assert!(
            Edges {
                top: e.bottom,
                bottom: e.top,
                ..e
            }
            .range(HEIGHTS, 0.14)
            .is_none()
        );
        assert!(
            Edges {
                bottom: Some(e.top.unwrap() + 2.0),
                ..e
            }
            .range(HEIGHTS, 0.14)
            .is_none()
        );
    }

    #[test]
    fn fresh_captures_register_three_distinct_origins_and_survive_occlusion() {
        let mut h = HousingTrack::default();
        let plane = Plane {
            ahead: 1.72,
            offset: 0.32,
            uncertainty: 0.05,
            both: true,
        };
        let frame = Sighting {
            frame: 8,
            at: 0.5,
            edges: edges(1.5, 0.14),
        };
        h.register(frame, plane, Some(1.47), Some(4.53));
        h.register(frame, plane, Some(1.47), Some(4.53));
        assert!(h.registered.is_none());
        let later = Sighting {
            frame: 9,
            at: 0.6,
            ..frame
        };
        h.register(
            later,
            Plane {
                ahead: 1.62,
                ..plane
            },
            Some(1.37),
            Some(4.43),
        );
        assert!(h.changed_reference());
        assert!(!h.changed_reference());
        for (map, tip, end, expected) in [
            (Some(1.0), None, None, 1.32),
            (Some(1.0), Some(1.0), None, 1.25),
            (None, None, Some(3.81), 1.0),
        ] {
            assert!((h.ahead(map, tip, end).unwrap().0 - expected).abs() < 1e-8);
        }
        assert!(h.ahead(None, None, None).is_none());
        h.latest = None;
        assert!(h.registered.is_some());
    }

    #[test]
    fn one_cropped_edge_needs_three_distinct_consistent_exposures() {
        let mut h = HousingTrack::default();
        for i in 1..=3 {
            h.register(
                Sighting {
                    frame: i,
                    at: i as f64 * 0.10,
                    edges: Edges::default(),
                },
                Plane {
                    ahead: 1.2,
                    offset: 0.28,
                    uncertainty: 0.05,
                    both: false,
                },
                None,
                None,
            );
            assert_eq!(h.registered.is_some(), i == 3);
        }
    }

    #[test]
    fn displaced_black_frame_moves_wheel_clock_once_and_live_posts_get_no_second_offset() {
        use crate::maneuver::Maneuver;
        use crate::py::V;
        use crate::race_light::{OpeningSupport, RaceLight};
        let lane = crate::dict! {"outer_lateral_m"=>0.45,"inner_lateral_m"=>-0.45,"width_m"=>0.90};
        for offset in [-0.15, 0.30] {
            let mut lamp = RaceLight::default();
            for id in [1, 2] {
                lamp.housing.register(
                    Sighting {
                        frame: id,
                        at: id as f64 * 0.1,
                        edges: Edges::default(),
                    },
                    Plane {
                        ahead: 2.0 + offset,
                        offset,
                        uncertainty: 0.04,
                        both: true,
                    },
                    None,
                    None,
                );
            }
            let get =
                |lamp: &mut RaceLight, seq: i64, time: f64, mapped: f64, station: Option<f64>| {
                    let scan = crate::dict! {"seq"=>seq,"received_at"=>time};
                    let d = lamp
                        .opening_ahead(
                            &scan,
                            0.0,
                            &lane,
                            OpeningSupport {
                                mapped: Some(mapped),
                                station,
                                axis_speed: Some(1.0),
                                live_legs: station.is_some(),
                                sees_both_stations: station.is_some(),
                                ..Default::default()
                            },
                        )
                        .unwrap();
                    let g = Maneuver::wheel_gap_to_lamp(d, 0.0, &lane, &V::None);
                    lamp.observe_beam(time, g);
                    d
                };
            let before = 1.24 - offset;
            assert!((get(&mut lamp, 10, 1.0, before, None) - 1.24).abs() < 1e-8);
            assert!(lamp.beam_at.is_none());
            get(&mut lamp, 11, 1.10, before - 0.10, None);
            assert!((lamp.beam_at.unwrap() - 1.05).abs() < 1e-8);
            assert!((get(&mut lamp, 12, 1.20, before - 0.2, Some(1.04)) - 1.04).abs() < 1e-8);
            // A separate rigid support at the entrance need not share the black plane.
            assert!((get(&mut lamp, 13, 1.30, before - 0.30, Some(0.52)) - 0.94).abs() < 1e-8);
            assert!((lamp.beam_at.unwrap() - 1.05).abs() < 1e-8);
        }
    }
}
