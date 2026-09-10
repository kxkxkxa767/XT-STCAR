//! Local polyline projection for common navigation scoring, without I/O.
//!
//! This geometry describes the cached polyline, not the planner's original
//! steering trajectory. In particular, it makes no claim about curvature.
use crate::autonomy::Point2;
use crate::tracking::MAX_PATH_POINTS;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ReferenceProjection {
    pub point: Point2,
    /// Direction of the selected original segment, in [-pi, pi].
    pub heading_rad: f64,
    /// Arc length from path[progress.saturating_sub(1)].
    pub arc_m: f64,
    pub distance_m: f64,
}

/// Borrow the incoming segment at `progress` and at most
/// `max_advance_m` of the forward polyline after that vertex. The final
/// segment is clipped to that arc-length budget, not admitted in full.
///
/// The local search never scans earlier than `progress - 1` or beyond this
/// forward budget. Equal-distance candidates choose the smaller arc length,
/// so a later branch at a crossing does not win a tie. The same progress and
/// budget must be used when subtracting two projections' arc lengths.
/// Duplicate vertices are skipped. A zero budget permits only the incoming
/// segment (or the initial point, using its next nonzero segment's heading).
/// Construction or projection returns None for invalid/excessive input,
/// degenerate local geometry, or unrepresentable arithmetic. Construction
/// validates the complete path once; subsequent queries inspect only the
/// borrowed local slice. Memory use is constant, without extra samples.
#[derive(Debug)]
pub struct PreparedReference<'a> {
    points: &'a [Point2],
    tail: &'a [Point2],
    final_admitted_m: f64,
}

impl<'a> PreparedReference<'a> {
    /// Validate once and borrow only the eligible local window. The last
    /// segment retains its original direction and a separate clipping length.
    /// Reuse this value for the current pose and all candidates in one tick.
    pub fn new(path: &'a [Point2], progress: usize, max_advance_m: f64) -> Option<Self> {
        if !(2..=MAX_PATH_POINTS).contains(&path.len())
            || progress >= path.len()
            || !max_advance_m.is_finite()
            || max_advance_m < 0.0
            || path.iter().any(|p| !p.valid())
        {
            return None;
        }
        let start = progress.saturating_sub(1);
        let mut remaining_m = max_advance_m;
        let mut arc_m = 0.0;
        let mut last = None;
        for index in start..path.len() - 1 {
            let length_m = path[index].distance(path[index + 1]);
            if !length_m.is_finite() {
                return None;
            }
            if length_m == 0.0 {
                continue;
            }
            let forward = index >= progress;
            let admitted_m = if forward {
                length_m.min(remaining_m)
            } else {
                length_m
            };
            if !(arc_m + admitted_m).is_finite() {
                return None;
            }
            last = Some((index, admitted_m));
            if forward && admitted_m >= remaining_m {
                break;
            }
            arc_m += length_m;
            if forward {
                remaining_m -= admitted_m;
            }
        }
        let (last, final_admitted_m) = last?;
        Some(Self {
            points: &path[start..=last + 1],
            tail: &path[start..],
            final_admitted_m,
        })
    }

    /// Sample forward from the same arc origin through the actual path tail.
    /// Projection remains local, but a rollout may extend past that window;
    /// its reference clamps only at the real route endpoint. No allocation.
    pub fn cursor_to_end(&self) -> ReferenceCursor<'a> {
        ReferenceCursor {
            points: self.tail,
            segment: 0,
            arc_start_m: 0.0,
            last_query_m: 0.0,
            last_location: None,
        }
    }

    /// Search only the already validated local slice, with constant memory.
    pub fn project(&self, point: Point2) -> Option<ReferenceProjection> {
        if !point.valid() {
            return None;
        }
        let mut best: Option<ReferenceProjection> = None;
        let mut arc_m = 0.0;
        for (index, pair) in self.points.windows(2).enumerate() {
            let a = pair[0];
            let b = pair[1];
            let dx = b.x_m - a.x_m;
            let dy = b.y_m - a.y_m;
            let length_m = dx.hypot(dy);
            if length_m == 0.0 {
                continue;
            }
            let admitted_m = if index == self.points.len() - 2 {
                self.final_admitted_m
            } else {
                length_m
            };
            // Unit-vector projection avoids squaring long or tiny segments.
            let ux = dx / length_m;
            let uy = dy / length_m;
            let along_m = (point.x_m - a.x_m) * ux + (point.y_m - a.y_m) * uy;
            if !along_m.is_finite() {
                return None;
            }
            let travel_m = along_m.clamp(0.0, admitted_m);
            let projected = Point2 {
                x_m: a.x_m + ux * travel_m,
                y_m: a.y_m + uy * travel_m,
            };
            let candidate = ReferenceProjection {
                point: projected,
                heading_rad: dy.atan2(dx),
                arc_m: arc_m + travel_m,
                distance_m: point.distance(projected),
            };
            if !projected.valid()
                || !candidate.arc_m.is_finite()
                || !candidate.distance_m.is_finite()
            {
                return None;
            }
            if best.is_none_or(|old| {
                candidate.distance_m < old.distance_m
                    || (candidate.distance_m == old.distance_m && candidate.arc_m < old.arc_m)
            }) {
                best = Some(candidate);
            }
            arc_m += admitted_m;
        }
        best
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ReferenceLocation {
    pub point: Point2,
    pub heading_rad: f64,
}

/// Forward-only arc lookup along the actual path tail. No planner curvature
/// is fabricated, and each segment is visited at most once per cursor.
pub struct ReferenceCursor<'a> {
    points: &'a [Point2],
    segment: usize,
    arc_start_m: f64,
    last_query_m: f64,
    last_location: Option<ReferenceLocation>,
}

impl ReferenceCursor<'_> {
    pub fn at_arc(&mut self, arc_m: f64) -> Option<ReferenceLocation> {
        if !arc_m.is_finite() || arc_m < self.last_query_m {
            return None;
        }
        self.last_query_m = arc_m;
        loop {
            let a = self.points[self.segment];
            let b = self.points[self.segment + 1];
            let length = a.distance(b);
            if !length.is_finite() || !(self.arc_start_m + length).is_finite() {
                return None;
            }
            let final_segment = self.segment == self.points.len() - 2;
            if length > 0.0 {
                let heading_rad = (b.y_m - a.y_m).atan2(b.x_m - a.x_m);
                self.last_location = Some(ReferenceLocation {
                    point: b,
                    heading_rad,
                });
                if arc_m <= self.arc_start_m + length || final_segment {
                    let fraction = (arc_m - self.arc_start_m).clamp(0.0, length) / length;
                    let point = Point2 {
                        x_m: a.x_m + (b.x_m - a.x_m) * fraction,
                        y_m: a.y_m + (b.y_m - a.y_m) * fraction,
                    };
                    return point
                        .valid()
                        .then_some(ReferenceLocation { point, heading_rad });
                }
            }
            if final_segment {
                return self.last_location;
            }
            self.arc_start_m += length;
            self.segment += 1;
        }
    }
}

/// Convenience wrapper for a single query. Reuse `PreparedReference` when
/// projecting multiple candidates against the same route progress and horizon.
pub fn project_reference(
    path: &[Point2],
    progress: usize,
    point: Point2,
    max_advance_m: f64,
) -> Option<ReferenceProjection> {
    PreparedReference::new(path, progress, max_advance_m)?.project(point)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(x_m: f64, y_m: f64) -> Point2 {
        Point2 { x_m, y_m }
    }

    fn near(actual: f64, expected: f64) {
        assert!(
            (actual - expected).abs() < 1e-10,
            "actual {actual}, expected {expected}"
        );
    }

    #[test]
    fn arc_cursor_follows_turns_past_projection_window_and_stops_at_real_endpoint() {
        let path = [
            p(0.0, 0.0),
            p(1.0, 0.0),
            p(1.0, 0.0),
            p(1.0, 1.0),
            p(0.0, 1.0),
        ];
        let reference = PreparedReference::new(&path, 0, 1.5).unwrap();
        let mut cursor = reference.cursor_to_end();
        assert_eq!(cursor.at_arc(0.5).unwrap().point, p(0.5, 0.0));
        let turn = cursor.at_arc(1.25).unwrap();
        assert_eq!(turn.point, p(1.0, 0.25));
        near(turn.heading_rad, std::f64::consts::FRAC_PI_2);
        assert_eq!(cursor.at_arc(3.0).unwrap().point, p(0.0, 1.0));
        assert_eq!(cursor.at_arc(4.0).unwrap().point, p(0.0, 1.0));
        assert!(cursor.at_arc(2.0).is_none());
        assert!(cursor.at_arc(f64::NAN).is_none());
        assert!(reference.cursor_to_end().at_arc(-1.0).is_none());
    }

    #[test]
    fn straight_projection_has_consistent_local_arc_and_distance() {
        let path = [p(-1.0, 0.0), p(0.0, 0.0), p(2.0, 0.0)];
        let current = project_reference(&path, 1, p(0.25, 0.2), 1.5).unwrap();
        let predicted = project_reference(&path, 1, p(0.75, -0.1), 1.5).unwrap();
        assert_eq!(current.point, p(0.25, 0.0));
        near(current.heading_rad, 0.0);
        near(current.arc_m, 1.25);
        near(current.distance_m, 0.2);
        near(predicted.arc_m - current.arc_m, 0.5);
    }

    #[test]
    fn prepared_reference_borrows_only_the_clipped_local_window() {
        let path = [
            p(-1.0, 0.0),
            p(0.0, 0.0),
            p(10.0, 0.0),
            p(10.0, 10.0),
            p(0.0, 10.0),
        ];
        let reference = PreparedReference::new(&path, 1, 0.4).unwrap();
        // The incoming segment and clipped forward segment are borrowed once;
        // the unrelated tail cannot be scanned by any subsequent query.
        assert_eq!(reference.points, &path[..3]);
        for (query_x, expected_x) in [(-2.0, -1.0), (-0.5, -0.5), (0.2, 0.2), (8.0, 0.4)] {
            let projected = reference.project(p(query_x, 0.1)).unwrap();
            near(projected.point.x_m, expected_x);
            near(projected.arc_m, expected_x + 1.0);
        }
        assert!(reference.project(p(f64::NAN, 0.0)).is_none());
        let mut invalid_tail = path;
        invalid_tail[4].x_m = f64::INFINITY;
        assert!(PreparedReference::new(&invalid_tail, 1, 0.4).is_none());
    }

    #[test]
    fn segment_heading_is_well_defined_across_angle_wrap() {
        for sign in [-1.0, 1.0] {
            let path = [p(0.0, 0.0), p(-1.0, sign * 1e-6)];
            let result = project_reference(&path, 0, p(-0.5, 0.0), 2.0).unwrap();
            near(result.heading_rad, (sign * 1e-6_f64).atan2(-1.0));
            let delta = result.heading_rad - sign * std::f64::consts::PI;
            assert!(delta.sin().atan2(delta.cos()).abs() < 2e-6);
        }
    }

    #[test]
    fn repeated_vertices_do_not_consume_budget_or_create_nan() {
        let path = [p(0.0, 0.0), p(0.0, 0.0), p(1.0, 0.0), p(1.0, 0.0)];
        let result = project_reference(&path, 1, p(0.5, 0.2), 0.75).unwrap();
        near(result.arc_m, 0.5);
        near(result.distance_m, 0.2);
        near(result.heading_rad, 0.0);
        assert!(project_reference(&[p(0.0, 0.0); 3], 1, p(0.0, 0.0), 1.0).is_none());
    }

    #[test]
    fn crossing_does_not_select_a_distant_forward_branch() {
        let path = [
            p(-1.0, 0.0),
            p(1.0, 0.0),
            p(1.0, 2.0),
            p(0.0, 2.0),
            p(0.0, -1.0),
        ];
        // The later vertical branch contains the query exactly but lies well
        // beyond the supplied local arc budget.
        let result = project_reference(&path, 0, p(0.0, 0.1), 2.0).unwrap();
        assert_eq!(result.point, p(0.0, 0.0));
        near(result.distance_m, 0.1);
        near(result.heading_rad, 0.0);
        // If both crossings are in the requested window, equal distance still
        // chooses the earliest arc, never the later exact crossing.
        let tie = project_reference(&path, 0, p(0.0, 0.0), 20.0).unwrap();
        near(tie.arc_m, 1.0);
        near(tie.heading_rad, 0.0);
    }

    #[test]
    fn progress_and_advance_clip_the_eligible_segments() {
        let path = [p(-2.0, 0.0), p(-1.0, 0.0), p(0.0, 0.0), p(10.0, 0.0)];
        let behind = project_reference(&path, 2, p(-1.5, 0.0), 0.4).unwrap();
        assert_eq!(behind.point, p(-1.0, 0.0));
        near(behind.arc_m, 0.0);
        let ahead = project_reference(&path, 2, p(8.0, 0.0), 0.4).unwrap();
        near(ahead.point.x_m, 0.4);
        near(ahead.arc_m, 1.4);
        let zero = project_reference(&path, 2, p(8.0, 0.0), 0.0).unwrap();
        assert_eq!(zero.point, p(0.0, 0.0));
        let final_vertex = project_reference(&path, 3, p(8.0, 0.0), 0.4).unwrap();
        near(final_vertex.arc_m, 8.0);
    }

    #[test]
    fn rejects_invalid_unbounded_and_unrepresentable_inputs() {
        let path = [p(0.0, 0.0), p(1.0, 0.0)];
        assert!(project_reference(&[], 0, p(0.0, 0.0), 1.0).is_none());
        assert!(project_reference(&path, 2, p(0.0, 0.0), 1.0).is_none());
        assert!(project_reference(&path, 0, p(f64::NAN, 0.0), 1.0).is_none());
        for budget in [-1.0, f64::NAN, f64::INFINITY] {
            assert!(project_reference(&path, 0, p(0.0, 0.0), budget).is_none());
        }
        let excessive = vec![p(0.0, 0.0); MAX_PATH_POINTS + 1];
        assert!(project_reference(&excessive, 0, p(0.0, 0.0), 1.0).is_none());
        let overflow = [p(-f64::MAX, 0.0), p(f64::MAX, 0.0)];
        assert!(project_reference(&overflow, 0, p(0.0, 0.0), 1.0).is_none());
        let invalid = [p(0.0, 0.0), p(f64::INFINITY, 0.0)];
        assert!(project_reference(&invalid, 0, p(0.0, 0.0), 1.0).is_none());
    }
}
