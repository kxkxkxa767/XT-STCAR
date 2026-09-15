//! Shared planar geometry and timestamped road observations. No device access.
use crate::{FrameId, Timestamp, ValidationError};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Default, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Point2 {
    pub x_m: f64,
    pub y_m: f64,
}

impl Point2 {
    pub fn valid(self) -> bool {
        self.x_m.is_finite() && self.y_m.is_finite()
    }

    pub fn distance(self, other: Self) -> f64 {
        (self.x_m - other.x_m).hypot(self.y_m - other.y_m)
    }
}

#[derive(Clone, Copy, Debug, Default, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Pose2 {
    pub x_m: f64,
    pub y_m: f64,
    pub yaw_rad: f64,
}

impl Pose2 {
    pub fn valid(self) -> bool {
        self.point().valid() && self.yaw_rad.is_finite()
    }

    pub fn point(self) -> Point2 {
        Point2 {
            x_m: self.x_m,
            y_m: self.y_m,
        }
    }

    pub fn body_to_world(self, point: Point2) -> Point2 {
        let (s, c) = self.yaw_rad.sin_cos();
        Point2 {
            x_m: self.x_m + c * point.x_m - s * point.y_m,
            y_m: self.y_m + s * point.x_m + c * point.y_m,
        }
    }

    pub fn world_to_body(self, point: Point2) -> Point2 {
        let (s, c) = self.yaw_rad.sin_cos();
        let dx = point.x_m - self.x_m;
        let dy = point.y_m - self.y_m;
        Point2 {
            x_m: c * dx + s * dy,
            y_m: -s * dx + c * dy,
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Rect {
    pub min_x_m: f64,
    pub min_y_m: f64,
    pub max_x_m: f64,
    pub max_y_m: f64,
}

impl Rect {
    pub fn validate(&self) -> Result<(), ValidationError> {
        if [self.min_x_m, self.min_y_m, self.max_x_m, self.max_y_m]
            .iter()
            .all(|v| v.is_finite())
            && self.max_x_m > self.min_x_m
            && self.max_y_m > self.min_y_m
            && (self.max_x_m - self.min_x_m).is_finite()
            && (self.max_y_m - self.min_y_m).is_finite()
        {
            Ok(())
        } else {
            Err(ValidationError("invalid planar rectangle".into()))
        }
    }

    pub fn contains(&self, point: Point2) -> bool {
        point.valid()
            && point.x_m >= self.min_x_m
            && point.x_m <= self.max_x_m
            && point.y_m >= self.min_y_m
            && point.y_m <= self.max_y_m
    }
}

/// Allowed side of an oriented line: projection(point) <= max_projection_m.
/// With a lateral interval, only the part behind the line in that interval is
/// forbidden. This is a navigation domain constraint, not a sensor obstacle.
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct HalfPlane {
    origin: Point2,
    normal: Point2,
    max_projection_m: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    lateral_bounds_m: Option<[f64; 2]>,
}

impl HalfPlane {
    pub fn new(
        origin: Point2,
        forward_yaw_rad: f64,
        max_projection_m: f64,
    ) -> Result<Self, ValidationError> {
        if !origin.valid() || !forward_yaw_rad.is_finite() || !max_projection_m.is_finite() {
            return Err(ValidationError("invalid oriented travel boundary".into()));
        }
        let (s, c) = forward_yaw_rad.sin_cos();
        Ok(Self {
            origin,
            normal: Point2 { x_m: c, y_m: s },
            max_projection_m,
            lateral_bounds_m: None,
        })
    }

    /// The forward supporting line of the rectangle, for any approach yaw.
    pub fn at_region_front(
        origin: Point2,
        region: Rect,
        forward_yaw_rad: f64,
    ) -> Result<Self, ValidationError> {
        region.validate()?;
        let mut boundary = Self::new(origin, forward_yaw_rad, 0.0)?;
        let projections = [
            Point2 {
                x_m: region.min_x_m,
                y_m: region.min_y_m,
            },
            Point2 {
                x_m: region.min_x_m,
                y_m: region.max_y_m,
            },
            Point2 {
                x_m: region.max_x_m,
                y_m: region.min_y_m,
            },
            Point2 {
                x_m: region.max_x_m,
                y_m: region.max_y_m,
            },
        ]
        .map(|point| boundary.projection(point));
        if projections.iter().any(|value| !value.is_finite()) {
            return Err(ValidationError(
                "travel boundary projection is non-finite".into(),
            ));
        }
        boundary.max_projection_m = projections.into_iter().fold(f64::NEG_INFINITY, f64::max);
        Ok(boundary)
    }

    pub fn projection(self, point: Point2) -> f64 {
        (point.x_m - self.origin.x_m) * self.normal.x_m
            + (point.y_m - self.origin.y_m) * self.normal.y_m
    }

    pub fn max_projection_m(self) -> f64 {
        self.max_projection_m
    }

    /// Restrict the forbidden side to the rectangle's full lateral projection.
    /// The line's forward coordinate remains unchanged. For an oblique region,
    /// projecting all corners intentionally enlarges the forbidden band.
    pub fn with_lateral_region(mut self, region: Rect) -> Result<Self, ValidationError> {
        region.validate()?;
        let lateral = [
            Point2 {
                x_m: region.min_x_m,
                y_m: region.min_y_m,
            },
            Point2 {
                x_m: region.min_x_m,
                y_m: region.max_y_m,
            },
            Point2 {
                x_m: region.max_x_m,
                y_m: region.min_y_m,
            },
            Point2 {
                x_m: region.max_x_m,
                y_m: region.max_y_m,
            },
        ]
        .map(|point| self.lateral_projection(point));
        let min = lateral.into_iter().fold(f64::INFINITY, f64::min);
        let max = lateral.into_iter().fold(f64::NEG_INFINITY, f64::max);
        if lateral.iter().any(|value| !value.is_finite()) || min >= max {
            return Err(ValidationError(
                "invalid lateral travel boundary projection".into(),
            ));
        }
        self.lateral_bounds_m = Some([min, max]);
        Ok(self)
    }

    pub fn is_laterally_limited(self) -> bool {
        self.lateral_bounds_m.is_some()
    }

    fn lateral_projection(self, point: Point2) -> f64 {
        -(point.x_m - self.origin.x_m) * self.normal.y_m
            + (point.y_m - self.origin.y_m) * self.normal.x_m
    }

    /// Conservative separation margin of the whole convex hull plus padding.
    /// A single allowed half-space must contain the entire envelope. Checking
    /// each corner/end independently would miss a hull crossing the forbidden
    /// band while its vertices sit outside opposite lateral edges. Invalid
    /// input returns negative infinity so minimum-margin callers also reject it.
    pub fn signed_points_margin(self, points: &[Point2], padding_m: f64) -> f64 {
        if points.is_empty() || !padding_m.is_finite() || padding_m < 0.0 {
            return f64::NEG_INFINITY;
        }
        let mut front = f64::NEG_INFINITY;
        let mut left = f64::INFINITY;
        let mut right = f64::NEG_INFINITY;
        for &point in points {
            let projection = self.projection(point) + padding_m;
            if !point.valid() || !projection.is_finite() {
                return f64::NEG_INFINITY;
            }
            front = front.max(projection);
            if self.lateral_bounds_m.is_some() {
                let lateral = self.lateral_projection(point);
                if !lateral.is_finite()
                    || !(lateral - padding_m).is_finite()
                    || !(lateral + padding_m).is_finite()
                {
                    return f64::NEG_INFINITY;
                }
                left = left.min(lateral - padding_m);
                right = right.max(lateral + padding_m);
            }
        }
        let forward_margin = self.max_projection_m - front;
        self.lateral_bounds_m.map_or(forward_margin, |[min, max]| {
            forward_margin.max(min - right).max(left - max)
        })
    }

    pub fn contains_points(self, points: &[Point2], padding_m: f64) -> bool {
        if self.lateral_bounds_m.is_none() {
            // Retain the original per-corner arithmetic for global boundaries.
            !points.is_empty() && points.iter().all(|&p| self.contains_disc(p, padding_m))
        } else {
            self.signed_points_margin(points, padding_m) >= 0.0
        }
    }

    pub fn signed_disc_margin(self, center: Point2, radius_m: f64) -> f64 {
        if self.lateral_bounds_m.is_none() {
            self.max_projection_m - (self.projection(center) + radius_m)
        } else {
            self.signed_points_margin(&[center], radius_m)
        }
    }

    pub fn contains_disc(self, center: Point2, radius_m: f64) -> bool {
        if self.lateral_bounds_m.is_some() {
            return self.signed_disc_margin(center, radius_m) >= 0.0;
        }
        let front = self.projection(center) + radius_m;
        radius_m.is_finite()
            && radius_m >= 0.0
            && front.is_finite()
            && front <= self.max_projection_m
    }

    pub fn contains_footprint(self, footprint: Footprint, pose: Pose2, margin_m: f64) -> bool {
        pose.valid() && self.contains_points(&footprint.corners(pose), margin_m)
    }

    pub fn footprint_progress(self, footprint: Footprint, pose: Pose2) -> (f64, f64) {
        let projections = footprint
            .corners(pose)
            .map(|corner| self.projection(corner));
        if projections.iter().any(|value| !value.is_finite()) {
            return (f64::NAN, f64::NAN);
        }
        (
            projections.into_iter().fold(f64::NEG_INFINITY, f64::max),
            projections.into_iter().fold(f64::INFINITY, f64::min),
        )
    }
}

/// Conservative body envelope relative to pose origin (also covers the wheels).
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Footprint {
    pub front_m: f64,
    pub rear_m: f64,
    pub half_width_m: f64,
}

impl Footprint {
    pub fn validate(&self) -> Result<(), ValidationError> {
        if [self.front_m, self.rear_m, self.half_width_m]
            .iter()
            .all(|v| v.is_finite() && *v > 0.0 && *v <= 5.0)
        {
            Ok(())
        } else {
            Err(ValidationError(
                "footprint dimensions must be finite in (0, 5] m".into(),
            ))
        }
    }

    pub fn corners(&self, pose: Pose2) -> [Point2; 4] {
        [
            Point2 {
                x_m: self.front_m,
                y_m: self.half_width_m,
            },
            Point2 {
                x_m: self.front_m,
                y_m: -self.half_width_m,
            },
            Point2 {
                x_m: -self.rear_m,
                y_m: self.half_width_m,
            },
            Point2 {
                x_m: -self.rear_m,
                y_m: -self.half_width_m,
            },
        ]
        .map(|point| pose.body_to_world(point))
    }

    pub fn inside(&self, pose: Pose2, region: Rect) -> bool {
        pose.valid()
            && self
                .corners(pose)
                .iter()
                .all(|point| region.contains(*point))
    }
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct PoseEstimate {
    pub captured_at: Timestamp,
    pub frame_id: FrameId,
    pub pose: Pose2,
    pub speed_mps: f64,
    pub yaw_rate_radps: f64,
    pub quality: f64,
}

#[derive(Clone, Copy, Debug, Default, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum LightState {
    #[default]
    Unknown,
    Red,
    Yellow,
    Green,
    Conflicting,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct CrosswalkObservation {
    pub near_edge_m: f64,
    pub far_edge_m: f64,
    pub lateral_min_m: f64,
    pub lateral_max_m: f64,
    pub confidence: f64,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct RoadObservation {
    pub captured_at: Timestamp,
    pub frame_id: FrameId,
    pub crosswalk: Option<CrosswalkObservation>,
    pub light: LightState,
    pub light_confidence: f64,
    pub cones_body_m: Vec<Point2>,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct ObstacleDisc {
    pub center: Point2,
    pub radius_m: f64,
}

#[cfg(test)]
mod finite_boundary_tests {
    use super::*;

    #[test]
    fn finite_boundary_separates_complete_rotated_envelopes_and_capsules() {
        use std::f64::consts::{FRAC_PI_2, PI};
        for yaw in [0.0, 0.63, FRAC_PI_2, -FRAC_PI_2, PI, -0.63] {
            let frame = Pose2 {
                x_m: 2.0,
                y_m: -1.0,
                yaw_rad: yaw,
            };
            let boundary = HalfPlane::new(frame.point(), yaw, 0.2)
                .unwrap()
                .with_lateral_region(Rect {
                    min_x_m: 1.6,
                    max_x_m: 2.4,
                    min_y_m: -1.4,
                    max_y_m: -0.6,
                })
                .unwrap();
            let half_span = 0.4 * (yaw.sin().abs() + yaw.cos().abs());
            let point = |x_m, y_m| frame.body_to_world(Point2 { x_m, y_m });
            assert!(boundary.contains_disc(point(-0.5, 0.0), 0.1));
            assert!(!boundary.contains_disc(point(0.5, 0.0), 0.1));
            for side in [-1.0, 1.0] {
                assert!(boundary.contains_disc(point(0.5, side * (half_span + 0.11)), 0.1));
                assert!(!boundary.contains_disc(point(0.5, side * (half_span + 0.09)), 0.1));
            }
            let endpoints = [point(0.5, -half_span - 0.4), point(0.5, half_span + 0.4)];
            assert!(endpoints.iter().all(|&p| boundary.contains_disc(p, 0.1)));
            assert!(!boundary.contains_points(&endpoints, 0.1));
            let footprint = Footprint {
                front_m: 0.1,
                rear_m: 0.1,
                half_width_m: half_span + 0.4,
            };
            let center = point(0.6, 0.0);
            let pose = Pose2 {
                x_m: center.x_m,
                y_m: center.y_m,
                yaw_rad: yaw,
            };
            assert!(
                footprint
                    .corners(pose)
                    .into_iter()
                    .all(|p| boundary.contains_disc(p, 0.0))
            );
            assert!(!boundary.contains_footprint(footprint, pose, 0.0));
            assert!(boundary.signed_points_margin(&footprint.corners(pose), 0.0) < 0.0);
        }
    }

    #[test]
    fn global_boundary_preserves_original_arithmetic_and_serialization() {
        let footprint = Footprint {
            front_m: 0.22,
            rear_m: 0.18,
            half_width_m: 0.13,
        };
        for yaw in [0.0, 0.63, -1.3, std::f64::consts::PI] {
            let boundary = HalfPlane::new(
                Point2 {
                    x_m: 2.0,
                    y_m: -1.0,
                },
                yaw,
                0.2,
            )
            .unwrap();
            assert!(!boundary.is_laterally_limited());
            assert!(
                serde_json::to_value(boundary)
                    .unwrap()
                    .get("lateral_bounds_m")
                    .is_none()
            );
            for i in -20..=20 {
                let point = Point2 {
                    x_m: i as f64 * 0.2,
                    y_m: i as f64 * -0.31,
                };
                for radius in [0.0, 0.1, 0.3] {
                    let front = boundary.projection(point) + radius;
                    assert_eq!(
                        boundary.contains_disc(point, radius),
                        front <= boundary.max_projection_m()
                    );
                    assert_eq!(
                        boundary.signed_disc_margin(point, radius).to_bits(),
                        (boundary.max_projection_m() - front).to_bits()
                    );
                    let pose = Pose2 {
                        x_m: point.x_m,
                        y_m: point.y_m,
                        yaw_rad: yaw + 0.2,
                    };
                    assert_eq!(
                        boundary.contains_footprint(footprint, pose, radius),
                        footprint.corners(pose).into_iter().all(|corner| {
                            boundary.projection(corner) + radius <= boundary.max_projection_m()
                        })
                    );
                }
            }
        }
    }

    #[test]
    fn finite_boundary_rejects_invalid_bounds_points_and_padding() {
        let boundary = HalfPlane::new(Point2::default(), 0.0, 0.0).unwrap();
        let region = Rect {
            min_x_m: -1.0,
            max_x_m: 1.0,
            min_y_m: -1.0,
            max_y_m: 1.0,
        };
        let limited = boundary.with_lateral_region(region).unwrap();
        assert!(
            boundary
                .with_lateral_region(Rect {
                    min_y_m: f64::NAN,
                    ..region
                })
                .is_err()
        );
        assert!(!limited.contains_points(&[], 0.0));
        for padding in [f64::NAN, f64::INFINITY, -0.01] {
            assert!(!limited.contains_points(&[Point2::default()], padding));
        }
        for point in [
            Point2 {
                x_m: f64::INFINITY,
                y_m: 0.0,
            },
            Point2 {
                x_m: 0.0,
                y_m: f64::NAN,
            },
        ] {
            assert!(!limited.contains_disc(point, 0.0));
        }
    }
}
