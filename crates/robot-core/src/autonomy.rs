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
/// This is a navigation domain constraint, not a synthetic sensor obstacle.
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct HalfPlane {
    origin: Point2,
    normal: Point2,
    max_projection_m: f64,
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

    pub fn contains_disc(self, center: Point2, radius_m: f64) -> bool {
        let front = self.projection(center) + radius_m;
        radius_m.is_finite()
            && radius_m >= 0.0
            && front.is_finite()
            && front <= self.max_projection_m
    }

    pub fn contains_footprint(self, footprint: Footprint, pose: Pose2, margin_m: f64) -> bool {
        pose.valid()
            && footprint
                .corners(pose)
                .into_iter()
                .all(|corner| self.contains_disc(corner, margin_m))
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
