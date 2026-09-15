//! Meter-based construction of the draft competition field (rules figures 1/5).
//!
//! Coordinates are local: origin at the bottom-left, +x right, +y up. The route
//! runs right along the bottom, outside the right cone, back left outside the
//! left cone, then right along the top. Measurements describe a particular field;
//! they are not a scale factor for vehicle dimensions or controller parameters.
//! Passing this module's geometric checks is NOT a dynamic feasibility proof.
use crate::ValidationError;
use crate::autonomy::{Footprint, ObstacleDisc, Point2, Pose2, Rect};
use serde::{Deserialize, Serialize};

pub const START_FINISH_LENGTH_M: f64 = 0.8;
pub const CONE_BASE_SIDE_M: f64 = 0.28;
pub const CROSSWALK_DEPTH_M: f64 = 0.297;
pub const CROSSWALK_STRIPE_WIDTH_M: f64 = 0.105;
pub const CROSSWALK_GAP_M: f64 = 0.105;

/// Provenance only. `Measured` does not authorize hardware execution or certify
/// the vehicle calibration, localization, timing, or measured braking envelope.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum FieldSource {
    Simulation,
    Measured,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct FieldSpec {
    pub schema_version: u32,
    pub source: FieldSource,
    /// Official draft figure 5: 5..=8 m and 4..=6 m respectively.
    pub length_m: f64,
    pub width_m: f64,
    /// Official draft figure 5: each 1..=2 m, independently measured.
    pub bottom_lane_width_m: f64,
    pub top_lane_width_m: f64,
    /// Figure 5 bottom arrow: left boundary to the end of the crossing zone.
    pub bottom_straight_span_m: f64,
    /// Figure 5 top arrow: beginning of the light zone to the right boundary.
    pub top_straight_span_m: f64,
    /// Figure 5: each cone's distance from its corresponding side is 1..=3 m.
    pub left_cone_from_left_m: f64,
    pub right_cone_from_right_m: f64,
    /// Figure 5: 2..=3 m. The right cone's vertical position is NOT specified
    /// by a separate numerical range and must be measured/provided explicitly.
    pub left_cone_from_top_m: f64,
    pub right_cone_from_bottom_m: f64,
    /// Measured beginning of the crossing recognition zone. The draft does not
    /// assign this coordinate a separate official range.
    pub crosswalk_search_start_m: f64,
    /// Measured light plane x within the top recognition zone. The stop area
    /// ends at this plane; it is never inferred from the light color.
    pub light_front_x_m: f64,
    /// Measured longitudinal extent of the designated stop region. The draft
    /// places the photoelectric trigger 1 m before the light and calls the area
    /// between them the stop region; the example uses 1 m. Keep the actual venue
    /// marking explicit rather than silently scaling/inventing its coordinates.
    pub light_stop_length_m: f64,
}

/// Engineering inputs, not additional official competition dimensions.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct FieldPlanning {
    pub footprint: Footprint,
    pub clearance_m: f64,
    pub max_curvature_per_m: f64,
    /// Original navigation grid cell width, used to keep reference turn gates
    /// away from the conservative red-zone corner (not to resize that grid).
    pub grid_resolution_m: f64,
    /// Preferred radius of route gates around each cone. It must pass the
    /// geometry checks; it does not change the controller or its search budget.
    pub cone_route_radius_m: f64,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct FieldLayout {
    pub source: FieldSource,
    pub bounds: Rect,
    pub start_region: Rect,
    pub initial_pose: Pose2,
    pub crosswalk_search_region: Rect,
    pub crosswalk_search_end: Point2,
    /// Fixed order: right cone, then left cone; radius encloses the square base.
    pub cones: [ObstacleDisc; 2],
    /// Selected radii (right, left), bounded by measured outer-side room.
    pub selected_cone_route_radii_m: [f64; 2],
    /// Five half-circle gates per cone, then an oriented upper straight entry.
    pub cone_waypoints: Vec<Point2>,
    /// Optional tangent required at each gate. The upper straight entry must
    /// already face +x; reaching its position diagonally is insufficient.
    pub cone_waypoint_headings_rad: Vec<Option<f64>>,
    pub light_detection_region: Rect,
    /// The complete upper lane scopes the red-light plane. Cones may lie to the
    /// right of the light's x coordinate while being outside this lane.
    pub light_boundary_region: Rect,
    pub light_stop_region: Rect,
    pub light_stop_goal: Point2,
    pub light_approach_yaw_rad: f64,
    pub finish_region: Rect,
    pub finish_goal: Point2,
    pub finish_yaw_rad: f64,
}

impl FieldSpec {
    /// One explicit synthetic layout. These interior choices are not official
    /// defaults or measurements of the user's future competition venue.
    pub fn simulation_example() -> Self {
        Self {
            schema_version: 1,
            source: FieldSource::Simulation,
            length_m: 7.0,
            width_m: 5.0,
            bottom_lane_width_m: 1.2,
            top_lane_width_m: 1.2,
            bottom_straight_span_m: 4.5,
            top_straight_span_m: 4.5,
            left_cone_from_left_m: 1.5,
            right_cone_from_right_m: 1.5,
            left_cone_from_top_m: 2.5,
            right_cone_from_bottom_m: 2.5,
            crosswalk_search_start_m: 1.0,
            light_front_x_m: 6.2,
            light_stop_length_m: 1.0,
        }
    }

    pub fn layout(&self, planning: FieldPlanning) -> Result<FieldLayout, ValidationError> {
        self.validate_measurements()?;
        planning.footprint.validate()?;
        if !planning.clearance_m.is_finite()
            || planning.clearance_m < 0.0
            || !planning.max_curvature_per_m.is_finite()
            || planning.max_curvature_per_m <= 0.0
            || !planning.grid_resolution_m.is_finite()
            || !(0.025..=1.0).contains(&planning.grid_resolution_m)
            || !planning.cone_route_radius_m.is_finite()
            || planning.cone_route_radius_m < 1.0 / planning.max_curvature_per_m
        {
            return Err(invalid("invalid engineering clearance or route radius"));
        }
        let bounds = rect(0.0, 0.0, self.length_m, self.width_m);
        let bottom_y = self.bottom_lane_width_m / 2.0;
        let top_y = self.width_m - self.top_lane_width_m / 2.0;
        let top_start_x = self.length_m - self.top_straight_span_m;
        let start_region = rect(0.0, 0.0, START_FINISH_LENGTH_M, self.bottom_lane_width_m);
        let finish_region = rect(
            self.length_m - START_FINISH_LENGTH_M,
            self.width_m - self.top_lane_width_m,
            self.length_m,
            self.width_m,
        );
        let crosswalk_search_region = rect(
            self.crosswalk_search_start_m,
            0.0,
            self.bottom_straight_span_m,
            self.bottom_lane_width_m,
        );
        let light_detection_region = rect(
            top_start_x,
            self.width_m - self.top_lane_width_m,
            self.light_front_x_m,
            self.width_m,
        );
        let light_stop_region = rect(
            self.light_front_x_m - self.light_stop_length_m,
            self.width_m - self.top_lane_width_m,
            self.light_front_x_m,
            self.width_m,
        );
        let center_pose = |region: Rect| Pose2 {
            // Center the complete asymmetric footprint in a fixed-size region.
            x_m: (region.min_x_m + region.max_x_m + planning.footprint.rear_m
                - planning.footprint.front_m)
                / 2.0,
            y_m: (region.min_y_m + region.max_y_m) / 2.0,
            yaw_rad: 0.0,
        };
        // Place the unchanged body toward the front of the fixed 80 cm start
        // zone. Centering it at x=.38 in the example leaves its quantized grid
        // cell too close to the rear wall, even though the actual body fits.
        // This is an engineering placement policy, not a smaller safety margin.
        let initial_pose = Pose2 {
            x_m: start_region.max_x_m - planning.footprint.front_m - 2.0 * planning.clearance_m,
            ..center_pose(start_region)
        };
        let light_stop_goal = center_pose(light_stop_region).point();
        let finish_goal = center_pose(finish_region).point();
        let cone_radius_m = CONE_BASE_SIDE_M / 2.0 * std::f64::consts::SQRT_2;
        let cones = [
            ObstacleDisc {
                center: point(
                    self.length_m - self.right_cone_from_right_m,
                    self.right_cone_from_bottom_m,
                ),
                radius_m: cone_radius_m,
            },
            ObstacleDisc {
                center: point(
                    self.left_cone_from_left_m,
                    self.width_m - self.left_cone_from_top_m,
                ),
                radius_m: cone_radius_m,
            },
        ];
        let [right, left] = cones.map(|cone| cone.center);
        // Balance geometric clearance to the cone base and side wall when the
        // requested radius would hug the wall. This only moves reference gates;
        // it never scales the body, clearance or kinematic limits.
        let mut selected_cone_route_radii_m = [
            planning
                .cone_route_radius_m
                .min((self.right_cone_from_right_m + cone_radius_m) / 2.0),
            planning
                .cone_route_radius_m
                .min((self.left_cone_from_left_m + cone_radius_m) / 2.0),
        ];
        // The lamp forbids only the upper strip beyond its front plane. The
        // existing Grid uses a circumscribed body plus half-cell diagonal and
        // a common separating side at this non-convex corner. Keep the entire
        // reference half-circle outside that same expanded forbidden quadrant,
        // with the original clearance reserved for tracking drift. Runtime
        // motion/braking certification still controls actual speed and path.
        let grid_padding = planning
            .footprint
            .front_m
            .max(planning.footprint.rear_m)
            .hypot(planning.footprint.half_width_m)
            + planning.clearance_m
            + planning.grid_resolution_m * std::f64::consts::FRAC_1_SQRT_2;
        let forbidden_corner = point(
            self.light_front_x_m - grid_padding,
            self.width_m - self.top_lane_width_m - grid_padding,
        );
        for (radius, center) in selected_cone_route_radii_m.iter_mut().zip([right, left]) {
            let to_forbidden = (forbidden_corner.x_m - center.x_m)
                .max(0.0)
                .hypot((forbidden_corner.y_m - center.y_m).max(0.0));
            *radius = radius.min(to_forbidden - planning.clearance_m);
        }
        if selected_cone_route_radii_m
            .iter()
            .any(|r| *r < 1.0 / planning.max_curvature_per_m)
        {
            return Err(invalid(
                "cone corridor or light corner cannot fit the unchanged minimum turn radius",
            ));
        }
        let mut cone_waypoints = Vec::with_capacity(11);
        for (center, radius, direction) in [
            (right, selected_cone_route_radii_m[0], 1.0),
            (left, selected_cone_route_radii_m[1], -1.0),
        ] {
            for i in 0..=4 {
                let angle = -std::f64::consts::FRAC_PI_2
                    + direction * i as f64 * std::f64::consts::FRAC_PI_4;
                cone_waypoints.push(point(
                    center.x_m + radius * angle.cos(),
                    center.y_m + radius * angle.sin(),
                ));
            }
        }
        // Reserve longitudinal room to turn from the last cone into the upper
        // lane. A recognition-region edge alone is not a kinematic entry pose.
        cone_waypoints.push(point(
            top_start_x.max(left.x_m + 2.0 * planning.cone_route_radius_m),
            top_y,
        ));
        let mut cone_waypoint_headings_rad: Vec<_> = (0..=4)
            .map(|i| Some(i as f64 * std::f64::consts::FRAC_PI_4))
            .chain(
                (0..=4)
                    .map(|i| Some(std::f64::consts::PI - i as f64 * std::f64::consts::FRAC_PI_4)),
            )
            .collect();
        cone_waypoint_headings_rad.push(Some(0.0));
        let layout = FieldLayout {
            source: self.source,
            bounds,
            start_region,
            initial_pose,
            crosswalk_search_region,
            crosswalk_search_end: point(self.bottom_straight_span_m, bottom_y),
            cones,
            selected_cone_route_radii_m,
            cone_waypoint_headings_rad,
            cone_waypoints,
            light_detection_region,
            light_boundary_region: rect(
                0.0,
                self.width_m - self.top_lane_width_m,
                self.length_m,
                self.width_m,
            ),
            light_stop_region,
            light_stop_goal,
            light_approach_yaw_rad: 0.0,
            finish_region,
            finish_goal,
            finish_yaw_rad: 0.0,
        };
        layout.validate_geometry(planning)?;
        Ok(layout)
    }

    fn validate_measurements(&self) -> Result<(), ValidationError> {
        if self.schema_version != 1 {
            return Err(invalid("unsupported field schema"));
        }
        for (name, value, min, max) in [
            ("length_m", self.length_m, 5.0, 8.0),
            ("width_m", self.width_m, 4.0, 6.0),
            ("bottom_lane_width_m", self.bottom_lane_width_m, 1.0, 2.0),
            ("top_lane_width_m", self.top_lane_width_m, 1.0, 2.0),
            (
                "bottom_straight_span_m",
                self.bottom_straight_span_m,
                3.0,
                6.0,
            ),
            ("top_straight_span_m", self.top_straight_span_m, 3.0, 6.0),
            (
                "left_cone_from_left_m",
                self.left_cone_from_left_m,
                1.0,
                3.0,
            ),
            (
                "right_cone_from_right_m",
                self.right_cone_from_right_m,
                1.0,
                3.0,
            ),
            ("left_cone_from_top_m", self.left_cone_from_top_m, 2.0, 3.0),
        ] {
            if !value.is_finite() || !(min..=max).contains(&value) {
                return Err(invalid(&format!(
                    "{name} outside draft rule range [{min}, {max}] m"
                )));
            }
        }
        if ![
            self.right_cone_from_bottom_m,
            self.crosswalk_search_start_m,
            self.light_front_x_m,
            self.light_stop_length_m,
        ]
        .iter()
        .all(|value| value.is_finite())
            || self.bottom_straight_span_m > self.length_m
            || self.top_straight_span_m > self.length_m
            || self.bottom_lane_width_m + self.top_lane_width_m >= self.width_m
            || self.crosswalk_search_start_m < START_FINISH_LENGTH_M
            || self.crosswalk_search_start_m + CROSSWALK_DEPTH_M > self.bottom_straight_span_m
            || self.light_front_x_m > self.length_m - START_FINISH_LENGTH_M
            || self.light_stop_length_m <= 0.0
            || self.light_front_x_m - self.light_stop_length_m
                < self.length_m - self.top_straight_span_m
            || !(0.0..self.width_m).contains(&self.right_cone_from_bottom_m)
        {
            return Err(invalid(
                "field measurements conflict with fixed zones or outer bounds",
            ));
        }
        Ok(())
    }
}

impl FieldLayout {
    /// A synthetic or measured zebra placement inside the recognition region.
    /// White strips and gaps retain their exact rule dimensions. The number of
    /// strips is the largest odd count fitting the width, so one is centered.
    pub fn crosswalk_at(&self, near_x_m: f64) -> Result<Rect, ValidationError> {
        let region = self.crosswalk_search_region;
        let lane_width = region.max_y_m - region.min_y_m;
        let count = ((lane_width + CROSSWALK_GAP_M) / (CROSSWALK_STRIPE_WIDTH_M + CROSSWALK_GAP_M))
            .floor() as usize;
        let count = if count.is_multiple_of(2) {
            count.saturating_sub(1)
        } else {
            count
        };
        if !(5..=19).contains(&count) || !near_x_m.is_finite() {
            return Err(invalid("crosswalk cannot fit the fixed paper strips"));
        }
        let span = count as f64 * CROSSWALK_STRIPE_WIDTH_M + (count - 1) as f64 * CROSSWALK_GAP_M;
        let center_y = (region.min_y_m + region.max_y_m) / 2.0;
        let crossing = rect(
            near_x_m,
            center_y - span / 2.0,
            near_x_m + CROSSWALK_DEPTH_M,
            center_y + span / 2.0,
        );
        if !contains_rect(region, crossing) {
            return Err(invalid(
                "crosswalk placement exceeds its measured recognition region",
            ));
        }
        Ok(crossing)
    }

    fn validate_geometry(&self, planning: FieldPlanning) -> Result<(), ValidationError> {
        for region in [
            self.start_region,
            self.crosswalk_search_region,
            self.light_detection_region,
            self.light_stop_region,
            self.finish_region,
        ] {
            region.validate()?;
            if !contains_rect(self.bounds, region) {
                return Err(invalid("generated zone lies outside field"));
            }
        }
        for (pose, region) in [
            (self.initial_pose, self.start_region),
            (
                Pose2 {
                    x_m: self.light_stop_goal.x_m,
                    y_m: self.light_stop_goal.y_m,
                    yaw_rad: 0.0,
                },
                self.light_stop_region,
            ),
            (
                Pose2 {
                    x_m: self.finish_goal.x_m,
                    y_m: self.finish_goal.y_m,
                    yaw_rad: 0.0,
                },
                self.finish_region,
            ),
        ] {
            let inner = rect(
                region.min_x_m + planning.clearance_m,
                region.min_y_m + planning.clearance_m,
                region.max_x_m - planning.clearance_m,
                region.max_y_m - planning.clearance_m,
            );
            if inner.validate().is_err() || !planning.footprint.inside(pose, inner) {
                return Err(invalid(
                    "fixed start/light/finish zone cannot contain unchanged vehicle and margin",
                ));
            }
        }
        let [right, left] = self.cones;
        if left.center.x_m >= right.center.x_m
            || self.crosswalk_search_end.x_m >= right.center.x_m
            || self.cone_waypoints.last().expect("generated entry").x_m <= left.center.x_m
            || self.cone_waypoints.last().expect("generated entry").x_m >= self.light_stop_goal.x_m
        {
            return Err(invalid(
                "cone order or straight entries conflict with the required route topology",
            ));
        }
        for cone in self.cones {
            for region in [
                self.start_region,
                self.crosswalk_search_region,
                self.light_detection_region,
                self.finish_region,
            ] {
                if point_rect_distance(cone.center, region) <= cone.radius_m + planning.clearance_m
                {
                    return Err(invalid(
                        "cone base intersects a straight recognition/start/finish zone",
                    ));
                }
            }
        }
        // A conservative reference polygon check, not a steering/acceleration
        // certificate: runtime Navigator and final admission remain mandatory.
        let body_radius = planning
            .footprint
            .front_m
            .max(planning.footprint.rear_m)
            .hypot(planning.footprint.half_width_m)
            + planning.clearance_m;
        let safe_bounds = rect(
            self.bounds.min_x_m + body_radius,
            self.bounds.min_y_m + body_radius,
            self.bounds.max_x_m - body_radius,
            self.bounds.max_y_m - body_radius,
        );
        let mut previous = self.crosswalk_search_end;
        for current in self
            .cone_waypoints
            .iter()
            .copied()
            .chain([self.light_stop_goal])
        {
            if !safe_bounds.contains(previous) || !safe_bounds.contains(current) {
                return Err(invalid(
                    "route gate leaves insufficient full-body boundary clearance",
                ));
            }
            if previous.distance(current) <= 1e-6
                || self.cones.iter().any(|cone| {
                    point_segment_distance(cone.center, previous, current)
                        <= body_radius + cone.radius_m
                })
            {
                return Err(invalid(
                    "reference route gates collide or do not clear both cone bases",
                ));
            }
            previous = current;
        }
        self.crosswalk_at(self.crosswalk_search_region.min_x_m)?;
        Ok(())
    }
}

fn point(x_m: f64, y_m: f64) -> Point2 {
    Point2 { x_m, y_m }
}
fn rect(min_x_m: f64, min_y_m: f64, max_x_m: f64, max_y_m: f64) -> Rect {
    Rect {
        min_x_m,
        min_y_m,
        max_x_m,
        max_y_m,
    }
}
fn invalid(message: &str) -> ValidationError {
    ValidationError(format!("field: {message}"))
}
fn contains_rect(outer: Rect, inner: Rect) -> bool {
    outer.contains(point(inner.min_x_m, inner.min_y_m))
        && outer.contains(point(inner.max_x_m, inner.max_y_m))
}
fn point_rect_distance(p: Point2, region: Rect) -> f64 {
    (p.x_m - p.x_m.clamp(region.min_x_m, region.max_x_m))
        .hypot(p.y_m - p.y_m.clamp(region.min_y_m, region.max_y_m))
}
fn point_segment_distance(p: Point2, a: Point2, b: Point2) -> f64 {
    let dx = b.x_m - a.x_m;
    let dy = b.y_m - a.y_m;
    let length_squared = dx * dx + dy * dy;
    if length_squared <= f64::EPSILON {
        return p.distance(a);
    }
    let t = (((p.x_m - a.x_m) * dx + (p.y_m - a.y_m) * dy) / length_squared).clamp(0.0, 1.0);
    p.distance(point(a.x_m + t * dx, a.y_m + t * dy))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn planning() -> FieldPlanning {
        FieldPlanning {
            footprint: Footprint {
                front_m: 0.22,
                rear_m: 0.18,
                half_width_m: 0.13,
            },
            clearance_m: 0.04,
            max_curvature_per_m: 2.0,
            grid_resolution_m: 0.1,
            cone_route_radius_m: 0.85,
        }
    }

    #[test]
    fn official_topology_and_fixed_regions_are_not_scaled() {
        let spec = FieldSpec::simulation_example();
        let layout = spec.layout(planning()).unwrap();
        assert_eq!(layout.bounds.max_x_m, 7.0);
        assert_eq!(layout.cone_waypoints.len(), 11);
        let [right, left] = layout.cones;
        assert!(layout.cone_waypoints[2].x_m > right.center.x_m);
        assert!(layout.cone_waypoints[7].x_m < left.center.x_m);
        assert!(layout.cone_waypoints[10].y_m > left.center.y_m);
        assert_eq!(layout.start_region.max_x_m, START_FINISH_LENGTH_M);
        assert!(
            (layout.finish_region.max_x_m - layout.finish_region.min_x_m - START_FINISH_LENGTH_M)
                .abs()
                < 1e-12
        );
        assert!(
            (layout.light_stop_region.max_x_m - layout.light_stop_region.min_x_m - 1.0).abs()
                < 1e-12
        );
        assert!((right.radius_m - 0.14 * std::f64::consts::SQRT_2).abs() < 1e-12);
    }

    #[test]
    fn start_placement_keeps_body_and_original_quantized_grid_clearance() {
        let engineering = planning();
        let layout = FieldSpec::simulation_example().layout(engineering).unwrap();
        let config = crate::navigation::NavigationConfig::simulation(
            layout.bounds,
            engineering.footprint,
            crate::FrameId("field_test".into()),
        );
        let radius = engineering
            .footprint
            .front_m
            .max(engineering.footprint.rear_m)
            .hypot(engineering.footprint.half_width_m)
            + engineering.clearance_m
            + config.grid_resolution_m * std::f64::consts::FRAC_1_SQRT_2;
        let cell_center = |value: f64| {
            (value / config.grid_resolution_m).floor() * config.grid_resolution_m
                + config.grid_resolution_m / 2.0
        };
        // This is the original Grid's circumscribed body + half cell diagonal,
        // not a new, relaxed start-specific admission envelope.
        assert!(cell_center(layout.initial_pose.x_m) >= radius);
        assert!(cell_center(layout.initial_pose.y_m) >= radius);
        assert!(
            layout.initial_pose.x_m + engineering.footprint.front_m
                <= START_FINISH_LENGTH_M - engineering.clearance_m
        );
        assert!(
            engineering
                .footprint
                .inside(layout.initial_pose, layout.start_region)
        );
        let old_centered_x = (START_FINISH_LENGTH_M + engineering.footprint.rear_m
            - engineering.footprint.front_m)
            / 2.0;
        assert!(cell_center(old_centered_x) < radius);
    }

    #[test]
    fn explicit_small_and_large_layouts_fit_without_scaling_fixed_elements() {
        let mut small = FieldSpec::simulation_example();
        small.length_m = 5.0;
        small.width_m = 4.0;
        small.bottom_lane_width_m = 1.0;
        small.top_lane_width_m = 1.0;
        small.bottom_straight_span_m = 3.0;
        small.top_straight_span_m = 3.0;
        small.left_cone_from_left_m = 1.0;
        small.right_cone_from_right_m = 1.0;
        small.left_cone_from_top_m = 2.0;
        small.right_cone_from_bottom_m = 2.0;
        small.light_front_x_m = 4.2;
        let mut small_planning = planning();
        small_planning.cone_route_radius_m = 0.7;
        let small_layout = small.layout(small_planning).unwrap();
        let mut large = FieldSpec::simulation_example();
        large.length_m = 8.0;
        large.width_m = 6.0;
        large.bottom_straight_span_m = 5.5;
        large.top_straight_span_m = 5.5;
        large.right_cone_from_bottom_m = 3.0;
        large.light_front_x_m = 7.2;
        let large_layout = large.layout(planning()).unwrap();
        for layout in [small_layout, large_layout] {
            assert!(
                (layout.finish_region.max_x_m
                    - layout.finish_region.min_x_m
                    - START_FINISH_LENGTH_M)
                    .abs()
                    < 1e-12
            );
            assert_eq!(
                layout.cones[0].radius_m,
                CONE_BASE_SIDE_M / 2.0 * std::f64::consts::SQRT_2
            );
            assert!(
                planning()
                    .footprint
                    .inside(layout.initial_pose, layout.start_region)
            );
        }
    }

    #[test]
    fn individually_legal_but_conflicting_measurements_are_rejected() {
        let mut spec = FieldSpec::simulation_example();
        spec.length_m = 5.0;
        spec.bottom_straight_span_m = 6.0;
        assert!(spec.layout(planning()).is_err());
        let mut spec = FieldSpec::simulation_example();
        spec.left_cone_from_left_m = 3.0;
        spec.right_cone_from_right_m = 3.0;
        assert!(spec.layout(planning()).is_err());
        let mut spec = FieldSpec::simulation_example();
        spec.width_m = 4.0;
        spec.top_lane_width_m = 2.0;
        spec.bottom_lane_width_m = 2.0;
        assert!(spec.layout(planning()).is_err());
    }

    #[test]
    fn out_of_range_nonfinite_and_unknown_measurements_do_not_fall_back() {
        let mut spec = FieldSpec::simulation_example();
        for value in [f64::NAN, f64::INFINITY, 4.99, 8.01] {
            spec.length_m = value;
            assert!(spec.layout(planning()).is_err());
        }
        let mut value = serde_json::to_value(FieldSpec::simulation_example()).unwrap();
        value["length_cm"] = serde_json::json!(700);
        assert!(serde_json::from_value::<FieldSpec>(value).is_err());
        let mut value = serde_json::to_value(FieldSpec::simulation_example()).unwrap();
        value
            .as_object_mut()
            .unwrap()
            .remove("right_cone_from_bottom_m");
        assert!(serde_json::from_value::<FieldSpec>(value).is_err());
    }

    #[test]
    fn paper_dimensions_and_center_strip_survive_lane_width_change() {
        for width in [1.0, 1.2, 1.8, 2.0] {
            let mut spec = FieldSpec::simulation_example();
            spec.bottom_lane_width_m = width;
            // More vertical space keeps this particular engineered loop viable.
            spec.width_m = 6.0;
            spec.right_cone_from_bottom_m = 3.0;
            let layout = spec.layout(planning()).unwrap();
            let crossing = layout.crosswalk_at(2.0).unwrap();
            assert!((crossing.max_x_m - crossing.min_x_m - CROSSWALK_DEPTH_M).abs() < 1e-12);
            assert!((crossing.min_y_m + crossing.max_y_m - width).abs() < 1e-12);
            let count = ((crossing.max_y_m - crossing.min_y_m + CROSSWALK_GAP_M)
                / (CROSSWALK_STRIPE_WIDTH_M + CROSSWALK_GAP_M))
                .round() as usize;
            assert!(count >= 5 && !count.is_multiple_of(2));
            assert!(
                layout
                    .crosswalk_at(spec.bottom_straight_span_m - 0.1)
                    .is_err()
            );
        }
    }

    #[test]
    fn wide_turn_keeps_full_reference_arc_outside_the_light_corner() {
        let mut spec = FieldSpec::simulation_example();
        spec.length_m = 8.0;
        spec.width_m = 4.0;
        spec.bottom_lane_width_m = 1.0;
        spec.top_lane_width_m = 1.0;
        spec.bottom_straight_span_m = 5.5;
        spec.top_straight_span_m = 5.5;
        spec.left_cone_from_top_m = 2.0;
        spec.right_cone_from_bottom_m = 2.0;
        spec.light_front_x_m = 7.2;
        let engineering = planning();
        let layout = spec.layout(engineering).unwrap();
        let boundary = crate::autonomy::HalfPlane::at_region_front(
            layout.light_stop_goal,
            layout.light_stop_region,
            0.0,
        )
        .unwrap()
        .with_lateral_region(layout.light_boundary_region)
        .unwrap();
        let pad = engineering
            .footprint
            .front_m
            .max(engineering.footprint.rear_m)
            .hypot(engineering.footprint.half_width_m)
            + engineering.clearance_m
            + engineering.grid_resolution_m * std::f64::consts::FRAC_1_SQRT_2;
        // The old preferred .85 m arc intersects this same conservative corner.
        let center = layout.cones[0].center;
        let angle = std::f64::consts::PI / 3.0;
        let old_arc_point = point(
            center.x_m + 0.85 * angle.cos(),
            center.y_m + 0.85 * angle.sin(),
        );
        assert!(!boundary.contains_disc(old_arc_point, pad));
        assert!(layout.selected_cone_route_radii_m[0] < engineering.cone_route_radius_m);
        assert!(layout.selected_cone_route_radii_m[0] >= 1.0 / engineering.max_curvature_per_m);
        for (index, direction) in [(0, 1.0), (1, -1.0)] {
            let center = layout.cones[index].center;
            let radius = layout.selected_cone_route_radii_m[index];
            for degree in 0..=180 {
                let angle = -std::f64::consts::FRAC_PI_2 + direction * (degree as f64).to_radians();
                let p = point(
                    center.x_m + radius * angle.cos(),
                    center.y_m + radius * angle.sin(),
                );
                assert!(boundary.contains_disc(p, pad));
            }
        }
        for i in 0..10 {
            let yaw = layout.cone_waypoint_headings_rad[i].unwrap();
            let center = layout.cones[i / 5].center;
            let radial = point(
                layout.cone_waypoints[i].x_m - center.x_m,
                layout.cone_waypoints[i].y_m - center.y_m,
            );
            assert!((radial.x_m * yaw.cos() + radial.y_m * yaw.sin()).abs() < 1e-12);
        }
    }

    #[test]
    fn layout_never_shrinks_vehicle_to_fit_fixed_start_or_cone_loop() {
        let mut engineering = planning();
        engineering.footprint.front_m = 0.7;
        assert!(FieldSpec::simulation_example().layout(engineering).is_err());
        let mut engineering = planning();
        engineering.cone_route_radius_m = 0.50;
        assert!(FieldSpec::simulation_example().layout(engineering).is_err());
        let mut engineering = planning();
        engineering.cone_route_radius_m = 2.0;
        engineering.max_curvature_per_m = 0.5;
        assert!(FieldSpec::simulation_example().layout(engineering).is_err());
    }

    #[test]
    fn measured_provenance_is_preserved_without_implying_vehicle_readiness() {
        let mut spec = FieldSpec::simulation_example();
        spec.source = FieldSource::Measured;
        let layout = spec.layout(planning()).unwrap();
        assert_eq!(layout.source, FieldSource::Measured);
        assert_eq!(
            serde_json::from_str::<FieldSpec>(&serde_json::to_string(&spec).unwrap()).unwrap(),
            spec
        );
    }
}
