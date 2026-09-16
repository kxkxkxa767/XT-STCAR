//! Compile one measured field before starting the immutable controller/worker.
//! This adapter never estimates a lamp's distance from its color and never
//! updates mission coordinates behind an already running worker.
use crate::simulation::SimulationConfig;
use serde::{Deserialize, Serialize};
use xt_stcar_robot_core::autonomy::{Point2, Pose2};
use xt_stcar_robot_core::field::{FieldLayout, FieldPlanning, FieldSpec};

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct FieldScenario {
    pub schema_version: u32,
    pub spec: FieldSpec,
    /// Engineering route radius, not a scaled vehicle or relaxed steering limit.
    pub cone_route_radius_m: f64,
    /// Synthetic truth used only by the camera renderer. Mission receives the
    /// whole search region, never this exact crossing position.
    pub crosswalk_near_x_m: f64,
    pub light_red_duration_ms: u64,
}

impl FieldScenario {
    pub fn example() -> Self {
        Self {
            schema_version: 1,
            spec: FieldSpec::simulation_example(),
            cone_route_radius_m: 0.85,
            crosswalk_near_x_m: 2.5,
            light_red_duration_ms: 5000,
        }
    }

    pub fn layout(&self) -> Result<FieldLayout, String> {
        self.layout_for(&SimulationConfig::example())
    }

    fn layout_for(&self, config: &SimulationConfig) -> Result<FieldLayout, String> {
        if self.schema_version != 1 || !(3000..=10000).contains(&self.light_red_duration_ms) {
            return Err("invalid field scenario schema or red-light duration".into());
        }
        let layout = self
            .spec
            .layout(FieldPlanning {
                footprint: config.autonomy.navigation.footprint,
                clearance_m: config.autonomy.navigation.clearance_m,
                max_curvature_per_m: config.autonomy.navigation.max_curvature_per_m,
                grid_resolution_m: config.autonomy.navigation.grid_resolution_m,
                cone_route_radius_m: self.cone_route_radius_m,
            })
            .map_err(|e| e.to_string())?;
        // Any crossing admitted by the search region must leave the original
        // stop margin ahead of the initial complete footprint. Checking the
        // renderer's actual crossing instead would hide incompatible search
        // regions and make startup depend on undisclosed synthetic truth.
        let initial_front_x_m = config
            .autonomy
            .mission
            .footprint
            .corners(layout.initial_pose)
            .into_iter()
            .map(|corner| corner.x_m)
            .fold(f64::NEG_INFINITY, f64::max);
        let required_near_x_m = initial_front_x_m + config.autonomy.mission.crosswalk_stop_margin_m;
        let earliest_near_x_m = layout.crosswalk_search_region.min_x_m;
        if !required_near_x_m.is_finite() || earliest_near_x_m < required_near_x_m {
            return Err(format!(
                "earliest crosswalk near edge {earliest_near_x_m} m leaves insufficient initial \
                 stopping margin: require at least {required_near_x_m} m, shortfall {} m",
                required_near_x_m - earliest_near_x_m
            ));
        }
        Ok(layout)
    }

    /// Produce a complete offline configuration. Vehicle limits and confidence,
    /// stop, timing, and budget thresholds retain the original example values.
    pub fn compile(&self) -> Result<SimulationConfig, String> {
        let mut config = SimulationConfig::example();
        self.apply(&mut config)?;
        config.field = Some(self.clone());
        // Declared once for the larger official topology (not adjusted after a
        // run fails). The original 10-second ordinary-stop rule stays unchanged.
        config.max_duration_ms = 180_000;
        config.validate()?;
        Ok(config)
    }

    fn apply(&self, config: &mut SimulationConfig) -> Result<(), String> {
        let layout = self.layout_for(config)?;
        config.crosswalk = layout
            .crosswalk_at(self.crosswalk_near_x_m)
            .map_err(|e| e.to_string())?;
        config.initial_pose = layout.initial_pose;
        config.cones = layout.cones.to_vec();
        config.light_trigger_x_m = self.spec.light_front_x_m - 1.0;
        config.light_red_duration_ms = self.light_red_duration_ms;
        config.autonomy.navigation.bounds = layout.bounds;
        // The actual 280 mm square cone base needs its circumscribed circle.
        config.autonomy.cone_radius_m = layout.cones[0].radius_m;
        let mission = &mut config.autonomy.mission;
        mission.approach_goal = layout.crosswalk_search_end;
        mission.crosswalk_region = layout.crosswalk_search_region;
        mission.cone_waypoints = layout.cone_waypoints;
        mission.cone_waypoint_headings_rad = layout.cone_waypoint_headings_rad;
        mission.light_detection_region = layout.light_detection_region;
        mission.light_boundary_region = Some(layout.light_boundary_region);
        mission.light_stop_region = layout.light_stop_region;
        mission.light_stop_goal = layout.light_stop_goal;
        mission.light_approach_yaw_rad = layout.light_approach_yaw_rad;
        mission.finish_region = layout.finish_region;
        mission.finish_goal = layout.finish_goal;
        mission.finish_yaw_rad = layout.finish_yaw_rad;
        Ok(())
    }

    /// Imported compiled JSON cannot silently disagree with its specification.
    /// Recompute only at construction/validation, never on a control tick.
    pub(crate) fn validate_compiled(&self, config: &SimulationConfig) -> Result<(), String> {
        let mut expected = config.clone();
        self.apply(&mut expected)?;
        let geometry = |c: &SimulationConfig| {
            serde_json::json!({
                "mission": c.autonomy.mission,
                "bounds": c.autonomy.navigation.bounds,
                "cone_radius_m": c.autonomy.cone_radius_m,
                "initial_pose": c.initial_pose,
                "crosswalk": c.crosswalk,
                "cones": c.cones,
                "light_trigger_x_m": c.light_trigger_x_m,
                "light_red_duration_ms": c.light_red_duration_ms,
            })
        };
        if !same_compiled_geometry(&geometry(&expected), &geometry(config)) {
            return Err(
                "compiled field geometry differs from field spec; regenerate it before starting"
                    .into(),
            );
        }
        Ok(())
    }
}

/// JSON's default f64 parser can move a serialized trigonometric coordinate by
/// one ULP. Permit only roundoff-scale differences in floating leaves; object
/// keys, array lengths, integer values and all nonnumeric data remain exact.
/// This is a file round-trip tolerance, never a navigation/arrival tolerance.
fn same_compiled_geometry(expected: &serde_json::Value, actual: &serde_json::Value) -> bool {
    use serde_json::Value;
    match (expected, actual) {
        (Value::Object(expected), Value::Object(actual)) => {
            expected.len() == actual.len()
                && expected.iter().all(|(key, expected)| {
                    actual
                        .get(key)
                        .is_some_and(|actual| same_compiled_geometry(expected, actual))
                })
        }
        (Value::Array(expected), Value::Array(actual)) => {
            expected.len() == actual.len()
                && expected
                    .iter()
                    .zip(actual)
                    .all(|(expected, actual)| same_compiled_geometry(expected, actual))
        }
        (Value::Number(expected), Value::Number(actual))
            if expected.is_f64() && actual.is_f64() =>
        {
            let (Some(expected), Some(actual)) = (expected.as_f64(), actual.as_f64()) else {
                return false;
            };
            expected.is_finite()
                && actual.is_finite()
                && (expected - actual).abs()
                    <= 8.0 * f64::EPSILON * expected.abs().max(actual.abs()).max(1.0)
        }
        _ => expected == actual,
    }
}

/// Synthetic light triggering is spatially scoped to the measured upper lane.
/// The controller does not receive this trigger or the red timer.
pub(crate) fn light_triggered(config: &SimulationConfig, pose: Pose2) -> bool {
    let front = pose.body_to_world(Point2 {
        x_m: config.autonomy.mission.footprint.front_m,
        y_m: 0.0,
    });
    if let Some(scene) = &config.online_scene {
        let region = scene.light_boundary_region;
        return front.x_m >= config.light_trigger_x_m
            && front.y_m >= region.min_y_m
            && front.y_m <= region.max_y_m
            && pose.yaw_rad.cos() > 0.0;
    }
    if config.field.is_none() {
        // Preserve the legacy fixture's exact predicate, including its x-only
        // assumption; new field scenarios use their actual oriented footprint.
        return pose.x_m + config.autonomy.mission.footprint.front_m >= config.light_trigger_x_m;
    }
    let region = config
        .autonomy
        .mission
        .light_boundary_region
        .expect("validated field boundary");
    front.x_m >= config.light_trigger_x_m
        && front.y_m >= region.min_y_m
        && front.y_m <= region.max_y_m
        && pose.yaw_rad.cos() > 0.0
}

/// This is only a bounded visibility model for synthetic RGB, not geometric
/// camera calibration, lamp ranging, or a ground-truth input to Mission.
pub(crate) fn light_visible(config: &SimulationConfig, pose: Pose2) -> bool {
    if let Some(scene) = &config.online_scene {
        let region = scene.light_stop_region;
        let target = Point2 {
            x_m: region.max_x_m,
            y_m: (region.min_y_m + region.max_y_m) / 2.0,
        };
        let local = pose.world_to_body(target);
        return local.x_m >= -config.autonomy.mission.footprint.rear_m
            && local.x_m <= config.road.homography.max_forward_m
            && local.y_m.abs() <= region.max_y_m - region.min_y_m
            && pose.y_m >= scene.light_boundary_region.min_y_m - 0.5
            && pose.yaw_rad.cos() >= 0.8;
    }
    let Some(field) = &config.field else {
        return true;
    };
    let target = Point2 {
        x_m: field.spec.light_front_x_m,
        y_m: field.spec.width_m - field.spec.top_lane_width_m / 2.0,
    };
    let local = pose.world_to_body(target);
    local.x_m >= -config.autonomy.mission.footprint.rear_m
        && local.x_m <= config.road.homography.max_forward_m
        && local.y_m.abs() <= field.spec.top_lane_width_m
        && pose.y_m >= field.spec.width_m - field.spec.top_lane_width_m - 0.5
        && pose.yaw_rad.cos() >= 0.8
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn compiled_geometry_allows_only_float_roundoff_and_exact_structure() {
        use serde_json::json;
        let original = json!({"gates": [{"x": 1.3996699141100895, "heading": null}], "version": 1});
        let mut rounded = original.clone();
        rounded["gates"][0]["x"] = json!(1.3996699141100897);
        assert!(same_compiled_geometry(&original, &rounded));
        for delta in [0.01, 1e-12] {
            let mut changed = original.clone();
            changed["gates"][0]["x"] = json!(1.3996699141100895 + delta);
            assert!(!same_compiled_geometry(&original, &changed));
        }
        for changed in [
            json!({"gates": [], "version": 1}),
            json!({"gates": [{"x": 1.3996699141100895}], "version": 1}),
            json!({"gates": [{"x": 1.3996699141100895, "heading": null, "extra": 0}], "version": 1}),
            json!({"gates": [{"x": "1.3996699141100895", "heading": null}], "version": 1}),
            json!({"gates": [{"x": 1.3996699141100895, "heading": 0.0}], "version": 1}),
            json!({"gates": [{"x": 1.3996699141100895, "heading": null}], "version": 2}),
        ] {
            assert!(!same_compiled_geometry(&original, &changed));
        }
        assert!(!same_compiled_geometry(&json!(1), &json!(1.0)));
    }

    #[test]
    fn compiles_geometry_without_scaling_control_or_leaking_zebra_truth() {
        let first = FieldScenario::example();
        let a = first.compile().unwrap();
        let mut second = first.clone();
        second.crosswalk_near_x_m = 3.6;
        let b = second.compile().unwrap();
        assert_ne!(a.crosswalk, b.crosswalk);
        assert_eq!(
            serde_json::to_value(&a.autonomy).unwrap(),
            serde_json::to_value(&b.autonomy).unwrap()
        );
        let base = SimulationConfig::example();
        assert_eq!(
            a.autonomy.mission.footprint,
            base.autonomy.mission.footprint
        );
        assert_eq!(
            a.autonomy.navigation.max_speed_mps,
            base.autonomy.navigation.max_speed_mps
        );
        assert_eq!(
            a.autonomy.navigation.max_curvature_per_m,
            base.autonomy.navigation.max_curvature_per_m
        );
        assert_eq!(
            a.autonomy.mission.goal_tolerance_m,
            base.autonomy.mission.goal_tolerance_m
        );
        assert_eq!(a.autonomy.mission.crosswalk_hold_ms, 3000);
        assert!(a.autonomy.simulation_only);
        assert_eq!(a.autonomy.measurement_status, "unverified");
        let mut changed = a;
        changed.autonomy.mission.finish_goal.x_m -= 0.1;
        assert!(changed.validate().is_err());
    }

    #[test]
    fn lower_course_does_not_trigger_or_see_upper_light() {
        let c = FieldScenario::example().compile().unwrap();
        let lower = Pose2 {
            x_m: 6.0,
            y_m: 0.6,
            yaw_rad: 0.0,
        };
        assert!(!light_triggered(&c, lower));
        assert!(!light_visible(&c, lower));
        let upper = Pose2 {
            x_m: 5.5,
            y_m: 4.4,
            yaw_rad: 0.0,
        };
        assert!(light_triggered(&c, upper));
        assert!(light_visible(&c, upper));
    }
}
