//! Arbitrary rule-range truth scenes for the production sensor-only controller.
//! Does not require the old fixed-route gate polygon to be navigable.
use crate::online::OnlineControlConfig;
use crate::online_simulation::{OnlineScenario, RecognitionScope, VisualOcclusion};
use crate::simulation::SimulationConfig;
use crate::simulation_geometry::SimWall;
use serde::{Deserialize, Serialize};
use xt_stcar_robot_core::autonomy::{ObstacleDisc, Point2, Pose2, Rect};
use xt_stcar_robot_core::field::{FieldSource, FieldSpec};

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct SweepScene {
    pub spec: FieldSpec,
    pub crosswalk_near_x_m: f64,
    pub light_red_duration_ms: u64,
    pub corridor_walls: bool,
    pub scoped_ideal_semantics: bool,
    pub lidar_range_noise_m: f64,
}

impl SweepScene {
    pub fn example() -> Self {
        Self {
            spec: FieldSpec::simulation_example(),
            crosswalk_near_x_m: 2.5,
            light_red_duration_ms: 5000,
            corridor_walls: false,
            scoped_ideal_semantics: true,
            lidar_range_noise_m: 0.0,
        }
    }

    pub fn compile(&self) -> Result<SimulationConfig, String> {
        self.spec
            .validate_measurements()
            .map_err(|e| e.to_string())?;
        let s = &self.spec;
        if s.source != FieldSource::Simulation
            || !(3000..=10000).contains(&self.light_red_duration_ms)
            || !self.crosswalk_near_x_m.is_finite()
            || self.crosswalk_near_x_m < s.crosswalk_search_start_m
            || self.crosswalk_near_x_m + 0.297 > s.bottom_straight_span_m
        {
            return Err("invalid explicit offline sweep crossing/source/light timing".into());
        }
        // Only use the valid nominal scene to obtain the unchanged controller defaults.
        let mut config = OnlineScenario::example().compile()?;
        let footprint = config.autonomy.navigation.footprint;
        let clearance = config.autonomy.navigation.clearance_m;
        config.initial_pose = Pose2 {
            x_m: 0.8 - footprint.front_m - 2.0 * clearance,
            y_m: s.bottom_lane_width_m / 2.0,
            yaw_rad: 0.0,
        };
        if s.crosswalk_search_start_m
            < config.initial_pose.x_m
                + footprint.front_m
                + config.autonomy.mission.crosswalk_stop_margin_m
        {
            return Err(
                "crossing recognition interval lacks original initial stopping margin".into(),
            );
        }
        config.crosswalk = rect(
            self.crosswalk_near_x_m,
            0.0,
            self.crosswalk_near_x_m + 0.297,
            s.bottom_lane_width_m,
        );
        let radius = 0.14 * 2.0f64.sqrt();
        config.cones = vec![
            ObstacleDisc {
                center: Point2 {
                    x_m: s.length_m - s.right_cone_from_right_m,
                    y_m: s.right_cone_from_bottom_m,
                },
                radius_m: radius,
            },
            ObstacleDisc {
                center: Point2 {
                    x_m: s.left_cone_from_left_m,
                    y_m: s.width_m - s.left_cone_from_top_m,
                },
                radius_m: radius,
            },
        ];
        let bounds = rect(0.0, 0.0, s.length_m, s.width_m);
        if config.cones.iter().any(|c| {
            c.center.x_m - c.radius_m < bounds.min_x_m
                || c.center.x_m + c.radius_m > bounds.max_x_m
                || c.center.y_m - c.radius_m < bounds.min_y_m
                || c.center.y_m + c.radius_m > bounds.max_y_m
        }) || config.cones[1].center.x_m >= config.cones[0].center.x_m
            || config.cones[0].center.distance(config.cones[1].center) <= 2.0 * radius
        {
            return Err(
                "actual cone discs conflict with bounds, identity order or each other".into(),
            );
        }
        config.light_trigger_x_m = s.light_front_x_m - 1.0;
        config.light_red_duration_ms = self.light_red_duration_ms;
        config.autonomy.online = Some(OnlineControlConfig::lidar_first(&config.autonomy));
        let scene = config.online_scene.as_mut().expect("nominal online truth");
        scene.bounds = bounds;
        scene.light_stop_region = rect(
            s.light_front_x_m - s.light_stop_length_m,
            s.width_m - s.top_lane_width_m,
            s.light_front_x_m,
            s.width_m,
        );
        scene.light_boundary_region =
            rect(0.0, s.width_m - s.top_lane_width_m, s.length_m, s.width_m);
        scene.finish_region = rect(
            s.length_m - 0.8,
            s.width_m - s.top_lane_width_m,
            s.length_m,
            s.width_m,
        );
        scene.ideal_non_cone_semantics = true;
        scene.lidar_range_noise_m = self.lidar_range_noise_m;
        scene.occlusions = vec![VisualOcclusion {
            from_ms: 0,
            through_ms: 180000,
            cones: true,
            markers: false,
        }];
        if self.scoped_ideal_semantics {
            scene.recognition_scope = Some(RecognitionScope {
                crosswalk_region: rect(
                    s.crosswalk_search_start_m,
                    0.0,
                    s.bottom_straight_span_m,
                    s.bottom_lane_width_m,
                ),
                light_region: rect(
                    s.length_m - s.top_straight_span_m,
                    s.width_m - s.top_lane_width_m,
                    s.length_m,
                    s.width_m,
                ),
                marker_range_m: 3.0,
                horizontal_fov_rad: 150.0f64.to_radians(),
            });
        }
        if self.corridor_walls {
            // Explicit corridor hypothesis; recognition outlines alone do not prove these walls exist.
            scene.internal_walls = vec![
                SimWall {
                    start: Point2 {
                        x_m: 0.0,
                        y_m: s.bottom_lane_width_m,
                    },
                    end: Point2 {
                        x_m: s.bottom_straight_span_m,
                        y_m: s.bottom_lane_width_m,
                    },
                    half_width_m: 0.01,
                },
                SimWall {
                    start: Point2 {
                        x_m: s.length_m - s.top_straight_span_m,
                        y_m: s.width_m - s.top_lane_width_m,
                    },
                    end: Point2 {
                        x_m: s.length_m,
                        y_m: s.width_m - s.top_lane_width_m,
                    },
                    half_width_m: 0.01,
                },
            ];
            if scene.internal_walls.iter().any(|wall| {
                config
                    .cones
                    .iter()
                    .any(|c| wall.disc_clearance(c.center, c.radius_m) <= 0.0)
            }) {
                return Err("cone disc intersects declared corridor wall".into());
            }
        }
        config.validate()?;
        Ok(config)
    }
}

fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Rect {
    Rect {
        min_x_m: x0,
        min_y_m: y0,
        max_x_m: x1,
        max_y_m: y1,
    }
}

pub fn accepted(summary: &serde_json::Value) -> bool {
    summary["completed"] == true
        && summary["fault"].is_null()
        && summary["physical_output_enabled"] == false
        && summary["online_referee"]["both_completed"] == true
        && summary["online_referee"]["order_valid"] == true
        && summary["online_referee"]["trajectory_valid"] == true
        && summary["final_actual_speed_mps"].as_f64() == Some(0.0)
        && summary["final_actual_curvature_per_m"].as_f64() == Some(0.0)
        && summary["crosswalk_hold_ms"]
            .as_u64()
            .is_some_and(|v| v >= 3000)
        && summary["green_observed_ms"]
            .as_u64()
            .is_some_and(|v| v >= 300)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::simulation::{check_plant_pose, synthetic_scan};
    use xt_stcar_robot_core::Timestamp;
    #[test]
    fn scoped_semantics_do_not_disclose_future_markers_or_light() {
        use xt_stcar_robot_core::autonomy::LightState;
        let scene = SweepScene::example();
        let config = scene.compile().unwrap();
        let frame = crate::online_simulation::ideal_non_cone_frame(
            &config,
            config.initial_pose,
            LightState::Green,
            Timestamp(0),
            320,
            240,
        )
        .unwrap();
        assert!(frame.elements.unwrap().observations.is_empty());
        assert_eq!(frame.observation.light, LightState::Unknown);
        let visible = crate::online_simulation::ideal_non_cone_frame(
            &config,
            Pose2 {
                x_m: 1.2,
                ..config.initial_pose
            },
            LightState::Green,
            Timestamp(100),
            320,
            240,
        )
        .unwrap();
        let observations = visible.elements.unwrap().observations;
        assert_eq!(observations.len(), 1);
        assert_eq!(
            observations[0].kind,
            xt_stcar_robot_core::local_world::ElementKind::Crosswalk
        );
        let mut legacy = scene;
        legacy.scoped_ideal_semantics = false;
        let config = legacy.compile().unwrap();
        let frame = crate::online_simulation::ideal_non_cone_frame(
            &config,
            config.initial_pose,
            LightState::Green,
            Timestamp(0),
            320,
            240,
        )
        .unwrap();
        assert_eq!(frame.elements.unwrap().observations.len(), 3);
        assert_eq!(frame.observation.light, LightState::Green);
    }
    #[test]
    fn independent_spans_change_true_walls_and_visibility_without_changing_controller() {
        let mut a = SweepScene::example();
        a.corridor_walls = true;
        let mut b = a.clone();
        b.spec.bottom_straight_span_m = 3.5;
        b.spec.top_straight_span_m = 5.0;
        let ca = a.compile().unwrap();
        let cb = b.compile().unwrap();
        assert_eq!(
            serde_json::to_value(&ca.autonomy).unwrap(),
            serde_json::to_value(&cb.autonomy).unwrap()
        );
        assert_ne!(
            serde_json::to_value(&ca.online_scene).unwrap(),
            serde_json::to_value(&cb.online_scene).unwrap()
        );
        let pose = Pose2 {
            x_m: 4.0,
            y_m: 0.6,
            yaw_rad: 0.0,
        };
        assert_ne!(
            synthetic_scan(&ca, pose, &ca.cones, Timestamp(0)),
            synthetic_scan(&cb, pose, &cb.cones, Timestamp(0))
        );
    }
    #[test]
    fn inner_wall_is_also_checked_by_independent_plant_and_invalid_ranges_reject() {
        let mut scene = SweepScene::example();
        scene.corridor_walls = true;
        let c = scene.compile().unwrap();
        let mut minimum = f64::INFINITY;
        assert!(check_plant_pose(
            &c,
            Pose2 {
                x_m: 1.5,
                y_m: 1.2,
                yaw_rad: 0.0
            },
            &c.cones,
            None,
            &mut minimum
        ));
        assert!(!check_plant_pose(
            &c,
            c.initial_pose,
            &c.cones,
            None,
            &mut minimum
        ));
        scene.spec.bottom_straight_span_m = 6.1;
        assert!(scene.compile().is_err());
    }
}
