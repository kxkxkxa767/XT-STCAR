//! Simulator-owned truth for the sensor-only online controller.
//! Only the camera renderer, range renderer and independent referee use this data.
use crate::autonomy::{AutonomyConfig, Result, RoadFrame};
use crate::field::FieldScenario;
use crate::online::OnlineControlConfig;
use crate::simulation::SimulationConfig;
use image::RgbImage;
use serde::{Deserialize, Serialize};
use xt_stcar_robot_core::Timestamp;
use xt_stcar_robot_core::autonomy::{HalfPlane, Pose2, Rect};
use xt_stcar_vision::ground_markers::{ExperimentalMarkerConfig, GroundMarkerDetector};
use xt_stcar_vision::road::RoadDetector;

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct OnlineScene {
    pub bounds: Rect,
    pub light_stop_region: Rect,
    pub light_boundary_region: Rect,
    pub finish_region: Rect,
    /// Explicit experimental appearance, not a claim about the actual competition.
    pub markers: ExperimentalMarkerConfig,
    #[serde(default)]
    pub occlusions: Vec<VisualOcclusion>,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct VisualOcclusion {
    pub from_ms: u64,
    pub through_ms: u64,
    pub cones: bool,
    pub markers: bool,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct OnlineScenario {
    pub schema_version: u32,
    /// Truth is used to make sensor images/ranges, never to make online targets.
    pub scene: FieldScenario,
    pub experimental_markers_enabled: bool,
    #[serde(default)]
    pub occlusions: Vec<VisualOcclusion>,
}

impl OnlineScenario {
    pub fn example() -> Self {
        Self {
            schema_version: 1,
            scene: FieldScenario::example(),
            experimental_markers_enabled: true,
            occlusions: Vec::new(),
        }
    }

    pub fn compile(&self) -> Result<SimulationConfig> {
        if self.schema_version != 1
            || self.occlusions.len() > 16
            || self
                .occlusions
                .iter()
                .any(|o| o.from_ms > o.through_ms || o.through_ms > 180_000)
        {
            return Err("invalid online scene schema or bounded occlusion interval".into());
        }
        let mut simulation = self.scene.compile()?;
        let mut markers = ExperimentalMarkerConfig::simulation();
        markers.enabled = self.experimental_markers_enabled;
        let truth = OnlineScene {
            bounds: simulation.autonomy.navigation.bounds,
            light_stop_region: simulation.autonomy.mission.light_stop_region,
            light_boundary_region: simulation
                .autonomy
                .mission
                .light_boundary_region
                .ok_or("field scene requires finite light region")?,
            finish_region: simulation.autonomy.mission.finish_region,
            markers,
            occlusions: self.occlusions.clone(),
        };
        simulation.field = None;
        simulation.autonomy = controller_config();
        simulation.online_scene = Some(truth);
        simulation.validate()?;
        Ok(simulation)
    }
}

/// Byte-identical across changes to scene dimensions, cone, lamp and finish
/// positions. The broad operating envelope is a rule prior, not the true walls.
pub fn controller_config() -> AutonomyConfig {
    let mut config = SimulationConfig::example().autonomy;
    config.navigation.bounds = Rect {
        min_x_m: -1.0,
        max_x_m: 9.0,
        min_y_m: -1.0,
        max_y_m: 7.0,
    };
    config.cone_radius_m = 0.14 * 2.0f64.sqrt();
    config.online = Some(OnlineControlConfig::from_limits(&config));
    config
}

pub(crate) fn scene_bounds(config: &SimulationConfig) -> Rect {
    config
        .online_scene
        .as_ref()
        .map_or(config.autonomy.navigation.bounds, |s| s.bounds)
}

pub(crate) fn scene_finish(config: &SimulationConfig) -> (Rect, f64) {
    config.online_scene.as_ref().map_or(
        (
            config.autonomy.mission.finish_region,
            config.autonomy.mission.finish_yaw_rad,
        ),
        |s| (s.finish_region, 0.0),
    )
}

pub(crate) fn scene_light_boundary(config: &SimulationConfig) -> Result<HalfPlane> {
    let Some(scene) = &config.online_scene else {
        return config
            .autonomy
            .mission
            .light_stop_boundary()
            .map_err(|e| e.to_string());
    };
    let center = xt_stcar_robot_core::autonomy::Point2 {
        x_m: (scene.light_stop_region.min_x_m + scene.light_stop_region.max_x_m) / 2.0,
        y_m: (scene.light_stop_region.min_y_m + scene.light_stop_region.max_y_m) / 2.0,
    };
    HalfPlane::at_region_front(center, scene.light_stop_region, 0.0)
        .and_then(|b| b.with_lateral_region(scene.light_boundary_region))
        .map_err(|e| e.to_string())
}

pub(crate) fn detect_frame(
    config: &SimulationConfig,
    detector: &RoadDetector,
    image: &RgbImage,
    at: Timestamp,
    expected_heading_body_rad: f64,
) -> Result<RoadFrame> {
    let frame = config.autonomy.mission.body_frame.clone();
    let observation = detector.detect(image, &[], at, frame.clone())?;
    let elements = if let Some(scene) = &config.online_scene {
        let mut elements =
            detector.detect_elements(image, &[], at, frame.clone(), expected_heading_body_rad)?;
        let marker_detector =
            GroundMarkerDetector::new(config.road.clone(), scene.markers.clone())?;
        let markers = marker_detector.detect(image, &[], at, frame, expected_heading_body_rad)?;
        elements.observations.extend(markers.observations);
        if elements.observations.len() > xt_stcar_robot_core::local_world::MAX_ELEMENTS {
            return Err("online visual element capacity exceeded".into());
        }
        Some(elements)
    } else {
        None
    };
    Ok(RoadFrame {
        observation,
        elements,
        image_width_px: image.width(),
        image_height_px: image.height(),
    })
}

pub(crate) fn is_occluded(scene: &OnlineScene, at: Timestamp, cones: bool) -> bool {
    scene.occlusions.iter().any(|o| {
        at.0 >= o.from_ms && at.0 <= o.through_ms && if cones { o.cones } else { o.markers }
    })
}

pub(crate) fn validate_scene(config: &SimulationConfig) -> Result<()> {
    match (&config.online_scene, &config.autonomy.online) {
        (None, None) => return Ok(()),
        (Some(scene), Some(_)) => {
            scene.bounds.validate().map_err(|e| e.to_string())?;
            scene
                .light_stop_region
                .validate()
                .map_err(|e| e.to_string())?;
            scene.finish_region.validate().map_err(|e| e.to_string())?;
            scene_light_boundary(config)?;
            GroundMarkerDetector::new(config.road.clone(), scene.markers.clone())?;
            if config.field.is_some()
                || scene.occlusions.len() > 16
                || !config
                    .autonomy
                    .navigation
                    .footprint
                    .inside(config.initial_pose, scene.bounds)
                || scene
                    .occlusions
                    .iter()
                    .any(|o| o.from_ms > o.through_ms || o.through_ms > 180_000)
            {
                return Err("invalid separated online simulation scene".into());
            }
        }
        _ => return Err("online simulation requires both scene and sensor-only controller".into()),
    }
    Ok(())
}

pub(crate) fn marker_region_pose(region: Rect) -> Pose2 {
    Pose2 {
        x_m: region.max_x_m,
        y_m: (region.min_y_m + region.max_y_m) / 2.0,
        yaw_rad: 0.0,
    }
}

/// Independent simulated trajectory evidence. All cone positions come only from
/// the renderer/referee scene. This object is never passed to the controller.
#[derive(Clone, Debug, Serialize)]
pub struct OnlineRefereeSummary {
    pub available_cones: usize,
    pub actual_pose_samples: u64,
    pub completed_cones: usize,
    pub both_completed: bool,
    pub order_valid: bool,
    pub trajectory_valid: bool,
    pub failure_reason: Option<&'static str>,
    pub cones: [Option<ConePassageEvidence>; 2],
}

#[derive(Clone, Copy, Debug, Serialize)]
pub struct ConePassageEvidence {
    pub index: usize,
    pub direction: &'static str,
    pub entered_at: Option<Timestamp>,
    pub outer_side_at: Option<Timestamp>,
    pub passed_at: Option<Timestamp>,
    /// Signed observed progress: reverse travel subtracts; never clipped to zero.
    pub angular_progress_rad: f64,
    pub entry_heading_error_rad: Option<f64>,
    pub exit_heading_error_rad: Option<f64>,
    pub began_in_required_order: bool,
    pub passed_in_required_order: bool,
}

#[derive(Clone, Copy)]
struct PassageState {
    cone: xt_stcar_robot_core::autonomy::ObstacleDisc,
    evidence: ConePassageEvidence,
    active: bool,
    previous_angle: f64,
}

pub(crate) struct OnlineReferee {
    axis_rad: f64,
    footprint: xt_stcar_robot_core::autonomy::Footprint,
    heading_tolerance_rad: f64,
    passages: [Option<PassageState>; 2],
    previous: Option<(Timestamp, Pose2)>,
    samples: u64,
    order_valid: bool,
    trajectory_valid: bool,
    failure_reason: Option<&'static str>,
}

impl OnlineReferee {
    pub(crate) fn for_simulation(config: &SimulationConfig) -> Result<Option<Self>> {
        if config.online_scene.is_none() {
            return Ok(None);
        }
        Self::new(
            &config.cones,
            config.initial_pose,
            config.autonomy.navigation.footprint,
            config.autonomy.navigation.goal_heading_tolerance_rad,
        )
        .map(Some)
    }

    fn new(
        cones: &[xt_stcar_robot_core::autonomy::ObstacleDisc],
        initial: Pose2,
        footprint: xt_stcar_robot_core::autonomy::Footprint,
        heading_tolerance_rad: f64,
    ) -> Result<Self> {
        footprint.validate().map_err(|e| e.to_string())?;
        if cones.len() > 2
            || !initial.valid()
            || !heading_tolerance_rad.is_finite()
            || !(0.0..std::f64::consts::FRAC_PI_2).contains(&heading_tolerance_rad)
            || cones
                .iter()
                .any(|c| !c.center.valid() || !c.radius_m.is_finite() || c.radius_m <= 0.0)
            || (cones.len() == 2
                && cones[0].center.distance(cones[1].center)
                    <= cones[0].radius_m + cones[1].radius_m)
        {
            return Err("invalid independent online referee geometry".into());
        }
        let mut passages = [None; 2];
        for (index, &cone) in cones.iter().enumerate() {
            passages[index] = Some(PassageState {
                cone,
                active: false,
                previous_angle: 0.0,
                evidence: ConePassageEvidence {
                    index,
                    direction: if index == 0 {
                        "counterclockwise"
                    } else {
                        "clockwise"
                    },
                    entered_at: None,
                    outer_side_at: None,
                    passed_at: None,
                    angular_progress_rad: 0.0,
                    entry_heading_error_rad: None,
                    exit_heading_error_rad: None,
                    began_in_required_order: false,
                    passed_in_required_order: false,
                },
            });
        }
        let mut referee = Self {
            axis_rad: initial.yaw_rad,
            footprint,
            heading_tolerance_rad,
            passages,
            previous: None,
            samples: 0,
            order_valid: true,
            trajectory_valid: true,
            failure_reason: None,
        };
        referee.observe(Timestamp(0), initial);
        Ok(referee)
    }

    /// Called at every actual plant integration sample, including braking.
    /// at is the containing integration interval's end: events have <=100ms
    /// timestamp quantization in sync and <=1ms during normal async motion.
    pub(crate) fn observe(&mut self, at: Timestamp, pose: Pose2) {
        self.samples = self.samples.saturating_add(1);
        if !pose.valid() || self.previous.is_some_and(|(old, _)| at < old) {
            self.trajectory_valid = false;
            self.failure_reason
                .get_or_insert("invalid or regressing actual trajectory");
            return;
        }
        if self
            .passages
            .iter()
            .flatten()
            .any(|p| crate::simulation::obstacle_clearance(pose, self.footprint, p.cone) <= 0.0)
        {
            self.trajectory_valid = false;
            self.failure_reason
                .get_or_insert("actual body touched a true cone");
        }
        let Some((_, previous)) = self.previous.replace((at, pose)) else {
            return;
        };
        let right_already_passed =
            self.passages[0].is_some_and(|p| p.evidence.passed_in_required_order);
        for (index, slot) in self.passages.iter_mut().enumerate() {
            let Some(passage) = slot else {
                continue;
            };
            if passage.evidence.passed_at.is_some() {
                continue;
            }
            let direction = if index == 0 { 1.0 } else { -1.0 };
            let frame = Pose2 {
                x_m: passage.cone.center.x_m,
                y_m: passage.cone.center.y_m,
                yaw_rad: self.axis_rad,
            };
            let old = frame.world_to_body(previous.point());
            let current = frame.world_to_body(pose.point());
            let entry_heading = self.axis_rad
                + if index == 0 {
                    0.0
                } else {
                    std::f64::consts::PI
                };
            let exit_heading = self.axis_rad
                + if index == 0 {
                    std::f64::consts::PI
                } else {
                    0.0
                };
            let entering = direction * old.x_m <= 0.0 && direction * current.x_m > 0.0;
            let exiting = direction * old.x_m >= 0.0 && direction * current.x_m < 0.0;
            // These are rays through the true center, not coordinate waypoints:
            // below→outside→above is valid at any collision-free orbit radius.
            if entering {
                let crossing = interpolate_gate(previous, pose, old.x_m, current.x_m);
                let heading_error = angle_delta(crossing.yaw_rad - entry_heading).abs();
                let fully_below = self
                    .footprint
                    .corners(crossing)
                    .into_iter()
                    .all(|p| frame.world_to_body(p).y_m <= -passage.cone.radius_m);
                if fully_below && heading_error <= self.heading_tolerance_rad {
                    passage.active = true;
                    passage.previous_angle = -std::f64::consts::FRAC_PI_2;
                    passage.evidence.entered_at = Some(at);
                    passage.evidence.outer_side_at = None;
                    passage.evidence.angular_progress_rad = 0.0;
                    passage.evidence.entry_heading_error_rad = Some(heading_error);
                    passage.evidence.exit_heading_error_rad = None;
                    passage.evidence.began_in_required_order = index == 0 || right_already_passed;
                }
            }
            if !passage.active {
                continue;
            }
            let crossing = exiting.then(|| interpolate_gate(previous, pose, old.x_m, current.x_m));
            let valid_exit = crossing.filter(|crossing| {
                self.footprint
                    .corners(*crossing)
                    .into_iter()
                    .all(|p| frame.world_to_body(p).y_m >= passage.cone.radius_m)
                    && angle_delta(crossing.yaw_rad - exit_heading).abs()
                        <= self.heading_tolerance_rad
            });
            let endpoint = valid_exit.unwrap_or(pose);
            let relative = frame.world_to_body(endpoint.point());
            let angle = relative.y_m.atan2(relative.x_m);
            let delta = angle_delta(angle - passage.previous_angle);
            if delta.abs() > std::f64::consts::FRAC_PI_2 {
                self.trajectory_valid = false;
                self.failure_reason
                    .get_or_insert("actual cone progress skipped an unobserved quadrant");
                passage.active = false;
                continue;
            }
            passage.evidence.angular_progress_rad += direction * delta;
            passage.previous_angle = angle;
            let fully_outside = self
                .footprint
                .corners(pose)
                .into_iter()
                .all(|p| direction * frame.world_to_body(p).x_m >= passage.cone.radius_m);
            if fully_outside {
                passage.evidence.outer_side_at.get_or_insert(at);
            }
            if let Some(exit) = valid_exit {
                passage.evidence.exit_heading_error_rad =
                    Some(angle_delta(exit.yaw_rad - exit_heading).abs());
                if passage.evidence.outer_side_at.is_some()
                    && passage.evidence.angular_progress_rad + 1e-9 >= std::f64::consts::PI
                {
                    passage.evidence.passed_at = Some(at);
                    passage.evidence.passed_in_required_order =
                        passage.evidence.began_in_required_order;
                    if !passage.evidence.began_in_required_order {
                        self.order_valid = false;
                        self.failure_reason.get_or_insert(
                            "left cone orbit began before the right cone actually passed",
                        );
                    }
                }
                passage.active = false;
            }
        }
    }

    pub(crate) fn summary(&self) -> OnlineRefereeSummary {
        let completed = self
            .passages
            .iter()
            .flatten()
            .filter(|p| p.evidence.passed_in_required_order)
            .count();
        let available = self.passages.iter().flatten().count();
        OnlineRefereeSummary {
            available_cones: available,
            actual_pose_samples: self.samples,
            completed_cones: completed,
            both_completed: available == 2
                && completed == 2
                && self.order_valid
                && self.trajectory_valid,
            order_valid: self.order_valid,
            trajectory_valid: self.trajectory_valid,
            failure_reason: self.failure_reason,
            cones: self.passages.map(|p| p.map(|p| p.evidence)),
        }
    }

    pub(crate) fn completion_error(&self) -> Option<&'static str> {
        if self.summary().both_completed {
            None
        } else {
            Some(
                self.failure_reason
                    .unwrap_or("online completion lacked actual ordered right/left cone passage"),
            )
        }
    }
}

fn angle_delta(angle: f64) -> f64 {
    (angle + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}
fn interpolate_gate(from: Pose2, to: Pose2, old_x: f64, new_x: f64) -> Pose2 {
    let fraction = -old_x / (new_x - old_x);
    Pose2 {
        x_m: from.x_m + fraction * (to.x_m - from.x_m),
        y_m: from.y_m + fraction * (to.y_m - from.y_m),
        yaw_rad: from.yaw_rad + fraction * angle_delta(to.yaw_rad - from.yaw_rad),
    }
}

#[cfg(test)]
mod referee_tests {
    use super::*;
    use std::f64::consts::{FRAC_PI_2, PI};
    use xt_stcar_robot_core::autonomy::{Footprint, ObstacleDisc, Point2};

    #[test]
    fn marker_direction_uses_measured_body_yaw_and_fixed_task_axis() {
        let config = OnlineScenario::example().compile().unwrap();
        let pose = Pose2 {
            x_m: 6.5,
            y_m: 2.0,
            yaw_rad: 2.0,
        };
        let at = Timestamp(100);
        let image = crate::simulation::render_camera_at(
            &config,
            pose,
            xt_stcar_robot_core::autonomy::LightState::Unknown,
            at,
        )
        .unwrap();
        let detector = RoadDetector::new(config.road.clone()).unwrap();
        let observed = detect_frame(
            &config,
            &detector,
            &image,
            at,
            config.initial_pose.yaw_rad - pose.yaw_rad,
        )
        .unwrap();
        let line = observed
            .elements
            .unwrap()
            .observations
            .into_iter()
            .find(|o| o.kind == xt_stcar_robot_core::local_world::ElementKind::StopLine)
            .unwrap();
        assert!(angle_delta(line.heading_body_rad.unwrap() + pose.yaw_rad).abs() < 0.1);
        let near = pose.body_to_world(line.position_body_m);
        assert!((near.x_m - 5.2).abs() < 0.15, "{near:?}");
        assert!((near.y_m - 4.4).abs() < 0.15, "{near:?}");
    }
    fn cones() -> [ObstacleDisc; 2] {
        [
            ObstacleDisc {
                center: Point2 { x_m: 4.0, y_m: 2.0 },
                radius_m: 0.2,
            },
            ObstacleDisc {
                center: Point2 { x_m: 1.0, y_m: 2.0 },
                radius_m: 0.2,
            },
        ]
    }
    fn referee(truth: &[ObstacleDisc]) -> OnlineReferee {
        OnlineReferee::new(
            truth,
            Pose2 {
                x_m: 0.0,
                y_m: 0.5,
                yaw_rad: 0.0,
            },
            Footprint {
                front_m: 0.22,
                rear_m: 0.18,
                half_width_m: 0.13,
            },
            0.12,
        )
        .unwrap()
    }
    fn arc(referee: &mut OnlineReferee, index: usize, direction: f64, radius: f64, start: u64) {
        let cone = cones()[index];
        for step in 0..=340 {
            let progress = -0.12 + (PI + 0.24) * step as f64 / 340.0;
            let angle = -FRAC_PI_2 + direction * progress;
            referee.observe(
                Timestamp(start + step),
                Pose2 {
                    x_m: cone.center.x_m + radius * angle.cos(),
                    y_m: cone.center.y_m + radius * angle.sin(),
                    yaw_rad: angle + direction * FRAC_PI_2,
                },
            );
        }
    }
    #[test]
    fn actual_ordered_outer_semicircles_pass_at_different_radii() {
        for (right_radius, left_radius) in [(0.6, 0.8), (1.1, 0.65)] {
            let mut r = referee(&cones());
            arc(&mut r, 0, 1.0, right_radius, 100);
            assert_eq!(r.summary().completed_cones, 1, "{:?}", r.summary());
            arc(&mut r, 1, -1.0, left_radius, 1000);
            let evidence = r.summary();
            assert!(evidence.both_completed, "{evidence:?}");
            assert!(
                evidence
                    .cones
                    .iter()
                    .flatten()
                    .all(|c| (c.angular_progress_rad - PI).abs() < 1e-9)
            );
            assert!(evidence.cones[0].unwrap().passed_at < evidence.cones[1].unwrap().entered_at);
            assert!(r.completion_error().is_none());
        }
    }
    #[test]
    fn wrong_side_and_wrong_order_cannot_be_reported_as_completed() {
        let mut wrong_side = referee(&cones());
        arc(&mut wrong_side, 0, -1.0, 0.8, 100);
        arc(&mut wrong_side, 1, 1.0, 0.8, 1000);
        assert_eq!(wrong_side.summary().completed_cones, 0);
        assert!(wrong_side.completion_error().is_some());
        let mut wrong_order = referee(&cones());
        arc(&mut wrong_order, 1, -1.0, 0.8, 100);
        arc(&mut wrong_order, 0, 1.0, 0.8, 1000);
        assert!(!wrong_order.summary().order_valid);
        assert!(!wrong_order.summary().both_completed);
        assert!(wrong_order.completion_error().is_some());
    }
    #[test]
    fn fake_completion_missing_cones_and_heading_mismatch_have_no_fabricated_passage() {
        let duplicate = [cones()[0], cones()[0]];
        assert!(
            OnlineReferee::new(
                &duplicate,
                Pose2::default(),
                Footprint {
                    front_m: 0.22,
                    rear_m: 0.18,
                    half_width_m: 0.13
                },
                0.12
            )
            .is_err()
        );
        let absent = referee(&[]);
        assert_eq!(absent.summary().available_cones, 0);
        assert_eq!(absent.summary().completed_cones, 0);
        assert!(absent.summary().cones.iter().all(Option::is_none));
        assert!(absent.completion_error().is_some());
        let mut one = referee(&cones()[..1]);
        arc(&mut one, 0, 1.0, 0.8, 100);
        assert_eq!(one.summary().available_cones, 1);
        assert_eq!(one.summary().completed_cones, 1);
        assert!(one.summary().cones[1].is_none());
        assert!(one.completion_error().is_some());
        let mut r = referee(&cones());
        assert!(r.completion_error().is_some());
        let cone = cones()[0];
        for step in 0..=340 {
            let angle = -FRAC_PI_2 - 0.12 + (PI + 0.24) * step as f64 / 340.0;
            r.observe(
                Timestamp(step + 100),
                Pose2 {
                    x_m: cone.center.x_m + 0.8 * angle.cos(),
                    y_m: cone.center.y_m + 0.8 * angle.sin(),
                    yaw_rad: FRAC_PI_2,
                },
            );
        }
        assert_eq!(r.summary().completed_cones, 0);
        assert!(r.completion_error().is_some());
    }
    #[test]
    fn angular_backtracking_subtracts_instead_of_ratchet_counting() {
        let mut r = referee(&cones());
        let center = cones()[0].center;
        for (index, progress) in [-0.1, 0.1, 0.3, 0.1, 0.3].into_iter().enumerate() {
            let angle = -FRAC_PI_2 + progress;
            r.observe(
                Timestamp(100 + index as u64),
                Pose2 {
                    x_m: center.x_m + 0.8 * angle.cos(),
                    y_m: center.y_m + 0.8 * angle.sin(),
                    yaw_rad: progress,
                },
            );
        }
        let summary = r.summary();
        assert!((summary.cones[0].unwrap().angular_progress_rad - 0.3).abs() < 1e-12);
        assert_eq!(summary.completed_cones, 0);
        assert!(r.completion_error().is_some());
    }
}
