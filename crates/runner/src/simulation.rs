//! Synthetic camera + laser + Ackermann plant feedback. No recorded motion intents.
//! Ideal pose feedback is declared explicitly; it is not encoder or localization evidence.
use crate::autonomy::{AutonomyConfig, AutonomyController, Result, RoadFrame};
use crate::telemetry::{RunJournal, TelemetryConfig, TelemetryMode};
use image::{Rgb, RgbImage};
use serde::{Deserialize, Serialize};
use std::f64::consts::TAU;
use std::io::Write;
use xt_stcar_robot_core::autonomy::{
    Footprint, HalfPlane, LightState, ObstacleDisc, Point2, Pose2, PoseEstimate, Rect,
};
use xt_stcar_robot_core::mission::{MissionConfig, MissionPhase};
use xt_stcar_robot_core::navigation::NavigationConfig;
use xt_stcar_robot_core::scan::ScanConfig;
use xt_stcar_robot_core::{
    FrameId, LidarSample, MotionOutput, SafetyConfig, SensorFrames, SensorKind, Timestamp,
};
use xt_stcar_vision::road::{RoadConfig, RoadDetector};

#[derive(Clone, Copy, Debug, Default, Deserialize, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum SimulationFault {
    #[default]
    None,
    CameraDropout,
    LidarDropout,
    PoseDropout,
    Blocked,
    RedOnly,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct SimulationConfig {
    #[serde(default)]
    pub telemetry: TelemetryConfig,
    pub autonomy: AutonomyConfig,
    pub road: RoadConfig,
    pub initial_pose: Pose2,
    pub crosswalk: Rect,
    pub cones: Vec<ObstacleDisc>,
    pub light_trigger_x_m: f64,
    pub light_red_duration_ms: u64,
    pub time_step_ms: u64,
    pub max_duration_ms: u64,
    pub fault_at_ms: u64,
    pub fault: SimulationFault,
    pub plant_accel_mps2: f64,
    pub plant_brake_mps2: f64,
}

impl SimulationConfig {
    pub fn example() -> Self {
        let footprint = Footprint {
            front_m: 0.22,
            rear_m: 0.18,
            half_width_m: 0.13,
        };
        let bounds = Rect {
            min_x_m: 0.0,
            min_y_m: 0.0,
            max_x_m: 7.0,
            max_y_m: 5.0,
        };
        let world_frame = FrameId("sim_world".into());
        let body_frame = FrameId("sim_body".into());
        let laser_frame = FrameId("sim_laser".into());
        let mut navigation = NavigationConfig::simulation(bounds, footprint, world_frame.clone());
        navigation.control_period_ms = 100;
        navigation.goal_tolerance_m = 0.045;
        let mission = MissionConfig {
            schema_version: 1,
            simulation_only: true,
            world_frame: world_frame.clone(),
            body_frame: body_frame.clone(),
            footprint,
            approach_goal: Point2 { x_m: 2.0, y_m: 2.5 },
            crosswalk_region: Rect {
                min_x_m: 1.0,
                max_x_m: 2.5,
                min_y_m: 1.0,
                max_y_m: 4.0,
            },
            cone_waypoints: vec![
                Point2 {
                    x_m: 3.1,
                    y_m: 3.35,
                },
                Point2 {
                    x_m: 4.5,
                    y_m: 1.85,
                },
            ],
            light_detection_region: Rect {
                min_x_m: 4.4,
                max_x_m: 6.0,
                min_y_m: 1.5,
                max_y_m: 3.5,
            },
            light_stop_region: Rect {
                min_x_m: 5.0,
                max_x_m: 6.0,
                min_y_m: 1.9,
                max_y_m: 3.1,
            },
            light_stop_goal: Point2 {
                x_m: 5.55,
                y_m: 2.5,
            },
            light_approach_yaw_rad: 0.0,
            finish_region: Rect {
                min_x_m: 6.0,
                max_x_m: 6.8,
                min_y_m: 1.9,
                max_y_m: 3.1,
            },
            finish_goal: Point2 {
                x_m: 6.43,
                y_m: 2.5,
            },
            finish_yaw_rad: 0.0,
            goal_heading_tolerance_rad: 0.12,
            cruise_speed_mps: 0.3,
            approach_speed_mps: 0.18,
            goal_tolerance_m: 0.065,
            crosswalk_stop_margin_m: 0.15,
            stopped_speed_mps: 0.012,
            crosswalk_hold_ms: 3000,
            min_green_ms: 300,
            min_green_frames: 3,
            max_pose_age_ms: 250,
            max_road_age_ms: 250,
            max_pose_gap_ms: 250,
            max_road_gap_ms: 250,
            max_observation_skew_ms: 100,
            min_pose_quality: 0.6,
            min_detection_confidence: 0.5,
        };
        let safety = SafetyConfig {
            schema_version: 1,
            simulation_only: true,
            max_speed_mps: navigation.max_speed_mps,
            max_abs_curvature_per_m: navigation.max_curvature_per_m,
            heartbeat_timeout_ms: 300,
            deadman_timeout_ms: 300,
            command_timeout_ms: 300,
            sensor_timeout_ms: 300,
            required_sensors: vec![SensorKind::Lidar, SensorKind::Odometry, SensorKind::Vision],
            frames: SensorFrames {
                imu_frame: FrameId("sim_imu".into()),
                lidar_frame: laser_frame.clone(),
                odometry_frame: world_frame,
                body_frame,
                vision_frame: FrameId("sim_camera".into()),
            },
        };
        let autonomy = AutonomyConfig {
            schema_version: 1,
            simulation_only: true,
            measurement_status: "unverified".into(),
            safety,
            mission,
            navigation,
            scan: ScanConfig {
                frame_id: laser_frame,
                bins: 360,
                min_coverage_fraction: 0.95,
                min_valid_fraction: 0.95,
                max_missing_arc_rad: 0.12,
                max_revolution_ms: 200,
                max_packet_gap_ms: 20,
            },
            lidar_in_body: Pose2::default(),
            cone_radius_m: 0.14,
            laser_point_radius_m: 0.015,
            max_sensor_age_ms: 250,
            max_tick_gap_ms: 250,
        };
        let mut road = RoadConfig::simulation();
        road.light_rois = vec![[0.82, 0.02, 0.96, 0.25]];
        Self {
            telemetry: TelemetryConfig::default(),
            autonomy,
            road,
            initial_pose: Pose2 {
                x_m: 0.6,
                y_m: 2.5,
                yaw_rad: 0.0,
            },
            crosswalk: Rect {
                min_x_m: 1.8,
                max_x_m: 2.097,
                min_y_m: 1.555,
                max_y_m: 3.445,
            },
            cones: vec![
                ObstacleDisc {
                    center: Point2 { x_m: 3.0, y_m: 2.5 },
                    radius_m: 0.14,
                },
                ObstacleDisc {
                    center: Point2 {
                        x_m: 4.25,
                        y_m: 2.85,
                    },
                    radius_m: 0.14,
                },
            ],
            light_trigger_x_m: 5.0,
            light_red_duration_ms: 5000,
            time_step_ms: 100,
            max_duration_ms: 120_000,
            fault_at_ms: 2000,
            fault: SimulationFault::None,
            plant_accel_mps2: 0.6,
            plant_brake_mps2: 0.8,
        }
    }

    pub fn validate(&self) -> Result<()> {
        AutonomyController::new(self.autonomy.clone())?;
        RunJournal::new(self.telemetry.clone())?;
        RoadDetector::new(self.road.clone())?;
        self.crosswalk.validate().map_err(|e| e.to_string())?;
        if !self.initial_pose.valid()
            || !self
                .autonomy
                .mission
                .footprint
                .inside(self.initial_pose, self.autonomy.navigation.bounds)
            || self.cones.len() != 2
            || self.cones.iter().any(|c| {
                !c.center.valid() || !c.radius_m.is_finite() || !(0.05..=0.3).contains(&c.radius_m)
            })
            || !self.light_trigger_x_m.is_finite()
            || !(3000..=10_000).contains(&self.light_red_duration_ms)
            || !(20..=100).contains(&self.time_step_ms)
            || self.time_step_ms != self.autonomy.navigation.control_period_ms
            || self.max_duration_ms < self.time_step_ms
            || self.max_duration_ms > 180_000
            || ![self.plant_accel_mps2, self.plant_brake_mps2]
                .iter()
                .all(|v| v.is_finite() && (0.05..=5.0).contains(v))
            || self.plant_brake_mps2 < self.autonomy.navigation.max_decel_mps2
            || self.road.light_rois.len() != 1
        {
            return Err("invalid synthetic scenario geometry, timing, plant or light ROI".into());
        }
        inverse(self.road.homography.matrix)?;
        if self.cones.iter().any(|cone| {
            obstacle_clearance(self.initial_pose, self.autonomy.mission.footprint, *cone) <= 0.0
        }) {
            return Err("synthetic initial pose overlaps a cone".into());
        }
        Ok(())
    }
}

fn inverse(a: [[f64; 3]; 3]) -> Result<[[f64; 3]; 3]> {
    let det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
    if !det.is_finite() || det.abs() < 1e-12 {
        return Err("synthetic camera homography is singular".into());
    }
    let mut b = [[0.0; 3]; 3];
    for (i, row) in b.iter_mut().enumerate() {
        for (j, value) in row.iter_mut().enumerate() {
            *value = (a[(j + 1) % 3][(i + 1) % 3] * a[(j + 2) % 3][(i + 2) % 3]
                - a[(j + 1) % 3][(i + 2) % 3] * a[(j + 2) % 3][(i + 1) % 3])
                / det;
        }
    }
    Ok(b)
}

fn project(matrix: [[f64; 3]; 3], x: f64, y: f64) -> Option<Point2> {
    let d = matrix[2][0] * x + matrix[2][1] * y + matrix[2][2];
    if d.abs() < 1e-9 {
        return None;
    }
    let p = Point2 {
        x_m: (matrix[0][0] * x + matrix[0][1] * y + matrix[0][2]) / d,
        y_m: (matrix[1][0] * x + matrix[1][1] * y + matrix[1][2]) / d,
    };
    p.valid().then_some(p)
}

pub fn render_camera(
    config: &SimulationConfig,
    pose: Pose2,
    light: LightState,
) -> Result<RgbImage> {
    let (width, height) = (320, 240);
    let mut image = RgbImage::from_pixel(width, height, Rgb([35, 35, 35]));
    for (x, y, pixel) in image.enumerate_pixels_mut() {
        let Some(body) = project(
            config.road.homography.matrix,
            f64::from(x) / f64::from(width),
            f64::from(y) / f64::from(height),
        ) else {
            continue;
        };
        let world = pose.body_to_world(body);
        if config.crosswalk.contains(world)
            && (world.y_m - config.crosswalk.min_y_m).rem_euclid(0.21) < 0.105
        {
            *pixel = Rgb([245, 245, 245]);
        }
    }
    let inv = inverse(config.road.homography.matrix)?;
    for (index, cone) in config.cones.iter().enumerate() {
        let body = pose.world_to_body(cone.center);
        if body.x_m <= 0.0 {
            continue;
        }
        let Some(uv) = project(inv, body.x_m, body.y_m) else {
            continue;
        };
        if !(-0.1..=1.1).contains(&uv.x_m) || !(0.0..=1.2).contains(&uv.y_m) {
            continue;
        }
        let cx = (uv.x_m * f64::from(width)).round() as i32;
        let bottom = (uv.y_m * f64::from(height)).round() as i32;
        for dy in 0..30 {
            let half = dy * 11 / 30;
            for dx in -half..=half {
                let (x, y) = (cx + dx, bottom - 30 + dy);
                if x >= 0 && x < width as i32 && y >= 0 && y < height as i32 {
                    image.put_pixel(
                        x as u32,
                        y as u32,
                        if index == 0 {
                            Rgb([230, 15, 15])
                        } else {
                            Rgb([15, 35, 230])
                        },
                    );
                }
            }
        }
    }
    let roi = config.road.light_rois[0];
    let (left, top, right, bottom) = (
        (roi[0] * 320.0) as u32,
        (roi[1] * 240.0) as u32,
        (roi[2] * 320.0) as u32,
        (roi[3] * 240.0) as u32,
    );
    for y in top..bottom {
        for x in left..right {
            image.put_pixel(x, y, Rgb([0, 0, 0]));
        }
    }
    let (cx, cy) = (((left + right) / 2) as i32, ((top + bottom) / 2) as i32);
    let color = match light {
        LightState::Red => [240, 5, 5],
        LightState::Yellow => [240, 210, 0],
        LightState::Green => [5, 240, 5],
        _ => [0, 0, 0],
    };
    let radius = (((right - left).min(bottom - top)) / 4).min(7) as i32;
    for dy in -radius..=radius {
        for dx in -radius..=radius {
            let (x, y) = (cx + dx, cy + dy);
            if dx * dx + dy * dy <= radius * radius
                && x >= left as i32
                && x < right as i32
                && y >= top as i32
                && y < bottom as i32
            {
                image.put_pixel(x as u32, y as u32, Rgb(color));
            }
        }
    }
    Ok(image)
}

pub fn synthetic_scan(
    config: &SimulationConfig,
    pose: Pose2,
    obstacles: &[ObstacleDisc],
    at: Timestamp,
) -> LidarSample {
    let bounds = config.autonomy.navigation.bounds;
    let origin = pose.body_to_world(config.autonomy.lidar_in_body.point());
    let increment = -TAU / config.autonomy.scan.bins as f64;
    let ranges = (0..config.autonomy.scan.bins)
        .map(|index| {
            let a = pose.yaw_rad + config.autonomy.lidar_in_body.yaw_rad + index as f64 * increment;
            let (dy, dx) = a.sin_cos();
            let mut distance = 12.0f64;
            for (boundary, position, direction) in [
                (bounds.min_x_m, origin.x_m, dx),
                (bounds.max_x_m, origin.x_m, dx),
                (bounds.min_y_m, origin.y_m, dy),
                (bounds.max_y_m, origin.y_m, dy),
            ] {
                if direction.abs() > 1e-9 {
                    let t = (boundary - position) / direction;
                    if t > 0.0 {
                        distance = distance.min(t);
                    }
                }
            }
            for obstacle in obstacles {
                let ox = origin.x_m - obstacle.center.x_m;
                let oy = origin.y_m - obstacle.center.y_m;
                let b = ox * dx + oy * dy;
                let discriminant =
                    b * b - (ox * ox + oy * oy - obstacle.radius_m * obstacle.radius_m);
                if discriminant >= 0.0 {
                    let t = -b - discriminant.sqrt();
                    if t > 0.0 {
                        distance = distance.min(t);
                    }
                }
            }
            Some(distance)
        })
        .collect();
    LidarSample {
        captured_at: at,
        frame_id: config.autonomy.scan.frame_id.clone(),
        angle_min_rad: 0.0,
        angle_increment_rad: increment,
        range_min_m: 0.02,
        range_max_m: 12.0,
        ranges_m: ranges,
    }
}

pub fn obstacle_clearance(pose: Pose2, footprint: Footprint, obstacle: ObstacleDisc) -> f64 {
    let p = pose.world_to_body(obstacle.center);
    let dx = p.x_m - p.x_m.clamp(-footprint.rear_m, footprint.front_m);
    let dy = p.y_m - p.y_m.clamp(-footprint.half_width_m, footprint.half_width_m);
    dx.hypot(dy) - obstacle.radius_m
}

#[derive(Debug, Serialize)]
pub struct SimulationSummary {
    pub kind: &'static str,
    pub physical_output_enabled: bool,
    pub scene: &'static str,
    pub pose_source: &'static str,
    pub completed: bool,
    pub elapsed_ms: u64,
    pub ticks: usize,
    pub distance_m: f64,
    pub minimum_cone_clearance_m: f64,
    pub crosswalk_hold_ms: u64,
    pub green_observed_ms: u64,
    pub phases: Vec<MissionPhase>,
    pub final_pose: Pose2,
    pub final_actual_speed_mps: f64,
    pub braking_ticks: usize,
    pub dropped_trace_records: u64,
    pub logging_errors: usize,
    pub fault: Option<String>,
    pub first_navigation_failure: Option<crate::navigation_diagnostics::NavigationFailureWindow>,
}

pub fn simulate(config: &SimulationConfig, writer: &mut impl Write) -> Result<SimulationSummary> {
    config.validate()?;
    let mut journal = RunJournal::new(config.telemetry.clone())?;
    let mut navigation_failure = crate::navigation_diagnostics::FirstNavigationFailure::default();
    let mut logging_errors = 0;
    let detector = RoadDetector::new(config.road.clone())?;
    let mut controller = AutonomyController::new(config.autonomy.clone())?;
    controller.start()?;
    let mut pose = config.initial_pose;
    let mut speed = 0.0f64;
    let mut curvature = 0.0f64;
    let mut trigger = None;
    let mut previous_pose = None;
    let mut previous_scan = None;
    let mut previous_road = None;
    let mut phases = Vec::new();
    let mut held = 0;
    let mut green = 0;
    let mut distance = 0.0;
    let mut minimum = config
        .cones
        .iter()
        .map(|cone| obstacle_clearance(pose, config.autonomy.mission.footprint, *cone))
        .fold(12.0f64, f64::min);
    let mut ticks = 0;
    let dt = config.time_step_ms as f64 / 1000.0;
    let dynamics = PlantDynamics {
        acceleration_mps2: config.plant_accel_mps2,
        braking_mps2: config.plant_brake_mps2,
        curvature_rate_per_s: config.autonomy.navigation.max_curvature_rate_per_s,
    };
    let mut summary_fault = None;
    let mut completed = false;
    let mut elapsed = 0;
    let light_boundary = config
        .autonomy
        .mission
        .light_stop_boundary()
        .map_err(|e| e.to_string())?;
    let mut active_boundary = None;
    for ms in (0..=config.max_duration_ms).step_by(config.time_step_ms as usize) {
        elapsed = ms;
        let at = Timestamp(ms);
        if trigger.is_none()
            && pose.x_m + config.autonomy.mission.footprint.front_m >= config.light_trigger_x_m
        {
            trigger = Some(ms);
        }
        let light = if config.fault == SimulationFault::RedOnly
            || trigger.is_some_and(|start| ms - start < config.light_red_duration_ms)
        {
            LightState::Red
        } else {
            LightState::Green
        };
        let mut obstacles = config.cones.clone();
        if config.fault == SimulationFault::Blocked {
            obstacles.extend((0..26).map(|i| ObstacleDisc {
                center: Point2 {
                    x_m: 2.65,
                    y_m: i as f64 * 0.2,
                },
                radius_m: 0.22,
            }));
        }
        let fresh_pose = PoseEstimate {
            captured_at: at,
            frame_id: config.autonomy.mission.world_frame.clone(),
            pose,
            speed_mps: speed,
            yaw_rate_radps: speed * curvature,
            quality: 1.0,
        };
        let fresh_scan = synthetic_scan(config, pose, &obstacles, at);
        // A runtime perception error must enter the same stop/braking path as a
        // dropped sensor. Only startup/configuration failures may return early.
        let fresh_road = render_camera(config, pose, light).and_then(|camera| {
            Ok(RoadFrame {
                observation: detector.detect(
                    &camera,
                    &[],
                    at,
                    config.autonomy.mission.body_frame.clone(),
                )?,
                image_width_px: camera.width(),
                image_height_px: camera.height(),
            })
        });
        if ms < config.fault_at_ms || config.fault != SimulationFault::PoseDropout {
            previous_pose = Some(fresh_pose);
        }
        if ms < config.fault_at_ms || config.fault != SimulationFault::LidarDropout {
            previous_scan = Some(fresh_scan);
        }
        let step = match fresh_road {
            Ok(road) => {
                if ms < config.fault_at_ms || config.fault != SimulationFault::CameraDropout {
                    previous_road = Some(road);
                }
                match (&previous_pose, &previous_scan, &previous_road) {
                    (Some(p), Some(s), Some(r)) => controller.tick(at, p, s, r),
                    _ => controller.stop_with_fault(at, "synthetic sensor unavailable".into()),
                }
            }
            Err(error) => {
                controller.stop_with_fault(at, format!("runtime road perception failed: {error}"))
            }
        };
        ticks += 1;
        let mut phase_changed = false;
        navigation_failure.observe(crate::navigation_diagnostics::NavigationFrame::from_step(
            at, pose, speed, &step,
        ));
        if let Some(mission) = &step.mission {
            // Cones can preview the light approach. Keep the same boundary as
            // navigation during that phase, and preserve it while braking from
            // a fault. Confirmed admission into Finish releases the constraint.
            active_boundary = boundary_for_phase(active_boundary, mission.phase, light_boundary);
            if phases.last() != Some(&mission.phase) {
                phases.push(mission.phase);
                phase_changed = true;
            }
            held = held.max(mission.crosswalk_stop_elapsed_ms);
            green = green.max(mission.green_elapsed_ms);
            completed = mission.phase == MissionPhase::Completed
                && step.fault.is_none()
                && step.safety.state == xt_stcar_robot_core::State::Running;
        }
        let important = step.fault.is_some()
            || (phase_changed && config.telemetry.mode != TelemetryMode::Summary);
        if (important || config.telemetry.mode==TelemetryMode::Trace) && journal.record(&serde_json::json!({"control":step,"pose_feedback":pose,"actual_speed_mps":speed,
            "simulated_light":light,"road_frame":previous_road}),important).is_err() {logging_errors+=1;}
        if step.fault.is_some() || completed {
            summary_fault = step.fault;
            break;
        }
        if ms >= config.max_duration_ms {
            break;
        }
        let mut plant_violation = false;
        let plant = advance_plant(
            PlantState {
                pose,
                speed_mps: speed,
                curvature_per_m: curvature,
            },
            &step.command,
            dynamics,
            dt,
            |sample| {
                plant_violation |=
                    check_plant_pose(config, sample, &obstacles, active_boundary, &mut minimum);
            },
        );
        pose = plant.state.pose;
        speed = plant.state.speed_mps;
        curvature = plant.state.curvature_per_m;
        elapsed = ms + config.time_step_ms;
        distance += plant.distance_m;
        if plant_violation {
            summary_fault = Some("synthetic plant collision or boundary violation".into());
            break;
        }
    }
    if !completed && summary_fault.is_none() {
        summary_fault = Some("scenario duration exhausted before completion".into());
    }
    // Fault output is a stop request. Continue the synthetic plant until it has
    // actually decelerated, checking the swept poses instead of equating command and stop.
    let mut braking_ticks = 0;
    while speed.abs() > 1e-9 && braking_ticks < 5000 {
        braking_ticks += 1;
        elapsed += config.time_step_ms;
        let mut obstacles = config.cones.clone();
        if config.fault == SimulationFault::Blocked {
            obstacles.extend((0..26).map(|i| ObstacleDisc {
                center: Point2 {
                    x_m: 2.65,
                    y_m: i as f64 * 0.2,
                },
                radius_m: 0.22,
            }));
        }
        let mut plant_violation = false;
        let plant = advance_plant(
            PlantState {
                pose,
                speed_mps: speed,
                curvature_per_m: curvature,
            },
            &MotionOutput::Stop,
            dynamics,
            dt,
            |sample| {
                plant_violation |=
                    check_plant_pose(config, sample, &obstacles, active_boundary, &mut minimum);
            },
        );
        pose = plant.state.pose;
        speed = plant.state.speed_mps;
        curvature = plant.state.curvature_per_m;
        distance += plant.distance_m;
        if plant_violation {
            completed = false;
            summary_fault = Some("synthetic braking collision or boundary violation".into());
        }
        if journal
            .record(
                &serde_json::json!({"kind":"simulated_braking_step","physical_output_enabled":false,
            "at":elapsed,"command":{"type":"stop"},"pose_feedback":pose,"actual_speed_mps":speed}),
                false,
            )
            .is_err()
        {
            logging_errors += 1;
        }
    }
    if speed.abs() > 1e-9 {
        completed = false;
        summary_fault =
            Some("synthetic plant failed to stop within its bounded braking rollout".into());
    }
    if summary_fault.is_some() && phases.last() != Some(&MissionPhase::Fault) {
        phases.push(MissionPhase::Fault);
    }
    journal.write_to(writer).map_err(|e| e.to_string())?;
    // Every terminal path emits an explicit stop record even if the caller stops consuming steps.
    let terminal = serde_json::json!({"kind":"autonomy_terminal","physical_output_enabled":false,"at":elapsed,"command":{"type":"stop"}});
    serde_json::to_writer(&mut *writer, &terminal).map_err(|e| e.to_string())?;
    writer.write_all(b"\n").map_err(|e| e.to_string())?;
    let summary = SimulationSummary {
        kind: "simulation_summary",
        physical_output_enabled: false,
        scene: "synthetic RGB, raycast laser and bounded Ackermann plant",
        pose_source: "ideal simulated pose feedback; no encoder/localization accuracy claim",
        completed,
        elapsed_ms: elapsed,
        ticks,
        distance_m: distance,
        minimum_cone_clearance_m: minimum,
        crosswalk_hold_ms: held,
        green_observed_ms: green,
        phases,
        final_pose: pose,
        final_actual_speed_mps: speed,
        braking_ticks,
        fault: summary_fault,
        first_navigation_failure: navigation_failure.finish(),
        dropped_trace_records: journal.dropped_records(),
        logging_errors,
    };
    serde_json::to_writer(&mut *writer, &summary).map_err(|e| e.to_string())?;
    writer.write_all(b"\n").map_err(|e| e.to_string())?;
    Ok(summary)
}

fn boundary_for_phase(
    previous: Option<HalfPlane>,
    phase: MissionPhase,
    light_boundary: HalfPlane,
) -> Option<HalfPlane> {
    match phase {
        MissionPhase::Cones | MissionPhase::ApproachLight | MissionPhase::WaitGreen => {
            Some(light_boundary)
        }
        MissionPhase::Fault => previous,
        _ => None,
    }
}

fn check_plant_pose(
    config: &SimulationConfig,
    pose: Pose2,
    obstacles: &[ObstacleDisc],
    travel_boundary: Option<HalfPlane>,
    minimum: &mut f64,
) -> bool {
    for obstacle in obstacles {
        *minimum = minimum.min(obstacle_clearance(
            pose,
            config.autonomy.mission.footprint,
            *obstacle,
        ));
    }
    *minimum < 0.0
        || travel_boundary.is_some_and(|boundary| {
            !boundary.contains_footprint(config.autonomy.mission.footprint, pose, 0.0)
        })
        || !config
            .autonomy
            .mission
            .footprint
            .inside(pose, config.autonomy.navigation.bounds)
}

#[derive(Clone, Copy, Debug)]
struct PlantState {
    pose: Pose2,
    speed_mps: f64,
    curvature_per_m: f64,
}

#[derive(Clone, Copy)]
struct PlantDynamics {
    acceleration_mps2: f64,
    braking_mps2: f64,
    curvature_rate_per_s: f64,
}

struct PlantStep {
    state: PlantState,
    distance_m: f64,
}

/// One synthetic command interval. Speed and curvature change continuously at
/// the plant's rates; Stop selects zero targets and uses the same integration.
/// The caller checks the initial pose and every <=1 ms substep, including rate
/// saturation points. These samples do not assert measured actuator behavior or
/// prove collision freedom between samples.
fn advance_plant(
    mut state: PlantState,
    command: &MotionOutput,
    dynamics: PlantDynamics,
    dt: f64,
    mut observe: impl FnMut(Pose2),
) -> PlantStep {
    let (target_speed, target_curvature) = match command {
        MotionOutput::Stop => (0.0, 0.0),
        MotionOutput::Drive {
            speed_mps,
            curvature_per_m,
        } => (*speed_mps, *curvature_per_m),
    };
    // Runtime validation limits dt to 20..=100 ms, so there are at most 100
    // uniform intervals and two extra saturation splits per uniform interval.
    let substeps = (dt / 0.001).ceil() as usize;
    let mut distance_m = 0.0;
    observe(state.pose);
    for _ in 0..substeps {
        let mut remaining = dt / substeps as f64;
        for _ in 0..3 {
            if remaining <= 0.0 {
                break;
            }
            let acceleration = if target_speed > state.speed_mps {
                dynamics.acceleration_mps2
            } else if target_speed < state.speed_mps {
                -dynamics.braking_mps2
            } else {
                0.0
            };
            let curvature_rate = if target_curvature == state.curvature_per_m {
                0.0
            } else {
                (target_curvature - state.curvature_per_m).signum() * dynamics.curvature_rate_per_s
            };
            let speed_time = if acceleration == 0.0 {
                f64::INFINITY
            } else {
                (target_speed - state.speed_mps) / acceleration
            };
            let curvature_time = if curvature_rate == 0.0 {
                f64::INFINITY
            } else {
                (target_curvature - state.curvature_per_m) / curvature_rate
            };
            let h = remaining.min(speed_time).min(curvature_time);
            let v = state.speed_mps;
            let k = state.curvature_per_m;
            // On this segment v and k are linear. Integrating their product
            // exactly retains the a*k_rate cross term, unlike multiplying
            // endpoint values or separate time averages over the whole tick.
            let yaw_at = |t: f64| {
                state.pose.yaw_rad
                    + v * k * t
                    + (v * curvature_rate + acceleration * k) * t.powi(2) * 0.5
                    + acceleration * curvature_rate * t.powi(3) / 3.0
            };
            let middle_yaw = yaw_at(h * 0.5);
            let end_yaw = yaw_at(h);
            let middle_speed = v + acceleration * h * 0.5;
            let end_speed = v + acceleration * h;
            // Simpson integration of x/y over <=1 ms; a separate fine-step
            // RK4 reference in tests bounds the numerical position error.
            state.pose.x_m += h / 6.0
                * (v * state.pose.yaw_rad.cos()
                    + 4.0 * middle_speed * middle_yaw.cos()
                    + end_speed * end_yaw.cos());
            state.pose.y_m += h / 6.0
                * (v * state.pose.yaw_rad.sin()
                    + 4.0 * middle_speed * middle_yaw.sin()
                    + end_speed * end_yaw.sin());
            state.pose.yaw_rad =
                (end_yaw + std::f64::consts::PI).rem_euclid(TAU) - std::f64::consts::PI;
            distance_m += v * h + acceleration * h.powi(2) * 0.5;
            state.speed_mps = if h >= speed_time {
                target_speed
            } else {
                end_speed
            };
            state.curvature_per_m = if h >= curvature_time {
                target_curvature
            } else {
                k + curvature_rate * h
            };
            remaining -= h;
            observe(state.pose);
        }
    }
    PlantStep { state, distance_m }
}

#[cfg(test)]
mod stop_tests {
    use super::*;

    fn dynamics() -> PlantDynamics {
        PlantDynamics {
            acceleration_mps2: 0.6,
            braking_mps2: 0.8,
            curvature_rate_per_s: 4.0,
        }
    }

    #[test]
    fn stop_centers_gradually_and_keeps_zero_speed_while_steering_finishes() {
        for direction in [-1.0, 1.0] {
            let mut state = PlantState {
                pose: Pose2::default(),
                speed_mps: 0.3,
                curvature_per_m: 2.0 * direction,
            };
            state = advance_plant(state, &MotionOutput::Stop, dynamics(), 0.1, |_| {}).state;
            assert!((state.speed_mps - 0.22).abs() < 1e-12);
            assert!((state.curvature_per_m - 1.6 * direction).abs() < 1e-12);
            for _ in 0..3 {
                state = advance_plant(state, &MotionOutput::Stop, dynamics(), 0.1, |_| {}).state;
            }
            assert_eq!(state.speed_mps, 0.0);
            assert!(state.curvature_per_m.abs() > 0.0);
            let stationary_pose = state.pose;
            for _ in 0..3 {
                state = advance_plant(state, &MotionOutput::Stop, dynamics(), 0.1, |_| {}).state;
            }
            assert_eq!((state.speed_mps, state.curvature_per_m), (0.0, 0.0));
            assert_eq!(state.pose, stationary_pose);
        }
    }

    #[test]
    fn straight_stopping_distance_matches_energy_formula_at_each_control_period() {
        for dt in [0.02, 0.05, 0.1] {
            let mut state = PlantState {
                pose: Pose2::default(),
                speed_mps: 0.3,
                curvature_per_m: 0.0,
            };
            let mut distance = 0.0;
            for _ in 0..25 {
                let step = advance_plant(state, &MotionOutput::Stop, dynamics(), dt, |_| {});
                state = step.state;
                distance += step.distance_m;
            }
            let expected = 0.3_f64.powi(2) / (2.0 * 0.8);
            assert!((expected - 0.05625).abs() < 1e-15);
            assert!((distance - expected).abs() < 1e-12, "dt={dt}: {distance}");
            assert!((state.pose.x_m - expected).abs() < 1e-12);
            assert_eq!(state.speed_mps, 0.0);
            assert_eq!(state.pose.y_m, 0.0);
            assert_eq!(state.pose.yaw_rad, 0.0);
        }
    }

    // Independent 1 us RK4 reference. It samples explicitly clamped speed and
    // curvature functions in time rather than reusing production integration,
    // ramp breakpoints, yaw polynomials, or navigation's integrate helper.
    fn reference_plant(initial: PlantState, command: &MotionOutput, dt: f64) -> (Pose2, f64) {
        let (target_v, target_k) = match command {
            MotionOutput::Stop => (0.0, 0.0),
            MotionOutput::Drive {
                speed_mps,
                curvature_per_m,
            } => (*speed_mps, *curvature_per_m),
        };
        let velocity = |t: f64| {
            if target_v >= initial.speed_mps {
                (initial.speed_mps + 0.6 * t).min(target_v)
            } else {
                (initial.speed_mps - 0.8 * t).max(target_v)
            }
        };
        let curvature = |t: f64| {
            if target_k >= initial.curvature_per_m {
                (initial.curvature_per_m + 4.0 * t).min(target_k)
            } else {
                (initial.curvature_per_m - 4.0 * t).max(target_k)
            }
        };
        let rhs = |t: f64, value: [f64; 4]| {
            let v = velocity(t);
            [v * value[2].cos(), v * value[2].sin(), v * curvature(t), v]
        };
        let increment = |value: [f64; 4], slope: [f64; 4], h: f64| {
            std::array::from_fn(|i| value[i] + slope[i] * h)
        };
        let count = (dt / 0.000_001).ceil() as usize;
        let h = dt / count as f64;
        let mut value = [
            initial.pose.x_m,
            initial.pose.y_m,
            initial.pose.yaw_rad,
            0.0,
        ];
        for i in 0..count {
            let t = i as f64 * h;
            let a = rhs(t, value);
            let b = rhs(t + h * 0.5, increment(value, a, h * 0.5));
            let c = rhs(t + h * 0.5, increment(value, b, h * 0.5));
            let d = rhs(t + h, increment(value, c, h));
            for j in 0..4 {
                value[j] += h / 6.0 * (a[j] + 2.0 * b[j] + 2.0 * c[j] + d[j]);
            }
        }
        (
            Pose2 {
                x_m: value[0],
                y_m: value[1],
                yaw_rad: value[2],
            },
            value[3],
        )
    }

    #[test]
    fn joint_speed_and_steering_ramps_match_independent_fine_rk4_reference() {
        for (speed, curvature, command) in [
            (0.3, 2.0, MotionOutput::Stop),
            (0.3, -2.0, MotionOutput::Stop),
            (0.04, 0.2, MotionOutput::Stop),
            (
                0.0,
                -0.2,
                MotionOutput::Drive {
                    speed_mps: 0.04,
                    curvature_per_m: 0.12,
                },
            ),
            (
                0.22,
                1.6,
                MotionOutput::Drive {
                    speed_mps: 0.26,
                    curvature_per_m: -0.4,
                },
            ),
        ] {
            let initial = PlantState {
                pose: Pose2 {
                    x_m: 1.0,
                    y_m: 1.0,
                    yaw_rad: 0.7,
                },
                speed_mps: speed,
                curvature_per_m: curvature,
            };
            let (reference, distance) = reference_plant(initial, &command, 0.1);
            let actual = advance_plant(initial, &command, dynamics(), 0.1, |_| {});
            assert!(
                actual.state.pose.point().distance(reference.point()) < 1e-9,
                "{command:?}: {:?} vs {reference:?}",
                actual.state
            );
            assert!((actual.state.pose.yaw_rad - reference.yaw_rad).abs() < 1e-9);
            assert!((actual.distance_m - distance).abs() < 1e-9);
        }
    }

    #[test]
    fn intermediate_footprints_are_checked_even_when_tick_endpoints_are_clear() {
        // A deliberately small footprint isolates a collision between control
        // ticks; the checker must see the passage, not just the final pose.
        let mut config = SimulationConfig::example();
        config.autonomy.mission.footprint = Footprint {
            front_m: 0.001,
            rear_m: 0.001,
            half_width_m: 0.001,
        };
        let initial = PlantState {
            pose: Pose2 {
                x_m: 1.0,
                y_m: 1.0,
                yaw_rad: 0.0,
            },
            speed_mps: 0.3,
            curvature_per_m: 0.0,
        };
        let obstacle = ObstacleDisc {
            center: Point2 {
                x_m: 1.015,
                y_m: 1.0,
            },
            radius_m: 0.001,
        };
        let footprint = config.autonomy.mission.footprint;
        assert!(obstacle_clearance(initial.pose, footprint, obstacle) > 0.0);
        let mut minimum = f64::INFINITY;
        let mut violation = false;
        let mut observations = 0;
        let actual = advance_plant(
            initial,
            &MotionOutput::Drive {
                speed_mps: 0.3,
                curvature_per_m: 0.0,
            },
            dynamics(),
            0.1,
            |pose| {
                observations += 1;
                violation |= check_plant_pose(&config, pose, &[obstacle], None, &mut minimum);
            },
        );
        assert!(obstacle_clearance(actual.state.pose, footprint, obstacle) > 0.0);
        assert!(violation);
        assert!(minimum < 0.0);
        assert!(observations >= 101);
    }

    #[test]
    fn oriented_light_boundary_checks_the_intermediate_sweep_and_fault_braking() {
        let mut config = SimulationConfig::example();
        config.autonomy.mission.footprint = Footprint {
            front_m: 0.001,
            rear_m: 0.001,
            half_width_m: 0.001,
        };
        // A short synthetic U-turn has legal endpoints but crosses the line
        // between them. Rotating the scene exercises the oriented half-plane.
        for yaw_rad in [0.0, 1.3] {
            let initial = PlantState {
                pose: Pose2 {
                    x_m: 2.0,
                    y_m: 2.0,
                    yaw_rad,
                },
                speed_mps: 3.0,
                curvature_per_m: 10.0,
            };
            let boundary = HalfPlane::new(
                initial.pose.body_to_world(Point2 {
                    x_m: 0.05,
                    y_m: 0.0,
                }),
                yaw_rad,
                0.0,
            )
            .unwrap();
            let footprint = config.autonomy.mission.footprint;
            assert!(boundary.contains_footprint(footprint, initial.pose, 0.0));
            let mut minimum = f64::INFINITY;
            let mut violation = false;
            let actual = advance_plant(
                initial,
                &MotionOutput::Drive {
                    speed_mps: 3.0,
                    curvature_per_m: 10.0,
                },
                dynamics(),
                0.1,
                |pose| {
                    violation |= check_plant_pose(&config, pose, &[], Some(boundary), &mut minimum);
                },
            );
            assert!(boundary.contains_footprint(footprint, actual.state.pose, 0.0));
            assert!(violation, "yaw={yaw_rad}");
            for phase in [
                MissionPhase::Cones,
                MissionPhase::ApproachLight,
                MissionPhase::WaitGreen,
            ] {
                assert_eq!(boundary_for_phase(None, phase, boundary), Some(boundary));
            }
            assert_eq!(
                boundary_for_phase(Some(boundary), MissionPhase::Fault, boundary),
                Some(boundary)
            );
            assert_eq!(
                boundary_for_phase(None, MissionPhase::Fault, boundary),
                None
            );
            assert_eq!(
                boundary_for_phase(Some(boundary), MissionPhase::Finish, boundary),
                None
            );
            assert_eq!(
                boundary_for_phase(None, MissionPhase::CrosswalkStop, boundary),
                None
            );
        }
    }
}
