//! Sensor-driven competition control tick. Perception/localization execute outside it.
//! The only output is a checked command value and diagnostic record, never a device write.
use serde::{Deserialize, Serialize};
use std::sync::Mutex;
use xt_stcar_robot_core::autonomy::{ObstacleDisc, Point2, Pose2, PoseEstimate, RoadObservation};
use xt_stcar_robot_core::mission::{
    Mission, MissionConfig, MissionOutput, MissionPhase, MissionReport,
};
use xt_stcar_robot_core::navigation::{
    NavigationConfig, NavigationDecision, Navigator, SteeringEstimate,
};
use xt_stcar_robot_core::scan::{ScanConfig, validate_full_scan};
use xt_stcar_robot_core::{
    Controller, Event, LidarSample, MotionIntent, MotionOutput, OdometrySample, Quaternion,
    SafetyConfig, SensorKind, SensorSample, State, StepReport, TimedEvent, Timestamp, Vec3,
    VisionSample,
};

pub type Result<T> = std::result::Result<T, String>;

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct RoadFrame {
    pub observation: RoadObservation,
    pub image_width_px: u32,
    pub image_height_px: u32,
}

/// A single latest value. No unbounded queue, waiting for inference, or mutex wait.
pub struct Latest<T>(Mutex<Option<T>>);

impl<T> Default for Latest<T> {
    fn default() -> Self {
        Self(Mutex::new(None))
    }
}

impl<T> Latest<T> {
    pub fn publish(&self, value: T) -> Result<()> {
        *self
            .0
            .try_lock()
            .map_err(|_| "latest observation is busy or poisoned")? = Some(value);
        Ok(())
    }
}

impl<T: Clone> Latest<T> {
    pub fn snapshot(&self) -> Result<Option<T>> {
        Ok(self
            .0
            .try_lock()
            .map_err(|_| "latest observation is busy or poisoned")?
            .clone())
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct AutonomyConfig {
    pub schema_version: u32,
    pub simulation_only: bool,
    pub measurement_status: String,
    pub safety: SafetyConfig,
    pub mission: MissionConfig,
    pub navigation: NavigationConfig,
    pub scan: ScanConfig,
    /// Laser x-forward/y-left coordinates to the vehicle pose origin.
    pub lidar_in_body: Pose2,
    pub cone_radius_m: f64,
    pub laser_point_radius_m: f64,
    pub max_sensor_age_ms: u64,
    pub max_tick_gap_ms: u64,
}

#[derive(Debug, Serialize)]
pub struct AutonomyStep {
    pub kind: &'static str,
    pub mode: &'static str,
    pub physical_output_enabled: bool,
    pub at: Timestamp,
    pub mission: Option<MissionReport>,
    pub navigation: Option<NavigationDecision>,
    pub safety: StepReport,
    pub command: MotionOutput,
    pub fault: Option<String>,
}

pub struct AutonomyController {
    config: AutonomyConfig,
    mission: Mission,
    navigation: Navigator,
    safety: Controller,
    last_tick: Option<Timestamp>,
    last_pose: Option<Timestamp>,
    last_scan: Option<Timestamp>,
    previous_scan: Option<LidarSample>,
    last_road: Option<Timestamp>,
    fault: Option<String>,
    started: bool,
}

impl AutonomyController {
    pub fn new(config: AutonomyConfig) -> Result<Self> {
        if config.schema_version != 1
            || !config.simulation_only
            || config.measurement_status != "unverified"
            || !config.lidar_in_body.valid()
            || config.lidar_in_body.x_m.abs() > 5.0
            || config.lidar_in_body.y_m.abs() > 5.0
            || ![config.cone_radius_m, config.laser_point_radius_m]
                .iter()
                .all(|v| v.is_finite() && *v > 0.0 && *v <= 0.5)
            || config.max_sensor_age_ms == 0
            || config.max_sensor_age_ms > 2000
            || config.max_tick_gap_ms == 0
            || config.max_tick_gap_ms > 1000
        {
            return Err(
                "autonomy requires bounded explicit simulation_only/unverified configuration"
                    .into(),
            );
        }
        let frames = &config.safety.frames;
        if config.mission.world_frame != frames.odometry_frame
            || config.navigation.frame_id != frames.odometry_frame
            || config.mission.body_frame != frames.body_frame
            || config.scan.frame_id != frames.lidar_frame
            || config.mission.footprint != config.navigation.footprint
            || config.navigation.max_speed_mps > config.safety.max_speed_mps
            || config.navigation.max_curvature_per_m > config.safety.max_abs_curvature_per_m
            || config.navigation.goal_tolerance_m > config.mission.goal_tolerance_m
            || config.navigation.goal_heading_tolerance_rad
                > config.mission.goal_heading_tolerance_rad
            || [SensorKind::Lidar, SensorKind::Odometry, SensorKind::Vision]
                .iter()
                .any(|kind| !config.safety.required_sensors.contains(kind))
            || config.safety.required_sensors.contains(&SensorKind::Imu)
        {
            return Err(
                "autonomy frames, footprint, required sensors or motion limits disagree".into(),
            );
        }
        config.scan.validate().map_err(|e| e.to_string())?;
        let mission = Mission::new(config.mission.clone()).map_err(|e| e.to_string())?;
        let navigation = Navigator::new(config.navigation.clone()).map_err(|e| e.to_string())?;
        let safety = Controller::new(config.safety.clone()).map_err(|e| e.to_string())?;
        Ok(Self {
            config,
            mission,
            navigation,
            safety,
            last_tick: None,
            last_pose: None,
            last_scan: None,
            previous_scan: None,
            last_road: None,
            fault: None,
            started: false,
        })
    }

    /// Explicit start for an offline run; hardware commissioning has no entry point.
    pub fn start(&mut self) -> Result<()> {
        if self.started {
            return Err("autonomy is already started".into());
        }
        self.mission.start().map_err(|e| e.to_string())?;
        self.started = true;
        Ok(())
    }

    fn event(&mut self, at: Timestamp, event: Event) -> StepReport {
        self.safety.handle(TimedEvent { at, event })
    }

    pub fn stop_with_fault(&mut self, at: Timestamp, error: String) -> AutonomyStep {
        if self.fault.is_none() {
            self.fault = Some(error);
        }
        let safety = self.event(at, Event::EmergencyStop);
        AutonomyStep {
            kind: "autonomy_step",
            mode: "sensor_closed_loop",
            physical_output_enabled: false,
            at,
            mission: None,
            navigation: None,
            safety,
            command: MotionOutput::Stop,
            fault: self.fault.clone(),
        }
    }

    pub fn tick(
        &mut self,
        at: Timestamp,
        pose: &PoseEstimate,
        scan: &LidarSample,
        road: &RoadFrame,
    ) -> AutonomyStep {
        let step = self.tick_planned(at, pose, scan, road);
        if let Err(error) = self.navigation.adopt_command(at, &step.command) {
            return self.stop_with_fault(at, error.to_string());
        }
        step
    }

    /// Background planning uses the output owner's adopted-command history.
    /// Its result may be discarded and is deliberately not acknowledged here.
    pub fn tick_with_execution_state(
        &mut self,
        at: Timestamp,
        pose: &PoseEstimate,
        scan: &LidarSample,
        road: &RoadFrame,
        steering: SteeringEstimate,
    ) -> AutonomyStep {
        if steering.at > at {
            return self.stop_with_fault(at, "execution estimate is ahead of snapshot".into());
        }
        if let Err(error) = self.navigation.set_execution_state(steering) {
            return self.stop_with_fault(at, error.to_string());
        }
        self.tick_planned(at, pose, scan, road)
    }

    fn tick_planned(
        &mut self,
        at: Timestamp,
        pose: &PoseEstimate,
        scan: &LidarSample,
        road: &RoadFrame,
    ) -> AutonomyStep {
        if let Some(error) = self.fault.clone() {
            return self.stop_with_fault(at, error);
        }
        match self.checked_tick(at, pose, scan, road) {
            Ok(report) => report,
            Err(error) => self.stop_with_fault(at, error),
        }
    }

    fn checked_tick(
        &mut self,
        at: Timestamp,
        pose: &PoseEstimate,
        scan: &LidarSample,
        road_frame: &RoadFrame,
    ) -> Result<AutonomyStep> {
        if road_frame.image_width_px == 0
            || road_frame.image_height_px == 0
            || u64::from(road_frame.image_width_px) * u64::from(road_frame.image_height_px)
                > 64_000_000
        {
            return Err("invalid road image dimensions".into());
        }
        let road = &road_frame.observation;
        if !self.started {
            return Err("autonomy must be explicitly started".into());
        }
        if self
            .last_tick
            .is_some_and(|old| at <= old || at.0 - old.0 > self.config.max_tick_gap_ms)
        {
            return Err("control tick regressed or missed its watchdog deadline".into());
        }
        for (stamp, previous, name) in [
            (pose.captured_at, self.last_pose, "pose"),
            (scan.captured_at, self.last_scan, "laser"),
            (road.captured_at, self.last_road, "road perception"),
        ] {
            if stamp > at
                || at.0 - stamp.0 >= self.config.max_sensor_age_ms
                || previous.is_some_and(|old| stamp < old)
            {
                return Err(format!(
                    "{name} has a future, expired or regressing timestamp"
                ));
            }
        }
        validate_full_scan(scan, &self.config.scan).map_err(|e| e.to_string())?;
        // Projection requires a pose at the scan's capture time. Live adapters must
        // interpolate/associate odometry first; a merely recent pose is insufficient.
        if scan.captured_at != pose.captured_at {
            return Err(
                "laser obstacle projection requires a pose at the same capture timestamp".into(),
            );
        }
        if self
            .previous_scan
            .as_ref()
            .is_some_and(|old| old.captured_at == scan.captured_at && old != scan)
        {
            return Err("laser contents changed without advancing its capture timestamp".into());
        }
        if road.cones_body_m.len() > 256 || road.cones_body_m.iter().any(|p| !p.valid()) {
            return Err("invalid or excessive visual obstacle candidates".into());
        }
        if !road.cones_body_m.is_empty() && road.captured_at != pose.captured_at {
            return Err(
                "visual obstacle projection requires a pose at the same capture timestamp".into(),
            );
        }
        self.last_tick = Some(at);
        self.last_pose = Some(pose.captured_at);
        self.last_scan = Some(scan.captured_at);
        self.last_road = Some(road.captured_at);
        self.previous_scan = Some(scan.clone());
        // Navigation gets obstacle positions from range returns, not a simulator map.
        let mut obstacles = Vec::with_capacity(scan.ranges_m.len() + road.cones_body_m.len());
        for (index, range) in scan.ranges_m.iter().enumerate() {
            if let Some(range) = range {
                let angle = scan.angle_min_rad + index as f64 * scan.angle_increment_rad;
                let point = self.config.lidar_in_body.body_to_world(Point2 {
                    x_m: range * angle.cos(),
                    y_m: range * angle.sin(),
                });
                obstacles.push(ObstacleDisc {
                    center: pose.pose.body_to_world(point),
                    radius_m: self.config.laser_point_radius_m,
                });
            }
        }
        obstacles.extend(road.cones_body_m.iter().map(|point| ObstacleDisc {
            center: pose.pose.body_to_world(*point),
            radius_m: self.config.cone_radius_m,
        }));
        let mission = self.mission.update(at, pose, road);
        if mission.phase == MissionPhase::Fault {
            let mut stopped = self.stop_with_fault(at, mission.reason.clone());
            stopped.mission = Some(mission);
            return Ok(stopped);
        }
        // Use the exact line checked by Mission, including the transition tick
        // out of Cones. Only confirmed stationary green admission enters Finish.
        let travel_boundary = matches!(
            mission.phase,
            MissionPhase::Cones | MissionPhase::ApproachLight | MissionPhase::WaitGreen
        )
        .then(|| self.mission.light_stop_boundary());
        self.navigation.set_travel_boundary(travel_boundary);
        let (intent, navigation, hold) = match &mission.output {
            MissionOutput::Target {
                point,
                max_speed_mps,
                arrival,
            } => {
                let heading = match mission.phase {
                    MissionPhase::ApproachLight => Some(self.config.mission.light_approach_yaw_rad),
                    MissionPhase::Finish => Some(self.config.mission.finish_yaw_rad),
                    _ => None,
                };
                let decision = self
                    .navigation
                    .plan_with_arrival(
                        at,
                        pose,
                        &obstacles,
                        scan.captured_at.min(road.captured_at),
                        *point,
                        heading,
                        *max_speed_mps,
                        *arrival,
                    )
                    .map_err(|e| e.to_string())?;
                let intent = decision.intent;
                (intent, Some(decision), intent.speed_mps == 0.0)
            }
            MissionOutput::Stop => {
                self.navigation.plan_stop(at).map_err(|e| e.to_string())?;
                (
                    MotionIntent {
                        speed_mps: 0.0,
                        curvature_per_m: 0.0,
                    },
                    None,
                    true,
                )
            }
        };
        let q = pose.pose.yaw_rad * 0.5;
        let zero = Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let odom = SensorSample::Odometry(OdometrySample {
            captured_at: pose.captured_at,
            frame_id: pose.frame_id.clone(),
            child_frame_id: self.config.safety.frames.body_frame.clone(),
            position_m: Vec3 {
                x: pose.pose.x_m,
                y: pose.pose.y_m,
                z: 0.0,
            },
            orientation_xyzw: Quaternion {
                x: 0.0,
                y: 0.0,
                z: q.sin(),
                w: q.cos(),
            },
            linear_velocity_mps: Vec3 {
                x: pose.speed_mps,
                ..zero
            },
            angular_velocity_radps: Vec3 {
                z: pose.yaw_rate_radps,
                ..zero
            },
        });
        // Existing safety policy still checks previous deadlines before refreshing data.
        self.event(at, Event::Heartbeat);
        self.event(at, Event::Deadman { pressed: true });
        self.event(at, Event::Sensor { sample: odom });
        self.event(
            at,
            Event::Sensor {
                sample: SensorSample::Lidar(scan.clone()),
            },
        );
        self.event(
            at,
            Event::Sensor {
                sample: SensorSample::Vision(VisionSample {
                    captured_at: road.captured_at,
                    frame_id: self.config.safety.frames.vision_frame.clone(),
                    image_width_px: road_frame.image_width_px,
                    image_height_px: road_frame.image_height_px,
                    detection_count: road.cones_body_m.len() as u32,
                }),
            },
        );
        if self.safety.state() == State::Disarmed {
            self.event(at, Event::Arm);
        }
        self.event(at, Event::Motion { intent });
        if self.safety.state() == State::Armed {
            self.event(at, Event::Start);
        }
        let safety = self.event(at, Event::Tick);
        let command = if hold || safety.state != State::Running {
            MotionOutput::Stop
        } else {
            safety.output.command.clone()
        };
        if safety.state == State::Fault {
            self.fault = Some("safety controller latched a fault".into());
        }
        Ok(AutonomyStep {
            kind: "autonomy_step",
            mode: "sensor_closed_loop",
            physical_output_enabled: false,
            at,
            mission: Some(mission),
            navigation,
            safety,
            command,
            fault: self.fault.clone(),
        })
    }
}
