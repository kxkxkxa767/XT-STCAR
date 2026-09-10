//! Observation-driven competition task policy. This module never sends motor data.
//! Mission waiting is separate from the independent safety controller's Fault.
use crate::autonomy::{
    Footprint, HalfPlane, LightState, Point2, Pose2, PoseEstimate, Rect, RoadObservation,
};
use crate::{FrameId, Timestamp, ValidationError};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct MissionConfig {
    pub schema_version: u32,
    pub simulation_only: bool,
    pub world_frame: FrameId,
    pub body_frame: FrameId,
    pub footprint: Footprint,
    pub approach_goal: Point2,
    pub crosswalk_region: Rect,
    pub cone_waypoints: Vec<Point2>,
    pub light_detection_region: Rect,
    pub light_stop_region: Rect,
    pub light_stop_goal: Point2,
    pub light_approach_yaw_rad: f64,
    pub finish_region: Rect,
    pub finish_goal: Point2,
    pub finish_yaw_rad: f64,
    pub cruise_speed_mps: f64,
    pub approach_speed_mps: f64,
    pub goal_tolerance_m: f64,
    pub goal_heading_tolerance_rad: f64,
    pub crosswalk_stop_margin_m: f64,
    /// Upper bound on the speed of any footprint corner, including yaw motion.
    pub stopped_speed_mps: f64,
    pub crosswalk_hold_ms: u64,
    pub min_green_ms: u64,
    pub min_green_frames: u32,
    pub max_pose_age_ms: u64,
    pub max_road_age_ms: u64,
    pub max_pose_gap_ms: u64,
    pub max_road_gap_ms: u64,
    pub max_observation_skew_ms: u64,
    pub min_pose_quality: f64,
    pub min_detection_confidence: f64,
}

impl MissionConfig {
    pub fn light_stop_boundary(&self) -> Result<HalfPlane, ValidationError> {
        HalfPlane::at_region_front(
            self.light_stop_goal,
            self.light_stop_region,
            self.light_approach_yaw_rad,
        )
    }

    pub fn validate(&self) -> Result<(), ValidationError> {
        self.world_frame.validate()?;
        self.body_frame.validate()?;
        self.footprint.validate()?;
        for region in [
            self.crosswalk_region,
            self.light_detection_region,
            self.light_stop_region,
            self.finish_region,
        ] {
            region.validate()?;
        }
        if self.schema_version != 1
            || !self.simulation_only
            || self.world_frame == self.body_frame
            || !self.approach_goal.valid()
            || !self.light_stop_goal.valid()
            || !self.finish_goal.valid()
            || ![self.light_approach_yaw_rad, self.finish_yaw_rad]
                .iter()
                .all(|yaw| yaw.is_finite() && yaw.abs() <= std::f64::consts::PI)
            || !(2..=128).contains(&self.cone_waypoints.len())
            || self.cone_waypoints.iter().any(|p| !p.valid())
            || !self.finish_region.contains(self.finish_goal)
        {
            return Err(ValidationError(
                "invalid mission schema, frames or route".into(),
            ));
        }
        let light_pose = Pose2 {
            x_m: self.light_stop_goal.x_m,
            y_m: self.light_stop_goal.y_m,
            yaw_rad: self.light_approach_yaw_rad,
        };
        if !self.footprint.inside(light_pose, self.light_stop_region)
            || rect_corners(self.light_stop_region)
                .iter()
                .any(|p| !self.light_detection_region.contains(*p))
        {
            return Err(ValidationError(
                "light stop goal must fit the full footprint and its stop region must lie in the detection region".into(),
            ));
        }
        self.light_stop_boundary()?;
        let finish_pose = Pose2 {
            x_m: self.finish_goal.x_m,
            y_m: self.finish_goal.y_m,
            yaw_rad: self.finish_yaw_rad,
        };
        if !self.footprint.inside(finish_pose, self.finish_region) {
            return Err(ValidationError(
                "finish goal heading must fit the full footprint inside its region".into(),
            ));
        }
        if ![
            self.cruise_speed_mps,
            self.approach_speed_mps,
            self.goal_tolerance_m,
            self.goal_heading_tolerance_rad,
            self.crosswalk_stop_margin_m,
            self.min_pose_quality,
            self.min_detection_confidence,
        ]
        .iter()
        .all(|v| v.is_finite() && *v > 0.0)
            || self.approach_speed_mps > self.cruise_speed_mps
            || self.goal_heading_tolerance_rad > std::f64::consts::FRAC_PI_4
            || self.crosswalk_stop_margin_m <= self.goal_tolerance_m
            || !self.stopped_speed_mps.is_finite()
            || self.stopped_speed_mps < 0.0
            || self.stopped_speed_mps >= self.approach_speed_mps
            || self.min_pose_quality > 1.0
            || self.min_detection_confidence > 1.0
            || self.crosswalk_hold_ms < 3000
            || !(2..=1000).contains(&self.min_green_frames)
            || [
                self.min_green_ms,
                self.max_pose_age_ms,
                self.max_road_age_ms,
                self.max_pose_gap_ms,
                self.max_road_gap_ms,
                self.max_observation_skew_ms,
            ]
            .contains(&0)
        {
            return Err(ValidationError(
                "invalid mission speed, timing or confidence limits".into(),
            ));
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum MissionPhase {
    Idle,
    ApproachCrosswalk,
    CrosswalkStop,
    Cones,
    ApproachLight,
    WaitGreen,
    Finish,
    Completed,
    Fault,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum MissionOutput {
    Target {
        point: Point2,
        max_speed_mps: f64,
        arrival: crate::navigation::ArrivalBehavior,
    },
    Stop,
}

#[derive(Clone, Debug, Serialize)]
pub struct MissionReport {
    pub at: Timestamp,
    pub phase: MissionPhase,
    pub output: MissionOutput,
    pub reason: String,
    pub crosswalk_stop_elapsed_ms: u64,
    pub green_elapsed_ms: u64,
    pub waypoint_index: usize,
}

#[derive(Clone, Copy)]
struct CrosswalkLock {
    near_origin: Point2,
    forward: Point2,
    stop_target: Point2,
}

pub struct Mission {
    config: MissionConfig,
    light_boundary: HalfPlane,
    phase: MissionPhase,
    last_now: Option<Timestamp>,
    last_pose: Option<PoseEstimate>,
    last_road: Option<RoadObservation>,
    crosswalk: Option<CrosswalkLock>,
    stop_since: Option<Timestamp>,
    stop_elapsed_ms: u64,
    ordinary_stop_since: Option<Timestamp>,
    light_wait_since: Option<Timestamp>,
    green_since: Option<Timestamp>,
    green_frames: u32,
    green_elapsed_ms: u64,
    light_cleared: bool,
    waypoint_index: usize,
    fault_reason: Option<String>,
}

impl Mission {
    pub fn new(config: MissionConfig) -> Result<Self, ValidationError> {
        config.validate()?;
        let light_boundary = config.light_stop_boundary()?;
        Ok(Self {
            config,
            light_boundary,
            phase: MissionPhase::Idle,
            last_now: None,
            last_pose: None,
            last_road: None,
            crosswalk: None,
            stop_since: None,
            stop_elapsed_ms: 0,
            ordinary_stop_since: None,
            light_wait_since: None,
            green_since: None,
            green_frames: 0,
            green_elapsed_ms: 0,
            light_cleared: false,
            waypoint_index: 0,
            fault_reason: None,
        })
    }

    /// An explicit run start, never a reset of a finished or faulted run.
    pub fn start(&mut self) -> Result<(), ValidationError> {
        if self.phase != MissionPhase::Idle {
            return Err(ValidationError(
                "mission can only start from Idle; create a new run to reset".into(),
            ));
        }
        self.phase = MissionPhase::ApproachCrosswalk;
        Ok(())
    }

    pub fn phase(&self) -> MissionPhase {
        self.phase
    }

    pub fn config(&self) -> &MissionConfig {
        &self.config
    }

    pub fn light_stop_boundary(&self) -> HalfPlane {
        self.light_boundary
    }

    /// Uses measurement timestamps for progress; repeating a frame never accrues
    /// stop/green confirmation. No wall-clock sleeps or actuator writes occur.
    pub fn update(
        &mut self,
        now: Timestamp,
        pose: &PoseEstimate,
        road: &RoadObservation,
    ) -> MissionReport {
        let at = self.last_now.map_or(now, |old| old.max(now));
        if self.phase == MissionPhase::Fault {
            self.last_now = Some(at);
            return self.report(
                at,
                MissionOutput::Stop,
                self.fault_reason.as_deref().unwrap_or("fault latched"),
            );
        }
        if self.phase == MissionPhase::Completed {
            self.last_now = Some(at);
            return self.report(at, MissionOutput::Stop, "completed");
        }
        if let Err(reason) = self.validate_observations(now, pose, road) {
            return self.fault(at, reason);
        }
        let new_pose = self
            .last_pose
            .as_ref()
            .is_none_or(|old| pose.captured_at > old.captured_at);
        let new_road = self
            .last_road
            .as_ref()
            .is_none_or(|old| road.captured_at > old.captured_at);
        self.last_now = Some(now);
        self.last_pose = Some(pose.clone());
        self.last_road = Some(road.clone());
        let stopped = self.is_stopped(pose);

        let (output, reason) = match self.phase {
            MissionPhase::Idle => (MissionOutput::Stop, "not started"),
            MissionPhase::ApproachCrosswalk | MissionPhase::CrosswalkStop => {
                if self.crosswalk.is_none()
                    && new_road
                    && let Some(observed) = road.crosswalk
                    && observed.confidence >= self.config.min_detection_confidence
                    && observed.lateral_min_m < self.config.footprint.half_width_m
                    && observed.lateral_max_m > -self.config.footprint.half_width_m
                {
                    // Body-frame geometry belongs to the camera capture instant.
                    // Freshness/skew bounds do not compensate vehicle translation
                    // or rotation; adapters must supply the associated pose first.
                    if pose.captured_at != road.captured_at {
                        return self.fault(
                            now,
                            "crosswalk projection requires a pose at the same capture timestamp",
                        );
                    }
                    let near_origin = pose.pose.body_to_world(Point2 {
                        x_m: observed.near_edge_m,
                        y_m: 0.0,
                    });
                    let middle = pose.pose.body_to_world(Point2 {
                        x_m: (observed.near_edge_m + observed.far_edge_m) / 2.0,
                        y_m: (observed.lateral_min_m + observed.lateral_max_m) / 2.0,
                    });
                    if self.config.crosswalk_region.contains(middle) {
                        let (s, c) = pose.pose.yaw_rad.sin_cos();
                        let stop_target = pose.pose.body_to_world(Point2 {
                            x_m: observed.near_edge_m
                                - self.config.footprint.front_m
                                - self.config.crosswalk_stop_margin_m,
                            y_m: 0.0,
                        });
                        if !near_origin.valid() || !stop_target.valid() {
                            return self.fault(now, "crosswalk transform is non-finite");
                        }
                        self.crosswalk = Some(CrosswalkLock {
                            near_origin,
                            forward: Point2 { x_m: c, y_m: s },
                            stop_target,
                        });
                    }
                }
                if let Some(lock) = self.crosswalk {
                    let front = self
                        .config
                        .footprint
                        .corners(pose.pose)
                        .into_iter()
                        .map(|p| projection(p, lock.near_origin, lock.forward))
                        .fold(f64::NEG_INFINITY, f64::max);
                    if !front.is_finite() || front >= 0.0 {
                        return self.fault(now, "crosswalk crossed before the required stop");
                    }
                    let near_target = pose.pose.point().distance(lock.stop_target)
                        <= self.config.goal_tolerance_m;
                    if stopped && near_target {
                        self.phase = MissionPhase::CrosswalkStop;
                        if new_pose {
                            let since = *self.stop_since.get_or_insert(pose.captured_at);
                            self.stop_elapsed_ms = pose.captured_at.0 - since.0;
                        }
                        if self.stop_elapsed_ms >= self.config.crosswalk_hold_ms {
                            self.phase = MissionPhase::Cones;
                            (
                                self.target(self.config.cone_waypoints[0], false),
                                "crosswalk stop completed",
                            )
                        } else {
                            (MissionOutput::Stop, "holding before crosswalk")
                        }
                    } else {
                        self.stop_since = None;
                        self.stop_elapsed_ms = 0;
                        if self.phase == MissionPhase::CrosswalkStop && near_target {
                            (MissionOutput::Stop, "waiting for vehicle to stop again")
                        } else {
                            self.phase = MissionPhase::ApproachCrosswalk;
                            (
                                self.target(lock.stop_target, true),
                                "approaching locked crosswalk stop",
                            )
                        }
                    }
                } else if pose.pose.point().distance(self.config.approach_goal)
                    <= self.config.goal_tolerance_m
                {
                    return self.fault(now, "crosswalk not identified before the search goal");
                } else {
                    (
                        self.target(self.config.approach_goal, true),
                        "searching for crosswalk",
                    )
                }
            }
            MissionPhase::Cones => {
                while self.waypoint_index < self.config.cone_waypoints.len()
                    && pose
                        .pose
                        .point()
                        .distance(self.config.cone_waypoints[self.waypoint_index])
                        <= self.config.goal_tolerance_m
                {
                    self.waypoint_index += 1;
                }
                if self.waypoint_index == self.config.cone_waypoints.len() {
                    self.phase = MissionPhase::ApproachLight;
                    (
                        self.target(self.config.light_stop_goal, true),
                        "approaching light stop region",
                    )
                } else {
                    (
                        self.target(self.config.cone_waypoints[self.waypoint_index], false),
                        "following cone route",
                    )
                }
            }
            MissionPhase::ApproachLight | MissionPhase::WaitGreen => {
                let (front, _, boundary) = self.light_progress(pose.pose);
                if !front.is_finite() || !boundary.is_finite() || front > boundary {
                    return self.fault(now, "light stop line crossed without confirmed green");
                }
                let inside = self
                    .config
                    .footprint
                    .inside(pose.pose, self.config.light_stop_region);
                let near_goal = pose.pose.point().distance(self.config.light_stop_goal)
                    <= self.config.goal_tolerance_m;
                let aligned =
                    self.heading_matches(pose.pose.yaw_rad, self.config.light_approach_yaw_rad);
                if self.phase == MissionPhase::WaitGreen && !inside {
                    return self.fault(now, "vehicle footprint left the light stop region");
                }
                if stopped && near_goal && !inside {
                    return self.fault(
                        now,
                        "light stop requires the entire footprint inside its region",
                    );
                }
                if stopped && near_goal && inside && aligned {
                    if self.phase != MissionPhase::WaitGreen {
                        self.phase = MissionPhase::WaitGreen;
                        self.light_wait_since = Some(pose.captured_at);
                        self.reset_green();
                    }
                    let green = self
                        .config
                        .light_detection_region
                        .contains(pose.pose.point())
                        && road.light == LightState::Green
                        && road.light_confidence >= self.config.min_detection_confidence
                        && road.captured_at
                            >= self
                                .light_wait_since
                                .expect("wait timestamp is initialized");
                    if !green {
                        self.reset_green();
                        (MissionOutput::Stop, "waiting for confirmed green")
                    } else {
                        if new_road {
                            let since = *self.green_since.get_or_insert(road.captured_at);
                            self.green_elapsed_ms = road.captured_at.0 - since.0;
                            self.green_frames = self.green_frames.saturating_add(1);
                        }
                        if new_pose
                            && self.green_elapsed_ms >= self.config.min_green_ms
                            && self.green_frames >= self.config.min_green_frames
                        {
                            self.phase = MissionPhase::Finish;
                            (
                                self.target(self.config.finish_goal, true),
                                "confirmed green after stopping",
                            )
                        } else {
                            (MissionOutput::Stop, "debouncing green")
                        }
                    }
                } else if self.phase == MissionPhase::WaitGreen {
                    self.light_wait_since = Some(pose.captured_at);
                    self.reset_green();
                    (MissionOutput::Stop, "waiting for stationary light stop")
                } else {
                    (
                        self.target(self.config.light_stop_goal, true),
                        "approaching light stop region",
                    )
                }
            }
            MissionPhase::Finish => {
                let (_, rear, boundary) = self.light_progress(pose.pose);
                if !rear.is_finite() || !boundary.is_finite() {
                    return self.fault(now, "light clearance geometry is non-finite");
                }
                self.light_cleared |= rear > boundary;
                if !self.light_cleared
                    && (road.light != LightState::Green
                        || road.light_confidence < self.config.min_detection_confidence)
                {
                    return self.fault(
                        now,
                        "green signal lost before the vehicle cleared the light",
                    );
                }
                let near_goal = pose.pose.point().distance(self.config.finish_goal)
                    <= self.config.goal_tolerance_m;
                if near_goal && stopped {
                    if !self
                        .config
                        .footprint
                        .inside(pose.pose, self.config.finish_region)
                    {
                        return self.fault(
                            now,
                            "finish requires the entire footprint inside its region",
                        );
                    }
                    if self.heading_matches(pose.pose.yaw_rad, self.config.finish_yaw_rad) {
                        self.phase = MissionPhase::Completed;
                        (MissionOutput::Stop, "completed")
                    } else {
                        (
                            self.target(self.config.finish_goal, true),
                            "aligning finish heading",
                        )
                    }
                } else {
                    (
                        self.target(self.config.finish_goal, true),
                        "approaching finish",
                    )
                }
            }
            MissionPhase::Completed | MissionPhase::Fault => {
                unreachable!("terminal phases returned earlier")
            }
        };

        // Competition rule: ordinary stalls longer than 10 s fail the task. The
        // required crosswalk/traffic-light waits are exempt, never Safety resets.
        if !matches!(
            self.phase,
            MissionPhase::Idle
                | MissionPhase::CrosswalkStop
                | MissionPhase::WaitGreen
                | MissionPhase::Completed
        ) && stopped
        {
            if new_pose {
                let since = *self.ordinary_stop_since.get_or_insert(pose.captured_at);
                if pose.captured_at.0 - since.0 > 10_000 {
                    return self.fault(now, "ordinary stop exceeded 10 seconds");
                }
            }
        } else {
            self.ordinary_stop_since = None;
        }
        self.report(now, output, reason)
    }

    fn validate_observations(
        &self,
        now: Timestamp,
        pose: &PoseEstimate,
        road: &RoadObservation,
    ) -> Result<(), String> {
        if self.last_now.is_some_and(|old| now < old) {
            return Err("mission time regressed".into());
        }
        if pose.frame_id != self.config.world_frame || road.frame_id != self.config.body_frame {
            return Err("mission observation frame mismatch".into());
        }
        if !pose.pose.valid()
            || !pose.speed_mps.is_finite()
            || !pose.yaw_rate_radps.is_finite()
            || !pose.quality.is_finite()
            || !(self.config.min_pose_quality..=1.0).contains(&pose.quality)
            || !road.light_confidence.is_finite()
            || !(0.0..=1.0).contains(&road.light_confidence)
            || road.cones_body_m.len() > 1024
            || road.cones_body_m.iter().any(|p| !p.valid())
            || self
                .config
                .footprint
                .corners(pose.pose)
                .iter()
                .any(|p| !p.valid())
            || !pose
                .pose
                .point()
                .distance(self.config.approach_goal)
                .is_finite()
            || !pose
                .pose
                .point()
                .distance(self.config.light_stop_goal)
                .is_finite()
            || !pose
                .pose
                .point()
                .distance(self.config.finish_goal)
                .is_finite()
            || self
                .config
                .cone_waypoints
                .iter()
                .any(|p| !pose.pose.point().distance(*p).is_finite())
        {
            return Err("invalid or untrusted mission observation".into());
        }
        if let Some(crosswalk) = road.crosswalk
            && (![
                crosswalk.near_edge_m,
                crosswalk.far_edge_m,
                crosswalk.lateral_min_m,
                crosswalk.lateral_max_m,
                crosswalk.confidence,
            ]
            .iter()
            .all(|v| v.is_finite())
                || crosswalk.near_edge_m >= crosswalk.far_edge_m
                || crosswalk.lateral_min_m >= crosswalk.lateral_max_m
                || !(0.0..=1.0).contains(&crosswalk.confidence)
                || !(crosswalk.near_edge_m + crosswalk.far_edge_m).is_finite()
                || !(crosswalk.lateral_min_m + crosswalk.lateral_max_m).is_finite())
        {
            return Err("invalid crosswalk geometry or confidence".into());
        }
        if pose.captured_at > now || road.captured_at > now {
            return Err("mission observation is in the future".into());
        }
        if now.0 - pose.captured_at.0 >= self.config.max_pose_age_ms
            || now.0 - road.captured_at.0 >= self.config.max_road_age_ms
        {
            return Err("mission observation expired".into());
        }
        if pose.captured_at.0.abs_diff(road.captured_at.0) > self.config.max_observation_skew_ms {
            return Err("pose and road observation timestamps differ too much".into());
        }
        if let Some(old) = &self.last_pose {
            if pose.captured_at < old.captured_at
                || (pose.captured_at == old.captured_at && pose != old)
            {
                return Err(
                    "pose time regressed or a timestamp was reused for changed data".into(),
                );
            }
            if pose.captured_at.0 - old.captured_at.0 > self.config.max_pose_gap_ms {
                return Err("pose update continuity was lost".into());
            }
        }
        if let Some(old) = &self.last_road {
            if road.captured_at < old.captured_at
                || (road.captured_at == old.captured_at && road != old)
            {
                return Err(
                    "road time regressed or a timestamp was reused for changed data".into(),
                );
            }
            if road.captured_at.0 - old.captured_at.0 > self.config.max_road_gap_ms {
                return Err("road update continuity was lost".into());
            }
        }
        Ok(())
    }

    fn is_stopped(&self, pose: &PoseEstimate) -> bool {
        let radius = self
            .config
            .footprint
            .front_m
            .max(self.config.footprint.rear_m)
            .hypot(self.config.footprint.half_width_m);
        pose.speed_mps.abs() + pose.yaw_rate_radps.abs() * radius <= self.config.stopped_speed_mps
    }

    fn heading_matches(&self, actual: f64, target: f64) -> bool {
        use std::f64::consts::{PI, TAU};
        // Normalize before subtraction so even large finite measured headings do
        // not overflow, and equivalent headings across the -pi/pi seam agree.
        let error = (actual.rem_euclid(TAU) - target.rem_euclid(TAU) + PI).rem_euclid(TAU) - PI;
        error.abs() <= self.config.goal_heading_tolerance_rad
    }

    fn light_progress(&self, pose: Pose2) -> (f64, f64, f64) {
        let (front, rear) = self
            .light_boundary
            .footprint_progress(self.config.footprint, pose);
        (front, rear, self.light_boundary.max_projection_m())
    }

    fn reset_green(&mut self) {
        self.green_since = None;
        self.green_frames = 0;
        self.green_elapsed_ms = 0;
    }

    fn target(&self, point: Point2, approach: bool) -> MissionOutput {
        MissionOutput::Target {
            point,
            arrival: if approach {
                crate::navigation::ArrivalBehavior::Stop
            } else {
                let next = self
                    .config
                    .cone_waypoints
                    .get(self.waypoint_index + 1)
                    .copied();
                crate::navigation::ArrivalBehavior::PassThrough {
                    next: next.unwrap_or(self.config.light_stop_goal),
                    next_heading_rad: next.is_none().then_some(self.config.light_approach_yaw_rad),
                    next_max_speed_mps: if next.is_none() {
                        self.config.approach_speed_mps
                    } else {
                        self.config.cruise_speed_mps
                    },
                }
            },
            max_speed_mps: if approach {
                self.config.approach_speed_mps
            } else {
                self.config.cruise_speed_mps
            },
        }
    }

    fn fault(&mut self, at: Timestamp, reason: impl Into<String>) -> MissionReport {
        self.phase = MissionPhase::Fault;
        self.last_now = Some(self.last_now.map_or(at, |old| old.max(at)));
        self.fault_reason.get_or_insert_with(|| reason.into());
        self.report(
            at,
            MissionOutput::Stop,
            self.fault_reason.as_deref().unwrap_or("fault latched"),
        )
    }

    fn report(&self, at: Timestamp, output: MissionOutput, reason: &str) -> MissionReport {
        MissionReport {
            at,
            phase: self.phase,
            output,
            reason: reason.into(),
            crosswalk_stop_elapsed_ms: self.stop_elapsed_ms,
            green_elapsed_ms: self.green_elapsed_ms,
            waypoint_index: self.waypoint_index,
        }
    }
}

fn rect_corners(rect: Rect) -> [Point2; 4] {
    [
        Point2 {
            x_m: rect.min_x_m,
            y_m: rect.min_y_m,
        },
        Point2 {
            x_m: rect.min_x_m,
            y_m: rect.max_y_m,
        },
        Point2 {
            x_m: rect.max_x_m,
            y_m: rect.min_y_m,
        },
        Point2 {
            x_m: rect.max_x_m,
            y_m: rect.max_y_m,
        },
    ]
}

fn projection(point: Point2, origin: Point2, direction: Point2) -> f64 {
    (point.x_m - origin.x_m) * direction.x_m + (point.y_m - origin.y_m) * direction.y_m
}
