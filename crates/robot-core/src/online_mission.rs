//! Observation-driven task topology, without a surveyed route or field layout.
//!
//! Region markers describe a near edge and a region extending along their
//! heading. Their measured dimensions must be supplied by perception or an
//! explicit marker protocol. This policy cannot infer a stop area from lamp
//! color. Targets are proposals to the existing Navigator/admission pipeline;
//! they never authorize actuator output or certify an unobserved stopping path.
use crate::autonomy::{
    Footprint, HalfPlane, LightState, Point2, Pose2, PoseEstimate, Rect, RoadObservation,
};
use crate::local_world::{
    ElementColor, ElementGeometry, ElementKind, ElementTrack, LocalWorld, SpaceState, TrackId,
};
use crate::mission::{MissionConfig, MissionOutput, MissionPhase, MissionReport};
use crate::navigation::ArrivalBehavior;
use crate::{FrameId, Timestamp, ValidationError};
use serde::{Deserialize, Serialize};
use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI, TAU};

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct OnlineMissionConfig {
    pub schema_version: u32,
    pub simulation_only: bool,
    pub world_frame: FrameId,
    pub body_frame: FrameId,
    pub footprint: Footprint,
    pub cruise_speed_mps: f64,
    pub approach_speed_mps: f64,
    pub goal_tolerance_m: f64,
    pub goal_heading_tolerance_rad: f64,
    pub crosswalk_stop_margin_m: f64,
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
    /// Engineering search/geometry bounds, not official field dimensions.
    pub clearance_m: f64,
    pub max_curvature_per_m: f64,
    pub search_horizon_m: f64,
    pub max_search_distance_m: f64,
    pub max_search_ms: u64,
    pub max_track_loss_ms: u64,
    pub preferred_cone_radius_m: f64,
    pub arc_lookahead_rad: f64,
    pub processed_exclusion_m: f64,
    /// Optional experimental color identities. None still requires topology,
    /// a confirmed track, and exclusion of every already processed cone.
    pub required_cone_colors: [Option<ElementColor>; 2],
}

impl OnlineMissionConfig {
    /// Deliberately copies only rule/control limits: no route point, rectangle,
    /// world element coordinate, or measured field dimension is read here.
    pub fn from_limits(source: &MissionConfig) -> Self {
        Self {
            schema_version: 1,
            simulation_only: true,
            world_frame: source.world_frame.clone(),
            body_frame: source.body_frame.clone(),
            footprint: source.footprint,
            cruise_speed_mps: source.cruise_speed_mps,
            approach_speed_mps: source.approach_speed_mps,
            goal_tolerance_m: source.goal_tolerance_m,
            goal_heading_tolerance_rad: source.goal_heading_tolerance_rad,
            crosswalk_stop_margin_m: source.crosswalk_stop_margin_m,
            stopped_speed_mps: source.stopped_speed_mps,
            crosswalk_hold_ms: source.crosswalk_hold_ms,
            min_green_ms: source.min_green_ms,
            min_green_frames: source.min_green_frames,
            max_pose_age_ms: source.max_pose_age_ms,
            max_road_age_ms: source.max_road_age_ms,
            max_pose_gap_ms: source.max_pose_gap_ms,
            max_road_gap_ms: source.max_road_gap_ms,
            max_observation_skew_ms: source.max_observation_skew_ms,
            min_pose_quality: source.min_pose_quality,
            min_detection_confidence: source.min_detection_confidence,
            clearance_m: 0.04,
            max_curvature_per_m: 2.0,
            search_horizon_m: 0.8,
            max_search_distance_m: 12.0,
            max_search_ms: 60_000,
            max_track_loss_ms: 2000,
            preferred_cone_radius_m: 0.85,
            arc_lookahead_rad: FRAC_PI_4,
            processed_exclusion_m: 0.6,
            required_cone_colors: [None, None],
        }
    }

    pub fn validate(&self) -> Result<(), ValidationError> {
        self.world_frame.validate()?;
        self.body_frame.validate()?;
        self.footprint.validate()?;
        if self.schema_version != 1
            || !self.simulation_only
            || self.world_frame == self.body_frame
            || ![
                self.cruise_speed_mps,
                self.approach_speed_mps,
                self.goal_tolerance_m,
                self.goal_heading_tolerance_rad,
                self.crosswalk_stop_margin_m,
                self.clearance_m,
                self.max_curvature_per_m,
                self.search_horizon_m,
                self.max_search_distance_m,
                self.preferred_cone_radius_m,
                self.processed_exclusion_m,
                self.arc_lookahead_rad,
            ]
            .iter()
            .all(|x| x.is_finite() && *x > 0.0)
            || self.approach_speed_mps > self.cruise_speed_mps
            || self.goal_heading_tolerance_rad > FRAC_PI_4
            || self.crosswalk_stop_margin_m <= self.goal_tolerance_m
            || !self.stopped_speed_mps.is_finite()
            || self.stopped_speed_mps < 0.0
            || self.stopped_speed_mps >= self.approach_speed_mps
            || ![self.min_pose_quality, self.min_detection_confidence]
                .iter()
                .all(|x| x.is_finite() && *x > 0.0 && *x <= 1.0)
            || self.crosswalk_hold_ms < 3000
            || self.min_green_ms < 300
            || !(2..=1000).contains(&self.min_green_frames)
            || [
                self.max_pose_age_ms,
                self.max_road_age_ms,
                self.max_pose_gap_ms,
                self.max_road_gap_ms,
                self.max_observation_skew_ms,
                self.max_search_ms,
                self.max_track_loss_ms,
            ]
            .contains(&0)
            || self.search_horizon_m > 2.0
            || self.max_search_distance_m > 100.0
            || self.max_search_ms > 180_000
            || self.max_track_loss_ms > 10_000
            || self.arc_lookahead_rad > FRAC_PI_4
            || self.preferred_cone_radius_m < 1.0 / self.max_curvature_per_m
        {
            return Err(ValidationError(
                "invalid online mission limits or topology bounds".into(),
            ));
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum OnlineBehavior {
    Idle,
    Search,
    ApproachRegion,
    HoldCrosswalk,
    ConeEntry,
    OrbitCone,
    WaitGreen,
    Completed,
    Fault,
}

#[derive(Clone, Debug, Serialize)]
pub struct OnlineMissionReport {
    pub mission: MissionReport,
    pub goal_heading_rad: Option<f64>,
    pub travel_boundary: Option<HalfPlane>,
    pub behavior: OnlineBehavior,
    pub active_track_id: Option<TrackId>,
    /// Last accepted geometry, including its actual observation times. May be
    /// stale while holding for reacquisition; presence never authorizes motion.
    pub active_track: Option<ElementTrack>,
    pub processed_cones: usize,
    pub cone_progress_rad: f64,
    pub search_distance_m: f64,
    /// The execution owner may reject a proposed command whose complete
    /// stopping envelope is not certified as observed free space.
    #[serde(default, skip_serializing_if = "is_false")]
    pub execution_space_rejected: bool,
}

#[derive(Clone, Copy)]
struct Orbit {
    radius_m: f64,
    entry_aligned: bool,
    entered: bool,
    progress_rad: f64,
    saw_outer_side: bool,
    last_position: Point2,
}

#[derive(Clone, Copy)]
struct ProcessedCone {
    position: Point2,
    radius_m: f64,
    position_error_m: f64,
}

struct Proposal {
    output: MissionOutput,
    heading: Option<f64>,
    behavior: OnlineBehavior,
    reason: &'static str,
}
impl Proposal {
    fn stop(behavior: OnlineBehavior, reason: &'static str) -> Self {
        Self {
            output: MissionOutput::Stop,
            heading: None,
            behavior,
            reason,
        }
    }
}

pub struct OnlineMission {
    config: OnlineMissionConfig,
    phase: MissionPhase,
    initial_heading: Option<f64>,
    active: Option<TrackId>,
    active_geometry: Option<ElementTrack>,
    missing_since: Option<Timestamp>,
    orbit: Option<Orbit>,
    processed: [Option<ProcessedCone>; 2],
    cone_index: usize,
    light: Option<ElementTrack>,
    light_cleared: bool,
    light_missing_since: Option<Timestamp>,
    observed_signal_without_region: bool,
    last_now: Option<Timestamp>,
    last_pose: Option<PoseEstimate>,
    last_road: Option<RoadObservation>,
    search_since: Option<Timestamp>,
    search_distance: f64,
    stop_since: Option<Timestamp>,
    stop_elapsed_ms: u64,
    ordinary_stop_since: Option<Timestamp>,
    light_wait_since: Option<Timestamp>,
    green_since: Option<Timestamp>,
    green_frames: u32,
    green_elapsed_ms: u64,
    fault_reason: Option<String>,
}

impl OnlineMission {
    pub fn new(config: OnlineMissionConfig) -> Result<Self, ValidationError> {
        config.validate()?;
        Ok(Self {
            config,
            phase: MissionPhase::Idle,
            initial_heading: None,
            active: None,
            active_geometry: None,
            missing_since: None,
            orbit: None,
            processed: [None, None],
            cone_index: 0,
            light: None,
            light_cleared: false,
            light_missing_since: None,
            observed_signal_without_region: false,
            last_now: None,
            last_pose: None,
            last_road: None,
            search_since: None,
            search_distance: 0.0,
            stop_since: None,
            stop_elapsed_ms: 0,
            ordinary_stop_since: None,
            light_wait_since: None,
            green_since: None,
            green_frames: 0,
            green_elapsed_ms: 0,
            fault_reason: None,
        })
    }
    pub fn config(&self) -> &OnlineMissionConfig {
        &self.config
    }
    pub fn phase(&self) -> MissionPhase {
        self.phase
    }
    /// Last accepted stop-region observation, potentially stale. Execution
    /// must check its source age and full stopping-envelope intersection;
    /// retaining this restriction is not permission to cross its edge.
    pub fn last_observed_light_region(&self) -> Option<ElementTrack> {
        self.light
    }
    pub fn start(&mut self) -> Result<(), ValidationError> {
        if self.phase != MissionPhase::Idle {
            return Err(ValidationError("online mission can only start once".into()));
        }
        self.phase = MissionPhase::ApproachCrosswalk;
        Ok(())
    }

    pub fn update(
        &mut self,
        now: Timestamp,
        pose: &PoseEstimate,
        road: &RoadObservation,
        world: &mut LocalWorld,
    ) -> OnlineMissionReport {
        if matches!(self.phase, MissionPhase::Completed | MissionPhase::Fault) {
            return self.report(
                now,
                Proposal::stop(
                    if self.phase == MissionPhase::Fault {
                        OnlineBehavior::Fault
                    } else {
                        OnlineBehavior::Completed
                    },
                    "terminal task state",
                ),
            );
        }
        if let Err(error) = self.validate_input(now, pose, road, world) {
            return self.fault(now, error);
        }
        let new_pose = self
            .last_pose
            .as_ref()
            .is_none_or(|old| pose.captured_at > old.captured_at);
        let new_road = self
            .last_road
            .as_ref()
            .is_none_or(|old| road.captured_at > old.captured_at);
        if self.active.is_none()
            && new_pose
            && let Some(old) = &self.last_pose
        {
            self.search_distance += pose.pose.point().distance(old.pose.point());
        }
        self.last_now = Some(now);
        self.last_pose = Some(pose.clone());
        self.last_road = Some(road.clone());
        self.initial_heading.get_or_insert(wrap(pose.pose.yaw_rad));
        if let Some(light) = self.light {
            if let Some(updated) = world.tracks(now).find(|track| track.id == light.id) {
                self.light = Some(updated);
            }
        } else {
            self.light = self.select_track(now, pose.pose, ElementKind::StopLine, world);
        }
        if let Some(boundary) = self.travel_boundary()
            && !boundary.contains_footprint(self.config.footprint, pose.pose, 0.0)
        {
            return self.fault(now, "observed task boundary crossed before permission");
        }
        if let Some(id) = self.active {
            if let Some(track) = world.tracks(now).find(|track| track.id == id) {
                self.active_geometry = Some(track);
                self.missing_since = None;
            } else {
                self.stop_since = None;
                self.stop_elapsed_ms = 0;
                self.reset_green();
                let since = *self.missing_since.get_or_insert(now);
                if now.0 - since.0 >= self.config.max_track_loss_ms {
                    return self.fault(
                        now,
                        "current element lost beyond bounded reacquisition time",
                    );
                }
                return self.finish_tick(
                    now,
                    pose,
                    new_pose,
                    Proposal::stop(
                        OnlineBehavior::Search,
                        "current tracked element unavailable; holding for reacquisition",
                    ),
                );
            }
        }
        let proposal = match self.phase {
            MissionPhase::Idle => Ok(Proposal::stop(OnlineBehavior::Idle, "not started")),
            MissionPhase::ApproachCrosswalk | MissionPhase::CrosswalkStop => {
                self.crosswalk_step(now, pose, new_pose, world)
            }
            MissionPhase::Cones => self.cone_step(now, pose, new_pose, world),
            MissionPhase::ApproachLight | MissionPhase::WaitGreen => {
                self.light_step(now, pose, road, new_pose, new_road, world)
            }
            MissionPhase::Finish => self.finish_step(now, pose, road, world),
            MissionPhase::Completed | MissionPhase::Fault => unreachable!(),
        };
        match proposal {
            Ok(proposal) => self.finish_tick(now, pose, new_pose, proposal),
            Err(error) => self.fault(now, error),
        }
    }

    fn crosswalk_step(
        &mut self,
        now: Timestamp,
        pose: &PoseEstimate,
        new_pose: bool,
        world: &mut LocalWorld,
    ) -> Result<Proposal, String> {
        let Some(track) = self.acquire(now, pose.pose, ElementKind::Crosswalk, world) else {
            return self.search(now, pose.pose, world);
        };
        let heading = track
            .heading_rad
            .ok_or("crosswalk has no observed heading")?;
        let frame = frame(track);
        let goal = frame.body_to_world(Point2 {
            x_m: -self.config.footprint.front_m
                - self.config.crosswalk_stop_margin_m
                - region_error(track),
            y_m: 0.0,
        });
        let boundary = region_boundary(track, false)?;
        if !boundary.contains_footprint(self.config.footprint, pose.pose, 0.0) {
            return Err("crosswalk crossed before required observed stop".into());
        }
        if self.stopped(pose) && self.near(pose.pose, goal, heading) {
            self.phase = MissionPhase::CrosswalkStop;
            if new_pose {
                let since = *self.stop_since.get_or_insert(pose.captured_at);
                self.stop_elapsed_ms = pose.captured_at.0 - since.0;
            }
            if self.stop_elapsed_ms >= self.config.crosswalk_hold_ms {
                world
                    .mark_processed(track.id)
                    .map_err(|error| error.to_string())?;
                self.transition(MissionPhase::Cones);
                return Ok(Proposal::stop(
                    OnlineBehavior::Search,
                    "observed crosswalk stop completed",
                ));
            }
            return Ok(Proposal::stop(
                OnlineBehavior::HoldCrosswalk,
                "holding before observed crosswalk",
            ));
        }
        self.stop_since = None;
        self.stop_elapsed_ms = 0;
        self.phase = MissionPhase::ApproachCrosswalk;
        Ok(self.local_target(
            now,
            pose.pose,
            goal,
            Some(heading),
            None,
            true,
            OnlineBehavior::ApproachRegion,
            "approaching observed crosswalk",
            world,
        ))
    }

    fn cone_step(
        &mut self,
        now: Timestamp,
        pose: &PoseEstimate,
        new_pose: bool,
        world: &mut LocalWorld,
    ) -> Result<Proposal, String> {
        let Some(track) = self.acquire(now, pose.pose, ElementKind::Cone, world) else {
            return self.search(now, pose.pose, world);
        };
        let ElementGeometry::Cone { radius_m } = track.geometry else {
            return Err("cone track has incompatible geometry".into());
        };
        let axis = self.initial_heading.unwrap_or(pose.pose.yaw_rad);
        let direction = if self.cone_index == 0 { 1.0 } else { -1.0 };
        let entry_angle = axis - FRAC_PI_2;
        let entry_heading = wrap(axis + if self.cone_index == 0 { 0.0 } else { PI });
        let exit_heading = wrap(axis + if self.cone_index == 0 { PI } else { 0.0 });
        // On a tangent circular reference the closest point of the entire
        // rectangle to the cone is its inner lateral edge. Front/rear length
        // belongs in the swept-area proof below, not an artificial radial width.
        let minimum_radius = (radius_m
            + self.config.footprint.half_width_m
            + self.config.clearance_m
            + track.position_error_m)
            .max(1.0 / self.config.max_curvature_per_m);
        let preferred_radius = self.config.preferred_cone_radius_m.max(minimum_radius);
        let first_approach = self.orbit.is_none();
        let mut orbit = self.orbit.unwrap_or(Orbit {
            radius_m: preferred_radius,
            entry_aligned: false,
            entered: false,
            progress_rad: 0.0,
            saw_outer_side: false,
            last_position: pose.pose.point(),
        });
        if !orbit.entered && !orbit.entry_aligned {
            // Reject radii whose next two lookaheads intersect observed
            // obstacles, including the outer wall. A cone can occlude the far
            // side, so an Unknown future arc remains a planning hypothesis:
            // every current target and actual stopping envelope must still be
            // KnownFree. No future hypothesis grants execution permission.
            let mut selected = None;
            // Keep a still-feasible chosen radius while aligning. Re-selecting
            // a larger candidate after a viewpoint change moves the tangent
            // sideways and can make an otherwise stable entrance unreachable.
            let upper_radius = orbit.radius_m.max(minimum_radius);
            for step in 0..=4 {
                let radius = upper_radius - (upper_radius - minimum_radius) * step as f64 / 4.0;
                if self.arc_space_state(
                    now,
                    world,
                    track.position,
                    radius,
                    entry_angle,
                    direction * (2.0 * self.config.arc_lookahead_rad).min(FRAC_PI_2),
                ) != SpaceState::Obstacle
                {
                    selected = Some(radius);
                    break;
                }
            }
            let Some(radius) = selected else {
                // Every screened radius conservatively intersects measured
                // obstacles. Only a known-free tangent guide may be approached
                // for another observation; this fallback cannot enter the arc.
                // Future Unknown is merely a planning hypothesis. Each current
                // target and execution stopping envelope still needs KnownFree.
                let entry = polar(track.position, orbit.radius_m, entry_angle);
                let guide = polar(entry, self.config.search_horizon_m, entry_heading + PI);
                self.orbit = Some(orbit);
                if !self.near(pose.pose, guide, entry_heading) {
                    return Ok(self.local_target(
                        now,
                        pose.pose,
                        guide,
                        Some(entry_heading),
                        None,
                        true,
                        OnlineBehavior::ConeEntry,
                        "approaching observed entrance tangent to resolve hidden arc",
                        world,
                    ));
                }
                return Ok(Proposal::stop(
                    OnlineBehavior::ConeEntry,
                    "no observed whole-body arc supports a cone entry radius",
                ));
            };
            orbit.radius_m = radius;
        }
        if orbit.radius_m < minimum_radius {
            self.orbit = Some(orbit);
            return Ok(Proposal::stop(
                OnlineBehavior::OrbitCone,
                "updated cone uncertainty invalidated the committed orbit clearance",
            ));
        }
        if !orbit.entered {
            let entry = polar(track.position, orbit.radius_m, entry_angle);
            if self.near(pose.pose, entry, entry_heading) {
                orbit.entered = true;
                orbit.last_position = pose.pose.point();
            } else {
                // A distant point-only approach can cross the entrance before
                // an Ackermann vehicle reaches its required tangent. Reserve
                // one existing local horizon before that entrance for a
                // straight, oriented continuation. This guide is derived only
                // from the current observed cone and the task's passing side.
                let approach = Pose2 {
                    x_m: entry.x_m,
                    y_m: entry.y_m,
                    yaw_rad: entry_heading,
                }
                .world_to_body(pose.pose.point());
                let guide = polar(entry, self.config.search_horizon_m, entry_heading + PI);
                orbit.entry_aligned |= (first_approach
                    || (self.near(pose.pose, guide, entry_heading) && self.stopped(pose)))
                    && approach.x_m <= 0.0
                    && approach.x_m >= -self.config.search_horizon_m - self.config.goal_tolerance_m
                    && approach.y_m.abs() <= self.config.goal_tolerance_m
                    && wrap(pose.pose.yaw_rad - entry_heading).abs()
                        <= self.config.goal_heading_tolerance_rad;
                if !orbit.entry_aligned {
                    self.orbit = Some(orbit);
                    return Ok(self.local_target(
                        now,
                        pose.pose,
                        guide,
                        Some(entry_heading),
                        None,
                        true,
                        OnlineBehavior::ConeEntry,
                        "stopping on observed cone entrance tangent before entry",
                        world,
                    ));
                }
                let next = polar(
                    track.position,
                    orbit.radius_m,
                    entry_angle + direction * self.config.arc_lookahead_rad,
                );
                self.orbit = Some(orbit);
                let mut proposal = self.local_target(
                    now,
                    pose.pose,
                    entry,
                    Some(entry_heading),
                    Some((
                        next,
                        wrap(entry_heading + direction * self.config.arc_lookahead_rad),
                    )),
                    false,
                    OnlineBehavior::ConeEntry,
                    "approaching current tracked cone entrance",
                    world,
                );
                if matches!(proposal.output, MissionOutput::Target { .. }) {
                    // Preserve the acquired tangent even if a small tracking
                    // error puts this straight segment just over the horizon.
                    proposal.heading = Some(entry_heading);
                }
                return Ok(proposal);
            }
        }
        if new_pose {
            // Reproject BOTH actual vehicle positions around the newest center:
            // a changed landmark estimate alone cannot manufacture angular work.
            let previous = (orbit.last_position.y_m - track.position.y_m)
                .atan2(orbit.last_position.x_m - track.position.x_m);
            let current =
                (pose.pose.y_m - track.position.y_m).atan2(pose.pose.x_m - track.position.x_m);
            let delta = direction * wrap(current - previous);
            if delta.abs() > FRAC_PI_2 {
                return Err("cone progress discontinuity; cannot certify passing relation".into());
            }
            orbit.progress_rad = (orbit.progress_rad + delta).clamp(0.0, PI);
            orbit.last_position = pose.pose.point();
            let along = (pose.pose.x_m - track.position.x_m) * axis.cos()
                + (pose.pose.y_m - track.position.y_m) * axis.sin();
            orbit.saw_outer_side |=
                direction * along >= orbit.radius_m * 0.7 + track.position_error_m;
        }
        let exit = polar(track.position, orbit.radius_m, entry_angle + direction * PI);
        // Reaching a tolerance disc around the top tangent can still leave the
        // body reference point on the entry side of the exit centerline. Clear
        // the measured center uncertainty before handing over to the next task;
        // otherwise an immediately observed lamp may stop those last millimetres.
        let exit_goal = polar(
            exit,
            track.position_error_m + 2.0 * self.config.goal_tolerance_m,
            exit_heading,
        );
        let exit_projection = -direction
            * ((pose.pose.x_m - track.position.x_m) * axis.cos()
                + (pose.pose.y_m - track.position.y_m) * axis.sin());
        if orbit.saw_outer_side
            && orbit.progress_rad >= PI - self.config.goal_heading_tolerance_rad
            && exit_projection >= track.position_error_m
            && self.near(pose.pose, exit_goal, exit_heading)
        {
            self.processed[self.cone_index] = Some(ProcessedCone {
                position: track.position,
                radius_m,
                position_error_m: track.position_error_m,
            });
            world
                .mark_processed(track.id)
                .map_err(|error| error.to_string())?;
            self.cone_index += 1;
            self.transition(if self.cone_index == 2 {
                MissionPhase::ApproachLight
            } else {
                MissionPhase::Cones
            });
            return self.search(now, pose.pose, world);
        }
        let progress = (orbit.progress_rad + self.config.arc_lookahead_rad).min(PI);
        let angle = entry_angle + direction * progress;
        let goal = if progress == PI {
            exit_goal
        } else {
            polar(track.position, orbit.radius_m, angle)
        };
        let heading = wrap(angle + direction * FRAC_PI_2);
        let next_progress = (progress + self.config.arc_lookahead_rad).min(PI);
        let (next, next_heading) = if next_progress > progress {
            let a = entry_angle + direction * next_progress;
            (
                if next_progress == PI {
                    exit_goal
                } else {
                    polar(track.position, orbit.radius_m, a)
                },
                wrap(a + direction * FRAC_PI_2),
            )
        } else {
            (
                polar(exit_goal, self.config.search_horizon_m, exit_heading),
                exit_heading,
            )
        };
        // Slow before the not-yet-observed part of the rolling turn. Waiting
        // until the full execution envelope is rejected forces Stop/steering
        // centering, which can strand a tight Ackermann turn. This uses the
        // existing approach speed; the final known-free certificate is unchanged.
        let cautious_approach = self.arc_space_state(
            now,
            world,
            track.position,
            orbit.radius_m,
            entry_angle + direction * orbit.progress_rad,
            direction * (progress - orbit.progress_rad),
        ) != SpaceState::KnownFree;
        self.orbit = Some(orbit);
        Ok(self.local_target(
            now,
            pose.pose,
            goal,
            Some(heading),
            Some((next, next_heading)),
            cautious_approach,
            OnlineBehavior::OrbitCone,
            "following short arc about current tracked cone",
            world,
        ))
    }

    fn light_step(
        &mut self,
        now: Timestamp,
        pose: &PoseEstimate,
        road: &RoadObservation,
        new_pose: bool,
        new_road: bool,
        world: &mut LocalWorld,
    ) -> Result<Proposal, String> {
        let Some(track) = self.acquire(now, pose.pose, ElementKind::StopLine, world) else {
            // A visible signal warns that a permission boundary is nearby, but
            // supplies no line geometry. Free laser space cannot authorize
            // crossing it. The unchanged ordinary-stop timer bounds this wait.
            self.observed_signal_without_region |= road.light != LightState::Unknown
                && road.light_confidence >= self.config.min_detection_confidence;
            if self.observed_signal_without_region {
                return Ok(Proposal::stop(
                    OnlineBehavior::Search,
                    "observed light requires confirmed stop-region geometry before motion",
                ));
            }
            return self.search(now, pose.pose, world);
        };
        self.observed_signal_without_region = false;
        self.light = Some(track);
        let (goal, heading) = self.region_goal(track)?;
        let inside = inside_region(self.config.footprint, pose.pose, track);
        if !region_boundary(track, true)?.contains_footprint(self.config.footprint, pose.pose, 0.0)
        {
            return Err("observed stop-region far edge crossed without green".into());
        }
        if self.phase == MissionPhase::WaitGreen && !inside {
            return Err("vehicle left observed light stop region".into());
        }
        if self.stopped(pose) && self.near(pose.pose, goal, heading) && inside {
            if self.phase != MissionPhase::WaitGreen {
                self.phase = MissionPhase::WaitGreen;
                self.light_wait_since = Some(pose.captured_at);
                self.reset_green();
            }
            if road.light != LightState::Green
                || road.light_confidence < self.config.min_detection_confidence
                || road.captured_at < self.light_wait_since.expect("initialized wait")
            {
                self.reset_green();
                return Ok(Proposal::stop(
                    OnlineBehavior::WaitGreen,
                    "waiting for fresh confirmed green inside observed region",
                ));
            }
            if new_road {
                let since = *self.green_since.get_or_insert(road.captured_at);
                self.green_elapsed_ms = road.captured_at.0 - since.0;
                self.green_frames = self.green_frames.saturating_add(1);
            }
            if new_pose
                && self.green_elapsed_ms >= self.config.min_green_ms
                && self.green_frames >= self.config.min_green_frames
            {
                world
                    .mark_processed(track.id)
                    .map_err(|error| error.to_string())?;
                self.transition(MissionPhase::Finish);
                return self.search(now, pose.pose, world);
            }
            return Ok(Proposal::stop(
                OnlineBehavior::WaitGreen,
                "debouncing green after observed stationary stop",
            ));
        }
        if self.phase == MissionPhase::WaitGreen {
            self.light_wait_since = Some(pose.captured_at);
            self.reset_green();
            return Ok(Proposal::stop(
                OnlineBehavior::WaitGreen,
                "waiting for whole-body stationary stop",
            ));
        }
        Ok(self.local_target(
            now,
            pose.pose,
            goal,
            Some(heading),
            None,
            true,
            OnlineBehavior::ApproachRegion,
            "approaching observed light stop region",
            world,
        ))
    }

    fn finish_step(
        &mut self,
        now: Timestamp,
        pose: &PoseEstimate,
        road: &RoadObservation,
        world: &mut LocalWorld,
    ) -> Result<Proposal, String> {
        let mut light = self
            .light
            .ok_or("finish has no previously observed light stop region")?;
        if !self.light_cleared {
            if road.light != LightState::Green
                || road.light_confidence < self.config.min_detection_confidence
            {
                return Err(
                    "green lost before entire vehicle cleared observed light boundary".into(),
                );
            }
            if let Some(current) = world.tracks(now).find(|track| track.id == light.id) {
                light = current;
                self.light = Some(current);
                self.light_missing_since = None;
            } else {
                let since = *self.light_missing_since.get_or_insert(now);
                if now.0 - since.0 >= self.config.max_track_loss_ms {
                    return Err("light region lost before entire vehicle clearance".into());
                }
                return Ok(Proposal::stop(
                    OnlineBehavior::Search,
                    "awaiting fresh light-region geometry before clearing it",
                ));
            }
        }
        let boundary = region_boundary(light, true)?;
        let (_, rear) = boundary.footprint_progress(self.config.footprint, pose.pose);
        self.light_cleared |= rear > boundary.max_projection_m() + 2.0 * region_error(light);
        if !self.light_cleared
            && (road.light != LightState::Green
                || road.light_confidence < self.config.min_detection_confidence)
        {
            return Err("green lost before entire vehicle cleared observed light boundary".into());
        }
        let Some(track) = self.acquire(now, pose.pose, ElementKind::FinishMarker, world) else {
            return self.search(now, pose.pose, world);
        };
        let (goal, heading) = self.region_goal(track)?;
        let reached_finish = self.near(pose.pose, goal, heading)
            && self.stopped(pose)
            && inside_region(self.config.footprint, pose.pose, track);
        // Independently observed marker kinds do not establish their order.
        // Reaching an overlapping finish goal while still behind the uncertain
        // light edge cannot complete the task, or provide a useful next target.
        if reached_finish && !self.light_cleared {
            return Err("observed finish goal reached before whole-vehicle light clearance".into());
        }
        if self.light_cleared && reached_finish {
            world
                .mark_processed(track.id)
                .map_err(|error| error.to_string())?;
            self.phase = MissionPhase::Completed;
            return Ok(Proposal::stop(
                OnlineBehavior::Completed,
                "observed finish contains whole stationary vehicle",
            ));
        }
        Ok(self.local_target(
            now,
            pose.pose,
            goal,
            Some(heading),
            None,
            true,
            OnlineBehavior::ApproachRegion,
            "approaching observed finish region",
            world,
        ))
    }

    fn acquire(
        &mut self,
        now: Timestamp,
        pose: Pose2,
        kind: ElementKind,
        world: &LocalWorld,
    ) -> Option<ElementTrack> {
        if self.active.is_none() {
            let track = self.select_track(now, pose, kind, world)?;
            self.active = Some(track.id);
            self.active_geometry = Some(track);
            self.search_since = None;
            self.search_distance = 0.0;
        }
        self.active_geometry
    }

    fn select_track(
        &self,
        now: Timestamp,
        pose: Pose2,
        kind: ElementKind,
        world: &LocalWorld,
    ) -> Option<ElementTrack> {
        let heading = self.search_heading();
        let reference = Pose2 {
            yaw_rad: heading,
            ..pose
        };
        world
            .tracks(now)
            .filter(|track| {
                track.kind == kind
                    && !track.processed
                    && track.confidence >= self.config.min_detection_confidence
            })
            .filter(|track| {
                let relative = reference.world_to_body(track.position);
                if kind == ElementKind::Cone {
                    let color_matches = self.config.required_cone_colors[self.cone_index.min(1)]
                        .is_none_or(|color| color == track.color);
                    color_matches
                        && relative.x_m > self.config.goal_tolerance_m
                        // The second cone may be on either lateral side of
                        // the reverse departure direction when lane widths or
                        // cone offsets differ. Acquisition does not determine
                        // its required clockwise passing side.
                        && (self.cone_index == 1 || relative.y_m >= -track.position_error_m)
                        && relative.y_m.abs() <= relative.x_m
                        && !self.processed.iter().flatten().any(|old| {
                            old.position.distance(track.position)
                                <= self.config.processed_exclusion_m.max(old.radius_m * 2.0)
                                    + old.position_error_m
                                    + track.position_error_m
                        })
                } else {
                    relative.x_m >= -self.config.search_horizon_m
                        && track
                            .heading_rad
                            .is_some_and(|yaw| wrap(yaw - heading).abs() <= FRAC_PI_4)
                }
            })
            .min_by(|a, b| {
                let ar = reference.world_to_body(a.position);
                let br = reference.world_to_body(b.position);
                if kind == ElementKind::Cone {
                    ar.y_m
                        .atan2(ar.x_m)
                        .abs()
                        .total_cmp(&br.y_m.atan2(br.x_m).abs())
                        .then_with(|| ar.x_m.total_cmp(&br.x_m))
                } else {
                    pose.point()
                        .distance(a.position)
                        .total_cmp(&pose.point().distance(b.position))
                }
            })
    }

    fn search(
        &mut self,
        now: Timestamp,
        pose: Pose2,
        world: &LocalWorld,
    ) -> Result<Proposal, String> {
        let since = *self.search_since.get_or_insert(now);
        if now.0 - since.0 >= self.config.max_search_ms
            || self.search_distance >= self.config.max_search_distance_m
        {
            return Err(
                "bounded element search exhausted without an identified next task element".into(),
            );
        }
        let heading = self.search_heading();
        // From the bottom lane, expose the first cone's interior sector. After
        // its half-turn we are already inside the field: the second cone can
        // be on either side of the reverse axis. Continuing a left bias there
        // would progressively steer away from a valid right-forward landmark.
        // These task-relative directions never accumulate the chosen ego yaw.
        let offsets: &[f64] =
            if self.phase == MissionPhase::Cones && self.active.is_none() && self.cone_index == 0 {
                &[PI / 6.0, PI / 12.0, 0.0]
            } else {
                &[0.0]
            };
        for &offset in offsets {
            for scale in [1.0, 0.75, 0.5, 0.25] {
                let goal = polar(
                    pose.point(),
                    self.config.search_horizon_m * scale,
                    heading + offset,
                );
                if world.known_free_segment(now, pose.point(), goal, self.body_padding()) {
                    return Ok(Proposal {
                        output: MissionOutput::Target {
                            point: goal,
                            max_speed_mps: self.config.approach_speed_mps,
                            arrival: ArrivalBehavior::Stop,
                        },
                        // Exploration needs a bounded direction, not an exact
                        // terminal pose/S-curve on every moving short target.
                        heading: (offset == 0.0).then_some(heading),
                        behavior: OnlineBehavior::Search,
                        reason: if offset == 0.0 {
                            "bounded search along task departure direction in observed free space"
                        } else {
                            "bounded left-forward cone search in observed free space"
                        },
                    });
                }
            }
        }
        Ok(Proposal::stop(
            OnlineBehavior::Search,
            "search corridor is blocked or unknown",
        ))
    }

    #[allow(clippy::too_many_arguments)]
    fn local_target(
        &self,
        now: Timestamp,
        pose: Pose2,
        goal: Point2,
        heading: Option<f64>,
        continuation: Option<(Point2, f64)>,
        approach: bool,
        behavior: OnlineBehavior,
        reason: &'static str,
        world: &LocalWorld,
    ) -> Proposal {
        let distance = pose.point().distance(goal);
        // Oriented arrivals need room for both lateral approach and tangent
        // alignment. Keep two local horizons for that certified Navigator
        // connection; unoriented search retains its single rolling horizon.
        let target_horizon =
            self.config.search_horizon_m * if heading.is_some() { 2.0 } else { 1.0 };
        if distance > target_horizon {
            let yaw = (goal.y_m - pose.y_m).atan2(goal.x_m - pose.x_m);
            for scale in [1.0, 0.75, 0.5, 0.25] {
                let point = polar(pose.point(), self.config.search_horizon_m * scale, yaw);
                if world.known_free_segment(now, pose.point(), point, self.body_padding()) {
                    return Proposal {
                        output: MissionOutput::Target {
                            point,
                            max_speed_mps: self.config.approach_speed_mps,
                            arrival: ArrivalBehavior::Stop,
                        },
                        heading: None,
                        behavior,
                        reason,
                    };
                }
            }
            return Proposal::stop(behavior, "approach corridor not yet observed free");
        }
        let target_known_free = heading.map_or_else(
            || world.known_free_segment(now, goal, goal, self.body_padding()),
            |yaw_rad| {
                world.known_free_convex_hull(
                    now,
                    &self.config.footprint.corners(Pose2 {
                        x_m: goal.x_m,
                        y_m: goal.y_m,
                        yaw_rad,
                    }),
                    self.config.clearance_m,
                )
            },
        );
        if !target_known_free {
            return Proposal::stop(behavior, "short target body envelope is unknown or blocked");
        }
        let speed = if approach {
            self.config.approach_speed_mps
        } else {
            self.config.cruise_speed_mps
        };
        let arrival = continuation
            .filter(|(next, _)| {
                *next != goal && world.known_free_segment(now, goal, *next, self.body_padding())
            })
            .map_or(ArrivalBehavior::Stop, |(next, next_heading)| {
                ArrivalBehavior::PassThrough {
                    next,
                    next_heading_rad: Some(next_heading),
                    next_max_speed_mps: speed,
                    admission_radius_m: self.config.goal_tolerance_m,
                }
            });
        Proposal {
            output: MissionOutput::Target {
                point: goal,
                max_speed_mps: speed,
                arrival,
            },
            heading,
            behavior,
            reason,
        }
    }

    /// Every footprint corner follows a circular arc about the observed
    /// reference center. Endpoint convex hull plus its maximum sagitta contains
    /// each corner's complete arc, and hence every intermediate rectangle.
    #[allow(clippy::too_many_arguments)]
    fn arc_space_state(
        &self,
        now: Timestamp,
        world: &LocalWorld,
        center: Point2,
        radius: f64,
        start: f64,
        sweep: f64,
    ) -> SpaceState {
        if !center.valid()
            || !radius.is_finite()
            || radius <= 0.0
            || !start.is_finite()
            || !sweep.is_finite()
        {
            return SpaceState::Unknown;
        }
        // At the completed half-turn there is still a physical footprint,
        // even though no circular travel remains before the exit tangent.
        // Prove that rectangle instead of declaring an empty arc Unknown and
        // abruptly changing a valid rolling exit's speed request. Signed zero
        // retains the prescribed clockwise/counterclockwise tangent direction.
        let pieces = ((sweep.abs() / (PI / 16.0)).ceil() as usize).max(1);
        if pieces > 8 {
            return SpaceState::Unknown;
        }
        let delta = sweep / pieces as f64;
        let direction = sweep.signum();
        let footprint = self.config.footprint;
        let outer_radius =
            (radius + footprint.half_width_m).hypot(footprint.front_m.max(footprint.rear_m));
        let padding = self.config.clearance_m + outer_radius * (1.0 - (delta * 0.5).cos());
        let corners = |angle: f64| {
            let point = polar(center, radius, angle);
            footprint.corners(Pose2 {
                x_m: point.x_m,
                y_m: point.y_m,
                yaw_rad: wrap(angle + direction * FRAC_PI_2),
            })
        };
        let mut state = SpaceState::KnownFree;
        for i in 0..pieces {
            let a = corners(start + i as f64 * delta);
            let b = corners(start + (i + 1) as f64 * delta);
            let points = [a[0], a[1], a[2], a[3], b[0], b[1], b[2], b[3]];
            match world.convex_hull_space_state(now, &points, padding) {
                SpaceState::Obstacle => return SpaceState::Obstacle,
                SpaceState::Unknown => state = SpaceState::Unknown,
                SpaceState::KnownFree => {}
            }
        }
        state
    }

    #[cfg(test)]
    fn arc_is_known_free(
        &self,
        now: Timestamp,
        world: &LocalWorld,
        center: Point2,
        radius: f64,
        start: f64,
        sweep: f64,
    ) -> bool {
        self.arc_space_state(now, world, center, radius, start, sweep) == SpaceState::KnownFree
    }

    fn region_goal(&self, track: ElementTrack) -> Result<(Point2, f64), String> {
        let ElementGeometry::LineRegion {
            lateral_half_width_m,
            depth_m,
        } = track.geometry
        else {
            return Err("region track has incompatible geometry".into());
        };
        let heading = track
            .heading_rad
            .ok_or("region has no observed direction")?;
        let goal = frame(track).body_to_world(Point2 {
            x_m: (depth_m + self.config.footprint.rear_m - self.config.footprint.front_m) * 0.5,
            y_m: 0.0,
        });
        if lateral_half_width_m <= self.config.footprint.half_width_m
            || !inside_region(
                self.config.footprint,
                Pose2 {
                    x_m: goal.x_m,
                    y_m: goal.y_m,
                    yaw_rad: heading,
                },
                track,
            )
        {
            return Err("observed region cannot contain original footprint and uncertainty".into());
        }
        Ok((goal, heading))
    }
    fn transition(&mut self, phase: MissionPhase) {
        self.phase = phase;
        self.active = None;
        self.active_geometry = None;
        self.orbit = None;
        self.missing_since = None;
        self.search_since = None;
        self.search_distance = 0.0;
    }
    fn search_heading(&self) -> f64 {
        wrap(
            self.initial_heading.unwrap_or(0.0)
                + if self.phase == MissionPhase::Cones && self.cone_index == 1 {
                    PI
                } else {
                    0.0
                },
        )
    }
    fn near(&self, pose: Pose2, point: Point2, heading: f64) -> bool {
        pose.point().distance(point) <= self.config.goal_tolerance_m
            && wrap(pose.yaw_rad - heading).abs() <= self.config.goal_heading_tolerance_rad
    }
    fn body_padding(&self) -> f64 {
        self.config
            .footprint
            .front_m
            .max(self.config.footprint.rear_m)
            .hypot(self.config.footprint.half_width_m)
            + self.config.clearance_m
    }
    fn stopped(&self, pose: &PoseEstimate) -> bool {
        let radius = self
            .config
            .footprint
            .front_m
            .max(self.config.footprint.rear_m)
            .hypot(self.config.footprint.half_width_m);
        pose.speed_mps.abs() + pose.yaw_rate_radps.abs() * radius <= self.config.stopped_speed_mps
    }
    fn reset_green(&mut self) {
        self.green_since = None;
        self.green_frames = 0;
        self.green_elapsed_ms = 0;
    }
    fn travel_boundary(&self) -> Option<HalfPlane> {
        if matches!(
            self.phase,
            MissionPhase::ApproachCrosswalk | MissionPhase::CrosswalkStop
        ) && let Some(track) = self.active_geometry
        {
            return region_boundary(track, false).ok();
        }
        if !matches!(self.phase, MissionPhase::Finish | MissionPhase::Completed)
            && let Some(track) = self.light
        {
            return region_boundary(track, true).ok();
        }
        None
    }
    fn finish_tick(
        &mut self,
        now: Timestamp,
        pose: &PoseEstimate,
        new_pose: bool,
        proposal: Proposal,
    ) -> OnlineMissionReport {
        if !matches!(
            self.phase,
            MissionPhase::Idle
                | MissionPhase::CrosswalkStop
                | MissionPhase::WaitGreen
                | MissionPhase::Completed
        ) && self.stopped(pose)
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
        self.report(now, proposal)
    }
    fn report(&self, at: Timestamp, proposal: Proposal) -> OnlineMissionReport {
        OnlineMissionReport {
            mission: MissionReport {
                at,
                phase: self.phase,
                output: proposal.output,
                reason: self
                    .fault_reason
                    .clone()
                    .unwrap_or_else(|| proposal.reason.into()),
                crosswalk_stop_elapsed_ms: self.stop_elapsed_ms,
                green_elapsed_ms: self.green_elapsed_ms,
                waypoint_index: self.cone_index,
            },
            goal_heading_rad: proposal.heading,
            travel_boundary: self.travel_boundary(),
            behavior: proposal.behavior,
            active_track_id: self.active,
            active_track: self.active_geometry,
            processed_cones: self.cone_index,
            cone_progress_rad: self.orbit.map_or(0.0, |orbit| orbit.progress_rad),
            search_distance_m: self.search_distance,
            execution_space_rejected: false,
        }
    }
    fn fault(&mut self, at: Timestamp, reason: impl Into<String>) -> OnlineMissionReport {
        self.phase = MissionPhase::Fault;
        self.fault_reason.get_or_insert_with(|| reason.into());
        self.report(
            self.last_now.map_or(at, |old| old.max(at)),
            Proposal::stop(OnlineBehavior::Fault, "fault latched"),
        )
    }
    fn validate_input(
        &self,
        now: Timestamp,
        pose: &PoseEstimate,
        road: &RoadObservation,
        world: &LocalWorld,
    ) -> Result<(), String> {
        if self.last_now.is_some_and(|old| now < old)
            || pose.frame_id != self.config.world_frame
            || road.frame_id != self.config.body_frame
            || world.config().world_frame != self.config.world_frame
            || world.config().body_frame != self.config.body_frame
            || !pose.pose.valid()
            || !pose.speed_mps.is_finite()
            || !pose.yaw_rate_radps.is_finite()
            || !pose.quality.is_finite()
            || !(self.config.min_pose_quality..=1.0).contains(&pose.quality)
            || !road.light_confidence.is_finite()
            || !(0.0..=1.0).contains(&road.light_confidence)
            || pose.captured_at > now
            || road.captured_at > now
            || now.0.saturating_sub(pose.captured_at.0) >= self.config.max_pose_age_ms
            || now.0.saturating_sub(road.captured_at.0) >= self.config.max_road_age_ms
            || pose.captured_at.0.abs_diff(road.captured_at.0) > self.config.max_observation_skew_ms
        {
            return Err("invalid, stale or mismatched online mission observation".into());
        }
        if let Some(old) = &self.last_pose
            && (pose.captured_at < old.captured_at
                || (pose.captured_at == old.captured_at && pose != old)
                || pose.captured_at.0 - old.captured_at.0 > self.config.max_pose_gap_ms)
        {
            return Err("pose timestamp/continuity invalid in online mission".into());
        }
        if let Some(old) = &self.last_road
            && (road.captured_at < old.captured_at
                || (road.captured_at == old.captured_at && road != old)
                || road.captured_at.0 - old.captured_at.0 > self.config.max_road_gap_ms)
        {
            return Err("road timestamp/continuity invalid in online mission".into());
        }
        Ok(())
    }
}

fn wrap(angle: f64) -> f64 {
    (angle.rem_euclid(TAU) + PI).rem_euclid(TAU) - PI
}
fn is_false(value: &bool) -> bool {
    !value
}
fn polar(origin: Point2, length: f64, angle: f64) -> Point2 {
    Point2 {
        x_m: origin.x_m + length * angle.cos(),
        y_m: origin.y_m + length * angle.sin(),
    }
}
fn frame(track: ElementTrack) -> Pose2 {
    Pose2 {
        x_m: track.position.x_m,
        y_m: track.position.y_m,
        yaw_rad: track.heading_rad.unwrap_or(0.0),
    }
}
fn inside_region(footprint: Footprint, pose: Pose2, track: ElementTrack) -> bool {
    let ElementGeometry::LineRegion {
        lateral_half_width_m,
        depth_m,
    } = track.geometry
    else {
        return false;
    };
    let margin = region_error(track);
    footprint
        .corners(pose)
        .into_iter()
        .map(|point| frame(track).world_to_body(point))
        .all(|point| {
            point.x_m >= margin
                && point.x_m <= depth_m - margin
                && point.y_m.abs() <= lateral_half_width_m - margin
        })
}
/// Build the uncertainty-conservative near or far permission edge of an
/// observed region. The caller must validate observation freshness separately;
/// this geometry helper never confirms a track or grants crossing permission.
pub fn observed_region_boundary(track: ElementTrack, far: bool) -> Result<HalfPlane, String> {
    region_boundary(track, far)
}

fn region_boundary(track: ElementTrack, far: bool) -> Result<HalfPlane, String> {
    let ElementGeometry::LineRegion {
        lateral_half_width_m,
        depth_m,
    } = track.geometry
    else {
        return Err("boundary track is not a line region".into());
    };
    let heading = track.heading_rad.ok_or("line boundary has no heading")?;
    let origin = frame(track);
    let error = region_error(track);
    let lateral_half_width_m = lateral_half_width_m + error;
    let corners = [
        Point2 {
            x_m: 0.0,
            y_m: -lateral_half_width_m,
        },
        Point2 {
            x_m: 0.0,
            y_m: lateral_half_width_m,
        },
        Point2 {
            x_m: depth_m,
            y_m: -lateral_half_width_m,
        },
        Point2 {
            x_m: depth_m,
            y_m: lateral_half_width_m,
        },
    ]
    .map(|point| origin.body_to_world(point));
    let region = Rect {
        min_x_m: corners.iter().map(|p| p.x_m).fold(f64::INFINITY, f64::min),
        max_x_m: corners
            .iter()
            .map(|p| p.x_m)
            .fold(f64::NEG_INFINITY, f64::max),
        min_y_m: corners.iter().map(|p| p.y_m).fold(f64::INFINITY, f64::min),
        max_y_m: corners
            .iter()
            .map(|p| p.y_m)
            .fold(f64::NEG_INFINITY, f64::max),
    };
    HalfPlane::new(
        track.position,
        heading,
        (if far { depth_m } else { 0.0 }) - error,
    )
    .and_then(|boundary| boundary.with_lateral_region(region))
    .map_err(|error| error.to_string())
}

/// Bound the displacement of the finite observed region's corners under the
/// track's position/yaw uncertainty. Rotation displaces a point by at most r*dθ.
fn region_error(track: ElementTrack) -> f64 {
    let ElementGeometry::LineRegion {
        lateral_half_width_m,
        depth_m,
    } = track.geometry
    else {
        return f64::INFINITY;
    };
    track.position_error_m + depth_m.hypot(lateral_half_width_m) * track.heading_error_rad
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::LidarSample;
    use crate::local_world::{
        ElementFrame, ElementObservation, LocalWorldConfig, ObservationSource,
    };

    fn legacy() -> MissionConfig {
        let raw: serde_json::Value =
            serde_json::from_str(include_str!("../../../config/competition-sim.json")).unwrap();
        serde_json::from_value(raw["autonomy"]["mission"].clone()).unwrap()
    }
    fn setup() -> (OnlineMission, LocalWorld) {
        let config = OnlineMissionConfig::from_limits(&legacy());
        let mut world_config = LocalWorldConfig::simulation(
            config.body_frame.clone(),
            FrameId("sim_laser".into()),
            config.world_frame.clone(),
        );
        world_config.self_footprint = Some(config.footprint);
        // Exact synthetic policy inputs; these tests do not claim physical
        // calibration, image perception, or actuator-trajectory validation.
        world_config.pose_position_error_m = 0.0;
        world_config.pose_heading_error_rad = 0.0;
        let mut mission = OnlineMission::new(config).unwrap();
        mission.start().unwrap();
        (mission, LocalWorld::new(world_config).unwrap())
    }
    fn pose(at: u64, pose: Pose2) -> PoseEstimate {
        PoseEstimate {
            captured_at: Timestamp(at),
            frame_id: FrameId("sim_world".into()),
            pose,
            speed_mps: 0.0,
            yaw_rate_radps: 0.0,
            quality: 1.0,
        }
    }
    fn road(at: u64, light: LightState) -> RoadObservation {
        RoadObservation {
            captured_at: Timestamp(at),
            frame_id: FrameId("sim_body".into()),
            crosswalk: None,
            light,
            light_confidence: 1.0,
            cones_body_m: vec![],
        }
    }
    fn region(kind: ElementKind, x: f64, y: f64, depth: f64) -> ElementObservation {
        ElementObservation {
            kind,
            color: ElementColor::White,
            position_body_m: Point2 { x_m: x, y_m: y },
            heading_body_rad: Some(0.0),
            geometry: ElementGeometry::LineRegion {
                lateral_half_width_m: 0.6,
                depth_m: depth,
            },
            source: ObservationSource::GroundMarker,
            confidence: 0.99,
            position_error_m: 0.001,
            heading_error_rad: 0.0,
        }
    }
    fn cone(x: f64, y: f64, color: ElementColor) -> ElementObservation {
        ElementObservation {
            kind: ElementKind::Cone,
            color,
            position_body_m: Point2 { x_m: x, y_m: y },
            heading_body_rad: None,
            geometry: ElementGeometry::Cone { radius_m: 0.2 },
            source: ObservationSource::VisualLidar,
            confidence: 0.99,
            position_error_m: 0.001,
            heading_error_rad: 0.0,
        }
    }
    /// Landmarks passed here are world-coordinate TEST observations, projected
    /// into each supplied capture pose before using the public tracker API.
    fn observe(
        world: &mut LocalWorld,
        estimate: &PoseEstimate,
        observations: &[ElementObservation],
        free_scan: bool,
    ) {
        let frame = ElementFrame {
            captured_at: estimate.captured_at,
            frame_id: FrameId("sim_body".into()),
            observations: observations
                .iter()
                .map(|observed| ElementObservation {
                    position_body_m: estimate.pose.world_to_body(observed.position_body_m),
                    heading_body_rad: observed
                        .heading_body_rad
                        .map(|heading| wrap(heading - estimate.pose.yaw_rad)),
                    ..*observed
                })
                .collect(),
        };
        world
            .update(estimate.captured_at, estimate, &frame)
            .unwrap();
        let scan = LidarSample {
            captured_at: estimate.captured_at,
            frame_id: FrameId("sim_laser".into()),
            angle_min_rad: 0.0,
            angle_increment_rad: TAU / 720.0,
            range_min_m: 0.05,
            range_max_m: 20.0,
            ranges_m: vec![free_scan.then_some(5.0); 720],
        };
        world
            .update_scan(estimate.captured_at, estimate, &scan, Pose2::default())
            .unwrap();
    }

    #[test]
    fn limits_copy_does_not_read_legacy_geometry_and_does_not_weaken_rule_limits() {
        let mut source = legacy();
        let before = serde_json::to_value(OnlineMissionConfig::from_limits(&source)).unwrap();
        source.approach_goal.x_m = f64::NAN;
        source.cone_waypoints.clear();
        source.light_stop_goal.x_m = -999.0;
        source.finish_goal.y_m = 999.0;
        source.crosswalk_region.min_x_m = 999.0;
        assert_eq!(
            before,
            serde_json::to_value(OnlineMissionConfig::from_limits(&source)).unwrap()
        );
        let mut config = OnlineMissionConfig::from_limits(&source);
        assert!(config.validate().is_ok());
        config.crosswalk_hold_ms = 2999;
        assert!(config.validate().is_err());
        config.crosswalk_hold_ms = 3000;
        config.min_green_ms = 299;
        assert!(config.validate().is_err());
    }

    #[test]
    fn unknown_search_space_stops_and_preserves_the_ordinary_ten_second_limit() {
        let (mut mission, mut world) = setup();
        for at in (0..=10_100).step_by(100) {
            let estimate = pose(at, Pose2::default());
            observe(&mut world, &estimate, &[], false);
            let report = mission.update(
                Timestamp(at),
                &estimate,
                &road(at, LightState::Unknown),
                &mut world,
            );
            assert_eq!(report.mission.output, MissionOutput::Stop);
            assert_eq!(report.active_track_id, None);
            if at <= 10_000 {
                assert_eq!(report.mission.phase, MissionPhase::ApproachCrosswalk);
            } else {
                assert_eq!(report.mission.phase, MissionPhase::Fault);
                assert!(report.mission.reason.contains("10 seconds"));
            }
        }
    }

    #[test]
    fn cone_search_bias_only_for_first_task_and_does_not_accumulate_vehicle_yaw() {
        for axis in [0.0, 0.6] {
            for index in [0, 1] {
                let (mut mission, mut world) = setup();
                mission.phase = MissionPhase::Cones;
                mission.cone_index = index;
                mission.initial_heading = Some(axis);
                let departure = axis + if index == 0 { 0.0 } else { PI };
                for (at, extra_yaw) in [(100, 0.0), (200, 0.2)] {
                    let estimate = pose(
                        at,
                        Pose2 {
                            x_m: 2.0,
                            y_m: 2.0,
                            yaw_rad: wrap(departure + extra_yaw),
                        },
                    );
                    observe(&mut world, &estimate, &[], true);
                    let report = mission.update(
                        Timestamp(at),
                        &estimate,
                        &road(at, LightState::Unknown),
                        &mut world,
                    );
                    let MissionOutput::Target {
                        point,
                        max_speed_mps,
                        arrival,
                    } = report.mission.output
                    else {
                        panic!("observed free search sector must supply a target");
                    };
                    let expected = polar(
                        estimate.pose.point(),
                        mission.config.search_horizon_m,
                        departure + if index == 0 { PI / 6.0 } else { 0.0 },
                    );
                    assert!(point.distance(expected) < 1e-12);
                    assert_eq!(max_speed_mps, mission.config.approach_speed_mps);
                    assert_eq!(arrival, ArrivalBehavior::Stop);
                    assert_eq!(
                        report.goal_heading_rad,
                        (index == 1).then_some(wrap(departure))
                    );
                    assert!(matches!(report.behavior, OnlineBehavior::Search));
                    assert!(world.known_free_segment(
                        Timestamp(at),
                        estimate.pose.point(),
                        point,
                        mission.body_padding()
                    ));
                }
                let estimate = pose(
                    300,
                    Pose2 {
                        x_m: 2.0,
                        y_m: 2.0,
                        yaw_rad: wrap(departure),
                    },
                );
                observe(&mut world, &estimate, &[], false);
                let report = mission.update(
                    Timestamp(300),
                    &estimate,
                    &road(300, LightState::Unknown),
                    &mut world,
                );
                assert_eq!(report.mission.output, MissionOutput::Stop);
            }
        }
    }

    #[test]
    fn crosswalk_target_uses_the_same_finite_region_uncertainty_as_its_boundary() {
        for heading in [0.0, 0.7] {
            let (mut mission, mut world) = setup();
            let marker_frame = Pose2 {
                x_m: 2.0,
                y_m: 1.0,
                yaw_rad: heading,
            };
            let approach = marker_frame.body_to_world(Point2 {
                x_m: -1.0,
                y_m: 0.0,
            });
            let source = Pose2 {
                x_m: approach.x_m,
                y_m: approach.y_m,
                yaw_rad: heading,
            };
            let mut marker = region(ElementKind::Crosswalk, 2.0, 1.0, 0.3);
            marker.heading_body_rad = Some(heading);
            marker.heading_error_rad = 0.2;
            marker.geometry = ElementGeometry::LineRegion {
                lateral_half_width_m: 1.0,
                depth_m: 0.3,
            };
            for at in [100, 200] {
                observe(&mut world, &pose(at, source), &[marker], true);
            }
            let track = world
                .tracks(Timestamp(200))
                .find(|track| track.kind == ElementKind::Crosswalk)
                .unwrap();
            assert!(track.observations >= 2 && track.valid);
            let boundary = region_boundary(track, false).unwrap();
            // This legal heading error shifts a wide region's near edge by
            // more than the unchanged 15 cm stop margin. A position-only
            // target would put the actual front corners past that edge.
            let old_point = marker_frame.body_to_world(Point2 {
                x_m: -mission.config.footprint.front_m
                    - mission.config.crosswalk_stop_margin_m
                    - track.position_error_m,
                y_m: 0.0,
            });
            assert!(!boundary.contains_footprint(
                mission.config.footprint,
                Pose2 {
                    x_m: old_point.x_m,
                    y_m: old_point.y_m,
                    yaw_rad: heading
                },
                0.0
            ));
            let report = mission.update(
                Timestamp(200),
                &pose(200, source),
                &road(200, LightState::Unknown),
                &mut world,
            );
            let MissionOutput::Target { point, .. } = report.mission.output else {
                panic!("observed, free crosswalk approach should have a target");
            };
            let local_goal = marker_frame.world_to_body(point);
            let expected = -mission.config.footprint.front_m
                - mission.config.crosswalk_stop_margin_m
                - region_error(track);
            assert!((local_goal.x_m - expected).abs() < 1e-12);
            assert!(local_goal.y_m.abs() < 1e-12);
            assert!(wrap(report.goal_heading_rad.unwrap() - heading).abs() < 1e-12);
            let stopped_pose = Pose2 {
                x_m: point.x_m,
                y_m: point.y_m,
                yaw_rad: heading,
            };
            assert!(boundary.contains_footprint(
                mission.config.footprint,
                stopped_pose,
                mission.config.clearance_m
            ));
            observe(&mut world, &pose(300, stopped_pose), &[marker], true);
            let stopped = mission.update(
                Timestamp(300),
                &pose(300, stopped_pose),
                &road(300, LightState::Unknown),
                &mut world,
            );
            assert_eq!(stopped.mission.phase, MissionPhase::CrosswalkStop);
            assert_eq!(stopped.mission.output, MissionOutput::Stop);
            assert_eq!(stopped.mission.crosswalk_stop_elapsed_ms, 0);
        }
    }

    #[test]
    fn crosswalk_requires_confirmed_geometry_and_three_seconds_of_fresh_stationary_poses() {
        let (mut mission, mut world) = setup();
        let position = Pose2 {
            x_m: 2.0 - 0.22 - 0.15 - 0.001,
            y_m: 0.0,
            yaw_rad: 0.0,
        };
        let marker = region(ElementKind::Crosswalk, 2.0, 0.0, 0.3);
        for at in (100..=3200).step_by(100) {
            let estimate = pose(at, position);
            observe(&mut world, &estimate, &[marker], true);
            let report = mission.update(
                Timestamp(at),
                &estimate,
                &road(at, LightState::Unknown),
                &mut world,
            );
            if at == 100 {
                assert_eq!(report.active_track_id, None);
            }
            if (200..3200).contains(&at) {
                assert_eq!(report.mission.phase, MissionPhase::CrosswalkStop);
            }
            if at == 200 {
                let repeated = mission.update(
                    Timestamp(250),
                    &estimate,
                    &road(at, LightState::Unknown),
                    &mut world,
                );
                assert_eq!(repeated.mission.crosswalk_stop_elapsed_ms, 0);
            }
            if at == 3200 {
                assert_eq!(report.mission.phase, MissionPhase::Cones);
                assert_eq!(report.mission.crosswalk_stop_elapsed_ms, 3000);
            }
        }
        assert!(
            world
                .tracks(Timestamp(3200))
                .any(|track| track.kind == ElementKind::Crosswalk && track.processed)
        );
    }

    #[test]
    fn lamp_color_alone_cannot_create_a_stop_region_or_skip_the_task() {
        for light in [
            LightState::Red,
            LightState::Yellow,
            LightState::Green,
            LightState::Conflicting,
        ] {
            let (mut mission, mut world) = setup();
            mission.phase = MissionPhase::ApproachLight;
            for at in (100..=10_200).step_by(100) {
                let estimate = pose(at, Pose2::default());
                observe(&mut world, &estimate, &[], true);
                // One trustworthy signal sighting is sufficient to require
                // geometry; later Unknown frames cannot release the stop.
                let seen_light = if at == 100 {
                    light
                } else {
                    LightState::Unknown
                };
                let report =
                    mission.update(Timestamp(at), &estimate, &road(at, seen_light), &mut world);
                assert_eq!(
                    report.mission.phase,
                    if at <= 10_100 {
                        MissionPhase::ApproachLight
                    } else {
                        MissionPhase::Fault
                    }
                );
                assert_eq!(report.mission.output, MissionOutput::Stop);
                assert_eq!(report.goal_heading_rad, None);
                assert_eq!(report.mission.green_elapsed_ms, 0);
                assert_eq!(report.active_track_id, None);
                if at == 10_200 {
                    assert!(report.mission.reason.contains("10 seconds"));
                }
            }
        }
    }

    #[test]
    fn reported_boundary_carries_position_and_heading_uncertainty_into_navigation() {
        let (_, mut world) = setup();
        let mut observed = region(ElementKind::StopLine, 0.0, 0.0, 1.0);
        observed.position_error_m = 0.05;
        observed.heading_error_rad = 0.1;
        for at in [100, 200] {
            observe(&mut world, &pose(at, Pose2::default()), &[observed], true);
        }
        let track = world.tracks(Timestamp(200)).next().unwrap();
        let boundary = region_boundary(track, true).unwrap();
        let margin = 0.05 + 1.0_f64.hypot(0.6) * 0.1;
        assert!((boundary.max_projection_m() - (1.0 - margin)).abs() < 1e-12);
        let footprint = legacy().footprint;
        // Nominal geometry alone would permit this front at x=.96; the
        // uncertainty-propagated boundary must reject it before it is adopted.
        assert!(!boundary.contains_footprint(
            footprint,
            Pose2 {
                x_m: 0.74,
                y_m: 0.0,
                yaw_rad: 0.0
            },
            0.0
        ));
        // Also reject the uncertain lateral edge, outside the nominal .6 band.
        assert!(!boundary.contains_footprint(
            footprint,
            Pose2 {
                x_m: 1.0,
                y_m: 0.75,
                yaw_rad: 0.0
            },
            0.0
        ));
    }

    #[test]
    fn expired_light_geometry_cannot_certify_clearance_even_with_green_and_forward_pose() {
        let (mut mission, mut world) = setup();
        let observed = region(ElementKind::StopLine, 0.0, 0.0, 1.0);
        for at in [100, 200] {
            observe(&mut world, &pose(at, Pose2::default()), &[observed], true);
        }
        mission.light = world.tracks(Timestamp(200)).next();
        mission.phase = MissionPhase::Finish;
        // A policy-level expired-observation test: the forward pose must not
        // be interpreted as proof of clearing an indefinitely cached line.
        let estimate = pose(
            2200,
            Pose2 {
                x_m: 2.0,
                y_m: 0.0,
                yaw_rad: 0.0,
            },
        );
        let proposal = mission
            .finish_step(
                Timestamp(2200),
                &estimate,
                &road(2200, LightState::Green),
                &mut world,
            )
            .unwrap();
        assert_eq!(proposal.output, MissionOutput::Stop);
        assert!(!mission.light_cleared);
        assert!(
            mission
                .finish_step(
                    Timestamp(4200),
                    &pose(4200, estimate.pose),
                    &road(4200, LightState::Green),
                    &mut world
                )
                .is_err()
        );
    }

    #[test]
    fn observed_regions_require_whole_body_stop_and_fresh_green_before_finish() {
        let (mut mission, mut world) = setup();
        mission.phase = MissionPhase::ApproachLight;
        let marker = region(ElementKind::StopLine, 0.0, 0.0, 1.0);
        for at in (100..=500).step_by(100) {
            let estimate = pose(
                at,
                Pose2 {
                    x_m: 0.48,
                    y_m: 0.0,
                    yaw_rad: 0.0,
                },
            );
            observe(&mut world, &estimate, &[marker], true);
            let report = mission.update(
                Timestamp(at),
                &estimate,
                &road(at, LightState::Green),
                &mut world,
            );
            if (200..500).contains(&at) {
                assert_eq!(report.mission.phase, MissionPhase::WaitGreen);
                assert_eq!(report.mission.output, MissionOutput::Stop);
                assert!(report.travel_boundary.is_some());
            }
            if at == 200 {
                let repeated = mission.update(
                    Timestamp(250),
                    &estimate,
                    &road(at, LightState::Green),
                    &mut world,
                );
                assert_eq!(repeated.mission.green_elapsed_ms, 0);
            }
            if at == 500 {
                assert_eq!(report.mission.phase, MissionPhase::Finish);
                assert_eq!(report.mission.green_elapsed_ms, 300);
                assert!(report.travel_boundary.is_none());
            }
        }
        let finish = region(ElementKind::FinishMarker, 1.0, 0.0, 0.8);
        for at in [600, 700] {
            let estimate = pose(
                at,
                Pose2 {
                    x_m: 1.38,
                    y_m: 0.0,
                    yaw_rad: 0.0,
                },
            );
            observe(&mut world, &estimate, &[marker, finish], true);
            let report = mission.update(
                Timestamp(at),
                &estimate,
                &road(at, LightState::Green),
                &mut world,
            );
            if at == 700 {
                assert_eq!(report.mission.phase, MissionPhase::Completed);
                assert_eq!(report.mission.output, MissionOutput::Stop);
            }
        }
    }

    #[test]
    fn finish_completion_requires_clearance_even_when_observed_regions_overlap() {
        for (finish_near, expected) in [
            (0.1, MissionPhase::Fault),
            // Partial region overlap is allowed when the actual whole vehicle
            // has passed the uncertainty-conservative light boundary.
            (0.9, MissionPhase::Completed),
            (1.0, MissionPhase::Completed),
        ] {
            let (mut mission, mut world) = setup();
            mission.phase = MissionPhase::ApproachLight;
            let stop = region(ElementKind::StopLine, 0.0, 0.0, 1.0);
            for at in (100..=500).step_by(100) {
                let estimate = pose(
                    at,
                    Pose2 {
                        x_m: 0.48,
                        ..Pose2::default()
                    },
                );
                observe(&mut world, &estimate, &[stop], true);
                let report = mission.update(
                    Timestamp(at),
                    &estimate,
                    &road(at, LightState::Green),
                    &mut world,
                );
                if at == 500 {
                    assert_eq!(report.mission.phase, MissionPhase::Finish);
                    assert_eq!(report.mission.green_elapsed_ms, 300);
                    assert!(!mission.light_cleared);
                }
            }
            let finish = region(ElementKind::FinishMarker, finish_near, 0.0, 0.8);
            for at in [600, 700] {
                let estimate = pose(
                    at,
                    Pose2 {
                        x_m: finish_near + 0.38,
                        ..Pose2::default()
                    },
                );
                observe(&mut world, &estimate, &[stop, finish], true);
                let report = mission.update(
                    Timestamp(at),
                    &estimate,
                    &road(at, LightState::Green),
                    &mut world,
                );
                if at == 700 {
                    assert_eq!(report.mission.phase, expected, "finish_near={finish_near}");
                    assert_eq!(report.mission.output, MissionOutput::Stop);
                    let track = world
                        .tracks(Timestamp(at))
                        .find(|track| track.kind == ElementKind::FinishMarker)
                        .unwrap();
                    assert_eq!(track.processed, expected == MissionPhase::Completed);
                    assert_eq!(mission.light_cleared, expected == MissionPhase::Completed);
                    if expected == MissionPhase::Fault {
                        assert!(
                            report
                                .mission
                                .reason
                                .contains("before whole-vehicle light clearance")
                        );
                        assert!(inside_region(
                            mission.config.footprint,
                            estimate.pose,
                            mission.light.unwrap()
                        ));
                    }
                }
            }
        }
    }

    #[test]
    fn processed_stop_region_stays_fresh_through_gradual_green_exit_and_finish() {
        let (mut mission, mut world) = setup();
        mission.phase = MissionPhase::ApproachLight;
        let mut stop = region(ElementKind::StopLine, 0.0, 0.0, 1.0);
        stop.geometry = ElementGeometry::LineRegion {
            lateral_half_width_m: 0.4,
            depth_m: 1.0,
        };
        stop.position_error_m = 0.08;
        stop.heading_error_rad = 0.07;
        let mut finish = region(ElementKind::FinishMarker, 1.0, 0.0, 0.8);
        finish.geometry = ElementGeometry::LineRegion {
            lateral_half_width_m: 0.45,
            depth_m: 0.8,
        };
        finish.position_error_m = 0.08;
        finish.heading_error_rad = 0.07;
        let mut stop_id = None;
        let observations = [stop, finish];
        for at in (100u64..=6_700).step_by(100) {
            // Exact policy observations with nonzero geometry uncertainty;
            // this is not a claim that a real camera can see both markers.
            let x = 0.48 + (at.saturating_sub(600) as f64 * 0.00015).min(0.9);
            let mut estimate = pose(
                at,
                Pose2 {
                    x_m: x,
                    y_m: 0.0,
                    yaw_rad: 0.0,
                },
            );
            if (700..6_700).contains(&at) {
                estimate.speed_mps = 0.15;
            }
            observe(
                &mut world,
                &estimate,
                if at <= 600 {
                    std::slice::from_ref(&stop)
                } else {
                    &observations
                },
                true,
            );
            let light = if at <= 200 {
                LightState::Red
            } else {
                LightState::Green
            };
            let report = mission.update(Timestamp(at), &estimate, &road(at, light), &mut world);
            if at == 200 {
                stop_id = report.active_track_id;
            }
            if (200..600).contains(&at) {
                assert_eq!(report.mission.phase, MissionPhase::WaitGreen);
                assert_eq!(report.mission.output, MissionOutput::Stop);
                assert!(report.mission.green_elapsed_ms < 300);
            }
            if at >= 600 {
                let tracked = world
                    .tracks(Timestamp(at))
                    .find(|track| Some(track.id) == stop_id)
                    .unwrap();
                assert!(tracked.processed);
                assert_eq!(tracked.last_geometry_at, Timestamp(at));
                assert_eq!(tracked.last_visual_at, Timestamp(at));
                assert_eq!(tracked.observations as u64, at / 100);
                assert_eq!(report.mission.green_elapsed_ms, 300);
                assert_eq!(
                    report.mission.phase,
                    if at == 6_700 {
                        MissionPhase::Completed
                    } else {
                        MissionPhase::Finish
                    }
                );
                if at < 6_700 {
                    assert!(matches!(
                        report.mission.output,
                        MissionOutput::Target { .. }
                    ));
                }
            }
            if at == 6_700 {
                assert!(mission.light_cleared);
                assert_eq!(report.mission.output, MissionOutput::Stop);
                assert_eq!(estimate.speed_mps, 0.0);
                assert!(inside_region(
                    mission.config.footprint,
                    estimate.pose,
                    mission.active_geometry.unwrap()
                ));
            }
        }
    }

    #[test]
    fn permission_does_not_outlive_green_or_fresh_stop_region_geometry_before_clearance() {
        for failure in [LightState::Unknown, LightState::Red, LightState::Green] {
            let (mut mission, mut world) = setup();
            mission.phase = MissionPhase::ApproachLight;
            let marker = region(ElementKind::StopLine, 0.0, 0.0, 1.0);
            for at in (100..=4_300).step_by(100) {
                let estimate = pose(
                    at,
                    Pose2 {
                        x_m: 0.48,
                        y_m: 0.0,
                        yaw_rad: 0.0,
                    },
                );
                observe(
                    &mut world,
                    &estimate,
                    if at <= 500 || failure != LightState::Green {
                        std::slice::from_ref(&marker)
                    } else {
                        &[]
                    },
                    true,
                );
                let report = mission.update(
                    Timestamp(at),
                    &estimate,
                    &road(
                        at,
                        if at <= 500 {
                            LightState::Green
                        } else {
                            failure
                        },
                    ),
                    &mut world,
                );
                if at == 500 {
                    assert_eq!(report.mission.phase, MissionPhase::Finish);
                    assert_eq!(report.mission.green_elapsed_ms, 300);
                }
                if at >= 600 && failure != LightState::Green {
                    assert_eq!(report.mission.phase, MissionPhase::Fault);
                    assert_eq!(report.mission.output, MissionOutput::Stop);
                    assert!(report.mission.reason.contains("green lost"));
                    break;
                }
                if failure == LightState::Green && at >= 2_300 {
                    assert_eq!(report.mission.output, MissionOutput::Stop);
                    assert!(!mission.light_cleared);
                    assert_eq!(
                        report.mission.phase,
                        if at < 4_300 {
                            MissionPhase::Finish
                        } else {
                            MissionPhase::Fault
                        }
                    );
                    if at == 4_300 {
                        assert!(report.mission.reason.contains("light region lost"));
                    }
                }
            }
        }
    }

    #[test]
    fn confirmed_finish_region_too_narrow_for_original_uncertainty_fails_closed() {
        let (mut mission, mut world) = setup();
        mission.phase = MissionPhase::ApproachLight;
        let stop = region(ElementKind::StopLine, 0.0, 0.0, 1.0);
        let mut finish = region(ElementKind::FinishMarker, 1.0, 0.0, 0.8);
        finish.geometry = ElementGeometry::LineRegion {
            lateral_half_width_m: 0.25,
            depth_m: 0.8,
        };
        finish.position_error_m = 0.15;
        finish.heading_error_rad = 0.15;
        let observations = [stop, finish];
        for at in (100..=700).step_by(100) {
            let estimate = pose(
                at,
                Pose2 {
                    x_m: if at <= 500 { 0.48 } else { 1.38 },
                    y_m: 0.0,
                    yaw_rad: 0.0,
                },
            );
            observe(
                &mut world,
                &estimate,
                if at <= 500 {
                    std::slice::from_ref(&stop)
                } else {
                    &observations
                },
                true,
            );
            let report = mission.update(
                Timestamp(at),
                &estimate,
                &road(at, LightState::Green),
                &mut world,
            );
            if at == 700 {
                assert_eq!(report.mission.phase, MissionPhase::Fault);
                assert_eq!(report.mission.output, MissionOutput::Stop);
                assert!(report.mission.reason.contains("footprint and uncertainty"));
                assert_eq!(mission.active_geometry.unwrap().position_error_m, 0.15);
                assert_eq!(mission.active_geometry.unwrap().heading_error_rad, 0.15);
            }
        }
    }

    #[test]
    fn cone_identity_is_locked_until_actual_correct_side_half_turn_is_completed() {
        let (mut mission, mut world) = setup();
        mission.phase = MissionPhase::Cones;
        let cones = [
            cone(2.0, 1.0, ElementColor::Red),
            cone(0.5, 1.8, ElementColor::Blue),
        ];
        for at in [100, 200] {
            let estimate = pose(at, Pose2::default());
            observe(&mut world, &estimate, &cones, true);
            let report = mission.update(
                Timestamp(at),
                &estimate,
                &road(at, LightState::Unknown),
                &mut world,
            );
            if at == 200 {
                assert!(matches!(report.behavior, OnlineBehavior::ConeEntry));
                let MissionOutput::Target { point, .. } = report.mission.output else {
                    panic!("tracked entrance should be approached");
                };
                // An oriented target inside two horizons preserves the actual
                // tracked tangent guide. It is not the +30-degree search ray.
                let radius = mission.orbit.unwrap().radius_m;
                assert!((point.x_m - (2.0 - mission.config.search_horizon_m)).abs() < 1e-12);
                assert!((point.y_m - (1.0 - radius)).abs() < 1e-12);
                assert_eq!(report.goal_heading_rad, Some(0.0));
            }
        }
        let locked = mission.active.unwrap();
        assert_eq!(
            world
                .tracks(Timestamp(200))
                .find(|track| track.id == locked)
                .unwrap()
                .color,
            ElementColor::Red
        );
        // Policy-level sequence of measured poses, not a simulated car trajectory.
        let radius = mission.orbit.unwrap().radius_m;
        for step in 0..=4 {
            let at = 300 + step * 100;
            let angle = -FRAC_PI_2 + step as f64 * FRAC_PI_4;
            let point = polar(Point2 { x_m: 2.0, y_m: 1.0 }, radius, angle);
            let estimate = pose(
                at,
                Pose2 {
                    x_m: point.x_m,
                    y_m: point.y_m,
                    yaw_rad: wrap(angle + FRAC_PI_2),
                },
            );
            observe(&mut world, &estimate, &cones, true);
            let report = mission.update(
                Timestamp(at),
                &estimate,
                &road(at, LightState::Unknown),
                &mut world,
            );
            assert_eq!(report.active_track_id, Some(locked));
            assert_eq!(report.processed_cones, 0);
        }
        // Reaching the circle's top is not yet leaving the exit centerline.
        // Continue along the measured tangent past the center uncertainty.
        let estimate = pose(
            800,
            Pose2 {
                x_m: 2.0 - cones[0].position_error_m - 2.0 * mission.config.goal_tolerance_m,
                y_m: 1.0 + radius,
                yaw_rad: PI,
            },
        );
        observe(&mut world, &estimate, &cones, true);
        let report = mission.update(
            Timestamp(800),
            &estimate,
            &road(800, LightState::Unknown),
            &mut world,
        );
        assert_eq!(report.processed_cones, 1);
        assert_eq!(report.mission.phase, MissionPhase::Cones);
        // Even a fresh tracker/new identity cannot turn the already processed
        // physical position into the next cone after the old track expired.
        let (_, mut replacement_world) = setup();
        for at in [800, 900] {
            let estimate = pose(
                at,
                Pose2 {
                    x_m: 4.0,
                    y_m: 1.85,
                    yaw_rad: PI,
                },
            );
            observe(&mut replacement_world, &estimate, &[cones[0]], true);
        }
        assert!(
            mission
                .select_track(
                    Timestamp(900),
                    Pose2 {
                        x_m: 4.0,
                        y_m: 1.85,
                        yaw_rad: PI
                    },
                    ElementKind::Cone,
                    &replacement_world
                )
                .is_none()
        );
    }

    #[test]
    fn second_cone_near_exit_requires_observed_departure_beyond_center_uncertainty() {
        // Actual early-transition offsets from the shifted/missing-marker
        // simulations. These policy fixtures start after the first task and
        // supply the entire measured second half-turn, not synthetic progress.
        for (center_x, before_center_m) in [(1.8, 0.00605), (1.5, 0.005085)] {
            let (mut mission, mut world) = setup();
            mission.phase = MissionPhase::Cones;
            mission.cone_index = 1;
            mission.initial_heading = Some(0.0);
            mission.config.required_cone_colors[1] = Some(ElementColor::Blue);
            let mut observed = cone(center_x, 2.5, ElementColor::Blue);
            observed.position_error_m = 0.02;
            for at in [100, 200] {
                let estimate = pose(
                    at,
                    Pose2 {
                        x_m: center_x + 2.0,
                        y_m: 2.5,
                        yaw_rad: PI,
                    },
                );
                observe(&mut world, &estimate, &[observed], true);
                let report = mission.update(
                    Timestamp(at),
                    &estimate,
                    &road(at, LightState::Unknown),
                    &mut world,
                );
                assert_eq!(report.processed_cones, 1);
            }
            let locked = mission.active.unwrap();
            let radius = mission.orbit.unwrap().radius_m;
            for step in 0..=3 {
                let at = 300 + step * 100;
                let angle = -FRAC_PI_2 - step as f64 * FRAC_PI_4;
                let point = polar(observed.position_body_m, radius, angle);
                let estimate = pose(
                    at,
                    Pose2 {
                        x_m: point.x_m,
                        y_m: point.y_m,
                        yaw_rad: wrap(angle - FRAC_PI_2),
                    },
                );
                observe(&mut world, &estimate, &[observed], true);
                let report = mission.update(
                    Timestamp(at),
                    &estimate,
                    &road(at, LightState::Unknown),
                    &mut world,
                );
                assert_eq!(report.active_track_id, Some(locked));
                assert_eq!(report.processed_cones, 1);
            }
            assert!(mission.orbit.unwrap().saw_outer_side);
            let original_exit = Point2 {
                x_m: center_x,
                y_m: 2.5 + radius,
            };
            for (at, signed_departure_m) in [
                (700, -before_center_m),
                (800, 0.0),
                (900, observed.position_error_m * 0.5),
            ] {
                let estimate = pose(
                    at,
                    Pose2 {
                        x_m: center_x + signed_departure_m,
                        y_m: original_exit.y_m,
                        yaw_rad: 0.0,
                    },
                );
                // The former position/heading gate would accept all three:
                // before the center, exactly on it, or within its error band.
                assert!(mission.near(estimate.pose, original_exit, 0.0));
                observe(&mut world, &estimate, &[observed], true);
                let report = mission.update(
                    Timestamp(at),
                    &estimate,
                    &road(at, LightState::Unknown),
                    &mut world,
                );
                assert_eq!(report.mission.phase, MissionPhase::Cones);
                assert_eq!(report.processed_cones, 1);
                assert_eq!(report.active_track_id, Some(locked));
                assert!(report.cone_progress_rad >= PI - mission.config.goal_heading_tolerance_rad);
                assert!(mission.orbit.unwrap().saw_outer_side);
                assert!(
                    !world
                        .tracks(Timestamp(at))
                        .find(|track| track.id == locked)
                        .unwrap()
                        .processed
                );
            }
            let at = 1000;
            let estimate = pose(
                at,
                Pose2 {
                    x_m: center_x
                        + observed.position_error_m
                        + 2.0 * mission.config.goal_tolerance_m,
                    y_m: original_exit.y_m,
                    yaw_rad: 0.0,
                },
            );
            observe(&mut world, &estimate, &[observed], true);
            let report = mission.update(
                Timestamp(at),
                &estimate,
                &road(at, LightState::Unknown),
                &mut world,
            );
            assert_eq!(report.processed_cones, 2);
            assert_eq!(report.mission.phase, MissionPhase::ApproachLight);
            assert!(
                world
                    .tracks(Timestamp(at))
                    .find(|track| track.id == locked)
                    .unwrap()
                    .processed
            );
        }
    }

    #[test]
    fn cone_entrance_uses_observed_tangent_guide_before_the_original_arrival_gate() {
        for axis in [0.0, 0.7] {
            for index in [0, 1] {
                let (mut mission, mut world) = setup();
                mission.phase = MissionPhase::Cones;
                mission.cone_index = index;
                mission.initial_heading = Some(axis);
                let heading = wrap(axis + if index == 0 { 0.0 } else { PI });
                let center = Point2 { x_m: 2.0, y_m: 2.5 };
                let observed = cone(center.x_m, center.y_m, ElementColor::Red);
                let entry = polar(
                    center,
                    mission.config.preferred_cone_radius_m,
                    axis - FRAC_PI_2,
                );
                let tangent = Pose2 {
                    x_m: entry.x_m,
                    y_m: entry.y_m,
                    yaw_rad: heading,
                };
                let start = tangent.body_to_world(Point2 {
                    x_m: -1.4,
                    y_m: -0.4,
                });
                let guide = polar(entry, mission.config.search_horizon_m, heading + PI);
                for at in [100, 200, 300, 350, 400, 500, 600] {
                    let (point, yaw) = match at {
                        100 | 200 => (start, wrap(heading + 0.3)),
                        // Position alone at the guide must not release the
                        // tangent gate, just as at the real entrance.
                        300 => (guide, wrap(heading + 0.3)),
                        350 | 400 => (guide, heading),
                        // Once acquired, a small later error must not make
                        // the vehicle return to a guide behind its body.
                        500 => (polar(entry, 0.4, heading + PI), wrap(heading + 0.15)),
                        _ => (entry, heading),
                    };
                    let mut estimate = pose(
                        at,
                        Pose2 {
                            x_m: point.x_m,
                            y_m: point.y_m,
                            yaw_rad: yaw,
                        },
                    );
                    if at == 350 {
                        // The guide is a Stop goal, with no dwell time added:
                        // correct pose while moving still cannot release it.
                        estimate.speed_mps = mission.config.approach_speed_mps;
                    }
                    observe(&mut world, &estimate, &[observed], true);
                    let report = mission.update(
                        Timestamp(at),
                        &estimate,
                        &road(at, LightState::Unknown),
                        &mut world,
                    );
                    if at == 100 {
                        continue;
                    }
                    let MissionOutput::Target {
                        point: target,
                        arrival,
                        ..
                    } = report.mission.output
                    else {
                        panic!("fully observed entrance corridor must produce a target");
                    };
                    if at <= 350 {
                        assert!(target.distance(guide) < 1e-12);
                        assert_eq!(report.goal_heading_rad, Some(heading));
                        assert_eq!(arrival, ArrivalBehavior::Stop);
                        assert!(!mission.orbit.unwrap().entry_aligned);
                    } else if at < 600 {
                        assert!(target.distance(entry) < 1e-12);
                        assert_eq!(report.goal_heading_rad, Some(heading));
                        assert!(mission.orbit.unwrap().entry_aligned);
                        assert!(!mission.orbit.unwrap().entered);
                    } else {
                        assert!(mission.orbit.unwrap().entered);
                        assert!(matches!(report.behavior, OnlineBehavior::OrbitCone));
                    }
                    assert_eq!(report.processed_cones, index);
                    assert_eq!(report.cone_progress_rad, 0.0);
                }
            }
        }
    }

    #[test]
    fn tangent_guide_cannot_authorize_motion_through_unknown_space() {
        let (mut mission, mut world) = setup();
        mission.phase = MissionPhase::Cones;
        for at in [100, 200] {
            let estimate = pose(at, Pose2::default());
            observe(
                &mut world,
                &estimate,
                &[cone(2.0, 1.0, ElementColor::Red)],
                false,
            );
            let report = mission.update(
                Timestamp(at),
                &estimate,
                &road(at, LightState::Unknown),
                &mut world,
            );
            assert_eq!(report.mission.output, MissionOutput::Stop);
            assert_eq!(report.cone_progress_rad, 0.0);
            if at == 200 {
                assert!(report.active_track_id.is_some());
                assert!(!mission.orbit.unwrap().entry_aligned);
            }
        }
    }

    #[test]
    fn second_cone_can_be_right_of_reverse_departure_without_changing_passing_side() {
        let (mut mission, mut world) = setup();
        mission.phase = MissionPhase::Cones;
        mission.cone_index = 1;
        mission.initial_heading = Some(0.0);
        mission.config.required_cone_colors[1] = Some(ElementColor::Blue);
        let start = Pose2 {
            x_m: 4.0,
            y_m: 3.35,
            yaw_rad: PI,
        };
        let actual = cone(1.5, 3.5, ElementColor::Blue);
        let observations = [
            actual,
            cone(3.5, 3.35, ElementColor::Red), // Closer, wrong configured color.
            cone(4.5, 3.35, ElementColor::Blue), // Behind the task direction.
            region(ElementKind::FinishMarker, 3.5, 3.35, 0.8),
        ];
        for at in [100, 200] {
            let estimate = pose(at, start);
            observe(&mut world, &estimate, &observations, true);
            mission.update(
                Timestamp(at),
                &estimate,
                &road(at, LightState::Unknown),
                &mut world,
            );
        }
        let selected = mission.active_geometry.unwrap();
        assert!(selected.position.distance(actual.position_body_m) < 1e-12);
        assert!(start.world_to_body(selected.position).y_m < 0.0);
        let radius = mission.orbit.unwrap().radius_m;
        let entry = polar(selected.position, radius, -FRAC_PI_2);
        let estimate = pose(
            300,
            Pose2 {
                x_m: entry.x_m,
                y_m: entry.y_m,
                yaw_rad: PI,
            },
        );
        observe(&mut world, &estimate, &observations, true);
        let report = mission.update(
            Timestamp(300),
            &estimate,
            &road(300, LightState::Unknown),
            &mut world,
        );
        let MissionOutput::Target { point, .. } = report.mission.output else {
            panic!("confirmed second cone must supply the clockwise arc");
        };
        assert!(point.x_m < selected.position.x_m);
        assert!(point.y_m < selected.position.y_m);
        assert!((wrap(report.goal_heading_rad.unwrap() - 3.0 * FRAC_PI_4)).abs() < 1e-12);
        assert_eq!(report.processed_cones, 1);
    }

    #[test]
    fn refreshed_processed_regions_are_not_selected_as_new_tasks() {
        for kind in [ElementKind::StopLine, ElementKind::FinishMarker] {
            let (mut mission, mut world) = setup();
            mission.phase = if kind == ElementKind::StopLine {
                MissionPhase::ApproachLight
            } else {
                MissionPhase::Finish
            };
            let observed = region(kind, 1.0, 0.0, 1.0);
            for at in [100, 200] {
                observe(&mut world, &pose(at, Pose2::default()), &[observed], true);
            }
            let id = world.tracks(Timestamp(200)).next().unwrap().id;
            world.mark_processed(id).unwrap();
            observe(&mut world, &pose(300, Pose2::default()), &[observed], true);
            let track = world
                .tracks(Timestamp(300))
                .find(|track| track.id == id)
                .unwrap();
            assert!(track.processed);
            assert!(
                mission
                    .select_track(Timestamp(300), Pose2::default(), kind, &world)
                    .is_none()
            );
            assert!(
                mission
                    .acquire(Timestamp(300), Pose2::default(), kind, &world)
                    .is_none()
            );
            assert_eq!(mission.active, None);
        }
    }

    #[test]
    fn current_track_loss_cannot_switch_to_another_visible_element() {
        let (mut mission, mut world) = setup();
        mission.phase = MissionPhase::Cones;
        let selected = cone(2.0, 1.0, ElementColor::Red);
        let initial = [selected];
        let mut locked = None;
        for at in (100..=4100).step_by(100) {
            let estimate = pose(at, Pose2::default());
            observe(
                &mut world,
                &estimate,
                if at <= 200 { &initial } else { &[] },
                true,
            );
            let report = mission.update(
                Timestamp(at),
                &estimate,
                &road(at, LightState::Unknown),
                &mut world,
            );
            if at == 200 {
                locked = report.active_track_id;
            }
            if at >= 2000 {
                assert_eq!(report.mission.output, MissionOutput::Stop);
                assert_eq!(report.active_track_id, locked);
            }
            if at >= 4000 {
                assert_eq!(report.mission.phase, MissionPhase::Fault);
                assert!(report.mission.reason.contains("lost"));
            }
        }
    }

    /// Raycast test geometry only. Production receives this same-time scan,
    /// never these walls or circles. Keep the normal pose/range uncertainty.
    fn swept_geometry_world(
        source: Pose2,
        extra_obstacle: Option<(Point2, f64)>,
        missing_world_bearing: Option<f64>,
    ) -> (OnlineMission, LocalWorld) {
        let config = OnlineMissionConfig::from_limits(&legacy());
        let mut world_config = LocalWorldConfig::simulation(
            config.body_frame.clone(),
            FrameId("sim_laser".into()),
            config.world_frame.clone(),
        );
        world_config.self_footprint = Some(config.footprint);
        let mut world = LocalWorld::new(world_config).unwrap();
        let increment = TAU / 360.0;
        let ranges_m = (0..360)
            .map(|i| {
                let angle = source.yaw_rad + i as f64 * increment;
                let (sin, cos) = angle.sin_cos();
                let mut nearest = 12.0f64;
                for hit in [
                    (0.0 - source.x_m) / cos,
                    (5.0 - source.x_m) / cos,
                    (0.0 - source.y_m) / sin,
                    (4.0 - source.y_m) / sin,
                ] {
                    if hit.is_finite() && hit > 0.0 {
                        nearest = nearest.min(hit);
                    }
                }
                for (center, radius) in [
                    Some((Point2 { x_m: 4.0, y_m: 2.0 }, 0.14 * 2.0f64.sqrt())),
                    extra_obstacle,
                ]
                .into_iter()
                .flatten()
                {
                    let x = center.x_m - source.x_m;
                    let y = center.y_m - source.y_m;
                    let along = x * cos + y * sin;
                    let across = x * sin - y * cos;
                    if along > 0.0 && across.abs() <= radius {
                        let hit = along - (radius * radius - across * across).sqrt();
                        if hit > 0.0 {
                            nearest = nearest.min(hit);
                        }
                    }
                }
                if missing_world_bearing
                    .is_some_and(|bearing| wrap(angle - bearing).abs() <= increment * 0.5)
                {
                    None
                } else {
                    Some(nearest)
                }
            })
            .collect();
        world
            .update_scan(
                Timestamp(100),
                &pose(100, source),
                &LidarSample {
                    captured_at: Timestamp(100),
                    frame_id: FrameId("sim_laser".into()),
                    angle_min_rad: 0.0,
                    angle_increment_rad: increment,
                    range_min_m: 0.02,
                    range_max_m: 12.0,
                    ranges_m,
                },
                Pose2::default(),
            )
            .unwrap();
        (OnlineMission::new(config).unwrap(), world)
    }

    fn swept_geometry_pose(radius: f64, angle: f64) -> Pose2 {
        let point = polar(Point2 { x_m: 4.0, y_m: 2.0 }, radius, angle);
        Pose2 {
            x_m: point.x_m,
            y_m: point.y_m,
            yaw_rad: wrap(angle + FRAC_PI_2),
        }
    }

    #[test]
    fn zero_sweep_certifies_the_whole_stationary_footprint_or_remains_unknown() {
        let source = Pose2 {
            x_m: 4.7,
            y_m: 1.1,
            yaw_rad: 0.0,
        };
        let (mission, world) = swept_geometry_world(source, None, None);
        let center = Point2 { x_m: 4.0, y_m: 2.0 };
        for sweep in [0.0, -0.0] {
            assert_eq!(
                mission.arc_space_state(Timestamp(100), &world, center, 0.6, -FRAC_PI_2, sweep),
                SpaceState::KnownFree
            );
        }
        // A zero-length reference is not permission to ignore its front/rear
        // corners. This stationary rectangle reaches beyond the observed wall.
        assert_eq!(
            mission.arc_space_state(
                Timestamp(100),
                &world,
                Point2 { x_m: 4.9, y_m: 2.0 },
                0.6,
                -FRAC_PI_2,
                0.0,
            ),
            SpaceState::Obstacle
        );
        let missing = (1.4_f64 - source.y_m).atan2(4.0 - source.x_m);
        let (_, unknown) = swept_geometry_world(source, None, Some(missing));
        assert_eq!(
            mission.arc_space_state(Timestamp(100), &unknown, center, 0.6, -FRAC_PI_2, 0.0),
            SpaceState::Unknown
        );
        assert_eq!(
            mission.arc_space_state(Timestamp(401), &world, center, 0.6, -FRAC_PI_2, 0.0),
            SpaceState::Unknown
        );
        for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert_eq!(
                mission.arc_space_state(Timestamp(100), &world, center, 0.6, -FRAC_PI_2, invalid),
                SpaceState::Unknown
            );
        }
    }

    #[test]
    fn zero_remaining_arc_keeps_observed_exit_speed_without_completing_early() {
        for cone_index in [0, 1] {
            let heading = if cone_index == 0 { PI } else { 0.0 };
            let source = Pose2 {
                x_m: if cone_index == 0 { 4.01 } else { 3.99 },
                y_m: 2.6,
                yaw_rad: heading,
            };
            let (mut mission, mut world) = swept_geometry_world(source, None, None);
            let mut marker = cone(4.0, 2.0, ElementColor::Red);
            marker.geometry = ElementGeometry::Cone {
                radius_m: 0.14 * 2.0_f64.sqrt(),
            };
            marker.position_body_m = source.world_to_body(marker.position_body_m);
            for at in [0, 100] {
                world
                    .update(
                        Timestamp(at),
                        &pose(at, source),
                        &ElementFrame {
                            captured_at: Timestamp(at),
                            frame_id: FrameId("sim_body".into()),
                            observations: vec![marker],
                        },
                    )
                    .unwrap();
            }
            let track = world.tracks(Timestamp(100)).next().unwrap();
            mission.active = Some(track.id);
            mission.active_geometry = Some(track);
            mission.initial_heading = Some(0.0);
            mission.phase = MissionPhase::Cones;
            mission.cone_index = cone_index;
            mission.orbit = Some(Orbit {
                radius_m: 0.6,
                entry_aligned: true,
                entered: true,
                progress_rad: PI,
                saw_outer_side: true,
                last_position: source.point(),
            });
            let proposal = mission
                .cone_step(Timestamp(100), &pose(100, source), false, &mut world)
                .unwrap();
            let MissionOutput::Target { max_speed_mps, .. } = proposal.output else {
                panic!("observed exit should retain a target");
            };
            assert_eq!(max_speed_mps, mission.config.cruise_speed_mps);
            assert!(matches!(proposal.behavior, OnlineBehavior::OrbitCone));
            assert_eq!(mission.cone_index, cone_index);
            assert!(!world.tracks(Timestamp(100)).next().unwrap().processed);
        }
    }

    #[test]
    fn swept_geometry_small_observed_quarter_arc_accepts_tangent_rectangle_not_large_radius() {
        // A side-front observation sees the complete quarter sweep; the
        // distant entrance-view occlusion has a separate negative test below.
        let source = Pose2 {
            x_m: 4.7,
            y_m: 1.1,
            yaw_rad: 0.0,
        };
        let (mission, world) = swept_geometry_world(source, None, None);
        assert_eq!(mission.config.footprint.front_m, 0.22);
        assert_eq!(mission.config.footprint.rear_m, 0.18);
        assert_eq!(mission.config.footprint.half_width_m, 0.13);
        assert_eq!(mission.config.clearance_m, 0.04);
        // Preserve the .179005 m track error from the actual blocked small
        // snapshot. This reference remains above both physical minimum radii.
        assert!(0.6 >= 0.14 * 2.0f64.sqrt() + 0.13 + 0.04 + 0.17900496686854434);
        assert!(0.6 >= 1.0 / mission.config.max_curvature_per_m);
        let center = Point2 { x_m: 4.0, y_m: 2.0 };
        assert!(mission.arc_is_known_free(
            Timestamp(100),
            &world,
            center,
            0.6,
            -FRAC_PI_2,
            FRAC_PI_2
        ));
        // At the outer side, .85 + half-width + clearance exceeds the actual
        // 1 m center-to-wall gap. The complete sweep must be rejected.
        assert!(!mission.arc_is_known_free(
            Timestamp(100),
            &world,
            center,
            0.85,
            -FRAC_PI_2,
            FRAC_PI_2
        ));
    }

    #[test]
    fn swept_geometry_mid_arc_obstacle_rejects_despite_clear_endpoint_bodies() {
        // A side-front observation sees the complete quarter sweep; the
        // distant entrance-view occlusion has a separate negative test below.
        let source = Pose2 {
            x_m: 4.7,
            y_m: 1.1,
            yaw_rad: 0.0,
        };
        let middle = swept_geometry_pose(0.7, -FRAC_PI_4).point();
        let (mission, world) = swept_geometry_world(source, Some((middle, 0.025)), None);
        for angle in [-FRAC_PI_2, 0.0] {
            assert!(
                world.known_free_convex_hull(
                    Timestamp(100),
                    &mission
                        .config
                        .footprint
                        .corners(swept_geometry_pose(0.7, angle)),
                    mission.config.clearance_m,
                )
            );
        }
        assert!(!mission.arc_is_known_free(
            Timestamp(100),
            &world,
            Point2 { x_m: 4.0, y_m: 2.0 },
            0.7,
            -FRAC_PI_2,
            FRAC_PI_2
        ));
    }

    #[test]
    fn swept_geometry_missing_ray_cannot_certify_an_arc() {
        // A side-front observation sees the complete quarter sweep; the
        // distant entrance-view occlusion has a separate negative test below.
        let source = Pose2 {
            x_m: 4.7,
            y_m: 1.1,
            yaw_rad: 0.0,
        };
        let middle = swept_geometry_pose(0.7, -FRAC_PI_4).point();
        let bearing = (middle.y_m - source.y_m).atan2(middle.x_m - source.x_m);
        let (mission, world) = swept_geometry_world(source, None, Some(bearing));
        assert!(!mission.arc_is_known_free(
            Timestamp(100),
            &world,
            Point2 { x_m: 4.0, y_m: 2.0 },
            0.7,
            -FRAC_PI_2,
            FRAC_PI_2
        ));
    }

    #[test]
    fn swept_geometry_far_cone_occlusion_is_unknown_even_with_a_free_tangent_approach() {
        let source = Pose2 {
            x_m: 2.8,
            y_m: 1.3,
            yaw_rad: 0.0,
        };
        let (mission, world) = swept_geometry_world(source, None, None);
        let entry = swept_geometry_pose(0.7, -FRAC_PI_2).point();
        assert!(world.known_free_segment(
            Timestamp(100),
            source.point(),
            entry,
            mission.body_padding()
        ));
        // The cone occludes later portions from this distant source. This must
        // remain unknown; a separate approach policy may move closer and rescan.
        assert!(!mission.arc_is_known_free(
            Timestamp(100),
            &world,
            Point2 { x_m: 4.0, y_m: 2.0 },
            0.7,
            -FRAC_PI_2,
            FRAC_PI_2
        ));
    }

    #[test]
    fn swept_geometry_oriented_target_checks_interior_not_only_clear_corners() {
        let source = Pose2 {
            x_m: 4.2,
            y_m: 1.5,
            yaw_rad: 0.0,
        };
        let obstacle = Point2 {
            x_m: 4.59,
            y_m: 2.11,
        };
        let (mission, world) = swept_geometry_world(source, Some((obstacle, 0.006)), None);
        let target = Pose2 {
            x_m: 4.65,
            y_m: 2.0,
            yaw_rad: FRAC_PI_2,
        };
        let corners = mission.config.footprint.corners(target);
        assert!(
            corners
                .iter()
                .all(|point| world.space_state(Timestamp(100), *point, 0.0)
                    == crate::local_world::SpaceState::KnownFree)
        );
        let proposal = mission.local_target(
            Timestamp(100),
            source,
            target.point(),
            Some(target.yaw_rad),
            None,
            true,
            OnlineBehavior::ApproachRegion,
            "test interior target",
            &world,
        );
        assert_eq!(proposal.output, MissionOutput::Stop);
        assert_eq!(
            proposal.reason,
            "short target body envelope is unknown or blocked"
        );
    }

    #[test]
    fn swept_geometry_future_unknown_allows_observed_approach_but_unknown_target_stops() {
        let source = Pose2 {
            x_m: 2.8,
            y_m: 1.3,
            yaw_rad: 0.0,
        };
        let (mut mission, mut world) = swept_geometry_world(source, None, None);
        mission.phase = MissionPhase::Cones;
        mission.initial_heading = Some(0.0);
        for at in [0, 100] {
            let observed = ElementObservation {
                position_body_m: source.world_to_body(Point2 { x_m: 4.0, y_m: 2.0 }),
                position_error_m: 0.06,
                ..cone(0.0, 0.0, ElementColor::Red)
            };
            world
                .update(
                    Timestamp(at),
                    &pose(at, source),
                    &ElementFrame {
                        captured_at: Timestamp(at),
                        frame_id: FrameId("sim_body".into()),
                        observations: vec![observed],
                    },
                )
                .unwrap();
        }
        assert_eq!(
            mission.arc_space_state(
                Timestamp(100),
                &world,
                Point2 { x_m: 4.0, y_m: 2.0 },
                0.6,
                -FRAC_PI_2,
                FRAC_PI_2
            ),
            crate::local_world::SpaceState::Unknown
        );
        let report = mission.update(
            Timestamp(100),
            &pose(100, source),
            &road(100, LightState::Unknown),
            &mut world,
        );
        let MissionOutput::Target { point, .. } = report.mission.output else {
            panic!("a known-free tangent approach should remain available: {report:?}");
        };
        assert!(point.x_m > source.x_m && point.x_m < 4.0);
        assert_eq!(report.cone_progress_rad, 0.0);
        assert_eq!(report.processed_cones, 0);
        assert!(!mission.orbit.unwrap().entered);
        let (mission, unknown) = swept_geometry_world(source, None, Some(0.0));
        let proposal = mission.local_target(
            Timestamp(100),
            source,
            Point2 { x_m: 3.2, y_m: 1.3 },
            Some(0.0),
            None,
            true,
            OnlineBehavior::ConeEntry,
            "approach test",
            &unknown,
        );
        assert_eq!(proposal.output, MissionOutput::Stop);
    }

    #[test]
    fn swept_geometry_orbit_speed_uses_future_visibility_without_admitting_unknown_target() {
        let source = Pose2 {
            x_m: 4.7,
            y_m: 1.1,
            yaw_rad: 0.0,
        };
        for (missing_bearing, expected_state, expected_speed) in [
            (None, SpaceState::KnownFree, Some(0.3)),
            (Some(2.9), SpaceState::Unknown, Some(0.18)),
            (Some(2.1), SpaceState::Unknown, None),
        ] {
            let (mut mission, mut world) = swept_geometry_world(source, None, missing_bearing);
            // A policy-level committed orbit fixture; no fabricated movement
            // or angular progress is used to establish completion.
            let mut marker = cone(4.0, 2.0, ElementColor::Red);
            marker.geometry = ElementGeometry::Cone {
                radius_m: 0.14 * 2.0f64.sqrt(),
            };
            marker.position_error_m = 0.06;
            marker.position_body_m = source.world_to_body(marker.position_body_m);
            for at in [0, 100] {
                world
                    .update(
                        Timestamp(at),
                        &pose(at, source),
                        &ElementFrame {
                            captured_at: Timestamp(at),
                            frame_id: FrameId("sim_body".into()),
                            observations: vec![marker],
                        },
                    )
                    .unwrap();
            }
            let track = world
                .tracks(Timestamp(100))
                .find(|track| track.kind == ElementKind::Cone)
                .unwrap();
            mission.active = Some(track.id);
            mission.active_geometry = Some(track);
            mission.initial_heading = Some(0.0);
            mission.phase = MissionPhase::Cones;
            mission.orbit = Some(Orbit {
                radius_m: 0.6,
                entry_aligned: true,
                entered: true,
                progress_rad: 0.0,
                saw_outer_side: false,
                last_position: source.point(),
            });
            assert_eq!(
                mission.arc_space_state(
                    Timestamp(100),
                    &world,
                    track.position,
                    0.6,
                    -FRAC_PI_2,
                    mission.config.arc_lookahead_rad
                ),
                expected_state
            );
            let proposal = mission
                .cone_step(Timestamp(100), &pose(100, source), false, &mut world)
                .unwrap();
            if let Some(expected_speed) = expected_speed {
                let MissionOutput::Target {
                    point,
                    max_speed_mps,
                    ..
                } = proposal.output
                else {
                    panic!("current target is observed free; future state was {expected_state:?}");
                };
                assert_eq!(max_speed_mps, expected_speed);
                assert!(world.known_free_convex_hull(
                    Timestamp(100),
                    &mission.config.footprint.corners(Pose2 {
                        x_m: point.x_m,
                        y_m: point.y_m,
                        yaw_rad: proposal.heading.unwrap()
                    }),
                    mission.config.clearance_m
                ));
            } else {
                assert_eq!(proposal.output, MissionOutput::Stop);
            }
            assert_eq!(mission.orbit.unwrap().progress_rad, 0.0);
            assert_eq!(mission.cone_index, 0);
        }
    }

    #[test]
    fn swept_geometry_oriented_target_horizon_is_bounded_and_does_not_admit_unknown() {
        let source = Pose2 {
            x_m: 2.0,
            y_m: 1.0,
            yaw_rad: 0.0,
        };
        let (mission, world) = swept_geometry_world(source, None, None);
        let horizon = mission.config.search_horizon_m;
        assert_eq!(horizon, 0.8);
        for distance in [horizon, horizon * 1.5, horizon * 2.0] {
            let goal = Point2 {
                x_m: source.x_m + distance,
                y_m: source.y_m,
            };
            let proposal = mission.local_target(
                Timestamp(100),
                source,
                goal,
                Some(0.0),
                None,
                true,
                OnlineBehavior::ConeEntry,
                "bounded alignment",
                &world,
            );
            let MissionOutput::Target { point, .. } = proposal.output else {
                panic!("known-free oriented goal at {distance} m was rejected");
            };
            assert_eq!(point, goal);
            assert_eq!(proposal.heading, Some(0.0));
            assert!(world.known_free_convex_hull(
                Timestamp(100),
                &mission.config.footprint.corners(Pose2 {
                    x_m: goal.x_m,
                    y_m: goal.y_m,
                    yaw_rad: 0.0
                }),
                mission.config.clearance_m
            ));
        }
        // Beyond two horizons, even an oriented destination becomes a short
        // rolling point. Unoriented search keeps this original one-horizon cap.
        for (distance, heading) in [(horizon * 2.0 + 0.001, Some(0.0)), (horizon * 2.0, None)] {
            let goal = Point2 {
                x_m: source.x_m + distance,
                y_m: source.y_m,
            };
            let proposal = mission.local_target(
                Timestamp(100),
                source,
                goal,
                heading,
                None,
                true,
                OnlineBehavior::ConeEntry,
                "bounded alignment",
                &world,
            );
            let MissionOutput::Target { point, .. } = proposal.output else {
                panic!("known-free rolling approach was rejected");
            };
            assert!(source.point().distance(point) <= horizon);
            assert_ne!(point, goal);
            assert_eq!(proposal.heading, None);
        }
        let (mission, unknown) = swept_geometry_world(source, None, Some(0.0));
        let proposal = mission.local_target(
            Timestamp(100),
            source,
            Point2 {
                x_m: source.x_m + horizon * 2.0,
                y_m: source.y_m,
            },
            Some(0.0),
            None,
            true,
            OnlineBehavior::ConeEntry,
            "bounded alignment",
            &unknown,
        );
        assert_eq!(proposal.output, MissionOutput::Stop);
    }
}
