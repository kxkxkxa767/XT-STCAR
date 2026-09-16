//! Sensor-only local mission adapter. Scene truth never enters this module.
use crate::autonomy::{AutonomyConfig, Result, RoadFrame};
use crate::control_runtime::PlanningContext;
use serde::{Deserialize, Serialize};
use xt_stcar_robot_core::admission::StoppingEnvelope;
use xt_stcar_robot_core::autonomy::{Pose2, PoseEstimate};
use xt_stcar_robot_core::local_world::{
    ConeAssociationConfig, ElementFrame, ElementKind, ElementTrack, LocalWorld, LocalWorldConfig,
    MAX_ELEMENTS, TrackId,
};
use xt_stcar_robot_core::mission::{MissionOutput, MissionPhase};
use xt_stcar_robot_core::navigation::{ArrivalBehavior, NavigationConfig, SteeringEstimate};
use xt_stcar_robot_core::online_mission::{
    OnlineBehavior, OnlineMission, OnlineMissionConfig, OnlineMissionReport,
    observed_region_boundary,
};
use xt_stcar_robot_core::{LidarSample, MotionOutput, Timestamp};

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct OnlineControlConfig {
    pub mission: OnlineMissionConfig,
    pub world: LocalWorldConfig,
    pub cone_association: ConeAssociationConfig,
}

impl OnlineControlConfig {
    /// Copies only vehicle limits and frame identities, never legacy task geometry.
    pub fn from_limits(config: &AutonomyConfig) -> Self {
        let mut world = LocalWorldConfig::simulation(
            config.mission.body_frame.clone(),
            config.scan.frame_id.clone(),
            config.navigation.frame_id.clone(),
        );
        world.self_footprint = Some(config.navigation.footprint);
        let mut mission = OnlineMissionConfig::from_limits(&config.mission);
        mission.clearance_m = config.navigation.clearance_m;
        mission.max_curvature_per_m = config.navigation.max_curvature_per_m;
        Self {
            mission,
            world,
            cone_association: ConeAssociationConfig::simulation(
                config.mission.body_frame.clone(),
                config.scan.frame_id.clone(),
                config.lidar_in_body,
            ),
        }
    }
}

pub(crate) struct OnlineSession {
    mission: OnlineMission,
    world: LocalWorld,
    cone_association: ConeAssociationConfig,
    last_elements: Option<ElementFrame>,
    /// A first credible stop-region observation may restrict motion, but cannot
    /// create a task identity, confirmation or green admission. Fixed capacity.
    pending_stop_lines: [Option<ElementTrack>; MAX_ELEMENTS],
    confirmed_stop_line: Option<(ElementTrack, Timestamp)>,
    stop_lines_released: bool,
    orbit_speed_hint: OrbitSpeedHint,
}

impl OnlineSession {
    pub(crate) fn new(config: OnlineControlConfig, autonomy: &AutonomyConfig) -> Result<Self> {
        let expected = OnlineControlConfig::from_limits(autonomy).mission;
        let limits = |mission: &OnlineMissionConfig| -> Result<serde_json::Value> {
            let mut value = serde_json::to_value(mission).map_err(|e| e.to_string())?;
            for field in [
                "search_horizon_m",
                "max_search_distance_m",
                "max_search_ms",
                "max_track_loss_ms",
                "preferred_cone_radius_m",
                "arc_lookahead_rad",
                "processed_exclusion_m",
                "required_cone_colors",
            ] {
                value.as_object_mut().expect("mission object").remove(field);
            }
            Ok(value)
        };
        if limits(&config.mission)? != limits(&expected)? {
            return Err(
                "online mission limits must match the existing vehicle/control limits".into(),
            );
        }
        if config.world.body_frame != autonomy.mission.body_frame
            || config.world.world_frame != autonomy.navigation.frame_id
            || config.world.lidar_frame != autonomy.scan.frame_id
            || config.world.scan_ttl_ms < autonomy.max_sensor_age_ms
            || config.world.self_footprint != Some(autonomy.navigation.footprint)
            || config.cone_association.body_frame != config.world.body_frame
            || config.cone_association.lidar_frame != config.world.lidar_frame
            || config.cone_association.lidar_in_body != autonomy.lidar_in_body
        {
            return Err("online frame identities or scan lease disagree with controller".into());
        }
        config
            .cone_association
            .validate()
            .map_err(|e| e.to_string())?;
        Ok(Self {
            mission: OnlineMission::new(config.mission).map_err(|e| e.to_string())?,
            world: LocalWorld::new(config.world).map_err(|e| e.to_string())?,
            cone_association: config.cone_association,
            last_elements: None,
            pending_stop_lines: [None; MAX_ELEMENTS],
            confirmed_stop_line: None,
            stop_lines_released: false,
            orbit_speed_hint: OrbitSpeedHint::new(autonomy),
        })
    }

    pub(crate) fn start(&mut self) -> Result<()> {
        self.mission.start().map_err(|e| e.to_string())
    }

    pub(crate) fn update(
        &mut self,
        at: Timestamp,
        pose: &PoseEstimate,
        scan: &LidarSample,
        road: &RoadFrame,
        lidar_in_body: Pose2,
    ) -> Result<OnlineMissionReport> {
        let elements = road
            .elements
            .as_ref()
            .ok_or("online control requires an explicit element frame")?;
        if elements.captured_at != road.observation.captured_at
            || elements.captured_at != pose.captured_at
            || lidar_in_body != self.cone_association.lidar_in_body
        {
            return Err(
                "online elements, road, range and pose require the same capture time and calibrated extrinsic".into(),
            );
        }
        if elements.observations.len() > MAX_ELEMENTS
            || elements.observations.capacity() > MAX_ELEMENTS
            || self.last_elements.as_ref().is_some_and(|old| {
                elements.captured_at < old.captured_at
                    || (elements.captured_at == old.captured_at && elements != old)
            })
        {
            return Err("online raw elements changed at one timestamp or exceeded capacity".into());
        }
        let associated = xt_stcar_robot_core::local_world::associate_visual_cones(
            elements,
            scan,
            &self.cone_association,
        )
        .map_err(|e| e.to_string())?;
        self.world
            .update_scan(at, pose, scan, lidar_in_body)
            .map_err(|e| e.to_string())?;
        self.world
            .update(at, pose, &associated)
            .map_err(|e| e.to_string())?;
        self.world
            .maintain_confirmed_cones(at, &self.cone_association)
            .map_err(|e| e.to_string())?;
        self.last_elements = Some(elements.clone());
        let mut report = self
            .mission
            .update(at, pose, &road.observation, &mut self.world);
        self.retain_task_regions(at, &report)?;
        self.stop_lines_released = matches!(
            report.mission.phase,
            MissionPhase::Finish | MissionPhase::Completed
        );
        if let Some(light) = self.mission.last_observed_light_region()
            && let Some(fresh) = self.world.tracks(at).find(|track| track.id == light.id)
        {
            // The snapshot's errors already include age through `at`.
            self.confirmed_stop_line = Some((fresh, at));
        }
        self.update_pending_stop_lines(at, pose, &associated, &report)?;
        self.orbit_speed_hint
            .apply(at, pose, &self.world, &mut report);
        Ok(report)
    }

    fn retain_task_regions(&mut self, at: Timestamp, report: &OnlineMissionReport) -> Result<()> {
        let mut identities = [TrackId(0); 2];
        let mut len = 0;
        if !matches!(
            report.mission.phase,
            MissionPhase::Fault | MissionPhase::Completed
        ) {
            for track in [
                self.mission.last_observed_light_region(),
                report.active_track,
            ]
            .into_iter()
            .flatten()
            {
                if matches!(
                    track.kind,
                    ElementKind::StopLine | ElementKind::FinishMarker
                ) && !identities[..len].contains(&track.id)
                {
                    identities[len] = track.id;
                    len += 1;
                }
            }
        }
        // A previous tick's explicit references already protect allocator slots
        // during update. This post-update set handles newly selected regions and
        // phase changes, without inventing or transferring an erased identity.
        self.world
            .set_region_pins(at, &identities[..len])
            .map_err(|e| e.to_string())
    }

    fn update_pending_stop_lines(
        &mut self,
        at: Timestamp,
        pose: &PoseEstimate,
        frame: &ElementFrame,
        report: &OnlineMissionReport,
    ) -> Result<()> {
        // Only the unchanged, confirmed stationary-green task transition grants
        // this permission. A raw green pixel/first line frame cannot clear it.
        if matches!(
            report.mission.phase,
            MissionPhase::Finish | MissionPhase::Completed
        ) {
            self.pending_stop_lines.fill(None);
            return Ok(());
        }
        let config = self.world.config();
        let mut touched = [false; MAX_ELEMENTS];
        for candidate in source_stop_regions(pose, frame, config)?
            .into_iter()
            .flatten()
        {
            let slot = self
                .pending_stop_lines
                .iter()
                .position(|slot| slot.is_some_and(|old| same_stop_region(old, candidate, config)));
            let slot = slot
                .or_else(|| self.pending_stop_lines.iter().position(Option::is_none))
                .ok_or("unconfirmed stop-region capacity exhausted")?;
            if touched[slot] {
                return Err("ambiguous unconfirmed stop-region observations".into());
            }
            touched[slot] = true;
            self.pending_stop_lines[slot] = Some(candidate);
        }
        // Hand over only when the task actually publishes the same confirmed
        // boundary. An unrelated confirmed landmark must not drop this gate.
        for slot in &mut self.pending_stop_lines {
            if let Some(pending) = *slot
                && self.world.tracks(at).any(|track| {
                    track.kind == ElementKind::StopLine
                        && same_stop_region(pending, track, config)
                        && observed_region_boundary(track, true).ok() == report.travel_boundary
                })
            {
                *slot = None;
            }
        }
        Ok(())
    }

    /// A current-source planning hint only. The hypothetical lower target does
    /// not replace measured, projected or historical speed in permits_command.
    pub(crate) fn apply_source_stop_speed_hint(
        &self,
        at: Timestamp,
        source: &PoseEstimate,
        projection: Option<&PlanningContext>,
        dt_s: f64,
        report: &mut OnlineMissionReport,
    ) {
        if !matches!(report.behavior, OnlineBehavior::OrbitCone)
            || report.mission.phase != MissionPhase::Cones
            || self.stop_lines_released
            || (self.pending_stop_lines.iter().all(Option::is_none)
                && self.confirmed_stop_line.is_none())
        {
            return;
        }
        let nav = &self.orbit_speed_hint.navigation;
        if !dt_s.is_finite() || dt_s <= 0.0 || dt_s > nav.control_period_ms as f64 / 1000.0 {
            return;
        }
        let measured = projection.map_or(source.speed_mps, |p| p.projected_pose.speed_mps);
        if !measured.is_finite() || measured < 0.0 || measured > nav.max_speed_mps {
            return;
        }
        let mut executable_lower = (measured - nav.max_decel_mps2 * dt_s).max(0.0);
        if let Some(context) = projection {
            if context.planned_at != at || context.source_at != source.captured_at {
                return;
            }
            if let Some(constraints) = context.adoption_constraints {
                if constraints.planned_at != at || constraints.validate(nav).is_err() {
                    return;
                }
                executable_lower = executable_lower.max(constraints.speed_interval(nav).0);
            }
        }
        let MissionOutput::Target {
            max_speed_mps,
            arrival,
            ..
        } = &mut report.mission.output
        else {
            return;
        };
        let upper = *max_speed_mps;
        if !upper.is_finite() || upper <= 0.0 || upper > nav.max_speed_mps {
            return;
        }
        let permits = |speed| {
            StoppingEnvelope::new(
                nav,
                source.pose,
                speed,
                nav.max_curvature_per_m,
                self.orbit_speed_hint.horizon_s,
            )
            .is_some_and(|envelope| {
                let corners = envelope.corners().map(|p| source.pose.body_to_world(p));
                self.world.known_free_convex_hull(at, &corners, 0.0)
                    && self.stop_regions_permit(at, &corners)
            })
        };
        if permits(upper) || !permits(0.0) {
            return;
        }
        let (mut lower, mut higher) = (0.0, upper);
        for _ in 0..8 {
            let middle = (lower + higher) / 2.0;
            if permits(middle) {
                lower = middle;
            } else {
                higher = middle;
            }
        }
        // Never raise an unproved cap to the actuator lower bound. If no
        // positive, currently executable witness exists, the hint is absent;
        // the independent actual-source certificate still rejects unsafe Drive.
        if lower > 0.0 && lower >= executable_lower {
            *max_speed_mps = lower;
            if let ArrivalBehavior::PassThrough {
                next_max_speed_mps, ..
            } = arrival
            {
                *next_max_speed_mps = next_max_speed_mps.min(lower);
            }
        }
    }

    fn stop_regions_permit(
        &self,
        at: Timestamp,
        corners: &[xt_stcar_robot_core::autonomy::Point2],
    ) -> bool {
        self.pending_stop_lines.iter().flatten().all(|track| {
            stop_region_permits(at, *track, track.last_seen, self.world.config(), corners)
        }) && (self.stop_lines_released
            || self.confirmed_stop_line.is_none_or(|(track, checked_at)| {
                stop_region_permits(at, track, checked_at, self.world.config(), corners)
            }))
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn permits_command(
        &self,
        at: Timestamp,
        source: &PoseEstimate,
        projected: &PoseEstimate,
        command: &MotionOutput,
        config: &AutonomyConfig,
        projection: Option<&PlanningContext>,
        steering: SteeringEstimate,
    ) -> bool {
        let MotionOutput::Drive {
            speed_mps,
            curvature_per_m,
        } = *command
        else {
            return true;
        };
        let mut speed = source
            .speed_mps
            .abs()
            .max(projected.speed_mps.abs())
            .max(speed_mps);
        let mut curvature = curvature_per_m
            .abs()
            .max(steering.applied_curvature_per_m.abs())
            .max(steering.commanded_curvature_per_m.abs());
        if let Some(context) = projection {
            speed = speed
                .max(context.historical_speed_bound_mps)
                .max(context.held_speed_mps);
            curvature = curvature.max(context.historical_curvature_bound_per_m);
        }
        let horizon =
            (config.max_sensor_age_ms + config.navigation.control_period_ms) as f64 / 1000.0;
        StoppingEnvelope::new(&config.navigation, source.pose, speed, curvature, horizon)
            .is_some_and(|envelope| {
                let corners = envelope
                    .corners()
                    .map(|point| source.pose.body_to_world(point));
                self.world.known_free_convex_hull(at, &corners, 0.0)
                    && self.stop_regions_permit(at, &corners)
            })
    }
}

/// Bounded speed planning only. Six sampled reference poses do NOT certify the
/// continuous motion between them or grant Drive permission. The actual source,
/// history, lease, steering and full stopping-envelope gates remain unchanged.
struct OrbitSpeedHint {
    navigation: NavigationConfig,
    horizon_s: f64,
    heading_tolerance_rad: f64,
}
impl OrbitSpeedHint {
    fn new(config: &AutonomyConfig) -> Self {
        Self {
            navigation: config.navigation.clone(),
            horizon_s: (config.max_sensor_age_ms + config.navigation.control_period_ms) as f64
                / 1000.0,
            heading_tolerance_rad: config.mission.goal_heading_tolerance_rad,
        }
    }

    fn apply(
        &self,
        at: Timestamp,
        source: &PoseEstimate,
        world: &LocalWorld,
        report: &mut OnlineMissionReport,
    ) {
        let Some(probes) = self.probes(at, source.pose, world, report) else {
            return;
        };
        let MissionOutput::Target {
            max_speed_mps,
            arrival,
            ..
        } = &mut report.mission.output
        else {
            return;
        };
        let Some(cap) = self.speed_cap(at, world, &probes, *max_speed_mps) else {
            return;
        };
        *max_speed_mps = cap;
        if let ArrivalBehavior::PassThrough {
            next_max_speed_mps, ..
        } = arrival
        {
            *next_max_speed_mps = next_max_speed_mps.min(cap);
        }
        // A measured speed above this positive target is never replaced by the
        // hint in any execution certificate. Navigation applies its original
        // deceleration interval; if the cap cannot be reached this tick, its
        // ordinary Stop path still brakes the physical plant.
    }

    fn probes(
        &self,
        at: Timestamp,
        source: Pose2,
        world: &LocalWorld,
        report: &OnlineMissionReport,
    ) -> Option<[Pose2; 6]> {
        use std::f64::consts::{FRAC_PI_2, PI, TAU};
        if !matches!(report.behavior, OnlineBehavior::OrbitCone)
            || report.mission.phase != MissionPhase::Cones
            || report.processed_cones > 1
            || !source.valid()
        {
            return None;
        }
        let MissionOutput::Target { point, .. } = report.mission.output else {
            return None;
        };
        let heading = report.goal_heading_rad.filter(|h| h.is_finite())?;
        let id = report.active_track_id?;
        let track = world
            .tracks(at)
            .find(|track| track.id == id && track.kind == ElementKind::Cone && !track.processed)?;
        let radius = point.distance(track.position);
        if !radius.is_finite() || radius <= 0.0 {
            return None;
        }
        let wrap = |angle: f64| (angle + PI).rem_euclid(TAU) - PI;
        let direction = if report.processed_cones == 0 {
            1.0
        } else {
            -1.0
        };
        let start = (source.y_m - track.position.y_m).atan2(source.x_m - track.position.x_m);
        let end = (point.y_m - track.position.y_m).atan2(point.x_m - track.position.x_m);
        let progress = direction * wrap(end - start);
        // Exit-clearance targets and rolling point-only approaches need not be
        // circular. Do not invent an arc for an incompatible target/heading.
        if !(0.0..=FRAC_PI_2).contains(&progress)
            || wrap(heading - end - direction * FRAC_PI_2).abs() > self.heading_tolerance_rad
        {
            return None;
        }
        let mut probes = [source; 6];
        for (index, probe) in probes[1..].iter_mut().enumerate() {
            let angle = start + direction * progress * index as f64 / 4.0;
            *probe = Pose2 {
                x_m: track.position.x_m + radius * angle.cos(),
                y_m: track.position.y_m + radius * angle.sin(),
                yaw_rad: wrap(angle + direction * FRAC_PI_2),
            };
        }
        Some(probes)
    }

    fn speed_cap(
        &self,
        at: Timestamp,
        world: &LocalWorld,
        probes: &[Pose2; 6],
        upper: f64,
    ) -> Option<f64> {
        if !upper.is_finite() || upper <= 0.0 || upper > self.navigation.max_speed_mps {
            return None;
        }
        let observed = |speed| {
            probes.iter().all(|pose| {
                StoppingEnvelope::new(
                    &self.navigation,
                    *pose,
                    speed,
                    self.navigation.max_curvature_per_m,
                    self.horizon_s,
                )
                .is_some_and(|envelope| {
                    let corners = envelope.corners().map(|point| pose.body_to_world(point));
                    world.known_free_convex_hull(at, &corners, 0.0)
                })
            })
        };
        if observed(upper) {
            return Some(upper);
        }
        // Unknown future space is still a planning hypothesis. If even a
        // stationary footprint cannot be proved, this hint has no information;
        // it neither publishes a zero-speed Target nor clears any actual gate.
        if !observed(0.0) {
            return None;
        }
        let (mut lower, mut higher) = (0.0, upper);
        for _ in 0..8 {
            let middle = (lower + higher) / 2.0;
            if observed(middle) {
                lower = middle;
            } else {
                higher = middle;
            }
        }
        // The lower value is always a positively observed witness. A false
        // proof can be conservative/non-monotone; it can only lower efficiency,
        // never turn an unobserved candidate into a positive speed certificate.
        (lower > 0.0).then_some(lower)
    }
}

/// Inputs must already have passed same-time association and LocalWorld::update.
/// This returns restriction evidence only: no confirmed identity is created.
pub(crate) fn source_stop_regions(
    pose: &PoseEstimate,
    frame: &ElementFrame,
    config: &LocalWorldConfig,
) -> Result<[Option<ElementTrack>; MAX_ELEMENTS]> {
    if frame.observations.len() > MAX_ELEMENTS {
        return Err("stop-region frame capacity exceeded".into());
    }
    let mut output = [None; MAX_ELEMENTS];
    for (index, o) in frame.observations.iter().enumerate() {
        if o.kind != ElementKind::StopLine || o.confidence < config.min_confidence {
            continue;
        }
        // The complete frame, source, exact-time pose and geometry have
        // already passed association/update validation above.
        let position_error_m = o.position_error_m
            + config.pose_position_error_m
            + o.position_body_m.x_m.hypot(o.position_body_m.y_m) * config.pose_heading_error_rad;
        let heading_error_rad = o.heading_error_rad + config.pose_heading_error_rad;
        if position_error_m > config.max_position_error_m
            || heading_error_rad > config.max_heading_error_rad
        {
            continue;
        }
        let candidate = ElementTrack {
            id: TrackId(0), // Not a world identity or a task confirmation.
            kind: o.kind,
            color: o.color,
            position: pose.pose.body_to_world(o.position_body_m),
            heading_rad: o.heading_body_rad.map(|h| {
                (h + pose.pose.yaw_rad + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)
                    - std::f64::consts::PI
            }),
            geometry: o.geometry,
            confidence: o.confidence,
            position_error_m,
            heading_error_rad,
            first_seen: frame.captured_at,
            last_seen: frame.captured_at,
            last_visual_at: frame.captured_at,
            last_geometry_at: frame.captured_at,
            observations: 1,
            processed: false,
            valid: true,
        };
        observed_region_boundary(candidate, true)?;
        output[index] = Some(candidate);
    }
    Ok(output)
}

fn stop_region_permits(
    at: Timestamp,
    mut track: ElementTrack,
    error_at: Timestamp,
    config: &LocalWorldConfig,
    corners: &[xt_stcar_robot_core::autonomy::Point2],
) -> bool {
    if at < error_at || at < track.last_seen {
        return false;
    }
    let age_s = (at.0 - error_at.0) as f64 / 1000.;
    track.position_error_m += age_s * config.position_drift_mps;
    track.heading_error_rad += age_s * config.heading_drift_radps;
    // Track expiry removes positive task/green evidence, not the safe half of
    // this remembered negative constraint. Keep expanding uncertainty without
    // resetting its source clock: a complete envelope can still remain before
    // the conservative edge, or wholly outside its enlarged finite side band.
    // A stale observation never creates an identity or grants line crossing.
    observed_region_boundary(track, true)
        .is_ok_and(|boundary| boundary.contains_points(corners, 0.0))
}

fn same_stop_region(a: ElementTrack, b: ElementTrack, config: &LocalWorldConfig) -> bool {
    use xt_stcar_robot_core::local_world::ElementGeometry;
    let tolerance = config.association_distance_m + a.position_error_m + b.position_error_m;
    let heading_matches = a.heading_rad.zip(b.heading_rad).is_some_and(|(a, b)| {
        ((a - b + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI)
            .abs()
            <= config.contradiction_heading_rad
    });
    let geometry_matches = match (a.geometry, b.geometry) {
        (
            ElementGeometry::LineRegion {
                lateral_half_width_m: aw,
                depth_m: ad,
            },
            ElementGeometry::LineRegion {
                lateral_half_width_m: bw,
                depth_m: bd,
            },
        ) => {
            (aw - bw).abs() <= a.position_error_m + b.position_error_m
                && (ad - bd).abs() <= a.position_error_m + b.position_error_m
        }
        _ => false,
    };
    a.position.distance(b.position) <= tolerance && heading_matches && geometry_matches
}

#[cfg(test)]
#[path = "../tests/support/online_speed_hint.rs"]
mod speed_hint_tests;

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{FRAC_PI_2, TAU};
    use xt_stcar_robot_core::autonomy::{LightState, Point2, RoadObservation};
    use xt_stcar_robot_core::local_world::{
        ElementColor, ElementFrame, ElementGeometry, ElementKind, ElementObservation,
        ObservationSource,
    };

    #[test]
    fn recorded_stale_negative_line_preserves_safe_half_plane_and_growing_uncertainty() {
        let captured: serde_json::Value = serde_json::from_str(include_str!(
            "../tests/fixtures/online45180_stale_negative_boundary.json"
        ))
        .unwrap();
        assert_eq!(captured["known_free"], true);
        assert_eq!(captured["pending_permitted"], true);
        assert_eq!(captured["confirmed_permitted"], false);
        assert_eq!(captured["released"], false);
        let config: LocalWorldConfig =
            serde_json::from_value(captured["config"]["online"]["world"].clone()).unwrap();
        let saved = &captured["confirmed_stop_line"][0];
        macro_rules! read {
            ($field:ident) => {
                serde_json::from_value(saved[stringify!($field)].clone()).unwrap()
            };
        }
        let track = ElementTrack {
            id: read!(id),
            kind: read!(kind),
            color: read!(color),
            position: read!(position),
            heading_rad: read!(heading_rad),
            geometry: read!(geometry),
            confidence: read!(confidence),
            position_error_m: read!(position_error_m),
            heading_error_rad: read!(heading_error_rad),
            first_seen: read!(first_seen),
            last_seen: read!(last_seen),
            last_visual_at: read!(last_visual_at),
            last_geometry_at: read!(last_geometry_at),
            observations: read!(observations),
            processed: read!(processed),
            valid: read!(valid),
        };
        let error_at: Timestamp =
            serde_json::from_value(captured["confirmed_stop_line"][1].clone()).unwrap();
        let at: Timestamp = serde_json::from_value(captured["at"].clone()).unwrap();
        let corners: Vec<Point2> = serde_json::from_value(captured["corners"].clone()).unwrap();
        assert!(at.0 - track.last_seen.0 >= config.track_ttl_ms);
        assert!(stop_region_permits(at, track, error_at, &config, &corners));
        // Crossing the conservative far edge remains forbidden, including
        // an envelope whose interior intersects the finite lateral band.
        let heading = track.heading_rad.unwrap();
        let crossed: Vec<_> = corners
            .iter()
            .map(|p| Point2 {
                x_m: p.x_m + 2.0 * heading.cos(),
                y_m: p.y_m + 2.0 * heading.sin(),
            })
            .collect();
        assert!(!stop_region_permits(at, track, error_at, &config, &crossed));
        // Uncertainty keeps growing after the positive tracking lease expires;
        // a later time can invalidate even this formerly safe envelope.
        assert!(!stop_region_permits(
            Timestamp(at.0 + 20_000),
            track,
            error_at,
            &config,
            &corners
        ));
        assert!(!stop_region_permits(
            Timestamp(error_at.0 - 1),
            track,
            error_at,
            &config,
            &corners
        ));
        assert_eq!(track.last_geometry_at, Timestamp(39_600));
        assert_eq!(error_at, Timestamp(41_380));
        assert!(!track.processed);
    }

    fn sample(
        config: &AutonomyConfig,
        at: u64,
        yaw: f64,
        visual: bool,
    ) -> (PoseEstimate, LidarSample, RoadFrame) {
        let p = PoseEstimate {
            captured_at: Timestamp(at),
            frame_id: config.navigation.frame_id.clone(),
            pose: Pose2 {
                x_m: 0.,
                y_m: 0.,
                yaw_rad: yaw,
            },
            speed_mps: 0.,
            yaw_rate_radps: 0.,
            quality: 1.,
        };
        let center = p.pose.world_to_body(Point2 { x_m: 2., y_m: 0. });
        let laser_center = config.lidar_in_body.world_to_body(center);
        let increment = TAU / config.scan.bins as f64;
        let ranges_m = (0..config.scan.bins)
            .map(|i| {
                let angle = i as f64 * increment;
                let projection = laser_center.x_m * angle.cos() + laser_center.y_m * angle.sin();
                let cross = laser_center.x_m * angle.sin() - laser_center.y_m * angle.cos();
                Some(if projection > 0. && cross.abs() <= 0.2 {
                    projection - (0.04 - cross * cross).sqrt()
                } else {
                    5.
                })
            })
            .collect();
        let scan = LidarSample {
            captured_at: Timestamp(at),
            frame_id: config.scan.frame_id.clone(),
            angle_min_rad: 0.,
            angle_increment_rad: increment,
            range_min_m: 0.05,
            range_max_m: 20.,
            ranges_m,
        };
        let observations = if visual {
            vec![ElementObservation {
                kind: ElementKind::Cone,
                color: ElementColor::Red,
                position_body_m: center,
                heading_body_rad: None,
                geometry: ElementGeometry::Cone { radius_m: 0.2 },
                source: ObservationSource::GroundProjection,
                confidence: 0.9,
                position_error_m: 0.03,
                heading_error_rad: 0.,
            }]
        } else {
            vec![]
        };
        let road = RoadFrame {
            observation: RoadObservation {
                captured_at: Timestamp(at),
                frame_id: config.mission.body_frame.clone(),
                crosswalk: None,
                light: LightState::Unknown,
                light_confidence: 0.,
                cones_body_m: vec![],
            },
            elements: Some(ElementFrame {
                captured_at: Timestamp(at),
                frame_id: config.mission.body_frame.clone(),
                observations,
            }),
            image_width_px: 160,
            image_height_px: 120,
        };
        (p, scan, road)
    }
    #[test]
    fn online_adapter_maintains_confirmed_cone_from_side_scan_after_camera_disappears() {
        let config = crate::online_simulation::controller_config();
        let mut session = OnlineSession::new(config.online.clone().unwrap(), &config).unwrap();
        session.start().unwrap();
        for at in [0, 100] {
            let (p, scan, road) = sample(&config, at, 0., true);
            session
                .update(Timestamp(at), &p, &scan, &road, config.lidar_in_body)
                .unwrap();
        }
        let initial = session.world.tracks(Timestamp(100)).next().unwrap();
        for at in (200..=2100).step_by(100) {
            let (p, scan, road) = sample(&config, at, FRAC_PI_2, false);
            session
                .update(Timestamp(at), &p, &scan, &road, config.lidar_in_body)
                .unwrap();
            let track = session.world.tracks(Timestamp(at)).next().unwrap();
            assert_eq!(
                (track.id, track.color, track.observations),
                (initial.id, ElementColor::Red, 2)
            );
            assert_eq!(track.last_visual_at, Timestamp(100));
            assert_eq!(track.last_geometry_at, Timestamp(at));
            assert!(track.position.distance(initial.position) < 1e-9);
            assert!(track.position_error_m <= session.world.config().max_position_error_m);
        }
    }
    #[test]
    fn online_task_region_reference_survives_new_landmark_and_terminal_owner_releases_it() {
        for terminal in [MissionPhase::Fault, MissionPhase::Completed] {
            let config = crate::online_simulation::controller_config();
            let mut session = OnlineSession::new(config.online.clone().unwrap(), &config).unwrap();
            session.start().unwrap();
            let marker = ElementObservation {
                kind: ElementKind::StopLine,
                color: ElementColor::White,
                position_body_m: Point2 { x_m: 2.0, y_m: 0.0 },
                heading_body_rad: Some(0.0),
                geometry: ElementGeometry::LineRegion {
                    lateral_half_width_m: 0.4,
                    depth_m: 1.0,
                },
                source: ObservationSource::GroundMarker,
                confidence: 0.9,
                position_error_m: 0.03,
                heading_error_rad: 0.01,
            };
            let mut finish = marker;
            finish.kind = ElementKind::FinishMarker;
            finish.position_body_m.x_m = 4.0;
            let mut last = None;
            let mut id = None;
            // Task-level sensor sequence, not a claimed physical trajectory.
            // The line is initially confirmed, hidden beyond the original TTL,
            // and then an unrelated finish marker arrives before it reappears.
            for at in (0..=3200).step_by(100) {
                let (pose, scan, mut road) = sample(&config, at, 0.0, false);
                road.elements.as_mut().unwrap().observations = if at <= 100 || at >= 3100 {
                    vec![marker]
                } else if at >= 2200 {
                    vec![finish]
                } else {
                    vec![]
                };
                let report = session
                    .update(Timestamp(at), &pose, &scan, &road, config.lidar_in_body)
                    .unwrap();
                assert_ne!(report.mission.phase, MissionPhase::Fault);
                if at == 100 {
                    id = session.mission.last_observed_light_region().map(|t| t.id);
                }
                if (1900..=3100).contains(&at) {
                    assert!(
                        !session
                            .world
                            .tracks(Timestamp(at))
                            .any(|t| Some(t.id) == id)
                    );
                }
                last = Some(report);
            }
            let id = id.unwrap();
            let fresh = session
                .world
                .tracks(Timestamp(3200))
                .find(|t| t.id == id)
                .unwrap();
            assert_eq!(fresh.observations, 2);
            assert_eq!(fresh.first_seen, Timestamp(0));
            assert_eq!(session.confirmed_stop_line.unwrap().0.id, id);
            assert_eq!(
                session.confirmed_stop_line.unwrap().0.last_seen,
                Timestamp(3200)
            );
            // Explicit lifecycle call checks both terminal owners. It grants no
            // completion/Drive; the report's real task admission remains separate.
            let mut terminal_report = last.unwrap();
            terminal_report.mission.phase = terminal;
            session
                .retain_task_regions(Timestamp(3200), &terminal_report)
                .unwrap();
            let (pose, _, _) = sample(&config, 6000, 0.0, false);
            let mut cone = marker;
            cone.kind = ElementKind::Cone;
            cone.color = ElementColor::Red;
            cone.position_body_m.x_m = 6.0;
            cone.heading_body_rad = None;
            cone.geometry = ElementGeometry::Cone { radius_m: 0.2 };
            cone.source = ObservationSource::VisualLidar;
            session
                .world
                .update(
                    Timestamp(6000),
                    &pose,
                    &ElementFrame {
                        captured_at: Timestamp(6000),
                        frame_id: config.mission.body_frame.clone(),
                        observations: vec![cone],
                    },
                )
                .unwrap();
            assert!(
                session
                    .world
                    .set_region_pins(Timestamp(6000), &[id])
                    .is_err()
            );
        }
    }

    #[test]
    fn online_adapter_rejects_mismatched_extrinsic_before_world_mutation() {
        let config = crate::online_simulation::controller_config();
        let mut session = OnlineSession::new(config.online.clone().unwrap(), &config).unwrap();
        let (p, scan, road) = sample(&config, 0, 0., true);
        let mut wrong = config.lidar_in_body;
        wrong.x_m += 0.1;
        assert!(
            session
                .update(Timestamp(0), &p, &scan, &road, wrong)
                .is_err()
        );
        // Retrying the same timestamp with original input remains valid: neither
        // the cached scan nor the visual confirmations were partly committed.
        session
            .update(Timestamp(0), &p, &scan, &road, config.lidar_in_body)
            .unwrap();
        assert_eq!(session.world.tracks(Timestamp(0)).count(), 0);
        let (p, scan, road) = sample(&config, 100, 0., true);
        session
            .update(Timestamp(100), &p, &scan, &road, config.lidar_in_body)
            .unwrap();
        assert_eq!(
            session
                .world
                .tracks(Timestamp(100))
                .next()
                .unwrap()
                .observations,
            2
        );
    }
}

#[cfg(test)]
#[path = "../tests/support/online_source_stop_speed_hint.rs"]
mod online_source_stop_speed_hint;
