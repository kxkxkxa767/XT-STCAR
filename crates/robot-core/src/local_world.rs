//! Bounded, short-lived element memory and observed local free space.
//!
//! Positions use one local odometry frame, not a surveyed field map. No-return
//! beams are unknown. A free-space answer is a conservative certificate under
//! the calibrated adjacent-beam surface model, never a physical-world proof.
use crate::autonomy::{Footprint, Point2, Pose2, PoseEstimate};
use crate::{FrameId, LidarSample, Timestamp, ValidationError};
use serde::{Deserialize, Serialize};
use std::f64::consts::{PI, TAU};

pub const MAX_ELEMENTS: usize = 16;
pub const MAX_LOCAL_RAYS: usize = 1440;
const MAX_CORRIDOR_DISCS: usize = 128;
// One original circle query plus at most 63 adaptive OBB queries per hull.
const MAX_HULL_CELLS: usize = 63;

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(transparent)]
pub struct TrackId(pub u64);

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ElementKind {
    Cone,
    Crosswalk,
    StopLine,
    FinishMarker,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ElementColor {
    Unknown,
    Red,
    Blue,
    White,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
#[serde(tag = "shape", rename_all = "snake_case", deny_unknown_fields)]
pub enum ElementGeometry {
    Cone {
        radius_m: f64,
    },
    /// Front-edge midpoint, crossing heading, and region extending FORWARD.
    /// StopLine denotes the marked stop AREA: its far edge is the travel limit.
    LineRegion {
        lateral_half_width_m: f64,
        depth_m: f64,
    },
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ObservationSource {
    GroundProjection,
    VisualLidar,
    GroundMarker,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct ElementObservation {
    pub kind: ElementKind,
    pub color: ElementColor,
    pub position_body_m: Point2,
    pub heading_body_rad: Option<f64>,
    pub geometry: ElementGeometry,
    pub source: ObservationSource,
    pub confidence: f64,
    pub position_error_m: f64,
    pub heading_error_rad: f64,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct ElementFrame {
    pub captured_at: Timestamp,
    pub frame_id: FrameId,
    pub observations: Vec<ElementObservation>,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct ElementTrack {
    pub id: TrackId,
    pub kind: ElementKind,
    pub color: ElementColor,
    pub position: Point2,
    pub heading_rad: Option<f64>,
    pub geometry: ElementGeometry,
    pub confidence: f64,
    pub position_error_m: f64,
    pub heading_error_rad: f64,
    pub first_seen: Timestamp,
    /// Latest accepted geometry timestamp; retained for existing consumers.
    pub last_seen: Timestamp,
    pub last_visual_at: Timestamp,
    pub last_geometry_at: Timestamp,
    /// Distinct visual frames in the current confirmation sequence. Retained
    /// stop/finish regions restart at one after expiry; range-only maintenance
    /// never increments this count.
    pub observations: u32,
    pub processed: bool,
    pub valid: bool,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct LocalWorldConfig {
    pub body_frame: FrameId,
    pub lidar_frame: FrameId,
    pub world_frame: FrameId,
    /// The actual self body at the scan timestamp may cover its near-range blind
    /// disc. This does not label unobserved space outside that footprint free.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub self_footprint: Option<Footprint>,
    pub track_ttl_ms: u64,
    /// Maximum age of the last real visual confirmation while lidar maintains a
    /// cone. This finite semantic lease is independent of geometry freshness.
    pub cone_semantic_ttl_ms: u64,
    pub scan_ttl_ms: u64,
    pub min_confidence: f64,
    pub min_confirmations: u32,
    pub min_pose_quality: f64,
    pub max_position_error_m: f64,
    pub max_heading_error_rad: f64,
    pub position_drift_mps: f64,
    pub heading_drift_radps: f64,
    pub pose_position_error_m: f64,
    pub pose_heading_error_rad: f64,
    pub association_distance_m: f64,
    pub contradiction_heading_rad: f64,
    pub laser_error_m: f64,
}

impl LocalWorldConfig {
    pub fn simulation(body_frame: FrameId, lidar_frame: FrameId, world_frame: FrameId) -> Self {
        Self {
            body_frame,
            lidar_frame,
            world_frame,
            self_footprint: None,
            track_ttl_ms: 1800,
            // Synthetic .85 m half-orbit / .18 m/s = 14.84 s; 20 s leaves
            // bounded maneuver overhead. Slower/larger orbits must stop/reobserve.
            cone_semantic_ttl_ms: 20_000,
            scan_ttl_ms: 300,
            min_confidence: 0.6,
            min_confirmations: 2,
            min_pose_quality: 0.6,
            max_position_error_m: 0.35,
            max_heading_error_rad: 0.3,
            position_drift_mps: 0.03,
            heading_drift_radps: 0.02,
            pose_position_error_m: 0.02,
            pose_heading_error_rad: 0.01,
            association_distance_m: 0.3,
            contradiction_heading_rad: 0.6,
            laser_error_m: 0.02,
        }
    }
    pub fn validate(&self) -> Result<(), ValidationError> {
        self.body_frame.validate()?;
        self.lidar_frame.validate()?;
        self.world_frame.validate()?;
        if let Some(footprint) = self.self_footprint {
            footprint.validate()?;
        }
        if !(1..=10_000).contains(&self.track_ttl_ms)
            || !(self.track_ttl_ms..=60_000).contains(&self.cone_semantic_ttl_ms)
            || !(1..=1000).contains(&self.scan_ttl_ms)
            || !(2..=100).contains(&self.min_confirmations)
            || !unit(self.min_confidence)
            || self.min_confidence == 0.0
            || !unit(self.min_pose_quality)
            || self.min_pose_quality == 0.0
            || !between(self.max_position_error_m, 0.001, 1.0)
            || !between(self.max_heading_error_rad, 0.001, 0.8)
            || !between(self.position_drift_mps, 0.0, 1.0)
            || !between(self.heading_drift_radps, 0.0, 1.0)
            || !between(self.pose_position_error_m, 0.0, self.max_position_error_m)
            || !between(self.pose_heading_error_rad, 0.0, self.max_heading_error_rad)
            || !between(self.association_distance_m, 0.01, 1.0)
            || !between(self.contradiction_heading_rad, 0.01, PI)
            || !between(self.laser_error_m, 0.001, 0.5)
        {
            return Err(error(
                "invalid local-world confidence, uncertainty or time bounds",
            ));
        }
        Ok(())
    }
}

#[derive(Clone)]
struct StoredFrame {
    at: Timestamp,
    pose: PoseEstimate,
    observations: [Option<ElementObservation>; MAX_ELEMENTS],
    len: usize,
}
#[derive(Clone)]
struct StoredScan {
    at: Timestamp,
    pose: PoseEstimate,
    lidar_in_body: Pose2,
    ranges: [Option<f64>; MAX_LOCAL_RAYS],
    len: usize,
    angle_min: f64,
    increment: f64,
    range_min: f64,
    range_max: f64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum SpaceState {
    KnownFree,
    Obstacle,
    Unknown,
}

pub struct LocalWorld {
    config: LocalWorldConfig,
    elements: [Option<ElementTrack>; MAX_ELEMENTS],
    // Explicit task references only; a pin retains identity storage, never
    // positive geometry validity or its observation/confirmation clocks.
    region_pins: [bool; MAX_ELEMENTS],
    // Latest accepted VISUAL world-geometry error is a floor on subsequent
    // independent range fits, not a per-scan error to add repeatedly.
    visual_error_floor: [Option<(f64, f64)>; MAX_ELEMENTS],
    last_maintenance: Option<(Timestamp, ConeAssociationConfig)>,
    next_id: u64,
    last_frame: Option<StoredFrame>,
    scan: Option<StoredScan>,
}

impl LocalWorld {
    pub fn new(config: LocalWorldConfig) -> Result<Self, ValidationError> {
        config.validate()?;
        Ok(Self {
            config,
            elements: [None; MAX_ELEMENTS],
            region_pins: [false; MAX_ELEMENTS],
            visual_error_floor: [None; MAX_ELEMENTS],
            last_maintenance: None,
            next_id: 1,
            last_frame: None,
            scan: None,
        })
    }
    pub fn config(&self) -> &LocalWorldConfig {
        &self.config
    }

    /// Atomically retain the existing stop/finish identities referenced by the
    /// task. A new pin requires currently confirmed, fresh geometry. An existing
    /// pin may survive expiry or contradiction so its old negative constraint
    /// cannot silently lose identity. Pins never renew any observation or make
    /// `tracks` expose stale/invalid geometry. Omitted identities are unpinned.
    pub fn set_region_pins(
        &mut self,
        at: Timestamp,
        identities: &[TrackId],
    ) -> Result<(), ValidationError> {
        if identities.len() > MAX_ELEMENTS {
            return Err(error("retained region identity capacity exceeded"));
        }
        let mut next = [false; MAX_ELEMENTS];
        for identity in identities {
            let slot = self
                .elements
                .iter()
                .position(|track| track.is_some_and(|track| track.id == *identity))
                .ok_or_else(|| error("cannot retain an unknown or recycled region identity"))?;
            let track = self.elements[slot].expect("matched existing identity");
            if next[slot]
                || !matches!(
                    track.kind,
                    ElementKind::StopLine | ElementKind::FinishMarker
                )
                || (!self.region_pins[slot] && !self.tracks(at).any(|track| track.id == *identity))
            {
                return Err(error(
                    "new region pins require unique confirmed fresh identities",
                ));
            }
            next[slot] = true;
        }
        self.region_pins = next;
        Ok(())
    }

    /// Duplicate immutable frames are no-ops; they cannot manufacture confirmation.
    /// Contradictions quarantine identities. Retained stop/finish region slots
    /// remain association candidates after expiry, so invalid region identities
    /// cannot revive there. Other unprocessed expired slots keep the original
    /// recycling rule; processed slots are never evicted. A valid region needs
    /// fresh distinct confirmations after expiry, with its processed flag latched.
    pub fn update(
        &mut self,
        at: Timestamp,
        pose: &PoseEstimate,
        frame: &ElementFrame,
    ) -> Result<(), ValidationError> {
        self.validate_pose(at, pose, frame.captured_at, self.config.track_ttl_ms)?;
        if frame.frame_id != self.config.body_frame || frame.observations.len() > MAX_ELEMENTS {
            return Err(error("element frame mismatch or capacity exceeded"));
        }
        for observation in &frame.observations {
            validate_observation(observation, true)?;
        }
        if let Some(old) = &self.last_frame {
            if frame.captured_at < old.at {
                return Err(error("element timestamp regressed"));
            }
            if frame.captured_at == old.at {
                if old.pose != *pose
                    || old.len != frame.observations.len()
                    || frame
                        .observations
                        .iter()
                        .enumerate()
                        .any(|(i, o)| old.observations[i] != Some(*o))
                {
                    return Err(error(
                        "element frame changed without advancing capture time",
                    ));
                }
                return Ok(());
            }
        }
        // Work on a copy: malformed/capacity-exceeding batches cannot partly update.
        let mut elements = self.elements;
        let mut visual_error_floor = self.visual_error_floor;
        let mut next_id = self.next_id;
        let mut touched = [false; MAX_ELEMENTS];
        for observation in &frame.observations {
            if observation.confidence < self.config.min_confidence {
                continue;
            }
            let position = pose.pose.body_to_world(observation.position_body_m);
            let distance = observation
                .position_body_m
                .x_m
                .hypot(observation.position_body_m.y_m);
            let position_error = observation.position_error_m
                + self.config.pose_position_error_m
                + distance * self.config.pose_heading_error_rad;
            let heading_error = observation.heading_error_rad + self.config.pose_heading_error_rad;
            if !position.valid()
                || !position_error.is_finite()
                || position_error > self.config.max_position_error_m
                || heading_error > self.config.max_heading_error_rad
            {
                continue;
            }
            let heading = observation
                .heading_body_rad
                .map(|h| wrap(h + pose.pose.yaw_rad));
            let mut matches = [false; MAX_ELEMENTS];
            let mut count = 0;
            for (i, slot) in elements.iter().enumerate() {
                if let Some(track) = slot
                    && track.kind == observation.kind
                    && (track.processed
                        // Retained region identities can be reobserved after
                        // occlusion. Invalid slots still participate in the
                        // original association/ambiguity checks and cannot be
                        // revived by allocating a nearby fresh identity.
                        || matches!(track.kind, ElementKind::StopLine | ElementKind::FinishMarker)
                        || frame.captured_at.0.saturating_sub(track.last_seen.0)
                            < self.config.track_ttl_ms)
                    && track.position.distance(position)
                        <= self.config.association_distance_m
                            + track.position_error_m
                            + position_error
                {
                    matches[i] = true;
                    count += 1;
                }
            }
            if count > 1 {
                for (i, matched) in matches.iter().enumerate() {
                    if *matched {
                        elements[i].as_mut().expect("matched slot").valid = false;
                    }
                }
                continue;
            }
            if let Some(i) = matches.iter().position(|v| *v) {
                let track = elements[i].as_mut().expect("matched slot");
                let color_conflict = track.color != ElementColor::Unknown
                    && observation.color != ElementColor::Unknown
                    && track.color != observation.color;
                let heading_conflict = match (track.heading_rad, heading) {
                    (Some(a), Some(b)) => wrap(a - b).abs() > self.config.contradiction_heading_rad,
                    (None, None) => false,
                    _ => true,
                };
                let geometry_conflict = !compatible_geometry(
                    track.geometry,
                    observation.geometry,
                    position_error + track.position_error_m,
                );
                if touched[i] || color_conflict || heading_conflict || geometry_conflict {
                    track.valid = false;
                    continue;
                }
                touched[i] = true;
                // Passed regions still supply fresh clearance geometry until
                // the whole body clears them. Keep their processed identity;
                // refreshing measurements never makes them selectable again.
                let retain_region_geometry = matches!(
                    track.kind,
                    ElementKind::StopLine | ElementKind::FinishMarker
                );
                if !track.valid || (track.processed && !retain_region_geometry) {
                    continue;
                }
                let restart_confirmation = retain_region_geometry
                    && frame.captured_at.0.saturating_sub(track.last_seen.0)
                        >= self.config.track_ttl_ms;
                track.position = retain_roundoff_position(track.position, position);
                track.heading_rad = retain_roundoff_heading(track.heading_rad, heading);
                track.geometry = observation.geometry;
                if observation.color != ElementColor::Unknown {
                    track.color = observation.color;
                }
                track.position_error_m = position_error;
                track.heading_error_rad = heading_error;
                track.confidence = observation.confidence;
                track.last_seen = frame.captured_at;
                track.last_visual_at = frame.captured_at;
                track.last_geometry_at = frame.captured_at;
                visual_error_floor[i] = Some((position_error, heading_error));
                track.observations = if restart_confirmation {
                    // Preserve id, first_seen and processed status, but old
                    // confirmations cannot turn one new frame into fresh
                    // positive geometry. Duplicate frames return above.
                    1
                } else {
                    track.observations.saturating_add(1)
                };
            } else {
                let slot = elements
                    .iter()
                    .enumerate()
                    .position(|(index, s)| {
                        !self.region_pins[index]
                            && s.is_none_or(|t| {
                                !t.processed
                                    && frame.captured_at.0.saturating_sub(t.last_seen.0)
                                        >= self.config.track_ttl_ms
                            })
                    })
                    .ok_or_else(|| error("local element identity capacity exhausted"))?;
                let id = TrackId(next_id);
                next_id = next_id
                    .checked_add(1)
                    .ok_or_else(|| error("element identity exhausted"))?;
                elements[slot] = Some(ElementTrack {
                    id,
                    kind: observation.kind,
                    color: observation.color,
                    position,
                    heading_rad: heading,
                    geometry: observation.geometry,
                    confidence: observation.confidence,
                    position_error_m: position_error,
                    heading_error_rad: heading_error,
                    first_seen: frame.captured_at,
                    last_seen: frame.captured_at,
                    last_visual_at: frame.captured_at,
                    last_geometry_at: frame.captured_at,
                    observations: 1,
                    processed: false,
                    valid: true,
                });
                visual_error_floor[slot] = Some((position_error, heading_error));
                touched[slot] = true;
            }
        }
        let mut observations = [None; MAX_ELEMENTS];
        for (i, o) in frame.observations.iter().enumerate() {
            observations[i] = Some(*o);
        }
        self.elements = elements;
        self.visual_error_floor = visual_error_floor;
        self.next_id = next_id;
        self.last_frame = Some(StoredFrame {
            at: frame.captured_at,
            pose: pose.clone(),
            observations,
            len: frame.observations.len(),
        });
        Ok(())
    }

    /// Returns only confirmed, fresh, noncontradictory tracks. Processed identities
    /// retain their flag: task logic must not choose them for another action.
    pub fn tracks(&self, at: Timestamp) -> impl Iterator<Item = ElementTrack> + '_ {
        self.elements
            .iter()
            .flatten()
            .copied()
            .filter_map(move |mut t| {
                if at < t.last_seen
                    || at.0 - t.last_seen.0 >= self.config.track_ttl_ms
                    || (t.kind == ElementKind::Cone
                        && (at < t.last_visual_at
                            || at.0 - t.last_visual_at.0 >= self.config.cone_semantic_ttl_ms))
                    || !t.valid
                    || t.observations < self.config.min_confirmations
                {
                    return None;
                }
                let age_s = (at.0 - t.last_seen.0) as f64 / 1000.0;
                t.position_error_m += age_s * self.config.position_drift_mps;
                t.heading_error_rad += age_s * self.config.heading_drift_radps;
                if t.position_error_m > self.config.max_position_error_m
                    || t.heading_error_rad > self.config.max_heading_error_rad
                {
                    return None;
                }
                Some(t)
            })
    }

    /// Maintain only an existing visually confirmed, valid, unprocessed cone
    /// using the latest validated same-time pose/scan and explicit extrinsic.
    /// Does not create identities, colors, visual confirmations or semantic time.
    /// Missing/ambiguous fits do not renew geometry; the original TTL still wins.
    pub fn maintain_confirmed_cones(
        &mut self,
        at: Timestamp,
        association: &ConeAssociationConfig,
    ) -> Result<usize, ValidationError> {
        association.validate()?;
        let scan = self
            .scan
            .as_ref()
            .ok_or_else(|| error("cone maintenance requires a validated scan"))?;
        self.validate_pose(at, &scan.pose, scan.at, self.config.scan_ttl_ms)?;
        if association.body_frame != self.config.body_frame
            || association.lidar_frame != self.config.lidar_frame
            || association.lidar_in_body != scan.lidar_in_body
            || association.range_error_m < self.config.laser_error_m
            || self.last_frame.as_ref().is_some_and(|frame| {
                frame.at > scan.at || (frame.at == scan.at && frame.pose != scan.pose)
            })
        {
            return Err(error(
                "cone maintenance frame, pose, extrinsic, range bound or capture order mismatch",
            ));
        }
        if let Some((stamp, old)) = &self.last_maintenance {
            if scan.at < *stamp {
                return Err(error("cone maintenance capture regressed"));
            }
            if scan.at == *stamp {
                return if old == association {
                    Ok(0)
                } else {
                    Err(error(
                        "cone maintenance changed configuration at the same timestamp",
                    ))
                };
            }
        }
        let mut fits = [None; MAX_ELEMENTS];
        let mut errors = [None; MAX_ELEMENTS];
        for (i, track) in self
            .elements
            .iter()
            .enumerate()
            .filter_map(|(i, t)| t.map(|t| (i, t)))
        {
            if track.kind != ElementKind::Cone
                || track.processed
                || !track.valid
                || track.observations < self.config.min_confirmations
                || at < track.last_geometry_at
                || scan.at < track.last_geometry_at
                || at.0 - track.last_geometry_at.0 >= self.config.track_ttl_ms
                || at < track.last_visual_at
                || at.0 - track.last_visual_at.0 >= self.config.cone_semantic_ttl_ms
            {
                continue;
            }
            let age = (scan.at.0 - track.last_geometry_at.0) as f64 / 1000.;
            let predicted_error = track.position_error_m + age * self.config.position_drift_mps;
            let predicted_heading_error =
                track.heading_error_rad + age * self.config.heading_drift_radps;
            if predicted_error > self.config.max_position_error_m
                || predicted_heading_error > self.config.max_heading_error_rad
            {
                continue;
            }
            let position = scan.pose.pose.world_to_body(track.position);
            let distance = position.x_m.hypot(position.y_m);
            let pose_error =
                self.config.pose_position_error_m + distance * self.config.pose_heading_error_rad;
            let candidate = ElementObservation {
                kind: ElementKind::Cone,
                color: track.color,
                position_body_m: position,
                heading_body_rad: None,
                geometry: track.geometry,
                source: ObservationSource::VisualLidar,
                confidence: track.confidence,
                position_error_m: predicted_error + pose_error,
                heading_error_rad: 0.,
            };
            let Some(fit) = unique_cone_fit(
                &candidate,
                &scan.ranges[..scan.len],
                scan.angle_min,
                scan.increment,
                association,
            ) else {
                continue;
            };
            // Reserve every unique plausible cluster before its measurement
            // error gate, so a second identity cannot claim the same beams.
            fits[i] = Some(fit);
            let (visual_position_floor, visual_heading_floor) = self.visual_error_floor[i]
                .ok_or_else(|| error("confirmed cone lacks visual uncertainty evidence"))?;
            // A new range fit is an independent geometry measurement. Preserve
            // the visual floor; do not accumulate that same error every scan.
            let new_error = visual_position_floor.max(fit.center_error_m + pose_error);
            let new_heading_error = visual_heading_floor.max(self.config.pose_heading_error_rad);
            if !new_error.is_finite()
                || new_error > self.config.max_position_error_m
                || new_heading_error > self.config.max_heading_error_rad
            {
                continue;
            }
            errors[i] = Some((new_error, new_heading_error));
        }
        let mut ambiguous = [false; MAX_ELEMENTS];
        for i in 0..MAX_ELEMENTS {
            for j in i + 1..MAX_ELEMENTS {
                if let (Some(a), Some(b)) = (fits[i], fits[j])
                    && shared_beams(a, b, scan.len)
                {
                    ambiguous[i] = true;
                    ambiguous[j] = true;
                }
            }
        }
        let mut elements = self.elements;
        let mut maintained = 0;
        for i in 0..MAX_ELEMENTS {
            let (Some(fit), Some((position_error, heading_error)), Some(track)) =
                (fits[i], errors[i], elements[i].as_mut())
            else {
                continue;
            };
            if ambiguous[i] || track.last_geometry_at == scan.at {
                continue;
            }
            let position = scan.pose.pose.body_to_world(fit.center);
            if !position.valid() {
                return Err(error("cone maintenance produced invalid world geometry"));
            }
            track.position = retain_roundoff_position(track.position, position);
            track.position_error_m = position_error;
            track.heading_error_rad = heading_error;
            track.last_geometry_at = scan.at;
            track.last_seen = scan.at;
            maintained += 1;
        }
        self.elements = elements;
        self.last_maintenance = Some((scan.at, association.clone()));
        Ok(maintained)
    }

    pub fn mark_processed(&mut self, id: TrackId) -> Result<(), ValidationError> {
        let track = self
            .elements
            .iter_mut()
            .flatten()
            .find(|t| t.id == id)
            .ok_or_else(|| error("unknown element identity"))?;
        track.processed = true;
        Ok(())
    }

    /// The local snapshot is replaced, not accumulated as an implicit SLAM map.
    pub fn update_scan(
        &mut self,
        at: Timestamp,
        pose: &PoseEstimate,
        scan: &LidarSample,
        lidar_in_body: Pose2,
    ) -> Result<(), ValidationError> {
        self.validate_pose(at, pose, scan.captured_at, self.config.scan_ttl_ms)?;
        validate_scan(scan, &self.config.lidar_frame, lidar_in_body)?;
        if let Some(old) = &self.scan {
            if scan.captured_at < old.at {
                return Err(error("local scan timestamp regressed"));
            }
            if scan.captured_at == old.at {
                if old.pose != *pose
                    || old.lidar_in_body != lidar_in_body
                    || old.len != scan.ranges_m.len()
                    || old.angle_min != scan.angle_min_rad
                    || old.increment != scan.angle_increment_rad
                    || old.range_min != scan.range_min_m
                    || old.range_max != scan.range_max_m
                    || old.ranges[..old.len] != scan.ranges_m
                {
                    return Err(error("local scan changed without advancing capture time"));
                }
                return Ok(());
            }
        }
        let mut ranges = [None; MAX_LOCAL_RAYS];
        ranges[..scan.ranges_m.len()].copy_from_slice(&scan.ranges_m);
        self.scan = Some(StoredScan {
            at: scan.captured_at,
            pose: pose.clone(),
            lidar_in_body,
            ranges,
            len: scan.ranges_m.len(),
            angle_min: scan.angle_min_rad,
            increment: scan.angle_increment_rad,
            range_min: scan.range_min_m,
            range_max: scan.range_max_m,
        });
        Ok(())
    }

    pub fn space_state(&self, at: Timestamp, center: Point2, radius_m: f64) -> SpaceState {
        let Some(scan) = self
            .scan
            .as_ref()
            .filter(|s| at >= s.at && at.0 - s.at.0 < self.config.scan_ttl_ms)
        else {
            return SpaceState::Unknown;
        };
        if !center.valid() || !between(radius_m, 0.0, 20.0) {
            return SpaceState::Unknown;
        }
        let body = scan.pose.pose.world_to_body(center);
        let point = scan.lidar_in_body.world_to_body(body);
        let distance = point.x_m.hypot(point.y_m);
        let age_s = (at.0 - scan.at.0) as f64 / 1000.0;
        let error = self.config.laser_error_m
            + self.config.pose_position_error_m
            + age_s * self.config.position_drift_mps
            + body.x_m.hypot(body.y_m)
                * (self.config.pose_heading_error_rad + age_s * self.config.heading_drift_radps);
        let radius = radius_m + error;
        if !distance.is_finite() || !radius.is_finite() {
            return SpaceState::Unknown;
        }
        for i in 0..scan.len {
            if let Some(range) = scan.ranges[i] {
                let angle = scan.angle_min + i as f64 * scan.increment;
                let hit = Point2 {
                    x_m: range * angle.cos(),
                    y_m: range * angle.sin(),
                };
                if hit.distance(point) <= radius {
                    return SpaceState::Obstacle;
                }
            }
        }
        // A range return certifies no space inside the minimum-range blind
        // disc. Only an explicit current self-footprint prior may cover it.
        // Require the WHOLE blind disc to fit; partial overlap stays unknown.
        let blind_radius = scan.range_min + self.config.laser_error_m;
        if distance - radius < blind_radius {
            let self_covers_blind = self.config.self_footprint.is_some_and(|f| {
                let origin = scan.lidar_in_body.point();
                origin.x_m - blind_radius >= -f.rear_m
                    && origin.x_m + blind_radius <= f.front_m
                    && origin.y_m.abs() + blind_radius <= f.half_width_m
            });
            if !self_covers_blind {
                return SpaceState::Unknown;
            }
        }
        let angle = point.y_m.atan2(point.x_m);
        let half_span = if distance <= radius {
            PI
        } else {
            (radius / distance).asin()
        };
        let step = scan.increment.abs();
        // Adjacent-beam model: two finite returns certify only the triangular
        // sector truncated to the NEARER return. A far occlusion discontinuity
        // does not erase that near free portion. Beyond the shorter return the
        // sector remains unknown, even if its other ray travels much farther.
        // This is a sensor interpolation model, not proof about arbitrary thin
        // obstacles or real continuous space between measured laser beams.
        for i in 0..scan.len {
            let middle = scan.angle_min + (i as f64 + 0.5) * scan.increment;
            let nearest_angle = (wrap(middle - angle).abs() - step * 0.5).max(0.0);
            if half_span < PI && nearest_angle > half_span {
                continue;
            }
            let (Some(a), Some(b)) = (scan.ranges[i], scan.ranges[(i + 1) % scan.len]) else {
                return SpaceState::Unknown;
            };
            // Exact radial extent of the complete uncertainty-expanded disc
            // in this angular sector. Its maximum is at the angle nearest the
            // center bearing. Using d+r in every sector would compare the front
            // of the disc to a wall behind it and falsely reject safe motion.
            let perpendicular = distance * nearest_angle.sin();
            let discriminant = radius * radius - perpendicular * perpendicular;
            if discriminant < 0.0 {
                // Tangent roundoff stays conservative instead of skipping a
                // possibly intersecting sector.
                return SpaceState::Unknown;
            }
            let far = distance * nearest_angle.cos() + discriminant.sqrt();
            let roundoff = 8.0 * f64::EPSILON * (distance + radius + a.min(b)).max(1.0);
            if far + roundoff >= a.min(b) * step.cos() {
                return SpaceState::Unknown;
            }
        }
        SpaceState::KnownFree
    }

    /// Cover the COMPLETE capsule with overlapping discs. The expanded radius
    /// includes half a sampling interval, so this is not a point-sampling test.
    pub fn known_free_segment(
        &self,
        at: Timestamp,
        from: Point2,
        to: Point2,
        padding_m: f64,
    ) -> bool {
        if !from.valid() || !to.valid() || !between(padding_m, 0.0, 20.0) {
            return false;
        }
        let length = from.distance(to);
        if !length.is_finite() {
            return false;
        }
        if length < 1e-9 {
            return self.space_state(at, from, padding_m) == SpaceState::KnownFree;
        }
        let count = (length / 0.05).ceil() as usize;
        if count == 0 || count >= MAX_CORRIDOR_DISCS {
            return false;
        }
        let radius = padding_m + length / (2.0 * count as f64);
        (0..=count).all(|i| {
            let fraction = i as f64 / count as f64;
            self.space_state(
                at,
                Point2 {
                    x_m: from.x_m + (to.x_m - from.x_m) * fraction,
                    y_m: from.y_m + (to.y_m - from.y_m) * fraction,
                },
                radius,
            ) == SpaceState::KnownFree
        })
    }

    /// Certify the complete convex hull plus padding. Preserve the original
    /// enclosing-circle success path, then adaptively bisect the enclosing OBB.
    /// Every accepted leaf is a COMPLETE rectangle inside its certified disc;
    /// unknown, obstacle or budget exhaustion never certifies an uncovered leaf.
    /// At most 64 original space queries and a fixed stack are used per hull.
    pub fn known_free_convex_hull(&self, at: Timestamp, points: &[Point2], padding_m: f64) -> bool {
        let Some((center, radius)) = hull_disc(points, padding_m) else {
            return false;
        };
        if self.space_state(at, center, radius) == SpaceState::KnownFree {
            return true;
        }
        let Some(bounds) = HullBounds::new(points, padding_m) else {
            return false;
        };
        bounds.cover_with(|center, radius| {
            self.space_state(at, center, radius) == SpaceState::KnownFree
        })
    }

    /// Three-valued evidence for bounded geometric candidate screening.
    /// Unknown never authorizes Drive. Failed free-space coverage is followed by
    /// a complete hit-to-convex-hull distance check: an obstacle in excess OBB or
    /// circle area must not invent a collision with the actual requested hull.
    pub fn convex_hull_space_state(
        &self,
        at: Timestamp,
        points: &[Point2],
        padding_m: f64,
    ) -> SpaceState {
        if hull_disc(points, padding_m).is_none() {
            return SpaceState::Unknown;
        }
        if self.known_free_convex_hull(at, points, padding_m) {
            return SpaceState::KnownFree;
        }
        let Some(scan) = self
            .scan
            .as_ref()
            .filter(|scan| at >= scan.at && at.0 - scan.at.0 < self.config.scan_ttl_ms)
        else {
            return SpaceState::Unknown;
        };
        let Some(hull) = HitHull::new(points) else {
            return SpaceState::Unknown;
        };
        let farthest_body = points
            .iter()
            .map(|point| {
                let p = scan.pose.pose.world_to_body(*point);
                p.x_m.hypot(p.y_m)
            })
            .fold(0.0, f64::max)
            + padding_m;
        let age_s = (at.0 - scan.at.0) as f64 / 1000.0;
        let position_error = self.config.laser_error_m
            + self.config.pose_position_error_m
            + age_s * self.config.position_drift_mps;
        let heading_error =
            self.config.pose_heading_error_rad + age_s * self.config.heading_drift_radps;
        // A missing/unknown sector does not mask an observed collision elsewhere.
        for (index, range) in scan.ranges[..scan.len].iter().enumerate() {
            if let Some(range) = range {
                let angle = scan.angle_min + index as f64 * scan.increment;
                let body = scan.lidar_in_body.body_to_world(Point2 {
                    x_m: range * angle.cos(),
                    y_m: range * angle.sin(),
                });
                let hit = scan.pose.pose.body_to_world(body);
                let error =
                    position_error + body.x_m.hypot(body.y_m).max(farthest_body) * heading_error;
                if let Some(distance) = hull.distance(hit)
                    && error.is_finite()
                    && distance <= padding_m + error + hull.roundoff
                {
                    return SpaceState::Obstacle;
                }
            }
        }
        SpaceState::Unknown
    }

    fn validate_pose(
        &self,
        at: Timestamp,
        pose: &PoseEstimate,
        stamp: Timestamp,
        ttl: u64,
    ) -> Result<(), ValidationError> {
        if stamp != pose.captured_at
            || stamp > at
            || at.0 - stamp.0 >= ttl
            || pose.frame_id != self.config.world_frame
            || !pose.pose.valid()
            || !pose.speed_mps.is_finite()
            || !pose.yaw_rate_radps.is_finite()
            || !unit(pose.quality)
            || pose.quality < self.config.min_pose_quality
        {
            return Err(error(
                "local world requires a fresh, valid pose at the observation timestamp",
            ));
        }
        Ok(())
    }
}

/// Fixed-size monotone-chain convex hull for negative hit-distance evidence.
/// At most 32 input vertices; 64 slots only accommodate temporary lower/upper
/// chains. This is independent of the positive OBB/cell coverage certificate.
struct HitHull {
    vertices: [Point2; 64],
    len: usize,
    roundoff: f64,
}
impl HitHull {
    fn new(points: &[Point2]) -> Option<Self> {
        if points.is_empty() || points.len() > 32 || points.iter().any(|p| !p.valid()) {
            return None;
        }
        let mut sorted = [Point2::default(); 32];
        sorted[..points.len()].copy_from_slice(points);
        // Small bounded insertion sort avoids temporary heap storage.
        for i in 1..points.len() {
            let point = sorted[i];
            let mut j = i;
            while j > 0
                && (point
                    .x_m
                    .total_cmp(&sorted[j - 1].x_m)
                    .then(point.y_m.total_cmp(&sorted[j - 1].y_m)))
                .is_lt()
            {
                sorted[j] = sorted[j - 1];
                j -= 1;
            }
            sorted[j] = point;
        }
        let mut count = 0;
        for i in 0..points.len() {
            if count == 0 || sorted[i] != sorted[count - 1] {
                sorted[count] = sorted[i];
                count += 1;
            }
        }
        let mut hull = Self {
            vertices: [Point2::default(); 64],
            len: 0,
            roundoff: 128.0
                * f64::EPSILON
                * points
                    .iter()
                    .map(|p| p.x_m.abs().max(p.y_m.abs()))
                    .fold(1.0, f64::max),
        };
        for point in sorted[..count].iter().copied() {
            while hull.len >= 2
                && turn(
                    hull.vertices[hull.len - 2],
                    hull.vertices[hull.len - 1],
                    point,
                )? <= 0.0
            {
                hull.len -= 1;
            }
            hull.vertices[hull.len] = point;
            hull.len += 1;
        }
        let lower = hull.len;
        for point in sorted[..count.saturating_sub(1)].iter().rev().copied() {
            while hull.len > lower
                && turn(
                    hull.vertices[hull.len - 2],
                    hull.vertices[hull.len - 1],
                    point,
                )? <= 0.0
            {
                hull.len -= 1;
            }
            hull.vertices[hull.len] = point;
            hull.len += 1;
        }
        if hull.len > 1 {
            hull.len -= 1;
        }
        Some(hull)
    }
    fn distance(&self, point: Point2) -> Option<f64> {
        if !point.valid() {
            return None;
        }
        if self.len == 1 {
            return Some(point.distance(self.vertices[0]));
        }
        let mut inside = self.len >= 3;
        let mut distance = f64::INFINITY;
        for i in 0..self.len {
            let a = self.vertices[i];
            let b = self.vertices[(i + 1) % self.len];
            inside &= turn(a, b, point)? >= 0.0;
            let dx = b.x_m - a.x_m;
            let dy = b.y_m - a.y_m;
            let length = dx.hypot(dy);
            if !length.is_finite() || length == 0.0 {
                return None;
            }
            let ux = dx / length;
            let uy = dy / length;
            let along = ((point.x_m - a.x_m) * ux + (point.y_m - a.y_m) * uy).clamp(0.0, length);
            distance = distance.min(point.distance(Point2 {
                x_m: a.x_m + along * ux,
                y_m: a.y_m + along * uy,
            }));
        }
        Some(if inside { 0.0 } else { distance })
    }
}
fn turn(a: Point2, b: Point2, c: Point2) -> Option<f64> {
    let value = (b.x_m - a.x_m) * (c.y_m - a.y_m) - (b.y_m - a.y_m) * (c.x_m - a.x_m);
    value.is_finite().then_some(value)
}

fn hull_disc(points: &[Point2], padding_m: f64) -> Option<(Point2, f64)> {
    if points.is_empty()
        || points.len() > 32
        || points.iter().any(|p| !p.valid())
        || !between(padding_m, 0.0, 20.0)
    {
        return None;
    }
    let center = Point2 {
        x_m: points.iter().map(|p| p.x_m / points.len() as f64).sum(),
        y_m: points.iter().map(|p| p.y_m / points.len() as f64).sum(),
    };
    let radius = points
        .iter()
        .map(|p| p.distance(center))
        .fold(0.0, f64::max)
        + padding_m;
    Some((center, radius))
}

/// The OBB contains every supplied vertex and their complete convex hull.
/// Expanding both axes by padding contains the hull's Minkowski sum with a disc.
struct HullBounds {
    origin: Point2,
    axis: Point2,
    cell: HullCell,
    roundoff: f64,
}
#[derive(Clone, Copy, Default)]
struct HullCell {
    min_x: f64,
    max_x: f64,
    min_y: f64,
    max_y: f64,
}
impl HullCell {
    /// Both children share the exact same represented split coordinate, so
    /// there is no uncovered seam. Non-progress at floating-point limits fails.
    fn split(self) -> Option<[Self; 2]> {
        if self.max_x - self.min_x >= self.max_y - self.min_y {
            let middle = self.min_x + (self.max_x - self.min_x) / 2.0;
            (middle > self.min_x && middle < self.max_x).then_some([
                Self {
                    max_x: middle,
                    ..self
                },
                Self {
                    min_x: middle,
                    ..self
                },
            ])
        } else {
            let middle = self.min_y + (self.max_y - self.min_y) / 2.0;
            (middle > self.min_y && middle < self.max_y).then_some([
                Self {
                    max_y: middle,
                    ..self
                },
                Self {
                    min_y: middle,
                    ..self
                },
            ])
        }
    }
}
impl HullBounds {
    fn new(points: &[Point2], padding_m: f64) -> Option<Self> {
        let &origin = points.first()?;
        let axis = points
            .iter()
            .skip(1)
            .find_map(|point| {
                let x = point.x_m - origin.x_m;
                let y = point.y_m - origin.y_m;
                let length = x.hypot(y);
                (length.is_finite() && length > 0.).then(|| Point2 {
                    x_m: x / length,
                    y_m: y / length,
                })
            })
            .unwrap_or(Point2 { x_m: 1., y_m: 0. });
        let (mut min_x, mut max_x, mut min_y, mut max_y) = (
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
        );
        let mut scale = origin
            .x_m
            .abs()
            .max(origin.y_m.abs())
            .max(padding_m)
            .max(1.);
        for point in points {
            let dx = point.x_m - origin.x_m;
            let dy = point.y_m - origin.y_m;
            let x = dx * axis.x_m + dy * axis.y_m;
            let y = -dx * axis.y_m + dy * axis.x_m;
            if !x.is_finite() || !y.is_finite() {
                return None;
            }
            min_x = min_x.min(x);
            max_x = max_x.max(x);
            min_y = min_y.min(y);
            max_y = max_y.max(y);
            scale = scale
                .max(point.x_m.abs())
                .max(point.y_m.abs())
                .max(x.abs())
                .max(y.abs());
        }
        // Expand projection, reconstruction and circle radii outward. This is
        // floating-point roundoff only; no physical clearance/error is reduced.
        let roundoff = 64. * f64::EPSILON * scale;
        min_x -= padding_m + roundoff;
        max_x += padding_m + roundoff;
        min_y -= padding_m + roundoff;
        max_y += padding_m + roundoff;
        let width = max_x - min_x;
        let height = max_y - min_y;
        if !width.is_finite() || !height.is_finite() || width <= 0. || height <= 0. {
            return None;
        }
        Some(Self {
            origin,
            axis,
            cell: HullCell {
                min_x,
                max_x,
                min_y,
                max_y,
            },
            roundoff,
        })
    }
    fn disc(&self, cell: HullCell) -> (Point2, f64) {
        let width = cell.max_x - cell.min_x;
        let height = cell.max_y - cell.min_y;
        let x = cell.min_x + width / 2.0;
        let y = cell.min_y + height / 2.0;
        (
            Point2 {
                x_m: self.origin.x_m + x * self.axis.x_m - y * self.axis.y_m,
                y_m: self.origin.y_m + x * self.axis.y_m + y * self.axis.x_m,
            },
            width.hypot(height) / 2.0 + self.roundoff,
        )
    }
    fn cover_with(&self, mut is_free: impl FnMut(Point2, f64) -> bool) -> bool {
        // Depth-first longest-edge bisection. A failed disc is only a reason
        // to refine; it may include excess space outside this rectangle.
        // The fixed query budget bounds depth and pending siblings as well.
        let mut stack = [HullCell::default(); MAX_HULL_CELLS + 1];
        stack[0] = self.cell;
        let mut len = 1;
        let mut queries = 0;
        while len > 0 {
            if queries == MAX_HULL_CELLS {
                return false;
            }
            len -= 1;
            let cell = stack[len];
            let (center, radius) = self.disc(cell);
            queries += 1;
            if is_free(center, radius) {
                continue;
            }
            let Some([first, second]) = cell.split() else {
                return false;
            };
            if len + 2 > stack.len() {
                return false;
            }
            stack[len] = second;
            stack[len + 1] = first;
            len += 2;
        }
        true
    }
}

// Archived equal-cell cover is used only to reproduce the earlier excess-disc
// rejection in regressions. Production has only the adaptive area proof.
#[cfg(test)]
struct HullCover {
    bounds: HullBounds,
    cell_x: f64,
    cell_y: f64,
    columns: usize,
    rows: usize,
}
#[cfg(test)]
impl HullCover {
    fn new(points: &[Point2], padding_m: f64) -> Option<Self> {
        let bounds = HullBounds::new(points, padding_m)?;
        let width = bounds.cell.max_x - bounds.cell.min_x;
        let height = bounds.cell.max_y - bounds.cell.min_y;
        let mut shape = (1, MAX_HULL_CELLS);
        let mut best = f64::INFINITY;
        for columns in 1..=MAX_HULL_CELLS {
            let rows = MAX_HULL_CELLS / columns;
            let radius = (width / columns as f64).hypot(height / rows as f64);
            if radius < best {
                best = radius;
                shape = (columns, rows);
            }
        }
        Some(Self {
            bounds,
            cell_x: width / shape.0 as f64,
            cell_y: height / shape.1 as f64,
            columns: shape.0,
            rows: shape.1,
        })
    }
    fn cells(&self) -> usize {
        self.columns * self.rows
    }
    fn disc(&self, index: usize) -> (Point2, f64) {
        let min_x = self.bounds.cell.min_x + (index % self.columns) as f64 * self.cell_x;
        let min_y = self.bounds.cell.min_y + (index / self.columns) as f64 * self.cell_y;
        self.bounds.disc(HullCell {
            min_x,
            max_x: min_x + self.cell_x,
            min_y,
            max_y: min_y + self.cell_y,
        })
    }
}

fn compatible_geometry(a: ElementGeometry, b: ElementGeometry, tolerance: f64) -> bool {
    match (a, b) {
        (ElementGeometry::Cone { radius_m: a }, ElementGeometry::Cone { radius_m: b }) => {
            (a - b).abs() <= tolerance
        }
        (
            ElementGeometry::LineRegion {
                lateral_half_width_m: a,
                depth_m: ad,
            },
            ElementGeometry::LineRegion {
                lateral_half_width_m: b,
                depth_m: bd,
            },
        ) => (a - b).abs() <= tolerance && (ad - bd).abs() <= tolerance,
        _ => false,
    }
}
fn validate_observation(o: &ElementObservation, associated: bool) -> Result<(), ValidationError> {
    if !o.position_body_m.valid()
        || o.position_body_m.x_m.hypot(o.position_body_m.y_m) > 30.0
        || !unit(o.confidence)
        || !between(o.position_error_m, 0.0, 2.0)
        || !between(o.heading_error_rad, 0.0, PI)
        || o.heading_body_rad.is_some_and(|h| !between(h, -PI, PI))
    {
        return Err(error("invalid element observation geometry or confidence"));
    }
    match (o.kind, o.geometry) {
        (ElementKind::Cone, ElementGeometry::Cone { radius_m })
            if between(radius_m, 0.02, 0.5)
                && (!associated || o.source == ObservationSource::VisualLidar) => {}
        (
            ElementKind::Crosswalk | ElementKind::StopLine | ElementKind::FinishMarker,
            ElementGeometry::LineRegion {
                lateral_half_width_m,
                depth_m,
            },
        ) if between(lateral_half_width_m, 0.05, 5.0)
            && between(depth_m, 0.01, 5.0)
            && o.heading_body_rad.is_some()
            && o.source != ObservationSource::VisualLidar => {}
        _ => {
            return Err(error(
                "element lacks associated cone or oriented ground-region geometry",
            ));
        }
    }
    Ok(())
}
fn validate_scan(
    scan: &LidarSample,
    frame: &FrameId,
    extrinsic: Pose2,
) -> Result<(), ValidationError> {
    let n = scan.ranges_m.len();
    if scan.frame_id != *frame
        || !(64..=MAX_LOCAL_RAYS).contains(&n)
        || !extrinsic.valid()
        || extrinsic.point().distance(Point2::default()) > 2.0
        || !scan.angle_min_rad.is_finite()
        || !scan.angle_increment_rad.is_finite()
        || (scan.angle_increment_rad.abs() * n as f64 - TAU).abs() > 1e-6
        || !between(scan.range_min_m, 0.0, 1.0)
        || !between(scan.range_max_m, scan.range_min_m + 0.01, 50.0)
        || scan
            .ranges_m
            .iter()
            .flatten()
            .any(|r| !between(*r, scan.range_min_m, scan.range_max_m))
    {
        return Err(error(
            "invalid bounded full-turn local scan or explicit lidar extrinsic",
        ));
    }
    Ok(())
}
// Preserve the previous representation only for floating-point roundoff from
// body/world transforms and circle fitting. This is not a measurement filter:
// error estimates, timestamps and visual counts still use the accepted sample.
// Cap at one nanometre/radian because world coordinates are only finite-bounded;
// enormous coordinates must not turn a relative-ULP test into a metric deadband.
fn roundoff_window(old: f64, new: f64) -> f64 {
    (32. * f64::EPSILON * old.abs().max(new.abs()).max(1.)).min(1e-9)
}
fn retain_roundoff_scalar(old: f64, new: f64) -> f64 {
    if (new - old).abs() <= roundoff_window(old, new) {
        old
    } else {
        new
    }
}
fn retain_roundoff_position(old: Point2, new: Point2) -> Point2 {
    Point2 {
        x_m: retain_roundoff_scalar(old.x_m, new.x_m),
        y_m: retain_roundoff_scalar(old.y_m, new.y_m),
    }
}
fn retain_roundoff_heading(old: Option<f64>, new: Option<f64>) -> Option<f64> {
    match (old, new) {
        (Some(a), Some(b)) if wrap(b - a).abs() <= roundoff_window(a, b) => old,
        _ => new,
    }
}

fn error(message: &str) -> ValidationError {
    ValidationError(message.into())
}
fn between(x: f64, min: f64, max: f64) -> bool {
    x.is_finite() && x >= min && x <= max
}
fn unit(x: f64) -> bool {
    between(x, 0.0, 1.0)
}
fn wrap(x: f64) -> f64 {
    (x + PI).rem_euclid(TAU) - PI
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct ConeAssociationConfig {
    pub body_frame: FrameId,
    pub lidar_frame: FrameId,
    pub lidar_in_body: Pose2,
    pub min_points: usize,
    pub max_points: usize,
    pub range_error_m: f64,
    pub center_tolerance_m: f64,
    pub minimum_cluster_width_m: f64,
    pub max_point_gap_m: f64,
}
impl ConeAssociationConfig {
    pub fn simulation(body_frame: FrameId, lidar_frame: FrameId, lidar_in_body: Pose2) -> Self {
        Self {
            body_frame,
            lidar_frame,
            lidar_in_body,
            min_points: 3,
            max_points: 128,
            range_error_m: 0.02,
            center_tolerance_m: 0.2,
            minimum_cluster_width_m: 0.035,
            max_point_gap_m: 0.12,
        }
    }
    pub fn validate(&self) -> Result<(), ValidationError> {
        self.body_frame.validate()?;
        self.lidar_frame.validate()?;
        if !self.lidar_in_body.valid()
            || self.lidar_in_body.point().distance(Point2::default()) > 2.0
            || !(3..=128).contains(&self.min_points)
            || !(self.min_points..=128).contains(&self.max_points)
            || !between(self.range_error_m, 0.001, 0.2)
            || !between(self.center_tolerance_m, 0.001, 0.5)
            || !between(self.minimum_cluster_width_m, 0.005, 0.5)
            || !between(self.max_point_gap_m, 0.005, 0.3)
        {
            return Err(error("invalid cone association bounds or extrinsic"));
        }
        Ok(())
    }
}

/// Associate one visual ground-plane cone with a UNIQUE contiguous lidar cluster.
/// No pose or simulator landmarks are used: both inputs must be in calibrated
/// body/lidar frames at exactly the same acquisition time. Unknown/ambiguous
/// clusters drop that cone; malformed data rejects the complete frame.
/// The fitted center's error retains the visual error plus its displacement,
/// rather than claiming the sparse fit has magically improved uncertainty.
pub fn associate_visual_cones(
    frame: &ElementFrame,
    scan: &LidarSample,
    config: &ConeAssociationConfig,
) -> Result<ElementFrame, ValidationError> {
    config.validate()?;
    validate_scan(scan, &config.lidar_frame, config.lidar_in_body)?;
    if frame.frame_id != config.body_frame
        || frame.captured_at != scan.captured_at
        || frame.observations.len() > MAX_ELEMENTS
    {
        return Err(error(
            "cone association requires bounded same-time body/lidar observations",
        ));
    }
    let mut output = ElementFrame {
        captured_at: frame.captured_at,
        frame_id: frame.frame_id.clone(),
        observations: Vec::with_capacity(frame.observations.len()),
    };
    for o in &frame.observations {
        validate_observation(o, false)?;
        let ElementGeometry::Cone { .. } = o.geometry else {
            output.observations.push(*o);
            continue;
        };
        if o.source != ObservationSource::GroundProjection {
            return Err(error(
                "cone association requires an original visual ground projection",
            ));
        }
        if let Some(fit) = unique_cone_fit(
            o,
            &scan.ranges_m,
            scan.angle_min_rad,
            scan.angle_increment_rad,
            config,
        ) {
            let mut observation = *o;
            observation.position_body_m = fit.center;
            observation.position_error_m +=
                fit.center.distance(o.position_body_m) + config.range_error_m;
            observation.source = ObservationSource::VisualLidar;
            output.observations.push(observation);
        }
    }
    Ok(output)
}

#[derive(Clone, Copy)]
struct ConeFit {
    center: Point2,
    first_beam: usize,
    beam_count: usize,
    center_error_m: f64,
}

fn unique_cone_fit(
    o: &ElementObservation,
    ranges: &[Option<f64>],
    angle_min: f64,
    increment: f64,
    config: &ConeAssociationConfig,
) -> Option<ConeFit> {
    let ElementGeometry::Cone { radius_m } = o.geometry else {
        return None;
    };
    let laser_center = config.lidar_in_body.world_to_body(o.position_body_m);
    let bearing = laser_center.y_m.atan2(laser_center.x_m);
    let n = ranges.len();
    // Start opposite the candidate bearing, so a cluster crossing index zero
    // remains one cluster. Missing samples and jumps always break continuity.
    let middle = ((bearing - angle_min) / increment).round() as i64;
    let start = (middle + n as i64 / 2).rem_euclid(n as i64) as usize;
    let mut points = [Point2::default(); 128];
    let mut count = 0usize;
    let mut first_beam = 0usize;
    let mut overflow = false;
    let mut candidates = 0usize;
    let mut result = None;
    for offset in 0..=n {
        let index = (start + offset) % n;
        let point = if offset < n {
            ranges[index]
                .map(|range| {
                    let a = angle_min + index as f64 * increment;
                    config.lidar_in_body.body_to_world(Point2 {
                        x_m: range * a.cos(),
                        y_m: range * a.sin(),
                    })
                })
                .filter(|p| {
                    (p.distance(o.position_body_m) - radius_m).abs()
                        <= o.position_error_m + config.center_tolerance_m + config.range_error_m
                })
        } else {
            None
        };
        let gap = point.is_some_and(|p| {
            count > 0 && points[count.min(128) - 1].distance(p) > config.max_point_gap_m
        });
        if point.is_none() || gap {
            if !overflow
                && count >= config.min_points
                && let Some(center) = fitted_cone_center(&points[..count], o, config)
            {
                candidates += 1;
                result = Some(ConeFit {
                    center,
                    first_beam,
                    beam_count: count,
                    center_error_m: fitted_center_error(
                        &points[..count],
                        center,
                        radius_m,
                        config.range_error_m,
                    ),
                });
            }
            count = 0;
            overflow = false;
        }
        if let Some(point) = point {
            if count == 0 {
                first_beam = index;
            }
            if count < config.max_points {
                points[count] = point;
                count += 1;
            } else {
                overflow = true;
            }
        }
    }
    if candidates == 1 { result } else { None }
}

/// Circle-from-chord sensitivity: perturb both endpoints by range error plus
/// observed radial residual. Short arcs can have a very uncertain normal; near
/// diameter chords have uncertain center height. Neither is reported as .02 m
/// merely because the laser's range bound is .02 m. The radius is the explicit
/// prior already accepted from vision, not a new semantic inference from lidar.
fn fitted_center_error(points: &[Point2], center: Point2, radius: f64, range_error: f64) -> f64 {
    let width = points[0].distance(points[points.len() - 1]);
    let residual = points
        .iter()
        .map(|p| (p.distance(center) - radius).abs())
        .fold(0., f64::max);
    let endpoint_error = range_error + residual;
    let minimum_width = (width - 2. * endpoint_error).max(0.);
    let maximum_width = (width + 2. * endpoint_error).min(2. * radius);
    let height = (radius * radius - width * width / 4.).max(0.).sqrt();
    let maximum_height = (radius * radius - minimum_width * minimum_width / 4.)
        .max(0.)
        .sqrt();
    let minimum_height = (radius * radius - maximum_width * maximum_width / 4.)
        .max(0.)
        .sqrt();
    let normal_error = if minimum_width > 0. {
        (4. * endpoint_error / minimum_width).min(2.)
    } else {
        2.
    };
    endpoint_error
        + (height - minimum_height).max(maximum_height - height)
        + maximum_height * normal_error
}

fn shared_beams(a: ConeFit, b: ConeFit, n: usize) -> bool {
    (0..a.beam_count).any(|offset| ((a.first_beam + offset + n - b.first_beam) % n) < b.beam_count)
}

fn fitted_cone_center(
    points: &[Point2],
    observation: &ElementObservation,
    config: &ConeAssociationConfig,
) -> Option<Point2> {
    let ElementGeometry::Cone { radius_m } = observation.geometry else {
        return None;
    };
    let first = *points.first()?;
    let last = *points.last()?;
    let width = first.distance(last);
    if width < config.minimum_cluster_width_m || width > 2.0 * radius_m {
        return None;
    }
    let middle = Point2 {
        x_m: (first.x_m + last.x_m) * 0.5,
        y_m: (first.y_m + last.y_m) * 0.5,
    };
    let height = (radius_m * radius_m - width * width * 0.25).sqrt();
    let normal = Point2 {
        x_m: -(last.y_m - first.y_m) / width,
        y_m: (last.x_m - first.x_m) / width,
    };
    let a = Point2 {
        x_m: middle.x_m + normal.x_m * height,
        y_m: middle.y_m + normal.y_m * height,
    };
    let b = Point2 {
        x_m: middle.x_m - normal.x_m * height,
        y_m: middle.y_m - normal.y_m * height,
    };
    let da = a.distance(observation.position_body_m);
    let db = b.distance(observation.position_body_m);
    // Equal alternatives are ambiguous; do not choose a circle by array order.
    if (da - db).abs() <= config.range_error_m {
        return None;
    }
    let center = if da < db { a } else { b };
    if center.distance(observation.position_body_m)
        > config.center_tolerance_m + observation.position_error_m
        || points
            .iter()
            .any(|p| (p.distance(center) - radius_m).abs() > config.range_error_m)
    {
        return None;
    }
    Some(center)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn config() -> LocalWorldConfig {
        LocalWorldConfig::simulation(
            FrameId("body".into()),
            FrameId("laser".into()),
            FrameId("map".into()),
        )
    }
    fn pose(at: u64) -> PoseEstimate {
        PoseEstimate {
            captured_at: Timestamp(at),
            frame_id: FrameId("map".into()),
            pose: Pose2::default(),
            speed_mps: 0.0,
            yaw_rate_radps: 0.0,
            quality: 1.0,
        }
    }
    fn cone(x: f64) -> ElementObservation {
        ElementObservation {
            kind: ElementKind::Cone,
            color: ElementColor::Red,
            position_body_m: Point2 { x_m: x, y_m: 0.0 },
            heading_body_rad: None,
            geometry: ElementGeometry::Cone { radius_m: 0.2 },
            source: ObservationSource::VisualLidar,
            confidence: 0.9,
            position_error_m: 0.03,
            heading_error_rad: 0.0,
        }
    }
    fn frame(at: u64, observations: Vec<ElementObservation>) -> ElementFrame {
        ElementFrame {
            captured_at: Timestamp(at),
            frame_id: FrameId("body".into()),
            observations,
        }
    }
    fn scan(at: u64) -> LidarSample {
        LidarSample {
            captured_at: Timestamp(at),
            frame_id: FrameId("laser".into()),
            angle_min_rad: 0.0,
            angle_increment_rad: TAU / 720.0,
            range_min_m: 0.05,
            range_max_m: 20.0,
            ranges_m: vec![Some(5.0); 720],
        }
    }
    #[test]
    fn tracks_confirm_only_distinct_frames_retain_occlusion_then_expire() {
        let mut world = LocalWorld::new(config()).unwrap();
        let first = frame(100, vec![cone(2.0)]);
        world.update(Timestamp(100), &pose(100), &first).unwrap();
        world.update(Timestamp(150), &pose(100), &first).unwrap();
        assert_eq!(world.tracks(Timestamp(150)).count(), 0);
        world
            .update(Timestamp(200), &pose(200), &frame(200, vec![cone(2.0)]))
            .unwrap();
        let track = world.tracks(Timestamp(200)).next().unwrap();
        world
            .update(Timestamp(300), &pose(300), &frame(300, vec![]))
            .unwrap();
        let remembered = world.tracks(Timestamp(1000)).next().unwrap();
        assert_eq!(remembered.id, track.id);
        assert!(remembered.position_error_m > track.position_error_m);
        assert_eq!(world.tracks(Timestamp(2000)).count(), 0);
    }
    #[test]
    fn identity_survives_body_motion_and_processed_tombstone_blocks_reuse() {
        let mut world = LocalWorld::new(config()).unwrap();
        world
            .update(Timestamp(100), &pose(100), &frame(100, vec![cone(2.0)]))
            .unwrap();
        let mut moved = pose(200);
        moved.pose.x_m = 0.5;
        world
            .update(Timestamp(200), &moved, &frame(200, vec![cone(1.5)]))
            .unwrap();
        let id = world.tracks(Timestamp(200)).next().unwrap().id;
        world.mark_processed(id).unwrap();
        world
            .update(Timestamp(3000), &pose(3000), &frame(3000, vec![cone(2.0)]))
            .unwrap();
        assert_eq!(world.elements.iter().flatten().count(), 1);
        assert!(world.elements.iter().flatten().next().unwrap().processed);
        assert_eq!(world.tracks(Timestamp(3000)).count(), 0);
    }
    #[test]
    fn contradiction_and_ambiguous_same_frame_quarantine_identity() {
        let mut world = LocalWorld::new(config()).unwrap();
        for at in [100, 200] {
            world
                .update(Timestamp(at), &pose(at), &frame(at, vec![cone(2.0)]))
                .unwrap();
        }
        let mut blue = cone(2.01);
        blue.color = ElementColor::Blue;
        world
            .update(Timestamp(300), &pose(300), &frame(300, vec![blue]))
            .unwrap();
        assert_eq!(world.tracks(Timestamp(300)).count(), 0);
        world
            .update(Timestamp(400), &pose(400), &frame(400, vec![cone(2.0)]))
            .unwrap();
        assert_eq!(world.tracks(Timestamp(400)).count(), 0);
        let mut fresh = LocalWorld::new(config()).unwrap();
        fresh
            .update(
                Timestamp(100),
                &pose(100),
                &frame(100, vec![cone(2.0), cone(2.01)]),
            )
            .unwrap();
        fresh
            .update(Timestamp(200), &pose(200), &frame(200, vec![cone(2.0)]))
            .unwrap();
        assert_eq!(fresh.tracks(Timestamp(200)).count(), 0);
    }
    #[test]
    fn malformed_geometry_time_and_capacity_do_not_modify_tracks() {
        let mut world = LocalWorld::new(config()).unwrap();
        let mut o = cone(2.0);
        o.source = ObservationSource::GroundProjection;
        assert!(
            world
                .update(Timestamp(100), &pose(100), &frame(100, vec![o]))
                .is_err()
        );
        assert!(
            world
                .update(Timestamp(100), &pose(101), &frame(100, vec![]))
                .is_err()
        );
        assert!(
            world
                .update(
                    Timestamp(100),
                    &pose(100),
                    &frame(100, vec![cone(2.0); MAX_ELEMENTS + 1])
                )
                .is_err()
        );
        o = cone(2.0);
        o.kind = ElementKind::StopLine;
        o.geometry = ElementGeometry::LineRegion {
            lateral_half_width_m: 0.5,
            depth_m: 0.8,
        };
        o.source = ObservationSource::GroundMarker;
        assert!(
            world
                .update(Timestamp(100), &pose(100), &frame(100, vec![o]))
                .is_err()
        );
        assert!(world.elements.iter().all(Option::is_none));
        let first = frame(100, vec![cone(2.0)]);
        world.update(Timestamp(100), &pose(100), &first).unwrap();
        assert!(
            world
                .update(Timestamp(150), &pose(100), &frame(100, vec![cone(2.1)]))
                .is_err()
        );
    }
    #[test]
    fn complete_capsule_and_hull_reject_unknown_holes_and_internal_obstacles() {
        let mut world = LocalWorld::new(config()).unwrap();
        let mut s = scan(100);
        world
            .update_scan(Timestamp(100), &pose(100), &s, Pose2::default())
            .unwrap();
        let a = Point2 {
            x_m: 1.0,
            y_m: -0.5,
        };
        let b = Point2 { x_m: 1.0, y_m: 0.5 };
        assert!(world.known_free_segment(Timestamp(100), a, b, 0.05));
        assert!(world.known_free_convex_hull(Timestamp(100), &[a, b], 0.05));
        s.captured_at = Timestamp(200);
        s.ranges_m[0] = Some(1.0);
        world
            .update_scan(Timestamp(200), &pose(200), &s, Pose2::default())
            .unwrap();
        assert_eq!(
            world.space_state(Timestamp(200), Point2 { x_m: 1.0, y_m: 0.0 }, 0.05),
            SpaceState::Obstacle
        );
        assert!(!world.known_free_segment(Timestamp(200), a, b, 0.05));
        assert!(!world.known_free_convex_hull(Timestamp(200), &[a, b], 0.05));
        s.captured_at = Timestamp(300);
        s.ranges_m[0] = None;
        world
            .update_scan(Timestamp(300), &pose(300), &s, Pose2::default())
            .unwrap();
        assert_eq!(
            world.space_state(Timestamp(300), Point2 { x_m: 1.0, y_m: 0.0 }, 0.05),
            SpaceState::Unknown
        );
        assert!(!world.known_free_segment(Timestamp(300), a, b, 0.05));
        assert!(!world.known_free_segment(Timestamp(600), a, b, 0.05));
    }
    #[test]
    fn local_scan_rejects_changed_duplicate_missing_pose_and_unknown_full_scan() {
        let mut world = LocalWorld::new(config()).unwrap();
        let mut s = scan(100);
        assert!(
            world
                .update_scan(Timestamp(100), &pose(101), &s, Pose2::default())
                .is_err()
        );
        world
            .update_scan(Timestamp(100), &pose(100), &s, Pose2::default())
            .unwrap();
        s.ranges_m[0] = None;
        assert!(
            world
                .update_scan(Timestamp(100), &pose(100), &s, Pose2::default())
                .is_err()
        );
        s.captured_at = Timestamp(200);
        s.ranges_m.fill(None);
        world
            .update_scan(Timestamp(200), &pose(200), &s, Pose2::default())
            .unwrap();
        assert_eq!(
            world.space_state(Timestamp(200), Point2::default(), 0.1),
            SpaceState::Unknown
        );
    }
    fn cone_scan(at: u64, center: Point2, radius: f64, extrinsic: Pose2) -> LidarSample {
        let mut s = scan(at);
        let c = extrinsic.world_to_body(center);
        for (i, r) in s.ranges_m.iter_mut().enumerate() {
            let a = i as f64 * s.angle_increment_rad;
            let projection = c.x_m * a.cos() + c.y_m * a.sin();
            let cross = c.x_m * a.sin() - c.y_m * a.cos();
            if projection > 0.0 && cross.abs() <= radius {
                *r = Some(projection - (radius * radius - cross * cross).sqrt());
            }
        }
        s
    }
    #[test]
    fn lidar_association_uses_contiguous_shape_same_time_and_explicit_extrinsic() {
        let extrinsic = Pose2 {
            x_m: 0.15,
            y_m: 0.07,
            yaw_rad: 0.1,
        };
        let cfg = ConeAssociationConfig::simulation(
            FrameId("body".into()),
            FrameId("laser".into()),
            extrinsic,
        );
        let mut visual = cone(2.0);
        visual.source = ObservationSource::GroundProjection;
        let f = frame(100, vec![visual]);
        let mut s = cone_scan(100, visual.position_body_m, 0.2, extrinsic);
        let fused = associate_visual_cones(&f, &s, &cfg).unwrap();
        assert_eq!(fused.observations.len(), 1);
        assert!(
            fused.observations[0]
                .position_body_m
                .distance(visual.position_body_m)
                < 1e-9
        );
        assert_eq!(fused.observations[0].source, ObservationSource::VisualLidar);
        s.captured_at = Timestamp(101);
        assert!(associate_visual_cones(&f, &s, &cfg).is_err());
        s.captured_at = Timestamp(100);
        s.ranges_m.fill(None);
        s.ranges_m[0] = Some(1.8);
        assert!(
            associate_visual_cones(&f, &s, &cfg)
                .unwrap()
                .observations
                .is_empty()
        );
        let mut wrong = cfg;
        wrong.lidar_in_body.y_m = 1.0;
        let s = cone_scan(100, visual.position_body_m, 0.2, extrinsic);
        assert!(
            associate_visual_cones(&f, &s, &wrong)
                .unwrap()
                .observations
                .is_empty()
        );
    }
    #[test]
    fn uncertainty_limits_expire_before_ttl_and_config_nonfinite_rejected() {
        let mut cfg = config();
        cfg.position_drift_mps = 1.0;
        let mut world = LocalWorld::new(cfg).unwrap();
        for at in [100, 200] {
            world
                .update(Timestamp(at), &pose(at), &frame(at, vec![cone(2.0)]))
                .unwrap();
        }
        assert_eq!(world.tracks(Timestamp(200)).count(), 1);
        assert_eq!(world.tracks(Timestamp(500)).count(), 0);
        let mut cfg = config();
        cfg.laser_error_m = f64::NAN;
        assert!(LocalWorld::new(cfg).is_err());
    }
    #[test]
    fn near_range_blind_disc_requires_explicit_same_pose_body_prior() {
        let mut world = LocalWorld::new(config()).unwrap();
        let s = scan(100);
        world
            .update_scan(Timestamp(100), &pose(100), &s, Pose2::default())
            .unwrap();
        assert_eq!(
            world.space_state(Timestamp(100), Point2::default(), 0.1),
            SpaceState::Unknown
        );
        let mut cfg = config();
        cfg.self_footprint = Some(Footprint {
            front_m: 0.26,
            rear_m: 0.22,
            half_width_m: 0.14,
        });
        let mut world = LocalWorld::new(cfg).unwrap();
        let mut p = pose(100);
        p.pose.x_m = 3.0;
        p.pose.y_m = 2.0;
        p.pose.yaw_rad = 0.7;
        world
            .update_scan(Timestamp(100), &p, &s, Pose2::default())
            .unwrap();
        assert_eq!(
            world.space_state(Timestamp(100), p.pose.point(), 0.1),
            SpaceState::KnownFree
        );
        let offset = Pose2 {
            x_m: 0.0,
            y_m: 0.2,
            yaw_rad: 0.0,
        };
        let mut s = s;
        s.captured_at = Timestamp(200);
        p.captured_at = Timestamp(200);
        world.update_scan(Timestamp(200), &p, &s, offset).unwrap();
        assert_eq!(
            world.space_state(Timestamp(200), p.pose.body_to_world(offset.point()), 0.1),
            SpaceState::Unknown
        );
    }

    #[test]
    fn split_lidar_cluster_is_ambiguous_and_processed_storage_never_grows() {
        let cfg = ConeAssociationConfig::simulation(
            FrameId("body".into()),
            FrameId("laser".into()),
            Pose2::default(),
        );
        let mut visual = cone(2.0);
        visual.source = ObservationSource::GroundProjection;
        let mut s = cone_scan(100, visual.position_body_m, 0.2, Pose2::default());
        s.ranges_m[0] = None;
        assert!(
            associate_visual_cones(&frame(100, vec![visual]), &s, &cfg)
                .unwrap()
                .observations
                .is_empty()
        );
        let mut world = LocalWorld::new(config()).unwrap();
        for at in [100, 200] {
            let elements = (0..MAX_ELEMENTS).map(|i| cone(i as f64 + 0.5)).collect();
            world
                .update(Timestamp(at), &pose(at), &frame(at, elements))
                .unwrap();
        }
        let ids: Vec<_> = world.tracks(Timestamp(200)).map(|t| t.id).collect();
        assert_eq!(ids.len(), MAX_ELEMENTS);
        for id in ids {
            world.mark_processed(id).unwrap();
        }
        assert!(
            world
                .update(Timestamp(5000), &pose(5000), &frame(5000, vec![cone(25.0)]))
                .is_err()
        );
        assert_eq!(world.elements.iter().flatten().count(), MAX_ELEMENTS);
    }
    #[test]
    fn far_range_discontinuity_keeps_only_near_certified_free_sector() {
        let mut cfg = config();
        cfg.self_footprint = Some(Footprint {
            front_m: 0.22,
            rear_m: 0.18,
            half_width_m: 0.13,
        });
        let mut world = LocalWorld::new(cfg).unwrap();
        let s = cone_scan(100, Point2 { x_m: 2.0, y_m: 0.0 }, 0.2, Pose2::default());
        assert!(
            s.ranges_m
                .windows(2)
                .any(|pair| (pair[0].unwrap() - pair[1].unwrap()).abs() > 0.3)
        );
        world
            .update_scan(Timestamp(100), &pose(100), &s, Pose2::default())
            .unwrap();
        assert_eq!(
            world.space_state(Timestamp(100), Point2::default(), 0.3),
            SpaceState::KnownFree
        );
        assert!(world.known_free_segment(
            Timestamp(100),
            Point2::default(),
            Point2 { x_m: 0.8, y_m: 0.0 },
            0.3
        ));
        // The shorter hit bounds free space. Its hidden far side is NOT free.
        assert_eq!(
            world.space_state(Timestamp(100), Point2 { x_m: 2.5, y_m: 0.0 }, 0.02),
            SpaceState::Unknown
        );
        let mut blind = s;
        blind.captured_at = Timestamp(200);
        blind.ranges_m[0] = None;
        world
            .update_scan(Timestamp(200), &pose(200), &blind, Pose2::default())
            .unwrap();
        assert_eq!(
            world.space_state(Timestamp(200), Point2::default(), 0.3),
            SpaceState::Unknown
        );
        assert!(!world.known_free_segment(
            Timestamp(500),
            Point2::default(),
            Point2 { x_m: 0.8, y_m: 0.0 },
            0.3
        ));
    }

    #[test]
    fn complete_forward_capsule_near_rear_wall_uses_per_sector_radial_extent() {
        let mut cfg = config();
        cfg.self_footprint = Some(Footprint {
            front_m: 0.22,
            rear_m: 0.18,
            half_width_m: 0.13,
        });
        let mut world = LocalWorld::new(cfg).unwrap();
        let mut s = cone_scan(100, Point2 { x_m: 2.0, y_m: 0.3 }, 0.2, Pose2::default());
        // Source is .5m from the rear wall and .6m from the lower wall,
        // matching the initial-nominal trace; every ray measures the nearer hit.
        for (i, range) in s.ranges_m.iter_mut().enumerate() {
            let angle = i as f64 * s.angle_increment_rad;
            for (boundary, direction) in [(-0.5, angle.cos()), (-0.6, angle.sin())] {
                let hit = boundary / direction;
                if hit.is_finite() && hit > 0.0 {
                    *range = Some(range.unwrap().min(hit));
                }
            }
        }
        world
            .update_scan(Timestamp(100), &pose(100), &s, Pose2::default())
            .unwrap();
        assert!(world.known_free_segment(
            Timestamp(100),
            Point2::default(),
            Point2 { x_m: 0.8, y_m: 0.0 },
            0.3
        ));
        assert!(!world.known_free_segment(
            Timestamp(100),
            Point2::default(),
            Point2 {
                x_m: -0.4,
                y_m: 0.0
            },
            0.3
        ));
        // An obstacle in the interior is still rejected, even when both
        // endpoint centers themselves have no direct obstacle return.
        assert!(!world.known_free_segment(
            Timestamp(100),
            Point2 { x_m: 1.0, y_m: 0.3 },
            Point2 { x_m: 3.0, y_m: 0.3 },
            0.05
        ));
    }
    fn confirmed_world(observation: ElementObservation) -> LocalWorld {
        let mut world = LocalWorld::new(config()).unwrap();
        for at in [0, 100] {
            world
                .update(Timestamp(at), &pose(at), &frame(at, vec![observation]))
                .unwrap();
        }
        world
    }
    fn maintenance_config(extrinsic: Pose2) -> ConeAssociationConfig {
        ConeAssociationConfig::simulation(
            FrameId("body".into()),
            FrameId("laser".into()),
            extrinsic,
        )
    }
    fn maintain_with_scan(
        world: &mut LocalWorld,
        p: &PoseEstimate,
        s: &LidarSample,
        cfg: &ConeAssociationConfig,
    ) -> usize {
        world
            .update_scan(s.captured_at, p, s, cfg.lidar_in_body)
            .unwrap();
        world
            .update(s.captured_at, p, &frame(s.captured_at.0, vec![]))
            .unwrap();
        world.maintain_confirmed_cones(s.captured_at, cfg).unwrap()
    }
    #[test]
    fn lidar_maintenance_keeps_side_identity_without_visual_counts_or_error_accumulation() {
        let mut initial = cone(2.);
        initial.position_error_m = 0.18;
        let mut world = confirmed_world(initial);
        let old = world.tracks(Timestamp(100)).next().unwrap();
        let extrinsic = Pose2 {
            x_m: 0.15,
            y_m: 0.07,
            yaw_rad: 0.1,
        };
        let cfg = maintenance_config(extrinsic);
        // A complete bounded half-orbit with the cone always at the body's
        // side, outside a forward camera. Every ray is generated from current
        // pose and a nonzero laser extrinsic, with no further visual frames.
        for at in (200..=20_000).step_by(100) {
            let theta = PI + PI * (at - 200) as f64 / 19_800.;
            let mut p = pose(at);
            p.pose = Pose2 {
                x_m: 2. + 0.85 * theta.cos(),
                y_m: 0.85 * theta.sin(),
                yaw_rad: wrap(theta + PI / 2.),
            };
            let center = p.pose.world_to_body(old.position);
            assert!(center.x_m.abs() < 1e-12);
            let s = cone_scan(at, center, 0.2, extrinsic);
            assert_eq!(maintain_with_scan(&mut world, &p, &s, &cfg), 1, "at={at}");
            let track = world.tracks(Timestamp(at)).next().unwrap();
            assert_eq!(
                (track.id, track.color, track.observations),
                (old.id, old.color, 2)
            );
            assert_eq!(track.last_visual_at, Timestamp(100));
            assert_eq!(track.last_geometry_at, Timestamp(at));
            assert_eq!(track.last_seen, track.last_geometry_at);
            assert!(track.position.distance(old.position) < 1e-9);
            assert!(track.position_error_m >= old.position_error_m);
            assert!(track.position_error_m <= world.config.max_position_error_m);
            // Independent current fits do not add previous range error again.
            assert!((track.position_error_m - old.position_error_m).abs() < 1e-9);
        }
        assert_eq!(world.tracks(Timestamp(20_099)).count(), 1);
        // Semantic expiry is exact even with healthy fresh range geometry.
        assert_eq!(world.tracks(Timestamp(20_100)).count(), 0);
        let p = pose(20_100);
        let s = cone_scan(20_100, old.position, 0.2, extrinsic);
        assert_eq!(maintain_with_scan(&mut world, &p, &s, &cfg), 0);
        let stored = world.elements.iter().flatten().next().unwrap();
        assert_eq!(stored.last_geometry_at, Timestamp(20_000));
        assert_eq!(stored.last_visual_at, Timestamp(100));
    }
    #[test]
    fn lidar_maintenance_never_creates_confirms_recolors_or_revives_processed_and_invalid_tracks() {
        let cfg = maintenance_config(Pose2::default());
        let mut empty = LocalWorld::new(config()).unwrap();
        let s = cone_scan(200, Point2 { x_m: 2., y_m: 0. }, 0.2, cfg.lidar_in_body);
        assert_eq!(maintain_with_scan(&mut empty, &pose(200), &s, &cfg), 0);
        assert!(empty.elements.iter().all(Option::is_none));
        let mut unconfirmed = LocalWorld::new(config()).unwrap();
        unconfirmed
            .update(Timestamp(100), &pose(100), &frame(100, vec![cone(2.)]))
            .unwrap();
        assert_eq!(
            maintain_with_scan(&mut unconfirmed, &pose(200), &s, &cfg),
            0
        );
        assert_eq!(unconfirmed.elements[0].unwrap().observations, 1);
        assert_eq!(
            unconfirmed.elements[0].unwrap().last_geometry_at,
            Timestamp(100)
        );
        let mut processed = confirmed_world(cone(2.));
        let id = processed.elements[0].unwrap().id;
        processed.mark_processed(id).unwrap();
        assert_eq!(maintain_with_scan(&mut processed, &pose(200), &s, &cfg), 0);
        assert_eq!(
            processed.elements[0].unwrap().last_geometry_at,
            Timestamp(100)
        );
        let mut invalid = confirmed_world(cone(2.));
        let mut blue = cone(2.);
        blue.color = ElementColor::Blue;
        invalid
            .update(Timestamp(150), &pose(150), &frame(150, vec![blue]))
            .unwrap();
        assert_eq!(maintain_with_scan(&mut invalid, &pose(200), &s, &cfg), 0);
        let track = invalid.elements[0].unwrap();
        assert!(!track.valid);
        assert_eq!(track.color, ElementColor::Red);
        assert_eq!(track.observations, 2);
        assert_eq!(track.last_visual_at, Timestamp(100));
    }
    #[test]
    fn lidar_maintenance_rejects_missing_split_displaced_wrong_shape_and_short_arc() {
        let cfg = maintenance_config(Pose2::default());
        for variant in 0..5 {
            let mut world = confirmed_world(cone(2.));
            let mut s = cone_scan(200, Point2 { x_m: 2., y_m: 0. }, 0.2, cfg.lidar_in_body);
            match variant {
                0 => s.ranges_m.fill(None),
                1 => s.ranges_m[0] = None,
                2 => s = cone_scan(200, Point2 { x_m: 2.9, y_m: 0. }, 0.2, cfg.lidar_in_body),
                3 => s = cone_scan(200, Point2 { x_m: 2., y_m: 0. }, 0.45, cfg.lidar_in_body),
                4 => {
                    for i in 0..720 {
                        if !(i <= 2 || i >= 718) {
                            s.ranges_m[i] = None;
                        }
                    }
                }
                _ => unreachable!(),
            }
            assert_eq!(
                maintain_with_scan(&mut world, &pose(200), &s, &cfg),
                0,
                "variant={variant}"
            );
            assert_eq!(world.elements[0].unwrap().last_geometry_at, Timestamp(100));
            // Failed fits never move the original 1800 ms geometry deadline.
            assert_eq!(world.tracks(Timestamp(1899)).count(), 1);
            assert_eq!(world.tracks(Timestamp(1900)).count(), 0);
            let restored = cone_scan(1900, Point2 { x_m: 2., y_m: 0. }, 0.2, cfg.lidar_in_body);
            assert_eq!(
                maintain_with_scan(&mut world, &pose(1900), &restored, &cfg),
                0
            );
            assert_eq!(world.elements[0].unwrap().last_geometry_at, Timestamp(100));
        }
    }
    #[test]
    fn lidar_maintenance_rejects_two_identities_claiming_one_cluster() {
        let mut local = config();
        local.association_distance_m = 0.01;
        local.pose_position_error_m = 0.001;
        local.pose_heading_error_rad = 0.0001;
        let mut world = LocalWorld::new(local).unwrap();
        let observations = [1.94, 2.06].map(|x| {
            let mut o = cone(x);
            o.position_error_m = 0.005;
            o
        });
        for at in [0, 100] {
            world
                .update(Timestamp(at), &pose(at), &frame(at, observations.to_vec()))
                .unwrap();
        }
        assert_eq!(world.tracks(Timestamp(100)).count(), 2);
        let cfg = maintenance_config(Pose2::default());
        let s = cone_scan(200, Point2 { x_m: 2., y_m: 0. }, 0.2, cfg.lidar_in_body);
        assert_eq!(maintain_with_scan(&mut world, &pose(200), &s, &cfg), 0);
        for track in world.elements.iter().flatten() {
            assert_eq!(track.last_geometry_at, Timestamp(100));
            assert_eq!(track.observations, 2);
        }
    }
    #[test]
    fn lidar_maintenance_duplicate_and_failed_metadata_are_transactional() {
        let mut world = confirmed_world(cone(2.));
        let cfg = maintenance_config(Pose2::default());
        let s = cone_scan(200, Point2 { x_m: 2., y_m: 0. }, 0.2, cfg.lidar_in_body);
        assert_eq!(maintain_with_scan(&mut world, &pose(200), &s, &cfg), 1);
        let accepted = world.elements;
        assert_eq!(
            world
                .maintain_confirmed_cones(Timestamp(210), &cfg)
                .unwrap(),
            0
        );
        assert_eq!(world.elements, accepted);
        let mut altered = cfg.clone();
        altered.center_tolerance_m += 0.01;
        assert!(
            world
                .maintain_confirmed_cones(Timestamp(210), &altered)
                .is_err()
        );
        assert_eq!(world.elements, accepted);
        let mut altered_scan = s.clone();
        altered_scan.ranges_m[0] = None;
        assert!(
            world
                .update_scan(Timestamp(210), &pose(200), &altered_scan, cfg.lidar_in_body)
                .is_err()
        );
        let mut altered_pose = pose(200);
        altered_pose.pose.x_m = 0.01;
        assert!(
            world
                .update_scan(Timestamp(210), &altered_pose, &s, cfg.lidar_in_body)
                .is_err()
        );
        let s = cone_scan(300, Point2 { x_m: 2., y_m: 0. }, 0.2, cfg.lidar_in_body);
        world
            .update_scan(Timestamp(300), &pose(300), &s, cfg.lidar_in_body)
            .unwrap();
        let mut underestimated = cfg.clone();
        underestimated.range_error_m = 0.01;
        assert!(
            world
                .maintain_confirmed_cones(Timestamp(300), &underestimated)
                .is_err()
        );
        assert_eq!(world.elements, accepted);
        let mut wrong_extrinsic = cfg.clone();
        wrong_extrinsic.lidar_in_body.x_m = 0.1;
        assert!(
            world
                .maintain_confirmed_cones(Timestamp(300), &wrong_extrinsic)
                .is_err()
        );
        assert_eq!(world.elements, accepted);
        assert_eq!(
            world
                .maintain_confirmed_cones(Timestamp(300), &cfg)
                .unwrap(),
            1
        );
        let accepted = world.elements;
        // The same capture timestamp must also mean the same pose values.
        let mut incompatible_pose = pose(300);
        incompatible_pose.pose.x_m = 0.1;
        world
            .update(Timestamp(300), &incompatible_pose, &frame(300, vec![]))
            .unwrap();
        assert!(
            world
                .maintain_confirmed_cones(Timestamp(300), &cfg)
                .is_err()
        );
        assert_eq!(world.elements, accepted);
        world
            .update(Timestamp(400), &pose(400), &frame(400, vec![]))
            .unwrap();
        assert!(
            world
                .maintain_confirmed_cones(Timestamp(400), &cfg)
                .is_err()
        );
        assert_eq!(world.elements, accepted);
    }
    #[test]
    fn lidar_maintenance_error_during_staging_leaves_all_tracks_and_stamp_unchanged() {
        let mut world = LocalWorld::new(config()).unwrap();
        let mut side = cone(0.);
        side.position_body_m.y_m = 2.;
        for at in [0, 100] {
            world
                .update(Timestamp(at), &pose(at), &frame(at, vec![cone(2.), side]))
                .unwrap();
        }
        let cfg = maintenance_config(Pose2::default());
        let mut s = cone_scan(200, Point2 { x_m: 2., y_m: 0. }, 0.2, cfg.lidar_in_body);
        let side_scan = cone_scan(200, Point2 { x_m: 0., y_m: 2. }, 0.2, cfg.lidar_in_body);
        for (a, b) in s.ranges_m.iter_mut().zip(side_scan.ranges_m) {
            *a = Some(a.unwrap().min(b.unwrap()));
        }
        world
            .update_scan(Timestamp(200), &pose(200), &s, cfg.lidar_in_body)
            .unwrap();
        let elements = world.elements;
        let floor = world.visual_error_floor[1].take();
        assert!(
            world
                .maintain_confirmed_cones(Timestamp(200), &cfg)
                .is_err()
        );
        assert_eq!(world.elements, elements);
        assert!(world.last_maintenance.is_none());
        world.visual_error_floor[1] = floor;
        assert_eq!(
            world
                .maintain_confirmed_cones(Timestamp(200), &cfg)
                .unwrap(),
            2
        );
    }
    #[test]
    fn cone_semantic_lease_is_explicit_finite_and_cannot_undercut_geometry_ttl() {
        for ttl in [0, 1799, 60_001, u64::MAX] {
            let mut cfg = config();
            cfg.cone_semantic_ttl_ms = ttl;
            assert!(cfg.validate().is_err());
        }
        let cfg = config();
        assert_eq!(cfg.track_ttl_ms, 1800);
        assert_eq!(cfg.cone_semantic_ttl_ms, 20_000);
        assert!(cfg.validate().is_ok());
        let mut json = serde_json::to_value(cfg).unwrap();
        json.as_object_mut().unwrap().remove("cone_semantic_ttl_ms");
        assert!(serde_json::from_value::<LocalWorldConfig>(json).is_err());
    }
    fn ground_region(kind: ElementKind) -> ElementObservation {
        ElementObservation {
            kind,
            color: ElementColor::White,
            position_body_m: Point2 { x_m: 2., y_m: 0. },
            heading_body_rad: Some(0.),
            geometry: ElementGeometry::LineRegion {
                lateral_half_width_m: 0.8,
                depth_m: 1.,
            },
            source: ObservationSource::GroundMarker,
            confidence: 0.9,
            position_error_m: 0.03,
            heading_error_rad: 0.01,
        }
    }
    #[test]
    fn processed_stop_and_finish_refresh_same_identity_from_continued_and_renewed_visibility() {
        for kind in [ElementKind::StopLine, ElementKind::FinishMarker] {
            let original = ground_region(kind);
            let mut world = confirmed_world(original);
            let first = world.tracks(Timestamp(100)).next().unwrap();
            world.mark_processed(first.id).unwrap();
            let target = Point2 {
                x_m: 2.05,
                y_m: 0.01,
            };
            for at in (200..=2600).step_by(100) {
                let mut estimate = pose(at);
                estimate.pose.x_m = 0.25;
                estimate.pose.yaw_rad = 0.1;
                let mut observed = original;
                observed.position_body_m = estimate.pose.world_to_body(target);
                observed.heading_body_rad = Some(-0.05);
                let input = frame(at, vec![observed]);
                world.update(Timestamp(at), &estimate, &input).unwrap();
                let current = world.tracks(Timestamp(at)).next().unwrap();
                assert_eq!(current.id, first.id);
                assert!(current.processed && current.valid);
                assert!(current.position.distance(target) < 1e-12);
                assert!((current.heading_rad.unwrap() - 0.05).abs() < 1e-12);
                assert_eq!(current.last_visual_at, Timestamp(at));
                assert_eq!(current.last_geometry_at, Timestamp(at));
                assert_eq!(current.last_seen, Timestamp(at));
                assert_eq!(current.observations, 2 + (at / 100 - 1) as u32);
                assert_eq!(current.first_seen, first.first_seen);
                world.update(Timestamp(at + 20), &estimate, &input).unwrap();
                assert_eq!(world.elements[0].unwrap(), current);
            }
            // No observation still means expiry. A later real observation of
            // the same passed region can restore clearance evidence, not task
            // eligibility or a new identity.
            assert_eq!(world.tracks(Timestamp(4400)).count(), 0);
            world
                .update(Timestamp(6000), &pose(6000), &frame(6000, vec![original]))
                .unwrap();
            assert_eq!(world.tracks(Timestamp(6000)).count(), 0);
            assert_eq!(world.elements[0].unwrap().observations, 1);
            assert!(world.elements[0].unwrap().processed);
            world
                .update(Timestamp(6100), &pose(6100), &frame(6100, vec![original]))
                .unwrap();
            let restored = world.tracks(Timestamp(6100)).next().unwrap();
            assert_eq!(restored.id, first.id);
            assert!(restored.processed && restored.valid);
            assert_eq!(restored.last_seen, Timestamp(6100));
            assert_eq!(restored.last_visual_at, Timestamp(6100));
            assert_eq!(restored.observations, 2);
            assert_eq!(world.elements.iter().flatten().count(), 1);
        }
    }
    #[test]
    fn retained_regions_expire_without_evidence_and_reconfirm_only_on_distinct_real_frames() {
        for kind in [ElementKind::StopLine, ElementKind::FinishMarker] {
            let original = ground_region(kind);
            let mut world = confirmed_world(original);
            let old = world.tracks(Timestamp(100)).next().unwrap();
            let before_config = serde_json::to_value(world.config()).unwrap();
            assert_eq!(world.config.track_ttl_ms, 1800);
            world
                .update(Timestamp(1900), &pose(1900), &frame(1900, vec![]))
                .unwrap();
            assert_eq!(world.tracks(Timestamp(1900)).count(), 0);
            assert_eq!(world.elements[0].unwrap(), old);
            let first = frame(2000, vec![original]);
            world.update(Timestamp(2000), &pose(2000), &first).unwrap();
            let renewed = world.elements[0].unwrap();
            assert_eq!(renewed.id, old.id);
            assert_eq!(renewed.first_seen, old.first_seen);
            assert_eq!(renewed.observations, 1);
            assert_eq!(renewed.last_seen, Timestamp(2000));
            assert_eq!(world.tracks(Timestamp(2000)).count(), 0);
            world.update(Timestamp(2050), &pose(2000), &first).unwrap();
            assert_eq!(world.elements[0].unwrap(), renewed);
            let mut changed = first.clone();
            changed.observations[0].position_body_m.x_m += 0.01;
            assert!(
                world
                    .update(Timestamp(2060), &pose(2000), &changed)
                    .is_err()
            );
            assert_eq!(world.elements[0].unwrap(), renewed);
            world
                .update(Timestamp(2100), &pose(2100), &frame(2100, vec![original]))
                .unwrap();
            let confirmed = world.tracks(Timestamp(2100)).next().unwrap();
            assert_eq!(confirmed.id, old.id);
            assert_eq!(confirmed.first_seen, old.first_seen);
            assert_eq!(confirmed.observations, 2);
            assert_eq!(confirmed.last_geometry_at, Timestamp(2100));
            assert!(!confirmed.processed);
            assert_eq!(serde_json::to_value(world.config()).unwrap(), before_config);
        }
    }

    #[test]
    fn retained_region_contradiction_and_ambiguous_expired_slots_never_revive() {
        for kind in [ElementKind::StopLine, ElementKind::FinishMarker] {
            for conflict in 0..3 {
                let original = ground_region(kind);
                let mut world = confirmed_world(original);
                let old = world.elements[0].unwrap();
                let mut bad = original;
                match conflict {
                    0 => bad.color = ElementColor::Red,
                    1 => bad.heading_body_rad = Some(1.0),
                    _ => {
                        bad.geometry = ElementGeometry::LineRegion {
                            lateral_half_width_m: 0.8,
                            depth_m: 2.0,
                        }
                    }
                }
                world
                    .update(Timestamp(2200), &pose(2200), &frame(2200, vec![bad]))
                    .unwrap();
                assert!(!world.elements[0].unwrap().valid);
                for at in [2300, 5000] {
                    world
                        .update(Timestamp(at), &pose(at), &frame(at, vec![original]))
                        .unwrap();
                    assert_eq!(world.tracks(Timestamp(at)).count(), 0);
                    let stored = world.elements[0].unwrap();
                    assert_eq!(stored.id, old.id);
                    assert_eq!(stored.last_seen, old.last_seen);
                    assert_eq!(stored.observations, old.observations);
                    assert_eq!(world.elements.iter().flatten().count(), 1);
                }
            }
            let mut left = ground_region(kind);
            left.position_body_m.x_m = 1.;
            let mut right = left;
            right.position_body_m.x_m = 1.8;
            let mut world = LocalWorld::new(config()).unwrap();
            for at in [0, 100] {
                world
                    .update(Timestamp(at), &pose(at), &frame(at, vec![left, right]))
                    .unwrap();
            }
            let ids: Vec<_> = world.tracks(Timestamp(100)).map(|t| t.id).collect();
            assert_eq!(ids.len(), 2);
            let mut ambiguous = left;
            ambiguous.position_body_m.x_m = 1.4;
            world
                .update(Timestamp(2200), &pose(2200), &frame(2200, vec![ambiguous]))
                .unwrap();
            assert!(world.elements.iter().flatten().all(|t| !t.valid));
            world
                .update(
                    Timestamp(2300),
                    &pose(2300),
                    &frame(2300, vec![left, right]),
                )
                .unwrap();
            assert_eq!(world.tracks(Timestamp(2300)).count(), 0);
            assert_eq!(
                world
                    .elements
                    .iter()
                    .flatten()
                    .map(|t| t.id)
                    .collect::<Vec<_>>(),
                ids
            );
        }
    }

    #[test]
    fn pinned_regions_replay_recorded_crosswalk_slot_recycling_without_renewing_permissions() {
        let fixture: serde_json::Value = serde_json::from_str(include_str!(
            "../tests/fixtures/region36900_slot_lifecycle.json"
        ))
        .unwrap();
        for pin in [false, true] {
            let cfg: LocalWorldConfig =
                serde_json::from_value(fixture["world_config"].clone()).unwrap();
            let mut world = LocalWorld::new(cfg).unwrap();
            world.next_id = 3; // Preserve actual labels, not fabricated prehistory.
            for raw in fixture["records"].as_array().unwrap() {
                let at = Timestamp(raw["at"].as_u64().unwrap());
                let pose: PoseEstimate = serde_json::from_value(raw["pose"].clone()).unwrap();
                let frame: ElementFrame = serde_json::from_value(raw["frame"].clone()).unwrap();
                world.update(at, &pose, &frame).unwrap();
                if at.0 == 26600 && pin {
                    world.set_region_pins(at, &[TrackId(3)]).unwrap();
                }
                if [33800, 33900].contains(&at.0) {
                    assert!(
                        !world
                            .tracks(at)
                            .any(|track| track.kind == ElementKind::StopLine)
                    );
                    assert_eq!(
                        world
                            .elements
                            .iter()
                            .flatten()
                            .any(|track| track.id == TrackId(3)),
                        pin
                    );
                }
                if at.0 == 34300 {
                    assert!(
                        !world
                            .tracks(at)
                            .any(|track| track.kind == ElementKind::StopLine)
                    );
                    let line = world
                        .elements
                        .iter()
                        .flatten()
                        .find(|track| track.kind == ElementKind::StopLine)
                        .unwrap();
                    assert_eq!(line.id, TrackId(if pin { 3 } else { 5 }));
                    assert_eq!(line.observations, 1);
                    assert_eq!(line.last_seen, at);
                }
            }
            let line = world
                .tracks(Timestamp(34400))
                .find(|track| track.kind == ElementKind::StopLine)
                .unwrap();
            assert_eq!(line.id, TrackId(if pin { 3 } else { 5 }));
            assert_eq!(line.observations, 2);
            assert_eq!(line.first_seen, Timestamp(if pin { 26500 } else { 34300 }));
            assert_eq!(world.config.track_ttl_ms, 1800);
            assert_eq!(world.elements.len(), MAX_ELEMENTS);
        }
    }

    #[test]
    fn region_pins_are_atomic_references_and_never_create_or_revive_geometry() {
        let original = ground_region(ElementKind::StopLine);
        let mut world = confirmed_world(original);
        let old = world.tracks(Timestamp(100)).next().unwrap();
        world.set_region_pins(Timestamp(100), &[old.id]).unwrap();
        let pins = world.region_pins;
        for ids in [
            vec![TrackId(999)],
            vec![old.id, old.id],
            vec![old.id; MAX_ELEMENTS + 1],
        ] {
            assert!(world.set_region_pins(Timestamp(100), &ids).is_err());
            assert_eq!(world.region_pins, pins);
            assert_eq!(world.elements[0].unwrap(), old);
        }
        assert_eq!(world.tracks(Timestamp(2200)).count(), 0);
        world.set_region_pins(Timestamp(2200), &[old.id]).unwrap();
        assert_eq!(world.elements[0].unwrap(), old);
        let mut contradictory = original;
        contradictory.heading_body_rad = Some(1.0);
        world
            .update(
                Timestamp(2200),
                &pose(2200),
                &frame(2200, vec![contradictory]),
            )
            .unwrap();
        let invalid = world.elements[0].unwrap();
        assert!(!invalid.valid);
        world.set_region_pins(Timestamp(2200), &[old.id]).unwrap();
        world
            .update(Timestamp(2300), &pose(2300), &frame(2300, vec![original]))
            .unwrap();
        assert_eq!(world.elements[0].unwrap(), invalid);
        assert_eq!(world.tracks(Timestamp(2300)).count(), 0);
        world.set_region_pins(Timestamp(2300), &[]).unwrap();
        assert!(world.set_region_pins(Timestamp(2300), &[old.id]).is_err());
        world
            .update(Timestamp(2400), &pose(2400), &frame(2400, vec![cone(6.)]))
            .unwrap();
        assert_ne!(world.elements[0].unwrap().id, old.id);
        assert!(world.set_region_pins(Timestamp(2400), &[old.id]).is_err());
        let cone_id = world.elements[0].unwrap().id;
        world
            .update(Timestamp(2500), &pose(2500), &frame(2500, vec![cone(6.)]))
            .unwrap();
        assert!(world.set_region_pins(Timestamp(2500), &[cone_id]).is_err());
        let mut unconfirmed = LocalWorld::new(config()).unwrap();
        unconfirmed
            .update(Timestamp(100), &pose(100), &frame(100, vec![original]))
            .unwrap();
        assert!(
            unconfirmed
                .set_region_pins(Timestamp(100), &[TrackId(1)])
                .is_err()
        );
    }

    #[test]
    fn pinned_region_capacity_is_fixed_and_exhaustion_is_transactional_until_owner_release() {
        let observations: Vec<_> = (0..MAX_ELEMENTS)
            .map(|i| {
                let mut observed = ground_region(ElementKind::StopLine);
                observed.position_body_m.x_m = (i + 1) as f64;
                observed
            })
            .collect();
        let mut world = LocalWorld::new(config()).unwrap();
        for at in [0, 100] {
            world
                .update(Timestamp(at), &pose(at), &frame(at, observations.clone()))
                .unwrap();
        }
        let ids: Vec<_> = world.tracks(Timestamp(100)).map(|track| track.id).collect();
        assert_eq!(ids.len(), MAX_ELEMENTS);
        world.set_region_pins(Timestamp(100), &ids).unwrap();
        let before = world.elements;
        let new_frame = frame(2200, vec![cone(20.)]);
        assert!(
            world
                .update(Timestamp(2200), &pose(2200), &new_frame)
                .is_err()
        );
        assert_eq!(world.elements, before);
        assert_eq!(world.last_frame.as_ref().unwrap().at, Timestamp(100));
        assert_eq!(world.tracks(Timestamp(2200)).count(), 0);
        world.set_region_pins(Timestamp(2200), &[]).unwrap();
        world
            .update(Timestamp(2200), &pose(2200), &new_frame)
            .unwrap();
        assert_eq!(world.elements.iter().flatten().count(), MAX_ELEMENTS);
        assert_eq!(world.elements[0].unwrap().kind, ElementKind::Cone);
        assert_eq!(world.elements[0].unwrap().id, TrackId(17));
    }

    #[test]
    fn retained_region_slot_recycling_never_reconstructs_the_erased_identity() {
        let original = ground_region(ElementKind::StopLine);
        let mut world = confirmed_world(original);
        let old = world.elements[0].unwrap();
        world
            .update(Timestamp(2200), &pose(2200), &frame(2200, vec![cone(5.)]))
            .unwrap();
        assert_ne!(world.elements[0].unwrap().id, old.id);
        assert_eq!(world.elements[0].unwrap().kind, ElementKind::Cone);
        for at in [2300, 2400] {
            world
                .update(Timestamp(at), &pose(at), &frame(at, vec![original]))
                .unwrap();
        }
        let fresh = world
            .tracks(Timestamp(2400))
            .find(|t| t.kind == ElementKind::StopLine)
            .unwrap();
        assert_ne!(fresh.id, old.id);
        assert_eq!(fresh.first_seen, Timestamp(2300));
        assert_eq!(fresh.observations, 2);
        assert!(!world.elements.iter().flatten().any(|t| t.id == old.id));
        assert_eq!(world.elements.len(), MAX_ELEMENTS);
    }

    #[test]
    fn retained_region_captured_31000_frame_and_synthetic_second_frame_keep_old_confirmed_id() {
        let saved: serde_json::Value = serde_json::from_str(include_str!(
            "../tests/fixtures/region31060_retained_observation.json"
        ))
        .unwrap();
        let cfg: LocalWorldConfig = serde_json::from_value(saved["config"].clone()).unwrap();
        let source: PoseEstimate = serde_json::from_value(saved["source"].clone()).unwrap();
        let observed: ElementFrame = serde_json::from_value(saved["frame"].clone()).unwrap();
        let raw = &saved["cached"]["track"];
        macro_rules! read {
            ($field:ident) => {
                serde_json::from_value(raw[stringify!($field)].clone()).unwrap()
            };
        }
        // This is the actual captured negative cache, used as a retained-slot
        // policy fixture. It is not claimed to be a captured LocalWorld slot.
        let old = ElementTrack {
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
        assert_eq!(old.id, TrackId(4));
        assert_eq!(old.last_seen, Timestamp(17900));
        assert_eq!(source.captured_at, Timestamp(31000));
        assert_eq!(observed.captured_at, source.captured_at);
        let mut world = LocalWorld::new(cfg).unwrap();
        world.elements[0] = Some(old);
        world.next_id = 5;
        assert_eq!(world.tracks(Timestamp(31060)).count(), 0);
        world.update(Timestamp(31060), &source, &observed).unwrap();
        let once = world.elements[0].unwrap();
        assert_eq!(once.id, old.id);
        assert_eq!(once.first_seen, old.first_seen);
        assert_eq!(once.last_geometry_at, Timestamp(31000));
        assert_eq!(once.observations, 1);
        assert_eq!(world.tracks(Timestamp(31060)).count(), 0);
        world.update(Timestamp(31070), &source, &observed).unwrap();
        assert_eq!(world.elements[0].unwrap(), once);
        // Explicit synthetic next distinct capture, solely to test reconfirmation.
        let mut next_source = source.clone();
        next_source.captured_at = Timestamp(31100);
        let mut next_frame = observed.clone();
        next_frame.captured_at = next_source.captured_at;
        world
            .update(Timestamp(31100), &next_source, &next_frame)
            .unwrap();
        let renewed = world.tracks(Timestamp(31100)).next().unwrap();
        assert_eq!(renewed.id, old.id);
        assert_eq!(renewed.first_seen, old.first_seen);
        assert_eq!(renewed.observations, 2);
        assert_eq!(renewed.last_geometry_at, Timestamp(31100));
        assert!(
            renewed.position.distance(
                source
                    .pose
                    .body_to_world(observed.observations[0].position_body_m)
            ) < 1e-12
        );
        assert_eq!(world.elements.iter().flatten().count(), 1);
    }

    #[test]
    fn processed_region_contradiction_never_recovers_or_renews_geometry() {
        for kind in [ElementKind::StopLine, ElementKind::FinishMarker] {
            let original = ground_region(kind);
            let mut world = confirmed_world(original);
            let first = world.tracks(Timestamp(100)).next().unwrap();
            world.mark_processed(first.id).unwrap();
            let mut contradictory = original;
            contradictory.heading_body_rad = Some(1.);
            world
                .update(Timestamp(200), &pose(200), &frame(200, vec![contradictory]))
                .unwrap();
            for at in [300, 3000] {
                world
                    .update(Timestamp(at), &pose(at), &frame(at, vec![original]))
                    .unwrap();
                assert_eq!(world.tracks(Timestamp(at)).count(), 0);
                let stored = world.elements[0].unwrap();
                assert_eq!(stored.id, first.id);
                assert!(stored.processed && !stored.valid);
                assert_eq!(stored.last_seen, Timestamp(100));
                assert_eq!(stored.last_geometry_at, Timestamp(100));
                assert_eq!(stored.last_visual_at, Timestamp(100));
                assert_eq!(stored.observations, first.observations);
                assert_eq!(world.elements.iter().flatten().count(), 1);
            }
        }
    }
    #[test]
    fn processed_cone_and_crosswalk_remain_frozen_tombstones_despite_real_observations() {
        for observation in [cone(2.), ground_region(ElementKind::Crosswalk)] {
            let mut world = confirmed_world(observation);
            world.mark_processed(world.elements[0].unwrap().id).unwrap();
            let tombstone = world.elements[0].unwrap();
            for at in [200, 3000] {
                world
                    .update(Timestamp(at), &pose(at), &frame(at, vec![observation]))
                    .unwrap();
                assert_eq!(world.elements[0].unwrap(), tombstone);
                assert_eq!(world.elements.iter().flatten().count(), 1);
            }
            assert_eq!(world.tracks(Timestamp(3000)).count(), 0);
        }
    }
    #[test]
    fn real_visual_updates_retain_only_roundoff_geometry_and_keep_fresh_evidence() {
        let mut observation = ground_region(ElementKind::StopLine);
        observation.position_body_m = Point2 { x_m: 1.5, y_m: 2.5 };
        observation.heading_body_rad = Some(0.25);
        let mut world = confirmed_world(observation);
        let original = world.tracks(Timestamp(100)).next().unwrap();
        for (index, at) in [200, 300, 400].into_iter().enumerate() {
            let mut noisy = observation;
            let noise = if index % 2 == 0 {
                4. * f64::EPSILON
            } else {
                -4. * f64::EPSILON
            };
            noisy.position_body_m.x_m += noise;
            noisy.position_body_m.y_m -= noise;
            noisy.heading_body_rad = Some(0.25 + noise);
            noisy.position_error_m = 0.06 + index as f64 * 0.01;
            noisy.heading_error_rad = 0.02 + index as f64 * 0.01;
            world
                .update(Timestamp(at), &pose(at), &frame(at, vec![noisy]))
                .unwrap();
            let current = world.tracks(Timestamp(at)).next().unwrap();
            assert_eq!(
                current.position.x_m.to_bits(),
                original.position.x_m.to_bits()
            );
            assert_eq!(
                current.position.y_m.to_bits(),
                original.position.y_m.to_bits()
            );
            assert_eq!(
                current.heading_rad.unwrap().to_bits(),
                original.heading_rad.unwrap().to_bits()
            );
            assert_eq!(current.id, original.id);
            assert_eq!(
                current.observations,
                original.observations + index as u32 + 1
            );
            assert_eq!(current.last_seen, Timestamp(at));
            assert_eq!(current.last_visual_at, Timestamp(at));
            assert_eq!(current.last_geometry_at, Timestamp(at));
            let expected_error = noisy.position_error_m
                + world.config.pose_position_error_m
                + noisy.position_body_m.x_m.hypot(noisy.position_body_m.y_m)
                    * world.config.pose_heading_error_rad;
            assert_eq!(current.position_error_m, expected_error);
            assert_eq!(
                current.heading_error_rad,
                noisy.heading_error_rad + world.config.pose_heading_error_rad
            );
        }
        let mut changed = observation;
        changed.position_body_m.x_m += 0.001;
        changed.position_body_m.y_m -= 0.001;
        changed.heading_body_rad = Some(0.251);
        world
            .update(Timestamp(500), &pose(500), &frame(500, vec![changed]))
            .unwrap();
        let current = world.tracks(Timestamp(500)).next().unwrap();
        assert_eq!(current.position, changed.position_body_m);
        assert!((current.heading_rad.unwrap() - original.heading_rad.unwrap()).abs() > 0.0009);
        assert_eq!(current.last_visual_at, Timestamp(500));
    }
    #[test]
    fn range_maintenance_roundtrips_keep_float_representation_but_millimetre_motion_updates() {
        let mut observation = cone(1.5);
        observation.position_body_m.y_m = 2.5;
        let mut world = confirmed_world(observation);
        let original = world.tracks(Timestamp(100)).next().unwrap();
        let extrinsic = Pose2 {
            x_m: 0.15,
            y_m: 0.07,
            yaw_rad: 0.1,
        };
        let cfg = maintenance_config(extrinsic);
        for (index, at) in (200..=1300).step_by(100).enumerate() {
            let angle = 0.37 * index as f64;
            let mut estimate = pose(at);
            estimate.pose = Pose2 {
                x_m: original.position.x_m + 1.2 * angle.cos(),
                y_m: original.position.y_m + 1.2 * angle.sin(),
                yaw_rad: wrap(angle + PI / 2.),
            };
            let body_center = estimate.pose.world_to_body(original.position);
            let s = cone_scan(at, body_center, 0.2, extrinsic);
            assert_eq!(maintain_with_scan(&mut world, &estimate, &s, &cfg), 1);
            let current = world.tracks(Timestamp(at)).next().unwrap();
            assert_eq!(
                current.position.x_m.to_bits(),
                original.position.x_m.to_bits()
            );
            assert_eq!(
                current.position.y_m.to_bits(),
                original.position.y_m.to_bits()
            );
            assert_eq!(current.last_geometry_at, Timestamp(at));
            assert_eq!(current.last_visual_at, original.last_visual_at);
            assert_eq!(current.observations, original.observations);
            assert!(current.position_error_m >= original.position_error_m);
            assert!(current.position_error_m <= world.config.max_position_error_m);
        }
        let shifted = Point2 {
            x_m: original.position.x_m + 0.001,
            y_m: original.position.y_m - 0.001,
        };
        let s = cone_scan(1400, shifted, 0.2, extrinsic);
        assert_eq!(maintain_with_scan(&mut world, &pose(1400), &s, &cfg), 1);
        let current = world.tracks(Timestamp(1400)).next().unwrap();
        assert!(current.position.distance(shifted) < 1e-12);
        assert!((current.position.x_m - original.position.x_m).abs() > 0.0009);
        assert!((current.position.y_m - original.position.y_m).abs() > 0.0009);
        assert_eq!(current.last_geometry_at, Timestamp(1400));
    }
    #[test]
    fn roundoff_representation_window_handles_zero_angle_wrap_and_extreme_coordinates() {
        assert_eq!(
            retain_roundoff_scalar(0., f64::EPSILON / 2.).to_bits(),
            0_f64.to_bits()
        );
        assert_eq!(
            retain_roundoff_scalar(1.5, 1.5_f64.next_up()).to_bits(),
            1.5_f64.to_bits()
        );
        // Even a tiny change beyond the explicit ULP window is accepted;
        // a much wider, measurement-sized deadband would fail this check.
        let beyond_roundoff = 1.5 + 128. * f64::EPSILON;
        assert_eq!(
            retain_roundoff_scalar(1.5, beyond_roundoff).to_bits(),
            beyond_roundoff.to_bits()
        );
        for old in [0., 1.5, -2.5, 1e12] {
            let shifted = old + 0.001;
            assert_ne!(old, shifted);
            assert_eq!(
                retain_roundoff_scalar(old, shifted).to_bits(),
                shifted.to_bits()
            );
        }
        // Wrapping +pi to -pi must not invent a new heading for the same axis.
        let old = PI - 4. * f64::EPSILON;
        let same = -PI + 4. * f64::EPSILON;
        assert_eq!(retain_roundoff_heading(Some(old), Some(same)), Some(old));
        assert_eq!(
            retain_roundoff_heading(Some(old), Some(same + 0.001)),
            Some(same + 0.001)
        );
        assert_eq!(retain_roundoff_heading(None, Some(0.)), Some(0.));
        assert_eq!(retain_roundoff_heading(Some(0.), None), None);
    }

    fn hull_test_world(wall_y: Option<f64>, missing: bool, obstacle: bool) -> LocalWorld {
        let mut cfg = config();
        cfg.self_footprint = Some(Footprint {
            front_m: 0.22,
            rear_m: 0.18,
            half_width_m: 0.13,
        });
        let mut world = LocalWorld::new(cfg).unwrap();
        let mut ranges = scan(100);
        ranges.ranges_m = vec![Some(5.); 1440];
        ranges.angle_increment_rad = TAU / 1440.;
        for (i, range) in ranges.ranges_m.iter_mut().enumerate() {
            let a = i as f64 * ranges.angle_increment_rad;
            if let Some(y) = wall_y {
                let hit = y / a.sin();
                if hit > 0. && hit.is_finite() {
                    *range = Some(5f64.min(hit));
                }
            }
        }
        if missing {
            ranges.ranges_m[0] = None;
        }
        if obstacle {
            ranges.ranges_m[0] = Some(1.5);
        }
        world
            .update_scan(Timestamp(100), &pose(100), &ranges, Pose2::default())
            .unwrap();
        world
    }
    fn hull_rectangle() -> [Point2; 4] {
        [
            Point2 {
                x_m: -0.18,
                y_m: -0.13,
            },
            Point2 {
                x_m: 0.6,
                y_m: -0.13,
            },
            Point2 {
                x_m: 0.6,
                y_m: 0.13,
            },
            Point2 {
                x_m: -0.18,
                y_m: 0.13,
            },
        ]
    }
    fn old_hull_circle(world: &LocalWorld, at: Timestamp, points: &[Point2], padding: f64) -> bool {
        let center = Point2 {
            x_m: points.iter().map(|p| p.x_m / points.len() as f64).sum(),
            y_m: points.iter().map(|p| p.y_m / points.len() as f64).sum(),
        };
        let radius = points.iter().map(|p| p.distance(center)).fold(0., f64::max) + padding;
        world.space_state(at, center, radius) == SpaceState::KnownFree
    }
    #[test]
    fn hull_complete_thin_body_along_wall_succeeds_where_old_circle_cannot() {
        let world = hull_test_world(Some(0.24), false, false);
        let points = hull_rectangle();
        assert!(!old_hull_circle(&world, Timestamp(100), &points, 0.04));
        assert!(world.known_free_convex_hull(Timestamp(100), &points, 0.04));
        assert_eq!(
            world.convex_hull_space_state(Timestamp(100), &points, 0.04),
            SpaceState::KnownFree
        );
        assert!(!world.known_free_convex_hull(Timestamp(400), &points, 0.04));
        assert!(!world.known_free_convex_hull(Timestamp(99), &points, 0.04));
    }
    #[test]
    fn hull_complete_interior_unknown_ray_and_obstacle_reject_even_with_free_vertices() {
        let points = [
            Point2 {
                x_m: 1.,
                y_m: -0.25,
            },
            Point2 {
                x_m: 2.,
                y_m: -0.25,
            },
            Point2 { x_m: 2., y_m: 0.25 },
            Point2 { x_m: 1., y_m: 0.25 },
        ];
        for (missing, obstacle) in [(true, false), (false, true)] {
            let world = hull_test_world(None, missing, obstacle);
            assert!(
                points
                    .iter()
                    .all(|p| world.space_state(Timestamp(100), *p, 0.) == SpaceState::KnownFree)
            );
            assert!(!world.known_free_convex_hull(Timestamp(100), &points, 0.));
        }
        let blind = LocalWorld::new(config()).unwrap();
        assert!(!blind.known_free_convex_hull(Timestamp(100), &points, 0.));
    }
    #[test]
    fn hull_adaptive_leaves_cover_the_complete_irregular_area_and_padding_with_bounded_work() {
        let points = [
            Point2 { x_m: 0.4, y_m: 0.7 },
            Point2 { x_m: 1.6, y_m: 1.1 },
            Point2 { x_m: 1.3, y_m: 1.9 },
            Point2 { x_m: 0.2, y_m: 1.4 },
        ];
        let padding = 0.04;
        let bounds = HullBounds::new(&points, padding).unwrap();
        let mut calls = 0;
        let mut leaves = Vec::new();
        assert!(bounds.cover_with(|center, radius| {
            calls += 1;
            if radius <= 0.3 {
                leaves.push((center, radius));
                true
            } else {
                false
            }
        }));
        assert!(calls <= MAX_HULL_CELLS);
        assert!(leaves.len() > 1);
        // Convex combinations sample the interior, and disc offsets include
        // its padding. The actual proof is the complete shared-seam rectangle
        // partition, rather than these samples or its original vertices.
        for seed in 0..=200 {
            let weights: [f64; 4] =
                std::array::from_fn(|i| ((seed * (i + 3) * 7) % 101 + 1) as f64);
            let sum: f64 = weights.iter().sum();
            let center = Point2 {
                x_m: points
                    .iter()
                    .zip(weights)
                    .map(|(p, w)| p.x_m * w / sum)
                    .sum(),
                y_m: points
                    .iter()
                    .zip(weights)
                    .map(|(p, w)| p.y_m * w / sum)
                    .sum(),
            };
            for origin in points.iter().copied().chain(std::iter::once(center)) {
                for angle in 0..16 {
                    let angle = TAU * angle as f64 / 16.;
                    let point = Point2 {
                        x_m: origin.x_m + padding * angle.cos(),
                        y_m: origin.y_m + padding * angle.sin(),
                    };
                    assert!(
                        leaves
                            .iter()
                            .any(|(center, radius)| point.distance(*center) <= *radius)
                    );
                }
            }
        }
        let mut queries = 0;
        assert!(!bounds.cover_with(|_, _| {
            queries += 1;
            false
        }));
        assert_eq!(queries, MAX_HULL_CELLS);
        // A single unresolved interior point leaves at least one entire leaf
        // uncertified; success on every other leaf must not hide it.
        let interior = Point2 { x_m: 0.9, y_m: 1.3 };
        assert!(!bounds.cover_with(|center, radius| center.distance(interior) > radius));
        let adjacent = HullCell {
            min_x: 1.0,
            max_x: 1.0_f64.next_up(),
            min_y: 0.,
            max_y: 0.,
        };
        assert!(adjacent.split().is_none());
    }

    #[test]
    fn hull_adaptive_certifies_actual_small_stopping_envelope_without_losing_unknown_or_hit_rejection()
     {
        use crate::admission::StoppingEnvelope;
        use crate::navigation::NavigationConfig;
        let original: serde_json::Value =
            serde_json::from_str(include_str!("../../../config/competition-sim.json")).unwrap();
        let nav: NavigationConfig =
            serde_json::from_value(original["autonomy"]["navigation"].clone()).unwrap();
        let source = Pose2 {
            x_m: 4.5628402636868195,
            y_m: 1.627711072631966,
            yaw_rad: 0.977435753833519,
        };
        let speed = 0.18;
        let curvature = 1.5243796941456;
        let at = Timestamp(26500);
        let envelope = StoppingEnvelope::new(&nav, source, speed, curvature, 0.35).unwrap();
        let points = envelope.corners().map(|point| source.body_to_world(point));
        let mut cfg = config();
        cfg.self_footprint = Some(nav.footprint);
        assert_eq!(cfg.pose_position_error_m, 0.02);
        assert_eq!(cfg.pose_heading_error_rad, 0.01);
        assert_eq!(cfg.laser_error_m, 0.02);
        // Captured from the 26500 ms small-field refusal. Reconstruct exactly
        // the simulation ray equations; no scene geometry enters LocalWorld.
        let mut sample = scan(at.0);
        sample.angle_increment_rad = -TAU / 360.;
        sample.range_min_m = 0.02;
        sample.range_max_m = 12.;
        sample.ranges_m = (0..360)
            .map(|index| {
                let a = source.yaw_rad + index as f64 * sample.angle_increment_rad;
                let (dy, dx) = a.sin_cos();
                let mut distance = 12.0_f64;
                for (boundary, position, direction) in [
                    (0., source.x_m, dx),
                    (5., source.x_m, dx),
                    (0., source.y_m, dy),
                    (4., source.y_m, dy),
                ] {
                    if direction.abs() > 1e-9 {
                        let t = (boundary - position) / direction;
                        if t > 0. {
                            distance = distance.min(t);
                        }
                    }
                }
                for x in [4., 1.] {
                    let ox = source.x_m - x;
                    let oy = source.y_m - 2.;
                    let b = ox * dx + oy * dy;
                    let radius = 0.19798989873223333_f64;
                    let discriminant = b * b - (ox * ox + oy * oy - radius * radius);
                    if discriminant >= 0. {
                        let t = -b - discriminant.sqrt();
                        if t > 0. {
                            distance = distance.min(t);
                        }
                    }
                }
                Some(distance)
            })
            .collect();
        let mut estimate = pose(at.0);
        estimate.pose = source;
        estimate.speed_mps = speed;
        estimate.yaw_rate_radps = speed * curvature;
        for variant in [0, 1, 2] {
            let mut sample = sample.clone();
            if variant == 1 {
                sample.ranges_m[0] = None;
            }
            if variant == 2 {
                sample.ranges_m[0] = Some(0.15);
            }
            let mut world = LocalWorld::new(cfg.clone()).unwrap();
            world
                .update_scan(at, &estimate, &sample, Pose2::default())
                .unwrap();
            assert!(!old_hull_circle(&world, at, &points, 0.));
            let uniform = HullCover::new(&points, 0.).unwrap();
            assert!(!(0..uniform.cells()).all(|index| {
                let (center, radius) = uniform.disc(index);
                world.space_state(at, center, radius) == SpaceState::KnownFree
            }));
            assert_eq!(world.known_free_convex_hull(at, &points, 0.), variant == 0);
            assert_eq!(
                world.convex_hull_space_state(at, &points, 0.),
                match variant {
                    0 => SpaceState::KnownFree,
                    1 => SpaceState::Unknown,
                    _ => SpaceState::Obstacle,
                }
            );
            assert!(!world.known_free_convex_hull(Timestamp(at.0 + cfg.scan_ttl_ms), &points, 0.));
        }
    }

    #[test]
    fn hull_obb_discs_cover_irregular_convex_area_and_padding_with_fixed_capacity() {
        let points = [
            Point2 { x_m: 0.4, y_m: 0.7 },
            Point2 { x_m: 1.6, y_m: 1.1 },
            Point2 { x_m: 1.3, y_m: 1.9 },
            Point2 { x_m: 0.2, y_m: 1.4 },
            Point2 { x_m: 0.4, y_m: 0.7 },
        ];
        let padding = 0.04;
        let cover = HullCover::new(&points, padding).unwrap();
        assert!(cover.cells() <= MAX_HULL_CELLS);
        for seed in 1..=200 {
            let weights: [f64; 5] =
                std::array::from_fn(|i| ((seed * (i + 3) * 7) % 101 + 1) as f64);
            let sum: f64 = weights.iter().sum();
            let point = Point2 {
                x_m: points
                    .iter()
                    .zip(weights)
                    .map(|(p, w)| p.x_m * w / sum)
                    .sum(),
                y_m: points
                    .iter()
                    .zip(weights)
                    .map(|(p, w)| p.y_m * w / sum)
                    .sum(),
            };
            for angle in 0..16 {
                let a = angle as f64 * TAU / 16.;
                let q = Point2 {
                    x_m: point.x_m + padding * a.cos(),
                    y_m: point.y_m + padding * a.sin(),
                };
                assert!(
                    (0..cover.cells()).any(|i| {
                        let (c, r) = cover.disc(i);
                        c.distance(q) <= r
                    }),
                    "uncovered {q:?}"
                );
            }
        }
        for p in points {
            for angle in 0..16 {
                let a = angle as f64 * TAU / 16.;
                let q = Point2 {
                    x_m: p.x_m + padding * a.cos(),
                    y_m: p.y_m + padding * a.sin(),
                };
                assert!((0..cover.cells()).any(|i| {
                    let (c, r) = cover.disc(i);
                    c.distance(q) <= r
                }));
            }
        }
    }
    #[test]
    fn hull_input_bounds_and_existing_sensor_configuration_are_preserved() {
        let world = hull_test_world(None, false, false);
        assert_eq!(MAX_LOCAL_RAYS, 1440);
        assert_eq!(MAX_ELEMENTS, 16);
        assert!(!world.known_free_convex_hull(Timestamp(100), &[], 0.));
        assert!(!world.known_free_convex_hull(
            Timestamp(100),
            &[Point2 { x_m: 1., y_m: 0. }; 33],
            0.
        ));
        assert!(!world.known_free_convex_hull(
            Timestamp(100),
            &[Point2 {
                x_m: f64::NAN,
                y_m: 0.
            }],
            0.
        ));
        assert!(!world.known_free_convex_hull(Timestamp(100), &hull_rectangle(), -0.01));
        let mut without_prior = LocalWorld::new(config()).unwrap();
        without_prior
            .update_scan(Timestamp(100), &pose(100), &scan(100), Pose2::default())
            .unwrap();
        assert!(!without_prior.known_free_convex_hull(Timestamp(100), &hull_rectangle(), 0.04));
    }

    #[test]
    fn hull_tri_state_scans_past_unknown_cells_to_find_a_later_obstacle() {
        let mut world = LocalWorld::new(config()).unwrap();
        let mut source = scan(100);
        source.ranges_m[705] = None;
        let points = [
            Point2 {
                x_m: 1.0,
                y_m: -0.3,
            },
            Point2 {
                x_m: 2.0,
                y_m: -0.3,
            },
            Point2 { x_m: 2.0, y_m: 0.3 },
            Point2 { x_m: 1.0, y_m: 0.3 },
        ];
        world
            .update_scan(Timestamp(100), &pose(100), &source, Pose2::default())
            .unwrap();
        assert_eq!(
            world.convex_hull_space_state(Timestamp(100), &points, 0.0),
            SpaceState::Unknown
        );
        assert!(!world.known_free_convex_hull(Timestamp(100), &points, 0.0));
        source.captured_at = Timestamp(200);
        source.ranges_m[15] = Some(1.51);
        world
            .update_scan(Timestamp(200), &pose(200), &source, Pose2::default())
            .unwrap();
        let cover = HullCover::new(&points, 0.0).unwrap();
        let cells: Vec<_> = (0..cover.cells())
            .map(|i| {
                let (c, r) = cover.disc(i);
                world.space_state(Timestamp(200), c, r)
            })
            .collect();
        let unknown = cells
            .iter()
            .position(|s| *s == SpaceState::Unknown)
            .unwrap();
        let obstacle = cells
            .iter()
            .position(|s| *s == SpaceState::Obstacle)
            .unwrap();
        assert!(unknown < obstacle);
        assert_eq!(
            world.convex_hull_space_state(Timestamp(200), &points, 0.0),
            SpaceState::Obstacle
        );
        assert!(!world.known_free_convex_hull(Timestamp(200), &points, 0.0));
        assert_eq!(
            world.convex_hull_space_state(Timestamp(200), &[], 0.0),
            SpaceState::Unknown
        );
    }

    #[test]
    fn hull_tri_state_cover_excess_is_unknown_but_real_uncertainty_collision_is_obstacle() {
        let points = [
            Point2 {
                x_m: 1.0,
                y_m: -0.3,
            },
            Point2 {
                x_m: 2.0,
                y_m: -0.3,
            },
            Point2 { x_m: 1.0, y_m: 0.3 },
        ];
        let mut world = LocalWorld::new(config()).unwrap();
        let mut source = scan(100);
        // This hit lies in the OBB's extra upper-right triangle, more than
        // .37 m from the actual triangular hull, not in the requested sweep.
        source.ranges_m[12] = Some(1.91);
        world
            .update_scan(Timestamp(100), &pose(100), &source, Pose2::default())
            .unwrap();
        let cover = HullCover::new(&points, 0.0).unwrap();
        assert!((0..cover.cells()).any(|i| {
            let (c, r) = cover.disc(i);
            world.space_state(Timestamp(100), c, r) == SpaceState::Obstacle
        }));
        assert!(!world.known_free_convex_hull(Timestamp(100), &points, 0.0));
        assert_eq!(
            world.convex_hull_space_state(Timestamp(100), &points, 0.0),
            SpaceState::Unknown
        );
        // A different return is just outside the actual hull; the unchanged
        // range/pose error still intersects it, even with zero extra padding.
        source.captured_at = Timestamp(200);
        source.ranges_m[12] = Some(5.0);
        source.ranges_m[2] = Some(1.51);
        world
            .update_scan(Timestamp(200), &pose(200), &source, Pose2::default())
            .unwrap();
        assert_eq!(
            world.convex_hull_space_state(Timestamp(200), &points, 0.0),
            SpaceState::Obstacle
        );
        assert!(!world.known_free_convex_hull(Timestamp(200), &points, 0.0));
    }

    #[test]
    fn hull_hit_distance_handles_unordered_vertices_and_degenerate_hulls() {
        let points = [
            Point2 { x_m: 1.0, y_m: 0.3 },
            Point2 {
                x_m: 1.0,
                y_m: -0.3,
            },
            Point2 {
                x_m: 2.0,
                y_m: -0.3,
            },
            Point2 { x_m: 1.0, y_m: 0.3 },
            Point2 { x_m: 1.2, y_m: 0.0 },
        ];
        let hull = HitHull::new(&points).unwrap();
        assert_eq!(hull.len, 3);
        assert_eq!(hull.distance(Point2 { x_m: 1.2, y_m: 0.0 }), Some(0.0));
        let expected = 0.44 / 1.36f64.sqrt();
        assert!((hull.distance(Point2 { x_m: 1.9, y_m: 0.2 }).unwrap() - expected).abs() < 1e-12);
        let line = HitHull::new(&[
            Point2 { x_m: 1.0, y_m: 0.0 },
            Point2 { x_m: 2.0, y_m: 0.0 },
            Point2 { x_m: 1.5, y_m: 0.0 },
        ])
        .unwrap();
        assert_eq!(line.len, 2);
        assert!((line.distance(Point2 { x_m: 1.5, y_m: 0.2 }).unwrap() - 0.2).abs() < 1e-12);
        let repeated = HitHull::new(&[Point2 { x_m: 1.0, y_m: 0.0 }; 32]).unwrap();
        assert_eq!(repeated.len, 1);
        assert_eq!(repeated.distance(Point2 { x_m: 2.0, y_m: 0.0 }), Some(1.0));
        assert!(HitHull::new(&[Point2::default(); 33]).is_none());
    }
    #[test]
    #[ignore = "explicit host microbenchmark; no target deadline claim"]
    fn hull_benchmark_original_and_complete_obb_cover() {
        use std::{hint::black_box, time::Instant};
        let points = hull_rectangle();
        for (label, world) in [
            ("open", hull_test_world(None, false, false)),
            ("near_wall", hull_test_world(Some(0.24), false, false)),
            ("missing_ray", hull_test_world(None, true, false)),
        ] {
            let loops = 1000;
            let old_result = old_hull_circle(&world, Timestamp(100), &points, 0.04);
            let new_result = world.known_free_convex_hull(Timestamp(100), &points, 0.04);
            let old_start = Instant::now();
            for _ in 0..loops {
                black_box(old_hull_circle(
                    black_box(&world),
                    Timestamp(100),
                    black_box(&points),
                    0.04,
                ));
            }
            let old = old_start.elapsed().as_nanos() as f64 / loops as f64;
            let new_start = Instant::now();
            for _ in 0..loops {
                black_box(world.known_free_convex_hull(Timestamp(100), black_box(&points), 0.04));
            }
            let new = new_start.elapsed().as_nanos() as f64 / loops as f64;
            println!(
                "HULL_BENCH {label}: old_ns={old:.0} new_ns={new:.0} ratio={:.2} old_result={old_result} new_result={new_result} rays=1440 max_queries=64",
                new / old
            );
        }
    }
}
