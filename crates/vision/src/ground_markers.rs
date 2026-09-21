//! Explicit experimental ground patterns, NOT confirmed competition markings.
//!
//! The rules describe a stop area between a photosensor and a lamp and mention
//! a finish line, but do not specify a visible line pattern. These semantics
//! are therefore produced only by an enabled, declared simulation protocol.
//! Position comes from RGB geometry through the supplied ground homography.
//! The forward sign uses an expected BODY direction only, never field position.
//! Empty/occluded observations contain no inferred landmark; TTL and safe Stop
//! are the local-world/controller's responsibility.
use crate::road::{Blob, RoadConfig, RoadDetector};
use crate::{Detection, Result};
use image::RgbImage;
use serde::{Deserialize, Serialize};
use std::f64::consts::{FRAC_PI_2, PI};
use xt_stcar_robot_core::autonomy::Point2;
use xt_stcar_robot_core::local_world::{
    ElementColor, ElementFrame, ElementGeometry, ElementKind, ElementObservation, MAX_ELEMENTS,
    ObservationSource,
};
use xt_stcar_robot_core::{FrameId, Timestamp};

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct ExperimentalMarkerPattern {
    pub kind: ElementKind,
    pub bar_count: usize,
    /// Measured transverse white-bar length, used to identify the visual code.
    pub bar_span_m: f64,
    pub bar_thickness_m: f64,
    /// Center-to-center distance along the crossing direction (zero for one).
    pub bar_spacing_m: f64,
    /// Declared region depth, not inferred from lamp color or its image box.
    pub region_depth_m: f64,
    /// Explicit protocol width of the usable subregion. This is independent
    /// of the visual bar span and never inferred from simulator field bounds.
    pub region_width_m: f64,
    /// Pattern center is this distance BEHIND the region's far edge.
    pub anchor_from_far_edge_m: f64,
    pub size_tolerance_m: f64,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct ExperimentalMarkerConfig {
    pub enabled: bool,
    pub simulation_only: bool,
    pub patterns: Vec<ExperimentalMarkerPattern>,
    pub min_pixels: usize,
    pub min_fill: f64,
    pub max_alignment_error_rad: f64,
    pub max_lateral_misalignment_m: f64,
    pub position_error_floor_m: f64,
    pub heading_error_floor_rad: f64,
}
impl Default for ExperimentalMarkerConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            simulation_only: true,
            patterns: Vec::new(),
            min_pixels: 8,
            min_fill: 0.7,
            max_alignment_error_rad: 0.2,
            max_lateral_misalignment_m: 0.06,
            position_error_floor_m: 0.02,
            heading_error_floor_rad: 0.02,
        }
    }
}
impl ExperimentalMarkerConfig {
    /// Explicit opt-in synthetic protocol. Different bar spans prevent an
    /// isolated half of the two-bar finish pattern becoming a stop marker.
    pub fn simulation() -> Self {
        Self {
            enabled: true,
            patterns: vec![
                ExperimentalMarkerPattern {
                    kind: ElementKind::StopLine,
                    bar_count: 1,
                    bar_span_m: 0.6,
                    bar_thickness_m: 0.08,
                    bar_spacing_m: 0.,
                    region_depth_m: 1.,
                    region_width_m: 0.8,
                    anchor_from_far_edge_m: 0.12,
                    size_tolerance_m: 0.06,
                },
                ExperimentalMarkerPattern {
                    kind: ElementKind::FinishMarker,
                    bar_count: 2,
                    bar_span_m: 0.9,
                    bar_thickness_m: 0.08,
                    bar_spacing_m: 0.18,
                    region_depth_m: 0.8,
                    region_width_m: 0.9,
                    anchor_from_far_edge_m: 0.16,
                    size_tolerance_m: 0.06,
                },
            ],
            ..Self::default()
        }
    }
    pub fn validate(&self) -> Result<()> {
        if !self.simulation_only
            || self.patterns.len() > 8
            || (self.enabled && self.patterns.is_empty())
            || !(4..=10000).contains(&self.min_pixels)
            || !between(self.min_fill, 0.5, 1.)
            || !between(self.max_alignment_error_rad, 0.001, 0.4)
            || !between(self.max_lateral_misalignment_m, 0.001, 0.2)
            || !between(self.position_error_floor_m, 0., 0.5)
            || !between(self.heading_error_floor_rad, 0., 0.3)
        {
            return Err("invalid experimental marker bounds or simulation declaration".into());
        }
        for (i, p) in self.patterns.iter().enumerate() {
            if !matches!(p.kind, ElementKind::StopLine | ElementKind::FinishMarker)
                || !(1..=2).contains(&p.bar_count)
                || !between(p.bar_span_m, 0.3, 3.)
                || !between(p.bar_thickness_m, 0.025, 0.2)
                || p.bar_span_m < 4. * p.bar_thickness_m
                || !between(p.region_depth_m, 0.3, 2.)
                || !between(p.region_width_m, 0.3, 3.)
                || p.region_width_m < p.bar_span_m
                || !between(p.size_tolerance_m, 0.001, 0.1)
                || p.size_tolerance_m >= p.bar_span_m / 4.
                || !between(p.anchor_from_far_edge_m, 0., p.region_depth_m)
                || (p.bar_count == 1 && p.bar_spacing_m != 0.)
                || (p.bar_count == 2 && !between(p.bar_spacing_m, p.bar_thickness_m + 0.04, 0.5))
                || p.anchor_from_far_edge_m
                    < (p.bar_count - 1) as f64 * p.bar_spacing_m / 2. + p.bar_thickness_m / 2.
                || p.anchor_from_far_edge_m
                    + (p.bar_count - 1) as f64 * p.bar_spacing_m / 2.
                    + p.bar_thickness_m / 2.
                    > p.region_depth_m
            {
                return Err("invalid experimental marker pattern geometry".into());
            }
            for other in &self.patterns[..i] {
                // Distinct widths also reject incomplete multi-bar patterns.
                if (p.bar_span_m - other.bar_span_m).abs()
                    <= p.size_tolerance_m + other.size_tolerance_m
                {
                    return Err(
                        "experimental marker bar spans must be unambiguous even under occlusion"
                            .into(),
                    );
                }
            }
        }
        Ok(())
    }
}

pub struct GroundMarkerDetector {
    road: RoadDetector,
    config: ExperimentalMarkerConfig,
}
impl GroundMarkerDetector {
    pub fn new(road: RoadConfig, config: ExperimentalMarkerConfig) -> Result<Self> {
        config.validate()?;
        Ok(Self {
            road: RoadDetector::new(road)?,
            config,
        })
    }
    pub fn set_semantics(&mut self, semantics: crate::semantics::SemanticMap) {
        // Marker geometry is its own explicit protocol, not a crosswalk proposal.
        self.road.set_semantics(semantics.lamps_only());
    }
    pub fn config(&self) -> &ExperimentalMarkerConfig {
        &self.config
    }
    pub fn detect(
        &self,
        image: &RgbImage,
        detections: &[Detection],
        at: Timestamp,
        frame_id: FrameId,
        expected_heading_body_rad: f64,
    ) -> Result<ElementFrame> {
        frame_id.validate().map_err(|e| e.to_string())?;
        if !expected_heading_body_rad.is_finite() {
            return Err("expected marker body direction must be finite".into());
        }
        let mut frame = ElementFrame {
            captured_at: at,
            frame_id,
            observations: Vec::new(),
        };
        if !self.config.enabled {
            return Ok(frame);
        }
        let (blobs, w, h) =
            self.road
                .element_ground_components(image, detections, ElementColor::White)?;
        let mut stripes = Vec::new();
        for b in &blobs {
            if b.area < self.config.min_pixels
                || b.major / b.minor < 3.
                || b.area as f64 / (b.major * b.minor) < self.config.min_fill
                || !self.road.element_blob_unclipped(b, w, h)
            {
                continue;
            }
            stripes.push(project_stripe(&self.road, b, w, h)?);
            if stripes.len() > self.road.config().max_candidates {
                return Err("marker candidate limit exceeded".into());
            }
        }
        for pattern in &self.config.patterns {
            let candidates: Vec<_> = stripes
                .iter()
                .filter(|s| {
                    (s.length - pattern.bar_span_m).abs() <= pattern.size_tolerance_m
                        && (s.thickness - pattern.bar_thickness_m).abs() <= pattern.size_tolerance_m
                })
                .collect();
            let mut matched = Vec::<Point2>::new();
            for seed in &candidates {
                let Some(heading) = directed(seed.angle + FRAC_PI_2, expected_heading_body_rad)
                else {
                    continue;
                };
                let (forward, lateral) = axes(heading);
                let mut group: Vec<_> = candidates
                    .iter()
                    .copied()
                    .filter(|s| {
                        axis_difference(s.angle, seed.angle) <= self.config.max_alignment_error_rad
                            && dot(sub(s.center, seed.center), lateral).abs()
                                <= self.config.max_lateral_misalignment_m
                            && dot(sub(s.center, seed.center), forward).abs()
                                <= (pattern.bar_count as f64 * pattern.bar_spacing_m)
                                    .max(pattern.bar_thickness_m * 2.)
                                    + pattern.size_tolerance_m
                    })
                    .collect();
                if group.len() != pattern.bar_count {
                    continue;
                }
                group.sort_by(|a, b| dot(a.center, forward).total_cmp(&dot(b.center, forward)));
                if group.windows(2).any(|p| {
                    (dot(sub(p[1].center, p[0].center), forward) - pattern.bar_spacing_m).abs()
                        > pattern.size_tolerance_m
                }) {
                    continue;
                }
                let anchor = mean(group.iter().map(|s| s.center));
                if matched
                    .iter()
                    .any(|p| p.distance(anchor) < pattern.size_tolerance_m)
                {
                    continue;
                }
                matched.push(anchor);
                let heading = mean_heading(group.iter().map(|s| s.angle + FRAC_PI_2), heading);
                let (forward, _) = axes(heading);
                let pixel_error = group.iter().map(|s| s.pixel_error).fold(0., f64::max);
                let heading_error = self.config.heading_error_floor_rad
                    + (2. * pixel_error / pattern.bar_span_m).atan();
                let offset = pattern.region_depth_m - pattern.anchor_from_far_edge_m;
                frame.observations.push(ElementObservation {
                    kind: pattern.kind,
                    color: ElementColor::White,
                    position_body_m: sub(anchor, scale(forward, offset)),
                    heading_body_rad: Some(heading),
                    geometry: ElementGeometry::LineRegion {
                        lateral_half_width_m: pattern.region_width_m / 2.,
                        depth_m: pattern.region_depth_m,
                    },
                    source: ObservationSource::GroundMarker,
                    confidence: 0.9,
                    position_error_m: self.config.position_error_floor_m
                        + pixel_error
                        + offset * heading_error.sin().abs(),
                    heading_error_rad: heading_error,
                });
                if frame.observations.len() > MAX_ELEMENTS {
                    return Err("marker output limit exceeded".into());
                }
            }
        }
        Ok(frame)
    }
}

#[derive(Clone)]
struct GroundStripe {
    center: Point2,
    angle: f64,
    length: f64,
    thickness: f64,
    corners: [Point2; 4],
    pixel_error: f64,
}
fn project_stripe(road: &RoadDetector, b: &Blob, w: usize, h: usize) -> Result<GroundStripe> {
    let u = b.cx / w as f64;
    let v = b.cy / h as f64;
    let center = road.project(u, v)?;
    // Transform the covariance into metric ground coordinates BEFORE taking
    // its principal axis. Projecting an image PCA axis biases the heading
    // whenever horizontal and vertical pixel scales differ.
    let jx = sub(
        road.project(u + 0.5 / w as f64, v)?,
        road.project(u - 0.5 / w as f64, v)?,
    );
    let jy = sub(
        road.project(u, v + 0.5 / h as f64)?,
        road.project(u, v - 0.5 / h as f64)?,
    );
    let (sin, cos) = b.angle.sin_cos();
    let major_variance = b.major.powi(2) / 12.;
    let minor_variance = b.minor.powi(2) / 12.;
    let xx = major_variance * cos * cos + minor_variance * sin * sin;
    let yy = major_variance * sin * sin + minor_variance * cos * cos;
    let xy = (major_variance - minor_variance) * cos * sin;
    let vx = jx.x_m.powi(2) * xx + 2. * jx.x_m * jy.x_m * xy + jy.x_m.powi(2) * yy;
    let vy = jx.y_m.powi(2) * xx + 2. * jx.y_m * jy.y_m * xy + jy.y_m.powi(2) * yy;
    let covariance =
        jx.x_m * jx.y_m * xx + (jx.x_m * jy.y_m + jy.x_m * jx.y_m) * xy + jy.x_m * jy.y_m * yy;
    let discriminant = (vx - vy).hypot(2. * covariance);
    let angle = 0.5 * (2. * covariance).atan2(vx - vy);
    let length = (6. * (vx + vy + discriminant).max(0.)).sqrt();
    let thickness = (6. * (vx + vy - discriminant).max(0.)).sqrt();
    let (axis, normal) = axes(angle);
    let corner = |along, across| add(center, add(scale(axis, along), scale(normal, across)));
    // Bound the local-affine approximation error at the component's image
    // box corners. Strong perspective distortion is not silently exact.
    let mut distortion: f64 = 0.;
    for x in [b.x0, b.x1] {
        for y in [b.y0, b.y1] {
            let linear = add(
                center,
                add(scale(jx, x as f64 - b.cx), scale(jy, y as f64 - b.cy)),
            );
            distortion = distortion
                .max(linear.distance(road.project(x as f64 / w as f64, y as f64 / h as f64)?));
        }
    }
    Ok(GroundStripe {
        center,
        angle,
        length,
        thickness,
        corners: [
            corner(-length / 2., -thickness / 2.),
            corner(-length / 2., thickness / 2.),
            corner(length / 2., -thickness / 2.),
            corner(length / 2., thickness / 2.),
        ],
        pixel_error: road.element_pixel_error(u, v, w, h)? + distortion,
    })
}

pub(crate) fn detect_crosswalk_element(
    road: &RoadDetector,
    image: &RgbImage,
    detections: &[Detection],
    expected: f64,
) -> Result<Option<ElementObservation>> {
    let (blobs, w, h) = road.element_ground_components(image, detections, ElementColor::White)?;
    let c = &road.config().crosswalk;
    let mut stripes = Vec::new();
    for b in &blobs {
        if b.area < c.min_pixels
            || b.major / b.minor < c.min_aspect
            || b.area as f64 / (b.major * b.minor) < c.min_blob_fill
            || !road.element_blob_unclipped(b, w, h)
            || match c.stripe_axis {
                crate::road::StripeAxis::Either => false,
                crate::road::StripeAxis::ImageX => {
                    axis_difference(b.angle, 0.) > c.angle_tolerance_deg.to_radians()
                }
                crate::road::StripeAxis::ImageY => {
                    axis_difference(b.angle, FRAC_PI_2) > c.angle_tolerance_deg.to_radians()
                }
            }
        {
            continue;
        }
        let stripe = project_stripe(road, b, w, h)?;
        if stripe.length < c.min_depth_m || stripe.length > c.max_depth_m {
            continue;
        }
        stripes.push(stripe);
        if stripes.len() > road.config().max_candidates {
            return Err("oriented crosswalk candidate limit exceeded".into());
        }
    }
    let mut best: Option<ElementObservation> = None;
    for seed in &stripes {
        let Some(heading) = directed(seed.angle, expected) else {
            continue;
        };
        let (forward, lateral) = axes(heading);
        let mut group: Vec<_> = stripes
            .iter()
            .filter(|s| {
                axis_difference(seed.angle, s.angle) <= c.angle_tolerance_deg.to_radians()
                    && (s.length + seed.length) / 2.
                        - dot(sub(s.center, seed.center), forward).abs()
                        >= s.length.min(seed.length) * c.min_overlap
                    && s.length.max(seed.length) / s.length.min(seed.length) <= c.max_length_ratio
                    && s.thickness.max(seed.thickness) / s.thickness.min(seed.thickness)
                        <= c.max_width_ratio
            })
            .collect();
        if group.len() < c.min_stripes {
            continue;
        }
        group.sort_by(|a, b| dot(a.center, lateral).total_cmp(&dot(b.center, lateral)));
        let pitches: Vec<_> = group
            .windows(2)
            .map(|p| dot(sub(p[1].center, p[0].center), lateral))
            .collect();
        let average = pitches.iter().sum::<f64>() / pitches.len() as f64;
        if average <= 0.
            || pitches
                .iter()
                .any(|p| (p - average).abs() / average > c.max_spacing_variation)
            || group.windows(2).zip(&pitches).any(|(pair, pitch)| {
                let thickness = (pair[0].thickness + pair[1].thickness) / 2.;
                let gap = (pitch - thickness) / thickness;
                gap < c.min_gap_ratio || gap > c.max_gap_ratio
            })
        {
            continue;
        }
        let center = mean(group.iter().map(|s| s.center));
        let (mut xx, mut yy, mut xy) = (0., 0., 0.);
        for stripe in &group {
            let delta = sub(stripe.center, center);
            xx += delta.x_m * delta.x_m;
            yy += delta.y_m * delta.y_m;
            xy += delta.x_m * delta.y_m;
        }
        let row_angle = 0.5 * (2. * xy).atan2(xx - yy);
        let Some(heading) = directed(row_angle + FRAC_PI_2, heading) else {
            continue;
        };
        if group
            .iter()
            .any(|s| axis_difference(s.angle, heading) > c.angle_tolerance_deg.to_radians())
        {
            continue;
        }
        let (forward, lateral) = axes(heading);
        let points: Vec<_> = group.iter().flat_map(|s| s.corners).collect();
        let bounds = |axis: Point2| {
            (
                points
                    .iter()
                    .map(|p| dot(*p, axis))
                    .fold(f64::INFINITY, f64::min),
                points
                    .iter()
                    .map(|p| dot(*p, axis))
                    .fold(f64::NEG_INFINITY, f64::max),
            )
        };
        let (near, far) = bounds(forward);
        let (left, right) = bounds(lateral);
        if far - near < c.min_depth_m
            || far - near > c.max_depth_m
            || right - left < c.min_lateral_span_m
        {
            continue;
        }
        let position = add(scale(forward, near), scale(lateral, (left + right) / 2.));
        let error = group.iter().map(|s| s.pixel_error).fold(0., f64::max);
        let observation = ElementObservation {
            kind: ElementKind::Crosswalk,
            color: ElementColor::White,
            position_body_m: position,
            heading_body_rad: Some(heading),
            geometry: ElementGeometry::LineRegion {
                lateral_half_width_m: (right - left) / 2.,
                depth_m: far - near,
            },
            source: ObservationSource::GroundProjection,
            confidence: (1.
                - pitches
                    .iter()
                    .map(|p| (p - average).abs() / average)
                    .sum::<f64>()
                    / pitches.len() as f64)
                * (group.len() as f64 / (c.min_stripes as f64 + 2.)).min(1.),
            position_error_m: 0.02 + error,
            heading_error_rad: 0.02 + (2. * error / (right - left)).atan(),
        };
        if best
            .as_ref()
            .is_none_or(|old| position.x_m < old.position_body_m.x_m)
        {
            best = Some(observation);
        }
    }
    Ok(best)
}

fn between(x: f64, min: f64, max: f64) -> bool {
    x.is_finite() && (min..=max).contains(&x)
}
fn axes(a: f64) -> (Point2, Point2) {
    (
        Point2 {
            x_m: a.cos(),
            y_m: a.sin(),
        },
        Point2 {
            x_m: -a.sin(),
            y_m: a.cos(),
        },
    )
}
fn dot(a: Point2, b: Point2) -> f64 {
    a.x_m * b.x_m + a.y_m * b.y_m
}
fn add(a: Point2, b: Point2) -> Point2 {
    Point2 {
        x_m: a.x_m + b.x_m,
        y_m: a.y_m + b.y_m,
    }
}
fn sub(a: Point2, b: Point2) -> Point2 {
    Point2 {
        x_m: a.x_m - b.x_m,
        y_m: a.y_m - b.y_m,
    }
}
fn scale(a: Point2, s: f64) -> Point2 {
    Point2 {
        x_m: a.x_m * s,
        y_m: a.y_m * s,
    }
}
fn mean(points: impl Iterator<Item = Point2>) -> Point2 {
    let mut sum = Point2 { x_m: 0., y_m: 0. };
    let mut n = 0.;
    for p in points {
        sum = add(sum, p);
        n += 1.;
    }
    scale(sum, 1. / n)
}
fn wrap(a: f64) -> f64 {
    (a + PI).rem_euclid(2. * PI) - PI
}
fn axis_difference(a: f64, b: f64) -> f64 {
    ((a - b + FRAC_PI_2).rem_euclid(PI) - FRAC_PI_2).abs()
}
fn directed(a: f64, expected: f64) -> Option<f64> {
    let mut a = wrap(a);
    let delta = wrap(a - expected);
    if delta.abs() > FRAC_PI_2 {
        a = wrap(a + PI);
    }
    (wrap(a - expected).abs() < 1.4).then_some(a)
}
fn mean_heading(angles: impl Iterator<Item = f64>, expected: f64) -> f64 {
    let mut x = 0.;
    let mut y = 0.;
    for a in angles {
        if let Some(a) = directed(a, expected) {
            x += a.cos();
            y += a.sin();
        }
    }
    y.atan2(x)
}
