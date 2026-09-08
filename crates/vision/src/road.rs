//! Bounded RGB road cues for unverified simulation configurations.
//! No files, devices, timers, YOLO weights, or actuator commands are accessed.
use crate::{Detection, Result};
use image::RgbImage;
use serde::{Deserialize, Serialize};
use xt_stcar_robot_core::autonomy::{CrosswalkObservation, LightState, Point2, RoadObservation};
use xt_stcar_robot_core::{FrameId, Timestamp};

/// All ROIs and homography inputs use image fractions: u=x/width, v=y/height.
pub type Roi = [f64; 4];

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct HomographyConfig {
    pub matrix: [[f64; 3]; 3],
    pub max_forward_m: f64,
    pub max_abs_lateral_m: f64,
    pub min_denominator: f64,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum StripeAxis {
    ImageX,
    ImageY,
    Either,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct LightConfig {
    pub min_saturation: f64,
    pub min_value: f64,
    pub red_hue_range: [f64; 2],
    pub yellow_hue_range: [f64; 2],
    pub green_hue_range: [f64; 2],
    pub min_pixels: usize,
    pub min_roi_fraction: f64,
    pub min_blob_fill: f64,
    pub min_aspect: f64,
    pub max_aspect: f64,
    pub max_white_fraction: f64,
    pub yolo_confidence: f32,
    pub max_rois: usize,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct CrosswalkConfig {
    pub min_value: f64,
    pub max_saturation: f64,
    pub min_pixels: usize,
    pub min_stripes: usize,
    pub stripe_axis: StripeAxis,
    pub angle_tolerance_deg: f64,
    pub min_aspect: f64,
    pub min_blob_fill: f64,
    pub min_overlap: f64,
    pub max_width_ratio: f64,
    pub max_length_ratio: f64,
    pub min_gap_ratio: f64,
    pub max_gap_ratio: f64,
    pub max_spacing_variation: f64,
    pub min_lateral_span_m: f64,
    pub min_depth_m: f64,
    pub max_depth_m: f64,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct ConeConfig {
    pub min_saturation: f64,
    pub min_value: f64,
    pub red_hue_range: [f64; 2],
    pub blue_hue_range: [f64; 2],
    pub min_pixels: usize,
    pub min_height_fraction: f64,
    pub min_height_width_ratio: f64,
    pub max_height_width_ratio: f64,
    pub min_fill: f64,
    pub min_taper_ratio: f64,
    pub merge_distance_m: f64,
    pub max_cones: usize,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct RoadConfig {
    pub schema_version: u32,
    pub simulation_only: bool,
    pub calibration_status: String,
    pub max_work_width: u32,
    pub max_work_height: u32,
    pub max_components: usize,
    pub max_candidates: usize,
    pub ground_roi: Roi,
    pub light_rois: Vec<Roi>,
    pub homography: HomographyConfig,
    pub light: LightConfig,
    pub crosswalk: CrosswalkConfig,
    pub cone: ConeConfig,
}

impl RoadConfig {
    /// Artificial bird's-eye geometry, never a measured camera calibration.
    pub fn simulation() -> Self {
        Self {
            schema_version: 1,
            simulation_only: true,
            calibration_status: "unverified".into(),
            max_work_width: 320,
            max_work_height: 240,
            max_components: 2048,
            max_candidates: 128,
            ground_roi: [0.05, 0.35, 0.95, 0.98],
            light_rois: Vec::new(),
            homography: HomographyConfig {
                matrix: [[0., -5., 5.], [-4., 0., 2.], [0., 0., 1.]],
                max_forward_m: 6.,
                max_abs_lateral_m: 3.,
                min_denominator: 1e-6,
            },
            light: LightConfig {
                min_saturation: 0.45,
                min_value: 0.35,
                red_hue_range: [340., 20.],
                yellow_hue_range: [35., 75.],
                green_hue_range: [80., 170.],
                min_pixels: 4,
                min_roi_fraction: 0.01,
                min_blob_fill: 0.4,
                min_aspect: 0.4,
                max_aspect: 2.5,
                max_white_fraction: 0.7,
                yolo_confidence: 0.25,
                max_rois: 8,
            },
            crosswalk: CrosswalkConfig {
                min_value: 0.75,
                max_saturation: 0.2,
                min_pixels: 8,
                min_stripes: 4,
                stripe_axis: StripeAxis::Either,
                angle_tolerance_deg: 20.,
                // A 0.105m x 0.297m paper is only 4..5 by 7 pixels in a
                // 160x120 view of this synthetic ground map; allow quantization.
                min_aspect: 1.35,
                min_blob_fill: 0.65,
                min_overlap: 0.65,
                max_width_ratio: 1.8,
                max_length_ratio: 1.8,
                min_gap_ratio: 0.25,
                max_gap_ratio: 3.,
                max_spacing_variation: 0.35,
                min_lateral_span_m: 0.5,
                min_depth_m: 0.08,
                max_depth_m: 2.5,
            },
            cone: ConeConfig {
                min_saturation: 0.5,
                min_value: 0.3,
                red_hue_range: [340., 20.],
                blue_hue_range: [190., 255.],
                min_pixels: 10,
                min_height_fraction: 0.03,
                min_height_width_ratio: 1.,
                max_height_width_ratio: 5.,
                min_fill: 0.25,
                min_taper_ratio: 1.15,
                merge_distance_m: 0.15,
                max_cones: 32,
            },
        }
    }
}

fn fraction(v: f64) -> bool {
    v.is_finite() && (0.0..=1.0).contains(&v)
}
fn positive(v: f64) -> bool {
    v.is_finite() && v > 0.0
}
fn valid_roi(r: Roi) -> bool {
    r.iter().all(|&v| fraction(v)) && r[0] < r[2] && r[1] < r[3]
}
fn hue_range(r: [f64; 2]) -> bool {
    r.iter().all(|v| v.is_finite() && (0.0..360.0).contains(v)) && r[0] != r[1]
}
fn in_hue(h: f64, r: [f64; 2]) -> bool {
    if r[0] <= r[1] {
        h >= r[0] && h <= r[1]
    } else {
        h >= r[0] || h <= r[1]
    }
}
fn inside(r: Roi, u: f64, v: f64) -> bool {
    u >= r[0] && u < r[2] && v >= r[1] && v < r[3]
}

#[derive(Clone, Copy)]
struct Hsv {
    h: f64,
    s: f64,
    v: f64,
}
fn hsv(rgb: [u8; 3]) -> Hsv {
    let [r, g, b] = rgb.map(|v| f64::from(v) / 255.);
    let max = r.max(g).max(b);
    let min = r.min(g).min(b);
    let delta = max - min;
    let h = if delta == 0. {
        0.
    } else if max == r {
        60. * ((g - b) / delta).rem_euclid(6.)
    } else if max == g {
        60. * ((b - r) / delta + 2.)
    } else {
        60. * ((r - g) / delta + 4.)
    };
    Hsv {
        h,
        s: if max == 0. { 0. } else { delta / max },
        v: max,
    }
}

#[derive(Clone)]
struct Blob {
    area: usize,
    x0: usize,
    y0: usize,
    x1: usize,
    y1: usize,
    cx: f64,
    cy: f64,
    angle: f64,
    major: f64,
    minor: f64,
    top_width: f64,
    bottom_width: f64,
}
impl Blob {
    fn width(&self) -> f64 {
        (self.x1 - self.x0) as f64
    }
    fn height(&self) -> f64 {
        (self.y1 - self.y0) as f64
    }
    fn fill(&self) -> f64 {
        self.area as f64 / (self.width() * self.height())
    }
}

fn components(mut mask: Vec<bool>, width: usize, height: usize, max: usize) -> Result<Vec<Blob>> {
    let mut blobs = Vec::new();
    let mut pixels = Vec::new();
    for start in 0..mask.len() {
        if !mask[start] {
            continue;
        }
        if blobs.len() >= max {
            return Err("road component limit exceeded".into());
        }
        pixels.clear();
        pixels.push(start);
        mask[start] = false;
        let mut cursor = 0;
        let (mut x0, mut y0, mut x1, mut y1) = (width, height, 0, 0);
        let (mut sx, mut sy, mut sxx, mut syy, mut sxy) = (0., 0., 0., 0., 0.);
        while cursor < pixels.len() {
            let i = pixels[cursor];
            cursor += 1;
            let x = i % width;
            let y = i / width;
            x0 = x0.min(x);
            y0 = y0.min(y);
            x1 = x1.max(x + 1);
            y1 = y1.max(y + 1);
            let xf = x as f64 + 0.5;
            let yf = y as f64 + 0.5;
            sx += xf;
            sy += yf;
            sxx += xf * xf;
            syy += yf * yf;
            sxy += xf * yf;
            let neighbors = [
                (x > 0).then(|| i - 1),
                (x + 1 < width).then_some(i + 1),
                (y > 0).then(|| i - width),
                (y + 1 < height).then_some(i + width),
            ];
            for n in neighbors.into_iter().flatten() {
                if mask[n] {
                    mask[n] = false;
                    pixels.push(n);
                }
            }
        }
        let n = pixels.len() as f64;
        let cx = sx / n;
        let cy = sy / n;
        let vx = (sxx / n - cx * cx).max(0.);
        let vy = (syy / n - cy * cy).max(0.);
        let covariance = sxy / n - cx * cy;
        let discriminant = (vx - vy).hypot(2. * covariance);
        let major = (12. * ((vx + vy + discriminant) / 2.).max(0.) + 1.).sqrt();
        let minor = (12. * ((vx + vy - discriminant) / 2.).max(0.) + 1.).sqrt();
        let band = ((y1 - y0) / 3).max(1);
        let top = pixels.iter().filter(|&&i| i / width < y0 + band).count() as f64 / band as f64;
        let bottom =
            pixels.iter().filter(|&&i| i / width >= y1 - band).count() as f64 / band as f64;
        blobs.push(Blob {
            area: pixels.len(),
            x0,
            y0,
            x1,
            y1,
            cx,
            cy,
            angle: 0.5 * (2. * covariance).atan2(vx - vy),
            major,
            minor,
            top_width: top,
            bottom_width: bottom,
        });
    }
    Ok(blobs)
}

pub struct RoadDetector {
    config: RoadConfig,
}
impl RoadDetector {
    pub fn new(config: RoadConfig) -> Result<Self> {
        let l = &config.light;
        let c = &config.crosswalk;
        let cone = &config.cone;
        let h = &config.homography;
        if config.schema_version != 1
            || !config.simulation_only
            || config.calibration_status != "unverified"
        {
            return Err("road detector requires schema 1, simulation_only=true, calibration_status=unverified".into());
        }
        if !(32..=640).contains(&config.max_work_width)
            || !(32..=480).contains(&config.max_work_height)
            || !(1..=4096).contains(&config.max_components)
            || !(4..=256).contains(&config.max_candidates)
            || !valid_roi(config.ground_roi)
            || config.light_rois.iter().any(|&r| !valid_roi(r))
            || config.light_rois.len() > l.max_rois
            || !(1..=16).contains(&l.max_rois)
        {
            return Err("invalid road image, component, candidate or ROI limits".into());
        }
        if ![
            l.min_saturation,
            l.min_value,
            l.min_roi_fraction,
            l.min_blob_fill,
            l.max_white_fraction,
        ]
        .into_iter()
        .all(fraction)
            || l.min_pixels == 0
            || l.min_pixels > 10000
            || !positive(l.min_aspect)
            || l.max_aspect < l.min_aspect
            || !l.max_aspect.is_finite()
            || ![l.red_hue_range, l.yellow_hue_range, l.green_hue_range]
                .into_iter()
                .all(hue_range)
            || !l.yolo_confidence.is_finite()
            || !(0.0..=1.0).contains(&l.yolo_confidence)
        {
            return Err("invalid light HSV or geometry thresholds".into());
        }
        if ![
            c.min_value,
            c.max_saturation,
            c.min_blob_fill,
            c.min_overlap,
            c.max_spacing_variation,
        ]
        .into_iter()
        .all(fraction)
            || c.min_pixels == 0
            || c.min_pixels > 10000
            || !(3..=32).contains(&c.min_stripes)
            || c.min_stripes > config.max_candidates
            || !positive(c.angle_tolerance_deg)
            || c.angle_tolerance_deg > 45.
            || !positive(c.min_aspect)
            || c.min_aspect < 1.2
            || !positive(c.max_width_ratio)
            || c.max_width_ratio < 1.
            || !positive(c.max_length_ratio)
            || c.max_length_ratio < 1.
            || !positive(c.min_gap_ratio)
            || !c.max_gap_ratio.is_finite()
            || c.max_gap_ratio < c.min_gap_ratio
            || !positive(c.min_lateral_span_m)
            || !positive(c.min_depth_m)
            || !c.max_depth_m.is_finite()
            || c.max_depth_m < c.min_depth_m
        {
            return Err("invalid crosswalk stripe geometry thresholds".into());
        }
        if ![
            cone.min_saturation,
            cone.min_value,
            cone.min_height_fraction,
            cone.min_fill,
        ]
        .into_iter()
        .all(fraction)
            || ![cone.red_hue_range, cone.blue_hue_range]
                .into_iter()
                .all(hue_range)
            || cone.min_pixels == 0
            || cone.min_pixels > 10000
            || !positive(cone.min_height_width_ratio)
            || !cone.max_height_width_ratio.is_finite()
            || cone.max_height_width_ratio < cone.min_height_width_ratio
            || !positive(cone.min_taper_ratio)
            || cone.min_taper_ratio < 1.
            || !positive(cone.merge_distance_m)
            || !(1..=128).contains(&cone.max_cones)
        {
            return Err("invalid cone HSV or geometry thresholds".into());
        }
        if !h
            .matrix
            .iter()
            .flatten()
            .all(|v| v.is_finite() && v.abs() <= 1e6)
            || !positive(h.max_forward_m)
            || h.max_forward_m > 1000.
            || !positive(h.max_abs_lateral_m)
            || h.max_abs_lateral_m > 1000.
            || !positive(h.min_denominator)
            || h.min_denominator > 1.
        {
            return Err("invalid ground homography bounds".into());
        }
        let m = h.matrix;
        let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
        if det.abs() < 1e-10 {
            return Err("singular ground homography".into());
        }
        let detector = Self { config };
        let [left, top, right, bottom] = detector.config.ground_roi;
        let corners = [(left, top), (right, top), (left, bottom), (right, bottom)];
        let denominators = corners.map(|(u, v)| m[2][0] * u + m[2][1] * v + m[2][2]);
        if denominators
            .iter()
            .any(|d| d.signum() != denominators[0].signum())
        {
            return Err("ground homography horizon crosses the ROI".into());
        }
        let [tl, tr, bl, br] = corners
            .map(|(u, v)| detector.project(u, v))
            .into_iter()
            .collect::<Result<Vec<_>>>()?
            .try_into()
            .map_err(|_| "invalid projection corners")?;
        if tl.x_m + tr.x_m <= bl.x_m + br.x_m + 1e-6 || tl.y_m + bl.y_m <= tr.y_m + br.y_m + 1e-6 {
            return Err("ground homography must consistently map image up to body forward and image left to body left".into());
        }
        Ok(detector)
    }

    pub fn config(&self) -> &RoadConfig {
        &self.config
    }

    fn project(&self, u: f64, v: f64) -> Result<Point2> {
        let h = &self.config.homography;
        let m = h.matrix;
        let d = m[2][0] * u + m[2][1] * v + m[2][2];
        if !d.is_finite() || d.abs() < h.min_denominator {
            return Err("ground projection is near the horizon".into());
        }
        let point = Point2 {
            x_m: (m[0][0] * u + m[0][1] * v + m[0][2]) / d,
            y_m: (m[1][0] * u + m[1][1] * v + m[1][2]) / d,
        };
        if !point.x_m.is_finite()
            || !point.y_m.is_finite()
            || point.x_m < 0.
            || point.x_m > h.max_forward_m
            || point.y_m.abs() > h.max_abs_lateral_m
        {
            return Err("ground projection exceeds the configured body-coordinate envelope".into());
        }
        Ok(point)
    }

    pub fn detect(
        &self,
        image: &RgbImage,
        detections: &[Detection],
        at: Timestamp,
        frame_id: FrameId,
    ) -> Result<RoadObservation> {
        frame_id.validate().map_err(|e| e.to_string())?;
        if image.width() == 0
            || image.height() == 0
            || u64::from(image.width()) * u64::from(image.height()) > 64_000_000
            || detections.len() > 300
        {
            return Err(
                "road input must be positive, at most 64MP, with at most 300 YOLO detections"
                    .into(),
            );
        }
        let scale = (f64::from(self.config.max_work_width) / f64::from(image.width()))
            .min(f64::from(self.config.max_work_height) / f64::from(image.height()))
            .min(1.);
        let width = (f64::from(image.width()) * scale).floor() as usize;
        let height = (f64::from(image.height()) * scale).floor() as usize;
        if width < 2 || height < 2 {
            return Err("road input aspect ratio is too extreme".into());
        }
        let mut colors = Vec::with_capacity(width * height);
        for y in 0..height {
            for x in 0..width {
                let sx = (((x as f64 + 0.5) * f64::from(image.width()) / width as f64).floor()
                    as u32)
                    .min(image.width() - 1);
                let sy = (((y as f64 + 0.5) * f64::from(image.height()) / height as f64).floor()
                    as u32)
                    .min(image.height() - 1);
                colors.push(hsv(image.get_pixel(sx, sy).0));
            }
        }
        let mut rois = Vec::new();
        let mut traffic_proposal = false;
        for detection in detections
            .iter()
            .filter(|d| d.class_id == 9 && d.confidence > self.config.light.yolo_confidence)
        {
            traffic_proposal = true;
            let [x0, y0, x1, y1] = detection.xyxy;
            if !detection.confidence.is_finite()
                || detection.confidence > 1.
                || !detection.xyxy.iter().all(|v| v.is_finite())
                || x1 <= x0
                || y1 <= y0
            {
                continue;
            }
            let r = [
                (f64::from(x0) / f64::from(image.width())).clamp(0., 1.),
                (f64::from(y0) / f64::from(image.height())).clamp(0., 1.),
                (f64::from(x1) / f64::from(image.width())).clamp(0., 1.),
                (f64::from(y1) / f64::from(image.height())).clamp(0., 1.),
            ];
            if valid_roi(r) {
                rois.push(r);
            }
            if rois.len() > self.config.light.max_rois {
                return Err("traffic-light ROI limit exceeded".into());
            }
        }
        // An empty/outside YOLO lamp crop is Unknown. It must not silently
        // switch to a configured ROI that may depict a different lamp.
        if !traffic_proposal {
            rois.clone_from(&self.config.light_rois);
        }
        let (light, light_confidence) = self.detect_light(&colors, width, height, &rois)?;
        let mut exclusions = self.config.light_rois.clone();
        exclusions.extend(rois);
        let ground_mask = |predicate: &dyn Fn(Hsv) -> bool| -> Vec<bool> {
            colors
                .iter()
                .enumerate()
                .map(|(i, &color)| {
                    let u = (i % width) as f64 / width as f64 + 0.5 / width as f64;
                    let v = (i / width) as f64 / height as f64 + 0.5 / height as f64;
                    inside(self.config.ground_roi, u, v)
                        && !exclusions.iter().any(|&r| inside(r, u, v))
                        && predicate(color)
                })
                .collect()
        };
        let c = &self.config.crosswalk;
        let white = components(
            ground_mask(&|p| p.s <= c.max_saturation && p.v >= c.min_value),
            width,
            height,
            self.config.max_components,
        )?;
        let crosswalk = self.detect_crosswalk(&white, width, height)?;
        let cone = &self.config.cone;
        let colored = components(
            ground_mask(&|p| {
                p.s >= cone.min_saturation
                    && p.v >= cone.min_value
                    && (in_hue(p.h, cone.red_hue_range) || in_hue(p.h, cone.blue_hue_range))
            }),
            width,
            height,
            self.config.max_components,
        )?;
        let mut cones_body_m: Vec<Point2> = Vec::new();
        let mut candidates = 0;
        for blob in colored {
            let aspect = blob.height() / blob.width();
            if blob.area < cone.min_pixels
                || blob.height() / (height as f64) < cone.min_height_fraction
                || aspect < cone.min_height_width_ratio
                || aspect > cone.max_height_width_ratio
                || blob.fill() < cone.min_fill
                || blob.top_width <= 0.
                || blob.bottom_width / blob.top_width < cone.min_taper_ratio
            {
                continue;
            }
            candidates += 1;
            if candidates > self.config.max_candidates {
                return Err("cone candidate limit exceeded".into());
            }
            let foot = self.project(blob.cx / width as f64, blob.y1 as f64 / height as f64)?;
            if !cones_body_m
                .iter()
                .any(|old| old.distance(foot) < cone.merge_distance_m)
            {
                cones_body_m.push(foot);
            }
            if cones_body_m.len() > cone.max_cones {
                return Err("cone output limit exceeded".into());
            }
        }
        cones_body_m.sort_by(|a, b| a.x_m.total_cmp(&b.x_m).then(a.y_m.total_cmp(&b.y_m)));
        Ok(RoadObservation {
            captured_at: at,
            frame_id,
            crosswalk,
            light,
            light_confidence,
            cones_body_m,
        })
    }

    fn detect_light(
        &self,
        colors: &[Hsv],
        width: usize,
        height: usize,
        rois: &[Roi],
    ) -> Result<(LightState, f64)> {
        let c = &self.config.light;
        let mut confidence = [0f64; 3];
        for &roi in rois {
            let in_roi: Vec<bool> = (0..colors.len())
                .map(|i| {
                    inside(
                        roi,
                        (i % width) as f64 / width as f64 + 0.5 / width as f64,
                        (i / width) as f64 / height as f64 + 0.5 / height as f64,
                    )
                })
                .collect();
            let area = in_roi.iter().filter(|&&v| v).count();
            if area == 0 {
                continue;
            }
            let white = colors
                .iter()
                .zip(&in_roi)
                .filter(|&(p, inside)| *inside && p.s < 0.15 && p.v > 0.95)
                .count();
            if white as f64 / (area as f64) > c.max_white_fraction {
                continue;
            }
            for (color, range) in [c.red_hue_range, c.yellow_hue_range, c.green_hue_range]
                .into_iter()
                .enumerate()
            {
                let mask = colors
                    .iter()
                    .zip(&in_roi)
                    .map(|(p, &inside)| {
                        inside
                            && p.s >= c.min_saturation
                            && p.v >= c.min_value
                            && in_hue(p.h, range)
                    })
                    .collect();
                for blob in components(mask, width, height, self.config.max_components)? {
                    let aspect = blob.width() / blob.height();
                    if blob.area < c.min_pixels
                        || blob.area as f64 / (area as f64) < c.min_roi_fraction
                        || blob.fill() < c.min_blob_fill
                        || aspect < c.min_aspect
                        || aspect > c.max_aspect
                    {
                        continue;
                    }
                    let support = (blob.area as f64
                        / (area as f64 * c.min_roi_fraction.max(1e-6) * 4.))
                        .min(1.);
                    confidence[color] =
                        confidence[color].max((support * blob.fill()).sqrt().clamp(0., 1.));
                }
            }
        }
        let active: Vec<_> = confidence
            .iter()
            .enumerate()
            .filter(|(_, v)| **v > 0.)
            .collect();
        Ok(match active.as_slice() {
            [] => (LightState::Unknown, 0.),
            [(index, value)] => (
                [LightState::Red, LightState::Yellow, LightState::Green][*index],
                **value,
            ),
            _ => (
                LightState::Conflicting,
                active.iter().map(|(_, v)| **v).fold(1., f64::min),
            ),
        })
    }

    fn detect_crosswalk(
        &self,
        blobs: &[Blob],
        width: usize,
        height: usize,
    ) -> Result<Option<CrosswalkObservation>> {
        let c = &self.config.crosswalk;
        let tolerance = c.angle_tolerance_deg.to_radians();
        let angle_distance = |a: f64, b: f64| {
            ((a - b + std::f64::consts::FRAC_PI_2).rem_euclid(std::f64::consts::PI)
                - std::f64::consts::FRAC_PI_2)
                .abs()
        };
        let candidates: Vec<_> = blobs
            .iter()
            .filter(|b| {
                b.area >= c.min_pixels
                    && b.major / b.minor >= c.min_aspect
                    && b.area as f64 / (b.major * b.minor) >= c.min_blob_fill
                    && match c.stripe_axis {
                        StripeAxis::Either => true,
                        StripeAxis::ImageX => angle_distance(b.angle, 0.) <= tolerance,
                        StripeAxis::ImageY => {
                            angle_distance(b.angle, std::f64::consts::FRAC_PI_2) <= tolerance
                        }
                    }
            })
            .collect();
        if candidates.len() > self.config.max_candidates {
            return Err("crosswalk candidate limit exceeded".into());
        }
        let mut best: Option<CrosswalkObservation> = None;
        for seed in &candidates {
            let axis = (seed.angle.cos(), seed.angle.sin());
            let normal = (-axis.1, axis.0);
            let mut group: Vec<_> = candidates
                .iter()
                .copied()
                .filter(|b| {
                    let offset = ((b.cx - seed.cx) * axis.0 + (b.cy - seed.cy) * axis.1).abs();
                    angle_distance(b.angle, seed.angle) <= tolerance
                        && (b.major + seed.major) / 2. - offset
                            >= b.major.min(seed.major) * c.min_overlap
                        && b.major.max(seed.major) / b.major.min(seed.major) <= c.max_length_ratio
                        && b.minor.max(seed.minor) / b.minor.min(seed.minor) <= c.max_width_ratio
                })
                .collect();
            if group.len() < c.min_stripes {
                continue;
            }
            group.sort_by(|a, b| {
                (a.cx * normal.0 + a.cy * normal.1).total_cmp(&(b.cx * normal.0 + b.cy * normal.1))
            });
            let pitches: Vec<_> = group
                .windows(2)
                .map(|pair| {
                    ((pair[1].cx - pair[0].cx) * normal.0 + (pair[1].cy - pair[0].cy) * normal.1)
                        .abs()
                })
                .collect();
            let average = pitches.iter().sum::<f64>() / pitches.len() as f64;
            if average <= 0.
                || pitches
                    .iter()
                    .any(|pitch| (pitch - average).abs() / average > c.max_spacing_variation)
                || group.windows(2).zip(&pitches).any(|(pair, pitch)| {
                    let thickness = (pair[0].minor + pair[1].minor) / 2.;
                    let gap = (pitch - thickness) / thickness;
                    gap < c.min_gap_ratio || gap > c.max_gap_ratio
                })
            {
                continue;
            }
            let x0 = group.iter().map(|b| b.x0).min().unwrap();
            let x1 = group.iter().map(|b| b.x1).max().unwrap();
            let y0 = group.iter().map(|b| b.y0).min().unwrap();
            let y1 = group.iter().map(|b| b.y1).max().unwrap();
            let points = [(x0, y0), (x1, y0), (x0, y1), (x1, y1)]
                .map(|(x, y)| self.project(x as f64 / width as f64, y as f64 / height as f64))
                .into_iter()
                .collect::<Result<Vec<_>>>()?;
            let near = points.iter().map(|p| p.x_m).fold(f64::INFINITY, f64::min);
            let far = points
                .iter()
                .map(|p| p.x_m)
                .fold(f64::NEG_INFINITY, f64::max);
            let left = points.iter().map(|p| p.y_m).fold(f64::INFINITY, f64::min);
            let right = points
                .iter()
                .map(|p| p.y_m)
                .fold(f64::NEG_INFINITY, f64::max);
            if far - near < c.min_depth_m
                || far - near > c.max_depth_m
                || right - left < c.min_lateral_span_m
            {
                continue;
            }
            let regularity = 1.
                - pitches
                    .iter()
                    .map(|p| (p - average).abs() / average)
                    .sum::<f64>()
                    / pitches.len() as f64;
            let confidence = (regularity
                * (group.len() as f64 / (c.min_stripes as f64 + 2.)).min(1.))
            .clamp(0., 1.);
            if best.as_ref().is_none_or(|old| near < old.near_edge_m) {
                best = Some(CrosswalkObservation {
                    near_edge_m: near,
                    far_edge_m: far,
                    lateral_min_m: left,
                    lateral_max_m: right,
                    confidence,
                });
            }
        }
        Ok(best)
    }
}
