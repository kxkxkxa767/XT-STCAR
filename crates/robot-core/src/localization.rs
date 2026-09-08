//! Bounded planar scan-to-scan point-to-point ICP.
//!
//! Input points must already be expressed in the vehicle body frame, with a
//! single acquisition timestamp. The initial pose is an external anchor; this
//! estimator does not recover absolute position or manufacture wheel odometry.
//! Scan distortion, changing surfaces and local minima remain real limitations.
//! ICP reference: Besl and McKay, IEEE TPAMI 14(2), 1992, DOI 10.1109/34.121791:
//! <https://graphics.stanford.edu/courses/cs233-25-spring/ReferencedPapers/paper_icp.pdf>
use crate::autonomy::{Point2, Pose2, PoseEstimate};
use crate::{FrameId, Timestamp, ValidationError};
use serde::{Deserialize, Serialize};

const MAX_INPUT_POINTS: usize = 8192;

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct LocalizationConfig {
    pub frame_id: FrameId,
    pub max_points: usize,
    pub min_points: usize,
    pub max_iterations: usize,
    pub max_pair_distance_m: f64,
    pub min_overlap_ratio: f64,
    pub max_rmse_m: f64,
    pub min_axis_variance_m2: f64,
    pub max_translation_per_scan_m: f64,
    pub max_rotation_per_scan_rad: f64,
    pub max_speed_mps: f64,
    pub max_yaw_rate_radps: f64,
    pub max_scan_gap_ms: u64,
}

impl LocalizationConfig {
    /// Synthetic test values; these require real scan/TF/timing calibration.
    pub fn simulation(frame_id: FrameId) -> Self {
        Self {
            frame_id,
            max_points: 256,
            min_points: 16,
            max_iterations: 24,
            max_pair_distance_m: 0.3,
            min_overlap_ratio: 0.55,
            max_rmse_m: 0.05,
            min_axis_variance_m2: 0.005,
            max_translation_per_scan_m: 0.2,
            max_rotation_per_scan_rad: 0.3,
            max_speed_mps: 1.0,
            max_yaw_rate_radps: 3.0,
            max_scan_gap_ms: 250,
        }
    }

    pub fn validate(&self) -> Result<(), ValidationError> {
        self.frame_id.validate()?;
        if !(6..=512).contains(&self.max_points)
            || !(6..=self.max_points).contains(&self.min_points)
            || !(1..=40).contains(&self.max_iterations)
            || !self.max_pair_distance_m.is_finite()
            || !(0.001..=2.0).contains(&self.max_pair_distance_m)
            || !self.min_overlap_ratio.is_finite()
            || !(0.3..=1.0).contains(&self.min_overlap_ratio)
            || !self.max_rmse_m.is_finite()
            || !(0.00001..=self.max_pair_distance_m).contains(&self.max_rmse_m)
            || !self.min_axis_variance_m2.is_finite()
            || !(0.000001..=1.0).contains(&self.min_axis_variance_m2)
            || !self.max_translation_per_scan_m.is_finite()
            || !(0.001..=2.0).contains(&self.max_translation_per_scan_m)
            || !self.max_rotation_per_scan_rad.is_finite()
            || !(0.001..=0.8).contains(&self.max_rotation_per_scan_rad)
            || !self.max_speed_mps.is_finite()
            || !(0.01..=5.0).contains(&self.max_speed_mps)
            || !self.max_yaw_rate_radps.is_finite()
            || !(0.01..=10.0).contains(&self.max_yaw_rate_radps)
            || !(10..=2000).contains(&self.max_scan_gap_ms)
        {
            return Err(ValidationError(
                "invalid or excessive scan odometry limits".into(),
            ));
        }
        Ok(())
    }
}

#[derive(Clone, Debug, Serialize)]
pub struct LocalizationUpdate {
    pub estimate: Option<PoseEstimate>,
    pub accepted: bool,
    pub reason: Option<String>,
    pub matched_points: usize,
    pub rmse_m: Option<f64>,
}
impl LocalizationUpdate {
    fn rejected(reason: &str, matched_points: usize, rmse_m: Option<f64>) -> Self {
        Self {
            estimate: None,
            accepted: false,
            reason: Some(reason.into()),
            matched_points,
            rmse_m,
        }
    }
}

pub struct ScanOdometry {
    config: LocalizationConfig,
    pose: Pose2,
    previous: Option<(Timestamp, Vec<Point2>)>,
    last_received: Option<Timestamp>,
    lost: bool,
}

impl ScanOdometry {
    pub fn new(config: LocalizationConfig, initial_pose: Pose2) -> Result<Self, ValidationError> {
        config.validate()?;
        validate_anchor(initial_pose)?;
        Ok(Self {
            config,
            pose: initial_pose,
            previous: None,
            last_received: None,
            lost: false,
        })
    }

    pub fn config(&self) -> &LocalizationConfig {
        &self.config
    }

    /// The caller must supply an externally known anchor after a scan gap.
    /// A reset produces no pose estimate; two valid scans are required again.
    pub fn reset(&mut self, initial_pose: Pose2) -> Result<(), ValidationError> {
        validate_anchor(initial_pose)?;
        self.pose = initial_pose;
        self.previous = None;
        self.last_received = None;
        self.lost = false;
        Ok(())
    }

    pub fn update(
        &mut self,
        at: Timestamp,
        points: &[Point2],
    ) -> Result<LocalizationUpdate, ValidationError> {
        if self.last_received.is_some_and(|old| at <= old) {
            return Err(ValidationError(
                "scan odometry timestamp must strictly increase".into(),
            ));
        }
        if points.len() > MAX_INPUT_POINTS
            || points
                .iter()
                .any(|p| !p.valid() || p.x_m.abs() > 1000.0 || p.y_m.abs() > 1000.0)
        {
            return Err(ValidationError(
                "invalid or excessive scan point cloud".into(),
            ));
        }
        self.last_received = Some(at);
        if self.lost {
            return Ok(LocalizationUpdate::rejected(
                "lost_requires_explicit_reset",
                0,
                None,
            ));
        }
        if self
            .previous
            .as_ref()
            .is_some_and(|(old, _)| at.0 - old.0 >= self.config.max_scan_gap_ms)
        {
            self.lost = true;
            return Ok(LocalizationUpdate::rejected(
                "scan_gap_requires_explicit_reset",
                0,
                None,
            ));
        }
        if points.len() < self.config.min_points {
            return Ok(LocalizationUpdate::rejected("too_few_points", 0, None));
        }
        let count = points.len().min(self.config.max_points);
        let current: Vec<_> = (0..count)
            .map(|i| points[i * points.len() / count])
            .collect();
        if !geometry_supported(&current, self.config.min_axis_variance_m2) {
            return Ok(LocalizationUpdate::rejected("degenerate_geometry", 0, None));
        }
        let Some((previous_at, previous)) = &self.previous else {
            self.previous = Some((at, current));
            return Ok(LocalizationUpdate::rejected(
                "reference_initialized",
                0,
                None,
            ));
        };
        // T maps the current body frame into the previous body frame.
        let mut transform = Pose2::default();
        for _ in 0..self.config.max_iterations {
            let pairs = correspondences(
                &current,
                previous,
                transform,
                self.config.max_pair_distance_m,
            );
            if pairs.len() < self.config.min_points {
                return Ok(LocalizationUpdate::rejected(
                    "insufficient_correspondences",
                    pairs.len(),
                    None,
                ));
            }
            let Some(delta) = fit_rigid(&pairs, self.config.min_axis_variance_m2) else {
                return Ok(LocalizationUpdate::rejected(
                    "degenerate_correspondences",
                    pairs.len(),
                    None,
                ));
            };
            let translation = delta.body_to_world(transform.point());
            transform = Pose2 {
                x_m: translation.x_m,
                y_m: translation.y_m,
                yaw_rad: wrap_angle(delta.yaw_rad + transform.yaw_rad),
            };
            if delta.point().distance(Point2::default()) < 1e-6 && delta.yaw_rad.abs() < 1e-6 {
                break;
            }
        }
        let pairs = correspondences(
            &current,
            previous,
            transform,
            self.config.max_pair_distance_m,
        );
        let overlap = pairs.len() as f64 / current.len().max(previous.len()) as f64;
        if pairs.len() < self.config.min_points || overlap < self.config.min_overlap_ratio {
            return Ok(LocalizationUpdate::rejected(
                "insufficient_overlap",
                pairs.len(),
                None,
            ));
        }
        if fit_rigid(&pairs, self.config.min_axis_variance_m2).is_none() {
            return Ok(LocalizationUpdate::rejected(
                "degenerate_correspondences",
                pairs.len(),
                None,
            ));
        }
        let rmse = (pairs
            .iter()
            .map(|p| p.source.distance(p.target).powi(2))
            .sum::<f64>()
            / pairs.len() as f64)
            .sqrt();
        if !rmse.is_finite() || rmse > self.config.max_rmse_m {
            return Ok(LocalizationUpdate::rejected(
                "residual_too_large",
                pairs.len(),
                Some(rmse),
            ));
        }
        let dt = (at.0 - previous_at.0) as f64 / 1000.0;
        let translation = transform.point().distance(Point2::default());
        let yaw_rate = transform.yaw_rad / dt;
        if translation > self.config.max_translation_per_scan_m
            || transform.yaw_rad.abs() > self.config.max_rotation_per_scan_rad
            || translation / dt > self.config.max_speed_mps
            || yaw_rate.abs() > self.config.max_yaw_rate_radps
        {
            return Ok(LocalizationUpdate::rejected(
                "motion_jump_exceeds_limits",
                pairs.len(),
                Some(rmse),
            ));
        }
        let world = self.pose.body_to_world(transform.point());
        let pose = Pose2 {
            x_m: world.x_m,
            y_m: world.y_m,
            yaw_rad: wrap_angle(self.pose.yaw_rad + transform.yaw_rad),
        };
        validate_anchor(pose)?;
        // A heuristic registration score, not covariance or a probability.
        let quality = overlap * (1.0 - rmse / self.config.max_rmse_m).clamp(0.0, 1.0);
        let estimate = PoseEstimate {
            captured_at: at,
            frame_id: self.config.frame_id.clone(),
            pose,
            speed_mps: translation / dt * if transform.x_m < 0.0 { -1.0 } else { 1.0 },
            yaw_rate_radps: yaw_rate,
            quality,
        };
        self.pose = pose;
        self.previous = Some((at, current));
        Ok(LocalizationUpdate {
            estimate: Some(estimate),
            accepted: true,
            reason: None,
            matched_points: pairs.len(),
            rmse_m: Some(rmse),
        })
    }
}

fn validate_anchor(pose: Pose2) -> Result<(), ValidationError> {
    if !pose.valid() || pose.x_m.abs() > 1_000_000.0 || pose.y_m.abs() > 1_000_000.0 {
        Err(ValidationError("invalid scan odometry anchor".into()))
    } else {
        Ok(())
    }
}

fn wrap_angle(angle: f64) -> f64 {
    (angle + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

#[derive(Clone, Copy)]
struct Pair {
    source: Point2,
    target: Point2,
    squared_error: f64,
}

/// Retain only the closest source for each target to prevent duplicated points
/// from manufacturing an apparently high overlap ratio. Allocation is bounded.
fn correspondences(
    source: &[Point2],
    target: &[Point2],
    transform: Pose2,
    limit: f64,
) -> Vec<Pair> {
    let mut assigned: Vec<Option<Pair>> = vec![None; target.len()];
    for point in source {
        let transformed = transform.body_to_world(*point);
        let mut nearest = None;
        let mut best_squared = limit * limit;
        for (index, candidate) in target.iter().enumerate() {
            let squared = (transformed.x_m - candidate.x_m).powi(2)
                + (transformed.y_m - candidate.y_m).powi(2);
            if squared < best_squared {
                nearest = Some(index);
                best_squared = squared;
            }
        }
        if let Some(index) = nearest
            && assigned[index].is_none_or(|old| best_squared < old.squared_error)
        {
            assigned[index] = Some(Pair {
                source: transformed,
                target: target[index],
                squared_error: best_squared,
            });
        }
    }
    assigned.into_iter().flatten().collect()
}

fn geometry_supported(points: &[Point2], minimum_variance: f64) -> bool {
    if points.is_empty() {
        return false;
    }
    let n = points.len() as f64;
    let mean_x = points.iter().map(|p| p.x_m).sum::<f64>() / n;
    let mean_y = points.iter().map(|p| p.y_m).sum::<f64>() / n;
    let (mut xx, mut yy, mut xy) = (0.0, 0.0, 0.0);
    for p in points {
        let x = p.x_m - mean_x;
        let y = p.y_m - mean_y;
        xx += x * x;
        yy += y * y;
        xy += x * y;
    }
    let minimum = ((xx + yy) - (xx - yy).hypot(2.0 * xy)) / (2.0 * n);
    minimum.is_finite() && minimum >= minimum_variance
}

fn fit_rigid(pairs: &[Pair], minimum_variance: f64) -> Option<Pose2> {
    let source: Vec<_> = pairs.iter().map(|p| p.source).collect();
    let target: Vec<_> = pairs.iter().map(|p| p.target).collect();
    if !geometry_supported(&source, minimum_variance)
        || !geometry_supported(&target, minimum_variance)
    {
        return None;
    }
    let n = pairs.len() as f64;
    let source_mean = Point2 {
        x_m: source.iter().map(|p| p.x_m).sum::<f64>() / n,
        y_m: source.iter().map(|p| p.y_m).sum::<f64>() / n,
    };
    let target_mean = Point2 {
        x_m: target.iter().map(|p| p.x_m).sum::<f64>() / n,
        y_m: target.iter().map(|p| p.y_m).sum::<f64>() / n,
    };
    let (mut dot, mut cross) = (0.0, 0.0);
    for pair in pairs {
        let sx = pair.source.x_m - source_mean.x_m;
        let sy = pair.source.y_m - source_mean.y_m;
        let tx = pair.target.x_m - target_mean.x_m;
        let ty = pair.target.y_m - target_mean.y_m;
        dot += sx * tx + sy * ty;
        cross += sx * ty - sy * tx;
    }
    if dot.hypot(cross) <= 1e-12 {
        return None;
    }
    let angle = cross.atan2(dot);
    Some(Pose2 {
        x_m: target_mean.x_m - angle.cos() * source_mean.x_m + angle.sin() * source_mean.y_m,
        y_m: target_mean.y_m - angle.sin() * source_mean.x_m - angle.cos() * source_mean.y_m,
        yaw_rad: angle,
    })
}
