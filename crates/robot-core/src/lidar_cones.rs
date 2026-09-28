//! Bounded geometric cone candidates from raw scans. No visual or map labels.
//! The circular cross-section model is explicit and simulation-only until its
//! bounds have been measured at the physical scanner's mounting height.
use crate::ValidationError;
use crate::autonomy::Point2;
use crate::local_world::MAX_ELEMENTS;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct LidarConeConfig {
    pub min_points: usize,
    pub max_points: usize,
    pub min_radius_m: f64,
    pub max_radius_m: f64,
    pub max_point_gap_m: f64,
    pub range_error_m: f64,
    pub max_residual_m: f64,
    pub min_arc_depth_m: f64,
    pub min_confirmations: u32,
    pub max_confirmation_gap_ms: u64,
}

impl LidarConeConfig {
    pub fn simulation() -> Self {
        Self {
            min_points: 5,
            max_points: 128,
            min_radius_m: 0.08,
            max_radius_m: 0.26,
            max_point_gap_m: 0.12,
            range_error_m: 0.003,
            max_residual_m: 0.009,
            min_arc_depth_m: 0.015,
            min_confirmations: 3,
            max_confirmation_gap_ms: 300,
        }
    }

    pub fn validate(&self) -> Result<(), ValidationError> {
        if !(5..=128).contains(&self.min_points)
            || !(self.min_points..=128).contains(&self.max_points)
            || !(2..=100).contains(&self.min_confirmations)
            || !(1..=1800).contains(&self.max_confirmation_gap_ms)
            || ![
                self.min_radius_m,
                self.max_radius_m,
                self.max_point_gap_m,
                self.range_error_m,
                self.max_residual_m,
                self.min_arc_depth_m,
            ]
            .iter()
            .all(|v| v.is_finite() && *v > 0.0)
            || self.min_radius_m < 0.02
            || self.max_radius_m > 0.5
            || self.min_radius_m >= self.max_radius_m
            || self.max_point_gap_m > 0.3
            || self.range_error_m > 0.05
            || self.max_residual_m > 0.02
            || self.min_arc_depth_m <= 2.0 * self.range_error_m
            || self.min_arc_depth_m >= self.min_radius_m
        {
            return Err(ValidationError(
                "invalid lidar cone cross-section bounds".into(),
            ));
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct Candidate {
    pub center: Point2,
    pub radius_m: f64,
    pub error_m: f64,
}

/// Input has already passed full-scan validation. Clusters wrap at beam zero;
/// missing rays and distance jumps split them. Oversized clusters are rejected
/// whole, never chopped into cone-sized pieces along a wall.
pub(crate) fn detect(
    ranges: &[Option<f64>],
    angle_min: f64,
    increment: f64,
    cfg: &LidarConeConfig,
) -> Result<[Option<Candidate>; MAX_ELEMENTS], ValidationError> {
    let n = ranges.len();
    let point = |i: usize| {
        ranges[i].map(|r| {
            let a = angle_min + i as f64 * increment;
            Point2 {
                x_m: r * a.cos(),
                y_m: r * a.sin(),
            }
        })
    };
    let cut = |a: usize, b: usize| match (point(a), point(b)) {
        (Some(a), Some(b)) => a.distance(b) > cfg.max_point_gap_m,
        _ => true,
    };
    let mut result = [None; MAX_ELEMENTS];
    let Some(start) = (0..n).find(|&i| cut((i + n - 1) % n, i)) else {
        return Ok(result); // One continuous enclosure is not a cone.
    };
    let mut points = [Point2::default(); 128];
    let mut count = 0;
    let mut overflow = false;
    let mut output_count = 0;
    for offset in 0..=n {
        let i = (start + offset) % n;
        if offset == n || cut((i + n - 1) % n, i) {
            if !overflow
                && count >= cfg.min_points
                && let Some(candidate) = fit(&points[..count], cfg)
            {
                if output_count == MAX_ELEMENTS {
                    return Err(ValidationError(
                        "lidar cone candidate capacity exceeded".into(),
                    ));
                }
                result[output_count] = Some(candidate);
                output_count += 1;
            }
            count = 0;
            overflow = false;
        }
        if offset < n
            && let Some(p) = point(i)
        {
            if count < cfg.max_points {
                points[count] = p;
                count += 1;
            } else {
                overflow = true;
            }
        }
    }
    Ok(result)
}

fn fit(points: &[Point2], cfg: &LidarConeConfig) -> Option<Candidate> {
    let first = points[0];
    let last = points[points.len() - 1];
    let width = first.distance(last);
    if width < 0.04
        || points
            .iter()
            .any(|p| p.distance(first) > 2.0 * cfg.max_radius_m)
    {
        return None;
    }
    // Centered least-squares circle: 2*x*cx + 2*y*cy + c = x²+y².
    // Pivoted elimination rejects singular/line-like data without a dependency.
    let mean = Point2 {
        x_m: points.iter().map(|p| p.x_m).sum::<f64>() / points.len() as f64,
        y_m: points.iter().map(|p| p.y_m).sum::<f64>() / points.len() as f64,
    };
    let mut normal = [[0.0; 4]; 3];
    let mut depth: f64 = 0.0;
    for p in points {
        let x = p.x_m - mean.x_m;
        let y = p.y_m - mean.y_m;
        let row = [2.0 * x, 2.0 * y, 1.0];
        for i in 0..3 {
            for j in 0..3 {
                normal[i][j] += row[i] * row[j];
            }
            normal[i][3] += row[i] * (x * x + y * y);
        }
        depth = depth.max(
            ((p.x_m - first.x_m) * (last.y_m - first.y_m)
                - (p.y_m - first.y_m) * (last.x_m - first.x_m))
                .abs()
                / width,
        );
    }
    if depth < cfg.min_arc_depth_m {
        return None;
    }
    for column in 0..3 {
        let pivot = (column..3)
            .max_by(|&a, &b| normal[a][column].abs().total_cmp(&normal[b][column].abs()))?;
        normal.swap(column, pivot);
        let scale = normal[column][column];
        if scale.abs() < 1e-10 {
            return None;
        }
        for value in &mut normal[column][column..] {
            *value /= scale;
        }
        for i in 0..3 {
            if i == column {
                continue;
            }
            let factor = normal[i][column];
            let pivot_row = normal[column];
            for (value, pivot) in normal[i][column..].iter_mut().zip(&pivot_row[column..]) {
                *value -= factor * pivot;
            }
        }
    }
    let center = Point2 {
        x_m: mean.x_m + normal[0][3],
        y_m: mean.y_m + normal[1][3],
    };
    let radius = (normal[0][3].powi(2) + normal[1][3].powi(2) + normal[2][3]).sqrt();
    if !center.valid() || !(cfg.min_radius_m..=cfg.max_radius_m).contains(&radius) {
        return None;
    }
    let residual = points
        .iter()
        .map(|p| (p.distance(center) - radius).abs())
        .fold(0.0, f64::max);
    // Only the near, convex surface of a compact object is observable from
    // outside. Concave wall corners/circles around the scanner fail this test.
    if residual > cfg.max_residual_m
        || points.iter().any(|p| {
            (p.x_m - center.x_m) * p.x_m + (p.y_m - center.y_m) * p.y_m
                > cfg.range_error_m * p.x_m.hypot(p.y_m)
        })
    {
        return None;
    }
    // Short-arc sensitivity is retained as uncertainty; no claim of measured
    // physical accuracy. The configuration and model still need real calibration.
    let error_m = (cfg.range_error_m + residual) * (1.0 + 2.0 * radius / depth);
    Some(Candidate {
        center,
        radius_m: radius,
        error_m,
    })
}
