//! Full-scan coverage and explicit laser-to-body TF before scan odometry.
use crate::autonomy::Result;
use xt_stcar_robot_core::LidarSample;
use xt_stcar_robot_core::autonomy::{Point2, Pose2};
use xt_stcar_robot_core::localization::{LocalizationConfig, LocalizationUpdate, ScanOdometry};
use xt_stcar_robot_core::scan::{ScanConfig, validate_full_scan};

pub struct LaserPosePipeline {
    coverage: ScanConfig,
    lidar_in_body: Pose2,
    odometry: ScanOdometry,
}

impl LaserPosePipeline {
    pub fn new(
        coverage: ScanConfig,
        localization: LocalizationConfig,
        lidar_in_body: Pose2,
        initial_pose: Pose2,
    ) -> Result<Self> {
        coverage.validate().map_err(|e| e.to_string())?;
        if !lidar_in_body.valid() || lidar_in_body.x_m.abs() > 5.0 || lidar_in_body.y_m.abs() > 5.0
        {
            return Err("invalid laser-to-body extrinsic".into());
        }
        let odometry = ScanOdometry::new(localization, initial_pose).map_err(|e| e.to_string())?;
        Ok(Self {
            coverage,
            lidar_in_body,
            odometry,
        })
    }

    pub fn update(&mut self, scan: &LidarSample) -> Result<LocalizationUpdate> {
        validate_full_scan(scan, &self.coverage).map_err(|e| e.to_string())?;
        let points: Vec<Point2> = scan
            .ranges_m
            .iter()
            .enumerate()
            .filter_map(|(index, range)| {
                range.map(|r| {
                    let angle = scan.angle_min_rad + index as f64 * scan.angle_increment_rad;
                    self.lidar_in_body.body_to_world(Point2 {
                        x_m: r * angle.cos(),
                        y_m: r * angle.sin(),
                    })
                })
            })
            .collect();
        self.odometry
            .update(scan.captured_at, &points)
            .map_err(|e| e.to_string())
    }

    pub fn reset(&mut self, known_pose: Pose2) -> Result<()> {
        self.odometry.reset(known_pose).map_err(|e| e.to_string())
    }
}
