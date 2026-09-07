use serde::{Deserialize, Serialize};

/// Monotonic milliseconds since the beginning of this replay/session, never wall-clock time.
#[derive(Clone, Copy, Debug, Default, Deserialize, Eq, Ord, PartialEq, PartialOrd, Serialize)]
#[serde(transparent)]
pub struct Timestamp(pub u64);

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(transparent)]
pub struct FrameId(pub String);

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, Ord, PartialEq, PartialOrd, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum SensorKind {
    Imu,
    Lidar,
    Odometry,
    Vision,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct ImuSample {
    pub captured_at: Timestamp,
    pub frame_id: FrameId,
    pub acceleration_mps2: Vec3,
    pub angular_velocity_radps: Vec3,
    pub orientation_xyzw: Quaternion,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct LidarSample {
    pub captured_at: Timestamp,
    pub frame_id: FrameId,
    pub angle_min_rad: f64,
    pub angle_increment_rad: f64,
    pub range_min_m: f64,
    pub range_max_m: f64,
    /// None is explicitly "no return/unknown", never an obstacle at zero metres.
    pub ranges_m: Vec<Option<f64>>,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct OdometrySample {
    pub captured_at: Timestamp,
    pub frame_id: FrameId,
    pub child_frame_id: FrameId,
    pub position_m: Vec3,
    pub orientation_xyzw: Quaternion,
    pub linear_velocity_mps: Vec3,
    pub angular_velocity_radps: Vec3,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct VisionSample {
    pub captured_at: Timestamp,
    pub frame_id: FrameId,
    pub image_width_px: u32,
    pub image_height_px: u32,
    pub detection_count: u32,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum SensorSample {
    Imu(ImuSample),
    Lidar(LidarSample),
    Odometry(OdometrySample),
    Vision(VisionSample),
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct SensorFrames {
    pub imu_frame: FrameId,
    pub lidar_frame: FrameId,
    pub odometry_frame: FrameId,
    pub body_frame: FrameId,
    pub vision_frame: FrameId,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ValidationError(pub String);

impl std::fmt::Display for ValidationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}
impl std::error::Error for ValidationError {}

impl FrameId {
    pub fn validate(&self) -> Result<(), ValidationError> {
        if self.0.is_empty()
            || self.0.len() > 128
            || !self
                .0
                .bytes()
                .all(|v| v.is_ascii_alphanumeric() || b"_/-".contains(&v))
        {
            return Err(ValidationError(
                "frame_id must contain 1..128 ASCII letters/digits/_/-".into(),
            ));
        }
        Ok(())
    }
}

impl Vec3 {
    fn finite(&self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
}

impl Quaternion {
    fn valid(&self) -> bool {
        let norm = self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w;
        [self.x, self.y, self.z, self.w]
            .iter()
            .all(|v| v.is_finite())
            && (norm - 1.0).abs() <= 1e-3
    }
}

impl SensorSample {
    pub fn kind(&self) -> SensorKind {
        match self {
            Self::Imu(_) => SensorKind::Imu,
            Self::Lidar(_) => SensorKind::Lidar,
            Self::Odometry(_) => SensorKind::Odometry,
            Self::Vision(_) => SensorKind::Vision,
        }
    }

    pub fn captured_at(&self) -> Timestamp {
        match self {
            Self::Imu(v) => v.captured_at,
            Self::Lidar(v) => v.captured_at,
            Self::Odometry(v) => v.captured_at,
            Self::Vision(v) => v.captured_at,
        }
    }

    pub fn frame_id(&self) -> &FrameId {
        match self {
            Self::Imu(v) => &v.frame_id,
            Self::Lidar(v) => &v.frame_id,
            Self::Odometry(v) => &v.frame_id,
            Self::Vision(v) => &v.frame_id,
        }
    }

    pub fn validate(&self, frames: &SensorFrames) -> Result<(), ValidationError> {
        self.frame_id().validate()?;
        let expected = match self.kind() {
            SensorKind::Imu => &frames.imu_frame,
            SensorKind::Lidar => &frames.lidar_frame,
            SensorKind::Odometry => &frames.odometry_frame,
            SensorKind::Vision => &frames.vision_frame,
        };
        if self.frame_id() != expected {
            return Err(ValidationError(format!(
                "unexpected {:?} frame; explicit transform required",
                self.kind()
            )));
        }
        let valid = match self {
            Self::Imu(v) => {
                v.acceleration_mps2.finite()
                    && v.angular_velocity_radps.finite()
                    && v.orientation_xyzw.valid()
            }
            Self::Odometry(v) => {
                v.child_frame_id.validate()?;
                v.child_frame_id == frames.body_frame
                    && v.frame_id != v.child_frame_id
                    && v.position_m.finite()
                    && v.linear_velocity_mps.finite()
                    && v.angular_velocity_radps.finite()
                    && v.orientation_xyzw.valid()
            }
            Self::Lidar(v) => {
                v.angle_min_rad.is_finite()
                    && v.angle_increment_rad.is_finite()
                    && v.angle_increment_rad != 0.0
                    && v.range_min_m.is_finite()
                    && v.range_max_m.is_finite()
                    && v.range_min_m >= 0.0
                    && v.range_max_m > v.range_min_m
                    && !v.ranges_m.is_empty()
                    && v.ranges_m.len() <= 100_000
                    && v.ranges_m
                        .iter()
                        .flatten()
                        .all(|r| r.is_finite() && *r >= v.range_min_m && *r <= v.range_max_m)
            }
            Self::Vision(v) => {
                v.image_width_px > 0
                    && v.image_height_px > 0
                    && v.image_width_px <= 65_536
                    && v.image_height_px <= 65_536
                    && v.detection_count <= 4096
            }
        };
        if !valid {
            return Err(ValidationError(format!(
                "invalid {:?} units, dimensions, values or frame relation",
                self.kind()
            )));
        }
        Ok(())
    }
}
