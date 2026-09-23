//! Optional coarse correction for console scans, not a navigation calibration.
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Calibration {
    pub schema_version: u8,
    pub status: String,
    pub range_offset_m: f64,
    pub clockwise_yaw_bins: i32,
}

impl Calibration {
    pub fn validate(&self) -> Result<(), &'static str> {
        if self.schema_version != 1
            || self.status != "coarse"
            || !self.range_offset_m.is_finite()
            || self.range_offset_m.abs() > 0.1
            || !(-30..=30).contains(&self.clockwise_yaw_bins)
        {
            return Err("invalid coarse lidar calibration");
        }
        Ok(())
    }

    pub fn apply(&self, row: &mut serde_json::Value) -> Result<(), &'static str> {
        self.validate()?;
        if row.get("calibration").is_some() {
            return Err("lidar correction already applied");
        }
        let raw = row["ranges"].as_array().ok_or("missing lidar ranges")?;
        if raw.len() != 360 {
            return Err("coarse calibration requires 360 bins");
        }
        let mut corrected = vec![None; 360];
        for (i, value) in raw.iter().enumerate() {
            let r = if value.is_null() {
                None
            } else {
                let r = value.as_f64().ok_or("invalid lidar range")?;
                if !(0.02..=12.0).contains(&r) {
                    return Err("raw lidar range outside decoder limits");
                }
                let r = r + self.range_offset_m;
                (0.02..=12.0).contains(&r).then_some(r)
            };
            let target = (i as i32 + self.clockwise_yaw_bins).rem_euclid(360) as usize;
            corrected[target] = r;
        }
        // Preserve raw bins and their indexing; consumers must not apply this twice.
        row["raw_ranges"] = row["ranges"].clone();
        row["raw_valid_fraction"] = row["valid_fraction"].clone();
        row["valid_fraction"] =
            serde_json::json!(corrected.iter().filter(|r| r.is_some()).count() as f64 / 360.0);
        row["ranges"] = serde_json::json!(corrected);
        row["calibration"] = serde_json::to_value(self).map_err(|_| "calibration serialization")?;
        row["frame_id"] = serde_json::json!("lidar_origin_coarse_body_heading");
        row["navigation_validated"] = serde_json::json!(false);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn calibration() -> Calibration {
        Calibration {
            schema_version: 1,
            status: "coarse".into(),
            range_offset_m: -0.02,
            clockwise_yaw_bins: 5,
        }
    }
    #[test]
    fn clockwise_rotation_wraps_preserves_raw_and_unknown() {
        let mut ranges = vec![None; 360];
        ranges[0] = Some(1.17);
        ranges[359] = Some(1.69);
        ranges[90] = Some(0.025);
        let mut row = serde_json::json!({"ranges":ranges,"seq":7,"at_ms":123,"coverage":0.99,"valid_fraction":3.0/360.0});
        calibration().apply(&mut row).unwrap();
        assert!((row["ranges"][5].as_f64().unwrap() - 1.15).abs() < 1e-12);
        assert!((row["ranges"][4].as_f64().unwrap() - 1.67).abs() < 1e-12);
        assert!(row["ranges"][95].is_null());
        assert!(row["ranges"][0].is_null());
        assert_eq!(row["raw_ranges"][0], 1.17);
        assert_eq!(row["seq"], 7);
        assert_eq!(row["at_ms"], 123);
        assert_eq!(row["coverage"], 0.99);
        assert_eq!(row["navigation_validated"], false);
        assert!(calibration().apply(&mut row).is_err());
    }
    #[test]
    fn invalid_calibration_and_scan_are_rejected() {
        let mut c = calibration();
        for offset in [f64::NAN, f64::INFINITY, 0.11] {
            c.range_offset_m = offset;
            assert!(c.validate().is_err());
        }
        c = calibration();
        c.clockwise_yaw_bins = 31;
        assert!(c.validate().is_err());
        assert!(
            calibration()
                .apply(&mut serde_json::json!({"ranges":[1.0]}))
                .is_err()
        );
        let mut row = serde_json::json!({"ranges":vec![Some(-1.0);360]});
        assert!(calibration().apply(&mut row).is_err());
        assert!(row.get("raw_ranges").is_none());
    }
}
