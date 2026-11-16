//! Serial transport settings, deliberately separate from geometric calibration.
use serde::Deserialize;
use std::{fs, io, path::Path};

#[derive(Deserialize)]
struct Settings {
    baud_rate: u32,
}

/// Existing N10 installations without this file retain their original speed.
pub fn baud_rate(executable: &Path) -> Result<u32, Box<dyn std::error::Error>> {
    match fs::read(executable.with_file_name("lidar-serial.json")) {
        Ok(bytes) => Ok(serde_json::from_slice::<Settings>(&bytes)?.baud_rate),
        Err(error) if error.kind() == io::ErrorKind::NotFound => Ok(230400),
        Err(error) => Err(error.into()),
    }
}
