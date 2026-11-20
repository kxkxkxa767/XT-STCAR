//! Offline reader for a sequence of BGR images. Each image is prefixed with its
//! width and height as little-endian u32 values. Uses the deployed Rust detector.
#[path = "../route-engine/src/traffic_lamp.rs"]
mod traffic_lamp;

use std::io::{self, BufReader, Read};
use std::time::Instant;

fn main() -> io::Result<()> {
    let mut stream = BufReader::new(io::stdin().lock());
    let mut detector = traffic_lamp::Detector::default();
    let mut image = Vec::new();
    let mut header = [0u8; 8];
    while stream.read(&mut header[..1])? != 0 {
        stream.read_exact(&mut header[1..])?;
        let width = u32::from_le_bytes(header[..4].try_into().unwrap()) as usize;
        let height = u32::from_le_bytes(header[4..].try_into().unwrap()) as usize;
        let size = width
            .checked_mul(height)
            .and_then(|n| n.checked_mul(3))
            .ok_or_else(|| io::Error::other("invalid dimensions"))?;
        image.resize(size, 0);
        stream.read_exact(&mut image)?;
        let start = Instant::now();
        let light = detector.detect(&image, width, height);
        println!(
            "{} {} {}",
            u8::from(light.red),
            u8::from(light.green),
            start.elapsed().as_nanos()
        );
    }
    Ok(())
}
