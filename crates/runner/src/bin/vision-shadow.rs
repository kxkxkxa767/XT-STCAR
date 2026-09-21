//! Bounded JPEG IPC -> persistent perception -> JSON. No device or motion sink.
use serde::Deserialize;
use std::io::{self, BufRead, Read, Write};
use std::path::PathBuf;
use std::time::Instant;
use xt_stcar_robot_core::{FrameId, Timestamp};
use xt_stcar_robot_runner::{
    input::{MAX_CONFIG_BYTES, read_regular_file},
    perception::RoadPipeline,
    vision::VisionOptions,
};
use xt_stcar_vision::road::RoadConfig;

const MAX_JPEG: usize = 2 * 1024 * 1024;
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Header {
    sequence: u64,
    captured_at_ms: u64,
    jpeg_bytes: usize,
}
fn run() -> Result<(), String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.len() != 4 {
        return Err("usage: vision-shadow ROAD_CONFIG MODEL.onnx RUNTIME_LIB MODEL_SPEC; stdin: JSON header line {sequence,captured_at_ms,jpeg_bytes} + JPEG bytes; stdout: JSON lines; NO ACTUATOR OUTPUT".into());
    }
    let config: RoadConfig = serde_json::from_slice(&read_regular_file(
        &PathBuf::from(&args[0]),
        MAX_CONFIG_BYTES,
    )?)
    .map_err(|e| e.to_string())?;
    let options = VisionOptions {
        model: PathBuf::from(&args[1]),
        runtime_lib: PathBuf::from(&args[2]),
        spec: PathBuf::from(&args[3]),
    };
    let mut pipeline = RoadPipeline::new_online(config, Some(&options), Default::default())?;
    let mut output = io::stdout().lock();
    writeln!(
        output,
        "{}",
        serde_json::json!({"kind":"ready", "physical_output_enabled":false})
    )
    .map_err(|e| e.to_string())?;
    output.flush().map_err(|e| e.to_string())?;
    let mut input = io::BufReader::new(io::stdin().lock());
    let mut previous = None;
    loop {
        let mut line = String::new();
        if input
            .by_ref()
            .take(4097)
            .read_line(&mut line)
            .map_err(|e| e.to_string())?
            == 0
        {
            return Ok(());
        }
        if line.len() > 4096 || !line.ends_with('\n') {
            return Err("oversized/incomplete frame header".into());
        }
        let h: Header = serde_json::from_str(&line).map_err(|e| e.to_string())?;
        if h.jpeg_bytes == 0
            || h.jpeg_bytes > MAX_JPEG
            || previous.is_some_and(|(seq, at)| h.sequence <= seq || h.captured_at_ms <= at)
        {
            return Err("invalid JPEG length or non-increasing source sequence/time".into());
        }
        previous = Some((h.sequence, h.captured_at_ms));
        let mut jpeg = vec![0; h.jpeg_bytes];
        input.read_exact(&mut jpeg).map_err(|e| e.to_string())?;
        let start = Instant::now();
        let mut reader =
            image::ImageReader::with_format(io::Cursor::new(jpeg), image::ImageFormat::Jpeg);
        let mut limits = image::Limits::default();
        limits.max_image_width = Some(1920);
        limits.max_image_height = Some(1080);
        limits.max_alloc = Some(32 * 1024 * 1024);
        reader.limits(limits);
        let image = reader.decode().map_err(|e| e.to_string())?.into_rgb8();
        let jpeg_decode_ms = start.elapsed().as_secs_f64() * 1000.;
        let road = pipeline.process(&image, Timestamp(h.captured_at_ms), FrameId("body".into()))?;
        let value = serde_json::json!({"kind":"vision_shadow", "physical_output_enabled":false,
            "sequence": h.sequence, "captured_at_ms": h.captured_at_ms,
            "clock":"camera owner host-monotonic receive time, not hardware synchronized",
            "jpeg_decode_ms":jpeg_decode_ms, "diagnostics":pipeline.diagnostics(), "road":road});
        serde_json::to_writer(&mut output, &value).map_err(|e| e.to_string())?;
        output
            .write_all(b"\n")
            .and_then(|()| output.flush())
            .map_err(|e| e.to_string())?;
    }
}
fn main() {
    if let Err(error) = run() {
        eprintln!("vision-shadow: {error}");
        std::process::exit(1);
    }
}
