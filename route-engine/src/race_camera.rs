//! Rust camera owner for the numbered race programs. V4L2 supplies MJPEG
//! and a monotonic exposure timestamp; the race workers consume the latest BGR frame.
use serde_json::json;
use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::os::unix::net::UnixDatagram;
use std::path::Path;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;
use xt_stcar_camera_jpeg::decode_bgr_640x480;
use crate::race_frame_timing::{FrameLog, monotonic_s as monotonic};

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

fn stream(
    device: &str,
    shm: &Path,
    out: &Path,
    stop: &AtomicBool,
    id: &mut u64,
    save_all: bool,
    log: &mut BufWriter<File>,
    timings: &mut Option<FrameLog>,
) -> Result<()> {
    let mut cam = rscam::Camera::new(device)?;
    cam.start(&rscam::Config {
        interval: (1, 30),
        resolution: (640, 480),
        format: b"MJPG",
        ..Default::default()
    })?;
    let notify = UnixDatagram::unbound()?;
    notify.set_nonblocking(true)?;
    let lamp_socket = shm.with_added_extension("lamp.sock");
    let crosswalk_socket = crate::race_crosswalk_vision::wake_socket_path(shm);
    let temporary = shm.with_added_extension("tmp");
    let mut pixels = vec![0u8; 640 * 480 * 3];
    let mut last = f64::NEG_INFINITY;
    let mut saved = f64::NEG_INFINITY;
    let mut decode_errors = 0u32;
    while !stop.load(Ordering::Acquire) {
        let frame = cam.capture()?;
        let dequeue = monotonic();
        let stamp = frame.get_timestamp() as f64 * 1e-6;
        // A driver reporting wall time or zero has no usable monotonic stamp.
        // Old monotonic buffers are drained without relabelling them as fresh.
        let captured = if stamp <= 0. || stamp > dequeue + 1. {
            dequeue
        } else {
            stamp
        };
        if dequeue - captured > 0.25 || captured - last < 0.074 {
            if let Some(log) = timings {
                let reason = if dequeue - captured > 0.25 { "stale" } else { "rate_limit" };
                log.record(json!({"status":reason,"capture_s":captured,"driver_stamp_s":stamp,"dequeue_s":dequeue}));
            }
            continue;
        }
        let decode_start = timings.as_ref().map(|_| monotonic());
        let decoded = decode_bgr_640x480(&frame[..], &mut pixels);
        let decode_end = timings.as_ref().map(|_| monotonic());
        if !decoded {
            if let Some(log) = timings {
                log.record(json!({"status":"decode_error","capture_s":captured,"driver_stamp_s":stamp,
                    "dequeue_s":dequeue,"decode_start_s":decode_start,"decode_end_s":decode_end}));
            }
            decode_errors += 1;
            if decode_errors == 8 {
                return Err("camera repeatedly returned undecodable MJPEG".into());
            }
            continue;
        }
        decode_errors = 0;
        last = captured;
        *id += 1;
        let mut header = [0u8; 32];
        header[..4].copy_from_slice(b"XTF1");
        header[4..12].copy_from_slice(&id.to_le_bytes());
        header[12..20].copy_from_slice(&captured.to_le_bytes());
        header[20..24].copy_from_slice(&640u32.to_le_bytes());
        header[24..28].copy_from_slice(&480u32.to_le_bytes());
        {
            let mut f = File::create(&temporary)?;
            f.write_all(&header)?;
            f.write_all(&pixels)?;
        }
        fs::rename(&temporary, shm)?;
        let published = timings.as_ref().map(|_| monotonic());
        let _ = notify.send_to(b"1", &lamp_socket);
        let _ = notify.send_to(b"1", &crosswalk_socket);
        if let Some(log) = timings {
            log.record(json!({"status":"published","frame":*id,"capture_s":captured,"driver_stamp_s":stamp,
                "dequeue_s":dequeue,"decode_start_s":decode_start,"decode_end_s":decode_end,"publish_s":published}));
        }
        if *id == 1 {
            println!(
                "{}",
                json!({"kind":"ready","backend":"rust_v4l2","source":device,
            "frame":*id,"captured_monotonic_s":captured,"size":[640,480],"model":null})
            );
        }
        // Preserve original JPEG evidence without decoding or recompressing it again.
        if (save_all && *id <= 1500) || (!save_all && captured - saved >= 0.24) {
            let name = format!("frame-{:06}.jpg", *id);
            if fs::write(out.join("frames").join(&name), &frame[..]).is_ok() {
                let _ = writeln!(
                    log,
                    "{}",
                    json!({"frame":*id,"saved":name,"captured_monotonic_s":captured,"dequeue_s":dequeue})
                );
                let _ = log.flush();
            }
            saved = captured;
        }
    }
    Ok(())
}

pub fn main(args: &[String]) -> i32 {
    let save_all = args.len() == 4 && args[3] == "--save-all";
    let [device, shared, output] = (if save_all { &args[..3] } else { args }) else {
        eprintln!("capture-camera DEVICE SHM OUTPUT [--save-all]");
        return 2;
    };
    let (shm, out) = (Path::new(shared), Path::new(output));
    let run = || -> Result<()> {
        fs::create_dir_all(out.join("frames"))?;
        let mut log = BufWriter::new(File::create(out.join("frames.jsonl"))?);
        let mut timings = FrameLog::open("camera-timing.jsonl");
        let stop = Arc::new(AtomicBool::new(false));
        #[cfg(feature = "ros2")]
        {
            let flag = stop.clone();
            ctrlc::set_handler(move || flag.store(true, Ordering::Release))?;
        }
        let mut id = 0;
        while !stop.load(Ordering::Acquire) {
            if let Err(e) = stream(device, shm, out, &stop, &mut id, save_all, &mut log, &mut timings) {
                eprintln!("camera {device}: {e}");
                std::thread::sleep(Duration::from_millis(300));
            }
        }
        println!(
            "{}",
            json!({"kind":"summary","backend":"rust_v4l2","frames":id})
        );
        Ok(())
    };
    match run() {
        Ok(()) => 0,
        Err(e) => {
            eprintln!("camera: {e}");
            1
        }
    }
}
