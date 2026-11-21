//! Rust camera owner for the model-free numbered variants. V4L2 supplies MJPEG
//! and a monotonic exposure timestamp; the race workers consume the latest BGR frame.
use serde_json::json;
use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::os::unix::net::UnixDatagram;
use std::path::Path;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;
use zune_core::{bytestream::ZCursor, colorspace::ColorSpace, options::DecoderOptions};
use zune_jpeg::JpegDecoder;

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

fn monotonic() -> f64 {
    let t = rustix::time::clock_gettime(rustix::time::ClockId::Monotonic);
    t.tv_sec as f64 + t.tv_nsec as f64 * 1e-9
}

fn stream(
    device: &str,
    shm: &Path,
    out: &Path,
    stop: &AtomicBool,
    id: &mut u64,
    log: &mut BufWriter<File>,
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
    let socket = shm.with_added_extension("lamp.sock");
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
            continue;
        }
        let opts = DecoderOptions::default()
            .jpeg_set_out_colorspace(ColorSpace::BGR)
            .set_max_width(640)
            .set_max_height(480);
        let mut decoder = JpegDecoder::new_with_options(ZCursor::new(&frame[..]), opts);
        if decoder.decode_headers().is_err()
            || decoder.dimensions() != Some((640, 480))
            || decoder.decode_into(&mut pixels).is_err()
        {
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
        let _ = notify.send_to(b"1", &socket);
        if *id == 1 {
            println!(
                "{}",
                json!({"kind":"ready","backend":"rust_v4l2","source":device,
            "frame":*id,"captured_monotonic_s":captured,"size":[640,480],"model":null})
            );
        }
        // Preserve original JPEG evidence without decoding or recompressing it again.
        if captured - saved >= 0.24 {
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
    let [device, shared, output] = args else {
        eprintln!("capture-camera DEVICE SHM OUTPUT");
        return 2;
    };
    let (shm, out) = (Path::new(shared), Path::new(output));
    let run = || -> Result<()> {
        fs::create_dir_all(out.join("frames"))?;
        let mut log = BufWriter::new(File::create(out.join("frames.jsonl"))?);
        let stop = Arc::new(AtomicBool::new(false));
        #[cfg(feature = "ros2")]
        {
            let flag = stop.clone();
            ctrlc::set_handler(move || flag.store(true, Ordering::Release))?;
        }
        let mut id = 0;
        while !stop.load(Ordering::Acquire) {
            if let Err(e) = stream(device, shm, out, &stop, &mut id, &mut log) {
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
