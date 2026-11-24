use std::fs::File;
use std::io::Read;
use std::os::unix::net::UnixDatagram;
use std::path::PathBuf;
use std::sync::mpsc::{self, RecvTimeoutError, Sender};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use crate::race::Event;
use crate::race_crosswalk_vision::{SHM_HEADER_LEN, parse_shm_frame, pin_current_thread};
use crate::traffic_lamp::{Detector, Lights};

pub(crate) struct Observation {
    pub frame: i64,
    pub captured_mono_s: f64,
    pub lights: Lights,
    pub compute_ms: f64,
    pub pinned_cpu: Option<usize>,
}

pub(crate) fn lamp_phase(phase: Option<&str>) -> bool {
    matches!(phase, Some("light_approach" | "light_wait"))
}

/// Block on phase changes outside the gantry. While active, inspect just the
/// header until the camera atomically publishes a new capture.
pub(crate) fn spawn(
    path: PathBuf,
    events: Sender<Event>,
) -> Option<(Sender<bool>, JoinHandle<()>)> {
    let (phases, rx) = mpsc::channel();
    let thread = std::thread::Builder::new()
        .name("rust-light".into())
        .spawn(move || {
            let pinned_cpu = pin_current_thread(3);
            let socket_path = path.with_added_extension("lamp.sock");
            let mut wake = UnixDatagram::bind(&socket_path).ok();
            if wake
                .as_ref()
                .is_some_and(|s| s.set_read_timeout(Some(Duration::from_millis(25))).is_err())
            {
                wake = None;
                let _ = std::fs::remove_file(&socket_path);
            }
            let mut detector = Detector::default();
            let mut active = false;
            let mut last = None;
            let mut bytes = Vec::new();
            let mut header = [0; SHM_HEADER_LEN];
            loop {
                if !active {
                    let Ok(next) = rx.recv() else { break };
                    active = next;
                    continue;
                }
                let phase_update = if let Some(socket) = &wake {
                    // The camera notifies immediately after the atomic rename.
                    // A timed wake also handles an older producer or a dropped notification.
                    let _ = socket.recv(&mut [0u8; 1]);
                    rx.try_recv().map_err(|error| match error {
                        mpsc::TryRecvError::Disconnected => RecvTimeoutError::Disconnected,
                        mpsc::TryRecvError::Empty => RecvTimeoutError::Timeout,
                    })
                } else {
                    rx.recv_timeout(Duration::from_millis(8))
                };
                match phase_update {
                    Ok(next) => {
                        active = next;
                        continue;
                    }
                    Err(RecvTimeoutError::Disconnected) => break,
                    Err(RecvTimeoutError::Timeout) => {}
                }
                let Ok(mut file) = File::open(&path) else {
                    continue;
                };
                if file.read_exact(&mut header).is_err() {
                    continue;
                }
                let id = header[4..12].try_into().map(i64::from_le_bytes).ok();
                if id == last {
                    continue;
                }
                bytes.clear();
                bytes.extend_from_slice(&header);
                if file.read_to_end(&mut bytes).is_err() {
                    continue;
                }
                let Some((frame, captured_mono_s, w, h, pixels)) = parse_shm_frame(&bytes) else {
                    continue;
                };
                last = Some(frame);
                let start = Instant::now();
                let lights = detector.detect(pixels, w, h);
                let obs = Observation {
                    frame,
                    captured_mono_s,
                    lights,
                    pinned_cpu,
                    compute_ms: start.elapsed().as_secs_f64() * 1000.,
                };
                if events.send(Event::RustLamp(obs, Instant::now())).is_err() {
                    break;
                }
            }
            if wake.is_some() {
                let _ = std::fs::remove_file(socket_path);
            }
        })
        .inspect_err(|error| eprintln!("Rust traffic light worker: {error}"))
        .ok()?;
    Some((phases, thread))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tagged_camera_frame_reaches_only_the_active_lamp_worker() {
        let path =
            std::env::temp_dir().join(format!("xt-stcar-lamp-test-{}.bgr", std::process::id()));
        let mut bytes = b"XTF1".to_vec();
        bytes.extend(7u64.to_le_bytes());
        bytes.extend(12.5f64.to_le_bytes());
        bytes.extend(32u32.to_le_bytes());
        bytes.extend(32u32.to_le_bytes());
        bytes.extend(0u32.to_le_bytes());
        bytes.resize(SHM_HEADER_LEN + 32 * 32 * 3, 0);
        std::fs::write(&path, bytes).unwrap();
        let (events, incoming) = mpsc::channel();
        let (phases, worker) = spawn(path.clone(), events).unwrap();
        assert!(incoming.recv_timeout(Duration::from_millis(24)).is_err());
        phases.send(true).unwrap();
        let Ok(Event::RustLamp(obs, _)) = incoming.recv_timeout(Duration::from_secs(2)) else {
            panic!("no lamp frame");
        };
        assert_eq!((obs.frame, obs.captured_mono_s), (7, 12.5));
        assert!(!obs.lights.red && !obs.lights.green);
        assert!(
            incoming.recv_timeout(Duration::from_millis(24)).is_err(),
            "duplicate capture"
        );
        drop(phases);
        worker.join().unwrap();
        std::fs::remove_file(path).unwrap();
    }
}
