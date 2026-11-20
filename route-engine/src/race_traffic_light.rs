use std::fs::File;
use std::io::Read;
use std::os::unix::net::UnixDatagram;
use std::path::PathBuf;
use std::sync::mpsc::{self, RecvTimeoutError, Sender};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use crate::py::V;
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

/// Both sources refer to the same camera counter. A late YOLO result can add
/// colour to this capture, but cannot replace a newer observation.
pub(crate) fn merge(latest: &V, mut incoming: V) -> Option<V> {
    let frame = incoming.get("frame").oi()?;
    if let Some(previous) = latest.get("frame").oi() {
        if previous > frame {
            return None;
        }
        if previous == frame {
            for key in ["red", "green"] {
                let score = latest
                    .get(key)
                    .of()
                    .unwrap_or_default()
                    .max(incoming.get(key).of().unwrap_or_default());
                incoming.set(key, score);
            }
            if let Some(captured) = latest.get("captured_at").of() {
                incoming.set(
                    "captured_at",
                    incoming.get("captured_at").of()?.min(captured),
                );
            }
        }
    }
    incoming.set("lamp_fusion", true);
    Some(incoming)
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
    use crate::{dict, maneuver::Maneuver};

    #[test]
    fn the_two_sources_share_one_vote_per_capture() {
        let mut m = Maneuver::new(&0.0.into(), &40.0.into(), &1670.into(), &true.into());
        m.race_inputs = Some(Default::default());
        m.route_stage = "light_lane";
        m.drive_since = Some(0.);
        m.light_since = Some(0.);
        m.light_stop_state = Some("waiting");
        m.light_stop_evidence = dict! {"gantry" => dict! {"source_seq" => 1}};
        m._light_gantry_since = Some(1.);
        m._light_wait_since = Some(1.);
        m._light_wait_tick = Some(1.);
        let sample = |frame: i64, at: f64, red: f64, green: f64| dict! {"frame" => frame, "captured_at" => at, "red" => red, "green" => green};
        let green = merge(&V::None, sample(10, 1.2, 0., 1.)).unwrap();
        m.light_observe(&dict! {"light" => green.clone()}, 1.21);
        let same = merge(&green, sample(10, 1.23, 0., 0.8)).unwrap();
        m.light_observe(&dict! {"light" => same.clone()}, 1.25);
        assert_eq!(m._light_frames.len(), 1);
        let amended = merge(&same, sample(10, 1.24, 0.9, 0.)).unwrap();
        m.light_observe(&dict! {"light" => amended.clone()}, 1.26);
        m.light_observe(&dict! {"light" => amended.clone()}, 1.27);
        assert_eq!(m._light_frames.len(), 1);
        assert_eq!(m._light_red_frames, 1);
        assert_eq!(m._light_stop_red_frames, 1);
        assert!(merge(&amended, sample(9, 1.0, 1., 0.)).is_none());
        let scan = dict! {"seq" => 2, "at_ms" => 1500, "received_at" => 1.5, "ranges" => vec![V::Float(3.); 360]};
        m.light_observe(&dict! {"light" => sample(11, 1.4, 0., 1.)}, 1.41);
        assert!(!m.light_wait(&scan, 1.42, &V::None));
        m.light_observe(&dict! {"light" => sample(12, 1.6, 0., 1.)}, 1.61);
        assert!(m.light_wait(&scan, 1.62, &V::None));
    }

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
