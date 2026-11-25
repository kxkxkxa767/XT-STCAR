//! The standalone control program (round 97, part B): `route-engine race ...`.
//!
//! One control thread owns the Supervisor (supervisor.rs: the console's route
//! supervision, line for line) and runs the console's loop with no Python and
//! no web server: bridge lines are applied as they arrive (accept_lidar,
//! accept_control and its lock halt), a tick every 20 ms with the console's
//! pacing (at least 5 ms idle after a long tick) and timing records, the
//! health watch every 50 ms, the owner lease renewed by the program itself.
//! Round 99: a new lidar scan is decided on at once (a tick as soon as it is
//! applied, still at least 5 ms after the previous tick) instead of at the
//! next 20 ms tick (on average 10 ms later); the run's CPU time per program
//! thread and bridge goes to summary.json.
//! Reader threads only forward bridge lines; a writer thread writes the
//! evidence files (the trial helper's layout, so trial-timeline, trial-map and
//! route-replay read them unchanged). The bridges run in their own process
//! group: interrupting this program (Ctrl-C, a closed terminal) closes their
//! input and they hold neutral themselves; their own 300 ms command lease
//! also covers a stalled program.
//!
//! Start: the Rust race admission (turn_preview) is shown until it passes;
//! then `--start enter` waits for Enter on the terminal, `--start ready`
//! starts once it has passed continuously for `--ready-hold-s`; `--start
//! never` only shows and records the check for `--max-wait-s` and never arms
//! (static tests on the car).
//!
//! Rust camera observations carry the frame number and monotonic capture time.
//! The lamp worker publishes one colour vote per new frame in the light zone.
//! The crosswalk worker reads the same images during the start lane.

use std::collections::VecDeque;
use std::io::{BufRead, BufReader, Read, Write};
use std::os::unix::process::CommandExt;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender};
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use crate::dict;
use crate::maneuver::ROUTE_MAX_DRIVE_S;
use crate::py::*;
use crate::supervisor::Supervisor;

const TICK_S: f64 = 0.02;
const MIN_IDLE_S: f64 = 0.005;
const HEALTH_S: f64 = 0.05;
const EVIDENCE_S: f64 = 0.15;
pub(crate) const LINE_LIMIT: usize = 65536;

pub(crate) struct Options {
    pub bridge: String,
    out: PathBuf,
    max_drive_s: f64,
    initial_presteer_pwm: i64,
    start: String,
    ready_hold_s: f64,
    pub demo: bool,
    pub control_device: String,
    pub lidar_device: String,
    #[cfg(feature = "ros2")]
    pub lidar_calibration: Option<PathBuf>,
    max_wait_s: f64,
    light_max_wait_s: Option<f64>,
    status_s: f64,
    frame_shm: Option<PathBuf>,
    surface_initial_load: f64,
    ros2: bool,
    #[cfg(feature = "ros2")]
    pub ros_args: Vec<String>,
}

const USAGE: &str = "usage: route-engine race --out NEW_DIR [--bridge PATH (legacy)] [--start enter|ready|never|ros] [--ready-hold-s 3] \
[--max-drive-s 40] [--initial-presteer-pwm 1670] [--max-wait-s 300] [--light-max-wait-s SECONDS] [--status-s .5] [--demo] \
[--control-device /dev/car] [--lidar-device /dev/laser] [--frame-shm PATH] [--surface-initial-load MPS2] [--transport legacy|ros2]";

fn parse(args: &[String]) -> Result<Options, String> {
    #[cfg(feature = "ros2")]
    let (plain_args, ros_args) = split_ros_args(args);
    #[cfg(feature = "ros2")]
    let args = &plain_args;
    let mut o = Options {
        bridge: String::new(),
        out: PathBuf::new(),
        max_drive_s: ROUTE_MAX_DRIVE_S,
        initial_presteer_pwm: 1670,
        start: "enter".into(),
        ready_hold_s: 3.,
        demo: false,
        control_device: "/dev/car".into(),
        lidar_device: "/dev/laser".into(),
        #[cfg(feature = "ros2")]
        lidar_calibration: None,
        max_wait_s: 300.,
        light_max_wait_s: None,
        status_s: 0.5,
        frame_shm: None,
        surface_initial_load: 0.,
        ros2: cfg!(feature = "ros2"),
        #[cfg(feature = "ros2")]
        ros_args,
    };
    let mut i = 0;
    let value = |i: usize| args.get(i + 1).cloned().ok_or_else(|| format!("{} needs a value", args[i]));
    let number = |i: usize| -> Result<f64, String> {
        let v = value(i)?;
        v.parse::<f64>().ok().filter(|x| x.is_finite() && *x >= 0.).ok_or_else(|| format!("{}: invalid {v}", args[i]))
    };
    while i < args.len() {
        match args[i].as_str() {
            "--bridge" => o.bridge = value(i)?,
            "--out" => o.out = PathBuf::from(value(i)?),
            "--max-drive-s" => o.max_drive_s = number(i)?,
            "--initial-presteer-pwm" => {
                o.initial_presteer_pwm = value(i)?.parse().map_err(|_| "--initial-presteer-pwm: invalid".to_string())?
            }
            "--start" => o.start = value(i)?,
            "--ready-hold-s" => o.ready_hold_s = number(i)?,
            "--max-wait-s" => o.max_wait_s = number(i)?,
            "--light-max-wait-s" => o.light_max_wait_s = Some(number(i)?),
            "--status-s" => o.status_s = number(i)?,
            "--control-device" => o.control_device = value(i)?,
            "--lidar-device" => o.lidar_device = value(i)?,
            #[cfg(feature = "ros2")]
            "--lidar-calibration" => o.lidar_calibration = Some(PathBuf::from(value(i)?)),
            "--frame-shm" => o.frame_shm = Some(PathBuf::from(value(i)?)),
            "--surface-initial-load" => o.surface_initial_load = number(i)?,
            "--transport" => o.ros2 = match value(i)?.as_str() {
                "legacy" => false,
                "ros2" if cfg!(feature = "ros2") => true,
                _ => return Err("transport must be legacy, or ros2 in a ROS2-enabled build".into()),
            },
            "--demo" => {
                o.demo = true;
                i += 1;
                continue;
            }
            other => return Err(format!("unknown option {other}")),
        }
        i += 2;
    }
    if o.out.as_os_str().is_empty() { return Err("--out is required".into()); }
    if !o.ros2 && o.bridge.is_empty() { return Err("legacy transport requires --bridge".into()); }
    if !["enter", "ready", "never", "ros"].contains(&o.start.as_str()) || (o.start == "ros" && !o.ros2) {
        return Err("--start must be enter, ready, never, or ros with ROS2 transport".into());
    }
    // autonomy-control.py's route drive limit (1..ROUTE_MAX_DRIVE_S).
    if !(1. ..=ROUTE_MAX_DRIVE_S).contains(&o.max_drive_s) {
        return Err(format!("drive limit must be 1..{ROUTE_MAX_DRIVE_S} seconds"));
    }
    Ok(o)
}

/// ROS launch appends a --ros-args block. Pass that block to rcl, while the
/// existing race parser keeps ownership of its own options after an optional --.
#[cfg(any(feature = "ros2", test))]
pub(crate) fn split_ros_args(args: &[String]) -> (Vec<String>, Vec<String>) {
    let (mut plain, mut ros) = (Vec::new(), Vec::new());
    let mut in_ros = false;
    for arg in args {
        if arg == "--ros-args" { in_ros = true; ros.push(arg.clone()); }
        else if in_ros {
            ros.push(arg.clone());
            if arg == "--" { in_ros = false; }
        } else { plain.push(arg.clone()); }
    }
    (plain, ros)
}

pub(crate) enum Event {
    Line(&'static str, String, Instant),
    RustCrosswalk(crate::race_crosswalk_vision::RustCrosswalkObs, Instant),
    RustLamp(crate::race_traffic_light::Observation, Instant),
    Coast(u64, String),
    Ended(&'static str, String),
    Enter,
    StdinClosed,
    #[cfg(feature = "ros2")]
    Halt(&'static str),
}

pub(crate) type CommandWriter = Box<dyn FnMut(&crate::supervisor::Emit) -> Result<(), String>>;

pub(crate) struct Transport {
    pub writer: Option<CommandWriter>,
    pub children: Vec<(&'static str, Child)>,
    #[cfg(feature = "ros2")]
    pub ros2: Option<crate::race_ros2::Controller>,
}

impl Transport {
    fn legacy(o: &Options, tx: &Sender<Event>) -> Result<Self, String> {
        let mut control = spawn(o, "control", &o.control_device, "/dev/ttyUSB1")?;
        let mut lidar = match spawn(o, "lidar", &o.lidar_device, "/dev/ttyACM0") {
            Ok(child) => child,
            Err(error) => { drop(control.stdin.take()); let _ = control.wait(); return Err(error); }
        };
        reader("control", control.stdout.take().unwrap(), tx.clone());
        reader("lidar", lidar.stdout.take().unwrap(), tx.clone());
        let mut input = control.stdin.take().unwrap();
        Ok(Self {
            writer: Some(Box::new(move |emit| {
                let mut text = to_json(&emit.row());
                text.push('\n');
                input.write_all(text.as_bytes()).and_then(|_| input.flush()).map_err(|e| e.to_string())
            })),
            children: vec![("control", control), ("lidar", lidar)],
            #[cfg(feature = "ros2")]
            ros2: None,
        })
    }

    fn close(&mut self) {
        self.writer = None;
        for (_, child) in &mut self.children {
            drop(child.stdin.take());
            let deadline = Instant::now() + Duration::from_secs(3);
            loop {
                match child.try_wait() {
                    Ok(Some(_)) => break,
                    Ok(None) if Instant::now() < deadline => thread::sleep(Duration::from_millis(20)),
                    _ => { let _ = child.kill(); let _ = child.wait(); break; }
                }
            }
        }
        #[cfg(feature = "ros2")]
        if let Some(ros2) = self.ros2.take() { ros2.close(); }
    }
}

pub(crate) fn check_device(device: &str, expected: &str) -> Result<(), String> {
    let real = std::fs::canonicalize(device).map_err(|e| format!("device {device}: {e}"))?;
    if real != Path::new(expected) { return Err(format!("device identity changed: {device}")); }
    let check = Command::new("fuser").arg(device).stdout(Stdio::null()).stderr(Stdio::null()).status();
    if !matches!(check.map(|s| s.code()), Ok(Some(1))) {
        return Err(format!("device busy or owner check failed: {device} (stop the console service first)"));
    }
    Ok(())
}

/// The console's spawn(mode, device): device identity and ownership checks,
/// then the bridge in its own process group.
fn spawn(o: &Options, mode: &'static str, device: &str, expected: &str) -> Result<Child, String> {
    spawn_bridge(&o.bridge, o.demo, mode, device, expected)
}

pub(crate) fn spawn_bridge(bridge: &str, demo: bool, mode: &'static str, device: &str, expected: &str) -> Result<Child, String> {
    let mut command = Command::new(bridge);
    command.arg(mode);
    if !demo {
        check_device(device, expected)?;
        command.args(["--device", device, "--execute"]);
    }
    command
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit())
        .process_group(0)
        .spawn()
        .map_err(|e| format!("{mode} bridge: {e}"))
}

fn reader(kind: &'static str, stream: impl Read + Send + 'static, tx: Sender<Event>) {
    spawn_named(&format!("read-{kind}"), move || {
        let mut reader = BufReader::with_capacity(LINE_LIMIT, stream);
        loop {
            let mut line = String::new();
            match reader.by_ref().take(LINE_LIMIT as u64 + 1).read_line(&mut line) {
                Ok(0) => break,
                Ok(_) if line.len() > LINE_LIMIT => break,
                Ok(_) => {
                    if tx.send(Event::Line(kind, line, Instant::now())).is_err() {
                        return;
                    }
                }
                Err(_) => break,
            }
        }
        let _ = tx.send(Event::Ended(kind, format!("{kind} bridge ended")));
    });
}

pub(crate) enum FileOp {
    Put(&'static str, String),
    Append(&'static str, String),
    Stop,
}

pub(crate) fn writer(dir: PathBuf) -> (Sender<FileOp>, thread::JoinHandle<()>) {
    let (tx, rx) = mpsc::channel::<FileOp>();
    let handle = spawn_named("evidence-write", move || {
        let mut files: Vec<(&'static str, std::fs::File)> = Vec::new();
        for message in rx {
            match message {
                FileOp::Put(name, text) => {
                    let _ = std::fs::write(dir.join(name), text);
                }
                FileOp::Append(name, text) => {
                    if !files.iter().any(|(n, _)| *n == name)
                        && let Ok(f) = std::fs::OpenOptions::new().create(true).append(true).open(dir.join(name))
                    {
                        files.push((name, f));
                    }
                    if let Some((_, f)) = files.iter_mut().find(|(n, _)| *n == name) {
                        let _ = f.write_all(text.as_bytes());
                        let _ = f.flush();
                    }
                }
                FileOp::Stop => break,
            }
        }
    });
    (tx, handle)
}

/// This thread's CPU run time (Linux /proc/thread-self/schedstat, ns), if available.
fn thread_cpu_s() -> Option<f64> {
    let text = std::fs::read_to_string("/proc/thread-self/schedstat").ok()?;
    text.split_whitespace().next()?.parse::<f64>().ok().map(|ns| ns / 1e9)
}

/// CPU time so far (schedstat run time, s) of each thread of this program
/// (by thread name) and of each bridge process (all its threads); empty
/// without /proc. Exact in sum, unlike one tick's reading.
fn cpu_account(bridges: &[(&'static str, u32)]) -> Vec<(String, f64)> {
    fn tasks(pid: &str) -> Vec<(String, f64)> {
        let Ok(dir) = std::fs::read_dir(format!("/proc/{pid}/task")) else {
            return Vec::new();
        };
        let mut out = Vec::new();
        for entry in dir.flatten() {
            let path = entry.path();
            let run = std::fs::read_to_string(path.join("schedstat"))
                .ok()
                .and_then(|t| t.split_whitespace().next()?.parse::<f64>().ok());
            if let Some(ns) = run {
                let name = std::fs::read_to_string(path.join("comm")).unwrap_or_default();
                out.push((name.trim().to_string(), ns / 1e9));
            }
        }
        out
    }
    let mut out: Vec<(String, f64)> = Vec::new();
    let mut add = |name: String, cpu: f64| match out.iter_mut().find(|(n, _)| *n == name) {
        Some(slot) => slot.1 += cpu,
        None => out.push((name, cpu)),
    };
    for (name, cpu) in tasks("self") {
        add(format!("program/{name}"), cpu);
    }
    for (mode, pid) in bridges {
        for (_, cpu) in tasks(&pid.to_string()) {
            add(format!("bridge/{mode}"), cpu);
        }
    }
    out
}

/// The CPU used between two accounts over `wall` seconds, per thread or bridge.
fn cpu_report(start: &[(String, f64)], end: &[(String, f64)], wall: f64) -> V {
    if end.is_empty() {
        return V::None;
    }
    let mut used = V::dict();
    for (name, at_end) in end {
        let at_start = start.iter().find(|(n, _)| n == name).map_or(0., |(_, c)| *c);
        let cpu = at_end - at_start;
        used.set(name, dict! {"cpu_s" => cpu, "core_fraction" => if wall > 0. { cpu / wall } else { 0. }});
    }
    let load = std::fs::read_to_string("/proc/loadavg").unwrap_or_default();
    dict! {"wall_s" => wall, "used" => used, "loadavg_end" => load.trim(),
           "basis" => "schedstat run time per thread over the drive loop (program threads by name; each bridge: all its threads)"}
}

fn unix_s() -> f64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs_f64()).unwrap_or(0.)
}

fn token() -> String {
    let nanos = SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_nanos()).unwrap_or(0);
    format!("{:x}{:05x}", nanos, std::process::id())
}

fn opt_f(value: Option<f64>) -> V {
    value.map(V::Float).unwrap_or(V::None)
}

struct Program {
    o: Options,
    sup: Supervisor,
    base: Instant,
    base_monotonic_s: Option<f64>,
    imu: std::sync::Arc<crate::race_imu::LiveImu>,
    rx: Receiver<Event>,
    files: Sender<FileOp>,
    failed: Vec<&'static str>,
    entered: bool,
    stdin_closed: bool,
    max_drive_v: V,
    initial_v: V,
    coast: Sender<(u64, String)>,
    bridge_pids: Vec<(&'static str, u32)>,
    /// Receive time of the newest scan no tick has decided on yet.
    scan_at: Option<f64>,
    /// Round y2: the newest light frame (control['light']), and the counts.
    light: V,
    light_updates: i64,
    vision_phase: Option<&'static str>,
    lamp_worker: Option<Sender<bool>>,
    rust_cw_hint: std::sync::Arc<std::sync::Mutex<crate::race_crosswalk_vision::LaneLidarHint>>,
    rust_cw_stop: std::sync::Arc<std::sync::atomic::AtomicBool>,
    rust_cw_stats: std::sync::Arc<std::sync::Mutex<crate::race_crosswalk_vision::RustCrosswalkStats>>,
    prestart_rust_cw: Vec<(crate::race_crosswalk_vision::RustCrosswalkObs, f64)>,
    external_stop: Option<&'static str>,
}

/// Pure decision for `--start ready` launch timing:
/// launch immediately once sensors are ready and crosswalk distance is computed;
/// otherwise launch after `ready_hold_s` (3 s) so missing distance never blocks launch.
fn ready_launch_due(turn_ready: bool, crosswalk_ready: bool, elapsed_s: f64, ready_hold_s: f64) -> bool {
    turn_ready && (crosswalk_ready || elapsed_s >= ready_hold_s)
}

/// Whether the pure-Rust crosswalk worker should remain active while driving:
/// only while still in `start_lane` AND crosswalk distance has not yet been computed.
fn rust_crosswalk_needed_in_motion(
    route_stage: &str,
    crosswalk_state: Option<&str>,
    planned: Option<f64>,
    board_m: Option<f64>,
) -> bool {
    route_stage == "start_lane"
        && planned.is_none()
        && board_m.is_none()
        && !matches!(crosswalk_state, Some("waiting" | "done" | "released" | "passed"))
}

/// thread::spawn with a name (it shows in /proc and in the CPU account).
pub(crate) fn spawn_named<T: Send + 'static>(name: &str, f: impl FnOnce() -> T + Send + 'static) -> thread::JoinHandle<T> {
    thread::Builder::new().name(name.to_string()).spawn(f).expect("thread spawn")
}

impl Program {
    fn prestart_crosswalk_fixes(&self) -> usize {
        let (min, max) = crate::maneuver::CROSSWALK_RANGE_M;
        self.prestart_rust_cw.iter().filter(|(obs, _)| (min..=max).contains(&obs.front_to_edge_m)).count()
    }

    fn prestart_crosswalk_ready(&self) -> bool {
        self.prestart_crosswalk_fixes() >= crate::maneuver::CROSSWALK_FIX_FRAMES
    }

    fn stop_rust_crosswalk(&mut self, reason: &str) {
        if !self.rust_cw_stop.swap(true, std::sync::atomic::Ordering::AcqRel) {
            self.say(reason);
        }
        self.publish_vision_phase();
    }

    /// The camera exports raw frames for crosswalk ranging and again at the gantry.
    /// The crosswalk worker stops once resolved. The lamp worker sleeps between phases.
    fn publish_vision_phase(&mut self) {
        let rust_stopped = self.rust_cw_stop.load(std::sync::atomic::Ordering::Acquire);
        let phase = match self.sup.session.as_ref() {
            None => {
                if self.o.frame_shm.is_some() && !rust_stopped && !self.prestart_crosswalk_ready() {
                    "start_lane"
                } else {
                    "drive"
                }
            }
            Some(s) if s.motion.light_stop_state == Some("waiting") => "light_wait",
            Some(s) if self.lamp_worker.is_some() && matches!(s.motion.route_stage, "light_entry" | "light_lane")
                && !matches!(s.motion.light_stop_state, Some("released" | "passed"))
                && s.motion.phase != "coast" && s.motion.phase != "locked" => "light_approach",
            Some(s)
                if !rust_stopped
                    && rust_crosswalk_needed_in_motion(
                        s.motion.route_stage,
                        s.motion.crosswalk_state,
                        s.motion.race_crosswalk_stop.planned,
                        s.motion._crosswalk_board_m,
                    ) =>
            {
                "start_lane"
            }
            Some(_) => "drive",
        };
        if phase != "start_lane" && !self.rust_cw_stop.swap(true, std::sync::atomic::Ordering::AcqRel) {
            self.say("crosswalk distance resolved or passed; stopping rust crosswalk detector");
        }
        if self.o.frame_shm.is_none() {
            return;
        }
        if self.vision_phase != Some(phase) {
            let active = crate::race_traffic_light::lamp_phase(Some(phase));
            if active != crate::race_traffic_light::lamp_phase(self.vision_phase)
                && let Some(worker) = &self.lamp_worker {
                let _ = worker.send(active);
            }
            self.vision_phase = Some(phase);
            let _ = self.files.send(FileOp::Put("vision-phase.txt", format!("{phase}\n")));
        }
    }

    fn mono(&self) -> f64 {
        self.base.elapsed().as_secs_f64()
    }

    fn at(&self, instant: Instant) -> f64 {
        instant.saturating_duration_since(self.base).as_secs_f64()
    }

    fn apply_light(&mut self, light: V) {
        if self.light.get("frame").oi().zip(light.get("frame").oi()).is_some_and(|(old, new)| new <= old) {
            return;
        }
        self.light_updates += 1;
        let record = dict! {"received_at" => self.mono(), "light" => light.clone()};
        let _ = self.files.send(FileOp::Append("light.jsonl", to_json(&record) + "\n"));
        self.sup.set_camera_light(&light);
        self.light = light;
    }

    /// One bridge line or event, as read_bridge applies it under the console lock;
    /// true for an applied lidar scan.
    fn handle(&mut self, event: Event) -> bool {
        match event {
            Event::Line(kind, line, received) => {
                if self.failed.contains(&kind) {
                    return false;
                }
                let now = self.at(received);
                self.sup.mono = self.mono();
                let applied = py_try(|| {
                    let value = parse_json(line.trim_end()).unwrap_or_else(|e| value_error(e));
                    if kind == "control" {
                        self.sup.accept_control(&value, now);
                        if self.light.is_some() {
                            self.sup.set_light(&self.light);
                        }
                    } else {
                        if self.o.demo && let Some(pair) = value.get("sim_imu").ol() && pair.len() == 2
                            && let (Some(yaw), Some(dv)) = (pair[0].of(), pair[1].of()) {
                            let at_ms = (now * 1000.) as u64;
                            self.imu.yaw.publish(at_ms, yaw);
                            self.imu.forward_integral.publish(at_ms, dv);
                        }
                        self.sup.accept_lidar(value, now);
                        if !self.rust_cw_stop.load(std::sync::atomic::Ordering::Relaxed)
                            && let Some(hint) = crate::race_crosswalk_vision::LaneLidarHint::from_scan(&self.sup.scan)
                            && let Ok(mut slot) = self.rust_cw_hint.lock()
                        {
                            *slot = hint;
                        }
                    }
                });
                match applied {
                    Ok(()) => {
                        if kind == "lidar" {
                            self.scan_at = Some(now);
                            return true;
                        }
                        return false;
                    }
                    Err(error) => {
                        self.failed.push(kind);
                        self.sup.bridge_failed(kind, &error.msg);
                    }
                }
            }
            Event::Ended(kind, message) => {
                if !self.failed.contains(&kind) {
                    self.failed.push(kind);
                    self.sup.bridge_failed(kind, &message);
                }
            }
            Event::Coast(number, answer) => {
                self.sup.coast_worker.finish_job(number, &answer);
                self.dispatch();
            }
            Event::RustLamp(obs, received) => {
                let now = self.at(received);
                let captured = self.base_monotonic_s.map(|base| obs.captured_mono_s - base)
                    .unwrap_or(now - obs.compute_ms / 1000.);
                let row = dict! {"frame" => obs.frame, "captured_at" => captured, "received_at" => now,
                    "red" => obs.lights.red, "green" => obs.lights.green, "housing_lip_y"=>obs.housing_lip,"compute_ms" => obs.compute_ms,
                    "pinned_cpu" => obs.pinned_cpu.map(|n| n as i64),
                    "capture_to_result_ms" => self.base_monotonic_s.map(|_| (now - captured) * 1000.)};
                let _ = self.files.send(FileOp::Append("rust-light.jsonl", to_json(&row) + "\n"));
                if crate::race_traffic_light::lamp_phase(self.vision_phase)
                    && (0.0..=crate::maneuver::LIGHT_FRAME_MAX_AGE_S).contains(&(self.mono() - captured))
                {
                    let light = dict! {"frame" => obs.frame, "captured_at" => captured,
                        "red" => i64::from(obs.lights.red), "green" => i64::from(obs.lights.green),"housing_lip_y"=>obs.housing_lip};
                    self.apply_light(light);
                }
            }
            Event::RustCrosswalk(obs, received) => {
                let now = self.at(received);
                let captured_program_s = self
                    .base_monotonic_s
                    .map(|bm| obs.captured_mono_s - bm)
                    .filter(|t| t.is_finite() && (-10.0..=now + 1.0).contains(t))
                    .unwrap_or_else(|| (now - obs.compute_ms / 1000.0).max(0.0));
                let mut row = to_json(&dict! {
                    "received_at" => now,
                    "captured_at" => captured_program_s,
                    "frame" => obs.frame,
                    "front_to_edge_m" => obs.front_to_edge_m,
                    "edge_y" => obs.edge_y,
                    "pitch_deg" => obs.pitch_rad.to_degrees(),
                    "pitch_source" => obs.pitch_source,
                    "score" => obs.score,
                    "stripes" => obs.stripes as i64,
                    "gaps" => obs.gaps as i64,
                    "left_gap_x_m" => obs.left_gap_x_m,
                    "gap_to_crosswalk_m" => obs.gap_to_crosswalk_m,
                    "compute_ms" => obs.compute_ms,
                });
                row.push('\n');
                let _ = self.files.send(FileOp::Append("rust-crosswalk.jsonl", row));
                if self.sup.session.is_some() {
                    self.sup.observe_rust_crosswalk(&obs, captured_program_s);
                } else {
                    if !self.prestart_rust_cw.iter().any(|(o, _)| o.frame == obs.frame) {
                        self.prestart_rust_cw.push((obs, captured_program_s));
                        let n = self.prestart_rust_cw.len();
                        if n > 8 {
                            self.prestart_rust_cw.drain(..n - 8);
                        }
                    }
                    if self.prestart_crosswalk_ready() {
                        self.stop_rust_crosswalk("crosswalk distance ready at rest; stopping rust crosswalk detector");
                    }
                }
            }
            Event::Enter => self.entered = true,
            Event::StdinClosed => self.stdin_closed = true,
            #[cfg(feature = "ros2")]
            Event::Halt(reason) => {
                self.external_stop.get_or_insert(reason);
                self.sup.halt(reason);
            }
        }
        false
    }

    /// Hand the coast worker's pending scan to the fitting thread.
    fn dispatch(&mut self) {
        if let Some(job) = self.sup.coast_worker.take_job() {
            let _ = self.coast.send((job.number, job.request));
        }
    }

    /// Wait up to `until` (program clock), applying events as they come.
    fn pump(&mut self, until: f64) {
        self.pump_scan(until, false);
    }

    /// pump; with `on_scan` it returns as soon as a lidar scan was applied.
    /// True when one was.
    fn pump_scan(&mut self, until: f64, on_scan: bool) -> bool {
        let mut scan = false;
        loop {
            let left = until - self.mono();
            if left <= 0. || (on_scan && scan) {
                break;
            }
            match self.rx.recv_timeout(Duration::from_secs_f64(left)) {
                Ok(event) => scan |= self.handle(event),
                Err(RecvTimeoutError::Timeout) => break,
                Err(RecvTimeoutError::Disconnected) => break,
            }
        }
        while let Ok(event) = self.rx.try_recv() {
            scan |= self.handle(event);
        }
        scan
    }

    fn state(&mut self) -> V {
        let now = self.mono();
        let (max_drive, initial) = (self.max_drive_v.clone(), self.initial_v.clone());
        self.sup.state_view(now, &max_drive, &initial)
    }

    fn lidar(&self) -> V {
        dict! {"scan" => self.sup.scan.clone(), "age_s" => self.mono() - self.sup.scan_at.f(),
               "received_program_s" => self.sup.scan_at.clone()}
    }

    fn row(&mut self, started: f64) -> String {
        let state = self.state();
        let row = dict! {"elapsed_s" => self.mono() - started, "state" => state, "lidar" => self.lidar(),
                         "state_reused" => false};
        let mut text = to_json(&row);
        text.push('\n');
        text
    }

    fn say(&self, text: &str) {
        println!("{:8.2} {text}", self.mono());
        let _ = std::io::stdout().flush();
    }

    fn start_failure(&self, code: i32, reason: &str, mut details: V) -> i32 {
        if let Some(limit) = self.o.light_max_wait_s { details.set("light_max_wait_s", limit); }
        details.set("started", false);
        details.set("reason", reason);
        details.set("sensor_errors", self.sup.errors.clone());
        let _ = self.files.send(FileOp::Put("cli-result.json", to_json(&details)));
        code
    }
}

fn summary_line(report: &V) -> String {
    let turn = report.get("turn");
    format!(
        "phase={:<12} stage={:<15} M={:<4} S={:<4} {}",
        report.get("phase").os().unwrap_or("-"),
        turn.get("route_stage").os().unwrap_or("-"),
        to_json(report.get("current_pwm")),
        to_json(report.get("servo")),
        turn.get("reason").os().unwrap_or(""),
    )
}

/// `route-engine race ...`; the exit status.
pub fn main(args: &[String]) -> i32 {
    let o = match parse(args) {
        Ok(o) => o,
        Err(e) => {
            eprintln!("{e}\n{USAGE}");
            return 64;
        }
    };
    if o.out.exists() {
        eprintln!("evidence directory {} already exists (never overwritten)", o.out.display());
        return 64;
    }
    if let Err(e) = std::fs::create_dir_all(&o.out) {
        eprintln!("evidence directory {}: {e}", o.out.display());
        return 66;
    }
    let (tx, rx) = mpsc::channel::<Event>();
    #[cfg(feature = "ros2")]
    let transport = if o.ros2 { crate::race_ros2::connect(&o, tx.clone()) } else { Transport::legacy(&o, &tx) };
    #[cfg(not(feature = "ros2"))]
    let transport = Transport::legacy(&o, &tx);
    let mut transport = match transport {
        Ok(link) => link,
        Err(error) => {
            eprintln!("{error}");
            let result = dict! {"started" => false, "reason" => error};
            let _ = std::fs::write(o.out.join("cli-result.json"), to_json(&result));
            return 69;
        }
    };
    let rust_cw_hint = std::sync::Arc::new(std::sync::Mutex::new(
        crate::race_crosswalk_vision::LaneLidarHint::default(),
    ));
    let rust_cw_stop = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(o.frame_shm.is_none()));
    let rust_cw_stats = std::sync::Arc::new(std::sync::Mutex::new(
        crate::race_crosswalk_vision::RustCrosswalkStats::default(),
    ));
    let rust_cw_thread = o.frame_shm.as_ref().map(|path| {
        crate::race_crosswalk_vision::spawn_worker(
            path.clone(),
            rust_cw_hint.clone(),
            rust_cw_stop.clone(),
            rust_cw_stats.clone(),
            tx.clone(),
        )
    });
    if o.start == "enter" {
        let tx = tx.clone();
        spawn_named("read-stdin", move || {
            let stdin = std::io::stdin();
            for line in stdin.lock().lines() {
                if line.is_err() || tx.send(Event::Enter).is_err() {
                    break;
                }
            }
            let _ = tx.send(Event::StdinClosed);
        });
    }
    // The coast standstill fit (a worker process in the console) on its own thread.
    let (coast, jobs) = mpsc::channel::<(u64, String)>();
    {
        let tx = tx.clone();
        spawn_named("coast-fit", move || {
            let mut fitter = crate::coast::CoastFitter::default();
            for (number, request) in jobs {
                if tx.send(Event::Coast(number, fitter.fit(&request))).is_err() {
                    break;
                }
            }
        });
    }
    let (lamp_worker, lamp_thread) = o.frame_shm.as_ref().and_then(|path|
        crate::race_traffic_light::spawn(path.clone(), tx.clone())).unzip();
    drop(tx);
    let (files, writer_thread) = writer(o.out.clone());
    let mut sup = Supervisor::new(o.demo, &token(), "rust_program");
    sup.race_program = true;
    sup.unix_time = unix_s;
    sup.camera = false;
    sup.writer = transport.writer.take();
    let max_drive_v = V::Float(o.max_drive_s);
    let initial_v = V::Int(o.initial_presteer_pwm);
    let bridge_pids = transport.children.iter().map(|(name, child)| (*name, child.id())).collect();
    let base = Instant::now();
    #[cfg(target_os = "linux")]
    let base_monotonic_s = {
        let clock = rustix::time::clock_gettime(rustix::time::ClockId::Monotonic);
        Some(clock.tv_sec as f64 + clock.tv_nsec as f64 / 1e9 - base.elapsed().as_secs_f64())
    };
    #[cfg(not(target_os = "linux"))]
    let base_monotonic_s = None;
    let mut p = Program {
        o,
        sup,
        base,
        base_monotonic_s,
        imu: Default::default(),
        rx,
        files,
        failed: Vec::new(),
        entered: false,
        stdin_closed: false,
        max_drive_v,
        initial_v,
        coast,
        bridge_pids,
        scan_at: None,
        light: V::None,
        light_updates: 0,
        vision_phase: None,
        lamp_worker,
        rust_cw_hint,
        rust_cw_stop: rust_cw_stop.clone(),
        rust_cw_stats,
        prestart_rust_cw: Vec::new(),
        external_stop: None,
    };
    p.publish_vision_phase();
    // ROS discovery/setup happens before this clock zero. The helper's spawn
    // timestamp is not an accurate origin for matching vision/IMU evidence.
    let _ = p.files.send(FileOp::Put("clock.json", to_json(&dict! {
        "program_base_monotonic_s" => p.base_monotonic_s,
        "clock" => "Linux CLOCK_MONOTONIC; same origin as Python time.monotonic",
    })));
    let imu_stop = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
    let imu = if p.o.demo {
        None
    } else {
        let (out, base, stop, imu) = (p.o.out.clone(), p.base, imu_stop.clone(), p.imu.clone());
        thread::Builder::new().name("imu-log".into())
            .spawn(move || crate::race_imu::record(&out, base, &stop, &imu))
            .inspect_err(|e| eprintln!("IMU recorder: {e}")).ok()
    };
    let code = run(&mut p);
    drop(p.lamp_worker.take());
    rust_cw_stop.store(true, std::sync::atomic::Ordering::Release);
    imu_stop.store(true, std::sync::atomic::Ordering::Relaxed);
    let _ = p.files.send(FileOp::Stop);
    let _ = writer_thread.join();
    // Closing the bridges' input: each stops at neutral and exits.
    p.sup.writer = None;
    transport.close();
    if let Some(thread) = imu {
        let _ = thread.join();
    }
    if let Some(thread) = rust_cw_thread {
        let _ = thread.join();
    }
    if let Some(thread) = lamp_thread { let _ = thread.join(); }
    code
}

fn run(p: &mut Program) -> i32 {
    // First feedback from both bridges.
    let deadline = p.mono() + 5.;
    while (p.sup.scan.is_none() || !p.sup.status.has("control")) && p.mono() < deadline {
        let until = p.mono() + 0.05;
        p.pump(until);
        if let Some(reason) = p.external_stop { return p.start_failure(130, reason, V::dict()); }
    }
    if p.sup.scan.is_none() || !p.sup.status.has("control") {
        p.say("no lidar scan or control feedback within 5 s");
        return p.start_failure(69, "initial_feedback_timeout", dict! {
            "lidar_received" => p.sup.scan.is_some(), "control_received" => p.sup.status.has("control"),
        });
    }
    p.say(&format!("evidence {}", p.o.out.display()));
    // Start check, shown until it passes; then the trigger.
    // Crosswalk detection runs immediately; once distance is ready, stop the Rust worker and
    // launch right away; otherwise launch after ready_hold_s (3 s) and continue computing in motion.
    let wait_t0 = p.mono();
    let wait_until = wait_t0 + p.o.max_wait_s;
    let mut next_status = 0.;
    let mut prompted = false;
    let before = loop {
        let until = p.mono() + 0.1;
        p.pump(until);
        if let Some(reason) = p.external_stop { return p.start_failure(130, reason, V::dict()); }
        let now = p.mono();
        p.sup.health_watch(now);
        let state = p.state();
        let autonomy = state.idx("autonomy");
        let ready = autonomy.idx("turn_ready").is_true();
        let cw_ready = p.prestart_crosswalk_ready();
        if cw_ready {
            p.stop_rust_crosswalk("crosswalk distance ready at rest; stopping rust crosswalk detector");
        }
        if now >= next_status {
            let clearance = autonomy.get("turn_clearance");
            let front = clearance.get("front_m");
            let fixes = p.prestart_crosswalk_fixes();
            p.say(&format!(
                "start check: {} front={} cw_fixes={}",
                if ready { "ready".to_string() } else { format!("not ready ({})", to_json(autonomy.get("turn_rejection"))) },
                to_json(front),
                fixes,
            ));
            next_status = now + p.o.status_s.max(0.1);
        }
        if matches!(p.o.start.as_str(), "enter" | "ros") {
            if ready && !prompted {
                p.say(if p.o.start == "ros" { "READY: waiting for /xt_stcar/start=true (stop topic cancels)" }
                    else { "READY: press Enter to start the route (Ctrl-C quits; the car stays locked)" });
                prompted = true;
            }
            if p.entered {
                p.entered = false;
                if ready {
                    break state;
                }
                p.say("not ready; Enter again once the start check passes");
            }
            if p.o.start == "enter" && p.stdin_closed && !p.entered {
                p.say("terminal input closed; use --start ready without a terminal");
                return p.start_failure(64, "terminal_input_closed", V::dict());
            }
        } else if p.o.start == "ready" && ready_launch_due(ready, cw_ready, now - wait_t0, p.o.ready_hold_s) {
            if !cw_ready {
                p.say(&format!(
                    "crosswalk distance not yet computed after {:.1}s; starting and continuing detection in motion",
                    now - wait_t0
                ));
            }
            break state;
        }
        if p.o.start == "never" && now >= wait_until {
            // Check only: record what the check saw; nothing was armed.
            let _ = p.files.send(FileOp::Put("before.json", to_json(&state)));
            let _ = p.files.send(FileOp::Put("before-lidar.json", to_json(&p.lidar())));
            let result = dict! {"started" => false, "check_only" => true, "turn_ready" => ready,
                                "light_max_wait_s" => p.o.light_max_wait_s,
                                "turn_rejection" => state.idx("autonomy").get("turn_rejection").clone(),
                                "control" => p.sup.control(), "errors" => p.sup.errors.clone()};
            let _ = p.files.send(FileOp::Put("cli-result.json", to_json(&result)));
            p.say("check only: not started");
            return 0;
        }
        if now >= wait_until {
            let reason = autonomy.get("turn_rejection").os().unwrap_or("start_trigger_timeout");
            p.say(&format!("start check did not pass in time: {reason}"));
            let _ = p.files.send(FileOp::Put("before.json", to_json(&state)));
            let _ = p.files.send(FileOp::Put("before-lidar.json", to_json(&p.lidar())));
            return p.start_failure(75, reason, dict! {"turn_ready" => ready});
        }
    };
    assert!(p.o.start != "never", "check-only mode never reaches the start");
    let _ = p.files.send(FileOp::Put("before.json", to_json(&before)));
    let _ = p.files.send(FileOp::Put("before-lidar.json", to_json(&p.lidar())));
    let control = p.sup.control();
    let data = dict! {
        "op" => "turn_cone_start", "boot" => p.sup.boot.as_str(), "epoch" => p.sup.control_epoch,
        "tick" => control.get("tick").clone(), "placement_confirmed" => true, "continue_route" => true,
        "max_drive_s" => p.max_drive_v.clone(), "initial_presteer_pwm" => p.initial_v.clone(),
    };
    let run_id = token();
    let now = p.mono();
    p.sup.mono = now;
    let started = match p.sup.start_turn_cone(&data, &V::Float(now), &run_id) {
        Ok(result) => result,
        Err((error, clearance)) => {
            p.say(&format!("start refused: {}", error.msg));
            return p.start_failure(2, &error.msg, dict! {"error" => error.msg.as_str(), "clearance" => clearance});
        }
    };
    if let Some(session) = &mut p.sup.session {
        session.motion.race_inputs = Some(Default::default());
        session.motion.race_light_wait_s = p.o.light_max_wait_s;
        session.motion.field_map.planner.start();
        session.motion.race_surface.set_initial_load(p.o.surface_initial_load);
        // Enable crosswalk odometry immediately when pure-Rust vision is attached so
        // moving fallback always has lidar rear-wall history before the first optical fix.
        if p.o.frame_shm.is_some() {
            session.motion.light_vision = true;
        }
        // Replay standstill Rust crosswalk fixes captured before launch.
        for (obs, cap_s) in std::mem::take(&mut p.prestart_rust_cw) {
            session.motion.rust_crosswalk_observe(&obs, cap_s);
        }
    }
    p.say(&format!("started run {run_id}"));
    let run_v = V::from(run_id.as_str());
    let t0 = now;
    // The console's autonomy_watch with its pacing and timing records.
    let mut wait = TICK_S;
    let mut previous: Option<(f64, f64, f64, Option<f64>)> = None;
    let mut next_tick = p.mono() + wait;
    let (mut next_health, mut next_evidence, mut next_status) = (p.mono(), p.mono(), p.mono());
    let (mut last_end, mut scan_woken) = (p.mono(), false);
    let bridge_pids = p.bridge_pids.clone();
    let cpu_start = cpu_account(&bridge_pids);
    let cpu_t0 = p.mono();
    // Receive -> deciding tick, per new scan.
    let (mut waits, mut wait_sum, mut wait_max) = (0_i64, 0., 0_f64);
    p.scan_at = None;
    while p.sup.session.is_some() {
        let until = next_tick.min(next_health);
        if p.pump_scan(until, true) {
            // A new scan: decide on it now, at least MIN_IDLE_S after the last tick.
            let soon = p.mono().max(last_end + MIN_IDLE_S);
            if soon < next_tick {
                next_tick = soon;
                scan_woken = true;
            }
        }
        let now = p.mono();
        if now >= next_health {
            p.sup.renew_owner(now);
            p.sup.health_watch(now);
            next_health = now + HEALTH_S;
        }
        if p.sup.session.is_none() {
            break;
        }
        if now < next_tick {
            continue;
        }
        let requested = p.mono();
        let tick_at = p.mono();
        let cpu0 = thread_cpu_s();
        p.sup.mono = tick_at;
        if let Some(at) = p.scan_at.take() {
            let waited = (tick_at - at).max(0.);
            (waits, wait_sum, wait_max) = (waits + 1, wait_sum + waited, wait_max.max(waited));
        }
        if let Some(s) = p.sup.session.as_mut()
            && s.motion.race_inputs.is_some()
        {
            s.motion.set_live_imu(crate::motion_sensors::RaceInputs {
                yaw_rate: p.imu.yaw.at(tick_at),
                forward_integral: p.imu.forward_integral.at(tick_at),
            });
        }
        if let Err(error) = py_try(|| p.sup.autonomy_tick(&V::Float(tick_at))) {
            p.sup.halt(&format!("tick_internal_error_{}: {}", error.kind.name(), error.msg));
        }
        let ended = p.mono();
        let cpu = cpu0.and_then(|c0| thread_cpu_s().map(|c1| c1 - c0));
        let breakdown = previous.map(|(start0, end0, idle0, cpu0)| {
            dict! {"gap_s" => tick_at - start0, "previous_tick_wall_s" => end0 - start0,
                   "previous_tick_cpu_s" => opt_f(cpu0), "planned_idle_s" => idle0,
                   "wake_late_s" => requested - (end0 + idle0), "lock_wait_s" => tick_at - requested,
                   "scan_woken" => scan_woken}
        });
        p.sup.record_tick_timing(tick_at - requested, ended - tick_at, cpu, breakdown, &run_v);
        wait = MIN_IDLE_S.max(TICK_S - (ended - requested));
        previous = Some((tick_at, ended, wait, cpu));
        next_tick = ended + wait;
        (last_end, scan_woken) = (ended, false);
        // The scan this tick submitted for the coast fit goes to its thread.
        p.dispatch();
        p.publish_vision_phase();
        // Evidence and the terminal line in the idle time after a tick.
        let now = p.mono();
        if now >= next_evidence && p.sup.session.is_some() {
            let row = p.row(t0);
            let _ = p.files.send(FileOp::Append("feedback.jsonl", row));
            next_evidence = now + EVIDENCE_S;
        }
        if now >= next_status {
            if let Some(s) = &p.sup.session {
                let line = summary_line(&s.report);
                p.say(&line);
            }
            next_status = now + p.o.status_s.max(0.1);
        }
    }
    p.publish_vision_phase();
    let cpu = cpu_report(&cpu_start, &cpu_account(&bridge_pids), p.mono() - cpu_t0);
    let scan_to_tick = dict! {"scans" => waits, "mean_s" => if waits > 0 { wait_sum / waits as f64 } else { 0. },
                              "max_s" => wait_max, "basis" => "reader receive time -> start of the tick that decided on it"};
    let result = p.sup.auto_result.clone();
    let sensor_errors = p.sup.errors.clone();
    p.say(&format!("ended: {}", to_json(result.get("reason"))));
    // After the run: six samples about .2 s apart (the helper's after.json).
    let mut after: VecDeque<V> = VecDeque::new();
    for _ in 0..6 {
        let until = p.mono() + 0.2;
        p.pump(until);
        let now = p.mono();
        p.sup.health_watch(now);
        let state = p.state();
        after.push_back(dict! {"state" => state, "lidar" => p.lidar()});
    }
    let _ = p.files.send(FileOp::Put("after.json", to_json(&V::list(after.into_iter().collect()))));
    let rust_cw_summary = if let Ok(mut s) = p.rust_cw_stats.lock() {
        let mut ms = s.compute_ms.clone();
        ms.sort_by(|a, b| a.total_cmp(b));
        let med = if ms.is_empty() { None } else { Some(ms[ms.len() / 2]) };
        let mx = ms.last().copied();
        s.compute_ms.shrink_to_fit();
        dict! {
            "enabled" => p.o.frame_shm.is_some(),
            "frames_read" => s.frames_read,
            "detections" => s.detections,
            "compute_ms_median" => med,
            "compute_ms_max" => mx,
            "pinned_cpu" => s.pinned_cpu.map(|c| c as i64),
        }
    } else {
        V::None
    };
    let summary = dict! {
        "run_id" => run_v.clone(), "started" => started, "reason" => result.get("reason").clone(),
        "sensor_errors" => sensor_errors, "light_max_wait_s" => p.o.light_max_wait_s,
        "route_controller" => if p.o.ros2 { "ros2_rust" } else { "rust_program" },
        "transport" => if p.o.ros2 { "ros2_topics" } else { "legacy_pipes" },
        "program_base_monotonic_s" => p.base_monotonic_s,
        "max_tick_compute_s" => result.get("max_tick_compute_s").clone(),
        "max_actual_gap_s" => result.get("max_actual_gap_s").clone(), "max_gap_breakdown" => result.get("max_gap_breakdown").clone(),
        "cpu" => cpu, "scan_to_tick" => scan_to_tick, "result" => result,
        "rust_crosswalk" => rust_cw_summary,
        "light" => dict! {"source" => "rust_camera",
                          "updates" => p.light_updates, "last" => p.light.clone()},
    };
    let _ = p.files.send(FileOp::Put("summary.json", to_json(&summary)));
    let _ = p.files.send(FileOp::Put("cli-result.json", to_json(&summary)));
    0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ros_launch_arguments_preserve_race_options_after_the_ros_block() {
        let args = ["--demo", "--ros-args", "-r", "__node:=review_node", "--", "--start", "never"].map(String::from);
        let (plain, ros) = split_ros_args(&args);
        assert_eq!(plain, ["--demo", "--start", "never"]);
        assert_eq!(ros, ["--ros-args", "-r", "__node:=review_node", "--"]);
    }

    #[test]
    fn prestart_launches_early_when_crosswalk_ready_else_after_3s_without_blocking() {
        assert!(!ready_launch_due(true, false, 0.8, 3.0));
        assert!(ready_launch_due(true, true, 0.8, 3.0));
        assert!(ready_launch_due(true, false, 3.0, 3.0));
        assert!(!ready_launch_due(false, true, 3.5, 3.0));
    }

    #[test]
    fn rust_crosswalk_worker_runs_in_motion_only_until_distance_is_resolved() {
        assert!(rust_crosswalk_needed_in_motion("start_lane", Some("search"), None, None));
        assert!(!rust_crosswalk_needed_in_motion("start_lane", Some("approach"), Some(-2.4), None));
        assert!(!rust_crosswalk_needed_in_motion("start_lane", Some("approach"), None, Some(-2.4)));
        assert!(!rust_crosswalk_needed_in_motion("start_lane", Some("waiting"), None, None));
        assert!(!rust_crosswalk_needed_in_motion("first_target", Some("released"), None, None));
    }
}
