//! `route-engine tdrag`: one straight push and coast to measure a floor's drag before a race
//! (user, 2026-11-08). scripts/t-ground-drag.py runs it on the car and does the sums.
//!
//! Rest (the IMU forward bias at this spot), push at one PWM, neutral coast to a stop, rest
//! again. The gyro holds the heading; there is no route logic. Motion ends early on the free
//! length given, on a return ahead inside the stopping distance, or on a return at the body.
//! The bridges stop by themselves if this program stalls (their 300 ms command lease).
use std::io::{BufRead, BufReader, Read, Write};
use std::path::PathBuf;
use std::process::Child;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender};
use std::thread;
use std::time::{Duration, Instant};

use crate::dict;
use crate::helpers::{FRONT_BODY_EXTENT_M, NEUTRAL, REAR_BODY_EXTENT_M, SIDE_BODY_EXTENT_M, SCAN_AGE_S, dcos, dsin};
use crate::maneuver::LANE_BRAKE_MOTOR;
use crate::py::*;
use crate::race::{FileOp, spawn_bridge, spawn_named, writer};
use crate::race_imu::LiveImu;

const USAGE: &str = "usage: route-engine tdrag --bridge PATH --out NEW_DIR --length-m METERS [--motor 1570] [--push-s 1.6] [--rest-s 2] [--demo]";
const TICK_S: f64 = 0.02;
const REST_SKIP_S: f64 = 0.3; // the start of a rest is not yet still
const REST_AFTER_S: f64 = 1.2;
const COAST_MAX_S: f64 = 6.;
const STOPPED_MPS: f64 = 0.08;
const STOPPED_HOLD_S: f64 = 0.3;
const MAX_SPEED_MPS: f64 = 1.8;
const LENGTH_MARGIN_M: f64 = 0.3;
const CUT_LAG_S: f64 = 0.25; // the ESC's run-on after neutral (y-runs: 0-.3 s)
const COAST_DECEL_MPS2: f64 = 0.45; // weaker than the indoor .6: this floor is unknown
const BRAKE_DECEL_MPS2: f64 = 1.2;
const BRAKE_MAX_S: f64 = 1.;
const BODY_MARGIN_M: f64 = 0.06;
const FRONT_HALF_BINS: i64 = 8;
const FRONT_MIN_RETURNS: usize = 4;
const HEADING_PWM_PER_RAD: f64 = 300.;
const HEADING_PWM_MAX: f64 = 80.;
const IMU_STALE_S: f64 = 0.25;

struct Options {
    bridge: String,
    out: PathBuf,
    length_m: f64,
    motor: i64,
    push_s: f64,
    rest_s: f64,
    demo: bool,
}

fn parse(args: &[String]) -> Result<Options, String> {
    let mut o = Options { bridge: String::new(), out: PathBuf::new(), length_m: 0., motor: 1570, push_s: 1.6, rest_s: 2., demo: false };
    let mut i = 0;
    while i < args.len() {
        if args[i] == "--demo" {
            o.demo = true;
            i += 1;
            continue;
        }
        let value = args.get(i + 1).ok_or_else(|| format!("{} needs a value", args[i]))?;
        let number = || value.parse::<f64>().ok().filter(|x| x.is_finite() && *x > 0.).ok_or_else(|| format!("{}: invalid {value}", args[i]));
        match args[i].as_str() {
            "--bridge" => o.bridge = value.clone(),
            "--out" => o.out = PathBuf::from(value),
            "--length-m" => o.length_m = number()?,
            "--motor" => o.motor = number()? as i64,
            "--push-s" => o.push_s = number()?,
            "--rest-s" => o.rest_s = number()?,
            other => return Err(format!("unknown option {other}")),
        }
        i += 2;
    }
    if o.bridge.is_empty() || o.out.as_os_str().is_empty() || o.length_m == 0. {
        return Err("--bridge, --out and --length-m are required".into());
    }
    if !(1550..=1600).contains(&o.motor) {
        return Err("--motor must be 1550..1600".into());
    }
    Ok(o)
}

/// The median return ahead, if enough rays have one.
pub(crate) fn front_range(ranges: &[V]) -> Option<f64> {
    let mut rays: Vec<f64> = (-FRONT_HALF_BINS..=FRONT_HALF_BINS).filter_map(|i| ranges[i.rem_euclid(360) as usize].of()).collect();
    if rays.len() < FRONT_MIN_RETURNS {
        return None;
    }
    sort_floats(&mut rays);
    Some(rays[rays.len() / 2])
}

/// A return inside the car's outline plus a small margin.
pub(crate) fn body_clear(ranges: &[V]) -> bool {
    ranges.iter().enumerate().all(|(i, r)| {
        r.of().is_none_or(|r| {
            let (x, y) = (r * dcos(i), r * dsin(i));
            !(-REAR_BODY_EXTENT_M - BODY_MARGIN_M < x && x < FRONT_BODY_EXTENT_M + BODY_MARGIN_M
                && y.abs() < SIDE_BODY_EXTENT_M + BODY_MARGIN_M)
        })
    })
}

/// Hold the heading the rest ended with: left positive, a larger servo PWM steers left.
pub(crate) fn steer(heading_rad: f64) -> i64 {
    NEUTRAL - (HEADING_PWM_PER_RAD * heading_rad).clamp(-HEADING_PWM_MAX, HEADING_PWM_MAX).round() as i64
}

/// Whether a push must end here: what the speed needs to coast out inside the free length.
pub(crate) fn out_of_length(distance: f64, speed: f64, length: f64) -> bool {
    distance + speed * CUT_LAG_S + speed * speed / (2. * COAST_DECEL_MPS2) >= length - LENGTH_MARGIN_M
}

#[derive(Clone, Copy, PartialEq, Debug)]
enum Phase {
    Rest1,
    Push,
    Coast,
    Brake,
    Rest2,
}

impl Phase {
    fn name(self) -> &'static str {
        match self {
            Phase::Rest1 => "rest1",
            Phase::Push => "push",
            Phase::Coast => "coast",
            Phase::Brake => "brake",
            Phase::Rest2 => "rest2",
        }
    }
}

enum Event {
    Line(&'static str, String, Instant),
    Ended(&'static str),
}

fn reader(kind: &'static str, stream: impl Read + Send + 'static, tx: Sender<Event>) {
    spawn_named(&format!("read-{kind}"), move || {
        for line in BufReader::with_capacity(65536, stream).lines() {
            let Ok(line) = line else { break };
            if tx.send(Event::Line(kind, line, Instant::now())).is_err() {
                return;
            }
        }
        let _ = tx.send(Event::Ended(kind));
    });
}

struct Run {
    o: Options,
    base: Instant,
    imu: Arc<LiveImu>,
    rx: Receiver<Event>,
    files: Sender<FileOp>,
    bridge_in: std::process::ChildStdin,
    seq: i64,
    control: V,
    scan: V,
    scan_at: f64,
    ended: Option<&'static str>,
    sim: Sim,
}

/// --demo only: a first-order car in place of the IMU.
#[derive(Default)]
struct Sim {
    speed: f64,
    integral: f64,
    last: f64,
}

impl Run {
    fn mono(&self) -> f64 {
        self.base.elapsed().as_secs_f64()
    }

    fn pump(&mut self, until: f64) {
        loop {
            let left = until - self.mono();
            if left <= 0. {
                break;
            }
            match self.rx.recv_timeout(Duration::from_secs_f64(left)) {
                Ok(event) => self.handle(event),
                Err(RecvTimeoutError::Timeout) | Err(RecvTimeoutError::Disconnected) => break,
            }
        }
        while let Ok(event) = self.rx.try_recv() {
            self.handle(event);
        }
    }

    fn handle(&mut self, event: Event) {
        match event {
            Event::Ended(kind) => self.ended = Some(kind),
            Event::Line(kind, line, at) => {
                let Ok(value) = parse_json(line.trim_end()) else { return };
                let received = at.saturating_duration_since(self.base).as_secs_f64();
                if kind == "control" {
                    self.control = value.get("control").clone();
                } else {
                    let mut row = String::from("{\"received_at\":");
                    row.push_str(&received.to_string());
                    row.push_str(",\"row\":");
                    row.push_str(line.trim_end());
                    row.push_str("}\n");
                    let _ = self.files.send(FileOp::Append("lidar.jsonl", row));
                    self.scan = value;
                    self.scan_at = received;
                }
            }
        }
    }

    fn send(&mut self, op: &'static str, motor: i64, servo: i64) -> bool {
        self.seq += 1;
        let row = dict! {"op" => op, "seq" => self.seq, "tick" => self.control.get("tick").clone(), "motor" => motor, "servo" => servo};
        let mut text = to_json(&row);
        text.push('\n');
        self.bridge_in.write_all(text.as_bytes()).and_then(|_| self.bridge_in.flush()).is_ok()
    }

    fn fresh_ranges(&self, now: f64) -> Option<&[V]> {
        let ranges = self.scan.get("ranges");
        (ranges.is_list() && ranges.len() == 360 && now - self.scan_at <= SCAN_AGE_S).then(|| ranges.l())
    }

    fn step_sim(&mut self, now: f64, motor: i64, servo: i64) {
        let dt = now - self.sim.last;
        self.sim.last = now;
        if motor == NEUTRAL && self.sim.speed <= 0. {
            self.sim.speed = 0.;
        } else {
            let accel = if motor == NEUTRAL { -0.75 } else if motor < NEUTRAL { -1.5 } else { 2.34 - self.sim.speed };
            self.sim.speed = (self.sim.speed + accel * dt).max(0.);
            self.sim.integral += accel * dt;
        }
        let at_ms = (now * 1000.) as u64;
        self.imu.forward_integral.publish(at_ms, self.sim.integral);
        self.imu.yaw.publish(at_ms, (servo - NEUTRAL) as f64 * 0.002 * self.sim.speed);
    }
}

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
    let mut children: Vec<Child> = Vec::new();
    for (mode, device, expected) in [("control", "/dev/car", "/dev/ttyUSB1"), ("lidar", "/dev/laser", "/dev/ttyACM0")] {
        match spawn_bridge(&o.bridge, o.demo, mode, device, expected) {
            Ok(child) => children.push(child),
            Err(e) => {
                eprintln!("{e}");
                for child in &mut children {
                    drop(child.stdin.take());
                    let _ = child.wait();
                }
                return 69;
            }
        }
    }
    let (tx, rx) = mpsc::channel();
    reader("control", children[0].stdout.take().unwrap(), tx.clone());
    reader("lidar", children[1].stdout.take().unwrap(), tx);
    let (files, writer_thread) = writer(o.out.clone());
    let bridge_in = children[0].stdin.take().unwrap();
    let base = Instant::now();
    let imu: Arc<LiveImu> = Default::default();
    let imu_stop = Arc::new(AtomicBool::new(false));
    let imu_thread = if o.demo {
        None
    } else {
        let (out, stop, live) = (o.out.clone(), imu_stop.clone(), imu.clone());
        thread::Builder::new().name("imu-log".into()).spawn(move || crate::race_imu::record(&out, base, &stop, &live)).ok()
    };
    let mut run = Run { o, base, imu, rx, files, bridge_in, seq: 0, control: V::None, scan: V::None, scan_at: f64::NEG_INFINITY, ended: None, sim: Sim::default() };
    let summary = drive(&mut run);
    let code = i32::from(summary.get("reason").os() != Some("done"));
    println!("{}", to_json(&summary));
    let _ = run.files.send(FileOp::Put("summary.json", to_json(&summary)));
    let _ = run.send("stop", NEUTRAL, NEUTRAL);
    imu_stop.store(true, Ordering::Relaxed);
    let _ = run.files.send(FileOp::Stop);
    let _ = writer_thread.join();
    drop(run.bridge_in);
    drop(children[1].stdin.take());
    for child in &mut children {
        let deadline = Instant::now() + Duration::from_secs(3);
        loop {
            match child.try_wait() {
                Ok(Some(_)) => break,
                Ok(None) if Instant::now() < deadline => thread::sleep(Duration::from_millis(20)),
                _ => {
                    let _ = child.kill();
                    let _ = child.wait();
                    break;
                }
            }
        }
    }
    if let Some(thread) = imu_thread {
        let _ = thread.join();
    }
    code
}

fn refused(reason: String) -> V {
    dict! {"reason" => "refused", "detail" => reason, "moved" => false}
}

/// The whole run; the returned summary goes to summary.json.
fn drive(r: &mut Run) -> V {
    let deadline = r.mono() + 5.;
    while (r.scan.is_none() || !r.control.is_dict() || (!r.o.demo && r.imu.forward_integral.at(r.mono()).is_none())) && r.mono() < deadline && r.ended.is_none() {
        let until = r.mono() + 0.05;
        r.pump(until);
    }
    let now = r.mono();
    if r.o.demo {
        r.sim.last = now;
        r.step_sim(now, NEUTRAL, NEUTRAL);
    }
    if r.scan.is_none() || !r.control.is_dict() {
        return refused("no lidar scan or control feedback within 5 s".into());
    }
    if r.imu.forward_integral.at(now).is_none() || r.imu.yaw.at(now).is_none() {
        return refused("no IMU within 5 s".into());
    }
    if r.control.get("armed").truthy() || r.control.get("motor").i() != NEUTRAL || r.control.get("servo").i() != NEUTRAL {
        return refused(format!("control is not at a locked neutral: {}", to_json(&r.control)));
    }
    let Some(ranges) = r.fresh_ranges(now) else { return refused("no fresh complete scan".into()) };
    if !r.o.demo {
        if !body_clear(ranges) {
            return refused("something is within 6 cm of the body".into());
        }
        if let Some(front) = front_range(ranges) && front - FRONT_BODY_EXTENT_M < r.o.length_m {
            return refused(format!("only {:.2} m free ahead, less than the {:.2} m given", front - FRONT_BODY_EXTENT_M, r.o.length_m));
        }
    }
    if !r.send("arm", NEUTRAL, NEUTRAL) {
        return refused("control bridge not writable".into());
    }
    let deadline = r.mono() + 1.;
    while !r.control.get("armed").truthy() && r.mono() < deadline && r.ended.is_none() {
        let until = r.mono() + 0.02;
        r.pump(until);
    }
    if !r.control.get("armed").truthy() {
        return refused("the bridge did not arm".into());
    }
    let (mut phase, mut since) = (Phase::Rest1, r.mono());
    let mut spans: Vec<(&'static str, f64, f64)> = Vec::new();
    let mut guard: Option<&'static str> = None;
    let mut rest_start: Option<(f64, f64)> = None;
    let mut bias: Option<f64> = None;
    let mut zero: Option<(f64, f64)> = None;
    let (mut speed, mut distance, mut heading, mut peak_heading, mut peak_speed) = (0_f64, 0_f64, 0_f64, 0_f64, 0_f64);
    let mut stopped_since: Option<f64> = None;
    let mut last = r.mono();
    let mut reason = "done";
    let mut next = r.mono();
    loop {
        next += TICK_S;
        r.pump(next);
        let now = r.mono();
        let dt = now - last;
        last = now;
        if r.ended.is_some() {
            reason = "bridge_ended";
            break;
        }
        if !r.control.get("armed").truthy() {
            reason = "bridge_disarmed";
            break;
        }
        let integral = r.imu.forward_integral.at(now);
        let yaw = r.imu.yaw.at(now);
        if integral.zip(yaw).is_none() && now - since > IMU_STALE_S && phase != Phase::Rest2 {
            guard = Some("imu_stale");
            spans.push((phase.name(), since, now));
            (phase, since) = (Phase::Rest2, now);
        }
        if let (Some((t0, i0)), Some(b), Some(i)) = (zero, bias, integral) {
            speed = (i - i0 - b * (now - t0)).max(0.);
            distance += speed * dt;
            heading += yaw.unwrap_or(0.) * dt;
            peak_heading = peak_heading.max(heading.abs());
            peak_speed = peak_speed.max(speed);
        }
        let ranges = if r.o.demo { None } else { r.fresh_ranges(now).map(<[V]>::to_vec) };
        let front = ranges.as_deref().and_then(front_range).map(|f| f - FRONT_BODY_EXTENT_M);
        let inside = ranges.as_deref().is_some_and(|ranges| !body_clear(ranges));
        let elapsed = now - since;
        let mut change = None;
        match phase {
            Phase::Rest1 => {
                if elapsed >= REST_SKIP_S && rest_start.is_none() {
                    rest_start = integral.map(|i| (now, i));
                }
                if elapsed >= r.o.rest_s {
                    if let (Some((t0, i0)), Some(i)) = (rest_start, integral) {
                        bias = Some((i - i0) / (now - t0));
                        zero = Some((now, i));
                        change = Some(Phase::Push);
                    } else {
                        reason = "no_rest_bias";
                        break;
                    }
                }
            }
            Phase::Push => {
                if elapsed >= r.o.push_s || speed >= MAX_SPEED_MPS || out_of_length(distance, speed, r.o.length_m) {
                    change = Some(Phase::Coast);
                }
            }
            Phase::Coast => {
                stopped_since = if speed < STOPPED_MPS { stopped_since.or(Some(now)) } else { None };
                if elapsed >= COAST_MAX_S || (elapsed >= CUT_LAG_S && stopped_since.is_some_and(|at| now - at >= STOPPED_HOLD_S)) {
                    change = Some(Phase::Rest2);
                }
            }
            Phase::Brake => {
                if elapsed >= BRAKE_MAX_S || speed < STOPPED_MPS {
                    change = Some(Phase::Rest2);
                }
            }
            Phase::Rest2 => {
                if elapsed >= REST_AFTER_S {
                    spans.push((phase.name(), since, now));
                    break;
                }
            }
        }
        if matches!(phase, Phase::Push | Phase::Coast) {
            let cut = front.is_some_and(|gap| gap <= speed * (CUT_LAG_S + 0.05) + speed * speed / (2. * COAST_DECEL_MPS2) + 0.2);
            let brake = (front.is_some_and(|gap| gap <= speed * 0.15 + speed * speed / (2. * BRAKE_DECEL_MPS2) + 0.12) || inside) && speed > STOPPED_MPS;
            if brake {
                guard = Some(if inside { "body_proximity" } else { "front_brake" });
                change = Some(Phase::Brake);
            } else if cut && phase == Phase::Push {
                guard = Some("front_cut");
                change = Some(Phase::Coast);
            }
        }
        if let Some(next_phase) = change {
            spans.push((phase.name(), since, now));
            (phase, since) = (next_phase, now);
        }
        let motor = match phase {
            Phase::Push => r.o.motor,
            Phase::Brake => LANE_BRAKE_MOTOR,
            _ => NEUTRAL,
        };
        let servo = if matches!(phase, Phase::Push | Phase::Coast | Phase::Brake) { steer(heading) } else { NEUTRAL };
        if !r.send("drive", motor, servo) {
            reason = "control_write_failed";
            break;
        }
        if r.o.demo {
            r.step_sim(now, motor, servo);
        }
        let tick = dict! {"t" => now, "phase" => phase.name(), "motor" => motor, "servo" => servo,
                          "ack_motor" => r.control.get("motor").clone(), "speed" => speed, "distance" => distance,
                          "heading" => heading, "front_gap" => front};
        let mut text = to_json(&tick);
        text.push('\n');
        let _ = r.files.send(FileOp::Append("ticks.jsonl", text));
    }
    dict! {
        "reason" => reason, "moved" => true, "guard" => guard, "motor" => r.o.motor, "length_m" => r.o.length_m,
        "push_s" => r.o.push_s, "rest_s" => r.o.rest_s, "bias_mps2" => bias, "peak_speed_mps" => peak_speed,
        "distance_m" => distance, "peak_heading_rad" => peak_heading,
        "phases" => V::list(spans.iter().map(|(name, a, b)| V::list(vec![(*name).into(), (*a).into(), (*b).into()])).collect()),
        "clock" => "program seconds; imu.jsonl captured_at is this clock in ms",
    }
}
