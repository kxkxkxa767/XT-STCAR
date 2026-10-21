//! The standalone control program (round 97, part B): `route-engine race ...`.
//!
//! One control thread owns the Supervisor (supervisor.rs: the console's route
//! supervision, line for line) and runs the console's loop with no Python and
//! no web server: bridge lines are applied as they arrive (accept_lidar,
//! accept_control and its lock halt), a tick every 20 ms with the console's
//! pacing (at least 5 ms idle after a long tick) and timing records, the
//! health watch every 50 ms, the owner lease renewed by the program itself.
//! Reader threads only forward bridge lines; a writer thread writes the
//! evidence files (the trial helper's layout, so trial-timeline, trial-map and
//! route-replay read them unchanged). The bridges run in their own process
//! group: interrupting this program (Ctrl-C, a closed terminal) closes their
//! input and they hold neutral themselves; their own 300 ms command lease
//! also covers a stalled program.
//!
//! Start: the console's start check (turn_preview) is shown until it passes;
//! then `--start enter` waits for Enter on the terminal, `--start ready`
//! starts once it has passed continuously for `--ready-hold-s`; `--start
//! never` only shows and records the check for `--max-wait-s` and never arms
//! (static tests on the car).

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
const LINE_LIMIT: usize = 65536;

struct Options {
    bridge: String,
    out: PathBuf,
    max_drive_s: f64,
    initial_presteer_pwm: i64,
    start: String,
    ready_hold_s: f64,
    demo: bool,
    control_device: String,
    lidar_device: String,
    max_wait_s: f64,
    status_s: f64,
}

const USAGE: &str = "usage: route-engine race --bridge PATH --out NEW_DIR [--start enter|ready|never] [--ready-hold-s 3] \
[--max-drive-s 40] [--initial-presteer-pwm 1670] [--max-wait-s 300] [--status-s .5] [--demo] \
[--control-device /dev/car] [--lidar-device /dev/laser]";

fn parse(args: &[String]) -> Result<Options, String> {
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
        max_wait_s: 300.,
        status_s: 0.5,
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
            "--status-s" => o.status_s = number(i)?,
            "--control-device" => o.control_device = value(i)?,
            "--lidar-device" => o.lidar_device = value(i)?,
            "--demo" => {
                o.demo = true;
                i += 1;
                continue;
            }
            other => return Err(format!("unknown option {other}")),
        }
        i += 2;
    }
    if o.bridge.is_empty() || o.out.as_os_str().is_empty() {
        return Err("--bridge and --out are required".into());
    }
    if !["enter", "ready", "never"].contains(&o.start.as_str()) {
        return Err("--start must be enter, ready or never".into());
    }
    // autonomy-control.py's route drive limit (1..ROUTE_MAX_DRIVE_S).
    if !(1. ..=ROUTE_MAX_DRIVE_S).contains(&o.max_drive_s) {
        return Err(format!("drive limit must be 1..{ROUTE_MAX_DRIVE_S} seconds"));
    }
    Ok(o)
}

enum Event {
    Line(&'static str, String, Instant),
    Coast(u64, String),
    Ended(&'static str, String),
    Enter,
    StdinClosed,
}

/// The console's spawn(mode, device): device identity and ownership checks,
/// then the bridge in its own process group.
fn spawn(o: &Options, mode: &'static str, device: &str, expected: &str) -> Result<Child, String> {
    let mut command = Command::new(&o.bridge);
    command.arg(mode);
    if !o.demo {
        let real = std::fs::canonicalize(device).map_err(|e| format!("device {device}: {e}"))?;
        if real != Path::new(expected) {
            return Err(format!("device identity changed: {device}"));
        }
        let check = Command::new("fuser").arg(device).stdout(Stdio::null()).stderr(Stdio::null()).status();
        if !matches!(check.map(|s| s.code()), Ok(Some(1))) {
            return Err(format!("device busy or owner check failed: {device} (stop the console service first)"));
        }
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
    thread::spawn(move || {
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

enum FileOp {
    Put(&'static str, String),
    Append(&'static str, String),
    Stop,
}

fn writer(dir: PathBuf) -> (Sender<FileOp>, thread::JoinHandle<()>) {
    let (tx, rx) = mpsc::channel::<FileOp>();
    let handle = thread::spawn(move || {
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
    rx: Receiver<Event>,
    files: Sender<FileOp>,
    failed: Vec<&'static str>,
    entered: bool,
    stdin_closed: bool,
    max_drive_v: V,
    initial_v: V,
    coast: Sender<(u64, String)>,
}

impl Program {
    fn mono(&self) -> f64 {
        self.base.elapsed().as_secs_f64()
    }

    fn at(&self, instant: Instant) -> f64 {
        instant.saturating_duration_since(self.base).as_secs_f64()
    }

    /// One bridge line or event, as read_bridge applies it under the console lock.
    fn handle(&mut self, event: Event) {
        match event {
            Event::Line(kind, line, received) => {
                if self.failed.contains(&kind) {
                    return;
                }
                let now = self.at(received);
                self.sup.mono = self.mono();
                let applied = py_try(|| {
                    let value = parse_json(line.trim_end()).unwrap_or_else(|e| value_error(e));
                    if kind == "control" {
                        self.sup.accept_control(&value, now);
                    } else {
                        self.sup.accept_lidar(value, now);
                    }
                });
                if let Err(error) = applied {
                    self.failed.push(kind);
                    self.sup.bridge_failed(kind, &error.msg);
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
            Event::Enter => self.entered = true,
            Event::StdinClosed => self.stdin_closed = true,
        }
    }

    /// Hand the coast worker's pending scan to the fitting thread.
    fn dispatch(&mut self) {
        if let Some(job) = self.sup.coast_worker.take_job() {
            let _ = self.coast.send((job.number, job.request));
        }
    }

    /// Wait up to `until` (program clock), applying events as they come.
    fn pump(&mut self, until: f64) {
        loop {
            let left = until - self.mono();
            if left <= 0. {
                break;
            }
            match self.rx.recv_timeout(Duration::from_secs_f64(left)) {
                Ok(event) => self.handle(event),
                Err(RecvTimeoutError::Timeout) => break,
                Err(RecvTimeoutError::Disconnected) => break,
            }
        }
        while let Ok(event) = self.rx.try_recv() {
            self.handle(event);
        }
    }

    fn state(&mut self) -> V {
        let now = self.mono();
        let (max_drive, initial) = (self.max_drive_v.clone(), self.initial_v.clone());
        self.sup.state_view(now, &max_drive, &initial)
    }

    fn lidar(&self) -> V {
        dict! {"scan" => self.sup.scan.clone(), "age_s" => self.mono() - self.sup.scan_at.f()}
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
    let mut control = match spawn(&o, "control", &o.control_device, "/dev/ttyUSB1") {
        Ok(c) => c,
        Err(e) => {
            eprintln!("{e}");
            return 69;
        }
    };
    let mut lidar = match spawn(&o, "lidar", &o.lidar_device, "/dev/ttyACM0") {
        Ok(c) => c,
        Err(e) => {
            eprintln!("{e}");
            drop(control.stdin.take());
            let _ = control.wait();
            return 69;
        }
    };
    reader("control", control.stdout.take().unwrap(), tx.clone());
    reader("lidar", lidar.stdout.take().unwrap(), tx.clone());
    if o.start == "enter" {
        let tx = tx.clone();
        thread::spawn(move || {
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
        thread::spawn(move || {
            let mut fitter = crate::coast::CoastFitter::default();
            for (number, request) in jobs {
                if tx.send(Event::Coast(number, fitter.fit(&request))).is_err() {
                    break;
                }
            }
        });
    }
    drop(tx);
    let (files, writer_thread) = writer(o.out.clone());
    let mut bridge_in = control.stdin.take().unwrap();
    let mut sup = Supervisor::new(o.demo, &token(), "rust_program");
    sup.unix_time = unix_s;
    sup.camera = false;
    sup.writer = Some(Box::new(move |emit| {
        let mut text = to_json(&emit.row());
        text.push('\n');
        bridge_in.write_all(text.as_bytes()).and_then(|_| bridge_in.flush()).map_err(|e| e.to_string())
    }));
    let max_drive_v = V::Float(o.max_drive_s);
    let initial_v = V::Int(o.initial_presteer_pwm);
    let mut p = Program {
        o,
        sup,
        base: Instant::now(),
        rx,
        files,
        failed: Vec::new(),
        entered: false,
        stdin_closed: false,
        max_drive_v,
        initial_v,
        coast,
    };
    let code = run(&mut p);
    let _ = p.files.send(FileOp::Stop);
    let _ = writer_thread.join();
    // Closing the bridges' input: each stops at neutral and exits.
    p.sup.writer = None;
    for child in [&mut control, &mut lidar] {
        drop(child.stdin.take());
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
    code
}

fn run(p: &mut Program) -> i32 {
    // First feedback from both bridges.
    let deadline = p.mono() + 5.;
    while (p.sup.scan.is_none() || !p.sup.status.has("control")) && p.mono() < deadline {
        let until = p.mono() + 0.05;
        p.pump(until);
    }
    if p.sup.scan.is_none() || !p.sup.status.has("control") {
        p.say("no lidar scan or control feedback within 5 s");
        return 69;
    }
    p.say(&format!("evidence {}", p.o.out.display()));
    // Start check, shown until it passes; then the trigger.
    let wait_until = p.mono() + p.o.max_wait_s;
    let mut ready_since: Option<f64> = None;
    let mut next_status = 0.;
    let mut prompted = false;
    let before = loop {
        let until = p.mono() + 0.1;
        p.pump(until);
        let now = p.mono();
        p.sup.health_watch(now);
        let state = p.state();
        let autonomy = state.idx("autonomy");
        let ready = autonomy.idx("turn_ready").is_true();
        ready_since = if ready { ready_since.or(Some(now)) } else { None };
        if now >= next_status {
            let clearance = autonomy.get("turn_clearance");
            let front = clearance.get("front_m");
            p.say(&format!(
                "start check: {} front={}",
                if ready { "ready".to_string() } else { format!("not ready ({})", to_json(autonomy.get("turn_rejection"))) },
                to_json(front)
            ));
            next_status = now + p.o.status_s.max(0.1);
        }
        if p.o.start == "enter" {
            if ready && !prompted {
                p.say("READY: press Enter to start the route (Ctrl-C quits; the car stays locked)");
                prompted = true;
            }
            if p.entered {
                p.entered = false;
                if ready {
                    break state;
                }
                p.say("not ready; Enter again once the start check passes");
            }
            if p.stdin_closed && !p.entered {
                p.say("terminal input closed; use --start ready without a terminal");
                return 64;
            }
        } else if p.o.start == "ready" && ready_since.is_some_and(|since| now - since >= p.o.ready_hold_s) {
            break state;
        }
        if p.o.start == "never" && now >= wait_until {
            // Check only: record what the check saw; nothing was armed.
            let _ = p.files.send(FileOp::Put("before.json", to_json(&state)));
            let _ = p.files.send(FileOp::Put("before-lidar.json", to_json(&p.lidar())));
            let result = dict! {"started" => false, "check_only" => true, "turn_ready" => ready,
                                "turn_rejection" => state.idx("autonomy").get("turn_rejection").clone(),
                                "control" => p.sup.control(), "errors" => p.sup.errors.clone()};
            let _ = p.files.send(FileOp::Put("cli-result.json", to_json(&result)));
            p.say("check only: not started");
            return 0;
        }
        if now >= wait_until {
            p.say("start check did not pass in time");
            return 75;
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
            let result = dict! {"started" => false, "error" => error.msg, "clearance" => clearance};
            let _ = p.files.send(FileOp::Put("cli-result.json", to_json(&result)));
            return 2;
        }
    };
    p.say(&format!("started run {run_id}"));
    let run_v = V::from(run_id.as_str());
    let t0 = now;
    // The console's autonomy_watch with its pacing and timing records.
    let mut wait = TICK_S;
    let mut previous: Option<(f64, f64, f64, Option<f64>)> = None;
    let mut next_tick = p.mono() + wait;
    let (mut next_health, mut next_evidence, mut next_status) = (p.mono(), p.mono(), p.mono());
    while p.sup.session.is_some() {
        let until = next_tick.min(next_health);
        p.pump(until);
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
        if let Err(error) = py_try(|| p.sup.autonomy_tick(&V::Float(tick_at))) {
            p.sup.halt(&format!("tick_internal_error_{}: {}", error.kind.name(), error.msg));
        }
        let ended = p.mono();
        let cpu = cpu0.and_then(|c0| thread_cpu_s().map(|c1| c1 - c0));
        let breakdown = previous.map(|(start0, end0, idle0, cpu0)| {
            dict! {"gap_s" => tick_at - start0, "previous_tick_wall_s" => end0 - start0,
                   "previous_tick_cpu_s" => opt_f(cpu0), "planned_idle_s" => idle0,
                   "wake_late_s" => requested - (end0 + idle0), "lock_wait_s" => tick_at - requested}
        });
        p.sup.record_tick_timing(tick_at - requested, ended - tick_at, cpu, breakdown, &run_v);
        wait = MIN_IDLE_S.max(TICK_S - (ended - requested));
        previous = Some((tick_at, ended, wait, cpu));
        next_tick = ended + wait;
        // The scan this tick submitted for the coast fit goes to its thread.
        p.dispatch();
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
    let result = p.sup.auto_result.clone();
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
    let summary = dict! {
        "run_id" => run_v.clone(), "started" => started, "reason" => result.get("reason").clone(),
        "route_controller" => "rust_program", "max_tick_compute_s" => result.get("max_tick_compute_s").clone(),
        "max_actual_gap_s" => result.get("max_actual_gap_s").clone(), "max_gap_breakdown" => result.get("max_gap_breakdown").clone(),
        "result" => result,
    };
    let _ = p.files.send(FileOp::Put("summary.json", to_json(&summary)));
    let _ = p.files.send(FileOp::Put("cli-result.json", to_json(&summary)));
    0
}
