//! One installed Rust program, model selected before a detached single round.
//! Ports start-vehicle.py/race-trial.py orchestration. Driving still uses `race`;
//! the existing camera inference process and controller Guard are unchanged.
use rustix::{
    fs::{FlockOperation, flock},
    process::{Pid, Signal, kill_process},
};
use serde_json::{Value, json};
use std::{
    error::Error,
    fs::{self, File, OpenOptions},
    io::{BufRead, BufReader, Read, Write},
    net::{SocketAddr, TcpStream},
    os::unix::fs::DirBuilderExt,
    path::{Path, PathBuf},
    process::{Child, Command, ExitStatus, Stdio},
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
        mpsc,
    },
    thread,
    time::{Duration, Instant},
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const CONSOLE: &str = "xt-stcar-console.service";
const UNIT: &str = "xt-stcar-race";

struct Options {
    config: PathBuf,
    profile: String,
    action: String,
    light_mode: Option<String>,
    check_only: bool,
}

fn parse(args: &[String]) -> Result<Options> {
    let mut o = Options {
        config: "/home/bianbu/xt-stcar-console/race-launch.json".into(),
        profile: "default".into(),
        action: "start".into(),
        light_mode: None,
        check_only: false,
    };
    let mut it = args.iter();
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--config" => o.config = it.next().ok_or("missing config")?.into(),
            "--profile" => o.profile = it.next().ok_or("missing profile")?.clone(),
            "--show" | "--status" | "--stop" | "--test-status" | "--normal" | "--test" => {
                o.action = arg[2..].into()
            }
            "--test-lights" => o.light_mode = Some(it.next().ok_or("missing light mode")?.clone()),
            "--check-only" => o.check_only = true,
            _ => return Err(format!("unknown launch option {arg}").into()),
        }
    }
    if !matches!(o.profile.as_str(), "default" | "dark" | "light" | "mix") {
        return Err("profile must be default, dark, light or mix".into());
    }
    Ok(o)
}

fn read_json(path: &Path) -> Result<Value> {
    Ok(serde_json::from_slice(&fs::read(path)?)?)
}
fn write_json(path: &Path, value: &Value) -> Result<()> {
    fs::write(path, serde_json::to_string_pretty(value)? + "\n")?;
    Ok(())
}
fn text<'a>(value: &'a Value, key: &str) -> Result<&'a str> {
    value[key]
        .as_str()
        .ok_or_else(|| format!("missing launch field {key}").into())
}
fn checked(command: &mut Command) -> Result<()> {
    let status = command.status()?;
    if !status.success() {
        return Err(format!("command failed: {status}").into());
    }
    Ok(())
}
fn field_path(profile: &Value) -> Result<&Path> {
    Ok(Path::new(text(profile, "field_config")?))
}

fn field_options(mut config: Value) -> Result<(Value, Vec<String>, Vec<String>)> {
    let mut command = Vec::new();
    if config["enabled"] != true {
        return Ok((json!({}), command, Vec::new()));
    }
    match config["crosswalk_mode"].as_str().unwrap_or("fixed") {
        "vision" => {}
        "fixed" => {
            for name in [
                "start_zone_m",
                "start_line_to_crosswalk_back_edge_m",
                "crosswalk_length_m",
                "start_straight_m",
            ] {
                if !config[name]
                    .as_f64()
                    .is_some_and(|v| v.is_finite() && v > 0.)
                {
                    return Err(format!("invalid field dimension {name}").into());
                }
            }
            let edge = config["start_zone_m"].as_f64().unwrap()
                + config["start_line_to_crosswalk_back_edge_m"]
                    .as_f64()
                    .unwrap();
            config["crosswalk_back_edge_from_rear_wall_m"] = json!(edge);
            config["crosswalk_far_edge_from_rear_wall_m"] =
                json!(edge + config["crosswalk_length_m"].as_f64().unwrap());
            command.extend(["--test-crosswalk-from-rear-wall-m".into(), edge.to_string()]);
        }
        _ => return Err("crosswalk_mode must be vision or fixed".into()),
    }
    match config["light_mode"].as_str().unwrap_or("fixed10s") {
        "fixed10s" => command.push("--test-light-fixed-wait".into()),
        "vision" => {}
        _ => return Err("light_mode must be vision or fixed10s".into()),
    }
    let vision = [
        "--save-every",
        "1",
        "--max-saved",
        "1500",
        "--jpeg-quality",
        "95",
    ]
    .map(String::from)
    .to_vec();
    Ok((config, command, vision))
}

fn fresh_dir(base: &Path) -> Result<PathBuf> {
    for n in 1.. {
        let out = if n == 1 {
            base.to_path_buf()
        } else {
            base.with_file_name(format!(
                "{}-{n}",
                base.file_name()
                    .ok_or("missing evidence name")?
                    .to_string_lossy()
            ))
        };
        match fs::DirBuilder::new().mode(0o700).create(&out) {
            Ok(()) => return Ok(out),
            Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => {}
            Err(e) => return Err(e.into()),
        }
    }
    unreachable!()
}

fn state(access: &Value, query: &str) -> Result<Value> {
    let port = access["port"].as_u64().ok_or("missing console port")?;
    let addr: SocketAddr = format!("127.0.0.1:{port}").parse()?;
    let mut socket = TcpStream::connect_timeout(&addr, Duration::from_secs(2))?;
    socket.set_read_timeout(Some(Duration::from_secs(2)))?;
    socket.set_write_timeout(Some(Duration::from_secs(2)))?;
    write!(
        socket,
        "GET /api/state{query} HTTP/1.0\r\nHost: 127.0.0.1\r\nX-Control-Token: {}\r\nConnection: close\r\n\r\n",
        text(access, "token")?
    )?;
    let mut bytes = Vec::new();
    socket.read_to_end(&mut bytes)?;
    let split = bytes
        .windows(4)
        .position(|v| v == b"\r\n\r\n")
        .ok_or("invalid console HTTP response")?;
    let header = std::str::from_utf8(&bytes[..split])?;
    if header.split_whitespace().nth(1) != Some("200") {
        return Err("console state request failed".into());
    }
    Ok(serde_json::from_slice(&bytes[split + 4..])?)
}

fn neutral(s: &Value) -> bool {
    let c = &s["status"]["control"];
    s["healthy"] == true
        && s["owner"].is_null()
        && s["autonomy"]["active"].is_null()
        && s["autonomy"]["mode"] == "locked"
        && c["armed"] == false
        && c["motor"] == 1500
        && c["servo"] == 1500
        && s["ages"]["control"]
            .as_f64()
            .is_some_and(|v| (0.0..0.2).contains(&v))
}

fn busy(devices: &[&str]) -> Result<Vec<String>> {
    let mut result = Vec::new();
    for device in devices {
        if Command::new("fuser")
            .arg(device)
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status()?
            .code()
            != Some(1)
        {
            result.push((*device).into());
        }
    }
    Ok(result)
}

fn signal(child: &Child, sig: Signal) {
    if let Some(pid) = Pid::from_raw(child.id() as i32) {
        let _ = kill_process(pid, sig);
    }
}
fn finish(child: &mut Child, sig: Signal, timeout: Duration) -> Result<ExitStatus> {
    if let Some(status) = child.try_wait()? {
        return Ok(status);
    }
    signal(child, sig);
    let deadline = Instant::now() + timeout;
    loop {
        if let Some(status) = child.try_wait()? {
            return Ok(status);
        }
        if Instant::now() >= deadline {
            child.kill()?;
            return Ok(child.wait()?);
        }
        thread::sleep(Duration::from_millis(20));
    }
}
fn monotonic_s() -> f64 {
    let t = rustix::time::clock_gettime(rustix::time::ClockId::Monotonic);
    t.tv_sec as f64 + t.tv_nsec as f64 * 1e-9
}
fn ros_environment(command: &mut Command, out: &Path) {
    // Same Humble/Fast DDS environment as the existing setup.bash launch.
    let inherited = std::env::var("LD_LIBRARY_PATH").unwrap_or_default();
    command
        .env(
            "LD_LIBRARY_PATH",
            format!("/opt/ros/humble/lib/riscv64-linux-gnu:/opt/ros/humble/lib:{inherited}"),
        )
        .env("AMENT_PREFIX_PATH", "/opt/ros/humble")
        .env("ROS_DISTRO", "humble")
        .env("ROS_VERSION", "2")
        .env("ROS_DOMAIN_ID", "217")
        .env("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
        .env("ROS_LOG_DIR", out.join("ros-log"));
}

fn run_round(o: &Options, config: &Value, profile: &Value) -> Result<()> {
    let base = Path::new(text(config, "base")?);
    let lock = OpenOptions::new()
        .write(true)
        .create(true)
        .truncate(false)
        .open(base.join("race-trial.lock"))?;
    flock(&lock, FlockOperation::NonBlockingLockExclusive)?;
    let input = if field_path(profile)?.is_file() {
        read_json(field_path(profile)?)?
    } else {
        json!({})
    };
    let (field, field_command, field_vision) = field_options(input)?;
    let out = fresh_dir(&base.join(text(config, "evidence_prefix")?))?;
    write_json(
        &out.join("launch-profile.json"),
        &json!({"label":config["label"],"profile":o.profile,"selected":profile}),
    )?;
    if field["enabled"] == true {
        write_json(&out.join("field-test.json"), &field)?;
    }
    println!("{}", json!({"evidence_dir":out,"profile":o.profile}));
    let start = Instant::now();
    let access = read_json(&base.join("access.json"))?;
    let pre = state(
        &access,
        "?trial_mode=turn-cone&initial_presteer_pwm=1670&continue_route=true",
    )?;
    write_json(&out.join("before-console.json"), &pre)?;
    if !neutral(&pre) {
        return Err("console not at a fresh locked neutral; nothing started".into());
    }
    let stopped = Arc::new(AtomicBool::new(false));
    #[cfg(feature = "ros2")]
    {
        let flag = stopped.clone();
        ctrlc::set_handler(move || flag.store(true, Ordering::Release))?;
    }
    let mut result = json!({"evidence_dir":out,"check_only":o.check_only,"profile":o.profile,"label":config["label"],
        "launcher":"rust","field_test":field,"start_admission":"rust_race_live_sensors"});
    let mut timing = json!({"preflight_ms":start.elapsed().as_secs_f64()*1000.});
    let t = Instant::now();
    if let Err(error) = checked(Command::new("systemctl").args(["--user", "stop", CONSOLE])) {
        let _ = checked(Command::new("systemctl").args(["--user", "start", CONSOLE]));
        return Err(error);
    }
    timing["console_stop_ms"] = json!(t.elapsed().as_secs_f64() * 1000.);
    let mut vision: Option<Child> = None;
    let mut race: Option<Child> = None;
    let mut vision_rows = None;
    let mut vision_reader = None;
    let camera = profile["camera"].as_str().unwrap_or("/dev/video20");
    let devices = ["/dev/car", "/dev/laser", camera];
    let run_result = (|| -> Result<()> {
        let t = Instant::now();
        while !busy(&devices)?.is_empty()
            && t.elapsed() < Duration::from_secs(15)
            && !stopped.load(Ordering::Acquire)
        {
            thread::sleep(Duration::from_millis(200));
        }
        let occupied = busy(&devices)?;
        result["devices_busy_before_run"] = json!(occupied);
        if !occupied.is_empty() {
            return Err("devices still busy after stopping console".into());
        }
        timing["devices_wait_ms"] = json!(t.elapsed().as_secs_f64() * 1000.);
        let t = Instant::now();
        let mut child = Command::new("python3")
            .arg("-B")
            .arg(text(config, "vision_script")?)
            .arg("--model-dir")
            .arg(text(profile, "model_dir")?)
            .arg("--out")
            .arg(out.join("vision"))
            .args(["--camera", camera, "--max-s", "150"])
            .args(field_vision)
            .arg("--phase-state")
            .arg(out.join("run/vision-phase.txt"))
            .args(["--wait-threads", "4"])
            .arg("--light-state")
            .arg(out.join("light-state.json"))
            .stdin(Stdio::null())
            .stdout(Stdio::piped())
            .stderr(File::create(out.join("vision-stderr.txt"))?)
            .spawn()?;
        let stdout = child.stdout.take().ok_or("vision stdout missing")?;
        let (send, recv) = mpsc::channel();
        vision_reader = Some(thread::spawn(move || {
            for line in BufReader::new(stdout)
                .lines()
                .map_while(std::result::Result::ok)
            {
                if let Ok(row) = serde_json::from_str::<Value>(&line) {
                    let _ = send.send(row);
                }
            }
        }));
        vision = Some(child);
        let mut ready = Value::Null;
        while t.elapsed() < Duration::from_secs(25) && !stopped.load(Ordering::Acquire) {
            match recv.recv_timeout(Duration::from_millis(100)) {
                Ok(row) if row["kind"] == "ready" => {
                    ready = row;
                    break;
                }
                Ok(_) | Err(mpsc::RecvTimeoutError::Timeout) => {}
                Err(mpsc::RecvTimeoutError::Disconnected) => break,
            }
        }
        vision_rows = Some(recv);
        result["light_stop"] = json!(!ready.is_null());
        result["vision_ready"] = ready.clone();
        timing["vision_ready_ms"] = json!(t.elapsed().as_secs_f64() * 1000.);
        if stopped.load(Ordering::Acquire) {
            return Err("operator stopped during startup".into());
        }
        let mut command = Command::new(std::env::current_exe()?);
        command
            .arg("race")
            .arg("--bridge")
            .arg(text(config, "bridge")?)
            .arg("--out")
            .arg(out.join("run"))
            .args([
                "--max-drive-s",
                "40",
                "--initial-presteer-pwm",
                "1670",
                "--status-s",
                ".5",
                "--max-wait-s",
                "20",
            ]);
        if o.check_only {
            command.args(["--start", "never"]);
        } else {
            command.args(["--start", "ready", "--ready-hold-s", "1"]);
        }
        command.args(field_command);
        let surface = base.join("surface-initial.json");
        if surface.is_file() {
            let load = read_json(&surface)?;
            command.arg("--surface-initial-load").arg(
                load["extra_load_mps2"]
                    .as_f64()
                    .ok_or("invalid surface load")?
                    .to_string(),
            );
            result["surface_initial"] = load;
        }
        if !ready.is_null() {
            command
                .arg("--light-state")
                .arg(out.join("light-state.json"));
        }
        ros_environment(&mut command, &out);
        let stdout = File::create(out.join("race-stdout.txt"))?;
        command
            .stdin(Stdio::null())
            .stdout(stdout.try_clone()?)
            .stderr(stdout);
        let began = Instant::now();
        result["program_started_monotonic_s"] = json!(monotonic_s());
        timing["before_program_ms"] = json!(start.elapsed().as_secs_f64() * 1000.);
        race = Some(command.spawn()?);
        let child = race.as_mut().unwrap();
        let status = loop {
            if let Some(status) = child.try_wait()? {
                break status;
            }
            if stopped.load(Ordering::Acquire) || began.elapsed() >= Duration::from_secs(120) {
                break finish(child, Signal::INT, Duration::from_secs(5))?;
            }
            thread::sleep(Duration::from_millis(20));
        };
        result["program_exit"] = json!(status.code());
        result["program_elapsed_s"] = json!(began.elapsed().as_secs_f64());
        Ok(())
    })();
    if let Some(child) = race.as_mut() {
        let _ = finish(child, Signal::INT, Duration::from_secs(5));
    }
    if let Some(child) = vision.as_mut() {
        result["vision_exit"] = json!(
            finish(child, Signal::TERM, Duration::from_secs(10))
                .ok()
                .and_then(|s| s.code())
        );
    }
    if let Some(reader) = vision_reader {
        let _ = reader.join();
    }
    if let Some(rows) = vision_rows {
        for row in rows.try_iter() {
            if row["kind"] == "summary" {
                result["vision_summary"] = row;
            }
        }
    }
    let restore = checked(Command::new("systemctl").args(["--user", "start", CONSOLE]));
    let mut final_rows = Vec::<Value>::new();
    let deadline = Instant::now() + Duration::from_secs(90);
    while restore.is_ok() && final_rows.len() < 6 && Instant::now() < deadline {
        if let Ok(s) = state(&access, "") {
            let advanced = final_rows.last().is_none_or(|last| {
                s["status"]["control"]["tick"].as_u64() > last["status"]["control"]["tick"].as_u64()
            });
            if neutral(&s) && advanced {
                final_rows.push(s);
            } else if !neutral(&s) {
                final_rows.clear();
            }
        }
        thread::sleep(Duration::from_millis(300));
    }
    write_json(&out.join("after-console.json"), &json!(final_rows))?;
    let summary = read_json(&out.join("run/summary.json")).unwrap_or(json!({}));
    for key in ["reason", "max_actual_gap_s", "max_tick_compute_s"] {
        result[key] = summary[key].clone();
    }
    result["console_neutral_samples"] = json!(final_rows.len());
    result["console_healthy"] = final_rows
        .last()
        .map_or(Value::Null, |s| s["healthy"].clone());
    result["launch_timing_ms"] = timing;
    if let Err(error) = &run_result {
        result["error"] = json!(error.to_string());
    }
    write_json(&out.join("helper-result.json"), &result)?;
    println!("{result}");
    restore?;
    run_result?;
    if final_rows.len() != 6 {
        return Err("console fresh neutral lock not confirmed".into());
    }
    Ok(())
}

fn entry(o: Options, internal_run: bool) -> Result<()> {
    let config = read_json(&o.config)?;
    let profile = &config["profiles"][&o.profile];
    if profile.is_null() {
        return Err(format!("{} model is not installed", o.profile).into());
    }
    match o.action.as_str() {
        "show" => {
            println!("{} / {}", text(&config, "label")?, o.profile);
            return Ok(());
        }
        "stop" => {
            return checked(Command::new("systemctl").args([
                "--user",
                "stop",
                &format!("{UNIT}.service"),
            ]));
        }
        "status" => {
            let _ = Command::new("systemctl")
                .args(["--user", "status", &format!("{UNIT}.service"), "--no-pager"])
                .status()?;
            return Ok(());
        }
        _ => {}
    }
    if matches!(o.action.as_str(), "test" | "normal" | "test-status") || o.light_mode.is_some() {
        let path = field_path(profile)?;
        let mut field = if path.is_file() {
            read_json(path)?
        } else {
            json!({"enabled":false})
        };
        if o.action == "test" {
            field = if o.profile == "default" {
                json!({"enabled":true,"start_zone_m":0.8,"start_line_to_crosswalk_back_edge_m":2.0,"crosswalk_length_m":0.297,"start_straight_m":3.2,"light_mode":"fixed10s"})
            } else {
                json!({"enabled":true,"crosswalk_mode":"vision","light_mode":"fixed10s"})
            };
        }
        if o.action == "normal" {
            field["enabled"] = json!(false);
        }
        if let Some(mode) = o.light_mode {
            if field["enabled"] != true || !matches!(mode.as_str(), "fixed10s" | "vision") {
                return Err(
                    "--test-lights needs an enabled test profile and fixed10s or vision".into(),
                );
            }
            field["light_mode"] = json!(mode);
        }
        if o.action != "test-status" {
            let tmp = path.with_extension("tmp");
            write_json(&tmp, &field)?;
            fs::rename(tmp, path)?;
        }
        println!(
            "{}\n仅配置/查看，没有发车。",
            serde_json::to_string_pretty(&field)?
        );
        return Ok(());
    }
    if internal_run {
        return run_round(&o, &config, profile);
    }
    let mut command = Command::new("systemd-run");
    command
        .args([
            "--user",
            &format!("--unit={UNIT}"),
            "--collect",
            "--property=Type=exec",
            "--property=KillSignal=SIGINT",
        ])
        .arg(format!(
            "--property=WorkingDirectory={}",
            text(&config, "base")?
        ))
        .arg(std::env::current_exe()?)
        .arg("launch-round")
        .arg("--config")
        .arg(&o.config)
        .args(["--profile", &o.profile]);
    if o.check_only {
        command.arg("--check-only");
    }
    checked(&mut command)?;
    println!(
        "已提交 {} / {}；后台运行一轮，断网或退出SSH不影响运行。",
        text(&config, "label")?,
        o.profile
    );
    Ok(())
}

pub(crate) fn main(args: &[String], internal_run: bool) -> i32 {
    match parse(args).and_then(|o| entry(o, internal_run)) {
        Ok(()) => 0,
        Err(error) => {
            eprintln!("发车入口：{error}");
            1
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn visual_profiles_never_inherit_the_fixed_crosswalk_distance() {
        let (_, args, _) = field_options(
            json!({"enabled":true,"crosswalk_mode":"vision","light_mode":"fixed10s"}),
        )
        .unwrap();
        assert_eq!(args, ["--test-light-fixed-wait"]);
        let (_, args, _) =
            field_options(json!({"enabled":true,"crosswalk_mode":"vision","light_mode":"vision"}))
                .unwrap();
        assert!(args.is_empty());
    }
    #[test]
    fn existing_practice_dimensions_keep_the_same_arguments() {
        let (_, args, _) = field_options(json!({"enabled":true,"start_zone_m":0.8,"start_line_to_crosswalk_back_edge_m":2.,"crosswalk_length_m":0.297,"start_straight_m":3.2})).unwrap();
        assert_eq!(
            args,
            [
                "--test-crosswalk-from-rear-wall-m",
                "2.8",
                "--test-light-fixed-wait"
            ]
        );
    }
}
