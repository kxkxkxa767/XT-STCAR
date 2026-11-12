//! Rust ROS2 hardware nodes. Decoder, calibration, revolution assembly and
//! Guard are the same implementations used by vehicle-bridge. A parent EOF,
//! signal or expired Guard lease still stops the independently running driver.
#[path = "../../crates/runner/src/bin/support/lidar_calibration.rs"]
mod lidar_calibration;
#[path = "../../crates/runner/src/bin/support/lidar_revolution.rs"]
mod lidar_revolution;
use rclrs::*;
use ros_env::std_msgs::msg::String as RosString;
use std::{
    io::{self, Read}, path::PathBuf,
    sync::{Arc, atomic::{AtomicBool, Ordering}, mpsc},
    thread, time::{Duration, Instant},
};
use xt_stcar_device_io::SerialPort;
use xt_stcar_robot_core::{
    FrameId, LidarSample, Timestamp,
    junction::{detect_corridor_candidates, detect_front_boundary, detect_left_junction, detect_turn_geometry},
    protocol::{chassis::PwmCommand, n10::{N10Config, N10Decoder}},
    teleop::{Guard, Request},
};
use crate::{race::spawn_named, race_ros2::{COMMAND, CONTROL, SCAN}};

struct Options {
    control: bool, execute: bool, lifeline: bool, reverse: bool,
    device: Option<PathBuf>, calibration: Option<PathBuf>,
    ros_args: Vec<String>,
}

fn parse(args: &[String]) -> Result<Options, String> {
    let (plain, ros_args) = crate::race::split_ros_args(args);
    let args = &plain;
    let control = match args.first().map(String::as_str) {
        Some("control") => true, Some("lidar") => false,
        _ => return Err("ros2-hardware requires control or lidar".into()),
    };
    let mut out = Options { control, execute: false, lifeline: false, reverse: false, device: None, calibration: None, ros_args };
    let mut args = args[1..].iter();
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--execute" => out.execute = true,
            "--parent-lifeline" => out.lifeline = true,
            "--allow-reverse" => out.reverse = true,
            "--device" => out.device = Some(args.next().ok_or("missing device")?.into()),
            "--lidar-calibration" => out.calibration = Some(args.next().ok_or("missing calibration")?.into()),
            _ => return Err(format!("unknown hardware option {arg}")),
        }
    }
    if out.execute && out.device.is_none() { return Err("--execute requires --device".into()); }
    if out.control && out.calibration.is_some() { return Err("calibration only applies to lidar".into()); }
    Ok(out)
}

fn calibrated_scan(row: &mut serde_json::Value, calibration: &lidar_calibration::Calibration, at: u64) -> Result<(), Box<dyn std::error::Error>> {
    calibration.apply(row)?;
    let frame = FrameId("lidar_origin_coarse_body_heading".into());
    let sample = LidarSample { captured_at: Timestamp(at), frame_id: frame.clone(), angle_min_rad: 0.,
        angle_increment_rad: -std::f64::consts::TAU / 360., range_min_m: 0.02, range_max_m: 12.,
        ranges_m: serde_json::from_value(row["ranges"].clone())? };
    row["front_boundary_m"] = serde_json::to_value(detect_front_boundary(&sample, &frame).ok().flatten())?;
    row["left_junction"] = serde_json::to_value(detect_left_junction(&sample, &frame).ok().flatten())?;
    row["corridor_candidates"] = serde_json::to_value(detect_corridor_candidates(&sample, &frame).unwrap_or_default())?;
    let geometry = detect_turn_geometry(&sample, &frame).unwrap_or_default();
    row["wall_candidates"] = serde_json::to_value(geometry.walls)?;
    row["left_turn_goal"] = serde_json::to_value(geometry.left_goal)?;
    Ok(())
}

fn lidar(mut port: Option<SerialPort>, calibration: Option<lidar_calibration::Calibration>, publisher: &Publisher<RosString>, stopped: &AtomicBool) -> Result<(), Box<dyn std::error::Error>> {
    let start = Instant::now();
    let mut decoder = N10Decoder::new(N10Config { frame_id: FrameId("laser_link".into()),
        range_min_m: 0.02, range_max_m: 12., max_packet_age_ms: 50 })?;
    let mut revolution = lidar_revolution::Revolution::new();
    let mut seq = 0u64;
    while !stopped.load(Ordering::Acquire) {
        if let Some(p) = port.as_mut() {
            let mut buf = [0; 4096];
            if let Some(n) = p.read_until(&mut buf, Instant::now() + Duration::from_millis(20))? {
                for packet in decoder.feed(&buf[..n], Timestamp(start.elapsed().as_millis() as u64))?.packets {
                    let ranges = packet.points.map(|p| p.range_m);
                    if let Some(done) = revolution.push(packet.start_angle_cdeg, packet.end_angle_cdeg, &ranges) {
                        seq += 1;
                        let at = start.elapsed().as_millis() as u64;
                        let mut row = serde_json::json!({"seq":seq,"at_ms":at,"ranges":done.ranges,
                            "valid_fraction":done.valid as f64/360.,"coverage":done.coverage as f64/360.,
                            "seam_filled_bins":done.seam_filled,"navigation_validated":false});
                        if let Some(calibration) = &calibration { calibrated_scan(&mut row, calibration, at)?; }
                        if publisher.publish(RosString { data: row.to_string() }).is_err() { return Ok(()); }
                    }
                }
            }
        } else {
            seq += 1;
            let ranges: Vec<Option<f64>> = (0..360).map(|i| Some(3. + (i as f64/30.).sin()*0.3)).collect();
            let mut row = serde_json::json!({"seq":seq,"ranges":ranges,"valid_fraction":1.,"coverage":1.,"navigation_validated":false});
            if let Some(calibration) = &calibration { calibration.apply(&mut row)?; }
            let _ = publisher.publish(RosString { data: row.to_string() });
            thread::sleep(Duration::from_millis(100));
        }
    }
    Ok(())
}

fn control(mut port: Option<SerialPort>, execute: bool, reverse: bool, requests: mpsc::Receiver<Request>, publisher: &Publisher<RosString>, stopped: &AtomicBool) -> Result<(), Box<dyn std::error::Error>> {
    let start = Instant::now();
    let mut guard = Guard::new(reverse);
    let mut problem = None;
    loop {
        let now = start.elapsed().as_millis() as u64;
        guard.advance(now);
        if stopped.load(Ordering::Acquire) { guard.stop("input_closed"); break; }
        for _ in 0..16 {
            if let Ok(request) = requests.try_recv() { guard.apply(request, now); } else { break; }
        }
        if let Some(p) = port.as_mut()
            && let Err(error) = p.write_packet_until(&PwmCommand::new(guard.status.motor, guard.status.servo)?.encode(), Instant::now()+Duration::from_millis(15)) {
            problem = Some(error); break;
        }
        let _ = publisher.publish(RosString {
            data: serde_json::json!({"control":guard.status,"physical_output":execute,"reverse_enabled":reverse}).to_string(),
        });
        thread::sleep(Duration::from_millis(20));
    }
    if let Some(p) = port.as_mut() {
        for _ in 0..20 {
            let _ = p.write_packet_until(&PwmCommand::new(1500,1500)?.encode(), Instant::now()+Duration::from_millis(15));
            thread::sleep(Duration::from_millis(20));
        }
    }
    if let Some(error) = problem { return Err(error.into()); }
    Ok(())
}

fn run(o: Options) -> Result<(), Box<dyn std::error::Error>> {
    let calibration = o.calibration.as_ref().map(|path| -> Result<_, Box<dyn std::error::Error>> {
        let mut bytes = Vec::new();
        std::fs::File::open(path)?.take(4097).read_to_end(&mut bytes)?;
        if bytes.len() > 4096 { return Err("lidar calibration exceeds4096 bytes".into()); }
        let calibration: lidar_calibration::Calibration = serde_json::from_slice(&bytes)?;
        calibration.validate()?; Ok(calibration)
    }).transpose()?;
    let context = Context::new(std::iter::once("route-engine".to_owned()).chain(o.ros_args.iter().cloned()), InitOptions::default())?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node(if o.control { "xt_stcar_chassis" } else { "xt_stcar_lidar" })?;
    let worker = node.create_worker(());
    let publisher = node.create_publisher::<RosString>((if o.control { CONTROL } else { SCAN }).keep_last(1).best_effort().volatile())?;
    let stopped = Arc::new(AtomicBool::new(false));
    let signal = stopped.clone();
    ctrlc::set_handler(move || signal.store(true, Ordering::Release))?;
    if o.lifeline {
        let signal = stopped.clone();
        spawn_named("parent-lifeline", move || {
            let _ = io::stdin().lock().read(&mut [0u8; 1]);
            signal.store(true, Ordering::Release);
        });
    }
    let (send, requests) = mpsc::sync_channel(16);
    let signal = stopped.clone();
    let _commands = if o.control {
        Some(worker.create_subscription::<RosString, _>(COMMAND.keep_last(4).reliable().volatile(), move |_: &mut (), msg: RosString| {
            if msg.data.len() > 1024 { signal.store(true, Ordering::Release); return; }
            match serde_json::from_str::<Request>(&msg.data) {
                Ok(request) => { let _ = send.try_send(request); },
                Err(_) => signal.store(true, Ordering::Release),
            }
        })?)
    } else { None };
    // Complete ROS setup before opening any physical serial device.
    let port = if o.execute { Some(SerialPort::open(o.device.as_ref().unwrap(), if o.control { 38400 } else { 230400 })?) } else { None };
    let commands = executor.commands().clone();
    let signal = stopped.clone();
    let runner = spawn_named("ros2-executor", move || {
        let errors = executor.spin(SpinOptions::default());
        if !errors.is_empty() { eprintln!("hardware executor: {errors:?}"); }
        signal.store(true, Ordering::Release);
    });
    let result = if o.control { control(port, o.execute, o.reverse, requests, &publisher, &stopped) }
        else { lidar(port, calibration, &publisher, &stopped) };
    commands.halt_spinning();
    let _ = runner.join();
    result
}

pub(crate) fn main(args: &[String]) -> i32 {
    match parse(args).map_err(|e| e.into()).and_then(run) {
        Ok(()) => 0,
        Err(error) => { eprintln!("ROS2 hardware: {error}"); 69 }
    }
}
