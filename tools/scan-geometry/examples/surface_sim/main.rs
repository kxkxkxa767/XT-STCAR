#![allow(dead_code, unused_imports)]
//! Offline, deterministic closed-loop surface experiments. No vehicle connection.
//! cargo run --release --manifest-path tools/scan-geometry/Cargo.toml --example surface_sim -- CONFIG OUT
//! The real Rust Supervisor owns commands. Ray casting and a fitted empirical plant
//! supply sensor observations; geometry uses the vehicle bridge's native library.

#[path = "../../../../route-engine/src/clearance.rs"]
mod clearance;
#[path = "../../../../route-engine/src/coast.rs"]
mod coast;
#[path = "../../../../route-engine/src/compact_target.rs"]
mod compact_target;
#[path = "../../../../route-engine/src/dump.rs"]
mod dump;
#[path = "../../../../route-engine/src/helpers.rs"]
mod helpers;
#[path = "../../../../route-engine/src/maneuver.rs"]
mod maneuver;
#[path = "../../../../route-engine/src/maneuver_crosswalk.rs"]
mod maneuver_crosswalk;
#[path = "../../../../route-engine/src/maneuver_finish.rs"]
mod maneuver_finish;
#[path = "../../../../route-engine/src/maneuver_light.rs"]
mod maneuver_light;
#[path = "../../../../route-engine/src/maneuver_open.rs"]
mod maneuver_open;
#[path = "../../../../route-engine/src/maneuver_path.rs"]
mod maneuver_path;
#[path = "../../../../route-engine/src/maneuver_post.rs"]
mod maneuver_post;
#[path = "../../../../route-engine/src/maneuver_tm.rs"]
mod maneuver_tm;
#[path = "../../../../route-engine/src/maneuver_wait.rs"]
mod maneuver_wait;
#[path = "../../../../route-engine/src/motion_sensors.rs"]
mod motion_sensors;
mod plant;
#[path = "../../../../route-engine/src/py.rs"]
mod py;
#[path = "../../../../route-engine/src/race_crosswalk.rs"]
mod race_crosswalk;
#[path = "../../../../route-engine/src/race_crosswalk_stop.rs"]
mod race_crosswalk_stop;
#[path = "../../../../route-engine/src/race_surface.rs"]
mod race_surface;
#[path = "../../../../route-engine/src/race_surface_speed.rs"]
mod race_surface_speed;
#[path = "../../../../route-engine/src/race_light.rs"]
mod race_light;
#[path = "../../../../route-engine/src/race_light_brake.rs"]
mod race_light_brake;
#[path = "../../../../route-engine/src/race_transfer.rs"]
mod race_transfer;
#[path = "../../../../route-engine/src/supervisor.rs"]
mod supervisor;
mod world;

use plant::{Plant, Pose, number};
use py::V;
use serde_json::{Value, json};
use std::{
    collections::VecDeque,
    fs::{self, File},
    io::{BufWriter, Write},
    path::Path,
    time::Instant,
};
use xt_stcar_robot_core::{
    FrameId, LidarSample, Timestamp,
    junction::{
        detect_corridor_candidates, detect_front_boundary, detect_left_junction,
        detect_turn_geometry,
    },
};

struct Random(u64);
impl Random {
    fn uniform(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        ((self.0 >> 11) as f64 + 0.5) / ((1u64 << 53) as f64)
    }
    fn normal(&mut self) -> f64 {
        (-2. * self.uniform().ln()).sqrt() * (std::f64::consts::TAU * self.uniform()).cos()
    }
}

fn to_v(value: &Value) -> V {
    py::parse_json(&value.to_string()).unwrap()
}
fn json_v(value: &V) -> Value {
    match value {
        V::None => Value::Null,
        V::Bool(v) => json!(v),
        V::Int(v) => json!(v),
        V::Float(v) => serde_json::Number::from_f64(*v).map_or(Value::Null, Value::Number),
        V::Str(v) => Value::String(v.to_string()),
        V::List(items) => Value::Array(items.iter().map(json_v).collect()),
        V::Dict(items) => Value::Object(
            items
                .iter()
                .map(|(key, v)| (key.to_string(), json_v(v)))
                .collect(),
        ),
    }
}

fn scan(
    world: &world::World,
    plant: &Plant,
    seq: i64,
    received: f64,
    sensors: &Value,
    rng: &mut Random,
) -> V {
    let noise = number(sensors, "range_noise_m", 0.008);
    let distortion = number(sensors, "scan_distortion", 1.);
    let published = (received - number(sensors, "lidar_delivery_s", 0.012)).max(0.);
    // The N10 collects increasing raw angles. Vehicle calibration rotates raw
    // bins by +5 into body coordinates; the age seam rotates with those bins.
    let yaw_bins = number(sensors, "lidar_clockwise_yaw_bins", 5.).round() as i64;
    let ranges: Vec<Option<f64>> = (0..360)
        .map(|i| {
            let raw_bin = (i as i64 - yaw_bins).rem_euclid(360);
            let sampled = published - distortion * (359 - raw_bin) as f64 / 360. * 0.1;
            let pose = plant.pose_at(sampled);
            let value = world.cast(pose, pose.yaw - (i as f64).to_radians())?;
            let r = value + noise * rng.normal();
            (0.02..=12.)
                .contains(&r)
                .then_some((r * 1000.).round() / 1000.)
        })
        .collect();
    let at = (published * 1000.).round() as u64;
    let frame = FrameId("lidar_origin_coarse_body_heading".into());
    let sample = LidarSample {
        captured_at: Timestamp(at),
        frame_id: frame.clone(),
        angle_min_rad: 0.,
        angle_increment_rad: -std::f64::consts::TAU / 360.,
        range_min_m: 0.02,
        range_max_m: 12.,
        ranges_m: ranges.clone(),
    };
    let turn = detect_turn_geometry(&sample, &frame).unwrap_or_default();
    to_v(&json!({
        "seq":seq,"at_ms":at,"frame_id":frame.0,"ranges":ranges,
        "front_boundary_m":detect_front_boundary(&sample,&frame).ok().flatten(),
        "left_junction":detect_left_junction(&sample,&frame).ok().flatten(),
        "corridor_candidates":detect_corridor_candidates(&sample,&frame).unwrap_or_default(),
        "wall_candidates":turn.walls,"left_turn_goal":turn.left_goal
    }))
}

struct Camera {
    next: f64,
    frame: i64,
    pending: VecDeque<(f64, V)>,
    latest: V,
}
impl Camera {
    fn update(
        &mut self,
        t: f64,
        world: &world::World,
        car: &Plant,
        trigger: Option<f64>,
        config: &Value,
    ) {
        if t + 1e-9 >= self.next {
            self.next += 1. / number(config, "fps", 3.6);
            let age = number(config, "capture_age_s", 0.15);
            let sampled = t - age;
            let pose = car.pose_at(sampled);
            let (c, s) = (pose.yaw.cos(), pose.yaw.sin());
            let (dx, dy) = (world.gantry_x - pose.x, world.light_y - pose.y);
            let (ahead, side) = (c * dx + s * dy, -s * dx + c * dy);
            let visible =
                (0.60..=6.).contains(&ahead) && side.atan2(ahead).abs() <= 44_f64.to_radians();
            let red_start = trigger.map(|at| at + number(config, "red_on_delay_s", 0.8));
            let green_at = red_start.map(|at| at + number(config, "red_duration_s", 10.));
            let red = visible
                && red_start.is_some_and(|at| sampled >= at)
                && green_at.is_some_and(|at| sampled < at);
            let green = visible && green_at.is_some_and(|at| sampled >= at);
            let mut light = dict! {"frame"=>self.frame,"captured_at"=>t,"red"=>if red{0.90}else{0.},
            "green"=>if green{0.90}else{0.}};
            let stripe = (world.stripe_x - pose.x) / c - world::FRONT;
            if c > 0.866 && pose.y < world.lane_width + 0.15 && (0.20..=3.50).contains(&stripe) {
                light.set("crosswalk", 0.55);
                light.set("crosswalk_m", stripe);
                if let Some(board) = world.cast(pose, pose.yaw).filter(|d| *d > 0.2) {
                    let pitch = number(config, "camera_pitch_rad", 0.12);
                    let edge = (0.195 / (stripe + 0.041)).atan() + pitch;
                    let base = (0.195 / (board - 0.159)).atan() + pitch;
                    light.set("crosswalk_edge_y", edge.tan());
                    light.set("board_base_y", base.tan());
                }
            }
            self.pending
                .push_back((t + number(config, "latency_s", 0.24), light));
            self.frame += 1;
        }
        while self.pending.front().is_some_and(|x| x.0 <= t) {
            self.latest = self.pending.pop_front().unwrap().1;
        }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args().collect();
    if args.len() != 3 {
        return Err("usage: surface_sim CONFIG.json OUTPUT_DIRECTORY".into());
    }
    let config: Value = serde_json::from_str(&fs::read_to_string(&args[1])?)?;
    let out = Path::new(&args[2]);
    fs::create_dir(out)?;
    let world = world::World::new(&config["layout"]);
    let mut car = Plant::new(world.start, &config["plant"]);
    let mut rng = Random(
        config["seed"]
            .as_u64()
            .unwrap_or(1)
            .wrapping_add(0x9e3779b97f4a7c15),
    );
    let mut trace = BufWriter::new(File::create(out.join("trace.jsonl"))?);
    let mut events_file = BufWriter::new(File::create(out.join("decisions.jsonl"))?);
    let mut sup = supervisor::Supervisor::new(true, "surface-sim", "rust_engine");
    sup.camera = false;
    let mut camera = Camera {
        next: 0.,
        frame: 0,
        pending: VecDeque::new(),
        latest: V::None,
    };
    let mut ack: VecDeque<(f64, supervisor::Emit)> = VecDeque::new();
    let (mut armed, mut motor, mut servo, mut command_seq) = (false, 1500, 1500, 0);
    let mut seq = 0i64;
    let mut started = false;
    let mut start_error = None;
    let mut trigger = None;
    let mut move_at = None;
    let mut contact = None;
    let mut crosswalk_stop_event = None;
    let mut crosswalk_stop = None;
    let mut crosswalk_still_start = None;
    let mut crosswalk_still_end = None;
    let mut light_stop_event = None;
    let mut light_stop = None;
    let mut crossed_light_before_green = false;
    let mut finish_stop = None;
    let mut min_wall = f64::INFINITY;
    let mut min_cone = f64::INFINITY;
    let mut state = String::new();
    let mut stage_changes = Vec::new();
    let mut final_t = 0.;
    let wall_start = Instant::now();
    let max_t = number(&config, "max_seconds", 60.);
    let mut forward_after_brake = false;
    let mut brake_seen = false;
    for tick in 0..=(max_t / 0.02).ceil() as usize {
        let t = tick as f64 * 0.02;
        final_t = t;
        while ack.front().is_some_and(|x| x.0 <= t + 1e-9) {
            let (_, e) = ack.pop_front().unwrap();
            command_seq = e.seq;
            match e.op {
                "arm" => {
                    armed = true;
                    motor = 1500;
                    servo = 1500;
                }
                "stop" => {
                    armed = false;
                    motor = 1500;
                    servo = 1500;
                }
                "drive" => {
                    motor = e.motor.i();
                    servo = e.physical.i();
                }
                _ => {}
            }
        }
        if tick > 0 {
            car.step(
                t,
                0.02,
                if armed { motor } else { 1500 },
                if armed { servo } else { 1500 },
            );
        }
        let (front, rear) = world.front_rear_x(car.pose);
        if trigger.is_none() && world.intersects_beam(car.pose) {
            trigger = Some(t);
        }
        let green_at = trigger.map(|at| {
            at + number(&config["camera"], "red_on_delay_s", 0.8)
                + number(&config["camera"], "red_duration_s", 10.)
        });
        if car.pose.y >= world.width - world.light_width
            && front >= world.gantry_x
            && green_at.is_some_and(|green| t < green)
        {
            crossed_light_before_green = true;
        }
        camera.update(t, &world, &car, trigger, &config["camera"]);
        sup.mono = t;
        let status = dict! {"control"=>dict!{"tick"=>(t*1000.).round() as i64,"seq"=>command_seq,"armed"=>armed,"motor"=>motor,"servo"=>servo}};
        sup.accept_control(&status, t);
        if camera.latest.is_some() {
            sup.set_light(&camera.latest);
        }
        if tick % 5 == 0 {
            seq += 1;
            let value = scan(&world, &car, seq, t, &config["sensors"], &mut rng);
            sup.accept_lidar(value, t);
        }
        if !started && t >= 1.0 {
            let data = dict! {"op"=>"turn_cone_start","boot"=>"surface-sim","epoch"=>0,"tick"=>(t*1000.).round() as i64,
            "placement_confirmed"=>true,"continue_route"=>true,"max_drive_s"=>40.,"initial_presteer_pwm"=>1670};
            match sup.start_turn_cone(&data, &t.into(), "offline") {
                Ok(_) => started = true,
                Err((error, clearance)) => {
                    start_error = Some(format!("{}: {}", error.kind.name(), error.msg));
                    fs::write(out.join("start-error.json"), py::to_json(&clearance))?;
                    break;
                }
            }
        }
        if let Some(session) = sup.session.as_mut() {
            // Offline ablation only: remove the estimator's memory so the same
            // controller and physical surface can be compared without adaptation.
            if config["surface_adaptation"].as_bool() == Some(false) {
                session.motion.race_surface = Default::default();
            }
            session.motion.race_inputs = Some(motion_sensors::RaceInputs {
                yaw_rate: Some(car.yaw_rate),
                forward_integral: Some(
                    car.speed + number(&config["sensors"], "imu_drift_mps2", 0.) * t,
                ),
            });
            sup.renew_owner(t);
            if let Err(error) = py::py_try(|| sup.autonomy_tick(&t.into())) {
                sup.halt(&format!("simulation_controller_exception_{}", error.msg));
            }
            sup.coast_worker.run_pending();
        }
        for e in std::mem::take(&mut sup.emits) {
            writeln!(
                events_file,
                "{}",
                json!({"t":t,"op":e.op,"seq":e.seq,"motor":e.motor.i(),"nominal_servo":e.servo.i(),"physical_servo":e.physical.i()})
            )?;
            ack.push_back((t + 0.02, e));
        }
        sup.events.clear();
        let report = sup
            .session
            .as_ref()
            .map(|s| s.report.clone())
            .unwrap_or_else(|| sup.auto_result.clone());
        let turn = report.get("turn");
        let stage = turn.get("route_stage").os().unwrap_or("");
        let cross = turn.get("crosswalk_state").os().unwrap_or("");
        let lamp = turn.get("light_stop_state").os().unwrap_or("");
        let key = format!("{stage}/{cross}/{lamp}");
        if key != state {
            stage_changes.push(json!({"t":t,"stage":stage,"crosswalk":cross,"light":lamp}));
            state = key;
        }
        if move_at.is_none() && car.speed > 0.02 {
            move_at = Some(t);
        }
        if crosswalk_stop_event.is_none() && cross == "waiting" {
            crosswalk_stop_event =
                Some(json!({"t":t,"true_gap_m":world.stripe_x-front,"speed_mps":car.speed}));
        }
        if crosswalk_stop.is_none() && cross == "waiting" && car.speed < 0.05 {
            crosswalk_stop =
                Some(json!({"t":t,"true_gap_m":world.stripe_x-front,"speed_mps":car.speed}));
            crosswalk_still_start = Some(t);
        }
        if crosswalk_still_start.is_some() && crosswalk_still_end.is_none() && car.speed >= 0.05 {
            crosswalk_still_end = Some(t);
        }
        if light_stop_event.is_none() && lamp == "waiting" {
            light_stop_event = Some(
                json!({"t":t,"true_gap_m":world.gantry_x-front,"rear_past_beam_m":rear-world.beam_x,"speed_mps":car.speed}),
            );
        }
        if light_stop.is_none() && lamp == "waiting" && car.speed < 0.05 {
            light_stop = Some(
                json!({"t":t,"true_gap_m":world.gantry_x-front,"rear_past_beam_m":rear-world.beam_x,"speed_mps":car.speed}),
            );
        }
        if finish_stop.is_none() && turn.get("reason").os() == Some("route_finish_zone_stop") {
            finish_stop = Some(
                json!({"t":t,"true_gap_m":world.length-front,"speed_mps":car.speed,"rear_past_zone_m":rear-(world.length-0.8)}),
            );
        }
        if motor == 1300 {
            brake_seen = true;
        }
        if brake_seen && motor > 1500 {
            forward_after_brake = true;
        }
        let (wall, cone) = world.gaps(car.pose);
        min_wall = min_wall.min(wall);
        min_cone = min_cone.min(cone);
        if contact.is_none() && (wall <= 1e-6 || cone <= 1e-6) {
            contact = Some(
                json!({"t":t,"kind":if wall<=1e-6{"board"}else{"cone"},"pose":[car.pose.x,car.pose.y,car.pose.yaw]}),
            );
        }
        if tick % 5 == 0 {
            writeln!(
                trace,
                "{}",
                json!({"t":t,"seq":seq,"pose":[car.pose.x,car.pose.y,car.pose.yaw],"speed":car.speed,
                "motor":motor,"servo":servo,"wall_gap":wall,"cone_gap":cone,"stage":stage,"reason":turn.get("reason").os(),
                "crosswalk":json_v(turn.get("start_lane_feedback").get("crosswalk")),
                "surface_response":json_v(turn.get("surface_response")),
                "lane":json_v(turn.get("lane_feedback")),"open":json_v(turn.get("open_feedback")),
                "loop":json_v(turn.get("open_loop")),"quality":json_v(report.get("quality_issues"))})
            )?;
        }
        if contact.is_some() || (started && sup.session.is_none() && car.speed < 0.01) {
            break;
        }
    }
    trace.flush()?;
    events_file.flush()?;
    let report = sup
        .session
        .as_ref()
        .map(|s| s.report.clone())
        .unwrap_or_else(|| sup.auto_result.clone());
    let route_events = json_v(report.get("turn").get("route_events"));
    let event_time = |name: &str| route_events[name]["scan"]["received_at"].as_f64();
    let light_wait = event_time("light_go")
        .zip(event_time("light_stop"))
        .map(|(go, stop)| go - stop);
    let lap = event_time("finish_stop")
        .zip(move_at)
        .map(|(end, start)| end - start - light_wait.unwrap_or(0.));
    let crosswalk_still_s = crosswalk_still_start
        .map(|start| crosswalk_still_end.unwrap_or(final_t) - start)
        .unwrap_or(0.);
    let crosswalk_before_line = crosswalk_stop
        .as_ref()
        .and_then(|stop| stop["true_gap_m"].as_f64())
        .is_some_and(|gap| gap >= 0.);
    let light_stopped_in_zone = light_stop.as_ref().is_some_and(|stop| {
        stop["true_gap_m"].as_f64().is_some_and(|gap| gap >= 0.)
            && stop["rear_past_beam_m"]
                .as_f64()
                .is_some_and(|gap| gap >= 0.)
    });
    let (final_front, final_rear) = world.front_rear_x(car.pose);
    let finish_stopped_in_zone = car.speed < 0.05
        && car.pose.y >= world.width - world.light_width
        && final_front <= world.length
        && final_rear >= world.length - 0.8;
    let controller_finish = report.get("reason").os() == Some("route_finish_zone_stop");
    let modeled_task_pass = controller_finish
        && contact.is_none()
        && crosswalk_before_line
        && crosswalk_still_s + 1e-9 >= 3.
        && light_stopped_in_zone
        && !crossed_light_before_green
        && finish_stopped_in_zone;
    let summary = json!({
        "label":config["label"],"wall_seconds":wall_start.elapsed().as_secs_f64(),"sim_seconds":final_t,
        "surface_adaptation":config["surface_adaptation"].as_bool().unwrap_or(true),
        "start_error":start_error,"terminal_reason":report.get("reason").os(),"controller_finished":sup.session.is_none(),
        "contact":contact,"min_wall_gap_m":min_wall,"min_cone_gap_m":min_cone,
        "crosswalk_stop_event":crosswalk_stop_event,"crosswalk_stop":crosswalk_stop,
        "crosswalk_stationary_s":crosswalk_still_s,"crosswalk_stopped_before_line":crosswalk_before_line,
        "light_stop_event":light_stop_event,"light_stop":light_stop,"light_stopped_in_zone":light_stopped_in_zone,
        "light_crossed_before_green":crossed_light_before_green,"finish_stop":finish_stop,"route_events":route_events,
        "controller_finish":controller_finish,"physical_contact_free":contact.is_none(),
        "finish_stopped_in_zone":finish_stopped_in_zone,"modeled_task_pass":modeled_task_pass,
        "simulated_effective_s":lap,"light_wait_s":light_wait,"forward_after_brake":forward_after_brake,
        "max_traction_demand_ratio":car.max_traction_ratio,"traction_limited_s":car.slip_s,
        "stage_changes":stage_changes,"world":world.description(),"plant":car.model(),
        "sensors":{
            "lidar_delivery_s":number(&config["sensors"],"lidar_delivery_s",0.012),
            "lidar_clockwise_yaw_bins":number(&config["sensors"],"lidar_clockwise_yaw_bins",5.),
            "scan_distortion":number(&config["sensors"],"scan_distortion",1.),
            "camera_capture_age_s":number(&config["camera"],"capture_age_s",0.15),
            "camera_latency_s":number(&config["camera"],"latency_s",0.24)
        },
        "limitations":["Ideal semantic detector with specified latency and field of view; no outdoor image model inference.",
            "Empirical steering and speed response; friction-circle clipping approximates tire slip, without lateral tire dynamics.",
            "No cone displacement or impact dynamics. First true contact ends the simulation."],
        "result":json_v(&report)
    });
    fs::write(
        out.join("summary.json"),
        serde_json::to_string_pretty(&summary)?,
    )?;
    println!(
        "{}",
        json!({"out":out,"seconds":summary["simulated_effective_s"],"reason":summary["terminal_reason"],
                         "contact":summary["contact"],"start_error":summary["start_error"],"wall_seconds":summary["wall_seconds"]})
    );
    Ok(())
}
