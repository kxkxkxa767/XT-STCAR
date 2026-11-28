#![allow(dead_code, unused_imports)]
#![recursion_limit = "256"]
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
#[path = "../../../../route-engine/src/race_cam_features.rs"]
mod race_cam_features;
#[path = "../../../../route-engine/src/race_cam_loc.rs"]
mod race_cam_loc;
#[path = "../../../../route-engine/src/race_crosswalk.rs"]
mod race_crosswalk;
#[path = "../../../../route-engine/src/race_crosswalk_stop.rs"]
mod race_crosswalk_stop;
#[path = "../../../../route-engine/src/race_speed_kalman.rs"]
mod race_speed_kalman;
#[path = "../../../../route-engine/src/race_first_cone.rs"]
mod race_first_cone;
#[path = "../../../../route-engine/src/race_field_map.rs"]
mod race_field_map;
#[path = "../../../../route-engine/src/race_imu_fusion.rs"]
mod race_imu_fusion;
#[path = "../../../../route-engine/src/race_field_path.rs"]
mod race_field_path;
#[path = "../../../../route-engine/src/race_scan.rs"]
mod race_scan;
#[path = "../../../../route-engine/src/race_end_board.rs"]
mod race_end_board;
// The simulator supplies camera observations without starting a hardware worker.
use motion_sensors as race_crosswalk_vision;
#[path = "../../../../route-engine/src/race_start_lane.rs"]
mod race_start_lane;
#[path = "../../../../route-engine/src/race_surface.rs"]
mod race_surface;
#[path = "../../../../route-engine/src/race_surface_speed.rs"]
mod race_surface_speed;
#[path = "../../../../route-engine/src/race_light.rs"]
mod race_light;
#[path = "../../../../route-engine/src/race_lamp_range.rs"]
mod race_lamp_range;
#[path = "../../../../route-engine/src/traffic_lamp.rs"]
mod traffic_lamp;
#[path = "../../../../route-engine/src/race_light_brake.rs"]
mod race_light_brake;
#[path = "../../../../route-engine/src/race_transfer.rs"]
mod race_transfer;
#[path = "../../../../route-engine/src/supervisor.rs"]
mod supervisor;
mod cam_render;
mod world;
mod imu_fault;

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

/// Exact R1-R5 LiDAR degradation matching `run_benchmark.py::degrade_ranges`,
/// keeping near-body noise and raw 8 cm body proximity gate completely untouched.
fn degrade_ranges_regime(ranges: &mut [Option<f64>], regime: &str, speckle_min_m: f64, rng: &mut Random) {
    if regime == "R1" || regime.is_empty() {
        return;
    }
    let n = ranges.len();
    let orig = ranges.to_vec();
    for r in ranges.iter_mut() {
        *r = None;
    }
    let lo = speckle_min_m.clamp(0.15, 5.0);
    let rand_range = |rng: &mut Random| -> f64 {
        let v = lo + (5.5 - lo) * rng.uniform();
        (v * 10000.0).round() / 10000.0
    };
    match regime {
        "R2" => {
            for i in 0..n {
                if let Some(r) = orig[i]
                    && r <= 0.50
                {
                    ranges[i] = Some(r);
                }
            }
        }
        "R3" => {
            for i in 0..n {
                if let Some(r) = orig[i]
                    && r <= 0.50
                {
                    ranges[i] = Some(r);
                } else if rng.uniform() < 0.15 {
                    ranges[i] = Some(rand_range(rng));
                }
            }
            for i in 0..n {
                if ranges[i].is_some() && rng.uniform() < 0.05 {
                    ranges[i] = Some(rand_range(rng));
                }
            }
        }
        "R4" => {
            for i in 0..n {
                if let Some(r) = orig[i]
                    && r <= 0.50
                    && rng.uniform() > 0.20
                {
                    ranges[i] = Some(r);
                } else if rng.uniform() < 0.45 {
                    ranges[i] = Some(rand_range(rng));
                }
            }
        }
        "R5" => {
            for i in 0..n {
                if rng.uniform() < 0.50 {
                    ranges[i] = Some(rand_range(rng));
                }
            }
        }
        _ => {
            ranges.copy_from_slice(&orig);
        }
    }
}

fn scan(
    world: &world::World,
    plant: &Plant,
    seq: i64,
    received: f64,
    sensors: &Value,
    glare: Option<&Value>,
    rng: &mut Random,
    deg_rng: &mut Random,
) -> V {
    let reflective_posts = sensors["reflective_light_posts"].as_bool().unwrap_or(true);
    let noise = number(sensors, "range_noise_m", 0.008);
    let distortion = number(sensors, "scan_distortion", 1.);
    let published = (received - number(sensors, "lidar_delivery_s", 0.012)).max(0.);
    // The N10 collects increasing raw angles. Vehicle calibration rotates raw
    // bins by +5 into body coordinates; the age seam rotates with those bins.
    let yaw_bins = number(sensors, "lidar_clockwise_yaw_bins", 5.).round() as i64;
    let sun = &sensors["sunlight"];
    let direction = number(sun,"world_direction_deg",0.).to_radians();
    let half_angle = number(sun,"half_angle_deg",60.).to_radians();
    let mode = sun["mode"].as_str();
    let inside = |x:f64,y:f64| sun["bounds_xy"].as_array().is_none_or(|b|
        b.len()==4 && x>=b[0].as_f64().unwrap_or(f64::NEG_INFINITY) && x<=b[1].as_f64().unwrap_or(f64::INFINITY)
        && y>=b[2].as_f64().unwrap_or(f64::NEG_INFINITY) && y<=b[3].as_f64().unwrap_or(f64::INFINITY));
    let sparse=&sensors["sparse_patch"];
    let sparse_active=sparse["start_s"].as_f64().zip(sparse["end_s"].as_f64()).is_some_and(|(a,b)|a<=published && published<b);
    let mut patches:Vec<Vec<(usize,f64)>>=sparse["centers"].as_array().map_or_else(Vec::new,|v|vec![Vec::new();v.len()]);
    let mut ranges: Vec<Option<f64>> = (0..360)
        .map(|i| {
            let raw_bin = (i as i64 - yaw_bins).rem_euclid(360);
            let sampled = published - distortion * (359 - raw_bin) as f64 / 360. * 0.1;
            let pose = plant.pose_at(sampled);
            let angle=pose.yaw-(i as f64).to_radians();
            let value = world.cast_lidar(pose, angle, reflective_posts)?;
            let r = value + noise * rng.normal();
            if sparse_active {
                let hit=(pose.x+value*angle.cos(),pose.y+value*angle.sin());
                let mut visible=false;
                for (j,patch) in sparse["centers"].as_array()?.iter().enumerate() {
                    let x=patch["x"].as_f64()?;let y=patch["y"].as_f64()?;
                    let (across,along)=if patch["vertical"].as_bool().unwrap_or(false) {(hit.0-x,hit.1-y)} else {(hit.1-y,hit.0-x)};
                    if across.abs()<0.025 && along.abs()<=0.25 {
                        patches[j].push((i,along));visible=true;
                    }
                }
                if !visible {return None;}
            }
            if mode.is_some() && inside(pose.x,pose.y) && py::wrap(angle-direction).abs()<=half_angle {
                let visible=match mode {
                    Some("range")=>value<=number(sun,"visible_m",0.5),
                    Some("board_patch")=>{
                        let hit=(pose.x+value*angle.cos(),pose.y+value*angle.sin());
                        let center=(number(sun,"patch_x_m",world.length),number(sun,"patch_y_m",world.width/2.));
                        (hit.0-center.0).hypot(hit.1-center.1)<=0.5*number(sun,"visible_m",0.5)
                    }
                    _=>true
                };
                if !visible {return None;}
            }
            (0.02..=12.)
                .contains(&r)
                .then_some((r * 1000.).round() / 1000.)
        })
        .collect();
    if sparse_active {
        let count=sparse["rays_per_patch"].as_u64().unwrap_or(4).max(1) as usize;
        let mut keep=[false;360];
        for patch in &mut patches {
            patch.sort_by(|a,b|a.1.total_cmp(&b.1));
            let n=patch.len().min(count);
            for k in 0..n {
                keep[patch[if n==1 {patch.len()/2} else {k*(patch.len()-1)/(n-1)}].0]=true;
            }
        }
        for (i,r) in ranges.iter_mut().enumerate() {if !keep[i] {*r=None;}}
    }
    if let Some(regime) = glare.and_then(|g| g["radar"].as_str()).or(sensors["lidar_regime"].as_str()) {
        let speckle_min = glare.and_then(|g| g["speckle_min_m"].as_f64()).unwrap_or(number(sensors, "speckle_min_m", 0.15));
        degrade_ranges_regime(&mut ranges, regime, speckle_min, deg_rng);
    }
    if let Some((bin, range)) = glare.and_then(|g| g["echo_bin"].as_i64().zip(g["echo_range_m"].as_f64())) {
        // A compact glint fixed in scanner coordinates moves across the field as the car turns.
        for offset in -2..=2 {
            ranges[(bin + offset).rem_euclid(360) as usize] = Some(range);
        }
    }
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
        "point_age_ms":(0..360).map(|i| (distortion*(359-(i-yaw_bins).rem_euclid(360)) as f64/3.6).round() as u64).collect::<Vec<_>>(),
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
    let mut deg_rng = Random(
        config["degrade_seed"]
            .as_u64()
            .unwrap_or_else(|| config["seed"].as_u64().unwrap_or(1).wrapping_mul(0x517cc1b727220a95))
            .wrapping_add(0x2545f4914f6cdd1d),
    );
    let use_rust_cam_loc = config["camera"]["rust_cam_loc"]
        .as_bool()
        .unwrap_or_else(|| config["sensors"]["lidar_regime"].as_str().is_some());
    let cam_loc_pitch = number(
        &config["camera"],
        "loc_pitch_rad",
        cam_render::default_pitch_rad(),
    );
    let cam_loc_yaw = number(&config["camera"], "loc_yaw_rad", 0.0);
    let cam_loc_fps = number(&config["camera"], "loc_fps", 13.5);
    let cam_loc_age = number(&config["camera"], "loc_capture_age_s", 0.035);
    let cam_loc_latency = number(&config["camera"], "loc_latency_s", 0.035);
    let cam_outdoor_rail = config["camera"]["outdoor_rail"].as_bool().unwrap_or(false);
    let turn_glare = &config["sensors"]["first_turn_glare"];
    let glare_duration = number(turn_glare, "duration_s", 0.0);
    let mut glare_start = None;
    let mut hidden_frames = 0;
    let mut glare_scans = 0;
    let mut renderer = use_rust_cam_loc.then(|| cam_render::CameraRenderer::new(cam_loc_pitch, cam_outdoor_rail));
    let mut cam_loc_next = 0.0_f64;
    let mut cam_loc_frame = 0_i64;
    let mut cam_loc_pending = VecDeque::new();
    let show_housing = config["camera"]["render_lamp_housing"].as_bool().unwrap_or(false);
    let use_housing = config["camera"]["use_housing_range"].as_bool().unwrap_or(true);
    let mut lamp_detector = traffic_lamp::Detector::default();
    let mut housing_sightings=0usize;
    let mut first_housing=Value::Null;
    let mut last_housing=Value::Null;
    let mut sw_beam=None;
    let mut tyre_beam=None;
    let mut previous_wheel:Option<(f64,f64)>=None;
    let mut cam_red_detections = 0_u64;
    let mut cam_blue_detections = 0_u64;
    let mut cam_frames_rendered = 0_u64;
    // Physical cone-wrap tracking against TRUE world cone positions.
    let c0 = (world.cones[0].0, world.cones[0].1);
    let c1 = (world.cones[1].0, world.cones[1].1);
    let mut c0_prev_ang: Option<f64> = None;
    let mut c0_unwrapped = 0.0_f64;
    let mut c0_min_ang = 0.0_f64;
    let mut c0_max_ang = 0.0_f64;
    let mut c0_seen_south = false;
    let mut c0_seen_east = false;
    let mut c0_seen_north = false;
    let mut c0_seen_west_exit = false;
    let mut c0_east_at: Option<f64> = None;
    let mut c1_prev_ang: Option<f64> = None;
    let mut c1_unwrapped = 0.0_f64;
    let mut c1_min_ang = 0.0_f64;
    let mut c1_max_ang = 0.0_f64;
    let mut c1_seen_south = false;
    let mut c1_seen_west = false;
    let mut c1_seen_north = false;
    let mut c1_seen_east_exit = false;
    let mut last_observed_cones = json!([null, null]);
    let mut last_cone_sources = json!(["none", "none"]);
    let mut cone1_first_obs_t: Option<f64> = None;
    let mut cone2_first_obs_t: Option<f64> = None;
    let mut trace = BufWriter::new(File::create(out.join("trace.jsonl"))?);
    let mut events_file = BufWriter::new(File::create(out.join("decisions.jsonl"))?);
    let mut steering_file = BufWriter::new(File::create(out.join("steering.jsonl"))?);
    let mut sent_steering = None;
    let mut sup = supervisor::Supervisor::new(true, "surface-sim", "rust_engine");
    sup.camera = false;
    sup.race_program = true;
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
    let mut same_tick_lamp_release = false;
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
    let mut imu_faults=imu_fault::Faults::new(&config["sensors"]["imu_spikes"],max_t);
    let mut forward_after_brake = false;
    let mut brake_seen = false;
    // Rule timing and the 10 s stationary rule, judged on the true pose.
    let start_line_x = number(&config["layout"], "start_line_x_m", 0.8);
    let mut start_line_at = None;
    let mut finish_line_at = None;
    let mut still_since: Option<f64> = None;
    let mut stopped_elsewhere_s = 0_f64;
    // A real IMU sits ahead of the rotation centre, so its forward integral
    // loses lever * w^2 dt through every loop (car runs: about -.95 m/s a loop).
    let imu_lever = number(&config["sensors"], "imu_lever_arm_m", 0.);
    let mut imu_lever_loss = 0_f64;
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
            imu_lever_loss += imu_lever * car.yaw_rate * car.yaw_rate * 0.02;
        }
        let (front, rear) = world.front_rear_x(car.pose);
        let actual_wheel_gap=world.front_wheel_gap(car.pose);
        if tyre_beam.is_none() && let Some(g)=actual_wheel_gap && g<=0.0 && car.pose.yaw.cos()>0.0
            && let Some((old_t,old_g))=previous_wheel && old_g>0.0 {
            tyre_beam=Some(old_t+(t-old_t)*old_g/(old_g-g));
        }
        previous_wheel=actual_wheel_gap.map(|g|(t,g));
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
        if glare_duration > 0.0 && glare_start.is_none() && sup.session.is_some() && car.speed > 0.2
            && car.pose.x > number(&config["layout"], "start_divider_m", 3.2)
            && car.pose.yaw >= number(turn_glare, "start_yaw_rad", 0.20) {
            glare_start = Some(t);
        }
        let glare_active = |at: f64| glare_start.is_some_and(|start| (start..start + glare_duration).contains(&at));
        if let Some(rend) = renderer.as_mut() {
            if t + 1e-9 >= cam_loc_next {
                cam_loc_next += 1.0 / cam_loc_fps.max(1.0);
                let captured_t = (t - cam_loc_age).max(0.0);
                let cap_pose = car.pose_at(captured_t);
                let hide = turn_glare["hide_cones"].as_bool().unwrap_or(false) && (glare_active(captured_t)
                    || (turn_glare["hide_from_start"].as_bool() == Some(true) && glare_start.is_none_or(|s| captured_t < s)));
                hidden_frames += usize::from(hide);
                let bgr = rend.render_bgr(&world, cap_pose, cam_loc_yaw, hide, show_housing);
                cam_loc_frame += 1;
                cam_frames_rendered += 1;
                let vis = race_cam_features::extract_visual_frame(
                    bgr,
                    race_cam_features::IMG_W,
                    race_cam_features::IMG_H,
                    cam_loc_frame,
                    captured_t,
                );
                for cn in &vis.cones[..vis.n_cones as usize] {
                    match cn.kind {
                        race_cam_features::ConeKind::RedOrange => cam_red_detections += 1,
                        race_cam_features::ConeKind::Blue => cam_blue_detections += 1,
                    }
                }
                let housing = if show_housing && use_housing && sup.session.as_ref().is_some_and(|s|
                    matches!(s.motion.route_stage,"light_entry"|"light_lane")) {
                    lamp_detector.detect(bgr,640,480);
                    lamp_detector.backplate(bgr,640,480).map(|edges|race_lamp_range::Sighting{frame:cam_loc_frame,at:captured_t,edges})
                } else {None};
                housing_sightings += usize::from(housing.is_some());
                cam_loc_pending.push_back((t + cam_loc_latency, captured_t, vis, housing));
            }
            while cam_loc_pending.front().is_some_and(|x| x.0 <= t + 1e-9) {
                let (_, cap_t, vis, housing) = cam_loc_pending.pop_front().unwrap();
                sup.observe_rust_cam_loc(vis, cap_t);
                if let Some(shot)=housing && let Some(session)=sup.session.as_mut() {
                    session.motion.race_light.housing.latest=Some(shot);
                }
            }
        }
        sup.mono = t;
        let status = dict! {"control"=>dict!{"tick"=>(t*1000.).round() as i64,"seq"=>command_seq,"armed"=>armed,"motor"=>motor,"servo"=>servo}};
        sup.accept_control(&status, t);
        if camera.latest.is_some() {
            sup.set_light(&camera.latest);
        }
        if tick % 5 == 0 {
            seq += 1;
            let glare = glare_active(t - number(&config["sensors"], "lidar_delivery_s", 0.012)).then_some(turn_glare);
            glare_scans += usize::from(glare.is_some());
            let value = scan(&world, &car, seq, t, &config["sensors"], glare, &mut rng, &mut deg_rng);
            sup.accept_lidar(value, t);
        }
        if !started && t >= 1.0 {
            let data = dict! {"op"=>"turn_cone_start","boot"=>"surface-sim","epoch"=>0,"tick"=>(t*1000.).round() as i64,
            "placement_confirmed"=>true,"continue_route"=>true,"max_drive_s"=>40.,"initial_presteer_pwm"=>1670};
            match sup.start_turn_cone(&data, &t.into(), "offline") {
                Ok(_) => {
                    started = true;
                    if let Some(s)=sup.session.as_mut() {s.motion.field_map.planner.start();}
                }
                Err((error, clearance)) => {
                    start_error = Some(format!("{}: {}", error.kind.name(), error.msg));
                    fs::write(out.join("start-error.json"), py::to_json(&clearance))?;
                    break;
                }
            }
        }
        let imu_fault=imu_faults.sample(tick as usize);
        if let Some(session) = sup.session.as_mut() {
            // Offline ablation only: remove the estimator's memory so the same
            // controller and physical surface can be compared without adaptation.
            if config["surface_adaptation"].as_bool() == Some(false) {
                session.motion.race_surface = Default::default();
            }
            session.motion.set_live_imu(motion_sensors::RaceInputs {
                yaw_rate: Some(car.yaw_rate+imu_fault.gyro),
                forward_integral: Some(
                    car.speed + number(&config["sensors"], "imu_drift_mps2", 0.) * t
                        - imu_lever_loss+imu_faults.delta_velocity,
                ),
            });
            sup.renew_owner(t);
            if let Err(error) = py::py_try(|| sup.autonomy_tick(&t.into())) {
                sup.halt(&format!("simulation_controller_exception_{}", error.msg));
            }
            sup.coast_worker.run_pending();
        }
        for e in std::mem::take(&mut sup.emits) {
            if e.op=="drive" {sent_steering=Some((e.seq,e.servo.i(),e.physical.i(),t));}
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
        if let Some(session)=sup.session.as_ref() {
            let lamp=&session.motion.race_light;
            if let Some(r)=lamp.housing.registered {
                let rec=json!({"at":t,"offset_m":r.offset,"end_span_m":r.end_span,
                    "source":lamp.opening_source,"true_wheel_gap_m":actual_wheel_gap});
                if first_housing.is_null() {first_housing=rec.clone();}
                if last_housing.get("offset_m")!=rec.get("offset_m") {last_housing=rec;}
            }
            if sw_beam.is_none() {sw_beam=lamp.beam_at;}
        }
        let turn = report.get("turn");
        let stage = turn.get("route_stage").os().unwrap_or("");
        if sup.session.is_some() {
            let lp=turn.get("open_loop");
            let lane=turn.get("start_lane_feedback");
            let maps=turn.get("field_localization");
            writeln!(steering_file,"{}",json!({"t":t,"seq":seq,"stage":stage,"phase":turn.get("phase").os(),"reason":turn.get("reason").os(),
                "map_seq":maps.get("source_seq").oi(),"localized":maps.get("localized").is_true(),"previews":json_v(maps.get("preview_pwm")),
                "loop_seq":lp.get("source_seq").oi(),"support":lp.get("support_source").os(),"arbitration":lp.get("steering_basis").os(),
                "lidar_pwm":lp.get("lidar_target_pwm").oi(),"selected_pwm":lp.get("target_pwm").oi(),"loop_map_pwm":lp.get("map_target_pwm").oi(),
                "lane_seq":lane.get("source_seq").oi(),"lane_source":lane.get("steering_source").os(),
                "natural_pwm":turn.get("natural_steering_target").oi(),"target_pwm":turn.get("steering_target").oi(),"slewed_pwm":turn.get("servo").oi(),
                "sent":sent_steering,"acked_seq":command_seq,"acked_physical_pwm":servo,"wall_turn":turn.get("wall_turn").get("active").is_true(),
                "imu_event":imu_fault.event,"imu_gyro_spike":imu_fault.gyro,"imu_accel_spike":imu_fault.accel}))?;
        }
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
        if start_line_at.is_none() && front >= start_line_x && car.pose.yaw.cos() > 0. {
            start_line_at = Some(t);
        }
        if finish_line_at.is_none()
            && front >= world.length - 0.8
            && car.pose.yaw.cos() > 0.
            && car.pose.y >= world.width - world.light_width
        {
            finish_line_at = Some(t);
        }
        if move_at.is_some()
            && car.speed < 0.02
            && cross != "waiting"
            && lamp != "waiting"
            && sup.session.is_some()
        {
            let since = *still_since.get_or_insert(t);
            stopped_elsewhere_s = stopped_elsewhere_s.max(t - since);
        } else {
            still_since = None;
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
        let stopped_now = turn.get("route_events").get("light_stop").get("scan").get("received_at").of()
            .is_some_and(|at|(t-at).abs()<0.001);
        // The production controller can log its stop and use a previously seen
        // green in one tick. Sample the physical pose on that event as well.
        let observe_lamp_stop = lamp=="waiting" || stopped_now;
        same_tick_lamp_release |= stopped_now && lamp=="released";
        if light_stop_event.is_none() && observe_lamp_stop {
            light_stop_event = Some(
                json!({"t":t,"true_gap_m":world.gantry_x-front,"rear_past_beam_m":rear-world.beam_x,"speed_mps":car.speed}),
            );
        }
        if light_stop.is_none() && observe_lamp_stop && car.speed < 0.05 {
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
        // Track true physical trajectory winding around actual Cone 0 (CCW) and Cone 1 (CW).
        // Freeze Cone 0 sweep once the car moves on to Cone 1 so the finish-lane pass does not overwrite it.
        {
            if !c1_seen_west {
                let dx0 = car.pose.x - c0.0;
                let dy0 = car.pose.y - c0.1;
                if dx0.hypot(dy0) <= 3.0 {
                    let ang0 = dy0.atan2(dx0);
                    if let Some(prev) = c0_prev_ang {
                        c0_unwrapped += py::wrap(ang0 - prev);
                        c0_min_ang = c0_min_ang.min(c0_unwrapped);
                        c0_max_ang = c0_max_ang.max(c0_unwrapped);
                    } else {
                        c0_unwrapped = 0.0;
                        c0_min_ang = 0.0;
                        c0_max_ang = 0.0;
                    }
                    c0_prev_ang = Some(ang0);
                } else {
                    c0_prev_ang = None;
                }
                if dy0 < -0.10 && dx0.abs() < 1.8 {
                    c0_seen_south = true;
                }
                if c0_seen_south && dx0 > 0.10 && dy0.abs() < 1.8 {
                    c0_seen_east = true;
                    c0_east_at.get_or_insert(t);
                }
                if c0_seen_east && dy0 > 0.10 && dx0.abs() < 1.8 {
                    c0_seen_north = true;
                }
                if c0_seen_north && dx0 < -0.10 && dy0 > 0.0 {
                    c0_seen_west_exit = true;
                }
            }

            if c0_east_at.is_some() {
                let dx1 = car.pose.x - c1.0;
                let dy1 = car.pose.y - c1.1;
                if dx1.hypot(dy1) <= 3.0 {
                    let ang1 = dy1.atan2(dx1);
                    if let Some(prev) = c1_prev_ang {
                        c1_unwrapped += py::wrap(ang1 - prev);
                        c1_min_ang = c1_min_ang.min(c1_unwrapped);
                        c1_max_ang = c1_max_ang.max(c1_unwrapped);
                    } else {
                        c1_unwrapped = 0.0;
                        c1_min_ang = 0.0;
                        c1_max_ang = 0.0;
                    }
                    c1_prev_ang = Some(ang1);
                } else {
                    c1_prev_ang = None;
                }
                if dy1 < -0.10 && dx1.abs() < 1.8 {
                    c1_seen_south = true;
                }
                if c1_seen_south && dx1 < -0.10 && dy1.abs() < 1.8 {
                    c1_seen_west = true;
                }
                if c1_seen_west && dy1 > 0.10 && dx1.abs() < 1.8 {
                    c1_seen_north = true;
                }
                if c1_seen_north && dx1 > 0.10 && dy1 > 0.0 {
                    c1_seen_east_exit = true;
                }
            }
        }
        if let Some(maps) = sup.session.as_ref().map(|s| s.report.get("turn").get("field_localization")) {
            let oc = json_v(maps.get("observed_cones_world"));
            if oc.is_array() {
                if cone1_first_obs_t.is_none() && oc[0].is_array() {
                    cone1_first_obs_t = Some(t);
                }
                if cone2_first_obs_t.is_none() && oc[1].is_array() {
                    cone2_first_obs_t = Some(t);
                }
                last_observed_cones = oc;
                last_cone_sources = json_v(maps.get("cone_sources"));
            }
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
            let scan_truth=car.pose_at(sup.scan.get("at_ms").of().unwrap_or(0.)/1000.);
            writeln!(
                trace,
                "{}",
                json!({"t":t,"seq":seq,"pose":[car.pose.x,car.pose.y,car.pose.yaw],"speed":car.speed,
                "motor":motor,"servo":servo,"wall_gap":wall,"cone_gap":cone,"stage":stage,"reason":turn.get("reason").os(),
                "truth_at_scan":[scan_truth.x,scan_truth.y,scan_truth.yaw],"imu_event":imu_fault.event,"true_yaw_rate":car.yaw_rate,
                "injected_gyro_rad_s":imu_fault.gyro,"injected_accel_m_s2":imu_fault.accel,"integrated_fault_m_s":imu_faults.delta_velocity,
                "crosswalk":json_v(turn.get("start_lane_feedback").get("crosswalk")),
                "surface_response":json_v(turn.get("surface_response")),
                "field_localization":json_v(turn.get("field_localization")),
                "first_steering_source":json_v(turn.get("start_lane_feedback").get("steering_source")),
                 "housing":if show_housing {sup.session.as_ref().map(|s|{
                    let m=&s.motion;let l=&m.race_light;
                    let scan=sup.scan.merged(&dict!{"received_at"=>sup.scan_at.clone()});
                    let plane=l.housing.latest.and_then(|f|m.field_map.housing_plane(&scan,f));
                    json!({"frame":l.housing.latest.map(|f|f.frame),"age":l.housing.latest.map(|f|t-f.at),
                        "plane":plane.map(|p|[p.ahead,p.offset,p.uncertainty,if p.both {1.0}else{0.0}]),
                        "motion_supported":m.field_map.motion_supported(&scan),"source":l.opening_source,
                        "registered":l.housing.registered.map(|r|r.offset),"wheel_gap":l.front_wheel_gap_m,
                        "physical_wheel_gap":actual_wheel_gap,"beam_at":l.beam_at})
                })} else {None},
                "valid_rays":sup.scan.get("ranges").ol().map(|rs|rs.iter().filter(|v|v.is_num()).count()),
                "lane":json_v(turn.get("lane_feedback")),"open":json_v(turn.get("open_feedback")),"light":json_v(turn.get("light_feedback")),"wall_turn":json_v(turn.get("wall_turn")),"light_line":json_v(turn.get("light_line")),
                "loop":json_v(turn.get("open_loop")),"quality":json_v(report.get("quality_issues"))})
            )?;
        }
        if contact.is_some() || (started && sup.session.is_none() && car.speed < 0.01) {
            break;
        }
    }
    trace.flush()?;
    events_file.flush()?;
    steering_file.flush()?;
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
    let cone1_ccw_deg = (c0_max_ang - c0_min_ang).to_degrees();
    let cone2_cw_deg = (c1_max_ang - c1_min_ang).to_degrees();
    let wrapped_cone1_ccw = c0_seen_south
        && c0_seen_east
        && c0_seen_north
        && c0_seen_west_exit
        && (c0_max_ang - c0_min_ang).to_degrees() >= 150.0
        && c0_unwrapped > 0.0;
    let wrapped_cone2_cw = c1_seen_south
        && c1_seen_west
        && c1_seen_north
        && c1_seen_east_exit
        && (c1_max_ang - c1_min_ang).to_degrees() >= 150.0
        && c1_unwrapped < 0.0;
    let wrapped_both_actual_cones = wrapped_cone1_ccw && wrapped_cone2_cw;
    let cone_err = |idx: usize, truth: (f64, f64)| -> Value {
        last_observed_cones[idx]
            .as_array()
            .and_then(|a| a[0].as_f64().zip(a[1].as_f64()))
            .map(|(x, y)| json!((x - truth.0).hypot(y - truth.1)))
            .unwrap_or(Value::Null)
    };
    let modeled_task_pass = controller_finish
        && contact.is_none()
        && crosswalk_before_line
        && crosswalk_still_s + 1e-9 >= 3.
        && light_stopped_in_zone
        && !crossed_light_before_green
        && finish_stopped_in_zone;
    let strict_pass = modeled_task_pass && wrapped_both_actual_cones;
    let summary = json!({
        "label":config["label"],"wall_seconds":wall_start.elapsed().as_secs_f64(),"sim_seconds":final_t,
        "lidar_regime":config["sensors"]["lidar_regime"].as_str().unwrap_or("R1"),
        "seed":config["seed"],"degrade_seed":config["degrade_seed"],
        "rust_cam_loc":use_rust_cam_loc,
        "rendered_lamp_housing":show_housing,"ranged_lamp_frames":housing_sightings,
        "first_housing":first_housing,"last_housing":last_housing,"software_beam_at":sw_beam,"true_wheel_beam_at":tyre_beam,
        "first_turn_glare_start_s":glare_start,"occluded_camera_frames":hidden_frames,"first_turn_glare_scans":glare_scans,
        "cam_frames_rendered":cam_frames_rendered,
        "cam_red_detections":cam_red_detections,"cam_blue_detections":cam_blue_detections,
        "observed_cones_world":last_observed_cones,"cone_sources":last_cone_sources,
        "cone_first_obs_t":[cone1_first_obs_t, cone2_first_obs_t],
        "cone_est_err_m":[cone_err(0, c0), cone_err(1, c1)],
        "cone1_winding_deg":cone1_ccw_deg,"cone2_winding_deg":cone2_cw_deg,
        "wrapped_cone1_ccw":wrapped_cone1_ccw,"wrapped_cone2_cw":wrapped_cone2_cw,
        "wrapped_both_actual_cones":wrapped_both_actual_cones,
        "strict_pass":strict_pass,
        "surface_adaptation":config["surface_adaptation"].as_bool().unwrap_or(true),
        "start_error":start_error,"terminal_reason":report.get("reason").os(),"controller_finished":sup.session.is_none(),
        "contact":contact,"min_wall_gap_m":min_wall,"min_cone_gap_m":min_cone,
        "crosswalk_stop_event":crosswalk_stop_event,"crosswalk_stop":crosswalk_stop,
        "crosswalk_stationary_s":crosswalk_still_s,"crosswalk_stopped_before_line":crosswalk_before_line,
        "light_stop_event":light_stop_event,"same_tick_lamp_release":same_tick_lamp_release,"light_stop":light_stop,"light_stopped_in_zone":light_stopped_in_zone,
        "light_crossed_before_green":crossed_light_before_green,"finish_stop":finish_stop,"route_events":route_events,
        "controller_finish":controller_finish,"physical_contact_free":contact.is_none(),
        "finish_stopped_in_zone":finish_stopped_in_zone,"modeled_task_pass":modeled_task_pass,
        "simulated_effective_s":lap,"light_wait_s":light_wait,"forward_after_brake":forward_after_brake,
        "start_line_at":start_line_at,"finish_line_at":finish_line_at,"stopped_elsewhere_s":stopped_elsewhere_s,
        "beam_trigger_at":trigger,
        "max_traction_demand_ratio":car.max_traction_ratio,"traction_limited_s":car.slip_s,
        "stage_changes":stage_changes,"world":world.description(),"plant":car.model(),
        "sensors":{
            "lidar_regime":config["sensors"]["lidar_regime"].as_str().unwrap_or("R1"),
            "lidar_delivery_s":number(&config["sensors"],"lidar_delivery_s",0.012),
            "lidar_clockwise_yaw_bins":number(&config["sensors"],"lidar_clockwise_yaw_bins",5.),
            "scan_distortion":number(&config["sensors"],"scan_distortion",1.),
            "camera_capture_age_s":number(&config["camera"],"capture_age_s",0.15),
            "camera_latency_s":number(&config["camera"],"latency_s",0.24),
            "loc_fps":cam_loc_fps,"loc_capture_age_s":cam_loc_age,"loc_latency_s":cam_loc_latency
        },
        "limitations":["Synthetic 640x480 BGR perspective+distortion camera rendered from true world boards/cones and processed through production extract_visual_frame; ideal crosswalk/traffic-light semantic signal retained.",
            "Empirical steering and speed response; friction-circle clipping approximates tire slip, without lateral tire dynamics.",
            "No cone displacement or impact dynamics. First true contact ends the simulation."],
        "imu_spikes_applied":imu_faults.applied(),"result":json_v(&report)
    });
    fs::write(
        out.join("summary.json"),
        serde_json::to_string_pretty(&summary)?,
    )?;
    println!(
        "{}",
        json!({"out":out,"regime":summary["lidar_regime"],"strict_pass":strict_pass,
               "wrapped":[wrapped_cone1_ccw, wrapped_cone2_cw],
               "cone_err":summary["cone_est_err_m"],
               "seconds":summary["simulated_effective_s"],"reason":summary["terminal_reason"],
               "contact":summary["contact"],"start_error":summary["start_error"],"wall_seconds":summary["wall_seconds"]})
    );
    Ok(())
}
