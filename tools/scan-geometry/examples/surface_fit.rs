//! Fit the fixed y53 training and y54 validation segments from this workspace.
//! Usage: surface_fit REPOSITORY_ROOT OUTPUT_DIRECTORY
use serde_json::{Value, json};
use std::{
    collections::BTreeMap,
    fs,
    io::{BufRead, BufReader},
    path::Path,
};
#[derive(Clone)]
struct Sample {
    t: f64,
    motor: i64,
    v: Option<f64>,
    raw: Option<f64>,
}
struct Segment {
    name: String,
    rows: Vec<Sample>,
    distance: Option<f64>,
}
fn num(v: &Value) -> Option<f64> {
    v.as_f64()
}
fn median(mut x: Vec<f64>) -> f64 {
    x.sort_by(f64::total_cmp);
    x[x.len() / 2]
}
fn rear(row: &Value) -> f64 {
    let a = &row["lidar"]["scan"]["ranges"];
    median(
        (176..=184)
            .filter_map(|i| num(&a[i]).map(|r| r * ((i as f64 - 180.).to_radians().cos())))
            .collect(),
    )
}
fn load(root: &Path, name: &str) -> Vec<Segment> {
    let rows:Vec<Value>=BufReader::new(fs::File::open(root.join(format!("work/vehicle-dynamic-deploy-20261008/left-cone-trial-20261104-{name}/run/feedback.jsonl"))).unwrap()).lines().map(|s|serde_json::from_str(&s.unwrap()).unwrap()).collect();
    let sum: Value = serde_json::from_str(
        &fs::read_to_string(root.join(format!(
            "work/vehicle-dynamic-deploy-20261008/left-cone-trial-20261104-{name}/run/summary.json"
        )))
        .unwrap(),
    )
    .unwrap();
    let ev = &sum["result"]["turn"]["route_events"];
    let seq = |key: &str| ev[key]["scan"]["seq"].as_i64().unwrap();
    let ranges = [
        (
            "crosswalk_approach",
            seq("start_lane"),
            seq("crosswalk_stop"),
        ),
        (
            "crosswalk_exit",
            seq("crosswalk_go"),
            seq("first_approach_handover") - 1,
        ),
        ("finish", seq("light_go"), seq("finish_stop")),
    ];
    ranges.iter().map(|(segment,start,end)|{
   let within:Vec<_>=rows.iter().filter(|r|{let s=r["lidar"]["scan"]["seq"].as_i64().unwrap();s>=*start &&s<=*end}).collect();
   let mut samples=Vec::new();
   // Include the decision preceding the first observed powered acknowledgement.
   let before=rows.iter().rfind(|r|r["lidar"]["scan"]["seq"].as_i64().unwrap()<*start);
   if let Some(r)=before {samples.push(Sample{t:r["lidar"]["received_program_s"].as_f64().unwrap(),motor:1500,v:Some(0.),raw:None});}
   for r in &within{
      let t=&r["state"]["autonomy"]["active"]["turn"];let lf=&t["lane_feedback"];let sf=&t["start_lane_feedback"];
      let (v,raw)=match *segment{
       "crosswalk_approach"=>(num(&sf["crosswalk"]["imu_speed_mps"]),num(&sf["crosswalk"]["speed_mps"])),
       "crosswalk_exit"=>(num(&sf["first_approach_drive"]["speed_now_mps"]),num(&sf["speed_mps"])),
       _=>(num(&lf["imu_speed_mps"]),num(&lf["speed_mps"]))
      };
      let motor=t["motor"].as_i64().unwrap_or(r["state"]["status"]["control"]["motor"].as_i64().unwrap());
      samples.push(Sample{t:r["lidar"]["received_program_s"].as_f64().unwrap(),motor,v,raw});
   }
   let distance=if within.len()>1{if *segment=="finish"{
    num(&within.first().unwrap()["state"]["autonomy"]["active"]["turn"]["lane_feedback"]["front_range_m"]).zip(num(&within.last().unwrap()["state"]["autonomy"]["active"]["turn"]["lane_feedback"]["front_range_m"])).map(|(a,b)|a-b)
   }else if *segment=="crosswalk_approach"{Some(rear(within.last().unwrap())-rear(within.first().unwrap()))}else{None}}else{None};
   Segment{name:format!("{name}/{segment}"),rows:samples,distance}
 }).collect()
}
fn interp(table: &[(f64, f64)], x: f64) -> f64 {
    for p in table.windows(2) {
        if x <= p[1].0 {
            let w = ((x - p[0].0) / (p[1].0 - p[0].0)).clamp(0., 1.);
            return p[0].1 + w * (p[1].1 - p[0].1);
        }
    }
    table.last().unwrap().1
}
#[derive(Clone)]
struct Model {
    targets: [f64; 4],
    tau: [f64; 4],
    delay: f64,
    coast: f64,
    brake: f64,
    accel: f64,
}
impl Model {
    fn default() -> Self {
        Self {
            targets: [1.1, 2.2, 2.7, 4.2],
            tau: [0.7; 4],
            delay: 0.1,
            coast: 0.58,
            brake: 0.95,
            accel: 4.,
        }
    }
    fn fit_seed() -> Self {
        Self {
            targets: [1.75, 2.2, 2.7, 4.2],
            tau: [2.5, 0.9, 1., 0.7],
            ..Self::default()
        }
    }
    fn net(&self, m: i64, v: f64) -> f64 {
        if m < 1500 {
            return -self.brake;
        }
        if m == 1500 {
            return -self.coast;
        }
        let keys = [1550., 1560., 1570., 1600.];
        let targets: Vec<_> = keys.iter().copied().zip(self.targets).collect();
        let tau: Vec<_> = keys.iter().copied().zip(self.tau).collect();
        ((interp(&targets, m as f64) - v) / interp(&tau, m as f64)).clamp(-self.coast, self.accel)
    }
    fn vec(&self) -> Vec<f64> {
        self.targets
            .into_iter()
            .chain(self.tau)
            .chain([self.delay, self.coast, self.brake, self.accel])
            .collect()
    }
    fn from(p: &[f64]) -> Self {
        Self {
            targets: p[..4].try_into().unwrap(),
            tau: p[4..8].try_into().unwrap(),
            delay: p[8],
            coast: p[9],
            brake: p[10],
            accel: p[11],
        }
    }
    fn valid(&self) -> bool {
        self.targets.windows(2).all(|p| p[1] >= p[0])
    }
    fn json(&self) -> Value {
        json!({"speed_table":[[1550.,self.targets[0]],[1560.,self.targets[1]],[1570.,self.targets[2]],[1600.,self.targets[3]]],"speed_tau_table":[[1550.,self.tau[0]],[1560.,self.tau[1]],[1570.,self.tau[2]],[1600.,self.tau[3]]],"motor_delay_s":self.delay,"coast_decel_mps2":self.coast,"brake_decel_mps2":self.brake,"drive_accel_max_mps2":self.accel})
    }
}
fn simulate(m: &Model, s: &Segment) -> Value {
    let begin = s.rows.first().unwrap().t;
    let end = s.rows.last().unwrap().t;
    let (mut t, mut v, mut distance, mut motor_index, mut next) = (begin, 0., 0., 0, 0);
    let (mut se, mut maxe, mut n, mut peak, mut observed_peak) = (0., 0_f64, 0, 0_f64, 0_f64);
    let mut trace = Vec::new();
    while t <= end + 0.0001 {
        while next < s.rows.len() && s.rows[next].t <= t + 0.0001 {
            let p = &s.rows[next];
            if let Some(target) = p.v {
                let e = v - target;
                se += e * e;
                maxe = maxe.max(e.abs());
                observed_peak = observed_peak.max(target);
                n += 1;
                trace.push(json!([p.t - begin, p.motor, target, v, p.raw]));
            }
            next += 1;
        }
        while motor_index + 1 < s.rows.len() && s.rows[motor_index + 1].t + m.delay <= t {
            motor_index += 1
        }
        let motor = s.rows[motor_index].motor;
        let h: f64 = 0.002;
        let a = m.net(motor, v);
        let nv = (v + a * h).max(0.);
        distance += (v + nv) * h / 2.;
        v = nv;
        peak = peak.max(v);
        t += h;
    }
    json!({"name":s.name,"samples":n,"speed_rmse_mps":(se/n as f64).sqrt(),"speed_max_abs_error_mps":maxe,"observed_peak_mps":observed_peak,"predicted_peak_mps":peak,"integrated_distance_m":distance,"lidar_displacement_m":s.distance,"velocity_trace_columns":["seconds","command_pwm","reported_imu_speed","model_speed","raw_lidar_slope"],"trace":trace})
}
fn score(m: &Model, ss: &[Segment]) -> f64 {
    ss.iter()
        .map(|s| {
            let r = simulate(m, s);
            r["speed_rmse_mps"].as_f64().unwrap().powi(2)
        })
        .sum::<f64>()
        / ss.len() as f64
}
fn fit(ss: &[Segment]) -> Model {
    let bounds = [
        (1.3, 2.1),
        (1.6, 3.0),
        (1.9, 3.4),
        (3.3, 5.5),
        (0.8, 4.),
        (0.35, 2.),
        (0.35, 2.),
        (0.3, 1.2),
        (0., 0.2),
        (0.45, 0.75),
        (0.75, 1.15),
        (3., 5.),
    ];
    let mut best = Model::fit_seed();
    let mut loss = score(&best, ss);
    for pass in 0..8 {
        let mut improved = false;
        for k in 0..12 {
            let p = best.vec();
            let width = (bounds[k].1 - bounds[k].0)
                * [0.20, 0.12, 0.08, 0.05, 0.03, 0.02, 0.01, 0.005][pass];
            for step in [-2., -1., 1., 2.] {
                let mut q = p.clone();
                q[k] = (p[k] + width * step).clamp(bounds[k].0, bounds[k].1);
                let candidate = Model::from(&q);
                if !candidate.valid() {
                    continue;
                }
                let l = score(&candidate, ss);
                if l < loss {
                    loss = l;
                    best = candidate;
                    improved = true;
                }
            }
        }
        if !improved && pass > 4 {
            break;
        }
    }
    best
}
fn main() {
    let args: Vec<_> = std::env::args().collect();
    assert_eq!(
        args.len(),
        3,
        "usage: surface_fit REPOSITORY_ROOT OUTPUT_DIRECTORY"
    );
    let root = Path::new(&args[1]);
    let out = Path::new(&args[2]).to_owned();
    fs::create_dir_all(&out).unwrap();
    let train = load(root, "y53");
    let validation = load(root, "y54");
    let old = Model::default();
    let fitted = fit(&train);
    let mut coverage = BTreeMap::<i64, (f64, usize)>::new();
    for s in train.iter().chain(&validation) {
        for p in s.rows.windows(2) {
            let c = coverage.entry(p[0].motor).or_default();
            c.0 += p[1].t - p[0].t;
            c.1 += 1;
        }
    }
    let mut slopes = BTreeMap::<String, Vec<f64>>::new();
    for segment in train.iter().chain(&validation) {
        let mut unchanged_since = segment.rows[0].t;
        for pair in segment.rows.windows(2) {
            let (previous, current) = (&pair[0], &pair[1]);
            if previous.motor != current.motor {
                unchanged_since = current.t;
                continue;
            }
            let Some((v0, v1)) = previous.v.zip(current.v) else {
                continue;
            };
            if previous.t - unchanged_since < 0.15 || v0.min(v1) < 0.15 {
                continue;
            }
            let speed = (v0 + v1) * 0.5;
            let bucket = if speed < 0.5 {
                "v<0.5"
            } else if speed < 1.2 {
                "v=0.5..1.2"
            } else {
                "v>=1.2"
            };
            slopes
                .entry(format!("{}/{bucket}", current.motor))
                .or_default()
                .push((v1 - v0) / (current.t - previous.t));
        }
    }
    let slope_rows: Vec<_> = slopes.iter().map(|(name, values)| {
        let mut ordered = values.clone();
        ordered.sort_by(f64::total_cmp);
        json!({"motor_speed_bin":name,"samples":ordered.len(),"median_accel_mps2":median(ordered.clone()),
               "minimum":ordered[0],"maximum":ordered[ordered.len()-1]})
    }).collect();
    let report = json!({"fit_scope":"Three straight/release segments. y53 training, y54 held out. Reported current IMU velocities are reference; lidar displacement is independent.",
 "input_note":"Commands use saved turn.motor, because feedback motor ACKs omit some brief changes. Timing resolution is approximately 0.1-0.2 s, so delay fitting is not a calibrated actuator latency.",
 "default":old.json(),"fitted":fitted.json(),"default_train_rmse_mps":score(&old,&train).sqrt(),"fitted_train_rmse_mps":score(&fitted,&train).sqrt(),"default_validation_rmse_mps":score(&old,&validation).sqrt(),"fitted_validation_rmse_mps":score(&fitted,&validation).sqrt(),
 "default_segments":train.iter().chain(&validation).map(|s|simulate(&old,s)).collect::<Vec<_>>(),
 "fitted_segments":train.iter().chain(&validation).map(|s|simulate(&fitted,s)).collect::<Vec<_>>(),
 "pwm_training_and_validation_exposure":coverage.iter().map(|(k,(t,n))|json!({"pwm":k,"seconds":t,"intervals":n})).collect::<Vec<_>>(),
 "stable_command_observed_acceleration":slope_rows,
 "limitations":["Targets at 1560-1600 are extrapolated asymptotic parameters, not measured terminal speeds.","The 1560 target and time constant hit their search bounds; its separate response is poorly identified. 1555,1575,1580,1585,1590 have insufficient straight-segment observations and need interpolation/uncertainty scenarios.","No lateral tire identification. No layout/controller tuning.","Snapshots and IMU speed drift constrain accuracy. The exit segment bends, so uncorrected rear rays are excluded as displacement reference.","Finish IMU speed integration and independent front-wall displacement differ by up to ~0.33 m on y53; retain that model uncertainty before inferring stopping accuracy."]});
    fs::write(
        out.join("report.json"),
        serde_json::to_string_pretty(&report).unwrap(),
    )
    .unwrap();
    fs::write(
        out.join("fitted-plant.json"),
        serde_json::to_string_pretty(&fitted.json()).unwrap(),
    )
    .unwrap();
    println!(
        "{}",
        json!({"default_train":score(&old,&train).sqrt(),"fit_train":score(&fitted,&train).sqrt(),"default_validation":score(&old,&validation).sqrt(),"fit_validation":score(&fitted,&validation).sqrt(),"fitted":fitted.json()})
    );
}
