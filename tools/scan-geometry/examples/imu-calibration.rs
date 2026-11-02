//! Host-side planar IMU calibration. Pass the run/ directories of the driven trials.
//! cargo run --release --example imu-calibration -- RUN1 RUN2 RUN3 > calibration.json
use serde_json::{Value, json};
use std::{
    error::Error,
    fs::File,
    io::{self, BufRead, BufReader},
    path::Path,
};
use xt_stcar_robot_core::{ImuSample, Vec3};

type V3 = [f64; 3];
type Result<T> = std::result::Result<T, Box<dyn Error>>;
fn vec(v: Vec3) -> V3 {
    [v.x, v.y, v.z]
}
fn add(a: V3, b: V3) -> V3 {
    std::array::from_fn(|i| a[i] + b[i])
}
fn sub(a: V3, b: V3) -> V3 {
    std::array::from_fn(|i| a[i] - b[i])
}
fn mul(a: V3, s: f64) -> V3 {
    a.map(|x| x * s)
}
fn dot(a: V3, b: V3) -> f64 {
    (0..3).map(|i| a[i] * b[i]).sum()
}
fn unit(a: V3) -> V3 {
    mul(a, 1. / dot(a, a).sqrt())
}
fn cross(a: V3, b: V3) -> V3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn median(a: &mut [f64]) -> f64 {
    a.sort_by(f64::total_cmp);
    (a[(a.len() - 1) / 2] + a[a.len() / 2]) / 2.
}
fn lines(p: &Path) -> io::Result<io::Lines<BufReader<File>>> {
    Ok(BufReader::new(File::open(p)?).lines())
}

#[derive(Clone, Copy)]
struct Sample {
    t: f64,
    a: V3,
    w: V3,
}
#[derive(Clone, Copy)]
struct Integral {
    t: f64,
    velocity: V3,
    displacement: V3,
    angle: V3,
}
struct Scan {
    t: f64,
    heading: Option<f64>,
    wall: Option<(bool, f64)>,
    stationary: bool,
    powered: bool,
}
#[derive(Clone, Copy)]
struct Observation {
    t: [f64; 3],
    reference: f64,
}
struct Run {
    name: String,
    imu: Vec<Sample>,
    rest_a: V3,
    rest_w: V3,
    stationary_n: usize,
    yaw: Vec<Observation>,
    forward: Vec<Observation>,
}

// End wall behind the start straight, wall ahead in the finish lane. Project ranges
// onto the lane axis, with a median across beams so a post does not set the range.
fn wall_position(scan: &Value, h: f64, start: bool) -> Option<(bool, f64)> {
    if h.abs() > 0.20 {
        return None;
    }
    let bearing = h + if start { std::f64::consts::PI } else { 0. };
    let center = bearing.to_degrees().round() as i32;
    let mut xs: Vec<_> = (-6..=6)
        .filter_map(|j| {
            let i = center + j;
            let r = scan["ranges"][i.rem_euclid(360) as usize].as_f64()?;
            Some(r * ((i as f64).to_radians() - bearing).cos())
        })
        .collect();
    (xs.len() >= 3).then(|| (start, median(&mut xs) * if start { 1. } else { -1. }))
}

fn load(dir: &Path) -> Result<Run> {
    let mut imu = Vec::new();
    for line in lines(&dir.join("imu.jsonl"))? {
        let s: ImuSample = serde_json::from_str(&line?)?;
        imu.push(Sample {
            t: s.captured_at.0 as f64 * 0.001,
            a: vec(s.acceleration_mps2),
            w: vec(s.angular_velocity_radps),
        });
    }
    let mut scans = Vec::new();
    let mut last_seq = None;
    for line in lines(&dir.join("feedback.jsonl"))? {
        let r: Value = serde_json::from_str(&line?)?;
        let scan = &r["lidar"]["scan"];
        let Some(seq) = scan["seq"].as_i64() else {
            continue;
        };
        if last_seq == Some(seq) {
            continue;
        }
        last_seq = Some(seq);
        let turn = &r["state"]["autonomy"]["active"]["turn"];
        let Some(t) = r["lidar"]["received_program_s"].as_f64() else {
            continue;
        };
        let compass = &turn["compass"];
        let heading = compass["heading_rad"]
            .as_f64()
            .filter(|_| compass["measured"] == true && compass["source_at_ms"] == scan["at_ms"]);
        let stage = turn["route_stage"].as_str();
        let wall = heading.and_then(|h| match stage {
            Some("start_lane") => wall_position(scan, h, true),
            Some("light_lane") => wall_position(scan, h, false),
            _ => None,
        });
        let motor = turn["motor"].as_i64().unwrap_or(1500);
        let stationary = motor == 1500
            && (turn["crosswalk_state"] == "waiting" || turn["light_stop_state"] == "waiting");
        scans.push(Scan {
            t,
            heading,
            wall,
            stationary,
            powered: motor > 1500,
        });
    }
    let launch = scans
        .iter()
        .find(|s| s.powered)
        .ok_or("no drive in the supplied run")?
        .t;
    let mut windows = vec![(0.1, launch - 0.15)];
    let mut start = None;
    for s in &scans {
        if s.stationary {
            start.get_or_insert(s.t);
        } else if let Some(a) = start.take() {
            windows.push((a + 0.5, s.t - 0.3));
        }
    }
    let mut rest_a = [0.; 3];
    let mut rest_w = [0.; 3];
    let mut stationary_n = 0;
    for s in imu
        .iter()
        .filter(|s| windows.iter().any(|&(a, b)| a <= s.t && s.t <= b))
    {
        rest_a = add(rest_a, s.a);
        rest_w = add(rest_w, s.w);
        stationary_n += 1;
    }
    if stationary_n == 0 {
        return Err("no stationary IMU samples".into());
    }
    let mut yaw = Vec::new();
    let mut forward = Vec::new();
    for w in scans.windows(5) {
        let (a, b, c) = (&w[0], &w[2], &w[4]);
        let times = [a.t, b.t, c.t];
        if !(0.18..0.52).contains(&(b.t - a.t)) || !(0.18..0.52).contains(&(c.t - b.t)) {
            continue;
        }
        if let (Some(h0), Some(h1)) = (a.heading, c.heading) {
            let delta = (h1 - h0).sin().atan2((h1 - h0).cos());
            if delta.abs() > 0.055 {
                yaw.push(Observation {
                    t: times,
                    reference: delta,
                });
            }
        }
        if let (Some((s0, x0)), Some((s1, x1)), Some((s2, x2))) = (a.wall, b.wall, c.wall) {
            if s0 != s1
                || s0 != s2
                || !w
                    .iter()
                    .all(|s| s.wall.is_some_and(|(start, _)| start == s0))
            {
                continue;
            }
            let v0 = (x1 - x0) / (b.t - a.t);
            let v1 = (x2 - x1) / (c.t - b.t);
            if v0.max(v1) > 0.15 && (-0.15..3.5).contains(&v0) && (-0.15..3.5).contains(&v1) {
                forward.push(Observation {
                    t: times,
                    reference: v1 - v0,
                });
            }
        }
    }
    if yaw.is_empty() || forward.is_empty() {
        return Err("missing turns or straight-line excitation".into());
    }
    Ok(Run {
        name: dir
            .parent()
            .and_then(Path::file_name)
            .unwrap_or_default()
            .to_string_lossy()
            .into_owned(),
        imu,
        rest_a: mul(rest_a, 1. / stationary_n as f64),
        rest_w: mul(rest_w, 1. / stationary_n as f64),
        stationary_n,
        yaw,
        forward,
    })
}

struct Trace<'a> {
    run: &'a Run,
    nodes: Vec<Integral>,
}
impl<'a> Trace<'a> {
    fn new(run: &'a Run, rest_a: V3, rest_w: V3) -> Self {
        let mut n = Integral {
            t: run.imu[0].t,
            velocity: [0.; 3],
            displacement: [0.; 3],
            angle: [0.; 3],
        };
        let mut nodes = vec![n];
        for pair in run.imu.windows(2) {
            let (a, b) = (pair[0], pair[1]);
            let dt = b.t - a.t;
            if dt <= 0. {
                continue;
            }
            let a0 = sub(a.a, rest_a);
            let a1 = sub(b.a, rest_a);
            n.displacement = add(
                n.displacement,
                add(mul(n.velocity, dt), mul(add(mul(a0, 2.), a1), dt * dt / 6.)),
            );
            n.velocity = add(n.velocity, mul(add(a0, a1), dt / 2.));
            n.angle = add(n.angle, mul(sub(add(a.w, b.w), mul(rest_w, 2.)), dt / 2.));
            n.t = b.t;
            nodes.push(n);
        }
        Self { run, nodes }
    }
    fn at(&self, t: f64, yaw: bool) -> Option<V3> {
        let i = self.nodes.partition_point(|n| n.t < t);
        if i == 0 || i >= self.nodes.len() {
            return None;
        }
        let (a, b) = (self.nodes[i - 1], self.nodes[i]);
        let f = (t - a.t) / (b.t - a.t);
        let (x, y) = if yaw {
            (a.angle, b.angle)
        } else {
            (a.displacement, b.displacement)
        };
        Some(add(x, mul(sub(y, x), f)))
    }
    fn pairs(&self, delay: f64, yaw: bool) -> Vec<(V3, f64)> {
        let obs = if yaw {
            &self.run.yaw
        } else {
            &self.run.forward
        };
        obs.iter()
            .filter_map(|o| {
                let [t0, t1, t2] = o.t;
                let a = self.at(t0 + delay, yaw)?;
                let c = self.at(t2 + delay, yaw)?;
                let x = if yaw {
                    sub(c, a)
                } else {
                    // The change between two mean velocities cancels the unknown
                    // initial velocity. Integrating twice avoids differentiating noisy lidar twice.
                    let b = self.at(t1 + delay, false)?;
                    sub(
                        mul(sub(c, b), 1. / (t2 - t1)),
                        mul(sub(b, a), 1. / (t1 - t0)),
                    )
                };
                Some((x, o.reference))
            })
            .collect()
    }
}

#[derive(Clone, Copy)]
struct Fit {
    axis: V3,
    gain: f64,
    delay: f64,
    loss: f64,
}
fn solve(pairs: &[(V3, f64)], up: V3, yaw: bool, delay: f64) -> Fit {
    let width = if yaw { 0.04 } else { 0.12 }; // radians per turn window, m/s per speed-change window
    let mut weights = vec![1.; pairs.len()];
    let mut axis = up;
    let mut gain = 1.;
    for _ in 0..4 {
        if !yaw {
            let direction = pairs
                .iter()
                .zip(&weights)
                .fold([0.; 3], |v, ((x, y), w)| add(v, mul(*x, w * y)));
            axis = unit(sub(direction, mul(up, dot(direction, up))));
        }
        let (mut xy, mut xx) = (0., 0.);
        for ((x, y), w) in pairs.iter().zip(&weights) {
            let v = dot(*x, axis);
            xy += w * v * y;
            xx += w * v * v;
        }
        gain = xy / xx;
        for ((x, y), w) in pairs.iter().zip(&mut weights) {
            let error = (gain * dot(*x, axis) - y).abs();
            *w = width / error.max(width);
        }
    }
    let loss = pairs
        .iter()
        .map(|(x, y)| {
            let e = (gain * dot(*x, axis) - y).abs();
            if e <= width {
                e * e
            } else {
                width * (2. * e - width)
            }
        })
        .sum::<f64>()
        / pairs.len() as f64;
    Fit {
        axis,
        gain,
        delay,
        loss,
    }
}

fn fit_delay(traces: &[Trace], up: V3, yaw: bool) -> Fit {
    (-80..=80)
        .map(|tick| {
            let delay = tick as f64 * 0.005;
            let rows = traces
                .iter()
                .flat_map(|tr| tr.pairs(delay, yaw))
                .collect::<Vec<_>>();
            solve(&rows, up, yaw, delay)
        })
        .min_by(|a, b| a.loss.total_cmp(&b.loss))
        .unwrap()
}

fn errors(trace: &Trace, f: Fit, yaw: bool) -> Value {
    let pairs = trace.pairs(f.delay, yaw);
    let n = pairs.len() as f64;
    let mut e2 = 0.;
    let mut eabs = 0.;
    let mut reference2 = 0.;
    let mut sx = 0.;
    let mut sy = 0.;
    let mut sxx = 0.;
    let mut sxy = 0.;
    for (x, y) in &pairs {
        let pred = dot(*x, f.axis) * f.gain;
        let e = pred - y;
        e2 += e * e;
        eabs += e.abs();
        reference2 += y * y;
        sx += pred;
        sy += y;
        sxx += pred * pred;
        sxy += pred * y;
    }
    let scale = if yaw { 180. / std::f64::consts::PI } else { 1. };
    json!({"windows":pairs.len(),"rmse":(e2/n).sqrt()*scale,"mae":eabs/n*scale,
           "reference_rms":(reference2/n).sqrt()*scale,
           "correlation":(sxy-sx*sy/n)/((sxx-sx*sx/n)*(reference2-sy*sy/n)).sqrt(),
           "quantity":if yaw {"change of lidar heading, degrees"}else{"change in adjacent mean longitudinal speeds, m/s"}})
}

struct Calibration {
    rest_a: V3,
    bias_w: V3,
    yaw: Fit,
    forward: Fit,
}
impl Calibration {
    fn fit(runs: &[&Run]) -> Self {
        let n = runs.len() as f64;
        let rest_a = mul(runs.iter().fold([0.; 3], |s, r| add(s, r.rest_a)), 1. / n);
        let bias_w = mul(runs.iter().fold([0.; 3], |s, r| add(s, r.rest_w)), 1. / n);
        let traces = runs
            .iter()
            .map(|r| Trace::new(r, rest_a, bias_w))
            .collect::<Vec<_>>();
        let up = unit(rest_a);
        Self {
            rest_a,
            bias_w,
            yaw: fit_delay(&traces, up, true),
            forward: fit_delay(&traces, up, false),
        }
    }
    fn parameters(&self) -> Value {
        json!({"stationary_specific_force_mps2":self.rest_a,"gyro_bias_radps":self.bias_w,
            "body_axes_in_sensor":{"forward":self.forward.axis,"left":cross(self.yaw.axis,self.forward.axis),"up":self.yaw.axis},
            "yaw_rate_gain":self.yaw.gain,"forward_acceleration_gain":self.forward.gain,
            "yaw_imu_minus_lidar_receive_s":self.yaw.delay,"forward_imu_minus_lidar_receive_s":self.forward.delay})
    }
}

fn main() -> Result<()> {
    let dirs = std::env::args().skip(1).collect::<Vec<_>>();
    if dirs.len() < 2 {
        return Err(
            "pass at least two driven run/ directories to allow held-out validation".into(),
        );
    }
    let runs = dirs
        .iter()
        .map(|s| load(Path::new(s)))
        .collect::<Result<Vec<_>>>()?;
    let full = Calibration::fit(&runs.iter().collect::<Vec<_>>());
    let mut fits = Vec::new();
    for (i, r) in runs.iter().enumerate() {
        let training = runs
            .iter()
            .enumerate()
            .filter_map(|(j, r)| (j != i).then_some(r))
            .collect::<Vec<_>>();
        let cv = Calibration::fit(&training);
        let held = Trace::new(r, cv.rest_a, cv.bias_w);
        let gaps = r.imu.windows(2).map(|w| w[1].t - w[0].t).fold(0., f64::max);
        let fps = (r.imu.len() - 1) as f64 / (r.imu.last().unwrap().t - r.imu[0].t);
        fits.push(json!({"held_out":r.name,"imu_samples":r.imu.len(),"mean_rate_hz":fps,"max_sample_gap_s":gaps,
          "stationary_samples":r.stationary_n,"stationary_specific_force_mps2":r.rest_a,
          "stationary_gyro_radps":r.rest_w,"training_parameters":cv.parameters(),
          "yaw":errors(&held,cv.yaw,true),"forward":errors(&held,cv.forward,false)}));
    }
    let result = json!({"schema":1,"frame":"imu_link","method":"flat-floor planar calibration against lidar",
        "time_convention":"IMU query time = lidar scan receive time + fitted signed offset; these include scan/processing latency",
        "application":"yaw_rate = yaw_rate_gain * dot(gyro - gyro_bias, up); forward_accel = forward_acceleration_gain * dot(accel - stationary_specific_force, forward)",
        "observability":"rest acceleration includes gravity and bias; separate 3-D accelerometer biases and translational lever arm are not estimated",
        "parameters":full.parameters(),"leave_one_run_out":fits});
    println!("{}", serde_json::to_string_pretty(&result)?);
    Ok(())
}
