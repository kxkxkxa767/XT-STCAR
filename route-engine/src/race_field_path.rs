//! Arc-length preview of the surveyed S course. The two loop candidates are
//! prepared on every localized revolution, independently of live cone tracking.
use crate::helpers::pursuit_pwm;
use crate::maneuver::{OPEN_LOOKAHEAD_M, OPEN_PREDICT_SPEED_MPS};
use crate::race_first_cone::STEER_RESPONSE_S;
use std::cell::Cell;
use std::sync::{Arc, LazyLock, Mutex, mpsc};
use xt_stcar_robot_core::autonomy::{Point2, Pose2};

type XY = (f64, f64);
#[derive(Clone, Copy)]
struct Sample {
    p: XY,
    s: f64,
    tangent: XY,
    segment: usize,
}
struct Path {
    samples: Vec<Sample>,
    bend_s: [f64; 2],
}

static PATH: LazyLock<Option<Path>> = LazyLock::new(|| {
    let [size, lane, light, first, second] = super::race_field_map::path_anchors()?;
    let low = lane.1 * 0.5;
    let high = (light.1 + size.1) * 0.5;
    // Offsets from the two surveyed cone centres describe the outer/inner
    // passages in the verified drawing. The straight lanes follow their boards.
    let knots = [
        (0.34, low),
        (lane.0 - 0.15, low),
        (first.0 - 0.8, low - 0.06),
        (first.0 + 0.6, low + 0.12),
        (first.0 + 1.25, first.1 - 1.0),
        (first.0 + 1.48, first.1 - 0.15),
        (first.0 + 1.19, first.1 + 0.55),
        (first.0 + 0.65, first.1 + 0.79),
        (first.0 - 0.36, first.1 + 0.79),
        ((first.0 + second.0) * 0.5 + 0.25, (first.1 + second.1) * 0.5 + 0.03),
        (second.0 + 0.7, second.1 - 0.50),
        (second.0 - 0.55, second.1 - 0.61),
        (second.0 - 1.25, second.1 - 0.30),
        (second.0 - 1.49, second.1 + 0.31),
        (second.0 - 1.06, second.1 + 1.08),
        (second.0 - 0.31, high - 0.12),
        (second.0 + 0.65, high),
        (light.0, high),
        (size.0 - 0.45, high),
    ];
    let mut samples: Vec<Sample> = Vec::new();
    let mut arc = 0.;
    let mut bend_s = [0.; 2];
    for i in 0..knots.len() - 1 {
        let (a, b, c, d) = (
            knots[i.saturating_sub(1)],
            knots[i],
            knots[i + 1],
            knots[(i + 2).min(knots.len() - 1)],
        );
        for k in 0..=18 {
            if i > 0 && k == 0 {
                continue;
            }
            let t = k as f64 / 18.;
            let cubic = |a: f64, b: f64, c: f64, d: f64| {
                let u = 0.5 * (c - a);
                let v = 0.5 * (d - b);
                b + t * (u + t * ((3. * (c - b) - 2. * u - v) + t * (2. * (b - c) + u + v)))
            };
            let p = (cubic(a.0, b.0, c.0, d.0), cubic(a.1, b.1, c.1, d.1));
            let delta = samples.last().map_or((1., 0.), |last| (p.0 - last.p.0, p.1 - last.p.1));
            let step = delta.0.hypot(delta.1).max(1e-9);
            if !samples.is_empty() {
                arc += step;
            }
            samples.push(Sample {
                p,
                s: arc,
                tangent: (delta.0 / step, delta.1 / step),
                segment: i,
            });
            if k == 18 && (i == 2 || i == 10) {
                bend_s[usize::from(i == 10)] = arc;
            }
        }
    }
    Some(Path { samples, bend_s })
});

#[derive(Clone, Copy, Debug)]
pub struct Aim {
    pub point: XY,
    pub pwm: i64,
    pub curvature: f64,
    pub before: f64,
    pub lookahead: f64,
}
#[derive(Clone, Copy, Debug)]
struct Input {
    seq: i64,
    pose: Pose2,
    speed: f64,
    yaw: f64,
}
#[derive(Clone, Copy, Debug)]
struct Plans {
    seq: i64,
    aims: [Aim; 2],
}

fn calculate(input: Input) -> Option<Plans> {
    let path = PATH.as_ref()?;
    let speed = input.speed.max(0.);
    let dt = STEER_RESPONSE_S;
    let angle = input.pose.yaw_rad + 0.5 * dt * input.yaw;
    let (s, c) = angle.sin_cos();
    let pose = Pose2 {
        x_m: input.pose.x_m + speed * dt * c,
        y_m: input.pose.y_m + speed * dt * s,
        yaw_rad: input.pose.yaw_rad + dt * input.yaw,
    };
    let (sy, cx) = pose.yaw_rad.sin_cos();
    let lookahead = OPEN_LOOKAHEAD_M * (speed / OPEN_PREDICT_SPEED_MPS).clamp(1., 1.65);
    let aim = |side: usize| {
        let cost = |q: &Sample| {
            (pose.x_m - q.p.0).powi(2)
                + (pose.y_m - q.p.1).powi(2)
                + 0.40 * (1. - (q.tangent.0 * cx + q.tangent.1 * sy))
        };
        let closest = path
            .samples
            .iter()
            .filter(|q| if side == 0 { q.segment <= 9 } else { q.segment >= 8 })
            .min_by(|a, b| cost(a).total_cmp(&cost(b)))?;
        let ahead = closest.s + lookahead;
        let n = path
            .samples
            .partition_point(|p| p.s < ahead)
            .min(path.samples.len() - 1);
        let (a, b) = (&path.samples[n.saturating_sub(1)], &path.samples[n]);
        let u = ((ahead - a.s) / (b.s - a.s).max(1e-9)).clamp(0., 1.);
        let target = Point2 {
            x_m: a.p.0 + (b.p.0 - a.p.0) * u,
            y_m: a.p.1 + (b.p.1 - a.p.1) * u,
        };
        let future = pose.world_to_body(target);
        let current = input.pose.world_to_body(target);
        let point = (current.x_m, current.y_m);
        let (curvature, pwm) = pursuit_pwm((future.x_m, future.y_m));
        Some(Aim {
            point,
            pwm,
            curvature,
            before: (path.bend_s[side] - closest.s).max(0.),
            lookahead,
        })
    };
    Some(Plans {
        seq: input.seq,
        aims: [aim(0)?, aim(1)?],
    })
}

#[derive(Clone, Debug)]
struct Worker {
    send: mpsc::SyncSender<Input>,
    result: Arc<Mutex<Option<Plans>>>,
}
#[derive(Clone, Debug, Default)]
pub struct Planner {
    worker: Option<Worker>,
    input: Option<Input>,
    local: Cell<Option<Plans>>,
}
impl Planner {
    pub fn start(&mut self) {
        LazyLock::force(&PATH);
        let (send, recv) = mpsc::sync_channel(1);
        let result = Arc::new(Mutex::new(None));
        let output = Arc::clone(&result);
        if std::thread::Builder::new()
            .name("field-path".into())
            .spawn(move || {
                while let Ok(input) = recv.recv() {
                    let plan = calculate(input);
                    if let Ok(mut slot) = output.lock() {
                        *slot = plan;
                    }
                }
            })
            .is_ok()
        {
            self.worker = Some(Worker { send, result });
        }
    }
    pub fn observe(&mut self, seq: i64, pose: Pose2, speed: f64, yaw: f64) {
        let input = Input { seq, pose, speed, yaw };
        self.input = Some(input);
        self.local.set(None);
        if let Some(worker) = &self.worker {
            let _ = worker.send.try_send(input);
        } else {
            self.local.set(calculate(input));
        }
    }
    pub fn aim(&self, seq: i64, side: usize) -> Option<Aim> {
        let input = self.input.filter(|i| i.seq == seq)?;
        let plans = self
            .local
            .get()
            .or_else(|| self.worker.as_ref()?.result.try_lock().ok()?.filter(|p| p.seq == seq))
            .or_else(|| calculate(input))?;
        self.local.set(Some(plans));
        Some(plans.aims[side])
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn latest_parallel_preview_equals_inline_without_waiting_for_the_previous_scan() {
        let mut parallel = Planner::default();
        let mut inline = Planner::default();
        parallel.start();
        for seq in 1..=24 {
            let pose = Pose2 {
                x_m: 4.1 + seq as f64 * 0.09,
                y_m: 0.48,
                yaw_rad: 0.06,
            };
            for p in [&mut parallel, &mut inline] {
                p.observe(seq, pose, 2., 0.3);
            }
            for side in [0, 1] {
                let current = inline.aim(seq, side).unwrap();
                let concurrent = parallel.aim(seq, side).unwrap();
                assert_eq!(current.pwm, concurrent.pwm);
                assert_eq!(current.point, concurrent.point);
                assert!(parallel.aim(seq - 1, side).is_none());
            }
        }
    }
}
