//! Dynamic arc-length preview of the S course built strictly from surveyed
//! field walls/dividers and the two supplied cone estimates. The locator
//! replaces a nominal centre with this run's measurements as they arrive.
use crate::helpers::pursuit_pwm;
use crate::maneuver::{OPEN_LOOKAHEAD_M, OPEN_PREDICT_SPEED_MPS};
use crate::race_first_cone::STEER_RESPONSE_S;
use std::cell::RefCell;
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

struct SidePath {
    samples: Vec<Sample>,
    bend_s: f64,
}

struct DynamicPath {
    sides: [Option<SidePath>; 2],
}

static WALL_ANCHORS: LazyLock<Option<[XY; 3]>> = LazyLock::new(super::race_field_map::path_anchors);

#[inline]
fn clamp_safe(v: f64, lo: f64, hi: f64) -> f64 {
    if lo <= hi {
        v.clamp(lo, hi)
    } else {
        v.clamp(hi, lo)
    }
}

/// Center a passage coordinate safely between a cone edge (`cone_coord + side_sign * 0.10`)
/// and a fixed board (`wall_coord`), keeping exact nominal offset when clearance is ample.
fn fit_between(desired: f64, wall_coord: f64, cone_coord: f64, side_sign: f64, wall_pref: f64) -> f64 {
    let cone_edge = cone_coord + side_sign * 0.10;
    let gap = (wall_coord - cone_edge) * side_sign;
    if gap <= 0.12 {
        return 0.5 * (wall_coord + cone_edge);
    }
    let wall_margin = wall_pref.min(gap * 0.48);
    let cone_margin = 0.44_f64.min(gap * 0.48);
    let lo_bound = cone_edge + side_sign * cone_margin;
    let hi_bound = wall_coord - side_sign * wall_margin;
    clamp_safe(desired, lo_bound, hi_bound)
}

fn sample_knots(knots: &[XY], bend_seg: usize, size: XY, lane: XY, light: XY) -> SidePath {
    let mut samples: Vec<Sample> = Vec::with_capacity((knots.len() - 1) * 18 + 1);
    let mut arc = 0.0;
    let mut bend_s = 0.0;
    // Monotone-damped Hermite tangent prevents Catmull-Rom sag into nearby dividers
    // when a shifted cone flattens two consecutive knots while leaving smooth arcs untouched.
    let hermite_tangent = |prev: f64, cur: f64, next: f64| -> f64 {
        let d0 = cur - prev;
        let d1 = next - cur;
        let raw = 0.5 * (next - prev);
        if d0 * d1 <= 0.0 {
            0.0
        } else {
            let cap = 2.0 * d0.abs().min(d1.abs());
            raw.clamp(-cap, cap)
        }
    };
    for i in 0..knots.len() - 1 {
        let (a, b, c, d) = (
            knots[i.saturating_sub(1)],
            knots[i],
            knots[i + 1],
            knots[(i + 2).min(knots.len() - 1)],
        );
        let ux = if i == 0 { 0.5 * (c.0 - a.0) } else { hermite_tangent(a.0, b.0, c.0) };
        let uy = if i == 0 { 0.5 * (c.1 - a.1) } else { hermite_tangent(a.1, b.1, c.1) };
        let vx = if i + 2 >= knots.len() { 0.5 * (d.0 - b.0) } else { hermite_tangent(b.0, c.0, d.0) };
        let vy = if i + 2 >= knots.len() { 0.5 * (d.1 - b.1) } else { hermite_tangent(b.1, c.1, d.1) };
        for k in 0..=18 {
            if i > 0 && k == 0 {
                continue;
            }
            let t = k as f64 / 18.0;
            let cubic = |b: f64, c: f64, u: f64, v: f64| {
                b + t * (u + t * ((3.0 * (c - b) - 2.0 * u - v) + t * (2.0 * (b - c) + u + v)))
            };
            let mut px = cubic(b.0, c.0, ux, vx).clamp(0.32, size.0 - 0.32);
            let mut py = cubic(b.1, c.1, uy, vy).clamp(0.32, size.1 - 0.32);
            // Keep open-course segments safely clear of start/finish divider boards.
            if px <= lane.0 + 0.12 && py > lane.1 && py < lane.1 + 0.24 {
                py = lane.1 + 0.24;
            }
            if px >= light.0 - 0.12 && py < light.1 && py > light.1 - 0.24 {
                py = light.1 - 0.24;
            }
            px = px.clamp(0.32, size.0 - 0.32);
            let p = (px, py);
            let delta = samples
                .last()
                .map_or((1.0, 0.0), |last| (p.0 - last.p.0, p.1 - last.p.1));
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
            if k == 18 && i == bend_seg {
                bend_s = arc;
            }
        }
    }
    SidePath { samples, bend_s }
}

fn build_side0(size: XY, lane: XY, light: XY, first: XY, second: Option<XY>) -> SidePath {
    let low = lane.1 * 0.5;
    // Right-side wall clearance around Cone 0 (between first.0 and size.0).
    let right_x = |dx: f64| fit_between(first.0 + dx, size.0, first.0, 1.0, 0.42);
    // Lower wall clearance (between y = 0 and first.1) and upper clearance (between first.1 and finish divider/top wall).
    let south_y = |dy: f64| fit_between(first.1 + dy, 0.0, first.1, -1.0, 0.38);
    let upper_wall = |x: f64| if x >= light.0 - 0.25 { light.1 } else { size.1 };
    let north_y = |x: f64, dy: f64| fit_between(first.1 + dy, upper_wall(x), first.1, 1.0, 0.36);

    let k2_x = clamp_safe(first.0 - 0.85, lane.0 + 0.10, (first.0 - 0.35).max(lane.0 + 0.10));
    let k3_x = right_x(0.42).min(first.0 + 0.55);
    let k3_y = (low + 0.22).min(south_y(-0.62)).max(0.38);
    let k4_x = right_x(0.98);
    let k4_y = south_y(-0.78);
    let k5_x = right_x(1.08);
    let k5_y = clamp_safe(first.1 - 0.08, 0.45, upper_wall(k5_x) - 0.42);
    let mut k6_x = right_x(0.86);
    let mut k6_y = north_y(k6_x, 0.54);
    let k7_x = (first.0 + 0.32).min(size.0 - 0.45);
    let mut k7_y = north_y(k7_x, 0.72);
    let k8_x = clamp_safe(first.0 - 0.42, lane.0 + 0.15, size.0 - 0.55);
    let mut k8_y = north_y(k8_x, 0.68);
    // When Cone 0 sits high (close to the finish divider), swing wide in the open east space
    // and enter the narrow north corridor already aligned west with balanced wall/cone margin.
    if first.1 > 2.10 {
        k6_x = right_x(1.02);
        if k6_x >= light.0 - 0.25 {
            k6_y = (first.1 + 0.26).min(light.1 - 0.44);
        }
        let target_y = (light.1 - 0.44).min(0.5 * (first.1 + 0.10 + light.1) - 0.04);
        if k7_x >= light.0 - 0.25 {
            k7_y = k7_y.min(target_y.max(first.1 + 0.33));
        }
        if k8_x >= light.0 - 0.25 {
            k8_y = k8_y.min(target_y.max(first.1 + 0.33));
        }
    }

    // Knot 9 leads cleanly back into the open transfer toward Cone 1 (if already known) or open centre.
    let k9 = if let Some(sec) = second {
        let mid_x = clamp_safe((first.0 + sec.0) * 0.5 + 0.18, 0.80, size.0 - 0.80);
        let lo_y = if mid_x <= lane.0 + 0.20 { lane.1 + 0.36 } else { 0.45 };
        let hi_y = if mid_x >= light.0 - 0.20 { light.1 - 0.36 } else { size.1 - 0.45 };
        let mid_y = clamp_safe((first.1 + sec.1) * 0.5 + 0.02, lo_y, hi_y);
        (mid_x, mid_y)
    } else {
        let tx = clamp_safe(first.0 - 1.45, lane.0 + 0.20, (first.0 - 0.80).max(lane.0 + 0.20));
        let ty = clamp_safe(first.1 + 0.25, lane.1 + 0.45, light.1 - 0.45);
        (tx, ty)
    };
    let k10 = if let Some(sec) = second {
        let x10 = clamp_safe(sec.0 + 0.65, 0.60, size.0 - 0.50);
        let wall_lo = if x10 <= lane.0 + 0.35 { lane.1 } else { 0.0 };
        let y10 = fit_between(sec.1 - 0.60, wall_lo, sec.1, -1.0, 0.34);
        (x10, y10)
    } else {
        let tx = (k9.0 - 0.90).max(lane.0 - 0.30);
        (tx, clamp_safe(k9.1, lane.1 + 0.45, size.1 - 0.60))
    };

    let knots = [
        (0.34, low),
        (lane.0 - 0.15, low),
        (k2_x, low + 0.04),
        (k3_x, k3_y),
        (k4_x, k4_y),
        (k5_x, k5_y),
        (k6_x, k6_y),
        (k7_x, k7_y),
        (k8_x, k8_y),
        k9,
        k10,
    ];
    sample_knots(&knots, 2, size, lane, light)
}

fn build_side1(size: XY, lane: XY, light: XY, first: Option<XY>, second: XY) -> SidePath {
    let high = (light.1 + size.1) * 0.5;
    let left_x = |dx: f64| fit_between(second.0 + dx, 0.0, second.0, -1.0, 0.42);
    let south_wall = |x: f64| if x <= lane.0 + 0.35 { lane.1 } else { 0.0 };
    let south_y = |x: f64, dy: f64| fit_between(second.1 + dy, south_wall(x), second.1, -1.0, 0.34);
    let north_y = |dy: f64| fit_between(second.1 + dy, size.1, second.1, 1.0, 0.42);

    // Entry anchor from Cone 0 (if known) or transfer corridor.
    let (k0, k1) = if let Some(f) = first {
        let upper_wall = if f.0 - 0.42 >= light.0 - 0.25 { light.1 } else { size.1 };
        let f8 = (
            clamp_safe(f.0 - 0.42, lane.0 + 0.15, size.0 - 0.55),
            fit_between(f.1 + 0.68, upper_wall, f.1, 1.0, 0.36),
        );
        let mid_x = clamp_safe((f.0 + second.0) * 0.5 + 0.18, 0.80, size.0 - 0.80);
        let lo_y = if mid_x <= lane.0 + 0.20 { lane.1 + 0.36 } else { 0.45 };
        let hi_y = if mid_x >= light.0 - 0.20 { light.1 - 0.36 } else { size.1 - 0.45 };
        let mid_y = clamp_safe((f.1 + second.1) * 0.5 + 0.02, lo_y, hi_y);
        (f8, (mid_x, mid_y))
    } else {
        let e1_x = (second.0 + 1.85).min(size.0 - 0.60);
        let e0_x = (second.0 + 2.85).min(size.0 - 0.50);
        let ey = clamp_safe(second.1, lane.1 + 0.45, light.1 - 0.45);
        ((e0_x, ey), (e1_x, ey))
    };

    let k2_x = clamp_safe(second.0 + 0.65, 0.60, size.0 - 0.55);
    let k2_y = south_y(k2_x, -0.60);
    let k3_x = clamp_safe(second.0 - 0.48, 0.45, size.0 - 0.65);
    let k3_y = south_y(k3_x, -0.68);
    let k4_x = left_x(-1.08);
    let k4_y = clamp_safe(second.1 - 0.28, south_wall(k4_x) + 0.34, size.1 - 0.45);
    let mut k5_x = left_x(-1.18);
    let mut k5_y = clamp_safe(second.1 + 0.30, south_wall(k5_x) + 0.38, size.1 - 0.42);
    let mut k6_x = left_x(-0.85);
    let mut k6_y = north_y(1.00).min(high + 0.05);
    let k7_x = clamp_safe(second.0 - 0.20, 0.55, light.0 - 0.65);
    let mut k7_y = (high - 0.08).max(north_y(0.52).min(high));
    let mut k8_y = high;
    if second.1 > 2.15 {
        // Begin curving right lower on high-Y Cone 1 so the CW exit levels out near `high`
        // without overshooting into the y = size.1 wall or beam post.
        k5_x = left_x(-0.96);
        k5_y = k5_y.min(second.1 - 0.04).min(high - 0.96);
        k6_x = left_x(-0.52);
        k6_y = k6_y.min(high - 0.62).max(second.1 + 0.38);
        k7_y = k7_y.min(high - 0.42).max(second.1 + 0.38);
        k8_y = high - 0.10;
    }
    let k8_x = clamp_safe(second.0 + 0.68, (k7_x + 0.35).min(light.0 - 0.15), light.0 - 0.15);

    let knots = [
        k0,
        k1,
        (k2_x, k2_y),
        (k3_x, k3_y),
        (k4_x, k4_y),
        (k5_x, k5_y),
        (k6_x, k6_y),
        (k7_x, k7_y),
        (k8_x, k8_y),
        (light.0, high),
        (size.0 - 0.45, high),
    ];
    // Segment 2 ends at k3 (`second.0 - 0.48, second.1 - 0.68`), matching original segment 10.
    sample_knots(&knots, 2, size, lane, light)
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct ConePositions {
    pub centers: [Option<Point2>; 2],
    pub observed: [bool; 2],
}

impl From<[Option<Point2>; 2]> for ConePositions {
    fn from(centers: [Option<Point2>; 2]) -> Self {
        Self { observed: centers.map(|p| p.is_some()), centers }
    }
}

fn build_dynamic_path(cones: impl Into<ConePositions>) -> Option<DynamicPath> {
    let cones = cones.into();
    let [size, lane, light] = *WALL_ANCHORS.as_ref()?;
    let [c0, c1] = cones.centers.map(|p| p.map(|p| (p.x_m, p.y_m)));
    // Once this barrel is measured, the other's unverified nominal position
    // should not pull its entry/exit spline away from the measured turn.
    let side0 = c0.map(|first| build_side0(size, lane, light, first, c1.filter(|_| !cones.observed[0] || cones.observed[1])));
    let side1 = c1.map(|second| build_side1(size, lane, light, c0.filter(|_| !cones.observed[1] || cones.observed[0]), second));
    Some(DynamicPath {
        sides: [side0, side1],
    })
}

#[derive(Clone, Copy, Debug)]
pub struct Aim {
    pub point: XY,
    pub pwm: i64,
    pub curvature: f64,
    pub before: f64,
    pub lookahead: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct Input {
    seq: i64,
    pose: Pose2,
    speed: f64,
    yaw: f64,
    cones: ConePositions,
}

#[derive(Clone, Copy, Debug)]
struct Plans {
    input: Input,
    aims: [Option<Aim>; 2],
}

#[derive(Default)]
struct PathCache {
    key: Option<ConePositions>,
    path: Option<DynamicPath>,
}

impl PathCache {
    fn get_or_build(&mut self, cones: ConePositions) -> Option<&DynamicPath> {
        if self.key != Some(cones) {
            self.path = build_dynamic_path(cones);
            self.key = Some(cones);
        }
        self.path.as_ref()
    }
}

fn calculate_with_cache(input: Input, cache: &mut PathCache) -> Option<Plans> {
    let path = cache.get_or_build(input.cones)?;
    let speed = input.speed.max(0.0);
    let dt = STEER_RESPONSE_S;
    let angle = input.pose.yaw_rad + 0.5 * dt * input.yaw;
    let (s, c) = angle.sin_cos();
    let pose = Pose2 {
        x_m: input.pose.x_m + speed * dt * c,
        y_m: input.pose.y_m + speed * dt * s,
        yaw_rad: input.pose.yaw_rad + dt * input.yaw,
    };
    let (sy, cx) = pose.yaw_rad.sin_cos();
    let lookahead = OPEN_LOOKAHEAD_M * (speed / OPEN_PREDICT_SPEED_MPS).clamp(1.0, 1.65);

    let aim = |side: usize| -> Option<Aim> {
        let sp = path.sides[side].as_ref()?;
        let cost = |q: &Sample| {
            (pose.x_m - q.p.0).powi(2)
                + (pose.y_m - q.p.1).powi(2)
                + 0.40 * (1.0 - (q.tangent.0 * cx + q.tangent.1 * sy))
        };
        let closest = sp
            .samples
            .iter()
            .filter(|q| if side == 0 { q.segment <= 9 } else { true })
            .min_by(|a, b| cost(a).total_cmp(&cost(b)))?;
        let ahead = closest.s + lookahead;
        let n = sp
            .samples
            .partition_point(|p| p.s < ahead)
            .min(sp.samples.len() - 1);
        let (a, b) = (&sp.samples[n.saturating_sub(1)], &sp.samples[n]);
        let u = ((ahead - a.s) / (b.s - a.s).max(1e-9)).clamp(0.0, 1.0);
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
            before: (sp.bend_s - closest.s).max(0.0),
            lookahead,
        })
    };

    Some(Plans {
        input,
        aims: [aim(0), aim(1)],
    })
}

#[derive(Clone, Debug)]
struct Worker {
    send: mpsc::SyncSender<Input>,
    result: Arc<Mutex<Option<Plans>>>,
}

#[derive(Default)]
pub struct Planner {
    worker: Option<Worker>,
    input: Option<Input>,
    local: RefCell<(Option<Plans>, PathCache)>,
}

impl Clone for Planner {
    fn clone(&self) -> Self {
        Self {
            worker: self.worker.clone(),
            input: self.input,
            local: RefCell::new((self.local.borrow().0, PathCache::default())),
        }
    }
}

impl std::fmt::Debug for Planner {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Planner")
            .field("has_worker", &self.worker.is_some())
            .field("input", &self.input)
            .finish()
    }
}

impl Planner {
    pub fn start(&mut self) {
        LazyLock::force(&WALL_ANCHORS);
        let (send, recv) = mpsc::sync_channel(1);
        let result = Arc::new(Mutex::new(None));
        let output = Arc::clone(&result);
        if std::thread::Builder::new()
            .name("field-path".into())
            .spawn(move || {
                let mut cache = PathCache::default();
                while let Ok(input) = recv.recv() {
                    let plan = calculate_with_cache(input, &mut cache);
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

    pub fn observe(
        &mut self,
        seq: i64,
        pose: Pose2,
        speed: f64,
        yaw: f64,
        cones: impl Into<ConePositions>,
    ) {
        let input = Input {
            seq,
            pose,
            speed,
            yaw,
            cones: cones.into(),
        };
        self.input = Some(input);
        let mut guard = self.local.borrow_mut();
        guard.0 = None;
        if let Some(worker) = &self.worker {
            let _ = worker.send.try_send(input);
        } else {
            guard.0 = calculate_with_cache(input, &mut guard.1);
        }
    }

    pub fn aim(&self, seq: i64, side: usize) -> Option<Aim> {
        let input = self.input.filter(|i| i.seq == seq)?;
        let mut guard = self.local.borrow_mut();
        if guard.0.is_none() {
            let from_worker = self
                .worker
                .as_ref()
                .and_then(|w| w.result.try_lock().ok().and_then(|r| *r))
                .filter(|p| p.input == input);
            guard.0 = from_worker.or_else(|| calculate_with_cache(input, &mut guard.1));
        }
        guard.0?.aims[side]
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
        let cones = [
            Some(Point2 { x_m: 5.8, y_m: 2.0 }),
            Some(Point2 { x_m: 2.2, y_m: 2.0 }),
        ];
        for seq in 1..=24 {
            let pose = Pose2 {
                x_m: 4.1 + seq as f64 * 0.09,
                y_m: 0.48,
                yaw_rad: 0.06,
            };
            for p in [&mut parallel, &mut inline] {
                p.observe(seq, pose, 2.0, 0.3, cones);
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

    #[test]
    fn dynamic_planner_has_no_static_fallback_and_adapts_to_shifted_cones() {
        let mut planner = Planner::default();
        let pose = Pose2 {
            x_m: 3.8,
            y_m: 0.48,
            yaw_rad: 0.02,
        };
        // No cones observed yet -> neither side produces an aim.
        planner.observe(1, pose, 1.5, 0.0, [None, None]);
        assert!(planner.aim(1, 0).is_none());
        assert!(planner.aim(1, 1).is_none());

        // Only Cone 0 observed (shifted >0.7 m from old nominal) -> Side 0 ready, Side 1 None.
        let c0 = Point2 { x_m: 6.35, y_m: 2.45 };
        planner.observe(2, pose, 1.5, 0.0, [Some(c0), None]);
        assert!(planner.aim(2, 0).is_some());
        assert!(planner.aim(2, 1).is_none());

        // Both cones observed in tight shifted positions -> all spline samples stay clear of walls/dividers & cones.
        let c1 = Point2 { x_m: 1.65, y_m: 1.50 };
        let dp = build_dynamic_path([Some(c0), Some(c1)]).unwrap();
        let field = crate::race_field_map::FIELD.as_ref().unwrap();
        for (side, cone) in [(0, c0), (1, c1)] {
            let sp = dp.sides[side].as_ref().unwrap();
            for s in &sp.samples {
                let pt = Point2 { x_m: s.p.0, y_m: s.p.1 };
                assert!(pt.x_m > 0.25 && pt.x_m < 7.75, "side {side} x out: {:?}", s.p);
                assert!(pt.y_m > 0.25 && pt.y_m < 3.75, "side {side} y out: {:?}", s.p);
                let wall_d = crate::race_field_map::dist_to_any_wall(field, pt);
                assert!(wall_d >= 0.20, "side {side} too close to wall ({wall_d:.3}m) at {:?}", s.p);
                let cone_d = pt.distance(cone);
                assert!(cone_d >= 0.30, "side {side} too close to cone ({cone_d:.3}m) at {:?}", s.p);
            }
        }
    }

    #[test]
    fn dynamic_planner_never_panics_for_any_cone_coordinates_in_field() {
        for ix in 0..=14 {
            let x = 0.5 + ix as f64 * 0.5;
            for iy in 0..=6 {
                let y = 0.5 + iy as f64 * 0.5;
                let p = Point2 { x_m: x, y_m: y };
                assert!(build_dynamic_path([Some(p), None]).is_some());
                assert!(build_dynamic_path([None, Some(p)]).is_some());
                assert!(build_dynamic_path([Some(p), Some(Point2 { x_m: 8.0 - x, y_m: 4.0 - y })]).is_some());
            }
        }
    }
}

#[cfg(test)]
mod independent_snapshot_review {
    use super::*;

    #[test]
    fn same_scan_new_cone_is_available_before_worker_replies() {
        let (send, queued) = mpsc::sync_channel(1);
        let result = Arc::new(Mutex::new(None));
        let mut planner = Planner::default();
        planner.worker = Some(Worker { send, result: result.clone() });
        let pose = Pose2 { x_m: 4.8, y_m: 2.0, yaw_rad: 3.05 };
        let first = Some(Point2 { x_m: 5.15, y_m: 1.4 });
        let second = Some(Point2 { x_m: 2.85, y_m: 2.55 });
        planner.observe(70, pose, 1.2, 0.0, [first, None]);
        let initial = queued.recv().unwrap();
        *result.lock().unwrap() = calculate_with_cache(initial, &mut PathCache::default());
        // The new radar observation arrives later during the very same revolution.
        planner.observe(70, pose, 1.2, 0.0, [first, second]);
        assert!(planner.aim(70, 1).is_some(), "latest radar cone missing from same-scan preview");
    }

    #[test]
    fn equal_inputs_give_the_same_aim_across_cache_histories() {
        let pose = Pose2 { x_m: 5.98, y_m: 0.72, yaw_rad: 0.35 };
        let c = |x| Some(Point2 { x_m: x, y_m: 1.4 });
        let first = Input { seq: 70, pose, speed: 1.6, yaw: 0.15, cones:[c(5.150), None].into() };
        let next = Input { seq: 71, cones:[c(5.153), None].into(), ..first };
        let mut warmed = PathCache::default();
        let _ = calculate_with_cache(first, &mut warmed);
        let warm = calculate_with_cache(next, &mut warmed).unwrap().aims[0].unwrap();
        let cold = calculate_with_cache(next, &mut PathCache::default()).unwrap().aims[0].unwrap();
        assert_eq!(warm.point, cold.point, "identical latest observation and pose must produce identical plan");
    }
}
