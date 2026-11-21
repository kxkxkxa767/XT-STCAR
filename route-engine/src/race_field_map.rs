//! Race-course localization. Axes are forward from the start and left toward
//! the finish lane. All poses describe the lidar spindle, in metres and radians.
use std::collections::{BTreeMap, VecDeque};
use std::sync::LazyLock;

use crate::compact_target::{TABLES, ranges_info};
use crate::helpers::SCAN_AGE_S;
use crate::motion_sensors::RaceInputs;
use crate::py::V;
use xt_stcar_robot_core::autonomy::{Point2, Pose2};

type XY = (f64, f64);
const BRIDGE_S: f64 = 0.65; // lifetime of a coordinate supported only by inertial motion
const MATCH_M: f64 = 0.25;

struct Board {
    vertical: bool,
    fixed: f64,
    lo: f64,
    hi: f64,
}
struct Field {
    walls: [Board; 6],
    start: [f64; 2],
    cones: [Point2; 2],
    beam_spacing: f64,
}
static FIELD: LazyLock<Option<Field>> = LazyLock::new(|| {
    let m: BTreeMap<String, [f64; 2]> = serde_json::from_str(include_str!("../../config/race-field-map.json")).ok()?;
    let [length, width] = *m.get("field_size_m")?;
    let [a, lower] = *m.get("start_divider_tip_m")?;
    let [b, upper] = *m.get("finish_divider_tip_m")?;
    let [beam, gantry] = *m.get("beam_and_gantry_x_m")?;
    let board = |vertical, fixed, lo, hi| Board {
        vertical,
        fixed,
        lo,
        hi,
    };
    let point = |[x_m, y_m]: [f64; 2]| Point2 { x_m, y_m };
    Some(Field {
        walls: [
            board(true, 0., 0., width),
            board(true, length, 0., width),
            board(false, 0., 0., length),
            board(false, width, 0., length),
            board(false, lower, 0., a),
            board(false, upper, b, length),
        ],
        start: *m.get("start_zone_upper_right_m")?,
        cones: [point(*m.get("first_cone_m")?), point(*m.get("second_cone_m")?)],
        beam_spacing: gantry - beam,
    })
});

pub fn beam_spacing() -> f64 {
    FIELD
        .as_ref()
        .map_or(crate::maneuver::LIGHT_SENSOR_TO_GANTRY_M, |f| f.beam_spacing)
}

pub fn start_width() -> f64 {
    FIELD.as_ref().map_or(0.90, |f| f.start[1])
}

pub(super) fn path_anchors() -> Option<[XY; 5]> {
    let f = FIELD.as_ref()?;
    Some([
        (f.walls[1].fixed, f.walls[3].fixed),
        (f.walls[4].hi, f.walls[4].fixed),
        (f.walls[5].lo, f.walls[5].fixed),
        (f.cones[0].x_m, f.cones[0].y_m),
        (f.cones[1].x_m, f.cones[1].y_m),
    ])
}

impl Field {
    // The first intersected finite board wins, including the two dividers.
    fn hit(&self, x: f64, y: f64, dx: f64, dy: f64) -> Option<(usize, f64)> {
        let mut nearest = f64::INFINITY;
        let mut face = None;
        for (i, board) in self.walls.iter().enumerate() {
            let (q, u, other, v) = if board.vertical { (x, dx, y, dy) } else { (y, dy, x, dx) };
            if u.abs() < 1e-6 {
                continue;
            }
            let r = (board.fixed - q) / u;
            let along = other + r * v;
            if r > 0.01 && r < nearest && along >= board.lo - 0.035 && along <= board.hi + 0.035 {
                nearest = r;
                face = Some(i);
            }
        }
        face.map(|i| (i, nearest))
    }
}

#[derive(Clone, Copy, Default)]
struct FaceFit {
    count: usize,
    weight: f64,
    lever: f64,
    lever2: f64,
    residual: f64,
    moment: f64,
    absolute: f64,
}

#[derive(Clone, Copy)]
struct Ray {
    range: f64,
    dx: f64,
    dy: f64,
    ox: f64,
    oy: f64,
}

/// A short fragment of a known straight board still observes rotation. Fit
/// its direction separately from translation so a large IMU prior cannot
/// hide a yaw jump when there are only three or four actual returns.
fn fragment_heading(field: &Field, seed: Pose2, beams: &[Ray]) -> Option<f64> {
    #[derive(Clone, Copy, Default)]
    struct Fragment {
        n: usize,
        x: f64,
        y: f64,
        xx: f64,
        xy: f64,
        yy: f64,
        lo: f64,
        hi: f64,
    }
    let mut faces = [Fragment::default(); 6];
    let (s, c) = seed.yaw_rad.sin_cos();
    for r in beams {
        let (dx, dy) = (c * r.dx - s * r.dy, s * r.dx + c * r.dy);
        let ox = seed.x_m + c * r.ox - s * r.oy;
        let oy = seed.y_m + s * r.ox + c * r.oy;
        let Some((i, range)) = field.hit(ox, oy, dx, dy) else {
            continue;
        };
        let vertical = field.walls[i].vertical;
        let norm = if vertical { dx } else { dy };
        if norm.abs() < 0.19
            || ((r.range - range) * norm).abs() > MATCH_M
            || (r.range - range).abs() > 0.48_f64.max(range * 0.18)
        {
            continue;
        }
        let along = if vertical { oy + r.range * dy } else { ox + r.range * dx };
        let (x, y) = (r.ox + r.range * r.dx, r.oy + r.range * r.dy);
        let f = &mut faces[i];
        if f.n == 0 {
            f.lo = along;
            f.hi = along;
        }
        f.lo = f.lo.min(along);
        f.hi = f.hi.max(along);
        f.n += 1;
        f.x += x;
        f.y += y;
        f.xx += x * x;
        f.xy += x * y;
        f.yy += y * y;
    }
    let mut sum = 0.;
    let mut weight = 0.;
    for (i, f) in faces.iter().enumerate().filter(|(_, f)| f.n >= 3 && f.hi - f.lo >= 0.2) {
        let n = f.n as f64;
        let (xx, yy, xy) = (f.xx - f.x * f.x / n, f.yy - f.y * f.y / n, f.xy - f.x * f.y / n);
        let angle = 0.5 * (2. * xy).atan2(xx - yy);
        let minor = 0.5 * ((xx + yy) - (xx - yy).hypot(2. * xy));
        if minor.max(0.) / n > 0.03_f64.powi(2) {
            continue;
        }
        let axis = if field.walls[i].vertical {
            std::f64::consts::FRAC_PI_2
        } else {
            0.
        };
        let delta = 0.5 * crate::py::wrap(2. * (axis - angle - seed.yaw_rad));
        let w = (xx + yy).min(2.);
        sum += w * delta;
        weight += w;
    }
    (weight > 0.).then(|| seed.yaw_rad + sum / weight)
}

fn rays(scan: &V, ranges: &[Option<f64>], speed: f64, yaw_rate: f64) -> Vec<Ray> {
    let ages = scan.get("point_age_ms").ol();
    TABLES.with(|table| {
        ranges
            .iter()
            .enumerate()
            .filter_map(|(i, r)| {
                let r = (*r)?;
                let age = ages
                    .and_then(|v| v.get(i))
                    .and_then(V::of)
                    .unwrap_or(0.)
                    .clamp(0., 150.)
                    * 0.001;
                let (s, c) = (-age * yaw_rate).sin_cos();
                let (x, y) = table.directions[i];
                Some(Ray {
                    range: r,
                    dx: c * x - s * y,
                    dy: s * x + c * y,
                    ox: -speed * age * (1. + c) * 0.5,
                    oy: -speed * age * s * 0.5,
                })
            })
            .collect()
    })
}

fn register(field: &Field, prior: Pose2, rays: &[Ray], imu_yaw: bool) -> (Pose2, [usize; 2], f64) {
    let mut pose = prior;
    let mut support = [0; 2];
    let mut mean_error = 0.;
    for _ in 0..4 {
        let (s, c) = pose.yaw_rad.sin_cos();
        let mut faces = [FaceFit::default(); 6];
        for ray in rays {
            let dx = c * ray.dx - s * ray.dy;
            let dy = s * ray.dx + c * ray.dy;
            let ox = c * ray.ox - s * ray.oy;
            let oy = s * ray.ox + c * ray.oy;
            let Some((id, expected)) = field.hit(pose.x_m + ox, pose.y_m + oy, dx, dy) else {
                continue;
            };
            let (normal, lever) = if field.walls[id].vertical {
                (dx, -oy - ray.range * dy)
            } else {
                (dy, ox + ray.range * dx)
            };
            let difference = ray.range - expected;
            let e = difference * normal;
            if normal.abs() < 0.19 || e.abs() > MATCH_M || difference.abs() > 0.48_f64.max(expected * 0.18) {
                continue;
            }
            let w = 0.047 / e.abs().max(0.047);
            let f = &mut faces[id];
            f.count += 1;
            f.weight += w;
            f.lever += w * lever;
            f.lever2 += w * lever * lever;
            f.residual += w * e;
            f.moment += w * lever * e;
            f.absolute += e.abs();
        }
        let mut diagonal = [2.; 2];
        let mut cross = [0.; 2];
        let mut rhs = [2. * (prior.x_m - pose.x_m), 2. * (prior.y_m - pose.y_m)];
        let mut angular = if imu_yaw { 18. } else { 4. };
        let mut angular_rhs = angular * (prior.yaw_rad - pose.yaw_rad);
        support = [0; 2];
        let mut sum_abs = 0.;
        for (i, f) in faces.iter().enumerate().filter(|(_, f)| f.count >= 3) {
            let axis = usize::from(!field.walls[i].vertical);
            let scale = 26. / (f.count as f64).max(26.);
            diagonal[axis] += scale * f.weight;
            cross[axis] += scale * f.lever;
            rhs[axis] -= scale * f.residual;
            angular += scale * f.lever2;
            angular_rhs -= scale * f.moment;
            support[axis] += f.count;
            sum_abs += f.absolute;
        }
        let samples = support[0] + support[1];
        if samples == 0 {
            break;
        }
        mean_error = sum_abs / samples as f64;
        let dh = ((angular_rhs - cross[0] * rhs[0] / diagonal[0] - cross[1] * rhs[1] / diagonal[1])
            / (angular - cross[0] * cross[0] / diagonal[0] - cross[1] * cross[1] / diagonal[1]))
            .clamp(-0.075, 0.075);
        let dx = ((rhs[0] - cross[0] * dh) / diagonal[0]).clamp(-0.18, 0.18);
        let dy = ((rhs[1] - cross[1] * dh) / diagonal[1]).clamp(-0.18, 0.18);
        pose.x_m += dx;
        pose.y_m += dy;
        pose.yaw_rad += dh;
        if dx.abs().max(dy.abs()).max(dh.abs()) < 0.0005 {
            break;
        }
    }
    (pose, support, mean_error)
}

#[derive(Clone, Copy, Debug)]
struct Fix {
    seq: i64,
    at: f64,
    pose: Pose2,
}

#[derive(Clone, Debug, Default)]
pub struct Locator {
    seq: Option<i64>,
    at: Option<f64>,
    pose: Option<Pose2>,
    speed: f64,
    yaw_rate: Option<f64>,
    integral: Option<f64>,
    last_axis: [Option<f64>; 2],
    support: [usize; 2],
    mean_error: f64,
    fixes: VecDeque<Fix>,
    observed_centers: [Option<Point2>; 2],
    pub planner: crate::race_field_path::Planner,
    pub imu: crate::race_imu_fusion::Filter,
    scan_yaw_integral: Option<(f64, f64)>,
    prediction: Option<Pose2>,
    clear_since: [Option<f64>; 2],
    cone_fix: bool,
    fragment_fix: bool,
    valid_rays: usize,
}

impl Locator {
    pub fn prior(&self, scan: &V, old: &V, cone: usize) -> crate::compact_target::TrackingPrior {
        crate::compact_target::TrackingPrior {
            carried: self.carry(scan, old, cone),
            center: self.landmark(scan, cone),
        }
    }

    pub fn aid_compass(&self, scan: &V, compass: &mut crate::helpers::WallCompass, rate: Option<f64>) {
        let Some(h) = self.heading(scan) else { return };
        let h = compass.heading.map_or(h, |old| old + crate::py::wrap(h - old));
        let native = compass.evidence.get("basis").os() == Some("native_wall_directions_mod_90_vs_start_lane_axis")
            && compass.evidence.get("measured").is_true();
        if native && compass.heading.is_some_and(|old| (h - old).abs() < 5_f64.to_radians()) {
            return;
        }
        // WallCompass::current subtracts this exact millisecond quotient and
        // requires a nonnegative age even on the measurement tick.
        let Some(at) = scan.get("at_ms").oi().map(|v| v as f64 / 1000.) else {
            return;
        };
        compass.heading = Some(h);
        compass.at_s = Some(at);
        if self.support.iter().any(|&n| n >= 4) {
            compass.measured_at_s = Some(at);
            compass.last_measurement = Some((h, at));
        }
        if let Some(w) = rate.filter(|v| v.is_finite()) {
            compass.rate = w;
        }
        compass.evidence = crate::dict! {"basis"=>"surveyed_boards_and_imu","heading_rad"=>h,"source_at_ms"=>scan.get("at_ms").clone(),
        "rate_rad_s"=>compass.rate, "measured"=>self.support.iter().any(|&n| n>=4)};
    }

    pub fn observe(&mut self, scan: &V, now: f64, imu: RaceInputs, at_start: bool) {
        let Some(field) = FIELD.as_ref() else { return };
        let (Some(seq), Some(at)) = (scan.get("seq").oi(), scan.get("received_at").of()) else {
            return;
        };
        if self.seq.is_some_and(|n| seq <= n)
            || !(0.0..SCAN_AGE_S).contains(&(now - at))
            || self.at.is_some_and(|t| at <= t)
        {
            return;
        }
        let Some((ranges, true)) = ranges_info(scan.get("ranges")) else {
            return;
        };
        let interval = self.at.map_or(0., |old| at - old);
        let dt = if interval <= BRIDGE_S { interval } else { 0. };
        let w = imu.yaw_rate.filter(|v| v.is_finite());
        let endpoint_rate = self.yaw_rate.zip(w).map_or(w.unwrap_or(0.), |(a, b)| (a + b) * 0.5);
        let yaw = self
            .scan_yaw_integral
            .and_then(|(last, elapsed)| {
                let covered = self.imu.integrated_time - elapsed;
                if !self.imu.yaw_available || dt <= 0. || (covered - dt).abs() > 0.025 {
                    return None;
                }
                let delta = self.imu.yaw_integral - last;
                let corrected = self
                    .imu
                    .last_gyro_correction
                    .is_some_and(|t| (0.0..=SCAN_AGE_S).contains(&(now - t)));
                // Preserve the existing scan-cadence estimate when both agree.
                // Integrate the short high-rate samples when a spike was replaced
                // or endpoint sampling would alias a rapid angular change.
                (corrected || (delta - endpoint_rate * dt).abs() > 0.055).then_some(delta / dt)
            })
            .unwrap_or(endpoint_rate);
        let integral = imu.forward_integral.filter(|v| v.is_finite());
        let dv = if dt > 0. {
            integral.zip(self.integral).map_or(0., |(a, b)| a - b)
        } else {
            0.
        };
        let speed = self.speed + dv;
        let old = if let Some(pose) = self.pose {
            pose
        } else {
            if !at_start {
                return;
            }
            // A start anywhere inside the box is allowed. The rear board and
            // the right board set the origin when visible; the seed alone is never a fix.
            let sector = |start: usize, end: usize, axis: usize, max: f64| -> Option<f64> {
                let mut values = TABLES.with(|t| {
                    (start..=end)
                        .filter_map(|i| {
                            let (x, y) = t.directions[i];
                            let d = ranges[i]? * if axis == 0 { -x } else { -y };
                            (d > 0. && d <= max + 0.18).then_some(d)
                        })
                        .collect::<Vec<_>>()
                });
                if values.len() < 3 {
                    return None;
                }
                values.sort_by(f64::total_cmp);
                Some(values[values.len() / 2])
            };
            Pose2 {
                x_m: sector(167, 193, 0, field.start[0]).unwrap_or(field.start[0] * 0.5),
                y_m: sector(77, 103, 1, field.start[1]).unwrap_or(field.start[1] * 0.5),
                yaw_rad: 0.,
            }
        };
        let mid = old.yaw_rad + 0.5 * yaw * dt;
        let (s, c) = mid.sin_cos();
        let travel = (self.speed + 0.5 * dv) * dt;
        let mut prior = Pose2 {
            x_m: old.x_m + travel * c,
            y_m: old.y_m + travel * s,
            yaw_rad: old.yaw_rad + yaw * dt,
        };
        self.prediction = Some(prior);
        let mut beams = rays(scan, &ranges, speed, w.unwrap_or(0.));
        self.fragment_fix = false;
        if beams.len() <= 48
            && let Some(radar) = self.imu.radar.filter(|r| at - r.at <= BRIDGE_S)
        {
            let seed = Pose2 {
                yaw_rad: old.yaw_rad + radar.yaw * dt,
                ..prior
            };
            let reliable_rays = rays(scan, &ranges, speed, radar.yaw);
            if let Some(h) = fragment_heading(field, seed, &reliable_rays) {
                // Agreeing gyro retains its resolution; disagreeing increments
                // are corrected by the observed line, not by a steering limit.
                let discrepancy = crate::py::wrap(h - prior.yaw_rad);
                prior.yaw_rad += discrepancy * if discrepancy.abs() > 0.045 { 0.9 } else { 0.32 };
                beams = reliable_rays;
                self.fragment_fix = true;
            }
        }
        self.valid_rays = beams.len();
        let (mut pose, support, error) = register(field, prior, &beams, w.is_some());
        let mut anchored = support.map(|n| n >= 4);
        self.cone_fix = false;
        if !anchored.iter().all(|v| *v)
            && w.is_some()
            && self.at.is_some_and(|t| at - t <= BRIDGE_S)
            && self.last_axis.iter().all(|t| t.is_some_and(|t| at - t <= BRIDGE_S))
        {
            let candidates = crate::compact_target::candidates(
                scan.get("ranges"),
                crate::compact_target::CandidateArgs {
                    tolerant: true,
                    min_points: Some(3),
                    bearing_range_rad: (-std::f64::consts::PI, std::f64::consts::PI),
                    max_range_m: crate::compact_target::EXTENDED_MAX_RANGE_M,
                    ..Default::default()
                },
            );
            let mut matched = None;
            let mut hits = 0;
            for c in &candidates {
                let bins: Vec<_> = c.get("support_bins").l().iter().map(|v| v.i() as usize).collect();
                let (center, _) = crate::helpers::support_center(&ranges, &bins, c.get("diameter_m"));
                let actual = pose.body_to_world(Point2 {
                    x_m: center.0,
                    y_m: center.1,
                });
                for target in self.observed_centers.iter().flatten() {
                    if actual.distance(*target) < 0.23 {
                        hits += 1;
                        matched = Some((target.x_m - actual.x_m, target.y_m - actual.y_m));
                    }
                }
            }
            if let Some((dx, dy)) = matched
                && hits == 1
            {
                if !anchored[0] {
                    pose.x_m += dx;
                }
                if !anchored[1] {
                    pose.y_m += dy;
                }
                anchored = [true, true];
                self.cone_fix = true;
            }
        }
        if self.pose.is_none()
            && (beams.is_empty()
                || pose.x_m < 0.
                || pose.x_m > field.start[0] + 0.18
                || pose.y_m < 0.
                || pose.y_m > field.start[1] + 0.18)
        {
            return;
        }
        self.speed = if dt > 0. && ((anchored[0] || c.abs() < 0.25) && (anchored[1] || s.abs() < 0.25)) {
            let along = ((pose.x_m - old.x_m) * c + (pose.y_m - old.y_m) * s) / dt;
            0.45 * speed + 0.55 * along
        } else if interval > BRIDGE_S {
            0.
        } else {
            speed
        };
        self.seq = Some(seq);
        self.at = Some(at);
        self.yaw_rate = w;
        self.integral = integral;
        self.pose = Some(pose);
        self.scan_yaw_integral = self
            .imu
            .yaw_available
            .then_some((self.imu.yaw_integral, self.imu.integrated_time));
        if dt > 0. && (support[0] >= 6 || support[1] >= 6 || self.fragment_fix) && error < 0.12 {
            let forward = (support[0] >= 4 || c.abs() < 0.25) && (support[1] >= 4 || s.abs() < 0.25);
            self.imu.anchor(
                at,
                crate::py::wrap(pose.yaw_rad - old.yaw_rad) / dt,
                self.speed,
                forward,
            );
            if self.imu.yaw_available {
                self.integral = self.imu.current().forward_integral;
            }
        }
        self.support = support;
        self.mean_error = error;
        for (axis, ok) in anchored.into_iter().enumerate() {
            if ok {
                self.last_axis[axis] = Some(at);
            }
        }
        while self.fixes.front().is_some_and(|f| at - f.at > BRIDGE_S) {
            self.fixes.pop_front();
        }
        let single_board = self.fragment_fix && anchored.iter().any(|b| *b) && w.is_some() && integral.is_some();
        if self.last_axis.iter().all(|v| {
            v.is_some_and(|t| {
                let age = at - t;
                age <= BRIDGE_S || (single_board && 0.06 * age + 0.12 * age * age < MATCH_M)
            })
        }) {
            self.fixes.push_back(Fix { seq, at, pose });
            self.planner.observe(seq, pose, self.speed, w.unwrap_or(0.));
            for cone in 0..2 {
                let point = pose.world_to_body(self.observed_centers[cone].unwrap_or(field.cones[cone]));
                let dist = point.x_m.hypot(point.y_m);
                let bin = (-point.y_m.atan2(point.x_m).to_degrees()).round() as i32;
                let half = (0.08 / dist.max(0.08)).asin().to_degrees().ceil() as i32;
                let mut farther = 0;
                let mut near = 0;
                for off in -half..=half {
                    if let Some(r) = ranges[(bin + off).rem_euclid(360) as usize] {
                        if r > dist + 0.25 {
                            farther += 1;
                        }
                        if (r - dist).abs() < 0.25 {
                            near += 1;
                        }
                    }
                }
                if near == 0 && farther > half {
                    self.clear_since[cone].get_or_insert(at);
                } else {
                    self.clear_since[cone] = None;
                }
            }
        }
    }

    fn current(&self, scan: &V) -> Option<Fix> {
        self.fixes
            .back()
            .copied()
            .filter(|f| Some(f.seq) == scan.get("seq").oi() && Some(f.at) == scan.get("received_at").of())
    }

    pub fn longitudinal_position(&self, scan: &V) -> Option<f64> {
        Some(self.current(scan)?.pose.x_m)
    }

    pub fn heading(&self, scan: &V) -> Option<f64> {
        Some(self.current(scan)?.pose.yaw_rad)
    }

    pub fn landmark(&self, scan: &V, cone: usize) -> Option<XY> {
        let p = self
            .current(scan)?
            .pose
            .world_to_body(self.observed_centers[cone].unwrap_or(FIELD.as_ref()?.cones[cone]));
        Some((p.x_m, p.y_m))
    }

    pub fn planning_center(&self, scan: &V, cone: usize) -> Option<XY> {
        let p = self.landmark(scan, cone)?;
        let distance = p.0.hypot(p.1);
        if !(0.05..=crate::compact_target::EXTENDED_MAX_RANGE_M).contains(&distance) {
            return None;
        }
        // Only sustained positive observations through the predicted cone
        // contradict the map. One sparse revolution cannot switch steering back.
        (!self.clear_since[cone]
            .zip(self.at)
            .is_some_and(|(start, at)| at - start >= SCAN_AGE_S))
        .then_some(p)
    }

    pub fn matches_observation(&self, scan: &V, target: &V, cone: usize) -> bool {
        let Some(center) = self.landmark(scan, cone) else {
            return false;
        };
        let Some([x, y]) = target.get("point_left_m").ol() else {
            return false;
        };
        target.get("source_seq").oi() == scan.get("seq").oi()
            && x.of()
                .zip(y.of())
                .is_some_and(|(x, y)| (x - center.0).hypot(y - center.1) <= 0.40)
    }

    pub fn gantry_ahead(&self, scan: &V) -> Option<f64> {
        let pose = self.current(scan)?.pose;
        let f = FIELD.as_ref()?;
        let divider = &f.walls[5];
        (pose.y_m >= divider.fixed - 0.25 && crate::py::wrap(pose.yaw_rad).abs() < 1.10)
            .then_some(divider.lo - pose.x_m)
    }

    pub fn finish_ahead(&self, scan: &V) -> Option<f64> {
        let pose = self.current(scan)?.pose;
        let f = FIELD.as_ref()?;
        (pose.y_m >= f.walls[5].fixed - 0.05 && crate::py::wrap(pose.yaw_rad).abs() < 0.55)
            .then_some(f.walls[1].fixed - pose.x_m)
    }

    pub fn axis_speed(&self, scan: &V) -> Option<f64> {
        Some((self.speed * self.current(scan)?.pose.yaw_rad.cos()).max(0.))
    }

    pub fn two_measured_axes(&self, scan: &V) -> bool {
        self.current(scan).is_some() && self.support.iter().all(|&n| n >= 4)
    }

    pub fn missing_fraction(&self) -> f64 {
        1. - self.valid_rays as f64 / 360.
    }

    /// Compare the two already-computed steering commands against surveyed
    /// boards after the measured steering response. Reuse the live arc test.
    /// This selects steering only; it neither sets speed nor updates clearance.
    pub fn map_arc_has_more_room(&self, scan: &V, live: i64, mapped: i64) -> bool {
        if live == mapped || self.missing_fraction() < 0.03 {
            return false;
        }
        let (Some(fix), Some(field)) = (self.current(scan), FIELD.as_ref()) else {
            return false;
        };
        let dt = crate::race_first_cone::STEER_RESPONSE_S;
        let yaw = self.yaw_rate.unwrap_or(0.);
        let speed = self.speed.max(0.);
        let (s, c) = (fix.pose.yaw_rad + 0.5 * dt * yaw).sin_cos();
        let pose = Pose2 {
            x_m: fix.pose.x_m + c * dt * speed,
            y_m: fix.pose.y_m + s * dt * speed,
            yaw_rad: fix.pose.yaw_rad + dt * yaw,
        };
        let reach = crate::maneuver::OPEN_LOOKAHEAD_M.max(0.7 * speed);
        let limit = reach + crate::helpers::FRONT_BODY_EXTENT_M + crate::helpers::SIDE_BODY_EXTENT_M;
        let mut points = Vec::new();
        for wall in &field.walls {
            let (perpendicular, along) = if wall.vertical {
                (pose.x_m, pose.y_m)
            } else {
                (pose.y_m, pose.x_m)
            };
            if (perpendicular - wall.fixed).abs() > limit {
                continue;
            }
            let lo = wall.lo.max(along - limit);
            let hi = wall.hi.min(along + limit);
            if hi < lo {
                continue;
            }
            let steps = ((hi - lo) / 0.06).ceil().max(1.) as usize;
            for i in 0..=steps {
                let a = lo + (hi - lo) * i as f64 / steps as f64;
                let (x_m, y_m) = if wall.vertical {
                    (wall.fixed, a)
                } else {
                    (a, wall.fixed)
                };
                let q = pose.world_to_body(Point2 { x_m, y_m });
                points.push((q.x_m, q.y_m));
            }
        }
        let room = crate::maneuver::Maneuver::path_free_distance(&points, live);
        room < reach
            && crate::maneuver::Maneuver::path_free_distance(&points, mapped)
                > room + crate::helpers::MANEUVER_BODY_CLEARANCE_M
    }

    pub fn lower_divider_distance(&self, scan: &V) -> Option<f64> {
        Some(self.current(scan)?.pose.y_m - FIELD.as_ref()?.walls[4].fixed)
    }

    pub fn finish_lane(&self, scan: &V) -> V {
        let (Some(fix), Some(field)) = (self.current(scan), FIELD.as_ref()) else {
            return V::None;
        };
        let pose = fix.pose;
        let upper = &field.walls[5];
        let outer = field.walls[3].fixed - pose.y_m;
        let inner = upper.fixed - pose.y_m;
        crate::dict! {"outer_lateral_m"=>outer,"inner_lateral_m"=>inner,"inner_start_along_m"=>upper.lo-pose.x_m,
        "outer_source"=>"surveyed_board","inner_source"=>"surveyed_divider","width_m"=>outer-inner,
        "width_source"=>"surveyed","center_lateral_m"=>(outer+inner)*0.5,"walls"=>0}
    }

    // A tracked spot is usually on the near face of a cone, not at its centre.
    // Carry the actual surface point with the vehicle's translation and rotation.
    pub fn carry(&self, scan: &V, old: &V, cone: usize) -> Option<XY> {
        let seq = old.get("source_seq").oi()?;
        let old_pose = self.fixes.iter().rev().find(|f| f.seq == seq)?.pose;
        let point = old.get("point_left_m").ol()?;
        let world = old_pose.body_to_world(Point2 {
            x_m: point.first()?.of()?,
            y_m: point.get(1)?.of()?,
        });
        if world.distance(FIELD.as_ref()?.cones[cone]) > 0.65 {
            return None;
        }
        let p = self.current(scan)?.pose.world_to_body(world);
        Some((p.x_m, p.y_m))
    }

    pub fn observe_cone(&mut self, scan: &V, cone: usize, center: XY) {
        let Some(fix) = self.current(scan) else { return };
        let Some(field) = FIELD.as_ref() else { return };
        if self.support.iter().any(|&n| n < 4) {
            return;
        }
        let w = fix.pose.body_to_world(Point2 {
            x_m: center.0,
            y_m: center.1,
        });
        if w.distance(field.cones[cone]) <= 0.55 {
            let p = self.observed_centers[cone].unwrap_or(w);
            self.observed_centers[cone] = Some(Point2 {
                x_m: p.x_m * 0.7 + w.x_m * 0.3,
                y_m: p.y_m * 0.7 + w.y_m * 0.3,
            });
        }
    }

    pub fn diagnostics(&self, scan: &V) -> V {
        let fix = self.current(scan);
        let preview = |side| fix.and_then(|f| self.planner.aim(f.seq, side)).map(|a| a.pwm);
        crate::dict! {"source_seq"=>self.seq, "localized"=>fix.is_some(),
            "imu_prediction_xy_yaw"=>self.prediction.map(|p|vec![p.x_m,p.y_m,p.yaw_rad]),
            "short_board_heading_observed"=>self.fragment_fix,
            "gyro_replaced_from_radar"=>self.imu.gyro_replaced,"accel_replaced_from_radar"=>self.imu.accel_replaced,
            "corrected_yaw_rate"=>self.imu.current().yaw_rate,"radar_yaw_rate"=>self.imu.radar.map(|r|r.yaw),
        "pose_xy_yaw"=>fix.map(|f| vec![f.pose.x_m,f.pose.y_m,f.pose.yaw_rad]),
        "wall_points_xy"=>vec![self.support[0] as i64,self.support[1] as i64],
        "mean_wall_residual_m"=>self.mean_error,"speed_mps"=>self.speed,"measured_cone_fix"=>self.cone_fix,
        "preview_pwm"=>V::list(vec![V::from(preview(0)),V::from(preview(1))]),
        "axis_age_s"=>V::list(self.last_axis.iter().map(|t|V::from(self.at.zip(*t).map(|(a,b)|a-b))).collect())}
    }
}

impl crate::maneuver::Maneuver {
    pub(crate) fn set_live_imu(&mut self, input: RaceInputs) {
        self.raw_race_inputs = Some(input);
        self.race_inputs = Some(input);
    }
    pub(super) fn field_course_active(&self) -> bool {
        self.race_program
            && self.race_inputs.is_some()
            && self.entry_kind == Some("lane")
            && (self.crosswalk_state == Some("done")
                || matches!(
                    self.route_stage,
                    "first_target" | "open_transfer" | "second_loop" | "light_entry" | "light_lane"
                ))
    }
    pub(super) fn use_field_guidance(&self) -> bool {
        self.field_course_active() && self.phase == "drive"
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::helpers::WallCompass;

    #[test]
    fn surveyed_brake_coast_registers_a_measured_light_stop_without_colour() {
        use crate::helpers::FRONT_BODY_EXTENT_M;
        use crate::maneuver::Maneuver;
        use crate::race_light_brake::BrakeModel;

        let mut approach = Maneuver::new(&0.0.into(), &40.0.into(), &1670.into(), &true.into());
        approach.race_program = true;
        approach.race_inputs = Some(RaceInputs { yaw_rate: Some(0.), forward_integral: Some(0.) });
        approach.phase = "drive";
        approach.entry_kind = Some("lane");
        approach.route_stage = "light_lane";
        approach.light_vision = true;
        approach.light_stop_state = Some("approach");
        approach.drive_since = Some(0.);
        approach._light_gantry_since = Some(1.);
        approach.light_stop_evidence = crate::dict! { "gantry" => crate::dict! { "source_seq" => 1 } };
        approach._light_line = crate::dict! { "source" => "legs", "confirmed_seq" => 1, "station_spacing_m" => 1.10 };
        approach.race_light.speed_uses_gantry = true;
        const WAIT: f64 = 7.;
        approach.race_light_wait_s = Some(WAIT);

        // The brake remains applied until the projected coast ends inside the surveyed zone.
        let brake = &mut approach.race_light_brake;
        for i in 0..=4 {
            let dt = i as f64 / 10.;
            let speed = 1.30 - 1.875 * dt;
            let gap = 1.43 - 1.30 * dt + 0.9375 * dt * dt;
            let integral = Some(speed - 1.30);
            brake.set_surveyed_release_zone((gap <= 1.10).then_some(0.72));
            brake.observe(1. + dt, Some(gap), Some(speed), integral, 1., 0.35, BrakeModel::light(1., 0.35), 0.55);
            assert_eq!(brake.command(1. + dt, integral), Some(if i == 4 { 1500 } else { 1300 }));
        }
        assert_eq!(brake.diagnostics().get("end_reason").os(), Some("coast_reaches_zone"));

        let scan = |at: f64| crate::dict! { "seq" => (at * 10.).round() as i64, "at_ms" => (at * 1000.).round() as i64,
            "received_at" => at, "ranges" => vec![V::Float(3.); 360] };
        let at = 3.;
        for (gap, speed, support, should_wait, should_advance) in [
            (0.62, 0.45, None, false, false),         // still rolling after release
            (0.62, 0.00, None, true, false),          // only the gantry legs remain visible
            (0.62, 0.00, Some([0, 6]), true, false),  // one mapped board axis remains
            (-0.02, 0.00, Some([6, 6]), true, false), // nose just beyond the gantry
            (0.85, 0.00, Some([6, 6]), false, true),  // rear still upstream of the beam
        ] {
            let mut m = approach.clone();
            let ahead = gap + FRONT_BODY_EXTENT_M;
            if let Some(support) = support {
                let divider = &FIELD.as_ref().unwrap().walls[5];
                let pose = Pose2 { x_m: divider.lo - ahead, y_m: divider.fixed + 0.45, yaw_rad: 0. };
                m.field_map.fixes.push_back(Fix { seq: 30, at, pose });
                m.field_map.support = support;
            }
            m.race_light.ranges = vec![(2.8, 2.8, ahead + 0.2 * speed), (2.9, 2.9, ahead + 0.1 * speed)];
            let (basis, feedback) = m.light_approach_control(&scan(at), Some(ahead), at);
            assert_eq!(basis.is_some(), should_wait, "gap={gap}, speed={speed}, support={support:?}");
            assert_eq!(m.race_light.completing_surveyed_stop, should_advance);
            assert_eq!(m.lane_motor, if should_advance { 1560 } else { 1500 });
            if let Some(basis) = basis {
                m.lane_feedback = feedback;
                m.light_stop_reached(&scan(at), at, &V::None, &basis);
                assert_eq!(m.light_stop_state, Some("waiting"));
                assert!(!m.light_wait(&scan(at + WAIT - 0.1), at + WAIT - 0.1, &V::None));
                assert!(m.light_wait(&scan(at + WAIT), at + WAIT, &V::None));
                assert_eq!(m.light_stop_state, Some("released"));
            }
        }
    }

    #[test]
    fn current_scan_compass_uses_the_same_clock_and_an_unobserved_pose_expires() {
        let field = FIELD.as_ref().unwrap();
        let ranges = TABLES.with(|table| {
            V::list(
                table
                    .directions
                    .iter()
                    .map(|&(x, y)| V::from(field.hit(0.40, 0.45, x, y).map(|(_, range)| range)))
                    .collect(),
            )
        });
        let mut locator = Locator::default();
        let mut compass = WallCompass::default();
        let imu = RaceInputs {
            yaw_rate: Some(0.),
            forward_integral: Some(0.),
        };
        // Multiplying these millisecond stamps by .001 can put the purported
        // measurement just after the consumer's division by 1000.
        for (i, ms) in [10988, 11088, 11188].into_iter().enumerate() {
            let at = ms as f64 / 1000.;
            let scan = crate::dict! {"seq"=>i as i64,"at_ms"=>ms as i64,"received_at"=>at,"ranges"=>ranges.clone()};
            locator.observe(&scan, at, imu, true);
            locator.aid_compass(&scan, &mut compass, imu.yaw_rate);
            assert!(locator.two_measured_axes(&scan));
            assert!(compass.current(&scan).unwrap().abs() < 0.01);
        }
        let blank = V::list(vec![V::None; 360]);
        for n in 1..=8 {
            let ms = 11188 + 100 * n;
            let at = ms as f64 / 1000.;
            let scan = crate::dict! {"seq"=>(n+2) as i64,"at_ms"=>ms as i64,"received_at"=>at,"ranges"=>blank.clone()};
            locator.observe(&scan, at, imu, false);
            assert!(!locator.two_measured_axes(&scan));
            assert_eq!(locator.heading(&scan).is_some(), n <= 6);
        }
    }
}
