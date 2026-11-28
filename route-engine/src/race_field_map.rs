//! Race-course localization. Axes are forward from the start and left toward
//! the finish lane. All poses describe the lidar spindle, in metres and radians.
use std::collections::{BTreeMap, VecDeque};
use std::sync::LazyLock;

use crate::compact_target::{ConeReturnBins, TABLES, ranges_info};
use crate::helpers::SCAN_AGE_S;
use crate::motion_sensors::RaceInputs;
use crate::py::V;
use xt_stcar_robot_core::autonomy::{Point2, Pose2};

type XY = (f64, f64);
const BRIDGE_S: f64 = 0.65; // lifetime of a coordinate supported only by inertial motion
const MATCH_M: f64 = 0.25;

pub(crate) struct Board {
    pub(crate) vertical: bool,
    pub(crate) fixed: f64,
    pub(crate) lo: f64,
    pub(crate) hi: f64,
}
pub(crate) struct Field {
    pub(crate) walls: [Board; 6],
    // Optional starting estimates for steering. Vehicle pose is registered against the boards.
    cone_priors: [Option<Point2>; 2],
    pub(crate) start: [f64; 2],
    pub(crate) beam_spacing: f64,
    lamp_heights: Option<[f64; 2]>,
}
pub(crate) static FIELD: LazyLock<Option<Field>> = LazyLock::new(|| {
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
    Some(Field {
        walls: [
            board(true, 0., 0., width),
            board(true, length, 0., width),
            board(false, 0., 0., length),
            board(false, width, 0., length),
            board(false, lower, 0., a),
            board(false, upper, b, length),
        ],
        cone_priors: ["first_cone_prior_m", "second_cone_prior_m"]
            .map(|key| m.get(key).map(|&[x_m, y_m]| Point2 { x_m, y_m })),
        start: *m.get("start_zone_upper_right_m")?,
        beam_spacing: gantry - beam,
        lamp_heights: m.get("lamp_housing_bottom_top_m").copied().filter(|h|
            h[0] > crate::race_cam_features::CAM_HEIGHT_M && h[1] > h[0]),
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

pub(super) fn path_anchors() -> Option<[XY; 3]> {
    let f = FIELD.as_ref()?;
    Some([
        (f.walls[1].fixed, f.walls[3].fixed),
        (f.walls[4].hi, f.walls[4].fixed),
        (f.walls[5].lo, f.walls[5].fixed),
    ])
}

pub(crate) fn dist_to_any_wall(field: &Field, p: Point2) -> f64 {
    let mut best = f64::INFINITY;
    for b in &field.walls {
        let d = if b.vertical {
            let cy = p.y_m.clamp(b.lo, b.hi);
            (p.x_m - b.fixed).hypot(p.y_m - cy)
        } else {
            let cx = p.x_m.clamp(b.lo, b.hi);
            (p.x_m - cx).hypot(p.y_m - b.fixed)
        };
        best = best.min(d);
    }
    best
}

pub(crate) fn in_open_course(field: &Field, p: Point2) -> bool {
    let length = field.walls[1].fixed;
    let width = field.walls[3].fixed;
    if !(0.30..=(length - 0.30)).contains(&p.x_m) || !(0.30..=(width - 0.30)).contains(&p.y_m) {
        return false;
    }
    if dist_to_any_wall(field, p) < 0.30 {
        return false;
    }
    // Cones live in the open arena, never inside the start or finish divider lanes.
    if p.x_m < field.walls[4].hi + 0.05 && p.y_m < field.walls[4].fixed + 0.15 {
        return false;
    }
    if p.x_m > field.walls[5].lo - 0.05 && p.y_m > field.walls[5].fixed - 0.15 {
        return false;
    }
    true
}

impl Field {
    // The first intersected finite board wins, including the two dividers.
    pub(crate) fn hit(&self, x: f64, y: f64, dx: f64, dy: f64) -> Option<(usize, f64)> {
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
pub(crate) struct Ray {
    pub(crate) bin: usize,
    pub(crate) range: f64,
    pub(crate) dx: f64,
    pub(crate) dy: f64,
    pub(crate) ox: f64,
    pub(crate) oy: f64,
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
                    bin: i,
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

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ConeSource {
    #[default]
    None,
    Camera,
    Lidar,
    Fused,
}

impl ConeSource {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::None => "none",
            Self::Camera => "camera",
            Self::Lidar => "lidar",
            Self::Fused => "fused",
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct DynamicConeSlot {
    world: Option<Point2>,
    source: ConeSource,
    hits: u32,
    last_seen_at: f64,
    last_position_at: Option<f64>,
    pending_cam: Option<CameraVote>,
}

#[derive(Clone, Copy, Debug)]
struct CameraVote {
    first: Point2,
    lens: Point2,
    at: f64,
    frames: u8,
}

impl CameraVote {
    // Two nearly parallel rays, including a camera panning from one location, cannot verify range.
    fn verifies_range(self, point: Point2, lens: Point2, tolerance: f64) -> bool {
        let first = (self.first.x_m - self.lens.x_m, self.first.y_m - self.lens.y_m);
        let next = (point.x_m - lens.x_m, point.y_m - lens.y_m);
        let cross = |a: XY, b: XY| a.0 * b.1 - a.1 * b.0;
        let det = cross(first, next);
        let lengths = first.0.hypot(first.1) * next.0.hypot(next.1);
        if lengths <= 0.0 || det.abs() < 0.08 * lengths {
            return false;
        }
        let baseline = (lens.x_m - self.lens.x_m, lens.y_m - self.lens.y_m);
        let along = cross(baseline, next) / det;
        if along <= 0.0 || cross(baseline, first) / det <= 0.0 {
            return false;
        }
        let intersection = Point2 {
            x_m: self.lens.x_m + along * first.0,
            y_m: self.lens.y_m + along * first.1,
        };
        intersection.distance(point) <= tolerance && intersection.distance(self.first) <= tolerance
    }
}

impl DynamicConeSlot {
    fn confirm_camera_position(&mut self, point: Point2, lens: Point2, at: f64, tolerance: f64, different_view: bool) {
        if let Some(mut vote) = self.pending_cam {
            if vote.at == at {
                return;
            }
            if at - vote.at <= 0.90 && vote.first.distance(point) <= tolerance {
                vote.at = at;
                vote.frames = vote.frames.saturating_add(1);
                if different_view && (vote.frames < 3 || !vote.verifies_range(point, lens, tolerance)) {
                    self.pending_cam = Some(vote);
                    return;
                }
                self.world = Some(Point2 {
                    x_m: (vote.first.x_m + point.x_m) * 0.5,
                    y_m: (vote.first.y_m + point.y_m) * 0.5,
                });
                self.source = ConeSource::Camera;
                self.hits = u32::from(vote.frames);
                self.last_seen_at = at;
                self.pending_cam = None;
                return;
            }
        }
        self.pending_cam = Some(CameraVote {first: point, lens, at, frames: 1});
    }
}

#[derive(Clone, Copy, Debug)]
struct FilteredRanges([Option<f64>; 360]);

impl Default for FilteredRanges {
    fn default() -> Self {
        Self([None; 360])
    }
}

const MIN_CONE_SEPARATION_M: f64 = 0.75;

#[derive(Clone, Copy, Debug)]
struct LidarConeCluster {
    world: Point2,
    range_m: f64,
    bearing_rad: f64,
    bins: ConeReturnBins,
}

#[derive(Clone, Debug)]
struct StationaryEcho {
    origin: Point2,
    first_view: Pose2,
    last_at: f64,
    scans: u8,
    separate_views: bool,
}

const ECHO_REPEAT_M: f64 = 0.15;

// Anchor the comparison to the first sighting. A succession of small jumps
// in car coordinates must not walk an echo across the map while turning.
fn repeated_echo(tracks: &mut Vec<StationaryEcho>, world: Point2, pose: Pose2, at: f64) -> bool {
    let closest = tracks.iter_mut()
        .filter(|s| s.last_at < at && s.origin.distance(world) <= ECHO_REPEAT_M)
        .min_by(|a, b| a.origin.distance(world).total_cmp(&b.origin.distance(world)));
    if let Some(s) = closest {
        s.last_at = at;
        s.scans = s.scans.saturating_add(1);
        let scanner_fixed = pose.body_to_world(s.first_view.world_to_body(s.origin));
        s.separate_views |= scanner_fixed.distance(s.origin) >= 0.20;
        return s.scans >= 3 && s.separate_views;
    }
    tracks.push(StationaryEcho { origin: world, first_view: pose, last_at: at, scans: 1, separate_views: false });
    false
}

/// Fast zero-V extraction of compact open-course clusters from motion-compensated,
/// speckle-filtered LiDAR rays (eliminates ~15-20 cm high-speed scan-distortion shift).
fn lidar_cone_clusters(beams: &[Ray], pose: Pose2, field: &Field) -> Vec<LidarConeCluster> {
    let mut raw: Vec<(usize, usize, Vec<Ray>)> = Vec::new();
    let mut cur: Vec<Ray> = Vec::new();
    for &ray in beams {
        if !(0.22..=6.20).contains(&ray.range) {
            continue;
        }
        if let Some(&prev) = cur.last() {
            let gap = ray.bin.saturating_sub(prev.bin);
            if gap <= 3 && (ray.range - prev.range).abs() <= 0.14 {
                cur.push(ray);
            } else {
                if !cur.is_empty() {
                    let s = cur[0].bin;
                    let e = cur[cur.len() - 1].bin;
                    raw.push((s, e, std::mem::take(&mut cur)));
                }
                cur.push(ray);
            }
        } else {
            cur.push(ray);
        }
    }
    if !cur.is_empty() {
        let s = cur[0].bin;
        let e = cur[cur.len() - 1].bin;
        raw.push((s, e, cur));
    }
    // Wrap seam across bin 359 -> 0 if continuous.
    if raw.len() >= 2 {
        let first_i = raw[0].0;
        let last_i = raw[raw.len() - 1].1;
        let wrap_gap = (360 + first_i).saturating_sub(last_i);
        let r0 = raw[0].2[0].range;
        let rl = raw[raw.len() - 1].2.last().unwrap().range;
        if wrap_gap <= 3 && (r0 - rl).abs() <= 0.14 {
            let tail = raw.pop().unwrap().2;
            let mut merged = tail;
            merged.extend_from_slice(&raw[0].2);
            raw[0].2 = merged;
        }
    }

    let mut out = Vec::new();
    for (_, _, pts) in raw {
        let n = pts.len();
        if !(2..=42).contains(&n) {
            continue;
        }
        let r_min = pts.iter().map(|p| p.range).fold(f64::INFINITY, f64::min);
        if r_min <= 1.8 && n < 3 {
            continue;
        }
        let p0 = (pts[0].ox + pts[0].range * pts[0].dx, pts[0].oy + pts[0].range * pts[0].dy);
        let pl = (
            pts[n - 1].ox + pts[n - 1].range * pts[n - 1].dx,
            pts[n - 1].oy + pts[n - 1].range * pts[n - 1].dy,
        );
        let chord = (p0.0 - pl.0).hypot(p0.1 - pl.1);
        if chord > 0.44 {
            continue;
        }
        let mut sx = 0.0;
        let mut sy = 0.0;
        for r in &pts {
            sx += r.ox + r.range * r.dx;
            sy += r.oy + r.range * r.dy;
        }
        let mx = sx / n as f64;
        let my = sy / n as f64;
        let d = mx.hypot(my);
        if d < 0.22 {
            continue;
        }
        // Shift ~0.08 m from front surface toward ~0.10 m radius cone center.
        let scale = (d + 0.08) / d;
        let body = Point2 {
            x_m: mx * scale,
            y_m: my * scale,
        };
        let world = pose.body_to_world(body);
        if !in_open_course(field, world) {
            continue;
        }
        let mut bins = ConeReturnBins::default();
        for ray in &pts {
            bins.insert(ray.bin);
        }
        out.push(LidarConeCluster {
            world,
            range_m: d + 0.08,
            bearing_rad: body.y_m.atan2(body.x_m),
            bins,
        });
    }
    out
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
    cone_prediction: Option<Fix>,
    cones: [DynamicConeSlot; 2],
    slot0_seen_red: bool,
    last_cone_cam_frame: Option<i64>,
    cluttered_seq: Option<i64>,
    echo_tracks: Vec<StationaryEcho>,
    stable_echo_bins: ConeReturnBins,
    deferred_cone_echoes: usize,
    filtered: FilteredRanges,
    pub planner: crate::race_field_path::Planner,
    pub imu: crate::race_imu_fusion::Filter,
    scan_yaw_integral: Option<(f64, f64)>,
    prediction: Option<Pose2>,
    clear_since: [Option<f64>; 2],
    cone_fix: bool,
    fragment_fix: bool,
    valid_rays: usize,
    pub(crate) cam_loc: crate::race_cam_loc::CamLocState,
}

impl Locator {
    fn guidance_world(&self, field: &Field) -> [Option<Point2>; 2] {
        std::array::from_fn(|i| self.cones[i].world.or(field.cone_priors[i]))
    }

    fn path_cones(&self, field: &Field) -> crate::race_field_path::ConePositions {
        crate::race_field_path::ConePositions {
            centers: self.guidance_world(field),
            observed: self.cones.map(|slot| slot.world.is_some()),
        }
    }

    pub(super) fn cone_guidance_source(&self, cone: usize) -> &'static str {
        let slot = &self.cones[cone];
        if slot.world.is_none() {
            return if FIELD.as_ref().is_some_and(|f| f.cone_priors[cone].is_some()) {
                "nominal_prior"
            } else {
                "unknown"
            };
        }
        if self.at.is_some_and(|at| slot.last_seen_at < at) {
            "carried_observation"
        } else if self.cluttered_seq.is_some() && self.at != slot.last_position_at {
            "corroborated_hold"
        } else {
            slot.source.as_str()
        }
    }

    pub fn observe_camera(&mut self, obs: crate::race_cam_features::VisualFrame, captured_at: f64) {
        self.cam_loc.observe_camera(obs, captured_at);
    }

    pub fn filtered_ranges(&self) -> &[Option<f64>; 360] {
        &self.filtered.0
    }

    #[cfg(test)]
    pub fn observed_cone_world(&self, cone: usize) -> Option<Point2> {
        self.cones.get(cone).and_then(|c| c.world)
    }

    #[cfg(test)]
    pub fn observed_cone_source(&self, cone: usize) -> ConeSource {
        self.cones.get(cone).map_or(ConeSource::None, |c| c.source)
    }

    pub fn camera_localized(&self, scan: &V) -> bool {
        self.current(scan).is_some() && self.cam_loc.last_cam_anchored.iter().any(|&b| b)
    }

    pub fn raw_pose_xy_yaw(&self) -> Option<(f64, f64, f64)> {
        self.pose.map(|p| (p.x_m, p.y_m, p.yaw_rad))
    }

    pub fn force_state_for_eval(
        &mut self,
        seq: i64,
        at: f64,
        pose: Pose2,
        speed: f64,
        yaw_rate: Option<f64>,
        integral: Option<f64>,
    ) {
        self.seq = Some(seq);
        self.at = Some(at);
        self.pose = Some(pose);
        self.speed = speed;
        self.yaw_rate = yaw_rate;
        self.integral = integral;
        self.last_axis = [Some(at), Some(at)];
        self.support = [16, 16];
        self.scan_yaw_integral = None;
        self.cam_loc.last_cam_fix_at = Some(at);
        self.fixes.clear();
        self.fixes.push_back(Fix { seq, at, pose });
    }
    pub fn prior(&self, scan: &V, old: &V, cone: usize) -> crate::compact_target::TrackingPrior {
        crate::compact_target::TrackingPrior {
            carried: self.carry(scan, old, cone),
            center: self.landmark(scan, cone),
            stable_echo_bins: self.stable_echo_bins_for(scan),
        }
    }

    pub(super) fn stable_echo_bins_for(&self, scan: &V) -> Option<ConeReturnBins> {
        self.cluttered_seq.filter(|seq| Some(*seq) == scan.get("seq").oi()).map(|_| self.stable_echo_bins)
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
        let measured = self.support.iter().any(|&n| n >= 4)
            || self.cam_loc.last_cam_anchored.iter().all(|&b| b);
        if measured {
            compass.measured_at_s = Some(at);
            compass.last_measurement = Some((h, at));
        }
        if let Some(w) = rate.filter(|v| v.is_finite()) {
            compass.rate = w;
        }
        compass.evidence = crate::dict! {"basis"=>"surveyed_boards_and_imu","heading_rad"=>h,"source_at_ms"=>scan.get("at_ms").clone(),
        "rate_rad_s"=>compass.rate, "measured"=>measured};
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
        let Some((raw_ranges, true)) = ranges_info(scan.get("ranges")) else {
            return;
        };
        let (ranges, cluttered) = crate::race_cam_loc::filter_lidar_speckles(&raw_ranges);
        self.cluttered_seq = cluttered.then_some(seq);
        self.stable_echo_bins = ConeReturnBins::default();
        self.deferred_cone_echoes = 0;
        self.cone_prediction = None;
        if cluttered {
            self.clear_since = [None; 2];
        }
        self.filtered = FilteredRanges(ranges);
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
        let prev_cam_fix_at = self.cam_loc.last_cam_fix_at;
        self.valid_rays = beams.len();
        let (mut pose, mut support, mut error) = register(field, prior, &beams, w.is_some());
        let lidar_locked = support[0] >= 4 && support[1] >= 4;
        let mut cam_anchored = [false; 2];
        if lidar_locked {
            if support[0] >= 8 && support[1] >= 8 && beams.len() >= 40 && error < 0.08 {
                self.cam_loc
                    .calibrate_from_lidar(field, pose, at, speed, w.unwrap_or(0.));
            } else {
                self.cam_loc.last_used_cam = false;
                self.cam_loc.last_cam_anchored = [false; 2];
            }
        } else if self.cam_loc.has_new_obs(at) {
            let (cp, cs, ca, ce) =
                self.cam_loc
                    .joint_register(field, prior, &beams, w.is_some(), at, speed, w.unwrap_or(0.));
            if self.cam_loc.last_used_cam {
                pose = cp;
                support = cs;
                cam_anchored = ca;
                error = ce;
            }
        } else {
            self.cam_loc.last_used_cam = false;
            // Retain recent camera-anchor badge while bridging between consecutive captures (< BRIDGE_S).
            if self
                .cam_loc
                .last_cam_fix_at
                .is_none_or(|t| at - t > BRIDGE_S)
            {
                self.cam_loc.last_cam_anchored = [false; 2];
            }
        }
        let mut anchored = [
            support[0] >= 4 || cam_anchored[0],
            support[1] >= 4 || cam_anchored[1],
        ];
        self.cone_fix = false;
        if !cluttered && !anchored.iter().all(|v| *v)
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
                if bins.is_empty() || bins.iter().any(|&i| i >= ranges.len() || ranges[i].is_none()) {
                    continue;
                }
                let (center, _) = crate::helpers::support_center(&ranges, &bins, c.get("diameter_m"));
                let actual = pose.body_to_world(Point2 {
                    x_m: center.0,
                    y_m: center.1,
                });
                for slot in &self.cones {
                    let Some(target) = slot.world else { continue };
                    if !matches!(slot.source, ConeSource::Lidar | ConeSource::Fused)
                        || slot.hits < 3
                        || at - slot.last_seen_at < 0.05
                    {
                        continue;
                    }
                    if actual.distance(target) < 0.23 {
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
            && ((beams.is_empty() && !cam_anchored.iter().any(|&b| b))
                || pose.x_m < 0.
                || pose.x_m > field.start[0] + 0.35
                || pose.y_m < 0.
                || pose.y_m > field.start[1] + 0.25)
        {
            return;
        }
        // Keep degraded/unanchored pose inside physical walled field boundaries and prevent
        // unanchored accelerometer bias from running away during temporary camera/LiDAR gaps.
        if !lidar_locked {
            pose.x_m = pose.x_m.clamp(0.02, field.walls[1].fixed - 0.02);
            pose.y_m = pose.y_m.clamp(0.02, field.walls[3].fixed - 0.02);
        }
        self.speed = if dt > 0. && ((anchored[0] || c.abs() < 0.25) && (anchored[1] || s.abs() < 0.25)) {
            if lidar_locked {
                let along = ((pose.x_m - old.x_m) * c + (pose.y_m - old.y_m) * s) / dt;
                0.45 * speed + 0.55 * along
            } else if self.cam_loc.last_used_cam {
                // Correct velocity bias using innovation (pose - prior) over the real inter-frame interval
                // rather than dividing a 0.3 s accumulated correction by a single 0.1 s scan step.
                // A scan accepted by the IMU bridge can itself span up to BRIDGE_S.
                let dt_fix = prev_cam_fix_at
                    .map_or(dt.max(0.20), |t| (at - t).clamp(0.10, 0.60).max(dt));
                let delta_along = (pose.x_m - prior.x_m) * c + (pose.y_m - prior.y_m) * s;
                let corr = (delta_along / dt_fix).clamp(-0.6, 0.6);
                let lo = if speed > 0.05 { 0.0 } else { -0.4 };
                (speed + 0.45 * corr).clamp(lo, 2.5)
            } else {
                let along = ((pose.x_m - old.x_m) * c + (pose.y_m - old.y_m) * s) / dt;
                let lo = if speed > 0.05 { 0.0 } else { -0.4 };
                (0.75 * speed + 0.25 * along.clamp(lo, 2.5)).clamp(lo, 2.5)
            }
        } else if interval > BRIDGE_S {
            0.
        } else if !lidar_locked {
            (speed * 0.985).clamp(-0.4, 2.5)
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
        let cam_anchor_ok = cam_anchored.iter().all(|&b| b) && self.cam_loc.last_pixel_rms < 8.0;
        if dt > 0.
            && (support[0] >= 6 || support[1] >= 6 || self.fragment_fix || cam_anchor_ok)
            && error < 0.12
        {
            let forward = (anchored[0] || c.abs() < 0.25) && (anchored[1] || s.abs() < 0.25);
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
        let single_board = (self.fragment_fix
            || (self.cam_loc.last_used_cam && self.cam_loc.last_inliers >= 12))
            && anchored.iter().any(|b| *b)
            && w.is_some()
            && integral.is_some();
        let within_motion_uncertainty = |age: f64| 0.06 * age + 0.12 * age * age < MATCH_M;
        if self.last_axis.iter().all(|v| {
            v.is_some_and(|t| {
                let age = at - t;
                age <= BRIDGE_S || (single_board && within_motion_uncertainty(age))
            })
        }) {
            self.fixes.push_back(Fix { seq, at, pose });
            self.fuse_dynamic_cones(field, pose, at, w.unwrap_or(0.), &beams);
            for cone in 0..2 {
                let Some(cone_world) = self.cones[cone].world else {
                    self.clear_since[cone] = None;
                    continue;
                };
                if cluttered || self.valid_rays < 40 {
                    self.clear_since[cone] = None;
                    continue;
                }
                let point = pose.world_to_body(cone_world);
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
                    let start = *self.clear_since[cone].get_or_insert(at);
                    // If clean R1 LiDAR consistently sees straight through a camera-only
                    // false positive, clear the slot so the real cone can be acquired cleanly.
                    if self.cones[cone].source == ConeSource::Camera && at - start >= 0.45 {
                        self.cones[cone] = DynamicConeSlot::default();
                        self.clear_since[cone] = None;
                    }
                } else {
                    self.clear_since[cone] = None;
                }
            }
            self.planner.observe(seq, pose, self.speed, w.unwrap_or(0.0), self.path_cones(field));
        } else if dt > 0.0 && w.is_some() && integral.is_some()
            && self.last_axis.iter().all(|t| t.is_some_and(|t| within_motion_uncertainty(at - t))) {
            // Short IMU projection for a steering preview only. Do not mark the car localized,
            // learn new cone positions here, or use this projection to certify clearance or stopping.
            self.cone_prediction = Some(Fix { seq, at, pose });
            self.planner.observe(seq, pose, self.speed, w.unwrap_or(0.0), self.path_cones(field));
        }
    }

    /// Dual-sensor dynamic cone perception & fusion: runs on every localized scan
    /// The nominal path is never an input measurement to this function.
    fn fuse_dynamic_cones(
        &mut self,
        field: &Field,
        pose: Pose2,
        at: f64,
        yaw_rate: f64,
        beams: &[Ray],
    ) {
        let cluttered = self.cluttered_seq.is_some();
        let previous = self.cones.map(|slot| slot.world);
        let mut lidar_clusters = lidar_cone_clusters(beams, pose, field);
        self.echo_tracks.retain(|s| at - s.last_at <= crate::helpers::SCAN_AGE_S);
        lidar_clusters.retain(|c| {
            let repeated = repeated_echo(&mut self.echo_tracks, c.world, pose, at);
            if cluttered && !repeated {
                self.deferred_cone_echoes += 1;
                return false;
            }
            if cluttered {
                self.stable_echo_bins.extend(c.bins);
            }
            true
        });
        let mut used_cluster = vec![false; lidar_clusters.len()];

        // 1. Camera cone extraction + radar pairing (runs once per unique visual frame).
        if let Some((vis, dt_cam)) = self.cam_loc.latest_visual_obs(at)
            && self.last_cone_cam_frame != Some(vis.frame)
        {
            self.last_cone_cam_frame = Some(vis.frame);
            let lens = self.cam_loc.exposure_origin(pose, dt_cam, self.speed, yaw_rate);
            let n_cones = vis.n_cones as usize;
            let has_red = vis.cones[..n_cones]
                .iter()
                .any(|c| c.kind == crate::race_cam_features::ConeKind::RedOrange);

            for cn in &vis.cones[..n_cones] {
                // At the bottom of the search window the floor may have merged
                // into the silhouette. Use its bearing and a unique measured range.
                let clipped = cn.v_bot >= 468.0;
                let projection = if clipped {
                    let mut matches = lidar_clusters.iter().filter(|c| self.cam_loc.cone_silhouette_matches(
                        field, pose, dt_cam, self.speed, yaw_rate, cn, c.world));
                    matches.next().filter(|_| matches.next().is_none())
                        .map(|c| (c.world, c.range_m, c.bearing_rad))
                } else {
                    self.cam_loc.project_cone_to_world(field, pose, dt_cam, self.speed, yaw_rate, cn)
                };
                let Some((cam_w, cam_dist, cam_bearing)) = projection else {
                    continue;
                };
                if !in_open_course(field, cam_w) {
                    continue;
                }
                let (slot_idx, is_red) = match cn.kind {
                    crate::race_cam_features::ConeKind::RedOrange => (0usize, true),
                    crate::race_cam_features::ConeKind::Blue => {
                        let indoor_blue_slot0 = !self.slot0_seen_red
                            && !has_red
                            && (self.cones[0].world.is_some_and(|c0| cam_w.distance(c0) < 0.70)
                                || (self.cones[0].world.is_none()
                                    && pose.x_m < field.walls[4].hi + 0.60
                                    && pose.yaw_rad.abs() < 0.55
                                    && field.cone_priors[0].zip(field.cone_priors[1]).is_some_and(|(a,b)|
                                        cam_w.distance(a) < cam_w.distance(b))));
                        if indoor_blue_slot0 {
                            (0usize, false)
                        } else {
                            (1usize, false)
                        }
                    }
                };
                let red_identity_change = is_red && !self.slot0_seen_red;

                // Try pairing camera cone with a speckle-filtered LiDAR cluster (prefer radar range).
                let mut best_match: Option<(usize, f64)> = None;
                let range_tol = if cluttered { (0.18 + 0.10 * cam_dist).min(0.55) } else { 0.90_f64.max(0.28 * cam_dist) };
                let bearing_tol = if cluttered { 0.10 } else { 0.16 };
                for (ci, lc) in lidar_clusters.iter().enumerate() {
                    if used_cluster[ci] {
                        continue;
                    }
                    let established = &self.cones[slot_idx];
                    if !red_identity_change && established.source != ConeSource::Camera
                        && established.world.is_some_and(|old| old.distance(lc.world) > 0.55)
                    {
                        continue;
                    }
                    let d_ang = crate::py::wrap(lc.bearing_rad - cam_bearing).abs();
                    let d_rng = (lc.range_m - (cam_w.x_m - pose.x_m).hypot(cam_w.y_m - pose.y_m)).abs();
                    if d_ang <= bearing_tol && d_rng <= range_tol {
                        let score = d_ang * 4.0 + d_rng * 0.5;
                        if best_match.is_none_or(|b| score < b.1) {
                            best_match = Some((ci, score));
                        }
                    }
                }

                let other = 1 - slot_idx;
                if let Some((ci, _)) = best_match {
                    let lc = lidar_clusters[ci];
                    if self.cones[other]
                        .world
                        .is_some_and(|ow| lc.world.distance(ow) < MIN_CONE_SEPARATION_M)
                    {
                        continue;
                    }
                    used_cluster[ci] = true;
                    let slot = &mut self.cones[slot_idx];
                    slot.pending_cam = None;
                    slot.world = Some(match slot.world {
                        None => lc.world,
                        Some(old) if slot.source == ConeSource::Camera || red_identity_change => {
                            if old.distance(lc.world) <= 0.45 {
                                Point2 {
                                    x_m: old.x_m * 0.25 + lc.world.x_m * 0.75,
                                    y_m: old.y_m * 0.25 + lc.world.y_m * 0.75,
                                }
                            } else {
                                lc.world
                            }
                        }
                        Some(old) => Point2 {
                            x_m: old.x_m * 0.70 + lc.world.x_m * 0.30,
                            y_m: old.y_m * 0.70 + lc.world.y_m * 0.30,
                        },
                    });
                    self.slot0_seen_red |= is_red;
                    slot.source = ConeSource::Fused;
                    slot.hits = slot.hits.saturating_add(1);
                    slot.last_seen_at = at;
                } else if !clipped {
                    // Camera-only observation (radar dropped, blind, or cone beyond radar range).
                    if self.cones[other]
                        .world
                        .is_some_and(|ow| cam_w.distance(ow) < MIN_CONE_SEPARATION_M)
                    {
                        continue;
                    }
                    let slot = &mut self.cones[slot_idx];
                    // Several silhouettes can belong to the same slot in a single exposure.
                    // Only another capture can add temporal confirmation.
                    if (slot.world.is_some() && slot.last_seen_at == at)
                        || slot.pending_cam.is_some_and(|v| v.at == at)
                    {
                        continue;
                    }
                    let old_has_radar = slot.world.is_some_and(|old|
                        lidar_clusters.iter().any(|c| c.world.distance(old) <= ECHO_REPEAT_M));
                    // The old course drawing supplies a search prior, not an exclusion radius.
                    // A distant monocular hypothesis with poor radar pose support needs a second viewpoint.
                    let far_from_prior = field.cone_priors[slot_idx].is_some_and(|prior|
                        prior.distance(cam_w) > 0.65 + 0.12 * cam_dist);
                    let ambiguous = (cluttered || self.support.iter().any(|&n| n < 4)) && far_from_prior;
                    match slot.world {
                        Some(old) => {
                            let d = old.distance(cam_w);
                            if slot.source == ConeSource::Camera {
                                if d <= 0.85 {
                                    let alpha = if cam_dist < 2.2 {
                                        0.35
                                    } else if cam_dist < 3.6 {
                                        0.24
                                    } else {
                                        0.14
                                    };
                                    slot.world = Some(Point2 {
                                        x_m: old.x_m * (1.0 - alpha) + cam_w.x_m * alpha,
                                        y_m: old.y_m * (1.0 - alpha) + cam_w.y_m * alpha,
                                    });
                                    slot.hits = slot.hits.saturating_add(1);
                                    slot.last_seen_at = at;
                                    slot.pending_cam = None;
                                } else if cam_dist < 3.4 {
                                    // Closer camera view disagrees with earlier far camera estimate:
                                    // update once 2 consistent closer frames confirm the refined spot.
                                    slot.confirm_camera_position(cam_w, lens, at, 0.42, ambiguous);
                                }
                            } else if d <= 0.65 {
                                // Already radar-confirmed: keep accurate radar range, refresh timestamp.
                                if cam_dist < 2.0 && at - slot.last_seen_at > 1.2 {
                                    slot.world = Some(Point2 {
                                        x_m: old.x_m * 0.90 + cam_w.x_m * 0.10,
                                        y_m: old.y_m * 0.90 + cam_w.y_m * 0.10,
                                    });
                                }
                                slot.last_seen_at = at;
                                slot.pending_cam = None;
                            } else if (cluttered && (slot.source == ConeSource::Lidar || !old_has_radar))
                                || (red_identity_change && !old_has_radar) {
                                // Preserve the old plan during disagreement. A consistent second
                                // exposure may replace a radar hypothesis the new scan cannot verify.
                                slot.confirm_camera_position(cam_w, lens, at, 0.42, ambiguous);
                            }
                        }
                        None => {
                            let match_tol = if cam_dist > 3.5 { 0.62 } else { 0.45 };
                            slot.confirm_camera_position(cam_w, lens, at, match_tol, ambiguous);
                        }
                    }
                    if slot.last_seen_at == at {
                        self.slot0_seen_red |= is_red;
                    }
                }
            }
        }

        // 2. LiDAR-only maintenance on already-established dynamic cones (keeps ±2 cm lock
        //    even after the cone exits camera FOV during the 180-deg turn).
        for (ci, lc) in lidar_clusters.iter().enumerate() {
            if used_cluster[ci] {
                continue;
            }
            let mut best_slot: Option<(usize, f64)> = None;
            for slot_idx in 0..2 {
                let Some(existing) = self.cones[slot_idx].world else {
                    continue;
                };
                let gate = if cluttered {
                    if self.cones[slot_idx].source == ConeSource::Camera { 0.30 } else { ECHO_REPEAT_M }
                } else if self.cones[slot_idx].source == ConeSource::Camera {
                    0.55
                } else {
                    0.36
                };
                let d = existing.distance(lc.world);
                if d <= gate && best_slot.is_none_or(|b| d < b.1) {
                    best_slot = Some((slot_idx, d));
                }
            }
            if let Some((slot_idx, _)) = best_slot {
                let other = 1 - slot_idx;
                if self.cones[other]
                    .world
                    .is_some_and(|ow| lc.world.distance(ow) < MIN_CONE_SEPARATION_M)
                {
                    continue;
                }
                let slot = &mut self.cones[slot_idx];
                if let Some(old) = slot.world {
                    if cluttered {
                        // A stationary multi-view range may refine a camera-only centre once.
                        // On subsequent blind scans it may corroborate the point, not walk it.
                        if slot.source == ConeSource::Camera {
                            slot.world = Some(Point2 { x_m: 0.25 * old.x_m + 0.75 * lc.world.x_m,
                                y_m: 0.25 * old.y_m + 0.75 * lc.world.y_m });
                            slot.source = ConeSource::Fused;
                        }
                        if slot.last_seen_at != at {
                            slot.hits = slot.hits.saturating_add(1);
                            slot.last_seen_at = at;
                        }
                        continue;
                    }
                    let alpha = if slot.source == ConeSource::Camera {
                        0.75
                    } else {
                        0.25
                    };
                    slot.world = Some(Point2 {
                        x_m: old.x_m * (1.0 - alpha) + lc.world.x_m * alpha,
                        y_m: old.y_m * (1.0 - alpha) + lc.world.y_m * alpha,
                    });
                    if slot.source == ConeSource::Camera {
                        slot.source = ConeSource::Fused;
                    }
                    slot.hits = slot.hits.saturating_add(1);
                    slot.last_seen_at = at;
                }
            }
        }
        for (slot, old) in self.cones.iter_mut().zip(previous) {
            if slot.world != old {
                slot.last_position_at = Some(at);
            }
        }
    }

    fn current(&self, scan: &V) -> Option<Fix> {
        self.fixes
            .back()
            .copied()
            .filter(|f| Some(f.seq) == scan.get("seq").oi() && Some(f.at) == scan.get("received_at").of())
    }

    fn cone_pose(&self, scan: &V) -> Option<Fix> {
        self.current(scan).or_else(|| self.cone_prediction.filter(|f|
            Some(f.seq) == scan.get("seq").oi() && Some(f.at) == scan.get("received_at").of()))
    }

    pub(super) fn cone_heading(&self, scan: &V) -> Option<f64> {
        Some(self.cone_pose(scan)?.pose.yaw_rad)
    }

    pub fn longitudinal_position(&self, scan: &V) -> Option<f64> {
        Some(self.current(scan)?.pose.x_m)
    }

    pub fn heading(&self, scan: &V) -> Option<f64> {
        Some(self.current(scan)?.pose.yaw_rad)
    }

    pub fn landmark(&self, scan: &V, cone: usize) -> Option<XY> {
        let world = self.cones.get(cone)?.world?;
        let p = self.current(scan)?.pose.world_to_body(world);
        Some((p.x_m, p.y_m))
    }

    pub fn planning_center(&self, scan: &V, cone: usize) -> Option<XY> {
        let slot = self.cones.get(cone)?;
        let world = slot.world.or(FIELD.as_ref()?.cone_priors[cone])?;
        let body = self.cone_pose(scan)?.pose.world_to_body(world);
        let p = (body.x_m, body.y_m);
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

    pub fn finish_divider_ahead(&self, scan: &V) -> Option<f64> {
        let pose = self.current(scan)?.pose;
        let f = FIELD.as_ref()?;
        let divider = &f.walls[5];
        (pose.y_m >= divider.fixed - 0.25 && crate::py::wrap(pose.yaw_rad).abs() < 1.10)
            .then_some(divider.lo - pose.x_m)
    }

    /// Interpolate only observed, timestamped map poses. Both edges and the heading
    /// belong to the exposure time; the returned distance belongs to the current scan.
    pub(crate) fn housing_plane(
        &self,
        scan: &V,
        shot: crate::race_lamp_range::Sighting,
    ) -> Option<crate::race_lamp_range::Plane> {
        let now = self.current(scan)?;
        if !(0.0..=crate::helpers::SCAN_AGE_S).contains(&(now.at - shot.at)) {
            return None;
        }
        let lo = self.fixes.iter().rev().find(|f| f.at <= shot.at)?;
        let hi = self.fixes.iter().find(|f| f.at >= shot.at)?;
        if hi.at - lo.at > crate::helpers::SCAN_AGE_S {
            return None;
        }
        let f = if hi.at > lo.at {
            (shot.at - lo.at) / (hi.at - lo.at)
        } else {
            0.0
        };
        let heading = lo.pose.yaw_rad + f * crate::py::wrap(hi.pose.yaw_rad - lo.pose.yaw_rad);
        let (s, c) = heading.sin_cos();
        let cx = lo.pose.x_m
            + f * (hi.pose.x_m - lo.pose.x_m)
            + crate::race_cam_features::CAM_AHEAD_LIDAR_M * c;
        let cy = lo.pose.y_m
            + f * (hi.pose.y_m - lo.pose.y_m)
            + crate::race_cam_features::CAM_AHEAD_LIDAR_M * s;
        let field = FIELD.as_ref()?;
        let range = shot
            .edges
            .range(field.lamp_heights?, self.cam_loc.pitch_rad)?;
        let (sy, co) = (heading + self.cam_loc.cam_yaw_rad).sin_cos();
        let x = cx + co * range.forward - sy * range.left;
        let y = cy + sy * range.forward + co * range.left;
        let divider = &field.walls[5];
        if !(divider.fixed - 0.12..=field.walls[3].fixed + 0.12).contains(&y)
            || !(field.walls[4].hi..field.walls[1].fixed).contains(&x)
        {
            return None;
        }
        Some(crate::race_lamp_range::Plane {
            ahead: x - now.pose.x_m,
            offset: x - divider.lo,
            uncertainty: range.uncertainty,
            both: range.both,
        })
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

    pub fn body_speed(&self, scan: &V) -> Option<f64> {
        self.current(scan)?;
        Some(self.speed.max(0.))
    }

    pub fn two_measured_axes(&self, scan: &V) -> bool {
        self.current(scan).is_some()
            && (0..2).all(|a| self.support[a] >= 4 || self.cam_loc.last_cam_anchored[a])
    }

    /// For longitudinal decisions a carried map coordinate isn't a fresh wall fix.
    pub fn motion_supported(&self, scan: &V) -> bool {
        self.two_measured_axes(scan) && self.current(scan).zip(self.prediction).is_some_and(|(f,p)|
            (f.pose.x_m-p.x_m).hypot(f.pose.y_m-p.y_m) <= crate::maneuver::LIGHT_LINE_JUMP_M
            && crate::py::wrap(f.pose.yaw_rad-p.yaw_rad).abs() <= crate::maneuver::open_lane_entry_heading_rad())
    }

    pub fn missing_fraction(&self) -> f64 {
        1. - self.valid_rays as f64 / 360.
    }

    /// Compare the two already-computed steering commands against surveyed
    /// boards and observed dynamic cones after the measured steering response.
    /// Reuse the live arc test. Selects steering only; neither sets speed nor updates clearance.
    pub fn map_arc_has_more_room(&self, scan: &V, live: i64, mapped: i64) -> bool {
        if live == mapped {
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
        let eval_pose = |eval_p: Pose2, pwm: i64| -> f64 {
            let mut pts = Vec::new();
            for wall in &field.walls {
                let (perpendicular, along) = if wall.vertical {
                    (eval_p.x_m, eval_p.y_m)
                } else {
                    (eval_p.y_m, eval_p.x_m)
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
                    let q = eval_p.world_to_body(Point2 { x_m, y_m });
                    pts.push((q.x_m, q.y_m));
                }
            }
            for slot in &self.cones {
                let Some(cw) = slot.world else { continue };
                if (cw.x_m - eval_p.x_m).hypot(cw.y_m - eval_p.y_m) > limit + 0.15 {
                    continue;
                }
                for k in 0..16 {
                    let ang = k as f64 * (std::f64::consts::TAU / 16.0);
                    let q = eval_p.world_to_body(Point2 {
                        x_m: cw.x_m + 0.10 * ang.cos(),
                        y_m: cw.y_m + 0.10 * ang.sin(),
                    });
                    pts.push((q.x_m, q.y_m));
                }
            }
            let nom = crate::maneuver::Maneuver::path_free_distance(&pts, pwm);
            if pwm > crate::helpers::NEUTRAL + 40 {
                let tight = crate::helpers::NEUTRAL + ((pwm - crate::helpers::NEUTRAL) * 15) / 10;
                nom.min(crate::maneuver::Maneuver::path_free_distance(&pts, tight))
            } else {
                nom
            }
        };
        let room_of = |pwm: i64| eval_pose(fix.pose, pwm).min(eval_pose(pose, pwm));
        let room = room_of(live);
        room < reach && room_of(mapped) > room + 0.05
    }

    /// True when two-axis localization (LiDAR or camera+IMU) has remained physically
    /// stationary within 2.5 cm / 0.025 rad over at least `min_s` seconds.
    pub fn stationary_for(&self, scan: &V, min_s: f64) -> bool {
        let Some(now_fix) = self.current(scan) else {
            return false;
        };
        if !self.two_measured_axes(scan)
            || self.speed.abs() > 0.05
            || self.yaw_rate.is_some_and(|w| w.abs() > 0.04)
        {
            return false;
        }
        let Some(first) = self.fixes.front() else {
            return false;
        };
        if now_fix.at - first.at < min_s || self.fixes.len() < 4 {
            return false;
        }
        self.fixes.iter().all(|f| {
            (f.pose.x_m - now_fix.pose.x_m).hypot(f.pose.y_m - now_fix.pose.y_m) <= 0.025
                && crate::py::wrap(f.pose.yaw_rad - now_fix.pose.yaw_rad).abs() <= 0.025
        })
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
        let field = FIELD.as_ref()?;
        let seq = old.get("source_seq").oi()?;
        let old_pose = self.fixes.iter().rev().find(|f| f.seq == seq)?.pose;
        let point = old.get("point_left_m").ol()?;
        let world = old_pose.body_to_world(Point2 {
            x_m: point.first()?.of()?,
            y_m: point.get(1)?.of()?,
        });
        if !in_open_course(field, world) {
            return None;
        }
        let other = 1 - cone;
        if self.cones[other]
            .world
            .is_some_and(|ow| world.distance(ow) < MIN_CONE_SEPARATION_M)
        {
            return None;
        }
        if let Some(known) = self.cones[cone].world
            && world.distance(known) > 0.75
        {
            return None;
        }
        let p = self.current(scan)?.pose.world_to_body(world);
        Some((p.x_m, p.y_m))
    }

    pub fn observe_cone(&mut self, scan: &V, cone: usize, center: XY) {
        if cone >= 2 {
            return;
        }
        let Some(fix) = self.current(scan) else { return };
        let Some(field) = FIELD.as_ref() else { return };
        if !self.two_measured_axes(scan) {
            return;
        }
        let w = fix.pose.body_to_world(Point2 {
            x_m: center.0,
            y_m: center.1,
        });
        if !in_open_course(field, w) {
            return;
        }
        if self.cluttered_seq == Some(fix.seq) {
            if !self.echo_tracks.iter().any(|s| s.last_at == fix.at && s.scans >= 3 && s.separate_views
                && s.origin.distance(w) <= ECHO_REPEAT_M + 0.07) {
                return;
            }
            if self.cones[cone].world.is_some() {
                // Map maintenance above already checked this revolution.
                return;
            }
        }
        let other = 1 - cone;
        if self.cones[other]
            .world
            .is_some_and(|ow| w.distance(ow) < MIN_CONE_SEPARATION_M)
        {
            return;
        }
        let slot = &mut self.cones[cone];
        let old = slot.world;
        match slot.world {
            None => {
                slot.world = Some(w);
                slot.source = ConeSource::Lidar;
                slot.hits = 1;
                slot.last_seen_at = fix.at;
                slot.pending_cam = None;
            }
            Some(p) => {
                let d = p.distance(w);
                if slot.source == ConeSource::Camera && d <= 1.15 {
                    slot.world = Some(if d <= 0.40 {
                        Point2 {
                            x_m: p.x_m * 0.30 + w.x_m * 0.70,
                            y_m: p.y_m * 0.30 + w.y_m * 0.70,
                        }
                    } else {
                        w
                    });
                    slot.source = ConeSource::Fused;
                    slot.hits = slot.hits.saturating_add(1);
                    slot.last_seen_at = fix.at;
                    slot.pending_cam = None;
                } else if d <= 0.65 {
                    slot.world = Some(Point2 {
                        x_m: p.x_m * 0.70 + w.x_m * 0.30,
                        y_m: p.y_m * 0.70 + w.y_m * 0.30,
                    });
                    slot.hits = slot.hits.saturating_add(1);
                    slot.last_seen_at = fix.at;
                }
            }
        }
        if slot.world != old {
            slot.last_position_at = Some(fix.at);
        }
        // Keep planner synced immediately if a new cone was discovered on this scan.
        self.planner.observe(
            fix.seq,
            fix.pose,
            self.speed,
            self.yaw_rate.unwrap_or(0.),
            self.path_cones(field),
        );
    }

    pub fn diagnostics(&self, scan: &V) -> V {
        let fix = self.current(scan);
        let cone_pose = self.cone_pose(scan);
        let preview = |side| cone_pose.and_then(|f| self.planner.aim(f.seq, side)).map(|a| a.pwm);
        let obs_cones = V::list(
            self.cones
                .iter()
                .map(|c| match c.world {
                    Some(p) => V::from(vec![p.x_m, p.y_m]),
                    None => V::None,
                })
                .collect(),
        );
        let cone_sources = V::list(
            self.cones
                .iter()
                .map(|c| V::from(c.source.as_str()))
                .collect(),
        );
        crate::dict! {"source_seq"=>self.seq, "localized"=>fix.is_some(),
            "imu_prediction_xy_yaw"=>self.prediction.map(|p|vec![p.x_m,p.y_m,p.yaw_rad]),
            "short_board_heading_observed"=>self.fragment_fix,
            "gyro_replaced_from_radar"=>self.imu.gyro_replaced,"accel_replaced_from_radar"=>self.imu.accel_replaced,
            "corrected_yaw_rate"=>self.imu.current().yaw_rate,"radar_yaw_rate"=>self.imu.radar.map(|r|r.yaw),
        "pose_xy_yaw"=>fix.map(|f| vec![f.pose.x_m,f.pose.y_m,f.pose.yaw_rad]),
        "wall_points_xy"=>vec![self.support[0] as i64,self.support[1] as i64],
        "mean_wall_residual_m"=>self.mean_error,"speed_mps"=>self.speed,"measured_cone_fix"=>self.cone_fix,
        "preview_pwm"=>V::list(vec![V::from(preview(0)),V::from(preview(1))]),
        "observed_cones_world"=>obs_cones,
        "cone_observation_age_s"=>V::list(self.cones.iter()
            .map(|slot| V::from(slot.world.and(self.at).map(|at| at - slot.last_seen_at))).collect()),
        "inertial_cone_preview"=>fix.is_none() && cone_pose.is_some(),
        "cone_guidance_sources"=>V::list((0..2).map(|i|V::from(self.cone_guidance_source(i))).collect()),
        "cone_radar_cluttered"=>self.cluttered_seq.is_some(), "deferred_cone_echoes"=>self.deferred_cone_echoes,
        "guided_cones_world"=>FIELD.as_ref().map(|field| V::list(self.guidance_world(field).iter()
            .map(|p| V::from(p.map(|v| vec![v.x_m,v.y_m]))).collect())),
        "cone_sources"=>cone_sources,
        "cam_used"=>self.cam_loc.last_used_cam,
        "cam_anchored_xy"=>V::list(vec![V::from(self.cam_loc.last_cam_anchored[0]),V::from(self.cam_loc.last_cam_anchored[1])]),
        "cam_inliers"=>self.cam_loc.last_inliers as i64,
        "cam_rms_px"=>self.cam_loc.last_pixel_rms,
        "cam_solve_us"=>self.cam_loc.last_solve_us as i64,
        "cam_pitch_deg"=>self.cam_loc.pitch_rad.to_degrees(),
        "cam_yaw_deg"=>self.cam_loc.cam_yaw_rad.to_degrees(),
        "cam_rail_h_m"=>self.cam_loc.rail_height_m,
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
    fn surveyed_brake_registers_a_measured_light_stop_without_colour() {
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

        // The moving vehicle stays on its single brake even after entering the zone.
        let brake = &mut approach.race_light_brake;
        for i in 0..=4 {
            let dt = i as f64 / 10.;
            let speed = 1.30 - 1.875 * dt;
            let gap = 1.43 - 1.30 * dt + 0.9375 * dt * dt;
            let integral = Some(speed - 1.30);
            brake.observe(1. + dt, Some(gap), Some(speed), integral, 1., 0.35, BrakeModel::light(1., 0.35), 0.55);
            assert_eq!(brake.command(1. + dt, integral), Some(1300));
        }
        for t in [1.8, 1.92] {
            brake.observe(t, Some(0.55), Some(0.), Some(-1.3), 1., 0.35, BrakeModel::light(1., 0.35), 0.55);
            brake.command(t, Some(-1.3));
        }
        assert_eq!(brake.diagnostics().get("end_reason").os(), Some("stopped"));

        let scan = |at: f64| crate::dict! { "seq" => (at * 10.).round() as i64, "at_ms" => (at * 1000.).round() as i64,
            "received_at" => at, "ranges" => vec![V::Float(3.); 360] };
        let at = 3.;
        for (gap, speed, should_wait, should_advance) in [
            (0.55,0.45,false,false), (0.55,0.0,true,false),
            (-0.02,0.0,false,false), (0.85,0.0,false,true),
        ] {
            let mut m = approach.clone();
            m.race_light.beam_at=Some(at-1.);
            m.race_light.opening_source=Some("surveyed_finish_opening");
            let ahead=gap+FRONT_BODY_EXTENT_M;
            let mut stop=None;
            for i in 0..7 {
                let t=at+0.1*f64::from(i);
                m.compass.heading=Some(0.);m.compass.at_s=Some(t);m.compass.measured_at_s=Some(t);
                m.race_light.ranges=vec![(t-0.2,t-0.2,ahead+0.2*speed),(t-0.1,t-0.1,ahead+0.1*speed)];
                let (basis,fb)=m.light_approach_control(&scan(t),Some(ahead),t,&crate::dict!{"motor"=>1500,"command_acked"=>true});
                if let Some(b)=basis { m.lane_feedback=fb;m.light_stop_reached(&scan(t),t,&V::None,&b);stop=Some(t);break; }
            }
            assert_eq!(stop.is_some(),should_wait,"gap={gap} speed={speed}");
            assert_eq!(m.race_light.completing_zone_stop,should_advance);
            if should_wait {
                assert!(!m.light_wait(&scan(at-1.+WAIT-0.1),at-1.+WAIT-0.1,&V::None));
                assert!(m.light_wait(&scan(at-1.+WAIT),at-1.+WAIT,&V::None));
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
        for n in 1..=13 {
            let ms = 11188 + 100 * n;
            let at = ms as f64 / 1000.;
            let scan = crate::dict! {"seq"=>(n+2) as i64,"at_ms"=>ms as i64,"received_at"=>at,"ranges"=>blank.clone()};
            locator.observe(&scan, at, imu, false);
            assert!(!locator.two_measured_axes(&scan));
            assert_eq!(locator.heading(&scan).is_some(), n <= 6);
            assert_eq!(locator.cone_heading(&scan).is_some(), n <= 12);
            assert_eq!(locator.planner.aim(n + 2, 0).is_some(), n <= 12);
            assert!(!locator.camera_localized(&scan));
            assert!(!locator.stationary_for(&scan, 0.5));
        }
    }

    #[test]
    fn cone_priors_guide_steering_until_observed_and_accept_shifted_positions() {
        let field = FIELD.as_ref().unwrap();
        let pose = Pose2 {
            x_m: 3.40,
            y_m: 0.48,
            yaw_rad: 0.0,
        };
        let ranges = TABLES.with(|table| {
            V::list(
                table
                    .directions
                    .iter()
                    .map(|&(x, y)| V::from(field.hit(pose.x_m, pose.y_m, x, y).map(|(_, range)| range)))
                    .collect(),
            )
        });
        let mut loc = Locator::default();
        loc.force_state_for_eval(1, 1.0, pose, 0.0, Some(0.0), Some(0.0));
        let imu = RaceInputs {
            yaw_rate: Some(0.0),
            forward_integral: Some(0.0),
        };
        let scan2 = crate::dict! {"seq" => 2i64, "at_ms" => 1100i64, "received_at" => 1.1, "ranges" => ranges.clone()};
        loc.observe(&scan2, 1.1, imu, false);
        // The nominal path is immediately available without manufacturing a sensor hit.
        assert!(loc.landmark(&scan2, 0).is_none());
        assert!(loc.landmark(&scan2, 1).is_none());
        assert!(loc.planning_center(&scan2, 0).is_some());
        assert!(loc.planner.aim(2, 0).is_some());
        assert!(loc.planner.aim(2, 1).is_some());
        assert_eq!(loc.cone_guidance_source(0), "nominal_prior");
        assert!(loc.prior(&scan2, &V::None, 0).center.is_none());
        assert!(!loc.cone_fix);

        // Observe Cone 0 shifted >0.85 m away from old nominal (5.8, 2.0) -> at (5.15, 1.40).
        let shifted_c0 = Point2 { x_m: 5.15, y_m: 1.40 };
        let body_c0 = pose.world_to_body(shifted_c0);
        loc.observe_cone(&scan2, 0, (body_c0.x_m, body_c0.y_m));
        let got0 = loc.observed_cone_world(0).expect("cone 0 stored");
        assert!(got0.distance(shifted_c0) < 0.01);
        assert_eq!(loc.observed_cone_source(0), ConeSource::Lidar);
        assert!(loc.landmark(&scan2, 0).is_some());
        assert!(loc.planner.aim(2, 0).is_some());
        // Cone 1 still uses its own nominal preview.
        assert!(loc.landmark(&scan2, 1).is_none());
        assert!(loc.planner.aim(2, 1).is_some());

        // Anti-swap: trying to register Cone 1 at Cone 0's spot must be rejected.
        loc.observe_cone(&scan2, 1, (body_c0.x_m, body_c0.y_m));
        assert!(loc.observed_cone_world(1).is_none());

        // Observe Cone 1 shifted >0.80 m away from old nominal (2.2, 2.0) -> at (2.85, 2.55).
        let shifted_c1 = Point2 { x_m: 2.85, y_m: 2.55 };
        let body_c1 = pose.world_to_body(shifted_c1);
        loc.observe_cone(&scan2, 1, (body_c1.x_m, body_c1.y_m));
        let got1 = loc.observed_cone_world(1).expect("cone 1 stored");
        assert!(got1.distance(shifted_c1) < 0.01);
        assert!(loc.planner.aim(2, 1).is_some());
    }
}

#[cfg(test)]
mod independent_cone_evidence_review {
    use super::*;
    use crate::race_cam_features::{CAM_HEIGHT_M, ConeKind, ConeObs, NOMINAL_PITCH_RAD, VisualFrame};

    fn blue(left: f64) -> ConeObs {
        let (sp, cp) = NOMINAL_PITCH_RAD.sin_cos();
        let depth = 1.6 * cp - CAM_HEIGHT_M * sp;
        ConeObs {
            kind: ConeKind::Blue,
            xn: (-left / depth) as f32,
            yn: ((1.6 * sp + CAM_HEIGHT_M * cp) / depth) as f32,
            width_px: 55.0,
            height_px: 120.0,
            ..Default::default()
        }
    }

    fn observe(loc: &mut Locator, frame: i64, at: f64, observations: &[ConeObs], rays: &[Ray]) {
        let pose = Pose2 { x_m: 4.0, y_m: 1.7, yaw_rad: 0.0 };
        let mut vis = VisualFrame { frame, captured_mono_s: at, n_cones: observations.len() as u8, ..Default::default() };
        vis.cones[..observations.len()].copy_from_slice(observations);
        loc.observe_camera(vis, at);
        loc.fuse_dynamic_cones(FIELD.as_ref().unwrap(), pose, at, 0.0, rays);
    }

    #[test]
    fn camera_confirmation_needs_two_different_exposures() {
        let mut loc = Locator::default();
        loc.support = [8, 8];
        observe(&mut loc, 48, 13.692, &[blue(-0.10), blue(0.10)], &[]);
        assert!(loc.cones[1].world.is_none(), "two nearby silhouettes from frame 48 confirmed a single blue cone");
        observe(&mut loc, 49, 13.792, &[blue(0.0)], &[]);
        assert_eq!(loc.cones[1].source, ConeSource::Camera);
        assert!(loc.cones[1].world.is_some());
    }

    #[test]
    fn distant_radar_pair_does_not_refresh_an_unrelated_cone() {
        let mut loc = Locator::default();
        let old = Point2 { x_m: 2.6, y_m: 1.7 };
        loc.cones[1] = DynamicConeSlot { world: Some(old), source: ConeSource::Lidar, hits: 3, last_seen_at: 1.0, ..Default::default() };
        let rays: [Ray; 3] = std::array::from_fn(|bin| {
            let a = -(bin as f64).to_radians();
            Ray { bin, range: 1.77, dx: a.cos(), dy: a.sin(), ox: 0.0, oy: 0.0 }
        });
        observe(&mut loc, 136, 1.2, &[blue(0.0)], &rays);
        assert_eq!(loc.cones[1].world, Some(old));
        assert_eq!(loc.cones[1].last_seen_at, 1.0, "rejected measurement refreshed the old cone");
        assert_eq!(loc.cones[1].hits, 3);
        assert_eq!(loc.cones[1].source, ConeSource::Lidar);
    }

    fn radar_bundle(pose: Pose2, world: Point2) -> Vec<Ray> {
        let p = pose.world_to_body(world);
        let mid = (-p.y_m.atan2(p.x_m).to_degrees()).round() as i32;
        let mut rays: Vec<_> = (-2..=2).map(|off| {
            let bin = (mid + off).rem_euclid(360) as usize;
            let angle = -(bin as f64).to_radians();
            Ray { bin, range: p.x_m.hypot(p.y_m) - 0.08, dx: angle.cos(), dy: angle.sin(), ox: 0.0, oy: 0.0 }
        }).collect();
        rays.sort_unstable_by_key(|r| r.bin);
        rays
    }

    #[test]
    fn noisy_cone_stays_in_the_field_frame_while_car_turns_and_camera_loses_it() {
        let field = FIELD.as_ref().unwrap();
        let mut loc = Locator::default();
        loc.support = [8, 8];
        observe(&mut loc, 1, 1.0, &[blue(0.0)], &[]);
        observe(&mut loc, 2, 1.1, &[blue(0.0)], &[]);
        let known = loc.cones[1].world.unwrap();
        for i in 0..6 {
            let seq = 3 + i;
            let at = 1.2 + i as f64 * 0.1;
            let pose = Pose2 { x_m: 4.0 + 0.16 * i as f64, y_m: 1.7, yaw_rad: 0.13 * i as f64 };
            // A glint in a constant sensor bin first appears on top of the real cone.
            let moving = pose.body_to_world(Point2 {x_m: 1.85, y_m: 0.0});
            let noise = radar_bundle(pose, moving);
            loc.at = Some(at);
            loc.cluttered_seq = Some(seq);
            loc.stable_echo_bins = ConeReturnBins::default();
            loc.fixes.push_back(Fix {seq, at, pose});
            loc.support = [8, 8];
            loc.fuse_dynamic_cones(field, pose, at, 1.3, &noise);
            let scan = crate::dict! {"seq"=>seq, "received_at"=>at};
            let ghost = pose.world_to_body(moving);
            // The live tracker cannot bypass fusion and write the moving echo back.
            loc.observe_cone(&scan, 1, (ghost.x_m, ghost.y_m));
            assert_eq!(loc.cones[1].world, Some(known));
            assert_eq!(loc.cones[1].last_seen_at, 1.1);
            assert_eq!(loc.cone_guidance_source(1), "carried_observation");
            let p = pose.world_to_body(known);
            assert_eq!(loc.planning_center(&scan, 1), Some((p.x_m, p.y_m)));
            let target = crate::dict! {"support_bins"=>V::list(noise.iter().map(|r| V::from(r.bin)).collect()),"point_left_m"=>(ghost.x_m,ghost.y_m)};
            assert!(!loc.prior(&scan,&V::None,1).accepts_echo(&target));
        }
    }

    #[test]
    fn noisy_cone_stable_across_motion_can_be_acquired_off_prior_by_lidar() {
        let field = FIELD.as_ref().unwrap();
        let mut loc = Locator::default();
        let moved = Point2 { x_m: 5.1, y_m: 1.4 };
        for seq in 1..=4 {
            let at = 1.0 + seq as f64 * 0.1;
            let pose = Pose2 { x_m: 3.5 + 0.13 * seq as f64, y_m: 1.2, yaw_rad: 0.07 * seq as f64 };
            let scan = crate::dict! {"seq"=>seq, "received_at"=>at};
            let rays = radar_bundle(pose, moved);
            loc.cluttered_seq = Some(seq);
            loc.stable_echo_bins = ConeReturnBins::default();
            loc.fixes.push_back(Fix {seq, at, pose});
            loc.support = [8, 8];
            loc.fuse_dynamic_cones(field, pose, at, 0.7, &rays);
            let body = pose.world_to_body(moved);
            let target = crate::dict! {"support_bins"=>V::list(rays.iter().map(|r| V::from(r.bin)).collect()),"point_left_m"=>(body.x_m,body.y_m)};
            assert_eq!(loc.prior(&scan,&V::None,0).accepts_echo(&target), seq >= 3);
            loc.observe_cone(&scan, 0, (body.x_m, body.y_m));
            assert_eq!(loc.cones[0].world.is_some(), seq >= 3);
        }
        assert!(loc.cones[0].world.unwrap().distance(moved)<0.01);
    }

    #[test]
    fn two_camera_exposures_can_correct_a_conflicting_lidar_only_identity() {
        let mut loc = Locator::default();
        let pose = Pose2 {x_m: 4.0, y_m: 1.7, yaw_rad: 0.0};
        let wrong = Point2 {x_m: 4.85, y_m: 2.35};
        loc.cones[0] = DynamicConeSlot {world: Some(wrong), source: ConeSource::Lidar, hits: 4, last_seen_at: 0.8, ..Default::default()};
        loc.slot0_seen_red = true;
        let obstacle = radar_bundle(pose, wrong);
        let optical = [ConeObs {kind: ConeKind::RedOrange, ..blue(0.0)}];
        for i in 0..4 {
            loc.cluttered_seq = Some(i);
            observe(&mut loc, i, 1.0 + i as f64 * 0.1, if i < 2 { &[] } else { &optical }, &obstacle);
            if i < 3 { assert_eq!(loc.cones[0].world,Some(wrong)); }
        }
        assert_eq!(loc.cones[0].source,ConeSource::Camera);
        assert!(loc.cones[0].world.unwrap().distance(wrong) > 0.8);
    }

    #[test]
    fn distant_camera_only_cone_can_replace_a_prior_after_a_change_of_view() {
        let mut loc = Locator::default();
        let cone = blue(0.0);
        for (frame, at) in [(1, 1.0), (2, 1.1), (3, 1.2)] {
            observe(&mut loc, frame, at, &[cone], &[]);
            assert!(loc.cones[1].world.is_none());
        }
        let pose = Pose2 { x_m: 4.0, y_m: 1.95, yaw_rad: 0.0 };
        let mut vis = VisualFrame { frame: 4, n_cones: 1, captured_mono_s: 1.3, ..Default::default() };
        vis.cones[0] = blue(-0.25);
        loc.observe_camera(vis, 1.3);
        loc.fuse_dynamic_cones(FIELD.as_ref().unwrap(), pose, 1.3, 0.0, &[]);
        assert_eq!(loc.cones[1].source, ConeSource::Camera);
        assert!(loc.cones[1].world.unwrap().distance(FIELD.as_ref().unwrap().cone_priors[1].unwrap()) > 2.0);
    }

    #[test]
    fn clipped_blue_silhouette_alone_has_no_ground_range() {
        let mut loc = Locator::default();
        let cn = ConeObs {v_bot: 473.0, ..blue(0.0)};
        for (frame,at) in [(48,1.0),(49,1.1),(50,1.2)] {
            observe(&mut loc,frame,at,&[cn],&[]);
        }
        assert!(loc.cones.iter().all(|s| s.world.is_none()));
        assert_eq!(loc.cone_guidance_source(1),"nominal_prior");
    }

    #[test]
    fn noisy_cone_echo_needs_relative_motion_not_just_three_timestamps() {
        let world = Point2 { x_m: 5.4, y_m: 1.7 };
        let pose = Pose2 { x_m: 4.0, y_m: 1.7, yaw_rad: 0.0 };
        let mut tracks = Vec::new();
        for at in [1.0, 1.1, 1.2, 1.3] {
            assert!(!repeated_echo(&mut tracks, world, pose, at));
        }
        assert!(repeated_echo(&mut tracks, world, Pose2 { x_m: 4.25, ..pose }, 1.4));
    }

    #[test]
    fn clipped_cone_can_use_one_matching_radar_range() {
        use crate::race_cam_features::{CAM_AHEAD_LIDAR_M, distort_norm};
        let mut loc = Locator::default();
        loc.slot0_seen_red = true;
        let pose = Pose2 { x_m: 4.0, y_m: 1.7, yaw_rad: 0.0 };
        let world = Point2 { x_m: 5.8, y_m: 1.7 };
        let (s, c) = NOMINAL_PITCH_RAD.sin_cos();
        let fwd = world.x_m - pose.x_m - CAM_AHEAD_LIDAR_M;
        let z = 0.48 - CAM_HEIGHT_M;
        let (u, v) = distort_norm(0.0, (s * fwd - c * z) / (c * fwd + s * z));
        let cn = ConeObs {u: u as f32, v_top: v as f32, v_bot: 473.0, ..blue(0.0)};
        observe(&mut loc, 48, 1.0, &[cn], &radar_bundle(pose, world));
        assert_eq!(loc.cones[1].source, ConeSource::Fused);
        assert!(loc.cones[1].world.unwrap().distance(world) < 0.04);
    }
}
