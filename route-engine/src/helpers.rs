//! Module-level functions and small classes used by the route controller:
//! autonomy_live (corridor_walls, CorridorSteering, side_wall_fit),
//! turn_motion (wall/opening/candidate validation, surface supports) and
//! maneuver_sequence (compass, start-lane wall law, loop planning, pursuit,
//! axis lines). Same names and arithmetic as the Python originals.

use std::collections::BTreeSet;

use crate::dict;
use crate::py::*;

// ---------------------------------------------------------------- constants

pub const NEUTRAL: i64 = 1500;
pub const TRIAL_MOTOR: i64 = 1560;
pub const SERVO_MIN: i64 = 1270;
pub const SERVO_MAX: i64 = 1720;
pub const INITIAL_PRESTEER_MIN: i64 = 1650;
pub const MAX_DRIVE_S: f64 = 10.0;
pub const MAX_PRESTEER_S: f64 = 5.0;
pub const STEERING_ALLOWANCE_S: f64 = 1.2;
pub const PWM_STEP: i64 = 10;
pub const PWM_INTERVAL_S: f64 = 0.10;
pub const SCAN_AGE_S: f64 = 0.30;
pub const COAST_SERVO_DELTA: i64 = 55;
pub const FRAME: &str = "lidar_origin_coarse_body_heading";

// autonomy_live
pub const MIN_FORWARD_PWM: i64 = 1550;
pub const LIDAR_LATE_S: f64 = 0.30;
pub const CORRIDOR_LOOKAHEAD_M: f64 = 1.0;
pub const SIDE_BODY_EXTENT_M: f64 = 0.14;
pub const SIDE_MIN_NET_M: f64 = 0.10;
pub const LIDAR_RANGE_ALLOWANCE_M: f64 = 0.03;
pub const SIDE_CLEARANCE_M: f64 = SIDE_BODY_EXTENT_M + SIDE_MIN_NET_M + LIDAR_RANGE_ALLOWANCE_M;
pub const FRONT_BODY_EXTENT_M: f64 = 0.20;
pub const REAR_BODY_EXTENT_M: f64 = 0.18;
pub const MANEUVER_MIN_NET_M: f64 = 0.05;
pub const MANEUVER_BODY_CLEARANCE_M: f64 = MANEUVER_MIN_NET_M + LIDAR_RANGE_ALLOWANCE_M;
pub const STEERING_LIMIT_PWM: i64 = 55;
pub const SIDE_WALL_REACH_M: f64 = 6.;
pub const SIDE_WALL_INLIER_M: f64 = 0.03;
pub const SIDE_WALL_MIN_POINTS: usize = 30;
pub const SIDE_WALL_MIN_FRACTION: f64 = 0.6;
pub const SIDE_WALL_MIN_SPAN_M: f64 = 1.;
pub const SIDE_WALL_LATERAL_M: (f64, f64) = (0.15, 1.5);
pub const TURN_LATERAL_MARGIN_M: f64 = SIDE_BODY_EXTENT_M + MANEUVER_BODY_CLEARANCE_M;

// steer_calibration
pub const STEER_PLAY_PWM: i64 = 40;
pub fn nominal_right_limit_pwm() -> i64 {
    NEUTRAL + ceil(-1.2 * 160.)
}

// maneuver_sequence (module level)
pub const OPEN_PROBE_REACH_M: f64 = 3.;
pub const OPEN_PROBE_HALF_WIDTH_M: f64 = SIDE_BODY_EXTENT_M + 0.10;
pub const OPEN_INNER_GAP_M: f64 = 0.15;
pub const OPEN_OUTER_GAP_M: f64 = 0.15;
pub const OPEN_RADIUS_STEP_M: f64 = 0.05;
pub const OPEN_RADIUS_PREFERENCE: f64 = 0.10;
pub const OPEN_APPROACH_SLOPE: f64 = 1.5;
pub const OPEN_APPROACH_CONVERGE_RATIO: f64 = 2.5;
pub const OPEN_APPROACH_CONVERGE_CAP: f64 = 0.5;
pub const OPEN_APPROACH_MARGIN_M: f64 = 0.05;
pub const ROUTE_PWM_PER_CURVATURE: f64 = 160.;
pub const OPEN_LINE_BAND_M: f64 = 0.06;
pub const OPEN_LINE_MIN_POINTS: usize = 8;
pub const OPEN_LINE_GAP_M: f64 = 0.3;
pub const OPEN_LINE_GAP_SCALE: f64 = 0.03;
pub const OPEN_LANE_MIN_SPAN_M: f64 = 0.5;
pub const OPEN_BOARD_END_LATERAL_M: f64 = 0.15;
pub const OPEN_BOARD_END_ALONG_M: f64 = 0.5;
pub const MAX_DIAMETER_M: f64 = 0.35;
pub const COMPASS_MIN_SPAN_M: f64 = 0.5;
pub const COMPASS_MAX_FIT_M: f64 = 0.04;
pub const COMPASS_MIN_POINTS: i64 = 20;
pub const COMPASS_MIN_SUPPORT_M: f64 = 1.;
pub const COMPASS_MAX_RATE_RAD_S: f64 = 2.5;
pub const COMPASS_MAX_PREDICT_S: f64 = 0.3;
pub const COMPASS_RATE_WINDOW_S: (f64, f64) = (0.05, 0.5);
pub const COMPASS_MAX_STALE_S: f64 = 1.;
pub const LANE_WALL_LOOKAHEAD_M: f64 = 1.2;
pub const LANE_WALL_LEAD_S: f64 = 0.45;
pub const LANE_WALL_GAIN_PWM_PER_RAD: f64 = 100.;
pub const LANE_WALL_CAP_PWM: i64 = 33;
pub const LANE_WALL_HISTORY: usize = 3;
pub const LANE_WALL_GAP_S: f64 = 0.35;

pub fn compass_gate_rad() -> f64 {
    radians(30.)
}

/// cos/sin of each degree (maneuver_sequence._DIRECTION_COS/_SIN,
/// autonomy_live._BIN_COS/_BIN_SIN: the same expressions).
pub struct Trig {
    pub cos: Vec<f64>,
    pub sin: Vec<f64>,
}

thread_local! {
    pub static TRIG: Trig = Trig {
        cos: (0..360).map(|i| radians(i as f64).pcos()).collect(),
        sin: (0..360).map(|i| radians(i as f64).psin()).collect(),
    };
}

pub fn dcos(i: usize) -> f64 {
    TRIG.with(|t| t.cos[i])
}

pub fn dsin(i: usize) -> f64 {
    TRIG.with(|t| t.sin[i])
}

pub fn get_or<'a>(d: &'a V, key: &str, default: &'a V) -> &'a V {
    if d.has(key) { d.get(key) } else { default }
}

// ---------------------------------------------------------------- autonomy_live

/// corridor_walls(scan): two extended parallel boards.
pub fn corridor_walls(scan: &V) -> V {
    let bins = scan.idx("ranges");
    let list = bins.l();
    let mut fits = Vec::new();
    for sector in [45..136usize, 225..316] {
        let mut points: Vec<(f64, f64)> = Vec::new();
        for a in sector {
            let r = &list[a];
            if r.is_some() {
                let r = r.f();
                let angle = radians(a as f64);
                points.push((r * angle.pcos(), r * angle.psin()));
            }
        }
        if points.len() < 40 {
            return V::None;
        }
        let mut slopes = Vec::new();
        for (i, &(x1, y1)) in points.iter().enumerate() {
            let mut j = i + 8;
            while j < points.len() {
                if (points[j].0 - x1).abs() >= 0.3 {
                    slopes.push(div(points[j].1 - y1, points[j].0 - x1));
                }
                j += 8;
            }
        }
        if slopes.is_empty() {
            return V::None;
        }
        let slope = median(slopes);
        let intercept = median(points.iter().map(|&(x, y)| y - slope * x).collect());
        let inliers: Vec<(f64, f64)> =
            points.iter().cloned().filter(|&(x, y)| (y - slope * x - intercept).abs() <= 0.025).collect();
        if (inliers.len() as f64) < 0.80 * points.len() as f64 {
            return V::None;
        }
        let xmax = inliers.iter().map(|p| p.0).fold(f64::NEG_INFINITY, pmax);
        let xmin = inliers.iter().map(|p| p.0).fold(f64::INFINITY, pmin);
        if xmax - xmin < 0.25 {
            return V::None;
        }
        let xs: Vec<f64> = inliers.iter().map(|p| p.0).collect();
        let ys: Vec<f64> = inliers.iter().map(|p| p.1).collect();
        let mean_x = mean(&xs);
        let mean_y = mean(&ys);
        let variance = fsum_py(inliers.iter().map(|&(x, _)| sq(x - mean_x)));
        let slope = div(fsum_py(inliers.iter().map(|&(x, y)| (x - mean_x) * (y - mean_y))), variance);
        let intercept = mean_y - slope * mean_x;
        if slope.abs() > radians(15.).tan() {
            return V::None;
        }
        fits.push((slope, intercept));
    }
    let ((right_a, right_b), (left_a, left_b)) = (fits[0], fits[1]);
    if !(left_b < 0. && 0. < right_b)
        || !(0.65 <= right_b - left_b && right_b - left_b <= 2.5)
        || (right_a.atan() - left_a.atan()).abs() > radians(12.)
    {
        return V::None;
    }
    let slope = (right_a + left_a) / 2.;
    dict! {
        "offset_right_m" => (right_b + left_b) / 2.,
        "heading_right_deg" => degrees(slope.atan()),
        "slope" => slope, "width_m" => right_b - left_b,
    }
}

/// autonomy_live.CorridorSteering.
#[derive(Clone, Debug)]
pub struct CorridorSteering {
    pub servo: i64,
    pub correcting: bool,
    pub changed_at: f64,
    pub last_scan: Option<i64>,
    pub last_publication: Option<f64>,
    pub history: Vec<(f64, f64, f64, f64)>,
    pub direction: i64,
    pub reverse_count: i64,
}

impl Default for CorridorSteering {
    fn default() -> Self {
        CorridorSteering {
            servo: 1500,
            correcting: false,
            changed_at: f64::NEG_INFINITY,
            last_scan: None,
            last_publication: None,
            history: Vec::new(),
            direction: 0,
            reverse_count: 0,
        }
    }
}

impl CorridorSteering {
    pub fn dump(&self) -> V {
        dict! {
            "servo" => self.servo, "correcting" => self.correcting, "changed_at" => self.changed_at,
            "last_scan" => self.last_scan, "last_publication" => self.last_publication,
            "history" => V::list(self.history.iter().map(|h| V::from(vec![h.0, h.1, h.2, h.3])).collect()),
            "direction" => self.direction, "reverse_count" => self.reverse_count,
        }
    }

    pub fn update(&mut self, walls: &V, scan_seq: &V, now: f64, motion_ready: bool, scan_at_ms: &V) -> i64 {
        if !motion_ready || walls.is_none() {
            self.servo = 1500;
            self.correcting = false;
            self.changed_at = now;
            self.history.clear();
            self.direction = 0;
            self.reverse_count = 0;
            return self.servo;
        }
        if !scan_seq.is_int() || self.last_scan.is_some_and(|last| scan_seq.i() < last) {
            self.history.clear();
            self.servo = 1500;
            self.correcting = false;
            self.direction = 0;
            self.reverse_count = 0;
            self.changed_at = now;
            return self.servo;
        }
        let seq = scan_seq.i();
        if Some(seq) == self.last_scan {
            return self.servo;
        }
        let published = if scan_at_ms.is_int() && scan_at_ms.i() >= 0 { scan_at_ms.i() as f64 / 1000. } else { now };
        if self.last_publication.is_some_and(|last| published <= last) {
            self.history.clear();
            self.servo = 1500;
            self.correcting = false;
            self.direction = 0;
            self.reverse_count = 0;
            self.changed_at = now;
            return self.servo;
        }
        self.last_scan = Some(seq);
        self.last_publication = Some(published);
        let offset = walls.idx("offset_right_m").f();
        let heading = walls.idx("heading_right_deg").f();
        let width = walls.idx("width_m").f();
        let projected_offset = offset + CORRIDOR_LOOKAHEAD_M * walls.idx("slope").f();
        if let Some(&previous) = self.history.last()
            && (!(0. < published - previous.0 && published - previous.0 < LIDAR_LATE_S)
                || (width - previous.3).abs() > 0.1
                || (offset - previous.2).abs() > 0.15
                || (heading - previous.1).abs() > 8.)
        {
            self.history.clear();
            self.reverse_count = 0;
        }
        self.history.push((published, heading, offset, width));
        if self.history.len() > 3 {
            let n = self.history.len();
            self.history.drain(..n - 3);
        }
        let mut damped_offset = projected_offset;
        if self.history.len() == 3 && self.history.windows(2).all(|w| w[1].0 - w[0].0 >= 0.05) {
            let rates: Vec<f64> =
                self.history.windows(2).map(|w| radians(w[1].1 - w[0].1) / (w[1].0 - w[0].0)).collect();
            if rates.iter().map(|r| r.abs()).fold(f64::NEG_INFINITY, pmax) <= radians(30.) {
                let candidate = projected_offset + 0.15 * CORRIDOR_LOOKAHEAD_M * median(rates);
                if candidate * projected_offset <= 0. {
                    damped_offset = 0.;
                } else if candidate.abs() < projected_offset.abs() {
                    damped_offset = candidate;
                }
            }
        }
        let usable_half_width = pmax(0.01, width / 2. - SIDE_CLEARANCE_M);
        if !self.correcting {
            self.correcting = offset.abs() > pmin(0.08, usable_half_width * 0.5)
                || heading.abs() > 4.
                || (heading.abs() > 1.5 && projected_offset.abs() > pmin(0.05, usable_half_width * 0.3));
        } else if offset.abs() < pmin(0.04, usable_half_width * 0.25)
            && heading.abs() < 2.
            && projected_offset.abs() < pmin(0.035, usable_half_width * 0.2)
        {
            self.correcting = false;
        }
        let mut target = 1500i64;
        let mut urgent = false;
        if self.correcting {
            let ratio = div(damped_offset, usable_half_width);
            let clamped = pmax(-1., pmin(1., ratio));
            let correction = round(STEERING_LIMIT_PWM as f64 * clamped);
            target -= correction;
            urgent = usable_half_width - projected_offset.abs() < SIDE_MIN_NET_M;
        }
        let sign = (target > 1500) as i64 - (target < 1500) as i64;
        if sign != 0 && self.direction != 0 && sign != self.direction {
            self.reverse_count += 1;
            if self.servo != 1500 || self.reverse_count < 2 {
                target = 1500;
            } else {
                self.direction = sign;
                self.reverse_count = 0;
            }
        } else {
            self.reverse_count = 0;
            if sign != 0 {
                self.direction = sign;
            }
        }
        let (mut interval, mut step) = if urgent { (0.15, 20i64) } else { (0.25, 15) };
        if (target - 1500).abs() < (self.servo - 1500).abs() {
            interval = 0.05;
            let elapsed = now - self.changed_at;
            let bounded = if elapsed > 0. { pmin(0.1, elapsed) } else { 0. };
            step = floor(130. * bounded + 1e-9);
        }
        if now - self.changed_at >= interval && target != self.servo {
            self.servo += (-step).max(step.min(target - self.servo));
            self.changed_at = now;
        }
        self.servo
    }
}

/// side_wall_fit(bins, side).
pub fn side_wall_fit(bins: &[Option<f64>], side: &str) -> V {
    let sector = if side == "right" { 20..161usize } else { 200..341 };
    let mut points: Vec<(f64, f64)> = Vec::new();
    for a in sector {
        if let Some(r) = bins[a]
            && r <= SIDE_WALL_REACH_M
        {
            points.push((r * dcos(a), r * dsin(a)));
        }
    }
    if points.len() < SIDE_WALL_MIN_POINTS {
        return V::None;
    }
    let mut slopes = Vec::new();
    for (i, &(x1, y1)) in points.iter().enumerate() {
        let mut j = i + 8;
        while j < points.len() {
            if (points[j].0 - x1).abs() >= 0.3 {
                slopes.push(div(points[j].1 - y1, points[j].0 - x1));
            }
            j += 8;
        }
    }
    if slopes.is_empty() {
        return V::None;
    }
    sort_floats(&mut slopes);
    let slope = slopes[slopes.len() / 2];
    let mut intercepts: Vec<f64> = points.iter().map(|&(x, y)| y - slope * x).collect();
    sort_floats(&mut intercepts);
    let intercept = intercepts[points.len() / 2];
    let inliers: Vec<(f64, f64)> =
        points.iter().cloned().filter(|&(x, y)| (y - slope * x - intercept).abs() <= SIDE_WALL_INLIER_M).collect();
    if inliers.len() < SIDE_WALL_MIN_POINTS || (inliers.len() as f64) < SIDE_WALL_MIN_FRACTION * points.len() as f64 {
        return V::None;
    }
    let n = inliers.len() as f64;
    let mx = fsum_py(inliers.iter().map(|p| p.0)) / n;
    let my = fsum_py(inliers.iter().map(|p| p.1)) / n;
    let sxx = fsum_py(inliers.iter().map(|&(x, _)| sq(x - mx)));
    if sxx <= 0. {
        return V::None;
    }
    let slope = fsum_py(inliers.iter().map(|&(x, y)| (x - mx) * (y - my))) / sxx;
    let intercept = my - slope * mx;
    let xmin = inliers.iter().map(|p| p.0).fold(f64::INFINITY, pmin);
    let xmax = inliers.iter().map(|p| p.0).fold(f64::NEG_INFINITY, pmax);
    let lateral = if side == "right" { intercept } else { -intercept };
    if slope.abs() > radians(15.).tan()
        || xmax - xmin < SIDE_WALL_MIN_SPAN_M
        || !(SIDE_WALL_LATERAL_M.0 <= lateral && lateral <= SIDE_WALL_LATERAL_M.1)
    {
        return V::None;
    }
    let error = inliers.iter().map(|&(x, y)| (y - slope * x - intercept).abs()).fold(f64::NEG_INFINITY, pmax)
        / hypot(1., slope);
    dict! {
        "side" => side, "slope" => slope, "intercept_m" => intercept, "points" => inliers.len(),
        "support_x_m" => V::from(vec![xmin, xmax]), "fit_error_m" => error,
        "basis" => "current_frame_side_board_returns",
    }
}

// ---------------------------------------------------------------- turn_motion helpers

/// turn_motion._point.
pub fn point_ok(value: &V) -> bool {
    let V::Dict(d) = value else { return false };
    d.len() == 2
        && value.has("x_m")
        && value.has("y_m")
        && ["x_m", "y_m"].iter().all(|k| value.get(k).is_num() && value.get(k).f().abs() <= 12.)
}

/// turn_motion._valid_wall.
pub fn valid_wall(wall: &V) -> bool {
    if !wall.is_dict() {
        return false;
    }
    wall.get("candidate_only").is_true()
        && ["heading_left_rad", "rho_left_m", "support_span_m", "fit_error_m"].iter().all(|k| wall.get(k).is_num())
        && -PI <= wall.idx("heading_left_rad").f()
        && wall.idx("heading_left_rad").f() <= PI
        && wall.idx("rho_left_m").f().abs() <= 12.
        && wall.get("points").is_int()
        && wall.idx("points").i() >= 16
        && wall.idx("support_span_m").f() >= 0.35
        && 0. <= wall.idx("fit_error_m").f()
        && wall.idx("fit_error_m").f() <= 0.06
        && point_ok(wall.get("support_start_left_m"))
        && point_ok(wall.get("support_end_left_m"))
}

/// turn_motion._valid_opening.
pub fn valid_opening(goal: &V, scan: &V) -> bool {
    if !goal.is_dict() || !valid_wall(goal.get("outer_wall")) {
        return false;
    }
    let ray = goal.get("target_support_ray");
    let end_ray = goal.get("incoming_left_end_support");
    let nums = ["heading_left_rad", "center_offset_left_m", "width_m", "incoming_heading_left_rad", "front_wall_m", "incoming_left_end_m"];
    let ok = goal.get("candidate_only").is_true()
        && goal.get("turn_path_certified").is_false()
        && goal.get("observation_type").os() == Some("left_opening_outer_wall_alignment")
        && goal.get("origin_between_exit_walls").is_bool()
        && nums.iter().all(|k| goal.get(k).is_num())
        && (-PI..=PI).contains(&goal.idx("heading_left_rad").f())
        && (0.6..=5.).contains(&goal.idx("width_m").f())
        && (-PI..=PI).contains(&goal.idx("incoming_heading_left_rad").f())
        && (-0.3..=1.).contains(&goal.idx("incoming_left_end_m").f())
        && point_ok(goal.get("target_point_left_m"))
        && ray.is_dict()
        && ray.get("index").is_int()
        && (0..360).contains(&ray.idx("index").i())
        && ["angle_left_rad", "range_m", "target_range_m"].iter().all(|k| ray.get(k).is_num())
        && (0.02..=12.).contains(&ray.idx("range_m").f())
        && 0. < ray.idx("target_range_m").f()
        && ray.idx("target_range_m").f() < ray.idx("range_m").f()
        && goal.get("known_open_fraction").is_num()
        && (0.9..=1.).contains(&goal.idx("known_open_fraction").f())
        && end_ray.is_dict()
        && end_ray.get("index").is_int()
        && (0..360).contains(&end_ray.idx("index").i())
        && ["angle_left_rad", "range_m"].iter().all(|k| end_ray.get(k).is_num())
        && (0.02..=12.).contains(&end_ray.idx("range_m").f())
        && point_ok(end_ray.get("point_left_m"));
    if !ok {
        return false;
    }
    let wall = goal.idx("outer_wall");
    let point = goal.idx("target_point_left_m");
    let (theta, distance) = (ray.idx("angle_left_rad").f(), ray.idx("target_range_m").f());
    let ranges = scan.get("ranges");
    let end = end_ray.idx("point_left_m");
    let (end_angle, end_distance) = (end_ray.idx("angle_left_rad").f(), end_ray.idx("range_m").f());
    let incoming = goal.idx("incoming_heading_left_rad").f();
    let Some(list) = ranges.ol() else { return false };
    if list.len() != 360 {
        return false;
    }
    let ri = ray.idx("index").i() as usize;
    let ei = end_ray.idx("index").i() as usize;
    let (px, py) = (point.idx("x_m").f(), point.idx("y_m").f());
    let (ex, ey) = (end.idx("x_m").f(), end.idx("y_m").f());
    let heading = goal.idx("heading_left_rad").f();
    let offset = goal.idx("center_offset_left_m").f();
    list[ri].is_num()
        && (list[ri].f() - ray.idx("range_m").f()).abs() <= 1e-6
        && wrap(theta + radians(ri as f64)).abs() <= 1e-6
        && hypot(px - distance * theta.pcos(), py - distance * theta.psin()) <= 0.02
        && wrap(heading - wall.idx("heading_left_rad").f()).abs() <= radians(2.)
        && (offset - (wall.idx("rho_left_m").f() + goal.idx("width_m").f() / 2.)).abs() <= 0.05
        && (-heading.psin() * px + heading.pcos() * py - offset).abs() <= 0.06
        && list[ei].is_num()
        && (list[ei].f() - end_distance).abs() <= 1e-6
        && wrap(end_angle + radians(ei as f64)).abs() <= 1e-6
        && hypot(ex - end_distance * end_angle.pcos(), ey - end_distance * end_angle.psin()) <= 1e-6
        && (incoming.pcos() * ex + incoming.psin() * ey - goal.idx("incoming_left_end_m").f()).abs() <= 1e-6
        && -incoming.psin() * ex + incoming.pcos() * ey > 0.
}

const CANDIDATE_FIELDS: [&str; 10] = [
    "heading_left_rad",
    "center_offset_left_m",
    "width_m",
    "left_wall_points",
    "right_wall_points",
    "support_span_m",
    "fit_error_m",
    "origin_between_walls",
    "candidate_only",
    "turn_path_certified",
];

/// turn_motion._valid_candidate.
pub fn valid_candidate(value: &V, lateral_margin: f64) -> bool {
    if !value.is_dict() || CANDIDATE_FIELDS.iter().any(|k| !value.has(k)) {
        return false;
    }
    if !["heading_left_rad", "center_offset_left_m", "width_m", "support_span_m", "fit_error_m"]
        .iter()
        .all(|k| value.idx(k).is_num())
    {
        return false;
    }
    let width = value.idx("width_m").f();
    value.idx("candidate_only").is_true()
        && value.idx("turn_path_certified").is_false()
        && value.idx("origin_between_walls").is_true()
        && (-PI..=PI).contains(&value.idx("heading_left_rad").f())
        && (0.6..=5.).contains(&width)
        && value.idx("support_span_m").f() >= 0.35
        && (0. ..=0.06).contains(&value.idx("fit_error_m").f())
        && ["left_wall_points", "right_wall_points"].iter().all(|k| value.idx(k).is_int() && value.idx(k).i() >= 16)
        && value.idx("center_offset_left_m").f().abs() < width / 2. - lateral_margin
}

pub fn valid_candidate_default(value: &V) -> bool {
    valid_candidate(value, TURN_LATERAL_MARGIN_M)
}

/// A 360-bit set (Python's int bit masks over scan positions).
#[derive(Clone, Copy, PartialEq, Eq, Default, Debug)]
pub struct Bits([u64; 6]);

impl Bits {
    pub fn bit(position: usize) -> Bits {
        let mut b = Bits::default();
        b.0[position / 64] = 1u64 << (position % 64);
        b
    }
    pub fn or(self, o: Bits) -> Bits {
        let mut r = self;
        for i in 0..6 {
            r.0[i] |= o.0[i];
        }
        r
    }
    pub fn and(self, o: Bits) -> Bits {
        let mut r = self;
        for i in 0..6 {
            r.0[i] &= o.0[i];
        }
        r
    }
    pub fn count(self) -> u32 {
        self.0.iter().map(|w| w.count_ones()).sum()
    }
    pub fn is_zero(self) -> bool {
        self.0.iter().all(|&w| w == 0)
    }
    /// Bits low..=high.
    pub fn interval(low: usize, high: usize) -> Bits {
        let mut b = Bits::default();
        for p in low..=high {
            b.0[p / 64] |= 1u64 << (p % 64);
        }
        b
    }
}

/// (mature groups: label -> (mask, start, end), line bits) or None.
pub type Support = Option<(Vec<(usize, (Bits, usize, usize))>, Bits)>;

/// turn_motion._surface_supports(scan, walls).
pub fn surface_supports(scan: &V, walls: &[V]) -> Vec<Support> {
    let ranges = scan.get("ranges");
    let Some(list) = ranges.ol() else { return vec![None; walls.len()] };
    if list.len() != 360 || list.iter().any(|r| r.is_some() && (!r.is_num() || !(0.02..=12.).contains(&r.f()))) {
        return vec![None; walls.len()];
    }
    let rv: Vec<Option<f64>> = list.iter().map(|r| r.of()).collect();
    // _RAY_AXES[i] = (cos(radians(-i)), sin(radians(-i)))
    let points: Vec<Option<(f64, f64)>> = (0..360)
        .map(|i| {
            rv[i].map(|r| {
                let a = radians(-(i as f64));
                (r * a.pcos(), r * a.psin())
            })
        })
        .collect();
    let sample_angle = radians(1.);
    let mut basic = vec![false; 360];
    for i in 0..360 {
        let j = (i + 1) % 360;
        let (p, q) = (points[i], points[j]);
        basic[i] = match (p, q) {
            (Some(p), Some(q)) => {
                let limit = pmin(0.15, 0.04 + 3. * pmin(rv[i].unwrap(), rv[j].unwrap()) * sample_angle);
                hypot(q.0 - p.0, q.1 - p.1) <= limit
            }
            _ => false,
        };
    }
    let basic_pivot = basic.iter().position(|e| !e).map(|i| (i + 1) % 360).unwrap_or(0);
    let basic_order: Vec<usize> = (0..360).map(|i| (basic_pivot + i) % 360).collect();
    let ordered: Vec<Option<(f64, f64)>> = basic_order.iter().map(|&i| points[i]).collect();
    let circular = basic.iter().all(|&b| b);
    let mut run_start: Vec<Option<usize>> = vec![None; 360];
    let mut run_end: Vec<Option<usize>> = vec![None; 360];
    let mut start = 0usize;
    for (position, &i) in basic_order.iter().enumerate() {
        if points[i].is_none() {
            continue;
        }
        if position == 0 || !basic[basic_order[position - 1]] {
            start = position;
        }
        run_start[position] = Some(start);
        if position == 359 || !basic[i] {
            for member in start..=position {
                run_end[member] = Some(position + 1);
            }
        }
    }
    let mut extended: Vec<Option<(f64, f64)>> = ordered[355..].to_vec();
    extended.extend_from_slice(&ordered);
    extended.extend_from_slice(&ordered[..6]);
    let (mut sx, mut sy, mut sxx, mut syy, mut sxy) = (vec![0.], vec![0.], vec![0.], vec![0.], vec![0.]);
    for p in &extended {
        let (x, y) = p.unwrap_or((0., 0.));
        sx.push(sx.last().unwrap() + x);
        sy.push(sy.last().unwrap() + y);
        sxx.push(sxx.last().unwrap() + x * x);
        syy.push(syy.last().unwrap() + y * y);
        sxy.push(sxy.last().unwrap() + x * y);
    }
    let tangent = |window: i64| -> (f64, f64) {
        let i = (window + 5) as usize;
        let (x, y) = (sx[i + 6] - sx[i], sy[i + 6] - sy[i]);
        let xx = sxx[i + 6] - sxx[i] - x * x / 6.;
        let yy = syy[i + 6] - syy[i] - y * y / 6.;
        let xy = sxy[i + 6] - sxy[i] - x * y / 6.;
        let angle = 0.5 * (2. * xy).atan2(xx - yy);
        let direction = (angle.pcos(), angle.psin());
        let first = extended[i].unwrap_or_else(|| type_error("cannot unpack non-iterable NoneType object"));
        let last = extended[i + 5].unwrap_or_else(|| type_error("cannot unpack non-iterable NoneType object"));
        let (dx, dy) = (last.0 - first.0, last.1 - first.1);
        if direction.0 * dx + direction.1 * dy >= 0. { direction } else { (-direction.0, -direction.1) }
    };
    let mut strong = vec![false; 360];
    for (position, &i) in basic_order.iter().enumerate() {
        if !basic[i] {
            continue;
        }
        let (low, high) = (run_start[position].unwrap() as i64, run_end[position].unwrap() as i64);
        if high - low < 6 {
            continue;
        }
        let mut before_start = position as i64 - 5;
        let mut after_start = position as i64 + 1;
        if !circular {
            before_start = low.max(before_start.min(high - 6));
            after_start = low.max(after_start.min(high - 6));
        }
        let (before, after) = (tangent(before_start), tangent(after_start));
        let cross = before.0 * after.1 - before.1 * after.0;
        let dot = before.0 * after.0 + before.1 * after.1;
        let (p, q) = (points[i].unwrap(), points[(i + 1) % 360].unwrap());
        let (dx, dy) = (q.0 - p.0, q.1 - p.1);
        strong[i] = cross.atan2(dot).abs() <= radians(20.)
            && (before.0 * dy - before.1 * dx).abs() <= 0.04
            && (after.0 * dy - after.1 * dx).abs() <= 0.04;
    }
    let pivot = strong.iter().position(|e| !e).map(|i| (i + 1) % 360).unwrap_or(0);
    let order: Vec<usize> = (0..360).map(|i| (pivot + i) % 360).collect();
    let mut labels = Vec::with_capacity(360);
    let mut label = 0usize;
    for (position, _) in order.iter().enumerate() {
        if position > 0 && !strong[order[position - 1]] {
            label += 1;
        }
        labels.push(label);
    }
    let ordered_points: Vec<(usize, (f64, f64), usize, Bits)> = order
        .iter()
        .enumerate()
        .filter_map(|(position, &i)| points[i].map(|p| (position, p, labels[position], Bits::bit(position))))
        .collect();
    let mut supports = Vec::new();
    let mut projections: Vec<(f64, Vec<(usize, f64, f64, usize, Bits)>)> = Vec::new();
    for wall in walls {
        let theta = wall.idx("heading_left_rad").f();
        let (c, s) = (theta.pcos(), theta.psin());
        let mut ends: Vec<f64> = ["support_start_left_m", "support_end_left_m"]
            .iter()
            .map(|k| c * wall.idx(k).idx("x_m").f() + s * wall.idx(k).idx("y_m").f())
            .collect();
        sort_floats(&mut ends);
        let (mut low, mut high) = (ends[0], ends[1]);
        let mut groups: Vec<(usize, (Bits, usize, usize))> = Vec::new();
        let mut count = 0;
        let mut line_bits = Bits::default();
        if !projections.iter().any(|(t, _)| *t == theta) {
            projections.push((
                theta,
                ordered_points
                    .iter()
                    .map(|&(position, p, key, bit)| (position, -s * p.0 + c * p.1, c * p.0 + s * p.1, key, bit))
                    .collect(),
            ));
        }
        let projection = &projections.iter().find(|(t, _)| *t == theta).unwrap().1;
        let rho = wall.idx("rho_left_m").f();
        let error = wall.idx("fit_error_m").f() + 1e-6;
        low -= 1e-6;
        high += 1e-6;
        for &(position, normal, along, key, bit) in projection {
            if (normal - rho).abs() > error {
                continue;
            }
            line_bits = line_bits.or(bit);
            if !(low <= along && along <= high) {
                continue;
            }
            count += 1;
            match groups.iter_mut().find(|(k, _)| *k == key) {
                Some((_, value)) => *value = (value.0.or(bit), value.1, position),
                None => groups.push((key, (bit, position, position))),
            }
        }
        let mature: Vec<(usize, (Bits, usize, usize))> = groups.into_iter().filter(|(_, v)| v.0.count() >= 16).collect();
        supports.push(if count >= 16 { Some((mature, line_bits)) } else { None });
    }
    supports
}

/// turn_motion._same_surface: Some(true/false) or None.
pub fn same_surface(a: &Support, b: &Support) -> Option<bool> {
    let (Some((a, left_line)), Some((b, right_line))) = (a, b) else { return None };
    let shared = left_line.and(*right_line);
    if shared.is_zero() {
        return Some(false);
    }
    let mut mature = false;
    for (key, (left, lo, hi)) in a {
        let Some((_, (right, start, end))) = b.iter().find(|(k, _)| k == key) else { continue };
        if left.count() < 16 || right.count() < 16 {
            continue;
        }
        mature = true;
        let (low, high) = ((*lo).min(*start), (*hi).max(*end));
        let interval = Bits::interval(low, high);
        if left_line.or(*right_line).and(interval) == interval && shared.and(interval).count() >= 4 {
            return Some(true);
        }
    }
    if mature { Some(false) } else { None }
}

// ---------------------------------------------------------------- maneuver_sequence functions

/// _wrap90.
pub fn wrap90(angle: f64) -> f64 {
    let quarter = PI / 2.;
    fmod(angle + quarter / 2., quarter) - quarter / 2.
}

/// WallCompass: car heading relative to the start-lane axis.
#[derive(Clone, Debug, Default)]
pub struct WallCompass {
    pub heading: Option<f64>,
    pub rate: f64,
    pub at_s: Option<f64>,
    pub measured_at_s: Option<f64>,
    pub last_measurement: Option<(f64, f64)>,
    pub evidence: V,
}

impl WallCompass {
    pub fn dump(&self) -> V {
        dict! {
            "heading" => self.heading, "rate" => self.rate, "at_s" => self.at_s,
            "measured_at_s" => self.measured_at_s,
            "_last_measurement" => self.last_measurement.map(|m| V::from(m)),
            "evidence" => self.evidence.clone(),
        }
    }

    pub fn update(&mut self, scan: &V) -> Option<f64> {
        let at_ms = scan.idx("at_ms").i();
        let at = at_ms as f64 / 1000.;
        if let Some(at_s) = self.at_s
            && at <= at_s
        {
            return self.heading;
        }
        let predicted = match self.heading {
            None => 0.,
            Some(h) => h + self.rate * pmin(at - self.at_s.unwrap(), COMPASS_MAX_PREDICT_S),
        };
        let mut votes = Self::votes(scan, predicted);
        let support = fsum_py(votes.iter().map(|v| v.1));
        let mut median_residual = None;
        if support >= COMPASS_MIN_SUPPORT_M {
            votes.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap().then(a.1.partial_cmp(&b.1).unwrap()));
            let mut accumulated = 0.;
            for &(residual, span) in &votes {
                accumulated += span;
                if accumulated >= support / 2. {
                    median_residual = Some(residual);
                    break;
                }
            }
            let heading = predicted - median_residual.unwrap_or_else(|| type_error("unsupported operand type(s) for -"));
            if let Some(last) = self.last_measurement {
                let dt = at - last.1;
                if COMPASS_RATE_WINDOW_S.0 <= dt && dt <= COMPASS_RATE_WINDOW_S.1 {
                    self.rate = pmax(-COMPASS_MAX_RATE_RAD_S, pmin(COMPASS_MAX_RATE_RAD_S, (heading - last.0) / dt));
                }
            }
            self.last_measurement = Some((heading, at));
            self.heading = Some(heading);
            self.measured_at_s = Some(at);
        } else if self.heading.is_some() {
            self.heading = Some(predicted);
        }
        self.at_s = Some(at);
        self.evidence = dict! {
            "source_at_ms" => at_ms, "heading_rad" => self.heading, "rate_rad_s" => self.rate,
            "predicted_rad" => predicted, "votes" => votes.len(),
            // sum() of no votes is the int 0.
            "support_m" => if votes.is_empty() { V::Int(0) } else { V::Float(support) },
            "median_residual_rad" => median_residual, "measured" => median_residual.is_some(),
            "basis" => "native_wall_directions_mod_90_vs_start_lane_axis",
        };
        self.heading
    }

    pub fn votes(scan: &V, predicted: f64) -> Vec<(f64, f64)> {
        let mut votes = Vec::new();
        let walls = scan.get("wall_candidates");
        let empty: [V; 0] = [];
        let items: &[V] = if walls.truthy() { walls.l() } else { &empty };
        for wall in items {
            if !valid_wall(wall)
                || wall.idx("support_span_m").f() < COMPASS_MIN_SPAN_M
                || wall.idx("fit_error_m").f() > COMPASS_MAX_FIT_M
                || wall.idx("points").i() < COMPASS_MIN_POINTS
            {
                continue;
            }
            let residual = wrap90(predicted + wall.idx("heading_left_rad").f());
            if residual.abs() <= compass_gate_rad() {
                votes.push((residual, wall.idx("support_span_m").f()));
            }
        }
        votes
    }

    pub fn measurable(&self, scan: &V) -> bool {
        let (Some(heading), Some(at_s)) = (self.heading, self.at_s) else { return false };
        if !scan.is_dict() || !scan.get("at_ms").is_int() {
            return false;
        }
        let at = scan.idx("at_ms").i() as f64 / 1000.;
        let elapsed = if at - at_s > 0. { at - at_s } else { 0. };
        let predicted = heading + self.rate * pmin(elapsed, COMPASS_MAX_PREDICT_S);
        fsum_py(Self::votes(scan, predicted).iter().map(|v| v.1)) >= COMPASS_MIN_SUPPORT_M
    }

    pub fn current(&self, scan: &V) -> Option<f64> {
        let (Some(heading), Some(measured)) = (self.heading, self.measured_at_s) else { return None };
        if !scan.is_dict() || !scan.get("at_ms").is_int() {
            return None;
        }
        let at = scan.idx("at_ms").i() as f64 / 1000.;
        if !(0. <= at - measured && at - measured <= COMPASS_MAX_STALE_S) {
            return None;
        }
        let elapsed = at - self.at_s.unwrap();
        let elapsed = if elapsed > 0. { elapsed } else { 0. };
        Some(heading + self.rate * pmin(elapsed, COMPASS_MAX_PREDICT_S))
    }
}

/// LaneWallSteering: start-lane nominal servo from the current lane walls.
#[derive(Clone, Debug)]
pub struct LaneWallSteering {
    pub servo: i64,
    pub history: Vec<(f64, f64, f64, f64)>,
    pub last_scan: Option<i64>,
    pub last_publication: Option<f64>,
    pub evidence: V,
}

impl Default for LaneWallSteering {
    fn default() -> Self {
        LaneWallSteering { servo: NEUTRAL, history: Vec::new(), last_scan: None, last_publication: None, evidence: V::None }
    }
}

impl LaneWallSteering {
    pub fn dump(&self) -> V {
        dict! {
            "servo" => self.servo,
            "history" => V::list(self.history.iter().map(|h| V::from(vec![h.0, h.1, h.2, h.3])).collect()),
            "last_scan" => self.last_scan, "last_publication" => self.last_publication,
            "evidence" => self.evidence.clone(),
        }
    }

    fn reset(&mut self) -> i64 {
        self.history.clear();
        self.servo = NEUTRAL;
        self.servo
    }

    pub fn update(&mut self, walls: &V, scan_seq: &V, now: f64, motion_ready: bool, scan_at_ms: &V) -> i64 {
        if !motion_ready || walls.is_none() || !scan_seq.is_int() {
            self.last_scan = if scan_seq.is_int() { Some(scan_seq.i()) } else { None };
            return self.reset();
        }
        let seq = scan_seq.i();
        if self.last_scan.is_some_and(|last| seq < last) {
            self.last_scan = Some(seq);
            return self.reset();
        }
        if Some(seq) == self.last_scan {
            return self.servo;
        }
        let published = if scan_at_ms.is_int() && scan_at_ms.i() >= 0 { scan_at_ms.i() as f64 / 1000. } else { now };
        if self.last_publication.is_some_and(|last| published <= last) {
            self.last_scan = Some(seq);
            self.last_publication = Some(published);
            return self.reset();
        }
        self.last_scan = Some(seq);
        self.last_publication = Some(published);
        let heading = radians(walls.idx("heading_right_deg").f());
        let (offset, width) = (walls.idx("offset_right_m").f(), walls.idx("width_m").f());
        if let Some(&previous) = self.history.last()
            && (published - previous.0 > LANE_WALL_GAP_S
                || (width - previous.3).abs() > 0.1
                || (offset - previous.2).abs() > 0.15
                || (heading - previous.1).abs() > radians(8.))
        {
            self.history.clear();
        }
        self.history.push((published, heading, offset, width));
        if self.history.len() > LANE_WALL_HISTORY {
            let n = self.history.len();
            self.history.drain(..n - LANE_WALL_HISTORY);
        }
        let mut rates: Vec<f64> = self
            .history
            .windows(2)
            .filter(|w| w[1].0 - w[0].0 >= 0.05)
            .map(|w| (w[1].1 - w[0].1) / (w[1].0 - w[0].0))
            .collect();
        sort_floats(&mut rates);
        let trend = if self.history.len() == LANE_WALL_HISTORY && !rates.is_empty() {
            (rates[(rates.len() - 1) / 2] + rates[rates.len() / 2]) / 2.
        } else {
            0.
        };
        let error = heading + offset / LANE_WALL_LOOKAHEAD_M;
        let predicted = error + LANE_WALL_LEAD_S * trend;
        let correction = clampi(round(LANE_WALL_GAIN_PWM_PER_RAD * predicted), -LANE_WALL_CAP_PWM, LANE_WALL_CAP_PWM);
        self.servo = NEUTRAL - correction;
        self.evidence = dict! {
            "error_rad" => error, "trend_rad_s" => trend, "predicted_rad" => predicted,
            "correction_pwm" => correction, "history" => self.history.len(),
        };
        self.servo
    }
}

/// _axis_frame(axis, heading, side).
pub fn axis_frame(axis: f64, heading: f64, side: f64) -> ((f64, f64), (f64, f64)) {
    let angle = axis - heading;
    let e1 = (angle.pcos(), angle.psin());
    (e1, (-side * e1.1, side * e1.0))
}

/// _scan_points(ranges, reach).
pub fn scan_points(ranges: &[Option<f64>], reach: f64) -> Vec<(f64, f64)> {
    let mut out = Vec::new();
    for (i, r) in ranges.iter().enumerate() {
        if let Some(r) = *r
            && r.is_finite()
            && 0.02 <= r
            && r <= reach
        {
            out.push((r * dcos(i), -r * dsin(i)));
        }
    }
    out
}

/// _support_center(ranges, bins, diameter).
pub fn support_center(ranges: &[Option<f64>], bins: &[usize], diameter: &V) -> ((f64, f64), f64) {
    let points: Vec<(f64, f64)> = bins
        .iter()
        .map(|&i| {
            let r = ranges[i].unwrap_or_else(|| type_error("unsupported operand type(s) for *: 'NoneType' and 'float'"));
            (r * dcos(i), -r * dsin(i))
        })
        .collect();
    let n = points.len() as f64;
    let (mut x, mut y) = (div(fsum_py(points.iter().map(|p| p.0)), n), div(fsum_py(points.iter().map(|p| p.1)), n));
    let d = if diameter.is_num() { diameter.f() } else { 0.2 };
    let radius = pmax(0.05, pmin(MAX_DIAMETER_M / 2., d / 2.));
    let distance = hypot(x, y);
    if distance > 1e-6 {
        let (x0, y0) = (x, y);
        x = x0 + 2. * radius / PI * x0 / distance;
        y = y0 + 2. * radius / PI * y0 / distance;
    }
    ((x, y), radius)
}

/// _probe_free(points, origin, direction, skip).
pub fn probe_free(points: &[(f64, f64)], origin: (f64, f64), direction: (f64, f64), skip: f64) -> f64 {
    let (ox, oy) = origin;
    let (ux, uy) = direction;
    let mut best = INF;
    for &(x, y) in points {
        let (dx, dy) = (x - ox, y - oy);
        if dx * dx + dy * dy <= skip * skip {
            continue;
        }
        let along = dx * ux + dy * uy;
        if 0. < along && along < best && along <= OPEN_PROBE_REACH_M && (dy * ux - dx * uy).abs() <= OPEN_PROBE_HALF_WIDTH_M {
            best = along;
        }
    }
    best
}

/// _loop_extent(a, b, radius, theta).
pub fn loop_extent(a: f64, b: f64, radius: f64, theta: f64) -> f64 {
    let projection = a * theta.pcos() + b * theta.psin();
    projection + sqrt(pmax(0., projection * projection - (a * a + b * b) + radius * radius))
}

/// _plan_loop(free, support_radius, radii, approach).
pub fn plan_loop(free: &[(f64, f64)], support_radius: f64, radii: (f64, f64), approach: Option<(f64, f64)>) -> V {
    let lo = support_radius + SIDE_BODY_EXTENT_M + OPEN_INNER_GAP_M;
    let bands: Vec<(f64, f64)> =
        free.iter().map(|&(theta, distance)| (theta, distance - SIDE_BODY_EXTENT_M - OPEN_OUTER_GAP_M)).collect();
    let inner_floor =
        if bands.iter().any(|&(_, hi)| hi < lo) { support_radius + SIDE_BODY_EXTENT_M } else { lo };
    let trig: Vec<(f64, f64, f64, bool)> = bands.iter().map(|&(theta, hi)| (theta.pcos(), theta.psin(), hi, hi >= lo)).collect();
    let score = |radius: f64, a: f64, b: f64| -> f64 {
        if hypot(a, b) > radius - inner_floor {
            return f64::NEG_INFINITY;
        }
        let mut worst = INF;
        let (ab, rr) = (a * a + b * b, radius * radius);
        for &(c, s, hi, comfort) in &trig {
            let projection = a * c + b * s;
            let d = projection * projection - ab + rr;
            let t = projection + (if d > 0. { d } else { 0. }).sqrt();
            let margin = if comfort {
                let (inner, outer) = (t - lo, hi - t);
                if outer < inner { outer } else { inner }
            } else {
                -(t - hi).abs()
            };
            if margin < worst {
                worst = margin;
            }
        }
        if let Some(approach) = approach
            && a - approach.0 > 0.
        {
            let descent = OPEN_APPROACH_SLOPE * (approach.1 - (b - radius)).abs() - (a - approach.0);
            worst -= if descent > 0. { descent } else { 0. };
        }
        worst - 0.01 * hypot(a, b) - OPEN_RADIUS_PREFERENCE * (radii.1 - radius)
    };
    let sweep = |radius: f64, offsets: &[f64], a: Option<f64>, b: Option<f64>| -> f64 {
        let limit = radius - inner_floor;
        let mut choice = offsets[0];
        let mut top: Option<f64> = None;
        let fixed = if a.is_none() { b.unwrap() } else { a.unwrap() };
        let reach = limit * limit - fixed * fixed;
        if reach >= 0. {
            let middle = offsets.len() / 2;
            let k = ((reach.sqrt() + 1e-6) / 0.05) as usize + 1;
            let lo_i = middle.saturating_sub(k);
            let hi_i = (middle + k + 1).min(offsets.len());
            for &v in &offsets[lo_i..hi_i] {
                let (pa, pb) = if a.is_none() { (v, b.unwrap()) } else { (a.unwrap(), v) };
                if hypot(pa, pb) > limit {
                    continue;
                }
                let value = score(radius, pa, pb);
                if top.is_none() || value > top.unwrap() {
                    choice = v;
                    top = Some(value);
                }
            }
        }
        choice
    };
    let mut best: Option<(f64, f64, f64, f64)> = None;
    let mut radius = radii.0;
    while radius <= radii.1 + 1e-9 {
        let (mut a, mut b) = (0.0f64, 0.0f64);
        let n = ceil(radius / 0.05);
        let offsets: Vec<f64> = (-n..n + 1).map(|k| k as f64 * 0.05).collect();
        for _ in 0..2 {
            b = sweep(radius, &offsets, Some(a), None);
            a = sweep(radius, &offsets, None, Some(b));
        }
        let value = score(radius, a, b);
        if best.is_none() || value > best.unwrap().0 {
            best = Some((value, radius, a, b));
        }
        radius += OPEN_RADIUS_STEP_M;
    }
    let (value, radius, a, b) = best.unwrap();
    let board_slack = bands.iter().map(|&(theta, hi)| hi - loop_extent(a, b, radius, theta)).fold(INF, pmin);
    dict! {
        "radius_m" => round_n(radius, 3), "offset_axis_m" => round_n(a, 3), "offset_side_m" => round_n(b, 3),
        "min_slack_m" => value, "inner_bound_m" => lo, "inner_floor_m" => inner_floor,
        "board_slack_m" => board_slack,
        "extents_m" => V::from(free.iter().map(|&(theta, _)| round_n(loop_extent(a, b, radius, theta), 3)).collect::<Vec<f64>>()),
    }
}

/// _segment_distance(point, end).
pub fn segment_distance(point: (f64, f64), end: (f64, f64)) -> f64 {
    let length2 = sq(end.0) + sq(end.1);
    let t = if length2 < 1e-12 { 0. } else { pmax(0., pmin(1., (point.0 * end.0 + point.1 * end.1) / length2)) };
    hypot(point.0 - t * end.0, point.1 - t * end.1)
}

/// _loop_carrot(center, radius, side, lookahead, axis_dir, turn_normal, support).
pub fn loop_carrot(
    center: (f64, f64),
    radius: f64,
    side: f64,
    lookahead: f64,
    axis_dir: (f64, f64),
    turn_normal: (f64, f64),
    support: Option<((f64, f64), f64)>,
) -> ((f64, f64), &'static str, f64) {
    let (ox, oy) = center;
    let (ex, ey) = axis_dir;
    let (nx, ny) = turn_normal;
    let (tx, ty) = (ox - radius * nx, oy - radius * ny);
    let before = tx * ex + ty * ey;
    let distance = hypot(ox, oy);
    let mut lookahead = lookahead;
    if before > 0. {
        if before > lookahead {
            let offset = (ey * tx - ex * ty).abs();
            lookahead = pmax(lookahead, pmin(OPEN_APPROACH_CONVERGE_CAP * before, OPEN_APPROACH_CONVERGE_RATIO * offset));
        }
        let s = lookahead - before;
        let (point, mode) = if s <= 0. {
            ((tx + s * ex, ty + s * ey), "approach_line")
        } else {
            let phi = (ty - oy).atan2(tx - ox) + side * s / radius;
            ((ox + radius * phi.pcos(), oy + radius * phi.psin()), "approach_line_to_circle")
        };
        match support {
            None => return (point, mode, before),
            Some((c, r)) if segment_distance(c, point) >= r => return (point, mode, before),
            _ => {}
        }
    }
    if distance > radius + 1e-6 {
        let tangent = sqrt(distance * distance - radius * radius);
        let theta = oy.atan2(ox) - side * asin(radius / distance);
        if tangent >= lookahead {
            return ((lookahead * theta.pcos(), lookahead * theta.psin()), "tangent", pmax(before, 0.));
        }
        let (px, py) = (tangent * theta.pcos(), tangent * theta.psin());
        let phi = (py - oy).atan2(px - ox) + side * (lookahead - tangent) / radius;
        return ((ox + radius * phi.pcos(), oy + radius * phi.psin()), "tangent_to_circle", pmax(before, 0.));
    }
    let phi = (-oy).atan2(-ox) + side * lookahead / radius;
    ((ox + radius * phi.pcos(), oy + radius * phi.psin()), "circle", 0.)
}

/// _in_pose(point, pose).
pub fn in_pose(point: (f64, f64), pose: (f64, f64, f64, f64)) -> (f64, f64) {
    let (dx, dy) = (point.0 - pose.0, point.1 - pose.1);
    let (c, s) = ((-pose.2).pcos(), (-pose.2).psin());
    (c * dx - s * dy, s * dx + c * dy)
}

/// _turned(vector, turn).
pub fn turned(vector: (f64, f64), turn: f64) -> (f64, f64) {
    let (c, s) = ((-turn).pcos(), (-turn).psin());
    (c * vector.0 - s * vector.1, s * vector.0 + c * vector.1)
}

/// _overshoot_guard(pwm, predicted).
pub fn overshoot_guard(pwm: i64, predicted: i64) -> i64 {
    if pwm != NEUTRAL && (predicted - NEUTRAL) * (pwm - NEUTRAL) <= 0 { predicted } else { pwm }
}

/// _pursuit_pwm(point): (curvature, pwm).
pub fn pursuit_pwm(point: (f64, f64)) -> (f64, i64) {
    let curvature = 2. * point.1 / pmax(1e-6, sq(point.0) + sq(point.1));
    (curvature, clampi(NEUTRAL + round(ROUTE_PWM_PER_CURVATURE * curvature), nominal_right_limit_pwm(), SERVO_MAX))
}

/// _strip_clear(points, end, skip_center, skip_radius).
pub fn strip_clear(points: &[(f64, f64)], end: (f64, f64), skip_center: Option<(f64, f64)>, skip_radius: f64) -> bool {
    let length = hypot(end.0, end.1);
    if length < 1e-6 {
        return true;
    }
    let (ux, uy) = (end.0 / length, end.1 / length);
    let half = SIDE_BODY_EXTENT_M + OPEN_APPROACH_MARGIN_M;
    for &(x, y) in points {
        if let Some(c) = skip_center
            && dist((x, y), c) <= skip_radius
        {
            continue;
        }
        let along = x * ux + y * uy;
        if -REAR_BODY_EXTENT_M < along && along <= length + FRONT_BODY_EXTENT_M && (y * ux - x * uy).abs() <= half {
            return false;
        }
    }
    true
}

/// An _axis_lines run.
#[derive(Clone, Debug)]
pub struct Line {
    pub lateral_m: f64,
    pub along_m: (f64, f64),
    pub points: usize,
    pub source: &'static str,
}

impl Line {
    pub fn to_v(&self) -> V {
        dict! {"lateral_m" => self.lateral_m, "along_m" => V::from(self.along_m), "points" => self.points, "source" => self.source}
    }
}

/// Total-order key for floats as Python compares them (-0.0 == 0.0).
#[derive(Clone, Copy, Debug)]
pub struct OrdF(pub f64);
impl PartialEq for OrdF {
    fn eq(&self, o: &Self) -> bool {
        self.0 == o.0
    }
}
impl Eq for OrdF {}
impl PartialOrd for OrdF {
    fn partial_cmp(&self, o: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for OrdF {
    fn cmp(&self, o: &Self) -> std::cmp::Ordering {
        self.0.partial_cmp(&o.0).unwrap_or(std::cmp::Ordering::Equal)
    }
}

fn bisect_left(window: &[f64], value: f64) -> usize {
    let (mut lo, mut hi) = (0usize, window.len());
    while lo < hi {
        let mid = (lo + hi) / 2;
        if window[mid] < value {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    lo
}

/// _axis_lines(points, heading).
pub fn axis_lines(points: &[(f64, f64)], heading: f64) -> Vec<Line> {
    let (c, s) = ((-heading).pcos(), (-heading).psin());
    let mut rows: Vec<(f64, f64)> = points.iter().map(|&(x, y)| (-s * x + c * y, c * x + s * y)).collect();
    rows.sort_by(|a, b| OrdF(a.0).cmp(&OrdF(b.0)).then(OrdF(a.1).cmp(&OrdF(b.1))));
    let mut lines: Vec<Line> = Vec::new();
    let mut start = 0usize;
    let mut window: Vec<f64> = Vec::new();
    let mut wide: BTreeSet<(OrdF, OrdF)> = BTreeSet::new();
    // A Python set keeps the first inserted of equal keys; BTreeSet::insert
    // keeps the existing element too.
    for end in 0..rows.len() {
        let value = rows[end].1;
        let j = bisect_left(&window, value);
        if 0 < j && j < window.len() && window[j] - window[j - 1] > OPEN_LINE_GAP_M {
            wide.remove(&(OrdF(window[j - 1]), OrdF(window[j])));
        }
        if j > 0 && value - window[j - 1] > OPEN_LINE_GAP_M {
            wide.insert((OrdF(window[j - 1]), OrdF(value)));
        }
        if j < window.len() && window[j] - value > OPEN_LINE_GAP_M {
            wide.insert((OrdF(value), OrdF(window[j])));
        }
        window.insert(j, value);
        while rows[end].0 - rows[start].0 > OPEN_LINE_BAND_M {
            let value = rows[start].1;
            let j = bisect_left(&window, value);
            let left = if j > 0 { Some(window[j - 1]) } else { None };
            let right = if j + 1 < window.len() { Some(window[j + 1]) } else { None };
            if let Some(l) = left
                && value - l > OPEN_LINE_GAP_M
            {
                wide.remove(&(OrdF(l), OrdF(value)));
            }
            if let Some(r) = right
                && r - value > OPEN_LINE_GAP_M
            {
                wide.remove(&(OrdF(value), OrdF(r)));
            }
            if let (Some(l), Some(r)) = (left, right)
                && r - l > OPEN_LINE_GAP_M
            {
                wide.insert((OrdF(l), OrdF(r)));
            }
            window.remove(j);
            start += 1;
        }
        let count = end + 1 - start;
        if count < OPEN_LINE_MIN_POINTS || window[window.len() - 1] - window[0] < OPEN_LANE_MIN_SPAN_M {
            continue;
        }
        let lateral = rows[start + count / 2].0;
        let scale = pmax(lateral.abs(), 0.3);
        let mut runs: Vec<(usize, i64)> = Vec::new();
        let mut first = 0usize;
        for &(OrdF(a), OrdF(b)) in wide.iter() {
            if b - a > pmax(OPEN_LINE_GAP_M, OPEN_LINE_GAP_SCALE * pmax(a * a, b * b) / scale) {
                let k = bisect_left(&window, b);
                runs.push((first, k as i64 - 1));
                first = k;
            }
        }
        runs.push((first, window.len() as i64 - 1));
        for &(i0, i1) in &runs {
            let n = i1 - i0 as i64 + 1;
            if n < OPEN_LINE_MIN_POINTS as i64 {
                continue;
            }
            let i1 = i1 as usize;
            if window[i1] - window[i0] < OPEN_LANE_MIN_SPAN_M {
                continue;
            }
            let line = Line { lateral_m: lateral, along_m: (window[i0], window[i1]), points: n as usize, source: "raw_returns" };
            let same: Vec<usize> = lines
                .iter()
                .enumerate()
                .filter(|(_, old)| {
                    (old.lateral_m - lateral).abs() <= OPEN_LINE_BAND_M && old.along_m.0 <= window[i1] && window[i0] <= old.along_m.1
                })
                .map(|(i, _)| i)
                .collect();
            if same.is_empty() {
                lines.push(line);
            } else if n as usize > lines[same[0]].points {
                lines[same[0]] = line;
            }
        }
    }
    lines
}

/// _support_depth(ranges, bins).
pub fn support_depth(ranges: &[Option<f64>], bins: &[i64]) -> f64 {
    let values: Vec<f64> =
        bins.iter().filter_map(|&i| ranges[imod(i, 360) as usize]).filter(|r| r.is_finite()).collect();
    if values.is_empty() {
        0.
    } else {
        values.iter().cloned().fold(f64::NEG_INFINITY, pmax) - values.iter().cloned().fold(INF, pmin)
    }
}

/// _on_board_run(point, lines, heading).
pub fn on_board_run(point: (f64, f64), lines: &[Line], heading: f64) -> bool {
    let (c, s) = ((-heading).pcos(), (-heading).psin());
    let (x, y) = point;
    let (lateral, along) = (-s * x + c * y, c * x + s * y);
    lines.iter().any(|line| {
        (line.lateral_m - lateral).abs() <= OPEN_BOARD_END_LATERAL_M
            && line.along_m.0 - OPEN_BOARD_END_ALONG_M <= along
            && along <= line.along_m.1 + OPEN_BOARD_END_ALONG_M
    })
}

/// CPython's iteration order of a set display of small ints ({a, b, ...}).
pub fn py_int_set_order(values: &[i64]) -> Vec<i64> {
    // Open addressing as in Objects/setobject.c: linear probes (9), then
    // perturbation; resize to used*4 when fill*5 >= mask*3.
    const LINEAR_PROBES: usize = 9;
    const PERTURB_SHIFT: u32 = 5;
    fn insert(table: &mut [Option<i64>], value: i64) -> bool {
        let mask = table.len() - 1;
        let hash = value as u64; // hash(n) == n for small non-negative ints
        let mut perturb = hash;
        let mut i = (hash as usize) & mask;
        loop {
            let probes = if i + LINEAR_PROBES <= mask { LINEAR_PROBES } else { 0 };
            for j in 0..=probes {
                let slot = (i + j) & mask;
                match table[slot] {
                    None => {
                        table[slot] = Some(value);
                        return true;
                    }
                    Some(v) if v == value => return false,
                    _ => {}
                }
            }
            perturb >>= PERTURB_SHIFT;
            i = (i * 5 + 1 + perturb as usize) & mask;
        }
    }
    let mut table: Vec<Option<i64>> = vec![None; 8];
    let mut fill = 0usize;
    for &v in values {
        if insert(&mut table, v) {
            fill += 1;
            let mask = table.len() - 1;
            if fill * 5 >= mask * 3 {
                let minused = fill * 4;
                let mut newsize = 8;
                while newsize <= minused {
                    newsize <<= 1;
                }
                let old: Vec<i64> = table.iter().flatten().cloned().collect();
                table = vec![None; newsize];
                for v in old {
                    insert(&mut table, v);
                }
            }
        }
    }
    table.into_iter().flatten().collect()
}
