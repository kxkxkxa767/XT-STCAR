//! autonomy_live.probe_clearance and what it uses (round 97): the console's
//! current-scan body-clearance check, line for line, and ClearanceReuse.
//! Same results, raised errors and key order as Python; a known-body stop
//! (ProbeClearanceError) comes back as `Err(clearance)`, every other refusal
//! raises ValueError with Python's message.

use std::f64::consts::PI;

use crate::compact_target::contact_supports;
use crate::dict;
use crate::helpers::*;
use crate::py::*;

pub const CONTROL_AGE_LIMIT_S: f64 = 0.20;
pub const CAMERA_LATE_S: f64 = 0.50;
pub const FRONT_MAX_UNKNOWN: usize = 6;
pub const FRONT_MAX_GAP: i64 = 3;
pub const FRONT_BRIDGE_MAX_BINS: i64 = 5;
pub const FRONT_BRIDGE_MAX_STEP_M: f64 = 0.15;
pub const FRONT_LANE_WALL_MAX_RANGE_M: f64 = 6.;
pub const MANEUVER_TOLERABLE_MIN_VALID: usize = 324;
pub const MANEUVER_TOLERABLE_MAX_NULL_RUN: i64 = 12;
pub const MANEUVER_TOLERABLE_FRONT_RUN: i64 = FRONT_BRIDGE_MAX_BINS;
pub const MANEUVER_TOLERABLE_FRONT_UNKNOWN: usize = 15;
pub const CORRIDOR_HALF_WIDTH_M: f64 = 0.30;
pub const TURNING_LEFT_MIN_NET_M: f64 = 0.01;
pub const TURNING_LEFT_BODY_CLEARANCE_M: f64 = TURNING_LEFT_MIN_NET_M + LIDAR_RANGE_ALLOWANCE_M;

const SIDES: [&str; 2] = ["left", "right"];

/// `min(values)` of Python numbers: the first of equal minima, its own type.
fn first_min<'a>(values: impl IntoIterator<Item = &'a V>) -> V {
    let mut best: Option<&V> = None;
    for value in values {
        if best.is_none_or(|b| value.f() < b.f()) {
            best = Some(value);
        }
    }
    best.cloned().unwrap_or(V::None)
}

fn first_min_f(values: &[f64]) -> V {
    match values.split_first() {
        None => V::None,
        Some((&first, rest)) => V::Float(rest.iter().fold(first, |a, &b| pmin(a, b))),
    }
}

fn largest_run(angles: &[i64]) -> i64 {
    let (mut run, mut largest) = (0, 0);
    for angle in -30..31 {
        run = if angles.contains(&angle) { run + 1 } else { 0 };
        if run > largest {
            largest = run;
        }
    }
    largest
}

fn hit_angles(evidence: &V) -> Vec<i64> {
    evidence.idx("hits").l().iter().map(|hit| hit.idx("angle_deg").i()).collect()
}

/// _front_lane_wall_projection(bins, unknown, lane).
fn front_lane_wall_projection(bins: &[Option<f64>], unknown: &[i64], lane: &V) -> V {
    if !valid_candidate_default(lane) || lane.idx("heading_left_rad").f().abs() >= PI / 2. {
        value_error("probe_invalid_front_lane");
    }
    let heading = lane.idx("heading_left_rad").f();
    let (c, s) = (heading.pcos(), heading.psin());
    let left_rho = lane.idx("center_offset_left_m").f() + lane.idx("width_m").f() / 2.;
    let right_rho = lane.idx("center_offset_left_m").f() - lane.idx("width_m").f() / 2.;
    let through_margin = pmax(MANEUVER_BODY_CLEARANCE_M, 2. * lane.idx("fit_error_m").f());
    let mut limits = [FRONT_LANE_WALL_MAX_RANGE_M; 2];
    let mut hits: Vec<(i64, usize, f64, f64)> = Vec::new();
    for &angle in unknown {
        let i = imod(angle, 360) as usize;
        let normal = -s * dcos(i) - c * dsin(i);
        if normal.abs() <= 1e-12 {
            continue;
        }
        let (side, rho) = if normal > 0. { (0, left_rho) } else { (1, right_rho) };
        let hit = rho / normal;
        let along = c * dcos(i) - s * dsin(i);
        if 0. < hit && hit <= FRONT_LANE_WALL_MAX_RANGE_M && along > 0. && hit * along < limits[side] {
            hits.push((angle, side, hit, hit * along));
        }
    }
    for (i, r) in bins.iter().enumerate() {
        let Some(r) = *r else { continue };
        let normal = -s * dcos(i) - c * dsin(i);
        let (side, rho, sign) = if normal > 0. { (0, left_rho, 1.) } else { (1, right_rho, -1.) };
        if sign * normal <= 1e-12 || sign * (r * normal - rho) <= through_margin {
            continue;
        }
        let hit = rho / normal;
        let along = c * dcos(i) - s * dsin(i);
        if 0. < hit && hit <= FRONT_LANE_WALL_MAX_RANGE_M && along > 0. {
            limits[side] = pmin(limits[side], hit * along);
        }
    }
    let hits: Vec<V> = hits
        .into_iter()
        .filter(|h| h.3 < limits[h.1])
        .map(|(angle, side, range, along)| {
            dict! {"angle_deg" => angle, "side" => SIDES[side], "range_m" => range, "forward_along_lane_m" => along}
        })
        .collect();
    dict! {
        "basis" => "current_native_lane_side_wall_plane_projection",
        "occupied_projection" => true, "free_space_certified" => false,
        "max_range_m" => FRONT_LANE_WALL_MAX_RANGE_M,
        "known_opening_limit_along_lane_m" => dict! {"left" => limits[0], "right" => limits[1]},
        "lane" => lane.clone(), "hits" => V::list(hits),
    }
}

/// _front_side_wall_projection(bins, unknown).
fn front_side_wall_projection(bins: &[Option<f64>], unknown: &[i64]) -> V {
    let fits = [("right", side_wall_fit(bins, "right")), ("left", side_wall_fit(bins, "left"))];
    let mut limits = V::dict();
    let mut hits = Vec::new();
    for (side, fit) in &fits {
        if fit.is_none() {
            continue;
        }
        let (s, c) = (fit.idx("slope").f(), fit.idx("intercept_m").f());
        let sign = if *side == "right" { 1. } else { -1. };
        let margin = pmax(MANEUVER_BODY_CLEARANCE_M, 2. * fit.idx("fit_error_m").f());
        let mut limit = FRONT_LANE_WALL_MAX_RANGE_M;
        for (i, r) in bins.iter().enumerate() {
            let Some(r) = *r else { continue };
            let (x, y) = (r * dcos(i), r * dsin(i));
            let toward = sign * (dsin(i) - s * dcos(i));
            if x > 0. && toward > 1e-9 && sign * (y - s * x - c) > margin {
                let hit = c / (dsin(i) - s * dcos(i));
                if 0. < hit && hit <= FRONT_LANE_WALL_MAX_RANGE_M {
                    limit = pmin(limit, hit * dcos(i));
                }
            }
        }
        limits.set(side, limit);
        for &angle in unknown {
            let i = imod(angle, 360) as usize;
            let toward = sign * (dsin(i) - s * dcos(i));
            if toward <= 1e-9 {
                continue;
            }
            let hit = c / (dsin(i) - s * dcos(i));
            let along = hit * dcos(i);
            if 0. < hit && hit <= FRONT_LANE_WALL_MAX_RANGE_M && along > 0. && along < limit {
                hits.push(dict! {"angle_deg" => angle, "side" => *side, "range_m" => hit, "forward_m" => along});
            }
        }
    }
    dict! {
        "basis" => "current_frame_side_board_projection", "occupied_projection" => true,
        "free_space_certified" => false, "max_range_m" => FRONT_LANE_WALL_MAX_RANGE_M,
        "fits" => dict! {"right" => fits[0].1.clone(), "left" => fits[1].1.clone()},
        "known_opening_limit_forward_m" => limits, "hits" => V::list(hits),
    }
}

/// front_gap_evidence(bins, bridge=, front_lane=, side_walls=); `front_lane`
/// is None or the lane.
pub fn front_gap_evidence(bins: &[Option<f64>], bridge: bool, front_lane: &V, side_walls: bool) -> V {
    let at = |angle: i64| bins[imod(angle, 360) as usize];
    let unknown: Vec<i64> = (-30..31).filter(|&a| at(a).is_none()).collect();
    let largest_gap = largest_run(&unknown);
    let mut bridged: Vec<i64> = Vec::new();
    if bridge {
        let mut start: Option<i64> = None;
        for angle in -30..32 {
            let missing = angle <= 30 && at(angle).is_none();
            if missing && start.is_none() {
                start = Some(angle);
            } else if !missing && let Some(first) = start {
                if angle - first <= FRONT_BRIDGE_MAX_BINS
                    && let (Some(left), Some(right)) = (at(first - 1), at(angle))
                    && (left - right).abs() <= FRONT_BRIDGE_MAX_STEP_M
                {
                    bridged.extend(first..angle);
                }
                start = None;
            }
        }
    }
    let mut open_unknown: Vec<i64> = unknown.iter().copied().filter(|a| !bridged.contains(a)).collect();
    let mut wall_evidence = V::None;
    if front_lane.is_some() && side_walls {
        value_error("probe_invalid_front_lane");
    }
    if front_lane.is_some() && !open_unknown.is_empty() {
        wall_evidence = front_lane_wall_projection(bins, &open_unknown, front_lane);
        let projected = hit_angles(&wall_evidence);
        open_unknown.retain(|a| !projected.contains(a));
    } else if side_walls && !open_unknown.is_empty() {
        wall_evidence = front_side_wall_projection(bins, &open_unknown);
        let projected = hit_angles(&wall_evidence);
        open_unknown.retain(|a| !projected.contains(a));
    }
    let largest_open_gap = largest_run(&open_unknown);
    let mut result = dict! {
        "front_unknown_bins" => V::from(unknown), "largest_gap_deg" => largest_gap,
        "front_bridged_bins" => V::from(bridged), "front_open_unknown_bins" => V::from(open_unknown),
        "largest_open_gap_deg" => largest_open_gap,
    };
    if wall_evidence.is_some() {
        result.set("front_wall_projected_bins", V::from(hit_angles(&wall_evidence)));
        result.set("front_wall_projection", wall_evidence);
    }
    result
}

/// _body_contact_bins(target, scan, ages, maneuver): the exempt bins.
fn body_contact_bins(target: &V, scan: &V, ages: &V, maneuver: bool) -> Vec<bool> {
    let mut exempt = vec![false; 360];
    if target.is_none() {
        return exempt;
    }
    let role = if target.is_dict() { target.get("role") } else { &V::None };
    let cone_like = role.os() == Some("cone_like_support");
    let t = |key: &str| target.get(key);
    let lidar = ages.idx("lidar").f();
    if !maneuver
        || !target.is_dict()
        || scan.get("frame_id").os() != Some(FRAME)
        || !matches!(role.os(), Some("first_target" | "second_target" | "cone_like_support"))
        || (!cone_like && (!t("track_id").is_int() || t("track_id").i() <= 0 || !t("confirmed").is_true()))
        || (cone_like && (t("track_id").is_some() || !t("confirmed").is_bool()))
        || !t("full_observation").is_true()
        || t("semantic_class").os() != Some("unknown")
        || !t("source_seq").is_int()
        || t("source_seq").i() < 0
        || t("source_seq").i() != scan.idx("seq").i()
        || !t("source_at_ms").is_int()
        || t("source_at_ms").i() < 0
        || !scan.get("at_ms").is_int()
        || t("source_at_ms").i() != scan.get("at_ms").i()
        || !t("source_received_at").is_real()
        || !t("source_received_at").f().is_finite()
        || t("source_received_at").f() < 0.
        || !scan.get("received_at").is_real()
        || !py_eq(t("source_received_at"), scan.get("received_at"))
        || !(0. <= lidar && lidar < LIDAR_LATE_S)
    {
        value_error("probe_invalid_body_contact_target");
    }
    let (support, observed) = (t("support_bins"), t("support_ranges_m"));
    let ranges = scan.idx("ranges").l();
    let valid = match (support, observed) {
        (V::List(bins), V::List(seen)) => {
            (4..=360).contains(&bins.len())
                && bins.iter().all(|i| i.is_int() && (0..360).contains(&i.i()))
                && {
                    let mut seen_bins = vec![false; 360];
                    bins.iter().all(|i| !std::mem::replace(&mut seen_bins[i.i() as usize], true))
                }
                && seen.len() == bins.len()
                && bins.iter().zip(seen.iter()).all(|(i, r)| {
                    r.is_num() && (0.02..=12.).contains(&r.f()) && py_eq(&ranges[i.i() as usize], r)
                })
        }
        _ => false,
    };
    if !valid {
        value_error("probe_invalid_body_contact_target");
    }
    for i in support.l() {
        exempt[i.i() as usize] = true;
    }
    if cone_like {
        // User 2026-10-12: cone contact continues, boards stop. Recompute the
        // cone-like supports from this raw scan; the witness must name them all.
        let mut current = vec![false; 360];
        for candidate in contact_supports(scan.idx("ranges")) {
            for i in candidate.idx("support_bins").l() {
                current[i.i() as usize] = true;
            }
        }
        if current != exempt {
            value_error("probe_invalid_body_contact_target");
        }
    }
    exempt
}

struct BodyPoint {
    angle: usize,
    range: V,
    x: f64,
    left_y: f64,
    distance: f64,
    stop: f64,
    relaxed: bool,
}

impl BodyPoint {
    fn value(&self) -> V {
        dict! {
            "angle_deg" => self.angle, "range_m" => self.range.clone(), "forward_x_m" => self.x,
            "left_y_m" => self.left_y, "body_distance_m" => self.distance, "stop_distance_m" => self.stop,
            "relaxed_turn_left" => self.relaxed,
        }
    }
}

fn kwarg<'a>(kw: &'a V, key: &str, default: &'a V) -> &'a V {
    get_or(kw, key, default)
}

/// probe_clearance(scan, ages, demo, centering, **kw). Err: the known-body
/// stop (ProbeClearanceError('probe_obstacle_close_body', clearance)).
pub fn probe_clearance(scan: &V, ages: &V, demo: &V, centering: &V, kw: &V) -> Result<V, V> {
    let (f, t, none) = (V::Bool(false), V::Bool(true), V::None);
    let straight = V::str("straight");
    let rear_launch = kwarg(kw, "rear_launch", &f);
    let camera_required = kwarg(kw, "camera_required", &t);
    let clearance_profile = kwarg(kw, "clearance_profile", &straight);
    let relaxed_turn_left = kwarg(kw, "relaxed_turn_left", &f);
    let body_contact_target = kwarg(kw, "body_contact_target", &none);
    let front_lane = kwarg(kw, "front_lane", &none);
    let rear_start = kwarg(kw, "rear_start", &f);
    let front_side_walls = kwarg(kw, "front_side_walls", &f);

    if !matches!(clearance_profile.os(), Some("straight" | "maneuver")) {
        value_error("invalid_clearance_profile");
    }
    if !camera_required.is_bool() {
        value_error("invalid_camera_requirement");
    }
    let maneuver = clearance_profile.os() == Some("maneuver");
    if !relaxed_turn_left.is_bool() || (relaxed_turn_left.is_true() && !maneuver) {
        value_error("invalid_relaxed_turn_clearance");
    }
    let camera = camera_required.is_true();
    let sensor_inputs: &[&str] = if camera { &["camera", "lidar", "control"] } else { &["lidar", "control"] };
    if sensor_inputs.iter().any(|k| !ages.get(k).is_num() || ages.get(k).f() < 0.) {
        value_error("probe_invalid_sensor_age");
    }
    if ages.idx("control").f() >= CONTROL_AGE_LIMIT_S {
        value_error("probe_sensor_unavailable");
    }
    let mut issues: Vec<&str> = Vec::new();
    if camera && ages.idx("camera").f() >= CAMERA_LATE_S {
        issues.push("camera_late");
    }
    if camera && ages.idx("camera").f() >= 1. {
        issues.push("camera_unavailable");
    }
    let lidar_age = ages.idx("lidar").f();
    if lidar_age >= LIDAR_LATE_S {
        issues.push("lidar_late");
    }
    if !scan.is_dict() || !scan.get("seq").is_int() {
        value_error("probe_invalid_scan");
    }
    if !demo.truthy() && scan.get("frame_id").os() != Some(FRAME) {
        value_error("probe_unknown_lidar_heading");
    }
    let bins = match scan.get("ranges") {
        V::List(list) if list.len() == 360 => list.clone(),
        _ => value_error("probe_invalid_scan"),
    };
    let mut valid: Vec<&V> = Vec::new();
    for r in bins.iter() {
        if r.is_none() {
            continue;
        }
        if !r.is_num() || !(0.02..=12.).contains(&r.f()) {
            value_error("probe_invalid_scan");
        }
        valid.push(r);
    }
    let fb: Vec<Option<f64>> = bins.iter().map(|r| r.of()).collect();
    // A known obstacle remains a hard stop even while coverage is degraded.
    let nearest = first_min(valid.iter().copied());
    let mut corridor: Vec<f64> = Vec::new();
    let mut side_clearances: Vec<f64> = Vec::new();
    if !rear_start.is_false() && (!rear_start.is_true() || !maneuver) {
        value_error("probe_invalid_rear_start");
    }
    let contact_bins = body_contact_bins(body_contact_target, scan, ages, maneuver);
    let mut rear_start_exempt: Vec<usize> = Vec::new();
    let mut tolerated_contacts: Vec<usize> = Vec::new();
    let half_width = CORRIDOR_HALF_WIDTH_M + if centering.truthy() { 0.02 } else { 0. };
    let mut points: Vec<BodyPoint> = Vec::new();
    let mut nearest_body_point: Option<usize> = None;
    let mut stop_trigger_point: Option<usize> = None;
    let mut body_stop_requested = false;
    for (angle, r) in bins.iter().enumerate() {
        if r.is_none() {
            continue;
        }
        let rf = r.f();
        let (mut x, y) = (rf * dcos(angle), rf * dsin(angle));
        // These bins are exactly lateral (autonomy_live: cos(pi/2) rounds
        // positive, cos(3*pi/2) negative); neither is a forward return.
        if angle == 90 || angle == 270 {
            x = 0.;
        }
        let dx = pmax(pmax(-REAR_BODY_EXTENT_M - x, 0.), x - FRONT_BODY_EXTENT_M);
        let dy = pmax(y.abs() - SIDE_BODY_EXTENT_M, 0.);
        let body_distance = hypot(dx, dy);
        let relaxed_point = relaxed_turn_left.is_true() && -y >= SIDE_BODY_EXTENT_M;
        let point_stop_m = if relaxed_point { TURNING_LEFT_BODY_CLEARANCE_M } else { MANEUVER_BODY_CLEARANCE_M };
        let new_point = |points: &mut Vec<BodyPoint>| {
            points.push(BodyPoint {
                angle,
                range: r.clone(),
                x,
                left_y: -y,
                distance: body_distance,
                stop: point_stop_m,
                relaxed: relaxed_point,
            });
            points.len() - 1
        };
        let mut body_point = None;
        if nearest_body_point.is_none_or(|n| body_distance < points[n].distance) {
            let index = new_point(&mut points);
            body_point = Some(index);
            nearest_body_point = Some(index);
        }
        if maneuver {
            // Equality is allowed; tolerate only floating point roundoff at that boundary.
            if body_distance < point_stop_m && !isclose(body_distance, point_stop_m, 0., 1e-12) {
                let index = match body_point {
                    Some(index) => index,
                    None => new_point(&mut points),
                };
                if contact_bins[angle] {
                    tolerated_contacts.push(index);
                } else if rear_start.is_true() && x < -REAR_BODY_EXTENT_M + LIDAR_RANGE_ALLOWANCE_M {
                    // User 2026-10-18: the start-end wall behind the rear face
                    // (autonomy_live has the full argument). Reported, never a stop.
                    rear_start_exempt.push(index);
                } else {
                    body_stop_requested = true;
                    if stop_trigger_point
                        .is_none_or(|s| body_distance - point_stop_m < points[s].distance - points[s].stop)
                    {
                        stop_trigger_point = Some(index);
                    }
                }
            }
            if y.abs() > x.abs() {
                side_clearances.push(y.abs());
            }
        } else if rear_launch.truthy() && (115..=245).contains(&angle) {
            // Only a bounded neutral-steering forward launch may exempt rear rays.
        } else if y.abs() > x.abs() {
            side_clearances.push(y.abs());
            if y.abs() < SIDE_CLEARANCE_M {
                value_error("probe_obstacle_close_side");
            }
        } else if rf < 0.40 {
            value_error("probe_obstacle_close");
        }
        if x > 0. && y.abs() <= half_width {
            corridor.push(x);
        }
    }
    let corridor_front = first_min_f(&corridor);
    if !maneuver && corridor_front.is_some() && corridor_front.f() < CORRIDOR_LOOKAHEAD_M {
        value_error("probe_obstacle_in_straight_corridor");
    }
    if valid.len() < 342 {
        issues.push("scan_incomplete");
    }
    let (mut longest_null_run, mut run) = (0i64, 0i64);
    for r in bins.iter().chain(bins.iter()) {
        run = if r.is_none() { run + 1 } else { 0 };
        if run > longest_null_run {
            longest_null_run = run.min(360);
        }
    }
    let incomplete_tolerable = maneuver
        && (MANEUVER_TOLERABLE_MIN_VALID..342).contains(&valid.len())
        && longest_null_run <= MANEUVER_TOLERABLE_MAX_NULL_RUN;
    let front: Vec<&V> = (-30..31).map(|i| &bins[imod(i, 360) as usize]).collect();
    if front_lane.is_some() {
        let invalid = !maneuver || {
            let empty = V::list(Vec::new());
            let candidates = if scan.has("corridor_candidates") { scan.get("corridor_candidates") } else { &empty };
            match candidates {
                V::List(list) => !list.iter().any(|c| py_eq(c, front_lane)),
                other => type_error(format!("argument of type '{}' is not a container", other.type_name())),
            }
        };
        if invalid {
            value_error("probe_invalid_front_lane");
        }
    }
    if !front_side_walls.is_false() && (!front_side_walls.is_true() || !maneuver || front_lane.is_some()) {
        value_error("probe_invalid_front_lane");
    }
    let fresh = lidar_age < LIDAR_LATE_S;
    let front_evidence =
        front_gap_evidence(&fb, maneuver, if fresh { front_lane } else { &none }, front_side_walls.is_true() && fresh);
    let open_unknown = front_evidence.idx("front_open_unknown_bins").len();
    let largest_open_gap = front_evidence.idx("largest_open_gap_deg").i();
    if open_unknown > FRONT_MAX_UNKNOWN || largest_open_gap > FRONT_MAX_GAP {
        issues.push("front_sparse");
    }
    let front_sparse_tolerable = maneuver
        && issues.contains(&"front_sparse")
        && largest_open_gap <= MANEUVER_TOLERABLE_FRONT_RUN
        && open_unknown <= MANEUVER_TOLERABLE_FRONT_UNKNOWN;
    let front_values: Vec<&V> = front.iter().copied().filter(|r| r.is_some()).collect();
    let nearest_relaxed = nearest_body_point.is_some_and(|n| points[n].relaxed);
    let nearest_stop_m = if nearest_relaxed { TURNING_LEFT_BODY_CLEARANCE_M } else { MANEUVER_BODY_CLEARANCE_M };
    let relaxed = relaxed_turn_left.is_true();
    let when = |active: bool, value: V| if active { value } else { V::None };
    let point = |index: Option<usize>| index.map(|i| points[i].value()).unwrap_or(V::None);
    let point_list = |indices: &[usize]| V::list(indices.iter().map(|&i| points[i].value()).collect());
    let body_proximity = dict! {
        "scope" => "current_known_lidar_returns_only",
        "coordinate_convention" => "x_forward_y_left",
        "ray_bin_convention" => "clockwise_from_forward",
        "body_extents_source" => "user_measured_lidar_to_front_rear_and_outer_tyre_edges",
        "body_extents_measurement_date" => "2026-10-07",
        "body_extents_measurement_quality" => "operator_rough_measurement",
        "body_extents_error_bound_m" => V::None,
        "body_extents_m" => dict! {"front" => FRONT_BODY_EXTENT_M, "rear" => REAR_BODY_EXTENT_M,
                                    "left" => SIDE_BODY_EXTENT_M, "right" => SIDE_BODY_EXTENT_M},
        "body_rectangle_m" => dict! {"x_min" => -REAR_BODY_EXTENT_M, "x_max" => FRONT_BODY_EXTENT_M,
                                      "y_min" => -SIDE_BODY_EXTENT_M, "y_max" => SIDE_BODY_EXTENT_M},
        "min_net_clearance_m" => when(maneuver, V::Float(if nearest_relaxed { TURNING_LEFT_MIN_NET_M } else { MANEUVER_MIN_NET_M })),
        "min_net_clearance_source" => when(maneuver, V::str(if nearest_relaxed {
            "operator_authorized_turn_left_total4cm"
        } else {
            "operator_selected_maneuver_5cm"
        })),
        "lidar_range_allowance_m" => LIDAR_RANGE_ALLOWANCE_M,
        "stop_distance_m" => when(maneuver, V::Float(nearest_stop_m)),
        "default_stop_distance_m" => when(maneuver, V::Float(MANEUVER_BODY_CLEARANCE_M)),
        "relaxed_turn_left_active" => relaxed_turn_left.clone(),
        "turning_left_stop_distance_m" => when(relaxed, V::Float(TURNING_LEFT_BODY_CLEARANCE_M)),
        "stop_requested" => when(maneuver, V::Bool(body_stop_requested)),
        "current_known_min_distance_m" => nearest_body_point.map(|n| V::Float(points[n].distance)).unwrap_or(V::None),
        "nearest_point" => point(nearest_body_point),
        "stop_trigger_point" => point(stop_trigger_point),
        "target_contact_tolerated" => !tolerated_contacts.is_empty(),
        "target_contact_points" => point_list(&tolerated_contacts),
        "target_contact_evidence" => body_contact_target.clone(),
        "rear_start_exempt_points" => point_list(&rear_start_exempt),
        "axis_stop_thresholds_m" => when(maneuver, dict! {
            "front" => FRONT_BODY_EXTENT_M + MANEUVER_BODY_CLEARANCE_M,
            "rear" => REAR_BODY_EXTENT_M + MANEUVER_BODY_CLEARANCE_M,
            "left" => SIDE_BODY_EXTENT_M + if relaxed { TURNING_LEFT_BODY_CLEARANCE_M } else { MANEUVER_BODY_CLEARANCE_M },
            "right" => SIDE_BODY_EXTENT_M + MANEUVER_BODY_CLEARANCE_M,
        }),
        "free_space_certified" => false, "swept_path_certified" => false,
        "standstill_certified" => false,
    };
    let legacy: Vec<V> = if maneuver {
        ["corridor_front_m", "corridor_half_width_m", "corridor_lookahead_m", "side_clearance_m", "side_stop_threshold_m"]
            .iter()
            .map(|s| V::str(s))
            .collect()
    } else {
        Vec::new()
    };
    let mut clearance = dict! {
        "front_m" => first_min(front_values.iter().copied()),
        "clearance_profile" => clearance_profile.clone(),
        "clearance_gates" => dict! {
            "straight_corridor_active" => !maneuver, "side_lateral_active" => !maneuver, "radial_active" => !maneuver,
            "current_body_proximity_active" => maneuver,
            "relaxed_turn_left_active" => relaxed_turn_left.clone(),
            "rear_launch_exemption_active" => rear_launch.truthy() && !maneuver,
            "rear_start_exemption_active" => rear_start.clone(),
        },
        "body_proximity" => body_proximity,
        "legacy_diagnostic_only_fields" => V::list(legacy),
        "sensor_inputs" => V::list(sensor_inputs.iter().map(|s| V::str(s)).collect()),
        "camera_required" => camera_required.clone(),
        "corridor_front_m" => corridor_front,
        "nearest_m" => nearest,
        "scan_seq" => scan.idx("seq").clone(),
        "side_clearance_m" => first_min_f(&side_clearances),
        "side_stop_threshold_m" => if maneuver { SIDE_BODY_EXTENT_M + MANEUVER_BODY_CLEARANCE_M } else { SIDE_CLEARANCE_M },
        "side_body_extent_m" => SIDE_BODY_EXTENT_M,
        "side_min_net_m" => if maneuver { MANEUVER_MIN_NET_M } else { SIDE_MIN_NET_M },
        "lidar_range_allowance_m" => LIDAR_RANGE_ALLOWANCE_M,
    };
    clearance.update(&front_evidence);
    let motion_ready = !body_stop_requested
        && !["camera_unavailable", "lidar_late", "scan_incomplete", "front_sparse"].iter().any(|i| issues.contains(i));
    clearance.update(&dict! {
        "quality_issues" => V::list(issues.iter().map(|s| V::str(s)).collect()),
        "valid_returns" => valid.len(), "longest_null_run_bins" => longest_null_run,
        "scan_incomplete_tolerable" => incomplete_tolerable,
        "front_sparse_tolerable" => front_sparse_tolerable,
        "motion_ready" => motion_ready,
        "corridor_half_width_m" => half_width,
        "corridor_lookahead_m" => CORRIDOR_LOOKAHEAD_M,
    });
    if body_stop_requested { Err(clearance) } else { Ok(clearance) }
}

/// _age_sides(ages, camera_required).
fn age_sides(ages: &V, camera_required: &V) -> Option<V> {
    if !camera_required.is_bool() || !ages.is_dict() {
        return None;
    }
    let camera = camera_required.is_true();
    let names: &[&str] = if camera { &["camera", "lidar", "control"] } else { &["lidar", "control"] };
    if names.iter().any(|k| !ages.get(k).is_num() || ages.get(k).f() < 0.) {
        return None;
    }
    let mut sides = vec![
        V::Bool(ages.idx("control").f() >= CONTROL_AGE_LIMIT_S),
        V::Bool(ages.idx("lidar").f() >= LIDAR_LATE_S),
    ];
    if camera {
        sides.push(V::Bool(ages.idx("camera").f() >= CAMERA_LATE_S));
        sides.push(V::Bool(ages.idx("camera").f() >= 1.));
    }
    Some(V::list(sides))
}

/// autonomy_live._same: equal with equal types throughout; dict keys as a set.
fn same_typed(a: &V, b: &V) -> bool {
    match (a, b) {
        (V::None, V::None) => true,
        (V::Bool(x), V::Bool(y)) => x == y,
        (V::Int(x), V::Int(y)) => x == y,
        (V::Float(x), V::Float(y)) => x == y,
        (V::Str(x), V::Str(y)) => x == y,
        (V::List(x), V::List(y)) => x.len() == y.len() && x.iter().zip(y.iter()).all(|(p, q)| same_typed(p, q)),
        (V::Dict(x), V::Dict(y)) => {
            x.len() == y.len()
                && x.iter().all(|(k, v)| y.iter().find(|(k2, _)| k2 == k).is_some_and(|(_, w)| same_typed(v, w)))
        }
        _ => false,
    }
}

pub enum Probe {
    /// The stored result again (the console keeps that result).
    Reused,
    /// A new result; `stored`: kept for reuse.
    Fresh { clearance: V, stored: bool },
}

/// ClearanceReuse: the last result again for a scan equal to the one it was
/// checked on, the same arguments (equal types) and the same side of every
/// sensor age limit; a check that raised is never reused. The result itself
/// stays with the console (it hands out a shallow copy, as Python does);
/// `token` names the stored one.
#[derive(Default)]
pub struct ClearanceReuse {
    scan: V,
    inputs: V,
    stored: bool,
    pub reused: i64,
    pub token: i64,
}

impl ClearanceReuse {
    pub fn probe(&mut self, scan: &V, ages: &V, demo: &V, centering: &V, kw: &V) -> Result<Probe, V> {
        let sides = age_sides(ages, get_or(kw, "camera_required", &V::Bool(true)));
        let inputs = sides.map(|sides| V::list(vec![sides, demo.clone(), centering.clone(), kw.clone()]));
        if let Some(inputs) = &inputs
            && self.stored
            && py_eq(scan, &self.scan)
            && same_typed(inputs, &self.inputs)
        {
            self.reused += 1;
            return Ok(Probe::Reused);
        }
        self.scan = V::None;
        self.inputs = V::None;
        self.stored = false;
        let clearance = probe_clearance(scan, ages, demo, centering, kw)?;
        let stored = inputs.is_some();
        if let Some(inputs) = inputs {
            self.scan = scan.clone();
            self.inputs = inputs;
            self.stored = true;
            self.token += 1;
        }
        Ok(Probe::Fresh { clearance, stored })
    }
}
