//! Port of web/vehicle-console/compact_target.py (same names, same rules).

use std::cell::RefCell;
use std::rc::Rc;

use crate::dict;
use crate::py::*;

pub const FRAME: &str = "lidar_origin_coarse_body_heading";
pub const MAX_AGE_S: f64 = 0.30;
pub const MIN_CONFIRM_S: f64 = 0.25;
pub const MIN_POINTS: i64 = 5;
pub const MAX_POINT_GAP_M: f64 = 0.18;
pub const MAX_BOUNDARY_GAP_BINS: i64 = 3;
pub const MIN_DIAMETER_M: f64 = 0.06;
pub const MAX_DIAMETER_M: f64 = 0.35;
pub const MIN_RANGE_M: f64 = 0.35;
pub const MAX_RANGE_M: f64 = 3.;
pub const EXTENDED_MAX_RANGE_M: f64 = 3.5;
pub const CLOSE_MAINTENANCE_MIN_RANGE_M: f64 = 0.02;
pub fn min_bearing_rad() -> f64 {
    radians(45.)
}
pub fn max_bearing_rad() -> f64 {
    radians(135.)
}
pub const TOLERANT_INTERNAL_NULLS: i64 = 2;
pub const TOLERANT_EDGE_BINS: i64 = 2;
pub const TOLERANT_MIN_POINTS: i64 = 4;
pub const TOLERANT_GATE_M_PER_S: f64 = 1.5;
pub fn tolerant_gate_rad_per_s() -> f64 {
    radians(100.)
}
pub const TOLERANT_MAINTENANCE_EDGE_BINS: i64 = 8;
pub const TOLERANT_MAINTENANCE_MIN_POINTS: i64 = 3;
pub fn contact_min_facing_rad() -> f64 {
    radians(45.)
}

pub struct Tables {
    /// _DIRECTIONS: (cos, -sin) of each clockwise bin.
    pub directions: Vec<(f64, f64)>,
}

thread_local! {
    pub static TABLES: Tables = Tables {
        directions: (0..360).map(|i| {
            let a = radians(i as f64);
            (a.pcos(), -a.psin())
        }).collect(),
    };
}

pub fn directions() -> Vec<(f64, f64)> {
    TABLES.with(|t| t.directions.clone())
}

pub fn number(value: &V) -> bool {
    value.is_num()
}

/// Ranges as floats (None for a null bin). Valid only after valid_ranges.
pub type Ranges = Rc<Vec<Option<f64>>>;

thread_local! {
    static RANGES_CACHE: RefCell<Vec<(V, Ranges, bool)>> = const { RefCell::new(Vec::new()) };
}

/// The list's floats and whether it is valid (360 finite returns in .02..12 or None).
pub fn ranges_info(ranges: &V) -> Option<(Ranges, bool)> {
    let V::List(list) = ranges else { return None };
    let hit = RANGES_CACHE.with(|c| {
        c.borrow().iter().find_map(|(v, r, ok)| match v {
            V::List(l) if Rc::ptr_eq(l, list) => Some((r.clone(), *ok)),
            _ => None,
        })
    });
    if hit.is_some() {
        return hit;
    }
    let valid = list.len() == 360
        && list.iter().all(|r| r.is_none() || (r.is_num() && (0.02..=12.).contains(&r.f())));
    let floats: Vec<Option<f64>> =
        list.iter().map(|r| if r.is_real() || r.is_bool() { Some(r.f()) } else { None }).collect();
    let info = (Rc::new(floats), valid);
    RANGES_CACHE.with(|c| {
        let mut c = c.borrow_mut();
        c.insert(0, (ranges.clone(), info.0.clone(), valid));
        c.truncate(6);
    });
    Some(info)
}

pub fn valid_ranges(ranges: &V) -> bool {
    match ranges_info(ranges) {
        Some((_, valid)) => valid,
        None => false,
    }
}

/// Floats of a validated range list.
pub fn floats(ranges: &V) -> Ranges {
    ranges_info(ranges).map(|(r, _)| r).unwrap_or_else(|| type_error("ranges must be a list"))
}

pub fn bearing_range(value: (f64, f64), name: &str) -> (f64, f64) {
    if !(value.0.is_finite() && value.1.is_finite()) || !(-PI <= value.0 && value.0 < value.1 && value.1 <= PI) {
        value_error(format!("{name} must be an ordered bearing interval within [-pi, pi]"))
    }
    value
}

fn point_of(v: &V) -> (f64, f64) {
    let l = v.l();
    (l[0].f(), l[1].f())
}

/// _associated(current, old, slack_s).
pub fn associated(current: &V, old: &V, slack_s: f64) -> bool {
    if old.is_none() {
        return false;
    }
    let mut bearing_delta = (current.idx("bearing_left_rad").f() - old.idx("bearing_left_rad").f()).abs();
    if bearing_delta > PI {
        bearing_delta = 2. * PI - bearing_delta;
    }
    let slack_s = pmax(0., slack_s);
    let distance_gate = 0.30 + TOLERANT_GATE_M_PER_S * slack_s;
    let cp = point_of(current.idx("point_left_m"));
    let op = point_of(old.idx("point_left_m"));
    let near = pmin(hypot(cp.0, cp.1), hypot(op.0, op.1));
    let translation_bearing = if near <= distance_gate { PI } else { asin(div(distance_gate, near)) };
    let bearing_gate = pmax(radians(20.) + tolerant_gate_rad_per_s() * slack_s, translation_bearing);
    dist(cp, op) <= distance_gate
        && bearing_delta <= bearing_gate
        && (current.idx("diameter_m").f() - old.idx("diameter_m").f()).abs() <= 0.12
}

/// rotated_observation(observation, rotation_rad).
pub fn rotated_observation(observation: &V, rotation_rad: Option<f64>) -> V {
    let Some(rotation) = rotation_rad.filter(|r| r.is_finite()) else { return observation.clone() };
    if observation.is_none() {
        return observation.clone();
    }
    let (c, s) = ((-rotation).pcos(), (-rotation).psin());
    let (x, y) = point_of(observation.idx("point_left_m"));
    let point = (c * x - s * y, s * x + c * y);
    let mut out = observation.clone();
    out.set("point_left_m", V::from(point));
    out.set("bearing_left_rad", point.1.atan2(point.0));
    out
}

pub type Points = Vec<Option<(f64, f64)>>;

pub fn scan_points(ranges: &[Option<f64>]) -> Points {
    TABLES.with(|t| {
        ranges.iter().zip(t.directions.iter()).map(|(r, d)| r.map(|r| (r * d.0, r * d.1))).collect()
    })
}

/// _clusters_bridged(points, bridge_nulls).
pub fn clusters_bridged(points: &Points, bridge_nulls: i64) -> Vec<Vec<usize>> {
    let present: Vec<usize> = (0..360).filter(|&i| points[i].is_some()).collect();
    if present.is_empty() {
        return vec![];
    }
    if present.len() == 1 {
        return vec![present];
    }
    let linked = |a: usize, b: usize| -> bool {
        imod(b as i64 - a as i64, 360) <= 1 + bridge_nulls && dist(points[a].unwrap(), points[b].unwrap()) <= MAX_POINT_GAP_M
    };
    let count = present.len();
    let prev = |k: usize| present[(k + count - 1) % count];
    let Some(start) = (0..count).find(|&k| !linked(prev(k), present[k])) else { return vec![] };
    let mut groups = Vec::new();
    let mut group: Vec<usize> = Vec::new();
    for offset in 0..count {
        let k = (start + offset) % count;
        if !group.is_empty() && !linked(prev(k), present[k]) {
            groups.push(std::mem::take(&mut group));
        }
        group.push(present[k]);
    }
    if !group.is_empty() {
        groups.push(group);
    }
    groups
}

/// _clusters(points).
pub fn clusters(points: &Points) -> Vec<Vec<usize>> {
    let cut = |a: usize, b: usize| -> bool {
        match (points[a], points[b]) {
            (Some(p), Some(q)) => dist(p, q) > MAX_POINT_GAP_M,
            _ => true,
        }
    };
    let Some(start) = (0..360).find(|&i| cut((i + 359) % 360, i)) else { return vec![] };
    let mut groups = Vec::new();
    let mut group: Vec<usize> = Vec::new();
    for offset in 0..360 {
        let i = (start + offset) % 360;
        if !group.is_empty() && cut((i + 359) % 360, i) {
            groups.push(std::mem::take(&mut group));
        }
        if points[i].is_some() {
            group.push(i);
        }
    }
    if !group.is_empty() {
        groups.push(group);
    }
    groups
}

struct Edges {
    unknown: Vec<i64>,
    background: Vec<V>,
    gap_counts: Vec<i64>,
    tracking_only: bool,
}

/// _tolerant_edges(ranges, group, values, diameter, allow_boundary_gap, maintenance).
fn tolerant_edges(
    ranges: &V,
    floats: &[Option<f64>],
    group: &[usize],
    values: &[f64],
    diameter: f64,
    allow_boundary_gap: bool,
    maintenance: bool,
) -> Option<Edges> {
    let limit = if maintenance {
        TOLERANT_MAINTENANCE_EDGE_BINS
    } else if allow_boundary_gap {
        MAX_BOUNDARY_GAP_BINS
    } else {
        TOLERANT_EDGE_BINS
    };
    let far = values.iter().cloned().fold(f64::NEG_INFINITY, pmax) + MAX_POINT_GAP_M;
    let mut unknown = Vec::new();
    let mut background = Vec::new();
    let mut gap_counts = Vec::new();
    let list = ranges.l();
    for (edge, direction) in [(group[0] as i64, -1i64), (*group.last().unwrap() as i64, 1)] {
        let mut found = None;
        let mut count = 0;
        for step in 1..limit + 2 {
            let index = imod(edge + direction * step, 360);
            let value = floats[index as usize];
            if let Some(v) = value
                && v > far
            {
                found = Some(dict! {"index" => index, "range_m" => list[index as usize].clone()});
                break;
            }
            if step > limit {
                break;
            }
            unknown.push(index);
            count += 1;
        }
        let found = found?;
        background.push(found);
        gap_counts.push(count);
    }
    let vmax = values.iter().cloned().fold(f64::NEG_INFINITY, pmax);
    if gap_counts.iter().any(|&g| g != 0)
        && diameter + vmax * radians(gap_counts.iter().sum::<i64>() as f64) > MAX_DIAMETER_M
    {
        return None;
    }
    let tracking_only = !maintenance && *gap_counts.iter().max().unwrap() > TOLERANT_EDGE_BINS;
    Some(Edges { unknown, background, gap_counts, tracking_only })
}

thread_local! {
    static GROUP_CACHE: RefCell<Vec<(bool, Ranges, Rc<Points>, Rc<Vec<Vec<usize>>>)>> = const { RefCell::new(Vec::new()) };
}

/// _scan_groups(ranges, tolerant): points and components, once per content.
pub fn scan_groups(floats: &Ranges, tolerant: bool) -> (Rc<Points>, Rc<Vec<Vec<usize>>>) {
    let hit = GROUP_CACHE.with(|c| {
        c.borrow().iter().find_map(|(t, r, p, g)| {
            if *t == tolerant && (Rc::ptr_eq(r, floats) || **r == **floats) { Some((p.clone(), g.clone())) } else { None }
        })
    });
    if let Some(hit) = hit {
        return hit;
    }
    let points = scan_points(floats);
    let groups = if tolerant { clusters_bridged(&points, TOLERANT_INTERNAL_NULLS) } else { clusters(&points) };
    let out = (Rc::new(points), Rc::new(groups));
    GROUP_CACHE.with(|c| {
        let mut c = c.borrow_mut();
        c.retain(|(t, _, _, _)| *t != tolerant);
        c.push((tolerant, floats.clone(), out.0.clone(), out.1.clone()));
    });
    out
}

#[derive(Clone, Copy)]
pub struct CandidateArgs {
    pub allow_boundary_gap: bool,
    pub bearing_range_rad: (f64, f64),
    pub tolerant: bool,
    pub min_range_m: f64,
    pub maintenance: bool,
    pub min_points: Option<i64>,
    pub max_range_m: f64,
}

impl Default for CandidateArgs {
    fn default() -> Self {
        CandidateArgs {
            allow_boundary_gap: false,
            bearing_range_rad: (min_bearing_rad(), max_bearing_rad()),
            tolerant: false,
            min_range_m: MIN_RANGE_M,
            maintenance: false,
            min_points: None,
            max_range_m: MAX_RANGE_M,
        }
    }
}

fn mean_center(support: &[(f64, f64)]) -> (f64, f64) {
    let n = support.len() as f64;
    (fsum_py(support.iter().map(|p| p.0)) / n, fsum_py(support.iter().map(|p| p.1)) / n)
}

/// _candidates(ranges, ...).
pub fn candidates(ranges: &V, args: CandidateArgs) -> Vec<V> {
    let (bearing_min, bearing_max) = bearing_range(args.bearing_range_rad, "bearing_range_rad");
    if args.maintenance && !args.tolerant {
        value_error("maintenance edges are a tolerant-detection rule")
    }
    if !args.min_range_m.is_finite() || !(CLOSE_MAINTENANCE_MIN_RANGE_M <= args.min_range_m && args.min_range_m <= MIN_RANGE_M) {
        value_error("min_range_m must be within the valid lidar and acquisition bounds")
    }
    if !args.max_range_m.is_finite() || !(MAX_RANGE_M <= args.max_range_m && args.max_range_m <= EXTENDED_MAX_RANGE_M) {
        value_error("max_range_m must be within the acquisition bounds")
    }
    let floats = floats(ranges);
    let (points, groups) = scan_groups(&floats, args.tolerant);
    let list = ranges.l();
    let mut result = Vec::new();
    for group in groups.iter() {
        let minimum = match args.min_points {
            Some(m) => m,
            None if args.maintenance => TOLERANT_MAINTENANCE_MIN_POINTS,
            None if args.tolerant => TOLERANT_MIN_POINTS,
            None => MIN_POINTS,
        };
        let n = group.len() as i64;
        if !(minimum <= n && n <= 128) {
            continue;
        }
        let values: Vec<f64> = group.iter().map(|&i| floats[i].unwrap()).collect();
        let vmin = values.iter().cloned().fold(f64::INFINITY, pmin);
        let vmax = values.iter().cloned().fold(f64::NEG_INFINITY, pmax);
        if vmin < args.min_range_m || vmax > args.max_range_m {
            continue;
        }
        let support: Vec<(f64, f64)> = group.iter().map(|&i| points[i].unwrap()).collect();
        let span = |axis: usize| {
            let it = support.iter().map(|p| if axis == 0 { p.0 } else { p.1 });
            it.clone().fold(f64::NEG_INFINITY, pmax) - it.fold(f64::INFINITY, pmin)
        };
        if span(0) > MAX_DIAMETER_M || span(1) > MAX_DIAMETER_M {
            continue;
        }
        let mut diameter = f64::NEG_INFINITY;
        for (index, a) in support.iter().enumerate() {
            for b in &support[index + 1..] {
                diameter = pmax(diameter, dist(*a, *b));
            }
        }
        if support.len() < 2 {
            value_error("max() arg is an empty sequence")
        }
        if !(MIN_DIAMETER_M <= diameter && diameter <= MAX_DIAMETER_M) {
            continue;
        }
        let bins: Vec<V> = group.iter().map(|&i| V::Int(i as i64)).collect();
        if args.tolerant {
            let Some(edges) = tolerant_edges(ranges, &floats, group, &values, diameter, args.allow_boundary_gap, args.maintenance) else {
                continue;
            };
            let center = mean_center(&support);
            let (distance, bearing) = (hypot(center.0, center.1), center.1.atan2(center.0));
            if args.min_range_m <= distance && distance <= args.max_range_m && bearing_min <= bearing && bearing <= bearing_max {
                let first = group[0] as i64;
                let last = *group.last().unwrap() as i64;
                let mut internal = Vec::new();
                for i in first..first + imod(last - first, 360) + 1 {
                    if !group.contains(&(imod(i, 360) as usize)) {
                        internal.push(V::Int(imod(i, 360)));
                    }
                }
                let gap_sum: i64 = edges.gap_counts.iter().sum();
                let projected = if edges.gap_counts.iter().any(|&g| g != 0) { V::Float(vmax * radians(gap_sum as f64)) } else { V::None };
                result.push(dict! {
                    "point_left_m" => V::from(center), "range_m" => distance, "bearing_left_rad" => bearing,
                    "diameter_m" => diameter, "point_count" => group.len(), "support_bins" => V::list(bins),
                    "boundary_unknown_bins" => V::from(edges.unknown), "boundary_far_returns" => V::list(edges.background),
                    "internal_null_bins" => V::list(internal),
                    "boundary_gap_projected_span_m" => projected,
                    "tracking_only_boundary_gap" => edges.tracking_only, "tolerant_detection" => true,
                });
            }
            continue;
        }
        let mut unknown: Vec<i64> = Vec::new();
        let mut background: Vec<V> = Vec::new();
        let mut gap_counts: Vec<i64> = Vec::new();
        for (edge, direction) in [(group[0] as i64, -1i64), (*group.last().unwrap() as i64, 1)] {
            let mut index = imod(edge + direction, 360);
            let mut value = floats[index as usize];
            let mut edge_gaps = 0i64;
            while value.is_none() && args.allow_boundary_gap && edge_gaps < 360 {
                unknown.push(index);
                edge_gaps += 1;
                index = imod(index + direction, 360);
                value = floats[index as usize];
            }
            gap_counts.push(edge_gaps);
            match value {
                Some(v) if v > vmax + MAX_POINT_GAP_M => {}
                _ => break,
            }
            background.push(dict! {"index" => index, "range_m" => list[index as usize].clone()});
        }
        if background.len() != 2 || (gap_counts.iter().all(|&g| g != 0) && gap_counts != [1, 1]) {
            continue;
        }
        let mut projected_gap = V::None;
        let gmax = *gap_counts.iter().max().unwrap();
        if gmax > MAX_BOUNDARY_GAP_BINS {
            let projected = vmax * radians((gmax + 1) as f64);
            if projected > pmin(MAX_POINT_GAP_M, diameter) || diameter + projected > MAX_DIAMETER_M {
                continue;
            }
            projected_gap = V::Float(projected);
        }
        let center = mean_center(&support);
        let (distance, bearing) = (hypot(center.0, center.1), center.1.atan2(center.0));
        if args.min_range_m <= distance && distance <= args.max_range_m && bearing_min <= bearing && bearing <= bearing_max {
            let tracking_only = !unknown.is_empty();
            result.push(dict! {
                "point_left_m" => V::from(center), "range_m" => distance, "bearing_left_rad" => bearing,
                "diameter_m" => diameter, "point_count" => group.len(), "support_bins" => V::list(bins),
                "boundary_unknown_bins" => V::from(unknown), "boundary_far_returns" => V::list(background),
                "boundary_gap_projected_span_m" => projected_gap,
                "tracking_only_boundary_gap" => tracking_only,
            });
        }
    }
    result
}

/// _close_candidates(ranges, bearing_range_rad).
pub fn close_candidates(ranges: &V, bearing_range_rad: (f64, f64)) -> Vec<V> {
    let floats = floats(ranges);
    candidates(
        ranges,
        CandidateArgs {
            bearing_range_rad,
            tolerant: true,
            allow_boundary_gap: true,
            maintenance: true,
            min_range_m: CLOSE_MAINTENANCE_MIN_RANGE_M,
            ..CandidateArgs::default()
        },
    )
    .into_iter()
    .filter(|c| c.idx("support_bins").l().iter().any(|i| floats[i.i() as usize].unwrap() < MIN_RANGE_M))
    .collect()
}

/// faces_sensor(ranges, bins).
pub fn faces_sensor(floats: &[Option<f64>], bins: &[usize]) -> bool {
    let points: Vec<(f64, f64)> = TABLES.with(|t| {
        bins.iter()
            .map(|&i| {
                let r = floats[i].unwrap_or_else(|| type_error("unsupported operand type(s) for *: 'NoneType' and 'float'"));
                (r * t.directions[i].0, r * t.directions[i].1)
            })
            .collect()
    });
    if (points.len() as i64) < TOLERANT_MIN_POINTS {
        return false;
    }
    let n = points.len() as f64;
    let mx = fsum_py(points.iter().map(|p| p.0)) / n;
    let my = fsum_py(points.iter().map(|p| p.1)) / n;
    let sxx = fsum_py(points.iter().map(|p| sq(p.0 - mx)));
    let syy = fsum_py(points.iter().map(|p| sq(p.1 - my)));
    let sxy = fsum_py(points.iter().map(|p| (p.0 - mx) * (p.1 - my)));
    let axis = 0.5 * (2. * sxy).atan2(sxx - syy);
    let ray = my.atan2(mx);
    let angle = (fmod(axis - ray + PI / 2., PI) - PI / 2.).abs();
    angle >= contact_min_facing_rad()
}

/// contact_supports(ranges).
pub fn contact_supports(ranges: &V) -> Vec<V> {
    let floats = floats(ranges);
    candidates(
        ranges,
        CandidateArgs {
            bearing_range_rad: (-PI, PI),
            tolerant: true,
            allow_boundary_gap: true,
            min_range_m: CLOSE_MAINTENANCE_MIN_RANGE_M,
            ..CandidateArgs::default()
        },
    )
    .into_iter()
    .filter(|c| {
        let bins: Vec<usize> = c.idx("support_bins").l().iter().map(|i| i.i() as usize).collect();
        faces_sensor(&floats, &bins)
    })
    .collect()
}

#[derive(Clone, Debug)]
pub struct CompactTargetTracker {
    pub acquisition_bearing_rad: (f64, f64),
    pub maintenance_bearing_rad: Option<(f64, f64)>,
    pub tolerant: bool,
    pub acquisition_min_points: Option<i64>,
    pub acquisition_max_range_m: f64,
    pub held: V,
    pub select_associated: bool,
    pub unconfirmed_boundary_gap: bool,
    pub reason: String,
    pub last_now: Option<f64>,
    pub last_seq: Option<i64>,
    pub last_at: Option<i64>,
    pub last_received: V,
    pub signature: V,
    pub target: V,
    pub next_id: i64,
    pub first_at: Option<i64>,
    pub first_received: V,
    pub close_maintenance_track_id: Option<i64>,
}

#[derive(Clone, Copy)]
pub struct TrackerArgs {
    pub acquisition_bearing_rad: (f64, f64),
    pub maintenance_bearing_rad: Option<(f64, f64)>,
    pub select_associated: bool,
    pub unconfirmed_boundary_gap: bool,
    pub tolerant: bool,
    pub acquisition_min_points: Option<i64>,
}

impl Default for TrackerArgs {
    fn default() -> Self {
        TrackerArgs {
            acquisition_bearing_rad: (min_bearing_rad(), max_bearing_rad()),
            maintenance_bearing_rad: None,
            select_associated: false,
            unconfirmed_boundary_gap: false,
            tolerant: false,
            acquisition_min_points: None,
        }
    }
}

fn opt_i(v: Option<i64>) -> V {
    v.map(V::Int).unwrap_or(V::None)
}

fn opt_f(v: Option<f64>) -> V {
    v.map(V::Float).unwrap_or(V::None)
}

fn pair(v: (f64, f64)) -> V {
    V::from(v)
}

impl CompactTargetTracker {
    pub fn new(args: TrackerArgs) -> Self {
        let acquisition = bearing_range(args.acquisition_bearing_rad, "acquisition_bearing_rad");
        let maintenance = args.maintenance_bearing_rad.map(|b| bearing_range(b, "maintenance_bearing_rad"));
        if let Some(points) = args.acquisition_min_points
            && (!(TOLERANT_MAINTENANCE_MIN_POINTS..=MIN_POINTS).contains(&points) || !args.tolerant)
        {
            value_error("acquisition_min_points must be a tolerant 3..5 return count")
        }
        CompactTargetTracker {
            acquisition_bearing_rad: acquisition,
            maintenance_bearing_rad: maintenance,
            tolerant: args.tolerant,
            acquisition_min_points: args.acquisition_min_points,
            acquisition_max_range_m: MAX_RANGE_M,
            held: V::None,
            select_associated: args.select_associated,
            unconfirmed_boundary_gap: args.unconfirmed_boundary_gap,
            reason: "compact_target_not_observed".into(),
            last_now: None,
            last_seq: None,
            last_at: None,
            last_received: V::None,
            signature: V::None,
            target: V::None,
            next_id: 1,
            first_at: None,
            first_received: V::None,
            close_maintenance_track_id: None,
        }
    }

    /// The Python object's attributes, for state comparison.
    pub fn dump(&self) -> V {
        dict! {
            "_acquisition_bearing_rad" => pair(self.acquisition_bearing_rad),
            "_maintenance_bearing_rad" => self.maintenance_bearing_rad.map(pair),
            "_tolerant" => self.tolerant,
            "_acquisition_min_points" => opt_i(self.acquisition_min_points),
            "_acquisition_max_range_m" => self.acquisition_max_range_m,
            "held" => self.held.clone(),
            "_select_associated" => self.select_associated,
            "_unconfirmed_boundary_gap" => self.unconfirmed_boundary_gap,
            "reason" => self.reason.as_str(),
            "_last_now" => opt_f(self.last_now),
            "_last_seq" => opt_i(self.last_seq),
            "_last_at" => opt_i(self.last_at),
            "_last_received" => self.last_received.clone(),
            "_signature" => self.signature.clone(),
            "_target" => self.target.clone(),
            "_next_id" => self.next_id,
            "_first_at" => opt_i(self.first_at),
            "_first_received" => self.first_received.clone(),
            "_close_maintenance_track_id" => opt_i(self.close_maintenance_track_id),
        }
    }

    pub fn extend_acquisition_range(&mut self, max_range_m: f64) {
        if !max_range_m.is_finite() || !(MAX_RANGE_M..=EXTENDED_MAX_RANGE_M).contains(&max_range_m) {
            value_error("max_range_m must be within the acquisition bounds")
        }
        self.acquisition_max_range_m = max_range_m;
    }

    pub fn widen_acquisition(&mut self, bearing_range_rad: (f64, f64)) {
        self.acquisition_bearing_rad = bearing_range(bearing_range_rad, "acquisition_bearing_rad");
    }

    pub fn enable_close_maintenance(&mut self, track_id: &V) -> bool {
        let target = &self.target;
        if !track_id.is_int()
            || !target.is_dict()
            || !py_eq(target.idx("track_id"), track_id)
            || !target.idx("confirmed").is_true()
            || !target.idx("tracking_only_boundary_gap").is_false()
        {
            return false;
        }
        self.close_maintenance_track_id = Some(track_id.i());
        true
    }

    pub fn forget_pub(&mut self, reason: &str) -> V {
        self.forget(reason)
    }

    fn forget(&mut self, reason: &str) -> V {
        self.reason = reason.into();
        self.target = V::None;
        self.first_at = None;
        self.first_received = V::None;
        self.close_maintenance_track_id = None;
        V::None
    }

    /// update(scan, now, acquire=..., heading_rad=..., only_support_bins=...).
    pub fn update(&mut self, scan: &V, now: f64, acquire: bool, heading_rad: Option<f64>, only_support_bins: Option<&[i64]>) -> V {
        if let Some(bins) = only_support_bins
            && (bins.is_empty() || bins.iter().any(|&i| !(0..360).contains(&i)))
        {
            value_error("only_support_bins must be a non-empty set of bin indices")
        }
        if let Some(h) = heading_rad
            && !h.is_finite()
        {
            value_error("heading_rad must be a number")
        }
        self.held = V::None;
        if !now.is_finite() || now < 0. || self.last_now.is_some_and(|last| now < last) {
            return self.forget("compact_target_invalid_receive_clock");
        }
        self.last_now = Some(now);
        let received_v = scan.get("received_at");
        if !scan.is_dict()
            || scan.get("frame_id").os() != Some(FRAME)
            || !scan.get("seq").is_int()
            || scan.get("seq").i() < 0
            || !scan.get("at_ms").is_int()
            || scan.get("at_ms").i() < 0
            || !received_v.is_num()
            || !(0. <= received_v.f() && received_v.f() <= now)
        {
            return self.forget("compact_target_invalid_scan");
        }
        let (seq, published, received) = (scan.idx("seq").i(), scan.idx("at_ms").i(), received_v.f());
        let ranges = scan.get("ranges");
        let valid = valid_ranges(ranges);
        let signature = if valid { ranges.clone() } else { V::None };
        if let Some(last_seq) = self.last_seq
            && seq <= last_seq
        {
            if seq < last_seq {
                return self.forget("compact_target_reordered_scan");
            }
            if Some(published) != self.last_at
                || !py_eq(received_v, &self.last_received)
                || !py_eq(&signature, &self.signature)
            {
                return self.forget("compact_target_duplicate_changed");
            }
            if now - received >= MAX_AGE_S {
                return self.forget("compact_target_stale");
            }
            if self.target.is_some()
                && self.target.idx("tracking_only_boundary_gap").truthy()
                && now - self.target.idx("last_full_isolation_received_at").f() >= MAX_AGE_S
            {
                return self.forget("compact_target_boundary_gap_expired");
            }
            return self.target.clone();
        }
        if let Some(last_at) = self.last_at
            && (published <= last_at || received <= self.last_received.f())
        {
            return self.forget("compact_target_reordered_clock");
        }
        let publication_dt = self.last_at.map(|last| (published - last) as f64 / 1000.);
        let receive_dt = if self.last_received.is_none() { None } else { Some(received - self.last_received.f()) };
        self.last_seq = Some(seq);
        self.last_at = Some(published);
        self.last_received = received_v.clone();
        self.signature = signature;
        if !valid {
            return self.forget("compact_target_invalid_ranges");
        }
        if now - received >= MAX_AGE_S {
            return self.forget("compact_target_stale");
        }
        if let Some(publication_dt) = publication_dt {
            let receive_dt = receive_dt.unwrap();
            if publication_dt >= MAX_AGE_S || receive_dt >= MAX_AGE_S {
                self.forget("compact_target_observation_gap");
            } else if (publication_dt - receive_dt).abs() > 0.15 {
                return self.forget("compact_target_clock_gap");
            }
        }
        let old = self.target.clone();
        let confirmed_old = old.is_some() && old.idx("confirmed").truthy();
        let allow_boundary_gap = confirmed_old || (old.is_some() && self.unconfirmed_boundary_gap);
        let bearing = if confirmed_old && self.maintenance_bearing_rad.is_some() {
            self.maintenance_bearing_rad.unwrap()
        } else {
            self.acquisition_bearing_rad
        };
        let close_bound = confirmed_old && py_eq(old.idx("track_id"), &opt_i(self.close_maintenance_track_id));
        let maintenance = self.tolerant && confirmed_old;
        let floats = floats(ranges);
        let mut found = candidates(
            ranges,
            CandidateArgs {
                allow_boundary_gap,
                bearing_range_rad: bearing,
                tolerant: self.tolerant,
                maintenance,
                min_range_m: if close_bound { CLOSE_MAINTENANCE_MIN_RANGE_M } else { MIN_RANGE_M },
                min_points: if maintenance { None } else { self.acquisition_min_points },
                max_range_m: self.acquisition_max_range_m,
            },
        );
        for candidate in found.iter_mut() {
            let close = candidate.idx("support_bins").l().iter().any(|i| floats[i.i() as usize].unwrap() < MIN_RANGE_M);
            candidate.set("close_maintenance_observation", close);
        }
        if let Some(only) = only_support_bins {
            let mut want: Vec<i64> = only.to_vec();
            want.sort();
            want.dedup();
            found.retain(|c| {
                let mut have: Vec<i64> = c.idx("support_bins").l().iter().map(|i| i.i()).collect();
                have.sort();
                have.dedup();
                have == want
            });
        }
        let slack = if self.tolerant && old.is_some() {
            pmax(0., (published - old.idx("source_at_ms").i()) as f64 / 1000. - 0.1)
        } else {
            0.
        };
        let mut reference = old.clone();
        if let Some(heading) = heading_rad
            && old.is_some()
            && old.get("compass_heading_rad").is_num()
        {
            reference = rotated_observation(&old, Some(heading - old.get("compass_heading_rad").f()));
        }
        if self.select_associated && allow_boundary_gap {
            found.retain(|c| associated(c, &reference, slack));
        }
        self.held = V::None;
        if found.len() != 1 {
            let reason = if found.is_empty() { "compact_target_missing" } else { "compact_target_ambiguous" };
            if self.tolerant
                && old.is_some()
                && ((published - old.idx("source_at_ms").i()) as f64 / 1000.) < MAX_AGE_S
                && received - old.idx("source_received_at").f() < MAX_AGE_S
            {
                self.reason = format!("{reason}_held");
                self.held = old.clone();
                return V::None;
            }
            return self.forget(reason);
        }
        let current = found.pop().unwrap();
        let close_observation = current.get("close_maintenance_observation").is_true();
        let is_associated = associated(&current, &reference, slack);
        let boundary_gap = current.idx("tracking_only_boundary_gap").truthy();
        if boundary_gap && !is_associated {
            return self.forget("compact_target_boundary_gap_unassociated");
        }
        if boundary_gap
            && ((published - old.idx("last_full_isolation_at_ms").i()) as f64 / 1000. >= MAX_AGE_S
                || now - old.idx("last_full_isolation_received_at").f() >= MAX_AGE_S)
        {
            return self.forget("compact_target_boundary_gap_expired");
        }
        if !is_associated && !acquire {
            return self.forget("compact_target_acquisition_disabled");
        }
        let b = current.idx("bearing_left_rad").f();
        if !is_associated && !(self.acquisition_bearing_rad.0 <= b && b <= self.acquisition_bearing_rad.1) {
            return self.forget("compact_target_outside_acquisition_sector");
        }
        let (track_id, count) = if is_associated {
            (old.idx("track_id").clone(), old.idx("confirmation_count").i() + if boundary_gap { 0 } else { 1 })
        } else {
            let id = self.next_id;
            self.next_id += 1;
            self.first_at = Some(published);
            self.first_received = received_v.clone();
            self.close_maintenance_track_id = None;
            (V::Int(id), 1)
        };
        let confirmed = count >= 3
            && published - self.first_at.unwrap() >= round(MIN_CONFIRM_S * 1000.)
            && received - self.first_received.f() + 1e-9 >= MIN_CONFIRM_S;
        self.reason = if boundary_gap {
            "compact_target_tracked_with_boundary_gap"
        } else if confirmed {
            "compact_target_confirmed"
        } else {
            "compact_target_unconfirmed"
        }
        .into();
        let mut target = current;
        target.update(&dict! {
            "source_seq" => seq, "source_at_ms" => published, "source_received_at" => received_v.clone(),
            "confirmed" => confirmed, "close_maintenance_observation" => close_observation,
            "confirmation_count" => count, "semantic_class" => "unknown", "kind" => "lidar_compact_object",
            "track_id" => track_id,
            "last_full_isolation_seq" => if boundary_gap { old.idx("last_full_isolation_seq").clone() } else { V::Int(seq) },
            "last_full_isolation_at_ms" => if boundary_gap { old.idx("last_full_isolation_at_ms").clone() } else { V::Int(published) },
            "last_full_isolation_received_at" => if boundary_gap { old.idx("last_full_isolation_received_at").clone() } else { received_v.clone() },
            "candidate_only" => true, "physical_identity_verified" => false,
        });
        if let Some(heading) = heading_rad {
            target.set("compass_heading_rad", heading);
        }
        self.target = target;
        self.target.clone()
    }
}
