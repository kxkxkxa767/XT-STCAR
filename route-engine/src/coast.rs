//! coast_motion.py (round 97, part B): scan-to-scan low-motion evidence for
//! the coast standstill check (CoastMotion, _planes, _register, _fit) and the
//! worker's queue semantics (CoastMotionWorker: one latest pending scan, a
//! result usable only for the exact current scan). Line for line, same
//! results, key order and types as Python. The worker computes on
//! `run_pending` (the deterministic offline mode); the standalone program
//! runs it on its own thread.

use std::collections::HashSet;
use std::rc::Rc;

use crate::dict;
use crate::helpers::{dcos, dsin, FRAME};
use crate::py::*;

pub const COAST_MIN_VALID_POINTS: usize = 324;
pub const PLANE_IDENTITY_REQUIRED_SPAN_M: f64 = 1.5;
const MIN_PUBLICATION_STABLE_S: f64 = 0.5;
const MIN_RECEIVE_STABLE_S: f64 = 0.5;
const MAX_CLOCK_PROGRESS_DIFFERENCE_S: f64 = 0.15;
const MIN_LOW_MOTION_PAIRS: i64 = 5;
const MAX_LOW_SPEED_MPS: f64 = 0.05;
const MAX_LOW_YAW_RATE_RPS: f64 = 0.03;
const MAX_LOW_PAIR_TRANSLATION_M: f64 = 0.008;
const MAX_STABLE_TOTAL_TRANSLATION_M: f64 = 0.02;
const MAX_STABLE_TOTAL_ROTATION_RAD: f64 = 0.02;

/// SOFTWARE_THRESHOLDS, in its key order.
pub fn software_thresholds() -> V {
    dict! {
        "min_publication_stable_s" => MIN_PUBLICATION_STABLE_S,
        "min_receive_stable_s" => MIN_RECEIVE_STABLE_S,
        "max_clock_progress_difference_s" => MAX_CLOCK_PROGRESS_DIFFERENCE_S,
        "min_low_motion_pairs" => MIN_LOW_MOTION_PAIRS,
        "max_low_speed_mps" => MAX_LOW_SPEED_MPS,
        "max_low_yaw_rate_rps" => MAX_LOW_YAW_RATE_RPS,
        "max_low_pair_translation_m" => MAX_LOW_PAIR_TRANSLATION_M,
        "max_stable_total_translation_m" => MAX_STABLE_TOTAL_TRANSLATION_M,
        "max_stable_total_rotation_rad" => MAX_STABLE_TOTAL_ROTATION_RAD,
    }
}

type Point = (f64, f64);

/// A point as a set/dict key: Python tuples of floats compare (and hash) by
/// value, so 0.0 and -0.0 are the same key.
fn key(p: Point) -> (u64, u64) {
    ((p.0 + 0.0).to_bits(), (p.1 + 0.0).to_bits())
}

fn wrap_angle(angle: f64) -> f64 {
    fmod(angle + std::f64::consts::PI, 2. * std::f64::consts::PI) - std::f64::consts::PI
}

fn max_of(values: &[f64]) -> f64 {
    values[1..].iter().fold(values[0], |a, &b| pmax(a, b))
}

fn min_of(values: &[f64]) -> f64 {
    values[1..].iter().fold(values[0], |a, &b| pmin(a, b))
}

#[derive(Clone, Debug)]
struct Line {
    nx: f64,
    ny: f64,
    rho: f64,
    angle: f64,
    points: Rc<Vec<Point>>,
    span: f64,
    rmse: f64,
    max_residual: f64,
}

/// _fit(points).
fn fit(points: Vec<Point>) -> Line {
    let xs: Vec<f64> = points.iter().map(|p| p.0).collect();
    let ys: Vec<f64> = points.iter().map(|p| p.1).collect();
    let cx = fmean(&xs);
    let cy = fmean(&ys);
    let xx = fsum_py(points.iter().map(|&(x, _)| sq(x - cx)));
    let xy = fsum_py(points.iter().map(|&(x, y)| (x - cx) * (y - cy)));
    let yy = fsum_py(points.iter().map(|&(_, y)| sq(y - cy)));
    let angle = 0.5 * (2. * xy).atan2(xx - yy);
    let (tx, ty) = (angle.pcos(), angle.psin());
    let (mut nx, mut ny) = (-ty, tx);
    let mut rho = nx * cx + ny * cy;
    if rho < 0. {
        (nx, ny, rho) = (-nx, -ny, -rho);
    }
    let residuals: Vec<f64> = points.iter().map(|&(x, y)| nx * x + ny * y - rho).collect();
    let projections: Vec<f64> = points.iter().map(|&(x, y)| tx * x + ty * y).collect();
    let rmse = sqrt(fsum_py(residuals.iter().map(|r| r * r)) / points.len() as f64);
    let max_residual = residuals.iter().map(|r| r.abs()).fold(f64::NEG_INFINITY, pmax);
    Line {
        nx,
        ny,
        rho,
        angle: ny.atan2(nx),
        span: max_of(&projections) - min_of(&projections),
        rmse,
        max_residual,
        points: Rc::new(points),
    }
}

/// _planes(ranges).
fn planes(ranges: &[Option<f64>]) -> Vec<Line> {
    let mut groups: Vec<Vec<Point>> = Vec::new();
    let mut group: Vec<Point> = Vec::new();
    let mut previous_angle = 0usize;
    for (angle, r) in ranges.iter().enumerate() {
        let Some(r) = *r else { continue };
        let p = (r * dcos(angle), r * dsin(angle));
        if !group.is_empty() && (angle - previous_angle > 3 || dist(*group.last().unwrap(), p) > 0.30) {
            groups.push(std::mem::take(&mut group));
        }
        group.push(p);
        previous_angle = angle;
    }
    if !group.is_empty() {
        groups.push(group);
    }
    if groups.len() > 1 && dist(*groups.last().unwrap().last().unwrap(), groups[0][0]) <= 0.30 {
        let mut last = groups.pop().unwrap();
        last.extend(groups[0].iter().copied());
        groups[0] = last;
    }
    let mut lines: Vec<Line> = Vec::new();
    for group in &groups {
        let circular = dist(group[0], *group.last().unwrap()) <= 0.30 && group.len() > 96;
        let mut extended = group.clone();
        if circular {
            extended.extend(group[..95.min(group.len())].iter().copied());
        }
        for start in (0..group.len()).step_by(8) {
            for size in [12usize, 24, 48, 96] {
                if start + size > extended.len() {
                    continue;
                }
                let line = fit(extended[start..start + size].to_vec());
                if line.span >= 0.35 && line.rmse <= 0.015 && line.max_residual <= 0.035 {
                    lines.push(line);
                }
            }
        }
    }
    // sorted(..., key=len(points), reverse=True): stable, ties keep their order.
    lines.sort_by(|a, b| b.points.len().cmp(&a.points.len()));
    let mut merged: Vec<Line> = Vec::new();
    let mut owned: Vec<HashSet<(u64, u64)>> = Vec::new();
    for line in lines {
        let mut joined_into = false;
        for index in 0..merged.len() {
            let old = &merged[index];
            if wrap_angle(line.angle - old.angle).abs() < radians(2.) && (line.rho - old.rho).abs() < 0.035 {
                // list(dict.fromkeys(old + line)): first occurrences, in order.
                let mut seen = HashSet::new();
                let points: Vec<Point> =
                    old.points.iter().chain(line.points.iter()).copied().filter(|&p| seen.insert(key(p))).collect();
                let joined = fit(points);
                if joined.rmse <= 0.015 && joined.max_residual <= 0.035 {
                    owned[index] = joined.points.iter().map(|&p| key(p)).collect();
                    merged[index] = joined;
                    joined_into = true;
                    break;
                }
            }
        }
        if !joined_into {
            // A window made mostly of points another plane already holds is a
            // sub-window of that plane (coast_motion._planes), not a second plane.
            let n = line.points.len();
            if !owned.iter().any(|points| 2 * line.points.iter().filter(|&&p| points.contains(&key(p))).count() > n) {
                owned.push(line.points.iter().map(|&p| key(p)).collect());
                merged.push(line);
            }
        }
    }
    if merged.len() <= 12 { merged } else { Vec::new() }
}

fn nonparallel(lines: &[&Line]) -> bool {
    lines.iter().any(|a| lines.iter().any(|b| (a.nx * b.ny - a.ny * b.nx).abs() >= 0.5))
}

/// _register(previous, current, dt): (motion, error).
fn register(previous: &[Line], current: &[Line], dt: f64) -> Result<V, &'static str> {
    let near = |old: &Line, line: &Line| {
        wrap_angle(old.angle - line.angle).abs() <= 0.15 && (old.rho - line.rho).abs() <= pmin(0.80, 5. * dt + 0.035)
    };
    let mut matches: Vec<(&Line, &Line)> = Vec::new();
    for line in current {
        let candidates: Vec<&Line> = previous.iter().filter(|old| near(old, line)).collect();
        if candidates.len() == 1 && current.iter().filter(|other| near(candidates[0], other)).count() == 1 {
            matches.push((candidates[0], line));
        }
    }
    if current.iter().any(|line| line.span >= PLANE_IDENTITY_REQUIRED_SPAN_M && !previous.iter().any(|old| near(old, line)))
        || previous.iter().any(|old| old.span >= PLANE_IDENTITY_REQUIRED_SPAN_M && !current.iter().any(|line| near(old, line)))
    {
        return Err("plane_identity_unmatched");
    }
    if matches.len() < 2 {
        return Err("insufficient_associated_planes");
    }
    if !nonparallel(&matches.iter().map(|m| m.0).collect::<Vec<_>>()) {
        return Err("parallel_planes_unobservable");
    }
    let rotations: Vec<f64> = matches.iter().map(|(old, line)| wrap_angle(old.angle - line.angle)).collect();
    let rotation = median(rotations.clone());
    if rotations.iter().map(|r| wrap_angle(r - rotation).abs()).fold(f64::NEG_INFINITY, pmax) > radians(1.5) {
        return Err("inconsistent_plane_rotation");
    }
    let aa = fsum_py(matches.iter().map(|(old, _)| sq(old.nx)));
    let ab = fsum_py(matches.iter().map(|(old, _)| old.nx * old.ny));
    let bb = fsum_py(matches.iter().map(|(old, _)| sq(old.ny)));
    let ax = fsum_py(matches.iter().map(|(old, line)| old.nx * (old.rho - line.rho)));
    let bx = fsum_py(matches.iter().map(|(old, line)| old.ny * (old.rho - line.rho)));
    let determinant = aa * bb - ab * ab;
    if determinant / sq(aa + bb) < 0.03 {
        return Err("ill_conditioned_plane_geometry");
    }
    let (tx, ty) = ((ax * bb - bx * ab) / determinant, (bx * aa - ax * ab) / determinant);
    let (c, s) = (rotation.pcos(), rotation.psin());
    let mut errors: Vec<f64> = Vec::new();
    let (mut inliers, mut count) = (0usize, 0usize);
    for (old, line) in &matches {
        let transformed: Vec<Point> = line.points.iter().map(|&(x, y)| (c * x - s * y + tx, s * x + c * y + ty)).collect();
        let residuals: Vec<f64> = transformed.iter().map(|&(x, y)| old.nx * x + old.ny * y - old.rho).collect();
        errors.push(fsum_py(residuals.iter().map(|r| r * r)) / residuals.len() as f64);
        inliers += residuals.iter().filter(|r| r.abs() <= 0.035).count();
        count += residuals.len();
        let tangent = (-old.ny, old.nx);
        let old_along: Vec<f64> = old.points.iter().map(|&(x, y)| tangent.0 * x + tangent.1 * y).collect();
        let new_along: Vec<f64> = transformed.iter().map(|&(x, y)| tangent.0 * x + tangent.1 * y).collect();
        let overlap = pmin(max_of(&old_along), max_of(&new_along)) - pmax(min_of(&old_along), min_of(&new_along));
        if overlap < 0.20 || overlap < 0.4 * pmin(old.span, line.span) {
            return Err("plane_support_changed");
        }
    }
    let rmse = sqrt(mean(&errors));
    if rmse > 0.020 || (inliers as f64) < 0.90 * count as f64 {
        return Err("rigid_residual_too_large");
    }
    let translation = hypot(tx, ty);
    if translation > 0.80 || translation / dt > 5. || rotation.abs() / dt > 1.5 {
        return Err("motion_jump_exceeds_limits");
    }
    Ok(dict! {
        "translation_m" => translation, "rotation_rad" => rotation,
        "speed_mps" => translation / dt, "yaw_rate_rps" => rotation / dt,
        "matched_planes" => matches.len(), "rmse_m" => rmse,
        "inlier_fraction" => inliers as f64 / count as f64,
    })
}

fn finite(value: &V) -> bool {
    value.is_num()
}

/// CoastMotion: both clocks need >= .5 s of low-motion evidence.
#[derive(Clone, Debug, Default)]
pub struct CoastMotion {
    previous: Option<(f64, Vec<Line>)>,
    last_seq: Option<i64>,
    last_publication: Option<i64>,
    last_now: Option<f64>,
    last_receive: Option<f64>,
    last_ranges: Option<V>,
    cached: Option<V>,
    stable_since: Option<f64>,
    stable_receive_since: Option<f64>,
    stable_pairs: i64,
    stable_translation: f64,
    stable_rotation: f64,
}

impl CoastMotion {
    pub fn new() -> CoastMotion {
        CoastMotion::default()
    }

    fn clear_stable(&mut self) {
        self.stable_since = None;
        self.stable_receive_since = None;
        self.stable_pairs = 0;
        self.stable_translation = 0.;
        self.stable_rotation = 0.;
    }

    fn diagnostics(&self, publication_elapsed: f64, receive_elapsed: f64) -> V {
        dict! {
            "stable_ms" => round(pmin(publication_elapsed, receive_elapsed) * 1000.),
            "stable_publication_ms" => round(publication_elapsed * 1000.),
            "stable_receive_ms" => round(receive_elapsed * 1000.),
            "stable_pairs" => self.stable_pairs,
            "stable_translation_m" => self.stable_translation,
            "stable_rotation_rad" => self.stable_rotation,
            "evidence_type" => "uncalibrated_lidar_low_motion",
            "software_thresholds" => software_thresholds(),
        }
    }

    fn unknown_result(&mut self, reason: &str, reset_reference: bool) -> V {
        self.clear_stable();
        if reset_reference {
            self.previous = None;
        }
        let mut cached = dict! {
            "observable" => false, "stationary" => false, "low_motion" => false, "translation_m" => V::None,
            "rotation_rad" => V::None, "speed_mps" => V::None, "yaw_rate_rps" => V::None, "reason" => reason,
        };
        cached.update(&self.diagnostics(0., 0.));
        self.cached = Some(cached.clone());
        cached
    }

    /// CoastMotion()._unknown(reason): a fresh estimator's unknown evidence.
    pub fn unknown(reason: &str) -> V {
        CoastMotion::new().unknown_result(reason, true)
    }

    /// update(scan, lidar_age_s, now).
    pub fn update(&mut self, scan: &V, lidar_age_s: &V, now: &V) -> V {
        if !finite(now) || now.f() < 0. || self.last_now.is_some_and(|last| now.f() < last) {
            return self.unknown_result("invalid_receive_clock", true);
        }
        let now = now.f();
        self.last_now = Some(now);
        if !finite(lidar_age_s) || !(0. ..0.30).contains(&lidar_age_s.f()) {
            return self.unknown_result("lidar_stale_or_invalid_age", true);
        }
        if !scan.is_dict() || scan.get("frame_id").os() != Some(FRAME) {
            return self.unknown_result("unknown_lidar_frame", true);
        }
        let (seq, at) = (scan.get("seq"), scan.get("at_ms"));
        if !seq.is_int() || seq.i() < 0 || !at.is_int() || at.i() < 0 {
            return self.unknown_result("invalid_scan_clock", true);
        }
        let (seq, at) = (seq.i(), at.i());
        if self.last_seq == Some(seq) {
            if Some(at) != self.last_publication {
                return self.unknown_result("duplicate_sequence_changed_clock", true);
            }
            let same_ranges = scan.get("ranges").is_list()
                && self.last_ranges.as_ref().is_some_and(|last| py_eq(scan.get("ranges"), last));
            if !same_ranges {
                return self.unknown_result("duplicate_sequence_changed_ranges", true);
            }
            // Cached evidence cannot mature because server wall time passed.
            let mut result = match &self.cached {
                Some(cached) if cached.truthy() => cached.clone(),
                _ => self.unknown_result("repeated_scan", true),
            };
            result.set("reason", "repeated_scan");
            return result;
        }
        if let Some(last_seq) = self.last_seq
            && (seq < last_seq || self.last_publication.is_some_and(|last| at <= last))
        {
            return self.unknown_result("reordered_scan_or_publication_clock", true);
        }
        let publication_s = at as f64 / 1000.;
        let old_publication = self.last_publication;
        let old_receive = self.last_receive;
        self.last_seq = Some(seq);
        self.last_publication = Some(at);
        self.last_receive = Some(now);
        let ranges = scan.get("ranges");
        let valid = match ranges {
            V::List(list) => {
                list.len() == 360 && list.iter().all(|r| r.is_none() || (r.is_num() && (0.02..=12.).contains(&r.f())))
            }
            _ => false,
        };
        if !valid {
            return self.unknown_result("invalid_scan_ranges", true);
        }
        self.last_ranges = Some(ranges.clone());
        let floats: Vec<Option<f64>> = ranges.l().iter().map(|r| r.of()).collect();
        if floats.iter().filter(|r| r.is_some()).count() < COAST_MIN_VALID_POINTS {
            return self.unknown_result("scan_coverage_insufficient", true);
        }
        let current = planes(&floats);
        if current.len() < 2 {
            return self.unknown_result("insufficient_extended_planes", true);
        }
        if !nonparallel(&current.iter().collect::<Vec<_>>()) {
            return self.unknown_result("parallel_planes_unobservable", true);
        }
        let Some((previous_publication_s, previous)) = self.previous.take() else {
            self.previous = Some((publication_s, current));
            return self.unknown_result("reference_initialized", false);
        };
        let dt = publication_s - previous_publication_s;
        if !(0.05..0.30).contains(&dt) {
            self.previous = Some((publication_s, current));
            return self.unknown_result("scan_gap_or_rate_invalid", false);
        }
        if let (Some(old_publication), Some(old_receive)) = (old_publication, old_receive)
            && ((at - old_publication) as f64 / 1000. - (now - old_receive)).abs() > 0.15
        {
            self.previous = Some((publication_s, current));
            return self.unknown_result("publication_receive_clock_disagreement", false);
        }
        let registered = register(&previous, &current, dt);
        self.previous = Some((publication_s, current));
        let motion = match registered {
            Ok(motion) => motion,
            Err(error) => return self.unknown_result(error, false),
        };
        let (translation, rotation) = (motion.idx("translation_m").f(), motion.idx("rotation_rad").f());
        let low = motion.idx("speed_mps").f() <= MAX_LOW_SPEED_MPS
            && motion.idx("yaw_rate_rps").f().abs() <= MAX_LOW_YAW_RATE_RPS
            && translation <= MAX_LOW_PAIR_TRANSLATION_M;
        let mut reset_reason: Option<&str> = None;
        if !low {
            self.clear_stable();
        } else {
            if self.stable_since.is_none() {
                self.stable_since = Some(previous_publication_s);
                self.stable_receive_since = old_receive;
            }
            self.stable_pairs += 1;
            self.stable_translation += translation;
            self.stable_rotation += rotation.abs();
            if self.stable_translation > MAX_STABLE_TOTAL_TRANSLATION_M || self.stable_rotation > MAX_STABLE_TOTAL_ROTATION_RAD {
                self.clear_stable();
                reset_reason = Some("stable_total_motion_exceeds_limits");
            }
        }
        let mut elapsed = self.stable_since.map(|since| publication_s - since).unwrap_or(0.);
        let mut received = self.stable_receive_since.map(|since| now - since).unwrap_or(0.);
        if (elapsed - received).abs() > MAX_CLOCK_PROGRESS_DIFFERENCE_S {
            self.clear_stable();
            (elapsed, received) = (0., 0.);
            reset_reason = Some("stable_clock_progress_disagreement");
        }
        let stationary = self.stable_pairs >= MIN_LOW_MOTION_PAIRS
            && elapsed >= MIN_PUBLICATION_STABLE_S
            && received >= MIN_RECEIVE_STABLE_S;
        let reason = reset_reason.unwrap_or(if stationary {
            "stable_observed_low_motion"
        } else if low {
            "collecting_low_motion_evidence"
        } else {
            "observed_motion"
        });
        let mut cached = dict! {"observable" => true, "stationary" => stationary, "low_motion" => low};
        cached.update(&motion);
        cached.set("reason", reason);
        cached.update(&self.diagnostics(elapsed, received));
        self.cached = Some(cached.clone());
        cached
    }
}

/// CoastMotionWorker's queue: one latest pending scan, results only for the
/// exact current scan and generation.
#[derive(Default)]
pub struct CoastWorker {
    generation: i64,
    pending: Option<(i64, V, V, V, V, f64)>,
    result: Option<(i64, V, f64, V)>,
    last_input: Option<V>,
    motion: Option<CoastMotion>,
    motion_generation: Option<i64>,
    /// processing_ms in results (wall clock in Python; fixed for offline runs).
    pub processing_ms: Option<f64>,
}

fn identity(scan: &V, received: &V) -> V {
    let ranges = if scan.has("ranges") { scan.get("ranges").clone() } else { V::list(Vec::new()) };
    V::list(vec![scan.get("seq").clone(), scan.get("at_ms").clone(), scan.get("frame_id").clone(), received.clone(), ranges])
}

fn input_error(scan: &V, age: &V, received: &V, now: &V) -> Option<&'static str> {
    if !scan.is_dict() || !scan.get("ranges").is_list() {
        return Some("invalid_scan_ranges");
    }
    if !finite(received)
        || received.f() < 0.
        || !finite(now)
        || !finite(age)
        || !(0. ..0.30).contains(&age.f())
        || !(0. ..0.30).contains(&(now.f() - received.f()))
    {
        return Some("motion_result_stale_receive");
    }
    if ((now.f() - received.f()) - age.f()).abs() > 0.15 {
        return Some("motion_result_receive_age_disagreement");
    }
    let (seq, at) = (scan.get("seq"), scan.get("at_ms"));
    if !seq.is_int() || seq.i() < 0 || !at.is_int() || at.i() < 0 {
        return Some("invalid_scan_clock");
    }
    None
}

impl CoastWorker {
    pub fn new() -> CoastWorker {
        CoastWorker::default()
    }

    pub fn unknown(reason: &str) -> V {
        CoastMotion::unknown(reason)
    }

    pub fn reset(&mut self) -> i64 {
        self.generation += 1;
        self.pending = None;
        self.result = None;
        self.last_input = None;
        self.generation
    }

    /// submit(generation, scan, age, received, now): an error reason or None.
    pub fn submit(&mut self, generation: i64, scan: &V, age: &V, received: &V, now: &V) -> Option<&'static str> {
        if let Some(error) = input_error(scan, age, received, now) {
            return Some(error);
        }
        let id = identity(scan, received);
        if generation != self.generation {
            return Some("motion_worker_generation_changed");
        }
        if !self.last_input.as_ref().is_some_and(|last| py_eq(&id, last)) {
            let mut snapshot = dict! {
                "seq" => scan.get("seq").clone(), "at_ms" => scan.get("at_ms").clone(),
                "frame_id" => scan.get("frame_id").clone(),
            };
            snapshot.set("ranges", V::list(scan.idx("ranges").l().to_vec()));
            self.pending = Some((generation, id.clone(), snapshot, age.clone(), received.clone(), now.f()));
            self.last_input = Some(id);
        }
        None
    }

    /// result(generation, scan, age, received, now).
    pub fn result(&self, generation: i64, scan: &V, age: &V, received: &V, now: &V) -> V {
        if let Some(error) = input_error(scan, age, received, now) {
            return CoastWorker::unknown(error);
        }
        let id = identity(scan, received);
        if generation != self.generation {
            return CoastWorker::unknown("motion_worker_generation_changed");
        }
        let Some((result_generation, result_id, submitted, motion)) = &self.result else {
            return CoastWorker::unknown("motion_estimator_pending");
        };
        if *result_generation != generation || !py_eq(result_id, &id) {
            return CoastWorker::unknown("motion_result_superseded");
        }
        if !(0. ..0.30).contains(&(now.f() - submitted)) {
            return CoastWorker::unknown("motion_result_stale_submission");
        }
        motion.clone()
    }

    /// run_pending(): compute the pending scan now (offline/deterministic).
    pub fn run_pending(&mut self) {
        let Some((generation, id, scan, age, received, submitted)) = self.pending.take() else { return };
        let started = std::time::Instant::now();
        if self.motion_generation != Some(generation) {
            self.motion = Some(CoastMotion::new());
            self.motion_generation = Some(generation);
        }
        let estimator = self.motion.as_mut().unwrap();
        let mut motion = match py_try(|| estimator.update(&scan, &age, &received)) {
            Ok(motion) => motion,
            Err(_) => {
                self.motion = Some(CoastMotion::new());
                CoastWorker::unknown("motion_estimator_failed")
            }
        };
        let processing_ms = self.processing_ms.unwrap_or_else(|| started.elapsed().as_secs_f64() * 1000.);
        motion.update(&dict! {
            "source_scan_seq" => scan.idx("seq").clone(), "source_publication_at_ms" => scan.idx("at_ms").clone(),
            "source_receive_monotonic_s" => received.clone(), "processing_ms" => processing_ms,
        });
        if generation == self.generation {
            self.result = Some((generation, id, submitted, motion));
        }
    }
}
