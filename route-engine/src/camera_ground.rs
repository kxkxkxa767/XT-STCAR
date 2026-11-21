//! Camera observations of the two cones and the lower edge of the surveyed boards.
//! Coordinates stay in the camera frame until paired with a timestamped lidar scan.
use crate::camera_projection::{CAM_AHEAD_LIDAR, CAM_H, ground_point, undistort_pt};

pub(crate) const PITCH: f64 = 8.6_f64.to_radians();
pub(crate) type XY = (f64, f64);

#[derive(Clone, Copy, Debug)]
pub(crate) struct Foot {
    pub pixel: (u16, u16),
    pub normal: XY,
}
impl Foot {
    fn new(u: usize, v: usize) -> Option<Self> {
        Some(Self {
            pixel: (u as u16, v as u16),
            normal: undistort_pt(u as f64, v as f64)?,
        })
    }
    pub fn point(self, pitch: f64) -> Option<XY> {
        let p = ground_point(self.normal, pitch)?;
        ((0.25..=4.8).contains(&p.0) && p.1.abs() <= 4.0).then_some(p)
    }
}

#[derive(Clone, Debug, Default)]
pub(crate) struct GroundFrame {
    pub frame: i64,
    pub captured: f64,
    pub boards: Vec<Foot>,
    /// The first cone is red/orange, the second blue.
    pub cones: [Vec<Foot>; 2],
}

fn pixel(bgr: &[u8], x: usize, y: usize) -> [i32; 3] {
    let i = 3 * (640 * y + x);
    [bgr[i] as i32, bgr[i + 1] as i32, bgr[i + 2] as i32]
}
fn board([b, g, r]: [i32; 3]) -> bool {
    b - r > 30 && (b - g) * 9 > b && g > r + 5 && b > 50
}
fn rail([b, g, r]: [i32; 3]) -> bool {
    let hi = r.max(g).max(b);
    let lo = r.min(g).min(b);
    lo >= 90 && hi - lo <= (hi / 7).max(17)
}

fn panel_foot(bgr: &[u8], u: usize) -> Option<usize> {
    // The board is taller than the camera. A blue patch starting on the
    // ground or a painted basketball line cannot span the optical horizon.
    let mut begin = 0;
    let mut last = 0;
    for y in (80..475).step_by(2) {
        if board(pixel(bgr, u, y)) {
            if last == 0 || y - last > 6 {
                begin = y;
            }
            last = y;
        } else if last > 0 && y >= last + 6 {
            if begin < 282 && last > 295 && last - begin >= 26 {
                // Follow the silver lower trim to its floor-facing edge.
                // The upper trim can be outside the image on a near board.
                let top_trim = (begin.saturating_sub(10)..begin).any(|v| rail(pixel(bgr, u, v)));
                let mut end = last + 1;
                let mut seen = 0;
                for v in last + 1..(last + 36).min(476) {
                    if rail(pixel(bgr, u, v)) {
                        end = v;
                        seen += 1;
                    } else if (seen >= 3 && v >= end + 2) || (seen < 3 && v > last + 6) {
                        break;
                    }
                }
                if seen >= 3 || (top_trim && last - begin >= 40) {
                    return Some(if seen >= 3 { end + 1 } else { last + 1 });
                }
            }
            last = 0;
        }
    }
    None
}

#[derive(Clone, Copy)]
struct Span {
    lo: usize,
    hi: usize,
}
impl Span {
    fn width(self) -> usize {
        self.hi - self.lo + 1
    }
    fn mid(self) -> f64 {
        (self.hi + self.lo) as f64 * 0.5
    }
    fn near(self, other: Self, gap: usize) -> bool {
        self.lo <= other.hi + gap && other.lo <= self.hi + gap
    }
}
fn cone_color([b, g, r]: [i32; 3]) -> Option<usize> {
    if r > 60 && 100 * (r - g) > 15 * r && 100 * (r - b) > 30 * r && g + 14 >= b {
        Some(0)
    } else if b > 55 && b - r > 38 && 10 * b > 14 * g && 10 * (b - r) > 4 * b {
        Some(1)
    } else {
        None
    }
}

fn cone_feet(rows: &[Vec<Span>], bgr: &[u8], output: &mut Vec<Foot>) {
    const STEP: usize = 3;
    const Y0: usize = 56;
    for (top, spans) in rows.iter().enumerate().skip(1) {
        if Y0 + top * STEP > 295 {
            break;
        }
        for &tip in spans {
            if tip.width() > 9 || (top % 3 != 0 && rows[top - 1].iter().any(|s| s.near(tip, 1))) {
                continue;
            }
            let mut tail = tip;
            let mut bottom = top;
            let mut samples = 0;
            for (y, line) in rows
                .iter()
                .enumerate()
                .take((top + 105).min(rows.len()))
                .skip(top + 1)
            {
                if y - bottom > 5 {
                    break;
                }
                let Some(next) = line.iter().filter(|s| s.near(tail, 2)).min_by(|a, b| {
                    (a.mid() - tip.mid())
                        .abs()
                        .total_cmp(&(b.mid() - tip.mid()).abs())
                }) else {
                    continue;
                };
                let width = next.width();
                if width > (y - top) * 3 / 4 + 6
                    || width > 2 * tail.width() + 3
                    || (next.mid() - tip.mid()).abs() > 3. + width as f64 * 0.50
                {
                    break;
                }
                tail = *next;
                bottom = y;
                samples += 1;
            }
            let height = bottom - top;
            if height < 5
                || samples * 4 < height * 3
                || tail.width() < 3
                || tail.width() < tip.width() + 2
                || tail.lo > tip.lo
                || tail.hi < tip.hi
                || height as f64 / (tail.width() as f64) < 0.95
                || height as f64 / (tail.width() as f64) > 5.5
                || tail.lo == 0
                || tail.hi >= 212
                || Y0 + bottom * STEP >= 467
            {
                continue;
            }
            let widest_mid = rows[top + height / 2]
                .iter()
                .filter(|s| s.near(tail, 0))
                .map(|s| s.width())
                .min();
            if widest_mid.is_none_or(|w| w > tail.width()) {
                continue;
            }
            let u = (tail.mid() * STEP as f64).round() as usize;
            let shoulder = Y0 + (bottom + 1) * STEP;
            let width = tail.width() * STEP;
            // The rectangular foot often touches an orange court of the
            // same hue. Locate its bottom edge across the central columns.
            let end = (shoulder + (width / 2).clamp(3, 24)).min(475);
            let edge = |v: usize| {
                [-2i32, -1, 0, 1, 2]
                    .iter()
                    .map(|i| {
                        let x = (u as i32 + i * width as i32 / 8).clamp(0, 639) as usize;
                        let a = pixel(bgr, x, v - 1);
                        let b = pixel(bgr, x, v + 1);
                        (a[0] - b[0]).abs() + (a[1] - b[1]).abs() + (a[2] - b[2]).abs()
                    })
                    .sum::<i32>()
            };
            let strongest = (shoulder..=end)
                .max_by_key(|&v| edge(v))
                .unwrap_or(shoulder);
            let v = if edge(strongest) > 190 {
                strongest
            } else {
                (shoulder + width / 6).min(end)
            };
            let Some(foot) = Foot::new(u, v) else {
                continue;
            };
            let Some((x, _)) = foot.point(PITCH) else {
                continue;
            };
            let depth = x - CAM_AHEAD_LIDAR;
            let width_m = tail.width() as f64 * STEP as f64 * depth / 426.55;
            let Some((_, tip_y)) = undistort_pt(u as f64, (Y0 + top * STEP) as f64) else {
                continue;
            };
            let height_m = CAM_H + depth * (PITCH - tip_y.atan()).tan();
            if !(0.06..=0.44).contains(&width_m) || !(0.23..=0.95).contains(&height_m) {
                continue;
            }
            if !output.iter().any(|p| {
                p.pixel.0.abs_diff(foot.pixel.0) < 15 && p.pixel.1.abs_diff(foot.pixel.1) < 15
            }) {
                output.push(foot);
            }
        }
    }
}

/// When a blue cone stands in front of a blue board the colour regions merge.
/// The cone interrupts both board contours: its tip is above the top rail and
/// its base projects below the bottom rail at the same column.
fn blue_in_front_of_board(bgr: &[u8], found: &mut Vec<Foot>) {
    let profile: Vec<_> = (0..640)
        .step_by(4)
        .map(|u| {
            let mut best = None;
            let (mut first, mut last) = (0usize, 0usize);
            for y in (34..478).step_by(2) {
                if board(pixel(bgr, u, y)) {
                    if last == 0 || y - last > 6 {
                        first = y;
                    }
                    last = y;
                } else if last != 0 && y >= last + 6 {
                    if first < 287
                        && last >= 265
                        && last - first >= 18
                        && best.is_none_or(|(t, b)| last - first > b - t)
                    {
                        best = Some((first, last));
                    }
                    last = 0;
                }
            }
            best
        })
        .collect();
    let mut proposals = Vec::new();
    for (i, c) in profile.iter().enumerate() {
        let Some((top, bottom)) = *c else { continue };
        for radius in [4usize, 6, 8, 11, 15, 20] {
            if i < radius || i + radius >= profile.len() {
                continue;
            }
            let flank = |at: usize| {
                if at == 0 || at + 1 >= profile.len() {
                    return None;
                }
                let samples = [profile[at - 1]?, profile[at]?, profile[at + 1]?];
                let t = samples[1].0;
                let b = samples[1].1;
                samples
                    .iter()
                    .all(|&(y, v)| y.abs_diff(t) <= 7 && v.abs_diff(b) <= 7)
                    .then_some((t, b))
            };
            let (Some((lt, lb)), Some((rt, rb))) = (flank(i - radius), flank(i + radius)) else {
                continue;
            };
            if lt.abs_diff(rt) > 8 + radius * 2 || lb.abs_diff(rb) > 6 + radius {
                continue;
            }
            let t0 = (lt + rt) / 2;
            let b0 = (lb + rb) / 2;
            if lt.min(rt) < top + 14 || bottom + 3 < b0 || bottom - top < 32 {
                continue;
            }
            let rise = t0 - top;
            let height = bottom - top;
            let check_y = top + rise / 2;
            let mid = pixel(bgr, i * 4, check_y);
            let background = [
                pixel(bgr, (i - radius) * 4, check_y),
                pixel(bgr, (i + radius) * 4, check_y),
            ];
            let pale_blue = |[b, g, r]: [i32; 3]| b - r > 18 && b - g > 5;
            if background.iter().all(|&p| pale_blue(p))
                || background
                    .iter()
                    .any(|p| (0..3).map(|j| (p[j] - mid[j]).abs()).sum::<i32>() < 55)
            {
                continue;
            }
            let baseline = |k: usize, l: usize, r: usize| {
                l as f64 + (r as f64 - l as f64) * (k + radius - i) as f64 / (2 * radius) as f64
            };
            let mut shaft = Vec::new();
            let mut tip = 0;
            let mut base = Vec::new();
            for (k, &sample) in profile
                .iter()
                .enumerate()
                .take(i + radius)
                .skip(i - radius + 1)
            {
                if let Some((t, b)) = sample {
                    let rise_k = baseline(k, lt, rt) - t as f64;
                    if rise_k > 5. {
                        shaft.push(k);
                    }
                    if rise_k > rise as f64 * 0.74 {
                        tip += 1;
                    }
                    if b as f64 - baseline(k, lb, rb) > 3.0 {
                        base.push(k);
                    }
                }
            }
            if shaft.len() < 3
                || tip == 0
                || tip >= shaft.len()
                || shaft[shaft.len() - 1] - shaft[0] + 1 > shaft.len() + 2
                || 4 * shaft.len() > rise * 2 + 8
            {
                continue;
            }
            let center = (shaft[0] + shaft[shaft.len() - 1]) * 2;
            if center.abs_diff(i * 4) > shaft.len() * 2 {
                continue;
            }
            let rail_edge = |x: usize, low: usize, high: usize| {
                (low..=high).rev().find(|&row| {
                    rail(pixel(bgr, x, row))
                        && rail(pixel(bgr, x, row - 2))
                        && pale_blue(pixel(bgr, x, row + 6))
                })
            };
            if let Some(edge) = rail_edge(center, top.saturating_sub(72).max(4), top - 3) {
                let margin = 5 + radius;
                let l = rail_edge(
                    (i - radius) * 4,
                    edge.saturating_sub(margin).max(4),
                    edge + margin,
                );
                let r = rail_edge(
                    (i + radius) * 4,
                    edge.saturating_sub(margin).max(4),
                    edge + margin,
                );
                if let (Some(l), Some(r)) = (l, r)
                    && (l + r).abs_diff(2 * edge) <= 8
                {
                    continue;
                }
            }
            let distinct_base = base.len() >= 3
                && base[base.len() - 1] - base[0] <= base.len() + 1
                && (base[0] + base[base.len() - 1]).abs_diff(2 * i) <= base.len();
            let (width, foot_y) = if distinct_base {
                (
                    4 * (base[base.len() - 1] - base[0] + 1),
                    base.iter()
                        .filter_map(|&j| profile[j].map(|x| x.1))
                        .max()
                        .unwrap()
                        + 1,
                )
            } else {
                if rise < 21 {
                    continue;
                }
                let above = pixel(bgr, center, bottom - 3);
                let below = pixel(bgr, center, (bottom + 5).min(479));
                if board(below) || (0..3).map(|j| (above[j] - below[j]).abs()).sum::<i32>() < 65 {
                    continue;
                }
                (4 * shaft.len() * height / rise, bottom + 1)
            };
            if height as f64 / (width as f64) < 1.1 || height as f64 / (width as f64) > 5.8 {
                continue;
            }
            let Some(foot) = Foot::new(center, foot_y) else {
                continue;
            };
            let Some((x, _)) = foot.point(PITCH) else {
                continue;
            };
            let d = x - CAM_AHEAD_LIDAR;
            let (Some((_, yn)), width_m) = (
                undistort_pt((i * 4) as f64, top as f64),
                width as f64 * d / 426.55,
            ) else {
                continue;
            };
            let height_m = CAM_H + d * (PITCH - yn.atan()).tan();
            if !(0.075..=0.48).contains(&width_m) || !(0.24..=0.95).contains(&height_m) {
                continue;
            }
            proposals.push((rise, foot));
        }
    }
    proposals.sort_unstable_by_key(|q| std::cmp::Reverse(q.0));
    for (_, p) in proposals {
        if !found
            .iter()
            .any(|x| x.pixel.0.abs_diff(p.pixel.0) < 22 && x.pixel.1.abs_diff(p.pixel.1) < 18)
        {
            found.push(p);
        }
    }
}

pub(crate) fn detect(bgr: &[u8], w: usize, h: usize, frame: i64, captured: f64) -> GroundFrame {
    let mut result = GroundFrame {
        frame,
        captured,
        ..Default::default()
    };
    if (w, h) != (640, 480) || bgr.len() != 640 * 480 * 3 {
        return result;
    }
    let columns: Vec<_> = (8..633)
        .step_by(8)
        .map(|u| (u, panel_foot(bgr, u)))
        .collect();
    for three in columns.windows(3) {
        let [(.., Some(l)), (u, Some(v)), (.., Some(r))] = three else {
            continue;
        };
        if l.abs_diff(*r) <= 24
            && (2 * (*v) as i32 - (*l + *r) as i32).abs() <= 6
            && let Some(foot) = Foot::new(*u, *v)
            && foot.point(PITCH).is_some()
        {
            result.boards.push(foot);
        }
    }
    let mut rows: [Vec<Vec<Span>>; 2] = Default::default();
    for y in (56..476).step_by(3) {
        let mut lines: [Vec<Span>; 2] = Default::default();
        let mut old = None;
        let mut start = 0;
        for col in 0..=214 {
            let color = if col * 3 < 640 {
                cone_color(pixel(bgr, col * 3, y))
            } else {
                None
            };
            if color != old {
                if let Some(which) = old {
                    let line: &mut Vec<Span> = &mut lines[which];
                    line.push(Span {
                        lo: start,
                        hi: col - 1,
                    });
                }
                old = color;
                start = col;
            }
        }
        let [red, blue] = lines;
        let mut joined: Vec<Span> = Vec::new();
        for span in red {
            if let Some(prev) = joined.last_mut()
                && span.lo <= prev.hi + 3
            {
                prev.hi = span.hi;
            } else {
                joined.push(span);
            }
        }
        rows[0].push(joined);
        rows[1].push(blue);
    }
    for (spans, feet) in rows.iter().zip(result.cones.iter_mut()) {
        cone_feet(spans, bgr, feet);
    }
    result.cones[1].retain(|foot| {
        // A colour variation on a blue panel ends above the board's metal
        // foot. A nearer cone with an observable base occludes that trim.
        let (x, y) = (foot.pixel.0 as usize, foot.pixel.1 as usize);
        let on_panel = [18, 30, 45]
            .into_iter()
            .filter(|&dy| {
                board(pixel(bgr, x.saturating_sub(18), y - dy))
                    && board(pixel(bgr, (x + 18).min(639), y - dy))
            })
            .count()
            >= 2;
        !on_panel
            && !result.boards.iter().any(|wall| {
                foot.pixel.0.abs_diff(wall.pixel.0) < 28
                    && foot.pixel.1 < wall.pixel.1.saturating_add(8)
            })
    });
    blue_in_front_of_board(bgr, &mut result.cones[1]);
    result
}

/// Move a stationary ground point from camera exposure time to the scan time.
pub(crate) fn to_scan(p: XY, elapsed: f64, speed: f64, yaw_rate: f64) -> XY {
    let yaw = elapsed * yaw_rate;
    let (sin, cos) = yaw.sin_cos();
    let (sm, cm) = (0.5 * yaw).sin_cos();
    let x = p.0 - speed * elapsed * cm;
    let y = p.1 - speed * elapsed * sm;
    (cos * x + sin * y, -sin * x + cos * y)
}
pub(crate) fn bearing_bin(p: XY) -> usize {
    ((-p.1.atan2(p.0).to_degrees()).round() as i32).rem_euclid(360) as usize
}
impl GroundFrame {
    /// Ranges corroborating the metal foot refine pitch. Missing rays may
    /// subsequently be filled, but can never participate in this calibration.
    pub fn refine_pitch(
        &self,
        ranges: &[Option<f64>],
        old: f64,
        age: f64,
        speed: f64,
        rate: f64,
    ) -> f64 {
        let mut fits = Vec::new();
        for foot in &self.boards {
            let Some(p) = foot.point(old) else { continue };
            let p = to_scan(p, age, speed, rate);
            let i = bearing_bin(p);
            let Some(r) = ranges[i] else { continue };
            let supported = [(i + 359) % 360, (i + 1) % 360]
                .into_iter()
                .all(|k| ranges[k].is_some_and(|v| (v - r).abs() < 0.12_f64.max(r * 0.08)));
            if !supported || (p.0.hypot(p.1) - r).abs() > 0.30_f64.max(r * 0.17) {
                continue;
            }
            let a = -(i as f64).to_radians();
            let q = to_scan((r * a.cos(), r * a.sin()), -age, speed, rate);
            let x = q.0 - CAM_AHEAD_LIDAR;
            if x < 0.45 {
                continue;
            }
            let fit = foot.normal.1.atan() - (CAM_H / x).atan();
            if (6.5_f64.to_radians()..=10_f64.to_radians()).contains(&fit) {
                fits.push(fit);
            }
        }
        if fits.len() < 5 {
            return old;
        }
        fits.sort_unstable_by(f64::total_cmp);
        if fits[3 * fits.len() / 4] - fits[fits.len() / 4] > 1.0_f64.to_radians() {
            return old;
        }
        old + 0.3 * (fits[fits.len() / 2] - old).clamp(-0.8_f64.to_radians(), 0.8_f64.to_radians())
    }
}

/// A visible continuous panel ties nearby real laser returns to its missing
/// directions. Fit a line only to real returns; the image limits its extent.
#[derive(Clone, Copy, Debug)]
pub(crate) struct PanelLine {
    normal: XY,
    offset: f64,
    along_lo: f64,
    along_hi: f64,
}
impl PanelLine {
    pub fn fit(points: &[XY]) -> Option<Self> {
        if points.len() < 4 {
            return None;
        }
        let mut best = None;
        let mut count = 0;
        let step = points.len().div_ceil(12);
        for i in (0..points.len()).step_by(step) {
            for j in ((i + 1)..points.len()).step_by(step) {
                let (dx, dy) = (points[j].0 - points[i].0, points[j].1 - points[i].1);
                let span = dx.hypot(dy);
                if span < 0.16 {
                    continue;
                }
                let (nx, ny) = (-dy / span, dx / span);
                let d = nx * points[i].0 + ny * points[i].1;
                let n = points
                    .iter()
                    .filter(|&&(x, y)| (nx * x + ny * y - d).abs() < 0.045)
                    .count();
                if n > count {
                    count = n;
                    best = Some((nx, ny, d));
                }
            }
        }
        if count < 4 || count * 3 < points.len() * 2 {
            return None;
        }
        let (nx, ny, d) = best?;
        let best: Vec<_> = points
            .iter()
            .copied()
            .filter(|(x, y)| (nx * x + ny * y - d).abs() < 0.045)
            .collect();
        let n = best.len() as f64;
        let (x, y) = (
            best.iter().map(|p| p.0).sum::<f64>() / n,
            best.iter().map(|p| p.1).sum::<f64>() / n,
        );
        let (xx, yy, xy) = best.iter().fold((0., 0., 0.), |(xx, yy, xy), p| {
            let (dx, dy) = (p.0 - x, p.1 - y);
            (xx + dx * dx, yy + dy * dy, xy + dx * dy)
        });
        let h = 0.5 * (2. * xy).atan2(xx - yy);
        let (sy, cx) = h.sin_cos();
        let (lo, hi) = best
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| {
                let a = cx * p.0 + sy * p.1;
                (lo.min(a), hi.max(a))
            });
        if hi - lo < 0.18 {
            return None;
        }
        Some(Self {
            normal: (-sy, cx),
            offset: -sy * x + cx * y,
            along_lo: lo,
            along_hi: hi,
        })
    }
    pub fn intersect(self, origin: XY, approx: XY, tangent: XY) -> Option<XY> {
        let (nx, ny) = self.normal;
        let (x, y) = (approx.0 - origin.0, approx.1 - origin.1);
        let incidence = nx * x + ny * y;
        if incidence.abs() < 0.18 * x.hypot(y)
            || (nx * tangent.0 + ny * tangent.1).abs() > 0.42 * tangent.0.hypot(tangent.1)
        {
            return None;
        }
        let s = (self.offset - nx * origin.0 - ny * origin.1) / incidence;
        if !(0.52..=1.80).contains(&s) {
            return None;
        }
        let q = (origin.0 + s * x, origin.1 + s * y);
        let along = ny * q.0 - nx * q.1;
        let distance = (q.0 - origin.0).hypot(q.1 - origin.1);
        if along < self.along_lo - 0.9
            || along > self.along_hi + 0.9
            || !(0.09..=5.5).contains(&distance)
        {
            return None;
        }
        Some(q)
    }
}

#[cfg(test)]
mod projection_tests {
    use super::PanelLine;
    #[test]
    fn board_intersection_is_independent_of_the_survey_origin() {
        for x0 in [0., 5.2] {
            let points = [
                (x0 + 2.8, -0.5),
                (x0 + 2.8, -0.2),
                (x0 + 2.8, 0.1),
                (x0 + 2.8, 0.4),
            ];
            let line = PanelLine::fit(&points).unwrap();
            let (x, y) = line
                .intersect((x0 + 0.159, 0.), (x0 + 3.0, 0.), (0., 0.16))
                .unwrap();
            assert!((x - x0 - 2.8).abs() < 1e-8 && y.abs() < 1e-8);
        }
    }
}

impl GroundFrame {
    pub fn projected_boards(
        &self,
        ranges: &[Option<f64>],
        pitch: f64,
        age: f64,
        speed: f64,
        rate: f64,
    ) -> Vec<(XY, bool)> {
        let mut result = Vec::with_capacity(self.boards.len());
        let origin = to_scan((CAM_AHEAD_LIDAR, 0.), age, speed, rate);
        let mut start = 0;
        while start < self.boards.len() {
            let mut end = start + 1;
            while end < self.boards.len()
                && self.boards[end].pixel.0 - self.boards[end - 1].pixel.0 <= 16
                && self.boards[end]
                    .pixel
                    .1
                    .abs_diff(self.boards[end - 1].pixel.1)
                    <= 18
            {
                end += 1;
            }
            let feet: Vec<_> = self.boards[start..end]
                .iter()
                .filter_map(|f| f.point(pitch))
                .map(|p| to_scan(p, age, speed, rate))
                .collect();
            let mut anchors = Vec::new();
            let mut last_bin = None;
            for &q in &feet {
                let i = bearing_bin(q);
                if last_bin == Some(i) {
                    continue;
                }
                last_bin = Some(i);
                if let Some(r) = ranges[i]
                    && (q.0.hypot(q.1) - r).abs() < 0.42_f64.max(0.47 * r)
                    && [(i + 359) % 360, (i + 1) % 360].into_iter().all(|j| {
                        ranges[j].is_some_and(|val| (val - r).abs() < 0.09_f64.max(0.06 * r))
                    })
                {
                    let ang = -(i as f64).to_radians();
                    anchors.push((r * ang.cos(), r * ang.sin()));
                }
            }
            let line = PanelLine::fit(&anchors);
            for (i, &q) in feet.iter().enumerate() {
                let l = feet[i.saturating_sub(1)];
                let r = feet[(i + 1).min(feet.len() - 1)];
                let hit = line.and_then(|l0| l0.intersect(origin, q, (r.0 - l.0, r.1 - l.1)));
                result.push((hit.unwrap_or(q), hit.is_some()));
            }
            start = end;
        }
        result
    }
}
