//! Resolve LED arrays on a sunlit traffic signal. Compare bright islands with the
//! glass of the same lens. The adjacent amber lens shares the signal's backplate.
use super::{Frame, Lights};

const RED: u8 = 1;
const GREEN: u8 = 2;
const AMBER: u8 = 4;

#[derive(Clone, Copy)]
struct Lens {
    kind: u8,
    x: usize,
    y: usize,
    w: usize,
    h: usize,
}

impl Lens {
    fn centre(self) -> (f64, f64) {
        (
            self.x as f64 + (self.w - 1) as f64 * 0.5,
            self.y as f64 + (self.h - 1) as f64 * 0.5,
        )
    }
}

#[derive(Default)]
pub(super) struct Beads {
    coarse: Vec<u8>,
    queue: Vec<usize>,
    lenses: Vec<Lens>,
    islands: Vec<u8>,
    spans: Vec<(usize, usize, usize)>,
}

fn lens_colours((b, g, r): (u8, u8, u8)) -> u8 {
    let (b, g, r) = (i32::from(b), i32::from(g), i32::from(r));
    let red = r >= 72 && 5 * r >= 8 * g && r - b >= 28 && 2 * (g - b) <= r - b + 14;
    let green = g >= 45 && g + 4 >= r && g - b >= 10 && 2 * g - r - b >= 18;
    let amber = r >= 120
        && r - b >= 88
        && g - b >= 23
        && 100 * (g - b) >= 28 * (r - b)
        && 100 * (g - b) <= 77 * (r - b);
    u8::from(red) * RED | u8::from(green) * GREEN | u8::from(amber) * AMBER
}

fn channels((b, g, r): (u8, u8, u8), kind: u8) -> (usize, usize) {
    let (b, g, r) = (usize::from(b), usize::from(g), usize::from(r));
    (
        (b + 2 * g + r) / 4,
        if kind == RED {
            (b + g) / 2
        } else {
            (b + r) / 2
        },
    )
}

fn median(hist: &[usize; 256], len: usize) -> usize {
    let mut sum = 0;
    hist.iter()
        .position(|&n| {
            sum += n;
            sum > len / 2
        })
        .unwrap_or(255)
}

#[derive(Default)]
struct Component {
    n: usize,
    min_x: usize,
    min_y: usize,
    max_x: usize,
    max_y: usize,
    sum_x: usize,
    sum_y: usize,
}

// Both masks have a one-pixel zero border. Multiple lens colours can share a cell.
fn flood(
    mask: &mut [u8],
    stride: usize,
    bit: u8,
    start: usize,
    queue: &mut Vec<usize>,
) -> Component {
    let mut c = Component {
        min_x: usize::MAX,
        min_y: usize::MAX,
        ..Component::default()
    };
    queue.push(start);
    mask[start] &= !bit;
    while let Some(i) = queue.pop() {
        let (x, y) = (i % stride - 1, i / stride - 1);
        c.n += 1;
        c.sum_x += x;
        c.sum_y += y;
        c.min_x = c.min_x.min(x);
        c.min_y = c.min_y.min(y);
        c.max_x = c.max_x.max(x);
        c.max_y = c.max_y.max(y);
        for n in [
            i - stride - 1,
            i - stride,
            i - stride + 1,
            i - 1,
            i + 1,
            i + stride - 1,
            i + stride,
            i + stride + 1,
        ] {
            if mask[n] & bit != 0 {
                mask[n] &= !bit;
                queue.push(n);
            }
        }
    }
    c
}

fn opposite_lens(frame: &Frame<'_>, lens: Lens, ax: f64, ay: f64, span: f64) -> bool {
    // A very small or elongated red patch can also be skin or clothing.
    if lens.kind == RED && lens.w >= 18 && 4 * lens.w >= 3 * lens.h && 4 * lens.h >= 3 * lens.w {
        return true;
    }
    let (cx, cy) = lens.centre();
    // The other lenses lie on the same line as the lit green one.
    for spacing in [0.83, 1., 1.17] {
        let rx = ax + (ax - cx) * spacing;
        let ry = ay + (ay - cy) * spacing;
        let mut hits = 0;
        for (dx, dy) in [(0., 0.), (-0.19, 0.), (0.19, 0.), (0., 0.19), (0., -0.19)] {
            let x = (rx + dx * span).round() as isize;
            let y = (ry + dy * span).round() as isize;
            if x < 0 || y < 0 || x as usize >= frame.width || y as usize >= frame.height {
                continue;
            }
            let (b, g, r) = frame.pixel(x as usize, y as usize);
            let (b, g, r) = (i32::from(b), i32::from(g), i32::from(r));
            let opposite = if lens.kind == GREEN {
                r >= 65 && r - b >= 30 && 100 * (g - b) < 30 * (r - b) && 2 * g < r + b
            } else {
                // Daylight makes an unlit green cover olive, with a modest green excess.
                g >= 26 && g * 100 >= r * 87 && g - b >= 7 && 2 * g - r - b >= 10
            };
            if opposite {
                hits += 1;
            }
        }
        if hits >= 4 {
            return true;
        }
    }
    false
}
fn common_housing(frame: &Frame<'_>, lens: Lens, amber: Lens, mid: usize) -> bool {
    let (cx, cy) = lens.centre();
    let (ax, ay) = amber.centre();
    let separation = if lens.kind == RED { ax - cx } else { cx - ax };
    let diameter = (lens.w + amber.w) as f64 * 0.5;
    if separation < 1.12 * diameter
        || separation > 3.25 * diameter
        || amber.w * 100 < lens.w * 52
        || amber.w * 100 > lens.w * 192
        || amber.h * 100 < lens.h * 48
        || amber.h * 100 > lens.h * 210
        || (ay - cy).abs() > 0.65 * amber.h.max(lens.h) as f64
    {
        return false;
    }
    let x = ((ax + cx) * 0.5).round() as usize;
    let y = (ay + cy) * 0.5;
    let mut dark = 0;
    for offset in [-0.18, 0., 0.18] {
        let sy = (y + offset * lens.h as f64)
            .round()
            .clamp(0., (frame.height - 1) as f64) as usize;
        dark +=
            usize::from(channels(frame.pixel(x, sy), lens.kind).0 < 82.max(mid.saturating_sub(13)));
    }
    dark >= 2 && opposite_lens(frame, lens, ax, ay, amber.w as f64)
}

fn rim_visible(frame: &Frame<'_>, lens: Lens, mid: usize) -> bool {
    const RAYS: [(f64, f64); 12] = [
        (1., 0.),
        (0.866, 0.5),
        (0.5, 0.866),
        (0., 1.),
        (-0.5, 0.866),
        (-0.866, 0.5),
        (-1., 0.),
        (-0.866, -0.5),
        (-0.5, -0.866),
        (0., -1.),
        (0.5, -0.866),
        (0.866, -0.5),
    ];
    let (cx, cy) = lens.centre();
    let (mut visible, mut dark) = (0, 0);
    for (dx, dy) in RAYS {
        let mut seen = false;
        for radius in [1.15, 1.32, 1.52] {
            let x = (cx + dx * radius * lens.w as f64 * 0.5).round() as isize;
            let y = (cy + dy * radius * lens.h as f64 * 0.5).round() as isize;
            if x < 0 || y < 0 || x as usize >= frame.width || y as usize >= frame.height {
                continue;
            }
            seen = true;
            if channels(frame.pixel(x as usize, y as usize), lens.kind).0 + 15 < mid {
                dark += 1;
                break;
            }
        }
        visible += usize::from(seen);
    }
    dark >= 5 && dark * 100 >= 65 * visible
}

fn amber_pixel(pixel: (u8, u8, u8)) -> bool {
    let (b, g, r) = (i32::from(pixel.0), i32::from(pixel.1), i32::from(pixel.2));
    r >= 85
        && r - b >= 48
        && g - b >= 13
        && 100 * (g - b) >= 25 * (r - b)
        && 100 * (g - b) <= 78 * (r - b)
}

// When direct sunshine joins the housing to the amber mask, sample the adjacent lens
// at several spacings. A round amber interior sits between darker sections of housing.
fn amber_in_panel(frame: &Frame<'_>, lens: Lens, mid: usize) -> bool {
    let (cx, cy) = lens.centre();
    let direction = if lens.kind == RED { 1. } else { -1. };
    for spacing in [1.5, 1.85, 2.2, 2.55, 2.9] {
        for tilt in [0., -0.23, 0.23] {
            let ax = cx + direction * spacing * lens.w as f64;
            let ay = cy + tilt * lens.h as f64;
            let sample = |dx: f64, dy: f64| {
                let x = (ax + dx * lens.w as f64).round() as isize;
                let y = (ay + dy * lens.h as f64).round() as isize;
                if x < 0 || y < 0 || x as usize >= frame.width || y as usize >= frame.height {
                    None
                } else {
                    Some(frame.pixel(x as usize, y as usize))
                }
            };
            let Some(center) = sample(0., 0.) else {
                continue;
            };
            if !amber_pixel(center) {
                continue;
            }
            let mut bright = 0;
            let mut sum = 0;
            for (x, y) in [(0., 0.), (0.18, 0.), (-0.18, 0.), (0., 0.18), (0., -0.18)] {
                if let Some(p) = sample(x, y) {
                    if amber_pixel(p) {
                        bright += 1;
                        sum += channels(p, AMBER).0;
                    }
                }
            }
            if bright < 4 {
                continue;
            }
            let limit = (sum / bright).saturating_sub(23);
            let mut dark = 0;
            for (x, y) in [(-0.85, 0.), (0.85, 0.), (0., -0.68), (0., 0.68)] {
                if sample(x, y).is_some_and(|p| channels(p, AMBER).0 < limit) {
                    dark += 1;
                }
            }
            if dark >= 3 && opposite_lens(frame, lens, ax, ay, lens.w as f64) {
                let gx = ((ax + cx) * 0.5).round() as usize;
                let gy = ((ay + cy) * 0.5).round().max(0.) as usize;
                if gy < frame.height
                    && channels(frame.pixel(gx, gy), AMBER).0 < 90.max(mid.saturating_sub(13))
                {
                    return true;
                }
            }
        }
    }
    false
}

impl Beads {
    fn collect(&mut self, frame: &Frame<'_>) {
        let step = (frame.width / 320).max(1);
        let cols = frame.width.div_ceil(step);
        let rows = (frame.height * 3 / 4).div_ceil(step);
        let stride = cols + 2;
        self.coarse.resize(stride * (rows + 2), 0);
        self.coarse.fill(0);
        self.lenses.clear();
        for y in 0..rows {
            for x in 0..cols {
                self.coarse[(y + 1) * stride + x + 1] =
                    lens_colours(frame.pixel(x * step, y * step));
            }
        }
        for y in 0..rows {
            for x in 0..cols {
                let at = (y + 1) * stride + x + 1;
                let bits = self.coarse[at];
                if bits == 0 {
                    continue;
                }
                for kind in [RED, GREEN, AMBER] {
                    if bits & kind == 0 {
                        continue;
                    }
                    let c = flood(&mut self.coarse, stride, kind, at, &mut self.queue);
                    let (w, h) = (c.max_x - c.min_x + 1, c.max_y - c.min_y + 1);
                    if w.min(h) < 4
                        || w * 2 < h
                        || w * 10 > 23 * h
                        || w * step > frame.width / 4
                        || h * step > frame.height / 3
                        || c.n * 100 < 32 * w * h
                    {
                        continue;
                    }
                    let (sx, sy) = (c.min_x * step, c.min_y * step);
                    self.lenses.push(Lens {
                        kind,
                        x: sx,
                        y: sy,
                        w: (w * step).min(frame.width - sx),
                        h: (h * step).min(frame.height - sy),
                    });
                }
            }
        }
    }

    fn emitting(&mut self, frame: &Frame<'_>, lens: Lens) -> bool {
        if lens.w.min(lens.h) < 9 {
            return false;
        }
        self.spans.clear();
        let mut luminance = [0; 256];
        let mut fill_light = [0; 256];
        let mut count = 0;
        let (rx, ry) = (lens.w as f64 * 0.5, lens.h as f64 * 0.5);
        for y in 0..lens.h {
            let dy = (y as f64 - (ry - 0.5)) / ry;
            let half2 = 0.78f64.powi(2) - dy * dy;
            if half2 < 0. {
                continue;
            }
            let half = rx * half2.sqrt();
            let a = (rx - 0.5 - half).ceil() as usize;
            let b = (rx - 0.5 + half).floor() as usize;
            if b < a {
                continue;
            }
            self.spans.push((y, a, b));
            for x in a..=b {
                let (v, s) = channels(frame.pixel(lens.x + x, lens.y + y), lens.kind);
                luminance[v] += 1;
                fill_light[s] += 1;
                count += 1;
            }
        }
        if count < 15 {
            return false;
        }
        let mid = median(&luminance, count);
        if !rim_visible(frame, lens, mid) || !self.housing(frame, lens, mid) {
            return false;
        }
        let min_v = 92.max(mid + 42);
        let min_secondary = median(&fill_light, count) + 40;
        let stride = lens.w + 2;
        self.islands.resize(stride * (lens.h + 2), 0);
        self.islands.fill(0);
        for &(y, a, b) in &self.spans {
            for x in a..=b {
                let (v, s) = channels(frame.pixel(lens.x + x, lens.y + y), lens.kind);
                self.islands[(y + 1) * stride + x + 1] = u8::from(v >= min_v && s >= min_secondary);
            }
        }
        let (mut n, mut area, mut quadrants) = (0, 0, 0u8);
        let (mut xmin, mut ymin, mut xmax, mut ymax) = (usize::MAX, usize::MAX, 0, 0);
        for &(y, a, b) in &self.spans {
            for x in a..=b {
                let i = (y + 1) * stride + x + 1;
                if self.islands[i] == 0 {
                    continue;
                }
                let c = flood(&mut self.islands, stride, 1, i, &mut self.queue);
                if c.n < (count / 1600).max(1) || c.n > 4.max(count * 95 / 1000) {
                    continue;
                }
                n += 1;
                area += c.n;
                let cx = c.sum_x / c.n;
                let cy = c.sum_y / c.n;
                xmin = xmin.min(cx);
                xmax = xmax.max(cx);
                ymin = ymin.min(cy);
                ymax = ymax.max(cy);
                quadrants |=
                    1 << (usize::from(2 * cx + 1 > lens.w) + 2 * usize::from(2 * cy + 1 > lens.h));
            }
        }
        n >= 5
            && quadrants.count_ones() >= 3
            && (xmax - xmin) * 10 >= 3 * lens.w
            && (ymax - ymin) * 10 >= 3 * lens.h
            && area * 1000 >= 18 * count
            && area * 100 <= 38 * count
    }

    fn housing(&self, frame: &Frame<'_>, lens: Lens, mid: usize) -> bool {
        self.lenses
            .iter()
            .any(|&a| a.kind == AMBER && common_housing(frame, lens, a, mid))
            || amber_in_panel(frame, lens, mid)
    }

    pub(super) fn core_housing(
        &self,
        frame: &Frame<'_>,
        kind: u8,
        bounds: [usize; 4],
        step: usize,
    ) -> bool {
        let [left, top, right, bottom] = bounds;
        let (x, y) = (left * step, top * step);
        let lens = Lens {
            kind,
            x,
            y,
            w: ((right - left + 1) * step).min(frame.width - x),
            h: ((bottom - top + 1) * step).min(frame.height - y),
        };
        let (cx, cy) = lens.centre();
        let mid = channels(frame.pixel(cx as usize, cy as usize), kind).0;
        if self.housing(frame, lens, mid) {
            return true;
        }
        // Close to the gantry only the lower caps can remain in the top image row.
        // Compare all three caps along a visible row instead of extrapolating their heights.
        if lens.y <= step && kind == GREEN {
            for row in [1, 2.max(lens.h / 8), 2.max(lens.h / 4)] {
                for spacing in [1.7, 2., 2.3, 2.6, 2.9, 3.2] {
                    let ax = cx - spacing * lens.w as f64;
                    let rx = cx - 2. * spacing * lens.w as f64;
                    if rx < lens.w as f64 * 0.2 || row >= frame.height {
                        continue;
                    }
                    let y = row.min(frame.height - 1);
                    let colours = [-0.16, 0., 0.16].map(|off| {
                        let p = frame.pixel((rx + off * lens.w as f64) as usize, y);
                        let (b, g, r) = (i32::from(p.0), i32::from(p.1), i32::from(p.2));
                        amber_pixel(frame.pixel((ax + off * lens.w as f64) as usize, y))
                            && r >= 60
                            && r - b > 28
                            && 100 * (g - b) < 30 * (r - b)
                    });
                    let gap = channels(frame.pixel(((ax + cx) * 0.5) as usize, y), kind).0;
                    if colours.iter().all(|&c| c) && gap < 92 {
                        return true;
                    }
                }
            }
        }
        // A small white emitter may occupy only the middle of the full lens.
        for scale in [1.45, 2.] {
            let (w, h) = (
                (lens.w as f64 * scale).round() as usize,
                (lens.h as f64 * scale).round() as usize,
            );
            let xl = (cx - (w - 1) as f64 * 0.5).round().max(0.) as usize;
            let yl = (cy - (h - 1) as f64 * 0.5).round().max(0.) as usize;
            if self.housing(
                frame,
                Lens {
                    kind,
                    x: xl,
                    y: yl,
                    w: w.min(frame.width - xl),
                    h: h.min(frame.height - yl),
                },
                mid,
            ) {
                return true;
            }
        }
        // A single blooming emitter against a uniformly dark background, as seen
        // indoors, can fill the view while the other lenses are outside the crop.
        if mid < 195 {
            return false;
        }
        let mut black = 0;
        let mut valid = 0;
        for (dx, dy) in [
            (-1., 0.),
            (1., 0.),
            (0., -1.),
            (0., 1.),
            (-0.8, -0.8),
            (0.8, -0.8),
            (-0.8, 0.8),
            (0.8, 0.8),
        ] {
            let sx = (cx + dx * 1.5 * lens.w as f64).round() as isize;
            let sy = (cy + dy * 1.5 * lens.h as f64).round() as isize;
            if sx < 0 || sy < 0 || sx as usize >= frame.width || sy as usize >= frame.height {
                continue;
            }
            valid += 1;
            black += usize::from(channels(frame.pixel(sx as usize, sy as usize), kind).0 < 42);
        }
        valid >= 5 && black * 8 >= valid * 7
    }

    pub(super) fn detect(&mut self, frame: &Frame<'_>) -> Lights {
        self.collect(frame);
        let mut out = Lights::default();
        for i in 0..self.lenses.len() {
            let lens = self.lenses[i];
            if lens.kind == AMBER || !self.emitting(frame, lens) {
                continue;
            }
            if lens.kind == RED {
                out.red = true;
            } else {
                out.green = true;
            }
        }
        out
    }
}
