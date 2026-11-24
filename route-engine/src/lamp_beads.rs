//! Resolve LED arrays on a sunlit traffic signal. Compare bright islands with the
//! glass of the same lens. Identify the red left and green right endpoints of a housing.
use super::{Frame, Lights};

const RED: u8 = 1;
const GREEN: u8 = 2;
const WHITE_CORE: u8 = 4;
// Optional geometric witness only. A central cover never contributes a colour vote.
const COVER_MARKER: u8 = 8;

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
    white_ends: Vec<Lens>,
    middle_covers: Vec<Lens>,
    panels: Vec<(Lens, Lens, Lens)>,
    islands: Vec<u8>,
    spans: Vec<(usize, usize, usize)>,
    pub(super) bright_red_array: bool,
}

fn lens_colours((b, g, r): (u8, u8, u8)) -> u8 {
    let (b, g, r) = (i32::from(b), i32::from(g), i32::from(r));
    let red = r >= 72 && 5 * r >= 8 * g && r - b >= 28 && 100 * (g - b) < 33 * (r - b) + 160;
    let green = g >= 26 && 100 * g >= 86 * r && g - b >= 7 && 2 * g - r - b >= 10;
    let cover = r >= 120
        && r - b >= 88
        && g - b >= 23
        && 100 * (g - b) >= 28 * (r - b)
        && 100 * (g - b) <= 77 * (r - b);
    u8::from(red) * RED
        | u8::from(green) * GREEN
        | u8::from(r.min(g).min(b) >= 218) * WHITE_CORE
        | u8::from(cover) * COVER_MARKER
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

fn cover_pixel((b, g, r): (u8, u8, u8), kind: u8) -> bool {
    let (b, g, r) = (i32::from(b), i32::from(g), i32::from(r));
    if kind == RED {
        r >= 53 && r - b >= 24 && r - g >= 20 && 100 * (g - b) < 32 * (r - b)
    } else {
        g >= 22 && g * 100 >= r * 85 && g - b >= 6 && 2 * g - r - b >= 8
    }
}

// Look across the outside outline, perpendicular to the line of lenses. Three
// distinct locations on one long edge suffice when the other edge is clipped.
fn frame_edges(frame: &Frame<'_>, left: Lens, right: Lens) -> usize {
    let (lx, ly) = left.centre();
    let (rx, ry) = right.centre();
    let (dx, dy) = (rx - lx, ry - ly);
    let length = dx.hypot(dy);
    if length < 1. {
        return 0;
    }
    let (nx, ny) = (-dy / length, dx / length);
    let diameter = (left.w + right.w) as f64 * 0.5;
    let sample = |x: f64, y: f64| {
        let (x, y) = (x.round() as isize, y.round() as isize);
        if x < 0 || y < 0 || x as usize >= frame.width || y as usize >= frame.height {
            None
        } else {
            Some(channels(frame.pixel(x as usize, y as usize), RED).0)
        }
    };
    let mut visible_edges = 0;
    for side in [-1., 1.] {
        let mut edge = 0;
        for along in [0.02, 0.25, 0.75, 0.98] {
            let (x, y) = (lx + along * dx, ly + along * dy);
            let at = |radius: f64| {
                sample(
                    x + side * radius * diameter * nx,
                    y + side * radius * diameter * ny,
                )
            };
            let inner = [0.62, 0.76].into_iter().filter_map(at).min();
            let outer = [1.1, 1.32].into_iter().filter_map(at).max();
            if matches!((inner,outer),(Some(i),Some(o)) if i<135 && o>i+22) {
                edge += 1;
            }
        }
        if edge >= 3 {
            visible_edges += 1;
        }
    }
    visible_edges
}

fn panel_fraction(left: Lens, right: Lens, point: (f64, f64)) -> (f64, f64) {
    let (lx, ly) = left.centre();
    let (rx, ry) = right.centre();
    let (dx, dy) = (rx - lx, ry - ly);
    let span = dx.hypot(dy);
    let (x, y) = (point.0 - lx, point.1 - ly);
    (
        (x * dx + y * dy) / (span * span),
        (dx * y - dy * x).abs() / span,
    )
}

// A pair is ordered in the image and shares dark backing just inside each outer lens.
// The central lens is outside these samples, even when it reflects overhead lighting.
fn paired_backing(
    frame: &Frame<'_>,
    first: Lens,
    other: Lens,
    mid: usize,
    slanted_panel: bool,
) -> bool {
    if first.kind == other.kind {
        return false;
    }
    let (cx, cy) = first.centre();
    let (ox, oy) = other.centre();
    let sign = if first.kind == RED { 1. } else { -1. };
    let d = (first.w + other.w) as f64 * 0.5;
    let sep = sign * (ox - cx);
    let clipped = first.y < 2 || other.y < 2;
    if !(2.65 * d..=6.1 * d).contains(&sep)
        || other.w * 100 < first.w * 47
        || other.w * 100 > first.w * 211
        || (oy - cy).abs()
            > if slanted_panel {
                0.80 * sep
            } else if clipped {
                1.05 * d
            } else {
                0.20 * sep
            }
        || (!clipped && (other.h * 100 < first.h * 47 || other.h * 100 > first.h * 211))
    {
        return false;
    }
    let px = (cx + sign * 0.76 * first.w as f64).round();
    let qx = (ox - sign * 0.76 * other.w as f64).round();
    if !(0.0..frame.width as f64).contains(&px) || !(0.0..frame.width as f64).contains(&qx) {
        return false;
    }
    let slope = (oy - cy) / (ox - cx);
    for x in [px, qx] {
        let y = cy + (x - cx) * slope;
        let mut dark = 0;
        let mut seen = 0;
        for offset in [-0.15, 0., 0.15] {
            let sy = (y + offset * d).round() as isize;
            if sy < 0 || sy as usize >= frame.height {
                continue;
            }
            seen += 1;
            dark += usize::from(
                channels(frame.pixel(x as usize, sy as usize), first.kind).0
                    < 88.max(mid.saturating_sub(17)),
            );
        }
        if seen == 0 || dark * 2 < seen {
            return false;
        }
    }
    true
}

fn endpoint_colour(frame: &Frame<'_>, lens: Lens) -> bool {
    let (cx, cy) = lens.centre();
    let mut hits = 0;
    for (dx, dy) in [(0., 0.), (-0.18, 0.), (0.18, 0.), (0., -0.20), (0., 0.20)] {
        let x = (cx + dx * lens.w as f64)
            .round()
            .clamp(0., (frame.width - 1) as f64) as usize;
        let y = (cy + dy * lens.h as f64)
            .round()
            .clamp(0., (frame.height - 1) as f64) as usize;
        hits += usize::from(cover_pixel(frame.pixel(x, y), lens.kind));
    }
    if hits >= 3 {
        return true;
    }
    // A saturated glass can still be located by its coloured edge around a white core.
    let center = frame.pixel(cx as usize, cy as usize);
    if center.0.min(center.1).min(center.2) < 184 {
        return false;
    }
    let mut coloured = 0;
    for (dx, dy) in [
        (1., 0.),
        (-1., 0.),
        (0., 1.),
        (0., -1.),
        (0.7, 0.7),
        (-0.7, 0.7),
        (0.7, -0.7),
        (-0.7, -0.7),
    ] {
        if [0.36, 0.44, 0.51].iter().any(|&radius| {
            let x = (cx + dx * lens.w as f64 * radius).round() as isize;
            let y = (cy + dy * lens.h as f64 * radius).round() as isize;
            x >= 0
                && y >= 0
                && (x as usize) < frame.width
                && (y as usize) < frame.height
                && cover_pixel(frame.pixel(x as usize, y as usize), lens.kind)
        }) {
            coloured += 1;
        }
    }
    coloured >= 4
}

// Search for the opposite *end* when its dark glass merges into the backplate
// in the coarse mask. Only the cover and its immediate rim are sampled.
fn sampled_opposite(frame: &Frame<'_>, lens: Lens, mid: usize) -> bool {
    let (cx, cy) = lens.centre();
    let direction = if lens.kind == RED { 1. } else { -1. };
    let other = if lens.kind == RED { GREEN } else { RED };
    let d = lens.w as f64;
    for spacing in [2.75, 3.05, 3.35, 3.65, 3.95, 4.25, 4.55, 4.85, 5.2, 5.6] {
        let ox = cx + direction * spacing * d;
        if ox - 0.65 * d < 0. || ox + 0.65 * d >= frame.width as f64 {
            continue;
        }
        for tilt in [0., -0.25, 0.25, -0.55, 0.55] {
            let oy = cy + tilt * d;
            if oy < 0. || oy >= frame.height as f64 {
                continue;
            }
            let sample = |dx: f64, dy: f64| {
                let x = (ox + dx * d).round() as isize;
                let y = (oy + dy * d).round() as isize;
                if x < 0 || y < 0 || x as usize >= frame.width || y as usize >= frame.height {
                    None
                } else {
                    Some(frame.pixel(x as usize, y as usize))
                }
            };
            let Some(center) = sample(0., 0.) else {
                continue;
            };
            if !cover_pixel(center, other) && center.0.min(center.1).min(center.2) < 190 {
                continue;
            }
            let mut colours = 0;
            let mut whites = 0;
            let mut interior = 0;
            for (dx, dy) in [(0., 0.), (-0.16, 0.), (0.16, 0.), (0., -0.12), (0., 0.12)] {
                if let Some(p) = sample(dx, dy) {
                    if cover_pixel(p, other) {
                        colours += 1;
                        interior += channels(p, other).0;
                    } else if p.0.min(p.1).min(p.2) >= 190 {
                        whites += 1;
                    }
                }
            }
            if colours < 2 || colours + whites < 4 {
                continue;
            }
            let mut dark_rim = 0;
            for (dx, dy) in [(-0.60, 0.), (0.60, 0.), (0., -0.58), (0., 0.58)] {
                if let Some(p) = sample(dx, dy) {
                    let lum = channels(p, other).0;
                    if (mid >= 190 && lum + 65 < interior / colours)
                        || (!cover_pixel(p, other)
                            && lum
                                < 92.max((interior / colours).saturating_sub(12))
                                    .max(mid.saturating_sub(18)))
                    {
                        dark_rim += 1;
                    }
                }
            }
            if dark_rim >= 3 {
                return true;
            }
        }
    }
    false
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

impl Beads {
    fn collect(&mut self, frame: &Frame<'_>) {
        let step = (frame.width / 320).max(1);
        let cols = frame.width.div_ceil(step);
        let rows = (frame.height * 3 / 4).div_ceil(step);
        let stride = cols + 2;
        self.coarse.resize(stride * (rows + 2), 0);
        self.coarse.fill(0);
        self.lenses.clear();
        self.white_ends.clear();
        self.middle_covers.clear();
        self.panels.clear();
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
                for kind in [RED, GREEN, WHITE_CORE, COVER_MARKER] {
                    if bits & kind == 0 {
                        continue;
                    }
                    let c = flood(&mut self.coarse, stride, kind, at, &mut self.queue);
                    let (w, h) = (c.max_x - c.min_x + 1, c.max_y - c.min_y + 1);
                    if kind == WHITE_CORE {
                        if c.n >= 4 {
                            let bounds = [c.min_x, c.min_y, c.max_x, c.max_y];
                            if let Some(col) = frame
                                .lit_lens(3, bounds, step, c.n)
                                .or_else(|| frame.bloom_colour(bounds, step, c.n))
                            {
                                let (sx, sy) = (c.min_x * step, c.min_y * step);
                                self.white_ends.push(Lens {
                                    kind: col,
                                    x: sx,
                                    y: sy,
                                    w: (w * step).min(frame.width - sx),
                                    h: (h * step).min(frame.height - sy),
                                });
                            }
                        }
                        continue;
                    }
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
                    let lens = Lens {
                        kind,
                        x: sx,
                        y: sy,
                        w: (w * step).min(frame.width - sx),
                        h: (h * step).min(frame.height - sy),
                    };
                    if kind == COVER_MARKER {
                        self.middle_covers.push(lens);
                    } else {
                        self.lenses.push(lens);
                    }
                }
            }
        }
        for &marker in &self.middle_covers {
            let (mx, my) = marker.centre();
            for &left in self.lenses.iter().chain(self.white_ends.iter()) {
                let (lx, ly) = left.centre();
                let d = left.w as f64;
                if left.kind != RED
                    || mx - lx < 0.75 * d
                    || mx - lx > 3.5 * d
                    || (my - ly).abs() > 2.2 * d
                {
                    continue;
                }
                for &right in self.lenses.iter().chain(self.white_ends.iter()) {
                    if right.kind != GREEN
                        || right.centre().0 <= mx
                        || marker.w * 2 < left.w.min(right.w)
                        || marker.w > 2 * left.w.max(right.w)
                    {
                        continue;
                    }
                    let (fraction, perpendicular) = panel_fraction(left, right, (mx, my));
                    if !(0.30..=0.70).contains(&fraction) || perpendicular > d * 0.36 {
                        continue;
                    }
                    let mid = channels(frame.pixel(lx as usize, ly as usize), RED).0;
                    if paired_backing(frame, left, right, mid, true)
                        && frame_edges(frame, left, right) >= 1
                        && endpoint_colour(frame, left)
                        && endpoint_colour(frame, right)
                    {
                        self.panels.push((left, marker, right));
                        break;
                    }
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
        let (mut hot, mut hot_quadrants) = (0usize, 0u8);
        let (mut body, mut red_glass) = (0, 0);
        for &(y, a, b) in &self.spans {
            for x in a..=b {
                let pixel = frame.pixel(lens.x + x, lens.y + y);
                let (v, s) = channels(pixel, lens.kind);
                if lens.kind == RED && v <= mid {
                    let (b, g, r) = (i32::from(pixel.0), i32::from(pixel.1), i32::from(pixel.2));
                    body += 1;
                    red_glass +=
                        usize::from(r - b >= 24 && r - g >= 20 && 100 * (g - b) < 26 * (r - b));
                }
                self.islands[(y + 1) * stride + x + 1] = u8::from(v >= min_v && s >= min_secondary);
                if v >= min_v + 20 && s >= min_secondary + 20 {
                    hot += 1;
                    hot_quadrants |= 1
                        << (usize::from(2 * x + 1 > lens.w) + 2 * usize::from(2 * y + 1 > lens.h));
                }
            }
        }
        // Specular warm highlights on another surface do not turn its underlying glass red.
        if lens.kind == RED && (3 * red_glass < body) {
            return false;
        }
        if self.between_ends(frame, lens) {
            return false;
        }
        let (mut n, mut area, mut quadrants) = (0, 0, 0u8);
        let mut largest = 0;
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
                largest = largest.max(c.n);
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
        let distributed = n >= 5
            && largest * 2 <= area
            && quadrants.count_ones() >= 3
            && (xmax - xmin) * 10 >= 3 * lens.w
            && (ymax - ymin) * 10 >= 3 * lens.h
            && area * 1000 >= 18 * count
            && area * 100 <= 38 * count;
        if distributed
            && lens.kind == RED
            && hot_quadrants.count_ones() >= 3
            && hot * 100 >= 2 * count
        {
            self.bright_red_array = true;
        }
        distributed
    }

    /// A reflection between a real red left cover and green right cover is not an endpoint.
    fn between_ends(&self, frame: &Frame<'_>, candidate: Lens) -> bool {
        let (cx, cy) = candidate.centre();
        for &(left, middle, right) in &self.panels {
            let (f, perp) = panel_fraction(left, right, (cx, cy));
            if (0.28..=0.72).contains(&f)
                && perp < 0.44 * middle.w as f64
                && (cx - middle.centre().0).hypot(cy - middle.centre().1) < 0.53 * middle.w as f64
            {
                return true;
            }
        }
        for &left in &self.lenses {
            if left.kind != RED {
                continue;
            }
            let (lx, ly) = left.centre();
            let d = left.w as f64;
            if cx - lx < 0.9 * d || cx - lx > 3.3 * d || (cy - ly).abs() > d {
                continue;
            }
            for &right in &self.lenses {
                if right.kind != GREEN {
                    continue;
                }
                let (rx, ry) = right.centre();
                if rx <= cx {
                    continue;
                }
                let f = (cx - lx) / (rx - lx);
                if !(0.28..=0.72).contains(&f) || (cy - (ly + f * (ry - ly))).abs() > 0.45 * d {
                    continue;
                }
                let mid = channels(frame.pixel(lx as usize, ly as usize), RED).0;
                if paired_backing(frame, left, right, mid, false)
                    && endpoint_colour(frame, left)
                    && endpoint_colour(frame, right)
                {
                    return true;
                }
            }
        }
        false
    }

    fn housing(&self, frame: &Frame<'_>, lens: Lens, mid: usize) -> bool {
        let (cx, cy) = lens.centre();
        if self.panels.iter().any(|&(left, _, right)| {
            let end = if lens.kind == RED { left } else { right };
            let (ex, ey) = end.centre();
            (cx - ex).hypot(cy - ey) < 0.38 * end.w as f64
                && lens.w <= 2 * end.w
                && lens.h <= 2 * end.h
        }) {
            return true;
        }
        self.lenses.iter().any(|&other| {
            (paired_backing(frame, lens, other, mid, false)
                || (paired_backing(frame, lens, other, mid, true)
                    && 10 * lens.w.min(lens.h) >= 7 * lens.w.max(lens.h)
                    && 10 * other.w.min(other.h) >= 7 * other.w.max(other.h)
                    && frame_edges(frame, lens, other) == 2))
                && endpoint_colour(frame, other)
        }) || self.white_ends.iter().any(|&other| {
            other.w >= 10 && other.h >= 5 && paired_backing(frame, lens, other, mid, false)
        }) || (lens.kind == RED
            && lens.w >= 14
            && lens.w * 4 >= lens.h * 3
            && sampled_opposite(frame, lens, mid))
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
        self.core_in_housing(frame, lens, step)
    }

    fn core_in_housing(&self, frame: &Frame<'_>, lens: Lens, step: usize) -> bool {
        let kind = lens.kind;
        if self.between_ends(frame, lens) {
            return false;
        }
        let (cx, cy) = lens.centre();
        let mid = channels(frame.pixel(cx as usize, cy as usize), kind).0;
        if self.housing(frame, lens, mid) {
            return true;
        }
        // At the image top the other endpoint can be only a thin coloured cap.
        // Search the visible rows for the opposite cover and its local dark rim.
        if lens.y <= step {
            let sign = if kind == GREEN { -1. } else { 1. };
            let opposite = if kind == GREEN { RED } else { GREEN };
            for row in [1, 2.max(lens.h / 8), 2.max(lens.h / 4)] {
                if row >= frame.height {
                    continue;
                }
                for spacing in [2.8, 3.1, 3.4, 3.7, 4.0, 4.3, 4.6, 4.9, 5.2, 5.6] {
                    let ox = cx + sign * spacing * lens.w as f64;
                    let d = lens.w as f64;
                    if ox - 0.75 * d < 0. || ox + 0.75 * d >= frame.width as f64 {
                        continue;
                    }
                    let along = |off: f64| {
                        frame.pixel(
                            (ox + off * d).round().clamp(0., (frame.width - 1) as f64) as usize,
                            row,
                        )
                    };
                    let samples = [-0.15, 0., 0.15].map(|off| along(off));
                    if samples.iter().all(|&p| cover_pixel(p, opposite))
                        && [-0.70, 0.70].iter().all(|&off| {
                            channels(along(off), opposite).0
                                < 98.max(channels(samples[1], opposite).0.saturating_sub(12))
                        })
                    {
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

    /// White components already classified during the coarse pass need only the housing check.
    pub(super) fn saturated_ends(&self, frame: &Frame<'_>, step: usize) -> (Lights, Lights) {
        let (mut visible, mut broad) = (Lights::default(), Lights::default());
        for &lens in &self.white_ends {
            if self.core_in_housing(frame, lens, step) {
                visible.mark(lens.kind);
                if lens.w >= 10 && lens.h >= 5 {
                    broad.mark(lens.kind);
                }
            }
        }
        (visible, broad)
    }
    pub(super) fn detect(&mut self, frame: &Frame<'_>) -> Lights {
        self.bright_red_array = false;
        self.collect(frame);
        let mut out = Lights::default();
        for i in 0..self.lenses.len() {
            let lens = self.lenses[i];
            if !self.emitting(frame, lens) {
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
