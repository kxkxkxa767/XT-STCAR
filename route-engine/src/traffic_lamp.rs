//! Lit traffic signals in BGR camera frames. A coloured lens in daylight can be
//! quite saturated. Look for emitted light inside a compact lens and a dark housing.

#[derive(Default)]
pub struct Detector {
    mask: Vec<u8>,
    pending: Vec<usize>,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Lights {
    pub red: bool,
    pub green: bool,
}

fn colour(b: u8, g: u8, r: u8) -> u8 {
    let (b, g, r) = (i32::from(b), i32::from(g), i32::from(r));
    if r >= 211 && r - g >= 34 && r - b >= 22 && 2 * (g - b) < r - b + 16 {
        1
    } else if g >= 154 && g - r >= 24 && g - b >= 6 {
        2
    } else {
        0
    }
}

struct Frame<'a> {
    bgr: &'a [u8],
    width: usize,
    height: usize,
}

impl Frame<'_> {
    fn pixel(&self, x: usize, y: usize) -> (u8, u8, u8) {
        let at = (y * self.width + x) * 3;
        (self.bgr[at], self.bgr[at + 1], self.bgr[at + 2])
    }

    fn lit_lens(&self, kind: u8, bounds: [usize; 4], step: usize, count: usize) -> Option<u8> {
        let [left, top, right, bottom] = bounds;
        let (w, h) = (right - left + 1, bottom - top + 1);
        let area = w * h;
        let clipped = top == 0 || left == 0 || (right + 1) * step >= self.width;
        if w < 3
            || h < 2
            || w * step > self.width / 4
            || h * step > self.height / 3
            || w * 2 < h
            || w > h * if clipped { 4 } else { 2 }
            || count * 12 < area
        {
            return None;
        }
        // At the edge of the image only count the portion of the housing we can see.
        let cx = (left + right) as f64 * step as f64 / 2.;
        let cy = (top + bottom) as f64 * step as f64 / 2.;
        let rx = (w * step) as f64 / 2.;
        let ry = (h * step) as f64 / 2.;
        const DIRECTIONS: [(f64, f64); 12] = [
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
        let (mut visible, mut dark, mut red_rays, mut green_rays) = (0, 0, 0, 0);
        for (dx, dy) in DIRECTIONS {
            let sample = |radius: f64| {
                let x = (cx + dx * rx * radius).round() as isize;
                let y = (cy + dy * ry * radius).round() as isize;
                (x >= 0 && y >= 0 && x < self.width as isize && y < self.height as isize)
                    .then(|| self.pixel(x as usize, y as usize))
            };
            let mut seen = false;
            let mut black = false;
            for radius in [1.20, 1.55, 1.90, 2.35] {
                if let Some((b, g, r)) = sample(radius) {
                    seen = true;
                    if u16::from(b) + 2 * u16::from(g) + u16::from(r) < 370 && b.max(g).max(r) < 160
                    {
                        black = true;
                        break;
                    }
                }
            }
            visible += i32::from(seen);
            dark += i32::from(black);
            if kind == 3 {
                let (mut red, mut green) = (false, false);
                for radius in [0.92, 1.02, 1.14, 1.32, 1.55] {
                    if let Some((b, g, r)) = sample(radius) {
                        let c = colour(b, g, r);
                        red |= c == 1;
                        green |= c == 2;
                    }
                }
                red_rays += i32::from(red);
                green_rays += i32::from(green);
            }
        }
        if visible < 6 {
            return None;
        }
        if kind == 3 {
            // A clipped white emitter retains a coloured halo on several sides.
            // Bloom can wash out part of the black backplate, especially on red.
            if count * 2 < area || dark * 3 < visible {
                return None;
            }
            if red_rays >= 4 && red_rays * 2 >= visible && green_rays <= 2 {
                return Some(1);
            }
            if green_rays >= 4 && green_rays * 2 >= visible && red_rays <= 2 {
                return Some(2);
            }
            return None;
        }
        if dark * 10 < visible * 6 {
            return None;
        }

        let (mut white, mut bright_colour) = (0, 0);
        for y in top..=bottom {
            for x in left..=right {
                let (b, g, r) = self.pixel(x * step, y * step);
                white += usize::from(b.min(g).min(r) >= 208);
                let peak = if kind == 1 { r } else { g };
                bright_colour += usize::from(peak >= 225 && colour(b, g, r) == kind);
            }
        }
        let mut coloured_corners = 0;
        for x in [left + w / 10, right - w / 10] {
            for y in [top + h / 10, bottom - h / 10] {
                let (b, g, r) = self.pixel(x * step, y * step);
                coloured_corners += usize::from(colour(b, g, r) == kind);
            }
        }
        ((white * 18 >= area && bright_colour * 12 >= area)
            || (bright_colour * 2 >= area && dark * 4 >= visible * 3 && coloured_corners <= 2))
            .then_some(kind)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lit_lenses_are_distinct_from_red_panels_and_dark_lenses() {
        let mut detector = Detector::default();
        for (rim, lit) in [
            ([20, 30, 250], Some(1)),
            ([65, 243, 30], Some(2)),
            ([18, 26, 105], None),
        ] {
            let mut image = vec![0u8; 320 * 240 * 3];
            for y in 0..240i32 {
                for x in 0..320i32 {
                    let d = (x - 90).pow(2) + (y - 55).pow(2);
                    let pixel = if d < 81 && lit.is_some() {
                        [245, 245, 245]
                    } else if d < 196 {
                        rim
                    } else if (190..220).contains(&x) && (45..75).contains(&y) {
                        [10, 15, 245]
                    } else if y > 118 {
                        [35, 65, 225]
                    } else {
                        [26, 26, 26]
                    };
                    let i = (y * 320 + x) as usize * 3;
                    image[i..i + 3].copy_from_slice(&pixel);
                }
            }
            let lights = detector.detect(&image, 320, 240);
            assert_eq!((lights.red, lights.green), (lit == Some(1), lit == Some(2)));
        }
    }
}

impl Detector {
    pub fn detect(&mut self, bgr: &[u8], width: usize, height: usize) -> Lights {
        let mut lights = Lights::default();
        if width < 16
            || height < 16
            || width.checked_mul(height).and_then(|n| n.checked_mul(3)) != Some(bgr.len())
        {
            return lights;
        }
        let frame = Frame { bgr, width, height };
        let step = (width / 320).max(1);
        let cols = width.div_ceil(step);
        // The gantry is above the car. Include the horizon on approach and lamps cut by the top edge.
        let rows = (height * 3 / 4).div_ceil(step);
        let stride = cols + 2;
        self.mask.resize(stride * (rows + 2), 0);
        self.mask.fill(0);
        for y in 0..rows {
            for x in 0..cols {
                let (b, g, r) = frame.pixel(x * step, y * step);
                self.mask[(y + 1) * stride + x + 1] = if b.min(g).min(r) >= 218 {
                    3
                } else {
                    colour(b, g, r)
                };
            }
        }
        for y in 0..rows {
            for x in 0..cols {
                let at = (y + 1) * stride + x + 1;
                let kind = self.mask[at];
                if kind == 0 {
                    continue;
                }
                self.mask[at] = 0;
                self.pending.push(at);
                let (mut left, mut top, mut right, mut bottom, mut count) = (x, y, x, y, 0);
                while let Some(i) = self.pending.pop() {
                    let px = i % stride - 1;
                    let py = i / stride - 1;
                    (left, top, right, bottom) =
                        (left.min(px), top.min(py), right.max(px), bottom.max(py));
                    count += 1;
                    for j in [
                        i - stride - 1,
                        i - stride,
                        i - stride + 1,
                        i - 1,
                        i + 1,
                        i + stride - 1,
                        i + stride,
                        i + stride + 1,
                    ] {
                        if self.mask[j] == kind {
                            self.mask[j] = 0;
                            self.pending.push(j);
                        }
                    }
                }
                if count >= 4
                    && let Some(lens) =
                        frame.lit_lens(kind, [left, top, right, bottom], step, count)
                {
                    if lens == 1 {
                        lights.red = true;
                    } else {
                        lights.green = true;
                    }
                }
            }
        }
        lights
    }
}
