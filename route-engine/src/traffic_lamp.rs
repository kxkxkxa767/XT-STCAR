//! Lit traffic signals in BGR camera frames. A coloured lens in daylight can be
//! quite saturated. Look for emitted light inside a compact lens and a dark housing.

#[path = "lamp_beads.rs"]
mod lamp_beads;

#[derive(Default)]
pub struct Detector {
    beads: lamp_beads::Beads,
    mask: Vec<u8>,
    pending: Vec<usize>,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Lights {
    pub red: bool,
    pub green: bool,
}

impl Lights {
    fn mark(&mut self, kind: u8) {
        if kind == 1 {
            self.red = true;
        } else {
            self.green = true;
        }
    }
}

fn colour(b: u8, g: u8, r: u8) -> u8 {
    let (b, g, r) = (i32::from(b), i32::from(g), i32::from(r));
    if r >= 211 && r - g >= 34 && r - b >= 22 && 100 * (g - b) < 32 * (r - b) + 160 {
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

    // Indoor signals may saturate the camera across the entire disc. Read the
    // coloured halo outside a compact white core; the caller verifies the housing.
    fn bloom_colour(&self, bounds: [usize; 4], step: usize, count: usize) -> Option<u8> {
        let [left, top, right, bottom] = bounds;
        let (w, h) = ((right - left + 1) * step, (bottom - top + 1) * step);
        if w < 10
            || h < 4
            || w > self.width / 4
            || h > self.height / 4
            || 2 * w < h
            || w > h * if top == 0 { 4 } else { 2 }
            || count * step * step * 2 < w * h
        {
            return None;
        }
        let (cx, cy) = (
            (left + right) as f64 * step as f64 * 0.5,
            (top + bottom) as f64 * step as f64 * 0.5,
        );
        let (mut red, mut green, mut visible) = (0, 0, 0);
        for (dx, dy) in [
            (1., 0.),
            (0.87, 0.5),
            (0.5, 0.87),
            (0., 1.),
            (-0.5, 0.87),
            (-0.87, 0.5),
            (-1., 0.),
            (-0.87, -0.5),
            (-0.5, -0.87),
            (0., -1.),
            (0.5, -0.87),
            (0.87, -0.5),
        ] {
            let (mut seen, mut r, mut g) = (false, false, false);
            for factor in [0.51, 0.59, 0.7, 0.85] {
                let x = (cx + dx * w as f64 * factor).round() as isize;
                let y = (cy + dy * h as f64 * factor).round() as isize;
                if x < 0 || y < 0 || x as usize >= self.width || y as usize >= self.height {
                    continue;
                }
                seen = true;
                let (blue, gr, re) = self.pixel(x as usize, y as usize);
                let (b, v, u) = (i32::from(blue), i32::from(gr), i32::from(re));
                r |= u >= 160 && u - v >= 38 && u - b >= 36 && 100 * (v - b) < 32 * (u - b) + 160;
                g |= v >= 135 && v - u >= 22 && v - b >= 2;
            }
            red += i32::from(r);
            green += i32::from(g);
            visible += i32::from(seen);
        }
        if red >= 5 && red * 3 >= 2 * visible && green <= 2 {
            Some(1)
        } else if green >= 5 && green * 3 >= 2 * visible && red <= 2 {
            Some(2)
        } else {
            None
        }
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
        (white * 18 >= area && bright_colour * 12 >= area && coloured_corners <= 3).then_some(kind)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A tilted black housing with independently controllable left/red and right/green ends.
    /// The centre can be covered or reflect a small white patch without changing the signal.
    fn test_housing(angle: f64, left: u8, right: u8, middle: bool) -> Vec<u8> {
        let mut image = vec![200; 320 * 240 * 3];
        let (sin, cos) = angle.sin_cos();
        for y in 0..240 {
            for x in 0..320 {
                let dx = x as f64 - 160.;
                let dy = y as f64 - 118.;
                let u = dx * cos + dy * sin;
                let v = -dx * sin + dy * cos;
                if u.abs() > 103. || v.abs() > 25. {
                    continue;
                }
                let mut px = [32, 40, 46];
                for (index, cx) in [-69., 0., 69.].into_iter().enumerate() {
                    let a = u - cx;
                    let q = a * a + v * v;
                    if q > 17.5 * 17.5 {
                        continue;
                    }
                    if index == 1 {
                        if middle {
                            px = if (a - 1.).powi(2) + (v - 1.).powi(2) < 26. {
                                [252, 252, 250]
                            } else {
                                [20, 115, 207]
                            };
                        }
                        continue;
                    }
                    let kind = if index == 0 { left } else { right };
                    px = if index == 0 {
                        [20, 48, 197]
                    } else {
                        [65, 136, 109]
                    };
                    if kind == 1 {
                        // A single bright reflection with a few much smaller specks on the red cover.
                        if (a - 3.).powi(2) + (v - 4.).powi(2) < 11.
                            || [(-8., -6.), (-5., 6.), (6., -7.), (8., 3.)]
                                .iter()
                                .any(|(xx, yy)| (a - xx).powi(2) + (v - yy).powi(2) < 0.9)
                        {
                            px = [115, 168, 238];
                        }
                    } else if kind == 2 {
                        // Individual emitters can be fainter than the opposite tinted red glass.
                        if [-8., 0., 8.].iter().any(|xx| {
                            [-8., 0., 8.]
                                .iter()
                                .any(|yy| (a - xx).powi(2) + (v - yy).powi(2) < 3.)
                        }) {
                            px = if index == 0 {
                                [180, 220, 251]
                            } else {
                                [155, 209, 176]
                            };
                        }
                    } else if kind == 3 {
                        px = if q < 10.5 * 10.5 {
                            [244, 248, 244]
                        } else if index == 0 {
                            [18, 38, 250]
                        } else {
                            [56, 234, 49]
                        };
                    }
                }
                let k = (y * 320 + x) * 3;
                image[k..k + 3].copy_from_slice(&px);
            }
        }
        image
    }

    #[test]
    fn ends_win_over_centre_glare_and_fading_red_even_on_a_tilted_housing() {
        let mut detector = Detector::default();
        for angle in [0.0, 0.30, -0.32] {
            for middle in [false, true] {
                for (left, right, expected) in [
                    (0, 0, (false, false)),
                    (1, 0, (false, false)),
                    (1, 3, (false, true)),
                    (1, 2, (false, true)),
                    (3, 0, (true, false)),
                    (2, 3, (true, false)),
                    (3, 3, (true, false)),
                ] {
                    let image = test_housing(angle, left, right, middle);
                    let result = detector.detect(&image, 320, 240);
                    assert_eq!(
                        (result.red, result.green),
                        expected,
                        "angle {angle} middle {middle} ends {left}/{right}"
                    );
                }
            }
        }
    }
    #[test]
    fn sunlight_on_coloured_glass_does_not_mask_the_active_beads() {
        let (width, height) = (320, 240);
        let mut detector = Detector::default();
        for (lit, raised) in [(0, 0), (1, 0), (2, 0), (2, 12)] {
            let mut image = vec![190u8; width * height * 3];
            for y in 25..81usize {
                for x in 28..293usize {
                    let mut bgr = [34, 43, 54];
                    for (i, cx) in [80i32, 160, 240].into_iter().enumerate() {
                        let (dx, dy) = (x as i32 - cx, y as i32 - 53);
                        if dx * dx + dy * dy > 20 * 20 {
                            continue;
                        }
                        bgr = match i {
                            0 => [17, 48, 224],
                            1 => [24, 126, 241],
                            _ => [75, 157, 129],
                        };
                        if (i == 0 && lit == 1 || i == 2 && lit == 2)
                            && [-10, -2, 7].iter().any(|px| {
                                [-10, -2, 7]
                                    .iter()
                                    .any(|py| (dx - px) * (dx - px) + (dy - py) * (dy - py) <= 2)
                            })
                        {
                            bgr = if i == 0 {
                                [165, 217, 253]
                            } else {
                                [194, 244, 212]
                            };
                        }
                    }
                    let at = (y * width + x) * 3;
                    image[at..at + 3].copy_from_slice(&bgr.map(|v: u8| v.saturating_add(raised)));
                }
            }
            let lights = detector.detect(&image, width, height);
            assert_eq!((lights.red, lights.green), (lit == 1, lit == 2));
        }
    }

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
        if width < 16
            || height < 16
            || width.checked_mul(height).and_then(|n| n.checked_mul(3)) != Some(bgr.len())
        {
            return Lights::default();
        }
        let frame = Frame { bgr, width, height };
        let beads = self.beads.detect(&frame);
        let step = (width / 320).max(1);
        let (mut lights, white) = self.beads.saturated_ends(&frame, step);
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
                    0
                } else {
                    // Overexposed red LEDs retain an orange-red border around the white stripes.
                    // Keep that rim connected here; the later hue and housing checks assign the end.
                    let (red, green, blue) = (i32::from(r), i32::from(g), i32::from(b));
                    if red >= 211
                        && red - green >= 34
                        && red - blue >= 22
                        && 2 * (green - blue) < red - blue + 16
                    {
                        1
                    } else {
                        colour(b, g, r)
                    }
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
                let bounds = [left, top, right, bottom];
                if count >= 4
                    && let Some(lens) = frame.lit_lens(kind, bounds, step, count)
                    && self.beads.core_housing(&frame, lens, bounds, step)
                {
                    lights.mark(lens);
                }
            }
        }
        // Resolve the two endpoints only after checking their saturated cores as well
        // as their bead arrays. A real red emitter still wins a simultaneous transition.
        let red = white.red
            || self.beads.bright_red_array
            || (!white.green && (beads.red || (!beads.green && lights.red)));
        Lights {
            red,
            green: !red && (white.green || beads.green || lights.green),
        }
    }
}
