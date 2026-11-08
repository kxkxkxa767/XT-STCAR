use crate::plant::{Pose, number};
use serde_json::{Value, json};

type Point = (f64, f64);
type Segment = (Point, Point);
pub const FRONT: f64 = 0.20;
pub const REAR: f64 = 0.18;
pub const HALF_WIDTH: f64 = 0.14;

pub struct World {
    pub length: f64,
    pub width: f64,
    pub lane_width: f64,
    pub light_width: f64,
    pub start: Pose,
    pub stripe_x: f64,
    pub gantry_x: f64,
    pub beam_x: f64,
    pub light_y: f64,
    pub post_ys: (f64, f64),
    pub segments: Vec<Segment>,
    pub cones: Vec<(f64, f64, f64)>,
}

fn segment_gap(p: Point, a: Point, b: Point) -> f64 {
    let d = (b.0 - a.0, b.1 - a.1);
    let u = (((p.0 - a.0) * d.0 + (p.1 - a.1) * d.1) / (d.0 * d.0 + d.1 * d.1)).clamp(0., 1.);
    (p.0 - a.0 - u * d.0).hypot(p.1 - a.1 - u * d.1)
}

fn rectangle_gap(p: Point) -> f64 {
    (-REAR - p.0)
        .max(0.)
        .max(p.0 - FRONT)
        .hypot((p.1.abs() - HALF_WIDTH).max(0.))
}

fn segment_hits_rectangle(a: Point, b: Point) -> bool {
    let mut low: f64 = 0.;
    let mut high: f64 = 1.;
    for (origin, delta, min, max) in [
        (a.0, b.0 - a.0, -REAR, FRONT),
        (a.1, b.1 - a.1, -HALF_WIDTH, HALF_WIDTH),
    ] {
        if delta.abs() < 1e-12 {
            if origin < min || origin > max {
                return false;
            }
        } else {
            let u = (min - origin) / delta;
            let v = (max - origin) / delta;
            low = low.max(u.min(v));
            high = high.min(u.max(v));
            if low > high {
                return false;
            }
        }
    }
    true
}

impl World {
    pub fn new(config: &Value) -> Self {
        let length = number(config, "length_m", 8.);
        let width = number(config, "width_m", 4.);
        let lane_width = number(config, "lane_width_m", 0.9);
        let light_width = number(config, "light_width_m", lane_width);
        let divider = number(config, "start_divider_m", 3.2);
        let light_divider = number(config, "light_divider_m", divider);
        let start = Pose {
            x: number(config, "start_x_m", 0.4),
            y: number(config, "start_y_m", lane_width / 2.),
            yaw: number(config, "start_yaw_rad", 0.),
        };
        let gantry_x = number(config, "gantry_x_m", length - light_divider - 0.025);
        // Posts stand at the lane edges unless a narrower gantry is centred in the lane.
        let post_ys = match config["gantry_post_span_m"].as_f64() {
            Some(span) => (
                width - light_width / 2. - span / 2.,
                width - light_width / 2. + span / 2.,
            ),
            None => (width - light_width + 0.07, width - 0.11),
        };
        let mut world = Self {
            length,
            width,
            lane_width,
            light_width,
            start,
            stripe_x: number(config, "stripe_x_m", 2.95_f64.min(divider - 0.15)),
            gantry_x,
            beam_x: gantry_x - number(config, "beam_gap_m", 1.),
            light_y: width - light_width / 2.,
            post_ys,
            segments: vec![
                ((0., 0.), (length, 0.)),
                ((length, 0.), (length, width)),
                ((length, width), (0., width)),
                ((0., width), (0., 0.)),
            ],
            cones: vec![
                (
                    number(config, "cone1_x_m", 0.75 * length),
                    number(config, "cone1_y_m", width / 2.),
                    0.1,
                ),
                (
                    number(config, "cone2_x_m", 0.25 * length),
                    number(config, "cone2_y_m", width / 2.),
                    0.1,
                ),
            ],
        };
        world.board(0., divider, lane_width - 0.01, lane_width + 0.01);
        world.board(
            length - light_divider,
            length,
            width - light_width - 0.01,
            width - light_width + 0.01,
        );
        for y in [post_ys.1, post_ys.0] {
            world.board(gantry_x - 0.025, gantry_x + 0.025, y - 0.025, y + 0.025);
            if config["beam_posts"].as_bool().unwrap_or(true) {
                world.board(
                    world.beam_x - 0.015,
                    world.beam_x + 0.015,
                    y - 0.015,
                    y + 0.015,
                );
            }
        }
        world
    }

    fn board(&mut self, x0: f64, x1: f64, y0: f64, y1: f64) {
        let p = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)];
        for i in 0..4 {
            self.segments.push((p[i], p[(i + 1) % 4]));
        }
    }

    pub fn description(&self) -> Value {
        json!({"length_m":self.length,"width_m":self.width,"lane_width_m":self.lane_width,
               "light_width_m":self.light_width,"stripe_x_m":self.stripe_x,"gantry_x_m":self.gantry_x,
               "beam_x_m":self.beam_x,"post_ys":[self.post_ys.0,self.post_ys.1],"start":[self.start.x,self.start.y,self.start.yaw],
               "cones":self.cones,"segments":self.segments})
    }

    pub fn cast(&self, pose: Pose, angle: f64) -> Option<f64> {
        let (dx, dy) = (angle.cos(), angle.sin());
        let mut best = f64::INFINITY;
        for &((x1, y1), (x2, y2)) in &self.segments {
            let (ex, ey) = (x2 - x1, y2 - y1);
            let den = dx * ey - dy * ex;
            if den.abs() < 1e-12 {
                continue;
            }
            let t = ((x1 - pose.x) * ey - (y1 - pose.y) * ex) / den;
            let u = ((x1 - pose.x) * dy - (y1 - pose.y) * dx) / den;
            if t > 1e-6 && (-1e-9..=1. + 1e-9).contains(&u) {
                best = best.min(t);
            }
        }
        for &(cx, cy, radius) in &self.cones {
            let (fx, fy) = (pose.x - cx, pose.y - cy);
            let b = fx * dx + fy * dy;
            let disc = b * b - (fx * fx + fy * fy - radius * radius);
            if disc >= 0. {
                let t = -b - disc.sqrt();
                if t > 1e-6 {
                    best = best.min(t);
                }
            }
        }
        best.is_finite().then_some(best)
    }

    pub fn gaps(&self, pose: Pose) -> (f64, f64) {
        let (c, s) = (pose.yaw.cos(), pose.yaw.sin());
        let local = |p: Point| {
            (
                c * (p.0 - pose.x) + s * (p.1 - pose.y),
                -s * (p.0 - pose.x) + c * (p.1 - pose.y),
            )
        };
        let corners = [
            (-REAR, -HALF_WIDTH),
            (-REAR, HALF_WIDTH),
            (FRONT, -HALF_WIDTH),
            (FRONT, HALF_WIDTH),
        ];
        let wall = self
            .segments
            .iter()
            .map(|&(a, b)| {
                let (a, b) = (local(a), local(b));
                if segment_hits_rectangle(a, b) {
                    return 0.;
                }
                corners
                    .iter()
                    .map(|&p| segment_gap(p, a, b))
                    .fold(rectangle_gap(a).min(rectangle_gap(b)), f64::min)
            })
            .fold(f64::INFINITY, f64::min);
        let cone = self
            .cones
            .iter()
            .map(|&(x, y, r)| (rectangle_gap(local((x, y))) - r).max(0.))
            .fold(f64::INFINITY, f64::min);
        (wall, cone)
    }

    pub fn front_rear_x(&self, pose: Pose) -> (f64, f64) {
        let (c, s) = (pose.yaw.cos(), pose.yaw.sin().abs());
        let front = if c >= 0. { FRONT * c } else { -REAR * c };
        let rear = if c >= 0. { REAR * c } else { -FRONT * c };
        (
            pose.x + front + HALF_WIDTH * s,
            pose.x - rear - HALF_WIDTH * s,
        )
    }

    pub fn intersects_beam(&self, pose: Pose) -> bool {
        let (c, s) = (pose.yaw.cos(), pose.yaw.sin());
        let local = |y: f64| {
            let (dx, dy) = (self.beam_x - pose.x, y - pose.y);
            (c * dx + s * dy, -s * dx + c * dy)
        };
        segment_hits_rectangle(local(self.post_ys.0), local(self.post_ys.1))
    }
}
