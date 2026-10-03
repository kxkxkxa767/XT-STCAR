//! Offline capsule walls, shared by synthetic lidar and independent plant checks.
use serde::{Deserialize, Serialize};
use xt_stcar_robot_core::autonomy::{Footprint, Point2, Pose2, Rect};

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct SimWall {
    pub start: Point2,
    pub end: Point2,
    pub half_width_m: f64,
}

impl SimWall {
    pub fn disc_clearance(&self, center: Point2, radius_m: f64) -> f64 {
        point_segment_distance(center, self.start, self.end) - self.half_width_m - radius_m
    }
    pub fn valid(&self, bounds: Rect) -> bool {
        self.start.valid()
            && self.end.valid()
            && self.half_width_m.is_finite()
            && (0.001..=0.1).contains(&self.half_width_m)
            && self.start.distance(self.end) >= 0.01
            && bounds.contains(self.start)
            && bounds.contains(self.end)
    }

    /// Nearest ray/capsule intersection, including rounded end caps.
    pub fn ray_distance(&self, origin: Point2, dx: f64, dy: f64) -> Option<f64> {
        if point_segment_distance(origin, self.start, self.end) <= self.half_width_m {
            return Some(0.0);
        }
        let length = self.start.distance(self.end);
        let ux = (self.end.x_m - self.start.x_m) / length;
        let uy = (self.end.y_m - self.start.y_m) / length;
        let ox = origin.x_m - self.start.x_m;
        let oy = origin.y_m - self.start.y_m;
        let normal_origin = -uy * ox + ux * oy;
        let normal_direction = -uy * dx + ux * dy;
        let mut nearest = f64::INFINITY;
        if normal_direction.abs() > 1e-12 {
            for side in [-self.half_width_m, self.half_width_m] {
                let t = (side - normal_origin) / normal_direction;
                let along = (ox + t * dx) * ux + (oy + t * dy) * uy;
                if t >= 0.0 && (0.0..=length).contains(&along) {
                    nearest = nearest.min(t);
                }
            }
        }
        for center in [self.start, self.end] {
            let x = origin.x_m - center.x_m;
            let y = origin.y_m - center.y_m;
            let b = x * dx + y * dy;
            let discriminant = b * b - (x * x + y * y - self.half_width_m.powi(2));
            if discriminant >= 0.0 {
                let t = -b - discriminant.sqrt();
                if t >= 0.0 {
                    nearest = nearest.min(t);
                }
            }
        }
        nearest.is_finite().then_some(nearest)
    }

    /// Signed clearance from the complete oriented rectangular body.
    pub fn body_clearance(&self, footprint: Footprint, pose: Pose2) -> f64 {
        let a = pose.world_to_body(self.start);
        let b = pose.world_to_body(self.end);
        let xmin = -footprint.rear_m;
        let xmax = footprint.front_m;
        let ymin = -footprint.half_width_m;
        let ymax = footprint.half_width_m;
        let mut near: f64 = 0.0;
        let mut far: f64 = 1.0;
        let mut intersects = true;
        for (o, d, min, max) in [
            (a.x_m, b.x_m - a.x_m, xmin, xmax),
            (a.y_m, b.y_m - a.y_m, ymin, ymax),
        ] {
            if d.abs() < 1e-12 {
                if o < min || o > max {
                    intersects = false;
                }
            } else {
                let t1 = (min - o) / d;
                let t2 = (max - o) / d;
                near = near.max(t1.min(t2));
                far = far.min(t1.max(t2));
            }
        }
        if intersects && near <= far {
            return -self.half_width_m;
        }
        let endpoint_distance = |p: Point2| {
            (xmin - p.x_m)
                .max(p.x_m - xmax)
                .max(0.0)
                .hypot((ymin - p.y_m).max(p.y_m - ymax).max(0.0))
        };
        let mut distance = endpoint_distance(a).min(endpoint_distance(b));
        for (x, y) in [(xmin, ymin), (xmin, ymax), (xmax, ymin), (xmax, ymax)] {
            distance = distance.min(point_segment_distance(Point2 { x_m: x, y_m: y }, a, b));
        }
        distance - self.half_width_m
    }
}

fn point_segment_distance(p: Point2, a: Point2, b: Point2) -> f64 {
    let dx = b.x_m - a.x_m;
    let dy = b.y_m - a.y_m;
    let t = (((p.x_m - a.x_m) * dx + (p.y_m - a.y_m) * dy) / (dx * dx + dy * dy)).clamp(0.0, 1.0);
    (p.x_m - a.x_m - t * dx).hypot(p.y_m - a.y_m - t * dy)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn wall() -> SimWall {
        SimWall {
            start: Point2 { x_m: 0.0, y_m: 1.0 },
            end: Point2 { x_m: 2.0, y_m: 1.0 },
            half_width_m: 0.02,
        }
    }
    #[test]
    fn lidar_hits_side_and_round_end_but_not_open_extension() {
        let wall = wall();
        assert!(
            (wall
                .ray_distance(Point2 { x_m: 1.0, y_m: 0.0 }, 0.0, 1.0)
                .unwrap()
                - 0.98)
                .abs()
                < 1e-12
        );
        assert!(
            (wall
                .ray_distance(Point2 { x_m: 3.0, y_m: 1.0 }, -1.0, 0.0)
                .unwrap()
                - 0.98)
                .abs()
                < 1e-12
        );
        assert!(
            wall.ray_distance(Point2 { x_m: 3.0, y_m: 0.0 }, 0.0, 1.0)
                .is_none()
        );
    }
    #[test]
    fn complete_body_and_segment_crossing_are_checked_under_rotation() {
        let footprint = Footprint {
            front_m: 0.22,
            rear_m: 0.18,
            half_width_m: 0.13,
        };
        let wall = wall();
        let pose = Pose2 {
            x_m: 1.0,
            y_m: 0.8,
            yaw_rad: 0.0,
        };
        assert!((wall.body_clearance(footprint, pose) - 0.05).abs() < 1e-12);
        assert!(wall.body_clearance(footprint, Pose2 { y_m: 0.9, ..pose }) < 0.0);
        // Both wall ends can be outside the body while its middle intersects it.
        assert!(
            wall.body_clearance(
                footprint,
                Pose2 {
                    y_m: 1.0,
                    yaw_rad: std::f64::consts::FRAC_PI_2,
                    ..pose
                }
            ) < 0.0
        );
    }
}
