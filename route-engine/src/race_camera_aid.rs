use super::{Field, Ray, XY};
use crate::camera_ground::{GroundFrame, PITCH, PanelLine, bearing_bin, to_scan};
use crate::camera_projection::CAM_AHEAD_LIDAR;
use crate::py::V;
use xt_stcar_robot_core::autonomy::{Point2, Pose2};

pub(super) struct CameraScan<'a> {
    pub seq: i64,
    pub at: f64,
    pub ranges: &'a [Option<f64>],
    pub rays: &'a [Ray],
    pub speed: f64,
    pub rate: f64,
    pub role: Option<usize>,
}

#[derive(Clone, Debug, Default)]
pub(super) struct CameraAid {
    latest: Option<GroundFrame>,
    last_frame: Option<i64>,
    pub(super) seq: Option<i64>,
    pub(super) captured: Option<f64>,
    pub(super) points: Vec<XY>,
    pub(super) cone: [Option<XY>; 2],
    cone_depth: Option<&'static str>,
    previous_cone: [Option<(f64, XY)>; 2],
    pitch: Option<f64>,
    wall_hits: [usize; 2],
    depth_anchored: usize,
    measured_lines: [Option<(f64, PanelLine)>; 6],
    updated_axes: [bool; 2],
}

impl CameraAid {
    pub fn receive(&mut self, item: GroundFrame) {
        if item.captured.is_finite()
            && self
                .latest
                .as_ref()
                .is_none_or(|old| item.frame > old.frame && item.captured > old.captured)
        {
            self.latest = Some(item);
        }
    }

    /// Fill only currently missing calibrated ray directions, in a private
    /// planning cloud. A fresh camera frame supplies at most one pose update.
    pub fn update(
        &mut self,
        field: &Field,
        pose: &mut Pose2,
        actual: [bool; 2],
        scan: CameraScan<'_>,
    ) -> [bool; 2] {
        let CameraScan {
            seq,
            at,
            ranges,
            rays,
            speed,
            rate,
            role,
        } = scan;
        let last_confirmed = self.cone.map(|v| v.is_some());
        self.points.clear();
        self.cone = [None; 2];
        self.wall_hits = [0; 2];
        self.updated_axes = [false; 2];
        self.depth_anchored = 0;
        self.cone_depth = None;
        self.seq = Some(seq);
        self.captured = None;
        let Some(frame) = self.latest.as_ref() else {
            return self.updated_axes;
        };
        let age = at - frame.captured;
        if !(0.0..=0.20).contains(&age) {
            return self.updated_axes;
        }
        self.captured = Some(frame.captured);
        let fresh = self.last_frame != Some(frame.frame);
        if (-55i32..=55).all(|b| ranges[b.rem_euclid(360) as usize].is_some()) {
            return self.updated_axes;
        }
        let mut pitch = self.pitch.unwrap_or(PITCH);
        if fresh {
            pitch = frame.refine_pitch(ranges, pitch, age, speed, rate);
            self.pitch = Some(pitch);
        }
        let mut physical: [Vec<XY>; 6] = Default::default();
        let (sin, cos) = pose.yaw_rad.sin_cos();
        for ray in rays {
            let dx = cos * ray.dx - sin * ray.dy;
            let dy = sin * ray.dx + cos * ray.dy;
            let origin = pose.body_to_world(Point2 {
                x_m: ray.ox,
                y_m: ray.oy,
            });
            let Some((id, d)) = field.hit(origin.x_m, origin.y_m, dx, dy) else {
                continue;
            };
            let normal = if field.walls[id].vertical {
                dx.abs()
            } else {
                dy.abs()
            };
            if normal > 0.25
                && (ray.range - d).abs() < 0.25_f64.max(0.12 * d)
                && (ray.range - d).abs() * normal < 0.17
            {
                physical[id].push((origin.x_m + ray.range * dx, origin.y_m + ray.range * dy));
            }
        }
        for (id, points) in physical.iter().enumerate() {
            if let Some(line) = PanelLine::fit(points) {
                self.measured_lines[id] = Some((at, line));
            }
        }
        let mut by_wall: [Vec<(f64, f64, XY)>; 6] = Default::default();
        let mut used = [false; 360];
        let panels = frame.projected_boards(ranges, pitch, age, speed, rate);
        let cam = to_scan((CAM_AHEAD_LIDAR, 0.), age, speed, rate);
        let origin = pose.body_to_world(Point2 {
            x_m: cam.0,
            y_m: cam.1,
        });
        for (i, &(mut q, line_depth)) in panels.iter().enumerate() {
            let angle = pose.yaw_rad + q.1.atan2(q.0);
            let (dy, dx) = angle.sin_cos();
            let Some((id, d)) = field.hit(pose.x_m, pose.y_m, dx, dy) else {
                continue;
            };
            let wall = &field.walls[id];
            let mut w = pose.body_to_world(Point2 { x_m: q.0, y_m: q.1 });
            if !line_depth {
                let Some((when, model)) = self.measured_lines[id] else {
                    continue;
                };
                if at - when > 0.28 {
                    continue;
                }
                let l = panels[i.saturating_sub(1)].0;
                let r = panels[(i + 1).min(panels.len() - 1)].0;
                let (vx, vy) = (r.0 - l.0, r.1 - l.1);
                if !(0.01..=0.75).contains(&vx.hypot(vy)) {
                    continue;
                }
                let Some(hit) = model.intersect(
                    (origin.x_m, origin.y_m),
                    (w.x_m, w.y_m),
                    (cos * vx - sin * vy, sin * vx + cos * vy),
                ) else {
                    continue;
                };
                w = Point2 {
                    x_m: hit.0,
                    y_m: hit.1,
                };
                let body = pose.world_to_body(w);
                q = (body.x_m, body.y_m);
            }
            let bin = bearing_bin(q);
            if used[bin] || ranges[bin].is_some() {
                continue;
            }
            let (along, error, incidence) = if wall.vertical {
                (w.y_m, wall.fixed - w.x_m, dx.abs())
            } else {
                (w.x_m, wall.fixed - w.y_m, dy.abs())
            };
            if incidence < 0.24
                || error.abs() > 0.30
                || (q.0.hypot(q.1) - d).abs() > 0.40_f64.max(d * 0.20)
            {
                continue;
            }
            by_wall[id].push((along, error, q));
            used[bin] = true;
        }
        let mut correction: [Option<(usize, f64)>; 2] = [None; 2];
        for (id, list) in by_wall.iter_mut().enumerate() {
            if list.len() < 4 {
                continue;
            }
            let (lo, hi) = list
                .iter()
                .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), v| {
                    (lo.min(v.0), hi.max(v.0))
                });
            if hi - lo < 0.18 {
                continue;
            }
            list.sort_unstable_by(|a, b| a.1.total_cmp(&b.1));
            if list[3 * list.len() / 4].1 - list[list.len() / 4].1 > 0.12 {
                continue;
            }
            let axis = usize::from(!field.walls[id].vertical);
            let mid = list[list.len() / 2].1;
            self.wall_hits[axis] += list.len();
            self.points.extend(list.iter().map(|v| v.2));
            self.depth_anchored += list.len();
            if correction[axis].is_none_or(|(n, _)| list.len() > n) {
                correction[axis] = Some((list.len(), mid));
            }
        }
        for (index, feet) in frame.cones.iter().enumerate() {
            if role != Some(index) {
                self.previous_cone[index] = None;
                continue;
            }
            let landmark = field.cones[index];
            let mut candidates = Vec::new();
            for foot in feet {
                let Some(p) = foot.point(pitch) else { continue };
                let q = to_scan(p, age, speed, rate);
                let bin = bearing_bin(q);
                let r = q.0.hypot(q.1);
                // Actual returns through or in front of the proposed cone
                // take precedence. A pixel never changes an occupied ray.
                if ranges[bin].is_some_and(|actual| (actual - r).abs() > 0.22_f64.max(0.15 * r)) {
                    continue;
                }
                let world = pose.body_to_world(Point2 { x_m: q.0, y_m: q.1 });
                if (world.x_m - landmark.x_m).hypot(world.y_m - landmark.y_m) <= 0.48 {
                    candidates.push((q, (world.x_m, world.y_m), ranges[bin].is_none()));
                }
            }
            if let [(q, w, missing)] = candidates.as_slice() {
                let stable = self.previous_cone[index].is_some_and(|(when, prev)| {
                    let elapsed = frame.captured - when;
                    elapsed > 0. && elapsed <= 0.32 && (prev.0 - w.0).hypot(prev.1 - w.1) < 0.20
                });
                if fresh {
                    self.previous_cone[index] = Some((frame.captured, *w));
                }
                if *missing && ((fresh && stable) || (!fresh && last_confirmed[index])) {
                    let r = q.0.hypot(q.1);
                    let bin = bearing_bin(*q);
                    let mut near: Vec<_> = (-3i32..=3)
                        .filter_map(|j| ranges[(bin as i32 + j).rem_euclid(360) as usize])
                        .filter(|v| (v - r).abs() < 0.20_f64.max(0.13 * r))
                        .collect();
                    near.sort_unstable_by(f64::total_cmp);
                    let paired = near.len() >= 2 && near[near.len() - 1] - near[0] < 0.18;
                    let normal = (-((w.1 - pose.y_m) / r), (w.0 - pose.x_m) / r);
                    if paired {
                        let depth = near[near.len() / 2] + 0.04;
                        let body = (q.0 * depth / r, q.1 * depth / r);
                        self.cone[index] = Some(body);
                        self.points.push(body);
                        self.cone_depth = Some("adjacent_lidar");
                        let w = pose.body_to_world(Point2 {
                            x_m: body.0,
                            y_m: body.1,
                        });
                        for (axis, d) in [landmark.x_m - w.x_m, landmark.y_m - w.y_m]
                            .into_iter()
                            .enumerate()
                        {
                            if correction[axis].is_none() {
                                correction[axis] = Some((1, d));
                            }
                        }
                    } else {
                        // A single camera bearing does not measure translation along that ray.
                        let anchor = pose.world_to_body(landmark);
                        let d = (anchor.x_m * q.0 + anchor.y_m * q.1) / r;
                        self.cone[index] = Some((d * q.0 / r, d * q.1 / r));
                        self.cone_depth = Some("bearing_with_map_depth");
                        let across =
                            normal.0 * (landmark.x_m - w.0) + normal.1 * (landmark.y_m - w.1);
                        for (axis, n) in [normal.0, normal.1].into_iter().enumerate() {
                            if n.abs() > 0.60
                                && !actual[axis]
                                && correction[axis].is_none()
                                && (actual[1 - axis] || correction[1 - axis].is_some())
                            {
                                correction[axis] = Some((1, across / n));
                                break;
                            }
                        }
                    }
                }
            } else if fresh {
                self.previous_cone[index] = None;
            }
        }
        if fresh {
            for (i, value) in correction.into_iter().enumerate() {
                if !actual[i]
                    && let Some((_, d)) = value
                {
                    let step = 0.5 * d.clamp(-0.30, 0.30);
                    if i == 0 {
                        pose.x_m += step;
                    } else {
                        pose.y_m += step;
                    }
                    self.updated_axes[i] = true;
                }
            }
            self.last_frame = Some(frame.frame);
        }
        self.updated_axes
    }

    pub fn diagnostics(&self) -> V {
        if self.latest.is_none() {
            return V::None;
        }
        crate::dict! {"frame"=>self.last_frame,"source_seq"=>self.seq,"captured_at"=>self.captured,
        "filled_bins"=>self.points.len() as i64,"lidar_anchored_wall_bins"=>self.depth_anchored as i64,
        "wall_support_xy"=>vec![self.wall_hits[0] as i64,self.wall_hits[1] as i64],
        "corrected_axes"=>vec![V::Bool(self.updated_axes[0]),V::Bool(self.updated_axes[1])],
        "red_cone"=>self.cone[0].is_some(),"blue_cone"=>self.cone[1].is_some(),"cone_depth"=>self.cone_depth,
        "pitch_deg"=>self.pitch.map(f64::to_degrees)}
    }
}
