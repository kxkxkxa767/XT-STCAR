//! The virtual end line joins the two side/end-board corners. A bowed panel's
//! middle is not extra lane length. Only race distance consumers use this;
//! the original returns remain unchanged for the body-clearance gate.
use crate::{dict, helpers::SIDE_BODY_EXTENT_M, py::V};

#[derive(Clone, Copy, Debug)]
struct Point { x: f64, y: f64, angle: f64, bin: usize, age: f64 }

#[derive(Clone, Copy, Debug)]
pub(crate) struct EndBoard {
    pub range: f64,
    left: Point,
    right: Point,
    timed: bool,
}

impl EndBoard {
    pub fn diagnostics(self) -> V {
        dict! {"basis" => "side_end_corner_chord", "axis_range_m" => self.range,
            "left_corner_m" => vec![self.left.x, -self.left.y],
            "right_corner_m" => vec![self.right.x, -self.right.y],
            "corner_bins" => vec![self.left.bin as i64, self.right.bin as i64],
            "corner_age_ms" => vec![self.left.age * 1000., self.right.age * 1000.],
            "receive_time_deskew" => self.timed}
    }
}

fn line(points: &[Point], end: bool) -> Option<(f64, f64, f64)> {
    let n = points.len() as f64;
    let (u, v) = points.iter().fold((0., 0.), |(u,v), p| {
        if end { (u+p.y, v+p.x) } else { (u+p.x, v+p.y) }
    });
    let (u, v) = (u/n, v/n);
    let (uu, uv) = points.iter().fold((0., 0.), |(uu,uv), p| {
        let (a,b) = if end { (p.y-u, p.x-v) } else { (p.x-u, p.y-v) };
        (uu+a*a, uv+a*b)
    });
    if uu < 1e-6 { return None; }
    let slope = uv/uu;
    let intercept = v-slope*u;
    let error = (points.iter().map(|p| {
        let residual = if end { p.x-intercept-slope*p.y } else { p.y-intercept-slope*p.x };
        residual*residual
    }).sum::<f64>()/n).sqrt();
    Some((intercept, slope, error))
}

fn corner(points: &[Point], sign: f64, front: f64) -> Option<Point> {
    let mut arc: Vec<Point> = points.iter().copied().filter(|p| p.angle*sign > 0.).collect();
    arc.sort_unstable_by(|a,b| a.angle.abs().total_cmp(&b.angle.abs()));
    let mut sides: Vec<f64> = arc.iter().filter(|p| (30_f64.to_radians()..=80_f64.to_radians()).contains(&p.angle.abs()))
        .map(|p| p.y.abs()).collect();
    if arc.len() < 5 || sides.is_empty() { return None; }
    sides.sort_unstable_by(f64::total_cmp);
    // A side opening can expose a different lane. Use the near side surface
    // to delimit this arc, not the distant wall visible through that opening.
    let side = sides[sides.len()/4];
    let k = arc.iter().enumerate().filter(|(_,p)| p.x > 0. && (SIDE_BODY_EXTENT_M..=2.*side).contains(&p.y.abs()))
        .max_by(|(_,a),(_,b)| (a.x*side+a.y.abs()*front).total_cmp(&(b.x*side+b.y.abs()*front)))?.0;
    let mut best: Option<(f64, Point)> = None;
    for j in k.saturating_sub(2).max(2)..=(k+2).min(arc.len()-3) {
        let Some((a,b,e)) = line(&arc[j-2..=j], true) else { continue };
        let Some((c,d,f)) = line(&arc[j..=j+2], false) else { continue };
        if b.abs() > 2. || d.abs() > 0.5 || e > 0.02 || f > 0.02 || (1.-b*d).abs() < 0.1 { continue; }
        let x = (a+b*c)/(1.-b*d);
        let y = c+d*x;
        let p = arc[j];
        let span = (arc[j+1].x-arc[j-1].x).hypot(arc[j+1].y-arc[j-1].y);
        if (x-p.x).hypot(y-p.y) > 0.03_f64.max(span*0.5) || y.abs() < SIDE_BODY_EXTENT_M { continue; }
        if best.is_none_or(|(error,_)| e+f < error) { best = Some((e+f, Point { x, y, ..p })); }
    }
    best.map(|(_,p)| p)
}

/// Heading is the car heading relative to the end line's axis: add PI for
/// the start's rear board. Speed and yaw are the existing measured body motion.
pub(crate) fn observe(scan: &V, heading: f64, speed: Option<f64>, yaw: Option<f64>) -> Option<EndBoard> {
    let ranges = scan.get("ranges");
    if !ranges.is_list() || ranges.len() != 360 || !heading.is_finite() { return None; }
    let ages = scan.get("point_age_ms");
    let speed = speed.filter(|s| s.is_finite());
    let timed = ages.is_list() && ages.len() == 360 && speed.is_some();
    let speed = speed.unwrap_or(0.);
    let yaw = yaw.filter(|y| y.is_finite()).unwrap_or(0.);
    let mut points = Vec::with_capacity(180);
    for (bin, r) in ranges.l().iter().enumerate() {
        let Some(r) = r.of() else { continue };
        let age = if timed { ages.at(bin as i64).of().unwrap_or(0.)/1000. } else { 0. };
        let angle = (bin as f64).to_radians()-heading+yaw*age;
        let angle = angle.sin().atan2(angle.cos());
        if angle.abs() >= std::f64::consts::FRAC_PI_2 { continue; }
        let (dx,dy) = if yaw.abs() < 1e-6 { (speed*age*heading.cos(), -speed*age*heading.sin()) }
            else { (speed/yaw*(heading.sin()-(heading-yaw*age).sin()), speed/yaw*(heading.cos()-(heading-yaw*age).cos())) };
        points.push(Point { x:r*angle.cos()-dx, y:r*angle.sin()-dy, angle, bin, age });
    }
    let mut front: Vec<f64> = points.iter().filter(|p| p.angle.abs() < 3_f64.to_radians()).map(|p| p.x).collect();
    if front.is_empty() { return None; }
    front.sort_unstable_by(f64::total_cmp);
    let left = corner(&points, -1., front[front.len()/2])?;
    let right = corner(&points, 1., front[front.len()/2])?;
    let span = right.y-left.y;
    if left.x <= 0. || right.x <= 0. || left.y >= 0. || right.y <= 0. || (right.x-left.x).abs() > span { return None; }
    Some(EndBoard { range:(left.x*right.y-right.x*left.y)/span, left, right, timed })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bowed_board_edges_share_one_time_without_changing_raw_returns() {
        // A one-metre lane: the two edges are at 2 m, its bowed middle at
        // 2.2 m. The scanner takes 100 ms while the vehicle advances at 2 m/s.
        let mut ranges = Vec::new();
        let mut ages = Vec::new();
        for i in 0..360 {
            let angle = (i as f64).to_radians();
            let age = (360-i) as f64 / 3600.;
            let side = 0.5 / angle.sin().abs().max(1e-9);
            let end = if angle.cos() > 0. { (2.2+2.*age)/(angle.cos()+0.4*angle.sin().abs()) } else { 12. };
            ranges.push(side.min(end));
            ages.push(age*1000.);
        }
        let scan = dict! {"ranges" => ranges.clone(), "point_age_ms" => ages};
        let board = observe(&scan, 0., Some(2.), Some(0.)).unwrap();
        assert!((board.range-2.).abs() < 0.02);
        assert!((board.right.y-board.left.y-1.).abs() < 0.02);
        assert_eq!(scan.get("ranges").at(0).f(), ranges[0]);
    }

    #[test]
    fn recorded_start_and_finish_scans_select_the_two_edges() {
        let fixture = crate::py::parse_json(include_str!("../../web/vehicle-console/tests/fixtures/y106_bowed_end_boards.json")).unwrap();
        for frame in fixture.get("frames").l() {
            let rear = frame.get("rear").is_true();
            let heading = frame.get("heading_rad").f() + if rear { std::f64::consts::PI } else { 0. };
            let board = observe(frame.get("scan"), heading, None, None).unwrap();
            if rear { assert!((0.35..0.38).contains(&board.range)); }
            else if frame.get("scan").get("seq").i() == 285 {
                let old = crate::race_light::observed_end_wall(frame.get("scan"), heading).unwrap();
                assert!(old-board.range > 0.15);
            }
            assert!(board.left.y < -SIDE_BODY_EXTENT_M && board.right.y > SIDE_BODY_EXTENT_M);
        }
    }
}
