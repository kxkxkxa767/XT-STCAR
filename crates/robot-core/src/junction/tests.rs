use super::*;

fn sample(ranges: Vec<Option<f64>>) -> LidarSample {
    LidarSample {
        captured_at: crate::Timestamp(100),
        frame_id: FrameId("heading".into()),
        angle_min_rad: 0.0,
        angle_increment_rad: TAU / 360.0,
        range_min_m: 0.02,
        range_max_m: 12.0,
        ranges_m: ranges,
    }
}

fn polygon_scan(poly: &[(f64, f64)], yaw: f64) -> LidarSample {
    let cross = |a: (f64, f64), b: (f64, f64)| a.0 * b.1 - a.1 * b.0;
    let ranges = (0..360)
        .map(|i| {
            let angle = i as f64 * TAU / 360.0 + yaw;
            let direction = (angle.cos(), angle.sin());
            let mut closest: f64 = 12.0;
            for (a, b) in poly
                .iter()
                .zip(poly.iter().cycle().skip(1))
                .take(poly.len())
            {
                let edge = (b.0 - a.0, b.1 - a.1);
                let denominator = cross(direction, edge);
                if denominator.abs() < 1e-8 {
                    continue;
                }
                let t = cross(*a, edge) / denominator;
                let u = cross(*a, direction) / denominator;
                if t > 0.0 && (0.0..=1.0).contains(&u) {
                    closest = closest.min(t);
                }
            }
            Some(closest)
        })
        .collect();
    sample(ranges)
}

fn left(width: f64, end: f64, outgoing: f64, yaw: f64) -> LidarSample {
    let front = end + outgoing;
    polygon_scan(
        &[
            (-8., -width / 2.),
            (front, -width / 2.),
            (front, 8.),
            (end, 8.),
            (end, width / 2.),
            (-8., width / 2.),
        ],
        yaw,
    )
}

#[test]
fn left_opening_across_widths_and_heading_variation() {
    for width in [0.9, 1.0, 1.5, 2.0] {
        for outgoing in [1., 1.5, 2.] {
            for yaw in [-8., 0., 8.] {
                let scan = left(width, 0.25, outgoing, f64::to_radians(yaw));
                let result = detect_left_junction(&scan, &scan.frame_id)
                    .unwrap()
                    .unwrap_or_else(|| panic!("width={width}, outgoing={outgoing}, yaw={yaw}"));
                assert!((result.incoming_width_m - width).abs() < 0.06, "{result:?}");
                assert!(
                    (result.outgoing_width_m - outgoing).abs() < 0.1,
                    "{result:?}"
                );
                assert!(!result.turn_path_certified);
            }
        }
    }
}

#[test]
fn actual_observed_scan_requires_no_yolo_or_map_label() {
    let ranges = serde_json::from_str(include_str!("observed-left-20261005.json")).unwrap();
    let mut scan = sample(ranges);
    scan.angle_increment_rad = -TAU / 360.; // Console bins are clockwise.
    let result = detect_left_junction(&scan, &scan.frame_id)
        .unwrap()
        .unwrap();
    assert!((1.7..2.0).contains(&result.front_wall_m), "{result:?}");
    assert!(
        (-0.1..0.5).contains(&result.incoming_left_end_m),
        "{result:?}"
    );
}

#[test]
fn straight_dead_end_right_turn_and_t_do_not_claim_left() {
    let cases = [
        vec![(-8., -0.5), (8., -0.5), (8., 0.5), (-8., 0.5)],
        vec![(-8., -0.5), (1.8, -0.5), (1.8, 0.5), (-8., 0.5)],
        vec![
            (-8., -0.5),
            (0.25, -0.5),
            (0.25, -8.),
            (1.8, -8.),
            (1.8, 0.5),
            (-8., 0.5),
        ],
        vec![
            (-8., -0.5),
            (0.25, -0.5),
            (0.25, -8.),
            (1.8, -8.),
            (1.8, 8.),
            (0.25, 8.),
            (0.25, 0.5),
            (-8., 0.5),
        ],
    ];
    for poly in cases {
        let scan = polygon_scan(&poly, 0.);
        assert!(
            detect_left_junction(&scan, &scan.frame_id)
                .unwrap()
                .is_none()
        );
    }
}

#[test]
fn missing_opening_returns_are_unknown_and_format_errors_rejected() {
    let mut scan = left(1., 0.25, 1.5, 0.);
    for angle in 30..66 {
        scan.ranges_m[angle] = None;
    }
    assert!(detect_left_junction(&scan, &scan.frame_id).is_err());
    let scan = left(1., 0.25, 1.5, 0.);
    assert!(detect_left_junction(&scan, &FrameId("wrong".into())).is_err());
    let mut scan = scan;
    scan.ranges_m[0] = Some(f64::NAN);
    assert!(detect_left_junction(&scan, &scan.frame_id).is_err());
}

#[test]
fn isolated_close_object_does_not_replace_extended_walls() {
    let mut scan = sample(vec![Some(5.); 360]);
    for angle in 82..98 {
        scan.ranges_m[angle] = Some(0.6);
    }
    assert!(
        detect_left_junction(&scan, &scan.frame_id)
            .unwrap()
            .is_none()
    );
}

#[test]
fn front_boundary_slows_before_turn_identity_without_claiming_left() {
    for distance in [2., 3., 4.] {
        for yaw in [-8., 0., 8.] {
            let scan = polygon_scan(
                &[(-8., -0.5), (distance, -0.5), (distance, 0.5), (-8., 0.5)],
                f64::to_radians(yaw),
            );
            let front = detect_front_boundary(&scan, &scan.frame_id)
                .unwrap()
                .unwrap();
            assert!((front - distance).abs() < 0.02);
            assert!(
                detect_left_junction(&scan, &scan.frame_id)
                    .unwrap()
                    .is_none()
            );
        }
    }
    let scan = polygon_scan(&[(-8., -0.5), (8., -0.5), (8., 0.5), (-8., 0.5)], 0.);
    assert!(
        detect_front_boundary(&scan, &scan.frame_id)
            .unwrap()
            .is_none()
    );
}

#[test]
fn known_front_plane_replays_observed_junction_and_preserves_unknowns() {
    let mut scan =
        sample(serde_json::from_str(include_str!("observed-left-20261005.json")).unwrap());
    scan.angle_increment_rad = -TAU / 360.;
    let front = detect_front_boundary(&scan, &scan.frame_id)
        .unwrap()
        .unwrap();
    assert!((1.7..2.).contains(&front));
    for index in 0..36 {
        scan.ranges_m[index] = None;
    }
    assert!(detect_front_boundary(&scan, &scan.frame_id).is_err());
}
