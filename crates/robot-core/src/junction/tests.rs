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
    polygon_scan_bins(poly, yaw, 360)
}

fn polygon_scan_bins(poly: &[(f64, f64)], yaw: f64, bins: usize) -> LidarSample {
    let cross = |a: (f64, f64), b: (f64, f64)| a.0 * b.1 - a.1 * b.0;
    let ranges = (0..bins)
        .map(|i| {
            let angle = i as f64 * TAU / bins as f64 + yaw;
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
    let mut scan = sample(ranges);
    scan.angle_increment_rad = TAU / bins as f64;
    scan
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

fn assert_current_incoming_end_support(scan: &LidarSample, goal: &LeftTurnGoal) {
    let junction = detect_left_junction(scan, &scan.frame_id).unwrap().unwrap();
    let support = &goal.incoming_left_end_support;
    assert_eq!(scan.ranges_m[support.index], Some(support.range_m));
    let angle = (scan.angle_min_rad + support.index as f64 * scan.angle_increment_rad + PI)
        .rem_euclid(TAU)
        - PI;
    assert!((support.angle_left_rad - angle).abs() < 1e-12);
    assert!((support.point_left_m.x_m - support.range_m * angle.cos()).abs() < 1e-12);
    assert!((support.point_left_m.y_m - support.range_m * angle.sin()).abs() < 1e-12);
    let (s, c) = goal.incoming_heading_left_rad.sin_cos();
    let x = c * support.point_left_m.x_m + s * support.point_left_m.y_m;
    assert!((x - goal.incoming_left_end_m).abs() <= 1e-6);
    assert_eq!(goal.incoming_left_end_m, junction.incoming_left_end_m);
    assert_eq!(goal.known_open_fraction, junction.known_open_fraction);
    assert!((0.9..=1.0).contains(&goal.known_open_fraction));
    assert!(goal.candidate_only && !goal.turn_path_certified);
}

#[test]
fn corridor_candidates_follow_arbitrary_observed_axes_with_real_wall_support() {
    for yaw_deg in [-130.0_f64, -87.0, -42.0, -7.0, 0.0, 38.0, 94.0, 157.0] {
        let yaw = yaw_deg.to_radians();
        let scan = polygon_scan(
            &[(-8., -0.355), (8., -0.355), (8., 0.595), (-8., 0.595)],
            yaw,
        );
        let candidates = detect_corridor_candidates(&scan, &scan.frame_id).unwrap();
        let angle_error = |a: f64, b: f64| ((a - b + PI).rem_euclid(TAU) - PI).abs();
        let forward = candidates
            .iter()
            .find(|candidate| angle_error(candidate.heading_left_rad, -yaw) < 0.01)
            .unwrap_or_else(|| panic!("axis={yaw_deg}, candidates={candidates:?}"));
        let reverse = candidates
            .iter()
            .find(|candidate| angle_error(candidate.heading_left_rad, -yaw + PI) < 0.01)
            .unwrap();
        assert!((forward.width_m - 0.95).abs() < 0.01, "{forward:?}");
        assert!(
            (forward.center_offset_left_m - 0.12).abs() < 0.01,
            "{forward:?}"
        );
        assert!(
            (reverse.center_offset_left_m + 0.12).abs() < 0.01,
            "{reverse:?}"
        );
        for candidate in [forward, reverse] {
            assert!(candidate.left_wall_points >= 16 && candidate.right_wall_points >= 16);
            assert!(candidate.support_span_m >= 0.35 && candidate.fit_error_m <= 0.04);
            assert!(candidate.origin_between_walls && candidate.candidate_only);
            assert!(!candidate.turn_path_certified);
        }
    }
}

#[test]
fn corridor_candidates_accept_supported_taper_but_reject_divergent_wall_pairs() {
    // The observed portable boards need not be exactly parallel. Each side is
    // still a real straight segment, and their shared local support remains
    // inside the origin-straddling width. Rotation must not change acceptance.
    for difference_deg in [8.0_f64, 11.5, 13.0] {
        let taper = (difference_deg * 0.5).to_radians().tan();
        for yaw_deg in [-83.0_f64, -22.0, 0.0, 47.0, 121.0] {
            let yaw = yaw_deg.to_radians();
            let scan = polygon_scan(
                &[
                    (-4.0, -0.55 + taper * 4.0),
                    (4.0, -0.55 - taper * 4.0),
                    (4.0, 0.65 + taper * 4.0),
                    (-4.0, 0.65 - taper * 4.0),
                ],
                yaw,
            );
            let candidates = detect_corridor_candidates(&scan, &scan.frame_id).unwrap();
            if difference_deg > 12.0 {
                assert!(
                    candidates.is_empty(),
                    "{difference_deg}, {yaw_deg}: {candidates:?}"
                );
                continue;
            }
            assert_eq!(
                candidates.len(),
                2,
                "{difference_deg}, {yaw_deg}: {candidates:?}"
            );
            let candidate = candidates
                .iter()
                .find(|c| ((c.heading_left_rad + yaw + PI).rem_euclid(TAU) - PI).abs() < 0.02)
                .unwrap_or_else(|| panic!("{difference_deg}, {yaw_deg}: {candidates:?}"));
            assert!((candidate.width_m - 1.2).abs() < 0.01, "{candidate:?}");
            assert!(
                (candidate.center_offset_left_m - 0.05).abs() < 0.01,
                "{candidate:?}"
            );
            assert!(candidate.left_wall_points >= 16 && candidate.right_wall_points >= 16);
            assert!(candidate.support_span_m >= 0.35 && candidate.fit_error_m < 0.001);
            assert!(candidate.origin_between_walls && candidate.candidate_only);
            assert!(!candidate.turn_path_certified);
        }
    }
}

#[test]
fn corridor_candidates_keep_clockwise_scan_axes_and_unknown_returns_explicit() {
    let mut scan = polygon_scan(&[(-8., -0.5), (8., -0.5), (8., 0.5), (-8., 0.5)], 0.7);
    let original = scan.ranges_m.clone();
    scan.ranges_m = (0..360).map(|i| original[(360 - i) % 360]).collect();
    scan.angle_increment_rad = -TAU / 360.;
    let candidates = detect_corridor_candidates(&scan, &scan.frame_id).unwrap();
    assert!(
        candidates
            .iter()
            .any(|candidate| (candidate.heading_left_rad + 0.7).abs() < 0.01)
    );
    scan.ranges_m[4] = None;
    scan.ranges_m[5] = None;
    assert!(
        !detect_corridor_candidates(&scan, &scan.frame_id)
            .unwrap()
            .is_empty()
    );
    assert!(scan.ranges_m[4].is_none() && scan.ranges_m[5].is_none());
    for r in &mut scan.ranges_m[0..30] {
        *r = None;
    }
    assert!(detect_corridor_candidates(&scan, &scan.frame_id).is_err());
}

#[test]
fn corridor_candidates_do_not_invent_planes_or_an_unentered_outgoing_branch() {
    for scan in [
        sample(vec![Some(1.0); 360]),
        polygon_scan(
            &[(-0.15, -0.5), (0.15, -0.5), (0.15, 0.5), (-0.15, 0.5)],
            0.,
        ),
    ] {
        assert!(
            detect_corridor_candidates(&scan, &scan.frame_id)
                .unwrap()
                .is_empty()
        );
    }
    let scan = left(0.95, 0.25, 1., 0.);
    let candidates = detect_corridor_candidates(&scan, &scan.frame_id).unwrap();
    // The left opening is observable, but both outgoing side walls are still
    // ahead of the origin. The candidates do not fabricate a traversable 90deg corridor.
    assert!(
        detect_left_junction(&scan, &scan.frame_id)
            .unwrap()
            .is_some()
    );
    assert!(
        !candidates
            .iter()
            .any(|candidate| (candidate.heading_left_rad - PI / 2.).abs() < 0.1)
    );
}

#[test]
fn left_alignment_goal_uses_a_measured_outer_tangent_and_known_target_ray() {
    let tilt = 0.05;
    let scan = polygon_scan(
        &[
            (-8.0, -0.5),
            (1.75 - tilt * 0.5, -0.5),
            (1.75 + tilt * 8.0, 8.0),
            (0.25 + tilt * 8.0, 8.0),
            (0.25 + tilt * 0.5, 0.5),
            (-8.0, 0.5),
        ],
        0.0,
    );
    let observed = detect_turn_geometry(&scan, &scan.frame_id).unwrap();
    let goal = observed.left_goal.unwrap();
    assert_current_incoming_end_support(&scan, &goal);
    assert!(!goal.origin_between_exit_walls);
    assert!(goal.candidate_only && !goal.turn_path_certified);
    assert!((goal.heading_left_rad - 1.0_f64.atan2(tilt)).abs() < 0.01);
    assert!((goal.heading_left_rad - PI / 2.0).abs() > 0.02);
    assert!(goal.outer_wall.points >= 16 && goal.outer_wall.support_span_m >= 0.35);
    let ray = &goal.target_support_ray;
    assert_eq!(scan.ranges_m[ray.index], Some(ray.range_m));
    assert!(ray.target_range_m > 0.0 && ray.target_range_m + 0.03 < ray.range_m);
    assert!(
        (goal.target_point_left_m.x_m - ray.target_range_m * ray.angle_left_rad.cos()).abs()
            < 1e-12
    );
    assert!(
        (goal.target_point_left_m.y_m - ray.target_range_m * ray.angle_left_rad.sin()).abs()
            < 1e-12
    );
    let n = (-goal.heading_left_rad.sin(), goal.heading_left_rad.cos());
    assert!(
        (n.0 * goal.target_point_left_m.x_m + n.1 * goal.target_point_left_m.y_m
            - goal.center_offset_left_m)
            .abs()
            < 1e-12
    );
    assert!(observed.walls.iter().any(|wall| {
        (wall.heading_left_rad - goal.outer_wall.heading_left_rad).abs() < 0.04
            && (wall.rho_left_m - goal.outer_wall.rho_left_m).abs() < 0.05
    }));
}

#[test]
fn incoming_end_support_follows_real_rotated_and_clockwise_rays() {
    for (width, end, outgoing) in [(0.95, 0.2, 1.2), (1.3, 0.55, 1.7)] {
        for yaw_deg in [-8.0_f64, -3.0, 0.0, 7.0] {
            for clockwise in [false, true] {
                let mut scan = left(width, end, outgoing, yaw_deg.to_radians());
                if clockwise {
                    let original = scan.ranges_m.clone();
                    scan.ranges_m = (0..360).map(|i| original[(360 - i) % 360]).collect();
                    scan.angle_increment_rad = -TAU / 360.0;
                }
                let goal = detect_turn_geometry(&scan, &scan.frame_id)
                    .unwrap()
                    .left_goal
                    .unwrap_or_else(|| {
                        panic!("{width}, {end}, {outgoing}, {yaw_deg}, {clockwise}")
                    });
                assert_current_incoming_end_support(&scan, &goal);
                let serialized = serde_json::to_value(&goal).unwrap();
                let support = serialized["incoming_left_end_support"].as_object().unwrap();
                assert_eq!(support.len(), 4);
                assert!(support.contains_key("index") && support.contains_key("point_left_m"));
                assert_eq!(serialized["known_open_fraction"], goal.known_open_fraction);
            }
        }
    }
}

#[test]
fn incoming_end_support_keeps_small_board_angle_variation_observed() {
    for slope_deg in [-2.0_f64, 2.0] {
        let slope = slope_deg.to_radians().tan();
        let (width, end, front) = (1.1, 0.4, 1.9);
        let corner_y = width / 2.0 + slope * end;
        let tilt = 0.04;
        let poly = [
            (-8.0, -width / 2.0),
            (front - tilt * width / 2.0, -width / 2.0),
            (front + tilt * 8.0, 8.0),
            (end + tilt * (8.0 - corner_y), 8.0),
            (end, corner_y),
            (-8.0, width / 2.0 - slope * 8.0),
        ];
        for yaw_deg in [-6.0_f64, 0.0, 6.0] {
            let scan = polygon_scan(&poly, yaw_deg.to_radians());
            let goal = detect_turn_geometry(&scan, &scan.frame_id)
                .unwrap()
                .left_goal
                .unwrap_or_else(|| panic!("slope={slope_deg}, yaw={yaw_deg}"));
            assert_current_incoming_end_support(&scan, &goal);
            let a = (slope.atan() - yaw_deg.to_radians()).tan();
            let b =
                (width / 2.0) / (yaw_deg.to_radians().cos() + slope * yaw_deg.to_radians().sin());
            assert!(
                (goal.incoming_left_end_support.point_left_m.y_m
                    - a * goal.incoming_left_end_support.point_left_m.x_m
                    - b)
                    .abs()
                    <= 0.05
            );
        }
    }
}

#[test]
fn missing_incoming_endpoint_ray_is_not_interpolated_or_reused() {
    let mut scan = left(1.1, 0.4, 1.5, 0.0);
    let before = detect_turn_geometry(&scan, &scan.frame_id)
        .unwrap()
        .left_goal
        .unwrap();
    let missing = before.incoming_left_end_support.index;
    scan.ranges_m[missing] = None;
    let observed = detect_turn_geometry(&scan, &scan.frame_id).unwrap();
    if let Some(goal) = observed.left_goal {
        assert_current_incoming_end_support(&scan, &goal);
        assert_ne!(goal.incoming_left_end_support.index, missing);
        assert_ne!(
            goal.incoming_left_end_support.point_left_m,
            before.incoming_left_end_support.point_left_m
        );
        assert!(goal.incoming_left_end_m < before.incoming_left_end_m);
    }
    assert!(scan.ranges_m[missing].is_none());
}

#[test]
fn finite_wall_candidates_rotate_and_do_not_require_the_origin_inside_a_future_exit() {
    for yaw in [-1.7_f64, -0.3, 0.0, 0.8, 2.4] {
        let scan = polygon_scan(&[(-8.0, -0.5), (8.0, -0.5), (8.0, 0.5), (-8.0, 0.5)], yaw);
        let observed = detect_turn_geometry(&scan, &scan.frame_id).unwrap();
        assert!(observed.left_goal.is_none());
        let heading_error = |h: f64| ((h + yaw + PI).rem_euclid(TAU) - PI).abs();
        assert!(
            observed
                .walls
                .iter()
                .any(|wall| heading_error(wall.heading_left_rad) < 0.01
                    && (wall.rho_left_m.abs() - 0.5).abs() < 0.01)
        );
        for wall in observed.walls {
            assert!(wall.candidate_only && wall.points >= 16 && wall.support_span_m >= 0.35);
            for point in [wall.support_start_left_m, wall.support_end_left_m] {
                assert!(
                    ((-wall.heading_left_rad.sin()) * point.x_m
                        + wall.heading_left_rad.cos() * point.y_m
                        - wall.rho_left_m)
                        .abs()
                        <= wall.fit_error_m + 1e-9
                );
            }
        }
    }
}

#[test]
fn missing_opening_or_invalid_scan_cannot_create_a_left_alignment_goal() {
    let straight = polygon_scan(&[(-8.0, -0.5), (2.0, -0.5), (2.0, 0.5), (-8.0, 0.5)], 0.0);
    assert!(
        detect_turn_geometry(&straight, &straight.frame_id)
            .unwrap()
            .left_goal
            .is_none()
    );
    let mut opening = left(0.95, 0.5, 1.5, 0.0);
    for r in &mut opening.ranges_m[25..85] {
        *r = None;
    }
    assert!(detect_turn_geometry(&opening, &opening.frame_id).is_err());
    assert!(
        detect_turn_geometry(&sample(vec![Some(1.0); 360]), &FrameId("heading".into()))
            .unwrap()
            .left_goal
            .is_none()
    );
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
    let goal = detect_turn_geometry(&scan, &scan.frame_id)
        .unwrap()
        .left_goal
        .unwrap();
    assert_current_incoming_end_support(&scan, &goal);
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
    for distance in [2., 3., 4., 5., 6.] {
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
fn supported_distant_front_planes_use_the_declared_sensor_range() {
    for (width, distance) in [(1.5, 8.), (2., 10.), (2.1, 11.9)] {
        for yaw in [-8., 0., 8.] {
            let scan = polygon_scan(
                &[
                    (-8., -width / 2.),
                    (distance, -width / 2.),
                    (distance, width / 2.),
                    (-8., width / 2.),
                ],
                f64::to_radians(yaw),
            );
            let front = detect_front_boundary(&scan, &scan.frame_id)
                .unwrap()
                .unwrap_or_else(|| panic!("width={width}, distance={distance}, yaw={yaw}"));
            assert!((front - distance).abs() < 0.02, "{front}");
            assert!(
                detect_left_junction(&scan, &scan.frame_id)
                    .unwrap()
                    .is_none()
            );
        }
    }
}

#[test]
fn distant_narrow_plane_requires_enough_real_samples() {
    let poly = [(-8., -0.475), (8., -0.475), (8., 0.475), (-8., 0.475)];
    let coarse = polygon_scan(&poly, 0.);
    assert!(
        detect_front_boundary(&coarse, &coarse.frame_id)
            .unwrap()
            .is_none()
    );
    // Greater angular resolution supplies real ray intersections; the detector
    // does not interpolate the coarse scan's missing support into a plane.
    let fine = polygon_scan_bins(&poly, 0., 1440);
    let front = detect_front_boundary(&fine, &fine.frame_id)
        .unwrap()
        .unwrap();
    assert!((front - 8.).abs() < 0.02);
}

#[test]
fn observed_straight_scan_exposes_a_supported_plane_before_the_old_four_metre_limit() {
    // Continuous run 08, seq 571 / published at_ms 57095: the old detector
    // returned None despite the visible, supported front plane. This fixture
    // records only the existing coarse heading/range-corrected bins, not a stopping
    // distance or a vehicle-speed calibration.
    let mut scan =
        sample(serde_json::from_str(include_str!("observed-front-20261005.json")).unwrap());
    scan.angle_increment_rad = -TAU / 360.;
    let front = detect_front_boundary(&scan, &scan.frame_id)
        .unwrap()
        .unwrap();
    assert!((5.3..5.8).contains(&front), "{front}");
    assert!(
        detect_left_junction(&scan, &scan.frame_id)
            .unwrap()
            .is_none()
    );
}

#[test]
fn distant_front_plane_keeps_original_support_and_unknown_gates() {
    let poly = [(-8., -0.75), (8., -0.75), (8., 0.75), (-8., 0.75)];
    let scan = polygon_scan(&poly, 0.);
    let mut sparse = scan.clone();
    for index in [4, 356] {
        sparse.ranges_m[index] = None;
    }
    assert!(
        detect_front_boundary(&sparse, &sparse.frame_id)
            .unwrap()
            .is_none()
    );
    let mut no_plane = scan.clone();
    for index in [0, 1, 2, 3, 4, 356, 357, 358, 359] {
        no_plane.ranges_m[index] = None;
    }
    assert!(detect_front_boundary(&no_plane, &no_plane.frame_id).is_err());

    let mut outside_sensor_range = scan;
    outside_sensor_range.range_max_m = 7.9;
    assert!(detect_front_boundary(&outside_sensor_range, &outside_sensor_range.frame_id).is_err());
}

#[test]
fn known_distant_returns_outside_the_corridor_are_not_a_front_plane() {
    let poly = [(-8., -0.475), (8., -0.475), (8., 0.475), (-8., 0.475)];
    let mut scan = polygon_scan(&poly, 0.);
    for angle in 7..=20 {
        // All inserted known hits have x=8 m but lie outside both fitted walls.
        for index in [angle, 360 - angle] {
            scan.ranges_m[index] = Some(8. / (angle as f64).to_radians().cos());
        }
    }
    assert!(
        detect_front_boundary(&scan, &scan.frame_id)
            .unwrap()
            .is_none()
    );
}

#[test]
fn short_or_isolated_distant_returns_do_not_form_a_front_plane() {
    let poly = [(-8., -0.75), (8., -0.75), (8., 0.75), (-8., 0.75)];
    let mut short = polygon_scan_bins(&poly, 0., 1440);
    // Retain nine central real returns (<35 cm span) and disperse the others
    // into distinct range bands, without changing either extended side wall.
    for index in 5..=21 {
        for beam in [index, 1440 - index] {
            short.ranges_m[beam] = Some(5. + index as f64 * 0.15);
        }
    }
    assert!(
        detect_front_boundary(&short, &short.frame_id)
            .unwrap()
            .is_none()
    );
    let mut isolated = sample(vec![Some(8.); 360]);
    for angle in 82..98 {
        isolated.ranges_m[angle] = Some(0.6);
    }
    assert!(
        detect_front_boundary(&isolated, &isolated.frame_id)
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
