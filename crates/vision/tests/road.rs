use image::{Rgb, RgbImage};
use xt_stcar_robot_core::autonomy::{LightState, RoadObservation};
use xt_stcar_robot_core::{FrameId, Timestamp};
use xt_stcar_vision::Detection;
use xt_stcar_vision::road::{RoadConfig, RoadDetector, StripeAxis};

fn scene(width: u32, height: u32) -> RgbImage {
    RgbImage::from_pixel(width, height, Rgb([35, 35, 35]))
}
fn rect(image: &mut RgbImage, r: [f64; 4], color: [u8; 3]) {
    let (w, h) = image.dimensions();
    for (x, y, pixel) in image.enumerate_pixels_mut() {
        let (u, v) = (
            (f64::from(x) + 0.5) / f64::from(w),
            (f64::from(y) + 0.5) / f64::from(h),
        );
        if u >= r[0] && u < r[2] && v >= r[1] && v < r[3] {
            *pixel = Rgb(color);
        }
    }
}
fn circle(image: &mut RgbImage, cx: f64, cy: f64, radius: f64, color: [u8; 3]) {
    for (x, y, pixel) in image.enumerate_pixels_mut() {
        if (f64::from(x) + 0.5 - cx).hypot(f64::from(y) + 0.5 - cy) <= radius {
            *pixel = Rgb(color);
        }
    }
}
fn cone(image: &mut RgbImage, cx: f64, top: f64, bottom: f64, half_base: f64, color: [u8; 3]) {
    for (x, y, pixel) in image.enumerate_pixels_mut() {
        let (x, y) = (f64::from(x) + 0.5, f64::from(y) + 0.5);
        if y >= top && y < bottom && (x - cx).abs() <= half_base * (y - top) / (bottom - top) {
            *pixel = Rgb(color);
        }
    }
}
fn observe(config: RoadConfig, image: &RgbImage, detections: &[Detection]) -> RoadObservation {
    RoadDetector::new(config)
        .unwrap()
        .detect(
            image,
            detections,
            Timestamp(123),
            FrameId("base_link".into()),
        )
        .unwrap()
}
fn papers(width: u32, height: u32, count: usize) -> RgbImage {
    let mut image = scene(width, height);
    // Rules-sized papers: 0.297m forward, 0.105m lateral, same-size lateral gap.
    // The synthetic normalized homography is x=5*(1-v), y=2-4*u.
    for index in 0..count {
        let left = 0.25 + index as f64 * (0.105 + 0.105) / 4.;
        rect(
            &mut image,
            [left, 1. - 1.797 / 5., left + 0.105 / 4., 1. - 1.5 / 5.],
            [245, 245, 245],
        );
    }
    image
}

#[test]
fn side_by_side_rules_sized_papers_produce_metric_edges_and_survive_resizing() {
    for (width, height) in [(320, 240), (640, 480), (160, 120)] {
        let observation = observe(RoadConfig::simulation(), &papers(width, height, 8), &[]);
        let crossing = observation.crosswalk.unwrap_or_else(|| {
            panic!("side-by-side papers must be recognized at {width}x{height}")
        });
        assert!((crossing.near_edge_m - 1.5).abs() < 0.045);
        assert!((crossing.far_edge_m - 1.797).abs() < 0.045);
        assert!(crossing.lateral_max_m - crossing.lateral_min_m > 1.5);
        assert!(crossing.confidence > 0.8);
        assert_eq!(observation.captured_at, Timestamp(123));
        assert_eq!(observation.frame_id, FrameId("base_link".into()));
    }
}

#[test]
fn stripe_axis_is_explicit_and_both_pattern_directions_are_supported() {
    let image = papers(320, 240, 8);
    let mut config = RoadConfig::simulation();
    config.crosswalk.stripe_axis = StripeAxis::ImageY;
    assert!(observe(config.clone(), &image, &[]).crosswalk.is_some());
    config.crosswalk.stripe_axis = StripeAxis::ImageX;
    assert!(observe(config.clone(), &image, &[]).crosswalk.is_none());
    let mut horizontal = scene(320, 240);
    for i in 0..4 {
        let y = 0.45 + i as f64 * 0.07;
        rect(&mut horizontal, [0.25, y, 0.7, y + 0.025], [255, 255, 255]);
    }
    assert!(
        observe(config.clone(), &horizontal, &[])
            .crosswalk
            .is_some()
    );
    config.crosswalk.stripe_axis = StripeAxis::ImageY;
    assert!(observe(config, &horizontal, &[]).crosswalk.is_none());
}

#[test]
fn rotated_papers_obey_the_configured_orientation_tolerance() {
    let base = papers(320, 240, 8);
    for (degrees, expected) in [(-12_f64, true), (12., true), (35., false)] {
        let (sin, cos) = degrees.to_radians().sin_cos();
        let mut rotated = scene(320, 240);
        for (x, y, pixel) in rotated.enumerate_pixels_mut() {
            let (dx, dy) = (f64::from(x) + 0.5 - 160., f64::from(y) + 0.5 - 156.);
            let sx = (cos * dx + sin * dy + 160.).floor();
            let sy = (-sin * dx + cos * dy + 156.).floor();
            if (0.0..320.0).contains(&sx) && (0.0..240.0).contains(&sy) {
                *pixel = *base.get_pixel(sx as u32, sy as u32);
            }
        }
        let mut config = RoadConfig::simulation();
        config.crosswalk.stripe_axis = StripeAxis::ImageY;
        assert_eq!(
            observe(config, &rotated, &[]).crosswalk.is_some(),
            expected,
            "rotation {degrees}"
        );
    }
}

#[test]
fn isolated_white_blocks_parking_lines_and_irregular_gaps_are_not_crosswalks() {
    let config = RoadConfig::simulation();
    assert!(
        observe(config.clone(), &papers(320, 240, 3), &[])
            .crosswalk
            .is_none()
    );
    let mut sheet = scene(320, 240);
    rect(&mut sheet, [0.2, 0.5, 0.8, 0.8], [255, 255, 255]);
    assert!(observe(config.clone(), &sheet, &[]).crosswalk.is_none());
    let mut blocks = scene(320, 240);
    for i in 0..5 {
        let x = 0.2 + i as f64 * 0.12;
        rect(
            &mut blocks,
            [x, 0.6, x + 0.04, 0.6 + 0.04 * 320. / 240.],
            [255, 255, 255],
        );
    }
    assert!(observe(config.clone(), &blocks, &[]).crosswalk.is_none());
    let mut irregular = scene(320, 240);
    for x in [0.15, 0.22, 0.45, 0.72] {
        rect(&mut irregular, [x, 0.55, x + 0.025, 0.72], [255, 255, 255]);
    }
    assert!(observe(config, &irregular, &[]).crosswalk.is_none());
}

#[test]
fn hsv_lamps_report_three_colors_conflict_darkness_and_overexposure() {
    let mut config = RoadConfig::simulation();
    config.light_rois = vec![[0.7, 0.03, 0.95, 0.3]];
    for (rgb, expected) in [
        ([255, 0, 0], LightState::Red),
        ([255, 255, 0], LightState::Yellow),
        ([0, 255, 0], LightState::Green),
        ([5, 0, 0], LightState::Unknown),
        ([255, 255, 255], LightState::Unknown),
    ] {
        let mut image = scene(320, 240);
        circle(&mut image, 264., 36., 9., rgb);
        let observation = observe(config.clone(), &image, &[]);
        assert_eq!(observation.light, expected);
        if expected != LightState::Unknown {
            assert!(observation.light_confidence > 0.5);
        }
    }
    let mut image = scene(320, 240);
    circle(&mut image, 244., 36., 8., [255, 0, 0]);
    circle(&mut image, 280., 36., 8., [0, 255, 0]);
    assert_eq!(
        observe(config.clone(), &image, &[]).light,
        LightState::Conflicting
    );
    rect(&mut image, [0.7, 0.03, 0.95, 0.3], [255, 255, 255]);
    assert_eq!(
        observe(config.clone(), &image, &[]).light,
        LightState::Unknown
    );
    config.light_rois.clear();
    circle(&mut image, 264., 36., 9., [0, 255, 0]);
    assert_eq!(
        observe(config, &image, &[]).light,
        LightState::Unknown,
        "no fallback full-image lamp search"
    );
}

#[test]
fn yolo_class_nine_rois_take_priority_and_out_of_bounds_boxes_are_safe() {
    let mut image = scene(320, 240);
    circle(&mut image, 48., 36., 9., [255, 0, 0]);
    circle(&mut image, 264., 36., 9., [0, 255, 0]);
    let mut config = RoadConfig::simulation();
    config.light_rois = vec![[0.08, 0.05, 0.25, 0.25]];
    let valid = Detection {
        class_id: 9,
        confidence: 0.9,
        xyxy: [240., 15., 289., 58.],
    };
    assert_eq!(observe(config.clone(), &image, &[]).light, LightState::Red);
    assert_eq!(
        observe(config.clone(), &image, &[valid]).light,
        LightState::Green
    );
    let empty = Detection {
        class_id: 9,
        confidence: 0.9,
        xyxy: [5., 5., 5., 5.],
    };
    assert_eq!(
        observe(config, &image, &[empty]).light,
        LightState::Unknown,
        "an empty YOLO crop must not fall back to another lamp"
    );
    let config = RoadConfig::simulation();
    for bounds in [
        [-20., -20., 10., 10.],
        [350., 250., 400., 300.],
        [5., 5., 5., 5.],
        [f32::NAN, 0., 10., 10.],
    ] {
        let detection = Detection {
            class_id: 9,
            confidence: 0.9,
            xyxy: bounds,
        };
        assert_eq!(
            observe(config.clone(), &image, &[detection]).light,
            LightState::Unknown
        );
    }
}

#[test]
fn red_and_blue_cone_feet_project_to_body_and_lamp_regions_are_excluded() {
    let mut image = scene(320, 240);
    cone(&mut image, 80., 135., 180., 15., [255, 0, 0]);
    cone(&mut image, 224., 135., 180., 15., [0, 0, 255]);
    let config = RoadConfig::simulation();
    let result = observe(config.clone(), &image, &[]);
    assert_eq!(result.cones_body_m.len(), 2);
    for point in &result.cones_body_m {
        assert!((point.x_m - 1.25).abs() < 0.03);
    }
    assert!(
        result
            .cones_body_m
            .iter()
            .any(|p| (p.y_m - 1.).abs() < 0.03)
    );
    assert!(
        result
            .cones_body_m
            .iter()
            .any(|p| (p.y_m + 0.8).abs() < 0.03)
    );
    let light = Detection {
        class_id: 9,
        confidence: 0.9,
        xyxy: [60., 125., 100., 185.],
    };
    let result = observe(config, &image, &[light]);
    assert_eq!(result.cones_body_m.len(), 1);
    assert!(result.cones_body_m[0].y_m < 0.);
    let mut rectangles = scene(320, 240);
    rect(&mut rectangles, [0.3, 0.5, 0.35, 0.8], [255, 0, 0]);
    assert!(
        observe(RoadConfig::simulation(), &rectangles, &[])
            .cones_body_m
            .is_empty(),
        "untapered colored signs are not cones"
    );
}

#[test]
fn invalid_rois_singular_horizons_mirrors_and_excessive_projection_are_rejected() {
    let mut config = RoadConfig::simulation();
    config.light_rois = vec![[-0.1, 0.1, 0.2, 0.3]];
    assert!(RoadDetector::new(config).is_err());
    let mut config = RoadConfig::simulation();
    config.homography.matrix[1] = config.homography.matrix[0];
    assert!(
        RoadDetector::new(config)
            .err()
            .unwrap()
            .contains("singular")
    );
    let mut config = RoadConfig::simulation();
    config.homography.matrix[2] = [0., 1., -0.6];
    assert!(RoadDetector::new(config).err().unwrap().contains("horizon"));
    let mut config = RoadConfig::simulation();
    config.homography.matrix[1] = [4., 0., -2.];
    assert!(
        RoadDetector::new(config)
            .err()
            .unwrap()
            .contains("consistently")
    );
    let mut config = RoadConfig::simulation();
    config.homography.max_forward_m = 2.;
    assert!(
        RoadDetector::new(config)
            .err()
            .unwrap()
            .contains("envelope")
    );
    let mut config = RoadConfig::simulation();
    config.simulation_only = false;
    assert!(RoadDetector::new(config).is_err());
}

#[test]
fn extreme_images_bad_frames_and_excessive_components_fail_with_bounded_work() {
    let detector = RoadDetector::new(RoadConfig::simulation()).unwrap();
    assert!(
        detector
            .detect(&scene(0, 0), &[], Timestamp(0), FrameId("base_link".into()))
            .is_err()
    );
    assert!(
        detector
            .detect(&scene(320, 240), &[], Timestamp(0), FrameId("".into()))
            .is_err()
    );
    let mut config = RoadConfig::simulation();
    config.max_components = 2;
    let mut image = scene(64, 64);
    for x in [12, 24, 36, 48] {
        image.put_pixel(x, 40, Rgb([255, 255, 255]));
    }
    assert!(
        RoadDetector::new(config)
            .unwrap()
            .detect(&image, &[], Timestamp(0), FrameId("base_link".into()))
            .unwrap_err()
            .contains("component limit")
    );
}
