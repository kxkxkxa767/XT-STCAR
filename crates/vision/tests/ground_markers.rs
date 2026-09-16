use image::{Rgb, RgbImage};
use xt_stcar_robot_core::autonomy::Point2;
use xt_stcar_robot_core::local_world::{
    ElementColor, ElementGeometry, ElementKind, ObservationSource,
};
use xt_stcar_robot_core::{FrameId, Timestamp};
use xt_stcar_vision::Detection;
use xt_stcar_vision::ground_markers::{
    ExperimentalMarkerConfig, ExperimentalMarkerPattern, GroundMarkerDetector,
};
use xt_stcar_vision::road::{RoadConfig, RoadDetector};

fn blank(w: u32, h: u32) -> RgbImage {
    RgbImage::from_pixel(w, h, Rgb([35, 35, 35]))
}
fn frame() -> FrameId {
    FrameId("base_link".into())
}
fn rectangle(image: &mut RgbImage, center: Point2, heading: f64, depth: f64, span: f64) {
    let (w, h) = image.dimensions();
    let (s, c) = heading.sin_cos();
    for (x, y, p) in image.enumerate_pixels_mut() {
        // Inverse of the declared synthetic homography. Truth is used only
        // to render pixels; the detector is passed no scene coordinates.
        let dx = 5. * (1. - (f64::from(y) + 0.5) / f64::from(h)) - center.x_m;
        let dy = 2. - 4. * (f64::from(x) + 0.5) / f64::from(w) - center.y_m;
        if (c * dx + s * dy).abs() < depth / 2. && (-s * dx + c * dy).abs() < span / 2. {
            *p = Rgb([245, 245, 245]);
        }
    }
}
fn marker(
    image: &mut RgbImage,
    p: &ExperimentalMarkerPattern,
    far: Point2,
    heading: f64,
    only: Option<usize>,
) {
    for i in 0..p.bar_count {
        if only.is_some_and(|n| n != i) {
            continue;
        }
        let offset = -p.anchor_from_far_edge_m
            + (i as f64 - (p.bar_count - 1) as f64 / 2.) * p.bar_spacing_m;
        rectangle(
            image,
            Point2 {
                x_m: far.x_m + heading.cos() * offset,
                y_m: far.y_m + heading.sin() * offset,
            },
            heading,
            p.bar_thickness_m,
            p.bar_span_m,
        );
    }
}
fn angle_error(a: f64, b: f64) -> f64 {
    ((a - b + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI).abs()
}

#[test]
fn explicit_far_anchor_patterns_reconstruct_near_edge_and_measured_rotated_heading() {
    let config = ExperimentalMarkerConfig::simulation();
    let detector = GroundMarkerDetector::new(RoadConfig::simulation(), config.clone()).unwrap();
    for (w, h) in [(160, 120), (320, 240), (640, 480)] {
        for heading in [0_f64, 0.35, -0.55, 2.4] {
            for p in &config.patterns {
                let far = Point2 { x_m: 1.8, y_m: 0. };
                let mut image = blank(w, h);
                marker(&mut image, p, far, heading, None);
                let observed = detector
                    .detect(&image, &[], Timestamp(81), frame(), heading)
                    .unwrap();
                assert_eq!(
                    observed.observations.len(),
                    1,
                    "{w}x{h}, {heading}, {:?}: {observed:?}",
                    p.kind
                );
                let o = observed.observations[0];
                let truth = Point2 {
                    x_m: far.x_m - heading.cos() * p.region_depth_m,
                    y_m: far.y_m - heading.sin() * p.region_depth_m,
                };
                assert_eq!(o.kind, p.kind);
                assert_eq!(o.source, ObservationSource::GroundMarker);
                assert_eq!(observed.captured_at, Timestamp(81));
                assert!(
                    o.position_body_m.distance(truth) <= o.position_error_m,
                    "{o:?}, truth {truth:?}"
                );
                assert!(
                    angle_error(o.heading_body_rad.unwrap(), heading) <= o.heading_error_rad,
                    "{o:?} expected {heading}"
                );
                let ElementGeometry::LineRegion {
                    lateral_half_width_m,
                    depth_m,
                } = o.geometry
                else {
                    panic!()
                };
                assert_eq!(depth_m, p.region_depth_m);
                assert_eq!(lateral_half_width_m, p.region_width_m / 2.);
            }
        }
    }
}

#[test]
fn disabled_missing_and_incomplete_patterns_never_make_a_semantic_landmark() {
    let config = ExperimentalMarkerConfig::simulation();
    let p = &config.patterns[1];
    let far = Point2 { x_m: 1.8, y_m: 0. };
    let mut full = blank(320, 240);
    marker(&mut full, p, far, 0., None);
    let disabled = GroundMarkerDetector::new(
        RoadConfig::simulation(),
        ExperimentalMarkerConfig::default(),
    )
    .unwrap();
    assert!(
        disabled
            .detect(&full, &[], Timestamp(0), frame(), 0.)
            .unwrap()
            .observations
            .is_empty()
    );
    let detector = GroundMarkerDetector::new(RoadConfig::simulation(), config.clone()).unwrap();
    assert_eq!(
        detector
            .detect(&full, &[], Timestamp(0), frame(), 0.)
            .unwrap()
            .observations
            .len(),
        1
    );
    for only in [0, 1] {
        let mut partial = blank(320, 240);
        marker(&mut partial, p, far, 0., Some(only));
        assert!(
            detector
                .detect(&partial, &[], Timestamp(1), frame(), 0.)
                .unwrap()
                .observations
                .is_empty()
        );
    }
    // The detector has no stale state to synthesize an occluded last sighting.
    for t in [100, 1900, 10000] {
        assert!(
            detector
                .detect(&blank(320, 240), &[], Timestamp(t), frame(), 0.)
                .unwrap()
                .observations
                .is_empty()
        );
    }
    let mut clipped = blank(320, 240);
    marker(&mut clipped, p, Point2 { x_m: 0.1, y_m: 0. }, 0., None);
    assert!(
        detector
            .detect(&clipped, &[], Timestamp(2), frame(), 0.)
            .unwrap()
            .observations
            .is_empty()
    );
}

#[test]
fn protocol_region_width_is_separate_from_unchanged_visual_bar_geometry_and_errors() {
    let config = ExperimentalMarkerConfig::simulation();
    assert_eq!(config.patterns[0].bar_span_m, 0.6);
    assert_eq!(config.patterns[0].region_width_m, 0.8);
    let mut image = blank(320, 240);
    marker(
        &mut image,
        &config.patterns[0],
        Point2 { x_m: 1.8, y_m: 0.0 },
        0.0,
        None,
    );
    let first = GroundMarkerDetector::new(RoadConfig::simulation(), config.clone())
        .unwrap()
        .detect(&image, &[], Timestamp(1), frame(), 0.0)
        .unwrap()
        .observations[0];
    let mut wider = config;
    wider.patterns[0].region_width_m = 1.0;
    let second = GroundMarkerDetector::new(RoadConfig::simulation(), wider)
        .unwrap()
        .detect(&image, &[], Timestamp(1), frame(), 0.0)
        .unwrap()
        .observations[0];
    assert_eq!(first.position_body_m, second.position_body_m);
    assert_eq!(first.heading_body_rad, second.heading_body_rad);
    assert_eq!(first.position_error_m, second.position_error_m);
    assert_eq!(first.heading_error_rad, second.heading_error_rad);
    assert_eq!(
        first.geometry,
        ElementGeometry::LineRegion {
            lateral_half_width_m: 0.4,
            depth_m: 1.0
        }
    );
    assert_eq!(
        second.geometry,
        ElementGeometry::LineRegion {
            lateral_half_width_m: 0.5,
            depth_m: 1.0
        }
    );
    let mut invalid = ExperimentalMarkerConfig::simulation();
    invalid.patterns[0].region_width_m = f64::NAN;
    assert!(invalid.validate().is_err());
    invalid.patterns[0].region_width_m = 0.59;
    assert!(invalid.validate().is_err());
}

#[test]
fn ambiguous_protocol_and_nonfinite_direction_are_rejected_and_light_boxes_are_excluded() {
    let mut c = ExperimentalMarkerConfig::simulation();
    c.patterns[1].bar_span_m = c.patterns[0].bar_span_m;
    assert!(c.validate().is_err());
    c = ExperimentalMarkerConfig::simulation();
    c.patterns[0].region_depth_m = f64::NAN;
    assert!(c.validate().is_err());
    c = ExperimentalMarkerConfig::simulation();
    c.simulation_only = false;
    assert!(c.validate().is_err());
    c = ExperimentalMarkerConfig::simulation();
    c.patterns[0].kind = ElementKind::Cone;
    assert!(c.validate().is_err());
    let c = ExperimentalMarkerConfig::simulation();
    let mut image = blank(320, 240);
    marker(
        &mut image,
        &c.patterns[0],
        Point2 { x_m: 1.8, y_m: 0. },
        0.,
        None,
    );
    let detector = GroundMarkerDetector::new(RoadConfig::simulation(), c).unwrap();
    assert!(
        detector
            .detect(&image, &[], Timestamp(0), frame(), f64::NAN)
            .is_err()
    );
    let lamp = Detection {
        class_id: 9,
        confidence: 0.9,
        xyxy: [0., 0., 320., 240.],
    };
    assert!(
        detector
            .detect(&image, &[lamp], Timestamp(0), frame(), 0.)
            .unwrap()
            .observations
            .is_empty()
    );
    let mut colored = image.clone();
    for p in colored.pixels_mut() {
        if p[0] > 200 {
            *p = Rgb([255, 0, 0]);
        }
    }
    assert!(
        detector
            .detect(&colored, &[], Timestamp(0), frame(), 0.)
            .unwrap()
            .observations
            .is_empty()
    );
}

#[test]
fn crossing_heading_is_recovered_from_paper_geometry_not_assumed_body_zero() {
    let detector = RoadDetector::new(RoadConfig::simulation()).unwrap();
    for heading in [-0.5_f64, 0., 0.5] {
        let mut image = blank(320, 240);
        for i in 0..8 {
            let y = (i as f64 - 3.5) * 0.21;
            rectangle(
                &mut image,
                Point2 {
                    x_m: 1.7 - heading.sin() * y,
                    y_m: heading.cos() * y,
                },
                heading,
                0.297,
                0.105,
            );
        }
        let seen = detector
            .detect_elements(&image, &[], Timestamp(22), frame(), heading)
            .unwrap();
        let crosswalk = seen
            .observations
            .iter()
            .find(|o| o.kind == ElementKind::Crosswalk)
            .unwrap_or_else(|| panic!("{heading}: {seen:?}"));
        assert!(
            angle_error(crosswalk.heading_body_rad.unwrap(), heading)
                <= crosswalk.heading_error_rad
        );
        if heading != 0. {
            assert!(crosswalk.heading_body_rad.unwrap().abs() > 0.3);
        }
        let near = Point2 {
            x_m: 1.7 - heading.cos() * 0.297 / 2.,
            y_m: -heading.sin() * 0.297 / 2.,
        };
        assert!(
            crosswalk.position_body_m.distance(near) <= crosswalk.position_error_m,
            "{crosswalk:?} expected {near:?}"
        );
        let reverse = detector
            .detect_elements(
                &image,
                &[],
                Timestamp(22),
                frame(),
                heading + std::f64::consts::PI,
            )
            .unwrap();
        let opposite = reverse
            .observations
            .iter()
            .find(|o| o.kind == ElementKind::Crosswalk)
            .unwrap();
        assert!(
            angle_error(
                opposite.heading_body_rad.unwrap(),
                heading + std::f64::consts::PI
            ) <= opposite.heading_error_rad
        );
    }
}

#[test]
fn cone_colors_survive_ground_projection_without_changing_legacy_detect() {
    let detector = RoadDetector::new(RoadConfig::simulation()).unwrap();
    let mut image = blank(320, 240);
    for (cx, rgb) in [(110., [255, 0, 0]), (210., [0, 0, 255])] {
        for (x, y, p) in image.enumerate_pixels_mut() {
            let x = f64::from(x) + 0.5;
            let y = f64::from(y) + 0.5;
            if (125.0..185.0).contains(&y) && (x - cx).abs() < 18. * (y - 125.) / 60. {
                *p = Rgb(rgb);
            }
        }
    }
    let before = detector.detect(&image, &[], Timestamp(0), frame()).unwrap();
    let elements = detector
        .detect_elements(&image, &[], Timestamp(0), frame(), 0.)
        .unwrap();
    assert_eq!(elements.observations.len(), 2);
    assert_eq!(before.cones_body_m.len(), 2);
    for color in [ElementColor::Red, ElementColor::Blue] {
        let o = elements
            .observations
            .iter()
            .find(|o| o.color == color)
            .unwrap();
        assert_eq!(o.kind, ElementKind::Cone);
        assert_eq!(o.source, ObservationSource::GroundProjection);
        assert!(o.heading_body_rad.is_none());
        assert!(o.position_error_m > 0.);
        assert!(before.cones_body_m.contains(&o.position_body_m));
    }
    let after = detector.detect(&image, &[], Timestamp(0), frame()).unwrap();
    assert_eq!(
        serde_json::to_value(before).unwrap(),
        serde_json::to_value(after).unwrap()
    );
}
