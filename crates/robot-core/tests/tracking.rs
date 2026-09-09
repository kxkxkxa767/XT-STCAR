use std::f64::consts::{FRAC_PI_2, PI};
use xt_stcar_robot_core::autonomy::{Point2, Pose2};
use xt_stcar_robot_core::tracking::{
    LqrTracker, MAX_PATH_POINTS, PathTracker, PurePursuitTracker, TrackInput, TrackingConfig,
};

fn point(x_m: f64, y_m: f64) -> Point2 {
    Point2 { x_m, y_m }
}

fn pose(x_m: f64, y_m: f64, yaw_rad: f64) -> Pose2 {
    Pose2 { x_m, y_m, yaw_rad }
}

fn input(path: &[Point2]) -> TrackInput<'_> {
    TrackInput {
        pose: Pose2::default(),
        speed_mps: 0.3,
        path,
        progress: 0,
        lookahead_m: 0.55,
        max_curvature_per_m: 2.0,
    }
}

fn lqr_config() -> TrackingConfig {
    TrackingConfig::Lqr {
        q_lateral: 4.0,
        q_heading: 2.0,
        r_curvature: 1.0,
        min_speed_mps: 0.02,
        max_heading_error_rad: 0.5,
        max_lateral_error_m: 0.5,
    }
}

fn lqr() -> LqrTracker {
    LqrTracker::new(lqr_config()).unwrap()
}

fn near(actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() < 1e-10,
        "actual {actual}, expected {expected}"
    );
}

#[test]
fn pp_geometry_matches_known_left_and_right_targets() {
    for sign in [-1.0, 1.0] {
        let path = [point(0.0, 0.0), point(1.0, sign), point(2.0, sign)];
        let mut data = input(&path);
        data.lookahead_m = 2.0_f64.sqrt();
        let result = PurePursuitTracker.track(data).unwrap();
        assert_eq!(result.target, point(1.0, sign));
        near(result.curvature_per_m, sign);
    }
}

#[test]
fn pp_lookahead_crosses_segments_and_uses_pose_before_remaining_route() {
    let path = [
        point(-5.0, 4.0),
        point(-1.0, 2.0),
        point(1.0, 0.0),
        point(1.0, 1.0),
    ];
    let mut data = input(&path);
    data.progress = 1;
    data.lookahead_m = 1.5;
    let result = PurePursuitTracker.track(data).unwrap();
    assert_eq!(result.target, point(1.0, 0.5));
    near(result.curvature_per_m, 0.8);
    data.lookahead_m = 5.0;
    assert_eq!(
        PurePursuitTracker.track(data).unwrap().target,
        point(1.0, 1.0)
    );
}

#[test]
fn pp_is_rigid_transform_invariant_and_clamps_curvature() {
    let path = [point(3.0, -2.0), point(2.0, -1.0)];
    let mut data = input(&path);
    data.pose = pose(3.0, -2.0, FRAC_PI_2);
    data.lookahead_m = 2.0_f64.sqrt();
    near(PurePursuitTracker.track(data).unwrap().curvature_per_m, 1.0);
    data.max_curvature_per_m = 0.4;
    near(PurePursuitTracker.track(data).unwrap().curvature_per_m, 0.4);
}

#[test]
fn pp_matches_old_formula_over_progress_pose_and_lookahead_variations() {
    // Independent retained formula also checks floating-point operation order,
    // including a repeated route vertex and the final-vertex case.
    let route = [
        point(0.0, 0.0),
        point(0.4, 0.05),
        point(0.4, 0.05),
        point(0.8, 0.15),
        point(1.5, 0.2),
    ];
    for progress in 0..route.len() {
        for lookahead in [0.1, 0.55, 2.0] {
            let mut data = input(&route);
            data.pose = pose(route[progress].x_m - 0.05, route[progress].y_m - 0.01, 0.0);
            data.progress = progress;
            data.lookahead_m = lookahead;
            let mut chain = vec![data.pose.point()];
            chain.extend_from_slice(&route[(progress + 1).min(route.len() - 1)..]);
            let mut remaining = lookahead;
            let mut expected = *chain.last().unwrap();
            for pair in chain.windows(2) {
                let length = pair[0].distance(pair[1]);
                if length > remaining {
                    let ratio = remaining / length;
                    expected = point(
                        pair[0].x_m + ratio * (pair[1].x_m - pair[0].x_m),
                        pair[0].y_m + ratio * (pair[1].y_m - pair[0].y_m),
                    );
                    break;
                }
                remaining -= length;
            }
            let body = data.pose.world_to_body(expected);
            let expected_curvature =
                (2.0 * body.y_m / (body.x_m.powi(2) + body.y_m.powi(2))).clamp(-2.0, 2.0);
            let actual = PurePursuitTracker.track(data).unwrap();
            assert_eq!(actual.target, expected);
            assert_eq!(actual.curvature_per_m, expected_curvature);
        }
    }
}

#[test]
fn lqr_straight_reference_zero_error_has_zero_curvature() {
    let path = [point(0.0, 0.0), point(1.0, 0.0), point(2.0, 0.0)];
    let mut data = input(&path);
    data.pose = pose(0.25, 0.0, 0.0);
    near(lqr().track(data).unwrap().curvature_per_m, 0.0);
}

#[test]
fn lqr_lateral_and_heading_feedback_have_restoring_signs_and_gains() {
    let path = [point(0.0, 0.0), point(1.0, 0.0), point(2.0, 0.0)];
    for sign in [-1.0, 1.0] {
        let mut data = input(&path);
        data.pose = pose(0.25, sign * 0.1, 0.0);
        near(lqr().track(data).unwrap().curvature_per_m, -sign * 0.2);
        data.pose = pose(0.25, 0.0, sign * 0.1);
        near(
            lqr().track(data).unwrap().curvature_per_m,
            -sign * 0.1 * 6.0_f64.sqrt(),
        );
    }
}

#[test]
fn lqr_circle_vertices_recover_curvature_and_tangent_without_pose_in_path() {
    let radius = 2.0;
    for sign in [-1.0, 1.0] {
        let path: Vec<_> = (0..20)
            .map(|i| {
                let angle = i as f64 * 0.04;
                point(radius * angle.sin(), sign * radius * (1.0 - angle.cos()))
            })
            .collect();
        for progress in [0, 4, 10] {
            let mut data = input(&path);
            data.progress = progress;
            data.pose = pose(
                path[progress].x_m,
                path[progress].y_m,
                sign * progress as f64 * 0.04,
            );
            near(lqr().track(data).unwrap().curvature_per_m, sign / radius);
        }
    }
}

#[test]
fn lqr_mirror_symmetry_and_yaw_wrapping() {
    let path = [
        point(0.0, 0.0),
        point(0.8, 0.08),
        point(1.6, 0.3),
        point(2.5, 0.6),
    ];
    let mirrored: Vec<_> = path.iter().map(|p| point(p.x_m, -p.y_m)).collect();
    let mut a = input(&path);
    a.pose = pose(0.3, 0.04, 0.1);
    let mut b = input(&mirrored);
    b.pose = pose(a.pose.x_m, -a.pose.y_m, -a.pose.yaw_rad + 2.0 * PI);
    near(
        lqr().track(a).unwrap().curvature_per_m,
        -lqr().track(b).unwrap().curvature_per_m,
    );
}

#[test]
fn lqr_projects_only_incident_segments_at_a_path_crossing() {
    let path = [
        point(-1.0, 0.0),
        point(1.0, 0.0),
        point(2.0, 1.0),
        point(2.0, 2.0),
        point(0.0, 2.0),
        point(0.0, -1.0),
    ];
    let mut data = input(&path);
    data.pose = pose(0.0, 0.1, 0.0);
    data.progress = 0;
    // The later vertical crossing is closer, but may not change the reference.
    let local = [path[0], path[1], path[2]];
    let expected = lqr()
        .track(TrackInput {
            path: &local,
            ..data
        })
        .unwrap();
    let actual = lqr().track(data).unwrap();
    assert_eq!(actual.curvature_per_m, expected.curvature_per_m);
}

#[test]
fn lqr_repeated_vertices_do_not_create_nan_or_hide_actual_geometry() {
    let path = [
        point(0.0, 0.0),
        point(1.0, 0.0),
        point(1.0, 0.0),
        point(1.0, 0.0),
        point(2.0, 0.0),
        point(3.0, 0.0),
    ];
    for progress in 1..=3 {
        let mut data = input(&path);
        data.pose = pose(1.0, 0.1, 0.0);
        data.progress = progress;
        near(lqr().track(data).unwrap().curvature_per_m, -0.2);
    }
}

#[test]
fn lqr_low_speed_falls_back_to_pp_after_validation() {
    let path = [point(0.0, 0.0), point(1.0, 0.5), point(2.0, 0.5)];
    for speed in [0.0, 0.01] {
        let mut data = input(&path);
        data.speed_mps = speed;
        assert_eq!(
            lqr().track(data).unwrap(),
            PurePursuitTracker.track(data).unwrap()
        );
        data.path = &[];
        assert!(lqr().track(data).is_err());
    }
}

#[test]
fn lqr_rejects_large_errors_reversals_and_corners() {
    let path = [point(0.0, 0.0), point(1.0, 0.0), point(2.0, 0.0)];
    for actual in [pose(0.2, 0.6, 0.0), pose(0.2, 0.0, 0.6)] {
        assert!(
            lqr()
                .track(TrackInput {
                    pose: actual,
                    ..input(&path)
                })
                .is_err()
        );
    }
    for path in [
        [point(0.0, 0.0), point(1.0, 0.0), point(0.0, 0.0)],
        [point(0.0, 0.0), point(1.0, 0.0), point(1.0, 1.0)],
    ] {
        assert!(lqr().track(input(&path)).is_err());
    }
}

#[test]
fn trackers_reject_nonfinite_unbounded_reverse_and_degenerate_inputs() {
    let good = [point(0.0, 0.0), point(1.0, 0.0)];
    let repeated = [point(1.0, 0.0); 2];
    let invalid = [point(0.0, 0.0), point(f64::NAN, 0.0)];
    let huge = vec![point(0.0, 0.0); MAX_PATH_POINTS + 1];
    let backward = [point(0.0, 0.0), point(-1.0, 0.0)];
    let coincident = [point(-1.0, 0.0), point(0.0, 0.0)];
    let outside = [point(0.0, 0.0), point(1_000_001.0, 0.0)];
    let bad = [
        TrackInput {
            path: &[],
            ..input(&good)
        },
        TrackInput {
            path: &good[..1],
            ..input(&good)
        },
        input(&repeated),
        input(&invalid),
        input(&huge),
        input(&backward),
        input(&coincident),
        input(&outside),
        TrackInput {
            progress: usize::MAX,
            ..input(&good)
        },
        TrackInput {
            pose: pose(0.0, 0.0, f64::NAN),
            ..input(&good)
        },
        TrackInput {
            speed_mps: -0.1,
            ..input(&good)
        },
        TrackInput {
            speed_mps: f64::NAN,
            ..input(&good)
        },
        TrackInput {
            speed_mps: 3.1,
            ..input(&good)
        },
        TrackInput {
            lookahead_m: 0.0,
            ..input(&good)
        },
        TrackInput {
            lookahead_m: f64::INFINITY,
            ..input(&good)
        },
        TrackInput {
            max_curvature_per_m: f64::NAN,
            ..input(&good)
        },
        TrackInput {
            max_curvature_per_m: 0.0,
            ..input(&good)
        },
    ];
    for data in bad {
        assert!(
            PurePursuitTracker.track(data).is_err(),
            "PP accepted {data:?}"
        );
        assert!(lqr().track(data).is_err(), "LQR accepted {data:?}");
    }
}

#[test]
fn lqr_configuration_and_gain_overflow_are_rejected() {
    assert!(LqrTracker::new(TrackingConfig::PurePursuit).is_err());
    for (q_lateral, q_heading, r_curvature) in [
        (0.0, 1.0, 1.0),
        (-1.0, 1.0, 1.0),
        (1.0, f64::NAN, 1.0),
        (1.0, 1.0, 0.0),
        (f64::MAX, 1.0, f64::MIN_POSITIVE),
        (f64::MIN_POSITIVE, 1.0, f64::MAX),
    ] {
        let config = TrackingConfig::Lqr {
            q_lateral,
            q_heading,
            r_curvature,
            min_speed_mps: 0.02,
            max_heading_error_rad: 0.5,
            max_lateral_error_m: 0.5,
        };
        assert!(config.validate().is_err());
    }
    for (min_speed_mps, max_heading_error_rad, max_lateral_error_m) in [
        (0.0, 0.5, 0.5),
        (0.02, 0.8, 0.5),
        (0.02, 0.5, 3.0),
        (f64::NAN, 0.5, 0.5),
    ] {
        let config = TrackingConfig::Lqr {
            q_lateral: 4.0,
            q_heading: 2.0,
            r_curvature: 1.0,
            min_speed_mps,
            max_heading_error_rad,
            max_lateral_error_m,
        };
        assert!(config.validate().is_err());
    }
}

#[test]
fn configuration_serialization_is_explicit_and_strict() {
    assert_eq!(TrackingConfig::default(), TrackingConfig::PurePursuit);
    assert_eq!(
        serde_json::to_value(TrackingConfig::default()).unwrap(),
        serde_json::json!({"kind": "pure_pursuit"})
    );
    let config: TrackingConfig =
        serde_json::from_value(serde_json::to_value(lqr_config()).unwrap()).unwrap();
    assert_eq!(config, lqr_config());
    assert!(
        serde_json::from_value::<TrackingConfig>(
            serde_json::json!({"kind":"pure_pursuit", "q_lateral":1.0})
        )
        .is_err()
    );
    assert!(serde_json::from_value::<TrackingConfig>(serde_json::json!({"kind":"lqr"})).is_err());
}
