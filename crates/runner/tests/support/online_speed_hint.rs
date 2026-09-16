use super::*;
use crate::simulation::{SimulationConfig, synthetic_scan};
use xt_stcar_robot_core::autonomy::{HalfPlane, Point2, PoseEstimate};
use xt_stcar_robot_core::local_world::{
    ElementColor, ElementGeometry, ElementObservation, ObservationSource,
};
use xt_stcar_robot_core::mission::MissionReport;

fn capture(
    index: usize,
) -> (
    AutonomyConfig,
    OnlineSession,
    PoseEstimate,
    LidarSample,
    OnlineMissionReport,
) {
    let saved: serde_json::Value =
        serde_json::from_str(include_str!("../fixtures/online_orbit_speed_hint.json")).unwrap();
    let simulation: SimulationConfig =
        serde_json::from_value(saved["simulation_config"].clone()).unwrap();
    let raw = &saved["captures"][index];
    let at = Timestamp(raw["at"].as_u64().unwrap());
    let pose = PoseEstimate {
        captured_at: at,
        frame_id: simulation.autonomy.navigation.frame_id.clone(),
        pose: serde_json::from_value(raw["pose"].clone()).unwrap(),
        speed_mps: raw["actual_speed_mps"].as_f64().unwrap(),
        yaw_rate_radps: raw["actual_speed_mps"].as_f64().unwrap()
            * raw["curvature_per_m"].as_f64().unwrap(),
        quality: 1.0,
    };
    // Simulation truth is used here only to reconstruct the recorded sensor.
    // The hint sees LocalWorld, a measured pose and the observed task report.
    let scan = synthetic_scan(&simulation, pose.pose, &simulation.cones, at);
    let config = simulation.autonomy;
    let mut session = OnlineSession::new(config.online.clone().unwrap(), &config).unwrap();
    session
        .world
        .update_scan(at, &pose, &scan, config.lidar_in_body)
        .unwrap();
    let track = &raw["online"]["active_track"];
    let center: Point2 = serde_json::from_value(track["position"].clone()).unwrap();
    let marker = ElementObservation {
        kind: ElementKind::Cone,
        color: ElementColor::Red,
        position_body_m: pose.pose.world_to_body(center),
        heading_body_rad: None,
        geometry: ElementGeometry::Cone {
            radius_m: track["geometry"]["radius_m"].as_f64().unwrap(),
        },
        source: ObservationSource::VisualLidar,
        confidence: 0.9,
        position_error_m: 0.06,
        heading_error_rad: 0.0,
    };
    for stamp in [Timestamp(at.0 - 100), at] {
        let mut observed_pose = pose.clone();
        observed_pose.captured_at = stamp;
        session
            .world
            .update(
                stamp,
                &observed_pose,
                &ElementFrame {
                    captured_at: stamp,
                    frame_id: config.mission.body_frame.clone(),
                    observations: vec![marker],
                },
            )
            .unwrap();
    }
    let current = session.world.tracks(at).next().unwrap();
    let boundary = &raw["online"]["travel_boundary"];
    let normal: Point2 = serde_json::from_value(boundary["normal"].clone()).unwrap();
    let report = OnlineMissionReport {
        mission: MissionReport {
            at,
            phase: MissionPhase::Cones,
            output: MissionOutput::Target {
                point: serde_json::from_value(raw["online"]["mission"]["output"]["point"].clone())
                    .unwrap(),
                max_speed_mps: raw["online"]["mission"]["output"]["max_speed_mps"]
                    .as_f64()
                    .unwrap(),
                arrival: ArrivalBehavior::Stop,
            },
            reason: raw["online"]["mission"]["reason"].as_str().unwrap().into(),
            crosswalk_stop_elapsed_ms: 3000,
            green_elapsed_ms: 0,
            waypoint_index: 0,
        },
        goal_heading_rad: raw["online"]["goal_heading_rad"].as_f64(),
        // Keep a nonempty permission-plane sentinel. The speed hint does not
        // inspect or modify boundaries; finite-side admission is tested by
        // the existing full source-boundary certificate regressions.
        travel_boundary: Some(
            HalfPlane::new(
                serde_json::from_value(boundary["origin"].clone()).unwrap(),
                normal.y_m.atan2(normal.x_m),
                boundary["max_projection_m"].as_f64().unwrap(),
            )
            .unwrap(),
        ),
        behavior: OnlineBehavior::OrbitCone,
        active_track_id: Some(current.id),
        active_track: Some(current),
        processed_cones: 0,
        cone_progress_rad: raw["online"]["cone_progress_rad"].as_f64().unwrap(),
        search_distance_m: 0.0,
        execution_space_rejected: false,
    };
    (config, session, pose, scan, report)
}

fn target_speed(report: &OnlineMissionReport) -> f64 {
    let MissionOutput::Target { max_speed_mps, .. } = report.mission.output else {
        panic!("hint must not invent Stop or completion")
    };
    max_speed_mps
}

#[test]
fn orbit_hint_recorded_26500_and_26600_reduce_before_the_mid_arc_bottleneck() {
    for index in 0..2 {
        let (config, session, pose, _, mut report) = capture(index);
        let original = serde_json::to_value(&config).unwrap();
        let hint = &session.orbit_speed_hint;
        let probes = hint
            .probes(pose.captured_at, pose.pose, &session.world, &report)
            .unwrap();
        assert_eq!(probes[0], pose.pose);
        assert_eq!(probes.len(), 6);
        let upper = target_speed(&report);
        // A goal-only proof misses the constriction in these real captures.
        assert_eq!(
            hint.speed_cap(pose.captured_at, &session.world, &[probes[5]; 6], upper),
            Some(upper)
        );
        let heading = report.goal_heading_rad.unwrap();
        if let MissionOutput::Target { point, arrival, .. } = &mut report.mission.output {
            *arrival = ArrivalBehavior::PassThrough {
                next: Point2 {
                    x_m: point.x_m + heading.cos() * 0.8,
                    y_m: point.y_m + heading.sin() * 0.8,
                },
                next_heading_rad: Some(heading),
                next_max_speed_mps: config.navigation.max_speed_mps,
                admission_radius_m: config.online.as_ref().unwrap().mission.goal_tolerance_m,
            };
        }
        let before = report.clone();
        hint.apply(pose.captured_at, &pose, &session.world, &mut report);
        let cap = target_speed(&report);
        assert!(cap > 0.12 && cap < 0.16, "capture {index}: {cap}");
        assert!(cap < upper);
        assert!(
            cap >= pose.speed_mps
                - config.navigation.max_decel_mps2 * config.navigation.control_period_ms as f64
                    / 1000.
        );
        assert_eq!(report.cone_progress_rad, before.cone_progress_rad);
        assert_eq!(report.processed_cones, 0);
        assert_eq!(report.goal_heading_rad, before.goal_heading_rad);
        assert_eq!(report.travel_boundary, before.travel_boundary);
        let MissionOutput::Target {
            arrival:
                ArrivalBehavior::PassThrough {
                    next_max_speed_mps, ..
                },
            ..
        } = report.mission.output
        else {
            panic!("continuation was lost")
        };
        assert_eq!(next_max_speed_mps, cap);
        for probe in probes {
            let envelope = StoppingEnvelope::new(
                &config.navigation,
                probe,
                cap,
                config.navigation.max_curvature_per_m,
                0.35,
            )
            .unwrap();
            assert!(session.world.known_free_convex_hull(
                pose.captured_at,
                &envelope.corners().map(|p| probe.body_to_world(p)),
                0.
            ));
        }
        assert_eq!(serde_json::to_value(&config).unwrap(), original);
        if index == 1 {
            // The hint cannot replace the measured .18 m/s in the actual
            // source certificate. Its original 26600 ms rejection survives.
            let k = pose.yaw_rate_radps / pose.speed_mps;
            assert!(!session.permits_command(
                pose.captured_at,
                &pose,
                &pose,
                &MotionOutput::Drive {
                    speed_mps: cap,
                    curvature_per_m: k
                },
                &config,
                None,
                SteeringEstimate {
                    at: pose.captured_at,
                    applied_curvature_per_m: k,
                    commanded_curvature_per_m: k
                }
            ));
        }
    }
}

#[test]
fn orbit_hint_open_space_preserves_exact_upper_and_never_raises_a_lower_continuation() {
    let (config, mut session, mut pose, mut scan, mut report) = capture(0);
    scan.ranges_m.fill(Some(10.));
    pose.captured_at.0 += 100;
    scan.captured_at = pose.captured_at;
    report.mission.at = pose.captured_at;
    session
        .world
        .update_scan(pose.captured_at, &pose, &scan, config.lidar_in_body)
        .unwrap();
    let hint = &session.orbit_speed_hint;
    let target = if let MissionOutput::Target { point, .. } = report.mission.output {
        point
    } else {
        unreachable!()
    };
    let probes = [
        pose.pose,
        Pose2 {
            x_m: target.x_m,
            y_m: target.y_m,
            yaw_rad: report.goal_heading_rad.unwrap(),
        },
        pose.pose,
        pose.pose,
        pose.pose,
        pose.pose,
    ];
    for upper in [0.01, 0.18, 0.3] {
        assert_eq!(
            hint.speed_cap(pose.captured_at, &session.world, &probes, upper),
            Some(upper)
        );
    }
    // Invalid/expired evidence is absence of a hint, not a zero target or new
    // permission. The original world/mission/execution gates still own Stop.
    for upper in [0., -0.1, 0.31, f64::NAN, f64::INFINITY] {
        assert_eq!(
            hint.speed_cap(pose.captured_at, &session.world, &probes, upper),
            None
        );
    }
    assert_eq!(
        hint.speed_cap(
            Timestamp(pose.captured_at.0 + session.world.config().scan_ttl_ms),
            &session.world,
            &probes,
            0.18
        ),
        None
    );
    let before = serde_json::to_value(&report).unwrap();
    assert!(
        hint.probes(pose.captured_at, pose.pose, &session.world, &report)
            .is_some()
    );
    hint.apply(pose.captured_at, &pose, &session.world, &mut report);
    assert_eq!(serde_json::to_value(&report).unwrap(), before);
    if let MissionOutput::Target { point, arrival, .. } = &mut report.mission.output {
        *arrival = ArrivalBehavior::PassThrough {
            next: Point2 {
                x_m: point.x_m + 0.5,
                y_m: point.y_m,
            },
            next_heading_rad: Some(0.0),
            next_max_speed_mps: 0.09,
            admission_radius_m: 0.065,
        };
    }
    let before = serde_json::to_value(&report).unwrap();
    hint.apply(pose.captured_at, &pose, &session.world, &mut report);
    assert_eq!(serde_json::to_value(&report).unwrap(), before);
}

#[test]
fn orbit_hint_unknown_is_absent_and_cannot_relax_the_actual_execution_gate() {
    let (config, mut session, mut pose, mut scan, mut report) = capture(1);
    let probes = session
        .orbit_speed_hint
        .probes(pose.captured_at, pose.pose, &session.world, &report)
        .unwrap();
    let upper = target_speed(&report);
    scan.ranges_m[0] = None;
    pose.captured_at.0 += 100;
    scan.captured_at = pose.captured_at;
    report.mission.at = pose.captured_at;
    session
        .world
        .update_scan(pose.captured_at, &pose, &scan, config.lidar_in_body)
        .unwrap();
    assert_eq!(
        session
            .orbit_speed_hint
            .speed_cap(pose.captured_at, &session.world, &probes, upper),
        None
    );
    let before = serde_json::to_value(&report).unwrap();
    assert!(
        session
            .orbit_speed_hint
            .probes(pose.captured_at, pose.pose, &session.world, &report)
            .is_some()
    );
    session
        .orbit_speed_hint
        .apply(pose.captured_at, &pose, &session.world, &mut report);
    assert_eq!(serde_json::to_value(&report).unwrap(), before);
    assert!(!session.permits_command(
        pose.captured_at,
        &pose,
        &pose,
        &MotionOutput::Drive {
            speed_mps: 0.01,
            curvature_per_m: 0.
        },
        &config,
        None,
        SteeringEstimate {
            at: pose.captured_at,
            applied_curvature_per_m: 0.,
            commanded_curvature_per_m: 0.
        }
    ));
}

#[test]
fn orbit_hint_is_limited_to_fresh_matching_circular_cone_targets() {
    for mutation in 0..7 {
        let (_, session, pose, _, mut report) = capture(0);
        match mutation {
            0 => report.behavior = OnlineBehavior::ConeEntry,
            1 => report.mission.phase = MissionPhase::ApproachLight,
            2 => report.active_track_id = None,
            3 => report.goal_heading_rad = None,
            4 => report.goal_heading_rad = Some(report.goal_heading_rad.unwrap() + 0.5),
            5 => report.processed_cones = 2,
            _ => report.mission.output = MissionOutput::Stop,
        }
        let before = serde_json::to_value(&report).unwrap();
        session
            .orbit_speed_hint
            .apply(pose.captured_at, &pose, &session.world, &mut report);
        assert_eq!(serde_json::to_value(&report).unwrap(), before);
    }
}
