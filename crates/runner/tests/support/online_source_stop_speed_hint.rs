use super::*;
use serde_json::{Value, from_value};
use xt_stcar_robot_core::admission::AdoptionConstraints;
use xt_stcar_robot_core::autonomy::Point2;
use xt_stcar_robot_core::mission::MissionReport;

fn track(value: &Value) -> ElementTrack {
    macro_rules! read {
        ($field:ident) => {
            from_value(value[stringify!($field)].clone()).unwrap()
        };
    }
    ElementTrack {
        id: read!(id),
        kind: read!(kind),
        color: read!(color),
        position: read!(position),
        heading_rad: read!(heading_rad),
        geometry: read!(geometry),
        confidence: read!(confidence),
        position_error_m: read!(position_error_m),
        heading_error_rad: read!(heading_error_rad),
        first_seen: read!(first_seen),
        last_seen: read!(last_seen),
        last_visual_at: read!(last_visual_at),
        last_geometry_at: read!(last_geometry_at),
        observations: read!(observations),
        processed: read!(processed),
        valid: read!(valid),
    }
}
fn recorded(
    index: usize,
) -> (
    AutonomyConfig,
    OnlineSession,
    PoseEstimate,
    SteeringEstimate,
    OnlineMissionReport,
) {
    let saved: Value = serde_json::from_str(include_str!(
        "../fixtures/online_source_stop_speed_hint.json"
    ))
    .unwrap();
    let config: AutonomyConfig = from_value(saved["autonomy"].clone()).unwrap();
    let value = &saved["captures"][index];
    let pose: PoseEstimate = from_value(value["source"].clone()).unwrap();
    let at = pose.captured_at;
    let mut session = OnlineSession::new(config.online.clone().unwrap(), &config).unwrap();
    let raw = &value["world"]["scan"];
    let scan = LidarSample {
        captured_at: from_value(raw["at"].clone()).unwrap(),
        frame_id: config.scan.frame_id.clone(),
        angle_min_rad: raw["angle_min_rad"].as_f64().unwrap(),
        angle_increment_rad: raw["angle_increment_rad"].as_f64().unwrap(),
        range_min_m: raw["range_min_m"].as_f64().unwrap(),
        range_max_m: raw["range_max_m"].as_f64().unwrap(),
        ranges_m: from_value(raw["ranges_m"].clone()).unwrap(),
    };
    session
        .world
        .update_scan(at, &pose, &scan, config.lidar_in_body)
        .unwrap();
    session.confirmed_stop_line = Some((
        track(&value["confirmed"]["track"]),
        from_value(value["confirmed"]["checked_at"].clone()).unwrap(),
    ));
    let online = &value["online"];
    let active = track(&online["active_track"]);
    let report = OnlineMissionReport {
        mission: MissionReport {
            at,
            phase: MissionPhase::Cones,
            output: MissionOutput::Target {
                point: from_value(online["mission"]["output"]["point"].clone()).unwrap(),
                max_speed_mps: online["mission"]["output"]["max_speed_mps"]
                    .as_f64()
                    .unwrap(),
                arrival: ArrivalBehavior::Stop,
            },
            reason: online["mission"]["reason"].as_str().unwrap().into(),
            crosswalk_stop_elapsed_ms: 3000,
            green_elapsed_ms: 0,
            waypoint_index: 0,
        },
        goal_heading_rad: online["goal_heading_rad"].as_f64(),
        travel_boundary: observed_region_boundary(session.confirmed_stop_line.unwrap().0, true)
            .ok(),
        behavior: OnlineBehavior::OrbitCone,
        active_track_id: Some(active.id),
        active_track: Some(active),
        processed_cones: 0,
        cone_progress_rad: online["cone_progress_rad"].as_f64().unwrap(),
        search_distance_m: online["search_distance_m"].as_f64().unwrap(),
        execution_space_rejected: false,
    };
    (
        config,
        session,
        pose,
        SteeringEstimate {
            at,
            applied_curvature_per_m: value["steering"]["applied_curvature_per_m"]
                .as_f64()
                .unwrap(),
            commanded_curvature_per_m: value["steering"]["commanded_curvature_per_m"]
                .as_f64()
                .unwrap(),
        },
        report,
    )
}
fn speed(report: &OnlineMissionReport) -> f64 {
    match report.mission.output {
        MissionOutput::Target { max_speed_mps, .. } => max_speed_mps,
        _ => panic!("target lost"),
    }
}
#[test]
fn source_stop_hint_real_first_intervention_is_32400_without_overwriting_actual_speed() {
    for index in 0..4 {
        let (config, session, pose, steering, mut report) = recorded(index);
        let before = serde_json::to_value(&report).unwrap();
        let upper = speed(&report);
        session.apply_source_stop_speed_hint(pose.captured_at, &pose, None, 0.1, &mut report);
        let cap = speed(&report);
        if index < 2 {
            assert_eq!(cap, upper);
            assert_eq!(serde_json::to_value(&report).unwrap(), before);
        } else {
            let expected = [0.249169921875, 0.23587646484375][index - 2];
            assert!((cap - expected).abs() < 1e-12, "{index}: {cap}");
            assert!(cap >= pose.speed_mps - config.navigation.max_decel_mps2 * 0.1);
            assert!(cap < upper);
        }
        if index == 3 {
            // A hypothetical cap never replaces the actual .253125 m/s source.
            assert!(!session.permits_command(
                pose.captured_at,
                &pose,
                &pose,
                &MotionOutput::Drive {
                    speed_mps: cap,
                    curvature_per_m: 1.586053520705358
                },
                &config,
                None,
                steering
            ));
        }
    }
}
#[test]
fn source_stop_hint_pending_uses_same_error_clock_and_limits_continuation() {
    let (_, mut session, pose, _, mut report) = recorded(3);
    let (line, checked_at) = session.confirmed_stop_line.take().unwrap();
    assert_eq!(checked_at, line.last_seen);
    session.pending_stop_lines[0] = Some(line);
    if let MissionOutput::Target { point, arrival, .. } = &mut report.mission.output {
        *arrival = ArrivalBehavior::PassThrough {
            next: Point2 {
                x_m: point.x_m - 0.5,
                y_m: point.y_m,
            },
            next_heading_rad: Some(std::f64::consts::PI),
            next_max_speed_mps: 0.3,
            admission_radius_m: 0.065,
        };
    }
    session.apply_source_stop_speed_hint(pose.captured_at, &pose, None, 0.1, &mut report);
    assert!((speed(&report) - 0.23587646484375).abs() < 1e-12);
    match report.mission.output {
        MissionOutput::Target {
            arrival:
                ArrivalBehavior::PassThrough {
                    next_max_speed_mps, ..
                },
            max_speed_mps,
            ..
        } => assert_eq!(next_max_speed_mps, max_speed_mps),
        _ => panic!("continuation lost"),
    }
    let corners = StoppingEnvelope::new(
        &session.orbit_speed_hint.navigation,
        pose.pose,
        speed(&report),
        2.,
        0.35,
    )
    .unwrap()
    .corners()
    .map(|p| pose.pose.body_to_world(p));
    assert!(session.stop_regions_permit(pose.captured_at, &corners));
    // Explicitly age the same original negative evidence; no refreshed stamp.
    assert!(!session.stop_regions_permit(Timestamp(pose.captured_at.0 + 1000), &corners));
}
fn synthetic_context(pose: &PoseEstimate) -> PlanningContext {
    PlanningContext {
        source_at: pose.captured_at,
        planned_at: pose.captured_at,
        projected_pose: pose.clone(),
        steering: SteeringEstimate {
            at: pose.captured_at,
            applied_curvature_per_m: 0.,
            commanded_curvature_per_m: 0.,
        },
        adopted_revision: 1,
        last_command_change_at: Some(Timestamp(pose.captured_at.0 - 100)),
        held_speed_mps: 0.3,
        historical_speed_bound_mps: 0.3,
        historical_curvature_bound_per_m: 2.,
        adoption_constraints: Some(AdoptionConstraints {
            planned_at: pose.captured_at,
            last_command_change_at: Some(Timestamp(pose.captured_at.0 - 100)),
            adopted_revision: 1,
            source_pose: pose.pose,
            projected_pose: pose.pose,
            held_speed_mps: 0.3,
            source_age_s: 0.,
            adoption_window_s: 0.1,
            speed_bound_mps: 0.3,
            curvature_bound_per_m: 2.,
            projected_speed_mps: pose.speed_mps,
            projected_curvature_per_m: 0.,
            window_end_speed_mps: 0.3,
            window_end_curvature_per_m: 0.,
            held_curvature_per_m: 0.,
            source_travel_time_s: 0.35,
            transition_horizon_s: 0.35,
            future_travel_time_s: 0.35,
        }),
    }
}
#[test]
fn source_stop_hint_adoption_lower_invalid_clock_and_unknown_never_become_permission() {
    let (config, session, pose, steering, mut report) = recorded(3);
    let mut context = synthetic_context(&pose);
    assert!(
        context
            .adoption_constraints
            .unwrap()
            .validate(&config.navigation)
            .is_ok()
    );
    assert_eq!(
        context
            .adoption_constraints
            .unwrap()
            .speed_interval(&config.navigation)
            .0,
        0.24
    );
    let before = serde_json::to_value(&report).unwrap();
    // .235876 geometry cap is below the real window endpoint lower bound .24.
    session.apply_source_stop_speed_hint(pose.captured_at, &pose, Some(&context), 0.1, &mut report);
    assert_eq!(serde_json::to_value(&report).unwrap(), before);
    assert!(!session.permits_command(
        pose.captured_at,
        &pose,
        &pose,
        &MotionOutput::Drive {
            speed_mps: 0.1,
            curvature_per_m: 1.586
        },
        &config,
        Some(&context),
        steering
    ));
    context.planned_at.0 += 1;
    session.apply_source_stop_speed_hint(pose.captured_at, &pose, Some(&context), 0.1, &mut report);
    assert_eq!(serde_json::to_value(&report).unwrap(), before);
    // A shorter true tick raises the deceleration lower bound. Do not clamp
    // the geometric cap upward or manufacture an immediately unreachable one.
    session.apply_source_stop_speed_hint(pose.captured_at, &pose, None, 0.001, &mut report);
    assert_eq!(serde_json::to_value(&report).unwrap(), before);
    session.apply_source_stop_speed_hint(
        Timestamp(pose.captured_at.0 + 300),
        &pose,
        None,
        0.1,
        &mut report,
    );
    assert_eq!(serde_json::to_value(&report).unwrap(), before);
}
