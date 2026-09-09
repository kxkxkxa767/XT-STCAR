use xt_stcar_robot_core::autonomy::{LightState, PoseEstimate};
use xt_stcar_robot_core::mission::MissionPhase;
use xt_stcar_robot_core::{LidarSample, MotionOutput, Timestamp};
use xt_stcar_robot_runner::autonomy::{AutonomyController, RoadFrame};
use xt_stcar_robot_runner::simulation::{
    SimulationConfig, SimulationFault, render_camera, simulate, synthetic_scan,
};
use xt_stcar_vision::road::RoadDetector;

fn sample(config: &SimulationConfig, at: u64) -> (PoseEstimate, LidarSample, RoadFrame) {
    let pose = config.initial_pose;
    let estimate = PoseEstimate {
        captured_at: Timestamp(at),
        frame_id: config.autonomy.mission.world_frame.clone(),
        pose,
        speed_mps: 0.0,
        yaw_rate_radps: 0.0,
        quality: 1.0,
    };
    let scan = synthetic_scan(config, pose, &config.cones, Timestamp(at));
    let rgb = render_camera(config, pose, LightState::Green).unwrap();
    let road = RoadDetector::new(config.road.clone())
        .unwrap()
        .detect(
            &rgb,
            &[],
            Timestamp(at),
            config.autonomy.mission.body_frame.clone(),
        )
        .unwrap();
    (
        estimate,
        scan,
        RoadFrame {
            observation: road,
            image_width_px: rgb.width(),
            image_height_px: rgb.height(),
        },
    )
}

#[test]
fn competition_closes_the_loop_from_rgb_and_range_feedback_without_motion_events() {
    let config = SimulationConfig::example();
    let mut log = Vec::new();
    let summary = simulate(&config, &mut log).unwrap();
    assert!(summary.completed, "{summary:#?}");
    assert!(summary.fault.is_none());
    assert_eq!(summary.final_actual_speed_mps, 0.0);
    assert!(summary.minimum_cone_clearance_m > 0.0);
    assert!(summary.crosswalk_hold_ms >= 3000);
    assert!(summary.green_observed_ms >= config.autonomy.mission.min_green_ms);
    assert!(
        summary.distance_m
            > config
                .initial_pose
                .point()
                .distance(config.autonomy.mission.finish_goal)
    );
    for phase in [
        MissionPhase::CrosswalkStop,
        MissionPhase::Cones,
        MissionPhase::WaitGreen,
        MissionPhase::Completed,
    ] {
        assert!(summary.phases.contains(&phase), "{phase:?}");
    }
    assert!(
        config
            .autonomy
            .mission
            .footprint
            .inside(summary.final_pose, config.autonomy.mission.finish_region)
    );
    let rows: Vec<serde_json::Value> = std::str::from_utf8(&log)
        .unwrap()
        .lines()
        .map(|line| serde_json::from_str(line).unwrap())
        .collect();
    assert_eq!(rows[rows.len() - 2]["command"]["type"], "stop");
    assert!(
        log.len() < 100_000,
        "default event journal should avoid per-tick disk data"
    );
}

#[test]
fn stale_camera_pose_or_scan_stops_and_the_plant_actually_brakes() {
    for fault in [
        SimulationFault::CameraDropout,
        SimulationFault::LidarDropout,
        SimulationFault::PoseDropout,
    ] {
        let mut config = SimulationConfig::example();
        config.fault = fault;
        config.fault_at_ms = 1000;
        let summary = simulate(&config, &mut Vec::new()).unwrap();
        assert!(!summary.completed);
        assert!(summary.fault.is_some());
        assert!(summary.elapsed_ms < 2000, "{summary:?}");
        assert!(summary.braking_ticks > 0, "{summary:?}");
        assert_eq!(summary.final_actual_speed_mps, 0.0);
        assert!(summary.minimum_cone_clearance_m > 0.0);
    }
}

#[test]
fn snapshot_clock_frame_coverage_and_identity_faults_are_latched() {
    let config = SimulationConfig::example();
    for fault in 0..6 {
        let mut control = AutonomyController::new(config.autonomy.clone()).unwrap();
        control.start().unwrap();
        let (p, s, r) = sample(&config, 0);
        assert!(control.tick(Timestamp(0), &p, &s, &r).fault.is_none());
        let (mut p, mut s, mut r) = sample(&config, 100);
        match fault {
            0 => s.ranges_m[0] = None,
            1 => s.captured_at = Timestamp(0),
            2 => {
                s.captured_at = Timestamp(0);
                p.captured_at = Timestamp(0);
                s.ranges_m[5] = Some(1.0);
            }
            3 => p.frame_id.0 = "other_world".into(),
            4 => r.observation.captured_at = Timestamp(101),
            _ => p.quality = 0.01,
        }
        if fault == 0 {
            for range in &mut s.ranges_m[0..30] {
                *range = None;
            }
        }
        let rejected = control.tick(Timestamp(100), &p, &s, &r);
        assert!(rejected.fault.is_some(), "fault={fault}: {rejected:?}");
        assert_eq!(rejected.command, MotionOutput::Stop);
        let (p, s, r) = sample(&config, 200);
        assert!(control.tick(Timestamp(200), &p, &s, &r).fault.is_some());
    }
}

#[test]
fn control_deadline_is_independent_of_recent_sensor_values() {
    let config = SimulationConfig::example();
    let mut control = AutonomyController::new(config.autonomy.clone()).unwrap();
    control.start().unwrap();
    let (p, s, r) = sample(&config, 0);
    assert!(control.tick(Timestamp(0), &p, &s, &r).fault.is_none());
    let (p, s, r) = sample(&config, 500);
    let stopped = control.tick(Timestamp(500), &p, &s, &r);
    assert!(stopped.fault.unwrap().contains("watchdog"));
    assert_eq!(stopped.command, MotionOutput::Stop);
}

#[test]
fn experimental_lqr_still_passes_through_the_source_watchdog_and_fault_latch() {
    use xt_stcar_robot_core::tracking::TrackingConfig;
    let mut config = SimulationConfig::example();
    config.autonomy.navigation.tracking = TrackingConfig::Lqr {
        q_lateral: 4.0,
        q_heading: 2.0,
        r_curvature: 1.0,
        min_speed_mps: 0.03,
        max_heading_error_rad: 0.7,
        max_lateral_error_m: 0.5,
    };
    let mut control = AutonomyController::new(config.autonomy.clone()).unwrap();
    control.start().unwrap();
    let (mut pose, scan, road) = sample(&config, 0);
    pose.speed_mps = 0.2; // Exercise LQR above the low-speed PP fallback.
    let first = control.tick(Timestamp(0), &pose, &scan, &road);
    assert!(first.fault.is_none(), "{first:?}");
    assert!(matches!(first.command, MotionOutput::Drive { .. }));
    let stopped = control.tick(Timestamp(500), &pose, &scan, &road);
    assert!(stopped.fault.is_some());
    assert_eq!(stopped.command, MotionOutput::Stop);
    let (pose, scan, road) = sample(&config, 600);
    let latched = control.tick(Timestamp(600), &pose, &scan, &road);
    assert!(latched.fault.is_some());
    assert_eq!(latched.command, MotionOutput::Stop);
}

#[test]
fn renderer_handles_a_tiny_border_light_roi_and_invalid_initial_placement() {
    let mut config = SimulationConfig::example();
    config.road.light_rois = vec![[0.0, 0.0, 0.01, 0.01]];
    let rgb = render_camera(&config, config.initial_pose, LightState::Red).unwrap();
    assert_eq!(rgb.dimensions(), (320, 240));
    config.initial_pose.x_m = config.cones[0].center.x_m;
    config.initial_pose.y_m = config.cones[0].center.y_m;
    assert!(config.validate().unwrap_err().contains("overlaps"));
}

#[test]
fn perception_failure_while_moving_records_fault_and_finishes_braking() {
    let mut config = SimulationConfig::example();
    config.road.max_components = 1;
    config.road.homography.matrix = [[0.0, -1.0, 1.0], [-1.0, 0.0, 0.5], [0.0, 0.0, 1.0]];
    config.max_duration_ms = 6000;
    let mut log = Vec::new();
    let summary = simulate(&config, &mut log).unwrap();
    assert!(
        summary.fault.as_ref().unwrap().contains("component limit"),
        "{summary:?}"
    );
    assert!(summary.distance_m > 0.1);
    assert!(summary.braking_ticks > 0);
    assert_eq!(summary.final_actual_speed_mps, 0.0);
    assert_eq!(summary.phases.last(), Some(&MissionPhase::Fault));
    let rows: Vec<serde_json::Value> = std::str::from_utf8(&log)
        .unwrap()
        .lines()
        .map(|s| serde_json::from_str(s).unwrap())
        .collect();
    assert_eq!(rows[rows.len() - 2]["command"]["type"], "stop");
    assert!(rows.iter().any(|r| r["control"]["fault"].is_string()));
}

#[test]
fn visual_obstacles_cannot_be_projected_using_a_different_capture_pose() {
    let config = SimulationConfig::example();
    let mut control = AutonomyController::new(config.autonomy.clone()).unwrap();
    control.start().unwrap();
    let (p, s, mut r) = sample(&config, 100);
    r.observation.captured_at = Timestamp(0);
    r.observation.cones_body_m = vec![xt_stcar_robot_core::autonomy::Point2 { x_m: 1.0, y_m: 0.0 }];
    let report = control.tick(Timestamp(100), &p, &s, &r);
    assert_eq!(report.command, MotionOutput::Stop);
    assert!(report.fault.unwrap().contains("same capture timestamp"));
}

#[test]
fn exhausted_debug_log_budget_does_not_change_the_control_or_braking_result() {
    use xt_stcar_robot_runner::telemetry::TelemetryMode;
    let mut config = SimulationConfig::example();
    config.fault = SimulationFault::PoseDropout;
    config.fault_at_ms = 1000;
    config.telemetry.mode = TelemetryMode::Summary;
    let quiet = simulate(&config, &mut Vec::new()).unwrap();
    config.telemetry.mode = TelemetryMode::Trace;
    config.telemetry.max_trace_bytes = 1;
    let trace = simulate(&config, &mut Vec::new()).unwrap();
    assert!(trace.dropped_trace_records > 0);
    assert_eq!(trace.fault, quiet.fault);
    assert_eq!(trace.final_pose, quiet.final_pose);
    assert_eq!(trace.elapsed_ms, quiet.elapsed_ms);
    assert_eq!(trace.final_actual_speed_mps, quiet.final_actual_speed_mps);
    assert_eq!(trace.braking_ticks, quiet.braking_ticks);
}
