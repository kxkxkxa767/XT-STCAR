//! Synthetic measurement sequence for completion precedence, not a plant/odometry
//! accuracy test. Only the final measurement gap differs between the two runs.
use serde_json::Value;
use std::f64::consts::TAU;
use xt_stcar_robot_core::autonomy::{
    CrosswalkObservation, LightState, Point2, Pose2, PoseEstimate, RoadObservation,
};
use xt_stcar_robot_core::{LidarSample, Timestamp};
use xt_stcar_robot_runner::autonomy::{AutonomyConfig, RoadFrame};
use xt_stcar_robot_runner::autonomy_replay::{SensorSnapshot, replay_snapshots};
use xt_stcar_robot_runner::simulation::SimulationConfig;
use xt_stcar_robot_runner::telemetry::{TelemetryConfig, TelemetryMode};

fn snapshot(
    config: &AutonomyConfig,
    at: u64,
    pose: Pose2,
    speed_mps: f64,
    light: LightState,
    crosswalk: Option<CrosswalkObservation>,
) -> SensorSnapshot {
    let captured_at = Timestamp(at);
    SensorSnapshot {
        at: captured_at,
        pose: PoseEstimate {
            captured_at,
            frame_id: config.mission.world_frame.clone(),
            pose,
            speed_mps,
            yaw_rate_radps: 0.0,
            quality: 1.0,
        },
        scan: LidarSample {
            captured_at,
            frame_id: config.scan.frame_id.clone(),
            angle_min_rad: 0.0,
            angle_increment_rad: -TAU / config.scan.bins as f64,
            range_min_m: 0.02,
            range_max_m: 12.0,
            ranges_m: vec![Some(12.0); config.scan.bins],
        },
        road: RoadFrame {
            observation: RoadObservation {
                captured_at,
                frame_id: config.mission.body_frame.clone(),
                crosswalk,
                light,
                light_confidence: 1.0,
                cones_body_m: Vec::new(),
            },
            image_width_px: 320,
            image_height_px: 240,
        },
    }
}

fn at_point(point: Point2, yaw_rad: f64) -> Pose2 {
    Pose2 {
        x_m: point.x_m,
        y_m: point.y_m,
        yaw_rad,
    }
}

fn scenario(final_gap_ms: u64) -> (AutonomyConfig, Vec<SensorSnapshot>) {
    let example = SimulationConfig::example();
    let mut config = example.autonomy;
    // A 400 ms final gap is legal to mission/navigation input handling but misses
    // the independent 300 ms safety watchdog. All samples themselves stay fresh.
    config.max_tick_gap_ms = 500;
    config.mission.max_pose_gap_ms = 500;
    config.mission.max_road_gap_ms = 500;
    assert_eq!(config.safety.heartbeat_timeout_ms, 300);
    let crosswalk = CrosswalkObservation {
        near_edge_m: 1.2,
        far_edge_m: 1.497,
        lateral_min_m: -0.8,
        lateral_max_m: 0.8,
        confidence: 1.0,
    };
    let stop_point = example.initial_pose.body_to_world(Point2 {
        x_m: crosswalk.near_edge_m
            - config.mission.footprint.front_m
            - config.mission.crosswalk_stop_margin_m,
        y_m: 0.0,
    });
    let stop_pose = at_point(stop_point, example.initial_pose.yaw_rad);
    let mut snapshots = vec![snapshot(
        &config,
        0,
        example.initial_pose,
        0.1,
        LightState::Green,
        Some(crosswalk),
    )];
    // First stationary observation at 100, followed by a full 3 seconds of fresh
    // actual-speed feedback; no repeated old frame can count toward the hold.
    snapshots.extend(
        (100..=3100)
            .step_by(100)
            .map(|at| snapshot(&config, at, stop_pose, 0.0, LightState::Green, None)),
    );
    for (index, point) in config.mission.cone_waypoints.iter().enumerate() {
        snapshots.push(snapshot(
            &config,
            3200 + index as u64 * 100,
            at_point(*point, 0.0),
            0.1,
            LightState::Green,
            None,
        ));
    }
    assert_eq!(config.mission.cone_waypoints.len(), 2);
    let light_pose = at_point(
        config.mission.light_stop_goal,
        config.mission.light_approach_yaw_rad,
    );
    snapshots.push(snapshot(
        &config,
        3400,
        light_pose,
        0.0,
        LightState::Red,
        None,
    ));
    snapshots.extend(
        (3500..=3800)
            .step_by(100)
            .map(|at| snapshot(&config, at, light_pose, 0.0, LightState::Green, None)),
    );
    snapshots.push(snapshot(
        &config,
        3800 + final_gap_ms,
        at_point(config.mission.finish_goal, config.mission.finish_yaw_rad),
        0.0,
        LightState::Green,
        None,
    ));
    assert_eq!(snapshots.len(), 40);
    (config, snapshots)
}

#[test]
fn same_tick_safety_fault_takes_precedence_over_mission_completion() {
    // The successful control prevents a repair that simply reports every run as
    // incomplete; the two fixtures differ in just one timestamp and its captures.
    for (final_gap_ms, expected_completed) in [(100, true), (400, false)] {
        let (config, snapshots) = scenario(final_gap_ms);
        let mut output = Vec::new();
        let telemetry = TelemetryConfig {
            mode: TelemetryMode::Trace,
            ..TelemetryConfig::default()
        };
        let summary = replay_snapshots(config, &snapshots, telemetry, &mut output).unwrap();
        let records: Vec<Value> = std::str::from_utf8(&output)
            .unwrap()
            .lines()
            .map(|line| serde_json::from_str(line).unwrap())
            .collect();
        let last_control = &records[records.len() - 3];
        assert_eq!(summary.processed_snapshots, 40);
        assert_eq!(last_control["mission"]["phase"], "completed");
        assert_eq!(last_control["at"], 3800 + final_gap_ms);
        assert_eq!(last_control["command"]["type"], "stop");
        assert_eq!(summary.completed, expected_completed, "{summary:?}");
        assert_eq!(records.last().unwrap()["completed"], expected_completed);
        assert_eq!(records[records.len() - 2]["kind"], "autonomy_terminal");
        assert_eq!(records[records.len() - 2]["command"]["type"], "stop");
        assert_eq!(summary.logging_errors, 0);
        assert_eq!(summary.dropped_trace_records, 0);
        if expected_completed {
            assert!(summary.fault.is_none());
            assert!(last_control["fault"].is_null());
            assert_eq!(last_control["safety"]["state"], "running");
        } else {
            assert!(summary.fault.as_deref().unwrap().contains("safety"));
            assert_eq!(last_control["safety"]["state"], "fault");
            assert!(
                records[..records.len() - 3]
                    .iter()
                    .all(|record| record["fault"].is_null())
            );
        }
    }
}
