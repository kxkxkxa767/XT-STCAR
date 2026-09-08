use xt_stcar_robot_core::autonomy::{
    CrosswalkObservation, Footprint, LightState, Point2, Pose2, PoseEstimate, Rect, RoadObservation,
};
use xt_stcar_robot_core::mission::{
    Mission, MissionConfig, MissionOutput, MissionPhase, MissionReport,
};
use xt_stcar_robot_core::{FrameId, Timestamp};

fn point(x_m: f64, y_m: f64) -> Point2 {
    Point2 { x_m, y_m }
}

fn rect(min_x_m: f64, min_y_m: f64, max_x_m: f64, max_y_m: f64) -> Rect {
    Rect {
        min_x_m,
        min_y_m,
        max_x_m,
        max_y_m,
    }
}

fn config() -> MissionConfig {
    MissionConfig {
        schema_version: 1,
        simulation_only: true,
        world_frame: FrameId("map".into()),
        body_frame: FrameId("base_link".into()),
        footprint: Footprint {
            front_m: 0.2,
            rear_m: 0.2,
            half_width_m: 0.1,
        },
        approach_goal: point(1.4, 0.0),
        crosswalk_region: rect(1.4, -0.6, 1.8, 0.6),
        cone_waypoints: vec![point(2.5, 0.3), point(3.5, 0.0)],
        light_detection_region: rect(3.0, -1.0, 5.5, 1.0),
        light_stop_region: rect(4.0, -0.15, 5.0, 0.15),
        light_stop_goal: point(4.5, 0.0),
        light_approach_yaw_rad: 0.0,
        finish_region: rect(6.0, -0.15, 7.0, 0.15),
        finish_goal: point(6.5, 0.0),
        finish_yaw_rad: 0.0,
        cruise_speed_mps: 0.3,
        approach_speed_mps: 0.2,
        goal_tolerance_m: 0.02,
        goal_heading_tolerance_rad: 0.12,
        crosswalk_stop_margin_m: 0.1,
        stopped_speed_mps: 0.01,
        crosswalk_hold_ms: 3000,
        min_green_ms: 200,
        min_green_frames: 3,
        max_pose_age_ms: 250,
        max_road_age_ms: 250,
        max_pose_gap_ms: 200,
        max_road_gap_ms: 200,
        max_observation_skew_ms: 100,
        min_pose_quality: 0.8,
        min_detection_confidence: 0.8,
    }
}

fn pose(at: u64, x: f64, y: f64, speed: f64) -> PoseEstimate {
    PoseEstimate {
        captured_at: Timestamp(at),
        frame_id: FrameId("map".into()),
        pose: Pose2 {
            x_m: x,
            y_m: y,
            yaw_rad: 0.0,
        },
        speed_mps: speed,
        yaw_rate_radps: 0.0,
        quality: 1.0,
    }
}

fn road(at: u64, light: LightState) -> RoadObservation {
    RoadObservation {
        captured_at: Timestamp(at),
        frame_id: FrameId("base_link".into()),
        crosswalk: None,
        light,
        light_confidence: 1.0,
        cones_body_m: vec![],
    }
}

fn crosswalk() -> CrosswalkObservation {
    CrosswalkObservation {
        near_edge_m: 1.5,
        far_edge_m: 1.7,
        lateral_min_m: -0.5,
        lateral_max_m: 0.5,
        confidence: 1.0,
    }
}

struct Run {
    mission: Mission,
    at: u64,
}

impl Run {
    fn new() -> Self {
        let mut mission = Mission::new(config()).unwrap();
        mission.start().unwrap();
        Self { mission, at: 0 }
    }
    fn tick(&mut self, x: f64, y: f64, speed: f64, light: LightState) -> MissionReport {
        self.at += 100;
        self.mission.update(
            Timestamp(self.at),
            &pose(self.at, x, y, speed),
            &road(self.at, light),
        )
    }
    fn detect(&mut self) -> MissionReport {
        self.at += 100;
        let mut r = road(self.at, LightState::Unknown);
        r.crosswalk = Some(crosswalk());
        self.mission
            .update(Timestamp(self.at), &pose(self.at, 0.0, 0.0, 0.1), &r)
    }
    fn arrive_crosswalk(&mut self) {
        let report = self.detect();
        match report.output {
            MissionOutput::Target {
                point,
                max_speed_mps,
            } => {
                assert!((point.x_m - 1.2).abs() < 1e-12);
                assert_eq!(point.y_m, 0.0);
                assert_eq!(max_speed_mps, 0.2);
            }
            _ => panic!("expected locked crosswalk target"),
        }
        let report = self.tick(1.2, 0.0, 0.0, LightState::Unknown);
        assert_eq!(report.phase, MissionPhase::CrosswalkStop);
        assert_eq!(report.crosswalk_stop_elapsed_ms, 0);
    }
    fn finish_crosswalk(&mut self) {
        self.arrive_crosswalk();
        for step in 1..30 {
            let report = self.tick(1.2, 0.0, 0.0, LightState::Unknown);
            assert_eq!(report.phase, MissionPhase::CrosswalkStop);
            assert_eq!(report.output, MissionOutput::Stop);
            assert_eq!(report.crosswalk_stop_elapsed_ms, step * 100);
        }
        let report = self.tick(1.2, 0.0, 0.0, LightState::Unknown);
        assert_eq!(report.phase, MissionPhase::Cones);
        assert_eq!(report.crosswalk_stop_elapsed_ms, 3000);
    }
    fn approach_light(&mut self) {
        self.finish_crosswalk();
        assert_eq!(
            self.tick(2.5, 0.3, 0.1, LightState::Green).phase,
            MissionPhase::Cones
        );
        assert_eq!(
            self.tick(3.5, 0.0, 0.1, LightState::Green).phase,
            MissionPhase::ApproachLight
        );
    }
    fn wait_green(&mut self) {
        self.approach_light();
        let report = self.tick(4.5, 0.0, 0.0, LightState::Red);
        assert_eq!(report.phase, MissionPhase::WaitGreen);
        assert_eq!(report.output, MissionOutput::Stop);
    }
    fn release_green(&mut self) {
        self.wait_green();
        for i in 0..3 {
            let report = self.tick(4.5, 0.0, 0.0, LightState::Green);
            assert_eq!(
                report.phase,
                if i < 2 {
                    MissionPhase::WaitGreen
                } else {
                    MissionPhase::Finish
                }
            );
        }
    }
}

#[test]
fn complete_observation_driven_run_stops_three_seconds_and_latches_completion() {
    let mut run = Run::new();
    run.release_green();
    let report = run.tick(6.5, 0.0, 0.0, LightState::Unknown);
    assert_eq!(report.phase, MissionPhase::Completed);
    assert_eq!(report.output, MissionOutput::Stop);
    assert!(run.mission.start().is_err());
    let mut invalid = pose(0, f64::NAN, 0.0, 0.0);
    invalid.quality = 0.0;
    run.mission
        .update(Timestamp(run.at + 100), &invalid, &road(0, LightState::Red));
    let final_report = run
        .mission
        .update(Timestamp(0), &invalid, &road(0, LightState::Red));
    assert_eq!(final_report.phase, MissionPhase::Completed);
    assert_eq!(final_report.output, MissionOutput::Stop);
    assert_eq!(final_report.at, Timestamp(run.at + 100));
}

#[test]
fn crosswalk_world_projection_rejects_a_fresh_but_unassociated_pose() {
    let mut cfg = config();
    cfg.cruise_speed_mps = 1.5;
    cfg.approach_speed_mps = 1.5;
    let mut mission = Mission::new(cfg.clone()).unwrap();
    mission.start().unwrap();
    let mut captured = road(0, LightState::Unknown);
    captured.crosswalk = Some(crosswalk());
    // At capture, the body origin was x=0 and the real near edge was x=1.5.
    // The later pose is still within the permitted 100 ms skew, but using it
    // would move the line to x=1.65 and the stop target from 1.2 to 1.35.
    // At that incorrect target the 0.2 m nose has already crossed the real line.
    let rejected = mission.update(Timestamp(100), &pose(100, 0.15, 0.0, 1.5), &captured);
    assert_eq!(rejected.phase, MissionPhase::Fault);
    assert_eq!(rejected.output, MissionOutput::Stop);
    assert!(rejected.reason.contains("same capture timestamp"));
    captured.captured_at = Timestamp(200);
    assert_eq!(
        mission
            .update(Timestamp(200), &pose(200, 0.3, 0.0, 0.0), &captured)
            .phase,
        MissionPhase::Fault,
    );

    // A paired camera/pose sample still produces metric geometry normally.
    let mut paired = Mission::new(cfg).unwrap();
    paired.start().unwrap();
    captured.captured_at = Timestamp(100);
    let accepted = paired.update(Timestamp(100), &pose(100, 0.15, 0.0, 1.5), &captured);
    let MissionOutput::Target { point, .. } = accepted.output else {
        panic!("paired geometry should produce a stop target");
    };
    assert!((point.x_m - 1.35).abs() < 1e-12);
}

#[test]
fn light_only_observations_keep_the_configured_skew_allowance() {
    let mut mission = Mission::new(config()).unwrap();
    mission.start().unwrap();
    let report = mission.update(
        Timestamp(100),
        &pose(100, 0.03, 0.0, 0.3),
        &road(0, LightState::Red),
    );
    assert_eq!(report.phase, MissionPhase::ApproachCrosswalk);
    assert!(matches!(report.output, MissionOutput::Target { .. }));
}

#[test]
fn motion_resets_crosswalk_hold_and_yaw_motion_does_not_count_as_stopped() {
    let mut run = Run::new();
    run.arrive_crosswalk();
    for _ in 0..5 {
        run.tick(1.2, 0.0, 0.0, LightState::Unknown);
    }
    let moving = run.tick(1.2, 0.0, 0.02, LightState::Unknown);
    assert_eq!(moving.crosswalk_stop_elapsed_ms, 0);
    assert_eq!(moving.output, MissionOutput::Stop);
    run.tick(1.2, 0.0, 0.0, LightState::Unknown);
    run.at += 100;
    let mut turning = pose(run.at, 1.2, 0.0, 0.0);
    turning.yaw_rate_radps = 0.2;
    let turning_report = run.mission.update(
        Timestamp(run.at),
        &turning,
        &road(run.at, LightState::Unknown),
    );
    assert_eq!(turning_report.crosswalk_stop_elapsed_ms, 0);
    run.tick(1.2, 0.0, 0.0, LightState::Unknown);
    for _ in 0..29 {
        assert_eq!(
            run.tick(1.2, 0.0, 0.0, LightState::Unknown).phase,
            MissionPhase::CrosswalkStop
        );
    }
    assert_eq!(
        run.tick(1.2, 0.0, 0.0, LightState::Unknown).phase,
        MissionPhase::Cones
    );
}

#[test]
fn repeated_measurements_do_not_accrue_stop_time_and_eventually_expire() {
    let mut run = Run::new();
    run.arrive_crosswalk();
    let p = pose(run.at, 1.2, 0.0, 0.0);
    let r = road(run.at, LightState::Unknown);
    let duplicate = run.mission.update(Timestamp(run.at + 100), &p, &r);
    assert_eq!(duplicate.phase, MissionPhase::CrosswalkStop);
    assert_eq!(duplicate.crosswalk_stop_elapsed_ms, 0);
    let expired = run.mission.update(Timestamp(run.at + 250), &p, &r);
    assert_eq!(expired.phase, MissionPhase::Fault);
    assert_eq!(expired.output, MissionOutput::Stop);
    assert!(expired.reason.contains("expired"));
}

#[test]
fn crosswalk_requires_lateral_overlap_and_lock_is_not_moved_by_new_detection() {
    let mut run = Run::new();
    let mut observation = road(100, LightState::Unknown);
    let mut offside = crosswalk();
    // Merely touching the footprint's lateral boundary is not positive overlap.
    offside.lateral_min_m = config().footprint.half_width_m;
    offside.lateral_max_m = 0.5;
    observation.crosswalk = Some(offside);
    let result = run
        .mission
        .update(Timestamp(100), &pose(100, 0.0, 0.0, 0.1), &observation);
    assert_eq!(
        result.output,
        MissionOutput::Target {
            point: config().approach_goal,
            max_speed_mps: 0.2
        }
    );
    run.at = 100;
    let locked = run.detect();
    run.at += 100;
    let mut shifted = road(run.at, LightState::Unknown);
    let mut candidate = crosswalk();
    candidate.near_edge_m = 1.55;
    candidate.far_edge_m = 1.75;
    shifted.crosswalk = Some(candidate);
    let result = run
        .mission
        .update(Timestamp(run.at), &pose(run.at, 0.0, 0.0, 0.1), &shifted);
    assert_eq!(result.output, locked.output);
}

#[test]
fn a_repeated_crosswalk_after_release_does_not_restart_the_three_second_wait() {
    let mut run = Run::new();
    run.finish_crosswalk();
    run.at += 100;
    let mut observed_again = road(run.at, LightState::Unknown);
    observed_again.crosswalk = Some(crosswalk());
    let report = run.mission.update(
        Timestamp(run.at),
        &pose(run.at, 2.5, 0.3, 0.1),
        &observed_again,
    );
    assert_eq!(report.phase, MissionPhase::Cones);
    assert_eq!(report.waypoint_index, 1);
    assert_eq!(report.crosswalk_stop_elapsed_ms, 3000);
    assert!(matches!(report.output, MissionOutput::Target { .. }));
}

#[test]
fn crosswalk_overrun_and_missing_required_marker_fail_the_run() {
    let mut run = Run::new();
    run.detect();
    let overrun = run.tick(1.31, 0.0, 0.0, LightState::Unknown);
    assert_eq!(overrun.phase, MissionPhase::Fault);
    assert!(overrun.reason.contains("crosswalk crossed"));
    assert_eq!(
        run.tick(1.2, 0.0, 0.0, LightState::Green).phase,
        MissionPhase::Fault
    );
    let mut run = Run::new();
    let missing = run.tick(1.4, 0.0, 0.0, LightState::Unknown);
    assert_eq!(missing.phase, MissionPhase::Fault);
    assert!(missing.reason.contains("not identified"));
}

#[test]
fn red_yellow_unknown_conflicting_and_low_confidence_green_never_release() {
    let mut run = Run::new();
    run.wait_green();
    for light in [
        LightState::Red,
        LightState::Yellow,
        LightState::Unknown,
        LightState::Conflicting,
    ] {
        for _ in 0..30 {
            let report = run.tick(4.5, 0.0, 0.0, light);
            assert_eq!(report.phase, MissionPhase::WaitGreen);
            assert_eq!(report.output, MissionOutput::Stop);
        }
    }
    run.at += 100;
    let mut uncertain = road(run.at, LightState::Green);
    uncertain.light_confidence = 0.79;
    assert_eq!(
        run.mission
            .update(Timestamp(run.at), &pose(run.at, 4.5, 0.0, 0.0), &uncertain)
            .phase,
        MissionPhase::WaitGreen
    );
}

#[test]
fn green_needs_distinct_frames_after_actual_stop_and_any_ambiguity_resets_it() {
    let mut run = Run::new();
    run.approach_light();
    let moving = run.tick(4.5, 0.0, 0.02, LightState::Green);
    assert_eq!(moving.phase, MissionPhase::ApproachLight);
    let first = run.tick(4.5, 0.0, 0.0, LightState::Green);
    assert_eq!(first.phase, MissionPhase::WaitGreen);
    assert_eq!(first.green_elapsed_ms, 0);
    let old_pose = pose(run.at, 4.5, 0.0, 0.0);
    let old_road = road(run.at, LightState::Green);
    let duplicate = run
        .mission
        .update(Timestamp(run.at + 50), &old_pose, &old_road);
    assert_eq!(duplicate.green_elapsed_ms, 0);
    assert_eq!(duplicate.phase, MissionPhase::WaitGreen);
    assert_eq!(
        run.tick(4.5, 0.0, 0.0, LightState::Green).phase,
        MissionPhase::WaitGreen
    );
    assert_eq!(
        run.tick(4.5, 0.0, 0.0, LightState::Unknown)
            .green_elapsed_ms,
        0
    );
    for i in 0..3 {
        let report = run.tick(4.5, 0.0, 0.0, LightState::Green);
        assert_eq!(
            report.phase,
            if i < 2 {
                MissionPhase::WaitGreen
            } else {
                MissionPhase::Finish
            }
        );
    }
}

#[test]
fn light_and_finish_require_the_entire_footprint_not_just_the_pose_origin() {
    let mut run = Run::new();
    run.approach_light();
    run.at += 100;
    let mut rotated = pose(run.at, 4.5, 0.0, 0.0);
    rotated.pose.yaw_rad = std::f64::consts::FRAC_PI_2;
    let failed = run
        .mission
        .update(Timestamp(run.at), &rotated, &road(run.at, LightState::Red));
    assert_eq!(failed.phase, MissionPhase::Fault);
    assert!(failed.reason.contains("entire footprint"));
    let mut run = Run::new();
    run.release_green();
    run.at += 100;
    let mut rotated = pose(run.at, 6.5, 0.0, 0.0);
    rotated.pose.yaw_rad = std::f64::consts::FRAC_PI_2;
    let failed = run.mission.update(
        Timestamp(run.at),
        &rotated,
        &road(run.at, LightState::Unknown),
    );
    assert_eq!(failed.phase, MissionPhase::Fault);
    assert!(failed.reason.contains("finish requires"));
}

#[test]
fn light_and_finish_keep_a_target_when_position_is_near_but_heading_is_wrong() {
    let mut run = Run::new();
    run.approach_light();
    for _ in 0..4 {
        run.at += 100;
        let mut misaligned = pose(run.at, 4.5, 0.0, 0.0);
        misaligned.pose.yaw_rad = 0.2;
        assert!(
            config()
                .footprint
                .inside(misaligned.pose, config().light_stop_region)
        );
        let report = run.mission.update(
            Timestamp(run.at),
            &misaligned,
            &road(run.at, LightState::Green),
        );
        assert_eq!(report.phase, MissionPhase::ApproachLight);
        assert!(matches!(report.output, MissionOutput::Target { .. }));
        assert_eq!(report.green_elapsed_ms, 0);
    }
    assert_eq!(
        run.tick(4.5, 0.0, 0.0, LightState::Green).phase,
        MissionPhase::WaitGreen
    );
    run.tick(4.5, 0.0, 0.0, LightState::Green);
    assert_eq!(
        run.tick(4.5, 0.0, 0.0, LightState::Green).phase,
        MissionPhase::Finish
    );
    run.at += 100;
    let mut misaligned = pose(run.at, 6.5, 0.0, 0.0);
    misaligned.pose.yaw_rad = 0.2;
    assert!(
        config()
            .footprint
            .inside(misaligned.pose, config().finish_region)
    );
    let report = run.mission.update(
        Timestamp(run.at),
        &misaligned,
        &road(run.at, LightState::Unknown),
    );
    assert_eq!(report.phase, MissionPhase::Finish);
    assert!(matches!(report.output, MissionOutput::Target { .. }));
    assert_eq!(
        run.tick(6.5, 0.0, 0.0, LightState::Unknown).phase,
        MissionPhase::Completed
    );
}

#[test]
fn heading_drift_while_waiting_resets_green_confirmation() {
    let mut run = Run::new();
    run.wait_green();
    run.tick(4.5, 0.0, 0.0, LightState::Green);
    run.at += 100;
    let mut misaligned = pose(run.at, 4.5, 0.0, 0.0);
    misaligned.pose.yaw_rad = 0.2;
    let report = run.mission.update(
        Timestamp(run.at),
        &misaligned,
        &road(run.at, LightState::Green),
    );
    assert_eq!(report.phase, MissionPhase::WaitGreen);
    assert_eq!(report.output, MissionOutput::Stop);
    assert_eq!(report.green_elapsed_ms, 0);
    for i in 0..3 {
        let report = run.tick(4.5, 0.0, 0.0, LightState::Green);
        assert_eq!(
            report.phase,
            if i < 2 {
                MissionPhase::WaitGreen
            } else {
                MissionPhase::Finish
            }
        );
    }
}

#[test]
fn equivalent_finish_headings_across_pi_are_accepted() {
    let mut cfg = config();
    cfg.finish_yaw_rad = std::f64::consts::PI - 0.04;
    let mut mission = Mission::new(cfg).unwrap();
    mission.start().unwrap();
    let mut run = Run { mission, at: 0 };
    run.release_green();
    run.at += 100;
    let mut aligned = pose(run.at, 6.5, 0.0, 0.0);
    aligned.pose.yaw_rad = -std::f64::consts::PI + 0.04;
    let report = run.mission.update(
        Timestamp(run.at),
        &aligned,
        &road(run.at, LightState::Unknown),
    );
    assert_eq!(report.phase, MissionPhase::Completed);
}

#[test]
fn finish_heading_configuration_checks_range_tolerance_and_entire_footprint() {
    for yaw in [f64::NAN, f64::INFINITY, std::f64::consts::TAU] {
        let mut cfg = config();
        cfg.finish_yaw_rad = yaw;
        assert!(Mission::new(cfg).is_err());
    }
    for tolerance in [
        0.0,
        -0.1,
        f64::NAN,
        f64::INFINITY,
        std::f64::consts::FRAC_PI_2,
    ] {
        let mut cfg = config();
        cfg.goal_heading_tolerance_rad = tolerance;
        assert!(Mission::new(cfg).is_err());
    }
    let mut cfg = config();
    cfg.finish_yaw_rad = std::f64::consts::FRAC_PI_2;
    assert!(cfg.finish_region.contains(cfg.finish_goal));
    assert!(cfg.footprint.inside(
        Pose2 {
            x_m: cfg.finish_goal.x_m,
            y_m: cfg.finish_goal.y_m,
            yaw_rad: 0.0
        },
        cfg.finish_region
    ));
    assert!(
        matches!(Mission::new(cfg), Err(error) if error.to_string().contains("finish goal heading"))
    );
}

#[test]
fn crossing_light_without_green_or_losing_green_before_clearance_stops() {
    let mut run = Run::new();
    run.approach_light();
    let overrun = run.tick(4.9, 0.0, 0.0, LightState::Red);
    assert_eq!(overrun.phase, MissionPhase::Fault);
    assert!(overrun.reason.contains("without confirmed green"));
    let mut run = Run::new();
    run.release_green();
    let lost = run.tick(4.6, 0.0, 0.1, LightState::Yellow);
    assert_eq!(lost.phase, MissionPhase::Fault);
    assert_eq!(lost.output, MissionOutput::Stop);
    assert!(lost.reason.contains("green signal lost"));
}

#[test]
fn fresh_samples_after_a_long_gap_cannot_claim_continuous_stopping() {
    let mut run = Run::new();
    run.arrive_crosswalk();
    run.at += 3000;
    let gap = run.mission.update(
        Timestamp(run.at),
        &pose(run.at, 1.2, 0.0, 0.0),
        &road(run.at, LightState::Unknown),
    );
    assert_eq!(gap.phase, MissionPhase::Fault);
    assert!(gap.reason.contains("continuity"));
}

#[test]
fn bad_time_quality_frames_and_reused_timestamp_data_fault_and_stay_latched() {
    for kind in 0..6 {
        let mut run = Run::new();
        run.detect();
        let mut p = pose(200, 0.1, 0.0, 0.1);
        let mut r = road(200, LightState::Unknown);
        let mut now = 200;
        match kind {
            0 => p.quality = 0.79,
            1 => p.pose.x_m = f64::NAN,
            2 => r.frame_id = FrameId("camera".into()),
            3 => p.captured_at = Timestamp(201),
            4 => {
                now = 99;
                p.captured_at = Timestamp(99);
                r.captured_at = Timestamp(99);
            }
            _ => {
                now = 100;
                p.captured_at = Timestamp(100);
                r.captured_at = Timestamp(100);
            }
        }
        let failed = run.mission.update(Timestamp(now), &p, &r);
        assert_eq!(failed.phase, MissionPhase::Fault, "case {kind}");
        assert_eq!(failed.output, MissionOutput::Stop);
        assert!(run.mission.start().is_err());
        assert_eq!(
            run.mission
                .update(
                    Timestamp(300),
                    &pose(300, 0.0, 0.0, 0.0),
                    &road(300, LightState::Green)
                )
                .phase,
            MissionPhase::Fault
        );
    }
}

#[test]
fn ordinary_stalls_fail_after_ten_seconds_but_required_waits_are_exempt() {
    let mut run = Run::new();
    for _ in 0..101 {
        assert_eq!(
            run.tick(0.0, 0.0, 0.0, LightState::Unknown).phase,
            MissionPhase::ApproachCrosswalk
        );
    }
    let stalled = run.tick(0.0, 0.0, 0.0, LightState::Unknown);
    assert_eq!(stalled.phase, MissionPhase::Fault);
    assert!(stalled.reason.contains("10 seconds"));
}

#[test]
fn mission_configuration_rejects_unverified_live_claims_and_invalid_rules() {
    for kind in 0..7 {
        let mut c = config();
        match kind {
            0 => c.simulation_only = false,
            1 => c.crosswalk_hold_ms = 2999,
            2 => c.min_green_frames = 1,
            3 => c.crosswalk_stop_margin_m = c.goal_tolerance_m,
            4 => c.light_stop_goal = point(3.0, 0.0),
            5 => c.max_pose_gap_ms = 0,
            _ => c.cone_waypoints.clear(),
        }
        assert!(Mission::new(c).is_err());
    }
    let mut mission = Mission::new(config()).unwrap();
    assert_eq!(
        mission
            .update(
                Timestamp(0),
                &pose(0, 0.0, 0.0, 0.0),
                &road(0, LightState::Unknown)
            )
            .phase,
        MissionPhase::Idle
    );
    mission.start().unwrap();
    assert!(mission.start().is_err());
}
