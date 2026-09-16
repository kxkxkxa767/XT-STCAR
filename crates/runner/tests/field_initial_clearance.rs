use xt_stcar_robot_core::Timestamp;
use xt_stcar_robot_core::autonomy::{
    CrosswalkObservation, LightState, PoseEstimate, RoadObservation,
};
use xt_stcar_robot_core::mission::{Mission, MissionOutput, MissionPhase, MissionReport};
use xt_stcar_robot_runner::field::FieldScenario;
use xt_stcar_robot_runner::simulation::SimulationConfig;

fn required_near_edge(config: &SimulationConfig) -> f64 {
    config
        .autonomy
        .mission
        .footprint
        .corners(config.initial_pose)
        .into_iter()
        .map(|corner| corner.x_m)
        .fold(f64::NEG_INFINITY, f64::max)
        + config.autonomy.mission.crosswalk_stop_margin_m
}

#[test]
fn earliest_search_position_is_checked_without_using_renderer_truth() {
    let mut scenario = FieldScenario::example();
    scenario.spec.crosswalk_search_start_m = 0.8;
    for actual_near in [0.8, 2.5, 3.6] {
        scenario.crosswalk_near_x_m = actual_near;
        for result in [
            scenario.layout().map(|_| ()),
            scenario.compile().map(|_| ()),
        ] {
            let error = result.unwrap_err();
            assert!(
                error.contains("insufficient initial stopping margin"),
                "{error}"
            );
            assert!(error.contains("shortfall"), "{error}");
        }
    }
}

#[test]
fn exact_initial_clearance_boundary_is_accepted_but_one_ulp_short_is_not() {
    let original = FieldScenario::example().compile().unwrap();
    let boundary = required_near_edge(&original);
    let mut scenario = FieldScenario::example();
    scenario.spec.crosswalk_search_start_m = boundary;
    scenario.crosswalk_near_x_m = boundary;
    let accepted = scenario.compile().unwrap();
    assert_eq!(accepted.initial_pose, original.initial_pose);
    assert_eq!(accepted.autonomy.mission.crosswalk_stop_margin_m, 0.15);
    assert_eq!(
        accepted.autonomy.mission.goal_tolerance_m,
        original.autonomy.mission.goal_tolerance_m
    );
    scenario.spec.crosswalk_search_start_m = boundary.next_down();
    assert!(scenario.layout().is_err());
    assert!(scenario.compile().is_err());
}

#[test]
fn compiled_input_rechecks_the_actual_mission_stop_margin() {
    let mut scenario = FieldScenario::example();
    scenario.spec.crosswalk_search_start_m = 0.9;
    let mut config = scenario.compile().unwrap();
    config.autonomy.mission.crosswalk_stop_margin_m = 0.19;
    assert!(
        config
            .validate()
            .unwrap_err()
            .contains("insufficient initial stopping margin")
    );
}

fn observe_exact_front(
    mission: &mut Mission,
    config: &SimulationConfig,
    world_near: f64,
    at: u64,
) -> MissionReport {
    let pose = PoseEstimate {
        captured_at: Timestamp(at),
        frame_id: config.autonomy.mission.world_frame.clone(),
        pose: config.initial_pose,
        speed_mps: 0.0,
        yaw_rate_radps: 0.0,
        quality: 1.0,
    };
    let near = world_near - config.initial_pose.x_m;
    let road = RoadObservation {
        captured_at: Timestamp(at),
        frame_id: config.autonomy.mission.body_frame.clone(),
        crosswalk: Some(CrosswalkObservation {
            near_edge_m: near,
            far_edge_m: near + 0.297,
            lateral_min_m: -0.5,
            lateral_max_m: 0.5,
            confidence: 1.0,
        }),
        light: LightState::Unknown,
        light_confidence: 0.0,
        cones_body_m: vec![],
    };
    mission.update(Timestamp(at), &pose, &road)
}

#[test]
fn exact_observation_exposes_old_rearward_target_and_valid_boundary_holds_three_seconds() {
    let original = FieldScenario::example().compile().unwrap();
    // Reproduce the previously accepted mission geometry directly: the new
    // FieldScenario compiler must reject this scenario before Mission starts.
    let mut old = original.autonomy.mission.clone();
    old.crosswalk_region.min_x_m = 0.8;
    let mut mission = Mission::new(old).unwrap();
    mission.start().unwrap();
    let report = observe_exact_front(&mut mission, &original, 0.8, 0);
    let MissionOutput::Target { point, .. } = report.output else {
        panic!("expected the old rearward target: {report:?}");
    };
    assert!((original.initial_pose.x_m - 0.5).abs() < 1e-12);
    assert!((point.x_m - 0.43).abs() < 1e-12);
    assert!(original.initial_pose.x_m - point.x_m > original.autonomy.mission.goal_tolerance_m);

    let boundary = required_near_edge(&original);
    let mut scenario = FieldScenario::example();
    scenario.spec.crosswalk_search_start_m = boundary;
    scenario.crosswalk_near_x_m = boundary;
    let valid = scenario.compile().unwrap();
    let mut mission = Mission::new(valid.autonomy.mission.clone()).unwrap();
    mission.start().unwrap();
    for step in 0..30 {
        let report = observe_exact_front(&mut mission, &valid, boundary, step * 100);
        assert_eq!(report.phase, MissionPhase::CrosswalkStop);
        assert_eq!(report.output, MissionOutput::Stop);
        assert_eq!(report.crosswalk_stop_elapsed_ms, step * 100);
    }
    let released = observe_exact_front(&mut mission, &valid, boundary, 3000);
    assert_eq!(released.phase, MissionPhase::Cones);
    assert_eq!(released.crosswalk_stop_elapsed_ms, 3000);
}
