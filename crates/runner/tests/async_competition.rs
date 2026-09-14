use xt_stcar_robot_core::MotionOutput;
use xt_stcar_robot_core::mission::MissionPhase;
use xt_stcar_robot_core::tracking::TrackingConfig;
use xt_stcar_robot_runner::async_simulation::{AsyncSimulationOptions, simulate_async};
use xt_stcar_robot_runner::simulation::SimulationConfig;

#[test]
fn full_worker_source_dropout_stops_the_real_plant_and_preserves_failure_evidence() {
    let config = SimulationConfig::example();
    let timing = AsyncSimulationOptions {
        stop_input_at_ms: Some(100),
        ..Default::default()
    };
    let summary = simulate_async(&config, &timing, &mut std::io::sink()).unwrap();
    assert!(!summary.completed);
    assert_eq!(summary.fault.as_deref(), Some("runtime CommandExpired"));
    assert!(summary.counts.newly_adopted_plans > 0);
    assert!(summary.counts.drive_polls > 0);
    assert!(summary.first_problem.is_some());
    assert_eq!(summary.terminal_command, MotionOutput::Stop);
    assert_eq!(summary.final_actual_speed_mps, 0.0);
    assert_eq!(summary.final_actual_curvature_per_m, 0.0);
    assert!(summary.minimum_cone_clearance_m > 0.0);
    assert!(summary.elapsed_ms < 600);
}

#[test]
fn functional_timing_rejects_unbounded_or_ambiguous_event_schedules() {
    let config = SimulationConfig::example();
    for timing in [
        AsyncSimulationOptions {
            output_period_ms: 0,
            ..Default::default()
        },
        AsyncSimulationOptions {
            input_delays_ms: [60, 100],
            ..Default::default()
        },
        AsyncSimulationOptions {
            input_delays_ms: [61, 80],
            ..Default::default()
        },
        AsyncSimulationOptions {
            adoption_delays_ms: [0, 7, 20],
            ..Default::default()
        },
    ] {
        assert!(simulate_async(&config, &timing, &mut std::io::sink()).is_err());
    }
}

#[test]
fn default_pp_worker_completes_all_competition_phases_with_delayed_sensors() {
    assert_complete_competition(SimulationConfig::example());
}

#[test]
fn experimental_lqr_worker_completes_all_competition_phases_with_delayed_sensors() {
    let mut config = SimulationConfig::example();
    config.autonomy.navigation.tracking = TrackingConfig::Lqr {
        q_lateral: 4.0,
        q_heading: 2.0,
        r_curvature: 1.0,
        min_speed_mps: 0.03,
        max_heading_error_rad: 0.7,
        max_lateral_error_m: 0.5,
    };
    assert_complete_competition(config);
}

fn assert_complete_competition(config: SimulationConfig) {
    let summary = simulate_async(
        &config,
        &AsyncSimulationOptions::default(),
        &mut std::io::sink(),
    )
    .unwrap();
    assert!(summary.completed, "{:?}", summary.fault);
    assert_eq!(
        summary.phases,
        vec![
            MissionPhase::ApproachCrosswalk,
            MissionPhase::CrosswalkStop,
            MissionPhase::Cones,
            MissionPhase::ApproachLight,
            MissionPhase::WaitGreen,
            MissionPhase::Finish,
            MissionPhase::Completed,
        ]
    );
    assert_eq!(
        summary.crosswalk_hold_ms,
        config.autonomy.mission.crosswalk_hold_ms
    );
    assert!(summary.green_observed_ms >= config.autonomy.mission.min_green_ms);
    assert!(summary.counts.turning_drive_polls > 0);
    assert!(summary.minimum_cone_clearance_m > 0.0);
    assert_eq!(summary.terminal_command, MotionOutput::Stop);
    assert_eq!(summary.final_actual_speed_mps, 0.0);
    assert_eq!(summary.final_actual_curvature_per_m, 0.0);
    assert!(
        config
            .autonomy
            .mission
            .footprint
            .inside(summary.final_pose, config.autonomy.mission.finish_region)
    );
    assert_eq!(summary.timing_statistics.processor_ns.samples, 0);
    assert!(summary.recent_attempts.len() <= 16);
    assert!(summary.transitions.len() <= 128);
}

#[test]
fn explicit_host_timing_records_each_completed_plan_without_changing_source_expiry() {
    let config = SimulationConfig::example();
    let timing = AsyncSimulationOptions {
        stop_input_at_ms: Some(100),
        measure_wall_time: true,
        ..Default::default()
    };
    let summary = simulate_async(&config, &timing, &mut std::io::sink()).unwrap();
    assert_eq!(summary.fault.as_deref(), Some("runtime CommandExpired"));
    assert_eq!(
        summary.timing_statistics.processor_ns.samples,
        summary.counts.completed_plans
    );
    assert_eq!(
        summary.timing_statistics.certificate_ns.samples,
        summary.counts.completed_plans
    );
    assert!(summary.timing_statistics.navigation_ns.samples > 0);
    assert_eq!(summary.recent_attempts[0].source_at.0, 0);
    assert_eq!(summary.recent_attempts[0].planned_at.0, 60);
    assert_eq!(summary.final_actual_speed_mps, 0.0);
    assert_eq!(summary.final_actual_curvature_per_m, 0.0);
}
