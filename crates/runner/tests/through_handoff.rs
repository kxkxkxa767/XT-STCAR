use serde_json::Value;
use xt_stcar_robot_core::tracking::TrackingConfig;
use xt_stcar_robot_runner::simulation::{SimulationConfig, simulate};
use xt_stcar_robot_runner::telemetry::{MAX_TRACE_BYTES, TelemetryMode};

#[test]
fn original_competition_configuration_hands_cones_to_light_without_an_empty_speed_interval() {
    let mut config: SimulationConfig =
        serde_json::from_str(include_str!("../../../config/competition-sim.json")).unwrap();
    assert_eq!(config.autonomy.mission.goal_tolerance_m, 0.065);
    assert_eq!(config.autonomy.navigation.goal_tolerance_m, 0.045);
    config.autonomy.navigation.tracking = TrackingConfig::Lqr {
        q_lateral: 4.0,
        q_heading: 2.0,
        r_curvature: 1.0,
        min_speed_mps: 0.03,
        max_heading_error_rad: 0.7,
        max_lateral_error_m: 0.5,
    };
    // Capture only in memory. Original dynamics, tolerances and timing remain.
    config.telemetry.mode = TelemetryMode::Trace;
    config.telemetry.max_trace_bytes = MAX_TRACE_BYTES;
    let mut log = Vec::new();
    let summary = simulate(&config, &mut log).unwrap();
    assert_eq!(summary.final_actual_speed_mps, 0.0);
    assert_eq!(summary.dropped_trace_records, 0);
    let rows: Vec<Value> = std::str::from_utf8(&log)
        .unwrap()
        .lines()
        .map(|line| serde_json::from_str(line).unwrap())
        .filter(|row: &Value| row["control"].is_object())
        .collect();
    let transition = rows
        .windows(2)
        .position(|pair| {
            pair[0]["control"]["mission"]["phase"] == "cones"
                && pair[1]["control"]["mission"]["phase"] == "approach_light"
        })
        .expect("missing continuous cone-to-light handoff")
        + 1;
    assert!(transition >= 8 && transition + 4 < rows.len());
    let before = &rows[transition - 8..transition];
    assert!(
        before
            .iter()
            .any(|row| row["actual_speed_mps"].as_f64().unwrap() > 0.28)
    );
    for row in &rows[transition - 8..=transition + 4] {
        let control = &row["control"];
        assert_ne!(
            control["navigation"]["reason"], "empty_speed_interval",
            "{row}"
        );
        if control["mission"]["phase"] == "cones" {
            assert_eq!(
                control["mission"]["output"]["arrival"]["admission_radius_m"],
                0.065
            );
            assert_eq!(
                control["navigation"]["diagnostics"]["waypoint_admission_radius_m"],
                0.065
            );
        }
    }
    assert_eq!(rows[transition]["control"]["command"]["type"], "drive");
    let diagnostics = &rows[transition]["control"]["navigation"]["diagnostics"];
    assert!(
        diagnostics["speed_lower_mps"].as_f64().unwrap()
            <= config.autonomy.mission.approach_speed_mps
    );
    assert!(
        summary.completed,
        "handoff at {} passed, but the later race failed: at={}ms, fault={:?}, pose={:?}, first_navigation_failure={:?}",
        rows[transition]["control"]["at"],
        summary.elapsed_ms,
        summary.fault,
        summary.final_pose,
        summary
            .first_navigation_failure
            .as_ref()
            .map(|failure| failure.first_at)
    );
}
