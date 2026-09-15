use serde_json::Value;
use std::{fs, path::Path, process::Command};
use xt_stcar_robot_core::field::FieldLayout;
use xt_stcar_robot_runner::field::FieldScenario;
use xt_stcar_robot_runner::simulation::SimulationConfig;

fn field_command(command: &str, config: &Path, extra: &[&str]) -> std::process::Output {
    Command::new(env!("CARGO_BIN_EXE_xt-stcar-robot"))
        .args([command, "--config"])
        .arg(config)
        .args(extra)
        .output()
        .unwrap()
}

#[test]
fn example_compiles_to_valid_offline_config_and_reviewable_layout() {
    let example = Command::new(env!("CARGO_BIN_EXE_xt-stcar-robot"))
        .arg("field-example")
        .output()
        .unwrap();
    assert!(example.status.success());
    let scenario: FieldScenario = serde_json::from_slice(&example.stdout).unwrap();
    let temp = tempfile::tempdir().unwrap();
    let input = temp.path().join("field.json");
    fs::write(&input, &example.stdout).unwrap();

    let compiled = field_command("field-compile", &input, &[]);
    assert!(
        compiled.status.success(),
        "{}",
        String::from_utf8_lossy(&compiled.stderr)
    );
    let config: SimulationConfig = serde_json::from_slice(&compiled.stdout).unwrap();
    config.validate().unwrap();
    assert!(config.autonomy.simulation_only);
    assert_eq!(
        config.autonomy.navigation.bounds.max_x_m,
        scenario.spec.length_m
    );
    assert_eq!(
        config.autonomy.navigation.bounds.max_y_m,
        scenario.spec.width_m
    );
    assert!(config.field.is_some());

    let layout = field_command("field-layout", &input, &[]);
    assert!(
        layout.status.success(),
        "{}",
        String::from_utf8_lossy(&layout.stderr)
    );
    let layout: FieldLayout = serde_json::from_slice(&layout.stdout).unwrap();
    assert_eq!(layout.bounds, config.autonomy.navigation.bounds);
    assert_eq!(layout.initial_pose, config.initial_pose);
    assert_eq!(layout.finish_goal, config.autonomy.mission.finish_goal);

    for command in ["field-compile", "field-layout"] {
        let output_path = temp.path().join(format!("{command}.json"));
        fs::write(&output_path, "previous output").unwrap();
        let result = field_command(
            command,
            &input,
            &["--output", output_path.to_str().unwrap()],
        );
        assert!(result.status.success());
        assert!(result.stdout.is_empty());
        let saved: Value = serde_json::from_slice(&fs::read(output_path).unwrap()).unwrap();
        let stdout = field_command(command, &input, &[]);
        assert_eq!(
            saved,
            serde_json::from_slice::<Value>(&stdout.stdout).unwrap()
        );
    }
}

#[test]
fn invalid_field_or_json_preserves_previous_output() {
    let temp = tempfile::tempdir().unwrap();
    let input = temp.path().join("field.json");
    let output = temp.path().join("compiled.json");
    let mut invalid = serde_json::to_value(FieldScenario::example()).unwrap();
    invalid["spec"]["length_m"] = 2.0.into();
    for bytes in [serde_json::to_vec(&invalid).unwrap(), b"{broken".to_vec()] {
        fs::write(&input, &bytes).unwrap();
        for command in ["field-compile", "field-layout"] {
            fs::write(&output, "previous valid output").unwrap();
            let result = field_command(command, &input, &["--output", output.to_str().unwrap()]);
            assert!(!result.status.success());
            assert!(result.stdout.is_empty());
            assert_eq!(
                fs::read_to_string(&output).unwrap(),
                "previous valid output"
            );
            assert_eq!(fs::read(&input).unwrap(), bytes);
        }
    }
}

#[test]
fn field_cli_rejects_unknown_duplicate_and_missing_options() {
    let temp = tempfile::tempdir().unwrap();
    let input = temp.path().join("field.json");
    let original = serde_json::to_vec(&FieldScenario::example()).unwrap();
    fs::write(&input, &original).unwrap();
    for command in ["field-compile", "field-layout"] {
        for flags in [
            vec!["--unknown", "x"],
            vec!["--config", input.to_str().unwrap()],
            vec!["--output"],
            vec!["--output", ""],
            vec!["--output", "--config"],
            vec!["--output", "a", "--output", "b"],
            vec!["--trace"],
        ] {
            let result = field_command(command, &input, &flags);
            assert!(!result.status.success(), "{command} {flags:?}");
            assert!(result.stdout.is_empty());
            assert_eq!(fs::read(&input).unwrap(), original);
        }
        let missing_config = Command::new(env!("CARGO_BIN_EXE_xt-stcar-robot"))
            .arg(command)
            .output()
            .unwrap();
        assert!(!missing_config.status.success());
    }
    let extra = Command::new(env!("CARGO_BIN_EXE_xt-stcar-robot"))
        .args(["field-example", "--output", "x"])
        .output()
        .unwrap();
    assert!(!extra.status.success());
}

#[test]
fn output_aliases_never_replace_or_truncate_the_input() {
    let temp = tempfile::tempdir().unwrap();
    let input = temp.path().join("field.json");
    let original = serde_json::to_vec(&FieldScenario::example()).unwrap();
    fs::write(&input, &original).unwrap();
    for command in ["field-compile", "field-layout"] {
        let same = field_command(command, &input, &["--output", input.to_str().unwrap()]);
        assert!(!same.status.success());
        assert_eq!(fs::read(&input).unwrap(), original);

        let hard_link = temp.path().join(format!("{command}-hard-link.json"));
        fs::hard_link(&input, &hard_link).unwrap();
        let result = field_command(command, &input, &["--output", hard_link.to_str().unwrap()]);
        // Atomic replacement detaches the destination name instead of truncating
        // the source inode. This follows the other runner commands' policy.
        assert!(result.status.success());
        assert_eq!(fs::read(&input).unwrap(), original);
        assert_ne!(fs::read(&hard_link).unwrap(), original);

        #[cfg(unix)]
        {
            let symlink = temp.path().join(format!("{command}-symlink.json"));
            std::os::unix::fs::symlink(&input, &symlink).unwrap();
            let result = field_command(command, &input, &["--output", symlink.to_str().unwrap()]);
            assert!(!result.status.success());
            assert!(
                fs::symlink_metadata(&symlink)
                    .unwrap()
                    .file_type()
                    .is_symlink()
            );
            assert_eq!(fs::read(&input).unwrap(), original);
        }
    }
}

#[test]
fn nominal_wide_and_small_compiled_files_survive_roundtrip_and_simulation_startup() {
    let mut wide = FieldScenario::example();
    wide.spec.length_m = 8.0;
    wide.spec.width_m = 4.0;
    wide.spec.bottom_lane_width_m = 1.0;
    wide.spec.top_lane_width_m = 1.0;
    wide.spec.bottom_straight_span_m = 5.5;
    wide.spec.top_straight_span_m = 5.5;
    wide.spec.left_cone_from_top_m = 2.0;
    wide.spec.right_cone_from_bottom_m = 2.0;
    wide.spec.light_front_x_m = 7.2;
    let mut small = wide.clone();
    small.spec.length_m = 5.0;
    small.spec.bottom_straight_span_m = 3.0;
    small.spec.top_straight_span_m = 3.0;
    small.spec.left_cone_from_left_m = 1.0;
    small.spec.right_cone_from_right_m = 1.0;
    small.spec.light_front_x_m = 4.2;
    small.cone_route_radius_m = 0.7;
    small.crosswalk_near_x_m = 2.0;

    let temp = tempfile::tempdir().unwrap();
    for (name, scenario) in [
        ("nominal", FieldScenario::example()),
        ("wide", wide),
        ("small", small),
    ] {
        let spec_path = temp.path().join(format!("{name}-spec.json"));
        let compiled_path = temp.path().join(format!("{name}-compiled.json"));
        let run_path = temp.path().join(format!("{name}-run.jsonl"));
        fs::write(&spec_path, serde_json::to_vec_pretty(&scenario).unwrap()).unwrap();
        let compiled = field_command(
            "field-compile",
            &spec_path,
            &["--output", compiled_path.to_str().unwrap()],
        );
        assert!(
            compiled.status.success(),
            "{name}: {}",
            String::from_utf8_lossy(&compiled.stderr)
        );
        let mut decoded: SimulationConfig =
            serde_json::from_slice(&fs::read(&compiled_path).unwrap()).unwrap();
        decoded.validate().unwrap();

        // Exercise the real autonomy CLI's file parser and startup without a
        // full race: this test deliberately ends after one control interval.
        // Field geometry and competition thresholds are left unchanged.
        decoded.max_duration_ms = decoded.time_step_ms;
        fs::write(&compiled_path, serde_json::to_vec_pretty(&decoded).unwrap()).unwrap();
        let run = field_command(
            "autonomy-sim",
            &compiled_path,
            &["--output", run_path.to_str().unwrap()],
        );
        assert!(!run.status.success());
        assert!(run.stdout.is_empty());
        let log = fs::read_to_string(&run_path).unwrap_or_else(|error| {
            panic!(
                "{name}: no simulation output: {error}; {}",
                String::from_utf8_lossy(&run.stderr)
            )
        });
        let summary: Value = serde_json::from_str(log.lines().last().unwrap()).unwrap();
        assert_eq!(summary["kind"], "simulation_summary", "{name}");
        assert_eq!(
            summary["fault"], "scenario duration exhausted before completion",
            "{name}: {summary}"
        );
        assert_eq!(summary["completed"], false);
        assert!(summary["ticks"].as_u64().unwrap() >= 1);

        // ULP-scale tolerance must not hide an independently edited route.
        decoded.autonomy.mission.cone_waypoints[1].y_m += 0.01;
        assert!(decoded.validate().is_err());
        fs::write(&compiled_path, serde_json::to_vec_pretty(&decoded).unwrap()).unwrap();
        let old_log = fs::read(&run_path).unwrap();
        let rejected = field_command(
            "autonomy-sim",
            &compiled_path,
            &["--output", run_path.to_str().unwrap()],
        );
        assert!(!rejected.status.success());
        assert!(
            String::from_utf8_lossy(&rejected.stderr).contains("compiled field geometry differs")
        );
        assert_eq!(fs::read(&run_path).unwrap(), old_log);
    }
}
