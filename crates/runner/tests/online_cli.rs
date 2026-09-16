use std::{fs, process::Command};
use xt_stcar_robot_runner::online_simulation::{OnlineScenario, controller_config};
use xt_stcar_robot_runner::simulation::SimulationConfig;

#[test]
fn online_cli_roundtrip_separates_scene_and_constant_controller() {
    let example = Command::new(env!("CARGO_BIN_EXE_xt-stcar-robot"))
        .arg("online-example")
        .output()
        .unwrap();
    assert!(example.status.success());
    let nominal: OnlineScenario = serde_json::from_slice(&example.stdout).unwrap();
    let mut moved = nominal.clone();
    moved.scene.spec.length_m = 8.0;
    moved.scene.spec.right_cone_from_right_m = 1.8;
    moved.scene.spec.light_front_x_m = 7.2;
    let temp = tempfile::tempdir().unwrap();
    let input = temp.path().join("scene.json");
    let output = temp.path().join("compiled.json");
    for scenario in [nominal, moved] {
        fs::write(&input, serde_json::to_vec(&scenario).unwrap()).unwrap();
        let result = Command::new(env!("CARGO_BIN_EXE_xt-stcar-robot"))
            .args(["online-compile", "--config"])
            .arg(&input)
            .arg("--output")
            .arg(&output)
            .output()
            .unwrap();
        assert!(
            result.status.success(),
            "{}",
            String::from_utf8_lossy(&result.stderr)
        );
        let config: SimulationConfig = serde_json::from_slice(&fs::read(&output).unwrap()).unwrap();
        config.validate().unwrap();
        assert!(config.field.is_none());
        assert_eq!(
            config.online_scene.as_ref().unwrap().bounds.max_x_m,
            scenario.scene.spec.length_m
        );
        assert_eq!(
            serde_json::to_value(config.autonomy).unwrap(),
            serde_json::to_value(controller_config()).unwrap()
        );
    }
}

#[test]
fn invalid_online_scene_and_output_alias_preserve_input_and_previous_output() {
    let temp = tempfile::tempdir().unwrap();
    let input = temp.path().join("scene.json");
    let output = temp.path().join("compiled.json");
    let valid = serde_json::to_value(OnlineScenario::example()).unwrap();
    let mut unknown = valid.clone();
    unknown["guessed_finish_m"] = 9.0.into();
    let mut bad_interval = valid.clone();
    bad_interval["occlusions"] = serde_json::json!([
        {"from_ms":300,"through_ms":200,"cones":true,"markers":false}
    ]);
    for invalid in [unknown, bad_interval] {
        let original = serde_json::to_vec(&invalid).unwrap();
        fs::write(&input, &original).unwrap();
        fs::write(&output, "previous valid output").unwrap();
        let result = Command::new(env!("CARGO_BIN_EXE_xt-stcar-robot"))
            .args(["online-compile", "--config"])
            .arg(&input)
            .arg("--output")
            .arg(&output)
            .output()
            .unwrap();
        assert!(!result.status.success());
        assert_eq!(fs::read(&input).unwrap(), original);
        assert_eq!(
            fs::read_to_string(&output).unwrap(),
            "previous valid output"
        );
    }
    let original = serde_json::to_vec(&valid).unwrap();
    fs::write(&input, &original).unwrap();
    let result = Command::new(env!("CARGO_BIN_EXE_xt-stcar-robot"))
        .args(["online-compile", "--config"])
        .arg(&input)
        .arg("--output")
        .arg(&input)
        .output()
        .unwrap();
    assert!(!result.status.success());
    assert_eq!(fs::read(&input).unwrap(), original);
}
