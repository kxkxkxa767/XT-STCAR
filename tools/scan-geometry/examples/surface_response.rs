//! Fixed-PWM checks of the same plant used by surface_sim.
//! cargo run --release --manifest-path tools/scan-geometry/Cargo.toml --example surface_response -- CONFIG OUTPUT.json
#[path = "surface_sim/plant.rs"]
#[allow(dead_code)]
mod plant;
use plant::{Plant, Pose};
use serde_json::{Value, json};
use std::{env, fs};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = env::args().collect();
    if args.len() != 3 {
        return Err("usage: surface_response CONFIG OUTPUT.json".into());
    }
    let source: Value = serde_json::from_str(&fs::read_to_string(&args[1])?)?;
    let mut rows = Vec::new();
    for pwm in (1550..=1590).step_by(5).chain([1600]) {
        let mut previous_speed = f64::INFINITY;
        let mut previous_accel = f64::INFINITY;
        for scale in [1.0, 1.25, 1.5, 2.0] {
            let mut model = source["plant"].clone();
            model["resistance_scale"] = json!(scale);
            let origin = Pose {
                x: 0.,
                y: 0.,
                yaw: 0.,
            };
            let mut impulse_model = model.clone();
            impulse_model["motor_delay_s"] = json!(0.);
            let mut impulse = Plant::new(origin, &impulse_model);
            impulse.speed = 0.5;
            impulse.step(0.01, 0.01, pwm, 1500);
            let acceleration = (impulse.speed - 0.5) / 0.01;
            let mut constant = Plant::new(origin, &model);
            constant.speed = 0.5; // a rolling car, comparable across all PWM values
            for i in 1..=2500 {
                constant.step(i as f64 * 0.01, 0.01, pwm, 1500);
            }
            assert!(
                constant.speed <= previous_speed + 1e-8,
                "more resistance increased speed at {pwm}"
            );
            assert!(
                acceleration <= previous_accel + 1e-8,
                "more resistance increased acceleration at {pwm}"
            );
            previous_speed = constant.speed;
            previous_accel = acceleration;
            let before = constant.pose.x;
            let mut same_speed_coast = Plant::new(origin, &model);
            same_speed_coast.speed = 1.5;
            for i in 1..=5000 {
                same_speed_coast.step(i as f64 * 0.01, 0.01, 1500, 1500);
                if same_speed_coast.speed <= 1e-6 {
                    break;
                }
            }
            let mut stop_t = 0.;
            for i in 1..=5000 {
                stop_t = i as f64 * 0.01;
                constant.step(25. + stop_t, 0.01, 1500, 1500);
                if constant.speed <= 1e-6 {
                    break;
                }
            }
            rows.push(json!({"pwm":pwm,"resistance_scale":scale,"accel_from_0p5_mps":acceleration,
                "steady_speed_mps":previous_speed,"neutral_stop_distance_m":constant.pose.x-before,"neutral_stop_seconds":stop_t}));
            rows.last_mut().unwrap()["coast_distance_from_1p5_mps"] =
                json!(same_speed_coast.pose.x);
        }
    }
    let result = json!({"basis":"Model comparison at identical PWM and initial state; not vehicle measurements.",
        "config":source,"checks":{"rows":rows.len(),"higher_resistance_never_increases_speed_or_acceleration":true},"rows":rows});
    fs::write(&args[2], serde_json::to_string_pretty(&result)?)?;
    println!("{}", result["checks"]);
    Ok(())
}
