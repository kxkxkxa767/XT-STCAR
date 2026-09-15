//! Offline acceptance only. No planner or output path reads these budgets.
use serde::Deserialize;
use serde_json::{Value, json};

const BUDGET: &str = include_str!("../fixtures/motion-v9-performance-budget.json");

#[derive(Deserialize)]
struct Budget {
    runs: Vec<RunBudget>,
}

#[derive(Deserialize)]
struct RunBudget {
    tracker: String,
    input_delays_ms: [u64; 2],
    adoption_delays_ms: [u64; 3],
    maximum_elapsed_ms: u64,
    maximum_distance_m: f64,
    maximum_stop_duration_ms: u64,
    phases: Vec<PhaseBudget>,
}

#[derive(Deserialize)]
struct PhaseBudget {
    phase: String,
    maximum_duration_ms: u64,
}

fn bounded(value: &Value, maximum: f64) -> bool {
    value
        .as_f64()
        .is_some_and(|v| v.is_finite() && v >= 0.0 && v <= maximum)
}

fn check_run(run: &Value, budget: &RunBudget) -> Vec<String> {
    let mut failures = Vec::new();
    for (name, value, maximum) in [
        (
            "elapsed_ms",
            &run["summary"]["elapsed_ms"],
            budget.maximum_elapsed_ms as f64,
        ),
        (
            "distance_m",
            &run["summary"]["distance_m"],
            budget.maximum_distance_m,
        ),
        (
            "stop_duration_ms",
            &run["output_trace_counts"]["stop_duration_ms"],
            budget.maximum_stop_duration_ms as f64,
        ),
    ] {
        if !bounded(value, maximum) {
            failures.push(format!("{name}: {value}, maximum {maximum}"));
        }
    }
    let phases = run["summary"]["statistics"]["phases"].as_array();
    if phases.is_none_or(|p| p.len() != budget.phases.len()) {
        failures.push("missing or unexpected phase statistics".into());
    }
    for expected in &budget.phases {
        let matches: Vec<_> = phases
            .into_iter()
            .flatten()
            .filter(|p| p["phase"] == expected.phase)
            .collect();
        if matches.len() != 1
            || !bounded(
                &matches[0]["duration_ms"],
                expected.maximum_duration_ms as f64,
            )
        {
            failures.push(format!(
                "phase {}: missing, duplicate, invalid or over {} ms",
                expected.phase, expected.maximum_duration_ms
            ));
        }
    }
    failures
}

pub fn evaluate(runs: &Value) -> Value {
    let budget: Budget = serde_json::from_str(BUDGET).expect("checked-in performance budget");
    let mut reports = Vec::new();
    let mut all_passed = runs
        .as_array()
        .is_some_and(|r| r.len() == budget.runs.len());
    for expected in &budget.runs {
        let matches: Vec<_> = runs
            .as_array()
            .into_iter()
            .flatten()
            .filter(|run| {
                run["tracker"] == expected.tracker
                    && run["input_delays_ms"] == json!(expected.input_delays_ms)
                    && run["adoption_delays_ms"] == json!(expected.adoption_delays_ms)
            })
            .collect();
        let failures = if matches.len() == 1 {
            check_run(matches[0], expected)
        } else {
            vec!["missing or duplicate timing/tracker run".into()]
        };
        all_passed &= failures.is_empty();
        reports.push(json!({"tracker":expected.tracker, "input_delays_ms":expected.input_delays_ms,
            "adoption_delays_ms":expected.adoption_delays_ms, "passed":failures.is_empty(), "failures":failures}));
    }
    json!({"all_passed":all_passed, "budget":serde_json::from_str::<Value>(BUDGET).unwrap(), "runs":reports})
}

#[cfg(test)]
mod tests {
    use super::*;

    fn reference(b: &RunBudget) -> Value {
        json!({"tracker":b.tracker, "input_delays_ms":b.input_delays_ms, "adoption_delays_ms":b.adoption_delays_ms,
            "summary":{"elapsed_ms":b.maximum_elapsed_ms,"distance_m":b.maximum_distance_m,
                "statistics":{"phases":b.phases.iter().map(|p| json!({"phase":p.phase,"duration_ms":p.maximum_duration_ms})).collect::<Vec<_>>() }},
            "output_trace_counts":{"stop_duration_ms":b.maximum_stop_duration_ms}})
    }

    #[test]
    fn completed_v8_long_detour_fails_predeclared_budget() {
        let b: Budget = serde_json::from_str(BUDGET).unwrap();
        let mut run = reference(&b.runs[6]);
        run["summary"]["completed"] = json!(true);
        run["summary"]["elapsed_ms"] = json!(88_647);
        run["summary"]["distance_m"] = json!(16.51555838061635);
        run["output_trace_counts"]["stop_duration_ms"] = json!(4_513);
        let failures = check_run(&run, &b.runs[6]);
        assert_eq!(failures.len(), 2);
        assert!(failures[0].starts_with("elapsed_ms"));
        assert!(failures[1].starts_with("distance_m"));
    }

    #[test]
    fn missing_duplicate_and_invalid_metrics_cannot_pass_as_fast() {
        let b: Budget = serde_json::from_str(BUDGET).unwrap();
        let mut runs: Value = json!(b.runs.iter().map(reference).collect::<Vec<_>>());
        assert_eq!(evaluate(&runs)["all_passed"], true);
        runs[6] = runs[7].clone();
        assert_eq!(evaluate(&runs)["all_passed"], false);
        let mut run = reference(&b.runs[6]);
        run["summary"]["distance_m"] = Value::Null;
        run["summary"]["statistics"]["phases"][0]["duration_ms"] = json!(-1);
        assert_eq!(check_run(&run, &b.runs[6]).len(), 2);
    }

    #[test]
    fn faster_total_does_not_hide_a_slow_phase_or_long_stop() {
        let b: Budget = serde_json::from_str(BUDGET).unwrap();
        let mut run = reference(&b.runs[6]);
        run["summary"]["elapsed_ms"] = json!(50_000);
        run["summary"]["distance_m"] = json!(10.0);
        let light = b.runs[6]
            .phases
            .iter()
            .position(|p| p.phase == "approach_light")
            .unwrap();
        run["summary"]["statistics"]["phases"][light]["duration_ms"] = json!(30_000);
        run["output_trace_counts"]["stop_duration_ms"] =
            json!(b.runs[6].maximum_stop_duration_ms + 1);
        let failures = check_run(&run, &b.runs[6]);
        assert_eq!(failures.len(), 2);
        assert!(
            failures
                .iter()
                .any(|s| s.starts_with("phase approach_light"))
        );
        assert!(failures.iter().any(|s| s.starts_with("stop_duration_ms")));
    }
}
