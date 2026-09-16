//! Three actual worker states around the first async small-field refusal.
//! This deliberately reproduces a failure; the full-race gate must still fail.
use super::*;
use serde_json::{Value, from_value, json};

fn fixture() -> Value {
    serde_json::from_str(include_str!("fixtures/online33260_first_rejection.json")).unwrap()
}

fn arrival(v: &Value) -> ArrivalBehavior {
    match v["type"].as_str().unwrap() {
        "stop" => ArrivalBehavior::Stop,
        "pass_through" => ArrivalBehavior::PassThrough {
            next: from_value(v["next"].clone()).unwrap(),
            next_heading_rad: v["next_heading_rad"].as_f64(),
            next_max_speed_mps: v["next_max_speed_mps"].as_f64().unwrap(),
            admission_radius_m: v["admission_radius_m"].as_f64().unwrap(),
        },
        _ => panic!("unknown arrival"),
    }
}

fn restore(s: &Value) -> Navigator {
    assert_eq!(s["target_policy"], "RollingLocal");
    let mut nav = Navigator::new_with_target_policy(
        from_value(s["config"].clone()).unwrap(),
        TargetPolicy::RollingLocal,
    )
    .unwrap();
    // Test-only HalfPlane Deserialize preserves the actual normal and support
    // interval; no atan2/sin/cos or synthetic rectangle changes the input.
    nav.travel_boundary = from_value(s["travel_boundary"].clone()).unwrap();
    nav.adoption_constraints = from_value(s["adoption_constraints"].clone()).unwrap();
    nav.recovery_active = s["recovery_active"].as_bool().unwrap();
    nav.recovery_route_error_m = s["recovery_route_error_m"].as_f64().unwrap();
    nav.recovery_sample_arc_bound_m = s["recovery_sample_arc_bound_m"].as_f64().unwrap();
    nav.last_step = s["last_step"].as_u64().map(Timestamp);
    let steering = &s["steering"];
    nav.steering = SteeringEstimate {
        at: Timestamp(steering["at"].as_u64().unwrap()),
        commanded_curvature_per_m: steering["commanded_curvature_per_m"].as_f64().unwrap(),
        applied_curvature_per_m: steering["applied_curvature_per_m"].as_f64().unwrap(),
    };
    nav.arrival = arrival(&s["arrival"]);
    nav.continuation_checked = s["continuation_checked"].as_bool().unwrap();
    nav.via_index = s["via_index"].as_u64().unwrap() as usize;
    nav.route_revision = s["route_revision"].as_u64().unwrap();
    nav.pending_route_rebuild = match s["pending_route_rebuild"].as_str() {
        None => None,
        Some("boundary_changed") => Some(RouteRebuildReason::BoundaryChanged),
        other => panic!("unexpected rebuild {other:?}"),
    };
    nav.route = from_value(s["route"].clone()).unwrap();
    nav.terminal_seed = (!s["terminal_seed"].is_null()).then(|| {
        let v = &s["terminal_seed"];
        CachedTerminalSeed {
            goal: from_value(v["goal"].clone()).unwrap(),
            heading: v["heading"].as_f64().unwrap(),
            seed: from_value(v["seed"].clone()).unwrap(),
            anchor_at: Timestamp(v["anchor_at"].as_u64().unwrap()),
            anchor_pose: from_value(v["anchor_pose"].clone()).unwrap(),
            prefer_continuation: v["prefer_continuation"].as_bool().unwrap(),
        }
    });
    // plan_with_arrival unconditionally replaces diagnostics and resets the
    // ledger before any of these valid inputs reaches a geometry/search gate.
    // Their complete prior values remain in the fixture as evidence, not fake
    // carried work. Timing is the one ledger setting that persists across reset.
    nav.set_timing_enabled(s["terminal_budget"]["timing_enabled"].as_bool().unwrap());
    nav
}

fn replay(nav: &mut Navigator, input: &Value) -> NavigationDecision {
    nav.plan_with_arrival(
        Timestamp(input["at"].as_u64().unwrap()),
        &from_value(input["estimate"].clone()).unwrap(),
        &from_value::<Vec<ObstacleDisc>>(input["obstacles"].clone()).unwrap(),
        Timestamp(input["obstacles_at"].as_u64().unwrap()),
        from_value(input["goal"].clone()).unwrap(),
        input["goal_heading_rad"].as_f64(),
        input["speed_limit_mps"].as_f64().unwrap(),
        arrival(&input["arrival"]),
    )
    .unwrap()
}

fn compare(actual: &Value, recorded: &Value, path: &str) {
    match (actual, recorded) {
        (Value::Object(a), Value::Object(b)) => {
            assert_eq!(a.len(), b.len(), "{path}");
            for (key, value) in a {
                compare(value, b.get(key).unwrap(), &format!("{path}/{key}"));
            }
        }
        (Value::Array(a), Value::Array(b)) => {
            assert_eq!(a.len(), b.len(), "{path}");
            for (i, (a, b)) in a.iter().zip(b).enumerate() {
                compare(a, b, &format!("{path}/{i}"));
            }
        }
        (Value::Number(a), Value::Number(b)) if a.is_f64() || b.is_f64() => {
            let (a, b) = (a.as_f64().unwrap(), b.as_f64().unwrap());
            assert!(
                (a - b).abs() <= 2e-12 * a.abs().max(b.abs()).max(1.0),
                "{path}: {a} != {b}"
            );
        }
        _ => assert_eq!(actual, recorded, "{path}"),
    }
}

#[test]
fn actual_three_frame_cache_replay_preserves_first_refusal_and_next_stop_state() {
    let data = fixture();
    for input in data["frames"].as_array().unwrap() {
        let mut nav = restore(&input["before_boundary"]);
        let before = &input["before_plan"];
        nav.set_travel_boundary(from_value(input["travel_boundary"].clone()).unwrap());
        assert_eq!(serde_json::to_value(&nav.route).unwrap(), before["route"]);
        assert!(nav.terminal_seed.is_none());
        assert!(!nav.recovery_active);
        assert_eq!(
            nav.pending_route_rebuild,
            Some(RouteRebuildReason::BoundaryChanged)
        );
        let mut steering = nav.execution_state();
        steering
            .advance_to(
                Timestamp(input["at"].as_u64().unwrap()),
                nav.config.max_curvature_rate_per_s,
            )
            .unwrap();
        let actual = replay(&mut nav, input);
        compare(
            &serde_json::to_value(&actual).unwrap(),
            &input["decision"],
            "decision",
        );
        assert_eq!(
            nav.execution_state(),
            steering,
            "planning must not adopt output"
        );
        assert_eq!(
            nav.route_revision,
            input["after_plan"]["route_revision"].as_u64().unwrap()
        );
        compare(
            &serde_json::to_value(&nav.route).unwrap(),
            &input["after_plan"]["route"],
            "route",
        );
        if input["at"] == 33260 {
            assert_eq!(actual.reason.as_deref(), Some("no_forward_kinematic_path"));
            assert_eq!(actual.diagnostics.candidates.rollouts_evaluated, 0);
            let search = actual.diagnostics.forward_search.recovery;
            assert_eq!(
                (
                    search.allocated_nodes,
                    search.expanded_nodes,
                    search.primitive_attempts
                ),
                (6, 5, 25)
            );
            assert_eq!(search.exit, recovery::ForwardSearchExit::OpenEmpty);
            let work = actual.diagnostics.terminal_work;
            assert_eq!(
                (
                    work.solver_attempts,
                    work.iterations,
                    work.primitive_samples
                ),
                (2, 2, 48)
            );
            assert!(!work.budget_exhausted);
            let adoption = nav.adoption_constraints.unwrap();
            let dt = input["hint_dt_s"].as_f64().unwrap();
            let projected = input["estimate"]["speed_mps"].as_f64().unwrap();
            assert!(
                input["speed_limit_mps"].as_f64().unwrap()
                    > projected - nav.config.max_decel_mps2 * dt
            );
            assert_eq!(adoption.planned_at, Timestamp(33260));
        }
    }
}

#[test]
fn first_refusal_candidates_fail_finite_boundary_even_without_chord_tube_padding() {
    let data = fixture();
    let input = &data["frames"][1];
    let nav = restore(&input["before_plan"]);
    let obstacles: Vec<ObstacleDisc> = from_value(input["obstacles"].clone()).unwrap();
    let mut grid = Grid::with_boundary(&nav.config, &obstacles, nav.travel_boundary);
    grid.recovery.oriented_boundary = true;
    grid.enable_recovery();
    let no_boundary = Grid::new(&nav.config, &obstacles);
    no_boundary.enable_recovery();
    let period = nav.config.control_period_ms as f64 / 1000.0;
    let restart_speed = 0.5
        * nav
            .config
            .max_speed_mps
            .min(nav.config.max_accel_mps2 * period);
    let base_padding = nav.config.clearance_m
        + 2.0 * period * restart_speed
        + restart_speed.powi(2) / (2.0 * nav.config.max_decel_mps2);
    let mut count = 0;
    let mut below_clearance = 0;
    for event in input["events"]
        .as_array()
        .unwrap()
        .iter()
        .filter(|e| e["kind"] == "motion_rejected")
    {
        count += 1;
        let from: Pose2 = from_value(event["from"].clone()).unwrap();
        let to: Pose2 = from_value(event["to"].clone()).unwrap();
        let length = event["length"].as_f64().unwrap();
        let error = primitive_envelope::ErrorBound {
            position_m: event["error"][0].as_f64().unwrap(),
            heading_rad: event["error"][1].as_f64().unwrap(),
        };
        assert!(no_boundary.motion_transition_clear_with_error(
            from.point(),
            to.point(),
            length,
            error.position_m
        ));
        assert!(!grid.oriented_motion_transition_clear(from, to, length, error));
        let corners: Vec<Point2> = nav
            .config
            .footprint
            .corners(from)
            .into_iter()
            .chain(nav.config.footprint.corners(to))
            .collect();
        let margin = nav
            .travel_boundary
            .unwrap()
            .signed_points_margin(&corners, 0.0);
        assert!(
            margin < base_padding,
            "even zero tube padding cannot certify this candidate"
        );
        if margin < nav.config.clearance_m {
            below_clearance += 1;
        }
        assert_eq!(event["bounds_and_obstacles"], json!(true));
    }
    // Two terminal primitive failures plus fifteen lattice primitive failures.
    assert_eq!((count, below_clearance), (17, 15));
    // This says nothing about a different candidate or global physical reachability.
}
