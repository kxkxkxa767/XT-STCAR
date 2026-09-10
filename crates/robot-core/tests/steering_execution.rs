use xt_stcar_robot_core::autonomy::{Footprint, Point2, Pose2, PoseEstimate, Rect};
use xt_stcar_robot_core::navigation::{
    ArrivalBehavior, NavigationConfig, NavigationDecision, NavigationStatus, Navigator,
    SteeringEstimate,
};
use xt_stcar_robot_core::{FrameId, MotionOutput, Timestamp};

fn config() -> NavigationConfig {
    let mut config = NavigationConfig::simulation(
        Rect {
            min_x_m: 0.0,
            min_y_m: 0.0,
            max_x_m: 20.0,
            max_y_m: 20.0,
        },
        Footprint {
            front_m: 0.22,
            rear_m: 0.18,
            half_width_m: 0.13,
        },
        FrameId("map".into()),
    );
    config.control_period_ms = 100;
    config.preview_horizon_s = 0.1;
    config
}

fn estimate(at: u64, speed_mps: f64) -> PoseEstimate {
    PoseEstimate {
        captured_at: Timestamp(at),
        frame_id: FrameId("map".into()),
        pose: Pose2 {
            x_m: 5.0,
            y_m: 5.0,
            yaw_rad: 0.0,
        },
        speed_mps,
        yaw_rate_radps: 0.0,
        quality: 1.0,
    }
}

fn plan(nav: &mut Navigator, at: u64, speed: f64, goal: Point2) -> NavigationDecision {
    nav.plan_with_arrival(
        Timestamp(at),
        &estimate(at, speed),
        &[],
        Timestamp(at),
        goal,
        None,
        0.3,
        ArrivalBehavior::Stop,
    )
    .unwrap()
}

fn held_turn(sign: f64) -> SteeringEstimate {
    SteeringEstimate {
        at: Timestamp(0),
        commanded_curvature_per_m: sign * 2.0,
        applied_curvature_per_m: sign * 2.0,
    }
}

#[test]
fn adopted_stop_returns_steering_gradually_in_both_directions() {
    for sign in [-1.0, 1.0] {
        let mut steering = held_turn(sign);
        steering
            .adopt(Timestamp(0), &MotionOutput::Stop, 4.0, 2.0)
            .unwrap();
        assert_eq!(steering.applied_curvature_per_m, sign * 2.0);
        assert_eq!(steering.commanded_curvature_per_m, 0.0);
        steering.advance_to(Timestamp(100), 4.0).unwrap();
        assert!((steering.applied_curvature_per_m - sign * 1.6).abs() < 1e-12);

        let unchanged = steering;
        steering.advance_to(Timestamp(100), 4.0).unwrap();
        steering
            .adopt(Timestamp(100), &MotionOutput::Stop, 4.0, 2.0)
            .unwrap();
        assert_eq!(steering, unchanged, "same stamp must not invent motion");

        steering.advance_to(Timestamp(500), 4.0).unwrap();
        assert_eq!(steering.applied_curvature_per_m, 0.0);
        steering.advance_to(Timestamp(60_000), 4.0).unwrap();
        assert_eq!(steering.applied_curvature_per_m, 0.0);
        assert_eq!(steering.at, Timestamp(60_000));
    }
}

#[test]
fn invalid_adoption_or_time_does_not_partially_advance_the_estimate() {
    let mut extreme = SteeringEstimate {
        at: Timestamp(0),
        commanded_curvature_per_m: f64::MAX,
        applied_curvature_per_m: -f64::MAX,
    };
    let unchanged = extreme;
    assert!(extreme.advance_to(Timestamp(100), f64::MAX).is_err());
    assert_eq!(
        extreme, unchanged,
        "unrepresentable finite transition must reject atomically"
    );
    let initial = SteeringEstimate {
        at: Timestamp(100),
        commanded_curvature_per_m: 0.0,
        applied_curvature_per_m: 1.6,
    };
    for (at, command, rate, limit) in [
        (99, MotionOutput::Stop, 4.0, 2.0),
        (200, MotionOutput::Stop, 0.0, 2.0),
        (200, MotionOutput::Stop, f64::NAN, 2.0),
        (200, MotionOutput::Stop, 4.0, f64::INFINITY),
        (200, MotionOutput::Stop, 4.0, 1.0),
        (
            200,
            MotionOutput::Drive {
                speed_mps: -0.1,
                curvature_per_m: 0.0,
            },
            4.0,
            2.0,
        ),
        (
            200,
            MotionOutput::Drive {
                speed_mps: f64::NAN,
                curvature_per_m: 0.0,
            },
            4.0,
            2.0,
        ),
        (
            200,
            MotionOutput::Drive {
                speed_mps: 0.1,
                curvature_per_m: f64::NAN,
            },
            4.0,
            2.0,
        ),
        (
            200,
            MotionOutput::Drive {
                speed_mps: 0.1,
                curvature_per_m: 2.1,
            },
            4.0,
            2.0,
        ),
    ] {
        let mut steering = initial;
        assert!(
            steering
                .adopt(Timestamp(at), &command, rate, limit)
                .is_err()
        );
        assert_eq!(steering, initial);
    }
    for (at, rate) in [(99, 4.0), (200, -1.0), (200, f64::INFINITY)] {
        let mut steering = initial;
        assert!(steering.advance_to(Timestamp(at), rate).is_err());
        assert_eq!(steering, initial);
    }
}

#[test]
fn planning_changes_no_adopted_target_and_uses_the_owners_actual_acknowledgement() {
    let mut nav = Navigator::new(config()).unwrap();
    let goal = Point2 { x_m: 8.0, y_m: 6.0 };
    let first = plan(&mut nav, 0, 0.3, goal);
    assert_eq!(
        first.status,
        NavigationStatus::Driving,
        "{:?}",
        first.reason
    );
    assert!(first.intent.curvature_per_m > 0.0);
    assert_eq!(
        nav.execution_state(),
        SteeringEstimate::stationary(Timestamp(0))
    );

    // The proposal is discarded. A second plan still follows the last adopted
    // zero steering, even though the previous proposal requested a turn.
    let discarded = plan(&mut nav, 100, 0.3, goal);
    assert_eq!(discarded.status, NavigationStatus::Driving);
    let unacknowledged = SteeringEstimate::stationary(Timestamp(100));
    assert_eq!(nav.execution_state(), unacknowledged);
    assert_eq!(discarded.diagnostics.execution_state, Some(unacknowledged));

    // The output owner may adopt a different final command after its own
    // arbitration. The next plan must follow that command, not either proposal.
    nav.adopt_command(
        Timestamp(100),
        &MotionOutput::Drive {
            speed_mps: 0.2,
            curvature_per_m: -0.4,
        },
    )
    .unwrap();
    assert_eq!(nav.execution_state().applied_curvature_per_m, 0.0);
    let next = plan(&mut nav, 150, 0.3, goal);
    let state = nav.execution_state();
    assert_eq!(state.commanded_curvature_per_m, -0.4);
    assert!((state.applied_curvature_per_m + 0.2).abs() < 1e-12);
    assert_eq!(next.diagnostics.execution_state, Some(state));
}

#[test]
fn synchronous_wrappers_acknowledge_the_command_without_instant_steering_motion() {
    let mut nav = Navigator::new(config()).unwrap();
    let first = nav
        .step(
            Timestamp(0),
            &estimate(0, 0.3),
            &[],
            Timestamp(0),
            Point2 { x_m: 8.0, y_m: 6.0 },
        )
        .unwrap();
    assert_eq!(first.status, NavigationStatus::Driving);
    assert!(first.intent.curvature_per_m > 0.0);
    assert_eq!(
        nav.execution_state().commanded_curvature_per_m,
        first.intent.curvature_per_m
    );
    assert_eq!(nav.execution_state().applied_curvature_per_m, 0.0);
    let stop = nav.stop(Timestamp(100)).unwrap();
    assert_eq!(stop.status, NavigationStatus::Blocked);
    assert_eq!(nav.execution_state().commanded_curvature_per_m, 0.0);
    assert!(
        (nav.execution_state().applied_curvature_per_m - first.intent.curvature_per_m).abs()
            < 1e-12
    );
    assert!(
        stop.diagnostics
            .execution_state
            .unwrap()
            .applied_curvature_per_m
            > 0.0
    );
}

#[test]
fn stop_blocked_braking_and_reached_decisions_preserve_nonzero_execution_history() {
    for sign in [-1.0, 1.0] {
        for outcome in [
            "task_stop",
            "measured_speed_outside_forward_limits",
            "goal_braking",
            "goal_reached",
        ] {
            let mut nav = Navigator::new(config()).unwrap();
            nav.set_execution_state(held_turn(sign)).unwrap();
            nav.adopt_command(Timestamp(0), &MotionOutput::Stop)
                .unwrap();
            let decision = match outcome {
                "task_stop" => nav.plan_stop(Timestamp(100)).unwrap(),
                "measured_speed_outside_forward_limits" => {
                    plan(&mut nav, 100, 0.4, Point2 { x_m: 8.0, y_m: 5.0 })
                }
                "goal_braking" => plan(&mut nav, 100, 0.1, estimate(100, 0.1).pose.point()),
                "goal_reached" => plan(&mut nav, 100, 0.0, estimate(100, 0.0).pose.point()),
                _ => unreachable!(),
            };
            assert_eq!(decision.reason.as_deref(), Some(outcome));
            assert_eq!(decision.intent.speed_mps, 0.0);
            assert_eq!(decision.intent.curvature_per_m, 0.0);
            assert_eq!(
                decision.status,
                if outcome == "goal_reached" {
                    NavigationStatus::Reached
                } else {
                    NavigationStatus::Blocked
                }
            );
            let state = nav.execution_state();
            assert!((state.applied_curvature_per_m - sign * 1.6).abs() < 1e-12);
            assert_eq!(state.commanded_curvature_per_m, 0.0);
            assert_eq!(
                decision.diagnostics.execution_state,
                Some(state),
                "{outcome}"
            );
        }
    }
}

#[test]
fn selected_prediction_starts_from_remaining_steering_after_a_short_stop() {
    for sign in [-1.0, 1.0] {
        let mut nav = Navigator::new(config()).unwrap();
        nav.set_execution_state(held_turn(sign)).unwrap();
        nav.adopt_command(Timestamp(0), &MotionOutput::Stop)
            .unwrap();
        let decision = plan(
            &mut nav,
            100,
            0.3,
            Point2 {
                x_m: 10.0,
                y_m: 5.0,
            },
        );
        assert_eq!(
            decision.status,
            NavigationStatus::Driving,
            "{:?}",
            decision.reason
        );
        let state = decision.diagnostics.execution_state.unwrap();
        assert_eq!(state.commanded_curvature_per_m, 0.0);
        assert!((state.applied_curvature_per_m - sign * 1.6).abs() < 1e-12);
        assert!(decision.intent.curvature_per_m.abs() <= 0.4 + 1e-12);

        // All candidates are in [-.4, .4], but the actual model starts at ±1.6
        // and can move only .4 toward them in the .1-second preview. Its mean
        // curvature therefore remains in ±[1.2, 1.6]. Resetting to the previous
        // zero command would keep this yaw/distance ratio below .4 instead.
        let predicted = decision.diagnostics.selected_prediction_endpoint.unwrap();
        let distance = decision.diagnostics.selected_prediction_distance_m.unwrap();
        assert!(distance > 0.02);
        let mean_curvature = sign * predicted.yaw_rad / distance;
        assert!(
            (1.2 - 1e-12..=1.6 + 1e-12).contains(&mean_curvature),
            "{decision:?}"
        );
        assert_eq!(
            nav.execution_state(),
            state,
            "a proposal is not an acknowledgement"
        );
    }
}

#[test]
fn externally_supplied_execution_states_cannot_rewind_or_corrupt_history() {
    let mut nav = Navigator::new(config()).unwrap();
    nav.adopt_command(Timestamp(100), &MotionOutput::Stop)
        .unwrap();
    let initial = nav.execution_state();
    for state in [
        SteeringEstimate::stationary(Timestamp(99)),
        SteeringEstimate {
            at: Timestamp(200),
            applied_curvature_per_m: 2.1,
            ..initial
        },
        SteeringEstimate {
            at: Timestamp(200),
            commanded_curvature_per_m: f64::NAN,
            ..initial
        },
        SteeringEstimate {
            at: Timestamp(200),
            applied_curvature_per_m: f64::INFINITY,
            ..initial
        },
    ] {
        assert!(nav.set_execution_state(state).is_err());
        assert_eq!(nav.execution_state(), initial);
    }
    let authoritative = SteeringEstimate {
        at: Timestamp(150),
        commanded_curvature_per_m: -0.4,
        applied_curvature_per_m: -0.2,
    };
    nav.set_execution_state(authoritative).unwrap();
    assert_eq!(nav.execution_state(), authoritative);
    assert!(nav.plan_stop(Timestamp(149)).is_err());
    assert_eq!(nav.execution_state(), authoritative);
}
