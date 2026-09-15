//! A cached terminal seed is a numerical hint, never a path certificate.
//! Rebase only after the original finite solvers fail, and recheck every sample.
use super::*;

#[derive(Clone, Copy, Debug)]
pub(super) struct CachedTerminalSeed {
    pub(super) goal: Point2,
    pub(super) heading: f64,
    pub(super) seed: TwoArcSeed,
    /// The selected candidate's predicted next state, not an adopted timestamp.
    pub(super) anchor_at: Timestamp,
    pub(super) anchor_pose: Pose2,
    /// Set only after the original cold and unadjusted warm solves failed and
    /// an adjusted hint recovered a certified connection. Ordinary paths stay cold-first.
    pub(super) prefer_continuation: bool,
}
impl CachedTerminalSeed {
    pub(super) fn for_pose(self, now: Timestamp, pose: Pose2) -> Option<TwoArcSeed> {
        if !pose.valid() || !self.anchor_pose.valid() {
            return None;
        }
        if now <= self.anchor_at {
            return Some(self.seed);
        }
        // This longitudinal displacement adjusts an initial guess; it is not
        // an odometry/arc-length measurement. A fresh solve certifies the result.
        let forward = self.anchor_pose.world_to_body(pose.point()).x_m;
        if !forward.is_finite() {
            return None;
        }
        if forward <= 0.0 {
            return Some(self.seed);
        }
        self.seed.after_travel(forward)
    }
}

#[allow(clippy::too_many_arguments)]
pub(super) fn terminal_connection(
    config: &NavigationConfig,
    budget: &TerminalBudget,
    cached: Option<CachedTerminalSeed>,
    now: Option<Timestamp>,
    start: Pose2,
    initial_curvature: f64,
    goal: Point2,
    heading: f64,
    grid: &Grid,
    error: primitive_envelope::ErrorBound,
) -> Option<(terminal::TwoArcConnection, bool)> {
    let hint = cached.filter(|hint| hint.goal == goal && hint.heading == heading);
    let rebased = hint.and_then(|hint| now.and_then(|at| hint.for_pose(at, start)));
    if hint.is_some_and(|h| h.prefer_continuation)
        && let Some(seed) = rebased
        && let Some(connection) = terminal::continue_two_arc_with_error(
            start,
            initial_curvature,
            goal,
            heading,
            config,
            grid,
            budget,
            seed,
            error,
        )
    {
        return Some((connection, true));
    }
    if let Some(connection) = terminal::two_arc_with_error(
        start,
        initial_curvature,
        goal,
        heading,
        config,
        grid,
        budget,
        hint.map(|h| h.seed),
        error,
    ) {
        let keep_family = hint.is_some_and(|h| h.prefer_continuation) && connection.used_seed;
        return Some((connection, keep_family));
    }
    if budget.snapshot().budget_exhausted
        || hint.is_none_or(|h| {
            h.prefer_continuation
                || now.is_none_or(|at| at <= h.anchor_at)
                || h.anchor_pose.world_to_body(start.point()).x_m <= 0.0
        })
    {
        return None;
    }
    let connection = terminal::continue_two_arc_with_error(
        start,
        initial_curvature,
        goal,
        heading,
        config,
        grid,
        budget,
        rebased?,
        error,
    )?;
    Some((connection, true))
}

/// Same local progress rule used by the tracker; no shortcut across route loops.
pub(super) fn remaining_length(pose: Pose2, route: &[Point2], progress: usize) -> Option<f64> {
    let mut nearest = progress;
    let mut distance = pose.point().distance(*route.get(progress)?);
    let mut travel = 0.0;
    for index in (progress + 1)..route.len() {
        travel += route[index - 1].distance(route[index]);
        if travel > 1.0 {
            break;
        }
        let candidate = pose.point().distance(route[index]);
        if candidate < distance {
            nearest = index;
            distance = candidate;
        }
    }
    let next = (nearest + 1).min(route.len() - 1);
    let length = pose.point().distance(route[next])
        + route[next..]
            .windows(2)
            .map(|p| p[0].distance(p[1]))
            .sum::<f64>();
    length.is_finite().then_some(length)
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::{Value, from_value, json};

    fn fixture() -> Value {
        serde_json::from_str(include_str!("fixtures/lqr45360_terminal_seed_origin.json")).unwrap()
    }

    fn boundary(value: &Value) -> HalfPlane {
        HalfPlane::new(
            from_value(value["origin"].clone()).unwrap(),
            value["normal"]["y_m"]
                .as_f64()
                .unwrap()
                .atan2(value["normal"]["x_m"].as_f64().unwrap()),
            value["max_projection_m"].as_f64().unwrap(),
        )
        .unwrap()
    }

    fn hint(value: &Value, chosen: &Value) -> CachedTerminalSeed {
        CachedTerminalSeed {
            goal: from_value(value["goal"].clone()).unwrap(),
            heading: value["heading"].as_f64().unwrap(),
            seed: from_value(value["seed"].clone()).unwrap(),
            anchor_at: Timestamp(chosen["now"].as_u64().unwrap() + 100),
            anchor_pose: from_value(chosen["next"]["pose"].clone()).unwrap(),
            prefer_continuation: false,
        }
    }

    #[test]
    fn actual_120ms_state_recovers_only_after_original_seed_failed_and_rechecks_both_sides() {
        let data = fixture();
        let event = &data["navigation"][1];
        assert_eq!(event["now"], 45_160);
        let config: NavigationConfig = from_value(data["config"].clone()).unwrap();
        for mirror in [false, true] {
            let mut pose: Pose2 = from_value(event["estimate"]["pose"].clone()).unwrap();
            let mut cached = hint(&event["before_seed"], &data["chosen_next"][0]);
            let mut obstacles: Vec<ObstacleDisc> = from_value(event["obstacles"].clone()).unwrap();
            let mut k = event["before_steering"]["applied_curvature_per_m"]
                .as_f64()
                .unwrap();
            let mut edge = event["travel_boundary"].clone();
            if mirror {
                pose.y_m = 5.0 - pose.y_m;
                pose.yaw_rad = -pose.yaw_rad;
                k = -k;
                cached.anchor_pose.y_m = 5.0 - cached.anchor_pose.y_m;
                cached.anchor_pose.yaw_rad = -cached.anchor_pose.yaw_rad;
                cached.goal.y_m = 5.0 - cached.goal.y_m;
                let mut seed = serde_json::to_value(cached.seed).unwrap();
                for index in 0..2 {
                    seed["variables"][index] = json!(-seed["variables"][index].as_f64().unwrap());
                }
                cached.seed = from_value(seed).unwrap();
                for obstacle in &mut obstacles {
                    obstacle.center.y_m = 5.0 - obstacle.center.y_m;
                }
                edge["origin"]["y_m"] = json!(5.0 - edge["origin"]["y_m"].as_f64().unwrap());
                edge["normal"]["y_m"] = json!(-edge["normal"]["y_m"].as_f64().unwrap());
            }
            let grid = Grid::with_boundary(&config, &obstacles, Some(boundary(&edge)));
            let original = TerminalBudget::default();
            assert!(
                terminal::two_arc(
                    pose,
                    k,
                    cached.goal,
                    0.0,
                    &config,
                    &grid,
                    &original,
                    Some(cached.seed)
                )
                .is_none()
            );
            assert!(!original.snapshot().budget_exhausted);
            let budget = TerminalBudget::default();
            let (connection, preserve) = terminal_connection(
                &config,
                &budget,
                Some(cached),
                Some(Timestamp(45_160)),
                pose,
                k,
                cached.goal,
                0.0,
                &grid,
                primitive_envelope::ErrorBound::default(),
            )
            .unwrap();
            assert!(preserve && connection.used_seed);
            assert!(connection.endpoint.point().distance(cached.goal) < config.goal_tolerance_m);
            assert!(
                angle_error(connection.endpoint.yaw_rad, 0.0).abs()
                    <= config.goal_heading_tolerance_rad
            );
            assert!(
                connection
                    .points
                    .windows(2)
                    .all(|p| grid.transition_clear(p[0], p[1]))
            );
            assert!(
                budget.snapshot().iterations <= 1024
                    && budget.snapshot().primitive_samples <= 65_536
            );
        }
    }

    #[test]
    fn early_negative_invalid_and_exhausted_displacements_do_not_invent_travel() {
        let data = fixture();
        let event = &data["navigation"][1];
        let cached = hint(&event["before_seed"], &data["chosen_next"][0]);
        let original = serde_json::to_value(cached.seed).unwrap();
        let pose: Pose2 = from_value(event["estimate"]["pose"].clone()).unwrap();
        assert_eq!(
            serde_json::to_value(cached.for_pose(Timestamp(45_120), pose).unwrap()).unwrap(),
            original
        );
        let behind = cached.anchor_pose.body_to_world(Point2 {
            x_m: -0.1,
            y_m: 0.0,
        });
        assert_eq!(
            serde_json::to_value(
                cached
                    .for_pose(
                        Timestamp(45_160),
                        Pose2 {
                            x_m: behind.x_m,
                            y_m: behind.y_m,
                            yaw_rad: pose.yaw_rad
                        }
                    )
                    .unwrap()
            )
            .unwrap(),
            original
        );
        assert!(
            cached
                .for_pose(
                    Timestamp(45_160),
                    Pose2 {
                        x_m: f64::NAN,
                        ..pose
                    }
                )
                .is_none()
        );
        let far = cached
            .anchor_pose
            .body_to_world(Point2 { x_m: 2.0, y_m: 0.0 });
        assert!(
            cached
                .for_pose(
                    Timestamp(45_160),
                    Pose2 {
                        x_m: far.x_m,
                        y_m: far.y_m,
                        yaw_rad: pose.yaw_rad
                    }
                )
                .is_none()
        );
    }

    #[test]
    fn new_obstacles_boundary_and_shared_exhaustion_reject_old_hint() {
        let data = fixture();
        let event = &data["navigation"][1];
        let config: NavigationConfig = from_value(data["config"].clone()).unwrap();
        let pose: Pose2 = from_value(event["estimate"]["pose"].clone()).unwrap();
        let cached = hint(&event["before_seed"], &data["chosen_next"][0]);
        let k = event["before_steering"]["applied_curvature_per_m"]
            .as_f64()
            .unwrap();
        let obstacles: Vec<ObstacleDisc> = from_value(event["obstacles"].clone()).unwrap();
        let edge = Some(boundary(&event["travel_boundary"]));
        let grid = Grid::with_boundary(&config, &obstacles, edge);
        let (connection, _) = terminal_connection(
            &config,
            &TerminalBudget::default(),
            Some(cached),
            Some(Timestamp(45_160)),
            pose,
            k,
            cached.goal,
            0.0,
            &grid,
            primitive_envelope::ErrorBound::default(),
        )
        .unwrap();
        let mut changed = obstacles.clone();
        changed.push(ObstacleDisc {
            center: connection.points[connection.points.len() * 3 / 4],
            radius_m: 0.04,
        });
        for blocked in [
            Grid::with_boundary(&config, &changed, edge),
            Grid::with_boundary(
                &config,
                &obstacles,
                Some(HalfPlane::new(pose.point(), pose.yaw_rad, 0.32).unwrap()),
            ),
        ] {
            assert!(
                terminal_connection(
                    &config,
                    &TerminalBudget::default(),
                    Some(cached),
                    Some(Timestamp(45_160)),
                    pose,
                    k,
                    cached.goal,
                    0.0,
                    &blocked,
                    primitive_envelope::ErrorBound::default()
                )
                .is_none()
            );
        }
        let budget = TerminalBudget::default();
        for _ in 0..257 {
            let _ = terminal::continue_two_arc(
                pose,
                k,
                cached.goal,
                0.0,
                &config,
                &grid,
                &budget,
                cached.seed,
            );
            if budget.snapshot().budget_exhausted {
                break;
            }
        }
        assert!(budget.snapshot().budget_exhausted);
        let before = budget.snapshot();
        assert!(
            terminal_connection(
                &config,
                &budget,
                Some(cached),
                Some(Timestamp(45_160)),
                pose,
                k,
                cached.goal,
                0.0,
                &grid,
                primitive_envelope::ErrorBound::default()
            )
            .is_none()
        );
        assert_eq!(
            budget.snapshot().primitive_samples,
            before.primitive_samples
        );
        assert!(budget.snapshot().solver_attempts <= 256 && budget.snapshot().iterations <= 1024);
    }

    #[test]
    fn recorded_worker_history_drives_navigation_without_synthetic_clock() {
        let data = fixture();
        let event = &data["navigation"][1];
        let input = &data["worker_inputs"][1];
        let config: NavigationConfig = from_value(data["config"].clone()).unwrap();
        let constraints: AdoptionConstraints = from_value(event["constraints"].clone()).unwrap();
        let records = input["history"]["records"].as_array().unwrap();
        assert_eq!(records.len(), 128);
        assert_eq!(
            input["history"]["revision"].as_u64().unwrap(),
            constraints.adopted_revision
        );
        assert_eq!(
            records.last().unwrap()["steering"]["at"]
                .as_u64()
                .map(Timestamp),
            constraints.last_command_change_at
        );
        for record in records {
            let command = data["actual_command_changes"]
                .as_array()
                .unwrap()
                .iter()
                .find(|c| c["at"] == record["steering"]["at"])
                .unwrap();
            if record["stopped"] == true {
                assert_eq!(command["command"]["type"], "stop");
            } else {
                assert_eq!(command["command"]["speed_mps"], record["speed_target"]);
                assert_eq!(
                    command["command"]["curvature_per_m"],
                    record["steering"]["commanded_curvature_per_m"]
                );
            }
        }
        let obstacles: Vec<ObstacleDisc> = from_value(event["obstacles"].clone()).unwrap();
        let estimate: PoseEstimate = from_value(event["estimate"].clone()).unwrap();
        let mut nav = Navigator::new(config.clone()).unwrap();
        nav.set_travel_boundary(Some(boundary(&event["travel_boundary"])));
        nav.set_execution_state(SteeringEstimate {
            at: Timestamp(45_160),
            applied_curvature_per_m: event["before_steering"]["applied_curvature_per_m"]
                .as_f64()
                .unwrap(),
            commanded_curvature_per_m: event["before_steering"]["commanded_curvature_per_m"]
                .as_f64()
                .unwrap(),
        })
        .unwrap();
        nav.last_step = event["before_last_step"].as_u64().map(Timestamp);
        nav.route = from_value(event["before_route"].clone()).unwrap();
        nav.via_index = event["before_via"].as_u64().unwrap() as usize;
        nav.route_revision = event["before_revision"].as_u64().unwrap();
        nav.pending_route_rebuild = None;
        nav.terminal_seed = Some(hint(&event["before_seed"], &data["chosen_next"][0]));
        nav.set_adoption_constraints(Some(constraints));
        let decision = nav
            .plan_with_arrival(
                Timestamp(45_160),
                &estimate,
                &obstacles,
                Timestamp(event["obstacles_at"].as_u64().unwrap()),
                from_value(event["goal"].clone()).unwrap(),
                Some(0.0),
                0.18,
                ArrivalBehavior::Stop,
            )
            .unwrap();
        assert!(decision.intent.speed_mps > 0.0, "{decision:?}");
        assert!(nav.terminal_seed.is_some_and(|s| s.prefer_continuation));
        constraints
            .check_command(
                &config,
                decision.intent.speed_mps,
                decision.intent.curvature_per_m,
                &obstacles,
                nav.travel_boundary,
            )
            .unwrap();
    }

    #[test]
    fn same_current_pose_old_remaining_length_reproduces_material_route_growth() {
        let data = fixture();
        let e = &data["navigation"][3];
        assert_eq!(e["now"], 45_360);
        let pose: Pose2 = from_value(e["estimate"]["pose"].clone()).unwrap();
        let route: (Point2, Option<f64>, Vec<Point2>, usize) =
            from_value(e["before_route"].clone()).unwrap();
        let old = remaining_length(
            pose,
            &route.2[..=e["before_via"].as_u64().unwrap() as usize],
            route.3,
        )
        .unwrap();
        let new: Vec<Point2> = from_value(e["result"]["path"].clone()).unwrap();
        let length: f64 = new.windows(2).map(|p| p[0].distance(p[1])).sum();
        assert!((old - 0.5028645756577586).abs() < 1e-12);
        assert!((length - 4.910317091730606).abs() < 1e-12);
        assert_eq!(route.0, from_value::<Point2>(e["goal"].clone()).unwrap());
    }

    #[test]
    fn unadjusted_warm_success_keeps_an_already_recovered_family() {
        let data: Value =
            serde_json::from_str(include_str!("fixtures/lqr45180_continued_family.json")).unwrap();
        let e = &data["navigation"];
        let config: NavigationConfig = from_value(data["config"].clone()).unwrap();
        let pose: Pose2 = from_value(e["estimate"]["pose"].clone()).unwrap();
        let seed = &e["before_seed"];
        let cached = CachedTerminalSeed {
            goal: from_value(seed["goal"].clone()).unwrap(),
            heading: 0.0,
            seed: from_value(seed["seed"].clone()).unwrap(),
            anchor_at: Timestamp(seed["anchor_at"].as_u64().unwrap()),
            anchor_pose: from_value(seed["anchor_pose"].clone()).unwrap(),
            prefer_continuation: true,
        };
        let obstacles: Vec<ObstacleDisc> = from_value(e["obstacles"].clone()).unwrap();
        let grid = Grid::with_boundary(&config, &obstacles, Some(boundary(&e["travel_boundary"])));
        let (connection, keep) = terminal_connection(
            &config,
            &TerminalBudget::default(),
            Some(cached),
            Some(Timestamp(45_180)),
            pose,
            e["before_steering"]["applied_curvature_per_m"]
                .as_f64()
                .unwrap(),
            cached.goal,
            0.0,
            &grid,
            primitive_envelope::ErrorBound::default(),
        )
        .unwrap();
        assert!(connection.used_seed && keep);
    }

    #[test]
    fn boundary_change_and_task_hold_clear_hint_without_adopting_motion() {
        let data = fixture();
        let event = &data["navigation"][1];
        let config: NavigationConfig = from_value(data["config"].clone()).unwrap();
        let cached = hint(&event["before_seed"], &data["chosen_next"][0]);
        let mut nav = Navigator::new(config).unwrap();
        nav.terminal_seed = Some(cached);
        let execution = nav.execution_state();
        nav.set_travel_boundary(Some(boundary(&event["travel_boundary"])));
        assert!(nav.terminal_seed.is_none());
        assert_eq!(nav.execution_state(), execution);
        nav.terminal_seed = Some(cached);
        nav.plan_stop(Timestamp(0)).unwrap();
        assert!(nav.terminal_seed.is_none());
        assert_eq!(nav.execution_state(), execution);
    }
}
