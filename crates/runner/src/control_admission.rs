//! Prepare source/lease constraints once before background candidate selection.
use crate::autonomy::AutonomyConfig;
use crate::autonomy_replay::SensorSnapshot;
use crate::control_execution::PlanningContext;
use xt_stcar_robot_core::Timestamp;
use xt_stcar_robot_core::admission::{AdmissionRejection, AdoptionConstraints};
use xt_stcar_robot_core::motion_transition::{MotionTransition, project_motion};

pub(crate) fn prepare(
    config: &AutonomyConfig,
    input: &SensorSnapshot,
    context: &PlanningContext,
    oldest_at: Timestamp,
    max_age_ms: u64,
) -> Result<AdoptionConstraints, AdmissionRejection> {
    let invalid = AdmissionRejection::Invalid;
    let nav = &config.navigation;
    // Check raw history before max/min: f64::max intentionally ignores NaN.
    if context.source_at != input.at
        || context.planned_at != context.projected_pose.captured_at
        || context.planned_at != context.steering.at
        || context.projected_pose.frame_id != input.pose.frame_id
        || input.pose.captured_at > context.planned_at
        || oldest_at > input.pose.captured_at
        || !input.pose.speed_mps.is_finite()
        || input.pose.speed_mps < -1e-6
        || input.pose.speed_mps > nav.max_speed_mps + 1e-6
        || !(0.0..=nav.max_speed_mps).contains(&context.historical_speed_bound_mps)
        || !(0.0..=nav.max_curvature_per_m).contains(&context.historical_curvature_bound_per_m)
        || !(0.0..=nav.max_speed_mps).contains(&context.projected_pose.speed_mps)
        || !(0.0..=nav.max_speed_mps).contains(&context.held_speed_mps)
        || !context.steering.applied_curvature_per_m.is_finite()
        || !context.steering.commanded_curvature_per_m.is_finite()
        || context.steering.applied_curvature_per_m.abs() > nav.max_curvature_per_m
        || context.steering.commanded_curvature_per_m.abs() > nav.max_curvature_per_m
    {
        return Err(invalid);
    }
    let expires = oldest_at.0.checked_add(max_age_ms).ok_or(invalid)?;
    if context.planned_at.0 >= expires {
        return Err(invalid);
    }
    let through = context
        .planned_at
        .0
        .checked_add(nav.control_period_ms)
        .ok_or(invalid)?
        .min(expires - 1);
    let stop_at = expires.checked_add(nav.control_period_ms).ok_or(invalid)?;
    let window_s = (through - context.planned_at.0) as f64 / 1000.0;
    let end = project_motion(
        context.projected_pose.pose,
        MotionTransition {
            initial_speed_mps: context.projected_pose.speed_mps,
            initial_curvature_per_m: context.steering.applied_curvature_per_m,
            target_speed_mps: context.held_speed_mps,
            target_curvature_per_m: context.steering.commanded_curvature_per_m,
            max_accel_mps2: nav.max_accel_mps2,
            max_decel_mps2: nav.max_decel_mps2,
            max_curvature_rate_per_s: nav.max_curvature_rate_per_s,
        },
        window_s,
    )
    .ok_or(invalid)?;
    let constraints = AdoptionConstraints {
        source_pose: input.pose.pose,
        projected_pose: context.projected_pose.pose,
        held_speed_mps: context.held_speed_mps,
        source_age_s: (context.planned_at.0 - input.pose.captured_at.0) as f64 / 1000.0,
        adoption_window_s: window_s,
        speed_bound_mps: context
            .historical_speed_bound_mps
            .max(input.pose.speed_mps.clamp(0.0, nav.max_speed_mps))
            .max(context.projected_pose.speed_mps)
            .max(context.held_speed_mps),
        curvature_bound_per_m: context
            .historical_curvature_bound_per_m
            .max(context.steering.applied_curvature_per_m.abs())
            .max(context.steering.commanded_curvature_per_m.abs()),
        projected_speed_mps: context.projected_pose.speed_mps,
        projected_curvature_per_m: context.steering.applied_curvature_per_m,
        window_end_speed_mps: end.speed_mps,
        window_end_curvature_per_m: end.curvature_per_m,
        held_curvature_per_m: context.steering.commanded_curvature_per_m,
        source_travel_time_s: stop_at
            .checked_sub(input.pose.captured_at.0)
            .ok_or(invalid)? as f64
            / 1000.0,
        transition_horizon_s: stop_at.checked_sub(context.planned_at.0).ok_or(invalid)? as f64
            / 1000.0,
        future_travel_time_s: max_age_ms
            .checked_add(nav.control_period_ms)
            .ok_or(invalid)? as f64
            / 1000.0,
    };
    constraints.validate(nav)?;
    Ok(constraints)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::autonomy::RoadFrame;
    use crate::simulation::{SimulationConfig, synthetic_scan};
    use xt_stcar_robot_core::autonomy::{LightState, PoseEstimate, RoadObservation};
    use xt_stcar_robot_core::navigation::SteeringEstimate;

    fn fixture() -> (AutonomyConfig, SensorSnapshot, PlanningContext) {
        let simulation = SimulationConfig::example();
        let config = simulation.autonomy.clone();
        let pose = PoseEstimate {
            captured_at: Timestamp(0),
            frame_id: config.mission.world_frame.clone(),
            pose: simulation.initial_pose,
            speed_mps: 0.0,
            yaw_rate_radps: 0.0,
            quality: 1.0,
        };
        let input = SensorSnapshot {
            at: Timestamp(0),
            pose: pose.clone(),
            scan: synthetic_scan(&simulation, simulation.initial_pose, &[], Timestamp(0)),
            road: RoadFrame {
                observation: RoadObservation {
                    captured_at: Timestamp(0),
                    frame_id: config.mission.body_frame.clone(),
                    crosswalk: None,
                    light: LightState::Unknown,
                    light_confidence: 0.0,
                    cones_body_m: vec![],
                },
                image_width_px: 320,
                image_height_px: 240,
            },
        };
        let mut projected_pose = pose;
        projected_pose.captured_at = Timestamp(60);
        let context = PlanningContext {
            source_at: Timestamp(0),
            planned_at: Timestamp(60),
            projected_pose,
            steering: SteeringEstimate::stationary(Timestamp(60)),
            adopted_revision: 0,
            held_speed_mps: 0.0,
            historical_speed_bound_mps: 0.0,
            historical_curvature_bound_per_m: 0.0,
            adoption_constraints: None,
        };
        (config, input, context)
    }

    #[test]
    fn prepare_preserves_source_age_window_and_rejects_invalid_history_or_lease() {
        let (config, input, context) = fixture();
        let valid = prepare(&config, &input, &context, Timestamp(0), 250).unwrap();
        assert_eq!(valid.source_age_s, 0.06);
        assert_eq!(valid.adoption_window_s, 0.1);
        assert_eq!(valid.source_travel_time_s, 0.35);
        for invalid_case in 0..4 {
            let mut context = context.clone();
            match invalid_case {
                0 => context.historical_speed_bound_mps = f64::NAN,
                1 => context.historical_curvature_bound_per_m = -0.01,
                2 => context.source_at = Timestamp(1),
                _ => context.held_speed_mps = f64::NAN,
            }
            assert!(matches!(
                prepare(&config, &input, &context, Timestamp(0), 250),
                Err(AdmissionRejection::Invalid)
            ));
        }
        assert!(prepare(&config, &input, &context, Timestamp(0), 60).is_err());
        assert!(prepare(&config, &input, &context, Timestamp(0), u64::MAX).is_err());
    }
}
