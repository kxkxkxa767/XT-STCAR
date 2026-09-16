//! Explicit host-clock observation, separate from deterministic functional tests.
//! All source, submission and output times come from an advancing Instant clock.
//! The offline plant catches up under the last actual poll command; planner waits
//! never freeze this clock. This measures one host run, not target-board WCET.
use crate::autonomy::{Result, RoadFrame};
use crate::autonomy_replay::SensorSnapshot;
use crate::control_diagnostics::WorkerDiagnosticsOptions;
use crate::control_runtime::{AutonomyWorker, ControlRuntimeConfig, SubmitStatus};
use crate::simulation::{
    PlantDynamics, PlantState, SimulationConfig, advance_plant, check_plant_pose, render_camera,
    synthetic_scan,
};
use serde_json::{Value, json};
use std::collections::VecDeque;
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};
use xt_stcar_robot_core::autonomy::{LightState, PoseEstimate};
use xt_stcar_robot_core::{MotionOutput, Timestamp};
use xt_stcar_vision::road::RoadDetector;

/// A short turning approach, followed by loss of all source inputs. No physical
/// output is opened. Timing records are bounded in memory and returned at the end.
pub fn observe_host_clock() -> Result<Value> {
    let mut config = SimulationConfig::example();
    config.initial_pose.yaw_rad = 0.10;
    // This short observation exercises the turning search approach before a
    // crosswalk is visible. A detected crosswalk locks a straight stop target.
    // The full competition fixture remains in async_simulation, unchanged.
    config.crosswalk.min_x_m += 3.0;
    config.crosswalk.max_x_m += 3.0;
    config.validate()?;
    let detector = RoadDetector::new(config.road.clone())?;
    let mut worker = AutonomyWorker::spawn_with_diagnostics(
        config.autonomy.clone(),
        ControlRuntimeConfig {
            max_command_age_ms: config.autonomy.max_sensor_age_ms,
            startup_timeout_ms: config.autonomy.max_sensor_age_ms,
        },
        Timestamp(0),
        WorkerDiagnosticsOptions {
            measure_wall_time: true,
            schedule_hook: None,
        },
    )?;
    let epoch = Instant::now();
    let mut plant = PlantState {
        pose: config.initial_pose,
        speed_mps: 0.0,
        curvature_per_m: 0.0,
    };
    let dynamics = PlantDynamics {
        acceleration_mps2: config.plant_accel_mps2,
        braking_mps2: config.plant_brake_mps2,
        curvature_rate_per_s: config.autonomy.navigation.max_curvature_rate_per_s,
    };
    let mut pending = VecDeque::<(u64, Arc<SensorSnapshot>)>::with_capacity(4);
    let mut records = Vec::<Value>::with_capacity(64);
    let mut command = MotionOutput::Stop;
    let mut next_capture = 0;
    let mut next_poll = 0;
    let mut last_poll = None;
    let mut last_plan = None;
    let mut last_source = None;
    let mut integrated_at = 0;
    let mut captures = 0;
    let mut submitted = 0;
    let mut busy = 0;
    let mut drive_polls = 0;
    let mut turning_polls = 0;
    let mut max_poll_gap_ms = 0;
    let mut first_fault = None;
    let mut violation = false;
    let mut minimum = 12.0;
    let mut distance_m = 0.0;
    let mut max_speed_mps = 0.0_f64;
    let mut max_curvature_per_m = 0.0_f64;
    loop {
        let now = epoch.elapsed().as_millis().min(u128::from(u64::MAX)) as u64;
        // A heavily suspended host is a failed observation, not thousands of
        // catch-up output commands with invented capture times.
        if now > 6000 {
            worker.request_stop();
            return Err("host observation exceeded its 6 s bounded session".into());
        }
        while integrated_at < now {
            let advanced = advance_plant(plant, &command, dynamics, 0.001, |pose| {
                violation |= check_plant_pose(&config, pose, &config.cones, None, &mut minimum);
            });
            plant = advanced.state;
            distance_m += advanced.distance_m;
            max_speed_mps = max_speed_mps.max(plant.speed_mps);
            max_curvature_per_m = max_curvature_per_m.max(plant.curvature_per_m.abs());
            integrated_at += 1;
        }
        // Catch-up itself costs host time. Only poll at a current millisecond
        // anchor; if that millisecond advanced, integrate again first. Never
        // stamp a late output with the earlier loop-entry time.
        if epoch.elapsed().as_millis() != u128::from(now) {
            continue;
        }
        if now >= next_poll {
            if let Some(old) = last_poll {
                max_poll_gap_ms = max_poll_gap_ms.max(now - old);
            }
            last_poll = Some(now);
            next_poll = (now / 20 + 1) * 20;
            let poll = worker.poll(Timestamp(now));
            if let MotionOutput::Drive {
                curvature_per_m, ..
            } = poll.command
            {
                drive_polls += 1;
                turning_polls += u64::from(curvature_per_m.abs() > 1e-6);
            }
            if let Some(plan) = &poll.observed_plan
                && (Some(plan.planned_at) != last_plan || plan.newly_adopted)
            {
                last_plan = Some(plan.planned_at);
                if records.len() < 64 {
                    records.push(json!({
                        "at_ms":now,"source_at":plan.source_at,"planned_at":plan.planned_at,
                        "source_age_ms":plan.source_age_ms,"plan_age_ms":plan.plan_age_ms,
                        "newly_adopted":plan.newly_adopted,"rejection":poll.adoption_rejection,
                        "command":poll.command,"timings":plan.timings.as_deref(),
                        "publish_to_observation_ns":plan.timings.as_ref().zip(plan.observed_host_ns)
                            .map(|(t, end)|end.saturating_sub(t.published_host_ns)),
                    }));
                }
            }
            if let Some(fault) = poll.fault {
                first_fault
                    .get_or_insert(json!({"at_ms":now,"fault":fault,"command":poll.command}));
            }
            command = poll.command;
        }
        // Optional JSON observations above may cost time as well. Re-anchor
        // source capture/submission through the next catch-up pass if needed.
        if epoch.elapsed().as_millis() != u128::from(now) {
            continue;
        }
        if first_fault.is_none() && now >= next_capture && now < 2300 {
            next_capture = (now / 100 + 1) * 100;
            let at = Timestamp(now);
            let camera = render_camera(&config, plant.pose, LightState::Green)?;
            let input = Arc::new(SensorSnapshot {
                at,
                pose: PoseEstimate {
                    captured_at: at,
                    frame_id: config.autonomy.mission.world_frame.clone(),
                    pose: plant.pose,
                    speed_mps: plant.speed_mps,
                    yaw_rate_radps: plant.speed_mps * plant.curvature_per_m,
                    quality: 1.0,
                },
                scan: synthetic_scan(&config, plant.pose, &config.cones, at),
                road: RoadFrame {
                    elements: None,
                    observation: detector.detect(
                        &camera,
                        &[],
                        at,
                        config.autonomy.mission.body_frame.clone(),
                    )?,
                    image_width_px: camera.width(),
                    image_height_px: camera.height(),
                },
            });
            if pending.len() == 4 {
                return Err("host input delay queue reached its bound".into());
            }
            pending.push_back((now + [60, 80][captures % 2], input));
            captures += 1;
            // Rendering may take time. Poll/catch up before submitting again.
            continue;
        }
        if first_fault.is_none()
            && let Some((due, input)) = pending.front()
            && *due <= now
        {
            match worker.try_submit(Arc::clone(input), Timestamp(now))? {
                SubmitStatus::Queued | SubmitStatus::Replaced => {
                    submitted += 1;
                    last_source = Some(input.at);
                    pending.pop_front();
                }
                SubmitStatus::Busy => busy += 1,
                status => return Err(format!("host submission returned {status:?}")),
            }
        }
        if first_fault.is_some() && plant.speed_mps == 0.0 && plant.curvature_per_m == 0.0 {
            break;
        }
        thread::sleep(Duration::from_millis(1));
    }
    worker.request_stop();
    Ok(json!({
        "schema_version":1,"physical_output_enabled":false,
        "scope":"Single advancing Instant-clock host observation with real controller, RGB, scan and poll-driven turning plant; neither deterministic deadline proof nor target WCET.",
        "config":config,"nominal_capture_period_ms":100,"nominal_output_period_ms":20,
        "injected_source_delays_ms":[60,80],"source_stop_at_ms":2300,"hooks_enabled":false,
        "elapsed_ms":integrated_at,"captures":captures,"submitted":submitted,"busy_retries":busy,
        "drive_polls":drive_polls,"turning_drive_polls":turning_polls,
        "max_poll_gap_ms":max_poll_gap_ms,"last_submitted_source_at":last_source,
        "first_fault":first_fault,"final_command":command,"final_pose":plant.pose,
        "final_speed_mps":plant.speed_mps,"final_curvature_per_m":plant.curvature_per_m,
        "max_speed_mps":max_speed_mps,"max_abs_curvature_per_m":max_curvature_per_m,
        "distance_m":distance_m,"collision_or_boundary_violation":violation,
        "minimum_cone_clearance_m":minimum,"worker_counts":worker.diagnostics(),"plans":records,
    }))
}
