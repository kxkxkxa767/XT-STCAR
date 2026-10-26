//! Full state as the Python object's attributes (for parity checks) and the
//! attributes the console reads after every call.

use crate::dict;
use crate::maneuver::*;
use crate::py::*;

fn opt_pair(v: &Option<(f64, f64)>) -> V {
    v.map(V::from).unwrap_or(V::None)
}

impl Maneuver {
    /// Attributes the console and tools read after every call.
    pub fn mirrored(&self) -> V {
        dict! {
            "phase" => self.phase, "route_stage" => self.route_stage, "reason" => self.reason.as_str(),
            "continue_route" => self.continue_route, "max_drive_s" => self.max_drive_s,
            "max_presteer_s" => self.max_presteer_s, "started_at" => self.started_at,
            "handover_observed" => self.handover_observed, "drive_since" => self.drive_since,
            "first_pass_evidence" => self.first_pass_evidence.clone(),
            "right_exit_center_ack" => self.right_exit_center_ack.clone(), "right_exit_since" => self.right_exit_since,
            "second_center_ack" => self.second_center_ack.clone(),
            "quality_wait_servo" => self.quality_wait_servo, "quality_wait_previous_servo" => self.quality_wait_previous_servo,
            "quality_coast_servo" => self.quality_coast_servo, "quality_wait_refusal" => self.quality_wait_refusal.clone(),
            "last_seq" => self.last_seq, "last_publication" => self.last_publication, "last_now" => self.last_now,
            "last_geometry_seq" => self.last_geometry_seq, "last_geometry_publication" => self.last_geometry_publication,
            "entry_kind" => self.entry_kind, "neutral_pause_kind" => self.neutral_pause_kind,
            "servo" => self.servo, "steering_target" => self.steering_target,
            "natural_steering_target" => self.natural_steering_target,
            "compact_target" => self.compact_target.clone(), "second_target" => self.second_target.clone(),
            "path_evidence" => self.path_evidence.clone(), "recovery_evidence" => self.recovery_evidence.clone(),
            "orbit_track_id" => self.orbit_track_id.clone(), "second_track_id" => self.second_track_id.clone(),
            "corridor" => self.corridor.clone(), "outer_wall" => self.outer_wall.clone(),
            "turn_goal" => self.turn_goal.clone(), "exit_width_change" => self.exit_width_change.clone(),
            "quality_resume_pending" => self.quality_resume_pending,
            // Pure state predicate the console asks on every drive output.
            "lane_wall_output_active()" => self.lane_wall_output_active(),
            "first_loop_output_active()" => self.first_loop_output_active(),
        }
    }

    /// Every attribute, named as in Python (`__dict__`), for state comparison.
    pub fn dump(&self) -> V {
        let mut d = dict! {
            "started_at" => self.started_at, "initial_presteer_pwm" => self.initial_presteer_pwm,
            "natural_steering_target" => self.natural_steering_target, "max_drive_s" => self.max_drive_s,
            "max_presteer_s" => self.max_presteer_s, "phase" => self.phase, "servo" => self.servo,
            "steering_target" => self.steering_target, "last_change" => self.last_change, "last_now" => self.last_now,
            "last_seq" => self.last_seq, "last_publication" => self.last_publication, "last_receive" => self.last_receive,
            "last_signature" => self.last_signature.clone(), "corridor" => self.corridor.clone(),
            "geometry_mode" => self.geometry_mode, "outer_wall" => self.outer_wall.clone(), "turn_goal" => self.turn_goal.clone(),
            "last_geometry_receive" => self.last_geometry_receive, "last_geometry_publication" => self.last_geometry_publication,
            "last_geometry_seq" => self.last_geometry_seq, "geometry_missing" => self.geometry_missing,
            "selected" => self.selected, "settle_since" => self.settle_since,
            "settle_feedback_ticks" => self.settle_feedback_ticks, "last_control_tick" => self.last_control_tick,
            "last_control_seq" => self.last_control_seq, "drive_since" => self.drive_since,
            "presteer_ended_at" => self.presteer_ended_at, "last_error" => self.last_error,
            "alignment_count" => self.alignment_count, "alignment_publication" => self.alignment_publication,
            "alignment_receive" => self.alignment_receive, "observed_alignment" => self.observed_alignment,
            "entry_confirmed" => self.entry_confirmed, "alignment_evidence" => self.alignment_evidence,
            "reason" => self.reason.as_str(), "start_ready" => self.start_ready, "presteer_waiting" => self.presteer_waiting,
            "incoming_endpoint" => self.incoming_endpoint.clone(), "exit_width_change" => self.exit_width_change.clone(),
            "continue_route" => self.continue_route, "target_tracker" => self.target_tracker.dump(),
            "compact_target" => self.compact_target.clone(), "_route_left_hold_since" => self._route_left_hold_since,
            "left_overshoot_evidence" => self.left_overshoot_evidence.clone(),
            "_last_seen_compact" => self._last_seen_compact.as_ref().map(|(t, v)| V::list(vec![V::Float(*t), v.clone()])),
            "_last_confirmed_compact_target" => self._last_confirmed_compact_target.clone(),
            "compact_target_error" => self.compact_target_error.clone(),
            "compact_loss_evidence" => self.compact_loss_evidence.clone(), "handover_observed" => self.handover_observed,
            "orbit_since" => self.orbit_since, "orbit_reference_range_m" => self.orbit_reference_range_m.clone(),
            "orbit_bias_pwm" => self.orbit_bias_pwm.clone(), "orbit_feedback" => self.orbit_feedback.clone(),
            "orbit_left_entry_boost" => self.orbit_left_entry_boost, "_orbit_abeam_count" => self._orbit_abeam_count,
            "_orbit_abeam_publication" => self._orbit_abeam_publication.clone(), "_orbit_abeam_receive" => self._orbit_abeam_receive,
            "left_turn_feedback" => self.left_turn_feedback.clone(), "left_handover_preparing" => self.left_handover_preparing,
            "handover_control" => self.handover_control.clone(), "handover_wait_since" => self.handover_wait_since,
            "orbit_track_id" => self.orbit_track_id.clone(), "coast_since" => self.coast_since,
            "wall_ambiguity_hold" => self.wall_ambiguity_hold, "_last_actual_left_target" => self._last_actual_left_target,
            "_presteer_adoption_context" => self._presteer_adoption_context.as_ref().map(|(c, f)| dict! {"control" => c.clone(), "fresh_feedback" => *f}),
            "inner_clearance" => self.inner_clearance.clone(),
            "quality_coast_servo" => self.quality_coast_servo, "quality_wait_refusal" => self.quality_wait_refusal.clone(),
            "left_turn_pause_used" => self.left_turn_pause_used, "left_turn_pause_evidence" => self.left_turn_pause_evidence.clone(),
            "left_turn_pause_geometry_error" => self.left_turn_pause_geometry_error.clone(),
            "neutral_pause_kind" => self.neutral_pause_kind, "recovery_count" => self.recovery_count,
            "recovery_evidence" => self.recovery_evidence.clone(), "post_pass_pause_evidence" => self.post_pass_pause_evidence.clone(),
            "quality_coast_reason" => self.quality_coast_reason.clone(), "entry_bearing_required" => self.entry_bearing_required,
            "entry_bearing_released" => self.entry_bearing_released, "entry_bearing" => self.entry_bearing.clone(),
            "_incoming_association" => self._incoming_association.clone(), "_entry_last_receive" => self._entry_last_receive.clone(),
            "_entry_last_publication" => self._entry_last_publication, "_entry_ready_count" => self._entry_ready_count,
            "_entry_ready_receive" => self._entry_ready_receive.clone(), "_entry_ready_publication" => self._entry_ready_publication,
            "_entry_ready_observation_type" => self._entry_ready_observation_type.clone(),
            "_entry_ready_history" => rows_v(&self._entry_ready_history), "adopted_presteer_pwm" => self.adopted_presteer_pwm,
            "first_pass_preparing" => self.first_pass_preparing, "first_pass_evidence" => self.first_pass_evidence.clone(),
            "first_pass_progress" => self.first_pass_progress.clone(), "_first_pass_history" => V::list(self._first_pass_history.clone()),
            "_first_exit_prepare_history" => rows_v(&self._first_exit_prepare_history),
            "_quality_exit_prepare_ready" => self._quality_exit_prepare_ready, "quality_wait_release" => self.quality_wait_release.clone(),
            "quality_wait_previous_servo" => self.quality_wait_previous_servo, "_wait_release_history" => rows_v(&self._wait_release_history),
            "quality_wait_anchor" => self.quality_wait_anchor.clone(), "quality_wait_rebind" => self.quality_wait_rebind.clone(),
            "_wait_rebind_candidate" => self._wait_rebind_candidate.clone(),
            "first_exit_prepare_evidence" => self.first_exit_prepare_evidence.clone(), "right_exit_since" => self.right_exit_since,
            "right_exit_center_ack" => self.right_exit_center_ack.clone(),
            "right_exit_adoption_wait_since" => self.right_exit_adoption_wait_since,
            "right_exit_geometry" => self.right_exit_geometry.clone(), "_right_exit_last_receive" => self._right_exit_last_receive.clone(),
            "_right_exit_last_publication" => self._right_exit_last_publication.clone(), "route_stage" => self.route_stage,
            "route_events" => self.route_events.clone(), "quality_wait_since" => self.quality_wait_since,
            "quality_wait_servo" => self.quality_wait_servo, "quality_wait_used" => self.quality_wait_used,
            "quality_wait_evidence" => self.quality_wait_evidence.clone(), "quality_resume_pending" => self.quality_resume_pending,
            "quality_resume_since" => self.quality_resume_since, "_quality_good_history" => rows_v(&self._quality_good_history),
            "_quality_full_streak" => self._quality_full_streak, "_quality_pass_history" => rows_v(&self._quality_pass_history),
            "_route_alignment_history" => rows_v(&self._route_alignment_history),
            "route_alignment_evidence" => self.route_alignment_evidence.clone(), "_route_center_ack" => self._route_center_ack.clone(),
            "second_tracker" => self.second_tracker.as_ref().map(|t| t.dump()), "second_target" => self.second_target.clone(),
            "second_track_id" => self.second_track_id.clone(),
            "second_target_handover_observed" => self.second_target_handover_observed,
            "second_center_ack" => self.second_center_ack.clone(), "second_orbit_since" => self.second_orbit_since,
            "second_pass_evidence" => self.second_pass_evidence.clone(), "second_feedback" => self.second_feedback.clone(),
            "second_return_preparing" => self.second_return_preparing, "second_return_evidence" => self.second_return_evidence.clone(),
            "route_end_coast_servo" => self.route_end_coast_servo, "light_since" => self.light_since,
            "light_turn_started" => self.light_turn_started, "_lane_steering" => self._lane_steering.dump(),
            "_lane_speed_last" => opt_pair(&self._lane_speed_last),
            "_lane_pulse" => self._lane_pulse, "_lane_creep" => self._lane_creep, "_lane_creeps" => self._lane_creeps, "_light_history" => rows_v(&self._light_history),
            "_light_heading" => self._light_heading, "light_feedback" => self.light_feedback.clone(),
            "light_alignment_evidence" => self.light_alignment_evidence.clone(), "lane_motor" => self.lane_motor,
            "_lane_coast_track" => V::list(self._lane_coast_track.iter().map(|p| V::from(*p)).collect()),
            "_lane_coast_decel" => self._lane_coast_decel, "_lane_coast_decel_se" => self._lane_coast_decel_se,
            "_lane_ranges" => V::list(self._lane_ranges.iter().map(|r| V::from(vec![r.0, r.1, r.2])).collect()),
            "lane_feedback" => self.lane_feedback.clone(), "finish_evidence" => self.finish_evidence.clone(),
            "light_vision" => self.light_vision, "_light_frames" => V::list(self._light_frames.clone()),
            "light_stop_state" => self.light_stop_state, "light_stop_evidence" => self.light_stop_evidence.clone(),
            "_light_gantry_seen" => V::list(self._light_gantry_seen.clone()), "_light_line" => self._light_line.clone(),
            "_light_wait_since" => self._light_wait_since, "_light_red_frames" => self._light_red_frames,
            "_light_launch_until" => self._light_launch_until, "_light_seen" => self._light_seen,
            "_light_gantry_since" => self._light_gantry_since,
            "_light_rearm_min_m" => self._light_rearm_min_m, "light_wait_credit_s" => self.light_wait_credit_s,
            "_light_wait_tick" => self._light_wait_tick,
            "_exit_heading" => self._exit_heading, "role_distinct_evidence" => self.role_distinct_evidence.clone(),
            "second_acquisition_reason" => self.second_acquisition_reason.clone(), "_second_proof_at" => self._second_proof_at,
            "_second_pass_history" => rows_v(&self._second_pass_history), "_second_role_anchor" => self._second_role_anchor.clone(),
            "_second_recovering" => self._second_recovering, "_second_recovery_last" => self._second_recovery_last.clone(),
            "second_recovery_prior" => self.second_recovery_prior.clone(), "second_rebind_evidence" => self.second_rebind_evidence.clone(),
            "_pause_stage_held" => self._pause_stage_held, "_pause_steer_target" => self._pause_steer_target,
            "_avoid" => self._avoid.clone(), "path_evidence" => self.path_evidence.clone(),
            "path_override_count" => self.path_override_count, "avoid_count" => self.avoid_count,
            "avoid_evidence" => V::list(self.avoid_evidence.clone()), "wait_steering_changes" => self.wait_steering_changes,
            "compass" => self.compass.dump(), "entry_kind" => self.entry_kind,
            "_lane_start_steering" => self._lane_start_steering.dump(), "_lane_start_settle_since" => self._lane_start_settle_since,
            "_lane_start_ticks" => self._lane_start_ticks, "_lane_start_slow" => self._lane_start_slow,
            "_start_motor" => self._start_motor, "_start_speed_measured" => self._start_speed_measured, "_start_speed_last" => opt_pair(&self._start_speed_last),
            "start_lane_coast_steering" => self.start_lane_coast_steering.clone(), "start_side_wall" => self.start_side_wall.clone(),
            "_front_track" => self._front_track.as_ref().map(|(at, sector, value)| {
                let mut s = V::dict();
                for (j, v) in sector {
                    s.set(&j.to_string(), *v);
                }
                V::list(vec![V::Int(*at), s, V::Float(*value)])
            }),
            "_track_history" => V::list(self._track_history.iter().map(|r| V::from(vec![r.0, r.1, r.2])).collect()),
            "_power_cmd_at" => self._power_cmd_at, "_power_since" => self._power_since,
            "_power_prev_end" => self._power_prev_end, "_launch_cmd_at" => self._launch_cmd_at, "_power_cmd_motor" => self._power_cmd_motor,
            "_launch_until" => self._launch_until, "start_lane_feedback" => self.start_lane_feedback.clone(),
            "_lane_start_ok" => self._lane_start_ok, "first_approach" => self.first_approach.clone(),
            "_start_lane_width" => self._start_lane_width.clone(),
            "_open_lost_since" => self._open_lost_since,
            "_second_plan_pending" => self._second_plan_pending.as_ref().map(|(a, b)| V::list(vec![a.clone(), b.clone()])),
            "open_loop" => self.open_loop.clone(), "open_next_loop" => self.open_next_loop.clone(),
            "open_feedback" => self.open_feedback.clone(), "_open_lookout_seq" => self._open_lookout_seq,
            "_task_left_upper" => self._task_left_upper.as_ref().map(|(s, l)| V::list(vec![s.clone(), V::Int(*l)])),
            "_second_lane_heading" => self._second_lane_heading,
            "_second_lane_previous" => self._second_lane_previous.map(|p| V::from(vec![p.0, p.1, p.2])),
            "_second_lane_rate" => self._second_lane_rate,
        };
        let mut slots = V::dict();
        for (side, slot) in &self._open_slots {
            slots.set(&side.to_string(), slot.dump());
        }
        d.set("_open_slots", slots);
        d.set(
            "_exit_release_cache",
            self._exit_release_cache.as_ref().map(|(s, v)| V::list(vec![s.clone(), V::Int(*v)])),
        );
        d.set("_path_cache", self._path_cache.as_ref().map(|(k, v)| V::list(vec![k.clone(), V::Int(*v)])));
        d
    }
}
