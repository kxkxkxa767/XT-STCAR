//! Bounded forward-start PWM proposals. No device I/O or physical safety proof.
//! The caller must authorize the entire actuator envelope, including its measured
//! acceleration and stopping distance. A Drive request alone is not a permit.
use crate::autonomy::PoseEstimate;
use crate::protocol::chassis::PwmCommand;
use crate::{FrameId, MotionOutput, Timestamp, ValidationError};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct StartupConfig {
    #[serde(default)]
    pub enabled: bool,
    pub frame_id: FrameId,
    pub stop_motor_us: u16,
    pub stop_servo_us: u16,
    pub start_motor_us: u16,
    pub max_motor_us: u16,
    pub step_us: u16,
    pub step_wait_ms: u64,
    pub max_attempt_ms: u64,
    pub stationary_window_ms: u64,
    pub max_feedback_age_ms: u64,
    pub max_feedback_gap_ms: u64,
    pub min_stationary_samples: u32,
    pub min_motion_samples: u32,
    pub stationary_distance_m: f64,
    pub moving_distance_m: f64,
    pub stationary_speed_mps: f64,
    pub stationary_yaw_rate_radps: f64,
    pub moving_speed_mps: f64,
    pub max_speed_mps: f64,
    pub min_quality: f64,
    pub max_lateral_m: f64,
    pub max_start_distance_m: f64,
    pub max_yaw_rad: f64,
}
impl StartupConfig {
    pub fn validate(&self) -> Result<(), ValidationError> {
        self.frame_id.validate()?;
        PwmCommand::new(self.stop_motor_us, self.stop_servo_us)?;
        if !(self.stop_motor_us < self.start_motor_us
            && self.start_motor_us <= self.max_motor_us
            && self.max_motor_us <= 2500
            && self.step_us > 0
            && self.step_us <= self.max_motor_us - self.stop_motor_us
            && (10..=1000).contains(&self.max_feedback_age_ms)
            && (10..=1000).contains(&self.max_feedback_gap_ms)
            && self.stationary_window_ms >= self.max_feedback_gap_ms
            && self.step_wait_ms >= self.stationary_window_ms
            && self.max_attempt_ms > self.step_wait_ms
            && self.max_attempt_ms <= 30_000
            && (3..=100).contains(&self.min_stationary_samples)
            && (2..=100).contains(&self.min_motion_samples))
        {
            return Err(ValidationError(
                "invalid startup PWM or timing limits".into(),
            ));
        }
        let positive = [
            self.stationary_distance_m,
            self.moving_distance_m,
            self.stationary_speed_mps,
            self.stationary_yaw_rate_radps,
            self.moving_speed_mps,
            self.max_speed_mps,
            self.min_quality,
            self.max_lateral_m,
            self.max_start_distance_m,
            self.max_yaw_rad,
        ];
        if positive.iter().any(|v| !v.is_finite() || *v <= 0.0)
            || self.moving_distance_m <= 2.0 * self.stationary_distance_m
            || self.moving_speed_mps <= self.stationary_speed_mps
            || self.max_speed_mps <= self.moving_speed_mps
            || self.max_start_distance_m <= self.moving_distance_m
            || self.min_quality > 1.0
            || self.max_yaw_rad > 0.5
        {
            return Err(ValidationError(
                "invalid startup motion/quality thresholds".into(),
            ));
        }
        Ok(())
    }
}

/// An upstream, short-lived authorization for a *bounded PWM envelope*. It must
/// cover the boosted output, not merely the nominal controller speed. Replay
/// permits are synthetic; they cannot certify a real vehicle.
#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct StartupPermit {
    pub issued_at: Timestamp,
    pub valid_until: Timestamp,
    pub max_motor_us: u16,
    pub max_speed_mps: f64,
}
#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct StartupFeedback {
    pub accepted: bool,
    /// Must change whenever localization is reset/reanchored.
    pub localization_epoch: u64,
    pub estimate: PoseEstimate,
}
#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct StartupInput {
    pub now: Timestamp,
    pub command: MotionOutput,
    pub nominal_motor_us: u16,
    pub nominal_servo_us: u16,
    pub permit: Option<StartupPermit>,
    /// None means no new observation, NOT stationary. Cached data still expires.
    pub feedback: Option<StartupFeedback>,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum StartupPhase {
    Idle,
    Observing,
    Ramping,
    Moving,
    Fault,
}
#[derive(Clone, Debug, Serialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum StartupAction {
    Stop {
        command: PwmCommand,
    },
    /// Proposal only: must still pass the final actuator/safety gate.
    Preview {
        command: PwmCommand,
    },
    /// Do not retain the boost. Return control to the normal bounded speed loop.
    Handoff,
}
#[derive(Clone, Debug, Serialize)]
pub struct StartupDecision {
    pub at: Timestamp,
    pub phase: StartupPhase,
    pub action: StartupAction,
    pub reason: String,
}
struct Attempt {
    started: Timestamp,
    step_at: Timestamp,
    anchor: PoseEstimate,
    still_anchor: PoseEstimate,
    still_since: Timestamp,
    still_samples: u32,
    motion_samples: u32,
    motion_suspected: bool,
    pwm: u16,
    nominal_motor_us: u16,
    nominal_servo_us: u16,
    request: MotionOutput,
}

pub struct StartupAssist {
    config: StartupConfig,
    phase: StartupPhase,
    fault: Option<String>,
    last_tick: Option<Timestamp>,
    feedback: Option<StartupFeedback>,
    attempt: Option<Attempt>,
}
impl StartupAssist {
    pub fn new(config: StartupConfig) -> Result<Self, ValidationError> {
        config.validate()?;
        Ok(Self {
            config,
            phase: StartupPhase::Idle,
            fault: None,
            last_tick: None,
            feedback: None,
            attempt: None,
        })
    }
    pub fn config(&self) -> &StartupConfig {
        &self.config
    }
    /// Bridge an actual scan-odometry result. Warm up localization before arming;
    /// all rejections while armed (including reinitialization) are terminal.
    pub fn update_with_localization(
        &mut self,
        mut input: StartupInput,
        epoch: u64,
        update: &crate::localization::LocalizationUpdate,
    ) -> StartupDecision {
        // An upstream Stop always wins over the sensor adapter.
        if input.command == MotionOutput::Stop {
            return self.update(input);
        }
        if !update.accepted {
            return self.reject_localization(input.now);
        }
        let Some(estimate) = &update.estimate else {
            return self.reject_localization(input.now);
        };
        input.feedback = Some(StartupFeedback {
            accepted: true,
            localization_epoch: epoch,
            estimate: estimate.clone(),
        });
        self.update(input)
    }

    /// Feed rejected ScanOdometry/LaserPosePipeline updates here immediately;
    /// do not reinterpret a rejection as a missing (cached) observation.
    pub fn reject_localization(&mut self, now: Timestamp) -> StartupDecision {
        self.fail(now, "localization_rejected")
    }
    fn decision(&self, at: Timestamp, action: StartupAction, reason: &str) -> StartupDecision {
        StartupDecision {
            at,
            phase: self.phase,
            action,
            reason: reason.into(),
        }
    }
    fn stop(&self, at: Timestamp, reason: &str) -> StartupDecision {
        self.decision(
            at,
            StartupAction::Stop {
                command: PwmCommand::new(self.config.stop_motor_us, self.config.stop_servo_us)
                    .expect("validated neutral"),
            },
            reason,
        )
    }
    fn fail(&mut self, at: Timestamp, reason: &str) -> StartupDecision {
        self.phase = StartupPhase::Fault;
        self.fault = Some(reason.into());
        self.stop(at, reason)
    }
    /// Faults latch even across Stop. A new instance requires an explicit upstream
    /// reset/re-arm; its stationary observation phase runs again before any boost.
    pub fn update(&mut self, input: StartupInput) -> StartupDecision {
        let now = input.now;
        if let Some(reason) = &self.fault {
            return self.stop(now, reason);
        }
        if self.last_tick.is_some_and(|last| now <= last) {
            return self.fail(now, "control_time_not_increasing");
        }
        let control_gap = self
            .last_tick
            .is_some_and(|last| now.0 - last.0 > self.config.max_feedback_gap_ms);
        self.last_tick = Some(now);
        if input.command == MotionOutput::Stop {
            self.phase = StartupPhase::Idle;
            self.attempt = None;
            self.feedback = None;
            return self.stop(now, "upstream_stop");
        }
        if !self.config.enabled {
            return self.decision(now, StartupAction::Handoff, "disabled");
        }
        if control_gap && self.attempt.is_some() {
            return self.fail(now, "control_gap");
        }
        let MotionOutput::Drive {
            speed_mps,
            curvature_per_m,
        } = input.command
        else {
            unreachable!()
        };
        if !speed_mps.is_finite() || !curvature_per_m.is_finite() || speed_mps < 0.0 {
            return self.fail(now, "invalid_or_reverse_request");
        }
        if speed_mps == 0.0 {
            self.phase = StartupPhase::Idle;
            self.attempt = None;
            self.feedback = None;
            return self.stop(now, "zero_speed_request");
        }
        let Some(permit) = &input.permit else {
            return self.fail(now, "permit_missing");
        };
        if permit.issued_at > now
            || permit.valid_until <= now
            || permit.valid_until.0 - permit.issued_at.0 > self.config.max_feedback_age_ms
            || !permit.max_speed_mps.is_finite()
            || permit.max_speed_mps <= 0.0
        {
            return self.fail(now, "permit_invalid_or_expired");
        }
        let cap = permit.max_motor_us.min(self.config.max_motor_us);
        let speed_cap = permit.max_speed_mps.min(self.config.max_speed_mps);
        if speed_mps > speed_cap
            || input.nominal_motor_us <= self.config.stop_motor_us
            || input.nominal_motor_us > cap
            || PwmCommand::new(input.nominal_motor_us, input.nominal_servo_us).is_err()
        {
            return self.fail(now, "nominal_request_outside_authorized_envelope");
        }
        let mut fresh = false;
        if let Some(feedback) = input.feedback {
            let p = &feedback.estimate;
            if !feedback.accepted
                || p.frame_id != self.config.frame_id
                || !p.pose.valid()
                || !p.speed_mps.is_finite()
                || !p.yaw_rate_radps.is_finite()
                || !p.quality.is_finite()
                || p.quality < self.config.min_quality
                || p.quality > 1.0
                || p.captured_at > now
            {
                return self.fail(now, "feedback_rejected_or_invalid");
            }
            if let Some(old) = &self.feedback {
                if feedback.localization_epoch != old.localization_epoch {
                    return self.fail(now, "localization_reset");
                }
                if p.captured_at < old.estimate.captured_at {
                    return self.fail(now, "feedback_time_regression");
                }
                if p.captured_at == old.estimate.captured_at {
                    if p != &old.estimate {
                        return self.fail(now, "changed_duplicate_feedback");
                    }
                } else {
                    if p.captured_at.0 - old.estimate.captured_at.0
                        > self.config.max_feedback_gap_ms
                    {
                        return self.fail(now, "feedback_gap");
                    }
                    fresh = true;
                }
            } else {
                fresh = true;
            }
            self.feedback = Some(feedback);
        }
        let Some(pose) = self.feedback.as_ref().map(|f| f.estimate.clone()) else {
            return self.fail(now, "feedback_missing");
        };
        if now.0 - pose.captured_at.0 >= self.config.max_feedback_age_ms {
            return self.fail(now, "feedback_stale");
        }
        if pose.speed_mps.abs() > speed_cap {
            return self.fail(now, "measured_speed_limit");
        }
        if self.attempt.is_none() {
            self.attempt = Some(Attempt {
                started: now,
                step_at: now,
                anchor: pose.clone(),
                still_anchor: pose.clone(),
                still_since: pose.captured_at,
                still_samples: 0,
                motion_samples: 0,
                motion_suspected: false,
                pwm: self.config.start_motor_us.max(input.nominal_motor_us),
                nominal_motor_us: input.nominal_motor_us,
                nominal_servo_us: input.nominal_servo_us,
                request: input.command.clone(),
            });
            self.phase = StartupPhase::Observing;
        }
        let a = self.attempt.as_mut().expect("attempt created");
        if self.phase != StartupPhase::Moving
            && (a.request != input.command
                || a.nominal_motor_us != input.nominal_motor_us
                || a.nominal_servo_us != input.nominal_servo_us)
        {
            return self.fail(now, "request_changed_requires_stop");
        }
        if self.phase != StartupPhase::Moving && a.pwm > cap {
            return self.fail(now, "authorized_pwm_limit");
        }
        if self.phase != StartupPhase::Moving && now.0 - a.started.0 >= self.config.max_attempt_ms {
            return self.fail(now, "startup_timeout");
        }
        let relative = a.anchor.pose.world_to_body(pose.pose.point());
        let yaw = (pose.pose.yaw_rad - a.anchor.pose.yaw_rad + std::f64::consts::PI)
            .rem_euclid(std::f64::consts::TAU)
            - std::f64::consts::PI;
        if self.phase == StartupPhase::Ramping
            && (relative.x_m.hypot(relative.y_m) > self.config.max_start_distance_m
                || relative.x_m < -self.config.moving_distance_m
                || relative.y_m.abs() > self.config.max_lateral_m
                || yaw.abs() > self.config.max_yaw_rad)
        {
            return self.fail(now, "unexpected_motion");
        }
        if fresh
            && self.phase == StartupPhase::Ramping
            && (relative.x_m.hypot(relative.y_m) > self.config.stationary_distance_m
                || pose.speed_mps.abs() > self.config.stationary_speed_mps)
        {
            // A brief displacement followed by a return is not proof that more
            // torque is safe. Never escalate again in this episode after it.
            a.motion_suspected = true;
        }
        if fresh {
            let excursion = a.still_anchor.pose.point().distance(pose.pose.point());
            let rotating = pose.yaw_rate_radps.abs() > self.config.stationary_yaw_rate_radps;
            if excursion > self.config.stationary_distance_m
                || pose.speed_mps.abs() > self.config.stationary_speed_mps
                || rotating
            {
                a.still_anchor = pose.clone();
                a.still_since = pose.captured_at;
                a.still_samples = 0;
            } else {
                a.still_samples = a.still_samples.saturating_add(1);
            }
            if self.phase == StartupPhase::Ramping
                && relative.x_m >= self.config.moving_distance_m
                && pose.speed_mps >= self.config.moving_speed_mps
            {
                a.motion_samples += 1;
            } else {
                a.motion_samples = 0;
            }
        }
        let stationary = fresh
            && a.still_samples >= self.config.min_stationary_samples
            && pose.captured_at.0 - a.still_since.0 >= self.config.stationary_window_ms;
        if self.phase == StartupPhase::Moving {
            if stationary {
                return self.fail(now, "stalled_after_start_no_automatic_retry");
            }
            return self.decision(now, StartupAction::Handoff, "normal_speed_control");
        }
        if self.phase == StartupPhase::Observing {
            if !stationary {
                return self.stop(now, "confirming_stationary_baseline");
            }
            a.anchor = pose.clone();
            a.step_at = now;
            a.still_since = pose.captured_at;
            a.still_samples = 0;
            self.phase = StartupPhase::Ramping;
        } else if a.motion_samples >= self.config.min_motion_samples {
            self.phase = StartupPhase::Moving;
            a.still_anchor = pose.clone();
            a.still_since = pose.captured_at;
            a.still_samples = 0;
            return self.decision(
                now,
                StartupAction::Handoff,
                "motion_confirmed_stop_increasing",
            );
        } else if !a.motion_suspected
            && stationary
            && now.0 - a.step_at.0 >= self.config.step_wait_ms
            && pose.captured_at.0 >= a.step_at.0.saturating_add(self.config.stationary_window_ms)
        {
            let Some(next) = a.pwm.checked_add(self.config.step_us).filter(|v| *v <= cap) else {
                return self.fail(now, "startup_pwm_limit_without_motion");
            };
            a.pwm = next;
            a.step_at = now;
            a.still_since = pose.captured_at;
            a.still_samples = 0;
        }
        let pwm = self.attempt.as_ref().expect("active attempt").pwm;
        self.decision(
            now,
            StartupAction::Preview {
                command: PwmCommand::new(pwm, input.nominal_servo_us).expect("validated proposal"),
            },
            "bounded_startup_proposal",
        )
    }
}
