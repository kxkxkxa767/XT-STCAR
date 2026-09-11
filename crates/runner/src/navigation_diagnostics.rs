//! Bounded first-failure context. Pure observation: never changes a command or writes a file.
use crate::autonomy::AutonomyStep;
use serde::Serialize;
use std::collections::VecDeque;
use xt_stcar_robot_core::autonomy::Pose2;
use xt_stcar_robot_core::mission::MissionPhase;
use xt_stcar_robot_core::navigation::{NavigationDiagnostics, NavigationStatus};
use xt_stcar_robot_core::{MotionOutput, Timestamp};

pub const BEFORE_FRAMES: usize = 8;
pub const AFTER_FRAMES: usize = 8;

#[derive(Clone, Debug, Serialize)]
pub struct NavigationFrame {
    pub at: Timestamp,
    pub phase: Option<MissionPhase>,
    pub pose: Pose2,
    pub measured_speed_mps: f64,
    pub command: MotionOutput,
    pub navigation_status: Option<NavigationStatus>,
    pub navigation_reason: Option<String>,
    pub navigation: Option<NavigationDiagnostics>,
    pub fault: Option<String>,
}

impl NavigationFrame {
    pub fn from_step(at: Timestamp, pose: Pose2, speed: f64, step: &AutonomyStep) -> Self {
        Self {
            at,
            phase: step.mission.as_ref().map(|mission| mission.phase),
            pose,
            measured_speed_mps: speed,
            command: step.command.clone(),
            navigation_status: step.navigation.as_ref().map(|nav| nav.status),
            navigation_reason: step
                .navigation
                .as_ref()
                .and_then(|nav| bounded(&nav.reason)),
            navigation: step.navigation.as_ref().map(|nav| nav.diagnostics),
            fault: bounded(&step.fault),
        }
    }

    fn unexpected_stop(&self) -> bool {
        is_unexpected_stop(
            self.navigation_status,
            self.navigation_reason.as_deref(),
            self.fault.is_some(),
        )
    }
}

pub(crate) fn is_unexpected_stop(
    status: Option<NavigationStatus>,
    reason: Option<&str>,
    fault: bool,
) -> bool {
    fault
        || (status == Some(NavigationStatus::Blocked)
            && !matches!(reason, Some("task_stop" | "goal_braking")))
}

fn bounded(text: &Option<String>) -> Option<String> {
    text.as_ref().map(|text| text.chars().take(256).collect())
}

#[derive(Debug, Serialize)]
pub struct NavigationFailureWindow {
    pub first_at: Timestamp,
    pub trigger_index: usize,
    /// At most 8 before + trigger + 8 after. No path, image, or point-cloud payload.
    pub frames: Vec<NavigationFrame>,
}

pub struct FirstNavigationFailure {
    before: VecDeque<NavigationFrame>,
    first: Option<NavigationFailureWindow>,
}

impl Default for FirstNavigationFailure {
    fn default() -> Self {
        Self {
            before: VecDeque::with_capacity(BEFORE_FRAMES),
            first: None,
        }
    }
}

impl FirstNavigationFailure {
    pub fn observe(&mut self, mut frame: NavigationFrame) {
        // Public callers cannot turn this compact journal into unbounded strings.
        frame.navigation_reason = bounded(&frame.navigation_reason);
        frame.fault = bounded(&frame.fault);
        if let Some(first) = &mut self.first {
            if first.frames.len() < first.trigger_index + 1 + AFTER_FRAMES {
                first.frames.push(frame);
            }
        } else if frame.unexpected_stop() {
            let trigger_index = self.before.len();
            let mut frames = Vec::with_capacity(BEFORE_FRAMES + 1 + AFTER_FRAMES);
            frames.extend(self.before.drain(..));
            let first_at = frame.at;
            frames.push(frame);
            self.first = Some(NavigationFailureWindow {
                first_at,
                trigger_index,
                frames,
            });
        } else {
            if self.before.len() == BEFORE_FRAMES {
                self.before.pop_front();
            }
            self.before.push_back(frame);
        }
    }

    pub fn finish(self) -> Option<NavigationFailureWindow> {
        self.first
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(at: u64, reason: Option<&str>) -> NavigationFrame {
        NavigationFrame {
            at: Timestamp(at),
            phase: Some(MissionPhase::Cones),
            pose: Pose2::default(),
            measured_speed_mps: 0.2,
            command: if reason.is_some() {
                MotionOutput::Stop
            } else {
                MotionOutput::Drive {
                    speed_mps: 0.2,
                    curvature_per_m: 0.0,
                }
            },
            navigation_status: Some(if reason.is_some() {
                NavigationStatus::Blocked
            } else {
                NavigationStatus::Driving
            }),
            navigation_reason: reason.map(str::to_owned),
            navigation: None,
            fault: None,
        }
    }

    #[test]
    fn expected_stops_do_not_hide_first_unexpected_block_or_replace_it_with_timeout() {
        let mut journal = FirstNavigationFailure::default();
        for at in 0..20 {
            journal.observe(frame(at, Some("task_stop")));
        }
        journal.observe(frame(20, Some("goal_braking")));
        for at in 21..30 {
            journal.observe(frame(at, None));
        }
        journal.observe(frame(30, Some("no_collision_free_braking_trajectory")));
        for at in 31..1000 {
            journal.observe(frame(at, Some("no_forward_kinematic_path")));
        }
        let first = journal.finish().unwrap();
        assert_eq!(first.first_at, Timestamp(30));
        assert_eq!(first.trigger_index, 8);
        assert_eq!(first.frames.len(), 17);
        assert_eq!(first.frames.first().unwrap().at, Timestamp(22));
        assert_eq!(first.frames.last().unwrap().at, Timestamp(38));
        assert_eq!(
            first.frames[8].navigation_reason.as_deref(),
            Some("no_collision_free_braking_trajectory")
        );
    }

    #[test]
    fn startup_failures_and_short_runs_keep_bounded_utf8_context() {
        let mut journal = FirstNavigationFailure::default();
        journal.observe(frame(0, Some(&"曲".repeat(1000))));
        for at in 1..20 {
            journal.observe(frame(at, None));
        }
        let first = journal.finish().unwrap();
        assert_eq!(first.frames.len(), 9);
        assert_eq!(
            first.frames[0]
                .navigation_reason
                .as_ref()
                .unwrap()
                .chars()
                .count(),
            256
        );
        assert!(FirstNavigationFailure::default().finish().is_none());
    }
}
