use crate::{SensorFrames, SensorKind, SensorSample, Timestamp, ValidationError};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct SafetyConfig {
    pub schema_version: u32,
    pub simulation_only: bool,
    pub max_speed_mps: f64,
    pub max_abs_curvature_per_m: f64,
    pub heartbeat_timeout_ms: u64,
    pub deadman_timeout_ms: u64,
    pub command_timeout_ms: u64,
    pub sensor_timeout_ms: u64,
    pub required_sensors: Vec<SensorKind>,
    pub frames: SensorFrames,
}

impl SafetyConfig {
    pub fn validate(&self) -> Result<(), ValidationError> {
        if self.schema_version != 1 || !self.simulation_only {
            return Err(ValidationError(
                "schema_version=1 and simulation_only=true are required".into(),
            ));
        }
        if !self.max_speed_mps.is_finite()
            || self.max_speed_mps <= 0.0
            || !self.max_abs_curvature_per_m.is_finite()
            || self.max_abs_curvature_per_m <= 0.0
        {
            return Err(ValidationError(
                "positive finite speed/curvature limits required".into(),
            ));
        }
        if [
            self.heartbeat_timeout_ms,
            self.deadman_timeout_ms,
            self.command_timeout_ms,
            self.sensor_timeout_ms,
        ]
        .contains(&0)
        {
            return Err(ValidationError(
                "all timeout values must be explicitly positive".into(),
            ));
        }
        if self.required_sensors.iter().collect::<BTreeSet<_>>().len()
            != self.required_sensors.len()
        {
            return Err(ValidationError(
                "required_sensors contains duplicates".into(),
            ));
        }
        for frame in [
            &self.frames.imu_frame,
            &self.frames.lidar_frame,
            &self.frames.odometry_frame,
            &self.frames.body_frame,
            &self.frames.vision_frame,
        ] {
            frame.validate()?;
        }
        if self.frames.odometry_frame == self.frames.body_frame {
            return Err(ValidationError(
                "odometry and body frames must differ".into(),
            ));
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum State {
    Disarmed,
    Armed,
    Running,
    Fault,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct MotionIntent {
    pub speed_mps: f64,
    pub curvature_per_m: f64,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum Event {
    Heartbeat,
    Deadman { pressed: bool },
    Arm,
    Start,
    Disarm,
    EmergencyStop,
    ResetFault,
    Motion { intent: MotionIntent },
    Sensor { sample: SensorSample },
    Tick,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct TimedEvent {
    pub at: Timestamp,
    pub event: Event,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum StopReason {
    Disarmed,
    ArmedAwaitingStart,
    EmergencyStop,
    DeadmanReleased,
    DeadmanExpired,
    HeartbeatMissing,
    HeartbeatExpired,
    CommandMissing,
    CommandExpired,
    SensorMissing { sensor: SensorKind },
    SensorExpired { sensor: SensorKind },
    InvalidInput { message: String },
    LimitsExceeded,
    TimeRegression,
    InvalidTransition,
    FaultLatched,
    ResetRequiresDeadmanRelease,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum MotionOutput {
    Stop,
    Drive {
        speed_mps: f64,
        curvature_per_m: f64,
    },
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct OutputRecord {
    pub at: Timestamp,
    pub state: State,
    pub command: MotionOutput,
    pub reason: Option<StopReason>,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct StepReport {
    pub event_at: Timestamp,
    pub previous_state: State,
    pub state: State,
    pub output: OutputRecord,
    pub emergency_stop_latched: bool,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SinkError(pub String);
impl std::fmt::Display for SinkError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}
impl std::error::Error for SinkError {}

/// This repository supplies only offline sinks. A transport is not a motor protocol.
pub trait MotionSink {
    fn emit(&mut self, record: &OutputRecord) -> Result<(), SinkError>;
}

#[derive(Debug, Default)]
pub struct RecordingSink {
    records: Vec<OutputRecord>,
}
impl RecordingSink {
    pub fn records(&self) -> &[OutputRecord] {
        &self.records
    }
    pub fn into_records(self) -> Vec<OutputRecord> {
        self.records
    }
}
impl MotionSink for RecordingSink {
    fn emit(&mut self, record: &OutputRecord) -> Result<(), SinkError> {
        self.records.push(record.clone());
        Ok(())
    }
}

pub struct Controller {
    config: SafetyConfig,
    state: State,
    now: Timestamp,
    emergency_stop_latched: bool,
    fault_reason: Option<StopReason>,
    heartbeat_at: Option<Timestamp>,
    deadman_at: Option<Timestamp>,
    deadman_pressed: bool,
    motion: Option<(Timestamp, MotionIntent)>,
    sensors: BTreeMap<SensorKind, Timestamp>,
}

impl Controller {
    pub fn new(config: SafetyConfig) -> Result<Self, ValidationError> {
        config.validate()?;
        Ok(Self {
            config,
            state: State::Disarmed,
            now: Timestamp(0),
            emergency_stop_latched: false,
            fault_reason: None,
            heartbeat_at: None,
            deadman_at: None,
            deadman_pressed: false,
            motion: None,
            sensors: BTreeMap::new(),
        })
    }
    pub fn state(&self) -> State {
        self.state
    }
    pub fn config(&self) -> &SafetyConfig {
        &self.config
    }
    pub fn tick(&mut self, at: Timestamp) -> StepReport {
        self.handle(TimedEvent {
            at,
            event: Event::Tick,
        })
    }

    fn fault(&mut self, reason: StopReason) {
        self.state = State::Fault;
        self.motion = None;
        if self.fault_reason.is_none() {
            self.fault_reason = Some(reason);
        }
    }

    fn readiness(&self, include_motion: bool) -> Option<StopReason> {
        if self.emergency_stop_latched {
            return Some(StopReason::EmergencyStop);
        }
        if !self.deadman_pressed {
            return Some(StopReason::DeadmanReleased);
        }
        if self
            .deadman_at
            .is_none_or(|at| self.now.0.saturating_sub(at.0) >= self.config.deadman_timeout_ms)
        {
            return Some(StopReason::DeadmanExpired);
        }
        match self.heartbeat_at {
            None => return Some(StopReason::HeartbeatMissing),
            Some(at) if self.now.0.saturating_sub(at.0) >= self.config.heartbeat_timeout_ms => {
                return Some(StopReason::HeartbeatExpired);
            }
            Some(_) => {}
        }
        for kind in &self.config.required_sensors {
            match self.sensors.get(kind) {
                None => return Some(StopReason::SensorMissing { sensor: *kind }),
                Some(at) if self.now.0.saturating_sub(at.0) >= self.config.sensor_timeout_ms => {
                    return Some(StopReason::SensorExpired { sensor: *kind });
                }
                Some(_) => {}
            }
        }
        if include_motion {
            match self.motion {
                None => return Some(StopReason::CommandMissing),
                Some((at, _))
                    if self.now.0.saturating_sub(at.0) >= self.config.command_timeout_ms =>
                {
                    return Some(StopReason::CommandExpired);
                }
                Some(_) => {}
            }
        }
        None
    }

    pub fn handle(&mut self, input: TimedEvent) -> StepReport {
        let previous_state = self.state;
        if input.at < self.now {
            if matches!(input.event, Event::EmergencyStop) {
                self.emergency_stop_latched = true;
            }
            self.fault(StopReason::TimeRegression);
            return self.report(input.at, previous_state);
        }
        self.now = input.at;
        // Check previous observations before accepting a late keepalive/command.
        // A delayed packet cannot erase a timeout that already occurred.
        if matches!(self.state, State::Armed | State::Running)
            && let Some(reason) = self.readiness(self.state == State::Running)
        {
            self.fault(reason);
        }
        match input.event {
            Event::EmergencyStop => {
                self.emergency_stop_latched = true;
                self.fault_reason = Some(StopReason::EmergencyStop);
                self.fault(StopReason::EmergencyStop);
            }
            Event::Disarm => {
                if self.state != State::Fault {
                    self.state = State::Disarmed;
                }
                self.motion = None;
                self.deadman_at = None;
            }
            Event::ResetFault => {
                if self.state != State::Fault {
                    self.fault(StopReason::InvalidTransition);
                } else if self.deadman_pressed {
                    self.fault_reason = Some(StopReason::ResetRequiresDeadmanRelease);
                } else {
                    self.state = State::Disarmed;
                    self.emergency_stop_latched = false;
                    self.fault_reason = None;
                    self.motion = None;
                    self.heartbeat_at = None;
                    self.deadman_at = None;
                    self.sensors.clear();
                }
            }
            Event::Heartbeat => self.heartbeat_at = Some(self.now),
            Event::Deadman { pressed } => {
                self.deadman_pressed = pressed;
                self.deadman_at = Some(self.now);
                if !pressed && matches!(self.state, State::Armed | State::Running) {
                    self.fault(StopReason::DeadmanReleased);
                }
            }
            Event::Arm => {
                if self.state == State::Disarmed {
                    if let Some(reason) = self.readiness(false) {
                        self.fault(reason);
                    } else {
                        self.state = State::Armed;
                        self.motion = None;
                    }
                } else if self.state != State::Fault {
                    self.fault(StopReason::InvalidTransition);
                }
            }
            Event::Start => {
                if self.state == State::Armed {
                    if let Some(reason) = self.readiness(true) {
                        self.fault(reason);
                    } else {
                        self.state = State::Running;
                    }
                } else if self.state != State::Fault {
                    self.fault(StopReason::InvalidTransition);
                }
            }
            Event::Motion { intent } => {
                if !intent.speed_mps.is_finite() || !intent.curvature_per_m.is_finite() {
                    self.fault(StopReason::InvalidInput {
                        message: "motion intent must be finite".into(),
                    });
                } else if intent.speed_mps.abs() > self.config.max_speed_mps
                    || intent.curvature_per_m.abs() > self.config.max_abs_curvature_per_m
                {
                    self.fault(StopReason::LimitsExceeded);
                } else if matches!(self.state, State::Armed | State::Running) {
                    self.motion = Some((self.now, intent));
                }
            }
            Event::Sensor { sample } => {
                let kind = sample.kind();
                let captured = sample.captured_at();
                let result = sample.validate(&self.config.frames);
                if let Err(error) = result {
                    self.fault(StopReason::InvalidInput {
                        message: error.to_string(),
                    });
                } else if captured > self.now
                    || self.sensors.get(&kind).is_some_and(|old| captured < *old)
                {
                    self.fault(StopReason::InvalidInput {
                        message: "sensor timestamp is future or regresses".into(),
                    });
                } else if self.now.0 - captured.0 >= self.config.sensor_timeout_ms {
                    self.fault(StopReason::SensorExpired { sensor: kind });
                } else {
                    self.sensors.insert(kind, captured);
                }
            }
            Event::Tick => {}
        }
        self.report(input.at, previous_state)
    }

    fn report(&self, event_at: Timestamp, previous_state: State) -> StepReport {
        let (command, reason) = match self.state {
            State::Disarmed => (MotionOutput::Stop, Some(StopReason::Disarmed)),
            State::Armed => (MotionOutput::Stop, Some(StopReason::ArmedAwaitingStart)),
            State::Fault => (
                MotionOutput::Stop,
                Some(if self.emergency_stop_latched {
                    StopReason::EmergencyStop
                } else {
                    self.fault_reason
                        .clone()
                        .unwrap_or(StopReason::FaultLatched)
                }),
            ),
            State::Running => match self.motion {
                Some((_, intent)) => (
                    MotionOutput::Drive {
                        speed_mps: intent.speed_mps,
                        curvature_per_m: intent.curvature_per_m,
                    },
                    None,
                ),
                None => (MotionOutput::Stop, Some(StopReason::CommandMissing)),
            },
        };
        StepReport {
            event_at,
            previous_state,
            state: self.state,
            output: OutputRecord {
                at: self.now,
                state: self.state,
                command,
                reason,
            },
            emergency_stop_latched: self.emergency_stop_latched,
        }
    }
}
