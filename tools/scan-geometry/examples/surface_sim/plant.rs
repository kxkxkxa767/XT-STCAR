use serde_json::{Value, json};
use std::collections::VecDeque;

#[derive(Clone, Copy)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
}

pub fn number(value: &Value, key: &str, default: f64) -> f64 {
    value[key]
        .as_f64()
        .filter(|v| v.is_finite())
        .unwrap_or(default)
}

pub fn interpolate(table: &[(f64, f64)], x: f64) -> f64 {
    for pair in table.windows(2) {
        if x <= pair[1].0 {
            let weight = ((x - pair[0].0) / (pair[1].0 - pair[0].0)).clamp(0., 1.);
            return pair[0].1 + weight * (pair[1].1 - pair[0].1);
        }
    }
    table.last().unwrap().1
}

pub struct Plant {
    pub pose: Pose,
    pub speed: f64,
    pub yaw_rate: f64,
    pub curvature: f64,
    pub max_traction_ratio: f64,
    pub slip_s: f64,
    speed_table: Vec<(f64, f64)>,
    response_table: Vec<(f64, f64)>,
    servo_delay: f64,
    motor_delay: f64,
    servo_tau: f64,
    play: f64,
    gain: f64,
    turn_drag: f64,
    coast: f64,
    brake: f64,
    accel_max: f64,
    resistance_scale: f64,
    resistance_steps: Vec<(f64, f64)>,
    resistance_step: usize,
    active_resistance_scale: f64,
    traction_mu: f64,
    physical_servo: f64,
    wheel_pwm: f64,
    last_servo: f64,
    last_motor: f64,
    servo_queue: VecDeque<(f64, f64)>,
    motor_queue: VecDeque<(f64, f64)>,
    history: VecDeque<(f64, Pose)>,
}

impl Plant {
    pub fn new(pose: Pose, config: &Value) -> Self {
        let anchors = [
            (1550., 1.1),
            (1560., 2.2),
            (1570., 2.7),
            (1580., 3.2),
            (1600., 4.2),
        ];
        let mut speed_table: Vec<_> = config["speed_table"]
            .as_array()
            .map(|rows| {
                rows.iter()
                    .map(|p| (p[0].as_f64().unwrap(), p[1].as_f64().unwrap()))
                    .collect()
            })
            .unwrap_or_else(|| anchors.to_vec());
        speed_table.sort_by(|a, b| a.0.total_cmp(&b.0));
        assert!(speed_table.len() >= 2);
        assert!(
            speed_table
                .windows(2)
                .all(|p| p[1].0 > p[0].0 && p[1].1 >= p[0].1)
        );
        let response_table: Vec<_> = config["response_table"]
            .as_array()
            .map(|rows| {
                rows.iter()
                    .map(|p| (p[0].as_f64().unwrap(), p[1].as_f64().unwrap()))
                    .collect()
            })
            .unwrap_or_else(|| {
                vec![
                    (1550., number(config, "speed_tau_s", 0.70)),
                    (1600., number(config, "speed_tau_s", 0.70)),
                ]
            });
        assert!(response_table.len() >= 2 && response_table.iter().all(|p| p.1 > 0.));
        assert!(response_table.windows(2).all(|p| p[1].0 > p[0].0));
        let resistance_scale = number(config, "resistance_scale", 1.);
        let resistance_steps: Vec<(f64, f64)> = config["resistance_steps"]
            .as_array()
            .map(|rows| {
                rows.iter()
                    .map(|p| {
                        assert_eq!(p.as_array().map(Vec::len), Some(2));
                        (p[0].as_f64().unwrap(), p[1].as_f64().unwrap())
                    })
                    .collect()
            })
            .unwrap_or_default();
        assert!(resistance_steps.iter().all(|&(t, scale)| {
            t.is_finite() && t >= 0. && scale.is_finite() && scale > 0.
        }));
        assert!(resistance_steps.windows(2).all(|p| p[1].0 > p[0].0));
        let plant = Self {
            pose,
            speed: 0.,
            yaw_rate: 0.,
            curvature: 0.,
            max_traction_ratio: 0.,
            slip_s: 0.,
            speed_table,
            response_table,
            servo_delay: number(config, "servo_delay_s", 0.15),
            motor_delay: number(config, "motor_delay_s", 0.10),
            servo_tau: number(config, "servo_tau_s", 0.10),
            play: number(config, "steering_play_pwm", 40.),
            gain: number(config, "steering_gain", 1.2),
            turn_drag: number(config, "turn_drag", 0.5),
            coast: number(config, "coast_decel_mps2", 0.58),
            brake: number(config, "brake_decel_mps2", 0.95),
            accel_max: number(config, "drive_accel_max_mps2", 4.),
            resistance_scale,
            resistance_steps,
            resistance_step: 0,
            active_resistance_scale: resistance_scale,
            traction_mu: number(config, "traction_mu", 0.6),
            physical_servo: 1500.,
            wheel_pwm: 1500.,
            last_servo: 1500.,
            last_motor: 1500.,
            servo_queue: VecDeque::new(),
            motor_queue: VecDeque::new(),
            history: VecDeque::from([(0., pose)]),
        };
        assert!(plant.servo_tau > 0. && plant.coast > 0. && plant.brake > 0.);
        assert!(plant.resistance_scale > 0. && plant.traction_mu > 0.);
        assert!(plant.motor_delay >= 0. && plant.servo_delay >= 0.);
        plant
    }

    pub fn model(&self) -> Value {
        let mut model = json!({
            "speed_table": self.speed_table, "motor_delay_s": self.motor_delay,
            "servo_delay_s": self.servo_delay, "servo_tau_s": self.servo_tau,
            "steering_play_pwm": self.play, "steering_gain": self.gain, "turn_drag": self.turn_drag,
            "response_table": self.response_table,
            "coast_decel_mps2": self.coast, "brake_decel_mps2": self.brake,
            "drive_accel_max_mps2": self.accel_max, "resistance_scale": self.resistance_scale,
            "traction_mu": self.traction_mu,
            "coast_resistance_equivalent_coefficient": self.coast * self.resistance_scale / 9.80665,
            "friction_model": "Additional rolling/drive resistance is separate from the friction-circle traction bound.",
            "pwm_targets": (1550..=1600).step_by(5).map(|p| (p, interpolate(&self.speed_table, p as f64),interpolate(&self.response_table,p as f64))).collect::<Vec<_>>()
        });
        if !self.resistance_steps.is_empty() {
            model["resistance_steps"] = json!(self.resistance_steps);
            model["resistance_scale_current"] = json!(self.active_resistance_scale);
        }
        model
    }

    pub fn pose_at(&self, t: f64) -> Pose {
        for (a, b) in self.history.iter().zip(self.history.iter().skip(1)) {
            if a.0 <= t && t <= b.0 {
                let w = (t - a.0) / (b.0 - a.0);
                return Pose {
                    x: a.1.x + w * (b.1.x - a.1.x),
                    y: a.1.y + w * (b.1.y - a.1.y),
                    yaw: a.1.yaw + w * (b.1.yaw - a.1.yaw),
                };
            }
        }
        if t < self.history[0].0 {
            self.history[0].1
        } else {
            self.pose
        }
    }

    pub fn step(&mut self, t: f64, dt: f64, motor: i64, servo: i64) {
        // Surface changes are physical inputs, independent of the motor delay
        // and unavailable to the controller's online load estimate.
        while let Some(&(at, scale)) = self.resistance_steps.get(self.resistance_step) {
            if at > t {
                break;
            }
            self.active_resistance_scale = scale;
            self.resistance_step += 1;
        }
        self.motor_queue
            .push_back((t + self.motor_delay, motor as f64));
        self.servo_queue
            .push_back((t + self.servo_delay, servo as f64));
        while self.motor_queue.front().is_some_and(|c| c.0 <= t) {
            self.last_motor = self.motor_queue.pop_front().unwrap().1;
        }
        while self.servo_queue.front().is_some_and(|c| c.0 <= t) {
            self.last_servo = self.servo_queue.pop_front().unwrap().1;
        }
        self.physical_servo +=
            (self.last_servo - self.physical_servo) * (dt / self.servo_tau).min(1.);
        self.wheel_pwm = self.wheel_pwm.clamp(
            self.physical_servo - self.play,
            self.physical_servo + self.play,
        );
        let table = [(1200., -1.35), (1314., -1.2), (1500., 0.), (1720., 1.42)];
        let requested_k = interpolate(&table, self.wheel_pwm) * self.gain;
        let drive = self.last_motor > 1500. && (self.speed > 0.05 || self.last_motor >= 1560.);
        let extra_drag = (self.active_resistance_scale - 1.) * self.coast;
        let requested_accel = if drive {
            let target = interpolate(&self.speed_table, self.last_motor)
                / (1. + self.turn_drag * requested_k.abs());
            let response = interpolate(&self.response_table, self.last_motor);
            ((target - self.speed) / response).clamp(-self.coast, self.accel_max) - extra_drag
        } else if self.last_motor < 1500. {
            -self.brake - extra_drag
        } else {
            -self.coast * self.active_resistance_scale
        };
        let grip = self.traction_mu * 9.80665;
        let lateral = self.speed * self.speed * requested_k;
        let demand = lateral.hypot(requested_accel);
        self.max_traction_ratio = self.max_traction_ratio.max(demand / grip);
        let scale = if demand > grip { grip / demand } else { 1. };
        if scale < 1. {
            self.slip_s += dt;
        }
        self.curvature = requested_k * scale;
        self.speed = (self.speed + requested_accel * scale * dt).max(0.);
        self.yaw_rate = self.curvature * self.speed;
        for _ in 0..4 {
            self.pose.yaw += self.yaw_rate * dt / 4.;
            self.pose.x += self.speed * self.pose.yaw.cos() * dt / 4.;
            self.pose.y += self.speed * self.pose.yaw.sin() * dt / 4.;
        }
        self.history.push_back((t, self.pose));
        while self.history.len() > 2 && self.history[1].0 < t - 1. {
            self.history.pop_front();
        }
    }
}
