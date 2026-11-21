//! Sensor-only faults on a separate random stream. Commands, vehicle motion,
//! lidar ray noise and the simulation ground truth remain independent.
use super::{Random, Value, json};

#[derive(Clone, Copy, Default)]
pub struct Sample {
    pub gyro: f64,
    pub accel: f64,
    pub event: Option<usize>,
}
pub struct Faults {
    events: Vec<(usize, usize, Sample)>,
    next: usize,
    pub delta_velocity: f64,
    pub seen: usize,
}
impl Faults {
    pub fn new(config: &Value, duration: f64) -> Self {
        let mut events = Vec::new();
        if let Some(seed) = config["seed"].as_u64() {
            let mut random = Random(seed.wrapping_add(0xa0761d6478bd642f));
            let gyro = config["gyro_peak_rad_s"].as_f64().unwrap_or(0.);
            let accel = config["accel_peak_m_s2"].as_f64().unwrap_or(0.);
            let mut begin = 25 + (random.uniform() * 42.) as usize;
            while (begin as f64) * 0.02 < duration {
                let id = events.len() + 1;
                if id % 2 == 1 {
                    begin = begin.div_ceil(5) * 5;
                }
                let len = if id % 3 == 1 {
                    1
                } else {
                    3 + (random.uniform() * 5.) as usize
                };
                let mut signed = || (if random.uniform() < 0.5 { -1. } else { 1. }) * (0.55 + 0.45 * random.uniform());
                let sample = Sample {
                    gyro: gyro * signed(),
                    accel: accel * signed(),
                    event: Some(id),
                };
                events.push((begin, begin + len, sample));
                begin += len + 60 + (random.uniform() * 85.) as usize;
            }
        }
        Self {
            events,
            next: 0,
            delta_velocity: 0.,
            seen: 0,
        }
    }
    pub fn sample(&mut self, tick: usize) -> Sample {
        while self.events.get(self.next).is_some_and(|e| tick >= e.1) {
            self.next += 1;
        }
        let sample = self
            .events
            .get(self.next)
            .filter(|(a, b, _)| *a <= tick && tick < *b)
            .map_or(Sample::default(), |e| e.2);
        self.delta_velocity += sample.accel * 0.02;
        if let Some(id) = sample.event {
            self.seen = id;
        }
        sample
    }
    pub fn applied(&self) -> Value {
        Value::Array(
            self.events
                .iter()
                .take(self.seen)
                .map(|(a, b, s)| {
                    json!({
            "id":s.event,"start_s":*a as f64*0.02,"duration_s":(b-a) as f64*0.02,
            "gyro_offset_rad_s":s.gyro,"accel_offset_m_s2":s.accel})
                })
                .collect(),
        )
    }
}
