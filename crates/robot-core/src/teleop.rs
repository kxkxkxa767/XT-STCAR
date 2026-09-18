//! Manual teleoperation limits; independent of autonomous navigation.
use serde::{Deserialize, Serialize};
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Request {
    pub seq: u64,
    pub tick: u64,
    pub op: String,
    pub motor: u16,
    pub servo: u16,
}
#[derive(Debug, Clone, Serialize)]
pub struct Status {
    pub tick: u64,
    pub seq: u64,
    pub armed: bool,
    pub motor: u16,
    pub servo: u16,
    pub reason: String,
}
pub struct Guard {
    pub status: Status,
    reverse: bool,
    received: u64,
    last_time: u64,
    stopped_at: Option<u64>,
}
impl Guard {
    pub fn new(reverse: bool) -> Self {
        Self {
            status: Status {
                tick: 0,
                seq: 0,
                armed: false,
                motor: 1500,
                servo: 1500,
                reason: "locked".into(),
            },
            reverse,
            received: 0,
            last_time: 0,
            stopped_at: None,
        }
    }
    pub fn stop(&mut self, reason: &str) {
        self.stopped_at = Some(self.status.tick);
        self.status.armed = false;
        self.status.motor = 1500;
        self.status.servo = 1500;
        self.status.reason = reason.into();
    }
    pub fn advance(&mut self, now: u64) {
        self.status.tick = now;
        if now < self.last_time || (self.status.armed && now.saturating_sub(self.last_time) > 150) {
            self.stop("control_gap");
        }
        self.last_time = now;
        if self.status.armed && now.saturating_sub(self.received) >= 300 {
            self.stop("heartbeat_timeout");
        }
    }
    pub fn apply(&mut self, req: Request, now: u64) {
        self.advance(now);
        if req.op == "stop" {
            self.status.seq = self.status.seq.max(req.seq);
            self.stop("operator_stop");
            return;
        }
        if req.seq <= self.status.seq || req.tick > now || now - req.tick >= 250 {
            self.stop("stale_request");
            return;
        }
        self.status.seq = req.seq;
        if req.motor > 1620
            || req.motor < if self.reverse { 1350 } else { 1500 }
            || !(1350..=1650).contains(&req.servo)
        {
            self.stop("pwm_out_of_range");
            return;
        }
        match req.op.as_str() {
            "arm"
                if req.motor == 1500
                    && req.servo == 1500
                    && !self.status.armed
                    && self.stopped_at.is_none_or(|t| req.tick > t) =>
            {
                self.status.armed = true;
                self.status.reason = "armed".into();
            }
            "drive" if self.status.armed => {
                self.status.motor = req.motor;
                self.status.servo = req.servo;
                self.status.reason = "manual".into();
            }
            _ => {
                self.stop("not_armed_or_invalid_request");
                return;
            }
        }
        self.received = now;
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    fn req(seq: u64, tick: u64, op: &str, motor: u16) -> Request {
        Request {
            seq,
            tick,
            op: op.into(),
            motor,
            servo: 1500,
        }
    }
    #[test]
    fn calibrated_manual_limits_are_enforced() {
        for (motor, servo, allowed) in [
            (1350, 1350, true),
            (1350, 1650, true),
            (1349, 1500, false),
            (1500, 1349, false),
            (1500, 1651, false),
        ] {
            let mut g = Guard::new(true);
            g.apply(req(1, 0, "arm", 1500), 0);
            let mut command = req(2, 50, "drive", motor);
            command.servo = servo;
            g.apply(command, 50);
            assert_eq!(g.status.armed, allowed);
            if !allowed {
                assert_eq!((g.status.motor, g.status.servo), (1500, 1500));
            }
        }
        let mut g = Guard::new(false);
        g.apply(req(1, 0, "arm", 1500), 0);
        g.apply(req(2, 50, "drive", 1400), 50);
        assert!(!g.status.armed);
    }
    #[test]
    fn timeout_cannot_rearm_with_drive() {
        let mut g = Guard::new(false);
        g.apply(req(1, 0, "arm", 1500), 0);
        g.apply(req(2, 50, "drive", 1600), 50);
        for t in [100, 200, 300, 350] {
            g.advance(t);
        }
        assert!(!g.status.armed);
        assert_eq!(g.status.motor, 1500);
        g.apply(req(3, 350, "drive", 1600), 350);
        assert!(!g.status.armed);
    }
    #[test]
    fn held_keys_continue_while_heartbeats_are_fresh() {
        let mut g = Guard::new(false);
        g.apply(req(1, 0, "arm", 1500), 0);
        for i in 1..=100 {
            g.apply(req(i + 1, i * 100, "drive", 1550), i * 100);
        }
        assert!(g.status.armed);
        assert_eq!(g.status.motor, 1550);
        g.apply(req(102, 10100, "drive", 1500), 10100);
        assert_eq!(g.status.motor, 1500);
    }
    #[test]
    fn stale_reordered_reverse_and_bounds_stop() {
        for request in [
            req(1, 0, "drive", 1550),
            req(2, 1000, "drive", 1550),
            req(2, 0, "drive", 1550),
            req(2, 300, "drive", 1490),
            req(2, 300, "drive", 1621),
        ] {
            let mut g = Guard::new(false);
            g.apply(req(1, 100, "arm", 1500), 100);
            g.advance(200);
            g.apply(request, 300);
            assert!(!g.status.armed);
            assert_eq!(g.status.motor, 1500);
        }
    }
    #[test]
    fn release_centres_and_stop_wins() {
        let mut g = Guard::new(true);
        g.apply(req(1, 0, "arm", 1500), 0);
        g.apply(req(2, 50, "drive", 1480), 50);
        assert_eq!(g.status.motor, 1480);
        g.apply(req(3, 100, "drive", 1500), 100);
        assert_eq!(g.status.motor, 1500);
        g.apply(req(0, 0, "stop", 2500), 100);
        assert!(!g.status.armed);
    }
}

#[cfg(test)]
mod stop_fence_tests {
    use super::*;
    #[test]
    fn delayed_arm_cannot_undo_stop() {
        let mut g = Guard::new(false);
        g.advance(100);
        g.stop("operator_stop");
        g.apply(
            Request {
                seq: 1,
                tick: 90,
                op: "arm".into(),
                motor: 1500,
                servo: 1500,
            },
            120,
        );
        assert!(!g.status.armed);
        g.apply(
            Request {
                seq: 2,
                tick: 150,
                op: "arm".into(),
                motor: 1500,
                servo: 1500,
            },
            150,
        );
        assert!(g.status.armed);
    }
}
