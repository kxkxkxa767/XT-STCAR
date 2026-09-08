//! Factory racecar.zip / racecar_driver wire format. PWM is not a velocity unit.
use crate::ValidationError;

#[derive(Clone, Copy, Debug, Eq, PartialEq, serde::Serialize)]
pub struct PwmCommand {
    motor_us: u16,
    servo_us: u16,
}

/// Explicit names preserve incompatible factory Twist conventions.
#[derive(Clone, Copy, Debug)]
pub enum FactoryProfile {
    Navigation1300,
    NavigationOne1200,
    TeleopPwmDegrees,
}

impl PwmCommand {
    /// Protocol envelope from the factory README, not a vehicle safety limit.
    pub fn new(motor_us: u16, servo_us: u16) -> Result<Self, ValidationError> {
        if !(500..=2500).contains(&motor_us) || !(500..=2500).contains(&servo_us) {
            return Err(ValidationError(
                "PWM must be within 500..2500 microseconds".into(),
            ));
        }
        Ok(Self { motor_us, servo_us })
    }

    pub fn encode(self) -> [u8; 7] {
        let motor = self.motor_us.to_le_bytes();
        let servo = self.servo_us.to_le_bytes();
        let checksum = motor.into_iter().chain(servo).fold(0u8, u8::wrapping_add);
        [0xaa, motor[0], motor[1], servo[0], servo[1], checksum, 0x55]
    }
}

impl FactoryProfile {
    /// Reference conversion only. Deliberately does not accept MotionIntent:
    /// factory gains are not a calibrated speed/curvature -> PWM model.
    pub fn preview(self, linear_x: f64, angular_z: f64) -> Result<PwmCommand, ValidationError> {
        if !linear_x.is_finite() || !angular_z.is_finite() {
            return Err(ValidationError("factory input must be finite".into()));
        }
        let (motor, servo) = match self {
            Self::Navigation1300 => (1500.0 + linear_x * 100.0, 1500.0 + angular_z * 1300.0),
            Self::NavigationOne1200 => (1500.0 + linear_x * 100.0, 1500.0 + angular_z * 1200.0),
            Self::TeleopPwmDegrees => {
                if !(0.0..=180.0).contains(&angular_z) {
                    return Err(ValidationError(
                        "teleop angle must be 0..180 degrees".into(),
                    ));
                }
                (linear_x, 2500.0 - angular_z * 2000.0 / 180.0)
            }
        };
        // Validate before casting: never wrap, saturate, or silently clip bad inputs.
        if !(500.0..=2500.0).contains(&motor) || !(500.0..=2500.0).contains(&servo) {
            return Err(ValidationError(
                "factory mapping exceeds PWM envelope".into(),
            ));
        }
        PwmCommand::new(motor as u16, servo as u16)
    }
}

/// Writes complete packets to an already configured writer. Any I/O error latches
/// failure; retrying a partly transmitted frame could corrupt MCU framing.
/// A future device owner must provide bounded I/O, watchdog ticks and shutdown.
pub struct PacketWriter<W> {
    writer: W,
    failed: bool,
}
impl<W: std::io::Write> PacketWriter<W> {
    pub fn new(writer: W) -> Self {
        Self {
            writer,
            failed: false,
        }
    }
    pub fn send(&mut self, command: PwmCommand) -> std::io::Result<()> {
        if self.failed {
            return Err(std::io::Error::other("packet writer fault is latched"));
        }
        let result = self.writer.write_all(&command.encode());
        if result.is_err() {
            self.failed = true;
        }
        result
    }
}
