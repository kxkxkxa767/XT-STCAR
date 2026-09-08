//! Bounded serial I/O using safe rustix wrappers. No implicit device discovery.
//! Linux target; macOS pseudo-terminals exercise configuration and I/O in tests.
use rustix::{
    event::{PollFd, PollFlags, Timespec, poll},
    fd::OwnedFd,
    fs::{FileType, Mode, OFlags, fstat, open},
    io::{Errno, read, write},
    termios::{self, ControlModes, InputModes, OptionalActions, SpecialCodeIndex, Termios},
};
use std::{io, path::Path, time::Instant};

pub struct SerialPort {
    fd: OwnedFd,
    original: Termios,
    failed: bool,
}

impl SerialPort {
    /// Caller explicitly selects a device. Opening a tty may change modem lines;
    /// use the CLI plan before authorizing hardware capture.
    pub fn open(path: &Path, baud: u32) -> io::Result<Self> {
        if !matches!(baud, 38400 | 115200 | 230400) {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "unsupported factory baud rate",
            ));
        }
        let fd = open(
            path,
            OFlags::RDWR | OFlags::NOCTTY | OFlags::NONBLOCK | OFlags::CLOEXEC,
            Mode::empty(),
        )?;
        if FileType::from_raw_mode(fstat(&fd)?.st_mode) != FileType::CharacterDevice {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "serial input must be a character device",
            ));
        }
        let original = termios::tcgetattr(&fd)?;
        termios::ioctl_tiocexcl(&fd)?;
        let mut port = Self {
            fd,
            original,
            failed: false,
        };
        let mut options = port.original.clone();
        options.make_raw();
        // cfmakeraw does not portably clear IXOFF/IXANY. Binary sensor payloads
        // must neither trigger nor inherit software flow control in either direction.
        options
            .input_modes
            .remove(InputModes::IXON | InputModes::IXOFF | InputModes::IXANY);
        options.control_modes.remove(
            ControlModes::CSIZE
                | ControlModes::PARENB
                | ControlModes::CSTOPB
                | ControlModes::CRTSCTS,
        );
        options
            .control_modes
            .insert(ControlModes::CS8 | ControlModes::CREAD | ControlModes::CLOCAL);
        options.special_codes[SpecialCodeIndex::VMIN] = 1;
        options.special_codes[SpecialCodeIndex::VTIME] = 0;
        options.set_speed(baud)?;
        termios::tcsetattr(&port.fd, OptionalActions::Now, &options)?;
        let applied = termios::tcgetattr(&port.fd)?;
        if applied.input_speed() != baud
            || applied.output_speed() != baud
            || applied.control_modes & ControlModes::CSIZE != ControlModes::CS8
            || applied
                .control_modes
                .intersects(ControlModes::PARENB | ControlModes::CSTOPB | ControlModes::CRTSCTS)
            || applied.input_modes != options.input_modes
            || applied.output_modes != options.output_modes
            || applied.local_modes != options.local_modes
            || applied.special_codes[SpecialCodeIndex::VMIN] != 1
            || applied.special_codes[SpecialCodeIndex::VTIME] != 0
        {
            port.failed = true;
            return Err(io::Error::other("serial configuration readback differs"));
        }
        Ok(port)
    }

    fn ready(&self, flags: PollFlags, deadline: Instant) -> io::Result<bool> {
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                return Ok(false);
            }
            let timeout = Timespec {
                tv_sec: remaining
                    .as_secs()
                    .try_into()
                    .map_err(|_| io::Error::other("deadline too large"))?,
                tv_nsec: i64::from(remaining.subsec_nanos()) as _,
            };
            let mut fds = [PollFd::new(&self.fd, flags)];
            match poll(&mut fds, Some(&timeout)) {
                Err(Errno::INTR) => continue,
                Err(e) => return Err(e.into()),
                Ok(0) => return Ok(false),
                Ok(_) => {
                    let returned = fds[0].revents();
                    if returned.intersects(PollFlags::ERR | PollFlags::HUP | PollFlags::NVAL) {
                        return Err(io::Error::new(
                            io::ErrorKind::BrokenPipe,
                            "serial disconnected or unavailable",
                        ));
                    }
                    if returned.intersects(flags) {
                        return Ok(true);
                    }
                }
            }
        }
    }

    /// None means a timeout, not EOF; caller must still advance its watchdog clock.
    pub fn read_until(
        &mut self,
        buffer: &mut [u8],
        deadline: Instant,
    ) -> io::Result<Option<usize>> {
        if buffer.is_empty() {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "empty read buffer",
            ));
        }
        if self.failed {
            return Err(io::Error::other("serial fault is latched"));
        }
        let result = (|| {
            while self.ready(PollFlags::IN, deadline)? {
                match read(&self.fd, &mut *buffer) {
                    Err(Errno::INTR | Errno::AGAIN) => continue,
                    Err(e) => return Err(e.into()),
                    Ok(0) => {
                        return Err(io::Error::new(io::ErrorKind::UnexpectedEof, "serial EOF"));
                    }
                    Ok(n) => return Ok(Some(n)),
                }
            }
            Ok(None)
        })();
        if result.is_err() {
            self.failed = true;
        }
        result
    }

    /// Whole-packet deadline, including interruptions and partial writes.
    /// Any failed/partial/timed-out write latches failure; no automatic retry.
    pub fn write_packet_until(&mut self, packet: &[u8], deadline: Instant) -> io::Result<()> {
        if packet.is_empty() || packet.len() > 4096 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "packet must be 1..4096 bytes",
            ));
        }
        if self.failed {
            return Err(io::Error::other("serial fault is latched"));
        }
        let result = (|| {
            let mut offset = 0;
            while offset < packet.len() {
                if !self.ready(PollFlags::OUT, deadline)? {
                    return Err(io::Error::new(
                        io::ErrorKind::TimedOut,
                        "serial packet deadline",
                    ));
                }
                match write(&self.fd, &packet[offset..]) {
                    Err(Errno::INTR | Errno::AGAIN) => continue,
                    Err(e) => return Err(e.into()),
                    Ok(0) => return Err(io::ErrorKind::WriteZero.into()),
                    Ok(n) => offset += n,
                }
            }
            Ok(())
        })();
        if result.is_err() {
            self.failed = true;
        }
        result
    }
}
impl Drop for SerialPort {
    fn drop(&mut self) {
        let _ = termios::tcsetattr(&self.fd, OptionalActions::Now, &self.original);
        let _ = termios::ioctl_tiocnxcl(&self.fd);
    }
}
