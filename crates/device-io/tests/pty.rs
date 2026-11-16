#![cfg(any(target_os = "linux", target_os = "macos"))]
use rustix::{
    fd::OwnedFd,
    fs::{Mode, OFlags, open},
    pty::{OpenptFlags, grantpt, openpt, ptsname, unlockpt},
    termios::{self, InputModes, OptionalActions, SpecialCodeIndex},
};
use std::{
    path::PathBuf,
    time::{Duration, Instant},
};
use xt_stcar_device_io::SerialPort;

fn pty() -> (OwnedFd, PathBuf) {
    let master = openpt(OpenptFlags::RDWR | OpenptFlags::NOCTTY).unwrap();
    grantpt(&master).unwrap();
    unlockpt(&master).unwrap();
    let path = PathBuf::from(ptsname(&master, Vec::new()).unwrap().to_str().unwrap());
    (master, path)
}
#[test]
fn configured_pty_roundtrips_binary_and_timeout_is_bounded() {
    let (master, path) = pty();
    let mut port = SerialPort::open(&path, 115200).unwrap();
    let input = [0, 13, 10, 17, 19, 85, 255];
    rustix::io::write(&master, &input).unwrap();
    let mut buf = [0; 64];
    let n = port
        .read_until(&mut buf, Instant::now() + Duration::from_secs(1))
        .unwrap()
        .unwrap();
    assert_eq!(&buf[..n], input);
    let start = Instant::now();
    assert_eq!(
        port.read_until(&mut buf, start + Duration::from_millis(20))
            .unwrap(),
        None
    );
    assert!(start.elapsed() < Duration::from_secs(1));
    port.write_packet_until(&input, Instant::now() + Duration::from_secs(1))
        .unwrap();
    let n = rustix::io::read(&master, &mut buf).unwrap();
    assert_eq!(&buf[..n], input);
}
#[test]
fn write_timeout_latches_and_regular_files_are_not_serial_devices() {
    let (_master, path) = pty();
    let mut port = SerialPort::open(&path, 38400).unwrap();
    assert_eq!(
        port.write_packet_until(&[1], Instant::now())
            .unwrap_err()
            .kind(),
        std::io::ErrorKind::TimedOut
    );
    assert!(
        port.write_packet_until(&[1], Instant::now() + Duration::from_secs(1))
            .unwrap_err()
            .to_string()
            .contains("latched")
    );
    let file = tempfile::NamedTempFile::new().unwrap();
    std::fs::write(file.path(), b"unchanged").unwrap();
    assert!(SerialPort::open(file.path(), 115200).is_err());
    assert_eq!(std::fs::read(file.path()).unwrap(), b"unchanged");
    assert!(SerialPort::open(&path, 9600).is_err());
}
#[test]
fn dropped_port_releases_exclusive_and_restores_termios() {
    let (_master, path) = pty();
    // Keep an independent descriptor before TIOCEXCL so readback observes the
    // actual tty settings while SerialPort owns it, rather than cached options.
    let observer = open(
        &path,
        OFlags::RDWR | OFlags::NOCTTY | OFlags::NONBLOCK | OFlags::CLOEXEC,
        Mode::empty(),
    )
    .unwrap();
    let mut configured = termios::tcgetattr(&observer).unwrap();
    configured.make_raw();
    let software_flow = InputModes::IXON | InputModes::IXOFF | InputModes::IXANY;
    configured.input_modes.insert(software_flow);
    configured.special_codes[SpecialCodeIndex::VMIN] = 7;
    configured.special_codes[SpecialCodeIndex::VTIME] = 9;
    configured.set_speed(38400).unwrap();
    termios::tcsetattr(&observer, OptionalActions::Now, &configured).unwrap();
    let original = termios::tcgetattr(&observer).unwrap();
    assert!(original.input_modes.contains(software_flow));
    {
        let _port = SerialPort::open(&path, 115200).unwrap();
        let applied = termios::tcgetattr(&observer).unwrap();
        assert!(!applied.input_modes.intersects(software_flow));
        assert_eq!(applied.special_codes[SpecialCodeIndex::VMIN], 1);
        assert_eq!(applied.special_codes[SpecialCodeIndex::VTIME], 0);
        assert_eq!(applied.input_speed(), 115200);
        assert_eq!(applied.output_speed(), 115200);
    }
    let restored = termios::tcgetattr(&observer).unwrap();
    assert_eq!(restored.input_modes, original.input_modes);
    assert_eq!(restored.output_modes, original.output_modes);
    assert_eq!(restored.control_modes, original.control_modes);
    assert_eq!(restored.local_modes, original.local_modes);
    assert_eq!(restored.input_speed(), original.input_speed());
    assert_eq!(restored.output_speed(), original.output_speed());
    // rustix SpecialCodes has no PartialEq; its Debug implementation enumerates
    // every code, avoiding unsafe memory comparisons of termios padding.
    assert_eq!(
        format!("{:?}", restored.special_codes),
        format!("{:?}", original.special_codes)
    );
    #[cfg(target_os = "linux")]
    assert_eq!(restored.line_discipline, original.line_discipline);
    let reopened = SerialPort::open(&path, 230400).unwrap();
    assert_eq!(termios::tcgetattr(&observer).unwrap().input_speed(), 230400);
    drop(reopened);
    assert_eq!(termios::tcgetattr(&observer).unwrap().input_speed(), 38400);
}
