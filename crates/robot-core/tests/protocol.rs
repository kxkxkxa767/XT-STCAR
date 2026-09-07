use std::io::{self, Write};
use xt_stcar_robot_core::protocol::{
    chassis::{FactoryProfile, PacketWriter, PwmCommand},
    imu::{ImuConfig, ImuDecoder},
};
use xt_stcar_robot_core::{FrameId, Timestamp, Vec3};

fn config() -> ImuConfig {
    ImuConfig {
        frame_id: FrameId("imu_link".into()),
        acceleration_bias_mps2: Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        gyro_bias_radps: Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        max_component_age_ms: 100,
        max_component_skew_ms: 20,
    }
}
fn frame(kind: u8, values: [i16; 3]) -> Vec<u8> {
    let mut b = vec![0x55, kind];
    for v in values {
        b.extend(v.to_le_bytes());
    }
    b.extend([0, 0]);
    b.push(b.iter().fold(0u8, |s, v| s.wrapping_add(*v)));
    b
}
fn cycle() -> Vec<u8> {
    [
        frame(0x51, [0, -2048, 2048]),
        frame(0x52, [16384, 0, 0]),
        frame(0x53, [0, 0, 16384]),
    ]
    .concat()
}

#[test]
fn factory_golden_packet_and_profiles_do_not_share_units() {
    assert_eq!(
        PwmCommand::new(1500, 1500).unwrap().encode(),
        [0xaa, 0xdc, 5, 0xdc, 5, 0xc2, 0x55]
    );
    assert_ne!(
        FactoryProfile::Navigation1300.preview(0.1, 0.2).unwrap(),
        FactoryProfile::NavigationOne1200.preview(0.1, 0.2).unwrap()
    );
    assert_eq!(
        FactoryProfile::TeleopPwmDegrees
            .preview(1500.0, 90.0)
            .unwrap(),
        PwmCommand::new(1500, 1500).unwrap()
    );
    for (a, b) in [
        (f64::NAN, 0.0),
        (0.0, f64::INFINITY),
        (f64::MAX, 0.0),
        (0.0, 10.0),
    ] {
        assert!(FactoryProfile::Navigation1300.preview(a, b).is_err());
    }
    assert!(PwmCommand::new(499, 1500).is_err());
    assert!(FactoryProfile::TeleopPwmDegrees.preview(0.1, 90.0).is_err());
}

#[test]
fn partial_writes_and_interrupts_complete_packet_but_error_latches() {
    struct Writer {
        bytes: Vec<u8>,
        calls: usize,
        fail: bool,
    }
    impl Write for Writer {
        fn write(&mut self, b: &[u8]) -> io::Result<usize> {
            self.calls += 1;
            if self.calls == 1 {
                return Err(io::ErrorKind::Interrupted.into());
            }
            if self.fail && self.bytes.len() >= 2 {
                return Err(io::ErrorKind::BrokenPipe.into());
            }
            let n = b.len().min(2);
            self.bytes.extend(&b[..n]);
            Ok(n)
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }
    let cmd = PwmCommand::new(1500, 1500).unwrap();
    let mut good = Writer {
        bytes: vec![],
        calls: 0,
        fail: false,
    };
    PacketWriter::new(&mut good).send(cmd).unwrap();
    assert_eq!(good.bytes, cmd.encode());
    let mut bad = Writer {
        bytes: vec![],
        calls: 0,
        fail: true,
    };
    {
        let mut tx = PacketWriter::new(&mut bad);
        assert!(tx.send(cmd).is_err());
        assert!(tx.send(cmd).unwrap_err().to_string().contains("latched"));
    }
    assert_eq!(bad.bytes.len(), 2);
}

#[test]
fn every_split_and_concatenation_preserves_units_and_orientation() {
    let bytes = cycle();
    for split in 0..=bytes.len() {
        let mut decoder = ImuDecoder::new(config()).unwrap();
        let mut samples = decoder.feed(&bytes[..split], Timestamp(0)).unwrap().samples;
        samples.extend(decoder.feed(&bytes[split..], Timestamp(1)).unwrap().samples);
        assert_eq!(samples.len(), 1);
        let s = &samples[0];
        assert!((s.acceleration_mps2.y + 9.8).abs() < 1e-10);
        assert!((s.angular_velocity_radps.x - 1000.0_f64.to_radians()).abs() < 1e-10);
        assert!((s.orientation_xyzw.z - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-10);
    }
    let mut decoder = ImuDecoder::new(config()).unwrap();
    assert_eq!(
        decoder
            .feed(&[cycle(), cycle()].concat(), Timestamp(0))
            .unwrap()
            .samples
            .len(),
        2
    );
}

#[test]
fn noise_checksum_and_missing_components_never_refresh_a_sample() {
    let mut decoder = ImuDecoder::new(config()).unwrap();
    let mut bad = frame(0x51, [0, 0, 0]);
    bad[10] ^= 1;
    let decoded = decoder
        .feed(&[vec![1, 2, 3], bad, cycle()].concat(), Timestamp(0))
        .unwrap();
    assert!(decoded.checksum_errors > 0);
    assert_eq!(decoded.samples.len(), 1);
    for t in 1..10 {
        assert!(
            decoder
                .feed(&frame(0x51, [0, 0, 0]), Timestamp(t))
                .unwrap()
                .samples
                .is_empty()
        );
    }
}

#[test]
fn stale_partial_frames_skew_and_backward_time_are_rejected() {
    let mut decoder = ImuDecoder::new(config()).unwrap();
    decoder.feed(&frame(0x51, [0, 0, 0]), Timestamp(0)).unwrap();
    let b = decoder
        .feed(
            &[frame(0x52, [0, 0, 0]), frame(0x53, [0, 0, 0])].concat(),
            Timestamp(21),
        )
        .unwrap();
    assert!(b.samples.is_empty());
    assert_eq!(b.expired_components, 1);
    let b = decoder
        .feed(&frame(0x51, [0, 0, 0]), Timestamp(121))
        .unwrap();
    assert!(b.samples.is_empty());
    assert_eq!(b.expired_components, 2);
    assert!(decoder.feed(&[], Timestamp(120)).is_err());
    let mut decoder = ImuDecoder::new(config()).unwrap();
    let bytes = cycle();
    decoder.feed(&bytes[..5], Timestamp(0)).unwrap();
    let b = decoder.feed(&bytes[5..], Timestamp(100)).unwrap();
    assert!(b.samples.is_empty());
    assert!(decoder.feed(&vec![0; 4097], Timestamp(100)).is_err());
}

#[test]
fn calibration_and_frame_are_explicit() {
    let mut spec = config();
    spec.acceleration_bias_mps2.z = 1.0;
    let sample = ImuDecoder::new(spec)
        .unwrap()
        .feed(&cycle(), Timestamp(0))
        .unwrap()
        .samples
        .remove(0);
    assert!((sample.acceleration_mps2.z - 8.8).abs() < 1e-10);
    assert_eq!(sample.frame_id.0, "imu_link");
    let mut spec = config();
    spec.gyro_bias_radps.x = f64::NAN;
    assert!(ImuDecoder::new(spec).is_err());
}
