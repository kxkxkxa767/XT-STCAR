use xt_stcar_robot_core::protocol::n10::{MAX_CHUNK_BYTES, N10Config, N10Decoder};
use xt_stcar_robot_core::{FrameId, Timestamp};

// Synthetic wire fixture: 350 -> 5 degrees, sixteen 1.0..2.5 m returns.
// Fixed independently of the decoder; this is not a captured hardware packet.
const GOLDEN: [u8; 58] = [
    0xa5, 0x5a, 0x3a, 0x12, 0x34, 0x88, 0xb8, 0x03, 0xe8, 0x00, 0x04, 0x4c, 0x01, 0x04, 0xb0, 0x02,
    0x05, 0x14, 0x03, 0x05, 0x78, 0x04, 0x05, 0xdc, 0x05, 0x06, 0x40, 0x06, 0x06, 0xa4, 0x07, 0x07,
    0x08, 0x08, 0x07, 0x6c, 0x09, 0x07, 0xd0, 0x0a, 0x08, 0x34, 0x0b, 0x08, 0x98, 0x0c, 0x08, 0xfc,
    0x0d, 0x09, 0x60, 0x0e, 0x09, 0xc4, 0x0f, 0x01, 0xf4, 0xf1,
];

fn config() -> N10Config {
    N10Config {
        frame_id: FrameId("laser_link".into()),
        range_min_m: 0.02,
        range_max_m: 12.0,
        max_packet_age_ms: 50,
    }
}

fn checksum(frame: &mut [u8; 58]) {
    frame[57] = frame[..57].iter().fold(0u8, |sum, b| sum.wrapping_add(*b));
}

#[test]
fn golden_packet_preserves_byte_order_units_intensities_and_wrap() {
    let mut decoder = N10Decoder::new(config()).unwrap();
    let batch = decoder.feed(&GOLDEN, Timestamp(100)).unwrap();
    assert_eq!(batch.packets.len(), 1);
    assert_eq!(batch.discarded_bytes, 0);
    let packet = &batch.packets[0];
    assert_eq!(packet.header_bytes, [0x12, 0x34]);
    assert_eq!(packet.start_angle_cdeg, 35000);
    assert_eq!(packet.end_angle_cdeg, 500);
    for (i, point) in packet.points.iter().enumerate() {
        assert_eq!(point.raw_distance_mm, 1000 + i as u16 * 100);
        assert_eq!(
            point.range_m,
            Some(f64::from(1000 + i as u16 * 100) / 1000.0)
        );
        assert_eq!(point.intensity, i as u8);
    }
    let sample = packet.packet_sample().unwrap();
    assert_eq!(sample.captured_at, Timestamp(100));
    assert_eq!(sample.frame_id.0, "laser_link");
    assert_eq!(sample.ranges_m.len(), 16);
    assert!((sample.angle_min_rad.to_degrees() + 350.0).abs() < 1e-12);
    assert!((sample.angle_increment_rad.to_degrees() + 1.0).abs() < 1e-12);
    let last = sample.angle_min_rad + 15.0 * sample.angle_increment_rad;
    assert!((last.cos() - 5.0_f64.to_radians().cos()).abs() < 1e-12);
    assert!((last.sin() + 5.0_f64.to_radians().sin()).abs() < 1e-12);
}

#[test]
fn every_fragment_boundary_and_glued_packets_keep_original_receive_times() {
    for split in 0..=GOLDEN.len() {
        let mut decoder = N10Decoder::new(config()).unwrap();
        let mut packets = decoder
            .feed(&GOLDEN[..split], Timestamp(10))
            .unwrap()
            .packets;
        packets.extend(
            decoder
                .feed(&GOLDEN[split..], Timestamp(20))
                .unwrap()
                .packets,
        );
        assert_eq!(packets.len(), 1, "split {split}");
        assert_eq!(
            packets[0].first_received_at,
            Timestamp(if split == 0 { 20 } else { 10 })
        );
        assert_eq!(
            packets[0].received_at,
            Timestamp(if split == GOLDEN.len() { 10 } else { 20 })
        );
        assert_eq!(decoder.buffered_len(), 0);
    }
    let mut decoder = N10Decoder::new(config()).unwrap();
    assert_eq!(
        decoder
            .feed(&[GOLDEN, GOLDEN].concat(), Timestamp(0))
            .unwrap()
            .packets
            .len(),
        2
    );
}

#[test]
fn noise_bad_checksum_false_headers_and_wrong_lengths_resynchronize() {
    let mut broken = GOLDEN;
    broken[9] ^= 1;
    let input = [
        vec![0xa5, 0xa5, 0x5a, 0xff, 0, 1],
        broken.to_vec(),
        GOLDEN.to_vec(),
    ]
    .concat();
    let mut decoder = N10Decoder::new(config()).unwrap();
    let batch = decoder.feed(&input, Timestamp(0)).unwrap();
    assert_eq!(batch.packets.len(), 1);
    assert_eq!(batch.length_errors, 1);
    assert_eq!(batch.checksum_errors, 1);
    assert_eq!(batch.discarded_bytes, input.len() - GOLDEN.len());
    assert_eq!(decoder.buffered_len(), 0);
    // A valid header inside a corrupt packet must not be skipped wholesale.
    let mut prefix = GOLDEN[..20].to_vec();
    prefix.extend(GOLDEN);
    let batch = decoder.feed(&prefix, Timestamp(1)).unwrap();
    assert_eq!(batch.packets.len(), 1);
    assert!(batch.checksum_errors > 0);
}

#[test]
fn missing_out_of_range_and_zero_returns_keep_their_beam_slots() {
    let mut bytes = GOLDEN;
    for (slot, raw) in [(0, 0xffff_u16), (1, 0), (2, 19), (3, 12001)] {
        bytes[7 + 3 * slot..9 + 3 * slot].copy_from_slice(&raw.to_be_bytes());
    }
    checksum(&mut bytes);
    let packet = N10Decoder::new(config())
        .unwrap()
        .feed(&bytes, Timestamp(0))
        .unwrap()
        .packets
        .remove(0);
    let sample = packet.packet_sample().unwrap();
    assert_eq!(&sample.ranges_m[..4], &[None; 4]);
    assert_eq!(sample.ranges_m[4], Some(1.4));
    assert_eq!(sample.ranges_m[15], Some(2.5));
    assert!((sample.angle_increment_rad.to_degrees() + 1.0).abs() < 1e-12);
    assert_eq!(packet.points[0].raw_distance_mm, 0xffff);
    assert_eq!(packet.points[3].raw_distance_mm, 12001);
}

#[test]
fn unusable_packets_never_become_fresh_sensor_samples() {
    let mut bytes = GOLDEN;
    for slot in 0..16 {
        bytes[7 + 3 * slot..9 + 3 * slot].fill(0xff);
    }
    checksum(&mut bytes);
    let mut decoder = N10Decoder::new(config()).unwrap();
    let batch = decoder.feed(&bytes, Timestamp(0)).unwrap();
    assert_eq!(batch.packets.len(), 1);
    assert!(batch.packets[0].packet_sample().is_none());
    let mut zero_span = GOLDEN;
    zero_span[55..57].copy_from_slice(&35000u16.to_be_bytes());
    checksum(&mut zero_span);
    assert!(
        decoder.feed(&zero_span, Timestamp(1)).unwrap().packets[0]
            .packet_sample()
            .is_none()
    );
    let mut invalid_angle = GOLDEN;
    invalid_angle[5..7].copy_from_slice(&36001u16.to_be_bytes());
    checksum(&mut invalid_angle);
    let batch = decoder.feed(&invalid_angle, Timestamp(2)).unwrap();
    assert!(batch.packets.is_empty());
    assert_eq!(batch.angle_errors, 1);
    assert_eq!(
        decoder.feed(&GOLDEN, Timestamp(3)).unwrap().packets.len(),
        1
    );
}

#[test]
fn stale_fragments_expire_at_boundary_even_without_new_bytes() {
    let mut decoder = N10Decoder::new(config()).unwrap();
    decoder.feed(&GOLDEN[..12], Timestamp(10)).unwrap();
    assert_eq!(decoder.feed(&[], Timestamp(59)).unwrap().expired_packets, 0);
    let expired = decoder.feed(&[], Timestamp(60)).unwrap();
    assert_eq!(expired.expired_packets, 1);
    assert_eq!(expired.discarded_bytes, 12);
    assert_eq!(decoder.buffered_len(), 0);
    assert!(
        decoder
            .feed(&GOLDEN[12..], Timestamp(60))
            .unwrap()
            .packets
            .is_empty()
    );
    assert_eq!(
        decoder.feed(&GOLDEN, Timestamp(61)).unwrap().packets.len(),
        1
    );
}

#[test]
fn invalid_calls_do_not_mutate_state_and_noise_memory_is_bounded() {
    let mut decoder = N10Decoder::new(config()).unwrap();
    decoder.feed(&GOLDEN[..12], Timestamp(10)).unwrap();
    assert!(decoder.feed(&GOLDEN[12..], Timestamp(9)).is_err());
    assert!(
        decoder
            .feed(&vec![0; MAX_CHUNK_BYTES + 1], Timestamp(11))
            .is_err()
    );
    assert_eq!(decoder.buffered_len(), 12);
    assert_eq!(
        decoder
            .feed(&GOLDEN[12..], Timestamp(10))
            .unwrap()
            .packets
            .len(),
        1
    );
    let mut seed = 0x1d9bu32;
    for at in 11..111 {
        let noise: Vec<_> = (0..MAX_CHUNK_BYTES)
            .map(|_| {
                seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
                (seed >> 16) as u8
            })
            .collect();
        decoder.feed(&noise, Timestamp(at)).unwrap();
        assert!(decoder.buffered_len() < 58);
    }
    assert_eq!(
        decoder.feed(&GOLDEN, Timestamp(200)).unwrap().packets.len(),
        1
    );
}

#[test]
fn configuration_is_explicit_and_strict() {
    for (minimum, maximum, age) in [
        (-1.0, 12.0, 50),
        (12.0, 12.0, 50),
        (0.02, f64::INFINITY, 50),
        (f64::NAN, 12.0, 50),
        (0.02, 12.0, 0),
    ] {
        let mut cfg = config();
        cfg.range_min_m = minimum;
        cfg.range_max_m = maximum;
        cfg.max_packet_age_ms = age;
        assert!(N10Decoder::new(cfg).is_err());
    }
    let mut cfg = config();
    cfg.frame_id = FrameId(String::new());
    assert!(N10Decoder::new(cfg).is_err());
    assert!(serde_json::from_str::<N10Config>(r#"{"frame_id":"laser_link","range_min_m":0.02,"range_max_m":12.0,"max_packet_age_ms":50,"guess_rpm":true}"#).is_err());
}
