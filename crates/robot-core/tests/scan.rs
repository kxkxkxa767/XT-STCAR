use std::f64::consts::TAU;
use xt_stcar_robot_core::protocol::n10::{N10Config, N10Decoder};
use xt_stcar_robot_core::scan::{N10RevolutionAssembler, ScanConfig, validate_full_scan};
use xt_stcar_robot_core::{FrameId, LidarSample, Timestamp};

fn config() -> ScanConfig {
    ScanConfig {
        frame_id: FrameId("laser".into()),
        bins: 360,
        min_coverage_fraction: 0.95,
        min_valid_fraction: 0.95,
        max_missing_arc_rad: 0.12,
        max_revolution_ms: 200,
        max_packet_gap_ms: 15,
    }
}

fn wire(start: u16, invalid: bool) -> Vec<u8> {
    let mut bytes = vec![0; 58];
    bytes[..3].copy_from_slice(&[0xa5, 0x5a, 58]);
    bytes[5..7].copy_from_slice(&start.to_be_bytes());
    for slot in 0..16 {
        let range = if invalid { u16::MAX } else { 1000u16 };
        bytes[7 + slot * 3..9 + slot * 3].copy_from_slice(&range.to_be_bytes());
        bytes[9 + slot * 3] = 100;
    }
    bytes[55..57].copy_from_slice(&((start + 1500) % 36000).to_be_bytes());
    bytes[57] = bytes[..57]
        .iter()
        .fold(0u8, |sum, byte| sum.wrapping_add(*byte));
    bytes
}

fn decoder() -> N10Decoder {
    N10Decoder::new(N10Config {
        frame_id: FrameId("laser".into()),
        range_min_m: 0.02,
        range_max_m: 12.0,
        max_packet_age_ms: 50,
    })
    .unwrap()
}

#[test]
fn full_turn_is_emitted_only_at_boundary_with_conservative_timestamp() {
    let mut assembler = N10RevolutionAssembler::new(config()).unwrap();
    let mut decoder = decoder();
    for i in 0..24 {
        let batch = decoder
            .feed(&wire(i * 1500, false), Timestamp(u64::from(i) * 4))
            .unwrap();
        assert_eq!(batch.packets.len(), 1);
        assert!(assembler.push(&batch.packets[0]).unwrap().is_none());
    }
    let batch = decoder.feed(&wire(0, false), Timestamp(96)).unwrap();
    let full = assembler.push(&batch.packets[0]).unwrap().unwrap();
    assert_eq!(full.sample.captured_at, Timestamp(0));
    assert_eq!(full.completed_at, Timestamp(96));
    assert_eq!(full.coverage_fraction, 1.0);
    assert!(full.sample.ranges_m.iter().all(|range| *range == Some(1.0)));
    validate_full_scan(&full.sample, &config()).unwrap();
}

#[test]
fn packet_loss_and_unknown_returns_never_become_a_clear_scan() {
    for drop_packet in [false, true] {
        let mut assembler = N10RevolutionAssembler::new(config()).unwrap();
        let mut decoder = decoder();
        for i in 0..24 {
            if drop_packet && (8..11).contains(&i) {
                continue;
            }
            let batch = decoder
                .feed(
                    &wire(i * 1500, !drop_packet && (8..11).contains(&i)),
                    Timestamp(u64::from(i) * 4),
                )
                .unwrap();
            assert!(assembler.push(&batch.packets[0]).unwrap().is_none());
        }
        let batch = decoder.feed(&wire(0, false), Timestamp(96)).unwrap();
        assert!(assembler.push(&batch.packets[0]).unwrap().is_none());
        assert!(assembler.rejected_revolutions > 0);
    }
}

#[test]
fn missing_sector_across_zero_is_checked_as_one_blind_arc() {
    let mut sample = LidarSample {
        captured_at: Timestamp(0),
        frame_id: FrameId("laser".into()),
        angle_min_rad: 0.0,
        angle_increment_rad: -TAU / 360.0,
        range_min_m: 0.02,
        range_max_m: 12.0,
        ranges_m: vec![Some(3.0); 360],
    };
    validate_full_scan(&sample, &config()).unwrap();
    for i in [357, 358, 359, 0, 1, 2, 3, 4] {
        sample.ranges_m[i] = None;
    }
    assert!(validate_full_scan(&sample, &config()).is_err());
    sample.ranges_m = vec![Some(3.0); 16];
    assert!(validate_full_scan(&sample, &config()).is_err());
}
