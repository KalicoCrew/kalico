use proptest::prelude::*;

use kalico_host_rt::host_io::wire::{MESSAGE_SYNC, build_frame, extract_packet};

fn drain_all(buf: &mut Vec<u8>) -> Vec<Vec<u8>> {
    let mut out = Vec::new();
    while let Some(frame) = extract_packet(buf) {
        out.push(frame);
    }
    out
}

fn feed_with_splits(bytes: &[u8], splits: &[usize]) -> (Vec<Vec<u8>>, Vec<u8>) {
    let mut buf: Vec<u8> = Vec::new();
    let mut recovered = Vec::new();
    let mut prev = 0;
    let mut points: Vec<usize> = splits.iter().copied().collect();
    points.push(bytes.len());
    for &p in &points {
        if p > prev {
            buf.extend_from_slice(&bytes[prev..p]);
            recovered.extend(drain_all(&mut buf));
            prev = p;
        }
    }
    (recovered, buf)
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(64))]

    #[test]
    fn mid_length_prefix_split_recovers_frame(
        seq in 0u8..16,
        payload_len in 1usize..=20,
    ) {
        let payload = vec![0x42u8; payload_len];
        let frame = build_frame(&payload, seq);
        let (recovered, leftover) = feed_with_splits(&frame, &[1]);
        prop_assert_eq!(recovered.len(), 1);
        prop_assert_eq!(&recovered[0][..], &frame[..]);
        prop_assert!(leftover.is_empty());
    }
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(64))]

    #[test]
    fn mid_crc_split_recovers_frame(
        seq in 0u8..16,
        payload_len in 1usize..=20,
    ) {
        let payload = vec![0xABu8; payload_len];
        let frame = build_frame(&payload, seq);
        let split_in_crc = frame.len() - 2;
        let (recovered, leftover) = feed_with_splits(&frame, &[split_in_crc]);
        prop_assert_eq!(recovered.len(), 1);
        prop_assert!(leftover.is_empty());
    }
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(64))]

    #[test]
    fn mid_payload_split_recovers_frame(
        seq in 0u8..16,
        payload_len in 4usize..=20,
        split_at_payload_byte in 1usize..3,
    ) {
        let payload: Vec<u8> = (0..payload_len).map(|i| i as u8).collect();
        let frame = build_frame(&payload, seq);
        let split = 2 + split_at_payload_byte;
        let (recovered, leftover) = feed_with_splits(&frame, &[split]);
        prop_assert_eq!(recovered.len(), 1);
        prop_assert!(leftover.is_empty());
    }
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(64))]

    #[test]
    fn multi_frame_single_chunk(count in 2usize..=8) {
        let frames: Vec<Vec<u8>> = (0..count)
            .map(|i| build_frame(&[0xAB, i as u8], (i & 0x0F) as u8))
            .collect();
        let mut buf: Vec<u8> = frames.iter().flatten().copied().collect();
        let recovered = drain_all(&mut buf);
        prop_assert_eq!(recovered.len(), count);
        for (got, expected) in recovered.iter().zip(frames.iter()) {
            prop_assert_eq!(&got[..], &expected[..]);
        }
        prop_assert!(buf.is_empty());
    }
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(64))]

    #[test]
    fn resync_after_invalid_prefix_byte(
        invalid in 0u8..=4,
        seq in 0u8..16,
        payload_byte in any::<u8>(),
    ) {
        let valid = build_frame(&[payload_byte; 4], seq);
        let mut buf = vec![invalid];
        buf.extend_from_slice(&valid);
        let recovered = drain_all(&mut buf);
        prop_assert_eq!(recovered.len(), 1);
        prop_assert_eq!(&recovered[0][..], &valid[..]);
    }
}

#[test]
fn single_byte_at_a_time_recovers_frame() {
    let frame = build_frame(&[0x11, 0x22, 0x33], 5);
    let mut buf = Vec::new();
    let mut recovered = Vec::new();
    for &byte in &frame {
        buf.push(byte);
        recovered.extend(drain_all(&mut buf));
    }
    assert_eq!(recovered.len(), 1);
    assert_eq!(&recovered[0][..], &frame[..]);
    assert!(buf.is_empty());
}

#[test]
fn lone_sync_byte_is_dropped() {
    let mut buf = vec![MESSAGE_SYNC];
    let recovered = drain_all(&mut buf);
    assert!(recovered.is_empty());
    assert!(buf.is_empty());
}
