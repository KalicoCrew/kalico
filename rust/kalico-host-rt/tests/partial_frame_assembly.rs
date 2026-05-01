//! A5 — Partial-frame TCP-style read assembly. Spec §3.5.
//!
//! Pure parser test against `host_io::wire::extract_packet`. Five proptest
//! strategies covering mid-length, mid-CRC, mid-payload, multi-frame, and
//! resync-after-corruption paths.

use proptest::prelude::*;

use kalico_host_rt::host_io::wire::{build_frame, extract_packet, MESSAGE_SYNC};

/// Drain every parseable frame from `buf`. Returns the concatenated frames.
fn drain_all(buf: &mut Vec<u8>) -> Vec<Vec<u8>> {
    let mut out = Vec::new();
    while let Some(frame) = extract_packet(buf) {
        out.push(frame);
    }
    out
}

/// Feed `bytes` into `buf` chunk-by-chunk according to `splits` (sorted, all in
/// `[0, bytes.len()]`), draining whenever `extract_packet` succeeds. Returns
/// every frame recovered (in order).
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

// Strategy 1: mid-length-prefix splits.
// Build one frame; split at byte 0 (between length byte and remainder).
proptest! {
    #![proptest_config(ProptestConfig::with_cases(64))]

    #[test]
    fn mid_length_prefix_split_recovers_frame(
        seq in 0u8..16,
        payload_len in 1usize..=20,
    ) {
        let payload = vec![0x42u8; payload_len];
        let frame = build_frame(&payload, seq);
        // Split right after byte 0 (length byte alone in first chunk).
        let (recovered, leftover) = feed_with_splits(&frame, &[1]);
        prop_assert_eq!(recovered.len(), 1);
        prop_assert_eq!(&recovered[0][..], &frame[..]);
        prop_assert!(leftover.is_empty());
    }
}

// Strategy 2: mid-CRC splits.
// Build a frame; split inside the trailing CRC (2 bytes before SYNC).
proptest! {
    #![proptest_config(ProptestConfig::with_cases(64))]

    #[test]
    fn mid_crc_split_recovers_frame(
        seq in 0u8..16,
        payload_len in 1usize..=20,
    ) {
        let payload = vec![0xABu8; payload_len];
        let frame = build_frame(&payload, seq);
        // CRC-high is at frame.len() - 3, CRC-low at len-2, SYNC at len-1.
        let split_in_crc = frame.len() - 2;
        let (recovered, leftover) = feed_with_splits(&frame, &[split_in_crc]);
        prop_assert_eq!(recovered.len(), 1);
        prop_assert!(leftover.is_empty());
    }
}

// Strategy 3: mid-payload splits.
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
        // Header is 2 bytes, so payload starts at index 2.
        let split = 2 + split_at_payload_byte;
        let (recovered, leftover) = feed_with_splits(&frame, &[split]);
        prop_assert_eq!(recovered.len(), 1);
        prop_assert!(leftover.is_empty());
    }
}

// Strategy 4: multi-frame chunks (2-8 frames packed into a single read).
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

// Strategy 5: resync after corruption.
// Insert an invalid byte (chosen so it can't be a valid msglen prefix) before a
// valid frame; assert extract_packet drops bytes one at a time and recovers.
proptest! {
    #![proptest_config(ProptestConfig::with_cases(64))]

    #[test]
    fn resync_after_invalid_prefix_byte(
        invalid in 0u8..=4,           // < MESSAGE_MIN = 5, so msglen check fails
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

// Single-byte-at-a-time stress: feed one byte per iteration; the parser must
// eventually emit the full frame.
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

// Sanity: SYNC byte alone (no preceding frame) is silently dropped by the
// resync path, not interpreted as a frame.
#[test]
fn lone_sync_byte_is_dropped() {
    let mut buf = vec![MESSAGE_SYNC];
    let recovered = drain_all(&mut buf);
    assert!(recovered.is_empty());
    assert!(buf.is_empty());
}
