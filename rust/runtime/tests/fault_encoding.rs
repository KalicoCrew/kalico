//! `fault_detail` encoder tests. Spec §9.2.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use runtime::error::{
    encode_clock_sync_quality, encode_invalid_curve_handle, encode_stream_state_violation,
};

#[test]
fn invalid_curve_handle_encoding() {
    let d = encode_invalid_curve_handle(5, 100, 200);
    assert_eq!(d >> 16, 5);
    assert_eq!(d & 0xFFFF, u32::from(100_u16 ^ 200_u16));
}

#[test]
fn clock_sync_quality_encoding() {
    let d = encode_clock_sync_quality(150, 42);
    assert_eq!(d >> 16, 150);
    assert_eq!(d & 0xFFFF, 42);
}

#[test]
fn stream_state_violation_encoding() {
    let d = encode_stream_state_violation(2, 5);
    assert_eq!(d, (2_u32 << 8) | 5);
}

#[test]
fn invalid_curve_handle_xor_collapses_to_zero_when_match() {
    // If observed_gen == expected_gen, the XOR collapses to 0 — which is
    // the "no-mismatch" diagnostic value. This shouldn't happen at runtime
    // (lookup only fails on mismatch) but the encoder shape must be sound.
    let d = encode_invalid_curve_handle(7, 0xABCD, 0xABCD);
    assert_eq!(d >> 16, 7);
    assert_eq!(d & 0xFFFF, 0);
}

#[test]
fn stream_state_violation_max_bytes_pack() {
    let d = encode_stream_state_violation(0xFF, 0xFF);
    assert_eq!(d, 0x0000_FFFF);
}
