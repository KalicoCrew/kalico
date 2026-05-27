#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use super::*;
use core::mem::{align_of, offset_of, size_of};

#[test]
fn trace_sample_layout() {
    // Spec §13.2 — these offsets are mirrored in the C smoke build's
    // _Static_assert. Any drift here breaks the C consumer.
    assert_eq!(size_of::<TraceSample>(), 40);
    assert_eq!(align_of::<TraceSample>(), 8);
    assert_eq!(offset_of!(TraceSample, tick), 0);
    assert_eq!(offset_of!(TraceSample, motor_a), 8);
    assert_eq!(offset_of!(TraceSample, motor_b), 12);
    assert_eq!(offset_of!(TraceSample, motor_z), 16);
    assert_eq!(offset_of!(TraceSample, motor_e), 20);
    assert_eq!(offset_of!(TraceSample, segment_id), 24);
    assert_eq!(offset_of!(TraceSample, curve_handle), 28);
    assert_eq!(offset_of!(TraceSample, flags), 32);
}

fn sample(tick: u64, segment_id: u32) -> TraceSample {
    TraceSample {
        tick,
        motor_a: 0.0,
        motor_b: 0.0,
        motor_z: 0.0,
        motor_e: 0.0,
        segment_id,
        curve_handle: CurveHandle::new(0, 0),
        flags: 0,
        _pad: [0; 7],
    }
}

#[test]
fn drain_pulls_in_order() {
    let mut ring = TraceRing::<16>::new();
    for i in 0..5 {
        assert!(ring.try_emit(sample(i, 0)).is_ok());
    }
    let mut out = [TraceSample::default(); 8];
    let n = ring.drain_into(&mut out);
    assert_eq!(n, 5);
    for i in 0..5 {
        assert_eq!(out[i].tick, i as u64);
    }
}

#[test]
fn overflow_carries_into_next_sample() {
    let mut ring = TraceRing::<4>::new(); // effective capacity 3
    // Fill to capacity.
    for i in 0..3 {
        assert!(ring.try_emit(sample(i, 0)).is_ok());
    }
    // 4th emit fails; sets pending overflow flag.
    let r = ring.try_emit(sample(99, 0));
    assert!(r.is_err());
    assert!(ring.has_pending_overflow());

    // Drain everything to free space.
    let mut out = [TraceSample::default(); 8];
    let n = ring.drain_into(&mut out);
    assert_eq!(n, 3);

    // Pending overflow STILL set (drain doesn't clear it).
    assert!(ring.has_pending_overflow());

    // Next successful emit picks up the OVERFLOW flag.
    assert!(ring.try_emit(sample(100, 0)).is_ok());
    let n = ring.drain_into(&mut out);
    assert_eq!(n, 1);
    assert_eq!(out[0].tick, 100);
    assert_ne!(
        out[0].flags & TRACE_FLAG_OVERFLOW,
        0,
        "OVERFLOW must propagate into the next successful sample"
    );

    // After successful enqueue, pending bit cleared.
    assert!(!ring.has_pending_overflow());
}
