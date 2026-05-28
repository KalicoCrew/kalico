#![allow(
    clippy::ref_as_ptr,
    clippy::float_cmp,
    clippy::cast_sign_loss,
    clippy::cast_lossless,
    clippy::too_many_lines,
    clippy::uninlined_format_args,
    clippy::doc_markdown
)]

//! Trace-ring `PhaseStep` payload: round-trip serialization through the
//! existing fixed-size `TraceSample` wire format.
//!
//! The plan's snippet (Task 5) sketches `TraceSample::PhaseStep { … }` as if
//! `TraceSample` were a tagged enum, but the actual production type is a
//! `#[repr(C)]` 40-byte struct whose layout is mirrored in the C consumer
//! (`runtime_tick.c` + `kalico_runtime.h` `typedef struct TraceSample`).
//! Reshaping it into a Rust enum would break the `sizeof(TraceSample) == 40`
//! contract.
//!
//! Instead, we add a `TRACE_FLAG_PHASE_STEP` flag bit and pack the
//! `(motor, mscount, i_a, i_b, wrote_spi)` payload into the existing
//! `motor_a` / `motor_b` slots (16 bytes total available; payload is 8
//! bytes). The `TraceSample::phase_step` constructor produces a sample
//! flagged `PHASE_STEP`; `TraceSample::as_phase_step` reverses it.
//!
//! Round-trip equivalence to the plan's `to_bytes` / `from_bytes` pair is
//! provided by the fact that `TraceSample` is `#[repr(C)] + Copy + PartialEq`
//! — copying its 40 bytes to a `[u8; 40]` and back is the wire format.

#![allow(clippy::unwrap_used, unsafe_code)]

use runtime::trace::TraceSample;

#[test]
fn phase_step_sample_round_trip() {
    let sample = TraceSample::phase_step(
        12_345, // tick
        0,      // motor
        512,    // mscount
        0,      // i_a
        248,    // i_b
        true,   // wrote_spi
    );

    // "Serialize" via the existing wire format: TraceSample is
    // `#[repr(C)]` + `Copy`, mirrored bit-for-bit on the C side. The
    // 40-byte representation is the trace-ring wire format.
    let bytes: [u8; 40] = unsafe { core::mem::transmute::<TraceSample, [u8; 40]>(sample) };

    // "Deserialize" — bytewise reconstruct the struct.
    let restored: TraceSample = unsafe { core::mem::transmute::<[u8; 40], TraceSample>(bytes) };

    // Round-trip equivalence: every field, including the flag bits and the
    // packed phase-step payload, survives unchanged.
    assert_eq!(sample, restored);

    // Tag recognition: the restored sample must still decode back to the
    // same logical (motor, mscount, i_a, i_b, wrote_spi) tuple.
    let decoded = restored
        .as_phase_step()
        .expect("flag set → decode succeeds");
    assert_eq!(decoded.tick, 12_345);
    assert_eq!(decoded.motor, 0);
    assert_eq!(decoded.mscount, 512);
    assert_eq!(decoded.i_a, 0);
    assert_eq!(decoded.i_b, 248);
    assert!(decoded.wrote_spi);

    // Negative-i16 and wrote_spi=false also round-trip.
    let neg = TraceSample::phase_step(0xFFFF_FFFF, 3, 0xFFFF, -1, -32_768, false);
    let neg_bytes: [u8; 40] = unsafe { core::mem::transmute::<TraceSample, [u8; 40]>(neg) };
    let neg_back: TraceSample = unsafe { core::mem::transmute::<[u8; 40], TraceSample>(neg_bytes) };
    let neg_decoded = neg_back.as_phase_step().unwrap();
    assert_eq!(neg_decoded.tick, 0xFFFF_FFFF);
    assert_eq!(neg_decoded.motor, 3);
    assert_eq!(neg_decoded.mscount, 0xFFFF);
    assert_eq!(neg_decoded.i_a, -1);
    assert_eq!(neg_decoded.i_b, -32_768);
    assert!(!neg_decoded.wrote_spi);

    // A non-phase-step sample (default flags) must NOT decode as PhaseStep.
    let unrelated = TraceSample::default();
    assert!(unrelated.as_phase_step().is_none());
}
