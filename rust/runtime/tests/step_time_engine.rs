//! Engine-level arm_step_timer integration test.
//!
//! This test requires the `kalico-sim` feature because `init_test_runtime`
//! and `push_test_segment_linear_z` live in `runtime::sim_fixtures`, which
//! is gated on that feature.
//!
//! Run with:
//!   cargo test -p runtime --features kalico-sim --test step_time_engine
//!
//! Without `--features kalico-sim` this file compiles to an empty test binary
//! (the `#![cfg(...)]` below excludes all items).

#![cfg(feature = "kalico-sim")]

//! Clock + step math for the assertion:
//!   TEST_CLOCK_FREQ = 180_000_000 Hz  (see sim_fixtures::TEST_CLOCK_FREQ)
//!   Z step resolution = 400 steps/mm  (sim_fixtures::TEST_Z_STEPS_PER_MM)
//!   step_distance = 1/400 mm = 0.0025 mm
//!   velocity = 1.0 mm/s
//!   dt_to_first_step = step_distance / velocity = 0.0025 s
//!   cycles_to_first_step = 0.0025 × 180_000_000 = 450_000 cycles
//!
//! The Newton solver converges in 1 iteration for a linear curve at constant
//! velocity: the initial guess is exact, so `t_next ≈ 450_000`.

use runtime::engine::arm_step_timer_for_stepper;
use runtime::sim_fixtures::{
    TEST_CLOCK_FREQ, TEST_Z_STEPS_PER_MM, init_test_runtime, push_test_segment_linear_z,
    push_test_segment_linear_z_at,
};

#[test]
fn arm_step_timer_returns_first_step_time_on_linear_z() {
    let mut rt = init_test_runtime();
    // velocity_mm_s = 1.0, duration_s = 1.0  →  segment covers 0..1 mm
    push_test_segment_linear_z(&mut rt, /*velocity_mm_s=*/1.0, /*duration_s=*/1.0);

    // stepper_idx = 2 is the Z motor in Cartesian kinematics.
    let z_stepper_idx = 2u8;

    // Expected: first step at 0.0025 mm / (1 mm/s) = 0.0025 s
    // = 0.0025 × TEST_CLOCK_FREQ cycles = 450_000 cycles at 180 MHz.
    let expected_cycles: u64 =
        (0.0025_f32 * TEST_CLOCK_FREQ as f32) as u64; // = 450_000

    // Sanity-check the constant so this assertion self-documents clearly.
    // At 400 steps/mm: step_distance = 1/TEST_Z_STEPS_PER_MM mm.
    assert!(
        (0.0025_f32 - 1.0 / TEST_Z_STEPS_PER_MM).abs() < 1e-8,
        "step_distance sanity: 1/400 = 0.0025"
    );

    let result = arm_step_timer_for_stepper(&rt, z_stepper_idx, /*now_cycles=*/0);

    let (next, dir) = result.expect("expected NextAt for first Z step");
    assert!(
        (next as i64 - expected_cycles as i64).abs() < 10,
        "expected ~{} cycles for first Z step at 1 mm/s with 400 steps/mm, got {}",
        expected_cycles,
        next,
    );
    assert_eq!(dir, 1i8, "positive velocity should give dir=+1");
}

#[test]
fn arm_step_timer_correct_with_large_anchor() {
    // Push a segment starting at 2^30 cycles (~6 seconds into the print at
    // 180 MHz). f32 cannot represent every cycle at this anchor — catastrophic
    // cancellation in the absolute domain would corrupt the result.
    // With normalized-domain arithmetic the subtraction is exact.
    //
    // Expected first step:
    //   step_distance = 1/400 mm = 0.0025 mm
    //   velocity      = 1.0 mm/s
    //   dt_to_step    = 0.0025 s = 0.0025 × 180_000_000 = 450_000 cycles
    let mut rt = init_test_runtime();
    let anchor = 1u64 << 30; // 1_073_741_824 cycles ≈ 5.97 s at 180 MHz
    push_test_segment_linear_z_at(&mut rt, anchor, /*velocity_mm_s=*/1.0, /*duration_s=*/1.0);

    let z_stepper_idx = 2u8;
    let (next, dir) = arm_step_timer_for_stepper(&rt, z_stepper_idx, anchor)
        .expect("expected NextAt at start of segment with large anchor");

    let expected = anchor + 450_000;
    assert!(
        (next as i64 - expected as i64).abs() < 10,
        "expected ~{expected} (anchor={anchor}), got {next}",
    );
    assert_eq!(dir, 1i8, "positive velocity should give dir=+1");
}
