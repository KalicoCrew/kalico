//! Sim integration test: F4-config Z jog under step-time scheduling.
//!
//! Verifies that a 1 mm/s Z-only segment at 400 steps/mm produces exactly
//! 400 step pulses over 1 second, each at the expected cycle time within
//! ±10 cycles. Also asserts that `count_modulated_steppers == 0` after
//! `init_test_runtime` (all steppers default to `StepMode::StepTime`), which
//! is the condition under which `runtime_tick_enable` short-circuits on the
//! C side — meaning TIM5 is never armed in this configuration.
//!
//! Run with:
//!   cargo test -p runtime --features kalico-sim --test sim_steptime_z_jog
//!
//! Without `--features kalico-sim` this file compiles to an empty test binary
//! (the `#![cfg(...)]` below excludes all items).

#![cfg(feature = "kalico-sim")]

use core::sync::atomic::Ordering;

use runtime::engine::arm_step_timer_for_stepper;
use runtime::sim_fixtures::{
    TEST_CLOCK_FREQ, TEST_Z_STEPS_PER_MM, init_test_runtime, push_test_segment_linear_z,
};
use runtime::state::{MAX_STEPPER_OIDS, StepMode};

/// Clock + step math for the timing assertions:
///   TEST_CLOCK_FREQ = 180_000_000 Hz
///   Z step resolution = 400 steps/mm (TEST_Z_STEPS_PER_MM)
///   step_distance = 1/400 mm = 0.0025 mm
///   velocity = 1.0 mm/s
///   dt_per_step = step_distance / velocity = 0.0025 s
///   cycles_per_step = 0.0025 × 180_000_000 = 450_000 cycles
///
/// For a degree-3 Bézier with collinear CPs (exactly linear in u), the Newton
/// solver converges in 1 iteration and the spacing is uniform. Step k (1-based)
/// fires at t = 450_000 × k cycles.
const EXPECTED_STEP_SPACING: u64 = 450_000; // 1/400 mm / (1 mm/s) × 180 MHz

#[test]
fn all_steppers_default_to_step_time_and_tim5_never_armed() {
    // init_test_runtime() sets all step_modes to StepTime (SharedState::new()).
    let rt = init_test_runtime();

    let mut modulated_count = 0u32;
    for i in 0..MAX_STEPPER_OIDS {
        let raw = rt.shared.step_modes[i].load(Ordering::Acquire);
        if StepMode::from_u8(raw) == Some(StepMode::Modulated) {
            modulated_count += 1;
        }
    }

    assert_eq!(
        modulated_count, 0,
        "expected count_modulated_steppers == 0 (all StepTime); got {}. \
         TIM5 would have been armed — assertion fails.",
        modulated_count,
    );
}

#[test]
fn z_jog_produces_400_steps_with_correct_timing() {
    let mut rt = init_test_runtime();

    // velocity_mm_s = 1.0, duration_s = 1.0 → covers 0..1 mm on Z.
    push_test_segment_linear_z(&mut rt, /*velocity_mm_s=*/ 1.0, /*duration_s=*/ 1.0);

    // stepper_idx = 2 is the Z motor in Cartesian kinematics.
    let z_stepper_idx: u8 = 2;

    // Sanity-check: 1/TEST_Z_STEPS_PER_MM == 0.0025 mm per step.
    assert!(
        (1.0_f32 / TEST_Z_STEPS_PER_MM - 0.0025).abs() < 1e-8,
        "step_distance sanity check failed"
    );
    // Sanity-check: expected spacing constant is consistent with the two
    // fixture constants above.
    let computed_spacing = (0.0025_f32 * TEST_CLOCK_FREQ as f32) as u64;
    assert_eq!(
        computed_spacing, EXPECTED_STEP_SPACING,
        "EXPECTED_STEP_SPACING inconsistency"
    );

    // Drive the engine forward, collecting every step-pulse cycle.
    //
    // Protocol:
    //   1. Call arm_step_timer_for_stepper(ctx, z_stepper_idx, now_cycles).
    //      The engine reads `stepper_counts[2]` and computes the next step
    //      time relative to `now_cycles` and the current step position.
    //   2. On `Some((t_next, dir))`: record t_next, bump stepper_counts[2],
    //      advance `now_cycles` to `t_next + 1` so the *next* call starts
    //      just past the fired step.
    //   3. On `None` (SegmentExhausted): no more steps in this segment.
    let mut step_times: Vec<u64> = Vec::new();
    let mut now_cycles: u64 = 0;

    loop {
        match arm_step_timer_for_stepper(&rt, z_stepper_idx, now_cycles) {
            Some((t_next, dir)) => {
                assert_eq!(
                    dir, 1i8,
                    "step {}: expected dir=+1 for positive Z velocity, got {}",
                    step_times.len(),
                    dir,
                );
                step_times.push(t_next);

                // Simulate "step fired": the ISR increments stepper_counts so
                // the next call computes from `current_step + 1`.
                rt.shared.stepper_counts[z_stepper_idx as usize]
                    .fetch_add(1, Ordering::AcqRel);

                // Advance simulated clock to just past the step time.
                now_cycles = t_next + 1;
            }
            None => break, // SegmentExhausted — all 400 steps collected.
        }
    }

    // ── Assertion 1: step count ───────────────────────────────────────────
    let expected_steps: usize = (TEST_Z_STEPS_PER_MM * 1.0_f32) as usize; // 400
    assert_eq!(
        step_times.len(),
        expected_steps,
        "expected {} steps over 1 s @ {} steps/mm, got {}",
        expected_steps,
        TEST_Z_STEPS_PER_MM,
        step_times.len(),
    );

    // ── Assertion 2: timing accuracy ─────────────────────────────────────
    // Step k (1-based) fires at EXPECTED_STEP_SPACING × k cycles.
    // Tolerance: ±10 cycles (matches the Newton solver's convergence window
    // established in `step_time_engine.rs` for the first-step case).
    const TOLERANCE_CYCLES: i64 = 10;
    for (i, &t) in step_times.iter().enumerate() {
        let step_number = (i + 1) as u64;
        let expected = EXPECTED_STEP_SPACING * step_number;
        let delta = t as i64 - expected as i64;
        assert!(
            delta.abs() <= TOLERANCE_CYCLES,
            "step {} (1-based {}): fired at {}, expected {} (delta {})",
            i,
            step_number,
            t,
            expected,
            delta,
        );
    }
}
