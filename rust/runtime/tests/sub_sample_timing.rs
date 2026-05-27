//! Integration tests for the sub-sample step timing module.
//!
//! Verifies the secant-slope linear interpolation formula
//!   t_k = (step_pos_k - P_start) · sample_period / (P_end - P_start)
//! and the small-displacement uniform-spacing fallback.

use runtime::sub_sample_timing::{
    MAX_STEPS_PER_SAMPLE, StepTimeInputs, StepTimingResult, compute_step_times,
};

// H7 nominal clock — 520 MHz.
const CYCLES_PER_SEC: f32 = 520_000_000.0;
// 25 µs sample period at 520 MHz → 13_000 cycles.
const SAMPLE_PERIOD_SEC: f32 = 25e-6;
const SAMPLE_PERIOD_CYCLES: u32 = 13_000;

const _: () = assert!(MAX_STEPS_PER_SAMPLE >= 16);

#[test]
fn step_times_in_sample_for_constant_velocity() {
    // 4 steps in one 25 µs sample at constant velocity. Expected times:
    //   t_k = (k+1)/4 * sample_period   (k = 0..4)
    // Per-step drift bound: < 10 cycles.
    let inputs = StepTimeInputs {
        p_start: 0.0,
        p_end: 1.0,
        prev_step_count: 0,
        target_step_count: 4,
        microstep_distance: 0.25,
        sample_period_sec: SAMPLE_PERIOD_SEC,
        sample_start_cycles: 0,
        cycles_per_second: CYCLES_PER_SEC,
        displacement_threshold: 1e-3,
    };

    let result = compute_step_times(&inputs);
    let times = match result {
        StepTimingResult::SecantSlope(v) => v,
        other => panic!("expected SecantSlope, got {other:?}"),
    };

    assert_eq!(times.len(), 4, "expected exactly 4 step times");

    for k in 0..4u32 {
        let expected = ((k + 1) as u64 * SAMPLE_PERIOD_CYCLES as u64 / 4u64) as u32;
        let got = times[k as usize];
        let drift = if got > expected {
            got - expected
        } else {
            expected - got
        };
        assert!(
            drift < 10,
            "step {k}: drift {drift} cycles >= 10 (got {got}, expected {expected})"
        );
    }
}

#[test]
fn step_times_within_sample_for_decelerating() {
    // All 4 step times must fall within [0, sample_period_cycles].
    let inputs = StepTimeInputs {
        p_start: 0.0,
        p_end: 1.0,
        prev_step_count: 0,
        target_step_count: 4,
        microstep_distance: 0.25,
        sample_period_sec: SAMPLE_PERIOD_SEC,
        sample_start_cycles: 0,
        cycles_per_second: CYCLES_PER_SEC,
        displacement_threshold: 1e-3,
    };

    let result = compute_step_times(&inputs);
    let times = match result {
        StepTimingResult::SecantSlope(v) => v,
        other => panic!("expected SecantSlope, got {other:?}"),
    };

    assert_eq!(times.len(), 4);
    for (k, &t) in times.iter().enumerate() {
        assert!(
            t <= SAMPLE_PERIOD_CYCLES,
            "step {k} time {t} exceeds sample period {SAMPLE_PERIOD_CYCLES}"
        );
    }
}

#[test]
fn falls_back_to_uniform_when_displacement_too_small() {
    // Sub-threshold displacement (1e-4 < threshold 1e-3) → Uniform variant.
    // Expected: uniform spacing at (k+1)/(n+1) of sample period.
    let inputs = StepTimeInputs {
        p_start: 0.0,
        p_end: 1e-4,
        prev_step_count: 0,
        target_step_count: 3,
        microstep_distance: 1e-4 / 3.0,
        sample_period_sec: SAMPLE_PERIOD_SEC,
        sample_start_cycles: 0,
        cycles_per_second: CYCLES_PER_SEC,
        displacement_threshold: 1e-3,
    };

    let result = compute_step_times(&inputs);
    let times = match result {
        StepTimingResult::Uniform(v) => v,
        other => panic!("expected Uniform, got {other:?}"),
    };

    assert_eq!(times.len(), 3);
    let n = 3u64;
    for k in 0..n {
        let expected = (SAMPLE_PERIOD_CYCLES as u64 * (k + 1) / (n + 1)) as u32;
        assert_eq!(
            times[k as usize], expected,
            "uniform step {k}: expected {expected}, got {}",
            times[k as usize]
        );
    }
}
