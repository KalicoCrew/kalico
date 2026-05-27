use runtime::step::{MAX_STEPS_PER_TICK_DEFAULT, StepMotorState, StepResult};

#[test]
fn zero_delta_produces_no_steps() {
    let mut state = StepMotorState::new(160.0); // 160 steps/mm
    let result = state.update(0.0); // position = 0
    assert_eq!(result.unwrap().n_steps, 0);
}

#[test]
fn one_step_forward() {
    let mut state = StepMotorState::new(160.0);
    let result = state.update(1.0 / 160.0).unwrap();
    assert_eq!(result.n_steps, 1);
}

#[test]
fn four_steps_at_peak_speed() {
    let mut state = StepMotorState::new(160.0);
    let result = state.update(4.0 / 160.0).unwrap();
    assert_eq!(result.n_steps, 4);
}

#[test]
fn negative_steps_on_reversal() {
    let mut state = StepMotorState::new(160.0);
    state.update(10.0 / 160.0).unwrap(); // forward 10 steps
    let result = state.update(7.0 / 160.0).unwrap(); // back 3 steps
    assert_eq!(result.n_steps, -3);
}

#[test]
fn fractional_accumulation() {
    let mut state = StepMotorState::new(160.0);
    let r1 = state.update(0.5 / 160.0).unwrap();
    assert_eq!(r1.n_steps, 0);
    let r2 = state.update(1.0 / 160.0).unwrap();
    assert_eq!(r2.n_steps, 1);
}

#[test]
fn burst_cap_faults() {
    // The MVP burst cap (`MAX_STEPS_PER_TICK_DEFAULT`) was raised on
    // 2026-05-13 to absorb planner-emitted cross-segment discontinuities;
    // pin the fault path by stepping past the current cap explicitly.
    let mut state = StepMotorState::new(160.0);
    let over_cap_mm = ((MAX_STEPS_PER_TICK_DEFAULT as f32) + 1.0) / 160.0;
    let result = state.update(over_cap_mm);
    assert!(result.is_err(), "stepping past the burst cap must fault");
}

#[test]
fn seed_prevents_initial_burst() {
    let mut state = StepMotorState::new(160.0);
    state.seed(50.0);
    let result = state.update(50.0).unwrap();
    assert_eq!(result.n_steps, 0);
}

#[test]
fn drift_over_many_ticks() {
    let mut state = StepMotorState::new(160.0);
    let step_mm = 1.0 / 160.0;
    let mut pos = 0.0_f64;
    let mut total_steps: i64 = 0;
    for _ in 0..1_000_000 {
        pos += step_mm;
        let r = state.update(pos as f32).unwrap();
        total_steps += r.n_steps as i64;
    }
    assert_eq!(total_steps, 1_000_000);
}

// Verify the exported constant has the expected MVP value. The original
// "tight cap" value of 16 was raised on 2026-05-13 (see `rust/runtime/
// src/step.rs`) to absorb planner-emitted cross-segment discontinuities
// observed on bench. Restore the original cap and reinstate this assert
// once the planner-continuity invariant is enforced end-to-end.
#[test]
fn default_burst_cap_value_matches_mvp_raise() {
    assert_eq!(MAX_STEPS_PER_TICK_DEFAULT, 65536);
}

// StepResult must be Debug (compile-time check via format).
#[test]
fn step_result_is_debug() {
    let r = StepResult { n_steps: 3 };
    let _ = format!("{r:?}");
}
