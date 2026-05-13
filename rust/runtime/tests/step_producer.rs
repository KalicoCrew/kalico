use runtime::step_producer::{producer_step, ProducerState, ProducerTickResult};
use runtime::step_ring::{StepRing, STEP_RING_CAPACITY};

#[test]
fn producer_fills_ring_from_a_single_linear_curve() {
    // 100 steps over the curve, step_distance 0.1, x(u) = 10·u.
    let curve_eval = |u: f32| {
        let u64 = u as f64;
        (10.0 * u64, 10.0_f64, 0.0_f64)
    };
    let mut ring = StepRing::default();
    let mut state = ProducerState::new(0.1_f64);

    let result = producer_step(
        &mut [&mut ring],
        &mut [&mut state],
        &mut [Some(&curve_eval)],
        &[0_u64],
        &[1_000_000_u64],
        16,
    );

    // Ring should have 16 entries (batch cap reached), more work pending.
    assert_eq!(ring.available(), 16);
    assert_eq!(result, ProducerTickResult::WorkPending);
}

#[test]
fn producer_completes_short_curve_in_one_call() {
    // 5 steps total, step_distance 2.0, x(u) = 10·u.
    let curve_eval = |u: f32| {
        let u64 = u as f64;
        (10.0 * u64, 10.0_f64, 0.0_f64)
    };
    let mut ring = StepRing::default();
    let mut state = ProducerState::new(2.0_f64);

    let result = producer_step(
        &mut [&mut ring],
        &mut [&mut state],
        &mut [Some(&curve_eval)],
        &[0_u64],
        &[1_000_000_u64],
        32,
    );

    // 5 steps emitted; Newton's next call returned SegmentExhausted.
    assert_eq!(ring.available(), 5);
    assert!(state.is_idle(), "expected idle after curve completed");
    assert_eq!(result, ProducerTickResult::AllIdle);
}

#[test]
fn producer_respects_ring_space_backpressure() {
    let curve_eval = |u: f32| {
        let u64 = u as f64;
        (1000.0 * u64, 1000.0_f64, 0.0_f64)
    };
    let mut ring = StepRing::default();
    let mut state = ProducerState::new(0.001_f64); // many steps

    // Pre-fill the ring leaving 10 free slots.
    for _ in 0..(STEP_RING_CAPACITY - 10) {
        ring.push(0, 1);
    }
    let pre_head_available = ring.available();

    producer_step(
        &mut [&mut ring],
        &mut [&mut state],
        &mut [Some(&curve_eval)],
        &[0_u64],
        &[1_000_000_u64],
        100,
    );

    // Should fill exactly the 10 free slots; curve has more work pending.
    assert_eq!(ring.available(), pre_head_available + 10);
    assert!(!state.is_idle());
}
