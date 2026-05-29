use super::*;

#[test]
fn step_queue_overflow_publishes_code_and_bumps_counter() {
    let shared = SharedState::new();
    raise_step_queue_overflow(&shared, 2);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::StepQueueOverflow.as_i32()
    );
    // axis_idx 2 → 2 << 16 = 0x00020000
    assert_eq!(shared.fault_detail.load(Ordering::Acquire), 0x0002_0000);
    assert_eq!(shared.queue_overflow_count[2].load(Ordering::Acquire), 1);
    // Other axes untouched.
    assert_eq!(shared.queue_overflow_count[0].load(Ordering::Acquire), 0);
}

#[test]
fn step_queue_overflow_out_of_range_axis_does_not_panic() {
    let shared = SharedState::new();
    // 7 is outside the queue_overflow_count[4] range. The fault is
    // still latched but no counter is incremented.
    raise_step_queue_overflow(&shared, 7);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::StepQueueOverflow.as_i32()
    );
    assert_eq!(shared.fault_detail.load(Ordering::Acquire), 0x0007_0000);
}

#[test]
fn position_count_overflow_publishes_code_and_detail() {
    let shared = SharedState::new();
    raise_position_count_overflow(&shared, 1);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::PositionCountOverflow.as_i32()
    );
    assert_eq!(shared.fault_detail.load(Ordering::Acquire), 0x0001_0000);
}

#[test]
fn math_non_finite_publishes_code_and_detail() {
    let shared = SharedState::new();
    raise_math_non_finite(&shared, 3);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::MathNonFinite.as_i32()
    );
    assert_eq!(shared.fault_detail.load(Ordering::Acquire), 0x0003_0000);
}
