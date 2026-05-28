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

#[test]
fn piece_start_in_past_encodes_axis_and_lateness() {
    let shared = SharedState::new();
    // axis_idx=1, lateness_us=500
    // Expected detail: (1 << 24) | 500 = 0x0100_01F4
    raise_piece_start_in_past(&shared, 1, 500);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::PieceStartInPast.as_i32()
    );
    let detail = shared.fault_detail.load(Ordering::Acquire);
    // axis_idx in bits 24..31
    assert_eq!((detail >> 24) & 0xFF, 1);
    // lateness_us in bits 0..23
    assert_eq!(detail & 0x00FF_FFFF, 500);
}

#[test]
fn piece_start_in_past_lateness_saturates_at_24_bits() {
    let shared = SharedState::new();
    // Pass a value exceeding 24-bit max; should saturate to 0x00FF_FFFF.
    raise_piece_start_in_past(&shared, 0, 0x01FF_FFFF);
    let detail = shared.fault_detail.load(Ordering::Acquire);
    assert_eq!((detail >> 24) & 0xFF, 0);
    assert_eq!(detail & 0x00FF_FFFF, 0x00FF_FFFF);
}

#[test]
fn piece_start_in_past_zero_lateness() {
    let shared = SharedState::new();
    // axis_idx=2, lateness_us=0
    raise_piece_start_in_past(&shared, 2, 0);
    let detail = shared.fault_detail.load(Ordering::Acquire);
    assert_eq!((detail >> 24) & 0xFF, 2);
    assert_eq!(detail & 0x00FF_FFFF, 0);
}
