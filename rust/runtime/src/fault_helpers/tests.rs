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
fn piece_advance_underflow_publishes_code_and_detail() {
    let shared = SharedState::new();
    raise_piece_advance_underflow(&shared, 0);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::PieceAdvanceUnderflow.as_i32()
    );
    assert_eq!(shared.fault_detail.load(Ordering::Acquire), 0x0000_0000);
}

#[test]
fn phase_mode_not_available_publishes_code_and_detail() {
    let shared = SharedState::new();
    raise_phase_mode_not_available(&shared, 1);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::PhaseModeNotAvailable.as_i32()
    );
    assert_eq!(shared.fault_detail.load(Ordering::Acquire), 0x0001_0000);
}

#[test]
fn jog_parameters_invalid_publishes_code_and_zero_detail() {
    let shared = SharedState::new();
    raise_jog_parameters_invalid(&shared);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::JogParametersInvalid.as_i32()
    );
    assert_eq!(shared.fault_detail.load(Ordering::Acquire), 0);
}

#[test]
fn piece_start_in_past_publishes_code_and_detail() {
    let shared = SharedState::new();
    // axis 2, deficit 500 µs → detail = (2 << 16) | 500
    raise_piece_start_in_past(&shared, 2, 500);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::PieceStartInPast.as_i32()
    );
    assert_eq!(
        shared.fault_detail.load(Ordering::Acquire),
        (2 << 16) | 500
    );
}

#[test]
fn piece_start_in_past_saturates_deficit_at_65535() {
    let shared = SharedState::new();
    raise_piece_start_in_past(&shared, 0, 0x1_0000);
    assert_eq!(
        shared.fault_detail.load(Ordering::Acquire) & 0xFFFF,
        0xFFFF
    );
}

#[test]
fn tick_interval_exceeded_publishes_code_and_saturated_detail() {
    let shared = SharedState::new();
    raise_tick_interval_exceeded(&shared, 42);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::TickIntervalExceeded.as_i32()
    );
    assert_eq!(shared.fault_detail.load(Ordering::Acquire), 42);

    // Saturation.
    let shared2 = SharedState::new();
    raise_tick_interval_exceeded(&shared2, 0x1_0000);
    assert_eq!(shared2.fault_detail.load(Ordering::Acquire), 0xFFFF);
}

#[test]
fn steps_per_sample_exceeded_publishes_code_and_detail() {
    let shared = SharedState::new();
    raise_steps_per_sample_exceeded(&shared, 3, 200);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::StepsPerSampleExceeded.as_i32()
    );
    assert_eq!(
        shared.fault_detail.load(Ordering::Acquire),
        (3 << 16) | 200
    );
}

#[test]
fn unknown_step_mode_publishes_code_and_detail() {
    let shared = SharedState::new();
    raise_unknown_step_mode(&shared, 1, 0xAB);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::UnknownStepMode.as_i32()
    );
    assert_eq!(
        shared.fault_detail.load(Ordering::Acquire),
        (1 << 16) | 0xAB
    );
}

/// Prove that `emit_fault_log` compiles and links on the host build (stub path)
/// without panicking. The call must be entirely inert — no extern symbol is
/// referenced, no side-effect visible.
#[test]
fn emit_fault_log_stub_does_not_panic() {
    emit_fault_log(FaultCode::PieceStartInPast, 0x1_0000);
}
