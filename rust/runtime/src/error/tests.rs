use super::*;

#[test]
fn every_runtime_error_variant_maps_to_a_distinct_or_grouped_code() {
    let mappings = [
        (RuntimeError::NotInit, KALICO_ERR_NOT_INIT),
        (RuntimeError::NullPtr, KALICO_ERR_NULL_PTR),
        (RuntimeError::QueueFull, KALICO_ERR_QUEUE_FULL),
        (RuntimeError::InvalidCurve, KALICO_ERR_INVALID_CURVE),
        (RuntimeError::InvalidHandle, KALICO_ERR_INVALID_HANDLE),
        (RuntimeError::InvalidDuration, KALICO_ERR_INVALID_DURATION),
        (
            RuntimeError::InvalidKinematics,
            KALICO_ERR_INVALID_KINEMATICS,
        ),
        (RuntimeError::FaultLatched, KALICO_ERR_FAULT_LATCHED),
        (RuntimeError::Underrun, KALICO_ERR_UNDERRUN),
        (
            RuntimeError::StepBurstExceeded,
            KALICO_ERR_STEP_BURST_EXCEEDED,
        ),
        (
            RuntimeError::ZeroDurationSegment,
            KALICO_ERR_ZERO_DURATION_SEGMENT,
        ),
        (RuntimeError::HomingTrip, KALICO_ERR_HOMING_TRIP),
        (RuntimeError::BoundaryLoopExhausted, KALICO_ERR_INTERNAL),
        (RuntimeError::NaNOrInfFromEval, KALICO_ERR_INTERNAL),
        (RuntimeError::Internal, KALICO_ERR_INTERNAL),
    ];
    for (err, expected_code) in mappings {
        assert_eq!(i32::from(err), expected_code, "{err:?}");
    }
}

#[test]
fn fault_code_step5_numeric_values_preserved() {
    assert_eq!(FaultCode::QueueFull.as_i32(), KALICO_ERR_QUEUE_FULL);
    assert_eq!(FaultCode::InvalidCurve.as_i32(), KALICO_ERR_INVALID_CURVE);
    assert_eq!(FaultCode::InvalidHandle.as_i32(), KALICO_ERR_INVALID_HANDLE);
    assert_eq!(FaultCode::NotInit.as_i32(), KALICO_ERR_NOT_INIT);
    assert_eq!(FaultCode::FaultLatched.as_i32(), KALICO_ERR_FAULT_LATCHED);
    assert_eq!(FaultCode::Internal.as_i32(), KALICO_ERR_INTERNAL);
}

#[test]
fn fault_code_step6_numeric_values() {
    assert_eq!(FaultCode::BadCrc.as_i32(), KALICO_ERR_BAD_CRC);
    assert_eq!(
        FaultCode::ProtocolVersionUnsupported.as_i32(),
        KALICO_ERR_PROTOCOL_VERSION_UNSUPPORTED
    );
    assert_eq!(
        FaultCode::ClockSyncQuality.as_i32(),
        KALICO_ERR_CLOCK_SYNC_QUALITY
    );
    assert_eq!(FaultCode::Underrun.as_i32(), KALICO_ERR_UNDERRUN);
    assert_eq!(FaultCode::TraceOverflow.as_i32(), KALICO_ERR_TRACE_OVERFLOW);
    assert_eq!(
        FaultCode::SegmentIdNonMonotonic.as_i32(),
        KALICO_ERR_SEGMENT_ID_NON_MONOTONIC
    );
    assert_eq!(
        FaultCode::InvalidCurveHandle.as_i32(),
        KALICO_ERR_INVALID_CURVE_HANDLE
    );
    assert_eq!(FaultCode::NanInfOutput.as_i32(), KALICO_ERR_NAN_INF_OUTPUT);
}

#[test]
fn host_disconnect_round_trips() {
    assert_eq!(
        FaultCode::HostDisconnect.as_i32(),
        KALICO_ERR_HOST_DISCONNECT
    );
    assert_eq!(KALICO_ERR_HOST_DISCONNECT, -200);
}

#[test]
fn host_retransmit_exhausted_round_trips() {
    assert_eq!(
        FaultCode::HostRetransmitExhausted.as_i32(),
        KALICO_ERR_HOST_RETRANSMIT_EXHAUSTED
    );
    assert_eq!(KALICO_ERR_HOST_RETRANSMIT_EXHAUSTED, -201);
}

#[test]
fn host_dispatcher_timeout_round_trips() {
    assert_eq!(
        FaultCode::HostDispatcherTimeout.as_i32(),
        KALICO_ERR_HOST_DISPATCHER_TIMEOUT
    );
    assert_eq!(KALICO_ERR_HOST_DISPATCHER_TIMEOUT, -202);
}

#[test]
fn host_codes_distinct_from_mcu() {
    assert_ne!(KALICO_ERR_HOST_DISCONNECT, KALICO_ERR_TRACE_OVERFLOW);
    assert_ne!(
        KALICO_ERR_HOST_RETRANSMIT_EXHAUSTED,
        KALICO_ERR_TRACE_OVERFLOW
    );
    assert_ne!(
        KALICO_ERR_HOST_DISPATCHER_TIMEOUT,
        KALICO_ERR_TRACE_OVERFLOW
    );
    assert_ne!(
        KALICO_ERR_HOST_DISCONNECT,
        KALICO_ERR_HOST_RETRANSMIT_EXHAUSTED
    );
    assert_ne!(
        KALICO_ERR_HOST_DISCONNECT,
        KALICO_ERR_HOST_DISPATCHER_TIMEOUT
    );
    assert_ne!(
        KALICO_ERR_HOST_RETRANSMIT_EXHAUSTED,
        KALICO_ERR_HOST_DISPATCHER_TIMEOUT
    );
}

#[test]
fn fault_code_stepping_redesign_numeric_values() {
    // §9.2 — every Step-8 stepping-redesign FaultCode variant must
    // numerically agree with its KALICO_ERR_* twin. Reuse-of-value
    // would silently miscategorize faults to the host.
    assert_eq!(
        FaultCode::StepQueueOverflow.as_i32(),
        KALICO_ERR_STEP_QUEUE_OVERFLOW
    );
    assert_eq!(
        FaultCode::SpiQueueOverflow.as_i32(),
        KALICO_ERR_SPI_QUEUE_OVERFLOW
    );
    assert_eq!(
        FaultCode::MathNonFinite.as_i32(),
        KALICO_ERR_MATH_NON_FINITE
    );
    assert_eq!(
        FaultCode::PieceAdvanceUnderflow.as_i32(),
        KALICO_ERR_PIECE_ADVANCE_UNDERFLOW
    );
    assert_eq!(
        FaultCode::SampleRateMisconfigured.as_i32(),
        KALICO_ERR_SAMPLE_RATE_MISCONFIGURED
    );
    assert_eq!(
        FaultCode::PositionCountOverflow.as_i32(),
        KALICO_ERR_POSITION_COUNT_OVERFLOW
    );
    assert_eq!(
        FaultCode::JogParametersInvalid.as_i32(),
        KALICO_ERR_JOG_PARAMETERS_INVALID
    );
    assert_eq!(
        FaultCode::StepRateExceedsMcuCeiling.as_i32(),
        KALICO_ERR_STEP_RATE_EXCEEDS_MCU_CEILING
    );
    assert_eq!(
        FaultCode::PieceStartInPast.as_i32(),
        KALICO_ERR_PIECE_START_IN_PAST
    );
    assert_eq!(FaultCode::RingFull.as_i32(), KALICO_ERR_RING_FULL);
    assert_eq!(
        FaultCode::StepsPerSampleExceeded.as_i32(),
        KALICO_ERR_STEPS_PER_SAMPLE_EXCEEDED
    );
    assert_eq!(
        FaultCode::TickIntervalExceeded.as_i32(),
        KALICO_ERR_TICK_INTERVAL_EXCEEDED
    );
    // Cross-check: distinct from each other and from the existing
    // -7..-202 range.
    assert_eq!(KALICO_ERR_STEP_QUEUE_OVERFLOW, -300);
    assert_eq!(KALICO_ERR_STEP_RATE_EXCEEDS_MCU_CEILING, -307);
    assert_eq!(KALICO_ERR_TICK_INTERVAL_EXCEEDED, -311);
    assert_ne!(
        KALICO_ERR_STEP_QUEUE_OVERFLOW,
        KALICO_ERR_HOST_DISPATCHER_TIMEOUT
    );
}

#[test]
fn fault_code_as_u16_round_trips_negative_codes() {
    // -160 (InvalidCurveHandle) → as_i16 = -160 → as u16 = 0xFF60.
    // Host sign-extends 0xFF60 back through i16 → -160. clippy::cast_sign_loss
    // is the whole point — the wire format is u16 and the host reverses it.
    let code = FaultCode::InvalidCurveHandle.as_u16();
    #[allow(clippy::cast_sign_loss)]
    let expected = (-160_i16) as u16;
    assert_eq!(code, expected);
}
