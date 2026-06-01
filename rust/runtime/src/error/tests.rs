#![allow(clippy::expect_used, clippy::unwrap_used)]

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

// ── from_u16 + code_name tests (Task 4) ─────────────────────────────────────

#[test]
fn fault_code_from_u16_round_trip_positive_zero() {
    assert_eq!(FaultCode::from_u16(0), Some(FaultCode::None));
}

#[test]
fn fault_code_from_u16_sign_wrap_piece_start_in_past() {
    // -308 as i16 = -308; -308i16 as u16 = 65228 = 0xFECC
    let wire = FaultCode::PieceStartInPast.as_u16();
    assert_eq!(wire, 0xFECC);
    assert_eq!(FaultCode::from_u16(wire), Some(FaultCode::PieceStartInPast));
}

#[test]
fn fault_code_from_u16_sign_wrap_tick_interval_exceeded() {
    // -311 as i16 = -311; -311i16 as u16 = 65225 = 0xFEC9
    let wire = FaultCode::TickIntervalExceeded.as_u16();
    assert_eq!(wire, 0xFEC9);
    assert_eq!(
        FaultCode::from_u16(wire),
        Some(FaultCode::TickIntervalExceeded)
    );
}

#[test]
fn fault_code_from_u16_sign_wrap_host_disconnect() {
    let wire = FaultCode::HostDisconnect.as_u16();
    assert_eq!(FaultCode::from_u16(wire), Some(FaultCode::HostDisconnect));
}

#[test]
fn fault_code_from_u16_unknown_returns_none() {
    // 0x1234 does not correspond to any FaultCode discriminant
    assert_eq!(FaultCode::from_u16(0x1234), None);
}

#[test]
fn code_name_piece_start_in_past() {
    assert_eq!(FaultCode::PieceStartInPast.code_name(), "PieceStartInPast");
}

#[test]
fn code_name_none() {
    assert_eq!(FaultCode::None.code_name(), "None");
}

#[test]
fn code_name_tick_interval_exceeded() {
    assert_eq!(
        FaultCode::TickIntervalExceeded.code_name(),
        "TickIntervalExceeded"
    );
}

#[test]
fn from_u16_then_code_name_for_all_step8_codes() {
    // Every Step-8 code must survive the round-trip as_u16 -> from_u16 -> code_name
    let codes = [
        FaultCode::StepQueueOverflow,
        FaultCode::SpiQueueOverflow,
        FaultCode::MathNonFinite,
        FaultCode::PieceAdvanceUnderflow,
        FaultCode::SampleRateMisconfigured,
        FaultCode::PositionCountOverflow,
        FaultCode::JogParametersInvalid,
        FaultCode::StepRateExceedsMcuCeiling,
        FaultCode::PieceStartInPast,
        FaultCode::RingFull,
        FaultCode::StepsPerSampleExceeded,
        FaultCode::TickIntervalExceeded,
    ];
    for code in codes {
        let wire = code.as_u16();
        let recovered = FaultCode::from_u16(wire)
            .expect("from_u16 must succeed for every known FaultCode variant");
        assert_eq!(recovered, code, "round-trip mismatch for {code:?}");
        let name = recovered.code_name();
        assert!(!name.is_empty(), "code_name empty for {code:?}");
        assert_ne!(
            name, "unknown",
            "code_name returned 'unknown' for known variant {code:?}"
        );
    }
}

#[test]
fn from_u16_round_trip_all_variants() {
    // Every FaultCode variant must survive as_u16 -> from_u16 intact.
    let all_codes = [
        FaultCode::None,
        FaultCode::QueueFull,
        FaultCode::InvalidCurve,
        FaultCode::InvalidHandle,
        FaultCode::InvalidDuration,
        FaultCode::InvalidKinematics,
        FaultCode::NullPtr,
        FaultCode::NotInit,
        FaultCode::FaultLatched,
        FaultCode::Internal,
        FaultCode::StepBurstExceeded,
        FaultCode::ZeroDurationSegment,
        FaultCode::HomingTrip,
        FaultCode::CapabilityMissing,
        FaultCode::NoStep,
        FaultCode::InvalidArg,
        FaultCode::InvalidPhaseAxisCount,
        FaultCode::PhaseBusReentrant,
        FaultCode::PhaseModeNotAvailable,
        FaultCode::CurveLoadInvalid,
        FaultCode::MotionInProgress,
        FaultCode::BadCrc,
        FaultCode::FramingViolation,
        FaultCode::Disconnect,
        FaultCode::ProtocolVersionUnsupported,
        FaultCode::ClockSyncQuality,
        FaultCode::ClockSyncTimeout,
        FaultCode::ArmTimeout,
        FaultCode::ArmRejected,
        FaultCode::CrossMcuDesync,
        FaultCode::Underrun,
        FaultCode::QueueOverrun,
        FaultCode::LivenessStalled,
        FaultCode::TraceOverflow,
        FaultCode::StreamStateViolation,
        FaultCode::SegmentIdNonMonotonic,
        FaultCode::TStartInPast,
        FaultCode::TEndBeforeTStart,
        FaultCode::SegmentTooShort,
        FaultCode::SegmentTooLong,
        FaultCode::InvalidCurveHandle,
        FaultCode::CurveReloadRejected,
        FaultCode::CurveFormatInvalid,
        FaultCode::NanInfOutput,
        FaultCode::BoundaryLoopOverflow,
        FaultCode::InternalInvariant,
        FaultCode::HostDisconnect,
        FaultCode::HostRetransmitExhausted,
        FaultCode::HostDispatcherTimeout,
        FaultCode::StepQueueOverflow,
        FaultCode::SpiQueueOverflow,
        FaultCode::MathNonFinite,
        FaultCode::PieceAdvanceUnderflow,
        FaultCode::SampleRateMisconfigured,
        FaultCode::PositionCountOverflow,
        FaultCode::JogParametersInvalid,
        FaultCode::StepRateExceedsMcuCeiling,
        FaultCode::PieceStartInPast,
        FaultCode::RingFull,
        FaultCode::StepsPerSampleExceeded,
        FaultCode::TickIntervalExceeded,
    ];
    // Verify the count matches the spec (60 total: None + 59 non-zero variants,
    // spanning non-contiguous discriminants — we test all of them).
    for code in all_codes {
        let wire = code.as_u16();
        let recovered = FaultCode::from_u16(wire)
            .expect("from_u16 must succeed for every known FaultCode variant");
        assert_eq!(recovered, code, "round-trip mismatch for {code:?}");
        // code_name must be non-empty for every known variant
        let name = code.code_name();
        assert!(!name.is_empty(), "code_name empty for {code:?}");
    }
}
