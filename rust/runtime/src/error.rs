//! `RuntimeError` / `FaultCode` — runtime-error and fault-taxonomy enums.
//! Spec §5.1 / §9.1.
//!
//! FFI surface maps to `i32` codes per spec §5.2 (Step-5) and §9.1 (Step-6
//! extensions); never crosses C as a Rust type (Rust enum layouts are not
//! stable across compilations).

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RuntimeError {
    NotInit,
    NullPtr,
    QueueFull,
    InvalidCurve,
    InvalidHandle,
    InvalidDuration,
    InvalidKinematics,
    FaultLatched,
    BoundaryLoopExhausted,
    NaNOrInfFromEval,
    /// §8.2: queue empty while `stream_open == true` — host failed to keep
    /// the runtime fed. Hard fault.
    Underrun,
    NotHomed,
    StepBurstExceeded,
    ZeroDurationSegment,
    Internal,
}

// FFI return codes — must match the C-side #define table in spec §5.2 (Step-5)
// and §9.1 (Step-6 extensions). All Step-5 numeric values are PRESERVED
// (Round-4 fix verifier #2); Step-6 codes start at -100.
pub const KALICO_OK: i32 = 0;
pub const KALICO_ERR_QUEUE_FULL: i32 = -1;
pub const KALICO_ERR_INVALID_CURVE: i32 = -2;
pub const KALICO_ERR_INVALID_HANDLE: i32 = -3;
pub const KALICO_ERR_INVALID_DURATION: i32 = -4;
pub const KALICO_ERR_INVALID_KINEMATICS: i32 = -5;
pub const KALICO_ERR_NULL_PTR: i32 = -6;
pub const KALICO_ERR_NOT_INIT: i32 = -7;
pub const KALICO_ERR_FAULT_LATCHED: i32 = -8;
pub const KALICO_ERR_INTERNAL: i32 = -9;

// Step-6 transport-layer (§9.1).
pub const KALICO_ERR_BAD_CRC: i32 = -100;
pub const KALICO_ERR_FRAMING_VIOLATION: i32 = -101;
pub const KALICO_ERR_DISCONNECT: i32 = -102;
pub const KALICO_ERR_PROTOCOL_VERSION_UNSUPPORTED: i32 = -103;

// Step-6 clock-sync (§9.1).
pub const KALICO_ERR_CLOCK_SYNC_QUALITY: i32 = -110;
pub const KALICO_ERR_CLOCK_SYNC_TIMEOUT: i32 = -111;

// Step-6 multi-MCU coordination (§9.1).
pub const KALICO_ERR_ARM_TIMEOUT: i32 = -120;
pub const KALICO_ERR_ARM_REJECTED: i32 = -121;
pub const KALICO_ERR_CROSS_MCU_DESYNC: i32 = -122;

// Step-6 buffer-budget (§9.1).
pub const KALICO_ERR_UNDERRUN: i32 = -130;
pub const KALICO_ERR_QUEUE_OVERRUN: i32 = -131;
pub const KALICO_ERR_LIVENESS_STALLED: i32 = -132;
pub const KALICO_ERR_TRACE_OVERFLOW: i32 = -133;

// Step-6 protocol/state-machine (§9.1).
pub const KALICO_ERR_STREAM_STATE_VIOLATION: i32 = -140;
pub const KALICO_ERR_SEGMENT_ID_NON_MONOTONIC: i32 = -141;

// Step-6 time-domain (§9.1).
pub const KALICO_ERR_T_START_IN_PAST: i32 = -150;
pub const KALICO_ERR_T_END_BEFORE_T_START: i32 = -151;
pub const KALICO_ERR_SEGMENT_TOO_SHORT: i32 = -152;
pub const KALICO_ERR_SEGMENT_TOO_LONG: i32 = -153;

// Step-6 curve-pool (§9.1).
pub const KALICO_ERR_INVALID_CURVE_HANDLE: i32 = -160;
pub const KALICO_ERR_CURVE_RELOAD_REJECTED: i32 = -161;
pub const KALICO_ERR_CURVE_FORMAT_INVALID: i32 = -162;

// Step-6 runtime-numerical (§9.1).
pub const KALICO_ERR_NAN_INF_OUTPUT: i32 = -170;
pub const KALICO_ERR_BOUNDARY_LOOP_OVERFLOW: i32 = -171;
pub const KALICO_ERR_INTERNAL_INVARIANT: i32 = -172;

// Step 7-B: motion-safety faults.
pub const KALICO_ERR_NOT_HOMED: i32 = -20;
pub const KALICO_ERR_STEP_BURST_EXCEEDED: i32 = -21;
pub const KALICO_ERR_ZERO_DURATION_SEGMENT: i32 = -22;

// Step 7-C-io host-originated faults (§6.11).
pub const KALICO_ERR_HOST_DISCONNECT: i32 = -200;

/// Fault taxonomy. Spec §9.1. Each code has a specific recovery semantic;
/// collapsing to a catch-all loses diagnostic information.
///
/// Round-4 fix (verifier #2): preserve EXISTING Step-5 numeric values; do
/// not reuse `-7` (which is `NotInit`). Step-6 codes start at -100.
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaultCode {
    None = 0,

    // Step-5 carryover (preserve numeric values).
    QueueFull = -1,
    InvalidCurve = -2,
    InvalidHandle = -3,
    InvalidDuration = -4,
    InvalidKinematics = -5,
    NullPtr = -6,
    NotInit = -7,
    FaultLatched = -8,
    Internal = -9,

    // Step-6 transport-layer.
    BadCrc = -100,
    FramingViolation = -101,
    Disconnect = -102,
    ProtocolVersionUnsupported = -103,

    // Step-6 clock-sync.
    ClockSyncQuality = -110,
    ClockSyncTimeout = -111,

    // Step-6 multi-MCU coordination.
    ArmTimeout = -120,
    ArmRejected = -121,
    CrossMcuDesync = -122,

    // Step-6 buffer-budget.
    Underrun = -130,
    QueueOverrun = -131,
    LivenessStalled = -132,
    TraceOverflow = -133,

    // Step-6 protocol / state-machine.
    StreamStateViolation = -140,
    SegmentIdNonMonotonic = -141,

    // Step-6 time-domain.
    TStartInPast = -150,
    TEndBeforeTStart = -151,
    SegmentTooShort = -152,
    SegmentTooLong = -153,

    // Step-6 curve-pool.
    InvalidCurveHandle = -160,
    CurveReloadRejected = -161,
    CurveFormatInvalid = -162,

    // Step-6 runtime-numerical.
    NanInfOutput = -170,
    BoundaryLoopOverflow = -171,
    InternalInvariant = -172,

    // Step 7-B: motion-safety faults.
    NotHomed = -20,
    StepBurstExceeded = -21,
    ZeroDurationSegment = -22,

    // Step 7-C-io host-originated faults (§6.11).
    HostDisconnect = -200,
}

impl FaultCode {
    #[inline]
    pub const fn as_i32(self) -> i32 {
        self as i32
    }

    /// Cast to u16 for the `kalico_status` and `kalico_fault` wire formats
    /// (spec §5.4 / §9.1). Wraps the negative i32 through i16 then u16 so the
    /// host can sign-extend back to i32 if it wants.
    #[inline]
    #[allow(clippy::cast_sign_loss)] // intentional: negative i16 → u16 wire encoding
    pub const fn as_u16(self) -> u16 {
        (self as i32 as i16) as u16
    }
}

impl From<RuntimeError> for i32 {
    fn from(e: RuntimeError) -> i32 {
        match e {
            RuntimeError::NotInit => KALICO_ERR_NOT_INIT,
            RuntimeError::NullPtr => KALICO_ERR_NULL_PTR,
            RuntimeError::QueueFull => KALICO_ERR_QUEUE_FULL,
            RuntimeError::InvalidCurve => KALICO_ERR_INVALID_CURVE,
            RuntimeError::InvalidHandle => KALICO_ERR_INVALID_HANDLE,
            RuntimeError::InvalidDuration => KALICO_ERR_INVALID_DURATION,
            RuntimeError::InvalidKinematics => KALICO_ERR_INVALID_KINEMATICS,
            RuntimeError::FaultLatched => KALICO_ERR_FAULT_LATCHED,
            RuntimeError::Underrun => KALICO_ERR_UNDERRUN,
            RuntimeError::NotHomed => KALICO_ERR_NOT_HOMED,
            RuntimeError::StepBurstExceeded => KALICO_ERR_STEP_BURST_EXCEEDED,
            RuntimeError::ZeroDurationSegment => KALICO_ERR_ZERO_DURATION_SEGMENT,
            RuntimeError::BoundaryLoopExhausted
            | RuntimeError::NaNOrInfFromEval
            | RuntimeError::Internal => KALICO_ERR_INTERNAL,
        }
    }
}

/// Pack the 32-bit `fault_detail` for `KALICO_FAULT_INVALID_CURVE_HANDLE`
/// (spec §9.2): `(slot_idx << 16) | (observed_gen XOR expected_gen)`.
#[inline]
pub const fn encode_invalid_curve_handle(
    slot_idx: u16,
    observed_gen: u16,
    expected_gen: u16,
) -> u32 {
    ((slot_idx as u32) << 16) | ((observed_gen ^ expected_gen) as u32)
}

/// Pack the 32-bit `fault_detail` for `KALICO_FAULT_CLOCK_SYNC_QUALITY`
/// (spec §9.2): `(residual_us << 16) | drift_ppm`.
#[inline]
pub const fn encode_clock_sync_quality(residual_us: u16, drift_ppm: u16) -> u32 {
    ((residual_us as u32) << 16) | (drift_ppm as u32)
}

/// Pack the 32-bit `fault_detail` for `KALICO_FAULT_STREAM_STATE_VIOLATION`
/// (spec §9.2): `(observed << 8) | expected`.
#[inline]
pub const fn encode_stream_state_violation(observed: u8, expected: u8) -> u32 {
    ((observed as u32) << 8) | (expected as u32)
}

#[cfg(test)]
mod tests {
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
            (RuntimeError::NotHomed, KALICO_ERR_NOT_HOMED),
            (RuntimeError::StepBurstExceeded, KALICO_ERR_STEP_BURST_EXCEEDED),
            (RuntimeError::ZeroDurationSegment, KALICO_ERR_ZERO_DURATION_SEGMENT),
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
        assert_eq!(FaultCode::HostDisconnect.as_i32(), KALICO_ERR_HOST_DISCONNECT);
        assert_eq!(KALICO_ERR_HOST_DISCONNECT, -200);
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
}
