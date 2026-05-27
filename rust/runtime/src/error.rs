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
    StepBurstExceeded,
    ZeroDurationSegment,
    HomingTrip,
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
pub const KALICO_ERR_STEP_BURST_EXCEEDED: i32 = -21;
pub const KALICO_ERR_ZERO_DURATION_SEGMENT: i32 = -22;
pub const KALICO_ERR_HOMING_TRIP: i32 = -23;
// Step 7-D: step-time scheduling faults.
pub const KALICO_ERR_CAPABILITY_MISSING: i32 = -24;
pub const KALICO_ERR_NO_STEP: i32 = -25;
/// Invalid argument value (e.g. unknown discriminant for a decoded enum).
pub const KALICO_ERR_INVALID_ARG: i32 = -26;

// Phase-stepping configure_axes (Task 4 / spec §3.2, §4.1).
/// Spec §3.2 audible-band protection: at most 2 motors may be configured for
/// phase stepping. Hard parse-time reject when the 33-byte
/// `configure_axes_blob` requests more.
pub const KALICO_ERR_INVALID_PHASE_AXIS_COUNT: i32 = -27;
/// Two phase-stepped motors attempted to share a single SPI bus. Reserved
/// for Task 6 (`runtime_modulated_tick`); declared here so the configure
/// path can future-detect the case at install time.
pub const KALICO_ERR_PHASE_BUS_REENTRANT: i32 = -28;

// Step 7-C-io host-originated faults (§6.11).
pub const KALICO_ERR_HOST_DISCONNECT: i32 = -200;
pub const KALICO_ERR_HOST_RETRANSMIT_EXHAUSTED: i32 = -201;
pub const KALICO_ERR_HOST_DISPATCHER_TIMEOUT: i32 = -202;

// Task 14: two-phase configure_axis faults.
pub const KALICO_ERR_PHASE_MODE_NOT_AVAILABLE: i32 = -29;
pub const KALICO_ERR_CURVE_LOAD_INVALID: i32 = -30;
pub const KALICO_ERR_MOTION_IN_PROGRESS: i32 = -31;

// Step 8: stepping-redesign faults (per docs/superpowers/specs/2026-05-19-stepping-redesign-design.md §9.2).
pub const KALICO_ERR_STEP_QUEUE_OVERFLOW: i32 = -300;
pub const KALICO_ERR_SPI_QUEUE_OVERFLOW: i32 = -301;
pub const KALICO_ERR_MATH_NON_FINITE: i32 = -302;
pub const KALICO_ERR_PIECE_ADVANCE_UNDERFLOW: i32 = -303;
pub const KALICO_ERR_SAMPLE_RATE_MISCONFIGURED: i32 = -304;
pub const KALICO_ERR_POSITION_COUNT_OVERFLOW: i32 = -305;
pub const KALICO_ERR_JOG_PARAMETERS_INVALID: i32 = -306;
pub const KALICO_ERR_STEP_RATE_EXCEEDS_MCU_CEILING: i32 = -307;

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
    StepBurstExceeded = -21,
    ZeroDurationSegment = -22,
    HomingTrip = -23,
    // Step 7-D: step-time scheduling.
    CapabilityMissing = -24,
    NoStep = -25,
    InvalidArg = -26,

    // Task 14: two-phase configure_axis faults.
    /// Phase mode is not yet available (SPI dispatch path is a follow-up).
    PhaseModeNotAvailable = -29,
    /// A curve-load wire blob was structurally invalid.
    CurveLoadInvalid = -30,
    /// A configuration command arrived while a motion segment was in flight.
    MotionInProgress = -31,

    // Step 7-C-io host-originated faults (§6.11).
    HostDisconnect = -200,
    HostRetransmitExhausted = -201,
    HostDispatcherTimeout = -202,

    // Step 8: stepping-redesign faults (per docs/superpowers/specs/2026-05-19-stepping-redesign-design.md §9.2).
    StepQueueOverflow = -300,
    SpiQueueOverflow = -301,
    MathNonFinite = -302,
    PieceAdvanceUnderflow = -303,
    SampleRateMisconfigured = -304,
    PositionCountOverflow = -305,
    JogParametersInvalid = -306,
    StepRateExceedsMcuCeiling = -307,
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
            RuntimeError::StepBurstExceeded => KALICO_ERR_STEP_BURST_EXCEEDED,
            RuntimeError::ZeroDurationSegment => KALICO_ERR_ZERO_DURATION_SEGMENT,
            RuntimeError::HomingTrip => KALICO_ERR_HOMING_TRIP,
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
mod tests;
