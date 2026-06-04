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
    Underrun,
    StepBurstExceeded,
    ZeroDurationSegment,
    HomingTrip,
    Internal,
}

// FFI return codes — must match the C-side #define table. All Step-5 numeric
// values are PRESERVED; do not reorder or renumber.
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

pub const KALICO_ERR_BAD_CRC: i32 = -100;
pub const KALICO_ERR_FRAMING_VIOLATION: i32 = -101;
pub const KALICO_ERR_DISCONNECT: i32 = -102;
pub const KALICO_ERR_PROTOCOL_VERSION_UNSUPPORTED: i32 = -103;

pub const KALICO_ERR_CLOCK_SYNC_QUALITY: i32 = -110;
pub const KALICO_ERR_CLOCK_SYNC_TIMEOUT: i32 = -111;

pub const KALICO_ERR_ARM_TIMEOUT: i32 = -120;
pub const KALICO_ERR_ARM_REJECTED: i32 = -121;
pub const KALICO_ERR_CROSS_MCU_DESYNC: i32 = -122;

pub const KALICO_ERR_UNDERRUN: i32 = -130;
pub const KALICO_ERR_QUEUE_OVERRUN: i32 = -131;
pub const KALICO_ERR_LIVENESS_STALLED: i32 = -132;
pub const KALICO_ERR_TRACE_OVERFLOW: i32 = -133;

pub const KALICO_ERR_STREAM_STATE_VIOLATION: i32 = -140;
pub const KALICO_ERR_SEGMENT_ID_NON_MONOTONIC: i32 = -141;

pub const KALICO_ERR_T_START_IN_PAST: i32 = -150;
pub const KALICO_ERR_T_END_BEFORE_T_START: i32 = -151;
pub const KALICO_ERR_SEGMENT_TOO_SHORT: i32 = -152;
pub const KALICO_ERR_SEGMENT_TOO_LONG: i32 = -153;

pub const KALICO_ERR_INVALID_CURVE_HANDLE: i32 = -160;
pub const KALICO_ERR_CURVE_RELOAD_REJECTED: i32 = -161;
pub const KALICO_ERR_CURVE_FORMAT_INVALID: i32 = -162;

pub const KALICO_ERR_NAN_INF_OUTPUT: i32 = -170;
pub const KALICO_ERR_BOUNDARY_LOOP_OVERFLOW: i32 = -171;
pub const KALICO_ERR_INTERNAL_INVARIANT: i32 = -172;

pub const KALICO_ERR_STEP_BURST_EXCEEDED: i32 = -21;
pub const KALICO_ERR_ZERO_DURATION_SEGMENT: i32 = -22;
pub const KALICO_ERR_HOMING_TRIP: i32 = -23;
pub const KALICO_ERR_CAPABILITY_MISSING: i32 = -24;
pub const KALICO_ERR_NO_STEP: i32 = -25;
pub const KALICO_ERR_INVALID_ARG: i32 = -26;

pub const KALICO_ERR_INVALID_PHASE_AXIS_COUNT: i32 = -27;
pub const KALICO_ERR_PHASE_BUS_REENTRANT: i32 = -28;

pub const KALICO_ERR_HOST_DISCONNECT: i32 = -200;
pub const KALICO_ERR_HOST_RETRANSMIT_EXHAUSTED: i32 = -201;
pub const KALICO_ERR_HOST_DISPATCHER_TIMEOUT: i32 = -202;

pub const KALICO_ERR_PHASE_MODE_NOT_AVAILABLE: i32 = -29;
pub const KALICO_ERR_CURVE_LOAD_INVALID: i32 = -30;
pub const KALICO_ERR_MOTION_IN_PROGRESS: i32 = -31;

pub const KALICO_ERR_STEP_QUEUE_OVERFLOW: i32 = -300;
pub const KALICO_ERR_SPI_QUEUE_OVERFLOW: i32 = -301;
pub const KALICO_ERR_MATH_NON_FINITE: i32 = -302;
pub const KALICO_ERR_PIECE_ADVANCE_UNDERFLOW: i32 = -303;
pub const KALICO_ERR_SAMPLE_RATE_MISCONFIGURED: i32 = -304;
pub const KALICO_ERR_POSITION_COUNT_OVERFLOW: i32 = -305;
pub const KALICO_ERR_JOG_PARAMETERS_INVALID: i32 = -306;
pub const KALICO_ERR_STEP_RATE_EXCEEDS_MCU_CEILING: i32 = -307;
/// ISR reached a piece whose `start_time` is more than 2 ISR ticks in the
/// past — MCU was not fed in time. Hard fault.
pub const KALICO_ERR_PIECE_START_IN_PAST: i32 = -308;
pub const KALICO_ERR_RING_FULL: i32 = -309;
/// Steps-per-sample limit exceeded — unrecoverable position-baseline discontinuity.
pub const KALICO_ERR_STEPS_PER_SAMPLE_EXCEEDED: i32 = -310;
/// TIM5 inter-arrival gap exceeded the allowed multiple of `sample_period_cycles`.
/// ISR was starved; fail loud before acting on stale time.
pub const KALICO_ERR_TICK_INTERVAL_EXCEEDED: i32 = -311;
/// `dispatch_axis` encountered a `StepMode` byte that is not `Pulse` (0) or
/// `Phase` (1). Detail: `((axis_idx & 0xFF) << 16) | (mode & 0xFF)`.
pub const KALICO_ERR_UNKNOWN_STEP_MODE: i32 = -312;

#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaultCode {
    None = 0,

    QueueFull = -1,
    InvalidCurve = -2,
    InvalidHandle = -3,
    InvalidDuration = -4,
    InvalidKinematics = -5,
    NullPtr = -6,
    NotInit = -7,
    FaultLatched = -8,
    Internal = -9,

    BadCrc = -100,
    FramingViolation = -101,
    Disconnect = -102,
    ProtocolVersionUnsupported = -103,

    ClockSyncQuality = -110,
    ClockSyncTimeout = -111,

    ArmTimeout = -120,
    ArmRejected = -121,
    CrossMcuDesync = -122,

    Underrun = -130,
    QueueOverrun = -131,
    LivenessStalled = -132,
    TraceOverflow = -133,

    StreamStateViolation = -140,
    SegmentIdNonMonotonic = -141,

    TStartInPast = -150,
    TEndBeforeTStart = -151,
    SegmentTooShort = -152,
    SegmentTooLong = -153,

    InvalidCurveHandle = -160,
    CurveReloadRejected = -161,
    CurveFormatInvalid = -162,

    NanInfOutput = -170,
    BoundaryLoopOverflow = -171,
    InternalInvariant = -172,

    StepBurstExceeded = -21,
    ZeroDurationSegment = -22,
    HomingTrip = -23,
    CapabilityMissing = -24,
    NoStep = -25,
    InvalidArg = -26,

    InvalidPhaseAxisCount = -27,
    PhaseBusReentrant = -28,

    PhaseModeNotAvailable = -29,
    CurveLoadInvalid = -30,
    MotionInProgress = -31,

    HostDisconnect = -200,
    HostRetransmitExhausted = -201,
    HostDispatcherTimeout = -202,

    StepQueueOverflow = -300,
    SpiQueueOverflow = -301,
    MathNonFinite = -302,
    PieceAdvanceUnderflow = -303,
    SampleRateMisconfigured = -304,
    PositionCountOverflow = -305,
    JogParametersInvalid = -306,
    StepRateExceedsMcuCeiling = -307,
    PieceStartInPast = -308,
    RingFull = -309,
    StepsPerSampleExceeded = -310,
    TickIntervalExceeded = -311,
    UnknownStepMode = -312,
}

impl FaultCode {
    #[inline]
    pub const fn as_i32(self) -> i32 {
        self as i32
    }

    /// Cast to u16 for the `kalico_status` and `kalico_fault` wire formats.
    /// Wraps the negative i32 through i16 then u16 so the host can
    /// sign-extend back to i32 if it wants.
    #[inline]
    #[allow(clippy::cast_sign_loss)] // intentional: negative i16 → u16 wire encoding
    pub const fn as_u16(self) -> u16 {
        (self as i32 as i16) as u16
    }

    /// Reconstruct a [`FaultCode`] from its sign-wrapped `u16` wire encoding.
    ///
    /// # Examples
    ///
    /// ```
    /// # use runtime::error::FaultCode;
    /// assert_eq!(FaultCode::from_u16(0), Some(FaultCode::None));
    /// // PieceStartInPast = -308; -308i16 as u16 = 0xFECC
    /// assert_eq!(FaultCode::from_u16(0xFECC), Some(FaultCode::PieceStartInPast));
    /// // TickIntervalExceeded = -311; -311i16 as u16 = 0xFEC9
    /// assert_eq!(FaultCode::from_u16(0xFEC9), Some(FaultCode::TickIntervalExceeded));
    /// assert_eq!(FaultCode::from_u16(0x1234), None);
    /// ```
    #[allow(clippy::cast_possible_wrap)] // intentional: sign-extend u16 → i16 → i32
    pub fn from_u16(v: u16) -> Option<Self> {
        let i = i32::from(v as i16);
        Some(match i {
            0 => Self::None,
            -1 => Self::QueueFull,
            -2 => Self::InvalidCurve,
            -3 => Self::InvalidHandle,
            -4 => Self::InvalidDuration,
            -5 => Self::InvalidKinematics,
            -6 => Self::NullPtr,
            -7 => Self::NotInit,
            -8 => Self::FaultLatched,
            -9 => Self::Internal,
            -21 => Self::StepBurstExceeded,
            -22 => Self::ZeroDurationSegment,
            -23 => Self::HomingTrip,
            -24 => Self::CapabilityMissing,
            -25 => Self::NoStep,
            -26 => Self::InvalidArg,
            -27 => Self::InvalidPhaseAxisCount,
            -28 => Self::PhaseBusReentrant,
            -29 => Self::PhaseModeNotAvailable,
            -30 => Self::CurveLoadInvalid,
            -31 => Self::MotionInProgress,
            -100 => Self::BadCrc,
            -101 => Self::FramingViolation,
            -102 => Self::Disconnect,
            -103 => Self::ProtocolVersionUnsupported,
            -110 => Self::ClockSyncQuality,
            -111 => Self::ClockSyncTimeout,
            -120 => Self::ArmTimeout,
            -121 => Self::ArmRejected,
            -122 => Self::CrossMcuDesync,
            -130 => Self::Underrun,
            -131 => Self::QueueOverrun,
            -132 => Self::LivenessStalled,
            -133 => Self::TraceOverflow,
            -140 => Self::StreamStateViolation,
            -141 => Self::SegmentIdNonMonotonic,
            -150 => Self::TStartInPast,
            -151 => Self::TEndBeforeTStart,
            -152 => Self::SegmentTooShort,
            -153 => Self::SegmentTooLong,
            -160 => Self::InvalidCurveHandle,
            -161 => Self::CurveReloadRejected,
            -162 => Self::CurveFormatInvalid,
            -170 => Self::NanInfOutput,
            -171 => Self::BoundaryLoopOverflow,
            -172 => Self::InternalInvariant,
            -200 => Self::HostDisconnect,
            -201 => Self::HostRetransmitExhausted,
            -202 => Self::HostDispatcherTimeout,
            -300 => Self::StepQueueOverflow,
            -301 => Self::SpiQueueOverflow,
            -302 => Self::MathNonFinite,
            -303 => Self::PieceAdvanceUnderflow,
            -304 => Self::SampleRateMisconfigured,
            -305 => Self::PositionCountOverflow,
            -306 => Self::JogParametersInvalid,
            -307 => Self::StepRateExceedsMcuCeiling,
            -308 => Self::PieceStartInPast,
            -309 => Self::RingFull,
            -310 => Self::StepsPerSampleExceeded,
            -311 => Self::TickIntervalExceeded,
            _ => return None,
        })
    }

    /// Human-readable variant name for use in structured log output.
    ///
    /// # Examples
    ///
    /// ```
    /// # use runtime::error::FaultCode;
    /// assert_eq!(FaultCode::None.code_name(), "None");
    /// assert_eq!(FaultCode::PieceStartInPast.code_name(), "PieceStartInPast");
    /// assert_eq!(FaultCode::TickIntervalExceeded.code_name(), "TickIntervalExceeded");
    /// ```
    pub fn code_name(self) -> &'static str {
        match self {
            Self::None => "None",
            Self::QueueFull => "QueueFull",
            Self::InvalidCurve => "InvalidCurve",
            Self::InvalidHandle => "InvalidHandle",
            Self::InvalidDuration => "InvalidDuration",
            Self::InvalidKinematics => "InvalidKinematics",
            Self::NullPtr => "NullPtr",
            Self::NotInit => "NotInit",
            Self::FaultLatched => "FaultLatched",
            Self::Internal => "Internal",
            Self::StepBurstExceeded => "StepBurstExceeded",
            Self::ZeroDurationSegment => "ZeroDurationSegment",
            Self::HomingTrip => "HomingTrip",
            Self::CapabilityMissing => "CapabilityMissing",
            Self::NoStep => "NoStep",
            Self::InvalidArg => "InvalidArg",
            Self::InvalidPhaseAxisCount => "InvalidPhaseAxisCount",
            Self::PhaseBusReentrant => "PhaseBusReentrant",
            Self::PhaseModeNotAvailable => "PhaseModeNotAvailable",
            Self::CurveLoadInvalid => "CurveLoadInvalid",
            Self::MotionInProgress => "MotionInProgress",
            Self::BadCrc => "BadCrc",
            Self::FramingViolation => "FramingViolation",
            Self::Disconnect => "Disconnect",
            Self::ProtocolVersionUnsupported => "ProtocolVersionUnsupported",
            Self::ClockSyncQuality => "ClockSyncQuality",
            Self::ClockSyncTimeout => "ClockSyncTimeout",
            Self::ArmTimeout => "ArmTimeout",
            Self::ArmRejected => "ArmRejected",
            Self::CrossMcuDesync => "CrossMcuDesync",
            Self::Underrun => "Underrun",
            Self::QueueOverrun => "QueueOverrun",
            Self::LivenessStalled => "LivenessStalled",
            Self::TraceOverflow => "TraceOverflow",
            Self::StreamStateViolation => "StreamStateViolation",
            Self::SegmentIdNonMonotonic => "SegmentIdNonMonotonic",
            Self::TStartInPast => "TStartInPast",
            Self::TEndBeforeTStart => "TEndBeforeTStart",
            Self::SegmentTooShort => "SegmentTooShort",
            Self::SegmentTooLong => "SegmentTooLong",
            Self::InvalidCurveHandle => "InvalidCurveHandle",
            Self::CurveReloadRejected => "CurveReloadRejected",
            Self::CurveFormatInvalid => "CurveFormatInvalid",
            Self::NanInfOutput => "NanInfOutput",
            Self::BoundaryLoopOverflow => "BoundaryLoopOverflow",
            Self::InternalInvariant => "InternalInvariant",
            Self::HostDisconnect => "HostDisconnect",
            Self::HostRetransmitExhausted => "HostRetransmitExhausted",
            Self::HostDispatcherTimeout => "HostDispatcherTimeout",
            Self::StepQueueOverflow => "StepQueueOverflow",
            Self::SpiQueueOverflow => "SpiQueueOverflow",
            Self::MathNonFinite => "MathNonFinite",
            Self::PieceAdvanceUnderflow => "PieceAdvanceUnderflow",
            Self::SampleRateMisconfigured => "SampleRateMisconfigured",
            Self::PositionCountOverflow => "PositionCountOverflow",
            Self::JogParametersInvalid => "JogParametersInvalid",
            Self::StepRateExceedsMcuCeiling => "StepRateExceedsMcuCeiling",
            Self::PieceStartInPast => "PieceStartInPast",
            Self::RingFull => "RingFull",
            Self::StepsPerSampleExceeded => "StepsPerSampleExceeded",
            Self::TickIntervalExceeded => "TickIntervalExceeded",
            Self::UnknownStepMode => "UnknownStepMode",
        }
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

/// Pack the 32-bit `fault_detail` for `KALICO_FAULT_INVALID_CURVE_HANDLE`:
/// `(slot_idx << 16) | (observed_gen XOR expected_gen)`.
#[inline]
pub const fn encode_invalid_curve_handle(
    slot_idx: u16,
    observed_gen: u16,
    expected_gen: u16,
) -> u32 {
    ((slot_idx as u32) << 16) | ((observed_gen ^ expected_gen) as u32)
}

/// Pack the 32-bit `fault_detail` for `KALICO_FAULT_CLOCK_SYNC_QUALITY`:
/// `(residual_us << 16) | drift_ppm`.
#[inline]
pub const fn encode_clock_sync_quality(residual_us: u16, drift_ppm: u16) -> u32 {
    ((residual_us as u32) << 16) | (drift_ppm as u32)
}

/// Pack the 32-bit `fault_detail` for `KALICO_FAULT_STREAM_STATE_VIOLATION`:
/// `(observed << 8) | expected`.
#[inline]
pub const fn encode_stream_state_violation(observed: u8, expected: u8) -> u32 {
    ((observed as u32) << 8) | (expected as u32)
}

#[cfg(test)]
mod tests;
