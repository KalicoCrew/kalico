//! `RuntimeError` — internal Rust error enum. Spec §5.1.
//!
//! FFI surface maps to `i32` codes per spec §5.2; never crosses C as a Rust
//! type (Rust enum layouts are not stable across compilations).

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
    Internal,
}

// FFI return codes — must match the C-side #define table in spec §5.2.
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
            RuntimeError::BoundaryLoopExhausted
            | RuntimeError::NaNOrInfFromEval
            | RuntimeError::Internal => KALICO_ERR_INTERNAL,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_variant_maps_to_a_distinct_or_grouped_code() {
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
            (RuntimeError::BoundaryLoopExhausted, KALICO_ERR_INTERNAL),
            (RuntimeError::NaNOrInfFromEval, KALICO_ERR_INTERNAL),
            (RuntimeError::Internal, KALICO_ERR_INTERNAL),
        ];
        for (err, expected_code) in mappings {
            assert_eq!(i32::from(err), expected_code, "{err:?}");
        }
    }
}
