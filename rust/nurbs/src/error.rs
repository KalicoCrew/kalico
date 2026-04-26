//! Per-module error types with From-conversions to top-level `NurbsError`.
//! See spec §Substrate / Error taxonomy.

use crate::Float;
use core::fmt;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ConstructError {
    DegreeExceeded { actual: u8, max: u8 },
    KnotCountMismatch { expected: usize, got: usize },
    KnotsNotClamped,
    KnotsNotMonotone,
    DegenerateKnotRange,
    WeightCountMismatch { expected: usize, got: usize },
    NonPositiveWeight,
}

impl fmt::Display for ConstructError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DegreeExceeded { actual, max } => {
                write!(f, "degree {actual} exceeds maximum {max}")
            }
            Self::KnotCountMismatch { expected, got } => {
                write!(f, "knot count: expected {expected}, got {got}")
            }
            Self::KnotsNotClamped => write!(f, "knot vector is not clamped open"),
            Self::KnotsNotMonotone => write!(f, "knot vector is not non-decreasing"),
            Self::DegenerateKnotRange => {
                write!(f, "knot range is degenerate (knots[last] <= knots[0])")
            }
            Self::WeightCountMismatch { expected, got } => {
                write!(f, "weight count: expected {expected}, got {got}")
            }
            Self::NonPositiveWeight => write!(f, "weight is non-positive"),
        }
    }
}

impl core::error::Error for ConstructError {}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum WireError {
    Misaligned,
    UnknownVersion(u8),
    TruncatedBuffer { expected_len: usize, got: usize },
    AxisCountMismatch { expected: usize, got: u8 },
    Construct(ConstructError),
}

impl fmt::Display for WireError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Misaligned => write!(f, "wire buffer not aligned to T"),
            Self::UnknownVersion(v) => write!(f, "unknown wire format version {v}"),
            Self::TruncatedBuffer { expected_len, got } => write!(
                f,
                "wire buffer truncated: expected {expected_len} bytes, got {got}"
            ),
            Self::AxisCountMismatch { expected, got } => write!(
                f,
                "axis count mismatch: header says {got}, type expects {expected}"
            ),
            Self::Construct(e) => write!(f, "wire content invalid: {e}"),
        }
    }
}

impl core::error::Error for WireError {}

impl From<ConstructError> for WireError {
    fn from(e: ConstructError) -> Self {
        Self::Construct(e)
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum ArcLengthError<T: Float> {
    ToleranceNotMet {
        achieved_residual: T,
        samples_used: usize,
    },
    DegenerateCurve,
}

impl<T: Float> fmt::Display for ArcLengthError<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ToleranceNotMet { achieved_residual, samples_used } =>
                write!(f, "arc-length builder hit cap of {samples_used} samples; achieved residual {achieved_residual:?}"),
            Self::DegenerateCurve => write!(f, "arc-length integration encountered |dP/du| < MIN_PARAMETRIC_SPEED"),
        }
    }
}

impl<T: Float> core::error::Error for ArcLengthError<T> {}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AlgebraError {
    DegreeExceeded { result_degree: u8, max: u8 },
    KnotMismatch,
    NotImplemented(&'static str),
}

impl fmt::Display for AlgebraError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DegreeExceeded { result_degree, max } => {
                write!(f, "result degree {result_degree} exceeds maximum {max}")
            }
            Self::KnotMismatch => write!(f, "operands have incompatible knot vectors"),
            Self::NotImplemented(s) => write!(f, "algorithm not implemented: {s}"),
        }
    }
}

impl core::error::Error for AlgebraError {}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum KnotError {
    BoundaryInsertion,
    MultiplicityExceeded { existing: u8, requested: u8, max: u8 },
    OutOfRange,
    Invalid,
}

impl fmt::Display for KnotError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::BoundaryInsertion => {
                write!(f, "cannot insert knot at clamped boundary")
            }
            Self::MultiplicityExceeded { existing, requested, max } => {
                write!(f, "knot multiplicity {existing} + {requested} exceeds max {max}")
            }
            Self::OutOfRange => write!(f, "knot value out of knot vector range"),
            Self::Invalid => write!(f, "knot vector violates monotone or length invariants"),
        }
    }
}

impl core::error::Error for KnotError {}

#[derive(Debug, Clone, PartialEq)]
pub enum NurbsError<T: Float> {
    Construct(ConstructError),
    Wire(WireError),
    ArcLength(ArcLengthError<T>),
    Algebra(AlgebraError),
    Knot(KnotError),
}

impl<T: Float> fmt::Display for NurbsError<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Construct(e) => write!(f, "{e}"),
            Self::Wire(e) => write!(f, "{e}"),
            Self::ArcLength(e) => write!(f, "{e}"),
            Self::Algebra(e) => write!(f, "{e}"),
            Self::Knot(e) => write!(f, "{e}"),
        }
    }
}

impl<T: Float> core::error::Error for NurbsError<T> {}

impl<T: Float> From<ConstructError> for NurbsError<T> {
    fn from(e: ConstructError) -> Self {
        Self::Construct(e)
    }
}
impl<T: Float> From<WireError> for NurbsError<T> {
    fn from(e: WireError) -> Self {
        Self::Wire(e)
    }
}
impl<T: Float> From<ArcLengthError<T>> for NurbsError<T> {
    fn from(e: ArcLengthError<T>) -> Self {
        Self::ArcLength(e)
    }
}
impl<T: Float> From<AlgebraError> for NurbsError<T> {
    fn from(e: AlgebraError) -> Self {
        Self::Algebra(e)
    }
}
impl<T: Float> From<KnotError> for NurbsError<T> {
    fn from(e: KnotError) -> Self {
        Self::Knot(e)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn construct_error_converts_to_nurbs_error() {
        let e = ConstructError::DegreeExceeded {
            actual: 25,
            max: 20,
        };
        let n: NurbsError<f32> = e.into();
        assert!(matches!(
            n,
            NurbsError::Construct(ConstructError::DegreeExceeded { .. })
        ));
    }

    #[test]
    fn wire_error_wraps_construct_error() {
        let e = ConstructError::KnotsNotMonotone;
        let w: WireError = e.into();
        assert!(matches!(w, WireError::Construct(_)));
    }

    #[test]
    fn nurbs_error_implements_error_trait() {
        let e: NurbsError<f32> = ConstructError::KnotsNotClamped.into();
        let _: &dyn core::error::Error = &e;
    }

    #[test]
    fn display_renders_messages() {
        let e: NurbsError<f32> = ConstructError::DegreeExceeded {
            actual: 30,
            max: 20,
        }
        .into();
        let s = format!("{e}");
        assert!(s.contains("30"));
        assert!(s.contains("20"));
    }

    #[test]
    fn knot_error_converts_to_nurbs_error() {
        let e = KnotError::BoundaryInsertion;
        let n: NurbsError<f64> = e.into();
        assert!(matches!(n, NurbsError::Knot(KnotError::BoundaryInsertion)));
    }

    #[test]
    fn knot_error_displays_clearly() {
        let e = KnotError::MultiplicityExceeded { existing: 2, requested: 2, max: 3 };
        let s = format!("{e}");
        assert!(s.contains("multiplicity"));
        assert!(s.contains('2'));
        assert!(s.contains('3'));
    }
}
