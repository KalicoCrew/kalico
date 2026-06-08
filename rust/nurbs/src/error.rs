use crate::Float;
use core::fmt;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ConstructError {
    DegreeExceeded { actual: u8, max: u8 },
    KnotCountMismatch { expected: usize, got: usize },
    KnotsNotClamped,
    KnotsNotMonotone,
    DegenerateKnotRange,
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
    WeightsUnsupported,
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
            Self::WeightsUnsupported => write!(
                f,
                "wire header has has_weights set; rational curves are unsupported"
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
            Self::ToleranceNotMet {
                achieved_residual,
                samples_used,
            } => write!(
                f,
                "arc-length builder hit cap of {samples_used} samples; achieved residual {achieved_residual:?}"
            ),
            Self::DegenerateCurve => write!(
                f,
                "arc-length integration produced a curve with total length below MIN_PARAMETRIC_SPEED"
            ),
        }
    }
}

impl<T: Float> core::error::Error for ArcLengthError<T> {}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AlgebraError {
    DegreeExceeded { result_degree: u8, max: u8 },
    KnotMismatch,
    NotImplemented(&'static str),
    SupportMismatch,
}

impl fmt::Display for AlgebraError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DegreeExceeded { result_degree, max } => {
                write!(f, "result degree {result_degree} exceeds maximum {max}")
            }
            Self::KnotMismatch => write!(f, "operands have incompatible knot vectors"),
            Self::NotImplemented(s) => write!(f, "algorithm not implemented: {s}"),
            Self::SupportMismatch => write!(f, "Bezier pieces have mismatched support"),
        }
    }
}

impl core::error::Error for AlgebraError {}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum KnotError {
    BoundaryInsertion,
    MultiplicityExceeded {
        existing: u8,
        requested: u8,
        max: u8,
    },
    OutOfRange,
    Invalid,
}

impl fmt::Display for KnotError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::BoundaryInsertion => {
                write!(f, "cannot insert knot at clamped boundary")
            }
            Self::MultiplicityExceeded {
                existing,
                requested,
                max,
            } => {
                write!(
                    f,
                    "knot multiplicity {existing} + {requested} exceeds max {max}"
                )
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
mod tests;
