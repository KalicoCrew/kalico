#![cfg_attr(not(feature = "host"), no_std)]

#[cfg(any(
    all(feature = "mcu-h7", feature = "mcu-f4"),
    all(feature = "mcu-h7", feature = "mcu-g0"),
    all(feature = "mcu-f4", feature = "mcu-g0"),
))]
compile_error!("features `mcu-h7`, `mcu-f4`, and `mcu-g0` are mutually exclusive");

#[cfg(all(
    feature = "host",
    any(feature = "mcu-h7", feature = "mcu-f4", feature = "mcu-g0")
))]
compile_error!("feature `host` is incompatible with `mcu-*` features");

#[cfg(not(any(
    feature = "host",
    feature = "mcu-h7",
    feature = "mcu-f4",
    feature = "mcu-g0"
)))]
compile_error!("must specify exactly one of: `host`, `mcu-h7`, `mcu-f4`, `mcu-g0`");

mod float;
pub use float::Float;

pub mod error;
pub use error::{AlgebraError, ArcLengthError, ConstructError, KnotError, NurbsError, WireError};

mod view;
pub use view::{NurbsView, VectorNurbsView};

mod scalar;
#[cfg(feature = "host")]
pub use scalar::ScalarNurbs;
pub use scalar::ScalarNurbsRef;

mod vector;
#[cfg(feature = "host")]
pub use vector::VectorNurbs;
pub use vector::VectorNurbsRef;

pub mod wire;

pub mod eval;

pub mod arc_length;
#[cfg(feature = "host")]
pub use arc_length::ArcLengthTable;
pub use arc_length::ArcLengthTableRef;

#[cfg(feature = "host")]
pub mod algebra;

#[cfg(feature = "host")]
pub mod knot;
#[cfg(feature = "host")]
pub use knot::KnotVector;

#[cfg(feature = "host")]
pub mod bezier;
#[cfg(feature = "host")]
pub use bezier::BezierPiece;

pub const MAX_DEGREE: usize = 20;

pub const WORKSPACE_SIZE: usize = MAX_DEGREE + 1;

/// Numerical floor for parametric speed |dP/du|, weight denominators, and
/// curvature-divisor cubed-norms. Exposed as f64 so callers and `Float::from_f64`
/// see a single source of truth.
pub const MIN_PARAMETRIC_SPEED: f64 = 1e-9;

const _: () = assert!(WORKSPACE_SIZE == MAX_DEGREE + 1);
const _: () = assert!(MIN_PARAMETRIC_SPEED > 0.0);
