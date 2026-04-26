//! Layer 0 NURBS substrate.
//!
//! See `docs/superpowers/specs/2026-04-26-nurbs-evaluation-library-design.md`.

#![cfg_attr(not(feature = "host"), no_std)]

#[cfg(all(feature = "mcu-h7", feature = "mcu-f4"))]
compile_error!("features `mcu-h7` and `mcu-f4` are mutually exclusive");

#[cfg(all(feature = "host", any(feature = "mcu-h7", feature = "mcu-f4")))]
compile_error!("feature `host` is incompatible with `mcu-*` features");

#[cfg(not(any(feature = "host", feature = "mcu-h7", feature = "mcu-f4")))]
compile_error!("must specify exactly one of: `host`, `mcu-h7`, `mcu-f4`");

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

/// Maximum NURBS degree the crate will accept. See spec §Substrate.
pub const MAX_DEGREE: usize = 20;

/// Stack-workspace size for de Boor's algorithm.
pub const WORKSPACE_SIZE: usize = MAX_DEGREE + 1;

/// Numerical floor for parametric speed |dP/du|, weight denominators, and
/// curvature-divisor cubed-norms. Below this, the corresponding computation
/// either clamps (release) or fires a `debug_assert` (debug).
///
/// Exposed as f64 so callers and `Float::from_f64` see a single source of truth.
pub const MIN_PARAMETRIC_SPEED: f64 = 1e-9;

const _: () = assert!(WORKSPACE_SIZE == MAX_DEGREE + 1);
const _: () = assert!(MIN_PARAMETRIC_SPEED > 0.0);
