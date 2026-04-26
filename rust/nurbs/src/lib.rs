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
