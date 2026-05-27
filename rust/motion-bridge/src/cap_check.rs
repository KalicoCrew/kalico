//! Per-MCU curve-size validation.
//!
//! Spec: docs/superpowers/specs/2026-05-06-runtime-sizing-per-mcu-design.md §5.3.
//!
//! Cubic-only revision (2026-05-20 stepping redesign): the NURBS variant
//! (`fits(caps, &ScalarNurbs)`) was removed along with the rest of the NURBS
//! upload path. Only the cubic-piece validator remains.

use crate::dispatch::McuCaps;
use kalico_host_rt::producer::CurveLoadParams;

/// True if a `CurveLoadParams` payload fits the destination MCU's caps.
/// Cubic-piece curves carry a hard cap of `max_pieces_per_curve` (reported
/// by `QueryRuntimeCaps`) on the firmware side; the validator rejects any
/// upload that would exceed it.
pub fn fits_curve_load(caps: &McuCaps, curve: &CurveLoadParams) -> bool {
    curve.piece_count() <= caps.max_pieces_per_curve as usize
}

#[cfg(test)]
mod tests;
