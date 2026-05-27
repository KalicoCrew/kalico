//! Per-MCU curve-size validation.
//!
//! Spec: docs/superpowers/specs/2026-05-06-runtime-sizing-per-mcu-design.md §5.3.
//!
//! Cubic-only revision (2026-05-20 stepping redesign): the NURBS variant
//! (`fits(caps, &ScalarNurbs)`) was removed along with the rest of the NURBS
//! upload path. Only the cubic-piece validator remains.
//!
//! Simple-MCU-contract revision (2026-05-28): `max_pieces_per_curve` is no
//! longer a direct field on `McuCaps`; the per-curve ceiling is derived from
//! `total_piece_memory` via `McuCaps::total_pieces() / 4`.

use crate::dispatch::McuCaps;
use kalico_host_rt::producer::CurveLoadParams;

/// True if a `CurveLoadParams` payload fits the destination MCU's caps.
/// The per-curve piece ceiling is derived from `McuCaps::total_pieces() / 4`
/// (matching the `effective_max_pieces` derivation in the dispatch closure).
pub fn fits_curve_load(caps: &McuCaps, curve: &CurveLoadParams) -> bool {
    let max_pieces_per_curve = (caps.total_pieces() / 4).max(1);
    curve.piece_count() <= max_pieces_per_curve
}

#[cfg(test)]
mod tests;
