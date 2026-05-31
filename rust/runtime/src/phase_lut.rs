//! TMC5160 phase-stepping current LUT.
//!
//! Maps a 10-bit `mscount` (electrical-cycle position, 0..1023) and a
//! direction sign to a `(coil_A, coil_B)` current pair suitable for writing
//! to the TMC5160 `XDIRECT` register. The table is compile-time generated
//! by `build.rs` as a Prusa-faithful identity sinusoid with amplitude
//! `CURRENT_AMPLITUDE = 248` (matches `phase_stepping_opts.h`).
//!
//! For the identity LUT, `direction` is ignored — forward and reverse
//! produce the same currents because the sinusoid is symmetric. Calibration
//! LUTs (silicon follow-up) will introduce per-direction asymmetry to
//! compensate for back-EMF; the lookup signature is preserved here so the
//! call sites do not need to change.
//!
//! ## Two tables, two conventions
//!
//! There are two compile-time tables in this module, both 1024-entry, both
//! amplitude 248, but ordered differently:
//!
//! - `LUT_ENTRIES` — `(i_a = sin, i_b = cos)`. Pre-existing; consumed by
//!   `phase_lut::lookup` (and the `legacy_lut_entries_anchors` test). Anchor
//!   at idx 0 is `(0, 248)`. Do not change without sweeping all consumers.
//!
//! - `PHASE_LUT` — `(coil_A = cos, coil_B = sin)`. Added by the
//!   stepping-redesign work (spec 2026-05-19) so the TIM5 dispatch path
//!   can do `let (coil_a, coil_b) = PHASE_LUT[phase as usize];` with the
//!   plan-canonical anchor `(248, 0)` at idx 0. Consumed by
//!   `crate::tick::dispatch_axis`.
//!
//! Both tables are emitted by `build.rs`; the duplication is intentional
//! and the per-table doc comments here document the swap. When silicon
//! follow-up replaces the identity sinusoid with a calibration LUT, the
//! two tables will need to be reconciled — `LUT_ENTRIES` is now only used
//! by `lookup` and could be retired in that follow-up.

pub const MOTOR_PERIOD: usize = 1024;
pub const CURRENT_AMPLITUDE: i16 = 248;

/// Size of the [`PHASE_LUT`] sinusoid table. Equal to [`MOTOR_PERIOD`]
/// (one electrical cycle = 1024 microsteps = 4 full steps on a TMC5160).
pub const PHASE_LUT_SIZE: usize = MOTOR_PERIOD;

/// Peak coil current amplitude used by [`PHASE_LUT`]. Identical numeric
/// value as [`CURRENT_AMPLITUDE`]; the plan-canonical name is exposed for
/// the stepping-redesign call sites.
pub const COIL_AMPLITUDE: i16 = CURRENT_AMPLITUDE;

include!(concat!(env!("OUT_DIR"), "/phase_lut_table.rs"));

/// Return `(coil_A, coil_B)` for the given electrical-cycle position.
///
/// `mscount` may exceed `MOTOR_PERIOD` — the lookup masks the input to the
/// 10-bit electrical-cycle width so callers don't need to pre-wrap.
/// `direction` is `+1`, `0`, or `-1`; ignored for the identity LUT.
#[inline]
pub fn lookup(mscount: u16, _direction: i8) -> (i16, i16) {
    let idx = (mscount as usize) & (MOTOR_PERIOD - 1);
    #[allow(clippy::indexing_slicing)] // idx masked to in-bounds by the line above
    LUT_ENTRIES[idx]
}

#[cfg(test)]
mod tests;
