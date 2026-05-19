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
//! - `LUT_ENTRIES` — `(i_a = sin, i_b = cos)`. Pre-existing, consumed by
//!   `modulator.rs` and the modulator integration tests. Anchor at idx 0
//!   is `(0, 248)`. Do not change without sweeping all consumers.
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
//! two tables will need to be reconciled (probably by retiring
//! `LUT_ENTRIES` once the old modulator is removed).

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
mod tests {
    use super::{COIL_AMPLITUDE, PHASE_LUT, PHASE_LUT_SIZE};

    /// Plan-canonical anchor check: the `(cos, sin)`-ordered LUT must
    /// have its four quadrant points exactly at the amplitude axes.
    #[test]
    fn anchors_match_expectation() {
        assert_eq!(PHASE_LUT[0], (COIL_AMPLITUDE, 0));
        assert_eq!(PHASE_LUT[PHASE_LUT_SIZE / 4], (0, COIL_AMPLITUDE));
        assert_eq!(PHASE_LUT[PHASE_LUT_SIZE / 2], (-COIL_AMPLITUDE, 0));
        assert_eq!(PHASE_LUT[3 * PHASE_LUT_SIZE / 4], (0, -COIL_AMPLITUDE));
    }

    /// Every entry must be inside the i16 amplitude box.
    #[test]
    fn all_entries_within_amplitude() {
        for (i, (a, b)) in PHASE_LUT.iter().enumerate() {
            assert!(
                a.abs() <= COIL_AMPLITUDE,
                "PHASE_LUT[{i}].0 = {a} out of range"
            );
            assert!(
                b.abs() <= COIL_AMPLITUDE,
                "PHASE_LUT[{i}].1 = {b} out of range"
            );
        }
    }

    /// Sanity check on the legacy `(sin, cos)`-ordered table.
    #[test]
    fn legacy_lut_entries_anchors() {
        use super::{CURRENT_AMPLITUDE, LUT_ENTRIES, MOTOR_PERIOD};
        // LUT_ENTRIES[i] = (sin, cos)
        assert_eq!(LUT_ENTRIES[0], (0, CURRENT_AMPLITUDE));
        assert_eq!(LUT_ENTRIES[MOTOR_PERIOD / 4], (CURRENT_AMPLITUDE, 0));
    }
}
