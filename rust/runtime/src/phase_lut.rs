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

pub const MOTOR_PERIOD: usize = 1024;
pub const CURRENT_AMPLITUDE: i16 = 248;

include!(concat!(env!("OUT_DIR"), "/phase_lut_table.rs"));

/// Return `(coil_A, coil_B)` for the given electrical-cycle position.
///
/// `mscount` may exceed `MOTOR_PERIOD` — the lookup masks the input to the
/// 10-bit electrical-cycle width so callers don't need to pre-wrap.
/// `direction` is `+1`, `0`, or `-1`; ignored for the identity LUT.
#[inline]
pub fn lookup(mscount: u16, _direction: i8) -> (i16, i16) {
    let idx = (mscount as usize) & (MOTOR_PERIOD - 1);
    LUT_ENTRIES[idx]
}
