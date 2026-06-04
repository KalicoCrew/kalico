// Two compile-time tables, both 1024-entry, amplitude 248, ordered differently:
//
// - `LUT_ENTRIES` — `(i_a = sin, i_b = cos)`. Anchor at idx 0 is `(0, 248)`.
//   Consumed by `phase_lut::lookup`. Do not change without sweeping all consumers.
//
// - `PHASE_LUT` — `(coil_A = cos, coil_B = sin)`. Plan-canonical anchor
//   `(248, 0)` at idx 0. Consumed by the TIM5 dispatch path.
//
// Both are emitted by `build.rs`; the swap is intentional.

pub const MOTOR_PERIOD: usize = 1024;
pub const CURRENT_AMPLITUDE: i16 = 248;

/// Size of the [`PHASE_LUT`] sinusoid table (one electrical cycle = 1024 microsteps).
pub const PHASE_LUT_SIZE: usize = MOTOR_PERIOD;

/// Peak coil current amplitude used by [`PHASE_LUT`].
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
