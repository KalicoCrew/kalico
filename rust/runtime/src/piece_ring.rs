//! Per-axis piece ring-buffer entry for the MCU ISR.
//!
//! Each [`PieceEntry`] is a 32-byte, 8-byte-aligned record that the host
//! pushes into a ring buffer shared with the MCU ISR. The ISR reads entries
//! in order, converting from Bernstein control-point form to monomial form
//! once on load and then evaluating at 40 kHz via Horner's method.
//!
//! Layout contract (C ABI, matches the corresponding C struct):
//!
//! ```text
//! offset  0 ..  7 : start_time  (u64, little-endian MCU clock cycles)
//! offset  8 .. 11 : coeffs[0]   (f32, Bernstein b0)
//! offset 12 .. 15 : coeffs[1]   (f32, Bernstein b1)
//! offset 16 .. 19 : coeffs[2]   (f32, Bernstein b2)
//! offset 20 .. 23 : coeffs[3]   (f32, Bernstein b3)
//! offset 24 .. 27 : duration     (f32, piece duration in seconds)
//! offset 28 .. 31 : _reserved   (u32, must be zero)
//! total           : 32 bytes, align 8
//! ```
//!
//! # Example
//!
//! ```rust
//! use runtime::piece_ring::PieceEntry;
//!
//! let entry = PieceEntry {
//!     start_time: 0,
//!     coeffs: [0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0],
//!     duration: 0.01,
//!     _reserved: 0,
//! };
//! let (pos, vel) = entry.to_monomial();
//! // pos[1] ≈ 100.0 mm/s (linear ramp rescaled to seconds domain)
//! assert!((pos[1] - 100.0).abs() < 1e-3);
//! ```

use crate::monomial::bernstein_to_monomial_with_duration;

/// A single cubic Bézier piece in Bernstein form, ready to be loaded into the
/// MCU ISR ring buffer.
///
/// See module-level documentation for the field layout and the C ABI contract.
#[derive(Clone, Copy, Debug)]
#[repr(C, align(8))]
pub struct PieceEntry {
    /// Piece start time in MCU clock cycles.
    pub start_time: u64,
    /// Bernstein control points `[b0, b1, b2, b3]`.
    pub coeffs: [f32; 4],
    /// Piece duration in seconds.
    pub duration: f32,
    /// Reserved padding — must be written as zero; the C side may use this
    /// field in a future protocol version.
    pub _reserved: u32,
}

// Compile-time layout assertions — verified at crate compile time for every
// target (host and MCU alike).  We use `const _` blocks rather than a
// dev-dependency so the contract is checked in production builds, not just
// test builds.
const _: () = {
    assert!(core::mem::size_of::<PieceEntry>() == 32);
    assert!(core::mem::align_of::<PieceEntry>() == 8);
};

impl PieceEntry {
    /// Convert Bernstein control points to seconds-domain monomial form.
    ///
    /// Returns `(pos_coeffs, vel_coeffs)` where:
    /// - `pos_coeffs: [f32; 4]` — `[c0, c1, c2, c3]` for
    ///   `P(t) = c0 + c1·t + c2·t² + c3·t³`, `t ∈ [0, duration]`.
    /// - `vel_coeffs: [f32; 3]` — `[vc0, vc1, vc2]` for
    ///   `V(t) = vc0 + vc1·t + vc2·t²`, pre-baked as `[c1, 2c2, 3c3]`.
    ///
    /// The conversion is performed via
    /// [`bernstein_to_monomial_with_duration`][crate::monomial::bernstein_to_monomial_with_duration],
    /// which rescales the unit-interval monomial coefficients to the
    /// seconds domain so that evaluating `P(t_sec)` at a physical elapsed
    /// time `t_sec ∈ [0, self.duration]` yields the correct position.
    #[inline]
    pub fn to_monomial(&self) -> ([f32; 4], [f32; 3]) {
        let m = bernstein_to_monomial_with_duration(self.coeffs, self.duration);
        (m.coeffs, m.vel_coeffs)
    }

    /// Compute the MCU clock cycle at which this piece ends.
    ///
    /// `end = start_time + ⌊duration × clock_freq⌋`
    ///
    /// `clock_freq` is the MCU timer frequency in Hz (e.g. `550_000_000.0`
    /// for the H7 @ 550 MHz).
    ///
    /// # Precision note
    ///
    /// The cast `(self.duration * clock_freq) as u64` truncates toward zero,
    /// which is intentional: the ISR advances to the next piece when
    /// `current_time >= end_time`, so truncating ensures we never overshoot
    /// by a fractional cycle.
    #[inline]
    pub fn end_time(&self, clock_freq: f32) -> u64 {
        #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
        let cycles = (self.duration * clock_freq) as u64;
        self.start_time + cycles
    }
}
