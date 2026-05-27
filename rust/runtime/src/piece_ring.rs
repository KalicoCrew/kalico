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

/// A fixed-capacity SPSC ring buffer for [`PieceEntry`] values.
///
/// Ownership convention:
/// - The **producer** (foreground code) calls [`push`][PieceRing::push].
/// - The **consumer** (40 kHz ISR) calls [`peek`][PieceRing::peek] and
///   [`pop`][PieceRing::pop].
///
/// No lock-free synchronisation is performed — the caller is responsible for
/// ensuring that the producer and consumer do not run concurrently (single
/// core MCU with preemption disabled around push, or ISR-only consumer that
/// only reads after a fence).
///
/// Storage is provided by the caller so the struct is suitable for `no_std`
/// environments with no heap allocator.
///
/// # Example
///
/// ```rust
/// use runtime::piece_ring::{PieceEntry, PieceRing};
///
/// let mut storage = [PieceEntry { start_time: 0, coeffs: [0.0; 4], duration: 0.0, _reserved: 0 }; 4];
/// let mut ring = PieceRing::new(&mut storage);
///
/// let entry = PieceEntry { start_time: 1000, coeffs: [0.0; 4], duration: 0.001, _reserved: 0 };
/// assert!(ring.push(entry).is_ok());
/// assert_eq!(ring.peek().unwrap().start_time, 1000);
/// ring.pop();
/// assert_eq!(ring.consumed_count(), 1);
/// ```
#[derive(Debug)]
pub struct PieceRing<'a> {
    buf: &'a mut [PieceEntry],
    /// Next write position (producer index).
    head: usize,
    /// Next read position (consumer index).
    tail: usize,
    /// Current number of entries in the ring.
    count: usize,
    /// Monotonic counter of consumed (popped) pieces, for heartbeat reporting.
    consumed: u32,
}

impl<'a> PieceRing<'a> {
    /// Construct a new, empty ring backed by `storage`.
    ///
    /// The capacity of the ring equals `storage.len()`.
    #[inline]
    pub fn new(storage: &'a mut [PieceEntry]) -> Self {
        Self {
            buf: storage,
            head: 0,
            tail: 0,
            count: 0,
            consumed: 0,
        }
    }

    /// Returns the maximum number of entries the ring can hold.
    #[inline]
    pub fn capacity(&self) -> usize {
        self.buf.len()
    }

    /// Returns the number of entries currently in the ring.
    #[inline]
    pub fn len(&self) -> usize {
        self.count
    }

    /// Returns `true` if the ring contains no entries.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// Returns `true` if the ring is at capacity.
    #[inline]
    pub fn is_full(&self) -> bool {
        self.count == self.buf.len()
    }

    /// Append `entry` to the back of the ring.
    ///
    /// Returns `Err(())` if the ring is full; the entry is not stored.
    #[inline]
    pub fn push(&mut self, entry: PieceEntry) -> Result<(), ()> {
        if self.is_full() {
            return Err(());
        }
        self.buf[self.head] = entry;
        self.head = (self.head + 1) % self.buf.len();
        self.count += 1;
        Ok(())
    }

    /// Borrow the front entry without removing it.
    ///
    /// Returns `None` if the ring is empty.
    #[inline]
    pub fn peek(&self) -> Option<&PieceEntry> {
        if self.is_empty() {
            return None;
        }
        Some(&self.buf[self.tail])
    }

    /// Remove the front entry and increment the monotonic [`consumed_count`][Self::consumed_count].
    ///
    /// If the ring is empty this is a no-op.
    #[inline]
    pub fn pop(&mut self) {
        if self.is_empty() {
            return;
        }
        self.tail = (self.tail + 1) % self.buf.len();
        self.count -= 1;
        self.consumed = self.consumed.wrapping_add(1);
    }

    /// Monotonically increasing count of entries consumed via [`pop`][Self::pop].
    ///
    /// Wraps on overflow (u32). Used by the heartbeat subsystem to confirm the
    /// ISR is making forward progress.
    #[inline]
    pub fn consumed_count(&self) -> u32 {
        self.consumed
    }
}

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
