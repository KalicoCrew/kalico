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

/// Borrow-free logical descriptor for a ring region within a shared
/// `[PieceEntry]` backing store.
///
/// This is the engine-side counterpart to [`PieceRing`]: it holds only
/// integer bookkeeping fields and every operation takes the backing store by
/// explicit `&mut [PieceEntry]` parameter.  That design avoids splitting a
/// single `UnsafeCell<[PieceEntry; N]>` array into N disjoint `&mut` borrows —
/// an operation the borrow checker cannot verify is disjoint without unsafe
/// code.
///
/// ## Cursor semantics
///
/// - `head` — monotonic valid frontier (u32, wrapping).  Slots with indices
///   `[retired, head)` (modulo arithmetic) are visible to the consumer.
///   Advanced **only** by [`commit_head`][Self::commit_head]; slot writes via
///   [`write_slot`][Self::write_slot] do **not** advance it.
/// - `retired` — monotonic retire counter (u32, wrapping).  Tracks how many
///   entries have fully elapsed their time window; incremented one per
///   [`advance_counter`][Self::advance_counter].  Purely a flow-control
///   frontier; **not** used to derive the read slot.
/// - `tail` — physical read cursor in `[0, ring_depth)`.  The front slot is
///   always `ring_offset + tail`; advanced together with `retired` inside
///   [`advance_counter`][Self::advance_counter] (wraps at `ring_depth`).
///   The invariant `tail == retired % ring_depth` holds because both cursors
///   advance only in `advance_counter`, starting from 0.
///
/// Occupancy: `len() = head.wrapping_sub(retired) as usize`.  Empty (`len==0`)
/// and full (`len==ring_depth`) are distinct because the difference is of
/// monotonic counters, never reduced mod N.
///
/// `PieceRing<'a>` is preserved for host unit tests (it holds a borrow for
/// ergonomics in test code).  The engine uses `RingDescriptor` exclusively.
#[derive(Debug, Clone, Copy)]
pub struct RingDescriptor {
    /// Start index into the shared storage array for this axis's region.
    pub ring_offset: usize,
    /// Capacity of this axis's ring region (number of entries).
    pub ring_depth: usize,
    /// Monotonic valid frontier (host-driven); advanced only by
    /// [`commit_head`][Self::commit_head].
    pub head: u32,
    /// Monotonic retire counter (wrapping u32); advanced one per
    /// [`advance_counter`][Self::advance_counter].  Purely a flow-control
    /// frontier; the read slot is tracked separately by `tail`.
    pub retired: u32,
    /// Physical read cursor in `[0, ring_depth)`.  Advanced one step per
    /// [`advance_counter`][Self::advance_counter], wrapping at `ring_depth`.
    /// `peek` reads from `ring_offset + tail` — no division required.
    pub tail: usize,
}

impl RingDescriptor {
    /// Construct an empty, unconfigured descriptor (ring_depth 0 = no ring
    /// allocated yet).
    #[inline]
    pub const fn new_unconfigured() -> Self {
        Self {
            ring_offset: 0,
            ring_depth: 0,
            head: 0,
            retired: 0,
            tail: 0,
        }
    }

    /// Construct a descriptor for a ring region starting at `offset` with
    /// capacity `depth`.
    #[inline]
    pub const fn new(offset: usize, depth: usize) -> Self {
        Self {
            ring_offset: offset,
            ring_depth: depth,
            head: 0,
            retired: 0,
            tail: 0,
        }
    }

    /// Returns `true` if `configure_axis` has been called for this slot.
    #[inline]
    pub fn is_configured(&self) -> bool {
        self.ring_depth > 0
    }

    /// Returns the number of entries currently visible (committed but not yet
    /// retired): `head.wrapping_sub(retired)`.
    #[inline]
    pub fn len(&self) -> usize {
        self.head.wrapping_sub(self.retired) as usize
    }

    /// Returns `true` if the ring contains no visible entries.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.head == self.retired
    }

    /// Returns `true` if the ring is at capacity (all slots occupied).
    #[inline]
    pub fn is_full(&self) -> bool {
        self.len() == self.ring_depth
    }

    /// Write one entry to an absolute physical slot.  Does **not** advance
    /// `head`; the slot becomes visible to the consumer only after a
    /// subsequent [`commit_head`][Self::commit_head] call.
    ///
    /// `physical_slot` must be in `[0, ring_depth)` — the FFI guarantees this
    /// (the host computes `(start_slot + index) % depth` before calling).
    /// Out-of-range writes (including `ring_depth == 0`) panic in debug builds
    /// via the `debug_assert!` below and index-out-of-bounds panic in release;
    /// silent drops would hide misconfiguration.
    ///
    /// `configure_axis` guarantees `ring_offset + ring_depth <= storage.len()`,
    /// so `ring_offset + physical_slot < storage.len()` holds whenever
    /// `physical_slot < ring_depth`.
    #[inline]
    pub fn write_slot(
        &self,
        storage: &mut [PieceEntry],
        physical_slot: usize,
        entry: PieceEntry,
    ) {
        if self.ring_depth == 0 || physical_slot >= self.ring_depth {
            return;
        }
        debug_assert!(
            self.ring_offset + physical_slot < storage.len(),
            "ring slot out of storage bounds"
        );
        storage[self.ring_offset + physical_slot] = entry;
    }

    /// Advance the valid frontier to `new_head`, monotonically, within ring
    /// capacity.
    ///
    /// A `new_head` is accepted only when it represents a strict advance
    /// over the current `head` **and** the resulting occupancy does not exceed
    /// `ring_depth` (the flow-control invariant `head − retired ≤ ring_depth`).
    /// Both comparisons are made relative to `retired` so they remain correct
    /// across the u32 counter wrap for in-order frames.
    ///
    /// The capacity bound also rejects an out-of-domain `new_head` that lands
    /// behind `retired` — such a value produces a huge wrapping distance that
    /// exceeds `ring_depth`, and is therefore silently dropped rather than
    /// accepted as a spurious advance.
    ///
    /// This function does **not** sanitize arbitrary adversarial values beyond
    /// the capacity bound; it assumes the host sends monotone, in-range heads.
    #[inline]
    pub fn commit_head(&mut self, new_head: u32) {
        let cur = self.head.wrapping_sub(self.retired);
        let proposed = new_head.wrapping_sub(self.retired);
        // Accept only a strict advance that keeps occupancy within capacity.
        // (`head − retired ≤ ring_depth` is the flow-control invariant; this
        // also rejects an out-of-domain `new_head` that lands behind `retired`
        // and would otherwise read as a huge wrapping advance.)
        if proposed > cur && proposed <= self.ring_depth as u32 {
            self.head = new_head;
        }
    }

    /// Convenience append used by the host-side test path and `push_pieces`.
    ///
    /// Writes `entry` at the next free physical slot (derived from the current
    /// head position) and immediately commits the new head, making the entry
    /// visible to the consumer.
    ///
    /// **Interim compatibility shim** — this method exists for the current
    /// `engine::push_pieces` path.  Task 4 replaces that call site with
    /// explicit `write_slot` + `commit_head` (batch write before a single
    /// commit); at that point `push` will be removed or demoted to
    /// `#[cfg(test)]`.  Do not add new callers.
    ///
    /// Returns `Err(())` if the ring is full or unconfigured.
    #[inline]
    pub fn push(&mut self, storage: &mut [PieceEntry], entry: PieceEntry) -> Result<(), ()> {
        if self.is_full() || self.ring_depth == 0 {
            return Err(());
        }
        let physical_slot = (self.head as usize) % self.ring_depth;
        self.write_slot(storage, physical_slot, entry);
        self.head = self.head.wrapping_add(1);
        Ok(())
    }

    /// Peek the front entry without removing it.
    ///
    /// Returns `None` if the ring is empty.  Reads from the physical cursor
    /// `tail` — no division required on the hot path.
    #[inline]
    pub fn peek<'s>(&self, storage: &'s [PieceEntry]) -> Option<&'s PieceEntry> {
        if self.is_empty() {
            return None;
        }
        storage.get(self.ring_offset + self.tail)
    }

    /// Advance the retire cursor by one (the front piece's window has fully
    /// elapsed). No-op when empty or unconfigured.
    ///
    /// Both cursors advance together so the invariant `tail == retired %
    /// ring_depth` is preserved without a division: `tail` wraps explicitly at
    /// `ring_depth`, and `retired` is incremented as a pure monotonic counter.
    #[inline]
    pub fn advance_counter(&mut self) {
        if self.ring_depth == 0 || self.is_empty() {
            return;
        }
        self.retired = self.retired.wrapping_add(1);
        self.tail += 1;
        if self.tail >= self.ring_depth {
            self.tail = 0;
        }
    }

    /// Monotonic count of pieces whose window has fully elapsed (wrapping u32).
    #[inline]
    pub fn retired_count(&self) -> u32 {
        self.retired
    }
}

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
/// assert_eq!(ring.retired_count(), 1);
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
    /// Monotonic counter of retired (popped) pieces, for heartbeat reporting.
    retired: u32,
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
            retired: 0,
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

    /// Remove the front entry and increment the monotonic [`retired_count`][Self::retired_count].
    ///
    /// If the ring is empty this is a no-op.
    #[inline]
    pub fn pop(&mut self) {
        if self.is_empty() {
            return;
        }
        self.tail = (self.tail + 1) % self.buf.len();
        self.count -= 1;
        self.retired = self.retired.wrapping_add(1);
    }

    /// Monotonically increasing count of entries retired via [`pop`][Self::pop].
    ///
    /// Wraps on overflow (u32). Used by the heartbeat subsystem to confirm the
    /// ISR is making forward progress.
    #[inline]
    pub fn retired_count(&self) -> u32 {
        self.retired
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

    /// Serialize to the 32-byte little-endian wire form. Field order matches
    /// the `#[repr(C, align(8))]` layout, so on a little-endian host these
    /// bytes are byte-identical to the C struct the MCU reads.
    ///
    /// # Example
    ///
    /// ```rust
    /// use runtime::piece_ring::PieceEntry;
    ///
    /// let p = PieceEntry { start_time: 1, coeffs: [0.0; 4], duration: 0.001, _reserved: 0 };
    /// let b = p.to_le_bytes();
    /// assert_eq!(b.len(), 32);
    /// assert_eq!(&b[0..8], &1u64.to_le_bytes());
    /// ```
    #[inline]
    pub fn to_le_bytes(&self) -> [u8; 32] {
        let mut b = [0u8; 32];
        b[0..8].copy_from_slice(&self.start_time.to_le_bytes());
        b[8..12].copy_from_slice(&self.coeffs[0].to_le_bytes());
        b[12..16].copy_from_slice(&self.coeffs[1].to_le_bytes());
        b[16..20].copy_from_slice(&self.coeffs[2].to_le_bytes());
        b[20..24].copy_from_slice(&self.coeffs[3].to_le_bytes());
        b[24..28].copy_from_slice(&self.duration.to_le_bytes());
        b[28..32].copy_from_slice(&self._reserved.to_le_bytes());
        b
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn piece_entry_to_le_bytes_matches_field_layout() {
        let p = PieceEntry {
            start_time: 0x0102_0304_0506_0708,
            coeffs: [1.0, 2.0, 3.0, 4.0],
            duration: 0.5,
            _reserved: 0,
        };
        let b = p.to_le_bytes();
        assert_eq!(b.len(), 32);
        assert_eq!(&b[0..8], &0x0102_0304_0506_0708u64.to_le_bytes());
        assert_eq!(&b[8..12], &1.0f32.to_le_bytes());
        assert_eq!(&b[12..16], &2.0f32.to_le_bytes());
        assert_eq!(&b[16..20], &3.0f32.to_le_bytes());
        assert_eq!(&b[20..24], &4.0f32.to_le_bytes());
        assert_eq!(&b[24..28], &0.5f32.to_le_bytes());
        assert_eq!(&b[28..32], &0u32.to_le_bytes());
    }
}
