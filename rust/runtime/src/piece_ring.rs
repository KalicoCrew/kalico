use crate::monomial::bernstein_to_monomial_with_duration;

/// ## Cursor invariants (ISR/host safety boundary)
///
/// - `head` — monotonic valid frontier (wrapping u32). Advanced **only** by
///   `commit_head`; `write_slot` does **not** advance it.
/// - `retired` — monotonic retire counter (wrapping u32). Incremented one per
///   `advance_counter`. Purely a flow-control frontier.
/// - `tail` — physical read cursor in `[0, ring_depth)`. Invariant:
///   `tail == retired % ring_depth` — both advance only in `advance_counter`
///   starting from 0, so no division is needed on the hot path.
///
/// Occupancy: `head.wrapping_sub(retired)`.
#[derive(Debug, Clone, Copy)]
pub struct RingDescriptor {
    pub ring_offset: usize,
    pub ring_depth: usize,
    pub head: u32,
    pub retired: u32,
    pub tail: usize,
}

impl RingDescriptor {
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

    #[inline]
    pub fn is_configured(&self) -> bool {
        self.ring_depth > 0
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.head.wrapping_sub(self.retired) as usize
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.head == self.retired
    }

    #[inline]
    pub fn is_full(&self) -> bool {
        self.len() == self.ring_depth
    }

    #[inline]
    pub fn write_slot(&self, storage: &mut [PieceEntry], physical_slot: usize, entry: PieceEntry) {
        if self.ring_depth == 0 || physical_slot >= self.ring_depth {
            return;
        }
        debug_assert!(
            self.ring_offset + physical_slot < storage.len(),
            "ring slot out of storage bounds"
        );
        // SAFETY: `configure_axis` guarantees `ring_offset + ring_depth <=
        // storage.len()`; `physical_slot < ring_depth` is checked by the guard
        // at the top of this function, so `ring_offset + physical_slot <
        // storage.len()` holds unconditionally.
        #[allow(clippy::indexing_slicing)]
        {
            storage[self.ring_offset + physical_slot] = entry;
        }
    }

    #[inline]
    pub fn commit_head(&mut self, new_head: u32) {
        let cur = self.head.wrapping_sub(self.retired);
        let proposed = new_head.wrapping_sub(self.retired);
        if proposed > cur && proposed <= self.ring_depth as u32 {
            self.head = new_head;
        }
    }

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

    #[inline]
    pub fn front_slot(&self) -> Option<usize> {
        if self.is_empty() {
            return None;
        }
        Some(self.ring_offset + self.tail)
    }

    #[inline]
    pub fn peek<'s>(&self, storage: &'s [PieceEntry]) -> Option<&'s PieceEntry> {
        if self.is_empty() {
            return None;
        }
        storage.get(self.ring_offset + self.tail)
    }

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

    /// Touches only consumer-owned cursors (`retired`, `tail`) — never `head`.
    #[inline]
    pub fn drain(&mut self) {
        if self.ring_depth == 0 {
            return;
        }
        self.retired = self.head;
        self.tail = (self.head as usize) % self.ring_depth;
    }

    #[inline]
    pub fn retired_count(&self) -> u32 {
        self.retired
    }
}

/// No lock-free synchronisation is performed — the caller is responsible for
/// ensuring that the producer and consumer do not run concurrently (single
/// core MCU with preemption disabled around push, or ISR-only consumer that
/// only reads after a fence).
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
    head: usize,
    tail: usize,
    count: usize,
    retired: u32,
}

impl<'a> PieceRing<'a> {
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

    #[inline]
    pub fn capacity(&self) -> usize {
        self.buf.len()
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.count
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    #[inline]
    pub fn is_full(&self) -> bool {
        self.count == self.buf.len()
    }

    #[inline]
    pub fn push(&mut self, entry: PieceEntry) -> Result<(), ()> {
        if self.is_full() {
            return Err(());
        }
        // SAFETY: `head` is always maintained as `head < buf.len()` — it is
        // initialised to 0 and after every write advances as
        // `(head + 1) % buf.len()`. The `is_full()` guard above ensures at
        // least one free slot exists, so `head` can never equal `buf.len()`.
        #[allow(clippy::indexing_slicing)]
        {
            self.buf[self.head] = entry;
        }
        self.head = (self.head + 1) % self.buf.len();
        self.count += 1;
        Ok(())
    }

    #[inline]
    pub fn peek(&self) -> Option<&PieceEntry> {
        if self.is_empty() {
            return None;
        }
        // SAFETY: `tail` is always maintained as `tail < buf.len()` — it is
        // initialised to 0 and after every `pop` advances as
        // `(tail + 1) % buf.len()`. The `is_empty()` guard above ensures
        // at least one occupied slot exists, so `tail` is a valid index.
        #[allow(clippy::indexing_slicing)]
        Some(&self.buf[self.tail])
    }

    #[inline]
    pub fn pop(&mut self) {
        if self.is_empty() {
            return;
        }
        self.tail = (self.tail + 1) % self.buf.len();
        self.count -= 1;
        self.retired = self.retired.wrapping_add(1);
    }

    #[inline]
    pub fn retired_count(&self) -> u32 {
        self.retired
    }
}

/// Layout contract (C ABI, matches the corresponding C struct):
///
/// ```text
/// offset  0 ..  7 : start_time  (u64, little-endian MCU clock cycles)
/// offset  8 .. 11 : coeffs[0]   (f32, Bernstein b0)
/// offset 12 .. 15 : coeffs[1]   (f32, Bernstein b1)
/// offset 16 .. 19 : coeffs[2]   (f32, Bernstein b2)
/// offset 20 .. 23 : coeffs[3]   (f32, Bernstein b3)
/// offset 24 .. 27 : duration     (f32, piece duration in seconds)
/// offset 28 .. 31 : _reserved   (u32, must be zero)
/// total           : 32 bytes, align 8
/// ```
///
/// # Example
///
/// ```rust
/// use runtime::piece_ring::PieceEntry;
///
/// let entry = PieceEntry {
///     start_time: 0,
///     coeffs: [0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0],
///     duration: 0.01,
///     _reserved: 0,
/// };
/// let (pos, vel) = entry.to_monomial();
/// assert!((pos[1] - 100.0).abs() < 1e-3);
/// ```
#[derive(Clone, Copy, Debug)]
#[repr(C, align(8))]
pub struct PieceEntry {
    pub start_time: u64,
    pub coeffs: [f32; 4],
    pub duration: f32,
    // The underscore prefix signals "intentionally unused by Rust callers"
    // but the field must remain `pub` for `#[repr(C)]` ABI stability (the C
    // side reads this word). The allow suppresses `pub_underscore_fields`.
    #[allow(clippy::pub_underscore_fields)]
    pub _reserved: u32,
}

const _: () = {
    assert!(core::mem::size_of::<PieceEntry>() == 32);
    assert!(core::mem::align_of::<PieceEntry>() == 8);
};

impl PieceEntry {
    #[inline]
    pub fn to_monomial(&self) -> ([f32; 4], [f32; 3]) {
        let m = bernstein_to_monomial_with_duration(self.coeffs, self.duration);
        (m.coeffs, m.vel_coeffs)
    }

    /// `end = start_time + ⌊duration × clock_freq⌋`
    ///
    /// The cast truncates toward zero — the ISR advances to the next piece when
    /// `current_time >= end_time`, so truncating ensures we never overshoot
    /// by a fractional cycle.
    #[inline]
    pub fn end_time(&self, clock_freq: f32) -> u64 {
        #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
        let cycles = (self.duration * clock_freq) as u64;
        self.start_time + cycles
    }

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
mod tests;
