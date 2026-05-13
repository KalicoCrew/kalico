//! Per-motor SPSC append-only ring buffer of step pulse entries.
//!
//! Each entry is a `(cycles_abs_lo, dir)` pair: when (low 32 bits of the MCU
//! cycle counter) and which direction to step. The ring is the contract
//! between the `StepTime` producer (one shared Klipper timer that Newton-fills
//! step times from curves) and the per-stepper consumer (one Klipper timer
//! per motor that fires step pulses at the entry times).
//!
//! See `docs/superpowers/specs/2026-05-14-step-emission-architecture-design.md`
//! §3.3 for the architectural why. The key invariant: a ring entry, once
//! committed by the producer (`head` advance), is guaranteed to fire on the
//! wire — its time and direction are fixed and the consumer reads them
//! independently of any "current segment" state. Neither `head` nor `cursor`
//! ever decreases; neither resets on segment retire. This is the structural
//! fix for the silent step loss caused by the previous per-segment schedule
//! design (boundary loop retiring segments before the consumer drained them).
//!
//! Concurrency model: plain SPSC. The producer mutates the slots and advances
//! `head` with `Release`; the consumer reads `head` with `Acquire`, reads its
//! own `cursor` with `Relaxed`, reads the slot, then advances `cursor` with
//! `Release`. No seqlock — the slot a consumer reads is owned by the consumer
//! until it advances `cursor` past it, so the producer cannot overwrite it.
//!
//! `no_std` compatibility comes from the crate-level attribute in `lib.rs`;
//! the plan listed an inner `#![cfg_attr(not(test), no_std)]` here but Rust
//! warns that crate-level attributes belong in the root module, so it is
//! intentionally omitted.

use core::sync::atomic::{AtomicU32, Ordering};

/// Per-motor ring capacity. Sized per MCU available RAM; 1024 entries × 5 B
/// per motor × 4 motors = 20 KB, fits H7 `axi_ram` and F4 BSS.
pub const STEP_RING_CAPACITY: usize = 1024;

/// SPSC append-only ring of `(cycles_abs_lo, dir)` step pulse entries for one
/// motor.
///
/// Indices wrap modulo `STEP_RING_CAPACITY`. The 32-bit counters wrap modulo
/// 2³² — see [`StepRing::available`] / [`StepRing::space`] for how this is
/// handled with `wrapping_sub`.
#[derive(Debug)]
pub struct StepRing {
    /// Slot storage: when to fire (low 32 bits of MCU cycle counter).
    ///
    /// Crate-private: external callers must go through [`StepRing::push`]
    /// (producer) and [`StepRing::peek_head`] / [`StepRing::peek_next`]
    /// (consumer) so the Acquire/Release barriers on `head` / `cursor` are
    /// always paired correctly.
    pub(crate) cycles_abs_lo: [u32; STEP_RING_CAPACITY],
    /// Slot storage: direction. Valid values pushed by the producer are
    /// `+1` (forward) and `-1` (reverse); `0` is the uninitialized sentinel
    /// written by [`StepRing::new`] and is never observable through
    /// `peek_head` / `peek_next` because `head == cursor` before the
    /// producer's first push (so those accessors short-circuit to `None`).
    ///
    /// Crate-private for the same reason as `cycles_abs_lo` above.
    pub(crate) dirs: [i8; STEP_RING_CAPACITY],
    /// Producer monotonic counter. Advances only via [`StepRing::push`].
    ///
    /// Crate-private: direct mutation would bypass the Release store inside
    /// `push` and break the SPSC publication contract.
    pub(crate) head: AtomicU32,
    /// Consumer monotonic counter. Advances only via [`StepRing::advance`].
    ///
    /// Crate-private: direct mutation would bypass the Release store inside
    /// `advance` and break the SPSC slot-ownership contract.
    pub(crate) cursor: AtomicU32,
}

impl StepRing {
    /// Construct an empty ring. `const`-fn so static-storage instances can be
    /// declared at link time (per-motor rings live in `.axi_bss` on H7,
    /// `.bss` on F4).
    pub const fn new() -> Self {
        Self {
            cycles_abs_lo: [0; STEP_RING_CAPACITY],
            dirs: [0; STEP_RING_CAPACITY],
            head: AtomicU32::new(0),
            cursor: AtomicU32::new(0),
        }
    }

    /// Producer: number of free slots available to write.
    ///
    /// Reads `head` `Relaxed` (own counter) and `cursor` `Acquire` (the
    /// consumer's commit boundary). `saturating_sub` clamps to 0 in the
    /// transient window where the consumer races ahead — which cannot happen
    /// under the SPSC invariant `head - cursor ≤ N`, but is defensive.
    #[inline]
    pub fn space(&self) -> u32 {
        let head = self.head.load(Ordering::Relaxed);
        let cursor = self.cursor.load(Ordering::Acquire);
        (STEP_RING_CAPACITY as u32).saturating_sub(head.wrapping_sub(cursor))
    }

    /// Producer: append one entry. Caller must have verified `space() > 0`.
    ///
    /// The slot is written before `head` is advanced with `Release`, pairing
    /// with the consumer's `Acquire` load of `head` to make the slot
    /// contents visible.
    ///
    /// **SPSC contract.** The `&mut self` receiver is a single-producer
    /// *convention* — it marks that only one producer-side caller exists at
    /// a time. Concurrent consumer access through `&self` is permitted by
    /// the SPSC pattern (the consumer reads slot-array indices the producer
    /// has already finished writing, ordered through the Release barrier on
    /// `head` below, and only mutates `cursor` via atomic ops). The runtime
    /// deployment satisfies this by deriving the `&mut` receiver and the
    /// consumer's `&self` from an `UnsafeCell` projection inside the
    /// engine's shared state; the type-level `&mut` / `&` distinction here
    /// records the producer / consumer roles, not Rust's exclusive-borrow
    /// guarantee.
    #[allow(clippy::indexing_slicing)]
    pub fn push(&mut self, cycles_abs_lo: u32, dir: i8) {
        let head = self.head.load(Ordering::Relaxed);
        let slot = (head as usize) % STEP_RING_CAPACITY;
        self.cycles_abs_lo[slot] = cycles_abs_lo;
        self.dirs[slot] = dir;
        self.head.store(head.wrapping_add(1), Ordering::Release);
    }

    /// Consumer: number of entries available to read.
    ///
    /// Reads `head` `Acquire` (pairs with the producer's `Release` store) and
    /// `cursor` `Relaxed` (own counter). The `wrapping_sub` handles the
    /// 32-bit counter wrap: as long as the SPSC invariant
    /// `head - cursor ≤ N` holds (with N ≪ 2³¹), the wrap-aware difference
    /// is the true number of pending entries.
    #[inline]
    pub fn available(&self) -> u32 {
        let head = self.head.load(Ordering::Acquire);
        let cursor = self.cursor.load(Ordering::Relaxed);
        head.wrapping_sub(cursor)
    }

    /// Consumer: peek the entry at the cursor without advancing. Returns
    /// `None` if the ring is empty.
    #[allow(clippy::indexing_slicing)]
    pub fn peek_head(&self) -> Option<(u32, i8)> {
        let head = self.head.load(Ordering::Acquire);
        let cursor = self.cursor.load(Ordering::Relaxed);
        if head == cursor {
            return None;
        }
        let slot = (cursor as usize) % STEP_RING_CAPACITY;
        Some((self.cycles_abs_lo[slot], self.dirs[slot]))
    }

    /// Consumer: peek the *second* entry (the one after the cursor's head).
    /// Returns `None` if fewer than two entries are available.
    ///
    /// Used by the per-stepper consumer to decide whether the next step
    /// pulse already has a known direction (no DIR flip required) or whether
    /// the producer hasn't yet caught up.
    #[allow(clippy::indexing_slicing)]
    pub fn peek_next(&self) -> Option<(u32, i8)> {
        let head = self.head.load(Ordering::Acquire);
        let cursor = self.cursor.load(Ordering::Relaxed);
        if head.wrapping_sub(cursor) < 2 {
            return None;
        }
        let slot = (cursor.wrapping_add(1) as usize) % STEP_RING_CAPACITY;
        Some((self.cycles_abs_lo[slot], self.dirs[slot]))
    }

    /// Consumer: advance the cursor past `n` entries.
    ///
    /// `Release` pairs with the producer's `Acquire` load of `cursor` in
    /// [`StepRing::space`], publishing that the slots up to the new cursor
    /// are free to overwrite.
    pub fn advance(&self, n: u32) {
        let cursor = self.cursor.load(Ordering::Relaxed);
        self.cursor
            .store(cursor.wrapping_add(n), Ordering::Release);
    }

    /// Reset both counters to 0. Used by `runtime_force_idle` (foreground
    /// synchronous teardown).
    ///
    /// **Caller-side quiescence requirement.** Both the producer *and* the
    /// consumer must be quiesced at the moment of call — neither side may
    /// have an in-flight `push` / `advance` / `peek_*` against this ring.
    /// `reset` zeroes `head` and `cursor` independently, and a concurrent
    /// producer or consumer would observe a torn (head, cursor) pair and
    /// could either skip pending entries or read stale slots. The slot
    /// buffers are left as-is — they will be overwritten by future pushes
    /// before they are read.
    pub fn reset(&mut self) {
        self.head.store(0, Ordering::Release);
        self.cursor.store(0, Ordering::Release);
    }
}

impl Default for StepRing {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_ring_has_full_space_and_no_entries() {
        let r = StepRing::new();
        assert_eq!(r.space(), STEP_RING_CAPACITY as u32);
        assert_eq!(r.available(), 0);
        assert!(r.peek_head().is_none());
        assert!(r.peek_next().is_none());
    }

    #[test]
    fn push_then_peek_returns_pushed_entry() {
        let mut r = StepRing::new();
        r.push(0xDEAD_BEEF, 1);
        assert_eq!(r.available(), 1);
        assert_eq!(r.peek_head(), Some((0xDEAD_BEEF, 1)));
        assert!(r.peek_next().is_none());
    }

    #[test]
    fn advance_consumes_entries() {
        let mut r = StepRing::new();
        r.push(100, 1);
        r.push(200, -1);
        r.push(300, 1);
        assert_eq!(r.available(), 3);
        r.advance(2);
        assert_eq!(r.available(), 1);
        assert_eq!(r.peek_head(), Some((300, 1)));
    }

    #[test]
    fn wrap_around_at_capacity_boundary() {
        let mut r = StepRing::new();
        // Fill capacity-100 entries, drain 100, push 200 more — head wraps
        // past the array boundary while cursor/head stays within the SPSC
        // invariant `head - cursor ≤ N`. After phase 1 head=924; phase 2
        // takes cursor 0→100; phase 3 pushes go to slots 924..1023 then
        // 0..99 (head ends at 1124 — exactly `head - cursor == N`, the
        // invariant boundary). Slot 100 is untouched in phase 3, so the
        // original (100, 1) entry remains the consumer's next read.
        //
        // (Note: the original plan wrote `CAPACITY - 1` here; that
        // overflows the SPSC invariant — fill 1023, drain 100, push 200
        // requires the ring to hold 1123 entries, exceeding capacity 1024,
        // and the phase-3 wrap overwrites slot 100. The off-by-one is
        // corrected to `- 100` so the test exercises wrap-around without
        // violating the data structure's documented invariant.)
        for i in 0..(STEP_RING_CAPACITY as u32 - 100) {
            r.push(i, if i % 2 == 0 { 1 } else { -1 });
        }
        r.advance(100);
        for i in 0..200 {
            r.push(i + 10000, 1);
        }
        assert_eq!(
            r.available(),
            STEP_RING_CAPACITY as u32 - 100 - 100 + 200,
        );
        // First entry after drain: original index 100, value 100, dir alternates.
        assert_eq!(r.peek_head(), Some((100, 1)));
    }

    #[test]
    fn space_correctly_tracks_head_cursor_delta() {
        let mut r = StepRing::new();
        for _ in 0..500 {
            r.push(0, 0);
        }
        assert_eq!(r.space(), STEP_RING_CAPACITY as u32 - 500);
        r.advance(300);
        assert_eq!(r.space(), STEP_RING_CAPACITY as u32 - 200);
    }

    #[test]
    fn reset_clears_head_and_cursor() {
        let mut r = StepRing::new();
        for _ in 0..500 {
            r.push(0, 0);
        }
        r.advance(250);
        r.reset();
        assert_eq!(r.available(), 0);
        assert_eq!(r.space(), STEP_RING_CAPACITY as u32);
    }
}
