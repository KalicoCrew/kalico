//! Per-MCU curve-slot allocator with generation tracking.
//!
//! Backs the `kalico_load_curve` `slot: u16` field. The firmware-side curve
//! pool capacity is `runtime::curve_pool::CURVE_POOL_N = 64` (see
//! `rust/runtime/src/curve_pool.rs`). Slots `>= 64` are rejected by the
//! firmware bounds check.
//!
//! ## Lifecycle
//!
//! - `try_alloc()` pops a free slot, marks it in-flight, increments its
//!   generation, and returns `(slot_idx, generation)`. Returns `None` when
//!   the pool is exhausted — caller should backpressure.
//! - `retire_through_segment(seg_id)` releases every in-flight slot whose
//!   `register_segment` was called with `segment_id <= seg_id`. Idempotent —
//!   callers may pass duplicate / regressing ids without corruption.
//! - `register_segment(slot, segment_id)` records "this slot is referenced
//!   by this segment id," so the segment-id-driven retirement path can
//!   release the slot when it sees the firmware report a higher-or-equal
//!   `retired_through_segment_id` (in `kalico_credit_freed`).
//!
//! ## Generation packing
//!
//! Wire handles are `(generation << 16) | slot_idx` per the firmware
//! `CurveHandle::pack` convention (`rust/runtime/src/curve_pool.rs:90`).
//! Host-side generation must mirror what the MCU reports back in
//! `kalico_load_curve_response.curve_handle_packed`. The host increments
//! its tracked generation on each alloc; on a successful response the
//! caller can sanity-check that the firmware-reported generation matches.
//!
//! ## Concurrency
//!
//! `SlotPool` is `!Sync` by design. The bridge wraps it in
//! `Arc<Mutex<SlotPool>>` so the dispatch closure (planner thread) and
//! the eventual event-driven retirement callback (some MCU-event-routing
//! thread) can both touch it.
//!
//! ## Wire-routing dependency (Task 10 status)
//!
//! As of HEAD `799bdd867` the host bridge has NO inbound serial-event path —
//! `PassthroughRouter` only handles request/response notifies, and the
//! `EventDispatcher` that lifts `kalico_credit_freed` lives in the
//! `host_io::Reactor` which the bridge does not currently spin up. Until
//! that wiring lands, `retire_through_segment` is functionally unreachable
//! at runtime; the dispatch closure will starve once `CURVE_POOL_N`
//! segments are in flight without retirement.

use std::collections::{HashMap, HashSet, VecDeque};

/// Mirrors `runtime::curve_pool::CURVE_POOL_N` (cannot import directly
/// because `runtime` is `no_std` for the MCU build).
pub const CURVE_POOL_N: usize = 64;

/// Per-MCU free-slot allocator.
#[derive(Debug)]
pub struct SlotPool {
    /// Free slot indices, FIFO so reuse cycles all slots before repeating.
    free: VecDeque<u16>,
    /// Slots currently allocated and not yet retired.
    in_flight: HashSet<u16>,
    /// Current generation per slot — incremented on every successful
    /// `try_alloc`. Initial value 1 (firmware emits gen=1 on first load).
    generation: HashMap<u16, u16>,
    /// `slot -> segment_id` that currently owns it. Populated by
    /// `register_segment`; consulted by `retire_through_segment`.
    slot_to_segment: HashMap<u16, u32>,
}

impl SlotPool {
    /// Construct an empty pool with `CURVE_POOL_N` slots free.
    pub fn new() -> Self {
        let mut free = VecDeque::with_capacity(CURVE_POOL_N);
        for slot in 0..CURVE_POOL_N {
            free.push_back(slot as u16);
        }
        Self {
            free,
            in_flight: HashSet::with_capacity(CURVE_POOL_N),
            generation: HashMap::with_capacity(CURVE_POOL_N),
            slot_to_segment: HashMap::with_capacity(CURVE_POOL_N),
        }
    }

    /// Number of slots currently free.
    pub fn free_count(&self) -> usize {
        self.free.len()
    }

    /// Number of slots currently in-flight.
    pub fn in_flight_count(&self) -> usize {
        self.in_flight.len()
    }

    /// Reserve a slot. Returns `(slot_idx, generation)` on success; the
    /// caller must ship `slot_idx` in the `kalico_load_curve` request and
    /// can use `generation` to validate the response handle.
    pub fn try_alloc(&mut self) -> Option<(u16, u16)> {
        let slot = self.free.pop_front()?;
        debug_assert!(
            !self.in_flight.contains(&slot),
            "free queue contained an in-flight slot {slot}"
        );
        self.in_flight.insert(slot);
        let gen_entry = self.generation.entry(slot).or_insert(0);
        // Wraparound is fine — the firmware uses u16 generations too.
        *gen_entry = gen_entry.wrapping_add(1);
        Some((slot, *gen_entry))
    }

    /// Record that `slot` belongs to `segment_id`. Call after a successful
    /// `try_alloc` and before the segment is pushed.
    pub fn register_segment(&mut self, slot: u16, segment_id: u32) {
        if !self.in_flight.contains(&slot) {
            // Defensive — caller ordering bug. Don't panic; just no-op.
            log::warn!(
                "slot_pool: register_segment({slot}, {segment_id}) called for non-in-flight slot"
            );
            return;
        }
        self.slot_to_segment.insert(slot, segment_id);
    }

    /// Release `slot` back to the free pool unconditionally. Idempotent —
    /// duplicate releases are a no-op. Use this only when you have direct
    /// per-slot retirement signal (e.g. a future `kalico_curve_freed`
    /// event); otherwise prefer `retire_through_segment`.
    pub fn release(&mut self, slot: u16) {
        if !self.in_flight.remove(&slot) {
            return; // already free, idempotent
        }
        self.slot_to_segment.remove(&slot);
        self.free.push_back(slot);
    }

    /// Release every in-flight slot whose registered `segment_id` is
    /// `<= retired_through`. Driven by `kalico_credit_freed`'s
    /// `retired_through_segment_id` field. Slots that were allocated but
    /// never `register_segment`-ed (race window: alloc-before-push) are
    /// untouched. Returns the count released.
    pub fn retire_through_segment(&mut self, retired_through: u32) -> usize {
        let to_release: Vec<u16> = self
            .slot_to_segment
            .iter()
            .filter(|(_, seg)| **seg <= retired_through)
            .map(|(slot, _)| *slot)
            .collect();
        let n = to_release.len();
        for slot in to_release {
            self.release(slot);
        }
        n
    }
}

impl Default for SlotPool {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fresh_pool_has_full_capacity() {
        let p = SlotPool::new();
        assert_eq!(p.free_count(), CURVE_POOL_N);
        assert_eq!(p.in_flight_count(), 0);
    }

    #[test]
    fn alloc_advances_generation_per_slot() {
        let mut p = SlotPool::new();
        let (s0, g0) = p.try_alloc().unwrap();
        assert_eq!(g0, 1, "first alloc is gen=1");
        // Free and re-alloc — same slot should bump to gen=2.
        p.release(s0);
        let mut found = None;
        // The free queue is FIFO so after releasing s0 the next alloc may
        // not be s0. Drain until we get it back, advancing generations on
        // intervening slots.
        for _ in 0..CURVE_POOL_N + 1 {
            let (s, g) = p.try_alloc().unwrap();
            if s == s0 {
                found = Some(g);
                break;
            }
        }
        assert_eq!(
            found,
            Some(2),
            "second alloc of same slot must be gen=2"
        );
    }

    #[test]
    fn pool_exhausts_at_capacity() {
        let mut p = SlotPool::new();
        for _ in 0..CURVE_POOL_N {
            assert!(p.try_alloc().is_some());
        }
        assert!(
            p.try_alloc().is_none(),
            "exhausted pool must return None"
        );
        assert_eq!(p.in_flight_count(), CURVE_POOL_N);
    }

    #[test]
    fn release_is_idempotent() {
        let mut p = SlotPool::new();
        let (s, _) = p.try_alloc().unwrap();
        p.release(s);
        p.release(s); // duplicate
        assert_eq!(p.in_flight_count(), 0);
        assert_eq!(p.free_count(), CURVE_POOL_N);
    }

    #[test]
    fn retire_through_segment_releases_eligible_slots() {
        let mut p = SlotPool::new();
        let (s1, _) = p.try_alloc().unwrap();
        p.register_segment(s1, 1);
        let (s2, _) = p.try_alloc().unwrap();
        p.register_segment(s2, 2);
        let (s3, _) = p.try_alloc().unwrap();
        p.register_segment(s3, 3);

        assert_eq!(p.in_flight_count(), 3);

        // MCU reports "everything up to seg 2 retired."
        let n = p.retire_through_segment(2);
        assert_eq!(n, 2, "should release 2 slots");
        assert_eq!(p.in_flight_count(), 1);
        // Then a higher retirement releases the rest.
        let n = p.retire_through_segment(10);
        assert_eq!(n, 1);
        assert_eq!(p.in_flight_count(), 0);
    }

    #[test]
    fn retire_through_lower_id_is_noop() {
        let mut p = SlotPool::new();
        let (s, _) = p.try_alloc().unwrap();
        p.register_segment(s, 100);
        // Stale event from earlier in the print.
        assert_eq!(p.retire_through_segment(50), 0);
        assert_eq!(p.in_flight_count(), 1);
    }

    #[test]
    fn alloc_without_register_segment_skips_retirement() {
        // An alloc that hasn't yet been pushed (race window) must not be
        // released by a segment-id retirement event — the segment_id is
        // unknown, so the slot stays in-flight until either explicitly
        // released or its eventual segment-id retires.
        let mut p = SlotPool::new();
        let _ = p.try_alloc().unwrap();
        // No register_segment call yet.
        assert_eq!(p.retire_through_segment(u32::MAX), 0);
        assert_eq!(p.in_flight_count(), 1);
    }

    #[test]
    fn many_alloc_release_cycles_dont_leak() {
        // Cycle through more than CURVE_POOL_N allocations to verify the
        // pool stays balanced — this is the regression for the original
        // u16 rolling-counter bug (would have errored at slot 64).
        let mut p = SlotPool::new();
        for i in 0..(CURVE_POOL_N * 5) {
            let (s, _) = p.try_alloc().unwrap_or_else(|| {
                panic!("alloc {i} failed — pool starved")
            });
            p.register_segment(s, i as u32);
            // Retire immediately (simulates a flushed pipeline).
            p.retire_through_segment(i as u32);
        }
        assert_eq!(p.free_count(), CURVE_POOL_N);
    }
}
