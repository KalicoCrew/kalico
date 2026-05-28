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

/// Upper-bound default for tests and any caller that doesn't know the
/// MCU's actual pool size yet. The H7's `large` profile is 16; the F446's
/// `small` profile is 4. Production code MUST pass the per-MCU
/// `caps.curve_pool_n` value to `SlotPool::new` so the host's slot
/// allocator can't hand out indices the MCU will reject as out-of-range
/// (bench 2026-05-12: F446 reports `pool_n=4`, host previously hardcoded
/// 16 and the 5th sequential jog crashed with `KALICO_ERR_INVALID_HANDLE`).
pub const CURVE_POOL_N: usize = 16;

/// Per-MCU free-slot allocator.
#[derive(Debug)]
pub struct SlotPool {
    /// Capacity this pool was constructed with — caller-supplied so it
    /// matches the per-MCU `caps.curve_pool_n`. Used by diagnostics and
    /// the "slot pool exhausted" error path.
    capacity: usize,
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
    /// Construct an empty pool with `capacity` slots free (indices `0..capacity`).
    /// Pass the per-MCU `caps.curve_pool_n` value here so the host and MCU
    /// agree on the valid slot-index range.
    pub fn new(capacity: usize) -> Self {
        let mut free = VecDeque::with_capacity(capacity);
        for slot in 0..capacity {
            free.push_back(slot as u16);
        }
        Self {
            capacity,
            free,
            in_flight: HashSet::with_capacity(capacity),
            generation: HashMap::with_capacity(capacity),
            slot_to_segment: HashMap::with_capacity(capacity),
        }
    }

    /// Capacity this pool was constructed with.
    pub fn capacity(&self) -> usize {
        self.capacity
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
        Self::new(CURVE_POOL_N)
    }
}

// ---------------------------------------------------------------------------
// SharedSlotPool — thread-safe wrapper with condvar-based blocking acquire
// ---------------------------------------------------------------------------

use std::sync::atomic::{AtomicBool, Ordering as AtomicOrdering};
use std::sync::{Condvar, Mutex as StdMutex};
use std::time::{Duration, Instant};

/// Thread-safe wrapper around [`SlotPool`] that adds a condvar-based
/// [`Self::alloc_blocking`] method.
///
/// Callers that previously wrapped `SlotPool` in `Arc<Mutex<SlotPool>>`
/// should migrate to `Arc<SharedSlotPool>`. Every mutation path that could
/// return a slot to the free list (`release`, `retire_through_segment`)
/// signals the condvar so any parked `alloc_blocking` call wakes promptly.
///
/// ## Shutdown
///
/// [`Self::close`] sets a sticky closed flag and wakes all waiters. Once
/// closed, [`Self::alloc_blocking`] returns `None` immediately and
/// [`Self::try_alloc`] returns `None` regardless of free-slot count.
/// This prevents a 60-second stall when the MCU is released while the
/// dispatch closure is blocked waiting for slots.
///
/// ## Condvar pairing
///
/// `cv` is paired with `inner` — both `alloc_blocking` (waiter) and the
/// mutation methods (notifiers) lock `inner` before touching `cv`. The
/// `Condvar::notify_all` call in the notifier paths is issued while the
/// lock is still held, matching the std `Condvar` contract.
///
/// ## Poison recovery
///
/// All `Mutex::lock` calls use `.unwrap_or_else(|p| p.into_inner())` so a
/// panic in a background thread does not permanently poison the pool.
#[derive(Debug)]
pub struct SharedSlotPool {
    inner: StdMutex<SlotPool>,
    cv: Condvar,
    closed: AtomicBool,
}

impl SharedSlotPool {
    /// Construct a new shared pool with `capacity` slots.
    pub fn new(capacity: usize) -> Self {
        Self {
            inner: StdMutex::new(SlotPool::new(capacity)),
            cv: Condvar::new(),
            closed: AtomicBool::new(false),
        }
    }

    /// Mark the pool as closed. All current and future `alloc_blocking` /
    /// `try_alloc` calls return `None` immediately. Wakes all parked
    /// waiters so they can observe the closed state.
    pub fn close(&self) {
        self.closed.store(true, AtomicOrdering::Release);
        self.cv.notify_all();
    }

    pub fn is_closed(&self) -> bool {
        self.closed.load(AtomicOrdering::Acquire)
    }

    /// Non-blocking alloc. Returns `Some((slot_idx, generation))` when a
    /// free slot is available, `None` when the pool is exhausted or closed.
    pub fn try_alloc(&self) -> Option<(u16, u16)> {
        if self.is_closed() {
            return None;
        }
        self.inner
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .try_alloc()
    }

    /// Blocking alloc — waits up to `timeout` for a slot to become free.
    ///
    /// Returns `Some((slot_idx, generation))` when a slot was acquired,
    /// `None` when `timeout` elapsed without any slot becoming available.
    ///
    /// ## Pattern
    ///
    /// 1. Lock `inner`.
    /// 2. Try `guard.try_alloc()` — return immediately on success.
    /// 3. Check deadline; if past, return `None`.
    /// 4. `cv.wait_timeout(guard, remaining)` — releases the lock, parks
    ///    the thread until notified or the deadline arrives.
    /// 5. Re-acquire `guard`, loop back to step 2.
    /// 6. On condvar timeout: one final `try_alloc`, then `None`.
    pub fn alloc_blocking(&self, timeout: Duration) -> Option<(u16, u16)> {
        if self.is_closed() {
            return None;
        }
        let deadline = Instant::now() + timeout;
        let mut guard = self.inner.lock().unwrap_or_else(|p| p.into_inner());
        loop {
            if self.is_closed() {
                return None;
            }
            if let Some(pair) = guard.try_alloc() {
                return Some(pair);
            }
            let now = Instant::now();
            if now >= deadline {
                return None;
            }
            let remaining = deadline - now;
            let (g, wait_res) = self
                .cv
                .wait_timeout(guard, remaining)
                .unwrap_or_else(|p| p.into_inner());
            guard = g;
            if wait_res.timed_out() {
                return guard.try_alloc();
            }
        }
    }

    /// Record that `slot` belongs to `segment_id`. Delegates to
    /// [`SlotPool::register_segment`].
    pub fn register_segment(&self, slot: u16, segment_id: u32) {
        self.inner
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .register_segment(slot, segment_id);
    }

    /// Release `slot` back to the free pool. Signals the condvar if the
    /// slot was actually freed (i.e. it was in-flight before the call).
    pub fn release(&self, slot: u16) {
        let mut guard = self.inner.lock().unwrap_or_else(|p| p.into_inner());
        let before = guard.free_count();
        guard.release(slot);
        let freed = guard.free_count() > before;
        if freed {
            self.cv.notify_all();
        }
    }

    /// Release every in-flight slot whose registered `segment_id` is
    /// `<= retired_through`. Signals the condvar when one or more slots
    /// were actually freed. Returns the count released.
    pub fn retire_through_segment(&self, retired_through: u32) -> usize {
        let mut guard = self.inner.lock().unwrap_or_else(|p| p.into_inner());
        let n = guard.retire_through_segment(retired_through);
        if n > 0 {
            self.cv.notify_all();
        }
        n
    }

    /// Capacity this pool was constructed with.
    pub fn capacity(&self) -> usize {
        self.inner
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .capacity()
    }

    /// Number of slots currently in-flight.
    pub fn in_flight_count(&self) -> usize {
        self.inner
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .in_flight_count()
    }

    /// Number of slots currently free.
    pub fn free_count(&self) -> usize {
        self.inner
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .free_count()
    }
}

#[cfg(test)]
mod shared_tests;

#[cfg(test)]
mod tests;
