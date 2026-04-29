//! Per-MCU credit counter for spec §5 (α flow control).
//!
//! The host maintains one [`CreditCounter`] per MCU. Each `try_acquire`
//! speculatively decrements the count; if the underlying `push_segment`
//! fails downstream the caller restores it via `release`. The MCU's
//! `kalico_credit_freed` async events update `available` to the
//! latest value reported by the firmware (see [`Self::on_credit_freed`]).
//!
//! `try_acquire`/`release`/`on_credit_freed` use atomic fetch-add CAS
//! loops so the counter is thread-safe even though Step-6 only drives
//! it from a single foreground thread — this keeps Step-7 producers
//! free to share the counter across threads without rework.

use std::sync::atomic::{AtomicI32, Ordering};

#[derive(Debug)]
pub struct CreditCounter {
    available: AtomicI32,
    capacity: i32,
    /// Tracks the MCU-side `credit_epoch`. The MCU bumps the epoch on
    /// every stream-open / FAULT-clear so the host can detect "the MCU
    /// wiped its accept-side state and we should reset our local
    /// counter".
    credit_epoch: AtomicI32,
}

impl CreditCounter {
    /// Construct with the per-MCU capacity (§5.1: derived from
    /// `Q_N_MAX` and the worst-case host latency budget).
    pub fn new(capacity: i32) -> Self {
        Self {
            available: AtomicI32::new(capacity),
            capacity,
            credit_epoch: AtomicI32::new(0),
        }
    }

    pub fn capacity(&self) -> i32 {
        self.capacity
    }

    /// Speculatively decrements `available` if `> 0`. Returns `Some(())`
    /// on success; the caller must call [`Self::release`] if the push
    /// fails downstream so credit isn't permanently leaked.
    pub fn try_acquire(&self) -> Option<()> {
        loop {
            let cur = self.available.load(Ordering::Acquire);
            if cur <= 0 {
                return None;
            }
            match self.available.compare_exchange(
                cur,
                cur - 1,
                Ordering::AcqRel,
                Ordering::Acquire,
            ) {
                Ok(_) => return Some(()),
                Err(_) => {}
            }
        }
    }

    /// Roll back a failed acquire. Bounded by `capacity` so concurrent
    /// `on_credit_freed` events can't push `available` past it.
    pub fn release(&self) {
        loop {
            let cur = self.available.load(Ordering::Acquire);
            let next = (cur + 1).min(self.capacity);
            if next == cur {
                return;
            }
            match self.available.compare_exchange(
                cur,
                next,
                Ordering::AcqRel,
                Ordering::Acquire,
            ) {
                Ok(_) => return,
                Err(_) => {}
            }
        }
    }

    /// Reconcile to the value reported in a `kalico_credit_freed`
    /// event. The MCU is authoritative — the host doesn't try to
    /// integrate the deltas, it just snaps `available` to the new
    /// `free_slots` (clamped to `capacity` in case events arrive out
    /// of order or stale).
    pub fn on_credit_freed(&self, free_slots: u8) {
        let want = i32::from(free_slots).min(self.capacity);
        self.available.store(want, Ordering::Release);
    }

    /// Reset on `credit_epoch` change. The host fully restores
    /// `available = capacity` because the MCU's queue is empty
    /// post-epoch-bump (stream-open, FAULT-clear, etc.).
    pub fn on_epoch_change(&self, new_epoch: i32) {
        self.credit_epoch.store(new_epoch, Ordering::Release);
        self.available.store(self.capacity, Ordering::Release);
    }

    pub fn available(&self) -> i32 {
        self.available.load(Ordering::Acquire)
    }

    pub fn credit_epoch(&self) -> i32 {
        self.credit_epoch.load(Ordering::Acquire)
    }
}
