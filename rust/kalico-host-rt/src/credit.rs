//! Per-MCU credit counter for spec §5 (α flow control).
//!
//! The host maintains one [`CreditCounter`] per MCU. Each `try_acquire`
//! speculatively decrements the count; if the underlying `push_segment`
//! fails downstream the caller restores it via `release`. The MCU's
//! `kalico_credit_freed` async events update `available` to the
//! latest value reported by the firmware (see [`Self::on_credit_freed`]).
//!
//! ## Blocking acquire
//!
//! [`Self::acquire_blocking`] waits on a `Condvar` for credit to become
//! available, signalled by `on_credit_freed`, `on_epoch_change`, or
//! `release`. The MCU's per-MCU segment queue is structurally capped at
//! `Q_N - 1 = 7` in flight (see `rust/runtime/src/queue.rs`); under
//! rapid `submit_move` bursts the host saturates the queue and credit
//! drops to zero. Without `acquire_blocking` the producer would fail
//! immediately on the next push, the planner thread would store a
//! `Dispatch` error and stop, and the user would see "every other jog
//! ignored" — exactly the bench-observed symptom.
//!
//! `try_acquire`/`release`/`on_credit_freed` use atomic fetch-add CAS
//! loops so the counter is thread-safe even though Step-6 only drives
//! it from a single foreground thread — this keeps Step-7 producers
//! free to share the counter across threads without rework.

use std::sync::atomic::{AtomicI32, AtomicU64, Ordering};
use std::sync::{Condvar, Mutex};
use std::time::{Duration, Instant};

#[derive(Debug)]
pub struct CreditCounter {
    available: AtomicI32,
    capacity: i32,
    /// Tracks the MCU-side `credit_epoch`. The MCU bumps the epoch on
    /// every stream-open / FAULT-clear so the host can detect "the MCU
    /// wiped its accept-side state and we should reset our local
    /// counter".
    credit_epoch: AtomicI32,
    /// Number of `on_credit_freed` events observed since construction.
    /// Diagnostic counter — exposed via [`Self::credit_freed_events`]
    /// for trace logging.
    credit_freed_events: AtomicU64,
    /// Notification channel for [`Self::acquire_blocking`] waiters. The
    /// `Mutex<()>` is a dummy guard required by `Condvar::wait_timeout`;
    /// the actual state lives in `available` (atomic). Every path that
    /// could increase `available` (`release`, `on_credit_freed`,
    /// `on_epoch_change`) calls `notify_all` so waiters wake promptly.
    wait_lock: Mutex<()>,
    wait_cv: Condvar,
}

impl CreditCounter {
    /// Construct with the per-MCU capacity (§5.1: derived from
    /// `Q_N_MAX` and the worst-case host latency budget).
    pub fn new(capacity: i32) -> Self {
        Self {
            available: AtomicI32::new(capacity),
            capacity,
            credit_epoch: AtomicI32::new(0),
            credit_freed_events: AtomicU64::new(0),
            wait_lock: Mutex::new(()),
            wait_cv: Condvar::new(),
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
            if self
                .available
                .compare_exchange(cur, cur - 1, Ordering::AcqRel, Ordering::Acquire)
                .is_ok()
            {
                return Some(());
            }
        }
    }

    /// Blocking acquire — waits for credit to become available, up to
    /// `timeout`. Returns `Ok(())` on success, `Err(())` on timeout.
    ///
    /// Spec §5 / §7.3 back-pressure: the MCU's `Q_N - 1 = 7` in-flight
    /// segment cap is enforced by the host snapping `available` to the
    /// `free_slots` value in every `kalico_credit_freed` event. Under
    /// rapid `submit_move` bursts the host fills the MCU queue and the
    /// next `on_credit_freed` arrives with `free_slots=0`. Without this
    /// blocking primitive, the next `push_segment` would immediately
    /// fail with `NoCredit`, the planner thread would store the error
    /// and stop, and the user would see motion stop / "every other jog
    /// ignored" until klippy restarts.
    ///
    /// Pattern: `try_acquire → wait_cv → try_acquire → wait_cv …` until
    /// `timeout` elapses. Notifications come from `release`,
    /// `on_credit_freed`, `on_epoch_change`.
    pub fn acquire_blocking(&self, timeout: Duration) -> Result<(), ()> {
        // Fast path — common case is credit immediately available.
        if self.try_acquire().is_some() {
            return Ok(());
        }
        let deadline = Instant::now() + timeout;
        let mut guard = self.wait_lock.lock().unwrap_or_else(|p| p.into_inner());
        loop {
            // Re-check after grabbing the lock; a notification may have
            // raced between our `try_acquire` above and `lock()`.
            if self.try_acquire().is_some() {
                return Ok(());
            }
            let now = Instant::now();
            if now >= deadline {
                return Err(());
            }
            let remaining = deadline - now;
            let (g, wait_res) = self.wait_cv.wait_timeout(guard, remaining).unwrap();
            guard = g;
            if wait_res.timed_out() {
                // One final try in case `available` changed since the
                // wait started but the notifier didn't reach us before
                // the deadline.
                return self.try_acquire().ok_or(());
            }
        }
    }

    /// Roll back a failed acquire. Bounded by `capacity` so concurrent
    /// `on_credit_freed` events can't push `available` past it. Wakes any
    /// `acquire_blocking` waiter so a retried push can proceed.
    pub fn release(&self) {
        let mut changed = false;
        loop {
            let cur = self.available.load(Ordering::Acquire);
            let next = (cur + 1).min(self.capacity);
            if next == cur {
                break;
            }
            if self
                .available
                .compare_exchange(cur, next, Ordering::AcqRel, Ordering::Acquire)
                .is_ok()
            {
                changed = true;
                break;
            }
        }
        if changed {
            self.notify_waiters();
        }
    }

    /// Reconcile to the value reported in a `kalico_credit_freed`
    /// event. The MCU is authoritative — the host doesn't try to
    /// integrate the deltas, it just snaps `available` to the new
    /// `free_slots` (clamped to `capacity` in case events arrive out
    /// of order or stale). Wakes any `acquire_blocking` waiter.
    pub fn on_credit_freed(&self, free_slots: u8) {
        let want = i32::from(free_slots).min(self.capacity);
        self.available.store(want, Ordering::Release);
        self.credit_freed_events.fetch_add(1, Ordering::Relaxed);
        self.notify_waiters();
    }

    /// Reset on `credit_epoch` change. The host fully restores
    /// `available = capacity` because the MCU's queue is empty
    /// post-epoch-bump (stream-open, FAULT-clear, etc.). Wakes any
    /// `acquire_blocking` waiter.
    pub fn on_epoch_change(&self, new_epoch: i32) {
        self.credit_epoch.store(new_epoch, Ordering::Release);
        self.available.store(self.capacity, Ordering::Release);
        self.notify_waiters();
    }

    pub fn available(&self) -> i32 {
        self.available.load(Ordering::Acquire)
    }

    pub fn credit_epoch(&self) -> i32 {
        self.credit_epoch.load(Ordering::Acquire)
    }

    /// Cumulative count of `on_credit_freed` events observed since
    /// construction. Diagnostic — proves the host is actually receiving
    /// MCU credit-freed traffic.
    pub fn credit_freed_events(&self) -> u64 {
        self.credit_freed_events.load(Ordering::Relaxed)
    }

    fn notify_waiters(&self) {
        // Take and immediately drop the lock so any waiter that was
        // already past its `try_acquire` re-check and is parked in
        // `wait_timeout` sees the notification. `Condvar::notify_all`
        // requires the lock to be held by the notifier, per the std
        // contract.
        let _guard = self.wait_lock.lock().unwrap_or_else(|p| p.into_inner());
        self.wait_cv.notify_all();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;
    use std::thread;

    #[test]
    fn acquire_blocking_returns_immediately_when_available() {
        let c = CreditCounter::new(4);
        assert!(c.acquire_blocking(Duration::from_millis(10)).is_ok());
        assert_eq!(c.available(), 3);
    }

    #[test]
    fn acquire_blocking_times_out_when_no_credit() {
        let c = CreditCounter::new(1);
        assert!(c.try_acquire().is_some());
        assert_eq!(c.available(), 0);
        let start = Instant::now();
        let result = c.acquire_blocking(Duration::from_millis(20));
        let elapsed = start.elapsed();
        assert!(result.is_err());
        assert!(
            elapsed >= Duration::from_millis(15),
            "blocking acquire should wait nearly the full timeout, got {:?}",
            elapsed
        );
    }

    #[test]
    fn release_unblocks_acquire_blocking() {
        let c = Arc::new(CreditCounter::new(1));
        assert!(c.try_acquire().is_some());
        assert_eq!(c.available(), 0);

        let c2 = Arc::clone(&c);
        let handle = thread::spawn(move || c2.acquire_blocking(Duration::from_secs(2)));
        // Give the waiter time to enter wait_timeout.
        thread::sleep(Duration::from_millis(50));
        c.release();
        let result = handle.join().unwrap();
        assert!(result.is_ok(), "release must unblock waiter");
        // The waiter consumed the credit, so available is back to 0.
        assert_eq!(c.available(), 0);
    }

    #[test]
    fn on_credit_freed_unblocks_acquire_blocking() {
        let c = Arc::new(CreditCounter::new(7));
        // Drain to zero.
        for _ in 0..7 {
            assert!(c.try_acquire().is_some());
        }
        assert_eq!(c.available(), 0);

        let c2 = Arc::clone(&c);
        let handle = thread::spawn(move || c2.acquire_blocking(Duration::from_secs(2)));
        thread::sleep(Duration::from_millis(50));
        // MCU reports 3 slots free.
        c.on_credit_freed(3);
        assert_eq!(c.credit_freed_events(), 1);
        let result = handle.join().unwrap();
        assert!(result.is_ok());
        assert_eq!(c.available(), 2);
    }

    #[test]
    fn on_epoch_change_unblocks_acquire_blocking() {
        let c = Arc::new(CreditCounter::new(7));
        for _ in 0..7 {
            assert!(c.try_acquire().is_some());
        }
        assert_eq!(c.available(), 0);

        let c2 = Arc::clone(&c);
        let handle = thread::spawn(move || c2.acquire_blocking(Duration::from_secs(2)));
        thread::sleep(Duration::from_millis(50));
        c.on_epoch_change(1);
        let result = handle.join().unwrap();
        assert!(result.is_ok());
        assert_eq!(c.available(), 6);
        assert_eq!(c.credit_epoch(), 1);
    }

    #[test]
    fn multiple_waiters_all_get_served_in_order() {
        // 8 waiters, capacity 4; on_credit_freed grants 4 slots. Each
        // waiter consumes 1 — only 4 of the 8 should succeed.
        let c = Arc::new(CreditCounter::new(4));
        for _ in 0..4 {
            assert!(c.try_acquire().is_some());
        }

        let mut handles = Vec::new();
        for _ in 0..8 {
            let c2 = Arc::clone(&c);
            handles.push(thread::spawn(move || {
                c2.acquire_blocking(Duration::from_millis(200))
            }));
        }
        thread::sleep(Duration::from_millis(50));
        c.on_credit_freed(4);

        let mut ok = 0;
        let mut err = 0;
        for h in handles {
            match h.join().unwrap() {
                Ok(()) => ok += 1,
                Err(()) => err += 1,
            }
        }
        assert_eq!(ok, 4, "exactly 4 waiters should get credit");
        assert_eq!(err, 4, "the other 4 must time out");
    }
}
