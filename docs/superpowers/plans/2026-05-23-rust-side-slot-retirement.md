# Rust-Side Slot Retirement Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Route `SlotPool::retire_through_segment` directly from the Rust reactor thread so slot retirement no longer depends on the Python reactor polling `_bridge_event_poller`.

**Architecture:** The `EventDispatcher` on each per-MCU reactor thread already calls `CreditCounter::on_credit_freed` when a `CreditFreed` event arrives — fully Rust-internal, no Python. We add the same pattern for slot retirement: a `retirement_callback: Option<Arc<dyn Fn(u32) + Send + Sync>>` on `EventDispatcher`, called in the `CreditFreed` arm with `retired_through_segment_id`. The bridge installs this callback via a new `AttachRetirementCallback` reactor command. This avoids a circular crate dependency (`motion-bridge` → `kalico-host-rt` already exists; the reverse would be circular). On the dispatch side, `SlotPool` gets wrapped in `SharedSlotPool` with a `Condvar`-based `alloc_blocking`, replacing the immediate-fail `try_alloc` + `SlotPoolExhausted` error. The Python `_on_credit_freed` handler is removed.

**Tech Stack:** Rust (kalico-host-rt, motion-bridge), Python (klippy)

---

### Task 1: Add `SharedSlotPool` with condvar-based blocking acquire

**Files:**
- Modify: `rust/motion-bridge/src/slot_pool.rs`

- [ ] **Step 1: Write the failing tests**

Add at the bottom of `slot_pool.rs`:

```rust
use std::sync::{Condvar, Mutex};
use std::time::Duration;

#[derive(Debug)]
pub struct SharedSlotPool {
    inner: Mutex<SlotPool>,
    cv: Condvar,
}

// tests first — implementation in step 3

#[cfg(test)]
mod shared_tests {
    use super::*;
    use std::sync::Arc;
    use std::thread;
    use std::time::Duration;

    #[test]
    fn alloc_blocking_immediate_when_free() {
        let pool = Arc::new(SharedSlotPool::new(4));
        let result = pool.alloc_blocking(Duration::from_millis(10));
        assert!(result.is_some());
    }

    #[test]
    fn alloc_blocking_wakes_on_retire() {
        let pool = Arc::new(SharedSlotPool::new(2));
        let (s0, _) = pool.try_alloc().unwrap();
        pool.register_segment(s0, 1);
        let (s1, _) = pool.try_alloc().unwrap();
        pool.register_segment(s1, 2);

        let pool2 = Arc::clone(&pool);
        let handle = thread::spawn(move || pool2.alloc_blocking(Duration::from_secs(2)));
        thread::sleep(Duration::from_millis(50));
        pool.retire_through_segment(1);
        let result = handle.join().unwrap();
        assert!(result.is_some(), "alloc_blocking must succeed after retirement");
    }

    #[test]
    fn alloc_blocking_times_out_when_no_retirement() {
        let pool = Arc::new(SharedSlotPool::new(1));
        let (s, _) = pool.try_alloc().unwrap();
        pool.register_segment(s, 1);
        let start = std::time::Instant::now();
        let result = pool.alloc_blocking(Duration::from_millis(50));
        assert!(result.is_none());
        assert!(start.elapsed() >= Duration::from_millis(40));
    }

    #[test]
    fn multiple_alloc_blocking_all_served() {
        let pool = Arc::new(SharedSlotPool::new(1));
        let (s, _) = pool.try_alloc().unwrap();
        pool.register_segment(s, 1);

        let mut handles = Vec::new();
        for _ in 0..3 {
            let p = Arc::clone(&pool);
            handles.push(thread::spawn(move || p.alloc_blocking(Duration::from_secs(2))));
        }
        thread::sleep(Duration::from_millis(50));
        // Retire and alloc 3 times (capacity=1, so each retire frees exactly 1 slot).
        for seg in 1..=3 {
            pool.retire_through_segment(seg);
            // After each retire one waiter wakes, allocs, and we register+retire next round.
            thread::sleep(Duration::from_millis(30));
            // The woken thread consumed the slot; register it so next retire works.
            if seg < 3 {
                if let Some((s, _)) = pool.try_alloc() {
                    pool.register_segment(s, seg + 1);
                }
            }
        }
        // Actually this test is getting complicated with the interleaving.
        // Simplify: just retire all at once and check all 3 eventually succeed
        // by giving enough slots.
    }
}
```

Actually let me simplify the tests. The multi-waiter test is tricky because capacity=1 means only 1 waiter wakes per retire. Let me keep it simple.

- [ ] **Step 1: Write the failing tests for `SharedSlotPool`**

Add to the bottom of `slot_pool.rs`, before the existing `#[cfg(test)] mod tests`:

```rust
use std::sync::{Condvar, Mutex as StdMutex};

#[cfg(test)]
mod shared_tests {
    use super::*;
    use std::sync::Arc;
    use std::thread;
    use std::time::Duration;

    #[test]
    fn alloc_blocking_immediate_when_free() {
        let pool = Arc::new(SharedSlotPool::new(4));
        assert!(pool.alloc_blocking(Duration::from_millis(10)).is_some());
        assert_eq!(pool.in_flight_count(), 1);
    }

    #[test]
    fn alloc_blocking_wakes_on_retire() {
        let pool = Arc::new(SharedSlotPool::new(2));
        let (s0, _) = pool.try_alloc().unwrap();
        pool.register_segment(s0, 1);
        let (s1, _) = pool.try_alloc().unwrap();
        pool.register_segment(s1, 2);
        assert!(pool.try_alloc().is_none());

        let pool2 = Arc::clone(&pool);
        let handle = thread::spawn(move || pool2.alloc_blocking(Duration::from_secs(2)));
        thread::sleep(Duration::from_millis(50));
        pool.retire_through_segment(1);
        assert!(handle.join().unwrap().is_some());
    }

    #[test]
    fn alloc_blocking_times_out() {
        let pool = Arc::new(SharedSlotPool::new(1));
        pool.try_alloc().unwrap();
        let start = std::time::Instant::now();
        assert!(pool.alloc_blocking(Duration::from_millis(50)).is_none());
        assert!(start.elapsed() >= Duration::from_millis(40));
    }

    #[test]
    fn release_wakes_alloc_blocking() {
        let pool = Arc::new(SharedSlotPool::new(1));
        let (s, _) = pool.try_alloc().unwrap();

        let pool2 = Arc::clone(&pool);
        let handle = thread::spawn(move || pool2.alloc_blocking(Duration::from_secs(2)));
        thread::sleep(Duration::from_millis(50));
        pool.release(s);
        assert!(handle.join().unwrap().is_some());
    }
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd rust && cargo test -p motion-bridge shared_tests -- --nocapture 2>&1 | tail -20`
Expected: FAIL — `SharedSlotPool` not defined.

- [ ] **Step 3: Implement `SharedSlotPool`**

Add to `slot_pool.rs` after the `Default` impl and before `#[cfg(test)] mod tests`:

```rust
use std::sync::{Condvar, Mutex as StdMutex};

#[derive(Debug)]
pub struct SharedSlotPool {
    inner: StdMutex<SlotPool>,
    cv: Condvar,
}

impl SharedSlotPool {
    pub fn new(capacity: usize) -> Self {
        Self {
            inner: StdMutex::new(SlotPool::new(capacity)),
            cv: Condvar::new(),
        }
    }

    pub fn try_alloc(&self) -> Option<(u16, u16)> {
        self.inner
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .try_alloc()
    }

    pub fn alloc_blocking(&self, timeout: std::time::Duration) -> Option<(u16, u16)> {
        let deadline = std::time::Instant::now() + timeout;
        let mut guard = self.inner.lock().unwrap_or_else(|p| p.into_inner());
        loop {
            if let Some(r) = guard.try_alloc() {
                return Some(r);
            }
            let now = std::time::Instant::now();
            if now >= deadline {
                return None;
            }
            let (g, wait_res) = self.cv.wait_timeout(guard, deadline - now).unwrap();
            guard = g;
            if wait_res.timed_out() {
                return guard.try_alloc();
            }
        }
    }

    pub fn register_segment(&self, slot: u16, segment_id: u32) {
        self.inner
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .register_segment(slot, segment_id);
    }

    pub fn release(&self, slot: u16) {
        let released = {
            let mut pool = self.inner.lock().unwrap_or_else(|p| p.into_inner());
            let before = pool.in_flight_count();
            pool.release(slot);
            pool.in_flight_count() < before
        };
        if released {
            self.cv.notify_all();
        }
    }

    pub fn retire_through_segment(&self, retired_through: u32) -> usize {
        let n = self
            .inner
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .retire_through_segment(retired_through);
        if n > 0 {
            self.cv.notify_all();
        }
        n
    }

    pub fn capacity(&self) -> usize {
        self.inner
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .capacity()
    }

    pub fn in_flight_count(&self) -> usize {
        self.inner
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .in_flight_count()
    }

    pub fn free_count(&self) -> usize {
        self.inner
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .free_count()
    }
}
```

- [ ] **Step 4: Run tests**

Run: `cd rust && cargo test -p motion-bridge slot_pool -- --nocapture 2>&1 | tail -20`
Expected: All slot_pool tests PASS (both original and shared_tests).

- [ ] **Step 5: Commit**

```bash
git add rust/motion-bridge/src/slot_pool.rs
git commit -m "feat: add SharedSlotPool with condvar-based blocking acquire"
```

---

### Task 2: Add retirement callback to `EventDispatcher`

**Files:**
- Modify: `rust/kalico-host-rt/src/host_io/events.rs`
- Modify: `rust/kalico-host-rt/src/host_io/mod.rs`
- Modify: `rust/kalico-host-rt/src/host_io/reactor.rs`

The `EventDispatcher` needs a callback that fires on every `CreditFreed` event with the `retired_through_segment_id`. This callback is installed by the bridge at init time and calls `SharedSlotPool::retire_through_segment` + `HomingState::complete_if_retired`. Using a callback (`Arc<dyn Fn(u32) + Send + Sync>`) avoids a circular dependency — `kalico-host-rt` doesn't need to know about `motion-bridge` types.

- [ ] **Step 1: Write the failing test**

In `rust/kalico-host-rt/src/host_io/events.rs`, in the existing `dispatch_tests` module:

```rust
#[test]
fn credit_freed_calls_retirement_callback() {
    use std::sync::atomic::{AtomicU32, Ordering};

    let status = Arc::new(ArcSwap::from_pointee(StatusEvent::default()));
    let mut d = EventDispatcher::new(status, 16, 8);

    let retired_id = Arc::new(AtomicU32::new(0));
    let retired_id2 = Arc::clone(&retired_id);
    d.retirement_callback = Some(Arc::new(move |seg_id| {
        retired_id2.store(seg_id, Ordering::Release);
    }));

    d.dispatch(RuntimeEvent::CreditFreed(CreditFreedEvent {
        retired_through_segment_id: 42,
        free_slots: 5,
    }));
    assert_eq!(retired_id.load(Ordering::Acquire), 42);
}

#[test]
fn status_synthesized_credit_freed_calls_retirement_callback() {
    use std::sync::atomic::{AtomicU32, Ordering};

    let status = Arc::new(ArcSwap::from_pointee(StatusEvent::default()));
    let mut d = EventDispatcher::new(status, 16, 8);

    let retired_id = Arc::new(AtomicU32::new(0));
    let retired_id2 = Arc::clone(&retired_id);
    d.retirement_callback = Some(Arc::new(move |seg_id| {
        retired_id2.store(seg_id, Ordering::Release);
    }));

    // Dispatch a Status with advanced watermark — should synthesize CreditFreed
    // which should call the callback.
    d.dispatch(RuntimeEvent::Status(StatusEvent {
        retired_through_segment_id: 10,
        queue_depth: 3,
        ..StatusEvent::default()
    }));
    assert_eq!(retired_id.load(Ordering::Acquire), 10);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd rust && cargo test -p kalico-host-rt retirement_callback -- --nocapture 2>&1 | tail -20`
Expected: FAIL — no `retirement_callback` field.

- [ ] **Step 3: Add `retirement_callback` field to `EventDispatcher`**

In `rust/kalico-host-rt/src/host_io/events.rs`:

Add field to struct:
```rust
pub struct EventDispatcher {
    pub credit_counter: Option<Arc<CreditCounter>>,
    pub retirement_callback: Option<Arc<dyn Fn(u32) + Send + Sync>>,
    pub fault_latch: FaultLatch,
    // ... rest unchanged
}
```

Initialize in `new`:
```rust
retirement_callback: None,
```

In `dispatch`, in the `CreditFreed` arm, after the `credit_counter.on_credit_freed` call and its diagnostic `eprintln!`:
```rust
if let Some(cb) = &self.retirement_callback {
    cb(e.retired_through_segment_id);
}
```

- [ ] **Step 4: Add `AttachRetirementCallback` reactor command**

In `rust/kalico-host-rt/src/host_io/mod.rs`, add to `ReactorCommand`:
```rust
AttachRetirementCallback(Arc<dyn Fn(u32) + Send + Sync>),
```

In `rust/kalico-host-rt/src/host_io/reactor.rs`, handle alongside `AttachCreditCounter`:
```rust
ReactorCommand::AttachRetirementCallback(cb) => {
    self.event_dispatcher.retirement_callback = Some(cb);
}
```

Add the public method to `KalicoHostIo`:
```rust
pub fn attach_retirement_callback(&self, cb: Arc<dyn Fn(u32) + Send + Sync>) {
    let _ = self
        .submission_tx
        .send(ReactorCommand::AttachRetirementCallback(cb));
}
```

- [ ] **Step 5: Run tests**

Run: `cd rust && cargo test -p kalico-host-rt retirement_callback -- --nocapture 2>&1 | tail -20`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add rust/kalico-host-rt/src/host_io/events.rs rust/kalico-host-rt/src/host_io/mod.rs rust/kalico-host-rt/src/host_io/reactor.rs
git commit -m "feat: add retirement_callback to EventDispatcher for Rust-side slot retirement"
```

---

### Task 3: Migrate bridge to `SharedSlotPool` and install retirement callback

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs`

Replace `Arc<Mutex<SlotPool>>` with `Arc<SharedSlotPool>` everywhere. In `init_planner`, install a retirement callback on each MCU's `KalicoHostIo` that calls `SharedSlotPool::retire_through_segment` + `HomingState::complete_if_retired`. In the dispatch closure, replace `try_alloc` + `SlotPoolExhausted` with `alloc_blocking`.

- [ ] **Step 1: Update imports and struct field type**

```rust
// Change import:
use crate::slot_pool::{CURVE_POOL_N, SharedSlotPool, SlotPool};

// Change field:
// Before:
slot_pools: Arc<Mutex<HashMap<u32, Arc<Mutex<SlotPool>>>>>,
// After:
slot_pools: Arc<Mutex<HashMap<u32, Arc<SharedSlotPool>>>>,
```

Update the `new()` initializer accordingly.

- [ ] **Step 2: Update `init_planner` — pool construction and callback installation**

Change `dispatch_ios` type:
```rust
let mut dispatch_ios: HashMap<
    u32,
    (Weak<KalicoHostIo>, Arc<CreditCounter>, Arc<SharedSlotPool>),
> = HashMap::new();
```

Change pool construction:
```rust
// Before:
let slot_pool = Arc::new(Mutex::new(SlotPool::new(pool_capacity)));
// After:
let slot_pool = Arc::new(SharedSlotPool::new(pool_capacity));
```

After `io.attach_credit_counter(Arc::clone(&credit))`, install the retirement callback:
```rust
{
    let pool_for_cb = Arc::clone(&slot_pool);
    let homing_for_cb = Arc::clone(&self.homing);
    io.attach_retirement_callback(Arc::new(move |retired_through| {
        pool_for_cb.retire_through_segment(retired_through);
        homing_for_cb.complete_if_retired(retired_through);
    }));
}
```

- [ ] **Step 3: Update dispatch closure — `alloc_blocking` replaces `try_alloc`**

Add timeout constant near existing ones:
```rust
const DEFAULT_SLOT_ACQUIRE_TIMEOUT: Duration = Duration::from_secs(60);
```

Replace the slot allocation block in the dispatch closure:
```rust
// Before:
let alloc_result = {
    let mut pool =
        slot_pool.lock().unwrap_or_else(|p| p.into_inner());
    let cap = pool.capacity();
    let in_flight = pool.in_flight_count();
    pool.try_alloc()
        .ok_or(DispatchError::SlotPoolExhausted {
            mcu_id: sub_plan.mcu_id,
            capacity: cap,
            in_flight,
        })
};

// After:
let alloc_result = slot_pool
    .alloc_blocking(DEFAULT_SLOT_ACQUIRE_TIMEOUT)
    .ok_or_else(|| DispatchError::SlotPoolExhausted {
        mcu_id: sub_plan.mcu_id,
        capacity: slot_pool.capacity(),
        in_flight: slot_pool.in_flight_count(),
    });
```

- [ ] **Step 4: Update all remaining `slot_pool.lock()` calls in the dispatch closure**

Replace all `slot_pool.lock().unwrap_or_else(|p| p.into_inner()).method()` with `slot_pool.method()`:

- `register_segment` calls
- `release` calls (error cleanup path)
- `in_flight_count` calls (diagnostic logging)

- [ ] **Step 5: Remove slot retirement from `on_credit_freed` PyO3 method**

The `on_credit_freed` method currently does slot retirement + credit counter sync + homing completion. All three are now done by the Rust reactor thread's `EventDispatcher` + retirement callback. The PyO3 method is now dead code — but to be safe during the transition, keep it as a no-op that just logs:

```rust
fn on_credit_freed(
    &self,
    _mcu: u32,
    _retired_through_segment_id: u32,
    _free_slots: u8,
) -> PyResult<(u32, Option<u32>)> {
    Ok((0, None))
}
```

- [ ] **Step 6: Update `slot_pool_in_flight` diagnostic method**

```rust
fn slot_pool_in_flight(&self, mcu: u32) -> u32 {
    self.slot_pools
        .lock()
        .unwrap()
        .get(&mcu)
        .map(|p| p.in_flight_count() as u32)
        .unwrap_or(0)
}
```

- [ ] **Step 7: Update test helpers**

Change `install_mcu` and any test code that uses `Arc<Mutex<SlotPool>>` to use `Arc<SharedSlotPool>`:

```rust
fn install_mcu(bridge: &PyMotionBridge, mcu: u32) -> Arc<SharedSlotPool> {
    let pool = Arc::new(SharedSlotPool::new(crate::slot_pool::CURVE_POOL_N));
    bridge
        .slot_pools
        .lock()
        .unwrap()
        .insert(mcu, Arc::clone(&pool));
    pool
}
```

Update test call sites: `pool.lock().unwrap().try_alloc()` → `pool.try_alloc()`, `pool.lock().unwrap().register_segment(...)` → `pool.register_segment(...)`, etc.

- [ ] **Step 8: Build and test**

Run: `cd rust && cargo build -p motion-bridge 2>&1 | tail -20`
Run: `cd rust && cargo test -p motion-bridge -- --nocapture 2>&1 | tail -30`
Expected: All PASS.

- [ ] **Step 9: Commit**

```bash
git add rust/motion-bridge/src/bridge.rs
git commit -m "feat: migrate bridge to SharedSlotPool with blocking acquire and retirement callback"
```

---

### Task 4: Remove Python-side credit_freed handler registration

**Files:**
- Modify: `klippy/motion_toolhead.py`
- Modify: `klippy/serialhdl.py`

- [ ] **Step 1: Remove `_register_credit_freed_handlers` call and method**

In `klippy/motion_toolhead.py`:

Remove the call (around line 764):
```python
# Delete this line:
self._register_credit_freed_handlers(bridge_mcus)
```

Remove the entire `_register_credit_freed_handlers` method (lines 1122-1166).

- [ ] **Step 2: Skip `credit_freed` events in `_bridge_event_poller`**

In `klippy/serialhdl.py`, around line 102:
```python
# Before:
elif ev_type == "credit_freed":
    name = "kalico_credit_freed"

# After:
elif ev_type == "credit_freed":
    continue
```

- [ ] **Step 3: Verify the full workspace builds**

Run: `cd rust && cargo build --workspace 2>&1 | tail -20`
Expected: No errors.

- [ ] **Step 4: Run all workspace tests**

Run: `cd rust && cargo test --workspace 2>&1 | tail -30`
Expected: All PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/motion_toolhead.py klippy/serialhdl.py
git commit -m "refactor: remove Python-side credit_freed slot retirement — now fully Rust-internal"
```

---

### Task 5: Clean up — remove dead `on_credit_freed` PyO3 method

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs`

- [ ] **Step 1: Remove the `on_credit_freed` method entirely**

Since Python no longer calls `bridge.on_credit_freed(...)`, the PyO3 method is dead code. Remove it from the `#[pymethods]` impl block (lines ~2937-2976).

- [ ] **Step 2: Build and test**

Run: `cd rust && cargo build --workspace 2>&1 | tail -20`
Run: `cd rust && cargo test --workspace 2>&1 | tail -30`
Expected: All PASS.

- [ ] **Step 3: Commit**

```bash
git add rust/motion-bridge/src/bridge.rs
git commit -m "refactor: remove dead on_credit_freed PyO3 method"
```
