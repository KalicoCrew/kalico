# Motion Drain via Retired Cursor — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give the host a real "wait until the MCU has physically finished all motion I sent" barrier, so `M400`, homing's `wait_moves_and_mcu`, and `set_position` re-seeding no longer race in-flight pieces.

**Architecture:** The MCU's single per-axis ring cursor is relocated to bump when a piece's time window *ends* (retire) instead of when it is *armed*, and renamed `retired`. The host (bridge) independently tracks, per `(mcu, axis)`, how many pieces it `sent` and the latest `retired` count from the heartbeat, and blocks the G-code thread until `retired == sent` everywhere. Nothing flows back from the pump — the bridge already receives the heartbeat directly and knows what it sent.

**Tech Stack:** Rust (`rust/runtime` MCU engine, `rust/kalico-c-api` FFI, `rust/kalico-protocol` wire, `rust/motion-bridge` host bridge + pump, PyO3), Python (`klippy/motion_toolhead.py`, `klippy/motion_bridge.py`).

**Reference spec:** `docs/superpowers/specs/2026-05-30-motion-drain-retired-cursor-design.md`

---

## Background the implementer must know

- The MCU motion engine plays one cubic piece per axis at a time. `get_position_and_velocity` (`rust/runtime/src/engine.rs`, ~line 680) is a loop with four branches: (1) current armed piece still inside its window → evaluate & return; (2) ring empty → idle; (3) next piece starts too far in the past → hard fault; (4) **arm** the next piece (cache its coefficients) and `ring.pop()`.
- `RingDescriptor` (`rust/runtime/src/piece_ring.rs`, ~line 66) is `#[repr(C)]` with fields `storage, capacity, head, consumed`. `head` is the produced cursor (host pushes), `consumed` is the consumer cursor. **`consumed` is BOTH the read cursor (`peek`/`pop` index at `consumed % capacity`) AND the count reported to the host.** Today `pop()` (the only live increment, `piece_ring.rs:~214`) bumps `consumed` at *arm time* — before the piece has played a single step. That is the bug this plan fixes.
- There is a second, **test-only** `PieceRing<'a>` struct lower in `piece_ring.rs` (~line 280, with its own `pop`/`consumed`) used by host unit tests. It is NOT the engine path. Leave its behavior intact but rename in lockstep for consistency (Task 1 covers it).
- The heartbeat carries these counts: engine `consumed_counts()` (`engine.rs:~367`) → FFI `kalico_runtime_consumed_counts` (`rust/kalico-c-api/src/runtime_ffi.rs:~1718`) → wire `StatusHeartbeat.consumed_counts` (`rust/kalico-protocol/src/messages.rs:~299`) → host `attach_heartbeat_callback` closure in `bridge.rs:~2200` (receives `consumed: &[u32]`) → `PumpMsg::Heartbeat` → pump's flow-control mirror (`pump.rs` `AxisQueue.consumed`).
- The pump (`rust/motion-bridge/src/pump.rs`) tracks per-axis `pushed`/`consumed` purely for its own flow-control `room()`. The drain feature does NOT consult the pump.

## Naming decisions (apply consistently)

- Ring method `pop()` → **`advance_counter()`** (no longer returns the entry; the engine already copies coefficients via `peek` before advancing).
- `RingDescriptor` field `consumed` → **`retired`** (still `#[repr(C)]`, same offset/type — only the name changes). Likewise the test-only `PieceRing`'s field.
- Accessor `consumed_count()` → **`retired_count()`**; engine `consumed_counts()` → **`retired_counts()`**.
- FFI `kalico_runtime_consumed_counts` → **`kalico_runtime_retired_counts`**, param `out_consumed` → `out_retired`.
- Wire field `StatusHeartbeat.consumed_counts` → **`retired_counts`** (same layout, just the field name + docs).
- Pump mirror `AxisQueue.consumed` → **`retired`** (cosmetic honesty; flow-control math unchanged).

## File map

- `rust/runtime/src/piece_ring.rs` — rename field/methods; the bump site moves out of here conceptually (engine calls `advance_counter()` at a new point).
- `rust/runtime/src/engine.rs` — relocate the counter bump in `get_position_and_velocity`; rename `consumed_counts()`.
- `rust/kalico-c-api/src/runtime_ffi.rs` — rename FFI symbol + param.
- `rust/kalico-protocol/src/messages.rs` — rename wire struct field + docs.
- `rust/motion-bridge/src/pump.rs` — rename mirror field.
- `rust/motion-bridge/src/bridge.rs` — `DrainSync` state, heartbeat→retired + dispatch→sent tracking, `drain_motion()` method, `set_position` drains first.
- `klippy/motion_bridge.py` — expose `drain_motion`.
- `klippy/motion_toolhead.py` — `wait_moves_and_mcu` → drain; bridge-mode `M400` → drain.
- C side: grep for any reference to the ring field name (expected none) — Task 3 step 1.

---

## Task 1: Rename ring cursor to `retired` and `pop` → `advance_counter`

**Files:**
- Modify: `rust/runtime/src/piece_ring.rs`
- Test: `rust/runtime/src/piece_ring.rs` (existing inline `#[cfg(test)]` module) or `rust/runtime/tests/`

This task is a pure rename — no semantic change yet. The engine still calls the (renamed) `advance_counter()` from the same Branch 4 site; Task 2 moves it.

- [ ] **Step 1: Confirm current code**

Run: `grep -n "consumed\|fn pop\|consumed_count" rust/runtime/src/piece_ring.rs`
Expected: see `pub consumed: u32` in `RingDescriptor`, `pub fn pop(&mut self) -> Option<&PieceEntry>` (bumps `self.consumed`), `pub fn consumed_count(&self) -> u32`, and a second test-only `PieceRing` with `consumed`/`pop`/`consumed_count`.

- [ ] **Step 2: Rename the `RingDescriptor` field and its doc**

In `RingDescriptor`, rename `pub consumed: u32` → `pub retired: u32`. Update the struct/field doc comments that say "consumed" to "retired" (the cursor now counts pieces whose window has fully elapsed). Update `new`/constructors initializing `consumed: 0` → `retired: 0`. Update `len()` (`head.wrapping_sub(self.consumed)` → `self.retired`), `is_empty()` (`head == self.consumed` → `self.retired`), `set_head` (the two `self.consumed` reads → `self.retired`), `reset()` (`self.consumed = 0` → `self.retired = 0`), and `peek`/`pop` slot index (`self.consumed % self.capacity` → `self.retired % self.capacity`).

- [ ] **Step 3: Rename `pop` → `advance_counter` and drop the return value**

Replace:

```rust
    /// Pop the tail entry (advance `consumed`). No-op return `None` when empty.
    pub fn pop(&mut self) -> Option<&PieceEntry> {
        if self.is_empty() {
            return None;
        }
        let slot = (self.consumed % self.capacity) as usize;
        self.consumed = self.consumed.wrapping_add(1);
        Some(unsafe { &*self.storage.add(slot) })
    }
```

with:

```rust
    /// Advance the retire cursor by one (the front piece's window has fully
    /// elapsed). No-op when empty. The engine copies the piece's coefficients
    /// via `peek` before playing it, so nothing here needs to return the entry.
    pub fn advance_counter(&mut self) {
        if self.is_empty() {
            return;
        }
        self.retired = self.retired.wrapping_add(1);
    }
```

- [ ] **Step 4: Rename the accessor**

Replace `pub fn consumed_count(&self) -> u32 { self.consumed }` with:

```rust
    /// Monotonic count of pieces whose window has fully elapsed (wrapping u32).
    pub fn retired_count(&self) -> u32 {
        self.retired
    }
```

- [ ] **Step 5: Rename the test-only `PieceRing` in lockstep**

In the lower test-only `PieceRing<'a>` struct: field `consumed` → `retired`, `fn pop` keeps its name and return (it's test infrastructure that genuinely removes entries) **but** rename its `consumed_count()` → `retired_count()` and the `self.consumed` references → `self.retired`. Update the doc example (`assert_eq!(ring.consumed_count(), 1);` → `retired_count`).

- [ ] **Step 6: Fix the engine call site to compile**

Run: `grep -rn "\.pop()\|consumed_count\|\.consumed" rust/runtime/src/engine.rs`
In `engine.rs`, change `axis.ring.pop();` (Branch 4, ~line 727) → `axis.ring.advance_counter();` and `axis.ring.consumed_count()` (~line 372) → `axis.ring.retired_count()`. (Task 2 moves the call; this keeps it compiling now.)

- [ ] **Step 7: Build the runtime crate**

Run: `cd rust && cargo build -p runtime`
Expected: compiles clean (only this crate's refs changed).

- [ ] **Step 8: Run runtime tests**

Run: `cd rust && cargo test -p runtime`
Expected: PASS. Existing piece-tick tests still assert arm-time semantics here (the bump hasn't moved yet) — they should still pass because the call site is unchanged in timing.

- [ ] **Step 9: Commit**

```bash
git add rust/runtime/src/piece_ring.rs rust/runtime/src/engine.rs
git commit -m "refactor(runtime): rename ring cursor consumed->retired, pop->advance_counter"
```

---

## Task 2: Relocate the bump to retire-time in the engine loop

**Files:**
- Modify: `rust/runtime/src/engine.rs` (`get_position_and_velocity`, ~line 680)
- Test: `rust/runtime/tests/piece_tick.rs`

After this task, the cursor counts pieces whose window has **ended**, and the currently-playing piece's slot stays occupied until it retires.

- [ ] **Step 1: Write a failing test pinning retire semantics**

Add to `rust/runtime/tests/piece_tick.rs` (adapt the existing harness in that file — it builds an engine, pushes pieces, and ticks `get_position_and_velocity`; mirror its setup helpers):

```rust
#[test]
fn retired_count_bumps_at_window_end_not_arm() {
    // One axis, two pieces back-to-back, each duration D (in cycles).
    // While piece 0 is playing, retired must be 0 (nothing finished yet).
    // Only after `now` passes piece 0's end does retired become 1.
    let mut h = TestEngine::single_axis(); // existing helper in this file
    let d = h.cycles_for_secs(0.010);      // 10ms piece
    h.push_piece(0, /*start*/ 0, /*dur*/ d, /*coeffs*/ [0.0, 1.0, 0.0, 0.0]);
    h.push_piece(0, /*start*/ d, /*dur*/ d, /*coeffs*/ [10.0, 1.0, 0.0, 0.0]);

    // Tick mid-way through piece 0.
    h.tick(d / 2);
    assert_eq!(h.retired_counts()[0], 0, "piece 0 still playing -> retired 0");

    // Tick mid-way through piece 1 (past piece 0's end).
    h.tick(d + d / 2);
    assert_eq!(h.retired_counts()[0], 1, "piece 0 finished -> retired 1");

    // Tick past piece 1's end (ring drains).
    h.tick(2 * d + 1);
    assert_eq!(h.retired_counts()[0], 2, "both finished -> retired == sent(2)");
}
```

If the existing harness uses different helper names, match them; the asserted *values* (0, then 1, then 2) are the contract.

- [ ] **Step 2: Run it to confirm it fails**

Run: `cd rust && cargo test -p runtime --test piece_tick retired_count_bumps_at_window_end_not_arm`
Expected: FAIL — with arm-time bump, the mid-piece-0 tick already shows `retired == 1`.

- [ ] **Step 3: Relocate the bump in `get_position_and_velocity`**

Replace the loop body:

```rust
    loop {
        // Branch 1: current piece still relevant.
        if axis.has_piece && now < axis.piece_end_cycles {
            return Some(eval_horner(
                &axis.mono_coeffs, &axis.vel_coeffs,
                axis.piece_start_cycles, now, cycles_per_second,
            ));
        }

        // Branch 2: nothing there -> idle / underrun.
        let Some(next_entry) = axis.ring.peek(storage).copied() else {
            axis.has_piece = false;
            return None;
        };

        // Branch 3: start-in-past fault.
        let fault_tolerance = u64::from(sample_period_cycles) * 2;
        if now.saturating_sub(next_entry.start_time) > fault_tolerance {
            raise_piece_start_in_past(shared, axis_idx);
            axis.has_piece = false;
            return None;
        }

        // Branch 4: arm the piece (cache coeffs BEFORE freeing the slot), then
        // loop to re-test Branch 1.
        let (mono, vel) = next_entry.to_monomial();
        axis.mono_coeffs = mono;
        axis.vel_coeffs = vel;
        axis.piece_start_cycles = next_entry.start_time;
        axis.piece_end_cycles = next_entry.end_time(cycles_per_second);
        axis.has_piece = true;
        axis.ring.pop();
    }
```

with:

```rust
    loop {
        // Branch 1: current armed piece still inside its window.
        if axis.has_piece && now < axis.piece_end_cycles {
            return Some(eval_horner(
                &axis.mono_coeffs, &axis.vel_coeffs,
                axis.piece_start_cycles, now, cycles_per_second,
            ));
        }

        // The armed piece (if any) has now finished its window. Retire it:
        // advance the cursor (which also advances the read position to the
        // next slot) and mark no piece armed. This is the ONLY bump site, so
        // `retired` counts pieces whose window has fully elapsed.
        if axis.has_piece {
            axis.ring.advance_counter();
            axis.has_piece = false;
        }

        // Branch 2: nothing more to play -> idle / underrun. `retired` now
        // equals `head` (== host's `sent`), the host's "motion done" signal.
        let Some(next_entry) = axis.ring.peek(storage).copied() else {
            return None;
        };

        // Branch 3: start-in-past fault.
        let fault_tolerance = u64::from(sample_period_cycles) * 2;
        if now.saturating_sub(next_entry.start_time) > fault_tolerance {
            raise_piece_start_in_past(shared, axis_idx);
            return None;
        }

        // Branch 4: arm the next piece (cache coeffs). Do NOT advance the
        // cursor here — the slot stays occupied until this piece retires.
        let (mono, vel) = next_entry.to_monomial();
        axis.mono_coeffs = mono;
        axis.vel_coeffs = vel;
        axis.piece_start_cycles = next_entry.start_time;
        axis.piece_end_cycles = next_entry.end_time(cycles_per_second);
        axis.has_piece = true;
    }
```

Note: `peek` now reads at `retired % capacity`, which is exactly the currently-armed-or-next piece, so re-arming reads the correct slot. Branch 3 no longer needs to set `axis.has_piece = false` because we only reach it with `has_piece` already false (it was cleared above when the prior piece retired, or was false on first entry).

- [ ] **Step 4: Run the new test**

Run: `cd rust && cargo test -p runtime --test piece_tick retired_count_bumps_at_window_end_not_arm`
Expected: PASS.

- [ ] **Step 5: Update any existing piece-tick tests that asserted arm-time counts**

Run: `cd rust && cargo test -p runtime`
For any test that now fails because it asserted `consumed/retired == 1` *during* the first piece, update its expected value to reflect retire-time semantics (the count should be the number of pieces whose window has fully passed at that `now`). Each such change is the test catching the intended behavior change — adjust the expected number, do not revert the engine.

- [ ] **Step 6: Run full runtime test suite**

Run: `cd rust && cargo test -p runtime`
Expected: PASS.

- [ ] **Step 7: Commit**

```bash
git add rust/runtime/src/engine.rs rust/runtime/tests/piece_tick.rs
git commit -m "feat(runtime): retire cursor at piece window end, not at arm"
```

---

## Task 3: Rename FFI + wire field; sweep C for stale references

**Files:**
- Modify: `rust/kalico-c-api/src/runtime_ffi.rs`, `rust/kalico-protocol/src/messages.rs`
- Modify: `rust/runtime/src/engine.rs` (`consumed_counts` → `retired_counts`)
- Sweep: `src/`, `klippy/chelper/` for the FFI symbol / field name

- [ ] **Step 1: Sweep C and headers for the ring field and FFI symbol**

Run: `grep -rn "kalico_runtime_consumed_counts\|consumed_counts\|->consumed\|\.consumed" src/ klippy/chelper/ firmware/ 2>/dev/null`
Expected: the only hits should be the FFI symbol declaration (if C declares the extern) and possibly heartbeat-builder C code. Note each file/line — they must be renamed to `retired` in the steps below. If C accesses the ring's `consumed` field by name, rename it there too (the `#[repr(C)]` offset is unchanged, so only name references break). If there are zero C hits, record that and proceed.

- [ ] **Step 2: Rename the engine accessor**

In `rust/runtime/src/engine.rs`, rename `pub fn consumed_counts(&self) -> [u32; MAX_AXES]` → `retired_counts`, and its body's `axis.ring.retired_count()` is already correct from Task 1. Update the doc comment ("per-axis consumed piece counts" → "per-axis retired piece counts").

- [ ] **Step 3: Rename the FFI symbol and param**

In `rust/kalico-c-api/src/runtime_ffi.rs`, replace:

```rust
    #[no_mangle]
    pub extern "C" fn kalico_runtime_consumed_counts(
        engine: *mut MotionEngine,
        out_consumed: *mut u32,
        max_axes: usize,
    ) -> i32 {
        if engine.is_null() || out_consumed.is_null() {
            return -1;
        }
        let engine = unsafe { &*engine };
        let counts = engine.consumed_counts();
        let n = max_axes.min(counts.len());
        for i in 0..n {
            unsafe { out_consumed.add(i).write(counts[i]) };
        }
        0
    }
```

with:

```rust
    #[no_mangle]
    pub extern "C" fn kalico_runtime_retired_counts(
        engine: *mut MotionEngine,
        out_retired: *mut u32,
        max_axes: usize,
    ) -> i32 {
        if engine.is_null() || out_retired.is_null() {
            return -1;
        }
        let engine = unsafe { &*engine };
        let counts = engine.retired_counts();
        let n = max_axes.min(counts.len());
        for i in 0..n {
            unsafe { out_retired.add(i).write(counts[i]) };
        }
        0
    }
```

Update the doc comment above it accordingly.

- [ ] **Step 4: Update C callers found in Step 1**

For each C file from Step 1 that calls `kalico_runtime_consumed_counts` or names the ring `consumed` field, rename to `kalico_runtime_retired_counts` / `retired`. If the symbol is declared in a generated header or `__init__.py` cffi cdef (`klippy/chelper/__init__.py`), update that declaration too.

- [ ] **Step 5: Rename the wire field**

In `rust/kalico-protocol/src/messages.rs`, in `StatusHeartbeat`: rename `pub consumed_counts: Vec<u32>` → `pub retired_counts: Vec<u32>`. Update the struct doc (the wire-layout comment: `consumed_counts: num_axes × u32_le` → `retired_counts: …`; "The host uses `consumed_counts`…" → "retired_counts"). Update `Encode` (`self.consumed_counts` → `self.retired_counts`, both the `.len()` and the loop) and `Decode` (local `consumed_counts` → `retired_counts`, and the struct field in the returned literal). Update the inline tests in `messages/tests.rs` (`consumed_counts: vec![]` → `retired_counts`, `decoded.consumed_counts.len()` → `retired_counts`).

- [ ] **Step 6: Build the affected crates**

Run: `cd rust && cargo build -p kalico-protocol -p kalico-c-api -p runtime`
Expected: compiles clean.

- [ ] **Step 7: Run their tests**

Run: `cd rust && cargo test -p kalico-protocol -p runtime`
Expected: PASS.

- [ ] **Step 8: Commit**

```bash
git add rust/runtime/src/engine.rs rust/kalico-c-api/src/runtime_ffi.rs rust/kalico-protocol/src/messages.rs rust/kalico-protocol/src/messages/tests.rs klippy/chelper/__init__.py
git commit -m "refactor(protocol,ffi): rename consumed_counts -> retired_counts"
```

---

## Task 4: Rename the pump's flow-control mirror to `retired`

**Files:**
- Modify: `rust/motion-bridge/src/pump.rs`

Cosmetic honesty only — the flow-control math is unchanged. The mirror now holds retire counts; with the playing piece's slot held until retirement, `room()` is conservative by one slot per axis (intended, prevents overwriting a playing slot).

- [ ] **Step 1: Confirm current code**

Run: `grep -n "consumed\|HeartbeatMsg\|consumed_counts" rust/motion-bridge/src/pump.rs`
Expected: `AxisQueue.consumed: u32`, `room()` uses `self.pushed.wrapping_sub(self.consumed)`, the `Heartbeat` arm sets `q.consumed = c`, and `HeartbeatMsg.consumed_counts`.

- [ ] **Step 2: Rename the `AxisQueue` field**

`pub consumed: u32` → `pub retired: u32`; constructor `consumed: 0` → `retired: 0`; `room()` `self.pushed.wrapping_sub(self.consumed)` → `self.pushed.wrapping_sub(self.retired)`. Update the field doc and the `room_*` test bodies (`q.consumed = …` → `q.retired = …`).

- [ ] **Step 3: Rename `HeartbeatMsg.consumed_counts` and the handler**

`HeartbeatMsg { mcu_id, consumed_counts: Vec<u32> }` → `retired_counts`. In the `PumpMsg::Heartbeat` arm, `for (axis, &c) in consumed_counts.iter()…` → `retired_counts`, and `q.consumed = c` → `q.retired = c`. Update the `PIECEDIAG HB` log label if desired (`consumed=` → `retired=`).

- [ ] **Step 4: Build + test the pump**

Run: `cd rust && cargo test -p motion-bridge --lib pump`
Expected: PASS (rename-only; `room_full_then_drains` and `room_correct_across_u32_wrap` use the renamed field).

- [ ] **Step 5: Commit**

```bash
git add rust/motion-bridge/src/pump.rs
git commit -m "refactor(motion-bridge): rename pump mirror consumed -> retired"
```

---

## Task 5: Add `DrainSync` state to the bridge

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs` (struct + constructor, near the `PyMotionBridge` definition ~line 344 and `new()` ~line 638)
- Create: a small module `rust/motion-bridge/src/drain.rs` for the sync type + its unit tests

Keeping the drain primitive in its own file keeps `bridge.rs` (already ~3000 lines) from growing a tangled new concern.

- [ ] **Step 1: Write the failing unit test for the predicate**

Create `rust/motion-bridge/src/drain.rs`:

```rust
//! Host-side motion drain: the bridge tracks, per (mcu, axis), how many pieces
//! it has `sent` to the wire and the latest `retired` count from the heartbeat.
//! `drain` blocks until `retired == sent` for every axis that has been sent to.
//! Nothing flows back from the pump — the heartbeat callback feeds `retired`
//! directly and the dispatch path feeds `sent`.

use std::collections::HashMap;
use std::sync::{Condvar, Mutex};
use std::time::{Duration, Instant};

type AxisKey = (u32, u8); // (mcu_id, axis)

#[derive(Default)]
struct Counts {
    sent: HashMap<AxisKey, u32>,
    retired: HashMap<AxisKey, u32>,
}

pub struct DrainSync {
    counts: Mutex<Counts>,
    cv: Condvar,
}

impl DrainSync {
    pub fn new() -> Self {
        Self { counts: Mutex::new(Counts::default()), cv: Condvar::new() }
    }

    /// Record `n` pieces handed to the wire for `(mcu, axis)`.
    pub fn add_sent(&self, mcu: u32, axis: u8, n: u32) {
        let mut c = self.counts.lock().unwrap_or_else(|p| p.into_inner());
        let e = c.sent.entry((mcu, axis)).or_insert(0);
        *e = e.wrapping_add(n);
        // No notify: more `sent` can only delay the predicate.
    }

    /// Update the latest retired count for `(mcu, axis)` from a heartbeat.
    pub fn set_retired(&self, mcu: u32, axis: u8, retired: u32) {
        let mut c = self.counts.lock().unwrap_or_else(|p| p.into_inner());
        c.retired.insert((mcu, axis), retired);
        drop(c);
        self.cv.notify_all();
    }

    /// Reset all counters (stream re-open / ring reset). Both sides go to 0.
    pub fn reset(&self) {
        let mut c = self.counts.lock().unwrap_or_else(|p| p.into_inner());
        c.sent.clear();
        c.retired.clear();
        drop(c);
        self.cv.notify_all();
    }

    /// True iff every axis with sent>0 has retired == sent.
    fn is_drained(c: &Counts) -> bool {
        c.sent.iter().all(|(k, &s)| c.retired.get(k).copied().unwrap_or(0) == s)
    }

    /// Block until drained or `timeout` elapses. Returns Err(message) on timeout.
    pub fn wait_drained(&self, timeout: Duration) -> Result<(), String> {
        let deadline = Instant::now() + timeout;
        let mut c = self.counts.lock().unwrap_or_else(|p| p.into_inner());
        while !Self::is_drained(&c) {
            let now = Instant::now();
            if now >= deadline {
                // Snapshot the lagging axes for a loud, actionable error.
                let lagging: Vec<String> = c
                    .sent
                    .iter()
                    .filter(|(k, &s)| c.retired.get(k).copied().unwrap_or(0) != s)
                    .map(|(k, &s)| {
                        let r = c.retired.get(k).copied().unwrap_or(0);
                        format!("mcu{} axis{}: retired {} / sent {}", k.0, k.1, r, s)
                    })
                    .collect();
                return Err(format!(
                    "motion drain timed out after {:?}; not finished: [{}]",
                    timeout,
                    lagging.join(", ")
                ));
            }
            let (guard, _) =
                self.cv.wait_timeout(c, deadline - now).unwrap_or_else(|p| p.into_inner());
            c = guard;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn drained_when_retired_equals_sent() {
        let d = DrainSync::new();
        d.add_sent(1, 0, 3);
        d.add_sent(1, 1, 2);
        // Not drained yet.
        assert!(d.wait_drained(Duration::from_millis(20)).is_err());
        d.set_retired(1, 0, 3);
        d.set_retired(1, 1, 2);
        // Now drained immediately.
        assert!(d.wait_drained(Duration::from_millis(20)).is_ok());
    }

    #[test]
    fn no_sent_is_trivially_drained() {
        let d = DrainSync::new();
        assert!(d.wait_drained(Duration::from_millis(20)).is_ok());
    }

    #[test]
    fn reset_clears_both_sides() {
        let d = DrainSync::new();
        d.add_sent(1, 0, 5);
        d.reset();
        assert!(d.wait_drained(Duration::from_millis(20)).is_ok());
    }
}
```

- [ ] **Step 2: Register the module**

Run: `grep -n "^mod \|^pub mod \|^pub(crate) mod " rust/motion-bridge/src/lib.rs`
Add `mod drain;` alongside the other module declarations in `rust/motion-bridge/src/lib.rs`.

- [ ] **Step 3: Run the test to verify it passes**

Run: `cd rust && cargo test -p motion-bridge --lib drain`
Expected: PASS (3 tests).

- [ ] **Step 4: Add the field to `PyMotionBridge` and construct it**

Run: `grep -n "struct PyMotionBridge\|pump_thread: Mutex" rust/motion-bridge/src/bridge.rs`
In the `PyMotionBridge` struct add: `drain: std::sync::Arc<crate::drain::DrainSync>,`
In `new()` (alongside `pump_tx: Mutex::new(None),`) add: `drain: std::sync::Arc::new(crate::drain::DrainSync::new()),`

- [ ] **Step 5: Build**

Run: `cd rust && cargo build -p motion-bridge`
Expected: compiles (field unused for now — that's fine, it's wired in Task 6).

- [ ] **Step 6: Commit**

```bash
git add rust/motion-bridge/src/drain.rs rust/motion-bridge/src/lib.rs rust/motion-bridge/src/bridge.rs
git commit -m "feat(motion-bridge): DrainSync state for sent/retired tracking"
```

---

## Task 6: Feed `sent` from dispatch and `retired` from the heartbeat

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs` (`init_planner`: heartbeat callback ~line 2200 and the dispatch→enqueue closure ~line 2223-2270)

- [ ] **Step 1: Confirm the two sites**

Run: `grep -n "attach_heartbeat_callback\|PumpMsg::Enqueue\|PumpMsg::Heartbeat\|pump_tx_for_cb\|pump_tx_hb" rust/motion-bridge/src/bridge.rs`
Expected: the heartbeat callback closure (captures `pump_tx_hb`, receives `consumed: &[u32]`) and the dispatch closure (captures `pump_tx_for_cb`, sends `PumpMsg::Enqueue(m)` per `EnqueueMsg`).

- [ ] **Step 2: Clone the drain handle for both closures**

Just before the heartbeat callback is attached, add:

```rust
        let drain_hb = self.drain.clone();
```

and just before the dispatch closure is created, add:

```rust
        let drain_disp = self.drain.clone();
```

- [ ] **Step 3: Update the heartbeat callback to record retired**

Inside the `attach_heartbeat_callback` closure (it already has `mcu_id` in scope and `consumed: &[u32]` — note the param is the renamed `retired_counts` payload now, same shape), after the existing `pump_tx_hb.send(...)`, add:

```rust
            for (axis, &r) in consumed.iter().enumerate() {
                drain_hb.set_retired(mcu_id, axis as u8, r);
            }
```

(Leave the closure param named `consumed` or rename to `retired` for clarity — cosmetic.)

- [ ] **Step 4: Update the dispatch closure to record sent**

At the site that sends each `EnqueueMsg` to the pump (the `pump_tx_for_cb.send(PumpMsg::Enqueue(m))` loop), record the count BEFORE moving `m`:

```rust
            for m in enqueue_msgs {              // match the existing iteration var
                drain_disp.add_sent(m.key.mcu_id, m.key.axis, m.pieces.len() as u32);
                if let Err(e) = pump_tx_for_cb.send(crate::pump::PumpMsg::Enqueue(m)) {
                    log::error!("pump enqueue send failed: {e}");
                }
            }
```

If the existing code sends a single `m` (not a loop), add the one `drain_disp.add_sent(...)` line immediately before that send. The key invariant: every piece handed to the pump increments `sent` exactly once for its `(mcu, axis)`.

- [ ] **Step 5: Build**

Run: `cd rust && cargo build -p motion-bridge`
Expected: compiles clean.

- [ ] **Step 6: Run motion-bridge tests**

Run: `cd rust && cargo test -p motion-bridge`
Expected: PASS (existing tests unaffected; drain unit tests still green).

- [ ] **Step 7: Commit**

```bash
git add rust/motion-bridge/src/bridge.rs
git commit -m "feat(motion-bridge): track sent in dispatch, retired in heartbeat"
```

---

## Task 7: Add the `drain_motion` PyO3 method + reset hook

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs` (new `#[pymethods]` entry near `wait_moves` ~line 2365; reset at the `kalico_stream_open` site(s))

- [ ] **Step 1: Find the stream-open / reset sites**

Run: `grep -n "kalico_stream_open\|fn wait_moves\|fn set_position\|fn init_planner" rust/motion-bridge/src/bridge.rs`
Note every `kalico_stream_open` call (at minimum `set_position` ~line 2700, and possibly `init_planner`). Each is a point where the MCU ring resets `retired`→0, so the host's `sent`/`retired` must reset too.

- [ ] **Step 2: Add `drain_motion`**

Add inside the `#[pymethods] impl PyMotionBridge` block, next to `wait_moves`:

```rust
    /// Block the calling (G-code) thread until every axis the bridge has sent
    /// to has `retired == sent` — i.e. the MCU has physically finished all
    /// queued motion. Flushes the planner first (which shapes + dispatches the
    /// decel-to-zero tail). The reactor/heartbeat threads keep running while we
    /// wait (GIL released). Loud timeout error on a wedged MCU.
    fn drain_motion(&self, py: Python<'_>) -> PyResult<()> {
        let planner = self.planner.get().ok_or_else(|| {
            PyRuntimeError::new_err("planner not initialized — call init_planner first")
        })?;
        // 1) Flush: dispatch the held-back decel-to-zero tail to the pump.
        py.allow_threads(|| planner.flush()).map_err(planner_err)?;
        // 2) Wait for the MCU to retire everything we sent.
        let drain = self.drain.clone();
        py.allow_threads(|| drain.wait_drained(DRAIN_TIMEOUT))
            .map_err(PyRuntimeError::new_err)?;
        self.homing.refresh_after_wait();
        Ok(())
    }
```

- [ ] **Step 3: Define the timeout constant**

Near the other bridge timeout constants (e.g. `CLOCK_SYNC_INTERVAL`), add:

```rust
/// Upper bound on a single motion drain. A real print's longest queued motion
/// plus buffer is well under this; exceeding it means a wedged MCU -> loud fail.
const DRAIN_TIMEOUT: Duration = Duration::from_secs(60);
```

- [ ] **Step 4: Reset drain counters at every stream-open site**

At each `kalico_stream_open` call found in Step 1, add immediately after it:

```rust
        self.drain.reset();
```

This keeps host `sent` aligned with the MCU's freshly-zeroed `retired`. (In `set_position` this lands after the flush+drain of Task 8 and right next to the existing stream-open.)

- [ ] **Step 5: Build**

Run: `cd rust && cargo build -p motion-bridge`
Expected: compiles clean.

- [ ] **Step 6: Commit**

```bash
git add rust/motion-bridge/src/bridge.rs
git commit -m "feat(motion-bridge): drain_motion barrier + reset on stream open"
```

---

## Task 8: Make `set_position` drain before re-seeding

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs` (`set_position` ~line 2688)

- [ ] **Step 1: Confirm current ordering**

Run: `grep -n "fn set_position\|kalico_stream_open\|build_seed_sends\|runtime_seed_position\|ordering against in-flight" rust/motion-bridge/src/bridge.rs`
Expected: `set_position` calls `planner.kalico_stream_open([x,y,z,0.0])`, then `build_seed_sends`, then fire-and-forget `runtime_seed_position`, with a comment saying ordering against in-flight pieces is out of scope.

- [ ] **Step 2: Drain before the stream re-open**

At the top of `set_position`'s body (before `kalico_stream_open`), add a flush+drain so no axis is mid-motion when we re-seed:

```rust
        // Re-seeding position while an axis is still stepping would stomp a
        // moving axis. Wait for the MCU to physically finish first.
        let planner = self.planner.get().ok_or_else(|| {
            PyRuntimeError::new_err("planner not initialized — call init_planner first")
        })?;
        py.allow_threads(|| planner.flush()).map_err(planner_err)?;
        {
            let drain = self.drain.clone();
            py.allow_threads(|| drain.wait_drained(DRAIN_TIMEOUT))
                .map_err(PyRuntimeError::new_err)?;
        }
```

Ensure `set_position`'s signature has `py: Python<'_>` (add it if absent; PyO3 injects it). If the method already fetches `planner` later, reuse this binding instead of fetching twice.

- [ ] **Step 3: Replace the "out of scope" comment**

Replace the comment that says ordering against in-flight pieces is out of scope with:

```rust
        // Ordering against in-flight pieces IS handled: we drained above, so
        // every axis has retired == sent before this re-seed. The stream
        // re-open below zeroes both the MCU ring and the host drain counters.
```

- [ ] **Step 4: Confirm the reset from Task 7 lands here**

The `self.drain.reset();` added after `kalico_stream_open` in Task 7 Step 4 applies here. Verify it is present right after the `kalico_stream_open` call in `set_position`.

- [ ] **Step 5: Build + test**

Run: `cd rust && cargo build -p motion-bridge && cargo test -p motion-bridge`
Expected: compiles, tests PASS.

- [ ] **Step 6: Commit**

```bash
git add rust/motion-bridge/src/bridge.rs
git commit -m "fix(motion-bridge): drain motion before set_position re-seed"
```

---

## Task 9: Wire the host Python entry points to drain

**Files:**
- Modify: `klippy/motion_bridge.py` (~line 314, the `wait_moves`/`submit_dwell` wrappers)
- Modify: `klippy/motion_toolhead.py` (`wait_moves_and_mcu` ~line 710; bridge-mode `M400`)

- [ ] **Step 1: Expose `drain_motion` on the bridge wrapper**

Run: `grep -n "def wait_moves\|def submit_dwell\|def set_position" klippy/motion_bridge.py`
After the `wait_moves` wrapper (~line 314), add:

```python
    def drain_motion(self):
        return self._bridge.drain_motion()
```

- [ ] **Step 2: Point `wait_moves_and_mcu` at the real drain**

Run: `grep -n "def wait_moves_and_mcu\|def flush_step_generation\|def wait_moves" klippy/motion_toolhead.py`
Replace:

```python
    def wait_moves_and_mcu(self):
        self.flush_step_generation()
```

with:

```python
    def wait_moves_and_mcu(self):
        # Real MCU-completion barrier: block until the MCU has physically
        # retired every piece we sent (not just dispatched to the wire).
        self.bridge.drain_motion()
        self._ground_pending_end_time_after_bridge_drain()
```

(Keep the `_ground_pending_end_time_after_bridge_drain()` call so downstream MCU-clock scheduling stays grounded, matching `flush_step_generation`.)

- [ ] **Step 3: Route bridge-mode `M400` through the drain**

Run: `grep -n "M400\|cmd_M400\|register_command" klippy/motion_toolhead.py klippy/toolhead.py`
Determine how `M400` is registered in bridge mode. `toolhead.py:353` registers `M400 -> cmd_M400` which calls `self.wait_moves()`. In `MotionToolhead`, override so `M400` drains. Add to `MotionToolhead`:

```python
    def cmd_M400(self, gcmd):
        # Wait for ALL the moves in the queue to physically finish on the MCU.
        self.wait_moves_and_mcu()
```

and ensure it is registered. If `MotionToolhead` runs the upstream init that already does `gcode.register_command("M400", self.cmd_M400)` (see the "Run upstream init: … M400 …" comment ~line 324), this override is picked up automatically because `cmd_M400` resolves on the subclass. If `M400` is registered directly to `wait_moves`, change that registration to `self.cmd_M400`. Confirm via grep which path applies and wire accordingly.

- [ ] **Step 4: Sanity-check the change compiles/loads**

Run: `python -c "import ast; ast.parse(open('klippy/motion_toolhead.py').read()); ast.parse(open('klippy/motion_bridge.py').read()); print('ok')'`
Expected: `ok` (syntax valid).

- [ ] **Step 5: Commit**

```bash
git add klippy/motion_bridge.py klippy/motion_toolhead.py
git commit -m "feat(host): M400 and wait_moves_and_mcu drain the MCU motion queue"
```

---

## Task 10: Build everything + end-to-end validation in the simulator

**Files:** none (build + sim verification)

- [ ] **Step 1: Full workspace build**

Run: `cd rust && cargo build --workspace`
Expected: clean build across all crates.

- [ ] **Step 2: Full workspace test**

Run: `cd rust && cargo test --workspace`
Expected: PASS. Pay attention to any protocol round-trip test asserting the heartbeat field name.

- [ ] **Step 3: Exercise drain in the simulator**

Use the kalico-sim skill/harness (see the `kalico-sim` skill) to run a short G-code program ending in `M400` against the real firmware in the Docker simulator. Confirm from logs that after `M400` the host observed `retired == sent` on all axes before the next command, and that a following `G92`/`SET_POSITION` (set_position path) is issued only after motion stopped.
Expected: no "drain timed out" error; `M400` returns only after the final piece's window elapses (not one piece early).

- [ ] **Step 4: Confirm the early-return bug is gone**

In the sim, issue a long move immediately followed by `SET_POSITION` (or `G92`) and verify the seeded position is applied only after the move fully completes (no position discontinuity / no overlap of seed with motion).
Expected: clean stop, then seed.

- [ ] **Step 5: Commit any sim harness notes (if a reusable script was produced)**

```bash
git add docs/ tests/   # only if a reusable sim script/fixture was added
git commit -m "test: simulator validation of motion drain (M400 / set_position)"
```

- [ ] **Step 6: Bench flash (hardware, when ready)**

Per the `flashing-trident-mcus` skill, flash BOTH MCUs (H7 + F446) with the new firmware and rebuild the host `motion_bridge_native.so` — the new `retired` cursor semantics and renamed heartbeat field must match on both ends. (Memory: flash H7 from `.config.h7.bak`, F446 from `.config.f446.test`; `make clean` between C builds.)

---

## Notes for the implementer

- **TDD anchors:** Tasks 2 and 5 have real failing-test-first steps; the rest are renames + wiring whose verification is "compiles + existing suites stay green." Don't skip running the suite after each task — a missed rename surfaces as a compile error in a downstream crate.
- **Line numbers drift** as you edit. Every task's Step 1 is a `grep` to re-anchor against the live file — trust the symbol names, not the line numbers.
- **Fail loudly:** per the spec and CLAUDE.md, no edge-case recovery. The only soft path is the drain timeout, which raises a `PyRuntimeError` (loud) rather than hanging.
- **No second counter:** if you find yourself adding a `consumed` *and* a `retired`, stop — there is exactly one cursor per axis; it just bumps later now.
```
