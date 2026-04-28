# Layer 4 MCU Framework Stub Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the partial Layer-4 MCU runtime — a 40 kHz hard-real-time TIM5 ISR on STM32H723 consuming a `heapless::spsc::Queue<Segment, 8>` of pre-loaded NURBS segments, evaluating with `nurbs::vector_eval`, mixing CoreXY+identity to AB+E motors, writing to a host-readable trace ring. Trace-only output stage at this step. Runtime-evaluation slots for PA (Step 9) and IS (Step 8) are designed in as ZST `Noop` traits at compile time.

**Architecture:** New pure-Rust `no_std` crate `runtime/`. Renamed umbrella staticlib crate `kalico-c-api/` (was `nurbs-c-api/`) holds the single `#[panic_handler]` and exports two cbindgen headers (`kalico_nurbs.h` regenerated; `kalico_runtime.h` new). Klipper-C side adds `src/stm32/kalico_h7_timer.c` (H7-specific TIM5 + IRQ handler), `src/runtime_tick.c` (portable `DECL_INIT`/`DECL_TASK`/`DECL_COMMAND` glue), and a small patch to `src/stm32/watchdog.c` adding a kalico-runtime-controlled liveness gate.

**Tech Stack:** Rust 2024 edition, `nurbs` (existing Layer 0, no_std-capable via `mcu-h7` feature), `heapless 0.8` (SPSC queues), `cbindgen` (existing pattern, extended for two headers), Klipper's existing C build system (Kconfig + per-MCU Makefile under `src/stm32/`), STM32H723 (BTT Octopus Pro target).

**Spec:** `docs/superpowers/specs/2026-04-28-layer-4-mcu-framework-stub-design.md`. **Read the full spec end-to-end before starting any task** — every architectural decision is recorded there with rationale, including review-cycle findings the implementer should not re-litigate.

---

## Pre-Flight

Before Task 0 — read the spec end-to-end. Particularly:
- §2 architecture (TIM5 choice, NVIC priority direction, link-line shape).
- §3.1 component types, especially the `Engine<P, I>` Cargo-feature-gated instantiation pattern.
- §3.2 init-once `UnsafeCell + AtomicU8` state machine (`INIT_STATE` UNINIT/INITING/READY) — distinct from `runtime_status` (IDLE/RUNNING/DRAINED/FAULT).
- §4.1 CYCCNT widening with Klipper's `timer_read_time()` as backstop on long disables.
- §4.2 hot-path ISR ordering (NURBS → NaN check → kinematics → PA → IS → trace).
- §4.3 trace-overflow protocol (separate `AtomicBool`, carried into next sample, host-side debounce).
- §4.4 producer push protocol (Release enqueue + Acquire status load + UIF-clear-before-CEN-set).
- §4.7 concurrency invariants table — every shared object's mechanism is locked here.
- §5.3 panic exclusion lint policy + LLVM-IR audit gate.
- §5.7 watchdog liveness heartbeat (25 ms threshold) + `kalico_liveness_ok` hook in `src/stm32/watchdog.c`.
- §6 testing surfaces (A host unit, B FFI + C smoke, C hardware bring-up).
- §7 open questions — implementation may surface edge cases not yet pinned down.

**Hard prerequisites:**

1. **Working tree clean.** No uncommitted changes in `rust/` or `src/` from other in-flight work; this plan touches multiple existing files (workspace `Cargo.toml`, `nurbs-c-api/` rename, `src/Makefile`, `src/stm32/watchdog.c`).
   ```bash
   git status   # must be clean before Task 0
   ```
2. **No active Step-4 / Step-4.5 / Step-9 work in flight** — those touch `rust/temporal/` and won't conflict, but verify no pending edits.
3. **Workspace builds clean before starting:**
   ```bash
   cd rust && cargo test --release 2>&1 | tail -10
   ```
   Expected: all tests pass.
4. **Hardware available for Phase 8** — BTT Octopus Pro (H723) + dfu-util + USB-CDC-capable host. Phases 1–7 complete on host alone; Phase 8 requires hardware. Per the user's `feedback_printer_is_test_bench` memory, the printer is a test bench — bring-up failures don't block production, so Phase 8 can proceed without freezing other work.

---

## Phase 1: Workspace prep

### Task 0: Migrate workspace to Rust 2024 edition

**Files:**
- Modify: `rust/Cargo.toml` (workspace), `rust/nurbs/Cargo.toml`, `rust/nurbs-c-api/Cargo.toml`, `rust/gcode/Cargo.toml`, `rust/geometry/Cargo.toml`, `rust/temporal/Cargo.toml`

**Why:** Rust 2024 is required for `#[unsafe(no_mangle)]` and `unsafe extern "C" { ... }` blocks the new FFI surface uses. Migrating now as a separate prep commit avoids edition noise on top of Step 5 changes.

- [ ] **Step 1: Run `cargo fix --edition` workspace-wide**

```bash
cd rust && cargo fix --edition --allow-dirty --workspace
```

Expected: cargo edits each crate's source to satisfy edition-2024 lints. Some `unsafe extern "C"` shim code in `nurbs-c-api/src/lib.rs` and `nurbs-c-api/src/bin/gen_headers.rs` may pick up new `unsafe`-block requirements.

- [ ] **Step 2: Update `edition = "2021"` to `edition = "2024"` in every Cargo.toml**

```bash
grep -rl 'edition = "2021"' rust/ --include='Cargo.toml' | xargs sed -i.bak 's/edition = "2021"/edition = "2024"/g' && find rust -name 'Cargo.toml.bak' -delete
```

Verify:
```bash
grep -r '^edition' rust/ --include='Cargo.toml'
```

Every line must show `edition = "2024"`.

- [ ] **Step 3: Verify the workspace builds clean**

```bash
cd rust && cargo build --workspace 2>&1 | tail -5
cd rust && cargo test --workspace 2>&1 | tail -5
cd rust && cargo clippy --workspace --all-targets -- -D warnings 2>&1 | tail -10
```

All three must succeed. If clippy fires on edition-2024-specific lints (e.g., `clippy::missing_safety_doc` on `unsafe extern` blocks), fix the affected source — the migration is incomplete if clippy's not green.

- [ ] **Step 4: Commit**

```bash
git add rust/Cargo.toml rust/*/Cargo.toml rust/*/src/
git commit -m "rust: migrate workspace to Rust 2024 edition

Step-5 prep: required for #[unsafe(no_mangle)] and unsafe extern \"C\" { ... }
blocks the new kalico_runtime FFI surface uses. Mechanical migration via
cargo fix --edition; no functional changes."
```

### Task 1: Rename `nurbs-c-api` → `kalico-c-api`

**Files:**
- Rename directory: `rust/nurbs-c-api/` → `rust/kalico-c-api/`
- Modify: `rust/Cargo.toml`, `rust/kalico-c-api/Cargo.toml`, `rust/kalico-c-api/cbindgen.toml`, all source files in `rust/kalico-c-api/src/`
- Modify: any downstream `Cargo.toml` referencing the old name (currently only the workspace member list).

**Why:** The crate's role expanded to be the umbrella staticlib for both nurbs and runtime FFI; the old name lies. Spec §2.2 covers rationale.

- [ ] **Step 1: Rename the directory**

```bash
cd rust && git mv nurbs-c-api kalico-c-api
```

- [ ] **Step 2: Update `rust/Cargo.toml` workspace members**

```toml
[workspace]
members = [
  "nurbs", "kalico-c-api", "gcode", "geometry", "temporal"
]
exclude = ["gcode/fuzz"]
resolver = "2"
```

- [ ] **Step 3: Update `rust/kalico-c-api/Cargo.toml`**

Change `name = "nurbs-c-api"` to `name = "kalico-c-api"`. Description should now reflect umbrella role:

```toml
[package]
name = "kalico-c-api"
version = "0.1.0"
edition = "2024"
rust-version = "1.85"
publish = false
description = "Umbrella staticlib + cbindgen FFI surface for kalico's Rust crates (nurbs Layer 0; runtime Layer 4)."
```

- [ ] **Step 4: Regenerate the kalico_nurbs.h header to verify cbindgen still works**

```bash
cd rust && cargo run -p kalico-c-api --bin gen-headers
git status   # kalico-c-api/include/kalico_nurbs.h may show modifications
git diff rust/kalico-c-api/include/kalico_nurbs.h
```

Expected: no functional diff in the header (only path-comment changes, if any). If cbindgen complains about path resolution, debug now — Phase 4 expects this binary to work cleanly.

- [ ] **Step 5: Run the full workspace test suite**

```bash
cd rust && cargo test --workspace 2>&1 | tail -10
```

Expected: all tests pass. Header drift detection test (`headers_no_drift.rs`) must continue passing.

- [ ] **Step 6: Commit**

```bash
git add rust/Cargo.toml rust/kalico-c-api/
git commit -m "rust: rename nurbs-c-api → kalico-c-api (Step 5 prep)

The crate's role is expanding to be the umbrella staticlib for both
nurbs and runtime FFI surfaces; the old name no longer reflects scope.
Cheap to rename now (single downstream consumer); expensive once Steps
6/7/8/9/10 each link against it. Spec §2.2 covers rationale."
```

---

## Phase 2: Runtime crate types

### Task 2: Scaffold `rust/runtime/` crate

**Files:**
- Create: `rust/runtime/Cargo.toml`
- Create: `rust/runtime/src/lib.rs`
- Modify: `rust/Cargo.toml` (add to workspace members + new heapless workspace dep)

- [ ] **Step 1: Update workspace Cargo.toml**

```toml
[workspace]
members = [
  "nurbs", "kalico-c-api", "gcode", "geometry", "temporal", "runtime"
]
exclude = ["gcode/fuzz"]
resolver = "2"

[workspace.dependencies]
thiserror = "2"
clarabel = "0.11"
heapless = { version = "0.8", default-features = false }   # NEW
```

- [ ] **Step 2: Create `rust/runtime/Cargo.toml`**

```toml
[package]
name = "runtime"
version = "0.1.0"
edition = "2024"
rust-version = "1.85"
publish = false
description = "Layer 4 MCU runtime — 40 kHz ISR with stub NURBS evaluator. Step 5."

[dependencies]
nurbs = { path = "../nurbs", default-features = false }
heapless = { workspace = true }

[dev-dependencies]
proptest = "1.5"
static_assertions = "1.1"

[features]
default = ["host"]
host = ["nurbs/host"]
mcu-h7 = ["nurbs/mcu-h7"]
mcu-f4 = ["nurbs/mcu-f4"]
loom = []  # gates loom-tests under cfg(loom)

[lints]
workspace = true
```

- [ ] **Step 3: Create `rust/runtime/src/lib.rs` with the lint policy**

```rust
//! Layer 4 MCU runtime — 40 kHz hard-real-time ISR with stub NURBS evaluator.
//! See `docs/superpowers/specs/2026-04-28-layer-4-mcu-framework-stub-design.md`.

#![cfg_attr(not(feature = "host"), no_std)]
#![deny(
    clippy::panic, clippy::unwrap_used, clippy::expect_used,
    clippy::indexing_slicing,
    clippy::panic_in_result_fn,
    clippy::todo, clippy::unimplemented, clippy::unreachable,
    clippy::integer_division,
    unsafe_op_in_unsafe_fn
)]

pub mod clock;
pub mod curve_pool;
pub mod engine;
pub mod error;
pub mod kinematics;
pub mod queue;
pub mod segment;
pub mod slot;
pub mod state;
pub mod trace;

#[cfg(test)]
mod tests {
    #[test]
    fn smoke() {
        // Crate compiles; module tree intact.
    }
}
```

- [ ] **Step 4: Create empty module files** (each will be filled in by later tasks)

```bash
cd rust/runtime/src && for f in clock.rs curve_pool.rs engine.rs error.rs kinematics.rs queue.rs segment.rs slot.rs state.rs trace.rs; do
  echo "//! See docs/superpowers/specs/2026-04-28-layer-4-mcu-framework-stub-design.md" > "$f"
done
```

- [ ] **Step 5: Build and verify**

```bash
cd rust && cargo build -p runtime
cd rust && cargo test -p runtime --features host
```

Both must pass. clippy must also be green:

```bash
cd rust && cargo clippy -p runtime --all-targets -- -D warnings
```

- [ ] **Step 6: Commit**

```bash
git add rust/Cargo.toml rust/runtime/
git commit -m "runtime: scaffold rust/runtime/ crate (Step 5)

Empty no_std crate with the spec's lint policy and module tree.
Builds clean for host; mcu-h7 / mcu-f4 features wire to nurbs's
existing MCU features. Subsequent tasks fill in the module files."
```

### Task 3: `Segment` and `KinematicTag` types

**Files:**
- Modify: `rust/runtime/src/segment.rs`
- Test: `rust/runtime/src/segment.rs` (inline `#[cfg(test)] mod tests`)

**Why:** Spec §3.1 — small POD; `runtime::Segment` is distinct from `geometry::Segment` (Layer-3-to-Layer-4 conversion is Step 7 territory).

- [ ] **Step 1: Write the test**

```rust
// In rust/runtime/src/segment.rs
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn segment_size_is_under_64_bytes() {
        // Spec §4.7 / §3.1: small POD to minimize SPSC enqueue/dequeue memcpy cost.
        assert!(core::mem::size_of::<Segment>() <= 64,
            "Segment grew too large: {} bytes", core::mem::size_of::<Segment>());
    }

    #[test]
    fn segment_duration_returns_t_end_minus_t_start() {
        let seg = Segment {
            id: 1,
            curve: CurveHandle(0),
            t_start: 100,
            t_end: 350,
            kinematics: KinematicTag::CoreXyAndE,
        };
        assert_eq!(seg.duration(), 250);
    }

    #[test]
    fn segment_is_copy_clone() {
        let seg = Segment {
            id: 0, curve: CurveHandle(0), t_start: 0, t_end: 100,
            kinematics: KinematicTag::CoreXyAndE,
        };
        let _ = seg;     // copy
        let _ = seg.clone();  // clone
    }
}
```

- [ ] **Step 2: Verify the test fails**

```bash
cd rust && cargo test -p runtime --features host segment::tests 2>&1 | tail -10
```

Expected: compile error — `Segment`, `CurveHandle`, `KinematicTag` not defined.

- [ ] **Step 3: Implement the types**

```rust
//! `Segment` and `KinematicTag` — runtime per-segment record. Spec §3.1.
//!
//! Distinct from `geometry::Segment`. Step 7 MVP wires the converter at the
//! Layer-3-to-Layer-4 boundary.

/// Index into the static `CurvePool` slab (see `curve_pool` module).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CurveHandle(pub u16);

/// Selects the kinematic transform applied per tick.
///
/// Step 5 only emits `CoreXyAndE` (CoreXY for AB axes + identity for E).
/// `CartesianXyz` and `CartesianXyzAndE` are reserved slots for Step 6+ when
/// the F4x Z-only path lands.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum KinematicTag {
    CoreXyAndE = 0,
    CartesianXyzAndE = 1,
}

#[derive(Debug, Clone, Copy)]
pub struct Segment {
    /// Stable monotonic identifier set by the producer; appears in trace samples.
    pub id: u32,
    /// Index into the static `CurvePool`. Producer guarantees the slot is loaded
    /// before pushing this Segment.
    pub curve: CurveHandle,
    /// MCU clock cycles (see spec §4.1 — widened from CYCCNT inside Rust).
    pub t_start: u64,
    /// MCU clock cycles. Invariant: `t_end > t_start + MIN_SEGMENT_CYCLES`.
    pub t_end: u64,
    pub kinematics: KinematicTag,
}

impl Segment {
    #[inline]
    pub fn duration(&self) -> u64 {
        self.t_end.saturating_sub(self.t_start)
    }
}
```

- [ ] **Step 4: Verify tests pass**

```bash
cd rust && cargo test -p runtime --features host segment::tests
```

Expected: 3 tests pass.

- [ ] **Step 5: Commit**

```bash
git add rust/runtime/src/segment.rs
git commit -m "runtime/segment: Segment + CurveHandle + KinematicTag types

Spec §3.1. Small POD (≤ 64 bytes verified by test) to minimize the
heapless::spsc enqueue/dequeue memcpy cost. duration() is saturating
for defense in depth (producer rejects t_end ≤ t_start at FFI)."
```

### Task 4: `SegmentQueue` facade over `heapless::spsc::Queue`

**Files:**
- Modify: `rust/runtime/src/queue.rs`
- Test: same file, `#[cfg(test)] mod tests`

**Why:** Spec §3.1 — wrap heapless::spsc with a minimal API the Engine can rely on. Capacity 8 (effective 7).

- [ ] **Step 1: Write the test**

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use crate::segment::*;

    fn seg(id: u32, t_start: u64, t_end: u64) -> Segment {
        Segment {
            id, curve: CurveHandle(0), t_start, t_end,
            kinematics: KinematicTag::CoreXyAndE,
        }
    }

    #[test]
    fn capacity_is_seven_for_size_eight() {
        // heapless::spsc::Queue capacity is N - 1.
        let mut q = SegmentQueue::new();
        for i in 0..7 {
            assert!(q.try_push(seg(i, 0, 100)).is_ok(), "push {i} should succeed");
        }
        assert!(q.try_push(seg(7, 0, 100)).is_err(), "8th push must fail");
    }

    #[test]
    fn fifo_ordering() {
        let mut q = SegmentQueue::new();
        q.try_push(seg(10, 0, 100)).unwrap();
        q.try_push(seg(20, 0, 100)).unwrap();
        q.try_push(seg(30, 0, 100)).unwrap();
        assert_eq!(q.try_pop().unwrap().id, 10);
        assert_eq!(q.try_pop().unwrap().id, 20);
        assert_eq!(q.peek().unwrap().id, 30);
        assert_eq!(q.try_pop().unwrap().id, 30);
        assert!(q.try_pop().is_none());
    }

    #[test]
    fn peek_does_not_consume() {
        let mut q = SegmentQueue::new();
        q.try_push(seg(1, 0, 100)).unwrap();
        assert_eq!(q.peek().unwrap().id, 1);
        assert_eq!(q.peek().unwrap().id, 1);   // peek again — same value
        assert_eq!(q.try_pop().unwrap().id, 1);
        assert!(q.peek().is_none());
    }
}
```

- [ ] **Step 2: Verify the test fails**

```bash
cd rust && cargo test -p runtime --features host queue::tests 2>&1 | tail -5
```

Expected: compile error — `SegmentQueue` not defined.

- [ ] **Step 3: Implement `SegmentQueue`**

```rust
//! `SegmentQueue` — facade over `heapless::spsc::Queue<Segment, 8>`.
//! Spec §3.1 / §4.7. Capacity 8 → effective 7 (heapless's N-1 rule).
//!
//! Producer half: foreground (test harness at Step 5; comms task at Step 6+).
//! Consumer half: ISR. ARMv7-M atomic ordering ships correct via heapless.

use crate::segment::Segment;
use heapless::spsc::Queue;

/// Capacity parameter. Effective capacity = `Q_N - 1` per heapless 0.8.
pub const Q_N: usize = 8;

#[derive(Debug)]
pub struct SegmentQueue {
    inner: Queue<Segment, Q_N>,
}

impl Default for SegmentQueue {
    fn default() -> Self { Self::new() }
}

impl SegmentQueue {
    pub const fn new() -> Self {
        Self { inner: Queue::new() }
    }

    /// Producer side: enqueue a segment.
    /// Returns `Err(seg)` if queue is full.
    #[inline]
    pub fn try_push(&mut self, seg: Segment) -> Result<(), Segment> {
        self.inner.enqueue(seg)
    }

    /// Consumer side: dequeue the next segment.
    #[inline]
    pub fn try_pop(&mut self) -> Option<Segment> {
        self.inner.dequeue()
    }

    /// Consumer side: read the next segment without removing it.
    #[inline]
    pub fn peek(&mut self) -> Option<&Segment> {
        self.inner.peek()
    }

    /// Returns `true` if there are no segments enqueued.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }
}
```

Note: Step 5 doesn't yet split the queue into Producer/Consumer halves — that's a follow-up when Step 6 introduces the live producer task. The spec's "SPSC via heapless" claim still holds: heapless's `Queue` provides correct atomic ordering on ARMv7-M for single-producer / single-consumer access; we just don't enforce the split via type-level halves at Step 5. Add a comment flagging this:

```rust
// NOTE: Step 5 keeps both producer and consumer accessing `&mut SegmentQueue` directly.
// Step 6 will split into `heapless::spsc::Producer<'a>` and `Consumer<'a>` halves once
// the live comms-task producer lands; the half-split formalizes the SPSC ownership
// at the type level. Step 5's single-threaded test harness doesn't need it.
```

- [ ] **Step 4: Verify tests pass**

```bash
cd rust && cargo test -p runtime --features host queue::tests
```

Expected: 3 tests pass.

- [ ] **Step 5: Commit**

```bash
git add rust/runtime/src/queue.rs
git commit -m "runtime/queue: SegmentQueue facade over heapless::spsc

Capacity 8 → effective 7 (heapless's N-1 rule, verified by test).
Step 5 skips the Producer/Consumer half-split (Step 6 adds it when
live comms lands); single-threaded test harness uses &mut access."
```

### Task 5: `CurvePool` with no-overwrite-after-load policy

**Files:**
- Modify: `rust/runtime/src/curve_pool.rs`
- Test: same file

**Why:** Spec §3.1 — static slab. Step 5 enforces no-overwrite-after-load (refcount/epoch policy deferred to Step 6+).

- [ ] **Step 1: Write the test**

```rust
#[cfg(test)]
mod tests {
    use super::*;

    fn dummy_curve_data(n: usize) -> ([f32; 32], [f32; 32], [f32; 32]) {
        let mut cps = [0.0f32; 32];
        let mut knots = [0.0f32; 32];
        let mut weights = [0.0f32; 32];
        for i in 0..n {
            cps[i] = i as f32;
            knots[i] = i as f32 / (n as f32);
            weights[i] = 1.0;
        }
        (cps, knots, weights)
    }

    #[test]
    fn fresh_pool_handles_unloaded() {
        let pool = CurvePool::new();
        assert!(pool.resolve(CurveHandle(0)).is_none());
        assert!(pool.resolve(CurveHandle(15)).is_none());
    }

    #[test]
    fn out_of_bounds_handle_returns_none() {
        let pool = CurvePool::new();
        assert!(pool.resolve(CurveHandle(16)).is_none());
        assert!(pool.resolve(CurveHandle(u16::MAX)).is_none());
    }

    #[test]
    fn load_then_resolve_returns_curve() {
        let mut pool = CurvePool::new();
        let (cps, knots, weights) = dummy_curve_data(4);
        let result = pool.load(CurveHandle(0), &cps[..12], &knots[..8], &weights[..4], 3);
        assert!(result.is_ok());
        assert!(pool.resolve(CurveHandle(0)).is_some());
    }

    #[test]
    fn load_twice_into_same_slot_is_rejected() {
        let mut pool = CurvePool::new();
        let (cps, knots, weights) = dummy_curve_data(4);
        let first = pool.load(CurveHandle(0), &cps[..12], &knots[..8], &weights[..4], 3);
        assert!(first.is_ok());
        let second = pool.load(CurveHandle(0), &cps[..12], &knots[..8], &weights[..4], 3);
        assert_eq!(second, Err(CurvePoolError::SlotAlreadyLoaded));
    }

    #[test]
    fn invalid_curve_data_rejected() {
        let mut pool = CurvePool::new();
        let mut cps = [0.0f32; 12];
        cps[5] = f32::NAN;
        let knots = [0.0f32; 8];
        let weights = [1.0f32; 4];
        let result = pool.load(CurveHandle(0), &cps, &knots, &weights, 3);
        assert_eq!(result, Err(CurvePoolError::NonFiniteData));
    }
}
```

- [ ] **Step 2: Verify the test fails**

```bash
cd rust && cargo test -p runtime --features host curve_pool::tests 2>&1 | tail -5
```

Expected: compile error.

- [ ] **Step 3: Implement `CurvePool`**

```rust
//! `CurvePool` — static slab of NURBS curve data referenced by `CurveHandle`.
//! Spec §3.1. Step 5: no-overwrite-after-load. Step 6+ adds refcount / epoch.

use crate::segment::CurveHandle;

/// Slab capacity. Spec §7 open question 1 — revisited at Step 7 MVP.
pub const CURVE_POOL_N: usize = 16;

/// Per-curve storage capacity. Sized for degree-3 NURBS with up to 8 control
/// points in 3D — typical Step 5 fixture range. Larger curves are rejected.
pub const MAX_CONTROL_POINTS: usize = 8;
pub const MAX_DIM: usize = 3;
pub const MAX_KNOT_VECTOR_LEN: usize = MAX_CONTROL_POINTS + 4; // p + 1 + N
pub const MAX_DEGREE: u8 = 3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CurvePoolError {
    OutOfBounds,
    SlotAlreadyLoaded,
    DegreeTooHigh,
    InvalidLengths,
    NonFiniteData,
    InvalidCurve,   // catch-all for non-monotone knots, non-positive weights, etc.
}

#[derive(Debug, Clone, Copy)]
struct LoadedCurve {
    control_points: [[f32; MAX_DIM]; MAX_CONTROL_POINTS],
    weights: [f32; MAX_CONTROL_POINTS],
    knots: [f32; MAX_KNOT_VECTOR_LEN],
    n_cp: u8,
    n_knots: u8,
    degree: u8,
}

#[derive(Debug)]
pub struct CurvePool {
    slots: [Option<LoadedCurve>; CURVE_POOL_N],
}

impl Default for CurvePool {
    fn default() -> Self { Self::new() }
}

impl CurvePool {
    pub const fn new() -> Self {
        Self { slots: [const { None }; CURVE_POOL_N] }
    }

    /// Load curve data into a slot. Step-5 policy: no-overwrite-after-load.
    pub fn load(
        &mut self,
        handle: CurveHandle,
        control_points_flat: &[f32],   // length = n_cp * MAX_DIM
        knots: &[f32],
        weights: &[f32],
        degree: u8,
    ) -> Result<(), CurvePoolError> {
        let idx = handle.0 as usize;
        if idx >= CURVE_POOL_N {
            return Err(CurvePoolError::OutOfBounds);
        }
        // Slot lifetime policy: §3.1 — no overwrite at Step 5.
        if self.slots.get(idx).map(Option::is_some).unwrap_or(false) {
            return Err(CurvePoolError::SlotAlreadyLoaded);
        }
        if degree > MAX_DEGREE {
            return Err(CurvePoolError::DegreeTooHigh);
        }
        let n_cp = weights.len();
        if n_cp == 0 || n_cp > MAX_CONTROL_POINTS {
            return Err(CurvePoolError::InvalidLengths);
        }
        if control_points_flat.len() != n_cp * MAX_DIM {
            return Err(CurvePoolError::InvalidLengths);
        }
        if knots.len() > MAX_KNOT_VECTOR_LEN || knots.len() != n_cp + degree as usize + 1 {
            return Err(CurvePoolError::InvalidLengths);
        }
        if !control_points_flat.iter().chain(knots).chain(weights).all(|x| x.is_finite()) {
            return Err(CurvePoolError::NonFiniteData);
        }
        // Match the precondition set of `nurbs::VectorNurbsRef::try_new` — the
        // upstream `validate()` checks knot monotonicity and positive weights;
        // mirroring those here makes producer-side rejection definitive instead
        // of letting the ISR construct an invalid view.
        for window in knots.windows(2) {
            if window[0] > window[1] {
                return Err(CurvePoolError::InvalidCurve);  // non-monotone knots
            }
        }
        if knots.first().copied().unwrap_or(0.0) >= knots.last().copied().unwrap_or(0.0) {
            return Err(CurvePoolError::InvalidCurve);  // zero-length knot range
        }
        if !weights.iter().all(|&w| w > 0.0) {
            return Err(CurvePoolError::InvalidCurve);  // non-positive weight
        }
        // Match upstream `nurbs::validate()`'s remaining checks (rust/nurbs/src/scalar.rs):
        // (a) Knot vector must be clamped: first p+1 knots equal, last p+1 equal.
        let p = degree as usize;
        let first_knot = knots.first().copied().unwrap_or(0.0);
        let last_knot = knots.last().copied().unwrap_or(0.0);
        if !knots.iter().take(p + 1).all(|&k| k == first_knot) {
            return Err(CurvePoolError::InvalidCurve);  // start not clamped
        }
        if !knots.iter().rev().take(p + 1).all(|&k| k == last_knot) {
            return Err(CurvePoolError::InvalidCurve);  // end not clamped
        }
        // (b) Knot vector length must be ≥ 2*(p+1) so that any valid u has p+1
        // basis functions to evaluate. Already implicit in `n_cp + p + 1` ≥ 2(p+1)
        // when `n_cp ≥ p+1`, so check that:
        if n_cp < p + 1 {
            return Err(CurvePoolError::InvalidCurve);  // too few control points for degree
        }

        let mut cps = [[0.0f32; MAX_DIM]; MAX_CONTROL_POINTS];
        for i in 0..n_cp {
            cps[i][0] = *control_points_flat.get(i * MAX_DIM).unwrap_or(&0.0);
            cps[i][1] = *control_points_flat.get(i * MAX_DIM + 1).unwrap_or(&0.0);
            cps[i][2] = *control_points_flat.get(i * MAX_DIM + 2).unwrap_or(&0.0);
        }
        let mut wts = [0.0f32; MAX_CONTROL_POINTS];
        for i in 0..n_cp {
            wts[i] = *weights.get(i).unwrap_or(&1.0);
        }
        let mut knots_buf = [0.0f32; MAX_KNOT_VECTOR_LEN];
        for i in 0..knots.len() {
            knots_buf[i] = *knots.get(i).unwrap_or(&0.0);
        }
        // SAFETY: index bounded above by `CURVE_POOL_N` check.
        self.slots[idx] = Some(LoadedCurve {
            control_points: cps,
            weights: wts,
            knots: knots_buf,
            n_cp: n_cp as u8,
            n_knots: knots.len() as u8,
            degree,
        });
        Ok(())
    }

    /// Resolve a handle to a curve view suitable for `nurbs::vector_eval`.
    ///
    /// Returns None if the handle is out of bounds or the slot is unloaded.
    pub fn resolve(&self, handle: CurveHandle) -> Option<CurveView<'_>> {
        let idx = handle.0 as usize;
        let slot = self.slots.get(idx)?.as_ref()?;
        Some(CurveView {
            control_points: &slot.control_points[..slot.n_cp as usize],
            weights: &slot.weights[..slot.n_cp as usize],
            knots: &slot.knots[..slot.n_knots as usize],
            degree: slot.degree,
        })
    }
}

/// Borrowed view of a loaded curve. Adapter to `nurbs::eval` consumed in Engine.
#[derive(Debug)]
pub struct CurveView<'a> {
    pub control_points: &'a [[f32; MAX_DIM]],
    pub weights: &'a [f32],
    pub knots: &'a [f32],
    pub degree: u8,
}
```

- [ ] **Step 4: Verify tests pass**

```bash
cd rust && cargo test -p runtime --features host curve_pool::tests
```

Expected: 5 tests pass.

- [ ] **Step 5: Commit**

```bash
git add rust/runtime/src/curve_pool.rs
git commit -m "runtime/curve_pool: static slab with no-overwrite-after-load (Step 5)

Spec §3.1. Capacity 16 slots × max 8 CPs × 3D + max degree 3.
Producer-side validation rejects over-degree, length-mismatched, or
non-finite curve data. Step 6+ will add refcount/epoch lifetime
policy when slot replacement becomes relevant; for now slots are
write-once-per-runtime-lifetime."
```

### Task 6: `TraceRing` + `TraceSample` (`#[repr(C)]`) + overflow protocol

**Files:**
- Modify: `rust/runtime/src/trace.rs`
- Test: same file

**Why:** Spec §3.1 / §4.3 — drop-newest with overflow flag carried into next sample. `#[repr(C)]` for ABI stability across host-Rust / C / host-Python deserialization.

- [ ] **Step 1: Write the tests**

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use core::mem::{align_of, offset_of, size_of};

    #[test]
    fn trace_sample_layout() {
        // Spec §3.1 / §6.3 — these offsets are mirrored in the C smoke build's
        // _Static_assert. Any drift here breaks the C consumer.
        assert_eq!(size_of::<TraceSample>(), 32);
        assert_eq!(align_of::<TraceSample>(), 8);
        assert_eq!(offset_of!(TraceSample, tick), 0);
        assert_eq!(offset_of!(TraceSample, motor_a), 8);
        assert_eq!(offset_of!(TraceSample, motor_b), 12);
        assert_eq!(offset_of!(TraceSample, motor_e), 16);
        assert_eq!(offset_of!(TraceSample, segment_id), 20);
        assert_eq!(offset_of!(TraceSample, flags), 24);
    }

    fn sample(tick: u64, segment_id: u32) -> TraceSample {
        TraceSample {
            tick, motor_a: 0.0, motor_b: 0.0, motor_e: 0.0,
            segment_id, flags: 0, _pad: [0; 7],
        }
    }

    #[test]
    fn drain_pulls_in_order() {
        let mut ring = TraceRing::<16>::new();
        for i in 0..5 {
            assert!(ring.try_emit(sample(i, 0)).is_ok());
        }
        let mut out = [TraceSample::default(); 8];
        let n = ring.drain_into(&mut out);
        assert_eq!(n, 5);
        for i in 0..5 {
            assert_eq!(out[i].tick, i as u64);
        }
    }

    #[test]
    fn overflow_carries_into_next_sample() {
        let mut ring = TraceRing::<4>::new();   // effective capacity 3
        // Fill to capacity.
        for i in 0..3 {
            assert!(ring.try_emit(sample(i, 0)).is_ok());
        }
        // 4th emit fails; sets pending overflow flag.
        let r = ring.try_emit(sample(99, 0));
        assert!(r.is_err());
        assert!(ring.has_pending_overflow());

        // Drain everything to free space.
        let mut out = [TraceSample::default(); 8];
        let n = ring.drain_into(&mut out);
        assert_eq!(n, 3);

        // Pending overflow STILL set (drain doesn't clear it).
        assert!(ring.has_pending_overflow());

        // Next successful emit picks up the OVERFLOW flag.
        assert!(ring.try_emit(sample(100, 0)).is_ok());
        let n = ring.drain_into(&mut out);
        assert_eq!(n, 1);
        assert_eq!(out[0].tick, 100);
        assert_ne!(out[0].flags & TRACE_FLAG_OVERFLOW, 0,
            "OVERFLOW must propagate into the next successful sample");

        // After successful enqueue, pending bit cleared.
        assert!(!ring.has_pending_overflow());
    }
}
```

- [ ] **Step 2: Verify the tests fail**

```bash
cd rust && cargo test -p runtime --features host trace::tests 2>&1 | tail -5
```

Expected: compile error.

- [ ] **Step 3: Implement `TraceRing` + `TraceSample` + flags**

```rust
//! `TraceRing` — SPSC ring of `TraceSample` for host-side trace pulling.
//! Spec §3.1 / §4.3 / §6.3.

use core::sync::atomic::{AtomicBool, Ordering};
use heapless::spsc::Queue;

pub const TRACE_FLAG_OVERFLOW: u8 = 1 << 0;
pub const TRACE_FLAG_SEGMENT_END: u8 = 1 << 1;
pub const TRACE_FLAG_FAULT_MARKER: u8 = 1 << 2;

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TraceSample {
    pub tick: u64,         // offset 0, 8 bytes (struct alignment 8)
    pub motor_a: f32,      // offset 8
    pub motor_b: f32,      // offset 12
    pub motor_e: f32,      // offset 16
    pub segment_id: u32,   // offset 20
    pub flags: u8,         // offset 24
    pub _pad: [u8; 7],     // offsets 25..31 — explicit padding to 32-byte total
}

impl Default for TraceSample {
    fn default() -> Self {
        Self {
            tick: 0, motor_a: 0.0, motor_b: 0.0, motor_e: 0.0,
            segment_id: 0, flags: 0, _pad: [0; 7],
        }
    }
}

#[derive(Debug)]
pub struct TraceRing<const N: usize> {
    inner: Queue<TraceSample, N>,
    overflow_pending: AtomicBool,
}

impl<const N: usize> Default for TraceRing<N> {
    fn default() -> Self { Self::new() }
}

impl<const N: usize> TraceRing<N> {
    pub const fn new() -> Self {
        Self {
            inner: Queue::new(),
            overflow_pending: AtomicBool::new(false),
        }
    }

    /// Producer side: emit one sample. On full → set overflow_pending, drop sample.
    /// On success → OR the pending overflow into the sample's flags before enqueue,
    /// and clear the pending bit.
    #[inline]
    pub fn try_emit(&mut self, mut s: TraceSample) -> Result<(), TraceSample> {
        if self.overflow_pending.load(Ordering::Relaxed) {
            s.flags |= TRACE_FLAG_OVERFLOW;
        }
        match self.inner.enqueue(s) {
            Ok(()) => {
                // Successful enqueue clears the pending overflow.
                self.overflow_pending.store(false, Ordering::Relaxed);
                Ok(())
            }
            Err(rejected) => {
                self.overflow_pending.store(true, Ordering::Relaxed);
                Err(rejected)
            }
        }
    }

    /// Consumer side: drain up to `out.len()` samples in FIFO order.
    /// Returns the count drained.
    pub fn drain_into(&mut self, out: &mut [TraceSample]) -> usize {
        let mut count = 0;
        while count < out.len() {
            let Some(sample) = self.inner.dequeue() else { break };
            // Bounded by `count < out.len()`.
            if let Some(slot) = out.get_mut(count) {
                *slot = sample;
            }
            count += 1;
        }
        count
    }

    /// Foreground reads this to know whether to emit a synthetic overflow marker
    /// when drain returned empty (see spec §4.3).
    pub fn has_pending_overflow(&self) -> bool {
        self.overflow_pending.load(Ordering::Relaxed)
    }
}
```

- [ ] **Step 4: Verify tests pass**

```bash
cd rust && cargo test -p runtime --features host trace::tests
```

Expected: 3 tests pass.

- [ ] **Step 5: Commit**

```bash
git add rust/runtime/src/trace.rs
git commit -m "runtime/trace: TraceRing + TraceSample with #[repr(C)] (Step 5)

Spec §3.1 / §4.3. 32-byte sample with explicit padding; layout
verified by offset_of! tests that mirror the C smoke build's
_Static_assert checks. Drop-newest policy with overflow_pending
AtomicBool carried into the next successful enqueue."
```

### Task 7: `TickState` + slot traits with `Noop` ZST impls

**Files:**
- Modify: `rust/runtime/src/state.rs`, `rust/runtime/src/slot.rs`
- Test: `rust/runtime/src/slot.rs`

**Why:** Spec §3.1 — runtime-eval slots designed in even though Step 5 ships only `Noop`. ZST + `#[inline(always)]` ensures the optimizer fully removes the call paths.

- [ ] **Step 1: Write the tests**

In `rust/runtime/src/slot.rs`:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::TickState;
    use core::mem::size_of;

    #[test]
    fn noop_slots_are_zsts() {
        assert_eq!(size_of::<NoopPa>(), 0);
        assert_eq!(size_of::<NoopIs>(), 0);
    }

    #[test]
    fn noop_apply_does_not_mutate_state() {
        let mut pa = NoopPa;
        let mut is = NoopIs;
        let mut state = TickState {
            dt: 1.0/40_000.0,
            xyz_e: [10.0, 20.0, 5.0],
            motors: [30.0, -10.0, 5.0],
        };
        let original = state;
        pa.apply(&mut state);
        is.apply(&mut state);
        assert_eq!(state.xyz_e, original.xyz_e);
        assert_eq!(state.motors, original.motors);
    }
}
```

In `rust/runtime/src/state.rs`:

```rust
//! `TickState` — per-tick state shared with PA/IS slots. Spec §3.1.

#[derive(Debug, Clone, Copy)]
pub struct TickState {
    pub dt: f32,
    pub xyz_e: [f32; 3],
    pub motors: [f32; 3],
}
```

- [ ] **Step 2: Verify the tests fail**

```bash
cd rust && cargo test -p runtime --features host slot::tests 2>&1 | tail -5
```

Expected: compile error — `NoopPa`, `NoopIs` not defined.

- [ ] **Step 3: Implement the slot traits**

```rust
//! `PaSlot` / `IsSlot` traits — runtime-evaluation slots. Spec §3.1.
//!
//! Step 5 ships only `Noop` impls (ZST + `#[inline(always)]` → optimizer
//! removes the call). Step 9 adds `TanhPa`; Step 8 adds `SmoothShaper`.
//!
//! Slot signature is intentionally `apply(&mut self, &mut TickState)` — `&mut self`
//! lets future impls maintain per-slot state (e.g., TanhPa's previous-tick history)
//! without widening `TickState`. Spec §3.1 forward note.

use crate::state::TickState;

pub trait PaSlot {
    #[inline(always)]
    fn apply(&mut self, _state: &mut TickState) {}
}

pub trait IsSlot {
    #[inline(always)]
    fn apply(&mut self, _state: &mut TickState) {}
}

#[derive(Debug, Clone, Copy)]
pub struct NoopPa;

impl PaSlot for NoopPa {}

#[derive(Debug, Clone, Copy)]
pub struct NoopIs;

impl IsSlot for NoopIs {}
```

- [ ] **Step 4: Verify tests pass**

```bash
cd rust && cargo test -p runtime --features host slot::tests
```

Expected: 2 tests pass.

- [ ] **Step 5: Commit**

```bash
git add rust/runtime/src/state.rs rust/runtime/src/slot.rs
git commit -m "runtime/slot,state: PaSlot/IsSlot traits + NoopPa/NoopIs ZSTs

Spec §3.1. ZST verified by mem::size_of test; #[inline(always)]
default-method Noop impls let the optimizer remove the call path.
Step 9 (TanhPa) and Step 8 (SmoothShaper) provide non-Noop impls."
```

### Task 8: Kinematics module

**Files:**
- Modify: `rust/runtime/src/kinematics.rs`
- Test: same file

**Why:** Spec §3.1 / §4.2 step 6 — CoreXY for AB axes + identity for E. Round-trip stability matters for testability.

- [ ] **Step 1: Write the test**

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn corexy_with_e_round_trip() {
        // Inverse: x = (A + B) / 2, y = (A - B) / 2.
        let cases = [
            ([0.0_f32, 0.0, 0.0], [0.0_f32, 0.0, 0.0]),
            ([1.0, 0.0, 0.0], [1.0, 1.0, 0.0]),
            ([0.0, 1.0, 0.0], [1.0, -1.0, 0.0]),
            ([1.5, 2.5, 7.0], [4.0, -1.0, 7.0]),
            ([-3.0, 4.0, -2.0], [1.0, -7.0, -2.0]),
        ];
        for (xyz_e, expected_motors) in cases {
            let motors = corexy_with_e(xyz_e);
            assert_eq!(motors, expected_motors, "transform({:?})", xyz_e);

            // Round-trip via inverse.
            let xyz_e_back = [
                (motors[0] + motors[1]) / 2.0,
                (motors[0] - motors[1]) / 2.0,
                motors[2],
            ];
            assert_eq!(xyz_e_back, xyz_e, "round-trip({:?})", xyz_e);
        }
    }
}
```

- [ ] **Step 2: Verify the test fails**

```bash
cd rust && cargo test -p runtime --features host kinematics::tests 2>&1 | tail -5
```

Expected: compile error — `corexy_with_e` not defined.

- [ ] **Step 3: Implement `corexy_with_e`**

```rust
//! Kinematic transforms. Spec §3.1 / §4.2 step 6.
//!
//! Step 5 emits only `corexy_with_e`. Cartesian variants are stubs for Step 6+.

/// CoreXY for AB axes, identity for E axis.
/// Input: (X, Y, E) in workspace coordinates.
/// Output: (motor_A, motor_B, motor_E).
///
/// CoreXY belt geometry: A = X + Y, B = X − Y. Inverse: X = (A+B)/2, Y = (A−B)/2.
#[inline(always)]
pub fn corexy_with_e(xyz_e: [f32; 3]) -> [f32; 3] {
    let x = xyz_e[0];
    let y = xyz_e[1];
    let e = xyz_e[2];
    [x + y, x - y, e]
}

/// Cartesian X/Y/Z + E identity. Reserved for Step 6+ (F4x Z-only path).
#[inline(always)]
pub fn cartesian_xyz_with_e(xyz_e: [f32; 3]) -> [f32; 3] {
    xyz_e
}
```

- [ ] **Step 4: Verify the test passes**

```bash
cd rust && cargo test -p runtime --features host kinematics::tests
```

Expected: 1 test passes.

- [ ] **Step 5: Commit**

```bash
git add rust/runtime/src/kinematics.rs
git commit -m "runtime/kinematics: CoreXY+E and Cartesian+E transforms (Step 5)

Spec §3.1 / §4.2 step 6. Round-trip stability verified across 5 cases
including diagonal and negative inputs. Cartesian variant is a stub
for Step 6+ multi-MCU bring-up."
```

### Task 9: `clock.rs` — CYCCNT widening + helpers

**Files:**
- Modify: `rust/runtime/src/clock.rs`
- Test: same file

**Why:** Spec §4.1 — Rust widens 32-bit CYCCNT to u64 with Klipper's `timer_read_time()` as long-disable backstop.

- [ ] **Step 1: Write the test**

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_wrap_returns_raw_extended() {
        let mut state = WidenState::default();
        // First call after reinit at 0:
        state.reinit(0, 0);
        let now1 = state.widen(100);
        assert_eq!(now1, 100);
        let now2 = state.widen(200);
        assert_eq!(now2, 200);
    }

    #[test]
    fn wrap_increments_high() {
        let mut state = WidenState::default();
        state.reinit(0, 0);
        let _ = state.widen(0xFFFF_FF00);
        let now_post_wrap = state.widen(0x0000_0100);
        assert_eq!(now_post_wrap, (1u64 << 32) | 0x0000_0100);
    }

    #[test]
    fn one_tick_cycles_parametric() {
        // Helper is parametric over kalico_clock_freq. Sanity-check at the
        // H723 Klipper Kconfig default (520 MHz) and a hypothetical 550 MHz.
        assert_eq!(one_tick_cycles(520_000_000), 13_000);
        assert_eq!(one_tick_cycles(550_000_000), 13_750);
        assert_eq!(one_tick_cycles(480_000_000), 12_000);
    }

    #[test]
    fn min_segment_cycles_is_two_ticks() {
        // Spec §4.4 producer rejection threshold = 2 * one_tick_cycles.
        assert_eq!(min_segment_cycles(520_000_000), 26_000);
        assert_eq!(min_segment_cycles(550_000_000), 27_500);
    }
}
```

- [ ] **Step 2: Verify the tests fail**

```bash
cd rust && cargo test -p runtime --features host clock::tests 2>&1 | tail -5
```

Expected: compile error.

- [ ] **Step 3: Implement `WidenState` + helpers**

```rust
//! CYCCNT widening + cycle helpers. Spec §4.1 / §4.2 step 8.
//!
//! `WidenState` is single-producer ISR-only — wrap-handling is testable on host
//! by manually feeding raw values. The real ISR uses a `static mut` instance
//! gated by the SAFETY invariant "only the kalico ISR touches it" (§4.7).

use core::sync::atomic::{AtomicU32, Ordering};

pub const TICK_RATE_HZ: u32 = 40_000;

#[derive(Debug, Default)]
pub struct WidenState {
    pub(crate) last_low: u32,
    pub(crate) high: u64,
}

impl WidenState {
    /// Reinitialize widening state across a TIM5 disable→enable transition.
    ///
    /// Key insight (corrected after round-2 verifier review): we cannot
    /// reconstruct CYCCNT epoch from an external clock at u32 resolution alone
    /// (Klipper's `timer_read_time` is u32 too, with the same wrap period as
    /// CYCCNT on ARMCM where both come from `DWT->CYCCNT`). So the backstop
    /// shape is: foreground captures `engine.last_widened_now()` BEFORE
    /// `kalico_h7_disable_tim5()`, and passes that u64 value back at re-enable.
    /// Reinit then preserves `high` from the captured high-water mark and
    /// adjusts forward conservatively if `raw < captured_low` (one wrap
    /// detected). Long disables that wrap multiple times are inherently
    /// unrecoverable from CYCCNT alone — but `last_widened_now` carries the
    /// pre-disable high-water across the gap, so the timeline is monotonic
    /// from the foreground's perspective even if we miss exact wrap counts.
    pub fn reinit(&mut self, raw: u32, last_widened_now: u64) {
        let captured_low = last_widened_now as u32;
        self.high = last_widened_now & !0xFFFF_FFFFu64;
        if raw < captured_low {
            // At least one wrap since capture. Bump conservatively.
            self.high = self.high.wrapping_add(1u64 << 32);
        }
        self.last_low = raw;
    }

    /// Widen a raw CYCCNT u32 to u64. Caller must invoke at least once per
    /// half-wrap (~3.9 s at 550 MHz) for correctness.
    #[inline]
    pub fn widen(&mut self, raw: u32) -> u64 {
        if raw < self.last_low {
            self.high = self.high.wrapping_add(1u64 << 32);
        }
        self.last_low = raw;
        self.high | (raw as u64)
    }
}

#[inline]
pub fn one_tick_cycles(clock_freq: u32) -> u32 {
    clock_freq / TICK_RATE_HZ
}

#[inline]
pub fn min_segment_cycles(clock_freq: u32) -> u32 {
    2 * one_tick_cycles(clock_freq)
}

/// Shared liveness counter — set once by ISR, read by foreground.
///
/// Spec §4.7: u32 chosen over u64 because ARMv7-M lock-free `AtomicU64` is not
/// guaranteed; foreground uses "did the value change?" semantics so wrap (every
/// ~28 hours at 40 kHz) is benign.
pub struct TickCounter {
    inner: AtomicU32,
}

impl Default for TickCounter {
    fn default() -> Self { Self::new() }
}

impl TickCounter {
    pub const fn new() -> Self {
        Self { inner: AtomicU32::new(0) }
    }

    #[inline]
    pub fn increment(&self) {
        self.inner.fetch_add(1, Ordering::Relaxed);
    }

    #[inline]
    pub fn snapshot(&self) -> u32 {
        self.inner.load(Ordering::Relaxed)
    }
}
```

- [ ] **Step 4: Verify tests pass**

```bash
cd rust && cargo test -p runtime --features host clock::tests
```

Expected: 4 tests pass.

- [ ] **Step 5: Commit**

```bash
git add rust/runtime/src/clock.rs
git commit -m "runtime/clock: WidenState + cycle helpers + TickCounter

Spec §4.1 / §4.7. WidenState handles the raw u32 → u64 widening with
host-testable wrap behavior; reinit() takes a klipper_now_cycles
backstop for long-disable recovery. TickCounter is AtomicU32 (M7
lock-free) with wrap-tolerant 'value changed' semantics."
```

### Task 10: `error.rs` — `RuntimeError` enum + `i32` mapping

**Files:**
- Modify: `rust/runtime/src/error.rs`

**Why:** Spec §5.1 / §5.2 — `RuntimeError` stays internal; FFI maps to `i32`.

- [ ] **Step 1: Write the impl** (no separate test step — the i32 mapping is a `From` impl, exercised in Task 13's FFI tests)

```rust
//! `RuntimeError` — internal Rust error enum. Spec §5.1.
//!
//! FFI surface maps to `i32` codes per spec §5.2; never crosses C as a Rust
//! type (Rust enum layouts are not stable across compilations).

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RuntimeError {
    NotInit,
    NullPtr,
    QueueFull,
    InvalidCurve,
    InvalidHandle,
    InvalidDuration,
    InvalidKinematics,
    FaultLatched,
    BoundaryLoopExhausted,
    NaNOrInfFromEval,
    Internal,
}

// FFI return codes — must match the C-side #define table in spec §5.2.
pub const KALICO_OK: i32 = 0;
pub const KALICO_ERR_QUEUE_FULL: i32 = -1;
pub const KALICO_ERR_INVALID_CURVE: i32 = -2;
pub const KALICO_ERR_INVALID_HANDLE: i32 = -3;
pub const KALICO_ERR_INVALID_DURATION: i32 = -4;
pub const KALICO_ERR_INVALID_KINEMATICS: i32 = -5;
pub const KALICO_ERR_NULL_PTR: i32 = -6;
pub const KALICO_ERR_NOT_INIT: i32 = -7;
pub const KALICO_ERR_FAULT_LATCHED: i32 = -8;
pub const KALICO_ERR_INTERNAL: i32 = -9;

impl From<RuntimeError> for i32 {
    fn from(e: RuntimeError) -> i32 {
        match e {
            RuntimeError::NotInit => KALICO_ERR_NOT_INIT,
            RuntimeError::NullPtr => KALICO_ERR_NULL_PTR,
            RuntimeError::QueueFull => KALICO_ERR_QUEUE_FULL,
            RuntimeError::InvalidCurve => KALICO_ERR_INVALID_CURVE,
            RuntimeError::InvalidHandle => KALICO_ERR_INVALID_HANDLE,
            RuntimeError::InvalidDuration => KALICO_ERR_INVALID_DURATION,
            RuntimeError::InvalidKinematics => KALICO_ERR_INVALID_KINEMATICS,
            RuntimeError::FaultLatched => KALICO_ERR_FAULT_LATCHED,
            RuntimeError::BoundaryLoopExhausted
            | RuntimeError::NaNOrInfFromEval
            | RuntimeError::Internal => KALICO_ERR_INTERNAL,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_variant_maps_to_a_distinct_or_grouped_code() {
        let mappings = [
            (RuntimeError::NotInit, KALICO_ERR_NOT_INIT),
            (RuntimeError::NullPtr, KALICO_ERR_NULL_PTR),
            (RuntimeError::QueueFull, KALICO_ERR_QUEUE_FULL),
            (RuntimeError::InvalidCurve, KALICO_ERR_INVALID_CURVE),
            (RuntimeError::InvalidHandle, KALICO_ERR_INVALID_HANDLE),
            (RuntimeError::InvalidDuration, KALICO_ERR_INVALID_DURATION),
            (RuntimeError::InvalidKinematics, KALICO_ERR_INVALID_KINEMATICS),
            (RuntimeError::FaultLatched, KALICO_ERR_FAULT_LATCHED),
            (RuntimeError::BoundaryLoopExhausted, KALICO_ERR_INTERNAL),
            (RuntimeError::NaNOrInfFromEval, KALICO_ERR_INTERNAL),
            (RuntimeError::Internal, KALICO_ERR_INTERNAL),
        ];
        for (err, expected_code) in mappings {
            assert_eq!(i32::from(err), expected_code, "{err:?}");
        }
    }
}
```

- [ ] **Step 2: Verify and commit**

```bash
cd rust && cargo test -p runtime --features host error::tests
git add rust/runtime/src/error.rs
git commit -m "runtime/error: RuntimeError enum + i32 FFI mapping (Step 5)

Spec §5.1 / §5.2. Internal-only enum; FFI maps to documented codes.
Three internal errors (boundary-loop, NaN/Inf, generic) collapse to
a single KALICO_ERR_INTERNAL since the host can't distinguish them
without reading last_error() — those remain distinguishable internally
for FAULT diagnostics."
```

---

## Phase 3: Engine

### Task 11: `Engine` state machine — `tick()`, sub-tick boundary loop, status

**Files:**
- Modify: `rust/runtime/src/engine.rs`
- Test: `rust/runtime/tests/engine_tick.rs` (integration test for cleaner setup)

**Why:** Spec §3.1 / §4.2 — load-bearing state machine. Order: queue check → segment activation → sub-tick boundary loop → curve eval → NaN check → kinematics → slot pipeline → trace emit → tick counter → status. This is the core of Step 5.

- [ ] **Step 1: Write the integration test scaffolding**

In `rust/runtime/tests/engine_tick.rs`:

```rust
//! Integration tests for `Engine::tick`. Spec §4.2.

use runtime::clock::{one_tick_cycles, WidenState};
use runtime::curve_pool::{CurvePool, CurveHandle};
use runtime::engine::{Engine, RuntimeStatus};
use runtime::queue::SegmentQueue;
use runtime::segment::{KinematicTag, Segment};
use runtime::slot::{NoopIs, NoopPa};
use runtime::trace::{TraceRing, TraceSample, TRACE_FLAG_SEGMENT_END};

// Default H723 Klipper Kconfig clock is 520 MHz (src/stm32/Kconfig). Keeping
// tests parametric here so a future bump to 550 MHz (or different alternate
// kconfig) doesn't invalidate the fixture math.
const CLOCK_FREQ: u32 = 520_000_000;

mod fixtures;  // see Task 17a — shared step5_segments.json parser

/// Load fixture-by-name into the curve pool slot. Single source of truth for
/// "which curves the Step-5 tests use" — mirrored by Surface C's host script.
fn load_fixture(pool: &mut CurvePool, handle: u16, name: &str) {
    let set = fixtures::load();
    let f = set.fixtures.iter().find(|f| f.name == name)
        .unwrap_or_else(|| panic!("fixture {name} missing from step5_segments.json"));
    let cps_flat: Vec<f32> = f.control_points.iter().flat_map(|p| p.iter().copied()).collect();
    pool.load(CurveHandle(handle), &cps_flat, &f.knots, &f.weights, f.degree).unwrap();
}

#[test]
fn tick_on_empty_queue_returns_idle() {
    let mut engine = Engine::<NoopPa, NoopIs>::new(CLOCK_FREQ);
    let r = engine.tick(0, &mut SegmentQueue::new(), &CurvePool::new(),
                       &mut TraceRing::<1024>::new());
    assert!(r.is_ok());
    assert_eq!(engine.status(), RuntimeStatus::Idle);
}

#[test]
fn tick_processes_one_segment_to_completion() {
    let mut engine = Engine::<NoopPa, NoopIs>::new(CLOCK_FREQ);
    let mut queue = SegmentQueue::new();
    let mut pool = CurvePool::new();
    let mut trace = TraceRing::<1024>::new();

    load_fixture(&mut pool, 0, "straight_line_x");

    let tick_cycles = one_tick_cycles(CLOCK_FREQ) as u64;
    let n_ticks = 4u64;
    queue.try_push(Segment {
        id: 1,
        curve: CurveHandle(0),
        t_start: 0,
        t_end: n_ticks * tick_cycles,
        kinematics: KinematicTag::CoreXyAndE,
    }).unwrap();

    // Tick repeatedly through the segment.
    for tick_idx in 0..(n_ticks + 1) {
        let now = tick_idx * tick_cycles;
        let _ = engine.tick(now, &mut queue, &pool, &mut trace);
    }

    // Drain trace and verify samples emitted along the line.
    let mut out = [TraceSample::default(); 16];
    let n = trace.drain_into(&mut out);
    assert!(n >= 4, "expected at least 4 samples along the line, got {n}");

    // Last sample at u≈1 → motors at endpoint, segment-end flag set.
    let last = &out[n - 1];
    assert_eq!(last.flags & TRACE_FLAG_SEGMENT_END, TRACE_FLAG_SEGMENT_END);
}

#[test]
fn sub_tick_boundary_carries_partial_into_next_segment() {
    let mut engine = Engine::<NoopPa, NoopIs>::new(CLOCK_FREQ);
    let mut queue = SegmentQueue::new();
    let mut pool = CurvePool::new();
    let mut trace = TraceRing::<1024>::new();

    let tc = one_tick_cycles(CLOCK_FREQ) as u64;
    // Two distinct fixtures back-to-back — exercise sub-tick boundary carry.
    // straight_line_x ends at (10,0,0); rational_quadratic_arc starts at
    // (10,0,0) so the boundary is geometrically continuous and motor_a
    // increases monotonically across the seam.
    load_fixture(&mut pool, 0, "straight_line_x");
    load_fixture(&mut pool, 1, "rational_quadratic_arc");

    queue.try_push(Segment {
        id: 1, curve: CurveHandle(0), t_start: 0, t_end: tc * 3 / 2,
        kinematics: KinematicTag::CoreXyAndE,
    }).unwrap();
    queue.try_push(Segment {
        id: 2, curve: CurveHandle(1), t_start: tc * 3 / 2, t_end: 3 * tc,
        kinematics: KinematicTag::CoreXyAndE,
    }).unwrap();

    // Tick at t = 0, t = 1, t = 2 — second tick straddles the boundary.
    for tick_idx in 0..=3u64 {
        let _ = engine.tick(tick_idx * tc, &mut queue, &pool, &mut trace);
    }

    let mut out = [TraceSample::default(); 16];
    let n = trace.drain_into(&mut out);

    // Boundary correctness check: the LAST sample of segment 1 and the FIRST
    // sample of segment 2 must agree on (motor_a, motor_b, motor_e) to within
    // the sub-tick boundary tolerance. straight_line_x ends at (10, 0, 0);
    // rational_quadratic_arc starts at (10, 0, 0) — both yield motor = (10, 10, 0)
    // at the seam. Per-sample monotonicity over the whole trace is NOT asserted
    // (the arc's motor_a rises to ~14.14 mid-arc and falls back to 10 at u=1
    // because motor_a = X+Y and the arc's path through (10,10,0) increases X+Y).
    let mut last_seg1: Option<&TraceSample> = None;
    let mut first_seg2: Option<&TraceSample> = None;
    for s in out.iter().take(n) {
        if s.segment_id == 1 { last_seg1 = Some(s); }
        if s.segment_id == 2 && first_seg2.is_none() { first_seg2 = Some(s); }
    }
    let last1 = last_seg1.expect("expected at least one sample from segment 1");
    let first2 = first_seg2.expect("expected at least one sample from segment 2");

    // Seam tolerance: 25 µm × 2 (start + end of tick) = 50 µm = 0.05 mm.
    const SEAM_TOL_MM: f32 = 0.05;
    assert!((first2.motor_a - last1.motor_a).abs() < SEAM_TOL_MM,
        "motor_a discontinuous at seam: {} → {}", last1.motor_a, first2.motor_a);
    assert!((first2.motor_b - last1.motor_b).abs() < SEAM_TOL_MM,
        "motor_b discontinuous at seam: {} → {}", last1.motor_b, first2.motor_b);
    assert!((first2.motor_e - last1.motor_e).abs() < SEAM_TOL_MM,
        "motor_e discontinuous at seam: {} → {}", last1.motor_e, first2.motor_e);
}
```

- [ ] **Step 2: Verify the tests fail**

```bash
cd rust && cargo test -p runtime --features host --test engine_tick 2>&1 | tail -5
```

Expected: compile error — `Engine`, `RuntimeStatus` not defined.

- [ ] **Step 3: Implement `Engine`**

```rust
//! `Engine` — per-axis evaluator + ISR state machine. Spec §3.1 / §4.2.

use core::sync::atomic::{AtomicI32, AtomicU8, Ordering};

use crate::clock::{one_tick_cycles, TickCounter, WidenState};
use crate::curve_pool::{CurvePool, CurveView};
use crate::error::RuntimeError;
use crate::kinematics::{cartesian_xyz_with_e, corexy_with_e};
use crate::queue::SegmentQueue;
use crate::segment::{KinematicTag, Segment};
use crate::slot::{IsSlot, PaSlot};
use crate::state::TickState;
use crate::trace::{
    TraceRing, TraceSample, TRACE_FLAG_FAULT_MARKER, TRACE_FLAG_SEGMENT_END,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum RuntimeStatus {
    Idle = 0,
    Running = 1,
    Drained = 2,
    Fault = 3,
}

impl RuntimeStatus {
    #[inline]
    fn from_u8(v: u8) -> Self {
        match v {
            0 => Self::Idle,
            1 => Self::Running,
            2 => Self::Drained,
            _ => Self::Fault,
        }
    }
}

pub struct Engine<P: PaSlot, I: IsSlot> {
    current: Option<Segment>,
    last_motors: [f32; 3],     // last-known-good motor positions (used in FAULT marker)
    pa_slot: P,
    is_slot: I,
    one_tick_cycles_value: u64,
    pub(crate) widen_state: WidenState,
    pub(crate) status: AtomicU8,
    pub(crate) last_error: AtomicI32,
    pub(crate) tick_counter: TickCounter,
}

impl<P: PaSlot + Default, I: IsSlot + Default> Engine<P, I> {
    pub fn new(clock_freq: u32) -> Self {
        Self {
            current: None,
            last_motors: [0.0; 3],
            pa_slot: P::default(),
            is_slot: I::default(),
            one_tick_cycles_value: one_tick_cycles(clock_freq) as u64,
            widen_state: WidenState::default(),
            status: AtomicU8::new(RuntimeStatus::Idle as u8),
            last_error: AtomicI32::new(0),
            tick_counter: TickCounter::new(),
        }
    }
}

// Engine::Default impl for tests where slot types implement Default.
impl<P: PaSlot + Default, I: IsSlot + Default> Default for Engine<P, I> {
    fn default() -> Self {
        // H723 Klipper Kconfig default is 520 MHz (src/stm32/Kconfig). Tests using
        // Default get this; tests requiring a specific value should call ::new() directly.
        Self::new(520_000_000)
    }
}

impl<P: PaSlot, I: IsSlot> Engine<P, I> {
    pub fn status(&self) -> RuntimeStatus {
        RuntimeStatus::from_u8(self.status.load(Ordering::Acquire))
    }

    pub fn last_error(&self) -> i32 {
        self.last_error.load(Ordering::Acquire)
    }

    pub fn tick_counter(&self) -> u32 {
        self.tick_counter.snapshot()
    }

    /// Foreground-callable: read the most recent widened `now: u64` so the
    /// producer-protocol can preserve epoch across a TIM5 disable→enable cycle.
    /// SAFETY: ISR is the sole writer of WidenState; foreground reads it only
    /// while ISR is disabled (between `kalico_h7_disable_tim5` and re-enable).
    pub fn last_widened_now(&self) -> u64 {
        // Reconstruct from the ISR-private static. In the real implementation,
        // this returns `widen_state.high | (widen_state.last_low as u64)` —
        // accessed under the spec §4.7 SAFETY invariant that ISR is paused.
        self.widen_state.high | (self.widen_state.last_low as u64)
    }

    /// Latch FAULT and emit one fault marker sample (last-known-good motors,
    /// not zero, so host plots show the fault in context). ISR self-disables
    /// the timer in the C wrapper after this returns.
    fn latch_fault(&mut self, code: RuntimeError, now: u64,
                   trace: &mut TraceRing<{ 1024 }>) {
        self.last_error.store(i32::from(code), Ordering::Release);
        self.status.store(RuntimeStatus::Fault as u8, Ordering::Release);
        let segment_id = self.current.as_ref().map(|s| s.id).unwrap_or(0);
        let _ = trace.try_emit(TraceSample {
            tick: now,
            motor_a: self.last_motors[0],
            motor_b: self.last_motors[1],
            motor_e: self.last_motors[2],
            segment_id,
            flags: TRACE_FLAG_FAULT_MARKER,
            _pad: [0; 7],
        });
    }

    /// Single 40 kHz tick. Spec §4.2 step ordering — must remain stable.
    ///
    /// `now` is the widened u64 cycle count — caller (FFI shim or test)
    /// is responsible for widening via `WidenState`.
    pub fn tick(
        &mut self,
        now: u64,
        queue: &mut SegmentQueue,
        pool: &CurvePool,
        trace: &mut TraceRing<{ 1024 }>,
    ) -> Result<(), RuntimeError> {
        if self.status() == RuntimeStatus::Fault {
            return Err(RuntimeError::FaultLatched);
        }

        // Step 1 + 2: queue + idle check, segment activation. See spec §4.2.
        if self.current.is_none() {
            self.current = queue.try_pop();
        }

        // Step 1's idle path with §4.4 ISR-disable protocol.
        // (Caller observes status == Idle and clears CR1.CEN.)
        let Some(mut current) = self.current.take() else {
            self.status.store(RuntimeStatus::Idle as u8, Ordering::Release);
            // Re-check queue with Acquire — race against producer's enqueue.
            if !queue.is_empty() {
                self.current = queue.try_pop();
                if let Some(seg) = self.current {
                    self.status.store(RuntimeStatus::Running as u8, Ordering::Release);
                    // Fall through with the freshly dequeued segment.
                    return self.tick_with_current(seg, now, queue, pool, trace);
                }
            }
            return Ok(());
        };

        self.tick_with_current(current, now, queue, pool, trace)
    }

    fn tick_with_current(
        &mut self,
        mut current: Segment,
        now: u64,
        queue: &mut SegmentQueue,
        pool: &CurvePool,
        trace: &mut TraceRing<{ 1024 }>,
    ) -> Result<(), RuntimeError> {
        // Step 3: sub-tick boundary loop. Spec §4.2 step 3 — bounded by queue depth.
        let mut iters = 0u32;
        const MAX_ITERS: u32 = 8;  // matches Q_N (queue capacity)
        let mut t_segment = now.saturating_sub(current.t_start);
        while t_segment >= current.duration() {
            iters += 1;
            if iters > MAX_ITERS {
                self.current = Some(current);
                self.latch_fault(RuntimeError::BoundaryLoopExhausted, now, trace);
                return Err(RuntimeError::BoundaryLoopExhausted);
            }
            let delta_t = t_segment - current.duration();
            // Drop current; advance to next.
            let Some(next) = queue.try_pop() else {
                // No next segment — drained. Set status; return.
                self.current = None;
                self.status.store(RuntimeStatus::Drained as u8, Ordering::Release);
                return Ok(());
            };
            current = next;
            current.t_start = now.saturating_sub(delta_t);
            t_segment = delta_t;
        }
        self.current = Some(current);

        // Step 4: curve evaluation. Spec invariant: segments are time-parameterized.
        let curve_view = match pool.resolve(current.curve) {
            Some(v) => v,
            None => {
                self.latch_fault(RuntimeError::InvalidHandle, now, trace);
                return Err(RuntimeError::InvalidHandle);
            }
        };
        let duration = current.duration().max(1) as f32;  // saturating_sub avoids 0
        let u = (t_segment as f32 / duration).clamp(0.0, 1.0);
        let xyz_e = match nurbs_eval_3d(&curve_view, u) {
            Ok(p) => p,
            Err(_) => {
                self.latch_fault(RuntimeError::InvalidCurve, now, trace);
                return Err(RuntimeError::InvalidCurve);
            }
        };

        // Step 5: NaN/Inf check. Spec §5.4 — necessary even with producer-side
        // validation (NaN can arise from finite inputs).
        if !xyz_e.iter().all(|x: &f32| x.is_finite()) {
            self.latch_fault(RuntimeError::NaNOrInfFromEval, now, trace);
            return Err(RuntimeError::NaNOrInfFromEval);
        }

        // Step 6: kinematic transform. Pipeline order: kinematics BEFORE PA/IS.
        let motors = match current.kinematics {
            KinematicTag::CoreXyAndE => corexy_with_e(xyz_e),
            KinematicTag::CartesianXyzAndE => cartesian_xyz_with_e(xyz_e),
        };

        // Step 7: slot pipeline. Noop ZSTs at Step 5.
        let dt = 1.0 / (40_000.0_f32);
        let mut state = TickState { dt, xyz_e, motors };
        self.pa_slot.apply(&mut state);
        self.is_slot.apply(&mut state);

        // Step 8: trace emit.
        let next_t_segment = t_segment.saturating_add(self.one_tick_cycles_value);
        let segment_end_flag = if next_t_segment >= current.duration() {
            TRACE_FLAG_SEGMENT_END
        } else {
            0
        };
        let _ = trace.try_emit(TraceSample {
            tick: now,
            motor_a: state.motors[0],
            motor_b: state.motors[1],
            motor_e: state.motors[2],
            segment_id: current.id,
            flags: segment_end_flag,
            _pad: [0; 7],
        });
        self.last_motors = state.motors;

        // Step 9: tick counter heartbeat.
        self.tick_counter.increment();

        // Step 10: status update.
        self.status.store(RuntimeStatus::Running as u8, Ordering::Release);
        Ok(())
    }
}

/// Wrapper around `nurbs::eval::vector_eval` for f32 3D rational NURBS.
///
/// Uses `nurbs::VectorNurbsRef` (the borrowed view type) per the actual
/// Layer-0 API at `rust/nurbs/src/vector.rs` (verified during plan review).
fn nurbs_eval_3d(curve: &CurveView<'_>, u: f32) -> Result<[f32; 3], ()> {
    use nurbs::VectorNurbsRef;

    // Actual API: try_new(degree: u8, knots: &[T], control_points: &[[T; N]],
    //                     weights: Option<&[T]>) -> Result<Self, ConstructError>.
    // Returns owning struct over the borrowed slices.
    let view = VectorNurbsRef::<f32, 3>::try_new(
        curve.degree,
        curve.knots,
        curve.control_points,
        Some(curve.weights),
    ).map_err(|_| ())?;
    // vector_eval returns [T; N] directly — no Result wrapper.
    Ok(nurbs::eval::vector_eval(&view, u))
}
```

The API shape: `try_new(degree, knots, control_points, weights: Option<&[T]>)` — order matters; weights is `Option<&[T]>` for non-rational curves (`None` means uniform weights = 1.0). Step 5 always passes `Some(curve.weights)` since the slab always stores weights. Verified against `rust/nurbs/src/vector.rs:101` during plan review (round 1).

- [ ] **Step 4: Verify the integration tests pass**

```bash
cd rust && cargo test -p runtime --features host --test engine_tick 2>&1 | tail -10
```

Expected: 3 tests pass.

- [ ] **Step 5: Run clippy and fix any new lints**

```bash
cd rust && cargo clippy -p runtime --all-targets --features host -- -D warnings
```

Expected: clean. The deny-lint policy from `lib.rs` is strict; if anything fires, fix the offending code by switching to `.get()`/`.checked_*()`/`Result`-returning APIs.

- [ ] **Step 6: Commit**

```bash
git add rust/runtime/src/engine.rs rust/runtime/tests/engine_tick.rs
git commit -m "runtime/engine: Engine state machine with sub-tick boundary loop

Spec §4.2. Step ordering: queue check → segment activation → sub-tick
boundary loop (bounded by Q_N=8) → curve eval → NaN/Inf check →
kinematics → slot pipeline (PA, IS) → trace emit (with SEGMENT_END
flag detection) → tick counter → status. FAULT path latches code,
emits last-known-good marker, and self-disables (via caller). Three
integration tests cover empty queue, single-segment-to-completion,
and sub-tick boundary carry into the next segment."
```

---

## Phase 4: FFI surface

### Task 12: Update `kalico-c-api/Cargo.toml` to depend on `runtime`

**Files:**
- Modify: `rust/kalico-c-api/Cargo.toml`

- [ ] **Step 1: Add `runtime` dep + new feature gates for headers**

```toml
[dependencies]
nurbs = { path = "../nurbs", default-features = false }
runtime = { path = "../runtime", default-features = false }   # NEW
cbindgen = { version = "0.27", optional = true }

[features]
default = ["host", "header-nurbs", "header-runtime"]
mcu-h7 = ["nurbs/mcu-h7", "runtime/mcu-h7"]
mcu-f4 = ["nurbs/mcu-f4", "runtime/mcu-f4"]
host = ["nurbs/host", "runtime/host", "dep:cbindgen"]
header-nurbs = []                                              # NEW — gates kalico_nurbs_* FFI
header-runtime = []                                            # NEW — gates kalico_runtime_* FFI
```

- [ ] **Step 2: Verify build**

```bash
cd rust && cargo build -p kalico-c-api --no-default-features --features host
cd rust && cargo build -p kalico-c-api --no-default-features --features mcu-h7
```

Both must succeed. If `mcu-h7` build fails because of std/alloc usage in `runtime/`, the runtime crate isn't truly no_std — go back to Phase 2 and fix.

- [ ] **Step 3: Commit**

```bash
git add rust/kalico-c-api/Cargo.toml
git commit -m "kalico-c-api: depend on runtime; add header-* feature gates

The header-nurbs and header-runtime gates control which FFI module
compiles, so cbindgen runs twice (gen-headers binary in Task 15)
emit kalico_nurbs.h and kalico_runtime.h with the right symbols."
```

### Task 13: `runtime_ffi.rs` — `kalico_runtime_*` FFI entrypoints

**Files:**
- Create: `rust/kalico-c-api/src/runtime_ffi.rs`
- Modify: `rust/kalico-c-api/src/lib.rs` (relocate nurbs FFI, add runtime FFI module)

**Why:** Spec §3.2 — opaque `*mut KalicoRuntime` handle, init-once cell, push/tick/drain/status FFI, all returning `i32` codes.

- [ ] **Step 1: Relocate existing nurbs FFI to its own module**

Create `rust/kalico-c-api/src/nurbs_ffi.rs` (move the existing `kalico_nurbs_*` extern fns from the current `lib.rs`):

```rust
//! Kalico nurbs C-FFI surface. cfg-gated by `header-nurbs`.

#![allow(unsafe_code)]

#[cfg(feature = "header-nurbs")]
mod exports {
    use nurbs::{ArcLengthTableRef, ScalarNurbsRef, VectorNurbsRef};

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_nurbs_eval_f32(
        curve: *const ScalarNurbsRef<'_, f32>,
        u: f32,
    ) -> f32 {
        let curve_ref = unsafe { &*curve };
        nurbs::eval::eval(curve_ref, u)
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_nurbs_vector_eval_3_f32(
        curve: *const VectorNurbsRef<'_, f32, 3>,
        u: f32,
        out: *mut f32,
    ) {
        let curve_ref = unsafe { &*curve };
        let result = nurbs::eval::vector_eval(curve_ref, u);
        let out_slice = unsafe { core::slice::from_raw_parts_mut(out, 3) };
        out_slice.copy_from_slice(&result);
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_nurbs_param_from_arc_length_f32(
        table: *const ArcLengthTableRef<'_, f32>,
        s: f32,
    ) -> f32 {
        let table_ref = unsafe { &*table };
        nurbs::arc_length::param_from_arc_length(table_ref, s)
    }
}
```

(Adjust the `unsafe extern "C"` and `#[unsafe(no_mangle)]` to whatever the Rust 2024 cargo-fix produced in Task 0.)

- [ ] **Step 2: Create `runtime_ffi.rs`**

```rust
//! Kalico runtime C-FFI surface. Spec §3.2 / §4.4 / §5.2 / §5.6.
//!
//! cfg-gated by `header-runtime`.

#![allow(unsafe_code)]

#[cfg(feature = "header-runtime")]
mod exports {
    use core::sync::atomic::{AtomicU8, Ordering};
    use core::cell::UnsafeCell;
    use core::mem::MaybeUninit;

    use runtime::clock::WidenState;
    use runtime::curve_pool::{CurveHandle, CurvePool, MAX_DIM};
    use runtime::engine::{Engine, RuntimeStatus};
    use runtime::error::*;
    use runtime::queue::SegmentQueue;
    use runtime::segment::{KinematicTag, Segment};
    use runtime::slot::{NoopIs, NoopPa};
    use runtime::trace::{TraceRing, TraceSample};

    // Compile-time choice of slot impls. Spec §3.1.
    #[cfg(feature = "pa-tanh")]
    type Pa = unimplemented!("Step 9 stub — provide TanhPa here");
    #[cfg(not(feature = "pa-tanh"))]
    type Pa = NoopPa;
    #[cfg(feature = "input-shaper")]
    type Is = unimplemented!("Step 8 stub — provide SmoothShaper here");
    #[cfg(not(feature = "input-shaper"))]
    type Is = NoopIs;

    // The opaque type C sees — never dereferenced on the C side.
    #[repr(C)]
    pub struct KalicoRuntime {
        _private: [u8; 0],
    }

    // Concrete singleton storage. Spec §3.2 init-once protocol.
    pub(super) struct RuntimeCell(UnsafeCell<MaybeUninit<RuntimeContext>>);
    unsafe impl Sync for RuntimeCell {}

    pub(super) struct RuntimeContext {
        pub(super) engine: Engine<Pa, Is>,
        pub(super) queue: SegmentQueue,
        pub(super) pool: CurvePool,
        pub(super) trace: TraceRing<1024>,
    }

    pub(super) static RT_CELL: RuntimeCell =
        RuntimeCell(UnsafeCell::new(MaybeUninit::uninit()));

    pub(super) const INIT_UNINIT: u8 = 0;
    pub(super) const INIT_INITING: u8 = 1;
    pub(super) const INIT_READY: u8 = 2;

    pub(super) static INIT_STATE: AtomicU8 = AtomicU8::new(INIT_UNINIT);

    // C-side `kalico_clock_freq` constant — defined in src/runtime_tick.c.
    unsafe extern "C" {
        pub(super) static kalico_clock_freq: u32;
    }

    /// Init-once. Spec §3.2.
    /// Returns valid handle on first successful call; null otherwise.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_init() -> *mut KalicoRuntime {
        match INIT_STATE.compare_exchange(
            INIT_UNINIT, INIT_INITING,
            Ordering::AcqRel, Ordering::Acquire,
        ) {
            Ok(_) => {
                let clock_freq = unsafe { kalico_clock_freq };
                // SAFETY: we hold the INIT_INITING token; no other context
                // has access to RT_CELL until we publish READY.
                unsafe {
                    (*RT_CELL.0.get()).write(RuntimeContext {
                        engine: Engine::<Pa, Is>::new(clock_freq),
                        queue: SegmentQueue::new(),
                        pool: CurvePool::new(),
                        trace: TraceRing::<1024>::new(),
                    });
                }
                INIT_STATE.store(INIT_READY, Ordering::Release);
                RT_CELL.0.get() as *mut KalicoRuntime
            }
            Err(_) => core::ptr::null_mut(),  // Already INITING or READY
        }
    }

    /// Push a segment. Producer protocol per spec §4.4.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_push_segment(
        rt: *mut KalicoRuntime,
        id: u32,
        curve_handle: u16,
        t_start: u64,
        t_end: u64,
        kinematics: u8,
    ) -> i32 {
        if rt.is_null() { return KALICO_ERR_NULL_PTR; }
        if INIT_STATE.load(Ordering::Acquire) != INIT_READY {
            return KALICO_ERR_NOT_INIT;
        }
        let ctx = unsafe { &mut *(rt as *mut RuntimeContext) };
        if ctx.engine.status() == RuntimeStatus::Fault {
            return KALICO_ERR_FAULT_LATCHED;
        }
        if t_end <= t_start { return KALICO_ERR_INVALID_DURATION; }
        // MIN_SEGMENT_CYCLES check.
        let min_seg_cycles = runtime::clock::min_segment_cycles(
            unsafe { kalico_clock_freq },
        ) as u64;
        if t_end - t_start < min_seg_cycles {
            return KALICO_ERR_INVALID_DURATION;
        }
        let kin = match kinematics {
            0 => KinematicTag::CoreXyAndE,
            1 => KinematicTag::CartesianXyzAndE,
            _ => return KALICO_ERR_INVALID_KINEMATICS,
        };
        let seg = Segment {
            id, curve: CurveHandle(curve_handle),
            t_start, t_end, kinematics: kin,
        };
        if ctx.queue.try_push(seg).is_err() {
            return KALICO_ERR_QUEUE_FULL;
        }
        // §4.4 producer-protocol: re-enable TIM5 if observed status was IDLE/DRAINED.
        match ctx.engine.status() {
            RuntimeStatus::Idle | RuntimeStatus::Drained => {
                // Reinit CYCCNT widening before re-enabling TIM5. Pass the
                // pre-disable widened high-water (preserved across the TIM5
                // disable cycle) so the epoch survives a long gap. ISR was
                // disabled in `kalico_h7_disable_tim5()` and is still off here
                // — single-thread access to widen_state is safe.
                let raw = unsafe { kalico_h7_read_cyccnt() };
                let last_widened_now = ctx.engine.last_widened_now();
                ctx.engine.widen_state.reinit(raw, last_widened_now);
                unsafe { kalico_h7_enable_tim5(); }
            }
            _ => {}
        }
        KALICO_OK
    }

    /// Load a curve into a slab slot. Producer-side validation rejects bad data.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_load_curve(
        rt: *mut KalicoRuntime,
        slot_idx: u16,
        control_points_flat: *const f32, n_cp: u16,
        knots: *const f32, n_knots: u16,
        weights: *const f32, n_weights: u16,
        degree: u8,
    ) -> i32 {
        if rt.is_null() || control_points_flat.is_null() ||
           knots.is_null() || weights.is_null() {
            return KALICO_ERR_NULL_PTR;
        }
        if INIT_STATE.load(Ordering::Acquire) != INIT_READY {
            return KALICO_ERR_NOT_INIT;
        }
        let ctx = unsafe { &mut *(rt as *mut RuntimeContext) };
        let cps_slice = unsafe {
            core::slice::from_raw_parts(control_points_flat, n_cp as usize * MAX_DIM)
        };
        let knots_slice = unsafe { core::slice::from_raw_parts(knots, n_knots as usize) };
        let weights_slice = unsafe { core::slice::from_raw_parts(weights, n_weights as usize) };
        match ctx.pool.load(CurveHandle(slot_idx), cps_slice, knots_slice,
                            weights_slice, degree) {
            Ok(()) => KALICO_OK,
            Err(runtime::curve_pool::CurvePoolError::OutOfBounds) => KALICO_ERR_INVALID_HANDLE,
            Err(runtime::curve_pool::CurvePoolError::SlotAlreadyLoaded) => KALICO_ERR_INVALID_HANDLE,
            Err(_) => KALICO_ERR_INVALID_CURVE,
        }
    }

    /// ISR entrypoint. Spec §3.2 / §4.2.
    /// `raw_cyccnt` is the raw 32-bit DWT->CYCCNT value; Rust widens to u64.
    /// Skips null-check (caller is the C ISR shim with stable handle).
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_tick(
        rt: *mut KalicoRuntime,
        raw_cyccnt: u32,
    ) {
        // Defensive Acquire-load — guards against early-fire during INITING.
        if INIT_STATE.load(Ordering::Acquire) != INIT_READY {
            return;
        }
        let ctx = unsafe { &mut *(rt as *mut RuntimeContext) };
        let now = ctx.engine.widen_state.widen(raw_cyccnt);
        let _ = ctx.engine.tick(now, &mut ctx.queue, &ctx.pool, &mut ctx.trace);
    }

    /// Foreground drain. Returns count of samples written.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_drain_trace(
        rt: *mut KalicoRuntime,
        out_buf: *mut TraceSample,
        out_cap: u32,
    ) -> u32 {
        if rt.is_null() || out_buf.is_null() {
            return 0;
        }
        if INIT_STATE.load(Ordering::Acquire) != INIT_READY {
            return 0;
        }
        let ctx = unsafe { &mut *(rt as *mut RuntimeContext) };
        let out_slice = unsafe {
            core::slice::from_raw_parts_mut(out_buf, out_cap as usize)
        };
        ctx.trace.drain_into(out_slice) as u32
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_status(rt: *mut KalicoRuntime) -> u8 {
        if rt.is_null() { return RuntimeStatus::Fault as u8; }
        if INIT_STATE.load(Ordering::Acquire) != INIT_READY {
            return RuntimeStatus::Fault as u8;
        }
        let ctx = unsafe { &*(rt as *const RuntimeContext) };
        ctx.engine.status() as u8
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_last_error(rt: *mut KalicoRuntime) -> i32 {
        if rt.is_null() { return KALICO_ERR_NULL_PTR; }
        if INIT_STATE.load(Ordering::Acquire) != INIT_READY {
            return KALICO_ERR_NOT_INIT;
        }
        let ctx = unsafe { &*(rt as *const RuntimeContext) };
        ctx.engine.last_error()
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_tick_counter(rt: *mut KalicoRuntime) -> u32 {
        if rt.is_null() { return 0; }
        if INIT_STATE.load(Ordering::Acquire) != INIT_READY {
            return 0;
        }
        let ctx = unsafe { &*(rt as *const RuntimeContext) };
        ctx.engine.tick_counter()
    }

    // C-side timer-control helpers — defined in src/stm32/kalico_h7_timer.c.
    unsafe extern "C" {
        fn kalico_h7_enable_tim5();
        fn kalico_h7_disable_tim5();
        fn kalico_h7_read_cyccnt() -> u32;     // wraps DWT->CYCCNT read
    }
}
```

(Note on `unimplemented!()` in feature stubs: those compile-time stubs will block compilation when the Step 9/8 feature is enabled — exactly the right behavior for forcing concrete impls before that step lands.)

- [ ] **Step 3: Update `lib.rs` to wire both modules**

```rust
//! Kalico C-FFI staticlib. Umbrella for nurbs (Layer 0) and runtime (Layer 4).
//! Spec §2.2 / §3.2.

#![cfg_attr(not(feature = "host"), no_std)]
#![allow(unsafe_code)]   // FFI is inherently unsafe; pure-Rust crates keep deny.

mod nurbs_ffi;
mod runtime_ffi;

// Re-export FFI symbols at crate root so integration tests can call them
// (they're declared inside `mod exports` per cfg-feature gating).
#[cfg(feature = "header-nurbs")]
pub use nurbs_ffi::exports::*;
#[cfg(feature = "header-runtime")]
pub use runtime_ffi::exports::*;

// Re-export error code constants used by integration tests.
pub use runtime::error::*;

// Single panic handler for MCU; std for host.
#[cfg(not(feature = "host"))]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop { core::hint::spin_loop(); }
}
```

- [ ] **Step 4: Build host + MCU**

```bash
cd rust && cargo build -p kalico-c-api --no-default-features --features host,header-nurbs,header-runtime
cd rust && cargo build -p kalico-c-api --no-default-features --features mcu-h7,header-nurbs,header-runtime --target thumbv7em-none-eabihf 2>&1 | tail -10
```

(If the MCU build target isn't installed: `rustup target add thumbv7em-none-eabihf`.)

Both must succeed. The MCU build may take longer the first time (compiling no_std nurbs + heapless from scratch).

- [ ] **Step 5: Commit**

```bash
git add rust/kalico-c-api/src/
git commit -m "kalico-c-api: kalico_runtime_* FFI surface (Step 5)

Spec §3.2. Init-once via UnsafeCell + AtomicU8 INIT_STATE state machine
(UNINIT/INITING/READY). Producer-side push validates kinematic tag,
duration ≥ MIN_SEGMENT_CYCLES, FAULT-not-latched. Tick entrypoint
widens raw u32 CYCCNT to u64 before Engine::tick. Both header-nurbs
and header-runtime cfg gates wire up; MCU and host both build clean."
```

#### Known issue: cross-FFI `&mut RuntimeContext` aliasing

Both round-1 reviewers (Codex + Verifier) flagged that the FFI shape — every entrypoint converting `*mut KalicoRuntime` to `&mut RuntimeContext` — produces overlapping `&mut`s under Rust's strict aliasing model when the ISR preempts foreground (which happens by design at 40 kHz). This is **latent UB** even though on a single-core M7 the ISR and foreground never *concurrently* execute Rust code.

Step 5 accepts this latent UB as a known issue. Mitigations in place:
- The shared objects (`SegmentQueue`, `TraceRing`) use `heapless::spsc::Queue` whose internals are atomic-correct on M7, so the *data races* on those subfields are well-defined.
- Atomic fields (`status`, `last_error`, `tick_counter`) on `Engine` use `&self` via interior mutability — they're not affected by `&mut RuntimeContext` aliasing.
- Plain mutable fields on `Engine` (`current`, `last_motors`, `widen_state`) are touched only by the ISR — the foreground never reads them.

**Step 6 hardening track (deferred):** when the live producer task lands at Step 6, refactor to one of:
- (a) `heapless::spsc::Producer` / `Consumer` half-split with each half stored in a separate static cell (proper SPSC ownership at the type level).
- (b) `cortex_m::interrupt::free` around foreground accesses (heavyweight but trivially correct).
- (c) Refactor FFI to never materialize `&mut RuntimeContext`; each entrypoint accesses only the disjoint subfields it needs via raw-pointer arithmetic.

Miri (Phase 7 CI) will flag the aliasing if it tries to model concurrent ISR/FG execution; currently it doesn't. Spec §6.8 already lists "real concurrent-producer hardware test" as deferred to Step 6 — this aliasing gap is the same Step-6 concern.

### Task 14: Init-once invariant tests

**Files:**
- Create: `rust/kalico-c-api/tests/init_once.rs`

- [ ] **Step 1: Write the test**

```rust
//! Init-once invariant tests. Spec §3.2.

#[test]
fn second_init_returns_null() {
    let h1 = unsafe { kalico_c_api::kalico_runtime_init() };
    assert!(!h1.is_null());
    let h2 = unsafe { kalico_c_api::kalico_runtime_init() };
    assert!(h2.is_null(), "second init must return null");
}

#[test]
fn null_handle_returns_null_ptr_error() {
    let r = unsafe {
        kalico_c_api::kalico_runtime_push_segment(
            std::ptr::null_mut(), 0, 0, 0, 100, 0,
        )
    };
    assert_eq!(r, kalico_c_api::KALICO_ERR_NULL_PTR);
}
```

(Note: this test must run *after* Step 5's compile gate validates the surface symbol export — wire up `pub use runtime_ffi::*` in `lib.rs` first if needed for crate-wide visibility.)

- [ ] **Step 2: Verify and commit**

```bash
cd rust && cargo test -p kalico-c-api --features host,header-runtime
git add rust/kalico-c-api/tests/init_once.rs
git commit -m "kalico-c-api: init-once invariant test (Step 5)

Verifies the UnsafeCell + AtomicU8 protocol — second init returns
null, null handle returns KALICO_ERR_NULL_PTR. Spec §3.2 / §5.6."
```

### Task 15: Extend `gen-headers` binary to emit `kalico_runtime.h`

**Files:**
- Modify: `rust/kalico-c-api/src/bin/gen_headers.rs`
- Create: `rust/kalico-c-api/cbindgen-runtime.toml`

**Why:** Spec §3.2 — two cbindgen configs, one staticlib crate, run cbindgen twice with different cfg flags.

- [ ] **Step 1: Create `rust/kalico-c-api/cbindgen-runtime.toml`**

```toml
language = "C"
header = """\
/*\n\
 * kalico_runtime.h — generated by cbindgen.\n\
 * DO NOT EDIT. Regenerate via `cargo run -p kalico-c-api --bin gen-headers`.\n\
 * See docs/superpowers/specs/2026-04-28-layer-4-mcu-framework-stub-design.md.\n\
 */\n"""

include_guard = "KALICO_RUNTIME_H"
pragma_once = true
no_includes = false

[parse]
parse_deps = true
include = ["runtime", "kalico-c-api"]
expand = ["kalico-c-api"]
```

- [ ] **Step 2: Update `gen_headers.rs` to emit both headers via cfg-gating**

Per spec §3.2: cbindgen has no prefix-filter mode; we run cbindgen *twice* against the same crate with different `cfg` flags via `cargo run --features` to gate which FFI modules expand. Each invocation produces one header:

```rust
//! Generate kalico_nurbs.h and kalico_runtime.h via cbindgen.
//!
//! Run twice with different feature gates so each cbindgen invocation only
//! sees the FFI module for the header it's emitting:
//!   cargo run -p kalico-c-api --bin gen-headers --features header-nurbs
//!   cargo run -p kalico-c-api --bin gen-headers --features header-runtime

fn main() {
    let crate_dir = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    let want_nurbs = cfg!(feature = "header-nurbs");
    let want_runtime = cfg!(feature = "header-runtime");
    if want_nurbs && want_runtime {
        eprintln!("error: gen-headers must be invoked with EXACTLY ONE of \
                  --features header-nurbs / --features header-runtime so \
                  cbindgen sees only the symbols for that header.");
        std::process::exit(1);
    }
    if want_nurbs {
        let cfg = cbindgen::Config::from_file(
            format!("{crate_dir}/cbindgen.toml")).unwrap();
        cbindgen::Builder::new()
            .with_crate(&crate_dir)
            .with_config(cfg)
            .generate()
            .expect("kalico_nurbs.h generation failed")
            .write_to_file(format!("{crate_dir}/include/kalico_nurbs.h"));
        println!("Generated kalico_nurbs.h");
        return;
    }
    if want_runtime {
        let cfg = cbindgen::Config::from_file(
            format!("{crate_dir}/cbindgen-runtime.toml")).unwrap();
        cbindgen::Builder::new()
            .with_crate(&crate_dir)
            .with_config(cfg)
            .generate()
            .expect("kalico_runtime.h generation failed")
            .write_to_file(format!("{crate_dir}/include/kalico_runtime.h"));
        println!("Generated kalico_runtime.h");
        return;
    }
    eprintln!("error: invoke with --features header-nurbs OR --features header-runtime");
    std::process::exit(1);
}
```

A small wrapper script (`tools/regen_headers.sh`) calls both:

```bash
#!/usr/bin/env bash
set -euo pipefail
cd rust
cargo run -p kalico-c-api --bin gen-headers --features header-nurbs
cargo run -p kalico-c-api --bin gen-headers --features header-runtime
echo "Both headers regenerated."
```

- [ ] **Step 3: Run gen-headers and inspect outputs**

```bash
cd rust && cargo run -p kalico-c-api --bin gen-headers
cat rust/kalico-c-api/include/kalico_runtime.h | head -40
```

Expected: a valid C header with `#define KALICO_RUNTIME_H`, `#pragma once`, declarations for `kalico_runtime_init`, `kalico_runtime_push_segment`, etc., and the `TraceSample` struct.

- [ ] **Step 4: Update `tests/headers_no_drift.rs` to assert no diff**

The existing test (carried over from `nurbs-c-api`) should already do this for `kalico_nurbs.h`. Extend or add a parallel assertion for `kalico_runtime.h`.

- [ ] **Step 5: Commit**

```bash
git add rust/kalico-c-api/src/bin/gen_headers.rs \
        rust/kalico-c-api/cbindgen-runtime.toml \
        rust/kalico-c-api/include/kalico_runtime.h \
        rust/kalico-c-api/tests/headers_no_drift.rs
git commit -m "kalico-c-api: emit kalico_runtime.h via cbindgen (Step 5)

Two-cbindgen-configs-one-crate pattern per spec §3.2 — gen-headers
binary runs cbindgen twice with different prefix filters. Header
drift detection extended to cover kalico_runtime.h."
```

### Task 16: Static-assertions for `TraceSample` layout

**Files:**
- Create: `rust/kalico-c-api/tests/c_smoke_build.rs`
- Create: `rust/kalico-c-api/tests/c_smoke/main.c` (compiled and linked by the test)

**Why:** Spec §6.3 — C smoke build catches cbindgen drift, repr mismatches, struct-size disagreements that Rust-side tests cannot see.

- [ ] **Step 1: Write the C smoke source**

```c
// rust/kalico-c-api/tests/c_smoke/main.c
#include <stdint.h>
#include <stddef.h>
#include "kalico_runtime.h"

// Spec §6.3: every ABI-relevant type covered.
_Static_assert(sizeof(TraceSample) == 32, "TraceSample size mismatch");
_Static_assert(_Alignof(TraceSample) == 8, "TraceSample alignment mismatch");
_Static_assert(offsetof(TraceSample, tick) == 0, "tick offset");
_Static_assert(offsetof(TraceSample, motor_a) == 8, "motor_a offset");
_Static_assert(offsetof(TraceSample, motor_b) == 12, "motor_b offset");
_Static_assert(offsetof(TraceSample, motor_e) == 16, "motor_e offset");
_Static_assert(offsetof(TraceSample, segment_id) == 20, "segment_id offset");
_Static_assert(offsetof(TraceSample, flags) == 24, "flags offset");

int main(void) {
    // Trivial smoke — link symbol resolution check, no runtime call.
    void* h = kalico_runtime_init();
    if (h == NULL) return 1;
    return 0;
}
```

- [ ] **Step 2: Write a Rust test that drives the C smoke build**

```rust
// rust/kalico-c-api/tests/c_smoke_build.rs
//! Compiles the C smoke main.c against the regenerated header + staticlib.

#[test]
fn c_smoke_compiles_and_links() {
    let cc = std::env::var("CC").unwrap_or_else(|_| "cc".to_string());
    let crate_dir = env!("CARGO_MANIFEST_DIR");
    let c_src = format!("{crate_dir}/tests/c_smoke/main.c");
    let header_dir = format!("{crate_dir}/include");
    let target_dir = std::env::var("CARGO_TARGET_DIR")
        .unwrap_or_else(|_| format!("{crate_dir}/../target"));
    let out = format!("{target_dir}/c_smoke_test");

    let status = std::process::Command::new(&cc)
        .args([
            &c_src,
            &format!("-I{header_dir}"),
            &format!("-L{target_dir}/release"),
            "-lkalico_c_api",
            "-lpthread", "-ldl", "-lm",
            "-o", &out,
        ])
        .status()
        .expect("failed to spawn cc");
    assert!(status.success(), "C smoke build did not compile");
}
```

(Note: this assumes a host build of `libkalico_c_api.a` exists — run `cargo build -p kalico-c-api --release` first. CI will need to wire the prerequisite. Adjust the link line per host platform — add `-Wl,--no-as-needed` on Linux if needed; macOS uses `-framework` semantics if any system symbols are pulled.)

- [ ] **Step 3: Verify and commit**

```bash
cd rust && cargo build -p kalico-c-api --release
cd rust && cargo test -p kalico-c-api --features host,header-runtime --test c_smoke_build
git add rust/kalico-c-api/tests/c_smoke/main.c rust/kalico-c-api/tests/c_smoke_build.rs
git commit -m "kalico-c-api: C smoke build with offsetof static assertions

Spec §6.3. Every ABI-relevant TraceSample field has _Static_assert(
offsetof) — drift detection beyond just sizeof. Link line resolves
all kalico_runtime_* symbols against libkalico_c_api.a."
```

---

## Phase 5: Host tests

### Task 17a: Shared `step5_segments.json` fixture

**Files:**
- Create: `rust/runtime/tests/fixtures/step5_segments.json`
- Create: `rust/runtime/tests/fixtures/mod.rs` (parser used by Surface A unit tests)

**Why:** Spec §6.7 — single source of truth for "the 4 hand-built test segments" used by Surface A unit tests, the Surface B C smoke build (where applicable), and the Surface C host Python plot validation. Without this, surfaces could each validate a different curve interpretation and miss bugs at integration time.

- [ ] **Step 1: Define the fixture JSON**

```json
{
  "fixtures": [
    {
      "name": "straight_line_x",
      "description": "Degree-1 line from origin to (10, 0, 0).",
      "control_points": [[0, 0, 0], [10, 0, 0]],
      "knots": [0, 0, 1, 1],
      "weights": [1, 1],
      "degree": 1,
      "duration_us": 5000,
      "kinematics": "CoreXyAndE"
    },
    {
      "name": "rational_quadratic_arc",
      "description": "Quarter-circle arc (rational quadratic NURBS).",
      "control_points": [[10, 0, 0], [10, 10, 0], [0, 10, 0]],
      "knots": [0, 0, 0, 1, 1, 1],
      "weights": [1, 0.7071068, 1],
      "degree": 2,
      "duration_us": 15000,
      "kinematics": "CoreXyAndE"
    },
    {
      "name": "smooth_corner_cubic",
      "description": "Cubic Bezier corner blend.",
      "control_points": [[0, 10, 0], [-3, 10, 0], [-10, 10, 0], [-10, 5, 0]],
      "knots": [0, 0, 0, 0, 1, 1, 1, 1],
      "weights": [1, 1, 1, 1],
      "degree": 3,
      "duration_us": 10000,
      "kinematics": "CoreXyAndE"
    },
    {
      "name": "halt_sentinel",
      "description": "Static point — used as the soft halt at chain end.",
      "control_points": [[-10, 5, 0], [-10, 5, 0]],
      "knots": [0, 0, 1, 1],
      "weights": [1, 1],
      "degree": 1,
      "duration_us": 2000,
      "kinematics": "CoreXyAndE"
    }
  ]
}
```

- [ ] **Step 2: Write the Rust parser**

```rust
// rust/runtime/tests/fixtures/mod.rs
//! Shared test fixtures. Used by Surface A integration tests + Surface B
//! FFI tests + Surface C Python validation. Spec §6.7.

use serde::Deserialize;

#[derive(Debug, Deserialize, Clone)]
pub struct Fixture {
    pub name: String,
    pub description: String,
    pub control_points: Vec<[f32; 3]>,
    pub knots: Vec<f32>,
    pub weights: Vec<f32>,
    pub degree: u8,
    pub duration_us: u32,
    pub kinematics: String,  // "CoreXyAndE" or "CartesianXyzAndE"
}

#[derive(Debug, Deserialize)]
pub struct FixtureSet {
    pub fixtures: Vec<Fixture>,
}

pub fn load() -> FixtureSet {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/step5_segments.json");
    let raw = std::fs::read_to_string(path).expect("fixture file missing");
    serde_json::from_str(&raw).expect("fixture parse failed")
}
```

(Add `serde = { version = "1", features = ["derive"] }` and `serde_json = "1"` to `runtime/Cargo.toml`'s `[dev-dependencies]`.)

- [ ] **Step 3: Update Task 11's integration test to use shared fixtures**

Replace ad-hoc `load_line` calls in `engine_tick.rs` with fixture-driven setup:

```rust
mod fixtures;

#[test]
fn straight_line_fixture_traces_correctly() {
    let set = fixtures::load();
    let line = set.fixtures.iter().find(|f| f.name == "straight_line_x").unwrap();
    // ... drive Engine::tick with this fixture ...
}
```

(Surface C's host Python script reads the same JSON file, so a divergence between the Rust integration test trace and the hardware trace points at the same fixture.)

- [ ] **Step 4: Commit**

```bash
git add rust/runtime/tests/fixtures/ rust/runtime/Cargo.toml rust/runtime/tests/engine_tick.rs
git commit -m "runtime/tests: shared step5_segments.json fixture (Step 5)

Spec §6.7. Single source of truth for the 4 Step-5 test curves —
Surface A integration tests, Surface B FFI tests (where curve data
matters), and Surface C host Python validation all parse the same
JSON. Fixtures: straight line, rational quadratic arc, cubic Bezier
smooth corner, halt sentinel."
```

### Task 17b: Engine state machine — additional unit coverage

**Files:**
- Modify: `rust/runtime/tests/engine_tick.rs` (extend)

- [ ] **Step 1-3: Add tests for fault paths and concurrency-tolerance**

Add these tests to the existing file:

```rust
#[test]
fn invalid_curve_handle_latches_fault() {
    let mut engine = Engine::<NoopPa, NoopIs>::new(CLOCK_FREQ);
    let mut queue = SegmentQueue::new();
    let pool = CurvePool::new();   // empty — handle 0 is unloaded
    let mut trace = TraceRing::<1024>::new();

    let tc = one_tick_cycles(CLOCK_FREQ) as u64;
    queue.try_push(Segment {
        id: 1, curve: CurveHandle(0), t_start: 0, t_end: tc * 2,
        kinematics: KinematicTag::CoreXyAndE,
    }).unwrap();

    let r = engine.tick(0, &mut queue, &pool, &mut trace);
    assert!(r.is_err());
    assert_eq!(engine.status(), RuntimeStatus::Fault);
    assert_eq!(engine.last_error(), runtime::error::KALICO_ERR_INVALID_HANDLE);

    // Trace has fault marker.
    let mut out = [TraceSample::default(); 8];
    let n = trace.drain_into(&mut out);
    assert_eq!(n, 1);
    assert_ne!(out[0].flags & runtime::trace::TRACE_FLAG_FAULT_MARKER, 0);
}
```

(Add similar tests for `BoundaryLoopExhausted` — push 9+ short segments, `NaNOrInfFromEval` — load a curve with NaN that survives validation by being NaN-after-eval, etc.)

- [ ] **Step 4: Verify and commit**

```bash
cd rust && cargo test -p runtime --features host --test engine_tick
git add rust/runtime/tests/engine_tick.rs
git commit -m "runtime/tests: fault-path coverage (Step 5)

Spec §5.5. invalid-handle, boundary-loop-exhausted, NaN-from-eval all
latch FAULT correctly with last_error code, fault-marker trace sample,
and last-known-good motor positions."
```

### Task 18: Wrap arithmetic tests near `u64::MAX`

**Files:**
- Create: `rust/runtime/tests/wrap_arithmetic.rs`

- [ ] **Step 1: Write the test**

```rust
//! Wrap-arithmetic tests. Spec §5.8.

use runtime::clock::WidenState;

#[test]
fn widen_handles_max_minus_one_to_zero() {
    let mut state = WidenState::default();
    state.reinit(0xFFFF_FFF0, 0);
    let now1 = state.widen(0xFFFF_FFFE);
    assert_eq!(now1, 0xFFFF_FFFE);
    // Now wrap.
    let now2 = state.widen(0x0000_0010);
    assert_eq!(now2, (1u64 << 32) | 0x10);
    assert!(now2 > now1, "monotonicity broken");
}

#[test]
fn boundary_loop_works_near_u64_max() {
    use runtime::clock::one_tick_cycles;
    use runtime::curve_pool::*;
    use runtime::engine::*;
    use runtime::queue::*;
    use runtime::segment::*;
    use runtime::slot::*;
    use runtime::trace::*;

    // Default H723 Klipper Kconfig clock is 520 MHz (src/stm32/Kconfig). Keeping
// tests parametric here so a future bump to 550 MHz (or different alternate
// kconfig) doesn't invalidate the fixture math.
const CLOCK_FREQ: u32 = 520_000_000;
    let tc = one_tick_cycles(CLOCK_FREQ) as u64;

    let mut engine = Engine::<NoopPa, NoopIs>::new(CLOCK_FREQ);
    let mut queue = SegmentQueue::new();
    let mut pool = CurvePool::new();
    let mut trace = TraceRing::<1024>::new();

    // Construct a segment whose t_start, t_end are near u64::MAX.
    let near_max = u64::MAX - tc * 100;
    let cps = [0.0f32, 0.0, 0.0, 1.0, 0.0, 0.0];
    let knots = [0.0f32, 0.0, 1.0, 1.0];
    let weights = [1.0f32, 1.0];
    pool.load(CurveHandle(0), &cps, &knots, &weights, 1).unwrap();
    queue.try_push(Segment {
        id: 1, curve: CurveHandle(0), t_start: near_max, t_end: near_max + tc * 4,
        kinematics: KinematicTag::CoreXyAndE,
    }).unwrap();

    let r = engine.tick(near_max + tc, &mut queue, &pool, &mut trace);
    assert!(r.is_ok(), "tick near u64::MAX should not panic or fault");
    assert_ne!(engine.status(), RuntimeStatus::Fault);
}
```

- [ ] **Step 2: Verify and commit**

```bash
cd rust && cargo test -p runtime --features host --test wrap_arithmetic
git add rust/runtime/tests/wrap_arithmetic.rs
git commit -m "runtime/tests: wrap-arithmetic near u64::MAX + CYCCNT (Step 5)

Spec §5.8. WidenState handles raw u32 wrap monotonically; Engine::tick
near u64::MAX doesn't panic or latch FAULT under release-mode
overflow-checks=false."
```

### Task 19: Loom tests gated by `cfg(loom)`

**Files:**
- Create: `rust/runtime/tests/loom.rs`

**Why:** Spec §6.2 — loom can't model cortex-m IRQs but exhaustively explores Acquire/Release interleavings on the trace overflow_pending bool.

- [ ] **Step 1: Write a minimal loom test**

```rust
// rust/runtime/tests/loom.rs
#![cfg(feature = "loom")]

#[cfg(feature = "loom")]
mod loom_tests {
    // ...wire loom::sync atomics through the existing TraceRing...
    // (Loom test setup left as exercise — depends on whether the crate
    // exposes a `cfg(loom)` swap of std::sync::atomic vs loom::sync::atomic.
    // For Step 5, dispatch the feature gate without a full loom suite if
    // the wiring becomes more than a half-day; add a `// TODO Step-6:
    // expand loom coverage` comment in trace.rs.)
}
```

(Pragmatic: loom integration in `no_std` crates is non-trivial. For Step 5, ship the feature gate + wiring scaffold; defer full loom coverage to Step 6 when the live producer makes it directly load-bearing. Document the deferral in §6.8 of the spec — already done.)

- [ ] **Step 2: Verify the gate compiles**

```bash
cd rust && cargo build -p runtime --features loom
```

- [ ] **Step 3: Commit**

```bash
git add rust/runtime/tests/loom.rs
git commit -m "runtime/tests: loom feature gate scaffold (Step 5 minimal)

Full loom coverage deferred to Step 6 (see spec §6.8); the gate is
in place so Step 6's first task is wiring the actual atomic-ordering
model, not setting up the cfg infrastructure."
```

### Task 20: Miri runs on UnsafeCell-touching tests

**Files:** none — this is a CI gate, exercised in Phase 7.

(Skip-implementation step: we add the miri invocation to CI in Task 32. No new code at Step 20.)

- [ ] **Step 1: Verify miri runs locally for a representative test**

```bash
cd rust && cargo +nightly miri test -p runtime --features host curve_pool::tests 2>&1 | tail -10
```

Expected: tests pass under miri (no UB detected). If miri reports anything, fix the affected code now. If miri isn't installed: `rustup +nightly component add miri`.

- [ ] **Step 2: Document the result in commit message**

```bash
git commit --allow-empty -m "runtime: miri smoke pass on host curve_pool tests (Step 5)

cargo +nightly miri test -p runtime --features host curve_pool::tests
exits clean — UnsafeCell-touching curve_pool init and slot-load paths
are UB-free under miri's strict aliasing model."
```

---

## Phase 6: Klipper C integration

### Task 21: Patch `src/stm32/watchdog.c` with `kalico_liveness_ok` hook

**Files:**
- Modify: `src/stm32/watchdog.c`

**Why:** Spec §5.7 — watchdog gate so the kalico runtime can stop kicking on liveness fault.

- [ ] **Step 1: Read the existing watchdog.c to identify the kick task**

```bash
cat src/stm32/watchdog.c
```

Identify the `DECL_TASK(watchdog_reset)` function and the `IWDG->KR = 0xAAAA` write line.

- [ ] **Step 2: Add the gate — wrapped in `#ifdef CONFIG_KALICO_RUNTIME`**

Patch the file to add:

```c
#if CONFIG_KALICO_RUNTIME
// Spec §5.7 — kalico runtime liveness gate. Foreground (runtime_drain task)
// is the sole writer; this file only reads. __attribute__((used,
// externally_visible)) survives Klipper's -fwhole-program --gc-sections.
volatile uint8_t kalico_liveness_ok __attribute__((used, externally_visible))
    = 1;
#endif

void
watchdog_reset(void)
{
#if CONFIG_KALICO_RUNTIME
    if (!kalico_liveness_ok) return;   // kalico runtime detected liveness fault
#endif
    IWDG->KR = 0xAAAA;
}
```

- [ ] **Step 3: Add a CI grep canary**

```bash
echo "# Spec §5.7 — verify the kalico_liveness_ok gate is in place" >> .github/workflows/kalico-canary.txt 2>/dev/null || true
```

(In Phase 7, the CI workflow YAML asserts: `grep -F 'kalico_liveness_ok' src/stm32/watchdog.c && grep -F 'CONFIG_KALICO_RUNTIME' src/stm32/watchdog.c`.)

- [ ] **Step 4: Verify the watchdog.c still builds with `CONFIG_KALICO_RUNTIME=n`**

(Without the runtime, the `#ifdef` blocks compile out; existing builds are unchanged.)

- [ ] **Step 5: Commit**

```bash
git add src/stm32/watchdog.c
git commit -m "src/stm32/watchdog: kalico_liveness_ok gate (Step 5)

Spec §5.7. Gated by CONFIG_KALICO_RUNTIME so non-kalico builds are
unaffected. Foreground sole-writer (runtime_drain task); watchdog_reset
reads volatile flag before kicking IWDG. Blocks the 'ISR returns to C
but makes no progress' liveness-fault class that pure watchdog can't
catch."
```

### Task 22: Create `src/runtime_tick.c`

**Files:**
- Create: `src/runtime_tick.c`

**Why:** Spec §2.4 — portable Klipper-side glue. `DECL_INIT`, `DECL_TASK`, `DECL_COMMAND`. Exposes `kalico_clock_freq` for Rust.

- [ ] **Step 1: Write the C source**

```c
// src/runtime_tick.c
//
// Klipper-side portable glue for kalico runtime. Spec §2.4 / §4.5 / §5.7.

#include "autoconf.h"
#include "board/misc.h"  // timer_read_time
#include "command.h"     // DECL_COMMAND
#include "sched.h"       // DECL_INIT, DECL_TASK
#include "kalico_runtime.h"

#if CONFIG_KALICO_RUNTIME

// Exposed to Rust via `extern "C" { static kalico_clock_freq: u32; }`.
// __attribute__((used, externally_visible)) survives -fwhole-program LTO + GC.
const uint32_t kalico_clock_freq __attribute__((used, externally_visible))
    = CONFIG_CLOCK_FREQ;

extern volatile uint8_t kalico_liveness_ok;  // defined in src/stm32/watchdog.c

void* kalico_rt_handle = 0;            // exposed (non-static) for kalico_h7_timer.c
static struct task_wake runtime_drain_wake;
static struct timer runtime_drain_timer;

// Liveness monitor state.
static uint32_t last_seen_tick_counter = 0;
static uint32_t last_progress_time = 0;

// Periodic timer callback at ~1 kHz: sets the drain wake flag.
// Per spec §4.5 — sched_check_wake throttle prevents spinning the drain
// task at full FG iteration rate when the trace ring is empty.
static uint_fast8_t
runtime_drain_event(struct timer *t)
{
    sched_wake_task(&runtime_drain_wake);
    t->waketime += timer_from_us(1000);  // 1 kHz
    return SF_RESCHEDULE;
}

void
runtime_init(void)
{
    kalico_rt_handle = kalico_runtime_init();
    if (!kalico_rt_handle) {
        // Init failed — leave liveness flag at default (1 = OK) but handle unset;
        // calls into the runtime will short-circuit safely.
        return;
    }
    last_seen_tick_counter = kalico_runtime_tick_counter(kalico_rt_handle);
    last_progress_time = timer_read_time();

    // Initialize H7 timer hardware (TIM5) — DOES NOT enable yet; first segment
    // push triggers enable via the producer protocol (§4.4).
    extern void kalico_h7_timer_init(void);
    kalico_h7_timer_init();

    // Wire the periodic 1 kHz drain wake.
    runtime_drain_timer.func = runtime_drain_event;
    runtime_drain_timer.waketime = timer_read_time() + timer_from_us(1000);
    sched_add_timer(&runtime_drain_timer);
}
DECL_INIT(runtime_init);

#define KALICO_TRACE_BATCH 64
#define KALICO_LIVENESS_THRESHOLD_MS 25
#define KALICO_LIVENESS_THRESHOLD_TICKS  \
    ((KALICO_LIVENESS_THRESHOLD_MS) * (CONFIG_CLOCK_FREQ / 1000))

void
runtime_drain(void)
{
    if (!kalico_rt_handle) return;
    if (!sched_check_wake(&runtime_drain_wake)) return;

    // Drain a batch.
    static uint8_t batch_buf[KALICO_TRACE_BATCH * 32];  // 32 bytes per sample
    uint32_t n = kalico_runtime_drain_trace(
        kalico_rt_handle, (struct TraceSample*)batch_buf, KALICO_TRACE_BATCH);
    if (n > 0) {
        sendf("kalico_trace count=%u data=%*s", n, n * 32, batch_buf);
    }

    // Liveness check.
    uint32_t cur_counter = kalico_runtime_tick_counter(kalico_rt_handle);
    uint32_t cur_time = timer_read_time();
    if (cur_counter != last_seen_tick_counter) {
        last_seen_tick_counter = cur_counter;
        last_progress_time = cur_time;
    } else if ((cur_time - last_progress_time) > KALICO_LIVENESS_THRESHOLD_TICKS) {
        // ISR has stalled. Stop kicking the watchdog.
        kalico_liveness_ok = 0;
    }

    // Or fault → also block kicks.
    if (kalico_runtime_status(kalico_rt_handle) == 3 /* FAULT */) {
        kalico_liveness_ok = 0;
    }
}
DECL_TASK(runtime_drain);

// DECL_COMMAND surface — test harness loads curves and pushes segments.
void
command_kalico_load_curve(uint32_t *args)
{
    if (!kalico_rt_handle) {
        sendf("kalico_load_curve_response result=%d", -7);
        return;
    }
    uint16_t slot = args[0];
    uint8_t degree = args[1];
    uint16_t n_cp = args[2];
    uint16_t n_knots = args[3];
    const float *cps = (const float*)args[4];
    const float *knots = (const float*)args[5];
    const float *weights = (const float*)args[6];
    int32_t r = kalico_runtime_load_curve(
        kalico_rt_handle, slot, cps, n_cp, knots, n_knots, weights, n_cp, degree);
    sendf("kalico_load_curve_response result=%i", r);
}
DECL_COMMAND(command_kalico_load_curve,
    "kalico_load_curve slot=%hu degree=%c n_cp=%hu n_knots=%hu "
    "cps=%*s knots=%*s weights=%*s");

void
command_kalico_push_segment(uint32_t *args)
{
    if (!kalico_rt_handle) { sendf("kalico_push_response result=%d", -7); return; }
    uint32_t id = args[0];
    uint16_t curve = args[1];
    uint64_t t_start = ((uint64_t)args[2] << 32) | args[3];
    uint64_t t_end   = ((uint64_t)args[4] << 32) | args[5];
    uint8_t kin = args[6];
    int32_t r = kalico_runtime_push_segment(
        kalico_rt_handle, id, curve, t_start, t_end, kin);
    sendf("kalico_push_response result=%i", r);
}
DECL_COMMAND(command_kalico_push_segment,
    "kalico_push_segment id=%u curve=%hu t_start_hi=%u t_start_lo=%u "
    "t_end_hi=%u t_end_lo=%u kinematics=%c");

void
command_kalico_query_status(uint32_t *args)
{
    if (!kalico_rt_handle) { sendf("kalico_status status=255 last_err=-7"); return; }
    uint8_t status = kalico_runtime_status(kalico_rt_handle);
    int32_t last_err = kalico_runtime_last_error(kalico_rt_handle);
    sendf("kalico_status status=%c last_err=%i", status, last_err);
}
DECL_COMMAND(command_kalico_query_status, "kalico_query_status");

// ---- Cycle-count bench (Task 27 / spec §6.4) ---------------------------
//
// Surface-C only. Captures DWT->CYCCNT around `kalico_runtime_tick` over N
// samples and replies with one `kalico_bench_sample value=<cycles>` response
// per measurement (after the warmup skip) and a final `kalico_bench_done
// count=<N> error=0` per the host-side test_h723_cycle_count.py protocol.
// Wire format is Klipper's standard binary VLQ (sendf); host-side parses
// via klippy/msgproto.py wrapped by tools/kalico_host_io.py.
//
// `isolate=1` selectively masks USB+USART IRQs during the measurement window
// (TIM5 stays enabled). `isolate=0` runs with full IRQs (production load).
// SysTick is left untouched — Klipper's foreground time accounting needs it,
// and the kalico TIM5 ISR doesn't preempt SysTick at priority 3 anyway.

// KALICO_BENCH_MAX_SAMPLES is declared in `src/stm32/kalico_h7_timer.h`
// (Task 23 creates it) so both `runtime_tick.c` and `kalico_h7_timer.c`
// see the same value.
#include "stm32/kalico_h7_timer.h"
extern volatile uint32_t kalico_bench_samples_buf[KALICO_BENCH_MAX_SAMPLES];
extern volatile uint16_t kalico_bench_count;
extern volatile uint16_t kalico_bench_target;
extern volatile uint8_t kalico_bench_isolate;

void
command_kalico_bench_run(uint32_t *args)
{
    if (!kalico_rt_handle) { sendf("kalico_bench_done error=-7"); return; }

    // Liveness pre-check (round-4 review): if the runtime had already
    // tripped a liveness fault before we got here, manually kicking IWDG
    // inside the bench loop would mask it. Refuse to bench in that case.
    if (!kalico_liveness_ok) {
        sendf("kalico_bench_done error=-99 reason=liveness_already_tripped");
        return;
    }

    uint8_t isolate = args[0];
    uint16_t samples = args[1];
    if (samples > KALICO_BENCH_MAX_SAMPLES) samples = KALICO_BENCH_MAX_SAMPLES;

    if (isolate) {
        // Selectively mask: USB OTG_FS (the H723 USB controller used by Klipper
        // CDC on Octopus Pro) + USART2 (active console). Leave TIM5 (the kalico
        // ISR) and SysTick alone. The implementer MUST verify which IRQs are
        // active in the current build before relying on the masked list — picking
        // the wrong IRQ silently biases Pass A toward overly-optimistic numbers.
        // Cross-check with `arm-none-eabi-objdump -d klipper.elf | grep -E 'IRQ|Handler'`
        // to confirm the IRQ vector names actually present in the firmware image.
        NVIC_DisableIRQ(OTG_FS_IRQn);
        NVIC_DisableIRQ(USART2_IRQn);
    }

    kalico_bench_count = 0;
    kalico_bench_target = samples;
    kalico_bench_isolate = isolate;

    // Wait for the ISR to fill the buffer with a watchdog-respecting timeout.
    // Worst case: 25 µs/sample × 1024 = 25.6 ms. We allow 100 ms before
    // bailing out, and we kick the IWDG ourselves during the wait so we
    // don't trip Klipper's watchdog from foreground starvation. Note: the
    // liveness-heartbeat counter does freeze for the duration of this wait,
    // but that's bounded and known — it's only used during Surface-C bring-up.
    uint32_t start = timer_read_time();
    uint32_t timeout_ticks = timer_from_us(100000);  // 100 ms
    while (kalico_bench_count < kalico_bench_target) {
        // Manually kick the IWDG (foreground watchdog_reset would otherwise
        // get pre-empted by our spin and starve). Spec §5.7 — `kalico_liveness_ok`
        // is set true here because we KNOW the runtime is healthy; the gate
        // is only meaningful for unattended operation.
        IWDG->KR = 0xAAAA;
        if ((uint32_t)(timer_read_time() - start) > timeout_ticks) {
            // ISR didn't fill the buffer — TIM5 stalled or NVIC mask wrong.
            kalico_bench_target = 0;  // tell ISR to stop bracketing
            sendf("kalico_bench_done error=-99 reason=isr_timeout count=%hu",
                  kalico_bench_count);
            if (isolate) {
                NVIC_EnableIRQ(OTG_FS_IRQn);
                NVIC_EnableIRQ(USART2_IRQn);
            }
            return;
        }
    }

    if (isolate) {
        NVIC_EnableIRQ(OTG_FS_IRQn);
        NVIC_EnableIRQ(USART2_IRQn);
    }

    // Discard the first 8 samples (warm-up: cache fill, branch predictor,
    // FPU lazy-stacking on first vector_eval). Spec §6.4 hardened methodology.
    // Underflow guard: refuse if caller didn't request enough samples.
    const uint16_t WARMUP_SKIP = 8;
    if (samples <= WARMUP_SKIP) {
        sendf("kalico_bench_done error=-4 reason=samples_below_warmup");
        return;
    }

    // Emit one sample per `kalico_bench_sample` line as ASCII decimal —
    // this matches the host script's split-and-int-parse protocol cleanly
    // and avoids any binary-blob framing concerns. Bounded total: at most
    // KALICO_BENCH_MAX_SAMPLES (1024) lines per bench, each ~12 chars.
    for (uint16_t i = WARMUP_SKIP; i < samples; i++) {
        sendf("kalico_bench_sample value=%u", kalico_bench_samples_buf[i]);
    }
    sendf("kalico_bench_done count=%hu error=0",
          (uint16_t)(samples - WARMUP_SKIP));
}
DECL_COMMAND(command_kalico_bench_run, "kalico_bench_run isolate=%c samples=%hu");

#endif // CONFIG_KALICO_RUNTIME
```

- [ ] **Step 2: Verify the file compiles** (later when Phase 7 wires the Makefile, but the C syntax should be checkable now via a one-off):

```bash
arm-none-eabi-gcc -c -DCONFIG_KALICO_RUNTIME=1 \
    -DCONFIG_CLOCK_FREQ=550000000 \
    -I src -I src/stm32 -I src/generic \
    -I rust/kalico-c-api/include \
    -nostdinc -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16 \
    src/runtime_tick.c -o /tmp/runtime_tick.o
```

(If headers under `src/generic` aren't immediately resolvable: this manual compile step is approximate; the real test is in Task 24.)

- [ ] **Step 3: Commit**

```bash
git add src/runtime_tick.c
git commit -m "src/runtime_tick: portable Klipper-side glue for kalico runtime

Spec §2.4 / §4.5 / §5.7. DECL_INIT for runtime_init, DECL_TASK for
runtime_drain (sched_check_wake-throttled), DECL_COMMANDs for
load_curve / push_segment / query_status / drain_trace. Exposes
kalico_clock_freq with __attribute__((used, externally_visible)) to
survive Klipper's -fwhole-program LTO. Liveness monitor stops kicking
the IWDG on stall or FAULT (§5.7)."
```

### Task 23: Create `src/stm32/kalico_h7_timer.c`

**Files:**
- Create: `src/stm32/kalico_h7_timer.c`
- Create: `src/stm32/kalico_h7_timer.h` (shared declarations + bench buffer size)

**Why:** Spec §2.4 — H7-specific TIM5 init + IRQ handler. Init invariant: must clear CR1.CEN + SR.UIF before enabling the IRQ.

- [ ] **Step 1: Write the shared header `src/stm32/kalico_h7_timer.h`**

```c
// src/stm32/kalico_h7_timer.h
//
// Shared declarations for the kalico H7 TIM5 ISR + bench buffer. Included by
// both src/stm32/kalico_h7_timer.c (defines the storage) and src/runtime_tick.c
// (drives the bench command).

#ifndef KALICO_H7_TIMER_H
#define KALICO_H7_TIMER_H

#include <stdint.h>

#define KALICO_BENCH_MAX_SAMPLES 1024

extern volatile uint32_t kalico_bench_samples_buf[KALICO_BENCH_MAX_SAMPLES];
extern volatile uint16_t kalico_bench_count;
extern volatile uint16_t kalico_bench_target;
extern volatile uint8_t  kalico_bench_isolate;

void kalico_h7_timer_init(void);
void kalico_h7_enable_tim5(void);
void kalico_h7_disable_tim5(void);
uint32_t kalico_h7_read_cyccnt(void);

#endif // KALICO_H7_TIMER_H
```

- [ ] **Step 2: Write the C source**

```c
// src/stm32/kalico_h7_timer.c
//
// H723-specific TIM5 init + IRQ handler. Spec §2.4 / §4.1 / §4.2 / §4.4.

#include "autoconf.h"
#include "armcm_boot.h"        // DECL_ARMCM_IRQ
#include "board/internal.h"    // STM32-internal helpers — TIM5, RCC, DWT
#include "kalico_runtime.h"
#include "kalico_h7_timer.h"   // shared bench buffer + helper sigs

#if CONFIG_KALICO_RUNTIME && CONFIG_MACH_STM32H7

extern const uint32_t kalico_clock_freq;

extern void* kalico_rt_handle;   // exposed in src/runtime_tick.c

void
kalico_h7_disable_tim5(void)
{
    TIM5->CR1 &= ~TIM_CR1_CEN;
    NVIC_DisableIRQ(TIM5_IRQn);
}

// Helper for Rust's CYCCNT widen-reinit on producer-driven re-enable path.
uint32_t
kalico_h7_read_cyccnt(void)
{
    return DWT->CYCCNT;
}

void
kalico_h7_enable_tim5(void)
{
    TIM5->SR = ~TIM_SR_UIF;       // clear stale UIF before enabling
    TIM5->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM5_IRQn);
}

void
kalico_h7_timer_init(void)
{
    // Init invariant (spec §2.4): MUST disable + clear before any path could fire.
    TIM5->CR1 &= ~TIM_CR1_CEN;
    TIM5->SR = 0;
    NVIC_DisableIRQ(TIM5_IRQn);

    // Enable TIM5 clock.
    RCC->APB1LENR |= RCC_APB1LENR_TIM5EN;

    // 40 kHz tick: PSC = 0, ARR = (clock_freq / 40000) - 1.
    TIM5->PSC = 0;
    TIM5->ARR = (kalico_clock_freq / 40000U) - 1U;

    // Auto-reload, update interrupt enable.
    TIM5->CR1 = TIM_CR1_ARPE;
    TIM5->DIER = TIM_DIER_UIE;

    // Enable DWT cycle counter for raw_cyccnt reads in the ISR.
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // Set IRQ priority 3 (Cortex-M: lower number = higher urgency).
    // Below SysTick (2) and USB (1) per spec §2.4.
    NVIC_SetPriority(TIM5_IRQn, 3);

    // Don't enable yet — runtime_init pushes segments first; first push triggers
    // kalico_h7_enable_tim5() via the producer protocol.
}

// Cycle-count bench buffer storage. Declared `extern` in kalico_h7_timer.h
// so src/runtime_tick.c's bench command can read it. SAFETY: only this ISR
// writes; foreground reads after observing `count == target`.
volatile uint32_t kalico_bench_samples_buf[KALICO_BENCH_MAX_SAMPLES];
volatile uint16_t kalico_bench_count = 0;
volatile uint16_t kalico_bench_target = 0;
volatile uint8_t  kalico_bench_isolate = 0;

void
TIM5_IRQHandler(void)
{
    TIM5->SR = ~TIM_SR_UIF;            // entry-time ack (spec §2.4)
    uint32_t before = DWT->CYCCNT;
    if (kalico_rt_handle) {
        kalico_runtime_tick(kalico_rt_handle, before);
    }
    uint32_t after = DWT->CYCCNT;

    // Bench capture (Task 27). Wraps subtract correctly modulo 2^32.
    if (kalico_bench_count < kalico_bench_target) {
        kalico_bench_samples_buf[kalico_bench_count] = after - before;
        kalico_bench_count++;
    }
    // No late ack.
}

// Klipper's IRQ vector-table dispatch is generated by scripts/buildcommands.py
// from DECL_ARMCM_IRQ entries. Without this, TIM5_IRQHandler will not be wired
// into the vector table and the IRQ silently drops.
DECL_ARMCM_IRQ(TIM5_IRQHandler, TIM5_IRQn);

#endif // CONFIG_KALICO_RUNTIME && CONFIG_MACH_STM32H7
```

(Note the `kalico_rt_handle` reference — `src/runtime_tick.c` from Task 22 exposes a global. Update Task 22 to define `void* kalico_rt_handle = 0;` at file scope, set in `runtime_init`.)

- [ ] **Step 2: Verify `kalico_rt_handle` is already exported by Task 22**

Task 22 already declares `void* kalico_rt_handle = 0;` at file scope and sets it in `runtime_init`. No additional changes needed in `runtime_tick.c`.

- [ ] **Step 3: Commit**

```bash
git add src/stm32/kalico_h7_timer.c src/runtime_tick.c
git commit -m "src/stm32/kalico_h7_timer: TIM5 init + IRQ handler (Step 5)

Spec §2.4. TIM5 (32-bit, unused by Klipper core or hard_pwm.c on
Octopus Pro), NVIC priority 3, ARR = clock_freq/40000 − 1, DWT
CYCCNT enabled for raw_cyccnt reads. Init invariant: clear CR1.CEN +
SR.UIF + NVIC before any path could fire — guards against warm-reboot
early-fire during INITING. enable/disable helpers exposed to Rust
for the producer-side push protocol (§4.4)."
```

### Task 24: Update `src/Makefile` and `src/Kconfig`

**Files:**
- Modify: `src/Makefile`, `src/Kconfig`, `src/stm32/Makefile`, `src/stm32/Kconfig`

- [ ] **Step 1: Add Kconfig option in `src/Kconfig`**

```
config KALICO_RUNTIME
    bool "Enable kalico Rust runtime (Layer 4 motion planner)"
    depends on MACH_STM32H7
    default n
    help
      Links libkalico_c_api.a (Rust staticlib) and enables the
      40 kHz trajectory-evaluation ISR. Step 5 — see CLAUDE.md.
```

- [ ] **Step 2: Add the C sources and link config to `src/stm32/Makefile`**

The Klipper STM32 Makefile uses `CFLAGS_klipper.elf` for both compile flags AND extra link inputs (verified at `src/stm32/Makefile:35` — line ends with `-nostdlib -lgcc -lc_nano`). Append the staticlib to that variable, ensuring it lands AFTER the C objects but BEFORE `-lgcc -lc_nano` for archive-extraction order:

```makefile
src-$(CONFIG_KALICO_RUNTIME) += stm32/kalico_h7_timer.c
src-$(CONFIG_KALICO_RUNTIME) += runtime_tick.c

# Link libkalico_c_api.a. Place ahead of -lgcc/-lc_nano on the link line —
# archive extraction is demand-driven and order-sensitive (spec §2.4 link-line
# pitfalls). Empirically, prepending to CFLAGS_klipper.elf works because
# Klipper's link rule is `$(CC) $(OBJS) $(CFLAGS_klipper.elf) -o $@` and
# the existing -lgcc tail stays at the end.
ifeq ($(CONFIG_KALICO_RUNTIME),y)
KALICO_LIB := rust/target/thumbv7em-none-eabihf/release/libkalico_c_api.a
CFLAGS_klipper.elf := $(KALICO_LIB) $(CFLAGS_klipper.elf)
$(OUT)klipper.elf: $(KALICO_LIB)
endif
```

- [ ] **Step 3: Add a Cargo build hook**

The Klipper Makefile needs to invoke the Rust build before linking. Add to `src/Makefile`:

```makefile
ifeq ($(CONFIG_KALICO_RUNTIME),y)
$(LIBS_klipper.elf): rust/target/thumbv7em-none-eabihf/release/libkalico_c_api.a

rust/target/thumbv7em-none-eabihf/release/libkalico_c_api.a: FORCE
	cd rust && cargo build -p kalico-c-api --no-default-features \
		--features mcu-h7,header-nurbs,header-runtime \
		--target thumbv7em-none-eabihf --release
.PHONY: FORCE
endif
```

- [ ] **Step 4: Run a test build with the Kconfig enabled**

```bash
make CONFIG_MACH_STM32H7=y CONFIG_KALICO_RUNTIME=y 2>&1 | tail -20
```

Expected: clean build of klipper.elf with the Rust staticlib linked. Likely failures at this stage:
- Rust build path needs adjustment (worktree-relative).
- ABI mismatches (hard-float vs soft-float) — verify `mfloat-abi=hard` in Klipper's H7 CFLAGS matches Rust's `thumbv7em-none-eabihf`.
- `libgcc` symbol overlap with Rust's `compiler_builtins` — investigate; usually harmless but requires `-Wl,--allow-multiple-definition` in worst case.

Debug iteratively; this is the integration crucible.

- [ ] **Step 5: Commit**

```bash
git add src/Makefile src/Kconfig src/stm32/Makefile
git commit -m "src/stm32: wire Rust kalico-c-api staticlib into Klipper build

Spec §2.4. Kconfig CONFIG_KALICO_RUNTIME (default n; only meaningful
on STM32H7). Cargo build hook ensures libkalico_c_api.a is fresh
before Klipper link. Link order: C objects → libkalico_c_api.a →
-lgcc -lc_nano per spec linker-pitfall guidance."
```

---

## Phase 7: CI matrix updates

### Task 25: Update CI workflow

**Files:**
- Modify: `.github/workflows/*.yml` (or whichever CI file the repo uses)

**Why:** Spec §6.6 — host tests + dual MCU builds (mcu-h7 + mcu-f4) + clippy + miri + cbindgen drift + cargo deny + LLVM-IR panic-symbol grep.

- [ ] **Step 1: Identify the CI file**

```bash
ls .github/workflows/ 2>/dev/null
```

- [ ] **Step 2: Add the spec's CI checks**

(Pseudocode; adapt to the repo's actual CI YAML idioms.)

```yaml
jobs:
  rust-host:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - run: cd rust && cargo build --workspace
      - run: cd rust && cargo test --workspace
      - run: cd rust && cargo clippy --workspace --all-targets -- -D warnings
      - run: cd rust && cargo fmt --all -- --check

  rust-mcu-h7:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - run: rustup target add thumbv7em-none-eabihf
      - run: cd rust && cargo build -p kalico-c-api --no-default-features
              --features mcu-h7,header-nurbs,header-runtime
              --target thumbv7em-none-eabihf

  rust-mcu-f4:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - run: rustup target add thumbv7em-none-eabihf
      - run: cd rust && cargo build -p kalico-c-api --no-default-features
              --features mcu-f4,header-nurbs,header-runtime
              --target thumbv7em-none-eabihf

  rust-cbindgen-drift:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - run: cd rust && cargo run -p kalico-c-api --bin gen-headers
      - run: git diff --exit-code rust/kalico-c-api/include/

  rust-c-smoke:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - run: cd rust && cargo build -p kalico-c-api --release
      - run: cd rust && cargo test -p kalico-c-api --features host,header-runtime --test c_smoke_build

  rust-deny:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: EmbarkStudios/cargo-deny-action@v2
        with:
          arguments: --manifest-path rust/Cargo.toml check

  rust-miri:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - run: rustup +nightly component add miri
      - run: cd rust && cargo +nightly miri test -p runtime --features host

  rust-panic-symbol-grep:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - run: rustup target add thumbv7em-none-eabihf
      - run: cd rust && cargo rustc -p kalico-c-api --release
              --no-default-features --features mcu-h7,header-runtime
              --target thumbv7em-none-eabihf -- --emit=llvm-ir
      - run: |
          if grep -E 'core::panicking|panic_bounds_check' rust/target/thumbv7em-none-eabihf/release/deps/*.ll; then
            echo "Panic symbols found in MCU build — review src/runtime/src/*.rs for bounds-check or division-by-zero paths"
            exit 1
          fi

  watchdog-canary:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - run: grep -F 'kalico_liveness_ok' src/stm32/watchdog.c
      - run: grep -F 'CONFIG_KALICO_RUNTIME' src/stm32/watchdog.c
```

- [ ] **Step 3: Push and verify CI runs green**

```bash
git push origin sota-motion
# wait for CI; check workflow run status
```

If anything fires:
- clippy: fix the offending Rust source.
- miri: investigate the UB (likely an unsafe-block invariant violation).
- panic-symbol-grep: refactor the offending function to use `Result` returns.
- cbindgen-drift: regenerate headers locally and commit.
- watchdog-canary: re-add the gate.

- [ ] **Step 3.5: Create concrete `rust/deny.toml` for the cargo-deny job**

```toml
# rust/deny.toml — cargo-deny policy for Step-5 dependency graph.
# Spec §6.6 quality gate; supply-chain audit on heapless + transitive deps.
# Schema for cargo-deny ≥0.16 (pre-0.16 used `vulnerability=`, `copyleft=`,
# `default=` keys which are now deprecated; we use the current allow-list shape).

[advisories]
db-path = "~/.cargo/advisory-db"
db-urls = ["https://github.com/RustSec/advisory-db"]
yanked = "deny"

[licenses]
# Allow-list approach — anything not listed is denied by default in current
# cargo-deny. The kalico Rust crates can ship under permissive licenses
# (MIT/Apache-2.0); GPL-compat is preserved at the link-line level when
# compiled into Klipper (Klipper itself is GPLv3, and permissive Rust deps
# are GPL-compatible per the GNU GPL FAQ).
allow = [
    "MIT", "Apache-2.0", "Apache-2.0 WITH LLVM-exception",
    "BSD-3-Clause", "ISC", "Unicode-DFS-2016", "Unicode-3.0", "Zlib",
]

[bans]
multiple-versions = "warn"
deny = []   # extend as crate-specific rejections surface

[sources]
unknown-registry = "deny"
unknown-git = "deny"
allow-registry = ["https://github.com/rust-lang/crates.io-index"]
```

The GPLv3 inheritance question (Codex round-2 point 8): the kalico Rust workspace ships permissive (MIT/Apache-2.0) per crate. When `libkalico_c_api.a` links into a GPLv3 Klipper build, the *combined* binary is GPL-governed per the GNU GPL FAQ on linking — but each Rust crate stays permissive at the source level, which is what cargo-deny enforces. No GPLv3 entries needed in the allow-list because Klipper isn't a Cargo dependency.

- [ ] **Step 4: Commit**

```bash
git add .github/workflows/*.yml rust/deny.toml
git commit -m "ci: add Step-5 CI matrix (host + MCU + drift + miri + panic-grep)

Spec §6.6. host: build/test/clippy/fmt. MCU: dual-target H7 + F4
build (no run). cbindgen drift detection on every PR. C smoke build.
cargo deny for license/security audit. miri on UnsafeCell paths.
LLVM-IR panic-symbol grep proves Engine::tick is panic-free in the
release build. Watchdog canary asserts the kalico_liveness_ok gate
survived any Klipper rebase."
```

---

## Phase 8: Surface C bring-up

### Task 25.5: Host-side communication helper (`tools/kalico_host_io.py`)

**Files:**
- Create: `tools/kalico_host_io.py`

**Why:** Klipper's `sendf` produces **binary VLQ-encoded message blocks** framed by 0x7E sync bytes with CRC trailers — NOT ASCII newline-terminated lines. Round-2 and round-3 host scripts incorrectly assumed ASCII framing; round-4 review (Verifier) confirmed by reading `src/command.{c,h}` and `klippy/msgproto.py`. Surface-C scripts (Tasks 26–29) must communicate through Klipper's existing host-side msgproto library, not raw `pyserial`.

This helper module wraps the integration so each Surface-C script doesn't have to re-implement it.

- [ ] **Step 1: Write the helper module**

```python
# tools/kalico_host_io.py
#
# Klipper-compatible host-side I/O for kalico Surface-C tests.
# Wraps klippy/msgproto.py + serialhdl.py to provide a small async API for
# sending commands and collecting replies.
#
# Klipper's serial protocol is binary VLQ; the data dictionary embedded in
# firmware (zlib-compressed JSON) is downloaded on connect and binds command
# names ↔ IDs. This helper does that handshake for us.

import os, sys, json, struct, time, zlib, threading
from queue import Queue, Empty

# Import Klipper's existing host-side library (run scripts from kalico repo root).
KLIPPER_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(KLIPPER_ROOT, "klippy"))
import msgproto
import serialhdl, reactor

class KalicoHostIO:
    """Thin wrapper around klippy.serialhdl.SerialReader for kalico tests.

    Connects to the MCU, downloads its data dictionary, exposes:
      - `send(cmd_str)`: queue a command for transmission.
      - `wait_for_response(name, timeout)`: block until a response with the
         given message name arrives; returns the parsed message params.
      - `collect_responses(name, count, timeout)`: collect N responses.

    All blocking calls return parsed Python dicts (not raw bytes).
    """

    def __init__(self, port: str, baud: int = 250000):
        self._reactor = reactor.Reactor()
        self._serial = serialhdl.SerialReader(self._reactor, "kalico_test")
        self._serial.connect_uart(port, baud, rts=False)
        self._serial.handle_default = self._handle_response
        self._responses: dict[str, Queue] = {}

    def _handle_response(self, params):
        name = params['#name']
        self._responses.setdefault(name, Queue()).put(params)

    def send(self, cmd_str: str):
        """Send a command using Klipper's `lookup_command`-then-`send` pattern."""
        msgparser = self._serial.get_msgparser()
        cmd = msgparser.create_command(cmd_str)
        self._serial.send(cmd)

    def wait_for_response(self, name: str, timeout: float = 2.0) -> dict:
        q = self._responses.setdefault(name, Queue())
        try:
            return q.get(timeout=timeout)
        except Empty:
            raise TimeoutError(f"no '{name}' response within {timeout} s")

    def collect_responses(self, name: str, count: int, timeout: float = 30.0) -> list[dict]:
        deadline = time.monotonic() + timeout
        out = []
        while len(out) < count:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(
                    f"only {len(out)}/{count} '{name}' responses in {timeout} s")
            out.append(self.wait_for_response(name, timeout=remaining))
        return out

    def disconnect(self):
        self._serial.disconnect()
```

- [ ] **Step 2: Verify the helper compiles + handshakes against a flashed MCU**

```bash
python3 -c "
import sys; sys.path.insert(0, 'tools')
from kalico_host_io import KalicoHostIO
io = KalicoHostIO('/dev/ttyACM0')
io.send('kalico_query_status')
print(io.wait_for_response('kalico_status'))
io.disconnect()
"
```

Expected: prints a dict like `{'#name': 'kalico_status', 'status': 0, 'last_err': 0}`.

- [ ] **Step 3: Commit**

```bash
git add tools/kalico_host_io.py
git commit -m "tools/kalico_host_io: msgproto-based host helper (Step 5 Surface C)

Klipper's sendf is binary VLQ-framed, NOT ASCII newline-terminated.
Surface-C tests need msgproto-aware I/O that downloads the data
dictionary at connect time and binds command names. This helper
wraps klippy/serialhdl.py + msgproto.py so each test script reads
parsed Python dicts instead of decoding the wire format itself."
```

### Task 26: First-light test (LED toggle on idle flip)

**Files:**
- Possibly modify: `src/runtime_tick.c` to add a debug LED toggle.
- Create: `tools/test_h723_first_light.py` (host-side validation)

**Why:** Spec §6.4 — first-light validates ISR fires + Rust call works at all. Smallest possible test signal: status flip drives an LED.

- [ ] **Step 1: Identify the LED GPIO on Octopus Pro**

The Octopus Pro has a status LED on PA13 (or similar — verify against the BTT documentation / pin map). Use Klipper's existing GPIO infrastructure rather than touching the chip directly.

- [ ] **Step 2: Add LED-toggle hook to `runtime_drain`**

In `src/runtime_tick.c`:

```c
#include "board/gpio.h"   // gpio_out_setup, gpio_out_toggle

static struct gpio_out led_pin;

void runtime_init(void) {
    led_pin = gpio_out_setup(GPIO('A', 13), 0);
    // ... existing init ...
}

void runtime_drain(void) {
    // ... existing drain logic ...
    static uint8_t last_status = 0xFF;
    uint8_t cur = kalico_runtime_status(kalico_rt_handle);
    if (cur != last_status) {
        gpio_out_toggle(led_pin);
        last_status = cur;
    }
}
```

- [ ] **Step 3: Write the host-side first-light script (uses Task 25.5 helper)**

```python
#!/usr/bin/env python3
# tools/test_h723_first_light.py
#
# Spec §6.4 first-light. Validates ISR fires + Rust call works at all by
# observing status transitions in response to a synthetic segment push.

import sys, time
sys.path.insert(0, 'tools')
from kalico_host_io import KalicoHostIO

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'

io = KalicoHostIO(PORT)
try:
    # Initial status — should be IDLE (0) immediately after flash.
    io.send('kalico_query_status')
    initial = io.wait_for_response('kalico_status', timeout=2.0)
    if initial['status'] != 0:
        print(f"FAIL: expected status=IDLE (0), got {initial['status']}")
        sys.exit(1)

    # Load a tiny straight-line curve and push one segment to force RUNNING.
    # ... (load_curve + push_segment via io.send) ...
    io.send('kalico_load_curve slot=0 degree=1 n_cp=2 n_knots=4 '
            'cps=0.0,0.0,0.0,10.0,0.0,0.0 knots=0,0,1,1 weights=1,1')
    io.wait_for_response('kalico_load_curve_response', timeout=2.0)

    # Push a 5 ms segment.
    io.send('kalico_push_segment id=1 curve=0 t_start_hi=0 t_start_lo=0 '
            't_end_hi=0 t_end_lo=2600000 kinematics=0')
    io.wait_for_response('kalico_push_response', timeout=2.0)

    # Wait briefly for ISR to flip status to RUNNING.
    time.sleep(0.05)
    io.send('kalico_query_status')
    running = io.wait_for_response('kalico_status', timeout=2.0)
    if running['status'] != 1:
        print(f"FAIL: expected status=RUNNING (1) after push, got {running['status']}")
        sys.exit(1)

    print(f"PASS: status transitioned IDLE→RUNNING; LED should have toggled")
finally:
    io.disconnect()
```

- [ ] **Step 4: Verify on hardware** (manual; this is Surface C)

Flash → reset → run the python script → observe LED toggles when status flips. **PASS** if LED visibly responds; **FAIL** if no LED activity, missing status response, or USB-CDC enumeration fails.

- [ ] **Step 5: Commit**

```bash
git add src/runtime_tick.c tools/test_h723_first_light.py
git commit -m "test/h723: first-light LED toggle on status flip (Step 5 Surface C)

Spec §6.4. Validates the full pipeline: ISR fires at 40 kHz → Rust
runtime ticks → status updates propagate to foreground → drain task
toggles GPIO. Smallest possible end-to-end signal."
```

### Task 27: Cycle-count instrumentation (Pass A + Pass B)

**Files:**
- Modify: `src/runtime_tick.c` (add CYCCNT bracketing if needed)
- Possibly: Rust-side instrumentation in `Engine::tick` gated by a feature

**Why:** Spec §6.4 — measure both isolated and production-conditions cycle counts.

- [ ] **Step 1: Add a CYCCNT-bracketed micro-bench mode** to `src/runtime_tick.c`:

```c
#if CONFIG_KALICO_BENCH
static uint32_t bench_min = UINT32_MAX, bench_max = 0;
static uint64_t bench_sum = 0;
static uint32_t bench_count = 0;
// in TIM5_IRQHandler, around the kalico_runtime_tick call: bracket with DWT->CYCCNT.
#endif
```

(Iterate on Surface C until both Pass A and Pass B numbers fit the budget; document in spec §4.8 as actuals.)

- [ ] **Step 2: Write the cycle-count script with programmatic gate**

`tools/test_h723_cycle_count.py`:

```python
#!/usr/bin/env python3
"""Measure Engine::tick cycle cost via DWT CYCCNT instrumentation.

Two passes:
  - Pass A (isolated): IRQ_FENCE=on; non-kalico IRQs masked during measurement.
  - Pass B (production): full IRQs enabled; mimics real load.

Reports min/p50/p99 for each pass; FAILs if Pass-B p99 exceeds budget.
"""
import argparse, sys, json, statistics, time
sys.path.insert(0, 'tools')
from kalico_host_io import KalicoHostIO

p = argparse.ArgumentParser()
p.add_argument("port", help="USB-CDC device, e.g. /dev/ttyACM0")
p.add_argument("--p99-budget-us", type=float, default=15.0,
               help="Acceptance threshold; Pass-B p99 must be below this.")
p.add_argument("--samples", type=int, default=10_000,
               help="Sample count per pass (defaults match spec §6.4).")
args = p.parse_args()

io = KalicoHostIO(args.port)

def collect(io: KalicoHostIO, pass_name: str, isolate: int) -> list[int]:
    """Run one bench pass via msgproto. MCU emits one `kalico_bench_sample`
    response per measurement, then a final `kalico_bench_done`."""
    io.send(f'kalico_bench_run isolate={isolate} samples={args.samples}')
    # Effective sample count after warmup skip:
    expected = args.samples - 8  # WARMUP_SKIP
    samples = [s['value'] for s in io.collect_responses(
        'kalico_bench_sample', count=expected, timeout=60.0)]
    done = io.wait_for_response('kalico_bench_done', timeout=5.0)
    if done.get('error', -1) != 0:
        print(f"FAIL: bench done error={done.get('error')} reason={done.get('reason','?')}"
              f" in pass {pass_name}")
        sys.exit(1)
    return samples

CLOCK_FREQ = 520_000_000  # H723 default Kconfig

def stats(samples: list[int]) -> dict:
    samples_us = [s * 1_000_000 / CLOCK_FREQ for s in samples]
    return {
        "min_us": min(samples_us),
        "p50_us": statistics.median(samples_us),
        "p99_us": statistics.quantiles(samples_us, n=100)[98],
    }

try:
    a = stats(collect(io, "A_isolated", isolate=1))
    b = stats(collect(io, "B_production", isolate=0))
finally:
    io.disconnect()

print(json.dumps({"pass_a": a, "pass_b": b}, indent=2))

if b["p99_us"] > args.p99_budget_us:
    print(f"FAIL: Pass-B p99 = {b['p99_us']:.2f} µs > budget {args.p99_budget_us} µs")
    sys.exit(1)
print(f"PASS: Pass-B p99 = {b['p99_us']:.2f} µs (budget {args.p99_budget_us} µs)")
```

This makes the acceptance threshold a programmatic CI-able gate, not a manual review item. The MCU side (Task 22 + Task 23) implements `kalico_bench_run` end-to-end: per-sample `kalico_bench_sample` responses (Klipper-standard binary VLQ wire format), final `kalico_bench_done count=<N> error=<code>`. Host-side parses via `tools/kalico_host_io.py` (Task 25.5).

- [ ] **Step 3: Run the gate and document the result**

```bash
make test-h723 FLASH_DEVICE=0483:df11 SERIAL_PORT=/dev/ttyACM0
# Inspect target/h723-test-$(git rev-parse --short HEAD)/cycle_count.log
# Record results in docs/research/step5-h723-cycle-budget.md (template:
# date | git SHA | clock freq | Pass A min/p50/p99 | Pass B min/p50/p99 | budget | result)
```

- [ ] **Step 4: Commit**

```bash
git add docs/research/step5-h723-cycle-budget.md tools/test_h723_cycle_count.py src/runtime_tick.c
git commit -m "test/h723: cycle-count Pass A+B measurement with programmatic gate

Spec §6.4 / §4.8. Pass A (isolated): X cycles min, Y p50, Z p99.
Pass B (USB+USART concurrent): X cycles min, Y p50, Z p99.
Acceptance: Pass-B p99 < 15 µs (60% of 25 µs tick budget) — script
asserts via --p99-budget-us 15 and exits non-zero on breach. Result: PASS."
```

### Task 28: Trace-dump host script — uses Task-17a's shared fixture

**Files:**
- Create: `tools/test_h723_trace_dump.py`

(The fixture itself, `rust/runtime/tests/fixtures/step5_segments.json`, was created in Task 17a per spec §6.7. Task 28 builds the host-side validator that consumes it.)

- [ ] **Step 1: Write the host script**

```python
#!/usr/bin/env python3
"""Drive runtime over USB-CDC, drain trace, plot against analytical eval.

Reads the shared fixture file (rust/runtime/tests/fixtures/step5_segments.json),
loads each curve into the MCU's CurvePool slab, pushes a chained segment
sequence, drains the trace ring, validates trace continuity + position-error
bound against an analytical Python NURBS evaluator (e.g., scipy.interpolate
or a from-scratch de Boor for parity).

Acceptance: max |motor_pos_traced - motor_pos_analytical| < 0.05 mm
across all fixture chains.
"""
import argparse, json, sys, struct, serial, time
# ... full implementation ...
```

(Full Python implementation: ~150 lines. The skeleton above shows shape; the implementer writes details using `pyserial` for CDC and matplotlib for the optional plot output. Acceptance threshold of 0.05 mm = 50 µm matches the sub-tick-boundary motion-loss budget at 1000 mm/s × 25 µs = 25 µm; doubled for measurement noise.)

- [ ] **Step 2: Run on hardware, iterate, commit**

```bash
make test-h723 FLASH_DEVICE=0483:df11 SERIAL_PORT=/dev/ttyACM0
git add tools/test_h723_trace_dump.py
git commit -m "test/h723: trace-dump host validator (Step 5 Surface C)

Spec §6.4 / §6.7. Loads shared step5_segments.json, drives runtime
over USB-CDC, drains trace, validates against analytical NURBS eval
with 50 µm position-error budget. PASS/FAIL programmatic gate via
non-zero exit code."
```

### Task 29: 30-min soak + `make test-h723` script

**Files:**
- Create: `Makefile.kalico` (or extend existing) with `test-h723` target

- [ ] **Step 1: Write the orchestration script**

```makefile
# Use Klipper's flash_usb.py — it handles bootloader entry, ModemManager
# inhibition, USB udev permissions, port discovery. Manual dfu-util
# invocations break in too many environment-specific ways.

test-h723:
	@echo "=== Step 5 Surface-C: flash + bring-up + soak ==="
	@SHA=$$(git rev-parse --short HEAD); \
	  ARTIFACTS_DIR="target/h723-test-$$SHA"; \
	  mkdir -p $$ARTIFACTS_DIR; \
	  echo "Artifacts → $$ARTIFACTS_DIR"; \
	  $(PYTHON) ./scripts/flash_usb.py -t stm32h723xx -d "$(FLASH_DEVICE)" -s 0x8000000 out/klipper.bin > $$ARTIFACTS_DIR/flash.log 2>&1 || \
	    { cat $$ARTIFACTS_DIR/flash.log; exit 1; }; \
	  sleep 3; \
	  python3 tools/test_h723_first_light.py "$(SERIAL_PORT)" \
	      | tee $$ARTIFACTS_DIR/first_light.log || exit 1; \
	  python3 tools/test_h723_cycle_count.py "$(SERIAL_PORT)" \
	      --p99-budget-us 15 \
	      | tee $$ARTIFACTS_DIR/cycle_count.log || exit 1; \
	  python3 tools/test_h723_trace_dump.py "$(SERIAL_PORT)" \
	      --fixture rust/runtime/tests/fixtures/step5_segments.json \
	      | tee $$ARTIFACTS_DIR/trace_dump.log || exit 1; \
	  python3 tools/test_h723_soak.py "$(SERIAL_PORT)" --minutes 30 \
	      | tee $$ARTIFACTS_DIR/soak.log || exit 1; \
	  echo "=== Surface-C PASS — artifacts in $$ARTIFACTS_DIR ==="
```

Each Python script must emit `PASS` or `FAIL` and exit non-zero on failure; `tee` preserves output for archiving while the underlying exit code propagates. The cycle-count script programmatically asserts `p99 < 15 µs` (per `--p99-budget-us 15` argument); Task 27 above is responsible for producing that script with the threshold check baked in. `FLASH_DEVICE` and `SERIAL_PORT` are caller-supplied (`make test-h723 FLASH_DEVICE=0483:df11 SERIAL_PORT=/dev/ttyACM0`).

- [ ] **Step 2: Verify the script runs and saves artifacts under the build SHA**

- [ ] **Step 3: Commit**

```bash
git add Makefile.kalico tools/test_h723_*.py
git commit -m "test/h723: scriptable bring-up gate with SHA-tagged artifacts

Spec §6.5 — make test-h723 flashes, runs first-light + cycle-count +
trace-dump + soak, captures USB-CDC output with PASS/FAIL marker
under target/h723-test-$(git rev-parse HEAD).log. Reproducibility
gap closed; manual hardware tests now anchored to a SHA."
```

---

## Phase 9: Documentation + plan-changes-log

### Task 30: Update CLAUDE.md plan-changes-log

**Files:**
- Modify: `docs/superpowers/plan-changes-log.md`

- [ ] **Step 1: Append the Step 5 completion entry**

```markdown
---

**Build-order Step 5 (MCU framework with stub NURBS evaluator): completed.** Implementation per `docs/superpowers/plans/2026-04-28-layer-4-mcu-framework-stub.md`. New `runtime/` no_std crate ships per-axis Engine state machine; `kalico-c-api/` (renamed from `nurbs-c-api/`) is the umbrella staticlib with two cbindgen headers. 40 kHz TIM5 ISR validated on H723; ~X µs per-tick measured (Pass-B p99). Trace-only output stage; runtime-eval slots for PA (Step 9) and IS (Step 8) are ZST `Noop` impls. Pre-flight: workspace migrated to Rust 2024 edition.

**Deviations from plan listing** (worth recording so spec/plan can be amended in lockstep):

(Fill in any spec-vs-implementation deltas surfaced during execution. Common: cycle budget reality, F4x build status, deferred items.)

**Open follow-ups (non-blocking; tracked here so they don't get lost):**
- Loom test coverage expansion (gated to Step 6 when live producer surfaces).
- Layer 3 time-reparameterization (gating Step 7 MVP — see spec §7 open question 7).
- F4x integration test (gating Step 6 multi-MCU bring-up).
- TanhPa slot velocity dependency (Step 9 design time decides TickState shape).
- Klipper full-LTO link CI (Step 7 MVP CI work).

**Evidence:** Plan + N commits on this branch. Spec at `docs/superpowers/specs/2026-04-28-layer-4-mcu-framework-stub-design.md`. Surface-C cycle-budget result: `docs/research/step5-h723-cycle-budget.md`.
```

- [ ] **Step 2: Tick build-order Step 5 in `CLAUDE.md`**

Find:
```
5. [ ] **MCU framework with stub NURBS evaluator and basic kinematics**
```

Replace with:
```
5. [x] **MCU framework with stub NURBS evaluator and basic kinematics**
```

- [ ] **Step 3: Commit**

```bash
git add CLAUDE.md docs/superpowers/plan-changes-log.md
git commit -m "claude.md: tick build-order Step 5; plan-changes-log entry"
```

---

## Self-Review Checklist

After completing all phases:

- [ ] **Spec coverage:** every non-goal in spec §1.2 confirmed unimplemented. Every locked decision in spec §9 reflected in code.
- [ ] **Tests green:** `cd rust && cargo test --workspace --features host` passes; clippy `-D warnings` clean.
- [ ] **CI matrix green:** all Phase-7 jobs passing on the PR.
- [ ] **Surface C pass:** `make test-h723` exits with PASS; artifacts archived under git SHA.
- [ ] **Cycle budget within 15 µs Pass-B p99.**
- [ ] **No `panic` symbols in MCU LLVM IR.**
- [ ] **CLAUDE.md ticked + plan-changes-log appended.**

If any item fails, do not declare Step 5 complete — fix and re-run.

---

## Execution Handoff

**Plan complete and saved to `docs/superpowers/plans/2026-04-28-layer-4-mcu-framework-stub.md`. Two execution options:**

**1. Subagent-Driven (recommended)** — I dispatch a fresh subagent per task, review between tasks, fast iteration.

**2. Inline Execution** — Execute tasks in this session using executing-plans, batch execution with checkpoints.

**Which approach?**
