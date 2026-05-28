# Segment-Era Dead-Module Removal Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. All Rust work should be executed via the `rust-engineer` subagent (project rule).

**Goal:** Delete the segment/curve-pool–era modules (`segment`, `reclaim`, `trace`, `c_segment_queue`, `queue`, `modulator`, `config`) that survived the piece-ring engine rewrite but remain wired into `RuntimeContext`/`engine`/`tick` and the C↔Rust FFI; reduce `stream` to a `flush`-only shell.

**Architecture:** Removal-driven, not test-driven — there is no new behavior. The safety net is the existing workspace test suite + `cargo build` + clippy staying green, plus the MCU firmware building for both targets. Work proceeds in four clusters (one commit each), leaf-first within each cluster: remove consumers (dead tests → C `DECL_COMMAND`s + handlers → FFI exports → struct fields) before deleting the module definitions, so the compiler enumerates every remaining reader and nothing is missed.

**Tech Stack:** Rust (`rust/` cargo workspace, `runtime` + `kalico-c-api` crates), C MCU firmware (`src/`), `cargo`/`clippy` on host, bench firmware build on the Pi.

**Spec:** `docs/superpowers/specs/2026-05-28-segment-era-dead-module-removal-design.md`

---

## Conventions used in every task

- **"Build green" gate** (host) — run from `rust/`:
  ```bash
  cargo build --workspace && cargo test --workspace && cargo clippy --workspace -- -D warnings
  ```
  Expected: all compile, all tests pass, no clippy errors. The `runtime` crate denies `panic`/`unwrap`/`expect`/`indexing_slicing`/etc. at the crate root — removals must not introduce violations.
- **"Firmware build" gate** (end of each cluster) — per the bench flow (commit → push → pull → build on the Pi), build **both** MCU targets (H7 from `.config.h7.bak`, F446 from `.config.f446.test`), running `make clean` between them. A C `DECL_COMMAND`/handler removal must not leave a dangling `extern` declaration or an unreferenced static.
- **"Compiler enumerates readers"** — when a step says "remove field X," after deleting the field declaration + its constructor assignment, `cargo build` will error at every remaining reader. Remove each flagged reader (they are all dead segment-era code paths) until the build is green. Do not add shims to keep dead readers alive.
- Deletions are exact: each step names the file and the symbol/line region. Line numbers are from the current `HEAD` and may drift as edits land within a file — match on the symbol name, not the bare line number.

---

## Task 1 (Cluster 1): Remove the unused phase modulator

**Files:**
- Delete: `rust/runtime/src/modulator.rs`
- Delete: `rust/runtime/tests/modulator_math.rs`
- Modify: `rust/runtime/src/lib.rs` (remove `pub mod modulator;`, line 40)
- Modify: `rust/runtime/src/engine.rs` (remove `phase_modulators` field + import + reset)

- [ ] **Step 1: Delete the modulator unit test**

```bash
git rm rust/runtime/tests/modulator_math.rs
```

- [ ] **Step 2: Remove the modulator import and field from `engine.rs`**

In `rust/runtime/src/engine.rs`:
- Delete line 29: `use crate::modulator::PhaseDirectModulator;`
- Delete the field at line 86: `phase_modulators: [Option<PhaseDirectModulator>; MAX_STEPPER_OIDS],`
- Delete the `new` initializer (line 112): `phase_modulators: [const { None }; MAX_STEPPER_OIDS],`
- Delete the `init_in_place` initializer (line 155): `addr_of_mut!((*ptr).phase_modulators).write([const { None }; MAX_STEPPER_OIDS]);`
- In `runtime_force_idle` (around lines 581–583), delete the reset loop:
  ```rust
  for slot in &mut self.phase_modulators {
      *slot = None;
  }
  ```

- [ ] **Step 3: Delete the module file and its `lib.rs` declaration**

```bash
git rm rust/runtime/src/modulator.rs
```
In `rust/runtime/src/lib.rs` delete line 40: `pub mod modulator;`

- [ ] **Step 4: Build green gate**

Run the host build-green gate (see Conventions). Expected: PASS. If `sim_fixtures.rs` or any other file still references `modulator`, the build will flag it — there are no expected references after this task (sim_fixtures uses `config`, handled in Task 2).

- [ ] **Step 5: Firmware build gate, then commit**

Build both MCU targets (see Conventions). Then:
```bash
git add -A
git commit -m "remove: unused PhaseDirectModulator (dead since segment-engine removal)"
```

---

## Task 2 (Cluster 2): Remove the legacy axes-blob config path

The new per-axis path (`kalico_runtime_configure_axis`, `command_kalico_configure_axis`) stays. This removes only the legacy blob path that reaches `runtime::config` + `engine.configure()`.

**Files:**
- Modify (C): `src/runtime_commands.c`, `src/kalico_dispatch.c`
- Modify (FFI): `rust/kalico-c-api/src/runtime_ffi.rs`
- Modify: `rust/runtime/src/engine.rs`, `rust/runtime/src/sim_fixtures.rs`
- Delete: `rust/runtime/src/config.rs`
- Modify: `rust/runtime/src/lib.rs` (remove `pub mod config;`, line 35)

- [ ] **Step 1: Remove the C command + caller**

In `src/runtime_commands.c`: delete the `command_runtime_configure_axes_blob` handler (starts line 422) through its `DECL_COMMAND(command_runtime_configure_axes_blob, ...)` (line 452–453), and the related comment block at lines 457–469 that references it.
In `src/kalico_dispatch.c`: remove the call at line 334 (`kalico_runtime_configure_axes_blob(...)`) and the surrounding dispatch branch that builds `body`/`body_len` for it. If `kalico_dispatch.c` has no other use of that branch's locals, remove them too.
In `src/stepper.c`: update the stale comments at lines 200–201 that reference `kalico_configure_axes_blob` (the function is gone).

- [ ] **Step 2: Remove the FFI export**

In `rust/kalico-c-api/src/runtime_ffi.rs`: delete the entire `kalico_runtime_configure_axes_blob` function (starts line 919; it contains the `use runtime::config::{EMode as _Unused, McuAxisConfig, MotorConfig};` import at 924 and the `engine.configure(cfg)` call at 986). Delete the function up to but not including the next `pub ... extern "C" fn`.

- [ ] **Step 3: Remove `engine.configure()` and `mcu_config`**

In `rust/runtime/src/engine.rs`:
- Delete the `mcu_config` field (line 93), its `new` initializer (line 115: `mcu_config: None,`), and its `init_in_place` initializer (line 158).
- Delete the `configure(&mut self, config: crate::config::McuAxisConfig)` method (lines 543–552).
- **`step_state` audit:** `configure()` was the only writer of `step_state[i].steps_per_mm` via real config. Search for live readers:
  ```bash
  grep -rnE "step_state|StepMotorState|debug_steps_per_mm|debug_accumulator" rust/runtime/src rust/kalico-c-api/src | grep -v "/target/"
  ```
  The live dispatch path (`dispatch_pulse`/`dispatch_phase` in `tick.rs`) uses `axis.microstep_distance`, not `step_state`. If the only remaining `step_state` references are `seed_position`, `runtime_force_idle`, and the `debug_*` accessors (and the `debug_*` accessors are not exported by any kept FFI/C command), remove `step_state`, the three `debug_*` methods, and their callers. If a kept FFI export still reads a `debug_*` accessor, leave `step_state` and those accessors in place and stop here. Record which choice you made in the commit message.

- [ ] **Step 4: Fix `sim_fixtures.rs`**

In `rust/runtime/src/sim_fixtures.rs`:
- Delete the `use crate::config::{McuAxisConfig, MotorConfig};` (line 104) and `use crate::config::EMode;` (line 199).
- Delete the `engine.configure(McuAxisConfig { ... });` call (starts line 120) and the `e_mode: EMode::Travel` usage (line 236) if it belongs to a `config`/`segment` struct literal being removed. (Cluster 3 removes the remaining `segment`/`reclaim`/`stream`/`trace` refs in this file; leave those for now — only remove the `config`-related lines in this task.)

- [ ] **Step 5: Delete the module file and `lib.rs` declaration**

```bash
git rm rust/runtime/src/config.rs
```
In `rust/runtime/src/lib.rs` delete line 35: `pub mod config;`

- [ ] **Step 6: Build green gate**

Run the host build-green gate. Expected: PASS. Remaining `config` references would be flagged — none expected after Steps 1–5.

- [ ] **Step 7: Firmware build gate, then commit**

Build both MCU targets. Then:
```bash
git add -A
git commit -m "remove: legacy configure_axes_blob path + runtime::config (per-axis configure_axis is the only config path)"
```

---

## Task 3 (Cluster 3): Remove segment + trace + reclaim + stream-lifecycle

This is the coupled cluster. `TraceSample` carries `segment::CurveHandle`; `reclaim` is trace's only consumer; the stream open/arm/terminal stubs and the segment-id queries are the remaining FFI surface over these. `flush` survives as a decoupled no-op shell.

**Files:**
- Delete (tests): `rust/kalico-c-api/tests/drain_trace_credit.rs`, `rust/motion-bridge/tests/bridge_to_runtime_step_chain.rs`
- Modify (C): `src/runtime_commands.c`, `src/runtime_tick.c`
- Modify (FFI): `rust/kalico-c-api/src/runtime_ffi.rs`
- Modify: `rust/runtime/src/state.rs`, `rust/runtime/src/engine.rs`, `rust/runtime/src/tick.rs`, `rust/runtime/src/sim_fixtures.rs`, `rust/runtime/src/stream.rs`, `rust/runtime/src/lib.rs`
- Delete: `rust/runtime/src/segment.rs`, `rust/runtime/src/reclaim.rs`, `rust/runtime/src/trace.rs` (+ any `*/tests.rs` siblings)

- [ ] **Step 1: Delete dead cross-crate tests**

```bash
git rm rust/kalico-c-api/tests/drain_trace_credit.rs
git rm rust/motion-bridge/tests/bridge_to_runtime_step_chain.rs
```

- [ ] **Step 2: Remove C stream-lifecycle commands (keep flush) + the reclaim drain call**

In `src/runtime_commands.c`, delete these handlers + their `DECL_COMMAND`s:
- `command_runtime_stream_open` (handler line 295, `DECL_COMMAND` line 308)
- `command_runtime_stream_arm` (handler line 311, `DECL_COMMAND` lines 328–329)
- `command_runtime_stream_terminal` (handler line 332, `DECL_COMMAND` lines 342–343)
- `command_runtime_set_phase_trace` (`DECL_COMMAND` line 407 + its handler) — trace control, dead once the trace ring is gone.
**Keep** `command_runtime_stream_flush` (handler line 346, `DECL_COMMAND` line 358).

In `src/runtime_tick.c`, delete the `kalico_runtime_drain_and_reclaim` call block (lines ~371–420): the `reclaim_status` assignment (line 404), the `fresh_overflow_fault` derivation (line 406), and any downstream use of those two locals. If removing them empties an `if`/helper, remove that too.

- [ ] **Step 3: Remove FFI exports over segment/trace/reclaim/stream-lifecycle**

In `rust/kalico-c-api/src/runtime_ffi.rs`, delete these functions in full (each runs to the next `pub ... extern "C" fn`):
- `runtime_handle_drain_trace` (line 256)
- `kalico_runtime_pending_segment_is_some` (line 398)
- `runtime_handle_credit_epoch` (line 505)
- `runtime_handle_accepted_segment_id` (line 523)
- `runtime_handle_retired_through_segment_id` (line 543)
- `runtime_handle_current_segment_id` (line 659)
- `kalico_runtime_drain_and_reclaim` (line 1262)
- `kalico_runtime_stream_open` (line 1336)
- `kalico_runtime_stream_arm` (line 1362)
- `kalico_runtime_stream_terminal` (line 1389)
- `kalico_runtime_set_phase_trace_enabled` (around line 1863 — trace control)

Also delete the top-of-module `use runtime::segment::KinematicTag;` (line 33).
**Keep** `kalico_runtime_stream_flush` (line 1414) — but rewrite its body in Step 6 to drop the `RuntimeContext` stream-state dependency once `stream::flush` is reduced. For now leave it calling `runtime::stream::flush`.

- [ ] **Step 4: Reduce `stream.rs` to the flush shell**

Replace the entire contents of `rust/runtime/src/stream.rs` with:

```rust
//! Stream lifecycle — reduced to the `flush` command surface.
//!
//! `open`/`arm`/`terminal`/`check_terminal_on_retire` and `FgStreamState`
//! were segment-era stubs and have been removed. `flush` is preserved as a
//! no-op shell; the host rewrite (same branch) replaces it with the real
//! force-idle/cancel mechanism.

#![allow(unsafe_code)]

use crate::error::KALICO_OK;
use crate::state::RuntimeContext;

/// Stream flush — no-op shell pending the host-rewrite mechanism.
///
/// # Safety
/// `ctx` must be non-null and point to a valid `RuntimeContext`.
/// `out_credit_epoch` may be null; if non-null it must be a valid `*mut u32`.
pub unsafe fn flush(_ctx: *mut RuntimeContext, _out_credit_epoch: *mut u32) -> i32 {
    KALICO_OK
}
```

- [ ] **Step 5: Remove segment/trace/stream fields from `state.rs`**

In `rust/runtime/src/state.rs`:
- Delete imports: `use crate::segment::Segment;` (38), `use crate::trace::{TRACE_RING_N, TraceSample};` (39).
- Delete `FgState` fields: `trace_consumer` (131), `stream_state_machine` (132), `retirement_table` (149), `pending_segment` (197), `first_priming_segment_t_start` (140), `terminal_segment_id` (143).
- Delete `IsrState` field `trace_producer` (178).
- Delete `RuntimeContext` field `trace_storage` (872) and its construction (918–921, 1031).
- Delete the `SharedState` segment-id counters and their construction: `current_segment_id` (236), `accepted_segment_id` (238), `retired_through_segment_id` (239), `terminal_segment_id_set` (282), `terminal_segment_id_value` (283), `accepted_segment_id_seen` (287), `producer_segment_retired_total` (370), `producer_segment_dequeued_total` (372), and any sibling segment-diagnostic counters (e.g. the `consumers_remaining` snapshot at ~264) whose only readers were the FFI handlers deleted in Step 3.
- Delete the constructor lines that build the removed fields: `trace_consumer: t_consumer` (1000), `stream_state_machine: ...` (1001), `retirement_table: ...` (1007), `trace_producer` write (1031).
- **Compiler enumerates readers:** build after these deletions; remove every flagged reader (all are dead segment/trace code). Note: `queue_producer`/`queue_consumer`/`queue_storage` and the `Q_N` import remain until Task 4 — leave them.

- [ ] **Step 6: Decouple the kept flush FFI + remove engine `_trace` param**

In `rust/kalico-c-api/src/runtime_ffi.rs`: confirm `kalico_runtime_stream_flush` still compiles against the reduced `stream::flush` (signature unchanged, so it should).
In `rust/runtime/src/engine.rs`:
- Delete the import `use crate::trace::{TRACE_RING_N, TraceSample};` (line 34) and the `pub use crate::stepping_state::N_AXES;` is unrelated — keep it.
- Remove the `_trace: &mut Producer<'_, TraceSample, TRACE_RING_N>` parameter from `Engine::tick` (line 284) and the `heapless::spsc::Producer` import (line 24) if now unused.
- Delete the `debug_current_segment_id` stub (lines 632–635).
In `rust/runtime/src/tick.rs`: at the `engine.tick(now, shared, storage, trace_producer)` call (line 364) remove the `trace_producer` argument, and remove `trace_producer` from the `IsrState` destructuring (lines 359–363).

- [ ] **Step 7: Fix remaining `sim_fixtures.rs` references**

In `rust/runtime/src/sim_fixtures.rs`, delete the now-dead imports and constructions: `use crate::reclaim::RetirementTable;` (106), `use crate::segment::{KinematicTag, Segment};` (107), `use crate::stream::FgStreamState;` (109), `use crate::trace::{TRACE_RING_N, TraceSample};` (110), the `trace_queue` setup (115–), `stream_state_machine: FgStreamState::Idle` (151), `retirement_table: RetirementTable::new()` (157), and `use crate::segment::{CurveHandle, KinematicTag, Segment};` (201) with the `e_mode: EMode::Travel` literal (236) if it is part of a removed struct. Leave `c_segment_queue` producer/consumer (112–113) for Task 4.

- [ ] **Step 8: Delete the module files + `lib.rs` declarations**

```bash
git rm rust/runtime/src/segment.rs rust/runtime/src/reclaim.rs rust/runtime/src/trace.rs
# also remove any tests siblings if present:
git rm -r rust/runtime/src/trace 2>/dev/null || true
```
In `rust/runtime/src/lib.rs` delete: `pub mod reclaim;` (47), `pub mod segment;` (48), `pub mod trace;` (last mod line). Keep `pub mod stream;`.

- [ ] **Step 9: Build green gate**

Run the host build-green gate. Expected: PASS. The `motion-bridge`/`kalico-c-api` crates must still build — `dispatch_corexy.rs` uses `geometry::segment` + host-side `dispatch::McuAxisConfig`, which are untouched.

- [ ] **Step 10: Firmware build gate, then commit**

Build both MCU targets. Then:
```bash
git add -A
git commit -m "remove: segment + trace + reclaim + stream open/arm/terminal (piece ring is the only data path; flush kept as shell)"
```

---

## Task 4 (Cluster 4): Remove the segment-queue infrastructure

**Files:**
- Modify: `rust/runtime/src/state.rs`, `rust/runtime/src/sim_fixtures.rs`, `rust/runtime/src/lib.rs`
- Delete: `rust/runtime/src/queue.rs` (+ `rust/runtime/src/queue/tests.rs`), `rust/runtime/src/c_segment_queue.rs`

- [ ] **Step 1: Remove the queue fields from `state.rs`**

In `rust/runtime/src/state.rs`:
- Delete the import `use crate::queue::Q_N;` (37).
- Delete `FgState` field `queue_producer` (130) and `IsrState` field `queue_consumer` (177).
- Delete `RuntimeContext` field `queue_storage: UnsafeCell<Queue<Segment, Q_N>>` (869) and its construction (`q_producer`/`q_consumer` at 914–915, `queue_producer: q_producer` at 999, `queue_consumer` write at 1030).
- Build; the compiler enumerates any remaining readers (none expected — they were segment-era).

- [ ] **Step 2: Fix `sim_fixtures.rs`**

In `rust/runtime/src/sim_fixtures.rs`, delete the `crate::c_segment_queue::Producer::new()` / `Consumer::new()` lines (112–113) and the corresponding `queue_producer`/`queue_consumer` assignments in the struct literal.

- [ ] **Step 3: Delete the module files + `lib.rs` declarations**

```bash
git rm rust/runtime/src/queue.rs rust/runtime/src/c_segment_queue.rs
git rm rust/runtime/src/queue/tests.rs 2>/dev/null || true
```
In `rust/runtime/src/lib.rs` delete: `pub mod c_segment_queue;` (33) and `pub mod queue;` (46).

- [ ] **Step 4: Build green gate**

Run the host build-green gate. Expected: PASS.

- [ ] **Step 5: Firmware build gate, then commit**

Build both MCU targets. Then:
```bash
git add -A
git commit -m "remove: segment SPSC queue infrastructure (queue + c_segment_queue)"
```

---

## Final verification

- [ ] **Confirm dead modules are gone:**
```bash
ls rust/runtime/src | grep -E "^(segment|reclaim|trace|c_segment_queue|queue|modulator|config)\.rs$"
```
Expected: no output (all removed). `stream.rs`, `sub_sample_timing.rs`, `test_xdirect_capture.rs` remain.

- [ ] **Confirm no stale references remain:**
```bash
grep -rnE "crate::(segment|reclaim|trace|c_segment_queue|queue|modulator|config)::|runtime::(segment|reclaim|trace|c_segment_queue|modulator|config)::" rust --include="*.rs" | grep -v "/target/" | grep -vE "geometry::segment|dispatch::McuAxisConfig"
```
Expected: no output. (Matches against `geometry::segment` and host-side `dispatch::McuAxisConfig` are intentionally excluded — those are live and unrelated.)

- [ ] **Confirm flush survives:**
```bash
grep -rn "stream_flush\|stream::flush" src rust/kalico-c-api/src rust/runtime/src
```
Expected: the `command_runtime_stream_flush` C command, the `kalico_runtime_stream_flush` FFI, and `stream::flush` shell all present.

- [ ] **Full workspace + firmware green:** host build-green gate passes; both MCU targets build clean.

- [ ] **Final state matches spec §6:** piece ring is the only MCU data path; no segment/stream-lifecycle/reclaim/trace/modulator/legacy-config code remains; `flush` is a no-op shell; `sub_sample_timing` and `test_xdirect_capture` untouched.
