# Segment-Era Dead-Module Removal

**Date:** 2026-05-28
**Branch:** `simple-mcu-contract`
**Goal:** Remove the segment/curve-pool–era modules that survived the piece-ring engine rewrite but remain structurally wired into the live `RuntimeContext` / `engine` / `tick` and the C↔Rust FFI. After this work, the piece ring is the only data path on the MCU, and the runtime carries no dead segment, stream-lifecycle, reclaim, trace, modulator, or legacy-config code.

This is a pure removal task. It does not add behavior. The host rewrite and the flush/cancel rewrite happen later in the same branch/PR and are out of scope here, except that we deliberately preserve the `flush` command surface as a decoupled shell for that rewrite to build on.

---

## 1. Background

The stepping redesign replaced the curve-pool + segment engine with a per-axis piece-ring walker (`Engine` in `rust/runtime/src/engine.rs`). Prior commits on this branch removed the curve pool, the segment/curve protocol messages, and the C-side segment/curve dispatch. What remains is a set of modules that are no longer on any live data path but are still referenced by struct fields, FFI exports, and C command registrations, so they still compile and link.

### 1.1 Module classification

The nine modules originally flagged split into three categories:

- **Dead — remove:** `segment`, `reclaim`, `c_segment_queue`, `queue`, `modulator`, `config`. Plus `trace`, which is collateral-dead (see §3.3).
- **Reduce, do not delete:** `stream` — keep `flush` only.
- **Live — keep untouched:** `sub_sample_timing` (the pulse-stepping timing kernel, `compute_step_times`, called from `dispatch_pulse` in `tick.rs`), and `test_xdirect_capture` (a `#[cfg(any(test, feature = "host"))]` observation seam in `dispatch_phase`, used by the `phase_xdirect_dispatch` integration test).

`sub_sample_timing` and `test_xdirect_capture` were on the original "dead" list but are demonstrably live; the spec keeps them.

### 1.2 What is explicitly NOT touched

- `geometry::segment` — the live planner geometry crate (`CubicSegment`, `EMode`, `SourceRange`). Unrelated to `runtime::segment`.
- `motion_bridge::dispatch::McuAxisConfig` — the host-side config type. Unrelated to `runtime::config::McuAxisConfig`.
- `rust/runtime/src/phase_lut.rs`, `dispatch_phase`, and the entire pulse/phase output stage in `tick.rs` — confirmed identical across `sota-motion` and HEAD; the phase output logic is the known-good live path and stays.
- `StatusHeartbeat` telemetry (per-axis consumed counts) — the current and future telemetry path.

---

## 2. Phase-stepping investigation (resolved, no action)

During scoping we reviewed whether `dispatch_phase` had diverged from the older `PhaseDirectModulator` design. Findings, for the record:

- `dispatch_phase`'s body is identical on `sota-motion` and HEAD (only comments differ). The phase output logic did not change on this branch.
- `PhaseDirectModulator` (`modulator.rs`) was already dead on `sota-motion` (the `phase_modulators` array was never populated to `Some`). It was last live in the pre-segment-removal `runtime_modulated_tick` era.
- The live `dispatch_phase` and the historical modulator differ in three ways: LUT convention (`PHASE_LUT` uses `(coil_A=cos, coil_B=sin)` vs `LUT_ENTRIES` `(sin, cos)`), no per-tick burst cap, and f32 vs f64 quantization. Dropping the f64 accumulator does not cause drift because the position source is absolute, not incremental.

**Decision:** `dispatch_phase` is the live, known-good code and is kept as-is. `modulator.rs` is unused and is removed. The LUT-convention and burst-cap observations are noted but out of scope for this removal task.

---

## 3. Removal scope

### 3.1 Rust runtime modules (`rust/runtime/src/`)

**Delete wholesale** (file + any `tests.rs`/`tests/` sibling + `lib.rs` `pub mod` line):

- `segment.rs`
- `reclaim.rs`
- `trace.rs`
- `c_segment_queue.rs`
- `queue.rs`
- `modulator.rs`
- `config.rs`

**Reduce:**

- `stream.rs` → keep only `flush` (the `KALICO_OK` no-op shell), decoupled from removed state. Remove `FgStreamState`, `open`, `arm`, `terminal`, and `check_terminal_on_retire`.

**Keep untouched:**

- `sub_sample_timing.rs`, `test_xdirect_capture.rs`.

### 3.2 `state.rs` (`RuntimeContext` / `FgState` / `IsrState`)

Remove fields and their construction in `new` / `init_in_place` / `sim_fixtures`:

- `queue_producer`, `queue_consumer` (`c_segment_queue`)
- `stream_state_machine` (`FgStreamState`)
- `retirement_table` (`reclaim`)
- `trace_producer`, `trace_consumer`, `trace_storage` (`trace`)
- Imports of `crate::queue::Q_N`, `crate::segment::Segment`, `crate::trace::{TraceSample, TRACE_RING_N}`
- The segment-era diagnostic counters whose only purpose was tracking segment dequeue / SPSC wedge behavior (e.g. `dequeued`, `observed_none`, queue-len snapshots). Each is removed only after confirming no live reader remains.

### 3.3 `engine.rs`

- Remove `phase_modulators` field (+ const-init in `new`/`init_in_place`, + reset loop in `runtime_force_idle`) and the `crate::modulator::PhaseDirectModulator` import.
- Remove `mcu_config` field, the `configure(&mut self, McuAxisConfig)` method, and the `crate::config::McuAxisConfig` import.
- Remove the unused `_trace: &mut Producer<TraceSample, TRACE_RING_N>` parameter from `tick` and its call site in `isr_sample_tick` (`tick.rs`), plus the `crate::trace::{...}` import.
- Remove the `debug_current_segment_id` stub (returns `None`).
- `step_state` / `seed`-related accessors: `config.rs`/`configure()` fed `step_state[i].steps_per_mm`, which is not read by the live dispatch path (pulse + phase both use `axis.microstep_distance`). Removing `engine.configure()` orphans the steps-per-mm seeding of `step_state`. Audit `step_state` usage during implementation; if it is fully vestigial after `configure()` is gone, remove it too. If any live reader remains (e.g. a debug FFI), leave `step_state` in place and only remove the `config`-driven write path.

> **Trace is collateral-dead, removed with the segment cluster.** The engine already ignores the trace producer (`_trace` is unused). The trace ring's only consumers are the `reclaim`/drain FFI handlers. `TraceSample` carries a `segment::CurveHandle`. Removing `segment` forces decoupling `trace`; removing `reclaim` removes trace's only consumer. Therefore `segment` + `trace` + `reclaim` must be removed as one coupled cluster (§5, cluster 3).

### 3.4 FFI (`rust/kalico-c-api/src/runtime_ffi.rs`)

Remove these exports and their bodies:

- `kalico_runtime_stream_open`, `kalico_runtime_stream_arm`, `kalico_runtime_stream_terminal`
- `kalico_runtime_drain_and_reclaim`, `runtime_handle_drain_trace`
- `kalico_runtime_configure_axes_blob`
- Segment-id query handlers: `runtime_handle_accepted_segment_id`, `runtime_handle_retired_through_segment_id`, `runtime_handle_current_segment_id`, `kalico_runtime_pending_segment_is_some`, `runtime_handle_credit_epoch`
- The `use runtime::segment::KinematicTag` and `use runtime::config::{...}` imports

**Keep:** `kalico_runtime_stream_flush` (decoupled to call the `stream::flush` shell). Keep `kalico_runtime_configure_axis` (the new per-axis path) and its siblings `configure_kinematics` / `configure_pressure_advance` (already no-op ABI stubs; leave unless the host rewrite removes them).

### 3.5 C firmware (`src/`)

Remove `DECL_COMMAND`s + handlers:

- `command_runtime_stream_open`, `command_runtime_stream_arm`, `command_runtime_stream_terminal` (`runtime_commands.c`)
- `command_runtime_configure_axes_blob` (`runtime_commands.c`) and its callers (`kalico_dispatch.c`)
- The `kalico_runtime_drain_and_reclaim` call in `runtime_tick.c`

**Keep:** `command_runtime_stream_flush` (calls the kept flush FFI).

### 3.6 Cross-crate dead tests

Delete (they test removed behavior):

- `rust/kalico-c-api/tests/drain_trace_credit.rs`
- `rust/motion-bridge/tests/bridge_to_runtime_step_chain.rs`
- `rust/runtime/tests/modulator_math.rs`

Keep (they test live code):

- `rust/runtime/tests/sub_sample_timing.rs`
- `rust/runtime/tests/phase_xdirect_dispatch.rs`
- `rust/motion-bridge/tests/dispatch_corexy.rs` (uses host-side `dispatch::McuAxisConfig` + `geometry::segment`, both live)

---

## 4. Out of scope (later in this branch/PR)

- Host rewrite (new configuration flow, new stream/flush/cancel driver).
- Rewriting `flush` into a real mechanism. This task only preserves the command surface as a no-op shell.
- A `cancel` command (net-new; does not exist today).
- Any change to `dispatch_phase`, the LUT convention, or a phase burst cap.

---

## 5. Execution plan — cluster sequencing

Each cluster is one reviewable commit. Within each cluster, work leaf-first: remove consumers (tests, C `DECL_COMMAND`s, FFI exports, struct fields) before the module definitions, so the workspace and the bench firmware build stay green at every commit boundary.

**Cluster 1 — Modulator (independent).**
Delete `modulator_math.rs`; remove `phase_modulators` field + import + `runtime_force_idle` reset; delete `modulator.rs` + `lib.rs` line.

**Cluster 2 — Legacy config.**
Remove the C `command_runtime_configure_axes_blob` + callers in `kalico_dispatch.c`; remove the `kalico_runtime_configure_axes_blob` FFI; remove `engine.configure()` / `mcu_config` / `McuAxisConfig` import; audit and (if vestigial) remove `step_state`; delete `config.rs` + `lib.rs` line.

**Cluster 3 — Segment + trace + reclaim + stream-lifecycle (the coupled cluster).**
Delete dead tests (`drain_trace_credit.rs`, `bridge_to_runtime_step_chain.rs`); remove C `DECL_COMMAND`s for `stream_open`/`stream_arm`/`stream_terminal` + the `drain_and_reclaim` call in `runtime_tick.c`; remove the corresponding FFI exports (stream open/arm/terminal, drain_and_reclaim, drain_trace, segment-id queries) + `KinematicTag` import; remove `state.rs` fields (`retirement_table`, `trace_*`, `Segment`/`TraceSample` imports); remove engine `_trace` param + trace import; reduce `stream.rs` to the `flush` shell (drop `FgStreamState`, `open`, `arm`, `terminal`, `check_terminal_on_retire`) and remove the `stream_state_machine` field; delete `segment.rs`, `reclaim.rs`, `trace.rs` + `lib.rs` lines.

**Cluster 4 — Segment-queue infrastructure.**
Remove `state.rs` `queue_producer`/`queue_consumer` fields + `Q_N` import (if not already gone with cluster 3); delete `queue.rs`, `c_segment_queue.rs` + `lib.rs` lines.

> Cluster ordering note: clusters 3 and 4 share `state.rs` field edits and the `c_segment_queue` producer/consumer hold `Segment`. If leaf-first ordering within cluster 3 requires the queue fields gone first, clusters 3 and 4 may be reordered or merged — the implementer decides based on what keeps the build green. The four-cluster split is the default; correctness of the green-at-every-commit invariant takes precedence over the exact grouping.

---

## 6. Verification

At each cluster commit boundary:

1. `cargo build` and `cargo test` for the workspace (host target) — green.
2. Clippy passes (the runtime crate denies `panic`/`unwrap`/`indexing_slicing`/etc. at the crate root; removals must not introduce violations).
3. The MCU firmware builds for both targets (H7 and F446) per the bench flow (commit → push → pull → build on the Pi). C `DECL_COMMAND` removals must not leave dangling `extern` declarations or unreferenced handlers.

The final state: piece ring is the only MCU data path; no `segment`/`stream`-lifecycle/`reclaim`/`trace`/`modulator`/legacy-`config` code remains; `flush` survives as a no-op command shell awaiting the host rewrite; `sub_sample_timing` and `test_xdirect_capture` are untouched.

---

## 7. Open questions

None. Scope, disposition, and sequencing are settled. The `step_state` vestigiality audit (§3.3) is the one item resolved during implementation rather than now.
