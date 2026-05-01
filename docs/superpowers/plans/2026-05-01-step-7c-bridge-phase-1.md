# Step 7-C-bridge Phase 1: scaffold + delete + all-MCU passthrough router

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Klippy boots against the user's Trident config, configures TMC drivers across all MCUs, reads thermistors on Octopus + F446 + frame, drives heaters (extruder + bed), enumerates beacon and NIS — all through a new Rust-side passthrough router replacing `serialqueue.c`. No motion possible (homing returns "not yet implemented"). Phase 1 ends with the new motion path's wire ownership in place; Phase 2 adds first motion.

**Architecture:** Add a `motion-bridge` PyO3 crate (Rust cdylib, imported by klippy as a Python extension). The bridge owns the serial fd to every Klipper-protocol MCU. A new `kalico-host-rt::passthrough_queue` module ports `klippy/chelper/serialqueue.c` to Rust, integrating with the existing 7-C-io reactor. Klippy `mcu.py` is patched to allocate a `MotionMcuProxy` (delegates to bridge) instead of allocating a C `serialqueue` and opening the fd directly. Trapezoidal motion C code (`itersolve`, `stepcompress`, `trapq`, `kin_*.c`) and the just-displaced `serialqueue.*` / `trdispatch.c` are deleted. Motion-related Python (`toolhead.py`, `kinematics/*.py` step generators, `gcode_arcs.py`) is deleted. `motion_toolhead.py` / `motion_mcu.py` / `motion_kinematics.py` skeletons land. CLAUDE.md and dependency-graph.md amendments per spec §1.4 / §9 land in the same batch.

**Tech Stack:**
- **Rust:** PyO3 0.24 (already declared as optional in `kalico-host-rt`), arc-swap, serde, indexmap, log, flate2, serialport. Workspace already exists at `rust/`.
- **Python:** klippy (Python 3 + cffi for the surviving non-motion C bits if any).
- **Build:** klippy uses `make` with `chelper/__init__.py` cffi loading. Phase 1 adds a step that builds the PyO3 crate and drops the resulting `.so` where klippy can `import motion_bridge`.
- **Test:** Rust unit tests + proptest in workspace. Python `pytest` for klippy-side smoke tests. `kalico-sim` (existing host-process MCU sim) for the boot-to-heater-setpoint smoke test.

**Reference docs (engineers should read before starting):**
- `docs/superpowers/specs/2026-05-01-step-7c-bridge-design.md` — the design spec (esp. §2.2, §2.3, §3.5, §3.5.1, §3.6, §3.8, §5).
- `docs/superpowers/specs/2026-04-30-step-7c-io-design.md` and `2026-05-01-step-7c-io-tail-design.md` — existing host I/O reactor and Clock seam.
- `klippy/chelper/serialqueue.c` (992 LOC) — the C source being ported.
- `klippy/chelper/trdispatch.c` (226 LOC) — the C source being deleted (Phase 4 replaces).
- `klippy/serialhdl.py` — the Python wrapper of `serialqueue.c` (msgparser + identify + response dispatch).
- `klippy/mcu.py` — the Klipper-protocol Python machinery preserved.
- `rust/kalico-host-rt/src/host_io/` — existing reactor, parser, window, runtime_events.

---

## File Structure

**Created:**

- `rust/motion-bridge/` — new PyO3 cdylib crate
  - `Cargo.toml`
  - `src/lib.rs` — `#[pymodule] motion_bridge` registration
  - `src/api.rs` — PyO3-bound methods on `MotionBridge` (lifecycle, passthrough_*, claim_mcu)
  - `src/events.rs` — Rust event types + Python event drain queue
- `rust/kalico-host-rt/src/passthrough_queue/` — new Rust module porting serialqueue.c
  - `mod.rs` — public surface
  - `command_queue.rs` — per-driver command queues with upcoming/ready promotion
  - `entry.rs` — `PassthroughEntry { bytes, min_clock, req_clock, notify_id, queue_id }`
  - `notify.rs` — notify-id correlation for queries
  - `window.rs` — receive-window backpressure (separate from existing host_io::window)
  - `stats.rs` — per-MCU stats for `serialqueue_get_stats` parity
- `rust/kalico-host-rt/tests/passthrough_queue_*.rs` — module tests
- `klippy/motion_bridge.py` — Python wrapper around the PyO3 module (reactor adapter)
- `klippy/motion_toolhead.py` — toolhead replacement (Phase 1: skeleton with §3.6.2 surface; full motion methods stubbed with NotImplementedError until Phase 2+)
- `klippy/motion_mcu.py` — MotionMcuProxy class implementing klippy mcu.py public surface via bridge
- `klippy/motion_kinematics.py` — kinematics config-parser → KinematicsSpec (Phase 1: skeleton + Cartesian + CoreXY parsers; no runtime logic until Phase 2)
- `tests/motion_bridge/test_smoke.py` — Python pytest for bridge import + lifecycle smoke
- `tests/motion_bridge/test_klippy_boot.py` — full klippy boot smoke test under kalico-sim

**Modified:**

- `Makefile` / `Makefile.kalico` — add cargo build step that drops `motion_bridge.so` where klippy imports it
- `rust/Cargo.toml` — add `motion-bridge` workspace member
- `rust/kalico-host-rt/src/lib.rs` — register `passthrough_queue` module + re-exports
- `rust/kalico-host-rt/src/host_io/reactor.rs` — integrate passthrough_queue into the tick loop
- `rust/kalico-host-rt/src/transport.rs` — extend transport surface with passthrough commands
- `rust/kalico-host-rt/Cargo.toml` — flip `pyo3` from `optional = true` to required behind `python-bridge` feature
- `klippy/mcu.py` — patch constructor to allocate `MotionMcuProxy` via bridge instead of `serialqueue`
- `klippy/serialhdl.py` — gut the C-side serialqueue allocation; thin Python-side msgparser glue stays (or fold into motion_mcu.py — decide during execution)
- `klippy/stepper.py` — preserve config-object surface; gut motion internals (queue_step, itersolve binding); see spec §5.2 for full preserved API list
- `klippy/kinematics/extruder.py` — patch per spec §5.2: keep `PrinterExtruder` / `ExtruderStepper` surface; route trapezoidal bits through bridge
- `klippy/kinematics/idex_modes.py` — patch to no-op refusal of mode-switch
- `klippy/extras/motion_report.py` — patch trapq dump endpoint to bridge-state-backed; preserve `trapqs` dict shape for `load_cell/tap_analysis` consumer
- `klippy/extras/input_shaper.py` — gut trapezoidal IS C path; convert to ShaperSpec config-parser + `SET_INPUT_SHAPER` → `bridge.update_shaper()` (Phase 1: scaffold + reject; full impl Phase 3)
- `klippy/printer.py` — instantiate the bridge during `_connect()` before MCU objects are constructed
- `CLAUDE.md` — amendment per spec §9
- `docs/kalico-rewrite/dependency-graph.md` — amendment per spec §9

**Deleted:**

- `klippy/toolhead.py`
- `klippy/kinematics/cartesian.py`, `corexy.py`, `corexz.py`, `cartesian_abc.py`, `delta.py`, `deltesian.py`, `polar.py`, `rotary_delta.py`, `winch.py`, `hybrid_corexy.py`, `hybrid_corexz.py`, `limited_cartesian.py`, `limited_corexy.py`, `limited_corexz.py`, `none.py`
- `klippy/extras/gcode_arcs.py`
- `klippy/chelper/itersolve.c`, `itersolve.h`
- `klippy/chelper/stepcompress.c`, `stepcompress.h`
- `klippy/chelper/serialqueue.c`, `serialqueue.h`
- `klippy/chelper/trapq.c`, `trapq.h`
- `klippy/chelper/trdispatch.c`
- `klippy/chelper/kin_cartesian.c`, `kin_corexy.c`, `kin_delta.c`, `kin_extruder.c`, `kin_polar.c`, `kin_rotary_delta.c`, `kin_winch.c`, `kin_shaper.c`, `pollreactor.c`/`.h` if motion-only

**Hard-disabled at config-time** (config-loader patched per §5.3):

- `klippy/extras/manual_stepper.py` — Phase 5 reimplements; Phase 1 raises "manual_stepper not yet supported under the new motion path; deferred to Phase 5".
- `klippy/extras/force_move.py` — same.
- `klippy/extras/mixing_extruder.py` — post-MVP; permanent hard-disable for now.
- `klippy/extras/trad_rack.py` — post-MVP; permanent hard-disable.
- `klippy/extras/pwm_tool.py` — post-MVP; permanent hard-disable.
- `klippy/extras/z_tilt.py`, `klippy/extras/z_tilt_ng.py` — Phase 5 reimplements; Phase 1 raises "Z tilt not yet supported until Phase 5".
- `klippy/extras/homing.py` — Phase 4 reimplements; Phase 1 raises "homing not yet supported until Phase 4" if the user actually invokes G28; module imports cleanly so klippy can boot.
- `klippy/extras/load_cell/*` (if present) — same pattern.
- `klippy/mcu.py::MCU_trsync` — stub class that refuses to arm; raises during homing.

---

## Stage A — Scaffolding + amendment (Tasks 1-10)

These tasks land the empty PyO3 crate, the build integration, and the CLAUDE.md / dependency-graph.md amendment. The goal: `import motion_bridge` from a Python REPL succeeds, and the docs match the design.

### Task 1: Create `rust/motion-bridge` crate skeleton

**Files:**
- Create: `rust/motion-bridge/Cargo.toml`
- Create: `rust/motion-bridge/src/lib.rs`
- Modify: `rust/Cargo.toml`

- [ ] **Step 1: Add workspace member**

In `rust/Cargo.toml`, add `motion-bridge` to `[workspace] members`:

```toml
members = [
    "nurbs",
    "kalico-c-api",
    "gcode",
    "geometry",
    "temporal",
    "trajectory",
    "runtime",
    "kalico-host-rt",
    "compat",
    "motion-bridge",
]
```

- [ ] **Step 2: Write `motion-bridge/Cargo.toml`**

```toml
[package]
name = "motion-bridge"
version = "0.1.0"
edition = "2024"
rust-version = "1.85"
publish = false
description = "PyO3 bridge between klippy (Python) and the Rust motion stack."
license.workspace = true

[lib]
name = "motion_bridge"
crate-type = ["cdylib"]

[dependencies]
pyo3 = { version = "0.24", features = ["extension-module", "abi3-py39"] }
kalico-host-rt = { path = "../kalico-host-rt", features = ["python-bridge"] }
trajectory = { path = "../trajectory" }
temporal = { path = "../temporal" }
geometry = { path = "../geometry" }
gcode = { path = "../gcode" }
compat = { path = "../compat" }
log = "0.4"
arc-swap = "1"

[lints]
workspace = true
```

- [ ] **Step 3: Write minimal `src/lib.rs`**

```rust
//! `motion-bridge` — PyO3 bridge between klippy and the Rust motion stack.
//! Phase 1: empty surface; only the module registers so klippy can `import motion_bridge`.
//! See docs/superpowers/specs/2026-05-01-step-7c-bridge-design.md for the design.

use pyo3::prelude::*;

/// Phase 1 scaffold; method surface lands in subsequent tasks.
#[pyclass(name = "MotionBridge")]
pub struct PyMotionBridge {
    _placeholder: (),
}

#[pymethods]
impl PyMotionBridge {
    #[new]
    fn new() -> Self {
        Self { _placeholder: () }
    }

    /// Returns a string identifying the build. Used by the smoke test.
    fn version(&self) -> &'static str {
        env!("CARGO_PKG_VERSION")
    }
}

#[pymodule]
fn motion_bridge(_py: Python<'_>, m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyMotionBridge>()?;
    Ok(())
}
```

- [ ] **Step 4: Verify the crate compiles**

Run: `cd rust && cargo build -p motion-bridge --release`

Expected: builds. Output `.so` lands at `rust/target/release/libmotion_bridge.so` (Linux) or `libmotion_bridge.dylib` (macOS).

- [ ] **Step 5: Commit**

```bash
git add rust/Cargo.toml rust/motion-bridge/
git commit -m "feat(motion-bridge): add PyO3 crate skeleton"
```

### Task 2: Wire `python-bridge` feature in `kalico-host-rt`

**Files:**
- Modify: `rust/kalico-host-rt/Cargo.toml`

- [ ] **Step 1: Make pyo3 dependency available behind feature flag**

The crate already declares pyo3 as optional. Verify the feature gate is set up correctly:

```toml
[dependencies.pyo3]
version = "0.24"
optional = true

[features]
default = []
python-bridge = ["dep:pyo3"]
test-harness = []
```

Ensure `motion-bridge` activating `python-bridge` actually does something useful — for now, leave the feature as a placeholder; later tasks will gate code behind it.

- [ ] **Step 2: Verify build with feature flag**

Run: `cd rust && cargo build -p kalico-host-rt --features python-bridge`

Expected: builds.

- [ ] **Step 3: Commit**

```bash
git add rust/kalico-host-rt/Cargo.toml
git commit -m "feat(kalico-host-rt): wire python-bridge feature flag"
```

### Task 3: Add Makefile target to build motion-bridge and install the .so

**Files:**
- Modify: `Makefile.kalico` (or `Makefile` — check which one klippy uses for chelper builds)

- [ ] **Step 1: Inspect existing Makefile structure**

Run: `grep -n "chelper\|cargo\|\.so" Makefile.kalico Makefile 2>/dev/null`

Note the existing pattern for chelper compilation.

- [ ] **Step 2: Add motion-bridge build target**

Add to `Makefile.kalico` (or whichever file owns klippy's build orchestration):

```makefile
# Step 7-C-bridge — build the PyO3 motion-bridge module and drop the .so
# where klippy can import it. See docs/superpowers/specs/2026-05-01-step-7c-bridge-design.md.

MOTION_BRIDGE_TARGET = rust/target/release/libmotion_bridge.$(if $(filter Darwin,$(shell uname)),dylib,so)
MOTION_BRIDGE_DEST = klippy/motion_bridge.so

motion-bridge:
	cd rust && cargo build -p motion-bridge --release
	cp $(MOTION_BRIDGE_TARGET) $(MOTION_BRIDGE_DEST)

clean-motion-bridge:
	rm -f $(MOTION_BRIDGE_DEST)
	cd rust && cargo clean -p motion-bridge

# Make this part of the default build target. Adjust the dependency chain
# to match the existing klippy build pattern; if klippy currently builds
# chelper as part of `all`, hook motion-bridge in the same way.
all: motion-bridge

.PHONY: motion-bridge clean-motion-bridge
```

Adjust the rule integration to match what klippy currently does for chelper.

- [ ] **Step 3: Test the build target**

Run: `make motion-bridge`

Expected: builds the Rust crate and copies `libmotion_bridge.so` (or `.dylib`) to `klippy/motion_bridge.so`.

- [ ] **Step 4: Verify Python can import**

Run: `cd klippy && python3 -c "import motion_bridge; b = motion_bridge.MotionBridge(); print(b.version())"`

Expected: prints `0.1.0` (or whatever PKG_VERSION is).

- [ ] **Step 5: Commit**

```bash
git add Makefile.kalico
git commit -m "build: integrate motion-bridge cargo build into klippy make"
```

### Task 4: Write Python smoke test for motion-bridge import

**Files:**
- Create: `tests/motion_bridge/test_smoke.py`
- Create: `tests/motion_bridge/__init__.py` (empty)

- [ ] **Step 1: Write the failing test**

```python
# tests/motion_bridge/test_smoke.py
"""Smoke test that motion_bridge imports and instantiates."""

def test_module_imports():
    import motion_bridge
    assert hasattr(motion_bridge, "MotionBridge")

def test_bridge_instantiates():
    import motion_bridge
    bridge = motion_bridge.MotionBridge()
    assert bridge.version() != ""
```

- [ ] **Step 2: Run the test**

Run: `cd tests && python3 -m pytest motion_bridge/test_smoke.py -v`

Expected: PASS (assuming Task 3 ran successfully and the .so is in the import path).

If it fails with `ModuleNotFoundError`, adjust `PYTHONPATH` in the test runner: `PYTHONPATH=$(pwd)/klippy python3 -m pytest tests/motion_bridge/test_smoke.py -v`

Decide where the `.so` should live (klippy/ or a separate native/ directory) and document.

- [ ] **Step 3: Commit**

```bash
git add tests/motion_bridge/
git commit -m "test(motion-bridge): smoke test for module import + version"
```

### Task 5: Verify build & test orchestration is reproducible

- [ ] **Step 1: Clean and rebuild end-to-end**

```bash
make clean-motion-bridge
make motion-bridge
cd tests && python3 -m pytest motion_bridge/test_smoke.py -v
```

Expected: builds clean, test passes. If it fails, fix the build glue before proceeding.

- [ ] **Step 2: Commit any fixes from Step 1**

```bash
git commit -am "fix(build): <whatever was needed>"
```

(Skip if no changes.)

### Task 6: Apply CLAUDE.md amendment (spec §9)

**Files:**
- Modify: `CLAUDE.md`

- [ ] **Step 1: Locate the "G5 / G5.1 only" bullet**

Run: `grep -n "G5 / G5.1 only" CLAUDE.md`

The bullet is in the high-level feature scope section.

- [ ] **Step 2: Edit the bullet text**

Change the existing rule (which says "anything that reaches the planner is G5 or G5.1; anything else is rejected at the lexer/reduce boundary as a hard error") to match spec §9's amended wording:

```
- **G5 / G5.1 only — no legacy G-code in the planner reduce stage.** G5 → cubic Bézier direct; G5.1 → cubic via exact degree-elevation. The planner reduce stage (`rust/geometry`'s reducer + `rust/temporal` + `rust/trajectory`) has zero internal handling for G0 / G1 / G2 / G3 — no reduction code paths, no `Linear` / `RationalQuadratic` / `FittedSegment` / `ArcSegment` types, no feature-flagged "legacy mode." Anything reaching reduce is G5 or G5.1; anything else is rejected at the reduce boundary as a hard error.

  The `rust/gcode` lexer remains capable of tokenizing legacy G-code, because the `compat` crate (Step 13's normalizer) and the bridge's live-G1-conversion path both depend on it. Tokenization is not the rejection boundary.

  The `compat` crate has two callers: the offline Step-13 binary (file → file) and the live bridge (terminal/macro G1/G2/G3 conversion via `compat::collinear::to_collinear_g5`, `compat::arc::arc_to_g5`, `compat::degree_elev::elevate_g51_to_g5`). Both share the lexer.
```

- [ ] **Step 3: Inspect Step 7 / Step 13 prose for any related text that needs updating**

Run: `grep -n "compat\|G5 only\|legacy G-code\|Step 13\|G1 → G5" CLAUDE.md`

Update any other references that contradict the amended rule.

- [ ] **Step 4: Verify CLAUDE.md still reads coherently**

Read the bullet in context; ensure it doesn't contradict surrounding bullets.

### Task 7: Apply dependency-graph.md amendment (spec §9)

**Files:**
- Modify: `docs/kalico-rewrite/dependency-graph.md`

- [ ] **Step 1: Locate the Layer 1 G-code parser bullet**

Run: `grep -n "Legacy G0\|live parser\|live pipeline" docs/kalico-rewrite/dependency-graph.md`

- [ ] **Step 2: Edit the Layer 1 bullet**

Replace the existing "Legacy G0/G1/G2/G3 are not handled by the live parser..." text with:

```
- **G-code parser (live pipeline)** accepts G5 / G5.1 (and the standard non-motion CNC machinery — work coordinates, override characters, comments, M-codes routed to telemetry). The shared `rust/gcode` lexer also tokenizes legacy G0/G1/G2/G3 because both `compat` (Step 13's normalizer) and the bridge's live-conversion path depend on it; rejection happens at the *planner reduce stage*, not the lexer.
- **Geometric reduction (planner reduce stage):** G5 → cubic Bézier polynomial NURBS direct; G5.1 → cubic via exact degree-elevation (degree 2 → 3, +1 control point, no fit error). Legacy G0/G1/G2/G3 reaching reduce is rejected as a hard error. Live legacy support is provided by the bridge (above the reduce stage), which converts via `compat` primitives before invoking reduce. File-based prints are normalized once at print-start by the same primitives.
```

- [ ] **Step 3: Update Step 13 closing notes**

Find the prose that frames Step 13 as offline-only and add the second-caller note:

```
The `compat` crate has two callers: the offline Step-13 binary (file → file, the original framing) and the live bridge for terminal/macro G1/G2/G3 conversion. Both share the lexer in `rust/gcode` and the primitive functions in `compat::{collinear,arc,degree_elev}`.
```

- [ ] **Step 4: Verify the document still reads coherently**

Read the dependency graph section; ensure the layer descriptions remain consistent.

### Task 8: Commit the doc amendment

- [ ] **Step 1: Diff review**

Run: `git diff CLAUDE.md docs/kalico-rewrite/dependency-graph.md`

- [ ] **Step 2: Commit**

```bash
git add CLAUDE.md docs/kalico-rewrite/dependency-graph.md
git commit -m "docs: amend G5-only rule for bridge live-conversion (spec §9)"
```

### Task 9: Add the plan-changes-log entry

**Files:**
- Modify: `docs/superpowers/plan-changes-log.md`

- [ ] **Step 1: Append entry**

Add to the log file:

```
## 2026-05-01 — CLAUDE.md G5-only rule amended for bridge live-conversion

**What changed:** CLAUDE.md and dependency-graph.md G5-only rule clarified: the planner *reduce stage* (geometry reducer + temporal + trajectory) is the rejection boundary, not the lexer. The `rust/gcode` lexer tokenizes legacy G-code (compat depends on it). Bridge converts live G1/G2/G3 via compat primitives before the reduce stage.

**Why:** Step 7-C-bridge needs to support live G1 from gcode-macros/terminal (user's PRINT_START etc. emit G1). Original rule placed rejection at the lexer, which would have broken compat itself.

**Evidence:** docs/superpowers/specs/2026-05-01-step-7c-bridge-design.md §1.4, §9. Codex review passes 1/2/3 surfaced and confirmed the resolution.
```

- [ ] **Step 2: Commit**

```bash
git add docs/superpowers/plan-changes-log.md
git commit -m "docs: log G5-rule amendment for 7-C-bridge"
```

### Task 10: Stage A done — manual smoke check

- [ ] **Step 1: Verify the build still works**

Run: `make motion-bridge && cd tests && python3 -m pytest motion_bridge/test_smoke.py -v`

Expected: PASS.

- [ ] **Step 2: Verify CLAUDE.md / dependency-graph.md read sensibly**

```bash
git log --oneline | head -5
```

Sanity-check that recent commits are coherent. No additional code changes here.

---

## Stage B — `passthrough_queue` Rust port (Tasks 11-30)

This is the load-bearing piece of Phase 1: a Rust port of `klippy/chelper/serialqueue.c` (992 LOC) integrated with the existing `kalico-host-rt::host_io` reactor. Per spec §3.5.3, the new module lives at `rust/kalico-host-rt/src/passthrough_queue/`.

**Working principle:** TDD per feature. Each task ports one concept from `serialqueue.c`, with a Rust-native test that pins down behavior. Reference the C source by line range so the engineer can compare semantics. Preserve the externally-observable behavior (klippy's `serialhdl.py` consumers don't notice the swap), not the internal data structure.

The order roughly follows the C source's complexity ordering: data types and entry insertion first, then min_clock ordering, then upcoming/ready promotion, then the reactor integration, then notify-id correlation and timestamps, then receive-window backpressure, then flush callbacks and stats.

### Task 11: Define core data types

**Files:**
- Create: `rust/kalico-host-rt/src/passthrough_queue/mod.rs`
- Create: `rust/kalico-host-rt/src/passthrough_queue/entry.rs`
- Modify: `rust/kalico-host-rt/src/lib.rs`

**Source reference:** `klippy/chelper/serialqueue.c:46-100` (struct queue_message, struct command_queue).

- [ ] **Step 1: Write the failing test**

```rust
// rust/kalico-host-rt/src/passthrough_queue/entry.rs (test module at bottom)
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn entry_holds_bytes_and_clocks() {
        let bytes = vec![0x10, 0x20, 0x30];
        let entry = PassthroughEntry::new(
            bytes.clone(),
            /* min_clock */ 1000,
            /* req_clock */ 2000,
            NotifyId::none(),
        );
        assert_eq!(entry.bytes(), bytes.as_slice());
        assert_eq!(entry.min_clock(), 1000);
        assert_eq!(entry.req_clock(), 2000);
        assert_eq!(entry.notify_id(), NotifyId::none());
    }

    #[test]
    fn notify_id_distinct() {
        let a = NotifyId::new(1);
        let b = NotifyId::new(2);
        assert_ne!(a, b);
        assert_ne!(a, NotifyId::none());
    }
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cd rust && cargo test -p kalico-host-rt passthrough_queue::entry`

Expected: FAIL with "module not found" or "type not found".

- [ ] **Step 3: Implement the types**

```rust
// rust/kalico-host-rt/src/passthrough_queue/entry.rs

/// Notify ID for correlating passthrough query ↔ response.
/// `NotifyId::none()` means "no response expected (fire-and-forget)".
/// Mirrors klippy serialqueue.c queue_message.notify_id (uint64_t, 0 = none).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct NotifyId(u64);

impl NotifyId {
    pub const fn none() -> Self {
        NotifyId(0)
    }
    pub const fn new(id: u64) -> Self {
        debug_assert!(id != 0, "NotifyId::new requires non-zero id");
        NotifyId(id)
    }
    pub fn is_none(&self) -> bool {
        self.0 == 0
    }
    pub fn raw(&self) -> u64 {
        self.0
    }
}

/// One Klipper-protocol message bound for a specific MCU's command queue.
/// Mirrors klippy serialqueue.c struct queue_message.
#[derive(Debug, Clone)]
pub struct PassthroughEntry {
    bytes: Vec<u8>,
    /// Minimum MCU clock at which this entry may be emitted onto the wire.
    /// 0 = emit immediately. Sourced from CommandWrapper.send(minclock=...).
    min_clock: u64,
    /// Requested MCU clock — annotation passed in the message itself.
    /// Drives upcoming → ready promotion.
    req_clock: u64,
    /// Correlation id for queries; NotifyId::none() for fire-and-forget.
    notify_id: NotifyId,
}

impl PassthroughEntry {
    pub fn new(bytes: Vec<u8>, min_clock: u64, req_clock: u64, notify_id: NotifyId) -> Self {
        Self { bytes, min_clock, req_clock, notify_id }
    }
    pub fn bytes(&self) -> &[u8] { &self.bytes }
    pub fn min_clock(&self) -> u64 { self.min_clock }
    pub fn req_clock(&self) -> u64 { self.req_clock }
    pub fn notify_id(&self) -> NotifyId { self.notify_id }
}

#[cfg(test)]
mod tests {
    // (test code from Step 1)
}
```

- [ ] **Step 4: Wire the module**

```rust
// rust/kalico-host-rt/src/passthrough_queue/mod.rs
//! Rust port of klippy/chelper/serialqueue.c.
//!
//! Per spec §3.5 / §3.5.1, this module owns:
//! - per-MCU command queues with upcoming/ready promotion by req_clock
//! - ready ordering by min_clock
//! - receive-window backpressure
//! - notify-id correlation for queries
//! - sent_time / receive_time annotation on responses
//!
//! Integrates with kalico-host-rt::host_io::reactor for wire transmit/receive.

mod entry;

pub use entry::{NotifyId, PassthroughEntry};
```

```rust
// rust/kalico-host-rt/src/lib.rs (add module declaration)
pub mod passthrough_queue;
```

- [ ] **Step 5: Verify the test passes**

Run: `cd rust && cargo test -p kalico-host-rt passthrough_queue::entry`

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add rust/kalico-host-rt/src/passthrough_queue/ rust/kalico-host-rt/src/lib.rs
git commit -m "feat(passthrough_queue): NotifyId + PassthroughEntry core types"
```

### Task 12: CommandQueue with min_clock-ordered ready queue

**Files:**
- Create: `rust/kalico-host-rt/src/passthrough_queue/command_queue.rs`
- Modify: `rust/kalico-host-rt/src/passthrough_queue/mod.rs`

**Source reference:** `klippy/chelper/serialqueue.c:744-805` (`serialqueue_alloc_commandqueue`, `serialqueue_send_batch`, `serialqueue_send`, `serialqueue_send_one`).

The C source has separate `upcoming_queue` and `ready_queue` per command queue. This task implements the `ready_queue` (entries ready to emit, ordered by `min_clock`). Upcoming-queue + promotion lands in Task 13.

- [ ] **Step 1: Write the failing test**

```rust
// at the bottom of command_queue.rs
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ready_queue_emits_in_min_clock_order() {
        let mut cq = CommandQueue::new();
        cq.push_ready(PassthroughEntry::new(vec![0x01], 100, 100, NotifyId::none()));
        cq.push_ready(PassthroughEntry::new(vec![0x02], 50, 50, NotifyId::none()));
        cq.push_ready(PassthroughEntry::new(vec![0x03], 200, 200, NotifyId::none()));

        // Ordered emission: 50, 100, 200.
        let first = cq.pop_ready_due(/* now_clock */ 1000).expect("entry");
        assert_eq!(first.bytes(), &[0x02]);
        let second = cq.pop_ready_due(1000).expect("entry");
        assert_eq!(second.bytes(), &[0x01]);
        let third = cq.pop_ready_due(1000).expect("entry");
        assert_eq!(third.bytes(), &[0x03]);
    }

    #[test]
    fn pop_ready_due_respects_min_clock() {
        let mut cq = CommandQueue::new();
        cq.push_ready(PassthroughEntry::new(vec![0xAA], /* min_clock */ 500, 500, NotifyId::none()));
        // now_clock < min_clock — entry is held.
        assert!(cq.pop_ready_due(/* now_clock */ 100).is_none());
        // now_clock ≥ min_clock — entry is released.
        let popped = cq.pop_ready_due(/* now_clock */ 600).expect("entry");
        assert_eq!(popped.bytes(), &[0xAA]);
    }

    #[test]
    fn empty_command_queue_returns_none() {
        let mut cq = CommandQueue::new();
        assert!(cq.pop_ready_due(0).is_none());
    }
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cd rust && cargo test -p kalico-host-rt passthrough_queue::command_queue`

Expected: FAIL.

- [ ] **Step 3: Implement `CommandQueue`**

```rust
// rust/kalico-host-rt/src/passthrough_queue/command_queue.rs
//! One per driver instance (TMC SPI, GPIO, etc.). Orders entries by
//! min_clock for ready emission. Upcoming → ready promotion lives in
//! the parent module's mcu_state once req_clock is implemented.
//!
//! Source: klippy/chelper/serialqueue.c:744-805.

use super::entry::PassthroughEntry;

/// Per-driver command queue.
///
/// Has two logical lists:
/// - `upcoming` — entries waiting on req_clock to become imminent (Task 13).
/// - `ready`    — entries ready to emit, ordered by min_clock.
///
/// Phase 1 implementation note: a Vec ordered by `min_clock` is fine for
/// realistic queue depths (klippy command queues rarely exceed dozens of
/// entries). If profiling shows hot O(n) inserts, swap for BinaryHeap.
#[derive(Debug, Default)]
pub struct CommandQueue {
    ready: Vec<PassthroughEntry>,
    // upcoming: Vec<PassthroughEntry>, // Task 13
}

impl CommandQueue {
    pub fn new() -> Self {
        Self::default()
    }

    /// Push an entry directly onto the ready queue (skipping upcoming).
    /// Used for entries with req_clock already past — i.e., emit ASAP.
    pub fn push_ready(&mut self, entry: PassthroughEntry) {
        // Insert in min_clock order.
        let pos = self
            .ready
            .iter()
            .position(|e| e.min_clock() > entry.min_clock())
            .unwrap_or(self.ready.len());
        self.ready.insert(pos, entry);
    }

    /// Pop the next entry whose min_clock has been reached, or `None` if
    /// the head entry is still scheduled in the future.
    pub fn pop_ready_due(&mut self, now_clock: u64) -> Option<PassthroughEntry> {
        if self.ready.first().map(|e| e.min_clock() <= now_clock).unwrap_or(false) {
            Some(self.ready.remove(0))
        } else {
            None
        }
    }

    pub fn ready_len(&self) -> usize {
        self.ready.len()
    }

    pub fn is_empty(&self) -> bool {
        self.ready.is_empty()
    }
}
```

- [ ] **Step 4: Wire into mod.rs**

```rust
// rust/kalico-host-rt/src/passthrough_queue/mod.rs
mod command_queue;
mod entry;

pub use command_queue::CommandQueue;
pub use entry::{NotifyId, PassthroughEntry};
```

- [ ] **Step 5: Run test to verify it passes**

Run: `cd rust && cargo test -p kalico-host-rt passthrough_queue::command_queue`

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add rust/kalico-host-rt/src/passthrough_queue/
git commit -m "feat(passthrough_queue): CommandQueue with min_clock ready ordering"
```

### Task 13: Upcoming queue + req_clock promotion

**Files:**
- Modify: `rust/kalico-host-rt/src/passthrough_queue/command_queue.rs`

**Source reference:** `klippy/chelper/serialqueue.c:455-520` (`build_and_send_command` upcoming/ready transition logic), `:789-836` (`serialqueue_send_batch` initial placement).

The C source places entries with `req_clock` in the future onto the upcoming queue; promotion to ready happens when `req_clock - lookahead_threshold <= now_clock`.

- [ ] **Step 1: Write the failing tests**

```rust
#[test]
fn upcoming_queue_holds_future_req_clock() {
    let mut cq = CommandQueue::new();
    cq.push(PassthroughEntry::new(vec![0xAA], /* min_clock */ 100, /* req_clock */ 1000, NotifyId::none()));

    // Far before req_clock — entry stays in upcoming.
    cq.promote_upcoming(/* now_clock */ 50, /* lookahead */ 100);
    assert_eq!(cq.upcoming_len(), 1);
    assert_eq!(cq.ready_len(), 0);

    // Within lookahead — entry promotes.
    cq.promote_upcoming(/* now_clock */ 950, /* lookahead */ 100);
    assert_eq!(cq.upcoming_len(), 0);
    assert_eq!(cq.ready_len(), 1);
}

#[test]
fn push_with_zero_req_clock_goes_straight_to_ready() {
    let mut cq = CommandQueue::new();
    cq.push(PassthroughEntry::new(vec![0xBB], 0, 0, NotifyId::none()));
    assert_eq!(cq.ready_len(), 1);
    assert_eq!(cq.upcoming_len(), 0);
}
```

- [ ] **Step 2: Implement upcoming queue + promotion**

Replace `push_ready` with a unified `push` method that routes to upcoming or ready based on `req_clock`. Add `promote_upcoming`. Remove the previous `push_ready` if no longer needed (keep public for tests if convenient).

```rust
// CommandQueue addition:

pub fn push(&mut self, entry: PassthroughEntry) {
    if entry.req_clock() == 0 {
        // No req_clock specified — emit ASAP, ordered by min_clock.
        self.push_ready_internal(entry);
    } else {
        // Insert into upcoming, ordered by req_clock.
        let pos = self
            .upcoming
            .iter()
            .position(|e| e.req_clock() > entry.req_clock())
            .unwrap_or(self.upcoming.len());
        self.upcoming.insert(pos, entry);
    }
}

/// Move entries from upcoming → ready when req_clock - lookahead <= now_clock.
/// Should be called from the reactor tick before pop_ready_due.
pub fn promote_upcoming(&mut self, now_clock: u64, lookahead_clock: u64) {
    while let Some(head) = self.upcoming.first() {
        if head.req_clock().saturating_sub(lookahead_clock) <= now_clock {
            let entry = self.upcoming.remove(0);
            self.push_ready_internal(entry);
        } else {
            break;
        }
    }
}

fn push_ready_internal(&mut self, entry: PassthroughEntry) {
    let pos = self
        .ready
        .iter()
        .position(|e| e.min_clock() > entry.min_clock())
        .unwrap_or(self.ready.len());
    self.ready.insert(pos, entry);
}

pub fn upcoming_len(&self) -> usize {
    self.upcoming.len()
}
```

Add the field:

```rust
pub struct CommandQueue {
    upcoming: Vec<PassthroughEntry>,
    ready: Vec<PassthroughEntry>,
}
```

- [ ] **Step 3: Run tests**

Run: `cd rust && cargo test -p kalico-host-rt passthrough_queue::command_queue`

Expected: PASS (both new tests + the prior tests still passing).

- [ ] **Step 4: Commit**

```bash
git add rust/kalico-host-rt/src/passthrough_queue/command_queue.rs
git commit -m "feat(passthrough_queue): upcoming queue + req_clock promotion"
```

### Task 14: McuState — owns multiple CommandQueues per MCU

**Files:**
- Create: `rust/kalico-host-rt/src/passthrough_queue/mcu_state.rs`
- Modify: `rust/kalico-host-rt/src/passthrough_queue/mod.rs`

**Source reference:** `klippy/chelper/serialqueue.c:46-100` (struct serialqueue), `:636-710` (alloc/init).

Each motion or non-motion MCU owns its own set of `CommandQueue` instances. The `McuState` is the top-level Rust struct corresponding to a single `serialqueue` allocation in C.

- [ ] **Step 1: Write the failing test**

```rust
// rust/kalico-host-rt/src/passthrough_queue/mcu_state.rs (test module)
#[cfg(test)]
mod tests {
    use super::*;
    use crate::passthrough_queue::{CommandQueue, NotifyId, PassthroughEntry};

    #[test]
    fn mcu_state_allocates_command_queues() {
        let mut state = McuState::new();
        let q1 = state.alloc_command_queue();
        let q2 = state.alloc_command_queue();
        assert_ne!(q1, q2);
    }

    #[test]
    fn mcu_state_dispatches_send_to_correct_queue() {
        let mut state = McuState::new();
        let q1 = state.alloc_command_queue();
        let q2 = state.alloc_command_queue();

        state.push(q1, PassthroughEntry::new(vec![0xA1], 0, 0, NotifyId::none()));
        state.push(q2, PassthroughEntry::new(vec![0xB2], 0, 0, NotifyId::none()));

        assert_eq!(state.command_queue(q1).unwrap().ready_len(), 1);
        assert_eq!(state.command_queue(q2).unwrap().ready_len(), 1);
    }

    #[test]
    fn mcu_state_round_robin_pop_across_queues() {
        let mut state = McuState::new();
        let q1 = state.alloc_command_queue();
        let q2 = state.alloc_command_queue();

        state.push(q1, PassthroughEntry::new(vec![0xA1], 100, 0, NotifyId::none()));
        state.push(q2, PassthroughEntry::new(vec![0xB2], 50, 0, NotifyId::none()));

        // Earliest min_clock across all queues wins.
        let popped = state.pop_next_due(/* now_clock */ 1000).expect("entry");
        assert_eq!(popped.bytes(), &[0xB2]);
        let popped = state.pop_next_due(/* now_clock */ 1000).expect("entry");
        assert_eq!(popped.bytes(), &[0xA1]);
    }
}
```

- [ ] **Step 2: Implement `McuState`**

```rust
// rust/kalico-host-rt/src/passthrough_queue/mcu_state.rs

use super::{CommandQueue, PassthroughEntry};
use indexmap::IndexMap;

/// Opaque handle for a command queue within an MCU.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct CommandQueueId(u32);

/// All passthrough state for one MCU.
/// Mirrors klippy/chelper/serialqueue.c struct serialqueue.
#[derive(Debug, Default)]
pub struct McuState {
    queues: IndexMap<CommandQueueId, CommandQueue>,
    next_id: u32,
}

impl McuState {
    pub fn new() -> Self {
        Self::default()
    }

    /// Allocate a new command queue (klippy serialqueue_alloc_commandqueue).
    pub fn alloc_command_queue(&mut self) -> CommandQueueId {
        let id = CommandQueueId(self.next_id);
        self.next_id = self.next_id.checked_add(1).expect("CommandQueueId exhausted");
        self.queues.insert(id, CommandQueue::new());
        id
    }

    /// Push an entry onto a specific command queue.
    pub fn push(&mut self, queue: CommandQueueId, entry: PassthroughEntry) {
        if let Some(q) = self.queues.get_mut(&queue) {
            q.push(entry);
        } else {
            log::warn!("push to unknown CommandQueueId {:?}", queue);
        }
    }

    /// Promote upcoming entries to ready across all queues.
    /// Should be called from the reactor tick.
    pub fn promote_upcoming(&mut self, now_clock: u64, lookahead_clock: u64) {
        for q in self.queues.values_mut() {
            q.promote_upcoming(now_clock, lookahead_clock);
        }
    }

    /// Pop the next entry due across all queues, choosing the smallest
    /// min_clock as the tie-breaker. Returns None if no entry is due.
    pub fn pop_next_due(&mut self, now_clock: u64) -> Option<PassthroughEntry> {
        // Find queue with the smallest ready-head min_clock that is ≤ now_clock.
        let chosen = self
            .queues
            .iter()
            .filter_map(|(id, q)| {
                q.peek_ready_min_clock()
                    .filter(|&c| c <= now_clock)
                    .map(|c| (c, *id))
            })
            .min_by_key(|(c, _)| *c)
            .map(|(_, id)| id);

        chosen.and_then(|id| self.queues.get_mut(&id)?.pop_ready_due(now_clock))
    }

    /// Read-only access for tests.
    pub fn command_queue(&self, queue: CommandQueueId) -> Option<&CommandQueue> {
        self.queues.get(&queue)
    }

    pub fn queue_count(&self) -> usize {
        self.queues.len()
    }
}
```

Add to `CommandQueue`:

```rust
pub fn peek_ready_min_clock(&self) -> Option<u64> {
    self.ready.first().map(|e| e.min_clock())
}
```

- [ ] **Step 3: Wire mod.rs**

```rust
// rust/kalico-host-rt/src/passthrough_queue/mod.rs
mod command_queue;
mod entry;
mod mcu_state;

pub use command_queue::CommandQueue;
pub use entry::{NotifyId, PassthroughEntry};
pub use mcu_state::{CommandQueueId, McuState};
```

- [ ] **Step 4: Run tests**

Run: `cd rust && cargo test -p kalico-host-rt passthrough_queue::mcu_state`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/kalico-host-rt/src/passthrough_queue/
git commit -m "feat(passthrough_queue): McuState owns multiple CommandQueues"
```

### Task 15: NotifyTable — correlate notify_id ↔ pending callback

**Files:**
- Create: `rust/kalico-host-rt/src/passthrough_queue/notify.rs`

**Source reference:** `klippy/chelper/serialqueue.c:222-300` (`handle_message` notify dispatch), `:838-852` (`serialqueue_send` notify allocation).

For queries (`send_with_response` / `send_wait_ack`): caller registers a callback indexed by `NotifyId`; when a matching response arrives, dispatch fires the callback once and forgets the entry.

- [ ] **Step 1: Write the failing test**

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicU32, Ordering};
    use std::sync::Arc;

    #[test]
    fn notify_table_dispatch_fires_once() {
        let counter = Arc::new(AtomicU32::new(0));
        let counter_cb = counter.clone();

        let mut table = NotifyTable::new();
        let id = table.register(Box::new(move |_resp| {
            counter_cb.fetch_add(1, Ordering::SeqCst);
        }));

        // Dispatch matching id.
        table.dispatch(id, NotifyResponse::default());
        assert_eq!(counter.load(Ordering::SeqCst), 1);

        // Second dispatch is a no-op (already consumed).
        table.dispatch(id, NotifyResponse::default());
        assert_eq!(counter.load(Ordering::SeqCst), 1);
    }

    #[test]
    fn notify_table_unique_ids() {
        let mut table = NotifyTable::new();
        let id_a = table.register(Box::new(|_| {}));
        let id_b = table.register(Box::new(|_| {}));
        assert_ne!(id_a, id_b);
    }
}
```

- [ ] **Step 2: Implement `NotifyTable`**

```rust
// rust/kalico-host-rt/src/passthrough_queue/notify.rs

use super::entry::NotifyId;
use std::collections::HashMap;

/// Response payload delivered to a notify callback.
/// Phase 1 minimal shape; extend with sent/receive timestamps in Task 19.
#[derive(Debug, Clone, Default)]
pub struct NotifyResponse {
    pub bytes: Vec<u8>,
    pub sent_time: f64,
    pub receive_time: f64,
}

pub type NotifyCallback = Box<dyn FnOnce(NotifyResponse) + Send>;

#[derive(Default)]
pub struct NotifyTable {
    pending: HashMap<NotifyId, NotifyCallback>,
    next_id: u64,
}

impl std::fmt::Debug for NotifyTable {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("NotifyTable")
            .field("pending_count", &self.pending.len())
            .field("next_id", &self.next_id)
            .finish()
    }
}

impl NotifyTable {
    pub fn new() -> Self {
        Self::default()
    }

    /// Register a one-shot callback; returns the NotifyId to attach to the
    /// outgoing PassthroughEntry.
    pub fn register(&mut self, cb: NotifyCallback) -> NotifyId {
        self.next_id = self.next_id.checked_add(1).expect("NotifyId exhausted");
        let id = NotifyId::new(self.next_id);
        self.pending.insert(id, cb);
        id
    }

    /// Dispatch a response for the given id. No-op if id is unknown
    /// (already-consumed or never-registered notify).
    pub fn dispatch(&mut self, id: NotifyId, response: NotifyResponse) {
        if let Some(cb) = self.pending.remove(&id) {
            cb(response);
        }
    }

    pub fn pending_count(&self) -> usize {
        self.pending.len()
    }
}
```

Wire into mod.rs:

```rust
mod notify;
pub use notify::{NotifyCallback, NotifyResponse, NotifyTable};
```

- [ ] **Step 3: Run tests**

Run: `cd rust && cargo test -p kalico-host-rt passthrough_queue::notify`

Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add rust/kalico-host-rt/src/passthrough_queue/
git commit -m "feat(passthrough_queue): NotifyTable for query/response correlation"
```

### Task 16: Receive-window backpressure

**Files:**
- Create: `rust/kalico-host-rt/src/passthrough_queue/receive_window.rs`

**Source reference:** `klippy/chelper/serialqueue.c:135-160` (calculate_bittime, kick_bg_thread receive_window check), `:903-913` (`serialqueue_set_receive_window`).

Each MCU has a receive-window limit (in bytes) — total bytes in flight cannot exceed this without ack from MCU. Bridge stops emitting when full.

- [ ] **Step 1: Write the failing test**

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn receive_window_blocks_when_full() {
        let mut window = ReceiveWindow::new(/* limit_bytes */ 100);
        assert!(window.try_charge(60));
        assert!(window.try_charge(30));
        // 90 of 100 used; 30 more would exceed.
        assert!(!window.try_charge(30));
        // Acked 60 → only 30 used.
        window.ack(60);
        // Now there's 70 free.
        assert!(window.try_charge(30));
    }

    #[test]
    fn receive_window_default_starts_empty() {
        let window = ReceiveWindow::new(100);
        assert_eq!(window.in_flight_bytes(), 0);
        assert_eq!(window.limit_bytes(), 100);
    }
}
```

- [ ] **Step 2: Implement**

```rust
// rust/kalico-host-rt/src/passthrough_queue/receive_window.rs

#[derive(Debug)]
pub struct ReceiveWindow {
    limit: usize,
    in_flight: usize,
}

impl ReceiveWindow {
    pub fn new(limit_bytes: usize) -> Self {
        Self { limit: limit_bytes, in_flight: 0 }
    }

    /// Try to reserve `bytes` of in-flight capacity. Returns true if
    /// reservation succeeded, false if it would exceed the window.
    pub fn try_charge(&mut self, bytes: usize) -> bool {
        if self.in_flight + bytes > self.limit {
            return false;
        }
        self.in_flight += bytes;
        true
    }

    /// Release `bytes` of in-flight capacity (called on ack from MCU).
    pub fn ack(&mut self, bytes: usize) {
        self.in_flight = self.in_flight.saturating_sub(bytes);
    }

    pub fn in_flight_bytes(&self) -> usize { self.in_flight }
    pub fn limit_bytes(&self) -> usize { self.limit }
    pub fn set_limit(&mut self, new_limit: usize) { self.limit = new_limit; }
}
```

Wire into mod.rs:

```rust
mod receive_window;
pub use receive_window::ReceiveWindow;
```

- [ ] **Step 3: Run tests**

Run: `cd rust && cargo test -p kalico-host-rt passthrough_queue::receive_window`

Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add rust/kalico-host-rt/src/passthrough_queue/
git commit -m "feat(passthrough_queue): receive-window backpressure"
```

### Task 17: Integrate passthrough_queue into McuState (window + notify table)

**Files:**
- Modify: `rust/kalico-host-rt/src/passthrough_queue/mcu_state.rs`

- [ ] **Step 1: Add fields**

```rust
pub struct McuState {
    queues: IndexMap<CommandQueueId, CommandQueue>,
    notify_table: NotifyTable,
    receive_window: ReceiveWindow,
    next_id: u32,
}

impl McuState {
    pub fn new() -> Self {
        Self {
            queues: IndexMap::new(),
            notify_table: NotifyTable::new(),
            // Phase 1 default; klippy serialqueue.c uses 64 KB conservatively.
            receive_window: ReceiveWindow::new(64 * 1024),
            next_id: 0,
        }
    }

    pub fn notify_table(&mut self) -> &mut NotifyTable { &mut self.notify_table }
    pub fn receive_window(&mut self) -> &mut ReceiveWindow { &mut self.receive_window }
}
```

- [ ] **Step 2: Update `pop_next_due` to respect window**

```rust
pub fn pop_next_due(&mut self, now_clock: u64) -> Option<PassthroughEntry> {
    let chosen = /* same as before */;
    let entry = chosen.and_then(|id| self.queues.get_mut(&id)?.pop_ready_due(now_clock))?;

    // Charge against receive window. If charge fails, push entry back.
    if !self.receive_window.try_charge(entry.bytes().len()) {
        // Push back onto a synthetic "head" position; for simplicity, re-push;
        // ordering invariant preserved because min_clock is unchanged.
        self.queues
            .get_mut(&chosen.unwrap())
            .expect("queue exists")
            .push_ready_internal(entry);
        return None;
    }
    Some(entry)
}
```

- [ ] **Step 3: Add a test**

```rust
#[test]
fn pop_blocked_by_full_receive_window() {
    let mut state = McuState::new();
    state.receive_window().set_limit(10);
    let q = state.alloc_command_queue();
    state.push(q, PassthroughEntry::new(vec![0u8; 8], 100, 0, NotifyId::none()));
    state.push(q, PassthroughEntry::new(vec![0u8; 8], 200, 0, NotifyId::none()));

    let first = state.pop_next_due(1000).expect("first fits");
    assert_eq!(first.bytes().len(), 8);

    // Window is 8/10 used; second 8-byte entry won't fit.
    assert!(state.pop_next_due(1000).is_none());

    // Ack first → window has 8 free again.
    state.receive_window().ack(8);
    let second = state.pop_next_due(1000).expect("second fits after ack");
    assert_eq!(second.bytes().len(), 8);
}
```

- [ ] **Step 4: Make `push_ready_internal` accessible to McuState**

Either change visibility (`pub(crate) fn push_ready_internal` in command_queue.rs) or expose a `push_back_to_ready_head` helper. Pick one.

- [ ] **Step 5: Run tests**

Run: `cd rust && cargo test -p kalico-host-rt passthrough_queue`

Expected: all PASS.

- [ ] **Step 6: Commit**

```bash
git add rust/kalico-host-rt/src/passthrough_queue/
git commit -m "feat(passthrough_queue): integrate receive_window + NotifyTable into McuState"
```

### Task 18: Per-MCU registry — PassthroughRouter

**Files:**
- Create: `rust/kalico-host-rt/src/passthrough_queue/router.rs`
- Modify: `rust/kalico-host-rt/src/passthrough_queue/mod.rs`

The router owns one `McuState` per claimed MCU and routes `passthrough_send`/`query` calls to the right one. This is the boundary the bridge will call.

- [ ] **Step 1: Write the failing test**

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn router_claims_and_dispatches() {
        let mut router = PassthroughRouter::new();
        let mcu_a = router.claim_mcu("mock-a");
        let mcu_b = router.claim_mcu("mock-b");
        assert_ne!(mcu_a, mcu_b);

        let q_a = router.alloc_command_queue(mcu_a).expect("alloc");
        router
            .push(mcu_a, q_a, PassthroughEntry::new(vec![0xAA], 0, 0, NotifyId::none()))
            .expect("push");

        let popped = router.pop_next_due(mcu_a, 1000).expect("Some");
        assert_eq!(popped.bytes(), &[0xAA]);

        // mcu_b is empty.
        assert!(router.pop_next_due(mcu_b, 1000).is_none());
    }

    #[test]
    fn router_release_removes_state() {
        let mut router = PassthroughRouter::new();
        let mcu = router.claim_mcu("mock");
        assert!(router.release_mcu(mcu));
        assert!(!router.release_mcu(mcu)); // already released
    }
}
```

- [ ] **Step 2: Implement `PassthroughRouter`**

```rust
// rust/kalico-host-rt/src/passthrough_queue/router.rs

use super::{CommandQueueId, McuState, PassthroughEntry};
use indexmap::IndexMap;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct McuHandle(u32);

#[derive(Debug, thiserror::Error)]
pub enum RouterError {
    #[error("unknown MCU handle {0:?}")]
    UnknownMcu(McuHandle),
    #[error("unknown CommandQueueId {0:?} on MCU {1:?}")]
    UnknownQueue(CommandQueueId, McuHandle),
}

#[derive(Default)]
pub struct PassthroughRouter {
    mcus: IndexMap<McuHandle, McuState>,
    next_handle: u32,
    /// Debug label per MCU (e.g., serial path).
    labels: IndexMap<McuHandle, String>,
}

impl PassthroughRouter {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn claim_mcu(&mut self, label: impl Into<String>) -> McuHandle {
        let handle = McuHandle(self.next_handle);
        self.next_handle = self.next_handle.checked_add(1).expect("McuHandle exhausted");
        self.mcus.insert(handle, McuState::new());
        self.labels.insert(handle, label.into());
        handle
    }

    pub fn release_mcu(&mut self, mcu: McuHandle) -> bool {
        self.mcus.shift_remove(&mcu).is_some() && self.labels.shift_remove(&mcu).is_some()
    }

    pub fn alloc_command_queue(&mut self, mcu: McuHandle) -> Result<CommandQueueId, RouterError> {
        let state = self.mcus.get_mut(&mcu).ok_or(RouterError::UnknownMcu(mcu))?;
        Ok(state.alloc_command_queue())
    }

    pub fn push(
        &mut self,
        mcu: McuHandle,
        queue: CommandQueueId,
        entry: PassthroughEntry,
    ) -> Result<(), RouterError> {
        let state = self.mcus.get_mut(&mcu).ok_or(RouterError::UnknownMcu(mcu))?;
        state.push(queue, entry);
        Ok(())
    }

    pub fn pop_next_due(&mut self, mcu: McuHandle, now_clock: u64) -> Option<PassthroughEntry> {
        self.mcus.get_mut(&mcu)?.pop_next_due(now_clock)
    }

    pub fn mcu_state(&mut self, mcu: McuHandle) -> Option<&mut McuState> {
        self.mcus.get_mut(&mcu)
    }
}
```

Add `thiserror` to `kalico-host-rt`'s Cargo.toml dependencies:

```toml
thiserror = "1"
```

(If thiserror isn't already a workspace dep, propagate. If the project prefers manual error enums, swap to a hand-rolled `Display`+`std::error::Error` impl.)

- [ ] **Step 3: Wire mod.rs**

```rust
mod router;
pub use router::{McuHandle, PassthroughRouter, RouterError};
```

- [ ] **Step 4: Run tests**

Run: `cd rust && cargo test -p kalico-host-rt passthrough_queue`

Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/kalico-host-rt/Cargo.toml rust/kalico-host-rt/src/passthrough_queue/
git commit -m "feat(passthrough_queue): PassthroughRouter — per-MCU registry"
```

### Task 19: Sent_time / receive_time annotation

**Files:**
- Modify: `rust/kalico-host-rt/src/passthrough_queue/mcu_state.rs`
- Modify: `rust/kalico-host-rt/src/passthrough_queue/entry.rs`

**Source reference:** `klippy/chelper/serialqueue.c:300-356` (`input_event` annotation), `:455-520` (`build_and_send_command` sent_time stamp).

Klippy response handlers receive `#sent_time` and `#receive_time` annotations on the parsed dict. The bridge must record sent time on emission and receive time on response arrival, then stamp both onto the `NotifyResponse` (and onto unsolicited responses fed into the dispatch handler).

- [ ] **Step 1: Add timestamp recording on emission**

When `pop_next_due` returns an entry, record the current host-side wall-clock time in a side table keyed by the message's seq. (The `seq` is assigned by the host_io reactor when it transmits.)

- [ ] **Step 2: Add timestamp lookup on response**

When a response arrives via the host_io parser, look up the original send time by seq, compute `receive_time` from current wall-clock, and pass both to the `NotifyTable.dispatch()` (or to the unsolicited-response path).

- [ ] **Step 3: Write a test**

(Use a `Clock` trait from 7-C-io tail's existing seam to inject deterministic time.)

```rust
#[test]
fn sent_and_receive_times_propagate() {
    use crate::clock::MockClock;
    let clock = MockClock::new();
    let mut router = PassthroughRouter::with_clock(clock.clone());
    let mcu = router.claim_mcu("mock");
    let q = router.alloc_command_queue(mcu).unwrap();

    let received: std::sync::Arc<std::sync::Mutex<Option<NotifyResponse>>> =
        std::sync::Arc::new(std::sync::Mutex::new(None));
    let received_cb = received.clone();
    let id = router.register_notify(mcu, Box::new(move |resp| {
        *received_cb.lock().unwrap() = Some(resp);
    })).unwrap();

    clock.advance_secs(1.0);
    router.push(mcu, q, PassthroughEntry::new(vec![0xAA], 0, 0, id)).unwrap();
    let entry = router.pop_next_due(mcu, 1000).expect("entry");
    let recorded_sent = router.peek_sent_time(mcu, entry.notify_id()).expect("sent_time");
    assert!((recorded_sent - 1.0).abs() < 1e-9);

    clock.advance_secs(0.5);
    router.dispatch_response(mcu, entry.notify_id(), vec![]);
    let resp = received.lock().unwrap().take().expect("dispatched");
    assert!((resp.sent_time - 1.0).abs() < 1e-9);
    assert!((resp.receive_time - 1.5).abs() < 1e-9);
}
```

(Adjust `Clock` integration to match the existing seam in `kalico-host-rt::clock`.)

- [ ] **Step 4: Run tests**

Run: `cd rust && cargo test -p kalico-host-rt passthrough_queue`

Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/kalico-host-rt/src/passthrough_queue/
git commit -m "feat(passthrough_queue): sent/receive timestamp annotation"
```

### Tasks 20-25: Remaining serialqueue.c port pieces

For each of these, follow the same TDD pattern as Tasks 11-19: write a Rust-native test pinning down the externally-observable behavior, port the C logic, verify the test passes, commit.

- [ ] **Task 20: Flush callbacks** (`serialqueue.c:622-636` background thread "all queues drained" notification). Bridge fires registered Python callbacks via the event_fd queue when an MCU's queues all reach empty.
- [ ] **Task 21: Stats / `serialqueue_get_stats` parity** (`:936-958`). Per-MCU counter struct (bytes sent/received, ack count, retransmits, NAKs, queue high-water marks). Exposed to Python for the periodic klippy stats line.
- [ ] **Task 22: `serialqueue_extract_old`** (`:958-992`). Used by klippy debug-only paths to read out the in-flight queue. Phase 1: implement minimally (return all entries past a given seq); detailed shape per klippy's `pull_queue_message` consumers.
- [ ] **Task 23: `serialqueue_set_clock_est` / `set_wire_frequency`** (`:890-927`). Wires per-MCU clock-sync state into the queue scheduler. Reuse the existing `kalico-host-rt::clock_sync` machinery.
- [ ] **Task 24: Identify-time config commands** (`add_config_cmd`). Entries flagged as init-stage emit exactly once at MCU restart, before any runtime traffic.
- [ ] **Task 25: Reactor integration** — wire `PassthroughRouter` into `kalico-host-rt::host_io::reactor::tick_once`. On each tick: promote upcoming, pop next due across all MCUs, hand bytes to the existing wire framer, parse responses via the existing parser, dispatch into NotifyTable. Reuse 7-C-io tail's `Clock` trait + `tick_once` seam.

For each: small commit, full test coverage, docstrings referencing the C source line range. The work is bounded; don't expand scope.

### Tasks 26-30: PassthroughRouter integration tests

- [ ] **Task 26:** End-to-end test using `MockTransport` (already exists) — push entries, drive the reactor, verify they're emitted in min_clock order.
- [ ] **Task 27:** Notify-id correlation through the full path — push a query with notify_id, simulate response, verify callback fires with correct sent/receive times.
- [ ] **Task 28:** Receive-window-blocked test — fill the window, verify emission stops, ack the in-flight bytes, verify emission resumes.
- [ ] **Task 29:** Multi-MCU test — claim two MCUs, exercise both concurrently, verify isolation (no cross-talk).
- [ ] **Task 30:** Identify-stage commands test — register init commands, drive identify, verify they're emitted before runtime traffic.

---

## Stage C — `motion-bridge` PyO3 surface (Tasks 31-40)

The bridge crate exposes the §3.5.1 parity matrix to Python.

- [ ] **Task 31:** `MotionBridge::new(event_fd, ...)` — initializes the `PassthroughRouter`, spawns the host_io reactor thread, registers the event_fd write end.
- [ ] **Task 32:** `claim_mcu(serial_path, baud, mcu_type) -> McuHandle` — opens the serial fd via existing `kalico-host-rt::transport`, runs identify, returns handle. Phase 1: use the existing `KalicoHostIo::open` flow; `mcu_type` is currently informational.
- [ ] **Task 33:** `release_mcu(handle)` — disconnect, free state.
- [ ] **Task 34:** `alloc_command_queue(handle) -> CommandQueueId`.
- [ ] **Task 35:** `passthrough_send(mcu, queue, bytes, min_clock=0, req_clock=0, notify_id=0)` — enqueue.
- [ ] **Task 36:** `passthrough_query(mcu, queue, bytes, response_name, oid, min_clock=, req_clock=) -> notify_id` — register handler then enqueue with notify_id.
- [ ] **Task 37:** `passthrough_send_wait_ack(mcu, queue, bytes, timeout) -> bytes` — synchronous; releases GIL while waiting.
- [ ] **Task 38:** `passthrough_register_handler(mcu, name, oid, callback)` — typed-response registration with `#sent_time` / `#receive_time` / `#oid` annotation injection.
- [ ] **Task 39:** `passthrough_register_flush_callback(mcu, callback)` — fires when MCU's queues drain.
- [ ] **Task 40:** `poll_event() -> Option<EventDict>` — drain side; reactor pushes events here, klippy drains via the registered fd.

For each task: PyO3 method definition + a Python pytest hitting it through the smoke harness from Task 4. Each task ends with a commit.

---

## Stage D — Klippy-side patches (Tasks 41-55)

This stage gets klippy actually using the bridge. Order matters: the most foundational patches first (printer.py instantiates bridge → mcu.py allocates proxy → stepper.py + heaters reach setpoint). Each task ends with a smoke-test verification before commit.

- [ ] **Task 41:** `klippy/motion_bridge.py` Python wrapper — opens the event-fd pipe, instantiates the PyO3 `MotionBridge`, registers the read end with `reactor.register_fd`.
- [ ] **Task 42:** `klippy/printer.py` — instantiate the bridge during connection setup, before MCU objects.
- [ ] **Task 43:** `klippy/motion_mcu.py` — `MotionMcuProxy` class implementing the public surface listed in spec §3.5.1 + §3.6 (`lookup_command`, `lookup_query_command`, `add_config_cmd`, `register_response`, `register_flush_callback`, `alloc_command_queue`, `estimated_print_time`, `print_time_to_clock`, `clock_to_print_time`, `seconds_to_clock`, `clock_to_seconds`, `is_fileoutput`, `is_shutdown`, `get_constants`, `create_oid`, `get_status`, etc.). Each method delegates to the bridge.
- [ ] **Task 44:** Patch `klippy/mcu.py` — constructor branches: instead of `serialqueue_alloc` + opening fd, allocates a `MotionMcuProxy` for any `[mcu*]` config. Make this gating explicit (eg, `if printer.lookup_object('motion_bridge', None)`). Test: import klippy with the user's config — heaters and TMC config commands flow through the proxy.
- [ ] **Task 45:** Patch `klippy/serialhdl.py` — gut the C-side serialqueue allocation. Decision: keep the file as a thin wrapper over `motion_mcu.py` that preserves the `SerialReader` API surface for any existing direct consumer, OR delete `serialhdl.py` outright and migrate any direct consumers to `motion_mcu.py`. Pick the smaller-diff option.
- [ ] **Task 46:** Patch `klippy/stepper.py` — preserve `PrinterStepper` / `MCU_stepper` / `PrinterRail` config-object surface per §5.2; gut motion internals; route `set_trapq` / `setup_itersolve` / `set_stepper_kinematics` to bridge stub methods (Phase 1: record-only, no runtime motion).
- [ ] **Task 47:** Patch `klippy/kinematics/extruder.py` — keep `PrinterExtruder` / `ExtruderStepper` / `cmd_SET_PRESSURE_ADVANCE` / `cmd_SYNC_EXTRUDER_MOTION` per §5.2; PA params no-op; route to bridge stubs.
- [ ] **Task 48:** Patch `klippy/kinematics/idex_modes.py` — refuse runtime mode switches with "not yet supported" error.
- [ ] **Task 49:** Patch `klippy/extras/motion_report.py` — drop trapq dump endpoint; preserve `trapqs` dict shape backed by bridge state queries (Phase 1: empty stub returning current bridge state).
- [ ] **Task 50:** Patch `klippy/extras/input_shaper.py` — drop trapezoidal IS C path; convert to ShaperSpec config-parser; `SET_INPUT_SHAPER` raises "not yet supported until Phase 3".
- [ ] **Task 51:** Stub `klippy/motion_toolhead.py` — implement the §3.6.2 compatibility matrix at scaffold level. Methods that don't yet have a real bridge backing (Phase 1) raise `NotImplementedError("not yet supported until Phase 2")` for any move-issuing call. Methods that work (`get_kinematics`, `get_status`, `get_extruder`, `get_last_move_time` returning a sensible default until motion lands) work now.
- [ ] **Task 52:** Stub `klippy/motion_kinematics.py` — Cartesian + CoreXY config parsers; emit `KinematicsSpec` to bridge. No runtime motion logic.
- [ ] **Task 53:** Stub `mcu.MCU_trsync` — class refuses to arm; raises during homing. (`G28` will hit this.)
- [ ] **Task 54:** Hard-disable list patches per spec §5.3 — for each module in §5.3 (mixing_extruder, trad_rack, pwm_tool, manual_stepper, force_move, z_tilt, z_tilt_ng, homing, load_cell), patch the config-loader to raise a clear "not yet supported" error if the user has them enabled.
- [ ] **Task 55:** Preserve the `gcode_arcs` configuration error per spec §4.3 — config-loader raises "remove `[gcode_arcs]` from your config" error.

---

## Stage E — Deletion sweep (Tasks 56-60)

Done **after** Stage D so klippy already imports cleanly with the new code path. Each deletion is a separate commit.

- [ ] **Task 56:** `git rm klippy/toolhead.py` — verify no remaining imports.
- [ ] **Task 57:** `git rm klippy/kinematics/cartesian.py corexy.py corexz.py cartesian_abc.py delta.py deltesian.py polar.py rotary_delta.py winch.py hybrid_corexy.py hybrid_corexz.py limited_cartesian.py limited_corexy.py limited_corexz.py none.py` — verify no remaining imports.
- [ ] **Task 58:** `git rm klippy/extras/gcode_arcs.py`.
- [ ] **Task 59:** `git rm klippy/chelper/itersolve.* stepcompress.* serialqueue.* trapq.c trapq.h trdispatch.c kin_*.c` — and remove their cffi declarations from `klippy/chelper/__init__.py`.
- [ ] **Task 60:** Re-run klippy boot smoke test — verify nothing broke.

---

## Stage F — Smoke test under kalico-sim (Tasks 61-65)

End-to-end Phase 1 verification: klippy boots against the user's Trident config (or a sanitized version), heaters reach setpoint, no motion attempted.

- [ ] **Task 61:** Set up `kalico-sim` config that mimics the user's MCU layout (one main motion MCU + one bottom + one beacon + one NIS, all served via `kalico-sim`'s host-process MCU sim).
- [ ] **Task 62:** Build a minimal `printer.cfg` derived from `~/printer_data/config/printer.cfg` (sanitized) that exercises: `[mcu]`, `[mcu bottom]`, `[beacon]`, `[stepper_x/y/z]`, `[extruder]`, `[heater_bed]`, `[input_shaper]`, `[tmc5160 stepper_x]`, `[fan]`, etc.
- [ ] **Task 63:** Write `tests/motion_bridge/test_klippy_boot.py` — pytest that spawns klippy with the smoke config under `kalico-sim`, waits for "ready" on the API, issues `M105`/`M104 S60`, verifies the bed heater PID actually drives temp toward setpoint in the sim.
- [ ] **Task 64:** Run the full boot test:

```bash
make motion-bridge
cd tests && python3 -m pytest motion_bridge/test_klippy_boot.py -v
```

Expected: PASS — klippy boots, configures TMCs (verified by checking SPI register-write commands appear on the wire), reads thermistors (verified by temp readback), drives heater PWM. Beacon's MCU enumerates and reports its initial temperature read.

- [ ] **Task 65:** Commit the smoke test fixtures + harness.

```bash
git add tests/motion_bridge/
git commit -m "test(motion-bridge): klippy boot + heater smoke under kalico-sim"
```

---

## Phase 1 Done

After Task 65: klippy boots, heaters reach setpoint, beacon enumerates, no trapezoidal motion code remains in the repo. The new wire ownership is in place. Move to Phase 2's plan (first motion).

**Don't start Phase 2 until:**
- All Phase 1 tests green in CI
- Audit script (Phase 6) is **not** required yet — it lands at the end. Phase 1 verification is the boot smoke test plus the Rust unit tests.
- The user has had a chance to review the structural changes and confirm direction.

---

## Self-review notes

**Spec coverage:**
- §1.1, §1.4, §2, §2.2, §2.3, §3.5, §3.5.1: covered by Stages A–C.
- §3.6.2 motion_toolhead matrix: scaffolded in Task 51 (full implementation lands in subsequent phases).
- §3.8 print-time semantics: Phase 1 only exercises the deterministic `motion_mcu.estimated_print_time` / `print_time_to_clock` / `clock_to_print_time` path (Task 23, Task 43). Provisional/finalized handle semantics land in Phase 3 with first motion.
- §4 config translation: §4.3 hard-error knobs covered by Task 55 + Task 54. Other §4 sections cover later phases.
- §5 deletion + patch list: covered by Stages D + E.
- §6 phasing: this plan IS Phase 1; Phases 2-6 get their own plans.
- §9 doc amendments: covered by Tasks 6-9.

**Out-of-scope-for-this-plan (intentional):**
- All motion submission (`submit_move`, `submit_g1`, etc.) — Phase 2.
- TOPP-RA + shaper bake — Phase 3.
- Homing — Phase 4.
- Ring 2 per-stepper override — Phase 5.
- Audit script — Phase 6.

**Known speculative decisions** (engineer may revise during execution):
- `klippy/serialhdl.py` keep-thin-wrapper vs delete-outright (Task 45).
- `klippy/mcu.py` constructor branch vs separate `motion_mcu.py` factory (Tasks 43-44).
- `compat` crate split vs single-crate-with-gated-text-IO (deferred to Phase 2 first dependency).
- Receive-window default size (Task 16) — picked 64KB conservatively; tune per `klippy/chelper/serialqueue.c`'s actual default.
