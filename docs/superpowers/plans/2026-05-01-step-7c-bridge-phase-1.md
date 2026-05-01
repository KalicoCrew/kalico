# Step 7-C-bridge Phase 1: scaffold + delete + all-MCU passthrough router

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Klippy boots against the user's Trident config, configures TMC drivers across all MCUs, reads thermistors on Octopus + F446 + frame, drives heaters (extruder + bed), enumerates beacon and NIS — all through a new Rust-side passthrough router replacing `serialqueue.c`. No motion possible (homing returns "not yet implemented"). Phase 1 ends with the new motion path's wire ownership in place; Phase 2 adds first motion.

**Architecture:** Add a `motion-bridge` PyO3 crate (Rust cdylib, imported by klippy as a Python extension). The bridge owns the serial fd to every Klipper-protocol MCU. A new `kalico-host-rt::passthrough_queue` module ports `klippy/chelper/serialqueue.c` to Rust, integrating with the existing 7-C-io reactor. Klippy `mcu.py` is patched to allocate a `MotionMcuProxy` (delegates to bridge) instead of allocating a C `serialqueue` and opening the fd directly. Trapezoidal motion C code (`itersolve`, `stepcompress`, `trapq`, `kin_*.c`) and the just-displaced `serialqueue.*` / `trdispatch.c` are deleted. Motion-related Python (`toolhead.py`, `kinematics/*.py` step generators, `gcode_arcs.py`) is deleted. `motion_toolhead.py` / `motion_mcu.py` / `motion_kinematics.py` skeletons land. CLAUDE.md and dependency-graph.md amendments per spec §1.4 / §9 land in the same batch.

**Tech Stack:**
- **Rust:** PyO3 0.24 (already declared as optional in `kalico-host-rt`), arc-swap, serde, indexmap, log, flate2, serialport. Workspace already exists at `rust/`.
- **Python:** klippy (Python 3 + cffi for the surviving non-motion C bits if any).
- **Build:** klippy uses `make` with `chelper/__init__.py` cffi loading. Phase 1 adds a step that builds the PyO3 crate and drops the resulting `.so` where klippy can `import motion_bridge`.
- **Test:** Rust unit tests + proptest in workspace. Python `pytest` for klippy-side smoke tests. **Renode-based H723 firmware sim** (`tools/sim/run_sim.sh` — already present) for the boot-to-heater-setpoint smoke test. Note: the simulator runs one MCU firmware image at a time. Phase 1 smoke test config uses a single H723 motion MCU; the `[mcu bottom]`, `[beacon]`, `[mcu NIS]` MCUs are stubbed at the bridge boundary in tests (the bridge's `claim_mcu` for those returns a "claimed-but-not-actually-opened" handle that emits canned identify responses and ignores other passthrough commands). Multi-MCU end-to-end testing waits for Phase 4 (homing) when beacon coordination matters.

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
- `tests/motion_bridge/test_klippy_boot.py` — full klippy boot smoke test against Renode H723 sim + bridge-stubbed non-motion MCUs

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
- `klippy/extras/z_tilt.py`, `klippy/extras/z_tilt_ng.py` — these are PATCHED, not hard-disabled (spec §5.2). Phase 1 patch: their config-loaders import cleanly; the runtime `Z_TILT_ADJUST` command raises "homing/probing not yet supported until Phase 4" because the underlying probe path uses `homing.py` (which IS hard-disabled). The `set_trapq()` calls these modules make on Z steppers must be supported by `MCU_stepper.set_trapq()` (spec §5.2 patched API surface) — Phase 1 implementation: bridge tracks "this stepper is/isn't attached to the kinematic transform" as a flag; runtime motion isn't possible yet, so the flag is record-only.
- `klippy/extras/homing.py` — Phase 4 reimplements; Phase 1 raises "homing not yet supported until Phase 4" if the user actually invokes G28; module imports cleanly so klippy can boot.
- `klippy/extras/load_cell/*` (if present) — same pattern.
- `klippy/mcu.py::MCU_trsync` — must boot inertly. `MCU_endstop.__init__` constructs `TriggerDispatch` which constructs `MCU_trsync` *for every endstop pin in the config*, at config-load time, not at G28 time. The constructor calls `mcu.register_config_callback(self._build_config)`, and `_build_config` calls `mcu.lookup_command("trsync_start ..."), lookup_command("trsync_set_timeout ..."), lookup_command("trsync_trigger ..."), lookup_query_command("trsync_trigger ...", "trsync_state ..."), lookup_command("stepper_stop_on_trigger ..."), add_config_cmd("config_trsync ..."), register_response(handler, "trsync_state", oid)`. **Stub semantics for Phase 1:** preserve the constructor surface (`get_oid`, `get_command_queue`, `add_stepper`); `_build_config` runs all the lookup_command/add_config_cmd/register_response calls inertly through the bridge proxy (they succeed without doing anything useful — bridge passthrough to MCU is fine, the command tables exist on the firmware side). Only the `start`/`stop`/`add_stepper` runtime methods raise "homing not yet implemented" — and only when actually called from `home_start()`. `G28` will hit this; config-time construction must not.

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

## Stage B — `passthrough_queue` Rust port (Tasks 11-28)

This is the load-bearing piece of Phase 1: a Rust port of `klippy/chelper/serialqueue.c` (992 LOC) integrated with the existing `kalico-host-rt::host_io` reactor. Per spec §3.5.3, the new module lives at `rust/kalico-host-rt/src/passthrough_queue/`.

**Working principle:** TDD per feature. Each task ports one concept from `serialqueue.c`, with a Rust-native test that pins down behavior. Reference the C source by line range so the engineer can compare semantics. Preserve the externally-observable behavior (klippy's `serialhdl.py` consumers don't notice the swap), not the internal data structure.

### Critical semantic invariants (read before starting Stage B)

These come straight from `serialqueue.c`; getting them wrong propagates through every Stage-B task:

1. **Two queues per `command_queue`:** `upcoming_queue` (sorted by `min_clock` for promotion checks) and `ready_queue` (sorted by `req_clock` for emission priority). See `serialqueue.c:46-100` (struct layout).
2. **Promotion gate (`upcoming → ready`):** `serialqueue.c:544-557`. A message moves from upcoming to ready when `ack_clock >= qm->min_clock`, where `ack_clock = clock_from_time(idle_time + bittime)`. `min_clock` is **the gate, not the priority key**.
3. **Emission ordering (`ready → wire`):** `serialqueue.c:459-474`. Across all command_queues attached to one `serialqueue`, pick the queue whose ready-head has the **lowest `req_clock`**. Pop that one entry, append to outgoing block, repeat until block fills (`MESSAGE_MAX - MESSAGE_TRAILER_SIZE`) or no more ready. `req_clock` is the priority key.
4. **`BACKGROUND_PRIORITY_CLOCK`:** sentinel value on `req_clock` meaning "low priority — only send when bus is idle." Computed at promotion as `clock_from_time(bgtime + bgoffset)` (`serialqueue.c:565-566`). Treat as a special case in the priority comparison; ignored if other ready entries exist with real `req_clock`.
5. **In-flight backpressure:** `serialqueue.c:524-534`. Two checks must both pass before emission: `(send_seq - receive_seq) < MAX_PENDING_BLOCKS`, and `(need_ack_bytes + MESSAGE_MAX [+ last_ack_bytes if last_ack_seq < receive_seq]) <= receive_window`. Per-message bytes alone is **not** sufficient.
6. **Notify-queue handoff:** `serialqueue.c:484-490`. After emission, messages with `notify_id != 0` move to the `sent`/`notify` queue and stay there until acked; only then is the callback fired. Messages without notify_id are freed at emission.

Mismatched ordering invariants in the original draft of this plan caused the Stage B revision; do not re-introduce them.

The task order: types → CommandQueue (correct ordering) → McuState (cross-queue priority pick) → NotifyTable → ReceiveWindow → PassthroughRouter (full surface) → reactor integration → BACKGROUND_PRIORITY / config-stage handling → flush callbacks / stats / extract_old → integration tests.

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

### Task 12: `CommandQueue` — req_clock-ordered ready + min_clock-gated upcoming

**Files:**
- Create: `rust/kalico-host-rt/src/passthrough_queue/command_queue.rs`
- Modify: `rust/kalico-host-rt/src/passthrough_queue/mod.rs`

**Source reference:** `serialqueue.c:455-490` (build_and_send_command emission ordering by req_clock), `:537-588` (check_send_command — promotion gate by min_clock), `:744-805` (alloc + push helpers).

**Invariant (read first):** ready queue is ordered by `req_clock` (priority); upcoming queue holds messages whose `min_clock` hasn't been reached, sorted by `min_clock` for cheap promotion. `pop_ready()` returns the head of ready (lowest req_clock); the cross-queue priority pick happens at `McuState` (Task 13), not here.

- [ ] **Step 1: Write failing tests**

```rust
// rust/kalico-host-rt/src/passthrough_queue/command_queue.rs (test module)
#[cfg(test)]
mod tests {
    use super::*;
    use crate::passthrough_queue::{NotifyId, PassthroughEntry};

    fn entry(bytes: u8, min_clock: u64, req_clock: u64) -> PassthroughEntry {
        PassthroughEntry::new(vec![bytes], min_clock, req_clock, NotifyId::none())
    }

    #[test]
    fn push_routes_by_min_clock_vs_ack_clock() {
        let mut cq = CommandQueue::new();
        // ack_clock = 100. Entry's min_clock 50 < ack_clock → ready.
        cq.push(entry(0xA1, 50, 200), /* ack_clock */ 100);
        assert_eq!(cq.ready_len(), 1);
        assert_eq!(cq.upcoming_len(), 0);
        // Entry's min_clock 500 > ack_clock → upcoming.
        cq.push(entry(0xA2, 500, 600), /* ack_clock */ 100);
        assert_eq!(cq.ready_len(), 1);
        assert_eq!(cq.upcoming_len(), 1);
    }

    #[test]
    fn ready_orders_by_req_clock_not_min_clock() {
        let mut cq = CommandQueue::new();
        // Push out of order; both have min_clock satisfied.
        cq.push(entry(0xB1, 10, 300), 1000);
        cq.push(entry(0xB2, 20, 100), 1000);
        cq.push(entry(0xB3, 30, 200), 1000);
        // Pop order should be 100, 200, 300 (by req_clock).
        assert_eq!(cq.pop_ready().unwrap().bytes(), &[0xB2]);
        assert_eq!(cq.pop_ready().unwrap().bytes(), &[0xB3]);
        assert_eq!(cq.pop_ready().unwrap().bytes(), &[0xB1]);
        assert!(cq.pop_ready().is_none());
    }

    #[test]
    fn promote_moves_when_min_clock_reached() {
        let mut cq = CommandQueue::new();
        cq.push(entry(0xC1, /* min_clock */ 1000, 2000), /* ack_clock */ 500);
        assert_eq!(cq.upcoming_len(), 1);
        // ack_clock still below min_clock — no promotion.
        cq.promote(900);
        assert_eq!(cq.upcoming_len(), 1);
        // ack_clock reaches min_clock — promotion fires.
        cq.promote(1500);
        assert_eq!(cq.upcoming_len(), 0);
        assert_eq!(cq.ready_len(), 1);
    }

    #[test]
    fn promote_preserves_min_clock_order_for_remaining() {
        let mut cq = CommandQueue::new();
        cq.push(entry(0xD1, 1000, 2000), 0);
        cq.push(entry(0xD2, 2000, 1000), 0);
        cq.push(entry(0xD3, 3000, 500), 0);
        assert_eq!(cq.upcoming_len(), 3);
        // Only the first entry's min_clock is reached.
        cq.promote(1500);
        assert_eq!(cq.upcoming_len(), 2);
        assert_eq!(cq.ready_len(), 1);
        assert_eq!(cq.peek_ready_req_clock(), Some(2000)); // 0xD1
    }

    #[test]
    fn peek_ready_req_clock_returns_head_priority() {
        let mut cq = CommandQueue::new();
        assert_eq!(cq.peek_ready_req_clock(), None);
        cq.push(entry(0xE1, 0, 500), 1000);
        cq.push(entry(0xE2, 0, 100), 1000);
        assert_eq!(cq.peek_ready_req_clock(), Some(100));
    }
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cd rust && cargo test -p kalico-host-rt passthrough_queue::command_queue`

Expected: FAIL (`CommandQueue` not defined).

- [ ] **Step 3: Implement `CommandQueue`**

```rust
// rust/kalico-host-rt/src/passthrough_queue/command_queue.rs
//! Per-driver command queue.
//!
//! Two logical lists:
//! - `upcoming`: entries whose min_clock has not been reached.
//!   Sorted ascending by min_clock so promotion checks the head only.
//! - `ready`: entries eligible for emission, sorted ascending by
//!   req_clock (the emission-priority key).
//!
//! Source: klippy/chelper/serialqueue.c:455-490 (emission), :537-588
//! (promotion). The C source keeps `ready_queue` ordered by req_clock
//! (head = lowest = highest priority); `upcoming_queue` holds
//! min_clock-stalled entries waiting for `ack_clock >= min_clock`.

use super::entry::PassthroughEntry;

#[derive(Debug, Default)]
pub struct CommandQueue {
    /// Sorted ascending by min_clock. Head is the next eligible promotion candidate.
    upcoming: Vec<PassthroughEntry>,
    /// Sorted ascending by req_clock. Head is the highest-priority emission candidate.
    ready: Vec<PassthroughEntry>,
}

impl CommandQueue {
    pub fn new() -> Self {
        Self::default()
    }

    /// Insert one entry. Routes to ready if `entry.min_clock() <= ack_clock`,
    /// else to upcoming. `ack_clock` is the projected MCU clock at the time
    /// the next outgoing block would be acked — caller computes it from
    /// host-side wall clock + per-MCU clock-sync state. `0` is the
    /// "no-min_clock-gate" / "send-asap" caller.
    pub fn push(&mut self, entry: PassthroughEntry, ack_clock: u64) {
        if entry.min_clock() == 0 || entry.min_clock() <= ack_clock {
            self.insert_ready(entry);
        } else {
            self.insert_upcoming(entry);
        }
    }

    /// Promote eligible upcoming entries to ready, in min_clock order.
    /// Called from the reactor tick before any pop_ready.
    pub fn promote(&mut self, ack_clock: u64) {
        while let Some(head) = self.upcoming.first() {
            if head.min_clock() <= ack_clock {
                let promoted = self.upcoming.remove(0);
                self.insert_ready(promoted);
            } else {
                break;
            }
        }
    }

    /// Pop the head of the ready queue (lowest req_clock).
    pub fn pop_ready(&mut self) -> Option<PassthroughEntry> {
        if self.ready.is_empty() {
            None
        } else {
            Some(self.ready.remove(0))
        }
    }

    /// Read-only access to head's req_clock without popping.
    /// Used by McuState for the cross-queue priority pick.
    pub fn peek_ready_req_clock(&self) -> Option<u64> {
        self.ready.first().map(|e| e.req_clock())
    }

    pub fn ready_len(&self) -> usize { self.ready.len() }
    pub fn upcoming_len(&self) -> usize { self.upcoming.len() }
    pub fn is_empty(&self) -> bool { self.ready.is_empty() && self.upcoming.is_empty() }

    fn insert_ready(&mut self, entry: PassthroughEntry) {
        let pos = self.ready
            .iter()
            .position(|e| e.req_clock() > entry.req_clock())
            .unwrap_or(self.ready.len());
        self.ready.insert(pos, entry);
    }

    fn insert_upcoming(&mut self, entry: PassthroughEntry) {
        let pos = self.upcoming
            .iter()
            .position(|e| e.min_clock() > entry.min_clock())
            .unwrap_or(self.upcoming.len());
        self.upcoming.insert(pos, entry);
    }
}
```

- [ ] **Step 4: Wire mod.rs**

```rust
// rust/kalico-host-rt/src/passthrough_queue/mod.rs
mod command_queue;
mod entry;

pub use command_queue::CommandQueue;
pub use entry::{NotifyId, PassthroughEntry};
```

- [ ] **Step 5: Run the test to verify it passes**

Run: `cd rust && cargo test -p kalico-host-rt passthrough_queue::command_queue`

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add rust/kalico-host-rt/src/passthrough_queue/
git commit -m "feat(passthrough_queue): CommandQueue with req_clock ready + min_clock upcoming"
```

### Task 13: `McuState` — cross-queue priority emission

**Files:**
- Create: `rust/kalico-host-rt/src/passthrough_queue/mcu_state.rs`
- Modify: `rust/kalico-host-rt/src/passthrough_queue/mod.rs`

**Source reference:** `serialqueue.c:46-100` (struct serialqueue), `:459-474` (cross-queue priority pick by req_clock), `:744-755` (alloc_commandqueue).

**Invariant:** Within one MCU, emission picks the command_queue whose ready-head has the lowest `req_clock`. After popping that one entry, the next iteration re-evaluates — i.e., consecutive emissions can come from different command_queues based on whose head is now the lowest.

- [ ] **Step 1: Write failing tests**

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use crate::passthrough_queue::{NotifyId, PassthroughEntry};

    fn entry(bytes: u8, req_clock: u64) -> PassthroughEntry {
        PassthroughEntry::new(vec![bytes], 0, req_clock, NotifyId::none())
    }

    #[test]
    fn allocates_distinct_command_queue_ids() {
        let mut state = McuState::new();
        let q1 = state.alloc_command_queue();
        let q2 = state.alloc_command_queue();
        assert_ne!(q1, q2);
    }

    #[test]
    fn pop_picks_lowest_req_clock_across_queues() {
        let mut state = McuState::new();
        let q_a = state.alloc_command_queue();
        let q_b = state.alloc_command_queue();
        // q_a head: req_clock=300; q_b head: req_clock=100. Expect q_b first.
        state.push(q_a, entry(0xA1, 300), /* ack_clock */ 1000).unwrap();
        state.push(q_b, entry(0xB1, 100), 1000).unwrap();
        state.push(q_a, entry(0xA2, 200), 1000).unwrap();
        // Order: 100 (q_b), 200 (q_a), 300 (q_a).
        assert_eq!(state.pop_next().unwrap().bytes(), &[0xB1]);
        assert_eq!(state.pop_next().unwrap().bytes(), &[0xA2]);
        assert_eq!(state.pop_next().unwrap().bytes(), &[0xA1]);
        assert!(state.pop_next().is_none());
    }

    #[test]
    fn promote_runs_across_all_queues() {
        let mut state = McuState::new();
        let q_a = state.alloc_command_queue();
        let q_b = state.alloc_command_queue();
        state.push(q_a, PassthroughEntry::new(vec![0xA1], 1000, 100, NotifyId::none()), 0).unwrap();
        state.push(q_b, PassthroughEntry::new(vec![0xB1], 2000, 200, NotifyId::none()), 0).unwrap();
        // Below both min_clocks — neither promotes.
        state.promote_all(500);
        assert!(state.pop_next().is_none());
        // ack_clock reaches q_a's min_clock — only q_a promotes.
        state.promote_all(1500);
        assert_eq!(state.pop_next().unwrap().bytes(), &[0xA1]);
        assert!(state.pop_next().is_none());
        // Now q_b's min_clock is reached.
        state.promote_all(2500);
        assert_eq!(state.pop_next().unwrap().bytes(), &[0xB1]);
    }
}
```

- [ ] **Step 2: Implement `McuState`**

```rust
// rust/kalico-host-rt/src/passthrough_queue/mcu_state.rs

use super::{CommandQueue, PassthroughEntry};
use indexmap::IndexMap;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct CommandQueueId(u32);

#[derive(Debug)]
pub enum PushError {
    UnknownQueue(CommandQueueId),
}

#[derive(Debug, Default)]
pub struct McuState {
    queues: IndexMap<CommandQueueId, CommandQueue>,
    next_id: u32,
}

impl McuState {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn alloc_command_queue(&mut self) -> CommandQueueId {
        let id = CommandQueueId(self.next_id);
        self.next_id = self.next_id.checked_add(1).expect("CommandQueueId exhausted");
        self.queues.insert(id, CommandQueue::new());
        id
    }

    pub fn push(
        &mut self,
        queue: CommandQueueId,
        entry: PassthroughEntry,
        ack_clock: u64,
    ) -> Result<(), PushError> {
        let q = self.queues.get_mut(&queue).ok_or(PushError::UnknownQueue(queue))?;
        q.push(entry, ack_clock);
        Ok(())
    }

    /// Promote upcoming → ready across all queues.
    pub fn promote_all(&mut self, ack_clock: u64) {
        for q in self.queues.values_mut() {
            q.promote(ack_clock);
        }
    }

    /// Pop the next entry across all queues, picking the lowest req_clock head.
    /// Returns `None` if no queue has any ready entry.
    pub fn pop_next(&mut self) -> Option<PassthroughEntry> {
        // Find the queue whose ready-head has the smallest req_clock.
        let chosen_id = self
            .queues
            .iter()
            .filter_map(|(id, q)| q.peek_ready_req_clock().map(|c| (c, *id)))
            .min_by_key(|&(c, _)| c)
            .map(|(_, id)| id)?;
        self.queues.get_mut(&chosen_id)?.pop_ready()
    }

    pub fn command_queue(&self, queue: CommandQueueId) -> Option<&CommandQueue> {
        self.queues.get(&queue)
    }

    pub fn queue_count(&self) -> usize {
        self.queues.len()
    }
}
```

- [ ] **Step 3: Wire mod.rs**

```rust
mod mcu_state;
pub use mcu_state::{CommandQueueId, McuState, PushError};
```

- [ ] **Step 4: Run tests**

Run: `cd rust && cargo test -p kalico-host-rt passthrough_queue::mcu_state`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/kalico-host-rt/src/passthrough_queue/
git commit -m "feat(passthrough_queue): McuState — cross-queue priority emission by req_clock"
```

### Task 14: `NotifyTable` — query/response correlation

**Files:**
- Create: `rust/kalico-host-rt/src/passthrough_queue/notify.rs`
- Modify: `rust/kalico-host-rt/src/passthrough_queue/mod.rs`

**Source reference:** `serialqueue.c:222-300` (handle_message notify dispatch), `:484-490` (notify-queue handoff after emission), `:838-852` (serialqueue_send notify allocation).

- [ ] **Step 1: Write failing tests**

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicU32, Ordering};
    use std::sync::Arc;

    #[test]
    fn dispatch_fires_callback_once() {
        let counter = Arc::new(AtomicU32::new(0));
        let counter_cb = counter.clone();
        let mut table = NotifyTable::new();
        let id = table.register(Box::new(move |_| {
            counter_cb.fetch_add(1, Ordering::SeqCst);
        }));
        table.dispatch(id, NotifyResponse::default());
        assert_eq!(counter.load(Ordering::SeqCst), 1);
        // Second dispatch is a no-op.
        table.dispatch(id, NotifyResponse::default());
        assert_eq!(counter.load(Ordering::SeqCst), 1);
    }

    #[test]
    fn unique_ids() {
        let mut table = NotifyTable::new();
        let a = table.register(Box::new(|_| {}));
        let b = table.register(Box::new(|_| {}));
        assert_ne!(a, b);
    }

    #[test]
    fn dispatch_propagates_response_payload() {
        let received: Arc<std::sync::Mutex<Option<NotifyResponse>>> =
            Arc::new(std::sync::Mutex::new(None));
        let received_cb = received.clone();
        let mut table = NotifyTable::new();
        let id = table.register(Box::new(move |resp| {
            *received_cb.lock().unwrap() = Some(resp);
        }));
        let payload = NotifyResponse {
            bytes: vec![0xAA, 0xBB],
            sent_time: 1.0,
            receive_time: 1.5,
        };
        table.dispatch(id, payload);
        let got = received.lock().unwrap().take().expect("dispatched");
        assert_eq!(got.bytes, vec![0xAA, 0xBB]);
        assert!((got.sent_time - 1.0).abs() < 1e-9);
        assert!((got.receive_time - 1.5).abs() < 1e-9);
    }
}
```

- [ ] **Step 2: Implement `NotifyTable`**

```rust
// rust/kalico-host-rt/src/passthrough_queue/notify.rs
use super::entry::NotifyId;
use std::collections::HashMap;

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
    pub fn new() -> Self { Self::default() }

    pub fn register(&mut self, cb: NotifyCallback) -> NotifyId {
        self.next_id = self.next_id.checked_add(1).expect("NotifyId exhausted");
        let id = NotifyId::new(self.next_id);
        self.pending.insert(id, cb);
        id
    }

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

Wire mod.rs: `mod notify; pub use notify::{NotifyCallback, NotifyResponse, NotifyTable};`

- [ ] **Step 3: Run tests**

Run: `cd rust && cargo test -p kalico-host-rt passthrough_queue::notify`

Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add rust/kalico-host-rt/src/passthrough_queue/
git commit -m "feat(passthrough_queue): NotifyTable for query/response correlation"
```

### Task 15: `ReceiveWindow` with full backpressure semantics

**Files:**
- Create: `rust/kalico-host-rt/src/passthrough_queue/receive_window.rs`
- Modify: `rust/kalico-host-rt/src/passthrough_queue/mod.rs`

**Source reference:** `serialqueue.c:524-534` (receive-window check), `:903-913` (set_receive_window).

The check that gates emission is **not** "do these bytes fit": it's

```
need_ack_bytes_total = sq->need_ack_bytes + MESSAGE_MAX
if (sq->last_ack_seq < sq->receive_seq):
    need_ack_bytes_total += sq->last_ack_bytes
if need_ack_bytes_total > sq->receive_window:
    // block emission
```

Plus a separate gate: `(send_seq - receive_seq) < MAX_PENDING_BLOCKS`. Both must pass.

- [ ] **Step 1: Write failing tests**

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn window_default_starts_empty() {
        let w = ReceiveWindow::new(/* limit */ 192, /* message_max */ 64);
        assert_eq!(w.in_flight_bytes(), 0);
        assert!(w.can_emit());
    }

    #[test]
    fn emit_check_includes_message_max_overhead() {
        // With limit=100 and MESSAGE_MAX=64, only one in-flight block fits
        // (need_ack 64 + MESSAGE_MAX 64 = 128 > 100 — blocked after first).
        let mut w = ReceiveWindow::new(100, 64);
        assert!(w.can_emit());
        w.record_emit(64);
        assert!(!w.can_emit()); // need_ack 64 + MESSAGE_MAX 64 = 128 > 100
        w.record_ack(64, /* last_ack_bytes_carry */ 0);
        assert!(w.can_emit());
    }

    #[test]
    fn pending_blocks_gate() {
        let mut w = ReceiveWindow::new(/* big */ 1_000_000, 64);
        // Configurable MAX_PENDING_BLOCKS for this test.
        w.set_max_pending_blocks(2);
        assert!(w.can_emit());
        w.record_emit(10); // pending=1
        assert!(w.can_emit());
        w.record_emit(10); // pending=2 — at limit
        assert!(!w.can_emit());
        w.record_ack(10, 0); // pending=1
        assert!(w.can_emit());
    }

    #[test]
    fn last_ack_bytes_carry_when_acks_lag() {
        // Models the C condition `last_ack_seq < receive_seq`: extra bytes
        // counted toward the in-flight budget.
        let mut w = ReceiveWindow::new(100, 64);
        w.record_emit(20);
        // ack-with-carry: 0 bytes acked, but 16 bytes still attributed to
        // last_ack overhead.
        w.set_last_ack_carry(16);
        // Required: need_ack(20) + MESSAGE_MAX(64) + last_ack_carry(16) = 100 — exactly at limit.
        assert!(!w.can_emit_strict()); // strict-> would-exceed
        // Drop carry → can emit.
        w.set_last_ack_carry(0);
        assert!(w.can_emit_strict() == false); // 20+64=84 < 100; can_emit_strict means "doesn't exceed", so this should be true. Adjust test.
    }
}
```

(The last test is intentionally ugly — semantics around `last_ack_bytes` carry are a known wart in `serialqueue.c`. Engineer should consult the C source while writing this test, then translate the predicate exactly.)

- [ ] **Step 2: Implement `ReceiveWindow`**

Translate the C predicate from `serialqueue.c:524-534` directly:

```rust
// rust/kalico-host-rt/src/passthrough_queue/receive_window.rs
//! Receive-window backpressure.
//!
//! Source: klippy/chelper/serialqueue.c:524-534 (the gate),
//! :903-913 (window setter).
//!
//! Two predicates must both be true to emit:
//!   1. (send_seq - receive_seq) < MAX_PENDING_BLOCKS
//!   2. need_ack_bytes + MESSAGE_MAX [+ last_ack_bytes_carry] <= receive_window

#[derive(Debug)]
pub struct ReceiveWindow {
    receive_window: usize,
    message_max: usize,
    /// Bytes of in-flight (sent-but-unacked) message data.
    need_ack_bytes: usize,
    /// Pending block count: send_seq - receive_seq.
    pending_blocks: u64,
    /// Carry term active when last_ack_seq < receive_seq (per C source).
    last_ack_bytes_carry: usize,
    /// Per-MCU tunable, default MAX_PENDING_BLOCKS=12 (klippy default).
    max_pending_blocks: u64,
}

impl ReceiveWindow {
    pub fn new(receive_window: usize, message_max: usize) -> Self {
        Self {
            receive_window,
            message_max,
            need_ack_bytes: 0,
            pending_blocks: 0,
            last_ack_bytes_carry: 0,
            max_pending_blocks: 12,
        }
    }

    pub fn set_max_pending_blocks(&mut self, n: u64) { self.max_pending_blocks = n; }
    pub fn set_last_ack_carry(&mut self, n: usize)    { self.last_ack_bytes_carry = n; }

    /// True iff a new outgoing block of up to `message_max` bytes can be sent
    /// without violating either gate.
    pub fn can_emit(&self) -> bool {
        if self.pending_blocks >= self.max_pending_blocks {
            return false;
        }
        let need = self.need_ack_bytes + self.message_max + self.last_ack_bytes_carry;
        need <= self.receive_window
    }

    pub fn record_emit(&mut self, bytes: usize) {
        self.need_ack_bytes += bytes;
        self.pending_blocks += 1;
    }

    pub fn record_ack(&mut self, bytes: usize, last_ack_bytes_carry: usize) {
        self.need_ack_bytes = self.need_ack_bytes.saturating_sub(bytes);
        self.pending_blocks = self.pending_blocks.saturating_sub(1);
        self.last_ack_bytes_carry = last_ack_bytes_carry;
    }

    pub fn in_flight_bytes(&self) -> usize { self.need_ack_bytes }
    pub fn pending_blocks(&self) -> u64 { self.pending_blocks }
    pub fn limit(&self) -> usize { self.receive_window }
    pub fn set_limit(&mut self, n: usize) { self.receive_window = n; }
}
```

- [ ] **Step 3: Run tests + iterate against C source until predicate matches**

Run: `cd rust && cargo test -p kalico-host-rt passthrough_queue::receive_window`

Adjust test predicates against the C source until both match. The test names above are correct; the assertions may need fine-tuning during execution.

- [ ] **Step 4: Commit**

```bash
git add rust/kalico-host-rt/src/passthrough_queue/
git commit -m "feat(passthrough_queue): ReceiveWindow — full need_ack_bytes + pending_blocks gating"
```

### Task 16: `PassthroughRouter` — full surface

**Files:**
- Create: `rust/kalico-host-rt/src/passthrough_queue/router.rs`
- Modify: `rust/kalico-host-rt/src/passthrough_queue/mod.rs`

This is the boundary the bridge calls. Owns one `McuState` + one `NotifyTable` + one `ReceiveWindow` per claimed MCU. **Defines all methods Tasks 17, 19, 20 will call** — this is the "no forward references" task.

- [ ] **Step 1: Sketch the surface**

The router exposes:

```rust
pub struct PassthroughRouter {
    mcus: IndexMap<McuHandle, McuRecord>,
    next_handle: u32,
    clock: Arc<dyn Clock + Send + Sync>,  // 7-C-io tail Clock seam
}

struct McuRecord {
    label: String,
    state: McuState,
    notify_table: NotifyTable,
    window: ReceiveWindow,
    /// Maps notify_id → emit-time wall clock (for #sent_time annotation).
    sent_times: HashMap<NotifyId, f64>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct McuHandle(u32);
```

Methods:

| Method | Signature | Used by |
|---|---|---|
| `with_clock` | `fn(Arc<dyn Clock>) -> Self` | constructor |
| `claim_mcu` | `fn(&mut self, label) -> McuHandle` | bridge.claim_mcu |
| `release_mcu` | `fn(&mut self, McuHandle) -> bool` | bridge.shutdown |
| `alloc_command_queue` | `fn(&mut self, McuHandle) -> Result<CommandQueueId, RouterError>` | bridge.alloc_command_queue |
| `register_notify` | `fn(&mut self, McuHandle, NotifyCallback) -> Result<NotifyId, RouterError>` | bridge.passthrough_query / send_wait_ack |
| `push` | `fn(&mut self, McuHandle, CommandQueueId, PassthroughEntry, ack_clock) -> Result<(), RouterError>` | bridge.passthrough_send |
| `promote_all` | `fn(&mut self, McuHandle, ack_clock) -> Result<(), RouterError>` | reactor tick |
| `pop_next_for_emission` | `fn(&mut self, McuHandle) -> Option<PassthroughEntry>` | reactor — only returns if window allows; records emit time + emit bytes against window. |
| `dispatch_response` | `fn(&mut self, McuHandle, NotifyId, Vec<u8>)` | reactor on incoming response with notify_id |
| `peek_sent_time` | `fn(&self, McuHandle, NotifyId) -> Option<f64>` | tests; also used internally for #sent_time annotation |
| `record_ack` | `fn(&mut self, McuHandle, bytes, last_ack_carry)` | reactor on ACK |
| `mcu_state` (test-only) | `fn(&mut self, McuHandle) -> Option<&mut McuState>` | tests |

- [ ] **Step 2: Write tests covering each method**

Tests come from the existing pattern in Tasks 12-15. Cover:
- Two MCUs claim/release independently
- alloc_command_queue per MCU
- push routes correctly through McuState
- register_notify + dispatch_response round-trip with sent_time/receive_time correctly populated
- pop_next_for_emission respects the window gate (returns None when blocked)
- record_ack frees window capacity

- [ ] **Step 3: Implement the router**

Bring in the existing `kalico-host-rt::clock::Clock` trait + `RealClock` / `MockClock` (already exists from 7-C-io tail). When recording emit time:

```rust
fn pop_next_for_emission(&mut self, mcu: McuHandle) -> Option<PassthroughEntry> {
    let rec = self.mcus.get_mut(&mcu)?;
    if !rec.window.can_emit() { return None; }
    let entry = rec.state.pop_next()?;
    rec.window.record_emit(entry.bytes().len());
    if !entry.notify_id().is_none() {
        rec.sent_times.insert(entry.notify_id(), self.clock.now_secs());
    }
    Some(entry)
}

fn dispatch_response(&mut self, mcu: McuHandle, id: NotifyId, bytes: Vec<u8>) {
    let Some(rec) = self.mcus.get_mut(&mcu) else { return };
    let sent_time = rec.sent_times.remove(&id).unwrap_or(0.0);
    let receive_time = self.clock.now_secs();
    let resp = NotifyResponse { bytes, sent_time, receive_time };
    rec.notify_table.dispatch(id, resp);
}
```

- [ ] **Step 4: Run tests**

Run: `cd rust && cargo test -p kalico-host-rt passthrough_queue::router`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/kalico-host-rt/src/passthrough_queue/
git commit -m "feat(passthrough_queue): PassthroughRouter — full method surface (claim/push/promote/pop/notify/ack)"
```

### Task 17: Reactor integration

**Files:**
- Modify: `rust/kalico-host-rt/src/host_io/reactor.rs`
- Modify: `rust/kalico-host-rt/src/lib.rs` (re-exports)

**Source reference:** `serialqueue.c:520-636` (check_send_command + command_event + background_thread). The 7-C-io reactor already owns the wire framing/seq/retransmit; this task wires `PassthroughRouter` into the reactor's `tick_once`.

**Per-tick flow:**

1. Compute current `ack_clock` from `Clock::now_secs()` + `clock_sync` state for each MCU.
2. `router.promote_all(mcu, ack_clock)` for each MCU.
3. Pack outgoing block from `router.pop_next_for_emission(mcu)` calls until block fills (per `serialqueue.c:475-491`).
4. Hand block to existing wire framer (sequence number, CRC, sync byte) — already done by 7-C-io.
5. On incoming responses: parse via existing parser; if response carries a recognized `notify_id`, call `router.dispatch_response(mcu, notify_id, payload)`. Otherwise route to the "unsolicited response" path (Task 19's flush-callback / type-keyed handler chain) and to runtime_events as today.
6. On ACK: call `router.record_ack(mcu, bytes, last_ack_carry)`.

- [ ] **Step 1: Add hook points in `reactor.rs::tick_once`**

Identify the current tick body and add:
- `pre_emit` slot: promote + window-gated pack-and-send.
- `post_response` slot: dispatch_response on notify match.
- `post_ack` slot: record_ack.

Each slot is a method on a new `PassthroughHook` trait that the reactor calls; or, simpler, the router becomes a member of the reactor.

- [ ] **Step 2: Add an integration test using `MockTransport`**

Build on `tests/mock_transport.rs` — push entries into the router, drive `tick_once`, observe emitted bytes on the mock wire, simulate ACK and response from the mock side, verify notify dispatch + window accounting.

- [ ] **Step 3: Run tests**

Run: `cd rust && cargo test -p kalico-host-rt --tests`

Expected: existing tests still pass; new integration test passes.

- [ ] **Step 4: Commit**

```bash
git add rust/kalico-host-rt/
git commit -m "feat(passthrough_queue): wire router into host_io reactor tick_once"
```

### Tasks 18-23: Remaining serialqueue.c features (coarser granularity)

For each: TDD pattern as Tasks 12-17 (test first, port from C, verify, commit). Reference C source by line range.

- [ ] **Task 18: `BACKGROUND_PRIORITY_CLOCK`** — special sentinel on `req_clock`. `serialqueue.c:564-566`. When seen, treat as `clock_from_time(bgtime + bgoffset)` in priority comparison; effectively "send only when bus is idle." Add to `PassthroughEntry` builder helpers + `CommandQueue::peek_ready_req_clock` substitution.
- [ ] **Task 19: `add_config_cmd` — config_cmds / restart_cmds / init_cmds distinction.** Klippy `mcu.add_config_cmd(cmd, is_init=False)` distinguishes init-stage commands (sent once after MCU restart, before runtime traffic) from runtime config commands. See `klippy/mcu.py:1002-1048` for `_send_config()`. Bridge needs three named queues per MCU for these phases, drained in order at MCU startup, then runtime commands flow normally.
- [ ] **Task 20: `serialqueue_set_clock_est` / `set_wire_frequency`** (`serialqueue.c:890-927`). Wires per-MCU clock-sync state into the router so `ack_clock` projection (Task 17) is accurate. Reuse existing `kalico-host-rt::clock_sync::ClockSyncEstimator`.
- [ ] **Task 21: Flush callbacks** — fire when an MCU's queues all reach empty. Klippy `mcu.register_flush_callback()` consumers (output_pin GPIO coalescing, fan PWM batching). Triggered from the reactor when `router.is_drained(mcu)` transitions false→true.
- [ ] **Task 22: Stats / `serialqueue_get_stats` parity** (`serialqueue.c:936-958`). Per-MCU counters: bytes sent/received, ACK count, retransmits, NAKs, queue high-water marks. Exposed as a struct readable by the bridge for klippy's periodic stats string.
- [ ] **Task 23: `serialqueue_extract_old`** (`:958-992`). Used by `klippy/serialhdl.py:dump_debug` for crash diagnostics — reads out the in-flight `sent_queue` and the most recent `receive_queue`. Implement minimally; what's needed is the data shape `pull_queue_message` consumers expect (see `serialhdl.py:414-444`).

### Tasks 24-28: Stage-B integration tests

Run end-to-end against `MockTransport`:

- [ ] **Task 24:** Single-MCU emission ordering — push three entries with different req_clocks, verify wire-side bytes appear in req_clock order.
- [ ] **Task 25:** Multi-MCU isolation — claim two MCUs, push to each, verify no cross-talk.
- [ ] **Task 26:** Notify round-trip — push query with notify_id, simulate response with matching `notify_id`, verify callback fires with sent_time/receive_time set; verify `#oid` annotation if oid attached.
- [ ] **Task 27:** Window backpressure — fill the receive window, verify emission stops; ack the in-flight bytes; verify emission resumes.
- [ ] **Task 28:** Config-stage emission ordering — register config_cmds + init_cmds, drive identify, verify config_cmds emit before init_cmds, both before runtime traffic.

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

This stage gets klippy actually using the bridge. Order matters: the most foundational patches first (printer.py instantiates bridge → mcu.py allocates proxy → stepper.py + heaters reach setpoint).

**Important:** klippy can NOT boot at any intermediate point during Stage D. Tasks 41-55 are co-dependent — `mcu.py` (Task 44) calls `MotionMcuProxy` (Task 43), which depends on bridge construction (Tasks 41-42), which depends on the gutted `stepper.py` (Task 46) constructing without `stepcompress`, which depends on `motion_toolhead` skeleton (Task 51) so `klippy/printer.py` doesn't try to import `toolhead.py` (deleted in Stage E). The intermediate state is not buildable; **boot smoke verification only happens at the end of Stage D**, then Stage E deletes the orphaned C/Python files (which by Stage E's start are already not in any import path).

Each task ends with a `python3 -c "from klippy import <module>"` import smoke check (no runtime exercise) before commit. Full klippy boot only at end of stage.

- [ ] **Task 41:** `klippy/motion_bridge.py` Python wrapper — opens the event-fd pipe, instantiates the PyO3 `MotionBridge`, registers the read end with `reactor.register_fd`.
- [ ] **Task 42:** `klippy/printer.py` — instantiate the bridge during connection setup, before MCU objects.
- [ ] **Task 43:** `klippy/motion_mcu.py` — `MotionMcuProxy` class implementing the public surface listed in spec §3.5.1 + §3.6 (`lookup_command`, `lookup_query_command`, `add_config_cmd`, `register_response`, `register_flush_callback`, `alloc_command_queue`, `estimated_print_time`, `print_time_to_clock`, `clock_to_print_time`, `seconds_to_clock`, `clock_to_seconds`, `is_fileoutput`, `is_shutdown`, `get_constants`, `create_oid`, `get_status`, etc.). Each method delegates to the bridge.
- [ ] **Task 44:** Patch `klippy/mcu.py` — constructor branches: instead of `serialqueue_alloc` + opening fd, allocates a `MotionMcuProxy` for any `[mcu*]` config. Make this gating explicit (e.g., `if printer.lookup_object('motion_bridge', None)`). Per Stage-D opening note, full boot test deferred to end of stage; for this task just verify the file imports and `MCU` constructs without raising.
- [ ] **Task 45:** Patch `klippy/serialhdl.py` — gut the C-side serialqueue allocation. Decision: keep the file as a thin wrapper over `motion_mcu.py` that preserves the `SerialReader` API surface for any existing direct consumer, OR delete `serialhdl.py` outright and migrate any direct consumers to `motion_mcu.py`. Pick the smaller-diff option.
- [ ] **Task 46:** Patch `klippy/stepper.py` — preserve `PrinterStepper` / `MCU_stepper` / `PrinterRail` config-object surface per §5.2; gut motion internals; route `set_trapq` / `setup_itersolve` / `set_stepper_kinematics` to bridge stub methods (Phase 1: record-only, no runtime motion).
- [ ] **Task 47:** Patch `klippy/kinematics/extruder.py` — keep `PrinterExtruder` / `ExtruderStepper` / `cmd_SET_PRESSURE_ADVANCE` / `cmd_SYNC_EXTRUDER_MOTION` per §5.2; PA params no-op; route to bridge stubs.
- [ ] **Task 48:** Patch `klippy/kinematics/idex_modes.py` — refuse runtime mode switches with "not yet supported" error.
- [ ] **Task 49:** Patch `klippy/extras/motion_report.py` — drop trapq dump endpoint; preserve `trapqs` dict shape backed by bridge state queries (Phase 1: empty stub returning current bridge state).
- [ ] **Task 50:** Patch `klippy/extras/input_shaper.py` — drop trapezoidal IS C path; convert to ShaperSpec config-parser; `SET_INPUT_SHAPER` raises "not yet supported until Phase 3".
- [ ] **Task 51:** Stub `klippy/motion_toolhead.py` — implement the §3.6.2 compatibility matrix at scaffold level. Methods that don't yet have a real bridge backing (Phase 1) raise `NotImplementedError("not yet supported until Phase 2")` for any move-issuing call. Methods that work (`get_kinematics`, `get_status`, `get_extruder`, `get_last_move_time` returning a sensible default until motion lands) work now.
- [ ] **Task 52:** Stub `klippy/motion_kinematics.py` — Cartesian + CoreXY config parsers; emit `KinematicsSpec` to bridge. No runtime motion logic.
- [ ] **Task 53:** Stub `mcu.MCU_trsync` — preserve config-time constructor + `_build_config` callback (which issues `mcu.lookup_command("trsync_start..."), lookup_query_command, add_config_cmd("config_trsync..."), register_response(handler, "trsync_state", oid)` — all flow through bridge passthrough as today; firmware command table exists). Only the runtime methods (`start`, `stop`, `add_stepper` from homing.py) raise "homing not yet implemented". This is what lets klippy boot with the user's config (every endstop config section constructs a `TriggerDispatch` → `MCU_trsync` at startup). See file-structure note above.
- [ ] **Task 54:** Hard-disable list patches per spec §5.3. Two categories:
   - **Permanent hard-disable (post-MVP not in this build):** `mixing_extruder`, `trad_rack`, `pwm_tool` — config-loader raises "not supported under the new motion path."
   - **Phase-deferred hard-disable (will land in later phase):** `manual_stepper` (Phase 5), `force_move` (Phase 5), `homing.py` runtime path (Phase 4 — the import has to succeed; only `home_start` etc. raise). The user's config has `[motors_sync]` and `[z_tilt_ng]`; both are *not* hard-disabled — `motors_sync` runs against `force_move` so it'll fail at runtime if invoked (acceptable for Phase 1 — no one's going to invoke `SYNC_MOTORS` while heaters warm up); `z_tilt_ng` is patched per Task 49-equivalent.
   - **Note on z_tilt / z_tilt_ng:** these are PATCHED, not hard-disabled (spec §5.2). Phase 1 patch lets the config import cleanly and `set_trapq()` calls succeed inertly. Runtime `Z_TILT_ADJUST` raises "probing/homing not yet supported until Phase 4."
- [ ] **Task 55:** Preserve the `gcode_arcs` configuration error per spec §4.3 — config-loader raises "remove `[gcode_arcs]` from your config" error.

---

## Stage E — Deletion sweep (Tasks 56-60)

Done **after** Stage D so klippy already imports cleanly with the new code path. Each deletion is a separate commit.

- [ ] **Task 56:** `git rm klippy/toolhead.py` — verify no remaining imports.
- [ ] **Task 57:** `git rm klippy/kinematics/cartesian.py corexy.py corexz.py cartesian_abc.py delta.py deltesian.py polar.py rotary_delta.py winch.py hybrid_corexy.py hybrid_corexz.py limited_cartesian.py limited_corexy.py limited_corexz.py none.py` — verify no remaining imports.
- [ ] **Task 58:** `git rm klippy/extras/gcode_arcs.py`.
- [ ] **Task 59:** `git rm klippy/chelper/itersolve.* stepcompress.* serialqueue.* trapq.c trapq.h trdispatch.c kin_*.c`. Then patch `klippy/chelper/__init__.py` to remove **all** related artifacts:
   - From `SOURCE_FILES` (line ~22-42): drop `serialqueue.c`, `stepcompress.c`, `itersolve.c`, `trapq.c`, `trdispatch.c`, `kin_cartesian.c`, `kin_corexy.c`, `kin_delta.c`, `kin_extruder.c`, `kin_polar.c`, `kin_rotary_delta.c`, `kin_winch.c`, `kin_shaper.c`. (`pyhelper.c` stays — used for CRC + msgblock util that may still be referenced; revisit during execution.)
   - From `OTHER_FILES` (line ~44-53): drop the matching `.h` files.
   - Remove `defs_serialqueue`, `defs_stepcompress`, `defs_itersolve`, `defs_trapq`, `defs_trdispatch`, `defs_kin_*` blocks (around line ~190-262) and the entries referencing them in `defs_all` (line ~243-262).
   - Verify any remaining `chelper.get_ffi()` callers don't reference removed cdefs. Grep: `grep -rn "ffi_lib\.\(serialqueue\|stepcompress\|itersolve\|trapq\|trdispatch\|cartesian_stepper_alloc\|corexy_stepper_alloc\|extruder_stepper_alloc\|delta_stepper_alloc\|polar_stepper_alloc\|rotary_delta_stepper_alloc\|winch_stepper_alloc\|input_shaper\)" klippy/` — every result is either an already-patched-or-deleted-file site, or a missed audit target.
- [ ] **Task 60:** Re-run klippy boot smoke test — verify nothing broke.

---

## Stage F — Smoke test under Renode H723 (Tasks 61-65)

End-to-end Phase 1 verification: klippy boots against the user's Trident config (or a sanitized version), heaters reach setpoint, no motion attempted.

- [ ] **Task 61:** Use `tools/sim/run_sim.sh` (the existing Renode H723 firmware sim) for a single motion MCU. Add a Phase-1 test fixture that, in the Python smoke test harness, intercepts bridge `claim_mcu` for the `[mcu bottom]`, `[beacon]`, `[mcu NIS]` config sections and returns canned identify-response handles that emit a minimal data-dictionary on identify and ack any subsequent passthrough commands without acting on them. (Real beacon / NIS / bottom-MCU integration testing is out of Phase 1 scope; lands in Phase 4 when probing/homing matters.)
- [ ] **Task 62:** Build a minimal `printer.cfg` derived from `~/printer_data/config/printer.cfg` (sanitized) that exercises: `[mcu]`, `[mcu bottom]`, `[beacon]`, `[stepper_x/y/z]`, `[extruder]`, `[heater_bed]`, `[input_shaper]`, `[tmc5160 stepper_x]`, `[fan]`, etc.
- [ ] **Task 63:** Write `tests/motion_bridge/test_klippy_boot.py` — pytest that spawns the Renode H723 sim (single motion MCU image), spawns klippy with the smoke config + bridge-stub fixture for non-motion MCUs, waits for "ready" on the klippy API, issues `M105` / `M104 S60`, verifies the *extruder* heater PID drives temp toward setpoint. (Bed heater is on the bottom MCU which is stubbed in Phase 1; verify on the extruder heater which is on the H723.)
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
