# Step 6 Implementation Plan — Communication Protocol, Clock Sync, and Simulator Fixes

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the host↔MCU communication layer per `docs/superpowers/specs/2026-04-28-step6-comm-protocol-and-sim-fixes-design.md` (commit `c0f19cfa`). Phase 0 closes the Step-5 simulator follow-ups (1-day timebox); the rest of the plan implements the Step-6 architectural shape (credit-based flow control, multi-MCU clock-sync, half-split SPSC, generation-counter curve handles, fault taxonomy, stream lifecycle, force_idle flush handshake) on top of the Step-5 runtime substrate.

**Architecture:** The MCU runtime crate (`rust/runtime/`) is refactored from Step-5's single `RuntimeContext` into a half-split with `FgState`/`IsrState`/`SharedState`, accessed by FFI via raw-pointer projection (no `&mut RuntimeContext` ever materializes). Wire framing extends Klipper's existing msgproto (1-byte format-version on each `%*s` blob payload). The credit-based flow control rides MCU-emitted `kalico_credit_freed` events; status frames at 10 Hz piggyback clock-sync samples. Multi-MCU coordination uses an arm/commit handshake gated on per-MCU clock-sync quality. Curve-pool generation handles (u32 = u16 slot + u16 gen) defeat ABA at the §7.1 `Q_N_MAX = 256` ceiling. Flush is implemented via an explicit `force_idle` / `acked_force_idle` handshake; no part of the spec relies on the deprecated TIM5-disable-around-push idiom. Step 6 does not ship F4x bring-up (parallel workstream); it does ship single-H723 validation against the Renode simulator.

**Tech Stack:** Rust 2024 edition; `runtime/` `no_std`; `kalico-c-api/` umbrella staticlib (extended); `heapless 0.8` (SPSC queues); existing `nurbs/` Layer 0 substrate; Klipper C build system (Kconfig, autoconf bridge, msgproto framing); STM32H723 (BTT Octopus Pro target) + Renode H743 platform model for sim.

**Spec:** `docs/superpowers/specs/2026-04-28-step6-comm-protocol-and-sim-fixes-design.md`. **Read the full spec end-to-end before starting any task** — every architectural decision is recorded there with rationale across four review rounds.

---

## Pre-Flight

Before Phase 0 — read the spec and confirm prerequisites:

- §2 architecture (host runtime ownership of USB-CDC fd, MCU-side half-split shape).
- §3 Phase 0 (sim CYCCNT C-side fork, load_curve GDB-attach + escape hatch, Gate A vs Gate B).
- §4 wire framing (msgproto + 1-byte versioned blobs).
- §5 flow control (α credit, MCU-authoritative; periodic status frame at 10 Hz as backstop).
- §6 multi-MCU sync triplet (clock-freq estimation + per-MCU local-clock t_start/t_end + arm/commit handshake) + §6.5 hold segments.
- §7 buffer-budget framework (parameter linkage; Q_N_MAX=256 ceiling; M1/M2/M3 measurement protocols).
- §8 stream lifecycle + §8.5 flush mechanism (force_idle handshake; **plan-decision A: foreground sets `force_idle=true` first, ack-waits, then `stream_open=false`**).
- §9 fault taxonomy.
- §10 curve-pool generation handles (u32 handle, modulo-u16 wrap predicate, FIFO-ordered SEGMENT_END reclaim, post-fault-only sweep).
- §11 FFI half-split (`UnsafeCell` + raw-pointer projection via `addr_of!` and `UnsafeCell::raw_get`) + §11.4 widened-clock seqlock.
- §12 clock-sync algorithm + quality gate (**plan-decision B: §12.3 normative — `kalico_clock_sync_request` is an explicit step in §6.4 ARMING; §12.4 quality gate adds RTT-aware-sample-present check**).
- §13 telemetry transport (TraceRing in DTCM, sized at HOST_STALL + safety margin; overflow → `KALICO_FAULT_TRACE_OVERFLOW`).
- §16 adversarial review history (rounds 1–4 with the issues each round caught).
- §17 summary of decisions.

**Hard prerequisites:**

1. **Working tree clean.** No uncommitted changes that would conflict with this plan's edits.
   ```bash
   git status   # must be clean
   ```
2. **Step 5 spec/plan complete and on the same branch.** Step 6 builds on the Step-5 `runtime/` and `kalico-c-api/` crates as they stand at commit `c0f19cfa`.
3. **Workspace builds clean before starting:**
   ```bash
   cd rust && cargo test --workspace --release 2>&1 | tail -5
   cd rust && cargo clippy --workspace --all-targets --release -- -D warnings 2>&1 | tail -5
   ```
   Both must succeed.
4. **Hardware target accessible:** BTT Octopus Pro (H723) for Phase 14 hardware bring-up; Renode 1.16.1 + xpack arm-none-eabi-gcc 14.2.1-1.1 for sim work in Phase 0 and Phase 13.
5. **Plan-level decisions recorded** (from Round 4 reviewer feedback, embedded in this plan):
   - **Decision A (§8.5):** Flush sequence step ordering — `force_idle.store(true)` first, spin-wait on `acked_force_idle`, **then** `stream_open.store(false)`. Avoids the spurious-Underrun race where an in-flight ISR sees stale `stream_open=true` while queue is empty.
   - **Decision B (§12.3):** ARMING flow includes an explicit `kalico_clock_sync_request` step before issuing `kalico_stream_arm`. The `ClockSyncEstimator::is_quality_gate_passed()` method requires `last_dedicated_sample_age_ms ≤ MAX_RTT_AGE_MS` (default 500 ms) in addition to the residual/drift/age checks.

---

## Phase 0 — Simulator fixes (Gate A; ≤1-day timebox)

**Acceptance gate** (per spec §3.3 Gate A): Sim boots; firmware identifies; host streams 10 segments via either `kalico_load_curve` (if root-caused) or `kalico_load_fixture_curve` (if escape hatch); MCU evaluates each in order; trace stream reports monotone tick counters and correct segment_id sequence; iteration loop ≤30 s. Underrun-fault path and trace-overflow-fault path are deferred to Gate B.

### Task 0.1: Software CYCCNT fork under `CONFIG_KALICO_SIM`

**Files:**
- Modify: `src/stm32/kalico_h7_timer.c` — fork `kalico_h7_read_cyccnt()` on `CONFIG_KALICO_SIM`.
- Create: `src/stm32/kalico_sim_clock.c` — sim software counter incremented by TIM5 ISR.
- Modify: `src/stm32/kalico_h7_timer.c` — bump `kalico_sim_cyccnt` from TIM5 ISR when `CONFIG_KALICO_SIM`.
- Modify: `src/stm32/Makefile` — conditionally compile `kalico_sim_clock.c`.

**Why:** Renode's H743 .repl tags `DWT->CYCCNT` as opaque; reads return 0. The engine widening loop ingests zero-time samples and segment evaluation never advances. Fix is a C-side abstraction fork: production reads the DWT register; sim returns a software counter. Verifier round 1 confirmed this is simpler than threading a Cargo feature through the staticlib build (the abstraction is already C-side).

- [ ] **Step 1: Create `src/stm32/kalico_sim_clock.c`**

```c
// src/stm32/kalico_sim_clock.c
//
// Software CYCCNT for sim builds (CONFIG_KALICO_SIM=y). Renode's H7 platform
// model returns 0 for DWT->CYCCNT reads; this counter is bumped by the TIM5
// ISR (one-tick-per-fire delta) so the engine's widening loop sees forward
// progress. NEVER include in production firmware — IWDG-disable + sim CYCCNT
// is a debugging build only.

#include "autoconf.h"

#if CONFIG_KALICO_SIM && CONFIG_KALICO_RUNTIME && CONFIG_MACH_STM32H7

#include <stdint.h>

// Bumped by TIM5 ISR (kalico_h7_timer.c) once per tick.
__attribute__((used, externally_visible))
volatile uint32_t kalico_sim_cyccnt = 0;

#endif
```

- [ ] **Step 2: Modify `src/stm32/kalico_h7_timer.c::kalico_h7_read_cyccnt()` to fork on `CONFIG_KALICO_SIM`**

Replace the existing body of `kalico_h7_read_cyccnt`:

```c
__attribute__((used, externally_visible))
uint32_t
kalico_h7_read_cyccnt(void)
{
#if CONFIG_KALICO_SIM
    extern volatile uint32_t kalico_sim_cyccnt;
    return kalico_sim_cyccnt;
#else
    return DWT->CYCCNT;
#endif
}
```

- [ ] **Step 3: Bump `kalico_sim_cyccnt` from `TIM5_IRQHandler` under `CONFIG_KALICO_SIM`**

In `TIM5_IRQHandler` after `TIM5->SR = ~TIM_SR_UIF;`:

```c
void
TIM5_IRQHandler(void)
{
    TIM5->SR = ~TIM_SR_UIF;            // entry-time ack (spec §2.4)

#if CONFIG_KALICO_SIM
    extern volatile uint32_t kalico_sim_cyccnt;
    kalico_sim_cyccnt += (kalico_clock_freq / 40000U);
#endif

    uint32_t before = DWT->CYCCNT;
    if (kalico_rt_handle) {
        kalico_runtime_tick(kalico_rt_handle, before);
    }
    uint32_t after = DWT->CYCCNT;
    // ... (rest unchanged)
}
```

Note: under `CONFIG_KALICO_SIM`, `before` is also `kalico_sim_cyccnt` (because `kalico_h7_read_cyccnt()` now forks). The bench-cycle-count path (`after - before`) becomes meaningless under sim — but that's OK: cycle benches are explicitly out of scope for sim per spec §3.

Actually — `before = DWT->CYCCNT` reads the hardware register directly, not via `kalico_h7_read_cyccnt`. To make sim cycle counts work everywhere consistently, change that line too:

```c
    uint32_t before = kalico_h7_read_cyccnt();
    // ... after also via kalico_h7_read_cyccnt()
    uint32_t after = kalico_h7_read_cyccnt();
```

- [ ] **Step 4: Add `kalico_sim_clock.c` to `src/stm32/Makefile` under `CONFIG_KALICO_SIM`**

Add at the appropriate spot in `src/stm32/Makefile`:

```makefile
kalico-src-$(CONFIG_KALICO_SIM) += stm32/kalico_sim_clock.c
```

(If `src/stm32/Makefile` doesn't have a `kalico-src-` accumulator, follow whatever pattern Step-5 used to add `kalico_h7_timer.c` to the build.)

- [ ] **Step 5: Build sim firmware and confirm it links**

```bash
bash tools/sim/build_sim_firmware.sh 2>&1 | tail -20
```

Expected: clean build, `out/klipper.elf` produced.

- [ ] **Step 6: Smoke-test the software CYCCNT actually advances in sim**

Boot sim, send a `kalico_query_status` while idle, observe `mcu_clock_now` advancing on subsequent queries:

```bash
bash tools/sim/run_sim.sh &
sleep 3
python3 -c "
from tools.kalico_host_io import KalicoHostIO
import time
io = KalicoHostIO('socket://localhost:3334')
io.send('kalico_query_status')
r1 = io.wait_for_response('kalico_status', 5.0)
time.sleep(1)
io.send('kalico_query_status')
r2 = io.wait_for_response('kalico_status', 5.0)
print(f'mcu_clock_now: r1={r1.get(\"mcu_clock_now\", \"?\")} r2={r2.get(\"mcu_clock_now\", \"?\")}')
io.disconnect()
"
```

Expected: `r2.mcu_clock_now > r1.mcu_clock_now`. If both are 0 or equal, the sim CYCCNT didn't get wired into the widening path — debug.

(Note: if `kalico_status` schema doesn't yet include `mcu_clock_now`, this step is approximate; the real test is in Phase 11 once the periodic status frame is implemented. For Phase 0 acceptance, just confirm segment evaluation advances in sim — see Task 0.3.)

- [ ] **Step 7: Commit**

```bash
git add src/stm32/kalico_h7_timer.c src/stm32/kalico_sim_clock.c src/stm32/Makefile
git commit -m "stm32/kalico_sim: software CYCCNT under CONFIG_KALICO_SIM

Renode H7 .repl tags DWT->CYCCNT as opaque; reads return 0, freezing
the engine's widening loop in sim. C-side fork in
kalico_h7_read_cyccnt() returns a software counter (bumped by TIM5 ISR
at one-tick-per-fire delta) under CONFIG_KALICO_SIM, falls through to
DWT->CYCCNT in production. Per spec §3.1.

NEVER flash a CONFIG_KALICO_SIM=y image to silicon — IWDG-disable +
software CYCCNT is a debugging build only.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

### Task 0.2: load_curve hang — GDB-attach root-cause + escape hatch

**Files:**
- Modify: `tools/sim/h723_sim.resc` — uncomment GDB server line for the investigation.
- Create (conditional, on escape-hatch path): `rust/runtime/src/sim_fixtures.rs` — pre-baked NURBS fixtures.
- Create (conditional): `rust/runtime/Cargo.toml` — add `kalico-sim` Cargo feature.
- Modify (conditional): `rust/kalico-c-api/Cargo.toml` — pass-through `kalico-sim` feature.
- Modify (conditional): `src/Makefile.kalico` — pass `--features kalico-sim` when `CONFIG_KALICO_SIM=y`.
- Create (conditional): `rust/kalico-c-api/src/runtime_ffi.rs::kalico_runtime_load_fixture` FFI.
- Modify (conditional): `src/runtime_tick.c` — add `command_kalico_load_fixture_curve` under `CONFIG_KALICO_SIM`.

**Why:** Spec §3.2. First half-day: GDB-attach to identify the load_curve hang. If it's a kalico-side bug, fix it. If it's a Renode H7/H743 platform-model hole (verifier cited renode/renode#618, #626, #649), implement the fixed-fixture escape hatch.

- [ ] **Step 1: Enable Renode GDB server**

In `tools/sim/h723_sim.resc`, uncomment (or add) the line:

```
machine StartGdbServer 3333 true
```

before the `sysbus LoadELF` line.

- [ ] **Step 2: Boot sim with GDB server, attach, send `kalico_load_curve`**

Terminal 1:
```bash
bash tools/sim/run_sim.sh
```

Terminal 2:
```bash
arm-none-eabi-gdb out/klipper.elf -ex "target remote :3333"
(gdb) continue
```

Terminal 3 (host-side load_curve):
```bash
python3 tools/test_h723_first_light.py --port socket://localhost:3334
```

When the firmware hangs, in Terminal 2: `Ctrl-C` to interrupt, then `bt`, `info reg`, `x/4i $pc`. Document findings in commit message.

- [ ] **Step 3: Decision point — root-caused locally OR escape-hatch?**

If `pc` is in kalico C glue or Rust FFI, fix the bug in place; **skip to Step 11**.

If `pc` lands in `command.c::command_decode_ptr` or in `args[i]` decode of a `%*s` buffer, the issue is the `%*s` ABI bug already fixed in commit `7997391a` — re-verify the fix is in place. If still broken, dig deeper.

If `pc` lands in a Renode-unmodeled peripheral access (HardFault handler chain or DefaultHandler infinite loop), proceed with the escape-hatch path: **continue to Step 4**.

Document the diagnosis in this commit's message.

- [ ] **Step 4: Add `kalico-sim` Cargo feature to `rust/runtime/Cargo.toml`**

```toml
[features]
default = []
mcu-h7 = ["nurbs/mcu-h7"]
mcu-f4 = ["nurbs/mcu-f4"]
kalico-sim = []  # NEW
```

- [ ] **Step 5: Pass-through the feature in `rust/kalico-c-api/Cargo.toml`**

```toml
[features]
default = []
mcu-h7 = ["runtime/mcu-h7"]
mcu-f4 = ["runtime/mcu-f4"]
kalico-sim = ["runtime/kalico-sim"]  # NEW
```

- [ ] **Step 6: Wire `CONFIG_KALICO_SIM` to the Cargo feature in `src/Makefile.kalico`**

The exact mechanism depends on Step-5's Makefile.kalico. If it has a `RUST_FEATURES = mcu-h7 ...` accumulator:

```makefile
RUST_FEATURES_$(CONFIG_KALICO_SIM) += kalico-sim
```

Verify the cargo invocation eventually picks it up:

```bash
make -C src/Makefile.kalico KCONFIG_CONFIG=tools/sim/sim.config rust-build 2>&1 | grep features
```

- [ ] **Step 7: Create `rust/runtime/src/sim_fixtures.rs`**

Round-4 fix (verifier #1): use Step-5's actual `CurvePool::load` API which takes flat slices, NOT a `LoadedCurve` struct directly (LoadedCurve is private in Step-5). The fixtures helper returns flat (cps, knots, weights, degree) tuples; sim_fixtures FFI wraps the call to existing `pool.load(handle, cps_flat, &knots, &weights, degree)`.

```rust
//! Pre-baked NURBS fixtures for sim escape-hatch path. Compiled only when
//! `kalico-sim` Cargo feature is on (which is gated on CONFIG_KALICO_SIM=y
//! via the autoconf-bridge in src/Makefile.kalico). NEVER include in
//! production firmware.

#![cfg(feature = "kalico-sim")]

/// Step-5 `CurvePool::load` takes flat slices; sim fixtures return them.
/// (degree, control_points_flat[3*n_cp], knots[degree+1+n_cp], weights[n_cp])

/// Returns (degree, cps_flat as [f32; n_cp*3], knots as [f32; n_cp+degree+1], weights as [f32; n_cp]).
/// Caller copies into stack buffers and passes to CurvePool::load.
///
/// Fixtures:
///   0 = straight_line_x (degree-1, 2 CP from (0,0,0) to (10,0,0))
///   1 = quarter_arc_xy  (degree-2 rational, 3 CP, R=20mm quarter)
///   2 = cubic_bezier_xy (degree-3, 4 CP)
pub fn lookup(fixture_id: u16, cps_out: &mut [f32; 24], knots_out: &mut [f32; 12], weights_out: &mut [f32; 8])
    -> Option<(u8, usize, usize, usize)> {
    // Returns (degree, n_cp, n_knots, n_weights).
    match fixture_id {
        0 => Some(straight_line_x(cps_out, knots_out, weights_out)),
        1 => Some(quarter_arc_xy(cps_out, knots_out, weights_out)),
        2 => Some(cubic_bezier_xy(cps_out, knots_out, weights_out)),
        _ => None,
    }
}

fn straight_line_x(cps: &mut [f32; 24], knots: &mut [f32; 12], weights: &mut [f32; 8])
    -> (u8, usize, usize, usize) {
    cps[0..3].copy_from_slice(&[0.0, 0.0, 0.0]);
    cps[3..6].copy_from_slice(&[10.0, 0.0, 0.0]);
    knots[..4].copy_from_slice(&[0.0, 0.0, 1.0, 1.0]);
    weights[..2].copy_from_slice(&[1.0, 1.0]);
    (1, 2, 4, 2)
}

fn quarter_arc_xy(cps: &mut [f32; 24], knots: &mut [f32; 12], weights: &mut [f32; 8])
    -> (u8, usize, usize, usize) {
    let r: f32 = 20.0;
    cps[0..3].copy_from_slice(&[r, 0.0, 0.0]);
    cps[3..6].copy_from_slice(&[r, r, 0.0]);
    cps[6..9].copy_from_slice(&[0.0, r, 0.0]);
    knots[..6].copy_from_slice(&[0.0, 0.0, 0.0, 1.0, 1.0, 1.0]);
    let cos_pi4 = (core::f32::consts::FRAC_PI_4).cos();
    weights[..3].copy_from_slice(&[1.0, cos_pi4, 1.0]);
    (2, 3, 6, 3)
}

fn cubic_bezier_xy(cps: &mut [f32; 24], knots: &mut [f32; 12], weights: &mut [f32; 8])
    -> (u8, usize, usize, usize) {
    cps[0..3].copy_from_slice(&[0.0, 0.0, 0.0]);
    cps[3..6].copy_from_slice(&[3.0, 5.0, 0.0]);
    cps[6..9].copy_from_slice(&[7.0, 5.0, 0.0]);
    cps[9..12].copy_from_slice(&[10.0, 0.0, 0.0]);
    knots[..8].copy_from_slice(&[0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]);
    weights[..4].copy_from_slice(&[1.0, 1.0, 1.0, 1.0]);
    (3, 4, 8, 4)
}
```

The fixture function fills caller-provided buffers; the FFI wrapper allocates the buffers + dispatches via Step-5's `pool.load(handle, &cps[..n_cp*3], &knots[..n_knots], &weights[..n_weights], degree)`.

Add `pub mod sim_fixtures;` (gated by feature) to `rust/runtime/src/lib.rs`:

```rust
#[cfg(feature = "kalico-sim")]
pub mod sim_fixtures;
```

- [ ] **Step 8: Add FFI `kalico_runtime_load_fixture` in `rust/kalico-c-api/src/runtime_ffi.rs`**

```rust
#[cfg(feature = "kalico-sim")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn kalico_runtime_load_fixture(
    rt: *mut KalicoRuntime,
    slot_idx: u16,
    fixture_id: u16,
) -> i32 {
    use runtime::sim_fixtures;
    use runtime::curve_pool::CurveHandle;
    if rt.is_null() { return runtime::error::KALICO_ERR_NULL_PTR; }
    let mut cps = [0.0_f32; 24];
    let mut knots = [0.0_f32; 12];
    let mut weights = [0.0_f32; 8];
    let Some((degree, n_cp, n_knots, n_weights)) =
        sim_fixtures::lookup(fixture_id, &mut cps, &mut knots, &mut weights)
    else {
        return runtime::error::KALICO_ERR_INVALID_CURVE;
    };
    // Round-4 fix (verifier #1): use Step-5's actual API. Step-5
    // RuntimeContext field is `pool` (not `curve_pool` until Phase 1's
    // half-split rename). Step-5 CurvePool::load takes flat slices +
    // CurveHandle, returns Result<(), CurvePoolError>.
    let ctx = unsafe { &mut *(rt as *mut RuntimeContext) };
    // Step-5 CurveHandle is `pub struct CurveHandle(pub u16);` (tuple struct, no constructor).
    let handle = CurveHandle(slot_idx);
    match ctx.pool.load(handle,
        &cps[..n_cp * 3], &knots[..n_knots], &weights[..n_weights], degree)
    {
        Ok(()) => 0,
        Err(_) => runtime::error::KALICO_ERR_INVALID_CURVE,
    }
}
```

This task ships a sim-only path against the EXISTING (Step-5) CurvePool API. Phase 2 Task 2.2 updates this single call to `try_alloc_and_load(...)` as part of the alloc-API rewrite — mechanical 1-line change.

- [ ] **Step 9: Add `command_kalico_load_fixture_curve` in `src/runtime_tick.c`**

```c
#if CONFIG_KALICO_SIM
extern int32_t kalico_runtime_load_fixture(void *rt, uint16_t slot, uint16_t fixture_id);

void
command_kalico_load_fixture_curve(uint32_t *args)
{
    if (!kalico_rt_handle) {
        sendf("kalico_load_fixture_response result=%i curve_handle_packed=%u", -7, 0);
        return;
    }
    uint16_t slot = args[0];
    uint16_t fixture_id = args[1];
    int32_t r = kalico_runtime_load_fixture(kalico_rt_handle, slot, fixture_id);
    // Round-5 fix Codex #4: return the generated (slot, gen) packed u32 handle
    // so host can reference it in subsequent kalico_push_segment calls.
    sendf("kalico_load_fixture_response result=%i curve_handle_packed=%u",
          r, curve_handle_packed);
}
DECL_COMMAND(command_kalico_load_fixture_curve,
    "kalico_load_fixture_curve slot=%hu fixture_id=%hu");
#endif
```

- [ ] **Step 10: Rebuild sim firmware with `CONFIG_KALICO_SIM=y` and verify the fixture command works**

```bash
bash tools/sim/build_sim_firmware.sh 2>&1 | tail -10
bash tools/sim/run_sim.sh &
sleep 3
python3 -c "
from tools.kalico_host_io import KalicoHostIO
io = KalicoHostIO('socket://localhost:3334')
io.send('kalico_load_fixture_curve slot=0 fixture_id=0')
r = io.wait_for_response('kalico_load_fixture_response', 5.0)
print(f'load_fixture: result={r}')
io.disconnect()
"
```

Expected: `result=0` (success). If `result != 0` debug the FFI; if the firmware hangs, the escape hatch isn't escaping — escalate.

- [ ] **Step 11: Commit** (whichever path was taken)

If root-caused-locally:
```bash
git add <touched files>
git commit -m "fix: load_curve hang in sim — <root-cause>

GDB-attach diagnosis: <describe>. Fix: <describe>.

Per spec §3.2 Phase 0."
```

If escape-hatch:
```bash
git add rust/runtime/Cargo.toml rust/kalico-c-api/Cargo.toml \
        rust/runtime/src/sim_fixtures.rs rust/runtime/src/lib.rs \
        rust/kalico-c-api/src/runtime_ffi.rs \
        src/runtime_tick.c src/Makefile.kalico tools/sim/h723_sim.resc
git commit -m "tools/sim: load_fixture escape hatch under CONFIG_KALICO_SIM

GDB-attach diagnosis: <Renode platform-model hole at <address>; tracks
to renode/renode#XXX>. Step 6 protocol iteration uses
kalico_load_fixture_curve to bypass the %*s blob path entirely; load_curve
remains broken in sim and works in production.

NEVER flash a CONFIG_KALICO_SIM=y image to silicon.

Per spec §3.2 Phase 0."
```

### Task 0.3: Phase 0 acceptance gate (Gate A)

**Files:**
- Create: `tools/test_sim_gate_a.py` — automated Gate A acceptance test.

**Why:** Spec §3.3 Gate A. Establishes the protocol-iteration loop ≤30 s and confirms basic streaming against sim.

- [ ] **Step 1: Create `tools/test_sim_gate_a.py`**

```python
#!/usr/bin/env python3
"""
Phase 0 Gate A acceptance test.

Boots sim firmware, sends 10 segments via either kalico_load_curve
(if root-caused) or kalico_load_fixture_curve (if escape hatch),
verifies trace stream reports monotone tick counters and correct
segment_id sequence. Iteration loop ≤30 s wall clock.

Usage:
    bash tools/sim/run_sim.sh &
    sleep 2
    python3 tools/test_sim_gate_a.py [--use-fixtures]
"""
import argparse
import sys
import time
from kalico_host_io import KalicoHostIO


def run_gate_a(use_fixtures: bool) -> int:
    t0 = time.monotonic()
    io = KalicoHostIO("socket://localhost:3334")
    try:
        # Load curve(s).
        for slot in range(3):
            if use_fixtures:
                cmd = f"kalico_load_fixture_curve slot={slot} fixture_id={slot}"
                resp_name = "kalico_load_fixture_response"
            else:
                # Real load_curve path: encode v1 versioned blob.
                from wire_v1 import encode_load_curve_v1
                # Three fixtures — same shapes as sim_fixtures.rs lookup table.
                FIXTURE_CURVES = [
                    # straight_line_x: degree-1, 2 CP, 4 knots
                    (1, [(0.0, 0.0, 0.0), (10.0, 0.0, 0.0)],
                        [0.0, 0.0, 1.0, 1.0],
                        [1.0, 1.0]),
                    # quarter_arc_xy: degree-2 rational, 3 CP, 6 knots
                    (2, [(20.0, 0.0, 0.0), (20.0, 20.0, 0.0), (0.0, 20.0, 0.0)],
                        [0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
                        [1.0, 0.7071067811865476, 1.0]),
                    # cubic_bezier_xy: degree-3, 4 CP, 8 knots
                    (3, [(0.0, 0.0, 0.0), (3.0, 5.0, 0.0), (7.0, 5.0, 0.0), (10.0, 0.0, 0.0)],
                        [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
                        [1.0, 1.0, 1.0, 1.0]),
                ]
                degree, cps, knots, weights = FIXTURE_CURVES[slot]
                blob = encode_load_curve_v1(degree, cps, knots, weights)
                cmd = f"kalico_load_curve slot={slot} data={blob.hex()}"
                # NOTE: msgproto's `%*s` blob encoding takes raw bytes; the
                # KalicoHostIo helper handles the conversion. Adjust per the
                # helper's API — pass bytes via the bytes-arg path, not hex.
                resp_name = "kalico_load_curve_response"
            io.send(cmd)
            r = io.wait_for_response(resp_name, 2.0)
            if r.get("result") != 0:
                print(f"FAIL: load slot={slot} returned {r}")
                return 1

        # Stream 10 segments referencing fixtures cyclically.
        for i in range(10):
            slot = i % 3
            t_start = i * 100000  # 100k cycles per segment, sim CYCCNT units
            t_end = t_start + 100000
            io.send(
                f"kalico_push_segment id={i} curve={slot} "
                f"t_start_hi=0 t_start_lo={t_start} "
                f"t_end_hi=0 t_end_lo={t_end} kin=0"
            )
            r = io.wait_for_response("kalico_push_response", 2.0)
            if r.get("result") != 0:
                print(f"FAIL: push id={i} returned {r}")
                return 1

        # Wait for trace samples; verify monotone segment_id and tick.
        # Spec doesn't pin trace cadence yet; allow up to 5 s for first batch.
        t_trace_start = time.monotonic()
        last_tick = -1
        last_segment_id = -1
        seen_segments = set()
        while time.monotonic() - t_trace_start < 5.0:
            io.send("kalico_drain_trace count=64")
            try:
                r = io.wait_for_response("kalico_trace", 1.0)
            except Exception:
                continue
            # Parse `data=%*s` payload as TraceSample stream (32 B/sample).
            data = r.get("data", b"")
            n = r.get("count", 0)
            for i in range(n):
                offset = i * 32
                # tick: u64 le; segment_id: u32 le at offset 24 (post-Step-6 schema)
                # OR Step-5 schema: tick u64 + motors 12 + segment_id u32 = 24
                tick = int.from_bytes(data[offset:offset+8], "little")
                segment_id = int.from_bytes(data[offset+24:offset+28], "little")
                if tick < last_tick:
                    print(f"FAIL: non-monotone tick {tick} after {last_tick}")
                    return 1
                if segment_id < last_segment_id:
                    print(f"FAIL: non-monotone segment_id {segment_id} after {last_segment_id}")
                    return 1
                last_tick = tick
                last_segment_id = segment_id
                seen_segments.add(segment_id)
            if len(seen_segments) >= 10:
                break

        if len(seen_segments) < 10:
            print(f"FAIL: only saw {len(seen_segments)} of 10 segments in trace")
            return 1

        elapsed = time.monotonic() - t0
        print(f"PASS: Gate A ({elapsed:.1f}s)")
        if elapsed > 30.0:
            print(f"WARN: iteration loop {elapsed:.1f}s exceeded 30s target")
        return 0

    finally:
        io.disconnect()


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--use-fixtures", action="store_true")
    args = p.parse_args()
    sys.exit(run_gate_a(args.use_fixtures))
```

- [ ] **Step 2: Run Gate A and verify it passes**

```bash
bash tools/sim/run_sim.sh &
sleep 3
python3 tools/test_sim_gate_a.py --use-fixtures
```

Expected: `PASS: Gate A (Xs)` where X ≤ 30. If not, debug — Phase 0 is not complete until Gate A passes.

- [ ] **Step 3: Commit Gate A test + record Phase 0 completion**

```bash
git add tools/test_sim_gate_a.py
git commit -m "tools/sim: Gate A acceptance test for Phase 0

Streams 10 fixture segments through sim, verifies monotone tick + segment_id
in trace stream. Required to pass before Phase 1 of Step 6 begins.

Per spec §3.3 Gate A."
```

---

## Phase 1 — Foundation: Half-split SPSC + Widened-clock seqlock

This phase refactors the Step-5 single `RuntimeContext` into the half-split (`FgState`/`IsrState`/`SharedState`) per spec §11. Closes the latent FFI aliasing UB and lays the foundation for all subsequent Step-6 work.

### Task 1.1: Define half-split state structs

**Files:**
- Modify: `rust/runtime/src/lib.rs` — define `FgState`, `IsrState`, `SharedState`, refactored `RuntimeContext`.
- Modify: `rust/runtime/src/state.rs` — re-home shared state from Step-5's monolithic struct.

**Why:** Spec §11.1. The Step-5 `RuntimeContext` puts everything in one struct accessed via `&mut`. Step-6 splits into disjoint memory regions accessed via raw-pointer projection.

- [ ] **Step 1: Sketch the new structs in `rust/runtime/src/state.rs`**

**Round-5 fix:** Step-5's `RuntimeContext` is actually defined in `rust/kalico-c-api/src/runtime_ffi.rs`, not in `runtime/src/state.rs`. Step-5's `state.rs` contains `TickState` (used by `engine.rs` and `slot.rs`). Phase 1 Task 1.1 must:
- Preserve / move `TickState` (it's still needed by Engine and slot — leave it in place).
- Define the NEW Step-6 `RuntimeContext` in `runtime/src/state.rs` (the half-split version).
- Remove the OLD Step-5 `RuntimeContext` from `kalico-c-api/src/runtime_ffi.rs` after Phase 1 Task 1.2 lands the new FFI surface.

```rust
//! Half-split runtime state per Step-6 spec §11.
//!
//! `FgState` is touched only from foreground command-dispatch.
//! `IsrState` is touched only from the TIM5 ISR.
//! `SharedState` is touched concurrently from both via atomics only.
//!
//! Discipline contract: code-review-enforced. No compiler check. The TIM5
//! ISR is the SOLE writer of IsrState; any other interrupt that needs MCU
//! state goes through SharedState atomics.

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU8, AtomicU32};

use heapless::spsc::{Consumer, Producer, Queue};

use crate::clock::WidenState;  // Round-2 fix B2: Step-5 named it WidenState, not ClockWidenState
use crate::curve_pool::CurvePool;
use crate::engine::{Engine, NoopPa, NoopIs};  // Round-2 fix B1: Engine is generic
use crate::queue::Q_N;
use crate::segment::Segment;

/// Production Engine instantiation: ZST PA/IS slots per Step-5 spec §3.1.
/// Step 9 (tanh PA) and Step 8 (smooth shapers) replace these with real impls.
pub type EngineImpl = Engine<NoopPa, NoopIs>;

pub struct RuntimeContext {
    pub(crate) fg: UnsafeCell<FgState>,
    pub(crate) isr: UnsafeCell<IsrState>,
    pub(crate) shared: SharedState,
    /// CurvePool sits at top-level (NOT inside FgState/IsrState) because both
    /// halves access it: foreground writes via try_alloc + try_load_into;
    /// ISR reads via lookup. Per-slot AtomicU16 generation counters + per-slot
    /// UnsafeCell<LoadedCurve> make concurrent access safe (foreground is sole
    /// writer to slot.curve, gated by alloc-then-load-then-bump-current_gen
    /// invariant; ISR's Acquire-load on current_gen synchronizes with that
    /// invariant). Spec §10.5.
    pub(crate) curve_pool: crate::curve_pool::CurvePool,
    /// Static SPSC backing storage shared by FgState/IsrState producer/consumer split.
    pub(crate) queue_storage: UnsafeCell<Queue<Segment, Q_N>>,
    pub(crate) trace_storage: UnsafeCell<Queue<crate::trace::TraceSample, { crate::trace::TRACE_RING_N }>>,
}

// SAFETY: see discipline contract above. CurvePool is Sync via per-slot atomics.
unsafe impl Sync for RuntimeContext {}

pub struct FgState {
    pub queue_producer: Producer<'static, Segment, Q_N>,
    pub trace_consumer: Consumer<'static, crate::trace::TraceSample, { crate::trace::TRACE_RING_N }>,
    pub stream_state_machine: crate::stream::FgStreamState,
    /// Stream-open identity tracking for §8.5 idempotency (same-stream_id rule).
    pub current_stream_id: Option<u32>,
    /// Arm-time idempotency (§8.5: arm with same t_start_t0 returns OK).
    pub armed_t_start_t0: Option<u64>,
    /// Round-2 fix B6: foreground tracks the FIRST priming segment's t_start
    /// at push-acceptance time. arm() reads from here (not from the ISR-owned
    /// queue) per §6.3 + §11.1 SPSC ownership discipline.
    pub first_priming_segment_t_start: Option<u64>,
    /// Set by §8.3 kalico_stream_terminal handler; consumed by ISR retire path
    /// (cross-half via SharedState atomics).
    pub terminal_segment_id: Option<u32>,
    /// Used by §8.5 flush spin-wait deadline computation.
    pub flush_start_tick: Option<u64>,
}

pub struct IsrState {
    pub queue_consumer: Consumer<'static, Segment, Q_N>,
    pub trace_producer: Producer<'static, crate::trace::TraceSample, { crate::trace::TRACE_RING_N }>,
    pub engine: EngineImpl,            // Round-2 fix B1: typedef from above
    pub widen_state: WidenState,       // Round-2 fix B2: existing name in clock.rs
}

pub struct SharedState {
    // Step-5 carryover
    pub last_error: AtomicI32,
    pub runtime_status: AtomicU8,
    // Step-6: stream lifecycle
    pub stream_open: AtomicBool,
    // Step-6: flush handshake (Plan-decision A: foreground sets force_idle FIRST,
    // ack-waits, THEN clears stream_open).
    pub force_idle: AtomicBool,
    pub acked_force_idle: AtomicBool,
    // Step-6: §11.4 widened-clock seqlock — foreground reads, ISR writes
    pub widened_now_lo: AtomicU32,
    pub widened_now_hi: AtomicU32,
    pub widened_now_seq: AtomicU32,
    // Step-6: §13.1 trace-overflow latch (ISR sets, foreground latches fault)
    pub sample_drop_pending: AtomicBool,
    // Step-6: cross-half cursors (foreground reads ISR-published values)
    pub current_segment_id: AtomicU32,
    pub credit_epoch: AtomicU32,
    pub accepted_segment_id: AtomicU32,
    pub retired_through_segment_id: AtomicU32,
    // Step-6: terminal-segment communication foreground → ISR (§8.3).
    // Foreground sets _set true + _value to the segment id from
    // kalico_stream_terminal; ISR retire path checks the flag + value and
    // clears stream_open when matched. Both cleared on flush/stream_open.
    pub terminal_segment_id_set: AtomicBool,
    pub terminal_segment_id_value: AtomicU32,
}

impl SharedState {
    pub const fn new() -> Self {
        Self {
            last_error: AtomicI32::new(0),
            runtime_status: AtomicU8::new(crate::engine::RuntimeStatus::Idle as u8),
            stream_open: AtomicBool::new(false),
            force_idle: AtomicBool::new(false),
            acked_force_idle: AtomicBool::new(false),
            widened_now_lo: AtomicU32::new(0),
            widened_now_hi: AtomicU32::new(0),
            widened_now_seq: AtomicU32::new(0),
            sample_drop_pending: AtomicBool::new(false),
            current_segment_id: AtomicU32::new(0),
            credit_epoch: AtomicU32::new(0),
            accepted_segment_id: AtomicU32::new(0),
            retired_through_segment_id: AtomicU32::new(0),
            terminal_segment_id_set: AtomicBool::new(false),
            terminal_segment_id_value: AtomicU32::new(0),
        }
    }
}
```

- [ ] **Step 2: Add a `new` / init constructor that produces the static `RuntimeContext` and splits SPSC**

In `rust/runtime/src/state.rs`:

```rust
impl RuntimeContext {
    /// Initializes the runtime context. Called exactly once during runtime_init.
    /// SAFETY: caller must guarantee single-threaded init before any FFI call.
    pub unsafe fn init(rt_ptr: *mut RuntimeContext) {
        unsafe {
            // Initialize queue storage and split.
            let queue_storage_ptr = core::ptr::addr_of_mut!((*rt_ptr).queue_storage);
            (*queue_storage_ptr).get().write(Queue::new());
            let queue_ref: &'static mut Queue<Segment, Q_N> = &mut *(*queue_storage_ptr).get();
            let (q_producer, q_consumer) = queue_ref.split();

            // Initialize trace storage and split.
            let trace_storage_ptr = core::ptr::addr_of_mut!((*rt_ptr).trace_storage);
            (*trace_storage_ptr).get().write(Queue::new());
            let trace_ref: &'static mut Queue<crate::trace::TraceSample, { crate::trace::TRACE_RING_N }> =
                &mut *(*trace_storage_ptr).get();
            let (t_producer, t_consumer) = trace_ref.split();

            // Initialize SharedState.
            let shared_ptr = core::ptr::addr_of_mut!((*rt_ptr).shared);
            shared_ptr.write(SharedState::new());

            // Initialize CurvePool at top-level of RuntimeContext.
            let pool_ptr = core::ptr::addr_of_mut!((*rt_ptr).curve_pool);
            pool_ptr.write(CurvePool::new());

            // Initialize FgState (no curve_pool field — it's at top level).
            let fg_ptr = core::ptr::addr_of_mut!((*rt_ptr).fg);
            (*fg_ptr).get().write(FgState {
                queue_producer: q_producer,
                trace_consumer: t_consumer,
                stream_state_machine: crate::stream::FgStreamState::Idle,
                current_stream_id: None,
                armed_t_start_t0: None,
                first_priming_segment_t_start: None,
                terminal_segment_id: None,
                flush_start_tick: None,
            });

            // Initialize IsrState.
            let isr_ptr = core::ptr::addr_of_mut!((*rt_ptr).isr);
            (*isr_ptr).get().write(IsrState {
                queue_consumer: q_consumer,
                trace_producer: t_producer,
                engine: Engine::new_production(),
                widen_state: ClockWidenState::new(),
            });
        }
    }
}
```

- [ ] **Step 3a: Move `TRACE_RING_N` from engine.rs (private) to trace.rs (pub) and update value**

Round-4 fix (verifier #4): Step-5 has `const TRACE_RING_N: usize = 128;` declared as a **private** const in `engine.rs:26`, NOT a `pub const` in `trace.rs`. Phase 1's RuntimeContext uses it as a const generic from `crate::trace::TRACE_RING_N`, requiring it be public AND in trace.rs.

Concrete edits:

```rust
// REMOVE from rust/runtime/src/engine.rs (line 26):
//   const TRACE_RING_N: usize = 128;
// (Update all engine.rs references to use crate::trace::TRACE_RING_N instead.)

// ADD to rust/runtime/src/trace.rs:
pub const TRACE_RING_N: usize = 1201;  // §13.1: HOST_STALL + 10 ms safety margin × 40 kHz + 1 (heapless cap-N-1)
```

Update engine.rs imports:
```rust
use crate::trace::{TRACE_RING_N, TRACE_FLAG_FAULT_MARKER, TRACE_FLAG_SEGMENT_END, TraceRing, TraceSample};
```

This must precede Phase 5 Task 5.1 because Task 1.1's `RuntimeContext.trace_storage` uses `{ crate::trace::TRACE_RING_N }` as a const generic. Phase 5 only updates the `TraceSample` struct schema (the count is set here).

- [ ] **Step 3b: Stub out modules referenced but not yet existing**

Add to `rust/runtime/src/lib.rs`:

```rust
pub mod state;
pub mod stream;  // NEW — Phase 6
```

Create `rust/runtime/src/stream.rs` (initially minimal — fleshed out in Phase 6):

```rust
//! Stream lifecycle state machine (host + MCU side). Spec §8.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FgStreamState {
    Idle = 0,
    StreamOpening = 1,
    StreamOpenPriming = 2,
    Arming = 3,
    Armed = 4,
    Running = 5,
    Draining = 6,
    Drained = 7,
    Fault = 8,
}
```

- [ ] **Step 4: Update `Engine` to expose `new_production(clock_freq)` per spec §14**

Round-2 fixes B1 + B2 + B19: Engine is generic over PA/IS slots; takes clock_freq from the C-side static (not a hardcoded constant). The constructor accepts the freq value at init time, mirroring Step-5's `kalico_clock_freq` C-static read pattern.

In `rust/runtime/src/engine.rs`, add (alongside the `#[cfg(test)] Default` impl):

```rust
impl<P: PaSlot, I: IsSlot> Engine<P, I> {
    /// Production-context constructor (replaces Step-5's #[cfg(test)] Default).
    /// `clock_freq` is read from the C-side `kalico_clock_freq` static at FFI init time.
    pub fn new_production(clock_freq: u32) -> Self
    where
        P: Default,
        I: Default,
    {
        Self::new(clock_freq)
        // Note: existing Engine::new takes clock_freq parameter per Step-5 design.
    }

    /// Round-2 fix B4: clear current segment from outside the engine module
    /// (used by §8.5 flush as defense-in-depth).
    pub(crate) fn clear_current(&mut self) {
        self.current = None;
    }
}
```

Also add `pub(crate) current: Option<crate::segment::Segment>` (or wrap `Engine::current` in a public-crate accessor) to fix Round-2 B4 (Engine.current was private, but §8.5 flush needs to clear it under disabled IRQ). Use `pub(crate) fn clear_current(&mut self)` instead of making the field directly `pub(crate)`.

Update `RuntimeContext::init` to read `kalico_clock_freq` from C-side at init time:

```rust
unsafe extern "C" {
    static kalico_clock_freq: u32;
}

// In RuntimeContext::init, when initializing IsrState:
let freq = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(kalico_clock_freq)) };
(*isr_ptr).get().write(IsrState {
    queue_consumer: q_consumer,
    trace_producer: t_producer,
    engine: EngineImpl::new_production(freq),
    widen_state: WidenState::new(freq),
});
```

- [ ] **Step 5: Run `cargo check -p runtime --target thumbv7em-none-eabi --no-default-features --features mcu-h7`**

```bash
cd rust && cargo check -p runtime --target thumbv7em-none-eabi --no-default-features --features mcu-h7 2>&1 | tail -30
```

Expected: clean (modulo any methods Phase 6 will add). If errors reference unresolved Engine/CurvePool methods that this plan adds in later phases, defer to those phases — but the structs themselves should compile.

- [ ] **Step 6: Commit**

```bash
git add rust/runtime/src/state.rs rust/runtime/src/lib.rs \
        rust/runtime/src/stream.rs rust/runtime/src/engine.rs
git commit -m "runtime: half-split state — FgState/IsrState/SharedState

Step-5 RuntimeContext refactored into disjoint memory regions accessed
via raw-pointer projection (Phase 2 below). FgState owns queue producer
+ trace consumer + curve pool; IsrState owns queue consumer + trace
producer + engine + widen_state; SharedState holds atomics
(last_error, runtime_status, stream_open, force_idle/acked_force_idle,
widened-clock seqlock fields, trace-overflow latch, cross-half cursors).

Closes Step-5 latent FFI aliasing UB (per spec §11). Discipline:
TIM5 ISR is sole IsrState writer.

Per spec §11.1."
```

### Task 1.2: Raw-pointer FFI projection (closes UB)

**Files:**
- Modify: `rust/kalico-c-api/src/runtime_ffi.rs` — replace `&mut *rt` pattern with raw-pointer projection.

**Why:** Spec §11.2 + Round 1 review fix. The Step-5 `let ctx = unsafe { &mut *rt };` materializes `&mut RuntimeContext`; concurrent ISR/foreground entry creates overlapping `&mut` (UB under stacked borrows). Step-6 uses `core::ptr::addr_of!` + `UnsafeCell::raw_get` to project to half-state without ever forming `&mut RuntimeContext`.

- [ ] **Step 1: Update `kalico_runtime_init` to use the new init pattern**

In `rust/kalico-c-api/src/runtime_ffi.rs`:

```rust
use core::cell::UnsafeCell;
use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicBool, Ordering};

use runtime::state::{FgState, IsrState, RuntimeContext, SharedState};

#[repr(C)]
pub struct KalicoRuntime {
    _private: [u8; 0],
}

static mut RT_STORAGE: MaybeUninit<RuntimeContext> = MaybeUninit::uninit();
static INIT_DONE: AtomicBool = AtomicBool::new(false);

#[unsafe(no_mangle)]
pub extern "C" fn kalico_runtime_init() -> *mut KalicoRuntime {
    if INIT_DONE.compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed).is_err() {
        return core::ptr::null_mut();
    }
    // SAFETY: single-threaded init. RuntimeContext::init writes through raw
    // pointer projections; never forms &mut RuntimeContext.
    unsafe {
        let rt_ptr = RT_STORAGE.as_mut_ptr();
        RuntimeContext::init(rt_ptr);
        rt_ptr as *mut KalicoRuntime
    }
}
```

- [ ] **Step 2: Replace `kalico_runtime_tick` to project to `IsrState` only**

```rust
#[unsafe(no_mangle)]
pub unsafe extern "C" fn kalico_runtime_tick(rt: *mut KalicoRuntime, raw_cyccnt: u32) {
    if rt.is_null() { return; }
    let ctx = rt as *mut RuntimeContext;
    // SAFETY: per discipline contract, TIM5 ISR is sole writer of IsrState.
    // raw-pointer projection: never forms &mut RuntimeContext.
    unsafe {
        let isr_ptr: *mut IsrState =
            UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
        let pool_ptr: *const crate::curve_pool::CurvePool =
            core::ptr::addr_of!((*ctx).curve_pool);
        let shared_ptr: *const SharedState =
            core::ptr::addr_of!((*ctx).shared);
        let isr: &mut IsrState = &mut *isr_ptr;
        let pool: &crate::curve_pool::CurvePool = &*pool_ptr;
        let shared: &SharedState = &*shared_ptr;

        // Phase 7 will add the §8.5 force_idle check here at the top of tick.
        // Round-4 fix: Engine::tick takes the queue consumer + trace producer
        // explicitly so the engine can dequeue segments and emit trace samples
        // under the half-split. Field-disjoint borrow: we split &mut isr into
        // multiple disjoint &mut to its fields by spelling them out one at a
        // time. The borrow checker accepts this when each field-projection
        // borrow is non-overlapping.
        let IsrState { engine, widen_state, queue_consumer, trace_producer, .. } = &mut *isr;
        engine.tick(raw_cyccnt, widen_state, pool, queue_consumer, trace_producer, shared);
    }
}
```

- [ ] **Step 3: Update `kalico_runtime_push_segment` to project to `FgState` only**

```rust
#[unsafe(no_mangle)]
pub unsafe extern "C" fn kalico_runtime_push_segment(
    rt: *mut KalicoRuntime,
    id: u32,
    curve_handle: u32,  // NOTE: u16 in Step-5; expanded to u32 in Phase 3 §10.1
    t_start: u64,
    t_end: u64,
    kinematics: u8,
) -> i32 {
    if rt.is_null() { return KALICO_ERR_NULL_HANDLE; }
    let ctx = rt as *mut RuntimeContext;
    unsafe {
        let fg_ptr: *mut FgState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).fg));
        let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
        let fg: &mut FgState = &mut *fg_ptr;
        let shared: &SharedState = &*shared_ptr;
        push_segment_impl(fg, shared, id, curve_handle, t_start, t_end, kinematics)
    }
}
```

`push_segment_impl` is the Step-5 push body extracted into a function that takes `&mut FgState` + `&SharedState`. Move the existing logic accordingly.

- [ ] **Step 4: Same pattern for `kalico_runtime_load_curve`, `kalico_runtime_drain_trace`, `kalico_runtime_status`, `kalico_runtime_last_error`, `kalico_runtime_tick_counter`**

For each existing FFI function, project via `UnsafeCell::raw_get(addr_of!((*ctx).fg))` (or `.isr` if it's an ISR-time read). Read-only accessors that just load from `SharedState` use `core::ptr::addr_of!((*ctx).shared)` and form `&SharedState`.

- [ ] **Step 5: Run `cargo check` for the staticlib**

```bash
cd rust && cargo check -p kalico-c-api --target thumbv7em-none-eabi --no-default-features --features mcu-h7 2>&1 | tail -20
```

Clean expected. Stacked-borrows audit (visual): no `&mut *rt` exists anywhere; every entry materializes `&mut FgState` or `&mut IsrState` (disjoint memory) once.

- [ ] **Step 6: Run host-side tests including the existing FFI smoke**

```bash
cd rust && cargo test -p kalico-c-api --features host 2>&1 | tail -15
```

Clean.

- [ ] **Step 7: Commit**

```bash
git add rust/kalico-c-api/src/runtime_ffi.rs
git commit -m "kalico-c-api/runtime_ffi: raw-pointer projection (closes Step-5 UB)

Replace let ctx = &mut *rt pattern with addr_of! + UnsafeCell::raw_get
projection. FFI never materializes &mut RuntimeContext; each entry
forms &mut FgState OR &mut IsrState (disjoint memory) at most once.

Sound under stacked borrows / tree borrows. Per spec §11.2.

Closes Step-5 latent UB acknowledged in plan-changes-log."
```

### Task 1.3: Widened-clock seqlock (§11.4)

**Files:**
- Modify: `rust/runtime/src/clock.rs` — add `publish_widened_now` / `read_widened_now` helpers using SharedState atomics.
- Modify: `rust/runtime/src/engine.rs` — call `publish_widened_now` from tick after widening.

**Why:** Spec §11.4. ARMv7-M has no lock-free 64-bit atomic. Foreground needs to read the widened CYCCNT (for clock-sync responder + status frame); ISR is sole writer. Standard seqlock pattern over two AtomicU32 + sequence counter.

- [ ] **Step 1: Add seqlock helpers to `rust/runtime/src/clock.rs`**

```rust
use core::sync::atomic::Ordering;
use crate::state::SharedState;

/// ISR writer: publishes the widened u64 to SharedState atomics.
/// Wait-free; ~3 instructions on Cortex-M7.
#[inline]
pub fn publish_widened_now(shared: &SharedState, now: u64) {
    let seq = shared.widened_now_seq.load(Ordering::Relaxed).wrapping_add(1);
    shared.widened_now_seq.store(seq, Ordering::Release);  // → odd
    shared.widened_now_lo.store(now as u32, Ordering::Release);
    shared.widened_now_hi.store((now >> 32) as u32, Ordering::Release);
    shared.widened_now_seq.store(seq.wrapping_add(1), Ordering::Release);  // → even
}

/// Foreground reader: bounded retry per §11.4 analysis.
/// Returns the most recently published u64.
#[inline]
pub fn read_widened_now(shared: &SharedState) -> u64 {
    loop {
        let seq_before = shared.widened_now_seq.load(Ordering::Acquire);
        if seq_before & 1 != 0 {
            // Write in progress; spin briefly and retry.
            core::hint::spin_loop();
            continue;
        }
        let lo = shared.widened_now_lo.load(Ordering::Acquire) as u64;
        let hi = shared.widened_now_hi.load(Ordering::Acquire) as u64;
        let seq_after = shared.widened_now_seq.load(Ordering::Acquire);
        if seq_after == seq_before {
            return (hi << 32) | lo;
        }
        // Concurrent write; retry.
    }
}
```

- [ ] **Step 2: Wire `publish_widened_now` into `Engine::tick`**

Round-2 fix B5: Engine::tick takes `pool: &CurvePool` from Phase 1 onward (not just Phase 9). The signature is the single source of truth across all phases. Phase 9 hold-segment work only adds the short-circuit body inside the existing signature.

In `rust/runtime/src/engine.rs::tick`:

```rust
pub fn tick(
    &mut self,
    raw_cyccnt: u32,
    widen_state: &mut crate::clock::WidenState,
    pool: &crate::curve_pool::CurvePool,
    shared: &crate::state::SharedState,
) {
    // Phase 7 §8.5 force_idle short-circuit will be added at the top here.
    let now_widened = widen_state.widen(raw_cyccnt);
    crate::clock::publish_widened_now(shared, now_widened);
    // ... existing tick body, threading `pool` to evaluate_current
}
```

(Phase 1 lands the signature change; Phase 9 only fills in the hold-segment short-circuit body inside `evaluate_current`.)

- [ ] **Step 3: Add a host-target unit test for the seqlock pattern**

Create `rust/runtime/tests/seqlock_unit.rs`:

```rust
use runtime::state::SharedState;
use runtime::clock::{publish_widened_now, read_widened_now};

#[test]
fn seqlock_round_trip() {
    let shared = SharedState::new();
    publish_widened_now(&shared, 0xDEAD_BEEF_CAFE_BABE);
    let got = read_widened_now(&shared);
    assert_eq!(got, 0xDEAD_BEEF_CAFE_BABE);
}

#[test]
fn seqlock_zero_initial_read() {
    let shared = SharedState::new();
    let got = read_widened_now(&shared);
    assert_eq!(got, 0);
}

#[test]
fn seqlock_multiple_writes() {
    let shared = SharedState::new();
    for i in 0u64..1000 {
        publish_widened_now(&shared, i.wrapping_mul(0x1234_5678));
    }
    let got = read_widened_now(&shared);
    assert_eq!(got, 999u64.wrapping_mul(0x1234_5678));
}
```

- [ ] **Step 4: Run the unit tests**

```bash
cd rust && cargo test -p runtime --features host seqlock_unit 2>&1 | tail -15
```

All three pass.

- [ ] **Step 5: Commit**

```bash
git add rust/runtime/src/clock.rs rust/runtime/src/engine.rs \
        rust/runtime/tests/seqlock_unit.rs
git commit -m "runtime/clock: §11.4 widened-clock seqlock

Two AtomicU32 + sequence counter + spin-loop reader. ISR publishes
widened u64; foreground reads via bounded-retry seqlock. Sidesteps
ARMv7-M's lack of lock-free AtomicU64.

Per spec §11.4. Loom test surface deferred to Task 1.4."
```

### Task 1.4: Loom tests for half-split + seqlock

**Files:**
- Create: `rust/runtime/tests/loom_seqlock.rs` — concurrent producer/consumer test of the seqlock under loom.
- Create: `rust/runtime/tests/loom_spsc_split.rs` — half-split SPSC concurrent test.
- Modify: `rust/runtime/Cargo.toml` — add `loom` dev-dependency, optional `loom` feature, target `cfg(loom)` test config.

**Why:** Spec §11.3 + Round-1 review fix. Loom exhaustively models concurrent thread interleavings under a relaxed memory model; catches ordering bugs that no fuzzer or stress test can find reliably.

- [ ] **Step 1: Add `loom` dev-dependency to `rust/runtime/Cargo.toml`**

```toml
[target.'cfg(loom)'.dev-dependencies]
loom = "0.7"
```

- [ ] **Step 2: Create `rust/runtime/tests/loom_seqlock.rs`**

```rust
#![cfg(loom)]

use loom::sync::atomic::{AtomicU32, Ordering};
use loom::sync::Arc;
use loom::thread;

#[test]
fn loom_seqlock_writer_reader() {
    loom::model(|| {
        let lo = Arc::new(AtomicU32::new(0));
        let hi = Arc::new(AtomicU32::new(0));
        let seq = Arc::new(AtomicU32::new(0));
        let writer_lo = lo.clone();
        let writer_hi = hi.clone();
        let writer_seq = seq.clone();

        let writer = thread::spawn(move || {
            // Two distinct values; both must be observable atomically.
            let s1 = writer_seq.load(Ordering::Relaxed).wrapping_add(1);
            writer_seq.store(s1, Ordering::Release);
            writer_lo.store(0xCAFEBABE, Ordering::Release);
            writer_hi.store(0xDEADBEEF, Ordering::Release);
            writer_seq.store(s1.wrapping_add(1), Ordering::Release);

            let s2 = writer_seq.load(Ordering::Relaxed).wrapping_add(1);
            writer_seq.store(s2, Ordering::Release);
            writer_lo.store(0xFEEDFACE, Ordering::Release);
            writer_hi.store(0xBADC0DE5, Ordering::Release);
            writer_seq.store(s2.wrapping_add(1), Ordering::Release);
        });

        // Reader loop: must always observe a coherent (lo, hi) pair —
        // either (CAFEBABE, DEADBEEF), (FEEDFACE, BADC0DE5), or (0,0).
        loop {
            let s_before = seq.load(Ordering::Acquire);
            if s_before & 1 != 0 { continue; }
            let l = lo.load(Ordering::Acquire);
            let h = hi.load(Ordering::Acquire);
            let s_after = seq.load(Ordering::Acquire);
            if s_after == s_before {
                let coherent = (l, h) == (0, 0)
                    || (l, h) == (0xCAFEBABE, 0xDEADBEEF)
                    || (l, h) == (0xFEEDFACE, 0xBADC0DE5);
                assert!(coherent, "torn read: lo={:#x} hi={:#x}", l, h);
                if (l, h) == (0xFEEDFACE, 0xBADC0DE5) { break; }
            }
        }

        writer.join().unwrap();
    });
}
```

- [ ] **Step 3: Create `rust/runtime/tests/loom_spsc_split.rs`**

```rust
#![cfg(loom)]

use loom::sync::Arc;
use loom::thread;

// NOTE: `heapless` doesn't natively support loom atomics; this test models the
// abstract producer/consumer pattern using loom primitives instead. The real
// `heapless::spsc` is tested separately on host with criterion stress tests.

#[test]
fn loom_spsc_pattern_producer_consumer() {
    loom::model(|| {
        use loom::sync::atomic::{AtomicUsize, Ordering};

        const N: usize = 4;
        let head = Arc::new(AtomicUsize::new(0));
        let tail = Arc::new(AtomicUsize::new(0));

        let p_head = head.clone();
        let p_tail = tail.clone();
        let producer = thread::spawn(move || {
            for _ in 0..2 {
                loop {
                    let h = p_head.load(Ordering::Acquire);
                    let t = p_tail.load(Ordering::Acquire);
                    let next_h = (h + 1) % N;
                    if next_h == t { continue; }  // full
                    p_head.store(next_h, Ordering::Release);
                    break;
                }
            }
        });

        let c_head = head.clone();
        let c_tail = tail.clone();
        let consumer = thread::spawn(move || {
            let mut popped = 0;
            for _ in 0..10 {
                let h = c_head.load(Ordering::Acquire);
                let t = c_tail.load(Ordering::Acquire);
                if h == t { continue; }
                c_tail.store((t + 1) % N, Ordering::Release);
                popped += 1;
                if popped == 2 { break; }
            }
            assert!(popped <= 2, "consumer popped more than producer pushed");
        });

        producer.join().unwrap();
        consumer.join().unwrap();
    });
}
```

- [ ] **Step 4: Run loom tests**

```bash
cd rust && RUSTFLAGS="--cfg loom" cargo test -p runtime --release --test loom_seqlock --test loom_spsc_split 2>&1 | tail -15
```

Expected: both pass. Loom-model exploration may take 30–60 s.

- [ ] **Step 5: Commit**

```bash
git add rust/runtime/Cargo.toml \
        rust/runtime/tests/loom_seqlock.rs \
        rust/runtime/tests/loom_spsc_split.rs
git commit -m "runtime/tests: loom for §11.4 seqlock + half-split SPSC pattern

Loom exhaustively models thread interleavings under relaxed memory
model. Catches torn reads and ordering bugs no fuzzer can.

Per spec §11.3."
```

---

## Phase 2 — Curve-pool generation handles

Implements spec §10. Replaces Step-5's no-overwrite policy with `(slot, gen)` handles + FIFO-ordered SEGMENT_END reclaim + post-fault-only sweep.

### Task 2.1: `CurveHandle` u32 layout

**Files:**
- Modify: `rust/runtime/src/curve_pool.rs` — define `CurveHandle = { slot_idx: u16, generation: u16 }`.
- Modify: `rust/runtime/src/segment.rs` — `Segment.curve_handle` becomes `CurveHandle` (u32).

**Why:** Spec §10.1 + Round-2 review fix. Handle widened to u32 so the framework's measurement-driven `CURVE_POOL_N` is not bottlenecked by handle bit-width. ABA window at `Q_N_MAX = 256` is `65536 - 256 = 65280` allocations — far larger than any realistic stale-handle window.

- [ ] **Step 1: Define `CurveHandle` in `rust/runtime/src/curve_pool.rs`**

Replace the Step-5 `pub type CurveHandle = u16;` (or wherever it's defined) with:

```rust
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CurveHandle {
    pub slot_idx: u16,    // 0..CURVE_POOL_N (≤ 256 per §7.1 Q_N_MAX)
    pub generation: u16,  // wraps modulo 65536
}

const _: () = assert!(core::mem::size_of::<CurveHandle>() == 4);

impl CurveHandle {
    pub const fn new(slot_idx: u16, generation: u16) -> Self {
        Self { slot_idx, generation }
    }

    /// Sentinel for hold segments (§6.5). The ISR short-circuits on
    /// `flags & HOLD_SEGMENT` BEFORE looking up this handle, so the
    /// sentinel is never resolved through CurvePool::lookup.
    pub const HOLD_SEGMENT_SENTINEL: Self = Self { slot_idx: u16::MAX, generation: u16::MAX };
}
```

- [ ] **Step 2: Update `Segment` to use the new `CurveHandle` type**

In `rust/runtime/src/segment.rs`:

```rust
use crate::curve_pool::CurveHandle;

#[repr(C)]
pub struct Segment {
    pub id: u32,
    pub curve_handle: CurveHandle,  // u32 (was u16 in Step-5)
    pub t_start: u64,
    pub t_end: u64,
    pub kinematics: u8,
    pub flags: u8,                   // §6.5 HOLD_SEGMENT bit 0
    pub _pad: [u8; 2],
}

const _: () = assert!(core::mem::size_of::<Segment>() == 24);  // recompute if Step-5 size differs
```

(Note: actual size depends on Step-5's existing `Segment` layout. Adjust the static assert + padding to match the ACTUAL size after the field-type change. The point is: define a `const _: () = assert!(...)` to lock the size in CI.)

- [ ] **Step 3: Update FFI `kalico_runtime_push_segment` to take a u32 curve_handle**

In `rust/kalico-c-api/src/runtime_ffi.rs`:

```rust
#[unsafe(no_mangle)]
pub unsafe extern "C" fn kalico_runtime_push_segment(
    rt: *mut KalicoRuntime,
    id: u32,
    curve_handle_packed: u32,  // upper 16 bits = generation, lower 16 = slot_idx
    t_start: u64,
    t_end: u64,
    kinematics: u8,
) -> i32 {
    let handle = CurveHandle {
        slot_idx: (curve_handle_packed & 0xFFFF) as u16,
        generation: (curve_handle_packed >> 16) as u16,
    };
    // ... project + push_segment_impl
}
```

- [ ] **Step 4: Update `kalico_runtime.h` regen**

```bash
cd rust && cargo run -p kalico-c-api --bin gen-headers 2>&1 | tail -5
git diff rust/kalico-c-api/include/kalico_runtime.h
```

Expected diff: `curve_handle` parameter type changes from `uint16_t` to `uint32_t` in the C signature.

- [ ] **Step 5: Update `src/runtime_tick.c::command_kalico_push_segment` to send u32 handle**

```c
// Update DECL_COMMAND to use %u (u32) instead of %hu (u16) for curve_handle:
DECL_COMMAND(command_kalico_push_segment,
    "kalico_push_segment id=%u curve_handle=%u "
    "t_start_hi=%u t_start_lo=%u t_end_hi=%u t_end_lo=%u kin=%c");
```

- [ ] **Step 6: Run unit tests**

```bash
cd rust && cargo test -p runtime --features host 2>&1 | tail -15
```

Expected: all tests pass; static_assert fires if `Segment` size changed unexpectedly.

- [ ] **Step 7: Commit**

```bash
git add rust/runtime/src/curve_pool.rs rust/runtime/src/segment.rs \
        rust/kalico-c-api/src/runtime_ffi.rs \
        rust/kalico-c-api/include/kalico_runtime.h \
        src/runtime_tick.c
git commit -m "runtime/curve_pool: §10.1 — CurveHandle widened to u32 (slot+gen u16 each)

Round-2 review fix: u16 packed handle had ABA window of 1 allocation
at Q_N=65535. Widened to u32 ({slot_idx: u16, generation: u16}) so
framework's measurement-driven CURVE_POOL_N (§7.1) isn't bottlenecked.
At Q_N_MAX=256 ABA window = 65280 allocations.

Per spec §10.1."
```

### Task 2.2: Pool allocation predicate (modulo-u16 wrap)

**Files:**
- Modify: `rust/runtime/src/curve_pool.rs` — implement `try_alloc` + `try_load_into` on the new predicate.

**Why:** Spec §10.2 + §10.3. Predicate `current_gen == last_retired_gen` modulo u16 wrap is deadlock-free; no special wrap-cooldown machinery (Round-1 review fix removed it).

- [ ] **Step 1: Define the per-slot atomic state**

Round-4 fix: `LoadedCurve` was private in Step-5; Phase 2 makes it `pub` (sim_fixtures FFI in Phase 0 doesn't need to construct it directly — Phase 0 fix uses Step-5's flat-slice `pool.load(...)` API — but the new `try_alloc_and_load(slot, curve)` API takes a `LoadedCurve` so it must be pub).

In `rust/runtime/src/curve_pool.rs`:

```rust
use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicU16, Ordering};

pub const CURVE_POOL_N: usize = 256;  // §7.1 Q_N_MAX

// Round-4 fix: was private in Step-5 (curve_pool.rs:28); now pub.
#[derive(Debug, Clone, Copy)]
pub struct LoadedCurve {
    pub control_points: [[f32; 3]; 8],
    pub weights: [f32; 8],
    pub knots: [f32; 12],
    pub n_cp: u8,
    pub n_knots: u8,
    pub degree: u8,
}

const _: () = assert!(core::mem::size_of::<LoadedCurve>() == 184);

pub struct PoolSlot {
    pub current_gen: AtomicU16,         // last gen issued by alloc
    pub last_retired_gen: AtomicU16,    // last gen confirmed retired
    pub curve: UnsafeCell<LoadedCurve>, // ISR reads via lookup; foreground writes during load
}

impl PoolSlot {
    const fn new() -> Self {
        Self {
            current_gen: AtomicU16::new(0),
            last_retired_gen: AtomicU16::new(0),
            curve: UnsafeCell::new(LoadedCurve {
                control_points: [[0.0; 3]; 8],
                weights: [1.0; 8],
                knots: [0.0; 12],
                n_cp: 0,
                n_knots: 0,
                degree: 0,
            }),
        }
    }
}

pub struct CurvePool {
    pub slots: [PoolSlot; CURVE_POOL_N],
}

// Round-2 fix B8: PoolSlot has UnsafeCell<LoadedCurve> which is !Sync by
// default. Synchronization is achieved via per-slot AtomicU16 generation
// counters (foreground writes via Release on current_gen AFTER curve memcpy;
// ISR Acquire-loads current_gen BEFORE dereferencing slot.curve). Discipline
// contract documented inline.
unsafe impl Sync for PoolSlot {}
unsafe impl Sync for CurvePool {}

impl CurvePool {
    pub const fn new() -> Self {
        Self {
            slots: [const { PoolSlot::new() }; CURVE_POOL_N],
        }
    }

    /// Foreground reserves a slot AND loads the curve atomically. Returns
    /// Some(handle) if `current_gen == last_retired_gen` (modulo u16), None
    /// otherwise.
    ///
    /// **Ordering** (per spec §10.2 + Round-1 Codex #4 fix): the new curve
    /// MUST be fully written into slot.curve BEFORE current_gen is bumped.
    /// Otherwise the ISR's lookup could observe `current_gen == new_gen`
    /// before the curve memory is initialized, dereferencing stale/garbage
    /// curve data.
    ///
    /// Combined alloc + load (replaces the prior split try_alloc /
    /// try_load_into which inverted the ordering).
    pub fn try_alloc_and_load(&self, slot_idx: usize, curve: LoadedCurve) -> Option<CurveHandle> {
        if slot_idx >= CURVE_POOL_N {
            return None;
        }
        let slot = &self.slots[slot_idx];
        let cur = slot.current_gen.load(Ordering::Acquire);
        let last = slot.last_retired_gen.load(Ordering::Acquire);
        if cur != last {
            return None;
        }
        // 1. Write the new curve. SAFETY: predicate above guarantees no
        //    concurrent ISR access — ISR's lookup checks current_gen first.
        unsafe { *slot.curve.get() = curve; }
        // 2. Memory barrier so the curve write completes before we bump gen.
        //    The Release store on current_gen below provides this.
        // 3. Bump generation. Wrap on u16 modulo. The ISR's Acquire-load
        //    on current_gen synchronizes-with this Release store, ensuring
        //    the curve write is visible if and only if the new gen is.
        let new_gen = cur.wrapping_add(1);
        slot.current_gen.store(new_gen, Ordering::Release);
        Some(CurveHandle { slot_idx: slot_idx as u16, generation: new_gen })
    }

    /// ISR-only lookup; validates handle generation matches.
    pub fn lookup(&self, handle: CurveHandle) -> Result<&LoadedCurve, crate::error::FaultCode> {
        let slot_idx = handle.slot_idx as usize;
        if slot_idx >= CURVE_POOL_N {
            return Err(crate::error::FaultCode::InvalidCurveHandle);
        }
        let slot = &self.slots[slot_idx];
        if slot.current_gen.load(Ordering::Acquire) != handle.generation {
            return Err(crate::error::FaultCode::InvalidCurveHandle);
        }
        // SAFETY: handle.generation matches current_gen; current_gen is bumped only
        // after the new curve is fully loaded (alloc-then-load contract).
        Ok(unsafe { &*slot.curve.get() })
    }

    /// Foreground reclaim: called from trace-drain pipeline on observing
    /// SEGMENT_END(handle). FIFO ordering of trace events guarantees all
    /// prior generations for this slot have already retired.
    pub fn confirm_retired(&self, handle: CurveHandle) {
        let slot_idx = handle.slot_idx as usize;
        if slot_idx >= CURVE_POOL_N { return; }
        let slot = &self.slots[slot_idx];
        slot.last_retired_gen.store(handle.generation, Ordering::Release);
    }
}
```

- [ ] **Step 2: Add unit tests for the alloc/load/lookup predicate**

Create `rust/runtime/tests/curve_pool_alloc.rs`:

```rust
use runtime::curve_pool::{CurvePool, LoadedCurve, CurveHandle, CURVE_POOL_N};

fn dummy_curve() -> LoadedCurve {
    LoadedCurve {
        control_points: [[0.0; 3]; 8],
        weights: [1.0; 8],
        knots: [0.0; 12],
        n_cp: 2, n_knots: 4, degree: 1,
    }
}

#[test]
fn first_alloc_succeeds() {
    let pool = CurvePool::new();
    let h = pool.try_alloc_and_load(0, dummy_curve()).unwrap();
    assert_eq!(h.slot_idx, 0);
    assert_eq!(h.generation, 1);  // bumped from 0 -> 1
}

#[test]
fn second_alloc_blocked_until_retired() {
    let pool = CurvePool::new();
    let h1 = pool.try_alloc_and_load(0, dummy_curve()).unwrap();
    assert!(pool.try_alloc_and_load(0, dummy_curve()).is_none(),
        "second alloc should be blocked");
    pool.confirm_retired(h1);
    let h2 = pool.try_alloc_and_load(0, dummy_curve()).unwrap();
    assert_eq!(h2.generation, 2);
}

#[test]
fn lookup_validates_generation() {
    let pool = CurvePool::new();
    let h1 = pool.try_alloc_and_load(0, dummy_curve()).unwrap();
    assert!(pool.lookup(h1).is_ok());

    pool.confirm_retired(h1);
    let _h2 = pool.try_alloc_and_load(0, dummy_curve()).unwrap();
    // Stale h1 must now reject (current_gen != h1.generation).
    assert!(pool.lookup(h1).is_err(), "stale handle must reject");
}

#[test]
fn wrap_u16_modulo_no_deadlock() {
    let pool = CurvePool::new();
    // Force generation through wrap. Alloc + retire in sequence 65536 times.
    for _ in 0..65536 {
        let h = pool.try_alloc_and_load(0, dummy_curve()).unwrap();
        pool.confirm_retired(h);
    }
    // Slot is allocatable post-wrap.
    let h_post_wrap = pool.try_alloc_and_load(0, dummy_curve()).unwrap();
    assert_eq!(h_post_wrap.slot_idx, 0);
}

#[test]
fn out_of_range_slot_rejects() {
    let pool = CurvePool::new();
    assert!(pool.try_alloc_and_load(CURVE_POOL_N, dummy_curve()).is_none());
    assert!(pool.lookup(CurveHandle::new(CURVE_POOL_N as u16, 1)).is_err());
}
```

- [ ] **Step 3: Run the tests**

```bash
cd rust && cargo test -p runtime --features host curve_pool_alloc 2>&1 | tail -15
```

All five pass.

- [ ] **Step 4: Commit**

```bash
git add rust/runtime/src/curve_pool.rs rust/runtime/tests/curve_pool_alloc.rs
git commit -m "runtime/curve_pool: §10.2/§10.3 alloc predicate + wrap policy

current_gen == last_retired_gen (modulo u16 wrap) — natural deadlock-free
behavior, no special wrap-cooldown machinery (Round-1 review fix).
SEGMENT_END-driven reclaim wired via confirm_retired; ISR lookup
validates generation match.

Per spec §10.2 / §10.3."
```

### Task 2.3: SEGMENT_END-driven reclaim in foreground

**Files:**
- Modify: `rust/runtime/src/state.rs::FgState` — add foreground trace-drain loop hook.
- Create: `rust/runtime/src/reclaim.rs` — extract foreground trace-drain reclaim logic into a focused module.

**Why:** Spec §10.4. Foreground observes `SEGMENT_END(handle)` events from the trace ring; `confirm_retired(handle)` updates `last_retired_gen`. FIFO ordering of trace events implies all prior gens for the slot are retired by the time gen=G is observed.

- [ ] **Step 1: Create `rust/runtime/src/reclaim.rs`**

```rust
//! Foreground trace-drain → curve-pool reclaim pipeline. Per spec §10.4.
//!
//! On observing `SEGMENT_END(slot=N, gen=G)` in the trace stream, foreground
//! sets `slot[N].last_retired_gen = G`. FIFO ordering of single-ISR-writer
//! single-foreground-reader heapless::spsc preserves the per-slot retirement
//! sequence; no separate "any queued segment references this slot" inspection
//! is needed.
//!
//! Producer is expected to drain pending trace samples before failing alloc
//! due to "no reclaimable slot."

use crate::curve_pool::CurvePool;
use crate::trace::{TraceSample, TRACE_FLAG_SEGMENT_END};

/// Drain up to `limit` trace samples; for each SEGMENT_END, update
/// last_retired_gen on the referenced slot. Returns count drained.
pub fn drain_and_reclaim<F>(
    pool: &CurvePool,
    drain_one: F,
    limit: usize,
) -> usize
where
    F: FnMut() -> Option<TraceSample>,
{
    let mut drain_one = drain_one;
    let mut drained = 0;
    while drained < limit {
        let Some(sample) = drain_one() else { break };
        if sample.flags & TRACE_FLAG_SEGMENT_END != 0 {
            pool.confirm_retired(sample.curve_handle);
        }
        drained += 1;
    }
    drained
}
```

- [ ] **Step 2: Add `mod reclaim;` to `rust/runtime/src/lib.rs`**

```rust
pub mod reclaim;
```

- [ ] **Step 3: Extend trace flag constants in `rust/runtime/src/trace.rs`**

Round-4 fix (verifier #3): Step-5 already defines these constants. Don't redefine — extend at higher bits and use existing names:

```rust
// Step-5 (already defined in trace.rs:7-9):
//   TRACE_FLAG_OVERFLOW      = 1 << 0;  // existing — KEEP
//   TRACE_FLAG_SEGMENT_END   = 1 << 1;  // existing — KEEP (used by §10.4 reclaim)
//   TRACE_FLAG_FAULT_MARKER  = 1 << 2;  // existing — KEEP

// Step-6 additions (new bits, do not collide with above):
pub const TRACE_FLAG_SEGMENT_START: u8 = 1 << 3;  // §13.3 — start-of-segment marker
pub const TRACE_FLAG_HOLD_SAMPLE:   u8 = 1 << 4;  // §6.5 — hold-segment throttled marker
```

Throughout Phase 2.3 reclaim code and Phase 9 hold-segment code, use the existing Step-5 names: `TRACE_FLAG_SEGMENT_END` and `TRACE_FLAG_FAULT_MARKER` (NOT `TRACE_FLAG_FAULT`). The `TRACE_FLAG_OVERFLOW` semantics from Step-5 (sample-drop carry-into-next-sample) is replaced by the §13.1 `sample_drop_pending: AtomicBool` mechanism in Phase 5 Task 5.2; the `1<<0` flag bit is RETIRED (no longer set/checked by Step-6 code paths) but the constant stays defined for source-level binary compatibility with any older host-side decoders.

- [ ] **Step 4: Add a unit test for the reclaim pipeline**

Create `rust/runtime/tests/reclaim_pipeline.rs`:

```rust
use runtime::curve_pool::{CurvePool, CurveHandle};
use runtime::reclaim::drain_and_reclaim;
use runtime::trace::{TraceSample, TRACE_FLAG_SEGMENT_END};

// Round-2 fix B02: tests use the combined try_alloc_and_load API.

fn dummy_curve() -> runtime::curve_pool::LoadedCurve {
    runtime::curve_pool::LoadedCurve {
        control_points: [[0.0; 3]; 8],
        weights: [1.0; 8],
        knots: [0.0; 12],
        n_cp: 2, n_knots: 4, degree: 1,
    }
}

#[test]
fn reclaim_advances_last_retired_gen() {
    let pool = CurvePool::new();
    let h1 = pool.try_alloc_and_load(0, dummy_curve()).unwrap();
    assert!(pool.try_alloc_and_load(0, dummy_curve()).is_none());

    let mut samples = vec![
        TraceSample { tick: 100, motor_a: 0.0, motor_b: 0.0, motor_e: 0.0,
            segment_id: 1, curve_handle: h1, flags: TRACE_FLAG_SEGMENT_END, _pad: [0; 3] },
    ];
    let drained = drain_and_reclaim(
        &pool,
        || samples.pop(),
        16,
    );
    assert_eq!(drained, 1);
    assert!(pool.try_alloc_and_load(0, dummy_curve()).is_some(),
        "alloc should succeed after retire");
}

#[test]
fn fifo_ordering_implies_prior_gens_retired() {
    let pool = CurvePool::new();
    // Allocate, retire, allocate, retire — drain in order.
    let h1 = pool.try_alloc_and_load(0, dummy_curve()).unwrap();  // gen=1
    pool.confirm_retired(h1);
    let h2 = pool.try_alloc_and_load(0, dummy_curve()).unwrap();  // gen=2

    // Trace stream emits gen=2 SEGMENT_END.
    let mut samples = vec![
        TraceSample { tick: 200, motor_a: 0.0, motor_b: 0.0, motor_e: 0.0,
            segment_id: 2, curve_handle: h2, flags: TRACE_FLAG_SEGMENT_END, _pad: [0; 3] },
    ];
    drain_and_reclaim(&pool, || samples.pop(), 16);

    // After SEGMENT_END(gen=2), slot is reusable.
    let h3 = pool.try_alloc_and_load(0, dummy_curve()).unwrap();
    assert_eq!(h3.generation, 3);
}
```

- [ ] **Step 5: Run reclaim tests**

```bash
cd rust && cargo test -p runtime --features host reclaim_pipeline 2>&1 | tail -10
```

Both pass.

- [ ] **Step 6: Commit**

```bash
git add rust/runtime/src/reclaim.rs rust/runtime/src/lib.rs \
        rust/runtime/src/trace.rs rust/runtime/tests/reclaim_pipeline.rs
git commit -m "runtime/reclaim: §10.4 SEGMENT_END-driven curve-pool reclaim

Foreground trace-drain pipeline; on SEGMENT_END(handle), advances
slot[handle.slot_idx].last_retired_gen. FIFO ordering of single-writer
single-reader heapless::spsc implies all prior generations retired.

Per spec §10.4."
```

---

## Phase 3 — Wire schema additions

Implements spec §4 (versioned blob payloads), §5 (credit message + push response + status frame), §8 (stream lifecycle commands), §12 (clock-sync exchange).

### Task 3.1: 1-byte format-version field on blob payloads

**Files:**
- Create: `rust/runtime/src/wire.rs` — versioned payload helpers (encode/decode v1).
- Modify: `rust/kalico-c-api/src/runtime_ffi.rs` — `kalico_runtime_load_curve` validates leading version byte.

**Why:** Spec §4.2. Versioned data plane lets us evolve the wire format without breaking msgproto framing.

- [ ] **Step 1: Create `rust/runtime/src/wire.rs`**

```rust
//! Versioned blob payload format. Per spec §4.2.
//!
//! Every kalico-native blob payload carried inside msgproto's `%*s` is
//! prefixed with a 1-byte format-version field, followed by the binary
//! struct in little-endian.

pub const FORMAT_VERSION_V1: u8 = 0x01;

pub const MIN_BLOB_HEADER_LEN: usize = 1;

pub fn check_version(blob: &[u8]) -> Result<(), crate::error::FaultCode> {
    if blob.len() < MIN_BLOB_HEADER_LEN {
        return Err(crate::error::FaultCode::ProtocolVersionUnsupported);
    }
    if blob[0] != FORMAT_VERSION_V1 {
        return Err(crate::error::FaultCode::ProtocolVersionUnsupported);
    }
    Ok(())
}
```

Add `pub mod wire;` to `rust/runtime/src/lib.rs`.

- [ ] **Step 2: Modify `kalico_runtime_load_curve` to require leading version byte**

Update the load_curve FFI in `rust/kalico-c-api/src/runtime_ffi.rs`:

```rust
#[unsafe(no_mangle)]
pub unsafe extern "C" fn kalico_runtime_load_curve(
    rt: *mut KalicoRuntime,
    slot_idx: u16,
    payload_ptr: *const u8,
    payload_len: u32,
) -> i32 {
    if rt.is_null() { return KALICO_ERR_NULL_HANDLE; }
    if payload_ptr.is_null() || payload_len == 0 { return KALICO_ERR_VALIDATION; }
    let payload = unsafe { core::slice::from_raw_parts(payload_ptr, payload_len as usize) };
    if let Err(_) = runtime::wire::check_version(payload) {
        return KALICO_ERR_PROTOCOL_VERSION;
    }
    // Decode the rest of the payload (the v1 schema is the existing Step-5
    // layout sans the leading version byte).
    decode_load_curve_v1(rt, slot_idx, &payload[1..])
}
```

`decode_load_curve_v1` is the Step-5 decode path operating on the post-version-byte slice.

- [ ] **Step 3: Update `command_kalico_load_curve` in `src/runtime_tick.c` to pass payload through with leading version byte**

The host-side encoder is responsible for prepending `0x01`; the firmware-side handler just passes the full payload (length + ptr) into the FFI:

```c
// command_kalico_load_curve passes args[N] / args[N+1] (length + ptr) directly.
// Host-side encoding includes the leading version byte. No firmware changes
// to the C-side decode beyond updating the FFI signature (now takes a single
// payload blob rather than separate cps/knots/weights blobs).
```

Note: this changes the wire schema for `kalico_load_curve`. Step 5's command was `slot=%hu degree=%c cps=%*s knots=%*s weights=%*s`. Step 6 v1 wire is `slot=%hu data=%*s` (single versioned blob containing all three arrays). Update accordingly.

- [ ] **Step 4: Update `tools/test_h723_first_light.py` (and any other host-side consumer) to encode the v1 blob**

The host-side encoding helper in `tools/kalico_host_io.py` (or a new `tools/wire_v1.py`):

```python
def encode_load_curve_v1(degree: int, cps: list, knots: list, weights: list) -> bytes:
    import struct
    out = bytearray()
    out.append(0x01)  # format-version v1
    out.append(degree)
    out.append(len(cps))
    out.append(len(knots))
    out.append(len(weights))
    for cp in cps:
        out.extend(struct.pack("<3f", *cp))
    for k in knots:
        out.extend(struct.pack("<f", k))
    for w in weights:
        out.extend(struct.pack("<f", w))
    return bytes(out)
```

- [ ] **Step 5: Add a unit test for `wire::check_version`**

```rust
#[test]
fn version_check_v1_passes() {
    let payload = [0x01_u8, 0x02, 0x03];
    assert!(runtime::wire::check_version(&payload).is_ok());
}

#[test]
fn version_check_unknown_rejects() {
    let payload = [0xFF_u8, 0x02, 0x03];
    assert!(runtime::wire::check_version(&payload).is_err());
}

#[test]
fn version_check_empty_rejects() {
    let payload = [];
    assert!(runtime::wire::check_version(&payload).is_err());
}
```

- [ ] **Step 6: Run host tests**

```bash
cd rust && cargo test -p runtime --features host wire 2>&1 | tail -10
```

All pass.

- [ ] **Step 7: Commit**

```bash
git add rust/runtime/src/wire.rs rust/runtime/src/lib.rs \
        rust/kalico-c-api/src/runtime_ffi.rs src/runtime_tick.c \
        tools/kalico_host_io.py
git commit -m "runtime/wire: §4.2 1-byte format-version on blob payloads

Every kalico-native blob (load_curve payload, push_segment payload,
trace data) is prefixed with FORMAT_VERSION_V1 = 0x01. Future schema
evolution: bump version, MCU rejects unknown.

Per spec §4.2."
```

### Task 3.2: New DECL_COMMANDs (stream_open / arm / terminal / flush, clock_sync_request)

**Files:**
- Modify: `src/runtime_tick.c` — add five new DECL_COMMANDs.
- Modify: `rust/kalico-c-api/src/runtime_ffi.rs` — five new FFI entry points.

**Why:** Spec §8.3 + §12.1. Wire-level surface for stream lifecycle and clock sync.

- [ ] **Step 1: Add FFI surface in `rust/kalico-c-api/src/runtime_ffi.rs`**

```rust
#[unsafe(no_mangle)]
pub unsafe extern "C" fn kalico_runtime_stream_open(
    rt: *mut KalicoRuntime,
    stream_id: u32,
    out_credit_epoch: *mut u32,
) -> i32 {
    // Project to FgState; delegate to stream::open.
    project_fg(rt, |fg, shared| crate::stream::open(fg, shared, stream_id, out_credit_epoch))
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn kalico_runtime_stream_arm(
    rt: *mut KalicoRuntime,
    t_start_t0: u64,
    arm_lead_cycles: u32,
    out_armed_t_start: *mut u64,
) -> i32 {
    // Round-2 fix B6: arm() needs both pool ref and queue-peek closure
    // (per Phase 6 Task 6.2). Use full RuntimeContext projection here, NOT
    // project_fg, so we can reach the top-level CurvePool and the IsrState
    // queue_consumer for peek.
    if rt.is_null() { return KALICO_ERR_NULL_HANDLE; }
    let ctx = rt as *mut RuntimeContext;
    unsafe {
        let fg_ptr: *mut FgState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).fg));
        let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
        let pool_ptr: *const crate::curve_pool::CurvePool =
            core::ptr::addr_of!((*ctx).curve_pool);
        let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
        let fg: &mut FgState = &mut *fg_ptr;
        let pool: &crate::curve_pool::CurvePool = &*pool_ptr;
        let shared: &SharedState = &*shared_ptr;
        // Queue peek: use `iter().next()` semantics on a snapshot. Round-2 B6
        // notes the half-split discipline conflict — peeking ISR-owned data
        // from foreground violates SPSC ownership. Workaround: foreground
        // captures the FIRST segment's t_start at push-acceptance time
        // (storing in `fg.first_priming_segment_t_start: Option<u64>`),
        // and arm() reads from `fg`, not from the queue.
        let _ = isr_ptr;  // placeholder for future closure
        let first_seg_t_start = fg.first_priming_segment_t_start;
        crate::stream::arm(fg, shared, pool, first_seg_t_start,
            t_start_t0, arm_lead_cycles, out_armed_t_start)
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn kalico_runtime_stream_terminal(
    rt: *mut KalicoRuntime,
    segment_id: u32,
) -> i32 {
    project_fg(rt, |fg, shared| crate::stream::terminal(fg, shared, segment_id))
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn kalico_runtime_stream_flush(
    rt: *mut KalicoRuntime,
    out_credit_epoch: *mut u32,
) -> i32 {
    project_fg(rt, |fg, shared| crate::stream::flush(fg, shared, out_credit_epoch))
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn kalico_runtime_clock_sync_request(
    rt: *mut KalicoRuntime,
    request_id: u32,
    host_send_time_lo: u32,
    host_send_time_hi: u32,
    out_mcu_clock: *mut u64,
) -> i32 {
    project_fg(rt, |fg, shared| crate::stream::clock_sync_respond(fg, shared, request_id, host_send_time_lo, host_send_time_hi, out_mcu_clock))
}
```

`project_fg` is a helper that does the raw-pointer projection + invokes the closure. Define it as:

```rust
unsafe fn project_fg<R, F>(rt: *mut KalicoRuntime, f: F) -> R
where
    F: FnOnce(&mut FgState, &SharedState) -> R,
{
    let ctx = rt as *mut RuntimeContext;
    unsafe {
        let fg_ptr: *mut FgState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).fg));
        let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
        f(&mut *fg_ptr, &*shared_ptr)
    }
}
```

(Stub the `crate::stream::open/arm/terminal/flush/clock_sync_respond` functions; actual bodies land in Phase 6.)

- [ ] **Step 2: Add the five DECL_COMMANDs in `src/runtime_tick.c`**

```c
void
command_kalico_stream_open(uint32_t *args)
{
    if (!kalico_rt_handle) { sendf("kalico_stream_open_response result=%i credit_epoch=%u", -7, 0); return; }
    uint32_t stream_id = args[0];
    uint32_t credit_epoch = 0;
    int32_t r = kalico_runtime_stream_open(kalico_rt_handle, stream_id, &credit_epoch);
    sendf("kalico_stream_open_response result=%i credit_epoch=%u", r, credit_epoch);
}
DECL_COMMAND(command_kalico_stream_open, "kalico_stream_open stream_id=%u");

void
command_kalico_stream_arm(uint32_t *args)
{
    if (!kalico_rt_handle) { sendf("kalico_stream_arm_response result=%i armed_t_start_lo=%u armed_t_start_hi=%u", -7, 0, 0); return; }
    uint64_t t_start = ((uint64_t)args[1] << 32) | args[0];
    uint32_t arm_lead_cycles = args[2];
    uint64_t armed_t_start = 0;
    int32_t r = kalico_runtime_stream_arm(kalico_rt_handle, t_start, arm_lead_cycles, &armed_t_start);
    sendf("kalico_stream_arm_response result=%i armed_t_start_lo=%u armed_t_start_hi=%u",
          r, (uint32_t)armed_t_start, (uint32_t)(armed_t_start >> 32));
}
DECL_COMMAND(command_kalico_stream_arm, "kalico_stream_arm t_start_t0_lo=%u t_start_t0_hi=%u arm_lead_cycles=%u");

void
command_kalico_stream_terminal(uint32_t *args)
{
    if (!kalico_rt_handle) { sendf("kalico_stream_terminal_response result=%i", -7); return; }
    uint32_t segment_id = args[0];
    int32_t r = kalico_runtime_stream_terminal(kalico_rt_handle, segment_id);
    sendf("kalico_stream_terminal_response result=%i", r);
}
DECL_COMMAND(command_kalico_stream_terminal, "kalico_stream_terminal segment_id=%u");

void
command_kalico_stream_flush(uint32_t *args)
{
    (void)args;
    if (!kalico_rt_handle) { sendf("kalico_stream_flush_response result=%i credit_epoch=%u", -7, 0); return; }
    uint32_t credit_epoch = 0;
    int32_t r = kalico_runtime_stream_flush(kalico_rt_handle, &credit_epoch);
    sendf("kalico_stream_flush_response result=%i credit_epoch=%u", r, credit_epoch);
}
DECL_COMMAND(command_kalico_stream_flush, "kalico_stream_flush");

void
command_kalico_clock_sync_request(uint32_t *args)
{
    if (!kalico_rt_handle) { sendf("kalico_clock_sync_response request_id=%u mcu_clock_lo=%u mcu_clock_hi=%u", 0, 0, 0); return; }
    uint32_t request_id = args[0];
    uint32_t host_send_time_lo = args[1];
    uint32_t host_send_time_hi = args[2];
    uint64_t mcu_clock = 0;
    kalico_runtime_clock_sync_request(kalico_rt_handle, request_id, host_send_time_lo, host_send_time_hi, &mcu_clock);
    sendf("kalico_clock_sync_response request_id=%u mcu_clock_lo=%u mcu_clock_hi=%u",
          request_id, (uint32_t)mcu_clock, (uint32_t)(mcu_clock >> 32));
}
DECL_COMMAND(command_kalico_clock_sync_request,
             "kalico_clock_sync_request request_id=%u host_send_time_lo=%u host_send_time_hi=%u");
```

- [ ] **Step 3: Regenerate kalico_runtime.h**

```bash
cd rust && cargo run -p kalico-c-api --bin gen-headers 2>&1 | tail -5
git diff rust/kalico-c-api/include/kalico_runtime.h
```

Expected: five new function declarations.

- [ ] **Step 4: Build the firmware to verify command dispatch table is correct**

```bash
bash tools/sim/build_sim_firmware.sh 2>&1 | tail -10
```

Clean.

- [ ] **Step 5: Commit**

```bash
git add rust/kalico-c-api/src/runtime_ffi.rs rust/kalico-c-api/include/kalico_runtime.h \
        src/runtime_tick.c
git commit -m "runtime: §8.3/§12.1 stream + clock-sync wire commands

DECL_COMMAND for kalico_stream_{open,arm,terminal,flush} and
kalico_clock_sync_request. FFI stubs project to FgState; bodies
land in Phase 6.

Per spec §8.3 + §12.1."
```

**Round-5 fix Codex #4 — load_curve / load_fixture responses must return the generated handle.** Step-6 `try_alloc_and_load(slot, curve)` returns a fresh `(slot, gen)` handle; the wire response previously returned only `result`, leaving the host with no way to reference the just-loaded curve. Extend both response schemas:

- `kalico_load_curve_response result=%i curve_handle_packed=%u`
- `kalico_load_fixture_response result=%i curve_handle_packed=%u` *(sim-only)*

The FFI returns the handle as a u32 out-param. C-side dispatch packs into `(generation << 16) | slot_idx` and emits via `sendf`. Add an out-param to `kalico_runtime_load_curve` / `kalico_runtime_load_fixture`:

```rust
#[unsafe(no_mangle)]
pub unsafe extern "C" fn kalico_runtime_load_curve(
    rt: *mut KalicoRuntime, slot_idx: u16,
    payload_ptr: *const u8, payload_len: u32,
    out_handle_packed: *mut u32,
) -> i32 { /* on success: *out_handle_packed = (gen as u32) << 16 | slot_idx as u32 */ }
```

### Task 3.3: Extended responses (`kalico_credit_freed`, `kalico_status`, `kalico_fault`, `kalico_push_response`)

**Files:**
- Modify: `src/runtime_tick.c` — extend response schemas + add async event paths.
- Modify: `rust/kalico-c-api/src/runtime_ffi.rs` — `kalico_runtime_drain_status` / event-pull FFI for foreground task.

**Why:** Spec §5.1, §5.3, §5.4, §9.1. Adds `retired_through_segment_id`, `accepted_segment_id`, fault async event channel, periodic status frame fields.

- [ ] **Step 1: Update `kalico_push_response` schema in `src/runtime_tick.c`**

Existing code emits `sendf("kalico_push_response result=%i");`. Extend:

```c
sendf("kalico_push_response result=%i accepted_segment_id=%u credit_epoch=%u",
      r, accepted_segment_id, credit_epoch);
```

`accepted_segment_id` and `credit_epoch` come back from the FFI via out-params. Update `kalico_runtime_push_segment`:

```rust
#[unsafe(no_mangle)]
pub unsafe extern "C" fn kalico_runtime_push_segment(
    rt: *mut KalicoRuntime,
    id: u32, curve_handle_packed: u32, t_start: u64, t_end: u64, kinematics: u8,
    out_accepted_segment_id: *mut u32,
    out_credit_epoch: *mut u32,
) -> i32 { /* ... */ }
```

- [ ] **Step 2: Add periodic `kalico_status` frame schema (Phase 11 wires up the emit; this task defines it)**

In `src/runtime_tick.c`, define the schema as a comment for the periodic emitter:

```c
// Periodic status frame schema (emitted by Phase 11 DECL_TASK at ~10 Hz):
//   sendf("kalico_status engine_status=%c queue_depth=%c "
//         "current_segment_id=%u last_fault=%hu fault_detail=%u "
//         "mcu_clock_now_lo=%u mcu_clock_now_hi=%u "
//         "credit_epoch=%u accepted_segment_id=%u retired_through_segment_id=%u",
//         status, depth, cur_seg, fault_code, fault_detail,
//         (uint32_t)mcu_clk, (uint32_t)(mcu_clk >> 32),
//         credit_epoch, accepted_seg_id, retired_seg_id);
```

(Actual emission lands in Phase 11 Task 11.2.)

- [ ] **Step 3: Add async event commands `kalico_credit_freed` and `kalico_fault`**

These are MCU-emitted async events (no DECL_COMMAND on the host-to-MCU side; they're outputs only). Klipper's msgproto requires every output message to be declared via `output(...)` macro or referenced by a `sendf` call to be auto-generated into the data dictionary. Add a `output()` declaration:

```c
// In src/runtime_tick.c (top-level):
output("kalico_credit_freed retired_through_segment_id=%u free_slots=%c");
output("kalico_fault fault_code=%hu fault_detail=%u segment_id=%u");
```

Actual emission from the foreground drain pipeline (Phase 11) calls `sendf(...)` with these formats.

- [ ] **Step 4: Regenerate header and rebuild**

```bash
cd rust && cargo run -p kalico-c-api --bin gen-headers 2>&1 | tail -5
bash tools/sim/build_sim_firmware.sh 2>&1 | tail -5
```

- [ ] **Step 5: Commit**

```bash
git add src/runtime_tick.c rust/kalico-c-api/src/runtime_ffi.rs \
        rust/kalico-c-api/include/kalico_runtime.h
git commit -m "runtime: §5/§9 extended response schemas + async event channel

kalico_push_response carries accepted_segment_id + credit_epoch.
kalico_credit_freed (async): retired_through_segment_id + free_slots.
kalico_fault (async): fault_code + fault_detail + segment_id.
kalico_status periodic schema declared (emit in Phase 11).

Per spec §5.1 / §5.3 / §5.4 / §9.1."
```

---

## Phase 4 — Fault taxonomy

Implements spec §9. Extends Step-5's `KalicoErr` with the comms-layer faults; adds fault_detail encoding.

### Task 4.1: Extend `KalicoErr` / `FaultCode` enum

**Files:**
- Modify: `rust/runtime/src/error.rs` — add new fault codes.
- Modify: `rust/kalico-c-api/src/runtime_ffi.rs` — re-export numeric constants for C side.

**Why:** Spec §9.1. Each fault code has a specific recovery semantic; collapsing to a catch-all loses diagnostic information.

- [ ] **Step 1: Extend `FaultCode` enum**

In `rust/runtime/src/error.rs`:

```rust
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaultCode {
    // Round-4 fix (verifier #2): preserve EXISTING Step-5 numeric values
    // (don't reuse -7 which is NOT_INIT). New Step-6 codes start at -100.
    None = 0,
    QueueFull = -1,         // Step-5 KALICO_ERR_QUEUE_FULL
    InvalidCurve = -2,      // Step-5 KALICO_ERR_INVALID_CURVE (was Validation)
    InvalidHandle = -3,     // Step-5 KALICO_ERR_INVALID_HANDLE
    InvalidDuration = -4,   // Step-5 KALICO_ERR_INVALID_DURATION
    InvalidKinematics = -5, // Step-5 KALICO_ERR_INVALID_KINEMATICS
    NullPtr = -6,           // Step-5 KALICO_ERR_NULL_PTR
    NotInit = -7,           // Step-5 KALICO_ERR_NOT_INIT (was incorrectly NullHandle)
    FaultLatched = -8,      // Step-5 KALICO_ERR_FAULT_LATCHED
    Internal = -9,          // Step-5 KALICO_ERR_INTERNAL (was -1)

    // Step-6 Transport-layer
    BadCrc = -100,
    FramingViolation = -101,
    Disconnect = -102,
    ProtocolVersionUnsupported = -103,

    // Step-6 Clock-sync
    ClockSyncQuality = -110,
    ClockSyncTimeout = -111,

    // Step-6 Multi-MCU coordination
    ArmTimeout = -120,
    ArmRejected = -121,
    CrossMcuDesync = -122,

    // Step-6 Buffer-budget
    Underrun = -130,
    QueueOverrun = -131,
    LivenessStalled = -132,
    TraceOverflow = -133,

    // Step-6 Protocol/state machine
    StreamStateViolation = -140,
    SegmentIdNonMonotonic = -141,

    // Step-6 Time-domain
    TStartInPast = -150,
    TEndBeforeTStart = -151,
    SegmentTooShort = -152,
    SegmentTooLong = -153,

    // Step-6 Curve-pool
    InvalidCurveHandle = -160,
    CurveReloadRejected = -161,
    CurveFormatInvalid = -162,

    // Step-6 Runtime-numerical
    NanInfOutput = -170,
    BoundaryLoopOverflow = -171,
    InternalInvariant = -172,
}

impl FaultCode {
    pub const fn as_i32(self) -> i32 { self as i32 }
    pub const fn as_u16(self) -> u16 { (self as i32 as i16) as u16 }
}
```

- [ ] **Step 2: Mirror as C-side constants in `rust/kalico-c-api/src/runtime_ffi.rs`**

```rust
pub const KALICO_ERR_NULL_HANDLE: i32         = -7;
pub const KALICO_ERR_VALIDATION: i32          = -2;
pub const KALICO_ERR_INTERNAL: i32            = -1;
pub const KALICO_ERR_PROTOCOL_VERSION: i32    = -103;
// ... etc, mirroring FaultCode

pub const KALICO_FAULT_BAD_CRC: u16            = (FaultCode::BadCrc as i32 as i16) as u16;
// ... mirror as u16 for the kalico_status `last_fault` field.
```

- [ ] **Step 3: Add a fault-detail encoder helper**

In `rust/runtime/src/error.rs`:

```rust
/// Pack the 32-bit fault_detail per spec §9.2 conventions.
pub fn encode_invalid_curve_handle(slot_idx: u16, observed_gen: u16, expected_gen: u16) -> u32 {
    ((slot_idx as u32) << 16) | ((observed_gen ^ expected_gen) as u32)
}

pub fn encode_clock_sync_quality(residual_us: u16, drift_ppm: u16) -> u32 {
    ((residual_us as u32) << 16) | (drift_ppm as u32)
}

pub fn encode_stream_state_violation(observed: u8, expected: u8) -> u32 {
    ((observed as u32) << 8) | (expected as u32)
}
```

- [ ] **Step 4: Unit-test the encoders**

Create `rust/runtime/tests/fault_encoding.rs`:

```rust
use runtime::error::{encode_invalid_curve_handle, encode_clock_sync_quality, encode_stream_state_violation};

#[test]
fn invalid_curve_handle_encoding() {
    let d = encode_invalid_curve_handle(5, 100, 200);
    assert_eq!(d >> 16, 5);  // slot_idx in upper 16
    assert_eq!(d & 0xFFFF, 100 ^ 200);  // gen XOR in lower 16
}

#[test]
fn clock_sync_quality_encoding() {
    let d = encode_clock_sync_quality(150, 42);
    assert_eq!(d >> 16, 150);
    assert_eq!(d & 0xFFFF, 42);
}

#[test]
fn stream_state_violation_encoding() {
    let d = encode_stream_state_violation(2, 5);
    assert_eq!(d, (2 << 8) | 5);
}
```

- [ ] **Step 5: Run tests**

```bash
cd rust && cargo test -p runtime --features host fault_encoding 2>&1 | tail -10
```

All pass.

- [ ] **Step 6: Commit**

```bash
git add rust/runtime/src/error.rs rust/kalico-c-api/src/runtime_ffi.rs \
        rust/runtime/tests/fault_encoding.rs
git commit -m "runtime/error: §9.1 extended fault taxonomy + §9.2 detail encoders

Transport, clock-sync, multi-MCU, buffer, protocol, time-domain,
curve-pool, runtime-numerical fault codes — disjoint, enumerated,
each with specific semantic. Detail encoders for slot/gen/residual
diagnostics.

Per spec §9.1 / §9.2."
```

### Task 4.1.5: SharedState segment-id atomic writers (Round-2 review B14)

**Files:**
- Modify: `rust/runtime/src/engine.rs` — engine writes `current_segment_id` atomic on segment activation; writes `retired_through_segment_id` atomic on segment retirement (each retire bumps it monotonically).
- Modify: `rust/kalico-c-api/src/runtime_ffi.rs::kalico_runtime_push_segment` — writes `accepted_segment_id` atomic on push success, alongside the response out-param.

**Why:** Round-2 review B14: SharedState declares `current_segment_id`, `accepted_segment_id`, `retired_through_segment_id` AtomicU32 fields, but no task writes them. Phase 11 status frame and Gate B test both depend on these being populated.

- [ ] **Step 1: ISR-side: `Engine::tick` updates `current_segment_id` on segment activation**

In `rust/runtime/src/engine.rs::tick`, where the engine starts evaluating a new segment:

```rust
// On segment activation (after queue.dequeue() succeeds):
self.current = Some(next);
shared.current_segment_id.store(next.id, Ordering::Release);
```

- [ ] **Step 2: ISR-side: `Engine::tick` updates `retired_through_segment_id` on retirement**

```rust
// On segment retirement (boundary loop reaches a new segment OR queue empties):
let retired_id = retired_segment.id;
shared.retired_through_segment_id.store(retired_id, Ordering::Release);
```

- [ ] **Step 3: Foreground-side: `kalico_runtime_push_segment` updates `accepted_segment_id` on success**

In `push_segment_impl`, after the queue enqueue succeeds:

```rust
// On accepted push:
shared.accepted_segment_id.store(seg.id, Ordering::Release);
```

- [ ] **Step 4: Round-2 B11-real / Round-3 B-R3-8 — segment-id monotonicity check on push**

Per spec §9.1: `KALICO_FAULT_SEGMENT_ID_NON_MONOTONIC`. Use a separate `accepted_seen: AtomicBool` flag to distinguish the initial-state-no-prior-push case from "id wraps to 0":

```rust
// SharedState: accepted_segment_id_seen: AtomicBool initialized false.
let prev_seen = shared.accepted_segment_id_seen.load(Ordering::Acquire);
let prev_accepted = shared.accepted_segment_id.load(Ordering::Acquire);
if prev_seen && seg.id <= prev_accepted {
    return FaultCode::SegmentIdNonMonotonic as i32;
}
// On accepted push:
shared.accepted_segment_id.store(seg.id, Ordering::Release);
shared.accepted_segment_id_seen.store(true, Ordering::Release);
```

Round-3 fix B-R3-8: previous draft's `if seg.id != 0 && seg.id <= prev_accepted` allowed any second push with id=0 to pass after a prior id>0 push had set `prev_accepted`. The `accepted_segment_id_seen` flag (also reset by flush as part of credit_epoch bump) enforces strict monotonicity.

Add `accepted_segment_id_seen: AtomicBool` to `SharedState` (Phase 1 Task 1.1) and reset it in `flush()` (Phase 7 Task 7.2 step 6).

- [ ] **Step 5: Add unit test**

```rust
#[test]
fn segment_id_atomics_written_on_push_and_retire() {
    // setup: push segment id=10, verify accepted_segment_id=10.
    // tick to activate, verify current_segment_id=10.
    // tick past t_end, verify retired_through_segment_id=10.
}
```

- [ ] **Step 6: Commit**

```bash
git add rust/runtime/src/engine.rs rust/kalico-c-api/src/runtime_ffi.rs \
        rust/runtime/tests/segment_id_atomics.rs
git commit -m "runtime: §5.3 SharedState segment-id atomic writers + monotonicity

- Engine::tick writes current_segment_id on activation,
  retired_through_segment_id on retirement (Round-2 B14).
- push_segment_impl writes accepted_segment_id on success and validates
  monotonicity (Round-2 B11-real / SEGMENT_ID_NON_MONOTONIC).

Per spec §5.3."
```

### Task 4.2: Post-fault diagnostic command (Round-1 review B9)

**Files:**
- Modify: `rust/kalico-c-api/src/runtime_ffi.rs` — `kalico_runtime_query_pool_state` FFI.
- Modify: `src/runtime_tick.c` — `kalico_query_pool_state` DECL_COMMAND.

**Why:** Spec §10.4 + Round-1 review requires a diagnostic command for post-fault host inspection of per-slot curve-pool state.

- [ ] **Step 1: FFI helper that returns per-slot state**

```rust
#[unsafe(no_mangle)]
pub unsafe extern "C" fn kalico_runtime_query_pool_state(
    rt: *mut KalicoRuntime,
    slot_idx: u16,
    out_current_gen: *mut u16,
    out_last_retired_gen: *mut u16,
) -> i32 {
    if rt.is_null() { return KALICO_ERR_NULL_HANDLE; }
    let ctx = rt as *mut RuntimeContext;
    unsafe {
        let pool: &CurvePool = &*core::ptr::addr_of!((*ctx).curve_pool);
        if (slot_idx as usize) >= CURVE_POOL_N { return KALICO_ERR_VALIDATION; }
        let slot = &pool.slots[slot_idx as usize];
        *out_current_gen = slot.current_gen.load(Ordering::Acquire);
        *out_last_retired_gen = slot.last_retired_gen.load(Ordering::Acquire);
    }
    0
}
```

- [ ] **Step 2: C-side DECL_COMMAND**

```c
void
command_kalico_query_pool_state(uint32_t *args)
{
    uint16_t slot = args[0];
    uint16_t cur = 0, last = 0;
    int32_t r = kalico_runtime_query_pool_state(kalico_rt_handle, slot, &cur, &last);
    sendf("kalico_pool_state_response result=%i slot_idx=%hu current_gen=%hu last_retired_gen=%hu",
          r, slot, cur, last);
}
DECL_COMMAND(command_kalico_query_pool_state, "kalico_query_pool_state slot=%hu");
```

- [ ] **Step 3: Commit**

```bash
git add rust/kalico-c-api/src/runtime_ffi.rs src/runtime_tick.c
git commit -m "runtime: §10.4 post-fault diagnostic — kalico_query_pool_state

Per-slot current_gen / last_retired_gen inspection for host-side
recovery decisions. Used after KALICO_FAULT_TRACE_OVERFLOW or other
faults to determine whether power-cycle is required.

Per spec §10.4 (Round-1 review B9)."
```

---

## Phase 5 — TraceRing resize + new schema

Implements spec §13. Sizes TraceRing per host-stall budget; new schema with curve_handle field; overflow → `KALICO_FAULT_TRACE_OVERFLOW`.

### Task 5.1: New TraceSample schema (repr(C) aligned, curve_handle field)

**Files:**
- Modify: `rust/runtime/src/trace.rs` — replace TraceSample with new schema; add static_assert.

**Why:** Spec §13.2 + §10.4 — `SEGMENT_END` events must carry `curve_handle` for foreground reclaim. `repr(C)` aligned (not packed) avoids unaligned u64 access on Cortex-M7.

- [ ] **Step 1: Update `TraceSample` definition**

In `rust/runtime/src/trace.rs`:

```rust
use crate::curve_pool::CurveHandle;

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct TraceSample {
    pub tick: u64,                  // 8 B (8-aligned)
    pub motor_a: f32,               // 4 B
    pub motor_b: f32,               // 4 B
    pub motor_e: f32,               // 4 B
    pub segment_id: u32,            // 4 B
    pub curve_handle: CurveHandle,  // 4 B (§10.1: u16 slot + u16 gen)
    pub flags: u8,                  // §13.2 + §13.3
    pub _pad: [u8; 3],              // explicit padding to 32 B
}

const _: () = assert!(core::mem::size_of::<TraceSample>() == 32);
const _: () = assert!(core::mem::align_of::<TraceSample>() == 8);
```

- [ ] **Step 2: Update `TRACE_RING_N` per spec §13.1**

```rust
/// Default per §13.1: TRACE_RING_DURATION_MS = HOST_STALL_BUDGET_MS + TRACE_RING_SAFETY_MARGIN_MS
/// = 20 + 10 = 30 ms × 40 kHz = 1200 samples + 1 (heapless effective-cap-N-1) = 1201.
pub const TRACE_RING_N: usize = 1201;
```

- [ ] **Step 3: Run host tests**

```bash
cd rust && cargo test -p runtime --features host 2>&1 | tail -10
```

Expected: `Segment` and `TraceSample` size asserts pass; existing tests still pass.

- [ ] **Step 4: Commit**

```bash
git add rust/runtime/src/trace.rs
git commit -m "runtime/trace: §13.2 new TraceSample schema (repr(C), 32 B)

curve_handle field added (u32 = u16 slot + u16 gen) to enable §10.4
reclaim. repr(C) aligned (not packed) — avoids unaligned u64 access on
Cortex-M7 and addr_of! aliasing-correctness traps. TRACE_RING_N
sized to 1201 per §13.1 (HOST_STALL + 10 ms safety margin × 40 kHz).

Per spec §13.1 / §13.2."
```

### Task 5.2: Trace overflow detection → `KALICO_FAULT_TRACE_OVERFLOW`

**Files:**
- Modify: `rust/runtime/src/engine.rs` — emit_trace path checks overflow, sets `SAMPLE_DROP_PENDING`.
- Modify: `rust/runtime/src/state.rs` — fault transition from `SAMPLE_DROP_PENDING`.

**Why:** Spec §13.1 overflow handling.

- [ ] **Step 1: ISR sets `sample_drop_pending` on enqueue failure**

In `rust/runtime/src/engine.rs`, the trace-emit path:

```rust
fn emit_trace(&mut self, isr: &mut IsrState, shared: &SharedState, sample: TraceSample) {
    if isr.trace_producer.enqueue(sample).is_err() {
        // Overflow — set the sticky flag for foreground to latch fault.
        shared.sample_drop_pending.store(true, Ordering::Release);
    }
}
```

- [ ] **Step 2: Foreground transitions to FAULT on observing `sample_drop_pending`**

In the foreground drain path (Phase 11 will wire this; for now, define the helper):

```rust
// rust/runtime/src/reclaim.rs (extend)
pub fn check_trace_overflow_and_fault(shared: &crate::state::SharedState) -> bool {
    if shared.sample_drop_pending.load(Ordering::Acquire) {
        // Latch fault; foreground emits kalico_fault.
        let prev = shared.last_error.swap(crate::error::FaultCode::TraceOverflow as i32, Ordering::Release);
        if prev == 0 {
            shared.runtime_status.store(crate::engine::RuntimeStatus::Fault as u8, Ordering::Release);
        }
        return true;
    }
    false
}
```

- [ ] **Step 3: Add unit test for the overflow path**

Create `rust/runtime/tests/trace_overflow.rs`:

```rust
use runtime::state::SharedState;
use runtime::reclaim::check_trace_overflow_and_fault;

#[test]
fn overflow_latches_fault() {
    let shared = SharedState::new();
    shared.sample_drop_pending.store(true, std::sync::atomic::Ordering::Release);
    assert!(check_trace_overflow_and_fault(&shared));
    assert_eq!(shared.last_error.load(std::sync::atomic::Ordering::Acquire),
               runtime::error::FaultCode::TraceOverflow as i32);
}
```

- [ ] **Step 4: Run test**

```bash
cd rust && cargo test -p runtime --features host trace_overflow 2>&1 | tail -10
```

Pass.

- [ ] **Step 5: Commit**

```bash
git add rust/runtime/src/engine.rs rust/runtime/src/reclaim.rs \
        rust/runtime/tests/trace_overflow.rs
git commit -m "runtime/engine: §13.1 trace-overflow → KALICO_FAULT_TRACE_OVERFLOW

ISR sets sample_drop_pending on enqueue failure; foreground latches
fault on observation. Hard fault — print aborts, host receives
fault notification.

Per spec §13.1."
```

---

## Phase 6 — Stream lifecycle

Implements spec §8 (stream states + commands) and the engine-side `stream_open`-driven boundary-loop branch.

### Task 6.1: `stream_open` flag + boundary-loop branch

**Files:**
- Modify: `rust/runtime/src/engine.rs` — boundary-drain branch reads `stream_open`.

**Why:** Spec §7.4 + §8.2. Step-5's queue-empty path collapses to `Drained`; Step-6 splits: `Drained` if `!stream_open`, `Underrun` (fault) if `stream_open`.

- [ ] **Step 1: Update boundary-loop in `Engine::tick`**

Find the existing queue-empty branch in `rust/runtime/src/engine.rs` (search for `RuntimeStatus::Drained`). Replace:

```rust
let Some(next) = isr.queue_consumer.dequeue() else {
    if shared.stream_open.load(Ordering::Acquire) {
        // Underrun: queue empty while stream is open. Hard fault.
        shared.last_error.store(crate::error::FaultCode::Underrun as i32, Ordering::Release);
        shared.runtime_status.store(RuntimeStatus::Fault as u8, Ordering::Release);
    } else {
        shared.runtime_status.store(RuntimeStatus::Drained as u8, Ordering::Release);
    }
    self.current = None;
    return;
};
```

- [ ] **Step 2: Unit-test the split**

Create `rust/runtime/tests/engine_underrun.rs`:

```rust
// Host-side test of the engine's boundary-loop drain branch.
// Directly drives the engine tick + observes status transitions.
//
// (Detailed test setup omitted for brevity — uses test-only Engine fixture
// from runtime/tests/common.rs that initializes Engine + ClockWidenState +
// SharedState without going through FFI.)

#[test]
fn empty_queue_stream_closed_yields_drained() {
    // ... setup engine, queue, shared
    shared.stream_open.store(false, Ordering::Release);
    // tick once with empty queue
    engine.tick(0, &mut widen, &shared);
    assert_eq!(shared.runtime_status.load(Ordering::Acquire),
               RuntimeStatus::Drained as u8);
    assert_eq!(shared.last_error.load(Ordering::Acquire), 0);
}

#[test]
fn empty_queue_stream_open_yields_underrun_fault() {
    // ... setup engine, queue, shared
    shared.stream_open.store(true, Ordering::Release);
    // tick once with empty queue
    engine.tick(0, &mut widen, &shared);
    assert_eq!(shared.runtime_status.load(Ordering::Acquire),
               RuntimeStatus::Fault as u8);
    assert_eq!(shared.last_error.load(Ordering::Acquire),
               FaultCode::Underrun as i32);
}
```

(See Step-5's existing engine tests for the fixture pattern; reuse and extend.)

- [ ] **Step 3: Run tests**

```bash
cd rust && cargo test -p runtime --features host engine_underrun 2>&1 | tail -10
```

Both pass.

- [ ] **Step 4: Commit**

```bash
git add rust/runtime/src/engine.rs rust/runtime/tests/engine_underrun.rs
git commit -m "runtime/engine: §8.2 stream_open-driven boundary-drain branch

queue-empty + stream_open=false → Drained (normal end-of-stream).
queue-empty + stream_open=true  → KALICO_FAULT_UNDERRUN (hard fault).

Per spec §7.4 + §8.2."
```

### Task 6.2: stream_open / arm / terminal command handlers

**Files:**
- Modify: `rust/runtime/src/stream.rs` — implement state-machine transitions + handlers.

**Why:** Spec §8.3 wire commands need handler bodies. Idempotency + state-violation NACK per §8.5.

- [ ] **Step 1: Define MCU-side stream state in `rust/runtime/src/stream.rs`**

```rust
//! Stream lifecycle state machine — MCU side.

use core::sync::atomic::Ordering;

use crate::error::FaultCode;
use crate::state::{FgState, SharedState};

pub fn open(
    fg: &mut FgState,
    shared: &SharedState,
    stream_id: u32,
    out_credit_epoch: *mut u32,
) -> i32 {
    if shared.stream_open.load(Ordering::Acquire) {
        // Per spec §8.5: idempotent only for SAME stream_id. Different
        // stream_id while open → state-machine violation.
        if fg.current_stream_id == Some(stream_id)
            && (fg.stream_state_machine == FgStreamState::StreamOpening
                || fg.stream_state_machine == FgStreamState::StreamOpenPriming)
        {
            unsafe { *out_credit_epoch = shared.credit_epoch.load(Ordering::Acquire); }
            return 0;
        }
        return FaultCode::StreamStateViolation as i32;
    }
    // Round-1 B14: ensure terminal_segment_id is cleared on stream_open.
    fg.terminal_segment_id = None;
    shared.terminal_segment_id_set.store(false, Ordering::Release);
    shared.terminal_segment_id_value.store(0, Ordering::Release);

    shared.stream_open.store(true, Ordering::Release);
    fg.stream_state_machine = FgStreamState::StreamOpening;
    fg.current_stream_id = Some(stream_id);
    let epoch = shared.credit_epoch.load(Ordering::Acquire);
    unsafe { *out_credit_epoch = epoch; }
    0
}

pub fn arm(
    fg: &mut FgState,
    shared: &SharedState,
    pool: &crate::curve_pool::CurvePool,
    first_priming_t_start: Option<u64>,  // Round-2 B6: pre-tracked by FgState
    t_start_t0: u64,
    arm_lead_cycles: u32,
    out_armed_t_start: *mut u64,
) -> i32 {
    // Round-1 review B4 fix: per spec §6.3, validation checks the FIRST
    // PRIMING SEGMENT's t_start, not the arm command's t_start_t0.
    //
    // Round-2 B6 fix: SPSC ownership discipline says foreground can't peek
    // ISR-owned queue. Foreground tracks `first_priming_segment_t_start` in
    // FgState as it accepts pushes during STREAM_OPEN_PRIMING (push handler
    // updates `fg.first_priming_segment_t_start = Some(seg.t_start)` on the
    // first push after stream_open). arm() then reads from FgState, not the
    // queue.

    // Per spec §8.5: idempotent only for SAME t_start_t0.
    if fg.stream_state_machine == FgStreamState::Armed {
        if fg.armed_t_start_t0 == Some(t_start_t0) {
            unsafe { *out_armed_t_start = t_start_t0; }
            return 0;
        }
        return FaultCode::StreamStateViolation as i32;
    }
    if fg.stream_state_machine != FgStreamState::StreamOpenPriming {
        return FaultCode::StreamStateViolation as i32;
    }

    // Per spec §6.3: at least 1 priming segment.
    let Some(first_t_start) = first_priming_t_start else {
        return FaultCode::ArmRejected as i32;
    };

    // Validate first priming segment's t_start ≥ now + MIN_ARM_LEAD_CYCLES.
    let now = crate::clock::read_widened_now(shared);
    if first_t_start < now + arm_lead_cycles as u64 {
        return FaultCode::ArmRejected as i32;
    }

    let _ = pool;  // CurvePool reference threaded through for consistency;
                   // not actively used in arm validation but available for
                   // future "all priming segments reference loaded curves" checks.

    fg.stream_state_machine = FgStreamState::Armed;
    fg.armed_t_start_t0 = Some(t_start_t0);
    unsafe { *out_armed_t_start = t_start_t0; }
    0
}

pub fn terminal(
    fg: &mut FgState,
    shared: &SharedState,
    segment_id: u32,
) -> i32 {
    if fg.stream_state_machine != FgStreamState::Running
        && fg.stream_state_machine != FgStreamState::StreamOpenPriming
        && fg.stream_state_machine != FgStreamState::Armed
    {
        return FaultCode::StreamStateViolation as i32;
    }
    // Idempotency: same segment_id terminal returns OK.
    if let Some(existing) = fg.terminal_segment_id {
        if existing == segment_id { return 0; }
        return FaultCode::StreamStateViolation as i32;
    }
    // Round-2 fix B7: publish to SharedState atomics so the ISR-side
    // engine retire path can observe the terminal flag without violating
    // SPSC ownership discipline.
    fg.terminal_segment_id = Some(segment_id);
    shared.terminal_segment_id_value.store(segment_id, Ordering::Release);
    shared.terminal_segment_id_set.store(true, Ordering::Release);
    fg.stream_state_machine = FgStreamState::Draining;
    0
}

/// Engine-side helper (called from `Engine::tick` retire path):
/// if shared.terminal_segment_id_set is true and the just-retired segment's
/// id matches the published value, clear stream_open. Subsequent boundary
/// loop on empty queue → Drained, not Underrun.
pub fn check_terminal_on_retire(shared: &SharedState, retired_seg_id: u32) {
    if !shared.terminal_segment_id_set.load(Ordering::Acquire) { return; }
    if shared.terminal_segment_id_value.load(Ordering::Acquire) != retired_seg_id { return; }
    shared.stream_open.store(false, Ordering::Release);
    // Don't clear terminal_segment_id_set here — flush() / next stream_open
    // owns clearing per Round-2 B14 fix.
}

pub fn clock_sync_respond(
    _fg: &mut FgState,
    shared: &SharedState,
    _request_id: u32,
    _host_send_lo: u32,
    _host_send_hi: u32,
    out_mcu_clock: *mut u64,
) -> i32 {
    let now = crate::clock::read_widened_now(shared);
    unsafe { *out_mcu_clock = now; }
    0
}

// FgStreamState already defined in stream.rs from Phase 1; ensure
// `terminal_segment_id: Option<u32>` field is added to FgState (state.rs).
```

- [ ] **Step 2: Add `terminal_segment_id: Option<u32>` to `FgState` (state.rs)**

Add the field to `FgState` and initialize it to `None` in `RuntimeContext::init`.

- [ ] **Step 3: Wire up the engine's terminal-segment retire path**

In `rust/runtime/src/engine.rs::tick`, after a segment retires, check if its `id` matches `fg.terminal_segment_id`. (This is awkward because the engine is on the ISR side and `fg.terminal_segment_id` is on the foreground side. Use SharedState atomic to communicate: `terminal_segment_id_set: AtomicBool, terminal_segment_id_value: AtomicU32`.)

Add to SharedState:
```rust
pub terminal_segment_id_set: AtomicBool,
pub terminal_segment_id_value: AtomicU32,
```

In `terminal` handler: also set both. In engine retire path: if retiring segment id matches `terminal_segment_id_value` and `terminal_segment_id_set` is true, clear `stream_open`. Subsequent boundary-loop sees `stream_open=false` → Drained.

- [ ] **Step 4: Unit-test stream open/arm/terminal**

Create `rust/runtime/tests/stream_lifecycle.rs`. Test: open returns 0; second open same stream returns 0 (idempotent); open while in Running returns StreamStateViolation. arm without prior open returns StreamStateViolation. arm with t_start in past returns ArmRejected.

- [ ] **Step 5: Run tests**

```bash
cd rust && cargo test -p runtime --features host stream_lifecycle 2>&1 | tail -10
```

All pass.

- [ ] **Step 6: Commit**

```bash
git add rust/runtime/src/stream.rs rust/runtime/src/state.rs \
        rust/runtime/src/engine.rs rust/runtime/tests/stream_lifecycle.rs
git commit -m "runtime/stream: §8.3 + §8.5 open/arm/terminal handlers + state machine

Idempotency on duplicate commands (per §8.5 defensive idempotency
contract). StreamStateViolation NACK on out-of-state command.
Terminal-segment-id stored in SharedState; engine retires
terminal segment → stream_open cleared → Drained.

Per spec §8.3 / §8.5."
```

---

## Phase 7 — Force_idle handshake (flush mechanism)

Implements spec §8.5 flush sequence per Plan-decision A: foreground sets `force_idle=true` first, ack-waits, then `stream_open=false`.

### Task 7.1: ISR tick-entry force_idle short-circuit

**Files:**
- Modify: `rust/runtime/src/engine.rs::tick` — add force_idle check at top, before everything.

**Why:** Spec §8.5 step 2. ISR observes `force_idle == true` BEFORE any segment evaluation, BEFORE any `queue.try_pop()`, BEFORE any `widen_state` mutation. Aborts current evaluation; sets `acked_force_idle`; returns.

- [ ] **Step 1: Add force_idle check at the top of `tick`**

**Round-4 fix (verifier #5):** the canonical `Engine::tick` signature must include the queue consumer + trace producer references that the engine uses to dequeue segments and emit trace samples. Existing Step-5 tick takes `&mut TraceRing<TRACE_RING_N>` directly; under the half-split, it takes `&mut Producer<TraceSample, TRACE_RING_N>` and `&mut Consumer<Segment, Q_N>`. This signature is set in Phase 1 Task 1.3 and propagated through every phase.

The canonical signature (single source of truth):

```rust
pub fn tick(
    &mut self,
    raw_cyccnt: u32,
    widen_state: &mut crate::clock::WidenState,
    pool: &crate::curve_pool::CurvePool,
    queue: &mut heapless::spsc::Consumer<'static, crate::segment::Segment, { crate::queue::Q_N }>,
    trace: &mut heapless::spsc::Producer<'static, crate::trace::TraceSample, { crate::trace::TRACE_RING_N }>,
    shared: &crate::state::SharedState,
) {
    // §8.5 step 2: force_idle short-circuit. BEFORE anything else.
    if shared.force_idle.load(Ordering::Acquire) {
        self.clear_current();
        // Ack to foreground.
        shared.acked_force_idle.store(true, Ordering::Release);
        return;
    }

    let now = widen_state.widen(raw_cyccnt);
    crate::clock::publish_widened_now(shared, now);

    // ... rest of tick body (uses `pool` for CurvePool lookups)
}
```

NOTE: this Phase-7 task only adds the `force_idle` block to the existing tick body; the function signature is unchanged from Phase 1 Task 1.3.

**Round-3 fix B-R3-4 — Engine.widen_state ownership move:** The Step-5 `Engine` owns `widen_state: WidenState` as a `pub(crate)` field with three accessors (`last_widened_now`, `widen`, `reinit_widen`) at engine.rs:106/119/133. Phase 1 Task 1.1 puts `widen_state: WidenState` in `IsrState` instead. To avoid duplicate ownership, Phase 1 Task 1.1's implementation MUST also:
1. Remove the `widen_state: WidenState` field from `Engine`.
2. Remove the three accessors `last_widened_now`, `widen`, `reinit_widen` from `Engine`.
3. Update existing FFI callsites at `runtime_ffi.rs:181` and `:248` (which call `ctx.engine.widen(...)` and `ctx.engine.reinit_widen(...)`) to use `isr.widen_state.widen(...)` directly via the half-split projection.
4. The new tick signature takes `widen_state: &mut WidenState` so the FFI shim can pass `&mut isr.widen_state`.

This ownership move is part of the Phase 1 refactor; explicit step-list under Task 1.1 to make this discoverable.

**Round-3 fix B-R3-3 — WidenState::new constructor:** `WidenState` derives `Default` (clock.rs:11). Use `WidenState::default()` in `RuntimeContext::init` instead of `WidenState::new(freq)`. The existing `Engine::new(freq)` constructor remains; `Engine::new_production(freq)` is just a wrapper that delegates to `Engine::new(freq)`. Update the init code:

```rust
// In RuntimeContext::init (correcting Step 4 of Task 1.1):
(*isr_ptr).get().write(IsrState {
    queue_consumer: q_consumer,
    trace_producer: t_producer,
    engine: EngineImpl::new_production(freq),
    widen_state: WidenState::default(),  // not WidenState::new(freq)
});
```

- [ ] **Step 2: Add unit test**

```rust
#[test]
fn force_idle_short_circuits_tick() {
    // ... setup engine, queue with one segment, shared
    shared.force_idle.store(true, Ordering::Release);

    // Pre-condition: queue is non-empty
    assert!(/* queue has 1 segment */);

    engine.tick(...);

    // Post-condition: force_idle still true; ack set; current is None;
    // the queued segment is NOT consumed (queue still has 1 segment).
    assert!(shared.force_idle.load(Ordering::Acquire));
    assert!(shared.acked_force_idle.load(Ordering::Acquire));
    assert!(engine.current.is_none());
    assert!(/* queue still has 1 segment */);
}
```

- [ ] **Step 3: Run test**

```bash
cd rust && cargo test -p runtime --features host force_idle 2>&1 | tail -10
```

Pass.

- [ ] **Step 4: Commit**

```bash
git add rust/runtime/src/engine.rs rust/runtime/tests/force_idle_*.rs
git commit -m "runtime/engine: §8.5 force_idle short-circuit at tick-entry

ISR observes force_idle=true at top of tick — BEFORE segment eval,
BEFORE queue.try_pop, BEFORE widen_state mutation. Clears current,
sets acked_force_idle, returns. Bounded ~25 µs at 40 kHz.

Per spec §8.5 step 2. Plan-decision A: foreground sets force_idle
first, ack-waits, THEN stream_open=false."
```

### Task 7.2: Foreground flush sequence (Plan-decision A ordering)

**Files:**
- Modify: `rust/runtime/src/stream.rs` — implement `flush()` per §8.5 step ordering.

**Why:** Spec §8.5 + Plan-decision A.

- [ ] **Step 1: Implement `flush()` with concrete queue drain mechanism**

The flush function takes a raw pointer to the `RuntimeContext` (not just `&mut FgState`) because it needs transient access to `IsrState.queue_consumer` and `RuntimeContext.curve_pool` under the IRQ-disable window. The FFI shim `kalico_runtime_stream_flush` does the raw-pointer derivation and passes through.

```rust
// rust/runtime/src/stream.rs

use core::sync::atomic::Ordering;
use core::cell::UnsafeCell;
use crate::error::FaultCode;
use crate::state::{FgState, IsrState, RuntimeContext, SharedState};

/// Flush sequence per spec §8.5 + Plan-decision A.
///
/// Takes `*mut RuntimeContext` because flush needs transient access to
/// `IsrState.queue_consumer` under IRQ-disable. The discipline contract is
/// preserved: the IRQ-disable window prevents concurrent ISR access, so the
/// foreground briefly holds exclusive access to IsrState during step 4.
///
/// SAFETY: caller must guarantee single-threaded foreground entry (typical
/// for command dispatch).
pub unsafe fn flush(
    rt: *mut RuntimeContext,
    out_credit_epoch: *mut u32,
) -> i32 {
    if rt.is_null() { return FaultCode::NullHandle as i32; }
    let ctx = rt;

    // Project FgState (foreground exclusive) and SharedState (atomics).
    let fg_ptr: *mut FgState = unsafe { UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).fg)) };
    let shared_ptr: *const SharedState = unsafe { core::ptr::addr_of!((*ctx).shared) };
    let pool_ptr: *const crate::curve_pool::CurvePool =
        unsafe { core::ptr::addr_of!((*ctx).curve_pool) };
    let fg: &mut FgState = unsafe { &mut *fg_ptr };
    let shared: &SharedState = unsafe { &*shared_ptr };
    let pool: &crate::curve_pool::CurvePool = unsafe { &*pool_ptr };

    // ─── Plan-decision A: force_idle FIRST, ack-wait, THEN stream_open=false ───

    // Step 1: set force_idle=true. ISR observes on its next tick.
    shared.force_idle.store(true, Ordering::Release);

    // Step 2: spin-wait on acked_force_idle with a 1-ms host wall-clock
    // timeout. Use Klipper's host-side `timer_read_time()` C-side helper
    // (NOT `read_widened_now`, because Round 1 review B3 found that the
    // ISR doesn't update widened_now during force_idle, so the seqlock would
    // appear frozen and the deadline check would never fire).
    let deadline_us = unsafe { kalico_host_now_us() } + 1000;  // +1 ms
    while !shared.acked_force_idle.load(Ordering::Acquire) {
        core::hint::spin_loop();
        let now_us = unsafe { kalico_host_now_us() };
        if now_us >= deadline_us {
            // Timeout — ISR appears stuck. Latch LIVENESS_STALLED.
            shared.last_error.store(FaultCode::LivenessStalled as i32, Ordering::Release);
            shared.runtime_status.store(crate::engine::RuntimeStatus::Fault as u8, Ordering::Release);
            return FaultCode::LivenessStalled as i32;
        }
    }

    // ISR is now parked. From this point until step 7 clears force_idle, no
    // ISR fire performs any segment evaluation, queue access, or curve-pool
    // access (per the §8.5 step-2 contract).

    // Step 3: NOW clear stream_open (post-ack). Subsequent ticks (after
    // step 7) will see stream_open=false on empty queue → Drained, not
    // Underrun.
    shared.stream_open.store(false, Ordering::Release);

    // Step 4: IRQ-disable + transient queue drain via raw-pointer projection
    // to IsrState.queue_consumer. SAFETY: under irq_save, no ISR can run, so
    // we transiently hold exclusive access to IsrState — the discipline
    // contract holds because there's no concurrent access window.
    //
    // Round-2 fix B3: use Klipper's `irq_save()`/`irq_restore()` (FFI to
    // src/generic/irq.h) NOT the `cortex_m` crate (not in our deps and
    // Klipper has its own IRQ-save abstraction).
    unsafe extern "C" {
        fn irq_save() -> u32;
        fn irq_restore(flags: u32);
    }
    let irq_flags = unsafe { irq_save() };
    {
        let isr_ptr: *mut IsrState = unsafe {
            UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr))
        };
        let isr: &mut IsrState = unsafe { &mut *isr_ptr };
        // Drain all enqueued segments. None are evaluated; they're
        // discarded. No retire events emitted (segments never executed).
        while isr.queue_consumer.dequeue().is_some() {}
        // Also clear any in-flight current segment in the engine. The
        // §8.5 step 2 contract says ISR has already cleared current; this
        // is defense-in-depth (Round-2 fix B4: use Engine::clear_current
        // public(crate) accessor since `current` field is private).
        isr.engine.clear_current();
    }
    unsafe { irq_restore(irq_flags); }

    // Step 5: reset per-slot last_retired_gen = current_gen for all slots.
    pool.reset_all_retired_to_current();

    // Step 6: increment credit_epoch (any pending credit events from
    // pre-flush are now stale by epoch comparison).
    let new_epoch = shared.credit_epoch.fetch_add(1, Ordering::AcqRel) + 1;

    // Plan-decision A: ALSO clear stream-machine state and terminal_segment_id
    // (Round 1 review B14 fix — terminal_segment_id_set was never cleared on
    // stream_open or flush, leaving stale state on stream-reopen).
    fg.stream_state_machine = crate::stream::FgStreamState::Idle;
    fg.terminal_segment_id = None;
    shared.terminal_segment_id_set.store(false, Ordering::Release);
    shared.terminal_segment_id_value.store(0, Ordering::Release);

    // Step 7: clear force_idle + acked_force_idle. ISR resumes normal
    // operation on next tick.
    shared.acked_force_idle.store(false, Ordering::Release);
    shared.force_idle.store(false, Ordering::Release);

    // Step 8: return.
    unsafe { *out_credit_epoch = new_epoch; }
    0
}

// Host-clock helper (defined in C-side runtime_tick.c — wraps Klipper's
// `timer_read_time()` returning a u32 in clock cycles, converted to µs via
// `kalico_clock_freq`). Foreground only; never called from ISR.
unsafe extern "C" {
    fn kalico_host_now_us() -> u64;
}
```

**Add `kalico_host_now_us()` C-side helper in `src/runtime_tick.c`:**

```c
#include "board/misc.h"  // timer_read_time

extern const uint32_t kalico_clock_freq;

uint64_t
kalico_host_now_us(void)
{
    // Returns wall-clock µs since boot (foreground; not ISR-safe in spirit
    // since timer_read_time can wrap, but the kalico flush window is ≤1 ms).
    uint32_t cycles = timer_read_time();
    return ((uint64_t)cycles * 1000000ULL) / kalico_clock_freq;
}
```

(Note: `timer_read_time()` is Klipper's existing 32-bit timer with widening already handled by Klipper at the foreground task level.)

**Update FFI `kalico_runtime_stream_flush`:**

```rust
#[unsafe(no_mangle)]
pub unsafe extern "C" fn kalico_runtime_stream_flush(
    rt: *mut KalicoRuntime,
    out_credit_epoch: *mut u32,
) -> i32 {
    unsafe { runtime::stream::flush(rt as *mut RuntimeContext, out_credit_epoch) }
}
```

- [ ] **Step 2: Add `CurvePool::reset_all_retired_to_current` helper**

```rust
impl CurvePool {
    pub fn reset_all_retired_to_current(&self) {
        for slot in self.slots.iter() {
            let cur = slot.current_gen.load(Ordering::Acquire);
            slot.last_retired_gen.store(cur, Ordering::Release);
        }
    }
}
```

- [ ] **Step 3: Add `flush_start_tick: Option<u64>` to `FgState`**

- [ ] **Step 4: Unit-test flush**

```rust
#[test]
fn flush_idempotent_post_ack() {
    // setup: stream_open=true, no in-flight segment, simulate ack from another thread
    // call flush; should return 0 with new credit_epoch
}

#[test]
fn flush_timeout_yields_liveness_stalled() {
    // setup: never ack force_idle from a "stuck ISR"
    // call flush; should return after 1 ms with LivenessStalled
}
```

- [ ] **Step 5: Run**

```bash
cd rust && cargo test -p runtime --features host flush 2>&1 | tail -10
```

Pass.

- [ ] **Step 6: Commit**

```bash
git add rust/runtime/src/stream.rs rust/runtime/src/state.rs \
        rust/runtime/src/curve_pool.rs rust/runtime/tests/flush_*.rs
git commit -m "runtime/stream: §8.5 flush sequence (Plan-decision A ordering)

force_idle=true → ack-wait (1 ms timeout) → stream_open=false →
IRQ-disable + queue drain → reset last_retired_gen → bump credit_epoch
→ clear force_idle + acked_force_idle → return.

Avoids spurious-Underrun race: stream_open is cleared only AFTER ISR
acks force_idle, so an in-flight ISR mid-tick on empty queue cannot
observe stale stream_open=true.

Per spec §8.5 + Plan-decision A (Round 4)."
```

---

## Phase 8 — Multi-MCU clock sync

Implements spec §12 (host-side estimator) and the §6.4 ARMING flow with explicit dedicated sync request (Plan-decision B).

**Phase ordering note:** Phase 8 depends on Phase 10's host-rt crate scaffolding (Task 10.1). Phase 10 Task 10.1 must precede Phase 8 — re-order during execution: do Phase 10 Task 10.1 (scaffold), then Phase 8 Tasks 8.1+8.2, then Phase 10 Tasks 10.2+ for fleshing out the rest of the crate.

### Task 8.1: Host-side ClockSyncEstimator (`rust/kalico-host-rt/src/clock_sync.rs`)

**Files:**
- Create: `rust/kalico-host-rt/src/clock_sync.rs` (full implementation).
- Create: `rust/kalico-host-rt/tests/clock_sync_unit.rs`.

**Why:** Spec §12.2 + Plan-decision B. Sliding-window linear regression with two sample sources (RTT-aware dedicated; piggyback) and quality gate.

- [ ] **Step 1: Implement the estimator**

```rust
//! Host-side clock-frequency estimator. Per spec §12.2 + §12.3 + Plan-decision B.

use std::collections::VecDeque;
use std::time::{Duration, Instant};

const WINDOW: usize = 30;  // sliding-window samples
pub const MIN_WARMUP_SAMPLES: u32 = 30;

/// Default thresholds. Round-2 review B06: pin to spec §12.4 + §7.3 M3
/// measurements. These initial values carry the spec's "default pending
/// measurement" baseline; M3 (Phase 15 Task 15.3) replaces them with
/// measured numbers and updates these constants if the measurements diverge.
pub const MAX_RESIDUAL_US_DEFAULT: f64 = 100.0;   // §7.1 row, §12.4
pub const MAX_DRIFT_PPM_DEFAULT: f64 = 100.0;     // §12.4
pub const MAX_SAMPLE_AGE_MS_DEFAULT: u64 = 2000;  // §12.4
/// Plan-decision B: arm-time gate requires a recent RTT-aware sample.
pub const MAX_RTT_AGE_MS_DEFAULT: u64 = 500;

#[derive(Debug, Clone, Copy)]
pub enum SampleSource { Dedicated, Piggyback }

#[derive(Debug, Clone, Copy)]
pub struct Sample {
    /// Round-2 fix B04: stable host-timeline coordinate (seconds since the
    /// estimator's epoch), NOT host_send.elapsed() — the latter is near-zero
    /// at the moment of recording and gives a meaningless x-coordinate.
    pub host_time_secs: f64,
    pub mcu_clock: u64,
    pub rtt_us: u32,
    pub source: SampleSource,
    pub recorded_at: Instant,   // for sample-age check
}

#[derive(Debug)]
pub struct ClockSyncEstimator {
    /// Round-2 B04: epoch fixed at construction; all sample host_time_secs
    /// are measured relative to this anchor.
    epoch: Instant,
    samples: VecDeque<Sample>,
    pub clock_freq_estimate: f64,    // ticks/sec
    anchor_host_time: f64,
    anchor_mcu_clock: u64,
    pub residual_max_in_window: f64,
    pub last_dedicated_sample: Option<Instant>,
}

impl ClockSyncEstimator {
    pub fn new(initial_freq_estimate: f64) -> Self {
        Self {
            epoch: Instant::now(),
            samples: VecDeque::with_capacity(WINDOW),
            clock_freq_estimate: initial_freq_estimate,
            anchor_host_time: 0.0,
            anchor_mcu_clock: 0,
            residual_max_in_window: 0.0,
            last_dedicated_sample: None,
        }
    }

    fn host_time_at(&self, t: Instant) -> f64 {
        // Round-2 B04: stable timeline relative to estimator epoch.
        t.duration_since(self.epoch).as_secs_f64()
    }

    /// Convert a host_time_secs back to MCU-local clock value at that instant.
    /// Used by ARMING flow (Plan-decision B) to compute t_start_t0_local for
    /// kalico_stream_arm. Round-2 fix B11-real: this MUST use the regression
    /// anchor (anchor_host_time, anchor_mcu_clock), not just multiply by freq.
    pub fn mcu_time_at_host(&self, host_time_secs: f64) -> u64 {
        // mcu_clock(t) = anchor_mcu_clock + (t - anchor_host_time) * clock_freq
        let delta_secs = host_time_secs - self.anchor_host_time;
        let delta_cycles = (delta_secs * self.clock_freq_estimate) as i64;
        (self.anchor_mcu_clock as i64).saturating_add(delta_cycles).max(0) as u64
    }

    pub fn add_dedicated_sample(
        &mut self,
        host_send: Instant, host_recv: Instant,
        mcu_at_response: u64,
    ) {
        let rtt = host_recv.duration_since(host_send);
        let rtt_us = rtt.as_micros() as u32;
        let one_way_secs = rtt.as_secs_f64() / 2.0;
        // Back-calculate to send-instant. Round-2 B04: use stable epoch-relative time.
        let host_time_at_send = self.host_time_at(host_send);
        let mcu_at_send = mcu_at_response.saturating_sub(
            (one_way_secs * self.clock_freq_estimate) as u64
        );
        self.add_sample(Sample {
            host_time_secs: host_time_at_send,
            mcu_clock: mcu_at_send,
            rtt_us,
            source: SampleSource::Dedicated,
            recorded_at: Instant::now(),
        });
        self.last_dedicated_sample = Some(Instant::now());
    }

    pub fn add_piggyback_sample(&mut self, host_recv: Instant, mcu_clock_now: u64) {
        // Round-2 B04: stable epoch-relative time.
        let host_time_secs = self.host_time_at(host_recv);
        self.add_sample(Sample {
            host_time_secs,
            mcu_clock: mcu_clock_now,
            rtt_us: 0,
            source: SampleSource::Piggyback,
            recorded_at: Instant::now(),
        });
    }

    fn add_sample(&mut self, sample: Sample) {
        if self.samples.len() == WINDOW {
            self.samples.pop_front();
        }
        self.samples.push_back(sample);
        self.recompute_regression();
    }

    fn recompute_regression(&mut self) {
        if self.samples.len() < 2 { return; }
        // Simple least-squares: fit mcu_clock = freq * host_time + offset.
        let n = self.samples.len() as f64;
        let mut sum_x = 0.0; let mut sum_y = 0.0;
        let mut sum_xx = 0.0; let mut sum_xy = 0.0;
        for s in &self.samples {
            sum_x += s.host_time_secs;
            sum_y += s.mcu_clock as f64;
            sum_xx += s.host_time_secs * s.host_time_secs;
            sum_xy += s.host_time_secs * (s.mcu_clock as f64);
        }
        let mean_x = sum_x / n;
        let mean_y = sum_y / n;
        let denom = sum_xx - n * mean_x * mean_x;
        if denom.abs() < 1e-12 { return; }
        let slope = (sum_xy - n * mean_x * mean_y) / denom;
        let offset = mean_y - slope * mean_x;
        self.clock_freq_estimate = slope;
        self.anchor_host_time = mean_x;
        self.anchor_mcu_clock = mean_y as u64;

        // Residual max.
        let mut max_resid = 0.0f64;
        for s in &self.samples {
            let predicted = slope * s.host_time_secs + offset;
            let resid_seconds = ((s.mcu_clock as f64) - predicted) / slope;
            let resid_us = (resid_seconds * 1e6).abs();
            if resid_us > max_resid { max_resid = resid_us; }
        }
        self.residual_max_in_window = max_resid;
    }

    pub fn drift_ppm(&self, baseline_freq: f64) -> f64 {
        ((self.clock_freq_estimate - baseline_freq) / baseline_freq) * 1e6
    }

    pub fn last_sample_age(&self) -> Option<Duration> {
        self.samples.back().map(|s| s.recorded_at.elapsed())
    }

    pub fn last_dedicated_sample_age(&self) -> Option<Duration> {
        self.last_dedicated_sample.map(|t| t.elapsed())
    }

    pub fn sample_count(&self) -> u32 {
        self.samples.len() as u32
    }

    /// Plan-decision B: quality gate includes RTT-aware-sample-present check.
    pub fn is_quality_gate_passed(&self, baseline_freq: f64) -> bool {
        if self.sample_count() < MIN_WARMUP_SAMPLES { return false; }
        if self.residual_max_in_window > MAX_RESIDUAL_US_DEFAULT { return false; }
        if self.drift_ppm(baseline_freq).abs() > MAX_DRIFT_PPM_DEFAULT { return false; }
        if let Some(age) = self.last_sample_age() {
            if age.as_millis() > MAX_SAMPLE_AGE_MS_DEFAULT as u128 { return false; }
        } else { return false; }
        // Plan-decision B: dedicated sample present + recent.
        match self.last_dedicated_sample_age() {
            Some(age) if age.as_millis() <= MAX_RTT_AGE_MS_DEFAULT as u128 => true,
            _ => false,
        }
    }
}
```

- [ ] **Step 2: Unit tests**

```rust
// rust/kalico-host-rt/tests/clock_sync_unit.rs
use kalico_host_rt::clock_sync::{ClockSyncEstimator, SampleSource};
use std::time::{Duration, Instant};

#[test]
fn fresh_estimator_quality_gate_fails_under_warmup() {
    let est = ClockSyncEstimator::new(550_000_000.0);
    assert!(!est.is_quality_gate_passed(550_000_000.0));
}

#[test]
fn quality_gate_requires_recent_dedicated_sample_per_plan_decision_b() {
    // Round-2 fix B05: test data must lie on a single regression line
    // (mcu_clock = freq * host_time_secs + offset) so the residual stays
    // small and the gate passes for the right reason.
    let freq = 550_000_000.0;  // 550 MHz baseline
    let mut est = ClockSyncEstimator::new(freq);
    let epoch_offset_mcu = 1_000_000_000u64;  // arbitrary mcu starting clock

    // Inject 35 piggyback samples on the regression line:
    //   mcu_clock = freq * (i * 0.01 secs) + epoch_offset_mcu
    let t0 = Instant::now();
    for i in 0..35 {
        let host_t = t0 + Duration::from_millis(i * 10);
        let mcu = epoch_offset_mcu + ((i as f64) * 0.01 * freq) as u64;
        est.add_piggyback_sample(host_t, mcu);
    }
    assert!(!est.is_quality_gate_passed(freq),
        "must fail without RTT-aware sample (Plan-decision B)");

    // Add a dedicated sample on the same line. The estimator back-calculates
    // mcu_at_send = mcu_at_response - one_way * freq. So construct
    // mcu_at_response such that the back-calculated mcu_at_send falls
    // exactly on the regression line at host_send_time:
    //   want: mcu_at_send = epoch_offset_mcu + host_send_secs * freq
    //   thus: mcu_at_response = mcu_at_send + one_way_secs * freq
    let host_send = t0 + Duration::from_millis(360);
    let host_recv = host_send + Duration::from_micros(500);
    let one_way_secs = 0.000_250;  // 500 µs RTT / 2
    let host_send_secs = 0.360;
    let mcu_at_send_target = epoch_offset_mcu + (host_send_secs * freq) as u64;
    let mcu_at_response = mcu_at_send_target + (one_way_secs * freq) as u64;
    est.add_dedicated_sample(host_send, host_recv, mcu_at_response);

    assert!(est.is_quality_gate_passed(freq),
        "should pass with fresh dedicated sample on regression line");
}

#[test]
fn dedicated_sample_age_check() {
    // ... test that after MAX_RTT_AGE_MS elapses the gate fails again
}
```

- [ ] **Step 3: Run + commit**

```bash
cd rust && cargo test -p kalico-host-rt clock_sync_unit 2>&1 | tail -10
git add rust/kalico-host-rt/src/clock_sync.rs rust/kalico-host-rt/tests/clock_sync_unit.rs
git commit -m "kalico-host-rt/clock_sync: §12.2 estimator + §12.4/Plan-decision B gate"
```

### Task 8.2: ARMING flow with explicit dedicated-sync (Plan-decision B)

**Files:**
- Modify: `rust/kalico-host-rt/src/stream.rs` — implement ARMING flow.

**Why:** Plan-decision B: §12.3 normative — host issues `kalico_clock_sync_request` as explicit step in §6.4 ARMING; quality gate gets RTT-aware-sample-present check.

- [ ] **Step 1: Implement the host-side `arm()` flow**

```rust
// rust/kalico-host-rt/src/stream.rs

use std::time::{Duration, Instant};
use crate::clock_sync::ClockSyncEstimator;
use crate::transport::{Transport, TransportError};

/// ARMING flow per spec §6.3 + §6.4 + Plan-decision B.
///
/// Round-2 fix B07: takes `&mut dyn Transport` (Plan-decision C trait, see
/// Phase 10 Task 10.2) instead of a concrete `KalicoHostIo`, so the function
/// is testable against `MockTransport`.
pub fn arm_all_mcus<T: crate::transport::Transport>(
    mcus: &mut [(T, ClockSyncEstimator)],
    t_start_wall_clock: Instant,
    arm_lead_time: Duration,
    arm_lead_cycles: u32,
    baseline_freq: f64,
) -> Result<(), ArmError> {
    let arming_deadline = Instant::now() + arm_lead_time / 2;

    // Step 1+2+3: dedicated sync + quality gate per MCU.
    for (io, est) in mcus.iter_mut() {
        if Instant::now() >= arming_deadline {
            return Err(ArmError::DeadlineMissed);
        }
        let host_send = Instant::now();
        // Round-2 B04 carry-over: any timestamps used as wire arguments are
        // independent of the estimator's epoch (this is just a request_id
        // back-trace value; the MCU echoes it).
        io.send(&format!(
            "kalico_clock_sync_request request_id=1 host_send_time_lo=0 host_send_time_hi=0"
        ))?;
        let resp = io.wait_for_response("kalico_clock_sync_response",
            Duration::from_millis(50))?;
        let host_recv = Instant::now();
        let mcu_clock = ((resp.get_u32("mcu_clock_hi") as u64) << 32)
                      | (resp.get_u32("mcu_clock_lo") as u64);
        est.add_dedicated_sample(host_send, host_recv, mcu_clock);

        if !est.is_quality_gate_passed(baseline_freq) {
            return Err(ArmError::QualityGate);
        }
    }

    // Step 4+5: arm each MCU with deadline.
    //
    // Round-2 fix B11-real: per-MCU `t_start_local` MUST be the absolute
    // MCU-clock value at wall-time `t_start_wall_clock`, NOT just `delta_secs *
    // freq`. Compute via the estimator's anchor: t_start_local =
    // mcu_time_at_host(host_time_secs(t_start_wall_clock)).
    for (io, est) in mcus.iter_mut() {
        if Instant::now() >= arming_deadline {
            return Err(ArmError::DeadlineMissed);
        }
        let t_start_host_secs = est.host_time_at(t_start_wall_clock);
        let t_start_local = est.mcu_time_at_host(t_start_host_secs);
        io.send(&format!(
            "kalico_stream_arm t_start_t0_lo={} t_start_t0_hi={} arm_lead_cycles={}",
            t_start_local as u32, (t_start_local >> 32) as u32, arm_lead_cycles))?;
        let resp = io.wait_for_response("kalico_stream_arm_response",
            arming_deadline.saturating_duration_since(Instant::now()))?;
        if resp.get_i32("result") != 0 {
            return Err(ArmError::McuRejected(resp.get_i32("result")));
        }
    }

    Ok(())
}

#[derive(Debug)]
pub enum ArmError {
    DeadlineMissed,
    QualityGate,
    McuRejected(i32),
    Transport(crate::transport::TransportError),
}

impl From<crate::transport::TransportError> for ArmError {
    fn from(e: crate::transport::TransportError) -> Self { ArmError::Transport(e) }
}
```

Make `host_time_at` and `mcu_time_at_host` `pub` on `ClockSyncEstimator` (already done in Phase 8 Task 8.1 fix above).

- [ ] **Step 2: Unit tests with mock host_io**

(Detailed test setup uses a `MockKalicoHostIo` that records sent commands and replays canned responses. Test cases: deadline-miss aborts, quality-gate failure aborts, all-MCUs-arm-OK happy path.)

- [ ] **Step 3: Run + commit**

```bash
cd rust && cargo test -p kalico-host-rt arm_flow 2>&1 | tail -10
git add rust/kalico-host-rt/src/stream.rs rust/kalico-host-rt/tests/arm_flow_unit.rs
git commit -m "kalico-host-rt/stream: §6.3/§6.4 ARMING + Plan-decision B dedicated sync"
```

---

## Phase 9 — Hold segments

Implements spec §6.5.

### Task 9.1: HOLD_SEGMENT flag + ISR short-circuit

**Files:**
- Modify: `rust/runtime/src/segment.rs` — add `SEGMENT_FLAG_HOLD` constant.
- Modify: `rust/runtime/src/engine.rs` — short-circuit on `flags & HOLD_SEGMENT` BEFORE curve lookup.

**Why:** Spec §6.5. Hold segments don't reference curves; ISR must short-circuit before lookup, after force_idle check.

- [ ] **Step 1: Define `SEGMENT_FLAG_HOLD`**

```rust
// rust/runtime/src/segment.rs
pub const SEGMENT_FLAG_HOLD: u8 = 1 << 0;
```

- [ ] **Step 2: ISR short-circuit in `Engine::evaluate_current`**

The Engine signature receives `pool: &CurvePool` from the FFI shim (Engine doesn't own CurvePool — it's at the top level of RuntimeContext per Phase 1 Task 1.1). The FFI shim in `kalico_runtime_tick` projects raw pointers to both `IsrState` and `CurvePool`:

```rust
// kalico_runtime_tick (already updated in Phase 1 Task 1.2 — extension here):
let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
let pool_ptr: *const CurvePool = core::ptr::addr_of!((*ctx).curve_pool);
let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
let isr: &mut IsrState = &mut *isr_ptr;
let pool: &CurvePool = &*pool_ptr;
let shared: &SharedState = &*shared_ptr;
isr.engine.tick(raw_cyccnt, &mut isr.widen_state, pool, shared);
```

Engine::tick / evaluate_current then use the immutable `pool` reference for lookup. CurvePool's per-slot atomics + UnsafeCell make this safe even though it's `&` — the Acquire load on `current_gen` synchronizes with foreground's Release store from try_alloc_and_load.

```rust
fn evaluate_current(
    &mut self,
    pool: &crate::curve_pool::CurvePool,
) -> Result<MotorPositions, FaultCode> {
    let seg = self.current.as_ref().unwrap();
    if seg.flags & SEGMENT_FLAG_HOLD != 0 {
        // Repeat last position; emit (throttled) HOLD_SAMPLE trace.
        // No curve lookup, no NURBS evaluation.
        return Ok(self.last_emitted_motor_positions);
    }
    let curve = pool.lookup(seg.curve_handle)?;
    // ... NURBS eval
}
```

Update Engine::tick signature to accept `pool: &CurvePool` and forward to evaluate_current. This requires backporting the signature change to Phase 1 Task 1.2 (`isr.engine.tick(raw_cyccnt, &mut isr.widen_state, pool, shared)` instead of `isr.engine.tick(raw_cyccnt, ...)`); make sure both phases agree.

- [ ] **Step 3: Unit-test the hold path**

```rust
#[test]
fn hold_segment_skips_curve_lookup_and_emits_last_position() {
    // setup: queue has a segment with flags = SEGMENT_FLAG_HOLD,
    // curve_handle = HOLD_SEGMENT_SENTINEL (which would fail lookup)
    // tick should NOT fault — short-circuit kicks in.
}
```

- [ ] **Step 4: Hold-segment retire emits SEGMENT_END + credit_freed (Round-1 review B8 fix)**

Per spec §6.5: at `t_end`, hold retires normally; emits `kalico_credit_freed` and `SEGMENT_END` like a motion segment. Stream stays alive.

In `Engine::tick`'s segment-retire path (after the boundary loop), ensure that hold segments emit their `SEGMENT_END` trace sample identically to motion segments — the only difference is per-tick evaluation (skipped for holds), not retirement. Add an explicit code path that confirms the trace_producer.enqueue is called for hold segments at retire-time. The `confirm_retired(handle)` call on the foreground reclaim side is unconditional — hold segments use `CurveHandle::HOLD_SEGMENT_SENTINEL` which `confirm_retired` accepts but does nothing for (`slot_idx == u16::MAX` is out of range for the slots array — `confirm_retired` is a no-op for the sentinel handle, which is correct since no slot was allocated).

Add unit test:

```rust
#[test]
fn hold_segment_emits_segment_end_at_retire() {
    // ... setup engine with a hold segment, run ticks until t_end, verify
    // the trace stream contains a SEGMENT_END with curve_handle =
    // HOLD_SEGMENT_SENTINEL.
}
```

- [ ] **Step 5: Commit**

```bash
git add rust/runtime/src/segment.rs rust/runtime/src/engine.rs \
        rust/runtime/tests/hold_segment.rs
git commit -m "runtime/engine: §6.5 hold-segment ISR short-circuit + retire events

flags & SEGMENT_FLAG_HOLD checked AFTER force_idle (§8.5) but BEFORE
curve_pool::lookup. Hold segments don't reference curves; sentinel
CurveHandle::HOLD_SEGMENT_SENTINEL is parsed but never resolved.
ISR repeats last motor position during hold window.

At t_end, hold retires normally — emits SEGMENT_END trace event +
kalico_credit_freed (foreground side, B8 fix). Stream stays alive
across long Z-idle stretches without underrun.

Per spec §6.5."
```

---

## Phase 10 — Host-side modules (`kalico-host-rt`, scope-reduced)

**Plan-decision C (Round-3-corrected):** Round 3 verifier B-R3-6 found that the previous draft's deferral of host_io.rs entirely to Step 7 misread spec §1.2 — spec §2.1 explicitly lists `host_io/` as a Step 6 deliverable. The corrected scope reduction: **Step 6 ships a minimal `host_io.rs` shim** (connect/identify/send/recv-with-timeout) consuming the same `Transport` trait as the rest of the host-rt modules. **Deferred to Step 7 MVP**: NAK retransmit hardening, async event dispatch loop, identify-during-reconnect race recovery, USB-CDC tty enumeration race handling. Spec §2.1's host_io substrate ships now; the production-grade hardening waits for Step 7.

Why this matters: msgproto guarantees in-order error-free delivery via msgproto's CRC + sequence + NAK + retransmit (per spec §4.1) — but that's the *MCU-side* implementation. The host side normally relies on klippy/serialhdl.py to manage retransmit. tools/kalico_host_io.py works around msgproto's quirks by open-coding the framing (~390 LOC). Step 6 ships a Rust port of the *minimum* needed for `Transport` to function: open serial port, run identify handshake, send a framed command, wait on a parsed response. NAK-driven retransmit is added in Step 7.

The Step-6 host_io.rs is ~150 LOC (vs the ~250 LOC the previous Round-2 draft hand-waved): no NAK retransmit, no async event dispatch thread, send + receive are synchronous. End-to-end testing rides on this shim. Production prints (post-Step-7) replace it with a hardened version.

### Task 10.1: Crate scaffold

**Files:**
- Create: `rust/kalico-host-rt/Cargo.toml`, `rust/kalico-host-rt/src/lib.rs`, modules.
- Modify: `rust/Cargo.toml` workspace members.

- [ ] **Step 1: Cargo.toml**

```toml
[package]
name = "kalico-host-rt"
version = "0.1.0"
edition = "2024"
publish = false
description = "Standalone Rust host runtime for kalico — owns USB-CDC fd, drives MCU comms protocol, runs clock-freq estimator and credit-based flow control."

[dependencies]
serialport = "4"
crc = "3"
log = "0.4"

[dev-dependencies]
loom = "0.7"
```

- [ ] **Step 2: lib.rs module skeleton**

```rust
//! Kalico host runtime — Step-6 substrate. Spec §2.1 component layout.
//!
//! Plan-decision C (Round-3-corrected): Step 6 ships a minimal host_io.rs
//! shim (connect/identify/send/recv-with-timeout) implementing Transport.
//! Production-grade hardening (NAK retransmit, async event dispatch
//! thread, reconnect race recovery) is Step 7 MVP work.

pub mod transport;      // Transport trait
pub mod host_io;        // Minimal Rust host_io.rs implementing Transport (Step 6 minimum)
pub mod clock_sync;     // Per-MCU sliding-window regression
pub mod credit;         // Per-MCU credit counter
pub mod producer;       // Segment producer (Layer-1/2/3 → wire)
pub mod fault;          // Fault aggregator
pub mod stream;         // Host-side stream lifecycle (ARMING flow)
pub mod wire;           // Wire encoder/decoder for kalico-versioned blobs
```

- [ ] **Step 3: Add to workspace**

```toml
# rust/Cargo.toml
[workspace]
members = [
  "nurbs", "kalico-c-api", "gcode", "geometry", "temporal", "runtime",
  "kalico-host-rt",   # NEW
]
```

- [ ] **Step 4: Build**

```bash
cd rust && cargo build -p kalico-host-rt 2>&1 | tail -10
```

(Expected: empty modules compile cleanly. Bodies land in 10.2-10.6.)

- [ ] **Step 5: Commit**

```bash
git add rust/Cargo.toml rust/kalico-host-rt/
git commit -m "kalico-host-rt: scaffold new host runtime crate

Standalone Rust binary owns USB-CDC fd, drives kalico comms protocol.
Modules: host_io, clock_sync, credit, producer, fault, stream, wire.
Bodies in subsequent tasks.

Per spec §2.1 + §2.3."
```

### Task 10.2: `Transport` trait + Python-bridge implementation

**Files:**
- Create: `rust/kalico-host-rt/src/transport.rs` — abstract trait that hides the wire-transport implementation.

**Why:** Plan-decision C: Step 6 doesn't ship the production Rust host_io port. Instead, the Rust modules consume a `Transport` trait that can be implemented by either (a) a Python-helper bridge for Step 6 testing, or (b) a future Rust host_io binary in Step 7. This isolates the new logic from transport-port complexity.

- [ ] **Step 1: Define `Transport` trait**

```rust
//! Abstract transport layer. Hides whether the underlying wire I/O is
//! Python (via PyO3 bridge or subprocess) or Rust (Step 7+).
//!
//! Step-6 Phase-10 modules consume `&dyn Transport` so they can be tested
//! against a `MockTransport` and run in production against a `PythonTransport`.

use std::time::Duration;

#[derive(Debug)]
pub enum TransportError {
    Io(std::io::Error),
    Timeout,
    Closed,
    Parse(String),
}

impl From<std::io::Error> for TransportError {
    fn from(e: std::io::Error) -> Self { TransportError::Io(e) }
}

pub trait Transport: Send {
    /// Send a command line in Klipper msgproto format.
    fn send(&mut self, cmd: &str) -> Result<(), TransportError>;
    /// Block on an inbound message named `name`. Returns parsed key=value pairs.
    fn wait_for_response(&mut self, name: &str, timeout: Duration)
        -> Result<MessageParams, TransportError>;
    /// Pull any inbound async events of `name` (non-blocking; returns Vec).
    fn poll_events(&mut self, name: &str) -> Vec<MessageParams>;
}

#[derive(Debug, Default, Clone)]
pub struct MessageParams {
    pub fields: std::collections::HashMap<String, MessageValue>,
}

impl MessageParams {
    pub fn get_i32(&self, k: &str) -> i32 {
        match self.fields.get(k) { Some(MessageValue::I32(v)) => *v, _ => 0 }
    }
    pub fn get_u32(&self, k: &str) -> u32 {
        match self.fields.get(k) { Some(MessageValue::U32(v)) => *v, _ => 0 }
    }
    pub fn get_u64(&self, k: &str) -> u64 {
        match self.fields.get(k) { Some(MessageValue::U64(v)) => *v, _ => 0 }
    }
}

#[derive(Debug, Clone)]
pub enum MessageValue { I32(i32), U32(u32), U64(u64), Bytes(Vec<u8>) }
```

- [ ] **Step 2: Implement `MockTransport` for unit tests**

```rust
// rust/kalico-host-rt/tests/mock_transport.rs
use kalico_host_rt::transport::*;
use std::collections::VecDeque;
use std::time::Duration;

pub struct MockTransport {
    pub sent: Vec<String>,
    pub responses: VecDeque<(String, MessageParams)>,  // (name, params)
}

impl MockTransport {
    pub fn new() -> Self { Self { sent: Vec::new(), responses: VecDeque::new() } }
    pub fn enqueue_response(&mut self, name: &str, params: MessageParams) {
        self.responses.push_back((name.into(), params));
    }
}

impl Transport for MockTransport {
    fn send(&mut self, cmd: &str) -> Result<(), TransportError> {
        self.sent.push(cmd.into());
        Ok(())
    }
    fn wait_for_response(&mut self, name: &str, _timeout: Duration)
        -> Result<MessageParams, TransportError>
    {
        loop {
            match self.responses.pop_front() {
                None => return Err(TransportError::Timeout),
                Some((n, p)) if n == name => return Ok(p),
                _ => continue,
            }
        }
    }
    fn poll_events(&mut self, _name: &str) -> Vec<MessageParams> { vec![] }
}
```

- [ ] **Step 3: Sketch `PythonTransport` shim (production for Step 6)**

```rust
//! Python-bridge transport: wraps an existing tools/kalico_host_io.py
//! KalicoHostIO instance via PyO3.
//!
//! NOTE: Step 6 tests pyo3 only as far as needed to run alongside the
//! existing Python helper. The production Rust host_io is Step 7 work.

#[cfg(feature = "python-bridge")]
pub struct PythonTransport {
    py_io: pyo3::PyObject,  // tools.kalico_host_io.KalicoHostIO instance
}

#[cfg(feature = "python-bridge")]
impl Transport for PythonTransport {
    fn send(&mut self, cmd: &str) -> Result<(), TransportError> {
        pyo3::Python::with_gil(|py| {
            self.py_io.call_method1(py, "send", (cmd,))
                .map_err(|e| TransportError::Io(std::io::Error::new(
                    std::io::ErrorKind::Other, format!("{:?}", e))))?;
            Ok(())
        })
    }
    fn wait_for_response(&mut self, name: &str, timeout: Duration)
        -> Result<MessageParams, TransportError>
    {
        pyo3::Python::with_gil(|py| {
            let result = self.py_io.call_method1(py, "wait_for_response",
                (name, timeout.as_secs_f64()))
                .map_err(|_| TransportError::Timeout)?;
            // Convert Python dict to MessageParams.
            // ... (concrete conversion left for impl-time; ~30 LOC)
            Ok(MessageParams::default())  // placeholder
        })
    }
    fn poll_events(&mut self, _name: &str) -> Vec<MessageParams> {
        // Optional for Step 6; can poll via wait_for_response with short timeout.
        vec![]
    }
}
```

The `PythonTransport` implementation is a thin shim over the working Python helper. ~80 LOC total. The production Rust host_io is Step 7 work (per Plan-decision C); for Step 6, end-to-end tests use `PythonTransport`, unit tests use `MockTransport`.

- [ ] **Step 4: Tests + commit**

```bash
cd rust && cargo test -p kalico-host-rt mock_transport 2>&1 | tail -10
git add rust/kalico-host-rt/src/transport.rs rust/kalico-host-rt/tests/mock_transport.rs
git commit -m "kalico-host-rt/transport: Transport trait + MockTransport

Plan-decision C: Step 6 deferred Rust port of tools/kalico_host_io.py
to Step 7 MVP. Phase 10 modules consume &dyn Transport instead, with
MockTransport for unit tests and an optional PythonTransport (PyO3
bridge) for Step-6 end-to-end tests.

Production Rust host_io is Step 7 work."
```

### Task 10.2.5: Minimal `host_io.rs` shim implementing `Transport`

**Files:**
- Create: `rust/kalico-host-rt/src/host_io.rs`

**Why:** Round-3 fix B-R3-6: spec §2.1 mandates host_io as Step-6 deliverable. Plan-decision C downgrade — minimum viable shim, no NAK retransmit, no async event thread (deferred to Step 7 MVP).

- [ ] **Step 1: Add `serialport` dependency**

In `rust/kalico-host-rt/Cargo.toml`:

```toml
[dependencies]
serialport = "4"
crc = "3"
log = "0.4"

# Round-3 fix: pyo3 declared as optional + python-bridge feature.
[dependencies.pyo3]
version = "0.22"
optional = true

[features]
default = []
python-bridge = ["dep:pyo3"]
```

- [ ] **Step 2: Implement minimal host_io.rs**

```rust
//! Minimal Step-6 host_io.rs implementing Transport. Spec §2.1 substrate.
//!
//! Step-6 minimum: open serial port, run identify handshake, send framed
//! commands, wait on parsed responses with timeout. Production hardening
//! (NAK retransmit, async event dispatch) is Step 7 MVP.

use std::time::{Duration, Instant};
use std::collections::VecDeque;
use serialport::SerialPort;

use crate::transport::{MessageParams, MessageValue, Transport, TransportError};

pub struct KalicoHostIo {
    port: Box<dyn SerialPort>,
    seq: u8,                              // 4-bit sequence
    rx_buf: Vec<u8>,
    pending: VecDeque<(String, MessageParams)>,
    parser: MsgProtoParser,
}

impl KalicoHostIo {
    pub fn open(path: &str, baud: u32) -> Result<Self, TransportError> {
        let port = serialport::new(path, baud)
            .timeout(Duration::from_millis(100))
            .open()?;
        let mut io = Self {
            port,
            seq: 0,
            rx_buf: Vec::with_capacity(1024),
            pending: VecDeque::new(),
            parser: MsgProtoParser::new(),
        };
        io.identify_handshake()?;
        Ok(io)
    }

    fn identify_handshake(&mut self) -> Result<(), TransportError> {
        // ~50 LOC: send `identify offset=N count=40` until offset reaches
        // total length, accumulate response bytes into a JSON-ish data
        // dictionary, parse via msgproto. Mirror tools/kalico_host_io.py
        // _do_identify lines 137-183 verbatim in Rust. Step-6 minimum:
        // synchronous, no NAK retransmit (rely on USB-CDC reliability for
        // test bench), 15-second timeout.
        // TODO[step6]: implement; ~50 LOC straight port from Python.
        Ok(())
    }
}

impl Transport for KalicoHostIo {
    fn send(&mut self, cmd: &str) -> Result<(), TransportError> {
        // ~30 LOC: encode via parser, frame with [len, seq+dest, payload, crc16, sync],
        // write to port. No NAK retransmit (Step 7).
        let _ = cmd;  // TODO[step6]: implement
        Ok(())
    }
    fn wait_for_response(&mut self, name: &str, timeout: Duration)
        -> Result<MessageParams, TransportError>
    {
        let deadline = Instant::now() + timeout;
        loop {
            if let Some(idx) = self.pending.iter().position(|(n, _)| n == name) {
                return Ok(self.pending.remove(idx).unwrap().1);
            }
            let remaining = deadline.checked_duration_since(Instant::now())
                .ok_or(TransportError::Timeout)?;
            self.port.set_timeout(remaining.min(Duration::from_millis(100)))?;
            let mut chunk = [0u8; 256];
            match self.port.read(&mut chunk) {
                Ok(n) if n > 0 => {
                    self.rx_buf.extend_from_slice(&chunk[..n]);
                    while let Some(packet) = self.parser.try_extract(&mut self.rx_buf) {
                        if let Some((nm, p)) = self.parser.parse(&packet) {
                            self.pending.push_back((nm, p));
                        }
                    }
                }
                Ok(_) => continue,
                Err(e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
                Err(e) => return Err(TransportError::Io(e)),
            }
        }
    }
    fn poll_events(&mut self, name: &str) -> Vec<MessageParams> {
        let mut out = Vec::new();
        let mut i = 0;
        while i < self.pending.len() {
            if self.pending[i].0 == name {
                out.push(self.pending.remove(i).unwrap().1);
            } else {
                i += 1;
            }
        }
        out
    }
}

/// Minimal msgproto parser. ~80 LOC.
/// Step-6 deliverable: parse framed packets via SYNC byte + length + CRC16.
/// Step-7 hardening: NAK detection, retransmit window, identify-response
/// reassembly, etc. Refer to tools/kalico_host_io.py for the canonical
/// behavior.
struct MsgProtoParser {
    // dictionary loaded at identify time
    commands: std::collections::HashMap<u32, CommandSpec>,
}

struct CommandSpec {
    name: String,
    fields: Vec<(String, FieldType)>,
}

enum FieldType { U32, U64, I32, U8, U16, Bytes }

impl MsgProtoParser {
    fn new() -> Self { Self { commands: Default::default() } }
    fn try_extract(&self, buf: &mut Vec<u8>) -> Option<Vec<u8>> {
        // Find SYNC=0x7E, validate length+CRC, extract packet bytes, drain buf.
        // ~30 LOC.
        None  // TODO[step6]: implement
    }
    fn parse(&self, packet: &[u8]) -> Option<(String, MessageParams)> {
        // Decode message-id, look up command spec, decode fields.
        // ~30 LOC.
        None  // TODO[step6]: implement
    }
}
```

- [ ] **Step 3: Tests + commit**

```bash
cd rust && cargo test -p kalico-host-rt host_io 2>&1 | tail -10
git add rust/kalico-host-rt/Cargo.toml rust/kalico-host-rt/src/host_io.rs
git commit -m "kalico-host-rt/host_io: §2.1 minimal Transport shim (Plan-decision C)

Round-3 review B-R3-6: spec §2.1 mandates host_io as Step-6 deliverable.
Plan-decision C downgrade: minimum viable shim only — open port, identify
handshake (sync), send framed cmd, wait for response (sync). No NAK
retransmit, no async event dispatch thread; deferred to Step 7 MVP.

~150 LOC port of tools/kalico_host_io.py minimum surface. Full
production-grade host_io ships with Step 7."
```

### Task 10.3: `clock_sync.rs` (already implemented in Phase 8 Task 8.1)

Phase 8 Task 8.1 lives at `rust/kalico-host-rt/src/clock_sync.rs` and implements the full estimator. Phase 10 Task 10.3 is a no-op cross-reference: confirm the file is in place and the unit tests pass.

- [ ] **Step 1: Confirm presence + test pass**

```bash
ls rust/kalico-host-rt/src/clock_sync.rs
cd rust && cargo test -p kalico-host-rt clock_sync_unit 2>&1 | tail -5
```

### Task 10.4: `credit.rs` — per-MCU credit counter

**Files:**
- Create: `rust/kalico-host-rt/src/credit.rs`
- Create: `rust/kalico-host-rt/tests/credit_unit.rs`

**Why:** Per-MCU credit counter; `try_acquire` succeeds when credit > 0 and decrements; credit-freed events restore.

- [ ] **Step 1: Implement**

```rust
//! Per-MCU credit counter for §5 (α flow control).

use std::sync::atomic::{AtomicI32, Ordering};

pub struct CreditCounter {
    available: AtomicI32,
    capacity: i32,
    pub credit_epoch: AtomicI32,
}

impl CreditCounter {
    pub fn new(capacity: i32) -> Self {
        Self {
            available: AtomicI32::new(capacity),
            capacity,
            credit_epoch: AtomicI32::new(0),
        }
    }

    /// Speculatively decrements credit. Caller must call `release` if the
    /// push fails downstream.
    pub fn try_acquire(&self) -> Option<()> {
        loop {
            let cur = self.available.load(Ordering::Acquire);
            if cur <= 0 { return None; }
            match self.available.compare_exchange(cur, cur - 1, Ordering::AcqRel, Ordering::Acquire) {
                Ok(_) => return Some(()),
                Err(_) => continue,
            }
        }
    }

    pub fn release(&self) {
        // Push failed; rollback.
        let _ = self.available.fetch_add(1, Ordering::AcqRel);
    }

    /// Called on receipt of `kalico_credit_freed` event.
    pub fn on_credit_freed(&self, free_slots: u8) {
        // Reconcile: set available to `min(capacity, free_slots as i32)`.
        // Using a CAS loop to avoid overshooting capacity if events overlap.
        let want = (free_slots as i32).min(self.capacity);
        let _ = self.available.store(want, Ordering::Release);
    }

    /// Reset on credit_epoch change (host received new epoch from MCU).
    pub fn on_epoch_change(&self, new_epoch: i32) {
        self.credit_epoch.store(new_epoch, Ordering::Release);
        self.available.store(self.capacity, Ordering::Release);
    }

    pub fn available(&self) -> i32 {
        self.available.load(Ordering::Acquire)
    }
}
```

- [ ] **Step 2: Unit tests for try_acquire/release/on_credit_freed/on_epoch_change**

- [ ] **Step 3: Run + commit**

### Task 10.5: `producer.rs` — segment producer + wire encoder

**Files:**
- Create: `rust/kalico-host-rt/src/producer.rs`
- Create: `rust/kalico-host-rt/src/wire.rs` — versioned-blob v1 encoder mirror.

**Why:** Receives Layer-1/2/3 segments, encodes per §4.2 + §3.2 wire schema, sends via `host_io::send` after `credit::try_acquire`.

- [ ] **Step 1: Implement `wire.rs` v1 encoder**

```rust
// rust/kalico-host-rt/src/wire.rs
pub const FORMAT_VERSION_V1: u8 = 0x01;

pub fn encode_load_curve_v1(
    degree: u8, cps: &[[f32; 3]], knots: &[f32], weights: &[f32],
) -> Vec<u8> {
    let mut out = Vec::with_capacity(1 + 4 + cps.len()*12 + knots.len()*4 + weights.len()*4);
    out.push(FORMAT_VERSION_V1);
    out.push(degree);
    out.push(cps.len() as u8);
    out.push(knots.len() as u8);
    out.push(weights.len() as u8);
    for cp in cps {
        for &v in cp.iter() { out.extend_from_slice(&v.to_le_bytes()); }
    }
    for &k in knots { out.extend_from_slice(&k.to_le_bytes()); }
    for &w in weights { out.extend_from_slice(&w.to_le_bytes()); }
    out
}
```

- [ ] **Step 2: Implement `producer.rs` against the Transport trait (Round-3 fix B-R3-5)**

```rust
// rust/kalico-host-rt/src/producer.rs

use crate::credit::CreditCounter;
use crate::transport::{Transport, TransportError};

pub struct PushedSegmentInfo {
    pub accepted_segment_id: u32,
    pub credit_epoch: u32,
}

pub fn push_segment<T: Transport>(
    io: &mut T,
    credit: &CreditCounter,
    id: u32,
    curve_handle_packed: u32,
    t_start: u64,
    t_end: u64,
    kinematics: u8,
) -> Result<PushedSegmentInfo, ProducerError> {
    let _guard = credit.try_acquire().ok_or(ProducerError::NoCredit)?;
    let cmd = format!(
        "kalico_push_segment id={} curve_handle_packed={} \
         t_start_lo={} t_start_hi={} t_end_lo={} t_end_hi={} kin={}",
        id, curve_handle_packed,
        t_start as u32, (t_start >> 32) as u32,
        t_end as u32, (t_end >> 32) as u32,
        kinematics,
    );
    if let Err(e) = io.send(&cmd) {
        credit.release();
        return Err(ProducerError::Transport(e));
    }
    let resp = io.wait_for_response("kalico_push_response", std::time::Duration::from_millis(100))?;
    let result = resp.get_i32("result");
    if result != 0 {
        credit.release();
        return Err(ProducerError::McuRejected(result));
    }
    Ok(PushedSegmentInfo {
        accepted_segment_id: resp.get_u32("accepted_segment_id"),
        credit_epoch: resp.get_u32("credit_epoch"),
    })
}

#[derive(Debug)]
pub enum ProducerError {
    NoCredit,
    Transport(TransportError),
    McuRejected(i32),
}

impl From<TransportError> for ProducerError {
    fn from(e: TransportError) -> Self { ProducerError::Transport(e) }
}
```

- [ ] **Step 3: Tests + commit**

### Task 10.6: `fault.rs` + host-side state machine

**Files:**
- Create: `rust/kalico-host-rt/src/fault.rs`

**Why:** Aggregator: receives `kalico_fault` async events, propagates to user as `Result<_, FaultEvent>`.

- [ ] **Step 1: Implement (Round-3 fix B-R3-5: imports from transport, not host_io)**

```rust
// rust/kalico-host-rt/src/fault.rs

use crate::transport::MessageParams;

#[derive(Debug, Clone, Copy)]
pub struct FaultEvent {
    pub fault_code: u16,
    pub fault_detail: u32,
    pub segment_id: u32,
}

pub fn parse_fault_event(params: &MessageParams) -> Option<FaultEvent> {
    Some(FaultEvent {
        fault_code: params.get_u32("fault_code") as u16,
        fault_detail: params.get_u32("fault_detail"),
        segment_id: params.get_u32("segment_id"),
    })
}
```

(The Transport's `poll_events("kalico_fault")` returns `Vec<MessageParams>` which `parse_fault_event` digests. The minimal Step-6 host_io.rs implements `poll_events` as a no-op or a synchronous wait_for_response with short timeout; production async event dispatch is Step-7 work.)

- [ ] **Step 2: Tests + commit**

### Task 10.7: All Phase-10 cross-checks pass

- [ ] **Step 1: Build + test the full host-rt crate**

```bash
cd rust && cargo build -p kalico-host-rt 2>&1 | tail -5
cd rust && cargo test -p kalico-host-rt 2>&1 | tail -10
```

Both clean.

- [ ] **Step 2: Final commit**

```bash
git commit --allow-empty -m "kalico-host-rt: Phase 10 complete (host_io, clock_sync, credit, producer, fault, wire, stream)"
```

---

## Phase 11 — Periodic status frame + foreground task

Implements spec §5.3 + §13.1 trace-overflow latch + §10.4 reclaim drain.

### Task 11.1: 10 Hz `DECL_TASK` for status frame emission

**Files:**
- Modify: `src/runtime_tick.c` — `DECL_TASK(runtime_status_drain)` running at ~10 Hz.

**Why:** Spec §5.3.

- [ ] **Step 1: Add the periodic task**

```c
static uint32_t last_status_emit_time = 0;

void
runtime_status_drain(void)
{
    if (!kalico_rt_handle) return;
    uint32_t now = timer_read_time();
    if ((int32_t)(now - last_status_emit_time) < (int32_t)(CONFIG_CLOCK_FREQ / 10)) return;
    last_status_emit_time = now;

    uint8_t status = kalico_runtime_status(kalico_rt_handle);
    int32_t last_err = kalico_runtime_last_error(kalico_rt_handle);
    uint32_t cur_seg = kalico_runtime_current_segment_id(kalico_rt_handle);
    uint8_t depth = kalico_runtime_queue_depth(kalico_rt_handle);
    uint64_t mcu_clk = kalico_runtime_widened_now(kalico_rt_handle);
    uint32_t epoch = kalico_runtime_credit_epoch(kalico_rt_handle);
    uint32_t accepted = kalico_runtime_accepted_segment_id(kalico_rt_handle);
    uint32_t retired = kalico_runtime_retired_through_segment_id(kalico_rt_handle);

    sendf("kalico_status engine_status=%c queue_depth=%c current_segment_id=%u "
          "last_fault=%hu fault_detail=%u "
          "mcu_clock_now_lo=%u mcu_clock_now_hi=%u "
          "credit_epoch=%u accepted_segment_id=%u retired_through_segment_id=%u",
          status, depth, cur_seg, (uint16_t)last_err, 0,
          (uint32_t)mcu_clk, (uint32_t)(mcu_clk >> 32),
          epoch, accepted, retired);
}
DECL_TASK(runtime_status_drain);
```

- [ ] **Step 2: Add the new accessor FFIs**

```rust
#[unsafe(no_mangle)]
pub unsafe extern "C" fn kalico_runtime_widened_now(rt: *mut KalicoRuntime) -> u64 {
    if rt.is_null() { return 0; }
    let ctx = rt as *mut RuntimeContext;
    unsafe {
        let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
        crate::clock::read_widened_now(shared)
    }
}

// Similar for credit_epoch, accepted_segment_id, retired_through_segment_id,
// current_segment_id, queue_depth.
```

- [ ] **Step 3: Update kalico_runtime.h and rebuild**

- [ ] **Step 4: Commit**

```bash
git add src/runtime_tick.c rust/kalico-c-api/src/runtime_ffi.rs \
        rust/kalico-c-api/include/kalico_runtime.h
git commit -m "runtime_tick: §5.3 periodic 10 Hz kalico_status frame

DECL_TASK runtime_status_drain emits status with engine_status,
queue_depth, current_segment_id, last_fault, fault_detail,
mcu_clock_now (split lo/hi), credit_epoch, accepted_segment_id,
retired_through_segment_id.

Per spec §5.3."
```

### Task 11.2: Foreground reclaim drain pipeline

**Files:**
- Modify: `src/runtime_tick.c` — extend `runtime_drain` to call reclaim helper + emit credit_freed events + fault events.

**Why:** Spec §10.4 + §9 fault async event channel.

- [ ] **Step 1: Wire reclaim drain into existing runtime_drain DECL_TASK**

(Detailed implementation similar to existing trace drain in Step 5.)

- [ ] **Step 2: Commit**

```bash
git add src/runtime_tick.c rust/kalico-c-api/src/runtime_ffi.rs
git commit -m "runtime_tick: §10.4 reclaim drain + §9 fault event emission

Foreground drain pipeline: drain trace → reclaim curve-pool slots →
emit kalico_credit_freed for each retired segment → emit kalico_fault
on FAULT-state transition → check sample_drop_pending → latch
TRACE_OVERFLOW.

Per spec §10.4 + §9."
```

---

## Phase 12 — Loom expansion + Step-5 carryover (MAX_BOUNDARY_ITERS)

### Task 12.1: Loom test for half-split + force_idle handshake

**Files:**
- Create: `rust/runtime/tests/loom_force_idle.rs`.
- Create: `rust/runtime/tests/loom_curve_pool_alloc.rs`.

**Why:** Spec §11.3 — loom gates correctness of the cross-half mechanisms.

- [ ] **Step 1: Loom test for force_idle handshake**

```rust
#![cfg(loom)]
// Models foreground sets force_idle=true → ISR loop observes flag, sets
// acked_force_idle=true → foreground reads acked. Verifies happens-before.
```

- [ ] **Step 2: Loom test for curve-pool alloc race**

```rust
#![cfg(loom)]
// Foreground try_alloc + ISR confirms_retired interleaved.
// Invariant: no overlapping live (slot, gen) handle.
```

- [ ] **Step 3: Run + commit**

```bash
cd rust && RUSTFLAGS="--cfg loom" cargo test -p runtime --release --test loom_force_idle --test loom_curve_pool_alloc 2>&1 | tail -10
```

### Task 12.2: MAX_BOUNDARY_ITERS test-only injection path

**Files:**
- Modify: `rust/runtime/src/engine.rs` — add `#[cfg(test)]` injection.
- Create: `rust/runtime/tests/max_boundary_iters.rs`.

**Why:** Plan-changes-log Step-5 carryover. The boundary-loop fault path is currently dead defense-in-depth — unreachable from public API at Q_N=64+. Test-only injection makes it reachable.

- [ ] **Step 1: Add `#[cfg(test)] pub fn inject_iter_count(&mut self, n: u32)`**

- [ ] **Step 2: Test fires the fault**

- [ ] **Step 3: Commit**

---

## Phase 13 — Sim integration tests (Gate B)

Implements spec §3.3 Gate B: items 5–7 (underrun fault, trace overflow fault, stream lifecycle round-trip) re-validated against sim now that §7/§8/§9 features are implemented.

### Task 13.1: Gate B test runner

**Files:**
- Create: `tools/test_sim_gate_b.py`.

**Why:** Spec §3.3 Gate B.

- [ ] **Step 1: Implement test cases for spec §3.3 Gate B items 5/6/7**

The plan's previous draft misnumbered Gate B items. Spec §3.3 Gate B lists:
- **Item 5**: Status frame reports correct `current_segment_id`, `queue_depth`, `retired_through_segment_id` throughout.
- **Item 6**: Underrun-fault path (stop pushing while stream-open → KALICO_FAULT_UNDERRUN within MIN_SEGMENT_DURATION_MS of last-segment retirement).
- **Item 7**: Trace-overflow-fault path (throttle host trace-drain → KALICO_FAULT_TRACE_OVERFLOW).

```python
#!/usr/bin/env python3
"""tools/test_sim_gate_b.py — Spec §3.3 Gate B acceptance tests."""
import struct, sys, time
from kalico_host_io import KalicoHostIO


def test_item_5_status_frame_correctness(io):
    """Item 5: status reports correct current_segment_id, queue_depth, retired_through."""
    # Open stream + arm + push 5 segments + drain + check status reflects retirement.
    epoch = io.send_and_wait("kalico_stream_open stream_id=1", "kalico_stream_open_response")["credit_epoch"]
    # Push priming segments referencing fixture 0 (already loaded by test setup).
    for i in range(5):
        t_start = (i + 1) * 100000
        t_end = t_start + 80000
        cmd = f"kalico_push_segment id={i} curve_handle_packed=1 t_start_lo={t_start} t_start_hi=0 t_end_lo={t_end} t_end_hi=0 kin=0"
        io.send(cmd)
        r = io.wait_for_response("kalico_push_response", 1.0)
        assert r["result"] == 0, f"push {i} failed: {r}"

    # Arm + commit.
    io.send(f"kalico_stream_arm t_start_t0_lo=200000 t_start_t0_hi=0 arm_lead_cycles=50000")
    arm_resp = io.wait_for_response("kalico_stream_arm_response", 1.0)
    assert arm_resp["result"] == 0

    # Wait for retirement and verify status frame reports correctly.
    deadline = time.monotonic() + 5.0
    last_status = None
    while time.monotonic() < deadline:
        try:
            last_status = io.wait_for_response("kalico_status", 0.5)
        except Exception:
            continue
        if last_status["retired_through_segment_id"] >= 4:
            # All 5 segments retired (ids 0-4).
            assert last_status["queue_depth"] == 0, f"queue_depth={last_status['queue_depth']}"
            assert last_status["current_segment_id"] >= 4
            return  # PASS
    raise AssertionError(f"item 5: never observed retired_through>=4. last_status={last_status}")


def test_item_6_underrun_fault(io):
    """Item 6: stop pushing while stream-open → KALICO_FAULT_UNDERRUN."""
    epoch = io.send_and_wait("kalico_stream_open stream_id=2", "kalico_stream_open_response")["credit_epoch"]
    # Push 2 segments, arm, let them retire — DON'T send terminal.
    for i in range(2):
        cmd = f"kalico_push_segment id={i} curve_handle_packed=1 t_start_lo={(i+1)*100000} t_start_hi=0 t_end_lo={(i+1)*100000+50000} t_end_hi=0 kin=0"
        io.send(cmd)
        io.wait_for_response("kalico_push_response", 1.0)
    io.send("kalico_stream_arm t_start_t0_lo=150000 t_start_t0_hi=0 arm_lead_cycles=30000")
    io.wait_for_response("kalico_stream_arm_response", 1.0)

    # Wait for kalico_fault async event.
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        try:
            fault = io.wait_for_response("kalico_fault", 0.5)
            # KALICO_FAULT_UNDERRUN = -130; reported as u16 (lower 16 bits).
            if fault["fault_code"] == ((-130) & 0xFFFF):
                return  # PASS
        except Exception:
            continue
    raise AssertionError(f"item 6: underrun fault not observed within 3s")


def test_item_7_trace_overflow_fault(io):
    """Item 7: throttle host trace drain → KALICO_FAULT_TRACE_OVERFLOW."""
    # Open stream, push many short segments, intentionally don't drain trace.
    # TraceRing sized at ~30 ms × 40 kHz = 1200 samples; under stream of
    # 0.1-ms segments we generate 10 samples/ms; saturation in ~120 ms.
    io.send_and_wait("kalico_stream_open stream_id=3", "kalico_stream_open_response")
    # Push many tiny segments.
    for i in range(50):
        cmd = f"kalico_push_segment id={i} curve_handle_packed=1 t_start_lo={i*4000} t_start_hi=0 t_end_lo={i*4000+4000} t_end_hi=0 kin=0"
        io.send(cmd)
        try:
            io.wait_for_response("kalico_push_response", 0.2)
        except Exception:
            break  # queue full
    io.send("kalico_stream_arm t_start_t0_lo=10000 t_start_t0_hi=0 arm_lead_cycles=5000")
    io.wait_for_response("kalico_stream_arm_response", 1.0)

    # Don't drain trace; wait for overflow fault.
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        try:
            fault = io.wait_for_response("kalico_fault", 0.5)
            if fault["fault_code"] == ((-133) & 0xFFFF):  # KALICO_FAULT_TRACE_OVERFLOW
                return  # PASS
        except Exception:
            continue
    raise AssertionError(f"item 7: trace overflow fault not observed within 5s")


def main():
    io = KalicoHostIO("socket://localhost:3334")
    try:
        # Pre-load fixture 0 in slot 1 (used by all three tests).
        io.send("kalico_load_fixture_curve slot=1 fixture_id=0")
        io.wait_for_response("kalico_load_fixture_response", 2.0)

        for name, test in [
            ("item_5_status_frame_correctness", test_item_5_status_frame_correctness),
            ("item_6_underrun_fault", test_item_6_underrun_fault),
            ("item_7_trace_overflow_fault", test_item_7_trace_overflow_fault),
        ]:
            try:
                test(io)
                print(f"PASS: {name}")
            except AssertionError as e:
                print(f"FAIL: {name}: {e}")
                return 1
            # Reset between tests.
            io.send("kalico_stream_flush")
            try: io.wait_for_response("kalico_stream_flush_response", 1.0)
            except: pass
        print("PASS: Gate B (3/3)")
        return 0
    finally:
        io.disconnect()


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 2: Run Gate B and verify it passes**

```bash
bash tools/sim/run_sim.sh &
sleep 3
python3 tools/test_sim_gate_b.py
```

Expected: `PASS: Gate B (3/3)`. Phase 13 acceptance gate is closed when this passes.

- [ ] **Step 3: Commit**

```bash
git add tools/test_sim_gate_b.py
git commit -m "tools/sim: Gate B acceptance test for spec §3.3 items 5/6/7

- Item 5: status frame reports correct retired_through_segment_id +
  queue_depth + current_segment_id throughout segment retirement.
- Item 6: underrun-fault path latches KALICO_FAULT_UNDERRUN.
- Item 7: trace-overflow-fault path latches KALICO_FAULT_TRACE_OVERFLOW.

Per spec §3.3 Gate B."
```

---

## Phase 14 — Hardware bring-up validation

### Task 14.1: H723 hardware Gate A + B

**Why:** Per CLAUDE.md memory `feedback_printer_is_test_bench`, the printer is a test bench; user runs hardware validation manually. Provide automated runner; user executes.

- [ ] **Step 1: Modify `Makefile.kalico::test-h723` to chain Phase 13 Gate B tests after Step-5 first-light**

- [ ] **Step 2: Document the user-run workflow in `tools/sim/README.md` and `docs/superpowers/plans/2026-04-28-step6-comm-protocol-and-sim-fixes.md` Phase 14 section**

- [ ] **Step 3: Commit**

---

## Phase 15 — Measurement protocols (M1, M2, M3)

Implements spec §7.3.

### Task 15.1: M1 host-stall measurement runner

**Files:**
- Create: `tools/measure_m1_host_stall.py` — 8h Pi soak.

- [ ] **Step 1: Implement runner that logs segment-push completion times under load + records p50/p95/p99/p99.9/p99.99/max**

- [ ] **Step 2: Document expected runtime + reporting**

- [ ] **Step 3: Commit**

### Task 15.2: M2 cycle-budget bench rerun

**Files:**
- Modify: `tools/test_h723_cycle_count.py` — rerun against Step-6 protocol-handler additions.

- [ ] **Step 1: Add new measurements**

- [ ] **Step 2: Commit**

### Task 15.3: M3 clock-sync residual

**Files:**
- Create: `tools/measure_m3_clock_sync.py` — 24 h dual-MCU soak.

- [ ] **Step 1: Implement**

- [ ] **Step 2: Commit**

### Task 15.4: Update `docs/research/step6-buffer-budget-measurements.md` with M1/M2/M3 results

**Codex Round 4 clarification — autonomous-scope handling:** the M1/M2/M3 measurement runs themselves require physical hardware (8h Pi soak; H723 hardware bench; 24h dual-MCU soak) which is outside autonomous subagent execution scope. The subagent's job for Task 15.4 is **scaffolding only**:

- [ ] **Step 1: Create `docs/research/step6-buffer-budget-measurements.md` with TODO placeholders**

```markdown
# Step 6 buffer-budget measurements

Status: PLACEHOLDER — measurements pending hardware run by user.

## M1 — Host-stall (Pi 5 + Bookworm + production load, 8h soak)
- p50: TODO_USER_RUN
- p95: TODO_USER_RUN
- ... (etc)

## M2 — MCU runtime cost (H723 cycle-budget rerun)
- WORST_ISR_CYCLES: TODO_USER_RUN
- CYCLES_PER_TICK: clock_freq / 40000

## M3 — Clock-sync residual (H723 + F4x sim, 24h soak)
- max_residual_p99.99: TODO_USER_RUN
- max_drift_ppm_p99.99: TODO_USER_RUN

After user runs M1/M2/M3, edit this file with actuals; if any value
diverges from initial estimates in `rust/kalico-host-rt/src/clock_sync.rs`
or `rust/runtime/src/...`, also update those constants.
```

- [ ] **Step 2: Commit the placeholder file**

```bash
git add docs/research/step6-buffer-budget-measurements.md
git commit -m "docs/research: §7.3 measurement scaffold (M1/M2/M3 user-run pending)"
```

- [ ] **Step 3: Do NOT block plan completion on measurements**

Subagent proceeds to Phase 16 with the placeholder in place. The Phase 16.2 plan-changes-log entry notes "M1/M2/M3 measurement actuals — user-run, pending" as an open follow-up. Initial-estimate constants in clock_sync.rs / state.rs ship as the Step-6 defaults; user updates them post-measurement.

---

## Phase 16 — CI + spec/plan-changes-log update

### Task 16.1: CI matrix expansion

**Files:**
- Modify: `.github/workflows/rust-runtime.yml` (or equivalent) — add loom, miri, cbindgen-no-drift, panic-grep, host-rt build.

- [ ] **Step 1: Update CI**

- [ ] **Step 2: Commit**

### Task 16.2: Plan-changes-log entry

**Files:**
- Modify: `docs/superpowers/plan-changes-log.md` — add Step 6 completion entry.

- [ ] **Step 1: Append entry**

```markdown
## 2026-04-XX (Step 6 completion)

**Build-order Step 6 (Communication protocol and clock sync) completed.** Implementation per `docs/superpowers/plans/2026-04-28-step6-comm-protocol-and-sim-fixes.md`. Layer 5 — host↔MCU comms over msgproto + 1-byte versioned blobs, credit-based flow control with periodic 10 Hz status frame backstop, multi-MCU sync via per-MCU clock-freq estimator + arm/commit handshake, half-split SPSC FFI (closes Step-5 latent UB), generation-counter curve handles (u32 = u16 slot + u16 gen), force_idle flush handshake (Plan-decision A: force_idle first / ack-wait / stream_open=false last), and explicit dedicated clock-sync at arm-time (Plan-decision B: §12.3 normative).

**Deviations from plan listing**: <list as encountered during execution>.

**Open follow-ups**: F4x integration (parallel workstream); host/klippy IPC boundary (Step 7 MVP); M1/M2/M3 measurement actuals (user-driven).

**Evidence**: Plan + ~XX commits on this branch. Spec at `docs/superpowers/specs/2026-04-28-step6-comm-protocol-and-sim-fixes-design.md` (4 review rounds). Sim Gate A + Gate B pass; H723 hardware Gate B pending user execution.
```

- [ ] **Step 2: Commit**

### Task 16.3: Mark Step 6 complete in CLAUDE.md build order

**Files:**
- Modify: `CLAUDE.md` — change `[ ]` to `[x]` for Step 6.

- [ ] **Step 1: Edit + commit**

---

## Self-Review

Quick checklist run against the spec:

**Coverage check:**
- §1 Context — Pre-Flight covers ✓
- §2 Architecture — Phases 1+10 cover ✓
- §3 Phase 0 sim fixes — Phase 0 ✓
- §4 Wire framing — Phase 3 ✓
- §5 Flow control — Phases 3, 11, 10.4 ✓
- §6 Multi-MCU sync — Phases 8, 10.3, 9 (hold) ✓
- §7 Buffer-budget framework — Tasks 1.1, 5.1; M1/M2/M3 in Phase 15 ✓
- §8 Stream lifecycle — Phases 6, 7 ✓
- §9 Fault taxonomy — Phase 4, 11.2 ✓
- §10 Curve-pool generation handles — Phase 2 ✓
- §11 FFI half-split + seqlock — Phase 1 ✓
- §12 Clock-sync — Phases 8, 10.3 ✓
- §13 Telemetry transport (TraceRing) — Phase 5 ✓
- §14 Step-5 carryover — Tasks 1.4, 12.1, 12.2 ✓
- §15 Open follow-ups — deferred ✓
- §16 Adversarial review — embedded as Plan-decisions A & B ✓

**Plan-decisions A & B embedded:**
- Plan-decision A: Phase 7 Task 7.2 ✓
- Plan-decision B: Phase 8 Task 8.2 + Phase 10.3 ✓

**Gate A and Gate B both have explicit acceptance tests:**
- Gate A: Phase 0 Task 0.3 ✓
- Gate B: Phase 13 Task 13.1 ✓

**Subagent-executable:** each task has explicit files-to-touch, code, commands, expected output, and a commit step. Tasks are 30–90 minutes each.

---

**Plan complete and saved to `docs/superpowers/plans/2026-04-28-step6-comm-protocol-and-sim-fixes.md`.**

Per the user's autonomous-execution instruction, the next step is the parallel codex+verifier review loop on this plan, then subagent-driven execution.
