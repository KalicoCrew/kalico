# MCU Log Endpoint — Stage 3 (Rust engine fault emit sites) Plan

> **For agentic workers:** REQUIRED SUB-SKILL: superpowers:subagent-driven-development. The Rust work goes through a `rust-engineer` subagent (project rule). Steps use `- [ ]`.

**Goal:** Make the MCU log actually useful — wire `kalico_log_emit` into the Rust engine's fault-raise path so real faults/anomalies appear in the host log store with resolved `code_name` ("why did it fault"), plus level-gating and ring-overflow accounting.

**Architecture:** All faults flow through the 11 `raise_*` helpers in `rust/runtime/src/fault_helpers.rs` (verified: the single choke point; no direct `last_error` assignment elsewhere). Add a centralized `emit_fault_log(fault, detail)` there that level-gates then calls the C `kalico_log_emit` (FFI, gated for MCU/sim builds, no-op stub on host). The specific fault identity rides in the `code` field → host `FaultCode::from_u16` → `code_name`. The C ring drain reports any overflow drops.

**Tech stack:** Rust `no_std` runtime (dual-target host/MCU), the `extern "C"` seam to `src/kalico_log.c` (Stage 2), the bench for end-to-end verification.

**Verification environment:** the Trident bench (now working end-to-end). Trigger a fault via a jog and confirm a `runtime.fault_latched` record with the right `code_name` lands in `events/<label>.jsonl`.

---

## Design decisions

- **Event mapping:** ALL faults → `(SUBSYSTEM_RUNTIME, EVENT_RUNTIME_FAULT_LATCHED)`, `level=error`, `code=fault.as_u16()`, `arg0=fault_detail`, `arg1=0`. The fault *identity* is carried by `code`→`code_name` (e.g. `"PieceStartInPast"`, `"TickIntervalExceeded"`); the per-fault context is in `fault_detail` (axis in bits 16..24). One frame per fault; uniform; no per-helper event divergence. (Subsystem-specific events like `tick.interval_exceeded` stay in the table for future use but are not used here — `code_name` already disambiguates.)
- **Level gating (decision E):** a process-global `static MIN_LEVEL: AtomicU8` (default `2` = warn). `emit_fault_log` checks `level >= MIN_LEVEL` before emitting. Faults are `error` (3) so they always pass today; the gate is the mechanism Stage 4 will drive with a runtime setter + debug/trace emits. No runtime setter in Stage 3.
- **FFI gating (the crux):** mirror `dispatch_stepper.rs`'s Pattern 2 — `#[cfg(any(not(any(test, feature = "host")), feature = "mcu-linux"))]` for the `extern "C"` block, and the inverse cfg for a no-op stub. This emits on bare-metal MCU (h7/f4) AND the `mcu-linux` sim firmware (both link `kalico_log.c`), and stubs on the pure-host cdylib / `cargo test` (no such symbol).
- **Ring-overflow accounting (spec §7):** the C drain (`kalico_log_drain`) reports any accumulated `kalico_log_drops` as one `runtime.log_drops` frame, then resets — fail-loud, never silent.

---

### Task 1: Rust — emit at the fault choke point (`rust-engineer`)

**Files:** Modify `rust/runtime/src/fault_helpers.rs`, `rust/runtime/src/log_codes.rs`. Test in `rust/runtime/src/fault_helpers.rs` (the existing `#[cfg(test)] mod tests`).

- [ ] **log_codes.rs:** refine `EVENT_RUNTIME_FAULT_LATCHED` so the `code` field (not an arg) carries the fault: update its doc to "`code`/`code_name` carry the fault; `arg0` = `fault_detail`" and its template to `"fault latched, detail={arg0}"`. Add a new event:

```rust
/// The MCU dropped N structured-log entries on ring overflow since the last
/// drain (fail-loud overflow accounting); `arg0` = dropped count.
pub const EVENT_RUNTIME_LOG_DROPS: u16 = 4;
```
and the `event_info` arm:
```rust
        (SUBSYSTEM_RUNTIME, EVENT_RUNTIME_LOG_DROPS) => {
            ("runtime.log_drops", "dropped {arg0} log entries (ring overflow)")
        }
```
Update the `event_info_all_runtime_events` test (if it enumerates) to include the two new/changed entries.

- [ ] **fault_helpers.rs:** add the FFI binding + gate + level static + centralized emit. Place near the top after the imports:

```rust
use core::sync::atomic::AtomicU8;

use crate::log_codes::{EVENT_RUNTIME_FAULT_LATCHED, SUBSYSTEM_RUNTIME};

/// Wire log levels — must match rust/motion-bridge/src/mcu_log.rs::mcu_level_str.
const LOG_LEVEL_ERROR: u8 = 3;

/// Minimum level emitted (gate at emit, spec decision E). Default = warn (2).
/// Stage 4 adds a runtime setter; Stage 3 leaves it at the default, so all
/// error-level faults pass. Process-global; relaxed ordering is fine (a missed
/// level change by one tick is harmless).
static MIN_LEVEL: AtomicU8 = AtomicU8::new(2);

// kalico_log_emit lives in src/kalico_log.c — present in MCU (h7/f4) and the
// mcu-linux sim firmware (both link kalico_log.c), absent in the pure-host
// cdylib / cargo test. Mirror dispatch_stepper.rs's gating exactly.
#[cfg(any(not(any(test, feature = "host")), feature = "mcu-linux"))]
unsafe extern "C" {
    fn kalico_log_emit(level: u8, subsystem: u8, event: u16, code: u16, arg0: u32, arg1: u32);
}

/// Emit a structured log for a latched fault, gated by `MIN_LEVEL`. The fault
/// identity rides in `code` (host resolves `code_name`); `arg0` = `fault_detail`.
/// No-op on the pure-host build (no `kalico_log_emit` symbol).
#[inline]
fn emit_fault_log(fault: FaultCode, detail: u32) {
    if LOG_LEVEL_ERROR < MIN_LEVEL.load(Ordering::Relaxed) {
        return;
    }
    #[cfg(any(not(any(test, feature = "host")), feature = "mcu-linux"))]
    unsafe {
        kalico_log_emit(
            LOG_LEVEL_ERROR,
            SUBSYSTEM_RUNTIME,
            EVENT_RUNTIME_FAULT_LATCHED,
            fault.as_u16(),
            detail,
            0,
        );
    }
    #[cfg(not(any(not(any(test, feature = "host")), feature = "mcu-linux")))]
    {
        let _ = (fault, detail);
    }
}
```

- [ ] **fault_helpers.rs:** in EACH of the 11 `raise_*` helpers, add `emit_fault_log(FaultCode::<X>, detail)` immediately AFTER the existing `last_error.store(...)` (do NOT alter the load-bearing detail-first/code-second store order). For `raise_jog_parameters_invalid` the detail is `0`. Example for `raise_tick_interval_exceeded`:

```rust
    let detail = gap_ticks.min(0xFFFF);
    shared.fault_detail.store(detail, Ordering::Release);
    shared
        .last_error
        .store(FaultCode::TickIntervalExceeded.as_i32(), Ordering::Release);
    emit_fault_log(FaultCode::TickIntervalExceeded, detail);
```
(For helpers that currently inline the detail in the store, hoist it to a `let detail = ...;` first so it can be passed to `emit_fault_log`.)

- [ ] **Tests:** add a host-side test that `emit_fault_log` is a no-op (compiles + doesn't panic) and that each `raise_*` still latches the right `(code, detail)` (the existing tests cover latching; add one asserting `emit_fault_log(FaultCode::PieceStartInPast, 0x10000)` returns without linking `kalico_log_emit`). Keep `MIN_LEVEL` test-visible if needed for a gate test.

- [ ] **Verify:** `cd rust && cargo test -p runtime fault && cargo test -p runtime log_codes && cargo build -p runtime` clean (host stub path). `cargo build -p motion-bridge` clean (cdylib must NOT reference `kalico_log_emit`).

---

### Task 2: C — ring-overflow drop reporting (main agent)

**Files:** Modify `src/kalico_log.h`, `src/kalico_log.c`.

- [ ] **kalico_log.h:** add the C mirror of the new event (next to the existing `KALICO_LOG_EVENT_RUNTIME_MCU_READY`):
```c
#define KALICO_LOG_EVENT_RUNTIME_LOG_DROPS 4
```

- [ ] **kalico_log.c:** at the top of `kalico_log_drain`, before the drain loop, snapshot+reset the drop counter under `irq_save` and, if nonzero, enqueue one report frame (the loop below ships it):
```c
    // Ring-overflow accounting (spec §7): report any entries dropped since the
    // last drain, then reset. Fail-loud — loss is surfaced, never silent.
    irqstatus_t df = irq_save();
    uint32_t drops = kalico_log_drops;
    kalico_log_drops = 0;
    irq_restore(df);
    if (drops)
        kalico_log_emit(KALICO_LOG_LEVEL_WARN, KALICO_LOG_SUBSYS_RUNTIME,
                        KALICO_LOG_EVENT_RUNTIME_LOG_DROPS, 0, drops, 0);
```

---

### Task 3: Build + bench verification

- [ ] `cargo` host build/tests green (Task 1).
- [ ] Commit + push `observability`.
- [ ] Bench: flash BOTH MCUs (firmware changed — `fault_helpers.rs` is `no_std` MCU code) + rebuild host `.so`, via the flashing skill (use the `trident` alias, not `trident.local`).
- [ ] Trigger a fault: `SET_KINEMATIC_POSITION` then a small `G1` jog (sim/bench is no-hardware-damage; the engine "can barely jog" so a jog is likely to latch a fault). Confirm a `runtime.fault_latched` record appears in `events/<label>.jsonl` with `level=error`, a real `code`, and `code_name` (e.g. `PieceStartInPast` / `StepsPerSampleExceeded`). Per-command motion authorization is the user's.

---

## Verification of success
A real engine fault, raised during a jog on the bench, appears in the structured log store as `event=runtime.fault_latched`, `level=error`, with `code` + resolved `code_name` and the `fault_detail` in `arg0` — the "why did it fault" payoff. Host build/tests green; the cdylib does not link `kalico_log_emit`; drops (if any) are reported, never silent. Stage 4 (runtime min-level control + dedup) remains.
