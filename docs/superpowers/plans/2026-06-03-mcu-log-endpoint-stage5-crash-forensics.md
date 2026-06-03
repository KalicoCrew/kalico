# MCU Log Endpoint — Stage 5 (crash forensics → structured log) Plan

> Routes the MCU's prior-boot persistent-diag (CPU fault record, reset cause, foreground-freeze) through the structured-log path, so a hard reset (watchdog / CPU fault) surfaces in the log store — not just klippy.log `output()` strings. Spec §11 ("possible stage 5"). Rust via `rust-engineer` (project rule).

**Goal:** When the MCU hard-resets, the next boot emits structured crash-summary frames (`runtime.mcu_reset`, `runtime.hard_fault`, `runtime.fault_status`, `runtime.fg_freeze`, `runtime.rt_progress`) so the crash is queryable in the log UI with the fault PC/cause + session correlation.

**Why:** a hard crash is exactly when forensics matter most, and it's currently invisible to the structured store. The prior-boot snapshot already survives the reset in `.bkp_bss`/`.persistent_diag`; we just re-emit it structured.

**Verification (deterministic):** `±10 mm F6000` jog hard-resets the MCU (watchdog). On reconnect, confirm `runtime.mcu_reset` (cause=IWDG) + `runtime.fg_freeze` (hung PC, if latched) appear in `events/*.jsonl`.

---

## Design

- **Trigger:** the existing post-connect one-shot in `command_kalico_configure_axis` (stepper.c) — host connected + mcu-log hook installed (frames before that are lost; same timing constraint as `mcu_ready`). The prior-boot data is snapshotted by `fault_handler_report_task` at boot-init, so it's readable there. Once per boot, right after the `mcu_ready` marker.
- **Data sources (all file-static/persistent in fault_handler.c, confirmed readable at configure time):**
  - `reset_cause_snapshot` (RCC RSR/CSR captured + cleared at boot)
  - `live_snap.iwdg_reset_count`, `live_snap.worst_fg_stall_pc`, `live_snap.worst_fg_stall_ticks` (cross-boot persistent)
  - `fault_rec` (CPU fault record: `magic`, `exc_kind`, `pc`, `lr`, `cfsr`, `hfsr`, `fault_count`) — preserved (current run hasn't faulted)
  - `runtime_diag_prior_packed_raw` (runtime_tick.c, `extern`, externally_visible)
- **Frames (2×u32 arg width; conditional to avoid clean-boot noise):**
  - `runtime.mcu_reset` — **always**; level warn if abnormal (IWDG or fault), else debug. `arg0`=reset-cause bits, `arg1`=iwdg_reset_count.
  - `runtime.hard_fault` + `runtime.fault_status` — only if `fault_rec.magic == FAULT_MAGIC`. level error. `code`=exc_kind, `arg0`=pc, `arg1`=lr; then `arg0`=cfsr, `arg1`=hfsr.
  - `runtime.fg_freeze` — only if `worst_fg_stall_ticks > 0`. level warn. `arg0`=hung pc, `arg1`=stall ticks.
  - `runtime.rt_progress` — only if abnormal. level warn. `arg0`=packed progress, `arg1`=fault_count.
- **Timestamps:** `_time` = replay time, `time_estimated:true` (prior-boot clock is a different epoch — not widenable). The raw values ride in the `arg0/arg1` JSON fields (operator hex-decodes the PC for `addr2line`).
- **Deferred (v1.1):** the full 32-entry diag-ring replay (timestamp-epoch + arg-width); still in klippy.log via `output()`.

---

### Task 1: host-side event codes (`rust-engineer`)

`rust/runtime/src/log_codes.rs` — add 5 events + `event_info` arms + test (codes 1–4 are taken):

```rust
/// MCU hardware reset; arg0 = reset-cause bits (RCC RSR/CSR), arg1 = cumulative IWDG resets.
pub const EVENT_RUNTIME_MCU_RESET: u16 = 5;
/// Prior-boot CPU hard fault; code = exc_kind, arg0 = fault PC, arg1 = LR.
pub const EVENT_RUNTIME_HARD_FAULT: u16 = 6;
/// Prior-boot fault status registers; arg0 = CFSR, arg1 = HFSR.
pub const EVENT_RUNTIME_FAULT_STATUS: u16 = 7;
/// Prior-boot foreground freeze; arg0 = hung PC, arg1 = stall ticks.
pub const EVENT_RUNTIME_FG_FREEZE: u16 = 8;
/// Prior-boot runtime progress at crash; arg0 = packed tag/stage/value, arg1 = fault_count.
pub const EVENT_RUNTIME_RT_PROGRESS: u16 = 9;
```
`event_info` arms (names/templates):
- `(SUBSYSTEM_RUNTIME, EVENT_RUNTIME_MCU_RESET) => ("runtime.mcu_reset", "mcu reset (cause bits={arg0}, iwdg_resets={arg1})")`
- `(SUBSYSTEM_RUNTIME, EVENT_RUNTIME_HARD_FAULT) => ("runtime.hard_fault", "cpu hard fault pc={arg0} lr={arg1}")`
- `(SUBSYSTEM_RUNTIME, EVENT_RUNTIME_FAULT_STATUS) => ("runtime.fault_status", "fault status cfsr={arg0} hfsr={arg1}")`
- `(SUBSYSTEM_RUNTIME, EVENT_RUNTIME_FG_FREEZE) => ("runtime.fg_freeze", "foreground freeze pc={arg0} stall_ticks={arg1}")`
- `(SUBSYSTEM_RUNTIME, EVENT_RUNTIME_RT_PROGRESS) => ("runtime.rt_progress", "runtime progress packed={arg0} fault_count={arg1}")`

Verify: `cargo test -p runtime log_codes`, `cargo build -p runtime -p motion-bridge`.

### Task 2: C-side mirrors + emit (main agent)

- `src/kalico_log.h`: add `#define KALICO_LOG_EVENT_RUNTIME_MCU_RESET 5` … `_RT_PROGRESS 9`.
- `src/generic/fault_handler.h`: declare `void kalico_diag_emit_prior_crash(void);`.
- `src/generic/fault_handler.c`: `#include "kalico_log.h"`; add `kalico_diag_emit_prior_crash()` reading the statics above and emitting the conditional frames.
- `src/stepper.c`: `#include "generic/fault_handler.h"`; call `kalico_diag_emit_prior_crash()` in the `command_kalico_configure_axis` one-shot block, right after the `mcu_ready` emit.

### Task 3: build + bench crash-verify

- Host: `cargo` green. Commit + push.
- Flash both MCUs + host `.so` (firmware changed — fault_handler.c + the Rust staticlib). `trident` alias.
- Crash test: `±10 mm F6000` burst → MCU watchdog reset → reconnect → confirm `runtime.mcu_reset` (cause=IWDG) [+ `runtime.fg_freeze`] in `events/*.jsonl`.

## Verification of success
A deliberate hard reset surfaces a structured `runtime.mcu_reset` (and fault/freeze details when present) in the log store with the reset cause + fault PC — the crash that was previously only in klippy.log text is now queryable with structure. Bench recovers cleanly.
