# TIM5 Always-On; Single Stop State = Klipper Shutdown

**Date:** 2026-05-28
**Branch:** `simple-mcu-contract` (from `sota-motion`)
**Goal:** Make the motion-engine ISR (TIM5 on STM32, the host pthread on Linux) run unconditionally while the firmware is alive, and route hard faults into Klipper's existing shutdown state instead of a private, dormant TIM5-disable path. This is a reconciliation step: the piece-ring rewrite removed the segment producer protocol that used to arm TIM5, leaving a gate that never fires on pulse-only machines.

---

## 1. Background — why the gate exists and why it's now wrong

On `sota-motion`, `runtime_tick_enable` (h7 + f4) armed TIM5 when **either** of two conditions held:

```c
if (count_modulated_steppers == 0 && kalico_native_queue_len() == 0)
    return; // stay disabled
```

- Clause 1 (`count_modulated_steppers > 0`) — a phase-stepping consumer needs per-sample writes.
- Clause 2 (`kalico_native_queue_len() > 0`) — a segment is pending in the C bridge queue. `push_segment_impl` called `runtime_tick_enable` on **every** enqueue.

Pulse-mode (regular-stepping) motion worked entirely through **clause 2**: pushing a segment armed TIM5 regardless of stepping mode. The Modulated count was only the alternate entry point for phase stepping.

This branch removed segments and the curve pool (`0af240a6f`, `60f32e162`). With them went the segment queue, `kalico_native_queue_len()`, and the `push_segment`-driven arm. What remains is the lonely **clause 1** (`count_modulated_steppers == 0`) whose only caller is `set_step_mode` (`rust/kalico-c-api/src/runtime_ffi.rs:1832`). Consequence: **on a pulse-only machine nothing arms TIM5, so the engine never ticks and no motion is produced.**

The original lazy-arm scheme was a compute / USB-CDC-starvation mitigation (the legacy modulator did SPI writes in the ISR). The redesigned unified tick does no SPI work in the ISR body, so that pressure is gone. The piece-ring model also has no per-push event to lazily arm the timer. Therefore the correct model is: **the ISR free-runs from boot.**

## 2. Current stop / fault landscape (verified)

- **Klipper shutdown is the one true global stop.** Host-triggered (M112, any host error) or MCU-triggered via `shutdown("reason")`. Path: `sched_shutdown` → `irq_disable()` + `longjmp` back to `sched_main`'s `setjmp` → `run_shutdown` → runs all `DECL_SHUTDOWN` handlers → `sched_timer_reset()` (`src/sched.c`). `sched_timer_reset` wipes the timer list down to the sentinels, so the per-axis step-consumer timers (`per_axis_timer_event_*`, `src/runtime_tick.c:640`) are unlinked and stop firing → no GPIO pulses → **physical motion stops.** Recovery requires `FIRMWARE_RESTART`.

- **The runtime's `Fault` state is dormant.** `runtime_handle_status` returns `shared.runtime_status` (`runtime_ffi.rs:320`), but nothing ever sets it to `Running` / `Drained` / `Fault` — it is stuck at `Idle` after init. The hard-fault helpers (`rust/runtime/src/fault_helpers.rs`) latch `last_error` + `fault_detail` only. Consequently:
  - The `runtime_drain` disable-on-FAULT/DRAINED at `src/runtime_tick.c:464` **never fires** (`cur_status` is always `Idle`), and is also **redundant** (shutdown already stops motion).
  - The RUNNING-state liveness check in `runtime_drain` likewise never runs.

- **`runtime_tick_init` is a Klipper MCU-init hook, not a host call.** `DECL_INIT(runtime_init)` (`src/runtime_tick.c:339`) → `runtime_init` calls `runtime_tick_init()` (`:323`). At firmware startup `sched_main()` runs `ctr_run_initfuncs()`, firing every `DECL_INIT`. Same path on H7, F4, and the Linux MCU (the Linux build runs the same `sched_main` in a host process). klippy connects **afterward** over the transport. This is why the Linux widen-seed cannot live in `init` — init runs before klippy connects, so there is no clock baseline yet.

## 3. Design

### 3.1 TIM5 free-runs while firmware is alive

- **`runtime_tick_init` (h7 + f4):** arm TIM5 at the end of init (`TIM5->CR1 |= TIM_CR1_CEN` + `NVIC_EnableIRQ(TIM5_IRQn)`). Remove the "don't enable yet" deferral. With no axes configured / empty rings the ISR idles cheaply (Horner eval is skipped when no piece is active).
- **`runtime_tick_enable` (h7 + f4):** delete the `!runtime_handle` / `count_modulated_steppers == 0` early-return gate. Keep only the idempotent `CR1.CEN` short-circuit so any later call is a harmless no-op.
- **`runtime_tick_enable` (linux):** unchanged in content — it keeps the simulator scaffolding (widen-seed via `runtime_handle_seed_widen`, `kalico_runtime_install_step_queues`, and flipping `host_tick_enabled`). This is MCU-firmware-side code that does nothing on STM32 builds, so it stays as-is.

### 3.2 Trigger relocation

- **Delete the `set_step_mode` enable/disable dance** (`rust/kalico-c-api/src/runtime_ffi.rs:1832-1836`). Step mode no longer gates the timer.
- **Call `runtime_tick_enable` once from `command_kalico_configure_axis`** (`src/stepper.c`), where `init_per_axis_step_timers` already runs. On STM32 this is a no-op (TIM5 already armed at init, caught by the `CR1.CEN` guard); on Linux it performs the post-connect widen-seed + step-queue install. `configure_axis` happens after klippy connects, so the seed baseline is valid.

### 3.3 One stop state = Klipper shutdown

- **Delete** the `runtime_drain` status-based TIM5 disable (`src/runtime_tick.c:464`, the `DRAINED || FAULT` block) — dormant and redundant.
- **Add `DECL_SHUTDOWN(runtime_tick_shutdown)`** in `src/runtime_tick.c`, body calls `runtime_tick_disable()`. TIM5 goes off in the one true stop state. It is re-armed on `FIRMWARE_RESTART` via `runtime_tick_init`. (Motion already stops via the consumer-timer wipe; disabling TIM5 additionally halts the now-pointless ISR compute and avoids Renode USART2 starvation.)

### 3.4 Hard faults escalate to shutdown

- **No `runtime_status` writes anywhere.** The field stays dormant and untouched. The broader status machine (Running / Drained wiring, liveness) is out of scope.
- **Hard-fault helpers unchanged:** they keep latching `last_error` + `fault_detail` (`rust/runtime/src/fault_helpers.rs`).
- **`runtime_drain` (foreground):** read `runtime_handle_last_error()`. On a fresh nonzero hard fault (edge-detected against the previously-reported error code), emit the async fault event (`kalico_native_emit_fault_event`, carrying the precise `FaultCode` + axis detail) and then call `shutdown("kalico runtime fault")`.
- **Why foreground, not the fault site:** `shutdown()` does `irq_disable()` + `longjmp` to `sched_main`'s `setjmp`. Calling it from the fault site would (a) unwind through Rust ISR frames — the C/Rust boundary hazard called out in `docs/kalico-rewrite/mcu-c-rust-boundary.md` — and (b) on Linux the fault fires in the `host_tick` pthread, where a `longjmp` to `sched_main`'s `setjmp` in a different thread is undefined behavior. `runtime_drain` runs in `sched_main`'s task loop on every platform, so `shutdown()` there is the established, safe Klipper pattern (mirrors `try_shutdown("Timer too close")`).
- **Latency:** `runtime_drain` runs at 1 kHz, so ≤1 ms from fault to shutdown. The faulted axis already idles immediately (the fault-raising path returns `None` for that axis every tick); other axes only play valid, already-queued pieces in that window — bounded and non-garbage motion, which is acceptable.
- **Edge detection:** the drain must track the last error code it acted on so it does not re-emit / re-`shutdown` every tick once `last_error` is latched. (The first call to `shutdown` longjmps away regardless, but the edge guard keeps the pre-shutdown logic correct and avoids duplicate fault events.)

### 3.5 Shutdown reason string

`shutdown()` takes a Klipper static string, so the reason is coarse: `"kalico runtime fault"`. The precise `FaultCode` + axis detail rides the async fault event emitted immediately before the `shutdown` call, so the host still gets the specifics. No per-fault reason strings.

## 4. What changes

### 4.1 C (`src/`)

- `src/stm32/runtime_tick_h7.c`: enable TIM5 in `runtime_tick_init`; strip the `count_modulated_steppers` gate from `runtime_tick_enable` (keep `CR1.CEN` guard).
- `src/stm32/runtime_tick_f4.c`: same two edits.
- `src/runtime_tick.c`: delete the `DRAINED || FAULT` TIM5-disable block in `runtime_drain`; add the `last_error`-driven fault-event-then-`shutdown` path with edge detection; add `DECL_SHUTDOWN(runtime_tick_shutdown)` → `runtime_tick_disable()`.
- `src/stepper.c`: call `runtime_tick_enable()` once from `command_kalico_configure_axis` (alongside `init_per_axis_step_timers`).

### 4.2 Rust (`rust/`)

- `rust/kalico-c-api/src/runtime_ffi.rs`: delete the `set_step_mode` enable/disable dance (`:1832-1836`); `set_step_mode` just sets the mode and returns.

### 4.3 Unchanged / out of scope

- The Linux host-tick simulator scaffolding (`src/linux/runtime_tick_host.c`) — kept as-is.
- The `runtime_status` state machine (Running / Drained wiring) and the liveness/watchdog logic that depended on it — remains dormant; this change neither revives nor removes it beyond deleting the dead disable path.
- Hard-fault helpers' latching behavior — unchanged.
- `runtime_tick_disable` — retained; now used solely by the shutdown handler.

## 5. Test / verification notes

- **Pulse-only motion regression (the core bug):** with phase stepping disabled, a configured axis must produce steps. This is the path that broke when segments were removed; it is the primary thing to confirm.
- **Phase-stepping motion** still works (TIM5 was already on in that case via the Modulated gate).
- **Shutdown disables TIM5:** trigger M112 / a host shutdown and confirm `runtime_tick_disable` runs (TIM5 `CR1.CEN` clear) and motion halts.
- **FIRMWARE_RESTART re-arms TIM5** via `runtime_tick_init`.
- **Hard-fault → shutdown:** induce a `PieceStartInPast` (starve the piece feed) and confirm the async fault event carries the code/axis and the MCU shuts down within ~1 ms.
- **Linux sim** still runs end-to-end (the `configure_axis`-driven `runtime_tick_enable` performs the widen-seed + queue install).

## 6. Open questions

None outstanding. (Reason-string granularity resolved in §3.5: coarse string + precise fault event.)
