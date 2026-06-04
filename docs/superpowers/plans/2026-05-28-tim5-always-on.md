# TIM5 Always-On; Fault → Klipper Shutdown — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
>
> **Rust tasks:** dispatch with `subagent_type: rust-engineer` (project rule — any change under `rust/`).
> **Firmware build/flash:** never cross-compile MCU firmware locally and scp it. Use the commit → push → pull → build-on-Pi → flash flow (the `flashing-trident-mcus` skill encapsulates it). Build with `make -j$(nproc)` on the Pi.

**Goal:** Make the motion-engine ISR (TIM5 on STM32, the host pthread on Linux) free-run while the firmware is alive, and route hard faults into Klipper's existing global shutdown instead of a dormant, private TIM5-disable path.

**Architecture:** Three coupled mechanisms gate TIM5 today (init defers enable; `runtime_tick_enable` gates on `count_modulated_steppers`; `set_step_mode` is the only caller). The piece-ring rewrite removed the segment producer protocol that used to arm TIM5 on pulse-mode motion, so on a pulse-only machine the timer never starts. This plan arms TIM5 at init, deletes the gate and the `set_step_mode` dance, relocates the enable trigger to `configure_axis` (needed for the Linux widen-seed), ties TIM5-off to Klipper shutdown via `DECL_SHUTDOWN`, and escalates hard faults to `shutdown()` from the foreground drain task.

**Tech Stack:** C (Klipper MCU firmware, STM32H7/F4 + Linux-MCU), Rust (`kalico-c-api` staticlib linked into the C build). Spec: `docs/superpowers/specs/2026-05-28-tim5-always-on-design.md`.

**Testing reality:** Rust logic is cargo-tested on the host target. The C changes are hardware-register pokes with no host unit test — they are verified by (a) a clean firmware build on the Pi, (b) `kalico-sim` end-to-end behavioral runs, and (c) a final bench flash. The sim run is the real integration test; treat its observations as the pass/fail gate.

---

## File Structure

| File | Responsibility | Change |
|------|----------------|--------|
| `rust/kalico-c-api/src/runtime_ffi.rs` | FFI surface | Remove `set_step_mode`'s enable/disable dance |
| `src/stm32/runtime_tick_h7.c` | H7 TIM5 init/IRQ | Arm TIM5 at init; strip `count_modulated` gate |
| `src/stm32/runtime_tick_f4.c` | F4 TIM5 init/IRQ | Same two edits |
| `src/stepper.c` | Axis config command | Call `runtime_tick_enable()` from `command_kalico_configure_axis` |
| `src/runtime_tick.c` | Engine glue + drain task | Add `DECL_SHUTDOWN` → disable; remove `:464` disable block; add `last_error` → `shutdown()` |

`src/linux/runtime_tick_host.c` and the `runtime_status` state machine are **unchanged** (out of scope per spec §4.3).

---

## Task 1: Rust — remove the `set_step_mode` enable/disable dance

**Files:**
- Modify: `rust/kalico-c-api/src/runtime_ffi.rs` (the `Ok(())` arm of `kalico_runtime_set_step_mode`, currently ~`:1814-1837`)
- Test: `rust/kalico-c-api/tests/configure_axes_blob_step_modes.rs` (existing; exercises `set_step_mode`)

**Why:** Step mode no longer gates the timer. `set_step_mode` should only set the mode. The `runtime_tick_enable`/`disable` extern decls stay (still defined and called on the C side); only the Rust call sites here go away.

- [ ] **Step 1: Verify the existing test currently passes (baseline)**

Run: `cd rust && cargo test -p kalico-c-api --test configure_axes_blob_step_modes`
Expected: PASS (this is the regression guard for `set_step_mode`).

- [ ] **Step 2: Replace the `Ok(())` arm**

Current code (the dance to delete):

```rust
                Ok(()) => {
                    // Spec §6.3: re-evaluate TIM5 arm state after every
                    // successful step-mode flip. ...
                    use runtime::state::MAX_STEPPER_OIDS;
                    let mut modulated_count = 0u8;
                    for i in 0..MAX_STEPPER_OIDS {
                        if shared.step_modes[i].load(Ordering::Acquire)
                            == runtime::state::StepMode::Modulated as u8
                        {
                            modulated_count = modulated_count.saturating_add(1);
                        }
                    }
                    if modulated_count == 0 {
                        runtime_tick_disable();
                    } else {
                        runtime_tick_enable();
                    }
                    KALICO_OK
                }
```

Replace with:

```rust
                Ok(()) => {
                    // TIM5 lifecycle is decoupled from step mode (spec
                    // 2026-05-28): the timer is armed at runtime_tick_init and
                    // disabled only on Klipper shutdown. Setting a step mode no
                    // longer arms/disarms the tick.
                    KALICO_OK
                }
```

- [ ] **Step 3: Run the test suite to confirm no regression**

Run: `cd rust && cargo test -p kalico-c-api`
Expected: PASS. (`set_step_mode` still returns `KALICO_OK` and sets the mode; the unused test stubs `runtime_tick_enable`/`disable` remain harmless no-ops.)

- [ ] **Step 4: Lint clean**

Run: `cd rust && cargo clippy -p kalico-c-api --all-targets -- -D warnings`
Expected: no warnings. (If `Ordering` / `runtime::state` imports become unused *in this function only*, they are still used elsewhere in the file — do not remove file-level imports unless clippy flags them.)

- [ ] **Step 5: Commit**

```bash
git add rust/kalico-c-api/src/runtime_ffi.rs
git commit -m "refactor(ffi): decouple TIM5 arm from set_step_mode"
```

---

## Task 2: H7 — arm TIM5 at init; strip the `count_modulated` gate

**Files:**
- Modify: `src/stm32/runtime_tick_h7.c` (`runtime_tick_init` tail ~`:159-163`; `runtime_tick_enable` body ~`:58-112`)

**Why:** Pulse-only motion needs the ISR running. With no segment-push event to lazily arm TIM5, the timer must start at init. `runtime_tick_enable` stays as an idempotent re-arm (no-op once armed) so the shared interface — and the Linux build's seed work — keep their entry point.

- [ ] **Step 1: Arm TIM5 at the end of `runtime_tick_init`**

Find the tail of `runtime_tick_init`, currently ending:

```c
    NVIC_SetPriority(TIM5_IRQn, 2);

    // Don't enable yet — runtime_init pushes segments first; first push triggers
    // runtime_tick_enable() via the producer protocol.
}
```

Replace that trailing comment + closing brace with:

```c
    NVIC_SetPriority(TIM5_IRQn, 2);

    // Always-on (spec 2026-05-28): the piece-ring engine has no per-push event
    // to lazily start TIM5 (segments are gone), so the ISR free-runs from boot.
    // It idles cheaply when no axis has an active piece. TIM5 is disabled only
    // on Klipper shutdown (DECL_SHUTDOWN in runtime_tick.c) and re-armed here on
    // FIRMWARE_RESTART.
    TIM5->EGR  = TIM_EGR_UG;
    TIM5->SR   = ~TIM_SR_UIF;     // clear stale UIF before enabling
    TIM5->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM5_IRQn);
}
```

- [ ] **Step 2: Strip the gate from `runtime_tick_enable`**

Replace the `runtime_tick_enable` body's gate (the `if (!runtime_handle) return;` and `if (count_modulated_steppers == 0) return;` blocks) so the function becomes an idempotent re-arm. Final body:

```c
__attribute__((used, externally_visible))
void
runtime_tick_enable(void)
{
    // Idempotent re-arm. TIM5 is armed at init and disabled only on Klipper
    // shutdown, so on STM32 this is normally a no-op (CR1.CEN already set).
    // The entry point is retained because the Linux build's runtime_tick_enable
    // performs the post-connect widen-seed + step-queue install
    // (src/linux/runtime_tick_host.c); configure_axis calls it on every build.
    if (TIM5->CR1 & TIM_CR1_CEN) {
        return;
    }
    TIM5->CR1 &= ~TIM_CR1_CEN;
    TIM5->ARR  = (runtime_clock_freq / CONFIG_KALICO_MOTION_SAMPLE_RATE_HZ) - 1U;
    TIM5->EGR  = TIM_EGR_UG;
    TIM5->SR   = 0;
    TIM5->SR   = ~TIM_SR_UIF;
    TIM5->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM5_IRQn);
}
```

- [ ] **Step 3: Sanity-check no remaining `count_modulated_steppers` reference in this file**

Run: `grep -n "count_modulated_steppers\|Don't enable yet" src/stm32/runtime_tick_h7.c`
Expected: no matches.

- [ ] **Step 4: Commit**

```bash
git add src/stm32/runtime_tick_h7.c
git commit -m "feat(h7): arm TIM5 at init, remove modulated-count gate"
```

---

## Task 3: F4 — arm TIM5 at init; strip the `count_modulated` gate

**Files:**
- Modify: `src/stm32/runtime_tick_f4.c` (`runtime_tick_init` tail; `runtime_tick_enable` body ~`:58-92`)

**Why:** Same as Task 2, for the F446. (The register names are identical; F4 uses `RCC->APB1ENR` only in init, which is untouched here.)

- [ ] **Step 1: Arm TIM5 at the end of `runtime_tick_init`**

Find the tail of `runtime_tick_init` (after `NVIC_SetPriority(TIM5_IRQn, 2);` and the "first push triggers runtime_tick_enable" comment) and replace the trailing comment + closing brace with:

```c
    NVIC_SetPriority(TIM5_IRQn, 2);

    // Always-on (spec 2026-05-28): ISR free-runs from boot; idles cheaply when
    // no axis has an active piece. Disabled only on Klipper shutdown; re-armed
    // here on FIRMWARE_RESTART.
    TIM5->EGR  = TIM_EGR_UG;
    TIM5->SR   = ~TIM_SR_UIF;
    TIM5->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM5_IRQn);
}
```

- [ ] **Step 2: Strip the gate from `runtime_tick_enable`**

Replace the `runtime_tick_enable` body (the `if (!runtime_handle) return;` and `if (count_modulated_steppers == 0) return;` blocks) with the idempotent re-arm:

```c
__attribute__((used, externally_visible))
void
runtime_tick_enable(void)
{
    // Idempotent re-arm. See runtime_tick_h7.c::runtime_tick_enable for the
    // full rationale (always-on at init; entry point kept for the Linux seed).
    if (TIM5->CR1 & TIM_CR1_CEN) {
        return;
    }
    TIM5->CR1 &= ~TIM_CR1_CEN;
    TIM5->ARR  = (runtime_clock_freq / CONFIG_KALICO_MOTION_SAMPLE_RATE_HZ) - 1U;
    TIM5->EGR  = TIM_EGR_UG;
    TIM5->SR   = 0;
    TIM5->SR   = ~TIM_SR_UIF;
    TIM5->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM5_IRQn);
}
```

- [ ] **Step 3: Sanity-check**

Run: `grep -n "count_modulated_steppers\|Don't enable yet" src/stm32/runtime_tick_f4.c`
Expected: no matches.

- [ ] **Step 4: Commit**

```bash
git add src/stm32/runtime_tick_f4.c
git commit -m "feat(f4): arm TIM5 at init, remove modulated-count gate"
```

---

## Task 4: `stepper.c` — call `runtime_tick_enable()` from `configure_axis`

**Files:**
- Modify: `src/stepper.c` (`command_kalico_configure_axis`, the per-axis-timer install block ~`:320-331`)

**Why:** The `set_step_mode` dance was the only caller of `runtime_tick_enable`. On STM32 the call is now an idempotent no-op (already armed), but the **Linux** build relies on it for the post-connect widen-seed + step-queue install. `configure_axis` runs after klippy connects, so the seed baseline is valid.

- [ ] **Step 1: Add the enable call after the timer-install block**

Current code:

```c
    extern void init_per_axis_step_timers(void);
    static uint8_t per_axis_timers_installed;
    if (!per_axis_timers_installed) {
        per_axis_timers_installed = 1;
        init_per_axis_step_timers();
    }
}
```

Replace with:

```c
    extern void init_per_axis_step_timers(void);
    static uint8_t per_axis_timers_installed;
    if (!per_axis_timers_installed) {
        per_axis_timers_installed = 1;
        init_per_axis_step_timers();
    }

    // Drive the platform tick-enable now that an axis is configured. On STM32
    // TIM5 is already armed at init, so the idempotent CR1.CEN guard makes this
    // a no-op; on the Linux MCU build this performs the post-connect widen-seed
    // + step-queue install (src/linux/runtime_tick_host.c). Replaces the old
    // set_step_mode-driven enable (removed 2026-05-28).
    extern void runtime_tick_enable(void);
    runtime_tick_enable();
}
```

- [ ] **Step 2: Commit**

```bash
git add src/stepper.c
git commit -m "feat(stepper): enable runtime tick from configure_axis"
```

---

## Task 5: `runtime_tick.c` — shutdown handler, remove disable block, fault → shutdown

**Files:**
- Modify: `src/runtime_tick.c` (`runtime_drain` fault/disable region ~`:443-467`; add a `DECL_SHUTDOWN` near the file's other decls)

**Why:** Klipper shutdown is the single stop state. Tie TIM5-off to it via `DECL_SHUTDOWN`; delete the dormant+redundant `:464` disable block; escalate hard faults to `shutdown()` from this foreground task (safe `longjmp`, unlike the ISR/Rust path).

- [ ] **Step 1: Confirm `sched.h` is included (for `DECL_SHUTDOWN` / `shutdown`)**

Run: `grep -n '#include "sched.h"\|#include "command.h"' src/runtime_tick.c`
Expected: at least one present. `DECL_SHUTDOWN` and `shutdown()` come from `sched.h`/`command.h`. If neither is present, add `#include "sched.h"` and `#include "command.h"` with the other includes at the top of the file.

- [ ] **Step 2: Add the shutdown handler**

Add near the end of the file (e.g. just after `DECL_TASK(runtime_drain);`):

```c
// Single stop state (spec 2026-05-28 §3.3): TIM5 goes off when Klipper shuts
// down. The per-axis step-consumer timers are already wiped by sched_timer_reset
// during shutdown, so motion has stopped; this just halts the now-pointless ISR
// compute (and avoids Renode USART2 starvation). Re-armed on FIRMWARE_RESTART
// via runtime_tick_init.
void
runtime_tick_shutdown(void)
{
    runtime_tick_disable();
}
DECL_SHUTDOWN(runtime_tick_shutdown);
```

- [ ] **Step 3: Add the `last_error` → shutdown escalation; remove the disable block**

Locate this region in `runtime_drain` (the dormant FAULT block, then the disable block):

```c
    // FAULT → also block kicks. Emit one-shot kalico_fault event if the
    // engine just transitioned INTO Fault since the last drain (so the host
    // gets a single notification, not a 1 kHz spam stream).
    if (cur_status == 3 /* FAULT */) {
        runtime_liveness_ok = 0;
        if (prev_engine_status != 3 /* FAULT */) {
            int32_t fault_code = runtime_handle_last_error(runtime_handle);
            uint32_t fault_detail = runtime_handle_fault_detail(runtime_handle);
            uint32_t cur_seg = runtime_handle_current_segment_id(runtime_handle);
            kalico_native_emit_fault_event((uint16_t)fault_code, fault_detail, cur_seg);
        }
    }

    // DRAINED or FAULT → disable TIM5 on the first transition into that
    // state. ...
    if ((cur_status == 2 /* DRAINED */ || cur_status == 3 /* FAULT */)
        && prev_engine_status != cur_status) {
        runtime_tick_disable();
    }
```

Leave the **first** block (`cur_status == 3`) exactly as-is — it is dormant scaffolding for the status machine (spec §3.4) and never fires today. **Delete the entire second block** (the `DRAINED || FAULT` `runtime_tick_disable()` call) and replace it with the `last_error`-driven escalation:

```c
    // Hard-fault escalation (spec 2026-05-28 §3.4). The runtime status machine
    // is dormant, so we key off last_error directly: on a fresh nonzero fault
    // code, notify the host with the specifics, then enter Klipper's global
    // shutdown — the single stop state. shutdown() does irq_disable()+longjmp
    // back to sched_main; that is safe HERE in foreground (DECL_TASK context)
    // but NOT from the ISR/Rust tick path (longjmp over Rust frames; on Linux a
    // cross-thread longjmp from the host_tick pthread), which is why escalation
    // lives in this drain rather than at the fault site. The edge guard prevents
    // re-emitting every 1 kHz tick before the longjmp takes effect.
    static int32_t last_acted_error;
    int32_t cur_error = runtime_handle_last_error(runtime_handle);
    if (cur_error != 0 && cur_error != last_acted_error) {
        last_acted_error = cur_error;
        uint32_t fdetail = runtime_handle_fault_detail(runtime_handle);
        uint32_t cseg = runtime_handle_current_segment_id(runtime_handle);
        kalico_native_emit_fault_event((uint16_t)cur_error, fdetail, cseg);
        runtime_liveness_ok = 0;
        shutdown("kalico runtime fault");
    }
```

- [ ] **Step 4: Sanity-check the edits**

Run: `grep -n "runtime_tick_disable\|DECL_SHUTDOWN\|shutdown(\"kalico runtime fault\")\|DRAINED .*FAULT" src/runtime_tick.c`
Expected: `runtime_tick_disable` now appears only inside `runtime_tick_shutdown`; the `DECL_SHUTDOWN(runtime_tick_shutdown)` line is present; the `shutdown("kalico runtime fault")` call is present; the old `DRAINED || FAULT` disable conditional is gone.

- [ ] **Step 5: Commit**

```bash
git add src/runtime_tick.c
git commit -m "feat(runtime): TIM5 off on shutdown; hard fault -> shutdown"
```

---

## Task 6: Build both MCUs on the Pi (compile gate for all C edits)

**Why:** None of Tasks 2–5 are host-compilable. This is the first real compiler check of the C changes. Build BOTH targets — msgid descriptors and oid types differ, so `make clean` between configs is mandatory (project rule).

- [ ] **Step 1: Push the branch so the Pi can pull**

```bash
git push
```

- [ ] **Step 2: Build on the Pi via the canonical flow**

Use the `flashing-trident-mcus` skill (it runs commit → push → pull → build-host → build-MCU). If running manually on `dderg@trident.local`: pull, then for **each** MCU restore its config and build:

```bash
# H723 (main):  cp .config.h7.bak .config && make clean && make -j$(nproc)
# F446 (bottom): cp .config.f446.test .config && make clean && make -j$(nproc)
```

Expected: both builds succeed with no errors. The Rust staticlib (`motion_bridge_native` / `kalico-c-api`) links cleanly — in particular `runtime_tick_enable`/`disable` still resolve (defined in C, no longer called from Rust).

- [ ] **Step 3: If the build fails, fix and re-commit**

Common failure modes to check first: missing `#include "sched.h"`/`"command.h"` in `runtime_tick.c` (Task 5 Step 1); a stray `count_modulated_steppers` reference; `_DECL_STATIC_STR` unhappy with the `shutdown()` string (it must be a string literal — it is). Fix, commit, re-push, rebuild.

---

## Task 7: `kalico-sim` behavioral verification (the integration test)

**Why:** This is where the actual behavior is proven without hardware. Use the `kalico-sim` skill to run G-code against the real firmware in the simulator.

- [ ] **Step 1: Pulse-only motion produces steps (the core fix)**

Using `kalico-sim`, configure a machine with **phase stepping disabled** (regular/pulse stepping on the moving axis) and run a short move (e.g. a single `G1 X10 F600` after homing, or the sim's standard motion fixture).
Expected: the axis emits step pulses — the step-queue consumer sees nonzero step counts. (Before this change, TIM5 never armed on a pulse-only axis, so step count would be zero. This is the regression-equivalent assertion.)

- [ ] **Step 2: Phase-stepping motion still works**

Run the same move with a phase-stepped axis configured.
Expected: motion produced as before (this path already armed TIM5 via the old Modulated gate; confirm no regression).

- [ ] **Step 3: Shutdown disables TIM5 / halts motion**

Trigger a shutdown in the sim (e.g. `M112`, or a host-side `shutdown`).
Expected: the `shutdown` message reaches the host with its reason; motion halts; `runtime_tick_shutdown` runs (TIM5 `CR1.CEN` cleared). After `FIRMWARE_RESTART`, motion works again (TIM5 re-armed at init).

- [ ] **Step 4: Hard fault escalates to shutdown**

Induce a `PieceStartInPast` fault (starve the piece feed so a piece's `start_time` is > 2 ISR ticks in the past when the ISR reaches it).
Expected: an async `kalico_fault` event carries the `FaultCode` + axis detail, then the MCU shuts down (host receives `shutdown` with reason `"kalico runtime fault"`) within ~1 ms. No 1 kHz duplicate fault spam (edge guard).

- [ ] **Step 5: Record results**

Note the observed step counts and message sequences in the commit message or PR description. If any check fails, return to the relevant task — do not proceed to bench.

---

## Task 8: Bench flash + hardware verification (permission-gated)

**Why:** Final confidence on real silicon. **Motion commands require explicit per-command user permission** (project hard rule — never issue G28/G1/etc. without an explicit "yes").

- [ ] **Step 1: Flash both MCUs**

Use the `flashing-trident-mcus` skill (H723 from `.config.h7.bak`, F446 from `.config.f446.test`). Flash BOTH — the protocol surface must match on both ends.

- [ ] **Step 2: Confirm clean bring-up**

After flash, confirm both MCUs enumerate and klippy connects without a shutdown. Check `klippy.log` (fetch to `/tmp/klippy-<timestamp>.log` first) for any `kalico runtime fault` / unexpected shutdown at idle — TIM5 free-running with empty rings must NOT fault or starve USB.

- [ ] **Step 3: Pulse-only motion on the bench — ASK FIRST**

Request explicit permission before any motion. With permission, run a small move on a pulse-stepped axis and confirm physical steps.
Expected: motion occurs. This confirms the end-to-end fix on hardware.

- [ ] **Step 4: Final commit / PR**

If a PR is wanted, summarize: TIM5 now free-runs from boot, the modulated-count gate and `set_step_mode` dance are gone, shutdown is the single stop state, and hard faults escalate to `shutdown()`.

---

## Self-Review (completed during authoring)

- **Spec coverage:** §3.1 → Tasks 2/3 (init arm + gate strip) and Task 1 (dance removal); §3.2 → Task 4 (configure_axis trigger); §3.3 → Task 5 (DECL_SHUTDOWN + disable-block removal); §3.4 → Task 5 (last_error → shutdown, dormant readers left intact); §3.5 → Task 5 (`"kalico runtime fault"` string). §5 verification → Tasks 7/8.
- **Placeholders:** none — every code step shows the exact before/after.
- **Consistency:** `runtime_tick_enable`/`runtime_tick_disable` names match across Rust, both STM32 files, `stepper.c`, and the shutdown handler; `runtime_tick_shutdown` defined once and registered once; `last_acted_error` edge guard defined where used.
- **Out-of-scope honored:** `runtime_status` writes and the dormant `cur_status` readers (`:430`/`:446`/`:474`/`:479`) are explicitly left untouched (Task 5 Step 3).
