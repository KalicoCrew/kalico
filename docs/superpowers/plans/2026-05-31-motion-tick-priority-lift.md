# Motion-Tick Priority Lift + Dedicated Step-Output Timer — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: use `superpowers:subagent-driven-development` (recommended) or `superpowers:executing-plans` to implement task-by-task. Steps use checkbox (`- [ ]`) tracking.
>
> **Rust tasks:** dispatch with `subagent_type: rust-engineer` (project rule — any change under `rust/`).
> **Firmware build/flash:** never cross-compile MCU firmware locally and scp it. Use commit → push → pull → build-on-Pi (`make -j$(nproc)`) → flash BOTH MCUs (H7 from `.config.h7.bak`, F446 from `.config.f446.test`; `make clean` between them). The `flashing-trident-mcus` skill encapsulates this.
> **`-311` capture:** the fault rides the host journal `[KALICO-FAULT]` log; the `tick_blocker` field + `addr2line` name the starving callback.

**Spec:** `docs/superpowers/specs/2026-05-31-motion-tick-priority-lift-design.md` (read it first — the NVIC map, the SPSC non-racing invariant, and the heater-safety backstop are all grounded there).

**Goal:** Lift the motion-critical pair (TIM5 producer + the step-emission consumer) to NVIC priority 2 *together*, demote the SysTick scheduler to priority 3, and move the consumer off the SysTick software-timer queue onto a dedicated 32-bit hardware step-output timer (TIM2 on both MCUs, pending Task 0 audit). Producer/consumer stay same-priority → the step-queue SPSC stays non-racing (no lock-free rework). Add a TIM5 self-budget fault and fail-loud queue-overflow.

**Decided priority scheme (do not re-litigate; verified NVIC map):** `serial/bxCAN @ 0` (unchanged) · `USB OTG/FS + FDCAN @ 1` (unchanged) · `TIM5 + step-output timer @ 2` · `SysTick @ 3` (demoted from 2). Comms (serial/USB/CAN at 0 and 1) stay strictly above motion; only the software scheduler drops below it. Both MCUs have `__NVIC_PRIO_BITS = 4` (16 levels) and reset-default PRIGROUP = 0 (no sub-priority → same-number IRQs can't preempt each other).

**Tech stack:** C (Klipper MCU firmware, STM32H723 + STM32F446, shared template `runtime_tick_timer.h`), Rust (`runtime` crate linked as staticlib). Host tests via `cargo test -p runtime`.

**Testing reality:** Rust logic (soonest-axis scan, wrap-safe deltas, budget threshold math, overflow fail-loud) is host-unit-testable. The hardware-register pokes (timer setup, NVIC priority, ISR wiring) are **MCU-only** and verified by build + `kalico-sim` end-to-end + bench flash. The incremental sequence below ensures a priority change can never silently brick: every step is independently flashable and testable, and the priority flip is the *last* gated step after same-priority parity is proven.

---

## Incremental safety principle (read before starting)

The dangerous edit is the priority flip (demoting SysTick). To make it non-bricking:

1. **Phase A** — add the dedicated step-output timer **at the SAME priority as SysTick (2)**, replacing the SysTick per-axis software timers, and prove **parity** (motion identical to today). No priority inversion change yet.
2. **Phase B** — only after parity is bench-confirmed, **flip**: demote SysTick 2→3. Now the motion pair is above the scheduler.

If Phase B regresses, revert is a one-constant change. If Phase A regresses, the dedicated-timer machinery is the suspect and the priority map is unchanged (still safe).

---

## File inventory

| File | Responsibility | Change |
|------|----------------|--------|
| `src/generic/armcm_timer.c` | SysTick scheduler setup | Demote SysTick priority 2→3 (Phase B only) |
| `src/stm32/runtime_tick_timer.h` | Shared STM32 TIM5 backend | TIM5 prio via `#define`; add `step_output_timer_init` + `STEP_OUT_TIM_IRQHandler`; add self-budget measurement |
| `src/runtime_tick.c` | Engine glue, per-axis consumer wiring | Replace per-axis SysTick timers with step-output-timer wiring; `kalico_kick_step_output` |
| New shared header (e.g. `src/generic/kalico_nvic_prio.h`) | One home for the two priority numbers | `#define KALICO_MOTION_NVIC_PRIO 2`, `KALICO_SCHED_NVIC_PRIO 3` |
| `rust/runtime/src/per_axis_timer.rs` | Consumer logic | `kalico_per_axis_step_event(axis)` → `kalico_step_output_event()` (scan owned axes, soonest next) |
| `rust/runtime/src/error.rs` | Fault codes | Add `TickBudgetExceeded` (`-313`; `-312` is taken by `PieceRingEmpty`). Reuse existing `StepQueueFull = -304` for overflow — no new overflow code |
| `rust/runtime/src/step_queue.rs` | SPSC accessor | No logic change; strengthen invariant comment |
| `rust/runtime/src/engine.rs` | Tick body | Raise overflow fault on `push` == false; raise budget fault on over-budget (or do budget in C — see Task 5) |
| `src/Kconfig` | Build config | `CONFIG_KALICO_TICK_BUDGET_PERCENT` (default 50) |

---

## Task 0: Audit — confirm TIM2 is free on BOTH flashed configs (NO code)

**Why:** the whole step-output-timer plan assumes a free 32-bit GP timer. TIM2 is the candidate; it could be claimed by a hardware-PWM pin on a specific board config.

- [ ] **Step 1: Enumerate claimed timers per config.** On the Pi, for each config (`.config.h7.bak`, `.config.f446.test`), build and `objdump`/grep the resulting firmware + the active `printer.cfg` pin assignments to determine which TIMx are claimed by `hardware_pwm.c` (heaters/fans) and by `runtime_tick` (TIM5). Confirm **TIM2 is unclaimed** on each.
- [ ] **Step 2: Record the decision.** If TIM2 is free on both → proceed with TIM2 on both. If TIM2 is taken on one, pick the next free **32-bit** GP timer for that MCU (per-family backends already differ, so mixed TIMx is fine) and record it. Only if no 32-bit timer is free, select a 16-bit GP timer and note the wrap constraint (spec §6 risk 3) — escalate before proceeding, because 16-bit changes the one-shot reprogram design.
- [ ] **Step 3: Write the chosen timer per MCU into this plan** (edit the table above) before any implementation task.

**Gate:** do not start Task 2 until the timer choice is recorded for both MCUs.

---

## Task 1: Rust — add the self-budget fault code (host-only, no MCU)

**Files:** `rust/runtime/src/error.rs` (the `FaultCode` enum, currently lines 9-24), `rust/kalico-c-api/include/kalico_runtime.h` (mirror), tests.

**Verified inventory (do not collide):** `Ok=0`, `PieceStartInPast=-301`, `PieceCoeffInvalid=-302`, `NewtonNoConverge=-303`, `StepQueueFull=-304`, `TickIntervalExceeded=-311`, `PieceRingEmpty=-312`. So **`StepQueueFull = -304` already exists** (reuse it for overflow, Task 4 — no new code) and **`-312` is taken** (do not reuse for the budget fault).

- [ ] **Step 1:** Add `TickBudgetExceeded = -313` to the `FaultCode` enum in `rust/runtime/src/error.rs`, after `PieceRingEmpty`. Mirror it into `rust/kalico-c-api/include/kalico_runtime.h` (the `error.rs` header comment mandates mirroring any code that crosses the FFI).
- [ ] **Step 2:** Add `#[test]`s asserting the discriminant values (`TickBudgetExceeded as i32 == -313`, and that no two variants collide) and that the host-side decode round-trips it.
- [ ] **Step 3:** `cd rust && cargo test -p runtime` — PASS. `cargo clippy -p runtime --all-targets -- -D warnings` — clean.
- [ ] **Step 4:** Commit (`feat(error): add TickBudgetExceeded(-313)`).

---

## Task 2: Rust — generalize the consumer to a single soonest-across-owned-axes scan

**Files:** `rust/runtime/src/per_axis_timer.rs` (+ tests), `rust/runtime/src/step_queue.rs` (comment only).

**Why:** the dedicated step-output timer is one timer for all owned axes; the consumer must scan owned axes, emit due/late steps, and return the soonest remaining head across owned axes.

- [ ] **Step 1:** Add `kalico_step_output_event() -> u32` (`#[no_mangle] extern "C"`). It takes no axis arg; it reads the owned-axis set (from a C-provided `extern "C"` mask getter, mirroring today's `per_axis_armed_mask`) and, for each owned axis: peek head; if `delta = head.cycle_abs.wrapping_sub(now) as i32 <= DUE_WINDOW_CYCLES`, pop + `runtime_emit_step_pulses`, bounded by a per-dispatch cap (reuse `MAX_STEPS_PER_EVENT`); track the **wrap-safe minimum** of remaining heads' `cycle_abs` across owned axes.
- [ ] **Step 2:** Return the soonest remaining `cycle_abs`; if all owned queues empty, return `now + kalico_runtime_get_idle_park_cycles()`. If the per-dispatch cap was hit (work remains), return `now` (re-fire immediately) — same semantics as today.
- [ ] **Step 3:** Keep `kalico_per_axis_step_event(axis)` temporarily as a thin shim *only if* the C side still calls it during Phase A bring-up; otherwise delete. (Decide based on Task 3 wiring; prefer deleting to avoid dead paths — project rule: no throwaway code.)
- [ ] **Step 4 (host tests — fail-loud-and-early):** add `#[test]`s using the existing host step_queue mock shim:
  - **soonest-axis selection:** two owned axes with heads at different `cycle_abs`; assert the returned next-waketime equals the smaller, wrap-safe (including a case where one is just past a u32 wrap boundary).
  - **due/late emit:** head in the past → emitted immediately; head far future → not emitted, returned as next waketime.
  - **per-dispatch cap:** a backlog > cap → exactly cap emitted, returns `now`.
  - **all-empty:** returns `now + idle_park`.
  - **unowned axis ignored:** an axis not in the owned mask with a due head is **not** emitted.
- [ ] **Step 5:** Strengthen the `step_queue.rs` invariant comment to name the *dedicated step-output timer same-priority dependency* (not just "the per-axis SysTick consumer"): the non-racing u32 SPSC holds **iff** producer (TIM5) and the step-output-timer consumer are the same NVIC priority; if that ever changes, upgrade to atomics.
- [ ] **Step 6:** `cargo test -p runtime` PASS; `cargo clippy` clean. Commit (`feat(runtime): single step-output consumer scanning owned axes`).

---

## Task 3: C — dedicated step-output hardware timer (Phase A: SAME priority as SysTick)

**Files:** `src/stm32/runtime_tick_timer.h`, `src/runtime_tick.c`, new `src/generic/kalico_nvic_prio.h`.

**Why:** stand up the new dispatch substrate at the *current* priority (2, same as SysTick) so motion is at parity before any priority inversion change. This is the safety-critical "prove it first" step.

- [ ] **Step 1:** Create `src/generic/kalico_nvic_prio.h` with `#define KALICO_MOTION_NVIC_PRIO 2` and `#define KALICO_SCHED_NVIC_PRIO 2` *(note: SCHED stays 2 in Phase A; Task 6 flips it to 3)*. Include it where TIM5 and SysTick priorities are set; replace the literal `2`s with the macros (TIM5 in `runtime_tick_timer.h:56`, SysTick in `armcm_timer.c:50`).
- [ ] **Step 2:** Add `step_output_timer_init()` in `runtime_tick_timer.h`: enable the chosen timer's pclock, configure 32-bit up-counter, one-shot (`OPM`) or free-running-with-compare per the chosen reprogram style, `DIER` compare/update IRQ, `NVIC_SetPriority(<TIMx>_IRQn, KALICO_MOTION_NVIC_PRIO)`, `NVIC_EnableIRQ`. Park the first compare far in the future. Call it from `runtime_tick_init` right after the TIM5 arm.
- [ ] **Step 3:** Add `STEP_OUT_TIM_IRQHandler`: clear the timer SR flag; call `kalico_step_output_event()`; load the returned absolute cycle into the timer's compare register (CCR/ARR per chosen style); re-arm the one-shot. Mirror the TIM5 ISR's structure.
- [ ] **Step 4:** Replace the per-axis SysTick consumer wiring in `runtime_tick.c`:
  - Delete `per_axis_timers[4]`, `per_axis_timer_event_0..3`, `per_axis_handlers`, `arm_per_axis_step_timer`'s `sched_add_timer` body — **but keep the owned-axis mask** (`per_axis_armed_mask` → rename to e.g. `step_output_owned_mask`), and expose it to Rust via an `extern "C"` getter for Task 2's scan. `arm_per_axis_step_timer(axis)` now just sets the owned bit (and, on the first owned axis, ensures the step-output timer is running).
  - Replace `kalico_kick_per_axis_timer(axis, waketime)` with `kalico_kick_step_output(uint32_t cycle_abs)`: read the step-output timer's current compare; if `cycle_abs` is sooner (wrap-safe `(int32_t)(cycle_abs - current_compare) < 0`), rewrite the compare register. Keep `__attribute__((used, externally_visible))` (referenced only from the Rust archive).
- [ ] **Step 5:** Update the Rust TIM5 producer path that calls the old kick to call `kalico_kick_step_output(cycle_abs)` instead (it currently calls `kalico_kick_per_axis_timer`). This is the one Rust→C call-site change for the kick.
- [ ] **Step 6 (build + sim):** build both configs on the Pi (`make clean` between H7/F446). Run `kalico-sim` end-to-end (the sim's host tick path must still produce identical step counts). **Gate: sim parity = step output identical to pre-change for a representative G-code.**
- [ ] **Step 7 (bench, Phase A parity):** flash both MCUs. Run a representative print/jog. **Confirm motion is at parity** and `-311` rate is no worse than baseline (it may already improve because the unowned-axis SysTick dispatch load is gone — spec §2.2). Capture `[KALICO-FAULT]` log.
- [ ] **Step 8:** Commit (`feat(mcu): dedicated step-output timer at scheduler priority (Phase A parity)`).

**Gate:** do not proceed to Task 6 (the flip) until Phase A parity is bench-confirmed.

---

## Task 4: C+Rust — fail-loud on step-queue overflow

**Files:** `rust/runtime/src/tick.rs` (where `step_queue::push` is called in the TIM5 path), `rust/runtime/src/step_queue.rs` (no logic change).

**First verify:** does the current TIM5 push path already call `raise_fault(FaultCode::StepQueueFull as i32)` on `Err(StepQueueFull)`, or does it silently drop the entry? If it already faults, this task is a no-op-plus-test; if it drops, this closes the gap.

- [ ] **Step 1:** At the `push` call site (`tick.rs`), on `Err(StepQueueFull)`, call `raise_fault(FaultCode::StepQueueFull as i32)` (the **existing** `-304` code — do not invent a new one) instead of silently dropping. Foreground `runtime_drain` already escalates a fresh nonzero `last_error` to `kalico_native_emit_fault_event` + `shutdown` (spec-2026-05-28 path) — no new escalation code needed.
- [ ] **Step 2 (host test):** fill a mock queue to `STEP_QUEUE_DEPTH - 1` (= 31), push once more, assert the fault is raised (and that no entry is silently lost).
- [ ] **Step 3:** `cargo test -p runtime` PASS; clippy clean. Commit (`feat(runtime): fail loud on step-queue overflow`).

---

## Task 5: C+Rust — TIM5 self-budget fault

**Files:** `src/stm32/runtime_tick_timer.h` (measurement), `rust/runtime/src/engine.rs` or a small `extern "C"` latch, `src/Kconfig`.

**Decision — measure where:** measure in **C** in `TIM5_IRQHandler` (the cycle counter and the period are both C-side: `runtime_cyccnt_read()`/`DWT->CYCCNT` and `modulation_tick_interval`). On over-budget, call an `extern "C"` Rust latch (`kalico_raise_tick_budget_fault(spent_cycles, period_cycles)`) that sets `FaultCode::TickBudgetExceeded`. This keeps the threshold math testable in Rust while keeping the hot-path measurement in C at the existing bench bracket.

- [ ] **Step 1:** Add `CONFIG_KALICO_TICK_BUDGET_PERCENT` to `src/Kconfig` (default 50, range 10..90).
- [ ] **Step 2:** In `TIM5_IRQHandler`, at the existing `runtime_bench_capture_enter/exit` bracket: read cyccnt on entry/exit; `spent = exit - entry`; `budget = (modulation_tick_interval * CONFIG_KALICO_TICK_BUDGET_PERCENT) / 100`; if `spent > budget`, call the Rust latch. (Wrap-safe subtraction; the ISR is short so no u32 wrap concern within one tick.)
- [ ] **Step 3:** Rust `kalico_raise_tick_budget_fault` (`#[no_mangle] extern "C"`) → `raise_fault(FaultCode::TickBudgetExceeded, detail = spent/period packed)`.
- [ ] **Step 4 (host tests):** test the budget math helper (extract the `spent > budget` decision into a pure Rust fn `tick_over_budget(spent, period, pct) -> bool`): boundary cases at 49/50/51 % for representative period values; assert the C ISR's intended threshold matches. Test that the latch sets the right FaultCode.
- [ ] **Step 5:** `cargo test -p runtime` PASS; build both configs on Pi. Commit (`feat(mcu): TIM5 self-budget fault (TickBudgetExceeded)`).

---

## Task 6: C — THE FLIP: demote SysTick below the motion pair (Phase B)

**Files:** `src/generic/kalico_nvic_prio.h` (one constant).

**Why last:** everything above is validated with the priority map unchanged. This single change is the actual inversion fix and the only step that can re-order the whole scheduler.

- [ ] **Step 1:** Change `#define KALICO_SCHED_NVIC_PRIO 2` → `3` in `kalico_nvic_prio.h`. (TIM5 and step-output timer stay at `KALICO_MOTION_NVIC_PRIO = 2`; nothing else moves. USB/CAN @1, ADC/DMA @0 untouched.)
- [ ] **Step 2 (build + sim):** build both configs; `kalico-sim` end-to-end (sim has no NVIC, so this only confirms no functional regression in logic; the priority effect is MCU-only).
- [ ] **Step 3 (bench — the payoff test):** flash both MCUs. Run the representative print that previously produced `-311`. **Confirm `-311 TickIntervalExceeded` is eliminated (or dramatically reduced) and motion smoothness improves.** Capture `[KALICO-FAULT]` log; if any `-311` remains, `addr2line` the `tick_blocker` to see what is still starving the tick (should now be impossible from SysTick; if it shows a SysTick callback, the flip didn't take — check `SCB->SHP`).
- [ ] **Step 4 (bench — safety sanity, document outcome):** confirm heaters still honor `max_duration` under normal load (the prio-3 safety timer fires on time); observe USB-CDC log/telemetry for backpressure (spec §6 risk 6). Record observations for the later safety review (spec §7) — do **not** redesign heater safety here.
- [ ] **Step 5:** Commit (`fix(mcu): demote SysTick below motion pair — eliminates -311`).

---

## Test suite summary (fail-loud-and-early)

**Host-testable (`cargo test -p runtime`) — must exist before/with the corresponding task:**
- Soonest-across-owned-axes selection, wrap-safe (Task 2) — incl. a u32-wrap-boundary case.
- Due / late / far-future emit branches (Task 2).
- Per-dispatch cap behavior (Task 2).
- All-empty → idle-park (Task 2).
- Unowned axis ignored (Task 2).
- Step-queue overflow raises `StepQueueOverflow`, no silent drop (Task 4).
- `tick_over_budget(spent, period, pct)` boundary math at 49/50/51 % (Task 5).
- Fault-code discriminants + round-trip (Task 1).
- SPSC non-racing invariant: a host test that documents/asserts the single-producer/single-consumer access pattern (it cannot test true concurrency, but it can assert the API contract — push only advances tail, pop only advances head — so a future refactor that crosses the streams fails a test).

**MCU-only (bench-verified, not host-testable):**
- NVIC priority actually applied (`SCB->SHP` for SysTick; NVIC IPR for TIM5/TIM2) — inspect via debugger or a diag readback.
- TIM5 self-budget fault fires on an artificially long ISR (bench: inject a stall).
- Step-output timer reprogram + kick correctness under real timing (Phase A parity, Task 3 Step 7).
- `-311` elimination after the flip (Task 6 Step 3) — **the headline pass/fail gate.**
- Heater `max_duration` still honored at prio 3 (Task 6 Step 4).

---

## Risks & open questions (carried from spec §6 — confirm during execution)

1. **The flip is the severe-failure step** — a long motion ISR now starves the whole scheduler incl. the software heater backstop. Mitigated by Task 5 (self-budget) + IWDG; the incremental order means the flip is the *only* unvalidated bit when it lands. **Open:** quantify worst-case prio-2 occupancy vs. prio-3 `max_duration` margin (feeds the later safety review).
2. **TIM2 availability** — Task 0 gate. Fallback to another 32-bit timer; 16-bit is a last resort (changes the one-shot design).
3. **16-bit wrap** (only if forced) — chunked re-arm + wrap-safe compare. Avoided if a 32-bit timer is free.
4. **Soonest-axis wrap math** — covered by the u32-wrap host test (Task 2).
5. **Kick vs. self-reprogram at equal priority** — non-racing by the same-priority invariant; documented in `step_queue.rs` and the new wiring comments.
6. **USB-CDC backpressure under scheduler demotion** — observe at Task 6 Step 4; USB stays at prio 1 so framing is safe, only the foreground TX drain is at prio 3.

**Open question to resolve at Task 0:** is TIM2 truly free on the *exact* `printer.cfg` in use on the bench (heater/fan pin → timer mapping is config-dependent)? Must be answered per MCU before Task 2.
