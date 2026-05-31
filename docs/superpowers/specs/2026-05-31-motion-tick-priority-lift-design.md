# Motion-Tick Priority Lift + Dedicated Step-Output Timer — Design

**Date:** 2026-05-31
**Branch:** `simple-mcu-contract` (HEAD `6305b38dd`)
**Goal:** Eliminate `-311 TickIntervalExceeded` (late motion tick) and the motion-smoothness problem it co-causes, by lifting the motion-critical interrupt *pair* — the TIM5 motion-sample tick (step-time **producer**) and the per-axis step-emission **consumer** — to a single NVIC priority level *above* Klipper's SysTick scheduler, while keeping producer and consumer at the **same** priority as each other so the step-queue SPSC handoff stays non-racing.

> This is a *plan-its-implementation* document. The architecture below is **decided**, not under debate. The job of this spec is to ground that architecture in the verified facts of the current firmware and to define the exact mechanism so the implementation plan (`docs/superpowers/plans/2026-05-31-motion-tick-priority-lift.md`) can execute it incrementally and bench-validatably.

---

## 0. Reading order / boundary discipline

This change adds **one new piece of shared hardware** (a dedicated step-output timer) and **moves one consumer** off the SysTick software-timer queue onto it. Per `docs/kalico-rewrite/mcu-c-rust-boundary.md`:

- The **hardware timer setup + ISR is C**, per-MCU, living in `src/stm32/runtime_tick_timer.h` (the shared STM32 template, already the home of the TIM5 ISR) or a sibling included by `runtime_tick_h7.c` / `runtime_tick_f4.c`.
- The **"which axis is next / pop+emit / next waketime" logic is Rust**, reached through a narrow `extern "C"` entry point — exactly mirroring today's `kalico_per_axis_step_event`.
- No Rust-typed structure crosses the seam. The step-queue storage is already a C struct in C-owned `.bss` (`step_queues[]`); this design does not change that.

---

## 1. Verified current state (the facts the plan stands on)

### 1.1 NVIC priority model on these chips

Both targets are ARMv7-M (Cortex-M7 on the H723, Cortex-M4F on the F446). Verified from the source:

- **Priority grouping is the reset default (PRIGROUP = 0).** Klipper does **not** call `NVIC_SetPriorityGrouping` anywhere (verified: no match in `src/`), so `SCB->AIRCR.PRIGROUP` stays at its reset value of `0`. `armcm_boot.c` zeroes all NVIC IP / SCB SHPR priority registers at boot (`armcm_boot.c:112-113`) but does not touch the grouping field. PRIGROUP = 0 ⇒ on a 4-priority-bit core (see below) all 4 bits are *preemption* (group) bits and 0 are sub-priority bits. Preemption is therefore decided purely by numeric priority (lower number = higher urgency), and same-numeric-priority interrupts **cannot preempt each other** (they run to completion, then the pending one runs). This is the bedrock the "keep producer/consumer at the same level" decision relies on. **Plan note:** the implementation should either rely on this reset default explicitly (document it) or call `NVIC_SetPriorityGrouping(0)` defensively at boot to make the invariant non-implicit — flagged as an open question (§6).
- **`__NVIC_PRIO_BITS = 4` on both families** (STM32 device headers; 16 priority levels, 0–15). The G0/F0 cm0 headers use 2 bits, but the H7/F4 motion MCUs use 4 — ample headroom; the firmware only uses 0–2 today. (Note: the literal priority *numbers* passed to `NVIC_SetPriority` are what matter for ordering; with 4 implemented bits the numbers 0/1/2/3 map to the top of the priority field as-is.)
- **`armcm_enable_irq(func, irqn, priority)`** (`src/generic/armcm_boot.h:13`, a macro that calls `NVIC_SetPriority((NUM),(PRIORITY))` then enables the vector) is the single chokepoint that installs a peripheral vector and sets its NVIC priority. SysTick and the runtime TIM5 set their priority directly via `NVIC_SetPriority` (system handler / pre-armed timer) rather than through this macro.

**Current numeric priority map for the STM32 motion MCUs (verified from `armcm_enable_irq` / `NVIC_SetPriority` call sites):**

| Priority | Owner(s) on H7/F4 | Source |
|---|---|---|
| **0** (highest) | **USART/serial** (`USARTx_IRQn`), **CAN bxCAN** (`CAN_RX0/RX1/TX/SCE_IRQn`) | `src/stm32/serial.c:111`, `src/stm32/can.c:329-335` (`armcm_enable_irq(..., 0)`) |
| **1** | **USB OTG** (`OTG_IRQn`), **USB FS** (`USBx_IRQn`), **FDCAN** (`CAN_IT0_IRQn`) | `src/stm32/usbotg.c:514`, `src/stm32/usbfs.c:438`, `src/stm32/fdcan.c:370` (`armcm_enable_irq(..., 1)`) |
| **2** | **SysTick scheduler** *and* **TIM5 motion tick** | `armcm_timer.c:154` (`NVIC_SetPriority(SysTick_IRQn, 2)`); `runtime_tick_h7.c:122` / `runtime_tick_f4.c:139` (`NVIC_SetPriority(TIM5_IRQn, 2)`) |

> Correction vs. an earlier draft: on these STM32 targets the **comms IRQs occupy priorities 0 and 1**, not ADC/DMA. (ADC on STM32 is polled/DMA-driven without a dedicated `armcm_enable_irq` priority on these configs; the `ADC_IRQn,0` and `SERCOM/PIO` entries in the tree are other-family boards — lpc176x / atsamd / rp2040 — not H7/F4.) The load-bearing fact for this design is unchanged: **SysTick and TIM5 are both at 2, and everything above them (0 and 1) is comms/serial/USB/CAN.**

- **The central scheduler clock is SysTick**, not a TIMx — `armcm_timer.c:1` ("Timer based on ARM Cortex-M3/M4 SysTick and DWT logic"), with `DWT->CYCCNT` as `timer_read_time()`'s 32-bit free-running counter. So *the SysTick interrupt is the scheduler dispatch*: when the soonest scheduled `struct timer` is due, SysTick fires and `timer_dispatch_many()` (`armcm_timer.c:176`) runs the due callbacks in a burst, deferring via `TIMER_REPEAT_TICKS` (`= timer_from_us(100)`, ~100 µs catch-up window) when it falls behind.

### 1.2 The priority inversion (root cause, restated against the facts)

- TIM5 (the step-time **producer**) is at priority 2.
- The per-axis step-emission **consumer** is a Klipper `struct timer` on the **SysTick software-timer queue** (`src/runtime_tick.c:612-680`, the `per_axis_timer_event_N` trampolines + `arm_per_axis_step_timer`). It therefore *runs inside the SysTick ISR*, also effectively at priority 2.
- SysTick (priority 2) runs **all** the scheduler's noisy work — sensors, comms drains, pulse counters, the runtime drain, heater/PWM `max_duration` timers (see §1.4) — and dispatches in bursts up to ~100 µs.
- Because grouping = 0 and TIM5 == SysTick numerically, **TIM5 cannot preempt a SysTick burst**. A long burst delays the next TIM5 entry past the late-tick threshold → the engine's own detector raises `-311`.
- The consumer, living *on* the SysTick queue, jitters on the same bursts → step pulses are emitted late/clustered → measurable motion roughness.

The engine's late-tick detector (verified): the tick path raises `FaultCode::TickIntervalExceeded as i32` via the shared-state `raise_fault(code: i32)` method (`rust/runtime/src/tick.rs:107/114/121`) when the inter-tick interval (measured via `runtime_cyccnt_read()` deltas against `sample_period_cycles`, `state.rs:73`) exceeds the threshold. `FaultCode` lives in **`rust/runtime/src/error.rs`** (not `fault.rs` — there is no `fault.rs`); `TickIntervalExceeded = -311` (`error.rs:21`). The blocking callback is named via `sched_last_dispatched_func` (`src/sched.c:304`) and shipped to the host as `tick_blocker` (the legacy "segment_id" slot) so `addr2line` can name the SysTick callback that starved the tick (`src/runtime_tick.c:398, 426`).

**Existing `FaultCode` inventory (`error.rs:9-24`, all negative i32):** `Ok = 0`, `PieceStartInPast = -301`, `PieceCoeffInvalid = -302`, `NewtonNoConverge = -303`, `StepQueueFull = -304`, `TickIntervalExceeded = -311`, `PieceRingEmpty = -312`. **Note for §4/§5:** a `StepQueueFull = -304` code *already exists* (so the queue-overflow fail-loud reuses it, not a new code), and `-312` is already taken by `PieceRingEmpty` — the new self-budget code must use the next free slot (e.g. `-313`).

### 1.3 The step-queue SPSC (rust/runtime/src/step_queue.rs)

- Storage is a **C struct in C-owned `.bss`** (`step_queues: UnsafeCell<[StepQueue; 4]>`, the `extern "C"` static at `step_queue.rs:113-115`), per the boundary doc. Rust is the typed accessor (`push`/`pop`/`peek`/`len`). `StepEntry` is `{ cycle_abs: u32, dir: i8, _pad }` (8 bytes); `StepQueue` is `{ tail: u16, head: u16, _pad, buf: [StepEntry; 32] }` (264 bytes), with `const _` layout asserts.
- Indices `head`/`tail` are **`u16` free-running counters accessed via `read_volatile`/`write_volatile`, paired with a `fence(Ordering::Release)` after the producer's `buf[slot]` write + `tail` publish and a `fence(Ordering::Acquire)` before the consumer's `buf[slot]` read** (`step_queue.rs:165-205`). This is a standard SPSC release/acquire discipline, but it is **non-racing rather than lock-free**: the comments and `# Safety` contracts require a *single* producer and a *single* consumer, and on a single core the producer (TIM5) and consumer (the per-axis timer) cannot interleave *only because they run at the same NVIC priority and so cannot preempt each other*. The fences guard compiler/cross-core ordering; the same-priority property is what guarantees the two endpoints never run concurrently on this single-core MCU.
- `push` is producer-only (TIM5 ISR); `pop`/`peek` are consumer-only. `head == tail ⇒ empty`; `tail.wrapping_sub(head) >= STEP_QUEUE_DEPTH (32) ⇒ full`. `STEP_QUEUE_DEPTH = 32`. `push` returns `Err(StepQueueFull)` on full (fail-loud hook point — see §5).

> **Load-bearing invariant for this whole design:** the non-racing SPSC stays correct **iff producer and consumer remain at the same NVIC priority** (and on a single core). The decided architecture *preserves this* (it lifts the pair *together*). If a future change ever makes the consumer a higher priority than the producer (or vice versa) — so one can preempt the other mid-update — the volatile-u16 + fence discipline is no longer sufficient (a preempting reader could observe a torn `buf[slot]`/counter pair) and the queue MUST be reworked to a true preemption-safe SPSC (full `core::sync::atomic` counters with the matching acquire/release, and a single-word commit so no torn entry is observable). This spec records the invariant so that change can't be made silently.

### 1.4 Heater / GPIO `max_duration` safety enforcement — where it lives

- `src/gpiocmds.c` (digital out, soft PWM) enforces `max_duration` per output (`d->max_duration`, set from the configure command at `gpiocmds.c:124`). The mechanism is a **Klipper `struct timer` armed on the SysTick software-timer queue** (verified: `gpiocmds.c` declares `struct timer` and calls `sched_add_timer` — the digital-out `struct digital_out` carries `timer`, `end_time`, `max_duration`): each scheduled `set` computes `end_time = waketime + max_duration` and the toggle/load event checks `timer_is_before(end_time, wake)` → `shutdown("Scheduled digital out event will exceed max_duration")` (`gpiocmds.c:89, 160`). On `shutdown`, the `DECL_SHUTDOWN` handler forces every output to its `default_value` via `gpio_out_write`.
- **Therefore the `max_duration` safety check runs on the SysTick scheduler — the exact queue this design demotes below the motion pair.** (The hardware-PWM and ADC siblings — `hard_pwm.c`, `*_adc.c` — are separate and not the focus here; the digital-out / heater `max_duration` path is the one whose scheduling priority changes.)

### 1.5 Existing self-budget hook site

The TIM5 ISR already brackets the engine tick with `runtime_bench_capture_enter()` / `runtime_bench_capture_exit()` (`runtime_tick_timer.h:95-101`), and the inline comment there already flags: *"Self-budget fault hook lives here in the redesign (TODO -311 plan): measure cycles spent in this ISR; if > X% of the period, fail loud."* This design fills that TODO (§5).

---

## 2. The decided architecture

Lift the motion-critical **pair** to one NVIC priority level **P**, with `P < 2` (i.e. above SysTick), keeping producer and consumer at the **same** P:

1. **TIM5 (producer)** stays a hardware timer; its NVIC priority moves from 2 to **P**.
2. **The step-emission consumer moves off the SysTick software-timer queue onto a dedicated hardware step-output timer**, also set to NVIC priority **P**. This is mandatory: the consumer cannot be above SysTick while it *is* a SysTick software timer. The dedicated timer is a **one-shot reprogrammed to the soonest pending step across all owned axes** — a small per-axis "next step" scheduler.
3. SysTick (the general scheduler: sensors, comms, heaters-PWM `max_duration`, drain) **stays at 2** — i.e. *below* the motion pair.

Net: `motion pair @ P` ; `SysTick @ 2` with `P < 2`. Producer/consumer same-priority ⇒ SPSC stays non-racing (no lock-free rework). The "kick" (today `sched_del`+`sched_add` from the TIM5 ISR, `kalico_kick_per_axis_timer`) becomes a trivial **same-priority compare-and-reprogram** of the step-output timer's compare register.

### 2.1 Choosing P — exact level and why it is safe

Priorities 0 and 1 are occupied by **serial/CAN (0)** and **USB OTG/FS + FDCAN (1)**. Two candidate schemes:

- **Scheme A — motion pair → 1 (share the level with USB/CAN).** Rejected as the default. USB and CAN at priority 1 have *hard-real-time framing obligations* (USB SOF / CAN bit-timing windows). Same-priority with the motion pair means a long motion ISR can delay a USB/CAN ISR (they can't preempt each other), risking USB-CDC stalls / CAN bus-off in pathological cases. Also, sharing the level re-creates a milder version of the very inversion we are fixing (motion vs. comms contention), just one rung up.

- **Scheme B — demote the scheduler below the motion pair (DECIDED).** With 16 NVIC levels available and only 0/1/2 in use, **keep the motion pair at the number `2`** it already (half-)occupies and **demote SysTick from 2 to 3** — free the `2` band of the scheduler by pushing the scheduler down. Result:

  | Priority | Owner | Change |
  |---|---|---|
  | 0 | USART/serial, bxCAN | unchanged |
  | 1 | USB OTG/FS, FDCAN | unchanged |
  | **2** | **TIM5 (producer) + step-output timer (consumer)** | TIM5 stays at 2; consumer joins it at 2 |
  | **3** | **SysTick scheduler** (sensors, comms drain, heater `max_duration`, runtime drain) | **demoted from 2 → 3** |

  This is the minimal, safest move: **nothing that was above the motion tick changes** (USB/CAN/serial at 0 and 1 stay strictly above motion, so comms framing is *unaffected* — they keep their hard-real-time precedence). The only relative change is **SysTick now sits below the motion pair**, which is the entire point. The motion pair keeps the number `2` it is already validated at; we are *demoting the scheduler*, not *promoting motion into a comms-shared band*.

**Why Scheme B is safe (the NVIC-map argument):**

- **Serial/CAN (prio 0) and USB/FDCAN (prio 1) remain strictly above motion (prio 2).** They can still preempt the motion ISR. So nothing with hard-real-time framing is demoted below motion. *Call-out:* USB-CDC and CAN keep their precedence — comms-being-below-motion is **not** what happens in Scheme B; only the *software scheduler* drops below motion.
- **The fault/exception handlers (HardFault, NMI, BusFault, MemManage, UsageFault) are negative-priority on Cortex-M and always above any IRQ** — unaffected by any of this.
- **The only thing that can now be *delayed* by a long motion ISR is the SysTick scheduler (prio 3).** That is acceptable *and bounded* because the motion ISR's own self-budget fault (§5) caps how long it may run; and the scheduler's own missed-deadline machinery (`try_shutdown("Rescheduled timer in the past")`, the `max_duration` chain, IWDG) is the backstop if motion ever monopolizes the core (a fail-loud outcome, which is correct per project policy).

> Implementation note: SysTick priority is set via `SCB->SHP` (system-handler priority), not the NVIC IRQ table, but `NVIC_SetPriority(SysTick_IRQn, n)` is the CMSIS call that does this and is exactly what `armcm_timer.c:50` already uses. Demoting it is a one-line change of the constant `2` → `3` there (or a `#define` so both this file and the motion backend reference one symbol — see §4).

### 2.2 The dedicated step-output timer (consumer relocation)

The consumer logic is unchanged in *meaning* (peek head per owned axis; if due/late, pop + `runtime_emit_step_pulses`; compute soonest next waketime across owned axes). What changes is the **dispatch substrate**: instead of one Klipper `struct timer` per axis on the SysTick queue, a single **dedicated hardware timer** fires the consumer at priority P=2, and the consumer is responsible for **reprogramming that timer's compare register to the soonest pending step across all owned axes**.

- **One hardware timer, all owned axes.** A single timer ISR (`STEP_OUT_TIM_IRQHandler`, C) calls one Rust entry point (e.g. `kalico_step_output_event()`), which scans the owned-axis step queues, emits all due/late steps (bounded per dispatch, mirroring `MAX_STEPS_PER_EVENT`), and returns the **absolute cycle of the soonest remaining head across all owned axes** (or an idle-park value if all empty). The C ISR loads that into the timer's compare register and re-arms the one-shot.
- This collapses the 4 per-axis SysTick timers + their independent reschedules into one hardware-timed scheduler — and removes the "unowned-axis timer adds sample-rate dispatch load" hazard that `arm_per_axis_step_timer`'s comment (`runtime_tick.c:644-657`) describes (that hazard was itself a -311 contributor). With a hardware compare timer, an empty/unowned axis costs nothing.
- **The "kick" becomes trivial.** Today `kalico_kick_per_axis_timer` does `sched_del_timer` + `sched_add_timer` from the TIM5 ISR to pull a parked consumer forward. With the dedicated timer at the **same** priority as TIM5 (so no preemption mid-update), the kick is just: *compare the new step's cycle against the timer's current compare; if sooner, rewrite the compare register* (a couple of register writes, no list surgery). Same-priority is what makes this a non-racing register poke rather than a lock-protected one.

### 2.3 Which hardware timer per MCU

Both H723 and F446 expose the **32-bit general-purpose timers TIM2 and TIM5**; all other GP timers (TIM3/4 16-bit, TIM9–14, etc.) are 16-bit. TIM5 is the motion-sample tick. **TIM2 is the natural choice for the step-output timer on both families**: it is 32-bit (matching the `u32 cycle_abs` compare range of `StepEntry`, so the one-shot compare needs no wrap gymnastics within a sample horizon), present and identical on both families, and not claimed by the motion engine.

- **STM32H723 (main):** **TIM2** (32-bit), `TIM2_IRQn`, set to priority 2.
- **STM32F446 (bottom):** **TIM2** (32-bit), `TIM2_IRQn`, set to priority 2.

Caveat to verify at implementation time (Plan Task 0): TIM2 must not be claimed by a configured **hardware-PWM pin** on the active board config. Klipper allocates a TIMx to a PWM pin on demand (`src/stm32/hardware_pwm.c` + the gpio timer-pin map). On the Octopus Pro / Manta configs in use, heaters/fans are driven by other timers (TIM1/3/4/8/12/15 appear in the pin tables), but the plan's first task is an explicit objdump/config audit confirming TIM2 is free on *each* flashed config. **If TIM2 is taken on a given board, fall back to the next free 32-bit timer; if none, fall back to a free 16-bit GP timer and constrain the one-shot horizon** (see §6 risk on 16-bit wrap). The per-family backends already differ (`runtime_tick_h7.c` vs `runtime_tick_f4.c`), so a different TIMx per MCU is permitted and cheap.

> 32-bit is strongly preferred: with `cycle_abs` a `u32` MCU-clock value and sample horizons far under 2^31 cycles, a 32-bit compare timer reprograms to any near-future step directly. A 16-bit timer can only express ~65535 ticks of horizon (≈ 238 µs @ 275 MHz on H7), forcing a "re-arm in chunks" loop for far-future parks — workable but more code and a wrap hazard.

---

## 3. C / Rust split (per the boundary doc)

| Piece | Language | Where |
|---|---|---|
| Step-output hardware timer setup (clock enable, 32-bit, one-shot/up-counter, IRQ enable at prio 2) | **C** | `src/stm32/runtime_tick_timer.h` (shared template) — new `step_output_timer_init()` |
| `STEP_OUT_TIM_IRQHandler` (clear flag, call Rust, load returned compare, re-arm) | **C** | same template |
| Demote SysTick 2→3; raise/confirm TIM5 + step-output timer at 2 | **C** | `armcm_timer.c` (SysTick), `runtime_tick_timer.h` (TIM5, step-out timer); shared `#define KALICO_MOTION_NVIC_PRIO 2` / `KALICO_SCHED_NVIC_PRIO 3` |
| "Which axis next / pop+emit / soonest next waketime" | **Rust** | `rust/runtime/src/per_axis_timer.rs` — generalize `kalico_per_axis_step_event(axis)` into `kalico_step_output_event()` (scan owned axes, return soonest) |
| The kick: compare-and-reprogram the step-output compare register | **C** (called from Rust TIM5 path via `extern "C"`) | `runtime_tick.c` — `kalico_kick_step_output(cycle_abs)` replaces `kalico_kick_per_axis_timer(axis, waketime)` |
| Step-queue SPSC | **Rust accessor over C `.bss`** (unchanged) | `step_queue.rs` / `src/kalico_step_queue.c` |

The seam stays `extern "C"` + `#[repr(C)]` only. No new shared *struct* crosses the boundary — only the existing `step_queues[]` (already C-owned) and two scalar `extern "C"` function calls (`kalico_step_output_event` C→Rust, `kalico_kick_step_output` Rust→C).

---

## 4. TIM5 self-budget fault

- **Where measured:** inside `TIM5_IRQHandler` (`runtime_tick_timer.h`), at the existing `runtime_bench_capture_enter()` / `..._exit()` bracket (which already exists for exactly this and is the documented TODO site). Read `runtime_cyccnt_read()` (or `DWT->CYCCNT` directly) on entry and exit; `spent = exit - entry`.
- **Threshold:** a configurable fraction of the sample period — start at **50 %** of `modulation_tick_interval` (the per-MCU reload value already computed in `runtime_tick_init`, = `timer_clk / CONFIG_KALICO_MOTION_SAMPLE_RATE_HZ`). 50 % gives generous headroom for the consumer + comms while still catching a runaway ISR before it eats the next tick. Expose as a Kconfig (`CONFIG_KALICO_TICK_BUDGET_PERCENT`, default 50) so it is tunable per family without code edits.
- **Which FaultCode:** reusing the **existing `-311 TickIntervalExceeded`** semantics is *wrong* here — that code means "the tick was late" (an *observed* gap), whereas the self-budget is "the tick took too long" (a *cause*). Add a **new, distinct** `FaultCode::TickBudgetExceeded` to `rust/runtime/src/error.rs` at the **next free negative code** — note `-312` is already taken by `PieceRingEmpty`, so use **`-313`** (or the next gap after the tick-reserved block). This lets host triage tell "I was starved" (-311) from "I starved myself / others" (-313). Mirror the addition in `rust/kalico-c-api/include/kalico_runtime.h` (the header comment in `error.rs:4-5` requires this for any code crossing the FFI). Both share the existing fault-emission path (`runtime_drain` → `kalico_native_emit_fault_event` → `[KALICO-FAULT]` host journal log). The over-budget ISR latches the new code via the same `raise_fault(code: i32)` mechanism used by `tick.rs`; escalation to Klipper shutdown then happens in the foreground `runtime_drain` exactly as today (the spec-2026-05-28 hard-fault path), so no `shutdown()`/`longjmp` from the ISR.

---

## 5. Fail-loud points (project policy: fail loudly, don't recover)

- **Step queue full** (`step_queue::push` returns `Err(StepQueueFull)`): a dropped push is a silent lost step → motion corruption. Under this redesign, a full queue means the consumer (now at prio 2) is being starved by the producer at the same prio — which should not happen if budgets hold. Make `push`-returns-`Err` a **hard fault** using the **already-defined `FaultCode::StepQueueFull = -304`** (no new code needed), raised from the TIM5 path and escalated via `runtime_drain`. **Verify at implementation time** whether the current TIM5 push path already calls `raise_fault(StepQueueFull)` on `Err` or silently drops; if it drops, that is the gap this point closes.
- **TIM5 self-budget exceeded** → new `TickBudgetExceeded` (`-313`), §4.
- **Late tick** → `TickIntervalExceeded` (-311), unchanged — but should become *rare* after the lift; its continued presence post-change is the regression signal.
- **Step-output timer ISR over-budget** (the consumer monopolizing prio 2): out of scope to fault on directly in v1, but the TIM5 self-budget already catches the aggregate-motion-starves-scheduler case indirectly (TIM5 will be late if the consumer hogs prio 2, raising -311). Flag for §6.

---

## 6. Risks & open questions

1. **The priority flip itself (highest risk).** Demoting SysTick below the motion pair changes the scheduling order of *every* Klipper task. If the motion ISRs ever run long (a bug, a pathological piece), they now starve the *entire* scheduler — sensors, comms, **and the heater `max_duration` safety timer** (§1.4 / §7). The mitigations are the TIM5 self-budget fault (§4) and the IWDG backstop (§7), but the failure mode is more severe than today. **Mitigation in the plan: stand the change up incrementally (Plan §Incremental) and never flip priority until the dedicated timer has been proven at parity at the *same* priority as SysTick first.**
2. **TIM2 availability per board config.** TIM2 might be claimed by a hardware-PWM pin on a specific flashed config. Plan Task 0 audits this via objdump/config on *both* MCUs before any code. Fallback path defined in §2.3.
3. **16-bit timer wrap** (only if TIM2 is unavailable and a 16-bit GP timer must be used): the one-shot compare horizon is limited (~238 µs @ 275 MHz), forcing a chunked re-arm for idle-park intervals and introducing a wrap-comparison hazard in the kick (compare-against-current must be wrap-safe `i32` deltas). Avoided entirely if a 32-bit timer is free (the expected case).
4. **Per-axis next-step scheduler under u32 wrap.** The "soonest across owned axes" computation must use wrap-safe signed-delta comparison (as `per_axis_timer.rs` already does for the due window). With the single shared timer scanning all axes, an off-by-one in the wrap comparison could pick the *wrong* axis as soonest → a step fires late. Covered by host unit tests (Plan test suite).
5. **Kick vs. self-reprogram race at same priority.** The kick (Rust TIM5 path → C `kalico_kick_step_output`) and the step-output ISR both write the compare register. They are same-priority and can't preempt each other, so it's non-racing *as long as TIM5 cannot interrupt the step-output ISR and vice versa* — which holds at equal priority. Document this as the same invariant as §1.3's SPSC invariant: equal priority is load-bearing.
6. **USB-CDC under scheduler demotion.** USB OTG stays at prio 1 (above motion), so enumeration/framing are safe; but the *foreground USB-CDC drain* (the task that empties the TX buffer) runs under SysTick (now prio 3). Under sustained motion load, host-bound telemetry/log frames could back up more than today. Likely benign (the drain is foreground, motion ISRs are bounded), but flag for bench observation: watch for `[KALICO-FAULT]` frame loss or status-frame gaps.

---

## 7. Heater-safety backstop chain (DOCUMENT ONLY — review later)

Per the task, this section **documents the current backstop and flags residual risk**; it does **not** design new heater safety.

**Current chain, once SysTick is demoted below the motion pair:**

1. **`max_duration` timer (now at prio 3, SysTick).** Each heater/PWM output arms a `struct timer` (`gpiocmds.c`, `sched_add_timer` ×3) that calls `shutdown(...)` if not refreshed by `end_time`. **Residual risk:** this timer now runs *below* the motion pair. If the motion ISRs monopolize the core (a bug not caught by the self-budget fault), the `max_duration` timer could be delayed — weakening the *software* runaway backstop. In normal operation the motion ISRs are bounded (self-budget fault, §4) and leave ample slack at prio 3, so the `max_duration` timer fires on time; the concern is only the pathological-starvation case.
2. **IWDG (~0.5 s hardware watchdog), `src/stm32/watchdog.c` (→ `generic/armcm_watchdog.c`), kicked from the foreground.** The foreground kick is gated by `runtime_liveness_ok` (cleared on fault/stall, `runtime_tick.c`). If the scheduler (and thus the foreground) is starved hard enough that the IWDG isn't kicked, the **hardware resets the MCU within ~0.5 s**, which drives all GPIOs to their reset (safe-off) state. This is the hardware backstop that survives *any* software starvation, including a motion ISR monopolizing the core — it is **above** all software priority and cannot be demoted.
3. **GPIO reset state.** On `shutdown()` the `DECL_SHUTDOWN` handlers force outputs to `default_value` (`gpio_out_write`); on IWDG reset the silicon forces pins to their reset state (inputs/Hi-Z → heater MOSFETs off given correct board pull-down design).

**Net:** the *software* runaway backstop (`max_duration`) is now nominally below motion (residual risk #1), but the *hardware* backstop (IWDG ~0.5 s → reset → safe pins) is unaffected and remains the true last line of defense. **Flagged for a dedicated safety review** before relying on this in production: quantify worst-case motion-ISR occupancy at prio 2 vs. the prio-3 `max_duration` timer's deadline margin, and confirm the IWDG kick cannot be indefinitely deferred by motion load (the self-budget fault should prevent it, but the interaction deserves explicit review).

---

## 8. What changes (file inventory)

### 8.1 C (`src/`)
- `src/generic/armcm_timer.c`: SysTick priority `2 → 3` (via shared `#define`).
- `src/stm32/runtime_tick_timer.h`: TIM5 priority via shared `#define` (stays 2); add `step_output_timer_init()` + `STEP_OUT_TIM_IRQHandler`; add the self-budget measurement + `TickBudgetExceeded` latch in `TIM5_IRQHandler`.
- `src/runtime_tick.c`: replace `arm_per_axis_step_timer` / `per_axis_timer_event_N` / `kalico_kick_per_axis_timer` with the single step-output-timer wiring + `kalico_kick_step_output(cycle_abs)`; keep `arm_per_axis_step_timer`'s owned-axis-mask concept (now "owned axis set" feeding the soonest-across-owned scan).
- A shared header `#define KALICO_MOTION_NVIC_PRIO 2` / `KALICO_SCHED_NVIC_PRIO 3` so the two numbers live in one place.

### 8.2 Rust (`rust/`)
- `rust/runtime/src/per_axis_timer.rs`: generalize `kalico_per_axis_step_event(axis)` → `kalico_step_output_event()` (scan owned axes, emit due/late, return soonest-across-owned next-waketime).
- `rust/runtime/src/error.rs`: add `TickBudgetExceeded` (`-313`; `-312` is taken by `PieceRingEmpty`). Reuse existing `StepQueueFull = -304` for the overflow fail-loud — no new overflow code. Mirror new codes into `rust/kalico-c-api/include/kalico_runtime.h`.
- `rust/runtime/src/step_queue.rs`: no logic change; **strengthen the invariant comment** to name the dedicated-step-output-timer same-priority dependency explicitly.

### 8.3 Unchanged / out of scope
- The step-queue *storage* (C `.bss`) and SPSC mechanism (stays non-racing u32 — invariant preserved).
- USB/CAN/ADC priorities (0 and 1) — untouched.
- Heater safety *design* — documented (§7), not redesigned.
- IWDG — unchanged.
