// src/stm32/runtime_tick_f4.c
//
// STM32F4-specific TIM5 init + IRQ handler. Spec §2.4 / §4.1 / §4.2 / §4.4.
// Mirrors runtime_tick_h7.c; the only family-level difference is the RCC
// clock-enable register (F4 has a single APB1ENR vs H7's split APB1LENR).

#include "autoconf.h"
#include "generic/armcm_boot.h" // DECL_ARMCM_IRQ
#include "internal.h"          // STM32-internal helpers — TIM5, RCC, DWT
#include "kalico_runtime.h"
#include "generic/runtime_tick.h"   // interface contract
#include "generic/runtime_bench.h" // runtime_bench_capture hook
#include "generic/kalico_nvic_prio.h" // KALICO_MOTION_NVIC_PRIO

#if CONFIG_MACH_STM32F4

extern const uint32_t runtime_clock_freq;

extern void* runtime_handle;   // exposed in src/runtime_tick.c

// Dedicated step-output timer (TIM2, 32-bit) setup. Defined later in this file;
// forward-declared so runtime_tick_init can stand it up after arming TIM5.
static void step_output_timer_init(void);

// Stepping-redesign Task 17: TIM5 ISR body. The canonical prototype for
// `kalico_runtime_tick_sample` is supplied by the included
// `kalico_runtime.h`; no local extern needed.

// These three are referenced ONLY from Rust (kalico-c-api's runtime_ffi.rs),
// not from any C translation unit. Klipper builds with `-fwhole-program -flto`
// which would otherwise treat them as internal and either inline them or
// strip them, leaving the Rust archive with unresolved references. The
// `used, externally_visible` attribute pair survives LTO + whole-program +
// --gc-sections, mirroring runtime_clock_freq / runtime_liveness_ok.
__attribute__((used, externally_visible))
void
runtime_tick_disable(void)
{
    TIM5->CR1 &= ~TIM_CR1_CEN;
    NVIC_DisableIRQ(TIM5_IRQn);
}

// Helper for Rust's CYCCNT widen-reinit on producer-driven re-enable path.
//
// Per Step-6 spec §3.1: under CONFIG_KALICO_SIM, Renode's H7 .repl tags
// DWT->CYCCNT as opaque memory (reads return 0), which freezes the engine's
// widening loop in sim. The F4 sim model behaves the same way, so we mirror
// the H7 fork verbatim. Production firmware (CONFIG_KALICO_SIM=n) reads the
// hardware DWT register directly. NEVER flash a CONFIG_KALICO_SIM=y image to
// silicon — IWDG-disable + software CYCCNT is a debugging build only.
__attribute__((used, externally_visible))
uint32_t
runtime_cyccnt_read(void)
{
#if CONFIG_KALICO_SIM
    extern volatile uint32_t runtime_sim_cyccnt;
    return runtime_sim_cyccnt;
#else
    return DWT->CYCCNT;
#endif
}

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

void
runtime_tick_init(void)
{
    // Disable IRQ at the NVIC first — that's safe even with TIM5 clock off,
    // since NVIC is core-local. Touching TIM5 registers before its peripheral
    // clock is enabled raises a bus fault on STM32 (caused first-light hangs
    // in early H7 bring-up); same hazard applies to F4. Clock-on must come
    // first.
    NVIC_DisableIRQ(TIM5_IRQn);

    // Enable TIM5 peripheral clock. APB1 bus, gated by RCC. The DSB barrier
    // ensures the clock is up before subsequent register accesses.
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    __DSB();

    // Now safe to touch TIM5 registers. Per spec §2.4 init invariant: clear
    // CEN + SR.UIF before any path could fire.
    TIM5->CR1 &= ~TIM_CR1_CEN;
    TIM5->SR = 0;

    // F4 band-aid (2026-05-12): 10 kHz tick instead of 40 kHz.
    //
    // F446 at 180 MHz can't sustain the runtime engine eval (~24 µs avg per
    // call for a degree-3 NURBS with 64 control points) at 40 kHz: each TIM5
    // fire takes 50+ µs, the IRQ tail-chains continuously, foreground is
    // starved for >1 second and IWDG fires at 511 ms. Captured via F4
    // prior_diag bench 2026-05-12 (tim5_max_cyc=10420, out_max_gap=191M
    // cycles, eval avg=4304 cycles, hist bucket 2 = 11290 fires at 45-68 µs).
    //
    // 10 kHz puts the IRQ at ~50% CPU and gives foreground the other ~50%,
    // which is enough for usb_bulk_out_task + watchdog kick. Step jitter
    // becomes 100 µs (vs 25 µs at 40 kHz), invisible on Z which doesn't do
    // phase stepping. The runtime engine handles multistepping per tick (one
    // ISR emits as many step pulses as the accumulator crossed), so max step
    // rate is not bound by tick rate.
    //
    // PROPER FIX: per-stepper StepTime scheduling for non-phase-stepped axes.
    // Spec: docs/superpowers/specs/2026-05-12-step-time-scheduling-design.md
    // (forthcoming). Once that lands, this rate can move back to whatever
    // makes sense for any phase-stepped axes hosted on F4 in the future
    // (none today), or TIM5 can be disabled entirely.
    TIM5->PSC = 0;
    TIM5->ARR = (runtime_clock_freq / CONFIG_KALICO_MOTION_SAMPLE_RATE_HZ) - 1U;

    // Auto-reload, update interrupt enable.
    TIM5->CR1 = TIM_CR1_ARPE;
    TIM5->DIER = TIM_DIER_UIE;

    // Enable DWT cycle counter for raw_cyccnt reads in the ISR.
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // Set IRQ priority 2 — same as SysTick (the Klipper scheduler dispatch
    // ISR, set in armcm_timer.c). Same-priority Cortex-M interrupts do not
    // nest: whichever fires first runs to completion before the other.
    // This is the mutual-exclusion guarantee that lets TIM5 and the
    // SysTick-dispatched `runtime_producer_event` both form `&mut IsrState`
    // soundly — neither can preempt the other. See the matching comment in
    // runtime_tick_h7.c for the full rationale.
    //
    // KALICO_MOTION_NVIC_PRIO (= 2 today) is shared with the dedicated
    // step-output timer (step_output_timer_init below) so producer and
    // consumer are EQUAL — the non-nesting invariant the step_queue SPSC
    // relies on. See src/generic/kalico_nvic_prio.h.
    NVIC_SetPriority(TIM5_IRQn, KALICO_MOTION_NVIC_PRIO);

    // Always-on (spec 2026-05-28): the piece-ring engine has no per-push event
    // to lazily start TIM5 (segments are gone), so the ISR free-runs from boot.
    // It idles cheaply when no axis has an active piece. TIM5 is disabled only
    // when the firmware stops motion (Klipper shutdown) and re-armed here on
    // FIRMWARE_RESTART.
    TIM5->EGR  = TIM_EGR_UG;
    TIM5->SR   = ~TIM_SR_UIF;     // clear stale UIF before enabling
    TIM5->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM5_IRQn);

    // Stand up the dedicated step-output timer (motion-tick priority-lift
    // Step 1). It free-runs disabled until the first owned step arrives.
    step_output_timer_init();
}

void
TIM5_IRQHandler(void)
{
    extern void diag_tim5_account(uint32_t enter, uint32_t exit);
    uint32_t diag_enter = DWT->CYCCNT;

    TIM5->SR = ~TIM_SR_UIF;            // entry-time ack (spec §2.4)

#if CONFIG_KALICO_SIM
    // Step-6 spec §3.1: Renode returns 0 for DWT->CYCCNT, so the engine
    // widening loop has no forward progress source unless we drive a
    // software counter from this ISR. Delta = cycles-per-tick so the widened
    // u64 advances at the same rate the engine would observe on silicon.
    extern volatile uint32_t runtime_sim_cyccnt;
    runtime_sim_cyccnt += (runtime_clock_freq / 40000U);
    // Sim-only wake of the drain task; the foreground timer system is
    // unreliable under Renode (DWT-based dispatch) so we drive the drain
    // cadence deterministically off TIM5 fires. Throttled in runtime_tick.c.
    extern void runtime_sim_isr_wake_drain(void);
    runtime_sim_isr_wake_drain();
#endif

    // Step 7.5 — sample any armed endstop GPIOs before the engine tick so
    // `endstop::tick` observes fresh pin levels in the same modulation
    // period. No-op when no arm is active (table empty).
    extern void runtime_endstop_sample_pins(void);
    runtime_endstop_sample_pins();

    // T17 (stepping-redesign): TIM5 dispatches the unified per-sample
    // evaluator `kalico_runtime_tick_sample`, mirroring H7. F4 today
    // never enables TIM5 (no phase-stepped axes), but the wiring stays
    // symmetric with H7 so a future Modulated motor on F4 picks up the
    // same path automatically.
    uint32_t before = runtime_cyccnt_read();
    if (runtime_handle) {
        kalico_runtime_tick_sample(runtime_handle);
    }
    uint32_t after = runtime_cyccnt_read();

    // Bench capture: weak no-op unless CONFIG_RUNTIME_BENCH=y.
    runtime_bench_capture(after - before);
    // No late ack.

    extern void diag_runtime_tick_account(uint32_t cycles);
    diag_runtime_tick_account(after - before);

    diag_tim5_account(diag_enter, DWT->CYCCNT);
}

// Klipper's IRQ vector-table dispatch is generated by scripts/buildcommands.py
// from DECL_ARMCM_IRQ entries. Without this, TIM5_IRQHandler will not be wired
// into the vector table and the IRQ silently drops.
DECL_ARMCM_IRQ(TIM5_IRQHandler, TIM5_IRQn);

// ===========================================================================
// Dedicated step-output timer (TIM2) — motion-tick priority-lift Step 1
// ===========================================================================
//
// Task 0 closed TIM2 for the F446: it is a free 32-bit GP timer. 32-bit means
// the one-shot compare reaches any near-future step directly (no 16-bit
// chaining like the H7's TIM3). Counter mode: free-running 32-bit up-counter at
// the timer clock (PSC = 0, ARR = 0xFFFFFFFF), output-compare channel 1 (CC1)
// as the wake source — arm by writing CCR1 + enabling CC1IE, disable by
// clearing CC1IE. The compare match sets SR.CC1IF and fires TIM2_IRQHandler.
//
// NVIC priority: KALICO_MOTION_NVIC_PRIO — IDENTICAL to TIM5 (Step 1 parity;
// no flip). Same-number Cortex-M interrupts cannot nest, so this consumer and
// the TIM5 producer never preempt each other → the step_queues SPSC and the
// kalico_kick_step_output compare poke stay non-racing.

// `step_out_target`  : absolute cycle the consumer must next fire at (32-bit).
// `step_out_running` : 1 while CC1IE is enabled (timer is arming toward a step).
static volatile uint32_t step_out_target;
static volatile uint8_t  step_out_running;

// (Re)arm the step-output timer to fire at absolute cycle `cycle_abs`, or
// disable it when `cycle_abs == KALICO_STEP_OUTPUT_DISABLE`. Called from the
// Rust kick path (kalico_kick_step_output) and from the IRQ re-arm below.
//
// 32-bit TIM2: the absolute `cycle_abs` IS the compare value, because TIM2->CNT
// runs in the same 32-bit cycle frame as the DWT clock the engine schedules in
// (both PSC = 0 off the same timer clock). A due/late step (compare just behind
// CNT) fires within one full 32-bit wrap, which at these horizons is "next
// tick" in practice; the engine never schedules a step more than a small
// fraction of 2^31 cycles out, so the wrap-safe arm cannot misfire.
//
// used, externally_visible: referenced from the Rust archive (via the C kick
// shim in runtime_tick.c) — keep it past --gc-sections / -fwhole-program LTO.
__attribute__((used, externally_visible))
void
step_output_timer_arm(uint32_t cycle_abs)
{
    if (cycle_abs == 0xFFFFFFFFu /* KALICO_STEP_OUTPUT_DISABLE */) {
        TIM2->DIER &= ~TIM_DIER_CC1IE;
        step_out_running = 0;
        return;
    }
    step_out_target = cycle_abs;
    step_out_running = 1;
    TIM2->CCR1 = cycle_abs;
    TIM2->SR = ~TIM_SR_CC1IF;          // clear stale compare flag
    TIM2->DIER |= TIM_DIER_CC1IE;
}

__attribute__((used, externally_visible))
uint32_t
step_output_timer_armed_target(void)
{
    return step_out_target;
}

__attribute__((used, externally_visible))
uint8_t
step_output_timer_is_running(void)
{
    return step_out_running;
}

static void
step_output_timer_init(void)
{
    NVIC_DisableIRQ(TIM2_IRQn);

    // Enable TIM2 peripheral clock (APB1 on F4) before touching its registers.
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    __DSB();

    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM2->SR = 0;
    TIM2->PSC = 0;
    TIM2->ARR = 0xFFFFFFFFu;            // free-running 32-bit up-counter
    TIM2->CCMR1 = 0;                    // CC1 = output compare, frozen output
    TIM2->CCR1 = 0;
    TIM2->DIER = 0;                     // CC1IE enabled only when armed
    TIM2->CR1 = TIM_CR1_ARPE;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0;
    TIM2->CR1 |= TIM_CR1_CEN;           // counter free-runs; no IRQ until armed

    step_out_running = 0;
    step_out_target = 0;

    NVIC_SetPriority(TIM2_IRQn, KALICO_MOTION_NVIC_PRIO);
    NVIC_EnableIRQ(TIM2_IRQn);
}

void
TIM2_IRQHandler(void)
{
    TIM2->SR = ~TIM_SR_CC1IF;           // ack the compare match

    // Run the Rust consumer: emit due steps, return the soonest remaining head
    // (or KALICO_STEP_OUTPUT_DISABLE), then re-arm / disable accordingly.
    extern uint32_t kalico_step_output_event(void);
    uint32_t next = kalico_step_output_event();
    step_output_timer_arm(next);
}

DECL_ARMCM_IRQ(TIM2_IRQHandler, TIM2_IRQn);

#endif
