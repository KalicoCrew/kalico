// src/stm32/runtime_tick_h7.c
//
// H723-specific TIM5 init + IRQ handler. Spec §2.4 / §4.1 / §4.2 / §4.4.

#include "autoconf.h"
#include "generic/armcm_boot.h" // DECL_ARMCM_IRQ
#include "internal.h"          // STM32-internal helpers — TIM5, RCC, DWT
#include "kalico_runtime.h"
#include "generic/runtime_tick.h"   // interface contract
#include "generic/runtime_bench.h" // runtime_bench_capture hook

#if CONFIG_MACH_STM32H7

extern const uint32_t runtime_clock_freq;

extern void* runtime_handle;   // exposed in src/runtime_tick.c

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
// widening loop in sim. Fork to a software counter (runtime_sim_cyccnt) bumped
// from the TIM5 ISR. Production firmware (CONFIG_KALICO_SIM=n) reads the
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
    // Spec §3.2 (step-emission-architecture, T9): TIM5 is enabled iff at
    // least one stepper on this MCU runs in Modulated mode (phase stepping
    // / polled-tick StepAccumulator). For the all-StepTime MVP, TIM5 stays
    // off entirely — the engine state machine is no longer TIM5-driven:
    //   - Segment dequeue + retirement run on the producer Klipper timer
    //     (`src/runtime_tick.c`, T8).
    //   - GPIO step pulses fire from per-stepper consumer Klipper timers
    //     (`step_time_event`, T7) keyed off Newton-iterated waketimes.
    //   - Widened MCU clock for `clock_sync_respond` is computed on-demand
    //     via `runtime_handle_widened_now` (T6), no seqlock seeding needed.
    if (!runtime_handle) {
        return;
    }

    if (kalico_runtime_count_modulated_steppers(runtime_handle) == 0) {
        // No phase-stepping / polled-tick consumers — TIM5 stays disabled.
        return;
    }

    // 2026-05-19: idempotent re-arm guard. push_segment_impl calls this on
    // every successful enqueue so the first segment lazily starts TIM5; if
    // TIM5 is already counting (CR1.CEN==1), short-circuit to avoid the
    // disable→reenable glitch and the dead-cycle USB-CDC bandwidth cost.
    // The pre-2026-05-19 path was called from configure_axes_blob alone,
    // which armed TIM5 immediately on connect — even before any segment
    // existed — so the ISR fired at 40 kHz writing zero-delta XDIRECT to
    // SPI3 for the entire idle period, starving the USB CDC pump and
    // eventually causing "No such device" disconnects under sustained load.
    if (TIM5->CR1 & TIM_CR1_CEN) {
        return;
    }

    // 2026-05-19: lower modulation rate from 40 kHz → 10 kHz to leave
    // CPU headroom for the USB CDC pump during motion. Per-tick ISR cost
    // (one round-robin SPI write + 4 modulator computes) at 40 kHz left
    // ~20% CPU for USB which was enough to drop the bulk-out endpoint
    // under any sustained jog. At 10 kHz the same per-tick work yields
    // ~80% CPU free for USB and other tasks. Per-motor write rate drops
    // from 10 kHz → 2.5 kHz (round-robin across 4 motors), which is
    // still above mass3d's "smooth" threshold for slow jogs but limits
    // top speed — fine as a v1 mitigation; the long-term answer is
    // either DMA SPI or per-tick write-coalescing.
    TIM5->CR1 &= ~TIM_CR1_CEN;
    TIM5->ARR  = (runtime_clock_freq / 10000U) - 1U;
    TIM5->EGR  = TIM_EGR_UG;
    TIM5->SR   = 0;
    TIM5->SR   = ~TIM_SR_UIF;     // clear stale UIF before enabling
    TIM5->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM5_IRQn);
}

void
runtime_tick_init(void)
{
    // Disable IRQ at the NVIC first — that's safe even with TIM5 clock off,
    // since NVIC is core-local. Touching TIM5 registers before its peripheral
    // clock is enabled raises a bus fault on H7 (caused first-light hangs in
    // early bring-up, manifesting as USB-CDC enumerating briefly then the MCU
    // resetting in a loop). So clock-on must come first.
    NVIC_DisableIRQ(TIM5_IRQn);

    // Enable TIM5 peripheral clock. APB1L bus, gated by RCC. The RMB barrier
    // (DSB) ensures the clock is up before subsequent register accesses.
    RCC->APB1LENR |= RCC_APB1LENR_TIM5EN;
    __DSB();

    // Now safe to touch TIM5 registers. Per spec §2.4 init invariant: clear
    // CEN + SR.UIF before any path could fire.
    TIM5->CR1 &= ~TIM_CR1_CEN;
    TIM5->SR = 0;

    // 10 kHz tick (see comment in runtime_tick_enable for rationale).
    // PSC = 0, ARR = (clock_freq / 10000) - 1.
    TIM5->PSC = 0;
    TIM5->ARR = (runtime_clock_freq / 10000U) - 1U;

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
    // soundly — neither can preempt the other. Promoting from 3 to 2 trades
    // at most "TIM5 ISR worst-case duration" of scheduler-dispatch latency,
    // which is bounded by motion correctness (must fit in the 25 µs
    // modulation period at 40 kHz) and orders of magnitude below the 3 s
    // heater deadline / 500 ms IWDG window.
    NVIC_SetPriority(TIM5_IRQn, 2);

    // Don't enable yet — runtime_init pushes segments first; first push triggers
    // runtime_tick_enable() via the producer protocol.
}

void
TIM5_IRQHandler(void)
{
    // Diag instrumentation: cycle stamp at IRQ entry. DWT->CYCCNT is
    // already enabled in this file's init (TRCENA + CYCCNTENA above).
    extern void diag_tim5_account(uint32_t enter, uint32_t exit);
    uint32_t diag_enter = DWT->CYCCNT;

    TIM5->SR = ~TIM_SR_UIF;            // entry-time ack (spec §2.4)

#if CONFIG_KALICO_SIM
    // Step-6 spec §3.1: Renode's H7 model returns 0 for DWT->CYCCNT, so the
    // engine widening loop has no forward progress source unless we drive a
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
    // period. No-op when no arm is active (table empty). Skipped under
    // CONFIG_KALICO_SIM: the Renode e2e test drives pin levels directly
    // via `command_runtime_sim_endstop_set_pin`, and a real-GPIO sample
    // here would clobber the test's override every tick.
#if !CONFIG_KALICO_SIM
    extern void runtime_endstop_sample_pins(void);
    runtime_endstop_sample_pins();
#endif

    // T10 (spec §3.2): TIM5 dispatches the Modulated polled-tick
    // StepAccumulator path exclusively. The legacy engine state machine
    // (`runtime_handle_tick` → `Engine::tick`) no longer runs from
    // TIM5; segment dequeue + retirement are driven by the producer
    // Klipper timer in `src/runtime_tick.c` (T8) and per-stepper step
    // pulses fire from the consumer Klipper timers (T7). The Modulated
    // entry computes its own widened clock from `timer_read_time` +
    // `stats_send_time*`, so no CYCCNT widening seed is needed here.
    uint32_t before = runtime_cyccnt_read();
    if (runtime_handle) {
        kalico_runtime_modulated_tick(runtime_handle);
    }
    uint32_t after = runtime_cyccnt_read();

    // Bench capture: weak no-op unless CONFIG_RUNTIME_BENCH=y.
    runtime_bench_capture(after - before);
    // No late ack.

    // Histogram the modulated-tick subwindow before the full-IRQ
    // accounting at exit. Distinguishing the two tells us whether the IRQ
    // cost lives in the engine evaluator or in the surrounding ISR overhead
    // (endstop sample, accounting, vector entry/exit).
    extern void diag_runtime_tick_account(uint32_t cycles);
    diag_runtime_tick_account(after - before);

    // Diag accounting at IRQ exit. Cost: ~10 cycles (DWT read + 3 mem
    // increments + max compare + threshold check). Negligible at 40 kHz.
    diag_tim5_account(diag_enter, DWT->CYCCNT);
}

// Klipper's IRQ vector-table dispatch is generated by scripts/buildcommands.py
// from DECL_ARMCM_IRQ entries. Without this, TIM5_IRQHandler will not be wired
// into the vector table and the IRQ silently drops.
DECL_ARMCM_IRQ(TIM5_IRQHandler, TIM5_IRQn);

#endif
