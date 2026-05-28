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

#if CONFIG_MACH_STM32F4

extern const uint32_t runtime_clock_freq;

extern void* runtime_handle;   // exposed in src/runtime_tick.c

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
    // TIM5 is enabled iff count_modulated_steppers > 0 (phase-stepping
    // consumer present). Mirrors runtime_tick_h7.c; see that file for
    // the full rationale.
    //
    // F4 today has no phase-stepped axis, so this gate keeps TIM5 off
    // entirely. The drain task disables TIM5 on the Drained transition.
    //
    // Remaining ISR responsibilities are unchanged from before — see the
    // bullet list in runtime_tick_h7.c::runtime_tick_enable.
    if (!runtime_handle) {
        return;
    }

    if (kalico_runtime_count_modulated_steppers(runtime_handle) == 0) {
        // No phase-stepping consumers — TIM5 stays disabled. The next
        // set_step_mode call will re-enter and arm TIM5.
        return;
    }

    // T17: TIM5 rate is set by `CONFIG_KALICO_MOTION_SAMPLE_RATE_HZ`
    // (Task 1 Kconfig; default 20 kHz on F4). See
    // runtime_tick_init for the peripheral-clock + DWT setup and the
    // F446 CPU-budget rationale that anchors the default.
    TIM5->CR1 &= ~TIM_CR1_CEN;
    TIM5->ARR  = (runtime_clock_freq / CONFIG_KALICO_MOTION_SAMPLE_RATE_HZ) - 1U;
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
    NVIC_SetPriority(TIM5_IRQn, 2);

    // Don't enable yet — runtime_init pushes segments first; first push triggers
    // runtime_tick_enable() via the producer protocol.
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

#endif
