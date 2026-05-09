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

#if CONFIG_KALICO_RUNTIME && CONFIG_MACH_STM32F4

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
    TIM5->SR = ~TIM_SR_UIF;       // clear stale UIF before enabling
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

    // 40 kHz tick: PSC = 0, ARR = (clock_freq / 40000) - 1.
    TIM5->PSC = 0;
    TIM5->ARR = (runtime_clock_freq / 40000U) - 1U;

    // Auto-reload, update interrupt enable.
    TIM5->CR1 = TIM_CR1_ARPE;
    TIM5->DIER = TIM_DIER_UIE;

    // Enable DWT cycle counter for raw_cyccnt reads in the ISR.
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // Set IRQ priority 3 (Cortex-M: lower number = higher urgency).
    // Below SysTick (2) and USB (1) per spec §2.4.
    NVIC_SetPriority(TIM5_IRQn, 3);

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
    // period. No-op when no arm is active (table empty). Skipped under
    // CONFIG_KALICO_SIM: the Renode e2e test drives pin levels directly
    // via `command_runtime_sim_endstop_set_pin`, and a real-GPIO sample
    // here would clobber the test's override every tick.
#if !CONFIG_KALICO_SIM
    extern void runtime_endstop_sample_pins(void);
    runtime_endstop_sample_pins();
#endif

    uint32_t before = runtime_cyccnt_read();
    if (runtime_handle) {
        runtime_handle_tick(runtime_handle, before);
    }
    uint32_t after = runtime_cyccnt_read();

    // Bench capture: weak no-op unless CONFIG_RUNTIME_BENCH=y.
    runtime_bench_capture(after - before);
    // No late ack.

    diag_tim5_account(diag_enter, DWT->CYCCNT);
}

// Klipper's IRQ vector-table dispatch is generated by scripts/buildcommands.py
// from DECL_ARMCM_IRQ entries. Without this, TIM5_IRQHandler will not be wired
// into the vector table and the IRQ silently drops.
DECL_ARMCM_IRQ(TIM5_IRQHandler, TIM5_IRQn);

#endif // CONFIG_KALICO_RUNTIME && CONFIG_MACH_STM32F4
