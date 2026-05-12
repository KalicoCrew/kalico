// src/stm32/runtime_tick_h7.c
//
// H723-specific TIM5 init + IRQ handler. Spec §2.4 / §4.1 / §4.2 / §4.4.

#include "autoconf.h"
#include "board/misc.h"        // timer_read_time
#include "command.h"           // output() diag emit
#include "generic/armcm_boot.h" // DECL_ARMCM_IRQ
#include "internal.h"          // STM32-internal helpers — TIM5, RCC, DWT
#include "kalico_runtime.h"
#include "generic/runtime_tick.h"   // interface contract
#include "generic/runtime_bench.h" // runtime_bench_capture hook

#if CONFIG_KALICO_RUNTIME && CONFIG_MACH_STM32H7

extern const uint32_t runtime_clock_freq;
// Both from src/basecmd.c. `stats_send_time_high` lags reality by up to one
// u32 wrap between consecutive `stats_update` calls (which run every ~5 s).
// `stats_send_time` is the timer value at the last bump. Klippy's
// `command_get_uptime` reconstructs the lookback-aware high count as
// `stats_send_time_high + (cur < stats_send_time)` — must match exactly here
// or the engine's WidenState lags klippy's `last_clock` by 2^32 cycles.
extern uint32_t stats_send_time_high;
extern uint32_t stats_send_time;

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
    // Step-time scheduling refactor (spec §6.3 follow-up, 2026-05-12 bench
    // wedge): TIM5 must run unconditionally because `Engine::tick` is the
    // only driver of the engine state machine — segment dequeue (Idle →
    // Running), retirement, and `kalico_credit_freed` emission all happen
    // inside the ISR. Conditional-disable when count_modulated == 0
    // stranded segments in the ring (engine stayed Idle), the host's slot
    // pool filled, klippy timed out and reset both MCUs.
    //
    // Per-axis step emission inside `Engine::tick` is now gated by
    // `step_modes[i]`: StepTime axes skip the polled StepAccumulator path
    // entirely; their per-stepper `struct timer` ISR handles GPIO output.
    //
    // 2026-05-13 follow-up: even with the per-axis gate, eval still costs
    // ~8 µs on H7 (4164 cycles measured) inside the TIM5 ISR — running
    // that at 40 kHz with a 25 µs interval saturates the CPU and starves
    // the USB out task (`out_max_gap=7.8s`, `ring_overflow=19k`, klippy
    // timeouts → command_reset wedge, bench 2026-05-13). Drop TIM5 rate
    // to 1 kHz when count_modulated == 0 — Engine::tick still drives the
    // state machine, publishes widened_now (for clock_sync_respond), and
    // retires segments, just at 1 kHz instead of 40 kHz. 1.5% CPU vs
    // 60+%. At 40× lower rate the worst-case Newton solver convergence
    // window for step_time scheduling still holds because step pulses
    // are emitted by the per-stepper struct timer ISR, NOT by TIM5.
    {
        uint32_t target_rate = (kalico_runtime_count_modulated_steppers(runtime_handle) > 0)
            ? 40000U  // 40 kHz when phase-stepping any axis
            : 1000U;  // 1 kHz state-machine-only when all-StepTime
        TIM5->CR1 &= ~TIM_CR1_CEN;
        TIM5->ARR = (runtime_clock_freq / target_rate) - 1U;
        // Force-update prescaler register so the new ARR loads immediately.
        TIM5->EGR = TIM_EGR_UG;
        TIM5->SR = 0;
    }

    // Seed the engine's WidenState to match klippy's widened MCU clock.
    //
    // Klippy widens the 32-bit MCU timer via `stats_send_time_high`, which
    // the firmware increments inside `stats_update` on every observed u32
    // wrap. The engine's `WidenState` starts at `high=0` and only catches
    // up via wraps observed in the ISR — but the first dispatched segment's
    // `t_start_clock` is stamped with klippy's widened view (which already
    // includes accumulated `stats_send_time_high` wraps from the boot →
    // first-push window). Without this seed the engine's `now` sits below
    // the segment's `t_start` for ~half a wrap period (~4 s at 520 MHz) and
    // the curve is evaluated at `u=0` the whole time — segments dequeue,
    // status reads `Running`, but zero step pulses fire ("first motion
    // only energizes" bench symptom, 2026-05-11).
    //
    // Investigation: `docs/superpowers/notes/2026-05-11-first-motion-no-movement-investigation.md`.
    // Linux-sim caller pattern: `src/linux/runtime_tick_host.c:148-156`.
    if (runtime_handle) {
        // Match klippy's command_get_uptime widening exactly: read `cur`
        // first, then compute `high` with the "pre-stats_update wrap" lookback.
        // If we read `high` first and then `cur`, a wrap interleaved between
        // them would yield `high` from before the wrap with `cur` from after,
        // off-by-one in the wrong direction.
        uint32_t low_at_seed = timer_read_time();
        uint32_t high_at_seed = stats_send_time_high
                              + (low_at_seed < stats_send_time);
        uint64_t baseline = ((uint64_t)high_at_seed) << 32
                          | (uint64_t)low_at_seed;
        runtime_handle_seed_widen(runtime_handle, baseline);
        output("widen_seed high=%u low=%u sst=%u sstime=%u",
               high_at_seed, low_at_seed,
               stats_send_time_high, stats_send_time);
    }
    TIM5->SR = ~TIM_SR_UIF;       // clear stale UIF before enabling
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

    // Use the abstraction so cycle-count snapshots stay consistent under the
    // sim fork. On production builds this collapses to a direct DWT->CYCCNT
    // read; under CONFIG_KALICO_SIM both `before` and `after` snapshot the
    // software counter (cycle-bench numbers are explicitly out of scope for
    // sim per spec §3, so the bench buffer becomes meaningless under sim and
    // that is acceptable).
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

    // Histogram the runtime_handle_tick subwindow before the full-IRQ
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

#endif // CONFIG_KALICO_RUNTIME && CONFIG_MACH_STM32H7
