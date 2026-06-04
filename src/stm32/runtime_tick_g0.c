// src/stm32/runtime_tick_g0.c
//
// STM32G0B1 (Cortex-M0+ / ARMv6-M) motion-engine timer backend. Mirrors
// runtime_tick_f4.c, with two family-level differences forced by the M0+ core:
//
//   1. No TIM5. The motion ISR runs on a configurable basic timer (default
//      TIM7) via the MOTION_TIM* alias in runtime_tick_timer.h. RCC clock-enable
//      is on RCC->APBENR1 (vs F4's APB1ENR / H7's APB1LENR).
//   2. No DWT cycle counter. ARMv6-M has no DWT, so runtime_cyccnt_read() returns
//      a SOFTWARE counter advanced by this ISR. The engine's widened clock
//      (rust/runtime/src/clock.rs) is built on this value, so without it engine
//      time would freeze and no segment would ever advance — it is functional,
//      not just profiling. This is the same mechanism the CONFIG_KALICO_SIM
//      builds use on H7/F4 (Renode also reports DWT->CYCCNT as 0); here it is the
//      production path. Each fire advances the counter by one tick's worth of
//      cycles (clock_freq / sample_rate) so the widened clock tracks real time at
//      the CONFIG_CLOCK_FREQ rate Klipper's own clock uses.
//
// Always-on timer (spec 2026-05-28): mirrors runtime_tick_h7.c /
// runtime_tick_f4.c. The piece-ring engine has no per-push event to lazily
// start the timer; the ISR free-runs from boot and idles cheaply when no axis
// has an active piece. The timer is disabled only on Klipper shutdown
// (runtime_tick_disable) and re-armed in runtime_tick_init on FIRMWARE_RESTART.
//
// See docs/superpowers/specs/2026-05-31-stm32g0-sb2209-target-design.md.

#include "autoconf.h"
#include "generic/armcm_boot.h"     // DECL_ARMCM_IRQ
#include "internal.h"               // CMSIS device header (TIMx, RCC, NVIC)
#include "runtime_tick_timer.h"     // MOTION_TIM* alias
#include "kalico_runtime.h"
#include "generic/runtime_tick.h"   // interface contract

#if CONFIG_MACH_STM32G0

extern const uint32_t runtime_clock_freq;

extern void* runtime_handle;   // exposed in src/runtime_tick.c

// Software cycle counter standing in for DWT->CYCCNT (absent on Cortex-M0+).
// Single-writer: only MOTION_TIM_IRQHandler advances it. `volatile` so the
// foreground widen-read in runtime_cyccnt_read() observes ISR updates. Reads of
// an aligned 32-bit word are atomic on ARMv6-M, so no lock is needed.
static volatile uint32_t runtime_g0_sw_cyccnt;

// Per-tick cycle increment for the software counter: clock_freq / sample_rate.
// Computed once; runtime_clock_freq is a link-time constant (= CONFIG_CLOCK_FREQ)
// and the sample rate is a compile-time Kconfig value.
#define RUNTIME_G0_CYC_PER_TICK \
    (CONFIG_CLOCK_FREQ / CONFIG_KALICO_MOTION_SAMPLE_RATE_HZ)

// These entry points are referenced ONLY from the Rust staticlib, not from any
// C translation unit, so they must survive -fwhole-program LTO + --gc-sections.
// See runtime_tick_f4.c for the full attribute rationale.
__attribute__((used, externally_visible))
void
runtime_tick_disable(void)
{
    MOTION_TIM->CR1 &= ~TIM_CR1_CEN;
    NVIC_DisableIRQ(MOTION_TIM_IRQn);
}

// On Cortex-M0+ there is no DWT, so the widening clock reads the software
// counter the ISR maintains. (Unlike H7/F4 there is no CONFIG_KALICO_SIM fork:
// the software counter is always the source here.)
__attribute__((used, externally_visible))
uint32_t
runtime_cyccnt_read(void)
{
    return runtime_g0_sw_cyccnt;
}

__attribute__((used, externally_visible))
void
runtime_tick_enable(void)
{
    // No-op if already running; configure_axis calls this on every build.
    if (MOTION_TIM->CR1 & TIM_CR1_CEN) {
        return;
    }
    MOTION_TIM->CR1 &= ~TIM_CR1_CEN;
    MOTION_TIM->ARR  = (runtime_clock_freq / CONFIG_KALICO_MOTION_SAMPLE_RATE_HZ) - 1U;
    MOTION_TIM->EGR  = TIM_EGR_UG;
    MOTION_TIM->SR   = 0;
    MOTION_TIM->SR   = ~TIM_SR_UIF;     // clear stale UIF before enabling
    MOTION_TIM->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(MOTION_TIM_IRQn);
}

void
runtime_tick_init(void)
{
    // NVIC first (core-local, safe with the timer clock off), then enable the
    // peripheral clock before touching any timer register — touching a timer
    // before its clock is gated raises a bus fault on STM32.
    NVIC_DisableIRQ(MOTION_TIM_IRQn);

    MOTION_TIM_RCC_ENABLE();
    __DSB();

    MOTION_TIM->CR1 &= ~TIM_CR1_CEN;
    MOTION_TIM->SR = 0;

    // 64 MHz / sample_rate. At the 2 kHz G0 default ARR = 31999, well within the
    // 16-bit range of TIM6/TIM7; PSC stays 0 for any rate >= ~977 Hz.
    MOTION_TIM->PSC = 0;
    MOTION_TIM->ARR = (runtime_clock_freq / CONFIG_KALICO_MOTION_SAMPLE_RATE_HZ) - 1U;

    // Auto-reload preload + update interrupt enable.
    MOTION_TIM->CR1 = TIM_CR1_ARPE;
    MOTION_TIM->DIER = TIM_DIER_UIE;

    // No DWT to enable on Cortex-M0+.

    // Priority 2 — the same priority Klipper's scheduler dispatch runs at on G0
    // (TIM2, armcm_enable_irq(..., 2) in stm32f0_timer.c; SysTick on H7/F4).
    // Equal-priority Cortex-M interrupts never nest, so the motion ISR and the
    // scheduler-dispatched runtime_producer_event cannot preempt one another —
    // the mutual-exclusion guarantee that makes the shared &mut IsrState sound.
    // See the matching comment in runtime_tick_f4.c.
    NVIC_SetPriority(MOTION_TIM_IRQn, 2);

    MOTION_TIM->EGR  = TIM_EGR_UG;
    MOTION_TIM->SR   = ~TIM_SR_UIF;    // clear stale UIF before enabling
    MOTION_TIM->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(MOTION_TIM_IRQn);
}

// ── -311 stacked-PC capture: not available on Cortex-M0+ ────────────────────
// H7/F4 capture the interrupted context's PC/xPSR via a naked TIM5_IRQHandler
// wrapper that stashes the exception-frame base, so the -311 late-tick fault
// path can name the code that held the CPU (frame[6]=PC, frame[7]=xPSR). The G0
// backend deliberately omits that wrapper: a naked frame-capture shim plus the
// per-tick stash cost is exactly the profiling overhead this bring-up target
// drops (see the spec "Out of scope … hardware profiling/bench (no DWT)"), and
// ARMv6-M has no DWT to time it against anyway. These two accessors are still
// referenced from src/runtime_tick.c (runtime_handle_tick_blocker_pc path) and
// from Rust (rust/runtime/src/tick.rs isr_sample_tick), so they MUST resolve at
// link time. Returning 0 is the HONEST answer — fault_handler / the host treat
// 0 as "no stacked-PC data captured," not as a real PC of 0x0. This mirrors the
// Linux host backend (src/linux/runtime_tick_host.c), which stubs the same pair
// for the same reason (no TIM5 exception frame to read). Do NOT return a
// fabricated value here: a fake PC would send addr2line chasing a phantom
// blocker. used,externally_visible: referenced only from Rust; must survive
// -fwhole-program LTO + --gc-sections.
__attribute__((used, externally_visible))
uint32_t
runtime_tim5_stacked_pc(void)
{
    return 0;  // G0: no exception-frame capture (no naked TIM wrapper) → "no data"
}

__attribute__((used, externally_visible))
uint32_t
runtime_tim5_stacked_exc(void)
{
    return 0;  // G0: no exception-frame capture → "no data"
}

void
MOTION_TIM_IRQHandler(void)
{
    extern void diag_tim5_account(uint32_t enter, uint32_t exit);

    MOTION_TIM->SR = ~TIM_SR_UIF;       // entry-time ack

    // Advance the software cycle counter by one tick's worth of cycles. This is
    // the engine's only forward-time source on M0+ (no DWT); it must happen
    // every fire. Captured before the sample so diag/widen see a fresh value.
    runtime_g0_sw_cyccnt += RUNTIME_G0_CYC_PER_TICK;
    uint32_t diag_enter = runtime_g0_sw_cyccnt;

    // Sample armed endstop GPIOs before the engine tick (no-op when none armed).
    extern void runtime_endstop_sample_pins(void);
    runtime_endstop_sample_pins();

    uint32_t before = runtime_cyccnt_read();
    if (runtime_handle) {
        kalico_runtime_tick_sample(runtime_handle);
    }
    uint32_t after = runtime_cyccnt_read();

    // M0+ has no DWT cycle counter; the software counter does not advance within
    // a single ISR, so the per-tick cycle delta below is always 0 here.
    extern void diag_runtime_tick_account(uint32_t cycles);
    diag_runtime_tick_account(after - before);

    diag_tim5_account(diag_enter, runtime_g0_sw_cyccnt);
}

// Wire the handler into the generated vector table (scripts/buildcommands.py).
// NOTE: MOTION_TIM_IRQHandler expands to the SHARED vector name on G0
// (TIM7_LPTIM2_IRQHandler or TIM6_DAC_LPTIM1_IRQHandler), which is correct —
// that is the actual vector the chosen basic timer raises.
DECL_ARMCM_IRQ(MOTION_TIM_IRQHandler, MOTION_TIM_IRQn);

// ===========================================================================
// Dedicated step-output timer — NOT IMPLEMENTED on G0 (no compare channel)
// ===========================================================================
// On H7/F4 a second hardware timer (TIM3 / TIM2) drains the per-axis step
// queues: kalico_kick_step_output() (src/runtime_tick.c) arms a CC1 one-shot,
// and that timer's ISR calls kalico_step_output_event() to emit due steps. That
// design needs a timer with an output-compare channel.
//
// The G0 motion timer is deliberately a *basic* timer (TIM6/TIM7) — chosen in
// the spec precisely BECAUSE basic timers have no output channels, so hard_pwm
// can never steal them (docs/.../2026-05-31-stm32g0-sb2209-target-design.md).
// The flip side is that a basic timer has NO capture/compare channel, so it
// cannot implement the F4/H7 CC1 one-shot step-output pattern. The only
// compare-capable G0 timers (TIM2 = Klipper's scheduler clock; TIM3/TIM4 =
// PWM-contended) are unavailable or out of scope. Standing up a real G0
// step-output timer is therefore explicitly deferred — the milestone scope is
// "fits, links, and ticks the engine," with step emission a follow-on (same
// spec, "Out of scope for this milestone").
//
// These three symbols are referenced unconditionally from kalico_kick_step_-
// output() in src/runtime_tick.c (compiled for every arch), so they MUST
// resolve at link time. They are no-ops here, mirroring the Linux host backend
// (src/linux/runtime_tick_host.c) which stubs the same trio for the same shape
// of reason (no dedicated step-output timer; steps handled elsewhere or not yet
// at all). is_running() returns 0 so the kick treats every call as a first-arm
// and never tries to pull a (non-existent) compare forward — a clean no-op, not
// fabricated state. used,externally_visible: referenced from the Rust archive
// via kalico_kick_step_output; must survive -fwhole-program LTO + --gc-sections.
static uint32_t step_out_target_g0;

__attribute__((used, externally_visible))
void
step_output_timer_arm(uint32_t cycle_abs)
{
    step_out_target_g0 = cycle_abs;  // recorded for armed_target(); no HW timer
}

__attribute__((used, externally_visible))
uint32_t
step_output_timer_armed_target(void)
{
    return step_out_target_g0;
}

__attribute__((used, externally_visible))
uint8_t
step_output_timer_is_running(void)
{
    return 0;  // G0: no step-output timer → never "running" (kick is a no-op)
}

#endif
