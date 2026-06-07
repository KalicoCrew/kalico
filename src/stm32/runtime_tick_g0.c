// STM32G0B1 (Cortex-M0+ / ARMv6-M) motion-engine timer backend. Mirrors
// runtime_tick_f4.c, with two M0+ differences:
//
//   1. No TIM5: the motion ISR runs on a basic timer via the MOTION_TIM* alias
//      (runtime_tick_timer.h); RCC clock-enable is on RCC->APBENR1.
//   2. No DWT: runtime_cyccnt_read() returns a software counter this ISR
//      advances. The engine's widened clock is built on it, so it is FUNCTIONAL,
//      not profiling — without it engine time freezes and no segment advances.

#include "autoconf.h"
#include "generic/armcm_boot.h"
#include "internal.h"
#include "runtime_tick_timer.h"
#include "kalico_runtime.h"
#include "generic/runtime_tick.h"

#if CONFIG_MACH_STM32G0

extern const uint32_t runtime_clock_freq;

extern void* runtime_handle;

// Stands in for DWT->CYCCNT (absent on M0+). Single-writer (the ISR); volatile
// so runtime_cyccnt_read() observes ISR updates. Aligned-32-bit reads are
// atomic on ARMv6-M, so no lock.
static volatile uint32_t runtime_g0_sw_cyccnt;

#define RUNTIME_G0_CYC_PER_TICK \
    (CONFIG_CLOCK_FREQ / CONFIG_KALICO_MOTION_SAMPLE_RATE_HZ)

__attribute__((used, externally_visible))
void
runtime_tick_disable(void)
{
    MOTION_TIM->CR1 &= ~TIM_CR1_CEN;
    NVIC_DisableIRQ(MOTION_TIM_IRQn);
}

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
    // Touching a timer before its clock is gated bus-faults, so NVIC first
    // (core-local), then clock-on, then registers.
    NVIC_DisableIRQ(MOTION_TIM_IRQn);

    MOTION_TIM_RCC_ENABLE();
    __DSB();

    MOTION_TIM->CR1 &= ~TIM_CR1_CEN;
    MOTION_TIM->SR = 0;

    MOTION_TIM->PSC = 0;
    MOTION_TIM->ARR = (runtime_clock_freq / CONFIG_KALICO_MOTION_SAMPLE_RATE_HZ) - 1U;

    MOTION_TIM->CR1 = TIM_CR1_ARPE;
    MOTION_TIM->DIER = TIM_DIER_UIE;

    // Priority 2 = Klipper's scheduler-dispatch priority on G0. Equal-priority
    // M0+ interrupts never nest, so the motion ISR and the scheduler-dispatched
    // producer can't preempt each other — the &mut IsrState soundness guarantee.
    NVIC_SetPriority(MOTION_TIM_IRQn, 2);

    MOTION_TIM->EGR  = TIM_EGR_UG;
    MOTION_TIM->SR   = ~TIM_SR_UIF;    // clear stale UIF before enabling
    MOTION_TIM->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(MOTION_TIM_IRQn);
}

// No exception-frame capture on M0+ (no naked wrapper). Still referenced from
// runtime_tick.c and Rust, so they must link. Returning 0 is the honest "no
// data" answer (the host treats 0 as no-capture, not a real PC) — do NOT
// fabricate a value. used,externally_visible: Rust-only callers.
__attribute__((used, externally_visible))
uint32_t
runtime_tim5_stacked_pc(void)
{
    return 0;
}

__attribute__((used, externally_visible))
uint32_t
runtime_tim5_stacked_exc(void)
{
    return 0;
}

void
MOTION_TIM_IRQHandler(void)
{
    extern void diag_tim5_account(uint32_t enter, uint32_t exit);

    MOTION_TIM->SR = ~TIM_SR_UIF;       // entry-time ack

    // The engine's only forward-time source on M0+; must advance every fire.
    runtime_g0_sw_cyccnt += RUNTIME_G0_CYC_PER_TICK;
    uint32_t diag_enter = runtime_g0_sw_cyccnt;

    extern void runtime_endstop_sample_pins(void);
    runtime_endstop_sample_pins();

    uint32_t before = runtime_cyccnt_read();
    if (runtime_handle) {
        kalico_runtime_tick_sample(runtime_handle);
    }
    uint32_t after = runtime_cyccnt_read();

    // The software counter doesn't advance within one ISR, so this delta is 0.
    extern void diag_runtime_tick_account(uint32_t cycles);
    diag_runtime_tick_account(after - before);

    diag_tim5_account(diag_enter, runtime_g0_sw_cyccnt);
}

// MOTION_TIM_IRQHandler expands to the SHARED vector name on G0
// (TIM7_LPTIM2_IRQHandler etc.) — the actual vector the chosen basic timer raises.
DECL_ARMCM_IRQ(MOTION_TIM_IRQHandler, MOTION_TIM_IRQn);

// Step-output timer NOT implemented on G0: the motion timer is a basic timer
// (TIM6/TIM7, no output channels so hard_pwm can't steal it), which has no
// compare channel for the F4/H7 CC1 one-shot pattern; step emission is deferred.
// The trio is referenced unconditionally from kalico_kick_step_output() (every
// arch), so it must link — these are no-ops. is_running() returns 0 so the kick
// treats every call as a first-arm. used,externally_visible: Rust-only callers.
static uint32_t step_out_target_g0;

__attribute__((used, externally_visible))
void
step_output_timer_arm(uint32_t cycle_abs)
{
    step_out_target_g0 = cycle_abs;
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
    return 0;
}

#endif
