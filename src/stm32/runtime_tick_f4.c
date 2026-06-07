// STM32F4-specific TIM5 init + IRQ handler. Mirrors runtime_tick_h7.c; the
// family-level difference is the RCC clock-enable register (F4's single APB1ENR
// vs H7's split APB1LENR).

#include "autoconf.h"
#include "generic/armcm_boot.h" // DECL_ARMCM_IRQ
#include "internal.h"          // STM32-internal helpers — TIM5, RCC, DWT
#include "kalico_runtime.h"
#include "generic/runtime_tick.h"   // interface contract
#include "generic/kalico_nvic_prio.h" // KALICO_MOTION_NVIC_PRIO

#if CONFIG_MACH_STM32F4

extern const uint32_t runtime_clock_freq;

extern void* runtime_handle;   // exposed in src/runtime_tick.c

static void step_output_timer_init(void);

// Kernel clock accounts for the APB timer-doubler; use this, not
// runtime_clock_freq, for ARR and delta scaling.
static uint32_t
motion_timer_clk(void)
{
    uint32_t pclk = get_pclock_frequency((uint32_t)TIM5);
    uint32_t clkdiv = CONFIG_CLOCK_FREQ / pclk;
    if (clkdiv > 1)
        clkdiv /= 2;
    return CONFIG_CLOCK_FREQ / clkdiv;
}

// used,externally_visible on the Rust-only entry points: survives
// -fwhole-program LTO.
__attribute__((used, externally_visible))
void
runtime_tick_disable(void)
{
    TIM5->CR1 &= ~TIM_CR1_CEN;
    NVIC_DisableIRQ(TIM5_IRQn);
}

// NEVER flash CONFIG_KALICO_SIM=y to silicon (IWDG-disabled debug build).
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
    // Idempotent; normally a no-op on STM32 (CEN already set).
    if (TIM5->CR1 & TIM_CR1_CEN) {
        return;
    }
    TIM5->CR1 &= ~TIM_CR1_CEN;
    TIM5->ARR  = (motion_timer_clk() / CONFIG_KALICO_MOTION_SAMPLE_RATE_HZ) - 1U;
    TIM5->EGR  = TIM_EGR_UG;
    TIM5->SR   = 0;
    TIM5->SR   = ~TIM_SR_UIF;
    TIM5->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM5_IRQn);
}

void
runtime_tick_init(void)
{
    NVIC_DisableIRQ(TIM5_IRQn);

    // Clock-on before any TIM5 register access.
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    __DSB();

    TIM5->CR1 &= ~TIM_CR1_CEN;
    TIM5->SR = 0;

    TIM5->PSC = 0;
    TIM5->ARR = (motion_timer_clk() / CONFIG_KALICO_MOTION_SAMPLE_RATE_HZ) - 1U;

    TIM5->CR1 = TIM_CR1_ARPE;
    TIM5->DIER = TIM_DIER_UIE;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // TIM5 and the step-output timer must be EQUAL priority (SPSC invariant;
    // see kalico_nvic_prio.h).
    NVIC_SetPriority(TIM5_IRQn, KALICO_MOTION_NVIC_PRIO);

    // Free-runs from boot.
    TIM5->EGR  = TIM_EGR_UG;
    TIM5->SR   = ~TIM_SR_UIF;
    TIM5->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM5_IRQn);

    step_output_timer_init();
}

// -311 stacked-PC capture (mirror of H7). NOT static: the naked asm is the sole
// writer, and a file-local static would be GC-stripped.
volatile uint32_t *tim5_exc_frame __attribute__((used, externally_visible));

__attribute__((used))
static void TIM5_IRQHandler_body(uint32_t *frame);

// Select MSP vs PSP from EXC_RETURN bit 2, stash the frame base, tail-call body.
__attribute__((naked))
void
TIM5_IRQHandler(void)
{
    __asm volatile (
        "tst    lr, #4              \n"  // EXC_RETURN bit 2: 0=MSP, 1=PSP
        "ite    eq                  \n"
        "mrseq  r0, msp             \n"
        "mrsne  r0, psp             \n"
        "ldr    r1, =tim5_exc_frame \n"
        "str    r0, [r1]            \n"
        "b      TIM5_IRQHandler_body\n"  // tail-call; LR (EXC_RETURN) preserved
    );
}

__attribute__((used, externally_visible))
uint32_t
runtime_tim5_stacked_pc(void)
{
    if (!tim5_exc_frame)
        return 0;
    return tim5_exc_frame[6];
}

__attribute__((used, externally_visible))
uint32_t
runtime_tim5_stacked_exc(void)
{
    if (!tim5_exc_frame)
        return 0;
    return tim5_exc_frame[7] & 0x1FFu;
}

static void
TIM5_IRQHandler_body(uint32_t *frame)
{
    (void)frame;
    extern void diag_tim5_account(uint32_t enter, uint32_t exit);
    uint32_t diag_enter = DWT->CYCCNT;

    TIM5->SR = ~TIM_SR_UIF;            // entry-time ack

#if CONFIG_KALICO_SIM
    extern volatile uint32_t runtime_sim_cyccnt;
    runtime_sim_cyccnt += (runtime_clock_freq / 40000U);
#endif

    uint32_t before = runtime_cyccnt_read();
    if (runtime_handle) {
        kalico_runtime_tick_sample(runtime_handle);
    }
    uint32_t after = runtime_cyccnt_read();

    extern void diag_runtime_tick_account(uint32_t cycles);
    diag_runtime_tick_account(after - before);

    diag_tim5_account(diag_enter, DWT->CYCCNT);
}

DECL_ARMCM_IRQ(TIM5_IRQHandler, TIM5_IRQn);

// Step-output timer: TIM2 (32-bit GP timer on F446; 32-bit reach needs no
// chaining). CC1 one-shot wake; NVIC priority = KALICO_MOTION_NVIC_PRIO (same as
// TIM5). step_out_clkdiv (= 2) scales DWT deltas to TIM2 ticks.
static volatile uint32_t step_out_target;
static volatile uint8_t  step_out_running;
static uint32_t          step_out_clkdiv = 1;

// used,externally_visible: Rust-only caller; must survive --gc-sections LTO.
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
    uint32_t now = runtime_cyccnt_read();
    uint32_t dwt_delta = cycle_abs - now;          // wrap-safe; >2^31 ⇒ already due
    if ((int32_t)dwt_delta <= 0)
        dwt_delta = step_out_clkdiv;
    uint32_t delta = dwt_delta / step_out_clkdiv;
    if (delta == 0)
        delta = 1;
    TIM2->CCR1 = TIM2->CNT + delta;
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

    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    __DSB();

    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM2->SR = 0;
    TIM2->PSC = 0;
    TIM2->ARR = 0xFFFFFFFFu;
    TIM2->CCMR1 = 0;
    TIM2->CCR1 = 0;
    TIM2->DIER = 0;                     // CC1IE enabled only when armed
    TIM2->CR1 = TIM_CR1_ARPE;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0;
    TIM2->CR1 |= TIM_CR1_CEN;

    step_out_running = 0;
    step_out_target = 0;
    step_out_clkdiv = CONFIG_CLOCK_FREQ / motion_timer_clk();

    NVIC_SetPriority(TIM2_IRQn, KALICO_MOTION_NVIC_PRIO);
    NVIC_EnableIRQ(TIM2_IRQn);
}

void
TIM2_IRQHandler(void)
{
    extern void diag_stepout_account(uint32_t enter, uint32_t exit);
    uint32_t diag_enter = DWT->CYCCNT;

    TIM2->SR = ~TIM_SR_CC1IF;           // ack the compare match

    extern uint32_t kalico_step_output_event(void);
    uint32_t next = kalico_step_output_event();
    step_output_timer_arm(next);

    diag_stepout_account(diag_enter, DWT->CYCCNT);
}

DECL_ARMCM_IRQ(TIM2_IRQHandler, TIM2_IRQn);

#endif
