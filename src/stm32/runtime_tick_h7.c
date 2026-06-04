// src/stm32/runtime_tick_h7.c
//
// H723-specific TIM5 init + IRQ handler. Spec §2.4 / §4.1 / §4.2 / §4.4.

#include "autoconf.h"
#include "generic/armcm_boot.h" // DECL_ARMCM_IRQ
#include "internal.h"          // STM32-internal helpers — TIM5, RCC, DWT
#include "kalico_runtime.h"
#include "generic/runtime_tick.h"   // interface contract
#include "generic/kalico_nvic_prio.h" // KALICO_MOTION_NVIC_PRIO

#if CONFIG_MACH_STM32H7

extern const uint32_t runtime_clock_freq;

extern void* runtime_handle;   // exposed in src/runtime_tick.c

// Dedicated step-output timer (TIM3, 16-bit) setup. Defined later in this file;
// forward-declared so runtime_tick_init can stand it up after arming TIM5.
static void step_output_timer_init(void);

// TIM5/TIM3 kernel clock = CONFIG_CLOCK_FREQ/2 when APB prescaler != 1
// (APB timer-doubler: TIMxCLK = 2 x PCLKx). Use this, not runtime_clock_freq.
static uint32_t
motion_timer_clk(void)
{
    uint32_t pclk = get_pclock_frequency((uint32_t)TIM5);
    uint32_t clkdiv = CONFIG_CLOCK_FREQ / pclk;
    if (clkdiv > 1)
        clkdiv /= 2;  // timer doubler when the APB prescaler != 1
    return CONFIG_CLOCK_FREQ / clkdiv;
}

// used, externally_visible: referenced from Rust only — survives LTO/--gc-sections.
__attribute__((used, externally_visible))
void
runtime_tick_disable(void)
{
    TIM5->CR1 &= ~TIM_CR1_CEN;
    NVIC_DisableIRQ(TIM5_IRQn);
}

// NEVER flash CONFIG_KALICO_SIM=y to silicon — IWDG-disable + software CYCCNT
// is a debug-only build. Sim: Renode returns 0 for DWT->CYCCNT; use SW counter.
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
    // Idempotent re-arm; on STM32 normally a no-op (CEN already set after init).
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
    // Disable IRQ before touching TIM5 registers (bus fault if peripheral
    // clock is off — clock-on must come first).
    NVIC_DisableIRQ(TIM5_IRQn);

    RCC->APB1LENR |= RCC_APB1LENR_TIM5EN;
    __DSB();  // clock must be up before register access

    TIM5->CR1 &= ~TIM_CR1_CEN;
    TIM5->SR = 0;

    // ARR from the true TIM5 kernel clock (motion_timer_clk = CONFIG_CLOCK_FREQ/2).
    TIM5->PSC = 0;
    TIM5->ARR = (motion_timer_clk() / CONFIG_KALICO_MOTION_SAMPLE_RATE_HZ) - 1U;

    TIM5->CR1 = TIM_CR1_ARPE;
    TIM5->DIER = TIM_DIER_UIE;

    // Enable DWT cycle counter.
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // KALICO_MOTION_NVIC_PRIO: TIM5 + step-output timer are EQUAL — the SPSC
    // same-priority invariant (see kalico_nvic_prio.h).
    NVIC_SetPriority(TIM5_IRQn, KALICO_MOTION_NVIC_PRIO);

    // TIM5 free-runs from boot; idles cheaply with no active piece.
    // Disabled only on Klipper shutdown; re-armed on FIRMWARE_RESTART.
    TIM5->EGR  = TIM_EGR_UG;
    TIM5->SR   = ~TIM_SR_UIF;     // clear stale UIF before enabling
    TIM5->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM5_IRQn);

    // Stand up the dedicated step-output timer (free-runs disabled until first step).
    step_output_timer_init();
}

// ── -311 stacked-PC capture ────────────────────────────────────────────────
// Naked wrapper saves the exception frame base into tim5_exc_frame on every
// tick. Rust reads frame[6] (PC) and frame[7] (xPSR) on the -311 path.
// frame[6/7] hold for both basic and extended FP frames (core regs always at
// the lowest addresses). NOT static: the naked asm is the sole writer; a
// file-local static would be GC-stripped (same trap as runtime_clock_freq).
volatile uint32_t *tim5_exc_frame __attribute__((used, externally_visible));

// Capture the exception frame pointer, then run the original handler body.
// Marked `used` so LTO/--gc-sections keep it (reached only via the naked
// wrapper's tail-branch, which is opaque to the optimizer).
__attribute__((used))
static void TIM5_IRQHandler_body(uint32_t *frame);

// Naked entry: select MSP vs PSP from EXC_RETURN bit 2 (the value still in LR
// on handler entry), stash it in `tim5_exc_frame`, and tail-call the body with
// the frame pointer in r0. A handful of instructions per tick — acceptable for
// a diagnostic build. (LR holds EXC_RETURN here; bit 2 == 0 ⇒ frame was on MSP,
// == 1 ⇒ on PSP. On this firmware the foreground also runs on MSP, so this is
// almost always MSP, but the test is correct for either.)
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
        "str    r0, [r1]            \n"  // tim5_exc_frame = frame base
        "b      TIM5_IRQHandler_body\n"  // tail-call; LR (EXC_RETURN) preserved
    );
}

// Stacked-PC getter — frame[6] is the return address (PC) of the interrupted
// context. Read only on the -311 path. used, externally_visible: referenced
// ONLY from the Rust archive, so --gc-sections / -fwhole-program LTO would
// otherwise drop it (same link trap as sched_last_dispatched_func).
__attribute__((used, externally_visible))
uint32_t
runtime_tim5_stacked_pc(void)
{
    if (!tim5_exc_frame)
        return 0;
    return tim5_exc_frame[6];
}

// frame[7] xPSR low 9 bits = active exception number (0 = thread/foreground).
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

    TIM5->SR = ~TIM_SR_UIF;            // entry-time ack (spec §2.4)

#if CONFIG_KALICO_SIM
    // Renode returns 0 for DWT->CYCCNT; drive a software counter so the
    // widened u64 advances at the expected rate.
    extern volatile uint32_t runtime_sim_cyccnt;
    runtime_sim_cyccnt += (runtime_clock_freq / 40000U);
#endif

    // Sample armed endstop GPIOs before the engine tick (no-op if none armed).
    extern void runtime_endstop_sample_pins(void);
    runtime_endstop_sample_pins();

    // Dispatch the unified per-sample evaluator: evaluates Bézier pieces,
    // runs Newton step-waketime iteration, pushes into per-axis SPSC step_queues.
    uint32_t before = runtime_cyccnt_read();
    if (runtime_handle) {
        kalico_runtime_tick_sample(runtime_handle);
    }
    uint32_t after = runtime_cyccnt_read();

    // Histogram engine-evaluator cost separately from full-IRQ overhead.
    extern void diag_runtime_tick_account(uint32_t cycles);
    diag_runtime_tick_account(after - before);

    diag_tim5_account(diag_enter, DWT->CYCCNT);
}

DECL_ARMCM_IRQ(TIM5_IRQHandler, TIM5_IRQn);

// ===========================================================================
// Dedicated step-output timer (TIM3)
// ===========================================================================
// TIM2 is taken by PWM, TIM5 by motion tick; TIM3 (16-bit) is the step-output
// timer. Free-running 16-bit up-counter, CC1 one-shot wake, chained for far
// targets via the ≤0xF000-clamp + IRQ re-arm.
// NVIC priority = KALICO_MOTION_NVIC_PRIO (same as TIM5) — no nesting, SPSC is safe.

// Max one-shot delta (16-bit, with wrap margin). Chained re-arm covers far targets.
#define STEP_OUT_MAX_DELTA 0xF000u

// cycle_abs is in DWT ticks (CONFIG_CLOCK_FREQ); TIM3 ticks at half that rate.
// step_out_clkdiv (= 2) scales DWT deltas to TIM3 ticks on each arm/re-arm.
// step_out_target: absolute DWT cycle of the next fire; step_out_running: CC1IE active.
static volatile uint32_t step_out_target;
static volatile uint8_t  step_out_running;
static uint32_t          step_out_clkdiv = 1;

// Scale dwt_delta to TIM3 ticks and program CC1 (clamped to 16-bit horizon).
static inline void
step_output_program_delta(uint32_t dwt_delta)
{
    uint32_t delta = dwt_delta / step_out_clkdiv;  // DWT cycles -> TIM3 ticks
    if (delta > STEP_OUT_MAX_DELTA)
        delta = STEP_OUT_MAX_DELTA;
    if (delta == 0)
        delta = 1;  // never arm in the past; fire next tick
    uint16_t ccr = (uint16_t)(TIM3->CNT + (uint16_t)delta);
    TIM3->CCR1 = ccr;
    TIM3->SR = (uint16_t)~TIM_SR_CC1IF;  // clear stale compare flag
    TIM3->DIER |= TIM_DIER_CC1IE;
}

// Arm TIM3 CC1 to fire at absolute DWT cycle cycle_abs, or disable when DISABLE sentinel.
// used,externally_visible: called from Rust via C shim; must survive --gc-sections LTO.
__attribute__((used, externally_visible))
void
step_output_timer_arm(uint32_t cycle_abs)
{
    if (cycle_abs == 0xFFFFFFFFu /* KALICO_STEP_OUTPUT_DISABLE */) {
        TIM3->DIER &= ~TIM_DIER_CC1IE;
        step_out_running = 0;
        return;
    }
    step_out_target = cycle_abs;
    step_out_running = 1;
    uint32_t now = runtime_cyccnt_read();
    uint32_t delta = cycle_abs - now;     // wrap-safe; >2^31 ⇒ already due
    if ((int32_t)delta <= 0)
        delta = 1;                         // due/late → fire next tick
    step_output_program_delta(delta);
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
    NVIC_DisableIRQ(TIM3_IRQn);

    // Enable TIM3 peripheral clock (APB1L on H7) before touching its registers.
    RCC->APB1LENR |= RCC_APB1LENR_TIM3EN;
    __DSB();

    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM3->SR = 0;
    TIM3->PSC = 0;
    TIM3->ARR = 0xFFFF;                 // free-running 16-bit up-counter
    TIM3->CCMR1 = 0;                    // CC1 = output compare, frozen output
    TIM3->CCR1 = 0;
    TIM3->DIER = 0;                     // CC1IE enabled only when armed
    TIM3->CR1 = TIM_CR1_ARPE;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->SR = 0;
    TIM3->CR1 |= TIM_CR1_CEN;           // counter free-runs; no IRQ until armed

    step_out_running = 0;
    step_out_target = 0;
    step_out_clkdiv = CONFIG_CLOCK_FREQ / motion_timer_clk(); // DWT/TIM3 rate ratio (= 2)

    NVIC_SetPriority(TIM3_IRQn, KALICO_MOTION_NVIC_PRIO);
    NVIC_EnableIRQ(TIM3_IRQn);
}

void
TIM3_IRQHandler(void)
{
    extern void diag_stepout_account(uint32_t enter, uint32_t exit);
    uint32_t diag_enter = DWT->CYCCNT;

    TIM3->SR = (uint16_t)~TIM_SR_CC1IF;   // ack the compare match

    // Emit due steps; returns soonest remaining target (or DISABLE for idle).
    extern uint32_t kalico_step_output_event(void);
    uint32_t next = kalico_step_output_event();
    step_output_timer_arm(next);

    diag_stepout_account(diag_enter, DWT->CYCCNT);
}

DECL_ARMCM_IRQ(TIM3_IRQHandler, TIM3_IRQn);

#endif
