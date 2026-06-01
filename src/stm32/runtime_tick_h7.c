// src/stm32/runtime_tick_h7.c
//
// H723-specific TIM5 init + IRQ handler. Spec §2.4 / §4.1 / §4.2 / §4.4.

#include "autoconf.h"
#include "generic/armcm_boot.h" // DECL_ARMCM_IRQ
#include "internal.h"          // STM32-internal helpers — TIM5, RCC, DWT
#include "kalico_runtime.h"
#include "generic/runtime_tick.h"   // interface contract
#include "generic/runtime_bench.h" // runtime_bench_capture hook
#include "generic/kalico_nvic_prio.h" // KALICO_MOTION_NVIC_PRIO

#if CONFIG_MACH_STM32H7

extern const uint32_t runtime_clock_freq;

extern void* runtime_handle;   // exposed in src/runtime_tick.c

// Dedicated step-output timer (TIM3, 16-bit) setup. Defined later in this file;
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

    // T17: TIM5 rate from CONFIG_KALICO_MOTION_SAMPLE_RATE_HZ (see
    // comment in runtime_tick_enable for the rationale).
    // PSC = 0, ARR = (clock_freq / SAMPLE_RATE_HZ) - 1.
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
    // soundly — neither can preempt the other. Promoting from 3 to 2 trades
    // at most "TIM5 ISR worst-case duration" of scheduler-dispatch latency,
    // which is bounded by motion correctness (must fit in the 25 µs
    // modulation period at 40 kHz) and orders of magnitude below the 3 s
    // heater deadline / 500 ms IWDG window.
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

// ── -311 stacked-PC capture (diagnostic, 2026-06-01) ──────────────────────
//
// On a `-311 TickIntervalExceeded` we want to know WHAT code held the CPU /
// global interrupt mask (PRIMASK; `irq_disable` == `cpsid i`) across the late
// tick. The most direct evidence is the exception stack frame the hardware
// pushed when TIM5 was taken: its stacked PC is the instruction that was about
// to execute in the interrupted context, and its stacked xPSR carries the
// active-exception number of that context.
//
// We capture the active exception stack-frame base pointer at handler entry
// into `tim5_exc_frame`, every tick. The Rust `-311` path then reads
// frame[6] (stacked PC) and frame[7] (stacked xPSR) via the getters below.
//
// FP-frame correctness (M7 has an FPU): on exception entry the core ALWAYS
// pushes the 8-word basic frame {R0,R1,R2,R3,R12,LR,PC,xPSR} at the LOWEST
// addresses of the stacked frame. If lazy FP context is active (EXC_RETURN
// bit 4 == 0, "extended frame"), the FP registers {S0..S15,FPSCR} plus an
// alignment word are stacked ABOVE those 8 words (higher addresses). So
// frame[6] == PC and frame[7] == xPSR hold for BOTH the basic and the extended
// frame — the core-register offsets never move. We therefore do not need to
// inspect EXC_RETURN bit 4 to read PC/xPSR correctly.
static volatile uint32_t *tim5_exc_frame;

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

// Stacked-xPSR exception-number getter — frame[7] is the interrupted context's
// xPSR; its low 9 bits are the active-exception number (0 = thread/foreground).
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
    // period. No-op when no arm is active (table empty).
    extern void runtime_endstop_sample_pins(void);
    runtime_endstop_sample_pins();

    // T17 (stepping-redesign): TIM5 dispatches the unified per-sample
    // evaluator `kalico_runtime_tick_sample`, which evaluates the
    // active per-axis Bezier piece(s), runs Newton iteration for step
    // waketimes, and pushes step entries into the per-axis SPSC
    // step_queues. Replaces the prior modulator-polled-tick path
    // (`kalico_runtime_modulated_tick`); the legacy symbol stays
    // linkable for parts not yet cut over but isn't called from here.
    // The widened MCU clock is published by the producer Klipper timer
    // (`runtime_widened_host_clock` in src/runtime_tick.c) into
    // `SharedState::widened_now_lo`; the Rust ISR reads that value
    // directly. No CYCCNT widening seed is needed here.
    uint32_t before = runtime_cyccnt_read();
    if (runtime_handle) {
        kalico_runtime_tick_sample(runtime_handle);
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

// ===========================================================================
// Dedicated step-output timer (TIM3) — motion-tick priority-lift Step 1
// ===========================================================================
//
// Task 0 closed TIM3 for the H7: TIM2 is taken by caselight hardware PWM and
// TIM5 by the motion tick; those are the only 32-bit GP timers, so the
// step-output timer is 16-bit TIM3. The consumer is event-driven and only ever
// near-step (the producer kick arms it to the soonest pending step's cycle),
// so a 16-bit one-shot horizon is sufficient with simple chaining for the rare
// far re-arm — see step_output_timer_arm's ≤0xF000 clamp + the IRQ re-arm.
//
// Counter mode: free-running 16-bit up-counter at the timer clock (PSC = 0,
// ARR = 0xFFFF), output-compare channel 1 (CC1) as the wake source. We arm by
// writing CCR1 and enabling CC1IE; we disable by clearing CC1IE. The compare
// match sets SR.CC1IF and fires TIM3_IRQHandler.
//
// Reference for 16-bit chaining already in-tree: src/stm32/stm32f0_timer.c and
// src/stm32/runtime_tick_g0.c.
//
// NVIC priority: KALICO_MOTION_NVIC_PRIO — IDENTICAL to TIM5 (Step 1 parity;
// no flip). Same-number Cortex-M interrupts cannot nest, so this consumer and
// the TIM5 producer never preempt each other → the step_queues SPSC and the
// kalico_kick_step_output compare poke stay non-racing.

// Largest 16-bit one-shot delta we program in a single arm. Below 0x10000 with
// margin so the chained re-arm always lands strictly before a full wrap, and so
// the "already past" check below has slack. 0xF000 cycles ≈ 230 µs @ 275 MHz.
#define STEP_OUT_MAX_DELTA 0xF000u

// Bridge between the 32-bit absolute `cycle_abs` domain (DWT/TIM5 frame) and
// the 16-bit TIM3 counter. We cannot directly compare a 16-bit TIM3 CNT to a
// 32-bit cycle, so we track the absolute target here and, on each (re)arm or
// IRQ, compute the remaining delta against the 32-bit DWT clock (runtime_cyccnt
// _read), clamp it to STEP_OUT_MAX_DELTA, and set CCR1 = TIM3->CNT + delta.
//
// `step_out_target`   : absolute cycle the consumer must next fire at.
// `step_out_running`  : 1 while CC1IE is enabled (timer is arming toward a step).
static volatile uint32_t step_out_target;
static volatile uint8_t  step_out_running;

// Program TIM3 CC1 to fire `delta` (clamped) timer-ticks from the current CNT.
static inline void
step_output_program_delta(uint32_t delta)
{
    if (delta > STEP_OUT_MAX_DELTA)
        delta = STEP_OUT_MAX_DELTA;
    if (delta == 0)
        delta = 1;  // never arm in the past; fire next tick
    uint16_t ccr = (uint16_t)(TIM3->CNT + (uint16_t)delta);
    TIM3->CCR1 = ccr;
    TIM3->SR = (uint16_t)~TIM_SR_CC1IF;  // clear stale compare flag
    TIM3->DIER |= TIM_DIER_CC1IE;
}

// (Re)arm the step-output timer to fire at absolute cycle `cycle_abs`, or
// disable it when `cycle_abs == KALICO_STEP_OUTPUT_DISABLE`. Called from the
// Rust kick path (kalico_kick_step_output) and from the IRQ re-arm below.
//
// used, externally_visible: referenced from the Rust archive (via the C kick
// shim in runtime_tick.c) — keep it past --gc-sections / -fwhole-program LTO.
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

    NVIC_SetPriority(TIM3_IRQn, KALICO_MOTION_NVIC_PRIO);
    NVIC_EnableIRQ(TIM3_IRQn);
}

void
TIM3_IRQHandler(void)
{
    TIM3->SR = (uint16_t)~TIM_SR_CC1IF;   // ack the compare match

    // Run the Rust consumer: emit due steps, get the soonest remaining head
    // (or KALICO_STEP_OUTPUT_DISABLE). If the 16-bit horizon hasn't elapsed yet
    // (a chained far re-arm), the consumer returns the same far target and we
    // re-arm another chunk without emitting — handled inside step_output_timer_arm.
    extern uint32_t kalico_step_output_event(void);
    uint32_t next = kalico_step_output_event();
    step_output_timer_arm(next);
}

DECL_ARMCM_IRQ(TIM3_IRQHandler, TIM3_IRQn);

#endif
