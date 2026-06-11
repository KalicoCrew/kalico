// Timer based on ARM Cortex-M3/M4 SysTick and DWT logic
//
// Copyright (C) 2017-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_CLOCK_FREQ
#include "armcm_boot.h" // DECL_ARMCM_IRQ
#include "board/internal.h" // SysTick
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_from_us
#include "command.h" // shutdown
#include "sched.h" // sched_timer_dispatch
#include "generic/kalico_nvic_prio.h" // KALICO_SCHED_NVIC_PRIO

DECL_CONSTANT("CLOCK_FREQ", CONFIG_CLOCK_FREQ);

uint32_t
timer_from_us(uint32_t us)
{
    return us * (CONFIG_CLOCK_FREQ / 1000000);
}

// Return true if time1 is before time2.  Always use this function to
// compare times as regular C comparisons can fail if the counter
// rolls over.
// `used, externally_visible` keeps the out-of-line copy alive under
// -ffunction-sections + --gc-sections so the Rust kalico runtime archive
// can resolve it (the C-side callers all inline this short helper, which
// would otherwise leave no live section for the linker to pick up).
__attribute__((used, externally_visible))
uint8_t
timer_is_before(uint32_t time1, uint32_t time2)
{
    return (int32_t)(time1 - time2) < 0;
}

#if CONFIG_KALICO_SIM
// Remember the value last passed to `timer_set_diff` so SysTick_Handler can
// advance runtime_sim_cyccnt by exactly the cycles that elapsed since the
// last reload. `timer_set_diff` zeros LOAD after the one-shot reload, so the
// LOAD register itself can't be read back. Reset to 0 on shutdown via
// `timer_reset`'s SysTick disable path; reset elsewhere is unnecessary.
volatile uint32_t timer_last_diff;
#endif

static void
timer_set_diff(uint32_t value)
{
#if CONFIG_KALICO_SIM
    // Cap SysTick wait at ~1 ms of virtual time (520k cycles at 520 MHz).
    // Without this, a far-future Klipper timer (e.g. status_drain's 100 ms
    // gate) sets SysTick LOAD to 52 M+ cycles. Renode's sim runs at <1×
    // wall, so the next SysTick fire takes 100+ ms wall — and since the
    // software CYCCNT only advances when SysTick fires, MCU virtual time
    // creeps along at a small fraction of wall. Capping at 1 ms keeps
    // virtual-time advance proportional to wall, at the cost of more ISR
    // entries. The producer + status drain need cadence-driven progress;
    // they don't care that SysTick wakes more often than strictly needed.
    if (value > 520000U)
        value = 520000U;
    timer_last_diff = value;
#endif
    SysTick->LOAD = value;
    SysTick->VAL = 0;
    SysTick->LOAD = 0;
}

// Under CONFIG_KALICO_SIM (Renode), DWT->CYCCNT is unmodeled and reads as
// 0. Fork to the software CYCCNT (runtime_sim_cyccnt, bumped from the TIM5
// ISR by cycles-per-tick per fire — see src/stm32/runtime_tick_h7.c) so
// timer_dispatch_many() and the engine's widen state both observe forward
// progress. Production builds (CONFIG_KALICO_SIM=n) read the hardware register
// directly. NEVER flash
// a CONFIG_KALICO_SIM=y image to silicon — IWDG-disable + software CYCCNT
// is a debugging build only.
// `used, externally_visible` keeps the out-of-line copy alive under
// -ffunction-sections + --gc-sections so the Rust kalico runtime archive
// can resolve it (most C-side callers inline a one-liner equivalent or
// only reach this helper via the Rust archive).
__attribute__((used, externally_visible))
uint32_t
timer_read_time(void)
{
#if CONFIG_KALICO_SIM
    extern volatile uint32_t runtime_sim_cyccnt;
    return runtime_sim_cyccnt;
#else
    return DWT->CYCCNT;
#endif
}

void
timer_kick(void)
{
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
    SCB->ICSR = SCB_ICSR_PENDSTSET_Msk;
}

void
udelay(uint32_t usecs)
{
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }

    uint32_t end = timer_read_time() + timer_from_us(usecs);
    while (timer_is_before(timer_read_time(), end))
        ;
}

uint_fast8_t
timer_wrap_event(struct timer *t)
{
    t->waketime += 0xffffff;
    return SF_RESCHEDULE;
}

void
timer_reset(void)
{
    if (timer_from_us(100000) <= 0xffffff)
        // Timer in sched.c already ensures SysTick wont overflow
        return;
    sched_add_timer(sched_get_wrap_timer());
}
DECL_SHUTDOWN(timer_reset);

void
timer_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;

    timer_reset();

    irqstatus_t flag = irq_save();
    NVIC_SetPriority(SysTick_IRQn, KALICO_SCHED_NVIC_PRIO);
    SysTick->CTRL = (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk
                     | SysTick_CTRL_ENABLE_Msk);
    timer_kick();
    irq_restore(flag);
}
DECL_INIT(timer_init);

static uint32_t timer_repeat_until;
#define TIMER_REPEAT_TICKS timer_from_us(100)

#define TIMER_MIN_TRY_TICKS 90
#define TIMER_DEFER_REPEAT_TICKS timer_from_us(5)

// The dispatch loop opens the .sched_protected MPU window once at entry
// and closes it before each return. sched_timer_dispatch (called inside
// the loop) and periodic_event / sentinel_event / deleted_event / the
// wrap_timer callback all write to SchedState inside that single window,
// avoiding per-call MPU toggle jitter on this hot path.
static uint32_t
timer_dispatch_many(void)
{
    uint32_t tru = timer_repeat_until;
    sched_writable_begin();
    for (;;) {
        uint32_t next = sched_timer_dispatch();

        uint32_t now = timer_read_time();
        int32_t diff = next - now;
        if (diff > (int32_t)TIMER_MIN_TRY_TICKS) {
            sched_writable_end();
            return diff;
        }

        if (unlikely(timer_is_before(tru, now))) {
            if (diff < (int32_t)(-timer_from_us(1000))) {
                uint32_t hidx;
                uint32_t haddrs[SCHED_DISPATCH_HISTORY_N];
                uint32_t hfuncs[SCHED_DISPATCH_HISTORY_N];
                sched_get_dispatch_history(&hidx, haddrs, hfuncs);
                output("rsched_past idx %u a0 %u f0 %u a1 %u f1 %u"
                       " a2 %u f2 %u diff_us %i",
                       hidx,
                       haddrs[0], hfuncs[0],
                       haddrs[1], hfuncs[1],
                       haddrs[2], hfuncs[2],
                       (int32_t)(diff / (int32_t)timer_from_us(1)));
                // Split: MESSAGE_MAX (64 B) can't fit all 6 (addr, func) pairs.
                output("rsched_past_more a3 %u f3 %u a4 %u f4 %u"
                       " a5 %u f5 %u",
                       haddrs[3], hfuncs[3],
                       haddrs[4], hfuncs[4],
                       haddrs[5], hfuncs[5]);
                extern volatile uint32_t sched_bad_add_caller;
                extern volatile uint32_t sched_bad_add_value;
                extern volatile uint32_t sched_bad_add_stack0;
                extern volatile uint32_t sched_bad_add_stack1;
                extern volatile uint32_t sched_bad_add_stack2;
                extern volatile uint32_t sched_bad_add_blocked_count;
                output("rsched_bad_add caller %u value %u blocked %u"
                       " sp0 %u sp1 %u sp2 %u",
                       sched_bad_add_caller, sched_bad_add_value,
                       sched_bad_add_blocked_count,
                       sched_bad_add_stack0,
                       sched_bad_add_stack1,
                       sched_bad_add_stack2);
                uint32_t cwa[SCHED_CHAIN_WALK_N];
                uint32_t cwn[SCHED_CHAIN_WALK_N];
                uint32_t cws;
                sched_walk_chain(cwa, cwn, &cws);
                output("rsched_chain steps %u"
                       " a0 %u n0 %u a1 %u n1 %u a2 %u n2 %u",
                       cws, cwa[0], cwn[0], cwa[1], cwn[1],
                       cwa[2], cwn[2]);
                output("rsched_chain_more"
                       " a3 %u n3 %u a4 %u n4 %u a5 %u n5 %u",
                       cwa[3], cwn[3], cwa[4], cwn[4],
                       cwa[5], cwn[5]);
                // Close the writable window before try_shutdown longjmps
                // out of this scope, so the rest of the shutdown path
                // doesn't accidentally observe RW protected memory.
                sched_writable_end();
                try_shutdown("Rescheduled timer in the past");
            }
            if (sched_check_set_tasks_busy()) {
                timer_repeat_until = now + TIMER_REPEAT_TICKS;
                sched_writable_end();
                return TIMER_DEFER_REPEAT_TICKS;
            }
            timer_repeat_until = tru = now + TIMER_REPEAT_TICKS;
        }

        irq_enable();
        while (unlikely(diff > 0))
            diff = next - timer_read_time();
        irq_disable();
    }
}

void __visible __aligned(16) // aligning helps stabilize perf benchmarks
SysTick_Handler(void)
{
    irq_disable();
#if CONFIG_MACH_STM32
    extern void diag_systick_account(uint32_t enter, uint32_t exit);
    uint32_t diag_systick_enter = DWT->CYCCNT;
#endif
#if CONFIG_KALICO_SIM
    // CONFIG_KALICO_SIM uses a software CYCCNT (runtime_sim_cyccnt) because
    // Renode's H7 model returns 0 from DWT->CYCCNT. SysTick fires
    // unconditionally regardless of TIM5 state, so we piggyback time
    // advance here.
    //
    // Renode's H7 sim runs at ~0.7% of real-time wall — a 282 ms trajectory
    // would otherwise take ~40 s wall to retire even with perfect cyccnt
    // accounting, and 5–10× longer once producer / status / clock_sync
    // ISRs eat into that budget. We deliberately advance cyccnt by
    // `KALICO_SIM_CLOCK_MULT × timer_last_diff` to compress MCU virtual
    // time into a tractable wall budget. The host's clock-sync estimator
    // tracks whatever rate the MCU reports, so segment t_start values
    // computed by the planner stay in-phase with the sped-up clock.
    // KALICO_SIM_CLOCK_MULT=16 turns a 40 s real-time trajectory into
    // ~2.5 s wall when sim runs at 1× real, ~5 s wall at 0.5×, etc.
    extern volatile uint32_t runtime_sim_cyccnt;
    extern volatile uint32_t timer_last_diff;
    uint32_t advance = timer_last_diff;
    if (advance == 0)
        advance = 10000;
    runtime_sim_cyccnt += advance * 2U;
#endif
    uint32_t diff = timer_dispatch_many();
    timer_set_diff(diff);
#if CONFIG_MACH_STM32
    diag_systick_account(diag_systick_enter, DWT->CYCCNT);
#endif
    irq_enable();
}
DECL_ARMCM_IRQ(SysTick_Handler, SysTick_IRQn);

// Make sure timer_repeat_until doesn't wrap 32bit comparisons
void
timer_task(void)
{
    uint32_t now = timer_read_time();
    irq_disable();
    if (timer_is_before(timer_repeat_until, now))
        timer_repeat_until = now;
    irq_enable();
}
DECL_TASK(timer_task);
