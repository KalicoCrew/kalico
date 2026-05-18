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

DECL_CONSTANT("CLOCK_FREQ", CONFIG_CLOCK_FREQ);

// Return the number of clock ticks for a given number of microseconds
uint32_t
timer_from_us(uint32_t us)
{
    return us * (CONFIG_CLOCK_FREQ / 1000000);
}

// Return true if time1 is before time2.  Always use this function to
// compare times as regular C comparisons can fail if the counter
// rolls over.
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

// Set the next irq time
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

// Return the current time (in absolute clock ticks).
//
// Under CONFIG_KALICO_SIM (Renode), DWT->CYCCNT is unmodeled and reads as
// 0. Fork to the software CYCCNT (runtime_sim_cyccnt, bumped from the TIM5
// ISR by cycles-per-tick per fire — see src/stm32/runtime_tick_h7.c) so
// timer_dispatch_many() and the engine's widen state both observe forward
// progress. Per Step-6 plan Phase 0 Task 0.1 + Task 0.3. Production builds
// (CONFIG_KALICO_SIM=n) read the hardware register directly. NEVER flash
// a CONFIG_KALICO_SIM=y image to silicon — IWDG-disable + software CYCCNT
// is a debugging build only.
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

// Activate timer dispatch as soon as possible
void
timer_kick(void)
{
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
    SCB->ICSR = SCB_ICSR_PENDSTSET_Msk;
}

// Implement simple early-boot delay mechanism
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

// Dummy timer to avoid scheduling a SysTick irq greater than 0xffffff
static uint_fast8_t
timer_wrap_event(struct timer *t)
{
    t->waketime += 0xffffff;
    return SF_RESCHEDULE;
}
static struct timer wrap_timer = {
    .func = timer_wrap_event,
    .waketime = 0xffffff,
};
void
timer_reset(void)
{
    if (timer_from_us(100000) <= 0xffffff)
        // Timer in sched.c already ensures SysTick wont overflow
        return;
    sched_add_timer(&wrap_timer);
}
DECL_SHUTDOWN(timer_reset);

// Latched PC of the instruction that wrote to the watchpointed address.
// Captured by DebugMon_Handler on the first hit; the comparator is disabled
// immediately afterward so we keep only the first writer (later writes from
// sched.c after corruption would overwrite it otherwise).
volatile uint32_t schedstatus_writer_pc
    __attribute__((used, externally_visible));

// DebugMon exception handler. Naked: no prologue, so MSP still points at the
// exception-stacked {R0,R1,R2,R3,R12,LR,PC,xPSR}. We:
//   1. Read the just-written value at &SchedStatus.timer_list (0x20000038).
//   2. If it's NOT in either known scratch buffer (transmit_buf or batch_buf),
//      this is a legit sched.c write — return without touching the latch.
//   3. If it IS in scratch range and the latch is empty, snapshot the
//      stacked PC into schedstatus_writer_pc and disable the comparator.
//   4. If it IS in scratch range but the latch is already set, leave it.
__attribute__((naked, used))
void
DebugMon_Handler(void)
{
    __asm volatile(
        "ldr r0, =0x20000038\n"
        "ldr r0, [r0]\n"
        // Range check: transmit_buf [0x20000114, 0x20000514)
        "movw r1, #0x0114\n"
        "movt r1, #0x2000\n"
        "cmp r0, r1\n"
        "blt 2f\n"
        "movw r1, #0x0514\n"
        "movt r1, #0x2000\n"
        "cmp r0, r1\n"
        "blt 3f\n"
        "2:\n"
        // Range check: batch_buf [0x20000ba4, 0x200015a4)
        "movw r1, #0x0ba4\n"
        "movt r1, #0x2000\n"
        "cmp r0, r1\n"
        "blt 4f\n"
        "movw r1, #0x15a4\n"
        "movt r1, #0x2000\n"
        "cmp r0, r1\n"
        "bge 4f\n"
        "3:\n"
        // Bogus value. Latch if empty.
        "ldr r2, =schedstatus_writer_pc\n"
        "ldr r3, [r2]\n"
        "cbnz r3, 4f\n"
        // Determine PC offset: depends on whether the faulting context had
        // FP context stacked. EXC_RETURN bit 4 (FType): 1=basic frame only
        // (PC at MSP+24), 0=basic+FP frame (PC at MSP+24+72=96). EXC_RETURN
        // is in LR while in handler.
        "mrs r1, msp\n"
        "tst lr, #0x10\n"          // FType bit
        "ite ne\n"
        "addne r1, r1, #24\n"      // basic frame
        "addeq r1, r1, #96\n"      // FP-extended frame
        "ldr r1, [r1]\n"
        "str r1, [r2]\n"
        // Disable DWT comparator 0 to silence further hits.
        "ldr r2, =0xE0001028\n"     // DWT->FUNCTION0
        "mov r3, #0\n"
        "str r3, [r2]\n"
        "4:\n"
        "bx lr\n"
    );
}
DECL_ARMCM_IRQ(DebugMon_Handler, -4);

void
timer_init(void)
{
    // Enable Debug Watchpoint and Trace (DWT) for its 32bit timer
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;

    // Diagnostic watchpoint: trap any write to &SchedStatus.timer_list
    // (address 0x20000038 in the current build). On hit, DebugMon_Handler
    // latches the faulting PC into schedstatus_writer_pc and disables the
    // comparator. The address is verified at runtime via nm against the
    // ELF; if the layout changes the LR diag below will simply not fire.
    CoreDebug->DEMCR |= CoreDebug_DEMCR_MON_EN_Msk;
    DWT->COMP0     = 0x20000038u;
    DWT->MASK0     = 0u;        // exact-address match, no mask
    // FUNCTION = 0b0110 = data-address-match on write (ARMv7-M DWT v1+v2).
    DWT->FUNCTION0 = (6u << 0);

    // Verification: emit the configured DWT state so we can confirm via
    // klippy.log that the watchpoint is actually armed. If MON_EN didn't
    // stick (e.g. debug authentication denied it), FUNCTION0 reads back
    // as 0 and the diag is silent.
    output("dwt_armed demcr %u func0 %u comp0 %u",
           CoreDebug->DEMCR, DWT->FUNCTION0, DWT->COMP0);

    // Schedule a recurring timer on fast cpus
    timer_reset();

    // Enable SysTick
    irqstatus_t flag = irq_save();
    NVIC_SetPriority(SysTick_IRQn, 2);
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

// Invoke timers
static uint32_t
timer_dispatch_many(void)
{
    uint32_t tru = timer_repeat_until;
    for (;;) {
        // Run the next software timer
        uint32_t next = sched_timer_dispatch();

        uint32_t now = timer_read_time();
        int32_t diff = next - now;
        if (diff > (int32_t)TIMER_MIN_TRY_TICKS)
            // Schedule next timer normally.
            return diff;

        if (unlikely(timer_is_before(tru, now))) {
            // Check if there are too many repeat timers
            if (diff < (int32_t)(-timer_from_us(1000))) {
                // Diagnostic: capture which timer's reschedule landed in the
                // past. The head of the dispatch list is whoever owns the
                // stale waketime; decode `func` via `nm out/klipper.elf`.
                // Space-separated (no `name=%type`) so the host bridge takes
                // the free-form decode path and surfaces the formatted msg
                // through klippy's `#output:` handler — `name=%type` markers
                // route through the structured path, which drops msg for
                // unrecognized output names.
                //
                // Emit head pointer + head->next + head->next->func too so
                // we can disambiguate "head's func field was clobbered" from
                // "head pointer itself is stale" (e.g. list points into a
                // freed/never-allocated struct).
                // DWT watchpoint captures the PC that wrote to
                // &SchedStatus.timer_list. addr2line on writer_pc identifies
                // the corrupting code path.
                uint32_t pred_addr, pred_func, bad_next, steps;
                sched_walk_for_corruption(&pred_addr, &pred_func,
                                          &bad_next, &steps);
                struct timer *head = sched_get_head_timer();
                output("rsched_past head %u hfunc %u writer_pc %u"
                       " pred %u predf %u bad %u steps %u diff_us %i",
                       (uint32_t)head,
                       head ? (uint32_t)head->func : 0u,
                       schedstatus_writer_pc,
                       pred_addr, pred_func, bad_next, steps,
                       (int32_t)(diff / (int32_t)timer_from_us(1)));
                try_shutdown("Rescheduled timer in the past");
            }
            if (sched_check_set_tasks_busy()) {
                timer_repeat_until = now + TIMER_REPEAT_TICKS;
                return TIMER_DEFER_REPEAT_TICKS;
            }
            timer_repeat_until = tru = now + TIMER_REPEAT_TICKS;
        }

        // Next timer in the past or near future - wait for it to be ready
        irq_enable();
        while (unlikely(diff > 0))
            diff = next - timer_read_time();
        irq_disable();
    }
}

// IRQ handler
void __visible __aligned(16) // aligning helps stabilize perf benchmarks
SysTick_Handler(void)
{
    irq_disable();
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
