// Definitions for irq enable/disable on ARM Cortex-M processors
//
// Copyright (C) 2017-2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/internal.h" // __CORTEX_M
#include "board/misc.h" // timer_read_time
#include "irq.h" // irqstatus_t
#include "sched.h" // DECL_SHUTDOWN

// SIPDIAG10: longest contiguous foreground PRIMASK-disabled window. The -308
// blocker masks the TIM5 evaluator ISR for ~200µs; if that mask is a foreground
// critical section (cpsid/irq_save) rather than a long ISR body, it shows up
// here. We track the OUTERMOST disable->enable window (start only recorded when
// PRIMASK was previously clear) and capture the return address of the caller
// that opened it, so the host can addr2line the offending critical section.
// Read from the Rust -308 path. REVERT this whole instrumentation after.
volatile uint32_t kalico_irqoff_max_cyc __attribute__((used));
volatile uint32_t kalico_irqoff_max_caller __attribute__((used));
static uint32_t irqoff_start_cyc;
static uint32_t irqoff_start_caller;

static inline void
irqoff_open(uint32_t caller)
{
    irqoff_start_cyc = timer_read_time();
    irqoff_start_caller = caller;
}

static inline void
irqoff_close(void)
{
    uint32_t dur = timer_read_time() - irqoff_start_cyc;
    if (dur > kalico_irqoff_max_cyc) {
        kalico_irqoff_max_cyc = dur;
        kalico_irqoff_max_caller = irqoff_start_caller;
    }
}

void
irq_disable(void)
{
    irqstatus_t prior;
    asm volatile("mrs %0, primask" : "=r" (prior) :: "memory");
    asm volatile("cpsid i" ::: "memory");
    // Record after masking so no ISR can corrupt the irqoff_* globals.
    if (!prior)
        irqoff_open((uint32_t)__builtin_return_address(0));
}

void
irq_enable(void)
{
    irqstatus_t prior;
    asm volatile("mrs %0, primask" : "=r" (prior) :: "memory");
    if (prior)
        irqoff_close();
    asm volatile("cpsie i" ::: "memory");
}

irqstatus_t
irq_save(void)
{
    irqstatus_t flag;
    asm volatile("mrs %0, primask" : "=r" (flag) :: "memory");
    asm volatile("cpsid i" ::: "memory");
    // Record after masking so no ISR can corrupt the irqoff_* globals.
    if (!flag)
        irqoff_open((uint32_t)__builtin_return_address(0));
    return flag;
}

void
irq_restore(irqstatus_t flag)
{
    irqstatus_t prior;
    asm volatile("mrs %0, primask" : "=r" (prior) :: "memory");
    if (prior && !flag)
        irqoff_close();
    asm volatile("msr primask, %0" :: "r" (flag) : "memory");
}

void
irq_wait(void)
{
    // SIPDIAG10: irq_wait re-enables interrupts (raw cpsie) each idle-loop
    // iteration, so TIM5 fires normally during idle. Bracket the raw window
    // with close/reopen so the idle-sleep loop in run_tasks is NOT mis-measured
    // as one multi-ms irq-off window (which masked the real blocker).
    irqoff_close();
    if (__CORTEX_M == 7)
        // Cortex-m7 may disable cpu counter on wfi, so use nop
        asm volatile("cpsie i\n    nop\n    cpsid i\n" ::: "memory");
    else
        asm volatile("cpsie i\n    wfi\n    cpsid i\n" ::: "memory");
    irqoff_open((uint32_t)__builtin_return_address(0));
}

void
irq_poll(void)
{
}

// Clear the active irq if a shutdown happened in an irq handler
void
clear_active_irq(void)
{
    uint32_t psr;
    asm volatile("mrs %0, psr" : "=r" (psr));
    if (!(psr & 0x1ff))
        // Shutdown did not occur in an irq - nothing to do.
        return;
    // Clear active irq status
    psr = 1<<24; // T-bit
    uint32_t temp;
    asm volatile(
        "  push { %1 }\n"
        "  adr %0, 1f\n"
        "  push { %0 }\n"
        "  push { r0, r1, r2, r3, r4, lr }\n"
        "  bx %2\n"
        ".balign 4\n"
        "1:\n"
        : "=&r"(temp) : "r"(psr), "r"(0xfffffff9) : "r12", "cc");
}
DECL_SHUTDOWN(clear_active_irq);
