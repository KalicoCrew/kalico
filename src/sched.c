// Basic scheduling functions and startup/shutdown code.
//
// Copyright (C) 2016-2024  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <setjmp.h> // setjmp
#include "autoconf.h" // CONFIG_*
#include "basecmd.h" // stats_update
#include "board/io.h" // readb
#include "board/irq.h" // irq_save
#include "board/misc.h" // timer_from_us
#include "board/pgm.h" // READP
#include "command.h" // shutdown
#include "sched.h" // sched_check_periodic
#include "stepper.h" // stepper_event

// Forward declarations of the timer-event handlers that own each foundation
// timer in SchedState. Defined below.
static uint_fast8_t periodic_event(struct timer *t);
static uint_fast8_t sentinel_event(struct timer *t);
static uint_fast8_t deleted_event(struct timer *t);

// Forward declaration of the wrap-event handler that drives wrap_timer. The
// implementation is CPU-specific (it pokes SysTick) and lives in
// src/generic/armcm_timer.c.
extern uint_fast8_t timer_wrap_event(struct timer *t);

// All scheduler foundation state lives in a single struct, placed in the
// MPU-protected `.sched_protected` linker section. New scheduler-foundation
// state added in future MUST go into this struct — there is no separate
// per-variable attribute that can be forgotten. mpu_protect_init() marks
// the section read-only at boot; sched_writable_begin() / sched_writable_end()
// toggle it RW for the brief window each sched.c writer needs. Any other
// code (Rust runtime, foreground tasks, IRQs) writing here will fault into
// MemManage_Handler — that captures the offending PC, so we identify
// the rogue writer immediately rather than chasing downstream corruption.
//
// Status flags (tasks_status / tasks_busy / shutdown_status / shutdown_reason)
// are deliberately kept OUTSIDE this struct in SchedFlags below. They get
// written from the hot run_tasks loop at very high frequency, and the
// observed corruption has only ever hit timer_list / last_insert / the
// foundation timers — never the flags. Keeping flags unprotected avoids
// thousands of unnecessary MPU toggles per second.
static struct sched_protected_state {
    struct timer periodic_timer;
    struct timer sentinel_timer;
    struct timer deleted_timer;
    struct timer wrap_timer;
    struct timer *timer_list;
    struct timer *last_insert;
} SchedState SCHED_PROTECTED = {
    .periodic_timer = { .func = periodic_event,
                        .next = &SchedState.sentinel_timer },
    .sentinel_timer = { .func = sentinel_event,
                        .waketime = 0x80000000 },
    .deleted_timer  = { .func = deleted_event },
    .wrap_timer     = { .func = timer_wrap_event,
                        .waketime = 0xffffff },
    .timer_list  = &SchedState.periodic_timer,
    .last_insert = &SchedState.periodic_timer,
};

// Status / shutdown flags — high-frequency writes, no MPU protection.
static struct {
    int8_t tasks_status, tasks_busy;
    uint8_t shutdown_status, shutdown_reason;
} SchedFlags;

// Expose the wrap_timer address to armcm_timer.c::timer_reset, which
// schedules it when SysTick can't cover a full 100 ms in one shot.
struct timer *
sched_get_wrap_timer(void)
{
    return &SchedState.wrap_timer;
}


/****************************************************************
 * Timers
 ****************************************************************/

// The periodic_timer simplifies the timer code by ensuring there is
// always a timer on the timer list and that there is always a timer
// not far in the future.
static uint_fast8_t
periodic_event(struct timer *t)
{
    // Make sure the stats task runs periodically
    sched_wake_tasks();
    // Reschedule timer. Writes to MPU-protected SchedState — caller chain
    // (SysTick_Handler → timer_dispatch_many) opens the protection window
    // around the dispatch loop, so no per-write toggle here.
    SchedState.periodic_timer.waketime += timer_from_us(100000);
    SchedState.sentinel_timer.waketime =
        SchedState.periodic_timer.waketime + 0x80000000;
    return SF_RESCHEDULE;
}

// The sentinel timer is always the last timer on timer_list - its
// presence allows the code to avoid checking for NULL while
// traversing timer_list.  Since sentinel_timer.waketime is always
// equal to (periodic_timer.waketime + 0x80000000) any added timer
// must always have a waketime less than one of these two timers.
static uint_fast8_t
sentinel_event(struct timer *t)
{
    shutdown("sentinel timer called");
}

// Find position for a timer in timer_list and insert it
static void __always_inline
insert_timer(struct timer *pos, struct timer *t, uint32_t waketime)
{
    struct timer *prev;
    for (;;) {
        prev = pos;
        if (CONFIG_MACH_AVR)
            // micro optimization for AVR - reduces register pressure
            asm("" : "+r"(prev));
        pos = pos->next;
        if (timer_is_before(waketime, pos->waketime))
            break;
    }
    t->next = pos;
    prev->next = t;
}

// Schedule a function call at a supplied time.
void
sched_add_timer(struct timer *add)
{
    uint32_t waketime = add->waketime;
    irqstatus_t flag = irq_save();
    sched_writable_begin();
    struct timer *tl = SchedState.timer_list;
    if (unlikely(timer_is_before(waketime, tl->waketime))) {
        // This timer is before all other scheduled timers
        if (timer_is_before(waketime, timer_read_time()))
            try_shutdown("Timer too close");
        if (tl == &SchedState.deleted_timer)
            add->next = SchedState.deleted_timer.next;
        else
            add->next = tl;
        SchedState.deleted_timer.waketime = waketime;
        SchedState.deleted_timer.next = add;
        SchedState.timer_list = &SchedState.deleted_timer;
        timer_kick();
    } else {
        insert_timer(tl, add, waketime);
    }
    sched_writable_end();
    irq_restore(flag);
}

// The deleted timer is used when deleting an active timer.
static uint_fast8_t
deleted_event(struct timer *t)
{
    return SF_DONE;
}

// Remove a timer that may be live.
void
sched_del_timer(struct timer *del)
{
    irqstatus_t flag = irq_save();
    sched_writable_begin();
    if (SchedState.timer_list == del) {
        // Deleting the next active timer - replace with deleted_timer
        SchedState.deleted_timer.waketime = del->waketime;
        SchedState.deleted_timer.next = del->next;
        SchedState.timer_list = &SchedState.deleted_timer;
    } else {
        // Find and remove from timer list (if present)
        struct timer *pos;
        for (pos = SchedState.timer_list; pos->next; pos = pos->next) {
            if (pos->next == del) {
                pos->next = del->next;
                break;
            }
        }
    }
    if (SchedState.last_insert == del)
        SchedState.last_insert = &SchedState.periodic_timer;
    sched_writable_end();
    irq_restore(flag);
}

// Diagnostic ring of the last few dispatched timers. Filled at the entry
// of sched_timer_dispatch, before t->func runs. Read via
// sched_get_dispatch_history. See sched.h for semantics.
static volatile uint32_t sched_dispatch_history_addr[SCHED_DISPATCH_HISTORY_N];
static volatile uint32_t sched_dispatch_history_func[SCHED_DISPATCH_HISTORY_N];
static volatile uint32_t sched_dispatch_history_idx;

void
sched_get_dispatch_history(uint32_t *idx,
                           uint32_t addrs[SCHED_DISPATCH_HISTORY_N],
                           uint32_t funcs[SCHED_DISPATCH_HISTORY_N])
{
    *idx = sched_dispatch_history_idx;
    for (int i = 0; i < SCHED_DISPATCH_HISTORY_N; i++) {
        addrs[i] = sched_dispatch_history_addr[i];
        funcs[i] = sched_dispatch_history_func[i];
    }
}

// Invoke the next timer - called from board hardware irq code.
// Caller (timer_dispatch_many in armcm_timer.c) is responsible for holding
// the MPU writable window open across the dispatch loop — we do not toggle
// per-call here to avoid jitter on the hot path.
unsigned int
sched_timer_dispatch(void)
{
    // Invoke timer callback
    struct timer *t = SchedState.timer_list;
    // Diagnostic: ring-buffer the dispatched timer's (addr, func) BEFORE
    // invoking its callback. On a "Rescheduled timer in past" trigger,
    // the most-recent entries identify which timer struct fed a bogus
    // `.next` value into SchedState.timer_list.
    uint32_t hidx = sched_dispatch_history_idx;
    sched_dispatch_history_addr[hidx % SCHED_DISPATCH_HISTORY_N] = (uint32_t)t;
    sched_dispatch_history_func[hidx % SCHED_DISPATCH_HISTORY_N] = (uint32_t)t->func;
    sched_dispatch_history_idx = hidx + 1;

    uint_fast8_t res;
    uint32_t updated_waketime;
    if (CONFIG_INLINE_STEPPER_HACK && likely(!t->func)) {
        res = stepper_event(t);
        updated_waketime = t->waketime;
    } else {
        res = t->func(t);
        updated_waketime = t->waketime;
    }

    // Update timer_list (rescheduling current timer if necessary)
    unsigned int next_waketime = updated_waketime;
    if (unlikely(res == SF_DONE)) {
        next_waketime = t->next->waketime;
        SchedState.timer_list = t->next;
        if (SchedState.last_insert == t)
            SchedState.last_insert = t->next;
    } else if (!timer_is_before(updated_waketime, t->next->waketime)) {
        next_waketime = t->next->waketime;
        SchedState.timer_list = t->next;
        struct timer *pos = SchedState.last_insert;
        if (timer_is_before(updated_waketime, pos->waketime))
            pos = SchedState.timer_list;
        insert_timer(pos, t, updated_waketime);
        SchedState.last_insert = t;
    }

    return next_waketime;
}

// Remove all user timers
void
sched_timer_reset(void)
{
    sched_writable_begin();
    SchedState.timer_list = &SchedState.deleted_timer;
    SchedState.deleted_timer.waketime = SchedState.periodic_timer.waketime;
    SchedState.deleted_timer.next = SchedState.last_insert
        = &SchedState.periodic_timer;
    SchedState.periodic_timer.next = &SchedState.sentinel_timer;
    sched_writable_end();
    timer_kick();
}


/****************************************************************
 * Tasks
 ****************************************************************/

#define TS_IDLE      -1
#define TS_REQUESTED 0
#define TS_RUNNING   1

// Note that at least one task is ready to run
void
sched_wake_tasks(void)
{
    SchedFlags.tasks_status = TS_REQUESTED;
}

// Check if tasks busy (called from low-level timer dispatch code)
uint8_t
sched_check_set_tasks_busy(void)
{
    // Return busy if tasks never idle between two consecutive calls
    if (SchedFlags.tasks_busy >= TS_REQUESTED)
        return 1;
    SchedFlags.tasks_busy = SchedFlags.tasks_status;
    return 0;
}

// Note that a task is ready to run
void
sched_wake_task(struct task_wake *w)
{
    sched_wake_tasks();
    writeb(&w->wake, 1);
}

// Check if a task is ready to run (as indicated by sched_wake_task)
uint8_t
sched_check_wake(struct task_wake *w)
{
    if (!readb(&w->wake))
        return 0;
    writeb(&w->wake, 0);
    return 1;
}

// Main task dispatch loop
static void
run_tasks(void)
{
    uint32_t start = timer_read_time();
    for (;;) {
        // Check if can sleep
        irq_poll();
        if (SchedFlags.tasks_status != TS_REQUESTED) {
            start -= timer_read_time();
            irq_disable();
            if (SchedFlags.tasks_status != TS_REQUESTED) {
                // Sleep processor (only run timers) until tasks woken
                SchedFlags.tasks_status = SchedFlags.tasks_busy = TS_IDLE;
                do {
                    irq_wait();
                } while (SchedFlags.tasks_status != TS_REQUESTED);
            }
            irq_enable();
            start += timer_read_time();
        }
        SchedFlags.tasks_status = TS_RUNNING;

        // Run all tasks
        extern void ctr_run_taskfuncs(void);
        ctr_run_taskfuncs();

        // Update statistics
        uint32_t cur = timer_read_time();
        stats_update(start, cur);
        start = cur;
    }
}


/****************************************************************
 * Shutdown processing
 ****************************************************************/

// Return true if the machine is in an emergency stop state
uint8_t
sched_is_shutdown(void)
{
    return !!SchedFlags.shutdown_status;
}

// Transition out of shutdown state
void
sched_clear_shutdown(void)
{
    if (!SchedFlags.shutdown_status)
        shutdown("Shutdown cleared when not shutdown");
    if (SchedFlags.shutdown_status == 2)
        // Ignore attempt to clear shutdown if still processing shutdown
        return;
    SchedFlags.shutdown_status = 0;
}

// Invoke all shutdown functions (as declared by DECL_SHUTDOWN)
static void
run_shutdown(int reason)
{
    irq_disable();
    uint32_t cur = timer_read_time();
    if (!SchedFlags.shutdown_status)
        SchedFlags.shutdown_reason = reason;
    SchedFlags.shutdown_status = 2;
    sched_timer_reset();
    extern void ctr_run_shutdownfuncs(void);
    ctr_run_shutdownfuncs();
    SchedFlags.shutdown_status = 1;
    irq_enable();

    sendf("shutdown clock=%u static_string_id=%hu", cur
          , SchedFlags.shutdown_reason);
}

// Report the last shutdown reason code
void
sched_report_shutdown(void)
{
    sendf("is_shutdown static_string_id=%hu", SchedFlags.shutdown_reason);
}

// Shutdown the machine if not already in the process of shutting down
void __always_inline
sched_try_shutdown(uint_fast8_t reason)
{
    if (!SchedFlags.shutdown_status)
        sched_shutdown(reason);
}

static jmp_buf shutdown_jmp;

// Force the machine to immediately run the shutdown handlers
void
sched_shutdown(uint_fast8_t reason)
{
    irq_disable();
    longjmp(shutdown_jmp, reason);
}


/****************************************************************
 * Startup
 ****************************************************************/

// Main loop of program
void
sched_main(void)
{
    extern void ctr_run_initfuncs(void);
    ctr_run_initfuncs();

    sendf("starting");

    irq_disable();
    int ret = setjmp(shutdown_jmp);
    if (ret) {
        // try_shutdown can longjmp from inside a sched_writable_begin()
        // window (e.g. sched_add_timer's "Timer too close" check), so the
        // matching end() is skipped and the depth counter is left
        // non-zero. Reset before running shutdown handlers so the
        // protection re-engages cleanly.
        sched_writable_reset();
        run_shutdown(ret);
    }
    irq_enable();

    run_tasks();
}
