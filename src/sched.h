#ifndef __SCHED_H
#define __SCHED_H

#include <stdint.h> // uint32_t
#include "ctr.h" // DECL_CTR

// Declare an init function (called at firmware startup)
#define DECL_INIT(FUNC) _DECL_CALLLIST(ctr_run_initfuncs, FUNC)
// Declare a task function (called periodically during normal runtime)
#define DECL_TASK(FUNC) _DECL_CALLLIST(ctr_run_taskfuncs, FUNC)
// Declare a shutdown function (called on an emergency stop)
#define DECL_SHUTDOWN(FUNC) _DECL_CALLLIST(ctr_run_shutdownfuncs, FUNC)

// Timer structure for scheduling timed events (see sched_add_timer() )
struct timer {
    struct timer *next;
    uint_fast8_t (*func)(struct timer*);
    uint32_t waketime;
};

// Section attribute for the single MPU-protected scheduler state struct
// in sched.c. The block is read-only at runtime except for the brief
// windows opened by sched_writable_begin() / sched_writable_end() (the
// public sched.c writers use these internally; timer_dispatch_many opens
// the window once per SysTick to amortize across many dispatches).
// Any other write to this region faults into MemManage_Handler — the
// faulting PC pinpoints the rogue writer.
#define SCHED_PROTECTED __attribute__((section(".sched_protected")))

// Open / close the MPU writable window over `.sched_protected`. Defined
// in src/generic/mpu_protect.c. No-op on builds without MPU support.
void sched_writable_begin(void);
void sched_writable_end(void);

// Initialize the MPU protection on `.sched_protected`. Called once from
// armcm_main after clock_setup, before sched_main.
void mpu_protect_init(void);

enum { SF_DONE=0, SF_RESCHEDULE=1 };

// Task waking struct
struct task_wake {
    uint8_t wake;
};

// sched.c
void sched_add_timer(struct timer*);
void sched_del_timer(struct timer *del);
unsigned int sched_timer_dispatch(void);
void sched_timer_reset(void);
void sched_wake_tasks(void);
uint8_t sched_check_set_tasks_busy(void);
void sched_wake_task(struct task_wake *w);
uint8_t sched_check_wake(struct task_wake *w);
uint8_t sched_is_shutdown(void);
void sched_clear_shutdown(void);
void sched_try_shutdown(uint_fast8_t reason);
void sched_shutdown(uint_fast8_t reason) __noreturn;
void sched_report_shutdown(void);
void sched_main(void);

// Return the wrap_timer for CPUs whose hardware SysTick can't cover a
// full 100 ms period in one shot. armcm_timer.c::timer_reset uses this
// to schedule a recurring wrap event. The pointer addresses the
// SCHED_PROTECTED struct, so callers may only read its fields; writes
// must go through sched_writable_begin()/end() or the public sched.c
// API (e.g. sched_add_timer).
struct timer *sched_get_wrap_timer(void);

// Compiler glue for DECL_X macros above.
#define _DECL_CALLLIST(NAME, FUNC)                                      \
    DECL_CTR("_DECL_CALLLIST " __stringify(NAME) " " __stringify(FUNC))

#endif // sched.h
