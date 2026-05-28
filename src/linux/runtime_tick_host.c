// Host-process modulation tick driver. Spawns a pthread that calls
// kalico_runtime_modulated_tick at 40 kHz, mirroring TIM5_IRQHandler
// behavior on the H7 firmware (src/stm32/runtime_tick_h7.c). Used by
// MACH_LINUX builds for klippy-in-loop integration testing.
//
// Step-emission T10 (2026-05-14): switched from the legacy
// `runtime_handle_tick` (Engine::tick path) to
// `kalico_runtime_modulated_tick`, matching the H7/F4 ISR shims. The
// modulated path computes its widened clock inline via
// `runtime_widened_host_clock` (defined in src/runtime_tick.c) — the
// `runtime_handle_seed_widen` call below remains for the foreground
// clock-sync widening seed.

#include "generic/runtime_tick.h"

#include <dlfcn.h>
#include <errno.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

#include "autoconf.h" // CONFIG_CLOCK_FREQ
#include "kalico_runtime.h"
#include "sched.h" // shutdown handling — currently unused but matches H7
#include "step_queue.h" // StepQueue, step_queues[], N_AXIS_STEP_QUEUES

extern void *runtime_handle;
extern void runtime_endstop_sample_pins(void); // src/runtime_tick.c

// Watchdog liveness flag (defined on H7 in src/stm32/watchdog.c). The
// Linux build has no IWDG; default to ok=1 so the runtime drain doesn't
// short-circuit. Mutated by the runtime liveness gate.
volatile uint8_t runtime_liveness_ok = 1;

// Sim-only cycle counter (defined on H7 under CONFIG_KALICO_SIM in
// stm32/kalico_sim_clock.c). The Linux build maps it onto the host
// monotonic-derived counter that runtime_cyccnt_read returns.
volatile uint32_t runtime_sim_cyccnt = 0;

#define HOST_TICK_HZ 40000UL
#define HOST_TICK_NS (1000000000UL / HOST_TICK_HZ)

static atomic_int host_tick_enabled = 0;
static atomic_int host_tick_thread_started = 0;
static pthread_t host_tick_thread;
static struct timespec host_tick_t0;

static uint64_t
host_monotonic_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t s  = (uint64_t)(ts.tv_sec  - host_tick_t0.tv_sec);
    int64_t  ns = (int64_t)ts.tv_nsec - host_tick_t0.tv_nsec;
    return s * 1000000000ULL + (uint64_t)(ns + (ns < 0 ? 1000000000 : 0));
}

extern uint32_t timer_read_time(void); // src/linux/timer.c
extern uint64_t timer_read_time_u64(void); // src/linux/timer.c
// runtime_handle_seed_widen is declared in kalico_runtime.h (included above)

__attribute__((used)) uint32_t
runtime_cyccnt_read(void)
{
    // Use Klipper's own clock (timer_read_time) so the engine's `now` lives
    // in the same reference frame as the values klippy's clocksync sends to
    // the host bridge via set_clock_est. If we use a different t0 here, the
    // host schedules t_start in Klipper's frame while the engine reads `now`
    // from this thread.
    return timer_read_time();
}

// Wrapper exposed for runtime_tick_init's widen-seeding call.
__attribute__((used)) uint64_t
runtime_host_widened_clock_now(void)
{
    return timer_read_time_u64();
}

// Step pin → GPIO line mapping for the sim step queue consumer.
// Matches printer_real/config after pin-overrides.toml:
// X(motor0)=PG4→gpio18, Y(motor1)=PF11→gpio7, Z(motor2)=PG0→gpio15.
static const int step_gpio_lines[N_AXIS_STEP_QUEUES] = { 18, 7, 15, -1 };

// dlsym-resolved shim function for notifying auto-endstop of steps.
static void (*sim_notify_step)(int chip, int line, int32_t n_steps);

static void *
host_tick_main(void *arg)
{
    (void)arg;

    // Lower priority so the main thread's ppoll (which advances vtime)
    // preempts us when both are runnable.
    pid_t tid = (pid_t)syscall(SYS_gettid);
    setpriority(PRIO_PROCESS, tid, 19);

    // Resolve the shim's step-notification function.
    sim_notify_step = dlsym(RTLD_DEFAULT, "sim_intercept_notify_step");

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    while (1) {
        next.tv_nsec += HOST_TICK_NS;
        while (next.tv_nsec >= 1000000000L) {
            next.tv_nsec -= 1000000000L;
            next.tv_sec  += 1;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);

        if (!atomic_load_explicit(&host_tick_enabled, memory_order_acquire))
            continue;
        if (!runtime_handle)
            continue;

#if !CONFIG_KALICO_SIM
        runtime_endstop_sample_pins();
#endif

        (void)runtime_cyccnt_read();
        kalico_runtime_tick_sample(runtime_handle);

        // Drain step queues: pop entries pushed by tick_sample and
        // notify the auto-endstop shim so it can count step pulses.
        for (int axis = 0; axis < N_AXIS_STEP_QUEUES; axis++) {
            StepQueue *q = &step_queues[axis];
            while (q->head != q->tail) {
                uint16_t idx = q->head & (STEP_QUEUE_DEPTH - 1);
                int8_t dir = q->buf[idx].dir;
                q->head++;
                if (sim_notify_step && step_gpio_lines[axis] >= 0)
                    sim_notify_step(0, step_gpio_lines[axis],
                                    dir ? -1 : 1);
            }
        }
    }
    return NULL;
}

extern void *runtime_handle;

__attribute__((used)) void
runtime_tick_init(void)
{
    if (atomic_exchange(&host_tick_thread_started, 1))
        return;
    clock_gettime(CLOCK_MONOTONIC, &host_tick_t0);


    pthread_attr_t attr;
    pthread_attr_init(&attr);
    int rc = pthread_create(&host_tick_thread, &attr, host_tick_main, NULL);
    pthread_attr_destroy(&attr);
    if (rc != 0) {
        fprintf(stderr, "kalico_host_tick: pthread_create failed: %d\n", rc);
        // Mark as not started so a later runtime_tick_init can retry —
        // but this is a fatal init path on the H7 too; we just log here.
        atomic_store(&host_tick_thread_started, 0);
    }
}

extern uint32_t stats_send_time_high; // src/basecmd.c
extern uint32_t stats_send_time;      // src/basecmd.c (exposed 2026-05-11)

__attribute__((used)) void
runtime_tick_enable(void)
{
    // Seed widen state to match Klippy's view of widened MCU clock.
    //
    // Klippy reads `get_uptime` on connect and widens incrementally from
    // there. `command_get_uptime` returns
    //   `high = stats_send_time_high + (cur < stats_send_time)`
    // — the second term captures a wrap that occurred AFTER the last
    // `stats_update` bumped `stats_send_time_high` but BEFORE this read.
    // Up to one wrap can fall in that window (stats_update runs every ~5 s,
    // u32 wraps at 520 MHz every ~8.26 s on H7; on Linux the wrap period
    // differs but the off-by-one risk is the same), so we MUST replicate
    // this exact arithmetic or the engine's WidenState lags klippy's
    // `last_clock` by 2^32 and the first dispatched segment's t_start ends
    // up unreachable from the engine's `now` (saturating-subtract to zero
    // → curve evaluated at u=0 → zero step pulses).
    if (runtime_handle) {
        uint32_t low = timer_read_time();
        uint32_t high = stats_send_time_high + (low < stats_send_time);
        uint64_t baseline = ((uint64_t)high) << 32 | (uint64_t)low;
        runtime_handle_seed_widen(runtime_handle, baseline);
        // Wire the C-owned step_queues array into the Rust engine so that
        // tick_sample's dispatch_axis can push step entries on this host
        // build. On the MCU the engine resolves the C extern directly; here
        // test_queue_ptrs stays null unless we install them explicitly.
        kalico_runtime_install_step_queues(runtime_handle,
                                           (uint8_t *)step_queues);
    }
    // Enable the tick unconditionally so the runtime evaluates segments and
    // pushes to the step queues installed above. This matches the MCU, which
    // now arms TIM5 unconditionally at init (free-running from boot, no arm
    // gate); the Linux sim likewise has no separate step-event path for
    // regular-stepping motors, so the tick must always be on.
    atomic_store_explicit(&host_tick_enabled, 1, memory_order_release);
}

__attribute__((used)) void
runtime_tick_disable(void)
{
    atomic_store_explicit(&host_tick_enabled, 0, memory_order_release);
}
