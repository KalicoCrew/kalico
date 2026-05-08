// Host-process modulation tick driver. Spawns a pthread that calls
// runtime_handle_tick at 40 kHz, mirroring TIM5_IRQHandler behavior on
// the H7 firmware (src/stm32/runtime_tick_h7.c). Used by MACH_LINUX
// builds for klippy-in-loop integration testing.

#include "generic/runtime_tick.h"

#include <errno.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "autoconf.h" // CONFIG_CLOCK_FREQ
#include "kalico_runtime.h"
#include "sched.h" // shutdown handling — currently unused but matches H7

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

static void *
host_tick_main(void *arg)
{
    (void)arg;
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    while (1) {
        // Sleep until the next 40 kHz boundary regardless of run/pause —
        // disabling TIM5 on the H7 just gates the ISR; here we keep the
        // loop alive and skip the runtime call when paused.
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

        // Sample any armed endstop GPIOs before the engine tick — same
        // ordering as TIM5_IRQHandler so endstop::tick observes fresh
        // levels in the same modulation period. Skipped under
        // CONFIG_KALICO_SIM (the Linux build is itself a sim) so the
        // FFI-driven kalico_sim_endstop_set_pin path isn't clobbered
        // every tick by an unhelpful gpio_in_read of an unconnected pin.
#if !CONFIG_KALICO_SIM
        runtime_endstop_sample_pins();
#endif

        uint32_t cyc = runtime_cyccnt_read();
        runtime_handle_tick(runtime_handle, cyc);
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

__attribute__((used)) void
runtime_tick_enable(void)
{
    // Seed widen state to match Klippy's view of widened MCU clock.
    // Klippy widens via stats_send_time_high (incremented by stats_update
    // when it observes a u32 wrap). On Linux sim, timer_read_time can
    // return wrapped values during the first second of process life
    // (because start_sec = tv_sec + 1), which causes stats_update to
    // bump stats_send_time_high once spuriously. The engine sees the
    // same physical u32 timer but widens independently and never sees
    // that bump — putting it ~86s behind Klippy's view.
    //
    // Seed from stats_send_time_high directly so the two widening paths
    // agree: engine.now == Klippy's last_clock view.
    if (runtime_handle) {
        uint64_t baseline = ((uint64_t)stats_send_time_high) << 32
                          | (uint64_t)timer_read_time();
        runtime_handle_seed_widen(runtime_handle, baseline);
    }
    atomic_store_explicit(&host_tick_enabled, 1, memory_order_release);
}

__attribute__((used)) void
runtime_tick_disable(void)
{
    atomic_store_explicit(&host_tick_enabled, 0, memory_order_release);
}
