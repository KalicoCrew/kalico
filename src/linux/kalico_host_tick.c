// Host-process modulation tick driver. Spawns a pthread that calls
// kalico_runtime_tick at 40 kHz, mirroring TIM5_IRQHandler behavior on
// the H7 firmware (src/stm32/kalico_h7_timer.c). Used by MACH_LINUX
// builds for klippy-in-loop integration testing.

#include "kalico_host_tick.h"

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

extern void *kalico_rt_handle;
extern void kalico_endstop_sample_pins(void); // src/runtime_tick.c

// Bench buffer storage — required by the symbol contract; never written
// on the Linux build (no DWT to sample).
volatile uint32_t kalico_bench_samples_buf[KALICO_BENCH_MAX_SAMPLES];
volatile uint16_t kalico_bench_count = 0;
volatile uint16_t kalico_bench_target = 0;
volatile uint8_t  kalico_bench_isolate = 0;

// Watchdog liveness flag (defined on H7 in src/stm32/watchdog.c). The
// Linux build has no IWDG; default to ok=1 so the runtime drain doesn't
// short-circuit. Mutated by the runtime liveness gate.
volatile uint8_t kalico_liveness_ok = 1;

// Sim-only cycle counter (defined on H7 under CONFIG_KALICO_SIM in
// stm32/kalico_sim_clock.c). The Linux build maps it onto the host
// monotonic-derived counter that kalico_h7_read_cyccnt returns.
volatile uint32_t kalico_sim_cyccnt = 0;

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
extern void kalico_runtime_seed_widen(void *rt, uint64_t baseline);

__attribute__((used)) uint32_t
kalico_h7_read_cyccnt(void)
{
    // Use Klipper's own clock (timer_read_time) so the engine's `now` lives
    // in the same reference frame as the values klippy's clocksync sends to
    // the host bridge via set_clock_est. If we use a different t0 here, the
    // host schedules t_start in Klipper's frame while the engine reads `now`
    // from this thread.
    return timer_read_time();
}

// Wrapper exposed for kalico_h7_timer_init's widen-seeding call.
__attribute__((used)) uint64_t
kalico_host_widened_clock_now(void)
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
        if (!kalico_rt_handle)
            continue;

        // Sample any armed endstop GPIOs before the engine tick — same
        // ordering as TIM5_IRQHandler so endstop::tick observes fresh
        // levels in the same modulation period. Skipped under
        // CONFIG_KALICO_SIM (the Linux build is itself a sim) so the
        // FFI-driven kalico_sim_endstop_set_pin path isn't clobbered
        // every tick by an unhelpful gpio_in_read of an unconnected pin.
#if !CONFIG_KALICO_SIM
        kalico_endstop_sample_pins();
#endif

        uint32_t cyc = kalico_h7_read_cyccnt();
        kalico_runtime_tick(kalico_rt_handle, cyc);
    }
    return NULL;
}

extern void *kalico_rt_handle;

__attribute__((used)) void
kalico_h7_timer_init(void)
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
        // Mark as not started so a later kalico_h7_timer_init can retry —
        // but this is a fatal init path on the H7 too; we just log here.
        atomic_store(&host_tick_thread_started, 0);
    }
}

__attribute__((used)) void
kalico_h7_enable_tim5(void)
{
    // Seed widen state with the current widened clock RIGHT BEFORE the
    // engine starts ticking. We can't do this at init time because the
    // u32 timer hasn't wrapped yet then; by the time enable_tim5 fires
    // (on first segment push, possibly minutes after process start),
    // wraps may have already occurred and the engine needs to know.
    if (kalico_rt_handle) {
        kalico_runtime_seed_widen(kalico_rt_handle, timer_read_time_u64());
    }
    atomic_store_explicit(&host_tick_enabled, 1, memory_order_release);
}

__attribute__((used)) void
kalico_h7_disable_tim5(void)
{
    atomic_store_explicit(&host_tick_enabled, 0, memory_order_release);
}
