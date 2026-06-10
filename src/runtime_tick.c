#include <string.h>
#if defined(__linux__) || defined(__APPLE__)
#include <stdio.h>
#include <time.h>
#endif
#include "autoconf.h"
#include "board/gpio.h"
#include "board/internal.h"
#include "board/irq.h"
#include "board/misc.h"
#include "command.h"
#include "sched.h"
#include "kalico_runtime.h"
#include "kalico_dispatch.h"
#include "kalico_log.h"
#include "generic/runtime_tick.h"
#include "generic/fault_handler.h"

// Read from Rust via `extern "C" { static runtime_clock_freq: u32; }`;
// used,externally_visible keeps it through -fwhole-program LTO.
const uint32_t runtime_clock_freq __attribute__((used, externally_visible))
    = CONFIG_CLOCK_FREQ;

const uint32_t runtime_sample_rate_hz __attribute__((used, externally_visible))
    = CONFIG_KALICO_MOTION_SAMPLE_RATE_HZ;


extern volatile uint8_t runtime_liveness_ok;  // defined in src/stm32/watchdog.c

#define ENGINE_STATUS_RUNNING 1
#define ENGINE_STATUS_FAULT   3

// Foreground-only; NEVER call from ISR.
__attribute__((used, externally_visible))
uint64_t
runtime_host_now_us(void)
{
    uint32_t cycles = timer_read_time();
    return ((uint64_t)cycles) / (CONFIG_CLOCK_FREQ / 1000000U);
}

// (tag, stage, value) packed: bits[31:24]=tag, [23:16]=stage, [15:0]=value.
volatile uint32_t runtime_diag_last_packed __attribute__((used, externally_visible));

// Survives NVIC_SystemReset via .persistent_diag (NOLOAD, outside bss); next
// boot checks magic == RT_DIAG_MAGIC.
#define RT_DIAG_MAGIC 0xD1A6BABE

struct rt_diag_persistent {
    uint32_t magic;
    uint32_t last_packed;
    uint32_t last_us;
    uint32_t fault_count;
};

volatile struct rt_diag_persistent rt_diag_persistent
    __attribute__((section(".persistent_diag"), used, externally_visible));

volatile uint32_t runtime_diag_prior_boot_snapshot
    __attribute__((used, externally_visible));

volatile uint32_t runtime_diag_prior_magic_raw
    __attribute__((used, externally_visible));
volatile uint32_t runtime_diag_prior_packed_raw
    __attribute__((used, externally_visible));

__attribute__((used, externally_visible))
void
runtime_diag_progress(uint32_t tag, uint32_t stage, uint32_t value)
{
    uint32_t packed = ((tag & 0xFFu) << 24)
                    | ((stage & 0xFFu) << 16)
                    | (value & 0xFFFFu);
    runtime_diag_last_packed = packed;
    rt_diag_persistent.magic = RT_DIAG_MAGIC;
    rt_diag_persistent.last_packed = packed;
    rt_diag_persistent.last_us = timer_read_time();
}

// Advances regardless of engine state, unlike the ISR-published widened_now.
// Foreground-only; do NOT call from ISR.
__attribute__((used, externally_visible))
uint64_t
runtime_widened_host_clock(void)
{
    extern uint32_t stats_send_time;
    extern uint32_t stats_send_time_high;
    uint32_t cur = timer_read_time();
    uint32_t high = stats_send_time_high + (cur < stats_send_time);
    return ((uint64_t)high << 32) | (uint64_t)cur;
}

// used,externally_visible: the Rust staticlib calls these; LTO would otherwise
// DCE the standalone symbols.
__attribute__((used, externally_visible))
uint32_t
runtime_irq_save(void)
{
    return (uint32_t)irq_save();
}

__attribute__((used, externally_visible))
void
runtime_irq_restore(uint32_t flags)
{
    irq_restore((irqstatus_t)flags);
}

void* runtime_handle = 0;            // exposed (non-static) for runtime_tick_h7.c
static struct task_wake runtime_drain_wake;
static struct timer runtime_drain_timer;

static uint32_t last_status_emit_time = 0;
static uint8_t prev_engine_status = 0;

static uint32_t last_seen_tick_counter = 0;
static uint32_t last_progress_time = 0;

static uint8_t last_seen_status = 255;

// Reschedule from now (not +=1ms) to avoid a "timer in past" shutdown if the
// foreground stalls for >1 ms.
static uint_fast8_t
runtime_drain_event(struct timer *t)
{
    sched_wake_task(&runtime_drain_wake);
    t->waketime = timer_read_time() + timer_from_us(1000);
    return SF_RESCHEDULE;
}

void
runtime_init(void)
{
    extern volatile uint32_t runtime_diag_prior_magic_raw;
    extern volatile uint32_t runtime_diag_prior_packed_raw;
    runtime_diag_prior_magic_raw = rt_diag_persistent.magic;
    runtime_diag_prior_packed_raw = rt_diag_persistent.last_packed;
    if (rt_diag_persistent.magic == RT_DIAG_MAGIC
        && rt_diag_persistent.last_packed != 0) {
        runtime_diag_prior_boot_snapshot = rt_diag_persistent.last_packed;
    }

    runtime_diag_progress(0xB0, 0, 0);

#define RUNTIME_INIT_STUB 0  /* DIAG: 1 stubs runtime_init for crash bisect */
#if RUNTIME_INIT_STUB
    runtime_diag_progress(0xBF, 0, 0xCAFE);
    return;
#endif

    runtime_diag_progress(0xB1, 0, 0);
    runtime_handle = runtime_handle_create();
    if (!runtime_handle) {
        runtime_diag_progress(0xB1, 1, 0xFFFF);
        return;
    }
    runtime_diag_progress(0xB2, 0, 0);
    last_seen_tick_counter = runtime_handle_tick_counter(runtime_handle);
    last_progress_time = timer_read_time();
    last_seen_status = runtime_handle_status(runtime_handle);
    runtime_diag_progress(0xB3, 0, 0);

    runtime_diag_progress(0xB4, 0, 0);
    runtime_tick_init();
    runtime_diag_progress(0xB5, 0, 0);

    runtime_drain_timer.func = runtime_drain_event;
    runtime_drain_timer.waketime = timer_read_time() + timer_from_us(1000);
    sched_add_timer(&runtime_drain_timer);

    last_status_emit_time = timer_read_time();
}
DECL_INIT(runtime_init);

#define KALICO_LIVENESS_THRESHOLD_MS 25
#define KALICO_LIVENESS_THRESHOLD_TICKS  \
    ((KALICO_LIVENESS_THRESHOLD_MS) * (CONFIG_CLOCK_FREQ / 1000))

#define KALICO_FAST_STATUS_MAX_AXES 8
#define KALICO_FAST_STATUS_RING_OCCUPANCY 4

void
runtime_drain(void)
{
    if (!runtime_handle) return;
    if (!sched_check_wake(&runtime_drain_wake)) return;

    diag_task_heartbeat(diag_slot_rt_drain_calls(),
                        diag_slot_rt_drain_last_tick(),
                        diag_slot_rt_drain_max_gap(),
                        timer_from_us(50000),
                        0); // no event tag — idle gaps are normal

    // Liveness acts only on RUNNING; other states refresh the anchor so a
    // transition INTO RUNNING doesn't trip on a stale anchor.
    uint32_t cur_counter = runtime_handle_tick_counter(runtime_handle);
    uint32_t cur_time = timer_read_time();
    uint8_t cur_status = runtime_handle_status(runtime_handle);
    if (cur_status == ENGINE_STATUS_RUNNING) {
        if (cur_counter != last_seen_tick_counter) {
            last_seen_tick_counter = cur_counter;
            last_progress_time = cur_time;
        } else if ((cur_time - last_progress_time) > KALICO_LIVENESS_THRESHOLD_TICKS) {
            runtime_liveness_ok = 0;
        }
    } else {
        last_progress_time = cur_time;
        last_seen_tick_counter = cur_counter;
    }

    if (cur_status == ENGINE_STATUS_FAULT) {
        runtime_liveness_ok = 0;
        if (prev_engine_status != ENGINE_STATUS_FAULT) {
            int32_t fault_code = runtime_handle_last_error(runtime_handle);
            uint32_t fault_detail = runtime_handle_fault_detail(runtime_handle);
            uint32_t tick_blocker_pc = runtime_handle_tick_blocker_pc(runtime_handle);
            kalico_native_emit_fault_event((uint16_t)fault_code, fault_detail,
                                           tick_blocker_pc);
        }
    }

    // Fresh nonzero last_error → emit + Klipper shutdown. shutdown() is safe in
    // foreground (DECL_TASK) but NOT from ISR. last_acted_error suppresses
    // re-emit on the post-longjmp trailing pass.
    static int32_t last_acted_error;
    int32_t cur_error = runtime_handle_last_error(runtime_handle);
    if (cur_error != 0 && cur_error != last_acted_error) {
        last_acted_error = cur_error;
        uint32_t fdetail = runtime_handle_fault_detail(runtime_handle);
        uint32_t tick_blocker_pc = runtime_handle_tick_blocker_pc(runtime_handle);
        kalico_native_emit_fault_event((uint16_t)cur_error, fdetail,
                                       tick_blocker_pc);
        // Persist before shutdown resets the USB stack.
        diag_ring_push(DIAG_EV_RUST_FAULT, (uint32_t)cur_error, fdetail);
        runtime_liveness_ok = 0;
        shutdown("kalico runtime fault");
    }

    if (cur_status != prev_engine_status) {
        diag_record_engine_xition(prev_engine_status, cur_status, cur_counter);
    }
    prev_engine_status = cur_status;
    if (cur_status != last_seen_status) {
        last_seen_status = cur_status;
    }

    {
        static uint32_t last_retired_seen[KALICO_FAST_STATUS_MAX_AXES];
        uint32_t occ[KALICO_FAST_STATUS_MAX_AXES];
        uint32_t retired[KALICO_FAST_STATUS_MAX_AXES];
        uint8_t st = 0;
        uint16_t fc = 0;
        int32_t no = kalico_runtime_get_occupancy(runtime_handle, occ,
                                                  KALICO_FAST_STATUS_MAX_AXES);
        int32_t nr = kalico_runtime_get_heartbeat(runtime_handle, &st, &fc,
                                                  retired,
                                                  KALICO_FAST_STATUS_MAX_AXES);
        if (no > 0 && nr > 0) {
            int32_t n = no < nr ? no : nr;
            uint8_t emit = 0;
            for (int32_t i = 0; i < n; i++) {
                if (retired[i] != last_retired_seen[i]) {
                    if (occ[i] <= KALICO_FAST_STATUS_RING_OCCUPANCY)
                        emit = 1;
                    last_retired_seen[i] = retired[i];
                }
            }
            if (emit) {
                send_status_heartbeat();
                last_status_emit_time = timer_read_time();
            }
        }
    }

    kalico_log_drain();
}
DECL_TASK(runtime_drain);

void
runtime_tick_shutdown(void)
{
    runtime_tick_disable();
}
DECL_SHUTDOWN(runtime_tick_shutdown);

void
runtime_status_drain(void)
{
    if (!runtime_handle) return;
    uint32_t now = timer_read_time();
    const uint32_t status_period_ticks = CONFIG_CLOCK_FREQ / 10;
    if ((int32_t)(now - last_status_emit_time) < (int32_t)status_period_ticks)
        return;
    last_status_emit_time = now;
    send_status_heartbeat();

    diag_task_heartbeat(diag_slot_rt_status_calls(),
                        diag_slot_rt_status_last_tick(),
                        diag_slot_rt_status_max_gap(),
                        timer_from_us(200000),
                        0); // no event tag — emit gap shows up as missing emits

#if defined(__linux__) || defined(__APPLE__)
    uint8_t status = runtime_handle_status(runtime_handle);
    int32_t c0 = kalico_runtime_get_stepper_count(runtime_handle, 0);
    int32_t c1 = kalico_runtime_get_stepper_count(runtime_handle, 1);
    int32_t c2 = kalico_runtime_get_stepper_count(runtime_handle, 2);
    extern uint32_t kalico_runtime_get_xdirect_write_count(void);
    uint32_t spi_writes = kalico_runtime_get_xdirect_write_count();
    fprintf(stderr,
        "[sim-progress] status=%u counts=[%d,%d,%d]"
        " spi_writes=%u\n",
        status, c0, c1, c2, spi_writes);
    fflush(stderr);
#endif
}
DECL_TASK(runtime_status_drain);

extern void runtime_emit_step_pulses(uint8_t motor_idx, int32_t n_steps);

// Step-output timer wiring (TIM3 on H7, TIM2 on F4). Step-output ISR runs at
// the same NVIC priority as TIM5, so the kick from the TIM5 ISR is SPSC-safe
// (see kalico_nvic_prio.h).

extern void step_output_timer_arm(uint32_t cycle_abs);
extern uint32_t step_output_timer_armed_target(void);
extern uint8_t step_output_timer_is_running(void);

// Read by Rust to scope the soonest-across scan.
static uint8_t step_output_owned_mask;

// used,externally_visible: Rust-only caller; must survive --gc-sections LTO.
__attribute__((used, externally_visible))
uint8_t
kalico_step_output_owned_mask(void)
{
    return step_output_owned_mask;
}

// Idempotent; does NOT arm the timer.
void
arm_per_axis_step_timer(uint8_t axis_idx)
{
    if (axis_idx >= 4)
        return;
    step_output_owned_mask |= (uint8_t)(1u << axis_idx);
}

// Producer kick from the TIM5 ISR; same-priority as the step-output ISR, so the
// compare write is non-racing. used,externally_visible: Rust-only caller.
__attribute__((used, externally_visible))
void
kalico_kick_step_output(uint8_t axis_idx, uint32_t cycle_abs)
{
    if (axis_idx >= 4)
        return;
    step_output_owned_mask |= (uint8_t)(1u << axis_idx);

    if (!step_output_timer_is_running()) {
        step_output_timer_arm(cycle_abs);
        return;
    }
    // Pull compare forward only if the new step is sooner (wrap-safe).
    uint32_t cur = step_output_timer_armed_target();
    if ((int32_t)(cycle_abs - cur) < 0)
        step_output_timer_arm(cycle_abs);
}

