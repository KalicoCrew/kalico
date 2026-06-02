// src/runtime_tick.c
//
// Klipper-side lifecycle for the kalico runtime: DECL_INIT brings up the
// Rust runtime + the per-family tick backend; DECL_TASK pumps drain the
// Rust → Klipper response queue. Shared globals (runtime_handle,
// runtime_clock_freq, runtime_aligned_*) live here as the single
// definition site.
//
// Klipper command surface is in src/runtime_commands.c.
// Sim-only commands are in src/runtime_sim_commands.c (gated CONFIG_KALICO_SIM).
// Bench is in src/generic/runtime_bench.c (gated CONFIG_RUNTIME_BENCH).
// Per-family backends:
//   src/stm32/runtime_tick_h7.c   (H7 TIM5 ISR)
//   src/linux/runtime_tick_host.c (pthread tick for host-sim)
// Backend interface contract: src/generic/runtime_tick.h.

#include <string.h>         // memcpy
#include "stepper.h"        // runtime_emit_step_pulses (via stepper.c)
#if defined(__linux__) || defined(__APPLE__)
#include <stdio.h>          // fprintf, stderr
#include <time.h>           // clock_gettime
#endif
#include "autoconf.h"
#include "board/gpio.h"     // gpio_in_setup / gpio_in_read
#include "board/internal.h" // NVIC_*, IWDG, OTG_HS_IRQn, USART2_IRQn
#include "board/irq.h"      // irq_save, irq_restore (Step-6 §8.5 flush)
#include "board/misc.h"     // timer_read_time
#include "command.h"        // DECL_COMMAND
#include "sched.h"          // DECL_INIT, DECL_TASK
#include "kalico_runtime.h"
#include "kalico_dispatch.h" // kalico_native_emit_*
#include "generic/runtime_tick.h"   // backend interface (consumer view)
#include "generic/fault_handler.h"  // diag_record_engine_xition, diag_take_snapshot


// H7 CMSIS only defines IWDG1/IWDG2; map the generic name to IWDG1
// (matching src/stm32/watchdog.c's pattern) so the bench-loop kick
// compiles cleanly.
#if CONFIG_MACH_STM32H7
#define IWDG IWDG1
#endif

// Exposed to Rust via `extern "C" { static runtime_clock_freq: u32; }`.
// __attribute__((used, externally_visible)) survives -fwhole-program LTO + GC.
const uint32_t runtime_clock_freq __attribute__((used, externally_visible))
    = CONFIG_CLOCK_FREQ;

const uint32_t runtime_sample_rate_hz __attribute__((used, externally_visible))
    = CONFIG_KALICO_MOTION_SAMPLE_RATE_HZ;


extern volatile uint8_t runtime_liveness_ok;  // defined in src/stm32/watchdog.c

// Foreground-only host-clock helper (flush ack-wait timeout). NEVER call from ISR.
__attribute__((used, externally_visible))
uint64_t
runtime_host_now_us(void)
{
    uint32_t cycles = timer_read_time();
    return ((uint64_t)cycles) / (CONFIG_CLOCK_FREQ / 1000000U);
}

// Boot/dispatch progress: (tag, stage, value) packed into a u32, piggybacked
// onto the 10 Hz kalico_status_v6 fault_detail field. bits[31:24]=tag,
// [23:16]=stage, [15:0]=value. Volatile: foreground single-threaded, no atomics needed.
volatile uint32_t runtime_diag_last_packed __attribute__((used, externally_visible));

// Crash diag survives NVIC_SystemReset via .persistent_diag (NOLOAD, outside bss).
// Next boot checks magic == RT_DIAG_MAGIC and emits the prior stage via output().
#define RT_DIAG_MAGIC 0xD1A6BABE

struct rt_diag_persistent {
    uint32_t magic;
    uint32_t last_packed;
    uint32_t last_us;
    uint32_t fault_count;
};

volatile struct rt_diag_persistent rt_diag_persistent
    __attribute__((section(".persistent_diag"), used, externally_visible));

// Prior-boot packed-diag snapshot, captured at runtime_init before the current
// run overwrites rt_diag_persistent. Status drain emits both live and snapshot.
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

// Klipper-stats-based widened clock: advances regardless of engine state
// (Idle/Drained/Fault), unlike the ISR-published engine widened_now.
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

// Thin wrappers so the Rust staticlib can call irq_save/irq_restore without
// LTO DCE-ing the standalone symbols (used,externally_visible keeps them).
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
struct task_wake runtime_drain_wake;  // non-static: shared with runtime_sim_commands.c
static struct timer runtime_drain_timer;

// 10 Hz status frame state; prev_engine_status gates the one-shot kalico_fault emit.
static uint32_t last_status_emit_time = 0;
static uint8_t prev_engine_status = 0;

// Liveness monitor state.
static uint32_t last_seen_tick_counter = 0;
static uint32_t last_progress_time = 0;

static uint8_t last_seen_status = 255;

// ~1 kHz drain wake. Reschedule from now (not +=1ms) to avoid "timer in past"
// shutdown if the foreground stalls for >1 ms.
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
    // Capture prior-boot diag FIRST (before our markers overwrite it).
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

// runtime_sim_drain_calls extern retired with runtime_sim_commands.c in
// 085b4b16f; the diag-heartbeat scaffolding now lives in
// diag_task_heartbeat below.

void
runtime_drain(void)
{
    if (!runtime_handle) return;
    if (!sched_check_wake(&runtime_drain_wake)) return;

    diag_task_heartbeat(diag_slot_rt_drain_calls(),
                        diag_slot_rt_drain_last_tick(),
                        diag_slot_rt_drain_max_gap(),
                        timer_from_us(50000),
                        0); // no event tag — runtime_drain idle gaps are normal

    // Liveness: only acts on RUNNING; refreshes anchor in other states so a
    // transition INTO RUNNING doesn't immediately trip on a stale anchor.
    uint32_t cur_counter = runtime_handle_tick_counter(runtime_handle);
    uint32_t cur_time = timer_read_time();
    uint8_t cur_status = runtime_handle_status(runtime_handle);
    if (cur_status == 1 /* RUNNING */) {
        if (cur_counter != last_seen_tick_counter) {
            last_seen_tick_counter = cur_counter;
            last_progress_time = cur_time;
        } else if ((cur_time - last_progress_time) > KALICO_LIVENESS_THRESHOLD_TICKS) {
            // ISR has stalled while RUNNING. Stop kicking the watchdog.
            runtime_liveness_ok = 0;
        }
    } else {
        last_progress_time = cur_time;
        last_seen_tick_counter = cur_counter;
    }

    // One-shot kalico_fault emit on FAULT transition; also block watchdog kicks.
    if (cur_status == 3 /* FAULT */) {
        runtime_liveness_ok = 0;
        if (prev_engine_status != 3 /* FAULT */) {
            int32_t fault_code = runtime_handle_last_error(runtime_handle);
            uint32_t fault_detail = runtime_handle_fault_detail(runtime_handle);
            // tick_blocker_pc: stacked PC from TIM5 exception frame on -311 path.
            // Feed to addr2line to identify the code holding the CPU at the late tick.
            uint32_t tick_blocker_pc = runtime_handle_tick_blocker_pc(runtime_handle);
            kalico_native_emit_fault_event((uint16_t)fault_code, fault_detail,
                                           tick_blocker_pc);
        }
    }

    // Live fault escalation: emit + Klipper shutdown on fresh nonzero last_error.
    // shutdown() is safe in foreground (DECL_TASK) but NOT from ISR.
    // last_acted_error (static) suppresses re-emit on the post-longjmp trailing pass.
    static int32_t last_acted_error;
    int32_t cur_error = runtime_handle_last_error(runtime_handle);
    if (cur_error != 0 && cur_error != last_acted_error) {
        last_acted_error = cur_error;
        uint32_t fdetail = runtime_handle_fault_detail(runtime_handle);
        uint32_t tick_blocker_pc = runtime_handle_tick_blocker_pc(runtime_handle);
        kalico_native_emit_fault_event((uint16_t)cur_error, fdetail,
                                       tick_blocker_pc);
        // Persist into BKPSRAM ring before shutdown resets the USB stack.
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
}
DECL_TASK(runtime_drain);

// TIM5 off on Klipper shutdown; step-output timer parks itself (no new steps pushed).
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

// Command surface and endstop GPIO sampler live in src/runtime_commands.c.
// This file: lifecycle (init/drain), sibling drains, shared globals.

DECL_CTR("_DECL_OUTPUT "
         "kalico_endstop_tripped arm_id=%u "
         "trip_clock_lo=%u trip_clock_hi=%u "
         "trip_source_idx=%c fmt_version=%c "
         "stepper_count=%c stepper_data=%*s");

void
runtime_endstop_drain(void)
{
    if (!runtime_handle) return;
    uint8_t buf[64];
    size_t actual = 0;
    int32_t r = kalico_endstop_poll_trip(buf, sizeof(buf), &actual);
    if (r != 1 || actual < 15) return;
    uint32_t arm_id     = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8)
                        | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
    uint32_t clock_lo   = (uint32_t)buf[4] | ((uint32_t)buf[5] << 8)
                        | ((uint32_t)buf[6] << 16) | ((uint32_t)buf[7] << 24);
    uint32_t clock_hi   = (uint32_t)buf[8] | ((uint32_t)buf[9] << 8)
                        | ((uint32_t)buf[10] << 16) | ((uint32_t)buf[11] << 24);
    uint8_t source_idx  = buf[12];
    uint8_t fmt_version = buf[13];
    uint8_t stepper_n   = buf[14];
    uint32_t blob_len   = (uint32_t)stepper_n * 5;
    if (15 + blob_len > actual) return;
    output("kalico_endstop_tripped arm_id=%u "
           "trip_clock_lo=%u trip_clock_hi=%u "
           "trip_source_idx=%c fmt_version=%c "
           "stepper_count=%c stepper_data=%*s",
           arm_id, clock_lo, clock_hi,
           source_idx, fmt_version,
           stepper_n, blob_len, &buf[15]);
}
DECL_TASK(runtime_endstop_drain);

extern void runtime_emit_step_pulses(uint8_t motor_idx, int32_t n_steps);

// ===========================================================================
// Step-output timer wiring (TIM3 on H7, TIM2 on F4)
// ===========================================================================
// ISR calls kalico_step_output_event() (per_axis_timer.rs): emits due steps,
// returns soonest remaining head or DISABLE. Same NVIC priority as TIM5 —
// SPSC non-racing, kick from TIM5 ISR is safe (see kalico_nvic_prio.h).

// Sentinel mirrored from per_axis_timer.rs::STEP_OUTPUT_DISABLE.
#define KALICO_STEP_OUTPUT_DISABLE 0xFFFFFFFFu

extern void step_output_timer_arm(uint32_t cycle_abs);
extern uint32_t step_output_timer_armed_target(void);
extern uint8_t step_output_timer_is_running(void);

// Bitmask of axes this MCU owns; read by Rust to scope the soonest-across scan.
static uint8_t step_output_owned_mask;

// used,externally_visible: referenced only from Rust; must survive --gc-sections LTO.
__attribute__((used, externally_visible))
uint8_t
kalico_step_output_owned_mask(void)
{
    return step_output_owned_mask;
}

// Register axis ownership for the soonest-across scan. Idempotent; does NOT arm the timer.
void
arm_per_axis_step_timer(uint8_t axis_idx)
{
    if (axis_idx >= 4)
        return;
    step_output_owned_mask |= (uint8_t)(1u << axis_idx);
}

// (Re)arm the step-output timer no later than cycle_abs (producer kick from TIM5 ISR).
// Same-priority as step-output ISR: no nesting, compare write is non-racing.
// used,externally_visible: referenced only from Rust archive; must survive LTO.
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
    // Pull compare forward only if the new step is sooner (wrap-safe signed compare).
    uint32_t cur = step_output_timer_armed_target();
    if ((int32_t)(cycle_abs - cur) < 0)
        step_output_timer_arm(cycle_abs);
}

