// Klipper command surface for the kalico runtime; also hosts the endstop
// arm/disarm commands and the per-tick endstop sampler.

#include <stdint.h>
#include <stdio.h>
#include "autoconf.h"
#include "board/gpio.h"           // gpio_in_setup / gpio_in_read / spi_setup
#include "command.h"              // DECL_COMMAND, sendf, command_decode_ptr
#include "sched.h"                // DECL_TASK
#include "board/misc.h"           // timer_read_time
#include "kalico_log.h"           // kalico_log_emit
#include "kalico_runtime.h"       // FFI export prototypes
#include "kalico_dispatch.h"      // kalico_native_emit_*
#include "trsync.h"               // trsync_add_signal, trsync_oid_lookup
#include "compiler.h"             // container_of
#if CONFIG_MACH_STM32
#include "stm32/phase_stepping_spi.h"
#elif CONFIG_MACH_LINUX
#include "linux/phase_stepping_spi.h"
#endif


extern void *runtime_handle;      // defined in src/runtime_tick.c

void
command_runtime_query_status(uint32_t *args)
{
    if (!runtime_handle) {
        sendf("kalico_status status=%c last_err=%i phase_spi_skip_count=%u",
              (uint8_t)255, -7, 0u);
        return;
    }
    uint8_t status = runtime_handle_status(runtime_handle);
    int32_t last_err = runtime_handle_last_error(runtime_handle);
    uint32_t phase_skip = 0;
#if CONFIG_MACH_STM32 || CONFIG_MACH_LINUX
    phase_skip = phase_spi_get_skip_count();
#endif
    sendf("kalico_status status=%c last_err=%i phase_spi_skip_count=%u",
          status, last_err, phase_skip);
}
DECL_COMMAND(command_runtime_query_status, "runtime_query_status");

// Dedicated endstop poll task — the mainline src/endstop.c architecture: each
// armed GPIO/StallGuard source gets its own sched timer that polls the pin off
// the motion path (NOT inside the TIM5 ISR), oversample-debounces exactly like
// mainline endstop_event/endstop_oversample_event, and on a confirmed trip
// stops TIM5 (the motion clock) and publishes the trip via kalico_software_trip.
// Removing detection from the motion ISR means the engine pays zero endstop
// cost when not homing, and a trip is an imperative stop rather than a poll.
#define KALICO_ENDSTOP_MAX_SOURCES 4
#define KALICO_ENDSTOP_SOURCE_RECORD_LEN 6

extern void runtime_tick_disable(void);
extern void runtime_tick_enable(void);
extern void kalico_runtime_request_tick_baseline_reset(void);
extern int32_t kalico_runtime_discard_pending(void *rt);
extern uint32_t stats_send_time;        // basecmd.c
extern uint32_t stats_send_time_high;   // basecmd.c

enum { EPF_PIN_HIGH = 1 << 0 };

// Poll cadence: slow rest sampling between candidates, fast oversampling to
// confirm — same shape as mainline. ~10 kHz rest poll, 15 µs oversample, 4
// confirmations: ~100 µs detection latency plus a 60 µs debounce window.
#define ENDSTOP_REST_TICKS   (timer_from_us(100))
#define ENDSTOP_SAMPLE_TICKS (timer_from_us(15))

struct endstop_poll {
    struct timer time;
    struct gpio_in pin;
    uint32_t nextwake;
    uint32_t arm_id;
    uint8_t flags;          // EPF_PIN_HIGH = asserted (triggered) pin level
    uint8_t sample_count;
    uint8_t trigger_count;
    uint8_t active;
};
static struct endstop_poll endstop_polls[KALICO_ENDSTOP_MAX_SOURCES];

static uint_fast8_t endstop_poll_oversample(struct timer *t);

// Confirmed trip: stop the motion clock first (imperative halt), then publish
// the trip through the software-trip path so the host learns it (and computes
// the trigger position from the commanded trajectory at trip_clock). Runs in
// timer-IRQ context — kalico_software_trip and runtime_tick_disable are both
// IRQ-safe (atomics / register writes only).
static void
endstop_poll_fire(struct endstop_poll *e)
{
    runtime_tick_disable();
    uint32_t clock_lo = timer_read_time();
    uint32_t clock_hi = stats_send_time_high + (clock_lo < stats_send_time);
    uint8_t status = 1;
    (void)kalico_software_trip(e->arm_id, clock_lo, clock_hi, &status);
    kalico_log_emit(KALICO_LOG_LEVEL_WARN, KALICO_LOG_SUBSYS_ENDSTOP,
                    KALICO_LOG_EVENT_ENDSTOP_TIM5_HALTED, 0,
                    e->arm_id, clock_lo);
    e->active = 0;
}

static uint_fast8_t
endstop_poll_event(struct timer *t)
{
    struct endstop_poll *e = container_of(t, struct endstop_poll, time);
    uint8_t val = gpio_in_read(e->pin);
    uint32_t nextwake = e->time.waketime + ENDSTOP_REST_TICKS;
    if ((val ? ~e->flags : e->flags) & EPF_PIN_HIGH) {
        // No match — keep slow-polling.
        e->time.waketime = nextwake;
        return SF_RESCHEDULE;
    }
    e->nextwake = nextwake;
    e->time.func = endstop_poll_oversample;
    return endstop_poll_oversample(t);
}

static uint_fast8_t
endstop_poll_oversample(struct timer *t)
{
    struct endstop_poll *e = container_of(t, struct endstop_poll, time);
    uint8_t val = gpio_in_read(e->pin);
    if ((val ? ~e->flags : e->flags) & EPF_PIN_HIGH) {
        // No longer matching — debounce reject, back to slow poll.
        e->time.func = endstop_poll_event;
        e->time.waketime = e->nextwake;
        e->trigger_count = e->sample_count;
        return SF_RESCHEDULE;
    }
    uint8_t count = e->trigger_count - 1;
    if (!count) {
        endstop_poll_fire(e);
        return SF_DONE;
    }
    e->trigger_count = count;
    e->time.waketime += ENDSTOP_SAMPLE_TICKS;
    return SF_RESCHEDULE;
}

static void
endstop_polls_cancel(void)
{
    for (int i = 0; i < KALICO_ENDSTOP_MAX_SOURCES; i++) {
        if (!endstop_polls[i].active)
            continue;
        sched_del_timer(&endstop_polls[i].time);
        endstop_polls[i].active = 0;
    }
}

// Record layout mirrors rust/kalico-c-api/src/runtime_ffi.rs::kalico_endstop_arm
// decode: kind u8, gpio u16 LE, active_high u8, policy u8, sample_n u8 — 6 bytes.
//
// TMC DIAG outputs are open-drain (GCONF.diag1_int_pushpull==0 at reset) and
// float LOW without a pullup, so a `^!PG9`-style config reads asserted at idle.
// The host's pullup flag is not on the wire yet, so apply "TmcDiag → pullup,
// Physical → no pull" here.
static void
endstop_polls_arm(uint32_t arm_id, uint8_t source_count,
                  const uint8_t *sources_ptr)
{
    endstop_polls_cancel();
    if (!sources_ptr || source_count == 0)
        return;
    uint8_t n = source_count;
    if (n > KALICO_ENDSTOP_MAX_SOURCES)
        n = KALICO_ENDSTOP_MAX_SOURCES;
    uint32_t now = timer_read_time();
    for (uint8_t i = 0; i < n; i++) {
        const uint8_t *r = sources_ptr + (uint32_t)i * KALICO_ENDSTOP_SOURCE_RECORD_LEN;
        uint8_t kind = r[0];   // 0=Physical, 1=TmcDiag, 2=Software
        if (kind == 2)
            continue;   // Software: trip arrives via the relay, no local poll
        uint16_t gpio_id = (uint16_t)r[1] | ((uint16_t)r[2] << 8);
        uint8_t active_high = r[3];
        uint8_t sample_n = r[5] ? r[5] : 1;
        int32_t pull_up = (kind == 1) ? 1 : 0;
        struct endstop_poll *e = &endstop_polls[i];
        e->pin = gpio_in_setup((uint8_t)gpio_id, pull_up);
        e->flags = active_high ? EPF_PIN_HIGH : 0;
        e->sample_count = sample_n;
        e->trigger_count = sample_n;
        e->arm_id = arm_id;
        e->active = 1;
        e->time.func = endstop_poll_event;
        e->time.waketime = now + ENDSTOP_REST_TICKS;
        sched_add_timer(&e->time);
    }
}

void
command_runtime_arm_endstop(uint32_t *args)
{
#if CONFIG_MACH_LINUX
    fprintf(stderr, "[mcu-arm] command_runtime_arm_endstop entered arm_id=%u\n", args[0]);
    fflush(stderr);
#endif
    uint32_t arm_id = args[0];
    uint32_t arm_clock_lo = args[1];
    uint32_t arm_clock_hi = args[2];
    uint8_t source_count = args[3];
    uint32_t sources_len = args[4];
    // PT_buffer args carry an encoded pointer; command_decode_ptr resolves it.
    // A bare cast works on 32-bit MCUs but segfaults on 64-bit sim.
    uint8_t *sources_ptr = command_decode_ptr(args[5]);
    uint8_t stepper_count = args[6];
    uint32_t steppers_len = args[7];
    uint8_t *steppers_ptr = command_decode_ptr(args[8]);
    uint8_t status = 2; // Rejected
    (void)kalico_endstop_arm(arm_id, arm_clock_lo, arm_clock_hi,
                             source_count, sources_ptr, sources_len,
                             stepper_count, steppers_ptr, steppers_len,
                             &status);
    // status: 0=Armed, 1=AlreadyTripped, 2=Rejected. Schedule the poll task
    // only on Armed; AlreadyTripped already published its snapshot.
    if (status == 0)
        endstop_polls_arm(arm_id, source_count, sources_ptr);
    sendf("kalico_arm_endstop_response arm_id=%u status=%c", arm_id, status);
}
DECL_COMMAND(command_runtime_arm_endstop,
    "runtime_arm_endstop arm_id=%u arm_clock_lo=%u arm_clock_hi=%u "
    "source_count=%c sources=%*s "
    "stepper_count=%c steppers=%*s");

void
command_runtime_disarm_endstop(uint32_t *args)
{
    uint32_t arm_id = args[0];
    uint8_t status = 2; // Unknown
    (void)kalico_endstop_disarm(arm_id, &status);
    endstop_polls_cancel();
    // status==1 (AlreadyTripped) means a trip halted TIM5 mid-move: TIM5 is
    // stopped, so it is safe to drain the abandoned ring pieces, and we must
    // restart the motion clock. Draining first removes the stale (now far in
    // the past) pieces that would otherwise fault PieceStartInPast on restart;
    // the baseline reset stops the stopped-window gap from faulting the
    // tick-interval guard. status==0 (Disarmed, no trip) leaves TIM5 running.
    if (status == 1) {
        kalico_runtime_discard_pending(runtime_handle);
        kalico_runtime_request_tick_baseline_reset();
        runtime_tick_enable();
    }
    sendf("kalico_disarm_endstop_response arm_id=%u status=%c", arm_id, status);
}
DECL_COMMAND(command_runtime_disarm_endstop, "runtime_disarm_endstop arm_id=%u");

void
command_runtime_software_trip(uint32_t *args)
{
    uint32_t arm_id = args[0];
    uint32_t clock_lo = timer_read_time();
    uint32_t clock_hi = stats_send_time_high + (clock_lo < stats_send_time);
    uint8_t status = 1; // NotArmed default
    (void)kalico_software_trip(arm_id, clock_lo, clock_hi, &status);
    // A confirmed trip (status==0) stops the motion clock on this MCU — the
    // sink side of cross-MCU homing (relay → trsync_trigger → here).
    if (status == 0)
        runtime_tick_disable();
    sendf("kalico_software_trip_response arm_id=%u status=%c",
          arm_id, status);
}
DECL_COMMAND(command_runtime_software_trip,
    "runtime_software_trip arm_id=%u");

// ---- runtime_stop_on_trigger: trsync signal that freezes the curve evaluator
//
// This is the bridge twin of stepper.c's stepper_stop_on_trigger. Where
// stepper_stop clears the (unused-in-bridge) C step queue, this freezes the
// curve evaluator via kalico_software_trip. The bridge reactor's TripDispatch
// relays `trsync_trigger` here; trsync_do_trigger fires this signal.
//
// One active homing arm per MCU at a time, so a single static instance is
// sufficient. (Multiple concurrent arms would need an array keyed by trsync.)
// Re-arming while the prior signal is still registered is caught loudly by
// trsync_add_signal itself (shutdown("Can't add signal that is already
// active") in src/trsync.c), so the one-arm contract is enforced fail-loudly
// rather than silently overwriting a live binding.
static struct runtime_stop_binding {
    struct trsync_signal signal;
    uint32_t arm_id;
} runtime_stop_binding;

// IRQ-context invariant: trsync_do_trigger invokes this callback inside an
// irq_save()/irq_restore() critical section, reachable from the timer-IRQ
// path (trsync_expire_event) and from the endstop GPIO IRQ. So the C->Rust
// kalico_software_trip call below MUST be non-blocking, allocation-free, and
// lock-free against the curve-eval path. It is: it only records a trip into
// the endstop state. (See docs/kalico-rewrite/mcu-c-rust-boundary.md.)
static void
runtime_stop_on_trigger_cb(struct trsync_signal *tss, uint8_t reason)
{
    struct runtime_stop_binding *b =
        container_of(tss, struct runtime_stop_binding, signal);
    kalico_log_emit(KALICO_LOG_LEVEL_DEBUG, KALICO_LOG_SUBSYS_ENDSTOP,
                    KALICO_LOG_EVENT_ENDSTOP_STOP_CB_ENTER, 0,
                    b->arm_id, (uint32_t)reason);
    uint32_t clock_lo = timer_read_time();
    uint32_t clock_hi = stats_send_time_high + (clock_lo < stats_send_time);
    // NotArmed default; no reply channel from trigger/IRQ context.
    uint8_t status = 1;
    (void)kalico_software_trip(b->arm_id, clock_lo, clock_hi, &status);
    // Confirmed trip stops the motion clock on this (sink) MCU — the cross-MCU
    // freeze for relayed homing trips.
    if (status == 0)
        runtime_tick_disable();
}

void
command_runtime_stop_on_trigger(uint32_t *args)
{
    uint32_t arm_id = args[0];
    struct trsync *ts = trsync_oid_lookup(args[1]);
    runtime_stop_binding.arm_id = arm_id;
    trsync_add_signal(ts, &runtime_stop_binding.signal,
                      runtime_stop_on_trigger_cb);
}
DECL_COMMAND(command_runtime_stop_on_trigger,
    "runtime_stop_on_trigger arm_id=%u trsync_oid=%c");


// Seed the MCU engine's position origin (SET_KINEMATIC_POSITION) so prev_x/y/z
// match the host's commanded position before the first segment, avoiding a
// huge first-segment delta. Positions are Q16.16 fixed-point (mm * 65536).
// Fire-and-forget; the following PushSegment provides sequencing.
void
command_runtime_seed_position(uint32_t *args)
{
    int32_t x_q16 = (int32_t)args[0];
    int32_t y_q16 = (int32_t)args[1];
    int32_t z_q16 = (int32_t)args[2];
    if (!runtime_handle)
        return;
    (void)kalico_runtime_seed_position(runtime_handle, x_q16, y_q16, z_q16);
}
DECL_COMMAND(command_runtime_seed_position,
    "runtime_seed_position x_q16=%i y_q16=%i z_q16=%i");

void
command_runtime_stream_flush(uint32_t *args)
{
    (void)args;
    if (!runtime_handle) {
        sendf("kalico_stream_flush_response result=%i credit_epoch=%u", -7, 0);
        return;
    }
    uint32_t credit_epoch = 0;
    int32_t r = kalico_runtime_stream_flush(runtime_handle, &credit_epoch);
    sendf("kalico_stream_flush_response result=%i credit_epoch=%u",
          r, credit_epoch);
}
DECL_COMMAND(command_runtime_stream_flush, "runtime_stream_flush");

// Widen the MCU clock in C with command_get_uptime's formula instead of the
// Rust FFI: runtime::stream::clock_sync_respond reads a TIM5-ISR-populated
// seqlock that the host filters as uninitialised in the all-StepTime path.
// (stats_send_time / stats_send_time_high are externed at the top of the file.)
void
command_runtime_clock_sync_request(uint32_t *args)
{
    uint32_t request_id = args[0];
    // args[1]/args[2] = host_send_time_{lo,hi} — unused; retained on the wire.
    uint32_t low = timer_read_time();
    uint32_t high = stats_send_time_high + (low < stats_send_time);
    sendf(
        "kalico_clock_sync_response request_id=%u mcu_clock_lo=%u mcu_clock_hi=%u",
        request_id, low, high);
}
DECL_COMMAND(command_runtime_clock_sync_request,
    "runtime_clock_sync_request request_id=%u "
    "host_send_time_lo=%u host_send_time_hi=%u");

// Two-stage phase-stepping registration, both before the first
// kalico_configure_axis: register_phase_bus once per bus_id (shared SPI cfg),
// register_phase_motor once per motor (its own CS GPIO — multiple TMC5160s
// share a bus). Non-STM32 hosts return -88.
void
command_runtime_register_phase_bus(uint32_t *args)
{
#if CONFIG_MACH_STM32 || CONFIG_MACH_LINUX
    uint8_t bus_id = (uint8_t)args[0];
    uint32_t rate = args[1];
    struct spi_config cfg = spi_setup(bus_id, 3 /* mode 3, TMC SPI */, rate);
    phase_stepping_register_bus(bus_id, cfg);
    sendf("kalico_register_phase_bus_response result=%i", 0);
#else
    (void)args;
    sendf("kalico_register_phase_bus_response result=%i", -88);
#endif
}
DECL_COMMAND(command_runtime_register_phase_bus,
    "runtime_register_phase_bus bus_id=%c rate=%u");

// Param is cs_pin_id, not cs_pin: msgproto resolves any `*_pin` param against
// the pin enumeration, which would force symbolic pin names instead of the raw
// GPIO encoding (port*16+pin) the rest of the phase_config surface uses.
void
command_runtime_register_phase_motor(uint32_t *args)
{
#if CONFIG_MACH_STM32 || CONFIG_MACH_LINUX
    uint8_t motor_idx = (uint8_t)args[0];
    uint8_t bus_id    = (uint8_t)args[1];
    uint8_t cs_pin_id = (uint8_t)args[2];
    phase_stepping_register_motor(motor_idx, bus_id, cs_pin_id);
    sendf("kalico_register_phase_motor_response result=%i", 0);
#else
    (void)args;
    sendf("kalico_register_phase_motor_response result=%i", -88);
#endif
}
DECL_COMMAND(command_runtime_register_phase_motor,
    "runtime_register_phase_motor motor_idx=%c bus_id=%c cs_pin_id=%c");

