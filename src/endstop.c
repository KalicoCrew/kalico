// Cross-MCU homing endstop watch.
//
// Polls a digital input in the cooperative foreground (no EXTI — a second async
// context would race the TIM5 sample ISR and the widened-clock seqlock). On the
// active edge it captures the exact widened u64 MCU clock at detection — not at
// emit, so transport/CRC latency cannot smear the trip time the host uses to
// reconstruct position — and ships a single EndstopTrip event on the events
// channel. The host turns that event into the stop broadcast; this MCU does not
// touch its own motion. This is the only firmware code that knows "homing".

#include "basecmd.h"        // oid_alloc, oid_lookup, foreach_oid
#include "board/gpio.h"     // struct gpio_in, gpio_in_setup, gpio_in_read
#include "board/misc.h"     // timer_read_time
#include "command.h"        // DECL_COMMAND, shutdown
#include "sched.h"          // struct timer, sched_add_timer, sched_del_timer
#include "kalico_runtime.h" // kalico_runtime_now_ticks
#include "kalico_dispatch.h" // kalico_native_emit_endstop_trip

extern void *runtime_handle;

struct endstop {
    struct timer time;
    uint32_t rest_ticks;
    uint32_t pin_id;
    struct gpio_in pin;
    uint8_t endstop_id;
    uint8_t invert;
    uint8_t armed;
};

static uint_fast8_t
endstop_event(struct timer *t)
{
    struct endstop *e = container_of(t, struct endstop, time);
    uint8_t raw = gpio_in_read(e->pin) ? 1 : 0;
    uint8_t active = raw ^ e->invert;
    if (active && e->armed) {
        uint64_t trip_clock = kalico_runtime_now_ticks(runtime_handle);
        e->armed = 0;
        kalico_native_emit_endstop_trip(e->endstop_id, trip_clock);
        return SF_DONE;
    }
    e->time.waketime += e->rest_ticks;
    return SF_RESCHEDULE;
}

void
command_config_endstop(uint32_t *args)
{
    struct endstop *e = oid_alloc(args[0], command_config_endstop, sizeof(*e));
    e->endstop_id = args[1];
    e->pin_id = args[2];
    e->pin = gpio_in_setup(args[2], args[3]);
    e->invert = args[4] ? 1 : 0;
    e->armed = 0;
    e->time.func = endstop_event;

    // Reject two endstops bound to the same physical pin. foreach_oid resets
    // per connection (the host reallocates oids on connect), so this never
    // false-trips across reconnects.
    uint8_t oid;
    struct endstop *other;
    foreach_oid(oid, other, command_config_endstop) {
        if (other != e && other->pin_id == e->pin_id)
            shutdown("endstop: duplicate pin");
    }
}
DECL_COMMAND(command_config_endstop,
             "config_endstop oid=%c endstop_id=%c pin=%u pull_up=%c invert=%c");

// Arm (or, with rest_ticks==0, disarm) the watch. Polling starts immediately —
// the trip clock is captured at edge detection, so the start instant is
// irrelevant and we avoid depending on a host/MCU clock correspondence.
void
command_query_endstop(uint32_t *args)
{
    struct endstop *e = oid_lookup(args[0], command_config_endstop);
    sched_del_timer(&e->time);
    e->rest_ticks = args[1];
    if (!e->rest_ticks) {
        e->armed = 0;
        return;
    }
    e->armed = 1;
    e->time.waketime = timer_read_time() + e->rest_ticks;
    sched_add_timer(&e->time);
}
DECL_COMMAND(command_query_endstop,
             "query_endstop oid=%c rest_ticks=%u");

// Passive read of the current pin state — no arming, no trip, no motion. Used
// for QUERY_ENDSTOPS and for verifying endstop polarity during bring-up.
void
command_endstop_query_state(uint32_t *args)
{
    struct endstop *e = oid_lookup(args[0], command_config_endstop);
    uint8_t raw = gpio_in_read(e->pin) ? 1 : 0;
    sendf("endstop_state oid=%c armed=%c pin_value=%c", args[0], e->armed, raw);
}
DECL_COMMAND(command_endstop_query_state, "endstop_query_state oid=%c");
