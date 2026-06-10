#include "basecmd.h"
#include "board/gpio.h"
#include "board/misc.h"
#include "command.h"
#include "sched.h"
#include "kalico_runtime.h"
#include "kalico_dispatch.h"

extern void *runtime_handle;

struct endstop {
    struct timer time;
    uint32_t rest_ticks;
    uint32_t pin_id;
    struct gpio_in pin;
    uint64_t trip_clock;
    uint8_t endstop_id;
    uint8_t invert;
    uint8_t armed;
    uint8_t trip_pending;
};

static struct task_wake endstop_trip_wake;

// Timer context (IRQ): capture the trip clock here for accuracy, but defer
// the transport write to the task below — kalico_transport_send_frame uses a
// shared tx_buf and the USB transmit cursor, neither safe against the
// foreground from IRQ.
static uint_fast8_t
endstop_event(struct timer *t)
{
    struct endstop *e = container_of(t, struct endstop, time);
    uint8_t raw = gpio_in_read(e->pin) ? 1 : 0;
    uint8_t active = raw ^ e->invert;
    if (active && e->armed) {
        e->trip_clock = kalico_runtime_now_ticks(runtime_handle);
        e->armed = 0;
        e->trip_pending = 1;
        sched_wake_task(&endstop_trip_wake);
        return SF_DONE;
    }
    e->time.waketime += e->rest_ticks;
    return SF_RESCHEDULE;
}

void
endstop_trip_task(void)
{
    if (!sched_check_wake(&endstop_trip_wake))
        return;
    uint8_t oid;
    struct endstop *e;
    foreach_oid(oid, e, command_config_endstop) {
        if (!e->trip_pending)
            continue;
        e->trip_pending = 0;
        kalico_native_emit_endstop_trip(e->endstop_id, e->trip_clock);
    }
}
DECL_TASK(endstop_trip_task);

void
command_config_endstop(uint32_t *args)
{
    struct endstop *e = oid_alloc(args[0], command_config_endstop, sizeof(*e));
    e->endstop_id = args[1];
    e->pin_id = args[2];
    e->pin = gpio_in_setup(args[2], args[3]);
    e->invert = args[4] ? 1 : 0;
    e->armed = 0;
    e->trip_pending = 0;
    e->time.func = endstop_event;

    uint8_t oid;
    struct endstop *other;
    foreach_oid(oid, other, command_config_endstop) {
        if (other != e && other->pin_id == e->pin_id)
            shutdown("endstop: duplicate pin");
    }
}
DECL_COMMAND(command_config_endstop,
             "config_endstop oid=%c endstop_id=%c pin=%u pull_up=%c invert=%c");

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

void
command_endstop_query_state(uint32_t *args)
{
    struct endstop *e = oid_lookup(args[0], command_config_endstop);
    uint8_t raw = gpio_in_read(e->pin) ? 1 : 0;
    sendf("endstop_state oid=%c armed=%c pin_value=%c", args[0], e->armed, raw);
}
DECL_COMMAND(command_endstop_query_state, "endstop_query_state oid=%c");
