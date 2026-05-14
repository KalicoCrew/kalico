// Handling of stepper drivers.
//
// Copyright (C) 2016-2025  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_*
#include "basecmd.h" // oid_alloc
#include "board/gpio.h" // gpio_out_write
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_is_before
#include "command.h" // DECL_COMMAND
#include "sched.h" // struct timer
#include "stepper.h" // stepper_event
#include "trsync.h" // trsync_add_signal

DECL_CONSTANT("STEPPER_STEP_BOTH_EDGE", 1);

#if CONFIG_INLINE_STEPPER_HACK && CONFIG_WANT_STEPPER_OPTIMIZED_BOTH_EDGE
 #define HAVE_OPTIMIZED_PATH 1
 #define HAVE_EDGE_OPTIMIZATION 1
 #define HAVE_AVR_OPTIMIZATION 0
#elif CONFIG_INLINE_STEPPER_HACK && CONFIG_MACH_AVR
 #define HAVE_OPTIMIZED_PATH 1
 #define HAVE_EDGE_OPTIMIZATION 0
 #define HAVE_AVR_OPTIMIZATION 1
#else
 #define HAVE_OPTIMIZED_PATH 0
 #define HAVE_EDGE_OPTIMIZATION 0
 #define HAVE_AVR_OPTIMIZATION 0
#endif

struct stepper_move {
    struct move_node node;
    uint32_t interval;
    int16_t add;
    uint16_t count;
    uint8_t flags;
};

enum { MF_DIR=1<<0 };

struct stepper {
    struct timer time;
    uint32_t interval;
    int16_t add;
    uint32_t count;
    uint32_t next_step_time, step_pulse_ticks;
    struct gpio_out step_pin, dir_pin;
    uint32_t position;
    struct move_queue_head mq;
    struct trsync_signal stop_signal;
    // gcc (pre v6) does better optimization when uint8_t are bitfields
    uint8_t flags : 8;
};

enum { POSITION_BIAS=0x40000000 };

enum {
    SF_LAST_DIR=1<<0, SF_NEXT_DIR=1<<1, SF_INVERT_STEP=1<<2, SF_NEED_RESET=1<<3,
    SF_SINGLE_SCHED=1<<4, SF_OPTIMIZED_PATH=1<<5, SF_HAVE_ADD=1<<6
};

// Setup a stepper for the next move in its queue
static uint_fast8_t
stepper_load_next(struct stepper *s)
{
    if (move_queue_empty(&s->mq)) {
        // There is no next move - the queue is empty
        s->count = 0;
        return SF_DONE;
    }

    // Read next 'struct stepper_move'
    struct move_node *mn = move_queue_pop(&s->mq);
    struct stepper_move *m = container_of(mn, struct stepper_move, node);
    uint32_t move_interval = m->interval;
    uint_fast16_t move_count = m->count;
    int_fast16_t move_add = m->add;
    uint_fast8_t need_dir_change = m->flags & MF_DIR;
    move_free(m);

    // Add all steps to s->position (stepper_get_position() can calc mid-move)
    s->position = (need_dir_change ? -s->position : s->position) + move_count;

    // Load next move into 'struct stepper'
    s->add = move_add;
    s->interval = move_interval + move_add;
    if (HAVE_OPTIMIZED_PATH && s->flags & SF_OPTIMIZED_PATH) {
        // Using optimized stepper_event_edge() or stepper_event_avr()
        s->time.waketime += move_interval;
        if (HAVE_AVR_OPTIMIZATION)
            s->flags = (move_add ? s->flags | SF_HAVE_ADD
                        : s->flags & ~SF_HAVE_ADD);
        s->count = move_count;
    } else {
        // Using fully scheduled stepper_event_full() code (the scheduler
        // may be called twice for each step)
        uint_fast8_t was_active = !!s->count;
        uint32_t min_next_time = s->time.waketime;
        s->next_step_time += move_interval;
        s->time.waketime = s->next_step_time;
        s->count = (s->flags & SF_SINGLE_SCHED ? move_count
                    : (uint32_t)move_count * 2);
        if (was_active && timer_is_before(s->next_step_time, min_next_time)) {
            // Actively stepping and next step event close to the last unstep
            int32_t diff = s->next_step_time - min_next_time;
            if (diff < (int32_t)-timer_from_us(1000))
                shutdown("Stepper too far in past");
            s->time.waketime = min_next_time;
        }
        if (was_active && need_dir_change) {
            // Must ensure minimum time between step change and dir change
            if (s->flags & SF_SINGLE_SCHED)
                while (timer_is_before(timer_read_time(), min_next_time))
                    ;
            gpio_out_toggle_noirq(s->dir_pin);
            uint32_t curtime = timer_read_time();
            min_next_time = curtime + s->step_pulse_ticks;
            if (timer_is_before(s->time.waketime, min_next_time))
                s->time.waketime = min_next_time;
            return SF_RESCHEDULE;
        }
    }

    // Set new direction (if needed)
    if (need_dir_change)
        gpio_out_toggle_noirq(s->dir_pin);
    return SF_RESCHEDULE;
}

// Edge optimization only enabled when fastest rate notably slower than 100ns
#define EDGE_STEP_TICKS DIV_ROUND_UP(CONFIG_CLOCK_FREQ, 8000000)
#if HAVE_EDGE_OPTIMIZATION
 DECL_CONSTANT("STEPPER_OPTIMIZED_EDGE", EDGE_STEP_TICKS);
#endif

// Optimized step function to step on each step pin edge
static uint_fast8_t
stepper_event_edge(struct timer *t)
{
    struct stepper *s = container_of(t, struct stepper, time);
    gpio_out_toggle_noirq(s->step_pin);
    uint32_t count = s->count - 1;
    if (likely(count)) {
        s->count = count;
        s->time.waketime += s->interval;
        s->interval += s->add;
        return SF_RESCHEDULE;
    }
    return stepper_load_next(s);
}

#define AVR_STEP_TICKS 40 // minimum instructions between step gpio pulses
#if HAVE_AVR_OPTIMIZATION
 DECL_CONSTANT("STEPPER_OPTIMIZED_UNSTEP", AVR_STEP_TICKS);
#endif

// AVR optimized step function
static uint_fast8_t
stepper_event_avr(struct timer *t)
{
    struct stepper *s = container_of(t, struct stepper, time);
    gpio_out_toggle_noirq(s->step_pin);
    uint16_t *pcount = (void*)&s->count, count = *pcount - 1;
    if (likely(count)) {
        *pcount = count;
        s->time.waketime += s->interval;
        gpio_out_toggle_noirq(s->step_pin);
        if (s->flags & SF_HAVE_ADD)
            s->interval += s->add;
        return SF_RESCHEDULE;
    }
    uint_fast8_t ret = stepper_load_next(s);
    gpio_out_toggle_noirq(s->step_pin);
    return ret;
}

// Regular "fully scheduled" step function
static uint_fast8_t
stepper_event_full(struct timer *t)
{
    struct stepper *s = container_of(t, struct stepper, time);
    gpio_out_toggle_noirq(s->step_pin);
    uint32_t curtime = timer_read_time();
    uint32_t min_next_time = curtime + s->step_pulse_ticks;
    uint32_t count = s->count - 1;
    if (likely(count & 1 && !(s->flags & SF_SINGLE_SCHED)))
        // Schedule unstep event
        goto reschedule_min;
    if (likely(count)) {
        s->next_step_time += s->interval;
        s->interval += s->add;
        if (unlikely(timer_is_before(s->next_step_time, min_next_time)))
            // The next step event is too close - push it back
            goto reschedule_min;
        s->count = count;
        s->time.waketime = s->next_step_time;
        return SF_RESCHEDULE;
    }
    s->time.waketime = min_next_time;
    return stepper_load_next(s);
reschedule_min:
    s->count = count;
    s->time.waketime = min_next_time;
    return SF_RESCHEDULE;
}

// Optimized entry point for step function (may be inlined into sched.c code)
uint_fast8_t
stepper_event(struct timer *t)
{
    if (HAVE_EDGE_OPTIMIZATION)
        return stepper_event_edge(t);
    if (HAVE_AVR_OPTIMIZATION)
        return stepper_event_avr(t);
    return stepper_event_full(t);
}

void
command_config_stepper(uint32_t *args)
{
    struct stepper *s = oid_alloc(args[0], command_config_stepper, sizeof(*s));
    int_fast8_t invert_step = args[3];
    if (invert_step > 0)
        s->flags = SF_INVERT_STEP;
    else if (invert_step < 0)
        s->flags = SF_SINGLE_SCHED;
    s->step_pin = gpio_out_setup(args[1], s->flags & SF_INVERT_STEP);
    s->dir_pin = gpio_out_setup(args[2], 0);
    s->position = -POSITION_BIAS;
    s->step_pulse_ticks = args[4];
    move_queue_setup(&s->mq, sizeof(struct stepper_move));
    if (HAVE_EDGE_OPTIMIZATION) {
        if (invert_step < 0 && s->step_pulse_ticks <= EDGE_STEP_TICKS)
            s->flags |= SF_OPTIMIZED_PATH;
        else
            s->time.func = stepper_event_full;
    } else if (HAVE_AVR_OPTIMIZATION) {
        if (invert_step >= 0 && s->step_pulse_ticks <= AVR_STEP_TICKS)
            s->flags |= SF_SINGLE_SCHED | SF_OPTIMIZED_PATH;
        else
            s->time.func = stepper_event_full;
    } else if (!CONFIG_INLINE_STEPPER_HACK) {
        s->time.func = stepper_event_full;
    }
}
DECL_COMMAND(command_config_stepper, "config_stepper oid=%c step_pin=%c"
             " dir_pin=%c invert_step=%c step_pulse_ticks=%u");

// Return the 'struct stepper' for a given stepper oid
static struct stepper *
stepper_oid_lookup(uint8_t oid)
{
    return oid_lookup(oid, command_config_stepper);
}

// Schedule a set of steps with a given timing
void
command_queue_step(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    struct stepper_move *m = move_alloc();
    m->interval = args[1];
    m->count = args[2];
    if (!m->count)
        shutdown("Invalid count parameter");
    m->add = args[3];
    m->flags = 0;

    irq_disable();
    uint8_t flags = s->flags;
    if (!!(flags & SF_LAST_DIR) != !!(flags & SF_NEXT_DIR)) {
        flags ^= SF_LAST_DIR;
        m->flags |= MF_DIR;
    }
    if (s->count) {
        s->flags = flags;
        move_queue_push(&m->node, &s->mq);
    } else if (flags & SF_NEED_RESET) {
        move_free(m);
    } else {
        s->flags = flags;
        move_queue_push(&m->node, &s->mq);
        stepper_load_next(s);
        sched_add_timer(&s->time);
    }
    irq_enable();
}
DECL_COMMAND(command_queue_step,
             "queue_step oid=%c interval=%u count=%hu add=%hi");

// Set the direction of the next queued step
void
command_set_next_step_dir(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    uint8_t nextdir = args[1] ? SF_NEXT_DIR : 0;
    irq_disable();
    s->flags = (s->flags & ~SF_NEXT_DIR) | nextdir;
    irq_enable();
}
DECL_COMMAND(command_set_next_step_dir, "set_next_step_dir oid=%c dir=%c");

// Set an absolute time that the next step will be relative to
void
command_reset_step_clock(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    uint32_t waketime = args[1];
    irq_disable();
    if (s->count)
        shutdown("Can't reset time when stepper active");
    s->next_step_time = s->time.waketime = waketime;
    s->flags &= ~SF_NEED_RESET;
    irq_enable();
}
DECL_COMMAND(command_reset_step_clock, "reset_step_clock oid=%c clock=%u");

// Return the current stepper position.  Caller must disable irqs.
static uint32_t
stepper_get_position(struct stepper *s)
{
    uint32_t position = s->position;
    // If stepper is mid-move, subtract out steps not yet taken
    if (s->flags & SF_SINGLE_SCHED)
        position -= s->count;
    else
        position -= s->count / 2;
    // The top bit of s->position is an optimized reverse direction flag
    if (position & 0x80000000)
        return -position;
    return position;
}

// Report the current position of the stepper
void
command_stepper_get_position(uint32_t *args)
{
    uint8_t oid = args[0];
    struct stepper *s = stepper_oid_lookup(oid);
    irq_disable();
    uint32_t position = stepper_get_position(s);
    irq_enable();
    sendf("stepper_position oid=%c pos=%i", oid, position - POSITION_BIAS);
}
DECL_COMMAND(command_stepper_get_position, "stepper_get_position oid=%c");

// Stop all moves for a given stepper (caller must disable IRQs)
static void
stepper_stop(struct trsync_signal *tss, uint8_t reason)
{
    struct stepper *s = container_of(tss, struct stepper, stop_signal);
    sched_del_timer(&s->time);
    s->next_step_time = s->time.waketime = 0;
    s->position = -stepper_get_position(s);
    s->count = 0;
    s->flags = ((s->flags & (SF_INVERT_STEP|SF_SINGLE_SCHED|SF_OPTIMIZED_PATH))
                | SF_NEED_RESET);
    gpio_out_write(s->dir_pin, 0);
    if (!(s->flags & SF_SINGLE_SCHED)
        || (HAVE_AVR_OPTIMIZATION && s->flags & SF_OPTIMIZED_PATH))
        // Must return step pin to "unstep" state
        gpio_out_write(s->step_pin, s->flags & SF_INVERT_STEP);
    while (!move_queue_empty(&s->mq)) {
        struct move_node *mn = move_queue_pop(&s->mq);
        struct stepper_move *m = container_of(mn, struct stepper_move, node);
        move_free(m);
    }
}

// Set the stepper to stop on a "trigger event" (used in homing)
void
command_stepper_stop_on_trigger(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    struct trsync *ts = trsync_oid_lookup(args[1]);
    trsync_add_signal(ts, &s->stop_signal, stepper_stop);
}
DECL_COMMAND(command_stepper_stop_on_trigger,
             "stepper_stop_on_trigger oid=%c trsync_oid=%c");

void
stepper_shutdown(void)
{
    uint8_t i;
    struct stepper *s;
    foreach_oid(i, s, command_config_stepper) {
        move_queue_clear(&s->mq);
        stepper_stop(&s->stop_signal, 0);
    }
}
DECL_SHUTDOWN(stepper_shutdown);

// ---------------------------------------------------------------------------
// Runtime-engine step pulse emission (Step 7-D first-light).
//
// The Rust runtime evaluates the trajectory inside the TIM5 ISR (40 kHz on
// H7) and produces a signed integer step delta per motor per tick. This file
// owns the GPIO toggle path: a small lookup table maps runtime motor index
// (0..3, post-kinematic-transform) to the existing klipper-protocol
// `struct stepper` already configured by `command_config_stepper` from
// printer.cfg, and `runtime_emit_step_pulses` does the actual pin toggles.
//
// The legacy `command_queue_step` / `stepper_event_*` scheduler is unused
// in the runtime path — the Rust engine emits steps directly per ISR fire,
// not by queueing future timer events.
// ---------------------------------------------------------------------------

#if CONFIG_KALICO_RUNTIME

#define RUNTIME_MOTOR_COUNT 4
// Max steppers physically driven by a single runtime motor index. CoreXY-
// with-twin-gantry topologies (e.g. Voron 2.4) need 2 per axis pair.
// Z-axis with 3 lifters needs 3. Keep some headroom; 4 covers anything
// reasonable without blowing memory.
#define RUNTIME_MAX_STEPPERS_PER_MOTOR 4

struct runtime_motor_stepper {
    struct stepper *stepper;
    uint8_t invert_dir; // XOR'd with the sign-of-n_steps direction at emit
};

static struct runtime_motor_stepper runtime_motor_steppers[RUNTIME_MOTOR_COUNT]
                                                          [RUNTIME_MAX_STEPPERS_PER_MOTOR];
static uint8_t runtime_motor_stepper_count[RUNTIME_MOTOR_COUNT];
// Last-emitted direction per motor: 0 = forward, 1 = reverse, -1 = unknown
// (forces a dir-pin write on the next non-zero pulse so the bench gets a
// known direction even before the first reversal).
static int8_t runtime_motor_last_dir[RUNTIME_MOTOR_COUNT] = { -1, -1, -1, -1 };

// Step 7-D bring-up diagnostic counters. `runtime_emit_calls` increments
// once per call to `runtime_emit_step_pulses`, regardless of n_steps;
// `runtime_emit_pulses` adds |n_steps|. Read by `runtime_status_drain` on
// each engine state transition so we can tell whether (a) my emit path
// is being invoked at all and (b) whether it's seeing non-zero step
// deltas. Volatile because TIM5 ISR writes; foreground reads.
volatile uint32_t runtime_emit_calls __attribute__((used, externally_visible));
volatile uint32_t runtime_emit_pulses __attribute__((used, externally_visible));

// Called from kalico_runtime_configure_axes_blob (Rust FFI) at the start
// of every klippy session. Clears the runtime motor->stepper binding
// table so the subsequent stream of config_runtime_stepper commands
// populates a fresh slate. Without this, the table accumulates across
// klippy restarts (MCU stays powered, klippy reconnects) and motor 0 /
// motor 1 hit RUNTIME_MAX_STEPPERS_PER_MOTOR=4 after two reconnects —
// the third reconnect shutdowns with "too many steppers per motor".
// Bench-observed 2026-05-11 after F446 KALICO_RUNTIME enablement, when
// klippy began re-running config_runtime_stepper for both H7 and F446
// each session.
// Foreground accessor — surfaces per-motor binding count for the
// runtime_status_drain diag rotation (runtime_tick.c phase 5).
__attribute__((used, externally_visible))
uint8_t
runtime_motor_binding_count(uint8_t motor_idx)
{
    if (motor_idx >= RUNTIME_MOTOR_COUNT) return 0;
    return runtime_motor_stepper_count[motor_idx];
}

// 2026-05-14 binding-bug investigation diagnostic counters. Exposed via
// fault_detail tags 0xB0..0xB3 so they survive the USB-CDC transmit_buf
// overflow that ate the previous output()-based attempt log. Each counter
// is incremented unconditionally at the very top of its callsite — so
// `runtime_bind_calls_total` equals the number of `config_runtime_stepper`
// commands that were dispatched by Klipper's command parser (it has
// nothing to do with whether the binding got added afterwards), and
// `runtime_bind_calls_for_motor[i]` is the per-motor-idx breakdown.
// If `runtime_bind_calls_total` < 5 at steady state on the H7 the
// command never even reached the dispatcher; if it's = 5 but
// `runtime_bind_calls_for_motor[1] < 2` the command reached the
// dispatcher but the parsed `motor_idx` doesn't match what Klipper
// thought it sent.
volatile uint32_t runtime_bind_calls_total __attribute__((used, externally_visible));
volatile uint8_t runtime_bind_calls_for_motor[RUNTIME_MOTOR_COUNT]
                __attribute__((used, externally_visible));
volatile uint32_t runtime_bind_reset_calls __attribute__((used, externally_visible));

__attribute__((used, externally_visible))
void
runtime_reset_stepper_bindings(void)
{
    runtime_bind_reset_calls++;
    for (uint8_t m = 0; m < RUNTIME_MOTOR_COUNT; m++) {
        runtime_motor_stepper_count[m] = 0;
        runtime_motor_last_dir[m] = -1;
        runtime_bind_calls_for_motor[m] = 0;
    }
}

void
command_config_runtime_stepper(uint32_t *args)
{
    runtime_bind_calls_total++;
    uint8_t motor_idx = args[0];
    uint8_t stepper_oid = args[1];
    uint8_t invert_dir = args[2];
    if (motor_idx < RUNTIME_MOTOR_COUNT) {
        runtime_bind_calls_for_motor[motor_idx]++;
    }
    if (motor_idx >= RUNTIME_MOTOR_COUNT)
        shutdown("config_runtime_stepper motor_idx out of range");
    uint8_t cnt = runtime_motor_stepper_count[motor_idx];
    if (cnt >= RUNTIME_MAX_STEPPERS_PER_MOTOR)
        shutdown("config_runtime_stepper too many steppers per motor");
    // oid_lookup walks the same allocation table populated by
    // command_config_stepper, so this fails (shutdowns) cleanly if the
    // referenced stepper hasn't been configured yet.
    struct stepper *s = oid_lookup(stepper_oid, command_config_stepper);
    runtime_motor_steppers[motor_idx][cnt].stepper = s;
    runtime_motor_steppers[motor_idx][cnt].invert_dir = invert_dir ? 1 : 0;
    runtime_motor_stepper_count[motor_idx] = cnt + 1;
    runtime_motor_last_dir[motor_idx] = -1;
}
DECL_COMMAND(command_config_runtime_stepper,
             "config_runtime_stepper motor_idx=%c stepper_oid=%c invert_dir=%c");

// Busy-wait until DWT->CYCCNT has advanced `ticks` cycles past `start`.
// Unsigned subtraction handles the u32 wrap correctly (~8.3 s at 520 MHz,
// far longer than any plausible pulse width).
static inline void
runtime_dwt_dwell(uint32_t start, uint32_t ticks)
{
    while ((timer_read_time() - start) < ticks)
        ;
}

// Called from the TIM5 ISR (priority 3 on H7) after `runtime_handle_tick`
// produces this tick's step delta. Emits |n_steps| pulses on every stepper
// bound to this motor index (primary + AWD partners — e.g. Voron 2.4-style
// 4-motor gantry binds stepper_x and stepper_x1 both to motor 0). All
// step_pins toggle in lockstep; each dir_pin is set to the per-stepper
// `want_dir XOR invert_dir` so that printer.cfg `dir_pin: !PIN` polarity
// is honored end-to-end.
//
// Pulse timing: most TMC drivers require ≥100 ns step high and ≥20 ns dir
// setup; klippy's default `step_pulse_duration` is 2 µs (~1040 cycles at
// 520 MHz), which is comfortably above both. The same budget is reused for
// dir-setup dwell — overkill for TMCs, but matches the user-configured
// driver-side timing without a second knob.
//
// Burst budget: the runtime's `MAX_STEPS_PER_TICK_DEFAULT = 16` cap bounds
// the worst case. At 2 µs pulse_ticks with 2 AWD partners per motor that
// is 16 × 4 µs = 64 µs of ISR time — exceeds a 25 µs tick. The cap exists
// for runaway-curve fault detection, not to rate-limit normal operation;
// at peak velocity (300 mm/s × 80 steps/mm = 24 kHz) average is 0.6
// step/tick. A genuine burst past 4-5 steps will eat the next tick's
// budget; the engine's own runaway detection latches a fault before this
// can sustain.
__attribute__((used, externally_visible))
void
runtime_emit_step_pulses(uint8_t motor_idx, int32_t n_steps)
{
    runtime_emit_calls++;
    if (motor_idx >= RUNTIME_MOTOR_COUNT)
        return;
    uint8_t cnt = runtime_motor_stepper_count[motor_idx];
    if (cnt == 0)
        return;
    if (n_steps == 0)
        return;
    runtime_emit_pulses += (n_steps < 0) ? (uint32_t)-n_steps : (uint32_t)n_steps;

    int8_t want_dir = (n_steps < 0) ? 1 : 0;
    uint32_t count = (n_steps < 0) ? (uint32_t)-n_steps : (uint32_t)n_steps;
    // All AWD partners share the same step_pulse_ticks at the printer.cfg
    // level (default 2 µs); we read it from the primary for simplicity.
    uint32_t pulse_ticks = runtime_motor_steppers[motor_idx][0].stepper
                                                              ->step_pulse_ticks;

    if (runtime_motor_last_dir[motor_idx] != want_dir) {
        // Drive each AWD partner's dir_pin so that printer.cfg
        // `dir_pin: !PIN` produces motion in the direction klippy
        // commanded. Empirically (verified on the test bench):
        //   pin_level = !want_dir XOR invert_dir
        // gives the right direction. The simpler-looking
        // `want_dir XOR invert_dir` produced reverse motion.
        for (uint8_t j = 0; j < cnt; j++) {
            uint8_t pin_level = (uint8_t)(!want_dir)
                              ^ runtime_motor_steppers[motor_idx][j].invert_dir;
            gpio_out_write(runtime_motor_steppers[motor_idx][j].stepper->dir_pin,
                           pin_level);
        }
        runtime_motor_last_dir[motor_idx] = want_dir;
        // Dir-setup dwell. Driver datasheet wants ≥20 ns; we burn the full
        // step_pulse_ticks for simplicity.
        uint32_t t = timer_read_time();
        runtime_dwt_dwell(t, pulse_ticks);
    }

    // Toggle-based pulse emission. The step pin's idle level is tracked by
    // the GPIO layer; toggling lifts it to "active" then back to "idle".
    // gpio_out_toggle_noirq is the irq-safe variant — caller (us) is in
    // ISR context with IRQs off by virtue of the priority-3 TIM5 vector.
    for (uint32_t i = 0; i < count; i++) {
        // Rising edge on every bound stepper's step pin in lockstep.
        for (uint8_t j = 0; j < cnt; j++)
            gpio_out_toggle_noirq(runtime_motor_steppers[motor_idx][j].stepper->step_pin);
        uint32_t t0 = timer_read_time();
        runtime_dwt_dwell(t0, pulse_ticks);
        // Falling edge.
        for (uint8_t j = 0; j < cnt; j++)
            gpio_out_toggle_noirq(runtime_motor_steppers[motor_idx][j].stepper->step_pin);
        uint32_t t1 = timer_read_time();
        runtime_dwt_dwell(t1, pulse_ticks);
    }
}


#endif // CONFIG_KALICO_RUNTIME
