// Handling of stepper drivers.
//
// Copyright (C) 2016-2025  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_*
#include "basecmd.h" // oid_alloc
#include "board/gpio.h" // gpio_out_write
#include "board/irq.h" // irq_disable / irq_enable
#include "board/misc.h" // timer_from_us, timer_read_time, timer_is_before
#include "command.h" // DECL_COMMAND, command_decode_ptr
#include "sched.h" // DECL_SHUTDOWN
#include "trsync.h" // trsync_add_signal
#include "kalico_runtime.h" // StepperBindingRust
#include "kalico_log.h" // kalico_log_emit (mcu structured-log ready marker)
#include "generic/fault_handler.h" // kalico_diag_emit_prior_crash (Stage 5)

struct stepper {
    struct gpio_out step_pin, dir_pin;
    struct trsync_signal stop_signal;
};

volatile uint32_t config_stepper_oids_seen
    __attribute__((used, externally_visible));

void
command_config_stepper(uint32_t *args)
{
    {
        uint8_t oid = args[0] & 0xFFu;
        if (oid < 32)
            config_stepper_oids_seen |= (1u << oid);
    }
    {
        extern void runtime_diag_progress(uint32_t tag, uint32_t stage,
                                          uint32_t value);
        uint32_t exp_lo = (uint32_t)((uintptr_t)command_config_stepper & 0xFFu);
        runtime_diag_progress(0xCD, args[0] & 0xFFu, exp_lo);
    }
    struct stepper *s = oid_alloc(args[0], command_config_stepper, sizeof(*s));
    s->step_pin = gpio_out_setup(args[1], 0);
    s->dir_pin = gpio_out_setup(args[2], 0);
}
DECL_COMMAND(command_config_stepper, "config_stepper oid=%c step_pin=%c"
             " dir_pin=%c invert_step=%c step_pulse_ticks=%u");

static struct stepper *
stepper_oid_lookup(uint8_t oid)
{
    return oid_lookup(oid, command_config_stepper);
}

static void
stepper_stop(struct trsync_signal *tss, uint8_t reason)
{
    struct stepper *s = container_of(tss, struct stepper, stop_signal);
    gpio_out_write(s->dir_pin, 0);
    gpio_out_write(s->step_pin, 0);
}

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
command_diag_stepper_buzz(uint32_t *args)
{
    uint8_t oid = args[0] & 0xFFu;
    uint8_t dir = args[1] & 0x01u;
    uint32_t step_count = args[2];
    uint32_t period_ticks = args[3];
    if (step_count > 2000) step_count = 2000;
    uint32_t min_period = timer_from_us(100);
    if (period_ticks < min_period) period_ticks = min_period;

    struct stepper *s = stepper_oid_lookup(oid);
    gpio_out_write(s->dir_pin, dir);

    // Settle direction before first step edge (TMC datasheet typically
    // requires >= 20 ns; 1 µs is generous).
    uint32_t settle_deadline = timer_read_time() + timer_from_us(1);
    while (timer_is_before(timer_read_time(), settle_deadline))
        ;

    uint32_t deadline = timer_read_time();
    for (uint32_t i = 0; i < step_count; i++) {
        gpio_out_toggle(s->step_pin);
        deadline += period_ticks;
        while (timer_is_before(timer_read_time(), deadline))
            ;
    }

    sendf("diag_stepper_buzz_response oid=%c step_count=%u",
          oid, step_count);
}
DECL_COMMAND(command_diag_stepper_buzz,
             "diag_stepper_buzz oid=%c dir=%c step_count=%u"
             " period_ticks=%u");

void
stepper_shutdown(void)
{
    uint8_t i;
    struct stepper *s;
    foreach_oid(i, s, command_config_stepper) {
        stepper_stop(&s->stop_signal, 0);
    }
}
DECL_SHUTDOWN(stepper_shutdown);

#define RUNTIME_MOTOR_COUNT 4
#define RUNTIME_MAX_STEPPERS_PER_MOTOR 4

struct runtime_motor_stepper {
    struct stepper *stepper;
    uint8_t invert_dir;
};

static struct runtime_motor_stepper runtime_motor_steppers[RUNTIME_MOTOR_COUNT]
                                                          [RUNTIME_MAX_STEPPERS_PER_MOTOR];
static uint8_t runtime_motor_stepper_count[RUNTIME_MOTOR_COUNT];
static int8_t runtime_motor_last_dir[RUNTIME_MOTOR_COUNT] = { -1, -1, -1, -1 };

volatile uint32_t runtime_emit_calls __attribute__((used, externally_visible));
volatile uint32_t runtime_emit_pulses __attribute__((used, externally_visible));

__attribute__((used, externally_visible))
uint8_t
runtime_motor_binding_count(uint8_t motor_idx)
{
    if (motor_idx >= RUNTIME_MOTOR_COUNT) return 0;
    return runtime_motor_stepper_count[motor_idx];
}

extern void *runtime_handle;

void
command_kalico_configure_axis(uint32_t *args)
{
    uint8_t axis_idx        = args[0];
    uint8_t mode            = args[1];
    uint32_t mstep_bits     = args[2];
    uint32_t extrusion_bits = args[3];
    uint8_t stepper_count   = args[4];
    uint16_t ring_depth     = (uint16_t)args[5];
    uint16_t blob_len       = (uint16_t)args[6];
    const uint8_t *blob     = command_decode_ptr(args[7]);

    if (axis_idx >= RUNTIME_MOTOR_COUNT)
        shutdown("configure_axis axis_idx out of range");
    if (mode > 1)
        shutdown("configure_axis mode invalid");
    if (stepper_count > RUNTIME_MAX_STEPPERS_PER_MOTOR)
        shutdown("configure_axis too many steppers per axis");
    if (blob_len != (uint16_t)stepper_count * 4)
        shutdown("configure_axis blob length mismatch");
    if (ring_depth == 0)
        shutdown("configure_axis ring_depth must be nonzero");
    if (!runtime_handle)
        shutdown("configure_axis before runtime init");

    struct {
        struct stepper *stepper;
        uint8_t invert_dir;
        uint8_t tmc_cs_oid;
    } staged[RUNTIME_MAX_STEPPERS_PER_MOTOR] = {{0}};

    extern void *command_config_spi(uint32_t *);
    for (uint8_t i = 0; i < stepper_count; i++) {
        uint8_t stepper_oid = blob[i*4 + 0];
        uint8_t dir_invert  = blob[i*4 + 1];
        uint8_t tmc_cs_oid  = blob[i*4 + 2];
        uint8_t flags       = blob[i*4 + 3];
        if (flags != 0)
            shutdown("configure_axis reserved stepper flags must be zero");
        if (dir_invert > 1)
            shutdown("configure_axis dir_invert must be 0 or 1");
        struct stepper *s = oid_lookup(stepper_oid, command_config_stepper);
        if (tmc_cs_oid != 0xFF) {
            (void)oid_lookup(tmc_cs_oid, command_config_spi);
        }
        staged[i].stepper = s;
        staged[i].invert_dir = dir_invert;
        staged[i].tmc_cs_oid = tmc_cs_oid;
    }

    struct StepperBindingRust bindings[RUNTIME_MAX_STEPPERS_PER_MOTOR];
    for (uint8_t i = 0; i < stepper_count; i++) {
        bindings[i].stepper_oid = blob[i*4 + 0];
        bindings[i].tmc_cs_oid = staged[i].tmc_cs_oid;
        bindings[i]._pad[0] = 0;
        bindings[i]._pad[1] = 0;
    }
    int32_t rc = kalico_runtime_configure_axis(
        runtime_handle, axis_idx, mode, mstep_bits,
        ring_depth,
        stepper_count > 0 ? bindings : 0,
        stepper_count);
    if (rc != 0)
        shutdown("configure_axis rejected by runtime");

    runtime_motor_stepper_count[axis_idx] = stepper_count;
    for (uint8_t i = 0; i < stepper_count; i++) {
        runtime_motor_steppers[axis_idx][i].stepper = staged[i].stepper;
        runtime_motor_steppers[axis_idx][i].invert_dir = staged[i].invert_dir;
    }
    runtime_motor_last_dir[axis_idx] = -1;
    (void)extrusion_bits;

    extern void arm_per_axis_step_timer(uint8_t axis_idx);
    arm_per_axis_step_timer(axis_idx);

    extern void runtime_tick_enable(void);
    runtime_tick_enable();

    // Emit only after the first configure_axis: the config phase runs after the
    // host's identify/attach handshake installs the mcu-log hook. Emitting at
    // MCU boot / first drain races ahead of the host connecting; the frame is
    // lost.
    static uint8_t kalico_log_ready_emitted;
    if (!kalico_log_ready_emitted) {
        kalico_log_ready_emitted = 1;
        kalico_log_emit(KALICO_LOG_LEVEL_DEBUG, KALICO_LOG_SUBSYS_RUNTIME,
                        KALICO_LOG_EVENT_RUNTIME_MCU_READY, 0, 0, 0);
        kalico_diag_emit_prior_crash();
    }
}
DECL_COMMAND(command_kalico_configure_axis,
             "kalico_configure_axis axis_idx=%c mode=%c microstep_distance=%u"
             " extrusion_per_xy_mm=%u stepper_count=%c ring_depth=%hu"
             " steppers=%*s");

void
command_kalico_runtime_reset(uint32_t *args)
{
    (void)args;
    if (!runtime_handle)
        shutdown("runtime reset before runtime init");
    irqstatus_t flag = irq_save();
    int32_t rc = kalico_runtime_reset(runtime_handle);
    irq_restore(flag);
    if (rc != 0)
        shutdown("runtime reset rejected");
}
DECL_COMMAND(command_kalico_runtime_reset, "kalico_runtime_reset");

void
command_kalico_diag_dump(uint32_t *args)
{
    (void)args;
    kalico_diag_emit_live();
}
DECL_COMMAND(command_kalico_diag_dump, "kalico_diag_dump");

void
command_kalico_phase_stepping_enable_spi(uint32_t *args)
{
    (void)args;
    extern void phase_stepping_enable_writes(void);
    phase_stepping_enable_writes();
}
DECL_COMMAND(command_kalico_phase_stepping_enable_spi,
             "kalico_phase_stepping_enable_spi");

void
command_kalico_phase_stepping_disable_spi(uint32_t *args)
{
    (void)args;
    extern void phase_stepping_disable_writes(void);
    phase_stepping_disable_writes();
}
DECL_COMMAND(command_kalico_phase_stepping_disable_spi,
             "kalico_phase_stepping_disable_spi");

void
command_kalico_set_axis_mode(uint32_t *args)
{
    if (!runtime_handle)
        shutdown("kalico_set_axis_mode before runtime init");
    uint8_t axis_idx = args[0];
    uint8_t mode = args[1];
    int32_t rc = kalico_runtime_set_axis_mode(runtime_handle, axis_idx, mode);
    if (rc != 0)
        shutdown("kalico_set_axis_mode rejected (motion in progress or bad arg)");
}
DECL_COMMAND(command_kalico_set_axis_mode,
             "kalico_set_axis_mode axis_idx=%c mode=%c");

void
command_kalico_set_stepper_offset(uint32_t *args)
{
    if (!runtime_handle)
        shutdown("kalico_set_stepper_offset before runtime init");
    uint8_t stepper_idx = args[0];
    int32_t delta = (int32_t)args[1];
    uint16_t max_per_sample = args[2];
    int32_t rc = kalico_runtime_set_stepper_offset(
        runtime_handle, stepper_idx, delta, max_per_sample);
    if (rc != 0)
        shutdown("kalico_set_stepper_offset rejected (bad parameters)");
}
DECL_COMMAND(command_kalico_set_stepper_offset,
             "kalico_set_stepper_offset stepper_idx=%c delta_microsteps=%i"
             " max_microsteps_per_sample=%hu");

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

    if (runtime_motor_last_dir[motor_idx] != want_dir) {
        // pin_level must be (!want_dir XOR invert_dir): bench-verified that
        // the simpler-looking (want_dir XOR invert_dir) drives reverse motion.
        for (uint8_t j = 0; j < cnt; j++) {
            uint8_t pin_level = (uint8_t)(!want_dir)
                              ^ runtime_motor_steppers[motor_idx][j].invert_dir;
            gpio_out_write(runtime_motor_steppers[motor_idx][j].stepper->dir_pin,
                           pin_level);
        }
        runtime_motor_last_dir[motor_idx] = want_dir;
    }

    for (uint32_t i = 0; i < count; i++) {
        for (uint8_t j = 0; j < cnt; j++)
            gpio_out_toggle_noirq(runtime_motor_steppers[motor_idx][j].stepper->step_pin);
    }
}
