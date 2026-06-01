// Handling of stepper drivers.
//
// Copyright (C) 2016-2025  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// The Rust runtime is the sole producer of step pulses on this fork
// (Layer 4 motion planner, 40 kHz TIM5 tick on H7, 10 kHz on F4). The
// legacy klipper-protocol queue_step / set_next_step_dir / stepper_event
// scheduling path is gone; this file is a thin host→MCU stepper-binding
// layer plus the runtime's GPIO emission helper.

#include "autoconf.h" // CONFIG_*
#include "basecmd.h" // oid_alloc
#include "board/gpio.h" // gpio_out_write
#include "board/irq.h" // irq_disable / irq_enable
#include "board/misc.h" // timer_from_us, timer_read_time, timer_is_before
#include "command.h" // DECL_COMMAND, command_decode_ptr
#include "sched.h" // DECL_SHUTDOWN
#include "stepper.h"
#include "trsync.h" // trsync_add_signal
#include "kalico_runtime.h" // StepperBindingRust

struct stepper {
    struct gpio_out step_pin, dir_pin;
    struct trsync_signal stop_signal;
};

// Durable monotonic bitmap — bit N set when command_config_stepper(oid=N)
// has been entered. Survives runtime_diag_progress overwrites; readable via
// the status-drain rotation tag 0x9D.
volatile uint32_t config_stepper_oids_seen
    __attribute__((used, externally_visible));

void
command_config_stepper(uint32_t *args)
{
    // Latch oid into the durable bitmap BEFORE oid_alloc — so if oid_alloc
    // shutdowns (out of range / already allocated / finalized), we still
    // know command_config_stepper was entered for this oid.
    {
        uint8_t oid = args[0] & 0xFFu;
        if (oid < 32)
            config_stepper_oids_seen |= (1u << oid);
    }
    // Also fire the prior tag 0xCD probe (best-effort, may be overwritten).
    {
        extern void runtime_diag_progress(uint32_t tag, uint32_t stage,
                                          uint32_t value);
        uint32_t exp_lo = (uint32_t)((uintptr_t)command_config_stepper & 0xFFu);
        runtime_diag_progress(0xCD, args[0] & 0xFFu, exp_lo);
    }
    struct stepper *s = oid_alloc(args[0], command_config_stepper, sizeof(*s));
    // args[3] (invert_step) and args[4] (step_pulse_ticks) are accepted for
    // host-side ABI compatibility but ignored: the runtime engine drives
    // step pin polarity and pulse timing itself, not via the host config.
    s->step_pin = gpio_out_setup(args[1], 0);
    s->dir_pin = gpio_out_setup(args[2], 0);
}
DECL_COMMAND(command_config_stepper, "config_stepper oid=%c step_pin=%c"
             " dir_pin=%c invert_step=%c step_pulse_ticks=%u");

// Return the 'struct stepper' for a given stepper oid
static struct stepper *
stepper_oid_lookup(uint8_t oid)
{
    return oid_lookup(oid, command_config_stepper);
}

// Drive the stepper to a safe known state. Invoked from trsync (homing
// trigger) and from stepper_shutdown.
static void
stepper_stop(struct trsync_signal *tss, uint8_t reason)
{
    struct stepper *s = container_of(tss, struct stepper, stop_signal);
    gpio_out_write(s->dir_pin, 0);
    gpio_out_write(s->step_pin, 0);
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

// 2026-05-21 minimal-firmware control test. Bypasses the entire Rust
// motion engine: just toggles the step pin N times with a fixed period
// to verify TMC + step pin + dir pin + motor wiring all work without
// any Klipper/Rust scheduling. If this moves the motor but the engine
// path doesn't, the engine is the only suspect. If THIS doesn't move
// the motor either, the bug is below the engine (TMC config, current,
// pin routing, mechanical).
//
// Bounded to step_count<=2000 + period_ticks>=timer_from_us(100) so
// the busy-wait can't trip IWDG (worst-case ~200ms of foreground
// blocking, well under the watchdog).
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

// ---------------------------------------------------------------------------
// Runtime-engine step pulse emission (Step 7-D first-light).
//
// The Rust runtime evaluates the trajectory inside the TIM5 ISR (40 kHz on
// H7) and produces a signed integer step delta per motor per tick. This file
// owns the GPIO toggle path: a small lookup table maps runtime motor index
// (0..3, post-kinematic-transform) to the existing klipper-protocol
// `struct stepper` already configured by `command_config_stepper` from
// printer.cfg, and `runtime_emit_step_pulses` does the actual pin toggles.
// ---------------------------------------------------------------------------


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

// Foreground accessor — surfaces per-motor binding count for the
// runtime_status_drain diag rotation (runtime_tick.c phase 5).
__attribute__((used, externally_visible))
uint8_t
runtime_motor_binding_count(uint8_t motor_idx)
{
    if (motor_idx >= RUNTIME_MOTOR_COUNT) return 0;
    return runtime_motor_stepper_count[motor_idx];
}


// ─── Stepping-redesign Task 11 — kalico_configure_* command handlers ─────
//
// Three foreground commands that publish per-axis configuration, the
// kinematic scale factor, and pressure-advance coefficients into the Rust
// runtime. Each handler is a thin shim: unpack the Klipper-protocol
// `uint32_t *args`, forward to the Rust FFI, and `shutdown(...)` on any
// non-zero return so configuration errors are loud rather than silent.
//
// `runtime_handle` is the global published by `runtime_handle_create()`
// in src/runtime_tick.c. All commands here call into Rust and need the
// handle. The legacy `command_config_runtime_stepper` was deleted in
// Task 17 and replaced by the two-phase `command_kalico_configure_axis`.
//
// Wire-format note: Klipper carries f32 fields as `u32` containing
// `f32::to_bits()` (the host packs, the MCU forwards the raw bits, and
// `f32::from_bits` reconstructs on the Rust side). `%u` matches u32.

extern void *runtime_handle; // defined in src/runtime_tick.c
// kalico_runtime_configure_axis / _kinematics / _pressure_advance are
// declared in kalico_runtime.h (included above).

void
command_kalico_configure_axis(uint32_t *args)
{
    uint8_t axis_idx        = args[0];
    uint8_t mode            = args[1];
    uint32_t mstep_bits     = args[2];
    uint32_t extrusion_bits = args[3]; // reserved — not forwarded to current Rust FFI
    uint8_t stepper_count   = args[4];
    uint16_t ring_depth     = (uint16_t)args[5];
    uint16_t blob_len       = (uint16_t)args[6];
    const uint8_t *blob     = command_decode_ptr(args[7]);

    // ── Phase 1: validate every input + every binding, no mutations.
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

    // ── Phase 2: Rust FFI.
    struct StepperBindingRust bindings[RUNTIME_MAX_STEPPERS_PER_MOTOR];
    for (uint8_t i = 0; i < stepper_count; i++) {
        bindings[i].stepper_oid = blob[i*4 + 0];
        bindings[i].tmc_cs_oid = staged[i].tmc_cs_oid;
        bindings[i]._pad[0] = 0;
        bindings[i]._pad[1] = 0;
    }
    // ring_depth: host-supplied number of 32-byte PieceEntry slots for this
    // axis, derived as total_piece_memory / 32 / num_axes in the Rust bridge
    // (axis_ring_depth in rust/motion-bridge/src/bridge.rs).
    int32_t rc = kalico_runtime_configure_axis(
        runtime_handle, axis_idx, mode, mstep_bits,
        ring_depth,
        stepper_count > 0 ? bindings : 0,
        stepper_count);
    if (rc != 0)
        shutdown("configure_axis rejected by runtime");

    // ── Phase 3: commit.
    runtime_motor_stepper_count[axis_idx] = stepper_count;
    for (uint8_t i = 0; i < stepper_count; i++) {
        runtime_motor_steppers[axis_idx][i].stepper = staged[i].stepper;
        runtime_motor_steppers[axis_idx][i].invert_dir = staged[i].invert_dir;
    }
    runtime_motor_last_dir[axis_idx] = -1;
    (void)extrusion_bits; // parsed for wire compatibility; no Rust FFI param yet

    // Register the per-axis Klipper timer consumer for THIS axis only — an MCU
    // arms a step-emission timer solely for axes it actually drives. Arming a
    // timer for an unowned axis adds a needless sample-rate dispatch at TIM5's
    // priority and can starve the motion tick (-311); see arm_per_axis_step_timer.
    // Idempotent per axis (the callee tracks an armed mask), so repeat
    // configure_axis calls for the same axis are safe.
    extern void arm_per_axis_step_timer(uint8_t axis_idx);
    arm_per_axis_step_timer(axis_idx);

    // Drive the platform tick-enable now that an axis is configured. On STM32
    // TIM5 is already armed at init, so the idempotent CR1.CEN guard makes this
    // a no-op; on the Linux MCU build this performs the post-connect widen-seed
    // + step-queue install (src/linux/runtime_tick_host.c). Replaces the old
    // set_step_mode-driven enable (removed 2026-05-28).
    extern void runtime_tick_enable(void);
    runtime_tick_enable();
}
DECL_COMMAND(command_kalico_configure_axis,
             "kalico_configure_axis axis_idx=%c mode=%c microstep_distance=%u"
             " extrusion_per_xy_mm=%u stepper_count=%c ring_depth=%hu"
             " steppers=%*s");

// Host-issued clean-state reset. Sent once per MCU on every klippy:connect,
// before the per-axis configure_axis calls, so the Rust engine's ring bump
// allocator (and all per-axis state) starts fresh whether or not the MCU was
// rebooted. Idempotent: a no-op on a freshly-booted MCU.
//
// IRQ guard: the reset clears engine state + the per-axis step queues, both of
// which are concurrently touched by the always-armed TIM5 sample ISR and the
// per-axis step-event timers. irq_save() blocks both for the bounded reset.
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
command_kalico_configure_kinematics(uint32_t *args)
{
    uint32_t k_xy_bits = args[0];
    if (!runtime_handle)
        shutdown("kalico_configure_kinematics before runtime init");
    int32_t rc = kalico_runtime_configure_kinematics(runtime_handle, k_xy_bits);
    if (rc != 0)
        shutdown("kalico_configure_kinematics rejected by runtime");
}
DECL_COMMAND(command_kalico_configure_kinematics,
             "kalico_configure_kinematics k_xy=%u");

void
command_kalico_configure_pressure_advance(uint32_t *args)
{
    uint32_t aa = args[0];
    uint32_t ad = args[1];
    if (!runtime_handle)
        shutdown("kalico_configure_pressure_advance before runtime init");
    int32_t rc = kalico_runtime_configure_pressure_advance(runtime_handle,
                                                            aa, ad);
    if (rc != 0)
        shutdown("kalico_configure_pressure_advance rejected by runtime");
}
DECL_COMMAND(command_kalico_configure_pressure_advance,
             "kalico_configure_pressure_advance advance_accel=%u"
             " advance_decel=%u");

// === Task 12: stepping-redesign axis-mode + stepper-offset handlers ===
//
// `kalico_set_axis_mode` flips one logical axis between Pulse-step and
// Phase-step output modes. Rejected by Rust (non-zero return) if any axis
// has an active Bezier piece (must be issued between segments) or if the
// axis index / mode byte is out of range. Spec sequence (engine-side):
// motion-active gate → flush per-axis step queue → SPI flush (Task 14
// stub) → resync `last_phase_target` on Pulse→Phase → atomic mode publish.
//
// `kalico_set_stepper_offset` adds a target-phase nudge to a single
// stepper's `phase_offset_target`. The Task-13 TIM5 ramp helper walks
// `phase_offset_microsteps` toward the target at most
// `max_microsteps_per_sample` microsteps per sample. Rust latches
// `JogParametersInvalid` on bad arguments before returning non-zero.
//
// Both commands shutdown the MCU on non-zero return — these are
// configuration-class operations, so a host that issues them with bad
// arguments is a hard misconfiguration rather than a recoverable runtime
// condition.

// kalico_runtime_set_axis_mode and kalico_runtime_set_stepper_offset
// are declared in kalico_runtime.h (included above).

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

// Called from the TIM5 ISR (priority 3 on H7) after `runtime_handle_tick`
// produces this tick's step delta. Emits |n_steps| edges on every stepper
// bound to this motor index (primary + AWD partners — e.g. Voron 2.4-style
// 4-motor gantry binds stepper_x and stepper_x1 both to motor 0). All
// step_pins toggle in lockstep; each dir_pin is set to the per-stepper
// `want_dir XOR invert_dir` so that printer.cfg `dir_pin: !PIN` polarity
// is honored end-to-end.
//
// Edge-triggered output (mirrors mainline klipper's stepper_event_edge fast
// path): each iteration produces one edge per stepper. Successive edges
// occur ~30-50 cycles apart (BSRR write + AWD inner loop + branch) = ~60-
// 100 ns at 520 MHz, comfortably above TMC datasheet minimums. No busy-
// wait dwell — neither for the step pulse width nor for dir setup —
// because:
//   * TMC drivers configured for double-edge stepping count every edge as
//     a step; no separate rising/falling pair is required.
//   * Dir setup time (≥20 ns on TMC2209/5160) is satisfied by the natural
//     execution between gpio_out_write of dir_pin and the first step toggle.
//
// Burst budget: at MAX_STEPS_PER_TICK_DEFAULT = 16 with up to 4 AWD
// partners, ~50 cycles per edge = ~3200 cycles ≈ 6 µs per tick worst case,
// well inside the 25 µs tick budget.
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
    }

    // gpio_out_toggle_noirq is the irq-safe variant — caller (us) is in
    // ISR context with IRQs off by virtue of the priority-2 TIM5 vector
    // (KALICO_MOTION_NVIC_PRIO = 2; see src/generic/kalico_nvic_prio.h).
    for (uint32_t i = 0; i < count; i++) {
        for (uint8_t j = 0; j < cnt; j++)
            gpio_out_toggle_noirq(runtime_motor_steppers[motor_idx][j].stepper->step_pin);
    }
}
