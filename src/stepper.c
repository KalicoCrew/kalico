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
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_SHUTDOWN
#include "stepper.h"
#include "trsync.h" // trsync_add_signal

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
// Incremented at the end of `command_config_runtime_stepper`, after the
// `runtime_motor_stepper_count[motor_idx] = cnt + 1` write. If this lags
// `runtime_bind_calls_total` at steady state, some path between the
// dispatch entry (line ~494) and the count write (line ~512) is
// short-circuiting — `oid_lookup` shutdown is the only known way, but a
// shutdown would also stop the firmware, so divergence suggests something
// stranger.
volatile uint32_t runtime_bind_writes_committed
                __attribute__((used, externally_visible));
// 2-bit-per-motor snapshot of `runtime_motor_stepper_count[i]` captured
// immediately after each successful write at line 512. Packed identically
// to the 0xE2 tag's 4-bit-per-motor layout so the value can be compared
// across snapshots — the LATEST per-motor `cnt + 1` is OR'd in (we don't
// reset between binds, so the high-water mark per motor sticks).
volatile uint32_t runtime_bind_count_snapshot_packed
                __attribute__((used, externally_visible));

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
    {
        // 2026-05-19 diag: pin down "Invalid oid type" shutdown that fires
        // from this oid_lookup on both H7 and F4. Capture, just before the
        // lookup that might shutdown:
        //   tag   = 0xCE
        //   stage = oid_get_count() — MCU's view of allocated oid range
        //   value = (peek_lo << 8) | (motor_idx << 5) | stepper_oid
        // peek_lo is the low byte of oids[stepper_oid].type. If peek_lo is 0
        // then the oid was never allocated (oids[N].type == NULL); if it's
        // 0x01 the oid was out of range (oid_type_peek sentinel); otherwise
        // it's the low byte of whatever function pointer was stamped there.
        // command_config_stepper's low byte is stable for a given build and
        // can be cross-checked offline via `nm`.
        extern void *oid_type_peek(uint8_t oid);
        extern uint8_t oid_get_count(void);
        extern void runtime_diag_progress(uint32_t tag, uint32_t stage,
                                          uint32_t value);
        void *peek = oid_type_peek(stepper_oid);
        uint32_t peek_lo = (uint32_t)((uintptr_t)peek & 0xFFu);
        uint32_t v = (peek_lo << 8)
                   | (((uint32_t)motor_idx & 0x7u) << 5)
                   | ((uint32_t)stepper_oid & 0x1Fu);
        runtime_diag_progress(0xCE, (uint32_t)oid_get_count(), v);
    }
    struct stepper *s = oid_lookup(stepper_oid, command_config_stepper);
    runtime_motor_steppers[motor_idx][cnt].stepper = s;
    runtime_motor_steppers[motor_idx][cnt].invert_dir = invert_dir ? 1 : 0;
    runtime_motor_stepper_count[motor_idx] = cnt + 1;
    runtime_motor_last_dir[motor_idx] = -1;
    extern void init_step_time_timers(void);
    init_step_time_timers();
    // Snapshot every per-motor count immediately after the write completes,
    // packed as 4-bits-per-motor (matches 0xE2 tag layout). Captures the
    // table state visible from `command_config_runtime_stepper`'s vantage
    // point — divergence between this and the 0xE2 read in
    // `runtime_status_drain` indicates an outside writer touching the array
    // after the dispatch returns. Each invocation overwrites with the live
    // values; final value = state after the last successful bind.
    {
        uint8_t s0 = runtime_motor_stepper_count[0];
        uint8_t s1 = runtime_motor_stepper_count[1];
        uint8_t s2 = runtime_motor_stepper_count[2];
        uint8_t s3 = runtime_motor_stepper_count[3];
        if (s0 > 15) s0 = 15;
        if (s1 > 15) s1 = 15;
        if (s2 > 15) s2 = 15;
        if (s3 > 15) s3 = 15;
        uint32_t packed = (uint32_t)s0
                        | ((uint32_t)s1 << 4)
                        | ((uint32_t)s2 << 8)
                        | ((uint32_t)s3 << 12);
        runtime_bind_count_snapshot_packed = packed;
    }
    runtime_bind_writes_committed++;
}
DECL_COMMAND(command_config_runtime_stepper,
             "config_runtime_stepper motor_idx=%c stepper_oid=%c invert_dir=%c");

// ─── Stepping-redesign Task 11 — kalico_configure_* command handlers ─────
//
// Three foreground commands that publish per-axis configuration, the
// kinematic scale factor, and pressure-advance coefficients into the Rust
// runtime. Each handler is a thin shim: unpack the Klipper-protocol
// `uint32_t *args`, forward to the Rust FFI, and `shutdown(...)` on any
// non-zero return so configuration errors are loud rather than silent.
//
// `runtime_handle` is the global published by `runtime_handle_create()`
// in src/runtime_tick.c. The legacy `command_config_runtime_stepper`
// above does NOT use it because it manipulates C-side stepper-binding
// arrays directly; these new commands DO call into Rust and therefore
// need the handle. Both code paths coexist until Task 16 deletes the
// legacy command.
//
// Wire-format note: Klipper carries f32 fields as `u32` containing
// `f32::to_bits()` (the host packs, the MCU forwards the raw bits, and
// `f32::from_bits` reconstructs on the Rust side). `%u` matches u32.

extern void *runtime_handle; // defined in src/runtime_tick.c

extern int32_t kalico_runtime_configure_axis(
    void *handle, uint8_t axis_idx, uint8_t mode,
    uint32_t microstep_distance_f32_bits,
    uint32_t extrusion_per_xy_mm_f32_bits,
    uint8_t stepper_count);

extern int32_t kalico_runtime_configure_kinematics(
    void *handle, uint32_t k_xy_f32_bits);

extern int32_t kalico_runtime_configure_pressure_advance(
    void *handle, uint32_t advance_accel_f32_bits,
    uint32_t advance_decel_f32_bits);

void
command_kalico_configure_axis(uint32_t *args)
{
    uint8_t axis_idx = args[0];
    uint8_t mode = args[1];
    uint32_t mstep_bits = args[2];
    uint32_t extrusion_bits = args[3];
    uint8_t stepper_count = args[4];
    if (!runtime_handle)
        shutdown("kalico_configure_axis before runtime init");
    int32_t rc = kalico_runtime_configure_axis(runtime_handle, axis_idx, mode,
                                                mstep_bits, extrusion_bits,
                                                stepper_count);
    if (rc != 0)
        shutdown("kalico_configure_axis rejected by runtime");
}
DECL_COMMAND(command_kalico_configure_axis,
             "kalico_configure_axis axis_idx=%c mode=%c microstep_distance=%u"
             " extrusion_per_xy_mm=%u stepper_count=%c");

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
    // ISR context with IRQs off by virtue of the priority-3 TIM5 vector.
    for (uint32_t i = 0; i < count; i++) {
        for (uint8_t j = 0; j < cnt; j++)
            gpio_out_toggle_noirq(runtime_motor_steppers[motor_idx][j].stepper->step_pin);
    }
}
