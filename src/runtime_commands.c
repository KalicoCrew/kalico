// src/runtime_commands.c
//
// Klipper command surface for the kalico runtime. Every DECL_COMMAND that
// is not part of the lifecycle (runtime_init / runtime_drain / sibling
// drains, which stay in src/runtime_tick.c) lives here. Also hosts the
// endstop arm/disarm commands and the per-tick endstop sampler called from
// each backend's ISR.

#include <stdint.h>
#include <stdio.h>
#include "autoconf.h"
#include "board/gpio.h"           // gpio_in_setup / gpio_in_read / spi_setup
#include "command.h"              // DECL_COMMAND, sendf, command_decode_ptr
#include "sched.h"                // DECL_TASK
#include "board/misc.h"           // timer_read_time
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

// Scratch buffer declarations for the curve-load path are kept in
// src/runtime_tick.c for historical layout continuity.

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

// ---- Step 7-D: endstop arm/disarm/tripped wire surface --------------------

// Step 7.5 — Production GPIO sampler. The runtime endstop module reads pin
// levels from an internal abstract pin table (rust/runtime/src/endstop.rs's
// PIN_LEVELS). To trip on real hardware we sample the configured GPIOs from
// the modulation ISR (TIM5_IRQHandler) once per tick and push the result
// through `kalico_endstop_set_pin_level` before `runtime_handle_tick`
// observes the table. The active set is populated when an arm succeeds and
// cleared on disarm. Slot count must match runtime::endstop::MAX_SOURCES.
#define KALICO_ENDSTOP_MAX_SOURCES 4
#define KALICO_ENDSTOP_SOURCE_RECORD_LEN 11
struct endstop_pin_slot {
    uint8_t        active;     // 0 = empty, non-zero = sampled each tick
    uint16_t       gpio_id;    // mirrored into runtime PIN_LEVELS index
    struct gpio_in pin;
};
static struct endstop_pin_slot endstop_pin_table[KALICO_ENDSTOP_MAX_SOURCES];

extern int32_t kalico_endstop_set_pin_level(uint16_t gpio, uint8_t level);

/// Scan all active endstop sources and update their trigger state.
///
/// `stepper_idx` is currently unused — the endstop pin table is
/// indexed by source slot, not by stepper OID, so any call samples
/// all active sources. The argument is kept for API symmetry with
/// `runtime_endstop_sample_one(stepper_idx)`, which the step-time
/// ISR calls per-step. If the pin table is ever made stepper-indexed,
/// this helper can be specialized; for now it's a single function
/// shared by both paths.
static inline void
sample_endstops_for_stepper(uint8_t stepper_idx)
{
    (void)stepper_idx;  // unused — documented above
    for (int i = 0; i < KALICO_ENDSTOP_MAX_SOURCES; i++) {
        if (!endstop_pin_table[i].active)
            continue;
        uint8_t level = gpio_in_read(endstop_pin_table[i].pin);
        (void)kalico_endstop_set_pin_level(endstop_pin_table[i].gpio_id, level);
    }
}

// Called from TIM5_IRQHandler immediately before runtime_handle_tick.
// Hot path: at most KALICO_ENDSTOP_MAX_SOURCES (=4) register reads per tick.
// The helper does a full scan of active sources internally; call it once.
void
runtime_endstop_sample_pins(void)
{
    // The helper does a full scan of active endstop sources internally.
    // The stepper_idx argument is unused (source-indexed table, not
    // stepper-indexed) — pass 0 as a convention.
    sample_endstops_for_stepper(0);
}

// Called from the per-stepper step-time ISR (Task D1) so the step-time
// path samples endstops at step resolution, not only at TIM5 cadence.
// Bounds-checked defensively against the engine stepper limit (8 —
// must match MAX_STEPPER_OIDS_C in src/runtime_tick.c and
// MAX_STEPPER_OIDS in rust/runtime/src/state.rs).
// The endstop_pin_table is source-indexed, not stepper-indexed, so any
// valid stepper_idx triggers a full active-source scan.
__attribute__((used, visibility("default")))
void
runtime_endstop_sample_one(uint8_t stepper_idx)
{
    if (stepper_idx >= 8)   // 8 == MAX_STEPPER_OIDS_C
        return;
    sample_endstops_for_stepper(stepper_idx);
}

static void
endstop_pin_table_clear(void)
{
    for (int i = 0; i < KALICO_ENDSTOP_MAX_SOURCES; i++)
        endstop_pin_table[i].active = 0;
}

// Populate the sampler table from the wire-format sources blob. Mirrors
// rust/kalico-c-api/src/runtime_ffi.rs::kalico_endstop_arm decode (record
// layout: kind u8, gpio u16 LE, active_high u8, policy u8, sample_n u8,
// velocity_axis u8, v_min_q16 u32 LE — 11 bytes).
//
// Pull configuration: TMC DIAG outputs are open-drain by default (TMC5160
// GCONF.diag1_int_pushpull == 0 at reset); without an internal pullup the
// pin floats LOW when stallguard is inactive and `^!PG9`-style configs see
// `asserted=True` at idle, so the post-retract "endstop still triggered"
// check fires before the motor moves. The host's pullup flag (the `^` in
// e.g. `^!PG9`) is not carried on the wire yet, so apply the
// "TmcDiag → pullup, Physical → no pull" convention here. Physical mech
// limits typically have external pulls per board and the firmware uses
// `pull_up=0` for them.
static void
endstop_pin_table_populate(uint8_t source_count, const uint8_t *sources_ptr)
{
    endstop_pin_table_clear();
    if (!sources_ptr || source_count == 0)
        return;
    uint8_t n = source_count;
    if (n > KALICO_ENDSTOP_MAX_SOURCES)
        n = KALICO_ENDSTOP_MAX_SOURCES;
    for (uint8_t i = 0; i < n; i++) {
        const uint8_t *r = sources_ptr + (uint32_t)i * KALICO_ENDSTOP_SOURCE_RECORD_LEN;
        // r[0] = kind (0 = Physical, 1 = TmcDiag, 2 = Software).
        uint8_t kind = r[0];
        if (kind == 2)
            continue;   // Software sources have no GPIO to sample
        uint16_t gpio_id = (uint16_t)r[1] | ((uint16_t)r[2] << 8);
        int32_t pull_up = (kind == 1) ? 1 : 0;
        endstop_pin_table[i].gpio_id = gpio_id;
        endstop_pin_table[i].pin = gpio_in_setup((uint8_t)gpio_id, pull_up);
        endstop_pin_table[i].active = 1;
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
    // PT_buffer args carry an encoded pointer (offset on 64-bit hosts);
    // command_decode_ptr resolves it to a real address. A bare cast
    // works on 32-bit MCUs but segfaults on Linux/64-bit sim.
    uint8_t *sources_ptr = command_decode_ptr(args[5]);
    uint8_t stepper_count = args[6];
    uint32_t steppers_len = args[7];
    uint8_t *steppers_ptr = command_decode_ptr(args[8]);
    uint8_t status = 2; // Rejected
    // Compute grant_ticks: 50ms at MCU clock freq for Software sources
    uint64_t grant_ticks = 0;
    if (sources_ptr && source_count > 0) {
        for (uint8_t i = 0; i < source_count; i++) {
            if (sources_ptr[(uint32_t)i * KALICO_ENDSTOP_SOURCE_RECORD_LEN] == 2) {
                grant_ticks = (uint64_t)CONFIG_CLOCK_FREQ / 20;
                break;
            }
        }
    }
    (void)kalico_endstop_arm(arm_id, arm_clock_lo, arm_clock_hi,
                             source_count, sources_ptr, sources_len,
                             stepper_count, steppers_ptr, steppers_len,
                             (uint32_t)grant_ticks,
                             (uint32_t)(grant_ticks >> 32),
                             &status);
    // Only wire up GPIO sampling when the runtime accepted the arm.
    // status: 0 = Armed, 1 = AlreadyTripped, 2 = Rejected. AlreadyTripped
    // means the snapshot is already published — no further sampling needed.
    if (status == 0)
        endstop_pin_table_populate(source_count, sources_ptr);
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
    // Stop sampling regardless of disarm outcome — Disarmed and
    // AlreadyTripped both terminate the active arm; Unknown means the
    // table is already stale.
    endstop_pin_table_clear();
    sendf("kalico_disarm_endstop_response arm_id=%u status=%c", arm_id, status);
}
DECL_COMMAND(command_runtime_disarm_endstop, "runtime_disarm_endstop arm_id=%u");

// ---- Software trip + deadline extension for external probe homing --------
// These use the basecmd.c clock-widening variables declared below the
// clock-sync handler — forward-declare here so the handlers compile.
extern uint32_t stats_send_time;        // basecmd.c
extern uint32_t stats_send_time_high;   // basecmd.c

void
command_runtime_software_trip(uint32_t *args)
{
    uint32_t arm_id = args[0];
    uint32_t clock_lo = timer_read_time();
    // Widen using the same pattern as command_runtime_clock_sync_request
    uint32_t clock_hi = stats_send_time_high + (clock_lo < stats_send_time);
    uint8_t status = 1; // NotArmed default
    (void)kalico_software_trip(arm_id, clock_lo, clock_hi, &status);
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
static struct runtime_stop_binding {
    struct trsync_signal signal;
    uint32_t arm_id;
} runtime_stop_binding;

static void
runtime_stop_on_trigger_cb(struct trsync_signal *tss, uint8_t reason)
{
    (void)reason;
    struct runtime_stop_binding *b =
        container_of(tss, struct runtime_stop_binding, signal);
    uint32_t clock_lo = timer_read_time();
    uint32_t clock_hi = stats_send_time_high + (clock_lo < stats_send_time);
    uint8_t status = 1; // NotArmed default
    (void)kalico_software_trip(b->arm_id, clock_lo, clock_hi, &status);
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


void
command_runtime_extend_homing_deadline(uint32_t *args)
{
    uint32_t arm_id = args[0];
    uint32_t clock_lo = timer_read_time();
    uint32_t clock_hi = stats_send_time_high + (clock_lo < stats_send_time);
    (void)kalico_extend_deadline(arm_id, clock_lo, clock_hi);
}
DECL_COMMAND(command_runtime_extend_homing_deadline,
    "runtime_extend_homing_deadline arm_id=%u");

// ---- Seed position: SET_KINEMATIC_POSITION → MCU engine origin fix --------
//
// When klippy issues SET_KINEMATIC_POSITION the host-side ShaperState is
// re-anchored but the MCU engine's prev_x/y/z stay at their boot values
// (0.0). The first segment after the anchor change carries the correct
// endpoint but the engine's delta = (endpoint - 0) instead of
// (endpoint - anchor), blowing past MAX_STEPS_PER_TICK_DEFAULT (65 536)
// and raising FaultCode::StepBurstExceeded (65515).
//
// This command seeds the MCU engine's position origin so its
// prev_x/y/z match the host's commanded position before the first
// segment arrives.
//
// Positions are Q16.16 fixed-point (i32 = mm * 65536). Decoded to f32
// in the Rust FFI. No response is sent — this is fire-and-forget;
// the following PushSegment provides the real sequencing guarantee.
// kalico_runtime_seed_position is declared in kalico_runtime.h (included above).

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

// ---- Step-6 §12.1 clock-sync request ----------------------------------
//
// Compute the widened MCU clock in C using the same formula as
// `command_get_uptime` (basecmd.c): `low = timer_read_time(); high =
// stats_send_time_high + (low < stats_send_time)`. We do NOT call into Rust
// for this — `runtime::stream::clock_sync_respond` reads a SharedState
// seqlock populated only by the TIM5 ISR, which stays disabled in the
// all-StepTime MVP, so the seqlock returns 0 and the host's clock-sync
// driver filters out every sample as "uninitialised". Bypassing the FFI
// keeps everything in C and matches the widening Klippy itself uses.
extern uint32_t stats_send_time;        // basecmd.c
extern uint32_t stats_send_time_high;   // basecmd.c
void
command_runtime_clock_sync_request(uint32_t *args)
{
    uint32_t request_id = args[0];
    // args[1] / args[2] = host_send_time_{lo,hi} — unused by the current
    // bridge regression (it derives RTT from wall-clock send/recv timestamps).
    // Retained on the wire for forward compatibility with §12.1 RTT bounding.
    uint32_t low = timer_read_time();
    uint32_t high = stats_send_time_high + (low < stats_send_time);
    sendf(
        "kalico_clock_sync_response request_id=%u mcu_clock_lo=%u mcu_clock_hi=%u",
        request_id, low, high);
}
DECL_COMMAND(command_runtime_clock_sync_request,
    "runtime_clock_sync_request request_id=%u "
    "host_send_time_lo=%u host_send_time_hi=%u");

// ---- 2026-05-18 phase-stepping SPI bus registration ----------------------
// Closes the gap between the Rust runtime's per-motor phase_config storage
// and the C-side `phase_stepping_write_xdirect` path. Without this command,
// every XDIRECT write from the modulator hits the `if (!configured) return;`
// early-exit in src/stm32/phase_stepping_spi.c and silently drops.
//
// Two-stage registration (2026-05-19 — fixes the multi-TMC5160-on-one-bus
// CS-aliasing bug, see docs/superpowers/specs/2026-05-19-phase-stepping-
// per-motor-cs-design.md):
//   1. `runtime_register_phase_bus bus_id=%c rate=%u` — once per unique
//      bus_id, installs the shared SPI cfg (rate, mode 3).
//   2. `runtime_register_phase_motor motor_idx=%c bus_id=%c cs_pin_id=%c`
//      — once per phase-stepped motor, installs that motor's CS GPIO.
// Both must precede the first `kalico_configure_axis` command.
//
// STM32-only because the underlying phase_stepping_spi.c is STM32-only.
// On linux/sim hosts (non-STM32 mach) both return -88 ("not supported on
// this target"); the Renode sim is STM32H7 so they are no-ops for it.
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

// NOTE: param is `cs_pin_id` not `cs_pin` deliberately. Klipper's msgproto
// matches param names against the `pin` enumeration via
// `name.endswith("_pin")` (see klippy/msgproto.py::lookup_params), which
// would force the host to send symbolic pin names ("PA5") instead of the
// raw stm32 GPIO encoding (port*16+pin = 5) used by the rest of the
// phase_config wire surface. The `_id` suffix sidesteps the enum lookup
// and keeps the encoding consistent with the `kalico_configure_axis` path.
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

