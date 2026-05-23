// src/runtime_commands.c
//
// Klipper command surface for the kalico runtime. Every DECL_COMMAND that
// is not part of the lifecycle (runtime_init / runtime_drain / sibling
// drains, which stay in src/runtime_tick.c) lives here. Also hosts the
// endstop arm/disarm commands and the per-tick endstop sampler called from
// each backend's ISR.

#include <stdint.h>
#include "autoconf.h"
#include "board/gpio.h"           // gpio_in_setup / gpio_in_read / spi_setup
#include "command.h"              // DECL_COMMAND, sendf, command_decode_ptr
#include "sched.h"                // DECL_TASK
#include "board/misc.h"           // timer_read_time
#include "kalico_runtime.h"       // FFI export prototypes
#include "kalico_dispatch.h"      // kalico_native_emit_*
#if CONFIG_MACH_STM32
#include "stm32/phase_stepping_spi.h"  // phase_stepping_register_bus
#endif


extern void *runtime_handle;      // defined in src/runtime_tick.c

// Aligned scratch buffers for curve-load are also declared here for
// continuity with the legacy file layout — definitions live in
// src/runtime_tick.c.

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
#if CONFIG_MACH_STM32
    // Cooperative SPI3 contention counter, incremented from the TIM5 ISR
    // when the phase-stepping write loses the busy-flag race vs Klipper's
    // low-priority TMC SPI register access. Surface on the periodic
    // status frame so the host (and the bench acceptance gate) can
    // monitor that contention stays inside the expected envelope
    // (<100/s sustained per spec §8.4 / Task 13).
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
        // r[0] = kind (0 = Physical, 1 = TmcDiag — must match the
        // SourceKind enum in rust/runtime/src/endstop.rs).
        uint8_t kind = r[0];
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

void
command_runtime_configure_axes(uint32_t *args)
{
    if (!runtime_handle) {
        sendf("kalico_configure_axes_response result=%i", -7);
        return;
    }
    uint8_t kinematics = args[0];
    int32_t r = kalico_configure_axes(runtime_handle, kinematics);
    sendf("kalico_configure_axes_response result=%i", r);
}
DECL_COMMAND(command_runtime_configure_axes, "runtime_configure_axes kinematics=%c");

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

// ---- Step-6 §8.3 stream lifecycle commands ----------------------------
// Phase 3.2 declares the wire surface; Phase 6 wires the actual state-
// machine transitions in `runtime::stream`. The FFIs return -140
// (KALICO_ERR_STREAM_STATE_VIOLATION) until Phase 6 lands.

void
command_runtime_stream_open(uint32_t *args)
{
    if (!runtime_handle) {
        sendf("kalico_stream_open_response result=%i credit_epoch=%u", -7, 0);
        return;
    }
    uint32_t stream_id = args[0];
    uint32_t credit_epoch = 0;
    int32_t r = kalico_runtime_stream_open(
        runtime_handle, stream_id, &credit_epoch);
    sendf("kalico_stream_open_response result=%i credit_epoch=%u",
          r, credit_epoch);
}
DECL_COMMAND(command_runtime_stream_open, "runtime_stream_open stream_id=%u");

void
command_runtime_stream_arm(uint32_t *args)
{
    if (!runtime_handle) {
        sendf(
            "kalico_stream_arm_response result=%i armed_t_start_lo=%u armed_t_start_hi=%u",
            -7, 0, 0);
        return;
    }
    uint64_t t_start_t0 = ((uint64_t)args[1] << 32) | args[0];
    uint32_t arm_lead_cycles = args[2];
    uint64_t armed_t_start = 0;
    int32_t r = kalico_runtime_stream_arm(
        runtime_handle, t_start_t0, arm_lead_cycles, &armed_t_start);
    sendf(
        "kalico_stream_arm_response result=%i armed_t_start_lo=%u armed_t_start_hi=%u",
        r, (uint32_t)armed_t_start, (uint32_t)(armed_t_start >> 32));
}
DECL_COMMAND(command_runtime_stream_arm,
    "runtime_stream_arm t_start_t0_lo=%u t_start_t0_hi=%u arm_lead_cycles=%u");

void
command_runtime_stream_terminal(uint32_t *args)
{
    if (!runtime_handle) {
        sendf("kalico_stream_terminal_response result=%i", -7);
        return;
    }
    uint32_t segment_id = args[0];
    int32_t r = kalico_runtime_stream_terminal(runtime_handle, segment_id);
    sendf("kalico_stream_terminal_response result=%i", r);
}
DECL_COMMAND(command_runtime_stream_terminal,
    "runtime_stream_terminal segment_id=%u");

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

// ---- Step-6 §10.4 / Round-1 B9 diagnostic --------------------------------
// Per-slot curve-pool generation snapshot. Used by the host after a fault to
// decide whether the pool can be reused or a power-cycle is required.
void
command_runtime_query_pool_state(uint32_t *args)
{
    if (!runtime_handle) {
        sendf(
            "kalico_pool_state_response result=%i slot_idx=%hu current_gen=%hu last_retired_gen=%hu",
            -7, (uint16_t)0, (uint16_t)0, (uint16_t)0);
        return;
    }
    uint16_t slot = args[0];
    uint16_t current_gen = 0;
    uint16_t last_retired_gen = 0;
    int32_t r = runtime_handle_query_pool_state(
        runtime_handle, slot, &current_gen, &last_retired_gen);
    sendf(
        "kalico_pool_state_response result=%i slot_idx=%hu current_gen=%hu last_retired_gen=%hu",
        r, slot, current_gen, last_retired_gen);
}
DECL_COMMAND(command_runtime_query_pool_state,
    "runtime_query_pool_state slot=%hu");

// ---- 2026-05-18 phase-stepping diagnostic gate -----------------------------
// Exposes `kalico_runtime_set_phase_trace_enabled` on the wire so host-side
// sim tests (tools/test_sim_phase_stepping.py) can flip the per-print
// PhaseStep trace push without going through the Klippy bridge crate.
// Production builds default to off; tests turn it on for the duration of a
// jog, drain the trace ring via the existing `kalico_trace` output frame,
// and turn it off again.
void
command_runtime_set_phase_trace(uint32_t *args)
{
    if (!runtime_handle) {
        sendf("kalico_set_phase_trace_response result=%i", -7);
        return;
    }
    uint8_t enabled = (uint8_t)args[0];
    int32_t r = kalico_runtime_set_phase_trace_enabled(runtime_handle, enabled);
    sendf("kalico_set_phase_trace_response result=%i", r);
}
DECL_COMMAND(command_runtime_set_phase_trace,
    "runtime_set_phase_trace enabled=%c");

// ---- 2026-05-18 configure_axes binary blob via msgproto --------------------
// Production routes configure_axes through the kalico-native binary frame
// transport (KALICO_MSG_CONFIGURE_AXES, see src/kalico_dispatch.c). That
// path requires a separate sync byte (0x55) and CRC scheme that the
// standalone host-io helper (tools/kalico_host_io.py) does not demux —
// responses never reach Python callers driving the sim from a plain
// pyserial socket. This DECL_COMMAND surfaces the same Rust FFI through
// the standard Klipper msgproto path so sim tests can install per-motor
// phase config (33-byte blob) without standing up the full bridge crate.
// Accepts 20-byte (legacy), 25-byte (extended StepMode), or 33-byte
// (phase-stepping per-motor SPI config) blobs.
void
command_runtime_configure_axes_blob(uint32_t *args)
{
    if (!runtime_handle) {
        sendf("kalico_configure_axes_blob_response result=%i", -7);
        return;
    }
    uint32_t blob_len = args[0];
    uint8_t *blob_ptr = command_decode_ptr(args[1]);
    // Accept 20 / 25 / 26+3N (N in 0..=16). Rust parser validates the
    // per-motor entries; wrapper only gates length to a recognized shape.
    int accept = (blob_len == 20) || (blob_len == 25);
    if (!accept && blob_len >= 26) {
        uint32_t tail = blob_len - 26;
        if (tail % 3 == 0 && (tail / 3) <= 16) {
            accept = 1;
        }
    }
    if (!accept) {
        sendf("kalico_configure_axes_blob_response result=%i", -1);
        return;
    }
    int32_t r = kalico_runtime_configure_axes_blob(runtime_handle,
                                                   blob_ptr,
                                                   blob_len);
    if (r == 0) {
        // New stepping path uses init_per_axis_step_timers; deleted in
        // stepping-redesign-finish Task 17.
    }
    sendf("kalico_configure_axes_blob_response result=%i", r);
}
DECL_COMMAND(command_runtime_configure_axes_blob,
    "runtime_configure_axes_blob blob=%*s");

// ---- 2026-05-18 sim test driver: push_segment via msgproto -----------------
// Mirrors KALICO_MSG_PUSH_SEGMENT through Klipper msgproto. The full 42-byte
// segment body is encoded as a single %*s blob to keep the framed packet
// under MESSAGE_MAX = 64 bytes (12 separate PT_uint32 args at max-varint
// width would exceed the cap).
//
// Wire body layout matches kalico_dispatch.c::handle_push_segment §7.4:
//   id u32 | x/y/z/e u32 each | t_start u64 | t_end u64 |
//   kinematics u8 | e_mode u8 | extrusion_ratio_bits u32  (42 bytes total)
void
command_runtime_push_segment_msgproto(uint32_t *args)
{
    if (!runtime_handle) {
        sendf(
            "kalico_push_segment_msgproto_response result=%i "
            "accepted_segment_id=%u credit_epoch=%u",
            -7, 0, 0);
        return;
    }
    uint32_t body_len = args[0];
    uint8_t *body = command_decode_ptr(args[1]);
    if (body_len != 42) {
        sendf(
            "kalico_push_segment_msgproto_response result=%i "
            "accepted_segment_id=%u credit_epoch=%u",
            -1, 0, 0);
        return;
    }
    uint32_t id = (uint32_t)body[0] | ((uint32_t)body[1] << 8)
                | ((uint32_t)body[2] << 16) | ((uint32_t)body[3] << 24);
    uint32_t x_handle = (uint32_t)body[4] | ((uint32_t)body[5] << 8)
                      | ((uint32_t)body[6] << 16) | ((uint32_t)body[7] << 24);
    uint32_t y_handle = (uint32_t)body[8] | ((uint32_t)body[9] << 8)
                      | ((uint32_t)body[10] << 16) | ((uint32_t)body[11] << 24);
    uint32_t z_handle = (uint32_t)body[12] | ((uint32_t)body[13] << 8)
                      | ((uint32_t)body[14] << 16) | ((uint32_t)body[15] << 24);
    uint32_t e_handle = (uint32_t)body[16] | ((uint32_t)body[17] << 8)
                      | ((uint32_t)body[18] << 16) | ((uint32_t)body[19] << 24);
    uint64_t t_start = 0;
    for (int i = 0; i < 8; i++)
        t_start |= ((uint64_t)body[20 + i]) << (8 * i);
    uint64_t t_end = 0;
    for (int i = 0; i < 8; i++)
        t_end |= ((uint64_t)body[28 + i]) << (8 * i);
    uint8_t kinematics = body[36];
    uint8_t e_mode = body[37];
    uint32_t extrusion_ratio_bits = (uint32_t)body[38] | ((uint32_t)body[39] << 8)
                                  | ((uint32_t)body[40] << 16) | ((uint32_t)body[41] << 24);
    uint32_t accepted_id = 0, credit_epoch = 0;
    int32_t r = runtime_handle_push_segment(
        runtime_handle, id, x_handle, y_handle, z_handle, e_handle,
        t_start, t_end, kinematics, e_mode, extrusion_ratio_bits,
        &accepted_id, &credit_epoch);
    if (r == 0) {
        // New stepping path's TIM5 ISR dequeues segments directly; no
        // producer timer to arm. Stepping-redesign-finish Task 17.
    }
    sendf(
        "kalico_push_segment_msgproto_response result=%i "
        "accepted_segment_id=%u credit_epoch=%u",
        r, accepted_id, credit_epoch);
}
DECL_COMMAND(command_runtime_push_segment_msgproto,
    "runtime_push_segment_msgproto body=%*s");

// ---- 2026-05-18 phase-stepping SPI bus registration ----------------------
// Closes the gap between the Rust runtime's per-motor phase_config storage
// (installed via runtime_configure_axes_blob) and the C-side
// `phase_stepping_write_xdirect` path. Without this command, every XDIRECT
// write from the modulator hits the `if (!configured) return;` early-exit
// in src/stm32/phase_stepping_spi.c and silently drops.
//
// Two-stage registration (2026-05-19 — fixes the multi-TMC5160-on-one-bus
// CS-aliasing bug, see docs/superpowers/specs/2026-05-19-phase-stepping-
// per-motor-cs-design.md):
//   1. `runtime_register_phase_bus bus_id=%c rate=%u` — once per unique
//      bus_id, installs the shared SPI cfg (rate, mode 3).
//   2. `runtime_register_phase_motor motor_idx=%c bus_id=%c cs_pin_id=%c`
//      — once per phase-stepped motor, installs that motor's CS GPIO.
// Both must precede `runtime_configure_axes_blob`.
//
// STM32-only because the underlying phase_stepping_spi.c is STM32-only.
// On linux/sim hosts (non-STM32 mach) both return -88 ("not supported on
// this target"); the Renode sim is STM32H7 so they are no-ops for it.
void
command_runtime_register_phase_bus(uint32_t *args)
{
#if CONFIG_MACH_STM32
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
// and keeps the encoding consistent with the 33-byte configure_axes blob.
void
command_runtime_register_phase_motor(uint32_t *args)
{
#if CONFIG_MACH_STM32
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

