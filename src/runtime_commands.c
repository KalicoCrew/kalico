// src/runtime_commands.c
//
// Klipper command surface for the kalico runtime. Every DECL_COMMAND that
// is not part of the lifecycle (runtime_init / runtime_drain / sibling
// drains, which stay in src/runtime_tick.c) lives here. Also hosts the
// endstop arm/disarm commands and the per-tick endstop sampler called from
// each backend's ISR.

#include <stdint.h>
#include "autoconf.h"
#include "board/gpio.h"           // gpio_in_setup / gpio_in_read
#include "command.h"              // DECL_COMMAND, sendf, command_decode_ptr
#include "sched.h"                // DECL_TASK
#include "kalico_runtime.h"       // FFI export prototypes
#include "kalico_dispatch.h"      // kalico_native_emit_*

#if CONFIG_KALICO_RUNTIME

extern void *runtime_handle;      // defined in src/runtime_tick.c

// Aligned scratch buffers for curve-load are also declared here for
// continuity with the legacy file layout — definitions live in
// src/runtime_tick.c.

void
command_runtime_query_status(uint32_t *args)
{
    if (!runtime_handle) {
        sendf("kalico_status status=%c last_err=%i", (uint8_t)255, -7);
        return;
    }
    uint8_t status = runtime_handle_status(runtime_handle);
    int32_t last_err = runtime_handle_last_error(runtime_handle);
    sendf("kalico_status status=%c last_err=%i", status, last_err);
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
// velocity_axis u8, v_min_q16 u32 LE — 11 bytes). Pull configuration is
// not carried on the wire (DIAG outputs are push-pull; mech limits rely on
// external pulls per board); pull_up=0 is requested. If a target board
// requires internal pulls, extend the wire format.
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
        uint16_t gpio_id = (uint16_t)r[1] | ((uint16_t)r[2] << 8);
        endstop_pin_table[i].gpio_id = gpio_id;
        endstop_pin_table[i].pin = gpio_in_setup((uint8_t)gpio_id, 0);
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
    (void)kalico_endstop_arm(arm_id, arm_clock_lo, arm_clock_hi,
                             source_count, sources_ptr, sources_len,
                             stepper_count, steppers_ptr, steppers_len,
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

#endif // CONFIG_KALICO_RUNTIME
