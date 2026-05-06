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

// ---- Step 7-B: homed gate + axis configuration --------------------------

void
command_runtime_set_homed(uint32_t *args)
{
    (void)args;
    if (!runtime_handle) {
        sendf("kalico_set_homed_response result=%i", -7);
        return;
    }
    int32_t r = kalico_set_homed(runtime_handle);
    sendf("kalico_set_homed_response result=%i", r);
}
DECL_COMMAND(command_runtime_set_homed, "runtime_set_homed");

// Step 7-D: parameterized homed-state setter. Spec §8 — sibling of the
// no-arg runtime_set_homed (preserved for backward compat), letting the
// host explicitly set or clear the gate (homed=0 clears, non-zero sets).
void
command_runtime_set_homed_state(uint32_t *args)
{
    if (!runtime_handle) {
        sendf("kalico_set_homed_response result=%i", -7);
        return;
    }
    uint8_t homed = args[0];
    int32_t r = kalico_set_homed_state(runtime_handle, homed);
    sendf("kalico_set_homed_response result=%i", r);
}
DECL_COMMAND(command_runtime_set_homed_state, "runtime_set_homed_state homed=%c");

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

// Called from TIM5_IRQHandler immediately before runtime_handle_tick.
// Hot path: at most KALICO_ENDSTOP_MAX_SOURCES (=4) register reads per tick.
void
runtime_endstop_sample_pins(void)
{
    for (int i = 0; i < KALICO_ENDSTOP_MAX_SOURCES; i++) {
        if (!endstop_pin_table[i].active)
            continue;
        uint8_t level = gpio_in_read(endstop_pin_table[i].pin);
        (void)kalico_endstop_set_pin_level(endstop_pin_table[i].gpio_id, level);
    }
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
void
command_runtime_clock_sync_request(uint32_t *args)
{
    if (!runtime_handle) {
        sendf(
            "kalico_clock_sync_response request_id=%u mcu_clock_lo=%u mcu_clock_hi=%u",
            0, 0, 0);
        return;
    }
    uint32_t request_id = args[0];
    uint32_t host_send_time_lo = args[1];
    uint32_t host_send_time_hi = args[2];
    uint64_t mcu_clock = 0;
    kalico_runtime_clock_sync_request(
        runtime_handle, request_id,
        host_send_time_lo, host_send_time_hi,
        &mcu_clock);
    sendf(
        "kalico_clock_sync_response request_id=%u mcu_clock_lo=%u mcu_clock_hi=%u",
        request_id, (uint32_t)mcu_clock, (uint32_t)(mcu_clock >> 32));
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
