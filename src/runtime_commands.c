// Klipper command surface for the kalico runtime; also hosts the endstop
// arm/disarm commands and the per-tick endstop sampler.

#include <stdint.h>
#include <stdio.h>
#include "autoconf.h"
#include "board/gpio.h"           // gpio_in_setup / gpio_in_read / spi_setup
#include "command.h"              // DECL_COMMAND, sendf, command_decode_ptr
#include "sched.h"                // DECL_TASK
#include "board/misc.h"           // timer_read_time
#include "kalico_runtime.h"       // FFI export prototypes
#include "kalico_dispatch.h"      // kalico_native_emit_*
#if CONFIG_MACH_STM32
#include "stm32/phase_stepping_spi.h"
#elif CONFIG_MACH_LINUX
#include "linux/phase_stepping_spi.h"
#endif


extern void *runtime_handle;      // defined in src/runtime_tick.c

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

// Sample the configured GPIOs from the modulation ISR and push results through
// kalico_endstop_set_pin_level before runtime_handle_tick observes the table.
// Slot count must match runtime::endstop::MAX_SOURCES.
#define KALICO_ENDSTOP_MAX_SOURCES 4
#define KALICO_ENDSTOP_SOURCE_RECORD_LEN 11
struct endstop_pin_slot {
    uint8_t        active;
    uint16_t       gpio_id;    // index into runtime PIN_LEVELS
    struct gpio_in pin;
};
static struct endstop_pin_slot endstop_pin_table[KALICO_ENDSTOP_MAX_SOURCES];

extern int32_t kalico_endstop_set_pin_level(uint16_t gpio, uint8_t level);

// stepper_idx is unused — the pin table is source-indexed, so any call samples
// all active sources. The argument is kept for symmetry with
// runtime_endstop_sample_one.
static inline void
sample_endstops_for_stepper(uint8_t stepper_idx)
{
    (void)stepper_idx;
    for (int i = 0; i < KALICO_ENDSTOP_MAX_SOURCES; i++) {
        if (!endstop_pin_table[i].active)
            continue;
        uint8_t level = gpio_in_read(endstop_pin_table[i].pin);
        (void)kalico_endstop_set_pin_level(endstop_pin_table[i].gpio_id, level);
    }
}

// Called from TIM5_IRQHandler immediately before runtime_handle_tick — at most
// KALICO_ENDSTOP_MAX_SOURCES (=4) register reads per tick.
void
runtime_endstop_sample_pins(void)
{
    sample_endstops_for_stepper(0);
}

// Bound must match MAX_STEPPER_OIDS_C (runtime_tick.c) / MAX_STEPPER_OIDS
// (rust/runtime/src/state.rs).
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

// Record layout mirrors rust/kalico-c-api/src/runtime_ffi.rs::kalico_endstop_arm
// decode: kind u8, gpio u16 LE, active_high u8, policy u8, sample_n u8,
// velocity_axis u8, v_min_q16 u32 LE — 11 bytes.
//
// TMC DIAG outputs are open-drain (GCONF.diag1_int_pushpull==0 at reset) and
// float LOW without a pullup, so a `^!PG9`-style config reads asserted at idle.
// The host's pullup flag is not on the wire yet, so apply "TmcDiag → pullup,
// Physical → no pull" here.
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
        uint8_t kind = r[0];   // 0=Physical, 1=TmcDiag, 2=Software
        if (kind == 2)
            continue;   // Software: no GPIO to sample
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
    // PT_buffer args carry an encoded pointer; command_decode_ptr resolves it.
    // A bare cast works on 32-bit MCUs but segfaults on 64-bit sim.
    uint8_t *sources_ptr = command_decode_ptr(args[5]);
    uint8_t stepper_count = args[6];
    uint32_t steppers_len = args[7];
    uint8_t *steppers_ptr = command_decode_ptr(args[8]);
    uint8_t status = 2; // Rejected
    (void)kalico_endstop_arm(arm_id, arm_clock_lo, arm_clock_hi,
                             source_count, sources_ptr, sources_len,
                             stepper_count, steppers_ptr, steppers_len,
                             &status);
    // status: 0=Armed, 1=AlreadyTripped, 2=Rejected. Sample GPIOs only on
    // Armed; AlreadyTripped already published its snapshot.
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
    // Stop sampling regardless of outcome.
    endstop_pin_table_clear();
    sendf("kalico_disarm_endstop_response arm_id=%u status=%c", arm_id, status);
}
DECL_COMMAND(command_runtime_disarm_endstop, "runtime_disarm_endstop arm_id=%u");

extern uint32_t stats_send_time;        // basecmd.c
extern uint32_t stats_send_time_high;   // basecmd.c

void
command_runtime_software_trip(uint32_t *args)
{
    uint32_t arm_id = args[0];
    uint32_t clock_lo = timer_read_time();
    uint32_t clock_hi = stats_send_time_high + (clock_lo < stats_send_time);
    uint8_t status = 1; // NotArmed default
    (void)kalico_software_trip(arm_id, clock_lo, clock_hi, &status);
    sendf("kalico_software_trip_response arm_id=%u status=%c",
          arm_id, status);
}
DECL_COMMAND(command_runtime_software_trip,
    "runtime_software_trip arm_id=%u");


// Seed the MCU engine's position origin (SET_KINEMATIC_POSITION) so prev_x/y/z
// match the host's commanded position before the first segment, avoiding a
// huge first-segment delta. Positions are Q16.16 fixed-point (mm * 65536).
// Fire-and-forget; the following PushSegment provides sequencing.
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

// Widen the MCU clock in C with command_get_uptime's formula instead of the
// Rust FFI: runtime::stream::clock_sync_respond reads a TIM5-ISR-populated
// seqlock that the host filters as uninitialised in the all-StepTime path.
extern uint32_t stats_send_time;        // basecmd.c
extern uint32_t stats_send_time_high;   // basecmd.c
void
command_runtime_clock_sync_request(uint32_t *args)
{
    uint32_t request_id = args[0];
    // args[1]/args[2] = host_send_time_{lo,hi} — unused; retained on the wire.
    uint32_t low = timer_read_time();
    uint32_t high = stats_send_time_high + (low < stats_send_time);
    sendf(
        "kalico_clock_sync_response request_id=%u mcu_clock_lo=%u mcu_clock_hi=%u",
        request_id, low, high);
}
DECL_COMMAND(command_runtime_clock_sync_request,
    "runtime_clock_sync_request request_id=%u "
    "host_send_time_lo=%u host_send_time_hi=%u");

// Two-stage phase-stepping registration, both before the first
// kalico_configure_axis: register_phase_bus once per bus_id (shared SPI cfg),
// register_phase_motor once per motor (its own CS GPIO — multiple TMC5160s
// share a bus). Non-STM32 hosts return -88.
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

// Param is cs_pin_id, not cs_pin: msgproto resolves any `*_pin` param against
// the pin enumeration, which would force symbolic pin names instead of the raw
// GPIO encoding (port*16+pin) the rest of the phase_config surface uses.
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

