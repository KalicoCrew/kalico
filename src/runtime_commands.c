// Klipper command surface for the kalico runtime.

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


extern void *runtime_handle;
extern uint32_t stats_send_time;        // basecmd.c
extern uint32_t stats_send_time_high;   // basecmd.c

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
// (stats_send_time / stats_send_time_high are externed at the top of the file.)
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

