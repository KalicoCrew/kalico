#include <stdint.h>
#include <stdio.h>
#include "autoconf.h"
#include "board/gpio.h"
#include "command.h"
#include "sched.h"
#include "board/misc.h"
#include "kalico_runtime.h"
#include "kalico_dispatch.h"
#if CONFIG_MACH_STM32
#include "stm32/phase_stepping_spi.h"
#elif CONFIG_MACH_LINUX
#include "linux/phase_stepping_spi.h"
#endif


extern void *runtime_handle;

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

// Must match runtime::endstop::MAX_SOURCES.
#define KALICO_ENDSTOP_MAX_SOURCES 4
#define KALICO_ENDSTOP_SOURCE_RECORD_LEN 11
struct endstop_pin_slot {
    uint8_t        active;
    uint16_t       gpio_id;
    struct gpio_in pin;
};
static struct endstop_pin_slot endstop_pin_table[KALICO_ENDSTOP_MAX_SOURCES];

extern int32_t kalico_endstop_set_pin_level(uint16_t gpio, uint8_t level);

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

void
runtime_endstop_sample_pins(void)
{
    sample_endstops_for_stepper(0);
}

// Must match MAX_STEPPER_OIDS_C (runtime_tick.c) / runtime::state::MAX_STEPPER_OIDS.
#define KALICO_MAX_STEPPER_OIDS 8

__attribute__((used, visibility("default")))
void
runtime_endstop_sample_one(uint8_t stepper_idx)
{
    if (stepper_idx >= KALICO_MAX_STEPPER_OIDS)
        return;
    sample_endstops_for_stepper(stepper_idx);
}

static void
endstop_pin_table_clear(void)
{
    for (int i = 0; i < KALICO_ENDSTOP_MAX_SOURCES; i++)
        endstop_pin_table[i].active = 0;
}

enum {
    ENDSTOP_KIND_PHYSICAL = 0,
    ENDSTOP_KIND_TMC_DIAG = 1,
    ENDSTOP_KIND_SOFTWARE = 2,
};

// Record byte layout mirrors rust/kalico-c-api/src/runtime_ffi.rs
// kalico_endstop_arm decode: kind u8, gpio u16 LE, active_high u8, policy u8,
// sample_n u8, velocity_axis u8, v_min_q16 u32 LE.
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
        uint8_t kind = r[0];
        if (kind == ENDSTOP_KIND_SOFTWARE)
            continue;
        uint16_t gpio_id = (uint16_t)r[1] | ((uint16_t)r[2] << 8);
        // TMC DIAG is open-drain (GCONF.diag1_int_pushpull==0 at reset) and
        // floats LOW without a pullup, reading asserted at idle. The host's
        // pullup flag is not on the wire yet, so pull up TmcDiag here.
        int32_t pull_up = (kind == ENDSTOP_KIND_TMC_DIAG) ? 1 : 0;
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
    // %*s args carry an encoded pointer; a bare cast segfaults on 64-bit sim.
    uint8_t *sources_ptr = command_decode_ptr(args[5]);
    uint8_t stepper_count = args[6];
    uint32_t steppers_len = args[7];
    uint8_t *steppers_ptr = command_decode_ptr(args[8]);
    uint8_t status = 2;
    uint64_t software_grant_ticks = 0;
    if (sources_ptr && source_count > 0) {
        for (uint8_t i = 0; i < source_count; i++) {
            uint8_t kind = sources_ptr[(uint32_t)i * KALICO_ENDSTOP_SOURCE_RECORD_LEN];
            if (kind == ENDSTOP_KIND_SOFTWARE) {
                software_grant_ticks = (uint64_t)CONFIG_CLOCK_FREQ * 50 / 1000;
                break;
            }
        }
    }
    (void)kalico_endstop_arm(arm_id, arm_clock_lo, arm_clock_hi,
                             source_count, sources_ptr, sources_len,
                             stepper_count, steppers_ptr, steppers_len,
                             (uint32_t)software_grant_ticks,
                             (uint32_t)(software_grant_ticks >> 32),
                             &status);
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
    uint8_t status = 2;
    (void)kalico_endstop_disarm(arm_id, &status);
    endstop_pin_table_clear();
    sendf("kalico_disarm_endstop_response arm_id=%u status=%c", arm_id, status);
}
DECL_COMMAND(command_runtime_disarm_endstop, "runtime_disarm_endstop arm_id=%u");

extern uint32_t stats_send_time;
extern uint32_t stats_send_time_high;

void
command_runtime_software_trip(uint32_t *args)
{
    uint32_t arm_id = args[0];
    uint32_t clock_lo = timer_read_time();
    uint32_t clock_hi = stats_send_time_high + (clock_lo < stats_send_time);
    uint8_t status = 1;
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

extern uint32_t stats_send_time;
extern uint32_t stats_send_time_high;
void
command_runtime_clock_sync_request(uint32_t *args)
{
    uint32_t request_id = args[0];
    // host_send_time_{lo,hi} (args[1]/[2]) are unused but retained on the wire.
    uint32_t low = timer_read_time();
    uint32_t high = stats_send_time_high + (low < stats_send_time);
    sendf(
        "kalico_clock_sync_response request_id=%u mcu_clock_lo=%u mcu_clock_hi=%u",
        request_id, low, high);
}
DECL_COMMAND(command_runtime_clock_sync_request,
    "runtime_clock_sync_request request_id=%u "
    "host_send_time_lo=%u host_send_time_hi=%u");

enum { TMC_SPI_MODE = 3 };

void
command_runtime_register_phase_bus(uint32_t *args)
{
#if CONFIG_MACH_STM32 || CONFIG_MACH_LINUX
    uint8_t bus_id = (uint8_t)args[0];
    uint32_t rate = args[1];
    struct spi_config cfg = spi_setup(bus_id, TMC_SPI_MODE, rate);
    phase_stepping_register_bus(bus_id, cfg);
    sendf("kalico_register_phase_bus_response result=%i", 0);
#else
    (void)args;
    sendf("kalico_register_phase_bus_response result=%i", -88);
#endif
}
DECL_COMMAND(command_runtime_register_phase_bus,
    "runtime_register_phase_bus bus_id=%c rate=%u");

// Wire param must stay cs_pin_id, not cs_pin: msgproto resolves any `*_pin`
// param against the pin enumeration, breaking the raw port*16+pin GPIO encoding.
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

