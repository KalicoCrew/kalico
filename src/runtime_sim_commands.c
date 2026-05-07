// src/runtime_sim_commands.c
//
// Sim-only Klipper command surface (CONFIG_KALICO_SIM=y) for the kalico
// runtime. Diagnostic / fixture / endstop-poke / engine-tick shims used by
// the Renode and host-sim test harnesses. Also hosts the sim-only direct
// drain wake invoked from the TIM5 ISR (see src/stm32/runtime_tick_h7.c).
//
// Spec §4.5: lifted from src/runtime_tick.c so the lifecycle TU is
// portable / sim-free. All wire strings carry the `runtime_sim_` /
// `runtime_load_fixture_curve` prefix; the legacy `kalico_sim_*` /
// `kalico_load_fixture_curve` names were retired with the runtime-tick
// rename pass.

#include <stdint.h>
#include "autoconf.h"
#include "board/gpio.h"           // gpio_in_setup / gpio_in_read
#include "command.h"              // DECL_COMMAND, sendf, output, DECL_CTR
#include "sched.h"                // task_wake, sched_wake_task
#include "kalico_runtime.h"       // FFI export prototypes
#include "generic/runtime_tick.h" // runtime_tick_enable

#if CONFIG_KALICO_RUNTIME
#if CONFIG_KALICO_SIM

extern void *runtime_handle;             // defined in src/runtime_tick.c
extern struct task_wake runtime_drain_wake; // defined in src/runtime_tick.c

// Sim-only direct wake from the TIM5 ISR. Under Renode, the DWT-based
// timer system is best-effort even with the runtime_sim_cyccnt fork
// (timer_set_diff -> SysTick->LOAD interactions are subtle with a
// stepping software counter), so the runtime_drain_timer's 1 kHz cadence
// can be unreliable. Step-6 plan Phase 0 Gate A trace-stream verification
// requires drain to be invoked deterministically while segments are
// retiring; this provides a guaranteed wake path keyed off TIM5 fires.
//
// Throttle: wake every RUNTIME_SIM_DRAIN_PERIOD_TICKS = 40 fires (= once
// per 1 ms at 40 kHz tick rate). sched_wake_task is ISR-safe (sets a
// volatile flag + atomic write).
volatile uint32_t runtime_sim_drain_counter = 0;
#define RUNTIME_SIM_DRAIN_PERIOD_TICKS 40

__attribute__((used, externally_visible))
void
runtime_sim_isr_wake_drain(void)
{
    if (++runtime_sim_drain_counter >= RUNTIME_SIM_DRAIN_PERIOD_TICKS) {
        runtime_sim_drain_counter = 0;
        sched_wake_task(&runtime_drain_wake);
    }
}

// Drain-call counter — bumped from runtime_drain() in src/runtime_tick.c
// (see the `#if CONFIG_KALICO_SIM` increment), surfaced via runtime_sim_diag.
volatile uint32_t runtime_sim_drain_calls = 0;

extern volatile uint32_t runtime_sim_cyccnt;

// Pre-register the gpio_sample output schema for the sim harness.
DECL_CTR("_DECL_OUTPUT "
         "runtime_sim_gpio_sample sample_id=%u pin=%c value=%c");

void
command_runtime_sim_diag(uint32_t *args)
{
    (void)args;
    uint8_t status = runtime_handle ? runtime_handle_status(runtime_handle) : 255;
    int32_t last_err = runtime_handle ? runtime_handle_last_error(runtime_handle) : 0;
    uint32_t tick_counter = runtime_handle ? runtime_handle_tick_counter(runtime_handle) : 0;
    sendf(
        "runtime_sim_diag_response drain_calls=%u cyccnt=%u drain_counter=%u "
        "status=%c last_err=%i tick_counter=%u",
        runtime_sim_drain_calls, runtime_sim_cyccnt, runtime_sim_drain_counter,
        status, last_err, tick_counter);
}
DECL_COMMAND(command_runtime_sim_diag, "runtime_sim_diag");

void
command_runtime_sim_gpio_sample(uint32_t *args)
{
    uint32_t sample_id = args[0];
    uint8_t pin = args[1];
    uint8_t pull_up = args[2];
    struct gpio_in g = gpio_in_setup(pin, pull_up);
    uint8_t value = gpio_in_read(g);

    sendf("runtime_sim_gpio_sample_response sample_id=%u pin=%c value=%c",
          sample_id, pin, value);
    output("runtime_sim_gpio_sample sample_id=%u pin=%c value=%c",
           sample_id, pin, value);
}
DECL_COMMAND(command_runtime_sim_gpio_sample,
    "runtime_sim_gpio_sample sample_id=%u pin=%c pull_up=%c");

// Phase 4 step-count diagnostic: returns the cumulative step count for the
// given stepper oid (0-indexed). Used by the sim test harness to verify that
// G1 moves produce real step pulses without needing GPIO state readback.
void
command_runtime_sim_stepper_count_query(uint32_t *args)
{
    uint8_t oid = (uint8_t)args[0];
    int32_t count = runtime_handle
        ? kalico_runtime_get_stepper_count(runtime_handle, oid)
        : 0;
    sendf("runtime_sim_stepper_count_response oid=%c count=%i", oid, count);
}
DECL_COMMAND(command_runtime_sim_stepper_count_query,
    "runtime_sim_stepper_count_query oid=%c");

// Phase 4 diagnostic: returns the configured steps_per_mm for axis `oid`
// (motor space, 0..=3). Used to verify that ConfigureAxes actually wrote
// the motor blob into the engine.
void
command_runtime_sim_axis_steps_query(uint32_t *args)
{
    uint8_t oid = (uint8_t)args[0];
    float spm = runtime_handle
        ? runtime_handle_get_axis_steps_per_mm(runtime_handle, oid)
        : 0.0f;
    // Send as i32 micro-steps-per-mm so we don't have to teach the wire
    // codec about f32 here.
    int32_t milli = (int32_t)(spm * 1000.0f);
    sendf("runtime_sim_axis_steps_response oid=%c milli_spm=%i", oid, milli);
}
DECL_COMMAND(command_runtime_sim_axis_steps_query,
    "runtime_sim_axis_steps_query oid=%c");

void
command_runtime_sim_axis_accum_query(uint32_t *args)
{
    uint8_t oid = (uint8_t)args[0];
    double a = runtime_handle
        ? kalico_runtime_get_axis_accumulator(runtime_handle, oid)
        : 0.0;
    int32_t milli = (int32_t)(a * 1000.0);
    sendf("runtime_sim_axis_accum_response oid=%c milli=%i", oid, milli);
}
DECL_COMMAND(command_runtime_sim_axis_accum_query,
    "runtime_sim_axis_accum_query oid=%c");

// Sim-only escape hatch (Step-6 plan Phase 0 Task 0.2). Diagnoses the
// load_curve hang in Renode (the H7 .repl ignores SCB->CPACR writes from
// SystemInit, so any FPU instruction in CurvePool::load — including
// is_finite() and > 0.0 checks — UsageFaults). The fixture path uses static
// pre-validated curve data and CurvePool::load_unchecked (integer-only
// memcpy), bypassing the FPU entirely. NEVER include in production.
extern int32_t kalico_runtime_load_fixture(
    void *rt, uint16_t slot, uint16_t fixture_id, uint32_t *out_handle_packed);

void
command_runtime_load_fixture_curve(uint32_t *args)
{
    if (!runtime_handle) {
        sendf("runtime_load_fixture_response result=%i curve_handle_packed=%u",
              -7, 0);
        return;
    }
    uint16_t slot = args[0];
    uint16_t fixture_id = args[1];
    uint32_t handle_packed = 0;
    int32_t r = kalico_runtime_load_fixture(
        runtime_handle, slot, fixture_id, &handle_packed);
    sendf("runtime_load_fixture_response result=%i curve_handle_packed=%u",
          r, handle_packed);
}
DECL_COMMAND(command_runtime_load_fixture_curve,
    "runtime_load_fixture_curve slot=%hu fixture_id=%hu");

// Step 7-D §10 Renode endstop e2e test scaffold. Production firmware does
// not yet wire real MCU GPIO sampling into `endstop::set_pin_level`
// (rust/runtime/src/endstop.rs:311) — that abstract-pin-level table is
// only addressable from the runtime crate and tests. The e2e test pokes
// it directly through this sim-only shim instead of driving a real GPIO
// in Renode. NEVER include in production firmware.
extern int32_t kalico_endstop_set_pin_level(uint16_t gpio, uint8_t level);

void
command_runtime_sim_endstop_set_pin(uint32_t *args)
{
    uint16_t gpio = args[0];
    uint8_t level = args[1];
    int32_t r = kalico_endstop_set_pin_level(gpio, level);
    sendf("runtime_sim_endstop_set_pin_response gpio=%hu level=%c result=%i",
          gpio, level, r);
}
DECL_COMMAND(command_runtime_sim_endstop_set_pin,
    "runtime_sim_endstop_set_pin gpio=%hu level=%c");

// Step 7-D §10 Renode endstop e2e: TIM5 (40 kHz modulation timer) is not
// enabled until the first segment push triggers the producer protocol
// (`runtime_tick_init` only configures it; `runtime_tick_enable`
// starts it). The endstop e2e test never pushes segments — it just
// arms, asserts a pin, expects a trip. Without TIM5 ticking, the
// modulation ISR never invokes `endstop::tick` and the trip never
// fires. This sim-only shim drives `runtime_tick_enable` directly so
// the test can run the engine in steady-state without a segment.
void
command_runtime_sim_engine_tick_start(uint32_t *args)
{
    (void)args;
    runtime_tick_enable();
    sendf("runtime_sim_engine_tick_start_response result=%i", 0);
}
DECL_COMMAND(command_runtime_sim_engine_tick_start,
    "runtime_sim_engine_tick_start");

#if CONFIG_MACH_LINUX

#include <string.h>
#include "linux/sim_chip_socket.h"

// Weak stub so the file links before Task 5.1 lands the real definition in
// src/linux/analog.c. When Task 5.1 lands, the strong symbol there overrides
// this automatically.
__attribute__((weak)) void
analog_set_simulated_value(uint8_t adc_pin, uint16_t value)
{
    (void)adc_pin;
    (void)value;
}

void
command_runtime_sim_route_spi(uint32_t *args)
{
    uint32_t bus = args[0];
    uint8_t path_len = args[1];
    char path[128] = {0};
    if (path_len >= sizeof(path)) shutdown("sim_route_spi path too long");
    memcpy(path, command_decode_ptr(args[2]), path_len);
    sim_spi_register_route(bus, path);
    sendf("runtime_sim_route_spi_response bus=%u result=%i", bus, 0);
}
DECL_COMMAND(command_runtime_sim_route_spi,
    "runtime_sim_route_spi bus=%u path=%*s");

void
command_runtime_sim_route_tmcuart(uint32_t *args)
{
    uint8_t oid = args[0];
    uint8_t path_len = args[1];
    char path[128] = {0};
    if (path_len >= sizeof(path)) shutdown("sim_route_tmcuart path too long");
    memcpy(path, command_decode_ptr(args[2]), path_len);
    sim_tmcuart_register_route(oid, path);
    sendf("runtime_sim_route_tmcuart_response oid=%c result=%i", oid, 0);
}
DECL_COMMAND(command_runtime_sim_route_tmcuart,
    "runtime_sim_route_tmcuart oid=%c path=%*s");

void
command_runtime_sim_adc_set(uint32_t *args)
{
    uint8_t adc_pin = args[0];
    uint16_t value = args[1];
    analog_set_simulated_value(adc_pin, value);
    sendf("runtime_sim_adc_set_response adc_pin=%c result=%i", adc_pin, 0);
}
DECL_COMMAND(command_runtime_sim_adc_set,
    "runtime_sim_adc_set adc_pin=%c value=%hu");

#endif // CONFIG_MACH_LINUX

#endif // CONFIG_KALICO_SIM
#endif // CONFIG_KALICO_RUNTIME
