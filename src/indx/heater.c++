#include "indx/status_led_driver.h"
extern "C" {
#include "atsamd/gpio.h"
#include "command.h"
#include "generic/misc.h"
#include "sched.h"
}

#include "coil_driver.h"
#include "heater.h"
#include "ir_sensor.h"

#include <cmath>
#include <optional>

#define PIN_SDA GPIO('A', 13)
#define PIN_SCL GPIO('A', 12)
DECL_CONSTANT_STR("RESERVE_PINS_INDX", "PA13,PA12");

#define LIVENESS_CHECK_INTERVAL CONFIG_CLOCK_FREQ
#define HEATER_TIMEOUT_US 3000000.0f
#define UPDATE_RATE_HZ 100
#define HEATER_MAX_POWER_CYCLES (UPDATE_RATE_HZ * 15)
#define HEATER_MAX_TEMPERATURE (KELVIN_OFFSET + 315.0)

void
indx_heater::run() {
    auto now = timer_read_time();

    float output = 0.0;

    this->state.nozzle_temperature =
        this->nozzle_temp_sensor.read_sensor(this->state.ambient_temperature);

    switch (this->state.nozzle_temperature.error) {
    case ir_sensor_error::ok:
        if (this->state.nozzle_temperature.object_temperature >
            HEATER_MAX_TEMPERATURE) {
            shutdown("Heater maximum safe temperature passed");
        }

        output =
            this->pid.step(this->state.nozzle_temperature.object_temperature,
                           this->update_interval);
        break;
    case ir_sensor_error::no_radiance:
        if (this->pid.set_point.has_value()) {
            shutdown("IR sensor calculated no radiance");
            return;
        }
        break;
    }

    if (output >= 1.0f) {
        if (++this->state.max_power_cycles >= HEATER_MAX_POWER_CYCLES &&
            this->coil_driver_inst.duty_limit == 0) {
            shutdown("Heater was at maximum power for too long");
            return;
        }
    } else {
        this->state.max_power_cycles = 0;
    }
    this->coil_driver_inst.set_duty(output);

    this->state.power_sum += output;
    this->state.updates_since_report += 1;
    if (timer_is_before(this->next_temperature_report, now)) {
        this->report_temperature(now);
        this->next_temperature_report += this->temperature_report_interval;
    }

    // Determine status pattern to use

    auto new_pattern = &pattern_off;

    if (this->pid.set_point.has_value()) {
        auto error = this->pid.set_point.value() -
                     this->state.nozzle_temperature.object_temperature;
        if (error > 1.0f) {
            new_pattern = &pattern_heating;
        } else {
            new_pattern = &pattern_target;
        }
        // Reset the cold timeout so we can show the hot->cold transition
        // effect when reaching cold.
        this->cold_timeout = 100 * 60;
    } else {
        auto cold = this->state.nozzle_temperature.object_temperature <
                    (KELVIN_OFFSET + 50.0f);
        if (!cold) {
            new_pattern = &pattern_cooling_down;
        } else {
            if (this->cold_timeout > 0) {
                this->cold_timeout--;
                new_pattern = &pattern_cooled_down;
            } else {
                new_pattern = &pattern_cold;
            }
        }
    }

    if (new_pattern != this->status_pattern) {
        this->status_pattern = new_pattern;
        this->status_led.set_pattern(*new_pattern);
    }

    // Update status LED after the main PID loop, as it's less timing sensitive.
    // This also ensures that a status LED update doesn't hold back IR sensor
    // readout.
    this->status_led.tick();

    this->state.update_count += 1;
}

void
indx_heater::report_temperature(uint32_t clock) {
    auto updates_f = (float)this->state.updates_since_report;
    auto power = this->state.power_sum / updates_f;

    sendf("indx_nozzle_temp clock=%u nozzle=%u sensor=%u power=%u charge=%u "
          "overvoltage=%u",
          clock,
          (int)(this->state.nozzle_temperature.object_temperature * 100.0),
          (int)(this->state.nozzle_temperature.sensor_ambient_temperature *
                100.0),
          (int)(power * 1000000.0), this->coil_driver_inst.get_total_charge(),
          this->coil_driver_inst.get_ov_count());

    if (this->debug_stream_raw_ir) {
        sendf("indx_debug_raw_ir object=%u ambient=%u",
              this->state.nozzle_temperature.raw.tp_object,
              this->state.nozzle_temperature.raw.tp_ambient);
    }

    this->state.power_sum = 0.0f;
    this->state.updates_since_report = 0;
}

void
indx_heater::shutdown_() {
    this->coil_driver_inst.shutdown_();
}

static std::optional<i2c_bus> i2c;
static std::optional<indx_heater> indx_heater_instance;
static task_wake indx_heater_wake;

static uint_fast8_t
indx_heater_update_wake(struct timer *timer) {
    indx_heater_instance->want_update = true;
    sched_wake_task(&indx_heater_wake);
    timer->waketime += indx_heater_instance->update_interval_ticks;
    return SF_RESCHEDULE;
}

static uint_fast8_t
indx_heater_timeout(struct timer *timer) {
    shutdown("INDX heater watchdog timeout");
    return SF_DONE;
}

static uint32_t last_update_count;

static uint_fast8_t
indx_heater_liveness_check(struct timer *timer) {
    auto new_update_count = indx_heater_instance->state.update_count;
    if (new_update_count == last_update_count) {
        shutdown("INDX heater liveness check failed");
    }
    last_update_count = new_update_count;
    timer->waketime += LIVENESS_CHECK_INTERVAL;
    return SF_RESCHEDULE;
}

indx_heater
indx_heater_create(i2c_bus *bus) {
    auto status_led = status_led_driver(bus);
    status_led.set_currents(0xFF, 0xCC, 0xCC);
    auto nozzle_temp_sensor = ir_sensor_create(bus);
    auto coil_driver = coil_driver_create();
    auto pid = pid_controller(pid_params::zero());

    auto update_interval = 1.0f / (float)UPDATE_RATE_HZ;

    return indx_heater{
        .i2c = bus,
        .nozzle_temp_sensor = nozzle_temp_sensor,
        .coil_driver_inst = coil_driver,
        .pid = pid,
        .status_led = status_led,
        .status_pattern = nullptr,

        .update_interval = update_interval,
        .update_interval_ticks = timer_from_us(update_interval * 1000000.0f),
        .update_timer = timer{.next = nullptr, .func = indx_heater_update_wake},
        .want_update = true,

        .timeout_timer = timer{.next = nullptr, .func = indx_heater_timeout},
        .liveness_timer =
            timer{.next = nullptr, .func = indx_heater_liveness_check},

        .next_temperature_report = timer_read_time(),
        .temperature_report_interval = timer_from_us(100000),

        .state =
            indx_heater_state{
                .ambient_temperature = 25.0 + KELVIN_OFFSET,
            },
    };
}

static void
indx_init() {
    if (indx_heater_instance)
        shutdown("INDX heater already active");

    gpio_peripheral(PIN_SDA, 'D', 0);
    gpio_peripheral(PIN_SCL, 'D', 0);

    i2c = i2c_bus(&SERCOM4->I2CM, SERCOM4_GCLK_ID_CORE, ID_SERCOM4);

    indx_heater_instance = indx_heater_create(&i2c.value());

    indx_heater_instance->update_timer.waketime =
        timer_read_time() + indx_heater_instance->update_interval_ticks;
    sched_add_timer(&indx_heater_instance->update_timer);
    sched_wake_task(&indx_heater_wake);

    indx_heater_instance->liveness_timer.waketime =
        timer_read_time() + LIVENESS_CHECK_INTERVAL;
    sched_add_timer(&indx_heater_instance->liveness_timer);
}

extern "C" void
command_indx_set_target(uint32_t *args) {
    if (!indx_heater_instance)
        return;

    auto set_point = *reinterpret_cast<float *>(&args[0]);
    if (std::isnan(set_point)) {
        indx_heater_instance->pid.set_point = std::nullopt;
        sched_del_timer(&indx_heater_instance->timeout_timer);
    } else {
        indx_heater_instance->pid.set_point = set_point;
        indx_heater_instance->timeout_timer.waketime =
            timer_read_time() + timer_from_us(HEATER_TIMEOUT_US);
        sched_del_timer(&indx_heater_instance->timeout_timer);
        sched_add_timer(&indx_heater_instance->timeout_timer);
    }
}
DECL_COMMAND(command_indx_set_target, "indx_set_target target=%u");

extern "C" void
command_indx_set_bracket_temp(uint32_t *args) {
    if (!indx_heater_instance)
        return;

    auto temp = *reinterpret_cast<float *>(&args[0]);
    if (std::isnan(temp)) {
        indx_heater_instance->state.ambient_temperature =
            KELVIN_OFFSET + 25.0; // Fallback value
    } else {
        indx_heater_instance->state.ambient_temperature = temp;
    }
}
DECL_COMMAND(command_indx_set_bracket_temp, "indx_set_bracket_temp temp=%u");

extern "C" void
command_indx_set_ir_sensor_params(uint32_t *args) {
    if (!indx_heater_instance)
        return;

    auto exponent = *reinterpret_cast<float *>(&args[0]);
    auto obj_gain = *reinterpret_cast<float *>(&args[1]);
    auto bracket_gain = *reinterpret_cast<float *>(&args[2]);
    if (!std::isnan(exponent) && !std::isnan(obj_gain) &&
        !std::isnan(bracket_gain)) {
        indx_heater_instance->nozzle_temp_sensor.model.set_tuning(
            exponent, obj_gain, bracket_gain);
    }
}
DECL_COMMAND(
    command_indx_set_ir_sensor_params,
    "indx_set_ir_sensor_params exponent=%u obj_gain=%u bracket_gain=%u");

extern "C" void
command_indx_set_control_params(uint32_t *args) {
    if (!indx_heater_instance)
        return;

    auto kp = *reinterpret_cast<float *>(&args[0]);
    auto ti = *reinterpret_cast<float *>(&args[1]);
    auto td = *reinterpret_cast<float *>(&args[2]);
    auto b = *reinterpret_cast<float *>(&args[3]);
    auto tt = td < 0.000001 ? ti / 8.0f : sqrtf(ti * td);
    auto params = pid_params{
        .kp = kp,
        .ti = ti,
        .td = td,
        .b = b,
        .tt = tt,
    };

    indx_heater_instance->pid.params = params;
}
DECL_COMMAND(command_indx_set_control_params,
             "indx_set_control_params kp=%u ti=%u td=%u b=%u");

extern "C" void
command_indx_set_coil_driver_params(uint32_t *args) {
    if (!indx_heater_instance)
        return;

    auto time_on = *reinterpret_cast<float *>(&args[0]);
    auto time_off = *reinterpret_cast<float *>(&args[1]);
    auto time_on_first = *reinterpret_cast<float *>(&args[2]);
    indx_heater_instance->coil_driver_inst.set_timings(time_on, time_off,
                                                       time_on_first);
}
DECL_COMMAND(
    command_indx_set_coil_driver_params,
    "indx_set_coil_driver_params time_on=%u time_off=%u time_on_first=%u");

extern "C" void
command_indx_cycle_limit(uint32_t *args) {
    if (!indx_heater_instance)
        return;

    indx_heater_instance->coil_driver_inst.set_cycle_limit(args[0]);
}
DECL_COMMAND(command_indx_cycle_limit, "indx_cycle_limit limit=%u");

// Start the on-device coil-driver auto-tune (commissioning). start_tune()
// faults if the coil is being driven; tuning and heating are mutually
// exclusive. Advanced to completion by tune_step() from indx_heater_task.
extern "C" void
command_indx_tune_coil(uint32_t *args) {
    if (!indx_heater_instance)
        return;

    indx_heater_instance->coil_driver_inst.start_tune(args[0] != 0);
}
DECL_COMMAND(command_indx_tune_coil, "indx_tune_coil debug=%c");

// Poll the latched tune outcome (replies indx_coil_tune_result). Reliable
// because the host can re-query; a dropped completion notification is fine.
extern "C" void
command_indx_query_coil_tune(uint32_t *args) {
    (void)args;
    if (!indx_heater_instance)
        return;

    indx_heater_instance->coil_driver_inst.report_tune_status();
}
DECL_COMMAND(command_indx_query_coil_tune, "indx_query_coil_tune");

// Read back the drive timings currently in effect (replies
// indx_coil_driver_params).
extern "C" void
command_indx_query_coil_driver_params(uint32_t *args) {
    (void)args;
    if (!indx_heater_instance)
        return;

    indx_heater_instance->coil_driver_inst.report_params();
}
DECL_COMMAND(command_indx_query_coil_driver_params,
             "indx_query_coil_driver_params");

extern "C" void
command_indx_led_force_color(uint32_t *args) {
    if (!indx_heater_instance)
        return;

    if (args[3]) {
        indx_heater_instance->status_led.force_color(
            color_t(args[0], args[1], args[2]));
    } else {
        indx_heater_instance->status_led.force_color(std::nullopt);
    }
}
DECL_COMMAND(command_indx_led_force_color,
             "indx_led_force_color r=%c g=%c b=%c force=%c");

extern "C" void
command_indx_led_set_current(uint32_t *args) {
    if (!indx_heater_instance)
        return;

    indx_heater_instance->status_led.set_currents(args[0], args[1], args[2]);
}
DECL_COMMAND(command_indx_led_set_current,
             "indx_led_set_current r=%c g=%c b=%c");

extern "C" void
command_indx_debug_stream_raw_ir_sensor(uint32_t *args) {
    if (!indx_heater_instance)
        return;

    indx_heater_instance->debug_stream_raw_ir = args[0] ? true : false;
}
DECL_COMMAND(command_indx_debug_stream_raw_ir_sensor,
             "indx_debug_stream_raw_ir_sensor enable=%c");

extern "C" void
command_indx_debug_ir_sensor_eeprom(uint32_t *args) {
    sendf("indx_debug_ir_sensor_eeprom_data data=%*s", 32, eeprom);
}
DECL_COMMAND(command_indx_debug_ir_sensor_eeprom,
             "indx_debug_ir_sensor_eeprom");

extern "C" void
indx_shutdown(void) {
    if (indx_heater_instance) {
        indx_heater_instance->shutdown_();
        indx_heater_instance->pid.set_point = std::nullopt;

        indx_heater_instance->update_timer.waketime =
            timer_read_time() + indx_heater_instance->update_interval_ticks;
        sched_add_timer(&indx_heater_instance->update_timer);
    }
}
DECL_SHUTDOWN(indx_shutdown);

extern "C" void
indx_heater_task(void) {
    if (!sched_is_shutdown() && !indx_heater_instance) {
        indx_init();
    }
    if (!indx_heater_instance) {
        return;
    }

    // Pump the auto-tuner every scheduler pass while it runs, so bounded test
    // bursts fire back to back. Cheap flag check otherwise.
    auto &coil = indx_heater_instance->coil_driver_inst;
    if (coil.tune_active() && !sched_is_shutdown()) {
        coil.tune_step();
    }

    if (!sched_check_wake(&indx_heater_wake)) {
        return;
    }
    if (indx_heater_instance->want_update) {
        indx_heater_instance->run();
        indx_heater_instance->want_update = false;
    }
}
DECL_TASK(indx_heater_task);
