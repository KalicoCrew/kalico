#ifndef __INDX_HEATER_H__
#define __INDX_HEATER_H__

#include "indx/coil_driver.h"
#include "indx/i2c_bus.h"
#include "indx/ir_sensor.h"
#include "indx/pid.h"
#include "indx/status_led_driver.h"
#include "sched.h"

struct indx_heater_state {
    float ambient_temperature;
    ir_sensor_reading nozzle_temperature;
    uint32_t update_count{0};
    uint32_t max_power_cycles{0};
    float power_sum{0.0f};
    uint32_t updates_since_report{0};
};

struct indx_heater {

    void
    run();

    bool
    is_timed_out();

    void
    report_temperature(uint32_t clock);

    void
    shutdown_();

    i2c_bus *i2c;
    ir_sensor nozzle_temp_sensor;
    coil_driver coil_driver_inst;
    pid_controller pid;

    status_led_driver status_led;
    const pattern_t *status_pattern{nullptr};
    uint32_t cold_timeout{0};

    float update_interval;
    uint32_t update_interval_ticks;
    timer update_timer;
    bool want_update;
    bool debug_stream_raw_ir{false};

    timer timeout_timer;
    timer liveness_timer;

    uint32_t next_temperature_report;
    uint32_t temperature_report_interval;

    indx_heater_state state;
};

struct indx_heater
indx_heater_create();

#endif
