#ifndef __INDX_IR_SENSOR_H__
#define __INDX_IR_SENSOR_H__

extern "C" {
#include "board/internal.h"
}

#include <cstddef>
#include <cstdint>

#include "i2c_bus.h"

#define KELVIN_OFFSET 273.15

extern uint8_t eeprom[32];

struct ir_sensor_raw_reading {
    uint32_t tp_object;
    uint32_t tp_ambient;
};

enum class ir_sensor_error {
    ok,
    no_radiance,
};

struct ir_sensor_reading {
    ir_sensor_error error;
    float object_temperature;
    float sensor_ambient_temperature;
    ir_sensor_raw_reading raw;

    static ir_sensor_reading
    new_error(ir_sensor_error error) {
        // Return high temperature for errors as that would make any calculated
        // error terms negative.
        return {ir_sensor_error::no_radiance, 1000.0f + KELVIN_OFFSET,
                1000.0f + KELVIN_OFFSET, ir_sensor_raw_reading{0, 0}};
    }
};

struct ir_sensor_model {
    ir_sensor_reading
    translate(ir_sensor_raw_reading raw_reading, float ambient_temperature);

    static ir_sensor_model
    from_eeprom(uint8_t (&eeprom)[32]);

    int32_t ptat25;
    float inv_m;
    int32_t U0;
    float inv_k;
    float exponent;
    float inv_exponent;
    float obj_gain;
    float bracket_gain;
    float cal_tobj1;
    float cal_du;

    void
    set_tuning(float exponent, float obj_gain, float bracket_gain);
};

struct ir_sensor {
    bool
    i2c_transfer(uint8_t address, uint8_t *send, size_t send_len, uint8_t *recv,
                 size_t recv_len);

    ir_sensor_raw_reading
    read_sensor_raw();

    ir_sensor_reading
    read_sensor(float ambient_temperature);

    i2c_bus *bus;
    ir_sensor_model model;
};

ir_sensor
ir_sensor_create(i2c_bus *i2c);

#endif
