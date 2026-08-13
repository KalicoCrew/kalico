extern "C" {
#include "board/internal.h"
#include "command.h"
#include "generic/misc.h"
#include "math.h"
#include "sched.h"
}

#include <cstdint>

#include "ir_sensor.h"

#define SENSOR_ADDRESS 12

// Default calibration constants, relative to the ones provided in the sensor
// EEPROM.
static constexpr float IR_EXPONENT = 3.9916f;
static constexpr float IR_OBJ_GAIN = 2.2280f;
static constexpr float IR_BRACKET_GAIN = -2.4739f;

static void
delay(uint32_t us) {
    uint32_t end = timer_read_time() + timer_from_us(us);
    while (timer_is_before(timer_read_time(), end))
        ;
}

uint8_t eeprom[32];

ir_sensor
ir_sensor_create(i2c_bus *i2c) {

    ir_sensor sensor{.bus = i2c, .model = ir_sensor_model()};

    // Initialize sensor
    {
        uint8_t buf[] = {0x04};
        if (!sensor.i2c_transfer(0, buf, sizeof(buf), nullptr, 0)) {
            shutdown("IR sensor didn't reply to general call");
        }
    }

    for (auto attempts = 4; attempts--;) {
        delay(100);

        {
            uint8_t buf[] = {0x1F, 0x80};
            // Configure ECR for readout
            if (!sensor.i2c_transfer(SENSOR_ADDRESS, buf, sizeof(buf), nullptr,
                                     0)) {
                if (attempts == 0) {
                    shutdown("Could not read IR sensor EEPROM");
                } else {
                    continue;
                }
            }
        }

        {
            uint8_t buf[] = {0x20};
            if (!sensor.i2c_transfer(SENSOR_ADDRESS, buf, sizeof(buf), eeprom,
                                     sizeof(eeprom))) {

                if (attempts == 0) {
                    shutdown("Could not read IR sensor EEPROM");
                } else {
                    continue;
                }
            }
        }

        {
            uint8_t buf[] = {0x1F, 0x00};
            // Disable ECR for readout. Ignore result.
            sensor.i2c_transfer(SENSOR_ADDRESS, buf, sizeof(buf), nullptr, 0);
        }

        uint16_t actual_checksum = eeprom[0];
        for (size_t i = 3; i < sizeof(eeprom); i++) {
            actual_checksum += eeprom[i];
        }
        uint16_t expected_checksum =
            ((uint16_t)eeprom[1] << 8) | (uint16_t)eeprom[2];

        if (!(eeprom[0] == 3 && eeprom[31] == (SENSOR_ADDRESS | 0x80) &&
              actual_checksum == expected_checksum)) {

            if (attempts == 0) {
                shutdown("IR sensor EEPROM is invalid");
            } else {
                continue;
            }
        }

        sensor.model = ir_sensor_model::from_eeprom(eeprom);
        break;
    }
    return sensor;
}

bool
ir_sensor::i2c_transfer(uint8_t address, uint8_t *send, size_t send_len,
                        uint8_t *recv, size_t recv_len) {
    return this->bus->transfer(address, send, send_len, recv, recv_len);
}

ir_sensor_raw_reading
ir_sensor::read_sensor_raw() {
    uint8_t send[] = {0x01};
    uint8_t recv[4];
    bool read = false;
    for (auto i = 0; i < 5; i++) {
        if (this->i2c_transfer(SENSOR_ADDRESS, send, sizeof(send), recv,
                               sizeof(recv))) {
            read = true;
            break;
        }
    }
    if (!read)
        shutdown("Could not read IR sensor value");

    uint32_t tp_object = ((uint32_t)recv[0] << 9) | ((uint32_t)recv[1] << 1) |
                         ((uint32_t)recv[2] >> 7);
    uint32_t t_ambient = ((uint32_t)recv[2] & 0x7F) << 8 | (uint32_t)recv[3];

    return ir_sensor_raw_reading{tp_object, t_ambient};
}

ir_sensor_reading
ir_sensor::read_sensor(float ambient_temperature) {
    auto raw_reading = this->read_sensor_raw();
    return this->model.translate(raw_reading, ambient_temperature);
}

ir_sensor_model
ir_sensor_model::from_eeprom(uint8_t (&eeprom)[32]) {

    uint16_t m = ((uint16_t)eeprom[12] << 8) | (uint16_t)eeprom[13];
    int32_t u0 = (((uint32_t)eeprom[14] << 8) | (uint32_t)eeprom[15]) + 32768L;
    int32_t uout1 = (((uint32_t)eeprom[16] << 8) | (uint32_t)eeprom[17]) * 2;

    ir_sensor_model model{};
    model.ptat25 = ((uint32_t)eeprom[10] << 8) | (uint32_t)eeprom[11];
    model.inv_m = 100.0f / (float)m;
    model.U0 = u0;
    model.cal_tobj1 = (float)eeprom[18];
    model.cal_du = (float)(uout1 - u0);
    // Sets exponent/obj_gain/bracket_gain and derives inv_k from the cal point.
    model.set_tuning(IR_EXPONENT, IR_OBJ_GAIN, IR_BRACKET_GAIN);
    return model;
}

void
ir_sensor_model::set_tuning(float exponent, float obj_gain,
                            float bracket_gain) {
    this->exponent = exponent;
    this->inv_exponent = 1.0f / exponent;
    this->obj_gain = obj_gain;
    this->bracket_gain = bracket_gain;
    this->inv_k = (powf(this->cal_tobj1 + KELVIN_OFFSET, exponent) -
                   powf(25.0f + KELVIN_OFFSET, exponent)) /
                  this->cal_du;
}

ir_sensor_reading
ir_sensor_model::translate(ir_sensor_raw_reading raw_reading,
                           float ambient_temp) {

    // Calculate sensor ambient temperature, see datasheet section 9.4
    float sensor_ambient_temperature =
        (25.0 + KELVIN_OFFSET) +
        ((float)((int32_t)raw_reading.tp_ambient - this->ptat25)) * this->inv_m;

    // Calculate radiances
    float rad_ambient = powf(sensor_ambient_temperature, this->exponent);
    float rad_received = this->obj_gain *
                         ((float)((int32_t)raw_reading.tp_object - this->U0)) *
                         this->inv_k;
    float rad_bracket =
        this->bracket_gain * (powf(ambient_temp, this->exponent) - rad_ambient);

    float object_radiance = rad_ambient + rad_received + rad_bracket;

    if (object_radiance < 0.0f) {
        return ir_sensor_reading::new_error(ir_sensor_error::no_radiance);
    }

    // Convert sum of radiances to temperature
    float object_temperature = powf(object_radiance, this->inv_exponent);

    // If object temperature is below ambient temperature, take the mean
    // instead.
    if (object_temperature < sensor_ambient_temperature) {
        object_temperature =
            (object_temperature + sensor_ambient_temperature) / 2.0f;
    }

    return ir_sensor_reading{ir_sensor_error::ok, object_temperature,
                             sensor_ambient_temperature, raw_reading};
}
