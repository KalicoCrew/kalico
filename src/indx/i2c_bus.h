#ifndef __INDX_I2C_BUS_H__
#define __INDX_I2C_BUS_H__

extern "C" {
#include "board/internal.h"
}

#include <cstddef>
#include <cstdint>

struct i2c_bus {
    i2c_bus(SercomI2cm *sercom, uint32_t sercom_gclk, uint32_t sercom_id);

    bool
    transfer(uint8_t address, uint8_t *send, size_t send_len, uint8_t *recv,
             size_t recv_len);

  private:
    SercomI2cm *si;
};

#endif
