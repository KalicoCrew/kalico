
extern "C" {
#include "board/internal.h"
#include "command.h"
#include "generic/misc.h"
#include "sched.h"
}

#include "i2c_bus.h"

i2c_bus::i2c_bus(SercomI2cm *sercom, uint32_t sercom_gclk, uint32_t sercom_id)
    : si(sercom) {
    enable_pclock(SERCOM4_GCLK_ID_CORE, ID_SERCOM4);

    // Reset and disable
    si->CTRLA.bit.SWRST = 1;
    while (si->SYNCBUSY.bit.SWRST)
        ;

    // 400 kHz fast mode
    si->CTRLA.reg = SERCOM_I2CM_CTRLA_LOWTOUTEN |
                    SERCOM_I2CM_CTRLA_INACTOUT(3) |
                    SERCOM_I2CM_CTRLA_SPEED(0x0) | SERCOM_I2CM_CTRLA_MODE(0x5);
    uint32_t pfreq = get_pclock_frequency(SERCOM4_GCLK_ID_CORE);
    uint32_t baud = (pfreq / 400000 - 10 - pfreq * 125 / 1000000000) / 2;
    si->BAUD.reg = baud;

    // Enable bus
    si->CTRLA.bit.ENABLE = 1;
    while (si->SYNCBUSY.bit.ENABLE)
        ;

    // Go to idle mode
    si->STATUS.bit.BUSSTATE = 0x1;
    while (si->SYNCBUSY.bit.SYSOP)
        ;
}

static bool
i2c_wait(SercomI2cm *si) {
    auto deadline = timer_read_time() + timer_from_us(500);
    while (timer_is_before(timer_read_time(), deadline)) {
        uint32_t intflag = si->INTFLAG.reg;
        if (!(intflag & SERCOM_I2CM_INTFLAG_MB)) {
            if (si->STATUS.reg & SERCOM_I2CM_STATUS_BUSERR) {
                return false;
            }
            continue;
        }
        if (intflag & SERCOM_I2CM_INTFLAG_ERROR) {
            return false;
        }
        return true;
    }
    output("i2c timeout");
    return false;
}

static void
i2c_start(SercomI2cm *si, uint8_t address, bool read) {
    if (si->STATUS.bit.BUSSTATE == 0x3) {
        shutdown("i2c bus was busy");
    }
    si->ADDR.reg = (address << 1) | (read & 0x1);
}

noinline static void
i2c_end(SercomI2cm *si) {
    si->CTRLB.reg = SERCOM_I2CM_CTRLB_CMD(0x3);
    while (si->SYNCBUSY.bit.SYSOP)
        ;
}

noinline static bool
i2c_send(SercomI2cm *si, uint8_t *send, size_t len) {
    for (size_t i = 0; i < len; i++, send++) {
        si->DATA.reg = *send;
        if (!i2c_wait(si)) {
            return false;
        }
    }
    return true;
}

noinline static bool
i2c_recv(SercomI2cm *si, uint8_t *buffer, size_t len) {
    while (len--) {
        auto deadline = timer_read_time() + timer_from_us(500);
        while (!(si->INTFLAG.bit.SB) &&
               timer_is_before(timer_read_time(), deadline))
            ;
        if (!(si->INTFLAG.bit.SB)) {
            i2c_end(si);
            shutdown("timeout");
            return false;
        }

        if (len) {
            // ACK
            si->CTRLB.bit.ACKACT = 0;
            while (si->SYNCBUSY.bit.SYSOP)
                ;

            // Do read
            si->CTRLB.bit.CMD = 0x2;
            while (si->SYNCBUSY.bit.SYSOP)
                ;
        } else {
            // NACK
            si->CTRLB.bit.ACKACT = 1;
            while (si->SYNCBUSY.bit.SYSOP)
                ;

            // Issue stop
            si->CTRLB.bit.CMD = 0x3;
            while (si->SYNCBUSY.bit.SYSOP)
                ;
        }

        *buffer++ = si->DATA.reg;
    }

    return true;
}

bool
i2c_bus::transfer(uint8_t address, uint8_t *send, size_t send_len,
                  uint8_t *recv, size_t recv_len) {

    if (send_len) {
        i2c_start(si, address, false);
        if (!i2c_wait(si)) {
            i2c_end(si);
            return false;
        }
        if (!i2c_send(si, send, send_len)) {
            i2c_end(si);
            return false;
        }
    }

    if (recv_len == 0) {
        if (send_len) {
            i2c_end(si);
        }
        return true;
    }

    i2c_start(si, address, true);
    return i2c_recv(si, recv, recv_len);
}
