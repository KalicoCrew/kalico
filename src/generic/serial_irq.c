// Generic interrupt based serial uart helper code
//
// Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memmove
#include "autoconf.h" // CONFIG_SERIAL_BAUD
#include "board/io.h" // readb
#include "board/irq.h" // irq_save
#include "board/misc.h" // console_sendf
#include "board/pgm.h" // READP
#include "command.h" // DECL_CONSTANT
#include "sched.h" // sched_wake_tasks
#include "serial_irq.h" // serial_enable_tx_irq

#define RX_BUFFER_SIZE 192

static uint8_t receive_buf[RX_BUFFER_SIZE], receive_pos;
#if CONFIG_KALICO_RUNTIME
// Sized to fit one full kalico frame (KALICO_TX_BUF_SIZE = 256 in
// src/kalico_dispatch.c) plus headroom for a Klipper response. AVR /
// non-runtime targets keep the original 96-byte buffer + uint8_t pos.
static uint8_t transmit_buf[320];
static uint16_t transmit_pos, transmit_max;
typedef uint16_t transmit_pos_t;
#define read_tpos(a)    readw(a)
#define write_tpos(a,v) writew((a),(v))
#else
static uint8_t transmit_buf[96], transmit_pos, transmit_max;
typedef uint_fast8_t transmit_pos_t;
#define read_tpos(a)    readb(a)
#define write_tpos(a,v) writeb((a),(v))
#endif

DECL_CONSTANT("SERIAL_BAUD", CONFIG_SERIAL_BAUD);
DECL_CONSTANT("RECEIVE_WINDOW", RX_BUFFER_SIZE);

// Rx interrupt - store read data
void
serial_rx_byte(uint_fast8_t data)
{
    if (data == MESSAGE_SYNC)
        sched_wake_tasks();
    if (receive_pos >= sizeof(receive_buf))
        // Serial overflow - ignore it as crc error will force retransmit
        return;
    receive_buf[receive_pos++] = data;
}

// Tx interrupt - get next byte to transmit
int
serial_get_tx_byte(uint8_t *pdata)
{
    if (transmit_pos >= transmit_max)
        return -1;
    *pdata = transmit_buf[transmit_pos++];
    return 0;
}

// Remove from the receive buffer the given number of bytes
static void
console_pop_input(uint_fast8_t len)
{
    uint_fast8_t copied = 0;
    for (;;) {
        uint_fast8_t rpos = readb(&receive_pos);
        uint_fast8_t needcopy = rpos - len;
        if (needcopy) {
            memmove(&receive_buf[copied], &receive_buf[copied + len]
                    , needcopy - copied);
            copied = needcopy;
            sched_wake_tasks();
        }
        irqstatus_t flag = irq_save();
        if (rpos != readb(&receive_pos)) {
            // Raced with irq handler - retry
            irq_restore(flag);
            continue;
        }
        receive_pos = needcopy;
        irq_restore(flag);
        break;
    }
}

// Process any incoming commands
void
console_task(void)
{
    uint_fast8_t rpos = readb(&receive_pos), pop_count;
    int_fast8_t ret = command_find_block(receive_buf, rpos, &pop_count);
    if (ret > 0)
        command_dispatch(receive_buf, pop_count);
    if (ret) {
        if (CONFIG_HAVE_BOOTLOADER_REQUEST && ret < 0 && pop_count == 32
            && !memcmp(receive_buf, " \x1c Request Serial Bootloader!! ~", 32))
            bootloader_request();
        console_pop_input(pop_count);
        if (ret > 0)
            command_send_ack();
    }
}
DECL_TASK(console_task);

// Encode and transmit a "response" message
void
console_sendf(const struct command_encoder *ce, va_list args)
{
    // Verify space for message
    transmit_pos_t tpos = read_tpos(&transmit_pos);
    transmit_pos_t tmax = read_tpos(&transmit_max);
    if (tpos >= tmax) {
        tpos = tmax = 0;
        write_tpos(&transmit_max, 0);
        write_tpos(&transmit_pos, 0);
    }
    uint_fast8_t max_size = READP(ce->max_size);
    if (tmax + max_size > sizeof(transmit_buf)) {
        if (tmax + max_size - tpos > sizeof(transmit_buf))
            // Not enough space for message
            return;
        // Disable TX irq and move buffer
        write_tpos(&transmit_max, 0);
        tpos = read_tpos(&transmit_pos);
        tmax -= tpos;
        memmove(&transmit_buf[0], &transmit_buf[tpos], tmax);
        write_tpos(&transmit_pos, 0);
        write_tpos(&transmit_max, tmax);
        serial_enable_tx_irq();
    }

    // Generate message
    uint8_t *buf = &transmit_buf[tmax];
    uint_fast8_t msglen = command_encode_and_frame(buf, ce, args);

    // Start message transmit
    write_tpos(&transmit_max, tmax + msglen);
    serial_enable_tx_irq();
}

#if CONFIG_KALICO_RUNTIME
// Raw byte writer used by the kalico-native transport TX path. Bytes are
// already a complete kalico frame (sync + len + channel + payload + CRC).
// Returns the number of bytes accepted, or -1 if there is no room. Frame
// contents are not split across calls — caller (kalico_transport_send_frame
// in src/kalico_dispatch.c) hands a single, complete frame.
int
kalico_console_write_raw(const uint8_t *buf, uint16_t len)
{
    transmit_pos_t tpos = read_tpos(&transmit_pos);
    transmit_pos_t tmax = read_tpos(&transmit_max);
    // Compact buffer if there's tail headroom we can reclaim.
    if (tpos && (uint32_t)tmax + (uint32_t)len > sizeof(transmit_buf)) {
        write_tpos(&transmit_max, 0);
        tpos = read_tpos(&transmit_pos);
        tmax -= tpos;
        memmove(&transmit_buf[0], &transmit_buf[tpos], tmax);
        write_tpos(&transmit_pos, 0);
        write_tpos(&transmit_max, tmax);
    }
    if ((uint32_t)tmax + (uint32_t)len > sizeof(transmit_buf))
        return -1;
    memcpy(&transmit_buf[tmax], buf, len);
    write_tpos(&transmit_max, tmax + len);
    serial_enable_tx_irq();
    return len;
}
#endif
