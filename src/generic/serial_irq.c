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
#include "kalico_demux.h" // kalico_demux_pump
#include "sched.h" // sched_wake_tasks
#include "serial_irq.h" // serial_enable_tx_irq

#if CONFIG_MACH_STM32H7
#define RX_BUFFER_SIZE 2048
__attribute__((section(".axi_bss")))
static uint8_t receive_buf[RX_BUFFER_SIZE];
static uint16_t receive_pos;
typedef uint16_t receive_pos_t;
#define read_rpos(a)    readw(a)
#define write_rpos(a,v) writew((a),(v))
static uint8_t transmit_buf[320];
static uint16_t transmit_pos, transmit_max;
typedef uint16_t transmit_pos_t;
#define read_tpos(a)    readw(a)
#define write_tpos(a,v) writew((a),(v))
#else
#define RX_BUFFER_SIZE 192
static uint8_t receive_buf[RX_BUFFER_SIZE], receive_pos;
typedef uint_fast8_t receive_pos_t;
#define read_rpos(a)    readb(a)
#define write_rpos(a,v) writeb((a),(v))
static uint8_t transmit_buf[320];
static uint16_t transmit_pos, transmit_max;
typedef uint16_t transmit_pos_t;
#define read_tpos(a)    readw(a)
#define write_tpos(a,v) writew((a),(v))
#endif

DECL_CONSTANT("SERIAL_BAUD", CONFIG_SERIAL_BAUD);
DECL_CONSTANT("RECEIVE_WINDOW", RX_BUFFER_SIZE);

// Rx interrupt - store read data
void
serial_rx_byte(uint_fast8_t data)
{
    if (data == MESSAGE_SYNC) {
        sched_wake_tasks();
    } else if (data == 0x55 /* KALICO_FRAME_SYNC */) {
        // Without a wake per kalico frame start, a long frame can fill
        // receive_buf before a Klipper 0x7E sync byte happens to occur in
        // its payload, dropping bytes silently mid-frame.
        sched_wake_tasks();
    } else if (receive_pos > (sizeof(receive_buf) * 3 / 4)) {
        sched_wake_tasks();
    }
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

// Process any incoming commands
void
console_task(void)
{
    receive_pos_t rpos = read_rpos(&receive_pos);

    kalico_demux_pump(receive_buf, rpos);

    // The rebasing memmove must run with IRQs masked: a fresh RX IRQ
    // between reading receive_pos and the move would write into a slot
    // we are relocating.
    irqstatus_t flag = irq_save();
    receive_pos_t now = read_rpos(&receive_pos);
    if (now == rpos) {
        receive_pos = 0;
    } else {
        receive_pos_t tail = now - rpos;
        memmove(receive_buf, &receive_buf[rpos], tail);
        receive_pos = tail;
    }
    irq_restore(flag);
}
DECL_TASK(console_task);

// Without this reset, a stale shutdown-triggering command left in
// receive_buf re-fires sched_shutdown's longjmp every task-loop pass,
// wedging the MCU. Runs with IRQs already masked (ctr_run_shutdownfuncs),
// so the RX IRQ cannot race the write.
void
serial_shutdown(void)
{
    receive_pos = 0;
}
DECL_SHUTDOWN(serial_shutdown);

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

int
kalico_console_write_raw(const uint8_t *buf, uint16_t len)
{
    transmit_pos_t tpos = read_tpos(&transmit_pos);
    transmit_pos_t tmax = read_tpos(&transmit_max);
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
