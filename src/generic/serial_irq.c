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

#if CONFIG_KALICO_RUNTIME && CONFIG_MACH_STM32H7
// H7 with kalico runtime — sized to hold the largest kalico-native frame on
// the wire while console_task is briefly preempted by TIM5 ISRs (40 kHz
// during active motion). 2048 bytes is enough headroom to fit a full
// LoadCurve frame plus a couple of clock-sync requests without dropping
// bytes, even when console_task is starved for ~20 ms by TIM5 ISR storms
// in Renode (sim) or real silicon TIM5 isr_n bursts. Buffer goes in AXI
// SRAM via `.axi_bss` because the H7's 128 KB DTCM is already saturated
// by the Rust runtime's RT_CELL curve pool.
// Sized to uint16_t (receive_pos) to escape the 256-byte uint8_t ceiling.
#define RX_BUFFER_SIZE 2048
__attribute__((section(".axi_bss")))
static uint8_t receive_buf[RX_BUFFER_SIZE];
static uint16_t receive_pos;
typedef uint16_t receive_pos_t;
#define read_rpos(a)    readw(a)
#define write_rpos(a,v) writew((a),(v))
// Sized to fit one full kalico frame (KALICO_TX_BUF_SIZE = 256 in
// src/kalico_dispatch.c) plus headroom for a Klipper response.
static uint8_t transmit_buf[320];
static uint16_t transmit_pos, transmit_max;
typedef uint16_t transmit_pos_t;
#define read_tpos(a)    readw(a)
#define write_tpos(a,v) writew((a),(v))
#elif CONFIG_KALICO_RUNTIME
// Non-H7 runtime targets (F4 bench) — keep DTCM buffer but at the
// legacy 192-byte size since AXI SRAM doesn't exist on F4. F4 still
// receives the same 700+ byte frames, but at the slower TIM5 cadence
// (or no TIM5 at all if step_time-only motors) console_task usually
// drains in time. If F4 starts losing bytes, revisit by moving to
// a CCM-RAM or .ccmram section.
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
#else
// AVR / non-runtime targets keep the original 192-byte buffer + uint8_t pos.
#define RX_BUFFER_SIZE 192
static uint8_t receive_buf[RX_BUFFER_SIZE], receive_pos;
typedef uint_fast8_t receive_pos_t;
#define read_rpos(a)    readb(a)
#define write_rpos(a,v) writeb((a),(v))
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
    if (data == MESSAGE_SYNC) {
        sched_wake_tasks();
#if CONFIG_KALICO_RUNTIME
    } else if (data == 0x55) {
        // Kalico-native frame sync byte. Without this wake, a long kalico
        // frame (e.g. 744-byte LoadCurve) can fill the 192-byte staging
        // buffer before any of Klipper's 0x7E bytes happen to occur within
        // the random payload, dropping bytes silently. Waking on each
        // kalico frame start guarantees console_task gets a chance to
        // drain into kalico_demux mid-frame.
        sched_wake_tasks();
    } else if (receive_pos > (sizeof(receive_buf) * 3 / 4)) {
        // Receive buffer is filling up — wake the drain task before we
        // overflow. Threshold (3/4 full = 144 bytes) leaves headroom for
        // a console_task scheduling latency window without triggering on
        // every quiescent byte.
        sched_wake_tasks();
#endif
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

#if !CONFIG_KALICO_RUNTIME
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
#endif

// Process any incoming commands
void
console_task(void)
{
    receive_pos_t rpos = read_rpos(&receive_pos);

#if CONFIG_KALICO_RUNTIME
    kalico_demux_pump(receive_buf, rpos);

    // Bytes the IRQ deposited during pump live in [rpos, now). Rebase
    // them down to the start of receive_buf and update receive_pos
    // atomically so a fresh IRQ doesn't write into a slot we just
    // moved. Doing the memmove inside irq_save trades latency for the
    // simpler invariant; no retry loop needed (unlike console_pop_input,
    // which does memmove outside irq_save and therefore must retry).
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
#else
    uint_fast8_t pop_count;
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
#endif
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
