// Support for WS2812 type "neopixel" LEDs using SPI hardware for timing
//
// Copyright (C) 2025  Russell Cloran <rcloran@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "basecmd.h" // oid_alloc
#include "board/irq.h" // irq_poll
#include "board/misc.h" // timer_read_time
#include "command.h" // DECL_COMMAND
#include "sched.h" // shutdown
#include <string.h> // memcpy
#include "spicmds.h" // spidev_transfer

// This code uses a SPI bus to generate neopixel-compatible data by sending
// different bytes on the SPI bus, each of which represents a bit in the 
// neopixel data.
//
// According to one source, a neopixel 0 can be represented by holding the line
// high for anywhere between 200 and 500 ns, and a 1 for at least 550ns. The
// amount of time the line must then be held low is actually fairly tolerant:
// https://wp.josh.com/2014/05/13/ws2812-neopixels-are-not-so-finicky-once-you-get-to-know-them/
//
// Another source shows that high times as short as 62.5ns are valid 0s:
// https://cpldcpu.com/2014/01/14/light_ws2812-library-v2-0-part-i-understanding-the-ws2812/
//
// Some LEDs may require as much as 800ns high time for a 1:
// https://github.com/Klipper3d/klipper/pull/7113
//
// 2 bits of SPI data take 200ns at 10MHz and 500ns at 4MHz
// 5 bits of SPI data take 550ns at 9.09MHz, or longer at slower rates
// 5 bits of SPI data take 800ns at 6.25MHz

#define ONE_BIT 0b01111100
#define ZERO_BIT 0b01100000

#define NP_BYTES_PER_TRANSFER 3

/****************************************************************
 * Neopixel interface
 ****************************************************************/

struct neopixel_spi_s {
    struct spidev_s *spi;
    uint32_t last_req_time, reset_min_ticks;
    size_t buf_size;
    uint8_t buf[0];
};

void
command_config_neopixel_spi(uint32_t *args)
{
    uint16_t data_size = args[2];
    if (data_size > 0x1000)
        shutdown("Invalid neopixel data_size");
    struct neopixel_spi_s *n = oid_alloc(args[0], command_config_neopixel_spi
                                     , sizeof(*n) + (data_size * 8));

    n->spi = spidev_oid_lookup(args[1]);

    n->buf_size = data_size * 8;
    n->reset_min_ticks = args[3];
}
DECL_COMMAND(command_config_neopixel_spi, "config_neopixel_spi oid=%c"
             " bus_oid=%u data_size=%hu reset_min_ticks=%u");

void
command_neopixel_update_spi(uint32_t *args)
{
    uint8_t oid = args[0];
    struct neopixel_spi_s *n = oid_lookup(oid, command_config_neopixel_spi);
    uint16_t pos = args[1];
    uint8_t data_len = args[2];
    uint8_t *data = command_decode_ptr(args[3]);
    if (pos & 0x8000 || (pos + data_len > (n->buf_size >> 3)))
        shutdown("Invalid neopixel update command");
    while (data_len) {
        uint_fast8_t byte = *data++;
        for (uint_fast8_t bit = 0; bit < 8; bit++) {
            if (byte & 0x80) {
                n->buf[pos * 8 + bit] = ONE_BIT;
            } else {
                n->buf[pos * 8 + bit] = ZERO_BIT;
            }
            byte <<= 1;
        }
        data_len--;
        pos++;
    }
}
DECL_COMMAND(command_neopixel_update_spi,
             "neopixel_update_spi oid=%c pos=%hu data=%*s");

void
command_neopixel_send_spi(uint32_t *args)
{
    uint8_t oid = args[0];
    struct neopixel_spi_s *n = oid_lookup(oid, command_config_neopixel_spi);
    // Make sure the reset time has elapsed since last request
    uint32_t last_req_time = n->last_req_time;
    uint32_t rmt = n->reset_min_ticks;
    uint32_t cur = timer_read_time();
    while (cur - last_req_time < rmt) {
        irq_poll();
        cur = timer_read_time();
    }

    spidev_transfer_large(n->spi, 0, n->buf_size, n->buf);
    n->last_req_time = timer_read_time();  // + transfer time?
    sendf("neopixel_result oid=%c success=%c", oid, 1);
}
DECL_COMMAND(command_neopixel_send_spi, "neopixel_send_spi oid=%c");
