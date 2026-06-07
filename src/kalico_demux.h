// Mirror of rust/kalico-native-transport/src/demux.rs — keep in sync.

#ifndef __KALICO_DEMUX_H
#define __KALICO_DEMUX_H

#include <stdint.h>
#include "autoconf.h"
#include "command.h"

#define KALICO_DEMUX_KLIPPER_BUF_SIZE MESSAGE_MAX
#define KALICO_DEMUX_KALICO_BUF_SIZE 512u
_Static_assert(KALICO_DEMUX_KALICO_BUF_SIZE >= 64u,
               "kalico_buf too small for control frames");

// Largest legal kalico frame of any channel = a full pieces frame:
// envelope(4) + per-msg header(7) + piece header(8) + 255*32 + crc(2).
#define KALICO_FRAME_MAX_LEN (4u + 7u + 8u + 255u * 32u + 2u)
_Static_assert(KALICO_FRAME_MAX_LEN >= KALICO_DEMUX_KALICO_BUF_SIZE,
               "global frame bound must cover the staging buffer");

typedef enum {
    KALICO_DEMUX_OUT_NONE,
    KALICO_DEMUX_OUT_KLIPPER,
    KALICO_DEMUX_OUT_KALICO,
    KALICO_DEMUX_OUT_ERROR,
} kalico_demux_output_t;

void kalico_demux_init(void);

kalico_demux_output_t kalico_demux_feed_byte(uint8_t b);

void kalico_demux_consume(void);

// Bootloader-request sentinel detection runs inside this on the OUT_KLIPPER
// branch (gated on CONFIG_HAVE_BOOTLOADER_REQUEST); callers need not check.
void kalico_demux_pump(const uint8_t *buf, uint16_t len);

const uint8_t *kalico_demux_klipper_buf(void);
uint8_t        kalico_demux_klipper_len(void);

const uint8_t *kalico_demux_kalico_payload(void);
uint16_t       kalico_demux_kalico_payload_len(void);
uint8_t        kalico_demux_kalico_channel(void);

#endif
