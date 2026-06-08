// C-side mirror of rust/kalico-native-transport/src/demux.rs — keep in sync.

#include <stdio.h>
#include <string.h>
#include "kalico_demux.h"
#include "board/misc.h"
#include "command.h"
#include "kalico_dispatch.h"
#include "sched.h"

#define KLIPPER_LEN_MIN          5
#define KLIPPER_LEN_MAX          64
#define KLIPPER_INTERFRAME_SYNC  0x7E
#define KALICO_FRAME_SYNC        0x55
#define KALICO_FRAME_MIN_LEN_FIELD  5
#define KALICO_FRAME_OVERHEAD       6u  /* envelope(sync+len2+channel=4) + crc(2) */

typedef enum {
    DEMUX_S_WAITING,
    DEMUX_S_KLIPPER,
    DEMUX_S_KALICO,
    DEMUX_S_PIECES,
} demux_state_t;

static demux_state_t state;

// Must match the one-shot crc16_ccitt() in src/generic/crc16_ccitt.c (seed
// 0xffff); the streaming pieces path folds byte-by-byte and never has a
// contiguous buffer to pass to the one-shot variant.
static inline uint16_t
crc16_ccitt_update(uint16_t crc, uint8_t b)
{
    uint8_t data = b ^ (crc & 0xff);
    data ^= data << 4;
    return ((((uint16_t)data << 8) | (crc >> 8))
            ^ (uint8_t)(data >> 4) ^ ((uint16_t)data << 3));
}

static uint16_t pieces_payload_remaining;
static uint16_t pieces_crc;
static uint8_t  pieces_crc_byte;
static uint8_t  pieces_crc_lo;

volatile uint32_t kalico_demux_out_kalico_total
                __attribute__((used, externally_visible));
volatile uint32_t kalico_demux_out_error_total
                __attribute__((used, externally_visible));
volatile uint32_t kalico_demux_out_klipper_total
                __attribute__((used, externally_visible));
volatile uint32_t kalico_demux_crc_mismatch_total
                __attribute__((used, externally_visible));

static uint8_t klipper_buf[KALICO_DEMUX_KLIPPER_BUF_SIZE];
static uint16_t klipper_pos;
static uint16_t klipper_remaining;

// Holds the whole on-wire frame including the sync byte:
// [sync(1)][len_lo(1)][len_hi(1)][channel(1)][payload..][crc(2)].
#if CONFIG_MACH_STM32H7
__attribute__((section(".axi_bss")))
#endif
static uint8_t kalico_buf[KALICO_DEMUX_KALICO_BUF_SIZE];
static uint16_t kalico_pos;
static uint16_t kalico_total_len; // 0 means header not yet known

void
kalico_demux_init(void)
{
    state = DEMUX_S_WAITING;
    klipper_pos = 0;
    klipper_remaining = 0;
    kalico_pos = 0;
    kalico_total_len = 0;
}
DECL_INIT(kalico_demux_init);

static kalico_demux_output_t
finalize_kalico_frame(void)
{
    // CRC covers [len .. crc-start).
    if (kalico_pos < 1 + KALICO_FRAME_MIN_LEN_FIELD)
        return KALICO_DEMUX_OUT_ERROR;
    uint16_t payload_end = kalico_pos - 2;
    uint16_t crc_expected = (uint16_t)kalico_buf[payload_end]
                          | ((uint16_t)kalico_buf[payload_end + 1] << 8);
    uint16_t crc_actual = crc16_ccitt(&kalico_buf[1], payload_end - 1);
    if (crc_actual != crc_expected) {
#if CONFIG_MACH_LINUX
        fprintf(stderr, "[mcu] crc mismatch: expected 0x%04x, got 0x%04x, kalico_pos=%u\n",
                crc_expected, crc_actual, kalico_pos);
        fflush(stderr);
#endif
        kalico_demux_crc_mismatch_total++;
        return KALICO_DEMUX_OUT_ERROR;
    }
    return KALICO_DEMUX_OUT_KALICO;
}

kalico_demux_output_t
kalico_demux_feed_byte(uint8_t b)
{
    switch (state) {
    case DEMUX_S_WAITING:
        if (b >= KLIPPER_LEN_MIN && b <= KLIPPER_LEN_MAX) {
            klipper_buf[0] = b;
            klipper_pos = 1;
            klipper_remaining = (uint16_t)b - 1;
            state = DEMUX_S_KLIPPER;
            return KALICO_DEMUX_OUT_NONE;
        }
        if (b == KALICO_FRAME_SYNC) {
            kalico_buf[0] = b;
            kalico_pos = 1;
            kalico_total_len = 0;
            state = DEMUX_S_KALICO;
            return KALICO_DEMUX_OUT_NONE;
        }
        return KALICO_DEMUX_OUT_NONE;

    case DEMUX_S_KLIPPER:
        klipper_buf[klipper_pos++] = b;
        klipper_remaining--;
        if (klipper_remaining == 0) {
            state = DEMUX_S_WAITING;
            return KALICO_DEMUX_OUT_KLIPPER;
        }
        return KALICO_DEMUX_OUT_NONE;

    case DEMUX_S_KALICO:
        if (kalico_pos >= KALICO_DEMUX_KALICO_BUF_SIZE) {
            state = DEMUX_S_WAITING;
            return KALICO_DEMUX_OUT_ERROR;
        }
        kalico_buf[kalico_pos++] = b;
        if (kalico_total_len == 0 && kalico_pos >= 3) {
            uint16_t len_field = (uint16_t)kalico_buf[1]
                               | ((uint16_t)kalico_buf[2] << 8);
            if (len_field < KALICO_FRAME_MIN_LEN_FIELD) {
                state = DEMUX_S_WAITING;
                return KALICO_DEMUX_OUT_ERROR;
            }
            uint32_t total = 1u + (uint32_t)len_field;
            // Channel is unknown until pos==4, so bound by the largest legal
            // frame of any channel; the staging-buffer bound is applied
            // per-channel below.
            if (total > KALICO_FRAME_MAX_LEN) {
                state = DEMUX_S_WAITING;
                return KALICO_DEMUX_OUT_ERROR;
            }
            kalico_total_len = (uint16_t)total;
        }
        if (kalico_pos == 4 && kalico_buf[3] == KALICO_CHANNEL_PIECES
            && kalico_total_len > 0) {
            pieces_payload_remaining =
                (uint16_t)(kalico_total_len - KALICO_FRAME_OVERHEAD);
            pieces_crc = 0xffff;
            pieces_crc = crc16_ccitt_update(pieces_crc, kalico_buf[1]);
            pieces_crc = crc16_ccitt_update(pieces_crc, kalico_buf[2]);
            pieces_crc = crc16_ccitt_update(pieces_crc, kalico_buf[3]);
            pieces_crc_byte = 0;
            piece_sink_begin();
            state = DEMUX_S_PIECES;
            return KALICO_DEMUX_OUT_NONE;
        }
        if (kalico_pos == 4 && kalico_buf[3] != KALICO_CHANNEL_PIECES
            && kalico_total_len > KALICO_DEMUX_KALICO_BUF_SIZE) {
            state = DEMUX_S_WAITING;
            return KALICO_DEMUX_OUT_ERROR;
        }
        if (kalico_total_len > 0 && kalico_pos == kalico_total_len) {
            kalico_demux_output_t out = finalize_kalico_frame();
            state = DEMUX_S_WAITING;
            return out;
        }
        return KALICO_DEMUX_OUT_NONE;

    case DEMUX_S_PIECES:
        if (pieces_payload_remaining > 0) {
            pieces_crc = crc16_ccitt_update(pieces_crc, b);
            piece_sink_feed(b);
            pieces_payload_remaining--;
            return KALICO_DEMUX_OUT_NONE;
        }
        // Trailing CRC, little-endian (low byte first).
        if (pieces_crc_byte == 0) {
            pieces_crc_lo = b;
            pieces_crc_byte = 1;
            return KALICO_DEMUX_OUT_NONE;
        }
        {
            uint16_t crc_expected = (uint16_t)pieces_crc_lo
                                  | ((uint16_t)b << 8);
            // The pieces path commits inline and returns OUT_NONE, bypassing
            // kalico_demux_consume(); this is the only reset of kalico_pos /
            // kalico_total_len for a committed pieces frame.
            state = DEMUX_S_WAITING;
            kalico_pos = 0;
            kalico_total_len = 0;
            if (crc_expected == pieces_crc) {
                piece_sink_commit();
                return KALICO_DEMUX_OUT_NONE;
            }
            kalico_demux_crc_mismatch_total++;
            return KALICO_DEMUX_OUT_ERROR;
        }
    }
    state = DEMUX_S_WAITING;
    return KALICO_DEMUX_OUT_ERROR;
}

void
kalico_demux_consume(void)
{
    klipper_pos = 0;
    klipper_remaining = 0;
    kalico_pos = 0;
    kalico_total_len = 0;
}

const uint8_t *
kalico_demux_klipper_buf(void)
{
    return klipper_buf;
}

uint8_t
kalico_demux_klipper_len(void)
{
    return (uint8_t)klipper_pos;
}

const uint8_t *
kalico_demux_kalico_payload(void)
{
    return &kalico_buf[4];
}

uint16_t
kalico_demux_kalico_payload_len(void)
{
    if (kalico_pos < 1 + KALICO_FRAME_MIN_LEN_FIELD)
        return 0;
    return (uint16_t)(kalico_pos - KALICO_FRAME_OVERHEAD);
}

uint8_t
kalico_demux_kalico_channel(void)
{
    return kalico_buf[3];
}

// 100 ms is above any legitimate inter-byte gap inside one host frame on
// USB-CDC FS, and below the host's identify timeout, so a stuck mid-frame
// demuxer self-heals before the host gives up.
static uint32_t last_byte_time;

void
kalico_demux_pump(const uint8_t *buf, uint16_t len)
{
    if (len == 0)
        return;
    uint32_t now = timer_read_time();
    if (state != DEMUX_S_WAITING) {
        uint32_t idle_ticks = now - last_byte_time;
        if (idle_ticks > timer_from_us(100000)) {
            state = DEMUX_S_WAITING;
            klipper_pos = 0;
            klipper_remaining = 0;
            kalico_pos = 0;
            kalico_total_len = 0;
        }
    }
    last_byte_time = now;
    for (uint16_t i = 0; i < len; i++) {
        kalico_demux_output_t out = kalico_demux_feed_byte(buf[i]);
        switch (out) {
        case KALICO_DEMUX_OUT_NONE:
            break;
        case KALICO_DEMUX_OUT_KLIPPER: {
            kalico_demux_out_klipper_total++;
#if CONFIG_MACH_LINUX
            {
                const uint8_t *kb = kalico_demux_klipper_buf();
                uint8_t kl = kalico_demux_klipper_len();
                fprintf(stderr, "[mcu-demux] KLIPPER len=%u seq=0x%02x total=%u\n",
                        kl, kl >= 2 ? kb[1] : 0, kalico_demux_out_klipper_total);
                fflush(stderr);
            }
#endif
            const uint8_t *kbuf = kalico_demux_klipper_buf();
            uint8_t klen = kalico_demux_klipper_len();
            if (CONFIG_HAVE_BOOTLOADER_REQUEST && klen == 32
                && !memcmp(kbuf,
                           " \x1c Request Serial Bootloader!! ~", 32))
                bootloader_request();
            uint_fast8_t pop_count;
            command_find_and_dispatch(
                (uint8_t *)kbuf, klen, &pop_count);
            kalico_demux_consume();
            break;
        }
        case KALICO_DEMUX_OUT_KALICO:
            kalico_demux_out_kalico_total++;
            kalico_dispatch_frame(
                kalico_demux_kalico_channel(),
                kalico_demux_kalico_payload(),
                kalico_demux_kalico_payload_len());
            kalico_demux_consume();
            break;
        case KALICO_DEMUX_OUT_ERROR:
            kalico_demux_out_error_total++;
            kalico_demux_consume();
            break;
        }
    }
}
