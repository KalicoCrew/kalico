// Stream-level demuxer for the kalico-native transport.
//
// C-side mirror of rust/kalico-native-transport/src/demux.rs.
// Spec: docs/superpowers/specs/2026-05-04-kalico-native-transport-design.md §6.

#include <string.h>
#include "kalico_demux.h"
#include "board/misc.h" // crc16_ccitt
#include "sched.h"      // DECL_INIT

#define KLIPPER_LEN_MIN          5
#define KLIPPER_LEN_MAX          64
#define KLIPPER_INTERFRAME_SYNC  0x7E
#define KALICO_FRAME_SYNC        0x55
// Minimum value of the on-wire kalico `len` field: u16 len(2) + channel(1) +
// crc(2). Smaller values are malformed.
#define KALICO_FRAME_MIN_LEN_FIELD  5

typedef enum {
    DEMUX_S_WAITING,
    DEMUX_S_KLIPPER,
    DEMUX_S_KALICO,
} demux_state_t;

static demux_state_t state;

static uint8_t klipper_buf[KALICO_DEMUX_KLIPPER_BUF_SIZE];
static uint16_t klipper_pos;
static uint16_t klipper_remaining;

// Layout of kalico_buf: [sync(1)][len_lo(1)][len_hi(1)][channel(1)][payload..][crc(2)].
// We accumulate the entire on-wire frame here, including the sync byte,
// because that simplifies header parsing (matches the Rust implementation).
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
    // CRC covers [len .. crc-start) per spec §4. kalico_pos is the total
    // frame length (sync + len_field). Validate CRC, then expose payload.
    if (kalico_pos < 1 + KALICO_FRAME_MIN_LEN_FIELD)
        return KALICO_DEMUX_OUT_ERROR;
    uint16_t payload_end = kalico_pos - 2;
    uint16_t crc_expected = (uint16_t)kalico_buf[payload_end]
                          | ((uint16_t)kalico_buf[payload_end + 1] << 8);
    uint16_t crc_actual = crc16_ccitt(&kalico_buf[1], payload_end - 1);
    if (crc_actual != crc_expected)
        return KALICO_DEMUX_OUT_ERROR;
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
        // Stray inter-frame 0x7E: tolerated; everything else: drop silently.
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
            // Overflow: resync.
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
            if (total > KALICO_DEMUX_KALICO_BUF_SIZE) {
                state = DEMUX_S_WAITING;
                return KALICO_DEMUX_OUT_ERROR;
            }
            kalico_total_len = (uint16_t)total;
        }
        if (kalico_total_len > 0 && kalico_pos == kalico_total_len) {
            kalico_demux_output_t out = finalize_kalico_frame();
            state = DEMUX_S_WAITING;
            return out;
        }
        return KALICO_DEMUX_OUT_NONE;
    }
    // Unreachable.
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
    // Payload starts after sync(1) + len(2) + channel(1) = 4 bytes.
    return &kalico_buf[4];
}

uint16_t
kalico_demux_kalico_payload_len(void)
{
    if (kalico_pos < 1 + KALICO_FRAME_MIN_LEN_FIELD)
        return 0;
    // total frame = sync + len_field; payload = total - 4 (header) - 2 (crc).
    return (uint16_t)(kalico_pos - 6);
}

uint8_t
kalico_demux_kalico_channel(void)
{
    return kalico_buf[3];
}
