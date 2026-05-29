// Stream-level demuxer for the kalico-native transport.
//
// C-side mirror of rust/kalico-native-transport/src/demux.rs.
// Spec: docs/superpowers/specs/2026-05-04-kalico-native-transport-design.md §6.

#include <stdio.h>
#include <string.h>
#include "kalico_demux.h"
#include "board/misc.h" // crc16_ccitt, bootloader_request
#include "command.h"    // command_find_and_dispatch
#include "kalico_dispatch.h" // kalico_dispatch_frame
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
    DEMUX_S_PIECES,   // streaming PushPieces payload straight into the ring
} demux_state_t;

static demux_state_t state;

// Incremental crc16-ccitt fold — one byte at a time. Matches the one-shot
// crc16_ccitt() in src/generic/crc16_ccitt.c exactly (seed 0xffff). Used by
// the streaming pieces path, which never materialises the whole frame in a
// contiguous buffer, so the one-shot crc16_ccitt(buf,len) cannot be applied.
static inline uint16_t
crc16_ccitt_update(uint16_t crc, uint8_t b)
{
    uint8_t data = b ^ (crc & 0xff);
    data ^= data << 4;
    return ((((uint16_t)data << 8) | (crc >> 8))
            ^ (uint8_t)(data >> 4) ^ ((uint16_t)data << 3));
}

// Streaming state for a KALICO_CHANNEL_PIECES frame. The payload is fed
// byte-by-byte into piece_sink_feed and never lands in kalico_buf; the CRC is
// folded incrementally over [len_lo, len_hi, channel, payload...] to match the
// host's one-shot crc16_ccitt over the same span.
static uint16_t pieces_payload_remaining; // payload bytes still to stream
static uint16_t pieces_crc;               // running fold
static uint8_t  pieces_crc_byte;          // count of trailing crc bytes seen
static uint8_t  pieces_crc_lo;            // first (low) crc byte

// 2026-05-14 PushSegment dispatch investigation. handle_push_segment is
// never entered despite host write_frame of kalico-native PushSegment.
// Counters at the demuxer level distinguish "frames reach the demuxer
// state machine" from "frames pass CRC" from "frames fail CRC silently".
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

// Layout of kalico_buf: [sync(1)][len_lo(1)][len_hi(1)][channel(1)][payload..][crc(2)].
// We accumulate the entire on-wire frame here, including the sync byte,
// because that simplifies header parsing (matches the Rust implementation).
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
    // CRC covers [len .. crc-start) per spec §4. kalico_pos is the total
    // frame length (sync + len_field). Validate CRC, then expose payload.
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
        // Once the channel byte has been stored (kalico_pos == 4) and the
        // total length is known, a pieces-channel frame switches to the
        // streaming sink: its payload bytes are fed straight into the ring
        // and never accumulated in kalico_buf. kalico_total_len is computed
        // at pos>=3 above, so it is always known by pos==4. Any other channel
        // stays in DEMUX_S_KALICO and accumulates + finalizes exactly as
        // before (control/events path unchanged).
        if (kalico_pos == 4 && kalico_buf[3] == KALICO_CHANNEL_PIECES
            && kalico_total_len > 0) {
            // payload = total frame - envelope(4: sync+len+channel) - crc(2).
            pieces_payload_remaining = (uint16_t)(kalico_total_len - 6);
            // Seed CRC over [len_lo, len_hi, channel]; payload bytes folded
            // as they stream in (DEMUX_S_PIECES case below).
            pieces_crc = 0xffff;
            pieces_crc = crc16_ccitt_update(pieces_crc, kalico_buf[1]);
            pieces_crc = crc16_ccitt_update(pieces_crc, kalico_buf[2]);
            pieces_crc = crc16_ccitt_update(pieces_crc, kalico_buf[3]);
            pieces_crc_byte = 0;
            piece_sink_begin();
            state = DEMUX_S_PIECES;
            return KALICO_DEMUX_OUT_NONE;
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
        // Trailing CRC bytes, little-endian: low byte first, then high.
        if (pieces_crc_byte == 0) {
            pieces_crc_lo = b;
            pieces_crc_byte = 1;
            return KALICO_DEMUX_OUT_NONE;
        }
        {
            uint16_t crc_expected = (uint16_t)pieces_crc_lo
                                  | ((uint16_t)b << 8);
            // Frame complete either way; reset for the next frame. kalico_pos
            // is left as-is (the next sync byte in DEMUX_S_WAITING resets it),
            // but clear it for cleanliness so no stale length lingers.
            state = DEMUX_S_WAITING;
            kalico_pos = 0;
            kalico_total_len = 0;
            if (crc_expected == pieces_crc) {
                // CRC verified: commit advances the ring frontier and sends
                // the PushPiecesResponse itself, so the pump does nothing.
                piece_sink_commit();
                return KALICO_DEMUX_OUT_NONE;
            }
            // CRC mismatch: no commit, no response. The pieces written
            // pre-CRC stay below the un-advanced frontier and are invisible
            // to the ISR.
            kalico_demux_crc_mismatch_total++;
            return KALICO_DEMUX_OUT_ERROR;
        }
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

// Idle-reset timeout for the demuxer's per-frame state. If a frame starts
// (sync byte received) but never finishes, we resync to WAITING after this
// many idle ticks, regardless of which mid-frame state we got stuck in.
// 100 ms is well above any legitimate inter-byte gap inside a single host
// frame on USB-CDC FS and short enough that a stuck demuxer self-heals
// before the host's identify timeout elapses.
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
            // Bootloader-request sentinel detection. The 32-byte sentinel
            // begins with 0x20 (= 32 decimal), which falls inside the
            // demuxer's [KLIPPER_LEN_MIN=5, KLIPPER_LEN_MAX=64] range, so
            // the demuxer reassembles all 32 bytes into klipper_buf
            // regardless of how the bytes arrive at the transport (one
            // burst, many small bursts, byte-by-byte). Checking here is
            // the only location that survives fragmentation.
            const uint8_t *kbuf = kalico_demux_klipper_buf();
            uint8_t klen = kalico_demux_klipper_len();
            if (CONFIG_HAVE_BOOTLOADER_REQUEST && klen == 32
                && !memcmp(kbuf,
                           " \x1c Request Serial Bootloader!! ~", 32))
                bootloader_request();   // does not return
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
