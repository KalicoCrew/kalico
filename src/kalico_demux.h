// Stream-level demultiplexer for kalico-native transport.
//
// Implements the state machine from §6 of
// docs/superpowers/specs/2026-05-04-kalico-native-transport-design.md.
// Mirror of rust/kalico-native-transport/src/demux.rs.
//
// Routes a single incoming USB-CDC byte stream into two parallel logical
// streams: legacy Klipper frames (length-prefixed, 5..=64 bytes) and
// kalico-native frames (sync 0x55 + u16 length + channel + payload + crc).
//
// The caller feeds bytes one at a time. Each call returns a status:
// NONE means the demuxer needs more bytes; KLIPPER means a complete legacy
// frame is sitting in the klipper buffer (length is the leading byte);
// KALICO means a complete kalico frame is in the kalico buffer (the caller
// can use the accessor functions to read channel, payload pointer + len).
// After consuming the frame, the caller calls kalico_demux_consume() to
// reset the accumulator.
//
// Buffers are static; sized to fit the largest expected frame. Pieces stream
// directly into the ring on KALICO_CHANNEL_PIECES (Task 7) and never touch the
// kalico buffer, so it is now sized only for the largest inbound CONTROL frame.

#ifndef __KALICO_DEMUX_H
#define __KALICO_DEMUX_H

#include <stdint.h>
#include "autoconf.h"
#include "command.h" // MESSAGE_MAX

#define KALICO_DEMUX_KLIPPER_BUF_SIZE MESSAGE_MAX
/* Pieces stream straight into the ring on KALICO_CHANNEL_PIECES (Task 7) and
 * never touch kalico_buf. This buffer now only stages the largest inbound
 * CONTROL frame: [sync(1)][len(2)][channel(1)][per-msg hdr(7)][body][crc(2)].
 * ConfigureAxis is the largest control body; 512 B leaves generous margin. */
#define KALICO_DEMUX_KALICO_BUF_SIZE 512u
_Static_assert(KALICO_DEMUX_KALICO_BUF_SIZE >= 64u,
               "kalico_buf too small for control frames");

/* Largest legal kalico frame of ANY channel is a full pieces frame:
 *   envelope(sync+len2+channel = 4) + per-msg header(7) + piece header(8)
 *   + 255 pieces * 32 B + crc(2) = 8181 bytes.
 * Pieces stream straight into the ring and never accumulate in kalico_buf,
 * so a pieces frame's declared length is bounded by THIS, not by the (much
 * smaller) staging buffer. The channel byte is not known until pos==4, so the
 * header-length sanity check applies this global bound; the staging-buffer
 * bound is then applied only to non-pieces channels once the channel decodes. */
#define KALICO_FRAME_MAX_LEN (4u + 7u + 8u + 255u * 32u + 2u) /* = 8181 */
_Static_assert(KALICO_FRAME_MAX_LEN >= KALICO_DEMUX_KALICO_BUF_SIZE,
               "global frame bound must cover the staging buffer");

typedef enum {
    KALICO_DEMUX_OUT_NONE,    // need more bytes; no frame ready
    KALICO_DEMUX_OUT_KLIPPER, // complete Klipper frame in klipper buffer
    KALICO_DEMUX_OUT_KALICO,  // complete kalico frame in kalico buffer
    KALICO_DEMUX_OUT_ERROR,   // stream-level error; demuxer resynced
} kalico_demux_output_t;

void kalico_demux_init(void);

// Feed a single byte. Returns the status. After a non-NONE return, the
// caller must dispatch the buffered frame and then call kalico_demux_consume.
kalico_demux_output_t kalico_demux_feed_byte(uint8_t b);

// Reset the per-frame accumulator after the caller has consumed the most
// recently emitted frame.
void kalico_demux_consume(void);

// Drain a buffer of bytes through the demuxer state machine, dispatching
// klipper frames via command_find_and_dispatch and kalico-native frames
// via kalico_dispatch_frame as they surface. Demuxer state persists across
// calls, so partial frames at buffer boundaries are handled correctly.
//
// Bootloader-request sentinel detection (32-byte magic string) runs inside
// this function on the OUT_KLIPPER branch, so callers do NOT need to check
// for the sentinel separately. The check is gated on
// CONFIG_HAVE_BOOTLOADER_REQUEST.
void kalico_demux_pump(const uint8_t *buf, uint16_t len);

const uint8_t *kalico_demux_klipper_buf(void);
uint8_t        kalico_demux_klipper_len(void);

const uint8_t *kalico_demux_kalico_payload(void);
uint16_t       kalico_demux_kalico_payload_len(void);
uint8_t        kalico_demux_kalico_channel(void);

#endif // kalico_demux.h
