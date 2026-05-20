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
// Buffers are static; sized to fit the largest expected frame. The kalico
// buffer is sized for an 8 KB upper bound on the LoadCurve frame for now;
// bump KALICO_DEMUX_KALICO_BUF_SIZE when the per-MCU pool sizing changes.

#ifndef __KALICO_DEMUX_H
#define __KALICO_DEMUX_H

#include <stdint.h>
#include "autoconf.h"
#include "command.h" // MESSAGE_MAX

#define KALICO_DEMUX_KLIPPER_BUF_SIZE MESSAGE_MAX
// Largest in-bound kalico frame is a LoadCurveCubic carrying one slot's
// worth of cubic Bézier pieces, plus the per-frame header. Sizing: each
// piece is 5 × u32 LE = 20 bytes (monomial form for one axis); the
// LoadCurveCubic body is `4 + piece_count * 20` bytes (slot_idx u16 +
// axis_idx u8 + piece_count u8 + pieces[]); plus 32 bytes for the
// sync/len/channel/header/CRC envelope. Stays in lockstep with the Rust
// runtime's `MAX_PIECES_PER_CURVE` (mirrored from Kconfig) so the firmware
// never has to drop a curve upload that the Rust side would still accept.
#define KALICO_DEMUX_KALICO_BUF_SIZE \
    (4u + 20u * (CONFIG_RUNTIME_MAX_PIECES_PER_CURVE) + 32u)

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
