// Kalico-native transport: Layer 4 dispatcher and TX path.
// Spec: docs/superpowers/specs/2026-05-04-kalico-native-transport-design.md.

#ifndef __KALICO_DISPATCH_H
#define __KALICO_DISPATCH_H

#include <stdint.h>

// Layer 1 channel ids (§4). Shared between the dispatcher and the demuxer
// (the demuxer routes KALICO_CHANNEL_PIECES directly into the streaming
// piece sink, bypassing the staging buffer).
#define KALICO_CHANNEL_CONTROL 0x00
#define KALICO_CHANNEL_EVENTS  0x01
#define KALICO_CHANNEL_PIECES  0x02

// Streaming piece sink (Task 7). Pieces arriving on KALICO_CHANNEL_PIECES are
// streamed byte-by-byte from the demuxer directly into the axis ring, never
// buffered in the demuxer's staging buffer. The demuxer folds the CRC
// incrementally and calls these in order:
//   piece_sink_begin()  once, when the pieces channel byte completes.
//   piece_sink_feed(b)  for every payload byte (pre-CRC-verify).
//   piece_sink_commit() once, only after the trailing CRC matches; advances
//                       the ring frontier and sends the PushPiecesResponse.
// A truncated/CRC-failed frame never calls commit, so partially-written ring
// slots stay below the (un-advanced) frontier and are invisible to the ISR.
void piece_sink_begin(void);
void piece_sink_feed(uint8_t b);
void piece_sink_commit(void);

// Dispatch a kalico frame received from the demuxer.
// `channel` is the Layer 1 channel; `payload` is the post-frame Layer 4
// payload (per-message header + body). The dispatcher is a foreground call
// (same context as Klipper's command dispatch).
void kalico_dispatch_frame(uint8_t channel, const uint8_t *payload,
                           uint16_t payload_len);

// Build and write a complete kalico frame (sync + len + channel + payload + crc)
// to the transport's output. `payload` is the per-message header + body.
// Returns the underlying console-write-raw result: frame length on success,
// -1 on transmit_buf overflow (silent drop). Callers that care about
// delivery must check this and retry on drop.
int kalico_transport_send_frame(uint8_t channel, const uint8_t *payload,
                                uint16_t payload_len);

// Generate a nonzero reset epoch on boot; called from the platform-specific
// init hook. Stored in a static; the IdentifyResponse handler reads it.
void kalico_reset_epoch_init(void);
uint32_t kalico_reset_epoch_get(void);

// Phase C: emit a fault notification on the events channel as a kalico-native
// FaultEvent frame (KALICO_MSG_FAULT_EVENT).
void kalico_native_emit_fault_event(uint16_t fault_code,
                                    uint32_t fault_detail,
                                    uint32_t segment_id);

// 10 Hz per-axis consumed-count heartbeat (StatusHeartbeat 0x0083).
// Called from runtime_status_drain in src/runtime_tick.c.
void send_status_heartbeat(void);

#endif // kalico_dispatch.h
