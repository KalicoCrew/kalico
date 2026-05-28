// Kalico-native transport: Layer 4 dispatcher and TX path.
// Spec: docs/superpowers/specs/2026-05-04-kalico-native-transport-design.md.

#ifndef __KALICO_DISPATCH_H
#define __KALICO_DISPATCH_H

#include <stdint.h>

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
