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
void kalico_transport_send_frame(uint8_t channel, const uint8_t *payload,
                                 uint16_t payload_len);

// Generate a nonzero reset epoch on boot; called from the platform-specific
// init hook. Stored in a static; the IdentifyResponse handler reads it.
void kalico_reset_epoch_init(void);
uint32_t kalico_reset_epoch_get(void);

#endif // kalico_dispatch.h
