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
// delivery (kalico_native_emit_credit_freed → host slot pool retirement)
// must check this and retry on drop.
int kalico_transport_send_frame(uint8_t channel, const uint8_t *payload,
                                uint16_t payload_len);

// Generate a nonzero reset epoch on boot; called from the platform-specific
// init hook. Stored in a static; the IdentifyResponse handler reads it.
void kalico_reset_epoch_init(void);
uint32_t kalico_reset_epoch_get(void);

// Phase C: emit MCU→host events on the events channel as kalico-native
// frames. Replaces the old Klipper-protocol `output("kalico_status_v6 …")` /
// `output("kalico_credit_freed …")` / `output("kalico_fault …")` paths.
// v2 (2026-05-17): added `retired_through_segment_id` tail field so the
// periodic 10 Hz status frame carries the credit-flow retirement watermark.
// Replaces fire-and-forget kalico_native_emit_credit_freed as the load-bearing
// signal for host slot-pool retirement under USB-CDC TX congestion.
void kalico_native_emit_status_event(uint8_t engine_status, uint8_t queue_depth,
                                     uint32_t current_segment_id,
                                     int32_t last_fault, uint32_t fault_detail,
                                     uint32_t retired_through_segment_id);
// Returns the underlying transport result: positive on success, -1 if the
// frame was dropped due to transmit_buf overflow. The caller (runtime_drain
// in src/runtime_tick.c) must NOT advance last_emitted_retired_id on drop
// so the retry on the next drain cycle re-emits the same cursor value.
int kalico_native_emit_credit_freed(uint32_t retired_through_segment_id,
                                    uint8_t free_slots);
void kalico_native_emit_fault_event(uint16_t fault_code,
                                    uint32_t fault_detail,
                                    uint32_t segment_id);

#endif // kalico_dispatch.h
