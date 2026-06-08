#ifndef __KALICO_DISPATCH_H
#define __KALICO_DISPATCH_H

#include <stdint.h>

#define KALICO_CHANNEL_CONTROL 0x00
#define KALICO_CHANNEL_EVENTS  0x01
#define KALICO_CHANNEL_PIECES  0x02

void piece_sink_begin(void);
void piece_sink_feed(uint8_t b);
void piece_sink_commit(void);

void kalico_dispatch_frame(uint8_t channel, const uint8_t *payload,
                           uint16_t payload_len);

int kalico_transport_send_frame(uint8_t channel, const uint8_t *payload,
                                uint16_t payload_len);

void kalico_reset_epoch_init(void);
uint32_t kalico_reset_epoch_get(void);

void kalico_native_emit_fault_event(uint16_t fault_code,
                                    uint32_t fault_detail,
                                    uint32_t segment_id);

void kalico_native_emit_endstop_trip(uint8_t endstop_id, uint64_t trip_clock);

void send_status_heartbeat(void);

#endif
