#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "kalico_dispatch.h"
#include "kalico_demux.h"
#include "kalico_protocol_schema.h"
#include "board/misc.h"
#include "board/irq.h"
#include "sched.h"
#include "autoconf.h"

#include "kalico_runtime.h"
extern void *runtime_handle;

extern int kalico_console_write_raw(const uint8_t *buf, uint16_t len);

#define KALICO_FRAME_SYNC 0x55
#define MESSAGE_VERSION_DEFAULT 0x01

#define KALICO_ERR_INVALID_CURVE -2
#define KALICO_ERR_NOT_INIT      -7

// type:u16_le | version:u8 | corr_id:u32_le.
#define PER_MESSAGE_HEADER_LEN 7

#define IDENTIFY_RESPONSE_BODY_LEN 81

#define KALICO_TX_BUF_SIZE 256
static uint8_t tx_buf[KALICO_TX_BUF_SIZE];

static uint32_t reset_epoch;

static void handle_query_runtime_caps(uint32_t correlation_id, const uint8_t *body, uint16_t body_len);
static void handle_stop(uint32_t correlation_id);

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <unistd.h>
static void
read_random_u32(uint32_t *out)
{
    int fd = open("/dev/urandom", O_RDONLY);
    if (fd < 0) {
        *out = 0;
        return;
    }
    uint32_t v = 0;
    ssize_t n = read(fd, &v, sizeof(v));
    close(fd);
    if (n != (ssize_t)sizeof(v))
        v = 0;
    *out = v;
}
#else
static void
read_random_u32(uint32_t *out)
{
    *out = 0xA5A5A5A5;
}
#endif

void
kalico_reset_epoch_init(void)
{
    uint32_t v = 0;
    int spins = 0;
    do {
        read_random_u32(&v);
        spins++;
    } while (v == 0 && spins < 8);
    if (v == 0)
        v = 1; // reset_epoch must never be zero
    reset_epoch = v;
}
DECL_INIT(kalico_reset_epoch_init);

uint32_t
kalico_reset_epoch_get(void)
{
    return reset_epoch;
}

int
kalico_transport_send_frame(uint8_t channel, const uint8_t *payload,
                            uint16_t payload_len)
{
    // len field covers [len .. crc] inclusive = 2 + 1 + payload + 2.
    uint32_t len_field = 2u + 1u + (uint32_t)payload_len + 2u;
    uint32_t total = 1u + len_field;
    if (total > KALICO_TX_BUF_SIZE)
        return -1;
    tx_buf[0] = KALICO_FRAME_SYNC;
    tx_buf[1] = (uint8_t)(len_field & 0xFF);
    tx_buf[2] = (uint8_t)((len_field >> 8) & 0xFF);
    tx_buf[3] = channel;
    if (payload_len > 0)
        memcpy(&tx_buf[4], payload, payload_len);
    uint16_t crc = crc16_ccitt(&tx_buf[1], (uint32_t)(2 + 1 + payload_len));
    tx_buf[total - 2] = (uint8_t)(crc & 0xFF);
    tx_buf[total - 1] = (uint8_t)((crc >> 8) & 0xFF);
    return kalico_console_write_raw(tx_buf, (uint16_t)total);
}

static void
encode_message_header(uint8_t *out, uint16_t kind, uint8_t version,
                      uint32_t correlation_id)
{
    out[0] = (uint8_t)(kind & 0xFF);
    out[1] = (uint8_t)((kind >> 8) & 0xFF);
    out[2] = version;
    out[3] = (uint8_t)(correlation_id & 0xFF);
    out[4] = (uint8_t)((correlation_id >> 8) & 0xFF);
    out[5] = (uint8_t)((correlation_id >> 16) & 0xFF);
    out[6] = (uint8_t)((correlation_id >> 24) & 0xFF);
}

static void
handle_identify(uint32_t correlation_id, const uint8_t *body, uint16_t body_len)
{
    if (body_len != 1)
        return;
    uint8_t proto_version = body[0];
    if (proto_version != KALICO_PROTO_VERSION) {
        return;
    }

    uint8_t payload[PER_MESSAGE_HEADER_LEN + IDENTIFY_RESPONSE_BODY_LEN];
    encode_message_header(payload, KALICO_MSG_IDENTIFY_RESPONSE,
                          0x01, correlation_id);
    uint8_t *body_out = &payload[PER_MESSAGE_HEADER_LEN];

    // IdentifyResponse body layout (offsets, must match host decode):
    //   0  proto_version u8 | 1  firmware_ver u32_le | 5  build_hash [u8;20]
    //   25 schema_hash [u8;32] | 57 reset_epoch u32_le | 61 capabilities u64_le
    //   69 mcu_serial [u8;12]
    body_out[0] = KALICO_PROTO_VERSION;
    uint32_t fw = 0x00000001;
    body_out[1] = (uint8_t)(fw & 0xFF);
    body_out[2] = (uint8_t)((fw >> 8) & 0xFF);
    body_out[3] = (uint8_t)((fw >> 16) & 0xFF);
    body_out[4] = (uint8_t)((fw >> 24) & 0xFF);
    memset(&body_out[5], 0, 20);
    memcpy(&body_out[25], KALICO_SCHEMA_HASH, 32);
    uint32_t epoch = reset_epoch;
    body_out[57] = (uint8_t)(epoch & 0xFF);
    body_out[58] = (uint8_t)((epoch >> 8) & 0xFF);
    body_out[59] = (uint8_t)((epoch >> 16) & 0xFF);
    body_out[60] = (uint8_t)((epoch >> 24) & 0xFF);
    // capabilities bit 0 = PHASE_STEPPING_CAPABLE, advertised unconditionally.
    memset(&body_out[61], 0, 8);
    body_out[61] = 0x01;
    memset(&body_out[69], 0, 12);

    kalico_transport_send_frame(KALICO_CHANNEL_CONTROL,
                                payload, sizeof(payload));
}

void
kalico_dispatch_frame(uint8_t channel, const uint8_t *payload,
                      uint16_t payload_len)
{
    extern void runtime_diag_progress(uint32_t tag, uint32_t stage, uint32_t value);
    (void)channel;
    if (payload_len < PER_MESSAGE_HEADER_LEN) {
        runtime_diag_progress(0xCD, 1, payload_len);
        return;
    }
    uint16_t kind = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
    runtime_diag_progress(0xCD, 2 + (uint32_t)kind, (uint32_t)payload_len);
    uint8_t version = payload[2];
    uint32_t correlation_id = (uint32_t)payload[3]
                            | ((uint32_t)payload[4] << 8)
                            | ((uint32_t)payload[5] << 16)
                            | ((uint32_t)payload[6] << 24);
    const uint8_t *body = &payload[PER_MESSAGE_HEADER_LEN];
    uint16_t body_len = payload_len - PER_MESSAGE_HEADER_LEN;
    (void)version;

    switch (kind) {
    case KALICO_MSG_IDENTIFY:
        handle_identify(correlation_id, body, body_len);
        return;
    case KALICO_MSG_QUERY_RUNTIME_CAPS:
        handle_query_runtime_caps(correlation_id, body, body_len);
        return;
    case KALICO_MSG_STOP:
        handle_stop(correlation_id);
        return;
    default:
        return;
    }
}

static void
handle_query_runtime_caps(uint32_t correlation_id, const uint8_t *body,
                          uint16_t body_len)
{
    (void)body;
    (void)body_len;
    uint8_t payload[PER_MESSAGE_HEADER_LEN + 4];
    encode_message_header(payload, KALICO_MSG_RUNTIME_CAPS_RESPONSE,
                          MESSAGE_VERSION_DEFAULT, correlation_id);
    uint8_t *b = &payload[PER_MESSAGE_HEADER_LEN];
    // u32 total_piece_memory (bytes); host divides by 32 (PieceEntry size)
    // and axis count for per-axis ring depth.
    uint32_t total_piece_memory = (uint32_t)CONFIG_RUNTIME_PIECE_RING_SIZE;
    b[0] = (uint8_t)(total_piece_memory & 0xFF);
    b[1] = (uint8_t)((total_piece_memory >> 8) & 0xFF);
    b[2] = (uint8_t)((total_piece_memory >> 16) & 0xFF);
    b[3] = (uint8_t)((total_piece_memory >> 24) & 0xFF);
    kalico_transport_send_frame(KALICO_CHANNEL_CONTROL,
                                payload, sizeof(payload));
}

// Pieces wire layout streamed through piece_sink_feed (the sink sees only the
// CRC-covered payload; envelope + CRC are the demuxer's):
//   per-message header (7): type u16_le | version u8 | corr_id u32_le
//   piece header       (8): axis_idx u8 | piece_count u8 | start_slot u16_le
//                           | new_head u32_le
//   then piece_count entries of 32 bytes each.
// Combined header offsets: corr_id [3..7), axis_idx [7], piece_count [8],
// start_slot [9..11), new_head [11..15). Each piece lands at
// (start_slot + index) % ring_depth; frontier advances only in commit.
#define PIECE_SINK_HEADER_LEN (PER_MESSAGE_HEADER_LEN + 8u)
#define PIECE_ENTRY_LEN       32u
// Bounds the write index against a malformed over-long frame; such a frame is
// rejected anyway by the piece_count self-check in piece_sink_commit.
#define PIECE_SINK_MAX_PIECES 0xFFu

extern uint32_t stats_send_time;
extern uint32_t stats_send_time_high;
uint32_t timer_read_time(void);

// PushPiecesResponse body (must match Rust decode):
//   result i32_le | arrival_clock u64_le | front_start_time u64_le = 20 bytes.
static void
send_push_pieces_response(uint32_t correlation_id, int32_t result,
                          uint64_t arrival_clock, uint64_t front_start_time)
{
    uint8_t payload[PER_MESSAGE_HEADER_LEN + 4 + 16];
    encode_message_header(payload, KALICO_MSG_PUSH_PIECES_RESPONSE,
                          MESSAGE_VERSION_DEFAULT, correlation_id);
    uint8_t *b = &payload[PER_MESSAGE_HEADER_LEN];
    b[0] = (uint8_t)(result & 0xFF);
    b[1] = (uint8_t)((result >> 8) & 0xFF);
    b[2] = (uint8_t)((result >> 16) & 0xFF);
    b[3] = (uint8_t)((result >> 24) & 0xFF);
    b[4]  = (uint8_t)(arrival_clock & 0xFF);
    b[5]  = (uint8_t)((arrival_clock >> 8) & 0xFF);
    b[6]  = (uint8_t)((arrival_clock >> 16) & 0xFF);
    b[7]  = (uint8_t)((arrival_clock >> 24) & 0xFF);
    b[8]  = (uint8_t)((arrival_clock >> 32) & 0xFF);
    b[9]  = (uint8_t)((arrival_clock >> 40) & 0xFF);
    b[10] = (uint8_t)((arrival_clock >> 48) & 0xFF);
    b[11] = (uint8_t)((arrival_clock >> 56) & 0xFF);
    b[12] = (uint8_t)(front_start_time & 0xFF);
    b[13] = (uint8_t)((front_start_time >> 8) & 0xFF);
    b[14] = (uint8_t)((front_start_time >> 16) & 0xFF);
    b[15] = (uint8_t)((front_start_time >> 24) & 0xFF);
    b[16] = (uint8_t)((front_start_time >> 32) & 0xFF);
    b[17] = (uint8_t)((front_start_time >> 40) & 0xFF);
    b[18] = (uint8_t)((front_start_time >> 48) & 0xFF);
    b[19] = (uint8_t)((front_start_time >> 56) & 0xFF);
    kalico_transport_send_frame(KALICO_CHANNEL_CONTROL,
                                payload, sizeof(payload));
}

// Single-threaded foreground (same context as kalico_demux_pump); no locking.
static struct {
    uint8_t  header[PIECE_SINK_HEADER_LEN];
    uint8_t  scratch[PIECE_ENTRY_LEN];
    uint32_t bytes_seen;
    uint32_t pieces_seen;
    uint32_t correlation_id;
    uint32_t new_head;
    uint16_t start_slot;
    uint8_t  axis_idx;
    uint8_t  piece_count;
    uint8_t  header_parsed;
    int32_t  write_rc;
    uint64_t first_start_time;
} piece_sink;

void
piece_sink_begin(void)
{
    piece_sink.bytes_seen = 0;
    piece_sink.pieces_seen = 0;
    piece_sink.header_parsed = 0;
    piece_sink.write_rc = 0;
    piece_sink.correlation_id = 0;
    piece_sink.new_head = 0;
    piece_sink.start_slot = 0;
    piece_sink.axis_idx = 0;
    piece_sink.piece_count = 0;
    piece_sink.first_start_time = 0;
}

void
piece_sink_feed(uint8_t b)
{
    uint32_t i = piece_sink.bytes_seen;
    if (i < PIECE_SINK_HEADER_LEN) {
        piece_sink.header[i] = b;
        piece_sink.bytes_seen = i + 1;
        if (piece_sink.bytes_seen == PIECE_SINK_HEADER_LEN) {
            const uint8_t *h = piece_sink.header;
            piece_sink.correlation_id = (uint32_t)h[3]
                                      | ((uint32_t)h[4] << 8)
                                      | ((uint32_t)h[5] << 16)
                                      | ((uint32_t)h[6] << 24);
            piece_sink.axis_idx    = h[7];
            piece_sink.piece_count = h[8];
            piece_sink.start_slot  = (uint16_t)h[9]
                                   | ((uint16_t)h[10] << 8);
            piece_sink.new_head    = (uint32_t)h[11]
                                   | ((uint32_t)h[12] << 8)
                                   | ((uint32_t)h[13] << 16)
                                   | ((uint32_t)h[14] << 24);
            piece_sink.header_parsed = 1;
        }
        return;
    }
    uint32_t piece_off = (i - PIECE_SINK_HEADER_LEN) % PIECE_ENTRY_LEN;
    piece_sink.scratch[piece_off] = b;
    piece_sink.bytes_seen = i + 1;
    if (piece_off == PIECE_ENTRY_LEN - 1) {
        if (piece_sink.pieces_seen == 0) {
            piece_sink.first_start_time =
                (uint64_t)piece_sink.scratch[0]
                | ((uint64_t)piece_sink.scratch[1] << 8)
                | ((uint64_t)piece_sink.scratch[2] << 16)
                | ((uint64_t)piece_sink.scratch[3] << 24)
                | ((uint64_t)piece_sink.scratch[4] << 32)
                | ((uint64_t)piece_sink.scratch[5] << 40)
                | ((uint64_t)piece_sink.scratch[6] << 48)
                | ((uint64_t)piece_sink.scratch[7] << 56);
        }
        // Written pre-CRC; the slot stays invisible to the ISR until commit
        // advances the frontier.
        if (runtime_handle && piece_sink.pieces_seen < PIECE_SINK_MAX_PIECES) {
            int32_t r = kalico_runtime_write_piece(
                runtime_handle, piece_sink.axis_idx, piece_sink.start_slot,
                (uint8_t)piece_sink.pieces_seen, piece_sink.scratch);
            if (r != 0 && piece_sink.write_rc == 0)
                piece_sink.write_rc = r;
        }
        piece_sink.pieces_seen++;
    }
}

void
piece_sink_commit(void)
{
    uint32_t clk_lo = timer_read_time();
    uint32_t clk_hi = stats_send_time_high + (clk_lo < stats_send_time);
    uint64_t arrival_clock = ((uint64_t)clk_hi << 32) | (uint64_t)clk_lo;

    if (!runtime_handle) {
        send_push_pieces_response(piece_sink.correlation_id,
                                  KALICO_ERR_NOT_INIT, 0, 0);
        return;
    }
    if (!piece_sink.header_parsed) {
        send_push_pieces_response(0, KALICO_ERR_INVALID_CURVE, 0, 0);
        return;
    }
    // CRC catches bit-corruption but not a count/length logic mismatch; if the
    // streamed piece count disagrees with the declared piece_count, refuse to
    // advance the frontier (partial slots stay below the head, ISR-invisible).
    if (piece_sink.pieces_seen != (uint32_t)piece_sink.piece_count) {
        send_push_pieces_response(piece_sink.correlation_id,
                                  KALICO_ERR_INVALID_CURVE, 0, 0);
        return;
    }
    int32_t rc = piece_sink.write_rc;
    if (rc == 0) {
        rc = kalico_runtime_commit_head(
            runtime_handle, piece_sink.axis_idx, piece_sink.new_head);
    }
    send_push_pieces_response(piece_sink.correlation_id, rc,
                              arrival_clock, piece_sink.first_start_time);
}

void
send_status_heartbeat(void)
{
    if (!runtime_handle)
        return;

    uint8_t st = 0;
    uint8_t fc = 0;
    uint32_t counts[8];
    int32_t n = kalico_runtime_get_heartbeat(runtime_handle,
                                             &st, &fc, counts, 8);
    if (n < 0)
        return;

    // Body = engine_state(1) + fault_code(1) + num_axes(1) + n*u32; max 35 B.
    uint8_t payload[KALICO_TX_BUF_SIZE];
    int off = 0;
    payload[off++] = (uint8_t)(KALICO_MSG_STATUS_HEARTBEAT & 0xFF);
    payload[off++] = (uint8_t)((KALICO_MSG_STATUS_HEARTBEAT >> 8) & 0xFF);
    payload[off++] = MESSAGE_VERSION_DEFAULT;
    payload[off++] = 0;  // correlation_id = 0 (async event)
    payload[off++] = 0;
    payload[off++] = 0;
    payload[off++] = 0;
    payload[off++] = st;
    payload[off++] = fc;
    payload[off++] = (uint8_t)n;
    for (int i = 0; i < n; i++) {
        uint32_t v = counts[i];
        payload[off++] = (uint8_t)(v & 0xFF);
        payload[off++] = (uint8_t)((v >> 8) & 0xFF);
        payload[off++] = (uint8_t)((v >> 16) & 0xFF);
        payload[off++] = (uint8_t)((v >> 24) & 0xFF);
    }
    kalico_transport_send_frame(KALICO_CHANNEL_CONTROL, payload, (uint16_t)off);
}

void
kalico_native_emit_fault_event(uint16_t fault_code, uint32_t fault_detail,
                               uint32_t segment_id)
{
    uint8_t payload[PER_MESSAGE_HEADER_LEN + 10];
    encode_message_header(payload, KALICO_MSG_FAULT_EVENT,
                          MESSAGE_VERSION_DEFAULT, 0);
    uint8_t *b = &payload[PER_MESSAGE_HEADER_LEN];
    b[0] = (uint8_t)(fault_code & 0xFF);
    b[1] = (uint8_t)((fault_code >> 8) & 0xFF);
    b[2] = (uint8_t)(fault_detail & 0xFF);
    b[3] = (uint8_t)((fault_detail >> 8) & 0xFF);
    b[4] = (uint8_t)((fault_detail >> 16) & 0xFF);
    b[5] = (uint8_t)((fault_detail >> 24) & 0xFF);
    b[6] = (uint8_t)(segment_id & 0xFF);
    b[7] = (uint8_t)((segment_id >> 8) & 0xFF);
    b[8] = (uint8_t)((segment_id >> 16) & 0xFF);
    b[9] = (uint8_t)((segment_id >> 24) & 0xFF);
    kalico_transport_send_frame(KALICO_CHANNEL_EVENTS, payload, sizeof(payload));
}

void
kalico_native_emit_endstop_trip(uint8_t endstop_id, uint64_t trip_clock)
{
    uint8_t payload[PER_MESSAGE_HEADER_LEN + 9];
    encode_message_header(payload, KALICO_MSG_ENDSTOP_TRIP,
                          MESSAGE_VERSION_DEFAULT, 0);
    uint8_t *b = &payload[PER_MESSAGE_HEADER_LEN];
    b[0] = endstop_id;
    for (int i = 0; i < 8; i++)
        b[1 + i] = (uint8_t)((trip_clock >> (8 * i)) & 0xFF);
    kalico_transport_send_frame(KALICO_CHANNEL_EVENTS, payload, sizeof(payload));
}

static void
send_stop_response(uint32_t correlation_id, int32_t result, uint64_t discard_clock)
{
    uint8_t payload[PER_MESSAGE_HEADER_LEN + 12];
    encode_message_header(payload, KALICO_MSG_STOP_RESPONSE,
                          MESSAGE_VERSION_DEFAULT, correlation_id);
    uint8_t *b = &payload[PER_MESSAGE_HEADER_LEN];
    b[0] = (uint8_t)(result & 0xFF);
    b[1] = (uint8_t)((result >> 8) & 0xFF);
    b[2] = (uint8_t)((result >> 16) & 0xFF);
    b[3] = (uint8_t)((result >> 24) & 0xFF);
    for (int i = 0; i < 8; i++)
        b[4 + i] = (uint8_t)((discard_clock >> (8 * i)) & 0xFF);
    kalico_transport_send_frame(KALICO_CHANNEL_CONTROL, payload, sizeof(payload));
}

static void
handle_stop(uint32_t correlation_id)
{
    int32_t rc = KALICO_ERR_NOT_INIT;
    uint64_t discard_clock = 0;
    if (runtime_handle) {
        irqstatus_t flag = irq_save();
        rc = kalico_runtime_discard_pending(runtime_handle);
        discard_clock = kalico_runtime_now_ticks(runtime_handle);
        irq_restore(flag);
    }
    send_stop_response(correlation_id, rc, discard_clock);
}
