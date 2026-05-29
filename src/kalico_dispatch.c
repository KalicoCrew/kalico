// Layer 4 dispatcher + frame builder for the kalico-native transport.
//
// Spec: docs/superpowers/specs/2026-05-04-kalico-native-transport-design.md.
// Phase B scope: Identify -> IdentifyResponse handshake. Other handlers are
// stubs that return KALICO_ERR_NOT_IMPLEMENTED via a FaultEvent-style frame
// (or, for Phase B, simply log and drop — the dispatch table is here so we
// can fill in handlers in Phase C without touching the demux/RX path).

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "kalico_dispatch.h"
#include "kalico_demux.h"
#include "kalico_protocol_schema.h" // KALICO_MSG_*, KALICO_SCHEMA_HASH
#include "board/misc.h"             // crc16_ccitt
#include "sched.h"                  // DECL_INIT
#include "autoconf.h"

#include "kalico_runtime.h"
extern void *runtime_handle;

// Forward decl: platform-specific raw byte output. The Linux sim implements
// this in linux/console.c; firmware builds will implement it on top of the
// USB CDC TX path (Phase D).
extern int kalico_console_write_raw(const uint8_t *buf, uint16_t len);

#define KALICO_FRAME_SYNC 0x55
// KALICO_CHANNEL_* live in kalico_dispatch.h (shared with the demuxer).
#define MESSAGE_VERSION_DEFAULT 0x01

// Phase C error codes (mirror runtime FFI conventions).
#define KALICO_ERR_INVALID_CURVE -2
#define KALICO_ERR_NOT_INIT      -7

// Per-message header layout (§7.2): type:u16_le | version:u8 | corr_id:u32_le.
#define PER_MESSAGE_HEADER_LEN 7

// IdentifyResponse body length per §5: 81 bytes.
#define IDENTIFY_RESPONSE_BODY_LEN 81

// Build buffer for one frame at TX time. Sync(1) + len(2) + channel(1) +
// payload (header + body) + crc(2). 256 bytes is plenty for the bootstrap
// path; LoadCurveResponse and friends are also small.
#define KALICO_TX_BUF_SIZE 256
static uint8_t tx_buf[KALICO_TX_BUF_SIZE];

// Reset epoch, generated once at boot. Nonzero by construction.
static uint32_t reset_epoch;

static void handle_query_runtime_caps(uint32_t correlation_id, const uint8_t *body, uint16_t body_len);

// ---------------------------------------------------------------------------
// reset_epoch generation
// ---------------------------------------------------------------------------

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
// STM32 / firmware path (Phase D). For now, deterministic placeholder.
// TODO Phase D: wire to HAL_RNG / device chip-id register.
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
        v = 1; // fallback so reset_epoch is never zero
    reset_epoch = v;
}
DECL_INIT(kalico_reset_epoch_init);

uint32_t
kalico_reset_epoch_get(void)
{
    return reset_epoch;
}

// ---------------------------------------------------------------------------
// TX path
// ---------------------------------------------------------------------------

int
kalico_transport_send_frame(uint8_t channel, const uint8_t *payload,
                            uint16_t payload_len)
{
    // len field per §4: covers [len .. crc] inclusive = 2 + 1 + payload + 2.
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
    // Return the underlying write_raw result so callers that care about
    // delivery can detect transmit_buf overflow drops and retry on the
    // next drain cycle. transmit_buf overflow returns -1; success returns
    // the frame length.
    return kalico_console_write_raw(tx_buf, (uint16_t)total);
}

// ---------------------------------------------------------------------------
// Dispatcher
// ---------------------------------------------------------------------------

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
        // Bootstrap ABI is frozen; older or newer protos must use their own
        // type tag. Drop silently.
        return;
    }

    uint8_t payload[PER_MESSAGE_HEADER_LEN + IDENTIFY_RESPONSE_BODY_LEN];
    encode_message_header(payload, KALICO_MSG_IDENTIFY_RESPONSE,
                          0x01, correlation_id);
    uint8_t *body_out = &payload[PER_MESSAGE_HEADER_LEN];

    // §5 IdentifyResponse layout (offsets relative to body):
    //   0   proto_version  u8
    //   1   firmware_ver   u32_le
    //   5   build_hash     [u8;20]
    //   25  schema_hash    [u8;32]
    //   57  reset_epoch    u32_le
    //   61  capabilities   u64_le
    //   69  mcu_serial     [u8;12]
    body_out[0] = KALICO_PROTO_VERSION;
    // firmware_ver: TODO wire to build-system version. Placeholder = 1.
    uint32_t fw = 0x00000001;
    body_out[1] = (uint8_t)(fw & 0xFF);
    body_out[2] = (uint8_t)((fw >> 8) & 0xFF);
    body_out[3] = (uint8_t)((fw >> 16) & 0xFF);
    body_out[4] = (uint8_t)((fw >> 24) & 0xFF);
    // build_hash: zero-fill (Phase D wires this).
    memset(&body_out[5], 0, 20);
    // schema_hash: copy from generated header.
    memcpy(&body_out[25], KALICO_SCHEMA_HASH, 32);
    // reset_epoch.
    uint32_t epoch = reset_epoch;
    body_out[57] = (uint8_t)(epoch & 0xFF);
    body_out[58] = (uint8_t)((epoch >> 8) & 0xFF);
    body_out[59] = (uint8_t)((epoch >> 16) & 0xFF);
    body_out[60] = (uint8_t)((epoch >> 24) & 0xFF);
    // capabilities: bit 0 = PHASE_STEPPING_CAPABLE. Advertised
    // unconditionally — every supported MCU runs the kalico runtime
    // (H7 modulates at 40 kHz via runtime_tick_h7.c, F4 at 10 kHz via
    // runtime_tick_f4.c). Until Step 10 wires true coil-current
    // synthesis to TMC5160 XDIRECT, both chips route Modulated mode
    // through the same `emit_step_pulses` GPIO path; the bit reflects
    // "this firmware can service a Modulated motor at this chip's tick
    // cadence", not "this firmware drives coil currents".
    memset(&body_out[61], 0, 8);
    body_out[61] = 0x01;
    // mcu_serial: zero-fill (Phase D wires this).
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
        runtime_diag_progress(0xCD, 1, payload_len);  // too-short frame
        return;
    }
    uint16_t kind = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
    // tag=0xCD = "kalico-native frame demuxed". stage=kind, value=payload_len.
    // Surfaces every received kalico-native message at the dispatcher
    // entry, before any handler-specific routing.
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
    // KALICO_MSG_PUSH_PIECES no longer reaches the frame dispatcher: pieces
    // arrive on KALICO_CHANNEL_PIECES and are streamed into the ring by the
    // demuxer's piece-sink path (piece_sink_begin/feed/commit), never
    // accumulated into a staging buffer and dispatched here.
    default:
        return;
    }
}

// ---------------------------------------------------------------------------
// QueryRuntimeCaps handler — per-MCU runtime sizing report (§5.1).
// ---------------------------------------------------------------------------
//
// RuntimeCapsResponse body is pulled from Kconfig at compile time via
// autoconf.h — same source of truth that sizes the Rust runtime's curve pool.
// Cubic-only revision (2026-05-20 stepping redesign): NURBS sizing fields
// (max_control_points / max_knot_vector_len / max_degree) were removed; the
// response now carries a single u32 `total_piece_memory` (bytes).
static void
handle_query_runtime_caps(uint32_t correlation_id, const uint8_t *body,
                          uint16_t body_len)
{
    (void)body;
    (void)body_len; // request body is empty
    uint8_t payload[PER_MESSAGE_HEADER_LEN + 4];
    encode_message_header(payload, KALICO_MSG_RUNTIME_CAPS_RESPONSE,
                          MESSAGE_VERSION_DEFAULT, correlation_id);
    uint8_t *b = &payload[PER_MESSAGE_HEADER_LEN];
    // u32 total_piece_memory (bytes). Host divides by 32 (PieceEntry size)
    // and by the configured axis count to derive per-axis ring depth.
    uint32_t total_piece_memory = (uint32_t)CONFIG_RUNTIME_PIECE_RING_SIZE;
    b[0] = (uint8_t)(total_piece_memory & 0xFF);
    b[1] = (uint8_t)((total_piece_memory >> 8) & 0xFF);
    b[2] = (uint8_t)((total_piece_memory >> 16) & 0xFF);
    b[3] = (uint8_t)((total_piece_memory >> 24) & 0xFF);
    kalico_transport_send_frame(KALICO_CHANNEL_CONTROL,
                                payload, sizeof(payload));
}

// ---------------------------------------------------------------------------
// PushPieces v2 — streaming piece sink (Task 7)
// ---------------------------------------------------------------------------
//
// Pieces arrive on KALICO_CHANNEL_PIECES and are streamed byte-by-byte from
// the demuxer straight into the axis ring — they are never accumulated into
// the demuxer's staging buffer (that staging buffer is what silently dropped
// oversized frames; removing it is the whole point of this change).
//
// Payload wire layout streamed through piece_sink_feed (envelope + CRC are
// handled by the demuxer; the sink sees only the CRC-covered payload bytes):
//
//   per-message header (7):  type u16_le | version u8 | corr_id u32_le
//   piece header       (8):  axis_idx u8 | piece_count u8 | start_slot u16_le
//                            | new_head u32_le
//   piece 0 (32) | piece 1 (32) | ...   (piece_count entries)
//
// So the leading 15 bytes are the combined header. Field offsets:
//   corr_id     = header[3..7)  LE
//   axis_idx    = header[7]
//   piece_count = header[8]
//   start_slot  = header[9..11) LE
//   new_head    = header[11..15) LE
//
// Each completed 32-byte piece is written with
// kalico_runtime_write_piece(rt, axis_idx, start_slot, index, scratch) where
// `index` counts 0,1,2,... so the FFI lands it at (start_slot + index) %
// ring_depth. The frontier is advanced only post-CRC, in piece_sink_commit.

// Combined header = per-message header (7) + piece header (8) = 15 bytes.
#define PIECE_SINK_HEADER_LEN (PER_MESSAGE_HEADER_LEN + 8u)
#define PIECE_ENTRY_LEN       32u
// piece_count is a u8 (header[8]), so a valid frame carries at most 255 pieces
// at indices 0..254. PIECE_SINK_MAX_PIECES guards the write `index` argument
// (passed as a u8 to kalico_runtime_write_piece) against a malformed over-long
// frame: once this many pieces have been written we stop writing but keep
// draining bytes. Such a frame is rejected anyway by the piece_count ==
// pieces_seen self-check in piece_sink_commit, so this is purely a
// belt-and-suspenders bound on the write index.
#define PIECE_SINK_MAX_PIECES 0xFFu

static void
send_push_pieces_response(uint32_t correlation_id, int32_t result)
{
    uint8_t payload[PER_MESSAGE_HEADER_LEN + 4];
    encode_message_header(payload, KALICO_MSG_PUSH_PIECES_RESPONSE,
                          MESSAGE_VERSION_DEFAULT, correlation_id);
    uint8_t *b = &payload[PER_MESSAGE_HEADER_LEN];
    b[0] = (uint8_t)(result & 0xFF);
    b[1] = (uint8_t)((result >> 8) & 0xFF);
    b[2] = (uint8_t)((result >> 16) & 0xFF);
    b[3] = (uint8_t)((result >> 24) & 0xFF);
    kalico_transport_send_frame(KALICO_CHANNEL_CONTROL,
                                payload, sizeof(payload));
}

// Streaming state for the in-flight pieces frame. Single-threaded foreground
// (same context as kalico_demux_pump), so no locking is required.
static struct {
    uint8_t  header[PIECE_SINK_HEADER_LEN];
    uint8_t  scratch[PIECE_ENTRY_LEN]; // current piece being assembled
    uint32_t bytes_seen;               // total payload bytes fed this frame
    uint32_t pieces_seen;              // completed pieces written so far
    // Parsed header fields (valid once bytes_seen >= PIECE_SINK_HEADER_LEN).
    uint32_t correlation_id;
    uint32_t new_head;
    uint16_t start_slot;
    uint8_t  axis_idx;
    uint8_t  piece_count;
    uint8_t  header_parsed;
    int32_t  write_rc;                 // first non-OK write_piece rc, or OK
} piece_sink;

void
piece_sink_begin(void)
{
    piece_sink.bytes_seen = 0;
    piece_sink.pieces_seen = 0;
    piece_sink.header_parsed = 0;
    piece_sink.write_rc = 0; // KALICO_OK
    piece_sink.correlation_id = 0;
    piece_sink.new_head = 0;
    piece_sink.start_slot = 0;
    piece_sink.axis_idx = 0;
    piece_sink.piece_count = 0;
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
    // Payload byte belongs to a 32-byte piece. Offset within the current
    // piece, and which piece this is.
    uint32_t piece_off = (i - PIECE_SINK_HEADER_LEN) % PIECE_ENTRY_LEN;
    piece_sink.scratch[piece_off] = b;
    piece_sink.bytes_seen = i + 1;
    if (piece_off == PIECE_ENTRY_LEN - 1) {
        // A full 32-byte piece just completed. Write it pre-CRC; the slot is
        // not visible to the ISR until commit advances the frontier.
        if (runtime_handle && piece_sink.pieces_seen < PIECE_SINK_MAX_PIECES) {
            int32_t r = kalico_runtime_write_piece(
                runtime_handle, piece_sink.axis_idx, piece_sink.start_slot,
                (uint8_t)piece_sink.pieces_seen, piece_sink.scratch);
            if (r != 0 && piece_sink.write_rc == 0)
                piece_sink.write_rc = r; // latch first failure
        }
        piece_sink.pieces_seen++;
    }
}

void
piece_sink_commit(void)
{
    // Called only after the demuxer verified the frame CRC. If runtime_handle
    // was null, no slots were written; report NOT_INIT and skip commit_head.
    if (!runtime_handle) {
        send_push_pieces_response(piece_sink.correlation_id, KALICO_ERR_NOT_INIT);
        return;
    }
    // If a header never completed (truncated/malformed frame that still
    // matched a CRC over too-few bytes), correlation_id is 0; respond with an
    // error rather than advancing the frontier.
    if (!piece_sink.header_parsed) {
        send_push_pieces_response(0, KALICO_ERR_INVALID_CURVE);
        return;
    }
    // Self-check the framing: the number of 32-byte pieces actually streamed
    // must equal the piece_count the host declared in the piece header. CRC
    // catches bit-corruption, not a count/length logic mismatch (e.g. a host
    // that miscomputed the frame length or piece_count). If they disagree,
    // refuse to advance the frontier — the partially-written slots stay below
    // the un-advanced head and are invisible to the ISR.
    if (piece_sink.pieces_seen != (uint32_t)piece_sink.piece_count) {
        send_push_pieces_response(piece_sink.correlation_id,
                                  KALICO_ERR_INVALID_CURVE);
        return;
    }
    int32_t rc = piece_sink.write_rc;
    if (rc == 0) {
        rc = kalico_runtime_commit_head(
            runtime_handle, piece_sink.axis_idx, piece_sink.new_head);
    }
    send_push_pieces_response(piece_sink.correlation_id, rc);
}

// ---------------------------------------------------------------------------
// StatusHeartbeat (0x0083) — 10 Hz MCU→Host per-axis consumed-count frame.
// ---------------------------------------------------------------------------

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

    // Body = engine_state(1) + fault_code(1) + num_axes(1) + n * u32(4 each).
    // Max body = 3 + 8*4 = 35 bytes. Full frame well within KALICO_TX_BUF_SIZE.
    uint8_t payload[KALICO_TX_BUF_SIZE];
    int off = 0;
    payload[off++] = (uint8_t)(KALICO_MSG_STATUS_HEARTBEAT & 0xFF);
    payload[off++] = (uint8_t)((KALICO_MSG_STATUS_HEARTBEAT >> 8) & 0xFF);
    payload[off++] = MESSAGE_VERSION_DEFAULT;
    // correlation_id = 0 (async event, not a reply).
    payload[off++] = 0;
    payload[off++] = 0;
    payload[off++] = 0;
    payload[off++] = 0;
    // Body.
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

// ---------------------------------------------------------------------------
// Event emitters (Phase C — events channel, fire-and-forget, correlation_id=0)
// ---------------------------------------------------------------------------

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
