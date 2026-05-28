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
#define KALICO_CHANNEL_CONTROL 0x00
#define KALICO_CHANNEL_EVENTS  0x01
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

static void handle_configure_axes(uint32_t correlation_id, const uint8_t *body, uint16_t body_len);
static void handle_query_runtime_caps(uint32_t correlation_id, const uint8_t *body, uint16_t body_len);
static void handle_push_pieces(uint32_t correlation_id, const uint8_t *body, uint16_t body_len);

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
    case KALICO_MSG_CONFIGURE_AXES:
        handle_configure_axes(correlation_id, body, body_len);
        return;
    case KALICO_MSG_QUERY_RUNTIME_CAPS:
        handle_query_runtime_caps(correlation_id, body, body_len);
        return;
    case KALICO_MSG_PUSH_PIECES:
        handle_push_pieces(correlation_id, body, body_len);
        return;
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
// ConfigureAxes handler
// ---------------------------------------------------------------------------

static void
send_configure_axes_response(uint32_t correlation_id, int32_t result)
{
    uint8_t payload[PER_MESSAGE_HEADER_LEN + 4];
    encode_message_header(payload, KALICO_MSG_CONFIGURE_AXES_RESPONSE,
                          MESSAGE_VERSION_DEFAULT, correlation_id);
    uint8_t *body = &payload[PER_MESSAGE_HEADER_LEN];
    body[0] = (uint8_t)(result & 0xFF);
    body[1] = (uint8_t)((result >> 8) & 0xFF);
    body[2] = (uint8_t)((result >> 16) & 0xFF);
    body[3] = (uint8_t)((result >> 24) & 0xFF);
    kalico_transport_send_frame(KALICO_CHANNEL_CONTROL,
                                payload, sizeof(payload));
}

static void
handle_configure_axes(uint32_t correlation_id, const uint8_t *body, uint16_t body_len)
{
    // C-side diag breadcrumbs. tag=0xCB = "C-side dispatch". The Rust FFI
    // uses tag=0xCA. Both update runtime_diag_last_packed which the 10 Hz
    // status drain surfaces in fault_detail.
    extern void runtime_diag_progress(uint32_t tag, uint32_t stage, uint32_t value);
    runtime_diag_progress(0xCB, 1, body_len);
    // Accept 20-byte (legacy), 25-byte (extended StepMode array), or
    // 26+3N-byte (variable-length per-motor phase config; N in 0..=16
    // motors). The Rust parser validates the per-motor entries; this
    // wrapper only gates the length to a recognized shape.
    int accept = (body_len == 20) || (body_len == 25);
    if (!accept && body_len >= 26) {
        uint16_t tail = body_len - 26;
        if (tail % 3 == 0 && (tail / 3) <= 16) {
            accept = 1;
        }
    }
    if (!accept) {
        send_configure_axes_response(correlation_id, KALICO_ERR_INVALID_CURVE);
        return;
    }
    runtime_diag_progress(0xCB, 2, 0);
    if (!runtime_handle) {
        send_configure_axes_response(correlation_id, KALICO_ERR_NOT_INIT);
        return;
    }
    runtime_diag_progress(0xCB, 3, 0);
    int32_t r = kalico_runtime_configure_axes_blob(runtime_handle, body, body_len);
    runtime_diag_progress(0xCB, 4, (uint32_t)r);
    if (r == 0 /* KALICO_OK */) {
        // The new stepping path uses init_per_axis_step_timers (installed
        // once at boot via kalico_configure_axis). No per-config-axes
        // timer registration needed here. Stepping-redesign-finish Task 17.
    }
    send_configure_axes_response(correlation_id, r);
    runtime_diag_progress(0xCB, 5, 0);
}

// ---------------------------------------------------------------------------
// PushPieces handler (0x0060) — Task 7
// ---------------------------------------------------------------------------
//
// Delivers a batch of pre-baked polynomial `PieceEntry` records for a single
// axis's ring buffer.
//
// Body wire format (§ PushPieces):
//   axis_idx    u8       (body[0])
//   piece_count u8       (body[1])
//   pieces      piece_count * 32 bytes  (body + 2)
//
// Total body length must equal 2 + piece_count * 32.
//
// The Rust FFI `kalico_runtime_push_pieces` validates alignment internally
// via `read_unaligned`; the C side only gates the overall body length.

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

static void
handle_push_pieces(uint32_t correlation_id, const uint8_t *body, uint16_t body_len)
{
    // Minimum body: axis_idx(1) + piece_count(1) = 2 bytes (0 pieces is legal).
    if (body_len < 2) {
        send_push_pieces_response(correlation_id, KALICO_ERR_INVALID_CURVE);
        return;
    }
    uint8_t axis_idx = body[0];
    uint8_t piece_count = body[1];
    uint32_t expected_len = 2u + (uint32_t)piece_count * 32u;
    if ((uint32_t)body_len != expected_len) {
        send_push_pieces_response(correlation_id, KALICO_ERR_INVALID_CURVE);
        return;
    }
    if (!runtime_handle) {
        send_push_pieces_response(correlation_id, KALICO_ERR_NOT_INIT);
        return;
    }
    // pieces_len is body_len - 2; the Rust FFI re-validates piece_count * 32.
    uint16_t pieces_len = (uint16_t)(body_len - 2u);
    int32_t r = kalico_runtime_push_pieces(
        runtime_handle, axis_idx, piece_count, body + 2, pieces_len);
    send_push_pieces_response(correlation_id, r);
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
