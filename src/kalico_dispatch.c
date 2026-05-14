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
#include "autoconf.h"               // CONFIG_KALICO_RUNTIME

#if CONFIG_KALICO_RUNTIME
#include "kalico_runtime.h"
extern void *runtime_handle;

// Phase C: shared aligned scratch with command_kalico_load_curve_finalize
// path retired in runtime_tick.c. Defined there.
extern float runtime_aligned_cps[CONFIG_RUNTIME_MAX_CONTROL_POINTS];
extern float runtime_aligned_knots[CONFIG_RUNTIME_MAX_KNOT_VECTOR_LEN];
#endif

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

static void handle_load_curve(uint32_t correlation_id, const uint8_t *body, uint16_t body_len);
static void handle_push_segment(uint32_t correlation_id, const uint8_t *body, uint16_t body_len);
static void handle_configure_axes(uint32_t correlation_id, const uint8_t *body, uint16_t body_len);
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

void
kalico_transport_send_frame(uint8_t channel, const uint8_t *payload,
                            uint16_t payload_len)
{
    // len field per §4: covers [len .. crc] inclusive = 2 + 1 + payload + 2.
    uint32_t len_field = 2u + 1u + (uint32_t)payload_len + 2u;
    uint32_t total = 1u + len_field;
    if (total > KALICO_TX_BUF_SIZE)
        return;
    tx_buf[0] = KALICO_FRAME_SYNC;
    tx_buf[1] = (uint8_t)(len_field & 0xFF);
    tx_buf[2] = (uint8_t)((len_field >> 8) & 0xFF);
    tx_buf[3] = channel;
    if (payload_len > 0)
        memcpy(&tx_buf[4], payload, payload_len);
    uint16_t crc = crc16_ccitt(&tx_buf[1], (uint32_t)(2 + 1 + payload_len));
    tx_buf[total - 2] = (uint8_t)(crc & 0xFF);
    tx_buf[total - 1] = (uint8_t)((crc >> 8) & 0xFF);
    kalico_console_write_raw(tx_buf, (uint16_t)total);
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
    // capabilities: 0 (no caps advertised in MVP).
    memset(&body_out[61], 0, 8);
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
    case KALICO_MSG_LOAD_CURVE:
        handle_load_curve(correlation_id, body, body_len);
        return;
    case KALICO_MSG_PUSH_SEGMENT:
        handle_push_segment(correlation_id, body, body_len);
        return;
    case KALICO_MSG_CONFIGURE_AXES:
        handle_configure_axes(correlation_id, body, body_len);
        return;
    case KALICO_MSG_QUERY_RUNTIME_CAPS:
        handle_query_runtime_caps(correlation_id, body, body_len);
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
static void
handle_query_runtime_caps(uint32_t correlation_id, const uint8_t *body,
                          uint16_t body_len)
{
    (void)body;
    (void)body_len; // request body is empty
    uint8_t payload[PER_MESSAGE_HEADER_LEN + 11];
    encode_message_header(payload, KALICO_MSG_RUNTIME_CAPS_RESPONSE,
                          MESSAGE_VERSION_DEFAULT, correlation_id);
    uint8_t *b = &payload[PER_MESSAGE_HEADER_LEN];
    uint32_t mcp = (uint32_t)CONFIG_RUNTIME_MAX_CONTROL_POINTS;
    uint32_t mkv = (uint32_t)CONFIG_RUNTIME_MAX_KNOT_VECTOR_LEN;
    uint32_t pool = (uint32_t)CONFIG_RUNTIME_CURVE_POOL_N;
    // u32 max_control_points
    b[0] = (uint8_t)(mcp & 0xFF);
    b[1] = (uint8_t)((mcp >> 8) & 0xFF);
    b[2] = (uint8_t)((mcp >> 16) & 0xFF);
    b[3] = (uint8_t)((mcp >> 24) & 0xFF);
    // u32 max_knot_vector_len
    b[4] = (uint8_t)(mkv & 0xFF);
    b[5] = (uint8_t)((mkv >> 8) & 0xFF);
    b[6] = (uint8_t)((mkv >> 16) & 0xFF);
    b[7] = (uint8_t)((mkv >> 24) & 0xFF);
    // u8 max_degree
    b[8] = (uint8_t)CONFIG_RUNTIME_MAX_DEGREE;
    // u16 curve_pool_n
    b[9] = (uint8_t)(pool & 0xFF);
    b[10] = (uint8_t)((pool >> 8) & 0xFF);
    kalico_transport_send_frame(KALICO_CHANNEL_CONTROL,
                                payload, sizeof(payload));
}

// ---------------------------------------------------------------------------
// LoadCurve / PushSegment handlers (Phase C)
// ---------------------------------------------------------------------------

static void
send_load_curve_response(uint32_t correlation_id, int32_t result,
                         uint32_t curve_handle_packed)
{
    uint8_t payload[PER_MESSAGE_HEADER_LEN + 8];
    encode_message_header(payload, KALICO_MSG_LOAD_CURVE_RESPONSE,
                          MESSAGE_VERSION_DEFAULT, correlation_id);
    uint8_t *body = &payload[PER_MESSAGE_HEADER_LEN];
    body[0] = (uint8_t)(result & 0xFF);
    body[1] = (uint8_t)((result >> 8) & 0xFF);
    body[2] = (uint8_t)((result >> 16) & 0xFF);
    body[3] = (uint8_t)((result >> 24) & 0xFF);
    body[4] = (uint8_t)(curve_handle_packed & 0xFF);
    body[5] = (uint8_t)((curve_handle_packed >> 8) & 0xFF);
    body[6] = (uint8_t)((curve_handle_packed >> 16) & 0xFF);
    body[7] = (uint8_t)((curve_handle_packed >> 24) & 0xFF);
    kalico_transport_send_frame(KALICO_CHANNEL_CONTROL,
                                payload, sizeof(payload));
}

static void
send_push_segment_response(uint32_t correlation_id, int32_t result,
                           uint32_t accepted_segment_id, uint32_t credit_epoch)
{
    uint8_t payload[PER_MESSAGE_HEADER_LEN + 12];
    encode_message_header(payload, KALICO_MSG_PUSH_SEGMENT_RESPONSE,
                          MESSAGE_VERSION_DEFAULT, correlation_id);
    uint8_t *body = &payload[PER_MESSAGE_HEADER_LEN];
    body[0] = (uint8_t)(result & 0xFF);
    body[1] = (uint8_t)((result >> 8) & 0xFF);
    body[2] = (uint8_t)((result >> 16) & 0xFF);
    body[3] = (uint8_t)((result >> 24) & 0xFF);
    body[4] = (uint8_t)(accepted_segment_id & 0xFF);
    body[5] = (uint8_t)((accepted_segment_id >> 8) & 0xFF);
    body[6] = (uint8_t)((accepted_segment_id >> 16) & 0xFF);
    body[7] = (uint8_t)((accepted_segment_id >> 24) & 0xFF);
    body[8] = (uint8_t)(credit_epoch & 0xFF);
    body[9] = (uint8_t)((credit_epoch >> 8) & 0xFF);
    body[10] = (uint8_t)((credit_epoch >> 16) & 0xFF);
    body[11] = (uint8_t)((credit_epoch >> 24) & 0xFF);
    kalico_transport_send_frame(KALICO_CHANNEL_CONTROL,
                                payload, sizeof(payload));
}

static void
handle_load_curve(uint32_t correlation_id, const uint8_t *body, uint16_t body_len)
{
#if CONFIG_KALICO_RUNTIME
    // §7.3 body: slot u16 | degree u8 | n_cps u32 | n_knots u32 | cps×f32 | knots×f32
    if (body_len < 11) {
        send_load_curve_response(correlation_id, KALICO_ERR_INVALID_CURVE, 0);
        return;
    }
    uint16_t slot = (uint16_t)body[0] | ((uint16_t)body[1] << 8);
    uint8_t degree = body[2];
    uint32_t n_cps = (uint32_t)body[3] | ((uint32_t)body[4] << 8)
                   | ((uint32_t)body[5] << 16) | ((uint32_t)body[6] << 24);
    uint32_t n_knots = (uint32_t)body[7] | ((uint32_t)body[8] << 8)
                     | ((uint32_t)body[9] << 16) | ((uint32_t)body[10] << 24);
    uint32_t cps_bytes = n_cps * 4u;
    uint32_t knots_bytes = n_knots * 4u;
    uint32_t expected_len = 11u + cps_bytes + knots_bytes;
    if (body_len != expected_len) {
        send_load_curve_response(correlation_id, KALICO_ERR_INVALID_CURVE, 0);
        return;
    }
    if (cps_bytes > sizeof(runtime_aligned_cps)
        || knots_bytes > sizeof(runtime_aligned_knots)) {
        send_load_curve_response(correlation_id, KALICO_ERR_INVALID_CURVE, 0);
        return;
    }
    if (!runtime_handle) {
        send_load_curve_response(correlation_id, KALICO_ERR_NOT_INIT, 0);
        return;
    }
    // Copy into 4-byte-aligned scratch (frame body offset is arbitrary).
    memcpy(runtime_aligned_cps, &body[11], cps_bytes);
    memcpy(runtime_aligned_knots, &body[11 + cps_bytes], knots_bytes);
    uint32_t handle_packed = 0;
    int32_t r = runtime_handle_load_curve(
        runtime_handle, slot,
        runtime_aligned_cps, (uint16_t)n_cps,
        runtime_aligned_knots, (uint16_t)n_knots,
        degree, &handle_packed);
    send_load_curve_response(correlation_id, r, handle_packed);
#else
    (void)body; (void)body_len;
    send_load_curve_response(correlation_id, KALICO_ERR_NOT_INIT, 0);
#endif
}

static void
handle_push_segment(uint32_t correlation_id, const uint8_t *body, uint16_t body_len)
{
#if CONFIG_KALICO_RUNTIME
    // §7.4 body: id u32, 4×handle u32, t_start u64, t_end u64, kin u8, e_mode u8, extrusion_ratio f32 — 42 bytes.
    if (body_len != 42) {
        send_push_segment_response(correlation_id, KALICO_ERR_INVALID_CURVE, 0, 0);
        return;
    }
    if (!runtime_handle) {
        send_push_segment_response(correlation_id, KALICO_ERR_NOT_INIT, 0, 0);
        return;
    }
    uint32_t id = (uint32_t)body[0] | ((uint32_t)body[1] << 8)
                | ((uint32_t)body[2] << 16) | ((uint32_t)body[3] << 24);
    uint32_t x_handle = (uint32_t)body[4] | ((uint32_t)body[5] << 8)
                      | ((uint32_t)body[6] << 16) | ((uint32_t)body[7] << 24);
    uint32_t y_handle = (uint32_t)body[8] | ((uint32_t)body[9] << 8)
                      | ((uint32_t)body[10] << 16) | ((uint32_t)body[11] << 24);
    uint32_t z_handle = (uint32_t)body[12] | ((uint32_t)body[13] << 8)
                      | ((uint32_t)body[14] << 16) | ((uint32_t)body[15] << 24);
    uint32_t e_handle = (uint32_t)body[16] | ((uint32_t)body[17] << 8)
                      | ((uint32_t)body[18] << 16) | ((uint32_t)body[19] << 24);
    uint64_t t_start = 0;
    for (int i = 0; i < 8; i++)
        t_start |= ((uint64_t)body[20 + i]) << (8 * i);
    uint64_t t_end = 0;
    for (int i = 0; i < 8; i++)
        t_end |= ((uint64_t)body[28 + i]) << (8 * i);
    uint8_t kinematics = body[36];
    uint8_t e_mode = body[37];
    uint32_t extrusion_ratio_bits = (uint32_t)body[38] | ((uint32_t)body[39] << 8)
                                  | ((uint32_t)body[40] << 16) | ((uint32_t)body[41] << 24);
    uint32_t accepted_id = 0, credit_epoch = 0;
    int32_t r = runtime_handle_push_segment(
        runtime_handle, id, x_handle, y_handle, z_handle, e_handle,
        t_start, t_end, kinematics, e_mode, extrusion_ratio_bits,
        &accepted_id, &credit_epoch);
    if (r == 0 /* KALICO_OK */) {
        // Wake the step-emission producer Klipper timer. The runtime's
        // `push_segment` already CAS-set `producer_pending=true` inside
        // `Engine::push_segment` (rust/runtime/src/engine.rs); we still
        // need to make sure the producer timer is queued with the
        // scheduler so the actual fill happens. `arm_producer_timer_if_kicked`
        // is idempotent and coalesces concurrent kicks.
        extern void arm_producer_timer_if_kicked(void);
        arm_producer_timer_if_kicked();
    }
    send_push_segment_response(correlation_id, r, accepted_id, credit_epoch);
#else
    (void)body; (void)body_len;
    send_push_segment_response(correlation_id, KALICO_ERR_NOT_INIT, 0, 0);
#endif
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
#if CONFIG_KALICO_RUNTIME
    // C-side diag breadcrumbs. tag=0xCB = "C-side dispatch". The Rust FFI
    // uses tag=0xCA. Both update runtime_diag_last_packed which the 10 Hz
    // status drain surfaces in fault_detail.
    extern void runtime_diag_progress(uint32_t tag, uint32_t stage, uint32_t value);
    runtime_diag_progress(0xCB, 1, body_len);
    // Accept 20-byte (legacy) or 25-byte (extended with StepMode array) blobs.
    if (body_len != 20 && body_len != 25) {
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
        // Register each StepTime-mode motor's consumer Klipper timer with
        // the scheduler now that the engine has a fresh axes config. The
        // consumer timer fires once at a short-poll delay to bootstrap;
        // its first run finds the ring empty, kicks the producer, and
        // then switches to ring-driven scheduling once the producer
        // fills the first batch.
        extern void init_step_time_timers(void);
        init_step_time_timers();
    }
    send_configure_axes_response(correlation_id, r);
    runtime_diag_progress(0xCB, 5, 0);
#else
    (void)body; (void)body_len;
    send_configure_axes_response(correlation_id, KALICO_ERR_NOT_INIT);
#endif
}

// ---------------------------------------------------------------------------
// Event emitters (Phase C — events channel, fire-and-forget, correlation_id=0)
// ---------------------------------------------------------------------------

void
kalico_native_emit_status_event(uint8_t engine_status, uint8_t queue_depth,
                                uint32_t current_segment_id,
                                int32_t last_fault, uint32_t fault_detail)
{
    uint8_t payload[PER_MESSAGE_HEADER_LEN + 18];
    encode_message_header(payload, KALICO_MSG_STATUS_EVENT,
                          MESSAGE_VERSION_DEFAULT, 0);
    uint8_t *b = &payload[PER_MESSAGE_HEADER_LEN];
    b[0] = engine_status;
    b[1] = queue_depth;
    b[2] = (uint8_t)(current_segment_id & 0xFF);
    b[3] = (uint8_t)((current_segment_id >> 8) & 0xFF);
    b[4] = (uint8_t)((current_segment_id >> 16) & 0xFF);
    b[5] = (uint8_t)((current_segment_id >> 24) & 0xFF);
    b[6] = (uint8_t)(last_fault & 0xFF);
    b[7] = (uint8_t)((last_fault >> 8) & 0xFF);
    b[8] = (uint8_t)((last_fault >> 16) & 0xFF);
    b[9] = (uint8_t)((last_fault >> 24) & 0xFF);
    b[10] = (uint8_t)(fault_detail & 0xFF);
    b[11] = (uint8_t)((fault_detail >> 8) & 0xFF);
    b[12] = (uint8_t)((fault_detail >> 16) & 0xFF);
    b[13] = (uint8_t)((fault_detail >> 24) & 0xFF);
    uint32_t epoch = reset_epoch;
    b[14] = (uint8_t)(epoch & 0xFF);
    b[15] = (uint8_t)((epoch >> 8) & 0xFF);
    b[16] = (uint8_t)((epoch >> 16) & 0xFF);
    b[17] = (uint8_t)((epoch >> 24) & 0xFF);
    kalico_transport_send_frame(KALICO_CHANNEL_EVENTS, payload, sizeof(payload));
}

void
kalico_native_emit_credit_freed(uint32_t retired_through_segment_id,
                                uint8_t free_slots)
{
    uint8_t payload[PER_MESSAGE_HEADER_LEN + 5];
    encode_message_header(payload, KALICO_MSG_CREDIT_FREED,
                          MESSAGE_VERSION_DEFAULT, 0);
    uint8_t *b = &payload[PER_MESSAGE_HEADER_LEN];
    b[0] = (uint8_t)(retired_through_segment_id & 0xFF);
    b[1] = (uint8_t)((retired_through_segment_id >> 8) & 0xFF);
    b[2] = (uint8_t)((retired_through_segment_id >> 16) & 0xFF);
    b[3] = (uint8_t)((retired_through_segment_id >> 24) & 0xFF);
    b[4] = free_slots;
    kalico_transport_send_frame(KALICO_CHANNEL_EVENTS, payload, sizeof(payload));
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
