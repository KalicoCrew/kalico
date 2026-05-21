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

// 2026-05-14 push_segment investigation counters. Surfaced via
// fault_detail tags 0xB6/0xB7 so we can localise whether
// PushSegment frames reach handle_push_segment and what runtime_handle_push_segment
// actually returns. All in .bss; cleared on MCU reset.
volatile uint32_t handle_push_segment_calls_total
                __attribute__((used, externally_visible));
volatile uint32_t handle_push_segment_invalid_body_total
                __attribute__((used, externally_visible));
volatile uint32_t handle_push_segment_no_handle_total
                __attribute__((used, externally_visible));
volatile int32_t handle_push_segment_last_r
                __attribute__((used, externally_visible));

static void handle_load_curve_cubic(uint32_t correlation_id, const uint8_t *body, uint16_t body_len);
static void handle_push_segment(uint32_t correlation_id, const uint8_t *body, uint16_t body_len);
static void handle_configure_axes(uint32_t correlation_id, const uint8_t *body, uint16_t body_len);
static void handle_query_runtime_caps(uint32_t correlation_id, const uint8_t *body, uint16_t body_len);
static void handle_reset_curve_pool(uint32_t correlation_id, const uint8_t *body, uint16_t body_len);

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
    // 2026-05-17: return the underlying write_raw result so callers that
    // care about delivery (kalico_native_emit_credit_freed → host slot
    // pool retirement) can detect transmit_buf overflow drops and retry
    // on the next drain cycle. transmit_buf overflow returns -1; success
    // returns the frame length.
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
    case KALICO_MSG_LOAD_CURVE_CUBIC:
        handle_load_curve_cubic(correlation_id, body, body_len);
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
    case KALICO_MSG_RESET_CURVE_POOL:
        handle_reset_curve_pool(correlation_id, body, body_len);
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
// response now carries only `curve_pool_n` and `max_pieces_per_curve`.
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
    uint32_t pool = (uint32_t)CONFIG_RUNTIME_CURVE_POOL_N;
    uint32_t mpc = (uint32_t)CONFIG_RUNTIME_MAX_PIECES_PER_CURVE;
    // u16 curve_pool_n
    b[0] = (uint8_t)(pool & 0xFF);
    b[1] = (uint8_t)((pool >> 8) & 0xFF);
    // u16 max_pieces_per_curve
    b[2] = (uint8_t)(mpc & 0xFF);
    b[3] = (uint8_t)((mpc >> 8) & 0xFF);
    kalico_transport_send_frame(KALICO_CHANNEL_CONTROL,
                                payload, sizeof(payload));
}

// ---------------------------------------------------------------------------
// LoadCurveCubic / PushSegment handlers (Phase C)
// ---------------------------------------------------------------------------

// FFI: Rust runtime — cubic curve loader.
// Declared in `rust/kalico-c-api/include/kalico_runtime.h` (already included
// at the top of this file); the prototype is kept here as a one-line forward
// declaration would shadow the canonical signature, so we let the include
// supply it.

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
handle_load_curve_cubic(uint32_t correlation_id, const uint8_t *body, uint16_t body_len)
{
    // Wire format (spec §3.2):
    //   slot_idx: u16 LE (offset 0)
    //   axis_idx: u8     (offset 2)
    //   piece_count: u8  (offset 3)
    //   pieces: piece_count * 20 bytes, each = 5 × u32 LE
    //     bp0_bits, bp1_bits, bp2_bits, bp3_bits, duration_bits
    if (body_len < 4) {
        send_load_curve_response(correlation_id, KALICO_ERR_INVALID_CURVE, 0);
        return;
    }
    uint16_t slot_idx = (uint16_t)body[0] | ((uint16_t)body[1] << 8);
    uint8_t axis_idx = body[2];
    uint8_t piece_count = body[3];
    uint32_t expected_len = 4u + (uint32_t)piece_count * 20u;
    if ((uint32_t)body_len != expected_len) {
        send_load_curve_response(correlation_id, KALICO_ERR_INVALID_CURVE, 0);
        return;
    }
    if (!runtime_handle) {
        send_load_curve_response(correlation_id, KALICO_ERR_NOT_INIT, 0);
        return;
    }
    uint32_t handle_packed = 0;
    int32_t rc = runtime_handle_load_curve_cubic(
        runtime_handle, slot_idx, axis_idx, piece_count,
        &body[4], &handle_packed);
    send_load_curve_response(correlation_id, rc, handle_packed);
}

static void
handle_push_segment(uint32_t correlation_id, const uint8_t *body, uint16_t body_len)
{
    extern volatile uint32_t handle_push_segment_calls_total;
    extern volatile uint32_t handle_push_segment_invalid_body_total;
    extern volatile uint32_t handle_push_segment_no_handle_total;
    extern volatile int32_t handle_push_segment_last_r;
    extern void runtime_diag_progress(uint32_t tag, uint32_t stage, uint32_t value);
    handle_push_segment_calls_total++;
    // Independent diag signal — if the counter remains at 0 across many
    // host PushSegment writes, this confirms it via the 0xCC stage-1 tag
    // (visible as a value 0xCC01...XX in fault_detail). The counter is
    // .bss; the diag is foreground-overwriteable, so they cross-check.
    runtime_diag_progress(0xCC, 1, body_len);
    // §7.4 body: id u32, 4×handle u32, t_start u64, t_end u64, kin u8, e_mode u8, extrusion_ratio f32 — 42 bytes.
    if (body_len != 42) {
        handle_push_segment_invalid_body_total++;
        send_push_segment_response(correlation_id, KALICO_ERR_INVALID_CURVE, 0, 0);
        return;
    }
    if (!runtime_handle) {
        handle_push_segment_no_handle_total++;
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
    // Diag: surface the parsed handle values so we can tell whether the
    // host is sending UNUSED sentinels (0xFFFFFFFF) or real handles.
    // 0xCD20 already covers per-frame dispatch; reuse with stage 0xD0..0xD3
    // for the 4 handles, but pack only the low 16 bits (slot_idx + low gen
    // bits) to fit in the value field.
    runtime_diag_progress(0xCE, 0xD0, x_handle & 0xFFFFu);
    runtime_diag_progress(0xCE, 0xD1, y_handle & 0xFFFFu);
    runtime_diag_progress(0xCE, 0xD2, z_handle & 0xFFFFu);
    runtime_diag_progress(0xCE, 0xD3, e_handle & 0xFFFFu);
    int32_t r = runtime_handle_push_segment(
        runtime_handle, id, x_handle, y_handle, z_handle, e_handle,
        t_start, t_end, kinematics, e_mode, extrusion_ratio_bits,
        &accepted_id, &credit_epoch);
    handle_push_segment_last_r = r;
    if (r == 0 /* KALICO_OK */) {
        // The new stepping path (TIM5 sample-driven cubic Bezier eval) has
        // no producer timer to arm here — the ISR dequeues segments from
        // the SPSC queue directly. Stepping-redesign-finish Task 17.
    }
    send_push_segment_response(correlation_id, r, accepted_id, credit_epoch);
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
// ResetCurvePool handler (0x0050)
// ---------------------------------------------------------------------------
//
// Called by the host during `init_planner` on every klippy reconnect where the
// MCU was not power-cycled. Sets `last_retired_gen = current_gen` for every
// curve pool slot so that all slots satisfy the alloc predicate
// (`current_gen == last_retired_gen`) before the new session's first
// `LoadCurveCubic` arrives. Without this, slots that held live curves in the
// prior session keep `current_gen != last_retired_gen` forever (the retirement
// trace events that would normally equalise them never fire after a host
// restart), causing every subsequent `load_curve` to fail with "slot busy".
//
// The FFI calls `CurvePool::reset_all_retired_to_current` which is a pure
// foreground operation (safe to call from the kalico message-dispatch task —
// same execution context as all other command handlers).

static void
send_reset_curve_pool_response(uint32_t correlation_id, int32_t result)
{
    uint8_t payload[PER_MESSAGE_HEADER_LEN + 4];
    encode_message_header(payload, KALICO_MSG_RESET_CURVE_POOL_RESPONSE,
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
handle_reset_curve_pool(uint32_t correlation_id, const uint8_t *body,
                        uint16_t body_len)
{
    (void)body;
    (void)body_len; // request body is empty
    if (!runtime_handle) {
        send_reset_curve_pool_response(correlation_id, KALICO_ERR_NOT_INIT);
        return;
    }
    int32_t r = kalico_runtime_reset_curve_pool(runtime_handle);
    send_reset_curve_pool_response(correlation_id, r);
}

// ---------------------------------------------------------------------------
// Event emitters (Phase C — events channel, fire-and-forget, correlation_id=0)
// ---------------------------------------------------------------------------

void
kalico_native_emit_status_event(uint8_t engine_status, uint8_t queue_depth,
                                uint32_t current_segment_id,
                                int32_t last_fault, uint32_t fault_detail,
                                uint32_t retired_through_segment_id)
{
    // v2 (2026-05-17): body is 22 bytes — added `retired_through_segment_id`
    // u32 tail field. The 10 Hz periodic status frame is now the load-bearing
    // credit-flow signal; the host advances its slot-pool watermark from this
    // field on every status frame so the lossy fire-and-forget
    // `kalico_native_emit_credit_freed` path is no longer required for
    // correctness.
    uint8_t payload[PER_MESSAGE_HEADER_LEN + 22];
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
    b[18] = (uint8_t)(retired_through_segment_id & 0xFF);
    b[19] = (uint8_t)((retired_through_segment_id >> 8) & 0xFF);
    b[20] = (uint8_t)((retired_through_segment_id >> 16) & 0xFF);
    b[21] = (uint8_t)((retired_through_segment_id >> 24) & 0xFF);
    kalico_transport_send_frame(KALICO_CHANNEL_EVENTS, payload, sizeof(payload));
}

int
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
    return kalico_transport_send_frame(
        KALICO_CHANNEL_EVENTS, payload, sizeof(payload));
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
