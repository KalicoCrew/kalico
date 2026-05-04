// Layer 4 dispatcher + frame builder for the kalico-native transport.
//
// Spec: docs/superpowers/specs/2026-05-04-kalico-native-transport-design.md.
// Phase B scope: Identify -> IdentifyResponse handshake. Other handlers are
// stubs that return KALICO_ERR_NOT_IMPLEMENTED via a FaultEvent-style frame
// (or, for Phase B, simply log and drop — the dispatch table is here so we
// can fill in handlers in Phase C without touching the demux/RX path).

#include <stdint.h>
#include <string.h>
#include "kalico_dispatch.h"
#include "kalico_demux.h"
#include "kalico_protocol_schema.h" // KALICO_MSG_*, KALICO_SCHEMA_HASH
#include "board/misc.h"             // crc16_ccitt
#include "sched.h"                  // DECL_INIT

// Forward decl: platform-specific raw byte output. The Linux sim implements
// this in linux/console.c; firmware builds will implement it on top of the
// USB CDC TX path (Phase D).
extern int kalico_console_write_raw(const uint8_t *buf, uint16_t len);

#define KALICO_FRAME_SYNC 0x55
#define KALICO_CHANNEL_CONTROL 0x00

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
    uint16_t crc = crc16_ccitt(&tx_buf[1], (uint_fast8_t)(2 + 1 + payload_len));
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
    (void)channel;
    if (payload_len < PER_MESSAGE_HEADER_LEN)
        return;
    uint16_t kind = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
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
    case KALICO_MSG_PUSH_SEGMENT:
        // Phase C — not implemented. For Phase B we silently drop; once
        // FaultEvent / response handling lands we'll surface NOT_IMPLEMENTED.
        return;
    default:
        return;
    }
}
