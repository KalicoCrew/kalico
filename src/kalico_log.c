// C-owned MCU structured-log ring + transport. Observability spec #2 Stage 2.
//
// Producers: the Rust motion engine (via extern "C" kalico_log_emit) and C
// foreground/ISR paths. Consumer: the runtime_drain DECL_TASK (~1 kHz), which
// widens each entry's captured 32-bit tick to u64 and transmits it as
// KALICO_MSG_LOG (0x0084). Boundary §B2/§B3: C owns the ring storage and the
// only ABI seam is kalico_log_emit (no Rust-typed structure crosses the ABI;
// the 2026-05-18 SPSC LLVM-miscompile lesson). irq_save/irq_restore (NOT
// irq_disable) per diag_ring_push — OTG (NVIC prio 1) preempts TIM5 (prio 2).
//
// Memory placement: plain .bss. On H7 that lands in DTCM (non-cached,
// single-cycle), so the ISR producer and foreground consumer share a coherent
// view with NO cache maintenance — matching step_queue.c, whose comment warns
// against .axi_bss (which would reintroduce cache cleans). The ring is drained
// continuously, so it needs no reset-persistence (unlike diag_ring's .bkp_bss).

#include <stdint.h>

#include "board/irq.h"               // irq_save, irq_restore, irqstatus_t
#include "board/misc.h"              // timer_read_time
#include "kalico_dispatch.h"         // kalico_transport_send_frame, KALICO_CHANNEL_EVENTS
#include "kalico_protocol_schema.h"  // KALICO_MSG_MCU_LOG
#include "kalico_log.h"

// runtime_widened_host_clock() lives in src/runtime_tick.c (foreground-safe,
// Klipper-stats-based widening). No public header declares it.
extern uint64_t runtime_widened_host_clock(void);

// Per-message protocol version. Mirrors MESSAGE_VERSION_DEFAULT in
// kalico_dispatch.c (file-local #define, not exported).
#define KALICO_LOG_MSG_VERSION 0x01
// Per-message header: type(u16) | version(u8) | corr_id(u32) = 7 bytes.
#define KALICO_LOG_HEADER_LEN 7
// McuLog body width (messages.rs McuLog wire layout) = 24 bytes.
#define KALICO_LOG_BODY_LEN 24

// Ring capacity (power of two for cheap masking). 64 entries is ample for
// warn/error bursts at the 1 kHz drain rate; ~1 KB in DTCM.
#define KALICO_LOG_RING_LEN 64
#define KALICO_LOG_RING_MASK (KALICO_LOG_RING_LEN - 1)

// One pending log entry. Stores the RAW 32-bit tick captured at emit; the
// drain widens it to u64 just before transmit.
struct kalico_log_entry {
    uint32_t tick;       // raw timer_read_time() at emit
    uint16_t event;
    uint16_t code;
    uint16_t seq;
    uint8_t  level;
    uint8_t  subsystem;
    uint32_t args[2];
};

// Plain .bss → DTCM on H7 (non-cached, coherent). volatile: shared across the
// ISR producer / foreground consumer; the irq_save pair fences ordering.
static volatile struct kalico_log_entry kalico_log_ring[KALICO_LOG_RING_LEN];

// Free-running head/tail counters (NOT masked — unsigned wrap is well-defined
// and head - tail gives the live count). Index = counter & MASK. Touched only
// under irq_save.
static volatile uint32_t kalico_log_head;
static volatile uint32_t kalico_log_tail;
// Per-MCU monotonic sequence assigned to each enqueued entry (truncated to u16
// on the wire for host drop detection).
static volatile uint32_t kalico_log_seq;
// Ring-overflow drop accounting (surfaced as a drop frame in Stage 3).
static volatile uint32_t kalico_log_drops;

void
kalico_log_emit(uint8_t level, uint8_t subsystem, uint16_t event,
                uint16_t code, uint32_t arg0, uint32_t arg1)
{
    irqstatus_t flag = irq_save();
    if ((kalico_log_head - kalico_log_tail) >= KALICO_LOG_RING_LEN) {
        // Ring full: drop newest, account for it. Never block. Spec §7.
        kalico_log_drops++;
        irq_restore(flag);
        return;
    }
    uint32_t idx = kalico_log_head & KALICO_LOG_RING_MASK;
    kalico_log_ring[idx].tick = timer_read_time();
    kalico_log_ring[idx].event = event;
    kalico_log_ring[idx].code = code;
    kalico_log_ring[idx].seq = (uint16_t)(kalico_log_seq & 0xFFFF);
    kalico_log_ring[idx].level = level;
    kalico_log_ring[idx].subsystem = subsystem;
    kalico_log_ring[idx].args[0] = arg0;
    kalico_log_ring[idx].args[1] = arg1;
    kalico_log_head++;
    kalico_log_seq++;
    irq_restore(flag);
}

// Widen a 32-bit tick captured <= 1 ms ago against the current widened clock,
// mirroring the arrival_clock pattern (kalico_dispatch.c::piece_sink_commit):
// if the captured low half exceeds the current low half, the u32 counter
// wrapped between capture and now, so the high half is one less.
static uint64_t
widen_log_tick(uint32_t tick)
{
    uint64_t now64 = runtime_widened_host_clock();   // foreground-safe
    uint32_t now_lo = (uint32_t)now64;
    uint32_t high = (uint32_t)(now64 >> 32);
    if (tick > now_lo)
        high -= 1;
    return ((uint64_t)high << 32) | (uint64_t)tick;
}

// Build + transmit one KALICO_MSG_LOG frame. Returns the send_frame result
// (frame length on success, -1 on transmit_buf overflow).
static int
send_log_frame(const struct kalico_log_entry *e)
{
    uint64_t mcu_tick = widen_log_tick(e->tick);

    uint8_t payload[KALICO_LOG_HEADER_LEN + KALICO_LOG_BODY_LEN];
    // Per-message header: type(u16 LE) | version(u8) | corr_id(u32 LE)=0.
    payload[0] = (uint8_t)(KALICO_MSG_MCU_LOG & 0xFF);
    payload[1] = (uint8_t)((KALICO_MSG_MCU_LOG >> 8) & 0xFF);
    payload[2] = KALICO_LOG_MSG_VERSION;
    payload[3] = 0;
    payload[4] = 0;
    payload[5] = 0;
    payload[6] = 0;
    // Body (LE): mcu_tick u64, level u8, subsystem u8, event u16, code u16,
    // seq u16, arg0 u32, arg1 u32 — must match messages.rs McuLog::decode.
    uint8_t *b = &payload[KALICO_LOG_HEADER_LEN];
    b[0] = (uint8_t)(mcu_tick & 0xFF);
    b[1] = (uint8_t)((mcu_tick >> 8) & 0xFF);
    b[2] = (uint8_t)((mcu_tick >> 16) & 0xFF);
    b[3] = (uint8_t)((mcu_tick >> 24) & 0xFF);
    b[4] = (uint8_t)((mcu_tick >> 32) & 0xFF);
    b[5] = (uint8_t)((mcu_tick >> 40) & 0xFF);
    b[6] = (uint8_t)((mcu_tick >> 48) & 0xFF);
    b[7] = (uint8_t)((mcu_tick >> 56) & 0xFF);
    b[8] = e->level;
    b[9] = e->subsystem;
    b[10] = (uint8_t)(e->event & 0xFF);
    b[11] = (uint8_t)((e->event >> 8) & 0xFF);
    b[12] = (uint8_t)(e->code & 0xFF);
    b[13] = (uint8_t)((e->code >> 8) & 0xFF);
    b[14] = (uint8_t)(e->seq & 0xFF);
    b[15] = (uint8_t)((e->seq >> 8) & 0xFF);
    b[16] = (uint8_t)(e->args[0] & 0xFF);
    b[17] = (uint8_t)((e->args[0] >> 8) & 0xFF);
    b[18] = (uint8_t)((e->args[0] >> 16) & 0xFF);
    b[19] = (uint8_t)((e->args[0] >> 24) & 0xFF);
    b[20] = (uint8_t)(e->args[1] & 0xFF);
    b[21] = (uint8_t)((e->args[1] >> 8) & 0xFF);
    b[22] = (uint8_t)((e->args[1] >> 16) & 0xFF);
    b[23] = (uint8_t)((e->args[1] >> 24) & 0xFF);

    return kalico_transport_send_frame(KALICO_CHANNEL_EVENTS, payload,
                                       (uint16_t)sizeof(payload));
}

void
kalico_log_drain(void)
{
    // Ring-overflow accounting (spec §7): report any entries dropped since the
    // last drain, then reset. Fail-loud — loss is surfaced, never silent. The
    // report is enqueued here so the loop below ships it the same cycle; if the
    // ring is momentarily full the report itself drops and is re-counted next
    // drain (self-healing).
    irqstatus_t df = irq_save();
    uint32_t drops = kalico_log_drops;
    kalico_log_drops = 0;
    irq_restore(df);
    if (drops)
        kalico_log_emit(KALICO_LOG_LEVEL_WARN, KALICO_LOG_SUBSYS_RUNTIME,
                        KALICO_LOG_EVENT_RUNTIME_LOG_DROPS, 0, drops, 0);

    for (;;) {
        struct kalico_log_entry e;
        irqstatus_t flag = irq_save();
        if (kalico_log_head == kalico_log_tail) {
            irq_restore(flag);
            break;                       // ring empty
        }
        // Copy the head-of-queue entry; do NOT advance tail until the TX
        // succeeds, so a transmit_buf-full drop retries on the next drain.
        // The producer drops-on-full (never overwrites the unconsumed tail),
        // so the slot is stable across the TX without holding irq.
        e = kalico_log_ring[kalico_log_tail & KALICO_LOG_RING_MASK];
        irq_restore(flag);

        int rc = send_log_frame(&e);
        if (rc < 0)
            break;                       // transmit_buf full — retry next tick

        flag = irq_save();
        kalico_log_tail++;
        irq_restore(flag);
    }
}
