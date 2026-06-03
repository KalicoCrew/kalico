# MCU Log Endpoint — Stage 2 (MCU C Transport) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Put MCU structured-log frames on the wire — a C-owned log ring + `kalico_log_emit` + a drain extension that transmits `KALICO_MSG_LOG (0x0084)`, proven end-to-end in the sim playground.

**Architecture:** A fixed-size C ring in plain `.bss` (DTCM on H7 — non-cached, coherent, no cache maintenance, mirroring `step_queue.c`; the ring is drained continuously so it needs no reset-persistence, unlike `diag_ring`'s `.bkp_bss`). `kalico_log_emit(level, subsystem, event, code, arg0, arg1)` is the sole ABI seam (boundary §B3: no Rust-typed structure crosses; both C and the Rust engine call this `extern "C"` function). It captures the raw 32-bit `timer_read_time()` under `irq_save`/`irq_restore` (NOT `irq_disable` — OTG NVIC prio 1 preempts TIM5 prio 2; the `diag_ring_push` pattern). The existing `runtime_drain` DECL_TASK (~1 kHz, foreground) drains the ring: it widens each entry's stored 32-bit tick to `u64` against the current widened clock (the `arrival_clock`/`piece_sink_commit` pattern), builds the 7-byte per-message header + 24-byte body, and transmits on `KALICO_CHANNEL_EVENTS`. The Stage-1 host side (decode `0x0084` → `RuntimeEvent::McuLog` → re-emit hook → `events/mcu-h7.jsonl` → Vector → VictoriaLogs) is already shipped and wired.

**Tech Stack:** C (Klipper/Kalico MCU firmware, MACH_LINUX sim + STM32H7/F4), Rust (`runtime` crate resolution tables), the kalico-native transport, the sim playground (Docker: klippy + MCUs + Moonraker/Mainsail + Vector + VictoriaLogs).

**Scope discipline (carried from Stage 1's over-reach):** logging changes only. Do NOT fix unrelated clippy/tests or touch non-logging code. Our code is expected to fail/sputter; Stage 2 only proves log frames are written.

**Wire layout (source of truth: `rust/kalico-protocol/src/messages.rs::McuLog`):** body is fixed 24 bytes LE — `mcu_tick:u64 (0..8)`, `level:u8 (8)`, `subsystem:u8 (9)`, `event:u16 (10..12)`, `code:u16 (12..14)`, `seq:u16 (14..16)`, `arg0:u32 (16..20)`, `arg1:u32 (20..24)`. Preceded by the 7-byte per-message header `type:u16_le | version:u8 | corr_id:u32_le` (`KALICO_MSG_MCU_LOG=0x0084`, version `0x01`, corr_id `0`). Host strips the 7-byte header (`decode_message_header`) then calls `KMcuLog::decode(body)`.

---

## File Structure

- `rust/runtime/src/log_codes.rs` (modify): add `EVENT_RUNTIME_MCU_READY` + `event_info` arm + test. The C boot marker uses this so the host resolves a real name (not `"unknown"`).
- `src/kalico_log.h` (create): the `extern "C"` interface — `kalico_log_emit`, `kalico_log_drain`, level `#define`s, and the minimal C-side subsystem/event code mirrors for the one C emit site.
- `src/kalico_log.c` (create): the ring, `kalico_log_emit`, the widener, `send_log_frame`, `kalico_log_drain`.
- `src/Makefile` (modify): register `kalico_log.c` in the kalico-native transport `src-y` group (built for stm32, f4, and linux-sim — same as `kalico_dispatch.c`).
- `src/runtime_tick.c` (modify): `#include "kalico_log.h"`; in `runtime_drain`, one-shot boot emit + `kalico_log_drain()`.

---

### Task 1: Resolvable boot-marker event code (Rust)

**Files:**
- Modify: `rust/runtime/src/log_codes.rs`

This is the only `rust/` change → dispatch a **rust-engineer** subagent. Adding an event code does NOT touch `schema_def.rs` / the wire schema / `KALICO_SCHEMA_HASH` — `log_codes.rs` is host-side resolution only, so there is no deploy-lockstep concern.

- [ ] **Step 1: Add the const** (runtime subsystem; 1 and 2 are taken):

```rust
/// The MCU firmware runtime is up and the log drain is online (emitted once
/// per boot from the C `runtime_drain` task). No args.
pub const EVENT_RUNTIME_MCU_READY: u16 = 3;
```

- [ ] **Step 2: Add the `event_info` arm** (in the `match (subsystem, event)`):

```rust
        (SUBSYSTEM_RUNTIME, EVENT_RUNTIME_MCU_READY) => {
            ("runtime.mcu_ready", "mcu firmware ready, log drain online")
        }
```

- [ ] **Step 3: Add a unit test** (alongside the existing doctests) asserting resolution:

```rust
#[test]
fn mcu_ready_resolves() {
    let (name, tmpl) = event_info(SUBSYSTEM_RUNTIME, EVENT_RUNTIME_MCU_READY);
    assert_eq!(name, "runtime.mcu_ready");
    assert_eq!(tmpl, "mcu firmware ready, log drain online");
}
```

- [ ] **Step 4: Verify** — `cargo test -p runtime log_codes` (and the crate doctests) pass.

- [ ] **Step 5: Commit** (after the C side too, or separately) — `git add rust/runtime/src/log_codes.rs`.

---

### Task 2: `src/kalico_log.h` — the C interface

**Files:**
- Create: `src/kalico_log.h`

- [ ] **Step 1: Write the header** verbatim:

```c
#ifndef KALICO_LOG_H
#define KALICO_LOG_H

#include <stdint.h>

// Wire log levels — MUST match rust/motion-bridge/src/mcu_log.rs::mcu_level_str
// and the McuLog wire-layout doc in rust/kalico-protocol/src/messages.rs.
#define KALICO_LOG_LEVEL_TRACE 0
#define KALICO_LOG_LEVEL_DEBUG 1
#define KALICO_LOG_LEVEL_WARN  2
#define KALICO_LOG_LEVEL_ERROR 3

// Subsystem / event codes used by C-side emit sites. These MIRROR the
// canonical table in rust/runtime/src/log_codes.rs — keep in sync. Rust emit
// sites (Stage 3, fault_helpers.rs) use the Rust constants directly; only the
// C boot marker needs these mirrors, so the drift surface is one pair.
#define KALICO_LOG_SUBSYS_RUNTIME 0
#define KALICO_LOG_EVENT_RUNTIME_MCU_READY 3

// Enqueue one structured log entry into the C-owned ring. Safe from ISR or
// foreground (irq_save critical section). Captures the raw 32-bit
// timer_read_time() now; the drain widens it to u64 before transmit. Drops
// (with accounting) when the ring is full — never blocks. The Rust motion
// engine and C both call this; it is the only ABI seam (boundary §B3).
void kalico_log_emit(uint8_t level, uint8_t subsystem, uint16_t event,
                     uint16_t code, uint32_t arg0, uint32_t arg1);

// Drain the ring and transmit KALICO_MSG_LOG (0x0084) on KALICO_CHANNEL_EVENTS.
// Foreground-only (calls runtime_widened_host_clock()). Called from the
// runtime_drain DECL_TASK (~1 kHz). Stops on transmit_buf backpressure and
// retries the un-sent entry on the next drain.
void kalico_log_drain(void);

#endif // KALICO_LOG_H
```

- [ ] **Step 2: Commit** with `src/kalico_log.c` (Task 3).

---

### Task 3: `src/kalico_log.c` — the ring + emit + drain

**Files:**
- Create: `src/kalico_log.c`

- [ ] **Step 1: Write the file** verbatim:

```c
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
```

- [ ] **Step 2: Commit** `src/kalico_log.c` + `src/kalico_log.h`.

---

### Task 4: Register the file in the build

**Files:**
- Modify: `src/Makefile`

- [ ] **Step 1: Add `kalico_log.c`** to the kalico-native transport `src-y` group so it builds for stm32, f4, AND linux-sim (same group as `kalico_dispatch.c`):

```make
src-y += kalico_demux.c kalico_dispatch.c \
    runtime_storage.c runtime_panic.c step_queue.c \
    spi_queue.c kalico_log.c
```

- [ ] **Step 2: Commit.**

---

### Task 5: Wire the drain + boot marker into `runtime_drain`

**Files:**
- Modify: `src/runtime_tick.c`

- [ ] **Step 1: Add the include** near the other kalico includes (after `#include "kalico_dispatch.h"`):

```c
#include "kalico_log.h"      // kalico_log_emit, kalico_log_drain
```

- [ ] **Step 2: Append to the end of `runtime_drain`** (just before the closing `}` of the function, after the `last_seen_status` block):

```c
    // Observability spec #2 Stage 2: one-shot "MCU runtime ready" structured
    // log on the first drain after the runtime is up (this point guarantees the
    // transport is identified and the host is listening). Permanent per-boot
    // marker; doubles as the 0x0084 end-to-end proof. Then ship queued entries.
    static uint8_t kalico_log_boot_emitted;
    if (!kalico_log_boot_emitted) {
        kalico_log_boot_emitted = 1;
        kalico_log_emit(KALICO_LOG_LEVEL_DEBUG, KALICO_LOG_SUBSYS_RUNTIME,
                        KALICO_LOG_EVENT_RUNTIME_MCU_READY, 0, 0, 0);
    }
    kalico_log_drain();
```

- [ ] **Step 3: Commit.**

---

### Task 6: Build verification

- [ ] **Step 1: Rust** — `cd rust && cargo test -p runtime log_codes && cargo test -p kalico-protocol mcu_log` pass (the host codec + the new event resolution).

- [ ] **Step 2: Sim firmware (MACH_LINUX) build** — proves the C compiles and links against the Rust archive. From the repo root:

```bash
cp tools/kalico-sim/configs/h7-sim.config .config
make olddefconfig 2>/dev/null || true
make clean && make -j$(sysctl -n hw.ncpu)
```

Expected: `out/klipper.elf` builds with no errors referencing `kalico_log`. Repeat with `f4-sim.config` to confirm the F4 path (`make clean` between — msgid/oid hygiene).

- [ ] **Step 3:** Restore any scratch `.config` so the working tree stays clean (the `.config` is not tracked; confirm `git status` shows only the intended files).

---

### Task 7: Playground end-to-end acceptance

- [ ] **Step 1: Rebuild** the sim firmware image + playground image so the new `kalico_log.c` is in the running MCUs:

```bash
# from tools/kalico-playground/ — rebuilds FROM the kalico-sim image
docker compose build
docker compose up -d
```

- [ ] **Step 2: Drive runtime activity** if needed — the boot marker fires on the first `runtime_drain` after `runtime_handle` is set (klippy connect configures the runtime at idle, so it should fire at startup). If it doesn't appear, issue a small jog in Mainsail (sim — no hardware, gcode is safe) to force runtime activity.

- [ ] **Step 3: Query VictoriaLogs** for the MCU log (vmui at `:9428/select/vmui`, or the `query-logs` skill):

```logsql
source:=mcu-h7 | sort by (_time desc) | limit 20
```

Expected: a record with `source=mcu-h7`, `subsystem=runtime`, `event=runtime.mcu_ready`, `_msg="mcu firmware ready, log drain online"`, `level=debug`, a sane RFC3339 `_time`, a `mcu_tick` u64, `seq`, and the current `session_id`. Confirm the same for `mcu-f4` if both MCUs are in the playground config.

- [ ] **Step 4: Confirm no decode errors** — `docker compose logs printer | grep -i "McuLog decode failed"` is empty (byte layout matches the host codec).

---

## Verification of success

Stage 2 is done when: (1) the sim firmware builds for H7-sim and F4-sim; (2) `cargo test` for the touched crates passes; (3) a `source:=mcu-h7` `runtime.mcu_ready` record appears in VictoriaLogs via the live playground with a correct `_time`/`mcu_tick`/`session_id` and no host-side decode errors. Nothing in the Rust engine emits yet — that is Stage 3 (`fault_helpers.rs raise_*` + level gating + ring-overflow drop frames).
