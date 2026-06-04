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
// C-side emits (the boot marker + the drain's ring-overflow report) need these
// mirrors.
#define KALICO_LOG_SUBSYS_RUNTIME 0
// Stage 5 v1.1 play-by-play: the prior-boot diag event ring is replayed under
// this subsystem with event-code == the DIAG_EV_* tag (1:1, see fault_handler).
#define KALICO_LOG_SUBSYS_DIAG    4
#define KALICO_LOG_EVENT_RUNTIME_MCU_READY 3
#define KALICO_LOG_EVENT_RUNTIME_LOG_DROPS 4
// Stage 5 crash-forensics (emitted from fault_handler.c on the next boot).
#define KALICO_LOG_EVENT_RUNTIME_MCU_RESET 5
#define KALICO_LOG_EVENT_RUNTIME_HARD_FAULT 6
#define KALICO_LOG_EVENT_RUNTIME_FAULT_STATUS 7
#define KALICO_LOG_EVENT_RUNTIME_FG_FREEZE 8
#define KALICO_LOG_EVENT_RUNTIME_RT_PROGRESS 9
// Stage 5 v1.1 crash-cause discriminators (runtime subsystem, emitted with the
// play-by-play when the prior boot ended abnormally).
#define KALICO_LOG_EVENT_RUNTIME_LAST_DISPATCH 10
#define KALICO_LOG_EVENT_RUNTIME_ISR_PHASE     11
#define KALICO_LOG_EVENT_RUNTIME_BLOCK_SOURCE  12
#define KALICO_LOG_EVENT_RUNTIME_TIM5_IA       13

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
