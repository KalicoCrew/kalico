#ifndef KALICO_LOG_H
#define KALICO_LOG_H

#include <stdint.h>

// Wire log levels — MUST match rust/motion-bridge/src/mcu_log.rs::mcu_level_str
// and the McuLog wire layout in rust/kalico-protocol/src/messages.rs.
#define KALICO_LOG_LEVEL_TRACE 0
#define KALICO_LOG_LEVEL_DEBUG 1
#define KALICO_LOG_LEVEL_WARN  2
#define KALICO_LOG_LEVEL_ERROR 3

// MUST mirror the canonical table in rust/runtime/src/log_codes.rs.
#define KALICO_LOG_SUBSYS_RUNTIME 0
#define KALICO_LOG_SUBSYS_ENDSTOP 3
#define KALICO_LOG_SUBSYS_DIAG    4
#define KALICO_LOG_EVENT_RUNTIME_MCU_READY 3
#define KALICO_LOG_EVENT_RUNTIME_LOG_DROPS 4
#define KALICO_LOG_EVENT_RUNTIME_MCU_RESET 5
#define KALICO_LOG_EVENT_RUNTIME_HARD_FAULT 6
#define KALICO_LOG_EVENT_RUNTIME_FAULT_STATUS 7
#define KALICO_LOG_EVENT_RUNTIME_FG_FREEZE 8
#define KALICO_LOG_EVENT_RUNTIME_RT_PROGRESS 9
#define KALICO_LOG_EVENT_RUNTIME_LAST_DISPATCH 10
#define KALICO_LOG_EVENT_RUNTIME_ISR_PHASE     11
#define KALICO_LOG_EVENT_RUNTIME_BLOCK_SOURCE  12
#define KALICO_LOG_EVENT_RUNTIME_TIM5_IA       13
#define KALICO_LOG_EVENT_RUNTIME_DIAG_DUMP     14
#define KALICO_LOG_EVENT_RUNTIME_STEPOUT_LATE  15

#define KALICO_LOG_EVENT_ENDSTOP_TRSYNC_TRIGGER_CMD  3
#define KALICO_LOG_EVENT_ENDSTOP_TRSYNC_DO_TRIGGER   4
#define KALICO_LOG_EVENT_ENDSTOP_STOP_CB_ENTER       5
#define KALICO_LOG_EVENT_ENDSTOP_TIM5_HALTED         7

// Safe from ISR or foreground (irq_save critical section); drops on full ring,
// never blocks. The only ABI seam — both Rust and C call it.
void kalico_log_emit(uint8_t level, uint8_t subsystem, uint16_t event,
                     uint16_t code, uint32_t arg0, uint32_t arg1);

// Foreground-only (calls runtime_widened_host_clock()).
void kalico_log_drain(void);

#endif
