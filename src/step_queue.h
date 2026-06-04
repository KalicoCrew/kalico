// SPSC step queue per motor axis. Producer = TIM5 ISR (Rust); consumer =
// per-axis Klipper timer (Rust, via SysTick dispatch). Storage C-owned per the
// B2/B3 invariant in docs/kalico-rewrite/mcu-c-rust-boundary.md. The struct
// layout mirrors Rust #[repr(C)] — keep in sync (static_asserts below).

#ifndef __KALICO_STEP_QUEUE_H
#define __KALICO_STEP_QUEUE_H

#include <stdint.h>
#include <stddef.h>

#define STEP_QUEUE_DEPTH       32
#define STEP_QUEUE_DEPTH_MASK  0x1F  // depth - 1; power-of-2 invariant
#define N_AXIS_STEP_QUEUES     4     // A, B, Z, E

typedef struct {
    uint32_t cycle_abs;   // low 32 bits of DWT CYCCNT; wrap-aware compare only
    int8_t   dir;
    uint8_t  _pad[3];
} StepEntry;

typedef struct {
    volatile uint16_t tail;
    volatile uint16_t head;
    uint8_t  _pad[4];
    StepEntry buf[STEP_QUEUE_DEPTH];
} StepQueue;

extern StepQueue step_queues[N_AXIS_STEP_QUEUES];

_Static_assert(sizeof(StepEntry) == 8, "StepEntry layout drift");
_Static_assert(sizeof(StepQueue) == 264, "StepQueue layout drift");
_Static_assert(offsetof(StepQueue, buf) == 8, "StepQueue.buf offset drift");
_Static_assert((STEP_QUEUE_DEPTH & STEP_QUEUE_DEPTH_MASK) == 0,
               "STEP_QUEUE_DEPTH must be power of 2");

#endif // __KALICO_STEP_QUEUE_H
