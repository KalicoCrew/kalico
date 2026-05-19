// SPSC step queue per motor axis. Producer = TIM5 ISR (Rust);
// consumer = per-axis Klipper timer (Rust extern "C", called from
// Klipper SysTick dispatch). Storage C-owned per architectural
// invariant B2/B3 in docs/kalico-rewrite/mcu-c-rust-boundary.md.
//
// 32-entry SPSC ring with u16 head/tail counters using wrapping
// subtraction for length. Power-of-2 depth allows mask indexing
// (& STEP_QUEUE_DEPTH_MASK) on the hot path.

#ifndef __KALICO_STEP_QUEUE_H
#define __KALICO_STEP_QUEUE_H

#include <stdint.h>
#include <stddef.h>

#define STEP_QUEUE_DEPTH       32
#define STEP_QUEUE_DEPTH_MASK  0x1F  // depth - 1; power-of-2 invariant
#define N_AXIS_STEP_QUEUES     4     // A, B, Z, E

typedef struct {
    uint32_t cycle_abs;   // lower 32 bits of DWT CYCCNT; wrap-aware compare only
    int8_t   dir;         // +1 / -1
    uint8_t  _pad[3];     // explicit padding, matches Rust #[repr(C)]
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
