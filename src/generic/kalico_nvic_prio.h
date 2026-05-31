#ifndef __KALICO_NVIC_PRIO_H
#define __KALICO_NVIC_PRIO_H
// Single home for the two motion-related NVIC priority numbers.
//
// Motion-tick priority-lift (docs/superpowers/specs/2026-05-31-motion-tick-
// priority-lift-design.md). The plan lands in two phases:
//
//   Phase A / Step 1 (THIS step): stand up the dedicated step-output hardware
//     timer at the SAME priority as SysTick (2), so motion is at functional
//     parity with today and independently bench-testable. NO priority flip.
//
//   Phase B (a LATER, separate step): demote SysTick 2 -> 3 so the motion pair
//     (TIM5 producer + step-output consumer) sits strictly above the software
//     scheduler. That is a one-line change to KALICO_SCHED_NVIC_PRIO here.
//
// Numeric NVIC priority: lower = more urgent. On both motion MCUs
// (STM32H723 Cortex-M7, STM32F446 Cortex-M4F) __NVIC_PRIO_BITS = 4 and
// PRIGROUP = 0 (reset default; Klipper never calls NVIC_SetPriorityGrouping),
// so all 4 bits are preemption bits and same-number interrupts cannot preempt
// each other (they run to completion). That non-nesting property is the
// load-bearing invariant that keeps the step_queue SPSC and the
// kalico_kick_step_output compare-register poke non-racing: the TIM5 producer
// and the step-output consumer share KALICO_MOTION_NVIC_PRIO, so they can
// never interleave. Keep them EQUAL to each other across any future change.
//
// Verified current map (serial/CAN @ 0, USB/FDCAN @ 1) is untouched by this
// change; only the motion consumer joins TIM5 at priority 2.

// TIM5 motion-sample producer + the dedicated step-output consumer timer.
#define KALICO_MOTION_NVIC_PRIO 2

// SysTick scheduler. Step 1 keeps this at 2 (parity). Phase B flips it to 3.
#define KALICO_SCHED_NVIC_PRIO 2

#endif // __KALICO_NVIC_PRIO_H
