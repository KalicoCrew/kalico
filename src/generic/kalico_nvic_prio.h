#ifndef __KALICO_NVIC_PRIO_H
#define __KALICO_NVIC_PRIO_H
// Single home for the motion-related NVIC priority numbers.
//
// Motion-tick priority-lift (docs/superpowers/specs/2026-05-31-motion-tick-
// priority-lift-design.md), now APPLIED. The motion pair — the TIM5 sample
// producer and the dedicated step-output consumer timer (TIM3 on H7, TIM2 on
// F4) — is the highest maskable interrupt priority, ABOVE USB / CAN / serial /
// SysTick.
//
// Why: this fixes -311 TickIntervalExceeded. The blocker (bench-measured,
// usb_burst=129690 cyc ~= 249 us ~= 10 sample periods on the H7) was the USB
// OTG_FS ISR (prio 1) running a contiguous back-to-back burst while receiving
// the piece stream, fencing the motion tick. That fence is the OTG ISR
// *running at NVIC priority* — the handler holds NO PRIMASK (verified:
// src/stm32/usbotg.c:416-466 has no cpsid/irq_disable; usb_irq_disable masks
// only the OTG line via NVIC, not the global flag). So it is priority-
// defeatable: TIM5 raised above prio 1 preempts the running OTG ISR via
// Cortex-M exception nesting. (Contrast SysTick, which fences via PRIMASK and
// is priority-immune — but its PRIMASK holds are bounded << 1 tick, so
// demoting it below motion is safe and also removes its same-priority fence.)
//
// Resulting NVIC map (lower number = more urgent; both motion MCUs have
// __NVIC_PRIO_BITS = 4 and PRIGROUP = 0, so all 4 bits preempt and same-number
// interrupts cannot nest — they run to completion):
//   0  motion pair: TIM5 (runtime_tick_{h7,f4}.c) + step-output TIM3/TIM2
//   1  USB OTG_FS (usbotg.c), CAN/FDCAN (fdcan.c) — now preemptible by motion.
//      Bulk USB-CDC tolerates the ~2.6 us/tick preemption (wire-bound, NAK
//      flow-controlled); heater comms keep this high priority.
//   2  serial / USART (serial.c) — not built on the USB-CDC bench configs
//   3  SysTick scheduler (armcm_timer.c)
//
// LOAD-BEARING INVARIANT: TIM5 and the step-output timer MUST stay EQUAL to
// each other (both KALICO_MOTION_NVIC_PRIO). PRIGROUP=0 means equal-priority
// interrupts cannot nest, which is exactly what keeps the step_queue SPSC and
// the kalico_kick_step_output compare-register poke non-racing (producer =
// TIM5, consumer = step-output timer can never interleave). If a future change
// ever splits their priorities, the volatile SPSC must become true-atomic
// first. Both timers read this one constant, so they move together by
// construction.
//
// Heater-off safety is INDEPENDENT of all of this: MCU-side max_duration (2 s
// default) auto-shutoff plus the ~0.5 s IWDG hardware watchdog, neither of
// which depends on interrupt priority.

// TIM5 motion-sample producer + the dedicated step-output consumer timer.
// Highest maskable priority so the motion tick preempts USB / CAN / serial /
// SysTick.
#define KALICO_MOTION_NVIC_PRIO 0

// SysTick scheduler — below the motion pair so a dispatch burst can never fence
// the tick by priority. Its residual PRIMASK holds (~3.5 us H7 / ~19 us F4)
// are far under the 2-period (-311) threshold.
#define KALICO_SCHED_NVIC_PRIO 3

#endif // __KALICO_NVIC_PRIO_H
