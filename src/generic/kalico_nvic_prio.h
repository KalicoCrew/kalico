#ifndef __KALICO_NVIC_PRIO_H
#define __KALICO_NVIC_PRIO_H
// Single home for the motion-related NVIC priority numbers.
//
// NVIC map (lower = more urgent; both motion MCUs have __NVIC_PRIO_BITS = 4 and
// PRIGROUP = 0, so all 4 bits preempt and same-number interrupts cannot nest —
// they run to completion):
//   0  serial / USART (serial.c) — not built on the USB-CDC bench configs
//   1  USB OTG_FS (usbotg.c), CAN/FDCAN (fdcan.c)
//   2  motion pair: TIM5 (runtime_tick_{h7,f4}.c) + step-output TIM3/TIM2,
//      and SysTick scheduler (armcm_timer.c)
//
// LOAD-BEARING INVARIANT: TIM5 (producer) and the step-output timer (consumer)
// MUST stay EQUAL to each other (both KALICO_MOTION_NVIC_PRIO). PRIGROUP=0 means
// equal-priority interrupts cannot nest, which is what keeps the step_queue SPSC
// and the kalico_kick_step_output compare-register poke non-racing (the producer
// and consumer can never interleave). If a future change ever splits their
// priorities, the volatile SPSC must become true-atomic first. Both timers read
// this one constant, so they move together by construction.
//
// HISTORY (2026-06-01): lifting the motion pair above USB / SysTick was tried as
// a -311 TickIntervalExceeded fix and REVERTED. It never fixed -311 (root cause
// was a clock-domain miscalibration — TIM5's ARR was programmed from the DWT/CPU
// clock while its kernel clock is CONFIG_CLOCK_FREQ/2; fixed in
// runtime_tick_{h7,f4}.c) and it broke USB-CDC liveness under sustained motion.
// Priorities are back to the pre-experiment baseline. Heater-off safety
// (MCU-side max_duration + IWDG) is priority-independent regardless.

// TIM5 motion-sample producer + the dedicated step-output consumer timer.
#define KALICO_MOTION_NVIC_PRIO 2

// SysTick scheduler — equal to the motion pair (baseline parity).
#define KALICO_SCHED_NVIC_PRIO 2

#endif // __KALICO_NVIC_PRIO_H
