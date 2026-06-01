#ifndef __KALICO_NVIC_PRIO_H
#define __KALICO_NVIC_PRIO_H
// Single home for the motion-related NVIC priority numbers.
//
// The motion pair — the TIM5 sample producer and the dedicated step-output
// consumer timer (TIM3 on H7, TIM2 on F4) — sits at the highest maskable
// interrupt priority, ABOVE USB / CAN / serial / SysTick.
//
// NOTE (2026-06-01): this priority lift was originally applied as a -311
// TickIntervalExceeded fix, on the theory that a USB OTG_FS burst was fencing
// the motion tick at NVIC priority. That theory is DISPROVEN — -311 is a
// clock-domain miscalibration (TIM5 free-runs at half its configured rate
// because its ARR is programmed from the DWT/CPU clock while its kernel clock
// is CONFIG_CLOCK_FREQ/2; see src/generic/fault_handler.c tim5_ia_* and the
// runtime_tick ARR fix). The lift is retained because TIM5-highest is a sound
// ordering on its own, but it is NOT what fixes -311.
//
// NVIC map (lower number = more urgent; both motion MCUs have
// __NVIC_PRIO_BITS = 4 and PRIGROUP = 0, so all 4 bits preempt and same-number
// interrupts cannot nest — they run to completion):
//   0  motion pair: TIM5 (runtime_tick_{h7,f4}.c) + step-output TIM3/TIM2
//   1  USB OTG_FS (usbotg.c), CAN/FDCAN (fdcan.c)
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
// Highest maskable priority; keeps the producer/consumer pair equal (the
// SPSC non-nesting invariant above).
#define KALICO_MOTION_NVIC_PRIO 0

// SysTick scheduler — below the motion pair.
#define KALICO_SCHED_NVIC_PRIO 3

#endif // __KALICO_NVIC_PRIO_H
