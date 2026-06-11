#ifndef __KALICO_NVIC_PRIO_H
#define __KALICO_NVIC_PRIO_H

// Single home for the motion-related NVIC priority numbers.
//
// The motion pair — TIM5 sample producer and the dedicated step-output
// consumer timer (TIM3 on H7, TIM2 on F4) — runs at NVIC priority 0, above
// USB OTG_FS / CAN / serial / SysTick.
//
// Root cause this fixes: the USB OTG_FS ISR (priority 1) runs contiguous
// bursts of ~138k cycles (266 µs @ 520 MHz). The step-output ISR fires at
// priority 2 — preemptible by USB — so a single OTG burst delays up to
// 4+ consecutive step-output firings. kalico_step_output_event drains the
// overdue entries back-to-back and runtime_emit_step_pulses emits them as
// GPIO toggles nanoseconds apart — below the TMC5160 DEDGE input-filter
// minimum. Raising motion above USB removes the preemption so catch-up
// batches become rare; the pacing backstop in runtime_emit_step_pulses
// handles any residual ones.
//
// Historical objections disproven:
//   -311 TickIntervalExceeded  — clock-domain bug (fixed separately, KEPT).
//   USB-CDC halts under motion — electrical bus dropouts (fixed in 2c1f0c0b8).
//
// NVIC map (lower = more urgent; both motion MCUs have __NVIC_PRIO_BITS = 4
// and PRIGROUP = 0, so all 4 bits preempt and same-number interrupts cannot
// nest — they run to completion):
//   0  motion pair: TIM5 (runtime_tick_{h7,f4}.c) + step-output TIM3/TIM2
//   1  USB OTG_FS (usbotg.c), CAN/FDCAN (fdcan.c) — preemptible by motion
//   2  serial / USART (serial.c) — not built on USB-CDC bench configs
//   3  SysTick scheduler (armcm_timer.c)
//
// LOAD-BEARING INVARIANT: TIM5 and the step-output timer MUST remain EQUAL
// (both KALICO_MOTION_NVIC_PRIO). PRIGROUP=0 means equal-priority interrupts
// cannot nest, which is what keeps the step_queue SPSC and the
// kalico_kick_step_output compare-register poke non-racing (producer = TIM5,
// consumer = step-output timer, can never interleave). If a future change
// ever splits their priorities, the volatile SPSC must become true-atomic
// first. Both timers read this one constant, so they move together by
// construction.
//
// Heater-off safety is INDEPENDENT of all of this: MCU-side max_duration
// (2 s default) auto-shutoff plus the ~0.5 s IWDG hardware watchdog.

// TIM5 motion-sample producer + dedicated step-output consumer timer.
// Highest maskable priority; MUST be applied to both (SPSC invariant above).
#define KALICO_MOTION_NVIC_PRIO 0

// SysTick scheduler — below the motion pair so dispatch bursts cannot fence
// the tick. Its residual PRIMASK holds (~3.5 µs H7 / ~19 µs F4) are far
// under the one-period (-311) threshold.
#define KALICO_SCHED_NVIC_PRIO 3

#endif // __KALICO_NVIC_PRIO_H
