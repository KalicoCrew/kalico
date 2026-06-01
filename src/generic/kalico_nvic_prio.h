#ifndef __KALICO_NVIC_PRIO_H
#define __KALICO_NVIC_PRIO_H
// Single home for the motion-related NVIC priority numbers.
//
// The motion pair — the TIM5 sample producer and the dedicated step-output
// consumer timer (TIM3 on H7, TIM2 on F4) — sits CO-EQUAL with USB OTG_FS and
// CAN/FDCAN at NVIC priority 1.
//
// HISTORY + WHY CO-EQUAL (2026-06-01): the motion pair was briefly lifted to
// prio 0 (ABOVE USB) on a theory that a USB burst fenced the motion tick. That
// theory is DISPROVEN — -311 was a clock-domain miscalibration (TIM5 ran at
// half rate; fixed by programming ARR from the true timer kernel clock, see the
// runtime_tick ARR fix + fault_handler.c tim5_ia_*). The prio-0 lift not only
// never fixed -311, it actively broke USB: with motion above USB AND now
// running at full rate, sustained jogs starved the USB OTG_FS path and the
// bulk-IN endpoint HALTED (host EPIPE -> critical-MCU abort). USB on this MCU is
// IRQ+FIFO+CPU-copy (no DMA) AND the packet movement happens in the cooperative
// foreground (the ISR only sets wake flags), so motion-above-USB starved BOTH
// the OTG ISR (NVIC axis) and — via the back-to-back step-output re-arm chain —
// the foreground drain (cooperative axis).
//
// FIX (two parts, both C-side): (1) THIS — drop motion from 0 to 1, co-equal
// with USB. PRIGROUP=0 + tail-chaining then bound USB<->motion mutual fencing to
// ONE run-to-completion ISR each (never the multi-ISR burst), so neither
// starves the other. (2) a yield-floor on the step-output re-arm chain
// (STEP_OUT_YIELD_MIN_DWT in runtime_tick_{h7,f4}.c) that guarantees the
// cooperative foreground a CPU window between dense fires — fixing the
// foreground axis that priority alone cannot. USB-ABOVE-motion is rejected: a
// >=138164-cyc USB burst would exceed the 40 kHz TIM5 fence budget by ~6x and
// re-trip -311.
//
// NVIC map (lower number = more urgent; both motion MCUs have
// __NVIC_PRIO_BITS = 4 and PRIGROUP = 0, so all 4 bits preempt and same-number
// interrupts cannot nest — they run to completion):
//   1  motion pair: TIM5 (runtime_tick_{h7,f4}.c) + step-output TIM3/TIM2;
//      CO-EQUAL with USB OTG_FS (usbotg.c, literal 1) + CAN/FDCAN (fdcan.c)
//   2  serial / USART (serial.c) — not built on the USB-CDC bench configs
//   3  SysTick scheduler (armcm_timer.c)
// (USB/CAN still use a literal `1` at their armcm_enable_irq sites; if those
// ever change, this co-equality must be preserved or USB starvation returns.)
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
// Co-equal with USB/CAN (prio 1): keeps the producer/consumer pair equal (the
// SPSC non-nesting invariant above) and bounds USB<->motion mutual fencing to
// one ISR via PRIGROUP=0 tail-chaining.
#define KALICO_MOTION_NVIC_PRIO 1

// SysTick scheduler — below the motion pair.
#define KALICO_SCHED_NVIC_PRIO 3

#endif // __KALICO_NVIC_PRIO_H
