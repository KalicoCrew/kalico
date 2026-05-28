// src/generic/runtime_tick.h
//
// Per-family runtime-tick backend interface. Implementations live in
// src/<arch>/runtime_tick_<family>.c and are selected at build time by the
// architecture-specific Makefile. The host-process simulator implementation
// lives in src/linux/runtime_tick_host.c.
//
// Lifecycle:
//   runtime_tick_init()    configures peripheral / IRQ / counter source AND
//                          starts the tick. Called once at boot. On STM32 this
//                          arms TIM5 at the end of init (free-running from boot,
//                          no arm gate); on Linux it spawns the host pthread
//                          that drives the tick.
//   runtime_tick_enable()  idempotent re-arm of the tick. On STM32 this is a
//                          no-op once init has already armed TIM5. Called from
//                          command_kalico_configure_axis; on the Linux sim build
//                          it performs the widen-seed + step-queue install and
//                          enables the host tick. (The old "producer protocol /
//                          first segment push" arming framing is gone — TIM5 is
//                          always-on now.) New backends MUST audit their
//                          host-clock-frame seeding requirements.
//   runtime_tick_disable() stops ticking. Now called only from the
//                          DECL_SHUTDOWN handler (TIM5 off on Klipper shutdown).
//                          Safe from foreground at any time.
//   runtime_cyccnt_read()  free-running cycle counter, wraps modulo 2^32.
//                          Consecutive calls observe monotone non-decreasing
//                          values modulo wrap; runtime widens to u64 host-side.
//                          DWT->CYCCNT on Cortex-M; monotonic-clock-derived
//                          on Linux.

#ifndef RUNTIME_TICK_H
#define RUNTIME_TICK_H

#include <stdint.h>

void runtime_tick_init(void);
void runtime_tick_enable(void);
void runtime_tick_disable(void);
uint32_t runtime_cyccnt_read(void);

#endif // RUNTIME_TICK_H
