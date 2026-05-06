// src/generic/runtime_tick.h
//
// Per-family runtime-tick backend interface. Implementations live in
// src/<arch>/runtime_tick_<family>.c and are selected at build time by the
// architecture-specific Makefile. The host-process simulator implementation
// lives in src/linux/runtime_tick_host.c.
//
// Lifecycle:
//   runtime_tick_init()    configures peripheral / IRQ / counter source.
//                          Does NOT start ticking. Called once at boot.
//   runtime_tick_enable()  arms the 40 kHz tick. Called by the producer
//                          protocol on first segment push. May have side
//                          effects beyond starting the tick — in particular
//                          the host-sim seeds Klipper's stats_send_time_high
//                          frame from the host clock here. New backends MUST
//                          audit their host-clock-frame seeding requirements.
//   runtime_tick_disable() stops ticking. Safe from foreground at any time.
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
