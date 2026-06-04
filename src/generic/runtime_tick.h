// Per-family runtime-tick backend interface. Implementations in
// src/<arch>/runtime_tick_<family>.c (host sim in src/linux/runtime_tick_host.c).
//
//   runtime_tick_init()    configures the counter source AND starts the tick
//                          (called once at boot; TIM5 free-runs from boot).
//   runtime_tick_enable()  idempotent re-arm; on the Linux sim also does the
//                          widen-seed + step-queue install.
//   runtime_tick_disable() stops ticking; called from the DECL_SHUTDOWN handler.
//   runtime_cyccnt_read()  free-running counter, wraps modulo 2^32; consecutive
//                          calls observe monotone-non-decreasing-modulo-wrap
//                          values. DWT->CYCCNT on Cortex-M, monotonic-derived
//                          on Linux.

#ifndef RUNTIME_TICK_H
#define RUNTIME_TICK_H

#include <stdint.h>

void runtime_tick_init(void);
void runtime_tick_enable(void);
void runtime_tick_disable(void);
uint32_t runtime_cyccnt_read(void);

#endif // RUNTIME_TICK_H
