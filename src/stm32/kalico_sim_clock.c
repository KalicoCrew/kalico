// src/stm32/kalico_sim_clock.c
//
// Software CYCCNT for sim builds (CONFIG_KALICO_SIM=y). Renode's H7 platform
// model returns 0 for DWT->CYCCNT reads; this counter is bumped by the TIM5
// ISR (one-tick-per-fire delta) so the engine's widening loop sees forward
// progress. NEVER include in production firmware — IWDG-disable + sim CYCCNT
// is a debugging build only.
//
// Per Step-6 spec §3.1.

#include "autoconf.h"

#if CONFIG_KALICO_SIM && (CONFIG_MACH_STM32H7 || CONFIG_MACH_STM32F4)

#include <stdint.h>

// Bumped by TIM5 ISR (runtime_tick_h7.c) once per tick.
__attribute__((used, externally_visible))
volatile uint32_t runtime_sim_cyccnt = 0;

#endif
