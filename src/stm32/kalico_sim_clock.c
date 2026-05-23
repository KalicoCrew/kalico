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
#include "command.h"

// Bumped by TIM5 ISR (runtime_tick_h7.c) once per tick.
__attribute__((used, externally_visible))
volatile uint32_t runtime_sim_cyccnt = 0;

// Directly set a pin level in the runtime's endstop PIN_LEVELS array,
// bypassing GPIO hardware. For Renode sim where GPIO IDR injection is
// not supported by the peripheral model.
extern int32_t kalico_endstop_set_pin_level(uint16_t gpio, uint8_t level);

void
command_runtime_sim_endstop_set_pin(uint32_t *args)
{
    uint16_t gpio = args[0];
    uint8_t level = args[1];
    kalico_endstop_set_pin_level(gpio, level);
}
DECL_COMMAND(command_runtime_sim_endstop_set_pin,
             "runtime_sim_endstop_set_pin gpio=%hu level=%c");

#endif
