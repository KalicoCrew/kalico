// Software CYCCNT for sim builds (CONFIG_KALICO_SIM=y): Renode's H7 model
// returns 0 for DWT->CYCCNT, so the TIM5 ISR bumps this counter to give the
// widening loop forward progress.

#include "autoconf.h"

#if CONFIG_KALICO_SIM && (CONFIG_MACH_STM32H7 || CONFIG_MACH_STM32F4)

#include <stdint.h>
#include "command.h"

__attribute__((used, externally_visible))
volatile uint32_t runtime_sim_cyccnt = 0;

// Set a pin level directly in the runtime's PIN_LEVELS array, bypassing GPIO
// hardware — Renode's peripheral model has no GPIO IDR injection.
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
