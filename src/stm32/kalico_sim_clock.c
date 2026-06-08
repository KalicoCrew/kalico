// Software CYCCNT for sim builds (CONFIG_KALICO_SIM=y): Renode's H7 model
// returns 0 for DWT->CYCCNT, so the TIM5 ISR bumps this counter to give the
// widening loop forward progress.

#include "autoconf.h"

#if CONFIG_KALICO_SIM && (CONFIG_MACH_STM32H7 || CONFIG_MACH_STM32F4)

#include <stdint.h>
#include "command.h"

__attribute__((used, externally_visible))
volatile uint32_t runtime_sim_cyccnt = 0;

#endif
