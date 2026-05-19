// Watchdog handler on STM32
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_MACH_STM32H7
#include "internal.h" // IWDG
#include "sched.h" // DECL_TASK

#if CONFIG_MACH_STM32H7 // stm32h7 libraries only define IWDG1 and IWDG2
#define IWDG IWDG1
#endif

// Spec §5.7 — kalico runtime liveness gate. Foreground (runtime_drain task)
// is the sole writer; this file only reads. __attribute__((used,
// externally_visible)) survives Klipper's -fwhole-program --gc-sections.
volatile uint8_t runtime_liveness_ok __attribute__((used, externally_visible))
    = 1;

void
watchdog_reset(void)
{
#if CONFIG_KALICO_SIM
    return;  // Renode's IWDG model misbehaves; sim build is silicon-unsafe
#endif
    if (!runtime_liveness_ok) return;   // kalico runtime detected liveness fault
    IWDG->KR = 0xAAAA;
}
DECL_TASK(watchdog_reset);

void
watchdog_init(void)
{
#if CONFIG_KALICO_SIM
    return;  // Don't arm IWDG in sim — see watchdog_reset
#endif
    IWDG->KR = 0x5555;
    IWDG->PR = 0;
    IWDG->RLR = 0x0FFF; // 410-512ms timeout (depending on stm32 chip)
    IWDG->KR = 0xCCCC;
}
DECL_INIT(watchdog_init);
