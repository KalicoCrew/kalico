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

#if CONFIG_KALICO_RUNTIME
// Spec §5.7 — kalico runtime liveness gate. Foreground (runtime_drain task)
// is the sole writer; this file only reads. __attribute__((used,
// externally_visible)) survives Klipper's -fwhole-program --gc-sections.
volatile uint8_t runtime_liveness_ok __attribute__((used, externally_visible))
    = 1;
#endif

void
watchdog_reset(void)
{
#if CONFIG_KALICO_SIM
    return;  // Renode's IWDG model misbehaves; sim build is silicon-unsafe
#endif
#if CONFIG_KALICO_RUNTIME
    if (!runtime_liveness_ok) return;   // kalico runtime detected liveness fault
#endif
    IWDG->KR = 0xAAAA;
}
DECL_TASK(watchdog_reset);

void
watchdog_init(void)
{
#if CONFIG_KALICO_SIM
    return;  // Don't arm IWDG in sim — see watchdog_reset
#endif
    // 2026-05-17 H7-wedge-after-tmcuart_send diagnostic: leave IWDG
    // disarmed entirely. Bench observation: H7 USB re-enumerates 30-50 s
    // after klippy starts, klippy loses its FD to ttyACMx, "klippy
    // pretends to be up" symptom. Hypothesis: foreground stalls > 512 ms
    // under TMC autotune load → IWDG fires → MCU resets → USB
    // disconnect/re-enumerate. If this diagnostic build doesn't wedge
    // (or wedges much later), IWDG starvation is the cause; the proper
    // fix is to find/eliminate the foreground stall, not to lengthen
    // the timeout. NEVER ship this to a real print without IWDG armed
    // again — silicon-unsafe.
    (void)0;
}
DECL_INIT(watchdog_init);
