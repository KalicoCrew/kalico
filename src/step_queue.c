// See step_queue.h for the design.
//
// Storage placement:
//   H7: DTCM-mapped .bss (non-cached, eliminates cache coherency between
//       TIM5 ISR producer and SysTick consumer). Q-LINKER open question:
//       confirm the existing H7 linker script's DTCM region name; if no
//       dedicated DTCM region exists, fall back to default .bss (which
//       may live in cached AXI SRAM and require explicit cache maintenance).
//   F4: default .bss (no DTCM/cache concern).

#include "autoconf.h"
#include "step_queue.h"

#if CONFIG_MACH_STM32H7
// TODO Q-LINKER: confirm section name. Default placement uses .bss
// in DTCM if the linker script maps it so. If a dedicated section is
// needed, add it via __attribute__((section(".dtcm_bss"))).
StepQueue step_queues[N_AXIS_STEP_QUEUES];
#else
StepQueue step_queues[N_AXIS_STEP_QUEUES];
#endif
