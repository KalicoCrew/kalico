// See step_queue.h for the design.
//
// Storage placement:
//   H7: default `.bss` already lives in DTCM. The shared
//       linker script `src/generic/armcm_link.lds.S` sets the `ram`
//       region to `CONFIG_RAM_START`/`CONFIG_RAM_SIZE`, and on the
//       H7 those are `0x20000000` / `0x20000` (128 KB) — i.e. DTCM,
//       not the AXI SRAM region (which is opted into via the
//       separate `.axi_bss` section). DTCM is non-cached and reached
//       in a single CPU cycle, so no cache-maintenance / explicit
//       attribute is required to give the TIM5-ISR producer and the
//       SysTick consumer a coherent view of `step_queues`.
//   F4: default `.bss` (single SRAM bank, no DTCM/cache split).
//
// Q-LINKER (resolved 2026-05-19, Task 18): default `.bss` is correct
// on H7. Do NOT migrate to `.axi_bss` — that would *introduce* the
// cache-coherency overhead this placement avoids.

#include "step_queue.h"

StepQueue step_queues[N_AXIS_STEP_QUEUES];
