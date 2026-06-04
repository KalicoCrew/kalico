// See step_queue.h for the design.
//
// Default .bss is correct here: on H7 it lands in DTCM (non-cached, coherent),
// so the TIM5-ISR producer and SysTick consumer share step_queues with no
// cache maintenance. Do NOT move to .axi_bss — that reintroduces cache cleans.

#include "step_queue.h"

// used,externally_visible: only the Rust staticlib references this
// (extern "C" { static step_queues; }), so -fwhole-program LTO would strip it.
__attribute__((used, externally_visible))
StepQueue step_queues[N_AXIS_STEP_QUEUES];
