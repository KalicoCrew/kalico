#ifndef RUNTIME_TICK_H
#define RUNTIME_TICK_H

#include <stdint.h>

void runtime_tick_init(void);
void runtime_tick_enable(void);
void runtime_tick_disable(void);
uint32_t runtime_cyccnt_read(void);

// Mirrors rust/runtime/src/per_axis_timer.rs::STEP_OUTPUT_DISABLE.
#define KALICO_STEP_OUTPUT_DISABLE 0xFFFFFFFFu

#endif
