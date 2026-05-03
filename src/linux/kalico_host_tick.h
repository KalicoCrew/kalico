// Host-process modulation tick driver. Linux equivalent of the H7
// TIM5 ISR (src/stm32/kalico_h7_timer.c). Provides the same symbol
// surface so src/runtime_tick.c and the Rust producer-protocol can
// link unchanged across MACH_STM32H7 / MACH_LINUX.

#ifndef KALICO_HOST_TICK_H
#define KALICO_HOST_TICK_H

#include <stdint.h>

#define KALICO_BENCH_MAX_SAMPLES 1024

extern volatile uint32_t kalico_bench_samples_buf[KALICO_BENCH_MAX_SAMPLES];
extern volatile uint16_t kalico_bench_count;
extern volatile uint16_t kalico_bench_target;
extern volatile uint8_t  kalico_bench_isolate;

// Same names as the H7 driver — the Rust producer-protocol calls these
// across an extern "C" boundary unconditionally.
void kalico_h7_timer_init(void);
void kalico_h7_enable_tim5(void);
void kalico_h7_disable_tim5(void);
uint32_t kalico_h7_read_cyccnt(void);

#endif
