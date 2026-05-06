// src/stm32/kalico_h7_timer.h
//
// Shared declarations for the kalico H7 TIM5 ISR + bench buffer. Included by
// both src/stm32/kalico_h7_timer.c (defines the storage) and src/runtime_tick.c
// (drives the bench command).

#ifndef KALICO_H7_TIMER_H
#define KALICO_H7_TIMER_H

#include <stdint.h>

// Halved from 1024 to 256 to free 3 KB of bss for the kalico-native dispatch
// path on H7 (transmit_buf bump + dispatch tx_buf). Bench still produces
// statistically useful samples; bump back if Surface-C cycle-actuals work
// needs longer runs.
#define KALICO_BENCH_MAX_SAMPLES 256

extern volatile uint32_t kalico_bench_samples_buf[KALICO_BENCH_MAX_SAMPLES];
extern volatile uint16_t kalico_bench_count;
extern volatile uint16_t kalico_bench_target;
extern volatile uint8_t  kalico_bench_isolate;

void kalico_h7_timer_init(void);
void kalico_h7_enable_tim5(void);
void kalico_h7_disable_tim5(void);
uint32_t kalico_h7_read_cyccnt(void);

#endif // KALICO_H7_TIMER_H
