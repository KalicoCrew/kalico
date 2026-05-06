// src/generic/runtime_bench.h
//
// Cycle-count benchmarking for the runtime tick. Storage and command logic
// live in src/generic/runtime_bench.c (selected by CONFIG_RUNTIME_BENCH).
// The per-family ISR calls runtime_bench_capture(cycles_delta) on every
// tick; without CONFIG_RUNTIME_BENCH, the weak no-op in
// src/runtime_tick_weak.c resolves and the call is effectively free.
//
// SWSR invariant: a single ISR is the only writer; foreground reads only
// after observing `runtime_bench_count == runtime_bench_target`. Adding a
// second writer or a polling reader breaks the invariant — touch with care.

#ifndef RUNTIME_BENCH_H
#define RUNTIME_BENCH_H

#include <stdint.h>

#define RUNTIME_BENCH_MAX_SAMPLES 256

extern volatile uint32_t runtime_bench_samples_buf[RUNTIME_BENCH_MAX_SAMPLES];
extern volatile uint16_t runtime_bench_count;
extern volatile uint16_t runtime_bench_target;
extern volatile uint8_t  runtime_bench_isolate;

// Per-family ISR call site. Strong def in runtime_bench.c when
// CONFIG_RUNTIME_BENCH=y; weak no-op in runtime_tick_weak.c otherwise.
void runtime_bench_capture(uint32_t cycles_delta);

#endif // RUNTIME_BENCH_H
