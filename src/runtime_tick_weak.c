// src/runtime_tick_weak.c
//
// Always-linked weak no-op fallbacks for optional runtime-tick ISR hooks.
// When the matching CONFIG_* enables a sibling TU that provides a strong
// override (runtime_bench.c, runtime_sim_commands.c), the linker selects
// that override; otherwise these no-ops resolve. Per-family ISRs call the
// hooks unconditionally — no #ifdef in any backend.
//
// Spec §4.4. Empirical link-time selection verified by Task 1.5.

#include <stdint.h>

__attribute__((weak)) void
runtime_bench_capture(uint32_t cycles_delta)
{
    (void)cycles_delta;
}

__attribute__((weak)) void
runtime_sim_isr_wake_drain(void)
{
}
