// Temporary: weak-symbol feasibility probe for the runtime-tick refactor.
// Spec §4.4 + plan Task 1.5. Removed in plan Task 2.
//
// This TU defines a weak no-op symbol. A second TU
// (runtime_tick_weak_probe_strong.c) provides a strong override when
// CONFIG_RUNTIME_WEAK_PROBE=y. The H7 ISR calls runtime_weak_probe()
// unconditionally; we verify post-link that the strong body wins.

#include <stdint.h>

__attribute__((weak)) void
runtime_weak_probe(uint32_t v)
{
    (void)v;  // weak no-op
}
