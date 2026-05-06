// Temporary: strong override for runtime_weak_probe.
// Plan Task 1.5; removed in Task 2.

#include <stdint.h>

// Volatile sink so LTO cannot eliminate the side effect that proves the
// strong body executed.
volatile uint32_t runtime_weak_probe_sink = 0;

void
runtime_weak_probe(uint32_t v)
{
    runtime_weak_probe_sink = v;
}
