/*
 * C smoke build for kalico-c-api. Spec §6.3.
 *
 * Two jobs:
 *   1. _Static_assert every ABI-relevant TraceSample field — drift detection
 *      that Rust-side size_of/offset_of tests cannot cover (catches cbindgen
 *      output going stale relative to the Rust struct).
 *   2. Link against libkalico_c_api.a — verifies every kalico_runtime_*
 *      symbol the header declares actually resolves.
 *
 * Host-side stubs for kalico_clock_freq + kalico_h7_* symbols are provided
 * here because the staticlib leaves them undefined (on MCU they come from
 * src/runtime_tick.c and the H7 timer driver; on host the unit tests in
 * tests/init_once.rs supply them via #[no_mangle], but a pure-C link cannot
 * pull those Rust symbols in, so we stub natively here).
 */

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include "kalico_runtime.h"

/* Spec §6.3 + §13.2 + §10.1: every ABI-relevant field covered.
 * Step-6 layout (Phase 2 + Phase 5): tick(0..8) motor_a(8..12) motor_b(12..16)
 * motor_e(16..20) segment_id(20..24) curve_handle(24..28) flags(28) pad(29..32).
 */
_Static_assert(sizeof(TraceSample) == 32, "TraceSample size mismatch");
_Static_assert(_Alignof(TraceSample) == 8, "TraceSample alignment mismatch");
_Static_assert(offsetof(TraceSample, tick) == 0, "tick offset");
_Static_assert(offsetof(TraceSample, motor_a) == 8, "motor_a offset");
_Static_assert(offsetof(TraceSample, motor_b) == 12, "motor_b offset");
_Static_assert(offsetof(TraceSample, motor_e) == 16, "motor_e offset");
_Static_assert(offsetof(TraceSample, segment_id) == 20, "segment_id offset");
_Static_assert(offsetof(TraceSample, curve_handle) == 24, "curve_handle offset");
_Static_assert(offsetof(TraceSample, flags) == 28, "flags offset");
_Static_assert(offsetof(TraceSample, _pad) == 29, "_pad offset");
_Static_assert(sizeof(((TraceSample *)0)->_pad) == 3, "_pad length");

/* Host-side stubs for symbols the staticlib leaves undefined. */
const uint32_t kalico_clock_freq = 520000000u;

void kalico_h7_enable_tim5(void) {}
void kalico_h7_disable_tim5(void) {}
uint32_t kalico_h7_read_cyccnt(void) { return 0u; }

/* Step-6 Phase 7 §8.5 force_idle handshake symbols. */
uint64_t kalico_host_now_us(void) { return 0ULL; }
uint32_t irq_save(void) { return 0u; }
void irq_restore(uint32_t flags) { (void)flags; }

int main(void) {
    /* Trivial smoke — link symbol resolution check. We don't assert on the
     * returned handle's value; init may legitimately succeed or (on a second
     * invocation in some test ordering) return null. The point is the symbol
     * resolves and the program runs without crashing. */
    KalicoRuntime *h = kalico_runtime_init();
    (void)h;
    return 0;
}
