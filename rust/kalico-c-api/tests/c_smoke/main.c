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
 * Host-side stubs for runtime_clock_freq + kalico_h7_* symbols are provided
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
 * Step 7-B layout: tick(0..8) motor_a(8..12) motor_b(12..16) motor_z(16..20)
 * motor_e(20..24) segment_id(24..28) curve_handle(28..32) flags(32) pad(33..40).
 */
_Static_assert(sizeof(TraceSample) == 40, "TraceSample size mismatch");
_Static_assert(_Alignof(TraceSample) == 8, "TraceSample alignment mismatch");
_Static_assert(offsetof(TraceSample, tick) == 0, "tick offset");
_Static_assert(offsetof(TraceSample, motor_a) == 8, "motor_a offset");
_Static_assert(offsetof(TraceSample, motor_b) == 12, "motor_b offset");
_Static_assert(offsetof(TraceSample, motor_z) == 16, "motor_z offset");
_Static_assert(offsetof(TraceSample, motor_e) == 20, "motor_e offset");
_Static_assert(offsetof(TraceSample, segment_id) == 24, "segment_id offset");
_Static_assert(offsetof(TraceSample, curve_handle) == 28, "curve_handle offset");
_Static_assert(offsetof(TraceSample, flags) == 32, "flags offset");
_Static_assert(offsetof(TraceSample, _pad) == 33, "_pad offset");
_Static_assert(sizeof(((TraceSample *)0)->_pad) == 7, "_pad length");

/* Host-side stubs for symbols the staticlib leaves undefined. */
const uint32_t runtime_clock_freq = 520000000u;

void runtime_tick_enable(void) {}
void runtime_tick_disable(void) {}
uint32_t runtime_cyccnt_read(void) { return 0u; }

/* Step-6 Phase 7 §8.5 force_idle handshake symbols. */
uint64_t runtime_host_now_us(void) { return 0ULL; }
uint32_t runtime_irq_save(void) { return 0u; }
void runtime_irq_restore(uint32_t flags) { (void)flags; }

/* Klipper timer + stats interfaces used by the widened-now path and the
 * per-axis step timer (runtime/src/per_axis_timer.rs, kalico-c-api widened
 * clock). On MCU these come from src/sched.c and src/basecmd.c; here we
 * stub them so the host link resolves. */
uint32_t timer_read_time(void) { return 0u; }
uint8_t timer_is_before(uint32_t a, uint32_t b) { (void)a; (void)b; return 0u; }
void runtime_emit_step_pulses(uint8_t axis_idx, int32_t n_steps) {
    (void)axis_idx; (void)n_steps;
}
uint32_t stats_send_time = 0u;
uint32_t stats_send_time_high = 0u;

int main(void) {
    /* Trivial smoke — link symbol resolution check. We don't assert on the
     * returned handle's value; init may legitimately succeed or (on a second
     * invocation in some test ordering) return null. The point is the symbol
     * resolves and the program runs without crashing. */
    KalicoRuntime *h = runtime_handle_create();
    (void)h;
    return 0;
}
