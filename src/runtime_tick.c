// src/runtime_tick.c
//
// Klipper-side portable glue for kalico runtime. Spec §2.4 / §4.5 / §5.7.

#include <string.h>         // memcpy
#include "autoconf.h"
#include "board/internal.h" // NVIC_*, IWDG, OTG_HS_IRQn, USART2_IRQn
#include "board/misc.h"     // timer_read_time
#include "command.h"        // DECL_COMMAND
#include "sched.h"          // DECL_INIT, DECL_TASK
#include "kalico_runtime.h"

#if CONFIG_KALICO_RUNTIME

// H7 CMSIS only defines IWDG1/IWDG2; map the generic name to IWDG1
// (matching src/stm32/watchdog.c's pattern) so the bench-loop kick
// compiles cleanly.
#if CONFIG_MACH_STM32H7
#define IWDG IWDG1
#endif

// Exposed to Rust via `extern "C" { static kalico_clock_freq: u32; }`.
// __attribute__((used, externally_visible)) survives -fwhole-program LTO + GC.
const uint32_t kalico_clock_freq __attribute__((used, externally_visible))
    = CONFIG_CLOCK_FREQ;

extern volatile uint8_t kalico_liveness_ok;  // defined in src/stm32/watchdog.c

void* kalico_rt_handle = 0;            // exposed (non-static) for kalico_h7_timer.c
static struct task_wake runtime_drain_wake;
static struct timer runtime_drain_timer;

// Liveness monitor state.
static uint32_t last_seen_tick_counter = 0;
static uint32_t last_progress_time = 0;

// First-light status LED removed for Surface C bring-up.
//
// The plan-literal placeholder pin was PA13, which is SWDIO on the H7 —
// reconfiguring it as a GPIO output kills the SWD debugger and is
// generally hostile to debugability during early bring-up. The PASS/FAIL
// gate of test_h723_first_light.py is the `kalico_status` response over
// USB-CDC, not a visual LED, so the visual signal is dispensable.
//
// Future work: pick a non-SWD pin from the actual Octopus Pro silkscreen
// (e.g. one of the unused fan headers or a dedicated debug LED) and
// reintroduce the toggle. Tracked as a Step-5 follow-up in plan-changes-log.
static uint8_t last_seen_status = 255;

// Periodic timer callback at ~1 kHz: sets the drain wake flag.
// Per spec §4.5 — sched_check_wake throttle prevents spinning the drain
// task at full FG iteration rate when the trace ring is empty.
static uint_fast8_t
runtime_drain_event(struct timer *t)
{
    sched_wake_task(&runtime_drain_wake);
    t->waketime += timer_from_us(1000);  // 1 kHz
    return SF_RESCHEDULE;
}

#if CONFIG_KALICO_SIM
// Sim-only direct wake from the TIM5 ISR. Under Renode, the DWT-based
// timer system is best-effort even with the kalico_sim_cyccnt fork
// (timer_set_diff -> SysTick->LOAD interactions are subtle with a
// stepping software counter), so the runtime_drain_timer's 1 kHz cadence
// can be unreliable. Step-6 plan Phase 0 Gate A trace-stream verification
// requires drain to be invoked deterministically while segments are
// retiring; this provides a guaranteed wake path keyed off TIM5 fires.
//
// Throttle: wake every KALICO_SIM_DRAIN_PERIOD_TICKS = 40 fires (= once
// per 1 ms at 40 kHz tick rate). sched_wake_task is ISR-safe (sets a
// volatile flag + atomic write).
extern void sched_wake_task(struct task_wake *w);
volatile uint32_t kalico_sim_drain_counter = 0;
#define KALICO_SIM_DRAIN_PERIOD_TICKS 40

__attribute__((used, externally_visible))
void
kalico_sim_isr_wake_drain(void)
{
    if (++kalico_sim_drain_counter >= KALICO_SIM_DRAIN_PERIOD_TICKS) {
        kalico_sim_drain_counter = 0;
        sched_wake_task(&runtime_drain_wake);
    }
}
#endif

void
runtime_init(void)
{
    kalico_rt_handle = kalico_runtime_init();
    if (!kalico_rt_handle) {
        // Init failed — leave liveness flag at default (1 = OK) but handle unset;
        // calls into the runtime will short-circuit safely.
        return;
    }
    last_seen_tick_counter = kalico_runtime_tick_counter(kalico_rt_handle);
    last_progress_time = timer_read_time();
    last_seen_status = kalico_runtime_status(kalico_rt_handle);

    // Initialize H7 timer hardware (TIM5) — DOES NOT enable yet; first segment
    // push triggers enable via the producer protocol (§4.4).
    extern void kalico_h7_timer_init(void);
    kalico_h7_timer_init();

    // Wire the periodic 1 kHz drain wake.
    runtime_drain_timer.func = runtime_drain_event;
    runtime_drain_timer.waketime = timer_read_time() + timer_from_us(1000);
    sched_add_timer(&runtime_drain_timer);
}
DECL_INIT(runtime_init);

#define KALICO_TRACE_BATCH 64
#define KALICO_LIVENESS_THRESHOLD_MS 25
#define KALICO_LIVENESS_THRESHOLD_TICKS  \
    ((KALICO_LIVENESS_THRESHOLD_MS) * (CONFIG_CLOCK_FREQ / 1000))

#if CONFIG_KALICO_SIM
volatile uint32_t kalico_sim_drain_calls = 0;
#endif

void
runtime_drain(void)
{
    if (!kalico_rt_handle) return;
    if (!sched_check_wake(&runtime_drain_wake)) return;

#if CONFIG_KALICO_SIM
    kalico_sim_drain_calls++;
#endif

    // Drain a batch.
    static uint8_t batch_buf[KALICO_TRACE_BATCH * 32];  // 32 bytes per sample
    uint32_t n = kalico_runtime_drain_trace(
        kalico_rt_handle, (struct TraceSample*)batch_buf, KALICO_TRACE_BATCH);
    if (n > 0) {
        sendf("kalico_trace count=%u data=%*s", n, n * 32, batch_buf);
    }

    // Liveness check. Only meaningful when the runtime is RUNNING — the ISR
    // is deliberately disabled in IDLE/DRAINED (no segment pushed yet) and
    // tick_counter cannot advance, so we'd trip a false positive within
    // KALICO_LIVENESS_THRESHOLD_MS of boot otherwise. We refresh the
    // last_progress_time anchor in non-RUNNING states so a state transition
    // INTO RUNNING doesn't immediately trip on a stale anchor.
    uint32_t cur_counter = kalico_runtime_tick_counter(kalico_rt_handle);
    uint32_t cur_time = timer_read_time();
    uint8_t cur_status = kalico_runtime_status(kalico_rt_handle);
    if (cur_status == 1 /* RUNNING */) {
        if (cur_counter != last_seen_tick_counter) {
            last_seen_tick_counter = cur_counter;
            last_progress_time = cur_time;
        } else if ((cur_time - last_progress_time) > KALICO_LIVENESS_THRESHOLD_TICKS) {
            // ISR has stalled while RUNNING. Stop kicking the watchdog.
            kalico_liveness_ok = 0;
        }
    } else {
        last_progress_time = cur_time;
        last_seen_tick_counter = cur_counter;
    }

    // FAULT → also block kicks.
    if (cur_status == 3 /* FAULT */) {
        kalico_liveness_ok = 0;
    }

    // Track last status (used by future LED hook on a non-SWD pin).
    if (cur_status != last_seen_status) {
        last_seen_status = cur_status;
    }
}
DECL_TASK(runtime_drain);

// DECL_COMMAND surface — test harness loads curves and pushes segments.
//
// Klipper's %*s blob format consumes TWO args slots per blob: a length
// followed by an encoded pointer that must be reconstituted via
// `command_decode_ptr` (declared in command.h). See src/i2ccmds.c and
// src/spicmds.c for canonical usage. Each f32 control point is 3 lanes ×
// 4 bytes = 12 bytes; each knot/weight is a single f32 (4 bytes). We
// derive `n_cp`, `n_knots`, `n_weights` from the blob byte-lengths and
// validate self-consistency before calling into Rust.
// Aligned scratch buffers for the load_curve handler. Klipper's RX buffer
// places the %*s payload at an arbitrary byte offset (typically not 4-byte
// aligned), so passing those pointers directly to Rust yields an unaligned
// `&[f32]` — UB on construction even though Cortex-M7 happens to allow
// unaligned word reads at the CPU level. Empirically this hardfaults the
// MCU and triggers a USB renumerate. Copy into 4-byte-aligned static
// buffers first, then pass to Rust.
//
// Sizing matches CurvePool's compile-time bounds (8 control points, 12
// knot vector entries, 8 weights). Static rather than stack: the load
// handler runs in command-dispatch foreground context and stack is only
// 512 B; ~144 B of locals would be tight.
static float kalico_aligned_cps[8 * 3];
static float kalico_aligned_knots[12];
static float kalico_aligned_weights[8];

void
command_kalico_load_curve(uint32_t *args)
{
    if (!kalico_rt_handle) {
        sendf("kalico_load_curve_response result=%i", -7);
        return;
    }
    uint16_t slot         = args[0];
    uint8_t  degree       = args[1];
    uint16_t cps_len      = args[2];
    const uint8_t *cps_b  = command_decode_ptr(args[3]);
    uint16_t knots_len    = args[4];
    const uint8_t *knots_b = command_decode_ptr(args[5]);
    uint16_t weights_len  = args[6];
    const uint8_t *weights_b = command_decode_ptr(args[7]);

    // Producer-side validation: cps must be a multiple of 12 (xyz × f32);
    // knots and weights must be a multiple of 4 (f32); weights count must
    // equal cp count. Mismatch → KALICO_ERR_INVALID_CURVE (-2).
    if ((cps_len % 12) || (knots_len % 4) || (weights_len % 4)) {
        sendf("kalico_load_curve_response result=%i", -2);
        return;
    }
    uint16_t n_cp      = cps_len / 12;
    uint16_t n_knots   = knots_len / 4;
    uint16_t n_weights = weights_len / 4;
    if (n_weights != n_cp) {
        sendf("kalico_load_curve_response result=%i", -2);
        return;
    }
    if (cps_len > sizeof(kalico_aligned_cps) ||
        knots_len > sizeof(kalico_aligned_knots) ||
        weights_len > sizeof(kalico_aligned_weights)) {
        sendf("kalico_load_curve_response result=%i", -2);
        return;
    }

    // Byte-copy into the aligned scratch buffers. memcpy on Cortex-M7 with
    // -O2 lowers to a tight LDR/STR loop; the source unalignment is fine
    // because we copy bytes, not words.
    memcpy(kalico_aligned_cps, cps_b, cps_len);
    memcpy(kalico_aligned_knots, knots_b, knots_len);
    memcpy(kalico_aligned_weights, weights_b, weights_len);

    int32_t r = kalico_runtime_load_curve(
        kalico_rt_handle, slot,
        kalico_aligned_cps, n_cp,
        kalico_aligned_knots, n_knots,
        kalico_aligned_weights, n_weights,
        degree);
    sendf("kalico_load_curve_response result=%i", r);
}
DECL_COMMAND(command_kalico_load_curve,
    "kalico_load_curve slot=%hu degree=%c "
    "cps=%*s knots=%*s weights=%*s");

void
command_kalico_push_segment(uint32_t *args)
{
    if (!kalico_rt_handle) { sendf("kalico_push_response result=%i", -7); return; }
    uint32_t id = args[0];
    uint16_t curve = args[1];
    uint64_t t_start = ((uint64_t)args[2] << 32) | args[3];
    uint64_t t_end   = ((uint64_t)args[4] << 32) | args[5];
    uint8_t kin = args[6];
    int32_t r = kalico_runtime_push_segment(
        kalico_rt_handle, id, curve, t_start, t_end, kin);
    sendf("kalico_push_response result=%i", r);
}
DECL_COMMAND(command_kalico_push_segment,
    "kalico_push_segment id=%u curve=%hu t_start_hi=%u t_start_lo=%u "
    "t_end_hi=%u t_end_lo=%u kinematics=%c");

void
command_kalico_query_status(uint32_t *args)
{
    if (!kalico_rt_handle) {
        sendf("kalico_status status=%c last_err=%i", (uint8_t)255, -7);
        return;
    }
    uint8_t status = kalico_runtime_status(kalico_rt_handle);
    int32_t last_err = kalico_runtime_last_error(kalico_rt_handle);
    sendf("kalico_status status=%c last_err=%i", status, last_err);
}
DECL_COMMAND(command_kalico_query_status, "kalico_query_status");

#if CONFIG_KALICO_SIM
extern volatile uint32_t kalico_sim_drain_calls;
extern volatile uint32_t kalico_sim_cyccnt;
extern volatile uint32_t kalico_sim_drain_counter;

void
command_kalico_sim_diag(uint32_t *args)
{
    uint8_t status = kalico_rt_handle ? kalico_runtime_status(kalico_rt_handle) : 255;
    int32_t last_err = kalico_rt_handle ? kalico_runtime_last_error(kalico_rt_handle) : 0;
    uint32_t tick_counter = kalico_rt_handle ? kalico_runtime_tick_counter(kalico_rt_handle) : 0;
    sendf(
        "kalico_sim_diag_response drain_calls=%u cyccnt=%u drain_counter=%u "
        "status=%c last_err=%i tick_counter=%u",
        kalico_sim_drain_calls, kalico_sim_cyccnt, kalico_sim_drain_counter,
        status, last_err, tick_counter);
}
DECL_COMMAND(command_kalico_sim_diag, "kalico_sim_diag");
#endif

#if CONFIG_KALICO_SIM
// Sim-only escape hatch (Step-6 plan Phase 0 Task 0.2). Diagnoses the
// load_curve hang in Renode (the H7 .repl ignores SCB->CPACR writes from
// SystemInit, so any FPU instruction in CurvePool::load — including
// is_finite() and > 0.0 checks — UsageFaults). The fixture path uses static
// pre-validated curve data and CurvePool::load_unchecked (integer-only
// memcpy), bypassing the FPU entirely. NEVER include in production.
extern int32_t kalico_runtime_load_fixture(
    void *rt, uint16_t slot, uint16_t fixture_id);

void
command_kalico_load_fixture_curve(uint32_t *args)
{
    if (!kalico_rt_handle) {
        sendf("kalico_load_fixture_response result=%i", -7);
        return;
    }
    uint16_t slot = args[0];
    uint16_t fixture_id = args[1];
    int32_t r = kalico_runtime_load_fixture(kalico_rt_handle, slot, fixture_id);
    sendf("kalico_load_fixture_response result=%i", r);
}
DECL_COMMAND(command_kalico_load_fixture_curve,
    "kalico_load_fixture_curve slot=%hu fixture_id=%hu");
#endif // CONFIG_KALICO_SIM

// ---- Cycle-count bench (Task 27 / spec §6.4) ---------------------------
//
// Surface-C only. Captures DWT->CYCCNT around `kalico_runtime_tick` over N
// samples and replies with one `kalico_bench_sample value=<cycles>` response
// per measurement (after the warmup skip) and a final `kalico_bench_done
// count=<N> error=0` per the host-side test_h723_cycle_count.py protocol.
// Wire format is Klipper's standard binary VLQ (sendf); host-side parses
// via klippy/msgproto.py wrapped by tools/kalico_host_io.py.
//
// `isolate=1` selectively masks USB+USART IRQs during the measurement window
// (TIM5 stays enabled). `isolate=0` runs with full IRQs (production load).
// SysTick is left untouched — Klipper's foreground time accounting needs it,
// and the kalico TIM5 ISR doesn't preempt SysTick at priority 3 anyway.

// KALICO_BENCH_MAX_SAMPLES is declared in `src/stm32/kalico_h7_timer.h`
// (Task 23 creates it) so both `runtime_tick.c` and `kalico_h7_timer.c`
// see the same value.
#include "stm32/kalico_h7_timer.h"
extern volatile uint32_t kalico_bench_samples_buf[KALICO_BENCH_MAX_SAMPLES];
extern volatile uint16_t kalico_bench_count;
extern volatile uint16_t kalico_bench_target;
extern volatile uint8_t kalico_bench_isolate;

// Bench error codes — all sites use the canonical sendf format
// `kalico_bench_done count=%hu error=%i` per Klipper's one-format-per-message
// rule (compile_time_request rejects format conflicts).
#define KALICO_BENCH_OK             0
#define KALICO_BENCH_ERR_NOT_INIT  -7
#define KALICO_BENCH_ERR_BELOW_WARMUP -4
#define KALICO_BENCH_ERR_LIVENESS  -100
#define KALICO_BENCH_ERR_ISR_TIMEOUT -101

void
command_kalico_bench_run(uint32_t *args)
{
    if (!kalico_rt_handle) {
        sendf("kalico_bench_done count=%hu error=%i", 0, KALICO_BENCH_ERR_NOT_INIT);
        return;
    }

    // Liveness pre-check (round-4 review): if the runtime had already
    // tripped a liveness fault before we got here, manually kicking IWDG
    // inside the bench loop would mask it. Refuse to bench in that case.
    if (!kalico_liveness_ok) {
        sendf("kalico_bench_done count=%hu error=%i", 0, KALICO_BENCH_ERR_LIVENESS);
        return;
    }

    uint8_t isolate = args[0];
    uint16_t samples = args[1];
    if (samples > KALICO_BENCH_MAX_SAMPLES) samples = KALICO_BENCH_MAX_SAMPLES;

    if (isolate) {
        // Selectively mask: USB OTG_HS (Octopus Pro's H723 has only the OTG_HS
        // controller; Klipper aliases it as OTG_IRQn elsewhere) + USART2 (active
        // console). Leave TIM5 (the kalico ISR) and SysTick alone. The implementer
        // MUST verify which IRQs are active in the current build before relying on
        // the masked list — picking the wrong IRQ silently biases Pass A toward
        // overly-optimistic numbers.
        // Cross-check with `arm-none-eabi-objdump -d klipper.elf | grep -E 'IRQ|Handler'`
        // to confirm the IRQ vector names actually present in the firmware image.
        NVIC_DisableIRQ(OTG_HS_IRQn);
        NVIC_DisableIRQ(USART2_IRQn);
    }

    kalico_bench_count = 0;
    kalico_bench_target = samples;
    kalico_bench_isolate = isolate;

    // Wait for the ISR to fill the buffer with a watchdog-respecting timeout.
    // Worst case: 25 µs/sample × 1024 = 25.6 ms. We allow 100 ms before
    // bailing out, and we kick the IWDG ourselves during the wait so we
    // don't trip Klipper's watchdog from foreground starvation. Note: the
    // liveness-heartbeat counter does freeze for the duration of this wait,
    // but that's bounded and known — it's only used during Surface-C bring-up.
    uint32_t start = timer_read_time();
    uint32_t timeout_ticks = timer_from_us(100000);  // 100 ms
    while (kalico_bench_count < kalico_bench_target) {
        // Manually kick the IWDG (foreground watchdog_reset would otherwise
        // get pre-empted by our spin and starve). Spec §5.7 — `kalico_liveness_ok`
        // is set true here because we KNOW the runtime is healthy; the gate
        // is only meaningful for unattended operation.
        IWDG->KR = 0xAAAA;
        if ((uint32_t)(timer_read_time() - start) > timeout_ticks) {
            // ISR didn't fill the buffer — TIM5 stalled or NVIC mask wrong.
            kalico_bench_target = 0;  // tell ISR to stop bracketing
            sendf("kalico_bench_done count=%hu error=%i",
                  kalico_bench_count, KALICO_BENCH_ERR_ISR_TIMEOUT);
            if (isolate) {
                NVIC_EnableIRQ(OTG_HS_IRQn);
                NVIC_EnableIRQ(USART2_IRQn);
            }
            return;
        }
    }

    if (isolate) {
        NVIC_EnableIRQ(OTG_HS_IRQn);
        NVIC_EnableIRQ(USART2_IRQn);
    }

    // Discard the first 8 samples (warm-up: cache fill, branch predictor,
    // FPU lazy-stacking on first vector_eval). Spec §6.4 hardened methodology.
    // Underflow guard: refuse if caller didn't request enough samples.
    const uint16_t WARMUP_SKIP = 8;
    if (samples <= WARMUP_SKIP) {
        sendf("kalico_bench_done count=%hu error=%i", 0,
              KALICO_BENCH_ERR_BELOW_WARMUP);
        return;
    }

    // Emit one Klipper-framed `kalico_bench_sample value=N` response per
    // measurement (after warmup). sendf encodes via Klipper's standard
    // VLQ framing (msgproto.py); host parses with klippy/console.py-style
    // MessageParser via tools/kalico_host_io.py. Bounded total: at most
    // KALICO_BENCH_MAX_SAMPLES (1024) responses per bench command.
    for (uint16_t i = WARMUP_SKIP; i < samples; i++) {
        sendf("kalico_bench_sample value=%u", kalico_bench_samples_buf[i]);
    }
    sendf("kalico_bench_done count=%hu error=%i",
          (uint16_t)(samples - WARMUP_SKIP), KALICO_BENCH_OK);
}
DECL_COMMAND(command_kalico_bench_run, "kalico_bench_run isolate=%c samples=%hu");

#endif // CONFIG_KALICO_RUNTIME
