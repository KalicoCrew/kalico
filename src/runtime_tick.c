// src/runtime_tick.c
//
// Klipper-side portable glue for kalico runtime. Spec §2.4 / §4.5 / §5.7.

#include "autoconf.h"
#include "board/misc.h"  // timer_read_time
#include "command.h"     // DECL_COMMAND
#include "sched.h"       // DECL_INIT, DECL_TASK
#include "kalico_runtime.h"

#if CONFIG_KALICO_RUNTIME

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

void
runtime_drain(void)
{
    if (!kalico_rt_handle) return;
    if (!sched_check_wake(&runtime_drain_wake)) return;

    // Drain a batch.
    static uint8_t batch_buf[KALICO_TRACE_BATCH * 32];  // 32 bytes per sample
    uint32_t n = kalico_runtime_drain_trace(
        kalico_rt_handle, (struct TraceSample*)batch_buf, KALICO_TRACE_BATCH);
    if (n > 0) {
        sendf("kalico_trace count=%u data=%*s", n, n * 32, batch_buf);
    }

    // Liveness check.
    uint32_t cur_counter = kalico_runtime_tick_counter(kalico_rt_handle);
    uint32_t cur_time = timer_read_time();
    if (cur_counter != last_seen_tick_counter) {
        last_seen_tick_counter = cur_counter;
        last_progress_time = cur_time;
    } else if ((cur_time - last_progress_time) > KALICO_LIVENESS_THRESHOLD_TICKS) {
        // ISR has stalled. Stop kicking the watchdog.
        kalico_liveness_ok = 0;
    }

    // Or fault → also block kicks.
    if (kalico_runtime_status(kalico_rt_handle) == 3 /* FAULT */) {
        kalico_liveness_ok = 0;
    }
}
DECL_TASK(runtime_drain);

// DECL_COMMAND surface — test harness loads curves and pushes segments.
void
command_kalico_load_curve(uint32_t *args)
{
    if (!kalico_rt_handle) {
        sendf("kalico_load_curve_response result=%d", -7);
        return;
    }
    uint16_t slot = args[0];
    uint8_t degree = args[1];
    uint16_t n_cp = args[2];
    uint16_t n_knots = args[3];
    const float *cps = (const float*)args[4];
    const float *knots = (const float*)args[5];
    const float *weights = (const float*)args[6];
    int32_t r = kalico_runtime_load_curve(
        kalico_rt_handle, slot, cps, n_cp, knots, n_knots, weights, n_cp, degree);
    sendf("kalico_load_curve_response result=%i", r);
}
DECL_COMMAND(command_kalico_load_curve,
    "kalico_load_curve slot=%hu degree=%c n_cp=%hu n_knots=%hu "
    "cps=%*s knots=%*s weights=%*s");

void
command_kalico_push_segment(uint32_t *args)
{
    if (!kalico_rt_handle) { sendf("kalico_push_response result=%d", -7); return; }
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
    if (!kalico_rt_handle) { sendf("kalico_status status=255 last_err=-7"); return; }
    uint8_t status = kalico_runtime_status(kalico_rt_handle);
    int32_t last_err = kalico_runtime_last_error(kalico_rt_handle);
    sendf("kalico_status status=%c last_err=%i", status, last_err);
}
DECL_COMMAND(command_kalico_query_status, "kalico_query_status");

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

void
command_kalico_bench_run(uint32_t *args)
{
    if (!kalico_rt_handle) { sendf("kalico_bench_done error=-7"); return; }

    // Liveness pre-check (round-4 review): if the runtime had already
    // tripped a liveness fault before we got here, manually kicking IWDG
    // inside the bench loop would mask it. Refuse to bench in that case.
    if (!kalico_liveness_ok) {
        sendf("kalico_bench_done error=-99 reason=liveness_already_tripped");
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
            sendf("kalico_bench_done error=-99 reason=isr_timeout count=%hu",
                  kalico_bench_count);
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
        sendf("kalico_bench_done error=-4 reason=samples_below_warmup");
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
    sendf("kalico_bench_done count=%hu error=0",
          (uint16_t)(samples - WARMUP_SKIP));
}
DECL_COMMAND(command_kalico_bench_run, "kalico_bench_run isolate=%c samples=%hu");

#endif // CONFIG_KALICO_RUNTIME
