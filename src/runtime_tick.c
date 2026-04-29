// src/runtime_tick.c
//
// Klipper-side portable glue for kalico runtime. Spec §2.4 / §4.5 / §5.7.

#include <string.h>         // memcpy
#include "autoconf.h"
#include "board/internal.h" // NVIC_*, IWDG, OTG_HS_IRQn, USART2_IRQn
#include "board/irq.h"      // irq_save, irq_restore (Step-6 §8.5 flush)
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

// Foreground host-clock helper for §8.5 flush ack-wait timeout. Returns
// wall-clock µs since boot, derived from Klipper's `timer_read_time()`
// (DWT->CYCCNT widened by Klipper's foreground task) divided by the clock
// frequency in MHz. NEVER call from ISR — `timer_read_time` is
// foreground-only by Klipper convention. Used only by the runtime's
// flush() ack-wait spin loop, which is foreground command dispatch.
//
// Wrap behaviour: `timer_read_time()` is u32, wraps every ~8.3 s at 520 MHz.
// The flush window is bounded to ≤1 ms by design, so a single wrap during
// the spin loop is the worst case — the saturating_add at the Rust caller
// site prevents UB from u64 overflow if we hit the boundary.
//
// Spec §8.5 + plan Phase 7 Task 7.2.
__attribute__((used, externally_visible))
uint64_t
kalico_host_now_us(void)
{
    uint32_t cycles = timer_read_time();
    // CONFIG_CLOCK_FREQ is in Hz; divide by 1e6 to get cycles-per-µs.
    // Integer division here is fine — CONFIG_CLOCK_FREQ is always a
    // multiple of 1 MHz on supported STM32 targets.
    return ((uint64_t)cycles) / (CONFIG_CLOCK_FREQ / 1000000U);
}

// kalico's stream::flush() FFI calls into Klipper's `irq_save` and
// `irq_restore` (board/irq.h) under the §8.5 disabled-IRQ window. The
// build uses `-flto=auto -fwhole-program`, which lets GCC consider
// non-`extern` definitions internal-only and inline/DCE them out — that
// drops the standalone `irq_save` / `irq_restore` symbols even though
// other Klipper TUs (sched.c, basecmd.c, …) call them, because LTO can
// inline the body at every callsite. The kalico_c_api.a archive then
// fails to resolve the symbols during the final link.
//
// Solution: provide thin wrappers `kalico_irq_save` / `kalico_irq_restore`
// that the staticlib calls instead of `irq_save` / `irq_restore` directly.
// The wrappers are marked `used, externally_visible` so LTO keeps them.
// They forward to the real functions, which LTO can still inline if it
// wants — but the staticlib only sees the wrapper symbols.
__attribute__((used, externally_visible))
uint32_t
kalico_irq_save(void)
{
    return (uint32_t)irq_save();
}

__attribute__((used, externally_visible))
void
kalico_irq_restore(uint32_t flags)
{
    irq_restore((irqstatus_t)flags);
}

void* kalico_rt_handle = 0;            // exposed (non-static) for kalico_h7_timer.c
static struct task_wake runtime_drain_wake;
static struct timer runtime_drain_timer;

// Phase 11 §5.3 periodic status frame state. Emit cadence is ~10 Hz against
// `timer_read_time()` (Klipper's u32 cycle clock). One-shot tracking of the
// last engine_status lets us emit a `kalico_fault` async event ONCE on
// the FAULT-state transition, not every 10 Hz tick — host gets one
// notification per fault, not a spam stream.
static uint32_t last_status_emit_time = 0;
static uint8_t prev_engine_status = 0;

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

    // Phase 11 §5.3: anchor the periodic-status emit timer so the first
    // status frame fires within one period of boot. The static `0` default
    // works fine in production where `timer_read_time()` quickly exceeds
    // the period; under CONFIG_KALICO_SIM with the software CYCCNT it can
    // take a noticeable fraction of a real-time second to advance one
    // period, but the gate self-corrects on the second iteration.
    last_status_emit_time = timer_read_time();
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

    // Phase 11 Task 11.2 §10.4 reclaim drain pipeline. Drains a batch of
    // trace samples for transport to the host, then asks the Rust side to
    // also drain-and-reclaim its own internal cursor for SEGMENT_END events
    // (so curve-pool slots get returned promptly) and check the §13.1
    // trace-overflow latch. The two drain paths share the same SPSC ring
    // (same FgState consumer); the order matters — `kalico_runtime_drain_trace`
    // moves samples to the wire FIRST so the host sees the trace data, THEN
    // `kalico_runtime_drain_and_reclaim` consumes any remaining samples for
    // bookkeeping. Both are safe back-to-back because each stops on the
    // first dequeue == None.
    static uint8_t batch_buf[KALICO_TRACE_BATCH * 32];  // 32 bytes per sample
    uint32_t n = kalico_runtime_drain_trace(
        kalico_rt_handle, (struct TraceSample*)batch_buf, KALICO_TRACE_BATCH);
    if (n > 0) {
        sendf("kalico_trace count=%u data=%*s", n, n * 32, batch_buf);
    }

    // Reclaim leg: drain whatever the wire-batch left behind and observe
    // SEGMENT_END events for curve-pool reclaim + trace-overflow check.
    // Returns a packed status word — see kalico_runtime_drain_and_reclaim
    // doc-comment for the bit layout.
    uint32_t reclaim_status = kalico_runtime_drain_and_reclaim(
        kalico_rt_handle, KALICO_TRACE_BATCH);
    uint8_t saw_segment_end = (reclaim_status >> 17) & 1;
    uint8_t fresh_overflow_fault = (reclaim_status >> 16) & 1;

    // §10.4: emit one `kalico_credit_freed` async event per drain cycle that
    // observed at least one SEGMENT_END. The host uses this to bump its
    // credit counter; it doesn't need one event per retired segment, just
    // a wake-up to re-read the cursors. `retired_through_segment_id` carries
    // the cumulative cursor; `free_slots = Q_N - queue_depth` (with Q_N - 1
    // being the structural cap; saturate at u8 in the Rust accessor).
    if (saw_segment_end) {
        uint32_t retired = kalico_runtime_retired_through_segment_id(kalico_rt_handle);
        uint8_t depth = kalico_runtime_queue_depth(kalico_rt_handle);
        uint8_t free_slots = (depth >= 7) ? 0 : (uint8_t)(7 - depth);
        // `output()` rather than `sendf()` so the format registers via
        // `_DECL_OUTPUT` (matching the explicit `DECL_CTR("_DECL_OUTPUT ...")`
        // at the bottom of this file). Mixing `sendf` (`_DECL_ENCODER`) and
        // explicit `_DECL_OUTPUT` for the same format crashes silently:
        // buildcommands.py de-dups by msgid + uses the FIRST registration's
        // category, so the second category's lookup returns NULL and the
        // emit `command_sendf(NULL, …)` no-ops.
        output("kalico_credit_freed retired_through_segment_id=%u free_slots=%c",
               retired, free_slots);
    }

    // §13.1: a fresh trace-overflow latch is reported via the `kalico_fault`
    // async event. The fault state itself is now in shared.last_error +
    // shared.runtime_status (latched by check_trace_overflow_and_fault on
    // the Rust side); the periodic `kalico_status_v6` frame echoes it on
    // the next 10 Hz tick. We send the async event here so the host gets
    // the fault notification immediately, not up to ~100 ms later.
    if (fresh_overflow_fault) {
        int32_t fault_code = kalico_runtime_last_error(kalico_rt_handle);
        uint32_t fault_detail = kalico_runtime_fault_detail(kalico_rt_handle);
        uint32_t cur_seg = kalico_runtime_current_segment_id(kalico_rt_handle);
        sendf("kalico_fault fault_code=%hu fault_detail=%u segment_id=%u",
              (uint16_t)fault_code, fault_detail, cur_seg);
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

    // FAULT → also block kicks. Emit one-shot kalico_fault event if the
    // engine just transitioned INTO Fault since the last drain (so the host
    // gets a single notification, not a 1 kHz spam stream).
    if (cur_status == 3 /* FAULT */) {
        kalico_liveness_ok = 0;
        if (prev_engine_status != 3 /* FAULT */) {
            int32_t fault_code = kalico_runtime_last_error(kalico_rt_handle);
            uint32_t fault_detail = kalico_runtime_fault_detail(kalico_rt_handle);
            uint32_t cur_seg = kalico_runtime_current_segment_id(kalico_rt_handle);
            // `output()` per the same registration-category rule as
            // `kalico_credit_freed` above.
            output("kalico_fault fault_code=%hu fault_detail=%u segment_id=%u",
                   (uint16_t)fault_code, fault_detail, cur_seg);
        }
    }
    prev_engine_status = cur_status;

    // Track last status (used by future LED hook on a non-SWD pin).
    if (cur_status != last_seen_status) {
        last_seen_status = cur_status;
    }
}
DECL_TASK(runtime_drain);

// Phase 11 Task 11.1 §5.3 periodic 10 Hz `kalico_status_v6` frame.
// Background task that polls the wake flag and emits a status frame at the
// 10 Hz cadence. Distinct from runtime_drain — this task does not depend on
// trace-ring state, so it keeps publishing status even when the engine is
// Idle/Drained and runtime_drain has nothing to do.
void
runtime_status_drain(void)
{
    if (!kalico_rt_handle) return;
    uint32_t now = timer_read_time();
    // Spec §5.3: 10 Hz cadence. The cast through int32_t handles u32 wrap
    // (~8.3 s at 520 MHz, ~83 s in sim) — at 100 ms cadence the difference
    // fits well inside a signed window.
    const uint32_t status_period_ticks = CONFIG_CLOCK_FREQ / 10;
    if ((int32_t)(now - last_status_emit_time) < (int32_t)status_period_ticks)
        return;
    last_status_emit_time = now;

    uint8_t status = kalico_runtime_status(kalico_rt_handle);
    int32_t last_err = kalico_runtime_last_error(kalico_rt_handle);
    uint32_t cur_seg = kalico_runtime_current_segment_id(kalico_rt_handle);
    uint8_t depth = kalico_runtime_queue_depth(kalico_rt_handle);
    uint64_t mcu_clk = kalico_runtime_widened_now(kalico_rt_handle);
    uint32_t epoch = kalico_runtime_credit_epoch(kalico_rt_handle);
    uint32_t accepted = kalico_runtime_accepted_segment_id(kalico_rt_handle);
    uint32_t retired = kalico_runtime_retired_through_segment_id(kalico_rt_handle);
    uint32_t fault_detail = kalico_runtime_fault_detail(kalico_rt_handle);

    // `output()` per the same registration-category rule — `_DECL_OUTPUT`
    // is the right channel for an async periodic frame.
    output("kalico_status_v6 engine_status=%c queue_depth=%c "
           "current_segment_id=%u "
           "last_fault=%hu fault_detail=%u "
           "mcu_clock_now_lo=%u mcu_clock_now_hi=%u "
           "credit_epoch=%u accepted_segment_id=%u "
           "retired_through_segment_id=%u",
           status, depth, cur_seg,
           (uint16_t)last_err, fault_detail,
           (uint32_t)mcu_clk, (uint32_t)(mcu_clk >> 32),
           epoch, accepted, retired);
}
DECL_TASK(runtime_status_drain);

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
        sendf("kalico_load_curve_response result=%i curve_handle_packed=%u", -7, 0);
        return;
    }
    // Step-6 §4.2: 1-byte format-version field travels as the first command
    // arg `version=%c`. Validate before decoding the rest of the payload.
    // KALICO_ERR_PROTOCOL_VERSION_UNSUPPORTED = -103 per §9.1.
    uint8_t  version      = args[0];
    if (version != 0x01) {
        sendf("kalico_load_curve_response result=%i curve_handle_packed=%u", -103, 0);
        return;
    }
    uint16_t slot         = args[1];
    uint8_t  degree       = args[2];
    uint16_t cps_len      = args[3];
    const uint8_t *cps_b  = command_decode_ptr(args[4]);
    uint16_t knots_len    = args[5];
    const uint8_t *knots_b = command_decode_ptr(args[6]);
    uint16_t weights_len  = args[7];
    const uint8_t *weights_b = command_decode_ptr(args[8]);

    // Producer-side validation: cps must be a multiple of 12 (xyz × f32);
    // knots and weights must be a multiple of 4 (f32); weights count must
    // equal cp count. Mismatch → KALICO_ERR_INVALID_CURVE (-2).
    if ((cps_len % 12) || (knots_len % 4) || (weights_len % 4)) {
        sendf("kalico_load_curve_response result=%i curve_handle_packed=%u", -2, 0);
        return;
    }
    uint16_t n_cp      = cps_len / 12;
    uint16_t n_knots   = knots_len / 4;
    uint16_t n_weights = weights_len / 4;
    if (n_weights != n_cp) {
        sendf("kalico_load_curve_response result=%i curve_handle_packed=%u", -2, 0);
        return;
    }
    if (cps_len > sizeof(kalico_aligned_cps) ||
        knots_len > sizeof(kalico_aligned_knots) ||
        weights_len > sizeof(kalico_aligned_weights)) {
        sendf("kalico_load_curve_response result=%i curve_handle_packed=%u", -2, 0);
        return;
    }

    // Byte-copy into the aligned scratch buffers. memcpy on Cortex-M7 with
    // -O2 lowers to a tight LDR/STR loop; the source unalignment is fine
    // because we copy bytes, not words.
    memcpy(kalico_aligned_cps, cps_b, cps_len);
    memcpy(kalico_aligned_knots, knots_b, knots_len);
    memcpy(kalico_aligned_weights, weights_b, weights_len);

    uint32_t handle_packed = 0;
    int32_t r = kalico_runtime_load_curve(
        kalico_rt_handle, slot,
        kalico_aligned_cps, n_cp,
        kalico_aligned_knots, n_knots,
        kalico_aligned_weights, n_weights,
        degree,
        &handle_packed);
    sendf("kalico_load_curve_response result=%i curve_handle_packed=%u",
          r, handle_packed);
}
DECL_COMMAND(command_kalico_load_curve,
    "kalico_load_curve version=%c slot=%hu degree=%c "
    "cps=%*s knots=%*s weights=%*s");

void
command_kalico_push_segment(uint32_t *args)
{
    if (!kalico_rt_handle) {
        sendf(
            "kalico_push_response result=%i accepted_segment_id=%u credit_epoch=%u",
            -7, 0, 0);
        return;
    }
    uint32_t id = args[0];
    // Step-6 §10.1: curve_handle widened from u16 to packed u32
    // ((generation << 16) | slot_idx). The Klipper VLQ %u encoder handles u32.
    uint32_t curve_handle_packed = args[1];
    uint64_t t_start = ((uint64_t)args[2] << 32) | args[3];
    uint64_t t_end   = ((uint64_t)args[4] << 32) | args[5];
    uint8_t kin = args[6];
    uint32_t accepted_id = 0;
    uint32_t credit_epoch = 0;
    int32_t r = kalico_runtime_push_segment(
        kalico_rt_handle, id, curve_handle_packed, t_start, t_end, kin,
        &accepted_id, &credit_epoch);
    sendf(
        "kalico_push_response result=%i accepted_segment_id=%u credit_epoch=%u",
        r, accepted_id, credit_epoch);
}
DECL_COMMAND(command_kalico_push_segment,
    "kalico_push_segment id=%u curve_handle=%u t_start_hi=%u t_start_lo=%u "
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

// ---- Step-6 §8.3 stream lifecycle commands ----------------------------
// Phase 3.2 declares the wire surface; Phase 6 wires the actual state-
// machine transitions in `runtime::stream`. The FFIs return -140
// (KALICO_ERR_STREAM_STATE_VIOLATION) until Phase 6 lands.

void
command_kalico_stream_open(uint32_t *args)
{
    if (!kalico_rt_handle) {
        sendf("kalico_stream_open_response result=%i credit_epoch=%u", -7, 0);
        return;
    }
    uint32_t stream_id = args[0];
    uint32_t credit_epoch = 0;
    int32_t r = kalico_runtime_stream_open(
        kalico_rt_handle, stream_id, &credit_epoch);
    sendf("kalico_stream_open_response result=%i credit_epoch=%u",
          r, credit_epoch);
}
DECL_COMMAND(command_kalico_stream_open, "kalico_stream_open stream_id=%u");

void
command_kalico_stream_arm(uint32_t *args)
{
    if (!kalico_rt_handle) {
        sendf(
            "kalico_stream_arm_response result=%i armed_t_start_lo=%u armed_t_start_hi=%u",
            -7, 0, 0);
        return;
    }
    uint64_t t_start_t0 = ((uint64_t)args[1] << 32) | args[0];
    uint32_t arm_lead_cycles = args[2];
    uint64_t armed_t_start = 0;
    int32_t r = kalico_runtime_stream_arm(
        kalico_rt_handle, t_start_t0, arm_lead_cycles, &armed_t_start);
    sendf(
        "kalico_stream_arm_response result=%i armed_t_start_lo=%u armed_t_start_hi=%u",
        r, (uint32_t)armed_t_start, (uint32_t)(armed_t_start >> 32));
}
DECL_COMMAND(command_kalico_stream_arm,
    "kalico_stream_arm t_start_t0_lo=%u t_start_t0_hi=%u arm_lead_cycles=%u");

void
command_kalico_stream_terminal(uint32_t *args)
{
    if (!kalico_rt_handle) {
        sendf("kalico_stream_terminal_response result=%i", -7);
        return;
    }
    uint32_t segment_id = args[0];
    int32_t r = kalico_runtime_stream_terminal(kalico_rt_handle, segment_id);
    sendf("kalico_stream_terminal_response result=%i", r);
}
DECL_COMMAND(command_kalico_stream_terminal,
    "kalico_stream_terminal segment_id=%u");

void
command_kalico_stream_flush(uint32_t *args)
{
    (void)args;
    if (!kalico_rt_handle) {
        sendf("kalico_stream_flush_response result=%i credit_epoch=%u", -7, 0);
        return;
    }
    uint32_t credit_epoch = 0;
    int32_t r = kalico_runtime_stream_flush(kalico_rt_handle, &credit_epoch);
    sendf("kalico_stream_flush_response result=%i credit_epoch=%u",
          r, credit_epoch);
}
DECL_COMMAND(command_kalico_stream_flush, "kalico_stream_flush");

// ---- Step-6 §12.1 clock-sync request ----------------------------------
void
command_kalico_clock_sync_request(uint32_t *args)
{
    if (!kalico_rt_handle) {
        sendf(
            "kalico_clock_sync_response request_id=%u mcu_clock_lo=%u mcu_clock_hi=%u",
            0, 0, 0);
        return;
    }
    uint32_t request_id = args[0];
    uint32_t host_send_time_lo = args[1];
    uint32_t host_send_time_hi = args[2];
    uint64_t mcu_clock = 0;
    kalico_runtime_clock_sync_request(
        kalico_rt_handle, request_id,
        host_send_time_lo, host_send_time_hi,
        &mcu_clock);
    sendf(
        "kalico_clock_sync_response request_id=%u mcu_clock_lo=%u mcu_clock_hi=%u",
        request_id, (uint32_t)mcu_clock, (uint32_t)(mcu_clock >> 32));
}
DECL_COMMAND(command_kalico_clock_sync_request,
    "kalico_clock_sync_request request_id=%u "
    "host_send_time_lo=%u host_send_time_hi=%u");

// ---- Step-6 §10.4 / Round-1 B9 diagnostic --------------------------------
// Per-slot curve-pool generation snapshot. Used by the host after a fault to
// decide whether the pool can be reused or a power-cycle is required.
void
command_kalico_query_pool_state(uint32_t *args)
{
    if (!kalico_rt_handle) {
        sendf(
            "kalico_pool_state_response result=%i slot_idx=%hu current_gen=%hu last_retired_gen=%hu",
            -7, (uint16_t)0, (uint16_t)0, (uint16_t)0);
        return;
    }
    uint16_t slot = args[0];
    uint16_t current_gen = 0;
    uint16_t last_retired_gen = 0;
    int32_t r = kalico_runtime_query_pool_state(
        kalico_rt_handle, slot, &current_gen, &last_retired_gen);
    sendf(
        "kalico_pool_state_response result=%i slot_idx=%hu current_gen=%hu last_retired_gen=%hu",
        r, slot, current_gen, last_retired_gen);
}
DECL_COMMAND(command_kalico_query_pool_state,
    "kalico_query_pool_state slot=%hu");

// ---- Step-6 §5/§9 async event channel declarations ---------------------
// `kalico_credit_freed` and `kalico_fault` are MCU-emitted async events
// (no DECL_COMMAND on the host-to-MCU side). The Klipper `output(FMT, ...)`
// macro at call sites already registers each format string into the data
// dictionary via `_DECL_OUTPUT` / `DECL_CTR`, so a static `DECL_CTR` here
// is the equivalent of pre-registering the schema before the first emit.
// The actual emits live in the foreground drain pipeline (Phase 11) and the
// fault-publish path (Phase 4 / Phase 11).
DECL_CTR("_DECL_OUTPUT "
         "kalico_credit_freed retired_through_segment_id=%u free_slots=%c");
DECL_CTR("_DECL_OUTPUT "
         "kalico_fault fault_code=%hu fault_detail=%u segment_id=%u");
// `kalico_status_v6` periodic frame (Phase 11 wires the emit).
DECL_CTR("_DECL_OUTPUT "
    "kalico_status_v6 engine_status=%c queue_depth=%c current_segment_id=%u "
    "last_fault=%hu fault_detail=%u "
    "mcu_clock_now_lo=%u mcu_clock_now_hi=%u "
    "credit_epoch=%u accepted_segment_id=%u retired_through_segment_id=%u");

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
    void *rt, uint16_t slot, uint16_t fixture_id, uint32_t *out_handle_packed);

void
command_kalico_load_fixture_curve(uint32_t *args)
{
    if (!kalico_rt_handle) {
        sendf("kalico_load_fixture_response result=%i curve_handle_packed=%u",
              -7, 0);
        return;
    }
    uint16_t slot = args[0];
    uint16_t fixture_id = args[1];
    uint32_t handle_packed = 0;
    int32_t r = kalico_runtime_load_fixture(
        kalico_rt_handle, slot, fixture_id, &handle_packed);
    sendf("kalico_load_fixture_response result=%i curve_handle_packed=%u",
          r, handle_packed);
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
