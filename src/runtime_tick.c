// src/runtime_tick.c
//
// Klipper-side lifecycle for the kalico runtime: DECL_INIT brings up the
// Rust runtime + the per-family tick backend; DECL_TASK pumps drain the
// Rust → Klipper response queue. Shared globals (runtime_handle,
// runtime_clock_freq, runtime_aligned_*) live here as the single
// definition site.
//
// Klipper command surface is in src/runtime_commands.c.
// Sim-only commands are in src/runtime_sim_commands.c (gated CONFIG_KALICO_SIM).
// Bench is in src/generic/runtime_bench.c (gated CONFIG_RUNTIME_BENCH).
// Per-family backends:
//   src/stm32/runtime_tick_h7.c   (H7 TIM5 ISR)
//   src/linux/runtime_tick_host.c (pthread tick for host-sim)
// Backend interface contract: src/generic/runtime_tick.h.

#include <string.h>         // memcpy
#include "stepper.h"        // runtime_emit_step_pulses (via stepper.c)
#if defined(__linux__) || defined(__APPLE__)
#include <stdio.h>          // fprintf, stderr
#include <time.h>           // clock_gettime
#endif
#include "autoconf.h"
#include "board/gpio.h"     // gpio_in_setup / gpio_in_read
#include "board/internal.h" // NVIC_*, IWDG, OTG_HS_IRQn, USART2_IRQn
#include "board/irq.h"      // irq_save, irq_restore (Step-6 §8.5 flush)
#include "board/misc.h"     // timer_read_time
#include "command.h"        // DECL_COMMAND
#include "sched.h"          // DECL_INIT, DECL_TASK
#include "kalico_runtime.h"
#include "kalico_dispatch.h" // kalico_native_emit_*
#include "generic/runtime_tick.h"   // backend interface (consumer view)
#include "generic/fault_handler.h"  // diag_record_engine_xition, diag_take_snapshot
#if CONFIG_MACH_LINUX
// Host build: pthread-driven tick replaces the TIM5 ISR. The Rust runtime
// still calls runtime_tick_enable/disable/runtime_cyccnt_read across the
// producer-protocol boundary; we provide host-side stubs in
// src/linux/runtime_tick_host.c.
#endif


// H7 CMSIS only defines IWDG1/IWDG2; map the generic name to IWDG1
// (matching src/stm32/watchdog.c's pattern) so the bench-loop kick
// compiles cleanly.
#if CONFIG_MACH_STM32H7
#define IWDG IWDG1
#endif

// Exposed to Rust via `extern "C" { static runtime_clock_freq: u32; }`.
// __attribute__((used, externally_visible)) survives -fwhole-program LTO + GC.
const uint32_t runtime_clock_freq __attribute__((used, externally_visible))
    = CONFIG_CLOCK_FREQ;

// Motion-engine sample rate (TIM5 ISR fire rate on STM32; host-pthread tick
// rate on Linux). Exposed to Rust via
// `extern "C" { static runtime_sample_rate_hz: u32; }` so Engine::init can
// publish `sample_period_sec = 1.0 / runtime_sample_rate_hz` without
// embedding a magic constant. Source: CONFIG_KALICO_MOTION_SAMPLE_RATE_HZ
// (src/Kconfig); defaults: 40000 (H7), 20000 (F4), 10000 (Linux sim).
// __attribute__((used, externally_visible)) survives -fwhole-program LTO + GC.
const uint32_t runtime_sample_rate_hz __attribute__((used, externally_visible))
    = CONFIG_KALICO_MOTION_SAMPLE_RATE_HZ;


extern volatile uint8_t runtime_liveness_ok;  // defined in src/stm32/watchdog.c

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
runtime_host_now_us(void)
{
    uint32_t cycles = timer_read_time();
    // CONFIG_CLOCK_FREQ is in Hz; divide by 1e6 to get cycles-per-µs.
    // Integer division here is fine — CONFIG_CLOCK_FREQ is always a
    // multiple of 1 MHz on supported STM32 targets.
    return ((uint64_t)cycles) / (CONFIG_CLOCK_FREQ / 1000000U);
}

// F446 configure_axes crash diagnostic (2026-05-11). Packs the latest
// (tag, stage, value) triple into a single u32 word that
// `runtime_status_drain` piggybacks onto the periodic `kalico_status_v6`
// frame's `fault_detail` field when no real fault is latched.
//
// Why not `output(...)` directly: kalico-native dispatch context (FFI
// handlers reached via the kalico-native demux) blocks the foreground
// task that drains the USB-CDC TX buffer until the handler returns.
// On F446, configure_axes_blob crashes BEFORE that return, so any
// `output()` line queued during the FFI never flushes — klippy sees
// nothing. The status-frame piggyback uses an already-running drain
// task (10 Hz cadence) that emits even while the foreground is busy.
//
// Layout: bits 24-31 = tag, 16-23 = stage, 0-15 = low 16 bits of value.
// Read by `runtime_status_drain` and surfaced as `fault_detail` when
// `last_err == 0`.
// Live diag: updated every call, sampled into the kalico_status_v6
// fault_detail field by the 10 Hz status drain (volatile so the
// compiler doesn't reorder writes — there are no atomic ordering
// requirements because foreground is single-threaded).
volatile uint32_t runtime_diag_last_packed __attribute__((used, externally_visible));

// Persistent crash diag: survives `NVIC_SystemReset` via the
// .persistent_diag linker section (NOLOAD, outside [_bss_start.._bss_end]
// so armcm_boot.c's bss-zero pass leaves it alone). On the next boot,
// `command_runtime_post_init` checks `magic == RT_DIAG_MAGIC` and emits
// the captured stage via output() so we can see WHERE the previous run
// crashed even when no BKPSRAM is available (F446 case).
#define RT_DIAG_MAGIC 0xD1A6BABE

struct rt_diag_persistent {
    uint32_t magic;
    uint32_t last_packed;
    uint32_t last_us;
    uint32_t fault_count;
};

// Non-static + volatile so fault_handler.c can `extern` it directly
// and LTO can't constant-propagate the zero-init values into the
// output() arguments. Section attribute places it outside `.bss` so
// armcm_boot.c's zero-pass leaves it alone across soft reset.
volatile struct rt_diag_persistent rt_diag_persistent
    __attribute__((section(".persistent_diag"), used, externally_visible));

// Snapshot of the prior run's packed-diag value, captured at runtime_init
// time BEFORE the current run starts overwriting rt_diag_persistent. The
// 10 Hz status drain alternates between the LIVE diag (runtime_diag_last_packed)
// and this BOOT snapshot (when valid) so klippy.log sees both — needed to
// catch F446 cause-of-death after NVIC_SystemReset, since the output() emit
// in fault_handler_report_task gets dropped by USB-CDC TX overrun during
// the boot_diag burst (320-byte transmit_buf vs ~600 B/cycle).
volatile uint32_t runtime_diag_prior_boot_snapshot
    __attribute__((used, externally_visible));

// Verification globals — capture rt_diag_persistent contents at runtime_init
// time so we can confirm whether .persistent_diag actually survives a soft
// reset on STM32F4 (unverified before this commit; F4 reference manual is
// ambiguous about SRAM survival across SYSRESETREQ).
volatile uint32_t runtime_diag_prior_magic_raw
    __attribute__((used, externally_visible));
volatile uint32_t runtime_diag_prior_packed_raw
    __attribute__((used, externally_visible));

__attribute__((used, externally_visible))
void
runtime_diag_progress(uint32_t tag, uint32_t stage, uint32_t value)
{
    uint32_t packed = ((tag & 0xFFu) << 24)
                    | ((stage & 0xFFu) << 16)
                    | (value & 0xFFFFu);
    runtime_diag_last_packed = packed;
    rt_diag_persistent.magic = RT_DIAG_MAGIC;
    rt_diag_persistent.last_packed = packed;
    rt_diag_persistent.last_us = timer_read_time();
}

// Emission of rt_diag_persistent is inlined into
// `src/generic/fault_handler.c::fault_handler_report_task` because
// whole-program LTO was stripping a standalone helper.

// Klipper-widened DWT/timer clock (cycles, u64). Mirrors
// command_get_uptime's widening (basecmd.c:300-304): reads `cur` first,
// then the high half with a "pre-stats_update wrap" lookback against
// stats_send_time. Because this widening rides on Klipper's stats task
// (~5 s cadence) and not on the kalico TIM5 ISR, the value advances
// monotonically regardless of whether the engine is Idle / Running /
// Drained / Fault — which is the property kalico_clock_sync_response
// needs after a drain. Foreground-only; do NOT call from ISR.
//
// Bench symptom 2026-05-11: clock_sync_respond read engine-side
// `read_widened_now` which is published only by the TIM5 ISR. On Drained
// → TIM5 disabled → widened_now froze → host's regression flatlined →
// the next jog's t_start_clock landed in the MCU's real past →
// boundary loop silently retired the segment without producing step
// pulses. Using Klipper's stats-based widening here decouples
// clock-sync from engine activity.
__attribute__((used, externally_visible))
uint64_t
runtime_widened_host_clock(void)
{
    extern uint32_t stats_send_time;
    extern uint32_t stats_send_time_high;
    uint32_t cur = timer_read_time();
    uint32_t high = stats_send_time_high + (cur < stats_send_time);
    return ((uint64_t)high << 32) | (uint64_t)cur;
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
// Solution: provide thin wrappers `runtime_irq_save` / `runtime_irq_restore`
// that the staticlib calls instead of `irq_save` / `irq_restore` directly.
// The wrappers are marked `used, externally_visible` so LTO keeps them.
// They forward to the real functions, which LTO can still inline if it
// wants — but the staticlib only sees the wrapper symbols.
__attribute__((used, externally_visible))
uint32_t
runtime_irq_save(void)
{
    return (uint32_t)irq_save();
}

__attribute__((used, externally_visible))
void
runtime_irq_restore(uint32_t flags)
{
    irq_restore((irqstatus_t)flags);
}

void* runtime_handle = 0;            // exposed (non-static) for runtime_tick_h7.c
struct task_wake runtime_drain_wake;  // non-static: shared with runtime_sim_commands.c
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
    // Now-relative reschedule, NOT `+= 1 ms` from the previous waketime.
    // Any 1 ms+ of foreground starvation makes the `+=` form's next
    // reschedule a past clock relative to wall-clock now, and Klipper's
    // armcm_timer.c dispatcher fires `try_shutdown("Rescheduled timer
    // in the past")`. Anchoring to `timer_read_time()` keeps the
    // reschedule strictly in the future regardless of upstream delay;
    // the drain timer's role is sample-shipping and 10 Hz status emit,
    // neither of which cares about exact phase-locking.
    t->waketime = timer_read_time() + timer_from_us(1000);  // 1 kHz
    return SF_RESCHEDULE;
}

void
runtime_init(void)
{
    // 2026-05-20: capture prior-boot diag FIRST (before our markers
    // overwrite it). Status drain emits this snapshot via klippy
    // periodic-status, so once we enumerate we can read the prior
    // boot's last marker over USB.
    extern volatile uint32_t runtime_diag_prior_magic_raw;
    extern volatile uint32_t runtime_diag_prior_packed_raw;
    runtime_diag_prior_magic_raw = rt_diag_persistent.magic;
    runtime_diag_prior_packed_raw = rt_diag_persistent.last_packed;
    if (rt_diag_persistent.magic == RT_DIAG_MAGIC
        && rt_diag_persistent.last_packed != 0) {
        runtime_diag_prior_boot_snapshot = rt_diag_persistent.last_packed;
    }

    // 2026-05-20 bisect probe.
    runtime_diag_progress(0xB0, 0, 0);

    // 2026-05-20 bisect: STUB=1 short-circuits runtime_init's body.
    // With STUB=1, USB enumeration tells us the crash is below this
    // point. The prior-boot snapshot capture above survives so a klippy
    // status frame can tell us the last marker the crashing firmware
    // wrote.
#define RUNTIME_INIT_STUB 0  /* DIAG: 1 stubs runtime_init for bisect */
#if RUNTIME_INIT_STUB
    runtime_diag_progress(0xBF, 0, 0xCAFE);
    return;
#endif

    runtime_diag_progress(0xB1, 0, 0);  // about to call runtime_handle_create
    runtime_handle = runtime_handle_create();
    if (!runtime_handle) {
        // Init failed — leave liveness flag at default (1 = OK) but handle unset;
        // calls into the runtime will short-circuit safely.
        runtime_diag_progress(0xB1, 1, 0xFFFF);  // handle_create returned NULL
        return;
    }
    runtime_diag_progress(0xB2, 0, 0);  // handle_create succeeded
    last_seen_tick_counter = runtime_handle_tick_counter(runtime_handle);
    last_progress_time = timer_read_time();
    last_seen_status = runtime_handle_status(runtime_handle);
    runtime_diag_progress(0xB3, 0, 0);  // status reads done

    // Initialize the modulation tick driver. On STM32H7 this configures
    // TIM5 (DOES NOT enable; the first segment push triggers enable via
    // the producer protocol §4.4). On Linux it spawns the host pthread
    // that calls runtime_handle_tick at 40 kHz.
    runtime_diag_progress(0xB4, 0, 0);  // about to call runtime_tick_init
    runtime_tick_init();
    runtime_diag_progress(0xB5, 0, 0);  // tick_init done

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

// runtime_sim_drain_calls extern retired with runtime_sim_commands.c in
// 085b4b16f; the diag-heartbeat scaffolding now lives in
// diag_task_heartbeat below.

void
runtime_drain(void)
{
    if (!runtime_handle) return;
    if (!sched_check_wake(&runtime_drain_wake)) return;

    // Diag heartbeat for runtime_drain. Threshold: 50 ms (engine drain is
    // expected to run ~1 kHz under load).
    diag_task_heartbeat(diag_slot_rt_drain_calls(),
                        diag_slot_rt_drain_last_tick(),
                        diag_slot_rt_drain_max_gap(),
                        timer_from_us(50000),
                        0); // no event tag — runtime_drain idle gaps are normal

    // Phase 11 Task 11.2 §10.4 reclaim drain pipeline. Drains a batch of
    // trace samples for transport to the host, then asks the Rust side to
    // also drain-and-reclaim its own internal cursor for SEGMENT_END events
    // (so curve-pool slots get returned promptly) and check the §13.1
    // trace-overflow latch. The two drain paths share the same SPSC ring
    // (same FgState consumer); the order matters — `runtime_handle_drain_trace`
    // moves samples to the wire FIRST so the host sees the trace data, THEN
    // `kalico_runtime_drain_and_reclaim` consumes any remaining samples for
    // bookkeeping. Both are safe back-to-back because each stops on the
    // first dequeue == None.
    static uint8_t batch_buf[KALICO_TRACE_BATCH * 40];  // 40 bytes per sample
    uint8_t trace_saw_segment_end = 0;
    uint32_t n = runtime_handle_drain_trace(
        runtime_handle, (struct TraceSample*)batch_buf, KALICO_TRACE_BATCH,
        &trace_saw_segment_end);
    if (n > 0) {
        // FORMAT-VERSION EXEMPTION (Phase 3.1 / closure-review):
        // Phase 3.1 added a 1-byte FORMAT_VERSION_V1 = 0x01 prefix on
        // host→MCU blob payloads (load_curve cps/knots/weights). The
        // MCU→host trace blob is intentionally NOT versioned: it is a
        // one-shot variable-length stream of `TraceSample` records whose
        // schema is sanity-checked at compile time via the static_assert
        // on `sizeof(TraceSample) == 40` plus the cbindgen-no-drift CI
        // check. Adding a per-batch version byte would burn 1.5% of every
        // 64-sample drain (32 vs 33-byte alignment loss) for no decoder
        // benefit — the host knows the schema from the data dictionary,
        // and a wire-protocol version bump would change the data dict
        // (different msgid for `kalico_trace`) anyway.
        // See plan-changes-log.md "Step-6 closure-review follow-up fixes"
        // entry for the full reasoning.
        output("kalico_trace count=%u data=%*s", n, n * 40, batch_buf);
    }

    // Reclaim leg: drain whatever the wire-batch left behind and observe
    // SEGMENT_END events for curve-pool reclaim + trace-overflow check.
    // Returns a packed status word — see kalico_runtime_drain_and_reclaim
    // doc-comment for the bit layout.
    //
    // Closure-review fix: `kalico_credit_freed` MUST OR the trace leg's
    // saw_segment_end bit with the reclaim leg's. The trace leg already
    // calls pool.confirm_retired and consumes SEGMENT_END samples, so under
    // steady-state push the reclaim leg sees nothing — gating credit
    // emission on the reclaim leg alone deadlocks host flow control once
    // the host's credit counter drains.
    uint32_t reclaim_status = kalico_runtime_drain_and_reclaim(
        runtime_handle, KALICO_TRACE_BATCH);
    uint8_t saw_segment_end = trace_saw_segment_end |
                              (uint8_t)((reclaim_status >> 17) & 1);
    uint8_t fresh_overflow_fault = (reclaim_status >> 16) & 1;

    // Credit-flow signal: fire-and-forget `kalico_credit_freed` as a
    // low-latency fast path. The load-bearing signal is the
    // `retired_through_segment_id` field on the 10 Hz periodic StatusEvent
    // (see kalico_dispatch.c::kalico_native_emit_status_event v2) — so if
    // this fire-and-forget emit is dropped under USB-CDC TX congestion, the
    // next status frame catches the host's slot pool up within 100 ms. We
    // therefore emit unconditionally on cursor advance / SEGMENT_END and
    // always advance the local tracker, regardless of transmit_buf result.
    static uint32_t last_emitted_retired_id = 0;
    uint32_t cur_retired = runtime_handle_retired_through_segment_id(runtime_handle);
    bool cursor_advanced = (int32_t)(cur_retired - last_emitted_retired_id) > 0;
    if (saw_segment_end || cursor_advanced) {
        uint8_t depth = runtime_handle_queue_depth(runtime_handle);
        uint8_t free_slots = (depth >= 7) ? 0 : (uint8_t)(7 - depth);
        (void)kalico_native_emit_credit_freed(cur_retired, free_slots);
        last_emitted_retired_id = cur_retired;
    }

    // §13.1: a fresh trace-overflow latch is reported via the `kalico_fault`
    // async event. The fault state itself is now in shared.last_error +
    // shared.runtime_status (latched by check_trace_overflow_and_fault on
    // the Rust side); the periodic `kalico_status_v6` frame echoes it on
    // the next 10 Hz tick. We send the async event here so the host gets
    // the fault notification immediately, not up to ~100 ms later.
    if (fresh_overflow_fault) {
        int32_t fault_code = runtime_handle_last_error(runtime_handle);
        uint32_t fault_detail = runtime_handle_fault_detail(runtime_handle);
        uint32_t cur_seg = runtime_handle_current_segment_id(runtime_handle);
        kalico_native_emit_fault_event((uint16_t)fault_code, fault_detail, cur_seg);
    }

    // Liveness check. Only meaningful when the runtime is RUNNING — the ISR
    // is deliberately disabled in IDLE/DRAINED (no segment pushed yet) and
    // tick_counter cannot advance, so we'd trip a false positive within
    // KALICO_LIVENESS_THRESHOLD_MS of boot otherwise. We refresh the
    // last_progress_time anchor in non-RUNNING states so a state transition
    // INTO RUNNING doesn't immediately trip on a stale anchor.
    uint32_t cur_counter = runtime_handle_tick_counter(runtime_handle);
    uint32_t cur_time = timer_read_time();
    uint8_t cur_status = runtime_handle_status(runtime_handle);
    if (cur_status == 1 /* RUNNING */) {
        if (cur_counter != last_seen_tick_counter) {
            last_seen_tick_counter = cur_counter;
            last_progress_time = cur_time;
        } else if ((cur_time - last_progress_time) > KALICO_LIVENESS_THRESHOLD_TICKS) {
            // ISR has stalled while RUNNING. Stop kicking the watchdog.
            runtime_liveness_ok = 0;
        }
    } else {
        last_progress_time = cur_time;
        last_seen_tick_counter = cur_counter;
    }

    // FAULT → also block kicks. Emit one-shot kalico_fault event if the
    // engine just transitioned INTO Fault since the last drain (so the host
    // gets a single notification, not a 1 kHz spam stream).
    if (cur_status == 3 /* FAULT */) {
        runtime_liveness_ok = 0;
        if (prev_engine_status != 3 /* FAULT */) {
            int32_t fault_code = runtime_handle_last_error(runtime_handle);
            uint32_t fault_detail = runtime_handle_fault_detail(runtime_handle);
            uint32_t cur_seg = runtime_handle_current_segment_id(runtime_handle);
            kalico_native_emit_fault_event((uint16_t)fault_code, fault_detail, cur_seg);
        }
    }

    // DRAINED or FAULT → disable TIM5 on the first transition into that
    // state. The engine has nothing left to evaluate; leaving TIM5 running at
    // 40 kHz needlessly burns CPU cycles. Under Renode the ISR load also
    // starves USART2 command dispatch, preventing host tools from talking to
    // the firmware after a print completes. The §4.4 producer protocol
    // re-enables TIM5 on the next runtime_handle_push_segment call when
    // status is IDLE or DRAINED, so this is safe. Under IDLE the ISR was
    // never enabled (no-op to call disable), but we gate on the transition
    // anyway to avoid redundant disable calls.
    if ((cur_status == 2 /* DRAINED */ || cur_status == 3 /* FAULT */)
        && prev_engine_status != cur_status) {
        runtime_tick_disable();
    }

    if (cur_status != prev_engine_status) {
        // Diag: capture every engine state transition with the engine's
        // own tick_counter as temporal context. Catches the hypothesised
        // "engine briefly armed in IRQ then reverted" scenario by virtue
        // of having multiple xitions in tight succession.
        diag_record_engine_xition(prev_engine_status, cur_status, cur_counter);
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
    if (!runtime_handle) return;
    uint32_t now = timer_read_time();
    // Spec §5.3: 10 Hz cadence. The cast through int32_t handles u32 wrap
    // (~8.3 s at 520 MHz, ~83 s in sim) — at 100 ms cadence the difference
    // fits well inside a signed window.
    const uint32_t status_period_ticks = CONFIG_CLOCK_FREQ / 10;
    if ((int32_t)(now - last_status_emit_time) < (int32_t)status_period_ticks)
        return;
    last_status_emit_time = now;

    // Diag heartbeat for the status emit task. Threshold: 200 ms (we run
    // at 10 Hz so a 200 ms gap means we missed two cycles, which is what
    // we expect during the 500 ms stall).
    diag_task_heartbeat(diag_slot_rt_status_calls(),
                        diag_slot_rt_status_last_tick(),
                        diag_slot_rt_status_max_gap(),
                        timer_from_us(200000),
                        0); // no event tag — emit gap shows up as missing emits

    uint8_t status = runtime_handle_status(runtime_handle);
    int32_t last_err = runtime_handle_last_error(runtime_handle);
    uint32_t cur_seg = runtime_handle_current_segment_id(runtime_handle);
    uint8_t depth = runtime_handle_queue_depth(runtime_handle);
    uint32_t fault_detail = runtime_handle_fault_detail(runtime_handle);

    // F446-configure_axes diag piggyback: when no real fault is latched,
    // surface the latest packed `(tag, stage, value)` diag triple in the
    // status frame's `fault_detail` field. Klippy already logs every
    // status frame's fault_detail, so we see live FFI progress at the
    // 10 Hz status cadence without needing the foreground-blocked
    // `output(...)` path. See `runtime_diag_progress` comments above.
    //
    // Alternation: cycle through 4 phases so klippy.log captures both the
    // live diag AND the post-reset snapshot data (live overwrites the
    // single u32 fault_detail field within ~100 ms of a reset, before
    // klippy can record the prior value).
    //   phase 0: live diag (runtime_diag_last_packed)
    //   phase 1: prior-boot snapshot (rt_diag_persistent.last_packed
    //            captured at runtime_init before overwrite)
    //   phase 2: raw magic word read at runtime_init (verifies
    //            .persistent_diag survives the reset — should be RT_DIAG_MAGIC)
    //   phase 3: raw last_packed read at runtime_init (= snapshot, doubled
    //            for redundancy in case the host drops one frame).
    static uint8_t status_emit_phase;
    // 2026-05-13 bench debug: extend to 6 phases so emit_calls /
    // emit_pulses / stepper_count snapshots are surfaced via fault_detail.
    // Goal: figure out why segments retire (engine reaches Drained) but no
    // step pulses reach the motor pins.
    status_emit_phase = (uint8_t)(status_emit_phase + 1);
    if (status_emit_phase >= 6) status_emit_phase = 0;
    if (last_err == 0) {
        extern volatile uint32_t runtime_emit_calls;
        extern volatile uint32_t runtime_emit_pulses;
        extern uint8_t runtime_motor_binding_count(uint8_t motor_idx);
        switch (status_emit_phase) {
        case 0:
            if (runtime_diag_last_packed != 0)
                fault_detail = runtime_diag_last_packed;
            break;
        case 1:
            if (runtime_diag_prior_boot_snapshot != 0)
                fault_detail = runtime_diag_prior_boot_snapshot;
            break;
        case 2:
            fault_detail = runtime_diag_prior_magic_raw;
            break;
        case 3:
            fault_detail = runtime_diag_prior_packed_raw;
            break;
        case 4:
            // Tag 0xE1 marker in high byte; low 24 bits = emit_calls.
            fault_detail = 0xE1000000u | (runtime_emit_calls & 0x00FFFFFFu);
            break;
        case 5:
            // Tag 0xE2 marker in high byte; bits 16..23 = emit_pulses & 0xFF;
            // bits 0..15 = motor_stepper_count[0..3], 4 bits each (clamped
            // to 15). The previous 2-bits-per-motor encoding silently aliased
            // count=4 to count=0 — RUNTIME_MAX_STEPPERS_PER_MOTOR is 4, so the
            // legitimate values 0..4 were observationally indistinguishable.
            // 4-bit clamp resolves it.
            {
                uint8_t c0 = runtime_motor_binding_count(0);
                uint8_t c1 = runtime_motor_binding_count(1);
                uint8_t c2 = runtime_motor_binding_count(2);
                uint8_t c3 = runtime_motor_binding_count(3);
                if (c0 > 15) c0 = 15;
                if (c1 > 15) c1 = 15;
                if (c2 > 15) c2 = 15;
                if (c3 > 15) c3 = 15;
                uint16_t cnts = (uint16_t)c0
                              | ((uint16_t)c1 << 4)
                              | ((uint16_t)c2 << 8)
                              | ((uint16_t)c3 << 12);
                uint8_t pulses_lo = (uint8_t)(runtime_emit_pulses & 0xFFu);
                fault_detail = 0xE2000000u
                             | ((uint32_t)pulses_lo << 16)
                             | (uint32_t)cnts;
            }
            break;
        }
    }
    extern uint8_t runtime_motor_binding_count(uint8_t motor_idx);
    extern volatile uint32_t handle_push_segment_calls_total;
    extern volatile uint32_t handle_push_segment_invalid_body_total;
    extern volatile uint32_t handle_push_segment_no_handle_total;
    extern volatile int32_t handle_push_segment_last_r;
    extern volatile uint32_t kalico_demux_out_kalico_total;
    extern volatile uint32_t kalico_demux_out_error_total;
    extern volatile uint32_t kalico_demux_crc_mismatch_total;
    if (last_err == 0 && status_emit_phase == 0) {
        // Diag rotation: producer-side tags (0xB2..0xB5), handler-side
        // tags (0xB6, 0xB7), curve-resolve tag (0xB8), demuxer tag (0xB9).
        static uint8_t st_emit_phase_ext;
        st_emit_phase_ext = (uint8_t)(st_emit_phase_ext + 1);
        if (st_emit_phase_ext >= 38) st_emit_phase_ext = 0;
        switch (st_emit_phase_ext) {
        case 3:
            // 0xE6 — Live step_mode discriminants for motors 0..3, two
            // bits each: bit 0 of each pair = mode (0=Modulated/1=StepTime),
            // bit 1 = "is at least one binding registered" (1 = yes).
            // Bit-packed into low byte; binding-presence in high nibble.
            {
                uint8_t modes_lo = 0;
                uint8_t binds_lo = 0;
                for (uint8_t i = 0; i < 4; i++) {
                    uint8_t m = kalico_runtime_get_step_mode(runtime_handle, i);
                    if (m == 1) modes_lo |= (uint8_t)(1u << i);
                    if (runtime_motor_binding_count(i) > 0)
                        binds_lo |= (uint8_t)(1u << i);
                }
                fault_detail = 0xE6000000u | ((uint32_t)binds_lo << 8) | modes_lo;
            }
            break;
        case 10:
            // 0xB6 — kalico_dispatch handle_push_segment counters.
            //   bits  0..15: handle_push_segment_calls_total & 0xFFFF
            //   bits 16..19: invalid_body_total & 0xF
            //   bits 20..23: no_handle_total & 0xF
            // If calls > 0 but invalid_body == 0 && no_handle == 0, the
            // C handler IS dispatching to runtime_handle_push_segment.
            {
                uint32_t c = handle_push_segment_calls_total & 0xFFFFu;
                uint32_t ib = handle_push_segment_invalid_body_total & 0xFu;
                uint32_t nh = handle_push_segment_no_handle_total & 0xFu;
                fault_detail = 0xB6000000u | (nh << 20) | (ib << 16) | c;
            }
            break;
        case 13:
            // 0xB9 — demuxer outcome counters. Distinguishes:
            //   bits  0.. 7: kalico_demux_out_kalico_total & 0xFF
            //                — frames demuxed and dispatched.
            //   bits  8..15: kalico_demux_crc_mismatch_total & 0xFF
            //                — frames silently dropped due to CRC fail.
            //   bits 16..23: kalico_demux_out_error_total & 0xFF
            //                — total OUT_ERROR (includes CRC fail and
            //                  bad-length-field paths).
            // If out_kalico > 0 → demuxer is delivering frames to dispatch.
            // If crc_mismatch > 0 while out_kalico stays near zero → frames
            //   are arriving but CRC is failing (kalico_buf in wrong RAM,
            //   USB byte loss, framing offset, etc.).
            {
                uint32_t ok = kalico_demux_out_kalico_total & 0xFFu;
                uint32_t crc = kalico_demux_crc_mismatch_total & 0xFFu;
                uint32_t err = kalico_demux_out_error_total & 0xFFu;
                fault_detail = 0xB9000000u | (err << 16) | (crc << 8) | ok;
            }
            break;
        case 11:
            // 0xB7 — last r returned by runtime_handle_push_segment
            // (the C-side capture of the Rust FFI return). 0 = OK,
            // negative = error. Compare with 0xB5
            // (kalico_runtime_last_push_segment_result, which is set
            // INSIDE the Rust FFI wrapper). If 0xB7 != 0xB5, something
            // between the Rust wrapper and the C caller is munging the
            // return value.
            {
                int32_t r = handle_push_segment_last_r;
                fault_detail = 0xB7000000u | ((uint32_t)r & 0x00FFFFFFu);
            }
            break;
        case 9:
            // 0xB5 — last push_segment result code. Signed i32 from the
            // runtime; carried in low 24 bits as two's-complement low
            // bytes. 0 = KALICO_OK. Negative values map to error codes in
            // rust/runtime/src/error.rs (e.g. -1 = NULL_PTR, -2 = NOT_INIT,
            // -10 = FAULT_LATCHED, -20 = ZERO_DURATION, -21 = INVALID_DURATION,
            // -22 = INVALID_KINEMATICS, -23 = QUEUE_FULL,
            // -141 = SEGMENT_ID_NON_MONOTONIC).
            {
                int32_t r = kalico_runtime_last_push_segment_result(runtime_handle);
                fault_detail = 0xB5000000u | ((uint32_t)r & 0x00FFFFFFu);
            }
            break;
        // 2026-05-17 H7 USB-OUT wedge investigation. The kalico_status_v6
        // emit (via bulk-IN) keeps flowing during the wedge, but the host
        // can no longer write to bulk-OUT. These 7 tags expose the live
        // OTG IRQ + bulk-OUT task counters so we can pin which stage stops
        // advancing at the wedge moment. Cheap: single u32 read each.
        case 19: {
            // 0xF0 — OTG RXFLVL IRQ count (low 24 bits). If this stops
            // advancing during the wedge, OTG IRQ is no longer firing on
            // RX (or RXFLVLM bit was cleared).
#if CONFIG_USBSERIAL && (CONFIG_MACH_STM32H7 || CONFIG_MACH_STM32F4 || CONFIG_MACH_STM32F7)
            extern uint32_t diag_get_otg_rxflvl(void);
            fault_detail = 0xF0000000u | (diag_get_otg_rxflvl() & 0x00FFFFFFu);
#else
            fault_detail = 0xF0000000u;
#endif
            break;
        }
        case 20: {
            // 0xF1 — usb_notify_bulk_out call count (low 24 bits). If
            // this advances but task_invoke (0xF2) stagnates, sched_wake
            // is being suppressed.
#if CONFIG_USBSERIAL && (CONFIG_MACH_STM32H7 || CONFIG_MACH_STM32F4 || CONFIG_MACH_STM32F7)
            extern uint32_t diag_get_notify_bulk_out(void);
            fault_detail = 0xF1000000u | (diag_get_notify_bulk_out() & 0x00FFFFFFu);
#else
            fault_detail = 0xF1000000u;
#endif
            break;
        }
        case 21: {
            // 0xF2 — usb_bulk_out_task entry count (low 24 bits). If
            // this stops while notify_n grows, foreground is starved.
#if CONFIG_USBSERIAL && (CONFIG_MACH_STM32H7 || CONFIG_MACH_STM32F4 || CONFIG_MACH_STM32F7)
            extern uint32_t diag_get_task_invoke(void);
            fault_detail = 0xF2000000u | (diag_get_task_invoke() & 0x00FFFFFFu);
#else
            fault_detail = 0xF2000000u;
#endif
            break;
        }
        case 22: {
            // 0xF3 — bulk-OUT reads that returned data (low 24 bits).
            // If this stops while task_n keeps growing, EP is being
            // drained but returning nothing — RX FIFO empty or NAKed.
#if CONFIG_USBSERIAL && (CONFIG_MACH_STM32H7 || CONFIG_MACH_STM32F4 || CONFIG_MACH_STM32F7)
            extern uint32_t diag_get_read_data(void);
            fault_detail = 0xF3000000u | (diag_get_read_data() & 0x00FFFFFFu);
#else
            fault_detail = 0xF3000000u;
#endif
            break;
        }
        case 23: {
            // 0xF4 — RX endpoint re-arm count (low 24 bits). If this
            // stops, EP never re-armed → host writes pile up unread.
#if CONFIG_USBSERIAL && (CONFIG_MACH_STM32H7 || CONFIG_MACH_STM32F4 || CONFIG_MACH_STM32F7)
            extern uint32_t diag_get_enable_rx_rearm(void);
            fault_detail = 0xF4000000u | (diag_get_enable_rx_rearm() & 0x00FFFFFFu);
#else
            fault_detail = 0xF4000000u;
#endif
            break;
        }
        case 24: {
            // 0xF5 — LIVE OUT EP DOEPCTL register (low 24 bits, masked).
            // Live-read (not cached snapshot) so we observe the actual
            // hardware state during the wedge. Bits of interest visible
            // here:
            //   0x800000 EPENA — EP enabled to receive (bit 31, dropped)
            //   0x020000 NAKSTS — EP NAKing (sticky)
            //   0x010000 STALL  — EP stalling
            //   0x008000 USBAEP — EP active in this configuration
#if CONFIG_USBSERIAL && (CONFIG_MACH_STM32H7 || CONFIG_MACH_STM32F4 || CONFIG_MACH_STM32F7)
            extern void usb_diag_read_out_ep(uint32_t *, uint32_t *, uint32_t *);
            uint32_t doepctl = 0, doeptsiz = 0, doepint = 0;
            usb_diag_read_out_ep(&doepctl, &doeptsiz, &doepint);
            fault_detail = 0xF5000000u | (doepctl & 0x00FFFFFFu);
#else
            fault_detail = 0xF5000000u;
#endif
            break;
        }
        case 25: {
            // 0xF6 — LIVE OUT EP DOEPTSIZ register (low 24 bits).
            //   bits 19..29 PKTCNT — packets remaining to receive
            //   bits 0..18  XFRSIZ — bytes remaining to receive
            // If PKTCNT==0 in the wedge state, EP is idle waiting to be
            // re-armed — confirming the re-arm path didn't fire.
#if CONFIG_USBSERIAL && (CONFIG_MACH_STM32H7 || CONFIG_MACH_STM32F4 || CONFIG_MACH_STM32F7)
            extern void usb_diag_read_out_ep(uint32_t *, uint32_t *, uint32_t *);
            uint32_t doepctl = 0, doeptsiz = 0, doepint = 0;
            usb_diag_read_out_ep(&doepctl, &doeptsiz, &doepint);
            fault_detail = 0xF6000000u | (doeptsiz & 0x00FFFFFFu);
#else
            fault_detail = 0xF6000000u;
#endif
            break;
        }
        case 26: {
            // 0xF7 — LIVE diag.tim5_irq_count (low 24 bits). 2026-05-17
            // diagnostic for "F4 dequeues segments via producer_step but
            // never retires" investigation: if this stays 0 while
            // current_segment_id > 0, TIM5 ISR is not firing →
            // runtime_modulated_tick can never advance retired_through_segment_id
            // → no kalico_credit_freed → host slot pool deadlocks.
            // Counter is incremented in diag_tim5_account
            // (src/generic/fault_handler.c:409) on every ISR entry.
            uint32_t tim5_n = diag_get_tim5_count();
            fault_detail = 0xF7000000u | (tim5_n & 0x00FFFFFFu);
            break;
        }
        case 27: {
            // 0xF8 — LIVE per-segment retire diagnostic. 2026-05-17 follow-on
            // diagnostic: if 0xF7 shows TIM5 firing but the slot pool still
            // deadlocks, retirement isn't completing. This tag exposes the
            // current_segment_id and shared.retired_through_segment_id
            // simultaneously so they can be compared without a status-frame
            // round-trip race.
            //   bits 0..11   current_segment_id (low 12 bits)
            //   bits 12..23  retired_through_segment_id (low 12 bits)
            uint32_t cur = runtime_handle_current_segment_id(runtime_handle);
            uint32_t ret = runtime_handle_retired_through_segment_id(runtime_handle);
            fault_detail = 0xF8000000u
                         | ((ret & 0xFFFu) << 12)
                         | (cur & 0xFFFu);
            break;
        }
        case 28: {
            // 0xF9 — LIVE TX drop counters. 2026-05-17 follow-on: if 0xF8
            // shows retired_through_segment_id advancing but the host never
            // receives kalico_credit_freed events, the F4 emit path is
            // probably TX-dropping the frame at console_sendf /
            // kalico_console_write_raw due to a full transmit_buf.
            //   bits 0..11   tx_drops_kalico  (low 12 bits)
            //   bits 12..23  tx_drops_klipper (low 12 bits)
            uint32_t kdrops = diag_get_tx_drops_kalico();
            uint32_t pdrops = diag_get_tx_drops_klipper();
            fault_detail = 0xF9000000u
                         | ((pdrops & 0xFFFu) << 12)
                         | (kdrops & 0xFFFu);
            break;
        }
        case 29: {
            // 0xFA — LIVE runtime_drain call count (low 24 bits). 2026-05-17
            // follow-on: if 0xF8 shows retired_through > 0 (segments retired)
            // but the host receives ZERO kalico_credit_freed events even
            // under brute-force 1 kHz re-emit, the runtime_drain task itself
            // may not be running on this MCU — credit_freed only emits from
            // runtime_drain. If this stays 0, the DECL_TASK isn't being
            // scheduled.
            volatile uint32_t *drain_calls_slot = diag_slot_rt_drain_calls();
            uint32_t drain_calls = drain_calls_slot ? *drain_calls_slot : 0;
            fault_detail = 0xFA000000u | (drain_calls & 0x00FFFFFFu);
            break;
        }
        case 30: {
            // 0xFB — LIVE last_modulated_elapsed (low 24 bits). 2026-05-17
            // F4 retire-stall investigation: if 0xF8 shows segments queued
            // (cur > 0) but retirement never advances (ret stays 0), the
            // engine's clock and the segment's `t_start` are misaligned —
            // `runtime_modulated_tick`'s `elapsed = now - t_start` stays
            // small (or 0 from saturating_sub), so the retirement branch
            // `elapsed >= duration` can't fire. Compare with 0xFC.
            uint32_t elapsed =
                runtime_handle_last_modulated_elapsed_lo(runtime_handle);
            fault_detail = 0xFB000000u | (elapsed & 0x00FFFFFFu);
            break;
        }
        case 31: {
            // 0xFC — LIVE last_modulated_duration (low 24 bits). Pair with
            // 0xFB: if elapsed < duration consistently while cur > 0, the
            // engine isn't advancing through the segment's wall-clock window.
            uint32_t duration =
                runtime_handle_last_modulated_duration_lo(runtime_handle);
            fault_detail = 0xFC000000u | (duration & 0x00FFFFFFu);
            break;
        }
        case 33: {
            // 0xFE — Last seg.consumers_remaining AFTER the clear-all-motors
            // loop in modulated_tick's retirement branch. If non-zero,
            // those are the bits the per-motor clear didn't reach.
            uint32_t cr =
                runtime_handle_last_retire_consumers_after_clear(runtime_handle);
            fault_detail = 0xFE000000u | (cr & 0x00FFFFFFu);
            break;
        }
        case 34: {
            // 0xFF — LIVE retired_through_segment_id low 24 bits. The F8 tag
            // packs cur+retired into 12 bits each, hiding actual retired_through
            // when seg IDs exceed 4095. This exposes the full low 24 bits.
            uint32_t ret =
                runtime_handle_retired_through_segment_id(runtime_handle);
            fault_detail = 0xFF000000u | (ret & 0x00FFFFFFu);
            break;
        }
        case 35: {
            // 0xE3 — ISR liveness + pending-segment state (2026-05-21).
            // Disambiguates the bench symptom "queue_depth > 0 but engine
            // never arms": three independent observables in 24 bits.
            //
            //   bits  0..15: tick_counter & 0xFFFF — bumped per
            //                 isr_sample_tick call. 0 = TIM5 not firing
            //                 (or kalico_runtime_tick_sample early-exits
            //                 at null/INIT_DONE guards).
            //   bit  16:     pending_segment_is_some — 1 if the ISR
            //                 dequeued a segment but parked it because
            //                 seg.t_start > widened_now.
            //   bits 17..23: queue_consumer_dequeues_total & 0x7F —
            //                 successful dequeues low 7 bits. 0 with
            //                 tick_counter > 0 means
            //                 queue_consumer.dequeue() returns None despite
            //                 kalico_native_queue_len() > 0 (C/Rust queue
            //                 sync bug — see 2026-05-18 SPSC miscompile).
            //
            // Reading guide for the host:
            //   tc=0                      → TIM5 dead (check carl's gate)
            //   tc>0, deq=0               → ISR fires, queue dequeue broken
            //   tc>0, deq>0, pending=1    → arm-from-queue parks forever
            //                                (widened_now stale / lead bug)
            //   tc>0, deq>0, pending=0,
            //   current_segment_id=0      → arm_segment ran but didn't
            //                                land — investigate engine state
            extern uint32_t kalico_runtime_get_tick_counter(void* handle);
            extern uint8_t  kalico_runtime_pending_segment_is_some(void* handle);
            extern uint32_t kalico_runtime_queue_consumer_dequeues_lo(void* handle);
            uint32_t tc = kalico_runtime_get_tick_counter(runtime_handle) & 0xFFFFu;
            uint32_t pending =
                (uint32_t)kalico_runtime_pending_segment_is_some(runtime_handle) & 1u;
            uint32_t deq =
                kalico_runtime_queue_consumer_dequeues_lo(runtime_handle) & 0x7Fu;
            fault_detail = 0xE3000000u | (deq << 17) | (pending << 16) | tc;
            break;
        }
        case 32: {
            // 0xFD — modulated retire attempts (low 12) / successes (low 12).
            // 2026-05-17 retire-branch instrumentation:
            //   - attempts=0 → modulated_tick never enters `elapsed >= duration`
            //     branch; engine clock issue or never-reached.
            //   - attempts > 0, successes = 0 → branch enters but
            //     consumers_done returns false (motor bits aren't being
            //     cleared correctly).
            //   - attempts ≈ successes → retirement working;
            //     credit_freed delivery is the bottleneck.
            uint32_t att = runtime_handle_modulated_retire_attempts(runtime_handle);
            uint32_t suc = runtime_handle_modulated_retire_successes(runtime_handle);
            fault_detail = 0xFD000000u
                         | ((suc & 0xFFFu) << 12)
                         | (att & 0xFFFu);
            break;
        }
        case 37: {
            // 0x9D — durable monotonic bitmap of oids that have entered
            // command_config_stepper. Set by stepper.c at the start of the
            // handler, never cleared. Unlike runtime_diag_last_packed it
            // survives all subsequent runtime_diag_progress writes, so we
            // can definitively confirm whether the F4/H7 dispatcher routes
            // msgid 24 (config_stepper) to command_config_stepper for each
            // expected oid. Low 24 bits = bits 0..23 of the seen bitmap.
            extern volatile uint32_t config_stepper_oids_seen;
            uint32_t seen = config_stepper_oids_seen;
            fault_detail = 0x9D000000u | (seen & 0x00FFFFFFu);
            break;
        }
        // 0xD0/0xD1/0xD2 — spi rx/eot timeout post-mortem (stm32h7_spi.c).
        // Volatiles populated just before the shutdown call; rotation cycles
        // through them so the host sees the full snapshot across 3 frames.
        // H7-only: the externs are defined in src/stm32/stm32h7_spi.c, which
        // F4 builds don't compile (F4 uses a different SPI driver). On F4
        // slot numbers 14/15/16 become no-op rotation gaps, same as the
        // other gaps already present in the switch.
#if CONFIG_MACH_STM32H7
        case 14: {
            // 0xD0 — low 12 bits of SR (status — RXP, TXP, OVR, UDR, MODF,
            // EOT, TIFRE, SUSP), low 12 bits of SPI base address (enough to
            // disambiguate spi1=0x013xxx / spi3=0x003Cxx / spi4=0x013400 etc.)
            extern volatile uint32_t kalico_spi_hang_addr;
            extern volatile uint32_t kalico_spi_hang_sr;
            fault_detail = 0xD0000000u
                         | ((kalico_spi_hang_addr & 0xFFFu) << 12)
                         | (kalico_spi_hang_sr & 0xFFFu);
            break;
        }
        case 15: {
            // 0xD1 — low 16 bits of CR1 (SPE, SSI, CSTART, MASRX, IOLOCK,
            // CRC32, FTHLV, etc.) + reason byte in bits 16..23 (low nibble =
            // remaining-byte count at hang; bit 7 = EOT path vs RX path).
            extern volatile uint32_t kalico_spi_hang_cr1;
            extern volatile uint8_t  kalico_spi_hang_reason;
            fault_detail = 0xD1000000u
                         | (((uint32_t)kalico_spi_hang_reason & 0xFFu) << 16)
                         | (kalico_spi_hang_cr1 & 0xFFFFu);
            break;
        }
        case 16: {
            // 0xD2 — CR2 (TSIZE in bits 0..15) + low 8 bits of CFG2 (CPHA,
            // CPOL, MASTER, SSM, AFCNTR, SSOE). Together they prove whether
            // the peripheral was correctly configured for the txn at hang.
            extern volatile uint32_t kalico_spi_hang_cr2;
            extern volatile uint32_t kalico_spi_hang_cfg2;
            fault_detail = 0xD2000000u
                         | ((kalico_spi_hang_cfg2 & 0xFFu) << 16)
                         | (kalico_spi_hang_cr2 & 0xFFFFu);
            break;
        }
#endif
        }
    }

    // Phase C: replace the legacy `kalico_status_v6` Klipper-protocol output
    // with a native StatusEvent on the events channel. The host bridge maps
    // it back into klippy's RuntimeEvent::Status path.
    //
    // v2 (2026-05-17): tail field `retired_through_segment_id` piggybacks the
    // credit-flow watermark on the periodic status frame. Replaces lossy
    // fire-and-forget kalico_native_emit_credit_freed for slot-pool retirement
    // under USB-CDC TX congestion.
    uint32_t cur_retired_for_status =
        runtime_handle_retired_through_segment_id(runtime_handle);
    kalico_native_emit_status_event(status, depth, cur_seg, last_err,
                                    fault_detail, cur_retired_for_status);

    // Diag emit — DISABLED for wedge-isolation test 2026-05-09. The
    // 5-lines-per-100ms rate was overrunning transmit_buf (320 bytes vs
    // ~600 B/cycle), generating klipper TX drops that may themselves be
    // the wedge trigger. Counters still update in BKPSRAM; read them via
    // prior_diag dump on next boot.
#if 0
    {
        struct diag_snapshot s;
        diag_take_snapshot(&s);
        // Convert DWT cycles → us for human-readable output. H7 is 520 MHz,
        // so 520 cyc/us; on F4 it's 168 or 180. We pass cycles raw since
        // the host knows the clock. Keep one line per logical group so
        // klippy's parser doesn't truncate.
        output("diag_v1 tim5_n %u tim5_max_cyc %u tim5_total_cyc %u"
               " otg_n %u otg_max_cyc %u otg_total_cyc %u",
               s.tim5_n, s.tim5_max, s.tim5_total,
               s.otg_n, s.otg_max, s.otg_total);
        output("diag_v1_tasks out_n %u out_max_gap %u in_n %u in_max_gap %u"
               " drain_n %u drain_max_gap %u stat_n %u stat_max_gap %u"
               " ring_seq %u ring_overflow %u",
               s.usb_out_calls, s.usb_out_max_gap,
               s.usb_in_calls, s.usb_in_max_gap,
               s.runtime_drain_calls, s.runtime_drain_max_gap,
               s.runtime_status_calls, s.runtime_status_max_gap,
               s.ring_seq, s.ring_overflow);
        if (s.tx_drops_kalico || s.tx_drops_klipper) {
            output("diag_v1_drops kalico %u klipper %u",
                   s.tx_drops_kalico, s.tx_drops_klipper);
        }
    }

    // Round 2 — wedge instrumentation. Snapshot OTG live registers and
    // emit a single line capturing per-flag IRQ counts, wake-path
    // counters, and live OTG state. The expected steady-state pattern
    // when bulk-OUT is healthy:
    //   notify_n ≈ task_n ≈ rxflvl_n
    //   read_data_n grows with notify_n (host → MCU bytes flowing)
    //   read_zero_n stays low
    //   gintmsk has RXFLVLM bit (0x10) set unless an IRQ just fired
    //   gintsts.RXFLVL (0x10) clears once foreground services it
    // The wedge signature we're trying to catch:
    //   notify_n grows but task_n stagnates → sched-side issue
    //   task_n grows but read_data_n stays flat → EP returns no data
    //   gintmsk has RXFLVLM bit CLEARED for >1 emit cycle → never re-armed
#if CONFIG_MACH_STM32H7 || CONFIG_MACH_STM32F4 || CONFIG_MACH_STM32F7
    {
        extern void usb_diag_read_otg_state(uint32_t *, uint32_t *);
        extern void usb_diag_read_out_ep(uint32_t *, uint32_t *, uint32_t *);
        uint32_t gintmsk = 0, gintsts = 0;
        uint32_t doepctl = 0, doeptsiz = 0, doepint = 0;
        usb_diag_read_otg_state(&gintmsk, &gintsts);
        usb_diag_read_out_ep(&doepctl, &doeptsiz, &doepint);
        diag_snapshot_otg_regs(gintmsk, gintsts);
        diag_snapshot_out_ep(doepctl, doeptsiz, doepint);
        output("diag_v1_otg rxflvl_n %u iepint_n %u other_n %u other_sts %u"
               " notify_n %u task_n %u read_data %u read_zero %u"
               " gintmsk %u gintsts %u",
               diag_get_otg_rxflvl(),
               diag_get_otg_iepint(),
               diag_get_otg_other(),
               diag_get_otg_other_sts(),
               diag_get_notify_bulk_out(),
               diag_get_task_invoke(),
               diag_get_read_data(),
               diag_get_read_zero(),
               gintmsk, gintsts);
        // Round 3 — OUT EP register snapshot + enable_rx + peek
        // counters. This emits one extra ~150 byte line per 100 ms
        // (1.5 KB/s extra wire load).
        // doepctl bits of interest:
        //   0x80000000 EPENA — EP enabled to receive
        //   0x00020000 NAKSTS — EP NAKing (sticky)
        //   0x00010000 STALL — EP stalling
        //   0x00008000 USBAEP — EP active in this configuration
        // doeptsiz bits of interest:
        //   bits 30..29 PKTCNT — packets remaining to receive
        //   bits 18..0  XFRSIZ — bytes remaining to receive
        // doepint bits of interest:
        //   bit 0 XFRC — transfer completed
        //   bit 1 EPDISD — EP disabled
        //   bit 3 STUP — setup phase done (only EP0)
        output("diag_v1_ep doepctl %u doeptsiz %u doepint %u"
               " enable_rx_n %u rearmed_n %u peek_data %u peek_empty %u",
               diag_get_out_ep_doepctl(),
               diag_get_out_ep_doeptsiz(),
               diag_get_out_ep_doepint(),
               diag_get_enable_rx_n(),
               diag_get_enable_rx_rearm(),
               diag_get_peek_data(),
               diag_get_peek_empty());
    }
#endif
#endif // 0 — diag emit disabled

#if defined(__linux__) || defined(__APPLE__)
    // Sim-only: dump stepper counters so a test that lost its klippy
    // bridge_call link can still observe motion progress via the elf log.
    // Phase 4 test polls this to confirm GATE GREEN.
    int32_t c0 = kalico_runtime_get_stepper_count(runtime_handle, 0);
    int32_t c1 = kalico_runtime_get_stepper_count(runtime_handle, 1);
    int32_t c2 = kalico_runtime_get_stepper_count(runtime_handle, 2);
    fprintf(stderr,
        "[sim-progress] status=%u seg=%u counts=[%d,%d,%d]\n",
        status, cur_seg, c0, c1, c2);
    fflush(stderr);
#endif
}
DECL_TASK(runtime_status_drain);

// DECL_COMMAND surface — test harness loads curves and pushes segments.
//
// Klipper's %*s blob format consumes TWO args slots per blob: a length
// followed by an encoded pointer that must be reconstituted via
// `command_decode_ptr` (declared in command.h). See src/i2ccmds.c and
// src/spicmds.c for canonical usage. Each f32 scalar control point is
// 4 bytes; each knot is a single f32 (4 bytes). We derive `n_cp`,
// `n_knots` from the blob byte-lengths and validate before calling
// into Rust.
// Cubic-only revision (2026-05-20 stepping redesign): the
// `runtime_aligned_cps` / `runtime_aligned_knots` scratch buffers that backed
// the legacy NURBS LoadCurve path were removed along with the NURBS upload
// command. Cubic-piece uploads (LoadCurveCubic, kalico_dispatch.c) carry
// fixed-stride 20-byte monomial pieces and do not need pre-aligned host-side
// scratch.


// Command surface (query_status, arm_endstop, disarm_endstop,
// configure_axes, stream_*, clock_sync_request, query_pool_state)
// plus the endstop GPIO sampler hot-path
// (`runtime_endstop_sample_pins` + `endstop_pin_table`) live in
// src/runtime_commands.c. This file keeps only lifecycle (init/drain),
// sibling drains (status_drain, endstop_drain), and shared globals.

DECL_CTR("_DECL_OUTPUT "
         "kalico_endstop_tripped arm_id=%u "
         "trip_clock_lo=%u trip_clock_hi=%u "
         "trip_source_idx=%c fmt_version=%c "
         "stepper_count=%c stepper_data=%*s");

// Periodic task: drain runtime-side trip events into async msgproto
// `kalico_endstop_tripped` outputs. Modeled on `runtime_status_drain` —
// runs at task cadence. Trips are infrequent (one per homing); the
// in-buffer max length matches kalico-c-api's KALICO_TRIP_EVENT_V1_MAX_LEN
// (15 header + 8 steppers * 5 = 55 bytes).
void
runtime_endstop_drain(void)
{
    if (!runtime_handle) return;
    uint8_t buf[64];
    size_t actual = 0;
    int32_t r = kalico_endstop_poll_trip(buf, sizeof(buf), &actual);
    if (r != 1 || actual < 15) return;
    uint32_t arm_id     = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8)
                        | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
    uint32_t clock_lo   = (uint32_t)buf[4] | ((uint32_t)buf[5] << 8)
                        | ((uint32_t)buf[6] << 16) | ((uint32_t)buf[7] << 24);
    uint32_t clock_hi   = (uint32_t)buf[8] | ((uint32_t)buf[9] << 8)
                        | ((uint32_t)buf[10] << 16) | ((uint32_t)buf[11] << 24);
    uint8_t source_idx  = buf[12];
    uint8_t fmt_version = buf[13];
    uint8_t stepper_n   = buf[14];
    uint32_t blob_len   = (uint32_t)stepper_n * 5;
    if (15 + blob_len > actual) return;
    output("kalico_endstop_tripped arm_id=%u "
           "trip_clock_lo=%u trip_clock_hi=%u "
           "trip_source_idx=%c fmt_version=%c "
           "stepper_count=%c stepper_data=%*s",
           arm_id, clock_lo, clock_hi,
           source_idx, fmt_version,
           stepper_n, blob_len, &buf[15]);
}
DECL_TASK(runtime_endstop_drain);

// ---- Step-6 §5/§9 async event channel declarations ---------------------
// `kalico_credit_freed` and `kalico_fault` are MCU-emitted async events
// (no DECL_COMMAND on the host-to-MCU side). The Klipper `output(FMT, ...)`
// macro at call sites already registers each format string into the data
// dictionary via `_DECL_OUTPUT` / `DECL_CTR`, so a static `DECL_CTR` here
// is the equivalent of pre-registering the schema before the first emit.
// The actual emits live in the foreground drain pipeline (Phase 11) and the
// fault-publish path (Phase 4 / Phase 11).
DECL_CTR("_DECL_OUTPUT "
         "kalico_trace count=%u data=%*s");
// kalico_credit_freed / kalico_fault / kalico_status_v6 retired Phase C —
// they now ride the kalico-native events channel via
// kalico_native_emit_credit_freed / _fault_event / _status_event in
// src/kalico_dispatch.c.
// Sim-only commands and the sim drain-wake hook live in
// src/runtime_sim_commands.c (CONFIG_KALICO_SIM). Spec §4.5.

// Cycle-count bench command + storage moved to src/generic/runtime_bench.c
// (selected by CONFIG_RUNTIME_BENCH). The H7 ISR calls the unconditional
// `runtime_bench_capture` hook; the weak fallback in
// src/runtime_tick_weak.c resolves when bench is disabled.

// Forward decl: defined in src/stepper.c.
extern void runtime_emit_step_pulses(uint8_t motor_idx, int32_t n_steps);

// ===========================================================================
// Per-axis step timer consumers (stepping-redesign Task 10)
// ===========================================================================
//
// One Klipper timer per axis (X=0, Y=1, Z=2, E=3). Each timer's `func`
// calls into the Rust body `kalico_per_axis_step_event`, which pops one
// StepEntry from `step_queues[axis_idx]` if its `cycle_abs` has arrived,
// emits the GPIO pulses via `runtime_emit_step_pulses`, and returns the
// next waketime. Wired into `command_kalico_configure_axis` (stepper.c)
// via `init_per_axis_step_timers`.

extern uint32_t kalico_per_axis_step_event(uint8_t axis_idx);

// Per-axis timers (4 axes). The `func` slot is dispatched by Klipper's
// SysTick scheduler; each trampoline below binds a literal axis_idx that
// the Rust body uses to project to `step_queues[axis_idx]`.
static struct timer per_axis_timers[4];

static uint_fast8_t per_axis_timer_event_0(struct timer *t) {
    t->waketime = kalico_per_axis_step_event(0);
    return SF_RESCHEDULE;
}
static uint_fast8_t per_axis_timer_event_1(struct timer *t) {
    t->waketime = kalico_per_axis_step_event(1);
    return SF_RESCHEDULE;
}
static uint_fast8_t per_axis_timer_event_2(struct timer *t) {
    t->waketime = kalico_per_axis_step_event(2);
    return SF_RESCHEDULE;
}
static uint_fast8_t per_axis_timer_event_3(struct timer *t) {
    t->waketime = kalico_per_axis_step_event(3);
    return SF_RESCHEDULE;
}

static uint_fast8_t (*const per_axis_handlers[4])(struct timer *) = {
    per_axis_timer_event_0,
    per_axis_timer_event_1,
    per_axis_timer_event_2,
    per_axis_timer_event_3,
};

// Install the four per-axis timers. Called once per boot from
// `command_kalico_configure_axis` (stepper.c) via the static-flag guard.
// Not idempotent — caller must ensure only one invocation per boot.
//
// `runtime_emit_step_pulses` is defined in src/stepper.c. The Rust body
// resolves the C-declared `step_queues` array internally; this file owns
// the trampolines + scheduler wiring.
void
init_per_axis_step_timers(void)
{
    uint32_t now = timer_read_time();
    for (int i = 0; i < 4; i++) {
        per_axis_timers[i].func = per_axis_handlers[i];
        // 1 ms boot delay so the first dispatch lands strictly in the
        // future (sched_add_timer trips "Timer too close" on a past
        // waketime). Subsequent waketimes come from
        // kalico_per_axis_step_event's return value.
        per_axis_timers[i].waketime = now + timer_from_us(1000);
        sched_add_timer(&per_axis_timers[i]);
    }
}

// === Task 14: SPI queue foreground drain (stub) ===
//
// The TIM5 ISR pushes SpiWrite entries into spi_queues[bus_idx] from
// dispatch_phase. A foreground struct timer firing at ~10 kHz should
// pop from each bus's queue and dispatch through Klipper's spidev /
// bus.c. For now: stub timer that clears the queue without writing to
// hardware — keeps the SPSC contract live without committing to a
// specific bus driver. Bench bring-up (Stage 5) wires real SPI.
//
// `init_spi_drain_timer` is publicly exposed but has no production
// caller yet; Task 14's scope is the queue + ISR push only. Once the
// real SPI bring-up lands, this hook will be invoked alongside
// `init_per_axis_step_timers` from `command_kalico_configure_axis`.

#include "spi_queue.h"

static struct timer spi_drain_timer;

static uint_fast8_t
spi_drain_event(struct timer *t)
{
    for (int bus = 0; bus < N_SPI_BUSES; bus++) {
        SpiQueue *q = &spi_queues[bus];
        while (q->head != q->tail) {
            // Stub: drop the entry without dispatching to hardware.
            // Bench Stage 5 replaces this with: spidev_write(...).
            q->head = (uint16_t)(q->head + 1);
        }
    }
    t->waketime = timer_read_time() + timer_from_us(100);  // 10 kHz
    return SF_RESCHEDULE;
}

void
init_spi_drain_timer(void)
{
    spi_drain_timer.func = spi_drain_event;
    spi_drain_timer.waketime = timer_read_time() + timer_from_us(1000);
    sched_add_timer(&spi_drain_timer);
}
