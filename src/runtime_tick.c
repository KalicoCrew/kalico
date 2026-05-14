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

#if CONFIG_KALICO_RUNTIME

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

// Minimum scheduling-into-future margin for SF_RESCHEDULE callbacks and
// for sched_add_timer waketimes computed relative to "now". Klipper's
// scheduler (src/sched.c sched_add_timer) trips `try_shutdown("Timer too
// close")` if a freshly-added timer's waketime is already < the current
// `timer_read_time()` by the time the insert check runs — and any value
// sampled before the irq_save in sched_add_timer is essentially guaranteed
// to be in the past a few cycles later. Always add this floor when a
// callback wants "ASAP but in the future."
//
// 1 µs at 520 MHz (H7) = 520 cycles; at 84 MHz (F4) = 84 cycles. Both are
// comfortably above any per-call drift between sampling `timer_read_time()`
// and the scheduler's bounds check.
// Minimum scheduling-into-future margin for SF_RESCHEDULE callbacks.
// Klipper's `armcm_timer.c:152` shuts down with "Rescheduled timer in
// the past" when a timer's waketime is >1 ms behind `now` AND the
// dispatch loop has been running tight (TIMER_REPEAT_TICKS = 100 µs).
// A 1 µs floor used to be enough to satisfy the "strictly in the
// future" requirement, but combined with the consumer's catch-up
// emit loop (which fires step pulses as fast as possible when step
// times are in the past) it pegged the dispatch loop at >1 MHz
// per-consumer, starving other timers until one drifted past the 1 ms
// limit and tripped the shutdown.
//
// 10 µs caps the worst-case catch-up emit rate at 100 kHz per motor
// (400 kHz aggregate across 4 motors), leaves ~50% of the dispatch
// loop budget for other timers (USB, status drain, drain task), and
// is still 2× faster than realistic TMC2240 step input rates
// (~250 kHz datasheet max).
#define SF_RESCHEDULE_FLOOR (runtime_clock_freq / 100000U)  // 10 µs

// Empty-poll cadence for the consumer when its ring has no entries.
// Independent of SF_RESCHEDULE_FLOOR — the consumer's "no work, sleep"
// path runs at 1 kHz, leaving most of the dispatch budget to the
// producer (which is the actually-loaded timer when the consumer is
// empty). The producer kicks the consumer indirectly by filling its
// ring; the consumer notices on its next 1 ms poll. 1 ms of first-
// step latency after a segment push is invisible at the bench.
// TEMPORARY DIAGNOSTIC: 100 ms empty-poll cadence. If empty_polls counter
// rate remains >40 Hz aggregate after this, our t->waketime isn't being
// respected by Klipper's scheduler and the dispatch-loop saturation has
// a different root cause than the consumer's reschedule cadence.
#define EMPTY_POLL_CYCLES (runtime_clock_freq / 10U)  // 100 ms

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
    t->waketime += timer_from_us(1000);  // 1 kHz
    return SF_RESCHEDULE;
}

void
runtime_init(void)
{
    // Capture prior-run cause-of-death from .persistent_diag BEFORE any
    // current-run runtime_diag_progress overwrites it. Status drain emits
    // this snapshot on every Nth status frame so klippy.log preserves it.
    // Also stash the magic + raw packed values into a SEPARATE static so
    // we can verify .persistent_diag actually survives (separately from
    // status drain emit logic).
    extern volatile uint32_t runtime_diag_prior_magic_raw;
    extern volatile uint32_t runtime_diag_prior_packed_raw;
    runtime_diag_prior_magic_raw = rt_diag_persistent.magic;
    runtime_diag_prior_packed_raw = rt_diag_persistent.last_packed;
    if (rt_diag_persistent.magic == RT_DIAG_MAGIC
        && rt_diag_persistent.last_packed != 0) {
        runtime_diag_prior_boot_snapshot = rt_diag_persistent.last_packed;
    }
    runtime_handle = runtime_handle_create();
    if (!runtime_handle) {
        // Init failed — leave liveness flag at default (1 = OK) but handle unset;
        // calls into the runtime will short-circuit safely.
        return;
    }
    last_seen_tick_counter = runtime_handle_tick_counter(runtime_handle);
    last_progress_time = timer_read_time();
    last_seen_status = runtime_handle_status(runtime_handle);

    // Initialize the modulation tick driver. On STM32H7 this configures
    // TIM5 (DOES NOT enable; the first segment push triggers enable via
    // the producer protocol §4.4). On Linux it spawns the host pthread
    // that calls runtime_handle_tick at 40 kHz.
    runtime_tick_init();

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

    // §10.4: emit one `kalico_credit_freed` async event per drain cycle that
    // observed at least one SEGMENT_END. The host uses this to bump its
    // credit counter; it doesn't need one event per retired segment, just
    // a wake-up to re-read the cursors. `retired_through_segment_id` carries
    // the cumulative cursor; `free_slots = Q_N - queue_depth` (with Q_N - 1
    // being the structural cap; saturate at u8 in the Rust accessor).
    if (saw_segment_end) {
        uint32_t retired = runtime_handle_retired_through_segment_id(runtime_handle);
        uint8_t depth = runtime_handle_queue_depth(runtime_handle);
        uint8_t free_slots = (depth >= 7) ? 0 : (uint8_t)(7 - depth);
        // Phase C: emit as kalico-native CreditFreed event (channel 1).
        kalico_native_emit_credit_freed(retired, free_slots);
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
            // Tag 0xE2 marker in high byte; mid byte = emit_pulses & 0xFF
            // (rough), low byte = packed motor_stepper_count for motors 0..3.
            {
                uint32_t pulses_lo = runtime_emit_pulses & 0xFFFFu;
                uint32_t cnt0 = runtime_motor_binding_count(0) & 0x3u;
                uint32_t cnt1 = runtime_motor_binding_count(1) & 0x3u;
                uint32_t cnt2 = runtime_motor_binding_count(2) & 0x3u;
                uint32_t cnt3 = runtime_motor_binding_count(3) & 0x3u;
                uint32_t cnts = (cnt0 << 0) | (cnt1 << 2)
                              | (cnt2 << 4) | (cnt3 << 6);
                fault_detail = 0xE2000000u | (cnts << 16) | pulses_lo;
            }
            break;
        }
    }
    // Re-roll the rotation for the four new step_time_event-side counters,
    // gated on the same `last_err == 0`. Cycle resets after these so each
    // counter gets one observation per ~600 ms at 10 Hz status drain.
    static uint8_t st_emit_phase;
    st_emit_phase = (uint8_t)(st_emit_phase + 1);
    if (st_emit_phase >= 4) st_emit_phase = 0;
    extern volatile uint32_t step_time_event_fires;
    extern volatile uint32_t step_time_producer_kicks;
    extern volatile uint32_t step_time_empty_polls;
    extern uint8_t runtime_motor_binding_count(uint8_t motor_idx);
    if (last_err == 0 && status_emit_phase == 0) {
        switch (st_emit_phase) {
        case 0:
            // 0xE3 — step_time_event fires (low 24 bits).
            fault_detail = 0xE3000000u | (step_time_event_fires & 0x00FFFFFFu);
            break;
        case 1:
            // 0xE4 — producer kicks (low 24 bits) — how often the
            // consumer / push_segment hook actually CAS-won and queued
            // the producer timer.
            fault_detail = 0xE4000000u
                         | (step_time_producer_kicks & 0x00FFFFFFu);
            break;
        case 2:
            // 0xE5 — empty polls (low 24 bits) — how often the consumer
            // found its ring empty and short-polled. High = producer
            // falling behind.
            fault_detail = 0xE5000000u | (step_time_empty_polls & 0x00FFFFFFu);
            break;
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
        }
    }

    // Phase C: replace the legacy `kalico_status_v6` Klipper-protocol output
    // with a native StatusEvent on the events channel. The host bridge maps
    // it back into klippy's RuntimeEvent::Status path.
    kalico_native_emit_status_event(status, depth, cur_seg, last_err, fault_detail);

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
// Aligned scratch buffers for the load_curve handler. Klipper's RX buffer
// places the %*s payload at an arbitrary byte offset (typically not 4-byte
// aligned), so passing those pointers directly to Rust yields an unaligned
// `&[f32]` — UB on construction even though Cortex-M7 happens to allow
// unaligned word reads at the CPU level. Empirically this hardfaults the
// MCU and triggers a USB renumerate. Copy into 4-byte-aligned static
// buffers first, then pass to Rust.
//
// Sizing matches CurvePool's compile-time bounds (MAX_CONTROL_POINTS = 1830,
// MAX_KNOT_VECTOR_LEN = 1850). Bumped per Phase C of the kalico-native
// transport spec
// (docs/superpowers/specs/2026-05-04-kalico-native-transport-design.md §10):
// H723 X+Y heavy-shaping worst case is degree 9, ~200 pieces over 100 mm,
// ~1810 cps and ~1820 knots. F446 will get a dedicated build with smaller
// constants in Phase D — for now the unified Linux-sim / H723 build picks
// the larger sizing.
// Non-static so kalico_dispatch.c's LoadCurve handler can reuse the same
// scratch (the legacy DECL_COMMAND begin/chunk/finalize path is retired).
float runtime_aligned_cps[CONFIG_RUNTIME_MAX_CONTROL_POINTS];
float runtime_aligned_knots[CONFIG_RUNTIME_MAX_KNOT_VECTOR_LEN];

// Phase C of the kalico-native transport spec
// (`docs/superpowers/specs/2026-05-04-kalico-native-transport-design.md` §15)
// retires the legacy begin/chunk/finalize command surface and the
// kalico_push_segment command. Curve uploads and segment pushes now arrive
// as native kalico frames; see src/kalico_dispatch.c handlers.


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

// ---------------------------------------------------------------------------
// Per-stepper step-time scheduling — step-emission architecture (spec §3.4 / §3.5).
//
// The host-side `Engine::producer_step` Newton-fills per-motor `StepRing`
// buffers with (cycles_abs_lo, dir) entries. Each StepTime-mode motor gets
// its own Klipper `struct timer` (the "consumer") that pops one entry per
// fire, drives the step+dir pins, samples endstops, and reschedules at the
// next entry's time. A single shared producer Klipper timer refills the
// rings on demand, kicked by `push_segment`'s producer-pending CAS or by
// the consumer's low-water hook.
//
// Step-pulse discipline — runtime_emit_step_pulses (complete rise+fall pair):
//   Each step_time_event issues one call to runtime_emit_step_pulses with
//   n_steps=±1, which drives a guaranteed rising edge + step_pulse_ticks dwell
//   + falling edge on every AWD partner's step pin (e.g. stepper_z / z1 / z2
//   for a 3-motor Z). The dir_pin is updated (with dwell) whenever direction
//   changes. This is the shared modulation path — no separate toggle logic
//   needed in the step-time ISR.
//
// MAX_STEPPER_OIDS_C must agree with Rust's MAX_STEPPER_OIDS in
// rust/runtime/src/state.rs (currently 8). A static_assert on the C side
// can't cross the FFI boundary, so we rely on code review and the comment.
#define MAX_STEPPER_OIDS_C 8   // must match rust/runtime/src/state.rs::MAX_STEPPER_OIDS

// Low-water threshold: spec §3.4 calls for kicking the producer when the
// step-ring drains below N/4 of its capacity. With Rust's
// `StepRing::CAPACITY == 1024` that's 256 entries.
#define STEP_RING_LOW_WATER 256

// Forward decl: defined in src/stepper.c. -Wimplicit-function-declaration is
// promoted to error under the sim build's stricter flags, so a header-less
// extern is required here.
extern void runtime_emit_step_pulses(uint8_t motor_idx, int32_t n_steps);

struct step_timer_ctx {
    struct timer timer;
    uint8_t stepper_idx; // 0-based engine stepper index
    uint8_t enabled;     // 1 = registered with scheduler, 0 = idle
};

static struct step_timer_ctx step_timers[MAX_STEPPER_OIDS_C];

// Single shared producer Klipper timer. Refills every StepTime motor's
// step-ring in a single pass via `kalico_runtime_producer_step`. The
// `enabled` byte mirrors `producer_pending`: set when a kicker (push_segment
// CAS or consumer low-water) has scheduled the timer, cleared when
// `runtime_producer_event` returns SF_DONE.
static struct {
    struct timer timer;
    uint8_t enabled;
} runtime_producer_timer;

// Diag counter: number of step_time_event ISR fires. Surfaced by the 10 Hz
// status drain via the 0xE3 fault_detail tag. Volatile because the ISR
// writes and the foreground task reads.
volatile uint32_t step_time_event_fires __attribute__((used, externally_visible));

// Diag counter: number of producer kicks (CAS-won) — when the consumer or
// `kalico_runtime_kick_producer` callers reschedule the producer timer.
// Surfaced via the 0xE4 fault_detail tag.
volatile uint32_t step_time_producer_kicks __attribute__((used, externally_visible));

// Diag counter: number of times step_time_event found an empty ring and
// fell back to a short-poll reschedule. Surfaced via the 0xE5 fault_detail
// tag. High counts indicate the producer is failing to keep up.
volatile uint32_t step_time_empty_polls __attribute__((used, externally_visible));

// Defined in src/runtime_commands.c (Task D3). Samples all active endstop
// GPIO slots for the given stepper index. Called from step_time_event so
// the step-time ISR path catches trips at step resolution.
extern void runtime_endstop_sample_one(uint8_t stepper_idx);

// Forward decl for the producer timer; defined below. Used by the consumer
// low-water hook and by `arm_producer_timer_if_kicked` (called from
// handle_push_segment in src/kalico_dispatch.c).
static uint_fast8_t runtime_producer_event(struct timer *t);

// Helper: if the runtime says the producer should run (CAS-won), make sure
// the producer Klipper timer is queued. Idempotent — the `enabled` flag
// guards against double-add. Safe to call from foreground (push_segment)
// or ISR (step_time_event) contexts.
static void
arm_producer_timer_if_kicked_inline(uint32_t waketime)
{
    if (!runtime_handle) return;
    if (!kalico_runtime_kick_producer(runtime_handle)) {
        // Another caller already won the CAS. Either the producer timer
        // is already queued, or the previously-pending run is in flight.
        // Either way, no new schedule is required.
        return;
    }
    step_time_producer_kicks++;
    if (runtime_producer_timer.enabled) {
        // Race: the timer was queued by an earlier kick whose
        // `runtime_producer_event` hasn't run yet (so `enabled` is still
        // set) AND it cleared `producer_pending` to false before we
        // CAS-set it back to true. The currently-queued run will observe
        // our new pending bit via `runtime_handle.shared` and process
        // accordingly, so no additional schedule is needed.
        return;
    }
    runtime_producer_timer.enabled = 1;
    // sched_add_timer trips `try_shutdown("Timer too close")` if the
    // waketime is already behind `timer_read_time()` by the time the
    // irq-save-protected bounds check runs. Callers pass "now-ish" values
    // (the result of an earlier `timer_read_time()`); enforce the floor
    // here so every entry into sched_add_timer is strictly in the future.
    uint32_t now_arm = timer_read_time();
    uint32_t floor_arm = now_arm + SF_RESCHEDULE_FLOOR;
    runtime_producer_timer.timer.waketime =
        ((int32_t)(waketime - floor_arm) < 0) ? floor_arm : waketime;
    sched_add_timer(&runtime_producer_timer.timer);
}

// Called from handle_push_segment in src/kalico_dispatch.c after
// runtime_handle_push_segment returns KALICO_OK. Replaces the previous
// `arm_step_time_steppers_after_push` per-stepper arming loop — the
// producer fills the rings and the per-stepper consumer timers (registered
// once at configure_axes time) drain them.
void
arm_producer_timer_if_kicked(void)
{
    arm_producer_timer_if_kicked_inline(timer_read_time());
}

// Per-stepper consumer ISR. Called by Klipper's scheduler at the
// `cycles_abs_lo` time of the next ring entry, or on a short-poll
// cadence when the ring is empty. Pops one entry per fire and emits one
// step pulse on this motor.
//
// Signature must match `uint_fast8_t (*func)(struct timer*)` — sched.h §14.
static uint_fast8_t
step_time_event(struct timer *t)
{
    step_time_event_fires++;
    struct step_timer_ctx *ctx =
        container_of(t, struct step_timer_ctx, timer);
    uint8_t motor = ctx->stepper_idx;

    uint32_t t_next = 0;
    int8_t dir = 1;
    bool have_head = kalico_runtime_step_ring_peek_head(
        runtime_handle, motor, &t_next, &dir);

    if (!have_head) {
        // Ring empty — the producer hasn't caught up yet (or there's no
        // active segment for this motor). Short-poll until the producer
        // refills. The consumer's low-water hook below kicks the producer
        // when AVAILABLE drops; here we additionally kick on full-empty
        // to handle the bootstrap case (timer queued by configure_axes
        // before the first segment).
        step_time_empty_polls++;
        arm_producer_timer_if_kicked_inline(timer_read_time());
        // Now-relative reschedule (NOT `+= 100 µs` from the prior waketime):
        // if the consumer fell behind for any reason, the `+=` form keeps
        // accumulating from a stale base and can re-schedule in the past on
        // the next iteration. Anchoring to `timer_read_time()` guarantees
        // the next fire is always 100 µs into the actual future.
        t->waketime = timer_read_time() + EMPTY_POLL_CYCLES;
        return SF_RESCHEDULE;
    }

    uint32_t now = timer_read_time();
    if ((int32_t)(t_next - now) > 0) {
        // Head entry is in the future — schedule the next wake at that
        // time. No emit, no advance. The scheduler will wake us at the
        // exact step time. Clamp to a minimum-future-floor in case the
        // entry is only a handful of cycles ahead (Klipper's
        // sched_add_timer-style "Timer too close" check expects strictly
        // > now after the irq_save races a few cycles).
        uint32_t floor = now + SF_RESCHEDULE_FLOOR;
        t->waketime = ((int32_t)(t_next - floor) < 0) ? floor : t_next;
        return SF_RESCHEDULE;
    }

    // Head entry is at or past `now` — emit one step pulse. The shared
    // runtime_emit_step_pulses path handles AWD partners (e.g.
    // stepper_z / z1 / z2 for a 3-motor Z), dir-pin updates with the
    // correct polarity, and the dir-setup dwell before the step edge.
    int32_t n_steps = (dir >= 0) ? 1 : -1;
    runtime_emit_step_pulses(motor, n_steps);

    // Commit the just-emitted step into `shared.stepper_counts` so the
    // engine's step counters track the consumer's progress (Modulated-mode
    // parity, and for any host-side step-position queries).
    kalico_runtime_apply_step(runtime_handle, motor, n_steps);

    // Sample endstops armed on this motor's axis at step resolution.
    runtime_endstop_sample_one(motor);

    // Advance the consumer cursor past this entry.
    kalico_runtime_step_ring_advance(runtime_handle, motor, 1);

    // Low-water hook: kick the producer if this motor's ring drained
    // below N/4. The kicker is a CAS, so multiple consumers calling this
    // simultaneously coalesce into a single producer wake.
    if (kalico_runtime_step_ring_available(runtime_handle, motor)
            < STEP_RING_LOW_WATER) {
        arm_producer_timer_if_kicked_inline(now);
    }

    // Reschedule for the next ring entry, or short-poll if drained.
    uint32_t t_next2 = 0;
    int8_t  dir2 = 1;
    if (kalico_runtime_step_ring_peek_next(
            runtime_handle, motor, &t_next2, &dir2)) {
        // The producer may have queued entries whose scheduled time is
        // already in the past (e.g. consumer catching up after a hiccup).
        // Clamp to a minimum-future-floor so Klipper's scheduler doesn't
        // see a waketime that races behind `timer_read_time()` between
        // here and the re-insert.
        uint32_t now2 = timer_read_time();
        uint32_t floor2 = now2 + SF_RESCHEDULE_FLOOR;
        t->waketime = ((int32_t)(t_next2 - floor2) < 0) ? floor2 : t_next2;
    } else {
        t->waketime = timer_read_time() + EMPTY_POLL_CYCLES;
    }
    return SF_RESCHEDULE;
}

// Producer Klipper timer callback. Runs one `Engine::producer_step` pass
// (Newton-fills the per-motor step rings up to PRODUCER_BATCH_CAP each),
// then either self-reschedules at `now` (more work pending) or marks
// itself disabled and exits (every StepTime motor reached AllIdle). The
// next push_segment / consumer low-water kick will re-arm.
static uint_fast8_t
runtime_producer_event(struct timer *t)
{
    bool work_pending = kalico_runtime_producer_step(runtime_handle);
    if (work_pending) {
        // Self-reschedule ASAP for the next batch.
        t->waketime = timer_read_time() + SF_RESCHEDULE_FLOOR;
        return SF_RESCHEDULE;
    }
    // No work — slow heartbeat. We CANNOT return SF_DONE here: that
    // races with concurrent `arm_producer_timer_if_kicked_inline` calls
    // from the SysTick-priority consumer ISR. Sequence (race):
    //   1. Producer sets `enabled = 0` and prepares to return SF_DONE.
    //   2. SysTick preempts; a consumer's empty-poll calls the kick
    //      helper. It CAS-wins `producer_pending`, sees `enabled == 0`,
    //      and `sched_add_timer`s the producer timer.
    //   3. Producer resumes, returns SF_DONE — Klipper attempts to
    //      remove the timer from its priority queue, but the consumer
    //      already re-added it. The queue is left in a corrupted state
    //      where a stale timer entry has a waketime far in the past;
    //      Klipper's `armcm_timer.c:152` eventually trips
    //      "Rescheduled timer in the past" on that stale entry.
    //
    // Fix: always SF_RESCHEDULE. Set a 1 ms idle cadence so the producer
    // runs ~1 kHz when nothing's happening (negligible CPU), and any
    // kick that lands between fires is observed on the next call (kicks
    // set `producer_pending` in shared state; the next
    // `kalico_runtime_producer_step` body sees the work even if it didn't
    // change scheduling state). `enabled` stays `1` for the lifetime of
    // the timer's residency in the scheduler queue — set once at the
    // first kick after `init_step_time_timers`, never cleared.
    t->waketime = timer_read_time() + runtime_clock_freq / 1000U;  // +1 ms
    return SF_RESCHEDULE;
}

// Called from handle_configure_axes in src/kalico_dispatch.c after
// kalico_runtime_configure_axes_blob succeeds. Registers each StepTime
// motor's consumer Klipper timer with the scheduler (one short-poll wake
// to bootstrap; the first poll will find the ring empty, kick the
// producer, and switch to ring-driven scheduling once entries arrive)
// and prepares the shared producer timer (not added to the scheduler
// yet — push_segment's kick will queue it).
void
init_step_time_timers(void)
{
    if (!runtime_handle) return;

    uint32_t now = timer_read_time();
    uint32_t boot_poll = EMPTY_POLL_CYCLES;

    for (uint8_t i = 0; i < MAX_STEPPER_OIDS_C; i++) {
        // Reset state. Note: if a consumer timer is already enabled from
        // a prior configure_axes call within the same boot, leave it
        // running — the scheduler-side `struct timer` is opaque and
        // mutating `func` while it's queued is unsafe. ConfigureAxes is
        // a one-shot per print job in normal operation, so this guard
        // primarily defends against repeated calls during host-side
        // bring-up scripts.
        if (step_timers[i].enabled) continue;
        step_timers[i].timer.func = step_time_event;
        step_timers[i].stepper_idx = i;

        // Only register StepTime-mode motors (discriminant = 1). Modulated
        // motors are driven by the TIM5 ISR; their consumer timer slot
        // stays unregistered.
        uint8_t mode = kalico_runtime_get_step_mode(runtime_handle, i);
        if (mode != 1 /* StepMode::StepTime */) continue;

        step_timers[i].enabled = 1;
        step_timers[i].timer.waketime = now + boot_poll;
        sched_add_timer(&step_timers[i].timer);
    }

    // Set up the producer timer (don't queue it yet — push_segment kicks).
    runtime_producer_timer.timer.func = runtime_producer_event;
    runtime_producer_timer.enabled = 0;
}

#endif // CONFIG_KALICO_RUNTIME
