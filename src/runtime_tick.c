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

// F446 configure_axes crash diagnostic (2026-05-11). Emits a tagged
// `#output` line so the host log shows how far the FFI got before
// crashing. `output(...)` queues to the USB-CDC TX buffer; even if
// the chip resets shortly after, klippy reads the buffered line
// before the disconnect. `stage` is a small u8 with stage IDs defined
// at the Rust call site (runtime_ffi.rs). Foreground-only.
__attribute__((used, externally_visible))
void
runtime_diag_progress(uint32_t tag, uint32_t stage, uint32_t value)
{
    output("rt_diag tag=%u stage=%u value=%u", tag, stage, value);
}

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

#endif // CONFIG_KALICO_RUNTIME
