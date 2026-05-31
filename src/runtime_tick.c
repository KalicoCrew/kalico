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
// Host build: pthread-driven tick replaces the TIM5 ISR. The only Rust→C
// call across this boundary now is runtime_cyccnt_read; runtime_tick_enable
// and runtime_tick_disable are called from C (configure_axis / the
// DECL_SHUTDOWN handler), not from Rust. Host-side implementations live in
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

// Boot/dispatch progress diagnostic (2026-05-11). Packs the latest
// (tag, stage, value) triple into a single u32 word that
// `runtime_status_drain` piggybacks onto the periodic `kalico_status_v6`
// frame's `fault_detail` field when no real fault is latched.
//
// Why not `output(...)` directly: kalico-native dispatch context (FFI
// handlers reached via the kalico-native demux) blocks the foreground
// task that drains the USB-CDC TX buffer until the handler returns.
// On F446, the kalico-native FFI handler crashes BEFORE that return, so any
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

    // Initialize the modulation tick driver. On STM32H7 this configures AND
    // arms TIM5 — it free-runs from boot now, with no arm gate (the old
    // "first segment push enables via the producer protocol" path is gone).
    // On Linux it spawns the host pthread that calls runtime_handle_tick at
    // the configured sample rate.
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

    // Liveness check. Only meaningful when the runtime is RUNNING — the gate
    // is keyed on RUNNING status, not on the ISR being off. (TIM5 free-runs
    // from boot, so the ISR and tick_counter advance even in IDLE/DRAINED;
    // the liveness logic just doesn't act unless status == RUNNING.) We
    // refresh the last_progress_time anchor in non-RUNNING states so a state
    // transition INTO RUNNING doesn't immediately trip on a stale anchor.
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
            // segment_id carries the -311 TickIntervalExceeded blocker func
            // address (0 for any other fault) so addr2line can name the
            // scheduler callback that starved the TIM5 tick.
            uint32_t tick_blocker = runtime_handle_tick_blocker(runtime_handle);
            kalico_native_emit_fault_event((uint16_t)fault_code, fault_detail,
                                           tick_blocker);
        }
    }

    // Hard-fault escalation (spec 2026-05-28 §3.4). The runtime status machine
    // is dormant, so we key off last_error directly: on a fresh nonzero fault
    // code, notify the host with the specifics, then enter Klipper's global
    // shutdown — the single stop state. shutdown() does irq_disable()+longjmp
    // back to sched_main; that is safe HERE in foreground (DECL_TASK context)
    // but NOT from the ISR/Rust tick path, which is why escalation lives in this
    // drain rather than at the fault site. The edge guard matters because after
    // shutdown() longjmps, sched_main resumes run_tasks() and this drain can run
    // once more with last_error still latched; last_acted_error is stored before
    // shutdown() (and survives the longjmp as a static), so the trailing pass is
    // suppressed instead of re-emitting + re-shutting-down.
    // (The cur_status == 3 block above is dormant — runtime_status is never set
    // away from Idle today — so this last_error path is the live escalation.)
    static int32_t last_acted_error;
    int32_t cur_error = runtime_handle_last_error(runtime_handle);
    if (cur_error != 0 && cur_error != last_acted_error) {
        last_acted_error = cur_error;
        uint32_t fdetail = runtime_handle_fault_detail(runtime_handle);
        // Segment ids are gone (piece-ring model); the legacy segment-id slot
        // now carries the -311 TickIntervalExceeded blocker func address (0
        // for any other fault) so addr2line can name the scheduler callback
        // that starved the TIM5 tick.
        uint32_t tick_blocker = runtime_handle_tick_blocker(runtime_handle);
        kalico_native_emit_fault_event((uint16_t)cur_error, fdetail,
                                       tick_blocker);
        // Belt-and-suspenders: persist the fault code + detail into the
        // BKPSRAM diagnostic ring BEFORE shutdown() resets the USB stack.
        // The ring survives a soft reset and is emitted by fault_handler_
        // report_task on the next boot as `prior_diag_ring ... tag 8`.
        // This captures the numeric code even when the USB-CDC FaultEvent
        // frame is lost (shutdown → USB drop race). Readable without sudo:
        // grep 'prior_diag_ring.*tag 8' ~/printer_data/logs/klippy.log
        diag_ring_push(DIAG_EV_RUST_FAULT, (uint32_t)cur_error, fdetail);
        runtime_liveness_ok = 0;
        shutdown("kalico runtime fault");
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

// Single stop state (spec 2026-05-28): TIM5 goes off when Klipper shuts down.
// The per-axis step-consumer timers are already wiped by sched_timer_reset
// during shutdown, so motion has stopped; this just halts the now-pointless ISR
// compute (and avoids Renode USART2 starvation). Re-armed on FIRMWARE_RESTART
// via runtime_tick_init.
void
runtime_tick_shutdown(void)
{
    runtime_tick_disable();
}
DECL_SHUTDOWN(runtime_tick_shutdown);

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
    send_status_heartbeat();

    // Diag heartbeat for the status emit task. Threshold: 200 ms (we run
    // at 10 Hz so a 200 ms gap means we missed two cycles, which is what
    // we expect during the 500 ms stall).
    diag_task_heartbeat(diag_slot_rt_status_calls(),
                        diag_slot_rt_status_last_tick(),
                        diag_slot_rt_status_max_gap(),
                        timer_from_us(200000),
                        0); // no event tag — emit gap shows up as missing emits

#if defined(__linux__) || defined(__APPLE__)
    // Sim-only: dump stepper counters so a test that lost its klippy
    // bridge_call link can still observe motion progress via the elf log.
    // Phase 4 test polls this to confirm GATE GREEN.
    uint8_t status = runtime_handle_status(runtime_handle);
    int32_t c0 = kalico_runtime_get_stepper_count(runtime_handle, 0);
    int32_t c1 = kalico_runtime_get_stepper_count(runtime_handle, 1);
    int32_t c2 = kalico_runtime_get_stepper_count(runtime_handle, 2);
    extern uint32_t kalico_runtime_get_xdirect_write_count(void);
    uint32_t spi_writes = kalico_runtime_get_xdirect_write_count();
    fprintf(stderr,
        "[sim-progress] status=%u counts=[%d,%d,%d]"
        " spi_writes=%u\n",
        status, c0, c1, c2, spi_writes);
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
// stream_flush, clock_sync_request) and the endstop GPIO sampler
// hot-path (`runtime_endstop_sample_pins` + `endstop_pin_table`) live
// in src/runtime_commands.c. This file keeps only lifecycle
// (init/drain), sibling drains (status_drain, endstop_drain), and
// shared globals.

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

// kalico_fault is an MCU-emitted async event. It rides the kalico-native
// events channel via kalico_native_emit_fault_event in src/kalico_dispatch.c.
// The periodic StatusHeartbeat (send_status_heartbeat) carries per-axis
// consumed-count credit flow and engine state.
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
// via `arm_per_axis_step_timer`, which arms a timer only for axes this MCU
// actually drives.

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

// Arm the per-axis step-emission timer for a single axis, the first time
// that axis is configured. Called per-axis from `command_kalico_configure_axis`
// (stepper.c) so an MCU only runs step timers for axes it actually drives.
//
// Rationale: every armed per-axis timer reschedules itself at the sample rate
// even when its step queue is empty (see kalico_per_axis_step_event's empty-
// queue path: it returns `now + sample_period`). Arming a timer for an axis
// this MCU does NOT own (e.g. Z on the XY board) therefore adds a needless
// sample-rate dispatch load at the SAME NVIC priority as TIM5 — and since
// TIM5 cannot preempt the SysTick dispatch loop, a cluster of those dispatches
// can starve the motion tick past 2 sample periods → -311 TickIntervalExceeded.
// Bench addr2line confirmed the starving callbacks were exactly the unowned-
// axis timers (per_axis_timer_event_2 on the H7, _3 on the F446). Only arming
// owned axes removes that load at the source.
//
// Re-adding an already-queued timer would corrupt the scheduler's linked list,
// so each axis is armed at most once (tracked by `per_axis_armed_mask`).
//
// `runtime_emit_step_pulses` is defined in src/stepper.c. The Rust body
// resolves the C-declared `step_queues` array internally; this file owns
// the trampolines + scheduler wiring.
void
arm_per_axis_step_timer(uint8_t axis_idx)
{
    static uint8_t per_axis_armed_mask;
    if (axis_idx >= 4)
        return;
    if (per_axis_armed_mask & (uint8_t)(1u << axis_idx))
        return; // already queued — re-adding would corrupt the timer list
    per_axis_armed_mask |= (uint8_t)(1u << axis_idx);

    per_axis_timers[axis_idx].func = per_axis_handlers[axis_idx];
    // Park the first dispatch far in the future (sched_add_timer trips
    // "Timer too close" on a past waketime). Subsequent waketimes come from
    // kalico_per_axis_step_event's return value.
    per_axis_timers[axis_idx].waketime = timer_read_time() + (uint32_t)0x3FFFFFFF;
    sched_add_timer(&per_axis_timers[axis_idx]);
}

// Task 14 SPI queue drain removed — dispatch_phase now calls
// phase_stepping_write_xdirect directly from the ISR. The SPSC queue
// could never keep up (160K entries/sec from 40 kHz ISR × 4 motors,
// drain processed ~10K/sec with blocking SPI). Direct ISR write with
// skip-not-block (phase_spi_try_acquire) matches mass3d/kalico's
// working architecture.
