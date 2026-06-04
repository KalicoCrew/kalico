// Cortex-M fault handler — captures the exception stack frame and fault
// status registers into a non-cleared AXI-RAM region (H7) or DTCM region
// (other) so the next boot can report what crashed the previous run.
//
// Wires HardFault / BusFault / UsageFault / MemManage handlers that all
// capture the same record and request a system reset. On the next boot,
// `report_prior_fault_task` checks for a valid record and emits an
// `output("prior_fault …")` console message that ends up in klippy.log.
//
// Designed for diagnosing silent reboots — when the H7 disappears off
// USB and reattaches, the fault record survives the soft-reset (SRAM
// contents persist) and tells us PC / LR / fault status of the crash.

#include <stdint.h>
#include <string.h>
#include "autoconf.h"
#include "board/internal.h"
#include "board/irq.h"        // irq_save / irq_restore for ring push
#include "command.h"
#include "sched.h"
#include "kalico_log.h"       // kalico_log_emit + KALICO_LOG_* (crash-forensics emit)

extern volatile uint8_t runtime_liveness_ok;
extern void *runtime_handle;
extern uint32_t runtime_handle_tick_counter(void *handle);
extern uint8_t  runtime_handle_status(void *handle);
// Persistent diag struct lives in .persistent_diag (runtime_tick.c).
// Forward-declare the struct + extern the global so fault_handler_report_task
// can read it directly. Inlining the read avoids LTO stripping a separate
// runtime_diag_emit_persistent() function whose call site LTO considered dead.
struct rt_diag_persistent {
    uint32_t magic;
    uint32_t last_packed;
    uint32_t last_us;
    uint32_t fault_count;
};
extern volatile struct rt_diag_persistent rt_diag_persistent;

// Magic word marks "fault record valid". Chosen unlikely to occur as
// random SRAM contents on power-on (when contents are undefined).
#define FAULT_MAGIC 0x46415541u   // "AAUF" — Asserted Authentic Uncovered Fault

struct fault_record {
    uint32_t magic;
    uint32_t exc_kind;     // 1=HardFault, 2=BusFault, 3=UsageFault, 4=MemManage
    uint32_t r0, r1, r2, r3, r12, lr, pc, psr;
    uint32_t cfsr, hfsr, dfsr, bfar, mmfar, afsr;
    uint32_t exc_return;
    uint32_t shcsr;
    uint32_t fault_count;
};

// Liveness snapshot in persistent SRAM — survives soft reset. `magic` lets
// boot distinguish "fresh power-on" from "valid prior-run snapshot".
#define LIVE_MAGIC 0x4C495645u  // "LIVE"

struct live_snapshot {
    uint32_t magic;
    uint32_t live;          // runtime_liveness_ok at sample time
    uint32_t engine_status; // runtime_handle_status (0=IDLE 1=RUNNING 2=DRAINED 3=FAULT)
    uint32_t tick_counter;  // runtime_handle_tick_counter
    uint32_t sample_time;   // timer_read_time() at sample
    uint32_t boot_count;
    uint32_t last_engine_running_tick;
    uint32_t samples_taken;
    // Cross-boot persistent foreground-freeze watchdog. TIM5 ISR watches
    // the foreground heartbeat (samples_taken); latches worst stall + stacked
    // PC/exc so a PRIMASK-freeze is identifiable after the IWDG reset.
    uint32_t worst_fg_stall_ticks;
    uint32_t worst_fg_stall_pc;
    uint32_t worst_fg_stall_exc;
    uint32_t iwdg_reset_count;     // # boots reset by IWDG (signature of foreground freeze)
    // Last scheduler callback dispatched (persistent, written BEFORE the call).
    // A hanging callback → PRIMASK held → IWDG → after reset, addr2line this.
    uint32_t last_dispatch_func;
    uint32_t last_dispatch_addr;
    // Per-run foreground-freeze flag (set THIS run when fg_stall_ticks crosses
    // FG_FREEZE_REPORT_THRESHOLD). Unlike worst_fg_stall_* (cross-boot max), this
    // describes only the current run, so the next boot can gate the crash report
    // on "the run that just ended froze" — surviving klippy's connect-reset,
    // which masks the immediate RCC reset cause. Reset to 0 at each boot-init.
    uint32_t this_run_froze;
};

// H7: backup SRAM (.bkp_bss) — survives IWDG/soft reset; requires
// PWR->CR2.BREN + RCC->AHB4ENR.BKPRAMEN (see fault_handler_init).
// non-H7: .persistent_diag NOLOAD section outside [_bss_start.._bss_end]
// so the boot zero-pass leaves it alone; main RAM survives soft reset.
#if CONFIG_MACH_STM32H7
__attribute__((section(".bkp_bss"), used))
#else
__attribute__((section(".persistent_diag"), used))
#endif
static volatile struct fault_record fault_rec;

#if CONFIG_MACH_STM32H7
__attribute__((section(".bkp_bss"), used))
#else
__attribute__((section(".persistent_diag"), used))
#endif
static volatile struct live_snapshot live_snap;

// H7: BKPSRAM is D-cache-backed; SCB_CleanDCache_by_Addr is required after
// writes — __DSB() alone only drains the store buffer, not the cache lines.
#if CONFIG_MACH_STM32H7
static inline void
diag_cache_clean(void)
{
    extern uint8_t _bkp_bss_start, _bkp_bss_end;
    uint32_t addr = (uint32_t)&_bkp_bss_start;
    uint32_t size = (uint32_t)(&_bkp_bss_end - &_bkp_bss_start);
    SCB_CleanDCache_by_Addr((uint32_t*)addr, (int32_t)size);
    __DSB();
}
#else
static inline void diag_cache_clean(void) { __DSB(); }
#endif

// =============================================================================
// Diagnostic counters + event ring (persistent across soft reset).
// IRQ-safe ring push uses irq_save/irq_restore; cost ~10 cycles — negligible.
// =============================================================================
#define DIAG_MAGIC      0x4449414Eu  // "DIAN" — diagnostic counters present
#define DIAG_RING_LEN   32           // power-of-two for cheap mask
#define DIAG_RING_MASK  (DIAG_RING_LEN - 1)
// Foreground stall ticks (TIM5-ISR-counted, one per sample period) that mark a
// genuine freeze for the per-run this_run_froze flag. 8 ≈ 0.8 ms of stalled
// foreground at the H7 sample rate — above scheduling jitter, below the IWDG
// timeout. Observed crash worst was 10.
#define FG_FREEZE_REPORT_THRESHOLD 8

// Event tags — small u8 so we have headroom for future events.
enum {
    DIAG_EV_NONE          = 0,
    DIAG_EV_TIM5_LONG     = 1,   // a=duration_cycles, b=enter_time
    DIAG_EV_OTG_LONG      = 2,   // a=duration_cycles, b=enter_time
    DIAG_EV_USB_OUT_GAP   = 3,   // a=gap_us, b=prev_call_time
    DIAG_EV_USB_IN_GAP    = 4,   // a=gap_us, b=prev_call_time
    DIAG_EV_TX_DROP_KAL   = 5,   // a=len, b=transmit_pos_at_drop
    DIAG_EV_TX_DROP_KLP   = 6,   // a=max_size, b=transmit_pos_at_drop
    DIAG_EV_ENGINE_XITION = 7,   // a=(prev<<8)|new, b=samples_taken
    DIAG_EV_RUST_FAULT    = 8,   // a=(uint32_t)last_error, b=fault_detail
                                 // (pushed from runtime_tick.c; see header)
};

struct diag_event {
    uint8_t  tag;
    uint8_t  _pad0;
    uint16_t seq;          // monotonic — distinguishes wrap from no-events
    uint32_t timestamp;    // timer_read_time() at push
    uint32_t a;
    uint32_t b;
};

// Per-IRQ DWT duration histogram: 16 buckets × 4096 cycles (~7.88 µs each
// at 520 MHz). Bucket index = clamp(dur >> 12, 0..15); bucket 15 absorbs
// everything ≥ 126 µs; absolute peak is tracked separately in *_cycles_max.
#define DIAG_HIST_NBUCKETS 16
#define DIAG_HIST_SHIFT    12

struct diag_counters {
    uint32_t magic;

    // IRQ counters. cycles_* in DWT units (520 MHz on H7). total is u64 to
    // avoid silent wrap (~138 s at 40 kHz × 60 µs/tick on u32).
    uint32_t tim5_irq_count;
    uint64_t tim5_irq_cycles_total;
    uint32_t tim5_irq_cycles_max;
    uint32_t otg_irq_count;
    uint64_t otg_irq_cycles_total;
    uint32_t otg_irq_cycles_max;

    // tim5_irq_buckets: full ISR entry→exit; rt_tick_buckets: engine only.
    // Pair them to isolate cost in engine vs. surrounding ISR overhead.
    uint32_t tim5_irq_buckets[DIAG_HIST_NBUCKETS];
    uint32_t rt_tick_count;
    uint32_t rt_tick_cycles_max;
    uint64_t rt_tick_cycles_total;
    uint32_t rt_tick_buckets[DIAG_HIST_NBUCKETS];

    // Per-stage cumulative timing inside runtime_handle_tick (vestigial from
    // the old scalar-eval engine — no active callers; retained for reference).
    uint32_t rt_eval_n;
    uint32_t rt_eval_cycles_max;
    uint64_t rt_eval_cycles_total;
    uint32_t rt_dvel_n;
    uint32_t rt_dvel_cycles_max;
    uint64_t rt_dvel_cycles_total;

    // Walk/monomialise split: walk = ring-walk (no conversion); monomial =
    // arm_and_load on cold-load ticks only. Decompose worst-case tick cost.
    uint32_t walk_cycles_max;
    uint32_t walk_n;
    uint32_t monomial_cycles_max;
    uint32_t monomial_n;

    // ISR-phase breadcrumb (RT_PHASE_* from fault_handler.h). Survives IWDG
    // reset; names the phase a PRIMASK-freeze hung in.
    uint32_t rt_isr_phase;

    // Curve shape: per-axis (degree, cps_len, knots_len) at last activation.
    // [0]=X, [1]=Y, [2]=Z; 0/0 = axis idle.
    uint8_t  rt_curve_degree[3];
    uint16_t rt_curve_cps_len[3];
    uint16_t rt_curve_knots_len[3];

    // Foreground task heartbeats: calls, last_tick, max_gap (timer_read_time units).
    uint32_t usb_out_calls;
    uint32_t usb_out_last_tick;
    uint32_t usb_out_max_gap_ticks;
    uint32_t usb_in_calls;
    uint32_t usb_in_last_tick;
    uint32_t usb_in_max_gap_ticks;
    uint32_t runtime_drain_calls;
    uint32_t runtime_drain_last_tick;
    uint32_t runtime_drain_max_gap_ticks;
    uint32_t runtime_status_calls;
    uint32_t runtime_status_last_tick;
    uint32_t runtime_status_max_gap_ticks;

    // TX-drop counters.
    uint32_t tx_drops_kalico;
    uint32_t tx_drops_klipper;
    uint32_t tx_drops_kalico_last_len;
    uint32_t tx_drops_klipper_last_max;

    // Ring write head + sequence (head wraps via mask; seq monotonic).
    uint32_t ring_head;
    uint32_t ring_seq;
    uint32_t ring_overflow; // count of overwritten unread entries

    // Boot-time bookkeeping.
    uint32_t boot_count;

    // Per-flag OTG IRQ counters (RXFLVL → host-to-MCU notify path).
    uint32_t otg_rxflvl_fires;
    uint32_t otg_iepint_fires;
    uint32_t otg_otherflag_fires;
    uint32_t otg_otherflag_last_sts;

    // Bulk-OUT path counters.
    uint32_t notify_bulk_out_calls;     // usb_notify_bulk_out invocations
    uint32_t task_invoke_count;         // usb_bulk_out_task entries (before wake gate)
    uint32_t usb_read_zero_returns;     // usb_read_bulk_out returned <= 0
    uint32_t usb_read_data_returns;     // usb_read_bulk_out returned > 0

    // OTG live register snapshots (read at periodic emit).
    uint32_t otg_gintmsk_now;
    uint32_t otg_gintsts_now;

    // Bulk-OUT EP register snapshots and re-arm accounting.
    uint32_t out_ep_doepctl;
    uint32_t out_ep_doeptsiz;
    uint32_t out_ep_doepint;
    uint32_t enable_rx_n;           // total enable_rx_endpoint calls
    uint32_t enable_rx_rearmed_n;   // branch that actually wrote EPENA|CNAK
    uint32_t peek_empty_n;          // queue empty → re-enable RXFLVLM
    uint32_t peek_data_n;           // queue had data → drained

    // -311 block-source discriminators (DWT cycles).
    // systick_max: SysTick single dispatch (holds PRIMASK — direct TIM5 fence).
    // stepout_max/burst: step-output one-shot single + contiguous-burst span.
    // usb_burst: USB OTG ISR contiguous-burst span (higher prio than TIM5).
    // A fence blocker reaches ~2x TIM5 period: H7 >= 26000 cyc, F446 >= 18000.
    uint32_t systick_max_cyc;
    uint32_t stepout_max_cyc;
    uint32_t stepout_burst_max_cyc;
    uint32_t usb_burst_max_cyc;

    // -311 PRIMARY discriminator: TIM5 entry-to-entry inter-arrival (DWT).
    // min ~= 2x period → clock-domain half-rate; min ~= 1x period → real fence.
    // min is immune to stale-prev (disable/enable only injects a large gap into max).
    uint32_t tim5_ia_min_cyc;
    uint32_t tim5_ia_max_cyc;
    uint32_t tim5_ia_last_cyc;

    // USB-CDC halt: in_busy = bulk-IN EPENA still set (host not draining);
    // gintsts_sticky = OR of all GINTSTS (catches transient USBRST/USBSUSP).
    uint32_t usb_in_busy_n;
    uint32_t usb_gintsts_sticky;
    uint32_t usb_gintsts_now;
    uint32_t usb_gintmsk_now;
    uint32_t usb_in_diepctl;
    uint32_t usb_in_diepint;
    uint32_t usb_in_dtxfsts;
    uint32_t usb_out_doepctl;
    uint32_t usb_out_doepint;
};

#if CONFIG_MACH_STM32H7
__attribute__((section(".bkp_bss"), used))
#else
__attribute__((section(".persistent_diag"), used))
#endif
static volatile struct diag_counters diag;

#if CONFIG_MACH_STM32H7
__attribute__((section(".bkp_bss"), used))
#else
__attribute__((section(".persistent_diag"), used))
#endif
static volatile struct diag_event diag_ring[DIAG_RING_LEN];

// Saved snapshot of prior-run counters + ring, populated once on boot and
// emitted by the report task. Lives in .bss (zero on reset, populated from
// the prior-run persistent struct before we overwrite it).
static struct diag_counters prior_diag;
static struct diag_event    prior_ring[DIAG_RING_LEN];
static uint32_t             prior_diag_present;
// Scratch copy for kalico_diag_emit_live: the live ring is snapshotted here
// under irq_save (ISRs push concurrently), then emitted outside the critical
// section. File-static to keep it off the command-handler stack.
static struct diag_event    dump_ring[DIAG_RING_LEN];

// IRQ-safe push to the diag ring. Used from foreground AND IRQ context, so
// the head/seq update is protected with irq_save. The struct stores are
// volatile — compiler can't reorder them across the irq_save/restore pair.
void
diag_ring_push(uint8_t tag, uint32_t a, uint32_t b)
{
    // timer_read_time is generally IRQ-safe on Cortex-M (read of a 32-bit
    // counter or a snapshot routine that itself disables IRQs). Per
    // armcm_timer.c the H7 implementation reads SysTick + a 32-bit
    // overflow counter under irq_save, so calling it from IRQ is fine.
    extern uint32_t timer_read_time(void);
    irqstatus_t flag = irq_save();
    uint32_t head = diag.ring_head & DIAG_RING_MASK;
    uint32_t next = (head + 1) & DIAG_RING_MASK;
    diag_ring[head].tag = tag;
    diag_ring[head]._pad0 = 0;
    diag_ring[head].seq = (uint16_t)(diag.ring_seq & 0xFFFF);
    diag_ring[head].timestamp = timer_read_time();
    diag_ring[head].a = a;
    diag_ring[head].b = b;
    diag.ring_head = next;
    diag.ring_seq++;
    if (diag.ring_seq > DIAG_RING_LEN
        && (diag.ring_seq - DIAG_RING_LEN) > diag.ring_overflow)
        diag.ring_overflow = diag.ring_seq - DIAG_RING_LEN;
    // Flush the cache so the ring entry + ring_seq survive a near-future
    // reset. SCB_CleanDCache_by_Addr is bounded (< 1 KB region) but
    // non-trivial in IRQ context — we accept the cost because the only
    // alternative is losing ring data to cache eviction lag.
    diag_cache_clean();
    irq_restore(flag);
}

// Update a task heartbeat. Called at the START of a task body, BEFORE the
// task does any work. Records `now`, computes gap from `last_tick`, updates
// max_gap, and pushes a `*_GAP` event when the gap exceeds threshold.
//
// `tag` selects which event tag is pushed when the gap is unusual. Pass
// 0 to suppress event emission (e.g., if the task itself is too noisy).
void
diag_task_heartbeat(volatile uint32_t *calls,
                    volatile uint32_t *last_tick,
                    volatile uint32_t *max_gap,
                    uint32_t threshold_ticks,
                    uint8_t event_tag)
{
    extern uint32_t timer_read_time(void);
    uint32_t now = timer_read_time();
    uint32_t prev = *last_tick;
    *calls = *calls + 1;
    *last_tick = now;
    if (prev != 0) {
        uint32_t gap = now - prev;
        if (gap > *max_gap)
            *max_gap = gap;
        if (event_tag && gap > threshold_ticks)
            diag_ring_push(event_tag, gap, prev);
    }
}

// =============================================================================
// IRQ instrumentation entrypoints — called from TIM5_IRQHandler and
// OTG_FS_IRQHandler. Inlined accumulator pattern to keep the IRQ-side cost
// bounded: one DWT read at entry, one at exit, three increments + a max.
//
// `long_irq_threshold_cycles` lets each handler set its own ring-push
// threshold based on its expected steady-state duration (TIM5 ~6us @ 520MHz
// = ~3000 cycles; OTG ~3us = ~1500 cycles).
// =============================================================================
uint32_t
diag_get_tim5_count(void)
{
    return diag.tim5_irq_count;
}

uint32_t
diag_get_rt_tick_count(void)
{
    return diag.rt_tick_count;
}

uint32_t
diag_get_rt_tick_cycles_max(void)
{
    return diag.rt_tick_cycles_max;
}

uint32_t
diag_get_tx_drops_kalico(void)
{
    return diag.tx_drops_kalico;
}

uint32_t
diag_get_tx_drops_klipper(void)
{
    return diag.tx_drops_klipper;
}

void
diag_tim5_account(uint32_t enter_cycles, uint32_t exit_cycles)
{
    uint32_t dur = exit_cycles - enter_cycles;
    diag.tim5_irq_count++;

    // -311 PRIMARY: entry-to-entry inter-arrival. min is robust (disable/enable
    // only injects a large gap into max, never into min).
    static uint32_t prev_enter;
    static uint8_t  have_prev;
    if (have_prev) {
        uint32_t ia = enter_cycles - prev_enter;  // wrap-safe u32 DWT delta
        diag.tim5_ia_last_cyc = ia;
        if (ia > diag.tim5_ia_max_cyc)
            diag.tim5_ia_max_cyc = ia;
        if (diag.tim5_ia_min_cyc == 0 || ia < diag.tim5_ia_min_cyc)
            diag.tim5_ia_min_cyc = ia;
    }
    prev_enter = enter_cycles;
    have_prev = 1;

    diag.tim5_irq_cycles_total += dur;
    if (dur > diag.tim5_irq_cycles_max)
        diag.tim5_irq_cycles_max = dur;
    uint32_t bucket = dur >> DIAG_HIST_SHIFT;
    if (bucket >= DIAG_HIST_NBUCKETS)
        bucket = DIAG_HIST_NBUCKETS - 1;
    diag.tim5_irq_buckets[bucket]++;
    // Threshold: 50us at 520 MHz = 26000 cycles. Steady-state TIM5 is ~3000
    // cycles. 50us ≈ 8x normal — a real outlier worth recording.
    if (dur > 26000u)
        diag_ring_push(DIAG_EV_TIM5_LONG, dur, enter_cycles);

    // Foreground-freeze watchdog: watches samples_taken; latches worst stall +
    // stacked PC/exc into cross-boot-persistent live_snap. fg_seen_advance gates
    // counting until the heartbeat has advanced at least once (excludes boot
    // window). If worst_fg_stall_pc stays 0 across a known freeze, PRIMASK held.
    static uint32_t fg_hb_prev;
    static uint32_t fg_stall_ticks;
    static uint8_t  fg_init;
    static uint8_t  fg_seen_advance;
    uint32_t hb = live_snap.samples_taken;
    if (!fg_init) {
        fg_hb_prev = hb;
        fg_init = 1;
    } else if (hb != fg_hb_prev) {
        fg_hb_prev = hb;
        fg_stall_ticks = 0;
        fg_seen_advance = 1;
    } else if (fg_seen_advance) {
        fg_stall_ticks++;
        // Per-run freeze flag (gates the next boot's crash report; survives
        // klippy's connect-reset via BKPSRAM, unlike the immediate RCC cause).
        if (fg_stall_ticks >= FG_FREEZE_REPORT_THRESHOLD)
            live_snap.this_run_froze = 1;
        if (fg_stall_ticks > live_snap.worst_fg_stall_ticks) {
            extern uint32_t runtime_tim5_stacked_pc(void);
            extern uint32_t runtime_tim5_stacked_exc(void);
            live_snap.worst_fg_stall_ticks = fg_stall_ticks;
            live_snap.worst_fg_stall_pc    = runtime_tim5_stacked_pc();
            live_snap.worst_fg_stall_exc   = runtime_tim5_stacked_exc();
        }
    }
}

// Per-stage timing inside runtime_handle_tick (vestigial; no active callers).
__attribute__((used, externally_visible))
void
diag_rt_eval_account(uint32_t cycles)
{
    diag.rt_eval_n++;
    diag.rt_eval_cycles_total += cycles;
    if (cycles > diag.rt_eval_cycles_max)
        diag.rt_eval_cycles_max = cycles;
}

// Curve-shape capture on segment activation. axis_idx 0..=2 (X/Y/Z).
__attribute__((used, externally_visible))
void
diag_rt_curve_meta(uint32_t axis_idx, uint32_t degree,
                   uint32_t cps_len, uint32_t knots_len)
{
    if (axis_idx >= 3) return;
    diag.rt_curve_degree[axis_idx]    = (uint8_t)(degree & 0xFFu);
    diag.rt_curve_cps_len[axis_idx]   = (uint16_t)(cps_len & 0xFFFFu);
    diag.rt_curve_knots_len[axis_idx] = (uint16_t)(knots_len & 0xFFFFu);
}

__attribute__((used, externally_visible))
void
diag_rt_dvel_account(uint32_t cycles)
{
    diag.rt_dvel_n++;
    diag.rt_dvel_cycles_total += cycles;
    if (cycles > diag.rt_dvel_cycles_max)
        diag.rt_dvel_cycles_max = cycles;
}

// Walk/monomialise split timing. Rust-only callers.
__attribute__((used, externally_visible))
void
diag_walk_account(uint32_t cycles)
{
    diag.walk_n++;
    if (cycles > diag.walk_cycles_max)
        diag.walk_cycles_max = cycles;
}

__attribute__((used, externally_visible))
void
diag_monomial_account(uint32_t cycles)
{
    diag.monomial_n++;
    if (cycles > diag.monomial_cycles_max)
        diag.monomial_cycles_max = cycles;
}

// ISR-phase breadcrumb setter. Survives reset; names the phase a freeze hung in.
__attribute__((used, externally_visible))
void
runtime_set_isr_phase(uint32_t phase)
{
    diag.rt_isr_phase = phase;
}

// Histogram the runtime_handle_tick subwindow (already-computed DWT delta).
void
diag_runtime_tick_account(uint32_t cycles)
{
    diag.rt_tick_count++;
    diag.rt_tick_cycles_total += cycles;
    if (cycles > diag.rt_tick_cycles_max)
        diag.rt_tick_cycles_max = cycles;
    uint32_t bucket = cycles >> DIAG_HIST_SHIFT;
    if (bucket >= DIAG_HIST_NBUCKETS)
        bucket = DIAG_HIST_NBUCKETS - 1;
    diag.rt_tick_buckets[bucket]++;
}

void diag_usb_burst_track(uint32_t enter_cycles, uint32_t exit_cycles); // defined below

void
diag_otg_account(uint32_t enter_cycles, uint32_t exit_cycles)
{
    uint32_t dur = exit_cycles - enter_cycles;
    diag.otg_irq_count++;
    diag.otg_irq_cycles_total += dur;
    if (dur > diag.otg_irq_cycles_max)
        diag.otg_irq_cycles_max = dur;
    if (dur > 26000u)
        diag_ring_push(DIAG_EV_OTG_LONG, dur, enter_cycles);
    diag_usb_burst_track(enter_cycles, exit_cycles);
}

// =============================================================================
// -311 block-source burst tracking.
// Stitches back-to-back ISR invocations into a contiguous fence span.
// Gap threshold = H7 TIM5 period (13000 cyc @ 40 kHz/520 MHz) — conservative
// upper bound so bursts on F446 are never falsely split.
// =============================================================================
#define DIAG_BURST_GAP_CYC 13000u   // ~1 TIM5 period (H7 worst case)

// Fold [enter, exit] into the running burst; bump max_out if span is larger.
static inline void
diag_burst_fold(volatile uint32_t *max_out,
                uint32_t *start, uint32_t *last_exit,
                uint32_t enter_cycles, uint32_t exit_cycles)
{
    // Gap since the previous invocation's exit. Wrap-safe subtraction.
    uint32_t gap = enter_cycles - *last_exit;
    if (*last_exit == 0 || gap > DIAG_BURST_GAP_CYC) {
        // Burst closed (or first ever) — start a fresh one at this entry.
        *start = enter_cycles;
    }
    *last_exit = exit_cycles;
    uint32_t span = exit_cycles - *start;
    if (span > *max_out)
        *max_out = span;
}

// SysTick: holds PRIMASK across dispatch; only single-invocation max needed.
void
diag_systick_account(uint32_t enter_cycles, uint32_t exit_cycles)
{
    uint32_t dur = exit_cycles - enter_cycles;
    if (dur > diag.systick_max_cyc)
        diag.systick_max_cyc = dur;
}

// Step-output one-shot ISR (TIM3 H7 / TIM2 F4): single + contiguous burst.
void
diag_stepout_account(uint32_t enter_cycles, uint32_t exit_cycles)
{
    static uint32_t burst_start;
    static uint32_t burst_last_exit;
    uint32_t dur = exit_cycles - enter_cycles;
    if (dur > diag.stepout_max_cyc)
        diag.stepout_max_cyc = dur;
    diag_burst_fold(&diag.stepout_burst_max_cyc,
                    &burst_start, &burst_last_exit,
                    enter_cycles, exit_cycles);
}

// USB OTG ISR contiguous burst. Called from diag_otg_account (single-stamp).
void
diag_usb_burst_track(uint32_t enter_cycles, uint32_t exit_cycles)
{
    static uint32_t burst_start;
    static uint32_t burst_last_exit;
    diag_burst_fold(&diag.usb_burst_max_cyc,
                    &burst_start, &burst_last_exit,
                    enter_cycles, exit_cycles);
}

// =============================================================================
// Snapshot accessors — irq_save for coherence with IRQ-updated counters.
// =============================================================================
struct diag_snapshot {
    uint32_t tim5_n, tim5_total, tim5_max;
    uint32_t otg_n,  otg_total,  otg_max;
    uint32_t usb_out_calls, usb_out_max_gap;
    uint32_t usb_in_calls,  usb_in_max_gap;
    uint32_t runtime_drain_calls, runtime_drain_max_gap;
    uint32_t runtime_status_calls, runtime_status_max_gap;
    uint32_t tx_drops_kalico, tx_drops_klipper;
    uint32_t ring_seq, ring_overflow;
};

void
diag_take_snapshot(struct diag_snapshot *s)
{
    irqstatus_t flag = irq_save();
    s->tim5_n      = diag.tim5_irq_count;
    // u64 totals truncated to u32 here; prior_diag dump emits lo/hi pairs.
    s->tim5_total  = (uint32_t)diag.tim5_irq_cycles_total;
    s->tim5_max    = diag.tim5_irq_cycles_max;
    s->otg_n       = diag.otg_irq_count;
    s->otg_total   = (uint32_t)diag.otg_irq_cycles_total;
    s->otg_max     = diag.otg_irq_cycles_max;
    s->usb_out_calls    = diag.usb_out_calls;
    s->usb_out_max_gap  = diag.usb_out_max_gap_ticks;
    s->usb_in_calls     = diag.usb_in_calls;
    s->usb_in_max_gap   = diag.usb_in_max_gap_ticks;
    s->runtime_drain_calls   = diag.runtime_drain_calls;
    s->runtime_drain_max_gap = diag.runtime_drain_max_gap_ticks;
    s->runtime_status_calls   = diag.runtime_status_calls;
    s->runtime_status_max_gap = diag.runtime_status_max_gap_ticks;
    s->tx_drops_kalico  = diag.tx_drops_kalico;
    s->tx_drops_klipper = diag.tx_drops_klipper;
    s->ring_seq      = diag.ring_seq;
    s->ring_overflow = diag.ring_overflow;
    // Reset max trackers per interval; counts/totals remain cumulative.
    diag.tim5_irq_cycles_max = 0;
    diag.otg_irq_cycles_max  = 0;
    diag.usb_out_max_gap_ticks = 0;
    diag.usb_in_max_gap_ticks  = 0;
    diag.runtime_drain_max_gap_ticks = 0;
    diag.runtime_status_max_gap_ticks = 0;
    diag_cache_clean();
    irq_restore(flag);
}

// Heartbeat slot accessors (avoid taking addresses of volatile members across TUs).
volatile uint32_t *diag_slot_usb_out_calls(void)        { return &diag.usb_out_calls; }
volatile uint32_t *diag_slot_usb_out_last_tick(void)    { return &diag.usb_out_last_tick; }
volatile uint32_t *diag_slot_usb_out_max_gap(void)      { return &diag.usb_out_max_gap_ticks; }
volatile uint32_t *diag_slot_usb_in_calls(void)         { return &diag.usb_in_calls; }
volatile uint32_t *diag_slot_usb_in_last_tick(void)     { return &diag.usb_in_last_tick; }
volatile uint32_t *diag_slot_usb_in_max_gap(void)       { return &diag.usb_in_max_gap_ticks; }
volatile uint32_t *diag_slot_rt_drain_calls(void)       { return &diag.runtime_drain_calls; }
volatile uint32_t *diag_slot_rt_drain_last_tick(void)   { return &diag.runtime_drain_last_tick; }
volatile uint32_t *diag_slot_rt_drain_max_gap(void)     { return &diag.runtime_drain_max_gap_ticks; }
volatile uint32_t *diag_slot_rt_status_calls(void)      { return &diag.runtime_status_calls; }
volatile uint32_t *diag_slot_rt_status_last_tick(void)  { return &diag.runtime_status_last_tick; }
volatile uint32_t *diag_slot_rt_status_max_gap(void)    { return &diag.runtime_status_max_gap_ticks; }

// TX-drop helpers — called from TX-buffer-full early-return paths.
void
diag_record_tx_drop_kalico(uint32_t len, uint32_t tpos)
{
    diag.tx_drops_kalico++;
    diag.tx_drops_kalico_last_len = len;
    diag_ring_push(DIAG_EV_TX_DROP_KAL, len, tpos);
}

void
diag_record_tx_drop_klipper(uint32_t max_size, uint32_t tpos)
{
    diag.tx_drops_klipper++;
    diag.tx_drops_klipper_last_max = max_size;
    diag_ring_push(DIAG_EV_TX_DROP_KLP, max_size, tpos);
}

void
diag_record_engine_xition(uint8_t prev, uint8_t cur, uint32_t samples_taken)
{
    diag_ring_push(DIAG_EV_ENGINE_XITION,
                   ((uint32_t)prev << 8) | (uint32_t)cur,
                   samples_taken);
}

// OTG/bulk-OUT slot accessors (same rationale as heartbeat slots above).
volatile uint32_t *diag_slot_otg_rxflvl(void)         { return &diag.otg_rxflvl_fires; }
volatile uint32_t *diag_slot_otg_iepint(void)         { return &diag.otg_iepint_fires; }
volatile uint32_t *diag_slot_otg_other(void)          { return &diag.otg_otherflag_fires; }
volatile uint32_t *diag_slot_otg_other_sts(void)      { return &diag.otg_otherflag_last_sts; }
volatile uint32_t *diag_slot_notify_bulk_out(void)    { return &diag.notify_bulk_out_calls; }
volatile uint32_t *diag_slot_task_invoke(void)        { return &diag.task_invoke_count; }
volatile uint32_t *diag_slot_read_zero(void)          { return &diag.usb_read_zero_returns; }
volatile uint32_t *diag_slot_read_data(void)          { return &diag.usb_read_data_returns; }

// Snapshot OTG live registers (foreground; no IRQ-safety needed).
void
diag_snapshot_otg_regs(uint32_t gintmsk, uint32_t gintsts)
{
    diag.otg_gintmsk_now = gintmsk;
    diag.otg_gintsts_now = gintsts;
}

// USB-CDC halt: called from usb_diag_poll_task each foreground iteration.
void
diag_usb_poll(uint32_t gintsts, uint32_t gintmsk, uint32_t in_diepctl,
              uint32_t in_diepint, uint32_t in_dtxfsts, uint32_t out_doepctl,
              uint32_t out_doepint)
{
    diag.usb_gintsts_sticky |= gintsts;
    diag.usb_gintsts_now = gintsts;
    diag.usb_gintmsk_now = gintmsk;
    diag.usb_in_diepctl = in_diepctl;
    diag.usb_in_diepint = in_diepint;
    diag.usb_in_dtxfsts = in_dtxfsts;
    diag.usb_out_doepctl = out_doepctl;
    diag.usb_out_doepint = out_doepint;
}

// Bumped when bulk-IN EPENA is still set (host stopped draining the IN endpoint).
void
diag_note_usb_in_busy(void)
{
    diag.usb_in_busy_n++;
}

// Mirror last-dispatched (func, addr) to persistent live_snap before the call.
// A hanging callback → PRIMASK → IWDG reset → addr2line from live_snap after reset.
void
diag_note_dispatch(uint32_t func, uint32_t addr)
{
    live_snap.last_dispatch_func = func;
    live_snap.last_dispatch_addr = addr;
}

// Extended snapshot accessors.
uint32_t diag_get_otg_rxflvl(void)        { return diag.otg_rxflvl_fires; }
uint32_t diag_get_otg_iepint(void)        { return diag.otg_iepint_fires; }
uint32_t diag_get_otg_other(void)         { return diag.otg_otherflag_fires; }
uint32_t diag_get_otg_other_sts(void)     { return diag.otg_otherflag_last_sts; }
uint32_t diag_get_notify_bulk_out(void)   { return diag.notify_bulk_out_calls; }
uint32_t diag_get_task_invoke(void)       { return diag.task_invoke_count; }
uint32_t diag_get_read_zero(void)         { return diag.usb_read_zero_returns; }
uint32_t diag_get_read_data(void)         { return diag.usb_read_data_returns; }
uint32_t diag_get_otg_gintmsk_now(void)   { return diag.otg_gintmsk_now; }
uint32_t diag_get_otg_gintsts_now(void)   { return diag.otg_gintsts_now; }

// OUT EP and enable_rx counters.
volatile uint32_t *diag_slot_enable_rx(void)        { return &diag.enable_rx_n; }
volatile uint32_t *diag_slot_enable_rx_rearm(void)  { return &diag.enable_rx_rearmed_n; }
volatile uint32_t *diag_slot_peek_empty(void)       { return &diag.peek_empty_n; }
volatile uint32_t *diag_slot_peek_data(void)        { return &diag.peek_data_n; }

void
diag_snapshot_out_ep(uint32_t doepctl, uint32_t doeptsiz, uint32_t doepint)
{
    diag.out_ep_doepctl  = doepctl;
    diag.out_ep_doeptsiz = doeptsiz;
    diag.out_ep_doepint  = doepint;
}

uint32_t diag_get_out_ep_doepctl(void)    { return diag.out_ep_doepctl; }
uint32_t diag_get_out_ep_doeptsiz(void)   { return diag.out_ep_doeptsiz; }
uint32_t diag_get_out_ep_doepint(void)    { return diag.out_ep_doepint; }
uint32_t diag_get_enable_rx_n(void)       { return diag.enable_rx_n; }
uint32_t diag_get_enable_rx_rearm(void)   { return diag.enable_rx_rearmed_n; }
uint32_t diag_get_peek_empty(void)        { return diag.peek_empty_n; }
uint32_t diag_get_peek_data(void)         { return diag.peek_data_n; }

void __attribute__((noreturn, used))
fault_capture_and_reset(uint32_t kind, uint32_t *frame, uint32_t exc_return)
{
    // Copy the auto-stacked exception frame: r0, r1, r2, r3, r12, lr, pc, psr.
    fault_rec.r0  = frame[0];
    fault_rec.r1  = frame[1];
    fault_rec.r2  = frame[2];
    fault_rec.r3  = frame[3];
    fault_rec.r12 = frame[4];
    fault_rec.lr  = frame[5];
    fault_rec.pc  = frame[6];
    fault_rec.psr = frame[7];
    fault_rec.exc_return = exc_return;

    // Fault status registers. ARMv7-M (Cortex-M3+) only — ARMv6-M (Cortex-M0+,
    // e.g. STM32G0) has no Configurable Fault Status / fault-address registers;
    // it only has HardFault. Record zeros there. SHCSR exists on both.
#if (__CORTEX_M >= 3)
    fault_rec.cfsr  = SCB->CFSR;
    fault_rec.hfsr  = SCB->HFSR;
    fault_rec.dfsr  = SCB->DFSR;
    fault_rec.bfar  = SCB->BFAR;
    fault_rec.mmfar = SCB->MMFAR;
    fault_rec.afsr  = SCB->AFSR;
#else
    fault_rec.cfsr  = 0;
    fault_rec.hfsr  = 0;
    fault_rec.dfsr  = 0;
    fault_rec.bfar  = 0;
    fault_rec.mmfar = 0;
    fault_rec.afsr  = 0;
#endif
    fault_rec.shcsr = SCB->SHCSR;

    fault_rec.exc_kind = kind;
    if (fault_rec.magic != FAULT_MAGIC)
        fault_rec.fault_count = 0;
    fault_rec.fault_count++;
    fault_rec.magic = FAULT_MAGIC;

    __DSB();  // flush to SRAM before reset
    NVIC_SystemReset();

    for (;;);
}

#include "armcm_boot.h"  // DECL_ARMCM_IRQ

// Naked trampolines: select MSP vs PSP from EXC_RETURN bit 2, tail-call the C
// handler. ARMv7-M uses IT+conditional MRS; ARMv6-M uses a branch-over+BL
// (no IT blocks, no conditional MRS, narrow B can't reach across .text).
#if (__CORTEX_M >= 3)
#define FAULT_TRAMPOLINE_SELECT_SP                                      \
            "tst lr, #4\n\t"                                            \
            "ite eq\n\t"                                                \
            "mrseq r1, msp\n\t"                                         \
            "mrsne r1, psp\n\t"
#define FAULT_TRAMPOLINE_TAIL "b fault_capture_and_reset\n\t"
#else
#define FAULT_TRAMPOLINE_SELECT_SP                                      \
            "movs r1, #4\n\t"                                          \
            "mov  r2, lr\n\t"                                          \
            "tst  r2, r1\n\t"                                          \
            "beq  1f\n\t"                                              \
            "mrs  r1, psp\n\t"                                         \
            "b    2f\n\t"                                              \
            "1:\n\t"                                                  \
            "mrs  r1, msp\n\t"                                        \
            "2:\n\t"
#define FAULT_TRAMPOLINE_TAIL "bl fault_capture_and_reset\n\t"
#endif

#define FAULT_TRAMPOLINE(NAME, KIND, IRQ_NUM)                           \
    void __attribute__((naked, used)) NAME(void)                        \
    {                                                                   \
        asm volatile (                                                  \
            FAULT_TRAMPOLINE_SELECT_SP                                  \
            "mov r0, %0\n\t"                                            \
            "mov r2, lr\n\t"                                            \
            FAULT_TRAMPOLINE_TAIL                                       \
            : : "i"(KIND) : "r0", "r1", "r2"                            \
        );                                                              \
    }                                                                   \
    DECL_ARMCM_IRQ(NAME, IRQ_NUM)

// Cortex-M exception numbers (negative IRQ).
FAULT_TRAMPOLINE(HardFault_Handler, 1, -13);
#if (__CORTEX_M >= 3)
FAULT_TRAMPOLINE(BusFault_Handler, 2, -11);
FAULT_TRAMPOLINE(UsageFault_Handler, 3, -10);
FAULT_TRAMPOLINE(MemManage_Handler, 4, -12);
#endif

// Enable configurable fault exceptions (BusFault, UsageFault, MemManage) and
// divide-by-zero trapping so they don't all escalate into HardFault.
void
fault_handler_init(void)
{
#if (__CORTEX_M >= 3)
    SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk
                | SCB_SHCSR_BUSFAULTENA_Msk
                | SCB_SHCSR_MEMFAULTENA_Msk;
    SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;  // trap divide-by-zero
    // Unaligned-access trap left off — half-word/word unaligned loads are common.
#endif
#if CONFIG_MACH_STM32H7
    // Enable backup SRAM (RM0468 §6.6.7): clock BKPRAM, lift write
    // protection, enable the backup regulator. Wait for BRRDY.
    RCC->AHB4ENR |= RCC_AHB4ENR_BKPRAMEN;
    PWR->CR1 |= PWR_CR1_DBP;
    PWR->CR2 |= PWR_CR2_BREN;
    {
        volatile int spin = 0;
        while (!(PWR->CR2 & PWR_CR2_BRRDY) && spin < 100000) spin++;
    }
#endif
}
DECL_INIT(fault_handler_init);

// Emit boot diagnostic + prior-fault record on a slow timer so they survive
// klippy reconnect. RCC reset-cause flags tell us how the chip got reset even
// when our fault handler never ran (IWDG, BOR, PMU, etc).
#include "board/misc.h"   // timer_read_time, timer_from_us

static uint32_t boot_first_tick;
static uint32_t boot_tick_initialized;
static uint32_t last_emit_tick;
static uint32_t emits_done;
static uint32_t reset_cause_snapshot;
static uint32_t reset_cause_raw;
// Cached prior-run live_snap (captured at boot before overwrite; .bss = zero).
static uint32_t prior_live_present_at_boot;
static uint32_t saved_prior_live;
static uint32_t saved_prior_engine;
static uint32_t saved_prior_tick;
static uint32_t saved_prior_last_run_tick;
static uint32_t saved_prior_samples;
// Did the run that just ended set the per-run freeze flag? Captured at boot-init
// from live_snap.this_run_froze (which the next line clears for the new run).
// Gates the crash report independently of the immediate RCC reset cause.
static uint32_t prior_run_froze;
// Last scheduler callback dispatched, snapshotted at boot-init. live_snap's copy
// is overwritten on the new run's first dispatch, so this is only faithful to
// the crash for an immediate-reset hang (PRIMASK/IWDG); for a late host reset it
// reflects whatever ran just before boot-init. Best-effort addr2line target.
static uint32_t saved_prior_last_dispatch_func;
static uint32_t saved_prior_last_dispatch_addr;

#if CONFIG_MACH_STM32H7
#include "board/internal.h"  // RCC, etc — pulls in stm32h7xx headers
#endif

static uint32_t
read_reset_cause(void)
{
#if CONFIG_MACH_STM32H7
    return RCC->RSR;
#elif CONFIG_MACH_STM32F4
    return RCC->CSR;
#else
    return 0;
#endif
}

static void
clear_reset_cause(void)
{
#if CONFIG_MACH_STM32H7
    RCC->RSR |= RCC_RSR_RMVF;
#elif CONFIG_MACH_STM32F4
    RCC->CSR |= RCC_CSR_RMVF;
#endif
}

void
fault_handler_report_task(void)
{
    uint32_t now = timer_read_time();
    if (!boot_tick_initialized) {
        boot_first_tick = now;
        boot_tick_initialized = 1;
        last_emit_tick = now - timer_from_us(2000000);  // emit immediately
        reset_cause_snapshot = read_reset_cause();
        reset_cause_raw = reset_cause_snapshot;
        clear_reset_cause();
        // Capture prior-run live_snap before this run overwrites it.
        if (live_snap.magic == LIVE_MAGIC) {
            prior_live_present_at_boot = 1;
            saved_prior_live          = live_snap.live;
            saved_prior_engine        = live_snap.engine_status;
            saved_prior_tick          = live_snap.tick_counter;
            saved_prior_last_run_tick = live_snap.last_engine_running_tick;
            saved_prior_samples       = live_snap.samples_taken;
            // Capture the prior run's freeze flag, then clear for the new run.
            prior_run_froze           = live_snap.this_run_froze;
            live_snap.this_run_froze  = 0;
            // Snapshot last-dispatch before the new run overwrites it (best-effort
            // — faithful only for an immediate-reset hang).
            saved_prior_last_dispatch_func = live_snap.last_dispatch_func;
            saved_prior_last_dispatch_addr = live_snap.last_dispatch_addr;
        } else {
            // First-ever boot: zero cross-boot freeze record before use.
            live_snap.worst_fg_stall_ticks = 0;
            live_snap.worst_fg_stall_pc    = 0;
            live_snap.worst_fg_stall_exc   = 0;
            live_snap.iwdg_reset_count     = 0;
            live_snap.last_dispatch_func   = 0;
            live_snap.last_dispatch_addr   = 0;
            live_snap.this_run_froze       = 0;
        }
        // Count IWDG resets (foreground freeze signature).
#if CONFIG_MACH_STM32H7
        if (reset_cause_raw & RCC_RSR_IWDG1RSTF)
            live_snap.iwdg_reset_count++;
#elif CONFIG_MACH_STM32F4
        if (reset_cause_raw & RCC_CSR_IWDGRSTF)
            live_snap.iwdg_reset_count++;
#endif
        live_snap.samples_taken = 0;  // reset for this run

        // Snapshot prior-run diag before new run overwrites. Skipped on first
        // power-on (magic uninitialised; 1-in-2^32 false-positive is acceptable).
        if (diag.magic == DIAG_MAGIC) {
            prior_diag_present = 1;
            // Field-by-field copy through volatile (struct is small).
            prior_diag.magic                = diag.magic;
            prior_diag.tim5_irq_count       = diag.tim5_irq_count;
            prior_diag.tim5_irq_cycles_total = diag.tim5_irq_cycles_total;
            prior_diag.tim5_irq_cycles_max  = diag.tim5_irq_cycles_max;
            prior_diag.otg_irq_count        = diag.otg_irq_count;
            prior_diag.otg_irq_cycles_total = diag.otg_irq_cycles_total;
            prior_diag.otg_irq_cycles_max   = diag.otg_irq_cycles_max;
            prior_diag.rt_tick_count        = diag.rt_tick_count;
            prior_diag.rt_tick_cycles_max   = diag.rt_tick_cycles_max;
            prior_diag.rt_tick_cycles_total = diag.rt_tick_cycles_total;
            prior_diag.rt_eval_n            = diag.rt_eval_n;
            prior_diag.rt_eval_cycles_max   = diag.rt_eval_cycles_max;
            prior_diag.rt_eval_cycles_total = diag.rt_eval_cycles_total;
            prior_diag.rt_dvel_n            = diag.rt_dvel_n;
            prior_diag.rt_dvel_cycles_max   = diag.rt_dvel_cycles_max;
            prior_diag.rt_dvel_cycles_total = diag.rt_dvel_cycles_total;
            prior_diag.walk_cycles_max      = diag.walk_cycles_max;
            prior_diag.walk_n               = diag.walk_n;
            prior_diag.monomial_cycles_max  = diag.monomial_cycles_max;
            prior_diag.monomial_n           = diag.monomial_n;
            prior_diag.rt_isr_phase         = diag.rt_isr_phase;
            for (uint32_t axis = 0; axis < 3; axis++) {
                prior_diag.rt_curve_degree[axis]    = diag.rt_curve_degree[axis];
                prior_diag.rt_curve_cps_len[axis]   = diag.rt_curve_cps_len[axis];
                prior_diag.rt_curve_knots_len[axis] = diag.rt_curve_knots_len[axis];
            }
            for (uint32_t i = 0; i < DIAG_HIST_NBUCKETS; i++) {
                prior_diag.tim5_irq_buckets[i] = diag.tim5_irq_buckets[i];
                prior_diag.rt_tick_buckets[i]  = diag.rt_tick_buckets[i];
            }
            prior_diag.usb_out_calls        = diag.usb_out_calls;
            prior_diag.usb_out_max_gap_ticks = diag.usb_out_max_gap_ticks;
            prior_diag.usb_in_calls         = diag.usb_in_calls;
            prior_diag.usb_in_max_gap_ticks  = diag.usb_in_max_gap_ticks;
            prior_diag.runtime_drain_calls   = diag.runtime_drain_calls;
            prior_diag.runtime_drain_max_gap_ticks = diag.runtime_drain_max_gap_ticks;
            prior_diag.runtime_status_calls   = diag.runtime_status_calls;
            prior_diag.runtime_status_max_gap_ticks = diag.runtime_status_max_gap_ticks;
            prior_diag.tx_drops_kalico        = diag.tx_drops_kalico;
            prior_diag.tx_drops_klipper       = diag.tx_drops_klipper;
            prior_diag.tx_drops_kalico_last_len = diag.tx_drops_kalico_last_len;
            prior_diag.tx_drops_klipper_last_max = diag.tx_drops_klipper_last_max;
            prior_diag.ring_head            = diag.ring_head;
            prior_diag.ring_seq             = diag.ring_seq;
            prior_diag.ring_overflow        = diag.ring_overflow;
            prior_diag.boot_count           = diag.boot_count;
            prior_diag.systick_max_cyc        = diag.systick_max_cyc;
            prior_diag.stepout_max_cyc        = diag.stepout_max_cyc;
            prior_diag.stepout_burst_max_cyc  = diag.stepout_burst_max_cyc;
            prior_diag.usb_burst_max_cyc      = diag.usb_burst_max_cyc;
            prior_diag.tim5_ia_min_cyc        = diag.tim5_ia_min_cyc;
            prior_diag.tim5_ia_max_cyc        = diag.tim5_ia_max_cyc;
            prior_diag.tim5_ia_last_cyc       = diag.tim5_ia_last_cyc;
            prior_diag.usb_in_busy_n          = diag.usb_in_busy_n;
            prior_diag.usb_gintsts_sticky     = diag.usb_gintsts_sticky;
            prior_diag.usb_gintsts_now        = diag.usb_gintsts_now;
            prior_diag.usb_gintmsk_now        = diag.usb_gintmsk_now;
            prior_diag.usb_in_diepctl         = diag.usb_in_diepctl;
            prior_diag.usb_in_diepint         = diag.usb_in_diepint;
            prior_diag.usb_in_dtxfsts         = diag.usb_in_dtxfsts;
            prior_diag.usb_out_doepctl        = diag.usb_out_doepctl;
            prior_diag.usb_out_doepint        = diag.usb_out_doepint;
            // Capture ring into non-volatile copy for stable iteration below.
            for (uint32_t i = 0; i < DIAG_RING_LEN; i++) {
                prior_ring[i].tag       = diag_ring[i].tag;
                prior_ring[i]._pad0     = diag_ring[i]._pad0;
                prior_ring[i].seq       = diag_ring[i].seq;
                prior_ring[i].timestamp = diag_ring[i].timestamp;
                prior_ring[i].a         = diag_ring[i].a;
                prior_ring[i].b         = diag_ring[i].b;
            }
        }
        // Reset diag for new run (old entries now in prior_ring).
        memset((void *)&diag, 0, sizeof(diag));
        diag.magic = DIAG_MAGIC;
        diag.boot_count = prior_diag_present ? (prior_diag.boot_count + 1) : 1;
        for (uint32_t i = 0; i < DIAG_RING_LEN; i++) {
            diag_ring[i].tag = DIAG_EV_NONE;
            diag_ring[i].seq = 0;
            diag_ring[i].timestamp = 0;
            diag_ring[i].a = 0;
            diag_ring[i].b = 0;
        }
        if (prior_diag_present) {
            output("prior_diag_at_init boot %u tim5_n %u otg_n %u out_n %u in_n %u"
                   " drain_n %u stat_n %u ring_seq %u ring_overflow %u"
                   " drops_kal %u drops_klp %u",
                   prior_diag.boot_count,
                   prior_diag.tim5_irq_count,
                   prior_diag.otg_irq_count,
                   prior_diag.usb_out_calls,
                   prior_diag.usb_in_calls,
                   prior_diag.runtime_drain_calls,
                   prior_diag.runtime_status_calls,
                   prior_diag.ring_seq,
                   prior_diag.ring_overflow,
                   prior_diag.tx_drops_kalico,
                   prior_diag.tx_drops_klipper);
        }
        diag_cache_clean();
        return;
    }
    // Refresh liveness snapshot every task call so it is fresh at any crash.
    {
        uint32_t live_now = runtime_liveness_ok;
        uint8_t engine_now = 0xFF;
        uint32_t tick_now = 0;
        if (runtime_handle) {
            tick_now = runtime_handle_tick_counter(runtime_handle);
            engine_now = runtime_handle_status(runtime_handle);
        }
        if (live_snap.magic != LIVE_MAGIC)
            live_snap.boot_count = 0;
        live_snap.live = live_now;
        live_snap.engine_status = (uint32_t)engine_now;
        live_snap.tick_counter = tick_now;
        live_snap.sample_time = now;
        live_snap.samples_taken++;
        if (engine_now == 1 /* RUNNING */)
            live_snap.last_engine_running_tick = tick_now;
        live_snap.magic = LIVE_MAGIC;
    }
    // Emit every 2 s for the first 6 s of boot (3 cycles covers all 32 ring entries).
    if (emits_done >= 3)
        return;
    uint32_t elapsed = now - last_emit_tick;
    if (elapsed < timer_from_us(2000000))
        return;
    last_emit_tick = now;
    uint32_t since_boot_us = (uint32_t)((uint64_t)(now - boot_first_tick)
                                        * 1000000u
                                        / CONFIG_CLOCK_FREQ);
    // Free-form %u (not name=%u) so the decoder builds #msg for klippy log.
    output("boot_diag emit %u since_us %u rcc %u prior %u live %u engine %u tick %u",
           emits_done, since_boot_us, reset_cause_raw,
           (uint32_t)(fault_rec.magic == FAULT_MAGIC),
           live_snap.live, live_snap.engine_status, live_snap.tick_counter);
    if (prior_live_present_at_boot) {
        output("prior_live live %u engine %u tick %u last_run_tick %u samples %u",
               saved_prior_live, saved_prior_engine,
               saved_prior_tick, saved_prior_last_run_tick,
               saved_prior_samples);
    }
    // Cross-boot foreground-freeze watchdog (worst stall ticks, frozen PC/exc, IWDG count).
    output("fg_freeze stall_ticks %u pc %u exc %u iwdg %u last_disp_func %u last_disp_addr %u",
           live_snap.worst_fg_stall_ticks,
           live_snap.worst_fg_stall_pc,
           live_snap.worst_fg_stall_exc,
           live_snap.iwdg_reset_count,
           live_snap.last_dispatch_func,
           live_snap.last_dispatch_addr);
    if (fault_rec.magic == FAULT_MAGIC) {
        output("prior_fault kind %u count %u pc %u lr %u psr %u"
               " r0 %u r1 %u r2 %u r3 %u r12 %u",
               fault_rec.exc_kind, fault_rec.fault_count,
               fault_rec.pc, fault_rec.lr, fault_rec.psr,
               fault_rec.r0, fault_rec.r1, fault_rec.r2,
               fault_rec.r3, fault_rec.r12);
        output("prior_fault_status cfsr %u hfsr %u bfar %u mmfar %u"
               " shcsr %u exc_return %u",
               fault_rec.cfsr, fault_rec.hfsr,
               fault_rec.bfar, fault_rec.mmfar,
               fault_rec.shcsr, fault_rec.exc_return);
    }
    // Persistent runtime-diag snapshot (src/runtime_tick.c). Inlined here
    // because whole-program LTO was stripping a standalone helper.
    output("rt_diag_prior magic=%u packed=%u us=%u faults=%u",
           rt_diag_persistent.magic,
           rt_diag_persistent.last_packed,
           rt_diag_persistent.last_us,
           rt_diag_persistent.fault_count);
    // Bogus-add diagnostic (sched.c::sched_add_timer).
    extern volatile uint32_t sched_bad_add_caller;
    extern volatile uint32_t sched_bad_add_value;
    extern volatile uint32_t sched_bad_add_stack0;
    extern volatile uint32_t sched_bad_add_stack1;
    extern volatile uint32_t sched_bad_add_stack2;
    extern volatile uint32_t sched_bad_add_blocked_count;
    output("sched_bad_add caller %u value %u blocked %u"
           " sp0 %u sp1 %u sp2 %u",
           sched_bad_add_caller, sched_bad_add_value,
           sched_bad_add_blocked_count,
           sched_bad_add_stack0,
           sched_bad_add_stack1,
           sched_bad_add_stack2);

    if (prior_diag_present) {
        // u64 totals split as lo/hi u32 pairs (output() only knows %u).
        // Combine on host: total = (hi<<32)|lo.
        output("prior_diag_summary boot %u tim5_n %u tim5_max_cyc %u"
               " tim5_total_lo %u tim5_total_hi %u",
               prior_diag.boot_count,
               prior_diag.tim5_irq_count,
               prior_diag.tim5_irq_cycles_max,
               (uint32_t)(prior_diag.tim5_irq_cycles_total & 0xFFFFFFFFu),
               (uint32_t)(prior_diag.tim5_irq_cycles_total >> 32));
        output("prior_diag_summary_rt rt_n %u rt_max_cyc %u"
               " rt_total_lo %u rt_total_hi %u",
               prior_diag.rt_tick_count,
               prior_diag.rt_tick_cycles_max,
               (uint32_t)(prior_diag.rt_tick_cycles_total & 0xFFFFFFFFu),
               (uint32_t)(prior_diag.rt_tick_cycles_total >> 32));
        output("prior_diag_summary_eval n %u max %u total_lo %u total_hi %u",
               prior_diag.rt_eval_n, prior_diag.rt_eval_cycles_max,
               (uint32_t)(prior_diag.rt_eval_cycles_total & 0xFFFFFFFFu),
               (uint32_t)(prior_diag.rt_eval_cycles_total >> 32));
        output("prior_diag_summary_dvel n %u max %u total_lo %u total_hi %u",
               prior_diag.rt_dvel_n, prior_diag.rt_dvel_cycles_max,
               (uint32_t)(prior_diag.rt_dvel_cycles_total & 0xFFFFFFFFu),
               (uint32_t)(prior_diag.rt_dvel_cycles_total >> 32));
        output("prior_diag_phase walk_max %u walk_n %u mono_max %u mono_n %u"
               " isr_phase %u",
               prior_diag.walk_cycles_max, prior_diag.walk_n,
               prior_diag.monomial_cycles_max, prior_diag.monomial_n,
               prior_diag.rt_isr_phase);
        output("prior_diag_summary_curve x_deg %u x_cps %u x_knots %u"
               " y_deg %u y_cps %u y_knots %u z_deg %u z_cps %u z_knots %u",
               (uint32_t)prior_diag.rt_curve_degree[0],
               (uint32_t)prior_diag.rt_curve_cps_len[0],
               (uint32_t)prior_diag.rt_curve_knots_len[0],
               (uint32_t)prior_diag.rt_curve_degree[1],
               (uint32_t)prior_diag.rt_curve_cps_len[1],
               (uint32_t)prior_diag.rt_curve_knots_len[1],
               (uint32_t)prior_diag.rt_curve_degree[2],
               (uint32_t)prior_diag.rt_curve_cps_len[2],
               (uint32_t)prior_diag.rt_curve_knots_len[2]);
        output("prior_diag_summary_otg otg_n %u otg_max_cyc %u"
               " otg_total_lo %u otg_total_hi %u",
               prior_diag.otg_irq_count,
               prior_diag.otg_irq_cycles_max,
               (uint32_t)(prior_diag.otg_irq_cycles_total & 0xFFFFFFFFu),
               (uint32_t)(prior_diag.otg_irq_cycles_total >> 32));
        // -311 block-source dump (DWT cyc): blocker reaches ~2x TIM5 period
        // (H7 >= 26000, F446 >= 18000). USB single is already in otg_max_cyc.
        output("prior_diag_summary_block systick %u stepout %u"
               " stepout_burst %u usb_burst %u",
               prior_diag.systick_max_cyc,
               prior_diag.stepout_max_cyc,
               prior_diag.stepout_burst_max_cyc,
               prior_diag.usb_burst_max_cyc);
        // -311 PRIMARY: TIM5 inter-arrival vs sample period.
        // min ~= 2x period → clock-domain half-rate; min ~= 1x → real fence.
        output("prior_diag_summary_tim5ia min %u max %u last %u period %u",
               prior_diag.tim5_ia_min_cyc,
               prior_diag.tim5_ia_max_cyc,
               prior_diag.tim5_ia_last_cyc,
               (uint32_t)(CONFIG_CLOCK_FREQ / CONFIG_KALICO_MOTION_SAMPLE_RATE_HZ));
        output("prior_diag_summary_usb in_busy %u gintsts_sticky %u gintsts %u"
               " gintmsk %u in_diepctl %u in_diepint %u in_dtxfsts %u"
               " out_doepctl %u out_doepint %u",
               prior_diag.usb_in_busy_n,
               prior_diag.usb_gintsts_sticky,
               prior_diag.usb_gintsts_now,
               prior_diag.usb_gintmsk_now,
               prior_diag.usb_in_diepctl,
               prior_diag.usb_in_diepint,
               prior_diag.usb_in_dtxfsts,
               prior_diag.usb_out_doepctl,
               prior_diag.usb_out_doepint);
        output("prior_diag_tasks out_n %u out_max_gap %u in_n %u in_max_gap %u"
               " drain_n %u drain_max_gap %u stat_n %u stat_max_gap %u",
               prior_diag.usb_out_calls,
               prior_diag.usb_out_max_gap_ticks,
               prior_diag.usb_in_calls,
               prior_diag.usb_in_max_gap_ticks,
               prior_diag.runtime_drain_calls,
               prior_diag.runtime_drain_max_gap_ticks,
               prior_diag.runtime_status_calls,
               prior_diag.runtime_status_max_gap_ticks);
        output("prior_diag_drops kalico %u last_len %u klipper %u last_max %u"
               " ring_seq %u ring_overflow %u",
               prior_diag.tx_drops_kalico,
               prior_diag.tx_drops_kalico_last_len,
               prior_diag.tx_drops_klipper,
               prior_diag.tx_drops_klipper_last_max,
               prior_diag.ring_seq,
               prior_diag.ring_overflow);
        // IRQ duration histogram (split lo/hi to stay within MESSAGE_MAX=64 B).
        output("prior_diag_hist_irq_lo b0 %u b1 %u b2 %u b3 %u b4 %u b5 %u b6 %u b7 %u",
               prior_diag.tim5_irq_buckets[0], prior_diag.tim5_irq_buckets[1],
               prior_diag.tim5_irq_buckets[2], prior_diag.tim5_irq_buckets[3],
               prior_diag.tim5_irq_buckets[4], prior_diag.tim5_irq_buckets[5],
               prior_diag.tim5_irq_buckets[6], prior_diag.tim5_irq_buckets[7]);
        output("prior_diag_hist_irq_hi b8 %u b9 %u b10 %u b11 %u b12 %u b13 %u b14 %u b15 %u",
               prior_diag.tim5_irq_buckets[8], prior_diag.tim5_irq_buckets[9],
               prior_diag.tim5_irq_buckets[10], prior_diag.tim5_irq_buckets[11],
               prior_diag.tim5_irq_buckets[12], prior_diag.tim5_irq_buckets[13],
               prior_diag.tim5_irq_buckets[14], prior_diag.tim5_irq_buckets[15]);
        output("prior_diag_hist_rt_lo b0 %u b1 %u b2 %u b3 %u b4 %u b5 %u b6 %u b7 %u",
               prior_diag.rt_tick_buckets[0], prior_diag.rt_tick_buckets[1],
               prior_diag.rt_tick_buckets[2], prior_diag.rt_tick_buckets[3],
               prior_diag.rt_tick_buckets[4], prior_diag.rt_tick_buckets[5],
               prior_diag.rt_tick_buckets[6], prior_diag.rt_tick_buckets[7]);
        output("prior_diag_hist_rt_hi b8 %u b9 %u b10 %u b11 %u b12 %u b13 %u b14 %u b15 %u",
               prior_diag.rt_tick_buckets[8], prior_diag.rt_tick_buckets[9],
               prior_diag.rt_tick_buckets[10], prior_diag.rt_tick_buckets[11],
               prior_diag.rt_tick_buckets[12], prior_diag.rt_tick_buckets[13],
               prior_diag.rt_tick_buckets[14], prior_diag.rt_tick_buckets[15]);
        // The prior-boot event ring is no longer dumped as text here — it is
        // replayed structured (diag.* under KALICO_LOG_SUBSYS_DIAG) by
        // kalico_diag_emit_prior_crash. The verbose summary above is kept as the
        // klippy.log deep-debug fallback (histograms, USB registers, task
        // heartbeats, fault addresses — fields the structured path doesn't carry).
    }

    emits_done++;
}
DECL_TASK(fault_handler_report_task);

// Map a diag ring tag to a structured-log level for the play-by-play replay.
// Long ISRs and TX drops are notable (warn); a Rust fault is an error; USB gaps
// and engine transitions are context (debug).
static uint8_t
diag_ring_tag_level(uint8_t tag)
{
    switch (tag) {
    case DIAG_EV_RUST_FAULT:
        return KALICO_LOG_LEVEL_ERROR;
    case DIAG_EV_TIM5_LONG:
    case DIAG_EV_OTG_LONG:
    case DIAG_EV_TX_DROP_KAL:
    case DIAG_EV_TX_DROP_KLP:
        return KALICO_LOG_LEVEL_WARN;
    default:
        return KALICO_LOG_LEVEL_DEBUG;
    }
}

// Re-emit the prior-boot crash summary through the structured-log path so a hard
// reset (watchdog / CPU fault) shows up in the host log store, not just
// klippy.log output() strings. Called once from the post-connect configure path
// (stepper.c) — by then the host has installed its mcu-log hook; emitting at
// boot would be lost (same timing constraint as runtime.mcu_ready). All sources
// are the prior-boot snapshot captured at boot-init / persistent SRAM, readable
// here because the current run hasn't overwritten them (fault_rec) or they're
// cross-boot accumulators (live_snap freeze/iwdg) / boot snapshots
// (reset_cause_snapshot). 2x u32 args, so the crash story is a few frames.
void
kalico_diag_emit_prior_crash(void)
{
    uint8_t iwdg = 0;
#if CONFIG_MACH_STM32H7
    iwdg = (reset_cause_snapshot & RCC_RSR_IWDG1RSTF) ? 1u : 0u;
#elif CONFIG_MACH_STM32F4
    iwdg = (reset_cause_snapshot & RCC_CSR_IWDGRSTF) ? 1u : 0u;
#endif
    uint8_t had_fault = (fault_rec.magic == FAULT_MAGIC) ? 1u : 0u;
    // `abnormal` gates the warn level + the play-by-play. It must NOT rely solely
    // on the immediate RCC cause (`iwdg`): klippy's connect-reset overwrites it
    // with SFTRST, so a real foreground freeze shows up only as the per-run
    // prior_run_froze flag (set this-run in the TIM5-ISR watchdog, survives the
    // connect-reset via BKPSRAM). had_fault covers true CPU faults.
    uint8_t abnormal = iwdg || had_fault || prior_run_froze;

    // Reset-cause marker (always): warn if the prior reset was abnormal
    // (watchdog or CPU fault), else debug for a clean power-on / pin reset.
    kalico_log_emit(abnormal ? KALICO_LOG_LEVEL_WARN : KALICO_LOG_LEVEL_DEBUG,
                    KALICO_LOG_SUBSYS_RUNTIME, KALICO_LOG_EVENT_RUNTIME_MCU_RESET,
                    0, reset_cause_snapshot, live_snap.iwdg_reset_count);

    // CPU hard fault: what (exc_kind in code) + where (pc, lr) + status regs
    // (cfsr, hfsr). Only when a fault record is present.
    if (had_fault) {
        kalico_log_emit(KALICO_LOG_LEVEL_ERROR, KALICO_LOG_SUBSYS_RUNTIME,
                        KALICO_LOG_EVENT_RUNTIME_HARD_FAULT,
                        (uint16_t)fault_rec.exc_kind, fault_rec.pc, fault_rec.lr);
        kalico_log_emit(KALICO_LOG_LEVEL_ERROR, KALICO_LOG_SUBSYS_RUNTIME,
                        KALICO_LOG_EVENT_RUNTIME_FAULT_STATUS, 0,
                        fault_rec.cfsr, fault_rec.hfsr);
    }

    // Foreground-freeze: the PC where foreground/ISR hung before the watchdog
    // fired (PRIMASK-freeze signature). Only when latched.
    if (live_snap.worst_fg_stall_ticks) {
        kalico_log_emit(KALICO_LOG_LEVEL_WARN, KALICO_LOG_SUBSYS_RUNTIME,
                        KALICO_LOG_EVENT_RUNTIME_FG_FREEZE, 0,
                        live_snap.worst_fg_stall_pc,
                        live_snap.worst_fg_stall_ticks);
    }

    // How far the runtime got (packed tag/stage/value) + cumulative CPU-fault
    // count. Only on an abnormal reset.
    if (abnormal) {
        extern volatile uint32_t runtime_diag_prior_packed_raw;
        uint32_t fc = had_fault ? fault_rec.fault_count : 0u;
        kalico_log_emit(KALICO_LOG_LEVEL_WARN, KALICO_LOG_SUBSYS_RUNTIME,
                        KALICO_LOG_EVENT_RUNTIME_RT_PROGRESS, 0,
                        runtime_diag_prior_packed_raw, fc);

        // Last scheduler callback running before the freeze (addr2line target).
        kalico_log_emit(KALICO_LOG_LEVEL_WARN, KALICO_LOG_SUBSYS_RUNTIME,
                        KALICO_LOG_EVENT_RUNTIME_LAST_DISPATCH, 0,
                        saved_prior_last_dispatch_func,
                        saved_prior_last_dispatch_addr);

        // Cause-naming discriminators + the play-by-play ring. Only meaningful
        // when the prior-boot diag snapshot is present (else all-zero).
        if (prior_diag_present) {
            // Which engine ISR phase was executing (RT_PHASE_*) + ring churn.
            kalico_log_emit(KALICO_LOG_LEVEL_WARN, KALICO_LOG_SUBSYS_RUNTIME,
                            KALICO_LOG_EVENT_RUNTIME_ISR_PHASE, 0,
                            prior_diag.rt_isr_phase, prior_diag.ring_overflow);
            // Who hogged the CPU (contiguous-burst DWT spans) — the cause-namer:
            // a span >~2x the TIM5 period starves the foreground.
            kalico_log_emit(KALICO_LOG_LEVEL_WARN, KALICO_LOG_SUBSYS_RUNTIME,
                            KALICO_LOG_EVENT_RUNTIME_BLOCK_SOURCE, 0,
                            prior_diag.usb_burst_max_cyc,
                            prior_diag.stepout_burst_max_cyc);
            // Was the engine itself starved? TIM5 entry-to-entry vs sample period.
            kalico_log_emit(KALICO_LOG_LEVEL_WARN, KALICO_LOG_SUBSYS_RUNTIME,
                            KALICO_LOG_EVENT_RUNTIME_TIM5_IA, 0,
                            prior_diag.tim5_ia_min_cyc,
                            prior_diag.tim5_ia_max_cyc);

            // Play-by-play: replay the prior-boot diag event ring oldest-first
            // (head = oldest). Each entry rides the diag subsystem with the
            // event-code == its DIAG_EV_* tag (1:1 with log_codes.rs). The
            // entry's own seq/timestamp are dropped — emit order is chronology,
            // and the salient timing rides in a/b. Skips empty slots.
            uint32_t head = prior_diag.ring_head & DIAG_RING_MASK;
            for (uint32_t i = 0; i < DIAG_RING_LEN; i++) {
                uint32_t idx = (head + i) & DIAG_RING_MASK;
                uint8_t tag = prior_ring[idx].tag;
                if (tag == DIAG_EV_NONE)
                    continue;
                kalico_log_emit(diag_ring_tag_level(tag), KALICO_LOG_SUBSYS_DIAG,
                                tag, 0, prior_ring[idx].a, prior_ring[idx].b);
            }
        }
    }
}

// On-demand live diag dump (KALICO_DIAG_DUMP gcode → command_kalico_diag_dump).
// Sibling of kalico_diag_emit_prior_crash, but reads the LIVE diag/live_snap
// (this run) so hiccups surface in the structured store without a reset. All
// frames at debug level (informational snapshot; the diag_dump header event
// name distinguishes it from a prior-boot crash replay). Foreground-only.
void
kalico_diag_emit_live(void)
{
    // Coherent snapshot of the live ring — ISRs push to diag_ring concurrently,
    // so copy head + entries under one irq_save, then emit from the copy.
    irqstatus_t flag = irq_save();
    uint32_t head          = diag.ring_head & DIAG_RING_MASK;
    uint32_t ring_seq      = diag.ring_seq;
    uint32_t ring_overflow = diag.ring_overflow;
    for (uint32_t i = 0; i < DIAG_RING_LEN; i++) {
        dump_ring[i].tag       = diag_ring[i].tag;
        dump_ring[i]._pad0     = diag_ring[i]._pad0;
        dump_ring[i].seq       = diag_ring[i].seq;
        dump_ring[i].timestamp = diag_ring[i].timestamp;
        dump_ring[i].a         = diag_ring[i].a;
        dump_ring[i].b         = diag_ring[i].b;
    }
    irq_restore(flag);

    // Header: marks a live dump + how far the run has progressed. Scalar diag
    // reads below are aligned u32 (atomic) — minor staleness is acceptable.
    uint32_t now = timer_read_time();
    uint32_t uptime_us = boot_tick_initialized
        ? (uint32_t)((uint64_t)(now - boot_first_tick) * 1000000u / CONFIG_CLOCK_FREQ)
        : 0u;
    kalico_log_emit(KALICO_LOG_LEVEL_DEBUG, KALICO_LOG_SUBSYS_RUNTIME,
                    KALICO_LOG_EVENT_RUNTIME_DIAG_DUMP, 0, uptime_us, ring_seq);

    // Live cause discriminators (whole-run-so-far extremes).
    kalico_log_emit(KALICO_LOG_LEVEL_DEBUG, KALICO_LOG_SUBSYS_RUNTIME,
                    KALICO_LOG_EVENT_RUNTIME_ISR_PHASE, 0,
                    diag.rt_isr_phase, ring_overflow);
    kalico_log_emit(KALICO_LOG_LEVEL_DEBUG, KALICO_LOG_SUBSYS_RUNTIME,
                    KALICO_LOG_EVENT_RUNTIME_BLOCK_SOURCE, 0,
                    diag.usb_burst_max_cyc, diag.stepout_burst_max_cyc);
    kalico_log_emit(KALICO_LOG_LEVEL_DEBUG, KALICO_LOG_SUBSYS_RUNTIME,
                    KALICO_LOG_EVENT_RUNTIME_TIM5_IA, 0,
                    diag.tim5_ia_min_cyc, diag.tim5_ia_max_cyc);

    // Foreground-freeze (worst stall latched this run, if any).
    if (live_snap.worst_fg_stall_ticks) {
        kalico_log_emit(KALICO_LOG_LEVEL_DEBUG, KALICO_LOG_SUBSYS_RUNTIME,
                        KALICO_LOG_EVENT_RUNTIME_FG_FREEZE, 0,
                        live_snap.worst_fg_stall_pc,
                        live_snap.worst_fg_stall_ticks);
    }

    // Live ring play-by-play, oldest-first (head = oldest), skip empty slots.
    for (uint32_t i = 0; i < DIAG_RING_LEN; i++) {
        uint32_t idx = (head + i) & DIAG_RING_MASK;
        uint8_t tag = dump_ring[idx].tag;
        if (tag == DIAG_EV_NONE)
            continue;
        kalico_log_emit(diag_ring_tag_level(tag), KALICO_LOG_SUBSYS_DIAG,
                        tag, 0, dump_ring[idx].a, dump_ring[idx].b);
    }
}
