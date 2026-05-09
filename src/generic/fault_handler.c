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

#if CONFIG_KALICO_RUNTIME
extern volatile uint8_t runtime_liveness_ok;
extern void *runtime_handle;
extern uint32_t runtime_handle_tick_counter(void *handle);
extern uint8_t  runtime_handle_status(void *handle);
#endif

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

// Liveness snapshot in AXI SRAM. Periodically refreshed while the system
// is healthy; on next boot, the most-recent values tell us what state the
// runtime engine was in just before the crash. `magic2` lets the boot
// distinguish "fresh power-on, garbage in SRAM" from "previous run wrote
// a real snapshot here".
#define LIVE_MAGIC 0x4C495645u  // "LIVE"

struct live_snapshot {
    uint32_t magic;
    uint32_t live;          // runtime_liveness_ok at sample time
    uint32_t engine_status; // runtime_handle_status (0=IDLE 1=RUNNING 2=DRAINED 3=FAULT)
    uint32_t tick_counter;  // runtime_handle_tick_counter
    uint32_t sample_time;   // timer_read_time() at sample
    uint32_t boot_count;    // bumped each boot, helps confirm fresh-vs-stale
    uint32_t last_engine_running_tick; // tick_counter sampled while RUNNING
    uint32_t samples_taken;
};

// Place in AXI SRAM on H7 (survives soft reset, not cleared by boot
// .bss memset). On other targets fall through to .bss — RAM contents
// still survive soft reset on Cortex-M, but the boot memset clears
// .bss before main runs, so the record would be lost. Move it to a
// `.noinit` section if/when other STM32 families need this.
// Place in backup SRAM (D3 domain) on H7 — survives IWDG / software /
// NRST resets. Requires PWR->CR2.BREN + RCC->AHB4ENR.BKPRAMEN to be
// set at init (see fault_handler_init below); without those the
// region reads as zero and writes are silently dropped.
#if CONFIG_MACH_STM32H7
__attribute__((section(".bkp_bss"), used))
#else
__attribute__((used))
#endif
static volatile struct fault_record fault_rec;

#if CONFIG_MACH_STM32H7
__attribute__((section(".bkp_bss"), used))
#else
__attribute__((used))
#endif
static volatile struct live_snapshot live_snap;

// =============================================================================
// Diagnostic counters + event ring — for the bridge-call stall investigation.
//
// The bug under test (`docs/superpowers/specs/2026-05-09-bridge-call-stall-*`)
// causes a ~500 ms USB-OUT NAK from the H7 during an `_do_enable` SPI burst
// concurrent with motion-bridge planner traffic. Existing firmware diagnostics
// don't observe what runs in that window — `runtime_status_drain` (which would
// emit) is itself starved.
//
// The strategy: keep monotonic counters in BKPSRAM updated from each IRQ /
// task entry, plus an event ring of "long IRQ" / "task gap" / "TX drop" /
// "engine state transition" records. The current run's emit task surfaces
// counter deltas at 10 Hz when foreground runs; the ring captures whatever
// happened during the windows it didn't. On next boot, both are dumped via
// the existing `fault_handler_report_task`.
//
// IRQ-safety: ring push uses `irq_save` / `irq_restore` (cpsid/cpsie). Cost
// in the IRQ path is ~10 cycles for the push window — negligible at 40 kHz.
// =============================================================================
#define DIAG_MAGIC      0x4449414Eu  // "DIAN" — diagnostic counters present
#define DIAG_RING_LEN   32           // power-of-two for cheap mask
#define DIAG_RING_MASK  (DIAG_RING_LEN - 1)

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
};

struct diag_event {
    uint8_t  tag;
    uint8_t  _pad0;
    uint16_t seq;          // monotonic — distinguishes wrap from no-events
    uint32_t timestamp;    // timer_read_time() at push
    uint32_t a;
    uint32_t b;
};

struct diag_counters {
    uint32_t magic;

    // IRQ counters. `cycles_*` are DWT cycles (520 MHz on H7 — 1us = 520
    // cycles). Both production and prior-run dumps use the same units;
    // emit converts to us for human readability.
    uint32_t tim5_irq_count;
    uint32_t tim5_irq_cycles_total;
    uint32_t tim5_irq_cycles_max;
    uint32_t otg_irq_count;
    uint32_t otg_irq_cycles_total;
    uint32_t otg_irq_cycles_max;

    // Foreground task heartbeats. `last_tick` is timer_read_time() at the
    // most recent task entry; max_gap_ticks is the largest observed gap
    // between consecutive entries (in timer ticks, same units as
    // timer_read_time()).
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

    // TX-drop counters. Investigation says these aren't firing on real
    // hardware, but cheap to confirm.
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
};

#if CONFIG_MACH_STM32H7
__attribute__((section(".bkp_bss"), used))
#else
__attribute__((used))
#endif
static volatile struct diag_counters diag;

#if CONFIG_MACH_STM32H7
__attribute__((section(".bkp_bss"), used))
#else
__attribute__((used))
#endif
static volatile struct diag_event diag_ring[DIAG_RING_LEN];

// Saved snapshot of prior-run counters + ring, populated once on boot and
// emitted by the report task. Lives in .bss (zero on reset, populated from
// the BKPSRAM diag struct before we overwrite it).
static struct diag_counters prior_diag;
static struct diag_event    prior_ring[DIAG_RING_LEN];
static uint32_t             prior_diag_present;
static uint32_t             prior_ring_emit_idx;

// IRQ-safe push to the diag ring. Used from foreground AND IRQ context, so
// the head/seq update is protected with irq_save. The struct stores are
// volatile — compiler can't reorder them across the irq_save/restore pair.
void
diag_ring_push(uint8_t tag, uint32_t a, uint32_t b)
{
#if CONFIG_KALICO_RUNTIME
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
    irq_restore(flag);
#else
    (void)tag; (void)a; (void)b;
#endif
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
#if CONFIG_KALICO_RUNTIME
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
#else
    (void)calls; (void)last_tick; (void)max_gap;
    (void)threshold_ticks; (void)event_tag;
#endif
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
void
diag_tim5_account(uint32_t enter_cycles, uint32_t exit_cycles)
{
#if CONFIG_KALICO_RUNTIME
    uint32_t dur = exit_cycles - enter_cycles;
    diag.tim5_irq_count++;
    diag.tim5_irq_cycles_total += dur;
    if (dur > diag.tim5_irq_cycles_max)
        diag.tim5_irq_cycles_max = dur;
    // Threshold: 50us at 520 MHz = 26000 cycles. Steady-state TIM5 is ~3000
    // cycles. 50us ≈ 8x normal — a real outlier worth recording.
    if (dur > 26000u)
        diag_ring_push(DIAG_EV_TIM5_LONG, dur, enter_cycles);
#else
    (void)enter_cycles; (void)exit_cycles;
#endif
}

void
diag_otg_account(uint32_t enter_cycles, uint32_t exit_cycles)
{
#if CONFIG_KALICO_RUNTIME
    uint32_t dur = exit_cycles - enter_cycles;
    diag.otg_irq_count++;
    diag.otg_irq_cycles_total += dur;
    if (dur > diag.otg_irq_cycles_max)
        diag.otg_irq_cycles_max = dur;
    if (dur > 26000u)
        diag_ring_push(DIAG_EV_OTG_LONG, dur, enter_cycles);
#else
    (void)enter_cycles; (void)exit_cycles;
#endif
}

// =============================================================================
// Accessors — invoked from the periodic emit in runtime_status_drain to read
// counter snapshots. Use a brief irq_save to take a coherent snapshot of
// counters that change in IRQ context.
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
#if CONFIG_KALICO_RUNTIME
    irqstatus_t flag = irq_save();
    s->tim5_n      = diag.tim5_irq_count;
    s->tim5_total  = diag.tim5_irq_cycles_total;
    s->tim5_max    = diag.tim5_irq_cycles_max;
    s->otg_n       = diag.otg_irq_count;
    s->otg_total   = diag.otg_irq_cycles_total;
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
    // Reset max trackers for next interval (so emits show per-interval peaks
    // instead of all-time peaks). Counts and totals are cumulative.
    diag.tim5_irq_cycles_max = 0;
    diag.otg_irq_cycles_max  = 0;
    diag.usb_out_max_gap_ticks = 0;
    diag.usb_in_max_gap_ticks  = 0;
    diag.runtime_drain_max_gap_ticks = 0;
    diag.runtime_status_max_gap_ticks = 0;
    irq_restore(flag);
#else
    memset(s, 0, sizeof(*s));
#endif
}

// Heartbeat slot accessors — exposed so other compilation units can pass
// pointers into our BKPSRAM struct without taking direct addresses of
// volatile members across translation units (which compilers warn about).
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

// Drop-event helpers — called from the TX-buffer-full early-return paths.
void
diag_record_tx_drop_kalico(uint32_t len, uint32_t tpos)
{
#if CONFIG_KALICO_RUNTIME
    diag.tx_drops_kalico++;
    diag.tx_drops_kalico_last_len = len;
    diag_ring_push(DIAG_EV_TX_DROP_KAL, len, tpos);
#else
    (void)len; (void)tpos;
#endif
}

void
diag_record_tx_drop_klipper(uint32_t max_size, uint32_t tpos)
{
#if CONFIG_KALICO_RUNTIME
    diag.tx_drops_klipper++;
    diag.tx_drops_klipper_last_max = max_size;
    diag_ring_push(DIAG_EV_TX_DROP_KLP, max_size, tpos);
#else
    (void)max_size; (void)tpos;
#endif
}

void
diag_record_engine_xition(uint8_t prev, uint8_t cur, uint32_t samples_taken)
{
#if CONFIG_KALICO_RUNTIME
    diag_ring_push(DIAG_EV_ENGINE_XITION,
                   ((uint32_t)prev << 8) | (uint32_t)cur,
                   samples_taken);
#else
    (void)prev; (void)cur; (void)samples_taken;
#endif
}

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

    // Fault status registers.
    fault_rec.cfsr  = SCB->CFSR;
    fault_rec.hfsr  = SCB->HFSR;
    fault_rec.dfsr  = SCB->DFSR;
    fault_rec.bfar  = SCB->BFAR;
    fault_rec.mmfar = SCB->MMFAR;
    fault_rec.afsr  = SCB->AFSR;
    fault_rec.shcsr = SCB->SHCSR;

    fault_rec.exc_kind = kind;
    if (fault_rec.magic != FAULT_MAGIC)
        fault_rec.fault_count = 0;
    fault_rec.fault_count++;
    fault_rec.magic = FAULT_MAGIC;

    // Make sure the record is actually written before we reset.
    __DSB();

    // Soft reset.
    NVIC_SystemReset();

    for (;;);
}

#include "armcm_boot.h"  // DECL_ARMCM_IRQ

// Naked trampolines: extract the active stack pointer (MSP or PSP based on
// EXC_RETURN bit 2) and the EXC_RETURN value, then tail into the C handler.
// This must be inline asm in a naked function so the compiler doesn't push
// anything onto the stack before we sample SP.
#define FAULT_TRAMPOLINE(NAME, KIND, IRQ_NUM)                           \
    void __attribute__((naked, used)) NAME(void)                        \
    {                                                                   \
        asm volatile (                                                  \
            "tst lr, #4\n\t"                                            \
            "ite eq\n\t"                                                \
            "mrseq r1, msp\n\t"                                         \
            "mrsne r1, psp\n\t"                                         \
            "mov r0, %0\n\t"                                            \
            "mov r2, lr\n\t"                                            \
            "b fault_capture_and_reset\n\t"                             \
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

// On boot, enable the configurable fault exceptions so they don't all
// escalate into HardFault (HardFault still catches them too if the
// configurable handlers escalate). Also enable divide-by-zero and
// unaligned-access trapping so we get UsageFault on those.
void
fault_handler_init(void)
{
#if (__CORTEX_M >= 3)
    SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk
                | SCB_SHCSR_BUSFAULTENA_Msk
                | SCB_SHCSR_MEMFAULTENA_Msk;
    SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;  // trap divide by zero
    // Don't enable unalign trap — half-word/word unaligned loads are common.
#endif
#if CONFIG_MACH_STM32H7
    // Enable backup SRAM. Steps from RM0468 §6.6.7:
    // 1. Clock the BKPRAM peripheral (AHB4ENR.BKPRAMEN).
    // 2. Disable backup-domain write protection (PWR->CR1.DBP).
    // 3. Enable the backup regulator (PWR->CR2.BREN) so the contents
    //    are retained during VBAT-only operation. Even without VBAT
    //    connected, the SRAM is RAM-backed and survives any reset
    //    short of a full power cycle, which is what we need.
    RCC->AHB4ENR |= RCC_AHB4ENR_BKPRAMEN;
    PWR->CR1 |= PWR_CR1_DBP;
    PWR->CR2 |= PWR_CR2_BREN;
    // Wait briefly for the regulator-ready flag — the snapshot writes
    // happen well after init runs, so a tight poll here is fine.
    {
        volatile int spin = 0;
        while (!(PWR->CR2 & PWR_CR2_BRRDY) && spin < 100000) spin++;
    }
#endif
}
DECL_INIT(fault_handler_init);

// Re-emit the boot diagnostic and (if present) the prior-fault record on
// a slow timer so it survives klippy reconnect. Most embedded silent-
// reset bugs are not caught by HardFault — they're watchdog resets, BOR,
// PMU events, etc — so even when our handler never ran, the RCC reset-
// cause flags still tell us how the chip got reset. We capture them at
// init and report every ~2s of MCU time for the first 60s of each boot.
#include "board/misc.h"   // timer_read_time, timer_from_us

static uint32_t boot_first_tick;
static uint32_t boot_tick_initialized;
static uint32_t last_emit_tick;
static uint32_t emits_done;
static uint32_t reset_cause_snapshot;
static uint32_t reset_cause_raw;
// Cached prior-run snapshot, captured at boot before we overwrite live_snap
// with this-run state. Static .bss = zero on each boot so the cache is
// derived once and reused.
static uint32_t prior_live_present_at_boot;
static uint32_t saved_prior_live;
static uint32_t saved_prior_engine;
static uint32_t saved_prior_tick;
static uint32_t saved_prior_last_run_tick;
static uint32_t saved_prior_samples;

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
        // Snapshot the prior-run live_snap BEFORE this run starts
        // overwriting it on subsequent task calls.
        if (live_snap.magic == LIVE_MAGIC) {
            prior_live_present_at_boot = 1;
            saved_prior_live          = live_snap.live;
            saved_prior_engine        = live_snap.engine_status;
            saved_prior_tick          = live_snap.tick_counter;
            saved_prior_last_run_tick = live_snap.last_engine_running_tick;
            saved_prior_samples       = live_snap.samples_taken;
        }
        live_snap.samples_taken = 0;  // reset for this run

        // Snapshot the prior-run diag counters + ring before the new run
        // overwrites them. Both live in BKPSRAM on H7; on F4 the .bss reset
        // already wiped them by the time we get here, so `diag.magic` will
        // not equal DIAG_MAGIC and the snapshot is skipped.
        if (diag.magic == DIAG_MAGIC) {
            prior_diag_present = 1;
            // Memcpy through volatile via field-by-field copy. The struct
            // is small enough (~128 B) that this is acceptable cost.
            prior_diag.magic                = diag.magic;
            prior_diag.tim5_irq_count       = diag.tim5_irq_count;
            prior_diag.tim5_irq_cycles_total = diag.tim5_irq_cycles_total;
            prior_diag.tim5_irq_cycles_max  = diag.tim5_irq_cycles_max;
            prior_diag.otg_irq_count        = diag.otg_irq_count;
            prior_diag.otg_irq_cycles_total = diag.otg_irq_cycles_total;
            prior_diag.otg_irq_cycles_max   = diag.otg_irq_cycles_max;
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
            // Capture the ring contents into a non-volatile copy so the
            // emit loop below has a stable snapshot to walk.
            for (uint32_t i = 0; i < DIAG_RING_LEN; i++) {
                prior_ring[i].tag       = diag_ring[i].tag;
                prior_ring[i]._pad0     = diag_ring[i]._pad0;
                prior_ring[i].seq       = diag_ring[i].seq;
                prior_ring[i].timestamp = diag_ring[i].timestamp;
                prior_ring[i].a         = diag_ring[i].a;
                prior_ring[i].b         = diag_ring[i].b;
            }
        }
        // Reset BKPSRAM diag for the new run. Set magic so next boot
        // recognises a valid record exists.
        memset((void *)&diag, 0, sizeof(diag));
        diag.magic = DIAG_MAGIC;
        diag.boot_count = prior_diag_present ? (prior_diag.boot_count + 1) : 1;
        // Zero the ring too — old entries are in prior_ring now.
        for (uint32_t i = 0; i < DIAG_RING_LEN; i++) {
            diag_ring[i].tag = DIAG_EV_NONE;
            diag_ring[i].seq = 0;
            diag_ring[i].timestamp = 0;
            diag_ring[i].a = 0;
            diag_ring[i].b = 0;
        }
        return;
    }
    // Refresh the liveness snapshot every task call (once per scheduler
    // iteration) so the snapshot is fresh when the IWDG fires. Saving
    // here rather than gated on `emits_done` ensures we capture state
    // right up to the crash, not just every 2 s.
#if CONFIG_KALICO_RUNTIME
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
#endif
    // Emit every 2 seconds for the first 60 seconds of boot.
    if (emits_done >= 30)
        return;
    uint32_t elapsed = now - last_emit_tick;
    if (elapsed < timer_from_us(2000000))
        return;
    last_emit_tick = now;
    uint32_t since_boot_us = (uint32_t)((uint64_t)(now - boot_first_tick)
                                        * 1000000u
                                        / CONFIG_CLOCK_FREQ);
    // Use free-form `%u` (no `name=%u`) so the parser flags these as
    // free-form outputs and the decoder populates `#msg` with the
    // interpolated string. With `name=%u` syntax the decoder routes
    // structured-style by message name and never builds `#msg`, leaving
    // klippy's handle_output logging an empty line.
    output("boot_diag emit %u since_us %u rcc %u prior %u live %u engine %u tick %u",
           emits_done, since_boot_us, reset_cause_raw,
           (uint32_t)(fault_rec.magic == FAULT_MAGIC),
           live_snap.live, live_snap.engine_status, live_snap.tick_counter);
    // Re-emit the prior-run snapshot every cycle for the first 30 emits
    // to ensure delivery survives any USB enumeration / klippy reconnect
    // timing.
    if (prior_live_present_at_boot) {
        output("prior_live live %u engine %u tick %u last_run_tick %u samples %u",
               saved_prior_live, saved_prior_engine,
               saved_prior_tick, saved_prior_last_run_tick,
               saved_prior_samples);
    }
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

    // Prior-run diag dump: one summary line each emit cycle, plus a few
    // ring entries per cycle (throttled to avoid flooding `transmit_buf`
    // with 32 entries in one go). Across 30 emit cycles we get plenty
    // of redundancy for the host to pick up at least one full pass.
    if (prior_diag_present) {
        output("prior_diag_summary boot %u tim5_n %u tim5_max_cyc %u tim5_total_cyc %u"
               " otg_n %u otg_max_cyc %u otg_total_cyc %u",
               prior_diag.boot_count,
               prior_diag.tim5_irq_count,
               prior_diag.tim5_irq_cycles_max,
               prior_diag.tim5_irq_cycles_total,
               prior_diag.otg_irq_count,
               prior_diag.otg_irq_cycles_max,
               prior_diag.otg_irq_cycles_total);
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
        // Walk the ring in stored order (head = next write slot, so the
        // OLDEST entry is at index `head`). Emit up to 4 per cycle so 30
        // cycles cover all 32 entries with margin.
        const uint32_t per_cycle = 4;
        uint32_t start = prior_ring_emit_idx;
        uint32_t end = start + per_cycle;
        if (end > DIAG_RING_LEN)
            end = DIAG_RING_LEN;
        uint32_t head = prior_diag.ring_head & DIAG_RING_MASK;
        for (uint32_t i = start; i < end; i++) {
            // Index into the ring in chronological order: oldest is at
            // `head`, newest is at `(head - 1) & MASK`.
            uint32_t idx = (head + i) & DIAG_RING_MASK;
            if (prior_ring[idx].tag != DIAG_EV_NONE) {
                output("prior_diag_ring i %u tag %u seq %u ts %u a %u b %u",
                       i,
                       prior_ring[idx].tag,
                       prior_ring[idx].seq,
                       prior_ring[idx].timestamp,
                       prior_ring[idx].a,
                       prior_ring[idx].b);
            }
        }
        prior_ring_emit_idx = end;
        if (prior_ring_emit_idx >= DIAG_RING_LEN) {
            // Wrap so subsequent cycles re-emit the same content (host
            // reconnect race tolerance).
            prior_ring_emit_idx = 0;
        }
    }

    emits_done++;
}
DECL_TASK(fault_handler_report_task);
