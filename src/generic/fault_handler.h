// Public API for the fault handler / diagnostic counter module.
// See fault_handler.c for the data layout and IRQ-safety notes.
#ifndef __GENERIC_FAULT_HANDLER_H
#define __GENERIC_FAULT_HANDLER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Diag ring event tags (must match the enum in fault_handler.c).
#define DIAG_EV_NONE          0
#define DIAG_EV_TIM5_LONG     1
#define DIAG_EV_OTG_LONG      2
#define DIAG_EV_USB_OUT_GAP   3
#define DIAG_EV_USB_IN_GAP    4
#define DIAG_EV_TX_DROP_KAL   5
#define DIAG_EV_TX_DROP_KLP   6
#define DIAG_EV_ENGINE_XITION 7
// Rust-raised runtime fault captured at the last_error escalation point in
// runtime_drain (runtime_tick.c). Stored in the BKPSRAM ring so the next
// boot's prior_diag_ring dump emits it even if the USB-CDC frame was lost.
// a = (uint32_t)last_error (i32 cast; negative fault codes wrap as expected
//     because the host reads them back as i32), b = fault_detail.
#define DIAG_EV_RUST_FAULT    8

// ISR-phase breadcrumb values. Written by the Rust motion ISR via
// runtime_set_isr_phase(); value at IWDG reset names the hung phase.
// MUST match rust/runtime/src/isr_phase.rs.
#define RT_PHASE_IDLE          0   // between ticks (foreground running)
#define RT_PHASE_ISR_ENTER     1   // TIM5 Rust ISR body entered
#define RT_PHASE_WIDEN         2   // clock widen + publish_widened_now
#define RT_PHASE_GUARD         3   // inter-arrival guard window
#define RT_PHASE_TICK          4   // engine.tick() entered (pre-walk)
#define RT_PHASE_WALK          5   // get_piece_for_time ring-walk
#define RT_PHASE_MONOMIAL      6   // arm_and_load / to_monomial (cold-load)
#define RT_PHASE_HORNER        7   // eval_horner
#define RT_PHASE_STEP_ENQ      8   // step enqueue / kick_per_axis_timer
#define RT_PHASE_ISR_EXIT      9   // TIM5 Rust ISR body returning
#define RT_PHASE_STEPOUT_ENTER 10  // kalico_step_output_event entered
#define RT_PHASE_STEPOUT_POP   11  // inside C queue_peek/pop (.axi_bss SPSC)
#define RT_PHASE_STEPOUT_EMIT  12  // runtime_emit_step_pulses
#define RT_PHASE_STEPOUT_EXIT  13  // step-output ISR returning

// Push a tagged record to the BKPSRAM event ring. IRQ-safe.
void diag_ring_push(uint8_t tag, uint32_t a, uint32_t b);

// Re-emit the prior-boot crash summary (reset cause, CPU fault, foreground
// freeze) through the structured-log path (KALICO_MSG_LOG). Call once from the
// post-host-connect path (the host's mcu-log hook must be installed first).
void kalico_diag_emit_prior_crash(void);

// Update a task-call heartbeat. Pass `event_tag=0` to suppress event
// emission (counters still update).
void diag_task_heartbeat(volatile uint32_t *calls,
                         volatile uint32_t *last_tick,
                         volatile uint32_t *max_gap,
                         uint32_t threshold_ticks,
                         uint8_t event_tag);

// IRQ accounting helpers. Call at IRQ exit with the DWT->CYCCNT values
// captured at IRQ entry / exit.
void diag_tim5_account(uint32_t enter_cycles, uint32_t exit_cycles);
void diag_otg_account(uint32_t enter_cycles, uint32_t exit_cycles);

// Histogram accounting for the runtime_handle_tick subwindow alone. Call
// from the TIM5 ISR with the already-computed `after - before` cycle delta.
void diag_runtime_tick_account(uint32_t cycles);

// Per-phase DWT accounting called from the Rust motion ISR. Rust-only callers —
// must keep external linkage through LTO (-fwhole-program).
void diag_walk_account(uint32_t cycles);
void diag_monomial_account(uint32_t cycles);

// Write the ISR-phase breadcrumb. Rust-only caller — must survive LTO.
void runtime_set_isr_phase(uint32_t phase);

// Heartbeat slot accessors — pointer to the BKPSRAM struct member, suitable
// to pass into diag_task_heartbeat. Each function names the task slot.
volatile uint32_t *diag_slot_usb_out_calls(void);
volatile uint32_t *diag_slot_usb_out_last_tick(void);
volatile uint32_t *diag_slot_usb_out_max_gap(void);
volatile uint32_t *diag_slot_usb_in_calls(void);
volatile uint32_t *diag_slot_usb_in_last_tick(void);
volatile uint32_t *diag_slot_usb_in_max_gap(void);
volatile uint32_t *diag_slot_rt_drain_calls(void);
volatile uint32_t *diag_slot_rt_drain_last_tick(void);
volatile uint32_t *diag_slot_rt_drain_max_gap(void);
volatile uint32_t *diag_slot_rt_status_calls(void);
volatile uint32_t *diag_slot_rt_status_last_tick(void);
volatile uint32_t *diag_slot_rt_status_max_gap(void);

// TX drop / engine xition recorders.
void diag_record_tx_drop_kalico(uint32_t len, uint32_t tpos);
void diag_record_tx_drop_klipper(uint32_t max_size, uint32_t tpos);
void diag_record_engine_xition(uint8_t prev, uint8_t cur,
                               uint32_t samples_taken);

// Snapshot returned by diag_take_snapshot. Cycles are DWT cycles (520 MHz
// on H7); gaps are timer ticks (CONFIG_CLOCK_FREQ).
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

// Snapshot the current diag counters and reset per-interval max trackers.
// Coherent under IRQ via brief irq_save.
void diag_take_snapshot(struct diag_snapshot *s);

// Round 2 — wedge instrumentation. Callers increment via the returned
// pointer (`(*p)++`); the pointer aliases a volatile member of the
// BKPSRAM-resident `diag` struct.
volatile uint32_t *diag_slot_otg_rxflvl(void);
volatile uint32_t *diag_slot_otg_iepint(void);
volatile uint32_t *diag_slot_otg_other(void);
volatile uint32_t *diag_slot_otg_other_sts(void);
volatile uint32_t *diag_slot_notify_bulk_out(void);
volatile uint32_t *diag_slot_task_invoke(void);
volatile uint32_t *diag_slot_read_zero(void);
volatile uint32_t *diag_slot_read_data(void);

// Snapshot OTG live registers (called from foreground in periodic emit).
void diag_snapshot_otg_regs(uint32_t gintmsk, uint32_t gintsts);

// Reads of the wedge counters and OTG-register snapshots.
uint32_t diag_get_otg_rxflvl(void);
uint32_t diag_get_otg_iepint(void);
uint32_t diag_get_otg_other(void);
uint32_t diag_get_otg_other_sts(void);
uint32_t diag_get_notify_bulk_out(void);
uint32_t diag_get_task_invoke(void);
uint32_t diag_get_read_zero(void);
uint32_t diag_get_read_data(void);
uint32_t diag_get_otg_gintmsk_now(void);
uint32_t diag_get_otg_gintsts_now(void);

// Round 3 — OUT EP register snapshot + enable_rx counters.
volatile uint32_t *diag_slot_enable_rx(void);
volatile uint32_t *diag_slot_enable_rx_rearm(void);
volatile uint32_t *diag_slot_peek_empty(void);
volatile uint32_t *diag_slot_peek_data(void);
void diag_snapshot_out_ep(uint32_t doepctl, uint32_t doeptsiz, uint32_t doepint);
uint32_t diag_get_out_ep_doepctl(void);
uint32_t diag_get_out_ep_doeptsiz(void);
uint32_t diag_get_out_ep_doepint(void);
uint32_t diag_get_enable_rx_n(void);
uint32_t diag_get_enable_rx_rearm(void);
uint32_t diag_get_peek_empty(void);
uint32_t diag_get_peek_data(void);

// LIVE counter accessor for TIM5 ISR fires — exposed for the 2026-05-17
// "F4 retire stall" investigation (fault_detail tag 0xF7). If 0 while
// current_segment_id > 0, TIM5 ISR is not firing → runtime_modulated_tick
// cannot retire queued segments.
uint32_t diag_get_tim5_count(void);

// LIVE accessors for the runtime_tick subwindow inside TIM5_IRQHandler —
// exposed for the 2026-05-21 "TIM5 fires but engine.tick_counter stays 0"
// investigation (fault_detail tags 0xE4/0xE5). If rt_tick_count < tim5_count
// the `if (runtime_handle)` early-skip in TIM5_IRQHandler is firing on every
// fire (runtime_handle null inside the IRQ context). If rt_tick_count ==
// tim5_count but rt_tick_cycles_max is tiny (~10 cycles), kalico_runtime_-
// tick_sample is being called but early-returning before isr_sample_tick.
uint32_t diag_get_rt_tick_count(void);
uint32_t diag_get_rt_tick_cycles_max(void);

// LIVE counter accessors for TX-side drops — kalico-native frame emits
// silently drop the frame when transmit_buf is full. Useful for diagnosing
// dropped StatusHeartbeat / FaultEvent frames under USB-CDC TX congestion.
uint32_t diag_get_tx_drops_kalico(void);
uint32_t diag_get_tx_drops_klipper(void);

#ifdef __cplusplus
}
#endif

#endif // __GENERIC_FAULT_HANDLER_H
