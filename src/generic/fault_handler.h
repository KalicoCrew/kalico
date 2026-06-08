#ifndef __GENERIC_FAULT_HANDLER_H
#define __GENERIC_FAULT_HANDLER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Diag ring event tags — must match the enum in fault_handler.c.
#define DIAG_EV_NONE          0
#define DIAG_EV_TIM5_LONG     1
#define DIAG_EV_OTG_LONG      2
#define DIAG_EV_USB_OUT_GAP   3
#define DIAG_EV_USB_IN_GAP    4
#define DIAG_EV_TX_DROP_KAL   5
#define DIAG_EV_TX_DROP_KLP   6
#define DIAG_EV_ENGINE_XITION 7
// a = (uint32_t)last_error (i32 cast), b = fault_detail.
#define DIAG_EV_RUST_FAULT    8

// ISR-phase breadcrumb; value at IWDG reset names the hung phase.
// MUST match rust/runtime/src/isr_phase.rs.
#define RT_PHASE_IDLE          0
#define RT_PHASE_ISR_ENTER     1
#define RT_PHASE_WIDEN         2
#define RT_PHASE_GUARD         3
#define RT_PHASE_TICK          4
#define RT_PHASE_WALK          5
#define RT_PHASE_MONOMIAL      6
#define RT_PHASE_HORNER        7
#define RT_PHASE_STEP_ENQ      8
#define RT_PHASE_ISR_EXIT      9
#define RT_PHASE_STEPOUT_ENTER 10
#define RT_PHASE_STEPOUT_POP   11
#define RT_PHASE_STEPOUT_EMIT  12
#define RT_PHASE_STEPOUT_EXIT  13

void diag_ring_push(uint8_t tag, uint32_t a, uint32_t b);

// Call once from the post-host-connect path (host mcu-log hook must be up).
void kalico_diag_emit_prior_crash(void);

// On-demand live snapshot (KALICO_DIAG_DUMP). Foreground-only.
void kalico_diag_emit_live(void);

// event_tag=0 suppresses event emission (counters still update).
void diag_task_heartbeat(volatile uint32_t *calls,
                         volatile uint32_t *last_tick,
                         volatile uint32_t *max_gap,
                         uint32_t threshold_ticks,
                         uint8_t event_tag);

void diag_tim5_account(uint32_t enter_cycles, uint32_t exit_cycles);
void diag_otg_account(uint32_t enter_cycles, uint32_t exit_cycles);

void diag_runtime_tick_account(uint32_t cycles);

// Rust-only callers — must keep external linkage through -fwhole-program LTO.
void diag_walk_account(uint32_t cycles);
void diag_monomial_account(uint32_t cycles);

void runtime_set_isr_phase(uint32_t phase);

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

void diag_record_tx_drop_kalico(uint32_t len, uint32_t tpos);
void diag_record_tx_drop_klipper(uint32_t max_size, uint32_t tpos);
void diag_record_engine_xition(uint8_t prev, uint8_t cur,
                               uint32_t samples_taken);

// Cycles are DWT cycles (520 MHz on H7); gaps are timer ticks (CONFIG_CLOCK_FREQ).
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

// Resets per-interval max trackers; coherent under irq_save.
void diag_take_snapshot(struct diag_snapshot *s);

volatile uint32_t *diag_slot_otg_rxflvl(void);
volatile uint32_t *diag_slot_otg_iepint(void);
volatile uint32_t *diag_slot_otg_other(void);
volatile uint32_t *diag_slot_otg_other_sts(void);
volatile uint32_t *diag_slot_notify_bulk_out(void);
volatile uint32_t *diag_slot_task_invoke(void);
volatile uint32_t *diag_slot_read_zero(void);
volatile uint32_t *diag_slot_read_data(void);

void diag_snapshot_otg_regs(uint32_t gintmsk, uint32_t gintsts);

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

uint32_t diag_get_tim5_count(void);

uint32_t diag_get_rt_tick_count(void);
uint32_t diag_get_rt_tick_cycles_max(void);

uint32_t diag_get_tx_drops_kalico(void);
uint32_t diag_get_tx_drops_klipper(void);

#ifdef __cplusplus
}
#endif

#endif
