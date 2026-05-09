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

// Push a tagged record to the BKPSRAM event ring. IRQ-safe.
void diag_ring_push(uint8_t tag, uint32_t a, uint32_t b);

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

#ifdef __cplusplus
}
#endif

#endif // __GENERIC_FAULT_HANDLER_H
