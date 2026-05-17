// Linux-host stubs for the diag/fault-handler API declared in
// src/generic/fault_handler.h. The real implementation in
// src/generic/fault_handler.c is Cortex-M / STM32 specific (custom
// link sections, SCB cache ops, exception handlers) and cannot be
// compiled for the Linux MCU build. The host sim doesn't need the
// wedge counters or fault-record persistence — provide no-ops so
// runtime_tick.c and friends link cleanly.

#include <stdint.h>
#include "generic/fault_handler.h"

static volatile uint32_t stub_zero;

void diag_ring_push(uint8_t tag, uint32_t a, uint32_t b)
{
    (void)tag; (void)a; (void)b;
}

void diag_task_heartbeat(volatile uint32_t *calls,
                         volatile uint32_t *last_tick,
                         volatile uint32_t *max_gap,
                         uint32_t threshold_ticks,
                         uint8_t event_tag)
{
    (void)calls; (void)last_tick; (void)max_gap;
    (void)threshold_ticks; (void)event_tag;
}

void diag_tim5_account(uint32_t enter_cycles, uint32_t exit_cycles)
{
    (void)enter_cycles; (void)exit_cycles;
}

void diag_otg_account(uint32_t enter_cycles, uint32_t exit_cycles)
{
    (void)enter_cycles; (void)exit_cycles;
}

void diag_runtime_tick_account(uint32_t cycles)
{
    (void)cycles;
}

#define DIAG_SLOT_STUB(name) \
    volatile uint32_t *diag_slot_##name(void) { return &stub_zero; }

DIAG_SLOT_STUB(usb_out_calls)
DIAG_SLOT_STUB(usb_out_last_tick)
DIAG_SLOT_STUB(usb_out_max_gap)
DIAG_SLOT_STUB(usb_in_calls)
DIAG_SLOT_STUB(usb_in_last_tick)
DIAG_SLOT_STUB(usb_in_max_gap)
DIAG_SLOT_STUB(rt_drain_calls)
DIAG_SLOT_STUB(rt_drain_last_tick)
DIAG_SLOT_STUB(rt_drain_max_gap)
DIAG_SLOT_STUB(rt_status_calls)
DIAG_SLOT_STUB(rt_status_last_tick)
DIAG_SLOT_STUB(rt_status_max_gap)
DIAG_SLOT_STUB(otg_rxflvl)
DIAG_SLOT_STUB(otg_iepint)
DIAG_SLOT_STUB(otg_other)
DIAG_SLOT_STUB(otg_other_sts)
DIAG_SLOT_STUB(notify_bulk_out)
DIAG_SLOT_STUB(task_invoke)
DIAG_SLOT_STUB(read_zero)
DIAG_SLOT_STUB(read_data)
DIAG_SLOT_STUB(enable_rx)
DIAG_SLOT_STUB(enable_rx_rearm)
DIAG_SLOT_STUB(peek_empty)
DIAG_SLOT_STUB(peek_data)

void diag_record_tx_drop_kalico(uint32_t len, uint32_t tpos)
{
    (void)len; (void)tpos;
}

void diag_record_tx_drop_klipper(uint32_t max_size, uint32_t tpos)
{
    (void)max_size; (void)tpos;
}

void diag_record_engine_xition(uint8_t prev, uint8_t cur,
                               uint32_t samples_taken)
{
    (void)prev; (void)cur; (void)samples_taken;
}

void diag_take_snapshot(struct diag_snapshot *s)
{
    if (s) {
        for (uint32_t *p = (uint32_t *)s;
             p < (uint32_t *)(s + 1); p++) {
            *p = 0;
        }
    }
}

void diag_snapshot_otg_regs(uint32_t gintmsk, uint32_t gintsts)
{
    (void)gintmsk; (void)gintsts;
}

void diag_snapshot_out_ep(uint32_t doepctl, uint32_t doeptsiz, uint32_t doepint)
{
    (void)doepctl; (void)doeptsiz; (void)doepint;
}

#define DIAG_GET_STUB(name) \
    uint32_t diag_get_##name(void) { return 0; }

DIAG_GET_STUB(otg_rxflvl)
DIAG_GET_STUB(otg_iepint)
DIAG_GET_STUB(otg_other)
DIAG_GET_STUB(otg_other_sts)
DIAG_GET_STUB(notify_bulk_out)
DIAG_GET_STUB(task_invoke)
DIAG_GET_STUB(read_zero)
DIAG_GET_STUB(read_data)
DIAG_GET_STUB(otg_gintmsk_now)
DIAG_GET_STUB(otg_gintsts_now)
DIAG_GET_STUB(out_ep_doepctl)
DIAG_GET_STUB(out_ep_doeptsiz)
DIAG_GET_STUB(out_ep_doepint)
DIAG_GET_STUB(enable_rx_n)
DIAG_GET_STUB(enable_rx_rearm)
DIAG_GET_STUB(peek_empty)
DIAG_GET_STUB(peek_data)
DIAG_GET_STUB(tim5_count)
DIAG_GET_STUB(tx_drops_kalico)
DIAG_GET_STUB(tx_drops_klipper)
