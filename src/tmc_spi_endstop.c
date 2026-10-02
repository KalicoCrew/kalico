// Sensorless homing by polling the tmc stallguard flag over spi
//
// Copyright (C) 2026  Rogerio Goncalves <rogerlz@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memset
#include "basecmd.h" // oid_alloc
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_read_time
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_TASK
#include "spicmds.h" // spidev_transfer
#include "trsync.h" // trsync_do_trigger

#define MAX_DATA_LEN 40
#define SPI_STATUS_SG2 0x04

struct tmc_spi_endstop {
    struct timer timer;
    uint32_t rest_ticks, trigger_clock;
    struct spidev_s *spi;
    struct trsync *ts;
    uint8_t pending, data_len, status_pos, trigger_reason;
};

static struct task_wake tmc_spi_endstop_wake;

static uint_fast8_t
tmc_spi_endstop_event(struct timer *t)
{
    struct tmc_spi_endstop *e = container_of(t, struct tmc_spi_endstop, timer);
    e->pending = 1;
    sched_wake_task(&tmc_spi_endstop_wake);
    e->timer.waketime += e->rest_ticks;
    return SF_RESCHEDULE;
}

void
command_config_tmc_spi_endstop(uint32_t *args)
{
    uint8_t data_len = args[2], status_pos = args[3];
    if (data_len > MAX_DATA_LEN || status_pos >= data_len)
        shutdown("Invalid tmc_spi_endstop config");
    struct tmc_spi_endstop *e = oid_alloc(
        args[0], command_config_tmc_spi_endstop, sizeof(*e));
    e->timer.func = tmc_spi_endstop_event;
    e->spi = spidev_oid_lookup(args[1]);
    e->data_len = data_len;
    e->status_pos = status_pos;
}
DECL_COMMAND(command_config_tmc_spi_endstop,
             "config_tmc_spi_endstop oid=%c spi_oid=%c data_len=%c"
             " status_pos=%c");

void
command_tmc_spi_endstop_home(uint32_t *args)
{
    struct tmc_spi_endstop *e = oid_lookup(
        args[0], command_config_tmc_spi_endstop);
    sched_del_timer(&e->timer);
    e->pending = 0;
    e->rest_ticks = args[2];
    if (!e->rest_ticks) {
        e->ts = NULL;
        return;
    }
    e->timer.waketime = e->trigger_clock = args[1];
    e->ts = trsync_oid_lookup(args[3]);
    e->trigger_reason = args[4];
    sched_add_timer(&e->timer);
}
DECL_COMMAND(command_tmc_spi_endstop_home,
             "tmc_spi_endstop_home oid=%c clock=%u rest_ticks=%u"
             " trsync_oid=%c trigger_reason=%c");

void
command_tmc_spi_endstop_query_state(uint32_t *args)
{
    struct tmc_spi_endstop *e = oid_lookup(
        args[0], command_config_tmc_spi_endstop);
    sendf("tmc_spi_endstop_state oid=%c trigger_clock=%u"
          , args[0], e->trigger_clock);
}
DECL_COMMAND(command_tmc_spi_endstop_query_state,
             "tmc_spi_endstop_query_state oid=%c");

void
tmc_spi_endstop_task(void)
{
    if (!sched_check_wake(&tmc_spi_endstop_wake))
        return;
    uint8_t oid;
    struct tmc_spi_endstop *e;
    foreach_oid(oid, e, command_config_tmc_spi_endstop) {
        if (!e->pending)
            continue;
        irq_disable();
        e->pending = 0;
        irq_enable();
        uint8_t data[MAX_DATA_LEN];
        memset(data, 0, e->data_len);
        uint32_t time = timer_read_time();
        spidev_transfer(e->spi, 1, e->data_len, data);
        if (!(data[e->status_pos] & SPI_STATUS_SG2))
            continue;
        sched_del_timer(&e->timer);
        e->pending = 0;
        e->trigger_clock = time;
        trsync_do_trigger(e->ts, e->trigger_reason);
    }
}
DECL_TASK(tmc_spi_endstop_task);
