// TLA2518 querying support
//
// Copyright (C) 2023  Lasse Dalegaard <dalegaard@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "basecmd.h"    // oid_alloc
#include "board/irq.h"  // irq_disable
#include "board/misc.h" // timer_read_time
#include "command.h"    // DECL_COMMAND
#include "sched.h"      // DECL_TASK
#include "spicmds.h"    // spidev_transfer

struct tla2518_instance {
  struct timer round_timer;
  struct timer sample_timer;

  uint32_t rest_time;
  uint32_t sample_time;
  struct spidev_s *spi;
  uint8_t channels;

  uint8_t flags;

  uint8_t missing_channels;
};

enum {
  FLAG_ACTIVE = 1,
  FLAG_ROUND_PENDING = 2,
  FLAG_SAMPLE_PENDING = 3,
};

static struct task_wake tla2518_wake;

static uint_fast8_t tla2518_round_event(struct timer *round_timer) {
  struct tla2518_instance *inst =
      container_of(round_timer, struct tla2518_instance, round_timer);

  inst->round_timer.waketime += inst->rest_time;
  // No current sample round ongoing, start a new one
  if (inst->flags == FLAG_ACTIVE && inst->missing_channels == 0) {
    inst->flags |= FLAG_ROUND_PENDING;
    sched_wake_task(&tla2518_wake);
  }
  return SF_RESCHEDULE;
}

static uint_fast8_t tla2518_sample_event(struct timer *sample_timer) {
  struct tla2518_instance *inst =
      container_of(sample_timer, struct tla2518_instance, sample_timer);

  sched_wake_task(&tla2518_wake);
  inst->flags |= FLAG_SAMPLE_PENDING;
  return SF_DONE;
}

void command_config_tla2518(uint32_t *args) {
  struct tla2518_instance *inst =
      oid_alloc(args[0], command_config_tla2518, sizeof(*inst));
  inst->round_timer.func = tla2518_round_event;
  inst->sample_timer.func = tla2518_sample_event;
  inst->spi = spidev_oid_lookup(args[1]);
  inst->channels = args[2];
  inst->missing_channels = 0;
  inst->flags = 0;
}
DECL_COMMAND(command_config_tla2518,
             "config_tla2518 oid=%c spi_oid=%c channels=%c");

void command_query_tla2518(uint32_t *args) {
  struct tla2518_instance *inst = oid_lookup(args[0], command_config_tla2518);

  sched_del_timer(&inst->round_timer);
  inst->round_timer.waketime = args[1];
  inst->rest_time = args[2];
  inst->sample_time = args[3];
  inst->flags = inst->rest_time != 0 ? FLAG_ACTIVE : 0;
  if (!inst->rest_time)
    return;
  sched_add_timer(&inst->round_timer);
}
DECL_COMMAND(command_query_tla2518,
             "query_tla2518 oid=%c clock=%u rest_ticks=%u sample_ticks=%u");

void tla2518_task(void) {
  if (!sched_check_wake(&tla2518_wake))
    return;
  uint8_t oid;
  struct tla2518_instance *inst;
  foreach_oid(oid, inst, command_config_tla2518) {
    int flags = inst->flags;
    // No sampling pending, skip over this.
    if (!(flags & (FLAG_SAMPLE_PENDING | FLAG_ROUND_PENDING)))
      continue;

    // Unset any flags except active, as we'll handle it now
    irq_disable();
    inst->flags &= FLAG_ACTIVE;
    irq_enable();

    uint8_t msg[3] = {0};
    if ((flags & FLAG_ROUND_PENDING) && (inst->missing_channels == 0)) {
      // Start new round
      inst->missing_channels = inst->channels;
      uint8_t first_channel = __builtin_ctz(inst->missing_channels);
      msg[0] = 0x80 | (first_channel << 3);
      spidev_transfer(inst->spi, 3, sizeof(msg), msg);
      inst->sample_timer.waketime = timer_read_time() + inst->sample_time;
      sched_add_timer(&inst->sample_timer);
    } else if (inst->missing_channels != 0 && (flags & FLAG_SAMPLE_PENDING)) {
      // Currently in a round, grab next sample and start the new one
      uint8_t channel = __builtin_ctz(inst->missing_channels);
      inst->missing_channels &= ~(1 << channel);

      uint8_t next_channel = __builtin_ctz(inst->missing_channels);
      msg[0] = 0x80 | (next_channel << 3);
      spidev_transfer(inst->spi, 3, sizeof(msg), msg);
      sendf("tla2518_result oid=%c clock=%u channel=%c value=%u", oid,
            timer_read_time(), msg[2] >> 4, msg[0] << 8 | msg[1]);

      if (inst->missing_channels != 0) {
        inst->sample_timer.waketime = timer_read_time() + inst->sample_time;
        sched_add_timer(&inst->sample_timer);
      }
    }
  }
}
DECL_TASK(tla2518_task);
