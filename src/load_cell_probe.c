// Load Cell based end stops.
//
// Copyright (C) 2025  Gareth Farrington <gareth@waves.ky>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // shutdown
#include "trsync.h" // trsync_do_trigger
#include "board/misc.h" // timer_read_time
#include "sos_filter.h" // fixedQ12_t
#include "load_cell_probe.h" //load_cell_probe_report_sample
#include <stdint.h> // int32_t
#include <stdlib.h> // abs

// Q2.29
typedef int32_t fixedQ2_t;
#define FIXEDQ2 2
#define FIXEDQ2_FRAC_BITS ((32 - FIXEDQ2) - 1)

// Q32.29 - a Q2.29 value stored in int64
typedef int64_t fixedQ32_t;
#define FIXEDQ32_FRAC_BITS FIXEDQ2_FRAC_BITS

// Q16.15
typedef int32_t fixedQ16_t;
#define FIXEDQ16 16
#define FIXEDQ16_FRAC_BITS ((32 - FIXEDQ16) - 1)

// Q48.15 - a Q16.15 value stored in int64
typedef int64_t fixedQ48_t;
#define FIXEDQ48_FRAC_BITS FIXEDQ16_FRAC_BITS

#define MAX_TRIGGER_GRAMS ((1UL << FIXEDQ16) - 1)
#define ERROR_SAFETY_RANGE 0
#define ERROR_OVERFLOW 1
#define ERROR_WATCHDOG 2

// Flags
enum {FLAG_IS_HOMING = 1 << 0
    , FLAG_IS_HOMING_TRIGGER = 1 << 1
    , FLAG_AWAIT_HOMING = 1 << 2
    };

// Endstop Structure
struct load_cell_probe {
    struct timer time;
    uint32_t trigger_grams, trigger_ticks, last_sample_ticks, rest_ticks;
    uint32_t homing_start_time;
    struct trsync *ts;
    int32_t safety_counts_min, safety_counts_max, tare_counts;
    uint8_t flags, trigger_reason, error_reason, watchdog_max
            , watchdog_count;
    fixedQ16_t trigger_grams_fixed;
    fixedQ2_t grams_per_count;
    struct sos_filter *sf;
};

static inline uint8_t
overflows_int32(int64_t value) {
    return value > (int64_t)INT32_MAX || value < (int64_t)INT32_MIN;
}

// returns the integer part of a fixedQ48_t
static inline int64_t
round_fixedQ48(const int64_t fixed_value) {
    return fixed_value >> FIXEDQ48_FRAC_BITS;
}

// Convert sensor counts to grams
static inline fixedQ48_t
counts_to_grams(struct load_cell_probe *lcp, const int32_t counts) {
    // tearing ensures readings are referenced to 0.0g
    const int32_t delta = counts - lcp->tare_counts;
    // convert sensor counts to grams by multiplication: 124 * 0.051 = 6.324
    // this optimizes to single cycle SMULL instruction
    const fixedQ32_t product = (int64_t)delta * (int64_t)lcp->grams_per_count;
    // after multiplication there are 30 fraction bits, reduce to 15
    // caller verifies this wont overflow a 32bit int when truncated
    const fixedQ48_t grams = product >>
                                (FIXEDQ32_FRAC_BITS - FIXEDQ48_FRAC_BITS);
    return grams;
}

static inline uint8_t
is_flag_set(const uint8_t mask, struct load_cell_probe *lcp)
{
    return !!(mask & lcp->flags);
}

static inline void
set_flag(uint8_t mask, struct load_cell_probe *lcp)
{
    lcp->flags |= mask;
}

static inline void
clear_flag(uint8_t mask, struct load_cell_probe *lcp)
{
    lcp->flags &= ~mask;
}

void
try_trigger(struct load_cell_probe *lcp, uint32_t ticks)
{
    uint8_t is_homing_triggered = is_flag_set(FLAG_IS_HOMING_TRIGGER, lcp);
    if (!is_homing_triggered) {
        // the first triggering sample when homing sets the trigger time
        lcp->trigger_ticks = ticks;
        // this flag latches until a reset, disabling further triggering
        set_flag(FLAG_IS_HOMING_TRIGGER, lcp);
        trsync_do_trigger(lcp->ts, lcp->trigger_reason);
    }
}

void
trigger_error(struct load_cell_probe *lcp, uint8_t error_code)
{
    trsync_do_trigger(lcp->ts, lcp->error_reason + error_code);
}

// Used by Sensors to report new raw ADC sample
void
load_cell_probe_report_sample(struct load_cell_probe *lcp
                                , const int32_t sample)
{
    // only process samples when homing
    uint8_t is_homing = is_flag_set(FLAG_IS_HOMING, lcp);
    if (!is_homing) {
        return;
    }

    // save new sample
    uint32_t ticks = timer_read_time();
    lcp->last_sample_ticks = ticks;
    lcp->watchdog_count = 0;

    // do not trigger before homing start time
    uint8_t await_homing = is_flag_set(FLAG_AWAIT_HOMING, lcp);
    if (await_homing && timer_is_before(ticks, lcp->homing_start_time)) {
        return;
    }
    clear_flag(FLAG_AWAIT_HOMING, lcp);

    // check for safety limit violations
    const uint8_t is_safety_trigger = sample <= lcp->safety_counts_min
                                        || sample >= lcp->safety_counts_max;
    // too much force, this is an error while homing
    if (is_safety_trigger) {
        trigger_error(lcp, ERROR_SAFETY_RANGE);
        return;
    }

    // convert sample to grams
    const fixedQ48_t raw_grams = counts_to_grams(lcp, sample);
    if (overflows_int32(raw_grams)) {
        trigger_error(lcp, ERROR_OVERFLOW);
        return;
    }

    // perform filtering
    const fixedQ16_t filtered_grams = sosfilt(lcp->sf, (fixedQ16_t)raw_grams);

    // update trigger state
    if (abs(filtered_grams) >= lcp->trigger_grams_fixed) {
        try_trigger(lcp, lcp->last_sample_ticks);
    }
}

// Timer callback that monitors for timeouts
static uint_fast8_t
watchdog_event(struct timer *t)
{
    struct load_cell_probe *lcp = container_of(t, struct load_cell_probe
                                        , time);
    uint8_t is_homing = is_flag_set(FLAG_IS_HOMING, lcp);
    uint8_t is_homing_trigger = is_flag_set(FLAG_IS_HOMING_TRIGGER, lcp);
    // the watchdog stops when not homing or when trsync becomes triggered
    if (!is_homing || is_homing_trigger) {
        return SF_DONE;
    }

    if (lcp->watchdog_count > lcp->watchdog_max) {
        trigger_error(lcp, ERROR_WATCHDOG);
    }
    lcp->watchdog_count += 1;

    // A sample was recently delivered, continue monitoring
    lcp->time.waketime += lcp->rest_ticks;
    return SF_RESCHEDULE;
}

static void
set_endstop_range(struct load_cell_probe *lcp
                , int32_t safety_counts_min, int32_t safety_counts_max
                , int32_t tare_counts, uint32_t trigger_grams
                , fixedQ2_t grams_per_count)
{
    if (!(safety_counts_max >= safety_counts_min)) {
        shutdown("Safety range reversed");
    }
    if (trigger_grams > MAX_TRIGGER_GRAMS) {
        shutdown("trigger_grams too large");
    }
    // grams_per_count must be a positive fraction in Q2 format
    const fixedQ2_t one = 1UL << FIXEDQ2_FRAC_BITS;
    if (grams_per_count < 0 || grams_per_count >= one) {
        shutdown("grams_per_count is invalid");
    }
    lcp->safety_counts_min = safety_counts_min;
    lcp->safety_counts_max = safety_counts_max;
    lcp->tare_counts = tare_counts;
    lcp->trigger_grams = trigger_grams;
    lcp->trigger_grams_fixed = trigger_grams << FIXEDQ16_FRAC_BITS;
    lcp->grams_per_count = grams_per_count;
}

// Create a load_cell_probe
void
command_config_load_cell_probe(uint32_t *args)
{
    struct load_cell_probe *lcp = oid_alloc(args[0]
                            , command_config_load_cell_probe, sizeof(*lcp));
    lcp->flags = 0;
    lcp->trigger_ticks = 0;
    lcp->watchdog_max = 0;
    lcp->watchdog_count = 0;
    lcp->sf = sos_filter_oid_lookup(args[1]);
    set_endstop_range(lcp, 0, 0, 0, 0, 0);
}
DECL_COMMAND(command_config_load_cell_probe, "config_load_cell_probe"
                                               " oid=%c sos_filter_oid=%c");

// Lookup a load_cell_probe
struct load_cell_probe *
load_cell_probe_oid_lookup(uint8_t oid)
{
    return oid_lookup(oid, command_config_load_cell_probe);
}

// Set the triggering range and tare value
void
command_load_cell_probe_set_range(uint32_t *args)
{
    struct load_cell_probe *lcp = load_cell_probe_oid_lookup(args[0]);
    set_endstop_range(lcp, args[1], args[2], args[3], args[4]
                , (fixedQ16_t)args[5]);
}
DECL_COMMAND(command_load_cell_probe_set_range, "load_cell_probe_set_range"
    " oid=%c safety_counts_min=%i safety_counts_max=%i tare_counts=%i"
    " trigger_grams=%u grams_per_count=%i");

// Home an axis
void
command_load_cell_probe_home(uint32_t *args)
{
    struct load_cell_probe *lcp = load_cell_probe_oid_lookup(args[0]);
    sched_del_timer(&lcp->time);
    // clear the homing trigger flag
    clear_flag(FLAG_IS_HOMING_TRIGGER, lcp);
    clear_flag(FLAG_IS_HOMING, lcp);
    lcp->trigger_ticks = 0;
    lcp->ts = NULL;
    // 0 samples indicates homing is finished
    if (args[3] == 0) {
        // Disable end stop checking
        return;
    }
    lcp->ts = trsync_oid_lookup(args[1]);
    lcp->trigger_reason = args[2];
    lcp->error_reason = args[3];
    lcp->time.waketime = args[4];
    lcp->homing_start_time = args[4];
    lcp->rest_ticks = args[5];
    lcp->watchdog_max = args[6];
    lcp->watchdog_count = 0;
    lcp->time.func = watchdog_event;
    set_flag(FLAG_IS_HOMING, lcp);
    set_flag(FLAG_AWAIT_HOMING, lcp);
    sched_add_timer(&lcp->time);
}
DECL_COMMAND(command_load_cell_probe_home,
             "load_cell_probe_home oid=%c trsync_oid=%c trigger_reason=%c"
             " error_reason=%c clock=%u rest_ticks=%u timeout=%u");

void
command_load_cell_probe_query_state(uint32_t *args)
{
    uint8_t oid = args[0];
    struct load_cell_probe *lcp = load_cell_probe_oid_lookup(args[0]);
    sendf("load_cell_probe_state oid=%c is_homing_trigger=%c trigger_ticks=%u"
            , oid
            , is_flag_set(FLAG_IS_HOMING_TRIGGER, lcp)
            , lcp->trigger_ticks);
}
DECL_COMMAND(command_load_cell_probe_query_state
                , "load_cell_probe_query_state oid=%c");
