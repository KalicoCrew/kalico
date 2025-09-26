#ifndef STEPCOMPRESS_H
#define STEPCOMPRESS_H

#include <stdint.h> // uint32_t
#include "list.h" // struct list_head

#define ERROR_RET -989898989

struct stepcompress {
    // Buffer management
    uint32_t *queue, *queue_end, *queue_pos, *queue_next;
    // Internal tracking
    uint32_t max_error;
    double mcu_time_offset, mcu_freq, last_step_print_time;
    // Message generation
    uint64_t last_step_clock;
    struct list_head *msg_queue;
    uint32_t oid;
    int32_t queue_step_msgtag, set_next_step_dir_msgtag;
    int sdir, invert_sdir;
    // Step+dir+step filter
    uint64_t next_step_clock;
    int next_step_dir;
    // History tracking
    int64_t last_position;
    struct list_head history_list;
    // Extra fields for high precision algorithm
    uint32_t next_expected_interval;
    uint16_t cached_count;
    struct rhs_3 *rhs_cache;
    struct points *errb_cache;
    // Callbacks
    int (*queue_flush)(struct stepcompress *, uint64_t);
    int (*queue_flush_far)(struct stepcompress *, uint64_t);
};

struct history_steps {
    struct list_node node;
    uint64_t first_clock, last_clock;
    int64_t start_position;
    int step_count, interval, add, add2, shift;
};

struct pull_history_steps {
    uint64_t first_clock, last_clock;
    int64_t start_position;
    int step_count, interval, add, add2, shift;
};

struct list_head;
struct stepcompress *stepcompress_alloc(struct list_head *msg_queue);
void stepcompress_fill(struct stepcompress *sc, uint32_t oid, uint32_t max_error
                       , int32_t queue_step_msgtag
                       , int32_t set_next_step_dir_msgtag);
void stepcompress_set_invert_sdir(struct stepcompress *sc
                                  , uint32_t invert_sdir);
void stepcompress_history_expire(struct stepcompress *sc, uint64_t end_clock);
void stepcompress_free(struct stepcompress *sc);
uint32_t stepcompress_get_oid(struct stepcompress *sc);
int stepcompress_get_step_dir(struct stepcompress *sc);
void stepcompress_set_time(struct stepcompress *sc
                           , double time_offset, double mcu_freq);
int stepcompress_append(struct stepcompress *sc, int sdir
                        , double print_time, double step_time);
int stepcompress_commit(struct stepcompress *sc);
int stepcompress_flush(struct stepcompress *sc, uint64_t move_clock);
int stepcompress_reset(struct stepcompress *sc, uint64_t last_step_clock);
int stepcompress_set_last_position(struct stepcompress *sc, uint64_t clock
                                   , int64_t last_position);
int64_t stepcompress_find_past_position(struct stepcompress *sc
                                        , uint64_t clock);
void stepcompress_calc_last_step_print_time(struct stepcompress *sc);
int stepcompress_extract_old(struct stepcompress *sc
                             , struct pull_history_steps *p, int max
                             , uint64_t start_clock, uint64_t end_clock);

#endif // stepcompress.h
