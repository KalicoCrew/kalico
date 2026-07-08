#ifndef __INDX_COIL_DRIVER_H__
#define __INDX_COIL_DRIVER_H__

extern "C" {
#include "board/internal.h"
#include <stdint.h>
}

struct coil_driver {
    // Maximum number of fired cycles per 512-cycle frame (0 = no limit).
    uint32_t duty_limit{0};

    void
    shutdown_();

    void
    set_duty(float duty);

    void
    set_cycle_limit(uint32_t limit);

    // time_on_first is the (shorter) ON time used for the first fired cycle of
    // every burst
    void
    set_timings(float time_on, float time_off, float time_on_first);

    // Cumulative number of overvoltage trips since boot.
    uint32_t
    get_ov_count();

    uint32_t
    get_total_charge();

    // Start driver tuning procedure, to find timing parameters.
    void
    start_tune(bool want_details);

    // True while a tuning session is in progress.
    bool
    tune_active();

    // Advance the tuning state machine one step.
    void
    tune_step();

    // Send the latched tuning outcome (status/error/found timings) to the host.
    void
    report_tune_status();

    // Send the drive timings currently in effect to the host.
    void
    report_params();
};

coil_driver
coil_driver_create();

#endif
