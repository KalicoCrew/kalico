#ifndef __INDX_COIL_DRIVER_H__
#define __INDX_COIL_DRIVER_H__

extern "C" {
#include "board/internal.h"
#include <stdint.h>
}

// Latched outcome of a nozzle-presence ringdown probe.
enum class ringdown_status : uint8_t {
    idle = 0,
    running = 1,
    valid = 2,
    not_enough_peaks = 3,
    aborted = 4,
    overvoltage = 5,
    timeout = 6,
};

enum class nozzle_presence : uint8_t {
    unknown = 0,
    present = 1,
    absent = 2,
};

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

    // --- Nozzle presence via LC ringdown ---

    // Configure continuous probe, heat gate, and amplitude thresholds.
    // present_peak_v / absent_peak_v / min_peak_v are tank volts (Bondtech:
    // seated loads the coil so peak is lower; require present < absent).
    // Periods are milliseconds. excite_scale multiplies soft-start ON ticks
    // (clamped to <= 1.0 so the pulse cannot exceed coil_time_on_first).
    // zero_margin_v is the ADC floor used for DUMP off_start annotation.
    // Overvoltage mid-probe always aborts.
    void
    set_ringdown_params(bool enable, bool heat_gate, float present_peak_v,
                        float absent_peak_v, uint32_t idle_ms, uint32_t heat_ms,
                        float excite_scale, float zero_margin_v,
                        float min_peak_v);

    // Start a oneshot ringdown probe. Fails closed if tune is active or coil
    // timings are not set. report=true sends indx_nozzle_presence when done.
    // dump=true streams the capture buffer (and peaks) before the status.
    void
    start_ringdown(bool report, bool dump);

    bool
    ringdown_active();

    // Advance the ringdown state machine; also starts background probes when
    // enabled. heating=true selects the longer recheck period.
    void
    ringdown_step(bool heating);

    // Last latched presence classification (unknown until a valid probe).
    nozzle_presence
    get_nozzle_presence();

    // Send latched ringdown outcome to the host.
    void
    report_ringdown_status();
};

coil_driver
coil_driver_create();

#endif
