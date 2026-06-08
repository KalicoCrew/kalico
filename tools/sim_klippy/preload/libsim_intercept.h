#ifndef LIBSIM_INTERCEPT_H
#define LIBSIM_INTERCEPT_H

#include <stdint.h>

// Fake-fd base — well above any real Linux fd allocation
// (RLIMIT_NOFILE defaults to 1024, hard limit ~1M).
#define FAKE_FD_BASE 0x10000000
#define MAX_FAKE_FDS 256

enum sim_slot_kind {
    SIM_NONE = 0,
    SIM_GPIOCHIP,
    SIM_GPIOLINE,
    SIM_SPIDEV,
    SIM_PWM_FILE,
    SIM_IIO_FILE,
};

#endif
