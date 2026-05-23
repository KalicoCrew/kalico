#ifndef KALICO_VTIME_H
#define KALICO_VTIME_H

#include <stdint.h>
#include <stdatomic.h>

#define VTIME_SHM_NAME "/kalico_vtime"

struct vtime_shm {
    _Atomic uint64_t nanos;
    _Atomic uint32_t num_sleepers;
    _Atomic uint32_t num_participants;
    _Atomic uint32_t initialized;
};

#endif
