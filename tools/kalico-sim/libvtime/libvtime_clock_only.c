// Minimal virtual-time shim for the klippy host process.
//
// Only intercepts clock_gettime so klippy's get_monotonic() returns the
// shared virtual clock. Does NOT intercept ppoll/poll/nanosleep/timer_*
// — klippy's reactor uses real I/O waiting, which lets it respond to
// MCU serial data immediately.
//
// The MCU processes load the full libvtime.so (which overrides ppoll
// etc.) so their timer dispatch advances the virtual clock. Klippy
// reads the same clock via this shim, keeping both sides synchronized.

#define _GNU_SOURCE
#include <dlfcn.h>
#include <stdatomic.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

struct vtime_shm {
    _Atomic uint64_t nanos;
    _Atomic uint32_t num_sleepers;
    _Atomic uint32_t num_participants;
    _Atomic uint32_t initialized;
};

static struct vtime_shm *vshm = NULL;
static int (*real_clock_gettime)(clockid_t, struct timespec *) = NULL;

__attribute__((constructor(101)))
static void vtime_clock_init(void)
{
    real_clock_gettime = dlsym(RTLD_NEXT, "clock_gettime");

    int fd = open("/dev/shm/kalico_vtime", O_RDWR);
    if (fd < 0)
        return;
    void *p = mmap(NULL, 32, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (p == MAP_FAILED)
        return;
    vshm = (struct vtime_shm *)p;
    atomic_fetch_add(&vshm->num_participants, 1);
}

int clock_gettime(clockid_t clk_id, struct timespec *tp)
{
    if (!vshm || !real_clock_gettime)
        return real_clock_gettime ? real_clock_gettime(clk_id, tp) : -1;

    // Only CLOCK_MONOTONIC_RAW — used by klippy's chelper get_monotonic().
    // CLOCK_MONOTONIC is left real so Rust Instant::now() (used for bridge
    // timeouts) doesn't expire in virtual time.
    if (clk_id == CLOCK_MONOTONIC_RAW) {
        uint64_t ns = atomic_load_explicit(&vshm->nanos, memory_order_acquire);
        tp->tv_sec = (time_t)(ns / 1000000000ULL);
        tp->tv_nsec = (long)(ns % 1000000000ULL);
        return 0;
    }
    return real_clock_gettime(clk_id, tp);
}
