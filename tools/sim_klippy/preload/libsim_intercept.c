// LD_PRELOAD shim that intercepts klipper's libc syscalls and replaces
// /dev/gpiochip*, /dev/spidev*, /sys/class/pwm/*, /sys/bus/iio/* access
// with sim-internal state and chip emulator sockets.
//
// See docs/superpowers/specs/2026-05-08-syscall-shim-design.md
#include "libsim_intercept.h"

#include <dlfcn.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

// Verbose logging — set KALICO_SIM_SHIM_VERBOSE=1 to enable.
static int verbose = 0;
#define LOG(fmt, ...) do { if (verbose) fprintf(stderr, "[shim] " fmt "\n", ##__VA_ARGS__); } while (0)

#include <pthread.h>
#include <errno.h>

// Per-fd slot — see spec §"Per-fd state and slot allocation"
struct sim_fd_slot {
    enum sim_slot_kind kind;
    union {
        struct { int chip_id; } gpiochip;
        struct {
            int chip_id;
            int line_offset;
            uint32_t flags;
            int last_value;
        } gpioline;
        struct {
            int bus, dev;
            uint32_t speed_hz;
            uint8_t mode;
            int chip_socket_fd;     // negative = not yet connected
        } spidev;
        struct {
            int chip_id, pwm_id;
            char file[32];          // "period" / "duty_cycle" / "enable"
            uint64_t last_value;
        } pwm_file;
        struct { int channel; } iio_file;
    } u;
};

static struct sim_fd_slot fake_slots[MAX_FAKE_FDS];
static pthread_mutex_t fake_slots_mtx = PTHREAD_MUTEX_INITIALIZER;

__attribute__((unused))
static int alloc_fake_fd(enum sim_slot_kind kind) {
    pthread_mutex_lock(&fake_slots_mtx);
    for (int i = 1; i < MAX_FAKE_FDS; i++) {  // slot 0 reserved
        if (fake_slots[i].kind == SIM_NONE) {
            memset(&fake_slots[i], 0, sizeof(fake_slots[i]));
            fake_slots[i].kind = kind;
            pthread_mutex_unlock(&fake_slots_mtx);
            return FAKE_FD_BASE + i;
        }
    }
    pthread_mutex_unlock(&fake_slots_mtx);
    LOG("alloc_fake_fd: out of slots");
    errno = ENFILE;
    return -1;
}

__attribute__((unused))
static void free_fake_fd(int fd) {
    int idx = fd - FAKE_FD_BASE;
    if (idx <= 0 || idx >= MAX_FAKE_FDS) return;
    pthread_mutex_lock(&fake_slots_mtx);
    fake_slots[idx].kind = SIM_NONE;
    pthread_mutex_unlock(&fake_slots_mtx);
}

__attribute__((unused))
static struct sim_fd_slot *slot_for_fd(int fd) {
    int idx = fd - FAKE_FD_BASE;
    if (idx <= 0 || idx >= MAX_FAKE_FDS) return NULL;
    if (fake_slots[idx].kind == SIM_NONE) return NULL;
    return &fake_slots[idx];
}

__attribute__((unused))
static int is_fake_fd(int fd) {
    return fd >= FAKE_FD_BASE && fd < FAKE_FD_BASE + MAX_FAKE_FDS;
}

__attribute__((constructor))
static void shim_init(void) {
    const char *v = getenv("KALICO_SIM_SHIM_VERBOSE");
    verbose = (v && v[0] == '1');
    LOG("init pid=%d", (int)getpid());
}
