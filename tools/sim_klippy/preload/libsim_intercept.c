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
#include <sys/types.h>
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

// Real libc symbols — populated by shim_init.
static int (*real_open)(const char *, int, ...) = NULL;
static int (*real_openat)(int, const char *, int, ...) = NULL;
static int (*real_close)(int) = NULL;
static int (*real_ioctl)(int, unsigned long, ...) = NULL;
static ssize_t (*real_read)(int, void *, size_t) = NULL;
static ssize_t (*real_pread)(int, void *, size_t, off_t) = NULL;
static ssize_t (*real_write)(int, const void *, size_t) = NULL;
static int (*real_fcntl)(int, int, ...) = NULL;
static int (*real_access)(const char *, int) = NULL;

__attribute__((constructor))
static void shim_init(void) {
    const char *v = getenv("KALICO_SIM_SHIM_VERBOSE");
    verbose = (v && v[0] == '1');
    real_open    = dlsym(RTLD_NEXT, "open");
    real_openat  = dlsym(RTLD_NEXT, "openat");
    real_close   = dlsym(RTLD_NEXT, "close");
    real_ioctl   = dlsym(RTLD_NEXT, "ioctl");
    real_read    = dlsym(RTLD_NEXT, "read");
    real_pread   = dlsym(RTLD_NEXT, "pread");
    real_write   = dlsym(RTLD_NEXT, "write");
    real_fcntl   = dlsym(RTLD_NEXT, "fcntl");
    real_access  = dlsym(RTLD_NEXT, "access");
    LOG("init pid=%d", (int)getpid());
}

// Path classification — returns the slot kind to allocate, or SIM_NONE
// if the path should pass through to the real syscall.
static enum sim_slot_kind classify_path(const char *path) {
    if (!path) return SIM_NONE;
    if (strncmp(path, "/dev/gpiochip", 13) == 0) return SIM_GPIOCHIP;
    if (strncmp(path, "/dev/spidev", 11) == 0) return SIM_SPIDEV;
    if (strncmp(path, "/sys/class/pwm/", 15) == 0) return SIM_PWM_FILE;
    if (strncmp(path, "/sys/bus/iio/", 13) == 0) return SIM_IIO_FILE;
    return SIM_NONE;
}

// Stubbed per-handler entries; later tasks fill in real allocation.
static int sim_open_gpiochip(const char *path, int flags) {
    LOG("open gpiochip(%s) STUB", path);
    errno = ENOSYS;
    return -1;
}
static int sim_open_spidev(const char *path, int flags) {
    LOG("open spidev(%s) STUB", path);
    errno = ENOSYS;
    return -1;
}
static int sim_open_pwm(const char *path, int flags) {
    LOG("open pwm(%s) STUB", path);
    errno = ENOSYS;
    return -1;
}
static int sim_open_iio(const char *path, int flags) {
    LOG("open iio(%s) STUB", path);
    errno = ENOSYS;
    return -1;
}

int open(const char *path, int flags, ...) {
    mode_t mode = 0;
    if (flags & (O_CREAT | O_TMPFILE)) {
        va_list ap; va_start(ap, flags); mode = va_arg(ap, mode_t); va_end(ap);
    }
    enum sim_slot_kind k = classify_path(path);
    switch (k) {
        case SIM_GPIOCHIP:  return sim_open_gpiochip(path, flags);
        case SIM_SPIDEV:    return sim_open_spidev(path, flags);
        case SIM_PWM_FILE:  return sim_open_pwm(path, flags);
        case SIM_IIO_FILE:  return sim_open_iio(path, flags);
        default:            return real_open(path, flags, mode);
    }
}

int openat(int dirfd, const char *path, int flags, ...) {
    mode_t mode = 0;
    if (flags & (O_CREAT | O_TMPFILE)) {
        va_list ap; va_start(ap, flags); mode = va_arg(ap, mode_t); va_end(ap);
    }
    enum sim_slot_kind k = classify_path(path);
    switch (k) {
        case SIM_GPIOCHIP:  return sim_open_gpiochip(path, flags);
        case SIM_SPIDEV:    return sim_open_spidev(path, flags);
        case SIM_PWM_FILE:  return sim_open_pwm(path, flags);
        case SIM_IIO_FILE:  return sim_open_iio(path, flags);
        default:            return real_openat(dirfd, path, flags, mode);
    }
}

int access(const char *path, int mode) {
    if (classify_path(path) != SIM_NONE) {
        // Sim paths always exist if the shim is loaded.
        return 0;
    }
    return real_access(path, mode);
}
