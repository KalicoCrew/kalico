// LD_PRELOAD shim that intercepts klipper's libc syscalls and replaces
// /dev/gpiochip*, /dev/spidev*, /sys/class/pwm/*, /sys/bus/iio/* access
// with sim-internal state and chip emulator sockets.
//
// See docs/superpowers/specs/2026-05-08-syscall-shim-design.md
#include "libsim_intercept.h"

#include <dlfcn.h>
#include <fcntl.h>
#include <linux/gpio.h>
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

// Per-line state. Indexed by [chip_id][offset].
// Must match firmware's src/linux/internal.h: MAX_GPIO_LINES=288, 9 chips.
#define MAX_GPIO_CHIPS 9
#define MAX_GPIO_LINES 288

struct sim_gpio_line {
    int direction;   // 0 = input, 1 = output, -1 = unconfigured
    int value;       // 0 or 1
};
static struct sim_gpio_line gpio_lines[MAX_GPIO_CHIPS][MAX_GPIO_LINES];
static pthread_mutex_t gpio_state_mtx = PTHREAD_MUTEX_INITIALIZER;

// Currently-asserted output line — used as the SPI CS demultiplex key.
struct sim_active_cs { int chip_id; int line_offset; int valid; };
static struct sim_active_cs active_cs;

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

static void free_fake_fd(int fd) {
    int idx = fd - FAKE_FD_BASE;
    if (idx <= 0 || idx >= MAX_FAKE_FDS) return;
    pthread_mutex_lock(&fake_slots_mtx);
    fake_slots[idx].kind = SIM_NONE;
    pthread_mutex_unlock(&fake_slots_mtx);
}

static struct sim_fd_slot *slot_for_fd(int fd) {
    int idx = fd - FAKE_FD_BASE;
    if (idx <= 0 || idx >= MAX_FAKE_FDS) return NULL;
    if (fake_slots[idx].kind == SIM_NONE) return NULL;
    return &fake_slots[idx];
}

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
    (void)flags;
    int chip_id = -1;
    if (sscanf(path, "/dev/gpiochip%d", &chip_id) != 1
        || chip_id < 0 || chip_id >= MAX_GPIO_CHIPS) {
        errno = ENOENT;
        return -1;
    }
    int fd = alloc_fake_fd(SIM_GPIOCHIP);
    if (fd < 0) return -1;
    slot_for_fd(fd)->u.gpiochip.chip_id = chip_id;
    LOG("open gpiochip(%s) -> fd=%d", path, fd);
    return fd;
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

static int gpio_handle_get_linehandle(int chip_fd, struct gpiohandle_request *req) {
    struct sim_fd_slot *slot = slot_for_fd(chip_fd);
    if (!slot || slot->kind != SIM_GPIOCHIP) {
        errno = EBADF;
        return -1;
    }
    if (req->lines != 1) {
        // Klipper always requests exactly one line; reject otherwise.
        errno = EINVAL;
        return -1;
    }
    int chip_id = slot->u.gpiochip.chip_id;
    int offset = req->lineoffsets[0];
    if (offset < 0 || offset >= MAX_GPIO_LINES) {
        errno = EINVAL;
        return -1;
    }
    int line_fd = alloc_fake_fd(SIM_GPIOLINE);
    if (line_fd < 0) return -1;
    struct sim_fd_slot *line_slot = slot_for_fd(line_fd);
    line_slot->u.gpioline.chip_id = chip_id;
    line_slot->u.gpioline.line_offset = offset;
    line_slot->u.gpioline.flags = req->flags;
    int direction = (req->flags & GPIOHANDLE_REQUEST_OUTPUT) ? 1 : 0;
    pthread_mutex_lock(&gpio_state_mtx);
    gpio_lines[chip_id][offset].direction = direction;
    if (direction == 1) {
        gpio_lines[chip_id][offset].value = req->default_values[0] ? 1 : 0;
    }
    line_slot->u.gpioline.last_value = gpio_lines[chip_id][offset].value;
    pthread_mutex_unlock(&gpio_state_mtx);
    req->fd = line_fd;
    LOG("gpio get_linehandle chip=%d offset=%d dir=%d -> fd=%d",
        chip_id, offset, direction, line_fd);
    return 0;
}

static int gpio_handle_set_values(int line_fd, struct gpiohandle_data *data) {
    struct sim_fd_slot *slot = slot_for_fd(line_fd);
    if (!slot || slot->kind != SIM_GPIOLINE) {
        errno = EBADF;
        return -1;
    }
    int chip_id = slot->u.gpioline.chip_id;
    int offset = slot->u.gpioline.line_offset;
    int v = data->values[0] ? 1 : 0;
    pthread_mutex_lock(&gpio_state_mtx);
    gpio_lines[chip_id][offset].value = v;
    slot->u.gpioline.last_value = v;
    // Track currently-asserted CS for SPI dispatch.
    if (v == 1 && gpio_lines[chip_id][offset].direction == 1) {
        active_cs.chip_id = chip_id;
        active_cs.line_offset = offset;
        active_cs.valid = 1;
    } else if (v == 0 && active_cs.valid
               && active_cs.chip_id == chip_id
               && active_cs.line_offset == offset) {
        active_cs.valid = 0;
    }
    pthread_mutex_unlock(&gpio_state_mtx);
    return 0;
}

static int gpio_handle_get_values(int line_fd, struct gpiohandle_data *data) {
    struct sim_fd_slot *slot = slot_for_fd(line_fd);
    if (!slot || slot->kind != SIM_GPIOLINE) {
        errno = EBADF;
        return -1;
    }
    int chip_id = slot->u.gpioline.chip_id;
    int offset = slot->u.gpioline.line_offset;
    pthread_mutex_lock(&gpio_state_mtx);
    data->values[0] = gpio_lines[chip_id][offset].value ? 1 : 0;
    pthread_mutex_unlock(&gpio_state_mtx);
    return 0;
}

int ioctl(int fd, unsigned long req, ...) {
    void *arg;
    va_list ap; va_start(ap, req); arg = va_arg(ap, void *); va_end(ap);
    if (!is_fake_fd(fd)) return real_ioctl(fd, req, arg);
    struct sim_fd_slot *slot = slot_for_fd(fd);
    if (!slot) { errno = EBADF; return -1; }
    switch (req) {
        case GPIO_GET_LINEHANDLE_IOCTL:
            return gpio_handle_get_linehandle(fd, (struct gpiohandle_request *)arg);
        case GPIOHANDLE_SET_LINE_VALUES_IOCTL:
            return gpio_handle_set_values(fd, (struct gpiohandle_data *)arg);
        case GPIOHANDLE_GET_LINE_VALUES_IOCTL:
            return gpio_handle_get_values(fd, (struct gpiohandle_data *)arg);
        default:
            LOG("ioctl(fd=%d, req=0x%lx) UNHANDLED on slot kind=%d", fd, req, slot->kind);
            errno = EINVAL;
            return -1;
    }
}

int fcntl(int fd, int cmd, ...) {
    if (!is_fake_fd(fd)) {
        // Pass through. Use va_arg conservatively (cmd determines arg type).
        va_list ap; va_start(ap, cmd);
        long arg = va_arg(ap, long);
        va_end(ap);
        return real_fcntl(fd, cmd, arg);
    }
    // Fake fd — no-op for FD_CLOEXEC, F_SETFL, F_GETFL.
    (void)cmd;
    return 0;
}

int close(int fd) {
    if (!is_fake_fd(fd)) return real_close(fd);
    free_fake_fd(fd);
    return 0;
}
