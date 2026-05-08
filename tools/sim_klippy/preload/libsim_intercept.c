// LD_PRELOAD shim that intercepts klipper's libc syscalls and replaces
// /dev/gpiochip*, /dev/spidev*, /sys/class/pwm/*, /sys/bus/iio/* access
// with sim-internal state and chip emulator sockets.
//
// See docs/superpowers/specs/2026-05-08-syscall-shim-design.md
#include "libsim_intercept.h"

#include <dlfcn.h>
#include <fcntl.h>
#include <linux/gpio.h>
#include <linux/spi/spidev.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>
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

#define MAX_IIO_CHANNELS 32
#define DEFAULT_ADC_VALUE 3900
static uint16_t iio_values[MAX_IIO_CHANNELS];
static pthread_mutex_t iio_state_mtx = PTHREAD_MUTEX_INITIALIZER;

__attribute__((constructor(101)))
static void iio_init(void) {
    for (int i = 0; i < MAX_IIO_CHANNELS; i++) iio_values[i] = DEFAULT_ADC_VALUE;
}

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
    (void)flags;
    int bus = -1, dev = -1;
    if (sscanf(path, "/dev/spidev%d.%d", &bus, &dev) != 2) {
        errno = ENOENT;
        return -1;
    }
    int fd = alloc_fake_fd(SIM_SPIDEV);
    if (fd < 0) return -1;
    struct sim_fd_slot *slot = slot_for_fd(fd);
    slot->u.spidev.bus = bus;
    slot->u.spidev.dev = dev;
    slot->u.spidev.chip_socket_fd = -1;
    LOG("open spidev(%s) -> fd=%d", path, fd);
    return fd;
}
static int sim_open_pwm(const char *path, int flags) {
    (void)flags;
    // Klipper opens /sys/class/pwm/pwmchip<N>/pwm<M>/<file>
    // or /sys/class/pwm/pwm-<chip>:<id>/<file> (BeagleBoard).
    int chip = 0, pwm = 0;
    char file[32] = {0};
    int matched = sscanf(path, "/sys/class/pwm/pwmchip%d/pwm%d/%31s", &chip, &pwm, file);
    if (matched != 3) {
        matched = sscanf(path, "/sys/class/pwm/pwm-%d:%d/%31s", &chip, &pwm, file);
    }
    if (matched != 3) {
        LOG("pwm open: unrecognized path %s", path);
        errno = ENOENT;
        return -1;
    }
    int fd = alloc_fake_fd(SIM_PWM_FILE);
    if (fd < 0) return -1;
    struct sim_fd_slot *slot = slot_for_fd(fd);
    slot->u.pwm_file.chip_id = chip;
    slot->u.pwm_file.pwm_id = pwm;
    snprintf(slot->u.pwm_file.file, sizeof(slot->u.pwm_file.file), "%s", file);
    slot->u.pwm_file.last_value = 0;
    LOG("open pwm chip=%d pwm=%d file=%s -> fd=%d", chip, pwm, file, fd);
    return fd;
}
static int sim_open_iio(const char *path, int flags) {
    (void)flags;
    int channel = -1;
    if (sscanf(path,
               "/sys/bus/iio/devices/iio:device0/in_voltage%d_raw",
               &channel) != 1
        || channel < 0 || channel >= MAX_IIO_CHANNELS) {
        LOG("iio open: unrecognized path %s", path);
        errno = ENOENT;
        return -1;
    }
    int fd = alloc_fake_fd(SIM_IIO_FILE);
    if (fd < 0) return -1;
    slot_for_fd(fd)->u.iio_file.channel = channel;
    LOG("open iio channel=%d -> fd=%d", channel, fd);
    return fd;
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

static int spi_get_chip_socket(struct sim_fd_slot *slot) {
    if (slot->u.spidev.chip_socket_fd >= 0) {
        return slot->u.spidev.chip_socket_fd;
    }
    if (!active_cs.valid) {
        LOG("spi xfer with no active CS");
        errno = EIO;
        return -1;
    }
    const char *sock_dir = getenv("KALICO_SIM_SOCK_DIR");
    if (!sock_dir) {
        LOG("KALICO_SIM_SOCK_DIR not set");
        errno = EIO;
        return -1;
    }
    char path[256];
    snprintf(path, sizeof(path), "%s/spi_cs_%d_%d",
             sock_dir, active_cs.chip_id, active_cs.line_offset);
    int sock = socket(AF_UNIX, SOCK_STREAM, 0);
    if (sock < 0) return -1;
    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    snprintf(addr.sun_path, sizeof(addr.sun_path), "%.*s",
             (int)(sizeof(addr.sun_path) - 1), path);
    if (connect(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        LOG("spi connect %s failed: %s", path, strerror(errno));
        real_close(sock);
        return -1;
    }
    slot->u.spidev.chip_socket_fd = sock;
    LOG("spi connected to %s -> sock=%d", path, sock);
    return sock;
}

static void spi_drop_chip_socket(struct sim_fd_slot *slot) {
    if (slot->u.spidev.chip_socket_fd >= 0) {
        real_close(slot->u.spidev.chip_socket_fd);
        slot->u.spidev.chip_socket_fd = -1;
    }
}

static int spi_handle_message(int fd, struct spi_ioc_transfer *xfer) {
    struct sim_fd_slot *slot = slot_for_fd(fd);
    if (!slot || slot->kind != SIM_SPIDEV) { errno = EBADF; return -1; }
    int sock = spi_get_chip_socket(slot);
    if (sock < 0) return -1;
    const uint8_t *tx = (const uint8_t *)(uintptr_t)xfer->tx_buf;
    uint8_t *rx = (uint8_t *)(uintptr_t)xfer->rx_buf;
    size_t off = 0;
    while (off < xfer->len) {
        ssize_t n = real_write(sock, tx + off, xfer->len - off);
        if (n <= 0) { spi_drop_chip_socket(slot); errno = EIO; return -1; }
        off += n;
    }
    if (rx) {
        off = 0;
        while (off < xfer->len) {
            ssize_t n = real_read(sock, rx + off, xfer->len - off);
            if (n <= 0) { spi_drop_chip_socket(slot); errno = EIO; return -1; }
            off += n;
        }
    }
    spi_drop_chip_socket(slot);
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
        case SPI_IOC_WR_MAX_SPEED_HZ:
            slot->u.spidev.speed_hz = *(uint32_t *)arg;
            return 0;
        case SPI_IOC_WR_MODE:
            slot->u.spidev.mode = *(uint8_t *)arg;
            return 0;
        case SPI_IOC_MESSAGE(1):
            return spi_handle_message(fd, (struct spi_ioc_transfer *)arg);
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

ssize_t write(int fd, const void *buf, size_t count) {
    if (!is_fake_fd(fd)) return real_write(fd, buf, count);
    struct sim_fd_slot *slot = slot_for_fd(fd);
    if (!slot) { errno = EBADF; return -1; }
    switch (slot->kind) {
        case SIM_SPIDEV: {
            // No-receive SPI write — same shape as a one-way transfer.
            int sock = spi_get_chip_socket(slot);
            if (sock < 0) return -1;
            size_t off = 0;
            while (off < count) {
                ssize_t n = real_write(sock, (const uint8_t *)buf + off, count - off);
                if (n <= 0) { spi_drop_chip_socket(slot); errno = EIO; return -1; }
                off += n;
            }
            spi_drop_chip_socket(slot);
            return (ssize_t)count;
        }
        case SIM_PWM_FILE: {
            // Parse the integer value klipper wrote (ASCII decimal).
            char buf2[32];
            size_t copy = count < sizeof(buf2) - 1 ? count : sizeof(buf2) - 1;
            memcpy(buf2, buf, copy);
            buf2[copy] = '\0';
            uint64_t val = strtoull(buf2, NULL, 10);
            slot->u.pwm_file.last_value = val;
            LOG("pwm write chip=%d pwm=%d file=%s val=%llu",
                slot->u.pwm_file.chip_id, slot->u.pwm_file.pwm_id,
                slot->u.pwm_file.file, (unsigned long long)val);
            return (ssize_t)count;
        }
        default:
            errno = EINVAL;
            return -1;
    }
}

ssize_t pread(int fd, void *buf, size_t count, off_t offset) {
    (void)offset;
    if (!is_fake_fd(fd)) return real_pread(fd, buf, count, offset);
    struct sim_fd_slot *slot = slot_for_fd(fd);
    if (!slot || slot->kind != SIM_IIO_FILE) { errno = EBADF; return -1; }
    pthread_mutex_lock(&iio_state_mtx);
    uint16_t val = iio_values[slot->u.iio_file.channel];
    pthread_mutex_unlock(&iio_state_mtx);
    char tmp[16];
    int n = snprintf(tmp, sizeof(tmp), "%u\n", (unsigned)val);
    if (n < 0) { errno = EIO; return -1; }
    size_t copy = (size_t)n < count ? (size_t)n : count;
    memcpy(buf, tmp, copy);
    return (ssize_t)copy;
}

ssize_t read(int fd, void *buf, size_t count) {
    if (!is_fake_fd(fd)) return real_read(fd, buf, count);
    return pread(fd, buf, count, 0);
}
