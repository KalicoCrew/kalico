#define _GNU_SOURCE
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

// THREADING MODEL
// ===============
// Klipper firmware on MACH_LINUX is single-threaded for /dev/* and /sys/*
// access — `console_task` is the sole consumer of these fds. The shim
// adds ONE additional thread (Task 8: control-socket accept thread).
//
// Lock invariants:
//   - fake_slots_mtx: held during alloc_fake_fd / free_fake_fd. Reads
//     via slot_for_fd are NOT locked, on the assumption that any given
//     slot is owned by exactly one of {klipper main, control thread}
//     at any time.
//   - gpio_state_mtx: held during gpio_lines[] and active_cs reads/writes.
//   - iio_state_mtx: held during iio_values[] reads/writes.
//
// The control-socket thread (Task 8+) must NOT touch active_cs or the
// spidev slot's chip_socket_fd — those belong to klipper's main thread.
// The control thread writes gpio_lines[] (under gpio_state_mtx) and
// iio_values[] (under iio_state_mtx). It may read pwm_file slot's
// last_value (under fake_slots_mtx) for the get_pwm diagnostic verb;
// that is read-only and atomic on the supported platforms.

static int verbose = 0;
#define LOG(fmt, ...) do { if (verbose) fprintf(stderr, "[shim] " fmt "\n", ##__VA_ARGS__); } while (0)

#include <pthread.h>
#include <errno.h>

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

// Must match firmware's src/linux/internal.h: MAX_GPIO_LINES=288, 9 chips.
#define MAX_GPIO_CHIPS 9
#define MAX_GPIO_LINES 288

struct sim_gpio_line {
    int direction;   // 0 = input, 1 = output, -1 = unconfigured
    int value;
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

#define MAX_AUTO_ENDSTOPS 8
struct auto_endstop {
    int active;
    int step_chip, step_line;
    int endstop_chip, endstop_line;
    int trigger_after_steps;
    int step_count;
    int last_step_value;          // for edge detection
    int triggered;
};
static struct auto_endstop auto_endstops[MAX_AUTO_ENDSTOPS];
static pthread_mutex_t auto_endstop_mtx = PTHREAD_MUTEX_INITIALIZER;

__attribute__((constructor(101)))
static void iio_init(void) {
    for (int i = 0; i < MAX_IIO_CHANNELS; i++) iio_values[i] = DEFAULT_ADC_VALUE;
    // Auto-endstop mappings matching printer_real/config after pin-overrides:
    // X: step PG4→gpio18, endstop→gpio200; Y: step PF11→gpio7, endstop→gpio201;
    // Z: step PG0→gpio15, endstop→gpio202. Trigger after 50 steps.
    auto_endstops[0] = (struct auto_endstop){1, 0,18, 0,200, 50, 0, 0, 0};
    auto_endstops[1] = (struct auto_endstop){1, 0,7,  0,201, 50, 0, 0, 0};
    auto_endstops[2] = (struct auto_endstop){1, 0,15, 0,202, 50, 0, 0, 0};
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

static int (*real_open)(const char *, int, ...) = NULL;
static int (*real_openat)(int, const char *, int, ...) = NULL;
static int (*real_close)(int) = NULL;
static int (*real_ioctl)(int, unsigned long, ...) = NULL;
static ssize_t (*real_read)(int, void *, size_t) = NULL;
static ssize_t (*real_pread)(int, void *, size_t, off_t) = NULL;
static ssize_t (*real_write)(int, const void *, size_t) = NULL;
static int (*real_fcntl)(int, int, ...) = NULL;
static int (*real_access)(const char *, int) = NULL;

static pthread_t control_thread;
static int control_listen_fd = -1;
static char control_path[256];

static int parse_kv(const char *args, const char *key, long *out) {
    char needle[32];
    snprintf(needle, sizeof(needle), "%s=", key);
    const char *p = strstr(args, needle);
    if (!p) return -1;
    if (out == NULL) return 0;
    p += strlen(needle);
    char *end;
    long v = strtol(p, &end, 10);
    if (end == p) return -1;
    *out = v;
    return 0;
}

static void send_resp(int fd, const char *s) {
    real_write(fd, s, strlen(s));
}

static void control_handle_line(int client_fd, char *line) {
    if (strncmp(line, "ping", 4) == 0) {
        send_resp(client_fd, "ok\n");
        return;
    }
    if (strncmp(line, "set_gpio_input", 14) == 0) {
        long chip, line_off, value;
        if (parse_kv(line, "chip", &chip) < 0
            || parse_kv(line, "line", &line_off) < 0
            || parse_kv(line, "value", &value) < 0) {
            send_resp(client_fd, "error: parse error\n");
            return;
        }
        if (chip < 0 || chip >= MAX_GPIO_CHIPS
            || line_off < 0 || line_off >= MAX_GPIO_LINES) {
            send_resp(client_fd, "error: chip or line out of range\n");
            return;
        }
        pthread_mutex_lock(&gpio_state_mtx);
        gpio_lines[chip][line_off].value = value ? 1 : 0;
        pthread_mutex_unlock(&gpio_state_mtx);
        send_resp(client_fd, "ok\n");
        return;
    }
    if (strncmp(line, "set_adc", 7) == 0) {
        long channel, value;
        if (parse_kv(line, "channel", &channel) < 0
            || parse_kv(line, "value", &value) < 0) {
            send_resp(client_fd, "error: parse error\n");
            return;
        }
        if (channel < 0 || channel >= MAX_IIO_CHANNELS) {
            send_resp(client_fd, "error: channel out of range\n");
            return;
        }
        pthread_mutex_lock(&iio_state_mtx);
        iio_values[channel] = (uint16_t)(value & 0xFFFF);
        pthread_mutex_unlock(&iio_state_mtx);
        send_resp(client_fd, "ok\n");
        return;
    }
    if (strncmp(line, "get_gpio_output", 15) == 0) {
        long chip, line_off;
        if (parse_kv(line, "chip", &chip) < 0
            || parse_kv(line, "line", &line_off) < 0) {
            send_resp(client_fd, "error: parse error\n");
            return;
        }
        if (chip < 0 || chip >= MAX_GPIO_CHIPS
            || line_off < 0 || line_off >= MAX_GPIO_LINES) {
            send_resp(client_fd, "error: chip or line out of range\n");
            return;
        }
        pthread_mutex_lock(&gpio_state_mtx);
        int v = gpio_lines[chip][line_off].value;
        pthread_mutex_unlock(&gpio_state_mtx);
        char buf[32];
        snprintf(buf, sizeof(buf), "value=%d\n", v);
        send_resp(client_fd, buf);
        return;
    }
    if (strncmp(line, "get_pwm", 7) == 0) {
        long chip, pwm;
        if (parse_kv(line, "chip", &chip) < 0
            || parse_kv(line, "pwm", &pwm) < 0) {
            send_resp(client_fd, "error: parse error\n");
            return;
        }
        const char *want_file = "duty_cycle";
        char want_buf[32] = {0};
        const char *p = strstr(line, "file=");
        if (p && sscanf(p, "file=%31s", want_buf) == 1) {
            want_file = want_buf;
        }
        uint64_t found = 0;
        int hit = 0;
        pthread_mutex_lock(&fake_slots_mtx);
        for (int i = 0; i < MAX_FAKE_FDS; i++) {
            if (fake_slots[i].kind == SIM_PWM_FILE
                && fake_slots[i].u.pwm_file.chip_id == chip
                && fake_slots[i].u.pwm_file.pwm_id == pwm
                && strcmp(fake_slots[i].u.pwm_file.file, want_file) == 0) {
                found = fake_slots[i].u.pwm_file.last_value;
                hit = 1;
                break;
            }
        }
        pthread_mutex_unlock(&fake_slots_mtx);
        if (!hit) {
            send_resp(client_fd, "error: pwm file not found\n");
            return;
        }
        char buf[64];
        snprintf(buf, sizeof(buf), "value=%llu\n", (unsigned long long)found);
        send_resp(client_fd, buf);
        return;
    }
    send_resp(client_fd, "error: unknown verb\n");
}

static void *control_accept_loop(void *unused) {
    (void)unused;
    char buf[1024];
    while (1) {
        int client = accept(control_listen_fd, NULL, NULL);
        if (client < 0) {
            if (errno == EINTR) continue;
            LOG("control accept failed: %s", strerror(errno));
            return NULL;
        }
        size_t pos = 0;
        while (1) {
            ssize_t n = real_read(client, buf + pos, sizeof(buf) - pos - 1);
            if (n <= 0) break;
            pos += n;
            buf[pos] = '\0';
            char *nl;
            while ((nl = memchr(buf, '\n', pos)) != NULL) {
                *nl = '\0';
                control_handle_line(client, buf);
                size_t len = nl - buf + 1;
                memmove(buf, nl + 1, pos - len);
                pos -= len;
            }
        }
        real_close(client);
    }
}

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
    const char *sock_dir = getenv("KALICO_SIM_SOCK_DIR");
    if (sock_dir) {
        snprintf(control_path, sizeof(control_path), "%s/sim_control", sock_dir);
        unlink(control_path);
        control_listen_fd = socket(AF_UNIX, SOCK_STREAM, 0);
        if (control_listen_fd < 0) {
            LOG("control socket() failed: %s", strerror(errno));
            return;
        }
        struct sockaddr_un addr;
        memset(&addr, 0, sizeof(addr));
        addr.sun_family = AF_UNIX;
        int sun_written = snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", control_path);
        if (sun_written < 0 || (size_t)sun_written >= sizeof(addr.sun_path)) {
            LOG("control socket path too long: %s", control_path);
            real_close(control_listen_fd);
            control_listen_fd = -1;
            return;
        }
        if (bind(control_listen_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            LOG("control bind %s failed: %s", control_path, strerror(errno));
            real_close(control_listen_fd);
            control_listen_fd = -1;
            return;
        }
        if (listen(control_listen_fd, 1) < 0) {
            LOG("control listen failed: %s", strerror(errno));
            real_close(control_listen_fd);
            control_listen_fd = -1;
            return;
        }
        if (pthread_create(&control_thread, NULL, control_accept_loop, NULL) != 0) {
            LOG("control pthread_create failed");
            real_close(control_listen_fd);
            control_listen_fd = -1;
            return;
        }
        LOG("control listening on %s", control_path);
    }
    LOG("init pid=%d", (int)getpid());
}

__attribute__((destructor))
static void shim_fini(void) {
    if (control_listen_fd >= 0) {
        unlink(control_path);
    }
}

static enum sim_slot_kind classify_path(const char *path) {
    if (!path) return SIM_NONE;
    if (strncmp(path, "/dev/gpiochip", 13) == 0) return SIM_GPIOCHIP;
    if (strncmp(path, "/dev/spidev", 11) == 0) return SIM_SPIDEV;
    if (strncmp(path, "/sys/class/pwm/", 15) == 0) return SIM_PWM_FILE;
    if (strncmp(path, "/sys/bus/iio/", 13) == 0) return SIM_IIO_FILE;
    return SIM_NONE;
}

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
        errno = EINVAL;
        return -1;
    }
    int chip_id = slot->u.gpiochip.chip_id;
    int offset = req->lineoffsets[0];
    if (offset < 0 || offset >= MAX_GPIO_LINES) {
        errno = EINVAL;
        return -1;
    }
    pthread_mutex_lock(&fake_slots_mtx);
    for (int i = 1; i < MAX_FAKE_FDS; i++) {
        if (fake_slots[i].kind == SIM_GPIOLINE
            && fake_slots[i].u.gpioline.chip_id == chip_id
            && fake_slots[i].u.gpioline.line_offset == offset) {
            fake_slots[i].kind = SIM_NONE;
            LOG("gpio re-request chip=%d offset=%d, freed old slot %d", chip_id, offset, i);
        }
    }
    pthread_mutex_unlock(&fake_slots_mtx);
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

static void check_auto_endstops(int chip_id, int offset, int new_value) {
    pthread_mutex_lock(&auto_endstop_mtx);
    for (int i = 0; i < MAX_AUTO_ENDSTOPS; i++) {
        struct auto_endstop *ae = &auto_endstops[i];
        if (!ae->active) continue;
        if (ae->step_chip != chip_id || ae->step_line != offset) continue;
        if (ae->triggered) {
            if (ae->last_step_value == 0 && new_value == 1) {
                ae->step_count++;
                if (ae->step_count >= 10) { // clear after 10 retract steps
                    pthread_mutex_lock(&gpio_state_mtx);
                    gpio_lines[ae->endstop_chip][ae->endstop_line].value = 0;
                    pthread_mutex_unlock(&gpio_state_mtx);
                    ae->triggered = 0;
                    ae->step_count = 0;
                }
            }
            ae->last_step_value = new_value;
            continue;
        }
        if (ae->last_step_value == 0 && new_value == 1) {
            ae->step_count++;
            if (ae->step_count >= ae->trigger_after_steps) {
                pthread_mutex_lock(&gpio_state_mtx);
                gpio_lines[ae->endstop_chip][ae->endstop_line].value = 1;
                pthread_mutex_unlock(&gpio_state_mtx);
                ae->triggered = 1;
                ae->step_count = 0;
            }
        }
        ae->last_step_value = new_value;
    }
    pthread_mutex_unlock(&auto_endstop_mtx);
}

// Exported entry point for the MCU tick thread to notify the auto-endstop
// of step pulses without going through the GPIO ioctl path. The tick
// thread populates the Rust step queues and calls this for each step
// via dlsym(RTLD_DEFAULT, "sim_intercept_notify_step").
__attribute__((visibility("default")))
void sim_intercept_notify_step(int chip, int line, int32_t n_steps) {
    int count = (n_steps < 0) ? -n_steps : n_steps;
    for (int i = 0; i < count; i++) {
        check_auto_endstops(chip, line, 1); // rising edge
        check_auto_endstops(chip, line, 0); // falling edge (reset for next)
    }
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
    if (v == 0 && gpio_lines[chip_id][offset].direction == 1) {
        active_cs.chip_id = chip_id;
        active_cs.line_offset = offset;
        active_cs.valid = 1;
    } else if (v == 1 && active_cs.valid
               && active_cs.chip_id == chip_id
               && active_cs.line_offset == offset) {
        active_cs.valid = 0;
    }
    pthread_mutex_unlock(&gpio_state_mtx);

    check_auto_endstops(chip_id, offset, v);

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
    int written = snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", path);
    if (written < 0 || (size_t)written >= sizeof(addr.sun_path)) {
        LOG("spi sock path too long (%d bytes, max %zu): %s",
            written, sizeof(addr.sun_path) - 1, path);
        real_close(sock);
        errno = ENAMETOOLONG;
        return -1;
    }
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
            if (slot->kind != SIM_SPIDEV) { errno = EINVAL; return -1; }
            slot->u.spidev.speed_hz = *(uint32_t *)arg;
            return 0;
        case SPI_IOC_WR_MODE:
            if (slot->kind != SIM_SPIDEV) { errno = EINVAL; return -1; }
            slot->u.spidev.mode = *(uint8_t *)arg;
            return 0;
        case SPI_IOC_MESSAGE(1):
            if (slot->kind != SIM_SPIDEV) { errno = EINVAL; return -1; }
            return spi_handle_message(fd, (struct spi_ioc_transfer *)arg);
        default:
            LOG("ioctl(fd=%d, req=0x%lx) UNHANDLED on slot kind=%d", fd, req, slot->kind);
            errno = EINVAL;
            return -1;
    }
}

int fcntl(int fd, int cmd, ...) {
    if (!is_fake_fd(fd)) {
        va_list ap; va_start(ap, cmd);
        long arg = va_arg(ap, long);
        va_end(ap);
        return real_fcntl(fd, cmd, arg);
    }
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
