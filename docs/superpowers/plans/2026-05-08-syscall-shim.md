# Syscall-Intercept Sim Shim Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the eight files of sim-aware code in klipper firmware (Category A) with one `LD_PRELOAD` shim under `tools/sim_klippy/preload/`, so MACH_LINUX firmware becomes bit-identical between Pi-Klipper deployments and the test sim.

**Architecture:** A C shared library intercepts the nine libc syscalls (`open`, `openat`, `ioctl`, `read`, `pread`, `write`, `close`, `fcntl`, `access`) that klipper uses to talk to `/dev/gpiochip*`, `/dev/spidev*`, `/sys/class/pwm/*`, and `/sys/bus/iio/*`. Path-based dispatch in `open()` allocates fake fds in the [1<<28, ...) range backed by per-device handler structs. SPI transfers are routed to chip emulator sockets keyed by the currently-asserted GPIO output pin (the CS demultiplex). A control socket exposes a small text protocol so the orchestrator can poke shim-owned state (GPIO inputs for endstops, ADC values for thermistors). One contained firmware exception persists for tmcuart, behind `CONFIG_KALICO_SIM_TMCUART_BYPASS`, gated on an env-var-driven canonical socket path.

**Tech Stack:** C99 (shim), Python 3 (orchestrator client), Klipper firmware C (deletions + the contained tmcuart exception), pthread (control-socket accept thread).

**Spec:** `docs/superpowers/specs/2026-05-08-syscall-shim-design.md`

---

## File Structure

**New files:**

| Path | Purpose |
|---|---|
| `tools/sim_klippy/preload/libsim_intercept.c` | The shim itself (~700 LOC) |
| `tools/sim_klippy/preload/libsim_intercept.h` | Public header (slot kinds, FAKE_FD_BASE) |
| `tools/sim_klippy/preload/Makefile` | Build rule for `libsim_intercept.so` |
| `tools/sim_klippy/preload/README.md` | What it is, how to debug it |
| `tools/sim_klippy/preload/tests/test_shim.c` | C test harness (dlopen + direct calls) |
| `tools/sim_klippy/preload/tests/Makefile` | Build + run rules for test_shim |
| `tools/sim_klippy/orchestrator/sim_control_client.py` | Python client for control socket |
| `tools/sim_klippy/tests/test_sim_control.py` | Pytest for sim_control_client.py |

**Modified files:**

| Path | Change |
|---|---|
| `tools/sim_klippy/orchestrator/launcher.py` | Set `LD_PRELOAD` + `KALICO_SIM_SOCK_DIR`; build shim before spawn |
| `tools/sim_klippy/conftest.py` | Add `SimContext.sim_control` field; bind chip emulator sockets at shim-expected paths |
| `tools/sim_klippy/Dockerfile` | (no change needed — gcc is already in the image) |
| `tools/sim_klippy/run_local.sh` | Build the shim before invoking the test |
| `tools/sim_klippy/configs/h7-sim.config` | `# CONFIG_KALICO_SIM is not set`, `CONFIG_KALICO_SIM_TMCUART_BYPASS=y` |
| `tools/sim_klippy/configs/f4-sim.config` | Same |
| `klippy/motion_toolhead.py` | `cmd_KALICO_SIM_ENDSTOP_SET_PIN` repointed to control socket |
| `src/Kconfig` | Add `CONFIG_KALICO_SIM_TMCUART_BYPASS` flag |
| `src/Makefile` | Drop `runtime_sim_commands.c` from build |
| `src/linux/Makefile` | Drop `sim_chip_socket.c` from build |
| `src/tmcuart.c` | Replace `flavor` heuristic + auto-route with env-var path; gate sim block on `CONFIG_KALICO_SIM_TMCUART_BYPASS` |
| `src/linux/gpio.c` | Delete all `#if CONFIG_KALICO_SIM` blocks + `sim_gpio_*` helpers |
| `src/linux/hard_pwm.c` | Delete `#if CONFIG_KALICO_SIM` block |
| `src/linux/analog.c` | Delete sim ADC table + `analog_set_simulated_value` + sim fallback |
| `src/linux/spidev.c` | Delete sim-route branch in `spi_setup` and `spi_transfer` |
| `src/spicmds.c` | Delete `sim_pending_cs` plumbing |

**Deleted files:**

| Path |
|---|
| `src/linux/sim_chip_socket.c` |
| `src/linux/sim_chip_socket.h` |
| `src/runtime_sim_commands.c` |

---

## Phase 1: Shim foundation

### Task 1: Skeleton + Makefile + first intercepted symbol

**Files:**
- Create: `tools/sim_klippy/preload/libsim_intercept.c`
- Create: `tools/sim_klippy/preload/libsim_intercept.h`
- Create: `tools/sim_klippy/preload/Makefile`
- Create: `tools/sim_klippy/preload/README.md`

- [ ] **Step 1: Create the directory + Makefile**

```bash
mkdir -p tools/sim_klippy/preload tools/sim_klippy/preload/tests
```

Create `tools/sim_klippy/preload/Makefile`:

```makefile
CC ?= gcc
CFLAGS = -O2 -Wall -Werror -fPIC -D_GNU_SOURCE -pthread
LDFLAGS = -shared -ldl -lpthread

all: libsim_intercept.so

libsim_intercept.so: libsim_intercept.c libsim_intercept.h
	$(CC) $(CFLAGS) -o $@ libsim_intercept.c $(LDFLAGS)

clean:
	rm -f libsim_intercept.so

.PHONY: all clean
```

- [ ] **Step 2: Create the public header**

Create `tools/sim_klippy/preload/libsim_intercept.h`:

```c
#ifndef LIBSIM_INTERCEPT_H
#define LIBSIM_INTERCEPT_H

#include <stdint.h>

// Fake-fd base — well above any real Linux fd allocation
// (RLIMIT_NOFILE defaults to 1024, hard limit ~1M).
#define FAKE_FD_BASE 0x10000000
#define MAX_FAKE_FDS 256

// Slot kinds — see spec §"Per-fd state and slot allocation"
enum sim_slot_kind {
    SIM_NONE = 0,
    SIM_GPIOCHIP,
    SIM_GPIOLINE,
    SIM_SPIDEV,
    SIM_PWM_FILE,
    SIM_IIO_FILE,
};

#endif
```

- [ ] **Step 3: Create the skeleton .c file**

Create `tools/sim_klippy/preload/libsim_intercept.c`:

```c
// LD_PRELOAD shim that intercepts klipper's libc syscalls and replaces
// /dev/gpiochip*, /dev/spidev*, /sys/class/pwm/*, /sys/bus/iio/* access
// with sim-internal state and chip emulator sockets.
//
// See docs/superpowers/specs/2026-05-08-syscall-shim-design.md
#define _GNU_SOURCE
#include "libsim_intercept.h"

#include <dlfcn.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

// Verbose logging — set KALICO_SIM_SHIM_VERBOSE=1 to enable.
static int verbose = 0;
#define LOG(fmt, ...) do { if (verbose) fprintf(stderr, "[shim] " fmt "\n", ##__VA_ARGS__); } while (0)

__attribute__((constructor))
static void shim_init(void) {
    const char *v = getenv("KALICO_SIM_SHIM_VERBOSE");
    verbose = (v && v[0] == '1');
    LOG("init pid=%d", (int)getpid());
}
```

- [ ] **Step 4: Build the skeleton**

Run: `make -C tools/sim_klippy/preload`
Expected: `libsim_intercept.so` created, no warnings.

- [ ] **Step 5: Smoke-test that LD_PRELOAD loads the shim**

Run inside Docker:
```bash
docker run --rm -v $PWD:/work -w /work --tmpfs /tmp:exec kalico-sim:latest bash -c "
  make -C tools/sim_klippy/preload
  KALICO_SIM_SHIM_VERBOSE=1 LD_PRELOAD=/work/tools/sim_klippy/preload/libsim_intercept.so /bin/true 2>&1 | head -3
"
```

Expected: `[shim] init pid=<N>` line printed.

- [ ] **Step 6: Create the README**

Create `tools/sim_klippy/preload/README.md`:

```markdown
# libsim_intercept.so — sim shim

LD_PRELOAD shim that lets klipper.elf (built for MACH_LINUX) run inside
the faithful sim without any sim-aware firmware code. Replaces
`/dev/gpiochip*` / `/dev/spidev*` / `/sys/class/pwm/*` / `/sys/bus/iio/*`
device access with shim-internal state plus per-chip Unix sockets.

## Build
    make

## Use
    LD_PRELOAD=$PWD/libsim_intercept.so \
    KALICO_SIM_SOCK_DIR=/tmp/sim/ \
    /path/to/klipper.elf -I /tmp/klipper_sim_pty

## Debug
    KALICO_SIM_SHIM_VERBOSE=1 LD_PRELOAD=...

Each intercept logs a one-line trace to stderr.

## Spec
See `docs/superpowers/specs/2026-05-08-syscall-shim-design.md`.
```

- [ ] **Step 7: Commit**

```bash
git add tools/sim_klippy/preload/
git commit -m "shim(skeleton): libsim_intercept.so foundation + Makefile + README"
```

---

### Task 2: Per-fd slot table + allocator

**Files:**
- Modify: `tools/sim_klippy/preload/libsim_intercept.c`

- [ ] **Step 1: Add the slot struct to the .c file**

Append to `libsim_intercept.c`, after the `LOG` macro:

```c
#include <pthread.h>

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
```

Add `#include <errno.h>` to the includes if not already there.

- [ ] **Step 2: Build**

Run: `make -C tools/sim_klippy/preload`
Expected: clean build, no warnings.

- [ ] **Step 3: Commit**

```bash
git add tools/sim_klippy/preload/libsim_intercept.c
git commit -m "shim(slots): per-fd slot table + allocator"
```

---

### Task 3: Intercept open/openat/access — path dispatch skeleton

**Files:**
- Modify: `tools/sim_klippy/preload/libsim_intercept.c`

- [ ] **Step 1: Add real-symbol resolution in shim_init**

Replace the body of `shim_init()` in `libsim_intercept.c`:

```c
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
```

- [ ] **Step 2: Add the path classifier**

Append to `libsim_intercept.c`:

```c
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
```

- [ ] **Step 3: Add stub open/openat handlers (return -1 ENOSYS for sim paths)**

Append:

```c
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
```

Add `#include <sys/types.h>` if not already present.

- [ ] **Step 4: Build**

Run: `make -C tools/sim_klippy/preload`
Expected: clean build.

- [ ] **Step 5: Commit**

```bash
git add tools/sim_klippy/preload/libsim_intercept.c
git commit -m "shim(open): path-based dispatch skeleton + access intercept"
```

---

### Task 4: GPIO chip handler

**Files:**
- Modify: `tools/sim_klippy/preload/libsim_intercept.c`

- [ ] **Step 1: Add GPIO state**

Append to `libsim_intercept.c` near the slot table:

```c
#include <linux/gpio.h>

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
```

- [ ] **Step 2: Implement sim_open_gpiochip**

Replace the stub `sim_open_gpiochip`:

```c
static int sim_open_gpiochip(const char *path, int flags) {
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
```

- [ ] **Step 3: Implement GPIO ioctl handlers**

Append:

```c
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
```

- [ ] **Step 4: Implement ioctl/fcntl/close interception**

Append:

```c
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
    return 0;
}

int close(int fd) {
    if (!is_fake_fd(fd)) return real_close(fd);
    free_fake_fd(fd);
    return 0;
}
```

- [ ] **Step 5: Build**

Run: `make -C tools/sim_klippy/preload`
Expected: clean build, no warnings.

- [ ] **Step 6: Commit**

```bash
git add tools/sim_klippy/preload/libsim_intercept.c
git commit -m "shim(gpio): chip + line handlers; ioctl/fcntl/close routing"
```

---

### Task 5: SPI handler

**Files:**
- Modify: `tools/sim_klippy/preload/libsim_intercept.c`

- [ ] **Step 1: Implement sim_open_spidev**

Replace the stub `sim_open_spidev`:

```c
static int sim_open_spidev(const char *path, int flags) {
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
```

- [ ] **Step 2: Add chip socket connect helper**

Append (uses the active CS to pick the per-chip socket path):

```c
#include <sys/socket.h>
#include <sys/un.h>

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
    snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", path);
    if (connect(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        LOG("spi connect %s failed: %s", path, strerror(errno));
        real_close(sock);
        return -1;
    }
    slot->u.spidev.chip_socket_fd = sock;
    LOG("spi connected to %s -> sock=%d", path, sock);
    return sock;
}

// Note: spi_get_chip_socket caches per-slot, but multiple chips share
// one slot when CS toggles. Reset the cache on each transfer so the next
// CS re-selects the right chip.
static void spi_drop_chip_socket(struct sim_fd_slot *slot) {
    if (slot->u.spidev.chip_socket_fd >= 0) {
        real_close(slot->u.spidev.chip_socket_fd);
        slot->u.spidev.chip_socket_fd = -1;
    }
}
```

- [ ] **Step 3: Add SPI ioctl + write handlers**

Append:

```c
#include <linux/spi/spidev.h>

static int spi_handle_message(int fd, struct spi_ioc_transfer *xfer) {
    struct sim_fd_slot *slot = slot_for_fd(fd);
    if (!slot || slot->kind != SIM_SPIDEV) { errno = EBADF; return -1; }
    int sock = spi_get_chip_socket(slot);
    if (sock < 0) return -1;
    // Send tx bytes. Read rx_buf bytes back.
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
    // Drop cached socket so next CS toggle picks a fresh chip.
    spi_drop_chip_socket(slot);
    return 0;
}
```

- [ ] **Step 4: Wire SPI ioctls into the main ioctl dispatch**

Edit the `ioctl` function — add cases before the default:

```c
        case SPI_IOC_WR_MAX_SPEED_HZ:
            slot->u.spidev.speed_hz = *(uint32_t *)arg;
            return 0;
        case SPI_IOC_WR_MODE:
            slot->u.spidev.mode = *(uint8_t *)arg;
            return 0;
        case SPI_IOC_MESSAGE(1):
            return spi_handle_message(fd, (struct spi_ioc_transfer *)arg);
```

(Note: `SPI_IOC_MESSAGE(N)` for N=1 is a constant; the macro expands at compile time.)

- [ ] **Step 5: Add SPI write fallback (no-receive path)**

Add a new top-level `write` function after `close`:

```c
ssize_t write(int fd, const void *buf, size_t count) {
    if (!is_fake_fd(fd)) return real_write(fd, buf, count);
    struct sim_fd_slot *slot = slot_for_fd(fd);
    if (!slot) { errno = EBADF; return -1; }
    switch (slot->kind) {
        case SIM_SPIDEV: {
            // No-receive SPI write — same as a one-way transfer.
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
        case SIM_PWM_FILE:
            // Filled in Task 6.
            errno = EINVAL;
            return -1;
        default:
            errno = EINVAL;
            return -1;
    }
}
```

- [ ] **Step 6: Build**

Run: `make -C tools/sim_klippy/preload`
Expected: clean build.

- [ ] **Step 7: Commit**

```bash
git add tools/sim_klippy/preload/libsim_intercept.c
git commit -m "shim(spi): spidev open + ioctl + write; CS-keyed chip socket dispatch"
```

---

### Task 6: PWM handler

**Files:**
- Modify: `tools/sim_klippy/preload/libsim_intercept.c`

- [ ] **Step 1: Implement sim_open_pwm**

Replace the stub `sim_open_pwm`:

```c
static int sim_open_pwm(const char *path, int flags) {
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
```

- [ ] **Step 2: Wire PWM into write()**

Replace the `case SIM_PWM_FILE:` branch in `write()`:

```c
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
```

Add `#include <stdlib.h>` if not present.

- [ ] **Step 3: Build**

Run: `make -C tools/sim_klippy/preload`
Expected: clean build.

- [ ] **Step 4: Commit**

```bash
git add tools/sim_klippy/preload/libsim_intercept.c
git commit -m "shim(pwm): open + write absorb sysfs PWM file traffic"
```

---

### Task 7: IIO ADC handler

**Files:**
- Modify: `tools/sim_klippy/preload/libsim_intercept.c`

- [ ] **Step 1: Add IIO state**

Append near the GPIO state:

```c
#define MAX_IIO_CHANNELS 32
#define DEFAULT_ADC_VALUE 3900
static uint16_t iio_values[MAX_IIO_CHANNELS];
static pthread_mutex_t iio_state_mtx = PTHREAD_MUTEX_INITIALIZER;

__attribute__((constructor(101)))
static void iio_init(void) {
    for (int i = 0; i < MAX_IIO_CHANNELS; i++) iio_values[i] = DEFAULT_ADC_VALUE;
}
```

- [ ] **Step 2: Implement sim_open_iio**

Replace the stub `sim_open_iio`:

```c
static int sim_open_iio(const char *path, int flags) {
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
```

- [ ] **Step 3: Implement pread + read for IIO**

Append a new top-level `pread`:

```c
ssize_t pread(int fd, void *buf, size_t count, off_t offset) {
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
```

- [ ] **Step 4: Build**

Run: `make -C tools/sim_klippy/preload`
Expected: clean build.

- [ ] **Step 5: Commit**

```bash
git add tools/sim_klippy/preload/libsim_intercept.c
git commit -m "shim(iio): ADC sysfs reads via pread/read; default 3900"
```

---

## Phase 2: Control socket

### Task 8: Control socket server thread + ping verb

**Files:**
- Modify: `tools/sim_klippy/preload/libsim_intercept.c`

- [ ] **Step 1: Add the accept thread state**

Append:

```c
#include <sys/types.h>

static pthread_t control_thread;
static int control_listen_fd = -1;
static char control_path[256];

static void control_handle_line(int client_fd, char *line) {
    // Stub — Tasks 9-10 fill this in.
    if (strncmp(line, "ping", 4) == 0) {
        const char *resp = "ok\n";
        real_write(client_fd, resp, 3);
        return;
    }
    const char *err = "error: unknown verb\n";
    real_write(client_fd, err, strlen(err));
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
```

- [ ] **Step 2: Bind and start the accept thread in shim_init**

Append to `shim_init` body, before the final `LOG`:

```c
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
        snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", control_path);
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
```

Add `__attribute__((destructor))` to clean up on exit:

```c
__attribute__((destructor))
static void shim_fini(void) {
    if (control_listen_fd >= 0) {
        unlink(control_path);
    }
}
```

- [ ] **Step 3: Build**

Run: `make -C tools/sim_klippy/preload`
Expected: clean build.

- [ ] **Step 4: Manual smoke-test of ping**

```bash
docker run --rm -v $PWD:/work -w /work --tmpfs /tmp:exec kalico-sim:latest bash -c "
  make -C tools/sim_klippy/preload >/dev/null
  mkdir -p /tmp/sim
  KALICO_SIM_SOCK_DIR=/tmp/sim LD_PRELOAD=/work/tools/sim_klippy/preload/libsim_intercept.so sleep 5 &
  sleep 0.5
  echo 'ping' | nc -U /tmp/sim/sim_control
  kill %1
"
```

Expected: `ok` printed.

- [ ] **Step 5: Commit**

```bash
git add tools/sim_klippy/preload/libsim_intercept.c
git commit -m "shim(control): accept thread + ping verb on \$KALICO_SIM_SOCK_DIR/sim_control"
```

---

### Task 9: set_gpio_input + set_adc

**Files:**
- Modify: `tools/sim_klippy/preload/libsim_intercept.c`

- [ ] **Step 1: Replace control_handle_line stub**

```c
static int parse_kv(const char *args, const char *key, long *out) {
    char needle[32];
    snprintf(needle, sizeof(needle), "%s=", key);
    const char *p = strstr(args, needle);
    if (!p) return -1;
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
    send_resp(client_fd, "error: unknown verb\n");
}
```

- [ ] **Step 2: Build**

Run: `make -C tools/sim_klippy/preload`
Expected: clean build.

- [ ] **Step 3: Smoke-test set commands**

```bash
docker run --rm -v $PWD:/work -w /work --tmpfs /tmp:exec kalico-sim:latest bash -c "
  make -C tools/sim_klippy/preload >/dev/null
  mkdir -p /tmp/sim
  KALICO_SIM_SOCK_DIR=/tmp/sim LD_PRELOAD=/work/tools/sim_klippy/preload/libsim_intercept.so sleep 5 &
  sleep 0.5
  echo 'set_gpio_input chip=0 line=20 value=1' | nc -U /tmp/sim/sim_control
  echo 'set_adc channel=3 value=2048' | nc -U /tmp/sim/sim_control
  echo 'set_gpio_input chip=99 line=0 value=1' | nc -U /tmp/sim/sim_control
  kill %1
"
```

Expected: two `ok`, then `error: chip or line out of range`.

- [ ] **Step 4: Commit**

```bash
git add tools/sim_klippy/preload/libsim_intercept.c
git commit -m "shim(control): set_gpio_input + set_adc verbs"
```

---

### Task 10: get_gpio_output + get_pwm verbs

**Files:**
- Modify: `tools/sim_klippy/preload/libsim_intercept.c`

- [ ] **Step 1: Add get_gpio_output handler**

In `control_handle_line`, before the `error: unknown verb` line, add:

```c
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
        // Search slot table for matching pwm file (file=duty_cycle by default).
        const char *want_file = "duty_cycle";
        char want_buf[32];
        if (parse_kv(line, "file", NULL) == 0) {
            // crude: re-extract the file= substring
            const char *p = strstr(line, "file=");
            if (p) {
                int n = sscanf(p, "file=%31s", want_buf);
                if (n == 1) want_file = want_buf;
            }
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
```

- [ ] **Step 2: Fix the `parse_kv` quirk used above**

`parse_kv` in Task 9 takes `long *out`. The `get_pwm` code calls it with `NULL` to test for key presence — that's a bug. Replace `parse_kv` with a version tolerant to `NULL`:

```c
static int parse_kv(const char *args, const char *key, long *out) {
    char needle[32];
    snprintf(needle, sizeof(needle), "%s=", key);
    const char *p = strstr(args, needle);
    if (!p) return -1;
    if (out == NULL) return 0;          // presence-only check
    p += strlen(needle);
    char *end;
    long v = strtol(p, &end, 10);
    if (end == p) return -1;
    *out = v;
    return 0;
}
```

- [ ] **Step 3: Build**

Run: `make -C tools/sim_klippy/preload`
Expected: clean build.

- [ ] **Step 4: Commit**

```bash
git add tools/sim_klippy/preload/libsim_intercept.c
git commit -m "shim(control): get_gpio_output + get_pwm verbs"
```

---

## Phase 3: Direct shim tests

### Task 11: C test harness

**Files:**
- Create: `tools/sim_klippy/preload/tests/test_shim.c`
- Create: `tools/sim_klippy/preload/tests/Makefile`

- [ ] **Step 1: Create the test Makefile**

```makefile
CC ?= gcc
CFLAGS = -O2 -Wall -Werror -D_GNU_SOURCE -pthread -I..
LDFLAGS = -ldl -lpthread

all: test_shim

test_shim: test_shim.c
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

run: test_shim
	./test_shim

clean:
	rm -f test_shim
.PHONY: all run clean
```

- [ ] **Step 2: Create the test harness**

Create `tools/sim_klippy/preload/tests/test_shim.c`:

```c
// Direct test harness for libsim_intercept.so. Loads the shim via
// LD_PRELOAD (not dlopen — we rely on the constructor running in the
// process before main() runs), then exercises each handler.
//
// Run via: ./test_shim   (the Makefile sets LD_PRELOAD=../libsim_intercept.so)
#define _GNU_SOURCE
#include <assert.h>
#include <fcntl.h>
#include <linux/gpio.h>
#include <linux/spi/spidev.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#define ASSERT_EQ(a, b) do { \
    long _a = (long)(a), _b = (long)(b); \
    if (_a != _b) { \
        fprintf(stderr, "FAIL %s:%d: %s = %ld != %ld\n", __FILE__, __LINE__, #a, _a, _b); \
        return 1; \
    } \
} while (0)

static int test_gpio_open_returns_fake_fd(void) {
    int fd = open("/dev/gpiochip0", O_RDWR);
    ASSERT_EQ(fd >= 0x10000000, 1);
    close(fd);
    return 0;
}

static int test_gpio_line_handle_roundtrip(void) {
    int chip = open("/dev/gpiochip0", O_RDWR);
    ASSERT_EQ(chip >= 0, 1);
    struct gpiohandle_request req = {0};
    req.lines = 1;
    req.flags = GPIOHANDLE_REQUEST_OUTPUT;
    req.lineoffsets[0] = 5;
    req.default_values[0] = 1;
    snprintf(req.consumer_label, sizeof(req.consumer_label), "test");
    ASSERT_EQ(ioctl(chip, GPIO_GET_LINEHANDLE_IOCTL, &req), 0);
    ASSERT_EQ(req.fd >= 0x10000000, 1);
    struct gpiohandle_data data = {0};
    ASSERT_EQ(ioctl(req.fd, GPIOHANDLE_GET_LINE_VALUES_IOCTL, &data), 0);
    ASSERT_EQ(data.values[0], 1);
    data.values[0] = 0;
    ASSERT_EQ(ioctl(req.fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data), 0);
    data.values[0] = 99;
    ASSERT_EQ(ioctl(req.fd, GPIOHANDLE_GET_LINE_VALUES_IOCTL, &data), 0);
    ASSERT_EQ(data.values[0], 0);
    close(req.fd);
    close(chip);
    return 0;
}

static int test_iio_default_value(void) {
    int fd = open("/sys/bus/iio/devices/iio:device0/in_voltage3_raw", O_RDONLY);
    ASSERT_EQ(fd >= 0x10000000, 1);
    char buf[16] = {0};
    ssize_t n = pread(fd, buf, sizeof(buf) - 1, 0);
    ASSERT_EQ(n > 0, 1);
    ASSERT_EQ(atoi(buf), 3900);
    close(fd);
    return 0;
}

static int test_pwm_write_absorbed(void) {
    int fd = open("/sys/class/pwm/pwmchip0/pwm0/period", O_WRONLY);
    ASSERT_EQ(fd >= 0x10000000, 1);
    const char *s = "1000000";
    ASSERT_EQ(write(fd, s, 7), 7);
    close(fd);
    return 0;
}

static int test_control_socket_ping(void) {
    const char *dir = getenv("KALICO_SIM_SOCK_DIR");
    if (!dir) { fprintf(stderr, "SKIP: KALICO_SIM_SOCK_DIR not set\n"); return 0; }
    char path[256];
    snprintf(path, sizeof(path), "%s/sim_control", dir);
    int sock = socket(AF_UNIX, SOCK_STREAM, 0);
    ASSERT_EQ(sock >= 0, 1);
    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", path);
    if (connect(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("connect");
        return 1;
    }
    const char *req = "ping\n";
    ASSERT_EQ(write(sock, req, 5), 5);
    char reply[8] = {0};
    ssize_t n = read(sock, reply, sizeof(reply) - 1);
    ASSERT_EQ(n > 0, 1);
    ASSERT_EQ(strncmp(reply, "ok", 2), 0);
    close(sock);
    return 0;
}

#define RUN(t) do { \
    fprintf(stderr, "RUN %s\n", #t); \
    if (t() != 0) { fails++; fprintf(stderr, "FAIL %s\n", #t); } \
    else { fprintf(stderr, "PASS %s\n", #t); } \
} while (0)

int main(void) {
    int fails = 0;
    RUN(test_gpio_open_returns_fake_fd);
    RUN(test_gpio_line_handle_roundtrip);
    RUN(test_iio_default_value);
    RUN(test_pwm_write_absorbed);
    RUN(test_control_socket_ping);
    fprintf(stderr, "DONE fails=%d\n", fails);
    return fails ? 1 : 0;
}
```

- [ ] **Step 3: Run the harness**

```bash
docker run --rm -v $PWD:/work -w /work --tmpfs /tmp:exec kalico-sim:latest bash -c "
  set -e
  make -C tools/sim_klippy/preload >/dev/null
  make -C tools/sim_klippy/preload/tests >/dev/null
  mkdir -p /tmp/sim
  KALICO_SIM_SOCK_DIR=/tmp/sim \
    LD_PRELOAD=/work/tools/sim_klippy/preload/libsim_intercept.so \
    /work/tools/sim_klippy/preload/tests/test_shim
"
```

Expected: 5x `PASS`, `DONE fails=0`.

- [ ] **Step 4: Commit**

```bash
git add tools/sim_klippy/preload/tests/
git commit -m "shim(test): direct C test harness for shim handlers"
```

---

## Phase 4: Orchestrator integration

### Task 12: Python sim_control_client

**Files:**
- Create: `tools/sim_klippy/orchestrator/sim_control_client.py`
- Create: `tools/sim_klippy/tests/test_sim_control.py`

- [ ] **Step 1: Write the client**

Create `tools/sim_klippy/orchestrator/sim_control_client.py`:

```python
"""Client for the LD_PRELOAD shim's control socket.

Wire format: line-oriented text. See
docs/superpowers/specs/2026-05-08-syscall-shim-design.md §"Control socket
protocol" for the grammar.
"""
import socket
import threading


class SimControlError(Exception):
    pass


class SimControlClient:
    """Synchronous client. Single-threaded usage; instantiate in tests."""

    def __init__(self, socket_path: str, timeout: float = 5.0):
        self.socket_path = socket_path
        self.timeout = timeout
        self._lock = threading.Lock()
        self._sock = None

    def connect(self) -> None:
        if self._sock is not None:
            return
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.settimeout(self.timeout)
        s.connect(self.socket_path)
        self._sock = s

    def close(self) -> None:
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *args):
        self.close()

    def _send_recv(self, line: str) -> str:
        with self._lock:
            self.connect()
            self._sock.sendall((line + "\n").encode("ascii"))
            buf = b""
            while b"\n" not in buf:
                chunk = self._sock.recv(256)
                if not chunk:
                    raise SimControlError("control socket closed unexpectedly")
                buf += chunk
            reply = buf.split(b"\n", 1)[0].decode("ascii")
            if reply.startswith("error:"):
                raise SimControlError(reply)
            return reply

    def ping(self) -> None:
        r = self._send_recv("ping")
        if r != "ok":
            raise SimControlError(f"unexpected ping reply: {r}")

    def set_gpio_input(self, chip: int, line: int, value: int) -> None:
        r = self._send_recv(f"set_gpio_input chip={chip} line={line} value={value}")
        if r != "ok":
            raise SimControlError(f"unexpected reply: {r}")

    def set_adc(self, channel: int, value: int) -> None:
        r = self._send_recv(f"set_adc channel={channel} value={value}")
        if r != "ok":
            raise SimControlError(f"unexpected reply: {r}")

    def get_gpio_output(self, chip: int, line: int) -> int:
        r = self._send_recv(f"get_gpio_output chip={chip} line={line}")
        if not r.startswith("value="):
            raise SimControlError(f"unexpected reply: {r}")
        return int(r[len("value="):])

    def get_pwm(self, chip: int, pwm: int, file: str = "duty_cycle") -> int:
        r = self._send_recv(f"get_pwm chip={chip} pwm={pwm} file={file}")
        if not r.startswith("value="):
            raise SimControlError(f"unexpected reply: {r}")
        return int(r[len("value="):])
```

- [ ] **Step 2: Write the failing test**

Create `tools/sim_klippy/tests/test_sim_control.py`:

```python
"""Pytest for sim_control_client.py against a running shim."""
import os
import subprocess
import time

import pytest

from tools.sim_klippy.orchestrator.sim_control_client import (
    SimControlClient,
    SimControlError,
)


REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))


@pytest.fixture
def shim_under_sleep(tmp_path):
    """Spawn /bin/sleep with the shim loaded; yield the control-socket path."""
    sock_dir = tmp_path / "sim"
    sock_dir.mkdir()
    shim = os.path.join(REPO_ROOT, "tools/sim_klippy/preload/libsim_intercept.so")
    assert os.path.exists(shim), "build shim first: make -C tools/sim_klippy/preload"
    env = os.environ.copy()
    env["LD_PRELOAD"] = shim
    env["KALICO_SIM_SOCK_DIR"] = str(sock_dir)
    p = subprocess.Popen(["/bin/sleep", "10"], env=env)
    deadline = time.time() + 3.0
    sock_path = sock_dir / "sim_control"
    while time.time() < deadline:
        if sock_path.exists():
            break
        time.sleep(0.05)
    else:
        p.terminate()
        pytest.fail("control socket never appeared")
    yield str(sock_path)
    p.terminate()
    p.wait()


def test_ping(shim_under_sleep):
    with SimControlClient(shim_under_sleep) as c:
        c.ping()


def test_set_and_get_gpio(shim_under_sleep):
    with SimControlClient(shim_under_sleep) as c:
        c.set_gpio_input(chip=0, line=20, value=1)
        # Note: set_gpio_input updates the shared GPIO table; reading
        # back via get_gpio_output reads the same table.
        assert c.get_gpio_output(chip=0, line=20) == 1


def test_set_adc(shim_under_sleep):
    with SimControlClient(shim_under_sleep) as c:
        c.set_adc(channel=3, value=2048)
        # No direct getter for ADC — verify by absence of error.


def test_unknown_verb(shim_under_sleep):
    with SimControlClient(shim_under_sleep) as c:
        with pytest.raises(SimControlError):
            c._send_recv("bogus_verb")
```

- [ ] **Step 3: Run the test (must build shim first)**

```bash
docker run --rm -v $PWD:/work -w /work --tmpfs /tmp:exec kalico-sim:latest bash -c "
  make -C tools/sim_klippy/preload >/dev/null
  python3 -m pytest tools/sim_klippy/tests/test_sim_control.py -x -q -s
"
```

Expected: 4 passed.

- [ ] **Step 4: Commit**

```bash
git add tools/sim_klippy/orchestrator/sim_control_client.py tools/sim_klippy/tests/test_sim_control.py
git commit -m "shim(client): Python SimControlClient + pytest"
```

---

### Task 13: launcher.py — set LD_PRELOAD + build the shim

**Files:**
- Modify: `tools/sim_klippy/orchestrator/launcher.py`

- [ ] **Step 1: Read the current launcher to confirm structure**

Run: `wc -l tools/sim_klippy/orchestrator/launcher.py` and read it.

- [ ] **Step 2: Add shim build helper**

In `launcher.py`, near the top (after imports):

```python
def _ensure_shim_built(repo_root: pathlib.Path) -> pathlib.Path:
    """Build libsim_intercept.so if missing or outdated. Returns absolute path."""
    preload_dir = repo_root / "tools" / "sim_klippy" / "preload"
    so_path = preload_dir / "libsim_intercept.so"
    src_path = preload_dir / "libsim_intercept.c"
    needs_build = (
        not so_path.exists()
        or so_path.stat().st_mtime < src_path.stat().st_mtime
    )
    if needs_build:
        subprocess.check_call(["make", "-C", str(preload_dir)])
    return so_path
```

Add `import pathlib` if not present.

- [ ] **Step 3: Wire LD_PRELOAD + KALICO_SIM_SOCK_DIR into _spawn_one**

Modify `_spawn_one` signature and body (existing function takes `elf, socket_path, log_path, name`):

```python
def _spawn_one(elf: str, socket_path: str, log_path: str,
               name: str, sock_dir: str, shim_so: str) -> McuHandle:
    if os.path.exists(socket_path):
        os.unlink(socket_path)
    log_fd = open(log_path, "wb")
    env = os.environ.copy()
    env["LD_PRELOAD"] = shim_so
    env["KALICO_SIM_SOCK_DIR"] = sock_dir
    proc = subprocess.Popen(
        [elf, "-I", socket_path],
        stdout=log_fd,
        stderr=subprocess.STDOUT,
        env=env,
    )
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        if os.path.exists(socket_path):
            return McuHandle(
                name=name, process=proc,
                socket_path=socket_path, log_path=log_path,
            )
        if proc.poll() is not None:
            log_fd.close()
            log_content = open(log_path).read()
            raise RuntimeError(
                f"{name}: klipper.elf exited early (rc={proc.returncode})\n"
                f"---log---\n{log_content}"
            )
        time.sleep(0.05)
    proc.kill()
    log_fd.close()
    raise RuntimeError(f"{name}: PTY {socket_path} did not appear in 5s")
```

- [ ] **Step 4: Modify spawn_mcus to compute sock_dir + shim path**

In `spawn_mcus`, before each `_spawn_one` call:

```python
def spawn_mcus(
    h7_elf: str,
    f4_elf: str,
    h7_socket: str,
    f4_socket: str,
    log_dir: str,
    repo_root: pathlib.Path | None = None,
) -> McuHandles:
    if repo_root is None:
        repo_root = pathlib.Path(__file__).resolve().parents[3]
    shim_so = str(_ensure_shim_built(repo_root))
    # Per-MCU sock_dir: shared root, per-MCU subdirs so each shim
    # creates its own sim_control + chip sockets without collision.
    sock_root = pathlib.Path(log_dir).parent / "sim"
    sock_root.mkdir(exist_ok=True)
    h7_sock_dir = sock_root / "h7"
    f4_sock_dir = sock_root / "f4"
    h7_sock_dir.mkdir(exist_ok=True)
    f4_sock_dir.mkdir(exist_ok=True)
    h7 = _spawn_one(h7_elf, h7_socket,
                    os.path.join(log_dir, "h7.log"), "h7",
                    str(h7_sock_dir), shim_so)
    f4 = _spawn_one(f4_elf, f4_socket,
                    os.path.join(log_dir, "f4.log"), "f4",
                    str(f4_sock_dir), shim_so)
    return McuHandles(h7=h7, f4=f4)
```

- [ ] **Step 5: Build + sanity-run boot test**

```bash
docker run --rm -v $PWD:/work -w /work --tmpfs /tmp:exec kalico-sim:latest bash -c "
  make -C tools/sim_klippy/preload
  python3 -m pytest tools/sim_klippy/tests/test_boot.py -x -q -s 2>&1 | tail -10
"
```

Expected: at this stage, **firmware still has KALICO_SIM=y**, so the shim's interception is mostly idle (firmware short-circuits before the syscalls). Test should PASS — we're verifying the shim doesn't break anything.

- [ ] **Step 6: Commit**

```bash
git add tools/sim_klippy/orchestrator/launcher.py
git commit -m "shim(launcher): build + LD_PRELOAD libsim_intercept.so per MCU"
```

---

### Task 14: conftest.py — expose SimContext.sim_control + bind chip sockets

**Files:**
- Modify: `tools/sim_klippy/conftest.py`

- [ ] **Step 1: Read current conftest.py around the chip-socket binding**

Identify the H7 SPI router setup (around lines 218-251 per the earlier
review) and the SimContext dataclass.

- [ ] **Step 2: Add sim_control_dir to SimContext**

Find the `@dataclasses.dataclass` for `SimContext` and add:

```python
@dataclasses.dataclass
class SimContext:
    mcus: McuHandles
    chip_servers: list
    beacon: BeaconSerialStub
    klippy_proc: subprocess.Popen
    klippy_log: pathlib.Path
    api_socket: str
    log_dir: pathlib.Path
    h7_sim_control: str
    f4_sim_control: str
    # ... existing fields
```

- [ ] **Step 3: Update chip-socket binding paths**

In the H7 SPI router setup (currently `/tmp/klipper_sim_h7_chip_spi0`),
change to use the per-MCU sock_dir:

```python
# H7 sock_dir = log_dir.parent / "sim" / "h7"
h7_sock = pathlib.Path(str(log_dir)).parent / "sim" / "h7"
# SPI server path matches what the shim computes (spi_cs_<chip>_<line>).
# Today's CS pin offsets per pin-overrides.toml [mcu_main.gpio]:
#   PC7=5, PC6=4, PD11=6, PC4=3, PF8=40 — chip 0.
h7_spi_router = SpiRouter()
h7_spi_router.attach(5,  TMC5160Emulator().transfer)
h7_spi_router.attach(4,  TMC5160Emulator().transfer)
h7_spi_router.attach(6,  TMC5160Emulator().transfer)
h7_spi_router.attach(3,  TMC5160Emulator().transfer)
h7_spi_router.attach(40, MAX31865Emulator().transfer)
# Shim demultiplexes by CS pin → opens spi_cs_0_<line> per chip.
# Bind one socket server per CS pin instead of one shared bus.
for cs_line, transfer in h7_spi_router.attached_pairs():
    path = str(h7_sock / f"spi_cs_0_{cs_line}")
    srv = ChipSocketServer(path, transfer, framed=False)
    srv.start()
    chip_servers.append(srv)
```

(This requires `SpiRouter.attached_pairs()` method — add it: returns
`[(cs_offset, transfer_fn), ...]`.)

- [ ] **Step 4: Update tmcuart binding paths**

For H7 oid=0 (extruder) and F4 oids 0/1/2 (Z stepper variants):

```python
# H7 tmcuart oid=0 → ${h7_sock}/tmcuart_0
chip = TMC2209Emulator(slave_addr=0)
srv = ChipSocketServer(str(h7_sock / "tmcuart_0"), chip.handle, chunk=10)
srv.start()
chip_servers.append(srv)

# F4 tmcuart oids 0..2
f4_sock = pathlib.Path(str(log_dir)).parent / "sim" / "f4"
for i in range(3):
    chip = TMC2209Emulator(slave_addr=0)
    path = str(f4_sock / f"tmcuart_{i}")
    srv = ChipSocketServer(path, chip.handle, chunk=10)
    srv.start()
    chip_servers.append(srv)
```

Note: `tmcuart_<oid>` here corresponds to the firmware exception's
`KALICO_SIM_SOCK_DIR/tmcuart_<oid>` — see Task 19 for the firmware side.

- [ ] **Step 5: Plumb sim_control paths into SimContext**

In the fixture, after `mcus = spawn_mcus(...)`:

```python
ctx = SimContext(
    mcus=mcus,
    chip_servers=chip_servers,
    beacon=beacon,
    klippy_proc=klippy,
    klippy_log=klippy_log,
    api_socket=api_socket,
    log_dir=log_dir,
    h7_sim_control=str(h7_sock / "sim_control"),
    f4_sim_control=str(f4_sock / "sim_control"),
)
```

- [ ] **Step 6: Add SpiRouter.attached_pairs**

Find `SpiRouter` in `tools/sim_klippy/orchestrator/spi_router.py`:

```python
def attached_pairs(self):
    """Return list of (cs_offset, transfer_fn) pairs for every attached chip."""
    return list(self._chips.items())
```

(Field name may be `_routes` or similar — adjust based on actual code.)

- [ ] **Step 7: Run the boot test**

```bash
docker run --rm -v $PWD:/work -w /work --tmpfs /tmp:exec kalico-sim:latest bash -c "
  make -C tools/sim_klippy/preload
  python3 -m pytest tools/sim_klippy/tests/test_boot.py -x -q -s 2>&1 | tail -10
"
```

Expected: test still passes (firmware still has KALICO_SIM=y so the new
chip socket paths aren't exercised yet, but conftest setup must not break).

- [ ] **Step 8: Commit**

```bash
git add tools/sim_klippy/conftest.py tools/sim_klippy/orchestrator/spi_router.py
git commit -m "shim(conftest): bind chip sockets at shim-expected paths; expose sim_control"
```

---

## Phase 5: Firmware-side cleanup

### Task 15: Add CONFIG_KALICO_SIM_TMCUART_BYPASS Kconfig

**Files:**
- Modify: `src/Kconfig`

- [ ] **Step 1: Add the new option**

Edit `src/Kconfig`. After the existing `KALICO_SIM` block (around line 453):

```kconfig
config KALICO_SIM_TMCUART_BYPASS
    bool "Bypass tmcuart bit-bang and write bytes to a Unix socket"
    depends on KALICO_SIM && MACH_LINUX
    default n
    help
      Sim-only escape hatch for the tmcuart driver. When enabled, the
      tmcuart_send command bypasses the GPIO bit-bang and writes raw
      bytes directly to ${KALICO_SIM_SOCK_DIR}/tmcuart_<oid>. This is
      the one architectural exception to the otherwise-clean
      "production firmware knows nothing about sim" rule for MACH_LINUX
      builds — the bit-bang's timing fragility under Linux scheduler
      jitter would make a shim-side decoder flake intermittently. See
      docs/superpowers/specs/2026-05-08-syscall-shim-design.md
      §"Chip emulator routing → tmcuart" for rationale.
```

- [ ] **Step 2: Verify Kconfig parses**

```bash
docker run --rm -v $PWD:/work -w /work --tmpfs /tmp:exec kalico-sim:latest bash -c "
  cp tools/sim_klippy/configs/h7-sim.config .config
  make olddefconfig 2>&1 | tail -3
"
```

Expected: clean output, no Kconfig parse errors.

- [ ] **Step 3: Commit**

```bash
git add src/Kconfig
git commit -m "feat(kconfig): KALICO_SIM_TMCUART_BYPASS for MACH_LINUX sim"
```

---

### Task 16: Rework src/tmcuart.c — env-var path, new flag

**Files:**
- Modify: `src/tmcuart.c`

- [ ] **Step 1: Replace the sim_uart_routes table + lookup**

Find the existing `#if CONFIG_MACH_LINUX` block (around lines 16-75). Replace it entirely with:

```c
#if CONFIG_KALICO_SIM_TMCUART_BYPASS
#include <stdio.h>     // snprintf
#include <stdlib.h>    // getenv
#include "linux/sim_chip_socket.h"

// Per-oid socket fd cache. Path computed from KALICO_SIM_SOCK_DIR env var
// at first use; orchestrator binds matching paths before spawn.
#define MAX_TMCUART_OIDS 8
static int sim_tmcuart_fds[MAX_TMCUART_OIDS];
static int sim_tmcuart_fds_initialized = 0;

static int sim_tmcuart_lookup_fd(uint8_t oid) {
    if (oid >= MAX_TMCUART_OIDS) return -1;
    if (!sim_tmcuart_fds_initialized) {
        for (int i = 0; i < MAX_TMCUART_OIDS; i++) sim_tmcuart_fds[i] = -1;
        sim_tmcuart_fds_initialized = 1;
    }
    if (sim_tmcuart_fds[oid] >= 0) return sim_tmcuart_fds[oid];
    const char *sock_dir = getenv("KALICO_SIM_SOCK_DIR");
    if (!sock_dir) return -1;
    char path[256];
    snprintf(path, sizeof(path), "%s/tmcuart_%u", sock_dir, (unsigned)oid);
    sim_tmcuart_fds[oid] = sim_chip_socket_connect(path);
    return sim_tmcuart_fds[oid];
}
#endif
```

(Note: `sim_chip_socket_connect` stays alive until Task 21 deletes it.
For now, we depend on it.)

- [ ] **Step 2: Update command_tmcuart_send to use the new lookup**

Replace the `#if CONFIG_MACH_LINUX` block in `command_tmcuart_send`:

```c
#if CONFIG_KALICO_SIM_TMCUART_BYPASS
    {
        uint8_t oid = args[0];
        int sfd = sim_tmcuart_lookup_fd(oid);
        if (sfd >= 0) {
            uint8_t write_len = args[1];
            uint8_t *write_data = command_decode_ptr(args[2]);
            uint8_t read_len = args[3];
            uint8_t reply[16] = {0};
            if (read_len > sizeof(reply))
                shutdown("sim tmcuart read_len too big");
            if (sim_chip_socket_xfer(sfd, write_data, write_len,
                                     reply, read_len) != 0)
                shutdown("sim tmcuart xfer failed");
            sendf("tmcuart_response oid=%c read=%*s", oid, read_len, reply);
            return;
        }
    }
#endif
```

- [ ] **Step 3: Build with the new flag enabled**

Edit `tools/sim_klippy/configs/h7-sim.config`: ensure
`CONFIG_KALICO_SIM=y` is present (still on, for now), then add
`CONFIG_KALICO_SIM_TMCUART_BYPASS=y` at the end.

Same for `f4-sim.config`.

```bash
docker run --rm -v $PWD:/work -w /work --tmpfs /tmp:exec kalico-sim:latest bash -c "
  cp tools/sim_klippy/configs/h7-sim.config .config
  make olddefconfig >/dev/null
  make -j4 2>&1 | tail -3
"
```

Expected: clean build.

- [ ] **Step 4: Commit**

```bash
git add src/tmcuart.c tools/sim_klippy/configs/h7-sim.config tools/sim_klippy/configs/f4-sim.config
git commit -m "feat(tmcuart): env-var-driven sim socket path; gated on KALICO_SIM_TMCUART_BYPASS"
```

---

### Task 17: Repoint klippy KALICO_SIM_ENDSTOP_SET_PIN to sim_control

**Files:**
- Modify: `klippy/motion_toolhead.py`

- [ ] **Step 1: Add control-socket import + helper**

Near the top of `klippy/motion_toolhead.py`:

```python
import os

def _open_sim_control():
    """Open the shim's control socket. Returns SimControlClient or None
    if shim is not in use (real hardware or vanilla MACH_LINUX)."""
    sock_dir = os.environ.get("KALICO_SIM_SOCK_DIR")
    if not sock_dir:
        return None
    sock_path = os.path.join(sock_dir, "sim_control")
    if not os.path.exists(sock_path):
        return None
    # Lazy-import to avoid hard dependency in non-sim runs.
    import sys
    if "tools.sim_klippy.orchestrator.sim_control_client" not in sys.modules:
        # Klippy doesn't put tools/ on PYTHONPATH; the orchestrator's
        # conftest does. If we can't import, the shim is presumed absent.
        try:
            from tools.sim_klippy.orchestrator.sim_control_client import (
                SimControlClient,
            )
        except ImportError:
            return None
    from tools.sim_klippy.orchestrator.sim_control_client import (
        SimControlClient,
    )
    return SimControlClient(sock_path)
```

- [ ] **Step 2: Replace cmd_KALICO_SIM_ENDSTOP_SET_PIN body**

Find `cmd_KALICO_SIM_ENDSTOP_SET_PIN` (around line 741) and replace its body:

```python
    def cmd_KALICO_SIM_ENDSTOP_SET_PIN(self, gcmd):
        gpio = gcmd.get_int("GPIO", minval=0, maxval=0xFFFF)
        level = gcmd.get_int("LEVEL", minval=0, maxval=1)
        # GPIO pin index is chip_id * MAX_GPIO_LINES + offset, matching
        # the firmware's GPIO() macro. MAX_GPIO_LINES=288 per
        # src/linux/internal.h.
        MAX_GPIO_LINES = 288
        chip_id = gpio // MAX_GPIO_LINES
        line = gpio % MAX_GPIO_LINES
        client = _open_sim_control()
        if client is None:
            raise gcmd.error(
                "KALICO_SIM_ENDSTOP_SET_PIN requires the shim "
                "(KALICO_SIM_SOCK_DIR not set or sim_control missing)"
            )
        try:
            with client:
                client.set_gpio_input(chip=chip_id, line=line, value=level)
            gcmd.respond_info(
                "KALICO_SIM_ENDSTOP_SET_PIN gpio=%d level=%d -> ok"
                % (gpio, level)
            )
        except Exception as e:
            raise gcmd.error("set_gpio_input failed: %s" % e)
```

- [ ] **Step 3: Build + test_boot smoke**

```bash
docker run --rm -v $PWD:/work -w /work --tmpfs /tmp:exec kalico-sim:latest bash -c "
  make -C tools/sim_klippy/preload >/dev/null
  python3 -m pytest tools/sim_klippy/tests/test_boot.py -x -q -s 2>&1 | tail -5
"
```

Expected: PASS. (KALICO_SIM_ENDSTOP_SET_PIN isn't exercised in
test_boot, but klippy startup must not crash on the new import path.)

- [ ] **Step 4: Commit**

```bash
git add klippy/motion_toolhead.py
git commit -m "feat(klippy): KALICO_SIM_ENDSTOP_SET_PIN routes via shim sim_control"
```

---

## Phase 6: Cutover — delete sim-aware firmware code

This phase is the **single atomic moment** the spec calls "big-bang." The
commits here can land in any order within the cutover, but the test must
go from green → green across them. We delete sim-isms while flipping
configs to `KALICO_SIM=n` (shim takes over).

### Task 18: Delete sim blocks in src/linux/gpio.c

**Files:**
- Modify: `src/linux/gpio.c`

- [ ] **Step 1: Read the file to confirm the blocks to delete**

The deletions: every `#if CONFIG_KALICO_SIM` block (7 of them per
earlier scan), plus `sim_gpio_in_set_state` and `sim_gpio_out_offset`.

- [ ] **Step 2: Apply deletions**

In `src/linux/gpio.c`:
- Lines 75-85 (the `#if CONFIG_KALICO_SIM` block in `gpio_out_setup`):
  delete the entire `#if/#else/#endif`, keep only the `#else` body.
- Lines 99-120 (block in `gpio_out_reset`): same pattern.
- Lines 157-164 (block in `gpio_in_setup`): same pattern.
- Lines 170-196 (block in `gpio_in_reset`): same pattern.
- Lines 202-209 (block in `gpio_in_read`): same pattern.
- Lines 212-223 (the entire `sim_gpio_out_offset` function and its
  guard): delete.
- Lines 225-241 (the entire `sim_gpio_in_set_state` function and its
  guard): delete.

After: `src/linux/gpio.c` should have zero `KALICO_SIM` references.

- [ ] **Step 3: Verify no `KALICO_SIM` remains**

```bash
grep -c "KALICO_SIM" src/linux/gpio.c
```

Expected: `0`.

- [ ] **Step 4: Commit (don't build yet — other deletions may reference these symbols)**

```bash
git add src/linux/gpio.c
git commit -m "feat(gpio.c): delete CONFIG_KALICO_SIM blocks; shim handles /dev/gpiochip"
```

---

### Task 19: Delete sim blocks in src/linux/{hard_pwm,analog}.c

**Files:**
- Modify: `src/linux/hard_pwm.c`
- Modify: `src/linux/analog.c`

- [ ] **Step 1: hard_pwm.c — delete the early-return sim block**

Lines 51-57 (the `#if CONFIG_KALICO_SIM` block in `gpio_pwm_setup`): delete.
Lines 112-115 (the `#if CONFIG_KALICO_SIM` block in `gpio_pwm_write`): delete.

- [ ] **Step 2: analog.c — delete sim ADC table + helpers**

Delete:
- Lines 33-46 (the `#if CONFIG_KALICO_SIM` block in `gpio_adc_setup`'s error path)
- Lines 56-77 (the `MAX_SIM_ADC` table + `analog_set_simulated_value`)
- Lines 79-97 (the `analog_get_simulated_value` function)
- Lines 110-122 (the simulated-value lookup in `gpio_adc_read`)

After this `gpio_adc_read` collapses to:

```c
uint16_t
gpio_adc_read(struct gpio_adc g)
{
    char buf[64];
    int ret = pread(g.fd, buf, sizeof(buf)-1, 0);
    if (ret <= 0) {
        report_errno("analog read", ret);
        try_shutdown("Error on analog read");
        return 0;
    }
    buf[ret] = '\0';
    return atoi(buf);
}
```

- [ ] **Step 3: Verify**

```bash
grep -c "KALICO_SIM" src/linux/hard_pwm.c src/linux/analog.c
```

Expected: `src/linux/hard_pwm.c:0`, `src/linux/analog.c:0`.

- [ ] **Step 4: Commit**

```bash
git add src/linux/hard_pwm.c src/linux/analog.c
git commit -m "feat(linux): delete KALICO_SIM blocks from hard_pwm.c + analog.c"
```

---

### Task 20: Delete sim blocks in spidev.c + spicmds.c

**Files:**
- Modify: `src/linux/spidev.c`
- Modify: `src/spicmds.c`

- [ ] **Step 1: spidev.c — delete sim-route logic in spi_setup**

Lines 133-157 (the entire sim-route auto-register block + sim_route
branch): delete. After deletion, `spi_setup` starts directly with the
real ioctl path.

Lines 186-200 (the sim short-circuit branch in `spi_transfer`): delete.

Drop the `#include` for `sim_chip_socket.h` if present.

- [ ] **Step 2: spicmds.c — delete sim_pending_cs plumbing**

Lines around 123-137 (the `sim_spi_set_pending_cs` /
`sim_spi_clear_pending_cs` calls): delete those calls. The functions
themselves are defined in `sim_chip_socket.c` which gets deleted in
Task 21.

- [ ] **Step 3: Commit**

```bash
git add src/linux/spidev.c src/spicmds.c
git commit -m "feat(spi): delete sim-route logic; shim intercepts /dev/spidev"
```

---

### Task 21: Delete sim_chip_socket.{c,h} + runtime_sim_commands.c + Makefile updates

**Files:**
- Delete: `src/linux/sim_chip_socket.c`
- Delete: `src/linux/sim_chip_socket.h`
- Delete: `src/runtime_sim_commands.c`
- Modify: `src/linux/Makefile`
- Modify: `src/Makefile`

- [ ] **Step 1: Check who still includes sim_chip_socket.h**

```bash
grep -rn "sim_chip_socket" src/
```

After Tasks 18-20, only `src/tmcuart.c` should still reference it
(it uses `sim_chip_socket_connect` / `sim_chip_socket_xfer` for the
tmcuart bypass). Verify.

If `tmcuart.c` is the only consumer, the `sim_chip_socket.{c,h}` files
must stay — they're now consumed only by the tmcuart exception. **Do not
delete them**; instead make their build conditional on
`CONFIG_KALICO_SIM_TMCUART_BYPASS`.

If the grep output is unexpected, stop and reread.

- [ ] **Step 2: Edit src/linux/Makefile**

Find the line that adds `sim_chip_socket.c` to `linux-src-y`. Change it
from `KALICO_SIM`-gated to `KALICO_SIM_TMCUART_BYPASS`-gated:

```makefile
linux-src-$(CONFIG_KALICO_SIM_TMCUART_BYPASS) += linux/sim_chip_socket.c
```

(Keep an existing similar block; only change the gate.)

- [ ] **Step 3: Delete runtime_sim_commands.c**

```bash
rm src/runtime_sim_commands.c
```

- [ ] **Step 4: Edit src/Makefile**

Drop the `runtime_sim_commands.c` reference. Find the line:

```makefile
src-$(CONFIG_KALICO_SIM) += runtime_sim_commands.c
```

Delete that line entirely.

- [ ] **Step 5: Build**

```bash
docker run --rm -v $PWD:/work -w /work --tmpfs /tmp:exec kalico-sim:latest bash -c "
  cp tools/sim_klippy/configs/h7-sim.config .config
  sed -i 's/^CONFIG_KALICO_SIM=y$/# CONFIG_KALICO_SIM is not set/' .config
  echo 'CONFIG_KALICO_SIM_TMCUART_BYPASS=y' >> .config
  make olddefconfig >/dev/null
  make clean >/dev/null
  make -j4 2>&1 | tail -3
"
```

Expected: clean build with `KALICO_SIM=n` and tmcuart bypass on.

- [ ] **Step 6: Commit**

```bash
git add src/Makefile src/linux/Makefile
git rm src/runtime_sim_commands.c
git commit -m "feat(build): drop runtime_sim_commands; gate sim_chip_socket on TMCUART_BYPASS"
```

---

### Task 22: Flip sim configs to KALICO_SIM=n, KALICO_SIM_TMCUART_BYPASS=y

**Files:**
- Modify: `tools/sim_klippy/configs/h7-sim.config`
- Modify: `tools/sim_klippy/configs/f4-sim.config`

- [ ] **Step 1: Edit h7-sim.config**

Replace:
```
CONFIG_KALICO_SIM=y
```
with:
```
# CONFIG_KALICO_SIM is not set
CONFIG_KALICO_SIM_TMCUART_BYPASS=y
```

(The bypass flag still needs a CONFIG_KALICO_SIM=y dependency in
Kconfig — adjust the dependency in src/Kconfig from
`depends on KALICO_SIM && MACH_LINUX` to `depends on MACH_LINUX`,
OR set CONFIG_KALICO_SIM=y but configure the shim to be the only sim
mechanism. Decision: drop the KALICO_SIM dependency. The bypass is a
MACH_LINUX-only consideration, not coupled to the Renode-flavored sim
flag.)

Edit `src/Kconfig`:
```kconfig
config KALICO_SIM_TMCUART_BYPASS
    bool "Bypass tmcuart bit-bang and write bytes to a Unix socket"
    depends on MACH_LINUX
    default n
    help
      ...
```

- [ ] **Step 2: Edit f4-sim.config**

Same change.

- [ ] **Step 3: Verify both build**

```bash
docker run --rm -v $PWD:/work -w /work --tmpfs /tmp:exec kalico-sim:latest bash -c "
  cp tools/sim_klippy/configs/h7-sim.config .config
  make olddefconfig >/dev/null && make clean >/dev/null && make -j4 2>&1 | tail -2
  cp tools/sim_klippy/configs/f4-sim.config .config
  make olddefconfig >/dev/null && make clean >/dev/null && make -j4 2>&1 | tail -2
"
```

Expected: both build clean.

- [ ] **Step 4: Commit**

```bash
git add tools/sim_klippy/configs/h7-sim.config tools/sim_klippy/configs/f4-sim.config src/Kconfig
git commit -m "feat(sim): flip configs to KALICO_SIM=n; rely on shim + TMCUART_BYPASS"
```

---

## Phase 7: End-to-end verification

### Task 23: Run faithful sim baseline tests

**Files:** none — verification only.

- [ ] **Step 1: Run test_boot end-to-end**

```bash
docker run --rm -v $PWD:/work -w /work --tmpfs /tmp:exec kalico-sim:latest bash -c "
  set -e
  make -C tools/sim_klippy/preload >/dev/null
  cp tools/sim_klippy/configs/h7-sim.config .config && make olddefconfig >/dev/null && make clean >/dev/null && make -j4 2>&1 | tail -1
  mkdir -p .build-stash && cp out/klipper.elf .build-stash/klipper-h7-sim.elf
  cp tools/sim_klippy/configs/f4-sim.config .config && make olddefconfig >/dev/null && make clean >/dev/null && make -j4 2>&1 | tail -1
  cp out/klipper.elf out/klipper-f4-sim.elf
  cp .build-stash/klipper-h7-sim.elf out/klipper-h7-sim.elf
  make -f Makefile.kalico motion-bridge 2>&1 | tail -1
  python3 -m pytest tools/sim_klippy/tests/test_boot.py -x -q -s 2>&1 | tail -10
"
```

Expected: `1 passed`. If failure, read `.local-logs/test_boot_clean/`
artifacts to diagnose.

- [ ] **Step 2: Run test_g28_x_smoke**

```bash
docker run --rm -v $PWD:/work -w /work --tmpfs /tmp:exec kalico-sim:latest bash -c "
  python3 -m pytest tools/sim_klippy/tests/test_g28_x_smoke.py -x -q -s 2>&1 | tail -10
"
```

Expected: the test now exposes whatever the **next** real bug is, not
the broken-sim-route bug from before. May still fail — that's expected;
the discovery loop continues from here.

- [ ] **Step 3: Run direct shim tests**

```bash
docker run --rm -v $PWD:/work -w /work --tmpfs /tmp:exec kalico-sim:latest bash -c "
  make -C tools/sim_klippy/preload/tests >/dev/null
  KALICO_SIM_SOCK_DIR=/tmp/sim_test \
    LD_PRELOAD=/work/tools/sim_klippy/preload/libsim_intercept.so \
    /work/tools/sim_klippy/preload/tests/test_shim
"
```

Expected: 5x PASS, 0 fails.

- [ ] **Step 4: Run sim_control_client pytest**

```bash
docker run --rm -v $PWD:/work -w /work --tmpfs /tmp:exec kalico-sim:latest bash -c "
  python3 -m pytest tools/sim_klippy/tests/test_sim_control.py -x -q -s 2>&1 | tail -5
"
```

Expected: 4 passed.

- [ ] **Step 5: Run Rust crate tests (no Rust changes — sanity check)**

```bash
cd rust && cargo test -p kalico-host-rt -p motion-bridge 2>&1 | tail -10
```

Expected: tests pass (or same baseline failures as before this PR — no
new ones).

- [ ] **Step 6: Append plan-changes-log entry**

Append to `docs/superpowers/plan-changes-log.md`:

```markdown
## 2026-05-08 — Syscall-intercept LD_PRELOAD shim for MACH_LINUX sim

**What changed:** Replaced ~660 lines of `#ifdef CONFIG_KALICO_SIM`
plumbing across 8 firmware files with a `tools/sim_klippy/preload/`
LD_PRELOAD shim (~700 LOC). Production firmware (Category A) is now
bit-identical between Pi-Klipper and the test sim. One contained
exception remains: `tmcuart` keeps a firmware short-circuit gated on
`CONFIG_KALICO_SIM_TMCUART_BYPASS` and reads its socket path from the
`KALICO_SIM_SOCK_DIR` env var (no flavor heuristic, no klippy involvement).

**Why:** Previous v1 spec (`2026-05-08-explicit-sim-chip-routes-design.md`)
fixed the immediate broken-flavor-heuristic bug but left the layer
violation (sim plumbing inside firmware) intact. Each future "I need to
fake X" change would have grown more firmware #ifdefs. The shim retires
the layer violation without touching production behavior on real
hardware.

**Evidence:**
- Spec: `docs/superpowers/specs/2026-05-08-syscall-shim-design.md`
- Plan: `docs/superpowers/plans/2026-05-08-syscall-shim.md`
- Smoke test (`test_g28_x_smoke`) progresses past the prior failure mode.

**Lesson:** When the second similar bug pattern shows up in the same
file (broken auto-route in tmcuart.c → broken auto-route in spidev.c),
that's the signal to fix the architecture, not the symptom.
```

- [ ] **Step 7: Commit + final PR-readiness**

```bash
git add docs/superpowers/plan-changes-log.md
git commit -m "doc(plan-changes-log): syscall-intercept LD_PRELOAD shim"
git status
```

Expected: clean working tree, all changes on the current branch.

---

## Notes for the implementer

### Order matters within Phase 6 (Tasks 18-22)
Tasks 18-20 delete sim-aware code paths in firmware files. Tasks 21-22
delete supporting files and flip configs to make `KALICO_SIM=n` actually
work. Build will be broken between Task 18 and Task 22 if you build with
`KALICO_SIM=y` (firmware short-circuits removed but config still expects
them). That's fine — the only build attempt within Phase 6 is at Task 21
Step 5, which uses `KALICO_SIM=n`. Don't run intermediate builds with
the old config.

### What "PASS" means at each stage
- After Task 13: shim builds, faithful sim test_boot still passes (firmware
  still has `KALICO_SIM=y`, shim is loaded but mostly idle).
- After Task 17: same, plus klippy startup tolerates the new sim_control
  import path.
- After Task 22 (cutover complete): firmware has `KALICO_SIM=n`, shim
  serves all `/dev/*` access, sim test_boot passes.

### If something breaks
1. Read the `.local-logs/test_boot_clean/` artifacts: `klippy.log`,
   `h7.log`, `f4.log`, `klippy.stdout`.
2. Set `KALICO_SIM_SHIM_VERBOSE=1` in conftest's launcher env to enable
   shim trace logging — every interception logs a one-liner.
3. Use the C test harness (`tools/sim_klippy/preload/tests/test_shim`)
   to isolate shim-side from orchestrator-side issues.
