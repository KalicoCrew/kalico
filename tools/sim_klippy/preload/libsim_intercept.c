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

__attribute__((constructor))
static void shim_init(void) {
    const char *v = getenv("KALICO_SIM_SHIM_VERBOSE");
    verbose = (v && v[0] == '1');
    LOG("init pid=%d", (int)getpid());
}
