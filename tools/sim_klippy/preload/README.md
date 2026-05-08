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
