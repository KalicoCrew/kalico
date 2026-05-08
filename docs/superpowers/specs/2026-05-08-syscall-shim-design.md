# Syscall-Intercept Sim Shim — replacing firmware sim-isms with `LD_PRELOAD`

## Status

Spec rev 2. Replaces the never-implemented `2026-05-08-explicit-sim-chip-routes-design.md`
(rev 1), which fixed the wrong layer.

## Problem

The faithful sim runs `klipper.elf` built with `MACH_LINUX` + `KALICO_SIM=y`.
That `KALICO_SIM` macro pulls `#ifdef`-gated code into eight production firmware
files plus two whole files of sim-only plumbing — all to replace
`/dev/gpiochip*`, `/dev/spidev*`, `/sys/class/pwm/*`, `/sys/bus/iio/*` device
access with Unix-domain sockets to chip emulators. New emulator types accrete
new `#ifdef` paths and new wire-protocol commands (e.g. `runtime_sim_route_*`)
in firmware C source.

Today's footprint:

| File | Sim-only lines | Disposition under v1 |
|---|---:|---|
| `src/linux/gpio.c` | 103 | new `#ifdef`s + new `runtime_sim_gpio_in_set` C cmd |
| `src/linux/hard_pwm.c` | 11 | kept |
| `src/linux/analog.c` | 21 + 30 (sim ADC table) | kept |
| `src/linux/spidev.c` | ~45 (`flavor` heuristic, sim-route branch) | broken; v1 patches |
| `src/linux/sim_chip_socket.{c,h}` | 137 (whole files) | kept |
| `src/spicmds.c` | 12 (`sim_pending_cs` plumbing) | kept |
| `src/tmcuart.c` | ~73 (sim_uart route, helpers) | broken; v1 patches |
| `src/runtime_sim_commands.c` | 263 (whole file) | grew |

Roughly 690 lines of sim plumbing inside production firmware, that production
firmware should not know about. Each new test capability ("fake the DIAG line",
"set ADC value", "route this oid to that socket") was an additive change
to firmware C source.

## Goal

Production firmware (Category A files: `linux/*`, `spicmds.c`, `tmcuart.c`,
`runtime_sim_commands.c`) becomes bit-identical between
"MACH_LINUX-on-Pi-Klipper" and "MACH_LINUX-as-test-sim". All sim-specific
behavior moves into a `LD_PRELOAD` shim under `tools/sim_klippy/preload/`.

`KALICO_SIM` does not disappear — it still gates Renode's STM32-side concerns
(`watchdog.c`'s IWDG-disable, `runtime_tick.c`'s sim-progress diag). But its
MACH_LINUX consumers all delete.

## Non-goals

- No Renode changes. Renode runs `MACH_STM32H7` / `MACH_STM32F4` ELFs on
  emulated bare-metal silicon. There is no `LD_PRELOAD` mechanism in that
  world; the shim never sees those builds.
- No real-hardware behavior changes. The shim only loads when
  `LD_PRELOAD` env var is set. A Pi running klipper without the env var sees
  exactly the same code path it sees today.
- No Klippy extras. Routing decisions are not in klippy.
- No new wire surface in the bridge. `runtime_sim_*` klipper-protocol commands
  are deleted, not relocated.

## Empirical verification done before spec writing

- `ldd out/klipper.elf` confirms dynamic linking (libc.so.6, libgcc_s.so.1).
- A 50-line POC `libpoc.so` built and loaded via `LD_PRELOAD` against
  `klipper.elf` inside the existing Docker harness:
  - `__attribute__((constructor))` runs.
  - `dlsym(RTLD_NEXT, "open")` resolves.
  - Real `open("/dev/urandom")` from libc startup is intercepted and logged.
- The empirical surface check confirmed that with `KALICO_SIM=y`, klipper
  short-circuits before the `open()` calls (existing firmware sim stubs).
  Without `KALICO_SIM`, klipper attempts the real `/dev/gpiochip0` opens
  (which fail in Docker without the shim — exactly the behavior we want
  the shim to fill in).

## Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│ klipper.elf  (MACH_LINUX, KALICO_SIM=n for Category A)               │
│                                                                      │
│ Real syscalls — from src/linux/gpio.c, spidev.c, hard_pwm.c, analog.c│
│   open("/dev/gpiochip0", O_RDWR | O_CLOEXEC)                         │
│   ioctl(chip_fd, GPIO_GET_LINEHANDLE_IOCTL, &req)  → req.fd          │
│   ioctl(line_fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data)            │
│   ioctl(line_fd, GPIOHANDLE_GET_LINE_VALUES_IOCTL, &data)            │
│                                                                      │
│   open("/dev/spidev<bus>.<dev>", O_RDWR | O_CLOEXEC)                 │
│   ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &rate)                      │
│   ioctl(spi_fd, SPI_IOC_WR_MODE, &mode)                              │
│   ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_ioc_transfer)               │
│   write(spi_fd, data, len)                                           │
│                                                                      │
│   open("/sys/class/pwm/pwmchip<N>/pwm<M>/{period,duty_cycle,enable}",│
│        O_WRONLY | O_CLOEXEC)                                         │
│   write(pwm_fd, "<n>", strlen)                                       │
│                                                                      │
│   open("/sys/bus/iio/devices/iio:device0/in_voltage<N>_raw", O_RDONLY)│
│   pread(iio_fd, buf, 64, 0)                                          │
│                                                                      │
│   access("/dev/gpiochip<N>", F_OK)        ← presence check           │
│   fcntl(fd, F_SETFD, FD_CLOEXEC)          ← set close-on-exec        │
│   fcntl(fd, F_GETFL); F_SETFL with O_NONBLOCK ← non-blocking         │
│   close(fd)                                                          │
└─────────────────────┬────────────────────────────────────────────────┘
                      │  libc syscalls
                      ▼
┌──────────────────────────────────────────────────────────────────────┐
│ tools/sim_klippy/preload/libsim_intercept.so  (LD_PRELOAD)           │
│                                                                      │
│ Intercepts (9 functions):                                            │
│   open, openat, ioctl, read, pread, write, close, fcntl, access      │
│   (klipper does not currently use dup/dup2/dup3 on /dev/* fds —      │
│    confirmed by grep across Category A files; see R4 for the         │
│    forward-compat plan if that ever changes)                         │
│                                                                      │
│ Path-based dispatch in open()/openat()/access():                     │
│   /dev/gpiochip<N>      → fake chip fd in [FAKE_FD_BASE, ...)        │
│   /dev/spidev<bus>.<d>  → fake spidev fd                             │
│   /sys/class/pwm/...    → fake pwm-file fd                           │
│   /sys/bus/iio/...      → fake iio-file fd                           │
│   anything else         → real_open (passthrough)                    │
│                                                                      │
│ Per-fd state table:                                                  │
│   fd in [1<<28, (1<<28) + N) is fake; lookup struct sim_fd_slot      │
│   fd outside that range is real; passthrough                         │
│                                                                      │
│ Fake fd lifecycle:                                                   │
│   open()       → allocates a slot, returns fake fd                   │
│   ioctl/read/  → dispatch on slot.kind                               │
│     write                                                            │
│   close()      → frees the slot                                      │
│                                                                      │
│ Shim-owned state:                                                    │
│   - GPIO chip table:   per-chip × per-line: direction + value        │
│   - Currently-asserted output GPIO line (chip_id, offset)            │
│     ↑ used as the SPI CS demultiplex key                             │
│   - ADC channel values: per-channel u16                              │
│   - PWM file last-written values: per-file string                    │
│                                                                      │
│ Outbound chip emulator sockets:                                      │
│   ${KALICO_SIM_SOCK_DIR}/spi_cs<chip>_<line>   (one per CS pin)      │
│   ${KALICO_SIM_SOCK_DIR}/uart_<chip>_<line>    (one per tmcuart pair)│
│   Routing: SPI dispatch keys on currently-asserted CS pin            │
│            tmcuart routes via the GPIO output pin (rx_pin from       │
│            firmware's config_tmcuart). The shim watches the GPIO     │
│            bit-bang sequence on that pin and bidirectionally bridges │
│            it through the per-pair socket.                           │
│                                                                      │
│ Inbound test control socket:                                         │
│   ${KALICO_SIM_SOCK_DIR}/sim_control                                 │
│   Protocol: see "Control socket" section below.                      │
└─────────────────────┬────────────────────────────────────────────────┘
                      │  Unix sockets
                      ▼
┌──────────────────────────────────────────────────────────────────────┐
│ Orchestrator (tools/sim_klippy/conftest.py + emulator processes)     │
│ - Spawns klipper.elf with:                                           │
│     LD_PRELOAD=/work/tools/sim_klippy/preload/libsim_intercept.so    │
│     KALICO_SIM_SOCK_DIR=/tmp/<test_path>/sim/                        │
│ - Owns chip emulator processes (TMC2209, TMC5160, MAX31865), each    │
│   binding its own per-CS socket path under sim/                      │
│ - Connects to sim/sim_control to poke shim-owned state               │
└──────────────────────────────────────────────────────────────────────┘
```

## Syscall surface

This is the actual surface, derived from reading every Category A source file.

### GPIO chip (`src/linux/gpio.c`)

| Operation | Source | Shim handler |
|---|---|---|
| `access("/dev/gpiochip<N>", F_OK)` | `get_chip_fd` | return 0 for known chip ids, errno=ENOENT otherwise |
| `open("/dev/gpiochip<N>", O_RDWR\|O_CLOEXEC)` | `get_chip_fd` | allocate `SIM_GPIOCHIP` slot, return fake fd |
| `ioctl(chip_fd, GPIO_GET_LINEHANDLE_IOCTL, struct gpiohandle_request*)` | `gpio_out_reset`, `gpio_in_reset` | record (chip, line, dir, flags, default_value); allocate child `SIM_GPIOLINE` slot; populate `req.fd` with fake child fd; return 0 |
| `ioctl(line_fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, struct gpiohandle_data*)` | `gpio_out_write` | update shim's line-value state; if line is currently the active CS, update "asserted CS" tracker; return 0 |
| `ioctl(line_fd, GPIOHANDLE_GET_LINE_VALUES_IOCTL, struct gpiohandle_data*)` | `gpio_in_read` | return shim's line-value state; return 0 |
| `fcntl(line_fd, F_SETFD, FD_CLOEXEC)` | `set_close_on_exec` | no-op; return 0 |
| `close(line_fd)`, `close(chip_fd)` | `gpio_release_line` | free the slot |

Klipper uses **only V1 ioctls** (`GPIOHANDLE_*`), not V2 (`GPIO_V2_LINE_*`).
This narrows the shim's GPIO surface materially. If a future klipper change
adopts V2, the shim grows by an additional handler — but until then, V1 is
the contract.

### SPI (`src/linux/spidev.c`)

| Operation | Source | Shim handler |
|---|---|---|
| `open("/dev/spidev<bus>.<dev>", O_RDWR\|O_CLOEXEC)` | `spi_open` | allocate `SIM_SPIDEV` slot, return fake fd |
| `fcntl(fd, F_SETFL, F_GETFL\|O_NONBLOCK)` | `set_non_blocking` | no-op; return 0 |
| `ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &rate)` | `spi_setup` | store on slot; return 0 |
| `ioctl(fd, SPI_IOC_WR_MODE, &mode)` | `spi_setup` | store on slot; return 0 |
| `ioctl(fd, SPI_IOC_MESSAGE(1), struct spi_ioc_transfer*)` | `spi_transfer` | look up the chip socket by currently-asserted CS pin; send `transfer.tx_buf[0..transfer.len]` over socket; read `transfer.len` bytes back into `transfer.rx_buf`; return 0 |
| `write(fd, data, len)` | `spi_transfer` (no-receive path) | same dispatch, no rx; return `len` |
| `close(fd)` | n/a (klipper doesn't close spidev fds during normal operation) | free the slot |

**Klipper uses only `SPI_IOC_MESSAGE(1)` — single transfer per call**, never
multi-transfer batches. Shim doesn't need to handle the variable-N variant.

### PWM (`src/linux/hard_pwm.c`)

| Operation | Source | Shim handler |
|---|---|---|
| `open("/sys/class/pwm/pwmchip<N>/pwm<M>/{period,duty_cycle,enable}", O_WRONLY\|O_CLOEXEC)` | `gpio_pwm_setup` | allocate `SIM_PWM_FILE` slot, return fake fd |
| `write(fd, "<integer>", strlen)` | `gpio_pwm_write` | store value on slot; return `len` |
| `close(fd)` | `gpio_pwm_setup` (period only) | free the slot |

The shim does not need to read or simulate PWM behavior — klipper writes-only
to PWM files. We just absorb the writes and let klipper believe they
succeeded. Tests that need to verify PWM duty cycles can query via the
control socket (future extension).

The BeagleBoard PWM path (`/sys/class/pwm/pwm-<chip>:<id>/...`) is also
covered by the same path-prefix dispatch in `open()`.

### IIO ADC (`src/linux/analog.c`)

| Operation | Source | Shim handler |
|---|---|---|
| `open("/sys/bus/iio/devices/iio:device0/in_voltage<N>_raw", O_RDONLY\|O_CLOEXEC)` | `gpio_adc_setup` | allocate `SIM_IIO_FILE` slot, return fake fd |
| `fcntl(fd, F_SETFL, F_GETFL\|O_NONBLOCK)` | `set_non_blocking` | no-op; return 0 |
| `pread(fd, buf, 64, 0)` | `gpio_adc_read` | format current channel value as ASCII into `buf`, return bytes written |
| `close(fd)` | n/a (klipper holds these open) | free the slot |

Default ADC value: 3900 (matches the existing `SIM_ADC_DEFAULT` in
`analog.c` — chosen for ~25 °C on a typical NTC thermistor with 4.7 kΩ
pull-up; tests seed actual values via control socket).

### Total syscalls intercepted

```
open, openat, ioctl, read, pread, write, close, fcntl, access
```

Nine functions. ~600 LOC of shim covering all four device surfaces and the
control-socket server.

## Per-fd state and slot allocation

```c
#define FAKE_FD_BASE 0x10000000   // 256M, well above any real allocation
#define MAX_FAKE_FDS 256

struct sim_fd_slot {
    enum { SIM_NONE, SIM_GPIOCHIP, SIM_GPIOLINE,
           SIM_SPIDEV, SIM_PWM_FILE, SIM_IIO_FILE } kind;
    union {
        struct { int chip_id; } gpiochip;
        struct {
            int chip_id;
            int line_offset;
            uint32_t flags;       // GPIOHANDLE_REQUEST_*
            int last_value;
        } gpioline;
        struct {
            int bus, dev;
            uint32_t speed_hz;
            uint8_t mode;
            int chip_socket_fd;   // negative = not yet connected
            uint8_t cs_chip_id;   // updated on every ioctl by inspecting active CS
            uint8_t cs_line_offset;
        } spidev;
        struct {
            int chip_id, pwm_id;
            char file[32];        // "period", "duty_cycle", "enable"
            uint64_t last_value;  // numeric value of last write
        } pwm_file;
        struct {
            int channel;
        } iio_file;
    } u;
};

static struct sim_fd_slot fake_slots[MAX_FAKE_FDS];
static pthread_mutex_t fake_slots_mtx = PTHREAD_MUTEX_INITIALIZER;
```

Slot 0 is reserved (a fake fd of `FAKE_FD_BASE + 0` would alias an unset
slot). Allocation is linear scan + first-free; `MAX_FAKE_FDS = 256` is more
than enough (the faithful sim today has under 60 active fds across all
device categories).

## Shim-owned global state

```c
// GPIO chip table.
struct sim_gpio_line { int direction; int value; };
struct sim_gpio_chip { struct sim_gpio_line lines[64]; };
static struct sim_gpio_chip gpio_chips[9];

// "Currently asserted" output line — last GPIO output that was set to 1.
// Used as the SPI CS demultiplex key. Tracking matches the existing firmware
// `sim_pending_cs` mechanism in src/spicmds.c.
struct active_cs { int chip_id; int line_offset; int valid; };
static struct active_cs active_cs;

// ADC channel values. Indexed by the channel number from
// /sys/bus/iio/devices/iio:device0/in_voltage<N>_raw.
#define MAX_IIO_CHANNELS 32
#define DEFAULT_ADC_VALUE 3900
static uint16_t iio_values[MAX_IIO_CHANNELS];

// Mutex protecting all of the above. Klipper's main loop is the only
// reader/writer from inside klipper-side; the control-socket accept
// thread is the only reader/writer from outside.
static pthread_mutex_t shim_state_mtx = PTHREAD_MUTEX_INITIALIZER;
```

Threading: klipper firmware on MACH_LINUX is single-threaded for I/O
(`console_task` is the sole consumer of /dev/* fds; signal handlers don't
touch them). The shim adds one accept thread for the control socket. A
single mutex over all shim state is sufficient and not a performance
concern at klipper's traffic rate.

## Chip emulator routing

### SPI

Klipper's existing pattern: assert CS pin via `gpio_out_write`, then issue
`SPI_IOC_MESSAGE(1)`. The shim's `gpio_out_write` handler updates
`active_cs`. The shim's `SPI_IOC_MESSAGE` handler reads `active_cs` to pick
the chip emulator socket.

Socket path: `${KALICO_SIM_SOCK_DIR}/spi_cs_<chip_id>_<line_offset>`.

The orchestrator (conftest.py) binds these paths before spawning klipper.elf.
Today's `SpiRouter` in conftest demultiplexes one shared bus by CS-pin offset
in a framed protocol; the shim moves that demultiplex to syscall time, and
each chip emulator gets its own un-framed socket. Cleaner protocol; chip
emulators stop knowing about CS pins.

### tmcuart

Klipper bit-bangs UART on a single GPIO pin (rx_pin == tx_pin for
single-wire). The `tmcuart_send` command in src/tmcuart.c sets a timer to
toggle the pin; the firmware's bit-banged sequence is what reaches the wire.

Today's sim short-circuit: `command_tmcuart_send` detects `oid` is
sim-routed, bypasses the bit-bang, sends the entire UART frame over a Unix
socket as bytes.

**Decision: keep the tmcuart short-circuit in firmware, but on a clean
explicit-route basis.** Replace the broken `flavor` heuristic and the
`runtime_sim_route_tmcuart` wire command with a simpler convention:
firmware reads `KALICO_SIM_SOCK_DIR` env var at startup; each tmcuart oid
generates its socket path as `${KALICO_SIM_SOCK_DIR}/tmcuart_<oid>`.
The orchestrator binds matching paths before spawning klipper.elf. No
runtime route registration; no klippy involvement; no flavor heuristic.

This is the **one architectural exception** to "production firmware
knows nothing about sim." The exception is contained:
- Gated behind `#if CONFIG_KALICO_SIM_TMCUART_BYPASS` (a new fine-grained
  flag, off by default; `MACH_LINUX` sim build sets it on).
- ~50 LOC of firmware: getenv, snprintf the path, connect, write+read.
- One direction (firmware → socket → emulator); no bit-bang state machine
  in the shim.

**Why the exception is justified:** the alternative (shim observes GPIO
toggles, reconstructs UART frames by start/stop bits, drives input at the
right rate during read windows) is timing-sensitive. Linux scheduler
jitter under load could desync the shim's UART decoder from firmware's
bit-clock. That's the kind of intermittent flake that poisons a discovery
loop — we'd waste time debugging shim timing instead of finding real
bugs in motion code. The LD_PRELOAD architecture is for breadth (8
firmware files become bit-identical sim/real); tmcuart is one
narrow corner where the cost-benefit flips. We trade ideological purity
for a smaller, more reliable foundation.

If a future need ever justifies it (e.g., we want to run real-hardware
tmcuart timing tests in the sim), the shim grows a bit-bang observer in
v2 and the firmware exception deletes.

Socket path: `${KALICO_SIM_SOCK_DIR}/tmcuart_<oid>`.

### PWM, IIO

No external chip emulator needed. PWM is write-only (klipper sets duty,
shim absorbs it; tests can query via control socket). IIO is read-only
(shim returns currently-set value; tests poke via control socket).

## Control socket protocol

### Wire format

Newline-delimited UTF-8 text, request/response, synchronous. One client at
a time (the orchestrator); concurrent clients are rejected with
`error: socket already in use\n`.

### Request grammar

```
request   := verb (' ' key '=' value)* '\n'
verb      := 'set_gpio_input' | 'set_adc' | 'get_gpio_output' | 'get_pwm' | 'ping'
key       := 'chip' | 'line' | 'channel' | 'value'
value     := decimal integer
```

### Response grammar

```
response  := 'ok\n'
           | 'value=' decimal '\n'
           | 'error: ' message '\n'
message   := printable ASCII, no newlines
```

### Commands

| Request | Replaces firmware surface | Response |
|---|---|---|
| `set_gpio_input chip=<N> line=<M> value=<0\|1>` | `runtime_sim_endstop_set_pin`, `runtime_sim_gpio_in_set`, `sim_gpio_in_set_state` | `ok` |
| `set_adc channel=<N> value=<u16>` | `runtime_sim_adc_set` | `ok` |
| `get_gpio_output chip=<N> line=<M>` | (new — diagnostic) | `value=<0\|1>` |
| `get_pwm chip=<N> pwm=<M> file=<period\|duty_cycle\|enable>` | (new — diagnostic) | `value=<u64>` |
| `ping` | (new — health check) | `ok` |

### Concurrency / ordering

- All state mutations under `shim_state_mtx`.
- A `set_gpio_input` from the control socket and a concurrent
  `GPIOHANDLE_GET_LINE_VALUES_IOCTL` from klipper serialize on the mutex.
  Either ordering is observable (test → klipper sees old, or test →
  klipper sees new), and there is no atomicity guarantee across multiple
  set_gpio_input calls — tests that need atomicity should issue multiple
  set commands as a single newline-separated batch in one `send()`
  (the shim accept thread will hold the mutex across the whole batch
  while parsing).

### Errors

- Malformed request → `error: parse error\n`. The connection stays open.
- Unknown verb → `error: unknown verb <verb>\n`. Connection stays open.
- Out-of-range parameter (e.g., `chip=99`) → `error: chip 99 out of range\n`.
- Socket disconnect mid-request → discard the partial buffer, accept
  a new client.

### State diagram for shim-side accept thread

```
INIT ─── bind+listen ───▶ ACCEPTING
                            │
                       accept() ◀── connection closed ──▶ ACCEPTING
                            │
                            ▼
                         CONNECTED
                            │ recv \n-delimited line
                            ▼
                      parse → mutex.lock
                            │
                       update state
                            │
                       mutex.unlock → send response → CONNECTED
```

The accept thread is the only thread that calls `recv()` and `send()` on
the control socket. Klipper's main thread never touches the control
socket fd.

## Migration

### What klippy gives up

`klippy/motion_toolhead.py:cmd_KALICO_SIM_ENDSTOP_SET_PIN` currently sends
the firmware command via the bridge. Under the shim, klippy connects
directly to `${KALICO_SIM_SOCK_DIR}/sim_control` and sends
`set_gpio_input chip=<N> line=<M> value=<v>`. The bridge call goes away.

Same for `KALICO_SIM_AXIS_ACCUM` if it exists for ADC poking — but on
inspection that gcode reads stepper accumulators, which is a runtime-side
state, not a sim concern. Stays as-is.

### Firmware deletions (precise per-file)

| File | Lines deleted | Lines kept | Notes |
|---|---:|---:|---|
| `src/linux/gpio.c` | 103 | 138 | All `#if CONFIG_KALICO_SIM` blocks; `sim_gpio_in_set_state`; `sim_gpio_out_offset` |
| `src/linux/hard_pwm.c` | 11 | 114 | One `#if CONFIG_KALICO_SIM` block in setup |
| `src/linux/analog.c` | 51 | 87 | Sim ADC table, `analog_set_simulated_value`, sim fallback in `gpio_adc_setup` |
| `src/linux/spidev.c` | ~45 | ~178 | `flavor` heuristic, sim-route branch, `sim_spi_*` plumbing |
| `src/linux/sim_chip_socket.c` | 100 | 0 | **whole file** |
| `src/linux/sim_chip_socket.h` | 37 | 0 | **whole file** |
| `src/linux/Makefile` | ~5 | n/a | drop `sim_chip_socket.c` reference |
| `src/spicmds.c` | 12 | 197 | `sim_pending_cs` plumbing |
| `src/tmcuart.c` | ~30 | ~300 | Drop the broken `flavor` heuristic; keep the explicit-route short-circuit gated on `CONFIG_KALICO_SIM_TMCUART_BYPASS` and reading path from `KALICO_SIM_SOCK_DIR` env var |
| `src/runtime_sim_commands.c` | 263 | 0 | **whole file** |
| `src/Makefile` | ~3 | n/a | drop `runtime_sim_commands.c` reference |
| **Total firmware deletions** | **~660** | | |

Note: `runtime_tick.c`'s `[sim-progress]` diagnostic stays. The shim could
expose stepper counters via a new control-socket verb in a follow-up; for
v1 we leave the diagnostic alone.

### Shim additions

| File | Lines | Purpose |
|---|---:|---|
| `tools/sim_klippy/preload/libsim_intercept.c` | ~700 | The shim itself |
| `tools/sim_klippy/preload/Makefile` | ~15 | Build rule |
| `tools/sim_klippy/preload/README.md` | ~30 | What it is, why it's here |
| `tools/sim_klippy/orchestrator/launcher.py` | +~25 | Set `LD_PRELOAD` and `KALICO_SIM_SOCK_DIR` env vars; pre-create socket dir |
| `tools/sim_klippy/orchestrator/sim_control_client.py` | ~80 | Python client for the control socket |
| `tools/sim_klippy/conftest.py` | +~30 | Wire `SimContext.sim_control` into the fixture |
| `klippy/motion_toolhead.py` | +20 / -25 | Repoint `cmd_KALICO_SIM_ENDSTOP_SET_PIN` to control socket |
| **Total additions** | **~870** | |

Net delta: roughly **+210 LOC** (+870 added / -660 removed). The added LOC
is concentrated in the shim — one file, single responsibility, syscall ABI
contract — instead of spread across eight firmware files behind preprocessor
gates. The tmcuart exception is the single residual sim-aware firmware
piece, contained behind one fine-grained Kconfig flag.

### `KALICO_SIM` Kconfig option

Stays. After this migration its consumers are:

- `src/stm32/watchdog.c` — IWDG-disable for Renode
- `src/stm32/kalico_sim_clock.c` — RCC stub for Renode
- `src/stm32/runtime_tick_h7.c` — Renode timing tweak
- `src/stm32/runtime_tick_f4.c` — Renode timing tweak
- `src/runtime_tick.c` — sim-progress diagnostic (Linux + STM32)
- `src/generic/armcm_timer.c` — minor

The Kconfig help text gets clarified to make it explicit: `KALICO_SIM`
is for Renode-style ARM-emulator builds. MACH_LINUX builds use
`LD_PRELOAD` instead (with `KALICO_SIM=n`).

## Build integration

`tools/sim_klippy/preload/Makefile`:

```makefile
CC ?= gcc
CFLAGS = -O2 -Wall -Werror -fPIC -D_GNU_SOURCE
LDFLAGS = -shared -ldl -lpthread

libsim_intercept.so: libsim_intercept.c
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

clean:
	rm -f libsim_intercept.so
```

`tools/sim_klippy/run_local.sh` and the `conftest.py` ELF-build path get a
new step: `make -C tools/sim_klippy/preload libsim_intercept.so` before
spawning klipper.elf.

`tools/sim_klippy/orchestrator/launcher.py:_spawn_one`:

```python
env = os.environ.copy()
sock_dir = pathlib.Path(socket_path).parent / 'sim'
sock_dir.mkdir(exist_ok=True)
env['LD_PRELOAD'] = str(REPO_ROOT / 'tools/sim_klippy/preload/libsim_intercept.so')
env['KALICO_SIM_SOCK_DIR'] = str(sock_dir)
proc = subprocess.Popen(
    [elf, '-I', socket_path],
    stdout=log_fd, stderr=subprocess.STDOUT,
    env=env,
)
```

Sim-config flips: `tools/sim_klippy/configs/{h7,f4}-sim.config` set
`# CONFIG_KALICO_SIM is not set` (the firmware doesn't need to know it's
in a sim — the shim handles everything).

## Risks (specific)

### R1: A syscall the shim doesn't cover yet
Klipper might issue a syscall pattern we missed (e.g., `pselect`, `poll`,
`signalfd` on a /dev/* fd). Mitigation: shim's `open()` for unrecognized
device paths logs a warning and passes through to real_open. During v1
bring-up the launcher sets `KALICO_SIM_SHIM_VERBOSE=1` which adds
"intercepted call to <syscall> on fake fd" logging. Coverage gaps surface
loudly within the first test run.

### R2: errno propagation
Each shim handler must set errno correctly on failure paths so klipper's
existing error handling (`report_errno`, `try_shutdown`) sees the same
values it would on real silicon. Pattern: every "this would have failed
in real life" path explicitly `errno = E*` before returning -1. Test
coverage in the shim's own unit tests includes errno checks for at least
the common failure modes (E_NOENT for missing chip, EBADF for unknown fd).

### R3: fd table collision at FAKE_FD_BASE
`FAKE_FD_BASE = 1<<28 = 268435456`. Linux's `RLIMIT_NOFILE` defaults to
1024 with hard limit ~1M. No real fd ever reaches our base. Defensive
assertion: `assert(real_fd < FAKE_FD_BASE)` after every real_open — if it
ever fires, we know to lift the base.

### R4: `dup()` / `dup2()` / `dup3()` on a fake fd
Klipper doesn't currently use these on /dev/* fds (verified by grep
through the surface files). If a future change introduces them, the shim
needs to intercept `dup*` and either reject (set EBADF) or duplicate the
slot. v1 ignores; v2 adds when the need surfaces.

### R5: ~~tmcuart bit-bang observation correctness~~ → resolved by the firmware exception
The original concern was timing fragility in a shim-side UART decoder.
Resolved by the design decision to keep the tmcuart firmware short-circuit
on an explicit-route basis (see "Chip emulator routing → tmcuart" above).
The shim does not observe bit-bang output; firmware bypasses bit-banging
entirely and writes bytes directly to `${KALICO_SIM_SOCK_DIR}/tmcuart_<oid>`.
The risk this entry tracked is no longer in scope for v1.

### R6: Test-control socket race
A test sends `set_gpio_input chip=0 line=20 value=1` while klipper is mid
`GPIOHANDLE_GET_LINE_VALUES_IOCTL` on line 20. The mutex serializes them;
either ordering is correct. Tests that need "set the line, then immediately
read what klipper saw" have to acknowledge that klipper's reads are
asynchronous to the control socket. The control socket's reply ordering
("ok\n" arrives) means the shim's state was updated; klipper's next read
sees the new state.

### R7: Klipper exits before the shim cleans up control socket
The shim uses `__attribute__((destructor))` to unlink the control socket
path on normal exit. SIGKILL leaves a stale socket; orchestrator unlinks
before binding (existing pattern in ChipSocketServer).

### R8: macOS host
LD_PRELOAD is Linux-specific. Tests run inside a Linux Docker container
(per `tools/sim_klippy/run_local.sh`); macOS host status is unchanged.

## Renode coexistence

Renode runs ELFs built for `MACH_STM32H7` or `MACH_STM32F4` on emulated
silicon. Those builds:
- Don't link against libc (different toolchain, freestanding).
- Don't issue Linux syscalls.
- Don't see `LD_PRELOAD` env var.

There is no overlap between the LD_PRELOAD path and Renode. The two paths
share `KALICO_SIM=y` only as a convenience flag for "this is a simulator
build" but their consumers diverge: the LD_PRELOAD path uses the flag for
nothing (Category A files no longer reference it), Renode uses it for
IWDG / RCC / timing tweaks (Category B, untouched).

The Kconfig help text grows a sentence:
> When set on a `MACH_LINUX` build, this flag is currently a no-op.
> The MACH_LINUX simulator uses `LD_PRELOAD=libsim_intercept.so` to
> replace device access; firmware doesn't change. The flag matters only
> for `MACH_STM32*` builds intended for Renode.

## Test plan

### Direct shim tests (`tools/sim_klippy/preload/tests/`)

A small C test harness that:
1. Loads `libsim_intercept.so` directly (`dlopen`, then call exported
   functions in-process — no LD_PRELOAD needed for these tests).
2. Issues each intercepted syscall with synthetic inputs (open, ioctl,
   etc.).
3. Verifies the per-fd state updates and the control-socket protocol.

Coverage targets:
- Every device path returns a fake fd in the expected range.
- GPIO_GET_LINEHANDLE_IOCTL produces a child fd; SET/GET line values
  round-trip.
- SPI_IOC_MESSAGE round-trips bytes through a stub chip emulator.
- Control socket protocol parses each verb correctly; rejects malformed.
- errno values on failure paths.

### Integration tests (existing faithful sim)

`tools/sim_klippy/tests/test_boot.py` — the smoke baseline. Must pass
unchanged after the migration (with `KALICO_SIM=n` and shim active).

`tools/sim_klippy/tests/test_g28_x_smoke.py` — the discovery-loop baton.
After the migration, this surfaces whatever the *next* real bug is. The
TMC tmcuart-route-blocking-console-task issue we hit in v1 disappears
entirely (the firmware no longer connects sockets; the shim does, in a
non-blocking-to-firmware way).

### Cross-checks

- `cargo test -p kalico-host-rt` passes (no Rust changes).
- `cargo test -p motion-bridge` passes.
- A diff between two MACH_LINUX builds (sim with shim, vs. a hypothetical
  Pi-Klipper build) shows zero `KALICO_SIM`-gated divergence in
  `out/src/{linux/gpio.o, linux/hard_pwm.o, ...}`.

## Alternatives considered

### A1: Klippy `[sim_router]` extra (the v1 spec)
Already implemented as v1; rejected because it puts sim plumbing in
klippy and only fixes the broken `flavor` heuristic without addressing
the underlying "firmware shouldn't know about sim" problem.

### A2: Library-level interception (libgpiod)
Klipper bypasses libgpiod entirely (uses raw kernel ioctls in
`src/linux/gpio.c`). Library-level interception catches nothing. Not
viable.

### A3: FUSE-served `/dev` and `/sys`
Mount a FUSE filesystem that serves fake `/dev/gpiochip*`,
`/dev/spidev*`, `/sys/class/pwm/*`, `/sys/bus/iio/*`. Klipper opens these
exactly like real silicon; no LD_PRELOAD needed. Architecturally
strongest (every syscall is real, including all the kernel-side checks)
but: requires CAP_SYS_ADMIN inside the container; FUSE not always
available; per-test mount/unmount is heavy (~100 ms × N tests). Higher
fidelity than LD_PRELOAD but the operational cost is worse for the
faithful-sim use case. Re-considered if test fidelity ever proves
insufficient.

### A4: QEMU/Renode for everything
The high-fidelity path. Already exists for STM32 sim. Slow (~10x
slower than host-process sim). Not a replacement for the fast sim;
complementary.

### A5: Keep the firmware short-circuits, fix only the immediate bug
The v0 approach. Fixes one symptom; doesn't address the layer
violation; sets up for the next "I need to fake X" change to add another
firmware #ifdef. Rejected by the user's "no throwaway code beyond 1-2
lines" rule.

## Out of scope

- Generalizing the shim to non-klipper consumers. The shim is calibrated
  to klipper's exact syscall pattern; growing it to a general device-mock
  framework is a separate project.
- Cycle-accurate fidelity (Renode's territory).
- Fault injection (syscall-level errno fault testing). Could grow a
  control-socket verb later: `inject_fault syscall=ioctl errno=EIO once=1`.
- Replacing the bridge-protocol PTY — klipper.elf still serves its
  klippy-facing PTY normally; that's not a /dev/* surface.

## Resolved decisions

- **tmcuart**: firmware keeps a contained short-circuit on an explicit
  env-var-driven path. Shim does not observe bit-bang output. Exception
  is gated behind `CONFIG_KALICO_SIM_TMCUART_BYPASS`, ~50 LOC. See
  "Chip emulator routing → tmcuart" for rationale.
- **Control socket**: line-oriented text protocol. Low-traffic
  test-control surface; debuggability beats throughput. See "Control
  socket protocol" for the full grammar.
- **Liveness check**: `ping` verb included in the control socket
  protocol. Launcher uses it to verify the shim is up before klipper
  starts issuing real syscalls — makes startup deterministic.
