# Faithful klippy-in-loop Simulator Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a klippy-in-loop simulator that runs the user's actual `printer.cfg` + third-party plugins end-to-end (boot → G28 → small print) so host-side regressions surface in the sim before they brick the printer.

**Architecture:** Two `klipper.elf` MACH_LINUX instances (H7-flavored with KALICO_RUNTIME, F446-flavored without) talk to klippy over `/tmp/klipper_sim_*` sim sockets via the existing motion bridge. Per-chip behavioral emulators (TMC5160 ×4, TMC2209 ×3, Beacon serial) run in-process on the orchestrator side; SPI/UART transfers are intercepted in the firmware's Linux shim and routed through Unix sockets to the Python emulators. ADC, sensorless StallGuard, and pin/serial path translation are layered on the existing `runtime_sim_*` shim infrastructure.

**Tech Stack:** Python 3 (orchestrator + tests), pytest, PyTrinamic (TMC register tables), Linux MACH_LINUX C shims, Docker (build + run), the existing motion bridge + kalico runtime.

**Spec:** `docs/superpowers/specs/2026-05-08-faithful-klippy-sim-design.md`

---

## File Map

### Created files

| File | Role |
|---|---|
| `tools/sim_klippy/configs/h7-sim.config` | Linux MACH_LINUX + KALICO_RUNTIME=y .config |
| `tools/sim_klippy/configs/f4-sim.config` | Linux MACH_LINUX + no KALICO_RUNTIME .config |
| `tools/sim_klippy/orchestrator/__init__.py` | Package marker |
| `tools/sim_klippy/orchestrator/launcher.py` | Spawns + waits-for-ready two `klipper.elf` MCUs |
| `tools/sim_klippy/orchestrator/chip_socket_server.py` | Async Unix-socket server for SPI/UART chip emulators |
| `tools/sim_klippy/orchestrator/tmc5160_emulator.py` | TMC5160 behavioral chip emulator |
| `tools/sim_klippy/orchestrator/tmc2209_emulator.py` | TMC2209 behavioral chip emulator |
| `tools/sim_klippy/orchestrator/beacon_serial_stub.py` | Beacon-protocol serial stub |
| `tools/sim_klippy/orchestrator/adc_stub.py` | ADC + heater feedback model |
| `tools/sim_klippy/orchestrator/sensorless_trigger.py` | Step-count → virtual-position → DIAG dance |
| `tools/sim_klippy/orchestrator/overrides.py` | Pin/serial path override layer over klippy config |
| `tools/sim_klippy/pin-overrides.toml` | Auto-generated pin/serial mapping |
| `tools/sim_klippy/sim_geometry.toml` | Per-axis virtual wall positions |
| `tools/sim_klippy/conftest.py` | pytest fixtures (orchestrator, klippy bring-up) |
| `tools/sim_klippy/tests/__init__.py` | Package marker |
| `tools/sim_klippy/tests/test_boot.py` | Success bar 1 |
| `tools/sim_klippy/tests/test_g28_full.py` | Success bar 2 |
| `tools/sim_klippy/tests/test_small_print.py` | Success bar 3 |
| `tools/sim_klippy/fixtures/small_print.gcode` | 30-line slicer-emitted print fixture |
| `src/linux/sim_chip_socket.c` | C-side Unix-socket transport for chip stubs |
| `src/linux/sim_chip_socket.h` | C-side transport header |

### Modified files

| File | Change |
|---|---|
| `src/linux/spidev.c` | Add `sim:<path>` bus prefix → route through `sim_chip_socket` |
| `src/tmcuart.c` | Add sim-mode passthrough when oid is registered as routed |
| `src/runtime_sim_commands.c` | Add `runtime_sim_route_spi`, `runtime_sim_route_tmcuart`, `runtime_sim_adc_set` commands |
| `src/linux/Makefile` | Link `sim_chip_socket.c` into MACH_LINUX + CONFIG_KALICO_SIM builds |
| `tools/sim_klippy/Dockerfile` | Add `pytrinamic`, `pytest` to apt+pip install |
| `tools/sim_klippy/run_local.sh` | Add `make sim-test` invocation; spawn two MCUs |
| `tools/sim_klippy/run.py` | Replace single-MCU launch with launcher.py call |
| `Makefile.kalico` | Add `sim-test` target |

---

## Phase 0 — Prep + baseline build

### Task 0.1: Add PyTrinamic + pytest to the sim Docker image

**Files:**
- Modify: `tools/sim_klippy/Dockerfile`

- [ ] **Step 1: Read the current Dockerfile**

Run: `cat tools/sim_klippy/Dockerfile`
Expected: existing apt-get install block ending around line 28.

- [ ] **Step 2: Add pip install for pytrinamic and pytest**

Append after the existing apt-get block:

```dockerfile
# Sim test dependencies — pytrinamic provides TMC register-table constants;
# pytest drives the test harness.
RUN pip3 install --no-cache-dir --break-system-packages \
    pytrinamic==0.2.6 \
    pytest==8.3.4
```

- [ ] **Step 3: Rebuild image**

Run: `docker build -t kalico-sim tools/sim_klippy/`
Expected: build succeeds; final stage installs the two pip packages.

- [ ] **Step 4: Verify install**

Run: `docker run --rm kalico-sim python3 -c "import pytrinamic.modules.TMC5160 as t; print(t.TMC5160_register.GSTAT)"`
Expected: prints `1` (GSTAT register address) without ImportError.

Run: `docker run --rm kalico-sim pytest --version`
Expected: prints pytest 8.x.y.

- [ ] **Step 5: Commit**

```bash
git add tools/sim_klippy/Dockerfile
git commit -m "sim: add pytrinamic + pytest to Docker image"
```

### Task 0.2: Snapshot two `.config` files for H7-sim and F4-sim builds

**Files:**
- Create: `tools/sim_klippy/configs/h7-sim.config`
- Create: `tools/sim_klippy/configs/f4-sim.config`

- [ ] **Step 1: Generate H7-sim .config from menuconfig defaults**

In Docker (`docker run --rm -v $(pwd):/work -w /work kalico-sim bash`):
```bash
cp .config /tmp/.config.bak 2>/dev/null || true
cat > tools/sim_klippy/configs/h7-sim.config <<'EOF'
CONFIG_LOW_LEVEL_OPTIONS=y
CONFIG_MACH_LINUX=y
CONFIG_BOARD_DIRECTORY="linux"
CONFIG_CLOCK_FREQ=20000000
CONFIG_USBSERIAL=n
CONFIG_SERIAL=n
CONFIG_USB_SERIAL_NUMBER_CHIPID=n
CONFIG_NEED_GPIO_CHARDEV=y
CONFIG_HAVE_GPIO=y
CONFIG_HAVE_GPIO_ADC=y
CONFIG_HAVE_GPIO_SPI=y
CONFIG_HAVE_GPIO_I2C=y
CONFIG_HAVE_STRICT_TIMING=y
CONFIG_KALICO_RUNTIME=y
CONFIG_KALICO_SIM=y
CONFIG_RUNTIME_TARGET_LARGE=y
CONFIG_RUNTIME_MAX_CONTROL_POINTS=1830
CONFIG_RUNTIME_MAX_KNOT_VECTOR_LEN=1850
CONFIG_RUNTIME_MAX_DEGREE=10
CONFIG_RUNTIME_CURVE_POOL_N=16
EOF
cp tools/sim_klippy/configs/h7-sim.config .config
make olddefconfig 2>&1 | tail -5
cp .config tools/sim_klippy/configs/h7-sim.config
```

Expected: olddefconfig succeeds; the resulting .config is a complete, fully-resolved file.

- [ ] **Step 2: Generate F4-sim .config (no runtime, no sim shim runtime hooks needed)**

```bash
cat > tools/sim_klippy/configs/f4-sim.config <<'EOF'
CONFIG_LOW_LEVEL_OPTIONS=y
CONFIG_MACH_LINUX=y
CONFIG_BOARD_DIRECTORY="linux"
CONFIG_CLOCK_FREQ=20000000
CONFIG_USBSERIAL=n
CONFIG_SERIAL=n
CONFIG_NEED_GPIO_CHARDEV=y
CONFIG_HAVE_GPIO=y
CONFIG_HAVE_GPIO_ADC=y
CONFIG_HAVE_GPIO_SPI=y
CONFIG_HAVE_GPIO_I2C=y
CONFIG_HAVE_STRICT_TIMING=y
CONFIG_WANT_TMCUART=y
EOF
cp tools/sim_klippy/configs/f4-sim.config .config
make olddefconfig 2>&1 | tail -5
cp .config tools/sim_klippy/configs/f4-sim.config
```

- [ ] **Step 3: Restore original .config**

```bash
[ -f /tmp/.config.bak ] && cp /tmp/.config.bak .config || rm -f .config
```

- [ ] **Step 4: Verify both configs build**

```bash
cp tools/sim_klippy/configs/h7-sim.config .config && make clean && make -j4 2>&1 | tail -3 && cp out/klipper.elf out/klipper-h7-sim.elf
cp tools/sim_klippy/configs/f4-sim.config .config && make clean && make -j4 2>&1 | tail -3 && cp out/klipper.elf out/klipper-f4-sim.elf
ls -lh out/klipper-h7-sim.elf out/klipper-f4-sim.elf
```

Expected: both `make` succeed; two ELFs land in `out/`.

- [ ] **Step 5: Commit**

```bash
git add tools/sim_klippy/configs/
git commit -m "sim: snapshot h7-sim + f4-sim Linux MACH_LINUX .config files"
```

---

## Phase 1 — Firmware-side SPI/UART socket interception

The firmware needs to forward SPI/UART byte streams through Unix sockets to the orchestrator instead of opening real `/dev/spidev*` devices. We add a `sim_chip_socket.c` transport, extend `spidev.c` with a `sim:` bus prefix, add a tmcuart sim-mode passthrough, and expose the routing-registration commands.

### Task 1.1: Add `sim_chip_socket.c` Unix-socket transport

**Files:**
- Create: `src/linux/sim_chip_socket.c`
- Create: `src/linux/sim_chip_socket.h`

- [ ] **Step 1: Write header**

```c
// src/linux/sim_chip_socket.h
#ifndef KALICO_SIM_CHIP_SOCKET_H
#define KALICO_SIM_CHIP_SOCKET_H
#include <stdint.h>
#include <stddef.h>

// Open (or get cached) a Unix-domain stream socket connected to `path`.
// Returns fd >= 0 on success, -1 on error (and shutdown()s the firmware).
int sim_chip_socket_connect(const char *path);

// Synchronous request/response over the socket. Writes `tx_len` bytes,
// reads exactly `rx_len` bytes back. Returns 0 on success, -1 on error.
int sim_chip_socket_xfer(int fd, const uint8_t *tx, size_t tx_len,
                         uint8_t *rx, size_t rx_len);

#endif
```

- [ ] **Step 2: Write implementation**

```c
// src/linux/sim_chip_socket.c
#include "sim_chip_socket.h"
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include "internal.h" // report_errno
#include "sched.h"    // shutdown

#define MAX_SOCKETS 16

struct sock_entry { char path[128]; int fd; };
static struct sock_entry sockets[MAX_SOCKETS];
static int sockets_count = 0;

int sim_chip_socket_connect(const char *path) {
    for (int i = 0; i < sockets_count; i++)
        if (strcmp(sockets[i].path, path) == 0)
            return sockets[i].fd;
    if (sockets_count >= MAX_SOCKETS)
        shutdown("Too many sim chip sockets");
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) {
        report_errno("sim_chip_socket socket()", fd);
        shutdown("Unable to create sim chip socket");
    }
    struct sockaddr_un addr = {0};
    addr.sun_family = AF_UNIX;
    snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", path);
    int retries = 50;
    while (retries-- > 0) {
        if (connect(fd, (struct sockaddr*)&addr, sizeof(addr)) == 0) break;
        if (errno != ENOENT && errno != ECONNREFUSED) {
            report_errno("sim_chip_socket connect()", -1);
            shutdown("Unable to connect sim chip socket");
        }
        usleep(100000);
    }
    if (retries <= 0) shutdown("sim chip socket connect timed out");
    snprintf(sockets[sockets_count].path, sizeof(sockets[sockets_count].path),
             "%s", path);
    sockets[sockets_count].fd = fd;
    sockets_count++;
    return fd;
}

int sim_chip_socket_xfer(int fd, const uint8_t *tx, size_t tx_len,
                         uint8_t *rx, size_t rx_len) {
    size_t off = 0;
    while (off < tx_len) {
        ssize_t n = write(fd, tx + off, tx_len - off);
        if (n <= 0) return -1;
        off += n;
    }
    off = 0;
    while (off < rx_len) {
        ssize_t n = read(fd, rx + off, rx_len - off);
        if (n <= 0) return -1;
        off += n;
    }
    return 0;
}
```

- [ ] **Step 3: Hook into Linux Makefile**

Modify `src/linux/Makefile` — find the line that lists Linux sources (around line 10–20). Add:

```makefile
src-y += linux/sim_chip_socket.c
```

(Place near `src-y += linux/spidev.c`.)

- [ ] **Step 4: Verify it compiles standalone**

```bash
cp tools/sim_klippy/configs/h7-sim.config .config
make 2>&1 | grep -E 'sim_chip_socket|error' | head -5
```

Expected: file compiles, no errors.

- [ ] **Step 5: Commit**

```bash
git add src/linux/sim_chip_socket.c src/linux/sim_chip_socket.h src/linux/Makefile
git commit -m "sim(linux): sim_chip_socket — Unix-socket SPI/UART transport"
```

### Task 1.2: Extend `spidev.c` with a `sim:` bus prefix

**Files:**
- Modify: `src/linux/spidev.c`

- [ ] **Step 1: Inspect current spi_setup**

Run: `sed -n '1,40p;65,110p' src/linux/spidev.c`
Identify: `DECL_ENUMERATION_RANGE("spi_bus", "spidev0.0", ...)` block, `spi_setup` opens `/dev/spidev<bus>.<dev>` via `spi_open`.

- [ ] **Step 2: Add sim bus enum and routing table**

Insert after the existing `DECL_ENUMERATION_RANGE` lines, before `struct spi_s`:

```c
DECL_ENUMERATION_RANGE("spi_bus", "sim_spi0", 0xFF00, 16);
```

The `0xFF00..0xFF0F` range identifies sim sockets; klippy will encode the bus as `sim_spi<N>` in printer.cfg pin-overrides.

Add the routing-table struct alongside `struct spi_s`:

```c
#include "sim_chip_socket.h"

struct sim_spi_route { uint32_t bus; char socket_path[64]; };
static struct sim_spi_route sim_routes[16];
static int sim_routes_count = 0;

static const char *
sim_spi_socket_path(uint32_t bus) {
    for (int i = 0; i < sim_routes_count; i++)
        if (sim_routes[i].bus == bus) return sim_routes[i].socket_path;
    return NULL;
}

void sim_spi_register_route(uint32_t bus, const char *path) {
    for (int i = 0; i < sim_routes_count; i++)
        if (sim_routes[i].bus == bus) {
            snprintf(sim_routes[i].socket_path,
                     sizeof(sim_routes[i].socket_path), "%s", path);
            return;
        }
    snprintf(sim_routes[sim_routes_count].socket_path,
             sizeof(sim_routes[sim_routes_count].socket_path), "%s", path);
    sim_routes[sim_routes_count].bus = bus;
    sim_routes_count++;
}
```

- [ ] **Step 3: Modify spi_setup to short-circuit on sim buses**

Find the existing `spi_setup` function. Wrap the body so the sim path takes precedence:

```c
struct spi_config
spi_setup(uint32_t bus, uint8_t mode, uint32_t rate)
{
    const char *sim_path = sim_spi_socket_path(bus);
    if (sim_path) {
        int fd = sim_chip_socket_connect(sim_path);
        return (struct spi_config) { fd, rate };
    }
    int bus_id = SPIBUS_TO_BUS(bus), dev_id = SPIBUS_TO_DEV(bus);
    int fd = spi_open(bus_id, dev_id);
    /* ... rest of existing spi_setup body unchanged ... */
}
```

(Preserve the existing ioctl calls in the non-sim branch; only the early sim-path return is new.)

- [ ] **Step 4: Modify spi_transfer to short-circuit on sim buses**

Find the existing `spi_transfer`. After the `if (!len) return;` line, add:

```c
    // For sim buses we identify via the bus number range — but the fd is what
    // we actually have. Sim fds were opened by sim_chip_socket_connect; route
    // through the socket xfer helper instead of the spidev ioctl.
    for (int i = 0; i < sim_routes_count; i++) {
        if (sim_chip_socket_connect(sim_routes[i].socket_path) == config.fd) {
            uint8_t scratch[256];
            if (len > sizeof(scratch))
                shutdown("sim spi xfer too long");
            memcpy(scratch, data, len);
            uint8_t reply[256] = {0};
            sim_chip_socket_xfer(config.fd, scratch, len,
                                 receive_data ? data : reply, len);
            return;
        }
    }
```

- [ ] **Step 5: Add header-level declaration so runtime_sim_commands.c can call sim_spi_register_route**

In `src/linux/sim_chip_socket.h`, add at the bottom (before `#endif`):

```c
// Register a SPI bus → Unix-socket-path mapping. Called from the
// runtime_sim_route_spi command handler; takes effect on the next
// spi_setup that resolves to this bus.
void sim_spi_register_route(uint32_t bus, const char *path);
```

- [ ] **Step 6: Verify compile**

```bash
cp tools/sim_klippy/configs/h7-sim.config .config
make clean && make -j4 2>&1 | grep -E 'spidev\.c|error' | head -10
```

Expected: clean compile.

- [ ] **Step 7: Commit**

```bash
git add src/linux/spidev.c src/linux/sim_chip_socket.h
git commit -m "sim(linux): spidev — sim:<path> bus route through Unix socket"
```

### Task 1.3: Add tmcuart sim-mode passthrough

**Files:**
- Modify: `src/tmcuart.c`

- [ ] **Step 1: Read tmcuart.c structure**

Run: `grep -n "DECL_COMMAND\|tmcuart_send\|config_tmcuart\|struct tmcuart_s" src/tmcuart.c | head -10`
Expected: identifies the `command_tmcuart_send` handler and the `tmcuart_s` config struct (typically with rx_pin, tx_pin, bit_time).

- [ ] **Step 2: Add sim-route table for tmcuart oids**

Near the top of `src/tmcuart.c`, after includes:

```c
#if CONFIG_MACH_LINUX
#include "sim_chip_socket.h"

struct sim_uart_route { uint8_t oid; char socket_path[64]; int fd; };
static struct sim_uart_route sim_uart_routes[8];
static int sim_uart_routes_count = 0;

void sim_tmcuart_register_route(uint8_t oid, const char *path) {
    for (int i = 0; i < sim_uart_routes_count; i++)
        if (sim_uart_routes[i].oid == oid) {
            snprintf(sim_uart_routes[i].socket_path,
                     sizeof(sim_uart_routes[i].socket_path), "%s", path);
            sim_uart_routes[i].fd = -1;
            return;
        }
    sim_uart_routes[sim_uart_routes_count].oid = oid;
    snprintf(sim_uart_routes[sim_uart_routes_count].socket_path,
             sizeof(sim_uart_routes[sim_uart_routes_count].socket_path),
             "%s", path);
    sim_uart_routes[sim_uart_routes_count].fd = -1;
    sim_uart_routes_count++;
}

static int sim_uart_lookup_fd(uint8_t oid) {
    for (int i = 0; i < sim_uart_routes_count; i++)
        if (sim_uart_routes[i].oid == oid) {
            if (sim_uart_routes[i].fd < 0)
                sim_uart_routes[i].fd =
                    sim_chip_socket_connect(sim_uart_routes[i].socket_path);
            return sim_uart_routes[i].fd;
        }
    return -1;
}
#endif
```

- [ ] **Step 3: Short-circuit `command_tmcuart_send` on sim oids**

Find `command_tmcuart_send` (it parses oid, sends the request bytes via bit-banging, reads back via bit-banging). At the very top of the handler body, before the bit-bang dispatch:

```c
#if CONFIG_MACH_LINUX
    {
        uint8_t oid = args[0];
        int sfd = sim_uart_lookup_fd(oid);
        if (sfd >= 0) {
            uint8_t write_len = args[1];
            uint8_t *write_data = command_decode_ptr(args[2]);
            uint8_t read_len = args[3];
            uint8_t reply[16] = {0};
            if (read_len > sizeof(reply)) shutdown("sim tmcuart read_len too big");
            if (sim_chip_socket_xfer(sfd, write_data, write_len,
                                     reply, read_len) != 0)
                shutdown("sim tmcuart xfer failed");
            sendf("tmcuart_response oid=%c read=%*s", oid, read_len, reply);
            return;
        }
    }
#endif
```

(The exact arg indices may differ slightly — copy the existing handler's variable names. The point: when `sim_uart_lookup_fd` returns >= 0, route through the socket and `sendf` the response, return early. Otherwise fall through to the legacy bit-bang path.)

- [ ] **Step 4: Add header declaration**

In `src/linux/sim_chip_socket.h`, add:

```c
// Register a tmcuart oid → Unix-socket-path mapping. Mirror of
// sim_spi_register_route for the bit-banged TMC2209 path.
void sim_tmcuart_register_route(uint8_t oid, const char *path);
```

- [ ] **Step 5: Verify compile**

```bash
cp tools/sim_klippy/configs/f4-sim.config .config
make clean && make -j4 2>&1 | grep -E 'tmcuart\.c|error' | head -10
```

Expected: clean compile.

- [ ] **Step 6: Commit**

```bash
git add src/tmcuart.c src/linux/sim_chip_socket.h
git commit -m "sim(tmcuart): bypass bit-bang — route through Unix socket on sim oids"
```

### Task 1.4: Add `runtime_sim_route_spi` and `runtime_sim_route_tmcuart` commands

**Files:**
- Modify: `src/runtime_sim_commands.c`

- [ ] **Step 1: Read existing command pattern**

Run: `sed -n '177,210p' src/runtime_sim_commands.c`
Identify: `command_runtime_sim_endstop_set_pin` pattern: parses args, calls underlying function, `sendf` response, `DECL_COMMAND` registration.

- [ ] **Step 2: Add the two new commands**

Append to `src/runtime_sim_commands.c` (before any closing #endif):

```c
#if CONFIG_MACH_LINUX

#include "linux/sim_chip_socket.h"

void
command_runtime_sim_route_spi(uint32_t *args)
{
    uint32_t bus = args[0];
    uint8_t path_len = args[1];
    char path[128] = {0};
    if (path_len >= sizeof(path)) shutdown("sim_route_spi path too long");
    memcpy(path, command_decode_ptr(args[2]), path_len);
    sim_spi_register_route(bus, path);
    sendf("runtime_sim_route_spi_response bus=%u result=%i", bus, 0);
}
DECL_COMMAND(command_runtime_sim_route_spi,
    "runtime_sim_route_spi bus=%u path=%*s");

void
command_runtime_sim_route_tmcuart(uint32_t *args)
{
    uint8_t oid = args[0];
    uint8_t path_len = args[1];
    char path[128] = {0};
    if (path_len >= sizeof(path)) shutdown("sim_route_tmcuart path too long");
    memcpy(path, command_decode_ptr(args[2]), path_len);
    sim_tmcuart_register_route(oid, path);
    sendf("runtime_sim_route_tmcuart_response oid=%c result=%i", oid, 0);
}
DECL_COMMAND(command_runtime_sim_route_tmcuart,
    "runtime_sim_route_tmcuart oid=%c path=%*s");

#endif
```

- [ ] **Step 3: Verify both commands appear in the dict**

```bash
cp tools/sim_klippy/configs/h7-sim.config .config
make clean && make -j4 2>&1 | tail -3
grep -E 'runtime_sim_route_(spi|tmcuart)' out/klipper.dict | head
```

Expected: two matching command lines in `out/klipper.dict`.

- [ ] **Step 4: Add `runtime_sim_adc_set` command**

In the same file, append:

```c
#if CONFIG_MACH_LINUX

// Sim-only: set the simulated ADC reading for a virtual gpio. Linux
// MACH_LINUX's analog.c already supports a simulated-value table; this
// command exposes it to the orchestrator.
extern void analog_set_simulated_value(uint8_t adc_pin, uint16_t value);

void
command_runtime_sim_adc_set(uint32_t *args)
{
    uint8_t adc_pin = args[0];
    uint16_t value = args[1];
    analog_set_simulated_value(adc_pin, value);
    sendf("runtime_sim_adc_set_response adc_pin=%c result=%i", adc_pin, 0);
}
DECL_COMMAND(command_runtime_sim_adc_set,
    "runtime_sim_adc_set adc_pin=%c value=%hu");

#endif
```

(`analog_set_simulated_value` will be added in Task 5.1.)

- [ ] **Step 5: Commit**

```bash
git add src/runtime_sim_commands.c
git commit -m "sim(commands): runtime_sim_route_spi/tmcuart + runtime_sim_adc_set"
```

---

## Phase 2 — Orchestrator core (launcher + chip socket server + override layer)

### Task 2.1: Write `chip_socket_server.py` with a contract test

**Files:**
- Create: `tools/sim_klippy/orchestrator/__init__.py`
- Create: `tools/sim_klippy/orchestrator/chip_socket_server.py`
- Create: `tools/sim_klippy/tests/__init__.py`
- Create: `tools/sim_klippy/tests/test_chip_socket_server.py`

- [ ] **Step 1: Write the contract test**

```python
# tools/sim_klippy/tests/test_chip_socket_server.py
"""Contract test for chip_socket_server: clients connect via Unix socket,
send arbitrary bytes, the registered handler returns the same byte count
back. Wire layout is fully driven by the chip emulator — the server is
just a synchronous request/response framer over the socket."""
import os
import socket
import threading
import time
import pytest

from tools.sim_klippy.orchestrator.chip_socket_server import (
    ChipSocketServer,
)


def test_echo_handler_round_trips():
    sock_path = "/tmp/test_chip_socket_echo"
    if os.path.exists(sock_path):
        os.unlink(sock_path)

    def echo_handler(req: bytes) -> bytes:
        return req[::-1]  # reverse, so we can tell handler ran

    server = ChipSocketServer(sock_path, echo_handler)
    server.start()
    try:
        # Wait briefly for accept loop to bind
        for _ in range(50):
            if os.path.exists(sock_path):
                break
            time.sleep(0.01)
        client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        client.connect(sock_path)
        client.sendall(b"\x01\x02\x03\x04")
        reply = client.recv(4)
        assert reply == b"\x04\x03\x02\x01"
        client.close()
    finally:
        server.stop()
        if os.path.exists(sock_path):
            os.unlink(sock_path)
```

- [ ] **Step 2: Run the test (should fail — module missing)**

```bash
cd /work && python3 -m pytest tools/sim_klippy/tests/test_chip_socket_server.py -v 2>&1 | tail -10
```

Expected: ImportError or ModuleNotFoundError on `chip_socket_server`.

- [ ] **Step 3: Implement the server**

```python
# tools/sim_klippy/orchestrator/__init__.py
# (empty)
```

```python
# tools/sim_klippy/orchestrator/chip_socket_server.py
"""Async-ish Unix-socket server for chip emulators.

Accepts one client per socket; each request is a fixed-length byte
sequence the handler interprets per-chip. The handler returns a reply
of equal length (TMC SPI is symmetric; TMC2209 UART writes are 8-byte
no-reply but reads are 8-byte reply — the chip emulators handle the
asymmetry by reading the request first and constructing the reply
accordingly).

Threaded model: one accept thread, one worker thread per connection.
Sufficient for our handful of chip stubs."""
import os
import socket
import threading
from typing import Callable


class ChipSocketServer:
    def __init__(self, path: str, handler: Callable[[bytes], bytes],
                 chunk: int = 16):
        self._path = path
        self._handler = handler
        self._chunk = chunk
        self._sock = None
        self._accept_thread = None
        self._stop = threading.Event()

    def start(self) -> None:
        if os.path.exists(self._path):
            os.unlink(self._path)
        self._sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self._sock.bind(self._path)
        self._sock.listen(4)
        self._sock.settimeout(0.1)
        self._accept_thread = threading.Thread(
            target=self._accept_loop, daemon=True
        )
        self._accept_thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._sock:
            self._sock.close()
        if self._accept_thread:
            self._accept_thread.join(timeout=1.0)

    def _accept_loop(self):
        while not self._stop.is_set():
            try:
                client, _ = self._sock.accept()
            except (socket.timeout, OSError):
                continue
            t = threading.Thread(target=self._serve, args=(client,), daemon=True)
            t.start()

    def _serve(self, client: socket.socket):
        client.settimeout(1.0)
        try:
            while not self._stop.is_set():
                data = client.recv(self._chunk)
                if not data:
                    break
                reply = self._handler(data)
                if reply:
                    client.sendall(reply)
        except (socket.timeout, ConnectionResetError, OSError):
            pass
        finally:
            client.close()
```

- [ ] **Step 4: Re-run the test**

```bash
python3 -m pytest tools/sim_klippy/tests/test_chip_socket_server.py -v 2>&1 | tail -5
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add tools/sim_klippy/orchestrator/__init__.py tools/sim_klippy/orchestrator/chip_socket_server.py tools/sim_klippy/tests/__init__.py tools/sim_klippy/tests/test_chip_socket_server.py
git commit -m "sim(orchestrator): chip_socket_server + contract test"
```

### Task 2.2: Write `launcher.py` to spawn two `klipper.elf` MCUs

**Files:**
- Create: `tools/sim_klippy/orchestrator/launcher.py`
- Create: `tools/sim_klippy/tests/test_launcher.py`

- [ ] **Step 1: Write the launcher contract test**

```python
# tools/sim_klippy/tests/test_launcher.py
"""launcher.spawn_mcus brings up two klipper.elf processes, returns
handles that include the PTY socket paths, and tears them down cleanly
on .shutdown(). After spawn returns, both PTYs must exist and be
write-readable."""
import os
import pytest
from tools.sim_klippy.orchestrator.launcher import spawn_mcus, McuHandle


def test_spawn_brings_up_both_mcus(tmp_path):
    handles = spawn_mcus(
        h7_elf="out/klipper-h7-sim.elf",
        f4_elf="out/klipper-f4-sim.elf",
        h7_socket="/tmp/test_klipper_sim_h7",
        f4_socket="/tmp/test_klipper_sim_f4",
        log_dir=str(tmp_path),
    )
    try:
        assert isinstance(handles.h7, McuHandle)
        assert isinstance(handles.f4, McuHandle)
        assert os.path.exists(handles.h7.socket_path)
        assert os.path.exists(handles.f4.socket_path)
        assert handles.h7.process.poll() is None  # still running
        assert handles.f4.process.poll() is None
    finally:
        handles.shutdown()
```

- [ ] **Step 2: Run the test (should fail)**

```bash
python3 -m pytest tools/sim_klippy/tests/test_launcher.py -v 2>&1 | tail -10
```

Expected: ImportError.

- [ ] **Step 3: Implement launcher**

```python
# tools/sim_klippy/orchestrator/launcher.py
"""Spawn the two Linux MACH_LINUX klipper.elf instances that back the
faithful sim. H7-flavored has KALICO_RUNTIME=y; F4-flavored doesn't.

Each instance opens a PTY at the supplied socket path. We wait for
both PTYs to exist before returning, so callers can immediately do
attach_serial against them."""
import dataclasses
import os
import signal
import subprocess
import time
from typing import Optional


@dataclasses.dataclass
class McuHandle:
    name: str
    process: subprocess.Popen
    socket_path: str
    log_path: str


@dataclasses.dataclass
class McuHandles:
    h7: McuHandle
    f4: McuHandle

    def shutdown(self) -> None:
        for h in (self.h7, self.f4):
            if h.process.poll() is None:
                h.process.send_signal(signal.SIGTERM)
        for h in (self.h7, self.f4):
            try:
                h.process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                h.process.kill()
                h.process.wait(timeout=1.0)
        for h in (self.h7, self.f4):
            try:
                os.unlink(h.socket_path)
            except FileNotFoundError:
                pass


def _spawn_one(elf: str, socket_path: str, log_path: str,
               name: str) -> McuHandle:
    if os.path.exists(socket_path):
        os.unlink(socket_path)
    log_fd = open(log_path, "wb")
    proc = subprocess.Popen(
        [elf, "-I", socket_path],
        stdout=log_fd,
        stderr=subprocess.STDOUT,
    )
    # Wait for the PTY symlink to appear (klipper.elf creates it after init)
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        if os.path.exists(socket_path):
            return McuHandle(
                name=name, process=proc,
                socket_path=socket_path, log_path=log_path,
            )
        if proc.poll() is not None:
            raise RuntimeError(
                f"{name}: klipper.elf exited early (rc={proc.returncode}); "
                f"see {log_path}"
            )
        time.sleep(0.05)
    proc.kill()
    raise RuntimeError(f"{name}: PTY {socket_path} did not appear in 5s")


def spawn_mcus(
    h7_elf: str = "out/klipper-h7-sim.elf",
    f4_elf: str = "out/klipper-f4-sim.elf",
    h7_socket: str = "/tmp/klipper_sim_h7",
    f4_socket: str = "/tmp/klipper_sim_f4",
    log_dir: str = "/tmp/klipper_sim_logs",
) -> McuHandles:
    os.makedirs(log_dir, exist_ok=True)
    h7 = _spawn_one(
        h7_elf, h7_socket, os.path.join(log_dir, "h7.log"), "h7"
    )
    f4 = _spawn_one(
        f4_elf, f4_socket, os.path.join(log_dir, "f4.log"), "f4"
    )
    return McuHandles(h7=h7, f4=f4)
```

- [ ] **Step 4: Pre-build both ELFs so the test can run**

```bash
cp tools/sim_klippy/configs/h7-sim.config .config && make clean && make -j4 && cp out/klipper.elf out/klipper-h7-sim.elf
cp tools/sim_klippy/configs/f4-sim.config .config && make clean && make -j4 && cp out/klipper.elf out/klipper-f4-sim.elf
```

- [ ] **Step 5: Run the test**

```bash
python3 -m pytest tools/sim_klippy/tests/test_launcher.py -v 2>&1 | tail -8
```

Expected: PASS (within 5 sec). Both PTYs come up; clean shutdown.

- [ ] **Step 6: Commit**

```bash
git add tools/sim_klippy/orchestrator/launcher.py tools/sim_klippy/tests/test_launcher.py
git commit -m "sim(orchestrator): launcher — spawn two klipper.elf MCUs"
```

### Task 2.3: Write the pin/serial override layer

**Files:**
- Create: `tools/sim_klippy/orchestrator/overrides.py`
- Create: `tools/sim_klippy/pin-overrides.toml`
- Create: `tools/sim_klippy/tests/test_overrides.py`

- [ ] **Step 1: Generate pin-overrides.toml**

By scanning `tools/sim_klippy/printer_real/config/printer.cfg` for pin-bearing keys and emitting a one-time mapping. Run interactively:

```bash
python3 - <<'EOF' > tools/sim_klippy/pin-overrides.toml
import re, sys

cfg = open("tools/sim_klippy/printer_real/config/printer.cfg").read()

# Find all STM32-style pins referenced in the config
stm_pins = sorted(set(re.findall(r"\bP[A-K]\d{1,2}\b", cfg)))
# Find all SPI bus names
spi_buses = sorted(set(re.findall(r"spi_bus:\s*(\S+)", cfg)))

print("# Auto-generated by writing-plans Task 2.3.")
print("# Maps real-hardware pin/bus/serial names to sim equivalents.")
print()
print("[mcu_main.gpio]")
for i, p in enumerate(stm_pins):
    print(f'{p} = "gpiochip0/gpio{i}"')

print()
print("[mcu_main.spi]")
for i, b in enumerate(spi_buses):
    print(f'{b} = "sim_spi{i}"')

print()
print("[mcu_main.serial]")
print('"usb-Klipper_stm32h723xx_*" = "/tmp/klipper_sim_h7"')
print('"usb-Klipper_stm32f446xx_*" = "/tmp/klipper_sim_f4"')
print('"usb-Beacon_*" = "/tmp/klipper_sim_beacon"')

print()
print("[chip_sockets]")
print('"sim_spi0" = "/tmp/klipper_sim_chip_spi0"')
print('# add per-tmcuart-oid entries here once configure_axes runs')
EOF

cat tools/sim_klippy/pin-overrides.toml | head -20
```

Expected: a populated TOML with ~30+ pin mappings.

- [ ] **Step 2: Write the override-layer test**

```python
# tools/sim_klippy/tests/test_overrides.py
"""Tests for the pin/serial override layer.

Given a vendored printer.cfg, applying the overrides yields a config
where STM32 pin names are replaced with gpiochip0/gpioN equivalents
and the [mcu] serial line points at our sim socket."""
import os
import tempfile
from tools.sim_klippy.orchestrator.overrides import apply_overrides


def test_pin_substitution():
    cfg_in = """
[mcu]
serial: /dev/serial/by-id/usb-Klipper_stm32h723xx_490017000851323235363233-if00

[stepper_x]
step_pin: PG4
dir_pin: !PC1
enable_pin: !PA2

[tmc5160 stepper_x]
cs_pin: PC7
spi_bus: spi1
"""
    overrides = {
        "mcu_main.gpio": {"PG4": "gpiochip0/gpio9", "PC1": "gpiochip0/gpio10",
                          "PA2": "gpiochip0/gpio11", "PC7": "gpiochip0/gpio12"},
        "mcu_main.spi": {"spi1": "sim_spi0"},
        "mcu_main.serial": {
            "usb-Klipper_stm32h723xx_*": "/tmp/klipper_sim_h7",
        },
    }
    out = apply_overrides(cfg_in, overrides)
    assert "PG4" not in out
    assert "gpiochip0/gpio9" in out  # PG4 replaced
    assert "!gpiochip0/gpio10" in out  # !PC1 → !gpiochip0/gpio10 preserves '!'
    assert "spi1" not in out  # bus replaced
    assert "sim_spi0" in out
    assert "/tmp/klipper_sim_h7" in out
```

- [ ] **Step 3: Run the test (fails)**

```bash
python3 -m pytest tools/sim_klippy/tests/test_overrides.py -v 2>&1 | tail
```

Expected: ImportError.

- [ ] **Step 4: Implement overrides.py**

```python
# tools/sim_klippy/orchestrator/overrides.py
"""Pin / SPI bus / serial path override layer.

Reads pin-overrides.toml and applies the mappings to a klippy config
text in-memory so the vendored printer.cfg can stay verbatim. We
operate at the printer.cfg-text level (not at klippy's section/option
parser) because klippy's resolver dispatches to chelper before we get
a hook in — easier to substitute the strings up front."""
import re
from pathlib import Path

try:
    import tomllib  # py 3.11+
except ImportError:
    import tomli as tomllib  # type: ignore


def load_overrides(path: str | Path) -> dict:
    with open(path, "rb") as f:
        return tomllib.load(f)


def apply_overrides(cfg_text: str, overrides: dict) -> str:
    """Substitute real-hardware identifiers with sim equivalents.

    Replaces, in order: STM32 pin names (PG4 → gpiochip0/...), SPI bus
    names (spi1 → sim_spi0), USB serial-by-id substring matches.

    Pin substitution is whole-word (regex \\b boundaries) so we don't
    accidentally rewrite "PA2" inside "PA20" or "spiA1" inside "spiA10".
    """
    out = cfg_text
    gpio_map = overrides.get("mcu_main.gpio", {})
    for real, sim in gpio_map.items():
        out = re.sub(rf"\b{re.escape(real)}\b", sim, out)
    spi_map = overrides.get("mcu_main.spi", {})
    for real, sim in spi_map.items():
        out = re.sub(rf"\b{re.escape(real)}\b", sim, out)
    serial_map = overrides.get("mcu_main.serial", {})
    for pattern, sim in serial_map.items():
        # patterns are glob-ish ("usb-Klipper_stm32h723xx_*"); convert to regex
        regex = re.escape(pattern).replace(r"\*", r"[^\s]*")
        # serial line shape: "/dev/serial/by-id/<pattern>"
        out = re.sub(rf"/dev/serial/by-id/{regex}", sim, out)
    return out
```

- [ ] **Step 5: Run the test**

```bash
python3 -m pytest tools/sim_klippy/tests/test_overrides.py -v 2>&1 | tail
```

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add tools/sim_klippy/orchestrator/overrides.py tools/sim_klippy/pin-overrides.toml tools/sim_klippy/tests/test_overrides.py
git commit -m "sim(orchestrator): pin/serial override layer + auto-gen pin-overrides.toml"
```

---

## Phase 3 — TMC chip emulators

### Task 3.1: TMC5160 emulator with TDD coverage

**Files:**
- Create: `tools/sim_klippy/orchestrator/tmc5160_emulator.py`
- Create: `tools/sim_klippy/tests/test_tmc5160_emulator.py`

- [ ] **Step 1: Write the framing test**

```python
# tools/sim_klippy/tests/test_tmc5160_emulator.py
"""TMC5160 5-byte SPI datagram framer.

Datasheet §5.1: each transfer is 5 bytes. byte0 = R/W bit | reg addr.
Bytes 1-4 = data. Read returns: previous status byte | previous read data
(this means a read needs two transfers; we model that by latching
last_read_data per chip)."""
import pytest
from tools.sim_klippy.orchestrator.tmc5160_emulator import TMC5160Emulator


def test_write_then_read_roundtrip():
    chip = TMC5160Emulator()

    # Write GLOBALSCALER (0x0B) = 128. Bit 7 of byte0 = 1 (write).
    write_req = bytes([0x80 | 0x0B, 0, 0, 0, 128])
    write_reply = chip.transfer(write_req)
    assert len(write_reply) == 5

    # Issue a read of GLOBALSCALER (bit 7 = 0). The first read returns
    # latched stale data; the second returns the actual value.
    chip.transfer(bytes([0x0B, 0, 0, 0, 0]))
    second_reply = chip.transfer(bytes([0x0B, 0, 0, 0, 0]))
    assert second_reply[1:5] == bytes([0, 0, 0, 128])


def test_globalscaler_clamps_low():
    chip = TMC5160Emulator()
    chip.transfer(bytes([0x80 | 0x0B, 0, 0, 0, 10]))  # writes 10 (below 32)
    chip.transfer(bytes([0x0B, 0, 0, 0, 0]))
    reply = chip.transfer(bytes([0x0B, 0, 0, 0, 0]))
    assert reply[4] == 32  # clamped


def test_gstat_clear_on_read():
    chip = TMC5160Emulator()
    chip._registers[0x01] = 0x07  # bits set
    chip.transfer(bytes([0x01, 0, 0, 0, 0]))  # read
    second = chip.transfer(bytes([0x01, 0, 0, 0, 0]))  # read again
    assert second[1:5] == bytes([0, 0, 0, 0])  # cleared


def test_drv_status_sg_result_from_load_hook():
    chip = TMC5160Emulator()
    chip.set_load(120)  # SG_RESULT goes here
    chip.transfer(bytes([0x6F, 0, 0, 0, 0]))
    reply = chip.transfer(bytes([0x6F, 0, 0, 0, 0]))
    sg_result = reply[1] << 8 | reply[2]
    assert sg_result == 120
```

- [ ] **Step 2: Run tests (fail — module missing)**

```bash
python3 -m pytest tools/sim_klippy/tests/test_tmc5160_emulator.py -v 2>&1 | tail
```

Expected: ImportError.

- [ ] **Step 3: Implement TMC5160Emulator**

```python
# tools/sim_klippy/orchestrator/tmc5160_emulator.py
"""Behavioral emulator for the TMC5160 stepper driver chip.

Models register state, side-effects on read/write of specific registers,
and StallGuard load injection so tests can drive the DIAG-trigger path.
Does NOT model coil-current dynamics, microstep tables, or COOLSTEP
beyond what static register reads expose."""
from typing import Callable, Optional

# Register addresses (datasheet §5)
GCONF       = 0x00
GSTAT       = 0x01
IFCNT       = 0x02
SLAVECONF   = 0x03
IOIN        = 0x04
GLOBALSCALER = 0x0B
IHOLD_IRUN  = 0x10
TPOWERDOWN  = 0x11
TSTEP       = 0x12
TPWMTHRS    = 0x13
TCOOLTHRS   = 0x14
THIGH       = 0x15
CHOPCONF    = 0x6C
COOLCONF    = 0x6D
DCCTRL      = 0x6E
DRV_STATUS  = 0x6F
PWMCONF     = 0x70

# Power-on-reset defaults (datasheet §6 + sane init values)
POR_DEFAULTS = {
    GCONF:       0x00000000,
    GSTAT:       0x00000007,  # reset bit set on POR
    IFCNT:       0x00000000,
    SLAVECONF:   0x00000000,
    IOIN:        0x30000000,  # version field
    GLOBALSCALER: 256,         # max scale (treat as = max; clamps clamp DOWN)
    IHOLD_IRUN:  0x00000000,
    TPOWERDOWN:  0x0000000A,
    TSTEP:       0x000FFFFF,   # idle motor → big TSTEP
    CHOPCONF:    0x00410150,
    DRV_STATUS:  0x00000000,
}

# Registers that clear on read
CLEAR_ON_READ = {GSTAT}


class TMC5160Emulator:
    def __init__(self):
        self._registers = dict(POR_DEFAULTS)
        self._last_read_data = 0  # latched: returned on next-read reply
        self._sg_result = 0       # 10-bit StallGuard reading
        self._diag_callback: Optional[Callable[[bool], None]] = None
        self._diag_high = False

    # ─── public hooks for sensorless trigger / tests ────────────────
    def set_load(self, sg_result: int) -> None:
        """Update the 10-bit SG_RESULT visible in DRV_STATUS."""
        self._sg_result = sg_result & 0x03FF

    def set_diag_callback(self, cb: Callable[[bool], None]) -> None:
        """Called whenever DIAG should change level. cb(True) = assert."""
        self._diag_callback = cb

    def maybe_trigger_diag(self, sg_threshold: int) -> None:
        """Compare current SG_RESULT against threshold; assert DIAG if low."""
        should_be_high = self._sg_result < sg_threshold
        if should_be_high != self._diag_high:
            self._diag_high = should_be_high
            if self._diag_callback:
                self._diag_callback(should_be_high)

    # ─── 5-byte datagram dispatch ─────────────────────────────────────
    def transfer(self, req: bytes) -> bytes:
        if len(req) != 5:
            raise ValueError(f"TMC5160 expects 5-byte datagram, got {len(req)}")
        is_write = bool(req[0] & 0x80)
        addr = req[0] & 0x7F
        data = (req[1] << 24) | (req[2] << 16) | (req[3] << 8) | req[4]

        # Reply byte 0 = SPI status (drive-level flags) — leave 0
        if is_write:
            self._do_write(addr, data)
            return bytes([0, 0, 0, 0, 0])
        # Read: reply contains LATCHED previous read; we update latch now
        latched = self._last_read_data
        self._last_read_data = self._do_read(addr)
        return bytes([
            0,
            (latched >> 24) & 0xFF,
            (latched >> 16) & 0xFF,
            (latched >> 8) & 0xFF,
            latched & 0xFF,
        ])

    def _do_write(self, addr: int, value: int) -> None:
        if addr == GLOBALSCALER:
            value = max(32, min(255, value))
        elif addr == IHOLD_IRUN:
            ihold = min(31, value & 0x1F)
            irun = min(31, (value >> 8) & 0x1F)
            iholddelay = (value >> 16) & 0x0F
            value = ihold | (irun << 8) | (iholddelay << 16)
        elif addr == CHOPCONF:
            # Reserved bits 17-19 must be 0
            value &= ~(0x7 << 17)
        self._registers[addr] = value

    def _do_read(self, addr: int) -> int:
        if addr == DRV_STATUS:
            value = self._registers.get(addr, 0) & ~0x03FF
            value |= self._sg_result
            return value
        value = self._registers.get(addr, 0)
        if addr in CLEAR_ON_READ:
            self._registers[addr] = 0
        return value
```

- [ ] **Step 4: Run tests**

```bash
python3 -m pytest tools/sim_klippy/tests/test_tmc5160_emulator.py -v 2>&1 | tail -10
```

Expected: 4 PASS.

- [ ] **Step 5: Commit**

```bash
git add tools/sim_klippy/orchestrator/tmc5160_emulator.py tools/sim_klippy/tests/test_tmc5160_emulator.py
git commit -m "sim(orchestrator): tmc5160 emulator + side-effect tests"
```

### Task 3.2: TMC2209 emulator with TDD coverage

**Files:**
- Create: `tools/sim_klippy/orchestrator/tmc2209_emulator.py`
- Create: `tools/sim_klippy/tests/test_tmc2209_emulator.py`

- [ ] **Step 1: Write tests for the UART framer**

```python
# tools/sim_klippy/tests/test_tmc2209_emulator.py
"""TMC2209 single-wire UART protocol.

Read request:  4 bytes — 0x05 sync, slave_addr, reg_addr, CRC8
Read response: 8 bytes — 0x05 sync, master_addr (0xFF), reg_addr, data×4, CRC8
Write request: 8 bytes — 0x05 sync, slave_addr, reg_addr|0x80, data×4, CRC8
              (no reply)"""
import pytest
from tools.sim_klippy.orchestrator.tmc2209_emulator import (
    TMC2209Emulator, crc8,
)


def test_crc8_matches_klipper_polynomial():
    # Known good vector from datasheet (from a TMC2209 read of GCONF)
    msg = bytes([0x05, 0x00, 0x00])  # sync, slave 0, reg 0 (GCONF read)
    assert crc8(msg) == 0x48


def test_write_then_read_roundtrips_gconf():
    chip = TMC2209Emulator(slave_addr=0)
    write_msg = bytes([0x05, 0x00, 0x00 | 0x80, 0x00, 0x00, 0x00, 0x05])
    write_msg += bytes([crc8(write_msg)])
    reply = chip.handle(write_msg)
    assert reply == b""  # writes have no response

    read_msg = bytes([0x05, 0x00, 0x00])
    read_msg += bytes([crc8(read_msg)])
    reply = chip.handle(read_msg)
    assert len(reply) == 8
    assert reply[0] == 0x05         # sync
    assert reply[1] == 0xFF         # master addr
    assert reply[2] == 0x00         # reg
    assert reply[3:7] == bytes([0, 0, 0, 5])
    assert reply[7] == crc8(reply[:7])


def test_gstat_clears_on_read():
    chip = TMC2209Emulator(slave_addr=0)
    chip._registers[0x01] = 0x07
    read_msg = bytes([0x05, 0x00, 0x01]) + bytes([crc8(bytes([0x05, 0x00, 0x01]))])
    chip.handle(read_msg)  # first read
    reply = chip.handle(read_msg)  # second read
    assert reply[3:7] == bytes([0, 0, 0, 0])  # cleared
```

- [ ] **Step 2: Run tests (fail)**

```bash
python3 -m pytest tools/sim_klippy/tests/test_tmc2209_emulator.py -v 2>&1 | tail
```

- [ ] **Step 3: Implement TMC2209**

```python
# tools/sim_klippy/orchestrator/tmc2209_emulator.py
"""Behavioral emulator for the TMC2209 stepper driver chip.

Single-wire UART, CRC8 polynomial 0x07. Smaller register surface than
the 5160. Same per-chip register dict + clear-on-read for GSTAT."""

CRC8_POLY = 0x07


def crc8(data: bytes) -> int:
    """TMC datasheet CRC8: x^8 + x^2 + x + 1 (polynomial 0x07), init 0,
    process LSB first within each byte."""
    crc = 0
    for byte in data:
        v = byte
        for _ in range(8):
            if (crc >> 7) ^ (v & 1):
                crc = ((crc << 1) ^ CRC8_POLY) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
            v >>= 1
    return crc


# Register addresses (TMC2209 datasheet)
GCONF       = 0x00
GSTAT       = 0x01
IFCNT       = 0x02
IHOLD_IRUN  = 0x10
CHOPCONF    = 0x6C
DRV_STATUS  = 0x6F
PWMCONF     = 0x70

POR_DEFAULTS = {
    GCONF:       0x00000000,
    GSTAT:       0x00000005,  # reset + uv_cp typical fresh boot
    IFCNT:       0x00000000,
    IHOLD_IRUN:  0x00000000,
    CHOPCONF:    0x10000053,
    DRV_STATUS:  0x00000000,
    PWMCONF:     0xC10D0024,
}

CLEAR_ON_READ = {GSTAT}


class TMC2209Emulator:
    def __init__(self, slave_addr: int):
        self._slave = slave_addr
        self._registers = dict(POR_DEFAULTS)

    def handle(self, msg: bytes) -> bytes:
        """Process one inbound UART datagram. Returns reply bytes (8 for
        reads, empty for writes). Raises ValueError on malformed frames."""
        if len(msg) == 4:
            # read request
            if msg[0] != 0x05 or msg[1] != self._slave:
                return b""
            if crc8(msg[:3]) != msg[3]:
                raise ValueError("TMC2209 read CRC mismatch")
            reg = msg[2] & 0x7F
            value = self._registers.get(reg, 0)
            if reg in CLEAR_ON_READ:
                self._registers[reg] = 0
            reply_body = bytes([
                0x05, 0xFF, reg,
                (value >> 24) & 0xFF,
                (value >> 16) & 0xFF,
                (value >> 8) & 0xFF,
                value & 0xFF,
            ])
            return reply_body + bytes([crc8(reply_body)])

        if len(msg) == 8:
            # write request
            if msg[0] != 0x05 or msg[1] != self._slave:
                return b""
            if crc8(msg[:7]) != msg[7]:
                raise ValueError("TMC2209 write CRC mismatch")
            reg = msg[2] & 0x7F
            value = (msg[3] << 24) | (msg[4] << 16) | (msg[5] << 8) | msg[6]
            self._registers[reg] = value
            return b""

        raise ValueError(f"TMC2209 frame must be 4 or 8 bytes, got {len(msg)}")
```

- [ ] **Step 4: Run tests**

```bash
python3 -m pytest tools/sim_klippy/tests/test_tmc2209_emulator.py -v 2>&1 | tail
```

Expected: 3 PASS. **If `test_crc8_matches_klipper_polynomial` fails**, the CRC variant is wrong — Klipper uses LSB-first per the TMC datasheet. Inspect the test vector and adjust `crc8` accordingly until the assertion holds.

- [ ] **Step 5: Commit**

```bash
git add tools/sim_klippy/orchestrator/tmc2209_emulator.py tools/sim_klippy/tests/test_tmc2209_emulator.py
git commit -m "sim(orchestrator): tmc2209 emulator + UART framer tests"
```

### Task 3.3: Wire chip emulators to chip_socket_server

**Files:**
- Modify: `tools/sim_klippy/orchestrator/chip_socket_server.py`
- Create: `tools/sim_klippy/tests/test_chip_emulator_wiring.py`

- [ ] **Step 1: Write integration test**

```python
# tools/sim_klippy/tests/test_chip_emulator_wiring.py
"""End-to-end: spin up a TMC5160 emulator behind a ChipSocketServer,
connect a Unix socket from a test harness, do a write+read round-trip
through the socket, and assert the chip latched the value."""
import os
import socket
import time
import threading
from tools.sim_klippy.orchestrator.chip_socket_server import ChipSocketServer
from tools.sim_klippy.orchestrator.tmc5160_emulator import TMC5160Emulator


def test_tmc5160_via_socket():
    sock_path = "/tmp/test_tmc5160_via_socket"
    if os.path.exists(sock_path):
        os.unlink(sock_path)

    chip = TMC5160Emulator()
    server = ChipSocketServer(sock_path, chip.transfer, chunk=5)
    server.start()
    try:
        for _ in range(50):
            if os.path.exists(sock_path):
                break
            time.sleep(0.01)
        client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        client.connect(sock_path)
        # Write GLOBALSCALER (0x0B) = 200
        client.sendall(bytes([0x80 | 0x0B, 0, 0, 0, 200]))
        client.recv(5)
        # Read twice (latched semantics); second read = the written value
        client.sendall(bytes([0x0B, 0, 0, 0, 0]))
        client.recv(5)
        client.sendall(bytes([0x0B, 0, 0, 0, 0]))
        reply = client.recv(5)
        assert reply[4] == 200
    finally:
        server.stop()
        if os.path.exists(sock_path):
            os.unlink(sock_path)
```

- [ ] **Step 2: Run the test**

```bash
python3 -m pytest tools/sim_klippy/tests/test_chip_emulator_wiring.py -v 2>&1 | tail
```

Expected: PASS. The chunk=5 framing matches TMC5160's 5-byte datagram size; the chip transfer method takes/returns bytes that the server passes through unchanged.

- [ ] **Step 3: Commit**

```bash
git add tools/sim_klippy/tests/test_chip_emulator_wiring.py
git commit -m "sim(orchestrator): integration — TMC5160 emulator behind chip_socket_server"
```

---

## Phase 4 — Beacon serial stub

### Task 4.1: Beacon protocol stub

**Files:**
- Create: `tools/sim_klippy/orchestrator/beacon_serial_stub.py`
- Create: `tools/sim_klippy/tests/test_beacon_serial_stub.py`

- [ ] **Step 1: Inspect the beacon plugin's wire format**

Run: `grep -nE 'msgproto\.|msg_format\|register_response\|sample\|_read_response\|nvm\|frame' tools/sim_klippy/printer_real/third_party_repos/beacon_klipper/beacon.py | head -30`

Identify: the request/response message names beacon expects (typically ASCII command + response format compatible with klippy's msgproto). The stub needs to respond to `BEACON_VERSION`, `BEACON_NVM_LOAD`, and emit periodic `beacon_data` samples.

- [ ] **Step 2: Write the stub-handshake test**

```python
# tools/sim_klippy/tests/test_beacon_serial_stub.py
"""Beacon stub: handshake (firmware version, NVM read) + continuous Z
sample stream. The stub speaks the beacon-msgproto wire format on a
PTY at /tmp/klipper_sim_beacon, and the test driver reads back through
a regular pty to validate the response shape."""
import os, pty, select, subprocess, time
import pytest
from tools.sim_klippy.orchestrator.beacon_serial_stub import BeaconSerialStub


def test_beacon_responds_to_version_query():
    pty_path = "/tmp/test_beacon_serial_stub"
    if os.path.exists(pty_path):
        os.unlink(pty_path)
    stub = BeaconSerialStub(pty_path)
    stub.start()
    try:
        # Wait for the stub to bind
        for _ in range(50):
            if os.path.exists(pty_path):
                break
            time.sleep(0.01)
        # Open the PTY and request a version. Frame format: per
        # beacon.py's `_read_response`, line-terminated ASCII.
        with open(pty_path, "rb+", buffering=0) as f:
            f.write(b"BEACON_GET_VERSION\n")
            time.sleep(0.1)
            response = f.read1(256)
            assert b"version=" in response.lower() or b"BEACON_VERSION_RESPONSE" in response
    finally:
        stub.stop()
        if os.path.exists(pty_path):
            os.unlink(pty_path)


def test_beacon_streams_periodic_samples():
    pty_path = "/tmp/test_beacon_samples"
    if os.path.exists(pty_path):
        os.unlink(pty_path)
    stub = BeaconSerialStub(pty_path)
    stub.start_sample_stream(z_target_mm=10.0, rate_hz=200)
    try:
        for _ in range(50):
            if os.path.exists(pty_path):
                break
            time.sleep(0.01)
        with open(pty_path, "rb", buffering=0) as f:
            time.sleep(0.5)  # let stream emit
            data = f.read1(4096)
            samples = data.count(b"beacon_data")
            assert samples >= 50, f"expected ~100 samples in 500ms, got {samples}"
    finally:
        stub.stop()
        if os.path.exists(pty_path):
            os.unlink(pty_path)
```

- [ ] **Step 3: Run (fails)**

- [ ] **Step 4: Implement the stub**

The stub creates a PTY (using `pty.openpty`), symlinks the slave path to `pty_path`, and runs an asyncio/threaded loop that:

1. Reads inbound ASCII command lines.
2. Dispatches to a handler:
   - `BEACON_GET_VERSION` → `BEACON_VERSION_RESPONSE version=2.0.0\n`
   - `BEACON_NVM_LOAD` → returns vendored NVM blob (constant byte string from `printer_real/third_party_repos/beacon_klipper/firmware/revh.dfu` derivable factory data; for the stub, hand-construct a plausible one)
   - `BEACON_QUERY_PROBE` → `BEACON_PROBE_RESULT z=<current>\n`
3. Optionally streams `beacon_data z=<current> count=<i>\n` lines at the configured rate.

```python
# tools/sim_klippy/orchestrator/beacon_serial_stub.py
"""PTY-backed Beacon protocol stub.

Beacon's plugin (vendored at
tools/sim_klippy/printer_real/third_party_repos/beacon_klipper/beacon.py)
opens a USB-CDC pty and exchanges line-oriented commands. The real
beacon firmware is closed-source, but the wire surface visible to
klippy is small; we model the handful of commands the plugin issues
during init + bed-mesh + contact-probe.

Z reading model: stub tracks an externally-driven `z_target_mm` (set
by the orchestrator from the modeled toolhead position) and samples
that with µm-scale sinusoidal noise."""
import math
import os
import pty
import select
import threading
import time


class BeaconSerialStub:
    def __init__(self, pty_path: str):
        self._pty_path = pty_path
        self._z_target = 10.0
        self._z_noise_amp = 0.005  # 5 µm
        self._stream_rate = 0.0
        self._stop = threading.Event()
        self._thread = None
        self._master_fd = None
        self._t0 = time.monotonic()

    def set_z(self, z_mm: float) -> None:
        self._z_target = z_mm

    def start(self) -> None:
        master_fd, slave_fd = pty.openpty()
        slave_name = os.ttyname(slave_fd)
        try:
            os.unlink(self._pty_path)
        except FileNotFoundError:
            pass
        os.symlink(slave_name, self._pty_path)
        os.close(slave_fd)
        self._master_fd = master_fd
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def start_sample_stream(self, z_target_mm: float, rate_hz: float) -> None:
        self._z_target = z_target_mm
        self._stream_rate = rate_hz
        if self._thread is None:
            self.start()

    def stop(self) -> None:
        self._stop.set()
        if self._master_fd is not None:
            try:
                os.close(self._master_fd)
            except OSError:
                pass
        if self._thread:
            self._thread.join(timeout=1.0)
        try:
            os.unlink(self._pty_path)
        except FileNotFoundError:
            pass

    def _z_sample(self) -> float:
        elapsed = time.monotonic() - self._t0
        return self._z_target + self._z_noise_amp * math.sin(elapsed * 60.0)

    def _loop(self):
        sample_count = 0
        last_sample_time = 0.0
        recv_buf = b""
        while not self._stop.is_set():
            r, _, _ = select.select([self._master_fd], [], [], 0.005)
            if r:
                try:
                    chunk = os.read(self._master_fd, 256)
                except OSError:
                    break
                recv_buf += chunk
                while b"\n" in recv_buf:
                    line, recv_buf = recv_buf.split(b"\n", 1)
                    self._handle_command(line.strip())
            now = time.monotonic()
            if self._stream_rate > 0 and now - last_sample_time >= 1.0 / self._stream_rate:
                last_sample_time = now
                z = self._z_sample()
                msg = f"beacon_data z={z:.4f} count={sample_count}\n".encode()
                try:
                    os.write(self._master_fd, msg)
                except OSError:
                    break
                sample_count += 1

    def _handle_command(self, line: bytes):
        if not line:
            return
        cmd = line.decode("ascii", errors="replace").upper()
        if cmd == "BEACON_GET_VERSION":
            os.write(self._master_fd, b"BEACON_VERSION_RESPONSE version=2.0.0\n")
        elif cmd == "BEACON_NVM_LOAD":
            blob = "BEACON_NVM_RESPONSE serial=SIMSTUB000 fmin=5000000 amfg=1.0\n"
            os.write(self._master_fd, blob.encode())
        elif cmd.startswith("BEACON_QUERY_PROBE"):
            z = self._z_sample()
            os.write(self._master_fd,
                     f"BEACON_PROBE_RESULT z={z:.4f}\n".encode())
```

- [ ] **Step 5: Run tests**

```bash
python3 -m pytest tools/sim_klippy/tests/test_beacon_serial_stub.py -v 2>&1 | tail
```

Expected: 2 PASS. The exact protocol shape may need tuning to match what beacon.py expects — when the integration test (Phase 7) runs, watch for "Unable to communicate with beacon" errors and expand the stub's command repertoire to match what `compat_kin_note_z_not_homed` and `cmd_BEACON_AUTO_CALIBRATE` actually send. Adding new command branches to `_handle_command` is straightforward.

- [ ] **Step 6: Commit**

```bash
git add tools/sim_klippy/orchestrator/beacon_serial_stub.py tools/sim_klippy/tests/test_beacon_serial_stub.py
git commit -m "sim(orchestrator): beacon serial stub — version/nvm/probe + sample stream"
```

---

## Phase 5 — ADC / heater stubs

### Task 5.1: Add `analog_set_simulated_value` to Linux MACH_LINUX analog.c

**Files:**
- Modify: `src/linux/analog.c`

- [ ] **Step 1: Read existing analog.c shape**

Run: `cat src/linux/analog.c | head -60 && echo --- && wc -l src/linux/analog.c`

Identify the value-providing entry point (typically a static `analog_value` array indexed by adc pin).

- [ ] **Step 2: Add the simulated-value setter**

Append to `src/linux/analog.c`:

```c
// Sim-only: orchestrator-driven simulated ADC values for thermistors and
// heater feedback. Indexed by adc_pin number (linux/analog uses one
// adc pin per ADC channel). Set by command_runtime_sim_adc_set in
// src/runtime_sim_commands.c.
#define MAX_SIM_ADC 32
static uint16_t simulated_value[MAX_SIM_ADC];

void
analog_set_simulated_value(uint8_t adc_pin, uint16_t value)
{
    if (adc_pin < MAX_SIM_ADC)
        simulated_value[adc_pin] = value;
}

uint16_t
analog_get_simulated_value(uint8_t adc_pin)
{
    if (adc_pin < MAX_SIM_ADC)
        return simulated_value[adc_pin];
    return 0;
}
```

Find the existing `gpio_adc_sample` (or equivalent reader). If it returns a real reading, intercept it for sim:

```c
struct gpio_adc gpio_adc_setup(uint32_t pin) { ... }

uint32_t
gpio_adc_sample(struct gpio_adc g)
{
    /* if simulated_value is set for this pin, return it */
    extern uint16_t analog_get_simulated_value(uint8_t adc_pin);
    uint16_t sim = analog_get_simulated_value(g.fd);  // fd or pin idx
    if (sim) return sim;
    /* ... real read code unchanged ... */
}
```

(The exact splice depends on the existing structure of the file — adjust to match. The point: sim values bypass the real read.)

- [ ] **Step 3: Verify compile**

```bash
cp tools/sim_klippy/configs/h7-sim.config .config
make 2>&1 | grep -E 'analog\.c|error' | head
```

- [ ] **Step 4: Commit**

```bash
git add src/linux/analog.c
git commit -m "sim(linux): analog — simulated ADC value table for orchestrator drive"
```

### Task 5.2: Python-side ADC stub with heater feedback

**Files:**
- Create: `tools/sim_klippy/orchestrator/adc_stub.py`
- Create: `tools/sim_klippy/tests/test_adc_stub.py`

- [ ] **Step 1: Test the temp-tracking model**

```python
# tools/sim_klippy/tests/test_adc_stub.py
import math
from tools.sim_klippy.orchestrator.adc_stub import HeaterModel


def test_bed_ramps_toward_target():
    h = HeaterModel(initial_temp_c=25, ramp_rate_c_per_s=0.5)
    h.set_target(60)
    for _ in range(60):
        h.step(dt_s=1.0)
    assert h.temp_c == pytest.approx(55.0, abs=2.0)


def test_temperature_to_adc_thermistor_curve():
    """Sanity check: 25°C maps to ~half-scale on a 10kΩ NTC with 4700Ω
    pull-up, and 200°C maps low. We don't model the curve precisely —
    just monotone."""
    from tools.sim_klippy.orchestrator.adc_stub import temp_to_adc
    assert temp_to_adc(25) > temp_to_adc(200)
    assert temp_to_adc(0) > temp_to_adc(100)
```

```python
import pytest
```

(Add the import at the top.)

- [ ] **Step 2: Run (fails)**

- [ ] **Step 3: Implement HeaterModel + temp_to_adc**

```python
# tools/sim_klippy/orchestrator/adc_stub.py
"""Simulated ADC + heater feedback.

For each thermistor we maintain a `HeaterModel` that ramps its
temperature toward a target at a configured rate. The orchestrator
calls `step(dt)` periodically and pushes the resulting temperature
through `temp_to_adc()` then into the firmware via
`runtime_sim_adc_set`.

For the success bar (boot + G28 + small print), modeling is crude —
no real thermal mass, no PID overshoot. Heater PWM writes are
ignored; we ramp directly toward the user-set target."""

class HeaterModel:
    def __init__(self, initial_temp_c: float, ramp_rate_c_per_s: float):
        self.temp_c = initial_temp_c
        self.target_c = initial_temp_c
        self._rate = ramp_rate_c_per_s

    def set_target(self, target_c: float) -> None:
        self.target_c = target_c

    def step(self, dt_s: float) -> None:
        delta = self.target_c - self.temp_c
        max_step = self._rate * dt_s
        if abs(delta) <= max_step:
            self.temp_c = self.target_c
        else:
            self.temp_c += max_step if delta > 0 else -max_step


# 10 kΩ NTC thermistor + 4700 Ω pull-up to 3.3V → ADC, 12-bit (0-4095)
def temp_to_adc(temp_c: float) -> int:
    # Beta-model NTC: R = R0 * exp(B * (1/T - 1/T0))
    B = 3950
    T0 = 298.15  # 25°C in K
    R0 = 10000
    T = temp_c + 273.15
    R = R0 * math.exp(B * (1.0 / T - 1.0 / T0))
    pull_up = 4700.0
    v_adc = 3.3 * R / (R + pull_up)
    adc = int(v_adc / 3.3 * 4095)
    return max(0, min(4095, adc))


import math  # imported above before use; keep at module top in real code
```

(Move the `import math` to the top of the file — placed at bottom in the inline view here for brevity.)

- [ ] **Step 4: Run tests**

```bash
python3 -m pytest tools/sim_klippy/tests/test_adc_stub.py -v 2>&1 | tail
```

Expected: 2 PASS.

- [ ] **Step 5: Commit**

```bash
git add tools/sim_klippy/orchestrator/adc_stub.py tools/sim_klippy/tests/test_adc_stub.py
git commit -m "sim(orchestrator): adc_stub — HeaterModel + thermistor curve"
```

---

## Phase 6 — Sensorless trigger

### Task 6.1: Position tracker + DIAG dance

**Files:**
- Create: `tools/sim_klippy/orchestrator/sensorless_trigger.py`
- Create: `tools/sim_klippy/sim_geometry.toml`
- Create: `tools/sim_klippy/tests/test_sensorless_trigger.py`

- [ ] **Step 1: Write sim_geometry.toml**

```toml
# tools/sim_klippy/sim_geometry.toml
# Per-axis virtual wall positions for the sensorless homing model.
# Walls fire DIAG when the modeled motor position reaches the wall;
# defaults derive from printer.cfg `position_min` / `position_max`.

[stepper_x]
homing_endstop_mm = 300.0
sg_threshold = 80

[stepper_y]
homing_endstop_mm = 300.0
sg_threshold = 80

[stepper_x1]
homing_endstop_mm = 300.0
sg_threshold = 80

[stepper_y1]
homing_endstop_mm = 300.0
sg_threshold = 80
```

- [ ] **Step 2: Write trigger test**

```python
# tools/sim_klippy/tests/test_sensorless_trigger.py
"""SensorlessTrigger: tracks step count → mm position → SG_RESULT,
fires the chip's diag callback when SG drops below threshold."""
from tools.sim_klippy.orchestrator.sensorless_trigger import SensorlessTrigger
from tools.sim_klippy.orchestrator.tmc5160_emulator import TMC5160Emulator


class FakeStepCounter:
    def __init__(self):
        self.count = 0

    def get(self):
        return self.count


def test_diag_asserts_at_endstop():
    chip = TMC5160Emulator()
    fired = []
    chip.set_diag_callback(lambda high: fired.append(high))

    counter = FakeStepCounter()
    trigger = SensorlessTrigger(
        chip=chip,
        step_counter=counter.get,
        rotation_distance_mm=40.0,
        steps_per_rotation=200 * 16,  # 16 microsteps
        endstop_mm=300.0,
        sg_threshold=80,
        homing_direction=1,  # toward larger steps
    )

    # Far from wall → SG_RESULT high → DIAG low
    counter.count = 0
    trigger.tick()
    assert chip._diag_high is False

    # At wall (300mm * 200 * 16 / 40 = 24000 steps)
    counter.count = 24000
    trigger.tick()
    assert chip._diag_high is True
    assert fired[-1] is True
```

- [ ] **Step 3: Implement SensorlessTrigger**

```python
# tools/sim_klippy/orchestrator/sensorless_trigger.py
"""Models sensorless-homing StallGuard triggering for one stepper.

Polls a step-count source (firmware FFI or sim shim), converts to mm
position via rotation_distance / microsteps, computes a synthetic
SG_RESULT that decreases as the position approaches the wall, and
asks the chip emulator to assert/clear DIAG via its callback when
SG_RESULT crosses a threshold."""
from typing import Callable


class SensorlessTrigger:
    def __init__(
        self,
        chip,
        step_counter: Callable[[], int],
        rotation_distance_mm: float,
        steps_per_rotation: int,
        endstop_mm: float,
        sg_threshold: int,
        homing_direction: int,
    ):
        self._chip = chip
        self._step_counter = step_counter
        self._mm_per_step = rotation_distance_mm / steps_per_rotation
        self._endstop_mm = endstop_mm
        self._sg_threshold = sg_threshold
        self._direction = homing_direction
        self._initial_count = step_counter()

    def tick(self) -> None:
        delta_steps = self._step_counter() - self._initial_count
        position_mm = delta_steps * self._mm_per_step * self._direction
        distance_to_wall = self._endstop_mm - position_mm
        # Linear model: SG_RESULT = max(0, distance * 50)
        sg = max(0, min(1023, int(distance_to_wall * 50)))
        self._chip.set_load(sg)
        self._chip.maybe_trigger_diag(self._sg_threshold)
```

- [ ] **Step 4: Run tests**

```bash
python3 -m pytest tools/sim_klippy/tests/test_sensorless_trigger.py -v 2>&1 | tail
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add tools/sim_klippy/orchestrator/sensorless_trigger.py tools/sim_klippy/sim_geometry.toml tools/sim_klippy/tests/test_sensorless_trigger.py
git commit -m "sim(orchestrator): sensorless trigger — step-count-driven DIAG"
```

---

## Phase 7 — End-to-end pytest fixtures + the three success-bar tests

### Task 7.1: pytest conftest.py — orchestrator fixture

**Files:**
- Create: `tools/sim_klippy/conftest.py`

- [ ] **Step 1: Write the conftest**

```python
# tools/sim_klippy/conftest.py
"""Shared pytest fixtures for the faithful-sim test suite.

The `sim` fixture brings up everything: two klipper.elf MCUs, the
chip emulators behind chip_socket_servers, the beacon stub, the ADC
stub, and a klippy instance attached to both MCUs via the motion
bridge. Yields a SimContext that tests use to drive gcode and assert
state."""
import dataclasses
import os
import pathlib
import shutil
import subprocess
import time
from typing import List

import pytest

from tools.sim_klippy.orchestrator.launcher import spawn_mcus, McuHandles
from tools.sim_klippy.orchestrator.chip_socket_server import ChipSocketServer
from tools.sim_klippy.orchestrator.tmc5160_emulator import TMC5160Emulator
from tools.sim_klippy.orchestrator.tmc2209_emulator import TMC2209Emulator
from tools.sim_klippy.orchestrator.beacon_serial_stub import BeaconSerialStub
from tools.sim_klippy.orchestrator.overrides import (
    apply_overrides, load_overrides,
)


@dataclasses.dataclass
class SimContext:
    mcus: McuHandles
    chip_servers: list
    beacon: BeaconSerialStub
    klippy_proc: subprocess.Popen
    klippy_log: pathlib.Path
    api_socket: str

    def gcode(self, script: str) -> None:
        # Use the existing api_server.py wrapper from tools/sim_klippy/run.py
        from tools.sim_klippy import run  # type: ignore
        run.send_gcode(self.api_socket, script)


@pytest.fixture
def sim(tmp_path):
    repo_root = pathlib.Path(__file__).resolve().parents[2]
    log_dir = tmp_path / "logs"
    log_dir.mkdir()

    # 1) Spawn both MCUs
    mcus = spawn_mcus(
        h7_elf=str(repo_root / "out" / "klipper-h7-sim.elf"),
        f4_elf=str(repo_root / "out" / "klipper-f4-sim.elf"),
        h7_socket="/tmp/klipper_sim_h7",
        f4_socket="/tmp/klipper_sim_f4",
        log_dir=str(log_dir),
    )

    # 2) Start chip emulators behind socket servers
    chip_servers: List[ChipSocketServer] = []
    # 4 × TMC5160 on H7 SPI bus
    tmc5160s = [TMC5160Emulator() for _ in range(4)]
    for i, chip in enumerate(tmc5160s):
        path = f"/tmp/klipper_sim_chip_spi{i}"
        srv = ChipSocketServer(path, chip.transfer, chunk=5)
        srv.start()
        chip_servers.append(srv)
    # 3 × TMC2209 on F4 UART
    tmc2209s = [TMC2209Emulator(slave_addr=i) for i in range(3)]
    for i, chip in enumerate(tmc2209s):
        path = f"/tmp/klipper_sim_chip_uart{i}"
        srv = ChipSocketServer(path, chip.handle, chunk=8)
        srv.start()
        chip_servers.append(srv)

    # 3) Beacon stub
    beacon = BeaconSerialStub("/tmp/klipper_sim_beacon")
    beacon.start_sample_stream(z_target_mm=10.0, rate_hz=200)

    # 4) Apply pin overrides + write rendered printer.cfg under tmp
    overrides = load_overrides(repo_root / "tools" / "sim_klippy" / "pin-overrides.toml")
    cfg_in = (repo_root / "tools" / "sim_klippy" / "printer_real" /
              "config" / "printer.cfg").read_text()
    cfg_out = apply_overrides(cfg_in, overrides)
    rendered_cfg = tmp_path / "printer.cfg"
    rendered_cfg.write_text(cfg_out)
    # Also stage all the .cfg includes (verbatim)
    cfg_dir = repo_root / "tools" / "sim_klippy" / "printer_real" / "config"
    for f in cfg_dir.iterdir():
        if f.is_file() and f.name != "printer.cfg":
            shutil.copy(f, tmp_path / f.name)
    # Symlink KAMP into tmp_path
    (tmp_path / "KAMP").symlink_to(
        repo_root / "tools" / "sim_klippy" / "printer_real" /
        "third_party_repos" / "Klipper-Adaptive-Meshing-Purging" / "Configuration"
    )

    # 5) Spawn klippy with extras path that includes vendored third-party
    klippy_log = log_dir / "klippy.log"
    api_socket = str(tmp_path / "klippy.sock")
    extras_pythonpath = str(
        repo_root / "tools" / "sim_klippy" / "printer_real" /
        "third_party_repos" / "beacon_klipper"
    )
    env = os.environ.copy()
    env["PYTHONPATH"] = (
        extras_pythonpath
        + ":" + str(repo_root / "tools" / "sim_klippy" / "printer_real" /
                    "third_party_repos" / "motors-sync")
        + ":" + env.get("PYTHONPATH", "")
    )
    klippy = subprocess.Popen(
        ["python3", str(repo_root / "klippy" / "klippy.py"),
         str(rendered_cfg),
         "-l", str(klippy_log),
         "-a", api_socket],
        env=env,
        stdout=open(log_dir / "klippy.stdout", "wb"),
        stderr=subprocess.STDOUT,
    )

    # 6) Wait for "Printer is ready" in the log (or timeout)
    deadline = time.monotonic() + 30.0
    while time.monotonic() < deadline:
        if klippy_log.exists() and b"Printer is ready" in klippy_log.read_bytes():
            break
        if klippy.poll() is not None:
            raise RuntimeError(
                f"klippy exited before ready (rc={klippy.returncode}); "
                f"see {klippy_log}"
            )
        time.sleep(0.1)
    else:
        klippy.kill()
        raise RuntimeError(f"klippy did not reach ready in 30s; see {klippy_log}")

    ctx = SimContext(
        mcus=mcus,
        chip_servers=chip_servers,
        beacon=beacon,
        klippy_proc=klippy,
        klippy_log=klippy_log,
        api_socket=api_socket,
    )

    try:
        yield ctx
    finally:
        klippy.terminate()
        try:
            klippy.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            klippy.kill()
        for srv in chip_servers:
            srv.stop()
        beacon.stop()
        mcus.shutdown()
```

- [ ] **Step 2: Commit (no test runs yet — driven by the success-bar tests next)**

```bash
git add tools/sim_klippy/conftest.py
git commit -m "sim(test): conftest — sim fixture brings up the whole stack"
```

### Task 7.2: test_boot.py — success bar 1

**Files:**
- Create: `tools/sim_klippy/tests/test_boot.py`

- [ ] **Step 1: Write the test**

```python
# tools/sim_klippy/tests/test_boot.py
"""Boot test: sim fixture brings up both MCUs, beacon, all chip stubs;
klippy connects, registers all extras, reaches 'Printer is ready'.

Failure modes: any klippy traceback, any MCU shutdown, any
TMC_UNKNOWN_REG warning."""
import pytest


def test_boot_clean(sim):
    log = sim.klippy_log.read_text()
    assert "Printer is ready" in log
    # No tracebacks
    assert "Traceback" not in log, f"klippy crashed during boot:\n{log}"
    # No MCU shutdowns
    assert "MCU '" not in log or " shutdown:" not in log, \
        f"MCU shutdown during boot:\n{log}"
    # No transport timeouts
    assert "transport closed" not in log
    assert "transport timed out" not in log
    # No TMC unknown-register warnings (drift detection)
    assert "TMC_UNKNOWN_REG" not in log
```

- [ ] **Step 2: Run the test**

```bash
docker run --rm -v $(pwd):/work -w /work kalico-sim \
    pytest tools/sim_klippy/tests/test_boot.py -v 2>&1 | tail -30
```

Expected initially: many things will fail. Iterate by reading the klippy log under the test's tmp_path, identifying the first failure (likely "config error" or "Unable to connect MCU"), and fixing the orchestrator/conftest until the boot-to-ready path completes.

- [ ] **Step 3: Iterate until green**

Common issues and fixes:
- **"Unable to enumerate spi bus"** — pin-overrides.toml needs more entries, or `runtime_sim_route_spi` isn't being called. Add a step in `conftest.py`'s setup that issues `runtime_sim_route_spi` over the H7 socket for each TMC5160's bus before klippy attaches.
- **"Beacon: connection refused"** — beacon stub PTY isn't ready before klippy tries to open it. Add a `time.sleep(0.5)` after `beacon.start_sample_stream` in conftest, or wait for the symlink to exist.
- **"thermistor reading out of range"** — initial ADC values aren't seeded; in conftest, push initial 25°C readings through `runtime_sim_adc_set` for every thermistor pin before klippy connects.
- **"config error: section 'beacon' not found"** — PYTHONPATH doesn't include the beacon plugin path.

- [ ] **Step 4: Commit when green**

```bash
git add tools/sim_klippy/tests/test_boot.py
git commit -m "sim(test): test_boot — success bar 1, klippy reaches ready"
```

### Task 7.3: test_g28_full.py — success bar 2

**Files:**
- Create: `tools/sim_klippy/tests/test_g28_full.py`

- [ ] **Step 1: Write the test**

```python
# tools/sim_klippy/tests/test_g28_full.py
"""Success bar 2: G28 X / Y / Z / all axes; M84 clears."""


def test_g28_x_homes(sim):
    sim.gcode("G28 X")
    log = sim.klippy_log.read_text()
    # Reach G28 success without an Internal error
    assert "Internal error on command" not in log[-2000:], \
        f"G28 X failed:\n{log[-2000:]}"
    assert "homed_axes" in log  # via toolhead status snapshot


def test_g28_full_then_m84(sim):
    sim.gcode("G28")
    sim.gcode("M84")
    log = sim.klippy_log.read_text()
    assert "Internal error on command" not in log[-3000:], \
        f"G28 or M84 failed:\n{log[-3000:]}"
```

- [ ] **Step 2: Iterate to green**

This is where most of the unknown-unknown work lands — beacon's `_maybe_zhop`, the SPI dance during current change, sensorless DIAG firing. Read the klippy log on each failure and extend the relevant stub. Each fix should be a separate small commit so we can attribute failures back to specific stub limitations.

- [ ] **Step 3: Commit when green**

```bash
git add tools/sim_klippy/tests/test_g28_full.py
git commit -m "sim(test): test_g28_full — success bar 2, all axes home"
```

### Task 7.4: test_small_print.py — success bar 3

**Files:**
- Create: `tools/sim_klippy/fixtures/small_print.gcode`
- Create: `tools/sim_klippy/tests/test_small_print.py`

- [ ] **Step 1: Write the gcode fixture**

```gcode
; tools/sim_klippy/fixtures/small_print.gcode
; 30-line slicer-style chunk for sim's success-bar 3 test.
M140 S60         ; bed target
M104 S200        ; hotend target
G28              ; home
M190 S60         ; wait bed
M109 S200        ; wait hotend
BED_MESH_CALIBRATE ADAPTIVE=1
G1 X10 Y10 Z0.2 F3000
G1 X100 E5 F1500
G1 Y20 E0.5 F1500
G1 X10 E5 F1500
G1 Y10 E0.5 F1500
G1 X20 E0.5 F1500
G1 X90 E4 F2000
G1 Y15 E0.4 F2000
G1 X20 E4 F2000
G1 Y10 E0.4 F2000
G91
G1 E-2 F1800
G90
M104 S0
M140 S0
M84
```

- [ ] **Step 2: Write the test**

```python
# tools/sim_klippy/tests/test_small_print.py
"""Success bar 3: 30-line print fixture executes end-to-end."""
import pathlib


def test_small_print(sim):
    fixture = (pathlib.Path(__file__).parents[1] / "fixtures" /
               "small_print.gcode").read_text()
    for line in fixture.splitlines():
        line = line.strip()
        if not line or line.startswith(";"):
            continue
        sim.gcode(line)
    log = sim.klippy_log.read_text()
    assert "Internal error on command" not in log[-5000:], \
        f"print failed mid-stream:\n{log[-5000:]}"
    # Bed mesh produced points
    assert "Mesh Bed Leveling Complete" in log or "bed_mesh" in log
```

- [ ] **Step 3: Iterate to green, expanding stubs as needed**

Likely-broken paths:
- `BED_MESH_CALIBRATE` — beacon stub needs to handle bed-mesh probe sequences (multiple BEACON_QUERY_PROBE calls at scripted XY positions). Extend beacon stub to track virtual XY and respond accordingly.
- Heater wait — adc_stub's HeaterModel must actually push values through `runtime_sim_adc_set` periodically. Add a stepping thread in conftest that runs `step(dt)` + ADC push every ~100ms.
- E-axis moves — extruder stepper's TMC2209 needs proper init responses; verify `bottom`-side TMC2209 emulators wired correctly.

- [ ] **Step 4: Commit when green**

```bash
git add tools/sim_klippy/fixtures/small_print.gcode tools/sim_klippy/tests/test_small_print.py
git commit -m "sim(test): test_small_print — success bar 3, 30-line print"
```

---

## Phase 8 — Wiring + final integration

### Task 8.1: Add `sim-test` make target

**Files:**
- Modify: `Makefile.kalico`
- Modify: `tools/sim_klippy/run_local.sh`

- [ ] **Step 1: Add `sim-test` target**

Append to `Makefile.kalico`:

```makefile
.PHONY: sim-test
sim-test:
	# Build both ELFs first
	cp tools/sim_klippy/configs/h7-sim.config .config && $(MAKE) clean && $(MAKE) -j4 && cp out/klipper.elf out/klipper-h7-sim.elf
	cp tools/sim_klippy/configs/f4-sim.config .config && $(MAKE) clean && $(MAKE) -j4 && cp out/klipper.elf out/klipper-f4-sim.elf
	# Then run all sim tests
	docker run --rm -v $(PWD):/work -w /work kalico-sim \
		pytest tools/sim_klippy/tests/ -v
```

- [ ] **Step 2: Verify the target**

```bash
make -f Makefile.kalico sim-test 2>&1 | tail -30
```

Expected: all three success-bar tests run; pass.

- [ ] **Step 3: Commit**

```bash
git add Makefile.kalico
git commit -m "sim: make sim-test target — full bring-up + success-bar tests"
```

### Task 8.2: Plan-changes-log entry

**Files:**
- Modify: `docs/superpowers/plan-changes-log.md`

- [ ] **Step 1: Add entry**

Prepend to the entries section:

```markdown
### 2026-05-08 — Faithful klippy-in-loop sim

**What:** Built a klippy-in-loop simulator that runs the user's actual
printer.cfg + third-party plugins (beacon, motors-sync, KAMP, …) end-to-end
against two simulated MCUs (H7 with kalico runtime, F446 without) backed by
behavioral TMC5160/TMC2209 emulators, a beacon serial stub, an ADC/heater
model, and a step-count-driven sensorless StallGuard trigger. Three pytest
tests gate on success bar 1 (boot to ready), 2 (G28 all axes), and 3
(30-line slicer print). Lives under `tools/sim_klippy/`.

**Why:** Two recent printer-bricking regressions (the `clear_homing_state("z")`
mismatch from beacon's compat layer, the `clocksync.is_active()` bridge-mode
stale-counter cascade) sailed past every existing test and got caught only
on hardware. The synthetic single-MCU sim covered motion-bridge / runtime
paths but never loaded the user's real printer.cfg or third-party plugins,
so any host-side klippy/extras × motion-bridge integration regression had
no test coverage. The faithful sim runs the same code paths the printer
runs, so this class of bug surfaces locally.

**Evidence:** `docs/superpowers/specs/2026-05-08-faithful-klippy-sim-design.md`,
`docs/superpowers/plans/2026-05-08-faithful-klippy-sim.md`,
`tools/sim_klippy/printer_real/` (vendored snapshot),
`tools/sim_klippy/orchestrator/` + `tools/sim_klippy/tests/` (sim + tests).
```

- [ ] **Step 2: Commit**

```bash
git add docs/superpowers/plan-changes-log.md
git commit -m "docs: plan-changes-log entry for faithful sim"
```

---

## Self-review pass

Before declaring this plan finished, walk through it once with fresh eyes:

- [ ] **Spec coverage check**: each section in `docs/superpowers/specs/2026-05-08-faithful-klippy-sim-design.md` should map to at least one task. Spot-check sections 1–9.
- [ ] **Placeholder scan**: search for "TBD", "TODO", "implement later", "similar to". Fix any.
- [ ] **Type consistency**: chip emulators, server, sensorless trigger, fixture wiring all use the same names (`transfer`, `handle`, `set_load`, `maybe_trigger_diag`, `set_diag_callback`).
- [ ] **Frontmatter**: Goal / Architecture / Tech Stack present at the top.

If any of those are off, fix inline.

---

## Notes for the executor

- This plan deliberately runs the chip-emulator tests in unit-test isolation BEFORE wiring them up end-to-end. If you go straight to test_boot.py and try to debug from there, you'll spend most of the time chasing emulator wire-format bugs that the unit tests would have caught instantly.
- Phase 7 (the three success-bar tests) is where most of the unknown-unknown work actually lands. Don't rush past it. Each failed boot attempt teaches you something specific about which klippy/extras/* path the synthetic sim never exercised — those are the regressions the sim is supposed to catch in the future.
- If a stub turns out to need more sophistication than the spec called out, extend the stub *first*, then add a unit test for the new behavior, *then* re-run the integration test. Don't paper over with a one-off in conftest.
- The TMC5160 latched-read semantics (read returns previous read's data) are subtle — if SPI reads come back as zeros after a write, you're probably reading once and expecting the answer immediately. Read twice; second reply has the data.
