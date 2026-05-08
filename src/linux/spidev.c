// Very basic shift-register support via a Linux SPI device
//
// Copyright (C) 2017-2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <fcntl.h> // open
#include <linux/spi/spidev.h> // SPI_IOC_MESSAGE
#include <stdio.h> // snprintf
#include <string.h> // memset
#include <sys/ioctl.h> // ioctl
#include <unistd.h> // write
#include "command.h" // DECL_COMMAND
#include "gpio.h" // spi_setup
#include "internal.h" // report_errno
#include "sched.h" // shutdown
#include "sim_chip_socket.h" // sim_chip_socket_connect, sim_chip_socket_xfer

#define SPIBUS(chip, pin) (((chip)<<8) + (pin))
#define SPIBUS_TO_BUS(spi_bus) ((spi_bus) >> 8)
#define SPIBUS_TO_DEV(spi_bus) ((spi_bus) & 0xff)

// sim_spi* bus range starts at 0xFF00; 16 slots mirrors the spidevN.0 count
#define SIM_SPI_BASE 0xFF00

DECL_ENUMERATION_RANGE("spi_bus", "spidev0.0", SPIBUS(0, 0), 16);
DECL_ENUMERATION_RANGE("spi_bus", "spidev1.0", SPIBUS(1, 0), 16);
DECL_ENUMERATION_RANGE("spi_bus", "spidev2.0", SPIBUS(2, 0), 16);
DECL_ENUMERATION_RANGE("spi_bus", "spidev3.0", SPIBUS(3, 0), 16);
DECL_ENUMERATION_RANGE("spi_bus", "spidev4.0", SPIBUS(4, 0), 16);
DECL_ENUMERATION_RANGE("spi_bus", "spidev5.0", SPIBUS(5, 0), 16);
DECL_ENUMERATION_RANGE("spi_bus", "spidev6.0", SPIBUS(6, 0), 16);
DECL_ENUMERATION_RANGE("spi_bus", "spidev7.0", SPIBUS(7, 0), 16);
DECL_ENUMERATION_RANGE("spi_bus", "sim_spi0", SIM_SPI_BASE, 16);

struct spi_s {
    uint32_t bus, dev;
    int fd;
};
static struct spi_s devices[16];
static int devices_count;

// --- sim SPI routing table ---
// fd is populated by spi_setup() after sim_chip_socket_connect(); used by
// spi_transfer() to identify sim buses without re-entering connect().
struct sim_spi_route { uint32_t bus; char socket_path[64]; int fd; };
static struct sim_spi_route sim_routes[16];
static int sim_routes_count = 0;

static struct sim_spi_route *
sim_spi_route_for_bus(uint32_t bus) {
    for (int i = 0; i < sim_routes_count; i++)
        if (sim_routes[i].bus == bus) return &sim_routes[i];
    return NULL;
}

static int
sim_spi_fd_is_sim(int fd) {
    for (int i = 0; i < sim_routes_count; i++)
        if (sim_routes[i].fd == fd) return 1;
    return 0;
}

// CS-pin side channel. spidev_transfer (in src/spicmds.c) sets this
// immediately before the gpio_out_write that asserts CS, and clears it
// after the post-transfer de-assert. The sim spi_transfer path reads it
// to dispatch the transfer to the per-chip emulator behind the shared
// Unix socket. CS_NONE is used for config_spi_without_cs.
#define SIM_SPI_CS_NONE 0xFF
static uint8_t sim_pending_cs = SIM_SPI_CS_NONE;

void
sim_spi_set_pending_cs(uint8_t cs) {
    sim_pending_cs = cs;
}

void
sim_spi_clear_pending_cs(void) {
    sim_pending_cs = SIM_SPI_CS_NONE;
}

void
sim_spi_register_route(uint32_t bus, const char *path) {
    for (int i = 0; i < sim_routes_count; i++)
        if (sim_routes[i].bus == bus) {
            snprintf(sim_routes[i].socket_path,
                     sizeof(sim_routes[i].socket_path), "%s", path);
            sim_routes[i].fd = -1;
            return;
        }
    if (sim_routes_count >= (int)ARRAY_SIZE(sim_routes))
        shutdown("Too many sim SPI routes");
    snprintf(sim_routes[sim_routes_count].socket_path,
             sizeof(sim_routes[sim_routes_count].socket_path), "%s", path);
    sim_routes[sim_routes_count].bus = bus;
    sim_routes[sim_routes_count].fd = -1;
    sim_routes_count++;
}

static int
spi_open(uint32_t bus, uint32_t dev)
{
    // Find existing device (if already opened)
    int i;
    for (i=0; i<devices_count; i++)
        if (devices[i].bus == bus && devices[i].dev == dev)
            return devices[i].fd;

    // Setup new SPI device
    if (devices_count >= ARRAY_SIZE(devices))
        shutdown("Too many spi devices");
    char fname[256];
    snprintf(fname, sizeof(fname), "/dev/spidev%d.%d", bus, dev);
    int fd = open(fname, O_RDWR|O_CLOEXEC);
    if (fd < 0) {
        report_errno("open spi", fd);
        shutdown("Unable to open spi device");
    }
    int ret = set_non_blocking(fd);
    if (ret < 0)
        shutdown("Unable to set non-blocking on spi device");

    devices[devices_count].bus = bus;
    devices[devices_count].dev = dev;
    devices[devices_count].fd = fd;
    devices_count++;
    return fd;
}

struct spi_config
spi_setup(uint32_t bus, uint8_t mode, uint32_t rate)
{
    // Sim-bus short-circuit: connect via Unix socket, skip ioctl setup.
    struct sim_spi_route *sim_route = sim_spi_route_for_bus(bus);
    if (!sim_route && bus >= SIM_SPI_BASE && bus < SIM_SPI_BASE + 16) {
        // Auto-route: any sim_spi<N> bus with no explicit route falls
        // back to a canonical per-MCU socket path. Flavor prefix mirrors
        // tmcuart's auto-route so H7 and F4 sim builds talk to distinct
        // emulator sockets. The orchestrator binds matching paths in
        // conftest.py before klippy attaches.
        char path[64];
        uint32_t idx = bus - SIM_SPI_BASE;
#if CONFIG_KALICO_RUNTIME
        const char *flavor = "h7";
#else
        const char *flavor = "f4";
#endif
        snprintf(path, sizeof(path), "/tmp/klipper_sim_%s_chip_spi%u",
                 flavor, (unsigned)idx);
        sim_spi_register_route(bus, path);
        sim_route = sim_spi_route_for_bus(bus);
    }
    if (sim_route) {
        int fd = sim_chip_socket_connect(sim_route->socket_path);
        sim_route->fd = fd;
        return (struct spi_config) { fd, (int)rate };
    }

    int bus_id = SPIBUS_TO_BUS(bus), dev_id = SPIBUS_TO_DEV(bus);
    int fd = spi_open(bus_id, dev_id);
    int ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &rate);
    if (ret < 0) {
        report_errno("ioctl set max spi speed", ret);
        shutdown("Unable to set SPI speed");
    }
    ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
    if (ret < 0) {
        report_errno("ioctl set spi mode", ret);
        shutdown("Unable to set SPI mode");
    }
    return (struct spi_config) { fd , rate};
}

void
spi_prepare(struct spi_config config)
{
}

void
spi_transfer(struct spi_config config, uint8_t receive_data
             , uint8_t len, uint8_t *data)
{
    if (!len)
        return;

    // Sim-bus short-circuit: route through Unix socket instead of ioctl.
    if (sim_spi_fd_is_sim(config.fd)) {
        uint8_t scratch[256];
        if (len > sizeof(scratch))
            shutdown("sim spi xfer too long");
        memcpy(scratch, data, len);
        uint8_t reply[256];
        memset(reply, 0, sizeof(reply));
        // Use the framed protocol so the orchestrator can demultiplex
        // transfers per CS pin (a single sim_spi<N> bus typically carries
        // multiple chips, e.g. four TMC5160s + one MAX31865 on H7 spi1).
        sim_chip_socket_xfer_framed(config.fd, sim_pending_cs, scratch, len,
                                    receive_data ? data : reply);
        return;
    }

    if (receive_data) {
        struct spi_ioc_transfer transfer;
        memset(&transfer, 0, sizeof(transfer));
        transfer.tx_buf = (uintptr_t)data;
        transfer.rx_buf = (uintptr_t)data;
        transfer.len = len;
        transfer.speed_hz = config.rate;
        transfer.bits_per_word = 8;
        transfer.cs_change = 0;
        int ret = ioctl(config.fd, SPI_IOC_MESSAGE(1), &transfer);
        if (ret < 0) {
            report_errno("spi ioctl", ret);
            try_shutdown("Unable to issue spi ioctl");
        }
    } else {
        int ret = write(config.fd, data, len);
        if (ret < 0) {
            report_errno("write spi", ret);
            try_shutdown("Unable to write to spi");
        }
    }
}
