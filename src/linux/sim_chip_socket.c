// src/linux/sim_chip_socket.c
#include "sim_chip_socket.h"
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include "command.h"  // shutdown
#include "internal.h" // report_errno

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
    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
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

static int sim_chip_socket_write_all(int fd, const uint8_t *buf, size_t len) {
    size_t off = 0;
    while (off < len) {
        ssize_t n = write(fd, buf + off, len - off);
        if (n <= 0) return -1;
        off += n;
    }
    return 0;
}

static int sim_chip_socket_read_all(int fd, uint8_t *buf, size_t len) {
    size_t off = 0;
    while (off < len) {
        ssize_t n = read(fd, buf + off, len - off);
        if (n <= 0) return -1;
        off += n;
    }
    return 0;
}

int sim_chip_socket_xfer_framed(int fd, uint8_t cs,
                                const uint8_t *tx, size_t tx_len,
                                uint8_t *rx) {
    if (tx_len == 0 || tx_len > 255) return -1;
    uint8_t hdr[2] = { cs, (uint8_t)tx_len };
    if (sim_chip_socket_write_all(fd, hdr, sizeof(hdr)) < 0) return -1;
    if (sim_chip_socket_write_all(fd, tx, tx_len) < 0) return -1;
    uint8_t rx_len_byte = 0;
    if (sim_chip_socket_read_all(fd, &rx_len_byte, 1) < 0) return -1;
    if (rx_len_byte != tx_len) return -1;
    if (sim_chip_socket_read_all(fd, rx, rx_len_byte) < 0) return -1;
    return 0;
}
