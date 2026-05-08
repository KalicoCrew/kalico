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

// Framed variant used for buses that multiplex multiple chip emulators
// behind a single Unix socket (e.g. shared SPI bus where each chip has
// its own CS pin). Wire format:
//   request:  [cs:1][tx_len:1][tx payload tx_len bytes]
//   reply:    [rx_len:1][rx payload rx_len bytes]
// SPI transfers are symmetric so the caller passes a single buffer of
// tx_len; the server replies with a frame of equal length. Returns 0 on
// success, -1 on protocol/IO error.
int sim_chip_socket_xfer_framed(int fd, uint8_t cs,
                                const uint8_t *tx, size_t tx_len,
                                uint8_t *rx);

// Register a SPI bus → Unix-socket-path mapping. Called from the
// runtime_sim_route_spi command handler; takes effect on the next
// spi_setup that resolves to this bus.
void sim_spi_register_route(uint32_t bus, const char *path);

// Register a tmcuart oid → Unix-socket-path mapping. Mirror of
// sim_spi_register_route for the bit-banged TMC2209 path.
void sim_tmcuart_register_route(uint8_t oid, const char *path);

#endif
