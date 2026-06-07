#ifndef KALICO_SIM_CHIP_SOCKET_H
#define KALICO_SIM_CHIP_SOCKET_H
#include <stdint.h>
#include <stddef.h>

// Open (or get cached) a Unix-domain stream socket connected to `path`.
// Returns fd >= 0 on success, -1 on error (and shutdown()s the firmware).
int sim_chip_socket_connect(const char *path);

int sim_chip_socket_xfer(int fd, const uint8_t *tx, size_t tx_len,
                         uint8_t *rx, size_t rx_len);

// Wire format:
//   request:  [cs:1][tx_len:1][tx payload tx_len bytes]
//   reply:    [rx_len:1][rx payload rx_len bytes]
int sim_chip_socket_xfer_framed(int fd, uint8_t cs,
                                const uint8_t *tx, size_t tx_len,
                                uint8_t *rx);

#endif
