// See spi_queue.h for the design. C-owned storage for the per-bus SPSC
// SPI write queues; the TIM5 ISR (Rust) pushes XDIRECT coil writes here
// and a foreground struct-timer drains them through Klipper's SPI driver.

#include "spi_queue.h"

// `used, externally_visible` survives Klipper's -fwhole-program -flto
// build, which would otherwise strip this symbol — only the Rust
// staticlib references it.
__attribute__((used, externally_visible))
SpiQueue spi_queues[N_SPI_BUSES];
