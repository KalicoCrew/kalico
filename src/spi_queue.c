// See spi_queue.h for the design.

#include "spi_queue.h"

// used,externally_visible: only the Rust staticlib references this, so
// -fwhole-program LTO would strip it.
__attribute__((used, externally_visible))
SpiQueue spi_queues[N_SPI_BUSES];
