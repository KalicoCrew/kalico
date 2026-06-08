// SPSC SPI write queue per bus. Producer = TIM5 ISR (Rust), consumer =
// foreground struct-timer (Klipper SysTick). Storage C-owned per the B2/B3
// invariant in docs/kalico-rewrite/mcu-c-rust-boundary.md; struct layout
// mirrors rust/runtime/src/spi_queue.rs.

#ifndef __KALICO_SPI_QUEUE_H
#define __KALICO_SPI_QUEUE_H

#include <stdint.h>
#include <stddef.h>

#define SPI_QUEUE_DEPTH       16
#define SPI_QUEUE_DEPTH_MASK  0x0F
#define N_SPI_BUSES           3

typedef struct {
    uint8_t  motor_idx;
    uint8_t  _pad;
    int16_t  coil_a;
    int16_t  coil_b;
    uint8_t  _pad2[2];
} SpiWrite;

typedef struct {
    volatile uint16_t tail;
    volatile uint16_t head;
    uint8_t _pad[4];
    SpiWrite buf[SPI_QUEUE_DEPTH];
} SpiQueue;

extern SpiQueue spi_queues[N_SPI_BUSES];

_Static_assert(sizeof(SpiWrite) == 8, "SpiWrite layout drift");
_Static_assert(sizeof(SpiQueue) == 136, "SpiQueue layout drift");
_Static_assert(offsetof(SpiQueue, buf) == 8, "SpiQueue.buf offset drift");
_Static_assert((SPI_QUEUE_DEPTH & SPI_QUEUE_DEPTH_MASK) == 0,
               "SPI_QUEUE_DEPTH must be power of 2");

#endif
