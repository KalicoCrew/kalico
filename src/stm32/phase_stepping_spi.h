#ifndef _PHASE_STEPPING_SPI_H
#define _PHASE_STEPPING_SPI_H

#include <stdint.h>

struct spi_config;

/* Cache an SPI bus config for XDIRECT writes. Call once per bus_id, after
 * spi_setup() and BEFORE any register_motor() naming this bus. Multiple
 * TMC5160s share the cfg but each owns its own CS GPIO. bus_id is a phase
 * slot, not kalico's SPI peripheral index. */
void phase_stepping_register_bus(uint8_t bus_id, struct spi_config cfg);

/* Cache the CS GPIO (idle-high) for one motor. Call after
 * register_bus(bus_id). motor_idx is the runtime per-motor slot; cs_pin_id is
 * the kalico GPIO encoding (port*16+pin). No-op if the bus is unregistered. */
void phase_stepping_register_motor(uint8_t motor_idx,
                                   uint8_t bus_id,
                                   uint8_t cs_pin_id);

/* Emit one TMC5160 XDIRECT write: CS low, blocking 5-byte transfer, CS high.
 *
 * Datagram (40-bit, MSB first):
 *   byte 0 = 0xAD              -- write (0x80) | XDIRECT (0x2D)
 *   byte 1 = (coil_b >> 8) & 1 -- coil_B sign bit
 *   byte 2 = coil_b & 0xFF     -- coil_B low 8 bits
 *   byte 3 = (coil_a >> 8) & 1 -- coil_A sign bit
 *   byte 4 = coil_a & 0xFF     -- coil_A low 8 bits
 *
 * coil_a/coil_b are signed 9-bit [-256,+255]; out-of-range values are clipped
 * by the bit-packing. No-op for an out-of-range / unregistered motor. */
void phase_stepping_write_xdirect(uint8_t motor_idx,
                                  int16_t coil_a, int16_t coil_b);

// Cooperative busy-flag mediating SPI3 between the TIM5-ISR XDIRECT writer and
// lower-priority Klipper-task TMC register access. Both MUST acquire before a
// transfer and release after; irq_save/irq_restore gives mutual exclusion (all
// writers are single-instruction uint8_t accesses). try_acquire returns 1 if
// acquired, 0 if busy — the ISR skips + counts on 0; the task path spins.
uint8_t phase_spi_try_acquire(void);
void    phase_spi_release(void);
uint32_t phase_spi_get_skip_count(void);
uint32_t phase_spi_get_write_count(void);

// Gate: the ISR skips XDIRECT writes until enable_writes, preventing contention
// with the foreground TMC register init.
void phase_stepping_enable_writes(void);
void phase_stepping_disable_writes(void);

// Bare transfer for ISR callers that already hold phase_spi_busy. External
// callers MUST use spi_transfer instead — calling spi_transfer while holding
// the flag deadlocks on the spin-acquire. H7-only (only stm32h7_spi.c gates on
// the flag); a static inline forwards to spi_transfer on other targets so this
// file links everywhere.
#ifdef CONFIG_MACH_STM32H7
void spi_transfer_locked(struct spi_config config, uint8_t receive_data,
                         uint8_t len, uint8_t *data);
#else
#include "gpio.h" // spi_transfer
static inline void
spi_transfer_locked(struct spi_config config, uint8_t receive_data,
                    uint8_t len, uint8_t *data)
{
    spi_transfer(config, receive_data, len, data);
}
#endif

#endif // phase_stepping_spi.h
