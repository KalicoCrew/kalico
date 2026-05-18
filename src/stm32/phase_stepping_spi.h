#ifndef _PHASE_STEPPING_SPI_H
#define _PHASE_STEPPING_SPI_H

#include <stdint.h>

struct spi_config;

/* Cache an SPI bus + chip-select pin for later phase-stepping XDIRECT
 * writes. Call once per phase-stepped motor during configure_axes_blob
 * processing, after kalico runtime has invoked spi_setup() to produce
 * the spi_config. The CS pin is configured as gpio_out idle-high.
 *
 *   bus_id: phase-stepping bus slot in [0, MAX_PHASE_BUSES). NOT the
 *           same as kalico's SPI peripheral index.
 *   cfg:    pre-initialized spi_config from spi_setup().
 *   cs_pin: kalico GPIO encoding (port * 16 + pin on stm32). The pin
 *           is set high (deasserted) on registration.
 */
void phase_stepping_register_bus(uint8_t bus_id, struct spi_config cfg,
                                 uint8_t cs_pin);

/* Emit a single TMC5160 XDIRECT register write to the bus previously
 * registered with phase_stepping_register_bus(). Asserts CS low,
 * performs a blocking 5-byte transfer, deasserts CS.
 *
 * Datagram layout per TMC5160 datasheet (40-bit, MSB first):
 *   byte 0 = 0xAD              -- write bit (0x80) | XDIRECT addr (0x2D)
 *   byte 1 = (coil_b >> 8) & 1 -- coil_B sign bit
 *   byte 2 = coil_b & 0xFF     -- coil_B low 8 bits
 *   byte 3 = (coil_a >> 8) & 1 -- coil_A sign bit
 *   byte 4 = coil_a & 0xFF     -- coil_A low 8 bits
 *
 * coil_a, coil_b: signed 9-bit values in [-256, +255]. Values outside
 * this range are silently clipped by the bit-packing (high bits dropped).
 *
 * The cs_pin argument is informational only — the actual CS line driven
 * is the one cached during phase_stepping_register_bus(). It is kept
 * in the signature so callers (FFI from the Rust modulator) can pass
 * the per-motor cs_pin without an extra lookup.
 *
 * If bus_id is out of range or no register_bus() has been called for
 * the slot, the function is a no-op.
 *
 * SIM-ONLY: this helper is blocking. Silicon implementation per spec §8
 * is DMA-driven with CS released by timer output-compare.
 */
void phase_stepping_write_xdirect(uint8_t bus_id, uint8_t cs_pin,
                                  int16_t coil_a, int16_t coil_b);

#endif // phase_stepping_spi.h
