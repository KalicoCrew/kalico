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

// ---------- 2026-05-18 SPI3 contention arbitration ----------------------
// Cooperative busy-flag mediating SPI3 access between two writers:
//   - TIM5-rate (40 kHz) phase_stepping_write_xdirect from the ISR
//   - Lower-priority TMC SPI register access from Klipper task code
//     (e.g. _do_periodic_check's 1 Hz DRV_STATUS polling)
//
// Both paths MUST acquire before initiating an SPI transfer and release
// after. The flag uses irq_save / irq_restore for mutual exclusion;
// CMSIS atomic primitives are not required because all writers are
// single-instruction reads/writes against a uint8_t.
//
// Return value of phase_spi_try_acquire(): 1 if acquired, 0 if busy.
// The TIM5 ISR's phase_stepping_write_xdirect skips its cycle on 0 and
// increments phase_spi_skip_count for telemetry. Klipper's spi_transfer
// for the registered phase bus instead spins (or yields, per
// stm32h7_spi.c convention) until acquire succeeds — that path tolerates
// the wait latency.
uint8_t phase_spi_try_acquire(void);
void    phase_spi_release(void);
uint32_t phase_spi_get_skip_count(void);

// Bare SPI3 transfer for ISR callers that already hold phase_spi_busy.
// External callers MUST NOT use this — use spi_transfer instead.
// Calling this without holding phase_spi_busy races against any
// concurrent task-context spi_transfer; calling spi_transfer while
// already holding phase_spi_busy deadlocks on the spin-acquire (see
// 2026-05-19 fix in stm32h7_spi.c).
void spi_transfer_locked(struct spi_config config, uint8_t receive_data,
                         uint8_t len, uint8_t *data);

#endif // phase_stepping_spi.h
