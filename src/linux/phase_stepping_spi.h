#ifndef _PHASE_STEPPING_SPI_H
#define _PHASE_STEPPING_SPI_H

#include <stdint.h>

struct spi_config;

/* Cache an SPI bus config for later phase-stepping XDIRECT writes. Call
 * once per unique bus_id during the per-MCU phase-stepping registration
 * sequence (after kalico runtime has invoked spi_setup() to produce the
 * spi_config) and BEFORE any phase_stepping_register_motor() call that
 * names this bus.
 *
 *   bus_id: phase-stepping bus slot in [0, MAX_PHASE_BUSES). NOT the
 *           same as kalico's SPI peripheral index.
 *   cfg:    pre-initialized spi_config from spi_setup().
 *
 * The CS line is NOT registered here — see phase_stepping_register_motor.
 * Multiple TMC5160s share the bus cfg (rate, mode) but each owns its own
 * CS GPIO, so CS state is per-motor, not per-bus.
 */
void phase_stepping_register_bus(uint8_t bus_id, struct spi_config cfg);

/* Cache the CS GPIO for a single phase-stepped motor. Call once per
 * phase-stepped motor, after phase_stepping_register_bus() for the
 * referenced bus_id has been called. The pin is configured as gpio_out
 * idle-high.
 *
 *   motor_idx:  Rust motor slot in [0, MAX_PHASE_MOTORS), matching
 *               the index used by the runtime's per-motor PhaseConfig
 *               storage.
 *   bus_id:     phase-stepping bus slot in [0, MAX_PHASE_BUSES); must
 *               have been registered via phase_stepping_register_bus
 *               first or the write_xdirect path will no-op.
 *   cs_pin_id:  kalico GPIO encoding (port * 16 + pin on stm32). Set
 *               high (deasserted) on registration.
 */
void phase_stepping_register_motor(uint8_t motor_idx,
                                   uint8_t bus_id,
                                   uint8_t cs_pin_id);

/* Emit a single TMC5160 XDIRECT register write for the given motor.
 * Asserts the motor's CS low, performs a blocking 5-byte transfer on
 * the motor's bus, deasserts CS.
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
 * Per-motor CS dispatch (added 2026-05-19 to fix the multi-TMC5160-on-
 * one-SPI-bus bug — the previous (bus_id, cs_pin) signature ignored
 * cs_pin and pulled a single bus-cached CS, so writes to motor N>0 on
 * a shared bus hit motor 0's driver instead).
 *
 * If motor_idx is out of range, or no register_motor() has been called
 * for it, or its bus has not been registered, the function is a no-op.
 *
 * SIM-ONLY: this helper is blocking. Silicon implementation per spec §8
 * is DMA-driven with CS released by timer output-compare.
 */
void phase_stepping_write_xdirect(uint8_t motor_idx,
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
uint32_t phase_spi_get_write_count(void);

void phase_stepping_enable_writes(void);
void phase_stepping_disable_writes(void);

// Bare SPI3 transfer for ISR callers that already hold phase_spi_busy.
// External callers MUST NOT use this — use spi_transfer instead.
// Calling this without holding phase_spi_busy races against any
// concurrent task-context spi_transfer; calling spi_transfer while
// already holding phase_spi_busy deadlocks on the spin-acquire (see
// 2026-05-19 fix in stm32h7_spi.c).
//
// H7-only symbol: only the H7's stm32h7_spi.c implements the busy-flag
// gating. On other targets (F4, G4, etc.) the busy-flag is a no-op and
// the bare transfer simply forwards to spi_transfer — there's no
// SPI3-style multi-writer contention to mediate. Declared as a static
// inline for non-H7 builds so phase_stepping_spi.c links cleanly on
// every target.
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
