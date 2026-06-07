#ifndef _PHASE_STEPPING_SPI_H
#define _PHASE_STEPPING_SPI_H

#include <stdint.h>

struct spi_config;

void phase_stepping_register_bus(uint8_t bus_id, struct spi_config cfg);

void phase_stepping_register_motor(uint8_t motor_idx,
                                   uint8_t bus_id,
                                   uint8_t cs_pin_id);

void phase_stepping_write_xdirect(uint8_t motor_idx,
                                  int16_t coil_a, int16_t coil_b);

// Every SPI3 writer MUST acquire before a transfer and release after: the
// TIM5 ISR (phase_stepping_write_xdirect) and task-context TMC register
// access share the bus, and an unguarded transfer corrupts an in-flight one.
uint8_t phase_spi_try_acquire(void);
void    phase_spi_release(void);
uint32_t phase_spi_get_skip_count(void);
uint32_t phase_spi_get_write_count(void);

void phase_stepping_enable_writes(void);
void phase_stepping_disable_writes(void);

// For ISR callers that already hold phase_spi_busy. Calling without the busy
// flag held races a concurrent spi_transfer; calling spi_transfer while
// holding it deadlocks on the spin-acquire.
#ifdef CONFIG_MACH_STM32H7
void spi_transfer_locked(struct spi_config config, uint8_t receive_data,
                         uint8_t len, uint8_t *data);
#else
#include "gpio.h"
static inline void
spi_transfer_locked(struct spi_config config, uint8_t receive_data,
                    uint8_t len, uint8_t *data)
{
    spi_transfer(config, receive_data, len, data);
}
#endif

#endif
