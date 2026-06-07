#include "autoconf.h"
#include "phase_stepping_spi.h"
#include "gpio.h"
#include "board/irq.h"
#include "board/misc.h"
#include "internal.h"

#define MAX_PHASE_BUSES  4
#define MAX_PHASE_MOTORS 16   // must match Rust state::MAX_STEPPER_OIDS

static volatile uint8_t  phase_spi_busy = 0;
static volatile uint32_t phase_spi_skip_count = 0;
static volatile uint32_t phase_spi_write_count = 0;

static volatile uint8_t phase_spi_writes_enabled = 0;

__attribute__((used, externally_visible))
uint8_t
phase_spi_try_acquire(void)
{
    irqstatus_t flag = irq_save();
    uint8_t was_busy = phase_spi_busy;
    if (!was_busy)
        phase_spi_busy = 1;
    irq_restore(flag);
    return !was_busy;
}

__attribute__((used, externally_visible))
void
phase_spi_release(void)
{
    phase_spi_busy = 0;
}

__attribute__((used, externally_visible))
uint32_t
phase_spi_get_skip_count(void)
{
    return phase_spi_skip_count;
}

__attribute__((used, externally_visible))
uint32_t
phase_spi_get_write_count(void)
{
    return phase_spi_write_count;
}

struct phase_bus_state {
    struct spi_config cfg;
    struct spi_config fast_cfg;
    uint8_t configured;
};

struct phase_motor_state {
    struct gpio_out cs;
    uint8_t bus_id;
    uint8_t configured;
};

static struct phase_bus_state  phase_buses[MAX_PHASE_BUSES];
static struct phase_motor_state phase_motors[MAX_PHASE_MOTORS];

// used,externally_visible: called only from Rust via FFI; without this,
// -fwhole-program LTO DCEs the bodies and the link fails.
__attribute__((used, externally_visible))
void
phase_stepping_register_bus(uint8_t bus_id, struct spi_config cfg)
{
    if (bus_id >= MAX_PHASE_BUSES)
        return;
    phase_buses[bus_id].cfg = cfg;
    // ~1 MHz TMC default = ~40 µs per 5-byte transfer, exceeds the 25 µs tick
    // budget; raise the MBR divisor to ~8 MHz (TMC5160 max SPI rate).
    struct spi_config fast = cfg;
    uint32_t pclk = get_pclock_frequency((uint32_t)(uintptr_t)cfg.spi);
    uint32_t target_rate = 8000000;
    uint32_t div = 0;
    while ((pclk >> (div + 1)) > target_rate && div < 7)
        div++;
    fast.div = div;
    phase_buses[bus_id].fast_cfg = fast;
    phase_buses[bus_id].configured = 1;
}

__attribute__((used, externally_visible))
void
phase_stepping_register_motor(uint8_t motor_idx, uint8_t bus_id,
                              uint8_t cs_pin_id)
{
    if (motor_idx >= MAX_PHASE_MOTORS || bus_id >= MAX_PHASE_BUSES)
        return;
    phase_motors[motor_idx].cs = gpio_out_setup(cs_pin_id, 1); // idle high
    phase_motors[motor_idx].bus_id = bus_id;
    phase_motors[motor_idx].configured = 1;
}

__attribute__((used, externally_visible))
void
phase_stepping_enable_writes(void)
{
    phase_spi_writes_enabled = 1;
}

__attribute__((used, externally_visible))
void
phase_stepping_disable_writes(void)
{
    phase_spi_writes_enabled = 0;
}

__attribute__((used, externally_visible))
void
phase_stepping_write_xdirect(uint8_t motor_idx,
                             int16_t coil_a, int16_t coil_b)
{
    if (!phase_spi_writes_enabled)
        return;
    if (motor_idx >= MAX_PHASE_MOTORS || !phase_motors[motor_idx].configured)
        return;
    uint8_t bus_id = phase_motors[motor_idx].bus_id;
    if (bus_id >= MAX_PHASE_BUSES || !phase_buses[bus_id].configured)
        return;

    if (!phase_spi_try_acquire()) {
        phase_spi_skip_count++;
        return;
    }

    // signed >> is implementation-defined; cast through uint16_t for logical shift.
    uint16_t ua = (uint16_t)coil_a;
    uint16_t ub = (uint16_t)coil_b;

    uint8_t datagram[5] = {
        0xAD,                                // write | XDIRECT (0x2D)
        (uint8_t)((ub >> 8) & 0x01),         // coil_B sign bit
        (uint8_t)(ub & 0xFF),                // coil_B low 8 bits
        (uint8_t)((ua >> 8) & 0x01),         // coil_A sign bit
        (uint8_t)(ua & 0xFF),                // coil_A low 8 bits
    };

#if CONFIG_MACH_STM32H7
    struct spi_config fast = phase_buses[bus_id].fast_cfg;
    SPI_TypeDef *spi = fast.spi;

    spi->CFG1 = ((uint32_t)fast.div << SPI_CFG1_MBR_Pos)
              | (7 << SPI_CFG1_DSIZE_Pos);
    spi->CFG2 = ((uint32_t)fast.mode << SPI_CFG2_CPHA_Pos)
              | SPI_CFG2_MASTER | SPI_CFG2_SSM | SPI_CFG2_AFCNTR
              | SPI_CFG2_SSOE;

    gpio_out_write(phase_motors[motor_idx].cs, 0);

    spi->CR2 = 5u << SPI_CR2_TSIZE_Pos;
    spi->CR1 = SPI_CR1_SSI | SPI_CR1_SPE;
    spi->CR1 = SPI_CR1_SSI | SPI_CR1_CSTART | SPI_CR1_SPE;

    for (int i = 0; i < 5; i++) {
        uint32_t deadline = timer_read_time() + timer_from_us(50);
        while (!(spi->SR & SPI_SR_TXP)) {
            if (!timer_is_before(timer_read_time(), deadline))
                goto bail;
        }
        *(volatile uint8_t *)&spi->TXDR = datagram[i];
    }
    for (int i = 0; i < 5; i++) {
        uint32_t deadline = timer_read_time() + timer_from_us(50);
        while (!(spi->SR & SPI_SR_RXP)) {
            if (!timer_is_before(timer_read_time(), deadline))
                goto bail;
        }
        (void)*(volatile uint8_t *)&spi->RXDR;
    }
    {
        uint32_t deadline = timer_read_time() + timer_from_us(100);
        while (!(spi->SR & SPI_SR_EOT)) {
            if (!timer_is_before(timer_read_time(), deadline))
                goto bail;
        }
    }

bail:
    spi->IFCR = 0xFFFFFFFF;
    spi->CR1 = SPI_CR1_SSI;
#else
    spi_prepare(phase_buses[bus_id].fast_cfg);
    gpio_out_write(phase_motors[motor_idx].cs, 0);
    spi_transfer(phase_buses[bus_id].fast_cfg, 0,
                 sizeof(datagram), datagram);
#endif
    gpio_out_write(phase_motors[motor_idx].cs, 1);

    phase_spi_write_count++;
    phase_spi_release();
}
