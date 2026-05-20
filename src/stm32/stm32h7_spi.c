// SPI functions on STM32H7
//
// Copyright (C) 2019-2025  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/io.h" // readb, writeb
#include "command.h" // shutdown
#include "gpio.h" // spi_setup
#include "internal.h" // gpio_peripheral
#include "sched.h" // sched_shutdown
#include "board/misc.h" // timer_is_before
#include "phase_stepping_spi.h" // phase_spi_try_acquire / phase_spi_release

// Spi-hang last-seen state. Written by spi_transfer_locked just before
// shutdown(); read by runtime_tick.c's diag rotation (tag 0xD0) so the
// stuck peripheral's register snapshot survives the shutdown via the
// next status frame, even when output() doesn't drain.
volatile uint32_t kalico_spi_hang_addr __attribute__((used, externally_visible));
volatile uint32_t kalico_spi_hang_sr   __attribute__((used, externally_visible));
volatile uint32_t kalico_spi_hang_cr1  __attribute__((used, externally_visible));
volatile uint32_t kalico_spi_hang_cr2  __attribute__((used, externally_visible));
volatile uint32_t kalico_spi_hang_cfg2 __attribute__((used, externally_visible));
// Low 4 bits = remaining-byte count at hang; bit 7 = R (rx) / E (eot)
volatile uint8_t  kalico_spi_hang_reason __attribute__((used, externally_visible));

struct spi_info {
    SPI_TypeDef *spi;
    uint8_t miso_pin, mosi_pin, sck_pin, function;
};

DECL_ENUMERATION("spi_bus", "spi2", __COUNTER__);
DECL_CONSTANT_STR("BUS_PINS_spi2", "PB14,PB15,PB13");

DECL_ENUMERATION("spi_bus", "spi1", __COUNTER__);
DECL_CONSTANT_STR("BUS_PINS_spi1", "PA6,PA7,PA5");
DECL_ENUMERATION("spi_bus", "spi1a", __COUNTER__);
DECL_CONSTANT_STR("BUS_PINS_spi1a", "PB4,PB5,PB3");

#if !CONFIG_MACH_STM32F1
DECL_ENUMERATION("spi_bus", "spi2a", __COUNTER__);
DECL_CONSTANT_STR("BUS_PINS_spi2a", "PC2,PC3,PB10");
#endif

#ifdef SPI3
DECL_ENUMERATION("spi_bus", "spi3a", __COUNTER__);
DECL_CONSTANT_STR("BUS_PINS_spi3a", "PC11,PC12,PC10");
#endif

#ifdef SPI4
DECL_ENUMERATION("spi_bus", "spi4", __COUNTER__);
DECL_CONSTANT_STR("BUS_PINS_spi4", "PE13,PE14,PE12");
#endif

#ifdef GPIOI
DECL_ENUMERATION("spi_bus", "spi2b", __COUNTER__);
DECL_CONSTANT_STR("BUS_PINS_spi2b", "PI2,PI3,PI1");
#endif

#ifdef SPI5
DECL_ENUMERATION("spi_bus", "spi5", __COUNTER__);
DECL_CONSTANT_STR("BUS_PINS_spi5", "PF8,PF9,PF7");
DECL_ENUMERATION("spi_bus", "spi5a", __COUNTER__);
DECL_CONSTANT_STR("BUS_PINS_spi5a", "PH7,PF11,PH6");
#endif

#ifdef SPI6
DECL_ENUMERATION("spi_bus", "spi6", __COUNTER__);
DECL_CONSTANT_STR("BUS_PINS_spi6", "PG12,PG14,PG13");
#endif


static const struct spi_info spi_bus[] = {
    { SPI2, GPIO('B', 14), GPIO('B', 15), GPIO('B', 13), GPIO_FUNCTION(5) },
    { SPI1, GPIO('A', 6), GPIO('A', 7), GPIO('A', 5), GPIO_FUNCTION(5) },
    { SPI1, GPIO('B', 4), GPIO('B', 5), GPIO('B', 3), GPIO_FUNCTION(5) },
#if !CONFIG_MACH_STM32F1
    { SPI2, GPIO('C', 2), GPIO('C', 3), GPIO('B', 10), GPIO_FUNCTION(5) },
#endif
#ifdef SPI3
    { SPI3, GPIO('C', 11), GPIO('C', 12), GPIO('C', 10), GPIO_FUNCTION(6) },
#endif
#ifdef SPI4
    { SPI4, GPIO('E', 13), GPIO('E', 14), GPIO('E', 12), GPIO_FUNCTION(5) },
#endif
    { SPI2, GPIO('I', 2), GPIO('I', 3), GPIO('I', 1), GPIO_FUNCTION(5) },
#ifdef SPI5
    { SPI5, GPIO('F', 8), GPIO('F', 9), GPIO('F', 7), GPIO_FUNCTION(5) },
    { SPI5, GPIO('H', 7), GPIO('F', 11), GPIO('H', 6), GPIO_FUNCTION(5) },
#endif
#ifdef SPI6
    { SPI6, GPIO('G', 12), GPIO('G', 14), GPIO('G', 13), GPIO_FUNCTION(5)},
#endif
};

struct spi_config
spi_setup(uint32_t bus, uint8_t mode, uint32_t rate)
{
    if (bus >= ARRAY_SIZE(spi_bus))
        shutdown("Invalid spi bus");

    // Enable SPI
    SPI_TypeDef *spi = spi_bus[bus].spi;
    if (!is_enabled_pclock((uint32_t)spi)) {
        enable_pclock((uint32_t)spi);
        gpio_peripheral(spi_bus[bus].miso_pin, spi_bus[bus].function, 1);
        gpio_peripheral(spi_bus[bus].mosi_pin, spi_bus[bus].function, 0);
        gpio_peripheral(spi_bus[bus].sck_pin, spi_bus[bus].function, 0);
    }

    // Calculate CR1 register
    uint32_t pclk = get_pclock_frequency((uint32_t)spi);
    uint32_t div = 0;
    while ((pclk >> (div + 1)) > rate && div < 7)
        div++;

    return (struct spi_config){ .spi = spi, .div = div, .mode = mode };
}

void
spi_prepare(struct spi_config config)
{
    uint32_t div = config.div;
    uint32_t mode = config.mode;
    SPI_TypeDef *spi = config.spi;

    // Load frequency
    spi->CFG1 = (div << SPI_CFG1_MBR_Pos) | (7 << SPI_CFG1_DSIZE_Pos);
    // Load mode
    uint32_t cfg2 = ((mode << SPI_CFG2_CPHA_Pos) | SPI_CFG2_MASTER
                     | SPI_CFG2_SSM | SPI_CFG2_AFCNTR | SPI_CFG2_SSOE);
    uint32_t diff = spi->CFG2 ^ cfg2;
    spi->CFG2 = cfg2;
    uint32_t end = timer_read_time() + timer_from_us(1);
    if (diff & SPI_CFG2_CPOL_Msk)
        while (timer_is_before(timer_read_time(), end))
            ;
}

// Bare SPI3 transfer — caller is responsible for holding phase_spi_busy
// and serializing access to the SPI peripheral. Used by both
// spi_transfer (task-context, takes the lock itself) and
// phase_stepping_write_xdirect (ISR-context, already holds the lock).
//
// Note: the shutdown("spi rx timeout") / shutdown("spi eot timeout")
// calls below are __noreturn -- leaving phase_spi_busy=1 latched is
// intentional. The ISR-side phase_stepping_write_xdirect skip path
// remains safe during MCU halt.
//
// Pipelined TX-fill / RX-drain pattern uses SPI_SR_RXP (rx-packet, bit 0)
// for per-byte rx detection in 8-bit data mode (CFG1.DSIZE=7). The earlier
// fork variant polled SPI_SR_RXWNE | SPI_SR_RXPLVL — those only fire at
// 32-bit word boundaries / packets, so any non-multiple-of-4 transfer
// (e.g. MAX31865 3-byte RTD read) hung forever, surfacing as
// "spi rx timeout" when the 100us deadline expired. Pattern matches
// Klipper upstream src/stm32/stm32h7_spi.c.
#define MAX_FIFO 8 // Limit tx fifo usage so rx fifo doesn't overrun

void
spi_transfer_locked(struct spi_config config, uint8_t receive_data,
                    uint8_t len, uint8_t *data)
{
    uint8_t *wptr = data, *end = data + len;
    SPI_TypeDef *spi = config.spi;

    spi->CR2 = (uint32_t)len << SPI_CR2_TSIZE_Pos;
    // Enable SPI and start transfer, these MUST be set in this sequence
    spi->CR1 = SPI_CR1_SSI | SPI_CR1_SPE;
    spi->CR1 = SPI_CR1_SSI | SPI_CR1_CSTART | SPI_CR1_SPE;

    // Bridge-call stall investigation (2026-05-09): bound the busy-wait
    // loops with a deadline. The original code had no timeout — a
    // hardware-level SPI deadlock (CS glitch, FIFO state inconsistency,
    // MISO transient) wedged the cooperative scheduler forever, ALL
    // tasks blocked, IWDG fired after 30s, host saw "transport
    // closed/timeout". This converts the silent wedge into a clean
    // shutdown with a diagnosable reason. Deadline is reset on every
    // observed byte of forward progress so a busy bus (multi-driver
    // pipeline) doesn't trip a false positive.
    //
    // Budget: 100us per pending byte. 4MHz SPI = 2us per byte clocked;
    // 100us is 50x headroom for FIFO drain + cooperative scheduling.
    uint32_t spi_deadline = timer_read_time() + timer_from_us(100 * len);
    while (data < end) {
        uint32_t sr = spi->SR & (SPI_SR_TXP | SPI_SR_RXP);
        if ((sr & SPI_SR_TXP) && wptr < end && wptr < data + MAX_FIFO) {
            writeb((void *)&spi->TXDR, *wptr++);
            spi_deadline = timer_read_time() + timer_from_us(100 * (uint32_t)(end - data));
            continue;
        }
        if (sr & SPI_SR_RXP) {
            uint8_t rdata = readb((void *)&spi->RXDR);
            if (receive_data) {
                *data = rdata;
            }
            data++;
            spi_deadline = timer_read_time() + timer_from_us(100 * (uint32_t)(end - data));
            continue;
        }
        if (!timer_is_before(timer_read_time(), spi_deadline)) {
            kalico_spi_hang_addr = (uint32_t)(uintptr_t)spi;
            kalico_spi_hang_sr   = spi->SR;
            kalico_spi_hang_cr1  = spi->CR1;
            kalico_spi_hang_cr2  = spi->CR2;
            kalico_spi_hang_cfg2 = spi->CFG2;
            kalico_spi_hang_reason = (uint8_t)((uint32_t)(end - data) & 0x0F);
            shutdown("spi rx timeout");
        }
    }

    uint32_t eot_deadline = timer_read_time() + timer_from_us(100);
    while ((spi->SR & SPI_SR_EOT) == 0) {
        if (!timer_is_before(timer_read_time(), eot_deadline)) {
            kalico_spi_hang_addr = (uint32_t)(uintptr_t)spi;
            kalico_spi_hang_sr   = spi->SR;
            kalico_spi_hang_cr1  = spi->CR1;
            kalico_spi_hang_cr2  = spi->CR2;
            kalico_spi_hang_cfg2 = spi->CFG2;
            kalico_spi_hang_reason = (uint8_t)0x80; // bit 7 = EOT path
            shutdown("spi eot timeout");
        }
    }

    // Clear flags and disable SPI
    spi->IFCR = 0xFFFFFFFF;
    spi->CR1 = SPI_CR1_SSI;
}

void
spi_transfer(struct spi_config config, uint8_t receive_data,
             uint8_t len, uint8_t *data)
{
    // 2026-05-18 phase-stepping SPI3 contention: Klipper's task-context
    // SPI access must coordinate with the TIM5-rate XDIRECT ISR. The
    // busy-flag is per-MCU global (one SPI3 instance per H723), so we
    // gate every spi_transfer call. Non-SPI3 transfers see the flag
    // uncontested and acquire/release with negligible overhead (~10
    // cycles per pair). The wait path is bounded: the TIM5 ISR releases
    // within ~25 us of acquiring.
    //
    // 2026-05-19 deadlock fix: this entry point is for task-context
    // callers only. ISR-context callers (phase_stepping_write_xdirect)
    // already hold phase_spi_busy and MUST call spi_transfer_locked
    // directly — otherwise the spin-acquire below loops forever against
    // the lock the ISR itself owns, USB CDC pump starves, the H7
    // re-enumerates, and klippy aborts via EXIT_ON_FAULT.
    while (!phase_spi_try_acquire()) {
        // Spin; the ISR-side write completes within one TIM5 period
        // (25 us at 40 kHz). On real hardware the CPU is not idle here
        // -- the next TIM5 ISR fire will release. In Renode sim, the
        // virtual time advances under the spin loop.
    }
    spi_transfer_locked(config, receive_data, len, data);
    phase_spi_release();
}
