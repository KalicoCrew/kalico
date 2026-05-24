// Phase-stepping XDIRECT SPI writer for TMC5160 (sim scope).
// See phase_stepping_spi.h for protocol and datagram layout details.
//
// Pattern matches src/spicmds.c::spidev_transfer():
//   spi_prepare(cfg) -> CS low -> spi_transfer(cfg, 0, len, buf) -> CS high.
// spi_prepare is required on STM32H7 because each bus's CR1 is rewritten
// per-transaction in stm32h7_spi.c; omitting it would re-use the previous
// caller's clock divider / mode. In Renode this is benign, but we follow
// the canonical pattern so the same .c is correct on real silicon.

#include "phase_stepping_spi.h"
#include "gpio.h"   // struct spi_config, spi_prepare, spi_transfer,
                    // struct gpio_out, gpio_out_setup, gpio_out_write
#include "board/irq.h" // irq_save, irq_restore, irqstatus_t
#include "internal.h" // get_pclock_frequency

#define MAX_PHASE_BUSES  4
#define MAX_PHASE_MOTORS 16   // matches Rust state::MAX_STEPPER_OIDS

// ---------- 2026-05-18 SPI3 contention arbitration ----------------------
// See phase_stepping_spi.h for the rationale and contract.
static volatile uint8_t  phase_spi_busy = 0;
static volatile uint32_t phase_spi_skip_count = 0;
static volatile uint32_t phase_spi_write_count = 0;

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

// Static, zero-initialized (.bss). `configured == 0` means "not registered".
static struct phase_bus_state  phase_buses[MAX_PHASE_BUSES];
static struct phase_motor_state phase_motors[MAX_PHASE_MOTORS];

// `used + externally_visible`: Klipper's MCU build uses
// `-flto=auto -fwhole-program`, which DCEs symbols not referenced from
// any C translation unit. All three helpers are called exclusively from
// the Rust `runtime` staticlib (via FFI), so without these attributes
// the LTO inliner drops the function bodies and the final link fails
// with `undefined reference to ...`. Same pattern used by
// `runtime_emit_step_pulses` in src/stepper.c and `runtime_irq_save` /
// `runtime_irq_restore` in src/runtime_tick.c.
__attribute__((used, externally_visible))
void
phase_stepping_register_bus(uint8_t bus_id, struct spi_config cfg)
{
    if (bus_id >= MAX_PHASE_BUSES)
        return;
    phase_buses[bus_id].cfg = cfg;
    // XDIRECT writes run from the TIM5 ISR at 40 kHz. At the default
    // TMC SPI rate (~1 MHz) a 5-byte transfer takes ~40 µs — well over
    // the 25 µs tick budget for even a single motor. Override the MBR
    // divisor so the XDIRECT path runs at ~4 MHz (5 µs per motor).
    // TMC5160 datasheet maximum is 8 MHz; 4 MHz is conservative.
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
phase_stepping_write_xdirect(uint8_t motor_idx,
                             int16_t coil_a, int16_t coil_b)
{
    if (motor_idx >= MAX_PHASE_MOTORS || !phase_motors[motor_idx].configured)
        return;
    uint8_t bus_id = phase_motors[motor_idx].bus_id;
    if (bus_id >= MAX_PHASE_BUSES || !phase_buses[bus_id].configured)
        return;

    // ISR-priority: if Klipper's spi_transfer holds the bus, skip this
    // modulation cycle. One skip = 25 us at 40 kHz, inaudible. The
    // skip-count telemetry is the canary for SPI3 contention going wild.
    if (!phase_spi_try_acquire()) {
        phase_spi_skip_count++;
        return;
    }

    // Cast through uint16_t before shifting so the sign bit lands in
    // bit 8 of the source word (C right-shift on signed negative values
    // is implementation-defined; uint16_t guarantees a logical shift).
    uint16_t ua = (uint16_t)coil_a;
    uint16_t ub = (uint16_t)coil_b;

    uint8_t datagram[5] = {
        0xAD,                                // write | XDIRECT (0x2D)
        (uint8_t)((ub >> 8) & 0x01),         // coil_B sign bit
        (uint8_t)(ub & 0xFF),                // coil_B low 8 bits
        (uint8_t)((ua >> 8) & 0x01),         // coil_A sign bit
        (uint8_t)(ua & 0xFF),                // coil_A low 8 bits
    };

    spi_prepare(phase_buses[bus_id].fast_cfg);
    gpio_out_write(phase_motors[motor_idx].cs, 0); // CS low (assert)
    spi_transfer_locked(phase_buses[bus_id].fast_cfg, 0,
                        sizeof(datagram), datagram);
    gpio_out_write(phase_motors[motor_idx].cs, 1); // CS high (deassert)

    phase_spi_write_count++;
    phase_spi_release();
}
