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

#define MAX_PHASE_BUSES 4

// ---------- 2026-05-18 SPI3 contention arbitration ----------------------
// See phase_stepping_spi.h for the rationale and contract.
static volatile uint8_t  phase_spi_busy = 0;
static volatile uint32_t phase_spi_skip_count = 0;

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
    // Single-byte volatile write is atomic on M4/M7. No critical
    // section needed because we are the sole writer of the cleared
    // state; preemption between read and write of the held state
    // cannot violate invariants.
    phase_spi_busy = 0;
}

__attribute__((used, externally_visible))
uint32_t
phase_spi_get_skip_count(void)
{
    return phase_spi_skip_count;
}

struct phase_bus_state {
    struct spi_config cfg;
    struct gpio_out cs;
    uint8_t configured;
};

// Static, zero-initialized (.bss). `configured == 0` means "not registered".
static struct phase_bus_state phase_buses[MAX_PHASE_BUSES];

// `used + externally_visible`: Klipper's MCU build uses
// `-flto=auto -fwhole-program`, which DCEs symbols not referenced from
// any C translation unit. Both helpers are called exclusively from the
// Rust `runtime` staticlib (via FFI), so without these attributes the
// LTO inliner drops the function bodies and the final link fails with
// `undefined reference to phase_stepping_write_xdirect`. Same pattern
// used by `runtime_emit_step_pulses` in src/stepper.c and
// `runtime_irq_save` / `runtime_irq_restore` in src/runtime_tick.c.
__attribute__((used, externally_visible))
void
phase_stepping_register_bus(uint8_t bus_id, struct spi_config cfg,
                            uint8_t cs_pin)
{
    if (bus_id >= MAX_PHASE_BUSES)
        return;
    phase_buses[bus_id].cfg = cfg;
    phase_buses[bus_id].cs = gpio_out_setup(cs_pin, 1); // idle high
    phase_buses[bus_id].configured = 1;
}

__attribute__((used, externally_visible))
void
phase_stepping_write_xdirect(uint8_t bus_id, uint8_t cs_pin,
                             int16_t coil_a, int16_t coil_b)
{
    // cs_pin is informational; the actual CS handle was cached in
    // phase_stepping_register_bus(). Marked unused to silence the
    // -Wunused-parameter warning that kalico builds with -Wall.
    (void)cs_pin;

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

    spi_prepare(phase_buses[bus_id].cfg);
    gpio_out_write(phase_buses[bus_id].cs, 0); // CS low (assert)
    // 2026-05-19: call the unlocked variant — we already own
    // phase_spi_busy via phase_spi_try_acquire() above. Calling the
    // public spi_transfer here would deadlock the ISR on its own lock.
    spi_transfer_locked(phase_buses[bus_id].cfg, 0,
                        sizeof(datagram), datagram);
    gpio_out_write(phase_buses[bus_id].cs, 1); // CS high (deassert)

    phase_spi_release();
}
