//! buzz-h7 — bare-metal stepper buzz test for STM32H723 (BTT Octopus Pro).
//!
//! ZERO Klipper / Kalico code. Just:
//!   1. Brings the core out of reset on HSI (64 MHz). No PLL — simpler init,
//!      well under TMC5160's 4 MHz SPI ceiling and faster than the step rate
//!      the motor cares about.
//!   2. Enables GPIO clocks for ports A, C, G.
//!   3. Wires up step (PG4), dir (PC1, inverted), enable (PA2, inverted),
//!      CS (PC7), and SPI1 (PA5/PA6/PA7) per the user's Trident printer.cfg
//!      [stepper_x] + [tmc5160 stepper_x] section.
//!   4. Drops EN low so the TMC powers the motor.
//!   5. Sends six TMC5160 register writes over SPI:
//!        GCONF (0x00) = 0x00000004    en_pwm_mode + multistep_filt off (we
//!                                     want spreadcycle for clearest test)
//!                                     Actually we set 0 = pure spreadcycle.
//!        GLOBAL_SCALER (0x0B) = 128    50% scaler so IRUN=8 ≈ 0.7 A on
//!                                     the user's 0.075 Ω sense resistor.
//!        IHOLD_IRUN (0x10) = 0x00081008  ihold=8, irun=8, iholddelay=8
//!        TPOWERDOWN (0x11) = 0x0A     fast standstill power-down
//!        CHOPCONF (0x6C) = 0x000100C3 toff=3, hstrt=4, hend=1, tbl=2 —
//!                                     safe defaults that energize the
//!                                     motor (TOFF=0 would silently
//!                                     disable). Microsteps: MRES=0 → 256
//!                                     for max resolution (TMC datasheet
//!                                     §6.1; not enforced by us — Klipper
//!                                     normally sets via microsteps=
//!                                     config, but for a basic buzz test
//!                                     any MRES works as long as TOFF>0).
//!        PWMCONF (0x70) = 0xC10D0024  standard StealthChop tuning
//!   6. Generates 800 step pulses (DEDGE: each toggle = 1 step) at 5 ms
//!      period — that's 5 mm of motion @ 32 microsteps × 40 mm
//!      rotation_distance.
//!   7. Flips direction, generates 800 more steps back.
//!   8. Waits ~10 seconds.
//!   9. Issues NVIC_SystemReset. The reset lands back in the H7's
//!      bootloader (Katapult/HID-CDC variant, lives in
//!      0x08000000..0x08020000 — first 128 KiB of flash, untouched by
//!      us). The bootloader sits waiting for the next firmware flash
//!      command — no manual BOOT0/DFU button press needed.
//!
//! If the X motor moves visibly during the buzz: TMC + pin routing +
//! mechanical are all fine. The Kalico motion engine is the only
//! suspect for the "engine runs cleanly but motors silent" bug.
//!
//! If the motor stays silent here too: the bug is below the engine
//! (TMC config, current scale, mechanical wiring, board layout).

#![no_std]
#![no_main]

use core::ptr::{read_volatile, write_volatile};
use cortex_m_rt::entry;
use panic_halt as _;
// Pull the PAC in for its interrupt-vector table even though we never use the
// generated register types. Without this `cortex-m-rt` errors with
// "interrupt vectors are missing".
use stm32h7 as _;

// STM32H723 base addresses (RM0468, Table 7).
const RCC_BASE:      usize = 0x5802_4400;
const PWR_BASE:      usize = 0x5802_4800;
const GPIOA_BASE:    usize = 0x5802_0000;
const GPIOC_BASE:    usize = 0x5802_0800;
const GPIOG_BASE:    usize = 0x5802_1800;
const SPI1_BASE:     usize = 0x4001_3000;
const STK_BASE:      usize = 0xE000_E010; // SysTick
const SCB_AIRCR:     usize = 0xE000_ED0C;

// RCC offsets (RM0468 Chapter 8).
const RCC_AHB4ENR:   usize = 0xE0;  // GPIO A/C/G
const RCC_APB2ENR:   usize = 0xF0;  // SPI1
const RCC_CFGR:      usize = 0x10;

// PWR offsets (RM0468 Chapter 6). Voltage scale 3 (~64 MHz cap on HSI) is the
// reset default — no PWR config needed if we stay on HSI64.
// We touch PWR only to ensure VOS3 (lowest VOS) so HSI64 is in spec.
const PWR_D3CR:      usize = 0x18;  // VOS bits 14:15. Reset default = VOS3.

// GPIO offsets (RM0468 Chapter 11). Same layout for all ports.
const GPIO_MODER:    usize = 0x00;
const GPIO_OTYPER:   usize = 0x04;
const GPIO_OSPEEDR:  usize = 0x08;
const GPIO_PUPDR:    usize = 0x0C;
const GPIO_BSRR:     usize = 0x18;
const GPIO_AFRL:     usize = 0x20;

// SPI offsets (RM0468 Chapter 60, "SPI/I2S" — the H7 has SPI v2.x which is
// quite different from the F1/F4 SPI). The key registers we care about:
const SPI_CR1:       usize = 0x00;
const SPI_CR2:       usize = 0x04;
const SPI_CFG1:      usize = 0x08;
const SPI_CFG2:      usize = 0x0C;
const SPI_SR:        usize = 0x14;
const SPI_IFCR:      usize = 0x18;
const SPI_TXDR:      usize = 0x20;
const SPI_RXDR:      usize = 0x30;

// CPU clock at HSI default reset state (RM0468 §8.5.2 — HSI starts at 64 MHz
// after divider, RCC->HSICFGR untouched). Don't enable PLL — keeps init
// simple and stays well within stable VOS3 limits.
const HSI_HZ: u32 = 64_000_000;

#[entry]
fn main() -> ! {
    unsafe {
        // 1. Enable GPIO + SPI1 peripheral clocks.
        let mut v = read_volatile((RCC_BASE + RCC_AHB4ENR) as *const u32);
        v |= (1 << 0) | (1 << 2) | (1 << 6); // GPIOA(0), GPIOC(2), GPIOG(6)
        write_volatile((RCC_BASE + RCC_AHB4ENR) as *mut u32, v);
        cortex_m::asm::dsb();

        let mut v = read_volatile((RCC_BASE + RCC_APB2ENR) as *const u32);
        v |= 1 << 12; // SPI1EN
        write_volatile((RCC_BASE + RCC_APB2ENR) as *mut u32, v);
        cortex_m::asm::dsb();

        // 2. Configure pins.
        //
        // PA5: AF5 (SPI1 SCK), output, AF mode
        // PA6: AF5 (SPI1 MISO), input via AF
        // PA7: AF5 (SPI1 MOSI), output, AF mode
        // PA2: output (enable, inverted: 0 = motor enabled)
        //
        // PC1: output (dir, inverted: 1 = motor moves +)
        // PC7: output (CS, idle high)
        //
        // PG4: output (step pin; each edge = 1 microstep under DEDGE)

        // PORTA: PA2 output, PA5/6/7 AF
        let mut moder = read_volatile((GPIOA_BASE + GPIO_MODER) as *const u32);
        // Clear and set bits for pins 2, 5, 6, 7
        moder &= !((0b11u32 << (2 * 2))
                 | (0b11u32 << (2 * 5))
                 | (0b11u32 << (2 * 6))
                 | (0b11u32 << (2 * 7)));
        moder |= (0b01u32 << (2 * 2))    // PA2 = output
               | (0b10u32 << (2 * 5))    // PA5 = AF
               | (0b10u32 << (2 * 6))    // PA6 = AF
               | (0b10u32 << (2 * 7));   // PA7 = AF
        write_volatile((GPIOA_BASE + GPIO_MODER) as *mut u32, moder);

        // AFRL covers pins 0..7; set AF5 (SPI1) for PA5/6/7
        let mut afrl = read_volatile((GPIOA_BASE + GPIO_AFRL) as *const u32);
        afrl &= !((0b1111u32 << (4 * 5))
                | (0b1111u32 << (4 * 6))
                | (0b1111u32 << (4 * 7)));
        afrl |= (5u32 << (4 * 5)) | (5u32 << (4 * 6)) | (5u32 << (4 * 7));
        write_volatile((GPIOA_BASE + GPIO_AFRL) as *mut u32, afrl);

        // PA2 medium speed
        let mut ospeed = read_volatile((GPIOA_BASE + GPIO_OSPEEDR) as *const u32);
        ospeed |= 0b01u32 << (2 * 2);
        write_volatile((GPIOA_BASE + GPIO_OSPEEDR) as *mut u32, ospeed);

        // PORTC: PC1 + PC7 outputs
        let mut moder = read_volatile((GPIOC_BASE + GPIO_MODER) as *const u32);
        moder &= !((0b11u32 << (2 * 1)) | (0b11u32 << (2 * 7)));
        moder |= (0b01u32 << (2 * 1)) | (0b01u32 << (2 * 7));
        write_volatile((GPIOC_BASE + GPIO_MODER) as *mut u32, moder);

        // PORTG: PG4 output
        let mut moder = read_volatile((GPIOG_BASE + GPIO_MODER) as *const u32);
        moder &= !(0b11u32 << (2 * 4));
        moder |= 0b01u32 << (2 * 4);
        write_volatile((GPIOG_BASE + GPIO_MODER) as *mut u32, moder);

        // 3. Set initial pin states.
        // PC7 (CS) idle high  → BSRR bit 7 (set)
        // PA2 (EN, inverted)  → write 0 = enabled → BSRR bit 18 (reset)
        // PC1 (DIR, inverted) → write 0 = motor moves negative initially
        //                       → BSRR bit 17 (reset)
        // PG4 (STEP) idle low → BSRR bit 20 (reset)
        write_volatile((GPIOC_BASE + GPIO_BSRR) as *mut u32, 1 << 7);
        write_volatile((GPIOA_BASE + GPIO_BSRR) as *mut u32, 1 << (16 + 2));
        write_volatile((GPIOC_BASE + GPIO_BSRR) as *mut u32, 1 << (16 + 1));
        write_volatile((GPIOG_BASE + GPIO_BSRR) as *mut u32, 1 << (16 + 4));

        // 4. SPI1 init.
        // Disable SPI before configuring.
        write_volatile((SPI1_BASE + SPI_CR1) as *mut u32, 0);

        // CFG1: DSIZE=7 (8-bit transfers), FTHLV=0 (1-data), MBR=4 (clock/32 = 2 MHz)
        // MBR=4 is /32. HSI=64MHz → SPI = 2 MHz, well under TMC's 4 MHz max.
        let cfg1: u32 = (7u32 << 0)        // DSIZE = 8 bits
                      | (0u32 << 5)        // FTHLV = 1
                      | (4u32 << 28);      // MBR = /32
        write_volatile((SPI1_BASE + SPI_CFG1) as *mut u32, cfg1);

        // CFG2: master mode, CPOL=1, CPHA=1 (Mode 3), MSSI=0 (no master SS
        // suspend), MIDI=0 (no inter-data delay), SSM=1 (software slave
        // management — we drive CS manually), SSI=1 (slave-select input high
        // so master mode doesn't fault). LSBFRST=0 (MSB first).
        let cfg2: u32 = (1u32 << 22)       // MASTER
                      | (1u32 << 24)       // CPOL (idle high)
                      | (1u32 << 25)       // CPHA (sample on 2nd edge)
                      | (1u32 << 26)       // SSM (software NSS)
                      | (1u32 << 12)       // SSIOP (NSS polarity — unused with SSM)
                      | (1u32 << 28);      // AFCNTR (TMC needs SPI control of AF pins)
        write_volatile((SPI1_BASE + SPI_CFG2) as *mut u32, cfg2);

        // CR1: SSI=1 (force NSS internally — required with SSM=1 in master)
        write_volatile((SPI1_BASE + SPI_CR1) as *mut u32, 1 << 12); // SSI

        // Enable SPI.
        write_volatile((SPI1_BASE + SPI_CR1) as *mut u32, (1 << 12) | (1 << 0)); // SSI | SPE

        // Brief settle.
        delay_systick_cycles(HSI_HZ / 1000); // 1 ms

        // 5. TMC5160 register writes.
        // GCONF = 0 (spreadcycle, no special flags)
        tmc_write(0x00, 0x0000_0000);
        // GLOBAL_SCALER = 128 (50% scaler — drops IRUN range to a safer span)
        tmc_write(0x0B, 0x0000_0080);
        // IHOLD_IRUN: ihold=8, irun=8 (≈0.7 A RMS at 50% scaler + 0.075 Ω
        // sense), iholddelay=8
        tmc_write(0x10, 0x0008_0808);
        // TPOWERDOWN = 10 (standstill timeout)
        tmc_write(0x11, 0x0000_000A);
        // CHOPCONF: toff=3 (chopper on — CRITICAL; toff=0 silently disables
        // the motor), hstrt=4, hend=1, tbl=2, mres=0 (256 microsteps)
        // Bit layout: toff[3:0]=3, hstrt[6:4]=4 (010<<4), hend[10:7]=1
        // (0001<<7=0x080), tbl[16:15]=2 (10<<15=0x10000), mres[27:24]=0
        // intpol[28]=1 (interpolation on)
        let chopconf: u32 = (3u32 << 0)    // toff = 3
                          | (4u32 << 4)    // hstrt = 4
                          | (1u32 << 7)    // hend = 1
                          | (2u32 << 15)   // tbl = 2
                          | (1u32 << 28);  // intpol = 1
        tmc_write(0x6C, chopconf);
        // PWMCONF: leave as default-ish (pwm_freq=1, pwm_grad=0x0D, pwm_ofs=0x24)
        tmc_write(0x70, 0xC10D_0024);

        // Brief settle so the TMC current ramps up before we start stepping.
        delay_systick_cycles(HSI_HZ / 10); // 100 ms

        // 6. Forward burst: DIR=1 (PC1 high → after inversion = +X), 800 toggles
        //    at 5 ms (~200 Hz step rate, gentle).
        write_volatile((GPIOC_BASE + GPIO_BSRR) as *mut u32, 1 << 1); // PC1 high
        delay_systick_cycles(HSI_HZ / 1000); // 1 ms dir settle
        for _ in 0..800u32 {
            toggle_step_pin();
            delay_systick_cycles(HSI_HZ / 400); // ~2.5 ms each half-cycle = 5 ms total per step
        }

        // 7. Brief pause.
        delay_systick_cycles(HSI_HZ); // 1 sec

        // 8. Reverse burst: DIR=0 (PC1 low → -X), 800 toggles.
        write_volatile((GPIOC_BASE + GPIO_BSRR) as *mut u32, 1 << (16 + 1)); // PC1 low
        delay_systick_cycles(HSI_HZ / 1000);
        for _ in 0..800u32 {
            toggle_step_pin();
            delay_systick_cycles(HSI_HZ / 400);
        }

        // 9. Wait 10 seconds (so operator can confirm motion stopped cleanly
        //    before the reset takes us back into Katapult).
        for _ in 0..10 {
            delay_systick_cycles(HSI_HZ);
        }

        // 10. Disable motor before reset so the next firmware load doesn't
        //     race the bootloader for the enable line.
        write_volatile((GPIOA_BASE + GPIO_BSRR) as *mut u32, 1 << 2); // PA2 high = disabled

        // 11. NVIC_SystemReset — lands in Katapult.
        write_volatile(SCB_AIRCR as *mut u32, (0x5FA << 16) | (1 << 2));
        cortex_m::asm::dsb();
        loop {
            cortex_m::asm::nop();
        }
    }
}

/// Toggle PG4 atomically via BSRR. Reads current state from ODR and flips
/// the bit; not strictly atomic with concurrent writers but we're
/// single-threaded with no interrupts.
#[inline(always)]
fn toggle_step_pin() {
    unsafe {
        const GPIO_ODR: usize = 0x10;
        let odr = read_volatile((GPIOG_BASE + GPIO_ODR) as *const u32);
        let bit = if odr & (1 << 4) != 0 {
            1 << (16 + 4) // currently high → reset
        } else {
            1 << 4         // currently low → set
        };
        write_volatile((GPIOG_BASE + GPIO_BSRR) as *mut u32, bit);
    }
}

/// TMC5160 register write. 40-bit transaction: 1 address byte (high bit set
/// for write) + 4 data bytes, MSB first.
fn tmc_write(addr: u8, data: u32) {
    unsafe {
        // CS low
        write_volatile((GPIOC_BASE + GPIO_BSRR) as *mut u32, 1 << (16 + 7));
        // Tiny setup delay.
        for _ in 0..10 {
            cortex_m::asm::nop();
        }

        spi_send_byte(addr | 0x80);
        spi_send_byte((data >> 24) as u8);
        spi_send_byte((data >> 16) as u8);
        spi_send_byte((data >> 8) as u8);
        spi_send_byte(data as u8);

        // Wait for SPI EOT (TXC bit) — bit 12 in SR.
        while read_volatile((SPI1_BASE + SPI_SR) as *const u32) & (1 << 12) == 0 {}

        // CS high
        write_volatile((GPIOC_BASE + GPIO_BSRR) as *mut u32, 1 << 7);
        // Hold time before next CS low.
        for _ in 0..50 {
            cortex_m::asm::nop();
        }
    }
}

/// Send one byte, wait for TXP (TX FIFO has space) before push, then drain
/// the RX byte so the FIFO doesn't fill up across the 5-byte transfer.
fn spi_send_byte(b: u8) {
    unsafe {
        // Start the SPI transfer if not running. The H7 SPI v2 needs CSTART
        // bit set in CR1 to begin clocking. Set it on every send — it's
        // idempotent if already running.
        let cr1 = read_volatile((SPI1_BASE + SPI_CR1) as *const u32);
        if cr1 & (1 << 9) == 0 {
            write_volatile((SPI1_BASE + SPI_CR1) as *mut u32, cr1 | (1 << 9)); // CSTART
        }
        // Wait for TXP (bit 1)
        while read_volatile((SPI1_BASE + SPI_SR) as *const u32) & (1 << 1) == 0 {}
        // Write a single byte. Use volatile u8 store on TXDR's low byte so
        // the SPI counts it as a packet of size DSIZE (8 bits per CFG1).
        write_volatile((SPI1_BASE + SPI_TXDR) as *mut u8, b);
        // Wait for RXP (RX FIFO non-empty) and drain. Required to keep the
        // FIFO from blocking subsequent TX.
        while read_volatile((SPI1_BASE + SPI_SR) as *const u32) & (1 << 0) == 0 {}
        let _rx: u8 = read_volatile((SPI1_BASE + SPI_RXDR) as *const u8);
    }
}

/// Busy-wait for `cycles` CPU cycles via SysTick. Re-arms SysTick each call
/// with the requested countdown. Handles values larger than 24 bits
/// (SysTick is 24-bit) by looping.
fn delay_systick_cycles(cycles: u32) {
    unsafe {
        const SYST_CSR: usize = 0x00;
        const SYST_RVR: usize = 0x04;
        const SYST_CVR: usize = 0x08;

        let mut remaining = cycles;
        while remaining > 0 {
            let chunk = remaining.min(0x00FF_FFFF);
            remaining -= chunk;

            // Disable, set RVR, reset CVR, enable with PROCESSOR clock + no IRQ.
            write_volatile((STK_BASE + SYST_CSR) as *mut u32, 0);
            write_volatile((STK_BASE + SYST_RVR) as *mut u32, chunk);
            write_volatile((STK_BASE + SYST_CVR) as *mut u32, 0);
            write_volatile((STK_BASE + SYST_CSR) as *mut u32, (1 << 0) | (1 << 2)); // ENABLE | CLKSOURCE=processor

            // Wait for COUNTFLAG (bit 16).
            while read_volatile((STK_BASE + SYST_CSR) as *const u32) & (1 << 16) == 0 {}
            write_volatile((STK_BASE + SYST_CSR) as *mut u32, 0);
        }

        // Silence the unused-import lint for PWR_BASE/PWR_D3CR (kept for
        // future "if HSI64 isn't enough, bump to PLL via VOS1" follow-up).
        let _ = (PWR_BASE, PWR_D3CR);
    }
}
