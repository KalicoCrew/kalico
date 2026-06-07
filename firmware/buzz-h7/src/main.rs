#![no_std]
#![no_main]

use core::ptr::{read_volatile, write_volatile};
use cortex_m_rt::entry;
use panic_halt as _;
// Pull the PAC in for its interrupt-vector table even though we never use the
// generated register types. Without this `cortex-m-rt` errors with
// "interrupt vectors are missing".
use stm32h7 as _;

const RCC_BASE:      usize = 0x5802_4400;
const PWR_BASE:      usize = 0x5802_4800;
const GPIOA_BASE:    usize = 0x5802_0000;
const GPIOC_BASE:    usize = 0x5802_0800;
const GPIOG_BASE:    usize = 0x5802_1800;
const SPI1_BASE:     usize = 0x4001_3000;
const STK_BASE:      usize = 0xE000_E010; // SysTick
const SCB_AIRCR:     usize = 0xE000_ED0C;

// STM32H723 ROM DFU bootloader entry point (AN2606 Rev 50, Table 150;
// RM0468 §2.3 "System memory" — STM32H72x/H73x system memory bank at
// 0x1FF0_9800). The vector table at this address follows the standard ARM
// Cortex-M convention: word 0 = initial MSP, word 1 = reset handler.
const ROM_BOOTLOADER_BASE: usize = 0x1FF0_9800;

const RCC_AHB4ENR:   usize = 0xE0;
const RCC_APB2ENR:   usize = 0xF0;
const RCC_CFGR:      usize = 0x10;

const PWR_D3CR:      usize = 0x18;

const GPIO_MODER:    usize = 0x00;
const GPIO_OTYPER:   usize = 0x04;
const GPIO_OSPEEDR:  usize = 0x08;
const GPIO_PUPDR:    usize = 0x0C;
const GPIO_BSRR:     usize = 0x18;
const GPIO_AFRL:     usize = 0x20;

const SPI_CR1:       usize = 0x00;
const SPI_CR2:       usize = 0x04;
const SPI_CFG1:      usize = 0x08;
const SPI_CFG2:      usize = 0x0C;
const SPI_SR:        usize = 0x14;
const SPI_IFCR:      usize = 0x18;
const SPI_TXDR:      usize = 0x20;
const SPI_RXDR:      usize = 0x30;

const HSI_HZ: u32 = 64_000_000;

#[entry]
fn main() -> ! {
    unsafe {
        let mut v = read_volatile((RCC_BASE + RCC_AHB4ENR) as *const u32);
        v |= (1 << 0) | (1 << 2) | (1 << 6); // GPIOA(0), GPIOC(2), GPIOG(6)
        write_volatile((RCC_BASE + RCC_AHB4ENR) as *mut u32, v);
        cortex_m::asm::dsb();

        let mut v = read_volatile((RCC_BASE + RCC_APB2ENR) as *const u32);
        v |= 1 << 12; // SPI1EN
        write_volatile((RCC_BASE + RCC_APB2ENR) as *mut u32, v);
        cortex_m::asm::dsb();

        let mut moder = read_volatile((GPIOA_BASE + GPIO_MODER) as *const u32);
        moder &= !((0b11u32 << (2 * 2))
                 | (0b11u32 << (2 * 5))
                 | (0b11u32 << (2 * 6))
                 | (0b11u32 << (2 * 7)));
        moder |= (0b01u32 << (2 * 2))    // PA2 = output
               | (0b10u32 << (2 * 5))    // PA5 = AF
               | (0b10u32 << (2 * 6))    // PA6 = AF
               | (0b10u32 << (2 * 7));   // PA7 = AF
        write_volatile((GPIOA_BASE + GPIO_MODER) as *mut u32, moder);

        let mut afrl = read_volatile((GPIOA_BASE + GPIO_AFRL) as *const u32);
        afrl &= !((0b1111u32 << (4 * 5))
                | (0b1111u32 << (4 * 6))
                | (0b1111u32 << (4 * 7)));
        afrl |= (5u32 << (4 * 5)) | (5u32 << (4 * 6)) | (5u32 << (4 * 7));
        write_volatile((GPIOA_BASE + GPIO_AFRL) as *mut u32, afrl);

        let mut ospeed = read_volatile((GPIOA_BASE + GPIO_OSPEEDR) as *const u32);
        ospeed |= 0b01u32 << (2 * 2);
        write_volatile((GPIOA_BASE + GPIO_OSPEEDR) as *mut u32, ospeed);

        let mut moder = read_volatile((GPIOC_BASE + GPIO_MODER) as *const u32);
        moder &= !((0b11u32 << (2 * 1)) | (0b11u32 << (2 * 7)));
        moder |= (0b01u32 << (2 * 1)) | (0b01u32 << (2 * 7));
        write_volatile((GPIOC_BASE + GPIO_MODER) as *mut u32, moder);

        let mut moder = read_volatile((GPIOG_BASE + GPIO_MODER) as *const u32);
        moder &= !(0b11u32 << (2 * 4));
        moder |= 0b01u32 << (2 * 4);
        write_volatile((GPIOG_BASE + GPIO_MODER) as *mut u32, moder);

        // PC7 (CS) idle high  → BSRR bit 7 (set)
        // PA2 (EN, inverted)  → write 0 = enabled → BSRR bit 18 (reset)
        // PC1 (DIR, inverted) → write 0 = motor moves negative initially
        //                       → BSRR bit 17 (reset)
        // PG4 (STEP) idle low → BSRR bit 20 (reset)
        write_volatile((GPIOC_BASE + GPIO_BSRR) as *mut u32, 1 << 7);
        write_volatile((GPIOA_BASE + GPIO_BSRR) as *mut u32, 1 << (16 + 2));
        write_volatile((GPIOC_BASE + GPIO_BSRR) as *mut u32, 1 << (16 + 1));
        write_volatile((GPIOG_BASE + GPIO_BSRR) as *mut u32, 1 << (16 + 4));

        write_volatile((SPI1_BASE + SPI_CR1) as *mut u32, 0);

        let cfg1: u32 = (7u32 << 0)        // DSIZE = 8 bits
                      | (0u32 << 5)        // FTHLV = 1
                      | (4u32 << 28);      // MBR = /32
        write_volatile((SPI1_BASE + SPI_CFG1) as *mut u32, cfg1);

        // CFG2: master mode, CPOL=1, CPHA=1 (Mode 3), SSM=1 (software slave
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
        write_volatile((SPI1_BASE + SPI_CR1) as *mut u32, 1 << 12);

        write_volatile((SPI1_BASE + SPI_CR1) as *mut u32, (1 << 12) | (1 << 0)); // SSI | SPE

        delay_systick_cycles(HSI_HZ / 1000);

        if false {
        tmc_write(0x00, 0x0000_000C);
        tmc_write(0x10, 0x0006_1A1A);
        tmc_write(0x11, 0x0000_000A);
        tmc_write(0x13, 0x000F_FFFF);
        tmc_write(0x6C, 0x3370_0378);
        tmc_write(0x70, 0xC40C_001E);
        }

        delay_systick_cycles(HSI_HZ / 10);

        write_volatile((GPIOC_BASE + GPIO_BSRR) as *mut u32, 1 << 1); // PC1 high
        delay_systick_cycles(HSI_HZ / 1000); // 1 ms dir settle
        for _ in 0..800u32 {
            toggle_step_pin();
            delay_systick_cycles(HSI_HZ / 400); // ~2.5 ms each half-cycle = 5 ms total per step
        }

        delay_systick_cycles(HSI_HZ);

        write_volatile((GPIOC_BASE + GPIO_BSRR) as *mut u32, 1 << (16 + 1)); // PC1 low
        delay_systick_cycles(HSI_HZ / 1000);
        for _ in 0..800u32 {
            toggle_step_pin();
            delay_systick_cycles(HSI_HZ / 400);
        }

        for _ in 0..10 {
            delay_systick_cycles(HSI_HZ);
        }

        write_volatile((GPIOA_BASE + GPIO_BSRR) as *mut u32, 1 << 2); // PA2 high = disabled

        jump_to_dfu_bootloader();
    }
}

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

fn tmc_write(addr: u8, data: u32) {
    unsafe {
        write_volatile((GPIOC_BASE + GPIO_BSRR) as *mut u32, 1 << (16 + 7));
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

        write_volatile((GPIOC_BASE + GPIO_BSRR) as *mut u32, 1 << 7);
        // Hold time before next CS low.
        for _ in 0..50 {
            cortex_m::asm::nop();
        }
    }
}

fn spi_send_byte(b: u8) {
    unsafe {
        // Start the SPI transfer if not running. The H7 SPI v2 needs CSTART
        // bit set in CR1 to begin clocking. Set it on every send — it's
        // idempotent if already running.
        let cr1 = read_volatile((SPI1_BASE + SPI_CR1) as *const u32);
        if cr1 & (1 << 9) == 0 {
            write_volatile((SPI1_BASE + SPI_CR1) as *mut u32, cr1 | (1 << 9));
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

fn delay_systick_cycles(cycles: u32) {
    unsafe {
        const SYST_CSR: usize = 0x00;
        const SYST_RVR: usize = 0x04;
        const SYST_CVR: usize = 0x08;

        let mut remaining = cycles;
        while remaining > 0 {
            let chunk = remaining.min(0x00FF_FFFF);
            remaining -= chunk;

            write_volatile((STK_BASE + SYST_CSR) as *mut u32, 0);
            write_volatile((STK_BASE + SYST_RVR) as *mut u32, chunk);
            write_volatile((STK_BASE + SYST_CVR) as *mut u32, 0);
            write_volatile((STK_BASE + SYST_CSR) as *mut u32, (1 << 0) | (1 << 2)); // ENABLE | CLKSOURCE=processor

            // Wait for COUNTFLAG (bit 16).
            while read_volatile((STK_BASE + SYST_CSR) as *const u32) & (1 << 16) == 0 {}
            write_volatile((STK_BASE + SYST_CSR) as *mut u32, 0);
        }

        let _ = (PWR_BASE, PWR_D3CR);
    }
}

/// Jump to the STM32H723 ROM DFU bootloader at 0x1FF09800.
///
/// Follows the standard Cortex-M "boot from system memory" recipe with the
/// H7-specific additions documented in AN2606 Rev 50 §33 and RM0468 §2.3:
///
/// 1. Mask all interrupts (CPSID I) so no peripheral IRQ fires mid-sequence.
/// 2. Disable SysTick — its COUNTFLAG fires can confuse the bootloader's own
///    timer init if we leave it counting.
/// 3. Disable SPI1 — clear SPE so the peripheral releases the AF pins cleanly.
///    The bootloader doesn't touch SPI1, but a running SPI with an active FIFO
///    can drive AF pins unexpectedly while MODER is still set to AF mode.
/// 4. Reset all GPIO MODER to the power-on default (0xABFF_FFFF for PORTA,
///    0xFFFF_FFFF for PORTC/G). This puts PA5/6/7 back to analog (input),
///    so the bootloader's USB PA11/PA12 init doesn't fight our SPI AF setting.
///    We do NOT reset the full RCC or SYSCFG — the ROM bootloader handles
///    the USB-HS / USB-FS PLL and clock gating internally.
/// 5. Load initial MSP from ROM_BOOTLOADER_BASE+0 (standard ARM vector table
///    layout: word 0 = SP_main, word 1 = Reset_Handler).
/// 6. Set MSP via `msr msp`.
/// 7. Call the bootloader's reset vector (loaded from ROM_BOOTLOADER_BASE+4).
///
/// We do NOT remap system memory to address 0 via SYSCFG. The H7 SYSCFG
/// memory-remap register (SYSCFG_UR0) is OTP-style and its "BOOT_ADD0/1"
/// fields set the Cortex-M boot alias, but they are write-once and irrelevant
/// here — we are *calling* the bootloader as a function, not rebooting into it
/// via the boot alias. The ROM bootloader runs fine from 0x1FF09800 when called
/// this way (confirmed in AN2606 §33.2 "Bootloader activation" for H72x/H73x).
///
/// After this function executes, the ROM bootloader initializes USB OTG-FS
/// (PA11=DM, PA12=DP on H723) and enumerates as VID:PID 0483:df11 (DFU mode).
/// `dfu-util -d 0483:df11 -a 0 -s 0x08020000:leave -D buzz-h7.bin` will then
/// flash directly — no BOOT0 pin or physical reset needed.
#[inline(never)]
fn jump_to_dfu_bootloader() -> ! {
    unsafe {
        cortex_m::interrupt::disable();

        const SYST_CSR: usize = 0x00;
        write_volatile((STK_BASE + SYST_CSR) as *mut u32, 0);

        write_volatile((SPI1_BASE + SPI_CR1) as *mut u32, 0);

        // Step 4: reset GPIO MODER to power-on defaults so the bootloader
        // can reinitialize USB pins (PA11 = D-, PA12 = D+) without fighting
        // our AF configuration. RM0468 §11.4.1 "GPIOx_MODER" reset values:
        //   PORTA: 0xABFF_FFFF (PA15/PA14/PA13 kept as debug AF at reset)
        //   PORTB: 0x0000_0280 (PB3/PB4 have pull-ups; not our business)
        //   PORTC/G: 0xFFFF_FFFF (all analog = input, no drive)
        // Writing these values un-does everything we did in step 2 of main.
        write_volatile((GPIOA_BASE + GPIO_MODER) as *mut u32, 0xABFF_FFFF);
        write_volatile((GPIOC_BASE + GPIO_MODER) as *mut u32, 0xFFFF_FFFF);
        write_volatile((GPIOG_BASE + GPIO_MODER) as *mut u32, 0xFFFF_FFFF);

        // Point the vector table at the bootloader so its IRQs (USB OTG_FS in
        // particular) dispatch to ROM handlers instead of our app's vector
        // table at 0x08020000. Without this, the bootloader enumerates briefly,
        // takes a USB IRQ, lands in our vectors which are now meaningless after
        // the jump, and the device falls off USB.
        // SCB->VTOR is at 0xE000_ED08 (ARM DDI 0403E §B3.2.5).
        const SCB_VTOR: usize = 0xE000_ED08;
        write_volatile(SCB_VTOR as *mut u32, ROM_BOOTLOADER_BASE as u32);

        // Memory barriers: ensure all peripheral writes above retire and
        // the VTOR update is observed before we transfer control. Cortex-M7
        // needs both DSB (data) and ISB (instruction) — the next fetch must
        // see the new vector table mapping. Skipping these is the classic
        // "works in debugger, fails at full speed" trap on M7.
        core::arch::asm!("dsb", "isb", options(nomem, nostack, preserves_flags));

        // Load MSP + PC from bootloader vector table. Standard Cortex-M layout
        // (ARM DDI 0403E §B1.5.3): [0x00] = initial MSP, [0x04] = reset vector.
        let bootloader_sp: u32 = read_volatile(ROM_BOOTLOADER_BASE as *const u32);
        let bootloader_pc: u32 = read_volatile((ROM_BOOTLOADER_BASE + 4) as *const u32);

        core::arch::asm!(
            "msr msp, {sp}",
            sp = in(reg) bootloader_sp,
            options(nomem, nostack),
        );

        // Branch to the bootloader's reset handler. Cast to a function pointer
        // with the Thumb bit cleared — the LSB is conventionally set in
        // Cortex-M vectors to indicate Thumb mode, but BLX/BX handles that.
        // We clear bit 0 for the raw address and use `bx` semantics via the
        // function pointer call, which the compiler emits as BLX — correct
        // for Thumb2. The function pointer is declared -> ! so the compiler
        // knows control never returns.
        let bootloader_entry: extern "C" fn() -> ! =
            core::mem::transmute(bootloader_pc as usize);
        bootloader_entry();
    }
}
