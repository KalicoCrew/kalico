// Generic ARM Cortex-M linker script
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_FLASH_APPLICATION_ADDRESS

OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)

MEMORY
{
  rom (rx) : ORIGIN = CONFIG_FLASH_APPLICATION_ADDRESS , LENGTH = CONFIG_FLASH_SIZE
  ram (rwx) : ORIGIN = CONFIG_RAM_START , LENGTH = CONFIG_RAM_SIZE
#if CONFIG_MACH_STM32H7
  // STM32H7 AXI SRAM (D1 domain). 320 KB on H723; clock is always-on after
  // reset. Used to host large statics (notably the kalico Rust runtime's
  // RT_CELL curve pool) that don't fit in the 128 KB DTCM `ram` region.
  axi_ram (rwx) : ORIGIN = 0x24000000 , LENGTH = 320K
  // Backup SRAM (D3 domain). 4 KB at 0x38800000. Survives any system
  // reset (IWDG, software, NRST) and even VBAT-only — the only thing
  // that wipes it is full power loss / POR with VBAT disconnected.
  // Used by the diagnostic fault handler to record CPU-fault details
  // and runtime-engine snapshots so we can read out what state the
  // firmware was in just before a watchdog-triggered reset.
  // Requires runtime enable: PWR->CR2 |= BREN, RCC->AHB4ENR |= BKPRAMEN
  // (see generic/fault_handler.c::fault_handler_init).
  bkp_ram (rwx) : ORIGIN = 0x38800000 , LENGTH = 4K
#endif
}

SECTIONS
{
    .text : {
        . = ALIGN(4);
        _text_vectortable_start = .;
        KEEP(*(.vector_table))
        _text_vectortable_end = .;
        *(.text .text.*)
        *(.rodata .rodata*)
    } > rom

    . = ALIGN(4);
    _data_flash = .;

#if CONFIG_ARMCM_RAM_VECTORTABLE
    .ram_vectortable (NOLOAD) : {
        _ram_vectortable_start = .;
        . = . + ( _text_vectortable_end - _text_vectortable_start ) ;
        _ram_vectortable_end = .;
    } > ram
#endif

    .data : AT (_data_flash)
    {
        . = ALIGN(4);
        _data_start = .;
        *(.ramfunc .ramfunc.*);
        *(.data .data.*);
        . = ALIGN(4);
        _data_end = .;
    } > ram

    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        _bss_start = .;
        *(.bss .bss.*)
        *(COMMON)
        . = ALIGN(4);
        _bss_end = .;
    } > ram

#if CONFIG_MACH_STM32H7
    // Symbols / statics tagged `__attribute__((section(".axi_bss")))` (C) or
    // `#[link_section = ".axi_bss"]` (Rust) are placed in AXI SRAM rather
    // than DTCM. NOLOAD because these are uninitialised (BSS-style) and we
    // do not want them in the flash image.
    .axi_bss (NOLOAD) : {
        . = ALIGN(8);
        _axi_bss_start = .;
        *(.axi_bss .axi_bss.*)
        . = ALIGN(8);
        _axi_bss_end = .;
    } > axi_ram

    // Backup-SRAM-resident statics. NOLOAD because the bootloader
    // doesn't initialize this region; we rely on prior contents
    // surviving the reset (or detect uninitialised state via a magic).
    .bkp_bss (NOLOAD) : {
        . = ALIGN(4);
        _bkp_bss_start = .;
        *(.bkp_bss .bkp_bss.*)
        . = ALIGN(4);
        _bkp_bss_end = .;
    } > bkp_ram
#endif

    _stack_start = CONFIG_RAM_START + CONFIG_RAM_SIZE - CONFIG_STACK_SIZE ;
    .stack _stack_start (NOLOAD) :
    {
        . = . + CONFIG_STACK_SIZE;
        _stack_end = .;
    } > ram

    /DISCARD/ : {
        // The .init/.fini sections are used by __libc_init_array(), but
        // that isn't needed so no need to include them in the binary.
        *(.init)
        *(.fini)
        // Don't include exception tables
        *(.ARM.extab)
        *(.ARM.exidx)
    }
}
