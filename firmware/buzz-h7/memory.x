/* STM32H723 (BTT Octopus Pro). Bootloader (Katapult for H7 / HID-CDC variant)
 * occupies 0x08000000..0x08020000 (128 KiB). User's printer.cfg confirms via
 * CONFIG_FLASH_APPLICATION_ADDRESS=0x8020000 / CONFIG_STM32_FLASH_START_20000=y.
 * Our app must link to that same offset so the bootloader's jump-to-app lands
 * cleanly here, and so NVIC_SystemReset at end-of-buzz lands back in the
 * bootloader (which sits in the lower 128 KiB and is untouched).
 *
 * RAM: 320 KiB AXI-SRAM at 0x24000000. We only need a few hundred bytes of
 * stack so this is wildly oversized; using the largest available region keeps
 * the linker math trivial. */
MEMORY
{
    FLASH : ORIGIN = 0x08020000, LENGTH = 896K
    RAM   : ORIGIN = 0x24000000, LENGTH = 320K
}

_stack_start = ORIGIN(RAM) + LENGTH(RAM);
