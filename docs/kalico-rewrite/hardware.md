# Target hardware

- A rigid machine with single spike on each axis resonance graph. 120hz on Y and 180hz on X
- With regular klipper it could achieve motion up to 1000mm/s and 65k acceleration with 65scv before skipping steps.
- Extruder could achieve roughly 50k with acceptable pressure advance before acceleration becomes too high.
- Max flow of about 80mm cubic.
- Host: Pi 5
- MCU1: Octopus Pro with H723, 4 5160 drivers for AB steppers, 1 more 5160 for extruder
- MCU2: Octopus with F4x chip, 2209 for Z

# Secondary bench — Neptune 3 Pro (`dderg@ethercatpi5.local`)

Host Pi reached at `ethercatpi5.local` (repurposed from an EtherCAT setup; no
EtherCAT attached at the moment — it drives the Neptune over USB serial).

- Elegoo Neptune 3 Pro bedslinger, ZNP Robin Nano DW v2.2 board
- MCU: STM32F401RCT6 (84 MHz Cortex-M4F, 256 KB flash, 64 KB SRAM, no CCM),
  8 MHz HSE, MS35775 step/dir drivers (no UART/SPI config)
- Host: Pi 3 (4 cores, 1 GB) running MainsailOS; klippy talks USART1
  (PA10/PA9) through the board's onboard CH340 at 250000 baud → `/dev/ttyUSB0`
- Flashing: SWD via ST-Link V2 on the Pi's USB + OpenOCD; stock ZNP 32 KiB
  bootloader kept, app at `0x8008000` (SD-card flash as `ZNP_ROBIN_NANO.bin`
  remains a recovery path)
- SWD caveat: PA13/PA14 double as the X-min endstop input on this board and
  the firmware parks in WFI with AHB gated, so attach requires
  connect-under-reset — the NRST wire is mandatory, not optional
- Runtime profile: rt_storage 36864, piece ring 32768, sample rate 10 kHz
  (F401 Kconfig defaults)
