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
- Host: Raspberry Pi 5 Model B Rev 1.1 (4-core Cortex-A76, 4 GB) on Debian 13
  (trixie); klippy talks USART1 (PA10/PA9) through the board's onboard CH340 at
  500000 baud → `/dev/ttyUSB0`. The host is fast — when the drip can't keep a move
  fed, the bottleneck is the F401 MCU foreground (84 MHz, sharing CPU with the
  sample ISR + status emission + piece ingestion), not the Pi.
- Flashing (ST-Link V2 over SWD, **NRST wire left disconnected**): app at
  `0x8008000`, stock ZNP 32 KiB bootloader kept (SD-card `ZNP_ROBIN_NANO.bin` is
  the recovery path). The X-min endstop is on **PA13 (= SWDIO)**, and the NRST
  wire is kept off because with it tied to the ST-Link the board won't boot — so
  connect-under-reset is NOT available. Once klippy runs it reconfigures PA13 to
  a GPIO (killing SWDIO) and the idle firmware parks in WFI with AHB gated, so
  you cannot attach to the running target. Flash in the clean post-boot window
  instead (verified 2026-06-09):
  1. **Hold klippy down so it can't reconfigure PA13.** It reconnects two ways and
     BOTH must be suppressed or the flash fails with "init mode failed (unable to
     connect to the target)" the moment klippy re-grabs PA13 as GPIO:
     - systemd `Restart=always` on klipper.service, and
     - a udev rule `99-klipper-mcu-autorestart.rules` that restarts klipper the
       instant the CH340 (`1a86:7523`) tty re-appears after the power-cycle.

         sudo systemctl stop klipper moonraker
         printf '[Service]\nRestart=no\n' | \
           sudo tee /etc/systemd/system/klipper.service.d/norestart.conf
         sudo systemctl daemon-reload
         sudo mv /etc/udev/rules.d/99-klipper-mcu-autorestart.rules{,.disabled}
         sudo udevadm control --reload-rules
  2. power-cycle the printer for a fresh boot (HomeKit `Plug 2 OFF` then
     `Plug 2 ON`) — PA13 stays SWDIO while no host is connected
  3. openocd: `reset_config none` (no NRST → software reset), `reset halt` to
     catch the core at the reset vector — NOT a plain `halt`, which HardFaults
     the flash algorithm by stopping the running 40 kHz ISR mid-flight:

         openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
           -c "reset_config none" -c "init" -c "reset halt" \
           -c "flash write_image erase out/klipper.bin 0x8008000" \
           -c "verify_image out/klipper.bin 0x8008000" \
           -c "reset run" -c "shutdown"

  4. restore the auto-restart machinery and bring klippy back:

         sudo mv /etc/udev/rules.d/99-klipper-mcu-autorestart.rules{.disabled,}
         sudo rm /etc/systemd/system/klipper.service.d/norestart.conf
         sudo systemctl daemon-reload && sudo udevadm control --reload-rules
         sudo systemctl start moonraker klipper
- Runtime profile: rt_storage 36864, piece ring 32768, sample rate 10 kHz
  (F401 Kconfig defaults)
