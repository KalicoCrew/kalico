# buzz-h7 — bare-metal stepper buzz test for STM32H723 (BTT Octopus Pro)

Standalone Rust firmware that drives stepper_x on the user's Trident bench
directly — no Klipper, no Kalico runtime, no host communication. Purpose:
isolate whether the "motors energize but stay silent during a jog" symptom is
caused by the Kalico motion engine or by something below it (TMC config,
step-pin routing, mechanical, current scale).

## What it does on boot

1. Brings the core up on HSI (64 MHz). No PLL — keeps init simple.
2. Configures GPIO + SPI1 pins per the user's `printer.cfg`:
   - PG4 step, PC1 dir (inverted), PA2 enable (inverted)
   - PC7 CS, PA5/6/7 SPI1 SCK/MISO/MOSI
3. Drives EN low → TMC powers the motor.
4. Sends 6 TMC5160 register writes over SPI:
   GCONF=0, GLOBAL_SCALER=128, IHOLD_IRUN (~0.7 A), TPOWERDOWN, CHOPCONF
   (TOFF=3 — critical), PWMCONF.
5. 800 step toggles at ~200 Hz forward (`~5 mm of motion`).
6. Pause 1 sec.
7. 800 step toggles back.
8. Wait 10 sec (operator confirms motion stopped).
9. `NVIC_SystemReset` → lands in Katapult. Ready for the next flash with
   zero manual BOOT0/DFU intervention.

## Build (on the Pi)

```sh
cd ~/klipper/firmware/buzz-h7
cargo build --release
arm-none-eabi-objcopy -O binary \
    target/thumbv7em-none-eabihf/release/buzz-h7 \
    buzz-h7.bin
```

## Flash via H7 bootloader

Same flow as Klipper itself — the H7 has a Katapult-variant bootloader in
the first 128 KiB of flash (`CONFIG_FLASH_APPLICATION_ADDRESS=0x8020000` in
`.config.h7.bak`). It stays untouched by us. After the bootloader signals
ready, push the .bin to the 0x08020000 offset.

`scripts/flash_can.py` or `scripts/flash_usb.py` (whichever the Pi uses for
the H7 normally) works the same way — point it at `buzz-h7.bin` instead of
`klipper.bin`.

## Reading the result

- **X motor moves visibly forward then back** → TMC + pins + driver +
  mechanical are all fine. The Kalico motion engine is the only suspect for
  the "engine clean, motors silent" bug.
- **Motor stays silent / energized-only** → bug is below the engine. Most
  likely TMC current too low for this specific motor, CHOPCONF wrong for
  this driver revision, or a pin-routing mismatch in printer.cfg vs the
  actual board wiring.

## Restoring Klipper after testing

Flash `klipper.bin` (built by the normal Kalico build) via the same
Katapult flow. Buzz firmware is gone, normal operation resumes.
