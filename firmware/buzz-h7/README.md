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
3. Drives EN low so the TMC powers the motor (retains mainline register state).
4. 800 step toggles at ~200 Hz forward (~5 mm of motion).
5. Pause 1 sec.
6. 800 step toggles back.
7. Wait 10 sec (operator confirms motion stopped).
8. Jumps to the STM32H723 ROM DFU bootloader at `0x1FF09800` (AN2606 Rev 50
   §33, RM0468 §2.3). USB re-enumerates as `0483:df11` automatically — no
   BOOT0 pin press or physical reset needed.

## Build (on the Pi)

```sh
cd ~/klipper/firmware/buzz-h7
cargo build --release
arm-none-eabi-objcopy -O binary \
    target/thumbv7em-none-eabihf/release/buzz-h7 \
    buzz-h7.bin
```

## First flash (one-time manual step)

The very first time you flash this build you need BOOT0+RESET to enter the
STM32 ROM bootloader, because there is no prior firmware that auto-jumps.
After that, every subsequent flash is fully automatic (see below).

```sh
# Hold BOOT0, tap RESET, release BOOT0.
dfu-util -d 0483:df11 -a 0 -s 0x08020000:leave -D buzz-h7.bin
```

## Subsequent flashes (auto-DFU)

After the buzz sequence completes and ~10 seconds elapse, the firmware jumps
to the ROM DFU bootloader on its own. USB re-enumerates as `0483:df11`.

```sh
# Wait for motor to stop + ~10 sec, then:
dfu-util -d 0483:df11 -a 0 -s 0x08020000:leave -D buzz-h7.bin
```

No BOOT0 pin, no physical button press. `dfu-util -l` will show the device
in DFU mode if you need to confirm before flashing.

## Reading the result

- **X motor moves visibly forward then back** — TMC + pins + driver +
  mechanical are all fine. The Kalico motion engine is the only suspect for
  the "engine clean, motors silent" bug.
- **Motor stays silent / energized-only** — bug is below the engine. Most
  likely TMC current too low for this specific motor, CHOPCONF wrong for
  this driver revision, or a pin-routing mismatch in printer.cfg vs the
  actual board wiring.

## Restoring Klipper after testing

After the H7 is in DFU mode (auto, no button press needed):

```sh
dfu-util -d 0483:df11 -a 0 -s 0x08020000:leave -D klipper.bin
```

Buzz firmware is gone, normal operation resumes.
