# Bondtech INDX

This document describes how to set up and calibrate a
[Bondtech INDX](https://www.bondtech.se/) toolboard with Kalico. The
INDX toolboard drives an inductive nozzle heater with a contactless IR
temperature sensor, and runs its own PID controller on the toolboard
MCU.

Support consists of the `[indx]` host module (see
[config reference](Config_Reference.md#indx)) and dedicated firmware
for the toolboard's SAME51 MCU, both included in Kalico. Kalico is
the reference implementation; the
[BondtechAB/indx_klipper](https://github.com/BondtechAB/indx_klipper)
module tracks Kalico to provide compatibility with mainline Klipper.
Also see the original
[commissioning guide](https://gist.github.com/dalegaard/b5106af63fcdde305e8c47421a8cc9b9).

## Flashing the firmware

Connect the toolboard over USB and bridge the `CAN RESET` jumper while
powering up to force the bootloader. The device should show up as
"INDX Toolboard Bootloader" with USB id `04d8:e483`:

```
$ lsusb | grep INDX
```

Kalico ships a ready-made build configuration. From the Kalico
directory run:

```
$ KCONFIG_CONFIG=board_configs/bondtech_indx_usb.config make
$ KCONFIG_CONFIG=board_configs/bondtech_indx_usb.config make flash FLASH_DEVICE=04d8:e483
```

## Configuration

A minimal configuration looks like:

```
[mcu indxmcu]
serial: /dev/serial/by-id/usb-Bondtech_INDX_<YOUR SERIAL>-if00

[indx]
mcu: indxmcu

[extruder]
sensor_type: indx
min_temp: 0
max_temp: 300
heater_pin: indx:heater
control: watermark
nozzle_diameter: 0.4
filament_diameter: 1.75
step_pin: indxmcu:motor_step
dir_pin: indxmcu:motor_dir
enable_pin: !indxmcu:motor_enable
microsteps: 32
rotation_distance: 5.7

[tmc2240 extruder]
cs_pin: indxmcu:motor_cs
spi_software_sclk_pin: indxmcu:motor_sclk
spi_software_mosi_pin: indxmcu:motor_mosi
spi_software_miso_pin: indxmcu:motor_miso
rref: 24000
run_current: 0.6

[fan]
pin: indxmcu:part_cooling
tachometer_pin: indxmcu:part_cooling_tacho
```

The `[indx]` module registers named aliases for all toolboard pins
(such as `indxmcu:motor_step`, `indxmcu:part_cooling`,
`indxmcu:endstop`) and configures the SERCOM peripherals
automatically. The extruder heater is exposed as the virtual pin
`indx:heater` and the nozzle temperature as `sensor_type: indx`. The
heatsink fan is managed by the toolboard module itself: it turns on
whenever the heater is active, the nozzle is hot or a stepper on the
toolboard is enabled, and a blocked fan triggers a shutdown.

## Calibration

The heater will not heat until it has been calibrated. With the nozzle
cold and the printer idle run:

```
INDX_CALIBRATE
```

This first tunes the inductive coil drive timings and then performs a
heat/hold/cooldown cycle to fit the thermal model. Afterwards run
`SAVE_CONFIG` to persist the results.

Optionally calibrate the part cooling fan compensation (requires the
part cooling fan to be configured):

```
INDX_FAN_CALIBRATE
SAVE_CONFIG
```

The thermal model also accounts for the energy carried away by molten
filament. Either set the filament parameters directly:

```
INDX_SET_MODEL_PARAMS FILAMENT_DENSITY=<g/cm3> FILAMENT_HEAT_CAPACITY=<J/g/K>
```

or measure them while loading filament with the nozzle at printing
temperature:

```
INDX_LOAD_FILAMENT
```

Use `INDX_CLEAR_FILAMENT` to reset the filament parameters, and
`SAVE_CONFIG` to persist any of these values.

See the [G-Code reference](G-Codes.md#indx) for the full list of INDX
commands and their parameters.
