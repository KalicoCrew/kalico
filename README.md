<p align="center"><a href="https://docs.kalico.gg"><img align="center" src="docs/logo/kalico-big.png" alt="Kalico Logo"></a></p>

[![Action Status](https://github.com/KalicoCrew/kalico/actions/workflows/ci-build_test.yaml/badge.svg?branch=main)](https://github.com/KalicoCrew/kalico/actions/workflows/ci-build_test.yaml)

# Welcome to the Kalico project!

This is a community-maintained fork of the [Klipper](https://github.com/Klipper3d/klipper) firmware.

Our goal is to support features and behavior that could be "risky" if used incorrectly.

If I want my printer to light itself on fire, I should be able to make my printer light itself on fire.

See the [Danger Features document](https://docs.kalico.gg/Danger_Features.html) for more information on *some* of the differences from Klipper.

## Features merged into the main branch:

- [core: no Python2 tests; no PRU boards](https://github.com/KalicoCrew/kalico/pull/39)

- [core: git-untracked folder, plugins for user-plugins](https://github.com/KalicoCrew/kalico/pull/82)

- [core: danger_options](https://github.com/KalicoCrew/kalico/pull/67)

- [core: rotate log file at every restart](https://github.com/KalicoCrew/kalico/pull/181)

- [fan: normalising Fan PWM power](https://github.com/KalicoCrew/kalico/pull/44) ([klipper#6307](https://github.com/Klipper3d/klipper/pull/6307))

- [fan: reverse FAN](https://github.com/KalicoCrew/kalico/pull/51) ([klipper#4983](https://github.com/Klipper3d/klipper/pull/4983))

- [heaters: modify PID without reload](https://github.com/KalicoCrew/kalico/pull/35)

- [heaters: MPC temperature control](https://github.com/KalicoCrew/kalico/pull/333)

- [heaters: velocity PID](https://github.com/KalicoCrew/kalico/pull/47) ([klipper#6272](https://github.com/Klipper3d/klipper/pull/6272))

- [heaters: PID-Profiles](https://github.com/KalicoCrew/kalico/pull/162)

- [heaters: expose heater thermistor out of min/max](https://github.com/KalicoCrew/kalico/pull/182)

- [heaters/fan: new heated_fan module](https://github.com/KalicoCrew/kalico/pull/259)

- [gcode: jinja2.ext.do extension](https://github.com/KalicoCrew/kalico/pull/26) ([klipper#5149](https://github.com/Klipper3d/klipper/pull/5149))

- [gcode: gcode_shell_command](https://github.com/KalicoCrew/kalico/pull/71) ([klipper#2173](https://github.com/Klipper3d/klipper/pull/2173) / [kiuah](https://github.com/dw-0/kiauh/blob/master/resources/gcode_shell_command.py) )

- [gcode: expose math functions to gcode macros](https://github.com/KalicoCrew/kalico/pull/173) ([klipper#4072](https://github.com/Klipper3d/klipper/pull/4072))

- [gcode: HEATER_INTERRUPT gcode command](https://github.com/KalicoCrew/kalico/pull/94)

- [gcode: RELOAD_GCODE_MACROS command](https://github.com/KalicoCrew/kalico/pull/305)

- [probe: dockable Probe](https://github.com/KalicoCrew/kalico/pull/43) ([klipper#4328](https://github.com/Klipper3d/klipper/pull/4328))

- [probe: drop the first result](https://github.com/KalicoCrew/kalico/pull/2) ([klipper#3397](https://github.com/Klipper3d/klipper/issues/3397))

- [probe: z_calibration](https://github.com/KalicoCrew/kalico/pull/31) ([klipper#4614](https://github.com/Klipper3d/klipper/pull/4614) / [protoloft/z_calibration](https://github.com/protoloft/klipper_z_calibration))

- [z_tilt: z-tilt calibration](https://github.com/KalicoCrew/kalico/pull/105) ([klipper3d#4083](https://github.com/Klipper3d/klipper/pull/4083) / [dk/ztilt_calibration](https://github.com/KalicoCrew/kalico/pull/54))

- [stepper: home_current](https://github.com/KalicoCrew/kalico/pull/65)

- [stepper: current_change_dwell_time](https://github.com/KalicoCrew/kalico/pull/90)

- [homing: post-home retract](https://github.com/KalicoCrew/kalico/pull/65)

- [homing: sensorless minimum home distance](https://github.com/KalicoCrew/kalico/pull/65)

- [homing: min_home_dist](https://github.com/KalicoCrew/kalico/pull/90)

- [virtual_sdcard: scanning of subdirectories](https://github.com/KalicoCrew/kalico/pull/68) ([klipper#6327](https://github.com/Klipper3d/klipper/pull/6327))

- [retraction: z_hop while retracting](https://github.com/KalicoCrew/kalico/pull/83) ([klipper#6311](https://github.com/Klipper3d/klipper/pull/6311))

- [danger_options: allow plugins to override conflicting extras](https://github.com/KalicoCrew/kalico/pull/82)

- [danger_options: expose the multi mcu homing timeout as a parameter](https://github.com/KalicoCrew/kalico/pull/93)

- [danger_options: option to configure the homing elapsed distance tolerance](https://github.com/KalicoCrew/kalico/pull/110)

- [danger_options: option to ignore ADC out of range](https://github.com/KalicoCrew/kalico/pull/129)

- [temperature_mcu: add reference_voltage](https://github.com/KalicoCrew/kalico/pull/99) ([klipper#5713](https://github.com/Klipper3d/klipper/pull/5713))

- [adxl345: improve ACCELEROMETER_QUERY command](https://github.com/KalicoCrew/kalico/pull/124)

- [extruder: add flag to use the PA constant from a trapq move vs a cached value](https://github.com/KalicoCrew/kalico/pull/132)

- [force_move: turn on by default](https://github.com/KalicoCrew/kalico/pull/135)

- [respond: turn on by default](https://github.com/KalicoCrew/kalico/pull/296)

- [exclude_object: turn on by default](https://github.com/KalicoCrew/kalico/pull/306)

- [bed_mesh: add bed_mesh_default config option](https://github.com/KalicoCrew/kalico/pull/143)

- [config: CONFIG_SAVE updates included files](https://github.com/KalicoCrew/kalico/pull/153)

- [kinematics: independent X&Y accel/velocity for corexy and cartesian](https://github.com/KalicoCrew/kalico/pull/4)

- [kinematics: independent X&Y accel/velocity for corexz](https://github.com/KalicoCrew/kalico/pull/267)

- [idle_timeout: allow the idle timeout to be disabled](https://github.com/KalicoCrew/kalico/issues/165)

- [canbus: custom CAN bus uuid hash for deterministic uuids](https://github.com/KalicoCrew/kalico/pull/156)

- [filament_switch|motion_sensor:  runout distance, smart and runout gcode](https://github.com/KalicoCrew/kalico/pull/158)

- [z_tilt|qgl: custom threshold for probe_points_increasing check](https://github.com/KalicoCrew/kalico/pull/189)

- [save_config: save without restarting the firmware](https://github.com/KalicoCrew/kalico/pull/191)

- [configfile: recursive globs](https://github.com/KalicoCrew/kalico/pull/200) / ([klipper#6375](https://github.com/Klipper3d/klipper/pull/6375))

- [temperature_fan: curve control algorithm](https://github.com/KalicoCrew/kalico/pull/193)

- [shaper_calibrate: store and expose accel_per_hz](https://github.com/KalicoCrew/kalico/pull/224)

- [resonance_tester: accepts ACCEL_PER_HZ in TEST_RESONANCES](https://github.com/KalicoCrew/kalico/pull/312)

- [mcu: support for AT32F403](https://github.com/KalicoCrew/kalico/pull/284)

- [z_tilt, quad_gantry_level: adaptive horizontal move z](https://github.com/KalicoCrew/kalico/pull/336)

- [core: non-critical-mcus](https://github.com/KalicoCrew/kalico/pull/339)

- [gcode_macros: !python templates](https://github.com/KalicoCrew/kalico/pull/360)

- [core: action_log](https://github.com/KalicoCrew/kalico/pull/367)

- [danger_options: configurable homing constants](https://github.com/KalicoCrew/kalico/pull/378)

If you're feeling adventurous, take a peek at the extra features in the bleeding-edge-v2 branch [feature documentation](docs/Bleeding_Edge.md)
and [feature configuration reference](docs/Config_Reference_Bleeding_Edge.md):

- [extruder/pa: do not smooth base extruder position, only advance](https://github.com/KalicoCrew/kalico/pull/266)

- [dmbutyugin's advanced-features branch - Pull Request #262](https://github.com/KalicoCrew/kalico/pull/262)
  - stepper: high precision stepping protocol
  - extruder: sync extruder motion with input shaper
  - extruder: new print_pa_tower utility
  - input_shaper: smooth input shapers
  - input_shaper: new print_ringing_tower utility

## Switch to Kalico

> [!NOTE]
> Any add-on modules you are using will need to be reinstalled after switching to Kalico. This includes things like Beacon support, led-effect, etc.
>
> Any data in ~/printer_data such as printer configs and macros will be unaffected.

### Option 1. Manually clone the repository

If desired, make a backup copy of your existing Klipper installation by running:

```bash
mv ~/klipper ~/klipper_old
```

Then clone the Kalico repo and restart the klipper service:

```bash
git clone https://github.com/KalicoCrew/kalico.git ~/klipper
sudo systemctl restart klipper
```

### Option 2. Using KIAUH

For users that are not comfortable using Git directly, [KIAUH v6](https://github.com/dw-0/kiauh) is able to use custom repositories.

To do this, add the Kalico repo to KIAUH's custom repository settings with the following steps:

From the KIAUH menu select:

- [S] Settings
- 1\) Set custom Klipper repository
- Use `https://github.com/KalicoCrew/kalico` as the new repository URL
- Use `main` or `bleeding-edge-v2` as the new branch name
- Select 'Y' to apply the changes
- Enter 'B' for back twice
- 'Q' to quit

### Option 3. Adding a git-remote to the existing installation
Can switch back to mainline klipper at any time via a `git checkout upstream_main`

```bash
cd ~/klipper
git remote add kalico https://github.com/KalicoCrew/kalico.git
git checkout -b upstream-main origin/master
git branch -D master
git checkout -b main kalico/main
sudo systemctl restart klipper
sudo systemctl restart moonraker
```

---

Kalico is a 3d-Printer firmware. It combines the power of a general
purpose computer with one or more micro-controllers. See the
[features document](https://docs.kalico.gg/Features.html) for more
information on why you should use Kalico.

To begin using Kalico start by
[installing](https://docs.kalico.gg/Installation.html) it.

Kalico is Free Software. See the [license](COPYING) or read the
[documentation](https://docs.kalico.gg/Overview.html).

[![Join me on Discord](https://discord.com/api/guilds/1297243471442214913/widget.png?style=banner2)](https://kalico.gg/discord)
