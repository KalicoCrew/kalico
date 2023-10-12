<p align="center"><a href="https://DangerKlippers.github.io/danger-klipper/"><img align="center" src="docs/img/klipper-logo.png" alt="Danger-Klipper Logo"></a></p>

[![Action Status](https://github.com/DangerKlippers/danger-klipper/actions/workflows/ci-build_test.yaml/badge.svg?branch=master)](https://github.com/DangerKlippers/danger-klipper/actions/workflows/ci-build_test.yaml)

Welcome to the Danger Klipper project!

This is a community-maintained fork of the [Klipper](https://github.com/Klipper3d/klipper) firmware.

Our goal is to support features and behavior that could be "risky" if used incorrectly.

If I want my printer to light itself on fire, I should be able to make my printer light itself on fire.

Features merged into the master branch:

- [core: No Python2 tests; No PRU boards](https://github.com/DangerKlippers/danger-klipper/pull/39)

- [fan: Normalising Fan PWM power](https://github.com/DangerKlippers/danger-klipper/pull/44) ([klipper#6307](https://github.com/Klipper3d/klipper/pull/6307))

- [fan: Reverse FAN](https://github.com/DangerKlippers/danger-klipper/pull/51) ([klipper#4983](https://github.com/Klipper3d/klipper/pull/4983))

- [heater: Modify PID without reload](https://github.com/DangerKlippers/danger-klipper/pull/35)

- [heater: Velocity PID](https://github.com/DangerKlippers/danger-klipper/pull/47) ([klipper#6272](https://github.com/Klipper3d/klipper/pull/6272))

- [gcode: Jinja2.ext.do extension](https://github.com/DangerKlippers/danger-klipper/pull/26) ([klipper#5149](https://github.com/Klipper3d/klipper/pull/5149))

- [probe: Dockable Probe](https://github.com/DangerKlippers/danger-klipper/pull/43) ([klipper#4328](https://github.com/Klipper3d/klipper/pull/4328))

- [probe: Drop the first result](https://github.com/DangerKlippers/danger-klipper/pull/2) ([klipper#3397](https://github.com/Klipper3d/klipper/issues/3397))

- [probe: z_calibration](https://github.com/DangerKlippers/danger-klipper/pull/31) ([klipper#4614](https://github.com/Klipper3d/klipper/pull/4614) / [protoloft/z_calibration](https://github.com/protoloft/klipper_z_calibration))

- [core: danger_options](https://github.com/DangerKlippers/danger-klipper/pull/67)

- [stepper: home_current](https://github.com/DangerKlippers/danger-klipper/pull/65)

- [homing: post-home retract](https://github.com/DangerKlippers/danger-klipper/pull/65)

- [homing: sensorless minimum home distance](https://github.com/DangerKlippers/danger-klipper/pull/65)

"Dangerous Klipper for dangerous users"

Klipper is a 3d-Printer firmware. It combines the power of a general
purpose computer with one or more micro-controllers. See the
[features document](https://DangerKlippers.github.io/danger-klipper/Features.html) for more
information on why you should use Klipper.

To begin using Klipper start by
[installing](https://DangerKlippers.github.io/danger-klipper/Installation.html) it.

Klipper is Free Software. See the [license](COPYING) or read the
[documentation](https://DangerKlippers.github.io/danger-klipper/Overview.html).
