# Beacon Klipper

Beacon Klipper is the klipper module for using the [Beacon](https://beacon3d.com) Eddy Current Scanner.

## Documentation

[Beacon](https://docs.beacon3d.com)

## Firmware Release Notes

### Beacon 2.1.0 - July 11, 2024
 - Added parameters to adjust contact noise tolerance
 - Adjusted contact latency values to match new parameters
 - Increased robustness of the primary contact trigger

### Beacon 2.0.1 - June 4, 2024
 - Fixed USB enumeration issue affecting fast host controllers

### Beacon 2.0.0 - May 29, 2024
 - Beacon Contact Release
 - Adopted RTIC - The Hardware Accelerated Rust RTOS
 - Added nozzle contact detection processing
 - Improved data transmit and processing efficiency
 - Reports MCU temperature and supply voltage
 - Added watchdog superviser
 - Improved error detection, reporting, and recovery
 - Reduced current consumption 10% overall
 - Reduced current consumption 55% when used above rated temperature

### Beacon 1.1.0 - Dec 27, 2023
 - RevH Enabling Release
 - Added Accel Driver

### Beacon 1.0.0 - Jan 26, 2023
 - Initial Release

