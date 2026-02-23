# Load Cells

This document describes support for load cells. Load cells measure applied force applied to a strain gauge with an ADC. They may be used to read force data, weigh things like filament spools, or function as a probe.

Warning: Prior to using a load cell it must be calibrated. If not correctly calibrated the force/weight reported will be incorrect and this may result in permanent damage to your load cell and/or printer. This module cannot compensate for poor calibration, damaged strain gauges or electrical noise.

## Basic Load Cell Configuration

Load cells can be configured for use as a scale.

```ini
[load_cell]
sensor_type: hx717
sclk_pin: PA5
dout_pin: PA4
sample_rate: 320
counts_per_gram: 245
reference_tare_counts: 12345
```

- `sensor_type: hx717`\
  _Required_\
  Each sensor has different required fields, check their configuration reference for details:

  * [`hx711`](Config_Reference.md#hx711)
  * [`hx717`](Config_Reference.md#hx717)
  * [`ads1220`](Config_Reference.md#ads1220)

- `counts_per_gram: 245`\
  _Default Value: None_\
  Conversion factor from raw sensor counts to grams, calculated by `LOAD_CELL_CALIBRATE`.

- `reference_tare_counts: 12345`\
  _Required_\
  Baseline tare value in raw sensor counts, set by `LOAD_CELL_CALIBRATE`.

## Diagnostics

### Checking Load Cell Operation

`LOAD_CELL_DIAGNOSTIC` ([docs](G-Codes.md#load_cell_diagnostic))

Collects samples from the load cell and reports health and statistics. Run this command when first connecting a load cell to verify wiring and configuration.

```
LOAD_CELL_DIAGNOSTIC
// Collecting load cell data for 10 seconds...
// Samples Collected: 3211
// Measured samples per second: 332.0
// Good samples: 3211, Saturated samples: 0, Unique values: 900
// Sample range: [4.01% to 4.02%]
// Sample range / sensor capacity: 0.00524%
```

Check the output:
- Measured samples per second should be close to the configured `sample_rate`. If not, check configuration. For HX711, sample rate is set by hardware.
- Saturated samples should be 0. Non-zero indicates excessive force beyond the sensor's measurement range.
- Unique values should be a large percentage of samples collected. If unique values is 1, verify wiring.
- Tap or push the sensor during the test. The sample range should increase if the sensor is functioning correctly.

## Calibration

### Calibrating a Load Cell

`LOAD_CELL_CALIBRATE` ([docs](G-Codes.md#load_cell_calibrate))

Starts the interactive calibration utility. The calibration process consists of three steps:

1. `TARE` - Establish zero force value and set `reference_tare_counts`
2. `CALIBRATE GRAMS=<value>` - Apply known force and calculate `counts_per_gram`
3. `ACCEPT` - Save calibration results to configuration

Use `ABORT` to cancel calibration at any time.

Running a `LOAD_CELL_DIAGNOSTIC` after calibration will show additional information in grams.

#### Applying a Known Force

The `CALIBRATE GRAMS=<value>` step requires applying a known force. The method depends on load cell location:

**Platform-mounted load cells** (under bed or filament holder):
Place an object of known mass on the platform. Ideally use a large percentage of the load cell's rated capacity (e.g., 5 kg for a 5 kg load cell).

**Toolhead load cells**:
Place a digital scale on the bed and gently lower the toolhead onto it (or raise the bed if the bed moves). Use at least 1 kg of force. Too much force may damage the bed or toolhead, so move in small steps. Take a reading from the digital scale and enter it into the `CALIBRATE GRAMS=<value>` command.

#### Understanding Calibration Results

```
CALIBRATE GRAMS=555
// Calibration value: -2.78% (-559467), Counts/gram: 87.944082,
Total capacity: +/- 29.14Kg
```

`Calibration value:` shows how much of the sensor's range, as a percentage, was used to calibrate.

`Counts/gram:` is the number of sensor counts equal to 1 gram of force. The larger this number is the more precise the scale will be.

`Total capacity` is the highest force that the sensor could register. The `Total capacity` should be close to the load cell's rated capacity. If much larger, consider a higher gain setting or more sensitive load cell. This is more critical for sensors with bit widths below 24 bits.

## Operations

### Reading Force Data

`LOAD_CELL_READ`

Reads the current force on the load cell.

```
LOAD_CELL_READ
// 10.6g (1.94%)
```

Force data is also available in the `load_cell` printer object:

```gcode
{% set grams = printer.load_cell.force_g %}
```

This value is averaged over the last 1 second, similar to temperature sensors.

### Taring a Load Cell

`LOAD_CELL_TARE`

Sets the current reading to zero force. Useful for measuring relative weight changes, such as filament consumption during printing.

```
LOAD_CELL_TARE
// Load cell tare value: 5.32% (445903)
```

The tare value is available in the `load_cell` printer object:

```gcode
{% set tare_counts = printer.load_cell.tare_counts %}
```

## Load Cell Probe Configuration

This example adds probe functionality to a calibrated load cell. A `[load_cell_probe]` section includes all `[load_cell]` parameters, load cell probing specific parameters and [`[probe]`](Config_Reference.md#probe) parameters.

```ini
[load_cell_probe]
# load cell settings
sensor_type: hx717 # sensor specific config
counts_per_gram: 245
reference_tare_counts: 12345
# load cell probe settings
trigger_force: 75
force_safety_limit: 5000
drift_filter_cutoff_frequency: 0.5
# probe settings
z_offset: 0.0
```

- `counts_per_gram: 245`\
  _Default Value: None_\
  Conversion factor from raw sensor counts to grams, calculated by `LOAD_CELL_CALIBRATE`. All probing force limits depend on this value being accurate.

- `reference_tare_counts: 12345`\
  _Default Value: None_\
  Baseline tare value in raw sensor counts, set by `LOAD_CELL_CALIBRATE`. Used as the zero value for with `force_safety_limit` to define the safe operating range.

- `force_safety_limit: 5000`\
  _Default Value: 5000 (2 kilograms)_\
  Maximum absolute force in grams, relative to `reference_tare_counts`, allowed during homing or probing. If exceeded, the probe stops with an error TK`!! Load Cell Probe Error: load exceeds safety limit`

- `trigger_force: 75`\
  _Default Value: 75 (75 grams)_\
  Force in grams to trigger the endstop during probing, measured relative to the tare value at the start of the probe. Expect overshoot; higher probing speed or lower sample rate increases peak force. See [Multi MCU Homing](Multi_MCU_Homing.md) for multi-MCU timing considerations.

- `drift_filter_cutoff_frequency: 0.5`\
  _Default Value: None (disabled)_\
  Cutoff frequency in Hz for the continuous tare drift filter. Enables a filter on the MCU to track drift from bowden tubes and drag chains. Requires [SciPy](#installing-scipy). Setting this value too high can delay probe triggering and increase force on the toolhead.

- `z_offset: 0.0`\
  _Required_\
  The distance (in mm) between the bed and the nozzle when the probe
  triggers. For load cell probes this is 0.

See the [configuration reference](Config_Reference.md#load_cell_probe) for all available options.

### Safety

Load cells are direct nozzle contact probes. The system includes safety checks to prevent excessive force on the toolhead. Poorly chosen configuration values can defeat these protections.

**Calibration check:**
Before homing or probing, the load cell probe checks that it is calibrated. If not, the printer stops with error `!! Load Cell Probe Error: Load Cell not calibrated`.

**Accurate `counts_per_gram`:**
This setting converts raw counts to grams. All safety limits are in gram units. An inaccurate value allows excessive force on the toolhead. Never guess this value—always use `LOAD_CELL_CALIBRATE`.

**Conservative `trigger_force`:**
Probing always overshoots `trigger_force` before stopping. A setting of 100 g may result in 350 g peak force. Overshoot increases with faster probing speed, low sample rate, or multi-MCU configurations.

**`force_safety_limit` protection:**
This setting prevents several failure modes:
- Excessive `drift_filter_cutoff_frequency` causing filtered-out probe events
- Repeated probing in one location without retraction accumulating force
- Damaged strain gauge changing `reference_tare_counts`
- Temperature changes altering strain gauge baseline readings

If the limit is exceeded, the probe will stop with an error: `!! Load Cell Probe Error: load exceeds safety limit`

**Watchdog task:**
During homing, a watchdog monitors sensor data. If the sensor fails to send measurements for 2 sample periods, the MCU shuts down with error `!! Load Cell Probe Error: timed out waiting for sensor data`. This usually indicates an ADC fault or inadequate grounding. Ensure the frame, power supply, and print bed are grounded. Multiple ground connections may be required. Sand anodized aluminum at ground connection points for good electrical contact.

### Testing Probe Operation

`LOAD_CELL_TEST_TAP [COUNT=<taps>] [TIMEOUT=<seconds>]`\
_Default COUNT: 3_\
_Default TIMEOUT: 30_

Tests probe operation without moving axes. Detects the specified number of taps before ending. If no tap is detected within the timeout period, the command fails.

**Note:** Load cell probes do not support `QUERY_ENDSTOPS` or `QUERY_PROBE`, they always return not triggered. Use `LOAD_CELL_TEST_TAP` to verify functionality before probing.

### Homing Configuration

Load cell probes support homing the Z axis. Homing is less accurate than probing with the `PROBE` command. After homing, use `PROBE` to do a high accuracy Z homing:

```gcode
PROBE HOME=Z
```

### Probing Temperature

Keep nozzle temperature below the filament oozing point during homing and probing. 140°C is a good starting point for all filament types.

Filament ooze is the primary source of probing error. Kalico does not detect poor quality taps caused by ooze, and modules like `quad_gantry_level` will repeatedly probe fouled locations. Probing at printing temperatures is not recommended.

### Nozzle Protection

See [Voron Tap's activate_gcode](https://github.com/VoronDesign/Voron-Tap/blob/main/config/tap_klipper_instructions.md) for protecting the print surface from a hot nozzle.

### Nozzle Cleaning

Clean the nozzle before probing. Suggested sequence:
1. Heat nozzle to probing temperature (e.g., `M109 S140`)
2. Home the machine (`G28`)
3. Scrub the nozzle
4. Heat soak the bed
5. Perform probing tasks (QGL, bed mesh, etc.)

### Nozzle Temperature Compensation

Due to ooze, it's not possible to probe at the printing temperature. The nozzle is heated up after probing, causing it to expand. The nozzle expands most along its length, towards to bed. This should be compensated with [z_thermal_adjust](Config_Reference.md#z_thermal_adjust).

Measure `PROBE_ACCURACY` at two temperatures (e.g., 180°C and 290°C) and calculate:

```
temp_coeff = (z_average_hot - z_average_cold) / (temp_hot - temp_cold)
```

Example: `temp_coeff = -0.05 / (290 - 180) = -0.00045455`

Expect a negative value (`z_thermal_adjust` will move the nozzle away from bed with negative values and towards it with positive values).

Example configuration:

```ini
[z_thermal_adjust nozzle]
temp_coeff: -0.00045455
sensor_type: temperature_combined
sensor_list: extruder
combination_method: max
min_temp: 0
max_temp: 400
max_z_adjustment: 0.1
```

### Bed Mesh Settings

**Disable `relative_reference_index`**
Because load cell probes give an absolute value for z that is not relative to anything, no `relative_reference_index` is required. Simple delete the setting in `[bed_mesh]`. Deleting the line from the config turns it off.

**Enable aggressive move splitting**
```ini
move_check_distance: 3.0
split_delta_z: 0.01
```
Set up the mesh to adjust the z height as frequently as possible. These two settings change how bed mesh evaluates z changes. Minimize the `split_delta_z` to get high resolution mesh following (0.01 is 10 microns, 10x the probe resolution). Choosing a small value for `move_check_distance` forces bed_mesh to re-evaluate the z height more frequently. If these settings are left at their defaults you may see streaks in the first layer caused by infrequent adjustments.

**Enable `horizontal_z_clearance`**
```ini
horizontal_z_clearance: 0.4
```
Using `horizontal_z_clearance`, the probe always retracts by that amount between mesh points. This can greatly reduce the z travel distance while adapting to the bed shape. Less travel distance speeds up probing.

## Advanced Configuration

### Continuous Tare Filtering

Load cell probes support a filter on the MCU that compensates for drift from external forces such as bowden tubes and umbilical cables. If the probe triggers before touching the bed this is probably the reason why. This is sometimes called *continuous taring* and is intended for toolhead-mounted sensors experiencing variable external forces during a probe.

#### Installing SciPy

The filter is off by default. The [SciPy](https://scipy.org/) library is required to compute the filter coefficients from configuration values. It needs to be installed in the klipper virtual environment. Usually: 

```bash
~/klippy-env/bin/pip install scipy
```

Pre-compiled builds are available for Python 3 on 32-bit Raspberry Pi systems.

#### Filter Tuning

The `drift_filter_cutoff_frequency` parameter should be selected based on observed drift during normal operation.

Basic tuning guidelines:
- Start with `drift_filter_cutoff_frequency: 0.5` Hz
- Prusa uses 0.8 Hz (MK4) and 11.2 Hz (XL); this range is reasonable for experimentation
- Increase only until bowden tube drift is eliminated
- Setting too high causes slow triggering and excessive force
- Keep `trigger_force` low (default 75 g); the drift filter maintains internal readings near zero
- Keep `force_safety_limit` conservative (default 5 kg) during tuning

Tuning of the other filter parameters is beyond the scope of this documentation. 
A Jupyter notebook is provided in [scripts/filter_workbench.ipynb](../scripts/filter_workbench.ipynb) with an example of a detailed analysis.

## Developer Notes

This section covers guidance for developing toolhead boards with load cell probe support.

### ADC Sensor Selection

Recommended sensor characteristics:
- At least 24-bit resolution
- SPI communication
- Data ready (`DRDY`) pin for sample ready indication without SPI queries
- Programmable gain amplifier with 128× gain to eliminate external amplifiers
- SPI reset indication to detect sensor restarts, a common indication of electrical problems
- Selectable sample rate between 350 Hz and 2 kHz (rates below 250 Hz require slower probing speeds and increase toolhead force)
- For under-bed applications with multiple load cells, simultaneous sampling on all channels (multiplexed ADCs have settling delays after channel switches)

Klipper's `bulk_sensor` and `load_cell_probe` infrastructure simplifies support for new sensors. Sensors can be configured from Python. with a minimal sampling loop written in C.

### Power Supply Filtering

Use larger capacitors than ADC manufacturer specifications suggest. ADC datasheets assume low-noise battery-powered environments. 3D printers generate significant 5V bus noise. Test sensors with typical power supplies and active stepper drivers before finalizing capacitor values.

The ADC chip and the load cell should be driven with LDOs. Switching buck converters are not a good fit for this application.

### Grounding

ADC chips are vulnerable to noise and ESD. Use a large ground plane on the first board layer under the chip. Keep the chip away from power sections and DC-DC converters. Ensure proper grounding to the DC supply.

### HX711 and HX717 Notes

These sensors are popular but have limitations:
- Bit-bang communication has high MCU overhead; SPI sensors are more efficient
- Cannot communicate reset events to the MCU, hiding electrical faults
- HX717 (320 Hz) strongly preferred over HX711 (80 Hz) for probing; limit HX711 probing speed to 2 mm/s
- HX711 Sample rate is hardware-configured, not software-configurable; 10 SPS versions must be rewired for 80 SPS
