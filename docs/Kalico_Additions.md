# Kalico additions

## Changes to Klipper defaults

- [`[force_move]`](./Config_Reference.md#force_move) is enabled by default. Use `[force_move] enable_force_move: False` to disable it
- [`[respond]`](./Config_Reference.md#respond) is enabled by default. Use `[respond] enable_respond: False` to disable it
- [`[exclude_object]`](./Config_Reference.md#exclude_object) is enabled by default. Use `[exclude_object] enable_exclude_object: False` to disable it

## Additional configuration options

- [`[mcu] is_non_critical`](./Config_Reference.md#mcu) enables marking of an mcu as optional - it can be freely disconnected and connected at will. (useful for MCU-based accelerometer boards, mcu-based probes that shut down in hot chambers, etc...)
- [`[danger_options]`](./Config_Reference.md#danger-options) - New configuration options to adjust Kalico values that were previously hidden
- Additional kinematics versions enabled per-axis acceleration, see [limited_cartesian](./Config_Reference.md#cartesian-kinematics-with-limits-for-x-and-y-axes) and [limited_corexy](./Config_Reference.md#corexy-kinematics-with-limits-for-x-and-y-axes)
- `--rotate-log-at-restart` can be added to your Kalico start script or service to force log rotation every restart.
- [`[virtual_sdcard] with_subdirs`](./Config_Reference.md#virtual_sdcard) enables scanning of subdirectories for .gcode files, for the menu and M20/M23 commands
- [`[firmware_retraction] z_hop_height`](./Config_Reference.md#firmware_retraction) adds an automatic z hop when using firmware retraction
- [`[constants]` and `${constants.value}`](./Config_Reference.md#configuration-references) allow re-using values in your configuration

## Enhanced behavior

- [`canbus_query.py`](./CANBUS.md#finding-the-canbus_uuid-for-new-micro-controllers) now responds with all Kalico devices, even after they've been assigned a node_id.
- Input shaper calibration now warns about active fans that may affect measurement accuracy.
- [`BED_MESH_CHECK`](./G-Codes.md#bed_mesh_check) validates the current bed mesh against specified criteria, allowing you to check maximum deviation and slope between adjacent points before printing.
- [`[resonance_tester]`](./Config_Reference.md#resonance_tester) now supports multiple accelerometer chips via the new `accel_chips` parameter, allowing data from multiple accelerometers to be combined for more accurate input shaper calibration.

## New Kalico Modules

- [gcode_shell_command](./G-Code_Shell_Command.md) - Execute linux commands and scripts from within Kalico

## Sensorless Homing

- [`[tmcXXXX] home_current`](./Config_Reference.md#tmc-stepper-driver-configuration) automatically sets a different current for homing
- [`[tmcXXXX] current_change_dwell_time`](./Config_Reference.md#tmc-stepper-driver-configuration) will add a delay before homing
- [`[stepper_X] homing_retract_dist, homing_retract_speed`](./Config_Reference.md#stepper) add a short retraction and a second homing for better accuracy
- [`[stepper_X] min_home_dist`](./Config_Reference.md#stepper) will move away from the endstop before homing

## Probes and Probing

- [`[probe] drop_first_result: True`](./Config_Reference.md#probe) will drop the first result when probing. This can improve probe accuracy for printers that have an outlier for the first sample.
- [`[dockable_probe]`](./Config_Reference.md#dockable_probe) brings helpful native support for docked probes, such as the Annex Quickdraw, Klicky/Unklicky, and countless others.
- [`[z_calibration]`](./Config_Reference.md#z_calibration) enables automatic probe Z offset calibration using a reference endstop like the Voron 2.4 nozzle endstop.
- [`[z_tilt_ng]`](./Config_Reference.md#z_tilt_ng) adds enforced 3-point z tilt calibration
- [`[z_tilt/quad_gantry_level] increasing_threshold`](./Config_Reference.md#z_tilt) allows you to customize the allowed variation when probing multiple times
- [`[z_tilt/quad_gantry_level] adaptive_horizontal_move_z`](./Config_Reference.md#z_tilt) adaptively decrease horizontal_move_z based on resulting error - z_tilt and QGL faster and safer!
- [`[safe_z_home] home_y_before_x`](./Config_Reference.md#safe_z_home) let you home Y before X.
- [`[z_tilt/quad_gantry_level/etc] use_probe_xy_offsets`](./Config_Reference.md#z_tilt) let you decide if the `[probe] XY offsets should be applied to probe positions.
- [`SAFE_MOVE`](./G-Codes.md#safe_move) allows protected axis moves using directional endstops.

## Heaters, Fans, and PID changes

- [Model Predictive Control](./MPC.md) is an advanced temperature control method that offers an alternative to traditional PID control.
- [Velocity PID](./PID.md) can be more accurate than positional PID, but is more susceptible to noisy sensors and may require larger smoothing times
- [`PID_PROFILE [LOAD/SAVE]`](./G-Codes.md#pid_profile) allows you to calibrate and save PID profiles at multiple temperatures and fan speeds, and later restore them. With some clever macros, automatic per-material pid tuning is within reach!
- [`SET_HEATER_PID HEATER= KP= KI= KD=`](./G-Codes.md#set_heater_pid) can update your PID parameters without a reload.
- [`HEATER_INTERRUPT`](./G-Codes.md#heater_interrupt) will interrupt a `TEMPERATURE_WAIT`.
- ADC out of range errors now include which heater, and additional information to assist in troubleshooting

- [`[temperature_fan] control: curve`](./Config_Reference.md#temperature_fan) lets you set a fan curve instead of linear control
- [`[temperature_fan] reverse: True`](./Config_Reference.md#temperature_fan) will let you control a fan in reverse to temperature control. The lower the temperature, the higher the fan runs.
- Fans now normalize PWM power within `min_power` and `max_power`, so setting a fan to 10% will get you 10% fan speed within your configured min/max range.
- Dual-loop PID control to accurately manage the bed's temperature while limiting heater power to prevent exceeding a maximum temperature.

## TMC Drivers

- [`[tmc2240] driver_CS and current_range`](./Config_Reference.md#tmc2240) let you tune the current scaler and current range of your tmc2240 drivers.

## Macros

- The jinja `do` extension has been enabled. You can now call functions in your macros without resorting to dirty hacks: `{% do array.append(5) %}`
- The python [`math`](https://docs.python.org/3/library/math.html) library is available to macros. `{math.sin(math.pi * variable)}` and more!
- New [`RELOAD_GCODE_MACROS`](./G-Codes.md#reload_gcode_macros) G-Code command to reload `[gcode_macro]` templates without requiring a restart.
- G-Code Macros can be written in Python. Read more [here](./Command_Templates.md)
  - Macros may also be loaded from other files, using `!!include path/to/file.py`
- Inside a macro you can use `RETURN` to end macro execution early without raising an error.

## Plugins

Extend your Kalico installation with custom plugins.

Your python plugins can now extend [`klippy/extras`](https://github.com/KalicoCrew/kalico/tree/main/klippy/extras) adding new modules to Kalico without causing updates to fail due to a "dirty" git tree.

Enable `[danger_options] allow_plugin_override: True` to override existing extras.
