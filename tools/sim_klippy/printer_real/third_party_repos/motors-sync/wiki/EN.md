## Welcome to the stepper motor synchronization project for AWD systems

### Supported kinematics:
`CoreXY` / `Cartesian` `4 WD`

### How does it work?
In AWD systems, a single belt is driven by two motors and split into two
loops by pulleys. When activated, motors fail to restore their previous
position correctly: one loop stretches while the other slackens.
Synchronization aligns the tension of these two loops. When one motor is
disabled, loop tension equalizes, and its rotor rotates. Upon reactivation,
this rotor returns to its previous position, causing a distinct impact.
The impact's force depends on the tension difference between the loops.
The greater the difference, the stronger the impact; with minimal
difference, there is no impact. The impact magnitude is measured by an
accelerometer fixed on the carriage, or an encoder mounted to a stepper.
Synchronization is software-based and resets when any stepper motor in the
belt loop is disabled. The script determines the rotor's rotation direction
and gradually adjusts its position until the impact magnitude decreases.
If the magnitude starts to increase, it reverts to the previous position,
completing synchronization or performs additional iterations to reach a
set magnitude threshold n-times.

### Notes:
* It is recommended to securely mount the accelerometer at the closest
  point to the carriage belt attachment. Mounting it elsewhere, like
  using a CAN-board or a Beacon accelerometer on a flexible mount, may
  distort measurements.

* Avoid using the `lis2dw` accelerometer due to its low sampling rate,
  which may poorly detect vibration peaks, however its operation has been
  optimized by disabling data filtering.

* Do not enable the hotend heater during synchronization. A running fan
  (or any fan in the printer) may interfere with accurate measurement.
  Compare noise levels using Klipper's standard command:
  [MEASURE_AXES_NOISE](https://www.klipper3d.org/G-Codes.html#measure_axes_noise).

* In most cases, avoid setting the sync parameter `microsteps` above 16
  for a 20t pulley; the magnitude difference between rotor positions will
  be negligible, potentially leading to false calibration.

### Installing the calibration script on the printer host
```
cd ~
git clone https://github.com/MRX8024/motors-sync
bash ~/motors-sync/install.sh
```

### Configuration
Most parameters support self-assignment to an axis, e.g., for `axes: x,y`,
the parameter `accel_chip` can be described as `accel_chip_x` and
`accel_chip_y`. However, `accel_chip` remains the default parameter if no
axis-specific value is defined. This is relevant for `cartesian` kinematics,
which may have different implementations of drives.

Parameters starting with `#` are optional. They have default values listed
next to them but can be adjusted for different printer configurations.
It is recommended to review all configuration parameters before using the
program.

Add the following section to the printer's configuration file and partially
configure it for the first measurement:

```
[motors_sync]
axes: x,y
#    Axes on which calibration will be performed.
accel_chip:
#    Accelerometer for vibrations collection: adxl345 / mpu9250 etc.
#encoder_chip_<axis>:
#    Axis, assigned encoder name, for measuring deviations.
#chip_filter: median
#    Filter type for data from the accelerometer: 'median' works well in
#    most cases, but some particularly noisy printers (fans, etc.) may
#    require a more powerful filter - 'kalman'. On lis2dw filters disabled.
#median_size: 3
#    Median filter window size.
#kalman_coeffs: 1.1, 1., 1e-1, 1e-2, .5, 1.
#    Simple coefficients describing the kalman filter.
#microsteps: 16
#    Maximum microstepping displacement of the stepper motor rotor.
#steps_model: linear, 20000, 0
#    Mathematical model and its coefficients representing the dependence
#    of stepper motor microstep displacement on the measured magnitude.
#max_step_size: 3
#    The maximum number of microsteps that the motor can take move at time,
#    to achieve the planned magnitude.
#axes_steps_diff: 4
#    The difference in motor positions in microsteps between two axes, used
#    to trigger a check for magnitude shift on the secondary axis in corexy
#    like kinematics. Default value: max_step_size + 1.
#retry_tolerance: 0
#    The forced threshold to which a pair of stepper motors on one belt
#    will have to lower the magnitude of the oscillations. It's recommended
#    to configure in order to filter possible inaccuracies. After several
#    iterations of starting synchronization, you will find the edge, to
#    which this parameter should be omitted.
#retries: 0
#    The maximum number of repetitions to achieve the forced threshold of
#    oscillations.
#head_fan:
#    Full name of the toolhead fan that will be turned off during sync to
#    eliminate its noise. Can be specified multiple fans by separating
#    their names with commas.
#axis_prefix_<axis>:
#    Allows you to set a custom prefix that will be displayed before the
#    axis name in terminal outputs. You can use special characters or
#    Unicode icons to make the axes easier to distinguish.
```

### Motor synchronization
Enter the `SYNC_MOTORS` command in the terminal on the main web page
interface and wait for the completion of the process. Some parameters can
be overridden:
```
SYNC_MOTORS AXES=[<axes>] ACCEL_CHIP=[<chip_name>] [RETRY_TOLERANCE=<value>] [RETRIES=<value>]
```
These can also be specified per axis, for example `ACCEL_CHIP_X`.
Otherwise, the parameter will override values for the selected or all axes.

### Automation
Synchronization is typically performed at the start of printing during
printer preheating. Add it to a macro or slicer. For example:
```
...
M140 S   ; set bed temp
SYNC_MOTORS
G28 Z
M190 S   ; wait for bed temp to stabilize
M104 S   ; set extruder temp
...
```

### Calibration status variable
A calibration status variable is introduced, which resets when the
printer's motors are turned off. You can check its state in a macro.
If the status is positive, skip unnecessary calibration if it has already
been performed in the current session. For example:
```
...
G28 X Y
{% if not printer.motors_sync.applied %}
    SYNC_MOTORS
{% endif %}
G28 Z
...
```

### Synchronization model calibration
The model represents the dependence of microsteps on magnitude. By default,
it is linear with a coefficient of 20k, meaning that for every 20k of
magnitude, there is one microstep of displacement. This is a basic and safe
value, but for each user, depending on their printer configuration and even
the accelerometer, this dependence will vary. The purpose of model
calibration is to find the appropriate function and its coefficients that 
describe the dependence of microsteps on magnitude specifically for your
printer.

```
SYNC_MOTORS_CALIBRATE AXIS=<axis> [DISTANCE=<value>] [REPEATS=<value>] [PLOT=<0/1>]
```

`AXIS` specifies the axis you wish to calibrate. For `corexy`, choose either
`x` or `y`; the calibration will be unified for both axes. By default, two
iterations are performed, increasing the rotor position by +16/16 step and
then decreasing it to -16/16 in 1/16 step increments, during which impacts
are measured and checkpoints are recorded. Calibration is performed in 1/16
steps to ensure result stability. However, the model will automatically
scale to match the number of microsteps in your configuration if it differs
from 16. `DISTANCE` sets relative desired microstep displacement, relative
to 16. `REPEATS` specifies the number of repetitions. `PLOT` - whether to
generate a graph, by default is generating.

After the calibration is complete, you will see a prompt in the terminal to
save the model to the config, along with the path to the graphical output.
Open the file to see something like this:

<img src="/wiki/pictures/model_calibrate.png" width="600">

The graph's table displays model names, coefficients, and RMSE
(Root Mean Square Error) from the measured points, sorted in ascending
order. If you are unsure whether the algorithm worked correctly in your
case, for example, function lines do not repeat your data well (dots) -
open issue and attach a graph with other data.

For `corexy` kinematics, the model is saves to both axes by default. 
For other kinematics, choose one of the following methods:
* Calibrate the model for each axis individually, if that makes sense;
* Manually assign the calibrated model to the desired axes:
  ```
  steps_model_a: ...
  steps_model_b: ...
  steps_model_c: ...
  ```
* Manually assign a common model to all axes:
  ```
  steps_model: ...
  ```
It is worth remembering that the value for a specific axis is more
significant than for all axes, and will override it.

### Calibration statistics
Each synchronization iteration is logged in a journal located in the
script directory. This allows the motor positions to be restored from the
last successful synchronization upon the first run sync after powering on the
printer, and also enables viewing sync statistics using the following command:
```
SYNC_MOTORS_STATS
```
To clear the journal, use the command:
```
SYNC_MOTORS_STATS CLEAR=true
```

### Encoder-based synchronization  
Encoders offer high precision down to 1/128 of a step, are independent of
the printer’s spatial orientation, and perform the synchronization process
faster. Any encoders listed in the [Klipper wiki
](https://www.klipper3d.org/Config_Reference.html#angle) are supported and
can be installed on any motor in the belt loop. After installation, it is
recommended to perform any available internal sensor calibrations, and then
calibration by klipper. By default, the number of microsteps per step is
determined by the rotor shaft deviation length. However, the synchronization
model can be calibrated similarly to accelerometer-based calibration. The
`max_step_size` parameter value can be increased, as this tends to provide
more stable operation compared to accelerometer and can significantly speed
up the process. However, on some printers, higher values may still lead to
unpredictable results.

### Contacts
Use the issue GitHub system.
You can also write to me personally in discord - @mrx8024.

### A bit of history:
The project idea emerged in the summer of 2023. Its first author,
@altzbox, realized that an accelerometer could measure impact force when
activating motors. The community was unenthusiastic about the idea, and 
insufficient programming skills hindered solo implementation. Six months
later, tired of manual motor synchronization, @altzbox wrote code using
ChatGPT, leading to the first working [version](
https://github.com/altzbox/motors_sync) in January 2024. However, the
skills were no longer enough for the further development of the project.
In spring 2024, I (@mrx8024) became interested in the idea and decided to
continue development.

