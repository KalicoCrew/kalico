# Model Predictive Control

Model Predictive Control (MPC) is an advanced temperature control method that offers an alternative to traditional PID control. MPC leverages a system model to simulate the temperature of the hotend and adjusts the heater power to align with the target temperature.  

Unlike reactive methods, MPC operates proactively, adjusting in anticipation of temperature fluctuations. It utilizes a model of the hotend, considering factors such as the thermal masses of the system, heater power, heat loss to ambient air, and fans, and heat transfer into the filament. This model allows MPC to predict the amount of heat energy that will be dissipated from the hotend over a given duration, and it compensates for this by adjusting the heater power accordingly. As a result, MPC can accurately calculate the necessary heat energy input to maintain a steady temperature or to transition to a new temperature.

MPC offers several advantages over PID control:

- **Faster and more responsive temperature control:** MPC’s proactive approach allows it to respond more quickly and accurately to changes in temperature from fans or flow rate changes. 
- **Broad functionality with single calibration:** Once calibrated, MPC functions effectively across a wide range of printing temperatures.  
- **Simplified calibration process:** MPC is easier to calibrate compared to traditional PID control. 
- **Compatibility with all hotend sensor types:** MPC works with all types of hotend sensors, including those that produce noisy temperature readings.
- **Versatility with heater types:** MPC performs well with standard cartridge heaters and PTC heaters.
- **Effective for high and low flow hotends:** Regardless of the flow rate of the hotend, MPC maintains effective temperature control.     

> [!CAUTION]
> This feature controls the portions of the 3D printer that can get very hot. All standard Kalico warnings apply. Please report all issues and bugs to [GitHub](https://github.com/KalicoCrew/kalico/issues) or [Discord](Contact.md#discord).

# Basic Configuration

To use MPC as the temperature controller for the extruder use the following basic configuration block.

```
[extruder]
control: mpc
heater_power: 50  
cooling_fan:
filament_diameter: 1.75
filament_density: 1.20
filament_heat_capacity: 1.8 
```

- `control: mpc`  
  *Required*  
  The temperature control method.
  
- `heater_power: 50`  
  *Required*   
  The nameplate heater power in watts.  
  For a PTC, a non-linear heater, MPC may not work optimally due
  to the change in power output relative to heater temperature for this style of
  heater. Setting heater_power to the power output at the expected printing
  temperature is recommended.
  
- `cooling_fan:`  
  _Default Value: Nothing_  
  The fan that is cooling extruded filament and the hotend. Default is no fan so 
  there will be no fan taken into account for controlling the heater.
  Specifying "fan" will automatically use the part cooling fan.
  
- `filament_diameter: 1.75`  
  _Default Value: 1.75 (mm)_  
  This is the filament diameter.  
  
- `filament_density: 1.20`   
  _Default Value: 1.20 (g/mm^3)_  
  This is the material density of the filament being printed.
  
- `filament_heat_capacity: 1.80`  
  _Default Value: 1.80 (J/g/K)_  
  This is the material specific heat capacity of the filament being printed.  

## Optional Config Parameters

These can be specified in the config but should not need to be changed from the default values for most users.

- `maximum_retract:`  
  _Default Value: 2.0 (mm)_  
  This value clamps how much the extruder is allowed to go backwards in a single period during MPC FFF calculations. This lets the filament power go negative and add a small amount of energy to the system.  

- `target_reach_time:`  
  _Default Value: 2.0 (sec)_  
 
- `smoothing:`  
  _Default Value: 0.83 (sec)_  
  This parameter affects how quickly the model learns and it represents the ratio of temperature difference applied per second. A value of 1.0 represents no smoothing used in the model.  
  
- `min_ambient_change:`  
  _Default Value: 1.0 (deg C/s)_  
  Larger values of MIN_AMBIENT_CHANGE will result in faster convergence but will also cause the simulated ambient temperature to flutter somewhat chaotically around the ideal value.  
  
- `steady_state_rate:`  
  _Default Value: 0.5 (deg C/s)_  
  
- `ambient_temp_sensor: temperature_sensor <sensor_name>`  
  _Default Value: MPC ESTIMATE_  
  It is recommended not to specify this parameter and let MPC will estimate. This is used for initial state temperature and calibration but not for actual control.
  Any temperature sensor could be used, but the sensor should be in proximity to the hotend or measuring the ambient air surrounding the hotend.  

## PTC Heater Power

The `heater power:` for PTC style heaters is recommended to be set at the normal print temperature for the printer. Some common PTC heaters are given below for reference. If your heater is not listed the manufacturer should be able to provide a temperature and power curve.

| Heater Temp (C) | Rapido 2 (W) | Rapido 1 (W) | Dragon Ace old (W) | Dragon Ace new (W) | Revo 40 (W) |Revo 60 (W) |
|:---------------:|:------------:|:------------:|:------------------:|:------------------:|:-----------:|:----------:|
| 180             | 72           | 52           | 51                 | 66                 | 30          |45          |
| 200             | 70           | 51           | 48                 | 63                 | 29          |44          |
| 220             | 67           | 50           | 46                 | 60                 | 28          |43          |
| 240             | 65           | 49           | 44                 | 58                 | 28          |42          |
| 260             | 64           | 48           | 43                 | 55                 | 27          |40          |
| 280             | 62           | 47           | 41                 | 53                 | 27          |39          |
| 300             | 60           | 46           | 39                 | 51                 | 26          |38          |

## Filament Feed Forward Configuration

The filament feed forward (FFF) feature allows MPC to look forward and see changes in extrusion rates which could require more or less heat input to maintain target temperature. This feature substantially improves the accuracy and responsiveness of the model during printing. It is enabled by default and can be defined is more detail with the `filament_density` and `filament_heat_capacity` config parameters. The default values are set to cover a wide range of standard materials including ABS, ASA, PLA, PETG. 

 FFF parameters can be set, for the printer session, via the `MPC_SET` G-Code command:  

`MPC_SET HEATER=<heater> FILAMENT_DENSITY=<value> FILAMENT_HEAT_CAPACITY=<value> [FILAMENT_TEMP=<sensor|ambient|<value>>]`

- `HEATER`:  
  Only extruder is supported
  
- `FILAMENT_DENSITY`:  
  Filament density in g/mm^3
  
- `FILAMENT_HEAT_CAPACITY`:  
  Filament heat capacity in J/g/K
  
- `FILAMENT_TEMP`:  
  This can be set to either `sensor`, `ambient`, or a set temperature value. FFF will use the specific energy required to heat the filament and the power loss will be calculated based on the temperature delta.  

For example, updating the filament material properties for ASA would be:   

```
MPC_SET HEATER=extruder FILAMENT_DENSITY=1.07 FILAMENT_HEAT_CAPACITY=1.7  
```

## Filament Physical Properties

MPC works best knowing how much energy (in Joules) it takes to heat 1mm of filament by 1°C. The material values from the tables below have been curated from popular filament manufacturers and material data references. These values are sufficient for MPC to implement the FFF feature.  Advanced users could tune the `filament_density` and `filament_heat_capacity` parameters based on manufacturers datasheets. 

### Common Materials

| Material | Density [g/cm³] | Specific heat [J/g/K] |
| -------- |:---------------:|:---------------------:|
| PLA      | 1.25            | 1.8 - 2.2             |
| PETG     | 1.27            | 1.7 - 2.2             |
| PC+ABS   | 1.15            | 1.5 - 2.2             |
| ABS      | 1.06            | 1.25 - 2.4            |
| ASA      | 1.07            | 1.3 - 2.1             |
| PA6      | 1.12            | 2 - 2.5               |
| PA       | 1.15            | 2 - 2.5               |
| PC       | 1.20            | 1.1 - 1.9             |
| TPU      | 1.21            | 1.5 - 2               |
| TPU-90A  | 1.15            | 1.5 - 2               |
| TPU-95A  | 1.22            | 1.5 - 2               |

### Common Carbon Fiber Filled Materials

| Material                                     | Density [g/cm³] | Specific heat [J/g/K] |
| -------------------------------------------- |:---------------:|:---------------------:|
| ABS-CF                                       | 1.11            | ^                     |
| ASA-CF                                       | 1.11            | ^                     |
| PA6-CF                                       | 1.19            | ^                     |
| PC+ABS-CF                                    | 1.22            | ^                     |
| PC+CF                                        | 1.36            | ^                     |
| PLA-CF                                       | 1.29            | ^                     |
| PETG-CF                                      | 1.30            | ^                     |  

^ Use the specific heat from the base polymer  

# Calibration

The MPC default calibration routine takes the following steps:

> 1. Cool to ambient: The calibration routine needs to know the approximate ambient temperature and waits until the hotend temperature stabilises and stops decreasing relative to ambient.
> 2. Heat past 200°C: Measure the point where the temperature is increasing most rapidly, and the time and temperature at that point. Also, three temperature measurements are needed at some point after the initial latency has taken effect. 
> 3. Hold temperature while measuring ambient heat-loss: At this point enough is known for the MPC algorithm to engage. The calibration routine makes a best guess at the overshoot past 200°C which will occur and targets this temperature for about a minute while ambient heat-loss is measured without and with the fan engaged (if a `cooling_fan` is specified).
> 4. The MPC calibration routine creates the appropriate model constants. At this time the model parameters are temporary and not yet saved to the printer configuration.  

The MPC calibration routine must be run for each heater, to be controlled by MPC, in order to determine the model parameters. For an MPC calibration to be successful an extruder must be able to reach 200C. Calibration is performed with the following G-code command.

`MPC_CALIBRATE HEATER=<heater> [TARGET=<temperature>] [FAN_BREAKPOINTS=<value>]`  

- `HEATER=<heater>`:  
  The extruder heater to be calibrated.  
  
- `TARGET=<temperature>`:  
  _Default Value: 200 (deg C)_  
  Sets the calibration temperature. The default of 200C is a good target for the extruder. MPC calibration is temperature independent, so calibrating the extruder at higher temperatures will not necessarily produce better model parameters. This is an area of exploration for advanced users.  
  
- `FAN_BREAKPOINTS=<value>`:  
  _Default Value: 3_  
  Sets the number off fan setpoint to test during calibration. An arbitrary number of breakpoints can be specified e.g. 7 breakpoints would result in (0, 16%, 33%, 50%, 66%, 83%, 100%) fan speeds.
  It is recommended to use a number that will capture one or more test points below the lowest level of fan normally used. For example, if 20% fan is the lowest commonly used speed, using 11 break points is recommended to test 10% and 20% fan at the low range.  
  
Default calibration of the hotend with seven fan breakpoints:  
```
MPC_CALIBRATE HEATER=extruder FAN_BREAKPOINTS=7
```
> [!NOTE]
> Ensure that the part cooling fan is off before starting calibration.  

After successful  calibration the method will generate the key model parameters into the log for future reference.  

![Calibration Parameter Output](img/MPC_calibration_output.png)

A `SAVE_CONFIG` command is then required to commit these calibrated model parameters to the printer config or the user can manually update the values. The _SAVE_CONFIG_ block should then look like: 

```
#*# <----------- SAVE_CONFIG ----------->
#*# DO NOT EDIT THIS BLOCK OR BELOW. The contents are auto-generated.
#*# [extruder]
#*# control = mpc
#*# block_heat_capacity = 22.3110
#*# sensor_responsiveness = 0.0998635
#*# ambient_transfer = 0.155082
#*# fan_ambient_transfer=0.155082, 0.20156, 0.216441
```

> [!NOTE]
> If the [extruder] section is in a .cfg file other than printer.cfg the `SAVE_CONFIG` command may not be able to write the calibration parameters and klippy will provide an error. 

These model parameters are not suitable for pre-configuration or are not explicitly determinable. Advanced users could tweak these post calibration based on the following guidance: Slightly increasing these values will increase the temperature where MPC settles and slightly decreasing them will decrease the settling temperature.  

- `block_heat_capacity:`  
  Heat capacity of the heater block in (J/K).  
  
- `ambient_transfer:`  
  Heat transfer from heater block to ambient in (W/K).  
  
- `sensor_responsiveness:`  
  A single constant representing the coefficient of heat transfer from heater block to sensor and heat capacity of the sensor in (K/s/K).  
  
- `fan_ambient_transfer:`  
  Heat transfer from heater block to ambient in with fan enabled in (W/K).  
  
# Support Macros

## Temperature Wait

The following macro can be used to replace `M109` hotend temperature set and `M190` bed temperature set G-code commands with a macro utilizing `temperature_wait` G-codes. This can be utilized in systems where the sensor temperature takes an extended time to converge on the set temperature. 
> [!NOTE]
> This behaviour occurs primarily because MPC controls the modelled block temperature and not the hotend temperature sensor. For almost all cases, when temperature sensor overshoot/undershoot occurs, the block modelled temperature will be correctly at the set temperature. However, the Kalico system performs actions based on the sensor temperature only which can lead to undesirable delays in print actions with stock `M109` and `M190` commands.

```
[gcode_macro M109] # Wait Hotend Temp
rename_existing: M109.1
gcode:
    #Parameters
    {% set s = params.S|float %}

    M104 {% for p in params %}{'%s%s' % (p, params[p])}{% endfor %}  # Set hotend temp
    {% if s != 0 %}
        TEMPERATURE_WAIT SENSOR=extruder MINIMUM={s-2} MAXIMUM={s+5}   # Wait for hotend temp (within n degrees)
    {% endif %}


[gcode_macro M190] # Wait Bed Temp
rename_existing: M190.1
gcode:
    #Parameters
    {% set s = params.S|float %}

    M140 {% for p in params %}{'%s%s' % (p, params[p])}{% endfor %}   # Set bed temp
    {% if s != 0 %}
        TEMPERATURE_WAIT SENSOR=heater_bed MINIMUM={s-2} MAXIMUM={s+5}  # Wait for bed temp (within n degrees)
    {% endif %}
```

### Setting FFF Parameters From The Slicer

This macro will set FFF parameters automatically when the material type is passed from the slicer. 

```ini
[gcode_macro _SET_MPC_MATERIAL]
description: Set heater MPC parameters for a given material
variable_filament_table:
    ## Update this table to adjust material settings
    {
        ## ( density, heat capacity )  # suggested heat capacity range
        "PLA"       : ( 1.25, 2.20 ),  # 1.80 - 2.20
        "PETG"      : ( 1.27, 2.20 ),  # 1.70 - 2.20
        "PC+ABS"    : ( 1.15, 2.20 ),  # 1.50 - 2.20
        "ABS"       : ( 1.06, 2.40 ),  # 1.25 - 2.40
        "ASA"       : ( 1.07, 2.10 ),  # 1.30 - 2.10
        "PA6"       : ( 1.12, 2.50 ),  # 2.00 - 2.50
        "PA"        : ( 1.15, 2.50 ),  # 2.00 - 2.50
        "PC"        : ( 1.20, 1.90 ),  # 1.10 - 1.90
        "TPU"       : ( 1.21, 2.00 ),  # 1.50 - 2.00
        "TPU-90A"   : ( 1.15, 2.00 ),  # 1.50 - 2.00
        "TPU-95A"   : ( 1.22, 2.00 ),  # 1.50 - 2.00
        "ABS-CF"    : ( 1.11, 2.40 ),  # 1.25 - 2.40
        "ASA-CF"    : ( 1.11, 2.10 ),  # 1.30 - 2.10
        "PA6-CF"    : ( 1.19, 2.50 ),  # 2.00 - 2.50
        "PC+ABS-CF" : ( 1.22, 2.20 ),  # 1.50 - 2.20
        "PC+CF"     : ( 1.36, 1.90 ),  # 1.10 - 1.90
        "PLA-CF"    : ( 1.29, 2.20 ),  # 1.80 - 2.20
        "PETG-CF"   : ( 1.30, 2.20 ),  # 1.70 - 2.20
    }
gcode:
    {% set material = params.MATERIAL | upper %}
    {% set heater = params.HEATER | default('extruder') %}
    {% set extruder_config = printer.configfile.settings[heater] %}

    {% if material in filament_table %}
        {% set (density, heat_capacity) = filament_table[material] %}

        RESPOND PREFIX=🔥 MSG="Configured {heater} MPC for {material}. Density: {density}, Heat Capacity: {heat_capacity}"
    {% else %}
        {% set density = extruder_config.filament_density %}
        {% set heat_capacity=extruder_config.filament_heat_capacity %}

        RESPOND PREFIX=🔥 MSG="Unknown material '{material}', using default mpc parameters for {heater}"
    {% endif %}

    MPC_SET HEATER={heater} FILAMENT_DENSITY={density} FILAMENT_HEAT_CAPACITY={heat_capacity}
```

The slicer must be configured to pass the current material type to your `PRINT_START` macro. For PrusaSlicer you should add the following parameter line to `print_start` in the Start G-Code section:

```
MATERIAL=[filament_type[initial_extruder]]
```

The print_start line, in PrusaSlicer, would look like:

```
start_print MATERIAL=[filament_type[initial_extruder]] EXTRUDER_TEMP={first_layer_temperature[initial_extruder]} BED_TEMP={first_layer_bed_temperature[initial_extruder]} CHAMBER_TEMP={chamber_temperature}
```

Then, in your `PRINT_START` macro include the following macro call:

```
_SET_MPC_MATERIAL MATERIAL={params.MATERIAL}
```

# Real-Time Model State

The real-time temperatures and model states can be viewed from a browser by entering the following local address for your computer.

```
https://192.168.xxx.xxx:7125/printer/objects/query?extruder
```

![Calibration](img/MPC_realtime_output.png)

# EXPERIMENTAL FEATURES

## Bed Heater

Using MPC for bed heater control is functional but the performance is not guaranteed or currently supported.  MPC for the bed can be configured simply.

```
[heater_bed]
control: mpc
heater_power: 400
```

- `control: mpc`  
  *Required*  
  The temperature control method.  
  
- `heater_power: 50`  
  *Required*  
  The nameplate heater power in watts.  
  
- `cooling_fan: fan_generic <fan_name>`  
  _No Default Value_  
  This is the fan cooling the bed. Optional parameter to support bed fans.  

The bed should be able to reach at least 90C to perform calibration with the following G-code. 

`MPC_CALIBRATE HEATER=<heater> [TARGET=<temperature>] [FAN_BREAKPOINTS=<value>]`  

- `HEATER=<heater>`:  
  The bed heater to be calibrated.  
  
- `TARGET=<temperature>`:  
  _Default Value: 90 (deg C)_  
  Sets the calibration temperature. The default of 90C is a good target for the bed.  
  
- `FAN_BREAKPOINTS=<value>`:  
  _Default Value: 3_  
  Sets the number of fan setpoint to test during calibration.    

Default calibration of the hotend with five fan breakpoints:  
```
MPC_CALIBRATE HEATER=heater_bed FAN_BREAKPOINTS=5
```

These calibrated model parameters need to be saved to the _SAVE_CONFIG_ block manually or by using the `SAVE_CONFIG` command.

## Updating calibration parameters at runtime

Similar to [`SET_HEATER_PID`](G-Codes.md#set_heater_pid), you can update your MPC calibration profile at runtime.

`MPC_SET HEATER=<heater_name> [BLOCK_HEAT_CAPACITY=0.0] [SENSOR_RESPONSIVENESS=0.0] [AMBIENT_TRANSFER=0.0] [FAN_AMBIENT_TRANSFER=0.01,0.02,0.03]`

# BACKGROUND

## MPC Algorithm

MPC models the hotend system as four thermal masses: ambient air, the filament, the heater block and the sensor. Heater power heats the modelled heater block directly. Ambient air heats or cools the heater block. Filament cools the heater block. The heater block heats or cools the sensor.  

Every time the MPC algorithm runs it uses the following information to calculate a new temperature for the simulated hotend and sensor:  

- The last power setting for the hotend.  
- The present best-guess of the ambient temperature.  
- The effect of the fan on heat-loss to the ambient air.  
- The effect of filament feedrate on heat-loss to the filament. Filament is assumed to be at the same temperature as the ambient air.  

Once this calculation is done, the simulated sensor temperature is compared to the measured temperature and a fraction of the difference is added to the modelled sensor and heater block temperatures. This drags the simulated system in the direction of the real system. Because only a fraction of the difference is applied, sensor noise is diminished and averages out to zero over time. Both the simulated and the real sensor exhibit the same (or very similar) latency. Consequently, the effects of latency are eliminated when these values are compared to each other. So, the simulated hotend is only minimally affected by sensor noise and latency.   

SMOOTHING is the factor applied to the difference between simulated and measured sensor temperature. At its maximum value of 1, the simulated sensor temperature is continually set equal to the measured sensor temperature. A lower value will result in greater stability in MPC output power but also in decreased responsiveness. A value around 0.25 seems to work quite well.  

No simulation is perfect and, anyway, real life ambient temperature changes. So MPC also maintains a best guess estimate of ambient temperature. When the simulated system is close to steady state the simulated ambient temperature is continually adjusted. Steady state is determined to be when the MPC algorithm is not driving the hotend at its limits (i.e., full or zero heater power) or when it is at its limit but temperatures are still not changing very much - which will occur at asymptotic temperature (usually when target temperature is zero and the hotend is at ambient).  

Steady_state_rate is used to recognize the asymptotic condition. Whenever the simulated hotend temperature changes at an absolute rate less than steady_state_rate between two successive runs of the algorithm, the steady state logic is applied. Since the algorithm runs frequently, even a small amount of noise can result in a fairly high instantaneous rate of change of hotend temperature. In practice 1°C/s seems to work well for steady_state_rate.  

When in steady state, the difference between real and simulated sensor temperatures is used to drive the changes to ambient temperature. However, when the temperatures are really close min_ambient_change ensures that the simulated ambient temperature converges relatively quickly. Larger values of min_ambient_change will result in faster convergence but will also cause the simulated ambient temperature to flutter somewhat chaotically around the ideal value. This is not a problem because the effect of ambient temperature is fairly small and short-term variations of even 10°C or more will not have a noticeable effect.  

It is important to note that the simulated ambient temperature will only converge on real world ambient temperature if the ambient heat transfer coefficients are exactly accurate. In practice this will not be the case and the simulated ambient temperature therefore also acts a correction to these inaccuracies.  

Finally, armed with a new set of temperatures, the MPC algorithm calculates how much power must be applied to get the heater block to target temperature in the next two seconds. This calculation takes into account the heat that is expected to be lost to ambient air and filament heating. This power value is then converted to a PWM output.  

## Additional Details

Please refer to that the excellent Marlin MPC Documentation for information on the model derivations, tuning methods, and heat transfer coefficients used in this feature.   

# Acknowledgements

This feature is a port of the Marlin MPC implementation, and all credit goes to their team and community for pioneering this feature for open source 3D printing. The Marlin MPC documentation and github pages were heavily referenced and, in some cases directly copied and edited to create this document.  

- Marlin MPC Documentation: [https://marlinfw.org/docs/features/model_predictive_control.html]
- GITHUB PR that implemented MPC in Marlin: [https://github.com/MarlinFirmware/Marlin/pull/23751]
- Marlin Source Code: [https://github.com/MarlinFirmware/Marlin]
