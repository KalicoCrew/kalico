# Model Predictive Control

Model Predictive Control (MPC) is an advanced temperature control method that offers an alternative to traditional PID control. MPC leverages a system model to simulate the temperature of the hotend and adjusts the heater power to align with the target temperature.  

Unlike reactive methods, MPC operates proactively, making adjustments in anticipation of temperature fluctuations. It utilizes a model of the hotend, taking into account factors such as the thermal masses of the system, heater power, heat loss to ambient air, and fans, and heat transfer into the filament. This model allows MPC to predict the amount of heat energy that will be dissipated from the hotend over a given duration, and it compensates for this by adjusting the heater power accordingly. As a result, MPC can accurately calculate the necessary heat energy input to maintain a steady temperature or to transition to a new temperature.

MPC offers several advantages over PID control:

- **Faster and more responsive temperature control:** MPC’s proactive approach allows it to respond more quickly and accurately to changes in temperature. 
- **Broad functionality with single calibration:** Once calibrated, MPC functions effectively across a wide range of printing temperatures.  
- **Simplified calibration process:** MPC is easier to calibrate compared to traditional PID control. 
- **Compatibility with all hotend sensor types:** MPC works with all types of hotend sensors, including those that produce noisy temperature readings.
- **Versatility with heater types:** MPC performs equally well with standard cartridge heaters and PTC heaters.
- **Applicable to both hotends and beds:** MPC can be used to control the temperature of both hotends and beds.
- **Effective for high and low flow hotends:** Regardless of the flow rate of the hotend, MPC maintains effective temperature control.     

> [!CAUTION]
> This feature controls the portions of the 3D printer that can get very hot. All standard Danger Klipper warnings apply. Please report all issues and bugs to github or discord.

# Configuration

To use MPC as the temperature controller set the following configuration parameters in the appropriate heater section of the config.   
Currently only [extruder] and [heater_bed] heater types are supported.  

```
[extruder] OR [heater_bed]
control: mpc
heater_power:
#   Nameplate heater power in watts. 

#   Note that for a PTC, a non-linear heater, MPC may not work optimally due
#   to the change in power output relative to temperature for this style of
#   heater. Setting heater_power to the power output at the expected printing
#   temperature is recommended.
cooling_fan: fan 
#   This is the fan that is cooling extruded filament and the hotend.
#   cooling_fan is supported for [heater_bed] but accurate performance has
#   not been verified.
#   Specifying "fan" will automatically use the part cooling fan.
#   Bed fans could be used for the [heater_bed] by specifying
#   <fan_generic BedFans> for example.
#ambient_temp_sensor: <temperature_sensor sensor_name>
#   Optional. If this is not given MPC will estimate this parameter
#   (reccomended).
#   It can use any sensor, but it should be a temperature sensor in proximity
#   to the hotend or measuring the ambient air surrounding the hotend such as
#   a chamber sensor. 
#   This is used for initial state temperature and calibration but not for
#   actual control.
#   Example sensor: temperature_sensor beacon_coil
```

## Filament Feed Forward Configuration


MPC can look forward to changes in extrusion rates which could require more or less heat input to maintain target temperatures. This substantially improves the accuracy and responsiveness of the model during printing. 


These should only be set under [extruder] and are not valid for [heater_bed]. 

```
#filament_diameter: 1.75
#   (mm)
#filament_density: 1.2
#   1.2 (g/mm^3) is the default to cover a wide range of standard
#   materials including ABS, ASA, PLA, PETG.
#filament_heat_capacity: 1.8
#   1.8 (J/g/K) is the default to cover a wide range of standard
#   materials including ABS, ASA, PLA, PETG.
```

Filament feed forward parameters can be set, for the printer session, via the command line or custom G-Code with the following command.

`MPC_SET HEATER=<heater> FILAMENT_DENSITY=<value> FILAMENT_HEAT_CAPACITY=<value>`

`HEATER=<heater>`: Only [extruder] is supported.

`FILAMENT_DENSITY=<value> `:  Filament density in g/mm^3

`FILAMENT_HEAT_CAPACITY=<value>`: Filament heat capacity in J/g/K

For example, updating the filament material properties for ASA would be:   

```
MPC_SET HEATER=extruder FILAMENT_DENSITY=1.07 FILAMENT_HEAT_CAPACITY=1.7  
```

## Optional model parameters

These can be tuned but should not need changing from the default values. 

```
#target_reach_time: 2.0
#   (sec) 
#smoothing: 0.83
#   (sec)
#   This parameter affects how quickly the model learns and it represents
#   the ratio of temperature difference applied per second.
#   A value of 1.0 represents no smoothing used in the model.
#min_ambient_change: 1.0
#   (deg C)
#   Larger values of MIN_AMBIENT_CHANGE will result in faster convergence
#   but will also cause the simulated ambient temperature to flutter
#   somewhat chaotically around the ideal value.  
#steady_state_rate: 0.5
#   (deg C/s)
```

## Example configuration block

```
[extruder]
control: mpc
heater_power: 50  
cooling_fan: fan
filament_density: 1.20
filament_heat_capacity: 1.8

[heater_bed]
control: mpc
heater_power: 400  
```

> [!IMPORTANT]
> Restart the firmware to enable MPC and proceed to calibration.

# Calibration

The MPC default calibration routine takes the following steps:

- Cool to ambient: The calibration routine needs to know the approximate ambient temperature. It switches the part cooling fan on and waits until the hotend temperature stops decreasing relative to ambient.
- Heat past 200°C: Measure the point where the temperature is increasing most rapidly, and the time and temperature at that point. Also, three temperature measurements are needed at some point after the initial latency has taken effect. The tuning algorithm heats the hotend to over 200°C.
- Hold temperature while measuring ambient heat-loss: At this point enough is known for the MPC algorithm to engage. The calibration routine makes a best guess at the overshoot past 200°C which will occur and targets this temperature for about a minute while ambient heat-loss is measured without (and optionally with) the fan.
- MPC calibration routine creates the appropriate model constants and saves them for use. At this time the model parameters are temporary and not yet saved to the printer configuration via SAVE_CONFIG.  

## Hotend or Bed Calibration

The MPC calibration routine has to be run initially for each heater to be controlled using MPC. In order for MPC to be functional an extruder must be able to reach 200C and a bed to reach 90C.

`MPC_CALIBRATE HEATER=<heater> [TARGET=<temperature>] [FAN_BREAKPOINTS=<value>]`

`HEATER=<heater>` :The heater to be calibrated. [extruder] or [heater_bed] supported.

`[TARGET=<temperature>]` : Sets the calibration temperature in degrees C. TARGET default is 200C for extruder and 90C for beds. MPC calibration is temperature independent so calibration the extruder at higher temperatures will not necessarily produce better model parameters. This is an area of exploration for advanced users

`[FAN_BREAKPOINTS=<value>]` : Sets the number off fan setpoint to test during calibration. Three fan powers (0%, 50%, 100%) are tested by default. An arbitrary number breakpoints can be specified e.g. 7 breakpoints would result in (0, 16%, 33%, 50%, 66%, 83%, 100%) fan speeds. Each breakpoint adds about 20s to the calibration.

> [!NOTE]
> Ensure that the part cooling fan is off before starting calibration.

Default calibration of the hotend:

```
MPC_CALIBRATE HEATER=extruder  
```

Default calibration of the bed: 

```
MPC_CALIBRATE HEATER=heater_bed TARGET=100  
```

After calibration the routine will generate the key model parameters which will be available for use in that printer session and are available in the log for future reference.  

![Calibration Parameter Output](/docs/img/MPC_calibration_output.png)

A **SAVE_CONFIG** command is then required to commit these calibrated parameters to the printer config. The save config block should then look similar to: 

```
#*# [extruder]
#*# control = mpc
#*# block_heat_capacity = 22.3110
#*# sensor_responsiveness = 0.0998635
#*# ambient_transfer = 0.155082
#*# fan_ambient_transfer=0.155082, 0.20156, 0.216441
#*# 
#*# [heater_bed]
#*# control = mpc
#*# block_heat_capacity = 2078.86
#*# sensor_responsiveness = 0.0139945
#*# ambient_transfer = 15.6868
```

> [!NOTE]
> If the [extruder] and [heater_bed] sections are located in a cfg file other than printer.cfg the SAVE_CONFIG command may not be able to write the calibration parameters and klippy will provide an error. 

The calibrated parameters are not suitable for pre-configuration or are not explicitly determinable. Advanced users could tweak these post calibration based on the following guidance: Slightly increasing these values will increase the temperature where MPC settles and slightly decreasing them will decrease the settling temperature.  

```
#block_heat_capacity:
#   Heat capacity of the heater block in (J/K).
#ambient_transfer:
#   Heat transfer from heater block to ambient in (W/K).
#sensor_responsiveness:
#   A single constant representing the coefficient of heat transfer from
#   heater block to sensor and heat capacity of the sensor in (K/s/K).
#fan_ambient_transfer:
#   Heat transfer from heater block to ambient in with fan enabled in (W/K).
```

## Filament Feed Forward Physical Properties

MPC works best knowing how much energy (in Joules) it takes to heat 1mm of filament by 1°C. The material values from the tables below have been curated from popular filament manufacturers and material data references. These values are sufficient for MPC to implement the filament feed forward feature.  Advanced users could tune the filament_density and filament_heat_capacity parameters based on manufacturers datasheets. 

### Common Materials

| Material | Density [g/cm³] | Specific heat [J/g/K] |
| -------- | --------------- | --------------------- |
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

| Material  | Density [g/cm³] | Specific heat [J/g/K] |
| --------- | --------------- | --------------------- |
| ABS-CF    | 1.11            | **                    |
| ASA-CF    | 1.11            | **                    |
| PA6-CF    | 1.19            | **                    |
| PC+ABS-CF | 1.22            | **                    |
| PC+CF     | 1.36            | **                    |
| PLA-CF    | 1.29            | **                    |
| PETG-CF   | 1.30            | **                    |

**Use the specific heat from the base polymer

# Real-Time Model State

The real-time temperatures and model states can be viewed from a browser by entering the following local address for your computer.

```
https://192.168.xxx.xxx:7125/printer/objects/query?extruder
```

![Calibration](/docs/img/MPC_realtime_output.png)

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

When in steady state, the difference between real and simulated sensor temperatures is used to drive the changes to ambient temperature. However when the temperatures are really close min_ambient_change ensures that the simulated ambient temperature converges relatively quickly. Larger values of min_ambient_change will result in faster convergence but will also cause the simulated ambient temperature to flutter somewhat chaotically around the ideal value. This is not a problem because the effect of ambient temperature is fairly small and short-term variations of even 10°C or more will not have a noticeable effect.  

It is important to note that the simulated ambient temperature will only converge on real world ambient temperature if the ambient heat transfer coefficients are exactly accurate. In practice this will not be the case and the simulated ambient temperature therefore also acts a correction to these inaccuracies.  

Finally, armed with a new set of temperatures, the MPC algorithm calculates how much power must be applied to get the heater block to target temperature in the next two seconds. This calculation takes into account the heat that is expected to be lost to ambient air and filament heating. This power value is then converted to a PWM output.  

## Additional Details

Please refer to that the excellent Marlin MPC Documentation for information on the model derivations, tuning methods, and heat transfer coefficients used in this feature.   

# Acknowledgements

This feature is a port of the Marlin MPC implementation, and all credit goes to their team and community for pioneering this feature for open source 3D printing. The Marlin MPC documentation and github pages were heavily referenced and, in some cases directly copied and edited to create this document.  

- Marlin MPC Documentation: [https://marlinfw.org/docs/features/model_predictive_control.html]
- GITHUB PR that implemented MPC in Marlin: [https://github.com/MarlinFirmware/Marlin/pull/23751]
- Marlin Source Code: [https://github.com/MarlinFirmware/Marlin]
