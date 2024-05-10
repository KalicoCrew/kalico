# Model Predictive Control

Model Predictive Control (MPC) control is an alternaive temperature control method to PID. MPC uses a system model to simulate hotend temperature and adjust the heater power to match the target.  

MPC operates as a forward-looking model, making compensations prior to actual temperature fluctuations. It uses a model of the hotend accounting for the thermal masses of the system, heater power, heat loss to ambient air and fans, heat transfer into filament. This model enables MPC to determine the quantity of heat energy that will be dissipated from the hotend over a specified duration and accounts for it by adjusting the heater power. MPC can accurately calculate the necessary heat energy input required to maintain a consistent temperature or to adjust to a new temperature.  

MPC has many advantages over PID control:  
- Faster response and better response to temperature  
- Single calibration is function over a wide range of print temperatures  
- Easier to calibrate  
- Works with all hotend sensor types including noisy temperature sensors  
- Works equally well with standard cartridge and PTC heater types
- MPC work equally well for hotends and beds
- Equally functional for high and low flow hotends     

> [!CAUTION]
> This feature controls the portions of the 3D printer that can get very hot. All standard DangerKlipper warnings apply. Please report all issues and bugs to github or discord.

# Installation
After installing DangerKlipper you can switch to the MPC feature branch by issuing the following console commands:

```
cd ~
cd klipper
git fetch
git switch feature/mpc_experimental
```

After installation of the branch the klipper service should be restarted with:

```
systemctl restart klipper
```

# Configuration
To use MPC as the temperature controller set the following configuration parameters in the appropriate heater section of the config.   
Currently only [extruder] and [heater_bed] heater types are supported.  

```
[extruder] OR [heater_bed]
heater_power: {watts}
  # Advertised heater power in watts. 
  # Note that for a PTC, a non-linear heater, MPC is not guarenteed to work.
  # Setting this value to the heater power at the expected print temperature, for a PTC type heater
  # is a good initial value to start tuning.
cooling_fan: fan 
  # This is the fan that is cooling extruded filament and the hotend.
  # cooling_fan is currently not supported for [heater_bed].
  # "fan" will automatically find the part_cooling_fan  (Q??)
#ambient_temp_sensor: {temperature_sensor sensor_name} 
  # Optional. If this is not given MPC will estimate this parameter (reccomended).
  # It can use any sensor but it should be a temperature sensor in proximity to the hotend or
  # measuring the ambient air surrounding the hotend such as a chamber sensor. 
  # This is used for initial state temperature and calibration but not for actual control.
  # Example: temperature_sensor beacon_coil
```

## Filament Configuration
In general MPC is capable of controlling the hotend without accounting for the heat required to melt filament. MPC can look forward to changes in extrusion rates which could require more or less heat input to maintain target temperatures.  This improves the accuracy and responsiveness of the model. Filament feed forward is not enabled by default unless the density and heat capacity are specified.  

These should only be set under [extruder] and are not valid for [heater_bed]. 

```
filament_diameter: 1.75
  # default=1.75 (mm) 
filament_density: 1.1
  # An initial setting of 1.1 (g/mm^2) should work well for most filaments.
  # The Default, if not specified, is 0.0 
filament_heat_capacity: 1.3
  # A initial setting of 1.3 (J/g/K) should work well for most filaments.
  # The Default, if not specified, is 0.0
```

## Optional model parameters 
These can be tuned but should not need changing from the default values.

```
#target_reach_time:  
  # default=2.0 (sec) 
#smoothing:  
  # default=0.25 (sec)
#min_ambient_change:
  # default=1.0 (deg C)
  # Larger values of MIN_AMBIENT_CHANGE will result in faster convergence but will also cause
  # the simulated ambient temperature to flutter somewhat chaotically around the ideal value.  
#steady_state_rate:
  # default=0.5 (deg C/s) 
```

## Example configuration block

```
[extruder]
heater_power: 70  
cooling_fan: fan
filament_density: 1.1
filament_heat_capacity: 1.3

[heater_bed]
heater_power: 500  
cooling_fan: fan
```

In preperation for a **SAVE_CONFIG** command after calibration the previous extruder or heater_bed parameters should be removed or commented out. If *control: pid_v* is present in the save config block there will be a conflict error when committing the changes. A save config block ready for MPC calibartion looks like this:

```
#*# <---------------------- SAVE_CONFIG ---------------------->
#*# DO NOT EDIT THIS BLOCK OR BELOW. The contents are auto-generated.
#*#
#*# [heater_bed]
#*# control = mpc
#*#
#*# [extruder]
#*# control = mpc
```


# Calibration
The MPC default calibration routine takes the following steps:
- Move to the center and close to bed so that tuning is done close to a surface to best emulate the conditions while printing.
- Cool to ambient: The calibration routine needs to know the approximate ambient temperature. It switches the part cooling fan on and waits until the hotend temperature stops decreasing relative to ambient.
- Heat past 200°C: Measure the point where the temperature is increasing most rapidly, and the time and temperature at that point. Also, three temperature measurements are needed at some point after the initial latency has taken effect. The tuning algorithm heats the hotend to over 200°C.
- Hold temperature while measuring ambient heat-loss: At this point enough is known for the MPC algorithm to engage. The calibration routine makes a best guess at the overshoot past 200°C which will occur and targets this temperature for about a minute while ambient heat-loss is measured without (and optionally with) the fan.  (*Q* does klipper MPC use the fan??)
- MPC calibration routine creates the appropriate model constants and saves them for use. At this time the model parameters are temporate and not yet saved to the printer configuration via SAVE_CONFIG.  

## Hotend or Bed Calibration
The MPC calibration routine has to be run intially for each heater to be controlled using MPC. In order for MPC to be functional an extruder must be able to reach 200C and a bed to reach 90C.

```
MPC_CALIBRATE HEATER={heater} TARGET={temperature} FAN_BREAKPOINTS={value]
  # TARGET (deg C) sets the calibration temperature. 
  # TARGET default is 200C for extruder and 90C for beds.
  # Note that MPC calibration is temperature independent so
  # calibration the extruder at higher temperatures hasnt been
  # shown to produce better model parameters. 
  # 
  # FAN_BREAKPOINTS defaults to three fan powers (0%, 50%, 100%) for extruder calibration.
  # Arbitrary number breakpoints can be specified e.g 7 breakpoints would
  # result in (0, 16%, 33%, 50%, 66%, 83%, 100%) fan speeds. Each breakpoint adds
  # about 20s to the calibration.
```

For example default calibration of the hotend would be. 

```
MPC_CALIBRATE HEATER=extruder  
```

For example default calibration of the bed would be. 

```
MPC_CALIBRATE HEATER=heater_bed TARGET=100  
```

After calibration the routine will generate the key model parameters which will be avaliable for use in that printer session and are avaliable in the log for future refernce.  
![Calibration Parameter Output](/docs/img/MPC_calibration_output.png)

A **SAVE_CONFIG** command is then required to commit these calibrated parameters to the printer config. The save config block should then look similar to the below: 

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
#*# fan_ambient_transfer = 1.06666, 1.95662, 2.31598, 2.41591, 2.42889, 2.41607, 2.60417
```

Calibrated parameters and not suitable for pre-configuration or not explicetly determinable. Advanced users could tweak these post calibration based on the following guidance: Slightly increasing these values will increase the temperature where MPC settles and slightly decreasing them will decrease the settling temperature.  
```
block_heat_capacity: **Add description of this param.
  # Units of (J/K)
ambient_transfer: **Add description of this param.
  # Units of (W/K)
sensor_responsiveness:  **Add description of this param.
  # Units of (K/s/K) 
fan_ambient_transfer:  **Add description of this param.
  # Units of (W/K)
```

# Filament Feed Forward
Filament feed forward parameters can be set, for the printer session, via the command line or custom G-Code with the following command.

```
MPC_SET HEATER={heater} FILAMENT_DENSITY={g/mm^2} FILAMENT_HEAT_CAPACITY={J/g/K}  
```

For example, updating the filament material properties for ASA would be:   
```
MPC_SET HEATER=extruder FILAMENT_DENSITY=1.09 FILAMENT_HEAT_CAPACITY=1.3  
```

## Filament Feed Forward Physical Properties
MPC works best knowing how much energy (in Joules) it takes to heat 1mm of filament by 1°C. The values from the table below should be sufficent references to allow MPC to accomodate for specific filaments.   

- Specific heat is not a typical value provided by any filament manufactures so we rely on typical polymer raw material values.  
- Filled filaments or polymer alloys will have different values for density and specific heat.
- Close enough is good enough for MPC.

```
        Density [g/cm³]     Specifc heat [J/g/K]
PLA     1.25                1.2
PETG    1.23                1.3
ABS     1.04                2.0
ASA     1.09                1.3
PA6     1.14                1.7
PA12    1.02                1.8
PC      1.20                1.2
```

# Real-Time Model State
The realtime temperatures and model states can be viewed from a browser by entering the following local address for your computer:

```
https://192.168.xxx.xxx:7125/printer/objects/query?extruder
```  

![Calibration](/docs/img/MPC_realtime_output.png)


# BACKGROUND

## MPC Algorithm
MPC models the hotend system as four thermal masses: ambient air, the filament, the heater block and the sensor. Heater power heats the modeled heater block directly. Ambient air heats or cools the heater block. Filament cools the heater block. The heater block heats or cools the sensor.  

Every time the MPC algorithm runs it uses the following information to calculate a new temperature for the simulated hotend and sensor:  

- The last power setting for the hotend.  
- The present best-guess of the ambient temperature.  
- The effect of the fan on heat-loss to the ambient air.  
- The effect of filament feedrate on heat-loss to the filament. Filament is assumed to be at the same temperature as the ambient air.  

Once this calculation is done, the simulated sensor temperature is compared to the measured temperature and a fraction of the difference is added to the modeled sensor and heater block temperatures. This drags the simulated system in the direction of the real system. Because only a fraction of the difference is applied, sensor noise is diminished and averages out to zero over time. Both the simulated and the real sensor exhibit the same (or very similar) latency. Consequently the effects of latency are eliminated when these values are compared to each other. So the simulated hotend is only minimally affected by sensor noise and latency.   

SMOOTHING is the factor applied to the difference between simulated and measured sensor temperature. At its maximum value of 1, the simulated sensor temperature is continually set equal to the measured sensor temperature. A lower value will result in greater stability in MPC output power but also in decreased responsiveness. A value around 0.25 seems to work quite well.  

No simulation is perfect and, anyway, real life ambient temperature changes. So MPC also maintains a best guess estimate of ambient temperature. When the simulated system is close to steady state the simulated ambient temperature is continually adjusted. Steady state is determined to be when the MPC algorithm is not driving the hotend at its limits (i.e., full or zero heater power) or when it is at its limit but temperatures are still not changing very much - which will occur at asymptotic temperature (usually when target temperature is zero and the hotend is at ambient).  

steady_state_rate is used to recognize the asymptotic condition. Whenever the simulated hotend temperature changes at an absolute rate less than steady_state_rate between two successive runs of the algorithm, the steady state logic is applied. Since the algorithm runs frequently, even a small amount of noise can result in a fairly high instantaneous rate of change of hotend temperature. In practice 1°C/s seems to work well for steady_state_rate.  

When in steady state, the difference between real and simulated sensor temperatures is used to drive the changes to ambient temperature. However when the temperatures are really close min_ambient_change ensures that the simulated ambient temperature converges relatively quickly. Larger values of min_ambient_change will result in faster convergence but will also cause the simulated ambient temperature to flutter somewhat chaotically around the ideal value. This is not a problem because the effect of ambient temperature is fairly small and short term variations of even 10°C or more will not have a noticeable effect.  

It is important to note that the simulated ambient temperature will only converge on real world ambient temperature if the ambient heat transfer coefficients are exactly accurate. In practice this will not be the case and the simulated ambient temperature therefore also acts a correction to these inaccuracies.  

Finally, armed with a new set of temperatures, the MPC algorithm calculates how much power must be applied to get the heater block to target temperature in the next two seconds. This calculation takes into account the heat that is expected to be lost to ambient air and filament heating. This power value is then converted to a PWM output.  

## Additional Details
Please refer to that the excellent Marlin MPC Documentation for information on the model derivations, tuning methods, and heat transfer coefficents used in this feature.   

# Acknowledgements

This feature is a port of the Marlin MPC implementation and all credit goes to their team and community for pioneering this feature for open source 3D printing. The Marlin MPC documentation and github pages were heavily referenced and in some cases directly copied and edited to create this document.  
- Marlin MPC Documentation: [https://marlinfw.org/docs/features/model_predictive_control.html]
- GITHUB PR that implemented MPC in Marlin: [https://github.com/MarlinFirmware/Marlin/pull/23751]
- Marlin Source Code: [https://github.com/MarlinFirmware/Marlin]


