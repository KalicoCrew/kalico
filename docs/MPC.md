# Model Predictive Control

Model Predictive Control (MPC) control is an alternaive temperature control method to PID. MPC uses a system model to simulate hotend temperature and adjust the heater power to match the target.

MPC operates as a forward-looking model, making compensations prior to actual temperature fluctuations. It uses a model of the hotend accounting for (1) Temperatures (heat block, sensor, ambient), (2)Heater Power, (3)Heat Transfer from the block to ambient from the fan and filament, (4) Block heat capacity, (5) Sensor responspivness . This model enables MPC to determine the quantity of heat energy that will be dissipated from the hotend over a specified duration and accounts for it by adjusting the heater power. MPC can accurately calculate the necessary heat energy input (also in Joules, or "Watts times Seconds") required to maintain a consistent temperature or to adjust to a new temperature. 

MPC has many advantages over PID control:
-Faster and more stable response to temperature
-Single calibration works over a wide range of print temperatures
-Easier to calibrate
-Works with all sensor types and noisy temperature sensors.
-Works equally well with standard and PTC heaters



The tuning algorithm does the following with the target hotend:

Move to the center and close to bed: Printing occurs close to the bed or printed model so tuning is done close to a surface to best emulate the conditions while printing.
Cool to ambient: The tuning algorithm needs to know the approximate ambient temperature. It switches the part cooling fan on and waits until the temperature stops decreasing.
Heat past 200°C: Measure the point where the temperature is increasing most rapidly, and the time and temperature at that point. Also, three temperature measurements are needed at some point after the initial latency has taken effect. The tuning algorithm heats the hotend to over 200°C.
Hold temperature while measuring ambient heat-loss: At this point enough is known for the MPC algorithm to engage. The tuning algorithm makes a best guess at the overshoot past 200°C which will occur and targets this temperature for about a minute while ambient heat-loss is measured without (and optionally with) the fan.
Set MPC up to use the measured constants and report them for use in Configuration.h.
NOTE: If the algorithm fails or is interrupted with M108, some or all of the MPC constants may be changed anyway and their values may not be reliable.


**Caution** This feature controls the portions of the 3D printer that can get very hot. All standard DangerKlipper warnings apply while using this. Please report all issues and bugs.


Configuration

To use, on heater set the following appropriate heater section of the config. Currently only [extruder] and [heater_bed] are supported. Note that this only works on extruders that can reach 200C.

control: mpc
part_cooling_fan: fan // this is the fan that is cooling extruded filament and the hotend
ambient_temp_sensor: [temperature_sensor {name} ]
    ex: temperature_sensor beacon_coil 
    # Optional. This parameter can use any sensor but it should be a temperature sensor in proximity to the hotend or measuring the ambient of the hotend such as a chamber sensor. If this is not given MPC will give an estimate. This is used for initial state temperature and calibration but not for actual control.

Filament parameters that can be set to improve the accuracy of the model. In general MPC is capable of controlling the hotend without accounting for the heat required to melt filament. The accuracy and responsiveness of MPC can be improved by accounting for the filament. Refer to the table below. 
filament_diameter  default=1.75
filament_density   default=0.0
filament_heat_capacity = default=0.0

The following are optional parameters that can be tuned but should not need
target_reach_time // default=2.0
smoothing // default=0.25
min_ambient_change  default=1.0 //Larger values of MIN_AMBIENT_CHANGE will result in faster convergence but will also cause the simulated ambient temperature to flutter somewhat chaotically around the ideal value.
steady_state_rate   default=0.5 //  (Q- this is 1 deg/s in marlin)

Calibrated parameters and not suitable for pre-configuration or not explicetly determinable. Advanced users could tweak based on the following:slightly increasing these values will increase the temperature where MPC settles and slightly decreasing them will decrease the settling temperature.
block_heat_capacity
ambient_transfer 
sensor_responsiveness 
fan_ambient_transfer


Initial Calibration
The MPC calibration routine takes the following steps:
-Move to the center and close to bed so that tuning is done close to a surface to best emulate the conditions while printing.
-Cool to ambient: The calibration routine needs to know the approximate ambient temperature. It switches the part cooling fan on and waits until the hotend temperature stops decreasing relative to ambient.
-Heat past 200°C: Measure the point where the temperature is increasing most rapidly, and the time and temperature at that point. Also, three temperature measurements are needed at some point after the initial latency has taken effect. The tuning algorithm heats the hotend to over 200°C.
-Hold temperature while measuring ambient heat-loss: At this point enough is known for the MPC algorithm to engage. The calibration routine makes a best guess at the overshoot past 200°C which will occur and targets this temperature for about a minute while ambient heat-loss is measured without (and optionally with) the fan.  (*Q* does klipper MPC use the fan??)
-MPC calibration routine creates the appropriate model constants and saves them for use. At this time the model parameters are temporate and not yet saved to the printer configuration. SAVE_CONFIG.
//note that MPC calibration default to Asymptotic Tuning method intitially and if that fails it will use Differential Tuning.


The MPC calibration routine has to be run intially 
MPC_CALIBRATE HEATER={heater}

For example initial calibration of the hotend would be.
MPC_CALIBRATE HEATER=extruder

To calibrate a bed the following additional parameter is required:
MPC_CALIBRATE HEATER={heater} TARGET={TEMPERATURE} 
MPC_CALIBRATE HEATER=bed_heater TARGET=100


After calibration the routine will generate the key model parameter. A SAVE_CONFIG command is then required to commit these calibrated parameters to the config.


MPC has the ability to use the material properties of the filament in the model and these can be set in the config or changed ad-hoc via the command line. The parameters from the table below should be more than sufficent to allow MPC to accomodate for heat transfer into the filament. 

MPC_SET HEATER={heater} FILAMENT_DENSITY={g/mm^2} FILAMENT_HEAT_CAPACITY={J/g/K}

MPC_SET HEATER=extruder FILAMENT_DENSITY=1.09 FILAMENT_HEAT_CAPACITY=1.3



Basic Table of Density and Specific Heat Capacities for Various Filament Types. MPC likes to know how much energy (in Joules) it takes to heat 1mm of filament by 1°C (or 1 Kelvin, which is the same thing). This can be calculated from the specific heat capacity and the density of the material.
-Note that specific heat is not a typical value provided by any filament manufactures so we rely on typical polymer raw material values.
-Note that filled filaments or polymer alloys will have differnt values for density and specific heat. Again, close enough is good enough.
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


BACKGROUND:

MPC Algorithm
MPC models the hotend system as four thermal masses: ambient air, the filament, the heater block and the sensor. Heater power heats the modeled heater block directly. Ambient air heats or cools the heater block. Filament cools the heater block. The heater block heats or cools the sensor.

Every time the MPC algorithm runs it uses the following information to calculate a new temperature for the simulated hotend and sensor:

-The last power setting for the hotend.
-The present best-guess of the ambient temperature.
-The effect of the fan on heat-loss to the ambient air.
-The effect of filament feedrate on heat-loss to the filament. Filament is assumed to be at the same temperature as the ambient air.

Once this calculation is done, the simulated sensor temperature is compared to the measured temperature and a fraction of the difference is added to the modeled sensor and heater block temperatures. This drags the simulated system in the direction of the real system. Because only a fraction of the difference is applied, sensor noise is diminished and averages out to zero over time. Both the simulated and the real sensor exhibit the same (or very similar) latency. Consequently the effects of latency are eliminated when these values are compared to each other. So the simulated hotend is only minimally affected by sensor noise and latency. //REMOVE? ->This is where the real magic of this MPC implementation lies.//

SMOOTHING is the factor applied to the difference between simulated and measured sensor temperature. At its maximum value of 1, the simulated sensor temperature is continually set equal to the measured sensor temperature. A lower value will result in greater stability in MPC output power but also in decreased responsiveness. A value around 0.25 seems to work quite well.

No simulation is perfect and, anyway, real life ambient temperature changes. So MPC also maintains a best guess estimate of ambient temperature. When the simulated system is close to steady state the simulated ambient temperature is continually adjusted. Steady state is determined to be when the MPC algorithm is not driving the hotend at its limits (i.e., full or zero heater power) or when it is at its limit but temperatures are still not changing very much - which will occur at asymptotic temperature (usually when target temperature is zero and the hotend is at ambient).

steady_state_rate is used to recognize the asymptotic condition. Whenever the simulated hotend temperature changes at an absolute rate less than steady_state_rate between two successive runs of the algorithm, the steady state logic is applied. Since the algorithm runs frequently, even a small amount of noise can result in a fairly high instantaneous rate of change of hotend temperature. In practice 1°C/s seems to work well for steady_state_rate.

When in steady state, the difference between real and simulated sensor temperatures is used to drive the changes to ambient temperature. However when the temperatures are really close min_ambient_change ensures that the simulated ambient temperature converges relatively quickly. Larger values of min_ambient_change will result in faster convergence but will also cause the simulated ambient temperature to flutter somewhat chaotically around the ideal value. This is not a problem because the effect of ambient temperature is fairly small and short term variations of even 10°C or more will not have a noticeable effect.

It is important to note that the simulated ambient temperature will only converge on real world ambient temperature if the ambient heat transfer coefficients are exactly accurate. In practice this will not be the case and the simulated ambient temperature therefore also acts a correction to these inaccuracies.

Finally, armed with a new set of temperatures, the MPC algorithm calculates how much power must be applied to get the heater block to target temperature in the next two seconds. This calculation takes into account the heat that is expected to be lost to ambient air and filament heating. This power value is then converted to a PWM output.

Please refer to that the excellent Marlin MPC Documentation for additional details on the model derivations, model tuning methods, and heat transfer coefficents. 


Acknowledgements

This feature is a port of the Marlin MPC implementation and all credit goes to their team and community for pioneering this feature for open source 3D printing. The Marlin MPC documentation and github pages were heavily referenced and in some cases directly copied and edited to create this document.
Marlin MPC Documentation: https://marlinfw.org/docs/features/model_predictive_control.html
GITHUB PR that implemented MPC in Marlin: https://github.com/MarlinFirmware/Marlin/pull/23751
Marlin Source Code: https://github.com/MarlinFirmware/Marlin


