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







Refer to the [control statement](Config_Reference.md#extruder) in the
Configuration Reference.


Configuration


To use, on heater set the following in the extruder section:

control: mpc
part_cooling_fan: fan
ambient_temp_sensor: temperature_sensor beacon_coil # can use any sensor, or leave option out and it will estimate

Run the command:
Start printer, run:
MPC_CALIBRATE HEATER=extruder

Let it run, SAVE_CONFIG at the end. Done.

To use filament feed forward, use MPC_SET e.g.:
MPC_SET HEATER=extruder FILAMENT_DENSITY=1.09 FILAMENT_HEAT_CAPACITY=1.3

add table of filament density and heat capacity:


This feature is a port of the Marlin implementation and all credit 
