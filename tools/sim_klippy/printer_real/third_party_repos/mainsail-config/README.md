# Mainsail klipper macros and settings

This is the new location of the macros and settings provided by the Mainsail team. The intent is to enable all users to setup important macros and settings needed by the mainsail UI.

### Important features

- Park position depending on printer kinematics or user preference
  - Round beds 0:Ymax
  - Square beds Xmax:Ymax
  - User Position
  - User can specify an differnt position for PAUSE and CANCEL_PRINT
- PAUSE now supports option input parameters [X,Y,Z_MIN]
  - That is helpful to direct the use of the PAUSE macro in your M600 (see the mainsail.cfg for an example)
- Customization via a single macro that contains all allowed variables
- Additional custom variables for stuff like extra retract at CANCEL_PRINT.
- "Pause at next Layer" and "Pause at Layer #"
- Different idle_timeout value when entering PAUSE
- Supports printer.cfg without any extruder e.g. cnc
- Custom macros inside PAUSE/RESUME/CANCEL_PRINT
- RESUME Safeguard features
  - Possibility to add a filament runout sensor
  - Usage of [extruder].min_extrude_temp 

### Why have we decided to use a dedicated repo?

There are 2 main reasons for that decision

1) The current moonraker changes have shown that we need to react very fast on changes.
2) We regularly improve the macros and that is the simplest way that the installed base can benefit from it.

### How to install

We currently do not provide an installer and we might never do. But doing it is simple copy and paste a few commands in an ssh terminal.
The instructions assume that you have an up-to-date and working moonraker installation. If not start with updating your moonraker first.

```sh
cd ~
git clone https://github.com/mainsail-crew/mainsail-config.git
ln -sf ~/mainsail-config/mainsail.cfg ~/printer_data/config/mainsail.cfg
```

### Add updater section

You need to update your moonraker.conf to alway get the latest version.

Either open your moonraker.conf and add

```ini
[update_manager mainsail-config]
type: git_repo
primary_branch: master
path: ~/mainsail-config
origin: https://github.com/mainsail-crew/mainsail-config.git
managed_services: klipper
```

below the mainsail updater section.

You can also link and include the prepared file to your moonraker.conf. ssh in your PI and

```sh
ln -sf ~/mainsail-config/mainsail-moonraker-update.conf ~/printer_data/config/mainsail-moonraker-update.conf
```

then open your moonraker.conf and add

```ini
[include mainsail-moonraker-update.conf]
```

below the mainsail updater section.

### How to setup

The setup process is still the same, simply add

```ini
[include mainsail.cfg]
```

to your printer.cfg file. Be aware the file will show up as read only. This was intended by use.

### How to customize your settings

##### Different virtual sd card location or a different on_error_gcode

for that simply copy the [virtual_sdcard] block from the mainsail.cfg and simple place it below your include. The result should look similar to:

```ini
[include mainsail.cfg]

[virtual_sdcard]
path: "my new location"
on_error_gcode: CANCEL_PRINT
```

##### Customize macros

We provided a variable setup that let you customize the provided PAUSE, RESUME and CANCEL_PRINT macros. If you need e.g., your own park position simple copy the complete _CLIENT_VARIABLE macro from the mainsail.cfg and place it below your mainsail include.
After that uncomment the needed variable and fill them with your values.

The result for a custom park position would look like:

```ini
[include mainsail.cfg]

[gcode_macro _CLIENT_VARIABLE]
variable_use_custom_pos  : True  ; use custom park coordinates for x,y [True/False] 
variable_custom_park_x   : 150.0 ; custom x position; value must be within your defined min and max of X
variable_custom_park_y   : 10.0  ; custom y position; value must be within your defined min and max of Y
#variable_custom_park_dz   : 2.0   ; custom dz value; the value in mm to lift the nozzle when move to park position
#variable_retract          : 1.0   ; the value to retract while PAUSE
#variable_cancel_retract   : 5.0   ; the value to retract while CANCEL_PRINT
#variable_speed_retract    : 35.0  ; retract speed in mm/s
#variable_unretract        : 1.0   ; the value to unretract while RESUME
#variable_speed_unretract  : 35.0  ; unretract speed in mm/s
#variable_speed_hop        : 15.0  ; z move speed in mm/s
#variable_speed_move       : 100.0 ; move speed in mm/s
#variable_park_at_cancel   : False ; allow to move the toolhead to park while execute CANCEL_PRINT [True/False]
#variable_park_at_cancel_x : None  ; different park position during CANCEL_PRINT [None/Position as Float]; park_at_cancel must be True
#variable_park_at_cancel_y : None  ; different park position during CANCEL_PRINT [None/Position as Float]; park_at_cancel must be True
## !!! Caution [firmware_retraction] must be defined in the printer.cfg if you set use_fw_retract: True !!!
#variable_use_fw_retract   : False ; use fw_retraction instead of the manual version [True/False]
#variable_idle_timeout     : 0     ; time in sec until idle_timeout kicks in. Value 0 means that no value will be set or restored
#variable_runout_sensor    : ""    ; If a sensor is defined, it will be used to cancel the execution of RESUME in case no filament is detected.
##                                   Specify the config name of the runout sensor e.g "filament_switch_sensor runout". Hint use the same as in your printer.cfg
## !!! Custom macros, please use with care and review the section of the corresponding macro.
## These macros are for simple operations like setting a status LED. Please make sure your macro does not interfere with the basic macro functions.
## Only  single line commands are supported, please create a macro if you need more than one command.
#variable_user_pause_macro : ""    ; Everything insight the "" will be executed after the klipper base pause (PAUSE_BASE) function
#variable_user_resume_macro: ""    ; Everything insight the "" will be executed before the klipper base resume (RESUME_BASE) function
#variable_user_cancel_macro: ""    ; Everything insight the "" will be executed before the klipper base cancel (CANCEL_PRINT_BASE) function
#gcode:
```

You can also uncomment all variables if you like. The values are the same as the user default.

##### Different park position

The following descrips how to setup a custom position for CANCEL_PRINT and PAUSE. As an example we asume the bed has asize of (300x300mm) and we want:

- Position for CANCEL_PRINT: back right (295x295 mm)
- Position for PAUSE       : front left (10x10 mm)

First copy the complete _CLIENT_VARIABLE macro from the mainsail.cfg and place it below your mainsail include. After that uncomment the needed variables or all. The values are the same as the default.
After that we need to enter the needed values. See the result below:

```ini
[include mainsail.cfg]

[gcode_macro _CLIENT_VARIABLE]
variable_use_custom_pos   : True  ; use custom park coordinates for x,y [True/False]
variable_custom_park_x    : 10.0  ; custom x position; value must be within your defined min and max of X
variable_custom_park_y    : 10.0  ; custom y position; value must be within your defined min and max of Y
variable_custom_park_dz   : 2.0   ; custom dz value; the value in mm to lift the nozzle when move to park position
variable_retract          : 1.0   ; the value to retract while PAUSE
variable_cancel_retract   : 5.0   ; the value to retract while CANCEL_PRINT
variable_speed_retract    : 35.0  ; retract speed in mm/s
variable_unretract        : 1.0   ; the value to unretract while RESUME
variable_speed_unretract  : 35.0  ; unretract speed in mm/s
variable_speed_hop        : 15.0  ; z move speed in mm/s
variable_speed_move       : 100.0 ; move speed in mm/s
variable_park_at_cancel   : True  ; allow to move the toolhead to park while execute CANCEL_PRINT [True/False]
variable_park_at_cancel_x : 295.0 ; different park position during CANCEL_PRINT [None/Position as Float]; park_at_cancel must be True
variable_park_at_cancel_y : 295.0 ; different park position during CANCEL_PRINT [None/Position as Float]; park_at_cancel must be True
# !!! Caution [firmware_retraction] must be defined in the printer.cfg if you set use_fw_retract: True !!!
variable_use_fw_retract   : False ; use fw_retraction instead of the manual version [True/False]
variable_idle_timeout     : 0     ; time in sec until idle_timeout kicks in. Value 0 means that no value will be set or restored
variable_runout_sensor    : ""    ; If a sensor is defined, it will be used to cancel the execution of RESUME in case no filament is detected.
#                                   Specify the config name of the runout sensor e.g "filament_switch_sensor runout". Hint use the same as in your printer.cfg
## !!! Custom macros, please use with care and review the section of the corresponding macro.
## These macros are for simple operations like setting a status LED. Please make sure your macro does not interfere with the basic macro functions.
## Only  single line commands are supported, please create a macro if you need more than one command.
variable_user_pause_macro : ""    ; Everything insight the "" will be executed after the klipper base pause (PAUSE_BASE) function
variable_user_resume_macro: ""    ; Everything insight the "" will be executed before the klipper base resume (RESUME_BASE) function
variable_user_cancel_macro: ""    ; Everything insight the "" will be executed before the klipper base cancel (CANCEL_PRINT_BASE) function
gcode:
```

This way insures that older configs still work as before.


### New Feature: "Pause at next Layer" and "Pause at Layer #"

This is based on a idea of Pedro Lamas. It let you add a Pause at the next layer change or if you reach a specific layer number.

First you need to prepare your slicer as described in https://github.com/Klipper3d/klipper/pull/5726

If you done that you can either use

```txt
SET_PAUSE_NEXT_LAYER [ENABLE=1] [MACRO=<name>]
```

to get execute the given GCODE macro at the next layer change. The MACRO is normally either PAUSE (default) or M600 (if you have specified it in your printer.cfg).

Or use

```txt
SET_PAUSE_AT_LAYER [ENABLE=1] [LAYER=<number>] [MACRO=<name>]
```

to get execute the given GCODE macro at the given LAYER number change. The MACRO is normally either PAUSE (default) or M600 (if you have specified it in your printer.cfg).

To remove the "Pause at next Layer" simply send

```txt
SET_PAUSE_NEXT_LAYER ENABLE=0
```

To remove the "Pause at Layer" simply send

```txt
SET_PAUSE_AT_LAYER ENABLE=0
```

Both will clear after execution.


### New Feature: Save/Restore extruder temperature on pause/resume
With commit https://github.com/mainsail-crew/mainsail-config/commit/ea19d723bcfbe3e57cff8a9abae36dc5e6a2d1a9 it is posible to switch off the extruder after entering PAUSE. RESUME will heatup the extruder if needed. That is helpful if you e.g. do a pause because of an runout and are not there. Be aware that doing that might have a negative effect on your print quality. Be aware the bed must be heated all the time!
The following example shows you how to modify your [idle_timeout] to switch of the extruder in case the idle timeout kicks in.

The old [idle_timeout] example is removed please use the newer one below.

**Update:** Some users reporting that this feature does interfere with their workflow. Therefor we improved it by
- The temperature is only restored if you come out of idle_timeout
- PAUSE gets a new paramter RESTORE [0/1]
  - 1: Thats the default and enables the restore of temperature when comming out of idle_timeout.
  - 0: The temperature will not restored.
  
That is helpfull if you use it in e.g a M600 macro where you want to insure that the temperature is never restored.

```ini
[gcode_macro M600]
description: Filament change
gcode: PAUSE X=10 Y=10 Z_MIN=50 RESTORE=0
```

**2nd Update:** After I get reports from user's that this does not work, during further debug I found out that I missed an important fact in my thoughts. Any interaction with the printer, e.g. change temperature or use a filament load/unload macro will disable the IDLE state.
The only solution to solve it is an independent controlled variable.
For that idle_state is added and that needs to be set in your [idle_timeout]
```ini
SET_GCODE_VARIABLE MACRO=RESUME VARIABLE=idle_state VALUE=True
```
See the example below:
```ini
[idle_timeout]
gcode:
  {% if printer.pause_resume.is_paused %}
    SET_GCODE_VARIABLE MACRO=RESUME VARIABLE=idle_state VALUE=True
    {action_respond_info("Idle Timeout: Extruder powered down")}
    M104 S0   ; Set Hot-end to 0C (off)
  {% else %}
    {action_respond_info("Idle Timeout: Stepper and Heater powered down")}
    TURN_OFF_HEATERS
    M84
  {% endif %}
```

### New Feature: Change idle_timeout when going in PAUSE
Many users falling in the trap of idle_timeout. This module is always activated an will shut down heaters and motors after 10 minutes if the printer is idle.
Users may detect this behaivior first time after the install a runout sensor and have a runout when they are away of the printer.

**To make it clear that is a safety feature and you should set your idle_timeout value with that in mind.**

We therefor enhanced the PAUSE/RESUME/CANCEL_PRINT macros with the posibility to set and restore a different idle_timeout value as defined in your printer.cfg

The following shows the example to set it to 1h when enetering PAUSE
```ini
[gcode_macro _CLIENT_VARIABLE]
#variable_use_custom_pos   : False ; use custom park coordinates for x,y [True/False]
#variable_custom_park_x    : 0.0   ; custom x position; value must be within your defined min and max of X
#variable_custom_park_y    : 0.0   ; custom y position; value must be within your defined min and max of Y
#variable_custom_park_dz   : 2.0   ; custom dz value; the value in mm to lift the nozzle when move to park position
#variable_retract          : 1.0   ; the value to retract while PAUSE
#variable_cancel_retract   : 5.0   ; the value to retract while CANCEL_PRINT
#variable_speed_retract    : 35.0  ; retract speed in mm/s
#variable_unretract        : 1.0   ; the value to unretract while RESUME
#variable_speed_unretract  : 35.0  ; unretract speed in mm/s
#variable_speed_hop        : 15.0  ; z move speed in mm/s
#variable_speed_move       : 100.0 ; move speed in mm/s
#variable_park_at_cancel   : False ; allow to move the toolhead to park while execute CANCEL_PRINT [True/False]
#variable_park_at_cancel_x : None  ; different park position during CANCEL_PRINT [None/Position as Float]; park_at_cancel must be True
#variable_park_at_cancel_y : None  ; different park position during CANCEL_PRINT [None/Position as Float]; park_at_cancel must be True
## !!! Caution [firmware_retraction] must be defined in the printer.cfg if you set use_fw_retract: True !!!
#variable_use_fw_retract   : False ; use fw_retraction instead of the manual version [True/False]
variable_idle_timeout     : 3600   ; time in sec until idle_timeout kicks in. Value 0 means that no value will be set or restored
#variable_runout_sensor    : ""    ; If a sensor is defined, it will be used to cancel the execution of RESUME in case no filament is detected.
##                                   Specify the config name of the runout sensor e.g "filament_switch_sensor runout". Hint use the same as in your printer.cfg
## !!! Custom macros, please use with care and review the section of the corresponding macro.
## These macros are for simple operations like setting a status LED. Please make sure your macro does not interfere with the basic macro functions.
## Only  single line commands are supported, please create a macro if you need more than one command.
#variable_user_pause_macro : ""    ; Everything insight the "" will be executed after the klipper base pause (PAUSE_BASE) function
#variable_user_resume_macro: ""    ; Everything insight the "" will be executed before the klipper base resume (RESUME_BASE) function
#variable_user_cancel_macro: ""    ; Everything insight the "" will be executed before the klipper base cancel (CANCEL_PRINT_BASE) function
gcode:
```

### New Feature: Custom macros inside of PAUSE/RESUME/CANCEL_PRINT
We are still not sure if that is a good idea, but I understand that many user needing a way to do simple stuff like adding the change of a status led to their PAUSE/RESUME/CANCEL_PRINT macros.
So please use it with care and test if your macro interferes with the general klipper PAUSE/RESUME/CANCEL_PRINT flows.

As an example lets assume you have a macro that allows you to change a LED based an your status and you have the following GCODE calls:
```ini
SET_MY_STATUS_LED STATUS=printing
SET_MY_STATUS_LED STATUS=pause
SET_MY_STATUS_LED STATUS=cancel
```
To add that the following needs to be added:
```ini
[gcode_macro _CLIENT_VARIABLE]
variable_use_custom_pos   : False ; use custom park coordinates for x,y [True/False]
variable_custom_park_x    : 0.0   ; custom x position; value must be within your defined min and max of X
variable_custom_park_y    : 0.0   ; custom y position; value must be within your defined min and max of Y
variable_custom_park_dz   : 2.0   ; custom dz value; the value in mm to lift the nozzle when move to park position
variable_retract          : 1.0   ; the value to retract while PAUSE
variable_cancel_retract   : 5.0   ; the value to retract while CANCEL_PRINT
variable_speed_retract    : 35.0  ; retract speed in mm/s
variable_unretract        : 1.0   ; the value to unretract while RESUME
variable_speed_unretract  : 35.0  ; unretract speed in mm/s
variable_speed_hop        : 15.0  ; z move speed in mm/s
variable_speed_move       : 100.0 ; move speed in mm/s
variable_park_at_cancel   : False ; allow to move the toolhead to park while execute CANCEL_PRINT [True/False]
variable_park_at_cancel_x : None  ; different park position during CANCEL_PRINT [None/Position as Float]; park_at_cancel must be True
variable_park_at_cancel_y : None  ; different park position during CANCEL_PRINT [None/Position as Float]; park_at_cancel must be True
# !!! Caution [firmware_retraction] must be defined in the printer.cfg if you set use_fw_retract: True !!!
variable_use_fw_retract   : False ; use fw_retraction instead of the manual version [True/False]
variable_idle_timeout     : 0     ; time in sec until idle_timeout kicks in. Value 0 means that no value will be set or restored
variable_runout_sensor    : ""    ; If a sensor is defined, it will be used to cancel the execution of RESUME in case no filament is detected.
#                                   Specify the config name of the runout sensor e.g "filament_switch_sensor runout". Hint use the same as in your printer.cfg
## !!! Custom macros, please use with care and review the section of the corresponding macro.
## These macros are for simple operations like setting a status LED. Please make sure your macro does not interfere with the basic macro functions.
## Only  single line commands are supported, please create a macro if you need more than one command.
variable_user_pause_macro : "SET_MY_STATUS_LED STATUS=pause"    ; Everything insight the "" will be executed after the klipper base pause (PAUSE_BASE) function
variable_user_resume_macro: "SET_MY_STATUS_LED STATUS=printing" ; Everything insight the "" will be executed before the klipper base resume (RESUME_BASE) function
variable_user_cancel_macro: "SET_MY_STATUS_LED STATUS=cancel"   ; Everything insight the "" will be executed before the klipper base cancel (CANCEL_PRINT_BASE) function
gcode:
```
That will then execute these 3 GCODE calls at appropriate positions in the 3 macros.

### New Feature: Add a filament runout sensor to abord RESUME
Many user using filament runout sensors nowadays, so it makes perfect sense to include them in the decision if it is safe to resume a print.

Let’s assume you have the following sensor defined in your printer.cfg:
```ini
[filament_switch_sensor filament_sensor]
...
```
To use that sensor following must be added:
```ini
[gcode_macro _CLIENT_VARIABLE]
variable_runout_sensor    : "filament_switch_sensor filament_sensor"    ; If a sensor is defined, it will be used to cancel the execution of RESUME in case no filament is detected.
gcode:
```
The detection state will be ignored if you temporarily disable the sensor.