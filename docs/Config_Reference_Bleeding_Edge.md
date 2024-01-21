# Configuration reference for Bleeding Edge Features

This document is a reference for options available in the Klipper
config file.

The descriptions in this document are formatted so that it is possible
to cut-and-paste them into a printer config file. See the
[installation document](Installation.md) for information on setting up
Klipper and choosing an initial config file.

## High precision stepping and stepcompress protocol

The configuration for this feature is done during klipper firmware compile 
by selecting "High Precision Stepping Support" option during the "make menuconfig" 
command. There are no configuration parameters for this feature.

ADD PHOTO OF MENU

## Syncronisation of extruder motion with input shaper

[input_shaper] 
#enabled_extruders: extruder


## Smooth Input Shapers
[input_shaper]
#shaper_type: mzv
#shaper_type_x: smooth_mzv
#smoother_freq_x: 67.0
#shaper_type_y: smooth_mzv


## Ringing Tower Print Utility
[ringing_tower]
# Interesting parameters that may require adjustment
size: 100
height: 60
band: 5
perimeters: 2
velocity: 80 # is the velocity one must use as V in a formula V * N / D when calculating the resonance frequency. N and D are the number of oscillations and the distance between them as usual:
brim_velocity: 30
accel_start: 1500  # the acceleration of the start of the test
accel_step: 500  # the increment of the acceleration every `band` mm
layer_height: 0.2
first_layer_height: 0.2
filament_diameter: 1.75
# Parameters that are computed automatically, but may be adjusted if necessary
center_x: ...  # Center of the bed by default (if detected correctly)
center_y: ...  # Center of the bed by default (if detected correctly)
brim_width: ... # computed based on the model size, but may be increased
# Parameters that are better left at their default values
# notch: 1  # size of the notch in mm
# notch_offset: ... # 0.275 * size by default
# deceleration_points: 100

## Pressure Advance Tower Print Utility
[pa_test]
# size_x: 100
# size_y: 50
# height: 50
# origin_x: #bed_center_x
# origin_y: #bed_center_y
# layer_height: 0.2
# first_layer_height: 0.3
# perimeters: 2
# brim_width: 10
# slow_velocity: 20
# fast_velocity: 80
# filament_diameter: 1.75


