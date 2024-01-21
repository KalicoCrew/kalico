Bleeding Edge Features Documentation:
https://github.com/dmbutyugin/klipper/commits/advanced-features

## High precision stepping and new stepcompress protocol
https://klipper.discourse.group/t/improved-stepcompress-implementation/3203

## Smooth Input Shapers
https://klipper.discourse.group/t/improved-stepcompress-implementation/3203

### Overview
The "Smooth Input Shaper" is an advanced feature in Klipper's advanced-features branch, designed to enhance 3D printing quality by employing polynomial smooth functions for input shaping. This feature aims to provide a smoother acceleration profile, similar to an S-curve, though with distinct characteristics due to its fixed timing.

### Key Features
Polynomial Smooth Functions: Unlike traditional discrete input shapers, Smooth Input Shaper uses polynomial smooth functions for more effective smoothing of the toolhead motion.
Similar to S-Curve Acceleration: Offers an acceleration profile that is akin to S-curve acceleration, but with fixed timing instead of spanning the entire acceleration/deceleration phase.
Improved Effectiveness: Generally more effective than corresponding discrete input shapers, providing slightly more smoothing.

### Hardware Requirements
Computational Intensity: This feature is computationally more demanding.
Minimum Hardware: Raspberry Pi 3 is the minimum required hardware. It runs effectively on Ender 3 up to speeds of approximately 250 mm/sec with 127 microsteps on a Raspberry Pi 3B+.
Ideal Hardware: Raspberry Pi 4 or Orange Pi 4 are recommended for optimal performance.

### Configuration and Usage
Configuration: Configuration is similar to regular input shapers, but with some differences in parameters.
smoother_freq_? Parameter: This parameter doesn't correspond exactly to previous settings. It represents the minimum frequency that the smoother cancels or, more precisely, the smallest frequency of the pole it cancels. This distinction is particularly relevant for smooth_ei and smooth_2hump_ei shapers.
Calibration Support: The scripts/calibrate_shapers.py in the advanced-features branch supports the calibration and overview of available smoothers.
Recommendations and Considerations
Performance vs. Hardware Limitations: Given the computational demands, users should consider the capabilities of their hardware when implementing this feature.
Testing and Feedback: Extensive testing and adjustments may be necessary to achieve optimal results. User feedback based on different hardware configurations and printing scenarios is valuable.


## Extruder PA Synchronization with Input Shaping
https://klipper.discourse.group/t/extruder-pa-synchronization-with-input-shaping/3843

### Overview
Klipper's new feature, "Extruder PA Synchronization with Input Shaping," is an advanced implementation designed to enhance 3D printing quality by synchronizing filament extrusion (Pressure Advance - PA) with the toolhead's motion. This synchronization aims to reduce artifacts by compensating for changes in toolhead motion, especially in scenarios where input shaping is employed to minimize vibration and ringing.

### Background
Input shaping is a technique used to alter toolhead motion to reduce vibrations. While Klipper's existing pressure advance algorithm helps in synchronizing filament extrusion with toolhead motion, it is not fully aligned with the input shaping alterations. This misalignment can be particularly noticeable in scenarios where X and Y axes have different resonance frequencies, or the PA smooth time significantly deviates from the input shaper duration.

### Implementation
The feature is implemented in Klipper branch 85. It involves:

Calculating toolhead motion across X, Y, and Z axes.
Applying input shaping to the X and Y axes.
Using linearization to project this motion onto the E (extruder) axis.
If the input shaper is consistent for both X and Y axes, the synchronization is precise for XY motion. In other cases, the feature provides a linear approximation over X/Y deviations, which is an improvement over the previous state.

### Observations and Improvements
Extrusion Moves: The implementation shows less erratic behavior in PA during extrusion moves, with fewer retractions and deretractions.
Stable Extruder Velocity: The extruder velocity becomes more stable, reflecting the steadier toolhead velocity due to input shaping.
Wiping Behavior: Improved wiping behavior with more consistent retraction velocity.
Testing and Results
The feature has been tested over several months, showing modest improvements in the quality of real prints. It is particularly effective for direct drive extruders with short filament paths. The impact on bowden extruders is expected to be neutral.

### Usage Recommendations
Retuning PA: It is advisable to retune the pressure advance settings when using this branch. Specifically, reducing the pressure_advance_smooth_time from the default 0.04 to around 0.02 or 0.01 is recommended for direct drive extruders using non-flex filaments.
Areas to Monitor: Pay attention to areas where toolhead velocity changes, such as corners, bridges, and infill connections to perimeters, for quality improvements or degradations.
### Conclusion
This feature offers an innovative approach to synchronize extruder motion with input shaping, leading to improved print quality. Users are encouraged to experiment with this feature and provide feedback based on their printing experiences.


## New ringing tower test print
https://klipper.discourse.group/t/alternative-ringing-tower-print-for-input-shaping-calibration/4517

sample command: 
RUN_RINGING_TEST NOZZLE=0.4 TARGET_TEMP=210 BED_TEMP=55.

[ringing_test]

'''
[delayed_gcode start_ringing_test]

gcode:
    {% set vars = printer["gcode_macro RUN_RINGING_TEST"] %}
    ; Add your start GCode here, for example:
    G28
    M190 S{vars.bed_temp}
    M109 S{vars.hotend_temp}
    M106 S255
    {% set flow_percent = vars.flow_rate|float * 100.0 %}
    {% if flow_percent > 0 %}
    M221 S{flow_percent}
    {% endif %}
    {% set layer_height = vars.nozzle * 0.5 %}
    {% set first_layer_height = layer_height * 1.25 %}
    PRINT_RINGING_TOWER {vars.rawparams} LAYER_HEIGHT={layer_height} FIRST_LAYER_HEIGHT={first_layer_height} FINAL_GCODE_ID=end_ringing_test
'''

[delayed_gcode end_ringing_test]
gcode:
    ; Add your end GCode here, for example:
    M104 S0 ; turn off temperature
    M140 S0 ; turn off heatbed
    M107 ; turn off fan
    G91 ; relative positioning
    G1 Z5 ; raise Z
    G90 ; absolute positioning
    G1 X0 Y200 ; present print
    M84 ; disable steppers
    RESTORE_GCODE_STATE NAME=RINGING_TEST_STATE

[gcode_macro RUN_RINGING_TEST]
variable_bed_temp: -1
variable_hotend_temp: -1
variable_nozzle: -1
variable_flow_rate: -1
variable_rawparams: ''
gcode:
    # Fail early if the required parameters are not provided
    {% if params.NOZZLE is not defined %}
    {action_raise_error('NOZZLE= parameter must be provided')}
    {% endif %}
    {% if params.TARGET_TEMP is not defined %}
    {action_raise_error('TARGET_TEMP= parameter must be provided')}
    {% endif %}
    SET_GCODE_VARIABLE MACRO=RUN_RINGING_TEST VARIABLE=bed_temp VALUE={params.BED_TEMP|default(60)}
    SET_GCODE_VARIABLE MACRO=RUN_RINGING_TEST VARIABLE=hotend_temp VALUE={params.TARGET_TEMP}
    SET_GCODE_VARIABLE MACRO=RUN_RINGING_TEST VARIABLE=nozzle VALUE={params.NOZZLE}
    SET_GCODE_VARIABLE MACRO=RUN_RINGING_TEST VARIABLE=flow_rate VALUE={params.FLOW_RATE|default(-1)}
    SET_GCODE_VARIABLE MACRO=RUN_RINGING_TEST VARIABLE=rawparams VALUE="'{rawparams}'"
    SAVE_GCODE_STATE NAME=RINGING_TEST_STATE
    UPDATE_DELAYED_GCODE ID=start_ringing_test DURATION=0.01


## New PA tower test print
https://klipper.discourse.group/t/extruder-pa-synchronization-with-input-shaping/3843/27

sample command:
RUN_PA_TEST NOZZLE=0.4 TARGET_TEMP=205 BED_TEMP=55

[delayed_gcode start_pa_test]
gcode:
    {% set vars = printer["gcode_macro RUN_PA_TEST"] %}
    ; Add your start GCode here, for example:
    G28
    M190 S{vars.bed_temp}
    M109 S{vars.hotend_temp}
    {% set flow_percent = vars.flow_rate|float * 100.0 %}
    {% if flow_percent > 0 %}
    M221 S{flow_percent}
    {% endif %}
    TUNING_TOWER COMMAND=SET_PRESSURE_ADVANCE PARAMETER=ADVANCE START=0 FACTOR=.005
    ; PRINT_PA_TOWER must be the last command in the start_pa_test script:
    ; it starts a print and then immediately returns without waiting for the print to finish
    PRINT_PA_TOWER {vars.rawparams} FINAL_GCODE_ID=end_pa_test

[delayed_gcode end_pa_test]
gcode:
    ; Add your end GCode here, for example:
    M104 S0 ; turn off temperature
    M140 S0 ; turn off heatbed
    M107 ; turn off fan
    G91 ; relative positioning
    G1 Z5 ; raise Z
    G90 ; absolute positioning
    G1 X0 Y200 ; present print
    M84 ; disable steppers
    RESTORE_GCODE_STATE NAME=PA_TEST_STATE

[gcode_macro RUN_PA_TEST]
variable_bed_temp: -1
variable_hotend_temp: -1
variable_flow_rate: -1
variable_rawparams: ''
gcode:
    # Fail early if the required parameters are not provided
    {% if params.NOZZLE is not defined %}
    {action_raise_error('NOZZLE= parameter must be provided')}
    {% endif %}
    {% if params.TARGET_TEMP is not defined %}
    {action_raise_error('TARGET_TEMP= parameter must be provided')}
    {% endif %}
    SET_GCODE_VARIABLE MACRO=RUN_PA_TEST VARIABLE=bed_temp VALUE={params.BED_TEMP|default(60)}
    SET_GCODE_VARIABLE MACRO=RUN_PA_TEST VARIABLE=hotend_temp VALUE={params.TARGET_TEMP}
    SET_GCODE_VARIABLE MACRO=RUN_PA_TEST VARIABLE=flow_rate VALUE={params.FLOW_RATE|default(-1)}
    SET_GCODE_VARIABLE MACRO=RUN_PA_TEST VARIABLE=rawparams VALUE="'{rawparams}'"
    SAVE_GCODE_STATE NAME=PA_TEST_STATE
    UPDATE_DELAYED_GCODE ID=start_pa_test DURATION=0.01



