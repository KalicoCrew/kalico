Bleeding Edge Features Documentation:
https://github.com/dmbutyugin/klipper/commits/advanced-features

## High precision stepping and new stepcompress protocol
https://klipper.discourse.group/t/improved-stepcompress-implementation/3203


## Sync extruder motion with input shaper
https://klipper.discourse.group/t/extruder-pa-synchronization-with-input-shaping/3843


## New ringing tower test print
https://klipper.discourse.group/t/alternative-ringing-tower-print-for-input-shaping-calibration/4517

sample command:
RUN_RINGING_TEST NOZZLE=0.4 TARGET_TEMP=210 BED_TEMP=55.

[ringing_test]

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



