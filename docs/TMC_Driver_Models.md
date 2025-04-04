# TMC Driver Model Definitions

This document explains how to use the `driver_model` configuration option for TMC stepper drivers.

## Overview

Many stepper motor drivers use non-standard sense resistor values, which can lead to incorrect current settings if not properly configured. Incorrect current settings may result in poor motor performance or even hardware damage.

Kalico addresses this by allowing you to specify predefined driver models with reviewed sense resistor and maximum current values, eliminating the need to manually look up and configure these values.

## Configuration

You can configure your TMC drivers in two ways:

1. **Recommended**: Use `driver_model` to select a predefined configuration
   ```
   [tmc2209 stepper_x]
   driver_model: REFERENCE_2209
   ```

2. **Alternative**: Manually specify the sense resistor value
   ```
   [tmc2209 stepper_x]
   sense_resistor: 0.11
   ```

> [!IMPORTANT]
> Either `driver_model` or `sense_resistor` **must** be specified. Kalico will raise an error if neither option is provided, as this could lead to incorrect current settings and potential hardware damage.

If both `driver_model` and `sense_resistor` are specified, Kalico will use the provided `sense_resistor` value and issue a warning.

> [!NOTE]
> Some drivers with variable sense resistors (like FYSETC EXT2160) are intentionally omitted from the table. For these drivers, you must manually set the sense_resistor value.

## Available Driver Models

| Driver Type                             | Config Name             | Sense Resistor | Max Current | Source |
|-----------------------------------------|-------------------------|----------------|-------------|--------|
| LDO Leviathan HV0,HV1                   | REFERENCE_5160          | 0.075          | 3           | [User Manual](https://github.com/MotorDynamicsLab/Leviathan/blob/master/Manual/Leviathan_V1.2_Manual.pdf) |
| LDO Leviathan S0-4                      | REFERENCE_2209          | 0.11           | 2           | [User Manual](https://github.com/MotorDynamicsLab/Leviathan/blob/master/Manual/Leviathan_V1.2_Manual.pdf) |
| BTT Kraken S1-4                         | KRAKEN_2160_8A          | 0.022          | 8           | [User Manual](https://github.com/bigtreetech/BIGTREETECH-Kraken/blob/master/BIGTREETECH%20Kraken%20V1.0%20User%20Manual.pdf) |
| BTT Kraken S5-8                         | REFERENCE_5160          | 0.075          | 3           | [User Manual](https://github.com/bigtreetech/BIGTREETECH-Kraken/blob/master/BIGTREETECH%20Kraken%20V1.0%20User%20Manual.pdf) |
| BTT TMC2208                             | REFERENCE_2209          | 0.11           | 2           | [Schematics](https://github.com/bigtreetech/BIGTREETECH-Stepper-Motor-Driver/blob/master/TMC2208/V3.0/Hardware/TMC2208-V3.0SCH.pdf) |
| BTT TMC2209                             | REFERENCE_2209          | 0.11           | 2           | [User Manual](https://github.com/bigtreetech/BIGTREETECH-Stepper-Motor-Driver/blob/master/TMC2209/V1.3/manual/BIGTREETECH%20TMC2209%20V1.3%20User%20Manual.pdf) |
| BTT TMC5160(T)                          | REFERENCE_5160          | 0.075          | 3           | [User Manual](https://github.com/bigtreetech/BIGTREETECH-Stepper-Motor-Driver/blob/master/TMC5160(T)/manual/BIGTREETECH%20TMC5160-V1.0%20manual.pdf) |
| BTT TMC5160T Pro                        | REFERENCE_5160          | 0.075          | 3           | [User Manual](https://github.com/bigtreetech/BIGTREETECH-Stepper-Motor-Driver/blob/master/TMC5160_Pro%20V1.0/BIGTREETECH%20TMC5160%20PRO-V1.0%20user%20manual.pdf) |
| BTT TMC5160T Plus                       | BTT_EXT_5160            | 0.022          | 10.6        | [User Manual](https://github.com/bigtreetech/BIGTREETECH-Stepper-Motor-Driver/blob/master/TMC5160T%20Plus/BIGTREETECH%20TMC5160T%20Plus%20User%20Manual.pdf) |
| BTT TMC5161                             | REFERENCE_5160          | 0.075          | 3           | [User Manual](https://github.com/bigtreetech/BIGTREETECH-Stepper-Motor-Driver/blob/master/TMC5161/TMC5161%20v1.0%20mamual.pdf) |
| BTT EZ2130                              | REFERENCE_2209          | 0.11           | 2           | [Schematics](https://github.com/bigtreetech/BIGTREETECH-EZ-Driver/blob/main/BIGTREETECH%20EZ2130%20V1.0/Hardware/BIGTREETECH%20EZ2130%20V1.0-SCH.pdf) |
| BTT EZ2208                              | REFERENCE_2209          | 0.11           | 2           | [Schematics](https://github.com/bigtreetech/BIGTREETECH-EZ-Driver/blob/main/BIGTREETECH%20EZ2208%20V1.0/BIGTREETECH%20EZ2208%20V1.0-SCH.pdf) |
| BTT EZ2209                              | REFERENCE_2209          | 0.11           | 2           | [Schematics](https://github.com/bigtreetech/BIGTREETECH-EZ-Driver/blob/main/BIGTREETECH%20EZZ2209%20V1.0/BIGTREETECH%20EZ2209%20V1.0-SCH.pdf) |
| BTT EZ2225                              | REFERENCE_2209          | 0.11           | 2           | [Schematics](https://github.com/bigtreetech/BIGTREETECH-EZ-Driver/blob/main/BIGTREETECH%20EZ2225%20V1.0/Hardware/BIGTREETECH%20EZ2225%20V1.0-SCH.pdf) |
| BTT EZ2226                              | REFERENCE_2209          | 0.11           | 2           | [Schematics](https://github.com/bigtreetech/BIGTREETECH-EZ-Driver/blob/main/BIGTREETECH%20EZ2226%20V1.0/Hardware/BIGTREETECH%20EZ2226%20V1.0-SCH.pdf) |
| BTT EZ5160 Pro V1.0                     | BTT_EZ_5160_PRO         | 0.075          | 2.5         | [User Manual](https://github.com/bigtreetech/BIGTREETECH-EZ-Driver/blob/main/BIGTREETECH%20EZ5160%20Pro%20V1.0/BIGTREETECH%20EZ5160%20V1.0%20User%20Manual.pdf) |
| BTT EZ5160 RGB                          | BTT_EZ_5160_RGB         | 0.05           | 4.7         | [User Manual](https://github.com/bigtreetech/BIGTREETECH-EZ-Driver/blob/main/BIGTREETECH%20EZ5160RGB/Hardware/BIGTREETECH%20EZ5160RGB%20v1.0%20User%20Manual.pdf) |
| COREVUS TMC2209                         | COREVUS_2209            | 0.1            | 3           |
| COREVUS TMC2160 OLD                     | COREVUS_2160_OLD        | 0.03           | 3           | [GitHub](https://github.com/calithameridi/corevus/blob/main/docs/driver-modules/modules.md#tmc2160) |
| COREVUS TMC2160 5A                      | COREVUS_2160_5A         | 0.03           | 5           | [GitHub](https://github.com/calithameridi/corevus/blob/main/docs/driver-modules/modules.md#tmc2160-5a) |
| COREVUS Dual TMC2160                    | COREVUS_DUAL_2160       | 0.05           | 3           | [GitHub](https://github.com/calithameridi/corevus/blob/main/docs/driver-modules/modules.md#dual-tmc2160) |
| Watterott SilentStepStick TMC2100       | REFERENCE_WOTT          | 0.11           | 1.2         | [Schematics](https://github.com/watterott/SilentStepStick/blob/master/hardware/SilentStepStick-TMC2100_v20.pdf) |
| Watterott SilentStepStick TMC2130       | REFERENCE_WOTT          | 0.11           | 1.2         | [Schematics](https://github.com/watterott/SilentStepStick/blob/master/hardware/SilentStepStick-TMC2130_v20.pdf) |
| Watterott SilentStepStick TMC2208       | REFERENCE_WOTT          | 0.11           | 1.2         | [Schematics](https://github.com/watterott/SilentStepStick/blob/master/hardware/SilentStepStick-TMC2208_v20.pdf) |
| Watterott SilentStepStick TMC2209       | WOTT_2209               | 0.11           | 1.7         | [Schematics](https://github.com/watterott/SilentStepStick/blob/master/hardware/SilentStepStick-TMC2209_v20.pdf) |
| Watterott SilentStepStick TMC5160       | REFERENCE_5160          | 0.075          | 3           | [Schematics](https://github.com/watterott/SilentStepStick/blob/master/hardware/SilentStepStick-TMC5160_v15.pdf) |
| Watterott SilentStepStick TMC5160HV     | REFERENCE_5160          | 0.075          | 3           | [Schematics](https://github.com/watterott/SilentStepStick/blob/master/hardware/SilentStepStick-TMC5160_v15.pdf) |
| FYSETC TMC2130                          | REFERENCE_WOTT          | 0.11           | 1.2         |
| FYSETC TMC2208                          | REFERENCE_WOTT          | 0.11           | 1.2         |
| FYSETC TMC2209                          | WOTT_2209               | 0.11           | 1.7         | [GitHub](https://github.com/FYSETC/FYSETC-TMC2209?tab=readme-ov-file#motor-current-setting) |
| FYSETC TMC2225                          | FYSETC_2225             | 0.11           | 1.4         |
| FYSETC TMC2226                          | REFERENCE_2209          | 0.11           | 2           |
| FYSETC HV5160                           | REFERENCE_5160          | 0.075          | 3           | [GitHub](https://github.com/FYSETC/FYSETC-HV5160?tab=readme-ov-file#4-specifications) |
| FYSETC QHV5160                          | REFERENCE_5160          | 0.075          | 3           | [GitHub](https://github.com/FYSETC/FYSETC-QHV5160?tab=readme-ov-file#4-specifications) |
| FYSETC Silent5161                       | FYSETC_5161             | 0.06           | 3.5         |
| MKS 2130                                | REFERENCE_2209          | 0.11           | 2           | [Schematics](https://github.com/makerbase-mks/MKS-StepStick-Driver/blob/master/MKS%20TMC2130/MKS%20TMC2130%20V1.0_001/MKS%20TMC2130%20V1.0_001%20SCH.pdf) |
| MKS 2208                                | REFERENCE_2209          | 0.11           | 2           | [Schematics](https://github.com/makerbase-mks/MKS-StepStick-Driver/blob/master/MKS%20TMC2208/MKS%20TMC2208%20V2.0_001/MKS%20TMC2208%20V2.0_001%20SCH.pdf) |
| MKS 2209                                | REFERENCE_2209          | 0.11           | 2           | [Schematics](https://github.com/makerbase-mks/MKS-StepStick-Driver/blob/master/MKS%20TMC2209/MKS%20TMC2209%20V2.0_001/MKS%20TMC2209%20V2.0_001%20SCH.pdf) |
| MKS 2225                                | REFERENCE_2209          | 0.11           | 2           | [Schematics](https://github.com/makerbase-mks/MKS-StepStick-Driver/blob/master/MKS%20TMC2225/MKS%20TMC2225%20V1.0_003/MKS%20TMC2225%20V1.0_003%20SCH.pdf) |
| MKS 2226                                | REFERENCE_2209          | 0.11           | 2           | [Schematics](https://github.com/makerbase-mks/MKS-StepStick-Driver/blob/master/MKS%20TMC2226/MKS%20TMC2226%20V1.0_001/MKS%20TMC2226%20V1.0_001%20SCH.pdf) |
| Mellow Fly 2209                         | REFERENCE_2209          | 0.11           | 2           | [Schematics](https://github.com/Mellow-3D/Fly-Drivers/blob/master/2209/Fly-2209-Schematic.pdf) |
| Mellow Fly 5160                         | REFERENCE_5160          | 0.75           | 3           | [Schematics](https://github.com/Mellow-3D/Fly-Drivers/blob/master/5160/Fly-5160-Schematic%20(stepstick).pdf) |
| Mellow Fly HV-TMC5160 Pro               | MELLOW_FLY_HV_5160_PRO  | 0.033          | 6           | [Docs](https://mellow-3d.github.io/fly_hv-tmc5160pro_general.html#firmware) |
