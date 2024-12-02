# Stepstick Type Definitions

This page contains information to assist in defining the `stepstick_type` config entry.

In stock Klipper behavior, selecting a driver automatically uses the TMC definitions' default sense resistor. While this is usually fine, it can cause issues with driver boards that have unconventional sense resistor sizes, leading to incorrect current values being set.

The goal of these changes to DangerKlipper is to allow you to set custom sense resistor and max current values for your stepper drivers. This results in more accurate configurations for your setup and enhances safety.


## Use
In practice, you should set your driver using `stepstick_type`. If you need to override the default value, you can then use `sense_resistor`. Additionally, `stepstick_type` is not required to use `sense_resistor`. If neither `stepstick_type` or `sense_resistor` is set, the default
Klipper behavior of using the TMC default value will be used (i.e. 0.075 for 5160).

> [!NOTE]
> Drivers with variable sense resistors (such as the FYSETC EXT2160) have not been defined and have been left out of the table intentionally. You must set sense resistor for them manually.


| Stepstick Type                          | Config Name             | Sense Resistor | Max Current |
|-----------------------------------------|-------------------------|----------------|-------------|
| BTT Kraken S1-4                         | KRAKEN_2160_8A          | 0.022          | 8           |
| BTT Kraken S5-8                         | KRAKEN_2160_3A          | 0.022          | 3           |
| LDO Leviathan HV0,HV1                   | REFERENCE_5160          | 0.075          | 3           |
| LDO Leviathan S0-4                      | REFERENCE_2209          | 0.11           | 2           |
| BTT TMC2208                             | REFERENCE_2209          | 0.11           | 2           |
| BTT TMC2209                             | REFERENCE_2209          | 0.11           | 2           |
| BTT TMC2240                             | BTT_2240                | 0.11           | 2           |
| BTT TMC5160T Pro                        | REFERENCE_5160          | 0.075          | 3           |
| BTT EZ2130                              | REFERENCE_2209          | 0.11           | 2           |
| BTT EZ2208                              | REFERENCE_2209          | 0.11           | 2           |
| BTT EZ2209                              | REFERENCE_2209          | 0.11           | 2           |
| BTT EZ2225                              | REFERENCE_2209          | 0.11           | 2           |
| BTT EZ2226                              | REFERENCE_2209          | 0.11           | 2           |
| BTT EZ5160 Pro                          | BTT_EZ_5160_PRO         | 0.075          | 2.5         |
| BTT EZ5160 RGB                          | BTT_EZ_5160_RGB         | 0.05           | 4.7         |
| BTT EZ6609                              | REFERENCE_2209          | 0.11           | 2           |
| BTT TMC5160T Plus                       | BTT_EXT_5160            | 0.022          | 10.6        |
| COREVUS TMC2209                         | COREVUS_2209            | 0.1            | 3           |
| COREVUS TMC2160 OLD                     | COREVUS_2160_OLD        | 0.03           | 3           |
| COREVUS TMC2160 5A                      | COREVUS_2160_5A         | 0.03           | 5           |
| COREVUS TMC2160                         | COREVUS_2160            | 0.05           | 3           |
| Watterott SilentStepStick TMC2100       | REFERENCE_WOTT          | 0.11           | 1.2         |
| Watterott SilentStepStick TMC2130       | REFERENCE_WOTT          | 0.11           | 1.2         |
| Watterott SilentStepStick TMC2208       | REFERENCE_WOTT          | 0.11           | 1.2         |
| Watterott SilentStepStick TMC2209       | WOTT_2209               | 0.11           | 1.7         |
| Watterott SilentStepStick TMC5160       | REFERENCE_5160          | 0.075          | 3           |
| Watterott SilentStepStick TMC5160HV     | REFERENCE_5160          | 0.075          | 3           |
| FYSETC TMC2100                          | REFERENCE_WOTT          | 0.11           | 1.2         |
| FYSETC TMC2130                          | REFERENCE_WOTT          | 0.11           | 1.2         |
| FYSETC TMC2208                          | REFERENCE_WOTT          | 0.11           | 1.2         |
| FYSETC TMC2209                          | WOTT_2209               | 0.11           | 1.7         |
| FYSETC TMC2225                          | FYSETC_2225             | 0.11           | 1.4         |
| FYSETC TMC2226                          | REFERENCE_2209          | 0.11           | 2           |
| FYSETC HV5160                           | REFERENCE_5160          | 0.075          | 3           |
| FYSETC QHV5160                          | REFERENCE_5160          | 0.075          | 3           |
| FYSETC Silent5161                       | FYSETC_5161             | 0.06           | 3.5         |
| MKS 2130                                | REFERENCE_2209          | 0.11           | 2           |
| MKS 2208                                | REFERENCE_2209          | 0.11           | 2           |
| MKS 2209                                | REFERENCE_2209          | 0.11           | 2           |
| MKS 2225                                | REFERENCE_2209          | 0.11           | 2           |
| MKS 2226                                | MKS_2226                | 0.17           | 2.5         |
| MKS 2240                                | REFERENCE_2209          | 0.11           | 2           |
| Mellow Fly 2209                         | REFERENCE_2209          | 0.11           | 2           |
| Mellow Fly 5160                         | MELLOW_FLY_5160         | 0.11           | 3           |
| Mellow Fly HV-TMC5160 Pro               | MELLOW_FLY_HV_5160_Pro  | 0.033          | 6           |
