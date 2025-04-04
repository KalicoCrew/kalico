# TMC stepper driver definitions
#
# This file contains definitions for various TMC stepper driver types.
# Each entry maps a driver model to a tuple of (sense_resistor, max_current).
# These values are used when a user specifies a 'driver_model' in their config
# instead of manually setting the 'sense_resistor' value.
#
# Format: "DRIVER_NAME": (sense_resistor_value, max_current_value)

DRIVER_DEFS = {
    "REFERENCE_WOTT": (0.11, 1.2),
    "REFERENCE_2209": (0.11, 2),
    "REFERENCE_5160": (0.075, 3),
    "KRAKEN_2160_8A": (0.022, 8),
    "BTT_EZ_5160_PRO": (0.075, 2.5),
    "BTT_EZ_5160_RGB": (0.05, 4.7),
    "BTT_EXT_5160": (0.022, 10.6),
    "WOTT_2209": (0.11, 1.7),
    "COREVUS_2209": (0.1, 3),
    "COREVUS_2160_OLD": (0.03, 3),
    "COREVUS_2160_5A": (0.03, 5),
    "COREVUS_DUAL_2160": (0.05, 3),
    "FYSETC_2225": (0.11, 1.4),
    "FYSETC_5161": (0.06, 3.5),
    "MELLOW_FLY_HV_5160_PRO": (0.033, 6),
}
