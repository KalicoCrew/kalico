# Load cell module setup
#
# Copyright (C) 2025  Gareth Farrington <gareth@waves.ky>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

from . import hx71x, ads1220
from .load_cell import LoadCell


def load_config(config):
    # Sensor types
    sensors = {}
    sensors.update(hx71x.HX71X_SENSOR_TYPES)
    sensors.update(ads1220.ADS1220_SENSOR_TYPE)
    sensor_class = config.getchoice("sensor_type", sensors)
    return LoadCell(config, sensor_class(config))


def load_config_prefix(config):
    return load_config(config)
