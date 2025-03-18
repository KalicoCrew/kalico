# Load Cell Probe
#
# Copyright (C) 2025  Gareth Farrington <gareth@waves.ky>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from klippy.extras.load_cell import ads1220, hx71x
from klippy.extras.load_cell.load_cell_probe import LoadCellPrinterProbe
from klippy.printer import Printer, SubsystemComponentCollection


# register sensors that implement LoadCellSensor
def register_components(subsystem: SubsystemComponentCollection):
    sensors = hx71x.HX71X_SENSOR_TYPES | ads1220.ADS1220_SENSOR_TYPE
    for name, sensor in sensors.items():
        subsystem.register_component("load_cell_probe_sensors", name, sensor)


def load_config(config):
    printer: Printer = config.get_printer()
    sensors = printer.lookup_components("load_cell_probe_sensors")
    sensor_class = config.getchoice("sensor_type", sensors)
    return LoadCellPrinterProbe(config, sensor_class(config))
