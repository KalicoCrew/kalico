# Package definition for the typings directory
#
# Copyright (C) 2025  Gareth Farrington <gareth@waves.ky>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

from collections.abc import Callable
from typing import Protocol, TypedDict, Union

from klippy.mcu import MCU


class BulkAdcData(TypedDict):
    """Dictionary returned by sensors containing raw sensor data"""

    data: Union[list[int], list[None]]
    errors: int
    overflows: int


"""Clients of the ADC receive a BulkAdcData dictionary and return True or 
False. Returning False causes the client to disconnect, True keeps the 
client connected"""
BulkAdcDataCallback = Callable[[BulkAdcData], bool]


class BulkAdcSensor(Protocol):
    """Attach a client to the sensor to receive BulkAdcData"""

    def get_mcu(self) -> MCU:
        """Get the MCU the sensor is attached to"""
        ...

    def get_samples_per_second(self) -> int:
        """Return the sample per second of the sensor"""
        ...

    def get_range(self) -> tuple[int, int]:
        """Return the range of the sensor as (min, max)"""
        ...

    def add_client(self, callback: BulkAdcDataCallback):
        """Attach a client to the sensor to receive BulkAdcData"""
        ...


class McuLoadCellProbe(Protocol):
    """Attach a sensor to a LoadCell object on the MCU"""

    def attach_load_cell_probe(self, load_cell_probe_oid: int): ...


class LoadCellSensor(BulkAdcSensor, McuLoadCellProbe):
    """The complete load cell interface"""

    ...
