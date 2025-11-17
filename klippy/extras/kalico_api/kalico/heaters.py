from __future__ import annotations

import math
import typing

from klippy.gcode import CommandError

if typing.TYPE_CHECKING:
    from klippy.extras.heaters import PrinterHeaters
    from klippy.gcode import GCodeDispatch
    from klippy.printer import Printer


class HeatersAPI:
    def __init__(self, printer: Printer):
        self._printer = printer
        self._gcode: GCodeDispatch = printer.lookup_object("gcode")

    ## API for heaters
    def set_temperature(
        self, heater_name: str, temp: typing.Optional[float] = None
    ):
        "Set the target temperature for a heater"
        heaters: PrinterHeaters = self._printer.lookup_object("heaters")
        heater = heaters.lookup_heater(heater_name)
        heaters.set_temperature(heater, temp)

    def temperature_wait(
        self,
        sensor_name,
        min_temp: float = -math.inf,
        max_temp: float = math.inf,
    ):
        """
        Wait for a heater or sensor to reach a temperature

        If no minimum or maximum is given, this will wait for the heater's control loop to settle
        """
        heaters: PrinterHeaters = self._printer.lookup_object("heaters")

        if math.isinf(min_temp) and math.isinf(max_temp):
            heater = heaters.lookup_heater(sensor_name)
            heaters._wait_for_temperature(heater)

        else:
            if sensor_name in heaters.heaters:
                sensor = heaters.lookup_heater(sensor_name)
            elif sensor_name in heaters.available_sensors:
                sensor = self._printer.lookup_object(sensor_name)
            else:
                raise CommandError(
                    f"{sensor_name} is not a valid temperature sensor"
                )

            def check(eventtime):
                temp, _ = sensor.get_temp(eventtime)
                if min_temp <= temp <= max_temp:
                    return False
                self._gcode.respond_raw(heaters._get_temp(eventtime))
                return True

            self._printer.wait_while(check)
