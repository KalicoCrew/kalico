from __future__ import annotations

import typing

from klippy.gcode import CommandError

if typing.TYPE_CHECKING:
    from klippy.extras.fan import PrinterFan
    from klippy.extras.fan_generic import PrinterFanGeneric
    from klippy.printer import Printer


class FanAPI:
    def __init__(self, printer: Printer):
        self._printer = printer

    def set_speed(self, fan_name: str, speed: float):
        "Set the speed of a fan"
        if fan_name == "fan":
            fan: PrinterFan = self._printer.lookup_object("fan", None)
        else:
            fan: PrinterFanGeneric = self._printer.lookup_object(
                f"fan_generic {fan_name}", None
            )
        if not fan:
            raise CommandError(f"No fan {fan_name} found")

        fan.fan.set_speed_from_command(speed)
