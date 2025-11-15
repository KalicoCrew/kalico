from __future__ import annotations

import shlex
import typing

if typing.TYPE_CHECKING:
    from klippy.gcode import GCodeDispatch


class GCodeAPI:
    def __init__(self, gcode: GCodeDispatch):
        self._gcode = gcode

    def __getattr__(self, command: str) -> GCodeCommand:
        if command.upper() not in self._gcode.status_commands:
            raise AttributeError(f"No such GCode command {command!r}")
        return GCodeCommand(self._gcode, command)

    def __call__(self, command: str):
        self._gcode.run_script_from_command(command)

    def absolute_movement(self):
        self._gcode.run_script_from_command("G90")

    def relative_movement(self):
        self._gcode.run_script_from_command("G91")

    def absolute_extrusion(self):
        self._gcode.run_script_from_command("M82")

    def relative_extrusion(self):
        self._gcode.run_script_from_command("M83")

    def display(self, msg: str):
        self._gcode.run_script_from_command(f"M117 {msg}")


class GCodeCommand:
    def __init__(self, gcode: GCodeDispatch, command: str):
        self._gcode = gcode
        self._command = command

    def _serialize_value(self, value):
        if value is True:
            return "1"
        if value is False:
            return "0"
        return shlex.quote(str(value))

    def format(self, *args, **params):
        command = [self._command]
        if args:
            command.extend(map(str, args))

        for key, raw_value in params.items():
            if raw_value is None:
                continue

            value = self._serialize_value(raw_value)
            if (
                self._gcode.is_traditional_gcode(self._command)
                and len(key) == 1
            ):
                command.append(f"{key}{value}")
            else:
                command.append(f"{key}={value}")

        return " ".join(command)

    def __call__(self, *args: str, **params):
        self._gcode.run_script_from_command(self.format(*args, **params))
