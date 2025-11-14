from __future__ import annotations
import collections
import contextlib
import logging
import math
import shlex
import threading
import typing

from klippy import configfile
from klippy.gcode import CommandError

if typing.TYPE_CHECKING:
    from klippy.extras.fan import PrinterFan
    from klippy.extras.fan_generic import PrinterFanGeneric
    from klippy.extras.gcode_macro import GCodeMacro
    from klippy.extras.gcode_move import GCodeMove
    from klippy.extras.heaters import PrinterHeaters
    from klippy.extras.save_variables import SaveVariables
    from klippy.gcode import GCodeDispatch
    from klippy.printer import Printer


BlockingResult = typing.TypeVar("BlockingResult")


class PythonGcodeWrapper:
    def __init__(self, gcode: GCodeDispatch):
        self._gcode = gcode

    def __getattr__(self, command: str) -> GCodeCommandWrapper:
        if command.upper() not in self._gcode.status_commands:
            raise AttributeError(f"No such GCode command {command!r}")
        return GCodeCommandWrapper(self._gcode, command)

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


class GCodeCommandWrapper:
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
        heaters.set_temperature(heater_name, temp)

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

            @self.wait_until
            def check(eventtime):
                temp, _ = sensor.get_temp(eventtime)
                if min_temp <= temp <= max_temp:
                    return True
                self._gcode.respond_raw(heaters._get_temp(eventtime))
                return False


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


class MoveAPI:
    def __init__(self, printer: Printer):
        self._gcode_move: GCodeMove = printer.lookup_object("gcode_move")

    def __call__(
        self,
        x: typing.Optional[float] = None,
        y: typing.Optional[float] = None,
        z: typing.Optional[float] = None,
        e: typing.Optional[float] = None,
        *,
        dx: float = 0.0,
        dy: float = 0.0,
        dz: float = 0.0,
        de: float = 0.0,
        speed: typing.Optional[float] = None,
    ):
        """
        Move to a position

        `speed` is in mm/s and unlike `G1 Fx` only affects this movement.
        """
        pos = self._gcode_move.last_position
        newpos = [
            (x if x is not None else pos[0]) + dx,
            (y if y is not None else pos[1]) + dy,
            (z if z is not None else pos[2]) + dz,
            (e if e is not None else pos[3]) + de,
        ]
        self._gcode_move.move_to(newpos, speed)

    def set_gcode_offset(
        self,
        x: typing.Optional[float] = None,
        y: typing.Optional[float] = None,
        z: typing.Optional[float] = None,
        *,
        dx: float = 0.0,
        dy: float = 0.0,
        dz: float = 0.0,
        move: bool = False,
        speed: float = None,
    ):
        """
        Set GCode offsets

        `speed` is in mm/s
        """
        offsets = self._gcode_move.homing_position
        x = (x if x is not None else offsets[0]) + dx
        y = (y if y is not None else offsets[1]) + dy
        z = (z if z is not None else offsets[2]) + dz
        self._gcode_move.set_gcode_offset(x, y, z, move=move, speed=speed)

    def set_speed(self, speed: float):
        "Set the speed for future moves in mm/s"
        self._gcode_move.set_speed(speed)

    def set_speed_factor(self, speed_factor: float = 1.0):
        "Set the movement speed factor"
        self._gcode_move.set_speed_factor(speed_factor)

    def set_extrude_factor(self, extrude_factor: float = 1.0):
        self._gcode_move.set_extrude_factor(extrude_factor)


class PythonMacroContext:
    'The magic "Printer" object for macros'

    _context: list[dict]

    status: GetStatusWrapperPython
    vars: TemplateVariableWrapperPython
    saved_vars: SaveVariablesWrapper

    gcode: PythonGcodeWrapper
    move: MoveAPI
    heaters: HeatersAPI

    def __init__(self, printer: Printer, name: str):
        self._name = name
        self._printer = printer

        self._gcode: GCodeDispatch = printer.lookup_object("gcode")
        self._gcode_macro: GCodeMacro = printer.lookup_object(
            f"gcode_macro {name}"
        )

        self.status = GetStatusWrapperPython(printer)
        self.vars = TemplateVariableWrapperPython(self._gcode_macro)
        self.saved_vars = SaveVariablesWrapper(printer)

        self.move = MoveAPI(printer)
        self.heaters = HeatersAPI(printer)
        self.gcode = PythonGcodeWrapper(self._gcode)

        self._context = []

    @property
    def raw_params(self) -> str:
        if not self._context:
            return ""
        return self._context[-1].get("rawparams", "")

    @property
    def params(self) -> dict[str, str]:
        if not self._context:
            return {}
        return self._context[-1].get("params", {})

    @contextlib.contextmanager
    def _with_context(self, context: dict):
        self._context.append(context)
        try:
            yield self
        finally:
            self._context.remove(context)

    def wait_while(self, condition: typing.Callable[[], bool]):
        "Wait while a condition is True"

        def inner(eventtime):
            return condition()

        self._printer.wait_while(inner)

    def wait_until(self, condition: typing.Callable[[], bool]):
        "Wait until a condition is True"

        def inner(eventtime):
            return not condition()

        self._printer.wait_until(condition)

    def wait_moves(self):
        "Wait until all moves are completed"
        toolhead = self._printer.lookup_object("toolhead")
        toolhead.wait_moves()

    def blocking(
        self, function: typing.Callable[[], BlockingResult]
    ) -> BlockingResult:
        "Run a blocking task in a thread, waiting for the result"
        completion = self._printer.get_reactor().completion()

        def run():
            try:
                ret = function()
                completion.complete((False, ret))
            except Exception as e:
                completion.complete((True, e))

        t = threading.Thread(target=run, daemon=True)
        t.start()
        [is_exception, ret] = completion.wait()
        if is_exception:
            raise ret
        else:
            return ret

    def sleep(self, timeout: float):
        "Wait a given number of seconds"
        reactor = self._printer.get_reactor()
        deadline = reactor.monotonic() + timeout

        def check(event):
            return deadline > reactor.monotonic()

        self._printer.wait_while(check)

    def set_gcode_variable(self, macro: str, variable: str, value: typing.Any):
        "Save a variable to a gcode_macro"
        macro = self._printer.lookup_object(f"gcode_macro {macro}")
        macro.variables = {**macro.variables, variable: value}

    def emergency_stop(self, msg: str = "action_emergency_stop"):
        "Immediately shutdown Kalico"
        self._printer.invoke_shutdown(f"Shutdown due to {msg}")

    def respond(self, prefix: str, msg: str):
        "Send a message to the console"
        self._gcode.respond_raw(f"{prefix} {msg}")

    def respond_info(self, msg: str):
        "Send a message to the console"
        self._gcode.respond_info(msg)

    def respond_raw(self, msg: str):
        self._gcode.respond_raw(msg)

    def raise_error(self, msg):
        "Raise a G-Code command error"
        raise self._printer.command_error(msg)

    def call_remote_method(self, method: str, **kwargs):
        "Call a Kalico webhooks method"
        webhooks = self._printer.lookup_object("webhooks")
        try:
            webhooks.call_remote_method(method, **kwargs)
        except self._printer.command_error:
            logging.exception("Remote call error")

    # TODO: Should this be `move.save_state()`?
    @contextlib.contextmanager
    def save_gcode_state(
        self,
        name: str = None,
        move_on_restore: bool = False,
        move_speed: float = None,
    ):
        "Save and restore the current gcode state"
        if name is None:
            name = self._name
        self.gcode.save_gcode_state(name=name)
        try:
            yield
        finally:
            self.gcode.restore_gcode_state(
                name=name, move=move_on_restore, move_speed=move_speed
            )


class TemplateVariableWrapperPython:
    def __init__(self, macro):
        self.__macro = macro

    def __setitem__(self, name, value):
        v = dict(self.__macro.variables)
        v[name] = value
        self.__macro.variables = v

    def __getitem__(self, name):
        return self.__macro.variables[name]

    def __contains__(self, val):
        return val in self.__macro.variables

    def __iter__(self):
        yield from iter(self.__macro.variables)

    def items(self):
        return self.__macro.variables.items()


class SaveVariablesWrapper:
    def __init__(self, printer: Printer):
        self._save_variables: SaveVariables = printer.lookup_object(
            "save_variables", None
        )

    def __getitem__(self, name):
        if self._save_variables is None:
            raise CommandError("save_variables is not enabled")
        return self._save_variables.allVariables[name]

    def __setitem__(self, name, value):
        if self._save_variables is None:
            raise CommandError("save_variables is not enabled")
        self._save_variables.set_variable(name, value)

    def __contains__(self, name):
        return (
            self._save_variables and name in self._save_variables.allVariables
        )

    def __iter__(self):
        yield from iter(self._save_variables.allVariables)

    def items(self):
        return self._save_variables.allVariables.items()


class GetStatusWrapperPython:
    def __init__(self, printer):
        self._printer = printer

    def __getitem__(self, val) -> StatusWrapper:
        sval = str(val).strip()
        po = self._printer.lookup_object(sval, None)
        if po is None or not hasattr(po, "get_status"):
            raise KeyError(val)
        eventtime = self._printer.get_reactor().monotonic()
        return StatusWrapper(po.get_status(eventtime))

    def __getattr__(self, val) -> StatusWrapper:
        return self.__getitem__(val)

    def __contains__(self, val):
        try:
            self.__getitem__(val)
        except KeyError as e:
            return False
        return True

    def __iter__(self):
        for name, obj in self._printer.lookup_objects():
            if hasattr(obj, "get_status"):
                yield name

    def get(self, key: str, default: configfile.sentinel) -> StatusWrapper:
        try:
            return self[key]
        except KeyError:
            if default is not configfile.sentinel:
                return default
            raise


class StatusWrapper(collections.UserDict):
    def __init__(self, dict):
        self.data = dict

    def __getattr__(self, name):
        return self.__getitem__(name)
