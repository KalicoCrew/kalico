from __future__ import annotations
import math
import logging
import threading
import contextlib
import typing
import shlex
from klippy import configfile
from klippy.gcode import CommandError

if typing.TYPE_CHECKING:
    from klippy.gcode import GCodeDispatch
    from klippy.printer import Printer
    from klippy.extras.heaters import PrinterHeaters
    from klippy.extras.save_variables import SaveVariables


BlockingResult = typing.TypeVar("BlockingResult")


class PythonGcodeWrapper:
    def __init__(self, gcode: GCodeDispatch):
        self.__gcode = gcode

    def __getattr__(self, command: str) -> GCodeCommandWrapper:
        if command.upper() not in self.__gcode.status_commands:
            raise AttributeError(f"No such GCode command {command!r}")
        return GCodeCommandWrapper(self.__gcode, command)

    def __call__(self, command: str):
        self.__gcode.run_script_from_command(command)


class GCodeCommandWrapper:
    def __init__(self, gcode: GCodeDispatch, command: str):
        self.__gcode = gcode
        self.__command = command

    def _serialize_value(self, value):
        if value is True:
            return "1"
        if value is False:
            return "0"
        return shlex.quote(str(value))

    def format(self, *args, **params):
        command = [self.__command]
        if args:
            command.extend(map(str, args))

        for key, raw_value in params.items():
            if raw_value is None:
                continue

            value = self._serialize_value(raw_value)
            if (
                self.__gcode.is_traditional_gcode(self.__command)
                and len(key) == 1
            ):
                command.append(f"{key}{value}")
            else:
                command.append(f"{key}={value}")

        return " ".join(command)

    def __call__(self, *args: str, **params):
        self.__gcode.run_script_from_command(self.format(*args, **params))


class PythonMacroContext:
    'The magic "Printer" object for macros'

    status: dict[str, dict[str, typing.Any]]
    vars: dict[str, typing.Any]

    raw_params: str
    params: dict[str, str]

    gcode: GCodeCommandWrapper

    def __init__(self, printer: Printer, name: str, context: dict):
        self._printer = printer
        self._gcode = printer.lookup_object("gcode")
        self._gcode_macro = printer.lookup_object(f"gcode_macro {name}")
        self._name = name

        self.status = GetStatusWrapperPython(printer)
        self.vars = TemplateVariableWrapperPython(self._gcode_macro)
        self.saved_vars = SaveVariablesWrapper(printer)

        self.raw_params = context.get("raw_params", None)
        self.params = context.get("params", {})

        self.gcode = PythonGcodeWrapper(self._gcode)

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
                self.raise_error(
                    f"{sensor_name} is not a valid temperature sensor"
                )

            @self.wait_until
            def check(eventtime):
                temp, _ = sensor.get_temp(eventtime)
                if min_temp <= temp <= max_temp:
                    return True
                self.respond_raw(heaters._get_temp(eventtime))
                return False

    def set_fan_speed(self, fan_name: str, speed: float):
        "Set the speed of a fan"
        if fan_name == "fan":
            fan = self._printer.lookup_object("fan", None)
        else:
            fan = self._printer.lookup_object(f"fan_generic {fan_name}", None)
        if not fan:
            raise CommandError(f"No fan {fan_name} found")

        fan.fan.set_speed_from_command(speed)

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
        self.__save_variables: SaveVariables = printer.lookup_object(
            "save_variables", None
        )

    def __getitem__(self, name):
        if self.__save_variables is None:
            raise CommandError("save_variables is not enabled")
        return self.__save_variables.allVariables[name]

    def __setitem__(self, name, value):
        if self.__save_variables is None:
            raise CommandError("save_variables is not enabled")
        self.__save_variables.set_variable(name, value)

    def __contains__(self, name):
        return (
            self.__save_variables and name in self.__save_variables.allVariables
        )

    def __iter__(self):
        yield from iter(self.__save_variables.allVariables)

    def items(self):
        return self.__save_variables.allVariables.items()


class GetStatusWrapperPython:
    def __init__(self, printer):
        self.printer = printer

    def __getitem__(self, val):
        sval = str(val).strip()
        po = self.printer.lookup_object(sval, None)
        if po is None or not hasattr(po, "get_status"):
            raise KeyError(val)
        eventtime = self.printer.get_reactor().monotonic()
        return po.get_status(eventtime)

    def __getattr__(self, val):
        return self.__getitem__(val)

    def __contains__(self, val):
        try:
            self.__getitem__(val)
        except KeyError as e:
            return False
        return True

    def __iter__(self):
        for name, obj in self.printer.lookup_objects():
            if hasattr(obj, "get_status"):
                yield name

    def get(self, key: str, default: configfile.sentinel):
        try:
            return self[key]
        except KeyError:
            if default is not configfile.sentinel:
                return default
            raise
