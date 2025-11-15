import collections
import typing

from klippy.extras.fan import PrinterFan as PrinterFan
from klippy.extras.fan_generic import PrinterFanGeneric as PrinterFanGeneric
from klippy.extras.gcode_macro import GCodeMacro as GCodeMacro
from klippy.extras.gcode_move import GCodeMove as GCodeMove
from klippy.extras.heaters import PrinterHeaters as PrinterHeaters
from klippy.extras.save_variables import SaveVariables as SaveVariables
from klippy.gcode import GCodeDispatch as GCodeDispatch

BlockingResult = typing.TypeVar("BlockingResult")

class GCodeAPI:
    def __getattr__(self, command: str) -> GCodeCommand: ...
    def __call__(self, command: str): ...
    def absolute_movement(self): ...
    def relative_movement(self): ...
    def absolute_extrusion(self): ...
    def relative_extrusion(self): ...
    def display(self, msg) -> None:
        "M117 {msg}"

class GCodeCommand:
    def format(self, *args, **params): ...
    def __call__(self, *args: str, **params): ...

class HeatersAPI:
    def set_temperature(self, heater_name: str, temp: float | None = None):
        """Set the target temperature for a heater"""
    def temperature_wait(
        self, sensor_name, min_temp: float = ..., max_temp: float = ...
    ):
        """
        Wait for a heater or sensor to reach a temperature

        If no minimum or maximum is given, this will wait for the heater's control loop to settle
        """

class FanAPI:
    def set_speed(self, fan_name: str, speed: float):
        """Set the speed of a fan"""

class MoveAPI:
    def __call__(
        self,
        x: float | None = None,
        y: float | None = None,
        z: float | None = None,
        e: float | None = None,
        *,
        dx: float = 0.0,
        dy: float = 0.0,
        dz: float = 0.0,
        de: float = 0.0,
        speed: float | None = None,
    ):
        """
        Move to a position

        `speed` is in mm/s and unlike `G1 Fx` only affects this movement.
        """
    def set_gcode_offset(
        self,
        x: float | None = None,
        y: float | None = None,
        z: float | None = None,
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
    def set_speed(self, speed: float):
        """Set the speed for future moves in mm/s"""
    def set_speed_factor(self, speed_factor: float = 1.0):
        """Set the movement speed multiplier"""
    def set_extrude_factor(self, extrude_factor: float = 1.0):
        """Set the extrusion multiplier"""

class Kalico:
    """The magic "Printer" object for macros"""

    status: GetStatusWrapperPython
    saved_vars: SaveVariablesWrapper

    fans: FanAPI
    gcode: GCodeAPI
    heaters: HeatersAPI
    move: MoveAPI

    def wait_while(self, condition: typing.Callable[[], bool]):
        """Wait while a condition is True"""
    def wait_until(self, condition: typing.Callable[[], bool]):
        """Wait until a condition is True"""
    def wait_moves(self) -> None:
        """Wait until all moves are completed"""
    def blocking(
        self, function: typing.Callable[[], BlockingResult]
    ) -> BlockingResult:
        """Run a blocking task in a thread, waiting for the result"""
    def sleep(self, timeout: float):
        """Wait a given number of seconds"""
    def set_gcode_variable(self, macro: str, variable: str, value: typing.Any):
        """Save a variable to a gcode_macro"""
    def emergency_stop(self, msg: str = "action_emergency_stop"):
        """Immediately shutdown Kalico"""
    def respond(self, prefix: str, msg: str):
        """Send a message to the console"""
    def respond_info(self, msg: str):
        """Send a message to the console"""
    def respond_raw(self, msg: str): ...
    def raise_error(self, msg) -> None:
        """Raise a G-Code command error"""
    def call_remote_method(self, method: str, **kwargs):
        """Call a Kalico webhooks method"""

class TemplateVariableWrapperPython:
    def __setitem__(self, name, value) -> None: ...
    def __getitem__(self, name): ...
    def __contains__(self, val) -> bool: ...
    def __iter__(self): ...
    def items(self): ...

class SaveVariablesWrapper:
    def __getitem__(self, name): ...
    def __setitem__(self, name, value) -> None: ...
    def __contains__(self, name) -> bool: ...
    def __iter__(self): ...
    def items(self): ...

class GetStatusWrapperPython:
    def __getitem__(self, val) -> StatusWrapper: ...
    def __getattr__(self, val) -> StatusWrapper: ...
    def __contains__(self, val) -> bool: ...
    def __iter__(self): ...
    def get(self, key: str, default: typing.Any = ...) -> StatusWrapper: ...

class StatusWrapper(collections.UserDict):
    def __getattr__(self, name): ...

MacroParams = typing.ParamSpec("MacroParams")
MacroReturn = typing.TypeVar("MacroReturn")
MacroFunction = typing.Callable[
    typing.Concatenate[Kalico, MacroParams], MacroReturn
]

class Macro(typing.Protocol, typing.Generic[MacroParams, MacroReturn]):
    rawparams: str
    params: dict[str, str]
    vars: TemplateVariableWrapperPython

    @staticmethod
    def __call__(
        kalico: Kalico, *args: MacroParams.args, **kwargs: MacroParams.kwargs
    ) -> MacroReturn: ...
    @staticmethod
    def delay(
        delay: float, /, *args: MacroParams.args, **kwargs: MacroParams.kwargs
    ):
        "Schedule this function to run after a delay"

    @staticmethod
    def every(
        period: float, /, *args: MacroParams.args, **kwargs: MacroParams.kwargs
    ):
        "Schedule this function to run every `period` seconds"

@typing.overload
def gcode_macro(
    function: typing.Callable[
        typing.Concatenate[Kalico, MacroParams], MacroReturn
    ],
    /,
) -> Macro[MacroParams, MacroReturn]: ...
@typing.overload
def gcode_macro(
    *, rename_existing: str
) -> typing.Callable[
    [typing.Callable[typing.Concatenate[Kalico, MacroParams], MacroReturn]],
    Macro[MacroParams, MacroReturn],
]: ...

__all__ = ("gcode_macro", "Kalico")
