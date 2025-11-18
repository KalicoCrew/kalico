import contextlib
import collections
import typing

from klippy.extras.display.menu import MenuManager
from klippy.extras.homing import Homing, HomingMove
from klippy.extras.load_cell import LoadCell
from klippy.stepper import (
    MCU_stepper,
    PrinterRail,
)

BlockingResult = typing.TypeVar("BlockingResult")

class GCodeAPI:
    def __getattr__(self, command: str) -> GCodeCommand: ...
    def __call__(self, command: str): ...
    def absolute_movement(self) -> None: ...
    def relative_movement(self) -> None: ...
    def absolute_extrusion(self) -> None: ...
    def relative_extrusion(self) -> None: ...
    def display(self, msg: str): ...

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
        """Set the movement speed factor"""
    def set_extrude_factor(self, extrude_factor: float = 1.0): ...
    @contextlib.contextmanager
    def save_state(self, restore_position: bool = False, speed: float = None):
        """
        Save the current gcode state
        """

class TimerCallback(typing.Protocol):
    def __call__(self, kalico: Kalico, eventtime: float):
        """Callback for timers or intervals"""

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
    def get_gcode_variables(
        self, macro: str
    ) -> TemplateVariableWrapperPython: ...
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
    def timer(self, delay: float, callback: TimerCallback) -> Timer:
        """Schedule a callback to run after a delay"""
    def interval(self, period: float, callback: TimerCallback) -> Interval: ...

class Interval:
    def cancel(self) -> None: ...
    @property
    def is_pending(self): ...
    @property
    def next_waketime(self): ...

class Timer(Interval): ...

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

class Macro(typing.Generic[MacroParams, MacroReturn]):
    name: str
    @property
    def vars(self) -> TemplateVariableWrapperPython: ...
    @property
    def raw_params(self) -> str: ...
    @property
    def params(self) -> dict[str, str]: ...
    def __call__(
        self,
        kalico: Kalico,
        *args: MacroParams.args,
        **kwargs: MacroParams.kwargs,
    ) -> MacroReturn: ...

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

Handler = typing.TypeVar("Handler", bound=typing.Callable)

class Decorator(typing.Protocol, typing.Generic[Handler]):
    def __call__(self, handler: Handler) -> Handler: ...

class ToolheadSyncEventHandler(typing.Protocol):
    def __call__(
        self,
        kalico: Kalico,
        eventtime: float,
        est_print_time: float,
        print_time: float,
    ): ...

class FilamentRunoutEventHandler(typing.Protocol):
    def __call__(self, kalico: Kalico, eventtime: float, sensor_name: str): ...

class HomingEventHandler(typing.Protocol):
    def __call__(
        self, kalico: Kalico, homing: Homing, rails: list[PrinterRail]
    ): ...

class HomingMoveEventHandler(typing.Protocol):
    def __call__(self, kalico: Kalico, homing_move: HomingMove): ...

class LoadCellEventHandler(typing.Protocol):
    def __call__(self, kalico: Kalico, load_cell: LoadCell): ...

class MenuEventHandler(typing.Protocol):
    def __call__(self, kalico: Kalico, menu: MenuManager): ...

class StepperEventHandler(typing.Protocol):
    def __call__(self, kalico: Kalico, mcu_stepper: MCU_stepper): ...

class EventTimeEventHandler(typing.Protocol):
    def __call__(self, kalico: Kalico, eventtime: float): ...

class GCodeUnknownCommandHandler(typing.Protocol):
    def __call__(self, kalico: Kalico, command: str): ...

class NonCriticalEventHandler(typing.Protocol):
    def __call__(self, kalico: Kalico, mcu_name: str): ...

class EventHandler(typing.Protocol):
    def __call__(self, kalico: Kalico): ...

@typing.overload
def event_handler(
    event: typing.Literal["load_cell:calibrate", "load_cell:tare"],
) -> Decorator[LoadCellEventHandler]: ...
@typing.overload
def event_handler(
    event: typing.Literal["menu:begin", "menu:exit"],
) -> Decorator[MenuEventHandler]: ...
@typing.overload
def event_handler(
    event: typing.Literal[
        "stepper:set_dir_inverted", "stepper:sync_mcu_position"
    ],
) -> Decorator[StepperEventHandler]: ...
@typing.overload
def event_handler(
    event: typing.Literal["toolhead:sync_print_time"],
) -> Decorator[ToolheadSyncEventHandler]: ...
@typing.overload
def event_handler(
    event: typing.Literal["filament:insert", "filament:runout"],
) -> Decorator[FilamentRunoutEventHandler]: ...
@typing.overload
def event_handler(
    event: typing.Literal["homing:home_rails_begin", "homing:home_rails_end"],
) -> Decorator[HomingEventHandler]: ...
@typing.overload
def event_handler(
    event: typing.Literal["homing:homing_move_begin", "homing:homing_move_end"],
) -> Decorator[HomingMoveEventHandler]: ...
@typing.overload
def event_handler(
    event: typing.Literal[
        "gcode:request_restart",
        "idle_timeout:idle",
        "idle_timeout:printing",
        "idle_timeout:ready",
        "stepper_enable:motor_off",
    ],
) -> Decorator[EventTimeEventHandler]: ...
@typing.overload
def event_handler(
    event: typing.Literal["gcode:unknown_command"],
) -> Decorator[GCodeUnknownCommandHandler]: ...
@typing.overload
def event_handler(
    event: typing.Literal[
        "danger:non_critical_mcu:disconnected",
        "danger:non_critical_mcu:reconnected",
    ],
) -> Decorator[NonCriticalEventHandler]: ...
@typing.overload
def event_handler(
    event: typing.Literal[
        "extruder:activate_extruder",
        "gcode:command_error",
        "klippy:connect",
        "klippy:disconnect",
        "klippy:firmware_restart",
        "klippy:mcu_identify",
        "klippy:ready",
        "klippy:shutdown",
        "print_stats:cancelled_printing",
        "print_stats:complete_printing",
        "print_stats:error_printing",
        "print_stats:paused_printing",
        "print_stats:reset",
        "print_stats:start_printing",
        "toolhead:manual_move",
        "toolhead:set_position",
        "trad_rack:forced_active_lane",
        "trad_rack:load_complete",
        "trad_rack:load_started",
        "trad_rack:reset_active_lane",
        "trad_rack:synced_to_extruder",
        "trad_rack:unload_complete",
        "trad_rack:unload_started",
        "trad_rack:unsyncing_from_extruder",
        "virtual_sdcard:load_file",
        "virtual_sdcard:reset_file",
    ],
) -> typing.Callable[[EventHandler], EventHandler]: ...

Number = typing.TypeVar("Number", int, float)

class Above(typing.Generic[Number]):
    def __init__(self, above: Number) -> None: ...

class Below(typing.Generic[Number]):
    def __init__(self, below: Number) -> None: ...

class Minimum(typing.Generic[Number]):
    def __init__(self, minimum: Number) -> None: ...

class Maximum(typing.Generic[Number]):
    def __init__(self, maximum: Number) -> None: ...

class Range(typing.Generic[Number]):
    def __init__(self, lower: Number, upper: Number) -> None: ...

class Between(typing.Generic[Number]):
    def __init__(self, above: Number, below: Number) -> None: ...

class IntRange:
    def __class_getitem__(cls, range: tuple[int, int]): ...

class IntBetween:
    def __class_getitem__(cls, range: tuple[int, int]): ...

class FloatRange:
    def __class_getitem__(cls, range: tuple[int, int]): ...

class FloatBetween:
    def __class_getitem__(cls, range: tuple[int, int]): ...

__all__ = [
    "gcode_macro",
    "event_handler",
    "Kalico",
    "Timer",
    "Interval",
    "Above",
    "Below",
    "Between",
    "FloatBetween",
    "FloatRange",
    "IntBetween",
    "IntRange",
    "Maximum",
    "Minimum",
    "Range",
]
