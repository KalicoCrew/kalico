from __future__ import annotations

import functools
import typing

from .context import Kalico
from .loader import load_context

if typing.TYPE_CHECKING:
    from klippy.extras.display.menu import MenuManager
    from klippy.extras.homing import Homing, HomingMove
    from klippy.extras.load_cell import LoadCell
    from klippy.stepper import MCU_stepper, PrinterRail

    from .types import Decorator


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


def event_handler(event):
    def decorator(handler):
        kalico = load_context.loader.kalico

        @functools.wraps(handler)
        def wrapped_handler(*args):
            handler(kalico, *args)

        load_context.loader.printer.register_event_handler(
            event, wrapped_handler
        )

    return decorator
