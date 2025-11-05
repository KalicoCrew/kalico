"""
Klipper plugin to monitor toolhead adc values
to detect the toolhead's attachment status
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...configfile import ConfigWrapper
    from ...gcode import GCodeCommand, GCodeDispatch
    from ...printer import Printer
    from ...reactor import SelectReactor
    from ...toolhead import ToolHead
    from ..gcode_macro import PrinterGCodeMacro, Template
    from ..gcode_move import GCodeMove
    from ..pause_resume import PauseResume
    from .toolhead import CocoaToolheadControl


def volume_cylinder(r, h):
    return math.pi * (r**2) * h


# Some constants we can use to estimate chocolate volume
NOMINAL_CORE_LENGTH = 145.0
NOMINAL_CORE_RADIUS = 22.65 / 2.0
NOMINAL_CORE_VOLUME = volume_cylinder(NOMINAL_CORE_RADIUS, NOMINAL_CORE_LENGTH)

# Estimated max positions
POSITION_BOTTOM_UNLOADED = 197.0  # Without a cartridge
POSITION_EMPTY = 164.0  # Empty cartridge
# New full length core
POSITION_FULL = POSITION_EMPTY - NOMINAL_CORE_LENGTH


class CocoaRunout:
    config: ConfigWrapper
    printer: Printer
    reactor: SelectReactor
    gcode: GCodeDispatch
    gcode_move: GCodeMove
    gcode_macro: PrinterGCodeMacro
    pause_resume: PauseResume
    toolhead: ToolHead

    runout_enabled: bool

    def __init__(
        self, cocoa_toolhead: CocoaToolheadControl, config: ConfigWrapper
    ):
        self.cocoa_toolhead = cocoa_toolhead
        self.name = cocoa_toolhead.name
        self.mux_name = cocoa_toolhead.mux_name
        self.printer = config.get_printer()
        self.reactor: SelectReactor = self.printer.get_reactor()
        self.logger = cocoa_toolhead.logger.getChild("runout")

        self.gcode = self.printer.lookup_object("gcode")
        self.gcode_move = self.printer.load_object(config, "gcode_move")
        self.gcode_macro = self.printer.load_object(config, "gcode_macro")
        self.pause_resume = self.printer.load_object(config, "pause_resume")

        self.toolhead = None

        self.enable_runout = config.getboolean("runout_enabled", True)
        self.runout_tmpl: Template = self.gcode_macro.load_template(
            config,
            "runout_gcode",
            "M600",
        )

        self.runout_enabled = False
        self.position_top = None
        self.position_empty = None
        self.position_full = None

        self.printer.register_event_handler("klippy:connect", self._on_connect)
        self.gcode.register_mux_command(
            "SET_FILAMENT_SENSOR",
            "SENSOR",
            self.mux_name,
            self.cmd_SET_FILAMENT_SENSOR,
        )

        for event in [
            "print_stats:paused_printing",
            "print_stats:complete_printing",
            "print_stats:error_printing",
            "print_stats:cancelled_printing",
        ]:
            self.printer.register_event_handler(event, self._disable_runout)

        if self.enable_runout:
            self.printer.register_event_handler(
                "print_stats:start_printing", self._enable_runout
            )

    def _on_connect(self):
        self.toolhead = self.printer.lookup_object("toolhead")
        self.next_transform = self.gcode_move.set_move_transform(self, True)
        self.get_position = self.next_transform.get_position

    def set_top(self, extruder_position: float):
        self.position_top = extruder_position
        self.position_empty = POSITION_EMPTY + self.position_top
        self.position_full = POSITION_FULL + self.position_top

    def _enable_runout(self):
        if self.position_top is None:
            raise self.gcode.error(
                f"cocoa_toolhead[{self.name}]: cannot enable runout without homing the toolhead"
            )

        self.runout_enabled = True
        self.logger.info(f"cocoa_toolhead[{self.name}]: runout enabled")

    def _disable_runout(self):
        self.runout_enabled = False

    # Move transformation methods
    def get_position(self):
        return self.next_transform.get_position()

    def move(self, newpos, speed):
        if self.runout_enabled and newpos[3] > self.position_empty:
            self._trigger_runout()

        return self.next_transform.move(newpos, speed)

    def _trigger_runout(self):
        runouttime = self.toolhead.get_last_move_time()
        self.reactor.register_callback(self._runout_callback, runouttime)

        self.pause_resume.send_pause_command()
        self._disable_runout()

    def _runout_callback(self, eventtime):
        ctx = self.gcode_macro.create_template_context()
        self.runout_tmpl.run_gcode_from_command(ctx)

        self.gcode.respond_info("action:runout")
        self.printer.send_event("filament:runout", eventtime, self.name)

        return self.reactor.NEVER

    def cmd_SET_FILAMENT_SENSOR(self, gcmd: GCodeCommand):
        "Enable or disable the chocolate runout sensor"
        enable = gcmd.get_int("ENABLE", None, minval=0, maxval=1)
        if enable == 1:
            self._enable_runout()
        if enable == 0:
            self._disable_runout()

    def calculate_length(self, raw_position: float) -> float:
        position = raw_position - self.position_top
        return POSITION_EMPTY - max(POSITION_FULL, position)

    def get_status(self, eventtime):
        status = {
            "pause_on_runout": self.runout_enabled,
            "position_top": self.position_top,
            "position_empty": self.position_empty,
            "position_full": self.position_full,
            "position": 0.0,
            "remaining": 0.0,
            "remaining_length": 0.0,
            "remaining_volume": 0.0,
        }

        if self.toolhead and self.position_top is not None:
            extruder_position = self.toolhead.get_position()[3]
            position = extruder_position - self.position_top

            current_length = POSITION_EMPTY - max(POSITION_FULL, position)
            remaining = (current_length) / NOMINAL_CORE_LENGTH * 100.0

            status["position"] = position
            status["remaining"] = remaining
            status["remaining_length"] = current_length
            status["remaining_volume"] = volume_cylinder(
                NOMINAL_CORE_RADIUS,
                current_length,
            )

        return status
