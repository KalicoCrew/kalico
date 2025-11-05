from __future__ import annotations

from enum import Enum
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...configfile import ConfigWrapper
    from ...gcode import GCodeCommand, GCodeDispatch
    from ...printer import Printer
    from .toolhead import CocoaToolheadControl


class States(Enum):
    """
    * home to top
    * home to bottom
    * if distance < empty_tube_travel_distance_cutoff
        * cartridge is installed
        * prompt user to remove screw
        * push extruder ~10mm
        * prompt user to remove toolhead + remove core, reinstall toolhead
    * else:
        * no cartridge installed
        * skip to the prompt to install red cap

    * wait for toolhead to be reinstalled
    * home to bottom
    * prompt user to install red cap
    * move to safe load height
    * prompt user to remove toolhead, install core, reinstall toolhead
    * wait for toolhead to be reinstalled
    * home to chocolate
    """

    ABORTED = -1
    UNKNOWN = 0
    INITIAL_UNLOAD = 1
    AWAITING_THUMBSCREW_REMOVAL = 2
    AWAITING_TOOLHEAD_DETACH_UNLOAD = 3
    UNLOADED = 4
    UNLOADED_READY_FOR_CAP = 5
    INITIAL_LOAD = 6
    AWAITING_TOOLHEAD_ATTACH_INITIAL_LOAD = 7
    AWAITING_PLUNGER_CAP_INSTALL = 8
    AWAITING_TOOLHEAD_DETACH_CORE_LOAD = 9
    AWAITING_TOOLHEAD_ATTACH_CORE_LOAD = 10
    LOADED = 11


ATTACH_LISTEN_STATES = [
    States.AWAITING_TOOLHEAD_ATTACH_INITIAL_LOAD,
    States.AWAITING_TOOLHEAD_ATTACH_CORE_LOAD,
]
DETACH_LISTEN_STATES = [
    States.AWAITING_TOOLHEAD_DETACH_UNLOAD,
    States.AWAITING_TOOLHEAD_DETACH_CORE_LOAD,
]


class CocoaLoadWizard:
    # Constants for homing
    load_retract_distance = 150  # mm
    load_nozzle_push_distance = 10  # mm
    empty_tube_travel_distance_cutoff = 180  # mm

    printer: Printer
    config: ConfigWrapper
    gcode: GCodeDispatch

    def __init__(
        self, cocoa_toolhead: CocoaToolheadControl, config: ConfigWrapper
    ):
        self.printer = config.get_printer()
        self.cocoa_toolhead = cocoa_toolhead
        self.name = cocoa_toolhead.name
        self.mux_name = cocoa_toolhead.mux_name
        self.toolhead = self.cocoa_toolhead.toolhead

        self.gcode = self.printer.lookup_object("gcode")

        self.load_move_speed = config.getfloat(
            "load_move_speed", 25.0, above=0.0
        )

        self.load_unload_message = None
        self.state = States.UNKNOWN
        self.state_pre_abort = None

        self.gcode.register_mux_command(
            "LOAD_COCOAPRESS",
            "TOOL",
            self.mux_name,
            self.cmd_LOAD_COCOAPRESS,
        )
        self.gcode.register_mux_command(
            "UNLOAD_COCOAPRESS",
            "TOOL",
            self.mux_name,
            self.cmd_UNLOAD_COCOAPRESS,
        )

        self.printer.register_event_handler(
            "cocoa_toolhead:attached",
            self._load_hook_for_toolhead_attach_detach,
        )
        self.printer.register_event_handler(
            "cocoa_toolhead:detached",
            self._load_hook_for_toolhead_attach_detach,
        )

    def cmd_LOAD_COCOAPRESS(self, gcmd):
        # if self.state == States.UNKNOWN or self.state == States.UNLOADED:
        #     gcmd.respond_info("Please unload before trying to load!")
        #     return
        if self.state != States.UNLOADED_READY_FOR_CAP:
            self.state = States.INITIAL_LOAD
        self.continue_load_unload()

    def cmd_UNLOAD_COCOAPRESS(self, gcmd: GCodeCommand):
        if self.state == States.UNLOADED:
            gcmd.respond_info("Already unloaded!")
            return
        self.state = States.INITIAL_UNLOAD
        self.continue_load_unload()

    def cmd_CONTINUE(self, gcmd):
        self._unregister_commands()
        self.continue_load_unload()

    def cmd_ABORT(self, gcmd: GCodeCommand):
        self._unregister_commands()
        self._abort()

    def _abort(self):
        self.state_pre_abort = self.state
        self.state = States.ABORTED

    def _register_commands(self):
        self._unregister_commands()
        self.gcode.register_command(
            "CONTINUE",
            self.cmd_CONTINUE,
        )
        self.gcode.register_command(
            "ABORT",
            self.cmd_ABORT,
        )

    def _unregister_commands(self):
        self.gcode.register_command(
            "ABORT",
            None,
        )
        self.gcode.register_command(
            "CONTINUE",
            None,
        )

    def _load_hook_for_toolhead_attach_detach(self, name):
        if (
            self.state in ATTACH_LISTEN_STATES
            or self.state in DETACH_LISTEN_STATES
        ):
            if self.name != name:
                return
            self.continue_load_unload()

    def _print_message__load_unload(self, message, error=False):
        if error:
            self.gcode._respond_error(message)
        else:
            self.gcode.respond_info(message)
        self.load_unload_message = message

    def continue_load_unload(self):
        if self.state == States.UNKNOWN or self.state == States.ABORTED:
            raise self._print_message__load_unload("Unknown state!", error=True)

        elif self.state == States.INITIAL_UNLOAD:
            self.cocoa_toolhead.home_extruder_to_top()
            homed_dist = self.cocoa_toolhead.home_extruder_to_bottom()
            if homed_dist > self.empty_tube_travel_distance_cutoff:
                # no tube installed, skip to prompt for cap
                self.state = States.UNLOADED_READY_FOR_CAP
                self._print_message__load_unload("Ready for load!")
                self._unregister_commands()
            else:
                self.state = States.AWAITING_THUMBSCREW_REMOVAL
                self._register_commands()
                self._print_message__load_unload(
                    "Please remove the thumbscrew and run CONTINUE"
                )

        elif self.state == States.AWAITING_THUMBSCREW_REMOVAL:
            self.move_extruder(
                self.load_nozzle_push_distance, self.load_move_speed
            )
            self._print_message__load_unload("Please remove the toolhead!")
            self.state = States.AWAITING_TOOLHEAD_DETACH_UNLOAD
            self._register_commands()

        elif self.state == States.AWAITING_TOOLHEAD_DETACH_UNLOAD:
            self._print_message__load_unload("Ready to load!")
            self.state = States.UNLOADED
            self._unregister_commands()

        elif self.state == States.INITIAL_LOAD:
            self._print_message__load_unload(
                "Reinstall toolhead with cartridge removed!"
            )
            self.state = States.AWAITING_TOOLHEAD_ATTACH_INITIAL_LOAD
            self._register_commands()

        elif self.state == States.UNLOADED_READY_FOR_CAP:
            self._print_message__load_unload(
                "Install red cap on plunger and run CONTINUE"
            )
            self.state = States.AWAITING_PLUNGER_CAP_INSTALL
            self._register_commands()

        elif self.state == States.AWAITING_TOOLHEAD_ATTACH_INITIAL_LOAD:
            self.home_extruder_to_bottom()
            self._print_message__load_unload(
                "Install red cap on plunger and run CONTINUE"
            )
            self.state = States.AWAITING_PLUNGER_CAP_INSTALL
            self._register_commands()

        elif self.state == States.AWAITING_PLUNGER_CAP_INSTALL:
            self.move_extruder(
                -self.load_retract_distance, self.load_move_speed
            )
            self._print_message__load_unload("Remove toolhead!")
            self.state = States.AWAITING_TOOLHEAD_DETACH_CORE_LOAD
            self._register_commands()

        elif self.state == States.AWAITING_TOOLHEAD_DETACH_CORE_LOAD:
            self._print_message__load_unload(
                "Load chocolate core into cartridge and reinstall toolhead!"
            )
            self.state = States.AWAITING_TOOLHEAD_ATTACH_CORE_LOAD
            self._register_commands()

        elif self.state == States.AWAITING_TOOLHEAD_ATTACH_CORE_LOAD:
            self.home_extruder_to_bottom()
            self._print_message__load_unload("Done loading, ready for preheat!")
            self.state = States.LOADED
            self._unregister_commands()

    def move_extruder(self, amount, speed):
        last_pos = self.toolhead.get_position()
        new_pos = (last_pos[0], last_pos[1], last_pos[2], last_pos[3] + amount)
        self.toolhead.manual_move(new_pos, speed)

    def get_state(self, eventtime):
        return {
            "load_state": self.state.name,
            "load_unload_message": self.load_unload_message,
        }
