from enum import Enum
from typing import TYPE_CHECKING


if TYPE_CHECKING:
    from ..toolhead import ToolHead
    from ..gcode import GCodeCommand
    from .homing import PrinterHoming


class States(Enum):
    UNKNOWN = 0
    INITIAL_UNLOAD = 1
    AWAITING_THUMSCREW_REMOVAL = 2
    AWAITING_TUBE_REMOVAL = 3
    UNLOADED = 4
    INITIAL_LOAD = 5
    AWAITING_CAP = 6
    AWAITING_CORE = 7
    LOADED = 8


LOADING_STATES = [
    States.INITIAL_LOAD,
    States.AWAITING_CAP,
    States.AWAITING_CORE,
]
UNLOADING_STATES = [
    States.INITIAL_UNLOAD,
    States.AWAITING_THUMSCREW_REMOVAL,
    States.AWAITING_TUBE_REMOVAL,
]


class CocoaPress:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()

        # register event handlers
        self.printer.register_event_handler(
            "klippy:connect", self.handle_connect
        )
        self.printer.register_event_handler(
            "klippy:mcu_identify", self._handle_config
        )
        self.printer.register_event_handler("klippy:ready", self.handle_ready)

        self.gcode = self.printer.lookup_object("gcode")
        self.toolhead: ToolHead = None
        self.move_speed = config.getfloat("load_speed", 15.0, above=0.0)
        self.load_retract_distance = 150  # mm
        self.load_nozzle_push_distance = 10  # mm

        self.homing_position = (0, 0, 0, 300)
        self.homing_speed = config.getfloat("homing_speed", 15.0, above=0.0)

        self.state = States.UNKNOWN
        # register commands
        self.gcode.register_command(
            "LOAD_COCOAPRESS",
            self.cmd_LOAD_COCOAPRESS,
        )
        self.gcode.register_command(
            "UNLOAD_COCOAPRESS",
            self.cmd_UNLOAD_COCOAPRESS,
        )

        self.endstop_pin = config.get("endstop_pin", None)
        ppins = self.printer.lookup_object("pins")
        self.mcu_endstop = ppins.setup_pin("endstop", self.endstop_pin)
        query_endstops = self.printer.load_object(config, "query_endstops")
        query_endstops.register_endstop(self.mcu_endstop, "extruder")

    def _handle_config(self):
        self.toolhead = self.printer.lookup_object("toolhead")

        extruder = self.toolhead.extruder
        extruder_stepper = extruder.extruder_stepper.stepper
        # self.mcu_endstop._build_config()
        self.mcu_endstop.add_stepper(extruder_stepper)

    def handle_connect(self):
        pass

    def handle_ready(self):
        pass

    def handle_enable(self):
        pass

    def cmd_LOAD_COCOAPRESS(self, gcmd: "GCodeCommand"):
        """
        * if unloaded or unknown:
            * move plunger forward to stall
            * prompt user to put cap on the plunger
            * retract plunger until stall, move forward a set amount (to over halfway)
            * prompt user to install core
            * move plunger forward to stall
            * tell user ready to preheat!
        * else:
            * tell user already loaded!
        """
        # if self.state == States.UNKNOWN or self.state == States.UNLOADED:
        #     gcmd.respond_info("Please unload before trying to load!")
        #     return
        self.state = States.INITIAL_LOAD
        self.continue_load(gcmd)

    def continue_load(self, gcmd):
        if self.state == States.INITIAL_LOAD:
            self.home_extruder_to_bottom()
            self.state = States.AWAITING_CAP
            self._register_commands()
            self.gcode.respond_info(
                "Please put the cap on the plunger and run CONTINUE"
            )
        elif self.state == States.AWAITING_CAP:
            self.move_extruder(-self.load_retract_distance, self.move_speed)
            self.state = States.AWAITING_CORE
            self._register_commands()
            self.gcode.respond_info("Please install the core and run CONTINUE")
        elif self.state == States.AWAITING_CORE:
            self.home_extruder_to_bottom()
            self.state = States.LOADED
            self._unregister_commands()
            self.gcode.respond_info("Ready to preheat!")

    def cmd_UNLOAD_COCOAPRESS(self, gcmd):
        """
        * if loaded or unknown:
            * prompt user to remove the thumbscrew
            * push extruder forward slightly (make easier to grab)
            * prompt user to remove tube
        * else:
            * tell user already unloaded!
        """
        if self.state == States.UNLOADED:
            gcmd.respond_info("Already unloaded!")
            return
        self.state = States.INITIAL_UNLOAD
        self.continue_unload(gcmd)

    def continue_unload(self, gcmd):
        if self.state == States.INITIAL_UNLOAD:
            self.state = States.AWAITING_THUMSCREW_REMOVAL
            self._register_commands()
            gcmd.respond_info("Please remove the thumbscrew and run CONTINUE")
        elif self.state == States.AWAITING_THUMSCREW_REMOVAL:
            self.move_extruder(self.load_nozzle_push_distance, self.move_speed)
            self.state = States.AWAITING_TUBE_REMOVAL
            self._register_commands()
            gcmd.respond_info("Please remove the tube and run CONTINUE")
        elif self.state == States.AWAITING_TUBE_REMOVAL:
            self.state = States.UNLOADED
            self._unregister_commands()
            gcmd.respond_info("Ready to load!")

    def home_extruder_to_bottom(self):
        # hmove: HomingMove = self.printer.lookup_object("homing_move")
        # self.printer.send_event("homing:homing_move_begin", hmove)
        # self.toolhead.flush_step_generation()
        # kin = self.toolhead.get_kinematics()
        # print_time = self.toolhead.get_last_move_time()
        phoming: PrinterHoming = self.printer.lookup_object("homing")
        phoming.manual_home(
            self.toolhead,
            [(self.mcu_endstop, "extruder")],
            self.homing_position,
            self.homing_speed,
            True,
            True,
        )

    def move_extruder(self, amount, speed):
        last_pos = self.toolhead.get_position()
        new_pos = (last_pos[0], last_pos[1], last_pos[2], last_pos[3] + amount)
        self.toolhead.manual_move(new_pos, speed)

    def cmd_CONTINUE(self, gcmd):
        self._unregister_commands()
        if self.state in LOADING_STATES:
            self.continue_load(gcmd)
        elif self.state in UNLOADING_STATES:
            self.continue_unload(gcmd)

    def cmd_ABORT(self, gcmd):
        self._unregister_commands()
        self._abort()

    def _abort(self):
        self.state = States.UNKNOWN

    def _register_commands(self):
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


def load_config(config):
    return CocoaPress(config)
