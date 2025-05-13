"""
Klipper plugin to monitor toolhead adc values
to detect the toolhead's attachment status
"""

from enum import Enum
import logging

from typing import TYPE_CHECKING, Literal
from klippy import chelper
from klippy.extras.homing import Homing

if TYPE_CHECKING:
    from .stepper_enable import EnableTracking
    from ..toolhead import ToolHead
    from ..gcode import GCodeDispatch
    from .homing import PrinterHoming
    from ..stepper import MCU_stepper
    from ..kinematics.extruder import PrinterExtruder


logger = logging.getLogger(__name__)

DIRECTION_TOP = -1
DIRECTION_BOTTOM = 1


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


class FakeExtruderHomingToolhead:
    def __init__(self, toolhead, extruder_stepper: "MCU_stepper"):
        self.toolhead: ToolHead = toolhead
        self.extruder_stepper = extruder_stepper

    def get_position(self):
        return self.toolhead.get_position()

    def set_position(self, pos, homing_axes=()):
        _ffi_main, ffi_lib = chelper.get_ffi()
        logging.info(f"setting position to {pos}, homing_axes={homing_axes}")
        self.toolhead.set_position(pos, homing_axes=homing_axes)
        ffi_lib.trapq_set_position(
            self.extruder_stepper._trapq,
            self.toolhead.print_time,
            pos[3],
            0.0,
            0.0,
        )
        self.extruder_stepper.set_position([pos[3], 0.0, 0.0])

    def get_last_move_time(self):
        return self.toolhead.get_last_move_time()

    def dwell(self, time):
        self.toolhead.dwell(time)

    def drip_move(self, dist, speed, drip_completion):
        self.toolhead.drip_move(dist, speed, drip_completion)

    def flush_step_generation(self):
        self.toolhead.flush_step_generation()

    # fake kinematics interface
    def get_kinematics(self):
        return self

    def calc_position(self, stepper_positions):
        logging.info(f"calc_position: {stepper_positions}")
        base_res = self.toolhead.get_kinematics().calc_position(
            stepper_positions
        )
        # add extruder position
        extruder_position = stepper_positions[self.extruder_stepper.get_name()]

        extruder_position = round(extruder_position, 6)
        res = base_res + [extruder_position]
        logging.info(f"calc_position result: {res}")
        return res

    def get_steppers(self):
        base_kin_steppers = self.toolhead.get_kinematics().get_steppers()
        return [*base_kin_steppers, self.extruder_stepper]


# Open circuits on the ADC cause a near 1.0 reading, typically above 0.998
OPEN_ADC_VALUE = 0.99

DEFAULT_EXTRUDER = "extruder"
DEFAULT_BODY_HEATER = "heater_generic Body_Heater"


class CocoaToolheadControl:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.config = config

        self.extruder_name = config.get(
            "extruder",
            default=DEFAULT_EXTRUDER,
        )
        self.body_heater_name = config.get(
            "body_heater",
            default=DEFAULT_BODY_HEATER,
        )

        self.attached = None
        self.last_readings = {}
        self.load_unload_message = None
        self.state = States.UNKNOWN
        self.state_pre_abort = None

        self.printer.register_event_handler("klippy:connect", self._on_ready)
        self.printer.register_event_handler(
            "klippy:mcu_identify", self._handle_config
        )

        self.gcode: GCodeDispatch = self.printer.lookup_object("gcode")

        self.toolhead: ToolHead = None
        self.fake_toolhead_for_homing: FakeExtruderHomingToolhead = None
        self.extruder: PrinterExtruder = None
        self.extruder_stepper: MCU_stepper = None
        self.extruder_stepper_enable: EnableTracking = None

        self.load_move_speed = config.getfloat(
            "load_move_speed", 25.0, above=0.0
        )
        self.homing_speed = config.getfloat("homing_speed", 25.0, above=0.0)

        # constants for homing
        self.load_retract_distance = 150  # mm
        self.load_nozzle_push_distance = 10  # mm
        self.total_maximum_homing_dist = 200  # mm
        self.empty_tube_travel_distance_cutoff = 180  # mm

        # register commands
        self.gcode.register_command(
            "LOAD_COCOAPRESS",
            self.cmd_LOAD_COCOAPRESS,
        )
        self.gcode.register_command(
            "UNLOAD_COCOAPRESS",
            self.cmd_UNLOAD_COCOAPRESS,
        )
        self.gcode.register_command(
            "HOME_COCOAPRESS",
            self.cmd_HOME_COCOAPRESS,
        )

        # extruder endstop setup
        self.endstop_pin = config.get("endstop_pin", None)
        ppins = self.printer.lookup_object("pins")
        self.mcu_endstop = ppins.setup_pin("endstop", self.endstop_pin)
        self.endstops = [(self.mcu_endstop, self.extruder_name)]
        query_endstops = self.printer.load_object(config, "query_endstops")
        query_endstops.register_endstop(self.mcu_endstop, self.extruder_name)

        self.gcode_macro = self.printer.load_object(config, "gcode_macro")
        self.attach_tmpl = self.gcode_macro.load_template(
            config, "attach_gcode", ""
        )
        self.detach_tmpl = self.gcode_macro.load_template(
            config, "detach_gcode", ""
        )

    def _on_ready(self):
        logger.info("Initializing Cocoa Toolhead")

        self.attached = None

        extruder = self.printer.lookup_object(self.extruder_name)
        body_heater = self.printer.lookup_object(self.body_heater_name)

        logger.info("Injecting adc callbacks")

        self.inject_adc_callback(extruder.heater)
        self.inject_adc_callback(body_heater)

    def _handle_config(self):
        self.toolhead = self.printer.lookup_object("toolhead")

        self.extruder = self.toolhead.extruder
        self.extruder_stepper = self.extruder.extruder_stepper.stepper
        self.extruder_stepper_enable = self.printer.lookup_object(
            "stepper_enable"
        ).lookup_enable("extruder")
        self.mcu_endstop.add_stepper(self.extruder_stepper)
        self.fake_toolhead_for_homing = FakeExtruderHomingToolhead(
            self.toolhead, self.extruder_stepper
        )

    def inject_adc_callback(self, heater):
        sensor = heater.sensor
        mcu_adc = sensor.mcu_adc

        def new_callback(read_time, read_value):
            if read_value < OPEN_ADC_VALUE:
                sensor.adc_callback(read_time, read_value)
            else:
                heater.set_pwm(read_time, 0.0)
            self.receive_sensor_value(heater, read_value)

        setattr(mcu_adc, "_callback", new_callback)
        logging.info(f"Intercepted ADC callback for {heater.name}")

    def receive_sensor_value(self, heater, value: float):
        self.last_readings[heater.name] = value

        is_attached = value < OPEN_ADC_VALUE
        if is_attached != self.attached:
            self.attached = is_attached

            self.gcode.respond_info(
                f"Cocoa Press: Toolhead {'attached' if is_attached else 'detached'}"
            )

            if is_attached:
                self.printer.send_event("cocoa_toolhead:attached")
                self._run_template(self.attach_tmpl)
                self._load_hook_for_toolhead_attach_detach()
            else:
                self.printer.send_event("cocoa_toolhead:detached")
                self._run_template(self.detach_tmpl)
                self._load_hook_for_toolhead_attach_detach()

    def _load_hook_for_toolhead_attach_detach(self):
        if (
            self.state in ATTACH_LISTEN_STATES
            or self.state in DETACH_LISTEN_STATES
        ):
            self.continue_load_unload()

    def _run_template(self, template):
        ctx = template.create_template_context()
        template.run_gcode_from_command(ctx)

    def cmd_LOAD_COCOAPRESS(self, gcmd):
        # if self.state == States.UNKNOWN or self.state == States.UNLOADED:
        #     gcmd.respond_info("Please unload before trying to load!")
        #     return
        if self.state != States.UNLOADED_READY_FOR_CAP:
            self.state = States.INITIAL_LOAD
        self.continue_load_unload()

    def cmd_UNLOAD_COCOAPRESS(self, gcmd):
        if self.state == States.UNLOADED:
            gcmd.respond_info("Already unloaded!")
            return
        self.state = States.INITIAL_UNLOAD
        self.continue_load_unload()

    def cmd_CONTINUE(self, gcmd):
        self._unregister_commands()
        self.continue_load_unload()

    def cmd_HOME_COCOAPRESS(self, gcmd):
        direction = gcmd.get_int("DIR", 1)
        if direction not in (DIRECTION_BOTTOM, DIRECTION_TOP):
            raise gcmd.error("Invalid direction %s" % (direction,))
        dist_moved = self._home_extruder_in_direction(direction)
        gcmd.respond_info(
            "Homed %s mm in direction %s" % (dist_moved, direction)
        )

    def cmd_ABORT(self, gcmd):
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
            self.home_extruder_to_top()
            homed_dist = self.home_extruder_to_bottom()
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

    def home_extruder_to_top(self) -> float:
        return self._home_extruder_in_direction(DIRECTION_TOP)

    def home_extruder_to_bottom(self) -> float:
        return self._home_extruder_in_direction(DIRECTION_BOTTOM)

    def _home_extruder_in_direction(self, dir: Literal[-1, 1]) -> float:
        if dir == DIRECTION_BOTTOM:
            # Ensure the bed is low enough to home.
            position = self.toolhead.get_position()
            status = self.toolhead.get_status(self.toolhead.print_time)
            if "z" in status.get("homed_axes", "") and position[2] < 50:
                position[2] = 50
                self.toolhead.move(position, 3600)

        self._set_extruder_current_for_homing(pre_homing=True)
        try:
            return self.__home_extruder_in_direction(dir)
        finally:
            self._set_extruder_current_for_homing(pre_homing=False)

    def _set_extruder_current_for_homing(self, pre_homing):
        print_time = self.toolhead.get_last_move_time()
        ch = self.extruder_stepper.get_tmc_current_helper()
        dwell_time = ch.set_current_for_homing(print_time, pre_homing)
        if dwell_time:
            self.toolhead.dwell(dwell_time)

    def __home_extruder_in_direction(self, dir: int) -> float:
        """
        dir should be 1 or -1
        """

        phoming: PrinterHoming = self.printer.lookup_object("homing")
        homing_state = Homing(self.printer)

        homing_distance = dir * self.total_maximum_homing_dist

        curpos = self.toolhead.get_position()
        starting_e_pos = curpos[3]
        ending_e_pos = starting_e_pos
        curpos[3] += homing_distance

        trig_pos = phoming.manual_home(
            toolhead=self.fake_toolhead_for_homing,
            endstops=self.endstops,
            pos=curpos,
            probe_pos=True,
            speed=self.homing_speed,
            triggered=True,
            check_triggered=True,  # raise exception if no trigger on full movement
        )
        homing_state._reset_endstop_states(self.endstops)

        ending_e_pos = trig_pos[3]
        total_homing_distance = round(abs(ending_e_pos - starting_e_pos), 6)

        logging.info("successfully homed!")
        logging.info(f"starting_position: {starting_e_pos}")
        logging.info(f"ending position: {ending_e_pos}")

        self.toolhead.flush_step_generation()
        return total_homing_distance

    def get_status(self, eventtime):
        return {
            "attached": self.attached,
            "adc": self.last_readings,
            "load_state": self.state.name,
            "load_unload_message": self.load_unload_message,
        }


def load_config(config):
    return CocoaToolheadControl(config)
