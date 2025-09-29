"""
Klipper plugin to monitor toolhead adc values
to detect the toolhead's attachment status
"""

from __future__ import annotations

import contextlib
import logging
import math
from enum import Enum
from typing import TYPE_CHECKING, Literal

from klippy import chelper
from klippy.extras.homing import Homing

if TYPE_CHECKING:
    from ..configfile import ConfigWrapper, PrinterConfig
    from ..gcode import GCodeCommand, GCodeDispatch
    from ..kinematics.extruder import PrinterExtruder
    from ..printer import Printer
    from ..reactor import SelectReactor
    from ..stepper import MCU_stepper
    from ..toolhead import ToolHead
    from .gcode_macro import PrinterGCodeMacro, Template
    from .gcode_move import GCodeMove
    from .homing import PrinterHoming
    from .pause_resume import PauseResume
    from .stepper_enable import EnableTracking
    from .tmc2209 import TMC2209


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


def volume_cylinder(r, h):
    return math.pi * (r**2) * h


# Open circuits on the ADC cause a near 1.0 reading, typically above 0.998
OPEN_ADC_VALUE = 0.99

DEFAULT_EXTRUDER = "extruder"
DEFAULT_BODY_HEATER = "heater_generic Body_Heater"

# This may need tweaking
MINIMUM_SAFE_SGTHRS = 50


class CocoaToolheadControl:
    # Constants for homing
    load_retract_distance = 150  # mm
    load_nozzle_push_distance = 10  # mm
    total_maximum_homing_dist = 200  # mm
    empty_tube_travel_distance_cutoff = 180  # mm

    def __init__(self, config: ConfigWrapper):
        self.printer = config.get_printer()
        self.config = config

        self.cocoa_runout = CocoaRunout(config)

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
        self.calibration_required = False

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
        self.extruder_driver: TMC2209 = None

        self.load_move_speed = config.getfloat(
            "load_move_speed", 25.0, above=0.0
        )
        self.homing_speed = config.getfloat("homing_speed", 25.0, above=0.0)

        self.runout_pause = config.getboolean("pause_on_runout", True)

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
        self.gcode.register_command(
            "CALIBRATE_EXTRUDER",
            self.cmd_CALIBRATE_EXTRUDER,
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

        # If the sgthrs is too low, set to max
        current_sgthrs = self.extruder_driver.fields.get_field("sgthrs")
        if current_sgthrs < MINIMUM_SAFE_SGTHRS:
            self._set_extruder_sgthrs(255)
            self.calibration_required = True
        elif current_sgthrs == 255:
            self.calibration_required = True

    def _handle_config(self):
        self.toolhead = self.printer.lookup_object("toolhead")

        self.extruder = self.toolhead.extruder
        self.extruder_driver = self.printer.lookup_object("tmc2209 extruder")
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
        verify_heater = self.printer.lookup_object(
            f"verify_heater {heater.short_name}"
        )

        def new_callback(read_time, read_value):
            if read_value < OPEN_ADC_VALUE:
                sensor.adc_callback(read_time, read_value)
            else:
                heater.set_pwm(read_time, 0.0)
                verify_heater.error = 0.0
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

    def _set_extruder_sgthrs(self, sgthrs: int):
        reg_val = self.extruder_driver.fields.set_field("sgthrs", sgthrs)
        self.extruder_driver.mcu_tmc.set_register(
            "SGTHRS", reg_val, self.toolhead.get_last_move_time()
        )

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

    def cmd_UNLOAD_COCOAPRESS(self, gcmd: GCodeCommand):
        if self.state == States.UNLOADED:
            gcmd.respond_info("Already unloaded!")
            return
        self.state = States.INITIAL_UNLOAD
        self.continue_load_unload()

    def cmd_CONTINUE(self, gcmd):
        self._unregister_commands()
        self.continue_load_unload()

    def cmd_CALIBRATE_EXTRUDER(self, gcmd: GCodeCommand):
        "Auto-calibrate sensorless homing for the extruder"

        full_calibration = gcmd.get_int("FULL", 0, minval=0, maxval=1)
        save_config = gcmd.get_int("SAVE", 1, minval=0, maxval=1)

        min_dist = 1.0
        min_deviation = 0.001
        min_threshold = 70

        sgthrs = (
            255
            if full_calibration
            else self.extruder_driver.fields.get_field("sgthrs") + 15
        )

        sgthrs = min(sgthrs, 255)
        delta = 5

        with self._extruder_homing_current():
            while True:
                dist_moved = 0.0
                while dist_moved < min_dist:
                    sgthrs -= delta
                    self._set_extruder_sgthrs(sgthrs)
                    bottom = self.__home_extruder_in_direction(DIRECTION_BOTTOM)
                    top = self.__home_extruder_in_direction(DIRECTION_TOP)
                    dist_moved = min(bottom, top)
                    gcmd.respond_info(
                        f"{sgthrs=} moved {bottom=:0.3f} {top=:0.3f}"
                    )

                bottom = self.__home_extruder_in_direction(DIRECTION_BOTTOM)
                top = self.__home_extruder_in_direction(DIRECTION_TOP)
                second_dist = min(bottom, top)
                diff_dist = abs(dist_moved - second_dist)
                deviation = abs(bottom - top)

                if diff_dist < 0.5:
                    delta = 1

                if diff_dist < 0.5 and deviation < min_deviation:
                    break

                if sgthrs <= min_threshold:
                    raise gcmd.error(
                        f"Calibration failed, could not find a working sgthrs value above {min_threshold}"
                    )

                gcmd.respond_info(
                    f"{sgthrs=} is not accurate enough {deviation=:0.4f}"
                )

        self.calibration_required = False
        self.cocoa_runout.set_top(self.toolhead.get_position()[3])

        if save_config:
            pconfig: PrinterConfig = self.printer.lookup_object("configfile")
            pconfig.set("tmc2209 extruder", "driver_sgthrs", str(sgthrs))
            self.gcode.run_script_from_command("SAVE_CONFIG RESTART=0")
            gcmd.respond_info(f"Calibration saved: {sgthrs=} {deviation=:0.4f}")

        else:
            gcmd.respond_info(
                f"Calibration complete: {sgthrs=} {deviation=:0.4f}"
            )

    def cmd_HOME_COCOAPRESS(self, gcmd: GCodeCommand):
        if self.calibration_required:
            raise gcmd.error("Unable to home toolhead, calibration is required")

        direction = gcmd.get_int("DIR", 1)
        if direction not in (DIRECTION_BOTTOM, DIRECTION_TOP):
            raise gcmd.error("Invalid direction %s" % (direction,))
        dist_moved = self._home_extruder_in_direction(direction)
        gcmd.respond_info(
            "Homed %s mm in direction %s" % (dist_moved, direction)
        )

        if direction is DIRECTION_TOP:
            self.cocoa_runout.set_top(self.toolhead.get_position()[3])

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
            # Z-Hop to ensure the plunger won't hit the bed
            kinematics = self.toolhead.get_kinematics()
            position = self.toolhead.get_position()
            status = self.toolhead.get_status(self.toolhead.print_time)

            # Safe Z Hop
            z_hop = 50
            z_hop_speed = self.config.getsection("stepper_z").getfloat(
                "homing_speed", 5.0
            )

            if "z" not in status["homed_axes"]:
                # Always perform the z_hop if the Z axis is not homed
                position[2] = 0
                self.toolhead.set_position(position, homing_axes=[2])
                self.toolhead.manual_move([None, None, z_hop], z_hop_speed)
                kinematics.clear_homing_state((2,))

            elif position[2] < z_hop:
                position[2] = z_hop
                self.toolhead.move(position, z_hop_speed)

        with self._extruder_homing_current():
            return self.__home_extruder_in_direction(dir)

    def _set_extruder_current_for_homing(self, pre_homing):
        print_time = self.toolhead.get_last_move_time()
        ch = self.extruder_stepper.get_tmc_current_helper()
        dwell_time = ch.set_current_for_homing(print_time, pre_homing)
        if dwell_time:
            self.toolhead.dwell(dwell_time)

    @contextlib.contextmanager
    def _extruder_homing_current(self):
        self._set_extruder_current_for_homing(pre_homing=True)
        try:
            yield
        finally:
            self._set_extruder_current_for_homing(pre_homing=False)

    def __home_extruder_in_direction(self, dir: Literal[-1, 1]) -> float:
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
            "calibration_required": self.calibration_required,
            **self.cocoa_runout.get_status(eventtime),
        }


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

    def __init__(self, config: ConfigWrapper):
        self.config = config
        self.name = config.get_name()
        self.printer = config.get_printer()
        self.reactor: SelectReactor = self.printer.get_reactor()

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
            self.name,
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
                "cocoa_toolhead: cannot enable runout without homing the toolhead"
            )

        self.runout_enabled = True
        logger.info("cocoa_toolhead: runout enabled")

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


def load_config(config):
    return CocoaToolheadControl(config)
