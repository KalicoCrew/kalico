from enum import Enum
from typing import TYPE_CHECKING
import math
import logging

from klippy.extras.homing import Homing
from .force_move import calc_move_time

if TYPE_CHECKING:
    from ..toolhead import ToolHead
    from ..gcode import GCodeCommand
    from .homing import PrinterHoming
    from ..stepper import MCU_stepper
    from ..kinematics.extruder import PrinterExtruder


class States(Enum):
    """
    UNKNOWN
    INITIAL_UNLOAD
    AWAITING_THUMBSCREW_REMOVAL
    AWAITING_TOOLHEAD_REMOVAL_UNLOAD
    UNLOADED
    UNLOADED_READY_FOR_CAP
    INITIAL_LOAD
    AWAITING_TOOLHEAD_INSTALL_INITIAL_LOAD
    AWAITING_PLUNGER_CAP_INSTALL
    AWAITING_TOOLHEAD_REMOVAL_CORE_LOAD
    AWAITING_TOOLHEAD_INSTALL_CORE_LOAD
    LOADED
    """

    ABORTED = -1
    UNKNOWN = 0
    INITIAL_UNLOAD = 1
    AWAITING_THUMBSCREW_REMOVAL = 2
    AWAITING_TOOLHEAD_REMOVAL_UNLOAD = 3
    UNLOADED = 4
    UNLOADED_READY_FOR_CAP = 5
    INITIAL_LOAD = 6
    AWAITING_TOOLHEAD_INSTALL_INITIAL_LOAD = 7
    AWAITING_PLUNGER_CAP_INSTALL = 8
    AWAITING_TOOLHEAD_REMOVAL_CORE_LOAD = 9
    AWAITING_TOOLHEAD_INSTALL_CORE_LOAD = 10
    LOADED = 11


class FakeExtruderHomingToolhead:
    def __init__(self, toolhead, extruder_stepper: "MCU_stepper"):
        self.toolhead: ToolHead = toolhead
        self.extruder_stepper = extruder_stepper

    def get_position(self):
        return self.toolhead.get_position()

    def set_position(self, pos, homing_axes=()):
        logging.info(f"setting position to {pos}, homing_axes={homing_axes}")
        self.toolhead.set_position(pos, homing_axes=homing_axes)

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
        self.fake_toolhead_for_homing: FakeExtruderHomingToolhead = None
        self.extruder: PrinterExtruder = None
        self.extruder_stepper: MCU_stepper = None
        self.move_speed = config.getfloat("load_speed", 25.0, above=0.0)
        self.homing_speed = config.getfloat("homing_speed", 25.0, above=0.0)

        self.load_retract_distance = 150  # mm
        self.load_nozzle_push_distance = 10  # mm
        self.total_maximum_homing_dist = 300  # mm
        self.homing_chunk_size = 5  # mm
        self.empty_tube_travel_distance_cutoff = 180  # mm

        self.state = States.UNKNOWN
        self.state_pre_abort = None
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

        self.endstop_pin = config.get("endstop_pin", None)
        ppins = self.printer.lookup_object("pins")
        self.mcu_endstop = ppins.setup_pin("endstop", self.endstop_pin)
        self.endstops = [(self.mcu_endstop, "extruder")]
        query_endstops = self.printer.load_object(config, "query_endstops")
        query_endstops.register_endstop(self.mcu_endstop, "extruder")

    def _handle_config(self):
        self.toolhead = self.printer.lookup_object("toolhead")

        self.extruder = self.toolhead.extruder
        self.extruder_stepper = self.extruder.extruder_stepper.stepper
        self.mcu_endstop.add_stepper(self.extruder_stepper)
        self.fake_toolhead_for_homing = FakeExtruderHomingToolhead(
            self.toolhead, self.extruder_stepper
        )

    def handle_connect(self):
        pass

    def handle_ready(self):
        pass

    def handle_enable(self):
        pass

    def cmd_LOAD_COCOAPRESS(self, gcmd: "GCodeCommand"):
        # if self.state == States.UNKNOWN or self.state == States.UNLOADED:
        #     gcmd.respond_info("Please unload before trying to load!")
        #     return
        if self.state != States.UNLOADED_READY_FOR_CAP:
            self.state = States.INITIAL_LOAD
        self.proceed_to_next(gcmd)

    def proceed_to_next(self, gcmd):
        if self.state == States.UNKNOWN or self.state == States.ABORTED:
            raise gcmd.error("Unknown state!")

        elif self.state == States.INITIAL_UNLOAD:
            self.home_extruder_to_top()
            homed_dist = self.home_extruder_to_bottom()
            if homed_dist > self.empty_tube_travel_distance_cutoff:
                # no tube installed, skip to prompt for cap
                self.state = States.UNLOADED_READY_FOR_CAP
                gcmd.respond_info("Ready for load!")
                self._unregister_commands()
            else:
                self.state = States.AWAITING_THUMBSCREW_REMOVAL
                self._register_commands()
                gcmd.respond_info(
                    "Please remove the thumbscrew and run CONTINUE"
                )

        elif self.state == States.AWAITING_THUMBSCREW_REMOVAL:
            self.move_extruder(self.load_nozzle_push_distance, self.move_speed)
            gcmd.respond_info("Please remove the toolhead and run CONTINUE")
            self.state = States.AWAITING_TOOLHEAD_REMOVAL_UNLOAD
            # TODO FRANK - event listening for toolhead disconnect to continue automatically here
            self._register_commands()

        elif self.state == States.AWAITING_TOOLHEAD_REMOVAL_UNLOAD:
            gcmd.respond_info("Ready to load!")
            self.state = States.UNLOADED
            self._unregister_commands()

        elif self.state == States.INITIAL_LOAD:
            # TODO FRANK - check if toolhead is installed,
            # if it is, skip this check
            gcmd.respond_info(
                "Reinstall toolhead with cartridge removed and run CONTINUE"
            )
            self.state = States.AWAITING_TOOLHEAD_INSTALL_INITIAL_LOAD
            self._register_commands()

        elif self.state == States.UNLOADED_READY_FOR_CAP:
            gcmd.respond_info("Install red cap on plunger and run CONTINUE")
            self.state = States.AWAITING_PLUNGER_CAP_INSTALL
            self._register_commands()

        elif self.state == States.AWAITING_TOOLHEAD_INSTALL_INITIAL_LOAD:
            self.home_extruder_to_bottom()
            gcmd.respond_info("Install red cap on plunger and run CONTINUE")
            self.state = States.AWAITING_PLUNGER_CAP_INSTALL
            self._register_commands()

        elif self.state == States.AWAITING_PLUNGER_CAP_INSTALL:
            self.move_extruder(-self.load_retract_distance, self.move_speed)
            gcmd.respond_info("Remove toolhead, install core, and run CONTINUE")
            self.state = States.AWAITING_TOOLHEAD_REMOVAL_CORE_LOAD
            # TODO FRANK - detect removal of toolhead and continue automatically
            self._register_commands()

        elif self.state == States.AWAITING_TOOLHEAD_REMOVAL_CORE_LOAD:
            gcmd.respond_info(
                "Reinstall toolhead (with core installed) and run CONTINUE"
            )
            self.state = States.AWAITING_TOOLHEAD_INSTALL_CORE_LOAD
            # TODO FRANK - detect install of toolhead and continue automatically
            self._register_commands()

        elif self.state == States.AWAITING_TOOLHEAD_INSTALL_CORE_LOAD:
            self.home_extruder_to_bottom()
            gcmd.respond_info("Loaded! Ready for preheat!")
            self.state = States.LOADED
            self._unregister_commands()

    def cmd_UNLOAD_COCOAPRESS(self, gcmd):
        if self.state == States.UNLOADED:
            gcmd.respond_info("Already unloaded!")
            return
        self.state = States.INITIAL_UNLOAD
        self.proceed_to_next(gcmd)

    def cmd_HOME_COCOAPRESS(self, gcmd):
        direction = gcmd.get_int("DIR", 1)
        if direction not in (1, -1):
            raise gcmd.error("Invalid direction %s" % (direction,))
        dist_moved = self._home_extruder_in_direction(direction)
        gcmd.respond_info(
            "Homed %s mm in direction %s" % (dist_moved, direction)
        )

    def home_extruder_to_top(self) -> float:
        return self._home_extruder_in_direction(-1)

    def home_extruder_to_bottom(self) -> float:
        return self._home_extruder_in_direction(1)

    def _set_extruder_current_for_homing(self, pre_homing):
        print_time = self.toolhead.get_last_move_time()
        ch = self.extruder_stepper.get_tmc_current_helper()
        dwell_time = ch.set_current_for_homing(print_time, pre_homing)
        if dwell_time:
            self.toolhead.dwell(dwell_time)

    def _home_extruder_in_direction(self, dir: int) -> float:
        self._set_extruder_current_for_homing(pre_homing=True)
        try:
            return self.__home_extruder_in_direction(dir)
        finally:
            self._set_extruder_current_for_homing(pre_homing=False)

    def __home_extruder_in_direction(self, dir: int) -> float:
        """
        dir should be 1 or -1
        """

        phoming: PrinterHoming = self.printer.lookup_object("homing")

        homing_distance = self.homing_chunk_size * dir

        max_iterations = math.ceil(
            self.total_maximum_homing_dist / self.homing_chunk_size
        )

        curpos = self.toolhead.get_position()
        starting_e_pos = curpos[3]
        ending_e_pos = starting_e_pos
        # logging.info(
        #     f"total_max_homing distance: {self.total_maximum_homing_dist}"
        # )
        # logging.info(f"homing distance: {homing_distance}")
        # logging.info(f"max iterations: {max_iterations}")
        homing_state = Homing(self.printer)
        for _ in range(max_iterations):
            curpos[3] += homing_distance
            try:
                logging.info("homing!")
                trig_pos = phoming.manual_home(
                    toolhead=self.fake_toolhead_for_homing,
                    endstops=self.endstops,
                    pos=curpos,
                    probe_pos=True,
                    speed=self.homing_speed,
                    triggered=True,
                    check_triggered=True,  # raise exception if no trigger on full movement
                )
                self.toolhead.dwell(0.1)
                self.toolhead.set_position(curpos)
                homing_state._reset_endstop_states(self.endstops)
                logging.info(f"triggered at {trig_pos}")
                ending_e_pos = trig_pos[3]
                break
            except Exception as e:
                if "No trigger on" in str(e):
                    # no trigger, move down
                    logging.info("no trigger!")
                    continue
                raise e
        logging.info("successfully homed!")
        logging.info(f"starting_position: {starting_e_pos}")
        logging.info(f"ending position: {ending_e_pos}")
        total_homing_distance = round(abs(ending_e_pos - starting_e_pos), 6)
        leftover_dist = total_homing_distance % self.homing_chunk_size

        leftover_dist = (
            math.ceil(leftover_dist * 1000) / 1000
        )  # to avoid float precision issues

        # dwell the time for the leftover distance
        _, accel_t, cruise_t, _ = calc_move_time(
            leftover_dist, self.homing_speed, self.toolhead.max_accel
        )
        est_segment_move_time = (2 * accel_t) + cruise_t
        self.toolhead.dwell(est_segment_move_time)
        self.toolhead.flush_step_generation()
        return total_homing_distance

    def move_extruder(self, amount, speed):
        last_pos = self.toolhead.get_position()
        new_pos = (last_pos[0], last_pos[1], last_pos[2], last_pos[3] + amount)
        self.toolhead.manual_move(new_pos, speed)

    def cmd_CONTINUE(self, gcmd):
        self._unregister_commands()
        self.proceed_to_next(gcmd)

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


def load_config(config):
    return CocoaPress(config)
