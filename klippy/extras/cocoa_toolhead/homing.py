"""
Klipper plugin to monitor toolhead adc values
to detect the toolhead's attachment status
"""

from __future__ import annotations

import contextlib
import logging
from typing import TYPE_CHECKING, Literal

from klippy import chelper
from klippy.extras.homing import Homing

if TYPE_CHECKING:
    from ...configfile import ConfigWrapper, PrinterConfig
    from ...gcode import GCodeCommand, GCodeDispatch
    from ...kinematics.extruder import PrinterExtruder
    from ...mcu import MCU_endstop
    from ...pins import PrinterPins
    from ...printer import Printer
    from ...stepper import MCU_stepper
    from ...toolhead import ToolHead
    from ..homing import PrinterHoming
    from ..query_endstops import QueryEndstops
    from ..stepper_enable import EnableTracking, PrinterStepperEnable
    from ..tmc import BaseTMCCurrentHelper
    from ..tmc2209 import TMC2209
    from .toolhead import CocoaToolheadControl

# This may need tweaking
MINIMUM_SAFE_SGTHRS = 50


DIRECTION_TOP = -1
DIRECTION_BOTTOM = 1


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


class CocoaHoming:
    # constants
    total_maximum_homing_dist = 200  # mm

    printer: Printer
    toolhead: ToolHead
    fake_toolhead_for_homing: FakeExtruderHomingToolhead
    extruder: PrinterExtruder
    extruder_stepper: MCU_stepper
    extruder_stepper_enable: EnableTracking
    extruder_driver: TMC2209

    stepper_enable: PrinterStepperEnable

    def __init__(
        self, cocoa_toolhead: CocoaToolheadControl, config: ConfigWrapper
    ):
        self.cocoa_toolhead = cocoa_toolhead
        self.name = cocoa_toolhead.name
        self.mux_name = cocoa_toolhead.mux_name
        self.printer = config.get_printer()
        self.logger = cocoa_toolhead.logger.getChild("runout")

        self.gcode: GCodeDispatch = self.printer.lookup_object("gcode")

        self.homing_speed = config.getfloat("homing_speed", 25.0, above=0.0)
        self.z_homing_speed = config.getsection("stepper_z").getfloat(
            "homing_speed", 5.0
        )

        # State
        self.has_z_hopped = False

        # extruder endstop setup
        self.endstop_pin = config.get("endstop_pin", None)
        ppins: PrinterPins = self.printer.lookup_object("pins")
        self.mcu_endstop: MCU_endstop = ppins.setup_pin(
            "endstop", self.endstop_pin
        )
        self.endstops = [(self.mcu_endstop, self.cocoa_toolhead.extruder_name)]
        query_endstops: QueryEndstops = self.printer.load_object(
            config, "query_endstops"
        )
        query_endstops.register_endstop(
            self.mcu_endstop, self.cocoa_toolhead.extruder_name
        )

        # register commands
        self.gcode.register_mux_command(
            "HOME_COCOAPRESS",
            "TOOL",
            self.mux_name,
            self.cmd_HOME_COCOAPRESS,
        )
        self.gcode.register_mux_command(
            "CALIBRATE_EXTRUDER",
            "TOOL",
            self.mux_name,
            self.cmd_CALIBRATE_EXTRUDER,
        )

        self.printer.register_event_handler(
            "klippy:mcu_identify", self._on_identify
        )
        self.printer.register_event_handler("klippy:ready", self._on_ready)
        self.printer.register_event_handler(
            "cocoa_memory:connected", self._memory_connected
        )
        self.printer.register_event_handler(
            "cocoa_memory:disconnected", self._memory_disconnected
        )

    def _on_identify(self):
        self.toolhead = self.printer.lookup_object("toolhead")

        self.extruder: PrinterExtruder = self.printer.lookup_object(
            self.cocoa_toolhead.extruder_name
        )
        self.extruder_driver = self.printer.lookup_object("tmc2209 extruder")
        self.extruder_stepper = self.extruder.extruder_stepper.stepper
        self.stepper_enable = self.printer.lookup_object("stepper_enable")
        self.extruder_stepper_enable = self.stepper_enable.lookup_enable(
            "extruder"
        )
        self.mcu_endstop.add_stepper(self.extruder_stepper)
        self.fake_toolhead_for_homing = FakeExtruderHomingToolhead(
            self.toolhead, self.extruder_stepper
        )

    def _on_ready(self):
        # If the sgthrs is too low, set to max
        current_sgthrs = self.extruder_driver.fields.get_field("sgthrs")
        if current_sgthrs < MINIMUM_SAFE_SGTHRS:
            self._set_extruder_sgthrs(255)
            self.calibration_required = True
        elif current_sgthrs == 255:
            self.calibration_required = True

    def _memory_connected(self, name: str, config: dict):
        if self.name != name:
            return

        self.calibration_required = "sgthrs" in config
        self._set_extruder_sgthrs(config.get("sgthrs", 255))

    def _memory_disconnected(self, name: str):
        if self.name != name:
            return
        ...

    def _set_extruder_sgthrs(self, sgthrs: int):
        reg_val = self.extruder_driver.fields.set_field("sgthrs", sgthrs)
        self.extruder_driver.mcu_tmc.set_register(
            "SGTHRS", reg_val, self.toolhead.get_last_move_time()
        )

    def cmd_CALIBRATE_EXTRUDER(self, gcmd: GCodeCommand):
        "Auto-calibrate sensorless homing for the extruder"

        full_calibration = gcmd.get_int("FULL", 0, minval=0, maxval=1)
        save_config = gcmd.get_int("SAVE", 1, minval=0, maxval=1)

        min_dist = 1.0
        min_deviation = 0.001
        min_threshold = 70

        if full_calibration:
            sgthrs = 255
        else:
            sgthrs = self.extruder_driver.fields.get_field("sgthrs") + 15

        initial_sgthrs = sgthrs = min(sgthrs, 255)
        if initial_sgthrs == 255:
            full_calibration = True

        delta = 5
        last_deviation = None
        deviation_increase_count = 0

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

                if last_deviation is not None:
                    if deviation > last_deviation:
                        deviation_increase_count += 1
                    else:
                        deviation_increase_count = 0
                        delta = abs(delta)

                    if deviation_increase_count > 3:
                        gcmd.respond_info(
                            "Failed to find a safe threshold, restarting calibration"
                        )
                        full_calibration = True
                        sgthrs = initial_sgthrs = 255
                        delta = 5
                        continue

                last_deviation = deviation

                if sgthrs <= min_threshold:
                    raise gcmd.error(
                        f"Calibration failed, could not find a working sgthrs value above {min_threshold}"
                    )

                gcmd.respond_info(
                    f"{sgthrs=} is not accurate enough {deviation=:0.4f}"
                )

        self.calibration_required = False
        self.cocoa_toolhead.runout.set_top(self.toolhead.get_position()[3])

        if self.cocoa_toolhead.memory.connected:
            self.cocoa_toolhead.memory.set("sgthrs", sgthrs)

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
        "`HOME_COCOAPRESS DIR=[-1|1]`: Home the toolhead."

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
            self.cocoa_toolhead.runout.set_top(self.toolhead.get_position()[3])

    def home_extruder_to_top(self) -> float:
        return self._home_extruder_in_direction(DIRECTION_TOP)

    def home_extruder_to_bottom(self) -> float:
        return self._home_extruder_in_direction(DIRECTION_BOTTOM)

    def _home_extruder_in_direction(self, dir: Literal[-1, 1]) -> float:
        if dir == DIRECTION_BOTTOM:
            # Z-Hop to ensure the plunger won't hit the bed
            position = self.toolhead.get_position()
            status = self.toolhead.get_status(self.toolhead.print_time)

            # Safe Z Hop
            z_hop = 50

            if "z" not in status["homed_axes"] and not self.has_z_hopped:
                # Always perform the z_hop if the Z axis is not homed
                position[2] = 0
                self.toolhead.set_position(position, homing_axes=[2])
                self.toolhead.manual_move(
                    [None, None, z_hop], self.z_homing_speed
                )
                self.has_z_hopped = True

            elif position[2] < z_hop:
                position[2] = z_hop
                self.toolhead.move(position, self.z_homing_speed)

        with self._extruder_homing_current():
            return self.__home_extruder_in_direction(dir)

    def _set_extruder_current_for_homing(self, pre_homing):
        print_time = self.toolhead.get_last_move_time()
        ch: BaseTMCCurrentHelper = (
            self.extruder_stepper.get_tmc_current_helper()
        )
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
        start_position = curpos[3]
        end_position = start_position
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

        end_position = trig_pos[3]
        total_homing_distance = round(abs(end_position - start_position), 6)

        self.logger.info(
            f"{self.name}: successfully homed! {start_position=} {end_position=}"
        )

        self.toolhead.flush_step_generation()
        return total_homing_distance
