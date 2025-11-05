"""
Klipper plugin to monitor toolhead adc values
to detect the toolhead's attachment status
"""

from __future__ import annotations

import contextlib
import logging
from typing import TYPE_CHECKING, Literal

from klippy.extras.homing import Homing

from .homing import FakeExtruderHomingToolhead
from .load_wizard import CocoaLoadWizard
from .memory import CocoaMemory
from .preheater import CocoaPreheater
from .runout import CocoaRunout

if TYPE_CHECKING:
    from ...configfile import ConfigWrapper, PrinterConfig
    from ...gcode import GCodeCommand, GCodeDispatch
    from ...kinematics.extruder import PrinterExtruder
    from ...mcu import MCU_endstop
    from ...pins import PrinterPins
    from ...printer import Printer
    from ...stepper import MCU_stepper
    from ...toolhead import ToolHead
    from ..gcode_macro import PrinterGCodeMacro
    from ..homing import HomingMove, PrinterHoming
    from ..query_endstops import QueryEndstops
    from ..stepper_enable import EnableTracking, PrinterStepperEnable
    from ..tmc2209 import TMC2209

DIRECTION_TOP = -1
DIRECTION_BOTTOM = 1


# Open circuits on the ADC cause a near 1.0 reading, typically above 0.998
OPEN_ADC_VALUE = 0.99

DEFAULT_EXTRUDER = "extruder"
DEFAULT_BODY_HEATER = "heater_generic Body_Heater"

# This may need tweaking
MINIMUM_SAFE_SGTHRS = 50


class CocoaToolheadControl:
    # constants
    total_maximum_homing_dist = 200  # mm

    printer: Printer
    config: ConfigWrapper

    runout: CocoaRunout
    memory: CocoaMemory
    load_wizard: CocoaLoadWizard

    def __init__(self, config: ConfigWrapper):
        self.printer = config.get_printer()
        self.config = config
        self.name = self.config.get_name().split()[-1]
        self.mux_name = self.name if self.name != "cocoa_toolhead" else None

        self.logger = logging.getLogger(__name__).getChild(
            self.name
            if self.name == "cocoa_toolhead"
            else f"cocoa_toolhead[{self.name}]"
        )

        self.toolhead: ToolHead = None
        self.fake_toolhead_for_homing: FakeExtruderHomingToolhead = None
        self.extruder: PrinterExtruder = None
        self.extruder_stepper: MCU_stepper = None
        self.extruder_stepper_enable: EnableTracking = None
        self.extruder_driver: TMC2209 = None

        self.load_wizard = CocoaLoadWizard(self, config)
        self.runout = CocoaRunout(self, config)
        self.memory = CocoaMemory(self, config)
        self.preheater = CocoaPreheater(self, config)

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
        self.calibration_required = False
        self.has_z_hopped = False

        self.printer.register_event_handler("klippy:connect", self._on_ready)
        self.printer.register_event_handler(
            "klippy:mcu_identify", self._on_identify
        )
        self.printer.register_event_handler(
            "cocoa_memory:connected", self._memory_connected
        )
        self.printer.register_event_handler(
            "cocoa_memory:disconnected", self._memory_disconnected
        )

        self.printer.register_event_handler(
            "homing:homing_move_end", self._on_homing_move_end
        )
        self.printer.register_event_handler(
            "stepper_enable:motor_off", self._on_motor_off
        )

        self.gcode: GCodeDispatch = self.printer.lookup_object("gcode")

        self.homing_speed = config.getfloat("homing_speed", 25.0, above=0.0)

        self.runout_pause = config.getboolean("pause_on_runout", True)

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
        self.gcode.register_mux_command(
            "SET_COCOA_TOOLHEAD",
            "TOOL",
            self.mux_name,
            self.cmd_SET_COCOA_TOOLHEAD,
        )

        # extruder endstop setup
        self.endstop_pin = config.get("endstop_pin", None)
        ppins: PrinterPins = self.printer.lookup_object("pins")
        self.mcu_endstop: MCU_endstop = ppins.setup_pin(
            "endstop", self.endstop_pin
        )
        self.endstops = [(self.mcu_endstop, self.extruder_name)]
        query_endstops: QueryEndstops = self.printer.load_object(
            config, "query_endstops"
        )
        query_endstops.register_endstop(self.mcu_endstop, self.extruder_name)

        self.gcode_macro: PrinterGCodeMacro = self.printer.load_object(
            config, "gcode_macro"
        )
        self.attach_tmpl = self.gcode_macro.load_template(
            config, "attach_gcode", ""
        )
        self.detach_tmpl = self.gcode_macro.load_template(
            config, "detach_gcode", ""
        )

    def _on_ready(self):
        self.logger.info("Initializing Cocoa Toolhead")

        self.attached = None

        extruder = self.printer.lookup_object(self.extruder_name)
        body_heater = self.printer.lookup_object(self.body_heater_name)

        self.logger.debug("Injecting adc callbacks")

        self.inject_adc_callback(extruder.heater)
        self.inject_adc_callback(body_heater)

        # If the sgthrs is too low, set to max
        current_sgthrs = self.extruder_driver.fields.get_field("sgthrs")
        if current_sgthrs < MINIMUM_SAFE_SGTHRS:
            self._set_extruder_sgthrs(255)
            self.calibration_required = True
        elif current_sgthrs == 255:
            self.calibration_required = True

    stepper_enable: PrinterStepperEnable
    extruder_stepper_enable: EnableTracking

    def _on_identify(self):
        self.toolhead = self.printer.lookup_object("toolhead")

        self.extruder = self.toolhead.extruder
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

    def _on_homing_move_end(self, homing_move: HomingMove): ...
    def _on_motor_off(self, eventtime): ...

    def _memory_connected(self, name: str, config: dict):
        if self.name != name:
            return

        if "sgthrs" in config:
            self._set_extruder_sgthrs(config.get("sgthrs"))
            self.calibration_required = False

    def _memory_disconnected(self, name: str):
        if self.name != name:
            return
        ...

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
        self.logger.debug(
            f"{self.name}: Intercepted ADC callback for {heater.name}"
        )

    def receive_sensor_value(self, heater, value: float):
        self.last_readings[heater.name] = value

        is_attached = value < OPEN_ADC_VALUE
        if is_attached != self.attached:
            self.attached = is_attached

            self.gcode.respond_info(
                f"Cocoa Press: Toolhead {'attached' if is_attached else 'detached'}"
            )

            if is_attached:
                self.printer.send_event("cocoa_toolhead:attached", self.name)
                self.attach_tmpl()
            else:
                self.printer.send_event("cocoa_toolhead:detached", self.name)
                self.detach_tmpl()

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

        sgthrs = (
            255
            if full_calibration
            else self.extruder_driver.fields.get_field("sgthrs") + 15
        )

        sgthrs = min(sgthrs, 255)
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

                    if deviation_increase_count > 3:
                        ...  # Reverse delta until corrects?
                last_deviation = deviation

                if sgthrs <= min_threshold:
                    raise gcmd.error(
                        f"Calibration failed, could not find a working sgthrs value above {min_threshold}"
                    )

                gcmd.respond_info(
                    f"{sgthrs=} is not accurate enough {deviation=:0.4f}"
                )

        self.calibration_required = False
        self.runout.set_top(self.toolhead.get_position()[3])

        if self.memory.connected:
            self.memory.set("sgthrs", sgthrs)
            self.memory.save()

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
            self.runout.set_top(self.toolhead.get_position()[3])

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

            if "z" not in status["homed_axes"] and not self.has_z_hopped:
                # Always perform the z_hop if the Z axis is not homed
                position[2] = 0
                self.toolhead.set_position(position, homing_axes=[2])
                self.toolhead.manual_move([None, None, z_hop], z_hop_speed)
                self.has_z_hopped = True

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

    def get_status(self, eventtime):
        return {
            **self.load_wizard.get_state(eventtime),
            "attached": self.attached,
            "adc": self.last_readings,
            "calibration_required": self.calibration_required,
            "runout": self.runout.get_status(eventtime),
            "memory": self.memory.get_status(eventtime),
        }

    # For updating user values (name of the toolhead)
    def cmd_SET_COCOA_TOOLHEAD(self, gcmd: GCodeCommand):
        """
        `SET_COCOA_TOOLHEAD NAME="my fancy toolhead"`

        Update Cocoa Toolhead settings
        """
        if not self.memory.connected:
            raise gcmd.error(
                "Unable to update toolhead memory when it's not connected"
            )

        new_name = gcmd.get("NAME", None)
        if new_name is not None:
            self.memory.set("name", new_name)
            gcmd.respond_info(f"{self.name}: Set name to {new_name}")

        self.memory.save()


# SET_COCOA_TOOLHEAD TOOL=left NAME="frank's shiny toolhead"
# SET_COCOA_TOOLHEAD TOOL=right NAME="frank's old toolhead"
