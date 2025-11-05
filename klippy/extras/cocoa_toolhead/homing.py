"""
Klipper plugin to monitor toolhead adc values
to detect the toolhead's attachment status
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING

from klippy import chelper

if TYPE_CHECKING:
    from ...stepper import MCU_stepper
    from ...toolhead import ToolHead


logger = logging.getLogger(__name__)


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
