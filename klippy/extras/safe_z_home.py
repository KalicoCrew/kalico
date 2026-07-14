# Perform Z Homing at specific XY coordinates.
#
# Copyright (C) 2019 Florian Heilmann <Florian.Heilmann@gmx.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

from __future__ import annotations

from typing import Callable

from ..mathutil import Point
from ..printer_info import PrinterInfo
from .probe import PrinterProbe


class SafeZHoming:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.home_x_pos, self.home_y_pos = config.getfloatlist(
            "home_xy_position", count=2, default=(None, None)
        )

        self.z_hop = config.getfloat("z_hop", default=0.0)
        self.z_hop_speed = config.getfloat("z_hop_speed", 15.0, above=0.0)

        zconfig = config.getsection("stepper_z")
        self.max_z = zconfig.getfloat("position_max", note_valid=False)

        self.speed = config.getfloat("speed", 50.0, above=0.0)
        self.move_to_previous = config.getboolean("move_to_previous", False)
        self.home_y_before_x = config.getboolean("home_y_before_x", False)

        self.printer.load_object(config, "homing")
        self.gcode = self.printer.lookup_object("gcode")
        self.prev_G28 = self.gcode.register_command("G28", None)
        self.gcode.register_command("G28", self.cmd_G28)

        if config.has_section("homing_override"):
            raise config.error(
                "homing_override and safe_z_homing cannot"
                + " be used simultaneously"
            )

        # Ensure the home position is set when the printer connects, making it available to other modules
        self.printer.register_event_handler(
            "klippy:connect", self._update_home_position
        )

    def _update_home_position(
        self,
        default_error: Callable[[str], Exception] | None = None,
        use_offsets: bool = False,
    ) -> tuple[float, float]:
        printer_info: PrinterInfo | None = self.printer.lookup_object(
            "printer_info", None
        )

        error = (
            self.printer.config_error
            if default_error is None
            else default_error
        )

        if printer_info is None and (
            self.home_x_pos is None or self.home_y_pos is None
        ):
            raise error(
                "printer properties not defined, therefore home_xy_position must be set"
            )

        # If the home position is set, use that.
        if self.home_x_pos is not None and self.home_y_pos is not None:
            return self.home_x_pos, self.home_y_pos

        # The probe might not be present, in that case, it should be okay to home
        # with the nozzle in the center of the bed.
        probe: PrinterProbe | None = self.printer.lookup_object("probe", None)
        probe_offsets = Point.origin()
        if probe is not None:
            offsets = probe.get_offsets()
            probe_offsets = Point(offsets[0], offsets[1])

        if not printer_info.is_rectangular:
            raise error(
                f"automatic home position calculation is not supported for"
                f" {printer_info.kinematics_name} kinematics, please specify"
                f" home_xy_position manually"
            )

        printer_info.require_properties(
            ["bed_corner_position", "bed_size"], error
        )

        bed_center = (
            printer_info.bed_corner_position
            + Point(*printer_info.bed_size) / 2.0
        )

        result = printer_info.nearest_point(bed_center - probe_offsets)

        if use_offsets:
            result += probe_offsets

        # In case the probe is currently not defined, it will not cache the calculated home position,
        # then on the next call, it will calculate it again with the probe defined:
        if probe is None:
            return result

        # Cache the calculated home position for future calls:
        self.home_x_pos, self.home_y_pos = (
            self.home_x_pos or result.x,
            self.home_y_pos or result.y,
        )

        return self.home_x_pos, self.home_y_pos

    def cmd_G28(self, gcmd):
        toolhead = self.printer.lookup_object("toolhead")

        # First get the home position for z, if it is not set, this will throw an error, which should
        # happen before the printer starts moving
        home_position = self._update_home_position()

        # Perform Z Hop if necessary
        if self.z_hop != 0.0:
            # Check if Z axis is homed and its last known position
            curtime = self.printer.get_reactor().monotonic()
            kin_status = toolhead.get_kinematics().get_status(curtime)
            pos = toolhead.get_position()

            if "z" not in kin_status["homed_axes"]:
                # Always perform the z_hop if the Z axis is not homed
                pos[2] = 0
                toolhead.set_position(pos, homing_axes=[2])
                toolhead.manual_move([None, None, self.z_hop], self.z_hop_speed)
                toolhead.get_kinematics().clear_homing_state((2,))
            elif pos[2] < self.z_hop:
                # If the Z axis is homed, and below z_hop, lift it to z_hop
                toolhead.manual_move([None, None, self.z_hop], self.z_hop_speed)

        # Determine which axes we need to home
        need_x, need_y, need_z = [
            gcmd.get(axis, None) is not None for axis in "XYZ"
        ]
        if not need_x and not need_y and not need_z:
            need_x = need_y = need_z = True

        if need_x or need_y:
            if self.home_y_before_x:
                axis_order = "yx"
            else:
                axis_order = "xy"
            for axis in axis_order:
                if axis == "x" and need_x:
                    g28_gcmd = self.gcode.create_gcode_command(
                        "G28", "G28", {"X": "0"}
                    )
                    self.prev_G28(g28_gcmd)
                elif axis == "y" and need_y:
                    g28_gcmd = self.gcode.create_gcode_command(
                        "G28", "G28", {"Y": "0"}
                    )
                    self.prev_G28(g28_gcmd)

        # Home Z axis if necessary
        if need_z:
            # Throw an error if X or Y are not homed
            curtime = self.printer.get_reactor().monotonic()
            kin_status = toolhead.get_kinematics().get_status(curtime)
            if (
                "x" not in kin_status["homed_axes"]
                or "y" not in kin_status["homed_axes"]
            ):
                raise gcmd.error("Must home X and Y axes first")

            # Do we need to detach the probe?
            dockable = self.printer.lookup_object("dockable_probe", None)
            if dockable is not None and dockable.detach_dockable_before_z_home:
                dockable.detach_probe()

            # Move to safe XY homing position
            prevpos = toolhead.get_position()
            toolhead.manual_move([*home_position], self.speed)

            # Home Z
            g28_gcmd = self.gcode.create_gcode_command("G28", "G28", {"Z": "0"})
            self.prev_G28(g28_gcmd)

            # Perform Z Hop again for pressure-based probes
            if self.z_hop:
                pos = toolhead.get_position()
                if pos[2] < self.z_hop:
                    toolhead.manual_move(
                        [None, None, self.z_hop], self.z_hop_speed
                    )

            # Move XY back to previous positions
            if self.move_to_previous:
                toolhead.manual_move(prevpos[:2], self.speed)


def load_config(config):
    return SafeZHoming(config)
