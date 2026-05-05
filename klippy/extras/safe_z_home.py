# Perform Z Homing at specific XY coordinates.
#
# Copyright (C) 2019 Florian Heilmann <Florian.Heilmann@gmx.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.


class SafeZHoming:
    def __init__(self, config):
        self.printer = config.get_printer()
        x_pos, y_pos = config.getfloatlist("home_xy_position", count=2)
        self.home_x_pos, self.home_y_pos = x_pos, y_pos

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
        self.printer.register_event_handler(
            "klippy:connect", self._handle_connect
        )

        if config.has_section("homing_override"):
            raise config.error(
                "homing_override and safe_z_homing cannot"
                + " be used simultaneously"
            )

    def _handle_connect(self):
        self.safe_move = self.printer.lookup_object("safe_move")

    def cmd_G28(self, gcmd):
        toolhead = self.printer.lookup_object("toolhead")

        # Perform Z Hop if necessary
        if self.z_hop != 0.0:
            self._z_lift(toolhead)

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
            toolhead.manual_move([self.home_x_pos, self.home_y_pos], self.speed)

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

    def _z_lift(self, toolhead):
        curtime = self.printer.get_reactor().monotonic()
        kin_status = toolhead.get_kinematics().get_status(curtime)
        pos = toolhead.get_position()

        if "z" not in kin_status["homed_axes"]:
            self.safe_move.move(
                toolhead,
                "z",
                self.z_hop,
                self.z_hop_speed,
                allow_unsafe=True,
            )
        elif pos[2] < self.z_hop:
            toolhead.manual_move([None, None, self.z_hop], self.z_hop_speed)


def load_config(config):
    return SafeZHoming(config)
