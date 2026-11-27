# Code for handling the kinematics of corexz robots
# with per-axis limits for velocity and acceleration
#
#
# Modified version of limited_cartesian to work as limited_corexz
#
# Modified by:
# Copyright © 2024 Brandon Smith <honestbrotherstv@gmail.com>
#
# Copyright (C) 2020-2021  Mael Kerbiriou <piezo.wdimd@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#


from math import atan2, hypot, pi
from sys import float_info

from . import corexz

EPSILON = float_info.epsilon


class LimitedCoreXZKinematics(corexz.CoreXZKinematics):
    def __init__(self, toolhead, config):
        corexz.CoreXZKinematics.__init__(self, toolhead, config)

        # Setup y axis limits
        max_velocity, max_accel = toolhead.get_max_velocity()
        for ax in "xyz":
            setattr(
                self,
                f"max_{ax}_velocity",
                config.getfloat(f"max_{ax}_velocity", max_velocity, above=0.0),
            )
            setattr(
                self,
                f"max_{ax}_accel",
                config.getfloat(f"max_{ax}_accel", max_accel, above=0.0),
            )
        self.scale_per_axis = config.getboolean("scale_xy_accel", False)
        config.get_printer().lookup_object("gcode").register_command(
            "SET_KINEMATICS_LIMIT", self.cmd_SET_KINEMATICS_LIMIT
        )

    def cmd_SET_KINEMATICS_LIMIT(self, gcmd):
        for ax in "xyz":
            for limit in ("velocity", "accel"):
                name = f"max_{ax}_{limit}"
                setattr(
                    self,
                    name,
                    gcmd.get_float(
                        f"{ax.upper()}_{limit.upper()}",
                        getattr(self, name),
                        above=0.0,
                    ),
                )
        self.scale_per_axis = bool(
            gcmd.get_int("SCALE", self.scale_per_axis, minval=0, maxval=1)
        )

        max_velocities = [
            self.max_x_velocity,
            self.max_y_velocity,
            self.max_z_velocity,
        ]
        max_accels = [self.max_x_accel, self.max_y_accel, self.max_z_accel]
        msg = ("x,y,z max_velocities: %r\nx,y,z max_accels: %r\n") % (
            max_velocities,
            max_accels,
        )

        if self.scale_per_axis:
            msg += "Per axis accelerations limits scale with current acceleration.\n"
        else:
            msg += "Per axis accelerations limits are independent of current acceleration.\n"
        msg += (
            "Maximum XY velocity of %.1f mm/s reached on %.0f degrees diagonals.\n"
            "Maximum XY acceleration of %.0f mm/s^2 reached on %.0f degrees diagonals."
        ) % (
            hypot(self.max_x_velocity, self.max_y_velocity),
            180 * atan2(self.max_y_velocity, self.max_x_velocity) / pi,
            hypot(self.max_x_accel, self.max_y_accel),
            180 * atan2(self.max_y_accel, self.max_x_accel) / pi,
        )
        gcmd.respond_info(msg)

    def check_move(self, move):
        if not move.is_kinematic_move:
            return

        self._check_endstops(move)

        x_r, y_r, z_r = move.axes_r[:3]
        x_r = max(abs(x_r), EPSILON)
        y_r = max(abs(y_r), EPSILON)
        max_v = min(self.max_x_velocity / x_r, self.max_y_velocity / y_r)
        max_a = min(self.max_x_accel / x_r, self.max_y_accel / y_r)

        if self.scale_per_axis:
            _, toolhead_max_a = move.toolhead.get_max_velocity()
            max_a *= toolhead_max_a / hypot(self.max_x_accel, self.max_y_accel)

        if z_r:
            z_r = abs(z_r)
            max_v = min(max_v, self.max_z_velocity / z_r)
            max_a = min(max_a, self.max_z_accel / z_r)

        move.limit_speed(max_v, max_a)


def load_kinematics(toolhead, config):
    return LimitedCoreXZKinematics(toolhead, config)
