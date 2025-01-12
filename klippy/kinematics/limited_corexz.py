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


from sys import float_info
from math import hypot, atan2, pi

from . import corexz

EPSILON = float_info.epsilon


class LimitedCoreXZKinematics(corexz.CoreXZKinematics):
    def __init__(self, toolhead, config):
        corexz.CoreXZKinematics.__init__(self, toolhead, config)

        # Setup y axis limits
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_velocities = [
            config.getfloat("max_%s_velocity" % ax, max_velocity, above=0.0)
            for ax in "xyz"
        ]
        self.max_accels = [
            config.getfloat("max_%s_accel" % ax, max_accel, above=0.0)
            for ax in "xyz"
        ]
        self.xy_hypot_accel = hypot(*self.max_accels[:2])
        self.scale_per_axis = config.getboolean("scale_xy_accel", False)
        config.get_printer().lookup_object("gcode").register_command(
            "SET_KINEMATICS_LIMIT", self.cmd_SET_KINEMATICS_LIMIT
        )

    def cmd_SET_KINEMATICS_LIMIT(self, gcmd):
        self.max_velocities = [
            gcmd.get_float("%s_VELOCITY" % ax, max_v, above=0.0)
            for max_v, ax in zip(self.max_velocities, "XYZ")
        ]
        self.max_accels = [
            gcmd.get_float("%s_ACCEL" % ax, max_a, above=0.0)
            for max_a, ax in zip(self.max_accels, "XYZ")
        ]
        self.xy_hypot_accel = hypot(*self.max_accels[:2])
        self.scale_per_axis = bool(
            gcmd.get_int("SCALE", self.scale_per_axis, minval=0, maxval=1)
        )

        msg = ("x,y,z max_velocities: %r\nx,y,z max_accels: %r\n") % (
            self.max_velocities,
            self.max_accels,
        )

        if self.scale_per_axis:
            msg += "Per axis accelerations limits scale with current acceleration.\n"
        else:
            msg += "Per axis accelerations limits are independent of current acceleration.\n"
        msg += (
            "Maximum XY velocity of %.1f mm/s reached on %.0f degrees diagonals.\n"
            "Maximum XY acceleration of %.0f mm/s^2 reached on %.0f degrees diagonals."
        ) % (
            hypot(*self.max_velocities[:2]),
            180 * atan2(self.max_velocities[1], self.max_velocities[0]) / pi,
            self.xy_hypot_accel,
            180 * atan2(self.max_accels[1], self.max_accels[0]) / pi,
        )
        gcmd.respond_info(msg)

    def check_move(self, move):
        if not move.is_kinematic_move:
            return

        self._check_endstops(move)

        x_r, y_r, z_r = move.axes_r[:3]
        x_max_v, y_max_v, z_max_v = self.max_velocities
        x_max_a, y_max_a, z_max_a = self.max_accels
        x_r = max(abs(x_r), EPSILON)
        y_r = max(abs(y_r), EPSILON)
        max_v = min(x_max_v / x_r, y_max_v / y_r)
        max_a = min(x_max_a / x_r, y_max_a / y_r)

        if self.scale_per_axis:
            _, toolhead_max_a = move.toolhead.get_max_velocity()
            max_a *= toolhead_max_a / self.xy_hypot_accel

        if z_r:
            z_r = abs(z_r)
            max_v = min(max_v, z_max_v / z_r)
            max_a = min(max_a, z_max_a / z_r)

        move.limit_speed(max_v, max_a)


def load_kinematics(toolhead, config):
    return LimitedCoreXZKinematics(toolhead, config)
