# Limits XYZ accelerations and velocities on cartesian robots
# with per-axis limits for velocity and acceleration
#
# Copyright (C) 2020-2021  Mael Kerbiriou <piezo.wdimd@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Usage:
# Copying this file under `klipper/klippy/kinematics/` should be enough (click
# the `raw` button on github, then save as)
# Then your config's [printer] should look like:
# [printer]
# kinematics: limited_cartesian
# max_velocity: 500 # Hypotenuse of the two values bellow
# max_x_velocity: 400
# max_y_velocity: 300
# max_z_velocity: 5
# max_accel: 1500 # Default acceleration of your choice
# max_x_accel: 12000
# max_y_accel: 9000
# max_z_accel: 100
# scale_xy_accel: [True/False, default False]
#
# max_velocity is usually the hypotenuses of X and Y velocity, For example:
# with max_x_velocity = 300 and max_y_velocity = 400, the recommended value
# is max_velocity = 500.
#
# If scale_xy_accel is False, `max_accel`, set by M204 or
# SET_VELOCITY_LIMIT, acts as a third limit. In that case, this module
# doesn't apply limitations to moves with an acceleration lower than
# max_x_accel and max_y_accel.
# When scale_xy_accel is True, max_x_accel and max_y_accel are scaled by
# the ratio of the dynamically set acceleration and the hypotenuse of
# max_x_accel and max_y_accel, as reported from `SET_KINEMATICS_LIMIT`.
# This means that the actual acceleration will always depend on the
# direction.
# For example, these settings:
#
# [printer]
# max_x_accel: 12000
# max_y_accel: 9000
# scale_xy_accel: true
#
# SET_KINEMATICS_LIMIT will report a maximum acceleration of 15000 mm/s^2
# on 37° diagonals. If the slicer emit `M204 S3000` (3000 mm/s^2 accel).
# On these 37° and 143° diagonals, the toolhead will accelerate at 3000 mm/s^2.
# On the X axis, the acceleration will be  12000 * 3000 / 15000 = 2400 mm/s^2,
# and 18000 mm/s^2 for pure Y moves.


import sys
from math import hypot, atan2, pi

from . import cartesian

EPSILON = sys.float_info.epsilon
SQRT2 = hypot(1.0, 1.0)


class LimitedCartKinematics(cartesian.CartKinematics):
    def __init__(self, toolhead, config):
        cartesian.CartKinematics.__init__(self, toolhead, config)
        # Setup y axis limits
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.config_max_velocity = max_velocity
        self.config_max_accel = max_accel
        self.max_velocities = [
            config.getfloat(
                f"max_{ax}_velocity",
                SQRT2 * max_velocity,
                above=0.0,
                maxval=max_velocity,
            )
            for ax in "xyz"
        ]
        self.max_accels = [
            config.getfloat(
                f"max_{ax}_accel", max_accel, above=0.0, maxval=max_accel
            )
            for ax in "xyz"
        ]
        self.xy_hypot_accel = hypot(*self.max_accels[:2])
        self.scale_per_axis = config.getboolean("scale_xy_accel", False)
        config.get_printer().lookup_object("gcode").register_command(
            "SET_KINEMATICS_LIMIT",
            self.cmd_SET_KINEMATICS_LIMIT,
            desc=self.cmd_SET_KINEMATICS_LIMIT_help,
        )
        self.supports_dual_carriage = True

    cmd_SET_KINEMATICS_LIMIT_help = "Set/get cartesian per axis velocity limits"

    def cmd_SET_KINEMATICS_LIMIT(self, gcmd):
        self.max_velocities = [
            gcmd.get_float(f"{ax}_VELOCITY", max_v, above=0.0)
            for max_v, ax in zip(self.max_velocities, "XYZ")
        ]
        self.max_accels = [
            gcmd.get_float(f"{ax}_ACCEL", max_a, above=0.0)
            for max_a, ax in zip(self.max_accels, "XYZ")
        ]
        self.xy_hypot_accel = hypot(*self.max_accels[:2])
        self.scale_per_axis = bool(
            gcmd.get_int("SCALE", self.scale_per_axis, minval=0, maxval=1)
        )

        if gcmd.get_command_parameters():
            return
        msg = [
            f"x,y,z max_velocities: {self.max_velocities!r}",
            f"x,y,z max_accels: {self.max_accels!r}",
        ]
        if self.scale_per_axis:
            msg.append(
                "Per axis accelerations limits scale with current acceleration."
            )
        else:
            msg.append(
                "Per axis accelerations limits are independent of current acceleration."
            )
        max_velocity = hypot(self.max_velocities[0], self.max_velocities[1])
        max_v_angle = atan2(self.max_velocities[1], self.max_velocities[0])
        msg.append(
            f"Maximum XY velocity of {max_velocity:.1f} mm/s reached on {max_v_angle * 180 / pi:.0f}° diagonals."
        )
        max_accel = hypot(self.max_accels[0], self.max_accels[1])
        max_a_angle = atan2(self.max_accels[1], self.max_accels[0])
        msg.append(
            f"Maximum XY acceleration of {max_accel:.1f} mm/s reached on {max_a_angle * 180 / pi:.0f}° diagonals."
        )
        gcmd.respond_info("\n".join(msg), log=False)

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
    return LimitedCartKinematics(toolhead, config)
