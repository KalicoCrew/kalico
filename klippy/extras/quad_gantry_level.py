# Mechanicaly conforms a moving gantry to the bed with 4 Z steppers
#
# Copyright (C) 2018  Maks Zolin <mzolin@vorondesign.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

from . import probe, z_tilt

# Leveling code for XY rails that are controlled by Z steppers as in:
#
# Z stepper1 ----> O                             O <---- Z stepper2
#                  | * <-- probe1   probe2 --> * |
#                  |                             |
#                  |                             | <--- Y2 rail
#   Y1 rail -----> |                             |
#                  |                             |
#                  |=============================|
#                  |            ^                |
#                  |            |                |
#                  |   X rail --/                |
#                  |                             |
#                  | * <-- probe0   probe3 --> * |
# Z stepper0 ----> O                             O <---- Z stepper3


class QuadGantryLevel:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.retry_helper = z_tilt.RetryHelper(
            config, "Possibly Z motor numbering is wrong"
        )
        self.max_adjust = config.getfloat("max_adjust", 4, above=0)
        self.horizontal_move_z = config.getfloat("horizontal_move_z", 5.0)
        self.probe_helper = probe.ProbePointsHelper(config, self.probe_finalize)
        if len(self.probe_helper.probe_points) != 4:
            raise config.error(
                "Need exactly 4 probe points for quad_gantry_level"
            )

        # Keep a copy of the configured logical point order. The QGL math below
        # expects positions[0], positions[1], positions[2], and positions[3] to
        # represent the same physical gantry corners on every pass, regardless
        # of the order in which those points were physically probed.
        self._logical_probe_points = list(self.probe_helper.probe_points)
        self._normal_probe_order = [0, 1, 2, 3]
        self._reverse_probe_order = [3, 2, 1, 0]
        self._active_probe_order = list(self._normal_probe_order)
        self._qgl_pass_index = 0

        # Optionally alternate the physical probing order between full QGL
        # passes/retries. This avoids repeatedly traversing the same loop around
        # the bed, which can help prevent Bowden tubes, umbilicals, and cable
        # bundles from accumulating twist on large-format machines.
        #
        # The alternating order implementation is inspired by Lazar at Modix3D
        # and the way he has solved this behavior in RepRapFirmware.
        self.alternate_probe_direction = config.getboolean(
            "alternate_probe_direction", False
        )
        self.start_reverse = config.getboolean("start_reverse", False)
        self.report_probe_order = config.getboolean("report_probe_order", False)

        self.z_status = z_tilt.ZAdjustStatus(self.printer)
        self.z_helper = z_tilt.ZAdjustHelper(config, 4)
        self.gantry_corners = config.getlists(
            "gantry_corners", parser=float, seps=(",", "\n"), count=2
        )
        if len(self.gantry_corners) < 2:
            raise config.error(
                "quad_gantry_level requires at least two gantry_corners"
            )

        # Restore the configured point order if a command aborts mid-run.
        self.printer.register_event_handler(
            "gcode:command_error", self._handle_command_error
        )

        # Register QUAD_GANTRY_LEVEL command
        self.gcode = self.printer.lookup_object("gcode")
        self.gcode.register_command(
            "QUAD_GANTRY_LEVEL",
            self.cmd_QUAD_GANTRY_LEVEL,
            desc=self.cmd_QUAD_GANTRY_LEVEL_help,
        )

    cmd_QUAD_GANTRY_LEVEL_help = (
        "Conform a moving, twistable gantry to the shape of a stationary bed"
    )

    def _handle_command_error(self):
        self._restore_logical_probe_order()

    def cmd_QUAD_GANTRY_LEVEL(self, gcmd):
        self.z_status.reset()
        self.retry_helper.start(gcmd)
        self._qgl_pass_index = 0
        self._set_probe_order_for_pass(self._qgl_pass_index)
        self.probe_helper.start_probe(gcmd)

    def _select_probe_order_for_pass(self, pass_index):
        if not self.alternate_probe_direction:
            return list(self._normal_probe_order)

        use_reverse = bool(pass_index % 2)
        if self.start_reverse:
            use_reverse = not use_reverse

        if use_reverse:
            return list(self._reverse_probe_order)
        return list(self._normal_probe_order)

    def _set_probe_order_for_pass(self, pass_index):
        self._active_probe_order = self._select_probe_order_for_pass(pass_index)
        self.probe_helper.probe_points = [
            self._logical_probe_points[index] 
            for index in self._active_probe_order
        ]

        if self.alternate_probe_direction and self.report_probe_order:
            order_text = " -> ".join(
                [str(index) for index in self._active_probe_order]
            )
            self.gcode.respond_info(
                "QGL pass %d probe order: %s" % (pass_index + 1, order_text)
            )

    def _restore_logical_probe_order(self):
        self._active_probe_order = list(self._normal_probe_order)
        self.probe_helper.probe_points = list(self._logical_probe_points)

    def _map_positions_to_logical_order(self, positions):
        # ProbePointsHelper returns positions in the order they were physically
        # probed. Convert them back to the logical order expected by the QGL
        # calculations below.
        if len(positions) != 4 or len(self._active_probe_order) != 4:
            return positions

        logical_positions = [None, None, None, None]
        for measured_position, logical_index in zip(
            positions, self._active_probe_order
        ):
            logical_positions[logical_index] = measured_position

        if any(position is None for position in logical_positions):
            raise self.gcode.error(
                "quad_gantry_level internal error: failed to map probe results"
            )
        return logical_positions

    def _is_retry_done(self, retry_result):
        return (
            (isinstance(retry_result, str) and retry_result == "done")
            or (isinstance(retry_result, float) and retry_result == 0.0)
            or retry_result == 0
        )

    def probe_finalize(self, offsets, positions):
        positions = self._map_positions_to_logical_order(positions)

        try:
            result = self._probe_finalize(offsets, positions)
        except Exception:
            self._restore_logical_probe_order()
            raise

        if self._is_retry_done(result):
            self._restore_logical_probe_order()
        else:
            # ProbePointsHelper will perform another full pass when RetryHelper
            # requests another retry. Prepare the next pass order before giving
            # control back to the helper.
            self._qgl_pass_index += 1
            self._set_probe_order_for_pass(self._qgl_pass_index)

        return result

    def _probe_finalize(self, offsets, positions):
        # Mirror our perspective so the adjustments make sense
        # from the perspective of the gantry
        z_positions = [self.horizontal_move_z - p[2] for p in positions]
        points_message = "Gantry-relative probe points:\n%s\n" % (
            " ".join(
                [
                    "%s: %.6f" % (z_id, z_positions[z_id])
                    for z_id in range(len(z_positions))
                ]
            )
        )
        self.gcode.respond_info(points_message)
        # Calculate slope along X axis between probe point 0 and 3
        ppx0 = [positions[0][0] + offsets[0], z_positions[0]]
        ppx3 = [positions[3][0] + offsets[0], z_positions[3]]
        slope_x_pp03 = self.linefit(ppx0, ppx3)
        # Calculate slope along X axis between probe point 1 and 2
        ppx1 = [positions[1][0] + offsets[0], z_positions[1]]
        ppx2 = [positions[2][0] + offsets[0], z_positions[2]]
        slope_x_pp12 = self.linefit(ppx1, ppx2)
        logging.info(
            "quad_gantry_level f1: %s, f2: %s" % (slope_x_pp03, slope_x_pp12)
        )
        # Calculate gantry slope along Y axis between stepper 0 and 1
        a1 = [
            positions[0][1] + offsets[1],
            self.plot(slope_x_pp03, self.gantry_corners[0][0]),
        ]
        a2 = [
            positions[1][1] + offsets[1],
            self.plot(slope_x_pp12, self.gantry_corners[0][0]),
        ]
        slope_y_s01 = self.linefit(a1, a2)
        # Calculate gantry slope along Y axis between stepper 2 and 3
        b1 = [
            positions[0][1] + offsets[1],
            self.plot(slope_x_pp03, self.gantry_corners[1][0]),
        ]
        b2 = [
            positions[1][1] + offsets[1],
            self.plot(slope_x_pp12, self.gantry_corners[1][0]),
        ]
        slope_y_s23 = self.linefit(b1, b2)
        logging.info(
            "quad_gantry_level af: %s, bf: %s" % (slope_y_s01, slope_y_s23)
        )
        # Calculate z height of each stepper
        z_height = [0, 0, 0, 0]
        z_height[0] = self.plot(slope_y_s01, self.gantry_corners[0][1])
        z_height[1] = self.plot(slope_y_s01, self.gantry_corners[1][1])
        z_height[2] = self.plot(slope_y_s23, self.gantry_corners[1][1])
        z_height[3] = self.plot(slope_y_s23, self.gantry_corners[0][1])

        ainfo = zip(["z", "z1", "z2", "z3"], z_height[0:4])
        apos = " ".join(["%s: %06f" % (x) for x in ainfo])
        self.gcode.respond_info("Actuator Positions:\n" + apos)

        z_ave = sum(z_height) / len(z_height)
        self.gcode.respond_info("Average: %0.6f" % z_ave)
        z_adjust = []
        for z in z_height:
            z_adjust.append(z_ave - z)

        adjust_max = max(z_adjust)
        if adjust_max > self.max_adjust:
            raise self.gcode.error(
                "Aborting quad_gantry_level"
                " required adjustment %0.6f"
                " is greater than max_adjust %0.6f"
                % (adjust_max, self.max_adjust)
            )

        speed = self.probe_helper.get_lift_speed()
        self.z_helper.adjust_steppers(z_adjust, speed)
        return self.z_status.check_retry_result(
            self.retry_helper.check_retry(z_positions)
        )

    def linefit(self, p1, p2):
        if p1[1] == p2[1]:
            # Straight line
            return 0, p1[1]
        m = (p2[1] - p1[1]) / (p2[0] - p1[0])
        b = p1[1] - m * p1[0]
        return m, b

    def plot(self, f, x):
        return f[0] * x + f[1]

    def get_status(self, eventtime):
        return self.z_status.get_status(eventtime)


def load_config(config):
    return QuadGantryLevel(config)
