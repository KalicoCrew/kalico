# Helper script to adjust bed screws tilt using Z probe
#
# Copyright (C) 2019  Rui Caridade <rui.mcbc@gmail.com>
# Copyright (C) 2021  Matthew Lloyd <github@matthewlloyd.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math

from . import probe

# Screw thread mapping: thread_name -> (pitch, direction)
SCREW_THREAD_MAP = {
    "CW-M3": (0.5, "CW"),
    "CCW-M3": (0.5, "CCW"),
    "CW-M4": (0.7, "CW"),
    "CCW-M4": (0.7, "CCW"),
    "CW-M5": (0.8, "CW"),
    "CCW-M5": (0.8, "CCW"),
    "CW-M6": (1.0, "CW"),
    "CCW-M6": (1.0, "CCW"),
    "CW-M8": (1.25, "CW"),
    "CCW-M8": (1.25, "CCW"),
}


class ScrewsTiltAdjust:
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.screws = []
        self.results = {}
        self.max_diff = None
        self.max_diff_error = False
        # Read config
        for i in range(99):
            prefix = "screw%d" % (i + 1,)
            if config.get(prefix, None) is None:
                break
            screw_coord = config.getfloatlist(prefix, count=2)
            screw_name = "screw at %.3f,%.3f" % screw_coord
            screw_name = config.get(prefix + "_name", screw_name)
            self.screws.append((screw_coord, screw_name))
        if len(self.screws) < 3:
            raise config.error(
                "screws_tilt_adjust: Must have at least three screws"
            )
        # Screw parameters: support both legacy 'screw_thread' and
        # universal 'screw_pitch'/'screw_direction' options.
        screw_thread = config.get("screw_thread", None)
        screw_pitch = config.getfloat("screw_pitch", None, above=0.0)
        screw_direction = config.getchoice(
            "screw_direction", {"CW": "CW", "CCW": "CCW"}, None
        )
        if screw_thread is not None:
            if screw_pitch is not None or screw_direction is not None:
                raise config.error(
                    "screws_tilt_adjust: 'screw_thread' cannot be used "
                    "together with 'screw_pitch' or 'screw_direction'"
                )
            screw_thread_result = SCREW_THREAD_MAP.get(screw_thread.upper())
            if screw_thread_result is None:
                raise config.error(
                    "screws_tilt_adjust: Invalid screw_thread '%s'. "
                    "Accepted values: %s"
                    % (screw_thread, ", ".join(sorted(SCREW_THREAD_MAP.keys())))
                )
            self.screw_pitch, self.screw_direction = screw_thread_result
        else:
            if screw_pitch is None or screw_direction is None:
                raise config.error(
                    "screws_tilt_adjust: Must specify either 'screw_thread' "
                    "or both 'screw_pitch' and 'screw_direction'"
                )
            self.screw_pitch = screw_pitch
            self.screw_direction = screw_direction
        # Initialize ProbePointsHelper
        points = [coord for coord, name in self.screws]
        self.probe_helper = probe.ProbePointsHelper(
            self.config, self.probe_finalize, default_points=points
        )
        self.probe_helper.minimum_points(3)
        # Register command
        self.gcode = self.printer.lookup_object("gcode")
        self.gcode.register_command(
            "SCREWS_TILT_CALCULATE",
            self.cmd_SCREWS_TILT_CALCULATE,
            desc=self.cmd_SCREWS_TILT_CALCULATE_help,
        )

    cmd_SCREWS_TILT_CALCULATE_help = (
        "Tool to help adjust bed leveling "
        "screws by calculating the number "
        "of turns to level it."
    )

    def cmd_SCREWS_TILT_CALCULATE(self, gcmd):
        self.max_diff = gcmd.get_float("MAX_DEVIATION", None)
        # Option to force all turns to be in the given direction (CW or CCW)
        direction = gcmd.get("DIRECTION", default=None)
        if direction is not None:
            direction = direction.upper()
            if direction not in ("CW", "CCW"):
                raise gcmd.error(
                    "Error on '%s': DIRECTION must be either CW or CCW"
                    % (gcmd.get_commandline(),)
                )
        self.direction = direction
        self.probe_helper.start_probe(gcmd)

    def get_status(self, eventtime):
        return {
            "error": self.max_diff_error,
            "max_deviation": self.max_diff,
            "results": self.results,
        }

    def probe_finalize(self, offsets, positions):
        self.results = {}
        self.max_diff_error = False
        is_clockwise_thread = self.screw_direction == "CW"
        screw_diff = []
        # Process the read Z values
        if self.direction is not None:
            # Lowest or highest screw is the base position used for comparison
            use_max = (is_clockwise_thread and self.direction == "CW") or (
                not is_clockwise_thread and self.direction == "CCW"
            )
            min_or_max = max if use_max else min
            i_base, z_base = min_or_max(
                enumerate([pos[2] for pos in positions]), key=lambda v: v[1]
            )
        else:
            # First screw is the base position used for comparison
            i_base, z_base = 0, positions[0][2]
        # Provide the user some information on how to read the results
        self.gcode.respond_info(
            "01:20 means 1 full turn and 20 minutes, "
            "CW=clockwise, CCW=counter-clockwise"
        )
        for i, screw in enumerate(self.screws):
            z = positions[i][2]
            coord, name = screw
            if i == i_base:
                # Show the results
                self.gcode.respond_info(
                    "%s : x=%.1f, y=%.1f, z=%.5f"
                    % (name + " (base)", coord[0], coord[1], z)
                )
                sign = "CW" if is_clockwise_thread else "CCW"
                self.results["screw%d" % (i + 1,)] = {
                    "z": z,
                    "sign": sign,
                    "adjust": "00:00",
                    "is_base": True,
                }
            else:
                # Calculate how knob must be adjusted for other positions
                diff = z_base - z
                screw_diff.append(abs(diff))
                if abs(diff) < 0.001:
                    adjust = 0
                else:
                    adjust = diff / self.screw_pitch
                if is_clockwise_thread:
                    sign = "CW" if adjust >= 0 else "CCW"
                else:
                    sign = "CCW" if adjust >= 0 else "CW"
                adjust = abs(adjust)
                full_turns = math.trunc(adjust)
                decimal_part = adjust - full_turns
                minutes = round(decimal_part * 60, 0)
                # Show the results
                self.gcode.respond_info(
                    "%s : x=%.1f, y=%.1f, z=%.5f : adjust %s %02d:%02d"
                    % (name, coord[0], coord[1], z, sign, full_turns, minutes)
                )
                self.results["screw%d" % (i + 1,)] = {
                    "z": z,
                    "sign": sign,
                    "adjust": "%02d:%02d" % (full_turns, minutes),
                    "is_base": False,
                }
        if self.max_diff and any((d > self.max_diff) for d in screw_diff):
            self.max_diff_error = True
            raise self.gcode.error(
                "bed level exceeds configured limits ({}mm)! "
                "Adjust screws and restart print.".format(self.max_diff)
            )


def load_config(config):
    return ScrewsTiltAdjust(config)
