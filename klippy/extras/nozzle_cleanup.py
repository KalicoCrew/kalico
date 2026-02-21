# Nozzle Cleanup Command
#
# Copyright (C) 2024  Gareth Farrington <gareth@waves.ky>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

from klippy import Printer
from klippy.configfile import ConfigWrapper
from klippy.extras.probe import GcodeNozzleScrubber, PrinterProbe, RetryPolicy
from klippy.gcode import GCodeCommand, GCodeDispatch
from klippy.toolhead import ToolHead


class NozzleCleanupOptions:
    probe_speed: float
    lift_speed: float
    horizontal_speed: float
    sample_retract_dist: float
    samples: int
    pattern_stepover: float
    pattern_x: int
    pattern_y: int

    def __init__(self, config: ConfigWrapper):
        printer: Printer = config.get_printer()
        probe: PrinterProbe = printer.lookup_object("probe")
        self._cfg_probe_speed = config.getfloat("speed", probe.speed, above=0.0)
        self._cfg_lift_speed = config.getfloat(
            "lift_speed", probe.lift_speed, above=0.0
        )
        self._cfg_horizontal_speed = config.getfloat(
            "horizontal_speed", probe.speed, above=0.0
        )
        self._cfg_sample_retract_dist = config.getfloat(
            "sample_retract_dist", probe.sample_retract_dist, above=0.0
        )
        # Nozzle cleanup configuration defaults
        self._cfg_samples: int = config.getint("samples", 3, minval=1)
        self._cfg_pattern_stepover: float = config.getfloat(
            "stepover", 2.0, above=0.0
        )
        self._cfg_pattern_x: int = config.getint("pattern_x", 10)
        self._cfg_pattern_y: int = config.getint("pattern_y", 4)
        # Initialize values from config
        self.probe_speed = self._cfg_probe_speed
        self.lift_speed = self._cfg_lift_speed
        self.horizontal_speed = self._cfg_horizontal_speed
        self.sample_retract_dist = self._cfg_sample_retract_dist
        self.samples = self._cfg_samples
        self.pattern_stepover = self._cfg_pattern_stepover
        self.pattern_x = self._cfg_pattern_x
        self.pattern_y = self._cfg_pattern_y

    def customize(self, gcmd: GCodeCommand):
        """Update settings from a GCode Command instance"""
        self.probe_speed = gcmd.get_float(
            "SPEED", self._cfg_probe_speed, above=0.0
        )
        self.lift_speed = gcmd.get_float(
            "LIFT_SPEED", self._cfg_lift_speed, above=0.0
        )
        self.horizontal_speed = gcmd.get_float(
            "HORIZONTAL_SPEED", self._cfg_horizontal_speed, above=0.0
        )
        self.sample_retract_dist = gcmd.get_float(
            "SAMPLE_RETRACT_DIST", self._cfg_sample_retract_dist, above=0.0
        )
        self.samples = gcmd.get_int("SAMPLES", self._cfg_samples, minval=1)
        self.pattern_stepover = gcmd.get_float(
            "PATTERN_STEPOVER", self._cfg_pattern_stepover, above=0.0
        )
        self.pattern_x = gcmd.get_int(
            "PATTERN_X", self._cfg_pattern_x, minval=1
        )
        self.pattern_y = gcmd.get_int(
            "PATTERN_Y", self._cfg_pattern_y, minval=1
        )

    def get_cleanup_positions(self) -> list[tuple[float, float]]:
        """Generate serpentine pattern of cleanup positions"""
        positions = []
        x_count = abs(self.pattern_x)
        y_count = abs(self.pattern_y)
        x_dir = 1 if self.pattern_x > 0 else -1
        y_dir = 1 if self.pattern_y > 0 else -1
        for row in range(y_count):
            y = row * self.pattern_stepover * y_dir
            if row % 2 == 0:
                for col in range(x_count):
                    x = col * self.pattern_stepover * x_dir
                    positions.append((x, y))
            else:
                for col in range(x_count - 1, -1, -1):
                    x = col * self.pattern_stepover * x_dir
                    positions.append((x, y))
        return positions


class NozzleCleanup:
    def __init__(self, config: ConfigWrapper):
        self.printer: Printer = config.get_printer()
        gcode: GCodeDispatch = self.printer.lookup_object("gcode")
        gcode.register_command(
            "NOZZLE_CLEANUP",
            self.cmd_NOZZLE_CLEANUP,
            desc=self.cmd_NOZZLE_CLEANUP_help,
        )
        self.options = NozzleCleanupOptions(config)
        self.retry_policy: RetryPolicy = RetryPolicy(config)
        self.nozzle_scrubber: GcodeNozzleScrubber = GcodeNozzleScrubber(config)

    def _get_toolhead(self) -> ToolHead:
        return self.printer.lookup_object("toolhead")

    def _get_probe(self) -> PrinterProbe:
        return self.printer.lookup_object("probe")

    def horizontal_move(self, pos: tuple[float, float]):
        self._get_toolhead().manual_move(
            pos + (None,), self.options.horizontal_speed
        )

    def probing_move(self, gcmd: GCodeCommand) -> bool:
        probe = self._get_probe()
        result, is_good = probe.probing_move(self.options.probe_speed, gcmd)
        return is_good

    def retract_move(self):
        z_pos = self._get_toolhead().get_position()[2]
        self._get_toolhead().manual_move(
            [None, None, z_pos + self.options.sample_retract_dist],
            self.options.lift_speed,
        )

    cmd_NOZZLE_CLEANUP_help = "Run nozzle cleanup"

    def cmd_NOZZLE_CLEANUP(self, gcmd: GCodeCommand):
        self.options.customize(gcmd)
        positions = self.options.get_cleanup_positions()
        origin_pos = self._get_toolhead().get_position()
        max_positions = len(positions)
        clean_samples = 0

        for i, position in enumerate(positions):
            # move to the next position (first move is a no-op)
            next_pos = (
                origin_pos[0] + position[0],
                origin_pos[1] + position[1],
            )
            # move / probe / retract
            self.horizontal_move(next_pos)
            is_good: bool = self.probing_move(gcmd)
            self.retract_move()
            if is_good:
                clean_samples += 1
                if clean_samples >= self.options.samples:
                    # success on last consecutive probe
                    gcmd.respond_info("NOZZLE_CLEANUP succeeded")
                    return
            else:
                clean_samples = 0
                # maybe scrub nozzle
                self.nozzle_scrubber.clean_nozzle(gcmd, i + 1, max_positions)

        # if all positions are exhausted raise an error
        raise self.printer.command_error(
            f"NOZZLE_CLEANUP failed after trying {max_positions} positions"
        )


def load_config(config):
    return NozzleCleanup(config)
