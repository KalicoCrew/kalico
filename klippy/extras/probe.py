# Z-Probe support
#
# Copyright (C) 2017-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import logging
import math
from enum import IntEnum
from typing import Optional, Union

from klippy import Printer, pins
from klippy.configfile import ConfigWrapper
from klippy.extras.gcode_macro import Template
from klippy.gcode import GCodeCommand
from klippy.toolhead import ToolHead

from . import manual_probe

HINT_TIMEOUT = """
If the probe did not move far enough to trigger, then
consider reducing the Z axis minimum position so the probe
can travel further (the Z minimum position can be negative).
"""


class RetryStrategy(IntEnum):
    # Probe strategy constants
    FAIL = 0  # bad probes fail with a command error
    IGNORE = 1  # bad probes are ignored
    RETRY = 2  # bad probes are retried in the same location, assumes
    # no fouling
    CIRCLE = 3  # bad probes are retried in a circular pattern to avoid


MAX_CIRCLE_RETRIES: int = 6 * 3


class RetryPolicy:
    retry_speed: float
    bad_probe_retries: int
    bad_probe_strategy: RetryStrategy
    pattern_spacing: float

    def __init__(self, config: ConfigWrapper):
        speed = config.getfloat("speed", 5.0, above=0.0)
        self._cfg_retry_speed: float = config.getfloat(
            "retry_speed", speed, above=0.0
        )
        # Probe retry configuration
        self._cfg_bad_probe_strategy = config.getchoice(
            "bad_probe_strategy",
            {strat.name.lower(): strat for strat in RetryStrategy},
            RetryStrategy.RETRY.name.lower(),
        )
        max_retries = None
        if self._cfg_bad_probe_strategy is RetryStrategy.CIRCLE:
            max_retries = MAX_CIRCLE_RETRIES
        self._cfg_bad_probe_retries = config.getint(
            "bad_probe_retries", 6, minval=0, maxval=max_retries
        )
        self._cfg_pattern_spacing = config.getfloat(
            "pattern_spacing", 2.0, above=0.0
        )
        # initialize values from config
        self.retry_speed = self._cfg_retry_speed
        self.bad_probe_strategy = self._cfg_bad_probe_strategy
        self.bad_probe_retries = self._cfg_bad_probe_retries
        self.pattern_spacing = self._cfg_pattern_spacing

    # update the settings from a GCode Command instance
    def customize(self, gcmd: GCodeCommand):
        self.retry_speed = gcmd.get_float(
            "RETRY_SPEED", self._cfg_retry_speed, above=0.0
        )
        strategy_name = gcmd.get(
            "BAD_PROBE_STRATEGY", self._cfg_bad_probe_strategy.name
        )
        self.bad_probe_strategy = RetryStrategy[strategy_name.upper()]
        max_retries = None
        if self.bad_probe_strategy is RetryStrategy.CIRCLE:
            max_retries = MAX_CIRCLE_RETRIES
        self.bad_probe_retries = gcmd.get_int(
            "BAD_PROBE_RETRIES",
            self._cfg_bad_probe_retries,
            minval=0,
            maxval=max_retries,
        )
        self.pattern_spacing = gcmd.get_float(
            "PATTERN_SPACING", self._cfg_pattern_spacing, above=0.0
        )


class GcodeNozzleScrubberConfig:
    scrubbing_frequency: int = 0
    template: Template

    def __init__(self, config: ConfigWrapper):
        gcode_macro = config.get_printer().load_object(config, "gcode_macro")
        self.template = gcode_macro.load_template(
            config, "nozzle_scrubber_gcode", ""
        )
        self.scrubbing_frequency = self._cfg_scrubbing_frequency = (
            config.getint("scrubbing_frequency", 0, minval=0)
        )

    def customize(self, gcmd: GCodeCommand):
        self.scrubbing_frequency = gcmd.get_int(
            "SCRUBBING_FREQUENCY", self._cfg_scrubbing_frequency, minval=0
        )


class GcodeNozzleScrubber:
    def __init__(
        self,
        config: ConfigWrapper,
    ):
        self._printer: Printer = config.get_printer()
        self.config = GcodeNozzleScrubberConfig(config)

    def clean_nozzle(self, gcmd: GCodeCommand, attempt, retries):
        self.config.customize(gcmd)
        if not self.config.template:
            return
        if attempt == 0 or self.config.scrubbing_frequency <= 0:
            return
        if attempt % self.config.scrubbing_frequency != 0:
            return
        toolhead = self._printer.lookup_object("toolhead")
        start_pos = toolhead.get_position()
        context = self.config.template.create_template_context()
        context["params"] = {
            "ATTEMPT": attempt,
            "RETRIES": retries,
        }
        self.config.template.run_gcode_from_command(context)
        end_pos = toolhead.get_position()
        if not start_pos[:3] == end_pos[:3]:
            self._printer.command_error(
                "Nozzle Scrubber GCode did not return to the start position. "
                "(Hint: Use RESTORE_GCODE_STATE MOVE=1)"
            )


class ProbeRetryState:
    @staticmethod
    def _build_hexagonal_offsets(
        distance_step: float,
    ) -> list[tuple[float, float]]:
        lookup = [(0.0, 0.0)]
        max_rings = 2
        size = distance_step / math.sqrt(3)
        # Hexagonal direction vectors in axial coordinates
        directions = [(1, 0), (1, -1), (0, -1), (-1, 0), (-1, 1), (0, 1)]
        for ring in range(1, max_rings + 1):
            # Start at ring steps in direction 4 (up-left)
            q, r = -ring, ring
            # Walk around the hexagon ring
            for direction in range(6):
                for step in range(ring):
                    # Convert axial to Cartesian
                    x = size * (math.sqrt(3) * q + math.sqrt(3) / 2 * r)
                    y = size * (3.0 / 2.0 * r)
                    lookup.append((x, y))
                    # Move to next hexagonal cell
                    dq, dr = directions[direction]
                    q, r = q + dq, r + dr
        return lookup

    def __init__(self, pos: tuple[float, float], retry_policy: RetryPolicy):
        self.retry_policy = retry_policy
        self._bad_probe_count = 0
        self._start_pos = pos
        self._fouled_count = 0
        # Build circular lookup based on pattern spacing
        self._circle_lookup = []
        logging.info(f"retry policy={self.retry_policy.bad_probe_strategy}")
        if self.retry_policy.bad_probe_strategy is RetryStrategy.CIRCLE:
            self._circle_lookup = self._build_hexagonal_offsets(
                retry_policy.pattern_spacing
            )

    def has_retries_remaining(self) -> bool:
        return self._bad_probe_count <= self.retry_policy.bad_probe_retries

    def reset(self):
        self._bad_probe_count = 0

    def get_attempt(self):
        return self._bad_probe_count

    def evaluate_probe(self, is_good: Optional[bool], gcmd) -> bool:
        """
        Evaluate probe result based on strategy.
        Returns True if probe should be accepted.
        Returns False if probe should be retried.
        Raises error if strategy is FAIL or retries exhausted.
        """
        if (
            is_good
            or self.retry_policy.bad_probe_strategy is RetryStrategy.IGNORE
        ):
            return True
        if self.retry_policy.bad_probe_strategy is RetryStrategy.FAIL:
            raise gcmd.error("Probe failed because it was deemed bad quality")
        self._bad_probe_count += 1
        if self.retry_policy.bad_probe_strategy is RetryStrategy.CIRCLE:
            self._fouled_count += 1
        if not self.has_retries_remaining():
            raise gcmd.error(
                f"Probing failed after {self._bad_probe_count} bad probes"
            )
        # Probe rejected, inform user and retry
        gcmd.respond_info(
            "Bad probe detected. Retrying "
            f"({self._bad_probe_count}/{self.retry_policy.bad_probe_retries})..."
        )
        return False

    def get_position(self) -> tuple[float, float, None]:
        if self.retry_policy.bad_probe_strategy is not RetryStrategy.CIRCLE:
            return self._start_pos[0], self._start_pos[1], None
        x, y = self._circle_lookup[self._fouled_count]
        return self._start_pos[0] + x, self._start_pos[1] + y, None

    def get_bad_probe_count(self):
        return self._bad_probe_count


class RetrySession:
    def __init__(self, config: ConfigWrapper):
        self.retry_policy = RetryPolicy(config)
        self._scrubber: GcodeNozzleScrubber = GcodeNozzleScrubber(config)
        self._point_lookup: dict[tuple[float, float], ProbeRetryState] = {}
        self._pos: Optional[tuple[float, float]] = None
        self._quantized_pos: Optional[tuple[float, float]] = None
        self._gcmd: Optional[GCodeCommand] = None
        self._retry_state: Optional[ProbeRetryState] = None

    @staticmethod
    def _quantize_position(pos: tuple[float, float]) -> tuple[float, float]:
        return round(pos[0], 1), round(pos[1], 1)

    def start(self, gcmd: GCodeCommand):
        self.retry_policy.customize(gcmd)
        self._gcmd = gcmd

    def end(self):
        self._gcmd = self._pos = self._quantized_pos = self._retry_state = None
        self._point_lookup.clear()

    def set_position(self, pos: list[float]) -> None:
        """Set the current ideal position being probed"""
        self._pos = (pos[0], pos[1])
        self._quantized_pos = self._quantize_position((pos[0], pos[1]))
        if self._quantized_pos not in self._point_lookup:
            self._point_lookup[self._quantized_pos] = ProbeRetryState(
                self._pos, self.retry_policy
            )
        self._retry_state = self._point_lookup[self._quantized_pos]

    def get_position(self) -> tuple[float, float]:
        return self._pos

    def get_probe_position(self) -> tuple[float, float, None]:
        """Get the actual probe position for the current 'ideal' position"""
        return self._retry_state.get_position()

    def can_retry(self) -> bool:
        return self._retry_state.has_retries_remaining()

    def get_bad_probe_count(self):
        return self._retry_state.get_bad_probe_count()

    def evaluate_probe(self, is_good: bool) -> bool:
        return self._retry_state.evaluate_probe(is_good, self._gcmd)

    def scrub_nozzle(self):
        """Scrub nozzle based on attempts at the current position"""
        self._scrubber.clean_nozzle(
            self._gcmd,
            self._retry_state.get_attempt(),
            self.retry_policy.bad_probe_retries,
        )

    def reset_all(self):
        for pos in self._point_lookup.values():
            pos.reset()

    def set_retry_strategy(self, retry_strategy: RetryStrategy) -> None:
        self.retry_policy.bad_probe_strategy = retry_strategy


class PrinterProbe:
    def __init__(self, config, mcu_probe):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.mcu_probe = mcu_probe
        self.speed = config.getfloat("speed", 5.0, above=0.0)
        self.retry_speed: float = config.getfloat(
            "retry_speed", self.speed, above=0.0
        )
        self.lift_speed = config.getfloat("lift_speed", self.speed, above=0.0)
        self.x_offset = config.getfloat("x_offset", 0.0)
        self.y_offset = config.getfloat("y_offset", 0.0)
        self.z_offset = config.getfloat("z_offset")
        self.drop_first_result = config.getboolean("drop_first_result", False)
        self.probe_calibrate_z = 0.0
        self.multi_probe_pending = False
        self.last_state = False
        self.last_z_result = 0.0
        self.was_last_result_good = False
        self.gcode_move = self.printer.load_object(config, "gcode_move")
        self.retry_session = RetrySession(config)
        # Infer Z position to move to during a probe
        if config.has_section("stepper_z"):
            zconfig = config.getsection("stepper_z")
            self.z_position = zconfig.getfloat(
                "position_min", 0.0, note_valid=False
            )
        else:
            pconfig = config.getsection("printer")
            self.z_position = pconfig.getfloat(
                "minimum_z_position", 0.0, note_valid=False
            )
        # Multi-sample support (for improved accuracy)
        self.sample_count = config.getint("samples", 1, minval=1)
        self.sample_retract_dist = config.getfloat(
            "sample_retract_dist", 2.0, above=0.0
        )
        atypes = ["median", "average"]
        self.samples_result = config.getchoice(
            "samples_result", atypes, "average"
        )
        self.samples_tolerance = config.getfloat(
            "samples_tolerance", 0.100, minval=0.0
        )
        self.samples_retries = config.getint(
            "samples_tolerance_retries", 0, minval=0
        )
        # Register z_virtual_endstop pin
        self.printer.lookup_object("pins").register_chip("probe", self)
        # Register homing event handlers
        self.printer.register_event_handler(
            "homing:homing_move_begin", self._handle_homing_move_begin
        )
        self.printer.register_event_handler(
            "homing:homing_move_end", self._handle_homing_move_end
        )
        self.printer.register_event_handler(
            "homing:home_rails_begin", self._handle_home_rails_begin
        )
        self.printer.register_event_handler(
            "homing:home_rails_end", self._handle_home_rails_end
        )
        self.printer.register_event_handler(
            "gcode:command_error", self._handle_command_error
        )
        # Register PROBE/QUERY_PROBE commands
        self.gcode = self.printer.lookup_object("gcode")
        self.gcode.register_command(
            "PROBE", self.cmd_PROBE, desc=self.cmd_PROBE_help
        )
        self.gcode.register_command(
            "QUERY_PROBE", self.cmd_QUERY_PROBE, desc=self.cmd_QUERY_PROBE_help
        )
        self.gcode.register_command(
            "PROBE_CALIBRATE",
            self.cmd_PROBE_CALIBRATE,
            desc=self.cmd_PROBE_CALIBRATE_help,
        )
        self.gcode.register_command(
            "PROBE_ACCURACY",
            self.cmd_PROBE_ACCURACY,
            desc=self.cmd_PROBE_ACCURACY_help,
        )
        self.gcode.register_command(
            "Z_OFFSET_APPLY_PROBE",
            self.cmd_Z_OFFSET_APPLY_PROBE,
            desc=self.cmd_Z_OFFSET_APPLY_PROBE_help,
        )

    def _handle_homing_move_begin(self, hmove):
        if self.mcu_probe in hmove.get_mcu_endstops():
            self.mcu_probe.probe_prepare(hmove)

    def _handle_homing_move_end(self, hmove):
        if self.mcu_probe in hmove.get_mcu_endstops():
            self.mcu_probe.probe_finish(hmove)

    def _handle_home_rails_begin(self, homing_state, rails):
        endstops = [es for rail in rails for es, name in rail.get_endstops()]
        if self.mcu_probe in endstops:
            self.multi_probe_begin(always_restore_toolhead=True)

    def _handle_home_rails_end(self, homing_state, rails):
        endstops = [es for rail in rails for es, name in rail.get_endstops()]
        if self.mcu_probe in endstops:
            self.multi_probe_end()

    def _handle_command_error(self):
        try:
            self.multi_probe_end()
        except:
            logging.exception("Multi-probe end")

    def multi_probe_begin(self, always_restore_toolhead=False):
        try:
            self.mcu_probe.multi_probe_begin(always_restore_toolhead)
        except TypeError:
            self.mcu_probe.multi_probe_begin()
        self.multi_probe_pending = True

    def multi_probe_end(self):
        if self.multi_probe_pending:
            self.multi_probe_pending = False
            self.mcu_probe.multi_probe_end()

    def setup_pin(self, pin_type, pin_params):
        if pin_type != "endstop" or pin_params["pin"] != "z_virtual_endstop":
            raise pins.error("Probe virtual endstop only useful as endstop pin")
        if pin_params["invert"] or pin_params["pullup"]:
            raise pins.error("Can not pullup/invert probe virtual endstop")
        return self.mcu_probe

    def get_lift_speed(self, gcmd=None):
        if gcmd is not None:
            return gcmd.get_float("LIFT_SPEED", self.lift_speed, above=0.0)
        return self.lift_speed

    def get_offsets(self):
        return self.x_offset, self.y_offset, self.z_offset

    def probing_move(
        self, speed, gcmd: GCodeCommand
    ) -> tuple[list[float], bool]:
        toolhead = self.printer.lookup_object("toolhead")
        curtime = self.printer.get_reactor().monotonic()
        if "z" not in toolhead.get_status(curtime)["homed_axes"]:
            raise self.printer.command_error("Must home before probe")
        pos = toolhead.get_position()
        pos[2] = self.z_position
        try:
            result = self.mcu_probe.probing_move(pos, speed, gcmd)
        except self.printer.command_error as e:
            reason = str(e)
            if "Timeout during endstop homing" in reason:
                reason += HINT_TIMEOUT
            raise self.printer.command_error(reason)
        # Normalize return value to (epos, is_good)
        if isinstance(result, tuple) and len(result) == 2:
            epos: list[float] = result[0]
            is_good: bool = result[1]
            return epos, is_good
        else:
            return result, True

    def _probe(
        self, speed, gcmd: GCodeCommand
    ) -> tuple[list[float], Optional[bool]]:
        toolhead = self.printer.lookup_object("toolhead")
        pos = toolhead.get_position()
        epos, is_good = self.probing_move(speed, gcmd)
        # get z compensation from axis_twist_compensation
        axis_twist_compensation = self.printer.lookup_object(
            "axis_twist_compensation", None
        )
        z_compensation = 0
        if axis_twist_compensation is not None:
            z_compensation = axis_twist_compensation.get_z_compensation_value(
                pos
            )
        # add z compensation to probe position
        epos[2] += z_compensation
        self.gcode.respond_info(
            "probe at %.3f,%.3f is z=%.6f" % (epos[0], epos[1], epos[2])
        )
        return epos[:3], is_good

    def _move(self, coord, speed):
        self.printer.lookup_object("toolhead").manual_move(coord, speed)

    def _calc_mean(self, positions):
        count = float(len(positions))
        return [sum([pos[i] for pos in positions]) / count for i in range(3)]

    def _calc_median(self, positions):
        z_sorted = sorted(positions, key=(lambda p: p[2]))
        middle = len(positions) // 2
        if (len(positions) & 1) == 1:
            # odd number of samples
            return z_sorted[middle]
        # even number of samples
        return self._calc_mean(z_sorted[middle - 1 : middle + 1])

    @property
    def _drop_first_result(self):
        if hasattr(self, "drop_first_result"):
            return self.drop_first_result
        return False

    def _discard_first_result(
        self, speed: float, retry_session: RetrySession, gcmd: GCodeCommand
    ):
        if self._drop_first_result:
            pos, is_good = self._probe(speed, gcmd)
            # marks the initial probing location as fouled if needed
            retry_session.evaluate_probe(is_good)
            self._retract(gcmd)

    # Raise the toolhead at the current x/y location
    def _retract(self, gcmd: GCodeCommand):
        sample_retract_dist = gcmd.get_float(
            "SAMPLE_RETRACT_DIST", self.sample_retract_dist, above=0.0
        )
        lift_speed = self.get_lift_speed(gcmd)
        toolhead: ToolHead = self.printer.lookup_object("toolhead")
        pos = toolhead.get_position()
        self._move([None, None, pos[2] + sample_retract_dist], lift_speed)

    def _run_probe_with_retries(
        self, speed: float, retry_session: RetrySession, gcmd: GCodeCommand
    ):
        """Probe for a single good result with retries based on strategy"""
        while retry_session.can_retry():
            self._move(retry_session.get_probe_position(), self.retry_speed)
            # Probe position
            pos, is_good = self._probe(speed, gcmd)
            if retry_session.evaluate_probe(is_good):
                # return the x/y of the original requested location
                return retry_session.get_position() + (pos[2],)
            # Probe was rejected, retry
            self._retract(gcmd)
            retry_session.scrub_nozzle()
        attempts = retry_session.get_bad_probe_count()
        raise gcmd.error(
            f"Probing failed to collect a good sample after {attempts} attempts"
        )

    def run_probe(
        self, gcmd: GCodeCommand, retry_session: Optional[RetrySession] = None
    ) -> list[float]:
        speed = gcmd.get_float("PROBE_SPEED", self.speed, above=0.0)
        sample_count = gcmd.get_int("SAMPLES", self.sample_count, minval=1)
        samples_tolerance = gcmd.get_float(
            "SAMPLES_TOLERANCE", self.samples_tolerance, minval=0.0
        )
        samples_retries = gcmd.get_int(
            "SAMPLES_TOLERANCE_RETRIES", self.samples_retries, minval=0
        )
        samples_result = gcmd.get("SAMPLES_RESULT", self.samples_result)
        must_notify_multi_probe = not self.multi_probe_pending
        if must_notify_multi_probe:
            self.multi_probe_begin(always_restore_toolhead=True)
        # Initialize probe retry state
        if retry_session is None:
            local_retry_session = self.retry_session
            self.retry_session.start(gcmd)
            toolhead = self.printer.lookup_object("toolhead")
            self.retry_session.set_position(toolhead.get_position())
        else:
            local_retry_session: RetrySession = retry_session
        # save X and Y position
        toolhead = self.printer.lookup_object("toolhead")
        request_pos = toolhead.get_position()[:2]
        # Handle drop first result, perform a probe and throw it away
        if self._drop_first_result:
            self._probe(speed, gcmd)
            self._retract(gcmd)
        retries = 0
        positions = []
        self._discard_first_result(speed, local_retry_session, gcmd)
        while len(positions) < sample_count:
            # Probe position with retries
            pos = self._run_probe_with_retries(speed, local_retry_session, gcmd)
            positions.append(pos)
            # Check samples tolerance
            z_positions = [p[2] for p in positions]
            if max(z_positions) - min(z_positions) > samples_tolerance:
                if retries >= samples_retries:
                    raise gcmd.error("Probe samples exceed samples_tolerance")
                gcmd.respond_info("Probe samples exceed tolerance. Retrying...")
                retries += 1
                positions = []
            # Retract
            if len(positions) < sample_count:
                self._retract(gcmd)
        if must_notify_multi_probe:
            self.multi_probe_end()
        if retry_session is None:
            self.retry_session.end()
        # Calculate and return result
        if samples_result == "median":
            return self._calc_median(positions)
        return self._calc_mean(positions)

    cmd_PROBE_help = "Probe Z-height at current XY position"

    def cmd_PROBE(self, gcmd: GCodeCommand):
        pos = self.run_probe(gcmd)
        gcmd.respond_info("Result is z=%.6f" % (pos[2],))
        self.last_z_result = pos[2]
        home = gcmd.get("HOME", default="").lower()
        if home == "z":
            toolhead: ToolHead = self.printer.lookup_object("toolhead")
            toolhead.get_last_move_time()
            toolhead_pos = toolhead.get_position()
            toolhead_pos[2] = toolhead_pos[2] - self.last_z_result
            toolhead.set_position(toolhead_pos, homing_axes=[2])

    cmd_QUERY_PROBE_help = "Return the status of the z-probe"

    def cmd_QUERY_PROBE(self, gcmd):
        toolhead = self.printer.lookup_object("toolhead")
        print_time = toolhead.get_last_move_time()
        res = self.mcu_probe.query_endstop(print_time)
        self.last_state = res
        gcmd.respond_info("probe: %s" % (["open", "TRIGGERED"][not not res],))

    def get_status(self, eventtime):
        return {
            "name": self.name,
            "last_query": self.last_state,
            "last_z_result": self.last_z_result,
        }

    cmd_PROBE_ACCURACY_help = "Probe Z-height accuracy at current XY position"

    def cmd_PROBE_ACCURACY(self, gcmd: GCodeCommand):
        speed = gcmd.get_float("PROBE_SPEED", self.speed, above=0.0)
        lift_speed = self.get_lift_speed(gcmd)
        sample_count = gcmd.get_int("SAMPLES", 10, minval=1)
        sample_retract_dist = gcmd.get_float(
            "SAMPLE_RETRACT_DIST", self.sample_retract_dist, above=0.0
        )
        toolhead = self.printer.lookup_object("toolhead")
        pos = toolhead.get_position()
        gcmd.respond_info(
            "PROBE_ACCURACY at X:%.3f Y:%.3f Z:%.3f"
            " (samples=%d retract=%.3f"
            " speed=%.1f lift_speed=%.1f)\n"
            % (
                pos[0],
                pos[1],
                pos[2],
                sample_count,
                sample_retract_dist,
                speed,
                lift_speed,
            )
        )
        # Probe bed sample_count times
        self.multi_probe_begin(always_restore_toolhead=True)
        # Initialize probe retry state
        self.retry_session.start(gcmd)
        # force FAIL behavior for PROBE_ACCURACY, never accept bad probes
        self.retry_session.set_retry_strategy(RetryStrategy.FAIL)
        self.retry_session.set_position(toolhead.get_position())
        # Discard first probe if required
        if self._drop_first_result:
            self._probe(speed, gcmd)
            self._retract(gcmd)
        positions = []
        self._discard_first_result(speed, self.retry_session, gcmd)
        while len(positions) < sample_count:
            # Probe position
            pos = self._run_probe_with_retries(speed, self.retry_session, gcmd)
            positions.append(pos)
            # Retract
            self._retract(gcmd)
        self.multi_probe_end()
        self.retry_session.end()
        # Calculate maximum, minimum and average values
        max_value = max([p[2] for p in positions])
        min_value = min([p[2] for p in positions])
        range_value = max_value - min_value
        avg_value = self._calc_mean(positions)[2]
        median = self._calc_median(positions)[2]
        # calculate the standard deviation
        deviation_sum = 0
        for i in range(len(positions)):
            deviation_sum += pow(positions[i][2] - avg_value, 2.0)
        sigma = (deviation_sum / len(positions)) ** 0.5
        # Show information
        gcmd.respond_info(
            "probe accuracy results: maximum %.6f, minimum %.6f, range %.6f, "
            "average %.6f, median %.6f, standard deviation %.6f"
            % (max_value, min_value, range_value, avg_value, median, sigma)
        )

    def probe_calibrate_finalize(self, kin_pos):
        if kin_pos is None:
            return
        z_offset = self.probe_calibrate_z - kin_pos[2]
        self.gcode.respond_info(
            "%s: z_offset: %.3f\n"
            "The SAVE_CONFIG command will update the printer config file\n"
            "with the above and restart the printer." % (self.name, z_offset)
        )
        configfile = self.printer.lookup_object("configfile")
        configfile.set(self.name, "z_offset", "%.3f" % (z_offset,))

    cmd_PROBE_CALIBRATE_help = "Calibrate the probe's z_offset"

    def cmd_PROBE_CALIBRATE(self, gcmd):
        manual_probe.verify_no_manual_probe(self.printer)
        # Perform initial probe
        lift_speed = self.get_lift_speed(gcmd)
        curpos = self.run_probe(gcmd)
        # Move away from the bed
        self.probe_calibrate_z = curpos[2]
        curpos[2] += 5.0
        self._move(curpos, lift_speed)
        # Move the nozzle over the probe point
        curpos[0] += self.x_offset
        curpos[1] += self.y_offset
        self._move(curpos, self.speed)
        # Start manual probe
        manual_probe.ManualProbeHelper(
            self.printer, gcmd, self.probe_calibrate_finalize
        )

    def cmd_Z_OFFSET_APPLY_PROBE(self, gcmd):
        offset = self.gcode_move.get_status()["homing_origin"].z
        configfile = self.printer.lookup_object("configfile")
        if offset == 0:
            self.gcode.respond_info("Nothing to do: Z Offset is 0")
        else:
            new_calibrate = self.z_offset - offset
            self.gcode.respond_info(
                "%s: z_offset: %.3f\n"
                "The SAVE_CONFIG command will update the printer config file\n"
                "with the above and restart the printer."
                % (self.name, new_calibrate)
            )
            configfile.set(self.name, "z_offset", "%.3f" % (new_calibrate,))

    cmd_Z_OFFSET_APPLY_PROBE_help = "Adjust the probe's z_offset"


# Endstop wrapper that enables probe specific features
class ProbeEndstopWrapper:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.position_endstop = config.getfloat("z_offset")
        self.stow_on_each_sample = config.getboolean(
            "deactivate_on_each_sample", True
        )
        gcode_macro = self.printer.load_object(config, "gcode_macro")
        self.activate_gcode = gcode_macro.load_template(
            config, "activate_gcode", ""
        )
        self.deactivate_gcode = gcode_macro.load_template(
            config, "deactivate_gcode", ""
        )
        # Create an "endstop" object to handle the probe pin
        ppins = self.printer.lookup_object("pins")
        pin = config.get("pin")
        pin_params = ppins.lookup_pin(pin, can_invert=True, can_pullup=True)
        mcu = pin_params["chip"]
        self.mcu_endstop = mcu.setup_pin("endstop", pin_params)
        self.printer.register_event_handler(
            "klippy:mcu_identify", self._handle_mcu_identify
        )
        # Wrappers
        self.get_mcu = self.mcu_endstop.get_mcu
        self.add_stepper = self.mcu_endstop.add_stepper
        self.get_steppers = self.mcu_endstop.get_steppers
        self.home_start = self.mcu_endstop.home_start
        self.home_wait = self.mcu_endstop.home_wait
        self.query_endstop = self.mcu_endstop.query_endstop
        # multi probes state
        self.multi = "OFF"

    def _handle_mcu_identify(self):
        kin = self.printer.lookup_object("toolhead").get_kinematics()
        for stepper in kin.get_steppers():
            if stepper.is_active_axis("z"):
                self.add_stepper(stepper)

    def _raise_probe(self):
        toolhead = self.printer.lookup_object("toolhead")
        start_pos = toolhead.get_position()
        self.deactivate_gcode.run_gcode_from_command()
        if toolhead.get_position()[:3] != start_pos[:3]:
            raise self.printer.command_error(
                "Toolhead moved during probe deactivate_gcode script"
            )

    def _lower_probe(self):
        toolhead = self.printer.lookup_object("toolhead")
        start_pos = toolhead.get_position()
        self.activate_gcode.run_gcode_from_command()
        if toolhead.get_position()[:3] != start_pos[:3]:
            raise self.printer.command_error(
                "Toolhead moved during probe activate_gcode script"
            )

    def multi_probe_begin(self):
        if self.stow_on_each_sample:
            return
        self.multi = "FIRST"

    def multi_probe_end(self):
        if self.stow_on_each_sample:
            return
        self._raise_probe()
        self.multi = "OFF"

    def probing_move(
        self, pos, speed, gcmd: GCodeCommand
    ) -> Union[list[float], tuple[list[float], bool]]:
        phoming = self.printer.lookup_object("homing")
        return phoming.probing_move(self, pos, speed)

    def probe_prepare(self, hmove):
        if self.multi == "OFF" or self.multi == "FIRST":
            self._lower_probe()
            if self.multi == "FIRST":
                self.multi = "ON"

    def probe_finish(self, hmove):
        if self.multi == "OFF":
            self._raise_probe()

    def get_position_endstop(self):
        return self.position_endstop


# Helper code that can probe a series of points and report the
# position at each point.
class ProbePointsHelper:
    def __init__(
        self,
        config,
        finalize_callback,
        default_points=None,
        option_name="points",
        use_offsets=False,
    ):
        self.printer = config.get_printer()
        self.finalize_callback = finalize_callback
        self.probe_points = default_points
        self.name = config.get_name()
        self.gcode = self.printer.lookup_object("gcode")
        # Read config settings
        if default_points is None or config.get(option_name, None) is not None:
            self.probe_points = config.getlists(
                option_name, seps=(",", "\n"), parser=float, count=2
            )
        def_move_z = config.getfloat("horizontal_move_z", 5.0)
        self.default_horizontal_move_z = def_move_z
        self.adaptive_horizontal_move_z = config.getboolean(
            "adaptive_horizontal_move_z", False
        )
        self.min_horizontal_move_z = config.getfloat(
            "min_horizontal_move_z", 1.0
        )
        self.speed = config.getfloat("speed", 50.0, above=0.0)
        self.use_offsets = config.getboolean(
            "use_probe_xy_offsets", use_offsets
        )

        self.enforce_lift_speed = config.getboolean("enforce_lift_speed", False)

        # Probe retry configuration
        self.retry_session = RetrySession(config)
        # Internal probing state
        self.lift_speed = self.speed
        self.probe_offsets = (0.0, 0.0, 0.0)
        self.results = []

    def get_probe_points(self):
        return self.probe_points

    def minimum_points(self, n):
        if len(self.probe_points) < n:
            raise self.printer.config_error(
                "Need at least %d probe points for %s" % (n, self.name)
            )

    def update_probe_points(self, points, min_points):
        self.probe_points = points
        self.minimum_points(min_points)

    def use_xy_offsets(self, use_offsets):
        self.use_offsets = use_offsets

    def get_lift_speed(self, gcmd=None):
        if gcmd is not None:
            return gcmd.get_float("LIFT_SPEED", self.lift_speed, above=0.0)
        return self.lift_speed

    def _lift_toolhead(self):
        toolhead = self.printer.lookup_object("toolhead")
        # Lift toolhead
        speed = self.lift_speed
        if not self.results and not self.enforce_lift_speed:
            # Use full speed to first probe position
            speed = self.speed
        toolhead.manual_move([None, None, self.horizontal_move_z], speed)

    def _next_pos(self):
        nextpos = list(self.probe_points[len(self.results)])
        if self.use_offsets:
            nextpos[0] -= self.probe_offsets[0]
            nextpos[1] -= self.probe_offsets[1]
        self.retry_session.set_position(nextpos)
        return self.retry_session.get_probe_position()

    def _move_next(self):
        toolhead = self.printer.lookup_object("toolhead")
        # Check if done probing
        done = False
        finalize = len(self.results) >= len(self.probe_points)
        if finalize:
            toolhead.get_last_move_time()
            res = self.finalize_callback(self.probe_offsets, self.results)
            if isinstance(res, (int, float)):
                if res == 0:
                    done = True
                if self.adaptive_horizontal_move_z:
                    # then res is error
                    error = math.ceil(res)
                    self.horizontal_move_z = max(
                        error + self.probe_offsets[2],
                        self.min_horizontal_move_z,
                    )
            elif res != "retry":
                done = True
        self._lift_toolhead()
        if finalize:
            self.results = []
        if done:
            return True
        # Move to next XY probe point
        toolhead.manual_move(self._next_pos(), self.speed)
        return False

    def start_probe(self, gcmd):
        self.retry_session.start(gcmd)
        self.retry_session.reset_all()
        manual_probe.verify_no_manual_probe(self.printer)
        # Lookup objects
        probe = self.printer.lookup_object("probe", None)
        method = gcmd.get("METHOD", "automatic").lower()
        if method == "rapid_scan":
            gcmd.respond_info(
                "METHOD=rapid_scan not supported, using automatic"
            )
            method = "automatic"

        self.results = []

        def_move_z = self.default_horizontal_move_z
        self.horizontal_move_z = gcmd.get_float("HORIZONTAL_MOVE_Z", def_move_z)

        enforce_lift_speed = gcmd.get_int(
            "ENFORCE_LIFT_SPEED", None, minval=0, maxval=1
        )
        if enforce_lift_speed is not None:
            self.enforce_lift_speed = enforce_lift_speed

        if probe is None or method != "automatic":
            # Manual probe
            self.lift_speed = self.speed
            self.probe_offsets = (0.0, 0.0, 0.0)
            self._manual_probe_start()
            return
        # Perform automatic probing
        self.lift_speed = probe.get_lift_speed(gcmd)
        self.probe_offsets = probe.get_offsets()
        if self.horizontal_move_z < self.probe_offsets[2]:
            raise gcmd.error(
                "horizontal_move_z can't be less than probe's z_offset"
            )
        probe.multi_probe_begin()
        while True:
            done = self._move_next()
            if done:
                break
            pos = probe.run_probe(gcmd, self.retry_session)
            logging.info(f"Probe pos:{pos}")
            self.results.append(pos)
        probe.multi_probe_end()
        self.retry_session.end()

    def _manual_probe_start(self):
        done = self._move_next()
        if not done:
            gcmd = self.gcode.create_gcode_command("", "", {})
            manual_probe.ManualProbeHelper(
                self.printer, gcmd, self._manual_probe_finalize
            )

    def _manual_probe_finalize(self, kin_pos):
        if kin_pos is None:
            return
        self.results.append(kin_pos)
        self._manual_probe_start()


def load_config(config):
    return PrinterProbe(config, ProbeEndstopWrapper(config))
