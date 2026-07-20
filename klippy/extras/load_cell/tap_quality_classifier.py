# Simple Tap Classifier
#
# Copyright (C) 2025  Gareth Farrington <gareth@waves.ky>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import logging
import math
from typing import Any, Optional

import numpy as np

from klippy import Printer
from klippy.configfile import ConfigWrapper, PrinterConfig
from klippy.extras.bed_mesh import BedMesh
from klippy.extras.load_cell.tap_analysis import (
    ForcePoint,
    TapAnalysis,
    TapClassifierModule,
    TapValidationError,
)
from klippy.extras.probe import ProbePointsHelper
from klippy.gcode import GCodeCommand


class TapQualityClassifierConfig:
    minimum_tap_quality: float = 40.0
    decompression_angle: Optional[float] = None
    max_baseline_force_delta: float = 0.25
    max_departure_force: float = 0.25
    max_approach_force: float = 0.5
    max_dwell_force_drop: float = 0.75

    def __init__(self, config: ConfigWrapper):
        self._config = config
        self._config_name = config.get_name()
        self._printer = config.get_printer()
        self._cfg_minimum_tap_quality = config.getfloat(
            "min_tap_quality",
            minval=0.0,
            maxval=100.0,
            default=40.0,
        )
        self._cfg_decompression_angle: Optional[float] = config.getfloat(
            "decompression_angle", above=0.0, below=90.0, default=None
        )
        self._cfg_max_approach_force: float = (
            config.getfloat(
                "max_approach_force", minval=0.0, maxval=100.0, default=50.0
            )
            / 100.0
        )
        self._cfg_max_baseline_force_delta: float = (
            config.getfloat(
                "max_baseline_force_delta",
                minval=0.0,
                maxval=100.0,
                default=25.0,
            )
            / 100.0
        )
        self._cfg_max_departure_force: float = (
            config.getfloat(
                "max_departure_force", minval=0.0, maxval=100.0, default=25.0
            )
            / 100.0
        )
        self._cfg_max_dwell_force_drop: float = (
            config.getfloat(
                "max_dwell_force_drop", minval=0.0, maxval=100.0, default=75.0
            )
            / 100.0
        )
        self.minimum_tap_quality = self._cfg_minimum_tap_quality
        self.decompression_angle = self._cfg_decompression_angle
        self.max_baseline_force_delta = self._cfg_max_baseline_force_delta
        self.max_approach_force = self._cfg_max_approach_force
        self.max_departure_force = self._cfg_max_departure_force
        self.max_dwell_force_drop = self._cfg_max_dwell_force_drop

    def customize(self, gcmd: GCodeCommand):
        self.minimum_tap_quality = gcmd.get_float(
            "MIN_TAP_QUALITY",
            minval=0.0,
            maxval=100.0,
            default=self._cfg_minimum_tap_quality,
        )
        self.decompression_angle = gcmd.get_float(
            "DECOMPRESSION_ANGLE",
            above=0.0,
            below=90.0,
            default=self._cfg_decompression_angle,
        )

    def _save_config(self, field, value: Any):
        configfile: PrinterConfig = self._printer.lookup_object("configfile")
        configfile.set(self._config_name, field, value)

    def set_minimum_tap_quality(self, minimum_tap_quality: float):
        minimum_tap_quality = round(float(minimum_tap_quality), 1)
        if not (0.0 <= minimum_tap_quality <= 100.0):
            raise self._printer.config_error(
                "min_tap_quality must be between 0.0 and 100.0"
            )
        self._cfg_minimum_tap_quality = self.minimum_tap_quality = (
            minimum_tap_quality
        )
        self._save_config("min_tap_quality", self.minimum_tap_quality)

    def set_decompression_angle(self, decompression_angle: float):
        decompression_angle = round(float(decompression_angle), 1)
        if not (0.0 < decompression_angle < 90.0):
            raise self._printer.config_error(
                "decompression_angle must be between 0.0 and 90.0"
            )
        self._cfg_decompression_angle = self.decompression_angle = (
            decompression_angle
        )
        self._save_config("decompression_angle", self.decompression_angle)


# Simple tap classifier rules:
# 1) The compression force must greater than the trigger force
# 2) Tap quality metric must be more than the minimum_tap_quality
class TapQualityClassifier(TapClassifierModule):
    def __init__(self, config: ConfigWrapper):
        self.printer: Printer = config.get_printer()
        self.bed_mesh_config = None
        if config.has_section("bed_mesh"):
            self.bed_mesh_config = config.getsection("bed_mesh")
        self.config: TapQualityClassifierConfig = TapQualityClassifierConfig(
            config
        )
        self._calibrating = False
        self._tap_analyses = []

    def set_minimum_tap_quality(self, minimum_tap_quality: float):
        self.config.set_minimum_tap_quality(minimum_tap_quality)

    def set_decompression_angle(self, decompression_angle: float):
        self.config.set_decompression_angle(decompression_angle)

    def _calculate_mean_decompression_angle(self) -> float:
        angles = []
        for tap_analysis in self._tap_analyses:
            if not tap_analysis.is_valid():
                continue
            lines = tap_analysis.get_tap_lines()
            decompression_line = lines[3]
            angle = abs(self._slope_to_degrees(decompression_line.slope))
            angles.append(angle)

        return float(np.mean(angles))

    def _recalculate_tap_qualities(self) -> list[float]:
        qualities: list[float] = []
        for tap_analysis in self._tap_analyses:
            if not tap_analysis.is_valid():
                continue
            quality = self.calculate_tap_quality(tap_analysis)
            qualities.append(quality)
        return qualities

    def _finalize_callback(self, probe_offsets, results):
        pass

    def calibrate(self, gcmd):
        gcmd.respond_info("Starting DECOMPRESSION_ANGLE_CALIBRATION...")
        bed_mesh: BedMesh = self.printer.lookup_object("bed_mesh", default=None)
        if bed_mesh is None:
            raise gcmd.error("bed_mesh not configured")

        self._calibrating = True
        self._tap_analyses = []
        try:
            logging.info("Probing Mesh Points...")
            points = bed_mesh.generate_points(gcmd, "tap_quality_calibrate")
            points_helper = ProbePointsHelper(
                self.bed_mesh_config,
                self._finalize_callback,
                points,
                use_offsets=True,
                enable_horizontal_z_clearance=True,
            )
            points_helper.start_probe(gcmd)
        finally:
            self._calibrating = False
        if self._tap_analyses is None:
            raise gcmd.error("No taps were collected!")
        gcmd.respond_info(f"Collected {len(self._tap_analyses)} taps")

        for tap in self._tap_analyses:
            if not tap.is_valid():
                gcmd.respond_error(
                    "Some taps failed! Make sure the nozzle is clean and free of ooze."
                )
        qualities = self._recalculate_tap_qualities()
        if not qualities:
            gcmd.respond_error(
                "Not taps passed the quality check! Make sure the nozzle is "
                "clean and free of ooze."
            )
        mean_quality = float(np.mean(qualities))
        std_quality = float(np.std(qualities))
        gcmd.respond_info(
            "DECOMPRESSION_ANGLE_CALIBRATION Complete"
            f"Tap Quality average: {mean_quality:.1f}%, "
            f"min: {min(qualities):.1f}%, max: {max(qualities):.1f}% "
            f"std dev: {std_quality:.1f}%"
        )

        # decompression angle
        mean_angle = self._calculate_mean_decompression_angle()
        self.set_decompression_angle(mean_angle)
        gcmd.respond_info(
            f"\nMean Decompression Angle: {mean_angle:.1f} degrees"
            f"\ndecompression_angle={mean_angle:.1f}\n This has been saved "
            f"for the current session.\nThe SAVE_CONFIG command will "
            f"update the printer config file and restart the printer."
        )

    @staticmethod
    def _slope_to_degrees(slope: float, time_scale=0.1, gram_scale=25):
        scaling_factor = time_scale / gram_scale
        this_slope = 0.0
        other_slope = slope * scaling_factor
        radians = math.atan2(this_slope, 1) - math.atan2(other_slope, 1)
        return math.degrees(radians)

    @staticmethod
    def _clamp(value: float, threshold: float) -> float:
        if value > threshold:
            value = 1.0
        return value

    @staticmethod
    def _calc_force_range(points: list[ForcePoint]):
        min_force = float("inf")
        max_force = float("-inf")
        for point in points:
            force = point.force
            min_force = min(min_force, force)
            max_force = max(max_force, force)
        return max_force - min_force

    def calculate_tap_quality(self, tap_analysis: TapAnalysis) -> float:
        # unpack tap points with names
        tap_points = tap_analysis.get_tap_points()
        approach_start_point = tap_points[0]
        approach_end_point = compression_start_point = tap_points[1]
        compression_end_point = dwell_start_point = tap_points[2]
        dwell_end_point = tap_points[3]
        decompression_end_point = departure_start_point = tap_points[4]
        departure_end_point = tap_points[5]

        # only the compression force in the compression line
        compression_force = abs(
            compression_end_point.force - compression_start_point.force
        )

        # These 2 metrics are expected to approach 0.0 in an ideal tap. If
        # they are large that usually indicates the tap is fouled.
        # They are clamped such that if they exceed 0.25, they become 1.0
        # this causes the quality score to drop to 0.0
        baseline_delta_pct = self._clamp(
            abs(compression_start_point.force - decompression_end_point.force)
            / compression_force,
            self.config.max_baseline_force_delta,
        )
        departure_force_pct = self._clamp(
            abs(departure_end_point.force - departure_start_point.force)
            / compression_force,
            self.config.max_departure_force,
        )

        # The approach force would also ideally be 0, but it can be subject to
        # bowden tube forces which we must allow. A larger 0.5 cutoff is used
        # to allow for this.
        approach_force_pct = self._clamp(
            abs(approach_end_point.force - approach_start_point.force)
            / compression_force,
            self.config.max_approach_force,
        )

        # Dwell force drop is loosely clamped because it can drop quite a lot
        # in an acceptable tap
        dwell_force_drop_pct = self._clamp(
            abs(dwell_start_point.force - dwell_end_point.force)
            / compression_force,
            self.config.max_dwell_force_drop,
        )

        # Normalize the decompression angle to the configured
        # mean angle for this printer
        lines = tap_analysis.get_tap_lines()
        decompression_line = lines[3]
        decomp_angle = abs(self._slope_to_degrees(decompression_line.slope))
        decompression_angle = self.config.decompression_angle
        if decompression_angle is None:
            decompression_angle = 80.0
        normalized_decompression_angle = (
            abs(decomp_angle - decompression_angle) / decompression_angle
        )

        # Calculate tap quality score
        tap_quality: float = 100.0 * (
            # Using (1.0 - x) converts the scores from 0 = good to 1 = good
            (1.0 - approach_force_pct)
            * (1.0 - departure_force_pct)
            * (1.0 - baseline_delta_pct)
            * (1.0 - dwell_force_drop_pct)
            * (1.0 - normalized_decompression_angle)
        )

        return tap_quality

    def classify(self, tap_analysis: TapAnalysis, gcmd: GCodeCommand):
        if self._calibrating:
            self._tap_analyses.append(tap_analysis)

        # This module cant rescue bad data
        if not tap_analysis.is_valid():
            return

        # if the user has not configured an angle, don't classify the tap
        if self.config.decompression_angle is None:
            return

        # unpack tap points with names
        tap_points = tap_analysis.get_tap_points()
        compression_start_point = tap_points[1]
        compression_end_point = tap_points[2]

        # only the compression force in the compression line
        compression_force = abs(
            compression_end_point.force - compression_start_point.force
        )

        # compression check: did we meet the minimum force requirement?
        if compression_force < tap_analysis.get_trigger_force():
            raise TapValidationError(
                "LOW_COMPRESSION_FORCE",
                f"Compression force, {compression_force}g, was less than the "
                f"trigger force ({tap_analysis.get_trigger_force()}g)",
            )

        if gcmd is not None:
            self.config.customize(gcmd)

        tap_quality = self.calculate_tap_quality(tap_analysis)

        lines = tap_analysis.get_tap_lines()
        decompression_line = lines[3]

        # TODO: should this be a debugging option? logged?
        # log info to console for now...
        gcode = self.printer.lookup_object("gcode")
        gcode.respond_info(
            f"Tap Quality: {round(tap_quality, 1)}%, Decompression angle: "
            f" {round(abs(self._slope_to_degrees(decompression_line.slope)), 1)}"
        )

        # tap quality check
        if tap_quality < self.config.minimum_tap_quality:
            raise TapValidationError(
                "LOW_TAP_QUALITY",
                f"Tap quality, {tap_quality}%, was less than the minimum tap "
                f"quality {self.config.minimum_tap_quality}%",
            )


def load_config(config):
    return TapQualityClassifier(config)
