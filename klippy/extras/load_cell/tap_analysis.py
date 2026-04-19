# Tap Analysis
#
# Copyright (C) 2025  Gareth Farrington <gareth@waves.ky>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import logging
import math
import time
from typing import Optional

import numpy as np

from klippy.gcode import GCodeCommand

from .load_cell import ApiClientHelper


class TapClassifierModule(object):
    def classify(self, tap_analysis: TapAnalysis, gcmd: Optional[GCodeCommand]):
        pass


# Capture and preserve a Trapezoidal Move as a python type
class TrapezoidalMove:
    print_time: float
    move_t: float
    start_v: float
    accel: float
    start_x: float
    start_y: float
    start_z: float
    x_r: float
    y_r: float
    z_r: float

    def __init__(self, move, time_offset=0.0):
        # copy c data to python memory
        self.print_time = float(move.print_time) - time_offset
        self.move_t = float(move.move_t)
        self.start_v = float(move.start_v)
        self.accel = float(move.accel)
        self.start_x = float(move.start_x)
        self.start_y = float(move.start_y)
        self.start_z = float(move.start_z)
        self.x_r = float(move.x_r)
        self.y_r = float(move.y_r)
        self.z_r = float(move.z_r)

    def to_dict(self) -> dict[str, float]:
        return {
            "print_time": float(self.print_time),
            "move_t": float(self.move_t),
            "start_v": float(self.start_v),
            "accel": float(self.accel),
            "start_x": float(self.start_x),
            "start_y": float(self.start_y),
            "start_z": float(self.start_z),
            "x_r": float(self.x_r),
            "y_r": float(self.y_r),
            "z_r": float(self.z_r),
        }


# point on a time/force graph
class ForcePoint(object):
    time: float
    force: float

    def __init__(self, time_t, force):
        self.time = float(time_t)
        self.force = float(force)

    def to_dict(self) -> dict[str, float]:
        return {"time": self.time, "force": self.force}


# slope/intercept based line where x is time and y is force
class ForceLine:
    slope: float
    intercept: float

    def __init__(self, slope, intercept):
        self.slope = float(slope)
        self.intercept = float(intercept)

    # measure angles between lines at the 25g = 0.1s (100ms) scale
    # Note: this is the same scale used by Prusa
    # returns +/- 0-180. Positive values represent clockwise rotation
    def angle(
        self,
        line: "ForceLine",
        time_scale: float = 0.1,
        gram_scale: float = 25.0,
    ) -> float:
        scaling_factor = time_scale / gram_scale
        this_slope = self.slope * scaling_factor
        other_slope = line.slope * scaling_factor
        radians = math.atan2(this_slope, 1) - math.atan2(other_slope, 1)
        return math.degrees(radians)

    def find_force(self, time: float) -> float:
        return self.slope * time + self.intercept

    def find_time(self, force: float) -> float:
        return (force - self.intercept) / self.slope

    def intersection(self, line: "ForceLine") -> ForcePoint:
        numerator = -self.intercept + line.intercept
        denominator = self.slope - line.slope
        # lines are parallel, will not intersect
        if denominator == 0.0:
            # to get debuggable data we want to return a clearly bad value here
            return ForcePoint(0.0, 0.0)
        intersection_time = numerator / denominator
        intersection_force = self.find_force(intersection_time)
        return ForcePoint(intersection_time, intersection_force)

    def to_dict(self) -> dict[str, float]:
        return {"slope": self.slope, "intercept": self.intercept}


#########################
# Math Support Functions


# helper class for working with a time/force graph
# work with subsections to find elbows and best fit lines
class ForceGraph:
    def __init__(self, time_nd_64: np.ndarray, force_nd_64: np.ndarray):
        self.time = time_nd_64
        self.force = force_nd_64
        # linear implementation:
        self._cum_x = np.cumsum(self.time)
        self._cum_y = np.cumsum(self.force)
        self._cum_xx = np.cumsum(self.time * self.time)
        self._cum_xy = np.cumsum(self.time * self.force)
        self._cum_yy = np.cumsum(self.force * self.force)

    def _get_segment_sum(self, arr, start_idx, end_idx):
        prior_sum = 0 if start_idx == 0 else arr[start_idx - 1]
        return arr[end_idx] - prior_sum

    def _get_segment_stats(self, start_idx, end_idx):
        """
        Get statistics for segment [start_idx:end_idx] using cumulative sums
        """
        n = end_idx - start_idx
        sum_x = self._get_segment_sum(self._cum_x, start_idx, end_idx)
        sum_y = self._get_segment_sum(self._cum_y, start_idx, end_idx)
        sum_xx = self._get_segment_sum(self._cum_xx, start_idx, end_idx)
        sum_xy = self._get_segment_sum(self._cum_xy, start_idx, end_idx)
        sum_yy = self._get_segment_sum(self._cum_yy, start_idx, end_idx)
        return n, sum_x, sum_y, sum_xx, sum_xy, sum_yy

    def _least_squares(self, start_idx, end_idx):
        """
        Compute slope/intercept and RSS for a segment of the data
        """
        n = (end_idx - start_idx) + 1
        if n < 2:
            raise ValueError("Error: fewer than 2 points used")

        sum_x = self._get_segment_sum(self._cum_x, start_idx, end_idx)
        sum_y = self._get_segment_sum(self._cum_y, start_idx, end_idx)
        sum_xx = self._get_segment_sum(self._cum_xx, start_idx, end_idx)
        sum_xy = self._get_segment_sum(self._cum_xy, start_idx, end_idx)
        sum_yy = self._get_segment_sum(self._cum_yy, start_idx, end_idx)

        denom = n * sum_xx - sum_x * sum_x
        if abs(denom) < 1e-10:
            # replicate Numpy behaviour for all x values being equal
            mean_y = sum_y / n
            slope = 0.0
            intercept = mean_y
            # RSS is just variance in y
            rss = sum_yy - n * mean_y * mean_y
            return [slope, intercept], max(0.0, rss)
        slope = (n * sum_xy - sum_x * sum_y) / denom
        intercept = (sum_y - slope * sum_x) / n
        rss = (
            sum_yy
            - 2 * slope * sum_xy
            - 2 * intercept * sum_y
            + slope * slope * sum_xx
            + 2 * slope * intercept * sum_x
            + n * intercept * intercept
        )
        return [slope, intercept], max(0.0, rss)

    # search exhaustively for the 2 lines that best fit the data
    # return the elbow index
    def _two_lines_best_fit(self, start_idx, end_idx):
        best_error = float("inf")
        best_fit_index = -1
        for i in range(1 + start_idx, end_idx - 1):
            params1, r1 = self._least_squares(start_idx, i)
            params2, r2 = self._least_squares(i + 1, end_idx)
            if params1 is not None and params2 is not None:
                error = r1 + r2
                if error < best_error:
                    best_error = error
                    best_fit_index = i
        # the index returns is the first point in the second line
        return best_fit_index

    def find_elbow(self, start_idx, end_idx):
        return self._two_lines_best_fit(start_idx, end_idx)

    # finds the index nearest to a time
    def index_near(self, instant):
        idx = int(np.searchsorted(self.time, instant))
        return min(idx, len(self.time) - 1)

    # construct a line from 2 points
    def _points_to_line(self, a, b):
        slope = (b.force - a.force) / (b.time - a.time)
        intercept = a.force - (slope * a.time)
        return ForceLine(slope, intercept)

    # construct a line using a subset of the graph
    def line(self, start_idx, end_idx):
        params, rss = self._least_squares(start_idx, end_idx)
        return ForceLine(params[0], params[1])

    # given a line and a range, calculate the standard deviation of the noise
    def noise_std(self, start_idx, end_idx, line):
        f = self.force[start_idx:end_idx]
        t = self.time[start_idx:end_idx]
        noise = []
        for i in range(len(f)):
            noise.append(f[i] - line.find_force(t[i]))
        return np.std(noise, dtype=np.float64)

    # true if the reference force won't be confused for noise in the graph chunk
    # reference force must be more than 3 standard deviations away from the line
    # at the reference index
    def is_clear_signal(
        self, start_idx, end_idx, line, reference_idx, force_idx
    ):
        noise = self.noise_std(start_idx, end_idx, line)
        noise_3_std = noise * 3
        base_force = line.find_force(self.time[reference_idx])
        return abs(base_force - self.force[force_idx]) > noise_3_std

    # return the first index that exceeds the median force between
    # start and end index
    def _split_by_force(self, start_idx, end_idx):
        start_f = self.force[start_idx]
        end_f = self.force[end_idx]
        median_f = (start_f + end_f) / 2.0
        scan = range(start_idx, end_idx)
        # if force is ascending, swap the scan direction
        if start_f > end_f:
            scan = reversed(scan)
        for i in scan:
            if self.force[i] > median_f:
                return i
        return None

    # break a tap event down into 6 points and 5 lines:
    #           |*----*\
    #           |       \
    #    *-----*|        \*-----*
    def tap_decompose(
        self,
        homing_end_time,
        pullback_start_time,
        pullback_cruise_time,
        pullback_cruise_duration,
    ):
        homing_end_idx = self.index_near(homing_end_time)
        # use the pullback duration to trim the amount of approach data used
        homing_start_time = homing_end_time - pullback_cruise_duration
        homing_start_idx = self.index_near(homing_start_time)
        # locate the point where the probe made contact with the bed
        contact_elbow_idx = self.find_elbow(homing_start_idx, homing_end_idx)

        pullback_start_idx = self.index_near(pullback_start_time)
        pullback_cruise_idx = self.index_near(pullback_cruise_time)
        # limit use of additional data after the pullback move ends
        pullback_end_time = pullback_cruise_time + (
            pullback_cruise_duration * 1.5
        )
        pullback_end_idx = self.index_near(pullback_end_time)

        # l1 is the approach line
        l1 = self.line(homing_start_idx, contact_elbow_idx)
        # sometime after contact_elbow_idx is the peak force and the start of
        # the dwell line
        dwell_end = self.time[contact_elbow_idx] + pullback_cruise_duration
        dwell_end_idx = min(pullback_start_idx, self.index_near(dwell_end))
        dwell_start_idx = self.find_elbow(contact_elbow_idx, dwell_end_idx)
        # l2 is the compression line
        # also +1 the last index in case its sequential [1, 2]
        l2 = self.line(contact_elbow_idx, dwell_start_idx + 1)
        # l3 is the dwell line
        l3 = self.line(dwell_start_idx, pullback_start_idx)

        # find the approximate elbow location
        break_contact_idx = self.find_elbow(
            pullback_cruise_idx, pullback_end_idx
        )
        # l5 is the line after decompression ends
        l5 = self.line(break_contact_idx, pullback_end_idx)
        # split the points between the elbow and the start of movement by force
        midpoint_idx = self._split_by_force(
            pullback_cruise_idx, break_contact_idx
        )
        # elbow finding success depends on their being good signal-to-noise
        # this checks if there will be enough clear data to analyze
        use_curve_optimization = False
        if midpoint_idx is not None:
            clear_dwell = self.is_clear_signal(
                dwell_start_idx,
                dwell_end_idx,
                l3,
                dwell_end_idx,
                midpoint_idx - 1,
            )
            clear_decomp = self.is_clear_signal(
                break_contact_idx,
                pullback_end_idx,
                l5,
                break_contact_idx,
                midpoint_idx,
            )
            use_curve_optimization = clear_dwell and clear_decomp
        if use_curve_optimization:
            # perform iterative refinement
            l4_start = self.line(pullback_cruise_idx, midpoint_idx)
            # real break contact index
            break_contact_idx = self.find_elbow(midpoint_idx, pullback_end_idx)
            l4_end = self.line(midpoint_idx, break_contact_idx)
            l5 = self.line(break_contact_idx, pullback_end_idx)
            # a synthetic l4 is built from 2 points:
            l4 = self._points_to_line(
                l4_start.intersection(l3), l4_end.intersection(l5)
            )
        else:
            # noise is too high, don't use the curve optimization
            l4 = self.line(pullback_cruise_idx, break_contact_idx)
            # log for user debugging
            logging.info("TapAnalysis: curve optimization not used")

        return [l1, l2, l3, l4, l5], homing_start_idx, pullback_end_idx


class TapValidationError(Exception):
    error_code: str

    def __init__(self, error_code: str, message: str):
        super().__init__(message)
        self.error_code = error_code
        pass

    def to_dict(self) -> dict[str, str]:
        return {"error_code": self.error_code, "message": str(self)}


# Move index constants. The PROBE_START move may be deleted from the trapq if
# the probe takes longer than 30s. Indexing from the end of the list
# is always consistent:
PROBE_START = -6
PROBE_CRUISE = -5
PROBE_HALT = -4
PULLBACK_START = -3
PULLBACK_CRUISE = -2
PULLBACK_END = -1


class TapAnalysis:
    def __init__(self, samples: list[float], trigger_force):
        self._is_valid = False
        self._tap_pos: Optional[tuple[float, float, float]] = None
        self._tap_points: list[ForcePoint] = []
        self._tap_lines: list[ForceLine] = []
        self._tap_angles: list[float] = []
        self._elapsed: float = 0.0
        self._collection_time: float = 0.0
        self._error: Optional[TapValidationError] = None
        self._home_end_time: Optional[float] = None
        self._pullback_start_time: Optional[float] = None
        self._pullback_end_time: Optional[float] = None
        self._pullback_cruise_time: Optional[float] = None
        self._pullback_duration: Optional[float] = None
        self._homing_start_index: int = 0
        self._pullback_end_index: int = -1
        nd_samples = np.asarray(samples, dtype=np.float64)
        self._time_offset = float(nd_samples[0, 0])
        self._time = nd_samples[:, 0] - self._time_offset
        self._force = nd_samples[:, 1]
        self._force_graph = ForceGraph(self._time, self._force)
        self._trigger_force = trigger_force
        self._moves: list[TrapezoidalMove] = []

    @staticmethod
    def _move_dist(move: TrapezoidalMove, print_time: float):
        move_t = move.move_t
        move_time = max(0.0, min(move_t, print_time - move.print_time))
        dist = (move.start_v + (0.5 * move.accel) * move_time) * move_time
        return dist

    @staticmethod
    def _move_pos(move: TrapezoidalMove, dist: float):
        return (
            move.start_x + (move.x_r * dist),
            move.start_y + (move.y_r * dist),
            move.start_z + (move.z_r * dist),
        )

    # get an XYZ position from the toolhead position history
    # positions before/after the captured history are assumed to be stationary
    def get_toolhead_position(self, print_time):
        for i, move in enumerate(self._moves):
            start_time = move.print_time
            # time before first move, assume printer was stationary
            if i == 0 and print_time < start_time:
                return self._move_pos(move, 0)
            end_time = float("inf")
            if i < (len(self._moves) - 1):
                end_time = self._moves[i + 1].print_time
            if start_time <= print_time < end_time:
                # we have found the move
                dist = self._move_dist(move, print_time)
                pos = self._move_pos(move, dist)
                return pos
            else:
                continue
        # time is after last move, assume printer was stationary
        move = self._moves[-1]
        dist = self._move_dist(move, print_time)
        return self._move_pos(move, dist)

    def log_trapq(self):
        logging.info("TapAnalysis - Trapezoidal Movement Queue contents:")
        for i, move in enumerate(self._moves):
            logging.info(f"Move {i}: {move.to_dict()}")

    # adjust move_t of PROBE_CRUISE to match the toolhead position of PROBE_HALT
    def _recalculate_homing_end(self):
        homing_move = self._moves[PROBE_CRUISE]
        halt_move = self._moves[PROBE_HALT]
        # acceleration should be 0! This is the 'coasting' move:
        if homing_move.accel != 0.0:
            raise TapValidationError(
                "COASTING_MOVE_ACCELERATION",
                "Probing move is accelerating/decelerating which is invalid",
            )
        # how long did it take to get to end_z?
        homing_move.move_t = abs(
            (halt_move.start_z - homing_move.start_z) / homing_move.start_v
        )
        return homing_move.print_time + homing_move.move_t

    # extract and save TrapQueue moves
    def _extract_trapq(self, printer):
        trapq = printer.lookup_object("motion_report").trapqs["toolhead"]
        moves, _ = trapq.extract_trapq(float(self._time[0] + self._time_offset))
        for move in moves:
            self._moves.append(TrapezoidalMove(move, self._time_offset))

    # perform analysis, throws exceptions
    def analyze(self, printer):
        self._extract_trapq(printer)
        num_moves = len(self._moves)
        if num_moves < 5:
            raise TapValidationError(
                "TOO_FEW_PROBING_MOVES",
                "5 Probing moves expected but there were fewer",
            )
        elif num_moves > 6:
            raise TapValidationError(
                "TOO_MANY_PROBING_MOVES",
                "More than 6 probing moves were found during the tap",
            )
        self._home_end_time = self._recalculate_homing_end()
        self._pullback_start_time = self._moves[PULLBACK_START].print_time
        self._pullback_end_time = (
            self._moves[PULLBACK_END].print_time
            + self._moves[PULLBACK_END].move_t
        )
        self._pullback_cruise_time = self._moves[PULLBACK_CRUISE].print_time
        self._pullback_duration = (
            self._pullback_end_time - self._pullback_start_time
        )
        lines, i, j = self._force_graph.tap_decompose(
            self._home_end_time,
            self._pullback_start_time,
            self._pullback_cruise_time,
            self._pullback_duration,
        )
        self._homing_start_index = i
        self._pullback_end_index = j
        self.set_tap_lines(lines)
        self._validate_order()
        self._validate_tap_shape()
        self._validate_break_contact_time()
        self._is_valid = True

    # validate that a set of ForcePoint objects are in chronological order
    def _validate_order(self):
        p = self._tap_points
        if not (
            p[0].time
            < p[1].time
            < p[2].time
            < p[3].time
            < p[4].time
            < p[5].time
        ):
            raise TapValidationError(
                "TAP_CHRONOLOGY", "Tap points are out of chronological order"
            )

    # Validate that the rotations between lines form a tap shape
    def _validate_tap_shape(self):
        a1, a2, a3, a4 = self._tap_angles
        # with two polarities there are 2 valid tap shapes:
        if not (
            (a1 > 0 and a2 < 0 and a3 < 0 and a4 > 0)
            or (a1 < 0 and a2 > 0 and a3 > 0 and a4 < 0)
        ):
            raise TapValidationError(
                "TAP_SHAPE_INVALID", "Force data does not form a tap shape"
            )

    # The proposed break contact point must fall inside the
    # first 3/4s of the pullback move
    def _validate_break_contact_time(self):
        break_contact_time = self._tap_points[4].time
        start_t = self._pullback_start_time
        end_t = self._pullback_end_time
        safety_margin = (end_t - start_t) / 4.0
        if break_contact_time < start_t:
            raise TapValidationError(
                "TAP_BREAK_CONTACT_TOO_EARLY",
                "Tap break-contact time is too early",
            )
        elif break_contact_time > end_t:
            raise TapValidationError(
                "TAP_BREAK_CONTACT_TOO_LATE",
                "Tap break-contact time is too late",
            )
        elif break_contact_time > (end_t - safety_margin):
            raise TapValidationError(
                "TAP_PULLBACK_TOO_SHORT",
                "Tap break-contact time is too late, pullback move may be too "
                "short",
            )

    def _calculate_points(self):
        l1, l2, l3, l4, l5 = self._tap_lines
        # Line intersections:
        p0 = ForcePoint(
            self._time[self._homing_start_index],
            l1.find_force(float(self._time[self._homing_start_index])),
        )
        p1 = l1.intersection(l2)
        p2 = l2.intersection(l3)
        p3 = l3.intersection(l4)
        p4 = l4.intersection(l5)
        p5 = ForcePoint(
            self._time[self._pullback_end_index],
            l5.find_force(float(self._time[self._pullback_end_index])),
        )
        self._tap_points = [p0, p1, p2, p3, p4, p5]

    def _calculate_angles(self):
        l1, l2, l3, l4, l5 = self._tap_lines
        self._tap_angles = [
            l1.angle(l2),
            l2.angle(l3),
            l3.angle(l4),
            l4.angle(l5),
        ]

    # 'read only' fields:
    def get_time(self) -> list[float]:
        return self._time.tolist()

    def get_force(self) -> list[float]:
        return self._force.tolist()

    def get_trigger_force(self) -> float:
        return self._trigger_force

    def get_moves(self) -> list[TrapezoidalMove]:
        return self._moves

    def get_home_end_time(self) -> float:
        return self._home_end_time

    def get_pullback_start_time(self) -> float:
        return self._pullback_start_time

    def get_pullback_end_time(self) -> float:
        return self._pullback_end_time

    def get_tap_points(self) -> list[ForcePoint]:
        return self._tap_points

    def get_tap_pos(self) -> Optional[tuple[float, float, float]]:
        return self._tap_pos

    def get_tap_angles(self) -> list[float]:
        return self._tap_angles

    # read/write fields
    def get_tap_lines(self) -> list[ForceLine]:
        return self._tap_lines

    # Allow TapClassifier modules to overwrite the lines
    # This also causes the tap points, angles and tap pos to be recalculated
    def set_tap_lines(self, tap_lines: list[ForceLine]):
        self._tap_lines = tap_lines
        self._calculate_points()
        self._calculate_angles()
        break_contact_time = self._tap_points[4].time
        self._tap_pos = self.get_toolhead_position(break_contact_time)

    def is_valid(self) -> bool:
        return self._is_valid

    def set_is_valid(self, is_valid: bool):
        self._is_valid = is_valid

    def get_validation_error(self) -> TapValidationError:
        return self._error

    def set_validation_error(self, error: TapValidationError):
        self._error = error

    def get_elapsed(self) -> float:
        return self._elapsed

    def set_elapsed(self, elapsed: float):
        self._elapsed = elapsed

    def get_collection_time(self):
        return self._collection_time

    def set_collection_time(self, collection_time):
        self._collection_time = collection_time

    # convert to dictionary for JSON encoder
    def to_dict(self):
        return {
            "time": self.get_time(),
            "force": self._force.tolist(),
            "tap_points": [point.to_dict() for point in self.get_tap_points()],
            "tap_lines": [line.to_dict() for line in self.get_tap_lines()],
            "tap_angles": self.get_tap_angles(),
            "tap_pos": self.get_tap_pos(),
            "moves": [move.to_dict() for move in self._moves],
            "home_end_time": self.get_home_end_time(),
            "pullback_start_time": self.get_pullback_start_time(),
            "pullback_end_time": self.get_pullback_end_time(),
            "collection_time": self.get_collection_time(),
            "elapsed": self.get_elapsed(),
            "is_valid": self.is_valid(),
            "error": None if self._error is None else self._error.to_dict(),
        }


# Orchestrate TapAnalysis and TapClassifier. Handle timing, error capture,
# event broadcast, clients & logging
class TapAnalysisHelper:
    def __init__(self, printer, name, tap_classifier):
        self._printer = printer
        self._tap_classifier = tap_classifier
        # webhooks support
        self._clients = ApiClientHelper(printer)
        header = {"header": ["probe_tap_event"]}
        self._clients.add_mux_endpoint(
            "load_cell_probe/dump_taps", "load_cell_probe", name, header
        )

    def analyze(self, samples, trigger_force, collection_time, gcmd):
        t_start = time.time()
        tap_analysis = TapAnalysis(samples, trigger_force)
        try:
            tap_analysis.analyze(self._printer)
        except TapValidationError as ve:
            tap_analysis.set_is_valid(False)
            tap_analysis.set_validation_error(ve)
            tap_analysis.log_trapq()
        # tap classifier always gets to process the data
        try:
            self._tap_classifier.classify(tap_analysis, gcmd)
        except TapValidationError as ve:
            tap_analysis.set_is_valid(False)
            tap_analysis.set_validation_error(ve)
        # total elapsed time for all calculations
        tap_analysis.set_elapsed(time.time() - t_start)
        tap_analysis.set_collection_time(collection_time)
        # broadcast tap event data:
        self._clients.send({"tap": tap_analysis.to_dict()})
        self._log_errors(tap_analysis)
        return tap_analysis

    # log errors to event log
    @staticmethod
    def _log_errors(tap_analysis):
        # if the tap is valid, don't log any errors
        if tap_analysis.is_valid():
            return
        # log errors
        ve = tap_analysis.get_validation_error()
        logging.info("Bad tap detected: %s - %s" % (ve.error_code, ve))

    # get internal tap events
    def add_client(self, callback):
        self._clients.add_client(callback)
