# Code for coordinating events on the printer toolhead
#
# Copyright (C) 2016-2025  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import importlib
import logging
import math

from . import chelper
from .extras import pathplan
from .extras.danger_options import get_danger_options
from .kinematics import extruder as kinematics_extruder

# Common suffixes: _d is distance (in mm), _v is velocity (in
#   mm/second), _v2 is velocity squared (mm^2/s^2), _t is time (in
#   seconds), _r is ratio (scalar between 0.0 and 1.0)


# Class to track each move request
class Move:
    def __init__(self, toolhead, start_pos, end_pos, speed):
        self.toolhead = toolhead
        self.start_pos = tuple(start_pos)
        self.end_pos = tuple(end_pos)
        self.accel = toolhead.max_accel
        self.junction_deviation = toolhead.junction_deviation
        self.timing_callbacks = []
        velocity = min(speed, toolhead.max_velocity)
        self.is_kinematic_move = True
        self.axes_d = axes_d = [ep - sp for sp, ep in zip(start_pos, end_pos)]
        self.move_d = move_d = math.sqrt(sum([d * d for d in axes_d[:3]]))
        if move_d < 0.000000001:
            # Extrude only move
            self.end_pos = (
                start_pos[0],
                start_pos[1],
                start_pos[2],
            ) + self.end_pos[3:]
            axes_d[0] = axes_d[1] = axes_d[2] = 0.0
            self.move_d = move_d = max([abs(ad) for ad in axes_d[3:]])
            inv_move_d = 0.0
            if move_d:
                inv_move_d = 1.0 / move_d
            self.accel = 99999999.9
            velocity = speed
            self.is_kinematic_move = False
        else:
            inv_move_d = 1.0 / move_d
        self.axes_r = [d * inv_move_d for d in axes_d]
        self.min_move_t = move_d / velocity
        # Junction speeds are tracked in velocity squared.  The
        # delta_v2 is the maximum amount of this squared-velocity that
        # can change in this move.
        self.max_start_v2 = 0.0
        self.max_cruise_v2 = velocity**2
        self.delta_v2 = 2.0 * move_d * self.accel
        self.max_smoothed_v2 = 0.0
        self.smooth_delta_v2 = 2.0 * move_d * toolhead.max_accel_to_decel
        self.next_junction_v2 = 999999999.9
        # Ramp-spanning state (unified planner, see SPAN_MAX_ANGLE). A notched
        # accel ramp always costs 2/f_n seconds of travel, so forcing it to
        # start and end at a==0 inside every move floors the reachable speed
        # at about f_n*move_d. Spanning lets ONE ramp cover a run of moves, so
        # the runway is the run's distance -- the ramp shape, and therefore its
        # spectral zero, is unchanged.
        #   max_junction_v2 -- the geometric junction limit (corner/cruise/extra
        #     axis), WITHOUT the reachability term. It is the speed a spanning
        #     ramp may pass through this junction at.
        #   span_start_v2 / span_start_d -- the speed at the start of the accel
        #     run this move belongs to, and the path distance from there to the
        #     start of this move.
        self.max_junction_v2 = 999999999.9
        self.span_start_v2 = 0.0
        self.span_start_d = 0.0
        self.span_link = False

    def limit_speed(self, speed, accel):
        speed2 = speed**2
        if speed2 < self.max_cruise_v2:
            self.max_cruise_v2 = speed2
            self.min_move_t = self.move_d / speed
        self.accel = min(self.accel, accel)
        self.delta_v2 = 2.0 * self.move_d * self.accel
        self.smooth_delta_v2 = min(self.smooth_delta_v2, self.delta_v2)

    def limit_next_junction_speed(self, speed):
        self.next_junction_v2 = min(self.next_junction_v2, speed**2)

    def move_error(self, msg="Move out of range"):
        ep = self.end_pos
        m = "%s: %.3f %.3f %.3f [%.3f]" % (msg, ep[0], ep[1], ep[2], ep[3])
        return self.toolhead.printer.command_error(m)

    def calc_junction(self, prev_move):
        if not self.is_kinematic_move or not prev_move.is_kinematic_move:
            return
        # Allow extra axes to calculate maximum junction
        ea_v2 = [
            ea.calc_junction(prev_move, self, e_index + 3)
            for e_index, ea in enumerate(self.toolhead.extra_axes)
        ]
        th = self.toolhead
        axes_r = self.axes_r
        prev_axes_r = prev_move.axes_r
        junction_cos_theta = -(
            axes_r[0] * prev_axes_r[0]
            + axes_r[1] * prev_axes_r[1]
            + axes_r[2] * prev_axes_r[2]
        )
        # Reachability into this junction. With ramp spanning the accel event
        # is allowed to have started before prev_move, so the runway is the
        # whole run rather than prev_move alone. It is still capped by the
        # STOCK constant-accel reach over prev_move: that keeps every move
        # individually feasible for the non-unified trapezoid if shaping is
        # later disabled.
        span_link = th._span_link_ok(prev_move, self, junction_cos_theta)
        if span_link:
            prev_reach_v2 = min(
                th._span_reach_v2(
                    prev_move,
                    prev_move.span_start_v2,
                    prev_move.span_start_d + prev_move.move_d,
                ),
                prev_move.max_start_v2 + prev_move.delta_v2,
            )
        else:
            prev_reach_v2 = th._move_reach_v2(prev_move, prev_move.max_start_v2)
        max_junction_v2 = min(
            [
                self.max_cruise_v2,
                prev_move.max_cruise_v2,
                prev_move.next_junction_v2,
            ]
            + ea_v2
        )
        max_start_v2 = min(max_junction_v2, prev_reach_v2)
        # Find max velocity using "approximated centripetal velocity"
        sin_theta_d2 = math.sqrt(max(0.5 * (1.0 - junction_cos_theta), 0.0))
        cos_theta_d2 = math.sqrt(max(0.5 * (1.0 + junction_cos_theta), 0.0))
        one_minus_sin_theta_d2 = 1.0 - sin_theta_d2
        if one_minus_sin_theta_d2 > 0.0 and cos_theta_d2 > 0.0:
            R_jd = sin_theta_d2 / one_minus_sin_theta_d2
            move_jd_v2 = R_jd * self.junction_deviation * self.accel
            pmove_jd_v2 = R_jd * prev_move.junction_deviation * prev_move.accel
            # Approximated circle must contact moves no further than mid-move
            #   centripetal_v2 = .5 * self.move_d * self.accel * tan_theta_d2
            quarter_tan_theta_d2 = 0.25 * sin_theta_d2 / cos_theta_d2
            move_centripetal_v2 = self.delta_v2 * quarter_tan_theta_d2
            pmove_centripetal_v2 = prev_move.delta_v2 * quarter_tan_theta_d2
            max_junction_v2 = min(
                max_junction_v2,
                move_jd_v2,
                pmove_jd_v2,
                move_centripetal_v2,
                pmove_centripetal_v2,
            )
            max_start_v2 = min(max_start_v2, max_junction_v2)
        # Apply limits
        self.max_junction_v2 = max_junction_v2
        self.max_start_v2 = max_start_v2
        self.max_smoothed_v2 = min(
            max_start_v2, prev_move.max_smoothed_v2 + prev_move.smooth_delta_v2
        )
        # Carry the accel run forward while the junction itself is not what
        # binds. When the geometry (a corner, a slower feedrate) holds this
        # junction below what the ramp could reach, the accel event genuinely
        # restarts here and so does the span.
        self.span_link = span_link
        if span_link and max_start_v2 >= prev_reach_v2 - 1e-9:
            self.span_start_v2 = prev_move.span_start_v2
            self.span_start_d = prev_move.span_start_d + prev_move.move_d
        else:
            self.span_start_v2 = max_start_v2
            self.span_start_d = 0.0

    def set_junction(self, start_v2, cruise_v2, end_v2):
        # Determine accel, cruise, and decel portions of the move distance
        half_inv_accel = 0.5 / self.accel
        accel_d = (cruise_v2 - start_v2) * half_inv_accel
        decel_d = (cruise_v2 - end_v2) * half_inv_accel
        cruise_d = self.move_d - accel_d - decel_d
        # Determine move velocities
        self.start_v = start_v = math.sqrt(start_v2)
        self.cruise_v = cruise_v = math.sqrt(cruise_v2)
        self.end_v = end_v = math.sqrt(end_v2)
        # Determine time spent in each portion of move (time is the
        # distance divided by average velocity)
        self.accel_t = accel_d / ((start_v + cruise_v) * 0.5)
        self.cruise_t = cruise_d / cruise_v
        self.decel_t = decel_d / ((end_v + cruise_v) * 0.5)


LOOKAHEAD_FLUSH_TIME = 0.250

# Ramp spanning: two consecutive moves may share one accel ramp only if the
# path barely turns between them.
#
# A ramp shapes the SCALAR path speed; the axes see a_x = rx*a(t). If the
# direction changes at time t_c while a(t) is still nonzero, the turning axis
# sees a TRUNCATED pulse, a(t)*u(t - t_c), scaled by the change in direction:
#     a_x(t) = r1x*a(t) + (r2x - r1x) * a(t)*u(t - t_c)
# The first term keeps the zero. The second does not -- a truncated triangle
# has no null at f_n. Numerically integrating the ideal pulse gives
# |A_tail(f_n)| / |a|_1 <= 0.159, worst at t_c on the accel peak, so the
# residual left on the turning axis is about
#     |r2 - r1| * 0.159,   with |r2 - r1| = 2*sin(theta/2)
# against the ~0.0066 the emitter's ZOH discretization leaves there WHEN
# unified_spectral_null is off. That puts 18 degrees at 7.5x the floor -- far
# too coarse -- while 2 degrees lands at 0.8x, under the noise already present:
#     0.5 deg -> 0.2x    2 deg -> 0.8x     10 deg -> 4.2x
#     1.0 deg -> 0.4x    5 deg -> 2.1x     18 deg -> 7.5x
# Hence a 2 degree default, tunable via unified_span_max_angle for anyone who
# wants to trade that residual for spanning across coarser geometry.
#
# With the null ON there is no 0.0066 floor to hide under, so this table no
# longer sets the trade -- see SPAN_COLLINEAR_EPS.
SPAN_MAX_ANGLE = 2.0

# How exactly two moves must agree on direction for a spanned run to count as
# straight, per unit-vector component. A straight run's axis ratios are
# constant, so one scalar spectral null transfers to every axis unchanged; any
# real turn makes them time-varying and it does not. Slicer output that is
# genuinely collinear reproduces the direction bit-for-bit, so this only has to
# absorb the rounding in axes_r itself.
#
# MEASURED 2026-07-29, and this is stricter than the physics needs. Nulling the
# scalar on a turning run does make the TURNING axis slightly worse (it cannot
# see the truncated tail, and redistributes ~20% into it), but it takes orders
# of magnitude off the dominant axis, so the vector residual at f_n still wins
# by a wide margin. Swept 50 geometries (4-40 moves, 0.2-2 mm segments, 60-300
# mm/s, 3000-20000 mm/s^2): the null stops helping only past 1.29 deg per
# junction / 19.3 deg of TOTAL heading drift, worst case. Neither variable
# collapses the crossover on its own. A drift budget of a few degrees would keep
# the null on exactly the near-straight runs it still helps -- notably small
# arcs, the case Jerk_Limiting.md calls out (r=4 mm at 0.2 mm chords turns
# 2.87 deg per segment). Not taken: all of it is the COMMANDED spectrum on
# monotone turns, and real gcode zigzags.
SPAN_COLLINEAR_EPS = 1e-12


# Class to track a list of pending move requests and to facilitate
# "look-ahead" across moves to reduce acceleration between moves.
class LookAheadQueue:
    def __init__(self):
        self.queue = []
        self.junction_flush = LOOKAHEAD_FLUSH_TIME

    def reset(self):
        del self.queue[:]
        self.junction_flush = LOOKAHEAD_FLUSH_TIME

    def set_flush_time(self, flush_time):
        self.junction_flush = flush_time

    def get_last(self):
        if self.queue:
            return self.queue[-1]
        return None

    def flush(self, lazy=False):
        self.junction_flush = LOOKAHEAD_FLUSH_TIME
        update_flush_count = lazy
        queue = self.queue
        flush_count = len(queue)
        # Traverse queue from last to first move and determine maximum
        # junction speed assuming the robot comes to a complete stop
        # after the last move.
        delayed = []
        next_end_v2 = next_smoothed_v2 = peak_cruise_v2 = 0.0
        # The toolhead is reached via any queued move.
        th = queue[0].toolhead if queue else None
        unified = th is not None and getattr(th, "unified_emit", False)
        # Mirror of Move.span_start_* for the DECEL side: the speed the run
        # must be down to by its far (downstream) end, and how much of that run
        # lies past the end of the move being examined.
        span_end_v2 = 0.0
        span_after_d = 0.0
        next_move = None
        for i in range(flush_count - 1, -1, -1):
            move = queue[i]
            if next_move is None or not next_move.span_start_d:
                # No shared ramp across this junction: the decel run this move
                # feeds ends where the next move begins.
                #
                # Gated on span_start_d, not span_link, so the decel run is the
                # SAME chain _span_groups renders and the accel clamp sized
                # against. span_link only says two moves MAY share a ramp;
                # span_start_d says the planner actually put them in one run.
                span_end_v2 = next_end_v2
                span_after_d = 0.0
            span_total_d = span_after_d + move.move_d
            if span_after_d > 0.0:
                # Same cap as the accel side: never plan a boundary speed the
                # stock constant-accel trapezoid could not also honour if
                # shaping is later disabled.
                reachable_start_v2 = min(
                    th._span_reach_v2(move, span_end_v2, span_total_d),
                    next_end_v2 + move.delta_v2,
                )
            else:
                reachable_start_v2 = th._move_reach_v2(move, next_end_v2)
            start_v2 = min(move.max_start_v2, reachable_start_v2)
            reachable_smoothed_v2 = next_smoothed_v2 + move.smooth_delta_v2
            smoothed_v2 = min(move.max_smoothed_v2, reachable_smoothed_v2)
            if smoothed_v2 < reachable_smoothed_v2:
                # It's possible for this move to accelerate
                if (
                    smoothed_v2 + move.smooth_delta_v2 > next_smoothed_v2
                    or delayed
                ):
                    # This move can decelerate or this is a full accel
                    # move after a full decel move
                    if (
                        update_flush_count
                        and peak_cruise_v2
                        and (
                            not unified
                            or (
                                start_v2 >= move.max_start_v2 - 1e-9
                                # ... and this move inherits no runway from its
                                # predecessors. queue[:i] is emitted now and
                                # queue[i:] replanned later, so splitting where
                                # span_start_d > 0 hands the emitter a move
                                # whose cruise was clamped against runway that
                                # stayed behind in the queue.
                                and not move.span_start_d
                            )
                        )
                    ):
                        # A lazy flush emits queue[:i] now and replans the rest
                        # once more moves arrive, so the split is only sound if
                        # THIS move's start speed can no longer change. Adding
                        # moves can only push the eventual stop further away,
                        # which only ever RAISES reachable_start_v2, while
                        # max_start_v2 was fixed at add_move() time -- so the
                        # speed is settled exactly when it has already reached
                        # that cap.
                        #
                        # Stock gets this for free: it picks the split from the
                        # smoothed chain, which uses the same constant-accel law
                        # as the real one, so "smoothed and settled" implies
                        # "settled". Under the notch law the real chain
                        # decelerates far more slowly than smooth_delta_v2
                        # suggests, so the smoothed proxy declares a move
                        # settled while it is still reach-limited hundreds of
                        # moves deep in a decel ramp. Splitting there emitted a
                        # boundary at 264 mm/s that the replan then raised to
                        # 300 -- a 36 mm/s step, i.e. an infinite-accel impulse
                        # in exactly the band the notch exists to keep quiet.
                        flush_count = i
                        update_flush_count = False
                    peak_cruise_v2 = min(
                        move.max_cruise_v2,
                        (smoothed_v2 + reachable_smoothed_v2) * 0.5,
                    )
                    if delayed:
                        # Propagate peak_cruise_v2 to any delayed moves.
                        #
                        # No unified cruise clamp is needed here (unlike the
                        # branch below): mc_v2 is min()'d with each delayed
                        # move's own ms_v2, so cruise == start and the move has
                        # no accel ramp to make jerk-feasible. Its decel to
                        # min(me_v2, mc_v2) was already bounded by the reverse
                        # sweep, which took me_v2 from _move_reach_v2().
                        if not update_flush_count and i < flush_count:
                            mc_v2 = peak_cruise_v2
                            for m, ms_v2, me_v2 in reversed(delayed):
                                mc_v2 = min(mc_v2, ms_v2)
                                m.set_junction(
                                    min(ms_v2, mc_v2), mc_v2, min(me_v2, mc_v2)
                                )
                        del delayed[:]
                if not update_flush_count and i < flush_count:
                    cruise_v2 = min(
                        (start_v2 + reachable_start_v2) * 0.5,
                        move.max_cruise_v2,
                        peak_cruise_v2,
                    )
                    if th._uses_unified_reach(move):
                        # Jerk S-curve peak is below the constant-accel midpoint;
                        # clamp cruise to what is jerk-reachable from each end so
                        # the emitted profile stays jerk-feasible. Must use the
                        # SAME jerk law as the emitter or the clamp lets through
                        # a cruise the emitter cannot ramp to. Under spanning
                        # the emitter ramps over the whole run, so the clamp
                        # gets the run's runway too --
                        # otherwise the clamp, not the ramp, becomes the cliff.
                        fwd_d = move.span_start_d + move.move_d
                        if fwd_d > move.move_d:
                            accel_reach_v2 = min(
                                th._span_reach_v2(
                                    move, move.span_start_v2, fwd_d
                                ),
                                start_v2 + move.delta_v2,
                            )
                        else:
                            accel_reach_v2 = th._move_reach_v2(move, start_v2)
                        cruise_v2 = min(
                            cruise_v2,
                            accel_reach_v2,
                            reachable_start_v2,
                        )
                        # Restore the identity the stock planner gets for free.
                        # With constant accel, reachable_start_v2 is exactly
                        # next_end_v2 + delta_v2 and start_v2 is next_end_v2 -
                        # delta_v2 on a full-accel move, so the midpoint above
                        # lands EXACTLY on next_end_v2 and this move's end_v
                        # equals the next move's start_v. The notch law breaks
                        # that algebra -- reach is not start + delta -- and the
                        # midpoint then lands BELOW next_end_v2, which
                        # set_junction turns into a hard velocity step at the
                        # move boundary: 80 mm/s on 5 mm segments at 55 Hz, an
                        # infinite-accel impulse in the exact band the notch
                        # exists to keep quiet. next_end_v2 is jerk-reachable
                        # from start_v2 (notch_reach_v2 is direction-symmetric,
                        # and start_v2 was itself clamped to reach of
                        # next_end_v2), so lifting the cruise back onto it is
                        # always feasible.
                        cruise_v2 = max(
                            cruise_v2, min(next_end_v2, move.max_cruise_v2)
                        )
                    move.set_junction(
                        min(start_v2, cruise_v2),
                        cruise_v2,
                        min(next_end_v2, cruise_v2),
                    )
            else:
                # Delay calculating this move until peak_cruise_v2 is known
                delayed.append((move, start_v2, next_end_v2))
            if start_v2 < reachable_start_v2 - 1e-9:
                # The junction, not the ramp, is what binds here, so the decel
                # event does not need to reach any further upstream.
                span_end_v2 = start_v2
                span_after_d = 0.0
            else:
                span_after_d = span_total_d
            next_move = move
            next_end_v2 = start_v2
            next_smoothed_v2 = smoothed_v2
        if update_flush_count or not flush_count:
            return []
        # Remove processed moves from the queue
        res = queue[:flush_count]
        del queue[:flush_count]
        return res

    def add_move(self, move):
        self.queue.append(move)
        if len(self.queue) == 1:
            return
        move.calc_junction(self.queue[-2])
        self.junction_flush -= move.min_move_t
        # Check if enough moves have been queued to reach the target flush time.
        return self.junction_flush <= 0.0


BUFFER_TIME_LOW = 1.0
BUFFER_TIME_HIGH = 2.0
BUFFER_TIME_START = 0.250
BGFLUSH_LOW_TIME = 0.200
BGFLUSH_BATCH_TIME = 0.200
MIN_KIN_TIME = 0.100
MOVE_BATCH_TIME = 0.500
STEPCOMPRESS_FLUSH_TIME = 0.050
SDS_CHECK_TIME = 0.001  # step+dir+step filter in stepcompress.c
MOVE_HISTORY_EXPIRE = 30.0

DRIP_SEGMENT_TIME = 0.050
DRIP_TIME = 0.100


# Main code to track events (and their timing) on the printer toolhead
class ToolHead:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.all_mcus = [
            m for n, m in self.printer.lookup_objects(module="mcu")
        ]
        self.mcu = self.all_mcus[0]
        self.lookahead = LookAheadQueue()
        self.lookahead.set_flush_time(BUFFER_TIME_HIGH)
        self.commanded_pos = [0.0, 0.0, 0.0, 0.0]
        # Velocity and acceleration control
        self.max_velocity = config.getfloat("max_velocity", above=0.0)
        self.max_accel = config.getfloat("max_accel", above=0.0)
        self.min_cruise_ratio = config.getfloat(
            "minimum_cruise_ratio", 0.5, below=1.0, minval=0.0
        )
        self.square_corner_velocity = config.getfloat(
            "square_corner_velocity", 5.0, minval=0.0
        )
        # Jerk-limited motion (opt-in). When enabled, each move is rendered as
        # jerk-bounded constant-accel slices instead of one hard accel step, and
        # the lookahead plans the boundary speeds that ramp can reach.
        self.unified_emit = config.getboolean("unified_planner", False)
        # Ramp integration time step (s).
        self.unified_jerk_dt = config.getfloat(
            "unified_jerk_dt", 0.001, minval=0.0001
        )
        # Solve the emitted slice accelerations for an exact spectral null
        # instead of sampling the ideal ramp. See pathplan.solve_ramp_null.
        self.unified_spectral_null = config.getboolean(
            "unified_spectral_null", False
        )
        # Per-ramp notch law: park the jerk ramp's shaper zero on a fixed mode
        # frequency (Hz) via J = dv*f_n^2, instead of a fixed jerk. Peak accel
        # then self-scales as a_peak = dv*f_n. See pathplan.Constraints.ramp_jerk.
        # 0 disables the notch law.
        self.unified_notch_freq = config.getfloat(
            "unified_notch_freq", 0.0, minval=0.0
        )
        # Optional PER-AXIS notch. A jerk-limited accel ramp shapes the SCALAR
        # path speed, so a_x and a_y are the same a(t) scaled by the move's unit
        # direction -- one pulse serving both axes. That still admits TWO zeros,
        # because zeros multiply under convolution: a trapezoidal a(t) built as
        # rect(1/f_hi) * rect(1/f_lo) nulls both modes on every axis regardless
        # of heading (see _move_notch_pair / pathplan.Constraints.ramp_jerk).
        #   - unified_notch_freq alone notches BOTH axes at that frequency.
        #   - unified_notch_freq_x AND _y set the per-axis modes; they must be
        #     given together (setting exactly one is a config error).
        nfx = config.getfloat("unified_notch_freq_x", None, minval=0.0)
        nfy = config.getfloat("unified_notch_freq_y", None, minval=0.0)
        for name, f in (
            ("unified_notch_freq", self.unified_notch_freq),
            ("unified_notch_freq_x", nfx),
            ("unified_notch_freq_y", nfy),
        ):
            self._check_notch_freq(f, name, config.error)
        try:
            self.unified_notch_freq_x, self.unified_notch_freq_y = (
                self._resolve_notch(self.unified_notch_freq, nfx, nfy)
            )
        except ValueError as e:
            raise config.error(str(e))
        # Ramp spanning. A notched ramp always lasts 2/f_n seconds, so it
        # needs (v0+v1)/f_n of travel; a move shorter than that cannot change
        # speed at all, which otherwise pins a chain of L-mm segments to about
        # f_n*L mm/s no matter how high max_velocity/max_accel are. The answer
        # that does NOT move the zero: let a single notched ramp cover a
        # run of consecutive near-collinear moves, so the runway is the run's
        # length instead of one segment's. The ramp keeps its shape, so the
        # zero stays exactly on unified_notch_freq; only the requirement that
        # acceleration start and end at zero INSIDE every move is dropped.
        self.unified_span_ramps = config.getboolean("unified_span_ramps", True)
        # Largest heading change (degrees) one ramp may span. See SPAN_MAX_ANGLE
        # for where the default comes from: it holds the residual left on the
        # turning axis at or below the emitter's own discretization floor.
        self.unified_span_max_angle = config.getfloat(
            "unified_span_max_angle", SPAN_MAX_ANGLE, minval=0.0, maxval=90.0
        )
        self._sync_span_cos()
        self._check_unified_settings(config.error)
        self.orig_cfg = {}
        self.orig_cfg["max_velocity"] = self.max_velocity
        self.orig_cfg["max_accel"] = self.max_accel
        self.orig_cfg["min_cruise_ratio"] = self.min_cruise_ratio
        self.orig_cfg["square_corner_velocity"] = self.square_corner_velocity
        self.orig_cfg["unified_emit"] = self.unified_emit
        self.orig_cfg["unified_jerk_dt"] = self.unified_jerk_dt
        self.orig_cfg["unified_notch_freq"] = self.unified_notch_freq
        self.orig_cfg["unified_notch_freq_x"] = self.unified_notch_freq_x
        self.orig_cfg["unified_notch_freq_y"] = self.unified_notch_freq_y
        self.orig_cfg["unified_span_ramps"] = self.unified_span_ramps
        self.orig_cfg["unified_span_max_angle"] = self.unified_span_max_angle
        self._unified_warned = set()
        self._unified_all_warned = False
        self.junction_deviation = self.max_accel_to_decel = 0
        self._calc_junction_deviation()
        # Input stall detection
        self.check_stall_time = 0.0
        self.print_stall = 0
        # Input pause tracking
        self.can_pause = True
        if self.mcu.is_fileoutput():
            self.can_pause = False
        self.need_check_pause = -1.0
        # Print time tracking
        self.print_time = 0.0
        self.special_queuing_state = "NeedPrime"
        self.priming_timer = None
        # Flush tracking
        self.flush_timer = self.reactor.register_timer(self._flush_handler)
        self.do_kick_flush_timer = True
        self.last_flush_time = self.min_restart_time = 0.0
        self.need_flush_time = self.step_gen_time = self.clear_history_time = (
            0.0
        )
        # Kinematic step generation scan window time tracking
        self.kin_flush_delay = SDS_CHECK_TIME
        self.kin_flush_times = []
        # Setup iterative solver
        ffi_main, ffi_lib = chelper.get_ffi()
        self.trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
        self.trapq_append = ffi_lib.trapq_append
        self.trapq_finalize_moves = ffi_lib.trapq_finalize_moves
        # Motion flushing
        self.step_generators = []
        self.flush_trapqs = [self.trapq]
        # Create kinematics class
        gcode = self.printer.lookup_object("gcode")
        self.Coord = gcode.Coord
        extruder = kinematics_extruder.DummyExtruder(self.printer)
        self.extra_axes = [extruder]
        kin_name = config.get("kinematics")
        try:
            mod = importlib.import_module("klippy.kinematics." + kin_name)
            self.kin = mod.load_kinematics(self, config)
        except config.error as e:
            raise
        except self.printer.lookup_object("pins").error as e:
            raise
        except:
            msg = "Error loading kinematics '%s'" % (kin_name,)
            logging.exception(msg)
            raise config.error(msg)
        if (
            config.has_section("dual_carriage")
            and not self.kin.supports_dual_carriage
        ):
            raise config.error(
                "dual_carriage not compatible with '%s' kinematics system"
                % (kin_name,)
            )
        if hasattr(self.kin, "max_x_velocity"):
            self.orig_cfg["max_x_velocity"] = self.kin.max_x_velocity
        if hasattr(self.kin, "max_x_accel"):
            self.orig_cfg["max_x_accel"] = self.kin.max_x_accel
        if hasattr(self.kin, "max_y_velocity"):
            self.orig_cfg["max_y_velocity"] = self.kin.max_y_velocity
        if hasattr(self.kin, "max_y_accel"):
            self.orig_cfg["max_y_accel"] = self.kin.max_y_accel
        if hasattr(self.kin, "max_z_velocity"):
            self.orig_cfg["max_z_velocity"] = self.kin.max_z_velocity
        if hasattr(self.kin, "max_z_accel"):
            self.orig_cfg["max_z_accel"] = self.kin.max_z_accel

        # Register commands
        gcode.register_command("G4", self.cmd_G4)
        gcode.register_command("M400", self.cmd_M400)
        gcode.register_command(
            "SET_VELOCITY_LIMIT",
            self.cmd_SET_VELOCITY_LIMIT,
            desc=self.cmd_SET_VELOCITY_LIMIT_help,
        )
        gcode.register_command(
            "RESET_VELOCITY_LIMIT",
            self.cmd_RESET_VELOCITY_LIMIT,
            desc=self.cmd_RESET_VELOCITY_LIMIT_help,
        )
        gcode.register_command("M204", self.cmd_M204)
        gcode.register_command(
            "SET_UNIFIED",
            self.cmd_SET_UNIFIED,
            desc=self.cmd_SET_UNIFIED_help,
        )
        self.printer.register_event_handler(
            "klippy:shutdown", self._handle_shutdown
        )
        self.printer.register_event_handler(
            "klippy:connect", self._handle_connect
        )
        # Load some default modules
        modules = [
            "gcode_move",
            "homing",
            "idle_timeout",
            "statistics",
            "manual_probe",
            "tuning_tower",
            "garbage_collection",
        ]
        for module_name in modules:
            self.printer.load_object(config, module_name)

    def get_active_rails_for_axis(self, axis):
        # axis is 'x,y,z'
        active_rails = []
        rails = self.kin.rails
        for rail in rails:
            for stepper in rail.get_steppers():
                if stepper.is_active_axis(axis):
                    active_rails.append(rail)
                    break
        return active_rails

    # Print time and flush tracking
    def _advance_flush_time(self, flush_time):
        flush_time = max(flush_time, self.last_flush_time)
        # Generate steps via itersolve
        sg_flush_want = min(
            flush_time + STEPCOMPRESS_FLUSH_TIME,
            self.print_time - self.kin_flush_delay,
        )
        sg_flush_time = max(sg_flush_want, flush_time)
        for sg in self.step_generators:
            sg(sg_flush_time)
        self.min_restart_time = max(self.min_restart_time, sg_flush_time)
        # Free trapq entries that are no longer needed
        clear_history_time = self.clear_history_time
        if not self.can_pause:
            clear_history_time = flush_time - MOVE_HISTORY_EXPIRE
        free_time = sg_flush_time - self.kin_flush_delay
        for trapq in self.flush_trapqs:
            self.trapq_finalize_moves(trapq, free_time, clear_history_time)
        # Flush stepcompress and mcu steppersync
        for m in self.all_mcus:
            m.flush_moves(flush_time, clear_history_time)
        self.last_flush_time = flush_time

    def _advance_move_time(self, next_print_time):
        pt_delay = self.kin_flush_delay + STEPCOMPRESS_FLUSH_TIME
        flush_time = max(self.last_flush_time, self.print_time - pt_delay)
        self.print_time = max(self.print_time, next_print_time)
        want_flush_time = max(flush_time, self.print_time - pt_delay)
        while 1:
            flush_time = min(flush_time + MOVE_BATCH_TIME, want_flush_time)
            self._advance_flush_time(flush_time)
            if flush_time >= want_flush_time:
                break

    def _calc_print_time(self):
        curtime = self.reactor.monotonic()
        est_print_time = self.mcu.estimated_print_time(curtime)
        kin_time = max(est_print_time + MIN_KIN_TIME, self.min_restart_time)
        kin_time += self.kin_flush_delay
        min_print_time = max(est_print_time + BUFFER_TIME_START, kin_time)
        if min_print_time > self.print_time:
            self.print_time = min_print_time
            self.printer.send_event(
                "toolhead:sync_print_time",
                curtime,
                est_print_time,
                self.print_time,
            )

    def _emit_slices(self, move, segs, next_move_time):
        # Push one move's constant-accel slices into the toolhead trapq and
        # into every extra axis, and return the time the move ends at.
        t = next_move_time
        pos = 0.0
        for at, ct, dt, sv, cv, a, dist in segs:
            self.trapq_append(
                self.trapq,
                t,
                at,
                ct,
                dt,
                move.start_pos[0] + move.axes_r[0] * pos,
                move.start_pos[1] + move.axes_r[1] * pos,
                move.start_pos[2] + move.axes_r[2] * pos,
                move.axes_r[0],
                move.axes_r[1],
                move.axes_r[2],
                sv,
                cv,
                a,
            )
            for e_index, ea in enumerate(self.extra_axes):
                if not move.axes_d[e_index + 3]:
                    continue
                if not hasattr(ea, "process_move_segment"):
                    raise self.printer.command_error(
                        "unified_planner requires extra axis '%s' to"
                        " implement process_move_segment"
                        % (ea.get_axis_gcode_id(),)
                    )
                ea.process_move_segment(
                    t, move, e_index + 3, at, ct, dt, sv, cv, a, dist
                )
            t += at + ct + dt
            pos += dist
        for e_index, ea in enumerate(self.extra_axes):
            if move.axes_d[e_index + 3] and hasattr(ea, "sync_position"):
                ea.sync_position(move, e_index + 3)
        return t

    def _span_groups(self, moves):
        # Split the flush batch into the runs the LOOKAHEAD already planned.
        # Yields (group, peak_v, cap_v).
        #
        # Membership is NOT re-derived here. span_start_d is the planner's own
        # record of "this move inherits runway from its predecessor", and the
        # cruise clamp sized every one of these moves against exactly that run
        # (see LookAheadQueue.flush and Move.calc_junction). Deciding
        # membership a second time by different rules is what made the emitter
        # render a different set of moves than the clamp had assumed: the plan
        # was feasible over the run the planner had in mind and impossible over
        # the group the emitter built, so validation shut the printer down --
        # 47 such moves in one 2 h print, every one of them a move with
        # inherited runway. One source of truth makes that unrepresentable
        # instead of patching its symptoms one at a time.
        #
        # The structural tests the old code repeated here -- shared direction,
        # matching notch target, a junction that does not bind -- are all
        # already conditions on span_start_d being carried forward at all.
        # Velocity continuity inside the run needs no check because the run is
        # rendered as ONE profile and then split by distance, which supplies
        # the interior speeds itself.
        if not self.unified_span_ramps or not self.unified_emit:
            return
        group = []
        peak = cap = 0.0
        for move in moves:
            usable = move.is_kinematic_move and self._uses_unified_reach(move)
            if group and usable and move.span_start_d > 0.0:
                group.append(move)
                cap = min(cap, math.sqrt(move.max_cruise_v2))
                peak = max(peak, move.cruise_v)
                continue
            # The run ends here. Yield it whatever ENDED it -- a run is just as
            # often ended by a move that is not usable at all as by one that
            # is. A bed-mesh-split travel is bookended by G10/G11, which are
            # extruder-only and so not kinematic; closing the group only on the
            # usable path silently DISCARDED every such run.
            if len(group) > 1:
                yield group, peak, cap
            group = [move] if usable else []
            if usable:
                cap = min(math.sqrt(move.max_cruise_v2), self.max_velocity)
                peak = move.cruise_v
        if len(group) > 1:
            yield group, peak, cap

    def _group_is_collinear(self, group):
        # Does every move in a run point the same way?
        #
        # A spanned run renders ONE scalar profile and splits it among its
        # members, and each axis sees a_i(t) = r_i(t)*a(t). If r_i is CONSTANT
        # the scalar null transfers to every axis unchanged -- a straight run
        # is exactly as safe as a single move. It is only a direction change
        # inside the pulse that makes r_i time-varying and breaks the null.
        #
        # unified_span_max_angle bounds each JUNCTION, not the run, so heading
        # can drift across many small turns. Compare against the run's first
        # move rather than pairwise.
        first = group[0].axes_r
        for move in group:
            r = move.axes_r
            if (
                abs(r[0] - first[0]) > SPAN_COLLINEAR_EPS
                or abs(r[1] - first[1]) > SPAN_COLLINEAR_EPS
                or abs(r[2] - first[2]) > SPAN_COLLINEAR_EPS
            ):
                return False
        return True

    def _span_plans(self, moves):
        # Validate each multi-move run without collecting slices. Return a
        # lightweight {id(move): (plan, index)} map; _process_lookahead renders
        # and splits one plan at a time when its first move reaches emission.
        out = {}
        for group, peak, cap in self._span_groups(moves):
            first, last = group[0], group[-1]
            vs, ve = first.start_v, last.end_v
            vc = max(min(peak, cap), vs, ve)
            total_d = math.fsum([m.move_d for m in group])
            # The run is rendered as ONE profile, so its acceleration ceiling
            # has to be the tightest any member allows -- members can differ
            # under a direction-dependent accel limit.
            a_span = min(m.accel for m in group)
            cons = self._pathplan_cons(
                first,
                v_ceil=vc + 1.0,
                a_const=a_span,
                for_span=not self._group_is_collinear(group),
            )
            pathplan.validate_profile(vs, vc, ve, total_d, cons)
            plan = (group, vs, vc, ve, total_d, cons)
            for index, move in enumerate(group):
                out[id(move)] = (plan, index)
        return out

    def _process_lookahead(self, lazy=False):
        moves = self.lookahead.flush(lazy=lazy)
        if not moves:
            return
        # Validate every unified profile before mutating print-time or any
        # motion queue. Validation runs the emitter's distance recurrence
        # without collecting slices, keeping memory bounded for dense batches.
        span_plans = self._span_plans(moves)
        for move in moves:
            if (
                id(move) in span_plans
                or not self.unified_emit
                or not move.is_kinematic_move
            ):
                continue
            cons = self._pathplan_cons(move)
            pathplan.validate_profile(
                move.start_v,
                move.cruise_v,
                move.end_v,
                move.move_d,
                cons,
            )
        # Resync print_time if necessary
        if self.special_queuing_state:
            # Transition from "NeedPrime"/"Priming" state to main state
            self.special_queuing_state = ""
            self.need_check_pause = -1.0
            self._calc_print_time()
        # Queue moves into trapezoid motion queue (trapq)
        next_move_time = self.print_time
        span_buckets = {}
        for move in moves:
            span_item = span_plans.get(id(move))
            segs = span_buckets.pop(id(move), None)
            if span_item is not None and span_item[1] == 0:
                group, vs, vc, ve, total_d, cons = span_item[0]
                rendered = pathplan._render_validated_profile(
                    vs, vc, ve, total_d, cons
                )
                buckets = pathplan.split_segments(
                    rendered, [m.move_d for m in group]
                )
                if not all(buckets):
                    raise AssertionError(
                        "validated span produced an empty move"
                    )
                span_buckets.update(
                    (id(member), bucket)
                    for member, bucket in zip(group, buckets)
                )
                segs = span_buckets.pop(id(move))
                if not self._unified_all_warned:
                    self._warn_unified_profile(
                        group[-1],
                        pathplan.notch_loss_reasons(vs, vc, ve, cons),
                    )
            elif segs is None and self.unified_emit and move.is_kinematic_move:
                cons = self._pathplan_cons(move)
                segs = (
                    pathplan._render_validated_profile(
                        move.start_v,
                        move.cruise_v,
                        move.end_v,
                        move.move_d,
                        cons,
                    )
                    or None
                )
                if segs and not self._unified_all_warned:
                    self._warn_unified_profile(
                        move,
                        pathplan.notch_loss_reasons(
                            move.start_v,
                            move.cruise_v,
                            move.end_v,
                            cons,
                        ),
                    )
            if segs is not None:
                # Emit the jerk-limited profile as a chain of constant-accel
                # slices, driving the toolhead trapq and each extra axis (the
                # extruder) in lock-step so pressure advance integrates the real
                # profile. The profile's duration differs from the nominal
                # trapezoid (a_peak = dv*f_n < max_accel), so advance
                # next_move_time by the ACTUAL emitted duration.
                next_move_time = self._emit_slices(move, segs, next_move_time)
            else:
                if move.is_kinematic_move:
                    self.trapq_append(
                        self.trapq,
                        next_move_time,
                        move.accel_t,
                        move.cruise_t,
                        move.decel_t,
                        move.start_pos[0],
                        move.start_pos[1],
                        move.start_pos[2],
                        move.axes_r[0],
                        move.axes_r[1],
                        move.axes_r[2],
                        move.start_v,
                        move.cruise_v,
                        move.accel,
                    )
                for e_index, ea in enumerate(self.extra_axes):
                    if move.axes_d[e_index + 3]:
                        ea.process_move(next_move_time, move, e_index + 3)
                # Keep this sum grouped EXACTLY as upstream computes it. Folding
                # it into a precomputed duration regroups the float addition to
                # nmt + (a+c+d) instead of ((nmt+a)+c)+d, which shifts
                # print_time by an ULP and trips "Internal error in
                # stepcompress" on long extruder-only runs.
                next_move_time = (
                    next_move_time + move.accel_t + move.cruise_t + move.decel_t
                )
            for cb in move.timing_callbacks:
                cb(next_move_time)
        # Generate steps for moves
        self.note_mcu_movequeue_activity(
            next_move_time + self.kin_flush_delay, set_step_gen_time=True
        )
        self._advance_move_time(next_move_time)

    def _flush_lookahead(self):
        # Transit from "NeedPrime"/"Priming"/"Drip"/main state to "NeedPrime"
        self._process_lookahead()
        self.special_queuing_state = "NeedPrime"
        self.need_check_pause = -1.0
        self.lookahead.set_flush_time(BUFFER_TIME_HIGH)
        self.check_stall_time = 0.0

    def flush_step_generation(self):
        self._flush_lookahead()
        self._advance_flush_time(self.step_gen_time)
        self.min_restart_time = max(self.min_restart_time, self.print_time)

    def get_last_move_time(self):
        if self.special_queuing_state:
            self._flush_lookahead()
            self._calc_print_time()
        else:
            self._process_lookahead()
        return self.print_time

    def _check_pause(self):
        eventtime = self.reactor.monotonic()
        est_print_time = self.mcu.estimated_print_time(eventtime)
        buffer_time = self.print_time - est_print_time
        if self.special_queuing_state:
            if self.check_stall_time:
                # Was in "NeedPrime" state and got there from idle input
                if est_print_time < self.check_stall_time:
                    self.print_stall += 1
                self.check_stall_time = 0.0
            # Transition from "NeedPrime"/"Priming" state to "Priming" state
            self.special_queuing_state = "Priming"
            self.need_check_pause = -1.0
            if self.priming_timer is None:
                self.priming_timer = self.reactor.register_timer(
                    self._priming_handler
                )
            wtime = eventtime + max(0.100, buffer_time - BUFFER_TIME_LOW)
            self.reactor.update_timer(self.priming_timer, wtime)
        # Check if there are lots of queued moves and pause if so
        while True:
            pause_time = buffer_time - BUFFER_TIME_HIGH
            if pause_time <= 0.0:
                break
            if not self.can_pause:
                self.need_check_pause = self.reactor.NEVER
                return
            eventtime = self.reactor.pause(eventtime + min(1.0, pause_time))
            est_print_time = self.mcu.estimated_print_time(eventtime)
            buffer_time = self.print_time - est_print_time
        if not self.special_queuing_state:
            # In main state - defer pause checking until needed
            self.need_check_pause = est_print_time + BUFFER_TIME_HIGH + 0.100

    def _priming_handler(self, eventtime):
        self.reactor.unregister_timer(self.priming_timer)
        self.priming_timer = None
        try:
            if self.special_queuing_state == "Priming":
                self._flush_lookahead()
                self.check_stall_time = self.print_time
        except:
            logging.exception("Exception in priming_handler")
            self.printer.invoke_shutdown("Exception in priming_handler")
        return self.reactor.NEVER

    def _flush_handler(self, eventtime):
        try:
            est_print_time = self.mcu.estimated_print_time(eventtime)
            if not self.special_queuing_state:
                # In "main" state - flush lookahead if buffer runs low
                print_time = self.print_time
                buffer_time = print_time - est_print_time
                if buffer_time > BUFFER_TIME_LOW:
                    # Running normally - reschedule check
                    return eventtime + buffer_time - BUFFER_TIME_LOW
                # Under ran low buffer mark - flush lookahead queue
                self._flush_lookahead()
                if print_time != self.print_time:
                    self.check_stall_time = self.print_time
            # In "NeedPrime"/"Priming" state - flush queues if needed
            while 1:
                end_flush = (
                    self.need_flush_time
                    + get_danger_options().bgflush_extra_time
                )
                if self.last_flush_time >= end_flush:
                    self.do_kick_flush_timer = True
                    return self.reactor.NEVER
                buffer_time = self.last_flush_time - est_print_time
                if buffer_time > BGFLUSH_LOW_TIME:
                    return eventtime + buffer_time - BGFLUSH_LOW_TIME
                ftime = est_print_time + BGFLUSH_LOW_TIME + BGFLUSH_BATCH_TIME
                self._advance_flush_time(min(end_flush, ftime))
        except:
            logging.exception("Exception in flush_handler")
            self.printer.invoke_shutdown("Exception in flush_handler")
        return self.reactor.NEVER

    # Movement commands
    def get_position(self):
        return list(self.commanded_pos)

    def set_position(self, newpos, homing_axes=""):
        self.flush_step_generation()
        ffi_main, ffi_lib = chelper.get_ffi()
        ffi_lib.trapq_set_position(
            self.trapq, self.print_time, newpos[0], newpos[1], newpos[2]
        )
        self.commanded_pos[:3] = newpos[:3]
        self.kin.set_position(newpos, homing_axes)
        self.printer.send_event("toolhead:set_position")

    def limit_next_junction_speed(self, speed):
        last_move = self.lookahead.get_last()
        if last_move is not None:
            last_move.limit_next_junction_speed(speed)

    def move(self, newpos, speed):
        move = Move(self, self.commanded_pos, newpos, speed)
        if not move.move_d:
            return
        if move.is_kinematic_move:
            self.kin.check_move(move)
        for e_index, ea in enumerate(self.extra_axes):
            if move.axes_d[e_index + 3]:
                ea.check_move(move, e_index + 3)
        self.commanded_pos[:] = move.end_pos
        want_flush = self.lookahead.add_move(move)
        if want_flush:
            self._process_lookahead(lazy=True)
        if self.print_time > self.need_check_pause:
            self._check_pause()

    def manual_move(self, coord, speed):
        curpos = list(self.commanded_pos)
        for i in range(len(coord)):
            if coord[i] is not None:
                curpos[i] = coord[i]
        self.move(curpos, speed)
        self.printer.send_event("toolhead:manual_move")

    def dwell(self, delay):
        next_print_time = self.get_last_move_time() + max(0.0, delay)
        self._advance_move_time(next_print_time)
        self._check_pause()

    def wait_moves(self):
        self._flush_lookahead()
        eventtime = self.reactor.monotonic()
        while (
            not self.special_queuing_state
            or self.print_time >= self.mcu.estimated_print_time(eventtime)
        ):
            if not self.can_pause:
                break
            eventtime = self.reactor.pause(eventtime + 0.100)

    def set_extruder(self, extruder, extrude_pos):
        # XXX - should use add_extra_axis
        if self.unified_emit and not hasattr(extruder, "process_move_segment"):
            raise self.printer.command_error(
                "unified_planner requires extra axis '%s' to implement"
                " process_move_segment" % (extruder.get_axis_gcode_id(),)
            )
        prev_ea_trapq = self.extra_axes[0].get_trapq()
        if prev_ea_trapq in self.flush_trapqs:
            self.flush_trapqs.remove(prev_ea_trapq)
        self.extra_axes[0] = extruder
        self.commanded_pos[3] = extrude_pos
        ea_trapq = extruder.get_trapq()
        if ea_trapq is not None:
            self.flush_trapqs.append(ea_trapq)

    def get_extruder(self):
        return self.extra_axes[0]

    def add_extra_axis(self, ea, axis_pos):
        self._flush_lookahead()
        if self.unified_emit and not hasattr(ea, "process_move_segment"):
            raise self.printer.command_error(
                "unified_planner requires extra axis '%s' to implement"
                " process_move_segment" % (ea.get_axis_gcode_id(),)
            )
        self.extra_axes.append(ea)
        self.commanded_pos.append(axis_pos)
        ea_trapq = ea.get_trapq()
        if ea_trapq is not None:
            self.flush_trapqs.append(ea_trapq)
        self.printer.send_event("toolhead:update_extra_axes")

    def remove_extra_axis(self, ea):
        self._flush_lookahead()
        if ea not in self.extra_axes:
            return
        ea_index = self.extra_axes.index(ea) + 3
        ea_trapq = ea.get_trapq()
        if ea_trapq in self.flush_trapqs:
            self.flush_trapqs.remove(ea_trapq)
        self.commanded_pos.pop(ea_index)
        self.extra_axes.pop(ea_index - 3)
        self.printer.send_event("toolhead:update_extra_axes")

    def get_extra_axes(self):
        return [None, None, None] + self.extra_axes

    # Homing "drip move" handling
    def drip_update_time(self, next_print_time, drip_completion, addstepper=()):
        # Transition from "NeedPrime"/"Priming"/main state to "Drip" state
        self.special_queuing_state = "Drip"
        self.need_check_pause = self.reactor.NEVER
        self.reactor.update_timer(self.flush_timer, self.reactor.NEVER)
        self.do_kick_flush_timer = False
        self.lookahead.set_flush_time(BUFFER_TIME_HIGH)
        self.check_stall_time = 0.0
        # Update print_time in segments until drip_completion signal
        flush_delay = DRIP_TIME + STEPCOMPRESS_FLUSH_TIME + self.kin_flush_delay
        while self.print_time < next_print_time:
            if drip_completion.test():
                break
            curtime = self.reactor.monotonic()
            est_print_time = self.mcu.estimated_print_time(curtime)
            wait_time = self.print_time - est_print_time - flush_delay
            if wait_time > 0.0 and self.can_pause:
                # Pause before sending more steps
                drip_completion.wait(curtime + wait_time)
                continue
            npt = min(self.print_time + DRIP_SEGMENT_TIME, next_print_time)
            self.note_mcu_movequeue_activity(
                npt + self.kin_flush_delay, set_step_gen_time=True
            )
            for stepper in addstepper:
                stepper.generate_steps(npt)
            self._advance_move_time(npt)
        # Exit "Drip" state
        self.reactor.update_timer(self.flush_timer, self.reactor.NOW)
        self.flush_step_generation()

    def _drip_load_trapq(self, submit_move):
        # Queue move into trapezoid motion queue (trapq)
        if submit_move.move_d:
            self.commanded_pos[:] = submit_move.end_pos
            self.lookahead.add_move(submit_move)
        moves = self.lookahead.flush()
        self._calc_print_time()
        next_move_time = self.print_time
        for move in moves:
            self.trapq_append(
                self.trapq,
                next_move_time,
                move.accel_t,
                move.cruise_t,
                move.decel_t,
                move.start_pos[0],
                move.start_pos[1],
                move.start_pos[2],
                move.axes_r[0],
                move.axes_r[1],
                move.axes_r[2],
                move.start_v,
                move.cruise_v,
                move.accel,
            )
            next_move_time = (
                next_move_time + move.accel_t + move.cruise_t + move.decel_t
            )
        self.lookahead.reset()
        return next_move_time

    def drip_move(self, newpos, speed, drip_completion):
        # Create and verify move is valid
        newpos = newpos[:3] + self.commanded_pos[3:]
        move = Move(self, self.commanded_pos, newpos, speed)
        if move.move_d:
            self.kin.check_move(move)
        # Make sure stepper movement doesn't start before nominal start time
        self.dwell(self.kin_flush_delay)
        # Transmit move in "drip" mode
        self._process_lookahead()
        next_move_time = self._drip_load_trapq(move)
        self.drip_update_time(next_move_time, drip_completion)
        # Move finished; cleanup any remnants on trapq
        self.trapq_finalize_moves(self.trapq, self.reactor.NEVER, 0)

    # Misc commands
    def stats(self, eventtime):
        max_queue_time = max(self.print_time, self.last_flush_time)
        for m in self.all_mcus:
            m.check_active(max_queue_time, eventtime)
        est_print_time = self.mcu.estimated_print_time(eventtime)
        self.clear_history_time = est_print_time - MOVE_HISTORY_EXPIRE
        buffer_time = self.print_time - est_print_time
        is_active = buffer_time > -60.0 or not self.special_queuing_state
        if self.special_queuing_state == "Drip":
            buffer_time = 0.0
        return is_active, "print_time=%.3f buffer_time=%.3f print_stall=%d" % (
            self.print_time,
            max(buffer_time, 0.0),
            self.print_stall,
        )

    def check_busy(self, eventtime):
        est_print_time = self.mcu.estimated_print_time(eventtime)
        lookahead_empty = not self.lookahead.queue
        return self.print_time, est_print_time, lookahead_empty

    def get_status(self, eventtime):
        print_time = self.print_time
        estimated_print_time = self.mcu.estimated_print_time(eventtime)
        extruder = self.extra_axes[0]
        res = dict(self.kin.get_status(eventtime))
        res.update(
            {
                "print_time": print_time,
                "stalls": self.print_stall,
                "estimated_print_time": estimated_print_time,
                "extruder": extruder.get_name(),
                "position": self.Coord(*self.commanded_pos[:4]),
                "max_velocity": self.max_velocity,
                "max_accel": self.max_accel,
                "minimum_cruise_ratio": self.min_cruise_ratio,
                "square_corner_velocity": self.square_corner_velocity,
            }
        )
        return res

    def _handle_shutdown(self):
        self.can_pause = False
        self.lookahead.reset()

    def _handle_connect(self):
        # kin exists by now, so the Z/XY actuator coupling can be resolved.
        self._z_couples_xy_cache = None
        self._z_couples_xy()
        self._check_unified_extra_axis_support(self.printer.command_error)

    def get_kinematics(self):
        return self.kin

    def get_trapq(self):
        return self.trapq

    def register_step_generator(self, handler):
        self.step_generators.append(handler)

    def unregister_step_generator(self, handler):
        if handler in self.step_generators:
            self.step_generators.remove(handler)

    def note_step_generation_scan_time(self, delay, old_delay=0.0):
        self.flush_step_generation()
        if old_delay:
            self.kin_flush_times.pop(self.kin_flush_times.index(old_delay))
        if delay:
            self.kin_flush_times.append(delay)
        new_delay = max(self.kin_flush_times + [SDS_CHECK_TIME])
        self.kin_flush_delay = new_delay

    def register_lookahead_callback(self, callback):
        last_move = self.lookahead.get_last()
        if last_move is None:
            callback(self.get_last_move_time())
            return
        last_move.timing_callbacks.append(callback)

    def note_mcu_movequeue_activity(self, mq_time, set_step_gen_time=False):
        self.need_flush_time = max(self.need_flush_time, mq_time)
        if set_step_gen_time:
            self.step_gen_time = max(self.step_gen_time, mq_time)
        if self.do_kick_flush_timer:
            self.do_kick_flush_timer = False
            self.reactor.update_timer(self.flush_timer, self.reactor.NOW)

    def get_max_velocity(self):
        return self.max_velocity, self.max_accel

    def _calc_junction_deviation(self):
        scv2 = self.square_corner_velocity**2
        self.junction_deviation = scv2 * (math.sqrt(2.0) - 1.0) / self.max_accel
        self.max_accel_to_decel = self.max_accel * (1.0 - self.min_cruise_ratio)

    # A notched ramp always lasts 2/f_n seconds and eats (v0+v1)/f_n of path
    # distance, so a very low f_n does not just shape gently -- it stalls the
    # machine. At 1 Hz a ramp is 2 s long and needs 100 mm of runway to reach
    # 50 mm/s. No printer structural mode sits below this floor, and the values
    # that land here in practice are typos (0.55 for 55).
    MIN_NOTCH_FREQ = 5.0

    @classmethod
    def _check_notch_freq(cls, freq, name, error_factory):
        if freq is None or freq <= 0.0:
            return  # 0/unset = notch disabled
        if freq < cls.MIN_NOTCH_FREQ:
            raise error_factory(
                "%s must be 0 or at least %.0f Hz (got %.4f); a notch below"
                " that stalls the toolhead -- each ramp takes 2/f_n seconds"
                % (name, cls.MIN_NOTCH_FREQ, freq)
            )

    @staticmethod
    def _resolve_notch(shared, nfx, nfy):
        # Resolve the notch config into a per-axis pair (f_x, f_y).
        #   shared   -> unified_notch_freq (0 = off), notches BOTH axes.
        #   nfx/nfy  -> unified_notch_freq_x/_y, or None when absent.
        # Per-axis values override `shared` but must be given as a pair: setting
        # exactly one is ambiguous (which axis keeps the shared value?), so it is
        # rejected. Raises ValueError on that case; the caller maps it to a
        # config error.
        if (nfx is None) != (nfy is None):
            raise ValueError(
                "unified_notch_freq_x and unified_notch_freq_y must be set"
                " together. Use unified_notch_freq to notch both axes at one"
                " frequency, or set BOTH unified_notch_freq_x and"
                " unified_notch_freq_y."
            )
        if nfx is None:
            return shared, shared
        return nfx, nfy

    def _notch_on(self):
        # True when a notch frequency is configured on either axis. The shared
        # unified_notch_freq is folded into both at config time (_resolve_notch).
        return bool(self.unified_notch_freq_x or self.unified_notch_freq_y)

    def _move_notch_freq(self, move):
        # Scalar notch frequency (Hz) for one move; 0.0 = off.
        #
        # This is the EQUIVALENT frequency of the move's notch pair -- their
        # harmonic mean -- which is what governs ramp distance and runway
        # ((v0+v1)/f_eq) everywhere in the planner. With one zero the pair is
        # (f, f) and f_eq is just f, so every runway formula is unchanged.
        f_lo, f_hi = self._move_notch_pair(move)
        if not f_lo or f_lo == f_hi:
            # Exact, not just close: 2/(1/f + 1/f) does not always round-trip to
            # f, and callers compare this value between moves to decide whether
            # one ramp may span them.
            return f_lo
        return 2.0 / (1.0 / f_lo + 1.0 / f_hi)

    def _move_notch_pair(self, move):
        # The (f_lo, f_hi) pair of mode frequencies this move's ramp nulls;
        # (0.0, 0.0) = off.
        #
        # A jerk-limited accel ramp shapes the SCALAR path speed, so the axes
        # see a_x(t) = rx*a(t) and a_y(t) = ry*a(t) -- one pulse, scaled. That
        # does NOT mean one zero: a(t) = rect(1/f_hi) * rect(1/f_lo) is a
        # trapezoid with exact zeros at BOTH frequencies, and every axis
        # inherits both. So naming two modes IS the request for two zeros:
        # unified_notch_freq_x != unified_notch_freq_y selects the trapezoid,
        # and the ramp shape then stops depending on heading entirely (see
        # pathplan.Constraints.ramp_jerk).
        #
        # There is no separate enable. f_x == f_y collapses the trapezoid back
        # to the triangle by construction, so the shared unified_notch_freq --
        # folded into both axes at config time by _resolve_notch -- yields the
        # single-zero ramp for every direction with no special case anywhere.
        #
        # Memoized on the move: this is called from _uses_unified_reach, which
        # runs on every lookahead reach probe (several per move per flush pass).
        # Safe because the notch config can only change via SET_UNIFIED, which
        # calls flush_step_generation() first and so retires every queued move.
        pair = getattr(move, "_unified_notch_pair", None)
        if pair is not None:
            return pair
        f = None
        fx = self.unified_notch_freq_x
        fy = self.unified_notch_freq_y
        if not fx and not fy:
            f = 0.0
        else:
            if abs(move.axes_r[0]) + abs(move.axes_r[1]) <= 1e-12:
                # No XY motion (Z-only or extrude-only). Whether that means
                # "nothing to cancel" depends on the kinematics.
                #
                # When Z has its own actuator (cartesian, CoreXY), the ramp
                # shapes the scalar path speed and the axes see a_x = rx*a(t),
                # a_y = ry*a(t) with rx = ry = 0 -- identically zero for ANY
                # a(t). No ramp shape changes the XY excitation, so notching
                # buys no quiet while costing the full 2/f_n ramp (36 ms at
                # 55 Hz) and its (v0+v1)/f_n of runway, which a Z hop does not
                # have. That is why notching a decoupled Z lift is wasteful.
                #
                # When Z shares actuators with X or Y -- CoreXZ and hybrid
                # CoreXZ (A = x + z, B = x - z), deltesian, delta, rotary
                # delta, cable winch -- a Z-only move accelerates the very
                # belts whose compliance the notch is aimed at. Commanded X
                # stays zero only to first order: belt-stiffness or motor-lag
                # asymmetry leaves a residual common-mode term that couples
                # straight into the XY mode. Keep shaping there.
                if not self._z_couples_xy():
                    f = 0.0
                elif not (fx and fy):
                    f = fx or fy
                # else: fall through to the pair -- a coupled Z move excites
                # whatever the belts carry, so null both modes.
            elif not (fx and fy):
                # Only one axis names a mode, so there is no second zero to
                # place. (_resolve_notch rejects this at config time; SET_UNIFIED
                # can transiently produce it before its own pair check runs.)
                f = fx or fy
        if f is not None:
            pair = (f, f)
        else:
            pair = (min(fx, fy), max(fx, fy))
        move._unified_notch_pair = pair
        return pair

    def _pathplan_cons(self, move, v_ceil=None, a_const=None, for_span=False):
        # Build the pathplan.Constraints for one move. a_const is the move's
        # constant acceleration ceiling; with a notch frequency the emitter
        # governs ordinary moves via a_peak = dv*f_n instead.
        # v_ceil is overridden when the constraints describe a whole spanning
        # run rather than this one move.
        f_lo, f_hi = self._move_notch_pair(move)
        notch = f_lo or None
        notch2 = f_hi or None
        if v_ceil is not None:
            pass
        elif hasattr(move, "cruise_v"):
            v_ceil = max(move.cruise_v, move.start_v, move.end_v) + 1.0
        else:
            # Reverse lookahead calls this before set_junction() has created
            # start_v/cruise_v/end_v. Use the move's configured cruise limit
            # as the conservative ceiling for reach calculations.
            v_ceil = math.sqrt(move.max_cruise_v2) + 1.0
        want_null = getattr(self, "unified_spectral_null", False)
        cons = pathplan.Constraints(
            a_const=move.accel if a_const is None else a_const,
            v_ceil=v_ceil,
            jerk_dt=self.unified_jerk_dt,
            notch_freq=notch,
            notch_freq2=notch2,
            # for_span is set only for a run that CHANGES DIRECTION. One scalar
            # profile is rendered across the whole run and split among its
            # moves, and each axis sees a_i(t) = r_i(t)*a(t) -- with r_i
            # constant, as on a straight run, a solved scalar null transfers to
            # every axis unchanged and spanning is exactly as safe as a single
            # move. Only a turn inside the pulse makes r_i time-varying and
            # breaks it. See ToolHead._group_is_collinear.
            spectral_null=want_null and not for_span,
        )
        if for_span and want_null:
            # Say so rather than quietly emitting an uncorrected run.
            cons.spectral_loss = "spectral_null_span_turned"
        return cons

    def _z_couples_xy(self):
        # Does a Z-only cartesian move drive the same actuators as X or Y?
        # True for CoreXZ, hybrid CoreXZ, deltesian, delta, rotary delta and
        # cable winch; False for cartesian, CoreXY and polar, where Z has a
        # dedicated stepper. Resolved from the itersolve
        # active-axis flags at connect time, so it needs no per-kinematics
        # table and no config. Unknown kinematics answer True: keeping the
        # shaping is the conservative direction, since the cost is throughput
        # on Z moves rather than ringing.
        coupled = getattr(self, "_z_couples_xy_cache", None)
        if coupled is not None:
            return coupled
        coupled = True
        rails = getattr(getattr(self, "kin", None), "rails", None)
        if rails:
            coupled = False
            for rail in rails:
                for s in rail.get_steppers():
                    if s.is_active_axis("z") and (
                        s.is_active_axis("x") or s.is_active_axis("y")
                    ):
                        coupled = True
                        break
                if coupled:
                    break
        self._z_couples_xy_cache = coupled
        return coupled

    def _uses_unified_reach(self, move):
        if (
            not getattr(self, "unified_emit", False)
            or not move.is_kinematic_move
        ):
            return False
        return self._move_notch_freq(move) > 0.0

    def _span_link_ok(self, prev_move, move, junction_cos_theta):
        # May these two consecutive moves share ONE accel ramp? Only structure
        # is decided here (direction, accel, notch target); how fast the ramp
        # may actually pass through the junction is a separate question that
        # Move.max_junction_v2 answers.
        if not getattr(self, "unified_span_ramps", False):
            return False
        min_cos = getattr(self, "unified_span_min_cos", None)
        if min_cos is None:
            min_cos = math.cos(math.radians(SPAN_MAX_ANGLE))
        if not self._uses_unified_reach(move):
            return False
        if not self._uses_unified_reach(prev_move):
            return False
        # Deliberately NOT comparing move.accel for equality. Kinematics like
        # limited_cartesian set the accel limit per move as
        # min(x_max_a/|rx|, y_max_a/|ry|), so it changes at EVERY segment of a
        # curve -- an exact comparison rejected every link and silently
        # disabled spanning on the real machine while the unit tests, which
        # used a constant accel, saw none of it. The run is emitted at the
        # group's minimum accel instead (see _span_plans), which respects
        # every member's limit.
        # No notch-target comparison is needed between two moves of a run.
        # Every move that reaches here carries the SAME (f_lo, f_hi): the
        # two-zero ramp nulls both modes on every axis, so the pair stopped
        # depending on heading. _move_notch_pair varies only for a move with no
        # XY component, and there it is either (0, 0) -- decoupled Z, rejected
        # by _uses_unified_reach above before we get this far -- or the very
        # same pair an XY move gets. The tolerance this used to apply was sized
        # for the single-zero blend, whose direction-weighted mean crept at
        # every segment of a curve; that drift no longer exists.
        # junction_cos_theta is the NEGATED dot product of the unit directions.
        return -junction_cos_theta >= min_cos

    def _span_reach_v2(self, move, start_v2, dist):
        # Max v^2 reachable from start_v2 over `dist` of path under this move's
        # ramp law. `dist` is a parameter rather than move.move_d because one
        # ramp may span a run of moves (see SPAN_MAX_ANGLE).
        if not self._uses_unified_reach(move):
            return start_v2 + 2.0 * dist * move.accel
        cons = self._pathplan_cons(move)
        # Memoize the last solve for this move. LookAheadQueue.flush probes the
        # same move up to three times per pass (once in the reverse sweep, twice
        # more in the unified cruise clamp) and repeats next_end_v2, so a
        # one-entry cache removes most of the calls. cons.v_ceil is part of the
        # key because it changes once set_junction() has run on this move.
        key = (start_v2, cons.v_ceil, dist)
        cached = getattr(move, "_unified_reach_cache", None)
        if cached is not None and cached[0] == key:
            return cached[1]
        # cons.v_ceil deliberately unused here: it is the EMITTER's per-move
        # ceiling (this move's own cruise target). Reachability asks how fast
        # the toolhead could arrive, so it is bounded by the machine limit;
        # move.max_cruise_v2 is applied separately by the caller.
        res = pathplan.notch_reach_v2(
            start_v2,
            dist,
            move.accel,
            self.max_velocity,
            notch_freq=cons.notch_freq,
            notch_freq2=cons.notch_freq2,
        )
        move._unified_reach_cache = (key, res)
        return res

    def _move_reach_v2(self, move, start_v2):
        if not self._uses_unified_reach(move):
            return start_v2 + move.delta_v2
        return self._span_reach_v2(move, start_v2, move.move_d)

    _UNIFIED_FIELDS = (
        "unified_emit",
        "unified_jerk_dt",
        "unified_notch_freq",
        "unified_notch_freq_x",
        "unified_notch_freq_y",
        "unified_span_ramps",
        "unified_span_max_angle",
    )

    def _unified_state(self):
        return tuple(getattr(self, n) for n in self._UNIFIED_FIELDS)

    def _restore_unified_state(self, state):
        for name, value in zip(self._UNIFIED_FIELDS, state):
            setattr(self, name, value)
        self._sync_span_cos()

    def _sync_span_cos(self):
        # unified_span_min_cos is DERIVED from unified_span_max_angle. Keep it
        # out of _UNIFIED_FIELDS (which is saved/restored against orig_cfg) and
        # recompute it whenever the angle moves, so the two can never disagree.
        self.unified_span_min_cos = math.cos(
            math.radians(self.unified_span_max_angle)
        )

    def _check_unified_extra_axis_support(self, error_factory=None):
        if not self.unified_emit:
            return
        for ea in self.extra_axes:
            if hasattr(ea, "process_move_segment"):
                continue
            axis = ea.get_axis_gcode_id()
            msg = (
                "unified_planner requires extra axis '%s' to implement"
                " process_move_segment" % (axis,)
            )
            if error_factory is not None:
                raise error_factory(msg)
            raise self.printer.command_error(msg)

    def _check_unified_settings(self, error_factory):
        if not self.unified_emit:
            return
        if not self._notch_on():
            raise error_factory(
                "unified_planner requires unified_notch_freq or both"
                " unified_notch_freq_x and unified_notch_freq_y"
            )
        max_freq = max(self.unified_notch_freq_x, self.unified_notch_freq_y)
        if self.unified_jerk_dt * max_freq > 0.1 + 1e-12:
            raise error_factory(
                "unified_jerk_dt must be at most %.9f for a %.3f Hz notch"
                " (at least 10 slices per fastest ramp edge)"
                % (0.1 / max_freq, max_freq)
            )
        self._check_unified_slice_budget(error_factory)

    def _check_unified_slice_budget(self, error_factory):
        # jerk_dt above is a LOWER bound on slices per ramp -- enough of them to
        # resolve the notch edge. This is the upper bound, and it exists because
        # over-resolving is not a recoverable condition at emit time: pathplan
        # reports it by raising InfeasibleProfile, nothing catches that, and the
        # print dies on the G1 that reached it. A low max_accel is what gets
        # there: nothing shortens a saturated plateau of dv/a_max.
        #
        # The worst ramp these settings allow is 0 -> max_velocity: an
        # unsaturated ramp lasts notch_period whatever dv is, but past
        # saturation this is the longest plateau. max_accel is the configured
        # one; M204 can lower it mid-print and lengthen that plateau further,
        # which is exactly why this bound is a config sanity check and pathplan
        # keeps a far looser runtime backstop rather than making it
        # load-bearing.
        freqs = [
            f
            for f in (self.unified_notch_freq_x, self.unified_notch_freq_y)
            if f
        ]
        cons = pathplan.Constraints(
            a_const=self.max_accel,
            v_ceil=self.max_velocity,
            jerk_dt=self.unified_jerk_dt,
            notch_freq=min(freqs),
            notch_freq2=max(freqs),
        )
        if pathplan.ramp_fits_slice_budget(0.0, self.max_velocity, cons):
            return
        raise error_factory(
            "unified planner settings need more than %d slices to ramp"
            " 0 -> %.0f mm/s (unified_jerk_dt=%.6f notch=%.2f/%.2f Hz at"
            " max_accel=%.0f). Raise unified_jerk_dt, or raise max_accel so"
            " the ramp needs less plateau."
            % (
                pathplan.MAX_RAMP_SLICES,
                self.max_velocity,
                self.unified_jerk_dt,
                min(freqs),
                max(freqs),
                self.max_accel,
            )
        )

    def _reset_unified_warnings(self):
        self._unified_warned = set()
        self._unified_all_warned = False

    def _warn_unified_profile(self, move, reasons):
        if not reasons:
            return
        if not hasattr(self, "_unified_warned"):
            self._unified_warned = set()
        for reason in reasons:
            if reason in self._unified_warned:
                continue
            self._unified_warned.add(reason)
            logging.warning(
                "unified_planner: target notch not preserved on move ending"
                " at %.3f %.3f %.3f: %s",
                move.end_pos[0],
                move.end_pos[1],
                move.end_pos[2],
                reason,
            )
        # Once every possible reason has been reported there is nothing left to
        # learn, so the caller can stop paying for the diagnostic entirely.
        if self._unified_warned >= pathplan.LOSS_REASONS:
            self._unified_all_warned = True

    def cmd_G4(self, gcmd):
        # Dwell
        delay = gcmd.get_float("P", 0.0, minval=0.0) / 1000.0
        self.dwell(delay)

    def cmd_M400(self, gcmd):
        # Wait for current moves to finish
        self.wait_moves()

    cmd_SET_VELOCITY_LIMIT_help = "Set printer velocity limits"

    def cmd_SET_VELOCITY_LIMIT(self, gcmd):
        max_velocity = gcmd.get_float("VELOCITY", None, above=0.0)
        max_accel = gcmd.get_float("ACCEL", None, above=0.0)
        square_corner_velocity = gcmd.get_float(
            "SQUARE_CORNER_VELOCITY", None, minval=0.0
        )
        min_cruise_ratio = gcmd.get_float(
            "MINIMUM_CRUISE_RATIO", None, minval=0.0, below=1.0
        )
        # Both limits size the worst ramp the unified emitter has to integrate
        # -- max_velocity is the dv it validates against, and a lower max_accel
        # lengthens a saturated plateau. Check them here, where the command can
        # still be refused and rolled back, rather than at the G1 that would
        # have hit it. M204 sets max_accel too and is deliberately NOT checked:
        # slicers emit it per feature, so failing there would be the mid-print
        # shutdown this is avoiding. pathplan's runtime backstop covers it.
        old_limits = (self.max_velocity, self.max_accel)
        if max_velocity is not None:
            self.max_velocity = max_velocity
        if max_accel is not None:
            self.max_accel = max_accel
        if max_velocity is not None or max_accel is not None:
            try:
                self._check_unified_settings(gcmd.error)
            except:
                self.max_velocity, self.max_accel = old_limits
                raise
        if square_corner_velocity is not None:
            self.square_corner_velocity = square_corner_velocity
        if min_cruise_ratio is not None:
            self.min_cruise_ratio = min_cruise_ratio
        msg = [
            "max_velocity: %.6f" % self.max_velocity,
            "max_accel: %.6f" % self.max_accel,
        ]
        if hasattr(self.kin, "max_x_velocity"):
            max_x_velocity = gcmd.get_float("X_VELOCITY", None)
            if max_x_velocity is not None:
                self.kin.max_x_velocity = max_x_velocity
            msg.append("max_x_velocity: %.6f" % self.kin.max_x_velocity)

        if hasattr(self.kin, "max_x_accel"):
            max_x_accel = gcmd.get_float("X_ACCEL", None)
            if max_x_accel is not None:
                self.kin.max_x_accel = max_x_accel
            msg.append("max_x_accel: %.6f" % self.kin.max_x_accel)

        if hasattr(self.kin, "max_y_velocity"):
            max_y_velocity = gcmd.get_float("Y_VELOCITY", None)
            if max_y_velocity is not None:
                self.kin.max_y_velocity = max_y_velocity
            msg.append("max_y_velocity: %.6f" % self.kin.max_y_velocity)

        if hasattr(self.kin, "max_y_accel"):
            max_y_accel = gcmd.get_float("Y_ACCEL", None)
            if max_y_accel is not None:
                self.kin.max_y_accel = max_y_accel
            msg.append(
                "max_y_accel: %.6f" % self.kin.max_y_accel,
            )

        if hasattr(self.kin, "max_z_velocity"):
            max_z_velocity = gcmd.get_float("Z_VELOCITY", None, above=0.0)
            if max_z_velocity is not None:
                self.kin.max_z_velocity = max_z_velocity
            msg.append("max_z_velocity: %.6f" % self.kin.max_z_velocity)

        if hasattr(self.kin, "max_z_accel"):
            max_z_accel = gcmd.get_float("Z_ACCEL", None, above=0.0)
            if max_z_accel is not None:
                self.kin.max_z_accel = max_z_accel
            msg.append("max_z_accel: %.6f" % self.kin.max_z_accel)

        self._calc_junction_deviation()
        msg.extend(
            (
                "minimum_cruise_ratio: %.6f" % self.min_cruise_ratio,
                "square_corner_velocity: %.6f" % self.square_corner_velocity,
            )
        )

        if get_danger_options().log_velocity_limit_changes:
            self.printer.set_rollover_info(
                "toolhead", "toolhead: %s" % (" ".join(msg),)
            )
            if (
                max_velocity is None
                and max_accel is None
                and square_corner_velocity is None
                and min_cruise_ratio is None
            ):
                gcmd.respond_info("\n".join(msg), log=False)

    cmd_RESET_VELOCITY_LIMIT_help = "Reset printer velocity limits"

    def cmd_RESET_VELOCITY_LIMIT(self, gcmd):
        self.max_velocity = self.orig_cfg["max_velocity"]
        self.max_accel = self.orig_cfg["max_accel"]
        msg = [
            "max_velocity: %.6f" % self.max_velocity,
            "max_accel: %.6f" % self.max_accel,
        ]

        if hasattr(self.kin, "max_x_velocity"):
            self.kin.max_x_velocity = self.orig_cfg["max_x_velocity"]
            msg.append("max_x_velocity: %.6f" % self.kin.max_x_velocity)

        if hasattr(self.kin, "max_x_accel"):
            self.kin.max_x_accel = self.orig_cfg["max_x_accel"]
            msg.append("max_x_accel: %.6f" % self.kin.max_x_accel)

        if hasattr(self.kin, "max_y_velocity"):
            self.kin.max_y_velocity = self.orig_cfg["max_y_velocity"]
            msg.append("max_y_velocity: %.6f" % self.kin.max_y_velocity)

        if hasattr(self.kin, "max_y_accel"):
            self.kin.max_y_accel = self.orig_cfg["max_y_accel"]
            msg.append(
                "max_y_accel: %.6f" % self.kin.max_y_accel,
            )

        if hasattr(self.kin, "max_z_velocity"):
            self.kin.max_z_velocity = self.orig_cfg["max_z_velocity"]
            msg.append("max_z_velocity: %.6f" % self.kin.max_z_velocity)

        if hasattr(self.kin, "max_z_accel"):
            self.kin.max_z_accel = self.orig_cfg["max_z_accel"]
            msg.append("max_z_accel: %.6f" % self.kin.max_z_accel)

        self.square_corner_velocity = self.orig_cfg["square_corner_velocity"]
        self.min_cruise_ratio = self.orig_cfg["min_cruise_ratio"]
        # Restore under rollback: extra axes can be added after startup, so the
        # support check can fail here even though the original config was valid.
        # Leaving half-restored unified state behind would be worse than the
        # error itself.
        old_unified = self._unified_state()
        # Go through _restore_unified_state rather than setattr-ing the fields
        # here: it is the one place that re-derives unified_span_min_cos from
        # the restored angle. A second, hand-rolled restore path is exactly how
        # the derived value goes stale -- reporting 2 degrees while planning
        # kept accepting 18.
        self._restore_unified_state(
            tuple(self.orig_cfg[n] for n in self._UNIFIED_FIELDS)
        )
        try:
            self._check_unified_settings(self.printer.command_error)
            self._check_unified_extra_axis_support()
        except:
            self._restore_unified_state(old_unified)
            raise
        if self._unified_state() != old_unified:
            self._reset_unified_warnings()
        self._calc_junction_deviation()
        msg.extend(
            (
                "minimum_cruise_ratio: %.6f" % self.min_cruise_ratio,
                "square_corner_velocity: %.6f" % self.square_corner_velocity,
                "unified_planner: %d" % self.unified_emit,
                "unified_jerk_dt: %.6f" % self.unified_jerk_dt,
                "unified_notch_freq: %.6f" % self.unified_notch_freq,
                "unified_notch_freq_x: %.6f" % self.unified_notch_freq_x,
                "unified_notch_freq_y: %.6f" % self.unified_notch_freq_y,
                "unified_span_ramps: %d" % self.unified_span_ramps,
                "unified_span_max_angle: %.6f" % self.unified_span_max_angle,
            )
        )
        if get_danger_options().log_velocity_limit_changes:
            gcmd.respond_info("\n".join(msg), log=False)

    def cmd_M204(self, gcmd):
        # Use S for accel
        accel = gcmd.get_float("S", None, above=0.0)
        if accel is None:
            # Use minimum of P and T for accel
            p = gcmd.get_float("P", None, above=0.0)
            t = gcmd.get_float("T", None, above=0.0)
            if p is None or t is None:
                gcmd.respond_info(
                    'Invalid M204 command "%s"' % (gcmd.get_commandline(),)
                )
                return
            accel = min(p, t)
        self.max_accel = accel
        self._calc_junction_deviation()

    def set_accel(self, accel):
        self.max_accel = accel
        self._calc_junction_deviation()

    def reset_accel(self):
        self.max_accel = self.orig_cfg["max_accel"]
        self._calc_junction_deviation()

    cmd_SET_UNIFIED_help = (
        "Toggle jerk-limited motion live. ENABLE=0/1 flips the jerk emitter; "
        "NOTCH_FREQ parks the jerk ramp's shaper zero on a mode frequency "
        "(Hz, 0 = disabled); "
        "NOTCH_FREQ_X / NOTCH_FREQ_Y set per-axis notch modes, nulled together "
        "in one trapezoidal ramp (0 = fall back to NOTCH_FREQ); "
        "SPAN_RAMPS=0/1 lets one ramp span a run of "
        "near-collinear moves so short segments get a runway without moving "
        "the zero; SPAN_MAX_ANGLE caps the heading change one ramp may span "
        "(degrees, larger trades ringing on the turning axis for spanning "
        "coarser geometry). No args = report state."
    )

    def cmd_SET_UNIFIED(self, gcmd):
        en = gcmd.get_int("ENABLE", None, minval=0, maxval=1)
        notch = gcmd.get_float("NOTCH_FREQ", None, minval=0.0)
        notch_x = gcmd.get_float("NOTCH_FREQ_X", None, minval=0.0)
        notch_y = gcmd.get_float("NOTCH_FREQ_Y", None, minval=0.0)
        span = gcmd.get_int("SPAN_RAMPS", None, minval=0, maxval=1)
        span_ang = gcmd.get_float(
            "SPAN_MAX_ANGLE", None, minval=0.0, maxval=90.0
        )
        # Flush pending moves so the change only affects moves planned after
        # this point (same live-mutation contract as SET_VELOCITY_LIMIT).
        self.flush_step_generation()
        old = self._unified_state()
        try:
            if en is not None:
                self.unified_emit = bool(en)
            for name, f in (
                ("NOTCH_FREQ", notch),
                ("NOTCH_FREQ_X", notch_x),
                ("NOTCH_FREQ_Y", notch_y),
            ):
                self._check_notch_freq(f, name, gcmd.error)
            if notch is not None:
                # NOTCH_FREQ notches both axes; explicit NOTCH_FREQ_X/Y below
                # win.
                self.unified_notch_freq = notch
                self.unified_notch_freq_x = notch
                self.unified_notch_freq_y = notch
            if notch_x is not None:
                self.unified_notch_freq_x = notch_x
            if notch_y is not None:
                self.unified_notch_freq_y = notch_y
            if span is not None:
                self.unified_span_ramps = bool(span)
            if span_ang is not None:
                self.unified_span_max_angle = span_ang
            self._sync_span_cos()
            # Enforce the same X/Y pair rule the config parser applies at
            # startup (_resolve_notch): notching one axis but not the other is
            # ambiguous, and setting only NOTCH_FREQ_X used to slip past it.
            if bool(self.unified_notch_freq_x) != bool(
                self.unified_notch_freq_y
            ):
                raise gcmd.error(
                    "NOTCH_FREQ_X and NOTCH_FREQ_Y must both be set or both be"
                    " 0. Use NOTCH_FREQ to notch both axes at one frequency."
                )
            self._check_unified_settings(gcmd.error)
            self._check_unified_extra_axis_support()
        except:
            self._restore_unified_state(old)
            raise
        if self._unified_state() != old:
            self._reset_unified_warnings()
        gcmd.respond_info(
            "unified_planner=%d unified_notch_freq=%.2f"
            " notch_freq_x=%.2f notch_freq_y=%.2f"
            " span_ramps=%d span_max_angle=%.2f"
            % (
                self.unified_emit,
                self.unified_notch_freq,
                self.unified_notch_freq_x,
                self.unified_notch_freq_y,
                self.unified_span_ramps,
                self.unified_span_max_angle,
            )
        )


def add_printer_objects(config):
    config.get_printer().add_object("toolhead", ToolHead(config))
    kinematics_extruder.add_printer_objects(config)
