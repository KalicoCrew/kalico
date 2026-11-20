# Jerk-limited motion planning with a per-ramp notch law
#
# A single phase-plane emitter that renders each move as exact constant-jerk
# (cubic-in-time) trapq segments instead of one hard accel step, plus a
# jerk-aware lookahead that plans the boundary speeds that emitter renders.
#
# Jerk is derived per ramp from a NOTCH FREQUENCY. A jerk-limited ramp whose
# acceleration does not saturate is
# triangular in a(t), which is an input shaper with a zero at 1/T_rise; the
# notch law pins T_rise = 1/f_n via J = dv * f_n^2. If the requested speed
# change would exceed max_accel, the saturated law sets J = max_accel * f_n and
# deliberately inserts a max-accel plateau; the ramp edge still has duration
# 1/f_n, preserving the ideal zero while honoring the accel limit. a(t) is
# piecewise linear, so the emitter sends it as at most three constant-jerk
# segments per ramp and the zeros are exact by construction.
#
# THROUGHPUT CONSEQUENCE. Parking the zero fixes the ramp DURATION at 2/f_n
# regardless of how small the speed change is, so a ramp always consumes
# (v0+v1)/f_n of path distance. The distance is therefore discontinuous at
# dv -> 0: it does not fall to zero but to 2*v0/f_n. A move shorter than that
# cannot change speed at all, and since each move ramps back to a = 0 at its own
# end, acceleration cannot be spread across a run of short segments. On a chain
# of equal-length segments the toolhead converges to roughly
#
#     v_terminal ~= f_n * segment_length
#
# (55 Hz, 1 mm segments -> ~55 mm/s) no matter how high max_velocity/max_accel
# are. This is inherent to per-move notched ramps, not a tuning problem. See
# docs/Jerk_Limiting.md "Throughput" for the measured table.
#
# Pure (only `math`) and toolhead-decoupled so it is unit-testable in isolation;
# the toolhead adapter supplies per-move constraints.
#
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
import math


class InfeasibleProfile(Exception):
    pass


class Constraints:
    """Per-move motion limits for the jerk-limited emitter and lookahead.

    a_const    -> constant max |acceleration| (mm/s^2). With the notch law,
                  unsaturated moves are governed by a_peak = dv * f_n; larger
                  moves use a designed max-accel plateau.
    v_ceil     -> hard speed ceiling (mm/s); the reachability search stops here.
    notch_freq -> mode frequency (Hz) to park the ramp's shaper zero on.
                  None/0 = no shaped motion. See ramp_jerk().
    """

    def __init__(self, a_const, v_ceil=1e9, notch_freq=None):
        self.a_const = a_const
        self.v_ceil = v_ceil
        self.notch_freq = notch_freq

    def a_max(self, v):
        return self.a_const

    def ramp_jerk(self, dv):
        """Jerk (mm/s^3) for ONE ramp of size |dv| (mm/s).

        With no notch frequency configured there is no shaped ramp.

        Notch mode: a jerk-limited ramp whose acceleration never saturates is
        TRIANGULAR in a(t) -- a_peak = sqrt(J*dv), rise time T = sqrt(dv/J) --
        and a triangular accel pulse is a shaper with a zero at f = 1/T. With
        a FIXED J that zero sits at sqrt(J/dv), i.e. it MOVES with every move's
        dv, which is useless for cancelling a mode at a FIXED frequency: short
        moves get a high-frequency notch and long ones a low-frequency notch.
        (Saturation needs dv >= a_max^2/J, which on a machine with a modest
        max_velocity never happens, so the triangular case is the normal case.)

        Solving 1/T = f_n for J instead gives the per-ramp law

            J = dv * f_n^2   ->   T        = 1/f_n        (constant: zero parked)
                                  a_peak   = dv * f_n     (linear in dv)
                                  distance = (v0+v1)/f_n

        so acceleration self-scales with the size of the velocity change while
        the shaper zero stays on the mode.

        If that would exceed max_accel, use a DESIGNED saturated trapezoid:

            J = a_max * f_n  ->  T_rise = a_max/J = 1/f_n
                                  plateau = dv/a_max - 1/f_n

        The accel pulse is then a ramp-edge smoothing kernel convolved with a
        rectangle; the ramp-edge zero remains parked at f_n while the plateau
        supplies the extra velocity change at the configured acceleration.
        """
        dv = abs(dv)
        if not self.notch_freq or dv <= 1e-12:
            return None
        a = self.a_const
        f = self.notch_freq
        if a and a > 0.0 and dv > a / f:
            j = a * f
        else:
            j = dv * f * f
        return j if j > 0.0 else None


def notch_dist(v0, v1, accel, notch_freq):
    # Path distance to change speed v0 -> v1 under a symmetric jerk-limited
    # notch profile (accel ramps 0 -> peak -> 0, so a(t) is continuous in the
    # ideal law). Closed form: distance = mean speed * duration, exact because
    # a symmetric velocity S-curve's time-average speed is (v0+v1)/2. Serves
    # accel and decel identically (uses |dv|). The cubic emitter renders
    # exactly this ramp, so the planning model IS the rendered profile.
    dv = abs(v1 - v0)
    if dv <= 1e-12:
        return 0.0
    if not notch_freq:
        raise ValueError("notch_dist requires a notch frequency")
    # Per-ramp law, mirrored from Constraints.ramp_jerk: the ramp lasts 2/f_n
    # until a_peak = dv*f_n would exceed accel; past that the plateau widens to
    # dv/A and the two edges keep their 1/f_n rise.
    if accel > 0.0 and dv > accel / notch_freq:
        t = dv / accel + 1.0 / notch_freq
    else:
        t = 2.0 / notch_freq
    return 0.5 * (v0 + v1) * t


def _fit_down(v1, fits, floor):
    # Closed-form solutions land on the feasibility boundary itself, where the
    # recomputed distance can exceed the budget by an ULP. Step down until the
    # distance check agrees, so callers always receive a feasible speed.
    for _ in range(8):
        if v1 <= floor or fits(v1):
            return v1
        v1 = math.nextafter(v1, -math.inf)
    return max(floor, v1 * (1.0 - 1e-12))


def notch_reach_v2(u0, dist, accel, v_ceil, notch_freq):
    # Max u = v^2 reachable from v0=sqrt(u0) over path-distance `dist` under a
    # notch profile. Direction-symmetric (forward accel == backward decel).
    #
    # Closed form while the ramp is unsaturated (a_peak = dv*f_n <= accel):
    # the ramp then lasts exactly 2/f_n, so its distance is (v0+v1)/f_n and the
    # reach is v1 = dist*f_n - v0. Past saturation the bisection on the
    # closed-form distance is kept.
    if dist <= 0.0:
        return u0
    if accel <= 0.0:
        return u0
    if not notch_freq:
        raise ValueError("notch_reach_v2 requires a notch frequency")
    v0 = math.sqrt(max(u0, 0.0))
    f = notch_freq
    # A ramp costs (v0+v1)/f_n of distance, and that does not go to zero as
    # v1 -> v0: its infimum is 2*v0/f_n. A move shorter than that has no room
    # for any speed change and reach is exactly u0.
    if dist < 2.0 * v0 / f:
        return u0

    def _fits(v1):
        return notch_dist(v0, v1, accel, f) <= dist

    hi = max(v0, v_ceil)
    v1 = dist * f - v0
    if v1 - v0 <= accel / f:
        # Unsaturated over the whole answer (and every speed below it), so the
        # distance is linear in v1 here and the boundary is exact.
        if v1 >= hi:
            return hi * hi
        v1 = _fit_down(max(v1, v0), _fits, v0)
        return v1 * v1
    return _notch_reach_v2_bisect(v0, hi, _fits)


def _notch_reach_v2_bisect(v0, hi, _fits):
    # Saturated-ramp fallback: bisection on the closed-form notch_dist.
    if _fits(hi):
        return hi * hi
    lo = v0
    # Converge to a velocity tolerance instead of burning a fixed iteration
    # count: this runs on every move in the lookahead hot loop, and 1e-4 mm/s
    # is far below anything the step generator can resolve.
    for _ in range(48):
        if hi - lo <= 1e-4:
            break
        mid = 0.5 * (lo + hi)
        if _fits(mid):
            lo = mid
        else:
            hi = mid
    return lo * lo


def _emit_sharp(vs, vc, ve, move_d, cons):
    # Constant-accel accel/cruise/decel profile (the classic trapezoid), used
    # when notch shaping is disabled. Closed form at cons.a_const.
    a = cons.a_const
    if a is None or a <= 0.0:
        return []
    vc = max(vc, vs, ve)
    d_acc = (vc * vc - vs * vs) / (2.0 * a)
    d_dec = (vc * vc - ve * ve) / (2.0 * a)
    cruise_d = move_d - d_acc - d_dec
    if cruise_d < -1e-9:
        # Triangle: solve the peak where accel vs->vp and decel vp->ve fill
        # move_d exactly: 2*vp^2 - vs^2 - ve^2 = 2*a*move_d.
        vp2 = 0.5 * (2.0 * a * move_d + vs * vs + ve * ve)
        vc = math.sqrt(max(vp2, 0.0))
        if vc < max(vs, ve) - 1e-9:
            return []
        d_acc = (vc * vc - vs * vs) / (2.0 * a)
        d_dec = (vc * vc - ve * ve) / (2.0 * a)
        cruise_d = max(0.0, move_d - d_acc - d_dec)
    segs = []
    if vc > vs + 1e-12:
        segs.append(((vc - vs) / a, 0.0, 0.0, vs, vc, a, d_acc))
    if cruise_d > 1e-9 and vc > 1e-9:
        segs.append((0.0, cruise_d / vc, 0.0, vc, vc, 0.0, cruise_d))
    if vc > ve + 1e-12:
        segs.append((0.0, 0.0, (vc - ve) / a, vc, vc, a, d_dec))
    return segs


# ---------------------------------------------------------------------------
# Cubic (constant-jerk) emitter
#
# The notch ramp is piecewise LINEAR in a(t): rise at +J, hold at a_peak, fall
# at -J. Each piece is one trapq_append_jerk segment whose position is cubic in
# time. A ramp is at most three segments, computed in closed form -- no
# integration loop -- and the zeros it parks are exact by construction.
#
# Segment tuple: (move_t, start_v, start_a, jerk, dist).
#
# The ramp has exactly the planning model's duration and distance
# (notch_dist), so the lookahead's analytic model is now the rendered profile
# rather than an approximation of it.
# ---------------------------------------------------------------------------


def _cubic_ramp_shape(v0, v1, cons):
    # (a_peak, J, t_rise, t_hold) for the ramp |v1 - v0|, or None.
    dv = abs(v1 - v0)
    if dv <= 1e-12 or not cons.notch_freq:
        return None
    a_ramp = cons.a_const
    J = cons.ramp_jerk(dv)
    if J is None or J <= 0.0:
        return None
    # Unsaturated: a_peak = dv*f_n, the triangle's own peak. Saturated:
    # a_peak = A.
    a_peak = dv * cons.notch_freq
    if a_ramp and 0.0 < a_ramp < a_peak:
        a_peak = a_ramp
    t_rise = a_peak / J
    t_hold = max(0.0, dv / a_peak - t_rise)
    return a_peak, J, t_rise, t_hold


def cubic_ramp_dist(v0, v1, cons):
    """Path distance of the exact ramp v0 -> v1 (closed form)."""
    shape = _cubic_ramp_shape(v0, v1, cons)
    if shape is None:
        return 0.0
    _a, _J, t_rise, t_hold = shape
    return 0.5 * (v0 + v1) * (2.0 * t_rise + t_hold)


def _cubic_ramp_up(v0, v1, cons):
    # Exact segments taking v0 -> v1 (v1 >= v0), rise / hold / fall.
    shape = _cubic_ramp_shape(v0, v1, cons)
    if shape is None:
        return []
    a_peak, J, t_rise, t_hold = shape
    dv_edge = 0.5 * a_peak * t_rise
    segs = []
    v = v0
    d = v * t_rise + J * t_rise**3 / 6.0
    segs.append((t_rise, v, 0.0, J, d))
    v = v0 + dv_edge
    if t_hold > 0.0:
        segs.append(
            (
                t_hold,
                v,
                a_peak,
                0.0,
                v * t_hold + 0.5 * a_peak * t_hold * t_hold,
            )
        )
    # Land the fall edge exactly on v1 rather than on the accumulated sum.
    v = v1 - dv_edge
    segs.append(
        (
            t_rise,
            v,
            a_peak,
            -J,
            (v + 0.5 * a_peak * t_rise - J * t_rise * t_rise / 6.0) * t_rise,
        )
    )
    return segs


def _cubic_reverse(segs):
    # Time-reverse an accelerating ramp into the matching decel. A segment
    # v(s) = sv + a0*s + j*s^2/2 played backwards has start speed v(T),
    # start accel -(a0 + j*T) and the SAME jerk. Magnitude spectrum, and so
    # the parked zeros, are preserved under time reversal.
    out = []
    for t, sv, a0, j, dist in reversed(segs):
        out.append((t, sv + a0 * t + 0.5 * j * t * t, -(a0 + j * t), j, dist))
    return out


def _cubic_peak_velocity(vs, ve, move_d, cons):
    # Highest cruise the move can reach (accel vs->vp plus decel vp->ve).
    #
    # Closed form while both ramps are unsaturated: each then costs
    # (v_end + vp)/f_n, so vp = (move_d*f_n - vs - ve)/2. A ramp of zero
    # size costs nothing (not 2*v/f_n), so vp == max(vs, ve) is feasible on
    # its own and is the answer whenever the formula lands at or below it.
    lo = max(vs, ve)
    hi = cons.v_ceil
    if cubic_ramp_dist(vs, lo, cons) + cubic_ramp_dist(ve, lo, cons) >= move_d:
        return lo
    if cons.notch_freq:
        vp = 0.5 * (move_d * cons.notch_freq - vs - ve)
        a = cons.a_const
        unsat = not (a and a > 0.0) or (vp - min(vs, ve)) * cons.notch_freq <= a
        if unsat:
            if vp <= lo:
                return lo
            if vp >= hi:
                vp = hi

            def _fits(v):
                return (
                    cubic_ramp_dist(vs, v, cons) + cubic_ramp_dist(ve, v, cons)
                ) <= move_d

            return _fit_down(vp, _fits, lo)
    return _cubic_peak_velocity_bisect(vs, ve, move_d, cons, lo, hi)


def _cubic_peak_velocity_bisect(vs, ve, move_d, cons, lo, hi):
    # Saturated-ramp fallback: bisection on the closed-form ramp distance.
    for _ in range(48):
        if hi - lo <= 1e-4:
            break
        mid = 0.5 * (lo + hi)
        need = cubic_ramp_dist(vs, mid, cons) + cubic_ramp_dist(ve, mid, cons)
        if need > move_d:
            hi = mid
        else:
            lo = mid
    return lo


def _emit_cubic_core(vs, vc, ve, move_d, cons, collect=True):
    # One notched profile: ramp vs->vc, cruise, ramp vc->ve. Returns None when
    # the endpoints are unreachable within move_d even at the connecting peak.
    vc = max(vc, vs, ve)
    d_acc = cubic_ramp_dist(vs, vc, cons)
    d_dec = cubic_ramp_dist(ve, vc, cons)
    cruise_d = move_d - d_acc - d_dec
    if cruise_d < -1e-9:
        vc = max(_cubic_peak_velocity(vs, ve, move_d, cons), vs, ve)
        d_acc = cubic_ramp_dist(vs, vc, cons)
        d_dec = cubic_ramp_dist(ve, vc, cons)
        cruise_d = move_d - d_acc - d_dec
        if cruise_d < -1e-6 * max(1.0, move_d):
            return None
        cruise_d = max(0.0, cruise_d)
    if cruise_d > 1e-9 and vc <= 1e-9:
        return None
    if not collect:
        return []
    segs = _cubic_ramp_up(vs, vc, cons)
    if cruise_d > 1e-9:
        segs.append((cruise_d / vc, vc, 0.0, 0.0, cruise_d))
    segs.extend(_cubic_reverse(_cubic_ramp_up(ve, vc, cons)))
    if not segs and move_d > 1e-9:
        return None
    return segs


def _infeasible(vs, vc, ve, move_d):
    return InfeasibleProfile(
        "notched cubic profile does not fit move: "
        "start=%.6f cruise=%.6f end=%.6f distance=%.6f" % (vs, vc, ve, move_d)
    )


def validate_cubic_profile(vs, vc, ve, move_d, cons):
    """Raise InfeasibleProfile if the exact cubic profile cannot fit."""
    if move_d <= 0.0 or not cons.notch_freq:
        return
    if _emit_cubic_core(vs, vc, ve, move_d, cons, collect=False) is None:
        raise _infeasible(vs, vc, ve, move_d)


def plan_cubic_profile(vs, vc, ve, move_d, cons):
    """Validate and render in ONE pass: the segments, or InfeasibleProfile.

    The toolhead's hot path. Rendering a cubic profile is closed form, so
    running the feasibility recurrence once to validate and again to render
    only doubled the work; the render already knows whether it fits.
    """
    if move_d > 0.0 and cons.notch_freq:
        segs = _emit_cubic_core(vs, vc, ve, move_d, cons)
        if segs is None:
            raise _infeasible(vs, vc, ve, move_d)
        return segs
    return render_cubic_profile(vs, vc, ve, move_d, cons)


def render_cubic_profile(vs, vc, ve, move_d, cons):
    """Render a validated profile as exact (move_t, sv, a0, jerk, dist)
    segments. Without a notch frequency it is the classic trapezoid, which
    is just the jerk == 0 case of the same tuple."""
    if move_d <= 0.0:
        return []
    if cons.notch_freq:
        segs = _emit_cubic_core(vs, vc, ve, move_d, cons)
        if segs is None:
            raise AssertionError("validated cubic profile failed to render")
        return segs
    out = []
    for at, ct, dt, sv, cv, a, dist in _emit_sharp(vs, vc, ve, move_d, cons):
        if at > 0.0:
            out.append((at, sv, a, 0.0, dist))
        elif ct > 0.0:
            out.append((ct, cv, 0.0, 0.0, dist))
        elif dt > 0.0:
            out.append((dt, cv, -a, 0.0, dist))
    return out
