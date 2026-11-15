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
# TWO MODES. A machine whose X and Y modes differ can null BOTH in the same
# ramp, because spectral zeros multiply under convolution and the triangle above
# is already rect(1/f_n) * rect(1/f_n). Widening one rect gives a TRAPEZOID,
# rect(1/f_hi) * rect(1/f_lo), with exact zeros at f_lo AND f_hi -- and since
# every axis sees the same a(t) scaled by the move's direction, every axis gets
# both. Set notch_freq2 to ask for it; see Constraints.ramp_jerk. f_lo == f_hi
# collapses every formula below back to the single-zero triangle, so there is no
# separate code path.
#
# THROUGHPUT CONSEQUENCE. Parking the zeros fixes the ramp DURATION at
# 1/f_lo + 1/f_hi (that is 2/f_n with one zero) regardless of how small the speed
# change is, so a ramp always consumes (v0+v1)/f_eq of path distance, where f_eq
# is the pair's harmonic mean. The distance is therefore discontinuous at
# dv -> 0: it does not fall to zero but to 2*v0/f_eq. A move shorter than that
# cannot change speed at all, and since each move ramps back to a = 0 at its own
# end, acceleration cannot be spread across a run of short segments. On a chain
# of equal-length segments the toolhead converges to roughly
#
#     v_terminal ~= f_eq * segment_length
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
                  None/0 = no shaped motion.
                  See ramp_jerk() for the ideal law, and notch_loss_reasons()
                  for cases where the emitted profile no longer preserves it.
    notch_freq2
               -> SECOND mode frequency (Hz) to null in the same ramp. None/0,
                  or equal to notch_freq, gives the single-zero triangle. See
                  ramp_jerk() for the two-zero law.
    """

    def __init__(
        self,
        a_const,
        v_ceil=1e9,
        notch_freq=None,
        notch_freq2=None,
    ):
        self.a_const = a_const
        self.v_ceil = v_ceil
        self.notch_freq = notch_freq
        self.notch_freq2 = notch_freq2
        # Derived pair. A ramp is rect(1/f_hi) * rect(1/f_lo) in a(t), so it
        # lasts notch_period = 1/f_lo + 1/f_hi and covers
        # 0.5*(v0+v1)*notch_period of path. Writing that as (v0+v1)/notch_f_eq
        # -- f_eq the HARMONIC mean of the pair -- makes every distance and
        # runway formula in this module identical to the single-notch ones,
        # where f_lo == f_hi == f_eq == notch_freq and notch_period == 2/f_n.
        f1 = notch_freq or 0.0
        f2 = notch_freq2 or f1
        if not f1:
            self.notch_lo = self.notch_hi = 0.0
            self.notch_period = 0.0
            self.notch_f_eq = 0.0
        else:
            self.notch_lo = min(f1, f2)
            self.notch_hi = max(f1, f2)
            self.notch_period = 1.0 / self.notch_lo + 1.0 / self.notch_hi
            self.notch_f_eq = 2.0 / self.notch_period

    def a_max(self, v):
        return self.a_const

    def ramp_accel_cap(self, dv):
        """Peak |accel| (mm/s^2) the two-zero law allows for ONE ramp of |dv|.

        The trapezoid rect(1/f_hi) * rect(1/f_lo) has area dv and base
        1/f_hi + 1/f_lo, so its plateau height is dv/max_width = dv*f_lo. That
        cap is what CREATES the plateau -- without it the integrator would keep
        rising and render a triangle, which nulls only one frequency.

        None means "no notch-imposed cap": in single-notch mode the triangle's
        peak (dv*f_n) falls out of the jerk law by itself, and capping there
        would be a no-op at best.
        """
        if not self.notch_freq or self.notch_lo >= self.notch_hi:
            return None
        dv = abs(dv)
        if dv <= 1e-12:
            return None
        return dv * self.notch_lo

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

        TWO zeros (notch_freq2 set and different). One scalar ramp shapes every
        axis at once -- a_i(t) = r_i*a(t) -- so a single zero has to serve both
        modes, and on a diagonal a direction-weighted compromise lands it on
        neither. It does not have to: spectral zeros MULTIPLY under convolution,
        and the triangle above is already rect(1/f_n) * rect(1/f_n). Widen one
        of the two rects and the pulse nulls two frequencies at once:

            a(t) = rect(1/f_hi) * rect(1/f_lo)     (a TRAPEZOID, not a triangle)
            |A(f)| = |sinc(f/f_hi) * sinc(f/f_lo)| -> exact zeros at BOTH

        Matching that shape to this integrator needs the rise edge to reach the
        plateau in 1/f_hi, i.e. a_peak/J = 1/f_hi with a_peak = dv*f_lo:

            J = dv * f_lo * f_hi   ->   T_rise  = 1/f_hi
                                        a_peak  = dv * f_lo   (ramp_accel_cap)
                                        T_total = 1/f_lo + 1/f_hi
                                        distance= 0.5*(v0+v1)*T_total

        f_lo == f_hi collapses every line of that back to the triangle, so the
        single-zero case is not a special case in the code -- it is this one.
        The trapezoid is only ~2% longer than a triangle at the two modes'
        weighted mean while leaving ~0.04% at each mode instead of 3%/1.6%, and
        its peak accel is LOWER (set by f_lo, not the mean).

        Past accel saturation the plateau is forced wider than 1/f_lo -- its
        width is dv/A -- and no choice of J recovers BOTH zeros, because the
        shape needed there is third order (an S-edge) rather than the constant
        jerk this integrator renders. The rise edge still holds one zero;
        sat_rise_freq() picks which mode to spend it on, and
        notch_loss_reasons() reports the loss.
        """
        dv = abs(dv)
        if not self.notch_freq or dv <= 1e-12:
            return None
        a = self.a_const
        f_lo, f_hi = self.notch_lo, self.notch_hi
        if a and a > 0.0 and dv > a / f_lo:
            j = a * sat_rise_freq(dv, a, f_lo, f_hi)
        else:
            j = dv * f_lo * f_hi
        return j if j > 0.0 else None


def _sinc(x):
    # sin(pi*x)/(pi*x); the normalized sinc, so the zeros sit on the integers.
    if abs(x) < 1e-12:
        return 1.0
    y = math.pi * x
    return math.sin(y) / y


def sat_rise_freq(dv, accel, f_lo, f_hi):
    """Which zero to keep when accel saturation costs us the other one.

    Below saturation the ramp is rect(1/f_hi) * rect(1/f_lo) and nulls both
    modes. Past it the plateau is no longer ours to choose -- its width is
    forced to dv/A -- so the pulse is rect(T_rise) * rect(dv/A) and only T_rise
    is free. Its zeros land on k/T_rise and k*A/dv, leaving

        residual(f) = |sinc(f*T_rise) * sinc(f*dv/A)|

    Pinning T_rise to one mode by fiat is arbitrarily bad: whether f_lo or f_hi
    is the better place to spend the one remaining zero depends on where the
    forced A/dv zero and its harmonics happen to fall, which moves with dv. So
    evaluate both candidates and keep whichever leaves the QUIETER worse mode.
    Ties go to f_lo -- the lower mode displaces more for the same acceleration
    (x ~ a/w^2), so it is the one to protect when there is nothing to choose.

    Returns f_lo/f_hi unchanged when the pair is degenerate or unsaturated,
    which makes this a no-op on single-notch machines.
    """
    return _sat_rise(dv, accel, f_lo, f_hi)[0]


def _sat_rise(dv, accel, f_lo, f_hi):
    # (chosen rise frequency, worst residual it leaves). See sat_rise_freq.
    if f_lo >= f_hi:
        return f_lo, 0.0
    t_flat = dv / accel
    best_f, best_worst = None, None
    for f_rise in (f_lo, f_hi):
        t_rise = 1.0 / f_rise
        worst = max(
            abs(_sinc(f * t_rise) * _sinc(f * t_flat)) for f in (f_lo, f_hi)
        )
        if best_worst is None or worst < best_worst:
            best_f, best_worst = f_rise, worst
    return best_f, best_worst


# How much residual at a mode counts as actually LOSING that zero, for
# reporting purposes. Below it the forced A/dv zero (or one of its harmonics) happened to land near
# enough to the second mode that nothing was really lost, and warning would be
# noise.
SECOND_NOTCH_EPS = 0.01


def notch_dist(v0, v1, accel, notch_freq, notch_freq2=None):
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
    # Per-ramp law, mirrored from Constraints.ramp_jerk. Both zeros survive
    # while the plateau is the notch's own (a_peak = dv*f_lo <= accel), and
    # then the duration is exactly 1/f_lo + 1/f_hi. Past saturation the plateau
    # widens to dv/A and the rise edge keeps one configured zero.
    f_lo = min(notch_freq, notch_freq2 or notch_freq)
    f_hi = max(notch_freq, notch_freq2 or notch_freq)
    saturated = accel > 0.0 and dv > accel / f_lo
    if saturated:
        f_rise = sat_rise_freq(dv, accel, f_lo, f_hi)
        t = dv / accel + 1.0 / f_rise
    else:
        t = 1.0 / f_lo + 1.0 / f_hi
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


def notch_reach_v2(
    u0,
    dist,
    accel,
    v_ceil,
    notch_freq,
    notch_freq2=None,
):
    # Max u = v^2 reachable from v0=sqrt(u0) over path-distance `dist` under a
    # notch profile. Direction-symmetric (forward accel == backward decel).
    #
    # Closed form while the ramp is unsaturated (a_peak = dv*f_lo <= accel):
    # the ramp then lasts exactly 1/f_lo + 1/f_hi, so its distance is
    # (v0+v1)/f_eq and the reach is v1 = dist*f_eq - v0. Past saturation the
    # rise frequency is chosen per dv (sat_rise_freq), which makes distance
    # non-monotonic in v1, so that case keeps the bisection.
    if dist <= 0.0:
        return u0
    if accel <= 0.0:
        return u0
    if not notch_freq:
        raise ValueError("notch_reach_v2 requires a notch frequency")
    v0 = math.sqrt(max(u0, 0.0))
    # Distances are governed by the pair's HARMONIC mean (Constraints
    # docstring): a ramp costs 0.5*(v0+v1)*(1/f_lo + 1/f_hi) = (v0+v1)/f_eq,
    # which is the single-notch formula with f_eq == notch_freq.
    f_lo = min(notch_freq, notch_freq2 or notch_freq)
    f_hi = max(notch_freq, notch_freq2 or notch_freq)
    f_eq = 2.0 / (1.0 / f_lo + 1.0 / f_hi)
    # A ramp costs (v0+v1)/f_eq of distance, and that does not go to zero as
    # v1 -> v0: its infimum is 2*v0/f_eq. A move shorter than that has no room
    # for any speed change and reach is exactly u0.
    if dist < 2.0 * v0 / f_eq:
        return u0

    def _fits(v1):
        return notch_dist(v0, v1, accel, f_lo, f_hi) <= dist

    hi = max(v0, v_ceil)
    v1 = dist * f_eq - v0
    if v1 - v0 <= accel / f_lo:
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
#
# Past accel saturation the two-zero trapezoid widens its plateau to dv/A and
# only ONE zero survives (see Constraints.ramp_jerk); notch_loss_reasons()
# reports that as second_notch_saturated.
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
    # Unsaturated: a_peak = dv*f_lo (the two-zero cap; dv*f_n for a single
    # notch, where it is the triangle's own peak). Saturated: a_peak = A.
    a_peak = dv * cons.notch_lo
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
    # (v_end + vp)/f_eq, so vp = (move_d*f_eq - vs - ve)/2. A ramp of zero
    # size costs nothing (not 2*v/f_eq), so vp == max(vs, ve) is feasible on
    # its own and is the answer whenever the formula lands at or below it.
    lo = max(vs, ve)
    hi = cons.v_ceil
    if cubic_ramp_dist(vs, lo, cons) + cubic_ramp_dist(ve, lo, cons) >= move_d:
        return lo
    if cons.notch_freq:
        vp = 0.5 * (move_d * cons.notch_f_eq - vs - ve)
        a = cons.a_const
        unsat = not (a and a > 0.0) or (vp - min(vs, ve)) * cons.notch_lo <= a
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


def _cubic_time_at(seg, d1):
    # Time at which segment `seg` has covered distance d1. Distance is
    # monotonic (speed never goes negative inside a ramp), so safeguarded
    # Newton on a bracket always converges.
    t, sv, a0, j, dist = seg
    lo, hi = 0.0, t
    x = t * d1 / dist if dist > 0.0 else 0.0
    for _ in range(60):
        f = ((j / 6.0 * x + 0.5 * a0) * x + sv) * x - d1
        if f > 0.0:
            hi = x
        else:
            lo = x
        v = (0.5 * j * x + a0) * x + sv
        nx = x - f / v if v > 0.0 else 0.5 * (lo + hi)
        if not (lo < nx < hi):
            nx = 0.5 * (lo + hi)
        if abs(nx - x) <= 1e-15 * max(t, 1e-9):
            x = nx
            break
        x = nx
    return min(max(x, 0.0), t)


def split_cubic_segments(segs, lengths):
    """Cut one profile into per-move buckets at cumulative `lengths`.

    A ramp that spans several moves is rendered ONCE, over the run's total
    distance, so the accel pulse keeps its shape (and therefore its zeros)
    across move boundaries. The toolhead still needs the segments bucketed per
    move, because each move carries its own direction and extra-axis ratios.
    A segment straddling a boundary is cut at the time it covers the
    remaining distance, so every bucket sums to exactly that move's length.
    """
    out = [[] for _ in lengths]
    if not lengths:
        return out
    i = 0
    remaining = lengths[0]
    for seg in segs:
        d = seg[4]
        while d > 0.0:
            if d <= remaining + 1e-12 or i >= len(out) - 1:
                out[i].append(seg)
                remaining -= d
                d = 0.0
            else:
                t, sv, a0, j, dist = seg
                t1 = _cubic_time_at(seg, remaining)
                out[i].append((t1, sv, a0, j, remaining))
                v1 = (0.5 * j * t1 + a0) * t1 + sv
                seg = (t - t1, v1, a0 + j * t1, j, dist - remaining)
                d = seg[4]
                remaining = 0.0
            if remaining <= 1e-12 and i < len(out) - 1:
                i += 1
                remaining = lengths[i]
    return out


# Every reason notch_loss_reasons() can ever return. The toolhead uses this to
# stop calling the diagnostic once it has reported all of them.
LOSS_REASONS = frozenset(("second_notch_saturated",))


def notch_loss_reasons(vs, vc, ve, cons):
    """Return reasons the requested notch frequency is not preserved exactly.

    Reports only ACTIONABLE losses -- ones a config change can remove.
    """
    if not cons.notch_freq:
        return []
    reasons = set()
    two_zero = cons.notch_hi > cons.notch_lo
    for dv in (abs(vc - vs), abs(vc - ve)):
        if dv <= 1e-12:
            continue
        saturated = (
            cons.a_const
            and cons.a_const > 0.0
            and dv > cons.a_const / cons.notch_lo
        )
        if two_zero and saturated:
            # a_peak would have to be dv*f_lo to keep the plateau exactly
            # 1/f_lo wide; max_accel is lower, so the plateau widens to dv/A
            # and only one zero is ours to place (sat_rise_freq). Report it
            # only when a mode is genuinely left excited -- the forced A/dv
            # zero sometimes covers the other mode anyway. Fixable by raising
            # max_accel, or by spanning the ramp across more moves so each dv
            # is smaller.
            _f, worst = _sat_rise(
                dv, cons.a_const, cons.notch_lo, cons.notch_hi
            )
            if worst > SECOND_NOTCH_EPS:
                reasons.add("second_notch_saturated")
    return sorted(reasons)
