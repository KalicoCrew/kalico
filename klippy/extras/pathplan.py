# Jerk-limited motion planning with a per-ramp notch law
#
# A single phase-plane emitter that renders each move as discretized
# jerk-limited acceleration slices instead of one hard accel step, plus a
# jerk-aware lookahead that approximates the boundary speeds that emitter can
# render.
#
# The jerk per ramp can either be a fixed value (max_jerk) or be derived from a
# NOTCH FREQUENCY. A jerk-limited ramp whose acceleration does not saturate is
# triangular in a(t), which is an input shaper with a zero at 1/T_rise; the
# notch law pins T_rise = 1/f_n via J = dv * f_n^2. If the requested speed
# change would exceed max_accel, the saturated law sets J = max_accel * f_n and
# deliberately inserts a max-accel plateau; the ramp edge still has duration
# 1/f_n, preserving the ideal zero while honoring the accel limit. The actual
# emitter uses zero-order-held constant-accel slices, so discretization, jerk
# clamping, insufficient runway, and the sharp fallback do not preserve an
# exact zero at the requested mode.
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


class Constraints:
    """Per-move motion limits for the jerk-limited emitter and lookahead.

    a_const    -> constant max |acceleration| (mm/s^2). With the notch law,
                  unsaturated moves are governed by a_peak = dv * f_n; larger
                  moves use a designed max-accel plateau. Short fallback moves
                  remain bounded by this value too.
    v_ceil     -> hard speed ceiling (mm/s); the reachability search stops here.
    max_jerk   -> max |da/dt| (mm/s^3). None/0 = no jerk limiting (sharp
                  constant-accel profile). With notch_freq set this is a CEILING
                  on the per-ramp jerk rather than the jerk itself.
    jerk_dt    -> integration time step (s) for the emitted ramp.
    max_da     -> optional cap on positive jerk-up acceleration steps between
                  emitted slices (mm/s^2). When fallback raises jerk, the
                  emitter shrinks slice dt so J*dt <= max_da.
    notch_freq -> mode frequency (Hz) to park the ramp's shaper zero on.
                  None/0 = fixed-jerk behaviour (max_jerk used verbatim).
                  See ramp_jerk() for the ideal law, and notch_loss_reasons()
                  for cases where the emitted profile no longer preserves it.
    notch_freq2
               -> SECOND mode frequency (Hz) to null in the same ramp. None/0,
                  or equal to notch_freq, gives the single-zero triangle. See
                  ramp_jerk() for the two-zero law.
    notch_max_freq
               -> highest frequency (Hz) the notch may be RAISED to when a move
                  is too short to fit a ramp at notch_freq. See
                  adapted_notch_freq(). None/0, or any value <= notch_freq,
                  disables the adaptation and short moves simply hold their
                  entry speed.
    """

    def __init__(
        self,
        a_const,
        v_ceil=1e9,
        max_jerk=None,
        jerk_dt=0.001,
        notch_freq=None,
        max_da=None,
        notch_max_freq=None,
        notch_freq2=None,
    ):
        self.a_const = a_const
        self.v_ceil = v_ceil
        self.max_jerk = max_jerk
        self.jerk_dt = jerk_dt
        self.notch_freq = notch_freq
        self.max_da = max_da
        self.notch_max_freq = notch_max_freq
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

        Fixed-jerk mode (notch_freq unset) returns max_jerk unchanged.

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

        max_jerk, when set in notch mode, limits the allowed ramp dv if the cap
        is lower than the jerk needed by either law. It does not lower the notch
        frequency to fit the cap.

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
            return self.max_jerk
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
# reporting purposes. The emitter's zero-order hold already leaves a few times
# 1e-4 at a parked zero, so this sits an order of magnitude above the floor:
# below it the forced A/dv zero (or one of its harmonics) happened to land near
# enough to the second mode that nothing was really lost, and warning would be
# noise.
SECOND_NOTCH_EPS = 0.01


def _pair_for(f_eq, ratio):
    """Expand an equivalent (harmonic-mean) frequency back into its pair.

    The pair (f_lo, f_hi = ratio*f_lo) with harmonic mean f_eq satisfies
    2/f_eq = 1/f_lo + 1/f_hi, so f_lo = f_eq*(1 + 1/ratio)/2. ratio == 1.0
    returns (f_eq, f_eq), i.e. the single-zero case unchanged.
    """
    if ratio <= 1.0:
        return f_eq, f_eq
    f_lo = 0.5 * f_eq * (1.0 + 1.0 / ratio)
    return f_lo, f_lo * ratio


def ramp_freq_for(v_sum, dist, notch_freq, notch_max_freq):
    """Smallest notch frequency >= notch_freq whose ramps fit in `dist`.

    A ramp v0 -> v1 under the notch law covers (v0+v1)/f of path distance, so
    for a total `v_sum` of ramp endpoints the shortest distance the requested
    f_n can do is v_sum/f_n. When the move is shorter than that, holding f_n
    means the move cannot change speed AT ALL -- which is what pins the toolhead
    to ~f_n*segment_length on short-segment geometry.

    Rather than give up on shaping (a hard accel step) or give up on motion
    (hold the entry speed), raise the frequency to exactly what fits:

        f = clamp(v_sum/dist, notch_freq, notch_max_freq)

    The ramp stays jerk-limited and keeps a shaper zero; the zero just sits
    above the target mode, so it attenuates that mode less. The degradation is
    continuous in `dist` -- as the move shrinks the zero slides smoothly up
    toward the cap, instead of falling off a cliff at v_sum/f_n. f is never
    lowered below notch_freq, so moves with room still get the exact target.

    notch_max_freq is what keeps this honest: past it the rise time is shorter
    than the emitter's slice resolution and the "ramp" is a step in all but
    name. A cap <= notch_freq disables the adaptation entirely.
    """
    if not notch_freq:
        return notch_freq
    cap = notch_max_freq or 0.0
    if cap <= notch_freq or dist <= 0.0:
        return notch_freq
    need = v_sum / dist
    if need <= notch_freq:
        return notch_freq
    return min(need, cap)


def adapted_notch_freq(vs, vc, ve, move_d, cons):
    """Runway-adapted EQUIVALENT frequency for one whole move (accel + decel).

    Returns an f_eq (see Constraints), not a mode frequency: with two zeros the
    pair is raised together, keeping their ratio -- and so both zeros' relative
    placement -- while the ramp shortens to fit. _with_notch() expands it back.
    """
    if not cons.notch_freq:
        return cons.notch_f_eq
    vc = max(vc, vs, ve)
    # Accel ramp needs (vs+vc)/f_eq, decel ramp needs (vc+ve)/f_eq.
    return ramp_freq_for(
        vs + 2.0 * vc + ve, move_d, cons.notch_f_eq, cons.notch_max_freq
    )


def _with_notch(cons, f_eq):
    # f_eq is a harmonic-mean frequency; expand it back into the pair at this
    # move's ratio so both zeros move together.
    ratio = cons.notch_hi / cons.notch_lo if cons.notch_lo else 1.0
    f_lo, f_hi = _pair_for(f_eq, ratio)
    return Constraints(
        a_const=cons.a_const,
        v_ceil=cons.v_ceil,
        max_jerk=cons.max_jerk,
        jerk_dt=cons.jerk_dt,
        notch_freq=f_lo,
        max_da=cons.max_da,
        notch_max_freq=cons.notch_max_freq,
        notch_freq2=f_hi,
    )


def jerk_dist(v0, v1, accel, jerk, notch_freq=None, notch_freq2=None):
    # Path distance to change speed v0 -> v1 under a symmetric jerk-limited
    # S-curve at constant max |accel| and max |jerk| (accel ramps 0 -> peak -> 0
    # so a(t) is continuous in the ideal law). Closed form: distance = mean
    # speed * duration, exact because a symmetric velocity S-curve's
    # time-average speed is (v0+v1)/2. Serves accel and decel identically (uses
    # |dv|). The actual emitter uses zero-order-held acceleration slices, so
    # this is the analytic planning model rather than a bit-exact rendering
    # model.
    dv = abs(v1 - v0)
    if dv <= 1e-12:
        return 0.0
    if notch_freq:
        # Per-ramp law, mirrored from Constraints.ramp_jerk. Both zeros survive
        # while the plateau is the notch's own (a_peak = dv*f_lo <= accel), and
        # then the duration is EXACTLY 1/f_lo + 1/f_hi -- independent of dv, and
        # equal to the single-notch 2/f_n when f_lo == f_hi. Past saturation the
        # plateau widens to dv/A and the rise edge keeps its zero on f_hi.
        f_lo = min(notch_freq, notch_freq2 or notch_freq)
        f_hi = max(notch_freq, notch_freq2 or notch_freq)
        saturated = accel > 0.0 and dv > accel / f_lo
        if saturated:
            f_rise = sat_rise_freq(dv, accel, f_lo, f_hi)
            t = dv / accel + 1.0 / f_rise
        else:
            t = 1.0 / f_lo + 1.0 / f_hi
        if jerk:
            # A max_jerk ceiling below the law's jerk stretches the rise edge;
            # fall back to the generic form so the estimate stays conservative.
            j = accel * f_rise if saturated else dv * f_lo * f_hi
            if jerk < j:
                return _jerk_dist_generic(v0, v1, dv, accel, jerk)
        return 0.5 * (v0 + v1) * t
    return _jerk_dist_generic(v0, v1, dv, accel, jerk)


def _jerk_dist_generic(v0, v1, dv, accel, jerk):
    # Fixed-jerk S-curve: triangular a(t) below saturation, trapezoidal above.
    if jerk is None or jerk <= 0.0 or accel <= 0.0:
        return abs(v1 * v1 - v0 * v0) / (2.0 * accel)
    if dv <= accel * accel / jerk:
        # Accel never saturates (triangular a(t)): T = 2*sqrt(dv/J).
        t = 2.0 * math.sqrt(dv / jerk)
    else:
        # Accel saturates at `accel` (trapezoidal a(t)): T = dv/A + A/J.
        t = dv / accel + accel / jerk
    return 0.5 * (v0 + v1) * t


def jerk_reach_v2(
    u0,
    dist,
    accel,
    jerk,
    v_ceil,
    notch_freq=None,
    notch_max_freq=None,
    notch_freq2=None,
):
    # Max u = v^2 reachable from v0=sqrt(u0) over path-distance `dist` under a
    # jerk-limited S-curve (constant max accel/jerk). Direction-symmetric
    # (forward accel == backward decel). Bisection on v1 over the O(1)
    # closed-form jerk_dist -- cheap enough for the hot lookahead loop (no ramp
    # integration). jerk None/0 (and no notch_freq) -> stock constant-accel
    # reach.
    #
    # Bisection stays valid under the notch law: distance is (v0+v1)/f while
    # a_peak is unsaturated and 0.5*(v0+v1)*(dv/A + 1/f) past that; both are
    # monotonically increasing in v1. It stays valid under the runway-adapted
    # frequency too: raising f only shortens the ramp, and f itself is
    # non-decreasing in v1, so the distance is still monotone.
    if dist <= 0.0:
        return u0
    if accel <= 0.0 or ((jerk is None or jerk <= 0.0) and not notch_freq):
        return u0 + 2.0 * accel * dist
    v0 = math.sqrt(max(u0, 0.0))
    # The frequency this move can actually ramp at is chosen per candidate
    # speed (ramp_freq_for), bounded above by the cap. Everything below uses
    # f_top for the "is any speed change possible at all" tests, since that is
    # the most permissive frequency available.
    f_top = notch_freq
    ratio = 1.0
    if notch_freq:
        # Distances are governed by the pair's HARMONIC mean (Constraints
        # docstring): a ramp costs 0.5*(v0+v1)*(1/f_lo + 1/f_hi) = (v0+v1)/f_eq,
        # which is the single-notch formula with f_eq == notch_freq. Adaptation
        # scales the pair, so track the pair through its ratio.
        f_lo = min(notch_freq, notch_freq2 or notch_freq)
        f_hi = max(notch_freq, notch_freq2 or notch_freq)
        ratio = f_hi / f_lo
        f_eq = 2.0 / (1.0 / f_lo + 1.0 / f_hi)
        f_top = f_eq
        cap = notch_max_freq or 0.0
        if cap > f_eq:
            f_top = cap
        # Even at f_top a ramp costs (v0+v1)/f of distance, and that does not
        # go to zero as v1 -> v0: its infimum is 2*v0/f_top. A move shorter
        # than that has no room for any speed change and reach is exactly u0.
        # This is the common short-move case, and testing it in closed form
        # skips the whole bisection.
        if dist < 2.0 * v0 / f_top:
            return u0
        if jerk:
            tri_dv_max = jerk / (f_lo * f_hi)
            sat_j = accel * f_hi if accel > 0.0 else 0.0
            if jerk < sat_j:
                v_ceil = min(v_ceil, v0 + tri_dv_max)
        notch_freq = f_eq

    def _fits(v1):
        f = notch_freq
        if notch_freq:
            f = ramp_freq_for(v0 + v1, dist, notch_freq, notch_max_freq)
        if not f:
            return jerk_dist(v0, v1, accel, jerk, f) <= dist
        # f is an f_eq; expand it back into the pair it stands for.
        f_a, f_b = _pair_for(f, ratio)
        return jerk_dist(v0, v1, accel, jerk, f_a, f_b) <= dist

    hi = max(v0, v_ceil)
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


def _ramp_up_jerk(v0, v1, cons, collect=True):
    # Jerk-limited acceleration ramp taking velocity v0 -> v1 (v1 >= v0). The
    # acceleration starts at ~0, rises toward a_max in steps bounded by J*dt,
    # then falls back toward ~0 landing on v1. Integrated at fixed jerk_dt; the
    # last slice lands exactly on v1 (velocity continuity is exact). Returns
    # (slices, total_d); slices is None when collect=False (distance-only, for
    # peak bisection).
    #
    # The taper is enforced by a "brake" cap a_brake = sqrt(2*J*(v1-v)): along
    # it da/dt = -J exactly, so following min(a_curve, a_brake, a+J*dt)
    # guarantees a bounded rise (a+J*dt) and a jerk-feasible fall to 0 at v1.
    #
    # Per-RAMP jerk: fixed (cons.max_jerk) in fixed-jerk mode, or dv*f_n^2 when
    # a notch frequency is configured, which holds this ramp's shaper zero on
    # f_n instead of letting it slide with dv. See Constraints.ramp_jerk.
    J = cons.ramp_jerk(v1 - v0)
    dt0 = cons.jerk_dt
    if cons.max_da is not None and J is not None and J > 0.0:
        dt0 = min(dt0, cons.max_da / J)
    # Two-zero mode caps the plateau at dv*f_lo, which is what turns the
    # triangle into the trapezoid that nulls both modes. None in single-zero
    # mode, where the peak already lands there on its own.
    a_notch = cons.ramp_accel_cap(v1 - v0)
    slices = [] if collect else None
    total = 0.0
    if v1 <= v0 + 1e-12 or J is None or J <= 0.0:
        return slices, total
    v = v0
    a = 0.0
    guard = 0
    while v < v1 - 1e-9:
        guard += 1
        if guard > 500000:
            # Did not converge to v1 -> signal infeasible, never emit.
            return None, float("inf")
        rem = v1 - v
        a_curve = cons.a_max(v)
        if a_curve is None or a_curve <= 0.0:
            a_curve = cons.a_const if cons.a_const else 1e30
        if a_notch is not None and a_notch < a_curve:
            a_curve = a_notch
        a_brake = math.sqrt(2.0 * J * rem)
        a_new = min(a_curve, a_brake, a + J * dt0)
        if a_new <= 0.0:
            a_new = min(a_curve, a_brake)
            if a_new <= 0.0:
                return None, float("inf")
        v_next = v + a_new * dt0
        this_dt = dt0
        if v_next >= v1:
            v_next = v1
            this_dt = (v1 - v) / a_new
        dist = 0.5 * (v + v_next) * this_dt
        if collect:
            slices.append((this_dt, 0.0, 0.0, v, v_next, a_new, dist))
        total += dist
        v, a = v_next, a_new
    return slices, total


def _decel_from_accel(acc_slices):
    # Time-reverse an increasing-velocity jerk ramp into a decel slice list:
    # an accel slice (dt,0,0, v_lo, v_hi, a, dist) becomes decel
    # (0,0,dt, v_hi, v_hi, a, dist) (velocity v_hi -> v_lo). Reversed order so
    # the chain runs vc -> ve and stays velocity-continuous.
    dec = []
    for at, ct, dt, sv, cv, a, dist in reversed(acc_slices):
        dec.append((0.0, 0.0, at, cv, cv, a, dist))
    return dec


def _notch_peak_limit(vs, vc, ve, cons):
    if not cons.notch_freq:
        return vc
    if cons.max_jerk:
        # Unsaturated law is J = dv*f_lo*f_hi, so a jerk ceiling caps the ramp
        # dv at max_jerk/(f_lo*f_hi); saturated it is J = A*f_hi.
        sat_j = (
            cons.a_const * cons.notch_hi
            if cons.a_const and cons.a_const > 0.0
            else 0.0
        )
        if not sat_j or cons.max_jerk < sat_j:
            dv_max = cons.max_jerk / (cons.notch_lo * cons.notch_hi)
            return min(vc, vs + dv_max, ve + dv_max)
    return vc


def _peak_velocity_jerk(vs, ve, move_d, cons):
    # Highest cruise a jerk-limited move can reach: bisection on the ramp
    # distance (accel vs->vp plus decel vp->ve). Returns max(vs,ve) when even
    # that connecting ramp overfills move_d (caller then falls back to sharp).
    lo = max(vs, ve)
    hi = _notch_peak_limit(vs, cons.v_ceil, ve, cons)
    need_lo = (
        _ramp_up_jerk(vs, lo, cons, collect=False)[1]
        + _ramp_up_jerk(ve, lo, cons, collect=False)[1]
    )
    if need_lo >= move_d:
        return lo
    # Each probe integrates TWO full ramps, so this is the most expensive thing
    # in the emitter. Stop at a velocity tolerance rather than a fixed 32 steps;
    # 1e-4 mm/s is well under step-generator resolution and typically halves the
    # probe count.
    for _ in range(32):
        if hi - lo <= 1e-4:
            break
        mid = 0.5 * (lo + hi)
        need = (
            _ramp_up_jerk(vs, mid, cons, collect=False)[1]
            + _ramp_up_jerk(ve, mid, cons, collect=False)[1]
        )
        if need > move_d:
            hi = mid
        else:
            lo = mid
    return lo


def _with_jerk(cons, J):
    # Shallow copy of the constraints with a different jerk (for the fallback
    # jerk-raising search). notch_freq is deliberately DROPPED: under the notch
    # law a ramp's distance is (v0+v1)/f_n no matter what J is, so a move too
    # short to fit it is infeasible at every J and the search would never
    # converge. Falling back to plain fixed-jerk is the right semantics: too
    # short to shape, so stop preserving the requested notch.
    return Constraints(
        a_const=cons.a_const,
        v_ceil=cons.v_ceil,
        max_jerk=J,
        jerk_dt=cons.jerk_dt,
        notch_freq=None,
        max_da=cons.max_da,
    )


def _emit_jerk_core(vs, vc, ve, move_d, cons, collect=True):
    # One jerk-limited profile at the per-ramp jerk: ramp vs->vc, cruise, ramp
    # vc->ve. Returns None when the move can't be jerk-limited (endpoints
    # unreachable within move_d even at the connecting peak).
    vc = max(_notch_peak_limit(vs, vc, ve, cons), vs, ve)
    acc, d_acc = _ramp_up_jerk(vs, vc, cons, collect=collect)
    dec_acc, d_dec = _ramp_up_jerk(ve, vc, cons, collect=collect)
    if not math.isfinite(d_acc) or not math.isfinite(d_dec):
        return None
    cruise_d = move_d - d_acc - d_dec
    if cruise_d < -1e-9:
        vc = max(_peak_velocity_jerk(vs, ve, move_d, cons), vs, ve)
        acc, d_acc = _ramp_up_jerk(vs, vc, cons, collect=collect)
        dec_acc, d_dec = _ramp_up_jerk(ve, vc, cons, collect=collect)
        if not math.isfinite(d_acc) or not math.isfinite(d_dec):
            return None
        cruise_d = move_d - d_acc - d_dec
        if cruise_d < -1e-6 * max(1.0, move_d):
            return None
        # Within that tolerance the two re-integrated ramps can overfill move_d
        # by up to 1e-6*move_d (sub-nanometer at printer scale, orders of
        # magnitude below one microstep). Clamping the cruise to zero leaves
        # sum(dist) larger than move_d by that residual; the step compressor
        # absorbs it, and trimming a slice instead would break the per-slice
        # "trapq-implied distance == dist" invariant.
        cruise_d = max(0.0, cruise_d)
    if not collect:
        return []
    segs = list(acc)
    if cruise_d > 1e-9 and vc > 1e-9:
        segs.append((0.0, cruise_d / vc, 0.0, vc, vc, 0.0, cruise_d))
    elif cruise_d > 1e-9:
        return None
    segs.extend(_decel_from_accel(dec_acc))
    if not segs and move_d > 1e-9:
        return None
    return segs


def _emit_jerk(vs, vc, ve, move_d, cons):
    # Discretized jerk-limited profile. If the move is infeasible at the
    # configured jerk, raise the jerk to the minimum value that fits and emit
    # there; acceleration remains staircase-shaped with bounded per-slice
    # changes. A hard step would become a multi-step position jump in
    # stepcompress on any downstream that reads accel.
    segs = _emit_jerk_core(vs, vc, ve, move_d, cons)
    if segs is not None:
        return segs
    # Seed the search from the jerk this move WOULD have used. In notch mode
    # max_jerk may be unset (the notch law supplies the jerk), so fall back to
    # the notch-law jerk for the move's largest ramp.
    J0 = cons.max_jerk
    if not J0:
        J0 = cons.ramp_jerk(max(abs(vc - vs), abs(vc - ve)))
    if not J0:
        return None  # nothing to ramp -> caller does sharp
    hi = J0
    feasible_hi = False
    for _ in range(40):  # expand until feasible
        hi *= 2.0
        s = _emit_jerk_core(
            vs, vc, ve, move_d, _with_jerk(cons, hi), collect=False
        )
        if s is not None:
            feasible_hi = True
            break
    if not feasible_hi:
        return None  # truly degenerate -> caller does sharp
    lo = hi * 0.5  # last infeasible jerk
    best = hi
    for _ in range(24):  # bisection for the minimum feasible J'
        mid = 0.5 * (lo + hi)
        s = _emit_jerk_core(
            vs, vc, ve, move_d, _with_jerk(cons, mid), collect=False
        )
        if s is not None:
            hi, best = mid, mid
        else:
            lo = mid
    return _emit_jerk_core(vs, vc, ve, move_d, _with_jerk(cons, best))


def _emit_sharp(vs, vc, ve, move_d, cons):
    # Constant-accel accel/cruise/decel profile (the classic trapezoid), used
    # when a move is too short to jerk-limit. Closed form at cons.a_const --
    # this is exactly what the a_max lookahead approved, so it always fits.
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


def emit_profile(vs, vc, ve, move_d, cons):
    """Emit ONE monotonic constant-accel segment list for a single move.

    Each segment is (accel_t, cruise_t, decel_t, start_v, cruise_v, accel, dist)
    -- the tuple the toolhead/extruder trapq consume. Invariant-guaranteed by
    construction: non-negative times/speeds, no decel past zero, per-segment
    trapq-implied distance == dist, inter-segment velocity continuity, and
    sum(dist) == move_d to within 1e-6*move_d (see _emit_jerk_core).

    When cons.max_jerk (or cons.notch_freq) is set, acceleration is emitted as
    constant-accel slices whose per-slice changes are jerk bounded; on a move
    too short to jerk-limit, it falls back to the sharp constant-accel profile.

    If the move is too short to ramp at cons.notch_freq, the notch is RAISED to
    whatever does fit (see adapted_notch_freq) so the move can still change
    speed. The lookahead applies the identical rule, so the cruise it asked for
    is the cruise this renders.
    """
    if move_d <= 0.0:
        return []
    if cons.max_jerk or cons.notch_freq:
        if cons.notch_freq:
            f_eff = adapted_notch_freq(vs, vc, ve, move_d, cons)
            if f_eff != cons.notch_f_eq:
                cons = _with_notch(cons, f_eff)
        segs = _emit_jerk(vs, vc, ve, move_d, cons)
        if segs is not None:
            return segs
        # else: jerk-infeasible for this short move -> sharp fallback below
    return _emit_sharp(vs, vc, ve, move_d, cons)


def _split_segment(seg, d1):
    """Split one slice at path-distance d1 into (first, second).

    Preserves the trapq invariant exactly: each piece's implied distance equals
    its recorded dist, and the pieces are velocity-continuous with each other.
    """
    at, ct, dt, sv, cv, a, dist = seg
    d2 = dist - d1
    if at > 0.0:
        # Accel slice: sv -> cv at +a. d(t) = sv*t + a*t^2/2.
        if a > 0.0:
            t1 = (math.sqrt(max(sv * sv + 2.0 * a * d1, 0.0)) - sv) / a
        else:
            t1 = d1 / sv if sv > 0.0 else 0.0
        t1 = min(max(t1, 0.0), at)
        v1 = sv + a * t1
        return (
            (t1, 0.0, 0.0, sv, v1, a, d1),
            (at - t1, 0.0, 0.0, v1, cv, a, d2),
        )
    if dt > 0.0:
        # Decel slice: starts at cv, v(t) = cv - a*t. d(t) = cv*t - a*t^2/2.
        if a > 0.0:
            disc = max(cv * cv - 2.0 * a * d1, 0.0)
            t1 = (cv - math.sqrt(disc)) / a
        else:
            t1 = d1 / cv if cv > 0.0 else 0.0
        t1 = min(max(t1, 0.0), dt)
        v1 = cv - a * t1
        return (
            (0.0, 0.0, t1, cv, cv, a, d1),
            (0.0, 0.0, dt - t1, v1, v1, a, d2),
        )
    # Cruise slice at cv.
    t1 = d1 / cv if cv > 0.0 else 0.0
    return (
        (0.0, t1, 0.0, cv, cv, 0.0, d1),
        (0.0, max(ct - t1, 0.0), 0.0, cv, cv, 0.0, d2),
    )


def split_segments(segs, lengths):
    """Cut one profile into per-move slice lists at cumulative `lengths`.

    A jerk ramp that spans several moves is emitted ONCE, over the run's total
    distance, so the accel pulse keeps its shape (and therefore its zero)
    across move boundaries. The toolhead still needs the slices bucketed per
    move, because each move carries its own direction and extra-axis ratios.
    Slices straddling a boundary are split with _split_segment, so every
    bucket's distances sum to exactly that move's length.
    """
    out = [[] for _ in lengths]
    if not lengths:
        return out
    i = 0
    remaining = lengths[0]
    for seg in segs:
        d = seg[6]
        while d > 0.0:
            if i >= len(out) - 1 and remaining <= 0.0:
                # Numerical spill past the last boundary: keep it in the last
                # bucket rather than dropping distance on the floor.
                out[-1].append(seg)
                d = 0.0
                break
            if d <= remaining + 1e-12 or i >= len(out) - 1:
                out[i].append(seg)
                remaining -= d
                d = 0.0
            else:
                first, second = _split_segment(seg, remaining)
                out[i].append(first)
                d -= remaining
                seg = second
                remaining = 0.0
            if remaining <= 1e-12 and i < len(out) - 1:
                i += 1
                remaining = lengths[i]
    return out


# Every reason notch_loss_reasons() can ever return. The toolhead uses this to
# stop calling the diagnostic once it has reported all of them.
LOSS_REASONS = frozenset(
    (
        "jerk_clamped",
        "insufficient_runway",
        "notch_raised",
        "second_notch_saturated",
    )
)


def notch_loss_reasons(vs, vc, ve, move_d, cons):
    """Return reasons the requested notch frequency is not preserved exactly.

    Reports only ACTIONABLE losses -- ones a config change can remove. The
    zero-order-hold discretization of the ramp always perturbs the zero a
    little; that is inherent to emitting the profile as constant-accel slices
    (tighten it with a smaller `unified_jerk_dt`), so it is not reported here.
    Flagging it unconditionally made every ordinary print log a warning.
    """
    if not cons.notch_freq:
        return []
    reasons = set()
    f_eff = adapted_notch_freq(vs, vc, ve, move_d, cons)
    if f_eff > cons.notch_f_eq:
        # The move was too short to ramp at the target, so the zero slid up to
        # fit. Motion is still shaped, just not on the requested mode.
        reasons.add("notch_raised")
        cons = _with_notch(cons, f_eff)
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
        target_j = dv * cons.notch_lo * cons.notch_hi
        if cons.max_jerk:
            sat_j = (
                cons.a_const
                * sat_rise_freq(dv, cons.a_const, cons.notch_lo, cons.notch_hi)
                if cons.a_const and cons.a_const > 0.0
                else 0.0
            )
            required_j = min(target_j, sat_j) if sat_j else target_j
            if cons.max_jerk < required_j:
                reasons.add("jerk_clamped")
    req_d = (
        _ramp_up_jerk(vs, max(vc, vs), cons, collect=False)[1]
        + _ramp_up_jerk(ve, max(vc, ve), cons, collect=False)[1]
    )
    # collect=False: this is a feasibility probe, so never build the slice list
    # just to throw it away.
    if (
        req_d > move_d + 1e-9
        or _emit_jerk_core(vs, vc, ve, move_d, cons, collect=False) is None
    ):
        reasons.add("insufficient_runway")
    return sorted(reasons)
