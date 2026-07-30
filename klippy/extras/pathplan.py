# Jerk-limited motion planning with a per-ramp notch law
#
# A single phase-plane emitter that renders each move as discretized
# jerk-limited acceleration slices instead of one hard accel step, plus a
# jerk-aware lookahead that approximates the boundary speeds that emitter can
# render.
#
# Jerk is derived per ramp from a NOTCH FREQUENCY. A jerk-limited ramp whose
# acceleration does not saturate is
# triangular in a(t), which is an input shaper with a zero at 1/T_rise; the
# notch law pins T_rise = 1/f_n via J = dv * f_n^2. If the requested speed
# change would exceed max_accel, the saturated law sets J = max_accel * f_n and
# deliberately inserts a max-accel plateau; the ramp edge still has duration
# 1/f_n, preserving the ideal zero while honoring the accel limit. The actual
# emitter uses zero-order-held constant-accel slices, so discretization
# perturbs the exact zero slightly.
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

# Slice budget one ramp is allowed to ask for at CONFIG time. Settings that
# need more than this are a typo rather than a machine: at the default jerk_dt
# it describes a four second ramp. ToolHead._check_unified_settings rejects
# them where a bad value is still a recoverable config error.
MAX_RAMP_SLICES = 4096

# Runtime backstop for the integrator, deliberately far above MAX_RAMP_SLICES.
# Exceeding a slice budget is not a recoverable condition down here: it is
# reported by returning infeasible, that becomes InfeasibleProfile, and nothing
# in klippy catches it -- the print dies on whatever G1 reached it. So this
# limit only has to catch NON-CONVERGENCE. The margin over MAX_RAMP_SLICES is
# what accel changes the config check cannot see (M204 lowers max_accel
# mid-print, which lengthens a saturated plateau) spend without going fatal.
RAMP_SLICE_BACKSTOP = 500000

# How much peak jerk a spectral-null correction may add, as a multiple of the
# baseline ramp's own peak. The correction is minimum-norm in acceleration and
# says nothing about adjacent slices, so unbounded it parks the zero by
# emitting acceleration steps larger than the profile the planner asked for.
# Measured residual against what the bound allows, 0->100 mm/s at jerk_dt=1ms:
#
#   limit   55 Hz single   55/75 @55   55/75 @75
#   1.0     REFUSED        REFUSED     REFUSED     (baseline peak IS the limit)
#   1.1     4.6e-17        1.2e-02     1.0e-02
#   1.25    4.6e-17        4.0e-03     3.3e-03
#   1.5     4.6e-17        5.5e-18     3.0e-17     (bound stops binding)
#
# 1.0 is unusable: the baseline's own peak sits exactly on it, so any positive
# perturbation there drives the correction to zero. 1.5 never binds, which
# makes it no bound at all. 1.25 keeps the single-notch null exact, still
# improves a two-notch ramp about 3.5x over the unsolved emitter, and caps the
# added jerk at a quarter -- against the 32% the unbounded solve took.
SPECTRAL_NULL_JERK_LIMIT = 1.25

# A solved profile must preserve the baseline endpoint contract. The Gram solve
# is deliberately small, but close target frequencies can still make its rows
# nearly dependent. Reject numerical answers that move velocity or distance
# instead of handing an inconsistent profile to trapq.
SPECTRAL_NULL_INVARIANT_RTOL = 1e-9
SPECTRAL_NULL_INVARIANT_ATOL = 1e-12


class InfeasibleProfile(Exception):
    pass


class Constraints:
    """Per-move motion limits for the jerk-limited emitter and lookahead.

    a_const    -> constant max |acceleration| (mm/s^2). With the notch law,
                  unsaturated moves are governed by a_peak = dv * f_n; larger
                  moves use a designed max-accel plateau.
    v_ceil     -> hard speed ceiling (mm/s); the reachability search stops here.
    jerk_dt    -> integration time step (s) for the emitted ramp.
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
        jerk_dt=0.001,
        notch_freq=None,
        notch_freq2=None,
        spectral_null=False,
    ):
        self.a_const = a_const
        self.v_ceil = v_ceil
        self.jerk_dt = jerk_dt
        self.notch_freq = notch_freq
        self.notch_freq2 = notch_freq2
        # Solve emitted slice accelerations for an exact null instead of
        # sampling the ideal curve. See solve_ramp_null. spectral_loss records
        # what actually happened so notch_loss_reasons can report it -- a
        # refused or scaled-back solve loses the null the user asked for, and
        # losing it silently is worse than not offering it.
        self.spectral_null = spectral_null
        self.spectral_loss = None
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
# reporting purposes. The emitter's zero-order hold already leaves a few times
# 1e-4 at a parked zero, so this sits an order of magnitude above the floor:
# below it the forced A/dv zero (or one of its harmonics) happened to land near
# enough to the second mode that nothing was really lost, and warning would be
# noise.
SECOND_NOTCH_EPS = 0.01


def notch_dist(v0, v1, accel, notch_freq, notch_freq2=None):
    # Path distance to change speed v0 -> v1 under a symmetric jerk-limited
    # notch profile (accel ramps 0 -> peak -> 0, so a(t) is continuous in the
    # ideal law). Closed form: distance = mean speed * duration, exact because
    # a symmetric velocity S-curve's time-average speed is (v0+v1)/2. Serves
    # accel and decel identically (uses |dv|). The actual emitter uses
    # zero-order-held acceleration slices, so this is the analytic planning
    # model rather than a bit-exact rendering model.
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
    # Bisection on v1 over the O(1) closed-form notch_dist is cheap enough for
    # the hot lookahead loop (no ramp integration).
    #
    # Bisection stays valid under the notch law: distance is (v0+v1)/f while
    # a_peak is unsaturated and 0.5*(v0+v1)*(dv/A + 1/f) past that; both are
    # monotonically increasing in v1.
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
    # for any speed change and reach is exactly u0. This is the common
    # short-move case, and testing it in closed form skips the whole bisection.
    if dist < 2.0 * v0 / f_eq:
        return u0

    def _fits(v1):
        return notch_dist(v0, v1, accel, f_lo, f_hi) <= dist

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


def ramp_fits_slice_budget(v0, v1, cons, limit=MAX_RAMP_SLICES):
    """True if the ramp v0 -> v1 integrates within `limit` slices.

    Asks the emitter's own integrator instead of re-deriving the slice count
    from f_n/dt/accel, so a config check and the ramp it is validating cannot
    drift apart as the ramp law changes.
    """
    return not math.isinf(
        _ramp_up_jerk(v0, v1, cons, collect=False, limit=limit)[1]
    )


def _ramp_up_jerk(v0, v1, cons, collect=True, limit=RAMP_SLICE_BACKSTOP):
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
    # Per-ramp jerk is derived from the configured notch frequencies. See
    # Constraints.ramp_jerk.
    J = cons.ramp_jerk(v1 - v0)
    dt0 = cons.jerk_dt
    # Two-zero mode caps the plateau at dv*f_lo, which is what turns the
    # triangle into the trapezoid that nulls both modes. None in single-zero
    # mode, where the peak already lands there on its own.
    a_notch = cons.ramp_accel_cap(v1 - v0)
    slices = [] if collect else None
    total = 0.0
    if v1 <= v0 + 1e-12 or J is None or J <= 0.0:
        return slices, total
    # The unsaturated ramp lasts notch_period however small dv is, so its slice
    # count is known before the first step. Answer without spinning.
    if cons.notch_period and cons.notch_period / dt0 > limit:
        return None, float("inf")
    v = v0
    a = 0.0
    guard = 0
    while v < v1 - 1e-9:
        guard += 1
        if guard > limit:
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


def _factor_min_norm(rows):
    """Invert the m x m Gram matrix A A^T for a minimum-norm solve.

    m is 2 + 2*len(freqs), so at most 6. Returns None when the rows are rank
    deficient, which callers must treat as "leave the ramp alone".
    """
    m = len(rows)
    n = len(rows[0])
    aug = [
        [math.fsum(rows[i][k] * rows[j][k] for k in range(n)) for j in range(m)]
        + [1.0 if i == j else 0.0 for j in range(m)]
        for i in range(m)
    ]
    scale = max(abs(aug[i][j]) for i in range(m) for j in range(m)) or 1.0
    for col in range(m):
        piv = max(range(col, m), key=lambda r: abs(aug[r][col]))
        if abs(aug[piv][col]) < 1e-12 * scale:
            return None
        aug[col], aug[piv] = aug[piv], aug[col]
        pv = aug[col][col]
        for c in range(col, 2 * m):
            aug[col][c] /= pv
        for r in range(m):
            if r == col:
                continue
            f = aug[r][col]
            if f:
                for c in range(col, 2 * m):
                    aug[r][c] -= f * aug[col][c]
    return [row[m:] for row in aug]


def _rebuild_ramp(slices, accels):
    """Re-integrate a ramp with new per-slice accelerations, dt unchanged."""
    out = []
    v = slices[0][3]
    for (dt, _c, _d, _sv, _cv, _a, _dist), a in zip(slices, accels):
        vn = v + a * dt
        out.append((dt, 0.0, 0.0, v, vn, a, 0.5 * (v + vn) * dt))
        v = vn
    return out


def _null_rows(slices, freqs):
    """Equality rows and targets for solve_ramp_null. See it for the model."""
    n = len(slices)
    dt = [s[0] for s in slices]
    a0 = [s[5] for s in slices]
    dv = math.fsum(a0[i] * dt[i] for i in range(n))
    dist = math.fsum(s[6] for s in slices)
    total_t = math.fsum(dt)
    # d(distance)/d(a_j): raising slice j's accel lifts velocity for the rest
    # of the ramp, so distance moves by dt_j^2/2 + dt_j * (time after j).
    tail = []
    acc_t = 0.0
    for j in range(n):
        acc_t += dt[j]
        tail.append(total_t - acc_t)
    rows = [dt, [dt[j] * dt[j] * 0.5 + dt[j] * tail[j] for j in range(n)]]
    # Only the part of the distance the accelerations control. The entry speed
    # carries v0*T of it regardless, and leaving that in the target makes the
    # residual non-zero for any ramp that starts moving -- which breaks the
    # null-space property the alpha scaling depends on.
    targets = [dv, dist - slices[0][3] * total_t]
    edges = [0.0]
    for d in dt:
        edges.append(edges[-1] + d)
    for f in freqs:
        w = 2.0 * math.pi * f
        re_row = []
        im_row = []
        for j in range(n):
            c0, s0 = math.cos(w * edges[j]), math.sin(w * edges[j])
            c1, s1 = math.cos(w * edges[j + 1]), math.sin(w * edges[j + 1])
            re_row.append((s1 - s0) / w)
            im_row.append((c1 - c0) / w)
        rows.append(re_row)
        rows.append(im_row)
        targets.append(0.0)
        targets.append(0.0)
    return rows, targets, a0, dv


def solve_ramp_null(slices, freqs, max_accel=None):
    """Re-solve slice accelerations so the emitted spectrum nulls at `freqs`.

    The ideal ramp is rect(1/f_hi) * rect(1/f_lo) in a(t), whose zeros sit on
    the modes -- but only in continuous time. Emitting it as constant-accel
    slices smears them to about 1.3% of dv at the default jerk_dt, and the only
    lever for that has been jerk_dt itself: halving it halves the residual and
    doubles the motion-queue traffic, bottoming out near 0.11% at 379 slices.

    It does not have to be approximated. The emitted spectrum is LINEAR in the
    slice accelerations and there are far more of those than constraints, so
    the null can be SOLVED for: 2 + 2*len(freqs) equality rows -- preserve dv,
    preserve distance, zero the real and imaginary parts at each mode --
    against len(slices) unknowns.

    Sampling the ideal curve was only ever a convenient way to choose those
    numbers. It makes the profile LOOK like the ideal on a plot, which was
    never the goal; an unconstrained successful solve is a slightly worse
    pointwise fit (~2%) and an exact spectral one.

    dv, distance, and terminal velocity are checked after rebuilding, so
    nothing upstream changes: runway is still sized from notch_dist and the
    next move still plans from the same exit speed. A numerically unstable
    answer is rejected.

    A correction that would exceed max_accel, or reverse acceleration inside
    the ramp, is SCALED BACK rather than refused. a0 already satisfies the dv
    and distance rows, so the correction lies in their null space and any
    multiple of it preserves both exactly -- only the spectral rows trade, and
    linearly. A ramp with little acceleration of its own takes the largest
    share it can carry instead of leaving the whole residual in place.

    Returns corrected slices, or None if rank deficient or too few slices.
    """
    n = len(slices)
    m = 2 + 2 * len(freqs)
    if n <= m:
        return None
    # A zero or negative frequency has no spectral row to build -- the rows
    # divide by w. The emitter never passes one, but this is public and the
    # contract is that it returns None rather than raising.
    if any(f <= 0.0 for f in freqs):
        return None
    rows, targets, a0, dv = _null_rows(slices, freqs)
    if abs(dv) <= 1e-12:
        return None
    # Slices already sitting on max_accel cannot move up. A saturated ramp
    # holds a whole plateau there, and the correction is one vector in the null
    # space of the dv and distance rows -- its components cannot be scaled
    # independently without leaving that space, so one blocked slice would
    # otherwise force the entire correction to zero.
    #
    # Drop them from the UNKNOWNS rather than adding an equality row each. A
    # row each keeps m growing with the plateau, and m is the dimension of the
    # Gram matrix that gets inverted -- a low runtime M204 acceleration
    # lengthens the plateau without limit, so that made the solve O(plateau^3)
    # in pure Python and could stall the planner mid-print. Eliminating the
    # variables instead leaves m at 4 or 6 whatever the ramp looks like; the
    # pinned slices just move their fixed contribution to the right-hand side.
    free = list(range(n))
    if max_accel is not None:
        lim = max_accel * (1.0 - 1e-9)
        free = [k for k in range(n) if abs(a0[k]) < lim]
        if len(free) <= m:
            return None
    if len(free) < n:
        pinned = set(range(n)) - set(free)
        targets = [
            targets[i] - math.fsum(rows[i][k] * a0[k] for k in pinned)
            for i in range(m)
        ]
        rows = [[row[k] for k in free] for row in rows]
    inv = _factor_min_norm(rows)
    if inv is None:
        return None
    a_free = [a0[k] for k in free]
    nf = len(free)
    resid = [
        targets[i] - math.fsum(rows[i][k] * a_free[k] for k in range(nf))
        for i in range(m)
    ]
    lam = [math.fsum(inv[i][j] * resid[j] for j in range(m)) for i in range(m)]
    delta = [0.0] * n
    for j, k in enumerate(free):
        delta[k] = math.fsum(lam[i] * rows[i][j] for i in range(m))
    alpha = 1.0
    # Relative: a pinned slice's delta solves to a rounding residual, not to
    # exactly zero, and an absolute epsilon reads that as a real correction
    # against zero remaining headroom.
    tiny = 1e-9 * max(abs(a) for a in a0)
    for k in range(n):
        d = delta[k]
        if abs(d) <= tiny:
            continue
        if a0[k] * d > 0.0:
            if max_accel is None:
                continue
            room = max_accel - abs(a0[k])
            if room <= 0.0:
                return None
        else:
            # Never let acceleration cross zero: past that the slice list is
            # no longer a ramp and stepcompress rejects the step sequence.
            room = abs(a0[k])
        if room < alpha * abs(d):
            alpha = room / abs(d)
    # Jerk. The correction is minimum-NORM, which says nothing about adjacent
    # slices, so left alone it parks the zero by introducing acceleration steps
    # bigger than the profile the planner asked for -- measured 10% to 32%
    # amplification. Jerk is linear in alpha, so bound it in the same ratio
    # test instead of validating afterwards, and take a partial null rather
    # than a violated one.
    #
    # Boundary-inclusive: the pulse is embedded in zero acceleration either
    # side, so (a_0 - 0)/dt_0 and (0 - a_n-1)/dt_n-1 are jerk steps too, and
    # they are exactly the ones a correction tends to enlarge.
    dt = [s[0] for s in slices]
    j_lim = 0.0
    steps = []
    prev_a = 0.0
    prev_d = 0.0
    for k in range(n + 1):
        cur_a = a0[k] if k < n else 0.0
        cur_d = delta[k] if k < n else 0.0
        span = dt[k] if k < n else dt[-1]
        if span > 0.0:
            j0 = (cur_a - prev_a) / span
            dj = (cur_d - prev_d) / span
            steps.append((j0, dj))
            if abs(j0) > j_lim:
                j_lim = abs(j0)
        prev_a, prev_d = cur_a, cur_d
    if j_lim > 0.0:
        lim = j_lim * SPECTRAL_NULL_JERK_LIMIT
        for j0, dj in steps:
            if abs(dj) <= 1e-12:
                continue
            hi = (lim - j0) / dj if dj > 0.0 else (-lim - j0) / dj
            if hi < alpha:
                alpha = hi
    if alpha <= 0.0:
        return None
    solved = _rebuild_ramp(slices, [a0[k] + alpha * delta[k] for k in range(n)])
    solved_dv = math.fsum(s[5] * s[0] for s in solved)
    baseline_d = math.fsum(s[6] for s in slices)
    solved_d = math.fsum(s[6] for s in solved)

    def invariant_close(actual, expected):
        tol = SPECTRAL_NULL_INVARIANT_ATOL + (
            SPECTRAL_NULL_INVARIANT_RTOL * abs(expected)
        )
        return abs(actual - expected) <= tol

    if not invariant_close(solved_dv, dv) or not invariant_close(
        solved_d, baseline_d
    ):
        return None
    if not invariant_close(solved[-1][4], slices[-1][4]):
        return None
    return solved


def _apply_spectral_null(ramp, cons):
    """Solve one ramp for an exact null; leave it alone if that is not possible.

    Applied to both the accel ramp and the accel-shaped ramp that
    _decel_from_accel reverses into the decel. Time reversal preserves the
    MAGNITUDE spectrum, so a decel built from a solved ramp carries the same
    null -- there is no external phase for the reversal to disturb.
    """
    if not ramp or not cons.spectral_null or not cons.notch_freq:
        return ramp
    if cons.notch_lo == cons.notch_hi:
        freqs = [cons.notch_lo]
    else:
        freqs = [cons.notch_lo, cons.notch_hi]
    solved = solve_ramp_null(ramp, freqs, cons.a_const)
    if solved is None:
        # Rank deficient, too few free slices, or nothing the ramp could carry.
        cons.spectral_loss = "spectral_null_failed"
        return ramp
    # Report what was ACHIEVED, not what was attempted. A correction scaled
    # back for max_accel or jerk leaves residual behind, and the whole point of
    # the option is that the null is exact -- so say when it is not.
    if cons.spectral_loss != "spectral_null_failed":
        dv = math.fsum(s[5] * s[0] for s in solved)
        if abs(dv) > 1e-12:
            edges = [0.0]
            for s in solved:
                edges.append(edges[-1] + s[0])
            for f in freqs:
                w = 2.0 * math.pi * f
                re = im = 0.0
                for j, s in enumerate(solved):
                    c0, s0 = math.cos(w * edges[j]), math.sin(w * edges[j])
                    c1, s1 = (
                        math.cos(w * edges[j + 1]),
                        math.sin(w * edges[j + 1]),
                    )
                    re += s[5] * (s1 - s0) / w
                    im += s[5] * (c1 - c0) / w
                if math.hypot(re, im) > 1e-6 * abs(dv):
                    cons.spectral_loss = "spectral_null_partial"
                    break
    return solved


def _decel_from_accel(acc_slices):
    # Time-reverse an increasing-velocity jerk ramp into a decel slice list:
    # an accel slice (dt,0,0, v_lo, v_hi, a, dist) becomes decel
    # (0,0,dt, v_hi, v_hi, a, dist) (velocity v_hi -> v_lo). Reversed order so
    # the chain runs vc -> ve and stays velocity-continuous.
    dec = []
    for at, ct, dt, sv, cv, a, dist in reversed(acc_slices):
        dec.append((0.0, 0.0, at, cv, cv, a, dist))
    return dec


def _peak_velocity_jerk(vs, ve, move_d, cons):
    # Highest cruise a jerk-limited move can reach: bisection on the ramp
    # distance (accel vs->vp plus decel vp->ve). Returns max(vs,ve) when even
    # that connecting ramp overfills move_d (the profile is infeasible).
    lo = max(vs, ve)
    hi = cons.v_ceil
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


def _emit_jerk_core(vs, vc, ve, move_d, cons, collect=True):
    # One jerk-limited profile at the per-ramp jerk: ramp vs->vc, cruise, ramp
    # vc->ve. Returns None when the move can't be jerk-limited (endpoints
    # unreachable within move_d even at the connecting peak).
    vc = max(vc, vs, ve)
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
    if cruise_d > 1e-9 and vc <= 1e-9:
        return None
    if not collect:
        return []
    segs = list(_apply_spectral_null(acc, cons))
    if cruise_d > 1e-9 and vc > 1e-9:
        segs.append((0.0, cruise_d / vc, 0.0, vc, vc, 0.0, cruise_d))
    elif cruise_d > 1e-9:
        return None
    segs.extend(_decel_from_accel(_apply_spectral_null(dec_acc, cons)))
    if not segs and move_d > 1e-9:
        return None
    return segs


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


def validate_profile(vs, vc, ve, move_d, cons):
    """Raise InfeasibleProfile if a notched profile cannot fit the move.

    This follows the same recurrence as emit_profile() without collecting
    slices, so callers can validate a whole flush batch without retaining its
    rendered profiles in memory.
    """
    if move_d <= 0.0 or not cons.notch_freq:
        return
    if _emit_jerk_core(vs, vc, ve, move_d, cons, collect=False) is None:
        raise InfeasibleProfile(
            "notched profile does not fit move: "
            "start=%.6f cruise=%.6f end=%.6f distance=%.6f"
            % (vs, vc, ve, move_d)
        )


def _render_validated_profile(vs, vc, ve, move_d, cons):
    """Render a profile that has already passed validate_profile()."""
    if move_d <= 0.0:
        return []
    if cons.notch_freq:
        segs = _emit_jerk_core(vs, vc, ve, move_d, cons)
        if segs is None:
            raise AssertionError("validated notch profile failed to render")
        return segs
    return _emit_sharp(vs, vc, ve, move_d, cons)


def emit_profile(vs, vc, ve, move_d, cons):
    """Validate and emit one constant-accel segment list for a single move.

    Each segment is (accel_t, cruise_t, decel_t, start_v, cruise_v, accel, dist)
    -- the tuple the toolhead/extruder trapq consume. Invariant-guaranteed by
    construction: non-negative times/speeds, no decel past zero, per-segment
    trapq-implied distance == dist, inter-segment velocity continuity, and
    sum(dist) == move_d to within 1e-6*move_d (see _emit_jerk_core).

    When cons.notch_freq is set, acceleration is emitted as constant-accel
    slices whose per-slice changes are jerk bounded. An infeasible notched move
    raises InfeasibleProfile instead of silently substituting a sharp profile.

    A move with no room to ramp at cons.notch_freq simply holds its entry speed
    -- the zero stays parked on the mode. The lookahead applies the identical
    rule, so the cruise it asked for is the cruise this renders.
    """
    validate_profile(vs, vc, ve, move_d, cons)
    return _render_validated_profile(vs, vc, ve, move_d, cons)


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
        "second_notch_saturated",
        "spectral_null_partial",
        "spectral_null_failed",
        "spectral_null_span_turned",
    )
)


def notch_loss_reasons(vs, vc, ve, cons):
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
            if worst > SECOND_NOTCH_EPS and not (
                cons.spectral_null and cons.spectral_loss is None
            ):
                # Suppressed when the spectral solve succeeded. This reason is
                # derived from the ANALYTIC saturated profile, where a widened
                # plateau leaves only one zero placeable. That reasoning is
                # about the ideal shape; the solve works on the emitted slices
                # and can put both zeros back, so the measured spectrum
                # supersedes the prediction. spectral_loss being None means the
                # emitted profile was checked at every mode and nulled.
                reasons.add("second_notch_saturated")
    # What the spectral solve actually managed, recorded during emission by
    # _apply_spectral_null. Both of these lose the null the user asked for:
    # "failed" fell back to the sampled ramp entirely, "partial" got a
    # correction scaled back to respect max_accel or the jerk bound.
    if cons.spectral_loss:
        reasons.add(cons.spectral_loss)
    return sorted(reasons)
