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
    """

    def __init__(
        self,
        a_const,
        v_ceil=1e9,
        max_jerk=None,
        jerk_dt=0.001,
        notch_freq=None,
        max_da=None,
    ):
        self.a_const = a_const
        self.v_ceil = v_ceil
        self.max_jerk = max_jerk
        self.jerk_dt = jerk_dt
        self.notch_freq = notch_freq
        self.max_da = max_da

    def a_max(self, v):
        return self.a_const

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
        """
        dv = abs(dv)
        if not self.notch_freq or dv <= 1e-12:
            return self.max_jerk
        a = self.a_const
        if a and a > 0.0 and dv > a / self.notch_freq:
            j = a * self.notch_freq
        else:
            j = dv * self.notch_freq * self.notch_freq
        return j if j > 0.0 else None


def jerk_dist(v0, v1, accel, jerk, notch_freq=None):
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
        # Per-ramp jerk law, mirrored from Constraints.ramp_jerk. In the ideal
        # non-saturating case this collapses to total ramp time 2/f_n and
        # distance = (v0+v1)/f_n. Past accel saturation, use the designed
        # trapezoid J=A*f_n so the ramp edge, not the total duration, keeps the
        # zero at f_n.
        if accel > 0.0 and dv > accel / notch_freq:
            j = accel * notch_freq
        else:
            j = dv * notch_freq * notch_freq
        jerk = min(j, jerk) if jerk else j
    if jerk is None or jerk <= 0.0 or accel <= 0.0:
        return abs(v1 * v1 - v0 * v0) / (2.0 * accel)
    if dv <= accel * accel / jerk:
        # Accel never saturates (triangular a(t)): T = 2*sqrt(dv/J).
        t = 2.0 * math.sqrt(dv / jerk)
    else:
        # Accel saturates at `accel` (trapezoidal a(t)): T = dv/A + A/J.
        t = dv / accel + accel / jerk
    return 0.5 * (v0 + v1) * t


def jerk_reach_v2(u0, dist, accel, jerk, v_ceil, notch_freq=None):
    # Max u = v^2 reachable from v0=sqrt(u0) over path-distance `dist` under a
    # jerk-limited S-curve (constant max accel/jerk). Direction-symmetric
    # (forward accel == backward decel). Bisection on v1 over the O(1)
    # closed-form jerk_dist -- cheap enough for the hot lookahead loop (no ramp
    # integration). jerk None/0 (and no notch_freq) -> stock constant-accel
    # reach.
    #
    # Bisection stays valid under the notch law: distance is (v0+v1)/f_n while
    # a_peak is unsaturated and 0.5*(v0+v1)*(dv/A + 1/f_n) past that; both are
    # monotonically increasing in v1.
    if dist <= 0.0:
        return u0
    if accel <= 0.0 or ((jerk is None or jerk <= 0.0) and not notch_freq):
        return u0 + 2.0 * accel * dist
    v0 = math.sqrt(max(u0, 0.0))
    if notch_freq:
        # Under the notch law EVERY ramp costs the same 2/f_n of TIME no matter
        # how small dv is, so the ramp distance (v0+v1)/f_n does not go to zero
        # as v1 -> v0: its infimum is 2*v0/f_n. A move shorter than that has no
        # room for any speed change at all and reach is exactly u0. Testing that
        # closed form up front is the common short-move case on sliced geometry
        # and skips the whole bisection. (The saturated branch only costs MORE
        # distance, so this stays a valid lower bound there too.)
        if dist < 2.0 * v0 / notch_freq:
            return u0
        if jerk:
            tri_dv_max = jerk / (notch_freq * notch_freq)
            sat_j = accel * notch_freq if accel > 0.0 else 0.0
            if jerk < sat_j:
                v_ceil = min(v_ceil, v0 + tri_dv_max)
    hi = max(v0, v_ceil)
    if jerk_dist(v0, hi, accel, jerk, notch_freq) <= dist:
        return hi * hi
    lo = v0
    # Converge to a velocity tolerance instead of burning a fixed iteration
    # count: this runs on every move in the lookahead hot loop, and 1e-4 mm/s
    # is far below anything the step generator can resolve.
    for _ in range(48):
        if hi - lo <= 1e-4:
            break
        mid = 0.5 * (lo + hi)
        if jerk_dist(v0, mid, accel, jerk, notch_freq) <= dist:
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
        sat_j = (
            cons.a_const * cons.notch_freq
            if cons.a_const and cons.a_const > 0.0
            else 0.0
        )
        if not sat_j or cons.max_jerk < sat_j:
            dv_max = cons.max_jerk / (cons.notch_freq * cons.notch_freq)
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
    """
    if move_d <= 0.0:
        return []
    if cons.max_jerk or cons.notch_freq:
        segs = _emit_jerk(vs, vc, ve, move_d, cons)
        if segs is not None:
            return segs
        # else: jerk-infeasible for this short move -> sharp fallback below
    return _emit_sharp(vs, vc, ve, move_d, cons)


# Every reason notch_loss_reasons() can ever return. The toolhead uses this to
# stop calling the diagnostic once it has reported all of them.
LOSS_REASONS = frozenset(("jerk_clamped", "insufficient_runway"))


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
    for dv in (abs(vc - vs), abs(vc - ve)):
        if dv <= 1e-12:
            continue
        target_j = dv * cons.notch_freq * cons.notch_freq
        if cons.max_jerk:
            sat_j = (
                cons.a_const * cons.notch_freq
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
