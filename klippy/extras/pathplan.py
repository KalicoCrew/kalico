# Jerk-limited motion planning with a per-ramp notch law
#
# A single phase-plane emitter that renders each move's acceleration as a
# continuous jerk-limited ramp instead of a hard accel step, plus a jerk-aware
# lookahead that plans exactly the boundary speeds that emitter can render (so
# no move is ever planned into the sharp constant-accel fallback).
#
# The jerk per ramp can either be a fixed value (max_jerk) or be derived from a
# NOTCH FREQUENCY: J = dv * f_n^2. A jerk-limited ramp whose acceleration does
# not saturate is triangular in a(t), which is an input shaper with a zero at
# 1/T_rise; the notch law pins T_rise = 1/f_n so that zero stays on a fixed
# structural mode regardless of the move's velocity change (a fixed jerk's zero
# slides with dv and cannot cancel a fixed mode). A side effect is that peak
# acceleration self-scales as a_peak = dv * f_n, so max_accel stops governing
# ordinary moves and only backstops moves too short to shape.
#
# Pure (only `math`) and toolhead-decoupled so it is unit-testable in isolation;
# the toolhead adapter supplies per-move constraints.
#
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
import math


class Constraints:
    """Per-move motion limits for the jerk-limited emitter and lookahead.

    a_const    -> constant max |acceleration| (mm/s^2). With the notch law this
                  only bounds moves too short to shape (the sharp fallback);
                  ordinary moves are governed by a_peak = dv * f_n.
    v_ceil     -> hard speed ceiling (mm/s); the reachability search stops here.
    max_jerk   -> max |da/dt| (mm/s^3). None/0 = no jerk limiting (sharp
                  constant-accel profile). With notch_freq set this is a CEILING
                  on the per-ramp jerk rather than the jerk itself.
    jerk_dt    -> integration time step (s) for the emitted ramp.
    notch_freq -> mode frequency (Hz) to park the ramp's shaper zero on.
                  None/0 = fixed-jerk behaviour (max_jerk used verbatim).
                  See ramp_jerk() for the law.
    """

    def __init__(self, a_const, v_ceil=1e9, max_jerk=None, jerk_dt=0.001,
                 notch_freq=None):
        self.a_const = a_const
        self.v_ceil = v_ceil
        self.max_jerk = max_jerk
        self.jerk_dt = jerk_dt
        self.notch_freq = notch_freq

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

        max_jerk, when set, is applied as a machine ceiling. Clamping J DOWN
        only lengthens T, i.e. moves the zero BELOW f_n (more shaping, less
        accel), so it is always safe.
        """
        dv = abs(dv)
        if not self.notch_freq or dv <= 1e-12:
            return self.max_jerk
        j = dv * self.notch_freq * self.notch_freq
        if self.max_jerk:
            j = min(j, self.max_jerk)
        return j if j > 0.0 else None


def jerk_dist(v0, v1, accel, jerk, notch_freq=None):
    # Path distance to change speed v0 -> v1 under a symmetric jerk-limited
    # S-curve at constant max |accel| and max |jerk| (accel ramps 0 -> peak -> 0
    # so a(t) is continuous). Closed form: distance = mean speed * duration,
    # exact because a symmetric velocity S-curve's time-average speed is
    # (v0+v1)/2. Serves accel and decel identically (uses |dv|). This is the
    # analytic twin of _ramp_up_jerk's integrated distance -- used by the
    # jerk-aware lookahead so it plans exactly the boundary speeds the jerk
    # emitter can render (no move gets planned into the sharp fallback).
    dv = abs(v1 - v0)
    if dv <= 1e-12:
        return 0.0
    if notch_freq:
        # Per-ramp jerk law, mirrored from Constraints.ramp_jerk so the
        # lookahead plans exactly the ramp the emitter renders. In the
        # (normal) non-saturating case this collapses to T = 1/f_n and
        # distance = (v0+v1)/f_n, independent of jerk.
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
    # a_peak is unsaturated and 0.5*(v0+v1)*(dv/A + A/(dv*f_n^2)) past that, and
    # both are monotonically increasing in v1 (the second branch only applies
    # for dv > A/f_n, where its derivative is positive).
    if dist <= 0.0:
        return u0
    if accel <= 0.0 or ((jerk is None or jerk <= 0.0) and not notch_freq):
        return u0 + 2.0 * accel * dist
    v0 = math.sqrt(max(u0, 0.0))
    hi = max(v0, v_ceil)
    if jerk_dist(v0, hi, accel, jerk, notch_freq) <= dist:
        return hi * hi
    lo = v0
    for _ in range(48):
        mid = 0.5 * (lo + hi)
        if jerk_dist(v0, mid, accel, jerk, notch_freq) <= dist:
            lo = mid
        else:
            hi = mid
    return lo * lo


def _ramp_up_jerk(v0, v1, cons, collect=True):
    # Jerk-limited acceleration ramp taking velocity v0 -> v1 (v1 >= v0). The
    # acceleration starts at ~0, rises toward a_max bounded by |da/dt| <= J,
    # then falls back to ~0 landing on v1 -- so a(t) is continuous (no hard
    # accel step). Integrated at fixed jerk_dt; the last slice lands exactly on
    # v1 (velocity continuity is exact). Returns (slices, total_d); slices is
    # None when collect=False (distance-only, for peak bisection).
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
    for (at, ct, dt, sv, cv, a, dist) in reversed(acc_slices):
        dec.append((0.0, 0.0, at, cv, cv, a, dist))
    return dec


def _peak_velocity_jerk(vs, ve, move_d, cons):
    # Highest cruise a jerk-limited move can reach: bisection on the ramp
    # distance (accel vs->vp plus decel vp->ve). Returns max(vs,ve) when even
    # that connecting ramp overfills move_d (caller then falls back to sharp).
    lo = max(vs, ve)
    hi = cons.v_ceil
    need_lo = (_ramp_up_jerk(vs, lo, cons, collect=False)[1]
               + _ramp_up_jerk(ve, lo, cons, collect=False)[1])
    if need_lo >= move_d:
        return lo
    for _ in range(32):
        mid = 0.5 * (lo + hi)
        need = (_ramp_up_jerk(vs, mid, cons, collect=False)[1]
                + _ramp_up_jerk(ve, mid, cons, collect=False)[1])
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
    # converge. Falling back to plain fixed-jerk is the right semantics -- "too
    # short to shape, so stop shaping and just stay accel-continuous".
    return Constraints(a_const=cons.a_const, v_ceil=cons.v_ceil,
                       max_jerk=J, jerk_dt=cons.jerk_dt, notch_freq=None)


def _emit_jerk_core(vs, vc, ve, move_d, cons):
    # One jerk-limited profile at the per-ramp jerk: ramp vs->vc, cruise, ramp
    # vc->ve. Returns None when the move can't be jerk-limited (endpoints
    # unreachable within move_d even at the connecting peak).
    vc = max(vc, vs, ve)
    acc, d_acc = _ramp_up_jerk(vs, vc, cons)
    dec_acc, d_dec = _ramp_up_jerk(ve, vc, cons)
    if acc is None or dec_acc is None:      # ramp did not converge -> reject
        return None
    cruise_d = move_d - d_acc - d_dec
    if cruise_d < -1e-9:
        vc = max(_peak_velocity_jerk(vs, ve, move_d, cons), vs, ve)
        acc, d_acc = _ramp_up_jerk(vs, vc, cons)
        dec_acc, d_dec = _ramp_up_jerk(ve, vc, cons)
        if acc is None or dec_acc is None:
            return None
        cruise_d = move_d - d_acc - d_dec
        if cruise_d < -1e-6 * max(1.0, move_d):
            return None
        cruise_d = max(0.0, cruise_d)
    segs = list(acc)
    if cruise_d > 1e-9 and vc > 1e-9:
        segs.append((0.0, cruise_d / vc, 0.0, vc, vc, 0.0, cruise_d))
    segs.extend(_decel_from_accel(dec_acc))
    return segs


def _emit_jerk(vs, vc, ve, move_d, cons):
    # Jerk-limited profile that NEVER emits a hard accel step. If the move is
    # infeasible at the configured jerk, raise the jerk to the minimum value
    # that fits and emit there -- acceleration stays continuous (bounded
    # da = J'*dt), just ramps faster. A hard step would become a multi-step
    # position jump in stepcompress on any downstream that reads accel.
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
        return None                     # nothing to ramp -> caller does sharp
    hi = J0
    feasible_hi = None
    for _ in range(40):                 # expand until feasible
        hi *= 2.0
        s = _emit_jerk_core(vs, vc, ve, move_d, _with_jerk(cons, hi))
        if s is not None:
            feasible_hi = s
            break
    if feasible_hi is None:
        return None                     # truly degenerate -> caller does sharp
    lo = hi * 0.5                        # last infeasible jerk
    best = feasible_hi
    for _ in range(24):                 # bisection for the minimum feasible J'
        mid = 0.5 * (lo + hi)
        s = _emit_jerk_core(vs, vc, ve, move_d, _with_jerk(cons, mid))
        if s is not None:
            hi, best = mid, s
        else:
            lo = mid
    return best


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
        vc = max(math.sqrt(max(vp2, 0.0)), vs, ve)
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
    sum(dist) == move_d.

    When cons.max_jerk (or cons.notch_freq) is set, the acceleration is ramped
    so a(t) is continuous; on a move too short to jerk-limit, it falls back to
    the sharp constant-accel profile.
    """
    if move_d <= 0.0:
        return []
    if cons.max_jerk or cons.notch_freq:
        segs = _emit_jerk(vs, vc, ve, move_d, cons)
        if segs is not None:
            return segs
        # else: jerk-infeasible for this short move -> sharp fallback below
    return _emit_sharp(vs, vc, ve, move_d, cons)
