# Tests for the TWO-zero notch ramp (J = dv*f_lo*f_hi, a_peak = dv*f_lo).
#
# One move has one scalar a(t), and every axis sees a_i(t) = r_i*a(t) -- one
# pulse, scaled. That is often read as "one move can only null one frequency",
# and it is why the earlier per-axis notch placed a single zero at a
# direction-weighted mean of f_x and f_y, landing it on NEITHER mode on a
# diagonal. It is not true. Spectral zeros MULTIPLY under convolution, and the
# single-zero triangle is already rect(1/f_n) * rect(1/f_n); widening one rect
# gives a TRAPEZOID
#
#     a(t) = rect(1/f_hi) * rect(1/f_lo)
#     |A(f)| = |sinc(f/f_hi) * sinc(f/f_lo)|   -> exact zeros at BOTH modes
#
# which every axis inherits whatever its heading. Matching that to the emitter's
# constant-jerk integrator needs J = dv*f_lo*f_hi with the plateau capped at
# dv*f_lo, giving T_rise = 1/f_hi and T_total = 1/f_lo + 1/f_hi.
#
# Verifies: both zeros actually appear in emitted profiles and beat the blend at
# both modes; f_lo == f_hi degenerates to the single-zero triangle bit-for-bit;
# the duration/peak/distance laws hold and are independent of dv; the lookahead
# twins (jerk_dist, jerk_reach_v2) agree with the integrator; the emitter's
# distance invariant survives; accel saturation, which leaves room for only one
# zero, spends it on whichever mode it helps more and reports the loss only when
# a mode is really left excited; and the pair's harmonic mean f_eq, which every
# distance formula is written in terms of, matches the measured ramp.
#
# Run: klippy-env/bin/python -m pytest test/test_notch_dual.py
import math
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "klippy", "extras"))
import pathplan  # noqa: E402
from test_pathplan import check_segs  # noqa: E402

F_LO = 55.0  # the two modes to null (red's real X and Y, roughly)
F_HI = 75.0
F_BLEND = 0.5 * (F_LO + F_HI)  # what the old single-zero law used at 45 deg
A_BIG = 1e9  # high enough that dv*f_lo never saturates
DT = 25e-6


def cons(
    f_lo=F_LO,
    f_hi=F_HI,
    a_const=A_BIG,
    max_jerk=None,
    jerk_dt=DT,
):
    return pathplan.Constraints(
        a_const=a_const,
        v_ceil=1e9,
        max_jerk=max_jerk,
        jerk_dt=jerk_dt,
        notch_freq=f_lo,
        notch_freq2=f_hi,
    )


def ramp(v0, v1, c):
    """(pulse, distance) for one accel ramp; pulse is [(dt, accel), ...]."""
    slices, dist = pathplan._ramp_up_jerk(v0, v1, c)
    return [(s[0], s[5]) for s in slices], dist


def profile_pulse(segs):
    """Flatten an emitted profile into a signed piecewise-constant a(t)."""
    out = []
    for accel_t, cruise_t, decel_t, _sv, _cv, a, _d in segs:
        if accel_t > 0.0:
            out.append((accel_t, a))
        elif cruise_t > 0.0:
            out.append((cruise_t, 0.0))
        elif decel_t > 0.0:
            out.append((decel_t, -a))
    return out


def residual(pulse, freq):
    """|A(f)| / |a|_1 -- what fraction of the pulse lands on the mode.

    Normalised by the L1 norm so pulses of different size/duration compare
    directly: 0 is a perfect null, and an unshaped step approaches 1.
    """
    t = re = im = l1 = 0.0
    w = 2.0 * math.pi * freq
    for dt, a in pulse:
        re += a * (math.sin(w * (t + dt)) - math.sin(w * t)) / w
        im += a * (math.cos(w * (t + dt)) - math.cos(w * t)) / w
        l1 += abs(a) * dt
        t += dt
    return math.hypot(re, im) / l1 if l1 else 0.0


def test_one_ramp_nulls_both_modes():
    # The headline claim, straight off the integrator: a single accel ramp
    # leaves essentially nothing at EITHER mode, while single-zero ramps at
    # f_lo, f_hi or their mean each leave one mode ringing.
    for dv in (20.0, 60.0, 200.0):
        dual, _ = ramp(0.0, dv, cons())
        r_lo, r_hi = residual(dual, F_LO), residual(dual, F_HI)
        assert r_lo < 1e-3, (dv, r_lo)
        assert r_hi < 1e-3, (dv, r_hi)
        for f_single in (F_LO, F_HI, F_BLEND):
            single, _ = ramp(0.0, dv, cons(f_single, f_single))
            worst_single = max(residual(single, F_LO), residual(single, F_HI))
            # The single-zero ramp always leaves at least one mode excited by
            # more than the discretization floor.
            assert worst_single > 1e-2, (dv, f_single, worst_single)
            assert max(r_lo, r_hi) < worst_single
        print(
            "  dv=%6.1f  dual leaves %.5f / %.5f at (%g, %g) Hz;"
            " blend leaves %.5f / %.5f"
            % (
                dv,
                r_lo,
                r_hi,
                F_LO,
                F_HI,
                residual(ramp(0.0, dv, cons(F_BLEND, F_BLEND))[0], F_LO),
                residual(ramp(0.0, dv, cons(F_BLEND, F_BLEND))[0], F_HI),
            )
        )


def test_equal_pair_is_the_single_zero_triangle():
    # No special-casing anywhere: f_lo == f_hi has to collapse the trapezoid
    # back onto the existing triangle EXACTLY, whether the second frequency is
    # passed as None or as the same number. If this drifts, every single-notch
    # machine silently changes behaviour.
    for dv in (20.0, 200.0):
        for a_const in (A_BIG, 5000.0):
            single, d_single = ramp(0.0, dv, cons(F_LO, None, a_const))
            twin, d_twin = ramp(0.0, dv, cons(F_LO, F_LO, a_const))
            assert single == twin, (dv, a_const)
            assert d_single == d_twin
            # ... and it is still a triangle parked on f_lo.
            assert residual(single, F_LO) < 1e-3
    print("  f_lo == f_hi reproduces the single-zero ramp bit-for-bit OK")


def test_duration_peak_and_distance_laws():
    # T_total = 1/f_lo + 1/f_hi, INDEPENDENT of dv (that is what parks the
    # zeros); a_peak = dv*f_lo; distance = 0.5*(v0+v1)*T_total.
    want_t = 1.0 / F_LO + 1.0 / F_HI
    for v0, v1 in ((0.0, 50.0), (0.0, 200.0), (40.0, 300.0)):
        pulse, dist = ramp(v0, v1, cons())
        dv = v1 - v0
        t_total = sum(dt for dt, _a in pulse)
        a_peak = max(a for _dt, a in pulse)
        assert abs(t_total - want_t) / want_t < 0.01, (v0, v1, t_total)
        assert abs(a_peak - dv * F_LO) / (dv * F_LO) < 0.01, (v0, v1, a_peak)
        want_d = 0.5 * (v0 + v1) * want_t
        assert abs(dist - want_d) / want_d < 0.01, (v0, v1, dist, want_d)
        # The plateau is real, not a numerical accident: a triangle would peak
        # at dv*f_eq, well above dv*f_lo.
        f_eq = 2.0 / want_t
        assert a_peak < 0.98 * dv * f_eq
    print(
        "  T=%.2f ms (dv-independent), a_peak=dv*%g, dist=(v0+v1)/%.3f OK"
        % (want_t * 1e3, F_LO, 2.0 / want_t)
    )


def test_lookahead_twins_agree_with_the_integrator():
    # jerk_dist is the closed form the LOOKAHEAD plans with; the integrator is
    # what actually renders. If they disagree the planner approves a cruise the
    # emitter cannot fit and the move drops to the sharp constant-accel
    # fallback -- losing the notch silently.
    for a_const in (A_BIG, 20000.0, 5000.0):
        for v0, v1 in ((0.0, 60.0), (0.0, 200.0), (30.0, 250.0)):
            _pulse, d_int = ramp(v0, v1, cons(a_const=a_const))
            d_cf = pathplan.jerk_dist(v0, v1, a_const, None, F_LO, F_HI)
            # 0.6% is the zero-order-hold discretization, the same error the
            # single-notch law carries; it is not a modelling difference.
            assert abs(d_int - d_cf) / d_cf < 0.01, (
                a_const,
                v0,
                v1,
                d_int,
                d_cf,
            )

    # And reach must be the exact inverse of distance.
    for dist in (0.5, 3.13, 5.0, 20.0):
        u = pathplan.jerk_reach_v2(0.0, dist, A_BIG, None, 1e9, F_LO, F_HI)
        v = math.sqrt(u)
        need = pathplan.jerk_dist(0.0, v, A_BIG, None, F_LO, F_HI)
        assert need <= dist + 1e-6, (dist, v, need)
        assert need > dist - 1e-3, (dist, v, need)  # and not leaving room
    print("  jerk_dist / jerk_reach_v2 agree with the integrated ramp OK")


def test_reach_is_monotone_in_distance():
    prev = 0.0
    for dist in [0.1 * i for i in range(1, 200)]:
        u = pathplan.jerk_reach_v2(4.0, dist, A_BIG, None, 1e9, F_LO, F_HI)
        assert u >= prev - 1e-9, (dist, u, prev)
        prev = u
    print("  reachable v^2 is non-decreasing in distance OK")


def test_emitted_profiles_keep_both_zeros_and_the_invariants():
    # Through the real entry point, on whole moves rather than bare ramps.
    # emit_profile runs two nested bisections, each integrating trial ramps,
    # so it uses a coarser slice than the bare-ramp spectral tests. That
    # raises the zero-order-hold floor (hence 5e-3, not 1e-3) without
    # changing what is being checked.
    coarse = 2e-4
    for a_const in (A_BIG, 20000.0):
        c = cons(a_const=a_const, jerk_dt=coarse)
        d = (
            2.0 * pathplan.jerk_dist(0.0, 200.0, a_const, None, F_LO, F_HI)
            + 5.0
        )
        segs = pathplan.emit_profile(0.0, 200.0, 0.0, d, c)
        check_segs(segs, d, 0.0, 0.0, "dual a=%g" % a_const)
        pulse = profile_pulse(segs)
        assert residual(pulse, F_LO) < 5e-3, residual(pulse, F_LO)
        assert residual(pulse, F_HI) < 5e-3, residual(pulse, F_HI)
        assert pathplan.notch_loss_reasons(0.0, 200.0, 0.0, d, c) == []
    # Distance conservation across a range of move lengths, including ones too
    # short to ramp (sharp fallback).
    for a_const in (A_BIG, 5000.0):
        c = cons(a_const=a_const, jerk_dt=coarse)
        for d in (0.5, 3.0, 12.0, 60.0):
            segs = pathplan.emit_profile(10.0, 200.0, 30.0, d, c)
            check_segs(segs, d, 10.0, 30.0, "dual d=%g" % d)
            assert abs(sum(s[6] for s in segs) - d) <= 1e-6 * d
    print("  emitted profiles null both modes and conserve distance OK")


def sat_worst(dv, a_const, f_rise):
    """Worst residual across both modes for a forced rise frequency."""
    return max(
        abs(pathplan._sinc(m / f_rise) * pathplan._sinc(m * dv / a_const))
        for m in (F_LO, F_HI)
    )


def test_saturation_spends_its_one_zero_where_it_helps_most():
    # Keeping the plateau exactly 1/f_lo wide needs a_peak = dv*f_lo. When
    # max_accel is lower, the plateau widens to dv/A and only the RISE edge is
    # still ours to shape -- recovering both zeros would need a third-order
    # (S-edge) pulse this constant-jerk integrator cannot render.
    #
    # Which mode to spend that one zero on is NOT a constant: it depends on
    # where the forced A/dv zero and its harmonics fall, which moves with dv.
    # A fixed policy is arbitrarily bad, so the law evaluates both.
    for dv, a_const, want in ((150.0, 8000.0, F_HI), (120.0, 5000.0, F_LO)):
        assert dv * F_LO > a_const, (dv, a_const)
        got = pathplan.sat_rise_freq(dv, a_const, F_LO, F_HI)
        assert got == want, (dv, a_const, got, want)
        # ... and it really is the better of the two, by a margin worth having.
        other = F_HI if want == F_LO else F_LO
        assert sat_worst(dv, a_const, got) < sat_worst(dv, a_const, other)
    # The flip case is the point: a fixed f_lo policy would be 4x worse here.
    assert sat_worst(150.0, 8000.0, F_LO) > 4.0 * sat_worst(150.0, 8000.0, F_HI)

    # The chosen zero is real in the emitted ramp, and the sacrificed mode is
    # bounded rather than unshaped.
    dv, a_const = 120.0, 5000.0
    c = cons(a_const=a_const)
    pulse, _d = ramp(0.0, dv, c)
    assert residual(pulse, F_LO) < 1e-3, "the chosen zero must survive"
    assert 1e-2 < residual(pulse, F_HI) < 0.05, residual(pulse, F_HI)

    # And the loss is reported, not silent.
    d = 2.0 * pathplan.jerk_dist(0.0, dv, a_const, None, F_LO, F_HI) + 5.0
    reasons = pathplan.notch_loss_reasons(0.0, dv, 0.0, d, c)
    assert "second_notch_saturated" in reasons, reasons
    assert "second_notch_saturated" in pathplan.LOSS_REASONS
    print("  saturated ramps pick the better zero and report the loss OK")


def test_saturation_is_silent_when_it_costs_nothing():
    # Saturation does not always cost a zero: when a harmonic of the forced
    # A/dv zero lands on the other mode, both survive anyway. Warning there
    # would be noise, so the report is keyed on the residual, not on the
    # structural condition.
    dv, a_const = 200.0, 5000.0
    assert dv * F_LO > a_const, "still a saturating case"
    assert abs(a_const / dv - 25.0) < 1e-9 and abs(F_HI / 25.0 - 3.0) < 1e-9
    c = cons(a_const=a_const)
    pulse, _d = ramp(0.0, dv, c)
    assert residual(pulse, F_LO) < 1e-3, residual(pulse, F_LO)
    assert residual(pulse, F_HI) < 1e-3, residual(pulse, F_HI)
    d = 2.0 * pathplan.jerk_dist(0.0, dv, a_const, None, F_LO, F_HI) + 5.0
    assert pathplan.notch_loss_reasons(0.0, dv, 0.0, d, c) == []

    # A dv small enough not to saturate at all is silent for the plain reason.
    small = 40.0
    assert small * F_LO < a_const
    d_small = (
        2.0 * pathplan.jerk_dist(0.0, small, a_const, None, F_LO, F_HI) + 5.0
    )
    assert pathplan.notch_loss_reasons(0.0, small, 0.0, d_small, c) == []
    print("  saturation that costs no zero is not reported OK")


def test_f_eq_is_the_harmonic_mean_of_the_pair():
    # Every distance in the planner is written (v0+v1)/f_eq and every duration
    # as notch_period, which is what makes the two-zero formulas textually
    # identical to the single-zero ones. Pin both against a measured ramp.
    for f_lo, f_hi in ((55.0, 75.0), (30.0, 31.0), (40.0, 160.0), (55.0, 55.0)):
        c = pathplan.Constraints(
            a_const=A_BIG, jerk_dt=DT, notch_freq=f_lo, notch_freq2=f_hi
        )
        assert abs(c.notch_period - (1.0 / f_lo + 1.0 / f_hi)) < 1e-12
        assert abs(c.notch_f_eq - 2.0 / (1.0 / f_lo + 1.0 / f_hi)) < 1e-12
        v0, v1 = 40.0, 140.0
        _pulse, dist = ramp(v0, v1, c)
        # 1% is the zero-order-hold discretization of the integrated ramp, the
        # same slack the jerk_dist agreement check above carries.
        assert abs(dist - (v0 + v1) / c.notch_f_eq) < 1e-2 * dist, (
            f_lo,
            f_hi,
            dist,
        )
    print("  notch_period / f_eq match the measured ramp OK")


def test_notch_unset_is_untouched():
    # Passing a second frequency with no first one must not switch anything on.
    c = pathplan.Constraints(
        a_const=A_BIG,
        v_ceil=1e9,
        max_jerk=None,
        jerk_dt=DT,
        notch_freq=None,
        notch_freq2=F_HI,
    )
    assert c.notch_f_eq == 0.0
    assert c.ramp_accel_cap(100.0) is None
    assert c.ramp_jerk(100.0) is None
    print("  notch_freq2 without notch_freq is inert OK")


if __name__ == "__main__":
    for name, fn in sorted(list(globals().items())):
        if name.startswith("test_"):
            print("==", name)
            fn()
    print("all dual-notch checks passed")
