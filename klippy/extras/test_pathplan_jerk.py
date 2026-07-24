# Tests for jerk limiting inside pathplan.emit_profile: the stepguard invariant
# holds; acceleration is continuous (bounded |da/dt| on the controllable side,
# tapering to ~0 at the phase boundaries); distance is conserved and endpoints
# are hit; short moves fall back to sharp; disabled == sharp.
#
# Run: klippy-env/bin/python klippy/extras/test_pathplan_jerk.py
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pathplan  # noqa: E402
from test_pathplan import check_segs  # noqa: E402


def max_abs_step(segs):
    # Largest |a_i - a_{i-1}| across ALL consecutive slices, including the
    # accel<->cruise<->decel seams, plus the final drop to rest. Bounded by the
    # discrete taper floor 2*J*dt0.
    worst = 0.0
    prev_a = 0.0
    for (at, ct, dt, sv, cv, a, dist) in segs:
        worst = max(worst, abs(a - prev_a))
        prev_a = a
    return max(worst, prev_a)


def make_cons(jerk=1.0e5, jerk_dt=0.001, a_const=8000.0):
    return pathplan.Constraints(a_const=a_const, v_ceil=400.0,
                                max_jerk=jerk, jerk_dt=jerk_dt)


def test_invariant_and_taper():
    cons = make_cons()
    J, dtj = cons.max_jerk, cons.jerk_dt
    cases = [(0.0, 200.0, 0.0, 60.0),
             (0.0, 250.0, 120.0, 50.0),
             (80.0, 300.0, 80.0, 90.0),
             (150.0, 150.0, 150.0, 20.0)]
    for vs, vc, ve, d in cases:
        segs = pathplan.emit_profile(vs, vc, ve, d, cons)
        assert segs, ("empty", vs, vc, ve, d)
        check_segs(segs, d, vs, ve, "jerk(%s,%s,%s,%s)" % (vs, vc, ve, d))
        step = max_abs_step(segs)
        assert step <= 2.1 * J * dtj + 1e-6, ("accel step exceeded", step,
                                              2.1 * J * dtj)
    print("  invariant + continuous-accel taper OK")


def test_jerk_widens_ramp_vs_sharp():
    # A jerk-limited accel ramp takes MORE distance than a hard constant-accel
    # ramp for the same dv (the S-curve's average accel is lower).
    cons = make_cons()
    _, d_jerk = pathplan._ramp_up_jerk(0.0, 200.0, cons)
    d_sharp = (200.0 ** 2) / (2.0 * cons.a_const)
    assert d_jerk > d_sharp, (d_jerk, d_sharp)
    print("  jerk ramp wider than sharp OK")


def test_short_move_falls_back_to_sharp():
    cons = make_cons()
    segs = pathplan.emit_profile(0.0, 200.0, 0.0, 0.5, cons)
    assert segs, "fallback returned empty"
    check_segs(segs, 0.5, 0.0, 0.0, "short")
    peak = max(s[5] for s in segs)
    # The sharp fallback is bounded by a_const, not blown up.
    assert peak <= cons.a_const + 1e-6, ("peak not clamped on fallback", peak)
    print("  short move falls back to sharp OK")


def test_disabled_matches_sharp():
    a = pathplan.emit_profile(0.0, 200.0, 50.0, 30.0,
                              pathplan.Constraints(a_const=8000.0,
                                                   v_ceil=400.0))
    b = pathplan.emit_profile(0.0, 200.0, 50.0, 30.0, make_cons(jerk=None))
    assert a == b, "max_jerk=None diverged from sharp"
    print("  disabled == sharp OK")


def main():
    test_invariant_and_taper()
    test_jerk_widens_ramp_vs_sharp()
    test_short_move_falls_back_to_sharp()
    test_disabled_matches_sharp()
    print("ALL PASS")


if __name__ == "__main__":
    main()
