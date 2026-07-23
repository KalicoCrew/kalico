# Invariant checks for pathplan.emit_profile (the "stepguard" invariant made
# structural) plus basic sharp/jerk profile shape.
#
# Run: klippy-env/bin/python klippy/extras/test_pathplan.py
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pathplan  # noqa: E402


def check_segs(segs, move_d, want_vs, want_ve, tag):
    # Non-negative times/speeds, velocity continuity across segments, exact
    # per-segment trapq-implied distance, and sum(dist) == move_d.
    eps_v = 1e-4
    total = 0.0
    prev_ev = None
    first_sv = None
    for k, (at, ct, dt, sv, cv, a, dist) in enumerate(segs):
        assert at >= -1e-9 and ct >= -1e-9 and dt >= -1e-9, (tag, "neg t", k)
        assert sv >= -eps_v and cv >= -eps_v, (tag, "neg v", k, sv, cv)
        ev = cv - a * dt
        assert ev >= -eps_v, (tag, "end vel<0", k, ev, cv, a, dt)
        d_impl = 0.5 * (sv + cv) * at + cv * ct + (cv * dt - 0.5 * a * dt * dt)
        assert abs(d_impl - dist) <= 1e-6 + 1e-4 * abs(dist), (
            tag, "dist", k, d_impl, dist)
        if prev_ev is not None:
            assert abs(sv - prev_ev) <= 1e-3, (tag, "discont", k, sv, prev_ev)
        else:
            first_sv = sv
        prev_ev = ev
        total += dist
    assert abs(total - move_d) <= 1e-3 + 1e-4 * move_d, (
        tag, "total", total, move_d)
    if segs:
        assert abs(first_sv - want_vs) <= 1e-3, (tag, "start", first_sv, want_vs)
        assert abs(prev_ev - want_ve) <= 1e-3, (tag, "end", prev_ev, want_ve)


def test_sharp_trapezoid():
    # No jerk, no notch -> classic accel/cruise/decel trapezoid.
    cons = pathplan.Constraints(a_const=8000.0, v_ceil=400.0)
    for vs, vc, ve, d in [(0.0, 200.0, 0.0, 60.0),
                          (0.0, 250.0, 120.0, 50.0),
                          (80.0, 300.0, 80.0, 90.0),
                          (0.0, 200.0, 0.0, 0.5)]:   # short -> triangle
        segs = pathplan.emit_profile(vs, vc, ve, d, cons)
        assert segs, ("empty", vs, vc, ve, d)
        check_segs(segs, d, vs, ve, "sharp(%s,%s,%s,%s)" % (vs, vc, ve, d))
    print("  sharp trapezoid invariants OK")


def test_jerk_profile_invariants():
    cons = pathplan.Constraints(a_const=8000.0, v_ceil=400.0,
                                max_jerk=1.0e5, jerk_dt=0.001)
    for vs, vc, ve, d in [(0.0, 200.0, 0.0, 60.0),
                          (0.0, 250.0, 120.0, 50.0),
                          (80.0, 300.0, 80.0, 90.0)]:
        segs = pathplan.emit_profile(vs, vc, ve, d, cons)
        assert segs, ("empty", vs, vc, ve, d)
        check_segs(segs, d, vs, ve, "jerk(%s,%s,%s,%s)" % (vs, vc, ve, d))
    print("  jerk profile invariants OK")


def test_disabled_matches_sharp():
    a = pathplan.emit_profile(0.0, 200.0, 50.0, 40.0,
                              pathplan.Constraints(a_const=8000.0,
                                                   v_ceil=400.0))
    b = pathplan.emit_profile(0.0, 200.0, 50.0, 40.0,
                              pathplan.Constraints(a_const=8000.0, v_ceil=400.0,
                                                   max_jerk=None))
    assert a == b, "max_jerk=None diverged from sharp"
    print("  max_jerk=None identical to sharp OK")


def main():
    test_sharp_trapezoid()
    test_jerk_profile_invariants()
    test_disabled_matches_sharp()
    print("ALL PASS")


if __name__ == "__main__":
    main()
