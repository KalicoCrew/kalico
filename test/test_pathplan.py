# Invariant checks for pathplan.emit_profile (the "stepguard" invariant made
# structural) plus basic sharp/jerk profile shape.
#
# Run: klippy-env/bin/python test/test_pathplan.py
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "klippy", "extras"))
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
            tag,
            "dist",
            k,
            d_impl,
            dist,
        )
        if prev_ev is not None:
            assert abs(sv - prev_ev) <= 1e-3, (tag, "discont", k, sv, prev_ev)
        else:
            first_sv = sv
        prev_ev = ev
        total += dist
    assert abs(total - move_d) <= 1e-3 + 1e-4 * move_d, (
        tag,
        "total",
        total,
        move_d,
    )
    if segs:
        assert abs(first_sv - want_vs) <= 1e-3, (
            tag,
            "start",
            first_sv,
            want_vs,
        )
        assert abs(prev_ev - want_ve) <= 1e-3, (tag, "end", prev_ev, want_ve)


def test_sharp_trapezoid():
    # No jerk, no notch -> classic accel/cruise/decel trapezoid.
    cons = pathplan.Constraints(a_const=8000.0, v_ceil=400.0)
    for vs, vc, ve, d in [
        (0.0, 200.0, 0.0, 60.0),
        (0.0, 250.0, 120.0, 50.0),
        (80.0, 300.0, 80.0, 90.0),
        (0.0, 200.0, 0.0, 0.5),
    ]:  # short -> triangle
        segs = pathplan.emit_profile(vs, vc, ve, d, cons)
        assert segs, ("empty", vs, vc, ve, d)
        check_segs(segs, d, vs, ve, "sharp(%s,%s,%s,%s)" % (vs, vc, ve, d))
    print("  sharp trapezoid invariants OK")


def test_impossible_endpoint_change_rejected():
    cons = pathplan.Constraints(a_const=8000.0, v_ceil=400.0)
    cases = [(100.0, 200.0, 0.0, 0.5), (50.0, 200.0, 100.0, 0.2)]
    for c in cases:
        assert pathplan.emit_profile(*c, cons) == [], (
            "impossible endpoint change emitted",
            c,
        )
    print("  impossible endpoint changes rejected OK")


def main():
    test_sharp_trapezoid()
    test_impossible_endpoint_change_rejected()
    print("ALL PASS")


if __name__ == "__main__":
    main()
