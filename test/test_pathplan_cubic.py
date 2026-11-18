# Tests for the cubic (constant-jerk) emitter.
#
# The notch law designs a(t) as a piecewise-LINEAR pulse. The cubic emitter
# sends that pulse itself, as at most three trapq_append_jerk segments per
# ramp. These tests pin down that the rendered
# profile is the designed one: exact zeros, the planning model's own duration
# and distance, continuity, and correct splitting across a spanned run.
#
# Run: klippy-env/bin/python test/test_pathplan_cubic.py
import cmath
import math
import os
import random
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "klippy", "extras"))
import pathplan  # noqa: E402

ACCEL = 20000.0


def cons_for(f1=55.0, f2=None, a_const=ACCEL, v_ceil=1e9):
    return pathplan.Constraints(
        a_const=a_const,
        v_ceil=v_ceil,
        notch_freq=f1,
        notch_freq2=f2,
    )


def seg_spectrum(segs, freq):
    """A(freq) of the piecewise-linear acceleration the segments encode."""
    w = 2.0 * math.pi * freq
    t0 = 0.0
    acc = 0.0
    for t, _sv, a0, j, _d in segs:
        e = cmath.exp(-1j * w * t)
        i0 = (1.0 - e) / (1j * w)
        i1 = (-t * e + i0) / (1j * w)
        acc += cmath.exp(-1j * w * t0) * (a0 * i0 + j * i1)
        t0 += t
    return acc


def residual(segs, freq):
    dv = sum(a0 * t + 0.5 * j * t * t for t, _sv, a0, j, _d in segs)
    return abs(seg_spectrum(segs, freq)) / abs(dv)


def seg_end_v(seg):
    t, sv, a0, j, _d = seg
    return sv + a0 * t + 0.5 * j * t * t


def seg_dist(seg):
    t, sv, a0, j, _d = seg
    return ((j / 6.0 * t + 0.5 * a0) * t + sv) * t


def check_invariants(segs, vs, ve, move_d):
    assert segs[0][1] == vs or abs(segs[0][1] - vs) < 1e-9
    for a, b in zip(segs, segs[1:]):
        assert abs(seg_end_v(a) - b[1]) < 1e-7, (a, b)
    assert abs(seg_end_v(segs[-1]) - ve) < 1e-7
    for s in segs:
        assert s[0] >= 0.0
        assert abs(seg_dist(s) - s[4]) < 1e-9 * max(1.0, s[4]), s
        # Speed never goes negative inside a segment (v is convex/concave
        # in t, so checking the ends and the stationary point suffices).
        t, sv, a0, j, _d = s
        if j:
            ts = -a0 / j
            if 0.0 < ts < t:
                assert sv + a0 * ts + 0.5 * j * ts * ts >= -1e-9
    total = math.fsum(s[4] for s in segs)
    assert abs(total - move_d) <= 1e-6 * max(1.0, move_d), (total, move_d)


def test_ramp_nulls_exactly():
    # Unsaturated single and two-zero ramps: the zeros are exact up to float
    # rounding, at ANY speed change.
    for f1, f2 in ((55.0, None), (55.0, 65.0), (48.0, 56.0)):
        cons = cons_for(f1, f2)
        for v0, v1 in ((0.0, 20.0), (0.0, 100.0), (5.0, 10.0), (100.0, 300.0)):
            segs = pathplan._cubic_ramp_up(v0, v1, cons)
            assert len(segs) <= 3
            for f in {f1, f2 or f1}:
                r = residual(segs, f)
                assert r < 1e-12, (f1, f2, v0, v1, f, r)


def test_ramp_matches_planning_model():
    # The lookahead plans with notch_dist; the cubic render IS that model.
    for f1, f2 in ((55.0, None), (55.0, 65.0)):
        cons = cons_for(f1, f2, a_const=5000.0)
        for v0, v1 in ((0.0, 20.0), (0.0, 400.0), (50.0, 60.0)):
            segs = pathplan._cubic_ramp_up(v0, v1, cons)
            d = math.fsum(s[4] for s in segs)
            ref = pathplan.notch_dist(v0, v1, 5000.0, f1, f2)
            assert abs(d - ref) < 1e-9 * max(1.0, ref), (v0, v1, d, ref)


def test_saturated_keeps_rise_zero():
    # Past saturation only the rise-edge zero is placeable (documented loss).
    cons = cons_for(55.0, 65.0, a_const=3000.0)
    segs = pathplan._cubic_ramp_up(0.0, 400.0, cons)
    f_rise = pathplan.sat_rise_freq(400.0, 3000.0, 55.0, 65.0)
    assert residual(segs, f_rise) < 1e-12
    peak = max(abs(s[2]) for s in segs)
    assert peak <= 3000.0 * (1 + 1e-12)


def test_decel_is_time_reverse():
    cons = cons_for(55.0, 65.0)
    up = pathplan._cubic_ramp_up(10.0, 200.0, cons)
    dn = pathplan._cubic_reverse(up)
    check_invariants(dn, 200.0, 10.0, math.fsum(s[4] for s in up))
    assert residual(dn, 55.0) < 1e-12 and residual(dn, 65.0) < 1e-12


def test_random_profiles():
    rng = random.Random(1234)
    for _ in range(3000):
        f1 = rng.uniform(30.0, 90.0)
        f2 = rng.choice([None, f1 * rng.uniform(1.0, 1.5)])
        a = rng.uniform(2000.0, 40000.0)
        cons = cons_for(f1, f2, a_const=a, v_ceil=rng.uniform(50.0, 700.0))
        vs = rng.uniform(0.0, cons.v_ceil * 0.5)
        ve = rng.uniform(0.0, cons.v_ceil * 0.5)
        vc = rng.uniform(max(vs, ve), cons.v_ceil)
        move_d = rng.uniform(0.01, 200.0)
        try:
            pathplan.validate_cubic_profile(vs, vc, ve, move_d, cons)
        except pathplan.InfeasibleProfile:
            continue
        segs = pathplan.render_cubic_profile(vs, vc, ve, move_d, cons)
        check_invariants(segs, vs, ve, move_d)


def test_split_preserves_profile():
    rng = random.Random(99)
    cons = cons_for(55.0, 65.0)
    segs = pathplan.render_cubic_profile(0.0, 250.0, 0.0, 30.0, cons)
    for _ in range(200):
        n = rng.randint(1, 8)
        cuts = sorted(rng.uniform(0.0, 30.0) for _ in range(n - 1))
        edges = [0.0] + cuts + [30.0]
        lengths = [b - a for a, b in zip(edges, edges[1:])]
        lengths = [d for d in lengths if d > 1e-6]
        lengths[-1] += 30.0 - math.fsum(lengths)
        buckets = pathplan.split_cubic_segments(segs, lengths)
        flat = [s for b in buckets for s in b]
        for b, d in zip(buckets, lengths):
            assert abs(math.fsum(s[4] for s in b) - d) < 1e-9
        check_invariants(flat, 0.0, 0.0, 30.0)
        # Splitting must not change total time or the spectrum.
        assert (
            abs(math.fsum(s[0] for s in flat) - math.fsum(s[0] for s in segs))
            < 1e-12
        )
        for f in (55.0, 65.0):
            assert abs(seg_spectrum(flat, f) - seg_spectrum(segs, f)) < 1e-9


def test_segment_count():
    # The point of the exercise for a slow host: trapq traffic per ramp. An
    # unsaturated two-zero ramp is rise / hold / fall; a single notch has no
    # hold, so its triangle is two segments.
    assert len(pathplan._cubic_ramp_up(0.0, 200.0, cons_for(55.0, 65.0))) == 3
    assert len(pathplan._cubic_ramp_up(0.0, 200.0, cons_for(55.0))) == 2


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("ok", name)
