# Tests for the per-ramp notch jerk law (J = dv * f_n^2) in pathplan.
#
# A jerk-limited ramp whose acceleration never saturates is TRIANGULAR in a(t),
# so it acts as a shaper with a zero at f = 1/T_rise where T_rise = sqrt(dv/J).
# With a FIXED jerk that zero slides as sqrt(J/dv) and cannot cancel a mode at a
# fixed frequency. Setting J = dv*f_n^2 per ramp pins T_rise at 1/f_n.
#
# Verifies: the zero is parked (rise time constant across dv, total ramp = 2/f_n);
# a_peak scales linearly with dv; ramp distance is (v0+v1)/f_n; the LOOKAHEAD
# twin jerk_dist agrees with the emitter's integrated distance (else moves get
# planned into the sharp fallback); jerk_reach_v2 stays monotone; max_jerk acts
# as a ceiling; notch_freq unset is a no-op; the stepguard invariants hold; and
# a full chain plans with zero sharp fallbacks.
#
# Run: klippy-env/bin/python klippy/extras/test_pathplan_notch.py
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pathplan  # noqa: E402
from test_pathplan import check_segs  # noqa: E402

FN = 44.0          # mode frequency to park the zero on (Hz)
A_CONST = 30000.0  # high enough that a_peak = dv*f_n never saturates below
DVS = [50.0, 100.0, 150.0, 200.0, 300.0, 400.0]


def make_cons(notch=FN, max_jerk=None, a_const=A_CONST, jerk_dt=0.0002):
    return pathplan.Constraints(a_const=a_const, v_ceil=600.0,
                                max_jerk=max_jerk, jerk_dt=jerk_dt,
                                notch_freq=notch)


def ramp_stats(v0, v1, cons):
    slices, dist = pathplan._ramp_up_jerk(v0, v1, cons)
    assert slices is not None, ("ramp did not converge", v0, v1)
    dur = sum(s[0] for s in slices)
    a_peak = max(s[5] for s in slices)
    return dur, a_peak, dist


def zoh_accel_spectrum(slices, freq):
    if freq <= 0.0:
        return sum(s[5] * s[0] for s in slices)
    w = 2.0 * math.pi * freq
    re = im = t = 0.0
    for s in slices:
        dt = s[0]
        a = s[5]
        nt = t + dt
        re += a * (math.sin(w * nt) - math.sin(w * t)) / w
        im += a * (math.cos(w * nt) - math.cos(w * t)) / w
        t = nt
    return math.hypot(re, im)


def test_zero_is_parked():
    # The accel pulse is a triangle: rise 0->a_peak over T_j, fall over T_j.
    # The shaper zero sits at 1/T_j, so it is the RISE time that must equal
    # 1/f_n -- the total ramp is 2*T_j = 2/f_n. _ramp_up_jerk integrates the
    # total, so check that.
    cons = make_cons()
    want_total = 2.0 / FN
    for dv in DVS:
        dur, _, _ = ramp_stats(10.0, 10.0 + dv, cons)
        assert abs(dur - want_total) <= 0.06 * want_total, (
            "zero not parked", dv, dur, want_total)
    print("  ramp duration == 2/f_n across dv=%s OK" % (DVS,))


def test_apeak_linear_in_dv():
    cons = make_cons()
    for dv in DVS:
        _, a_peak, _ = ramp_stats(10.0, 10.0 + dv, cons)
        want = dv * FN
        assert abs(a_peak - want) <= 0.10 * want, (
            "a_peak not dv*f_n", dv, a_peak, want)
    print("  a_peak == dv*f_n OK")


def test_ramp_distance_law():
    cons = make_cons()
    for dv in DVS:
        v0, v1 = 10.0, 10.0 + dv
        _, _, dist = ramp_stats(v0, v1, cons)
        want = (v0 + v1) / FN
        assert abs(dist - want) <= 0.06 * want, (
            "distance not (v0+v1)/f_n", dv, dist, want)
    print("  ramp distance == (v0+v1)/f_n OK")


def test_lookahead_matches_emitter():
    # jerk_dist() is what the lookahead plans with; _ramp_up_jerk() is what the
    # emitter renders. The analytic model is conservative against the
    # zero-order-held slices: it may under-use available runway, but it must not
    # approve a distance shorter than the emitted slices need.
    cons = make_cons()
    for dv in DVS:
        v0, v1 = 10.0, 10.0 + dv
        _, _, dist = ramp_stats(v0, v1, cons)
        pred = pathplan.jerk_dist(v0, v1, A_CONST, None, notch_freq=FN)
        assert pred >= dist - 1e-9, ("lookahead not conservative", dv,
                                     pred, dist)
        assert abs(pred - dist) <= 0.06 * max(pred, 1e-9), (
            "lookahead/emitter gap too large", dv, pred, dist)
    print("  jerk_dist conservatively bounds integrated ramp OK")


def test_lookahead_conservative_across_dt_and_saturation():
    cases = []
    for a_const in (3000.0, 8000.0, A_CONST):
        for jerk_dt in (0.001, 0.0005, 0.0002):
            cases.append(make_cons(notch=55.0, a_const=a_const,
                                   jerk_dt=jerk_dt))
    for cons in cases:
        for v0 in (0.0, 10.0, 50.0, 150.0):
            for dv in (5.0, 20.0, 50.0, 100.0, 200.0):
                slices, dist = pathplan._ramp_up_jerk(v0, v0 + dv, cons)
                assert slices is not None
                pred = pathplan.jerk_dist(v0, v0 + dv, cons.a_const,
                                          cons.max_jerk,
                                          notch_freq=cons.notch_freq)
                assert pred >= dist - 1e-8, (
                    "lookahead underestimates emitted distance", cons.a_const,
                    cons.jerk_dt, v0, dv, pred, dist)
    print("  jerk_dist stays conservative across dt/saturation cases OK")


def test_reach_is_monotone_and_finite():
    prev = -1.0
    for d in (1.0, 2.0, 5.0, 10.0, 25.0, 60.0, 150.0):
        u = pathplan.jerk_reach_v2(0.0, d, A_CONST, None, 500.0, notch_freq=FN)
        assert u >= prev - 1e-6, ("reach not monotone", d, u, prev)
        assert math.isfinite(u), ("reach not finite", d, u)
        prev = u
    print("  jerk_reach_v2 monotone under the notch law OK")


def test_max_jerk_is_a_ceiling():
    # Clamping J DOWN may only lengthen the ramp, but it does not preserve the
    # target notch at f_n.
    free = make_cons(max_jerk=None)
    capped = make_cons(max_jerk=1.0e5)
    for dv in DVS:
        d_free, _, _ = ramp_stats(10.0, 10.0 + dv, free)
        d_cap, _, _ = ramp_stats(10.0, 10.0 + dv, capped)
        assert d_cap >= d_free - 1e-6, ("ceiling shortened the ramp",
                                        dv, d_cap, d_free)
    print("  max_jerk clamps down only OK")


def test_emitted_zoh_notch_response():
    cons = make_cons(jerk_dt=0.0002)
    slices, _ = pathplan._ramp_up_jerk(10.0, 110.0, cons)
    dc = zoh_accel_spectrum(slices, 0.0)
    at_notch = zoh_accel_spectrum(slices, FN) / dc
    below = zoh_accel_spectrum(slices, FN * 0.75) / dc
    above = zoh_accel_spectrum(slices, FN * 1.25) / dc
    assert at_notch < 0.01, ("actual emitted notch too shallow", at_notch)
    assert at_notch < below * 0.1, ("notch not below lower neighbor",
                                    at_notch, below)
    assert at_notch < above * 0.1, ("notch not below upper neighbor",
                                    at_notch, above)
    print("  emitted zero-order-hold spectrum has a near-zero at f_n OK")


def test_zoh_notch_error_scales_with_dt():
    vals = []
    for dt in (0.001, 0.0005, 0.0002):
        cons = make_cons(jerk_dt=dt)
        slices, _ = pathplan._ramp_up_jerk(10.0, 110.0, cons)
        vals.append(zoh_accel_spectrum(slices, FN)
                    / zoh_accel_spectrum(slices, 0.0))
    assert vals[0] > vals[1] > vals[2], vals
    print("  emitted notch residual shrinks with jerk_dt OK")


def test_notch_loss_reasons():
    clean = make_cons()
    assert pathplan.notch_loss_reasons(
        0.0, 100.0, 0.0, 20.0, clean) == ["discrete_zero_order_hold"]
    saturated = make_cons(notch=55.0, a_const=3000.0)
    assert "accel_saturated" in pathplan.notch_loss_reasons(
        0.0, 100.0, 0.0, 20.0, saturated)
    clamped = make_cons(notch=55.0, max_jerk=100000.0)
    assert "jerk_clamped" in pathplan.notch_loss_reasons(
        0.0, 100.0, 0.0, 20.0, clamped)
    short = make_cons()
    assert "insufficient_runway" in pathplan.notch_loss_reasons(
        0.0, 200.0, 0.0, 0.5, short)
    print("  notch loss diagnostics report emitted-profile limits OK")


def test_notch_off_is_noop():
    a = pathplan.Constraints(a_const=8000.0, v_ceil=400.0, max_jerk=1.0e5,
                             jerk_dt=0.001)
    b = pathplan.Constraints(a_const=8000.0, v_ceil=400.0, max_jerk=1.0e5,
                             jerk_dt=0.001, notch_freq=None)
    x = pathplan.emit_profile(0.0, 200.0, 50.0, 30.0, a)
    y = pathplan.emit_profile(0.0, 200.0, 50.0, 30.0, b)
    assert x == y, "notch_freq=None diverged from fixed jerk"
    print("  notch_freq=None identical to fixed jerk OK")


def test_invariants_hold():
    cons = make_cons()
    cases = [(0.0, 200.0, 0.0, 60.0),
             (0.0, 250.0, 120.0, 50.0),
             (80.0, 300.0, 80.0, 90.0),
             (10.0, 210.0, 10.0, 30.0),
             (150.0, 150.0, 150.0, 20.0)]
    for vs, vc, ve, d in cases:
        segs = pathplan.emit_profile(vs, vc, ve, d, cons)
        assert segs, ("empty", vs, vc, ve, d)
        check_segs(segs, d, vs, ve, "notch(%s,%s,%s,%s)" % (vs, vc, ve, d))
    print("  emit_profile invariants hold under the notch law OK")


def _notch_lookahead(move_d, v_cap, v_junction, accel, notch, v_ceil=600.0):
    # Mirror the toolhead lookahead with the notch-aware reach: backward decel
    # pass, forward accel pass, then the per-move jerk peak clamp.
    n = len(move_d)
    vb = [0.0] * (n + 1)
    for k in range(n + 1):
        cap = v_junction[k]
        if k > 0:
            cap = min(cap, v_cap[k - 1])
        if k < n:
            cap = min(cap, v_cap[k])
        vb[k] = cap
    for i in range(n - 1, -1, -1):
        r = pathplan.jerk_reach_v2(vb[i + 1] ** 2, move_d[i], accel, None,
                                   v_ceil, notch_freq=notch)
        vb[i] = min(vb[i], math.sqrt(r))
    for i in range(n):
        r = pathplan.jerk_reach_v2(vb[i] ** 2, move_d[i], accel, None,
                                   v_ceil, notch_freq=notch)
        vb[i + 1] = min(vb[i + 1], math.sqrt(r))
    vs = [vb[i] for i in range(n)]
    ve = [vb[i + 1] for i in range(n)]
    vc = []
    for i in range(n):
        pk = min(pathplan.jerk_reach_v2(vs[i] ** 2, move_d[i], accel, None,
                                        v_ceil, notch_freq=notch),
                 pathplan.jerk_reach_v2(ve[i] ** 2, move_d[i], accel, None,
                                        v_ceil, notch_freq=notch))
        vc.append(min(v_cap[i], math.sqrt(pk)))
    return vs, vc, ve


def test_chain_no_sharp_fallback():
    # Moves planned by the notch-aware lookahead must be renderable by the
    # notch-aware emitter WITHOUT the sharp fallback.
    cons = make_cons()
    n = 24
    move_d = [12.0] * n
    v_cap = [200.0] * n
    v_junction = [0.0] + [200.0] * (n - 1) + [0.0]
    vs, vc, ve = _notch_lookahead(move_d, v_cap, v_junction, A_CONST, FN)
    fell_back = 0
    for i in range(n):
        if pathplan._emit_jerk_core(vs[i], vc[i], ve[i], move_d[i],
                                    cons) is None:
            fell_back += 1
        segs = pathplan.emit_profile(vs[i], vc[i], ve[i], move_d[i], cons)
        check_segs(segs, move_d[i], vs[i], ve[i], "chain[%d]" % i)
    assert fell_back == 0, ("%d/%d moves fell back to sharp" % (fell_back, n))
    print("  24-move chain: peak cruise=%.1f mm/s, 0 sharp fallbacks OK"
          % max(vc))


def test_short_chain_degrades_cleanly():
    cons = make_cons()
    n = 12
    move_d = [1.5] * n
    v_cap = [200.0] * n
    v_junction = [0.0] + [200.0] * (n - 1) + [0.0]
    vs, vc, ve = _notch_lookahead(move_d, v_cap, v_junction, A_CONST, FN)
    for i in range(n):
        segs = pathplan.emit_profile(vs[i], vc[i], ve[i], move_d[i], cons)
        assert segs, ("empty on short chain", i)
        check_segs(segs, move_d[i], vs[i], ve[i], "short_chain[%d]" % i)
    print("  sub-runway chain emits valid profiles (degrades, no hang) OK")


def main():
    test_zero_is_parked()
    test_apeak_linear_in_dv()
    test_ramp_distance_law()
    test_lookahead_matches_emitter()
    test_lookahead_conservative_across_dt_and_saturation()
    test_reach_is_monotone_and_finite()
    test_max_jerk_is_a_ceiling()
    test_emitted_zoh_notch_response()
    test_zoh_notch_error_scales_with_dt()
    test_notch_loss_reasons()
    test_notch_off_is_noop()
    test_invariants_hold()
    test_chain_no_sharp_fallback()
    test_short_chain_degrades_cleanly()
    print("ALL PASS")


if __name__ == "__main__":
    main()
